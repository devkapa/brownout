//! The static partition solver (core/README.md, "W0 kernel semantics"),
//! moved from `engine.rs` (W1 plan section 3.2, row B1) and extended for
//! W1b step 1 with the switch family's fresh-every-resolve stamping and the
//! solution cache (row B1's second half).
//!
//! Re-solving a dirty partition means cloning its pristine per-partition
//! base (shunt + every immutable static-stamp device — today, only the
//! resistor), stamping every currently-committed driver on top of it
//! (unchanged from W1a) and every switch-family device's CURRENT contact
//! state on top of that (new here — a switch's `closed`/`sel`/`sw_i` can
//! change after load, `bo_engine_set_params`, so nothing about it can be
//! baked into the pristine base the way an immutable resistor's `R` is),
//! then a dense LU solve, exactly as W0/W1a's did. The solution cache (below)
//! sits in front of that LU solve, keyed on everything that can change what
//! the clone-and-stamp step produces without changing the partition's own
//! topology.
//!
//! Devices in a partition are represented two ways, matching whether their
//! electrical state can change after load:
//! - **Resistors** are folded into `matrix_base`/`rhs_base` once, at
//!   [`StaticPartitionSolver::build`] — `devices::passives::stamp` is called
//!   exactly once per resistor, exactly as before.
//! - **Switch-family devices** keep their own mutable, current `params`
//!   (mutated by [`PartitionSolver::apply_params`]) and are stamped fresh —
//!   `devices::switches::stamp` — on every [`PartitionSolver::resolve_dirty`]
//!   call that touches their partition, the same way a driver's committed
//!   level already is.

use std::collections::{BTreeSet, HashMap};

use crate::devices;
use crate::drivers::Driver;
use crate::error::BoError;
use crate::events::{DeviceId, Mode, PartitionId};
use crate::kinds;
use crate::lu;
use crate::partition::{self, ResolvedSource};
use crate::program::Program;
use crate::solve::PartitionSolver;

/// Every node except node 0 carries this conductance to the datum (contract,
/// "W0 kernel semantics", point 2): it keeps an isolated net solvable (reads
/// 0 V) and its current is folded into every KCL sum below, exactly as the
/// classic engine stamps it on every node row.
pub const NODE_RSHUNT_G: f64 = 1e-12;

/// One static-stamp device's electrical identity: which device it is, its
/// kind, its own pins (as node ids, in device-pin order) and its current
/// params. Used for BOTH the immutable resistor list (params never change
/// after `build`) and the mutable switch-family list (`params` is exactly
/// what [`PartitionSolver::apply_params`] mutates in place) — the same shape
/// either way, since `devices::stamp`/`devices::current` only ever need a
/// kind, a pin count and a params slice.
#[derive(Debug, Clone)]
struct StaticStamp {
    device_index: usize,
    kind: u32,
    pins: Vec<usize>,
    params: Vec<f64>,
}

/// The solution cache's key (W1 plan row B1): everything that can change
/// what a partition solves to without changing the partition's own topology.
/// `rail_band` and `floating_pulls` are reserved for W1f's hub rails and
/// W1c's floating model respectively — always their default today, so
/// giving them real values later is exactly the "extend the key" the plan
/// calls for, not a reshape of it.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct CacheKey {
    /// `(driver id, committed mode)` for every driver whose node lives in
    /// this partition, sorted by driver id for a stable comparison. Always
    /// empty through W1b step 1: no kind ported so far creates a `Driver`.
    driver_modes: Vec<(u32, u8)>,
    /// Reserved for W1f's hub-rail band (pec3's 50 mV rail band); always 0
    /// until then.
    rail_band: i64,
    /// Reserved for W1c's floating model; always empty until then.
    floating_pulls: Vec<bool>,
    /// Bumped by `apply_params` on every live-parameter change (owner
    /// decision 1) — the only thing that can change a switch-family
    /// device's stamp after load. A single, solver-wide counter (not
    /// per-partition): `resolve_dirty` only ever checks this key for a
    /// partition actually named in `dirty`, so a bump from an unrelated
    /// partition's toggle costs that OTHER partition nothing unless it is
    /// independently dirtied later anyway, in which case recomputing (rather
    /// than serving a stale hit) is exactly what must happen.
    epoch: u64,
}

fn mode_discriminant(m: Mode) -> u8 {
    match m {
        Mode::Hiz => 0,
        Mode::High => 1,
        Mode::Low => 2,
    }
}

/// Folds one `(u, w, g)` conductance into the pristine per-partition base —
/// exactly the resistor-folding logic W0 always had, generalized to take its
/// three numbers as arguments instead of being written inline once for the
/// resistor loop only. A `g` of exactly `0.0` is skipped outright (README:
/// "open stamps nothing" — nothing calls this with `g == 0` today, since
/// only the immutable resistor list ever reaches it, but the guard costs
/// nothing and keeps the function honest about its own contract).
fn fold_static_stamp(
    partitions: &[Vec<usize>],
    local_index: &[Option<(usize, usize)>],
    fixed: &[Option<f64>],
    matrix_base: &mut [Vec<f64>],
    rhs_base: &mut [Vec<f64>],
    u: usize,
    w: usize,
    g: f64,
) {
    if g == 0.0 {
        return;
    }
    match (local_index[u], local_index[w]) {
        (Some((pu, lu_)), Some((pw, lw))) => {
            debug_assert_eq!(pu, pw, "partition::build must have unioned both ends of any free-free stamp");
            let size = partitions[pu].len();
            let a = &mut matrix_base[pu];
            a[lu_ * size + lu_] += g;
            a[lw * size + lw] += g;
            a[lu_ * size + lw] -= g;
            a[lw * size + lu_] -= g;
        }
        (Some((pu, lu_)), None) => {
            let size = partitions[pu].len();
            matrix_base[pu][lu_ * size + lu_] += g;
            rhs_base[pu][lu_] += g * fixed[w].unwrap();
        }
        (None, Some((pw, lw))) => {
            let size = partitions[pw].len();
            matrix_base[pw][lw * size + lw] += g;
            rhs_base[pw][lw] += g * fixed[u].unwrap();
        }
        (None, None) => {
            // Both ends fixed: no stamp. Current is Ohm's law on two
            // already-known voltages, not a solve.
        }
    }
}

#[derive(Debug)]
pub struct StaticPartitionSolver {
    /// Node ids per partition, in first-seen order (deterministic; contract
    /// point 3 of "W0 kernel semantics").
    partitions: Vec<Vec<usize>>,
    /// node -> (partition index, local index), `None` for a fixed node.
    local_index: Vec<Option<(usize, usize)>>,
    /// node -> fixed voltage, `None` for a free node. Node 0 is always
    /// `Some(0.0)`.
    fixed: Vec<Option<f64>>,
    /// Every source that fixed a node, in resolution order.
    resolved: Vec<ResolvedSource>,
    /// Pristine per-partition conductance matrix: shunt + resistor stamps
    /// only, never a driver's or a switch's. Cloned fresh before every
    /// solve so a driver or switch that has since changed never leaves a
    /// stale stamp behind.
    matrix_base: Vec<Vec<f64>>,
    /// Pristine per-partition right-hand side: resistor-to-fixed-neighbour
    /// stamps only.
    rhs_base: Vec<Vec<f64>>,
    /// Every resistor (immutable after `build`), kept so `recompute_currents`
    /// can recompute its published current.
    resistors: Vec<StaticStamp>,
    /// Every switch-family device, in device order. `params` is mutated in
    /// place by `apply_params` — the only state in this whole struct a live
    /// `bo_engine_set_params` call ever changes.
    switches: Vec<StaticStamp>,
    /// device index -> index into `switches`, for O(1) lookup during
    /// `validate_params`/`apply_params`/`param_value`.
    switch_index_by_device: HashMap<usize, usize>,
    device_count: usize,
    voltages: Vec<f64>,
    currents: Vec<f64>,
    /// `NO_SOLUTION_CACHE` (option key 2): bypasses the cache below when
    /// `true`, both for reading a hit and for writing a fresh solve back
    /// into it — so cache on and cache off run the identical LU-solve code
    /// path whenever they actually compute anything, which is what makes
    /// the two bit-identical (core/README.md's Options table; W1 plan row
    /// B1's verify step).
    no_solution_cache: bool,
    /// Bumped by `apply_params` on every live-parameter change.
    epoch: u64,
    /// partition index -> (the key it was last solved under, its solved
    /// local voltages).
    cache: HashMap<usize, (CacheKey, Vec<f64>)>,
    /// Contract stat 7.
    cache_hits: f64,
}

impl StaticPartitionSolver {
    /// Builds the static topology (`partition::build`) and the pristine
    /// per-partition matrix/RHS, but does not solve anything yet — that is
    /// `Kernel::settle`'s job, through `resolve_dirty`, the same way any
    /// later re-solve works.
    pub fn build(program: &Program) -> Result<Self, BoError> {
        let pb = partition::build(program)?;
        let partition::PartitionBuild { partitions, local_index, fixed, resolved, .. } = pb;

        let mut matrix_base: Vec<Vec<f64>> = partitions.iter().map(|p| vec![0.0; p.len() * p.len()]).collect();
        let mut rhs_base: Vec<Vec<f64>> = partitions.iter().map(|p| vec![0.0; p.len()]).collect();

        for (p_idx, nodes) in partitions.iter().enumerate() {
            let size = nodes.len();
            for l_idx in 0..size {
                matrix_base[p_idx][l_idx * size + l_idx] += NODE_RSHUNT_G;
            }
        }

        let mut resistors = Vec::new();
        let mut switches = Vec::new();
        let mut switch_index_by_device = HashMap::new();

        for (d, device) in program.devices.iter().enumerate() {
            match device.kind {
                kinds::RESISTOR => {
                    let pins: Vec<usize> = device.pins.iter().map(|p| p.node).collect();
                    for c in devices::stamp(device.kind, pins.len(), &device.params) {
                        fold_static_stamp(&partitions, &local_index, &fixed, &mut matrix_base, &mut rhs_base, pins[c.pin_a], pins[c.pin_b], c.conductance);
                    }
                    resistors.push(StaticStamp { device_index: d, kind: device.kind, pins, params: device.params.clone() });
                }
                kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH => {
                    let pins: Vec<usize> = device.pins.iter().map(|p| p.node).collect();
                    switch_index_by_device.insert(d, switches.len());
                    switches.push(StaticStamp { device_index: d, kind: device.kind, pins, params: device.params.clone() });
                }
                _ => {}
            }
        }

        let mut voltages = vec![0.0; program.node_count];
        for (node, v) in fixed.iter().enumerate() {
            if let Some(v) = v {
                voltages[node] = *v;
            }
        }

        Ok(StaticPartitionSolver {
            partitions,
            local_index,
            fixed,
            resolved,
            matrix_base,
            rhs_base,
            resistors,
            switches,
            switch_index_by_device,
            device_count: program.devices.len(),
            voltages,
            currents: vec![0.0; program.devices.len()],
            no_solution_cache: program.options.no_solution_cache,
            epoch: 0,
            cache: HashMap::new(),
            cache_hits: 0.0,
        })
    }

    /// The number of partitions built (contract stat 0), for `Engine::load`
    /// to snapshot at load time.
    pub fn partition_count(&self) -> usize {
        self.partitions.len()
    }

    /// The largest partition's node count (contract stat 2).
    pub fn largest_partition(&self) -> usize {
        self.partitions.iter().map(|p| p.len()).max().unwrap_or(0)
    }

    /// How many nodes fixed-node propagation fixed, node 0 included
    /// (contract stat 3).
    pub fn fixed_count(&self) -> usize {
        self.fixed.iter().filter(|v| v.is_some()).count()
    }

    fn cache_key(&self, p_idx: usize, drivers: &[Driver]) -> CacheKey {
        let mut driver_modes: Vec<(u32, u8)> = drivers
            .iter()
            .enumerate()
            .filter_map(|(id, d)| {
                let (dp, _) = self.local_index.get(d.node).copied().flatten()?;
                if dp != p_idx {
                    return None;
                }
                let mode = d.committed()?;
                Some((id as u32, mode_discriminant(mode)))
            })
            .collect();
        driver_modes.sort_unstable();
        CacheKey { driver_modes, rail_band: 0, floating_pulls: Vec::new(), epoch: self.epoch }
    }

    /// Resistor and switch currents by summing every contact's own
    /// conductance times its own terminal voltage difference (folding the
    /// same total into `leaving` for the KCL walk below), then source
    /// currents by the children-before-parents walk (contract, point 5),
    /// from `self.voltages` as it currently stands.
    fn recompute_currents(&self) -> Vec<f64> {
        let mut currents = vec![0.0; self.device_count];
        let mut leaving = vec![0.0; self.voltages.len()];

        for stamp in self.resistors.iter().chain(self.switches.iter()) {
            let mut total = 0.0;
            for c in devices::stamp(stamp.kind, stamp.pins.len(), &stamp.params) {
                let (u, w) = (stamp.pins[c.pin_a], stamp.pins[c.pin_b]);
                let i = c.conductance * (self.voltages[u] - self.voltages[w]);
                total += i;
                leaving[u] += i;
                leaving[w] -= i;
            }
            currents[stamp.device_index] = total;
        }

        // Shunt current, every node but the datum (contract, point 2).
        for (node, v) in self.voltages.iter().enumerate().skip(1) {
            leaving[node] += NODE_RSHUNT_G * v;
        }

        let mut extra = vec![0.0; self.voltages.len()];
        for r in self.resolved.iter().rev() {
            let total = leaving[r.child_node] + extra[r.child_node];
            let child_is_pos = !r.parent_is_pos;
            let i_s = if child_is_pos { -total } else { total };
            currents[r.device_index] = i_s;

            let contribution = if r.parent_is_pos { i_s } else { -i_s };
            extra[r.parent_node] += contribution;
        }

        currents
    }
}

impl PartitionSolver for StaticPartitionSolver {
    fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
        for &partition in dirty {
            let p_idx = partition.0 as usize;
            let nodes = &self.partitions[p_idx];
            let size = nodes.len();
            if size == 0 {
                continue;
            }

            let key = self.cache_key(p_idx, drivers);
            if !self.no_solution_cache {
                if let Some((cached_key, cached_x)) = self.cache.get(&p_idx) {
                    if cached_key == &key {
                        self.cache_hits += 1.0;
                        for (l_idx, &node) in nodes.iter().enumerate() {
                            self.voltages[node] = cached_x[l_idx];
                        }
                        continue;
                    }
                }
            }

            let mut a = self.matrix_base[p_idx].clone();
            let mut b = self.rhs_base[p_idx].clone();

            // Stamp every driver currently committed to HIGH or LOW whose
            // output node lives in this partition (contract point 2, spec
            // section 1.7's rail-referenced output stage).
            for driver in drivers {
                let mode = match driver.committed() {
                    Some(m) if m != Mode::Hiz => m,
                    _ => continue,
                };
                let (driver_partition, l) = match self.local_index.get(driver.node).copied().flatten() {
                    Some(loc) => loc,
                    None => continue, // the driver's own node is fixed: nothing to stamp into any partition.
                };
                if driver_partition != p_idx {
                    continue;
                }
                let f = match mode {
                    Mode::High => driver.hi_level,
                    Mode::Low => driver.lo_level,
                    Mode::Hiz => unreachable!("filtered above"),
                };
                let hi_v = self.fixed.get(driver.hi_node).copied().flatten();
                let lo_v = self.fixed.get(driver.lo_node).copied().flatten();
                let (hi_v, lo_v) = match (hi_v, lo_v) {
                    (Some(hv), Some(lv)) => (hv, lv),
                    _ => {
                        return Err(format!(
                            "driver at node {}: a free hi/lo rail node is not supported by the static solver (hub rails are W1f)",
                            driver.node
                        ));
                    }
                };
                if f > 1e-12 {
                    let gg = driver.conductance * f;
                    a[l * size + l] += gg;
                    b[l] += gg * hi_v;
                }
                if f < 1.0 - 1e-12 {
                    let gg = driver.conductance * (1.0 - f);
                    a[l * size + l] += gg;
                    b[l] += gg * lo_v;
                }
            }

            // Stamp every switch-family device's CURRENT contact state
            // (fresh every resolve, unlike the resistor's immutable base —
            // see this module's doc comment). A switch's own free pins are
            // always entirely within one partition (`partition::build`
            // unions every contact unconditionally), so a contact whose
            // free end lands outside `p_idx` belongs to a different
            // partition entirely and is simply skipped here.
            for sw in &self.switches {
                for c in devices::stamp(sw.kind, sw.pins.len(), &sw.params) {
                    if c.conductance == 0.0 {
                        continue; // README: "open stamps nothing".
                    }
                    let (u, w) = (sw.pins[c.pin_a], sw.pins[c.pin_b]);
                    match (self.local_index.get(u).copied().flatten(), self.local_index.get(w).copied().flatten()) {
                        (Some((pu, lu_)), Some((pw, lw))) if pu == p_idx => {
                            debug_assert_eq!(pu, pw);
                            a[lu_ * size + lu_] += c.conductance;
                            a[lw * size + lw] += c.conductance;
                            a[lu_ * size + lw] -= c.conductance;
                            a[lw * size + lu_] -= c.conductance;
                        }
                        (Some((pu, lu_)), None) if pu == p_idx => {
                            a[lu_ * size + lu_] += c.conductance;
                            b[lu_] += c.conductance * self.fixed[w].unwrap();
                        }
                        (None, Some((pw, lw))) if pw == p_idx => {
                            a[lw * size + lw] += c.conductance;
                            b[lw] += c.conductance * self.fixed[u].unwrap();
                        }
                        _ => {} // both fixed, or this contact belongs to a different partition.
                    }
                }
            }

            let x = lu::solve(a, size, b).map_err(|_| format!("partition {p_idx} ({size} node(s)): singular or non-finite pivot"))?;
            for (l_idx, &node) in nodes.iter().enumerate() {
                self.voltages[node] = x[l_idx];
            }
            if !self.no_solution_cache {
                self.cache.insert(p_idx, (key, x));
            }
        }

        // Recomputed unconditionally (even with an empty `dirty`): a
        // device's current can depend purely on already-fixed voltages,
        // which no partition solve ever touches, so this is the only place
        // that ever (re)publishes it.
        self.currents = self.recompute_currents();

        if let Some((node, v)) = self.voltages.iter().enumerate().find(|(_, v)| !v.is_finite()) {
            return Err(format!("node {node}: published voltage {v} is not finite"));
        }
        if let Some((device, i)) = self.currents.iter().enumerate().find(|(_, i)| !i.is_finite()) {
            return Err(format!("device {device}: published current {i} is not finite"));
        }

        // No reader concept exists for any kind ported so far (W1c+).
        Ok(Vec::new())
    }

    fn voltages(&self) -> &[f64] {
        &self.voltages
    }

    fn currents(&self) -> &[f64] {
        &self.currents
    }

    fn restore(&mut self, snapshot: (Vec<f64>, Vec<f64>)) {
        self.voltages = snapshot.0;
        self.currents = snapshot.1;
    }

    fn set_no_solution_cache(&mut self, off: bool) {
        self.no_solution_cache = off;
    }

    fn cache_hits(&self) -> f64 {
        self.cache_hits
    }

    /// Owner decision 1's "all or nothing": every triple is checked, in
    /// order, against the device it names — out of range, not live, or
    /// failing the kind's own `check` are each an error naming the first
    /// offending triple — before `apply_params` ever touches anything.
    fn validate_params(&self, triples: &[(usize, usize, f64)]) -> Result<(), String> {
        for &(device_index, param_index, value) in triples {
            if !value.is_finite() {
                return Err(format!("device {device_index} param {param_index}: value {value} is not finite"));
            }
            let sw = match self.switch_index_by_device.get(&device_index) {
                Some(&idx) => &self.switches[idx],
                None => return Err(format!("device {device_index}: has no live parameters")),
            };
            let shape = kinds::shape(sw.kind).expect("a stored switch device always has a known kind code");
            if !shape.live_params.contains(&param_index) {
                return Err(format!("device {device_index} ({}): param {param_index} is not live", shape.name));
            }
            if param_index >= sw.params.len() {
                return Err(format!("device {device_index} ({}): param index {param_index} out of range", shape.name));
            }
            let mut new_params = sw.params.clone();
            new_params[param_index] = value;
            (shape.check)(sw.pins.len(), &new_params).map_err(|msg| format!("device {device_index} ({}): {msg}", shape.name))?;
        }
        Ok(())
    }

    /// Applies an already-`validate_params`-checked batch: mutates every
    /// named device's param, bumps the epoch once (not once per triple —
    /// one `bo_engine_set_params` call is ONE event, owner decision 1) IF
    /// AND ONLY IF at least one triple's value actually differs from what
    /// was already stored, and returns every partition touched,
    /// deduplicated, for the caller (`kernel.rs`) to re-solve as that one
    /// event regardless.
    ///
    /// The "only on an actual change" condition is deliberately narrower
    /// than "every call bumps it": every touched partition is still marked
    /// dirty and re-solved unconditionally (a `bo_engine_set_params` call
    /// always runs its one event, per the contract), but a call that sets a
    /// switch to the value it already holds leaves the electrical system
    /// identical to what the cache already has recorded for it, so nothing
    /// needs to be invalidated. Two toggles back to back that happen to
    /// repeat the same end state (as a harness stimulus replaying a fixed
    /// sequence might) can then still hit the cache on the second one,
    /// instead of every `bo_engine_set_params` call unconditionally forcing
    /// a fresh solve.
    fn apply_params(&mut self, triples: &[(usize, usize, f64)]) -> Vec<PartitionId> {
        let mut dirty = BTreeSet::new();
        let mut changed = false;
        for &(device_index, param_index, value) in triples {
            if let Some(&idx) = self.switch_index_by_device.get(&device_index) {
                if self.switches[idx].params[param_index] != value {
                    changed = true;
                }
                self.switches[idx].params[param_index] = value;
                for &pin_node in &self.switches[idx].pins {
                    if let Some((p_idx, _)) = self.local_index[pin_node] {
                        dirty.insert(PartitionId(p_idx as u32));
                    }
                }
            }
        }
        if changed {
            self.epoch += 1;
        }
        dirty.into_iter().collect()
    }

    fn param_value(&self, device_index: usize, param_index: usize) -> Option<f64> {
        let &idx = self.switch_index_by_device.get(&device_index)?;
        self.switches[idx].params.get(param_index).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::program::{Device, Options, PinRecord};

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn source(component_index: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: kinds::VOLTAGE_SOURCE, component_index, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }

    fn resistor(component_index: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: kinds::RESISTOR, component_index, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }

    fn switch_dev(component_index: usize, a: usize, b: usize, closed: f64) -> Device {
        Device { kind: kinds::SWITCH, component_index, pins: vec![pin(a), pin(b)], params: vec![closed], blob: vec![] }
    }

    fn spdt(component_index: usize, com: usize, b: usize, c: usize, sel: f64) -> Device {
        Device { kind: kinds::SPDT_SWITCH, component_index, pins: vec![pin(com), pin(b), pin(c)], params: vec![sel], blob: vec![] }
    }

    fn dip(component_index: usize, pins: Vec<usize>, k: usize, sw: [f64; 8]) -> Device {
        let mut params = vec![k as f64];
        params.extend(sw);
        Device { kind: kinds::DIP_SWITCH, component_index, pins: pins.into_iter().map(pin).collect(), params, blob: vec![] }
    }

    fn program(node_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    }

    fn all_partitions(solver: &StaticPartitionSolver) -> Vec<PartitionId> {
        (0..solver.partitions.len() as u32).map(PartitionId).collect()
    }

    /// A closed switch in series with a resistor, across a source: exactly a
    /// two-resistor divider with the switch's 1 mΩ standing in for the
    /// second leg.
    #[test]
    fn a_closed_switch_in_series_with_a_resistor_divides_like_a_tiny_resistor() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 999.999), switch_dev(2, 2, 0, 1.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        // R_switch = 0.001 Ω, R_total ~= 1000 Ω, so node2 ~= 10 * 0.001/1000
        // -- close enough to 0 that this mostly proves it solved at all
        // without a singular matrix, and that the switch's contact really
        // did stamp a conductance (an unstamped/open contact would instead
        // read the source's full 10V at node2, since node2 would then have
        // no path to the datum but the 1e-12 S shunt).
        assert!(solver.voltages()[2].abs() < 0.001, "got {}", solver.voltages()[2]);
        assert!((solver.currents()[2] - solver.currents()[1]).abs() < 1e-9, "series current must match through the switch");
    }

    #[test]
    fn an_open_switch_stamps_nothing_and_the_node_reads_near_the_source_through_the_shunt() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 2, 0, 0.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        // No conductance from node2 to the datum except the 1e-12 S shunt in
        // series with the 1kΩ resistor from the 10V rail: node2 reads
        // essentially the full 10V (the shunt current is negligible).
        assert!((solver.voltages()[2] - 10.0).abs() < 1e-6, "got {}", solver.voltages()[2]);
        assert_eq!(solver.currents()[2], 0.0, "an open switch publishes zero current");
    }

    /// The README's "Pins by id" rule ("a pin id the component lacks maps to
    /// node 0") is applied by the TS compiler before Rust ever sees the
    /// program, but Rust must still handle the result correctly: an
    /// spdt_switch whose `c` pin was absent on the component lands here with
    /// `pins[2].node == 0` (the datum). Selecting that throw must solve
    /// against the datum exactly like any other fixed node, not panic or
    /// treat node 0 specially.
    #[test]
    fn spdt_with_a_missing_pin_mapped_to_the_datum_solves_against_it() {
        // com=2 (free, off the 10V rail through a 1kΩ resistor), b=3 (a
        // free, otherwise-unconnected node), c=0 (the datum -- what a pin id
        // the component lacks resolves to). sel=1 selects c: com is shorted
        // to the datum through the switch's 1 mΩ contact.
        let p = program(4, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), spdt(2, 2, 3, 0, 1.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        // com (node2) is shorted to the datum through 1 mΩ, dominating the
        // 1kΩ feed from the rail: it must read ~0V, not float toward 10V the
        // way an unstamped/open contact would.
        assert!(solver.voltages()[2].abs() < 1e-3, "got {}", solver.voltages()[2]);
        // b's node (3) has no other connection at all: it reads ~0V through
        // the shunt alone, confirming the UNselected contact really did
        // stamp nothing.
        assert!(solver.voltages()[3].abs() < 1e-6, "got {}", solver.voltages()[3]);
        // Essentially the whole loop is 10V over (1kΩ + 1mΩ): ~10mA.
        let i = solver.currents()[2];
        assert!((i - 0.01).abs() < 1e-4, "got {i}");
    }

    #[test]
    fn dip_switch_k_2_solves_two_independent_contacts_through_a_real_program() {
        // Two independent branches off the same 10V rail, each through a
        // 1kΩ resistor into one dip contact straight to the datum: contact 1
        // closed, contact 2 open.
        let p = program(4, vec![
            source(0, 1, 0, 10.0),
            resistor(1, 1, 2, 1000.0),
            resistor(2, 1, 3, 1000.0),
            dip(3, vec![2, 0, 3, 0], 2, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ]);
        // dip pins: a1=2, b1=0 (datum), a2=3, b2=0 (datum).
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        // Contact 1 closed: node2 -> datum through 1kΩ + 1mΩ, so node2
        // reads ~0V (dominated by the 1kΩ leg to the rail).
        assert!(solver.voltages()[2].abs() < 0.01, "got {}", solver.voltages()[2]);
        // Contact 2 open: node3 has no path to the datum except the shunt,
        // so it reads essentially the full 10V through its 1kΩ resistor.
        assert!((solver.voltages()[3] - 10.0).abs() < 1e-6, "got {}", solver.voltages()[3]);
        // Device 3 is the dip switch: its published current is the sum over
        // closed positions only -- contact 1's ~10mA, plus contact 2's 0.
        assert!((solver.currents()[3] - 0.01).abs() < 1e-4, "got {}", solver.currents()[3]);
    }

    #[test]
    fn dip_switch_k_8_solves_eight_independent_contacts_through_a_real_program() {
        let mut nodes = Vec::new();
        let mut devices = vec![source(0, 1, 0, 10.0)];
        let mut next_node = 2usize;
        let mut dip_pins = Vec::new();
        for i in 0..8 {
            let a = next_node;
            next_node += 1;
            devices.push(resistor(1 + i, 1, a, 1000.0));
            dip_pins.push(a);
            dip_pins.push(0); // b_i tied straight to the datum.
            nodes.push(a);
        }
        // All 8 contacts closed.
        devices.push(dip(9, dip_pins, 8, [1.0; 8]));
        let node_count = next_node;
        let p = program(node_count, devices);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        for &a in &nodes {
            assert!(solver.voltages()[a].abs() < 0.01, "node {a} got {}", solver.voltages()[a]);
        }
        // Sum over all 8 (identical) branches.
        let expected_total = 8.0 * (10.0 / 1000.0);
        assert!((solver.currents()[9] - expected_total).abs() < 1e-3, "got {}", solver.currents()[9]);
    }

    // ---- the solution cache (plan row B1) --------------------------------

    #[test]
    fn a_repeated_resolve_with_nothing_changed_is_a_cache_hit_and_bit_identical() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 2, 0, 1.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        let first = solver.voltages().to_vec();
        assert_eq!(solver.cache_hits(), 0.0);

        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        assert_eq!(solver.cache_hits(), 1.0, "an unchanged partition must hit the cache");
        assert_eq!(solver.voltages(), first.as_slice(), "a cache hit must be bit-identical to the fresh solve");
    }

    #[test]
    fn no_solution_cache_bypasses_the_cache_but_matches_it_bit_for_bit() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 2, 0, 1.0)]);
        let parts;
        let with_cache = {
            let mut solver = StaticPartitionSolver::build(&p).unwrap();
            parts = all_partitions(&solver);
            solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
            solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
            assert_eq!(solver.cache_hits(), 1.0);
            solver.voltages().to_vec()
        };
        let without_cache = {
            let mut solver = StaticPartitionSolver::build(&p).unwrap();
            solver.set_no_solution_cache(true);
            solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
            solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
            assert_eq!(solver.cache_hits(), 0.0, "NO_SOLUTION_CACHE must never record a hit");
            solver.voltages().to_vec()
        };
        assert_eq!(with_cache, without_cache, "cache on and cache off must give bit-identical results");
    }

    #[test]
    fn toggling_a_switch_busts_the_cache_and_the_new_value_is_not_reused() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 2, 0, 0.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        let open_voltage = solver.voltages()[2];

        let dirty = solver.apply_params(&[(2, 0, 1.0)]);
        assert_eq!(dirty, vec![PartitionId(0)]);
        solver.resolve_dirty(&dirty, 0.0, &[]).unwrap();
        let closed_voltage = solver.voltages()[2];

        assert_ne!(open_voltage, closed_voltage);
        assert_eq!(solver.cache_hits(), 0.0, "the epoch bump must prevent a stale hit");
    }

    // ---- set_params validation and application ---------------------------

    #[test]
    fn validate_params_rejects_an_out_of_range_device_index() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        assert!(solver.validate_params(&[(99, 0, 1.0)]).is_err());
    }

    #[test]
    fn validate_params_rejects_a_non_live_device_like_a_resistor() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        // Device 1 is the resistor: it has no live params at all.
        assert!(solver.validate_params(&[(1, 0, 500.0)]).is_err());
    }

    #[test]
    fn validate_params_rejects_a_param_index_not_in_live_params() {
        let p = program(4, vec![source(0, 1, 0, 10.0), dip(1, vec![1, 2, 3, 0], 2, [0.0; 8])]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        // Param 0 is `k`, not live for dip_switch.
        assert!(solver.validate_params(&[(1, 0, 8.0)]).is_err());
    }

    #[test]
    fn validate_params_rejects_a_value_the_kinds_check_function_rejects() {
        let p = program(3, vec![source(0, 1, 0, 10.0), switch_dev(1, 1, 2, 0.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        assert!(solver.validate_params(&[(1, 0, 2.0)]).is_err(), "switch state must be 0 or 1");
    }

    #[test]
    fn validate_params_is_all_or_nothing_one_bad_triple_fails_the_whole_batch() {
        let p = program(3, vec![source(0, 1, 0, 10.0), switch_dev(1, 1, 2, 0.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        // First triple valid, second invalid: the batch as a whole is Err,
        // and (per the caller's contract) nothing gets applied.
        let err = solver.validate_params(&[(1, 0, 1.0), (1, 0, 2.0)]);
        assert!(err.is_err());
    }

    #[test]
    fn apply_params_mutates_bumps_the_epoch_once_and_returns_touched_partitions() {
        let p = program(3, vec![source(0, 1, 0, 10.0), switch_dev(1, 1, 2, 0.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        assert_eq!(solver.param_value(1, 0), Some(0.0));
        let dirty = solver.apply_params(&[(1, 0, 1.0)]);
        assert_eq!(solver.param_value(1, 0), Some(1.0));
        assert_eq!(dirty, vec![PartitionId(0)]);
        assert_eq!(solver.epoch, 1);
    }

    #[test]
    fn param_value_is_none_for_a_device_with_no_live_parameters() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        assert_eq!(solver.param_value(1, 0), None);
        assert_eq!(solver.param_value(99, 0), None);
    }
}
