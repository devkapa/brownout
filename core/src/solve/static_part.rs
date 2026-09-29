//! The static partition solver (core/README.md, "W0 kernel semantics"),
//! moved from `engine.rs` (W1 plan section 3.2, row B1) and extended for
//! W1b step 1 with the switch family's fresh-every-resolve stamping and the
//! solution cache (row B1's second half).
//!
//! W1b step 2 (rows B2/B3) adds `ClosedForm`/`Stepped` partitions
//! (`solve/closed_form.rs`, `solve/stepped.rs`), owned by `solve::mod`'s
//! `PartitionedSolver`, not by this module. [`StaticPartitionSolver::build`]
//! now skips any resistor or switch-family device whose free pin(s) land in
//! a non-`Static` partition (`partition::PartitionKind`) — such a device is
//! entirely a dynamic partition's own resistive-network stamp instead (a
//! resistor in series with a capacitor, say), never split between the two —
//! and never allocates a matrix stamp for a non-`Static` partition's own
//! shunt row. `PartitionedSolver` only ever calls
//! [`PartitionSolver::resolve_dirty`] here with `Static`-classified
//! partition ids, so this module needs no defensive check of its own against
//! being handed one it does not own.
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
use crate::digital;
use crate::drivers::Driver;
use crate::error::BoError;
use crate::events::{DeviceId, Mode, PartitionId};
use crate::kinds;
use crate::lu;
use crate::partition::{self, PartitionKind, ResolvedSource};
use crate::program::Program;
use crate::solve::{collect_ics, ic_driver_target, ic_pull_target, PartitionSolver};

/// The fixed-point iteration's cap and Newton-style per-node tolerance (W1c
/// contract point 3: "classic's Newton test", `|dx| <= 1e-6 + 1e-3 |x|`) —
/// shared by every partition that holds a free IC supply node, since R_out
/// then moves with the very voltages the solve is finding.
const IC_FIXED_POINT_MAX_ITERATIONS: usize = 50;
const IC_FIXED_POINT_ABS_TOL: f64 = 1e-6;
const IC_FIXED_POINT_REL_TOL: f64 = 1e-3;

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
/// `rail_band` is reserved for W1f's hub rails — always its default today.
/// `floating_pulls` is W1c's: every would-be-pulled pin in this partition's
/// CURRENT `high` bit (this partition never reaches the cache at all when it
/// holds a free IC supply node — `Self::resolve_dirty`'s own branch —
/// so an IC touching this partition always has BOTH its supply nodes fixed,
/// meaning `digital::power::ic_power`'s result is a build-time constant
/// that does not need its own key entry; only the floating model's
/// bucket-dependent `high` bit can still move the stamp from one resolve to
/// the next).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct CacheKey {
    /// `(driver id, committed mode)` for every driver whose node lives in
    /// this partition, sorted by driver id for a stable comparison —
    /// generic over what kind of driver it is (an IC output included: its
    /// `device`/`open_collector` are fixed identity, not part of what
    /// varies resolve to resolve, so the committed mode alone is enough to
    /// key on, exactly like any other driver).
    driver_modes: Vec<(u32, u8)>,
    /// Reserved for W1f's hub-rail band (pec3's 50 mV rail band); always 0
    /// until then.
    rail_band: i64,
    /// One `high` bit per would-be-pulled pin touching this partition (an
    /// `OPEN` `FLOATING_INPUT`, non-`OPEN_HIGH` logic pin of an IC whose
    /// supply is fixed), in a fixed enumeration order (device order, then
    /// logic-pin order) — `Self::cache_key`'s own doc comment on why power
    /// itself never needs to be in this key. Included regardless of whether
    /// the owning IC is CURRENTLY powered (a cheap, always-safe
    /// over-inclusion: an unpowered IC's pull never stamps either way, so a
    /// spurious cache miss from its hash flipping costs nothing but a
    /// redundant, correct resolve).
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
    /// Every combinational IC whose driver/pull pins reach a `Static`
    /// partition (`partition.rs`'s own `-3` decline guarantees every IC's
    /// driver/pull footprint is entirely `Static`-owned this step), by
    /// device index — `solve::collect_ics` over the WHOLE program, not
    /// filtered here, since `Self::stamp_ics_into`'s own per-driver/pull
    /// `in_partition` guard already skips anything that does not touch the
    /// partition being resolved.
    ics: Vec<(usize, digital::IcRecord)>,
    /// partition index -> whether it holds at least one free `vcc`/`gnd`
    /// node of some IC (W1c contract point 3): such a partition's R_out
    /// moves with its own solved state, so `Self::resolve_dirty` iterates it
    /// to a fixed point instead of solving it once and reading (or writing)
    /// the solution cache — see `Self::iterate_solve`.
    holds_free_ic_supply: Vec<bool>,
    /// node -> every IC device index that reads it (its `vcc`/`gnd` pins and
    /// every logic pin, W1c contract point 4's reader rule) — built once so
    /// `Self::resolve_dirty` can turn "this node's published bits changed"
    /// into "these readers must be queued" by a single lookup per node.
    node_to_ic_readers: HashMap<usize, Vec<usize>>,
}

impl StaticPartitionSolver {
    /// Builds the static topology (`partition::build`) and the pristine
    /// per-partition matrix/RHS, but does not solve anything yet — that is
    /// `Kernel::settle`'s job, through `resolve_dirty`, the same way any
    /// later re-solve works.
    pub fn build(program: &Program) -> Result<Self, BoError> {
        let pb = partition::build(program)?;
        let partition::PartitionBuild { partitions, local_index, fixed, resolved, kinds: partition_kinds, .. } = pb;

        let mut matrix_base: Vec<Vec<f64>> = partitions.iter().map(|p| vec![0.0; p.len() * p.len()]).collect();
        let mut rhs_base: Vec<Vec<f64>> = partitions.iter().map(|p| vec![0.0; p.len()]).collect();

        // Only a `Static` partition's own shunt row is ever meant to be
        // solved here — a non-`Static` partition's matrix stays all-zero
        // (never factored, since `PartitionedSolver` never names it in
        // `resolve_dirty`'s `dirty`), but is still allocated above so every
        // partition id in this vector lines up with `partition::build`'s
        // global numbering (`solve::mod`'s `PartitionedSolver` depends on
        // that alignment to route a dirty id to the right sub-solver).
        for (p_idx, nodes) in partitions.iter().enumerate() {
            if partition_kinds[p_idx] != PartitionKind::Static {
                continue;
            }
            let size = nodes.len();
            for l_idx in 0..size {
                matrix_base[p_idx][l_idx * size + l_idx] += NODE_RSHUNT_G;
            }
        }

        // Whether `device`'s own free pin(s) land in a `Static` partition —
        // `None` (no free pin at all, both terminals fixed) is also this
        // solver's to own, exactly as before W1b step 2. A device's free
        // pins are always in the SAME partition (`partition::build`'s
        // union-find), so the first free pin found settles it.
        let owned_by_static = |device: &crate::program::Device| -> bool {
            device
                .pins
                .iter()
                .find_map(|p| local_index[p.node].map(|(p_idx, _)| partition_kinds[p_idx]))
                .is_none_or(|k| k == PartitionKind::Static)
        };

        let mut resistors = Vec::new();
        let mut switches = Vec::new();
        let mut switch_index_by_device = HashMap::new();

        for (d, device) in program.devices.iter().enumerate() {
            match device.kind {
                kinds::RESISTOR if owned_by_static(device) => {
                    let pins: Vec<usize> = device.pins.iter().map(|p| p.node).collect();
                    for c in devices::stamp(device.kind, pins.len(), &device.params) {
                        fold_static_stamp(&partitions, &local_index, &fixed, &mut matrix_base, &mut rhs_base, pins[c.pin_a], pins[c.pin_b], c.conductance);
                    }
                    resistors.push(StaticStamp { device_index: d, kind: device.kind, pins, params: device.params.clone() });
                }
                kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH if owned_by_static(device) => {
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

        let ics = collect_ics(program);
        let mut holds_free_ic_supply = vec![false; partitions.len()];
        let mut node_to_ic_readers: HashMap<usize, Vec<usize>> = HashMap::new();
        for (device_index, ic) in &ics {
            for &node in &[ic.vcc.node, ic.gnd.node] {
                node_to_ic_readers.entry(node).or_default().push(*device_index);
                if let Some((p, _)) = local_index[node] {
                    holds_free_ic_supply[p] = true;
                }
            }
            for pin in &ic.logic {
                node_to_ic_readers.entry(pin.node).or_default().push(*device_index);
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
            ics,
            holds_free_ic_supply,
            node_to_ic_readers,
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

    fn cache_key(&self, p_idx: usize, now: f64, drivers: &[Driver]) -> CacheKey {
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

        // Fixed device order, then logic-pin order (both already
        // deterministic — `self.ics`'/`IcRecord.logic`'s own construction
        // order), so this key compares stably across calls: the CONTENTS,
        // not merely the length, must match for two resolves to be
        // considered the same stamp.
        let mut floating_pulls = Vec::new();
        for (_, ic) in &self.ics {
            for pin in &ic.logic {
                if !(pin.open && pin.floating_input && !pin.open_high) {
                    continue;
                }
                if !self.in_partition(pin.node, p_idx) {
                    continue;
                }
                floating_pulls.push(digital::floating::floating_high(pin.open_hash, now, ic.ttl));
            }
        }

        CacheKey { driver_modes, rail_band: 0, floating_pulls, epoch: self.epoch }
    }

    /// Whether `node` is a FREE node belonging to partition `p_idx`
    /// specifically (`false` for a fixed node, or a free node in some OTHER
    /// partition).
    fn in_partition(&self, node: usize, p_idx: usize) -> bool {
        matches!(self.local_index.get(node).copied().flatten(), Some((p, _)) if p == p_idx)
    }

    /// `node`'s current voltage for the purposes of resolving partition
    /// `p_idx`: the fixed-point iterate `x` when `node` is free and IN
    /// `p_idx` (`Self::iterate_solve`'s own local solution vector, valid for
    /// indices `0..partitions[p_idx].len()`), otherwise `self.fixed[node]`.
    /// Never reads `self.voltages` directly: every node an IC stamp inside
    /// `p_idx` can ever need (its own free pin, or its IC's supply nodes) is
    /// guaranteed by `partition.rs`'s union rule to be either fixed or free
    /// IN `p_idx` — never free in some OTHER partition — so `x` (which may
    /// legitimately be `&[]` when `p_idx` holds no free IC supply node, see
    /// `Self::stamp_ics_into`'s own doc comment) is always the right place
    /// to look for the free case.
    fn read_node(&self, node: usize, p_idx: usize, x: &[f64]) -> f64 {
        match self.local_index.get(node).copied().flatten() {
            Some((p, l)) if p == p_idx => x[l],
            _ => self.fixed[node].expect("an IC pin's node reachable from a Static partition must be fixed or free within it (partition.rs's union rule / W1c step 2's -3 decline)"),
        }
    }

    /// Folds a two-terminal conductance `g` between global nodes `u` and `w`
    /// into partition `p_idx`'s LOCAL `size x size` scratch matrix `a`/`b` —
    /// the per-resolve counterpart of [`fold_static_stamp`] (which only ever
    /// runs once, at `build`, against the pristine GLOBAL `matrix_base`/
    /// `rhs_base`). Silently does nothing when NEITHER end is a free node in
    /// `p_idx` (both fixed, or free in some OTHER partition): every caller
    /// already guards its own "does this touch `p_idx` at all" question
    /// before calling in (`Self::stamp_switches_into`/`stamp_ics_into`'s own
    /// per-item skip), so that branch is a defensive no-op, never load-
    /// bearing.
    fn fold_local(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], u: usize, w: usize, g: f64) {
        if g == 0.0 {
            return;
        }
        match (self.local_index.get(u).copied().flatten(), self.local_index.get(w).copied().flatten()) {
            (Some((pu, lu_)), Some((pw, lw))) if pu == p_idx && pw == p_idx => {
                a[lu_ * size + lu_] += g;
                a[lw * size + lw] += g;
                a[lu_ * size + lw] -= g;
                a[lw * size + lu_] -= g;
            }
            (Some((pu, lu_)), _) if pu == p_idx => {
                a[lu_ * size + lu_] += g;
                b[lu_] += g * self.fixed[w].expect("the far end of a within-partition stamp must be fixed when it is not itself free in this partition");
            }
            (_, Some((pw, lw))) if pw == p_idx => {
                a[lw * size + lw] += g;
                b[lw] += g * self.fixed[u].expect("the far end of a within-partition stamp must be fixed when it is not itself free in this partition");
            }
            _ => {}
        }
    }

    /// Folds an open `FLOATING_INPUT` pin's Norton pull into partition
    /// `p_idx`'s local scratch matrix/rhs (`ic_pull_target`'s own doc
    /// comment: `g` from `pin_node` to the DATUM, plus the injection `g *
    /// target_v` into `pin_node`'s own row) — the pull's counterpart of
    /// [`Self::fold_local`], but simpler: unlike a driver's target, the
    /// datum is never itself a partition member with a KCL row of its own
    /// (node 0 is always fixed, `Self::fixed`'s own doc comment), so there is
    /// no "which end is free" dispatch to do, only the one free end
    /// `Self::stamp_ics_into` already confirmed with its own
    /// `Self::in_partition(pin.node, p_idx)` guard before ever calling in.
    /// Exactly classic's `mna.add(node, node, g); mna.addB(node, target *
    /// g)` (`_stampFloatingDigitalInputs`, sim-engine.ts:5202-5203).
    fn fold_pull(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], pin_node: usize, g: f64, target_v: f64) {
        if g == 0.0 {
            return;
        }
        let (p, l) = self
            .local_index
            .get(pin_node)
            .copied()
            .flatten()
            .expect("Self::stamp_ics_into already confirmed pin_node is free via Self::in_partition before calling fold_pull");
        debug_assert_eq!(p, p_idx, "Self::in_partition(pin_node, p_idx) already confirmed this");
        a[l * size + l] += g;
        b[l] += g * target_v;
    }

    /// Stamps every driver currently committed to HIGH or LOW whose OWN pin
    /// lives in partition `p_idx`, EXCEPT an IC driver (`Driver::device !=
    /// usize::MAX`) — `Self::stamp_ics_into` owns those, since an IC's
    /// conductance is not `driver.conductance` at all (the module's own
    /// pre-W1c behaviour, unchanged for whatever non-IC driver kind exists
    /// in the future).
    fn stamp_non_ic_drivers_into(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], drivers: &[Driver]) -> Result<(), String> {
        for driver in drivers {
            if driver.device != usize::MAX {
                continue;
            }
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
        Ok(())
    }

    /// Stamps every switch-family device's CURRENT contact state (fresh
    /// every resolve, unlike the resistor's immutable base — see this
    /// module's doc comment). A switch's own free pins are always entirely
    /// within one partition (`partition::build` unions every contact
    /// unconditionally), so a contact whose free end lands outside `p_idx`
    /// belongs to a different partition entirely and is simply skipped by
    /// `Self::fold_local`.
    fn stamp_switches_into(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64]) {
        for sw in &self.switches {
            for c in devices::stamp(sw.kind, sw.pins.len(), &sw.params) {
                if c.conductance == 0.0 {
                    continue; // README: "open stamps nothing".
                }
                self.fold_local(p_idx, size, a, b, sw.pins[c.pin_a], sw.pins[c.pin_b], c.conductance);
            }
        }
    }

    /// Stamps every IC driver/pull touching partition `p_idx` (W1c contract
    /// points 3/8): an IC is only even considered here when at least one of
    /// its driver/pull pins is free and IN `p_idx` — `partition.rs`'s union
    /// rule then guarantees its `vcc`/`gnd` are each either fixed or ALSO
    /// free in `p_idx`, never free in some other partition, so
    /// `Self::read_node` never has to fall through to a node this partition
    /// knows nothing about.
    ///
    /// `x` is the CURRENT fixed-point iterate (`Self::iterate_solve`'s local
    /// solution vector) when `p_idx` holds a free IC supply node, or `&[]`
    /// otherwise: `Self::read_node` only ever indexes into it for a node
    /// free-in-`p_idx`, and when `p_idx` holds no free IC supply at all,
    /// every IC touching it has both supply nodes fixed by definition
    /// (`self.holds_free_ic_supply[p_idx]` is exactly that fact) — so `x`
    /// is never actually indexed in that case, and an empty slice is safe.
    fn stamp_ics_into(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], now: f64, drivers: &[Driver], x: &[f64]) {
        for (device_index, ic) in &self.ics {
            // `touches` also fires when this IC's own free supply node lives
            // in `p_idx`, even if every one of its driver/pull pins is fixed
            // elsewhere (findings 3/8): an output pin wired straight onto a
            // fixed/ideal node still loads its IC's free `vcc`/`gnd` through
            // `1/R_out`, and that conductance belongs in whichever partition
            // holds the free end, regardless of which end named this IC in
            // the first place.
            let touches = drivers.iter().any(|d| d.device == *device_index && self.in_partition(d.node, p_idx))
                || ic.logic.iter().any(|p| p.open && p.floating_input && !p.open_high && self.in_partition(p.node, p_idx))
                || self.in_partition(ic.vcc.node, p_idx)
                || self.in_partition(ic.gnd.node, p_idx);
            if !touches {
                continue;
            }
            let v_vcc = self.read_node(ic.vcc.node, p_idx, x);
            let v_gnd = self.read_node(ic.gnd.node, p_idx, x);
            let power = digital::power::ic_power(ic, v_vcc, v_gnd);
            if !power.powered {
                continue;
            }
            for driver in drivers {
                // No `in_partition(driver.node, p_idx)` guard here (findings
                // 3/8): `Self::fold_local` already stamps correctly whether
                // the driver's own node or its commit target is the free end
                // in `p_idx` (or no-ops when NEITHER is), so pre-filtering on
                // the driver's own node only, as before, silently dropped
                // the "driver fixed, target free-in-`p_idx`" case entirely.
                if driver.device != *device_index {
                    continue;
                }
                if let Some((target, g)) = ic_driver_target(&power, driver, ic) {
                    self.fold_local(p_idx, size, a, b, driver.node, target, g);
                }
            }
            for pin in &ic.logic {
                if !self.in_partition(pin.node, p_idx) {
                    continue;
                }
                if let Some((g, target_v)) = ic_pull_target(ic, &power, pin, now) {
                    self.fold_pull(p_idx, size, a, b, pin.node, g, target_v);
                }
            }
        }
    }

    /// The ordinary, single-shot solve for a partition that holds no free IC
    /// supply node (`!self.holds_free_ic_supply[p_idx]`): every IC touching
    /// it has both supply nodes fixed, so its power (hence R_out, hence
    /// every driver/pull conductance) is constant for as long as the
    /// partition's topology is — computed once, not iterated (W1c contract
    /// point 3's last sentence: "When every supply node is fixed, the power
    /// is constant and the solve is direct.").
    fn direct_solve(&self, p_idx: usize, now: f64, drivers: &[Driver]) -> Result<Vec<f64>, String> {
        let size = self.partitions[p_idx].len();
        let mut a = self.matrix_base[p_idx].clone();
        let mut b = self.rhs_base[p_idx].clone();
        self.stamp_non_ic_drivers_into(p_idx, size, &mut a, &mut b, drivers)?;
        self.stamp_switches_into(p_idx, size, &mut a, &mut b);
        self.stamp_ics_into(p_idx, size, &mut a, &mut b, now, drivers, &[]);
        lu::solve(a, size, b).map_err(|_| format!("partition {p_idx} ({size} node(s)): singular or non-finite pivot"))
    }

    /// The fixed-point solve for a partition that holds at least one free IC
    /// supply node (W1c contract point 3): restamps every IC's
    /// driver/pull from the CURRENT iterate's supply voltage each pass
    /// (classic's own fixed-point iteration, no Jacobian term), until
    /// classic's Newton test passes per node or
    /// [`IC_FIXED_POINT_MAX_ITERATIONS`] is exceeded (`-4`). Writes the
    /// converged voltages straight into `self.voltages` (never the solution
    /// cache — W1c contract point 4: "bypass the cache for fixed-point
    /// partitions", since re-stamping every call already IS the point).
    fn iterate_solve(&mut self, p_idx: usize, now: f64, drivers: &[Driver]) -> Result<(), String> {
        let nodes = self.partitions[p_idx].clone();
        let size = nodes.len();
        // Warm start from whatever this partition last published (0.0 at
        // very first load, like every other free node) — a reasonable guess
        // that also makes a converged, unchanged re-resolve a fast, few-pass
        // confirmation rather than starting from scratch every time.
        let mut x: Vec<f64> = nodes.iter().map(|&n| self.voltages[n]).collect();
        let mut converged = false;
        for _iteration in 0..IC_FIXED_POINT_MAX_ITERATIONS {
            let mut a = self.matrix_base[p_idx].clone();
            let mut b = self.rhs_base[p_idx].clone();
            self.stamp_non_ic_drivers_into(p_idx, size, &mut a, &mut b, drivers)?;
            self.stamp_switches_into(p_idx, size, &mut a, &mut b);
            self.stamp_ics_into(p_idx, size, &mut a, &mut b, now, drivers, &x);
            let x_new = lu::solve(a, size, b).map_err(|_| format!("partition {p_idx} ({size} node(s)): singular or non-finite pivot"))?;
            // Classic's Newton test, per node: `|dx| <= 1e-6 + 1e-3 |x|`
            // (contract point 3), `|x|` read from the NEW iterate.
            let mut all_within_tol = true;
            for i in 0..size {
                let dx = (x_new[i] - x[i]).abs();
                if dx > IC_FIXED_POINT_ABS_TOL + IC_FIXED_POINT_REL_TOL * x_new[i].abs() {
                    all_within_tol = false;
                    break;
                }
            }
            x = x_new;
            for (l_idx, &node) in nodes.iter().enumerate() {
                self.voltages[node] = x[l_idx];
            }
            if all_within_tol {
                converged = true;
                break;
            }
        }
        if !converged {
            return Err(format!("partition {p_idx} ({size} node(s)): IC fixed-point iteration did not converge within {IC_FIXED_POINT_MAX_ITERATIONS} iterations"));
        }
        Ok(())
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
    /// W1c contract point 5's reader rule ("resolve_dirty returns every IC
    /// reader... that reads a node... whose published voltage changed in
    /// this re-solve, bitwise") is folded into the SAME per-partition loop
    /// that already solves each dirty partition: before either the
    /// cache-hit path or a fresh solve overwrites `self.voltages[node]`,
    /// this captures each node's PRE-resolve bits, then compares against the
    /// POST-resolve bits once whichever path ran has finished — a plain
    /// `f64 != f64` misses a NaN-to-NaN "no-op" (which never legitimately
    /// happens here, `check_finite` rejects NaN outright, but `to_bits`
    /// costs nothing extra and is exactly what "bitwise" calls for anyway).
    fn resolve_dirty(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
        let mut changed_readers: BTreeSet<usize> = BTreeSet::new();

        for &partition in dirty {
            let p_idx = partition.0 as usize;
            let nodes = self.partitions[p_idx].clone();
            let size = nodes.len();
            if size == 0 {
                continue;
            }

            let old_bits: Vec<u64> = nodes.iter().map(|&n| self.voltages[n].to_bits()).collect();

            if self.holds_free_ic_supply[p_idx] {
                // W1c contract point 3/4: R_out moves with this partition's
                // own solved state, so the solution cache never applies —
                // `Self::iterate_solve` writes `self.voltages` directly.
                self.iterate_solve(p_idx, now, drivers)?;
            } else {
                let key = self.cache_key(p_idx, now, drivers);
                let mut used_cache = false;
                if !self.no_solution_cache {
                    if let Some((cached_key, cached_x)) = self.cache.get(&p_idx) {
                        if cached_key == &key {
                            self.cache_hits += 1.0;
                            for (l_idx, &node) in nodes.iter().enumerate() {
                                self.voltages[node] = cached_x[l_idx];
                            }
                            used_cache = true;
                        }
                    }
                }
                if !used_cache {
                    let x = self.direct_solve(p_idx, now, drivers)?;
                    for (l_idx, &node) in nodes.iter().enumerate() {
                        self.voltages[node] = x[l_idx];
                    }
                    if !self.no_solution_cache {
                        self.cache.insert(p_idx, (key, x));
                    }
                }
            }

            for (l_idx, &node) in nodes.iter().enumerate() {
                if self.voltages[node].to_bits() != old_bits[l_idx] {
                    if let Some(readers) = self.node_to_ic_readers.get(&node) {
                        changed_readers.extend(readers.iter().copied());
                    }
                }
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

        Ok(changed_readers.into_iter().map(|d| DeviceId(d as u32)).collect())
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
            // Only switch-family kinds have live params, and none carries a blob.
            (shape.check)(sw.pins.len(), &new_params, &[]).map_err(|msg| format!("device {device_index} ({}): {msg}", shape.name))?;
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
