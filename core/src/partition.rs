//! Partitioning and classification (core/README.md, "W0 kernel semantics"
//! points 1-3; W1 plan section 3.2, rows B1/B2/B3). Fixed-node derivation,
//! the iterative union-find over free nodes, and classifying each resulting
//! partition as static, closed-form or stepped (Decision 14): `solve::mod`'s
//! `PartitionedSolver` (W1b step 2) uses [`PartitionBuild::kinds`] to decide,
//! per partition, whether `solve/static_part.rs`, `solve/closed_form.rs` or
//! `solve/stepped.rs` owns it. `FORCE_STEPPED` (core/README.md's Options
//! table, key 4 — tests only) is applied here, once, by [`build`]: every
//! `ClosedForm` partition becomes `Stepped` instead, so every downstream
//! consumer of `PartitionBuild::kinds` sees the final classification
//! directly and never has to re-check the option itself.
//!
//! Moved here from `engine.rs`'s former `StaticPartitionSolver::build`
//! (W0/W1a): fixed-node propagation from node 0 and the iterative,
//! union-by-size union-find over free nodes are unchanged. What is new:
//! capacitors and switch-family contacts also join the union (so a
//! capacitor-only link between two nodes still lands them in one partition
//! to classify), and every switch-family contact is unioned
//! UNCONDITIONALLY, regardless of whether it is open or closed right now.
//!
//! That last rule is deliberate, not an oversight: unlike a resistor's `R`,
//! a switch's `closed`/`sel`/`sw_i` can change after load
//! (`bo_engine_set_params`, owner decision 1), and nothing ever
//! re-partitions after load — partition membership is decided once, here,
//! at `bo_engine_load` time. If an open switch's pins were only unioned
//! while closed, toggling it could move a node from one partition to
//! another (or split/merge whole partitions) behind the kernel's back,
//! which nothing downstream (`Kernel::add_driver`'s fixed partition
//! assignment, the solution cache's per-partition keying) is built to
//! handle. Unioning unconditionally means a live toggle only ever changes
//! what gets STAMPED into an already-fixed partition
//! (`solve/static_part.rs`), exactly like a driver's commit already does —
//! never which partition a node belongs to. The cost is that an open
//! switch's two nodes still share a (possibly larger) dense partition with
//! whatever else the switch would connect if closed; correctness is
//! unaffected, since a zero-conductance contact contributes nothing to that
//! shared system either way.
//!
//! **A switch-family device's free pins are unioned as a WHOLE, not contact
//! by contact.** A single contact's own two ends (README point 1's second
//! bullet) is not enough: an `spdt_switch`'s `com` can sit on a fixed pin (a
//! source) while `b` and `c` are both free, so neither the `com`-`b` nor the
//! `com`-`c` contact ever has two free ends to union — yet `b` and `c` are
//! still the same device's two alternatives for the SAME current path, and
//! `solve::mod`'s single-owner dispatch (`owner_of`, `owned_by_static`,
//! `PartitionSolver::apply_params`'s dirtying) and `compile_dyn_shape`'s
//! per-partition `resolve` both depend on a device's free pins landing in
//! exactly one partition, no matter which pin happens to be fixed. So `build`
//! below unions every free pin of a switch-family device with every other
//! free pin of that SAME device directly — not only pairwise by contact —
//! whenever the device has two or more free pins at all. This subsumes the
//! plain "both ends of one contact, when both are free" rule as a special
//! case (a contact's two pins are just two of the device's own pins), and it
//! is what makes it safe for `compile_dyn_shape::resolve` to treat any other
//! partition's free node as unreachable, rather than a case it has to
//! silently paper over.

use crate::digital;
use crate::error::BoError;
use crate::kinds;
use crate::program::Program;

/// One entry per source that fixed a node during propagation, in the order
/// it was resolved. `parent_node` was already fixed at the moment `source`
/// (a device index) fixed `child_node`; `parent_is_pos` says whether
/// `parent_node` is that source's `pins[0]` (+) or `pins[1]` (-). Recorded so
/// the current pass can walk the propagation forest children-before-parents
/// (contract, point 5) by simply reversing this list.
#[derive(Debug, Clone)]
pub struct ResolvedSource {
    pub device_index: usize,
    pub parent_node: usize,
    pub child_node: usize,
    pub parent_is_pos: bool,
}

/// How a partition's own devices classify it (Decision 14, W1 plan row B1;
/// the `FORCE_STEPPED` override, row B2/B3, is applied by [`build`] before
/// this ever reaches a caller). Kept as a real three-way enum — not a bool —
/// so `solve::mod`'s `PartitionedSolver` can match on it directly instead of
/// re-deriving the classification from a capacitor count a second time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionKind {
    /// No capacitor at all: a pure conductance network, solved by
    /// `solve/static_part.rs` every time it is dirtied.
    Static,
    /// Exactly one capacitor: eligible for the analytic segment/crossing
    /// solver (Decision 14, `solve/closed_form.rs`) — unless `FORCE_STEPPED`
    /// is set, in which case [`build`] reclassifies it as `Stepped` instead
    /// (tests only, core/README.md's Options table key 4).
    ClosedForm,
    /// More than one capacitor (or, once ported, an inductor or a nonlinear
    /// element), or any `ClosedForm` partition under `FORCE_STEPPED`: the
    /// general stepped BE/trap path (`solve/stepped.rs`).
    Stepped,
}

fn classify(capacitor_count: usize) -> PartitionKind {
    match capacitor_count {
        0 => PartitionKind::Static,
        1 => PartitionKind::ClosedForm,
        _ => PartitionKind::Stepped,
    }
}

/// The result of partitioning: everything a solver needs to build its own
/// per-partition state, without redoing fixed-node derivation or
/// union-find. `kinds` is parallel to `partitions`, with `FORCE_STEPPED`
/// already folded in — a caller that only ever handles
/// [`PartitionKind::Static`] (there is none such left in this crate after
/// W1b step 2, but a future standalone tool might) can still ignore the
/// field entirely if it prefers.
pub struct PartitionBuild {
    /// Node ids per partition, in first-seen order (deterministic; contract
    /// point 3 of "W0 kernel semantics").
    pub partitions: Vec<Vec<usize>>,
    /// node -> (partition index, local index), `None` for a fixed node.
    pub local_index: Vec<Option<(usize, usize)>>,
    /// node -> fixed voltage, `None` for a free node. Node 0 is always
    /// `Some(0.0)`.
    pub fixed: Vec<Option<f64>>,
    /// Every source that fixed a node, in resolution order.
    pub resolved: Vec<ResolvedSource>,
    pub kinds: Vec<PartitionKind>,
    /// How many partitions have >= 2 capacitors — real multi-capacitor
    /// linear partitions, counted from the ORIGINAL capacitor count before
    /// `FORCE_STEPPED` could reclassify a single-capacitor `ClosedForm`
    /// partition into `Stepped` too (so this count does not move under that
    /// option). W1b's report to the orchestrator: this is a candidate for a
    /// new `bo_engine_stat` index 15, not yet in the ABI contract table
    /// (core/README.md's "Stat indices") — `engine.rs` already exposes it at
    /// that index, pending the orchestrator adding the row.
    pub multi_capacitor_partitions: usize,
}

/// The one pin-index pair (into a device's own `pins`) a resistor or
/// capacitor can ever carry a conductance between, for topology purposes
/// only — never for the actual electrical stamp, which reads the device's
/// CURRENT parameters (`solve/static_part.rs`, `devices::stamp`). A
/// switch-family kind is unioned by [`build`] itself instead (this module's
/// doc comment on why its free pins are unioned as a whole device, not pair
/// by pair the way a resistor's or capacitor's single pair is).
fn resistor_or_capacitor_pair(kind: u32) -> Option<(usize, usize)> {
    match kind {
        kinds::RESISTOR | kinds::CAPACITOR => Some((0, 1)),
        _ => None,
    }
}

/// Builds the static topology: fixed-node propagation from node 0
/// ([`derive_fixed_nodes`]), then union-find over free nodes joined by every
/// resistor or capacitor whose two ends are both free, and by every free pin
/// of a switch-family device with every other free pin of that same device
/// (this module's doc comment explains why the whole device, not contact by
/// contact — a switch-family contact can run through a fixed pin), then
/// classifies each resulting partition (Decision 14, `FORCE_STEPPED` applied
/// here). A capacitor with BOTH terminals fixed belongs to no partition
/// either — not a resistive network `solve/static_part.rs` can solve, and
/// not a free-node dynamical system `solve/closed_form.rs`/`solve/stepped.rs`
/// integrate — but, unlike a loop of ideal sources, this is NOT
/// `ERR_UNSUPPORTED_TOPOLOGY` (-3): core/README.md's rule 2a (a decoupling
/// capacitor straight across an ideal supply) gives it an exact closed form
/// of its own, `solve::mod`'s `PartitionedSolver` owns it directly (see that
/// module's `FixedPairCapacitor`), and this function simply excludes it from
/// every partition's `capacitor_count` below, the same way a self-shorted
/// capacitor (both pins the SAME node, free or fixed — contract point 2) is
/// excluded.
pub fn build(program: &Program) -> Result<PartitionBuild, BoError> {
    let n = program.node_count;
    let (fixed, resolved) = derive_fixed_nodes(program, n)?;

    // Every free IC supply (`vcc`/`gnd`) node — used below to promote a
    // ClosedForm partition that holds one to Stepped (contract point 3).
    // Collected in the SAME pass that unions an IC's driver/pull pins with
    // it, so this pass never has to re-decode an IC's pin flags a second
    // time. `fixed` is all this pass needs (not `local_index`, built after
    // every union below), matching the switch-family union loop's own
    // "topology only needs `fixed`" shape.
    let mut ic_free_supply_nodes: Vec<usize> = Vec::new();

    let mut uf = UnionFind::new(n);
    for device in &program.devices {
        if let Some((pa, pb)) = resistor_or_capacitor_pair(device.kind) {
            let (u, w) = (device.pins[pa].node, device.pins[pb].node);
            if fixed[u].is_none() && fixed[w].is_none() {
                uf.union(u, w);
            }
            continue;
        }
        if matches!(device.kind, kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH) {
            // Every free pin of this switch-family device joins the SAME
            // partition, however many contacts separate them and regardless
            // of which of the device's other pins are fixed (this module's
            // doc comment) — union the first free pin found with every other
            // one, which is enough to merge them all under union-by-size.
            let mut free_pins = device.pins.iter().map(|p| p.node).filter(|&node| fixed[node].is_none());
            if let Some(first) = free_pins.next() {
                for node in free_pins {
                    uf.union(first, node);
                }
            }
            continue;
        }
        if !digital::is_ic(device.kind) {
            continue;
        }
        // W1c contract point 2: "the union-find also joins each free
        // output-capable pin, and each free OPEN FLOATING_INPUT pin, with
        // every free supply pin of the same IC." `IcRecord::from_device` is
        // safe to call unconditionally here: `program.rs`'s reader has
        // already validated this device against `kinds::shape`'s IC shape
        // (pin/param/blob counts, flag codes) before `partition::build` ever
        // runs (`digital::mod`'s own doc comment on `from_device`). W1d
        // widened this from combinational-only to every IC kind
        // (`digital::is_ic`): a sequential/memory IC's own output-capable
        // and floating-input pins join its supply exactly the same way.
        let ic = digital::IcRecord::from_device(device);
        let free_supply: Vec<usize> =
            [ic.vcc.node, ic.gnd.node].into_iter().filter(|&node| fixed[node].is_none()).collect();
        ic_free_supply_nodes.extend(&free_supply);

        let output_capable = digital::output_capable(ic.kind);
        let free_driver_pull_nodes = output_capable
            .iter()
            .map(|&i| ic.logic[i].node)
            // Point 6's pull never applies to an OPEN_HIGH pin ("IC_OPEN_HIGH_PINS
            // gets no pull at all"), so such a pin never stamps anything and
            // has no business joining this union either — matching
            // `digital::floating::pull`'s own eligibility test exactly (minus
            // its `powered` check, which is a runtime fact, not a topology
            // one: partition membership is fixed at load regardless of
            // whether the IC happens to be powered at that instant).
            .chain(ic.logic.iter().filter(|p| p.open && p.floating_input && !p.open_high).map(|p| p.node))
            .filter(|&node| fixed[node].is_none());
        for node in free_driver_pull_nodes {
            for &supply in &free_supply {
                uf.union(supply, node);
            }
        }
    }

    // Group free nodes into partitions by union-find representative,
    // preserving first-seen order for determinism (node id order, not
    // representative id order — the latter depends on union direction).
    let mut partition_of: Vec<Option<usize>> = vec![None; n];
    let mut partitions: Vec<Vec<usize>> = Vec::new();
    for node in 1..n {
        if fixed[node].is_some() {
            continue;
        }
        let root = uf.find(node);
        let idx = match partition_of[root] {
            Some(idx) => idx,
            None => {
                let idx = partitions.len();
                partitions.push(Vec::new());
                partition_of[root] = Some(idx);
                idx
            }
        };
        partitions[idx].push(node);
    }

    let mut local_index: Vec<Option<(usize, usize)>> = vec![None; n];
    for (p_idx, nodes) in partitions.iter().enumerate() {
        for (l_idx, &node) in nodes.iter().enumerate() {
            local_index[node] = Some((p_idx, l_idx));
        }
    }

    let mut capacitor_count = vec![0usize; partitions.len()];
    for device in &program.devices {
        if device.kind != kinds::CAPACITOR {
            continue;
        }
        let (u, w) = (device.pins[0].node, device.pins[1].node);
        if u == w {
            // Self-shorted (contract point 2): both terminals land on the
            // SAME node, free or fixed (gnd included) — its companion stamps
            // cancel exactly like classic's `stampConductance` with `i==j`,
            // so it stamps nothing, publishes 0 and must not count toward
            // this node's partition classification (an otherwise-Static
            // node must not be pulled into ClosedForm/Stepped by a capacitor
            // that never actually contributes to the matrix), nor can it
            // ever be the DISTINCT-fixed-nodes case just below — that one is
            // for a capacitor with no free terminal AT ALL, not one whose
            // single shared terminal happens to be fixed.
            continue;
        }
        match (local_index[u], local_index[w]) {
            (Some((p, _)), _) | (_, Some((p, _))) => capacitor_count[p] += 1,
            // Rule 2a: both terminals are FIXED, DISTINCT nodes (a
            // decoupling capacitor straight across an ideal supply — not
            // rule 2's self-shorted case, already handled above). It
            // belongs to no partition and is not `ERR_UNSUPPORTED_TOPOLOGY`:
            // its terminal voltage `dV` never moves (no fixed-node voltage
            // is live in W1b), so it evolves on its own, exactly.
            // `solve::mod::PartitionedSolver` is the only place that ever
            // looks at this device again.
            (None, None) => {}
        }
    }

    // Decision 14 (W1 plan row B2/B3): classify by capacitor count, then fold
    // in `FORCE_STEPPED` — a test-only override, core/README.md's Options
    // table key 4 — which sends every single-capacitor `ClosedForm`
    // partition down the general stepped path instead, so the same fixture
    // can be run both ways to compare the exact closed form against the
    // BE/trap step-doubling controller (W1 plan row B3's verify step: "the
    // RC/RL discrete-conservation benchmarks under `FORCE_STEPPED`").
    // `Stepped` (already >= 2 capacitors, or nonlinear once W1c+ adds a
    // nonlinear kind) is left alone: forcing an already-stepped partition to
    // itself is a no-op.
    let multi_capacitor_partitions = capacitor_count.iter().filter(|&&c| c >= 2).count();

    // W1c contract point 3: "A dynamic partition holding a free IC supply
    // node is stepped" — R_out then moves with that partition's own solved
    // state, which only the general BE/trap path (never the exact
    // closed-form one) can re-stamp from. Computed from `ic_free_supply_nodes`
    // (collected in the union pass above) against the now-final `local_index`,
    // same shape as `capacitor_count`'s own per-partition accumulation.
    // A dynamic partition that must be stepped rather than closed-form
    // (W1c point 3): it holds a free IC supply node, whose R_out moves with
    // the partition's state, or a 74LS245 a/b pin (below).
    let mut must_step = vec![false; partitions.len()];
    for &node in &ic_free_supply_nodes {
        if let Some((p, _)) = local_index[node] {
            must_step[p] = true;
        }
    }
    // A 74LS245's a/b pins (and, W1d, an EEPROM's io0..io7) are read AND
    // driven, so the closed-form crossing watch cannot cover them: watching
    // a pin its own IC drives makes the crossing solve divide by a
    // near-zero coupling (solve/mod.rs, `register_ic_read_pin_watches`). A
    // stepped partition re-reads its ICs after every step instead, so a
    // partition holding one is stepped (core/README.md, W1c point 3 /
    // W1d memories point 1: "the EEPROM's io pins ... follow the 74ls245
    // rule").
    for device in &program.devices {
        if !digital::is_read_and_driven(device.kind) {
            continue;
        }
        let ic = digital::IcRecord::from_device(device);
        for &i in digital::output_capable(ic.kind) {
            if let Some((p, _)) = local_index[ic.logic[i].node] {
                must_step[p] = true;
            }
        }
    }

    let force_stepped = program.options.force_stepped;
    let kinds_by_partition: Vec<PartitionKind> = capacitor_count
        .into_iter()
        .map(classify)
        .enumerate()
        .map(|(p, kind)| {
            if kind == PartitionKind::ClosedForm && (force_stepped || must_step[p]) {
                PartitionKind::Stepped
            } else {
                kind
            }
        })
        .collect();

    // W1c step 2b (this worker's own scope, README's "Kernel semantics: the
    // digital core (W1c)" contract point 2's stamping half): an IC driver or
    // floating pull landing in a non-`Static` partition used to be declined
    // outright ("ic stamp in a dynamic partition (W1c step 2b)"), because
    // `solve::closed_form`/`solve::stepped` did not yet know how to stamp an
    // IC. They do now (`solve::stamp_ics_into_dyn`, called from both
    // modules' own segment/step builders), so nothing here needs to reject
    // this topology any more — every partition kind accepts every IC
    // footprint the union rule above can produce.
    Ok(PartitionBuild { partitions, local_index, fixed, resolved, kinds: kinds_by_partition, multi_capacitor_partitions })
}

/// Fixed-point propagation from node 0 (contract, point 1). Returns each
/// node's fixed voltage (`None` for free nodes) and the list of sources that
/// did the fixing, in resolution order. `ERR_UNSUPPORTED_TOPOLOGY` (-3) for a
/// loop of ideal sources or a floating one.
fn derive_fixed_nodes(program: &Program, n: usize) -> Result<(Vec<Option<f64>>, Vec<ResolvedSource>), BoError> {
    let mut fixed: Vec<Option<f64>> = vec![None; n];
    fixed[0] = Some(0.0);

    let source_indices: Vec<usize> = program
        .devices
        .iter()
        .enumerate()
        .filter(|(_, d)| d.kind == kinds::VOLTAGE_SOURCE)
        .map(|(i, _)| i)
        .collect();
    let mut is_resolved = vec![false; source_indices.len()];
    let mut resolved = Vec::with_capacity(source_indices.len());

    // Each pass can resolve at least one source or the loop is done; a chain
    // of k sources needs at most k passes in the worst ordering.
    for _pass in 0..=source_indices.len() {
        let mut progress = false;
        for (si, &device_index) in source_indices.iter().enumerate() {
            if is_resolved[si] {
                continue;
            }
            let device = &program.devices[device_index];
            let (pos, neg) = (device.pins[0].node, device.pins[1].node);
            let v = device.params[0];
            match (fixed[pos], fixed[neg]) {
                (Some(vp), None) => {
                    fixed[neg] = Some(vp - v);
                    is_resolved[si] = true;
                    resolved.push(ResolvedSource { device_index, parent_node: pos, child_node: neg, parent_is_pos: true });
                    progress = true;
                }
                (None, Some(vn)) => {
                    fixed[pos] = Some(vn + v);
                    is_resolved[si] = true;
                    resolved.push(ResolvedSource { device_index, parent_node: neg, child_node: pos, parent_is_pos: false });
                    progress = true;
                }
                // Both fixed or both free: nothing to do this pass. Both-free
                // may still resolve later; both-fixed never will (see the
                // final classification below) — it is left alone here so
                // topology errors are reported once, after propagation truly
                // stalls, not from an order-dependent snapshot.
                _ => {}
            }
        }
        if !progress {
            break;
        }
    }

    for (si, &device_index) in source_indices.iter().enumerate() {
        if is_resolved[si] {
            continue;
        }
        let device = &program.devices[device_index];
        let (pos, neg) = (device.pins[0].node, device.pins[1].node);
        return Err(BoError::unsupported_topology(if fixed[pos].is_some() && fixed[neg].is_some() {
            format!(
                "voltage_source (component {}): both terminals already fixed — a loop of ideal sources",
                device.component_index
            )
        } else {
            format!(
                "voltage_source (component {}): floating — neither terminal is reachable from the datum",
                device.component_index
            )
        }));
    }

    Ok((fixed, resolved))
}

struct UnionFind {
    parent: Vec<usize>,
    /// Size of the tree rooted at `i`, meaningful only when `parent[i] == i`.
    /// Drives union-by-size below, so a chain of unions in any order — not
    /// just an already-favorable one — keeps every tree at O(log n) depth.
    size: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        UnionFind { parent: (0..n).collect(), size: vec![1; n] }
    }

    /// Iterative with path halving: every other node on the walk up is
    /// re-parented straight to its grandparent, which flattens the tree over
    /// repeated calls. No recursion means no input — however long a chain of
    /// unions it builds — can make this walk consume call-stack frames the
    /// way a recursive find would (contract, "W0 kernel semantics", point 3).
    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }

    /// Union by size: the smaller tree is grafted under the larger tree's
    /// root, so depth grows as O(log n) regardless of the order unions
    /// arrive in.
    fn union(&mut self, a: usize, b: usize) {
        let mut ra = self.find(a);
        let mut rb = self.find(b);
        if ra == rb {
            return;
        }
        if self.size[ra] < self.size[rb] {
            std::mem::swap(&mut ra, &mut rb);
        }
        self.parent[rb] = ra;
        self.size[ra] += self.size[rb];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinds::ic_param;
    use crate::program::{Device, Options, PinRecord};

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }

    fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }

    fn capacitor(ci: usize, a: usize, b: usize) -> Device {
        Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![1e-6, 0.0, 0.0], blob: vec![] }
    }

    /// A minimal `74ls04` device: `pins[i]` is logic pin `i`'s node
    /// (`icPinMap` order `1a 1y 2a 2y ... 6a 6y`), every pin driven (never
    /// `PIN_FLAG_OPEN`). Standard LS-TTL params, matching
    /// `solve::ic_tests`'s own builder of the same shape.
    fn ls04(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12]) -> Device {
        let mut device_pins = vec![pin(vcc), pin(gnd)];
        device_pins.extend(pins.iter().map(|&n| pin(n)));
        let mut params = vec![0.0; ic_param::COUNT];
        params[ic_param::VCC_MIN] = 4.75;
        params[ic_param::VCC_MAX] = 5.25;
        params[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
        params[ic_param::VIL_FIXED] = 1.0;
        params[ic_param::VIL] = 0.8;
        params[ic_param::VIH_FIXED] = 1.0;
        params[ic_param::VIH] = 2.0;
        params[ic_param::NOMINAL] = 5.0;
        params[ic_param::IO_MAX] = 0.008;
        params[ic_param::DELAY] = 20e-9;
        let mut blob = vec![0.0];
        blob.extend((0..12).map(|i| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }));
        Device { kind: kinds::IC_74LS04, component_index: ci, pins: device_pins, params, blob }
    }

    fn program(node_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    }

    // ---- step 2b: the former dynamic-partition decline is lifted ---------
    //
    // These two fixtures are exactly the ones that used to prove the `-3`
    // decline (a git-log curiosity now); step 2b's own job is to prove the
    // opposite — the load now succeeds, with the IC's driver/pull footprint
    // classified into whatever dynamic partition it actually lands in. The
    // electrical correctness of what gets stamped there is `solve::ic_tests`'
    // job (a full kernel/solver round trip), not this module's: `partition.rs`
    // only ever decides topology and classification.

    #[test]
    fn an_ic_output_landing_in_a_closed_form_partition_now_loads_and_classifies_closed_form() {
        // vcc/gnd fixed (5V/0V, an ideal supply — contract point 2's "no IC
        // pin joins anything" case), so 1y's own partition is decided purely
        // by what else connects to it: a capacitor to gnd, making it
        // ClosedForm on its own, with no help from the IC's union rule at
        // all. That partition is exactly where 1y's driver needs to stamp —
        // not `Static` — which no longer fails the load.
        let p = program(4, vec![
            source(0, 1, 0, 5.0),
            capacitor(1, 2, 0),
            ls04(2, 1, 0, [0, 2, 0, 3, 0, 3, 0, 3, 0, 3, 0, 3]),
        ]);
        let pb = build(&p).expect("an IC driver landing in a ClosedForm partition must load now that step 2b stamps it");
        let (p_1y, _) = pb.local_index[2].expect("1y is a free node");
        assert_eq!(pb.kinds[p_1y], PartitionKind::ClosedForm);
    }

    #[test]
    fn an_open_floating_input_pin_landing_in_a_stepped_partition_now_loads_and_classifies_stepped() {
        // Two capacitors on the SAME node as an open, FLOATING_INPUT pin (2a
        // here): Stepped (>= 2 capacitors), not ClosedForm — the load must
        // succeed regardless of which dynamic kind the pull lands in.
        let p = program(5, vec![
            source(0, 1, 0, 5.0),
            capacitor(1, 2, 0),
            capacitor(2, 2, 0),
            {
                let mut d = ls04(3, 1, 0, [0, 3, 2, 4, 0, 3, 0, 3, 0, 3, 0, 3]);
                d.pins[4].flags = crate::program::PIN_FLAG_OPEN; // 2a (pins[0..2]=vcc,gnd, then 1a,1y,2a,...): index 4 = 2a.
                d
            },
        ]);
        let pb = build(&p).expect("an open floating-input pull landing in a Stepped partition must load now that step 2b stamps it");
        let (p_2a, _) = pb.local_index[2].expect("2a is a free node");
        assert_eq!(pb.kinds[p_2a], PartitionKind::Stepped);
    }

    // ---- point 3: a dynamic partition holding a free IC supply node is
    // classified Stepped ------------------------------------------------

    #[test]
    fn a_closed_form_partition_holding_a_free_ic_supply_node_is_classified_stepped() {
        // vcc FREE (through a resistor from the 5V source), gnd fixed. Every
        // logic pin is tied straight to a FIXED node (never free), so no IC
        // driver/pull ever lands in vcc's partition at all -- only a plain
        // decoupling capacitor from vcc to gnd does (one free end, so NOT
        // rule 2a's fixed-pair case), isolating the promotion rule from the
        // `-3` decline: this partition holds a free IC supply node and one
        // capacitor, so it must be classified Stepped, not ClosedForm, and
        // `build` must still succeed (nothing here ever needs step 2b).
        let p = program(3, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 2, 1, 10.0),
            capacitor(2, 2, 0),
            ls04(3, 2, 0, [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
        ]);
        let pb = build(&p).unwrap();
        let (p_vcc, _) = pb.local_index[2].expect("vcc is free");
        assert_eq!(pb.kinds[p_vcc], PartitionKind::Stepped, "a free IC supply node forces Stepped, never ClosedForm");
    }

    #[test]
    fn a_closed_form_partition_holding_a_74ls245_a_or_b_pin_is_classified_stepped() {
        // Fixed 5 V rails. a1 (logic pin 2) sits on an RC node: one counted
        // capacitor, so ClosedForm by count, but the pin is read AND driven,
        // so the partition must be Stepped. Every other logic pin is tied to
        // gnd.
        let mut pins = vec![pin(1), pin(0)];
        pins.extend((0..18).map(|i| pin(if i == 2 { 2 } else { 0 })));
        let mut params = vec![0.0; ic_param::COUNT];
        params[ic_param::VCC_MIN] = 4.75;
        params[ic_param::VCC_MAX] = 5.25;
        params[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
        params[ic_param::NOMINAL] = 5.0;
        params[ic_param::IO_MAX] = 0.024;
        params[ic_param::DELAY] = 20e-9;
        let ls245 = Device { kind: kinds::IC_74LS245, component_index: 3, pins, params, blob: vec![0.0; 19] };
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 10_000.0), capacitor(2, 2, 0), ls245]);
        let pb = build(&p).unwrap();
        let (p_a1, _) = pb.local_index[2].expect("a1's node is free");
        assert_eq!(pb.kinds[p_a1], PartitionKind::Stepped);
    }

    #[test]
    fn union_find_handles_a_million_node_chain_without_recursing() {
        // The historical bug (W0): a recursive `find` spends one call-stack
        // frame per link in the parent chain, and a plain "always point a at
        // b" union with no size heuristic turns a chain of unions listed in
        // ascending order into a parent chain as long as the input. At
        // 1,000,000 nodes — the program format's own ceiling
        // (`program::MAX_NODES_OR_DEVICES`) — that recursion overflows the
        // stack. This exercises `UnionFind` directly (no dense solve, so no
        // memory ceiling) at exactly that size: iterative `find` must handle
        // it regardless of how deep any particular union order would have
        // made a naive tree.
        let n = 1_000_000;
        let mut uf = UnionFind::new(n);
        for i in 0..n - 1 {
            uf.union(i, i + 1);
        }
        let root = uf.find(0);
        for i in (0..n).step_by(9973) {
            assert_eq!(uf.find(i), root, "node {i} did not join the single chain's tree");
        }
        assert_eq!(uf.find(n - 1), root);
    }
}
