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
/// here) and rejects a program no partition can ever own — a capacitor with
/// BOTH terminals fixed (so it belongs to no partition at all: not a
/// resistive network `solve/static_part.rs` can solve, and not a free-node
/// dynamical system `solve/closed_form.rs`/`solve/stepped.rs` integrate
/// either) — as `ERR_UNSUPPORTED_TOPOLOGY` (-3), exactly the code the
/// contract already uses for "the TS side falls back to classic". A
/// self-shorted capacitor (both pins the SAME node, free or fixed — contract
/// point 2) is neither of those: it is simply excluded below, before this
/// rejection could ever see it.
pub fn build(program: &Program) -> Result<PartitionBuild, BoError> {
    let n = program.node_count;
    let (fixed, resolved) = derive_fixed_nodes(program, n)?;

    let mut uf = UnionFind::new(n);
    for device in &program.devices {
        if let Some((pa, pb)) = resistor_or_capacitor_pair(device.kind) {
            let (u, w) = (device.pins[pa].node, device.pins[pb].node);
            if fixed[u].is_none() && fixed[w].is_none() {
                uf.union(u, w);
            }
            continue;
        }
        if !matches!(device.kind, kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH) {
            continue;
        }
        // Every free pin of this switch-family device joins the SAME
        // partition, however many contacts separate them and regardless of
        // which of the device's other pins are fixed (this module's doc
        // comment) — union the first free pin found with every other one,
        // which is enough to merge them all under union-by-size.
        let mut free_pins = device.pins.iter().map(|p| p.node).filter(|&node| fixed[node].is_none());
        if let Some(first) = free_pins.next() {
            for node in free_pins {
                uf.union(first, node);
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
            // ever hit the "both terminals fixed" rejection below — that
            // rejection is for a capacitor with no free terminal AT ALL,
            // not one whose single shared terminal happens to be fixed.
            continue;
        }
        match (local_index[u], local_index[w]) {
            (Some((p, _)), _) | (_, Some((p, _))) => capacitor_count[p] += 1,
            (None, None) => {
                return Err(BoError::unsupported_topology(format!(
                    "capacitor (component {}): both terminals are fixed nodes — a capacitor needs at least one free terminal to belong to a partition",
                    device.component_index
                )));
            }
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
    let force_stepped = program.options.force_stepped;
    let kinds_by_partition: Vec<PartitionKind> = capacitor_count
        .into_iter()
        .map(classify)
        .map(|kind| if force_stepped && kind == PartitionKind::ClosedForm { PartitionKind::Stepped } else { kind })
        .collect();

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
