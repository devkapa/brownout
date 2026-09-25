//! Partitioning and classification (core/README.md, "W0 kernel semantics"
//! points 1-3; W1 plan section 3.2, row B1). Fixed-node derivation, the
//! iterative union-find over free nodes, and — new in W1b — classifying each
//! resulting partition as static, closed-form or stepped (Decision 14), so
//! `solve/static_part.rs` knows exactly which partitions it may actually
//! solve this step and which are a `ClosedForm`/`Stepped` "not yet" (no
//! device stamps a capacitor's companion model until W1b step 2's
//! `solve/closed_form.rs`/`solve/stepped.rs` exist).
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

use crate::devices;
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

/// How a partition's own devices classify it (Decision 14, W1 plan row B1).
/// Only [`PartitionKind::Static`] is reachable through W1b step 1: [`build`]
/// below rejects a program with any `ClosedForm`/`Stepped` partition as
/// `ERR_UNSUPPORTED_TOPOLOGY` (-3) the same way a floating or looped ideal
/// source already is, so `solve/static_part.rs` never has to handle one.
/// Kept as a real three-way enum — not a bool — so W1b step 2 can match on
/// it directly instead of re-deriving the classification from a capacitor
/// count a second time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionKind {
    /// No capacitor at all: a pure conductance network, solved by
    /// `solve/static_part.rs` every time it is dirtied.
    Static,
    /// Exactly one capacitor: eligible for W1b step 2's analytic
    /// segment/crossing solver (Decision 14, `solve/closed_form.rs`).
    ClosedForm,
    /// More than one capacitor (or, once ported, an inductor): the general
    /// stepped BE/trap path (`solve/stepped.rs`).
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
/// union-find. `kinds` is parallel to `partitions`; `build` below already
/// guarantees every entry is [`PartitionKind::Static`] (it errors out
/// otherwise), so a step-1 caller can ignore the field entirely if it
/// prefers — it exists so W1b step 2 does not need a second pass over the
/// same data to get the same classification.
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
}

/// Every pin-index pair (into a device's own `pins`) that can ever carry a
/// conductance between them, for topology purposes only — never for the
/// actual electrical stamp, which reads the device's CURRENT parameters
/// (`solve/static_part.rs`, `devices::stamp`). A resistor or capacitor has
/// exactly one; a switch-family kind defers to
/// [`devices::switches::contact_pairs`] (see this module's doc comment on
/// why those are unioned unconditionally).
fn union_pairs(kind: u32, pin_count: usize) -> Vec<(usize, usize)> {
    match kind {
        kinds::RESISTOR | kinds::CAPACITOR => vec![(0, 1)],
        kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH => {
            devices::switches::contact_pairs(kind, pin_count)
        }
        _ => Vec::new(),
    }
}

/// Builds the static topology: fixed-node propagation from node 0
/// ([`derive_fixed_nodes`]), then union-find over free nodes joined by every
/// resistor, capacitor or switch-family contact whose two ends are both free
/// (see this module's doc comment for why a switch-family contact is
/// unioned regardless of its current open/closed state), then classifies
/// each resulting partition (Decision 14) and rejects a program this step
/// cannot solve — a `ClosedForm`/`Stepped` partition, or a capacitor with
/// BOTH terminals fixed (so it belongs to no partition at all, but still
/// needs dynamic state no kind here can give it before W1b step 2) — as
/// `ERR_UNSUPPORTED_TOPOLOGY` (-3), exactly the code the contract already
/// uses for "the TS side falls back to classic".
pub fn build(program: &Program) -> Result<PartitionBuild, BoError> {
    let n = program.node_count;
    let (fixed, resolved) = derive_fixed_nodes(program, n)?;

    let mut uf = UnionFind::new(n);
    for device in &program.devices {
        for (pa, pb) in union_pairs(device.kind, device.pins.len()) {
            let (u, w) = (device.pins[pa].node, device.pins[pb].node);
            if fixed[u].is_none() && fixed[w].is_none() {
                uf.union(u, w);
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
        match (local_index[u], local_index[w]) {
            (Some((p, _)), _) | (_, Some((p, _))) => capacitor_count[p] += 1,
            (None, None) => {
                return Err(BoError::unsupported_topology(format!(
                    "capacitor (component {}): both terminals are fixed nodes — dynamic partitions are not implemented until W1b step 2",
                    device.component_index
                )));
            }
        }
    }

    let kinds_by_partition: Vec<PartitionKind> = capacitor_count.into_iter().map(classify).collect();
    if let Some(p_idx) = kinds_by_partition.iter().position(|k| *k != PartitionKind::Static) {
        return Err(BoError::unsupported_topology(format!(
            "partition {p_idx}: contains a capacitor — dynamic partitions are not implemented until W1b step 2"
        )));
    }

    Ok(PartitionBuild { partitions, local_index, fixed, resolved, kinds: kinds_by_partition })
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
