//! The W0 kernel (core/README.md, "W0 kernel semantics"): fixed-node
//! derivation with parent tracking, union-find partitioning, a dense LU
//! solve per partition, and the KCL pass that publishes device currents.
//!
//! `Engine` is a plain, safe Rust type with no `extern "C"` on it anywhere —
//! `abi.rs` is the only place that touches raw pointers or a handle table.
//! Tests in this module call `Engine::load` / `Engine::advance` directly, on
//! hand-built `Program`s, the same way `abi.rs` will after parsing bytes.

use crate::error::BoError;
use crate::lu;
use crate::program::{DeviceKind, Program};

/// Every node except node 0 carries this conductance to the datum (contract,
/// "W0 kernel semantics", point 2): it keeps an isolated net solvable (reads
/// 0 V) and its current is folded into every KCL sum below, exactly as the
/// classic engine stamps it on every node row.
pub const NODE_RSHUNT_G: f64 = 1e-12;

/// `bo_engine_advance` rejects a `t_end` this far or more behind the current
/// time (contract, point 6). A plain `t_end < time` would re-reject a caller
/// re-issuing the same `t_end` it just advanced to, purely from float noise.
const TIME_EPS: f64 = 1e-15;

pub const STAT_PARTITIONS: u32 = 0;
pub const STAT_STATIC_SOLVES: u32 = 1;
pub const STAT_LARGEST_PARTITION: u32 = 2;
pub const STAT_FIXED_NODES: u32 = 3;

/// One entry per source that fixed a node during propagation, in the order
/// it was resolved. `parent_node` was already fixed at the moment `source`
/// (a device index) fixed `child_node`; `parent_is_pos` says whether
/// `parent_node` is that source's `pins[0]` (+) or `pins[1]` (-). Recorded
/// so the current pass can walk the propagation forest children-before-
/// parents (contract, point 5) by simply reversing this list.
struct ResolvedSource {
    device_index: usize,
    parent_node: usize,
    child_node: usize,
    parent_is_pos: bool,
}

#[derive(Debug)]
pub struct Engine {
    node_count: usize,
    device_count: usize,
    voltages: Vec<f64>,
    currents: Vec<f64>,
    time: f64,
    stats: [f64; 4],
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
    /// arrive in (a plain "always point a at b" union, as W0 originally had,
    /// degenerates to a straight-line chain when resistors are listed in
    /// node order — see `find`'s doc comment).
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

impl Engine {
    pub fn node_count(&self) -> u32 {
        self.node_count as u32
    }

    pub fn device_count(&self) -> u32 {
        self.device_count as u32
    }

    pub fn time(&self) -> f64 {
        self.time
    }

    pub fn voltages(&self) -> &[f64] {
        &self.voltages
    }

    pub fn currents(&self) -> &[f64] {
        &self.currents
    }

    pub fn stat(&self, i: u32) -> f64 {
        self.stats.get(i as usize).copied().unwrap_or(f64::NAN)
    }

    /// Advances to `t_end` (absolute seconds). W0 is entirely static, so
    /// this only moves the clock (contract, point 6) — there is nothing yet
    /// that a change in time could invalidate.
    pub fn advance(&mut self, t_end: f64) -> Result<(), BoError> {
        // Checked before the backwards-time comparison because every
        // comparison against NaN is false: without this, advance(NaN) would
        // fall through, get accepted, and turn the clock itself into NaN —
        // after which every future comparison against it is also false, so
        // the backwards-time guard would never fire again. +Infinity would
        // similarly get "accepted" and then reject every finite advance
        // after it. Neither is a real point in time, so both are -6 here
        // rather than becoming the new clock value.
        if !t_end.is_finite() {
            return Err(BoError::time_backwards("t_end must be finite".to_string()));
        }
        if t_end < self.time - TIME_EPS {
            return Err(BoError::time_backwards(format!(
                "advance({t_end}) is before the current time {}",
                self.time
            )));
        }
        self.time = t_end;
        Ok(())
    }

    /// Loads a program from scratch: fixed-node derivation, partitioning,
    /// one dense LU solve per partition, then the KCL current pass. Never
    /// panics on a well-formed `Program` — `Program::parse` is what turns
    /// malformed bytes into `BoError` before this ever runs.
    pub fn load(program: &Program) -> Result<Engine, BoError> {
        let n = program.node_count;

        let (fixed, resolved) = derive_fixed_nodes(program, n)?;

        let mut uf = UnionFind::new(n);
        for device in &program.devices {
            if device.kind != DeviceKind::Resistor {
                continue;
            }
            let (u, w) = (device.pins[0], device.pins[1]);
            if fixed[u].is_none() && fixed[w].is_none() {
                uf.union(u, w);
            }
        }

        // Group free nodes into partitions by union-find representative,
        // preserving first-seen order for determinism (node id order, not
        // representative id order — the latter depends on union direction).
        // Node ids are already a dense `0..n` range, so a representative ->
        // partition-index lookup is a plain indexed `Vec`, not a hash map.
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

        // node -> (partition index, local index within that partition), for
        // free nodes only; `None` for a fixed node.
        let mut local_index: Vec<Option<(usize, usize)>> = vec![None; n];
        for (p_idx, nodes) in partitions.iter().enumerate() {
            for (l_idx, &node) in nodes.iter().enumerate() {
                local_index[node] = Some((p_idx, l_idx));
            }
        }

        let mut matrices: Vec<Vec<f64>> = partitions.iter().map(|p| vec![0.0; p.len() * p.len()]).collect();
        let mut rhs: Vec<Vec<f64>> = partitions.iter().map(|p| vec![0.0; p.len()]).collect();

        for (p_idx, nodes) in partitions.iter().enumerate() {
            let size = nodes.len();
            for (l_idx, _) in nodes.iter().enumerate() {
                matrices[p_idx][l_idx * size + l_idx] += NODE_RSHUNT_G;
            }
        }

        for device in &program.devices {
            if device.kind != DeviceKind::Resistor {
                continue;
            }
            let (u, w) = (device.pins[0], device.pins[1]);
            let g = 1.0 / device.params[0];
            let free_u = local_index[u];
            let free_w = local_index[w];
            match (free_u, free_w) {
                (Some((pu, lu_)), Some((pw, lw))) => {
                    // Both free, so both must land in the same partition —
                    // the union-find pass above joined every free-free
                    // resistor's two ends before partitions were formed.
                    debug_assert_eq!(pu, pw);
                    let size = partitions[pu].len();
                    let a = &mut matrices[pu];
                    a[lu_ * size + lu_] += g;
                    a[lw * size + lw] += g;
                    a[lu_ * size + lw] -= g;
                    a[lw * size + lu_] -= g;
                }
                (Some((pu, lu_)), None) => {
                    let size = partitions[pu].len();
                    matrices[pu][lu_ * size + lu_] += g;
                    rhs[pu][lu_] += g * fixed[w].unwrap();
                }
                (None, Some((pw, lw))) => {
                    let size = partitions[pw].len();
                    matrices[pw][lw * size + lw] += g;
                    rhs[pw][lw] += g * fixed[u].unwrap();
                }
                (None, None) => {
                    // Both ends fixed: no stamp. Its current is Ohm's law
                    // from two already-known voltages (below), not a solve.
                }
            }
        }

        let mut voltages = vec![0.0; n];
        for (node, v) in fixed.iter().enumerate() {
            if let Some(v) = v {
                voltages[node] = *v;
            }
        }

        let mut largest_partition = 0usize;
        for (p_idx, nodes) in partitions.iter().enumerate() {
            let size = nodes.len();
            largest_partition = largest_partition.max(size);
            let a = std::mem::take(&mut matrices[p_idx]);
            let b = std::mem::take(&mut rhs[p_idx]);
            let x = lu::solve(a, size, b).map_err(|_| {
                BoError::solve_failed(format!(
                    "partition {p_idx} ({size} node(s)): singular or non-finite pivot"
                ))
            })?;
            for (l_idx, &node) in nodes.iter().enumerate() {
                voltages[node] = x[l_idx];
            }
        }

        let currents = compute_currents(program, n, &voltages, &resolved);

        // Every finite pivot only rules out a singular partition — it says
        // nothing about the fixed-voltage propagation (plain f64 addition,
        // engine.rs `derive_fixed_nodes`) or the right-hand side and currents
        // above, any of which can overflow to +-Infinity or NaN while every
        // pivot along the way stayed finite. The contract's -4 covers that
        // too ("solve failed: singular or non-finite"), so a load whose
        // published values are not all finite is a failure, not a silent
        // Infinity/NaN readout, and the first offending node or device is
        // named so the caller does not have to scan for it.
        if let Some((node, v)) = voltages.iter().enumerate().find(|(_, v)| !v.is_finite()) {
            return Err(BoError::solve_failed(format!(
                "node {node}: published voltage {v} is not finite"
            )));
        }
        if let Some((device, i)) = currents.iter().enumerate().find(|(_, i)| !i.is_finite()) {
            return Err(BoError::solve_failed(format!(
                "device {device}: published current {i} is not finite"
            )));
        }

        let fixed_count = fixed.iter().filter(|v| v.is_some()).count();
        let stats = [
            partitions.len() as f64,
            partitions.len() as f64, // W0 solves every partition exactly once, at load.
            largest_partition as f64,
            fixed_count as f64,
        ];

        Ok(Engine {
            node_count: n,
            device_count: program.devices.len(),
            voltages,
            currents,
            time: 0.0,
            stats,
        })
    }
}

/// Fixed-point propagation from node 0 (contract, point 1). Returns each
/// node's fixed voltage (`None` for free nodes) and the list of sources that
/// did the fixing, in resolution order. `ERR_UNSUPPORTED_TOPOLOGY` (-3) for
/// a loop of ideal sources or a floating one.
fn derive_fixed_nodes(program: &Program, n: usize) -> Result<(Vec<Option<f64>>, Vec<ResolvedSource>), BoError> {
    let mut fixed: Vec<Option<f64>> = vec![None; n];
    fixed[0] = Some(0.0);

    let source_indices: Vec<usize> = program
        .devices
        .iter()
        .enumerate()
        .filter(|(_, d)| d.kind == DeviceKind::VoltageSource)
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
            let (pos, neg) = (device.pins[0], device.pins[1]);
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
                // topology errors are reported once, after propagation
                // truly stalls, not from an order-dependent snapshot.
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
        let (pos, neg) = (device.pins[0], device.pins[1]);
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

/// Resistor currents by Ohm's law, then source currents by the children-
/// before-parents KCL walk (contract, point 5).
fn compute_currents(program: &Program, n: usize, voltages: &[f64], resolved: &[ResolvedSource]) -> Vec<f64> {
    let mut currents = vec![0.0; program.devices.len()];
    let mut leaving = vec![0.0; n];

    for (d, device) in program.devices.iter().enumerate() {
        if device.kind != DeviceKind::Resistor {
            continue;
        }
        let (u, w) = (device.pins[0], device.pins[1]);
        let r = device.params[0];
        let i = (voltages[u] - voltages[w]) / r;
        currents[d] = i;
        leaving[u] += i;
        leaving[w] -= i;
    }

    // Shunt current, every node but the datum (contract, point 2).
    for (node, v) in voltages.iter().enumerate().skip(1) {
        leaving[node] += NODE_RSHUNT_G * v;
    }

    let mut extra = vec![0.0; n];
    for r in resolved.iter().rev() {
        let total = leaving[r.child_node] + extra[r.child_node];
        let child_is_pos = !r.parent_is_pos;
        let i_s = if child_is_pos { -total } else { total };
        currents[r.device_index] = i_s;

        let contribution = if r.parent_is_pos { i_s } else { -i_s };
        extra[r.parent_node] += contribution;
    }

    currents
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::{ERR_SOLVE_FAILED, ERR_TIME_BACKWARDS, ERR_UNSUPPORTED_TOPOLOGY};
    use crate::program::Device;

    fn resistor(component_index: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: DeviceKind::Resistor, component_index, pins: vec![a, b], params: vec![r], blob: vec![] }
    }

    fn source(component_index: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: DeviceKind::VoltageSource, component_index, pins: vec![pos, neg], params: vec![v], blob: vec![] }
    }

    fn program(node_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, devices }
    }

    #[test]
    fn divider_reads_5v_at_the_midpoint_with_correct_currents() {
        // node0 = datum, node1 = source +, node2 = midpoint.
        let p = program(3, vec![
            source(0, 1, 0, 10.0),
            resistor(1, 1, 2, 1000.0),
            resistor(2, 2, 0, 1000.0),
        ]);
        let e = Engine::load(&p).unwrap();
        assert!((e.voltages()[2] - 5.0).abs() < 1e-6, "got {}", e.voltages()[2]);
        assert!((e.currents()[1] - 0.005).abs() < 1e-6, "R1 current {}", e.currents()[1]);
        assert!((e.currents()[2] - 0.005).abs() < 1e-6, "R2 current {}", e.currents()[2]);
        // Current into the source's + terminal is negative while it delivers
        // power: -5 mA.
        assert!((e.currents()[0] - (-0.005)).abs() < 1e-6, "source current {}", e.currents()[0]);
    }

    #[test]
    fn source_fixing_its_negative_terminal() {
        // pins = (datum +, node1 -): node1 ends up at -V.
        let p = program(2, vec![source(0, 0, 1, 7.0)]);
        let e = Engine::load(&p).unwrap();
        assert!((e.voltages()[1] - (-7.0)).abs() < 1e-9);
    }

    #[test]
    fn chained_sources_with_a_load_on_each_node_have_kcl_correct_currents() {
        // datum -> A at 5V -> B at 5+3=8V, with a 1k load from A and B each
        // to the datum, so both loads pull from A's own branch.
        let p = program(3, vec![
            source(0, 1, 0, 5.0),  // fixes node1 (A) = 5V
            source(1, 2, 1, 3.0),  // fixes node2 (B) = A + 3 = 8V
            resistor(2, 1, 0, 1000.0), // load at A
            resistor(3, 2, 0, 1000.0), // load at B
        ]);
        let e = Engine::load(&p).unwrap();
        assert!((e.voltages()[1] - 5.0).abs() < 1e-9);
        assert!((e.voltages()[2] - 8.0).abs() < 1e-9);

        let i_load_a = e.currents()[2];
        let i_load_b = e.currents()[3];
        assert!((i_load_a - 0.005).abs() < 1e-9);
        assert!((i_load_b - 0.008).abs() < 1e-9);

        // KCL at B: source_b supplies exactly the load at B (B's + terminal
        // is node2, since pins = (2,1)).
        let i_source_b = e.currents()[1];
        assert!((i_source_b - (-i_load_b)).abs() < 1e-6, "source_b {}", i_source_b);

        // KCL at A: source_a supplies A's own load AND everything source_b
        // draws through A (source_b's - terminal sits at A, so source_b
        // pulls +i_source_b out of the circuit at A, i.e. draws -i_source_b
        // from A into itself). A's + terminal is node1 (pins = (1,0)).
        let i_source_a = e.currents()[0];
        let expected = -(i_load_a + (-i_source_b));
        assert!((i_source_a - expected).abs() < 1e-6, "source_a {} expected {}", i_source_a, expected);
    }

    #[test]
    fn two_ideal_sources_in_parallel_is_unsupported_topology() {
        let p = program(2, vec![source(0, 1, 0, 5.0), source(1, 1, 0, 5.0)]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY);
    }

    #[test]
    fn a_source_with_both_terminals_on_the_same_node_is_unsupported_topology() {
        // Shorted to itself, and never fixed by anything else: floating.
        let p = program(2, vec![source(0, 1, 1, 5.0)]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY);
    }

    #[test]
    fn a_source_shorted_to_itself_after_being_otherwise_fixed_is_unsupported_topology() {
        // node1 gets fixed by a normal source, then a second source shorted
        // node1-to-node1 finds both its terminals already fixed.
        let p = program(2, vec![source(0, 1, 0, 5.0), source(1, 1, 1, 3.0)]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY);
    }

    #[test]
    fn a_floating_source_is_unsupported_topology() {
        // node1/node2 are a pair, but neither ever reaches the datum.
        let p = program(3, vec![source(0, 1, 2, 5.0)]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY);
    }

    #[test]
    fn an_isolated_net_with_no_elements_reads_zero_volts_and_does_not_fail() {
        let p = program(2, vec![]);
        let e = Engine::load(&p).unwrap();
        assert_eq!(e.voltages()[1], 0.0);
        assert_eq!(e.stat(STAT_PARTITIONS), 1.0);
    }

    #[test]
    fn two_independent_partitions_solve_separately() {
        let p = program(5, vec![
            source(0, 1, 0, 10.0),
            resistor(1, 1, 2, 1000.0),
            resistor(2, 2, 0, 1000.0),
            source(3, 3, 0, 20.0),
            resistor(4, 3, 4, 500.0),
            resistor(5, 4, 0, 1500.0),
        ]);
        let e = Engine::load(&p).unwrap();
        assert!((e.voltages()[2] - 5.0).abs() < 1e-6);
        assert!((e.voltages()[4] - 15.0).abs() < 1e-6);
        assert_eq!(e.stat(STAT_PARTITIONS), 2.0);
    }

    #[test]
    fn a_resistor_with_both_ends_fixed_has_a_computed_current_and_no_stamp() {
        let p = program(3, vec![
            source(0, 1, 0, 10.0),
            source(1, 2, 0, 4.0),
            resistor(2, 1, 2, 1000.0),
        ]);
        let e = Engine::load(&p).unwrap();
        // No free nodes at all: zero partitions.
        assert_eq!(e.stat(STAT_PARTITIONS), 0.0);
        assert!((e.currents()[2] - 0.006).abs() < 1e-9);
    }

    #[test]
    fn stats_report_partitions_solves_largest_partition_and_fixed_count() {
        let p = program(4, vec![
            source(0, 1, 0, 10.0),
            resistor(1, 1, 2, 1000.0),
            resistor(2, 2, 3, 1000.0),
            resistor(3, 3, 0, 1000.0),
        ]);
        let e = Engine::load(&p).unwrap();
        assert_eq!(e.stat(STAT_PARTITIONS), 1.0);
        assert_eq!(e.stat(STAT_STATIC_SOLVES), 1.0);
        assert_eq!(e.stat(STAT_LARGEST_PARTITION), 2.0);
        assert_eq!(e.stat(STAT_FIXED_NODES), 2.0); // node0 + node1
        assert!(e.stat(99).is_nan());
    }

    #[test]
    fn advance_rejects_moving_time_backwards_and_accepts_forward_motion() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        e.advance(1.0).unwrap();
        assert_eq!(e.time(), 1.0);
        let err = e.advance(0.5).unwrap_err();
        assert_eq!(err.code, -6);
        e.advance(2.0).unwrap();
        assert_eq!(e.time(), 2.0);
    }

    #[test]
    fn advance_rejects_non_finite_t_end_instead_of_poisoning_the_clock() {
        // Before this fix: `t_end < self.time - TIME_EPS` is false for every
        // comparison against NaN, so NaN was accepted and became the new
        // clock — after which every future comparison against it was also
        // false, permanently disabling the backwards-time guard. +Infinity
        // was accepted too, then rejected every later finite advance as
        // "backwards". Neither is a real point in time.
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        e.advance(1.0).unwrap();

        let err = e.advance(f64::NAN).unwrap_err();
        assert_eq!(err.code, ERR_TIME_BACKWARDS);
        assert_eq!(err.message, "t_end must be finite");
        assert_eq!(e.time(), 1.0, "a rejected advance must not touch the clock");

        let err = e.advance(f64::INFINITY).unwrap_err();
        assert_eq!(err.code, ERR_TIME_BACKWARDS);
        assert_eq!(e.time(), 1.0);

        // The clock must still be usable afterwards.
        e.advance(2.0).unwrap();
        assert_eq!(e.time(), 2.0);
    }

    #[test]
    fn load_fails_with_solve_failed_when_a_published_voltage_overflows_to_infinity() {
        // Two 1e308 V sources chained in series: fixed-node propagation adds
        // their voltages in plain f64 arithmetic (1e308 + 1e308 overflows a
        // finite double), and no LU pivot is ever involved, so the old
        // pivot-only check let this through as a converged load with an
        // Infinity readout.
        let p = program(3, vec![
            source(0, 1, 0, 1e308),
            source(1, 2, 1, 1e308),
        ]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_SOLVE_FAILED);
        assert!(err.message.contains("node 2"), "message was: {}", err.message);
    }

    #[test]
    fn load_fails_with_solve_failed_when_a_published_current_is_non_finite() {
        // A hand-built both-fixed resistor with R = 0 (the reader now
        // rejects this in a real program, but `Engine::load` is tested
        // directly on hand-built `Program`s here, the same way `abi.rs` will
        // call it after parsing). Both terminals are fixed, so this resistor
        // is never stamped into a matrix — its current is Ohm's law on two
        // already-known, finite voltages, dividing by zero. Listed first
        // (device 0) so it is unambiguously the "first offending device" the
        // message names, rather than a source whose current only went
        // non-finite by KCL propagation from this same resistor.
        let p = program(3, vec![
            resistor(0, 1, 2, 0.0),
            source(1, 1, 0, 10.0),
            source(2, 2, 0, 4.0),
        ]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_SOLVE_FAILED);
        assert!(err.message.contains("device 0"), "message was: {}", err.message);
    }

    #[test]
    fn union_find_handles_a_million_node_chain_without_recursing() {
        // The historical bug: a recursive `find` spends one call-stack frame
        // per link in the parent chain, and a plain "always point a at b"
        // union with no size heuristic turns a chain of unions listed in
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

    #[test]
    fn load_solves_a_long_series_chain_without_stack_overflow() {
        // The end-to-end counterpart of the UnionFind test above, through the
        // real `Engine::load` path (parse -> fixed-node derivation ->
        // partitioning -> solve), where the old recursive `find` actually
        // lived (the partition-grouping loop's first `find` call walked the
        // whole chain). A chain anywhere near the 1,000,000-node ceiling
        // needs a dense partition matrix of many gigabytes, which a native
        // test cannot allocate; this uses a chain long enough to exercise the
        // same code path with a real solve, at a size the dense LU can
        // actually factor in a test.
        const CHAIN_NODES: usize = 800; // nodes 1..=CHAIN_NODES, one free partition
        let mut devices = Vec::with_capacity(CHAIN_NODES - 1);
        for i in 1..CHAIN_NODES {
            devices.push(resistor(i, i, i + 1, 1000.0));
        }
        let p = program(CHAIN_NODES + 1, devices);
        let e = Engine::load(&p).unwrap();
        assert_eq!(e.stat(STAT_PARTITIONS), 1.0);
        assert_eq!(e.stat(STAT_LARGEST_PARTITION), CHAIN_NODES as f64);
        for (node, v) in e.voltages().iter().enumerate() {
            assert!(v.is_finite(), "node {node} voltage was not finite: {v}");
        }
    }
}
