//! A load-time guard for a bench supply's current-limit rail the core reads
//! wrong (mechanism M3 of the current-limit diagnosis, ship-logs/ccd and
//! ship-logs/fixm3).
//!
//! A default-mode `bench_psu` hub whose rail carries a capacitor to a fixed
//! node and an esr-0 capacitor into a STEPPED hub partition, closing an esr-0
//! capacitor loop through the rail, beside a CLOSED-FORM hub partition on the
//! rail, is held in current limit with the stepped partition's draw in its
//! frozen form: its `g` is a capacitor series conductance (`C / h`, 6.9e5 S on
//! the minimal board), so the segment aims the rail at `dl / g`, microvolts,
//! and the charge sent in goes nowhere. The rail reads up to volts off and a
//! capacitor current of millions of amps is published at a re-seed. The
//! proper fix (stepping the rail capacitors with the partition) is not
//! landed; until it is, such a board must not run on the core. Classic does it
//! right (ship-logs/guard3/NOTES.md).
//!
//! The test is the engine's own partition build, in four parts: the supply is
//! the hub rail and the rail is not free (`NO_RAIL_SHORTCUT` frees it: exact
//! mode steps the rail with its partitions and reads these boards right, so
//! the guard does not apply there); (1) a capacitor from the rail to a fixed
//! node; (2) an esr-0 capacitor from the rail into a stepped hub partition;
//! (3) a closed-form hub partition; (4) an esr-0 capacitor loop through the
//! rail (union-find over the esr-0 capacitors with every fixed node but the
//! rail merged with ground, the form FIXM3 measured, ship-logs/fixm3/h
//! `m3_structure`).
//!
//! What the broad form did wrong: it asked only for a rail capacitor, any
//! stepped hub partition and a closed-form one. That declined boards the core
//! reads right, among them the stepped variant of
//! `an_rc_driven_by_an_ic_across_its_power_floor_follows_the_coupled_closed_form`
//! (`rail_tests`), a supply in current limit the core reads within 1e-4 V of
//! the coupled closed form and classic's default (backward Euler) reads 3.4 W0
//! off it. The guard must never send a board the core reads right to a reader
//! that reads it worse, so it is narrow.
//!
//! What the narrow form misses (KNOWN GAPS, waiting for the proper fix): on
//! the 4,200-board bench-supply sweep it catches 189/5, 333/5 and 474/5, where
//! classic's default reads far closer to exact rail mode than the core does,
//! and misses 274/3 and 418/5, which the core also reads wrong.

use super::CC_RAIL_REASON;
#[cfg(test)]
use std::cell::Cell;
use crate::kinds;
use crate::partition::{PartitionBuild, PartitionKind};
use crate::program::Program;

#[cfg(test)]
thread_local! { static OFF: Cell<bool> = const { Cell::new(false) }; }

/// Test-only seam: runs `f` with this thread's guard switched off. The
/// failing-by-design current-limit tests (`cc_truth_tests.rs`) judge what the
/// core itself reads on the boards the guard declines; without this their
/// `Engine::load` would return the decline and the tests would fail for
/// "declined", not for the reading they state. The guard's own tests and
/// `cc_truth_tests`'s companion test (the guard declines those boards in
/// default mode) keep the guard on.
#[cfg(test)]
pub(super) fn without_guard<T>(f: impl FnOnce() -> T) -> T {
    let before = OFF.with(|o| o.replace(true));
    let out = f();
    OFF.with(|o| o.set(before));
    out
}

/// The decline message when `program`'s build has the board described above,
/// else `None`.
pub(super) fn decline(program: &Program, pb: &PartitionBuild) -> Option<String> {
    #[cfg(test)]
    if OFF.with(Cell::get) {
        return None;
    }
    let rail = pb.rail.as_ref().filter(|t| !t.exact && program.devices[t.device].kind == kinds::BENCH_PSU)?;
    let r = rail.rail;
    let fixed_to = |n: usize| n != r && pb.fixed[n].is_some();
    // (node a, node b, esr) of every capacitor that joins two nodes.
    let caps: Vec<(usize, usize, f64)> = program
        .devices
        .iter()
        .filter(|d| d.kind == kinds::CAPACITOR && d.pins[0].node != d.pins[1].node)
        .map(|d| (d.pins[0].node, d.pins[1].node, d.params[1]))
        .collect();
    // (1) a capacitor from the rail to a fixed node.
    if !caps.iter().any(|&(a, b, _)| (a == r && fixed_to(b)) || (b == r && fixed_to(a))) {
        return None;
    }
    // (2) an esr-0 capacitor from the rail into a stepped hub partition.
    let hub_of = |kind: PartitionKind| (0..pb.kinds.len()).any(|p| pb.hub[p] && pb.kinds[p] == kind);
    let part = |n: usize| pb.local_index[n].map(|(p, _)| p);
    let into_stepped = caps.iter().any(|&(a, b, esr)| {
        let other = if a == r { b } else if b == r { a } else { return false };
        esr == 0.0 && part(other).is_some_and(|p| pb.hub[p] && pb.kinds[p] == PartitionKind::Stepped)
    });
    // (3) a closed-form hub partition.
    if !into_stepped || !hub_of(PartitionKind::ClosedForm) {
        return None;
    }
    // (4) an esr-0 capacitor loop through the rail: union-find over the esr-0
    // capacitors, every fixed node but the rail merged with ground (node 0).
    let rep = |x: usize| if fixed_to(x) { 0 } else { x };
    let mut uf: Vec<usize> = (0..program.node_count).collect();
    fn find(uf: &mut [usize], mut x: usize) -> usize {
        while uf[x] != x {
            x = uf[x];
        }
        x
    }
    let mut closers = Vec::new();
    for &(a, b, esr) in &caps {
        if esr != 0.0 {
            continue;
        }
        let (a, b) = (rep(a), rep(b));
        if a == b {
            continue;
        }
        let (ra, rb) = (find(&mut uf, a), find(&mut uf, b));
        if ra == rb {
            closers.push(a);
        } else {
            uf[ra] = rb;
        }
    }
    let rail_root = find(&mut uf, r);
    if !closers.into_iter().any(|a| find(&mut uf, a) == rail_root) {
        return None;
    }
    Some(format!(
        "{CC_RAIL_REASON}: bench_psu (device {}) has a rail capacitor (one from its rail to a fixed node) and an esr-0 capacitor loop through the rail into a stepped capacitor branch, beside a closed-form capacitor branch; the core does not yet read such a supply right in current limit (the rail can read volts off and a capacitor current of millions of amps), so the circuit runs on the classic engine, which does (exact rail mode, `noRailShortcut`, is not declined)",
        rail.device
    ))
}
