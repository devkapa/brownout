//! Tests of `PartitionedSolver`'s rollback window (`begin_instant`,
//! `end_instant`, `rollback_instant`) and of what the publish path keeps in
//! step with it (the readout bounds, the copy-in from the static solver).
//!
//! The window keeps undo logs and journals, not copies, so what these tests
//! pin down is completeness: that after a rollback the solver is bit for bit
//! where it was, in every piece of state a doomed instant can touch, and that
//! it then goes on exactly as a twin that never ran the instant. Test builds
//! also check every window that any test opens (the `w_saved` oracles in the
//! solvers): a slot or partition changed without being recorded fails the
//! test that ran it.

use super::*;
use crate::drivers::{DelaySet, Driver};
use crate::events::Mode::{High, Low};
use crate::kinds::ic_param;
use crate::events::DeviceId;
use crate::program::{Device, Options, PinRecord, PIN_FLAG_OPEN};

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
    Device { kind: kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
}

fn capacitor(ci: usize, a: usize, b: usize, c: f64, esr: f64, leak: f64) -> Device {
    Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![c, esr, leak], blob: vec![] }
}

fn switch_dev(ci: usize, a: usize, b: usize, closed: f64) -> Device {
    Device { kind: kinds::SWITCH, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![closed], blob: vec![] }
}

fn led(ci: usize, anode: usize, cathode: usize) -> Device {
    Device { kind: kinds::LED, component_index: ci, pins: vec![pin(anode), pin(cathode)], params: vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0], blob: vec![] }
}

/// The device indices of [`composite`], for the tests that name one.
const D_SWITCH: usize = 13;

/// One program with every kind of state a rollback has to bring back:
/// - a fixed 5 V rail (node 1) and a 74ls04 (device 1) whose `1y` drives an RC
///   node (2, a closed-form partition, and read by the IC's own `2a`, so a
///   crossing watch is registered on it), whose `2y` drives the rail and whose
///   `1a` is an open floating pull on it;
/// - a 555 (device 4) whose OUT and DIS drive two static nodes;
/// - a two-capacitor ladder (nodes 5 and 6, a stepped partition);
/// - a static divider with a switch (node 7) that also feeds an RGB LED
///   (channels, and an iterated static partition);
/// - an LED behind a resistor (node 8, another one);
/// - a fixed-pair capacitor.
///
/// Devices: 0 source, 1 ls04, 2 R, 3 C, 4 ne555, 5 and 6 its loads, 7 fixed
/// pair C, 8-11 the ladder, 12 R, 13 the switch, 14 R, 15 R, 16 LED, 17 RGB.
fn composite(open_hash: u32) -> Program {
    let flags = |i: usize| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 };
    let mut ls04_pins = vec![pin(1), pin(0)];
    for (i, node) in [1usize, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0].into_iter().enumerate() {
        ls04_pins.push(if i == 0 { PinRecord { node, flags: PIN_FLAG_OPEN, open_hash, dead_hash: 0 } } else { pin(node) });
    }
    let mut ls04_params = vec![0.0; ic_param::COUNT];
    ls04_params[ic_param::VCC_MIN] = 4.75;
    ls04_params[ic_param::VCC_MAX] = 5.25;
    ls04_params[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    ls04_params[ic_param::VIL_FIXED] = 1.0;
    ls04_params[ic_param::VIL] = 0.8;
    ls04_params[ic_param::VIH_FIXED] = 1.0;
    ls04_params[ic_param::VIH] = 2.0;
    ls04_params[ic_param::NOMINAL] = 5.0;
    ls04_params[ic_param::IO_MAX] = 0.008;
    ls04_params[ic_param::DELAY] = 20e-9;
    let mut ls04_blob = vec![0.0];
    ls04_blob.extend((0..12).map(flags));
    let ls04 = Device { kind: kinds::IC_74LS04, component_index: 1, pins: ls04_pins, params: ls04_params, blob: ls04_blob };

    let mut ne555_params = vec![0.0; ic_param::COUNT];
    ne555_params[ic_param::VCC_MIN] = 2.0;
    ne555_params[ic_param::VCC_MAX] = 18.0;
    ne555_params[ic_param::NOMINAL] = 5.0;
    let ne555_pins = [1, 0, 0, 3, 1, 0, 0, 4].into_iter().map(pin).collect();
    let ne555 = Device { kind: kinds::NE555, component_index: 4, pins: ne555_pins, params: ne555_params, blob: vec![0.0; 7] };

    let rgb = Device {
        kind: kinds::RGB_LED,
        component_index: 17,
        pins: vec![pin(7), pin(7), pin(7), pin(0)],
        params: vec![2.0, 2.1, 3.0, 0.02, 0.03, 0.0],
        blob: vec![0.0],
    };

    Program {
        carry: Vec::new(),
        node_count: 9,
        options: Options::default(),
        digital_slot_count: 0,
        channel_slot_count: 3,
        devices: vec![
            source(0, 1, 0, 5.0),
            ls04,
            resistor(2, 2, 0, 1000.0),
            capacitor(3, 2, 0, 1e-6, 0.0, 0.0),
            ne555,
            resistor(5, 3, 0, 1000.0),
            resistor(6, 4, 1, 10000.0),
            capacitor(7, 1, 0, 1e-6, 0.5, 1e7),
            resistor(8, 1, 5, 1000.0),
            capacitor(9, 5, 0, 1e-6, 0.0, 0.0),
            resistor(10, 5, 6, 2000.0),
            capacitor(11, 6, 0, 2e-6, 0.0, 0.0),
            resistor(12, 1, 7, 1000.0),
            switch_dev(D_SWITCH, 7, 0, 0.0),
            resistor(14, 7, 0, 4700.0),
            resistor(15, 1, 8, 330.0),
            led(16, 8, 0),
            rgb,
        ],
    }
}

/// The composite's driver table: the ls04's `1y` (node 2) and `2y` (node 1),
/// the 555's OUT (node 3) and DIS (node 4).
fn composite_drivers() -> Vec<Driver> {
    let mut out = Driver::new_ic_output(3, 1, 0, 4, false);
    out.stage = DriverStage::Ne555Out;
    let mut dis = Driver::new_ic_output(4, 1, 0, 4, false);
    dis.stage = DriverStage::Ne555Dis;
    vec![Driver::new_ic_output(2, 1, 0, 1, false), Driver::new_ic_output(1, 1, 0, 1, false), out, dis]
}

fn commit_levels(drivers: &mut [Driver], levels: [Mode; 4], now: f64) {
    for (d, level) in drivers.iter_mut().zip(levels) {
        if d.committed() != Some(level) {
            let (_, mode, token) = d.request(level, now, &DelaySet::uniform(0.0)).expect("a level change schedules a commit");
            assert!(d.apply_commit(mode, token));
        }
    }
}

fn everything(s: &PartitionedSolver) -> Vec<PartitionId> {
    (0..s.partition_count()).map(|p| PartitionId(p as u32)).collect()
}

/// Every published number, as bits, after bringing the readout up to date.
fn published_bits(s: &mut PartitionedSolver) -> Vec<u64> {
    s.prepare_readout();
    s.voltages().iter().chain(s.currents()).chain(s.channels()).map(|v| v.to_bits()).collect()
}

/// The solver's state in named pieces, from the whole-copy snapshots (the
/// reference the window replaces): what a rollback has to bring back.
fn pieces(s: &PartitionedSolver) -> Vec<(&'static str, String)> {
    let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
    let (voltages, currents) = s.snapshot();
    let dynamic = s.dynamic_snapshot().downcast::<(DynamicSnapshot, super::hub::W1fSnapshot)>().expect("PartitionedSolver's dynamic snapshot");
    let ((static_snapshot, closed, stepped, pending, next_bucket, drivers, channels, dirty, dirty_now), w1f) = *dynamic;
    vec![
        ("voltages", format!("{:?}", bits(&voltages))),
        ("currents", format!("{:?}", bits(&currents))),
        // Its source currents are never read (the solver computes its own), so
        // it leaves them stale while it skips their walk and refreshes them
        // when it does not (after a rollback): not part of the state.
        ("static solver", {
            let mut currents = bits(&static_snapshot.1);
            for r in &s.resolved {
                currents[r.device_index] = 0;
            }
            format!("{:?} {currents:?}", bits(&static_snapshot.0))
        }),
        ("closed-form", closed.digest()),
        ("stepped", stepped.digest()),
        ("pending schedule", format!("{pending:?}")),
        ("bucket", format!("{next_bucket:?}")),
        // What the solver reads of a driver: not the pending transition or the
        // token, which a solver told of each commit (`Wired`) leaves as they
        // were when it last refreshed that entry.
        ("driver table", format!("{:?}", drivers.iter().map(|d| (d.node, d.hi_node, d.lo_node, d.device, d.stage, d.committed())).collect::<Vec<_>>())),
        ("channels", format!("{:?}", bits(&channels))),
        ("readout flags", format!("{dirty} {}", dirty_now.to_bits())),
        ("rails and sources", w1f.digest()),
    ]
}

/// Asserts two solvers' states are the same, naming the first piece that is not.
fn assert_same(a: &PartitionedSolver, b: &PartitionedSolver, what: &str) {
    for ((name, x), (_, y)) in pieces(a).iter().zip(pieces(b).iter()) {
        assert!(x == y, "{what}: the {name} differs\n left: {x}\nright: {y}");
    }
}

/// Applies every crossing and step the solver has queued that falls at or
/// before `until`, as the kernel would, and returns how many were current.
fn deliver(s: &mut PartitionedSolver, drivers: &[Driver], until: f64) -> usize {
    let mut queued = s.take_scheduled();
    queued.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut applied = 0;
    for (at, kind) in queued {
        if at > until {
            continue;
        }
        let _ = drivers;
        match kind {
            EventKind::Crossing { partition, version } if s.is_current(partition, true, version) => {
                s.handle_dynamic_event(partition, at, true).unwrap();
                applied += 1;
            }
            EventKind::Step { partition, version } if s.is_current(partition, false, version) => {
                s.handle_dynamic_event(partition, at, false).unwrap();
                applied += 1;
            }
            _ => {}
        }
    }
    applied
}

/// A pair of a solver and its driver table, built from `program`.
struct Rig {
    s: PartitionedSolver,
    drivers: Vec<Driver>,
}

impl Rig {
    fn new(p: &Program, eager: bool) -> Rig {
        let mut s = PartitionedSolver::build(p).unwrap();
        if eager {
            s.lazy_static_ok = false;
        }
        Rig { s, drivers: composite_drivers() }
    }

    fn resolve(&mut self, dirty: &[PartitionId], t: f64) {
        self.s.resolve_dirty(dirty, t, &self.drivers).unwrap();
    }

    fn resolve_all(&mut self, t: f64) {
        let all = everything(&self.s);
        self.resolve(&all, t);
    }

    fn toggle_switch(&mut self, closed: f64, t: f64) {
        let touched = self.s.apply_params(&[(D_SWITCH, 0, closed)]);
        self.resolve(&touched, t);
    }
}

/// The steps both twins share before the instant that fails, chosen to leave
/// the solver with a live crossing and step queued, a bucket pending and
/// nothing owed in its readout.
fn prefix(r: &mut Rig) {
    r.resolve_all(0.0);
    commit_levels(&mut r.drivers, [High, Low, High, Low], 1e-5);
    r.resolve_all(1e-5);
    r.s.sync_dynamic_to(2e-5).unwrap();
}

/// What the failed instant does, in a window: everything a delta cycle can ask
/// of the solver, more than once where it matters (a second resolve within
/// the instant is the driver table's copy-over-the-other-buffer path).
fn doomed(r: &mut Rig) {
    r.s.take_scheduled();
    commit_levels(&mut r.drivers, [Low, High, Low, High], 3e-5);
    r.resolve_all(3e-5);
    r.toggle_switch(1.0, 3e-5);
    commit_levels(&mut r.drivers, [High, High, Low, Low], 3e-5);
    r.resolve_all(3e-5);
    deliver(&mut r.s, &r.drivers, 1.0);
    r.s.sync_dynamic_to(1e-3).unwrap();
    r.resolve_all(1e-3);
    // The floating bucket boundary of the next 10 ms bucket: the bucket flag
    // moves on, which it does not at an earlier time in the same bucket.
    let _ = r.s.bucket_boundary(1.5e-2);
    r.s.prepare_readout();
}

/// The steps both twins take after, from whatever state the rollback left.
fn continuation(r: &mut Rig) -> Vec<Vec<u64>> {
    let mut log = Vec::new();
    r.toggle_switch(1.0, 5e-5);
    log.push(published_bits(&mut r.s));
    commit_levels(&mut r.drivers, [Low, High, Low, High], 6e-5);
    r.resolve_all(6e-5);
    deliver(&mut r.s, &r.drivers, 1.0);
    log.push(published_bits(&mut r.s));
    r.s.sync_dynamic_to(2e-3).unwrap();
    log.push(published_bits(&mut r.s));
    r.toggle_switch(0.0, 2e-3);
    log.push(published_bits(&mut r.s));
    log
}

fn a_floating_hash() -> u32 {
    // Any hash will do; a fixed one keeps the run repeatable.
    0x1234_5678
}

fn run_a_failed_instant(eager: bool) {
    let p = composite(a_floating_hash());
    let (mut failed, mut twin) = (Rig::new(&p, eager), Rig::new(&p, eager));
    assert_eq!(failed.s.stepped_parts.len(), 1, "the ladder is one stepped partition");
    assert_eq!(failed.s.cf_parts.len(), 1, "the RC node is one closed-form partition");
    prefix(&mut failed);
    prefix(&mut twin);
    assert_same(&failed.s, &twin.s, "after the shared prefix");
    assert!(!failed.s.pending_schedule.is_empty(), "the prefix must leave a crossing and a step queued");
    // The readout is brought up to date first: reading changes what is owed,
    // and this test compares the solver with itself across the instant.
    let bits_before = published_bits(&mut failed.s);
    published_bits(&mut twin.s);
    let before = pieces(&failed.s);
    let table_before = failed.drivers.clone();

    // A control that never rolls back shows the instant really moves every
    // piece it is meant to move.
    let mut control = Rig::new(&p, eager);
    prefix(&mut control);
    published_bits(&mut control.s);
    doomed(&mut control);
    let after = pieces(&control.s);
    for ((name, was), (_, now)) in before.iter().zip(&after) {
        // An eager solver never owes its readout, so that flag never moves.
        if eager && *name == "readout flags" {
            continue;
        }
        // This circuit has no rail and no waveform source; their rollback is
        // `rail_tests`' to prove.
        if *name == "rails and sources" {
            continue;
        }
        assert_ne!(was, now, "the failed instant must change the solver's {name}, or the test proves nothing about restoring it");
    }

    let token = failed.s.begin_instant();
    doomed(&mut failed);
    failed.s.rollback_instant(token);
    failed.drivers = table_before;
    for ((name, was), (_, now)) in before.iter().zip(pieces(&failed.s)) {
        assert!(*was == now, "a rollback must put back every piece of state: the {name} differs");
    }
    assert!(failed.s.static_solver.needs_full_recompute(), "and leave the static solver recomputing everything at its next resolve");
    assert_eq!(published_bits(&mut failed.s), bits_before);

    // Then it goes on exactly as the twin that never had the instant.
    assert_eq!(continuation(&mut failed), continuation(&mut twin));
    assert_same(&failed.s, &twin.s, "after the continuation");
}

#[test]
fn a_rolled_back_instant_leaves_every_piece_of_state_where_it_was_and_the_solver_goes_on_like_a_twin_that_never_ran_it() {
    run_a_failed_instant(false);
}

#[test]
fn a_rolled_back_instant_that_ran_the_eager_readout_puts_back_the_derived_slots_and_channels() {
    run_a_failed_instant(true);
}

#[test]
fn a_completed_instant_is_the_same_as_the_same_calls_outside_any_window() {
    for eager in [false, true] {
        let p = composite(a_floating_hash());
        let (mut windowed, mut plain) = (Rig::new(&p, eager), Rig::new(&p, eager));
        prefix(&mut windowed);
        prefix(&mut plain);
        let token = windowed.s.begin_instant();
        doomed(&mut windowed);
        windowed.s.end_instant(token);
        doomed(&mut plain);
        assert_eq!(pieces(&windowed.s), pieces(&plain.s), "a window must change no result (eager: {eager})");
        assert_eq!(continuation(&mut windowed), continuation(&mut plain));
        // And a window opens again afterwards, on the state it left.
        let token = windowed.s.begin_instant();
        windowed.s.end_instant(token);
    }
}

#[test]
fn a_rollback_taken_while_the_readout_is_owed_and_one_after_a_read_both_restore_it() {
    let p = composite(a_floating_hash());
    for read_first in [false, true] {
        let mut r = Rig::new(&p, false);
        prefix(&mut r);
        r.s.sync_dynamic_to(2.5e-5).unwrap();
        if read_first {
            r.s.prepare_readout();
        }
        assert_eq!(r.s.derived_dirty, !read_first);
        let before = pieces(&r.s);
        let token = r.s.begin_instant();
        r.resolve_all(2.6e-5);
        r.s.prepare_readout();
        r.s.rollback_instant(token);
        assert_eq!(pieces(&r.s), before, "read first: {read_first}");
        assert_eq!(r.s.derived_dirty, !read_first);
    }
}

#[test]
fn a_second_resolve_in_a_window_does_not_write_into_the_table_the_window_holds() {
    let p = composite(a_floating_hash());
    let mut r = Rig::new(&p, false);
    prefix(&mut r);
    let held = r.s.driver_snapshot.iter().map(|d| d.committed()).collect::<Vec<_>>();
    let token = r.s.begin_instant();
    for (i, levels) in [[Low, High, Low, High], [High, High, High, High], [Low, Low, Low, Low]].into_iter().enumerate() {
        commit_levels(&mut r.drivers, levels, 3e-5 + i as f64 * 1e-6);
        r.resolve_all(3e-5 + i as f64 * 1e-6);
        assert_eq!(r.s.driver_snapshot.iter().map(|d| d.committed()).collect::<Vec<_>>(), r.drivers.iter().map(|d| d.committed()).collect::<Vec<_>>());
    }
    r.s.rollback_instant(token);
    assert_eq!(r.s.driver_snapshot.iter().map(|d| d.committed()).collect::<Vec<_>>(), held);
}

// ---- the readout bounds ---------------------------------------------------

/// The exact maxima of |v| over the voltages and |i| over the partition
/// capacitors' currents, by scanning.
fn exact_bounds(s: &PartitionedSolver) -> (f64, f64) {
    let v = s.voltages.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    let i = s.partition_cap_devices.iter().fold(0.0f64, |m, &d| m.max(s.currents[d].abs()));
    (v, i)
}

#[test]
fn every_publish_site_raises_the_bounds_over_what_it_wrote() {
    // With the bounds reset to nothing, each entry point must lift them over
    // the values it wrote (a node the run never touches stays uncovered, so
    // the check is against the nodes and capacitor currents whose value moved).
    // Eager, so that no publish checks the bounds against the whole arrays
    // (a debug build's check in `lazy_publish_ok` would fail the bounds this
    // test empties on purpose); the folds happen at the same sites either way.
    let p = composite(a_floating_hash());
    let mut r = Rig::new(&p, true);
    r.resolve_all(0.0);
    let steps: Vec<(&str, Box<dyn Fn(&mut Rig)>)> = vec![
        ("a static resolve", Box::new(|r: &mut Rig| {
            commit_levels(&mut r.drivers, [Low, High, Low, High], 1e-5);
            r.resolve_all(1e-5);
        })),
        ("a closed-form and stepped re-aim", Box::new(|r: &mut Rig| {
            commit_levels(&mut r.drivers, [High, Low, High, Low], 2e-5);
            r.resolve_all(2e-5);
        })),
        ("a horizon clamp", Box::new(|r: &mut Rig| {
            r.s.sync_dynamic_to(5e-4).unwrap();
        })),
        ("the queued crossings and steps", Box::new(|r: &mut Rig| {
            let applied = deliver(&mut r.s, &r.drivers, 1.0);
            assert!(applied > 0, "something must have been queued");
        })),
    ];
    for (what, step) in steps {
        let (v0, c0) = (r.s.voltages.clone(), r.s.currents.clone());
        r.s.v_hint = 0.0;
        r.s.i_hint = 0.0;
        step(&mut r);
        let moved_v = r.s.voltages.iter().zip(&v0).filter(|(a, b)| a.to_bits() != b.to_bits()).fold(0.0f64, |m, (a, _)| m.max(a.abs()));
        let moved_i = r
            .s
            .partition_cap_devices
            .iter()
            .filter(|&&d| r.s.currents[d].to_bits() != c0[d].to_bits())
            .fold(0.0f64, |m, &d| m.max(r.s.currents[d].abs()));
        assert!(moved_v > 0.0, "{what} must move a voltage");
        assert!(r.s.v_hint >= moved_v, "{what}: the voltage bound {} must cover the largest voltage it wrote, {moved_v}", r.s.v_hint);
        assert!(r.s.i_hint >= moved_i, "{what}: the current bound {} must cover the largest capacitor current it wrote, {moved_i}", r.s.i_hint);
    }
}

#[test]
fn a_rollback_lifts_the_bounds_so_the_next_publish_scans_and_makes_them_exact_again() {
    let p = composite(a_floating_hash());
    let mut r = Rig::new(&p, false);
    prefix(&mut r);
    r.s.rescan_bounds();
    let exact = exact_bounds(&r.s);
    assert_eq!((r.s.v_hint, r.s.i_hint), exact);
    let token = r.s.begin_instant();
    r.resolve_all(3e-5);
    r.s.rollback_instant(token);
    assert!(r.s.v_hint.is_infinite() && r.s.i_hint.is_infinite(), "values from before the bounds were exact are back");
    assert!(r.s.lazy_publish_ok(1.0), "the values are within the limits");
    assert_eq!((r.s.v_hint, r.s.i_hint), exact_bounds(&r.s), "the scan leaves the bounds exact");
}

#[test]
fn a_bound_over_the_limit_is_rescanned_so_the_readout_defers_again_once_the_values_are_back_in() {
    let p = composite(a_floating_hash());
    let mut r = Rig::new(&p, false);
    prefix(&mut r);
    // A write past the limit and its repair, as a solve that overflowed and a
    // later one that did not would leave the bound.
    r.s.voltages[2] = 2.0 * LAZY_V_LIMIT;
    fold_bound(&mut r.s.v_hint, r.s.voltages[2]);
    assert!(!r.s.lazy_publish_ok(1.0), "the voltage is over the limit");
    r.s.voltages[2] = 1.0;
    assert!(r.s.lazy_publish_ok(1.0), "back inside it, the rescan clears the bound");
    r.s.voltages[2] = f64::NAN;
    fold_bound(&mut r.s.v_hint, r.s.voltages[2]);
    assert!(r.s.v_hint.is_infinite(), "a NaN never hides inside a bound");
    assert!(!r.s.lazy_publish_ok(1.0));
}

#[test]
fn a_deferred_publish_checks_the_currents_the_dynamic_partitions_wrote_and_names_the_same_device_the_full_scan_does() {
    let p = composite(a_floating_hash());
    let mut r = Rig::new(&p, false);
    prefix(&mut r);
    let cap = r.s.dyn_out[r.s.cf_parts[0]].caps[0];
    let resistor_in_ladder = r.s.dyn_out[r.s.stepped_parts[0]].devices.last().copied().unwrap();
    for device in [cap, resistor_in_ladder] {
        let held = r.s.currents[device];
        r.s.begin_dynamic_writes(&[], true, true);
        assert!(r.s.check_finite_written().is_ok());
        r.s.currents[device] = f64::NAN;
        let fast = r.s.check_finite_written().unwrap_err();
        assert_eq!(fast, r.s.check_finite(true).unwrap_err());
        assert!(fast.starts_with(&format!("device {device}:")), "{fast}");
        r.s.currents[device] = held;
    }
    // A partition that was not written is not looked at (its value passed the
    // check when it was written).
    r.s.begin_dynamic_writes(&[r.s.cf_parts[0]], false, false);
    r.s.currents[resistor_in_ladder] = f64::NAN;
    assert!(r.s.check_finite_written().is_ok());
}

// ---- the static solver's part ---------------------------------------------

#[test]
fn the_static_solvers_source_walk_runs_when_a_coefficient_could_overflow_and_the_instant_fails_as_it_always_did() {
    // Two resistors of 1e-308 ohm straight across a 1 V source: each carries a
    // finite 1e308 A, the source's sum is not. Nothing bounds that walk, so
    // it must run and fail the resolve on the source (device 0).
    let p = Program {
        carry: Vec::new(),
        node_count: 2,
        options: Options::default(),
        digital_slot_count: 0,
        channel_slot_count: 0,
        devices: vec![source(0, 1, 0, 1.0), resistor(1, 1, 0, 1e-308), resistor(2, 1, 0, 1e-308)],
    };
    let mut s = PartitionedSolver::build(&p).unwrap();
    assert!(!s.lazy_static_ok, "a coefficient that large cannot be bounded");
    let err = s.resolve_dirty(&[], 0.0, &[]).unwrap_err();
    assert_eq!(err, "device 0: published current -inf is not finite");
}

#[test]
fn the_static_solver_skips_its_source_walk_when_everything_is_bounded_and_publishes_the_same_readout() {
    let p = composite(a_floating_hash());
    let (mut skipping, mut walking) = (Rig::new(&p, false), Rig::new(&p, false));
    walking.s.static_solver.set_source_walk_skippable(false);
    for r in [&mut skipping, &mut walking] {
        r.resolve_all(0.0);
    }
    assert!(!skipping.s.static_solver.skipped_the_source_walk(), "the first resolve recomputes everything, the walk included");
    for (i, levels) in [[High, Low, High, Low], [Low, High, Low, High], [High, High, Low, Low]].into_iter().enumerate() {
        let t = (i + 1) as f64 * 1e-5;
        for r in [&mut skipping, &mut walking] {
            commit_levels(&mut r.drivers, levels, t);
            r.resolve_all(t);
            r.toggle_switch((i % 2) as f64, t);
        }
        assert!(skipping.s.static_solver.skipped_the_source_walk(), "with every input bounded the walk is skipped (round {i})");
        assert!(!walking.s.static_solver.skipped_the_source_walk());
        assert_eq!(published_bits(&mut skipping.s), published_bits(&mut walking.s), "commit round {i}");
    }

    // The walk runs again whenever the owner cannot vouch for the inputs: a
    // solver that is not lazy (a coefficient it could not bound), or one whose
    // voltage bound was lifted (a rollback, a failed call).
    let mut unproven = Rig::new(&p, true);
    unproven.resolve_all(0.0);
    unproven.toggle_switch(1.0, 1e-5);
    assert!(!unproven.s.static_solver.skipped_the_source_walk(), "the readout is not deferred, so nothing vouches for the walk's inputs");
    let mut lifted = Rig::new(&p, false);
    lifted.resolve_all(0.0);
    lifted.toggle_switch(1.0, 1e-5);
    assert!(lifted.s.static_solver.skipped_the_source_walk());
    lifted.s.v_hint = f64::INFINITY;
    lifted.toggle_switch(0.0, 2e-5);
    assert!(!lifted.s.static_solver.skipped_the_source_walk(), "a lifted bound means the voltages are unproven");
}

/// A closed-form partition (A) that queues a crossing, a stepped one (B1) that
/// queues a step, and a stepped one (B2) that fails: a switch closed on it
/// joins its capacitor to a 1e306 V node, and the solve overflows. Devices: 0
/// source, 1-3 A (a switch in parallel with R, and its C), 4 the poison
/// source, 5-8 B1 (a switch, C, R, C), 9-12 B2 (the same off the poison node).
fn a_crossing_a_step_and_a_failure() -> Program {
    program(8, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 1, 2, 0.0),
        resistor(2, 1, 2, 1000.0),
        capacitor(3, 2, 0, 1e-6, 0.0, 0.0),
        source(4, 3, 0, 1e306),
        switch_dev(5, 1, 4, 0.0),
        capacitor(6, 4, 0, 1e-6, 0.0, 0.0),
        resistor(7, 4, 5, 1000.0),
        capacitor(8, 5, 0, 1e-6, 0.0, 0.0),
        switch_dev(9, 3, 6, 0.0),
        capacitor(10, 6, 0, 1e-6, 0.0, 0.0),
        resistor(11, 6, 7, 1000.0),
        capacitor(12, 7, 0, 1e-6, 0.0, 0.0),
    ])
}

fn program(node_count: usize, devices: Vec<Device>) -> Program {
    Program { carry: Vec::new(), node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
}

#[test]
fn a_resolve_that_fails_after_other_partitions_queued_their_crossing_and_step_leaves_neither_queued() {
    let p = a_crossing_a_step_and_a_failure();
    let mut s = PartitionedSolver::build(&p).unwrap();
    assert_eq!(s.cf_parts.len(), 1);
    assert_eq!(s.stepped_parts.len(), 2);
    assert!(s.watch_node(2, DeviceId(0), Box::new(|_v: &[f64]| vec![Threshold { value: 3.0, rising: true }])));
    let all = everything(&s);
    s.resolve_dirty(&all, 0.0, &[]).unwrap();
    s.take_scheduled();
    let before = pieces(&s);

    let token = s.begin_instant();
    let touched = s.apply_params(&[(1, 0, 1.0), (5, 0, 1.0), (9, 0, 1.0)]);
    assert_eq!(touched.len(), 3, "all three partitions are dirty");
    let err = s.resolve_dirty(&touched, 1e-6, &[]).unwrap_err();
    assert!(err.contains("non-finite"), "{err}");
    let during = pieces(&s);
    for name in ["closed-form", "stepped"] {
        let (was, now) = (before.iter().find(|p| p.0 == name).unwrap(), during.iter().find(|p| p.0 == name).unwrap());
        assert_ne!(was.1, now.1, "the failed call must have left the {name} solver's own queue (or state) changed");
    }
    s.rollback_instant(token);
    s.apply_params(&[(1, 0, 0.0), (5, 0, 0.0), (9, 0, 0.0)]);
    for ((name, was), (_, now)) in before.iter().zip(pieces(&s)) {
        assert!(*was == now, "the {name} differs after the rollback");
    }
}

#[test]
fn the_copy_in_from_the_static_solver_moves_what_changed_and_only_that() {
    let p = composite(a_floating_hash());
    let mut r = Rig::new(&p, false);
    r.resolve_all(0.0);
    let (v0, i0) = (r.s.voltages.clone(), r.s.currents.clone());
    // One commit on the 555's OUT node (3, a static partition of its own).
    commit_levels(&mut r.drivers, [Low, Low, High, Low], 1e-5);
    let out_partition = r.s.partition_of_node(3).unwrap();
    r.resolve(&[out_partition], 1e-5);
    let changed: Vec<usize> = (0..v0.len()).filter(|&n| r.s.voltages[n].to_bits() != v0[n].to_bits()).collect();
    assert!(changed.contains(&3), "the driven node moved");
    assert!(r.s.static_solver.changed_nodes().iter().all(|n| changed.contains(n) || r.s.voltages[*n].to_bits() == v0[*n].to_bits()));
    // The shared arrays agree with the static solver's on every static node
    // and static device.
    for &n in &r.s.static_nodes {
        assert_eq!(r.s.voltages[n].to_bits(), r.s.static_solver.voltages()[n].to_bits(), "node {n}");
    }
    for &d in &r.s.static_kcl_devices {
        assert_eq!(r.s.currents[d].to_bits(), r.s.static_solver.currents()[d].to_bits(), "device {d}");
    }
    let recomputed: Vec<usize> = r.s.static_solver.recomputed_devices().collect();
    assert!(!recomputed.is_empty() && recomputed.len() < r.s.static_kcl_devices.len(), "only the devices on the changed node were recomputed");
    let moved: Vec<usize> = r.s.static_kcl_devices.iter().copied().filter(|&d| r.s.currents[d].to_bits() != i0[d].to_bits()).collect();
    assert!(moved.iter().all(|d| recomputed.contains(d)));
}

#[test]
fn a_failed_publishing_call_lifts_the_bounds_so_the_next_publish_scans() {
    let p = composite(a_floating_hash());
    let mut r = Rig::new(&p, false);
    prefix(&mut r);
    r.s.rescan_bounds();
    assert!(r.s.v_hint.is_finite() && r.s.i_hint.is_finite());
    // A crossing addressed to a partition the closed-form solver does not
    // own fails the call before it writes anything.
    let static_partition = r.s.partition_of_node(3).unwrap();
    let err = r.s.handle_dynamic_event(static_partition, 1e-5, true).unwrap_err();
    assert!(err.contains("unknown partition"), "{err}");
    assert!(r.s.v_hint.is_infinite() && r.s.i_hint.is_infinite(), "the call may have written past a limit before it failed");
}

// ---- the driver table's mirror ([`PartitionSolver::driver_changing`]) --------

/// A solver and the driver table a kernel would keep for it, with the
/// kernel's part in the protocol: `driver_changing(i)` before the committed
/// level of driver `i` changes (a commit, a reset), and none for a request,
/// which only touches the pending transition and the token.
struct Wired {
    s: PartitionedSolver,
    table: Vec<Driver>,
    notify: bool,
}

impl Wired {
    fn new(p: &Program, table: Vec<Driver>, notify: bool) -> Wired {
        Wired { s: PartitionedSolver::build(p).unwrap(), table, notify }
    }

    fn tell(&mut self, i: usize) {
        if self.notify {
            self.s.driver_changing(i);
        }
    }

    fn commit(&mut self, i: usize, level: Mode, t: f64) {
        if self.table[i].committed() == Some(level) {
            return;
        }
        // Nothing to commit when the driver is already on its way there.
        let Some((_, mode, token)) = self.table[i].request(level, t, &DelaySet::uniform(0.0)) else { return };
        self.tell(i);
        assert!(self.table[i].apply_commit(mode, token));
    }

    fn request_only(&mut self, i: usize, level: Mode, t: f64) {
        let _ = self.table[i].request(level, t, &DelaySet::uniform(1.0));
    }

    fn reset(&mut self, i: usize) {
        self.tell(i);
        self.table[i].reset();
    }

    fn resolve(&mut self, dirty: &[PartitionId], t: f64) {
        self.s.resolve_dirty(dirty, t, &self.table).unwrap();
    }

    fn resolve_all(&mut self, t: f64) {
        let all = everything(&self.s);
        self.resolve(&all, t);
    }
}

/// A rail (1), an 74ls04 whose `1y` drives node 2 of a stepped ladder (nodes 2
/// and 3, two capacitors) and whose `2y` drives node 4, a static one: the
/// stepped partition's catch-up at a commit needs the table from before it.
fn a_driver_on_a_stepped_ladder() -> (Program, Vec<Driver>) {
    let mut ls04_params = vec![0.0; ic_param::COUNT];
    ls04_params[ic_param::VCC_MIN] = 4.75;
    ls04_params[ic_param::VCC_MAX] = 5.25;
    ls04_params[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    ls04_params[ic_param::VIL_FIXED] = 1.0;
    ls04_params[ic_param::VIL] = 0.8;
    ls04_params[ic_param::VIH_FIXED] = 1.0;
    ls04_params[ic_param::VIH] = 2.0;
    ls04_params[ic_param::NOMINAL] = 5.0;
    ls04_params[ic_param::IO_MAX] = 0.008;
    ls04_params[ic_param::DELAY] = 20e-9;
    let mut blob = vec![0.0];
    blob.extend((0..12).map(|i| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }));
    let mut pins = vec![pin(1), pin(0)];
    pins.extend([0usize, 2, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0].into_iter().map(pin));
    let ls04 = Device { kind: kinds::IC_74LS04, component_index: 1, pins, params: ls04_params, blob };
    let p = Program {
        carry: Vec::new(),
        node_count: 5,
        options: Options::default(),
        digital_slot_count: 0,
        channel_slot_count: 0,
        devices: vec![
            source(0, 1, 0, 5.0),
            ls04,
            resistor(2, 2, 3, 1000.0),
            capacitor(3, 2, 0, 1e-6, 0.0, 0.0),
            capacitor(4, 3, 0, 1e-6, 0.0, 0.0),
            resistor(5, 3, 0, 10000.0),
            resistor(6, 4, 0, 1000.0),
        ],
    };
    // `1y` (node 2), then `2y` (node 4).
    (p, vec![Driver::new_ic_output(2, 1, 0, 1, false), Driver::new_ic_output(4, 1, 0, 1, false)])
}

#[test]
fn a_solver_told_of_each_commit_refreshes_only_those_entries_and_publishes_what_the_copying_one_does() {
    for (name, p, table) in [
        ("the composite", composite(a_floating_hash()), composite_drivers()),
        ("a driver on a stepped ladder", a_driver_on_a_stepped_ladder().0, a_driver_on_a_stepped_ladder().1),
    ] {
        let (mut told, mut copying) = (Wired::new(&p, table.clone(), true), Wired::new(&p, table, false));
        let n = told.table.len();
        let levels = [[High, Low, High, Low], [Low, High, Low, High], [High, High, Low, Low], [Low, Low, High, High]];
        for w in [&mut told, &mut copying] {
            w.resolve_all(0.0);
        }
        for (round, level) in levels.iter().cycle().take(12).enumerate() {
            let t = 1e-5 * (round + 1) as f64;
            for w in [&mut told, &mut copying] {
                // A request without a commit changes nothing the solver reads.
                w.request_only(0, level[0], t);
                w.commit(round % n, level[round % 4], t);
                if round == 5 {
                    w.reset(1);
                }
                w.resolve_all(t);
                w.s.sync_dynamic_to(t + 5e-6).unwrap();
                deliver(&mut w.s, &w.table, 1.0);
            }
            assert_same(&told.s, &copying.s, &format!("{name}, round {round}"));
            assert_eq!(published_bits(&mut told.s), published_bits(&mut copying.s), "{name}, round {round}");
        }
        assert!(told.s.mirror_live && told.s.mirror_valid, "{name}: the solver relies on the notifications");
        assert!(!copying.s.mirror_live);
        // The last resolve refreshed a few entries, not the table.
        let last = n - 1;
        told.commit(last, if told.table[last].committed() == Some(High) { Low } else { High }, 9e-4);
        told.resolve_all(9e-4);
        assert!(told.s.call_patches.len() == 1 && told.s.call_patches.len() < n.max(2), "{name}: {} entries refreshed", told.s.call_patches.len());
    }
}

#[test]
fn a_notified_solvers_windows_roll_the_table_back_through_the_entries_they_changed() {
    let p = composite(a_floating_hash());
    let (mut told, mut twin) = (Wired::new(&p, composite_drivers(), true), Wired::new(&p, composite_drivers(), false));
    for w in [&mut told, &mut twin] {
        w.resolve_all(0.0);
        w.commit(0, High, 1e-5);
        w.commit(2, High, 1e-5);
        w.resolve_all(1e-5);
        w.s.sync_dynamic_to(2e-5).unwrap();
        published_bits(&mut w.s);
    }
    assert!(told.s.mirror_live && told.s.mirror_valid);
    let before = pieces(&told.s);
    let table_before = told.table.clone();

    // The failing instant: commits between and after two resolves, one reset,
    // and a request that is never applied. A commit that is made and not yet
    // resolved when the instant fails is the entry `stale` holds.
    let token = told.s.begin_instant();
    assert!(told.s.window_mirror, "the table is rolled back through the log, not held");
    told.commit(0, Low, 3e-5);
    told.commit(1, High, 3e-5);
    told.resolve_all(3e-5);
    told.commit(0, High, 3e-5);
    told.reset(2);
    told.request_only(3, High, 3e-5);
    told.resolve_all(3e-5);
    told.commit(3, Low, 3e-5);
    assert_eq!(told.s.stale, vec![3], "the last commit is not yet in the mirror");
    told.s.rollback_instant(token);
    told.table = table_before;
    for ((name, was), (_, now)) in before.iter().zip(pieces(&told.s)) {
        assert!(*was == now, "a rollback must put back the {name}");
    }
    assert!(told.s.stale.is_empty(), "the kernel's table is back too, so no entry is stale");

    // It goes on like the twin that never had the instant.
    for w in [&mut told, &mut twin] {
        w.commit(1, High, 4e-5);
        w.resolve_all(4e-5);
        w.commit(3, High, 5e-5);
        w.resolve_all(5e-5);
        w.s.sync_dynamic_to(1e-3).unwrap();
    }
    assert_same(&told.s, &twin.s, "after the rolled-back instant");
    assert_eq!(published_bits(&mut told.s), published_bits(&mut twin.s));

    // A completed window changes nothing about the results either.
    let token = told.s.begin_instant();
    told.commit(0, Low, 6e-5);
    told.resolve_all(6e-5);
    told.s.end_instant(token);
    twin.commit(0, Low, 6e-5);
    twin.resolve_all(6e-5);
    assert_same(&told.s, &twin.s, "after a completed window");
}

#[test]
fn a_kernel_that_starts_telling_the_solver_mid_run_is_followed_from_the_next_whole_copy() {
    let p = composite(a_floating_hash());
    let (mut w, mut twin) = (Wired::new(&p, composite_drivers(), false), Wired::new(&p, composite_drivers(), false));
    for x in [&mut w, &mut twin] {
        x.resolve_all(0.0);
        x.commit(0, High, 1e-5);
        x.resolve_all(1e-5);
    }
    assert!(!w.s.mirror_live);
    // The first notification arrives inside a window, which keeps holding the
    // table its own way to its end.
    w.notify = true;
    let token = w.s.begin_instant();
    w.commit(1, High, 2e-5);
    twin.commit(1, High, 2e-5);
    assert!(w.s.mirror_seen && !w.s.mirror_live, "a window that began without the notifications does not switch mid-way");
    w.resolve_all(2e-5);
    twin.resolve_all(2e-5);
    w.s.end_instant(token);
    assert!(w.s.mirror_live && !w.s.mirror_valid, "now it relies on them, and the next resolve copies the table whole to start");
    for (i, x) in [&mut w, &mut twin].into_iter().enumerate() {
        x.commit(2, High, 3e-5);
        x.resolve_all(3e-5);
        assert_eq!(i == 0, x.s.mirror_live);
    }
    assert!(w.s.mirror_valid && w.s.call_patches.is_empty(), "that resolve was the whole copy");
    for x in [&mut w, &mut twin] {
        x.commit(3, High, 4e-5);
        x.resolve_all(4e-5);
    }
    assert_eq!(w.s.call_patches.len(), 1, "and this one the entry that moved");
    assert_same(&w.s, &twin.s, "after switching over");

    // A rollback taken in a window that began before the switch leaves the
    // solver copying the table whole again.
    let mut u = Wired::new(&p, composite_drivers(), false);
    u.resolve_all(0.0);
    let held = pieces(&u.s);
    u.notify = true;
    let token = u.s.begin_instant();
    u.commit(0, High, 1e-5);
    u.resolve_all(1e-5);
    u.s.rollback_instant(token);
    u.table = composite_drivers();
    for ((name, was), (_, now)) in held.iter().zip(pieces(&u.s)) {
        assert!(*was == now, "the {name}");
    }
    assert!(!u.s.mirror_valid && u.s.stale.is_empty());
}

#[test]
#[should_panic(expected = "changed without `driver_changing` being called first")]
fn a_committed_level_the_kernel_changed_without_saying_so_is_caught_in_a_debug_build() {
    let p = composite(a_floating_hash());
    let mut w = Wired::new(&p, composite_drivers(), true);
    w.resolve_all(0.0);
    w.commit(0, High, 1e-5);
    w.resolve_all(1e-5);
    assert!(w.s.mirror_live && w.s.mirror_valid);
    w.notify = false;
    w.commit(1, High, 2e-5);
    w.resolve_all(2e-5);
}

#[test]
fn a_resolve_handed_the_solvers_own_table_leaves_the_kernels_notices_for_the_next() {
    // The slow republish (`settle.rs`'s `republish_fixed`) resolves with the
    // solver's own `driver_snapshot`, the table as of the last resolve. A
    // commit the kernel announced since is not in that table: the refresh
    // must not count it as delivered, or the next real resolve finds the
    // entry changed behind its back (cold 64 tpl 5, hub 2) and, in a release
    // build, the solver keeps the old level for good.
    let p = composite(a_floating_hash());
    let (mut told, mut copying) = (Wired::new(&p, composite_drivers(), true), Wired::new(&p, composite_drivers(), false));
    for w in [&mut told, &mut copying] {
        w.resolve_all(0.0);
        w.commit(0, High, 1e-5);
        w.resolve_all(1e-5);
    }
    assert!(told.s.mirror_live && told.s.mirror_valid);
    for w in [&mut told, &mut copying] {
        let token = w.s.begin_instant();
        w.commit(1, High, 2e-5);
        let own = Rc::clone(&w.s.driver_snapshot);
        w.s.resolve_dirty(&[], 2e-5, &own).unwrap();
        w.s.end_instant(token);
    }
    assert_eq!(told.s.stale, vec![1], "the notice waits for a resolve with the kernel's table");
    for w in [&mut told, &mut copying] {
        w.resolve_all(3e-5);
        assert!(w.s.driver_snapshot.iter().zip(&w.table).all(|(a, b)| same_driver(a, b)), "the solver's table is the kernel's");
    }
    assert_same(&told.s, &copying.s, "after the next resolve");
}

#[test]
fn a_stepped_partitions_catch_up_sees_the_table_from_before_the_commit_when_the_mirror_refreshes_in_place() {
    let (p, table) = a_driver_on_a_stepped_ladder();
    let (mut told, mut copying) = (Wired::new(&p, table.clone(), true), Wired::new(&p, table, false));
    assert_eq!(told.s.stepped_parts.len(), 1);
    let stepped = PartitionId(told.s.stepped_parts[0] as u32);
    for w in [&mut told, &mut copying] {
        w.resolve_all(0.0);
        w.commit(0, High, 1e-5);
        w.resolve(&[stepped], 1e-5);
        w.s.take_scheduled();
    }
    // The next commit lands mid-step: the catch-up over the span before it must
    // use the level that was in force, LOW-to-HIGH not the HIGH-to-LOW just made.
    for w in [&mut told, &mut copying] {
        w.commit(0, Low, 2.5e-4);
        w.resolve(&[stepped], 2.5e-4);
    }
    assert!(told.s.call_patches.iter().any(|(idx, _)| *idx == 0));
    assert_same(&told.s, &copying.s, "after the second commit");
    assert_eq!(published_bits(&mut told.s), published_bits(&mut copying.s));
    assert_eq!(
        told.s.pre_commit_drivers.iter().map(|d| d.committed()).collect::<Vec<_>>(),
        copying.s.pre_commit_drivers.iter().map(|d| d.committed()).collect::<Vec<_>>(),
        "the table before this resolve is the same either way"
    );
    assert_eq!(told.s.pre_commit_drivers[0].committed(), Some(High));
}

#[test]
fn a_whole_copy_inside_a_window_that_rolls_back_through_its_log_is_undone_with_the_entries_before_it() {
    // The kernel never grows the table after load, but nothing the solver
    // relies on says it cannot, and a window that meets one must still come
    // back: the entries refreshed before the copy go back on the table the
    // copy replaced.
    let p = composite(a_floating_hash());
    let mut w = Wired::new(&p, composite_drivers(), true);
    w.resolve_all(0.0);
    w.commit(0, High, 1e-5);
    w.resolve_all(1e-5);
    let before = pieces(&w.s);
    let held = w.table.clone();

    let token = w.s.begin_instant();
    assert!(w.s.window_mirror);
    w.commit(1, High, 2e-5);
    w.commit(2, High, 2e-5);
    w.resolve_all(2e-5); // in place: entries 1 and 2.
    assert_eq!(w.s.call_patches.len(), 2);
    // Entry 0 changes and is not yet in the mirror when the longer table
    // arrives: the whole copy takes it in, and a later refresh of it records
    // that copy's value as the one it replaced.
    w.commit(0, Low, 2.2e-5);
    w.table.push(Driver::new_ic_output(8, 1, 0, 1, false)); // a longer table: a whole copy.
    w.resolve_all(2.5e-5);
    assert!(w.s.driver_snapshot.len() == 5 && w.s.call_patches.is_empty(), "the whole table was copied");
    w.commit(0, High, 2.8e-5);
    w.commit(3, High, 3e-5);
    w.resolve_all(3e-5);
    w.s.rollback_instant(token);
    w.table = held;
    for ((name, was), (_, now)) in before.iter().zip(pieces(&w.s)) {
        assert!(*was == now, "the {name} differs");
    }
    assert_eq!(w.s.driver_snapshot.len(), 4);
    // And the solver goes on from there, still in step with the table.
    w.commit(1, High, 4e-5);
    w.resolve_all(4e-5);
    assert_eq!(w.s.call_patches.len(), 1);
}

#[test]
fn a_window_that_begins_while_the_mirror_is_unproven_holds_its_table_and_leaves_it_unproven() {
    let p = composite(a_floating_hash());
    let mut w = Wired::new(&p, composite_drivers(), true);
    w.resolve_all(0.0);
    w.commit(0, High, 1e-5);
    w.resolve_all(1e-5);
    assert!(w.s.mirror_live && w.s.mirror_valid);

    // A table that came from a snapshot, not from a resolve, is not one the
    // notifications since can be applied to.
    let (snapshot, dynamic) = (w.s.snapshot(), w.s.dynamic_snapshot());
    w.s.restore(snapshot);
    w.s.dynamic_restore(dynamic);
    assert!(!w.s.mirror_valid && w.s.stale.is_empty());

    let before = pieces(&w.s);
    let held = w.table.clone();
    let token = w.s.begin_instant();
    assert!(!w.s.window_mirror, "the table is held, since no entry list vouches for it");
    w.commit(1, High, 2e-5);
    w.resolve_all(2e-5);
    assert!(w.s.mirror_valid, "the resolve copied the table whole");
    w.s.rollback_instant(token);
    w.table = held;
    for ((name, was), (_, now)) in before.iter().zip(pieces(&w.s)) {
        assert!(*was == now, "the {name} differs");
    }
    assert!(!w.s.mirror_valid, "the table it went back to is the unproven one, and the next resolve copies again");
}

// ---- every way a window's call can change a dynamic partition --------------

/// A 74ls04 on `vcc` whose pins sit on `pins` (`1a 1y 2a 2y ...`).
fn ls04_on(ci: usize, vcc: usize, pins: [usize; 12]) -> Device {
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
    let mut device_pins = vec![pin(vcc), pin(0)];
    device_pins.extend(pins.into_iter().map(pin));
    Device { kind: kinds::IC_74LS04, component_index: ci, pins: device_pins, params, blob }
}

/// The first queued event of the given kind that is still current, and the
/// partition it names.
fn queued_event(s: &PartitionedSolver, crossing: bool) -> Option<(f64, PartitionId)> {
    s.pending_schedule.iter().find_map(|&(at, kind)| match kind {
        EventKind::Crossing { partition, version } if crossing && s.is_current(partition, true, version) => Some((at, partition)),
        EventKind::Step { partition, version } if !crossing && s.is_current(partition, false, version) => Some((at, partition)),
        _ => None,
    })
}

/// Runs `call` in a window on `s` and rolls it back. The call must have moved
/// the piece named `moved` (or the test proves nothing about restoring it) and
/// the rollback must put every piece back; the solvers' own oracles (test
/// builds) fail the call that changed a partition without saving it first.
fn rolled_back(s: &mut PartitionedSolver, moved: &str, what: &str, call: impl FnOnce(&mut PartitionedSolver)) {
    let before = pieces(s);
    let token = s.begin_instant();
    call(s);
    let during = pieces(s);
    let (was, now) = (before.iter().find(|p| p.0 == moved).unwrap(), during.iter().find(|p| p.0 == moved).unwrap());
    assert_ne!(was.1, now.1, "{what} must change the {moved}, or the test proves nothing about restoring it");
    s.rollback_instant(token);
    for ((name, was), (_, now)) in before.iter().zip(pieces(s)) {
        assert!(*was == now, "after {what} and a rollback the {name} differs");
    }
}

#[test]
fn a_crossing_a_step_and_a_horizon_clamp_each_change_a_partition_the_window_then_puts_back() {
    let p = composite(a_floating_hash());
    let mut r = Rig::new(&p, false);
    prefix(&mut r);
    let (crossing_at, crossing_partition) = queued_event(&r.s, true).expect("the prefix leaves a crossing queued");
    let (step_at, step_partition) = queued_event(&r.s, false).expect("the prefix leaves a step queued");
    rolled_back(&mut r.s, "closed-form", "a closed-form crossing", |s| {
        s.handle_dynamic_event(crossing_partition, crossing_at, true).unwrap();
    });
    rolled_back(&mut r.s, "stepped", "a stepped step", |s| {
        s.handle_dynamic_event(step_partition, step_at, false).unwrap();
    });
    // Before the planned step, so the clamp advances the partition to a time
    // of its own and not to the one it already planned.
    rolled_back(&mut r.s, "stepped", "a horizon clamp", |s| {
        s.sync_dynamic_to(2e-5 + (step_at - 2e-5) / 2.0).unwrap();
    });
}

/// A stepped supply (node 2, behind 10 ohm) for a 74ls04 whose `1a` reads a
/// slow stepped RC (node 3): the supply's partition has to be brought up to the
/// instant of the pin's step before the pin's next one is planned.
fn a_stepped_pin_beside_a_stepped_supply() -> Program {
    program(4, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 10.0),
        capacitor(2, 2, 0, 1e-6, 0.0, 0.0),
        resistor(3, 1, 3, 10_000.0),
        capacitor(4, 3, 0, 1e-6, 0.0, 0.0),
        ls04_on(5, 2, [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
    ])
}

#[test]
fn a_supply_partition_brought_up_to_a_pins_step_goes_back_with_the_rollback() {
    let p = a_stepped_pin_beside_a_stepped_supply();
    let mut s = PartitionedSolver::build(&p).unwrap();
    let (supply, pin_partition) = (s.partition_of_node(2).unwrap(), s.partition_of_node(3).unwrap());
    assert_ne!(supply, pin_partition);
    assert_eq!(s.stepped_parts.len(), 2, "both are stepped, the pin's by promotion");
    let all = everything(&s);
    s.resolve_dirty(&all, 0.0, &[]).unwrap();
    // The pin's own first step is far longer than the supply's.
    let (at, partition) = s
        .pending_schedule
        .iter()
        .find_map(|&(at, kind)| match kind {
            EventKind::Step { partition, version } if partition == pin_partition && s.is_current(partition, false, version) => Some((at, partition)),
            _ => None,
        })
        .expect("the pin's partition has a step queued");
    let supply_was_at = s.stepped.time_of(supply.0 as usize).unwrap();
    assert!(supply_was_at < at - 1e-12, "the supply's partition is behind the instant of the pin's step");
    rolled_back(&mut s, "stepped", "the pin's step, which brings the supply along", |s| {
        s.handle_dynamic_event(partition, at, false).unwrap();
        assert!(s.stepped.time_of(supply.0 as usize).unwrap() >= at - 1e-12, "the supply's partition was brought up to the step");
    });
    assert_eq!(s.stepped.time_of(supply.0 as usize).unwrap(), supply_was_at, "and is back where it was");
}

// ---- a table someone else holds, the source-walk guard, and `restore` -------

type DynamicSnapshot = ((Vec<f64>, Vec<f64>), closed_form::Snapshot, stepped::Snapshot, Vec<(f64, EventKind)>, Option<f64>, Rc<Vec<Driver>>, Vec<f64>, bool, f64);

/// The committed levels of the driver table a `dynamic_snapshot()` holds.
fn held_table(snapshot: &dyn std::any::Any) -> Vec<Option<Mode>> {
    snapshot.downcast_ref::<(DynamicSnapshot, super::hub::W1fSnapshot)>().expect("PartitionedSolver's dynamic snapshot").0 .5.iter().map(|d| d.committed()).collect()
}

#[test]
fn a_dynamic_snapshot_held_across_a_resolve_keeps_its_table_and_the_mirror_stays_right() {
    let p = composite(a_floating_hash());
    let (mut told, mut twin) = (Wired::new(&p, composite_drivers(), true), Wired::new(&p, composite_drivers(), false));
    for w in [&mut told, &mut twin] {
        w.resolve_all(0.0);
        w.commit(0, High, 1e-5);
        w.resolve_all(1e-5);
    }
    assert!(told.s.mirror_live && told.s.mirror_valid);

    // Outside a window: the resolve writes its two entries into a copy of the
    // table, not into the one the snapshot shares.
    let held = told.s.dynamic_snapshot();
    let when_taken = held_table(&*held);
    for w in [&mut told, &mut twin] {
        w.commit(1, High, 2e-5);
        w.commit(2, High, 2e-5);
        w.resolve_all(2e-5);
    }
    assert_eq!(held_table(&*held), when_taken, "a holder's table must not change under it");
    assert_eq!(told.s.call_patches.len(), 2, "and the resolve still refreshed just the entries that moved");
    assert_same(&told.s, &twin.s, "after a resolve with a snapshot held");
    assert_eq!(published_bits(&mut told.s), published_bits(&mut twin.s));
    drop(held);

    // Inside a window that rolls the table back through its log, and then
    // rolled back.
    let held = told.s.dynamic_snapshot();
    let when_taken = held_table(&*held);
    let before = pieces(&told.s);
    let table_before = told.table.clone();
    let token = told.s.begin_instant();
    assert!(told.s.window_mirror);
    told.commit(3, High, 3e-5);
    told.resolve_all(3e-5);
    assert_eq!(held_table(&*held), when_taken);
    told.s.rollback_instant(token);
    told.table = table_before;
    for ((name, was), (_, now)) in before.iter().zip(pieces(&told.s)) {
        assert!(*was == now, "after a rollback with a snapshot held the {name} differs");
    }
    assert_eq!(held_table(&*held), when_taken);

    // And the solver goes on like the twin that never ran the instant.
    for w in [&mut told, &mut twin] {
        w.commit(3, High, 4e-5);
        w.resolve_all(4e-5);
    }
    assert_same(&told.s, &twin.s, "after the rolled-back instant");
    assert_eq!(published_bits(&mut told.s), published_bits(&mut twin.s));
}

/// A rail at 1e308 V with two 1-ohm resistors to free nodes that each have a
/// switch to ground, open. Closing both puts almost the whole rail across each
/// resistor: each current is finite, their sum at the rail is not.
fn two_resistors_that_sum_past_the_float_range() -> Program {
    program(4, vec![
        source(0, 1, 0, 1e308),
        resistor(1, 1, 2, 1.0),
        switch_dev(2, 2, 0, 0.0),
        resistor(3, 1, 3, 1.0),
        switch_dev(4, 3, 0, 0.0),
    ])
}

#[test]
fn the_source_walk_is_not_skipped_when_this_calls_own_solve_changes_a_node_past_the_voltage_limit() {
    // The owner vouches for every voltage as of the last publish, which is
    // all it can do: what this call's own solve writes is checked here, by the
    // solver, against the same limit. So with the vouching in place the walk
    // must still run, and fail the resolve on the source exactly as it does
    // when it is never skipped.
    let p = two_resistors_that_sum_past_the_float_range();
    let run = |skippable: bool| {
        let mut s = PartitionedSolver::build(&p).unwrap();
        s.static_solver.set_source_walk_skippable(skippable);
        let mut readers = Vec::new();
        let all = everything(&s);
        s.static_solver.resolve_dirty_into(&all, 0.0, &[], &mut readers).expect("the open switches leave every sum small");
        let touched = s.apply_params(&[(2, 0, 1.0), (4, 0, 1.0)]);
        s.static_solver.set_inputs_bounded(true);
        let err = s.static_solver.resolve_dirty_into(&touched, 1e-6, &[], &mut readers).unwrap_err();
        (err, s.static_solver.skipped_the_source_walk())
    };
    let (walking, _) = run(false);
    assert!(walking.starts_with("device 0:"), "the source's sum is the one that overflows: {walking}");
    let (skipping, skipped) = run(true);
    assert!(!skipped, "a node moved past the limit, so the walk ran");
    assert_eq!(skipping, walking, "and failed the same device the same way");
}

#[test]
fn restoring_a_snapshot_lifts_the_bounds_so_the_next_publish_scans_what_came_back() {
    let p = composite(a_floating_hash());
    let mut r = Rig::new(&p, false);
    prefix(&mut r);
    r.s.rescan_bounds();
    assert!(r.s.v_hint.is_finite() && r.s.lazy_publish_ok(1.0), "the bounds are exact and within the limits");
    // A snapshot of values the bounds never covered: one taken before they
    // were last made exact would hold a voltage like this one.
    let (mut voltages, currents) = r.s.snapshot();
    voltages[2] = 2.0 * LAZY_V_LIMIT;
    r.s.restore((voltages, currents));
    assert!(r.s.v_hint.is_infinite() && r.s.i_hint.is_infinite(), "values from before the bounds were exact are back");
    assert!(!r.s.lazy_publish_ok(1.0), "the scan finds the restored voltage past the limit");
    assert_eq!(r.s.v_hint, 2.0 * LAZY_V_LIMIT, "and leaves the bound exact");
}
