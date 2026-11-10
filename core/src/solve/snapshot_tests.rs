//! W2a: whole-engine snapshots (core/README.md, "Snapshots"). Each board runs
//! to a stop, takes a snapshot, runs on recording every stop, restores, and
//! must run on again bit for bit, with every stat reading on from where it
//! was rather than going back.

use super::led_tests::clocked_led;
use super::newton::hooks;
use super::rail_tests::{cold_1838_tpl1, cold_2575_tpl2, cold_287_tpl3, cold_474_tpl5, cold_757_tpl0, STOPS_2575};
use super::slow_tests::{ring_behind_a_fuse, rollback_rig};
use crate::engine::Engine;
use crate::error::ERR_SOLVE_FAILED;
use crate::program::Program;
use crate::snapshot::{net_allocated, HeapBytes, SnapshotTable, Walk};

const STAT_COUNT: u32 = 28;

/// Everything a stop publishes, as bits: time, voltages, currents, digital
/// and channel slots, the four record channels, the EEPROM update buffer and
/// the two next-due times. Reading the records and the EEPROM buffer drains
/// them, which both runs do at the same stops.
fn seen(e: &mut Engine) -> Vec<u64> {
    let mut row: Vec<u64> = vec![e.time().to_bits()];
    row.extend(e.voltages().iter().chain(e.currents()).chain(e.digital_slots()).chain(e.channel_slots()).map(|v| v.to_bits()));
    for channel in 0..4 {
        let records = e.records(channel).unwrap().to_vec();
        row.push(records.len() as u64);
        row.extend(records.iter().map(|v| v.to_bits()));
    }
    let eeprom = e.take_eeprom_updates().to_vec();
    row.push(eeprom.len() as u64);
    row.extend(eeprom.iter().map(|v| v.to_bits()));
    row.push(e.next_event_time().to_bits());
    row.push(e.next_digital_due_time().to_bits());
    row
}

fn stats(e: &Engine) -> Vec<u64> {
    (0..STAT_COUNT).map(|i| e.stat(i).to_bits()).collect()
}

/// The host's side of a run: the params it toggles, as `traced_run` keeps
/// them, so a restore can put them back with the engine.
#[derive(Clone)]
struct Host {
    state: Vec<f64>,
    next: usize,
}

/// Advances through `stops`, recording each, toggling a switch whose time
/// has come after the stop (as `traced_run` does). A failed advance ends
/// the run with its code and time.
fn run(e: &mut Engine, host: &mut Host, toggles: &[(f64, usize)], stops: &[f64]) -> Vec<Vec<u64>> {
    let mut rows = Vec::new();
    for &stop in stops {
        if let Err(er) = e.advance(stop) {
            rows.push(vec![er.code as u64, e.time().to_bits()]);
            break;
        }
        rows.push(seen(e));
        while host.next < toggles.len() && toggles[host.next].0 <= stop {
            let d = toggles[host.next].1;
            host.state[d] = 1.0 - host.state[d];
            e.set_params(&[(d as u32, 0, host.state[d])]).unwrap();
            host.next += 1;
        }
    }
    rows
}

/// Runs `stops[..k]`, snapshots, runs the rest, then restores twice and
/// runs the rest again each time: three identical futures, and stats that
/// never go back. Returns the first future.
fn round_trip(name: &str, p: &Program, toggles: &[(f64, usize)], stops: &[f64], k: usize) -> Vec<Vec<u64>> {
    let mut e = Engine::load(p).unwrap_or_else(|er| panic!("{name}: load: {}", er.message));
    let mut host = Host { state: p.devices.iter().map(|d| d.params.first().copied().unwrap_or(0.0)).collect(), next: 0 };
    let before = run(&mut e, &mut host, toggles, &stops[..k]);
    assert_eq!(before.len(), k, "{name}: the run reaches the snapshot");
    let mut table = SnapshotTable::default();
    let id = table.take(&e);
    let saved = host.clone();
    let at_snapshot = seen(&mut e);
    let first = run(&mut e, &mut host, toggles, &stops[k..]);
    for again in 0..2 {
        let published = stats(&e);
        assert!(table.restore(&mut e, id), "{name}: the id is held");
        assert_eq!(stats(&e), published, "{name}: a restore leaves every stat where it was");
        let mut host = saved.clone();
        assert_eq!(seen(&mut e), at_snapshot, "{name}: the restored state publishes the snapshot's");
        let rerun = run(&mut e, &mut host, toggles, &stops[k..]);
        assert_eq!(rerun.len(), first.len(), "{name}: restore {again}: stops reached");
        for (j, (a, b)) in first.iter().zip(&rerun).enumerate() {
            assert!(a == b, "{name}: restore {again}: stop {} at {} differs", k + j, stops[k + j]);
        }
        for i in 0..STAT_COUNT {
            assert!(e.stat(i) >= f64::from_bits(published[i as usize]) || f64::from_bits(published[i as usize]).is_nan(), "{name}: stat {i} went back");
        }
    }
    first
}

#[test]
fn a_restored_run_goes_on_bit_identically_on_the_rail_and_slow_boards() {
    // 2575 tpl 2: a CC rail capacitor and host edits. 1838 tpl 1: slow
    // re-anchors. 757 tpl 0: the hold guard's episodes. 474 tpl 5: a fuse's
    // stepped partition reading the rail (R80's end reads). Each snapshot
    // lands mid-run, with an edit still to come where the board has them.
    let t2575 = [(0.030082069747425742, 14), (0.3166121668293984, 21), (1.009710339541874, 21), (1.0359573122741632, 12)];
    let (p1838, t1838, s1838) = cold_1838_tpl1();
    let stops757: Vec<f64> = (1..=40).map(|k| k as f64 * 5e-3).collect();
    let stops474 = [0.0209, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05];
    let boards: Vec<(&str, Program, Vec<(f64, usize)>, Vec<f64>, usize)> = vec![
        ("2575 tpl 2", cold_2575_tpl2(), t2575.to_vec(), STOPS_2575[..60].to_vec(), 25),
        ("1838 tpl 1", p1838, t1838, s1838.clone(), s1838.len() / 2),
        ("757 tpl 0", cold_757_tpl0(), vec![], stops757, 13),
        ("474 tpl 5", cold_474_tpl5(), vec![], stops474.to_vec(), 1),
    ];
    for (name, p, toggles, stops, k) in &boards {
        round_trip(name, p, toggles, stops, *k);
    }
}

#[test]
fn a_restore_brings_back_the_trapezoids_history_after_a_discontinuity() {
    // INT-ACC: 287 tpl 3's pulse source steps at 10.4 ms and the next step
    // goes by backward Euler (`be_next_step`) before the trapezoid resumes;
    // its history (`i_prev`, `trap_off`, `trap_run`) is partition state. A
    // snapshot at each stop around the edge must give the same steps, and
    // the fallbacks counted after the restore match the first run's.
    let p = cold_287_tpl3();
    let stops = [0.006272879077239732, 0.0104, 0.010400001, 0.0104001, 0.0105, 0.011, 0.014446754687130081, 0.02, 0.04];
    for k in 1..5 {
        round_trip("287 tpl 3", &p, &[], &stops, k);
        let mut e = Engine::load(&p).unwrap();
        for &t in &stops[..k] {
            e.advance(t).unwrap();
        }
        let mut table = SnapshotTable::default();
        let id = table.take(&e);
        let counted = |e: &mut Engine| {
            let (steps, fallbacks) = (e.stat(10), e.stat(27));
            for &t in &stops[k..] {
                e.advance(t).unwrap();
            }
            (e.stat(10) - steps, e.stat(27) - fallbacks)
        };
        let first = counted(&mut e);
        assert!(table.restore(&mut e, id));
        assert_eq!(counted(&mut e), first, "snapshot after stop {k}: the same steps and fallbacks again");
    }
}

#[test]
fn a_fuse_tripped_after_the_snapshot_trips_again_with_the_same_records_and_probes() {
    // The fuse behind the 74LS04 trips at about 17 ms (records channel 0),
    // R80's stress probes rebuild on it (`probe_version`), and the reset at
    // 50 ms closes a ring. A snapshot at 10 ms, before the trip, must replay
    // the trip, the records and every readout after it bit for bit: the
    // probe cache is in the copy, so the restored probes are the ones built
    // on the restored failure state.
    let p = ring_behind_a_fuse();
    let stops: Vec<f64> = (1..=12).map(|k| k as f64 * 5e-3).collect();
    round_trip("fuse ring", &p, &[], &stops, 2);
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.01).unwrap();
    assert!(e.records(0).unwrap().is_empty(), "the snapshot is taken before the trip");
    e.advance(0.03).unwrap();
    assert_eq!(e.records(0).unwrap().len(), 7, "and the fuse trips after it");
}

#[test]
fn the_slow_state_and_its_latches_come_back_with_a_restore() {
    // The rollback rig: a resistor latching at 1 s, a self-heating LED and a
    // sagging 74LS00 output. Snapshot at 0.5 s, before the latch.
    let p = rollback_rig();
    let stops: Vec<f64> = (1..=16).map(|k| k as f64 * 0.125).collect();
    round_trip("rollback rig", &p, &[], &stops, 4);
}

#[test]
fn a_restore_after_a_failed_advance_and_after_a_failed_reset_works() {
    // A failed advance (-4) and a failed reset (-7) each leave the engine at
    // its last completed instant; a snapshot taken before either restores,
    // and the run goes on as one that never failed.
    let mut p = clocked_led(true);
    p.channel_slot_count = 0;
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.25e-3).unwrap();
    let mut table = SnapshotTable::default();
    let id = table.take(&e);
    let mut control = Engine::load(&p).unwrap();
    control.advance(0.25e-3).unwrap();
    e.advance(0.5e-3).unwrap();
    let err = {
        let _guard = hooks::force_unconverged(u32::MAX);
        e.advance(1.5e-3).unwrap_err()
    };
    assert_eq!(err.code, ERR_SOLVE_FAILED);
    assert!(table.restore(&mut e, id));
    for t in [0.5e-3, 1e-3, 1.5e-3, 2e-3] {
        e.advance(t).unwrap();
        control.advance(t).unwrap();
        assert_eq!(seen(&mut e), seen(&mut control), "after the failed advance, at {t}");
    }

    let mut e = Engine::load(&ring_behind_a_fuse()).unwrap();
    e.advance(0.05).unwrap();
    let id = table.take(&e);
    let mut control = Engine::load(&ring_behind_a_fuse()).unwrap();
    control.advance(0.05).unwrap();
    e.set_params(&[(4, 0, 1.0)]).unwrap();
    e.advance(0.06).unwrap();
    assert_eq!(e.reset_failures().unwrap_err().code, crate::error::ERR_DELTA_LIVELOCK);
    assert!(table.restore(&mut e, id));
    for t in [0.06, 0.08, 0.1] {
        e.advance(t).unwrap();
        control.advance(t).unwrap();
        assert_eq!(seen(&mut e), seen(&mut control), "after the failed reset, at {t}");
    }
}

#[test]
fn a_budgeted_host_step_yielded_before_the_snapshot_goes_on_the_same_after_a_restore() {
    // R86: an `advance_for` that yields at t_y < t_end leaves its host step
    // open (target, events at start, span). Snapshot there, go on to t_end,
    // restore, go on again: both continuations publish the same bits and
    // count the same events against the host step's cap, and the first
    // equals a run that never yielded.
    let p = cold_757_tpl0();
    // 757 tpl 0's own stops (the split-run test's): host steps short enough
    // for the hold guard, the snapshot inside the one from 53.1 to 70.3 ms.
    let before = [0.006272881300476146, 0.011116023500639817, 0.012362155978723055, 0.037731840680085206, 0.03788763379837714, 0.053120364878471986];
    let t_end = 0.07025570911480136;
    let mut now = 0.0;
    let mut clock = move || {
        now += 1.0;
        now
    };
    fn budget<'a>(clock: &'a mut dyn FnMut() -> f64) -> Option<crate::kernel::Budget<'a>> {
        Some(crate::kernel::Budget { clock, budget_ms: 2.0, check_every: 1 })
    }
    let mut e = Engine::load(&p).unwrap();
    for t in before {
        e.advance(t).unwrap();
    }
    let reached = e.advance_for(t_end, budget(&mut clock)).unwrap();
    assert!(!reached && e.time() < t_end, "it yielded short of t_end, at {}", e.time());
    let mut table = SnapshotTable::default();
    let id = table.take(&e);
    let finish = |e: &mut Engine, clock: &mut dyn FnMut() -> f64| -> (Vec<u64>, f64, u32) {
        let events = e.stat(4);
        let mut yields = 0;
        while !e.advance_for(t_end, budget(&mut *clock)).unwrap() {
            yields += 1;
        }
        (seen(e), e.stat(4) - events, yields)
    };
    let first = finish(&mut e, &mut clock);
    assert!(table.restore(&mut e, id));
    let second = finish(&mut e, &mut clock);
    assert_eq!((&first.0, first.1), (&second.0, second.1), "the same bits and the same events counted");
    assert!(first.2 > 0 && first.2 == second.2, "both continuations yielded the same number of times: {} and {}", first.2, second.2);
    let mut plain = Engine::load(&p).unwrap();
    for t in before {
        plain.advance(t).unwrap();
    }
    plain.advance(t_end).unwrap();
    assert_eq!(seen(&mut plain), first.0, "and both are the run that never yielded");
}

#[test]
fn a_snapshot_shares_the_eeprom_image_and_the_table_counts_it_once() {
    // The bytes the table reports: each copy, plus each shared image once.
    // A 28c16 holds a 2 KiB image; two snapshots of the same state share it
    // with the live engine, so the table counts 2048 once, not twice.
    use crate::kinds;
    use crate::program::{Device, Options, PinRecord};
    let pin = |node| PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 };
    let mut params = vec![0.0; kinds::ic_param::COUNT];
    params[kinds::ic_param::VCC_MIN] = 4.5;
    params[kinds::ic_param::VCC_MAX] = 5.5;
    params[kinds::ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    params[kinds::ic_param::VIL_FIXED] = 1.0;
    params[kinds::ic_param::VIL] = 0.8;
    params[kinds::ic_param::VIH_FIXED] = 1.0;
    params[kinds::ic_param::VIH] = 2.0;
    params[kinds::ic_param::NOMINAL] = 5.0;
    params.extend(std::iter::repeat(20e-9).take(10));
    params.extend_from_slice(&[10e-3, 0.0]);
    let mut blob = vec![0.0; 1 + 22];
    blob.extend(std::iter::repeat(0.0).take(2048 / 4));
    let eeprom = Device { kind: kinds::IC_28C16, component_index: 1, pins: std::iter::once(pin(1)).chain(std::iter::once(pin(0))).chain((0..22).map(|_| pin(0))).collect(), params, blob };
    let source = Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![5.0], blob: vec![] };
    let p = Program { node_count: 2, options: Options::default(), digital_slot_count: 8, channel_slot_count: 0, devices: vec![source, eeprom] };
    let e = Engine::load(&p).unwrap();
    let mut table = SnapshotTable::default();
    assert_eq!(table.bytes(), 0.0);
    let a = table.take(&e);
    let one = table.bytes();
    assert!(one > 2048.0, "a copy and the image: {one}");
    let b = table.take(&e);
    let two = table.bytes();
    let alone = table.walk_one(b).unwrap() as f64;
    assert!(alone > 2048.0, "walked alone, b counts the image: {alone}");
    assert!(two - one <= alone - 2048.0, "with a, the image is already counted: {one} then {two}, b alone {alone}");
    assert!(table.drop_snapshot(a) && table.drop_snapshot(b));
    assert!(!table.drop_snapshot(a), "a second drop is refused");
    assert_eq!((table.bytes(), table.len()), (0.0, 0));
}

/// Boards for the size checks: an RC network with a 555 and a CC rail
/// (2575 tpl 2), a 74LS04 behind a tripped fuse (the fuse ring), and the
/// rollback rig's resistor, LED and 74LS00 before the latch.
fn size_boards() -> Vec<(&'static str, Program, Vec<f64>)> {
    vec![
        ("2575 tpl 2", cold_2575_tpl2(), STOPS_2575[..30].to_vec()),
        // From 20 ms: the fuse's trip at about 17 ms rewrites its stamps
        // (a latch, as an edit does), which the sharing check leaves out.
        ("fuse ring", ring_behind_a_fuse(), (4..=10).map(|k| k as f64 * 5e-3).collect()),
        ("rollback rig", rollback_rig(), (1..=4).map(|k| k as f64 * 0.125).collect()),
    ]
}

#[test]
fn the_walk_counts_what_a_copy_allocates() {
    // `bo_engine_snapshot_bytes` walks the snapshots instead of counting
    // allocations (which cost about 1 % on every allocation of the advance
    // path). Held to the counter here: a snapshot's own heap, walked after
    // the live engine has marked everything the two share, is what its
    // copy allocated, within the hash tables' bucket estimate.
    for (name, p, stops) in size_boards() {
        let mut e = Engine::load(&p).unwrap();
        for &t in &stops {
            e.advance(t).unwrap();
        }
        e.clear_test_logs();
        let before = net_allocated();
        let copy = Box::new(e.snapshot());
        let counted = (net_allocated() - before) as f64;
        let mut w = Walk::default();
        e.heap_bytes(&mut w);
        let walked = ((*copy).heap_bytes(&mut w) + std::mem::size_of::<Engine>()) as f64;
        assert!((walked - counted).abs() <= 0.03 * counted + 512.0, "{name}: walked {walked} B, the copy allocated {counted} B");
    }
}

/// Every shared table of the solver and its three sub-solvers, by address.
fn shared_tables(e: &Engine) -> Vec<(String, usize)> {
    let s = e.debug_solver();
    let tag = |who: &str, v: Vec<(&'static str, usize)>| v.into_iter().map(|(n, a)| (format!("{who}.{n}"), a)).collect::<Vec<_>>();
    [tag("solver", s.shared_tables()), tag("static", s.static_solver.shared_tables()), tag("stepped", s.stepped.shared_tables()), tag("closed_form", s.closed_form.shared_tables())].concat()
}

#[test]
fn snapshots_share_the_tables_fixed_at_load_and_the_run_never_copies_them() {
    // Step-back keeps a snapshot a frame, so each must copy only the state
    // the run changes: the tables built at load (stamps, IC records, node
    // maps, the slow model) are shared by the live engine and every
    // snapshot. A run that wrote one would copy it at its first write after
    // each snapshot, so after running on, every table must still be the
    // snapshot's own allocation. The rollback rig is left out: its LED's
    // junction law re-arms as it heats (W1g), a write to the junction tables
    // that copies them once, as a latch or a host edit does.
    for (name, p, stops) in size_boards().into_iter().filter(|b| b.0 != "rollback rig") {
        let mut e = Engine::load(&p).unwrap();
        let half = stops.len() / 2;
        for &t in &stops[..half] {
            e.advance(t).unwrap();
        }
        let snapshot = e.snapshot();
        let tables = shared_tables(&snapshot);
        assert!(tables.len() >= 60, "{name}: {} shared tables", tables.len());
        for &t in &stops[half..] {
            e.advance(t).unwrap();
        }
        let moved: Vec<&String> = tables.iter().zip(shared_tables(&e)).filter(|(a, b)| a.1 != b.1).map(|(a, _)| &a.0).collect();
        assert!(moved.is_empty(), "{name}: the run copied {moved:?}");
    }
}





#[test]
fn an_engine_restored_at_every_stop_with_its_scratch_emptied_runs_as_one_never_restored() {
    // Scratch buffers (the kernel's delta lists, the solver's partition and
    // reader lists, the KCL fold's pair) copy empty: a snapshot holds none of
    // them. Restoring at every stop hands the engine empty buffers each time;
    // each use starts from an empty buffer anyway (`Scratch::take` asserts
    // it in debug builds), so the run must match an uninterrupted one bit
    // for bit.
    let t2575 = [(0.030082069747425742, 14), (0.3166121668293984, 21)];
    let boards: Vec<(&str, Program, Vec<(f64, usize)>, Vec<f64>)> = vec![
        ("2575 tpl 2", cold_2575_tpl2(), t2575.to_vec(), STOPS_2575[..40].to_vec()),
        ("fuse ring", ring_behind_a_fuse(), vec![], (1..=12).map(|k| k as f64 * 5e-3).collect()),
        ("287 tpl 3", cold_287_tpl3(), vec![], vec![0.006272879077239732, 0.0104, 0.0105, 0.011, 0.014446754687130081, 0.02]),
    ];
    for (name, p, toggles, stops) in &boards {
        let state = || Host { state: p.devices.iter().map(|d| d.params.first().copied().unwrap_or(0.0)).collect(), next: 0 };
        let mut plain = Engine::load(p).unwrap();
        let expected = run(&mut plain, &mut state(), toggles, stops);
        let mut e = Engine::load(p).unwrap();
        let mut host = state();
        let mut table = SnapshotTable::default();
        let mut rows = Vec::new();
        for k in 0..stops.len() {
            let id = table.take(&e);
            assert!(table.restore(&mut e, id), "{name}");
            assert!(table.drop_snapshot(id));
            rows.extend(run(&mut e, &mut host, toggles, &stops[k..=k]));
        }
        assert_eq!(rows.len(), expected.len(), "{name}: stops reached");
        for (j, (a, b)) in expected.iter().zip(&rows).enumerate() {
            assert!(a == b, "{name}: stop {j} at {} differs", stops[j]);
        }
    }
}

/// W2aV's adversarial round trips (the verifier's own test, kept): a snapshot
/// at every host stop, then 24 restores to random snapshots, each run on for
/// a random number of stops (with a live edit every third round where the
/// board has one), and finally every snapshot run to the end; every stop must
/// match a fresh run bit for bit, and no stat may ever go down. Trimmed to
/// run in seconds.
mod random_restores {
    use super::super::rail_tests::{cold_1838_tpl1, cold_2575_tpl2, cold_287_tpl3, cold_474_tpl5, cold_757_tpl0, STOPS_2575};
    use super::super::slow_tests::{ring_behind_a_fuse, rollback_rig};
    use crate::engine::Engine;
    use crate::program::Program;
    use crate::snapshot::SnapshotTable;

    fn seen(e: &mut Engine) -> Vec<u64> {
        let mut row: Vec<u64> = vec![e.time().to_bits()];
        row.extend(e.voltages().iter().chain(e.currents()).chain(e.digital_slots()).chain(e.channel_slots()).map(|v| v.to_bits()));
        for channel in 0..4 {
            let records = e.records(channel).unwrap().to_vec();
            row.push(records.len() as u64);
            row.extend(records.iter().map(|v| v.to_bits()));
        }
        let eeprom = e.take_eeprom_updates().to_vec();
        row.push(eeprom.len() as u64);
        row.extend(eeprom.iter().map(|v| v.to_bits()));
        row.push(e.next_event_time().to_bits());
        row.push(e.next_digital_due_time().to_bits());
        row
    }

    #[derive(Clone)]
    struct Host {
        state: Vec<f64>,
        next: usize,
    }

    fn step(e: &mut Engine, host: &mut Host, toggles: &[(f64, usize)], stop: f64) -> Vec<u64> {
        e.advance(stop).unwrap();
        let row = seen(e);
        while host.next < toggles.len() && toggles[host.next].0 <= stop {
            let d = toggles[host.next].1;
            host.state[d] = 1.0 - host.state[d];
            e.set_params(&[(d as u32, 0, host.state[d])]).unwrap();
            host.next += 1;
        }
        row
    }

    fn stats(e: &Engine) -> Vec<f64> {
        (0..28).map(|i| e.stat(i)).collect()
    }

    fn check_monotone(name: &str, prev: &mut Vec<f64>, e: &Engine, what: &str) {
        let now = stats(e);
        for i in 0..28 {
            assert!(now[i].is_nan() || prev[i].is_nan() || now[i] >= prev[i], "{name}: stat {i} went {} -> {} at {what}", prev[i], now[i]);
        }
        *prev = now;
    }

    fn board(name: &str, p: &Program, toggles: &[(f64, usize)], stops: &[f64], mutate: Option<(u32, u32)>) {
        let fresh = || Host { state: p.devices.iter().map(|d| d.params.first().copied().unwrap_or(0.0)).collect(), next: 0 };
        let mut plain = Engine::load(p).unwrap();
        let mut ph = fresh();
        let expected: Vec<Vec<u64>> = stops.iter().map(|&s| step(&mut plain, &mut ph, toggles, s)).collect();

        let mut e = Engine::load(p).unwrap();
        let mut host = fresh();
        let mut table = SnapshotTable::default();
        let mut snaps: Vec<(u32, Host)> = Vec::new();
        let mut prev = stats(&e);
        for (k, &s) in stops.iter().enumerate() {
            let row = step(&mut e, &mut host, toggles, s);
            assert!(row == expected[k], "{name}: taking snapshots perturbed stop {k}");
            snaps.push((table.take(&e), host.clone()));
            check_monotone(name, &mut prev, &e, "snapshot");
        }
        let n = stops.len();
        let mut rng: u64 = 0x9E3779B97F4A7C15;
        let mut next = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            rng
        };
        for round in 0..24 {
            let j = (next() % n as u64) as usize;
            assert!(table.restore(&mut e, snaps[j].0));
            check_monotone(name, &mut prev, &e, "restore");
            let mut h = snaps[j].1.clone();
            let reach = ((next() % (n - j) as u64) as usize).min(n - j - 1);
            for k in j + 1..=(j + reach).min(n - 1) {
                let row = step(&mut e, &mut h, toggles, stops[k]);
                assert!(row == expected[k], "{name}: round {round}: after restoring snapshot {j}, stop {k} differs");
            }
            check_monotone(name, &mut prev, &e, "run");
            if let Some((d, pi)) = mutate {
                if round % 3 == 0 {
                    let _ = e.set_params(&[(d, pi, 1.0)]);
                }
            }
        }
        for j in 0..n {
            assert!(table.restore(&mut e, snaps[j].0));
            check_monotone(name, &mut prev, &e, "final restore");
            let mut h = snaps[j].1.clone();
            for k in j + 1..n {
                let row = step(&mut e, &mut h, toggles, stops[k]);
                assert!(row == expected[k], "{name}: final: snapshot {j}, stop {k} differs");
            }
        }
        for (id, _) in &snaps {
            assert!(table.drop_snapshot(*id));
        }
        assert_eq!(table.bytes(), 0.0, "{name}: table empties");
    }

    #[test]
    fn snapshots_at_every_stop_survive_random_restores_with_live_edits() {
        let stops757 = [0.006272881300476146, 0.011116023500639817, 0.012362155978723055, 0.037731840680085206, 0.03788763379837714, 0.053120364878471986, 0.07025570911480136];
        board("757/0", &cold_757_tpl0(), &[], &stops757, None);
        let t2575 = [(0.030082069747425742, 14), (0.3166121668293984, 21)];
        board("2575/2", &cold_2575_tpl2(), &t2575, &STOPS_2575[..32], Some((14, 0)));
        let stops287 = [0.006272879077239732, 0.0104, 0.010400001, 0.0104001, 0.0105, 0.011, 0.014446754687130081, 0.02, 0.04];
        board("287/3", &cold_287_tpl3(), &[], &stops287, None);
        board("474/5", &cold_474_tpl5(), &[], &[0.0209, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05], None);
        let stops_fuse: Vec<f64> = (1..=14).map(|k| k as f64 * 5e-3).collect();
        board("fuse", &ring_behind_a_fuse(), &[], &stops_fuse, Some((4, 0)));
        let (p, t, s) = cold_1838_tpl1();
        let half = s.len() / 2;
        board("1838/1", &p, &t, &s[..half], None);
        let stops_rig: Vec<f64> = (1..=8).map(|k| k as f64 * 0.125).collect();
        board("rollback rig", &rollback_rig(), &[], &stops_rig, None);
    }
}
