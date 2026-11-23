//! A bench supply's exit from current limit with a loop of capacitors of no
//! resistance through its rail (ruling R68's board: a rail capacitor, and a
//! stepped hub partition joined to the rail and to ground by capacitors of
//! no esr). The last CC step is cut where its rail is just past the setpoint,
//! within `CROSS_TOLERANCE_V` (or within the cut's shortest bracket), and the
//! partition's charges hold that overshoot `d`. At the exit its CV seed holds
//! the rail at the setpoint, so the loop's capacitors no longer sum to it,
//! and the branch formulation of a loop of esr-free capacitors is singular:
//! the seed kept its companion's current, the loop's series capacitance over
//! `SEED_H` times `d`, which the supply's KCL read as a sink. On this board
//! `d` = 2.3 pV read as 1.6 uA, and the supply declined as back-driven
//! (`bench-psu-back-driven`) at host cadences that put `d` past 1.5 pV.
//!
//! The truth is the closed form: in CC the rail ramps at `I / (C10 + Cs)`,
//! `Cs` the series pair, its node at `C15 / (C15 + C50)` of the rail; from
//! the exit at `V (C10 + Cs) / I` the rail holds the setpoint and no current
//! flows.
use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

const V: f64 = 5.164233;
const I_LIM: f64 = 0.66476;
const C10: f64 = 1.857e-6;
const C15: f64 = 7.812e-5;
const C50: f64 = 6.926e-7;

fn pin(n: usize) -> PinRecord {
    PinRecord { node: n, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob: vec![] }
}

/// Nodes: 0 ground, 1 the rail, 2 the loop's node. Devices: 0 PSU, 1 C10
/// rail-gnd, 2 C15 rail-n2, 3 C50 n2-gnd.
fn board() -> Program {
    let devices = vec![
        dev(kinds::BENCH_PSU, 0, &[1, 0], vec![V, I_LIM]),
        dev(kinds::CAPACITOR, 1, &[1, 0], vec![C10, 0.0, 0.0]),
        dev(kinds::CAPACITOR, 2, &[1, 2], vec![C15, 0.0, 0.0]),
        dev(kinds::CAPACITOR, 3, &[2, 0], vec![C50, 0.0, 0.0]),
    ];
    Program { carry: Vec::new(), node_count: 3, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
}

/// The readings at `t`: node voltages by node, currents by device (the
/// supply's published leaving its + pin into the rail as negative).
fn truth(t: f64) -> ([f64; 3], [f64; 4]) {
    let cs = C15 * C50 / (C15 + C50);
    let ct = C10 + cs;
    let share = C15 / (C15 + C50);
    let t_exit = V * ct / I_LIM;
    if t < t_exit {
        let u = I_LIM * t / ct;
        ([0.0, u, share * u], [-I_LIM, C10 * I_LIM / ct, cs * I_LIM / ct, cs * I_LIM / ct])
    } else {
        ([0.0, V, share * V], [0.0; 4])
    }
}

fn w0v(p: f64, q: f64) -> f64 {
    (p - q).abs() / (1e-4 + 1e-3 * q.abs())
}

fn w0i(p: f64, q: f64) -> f64 {
    (p - q).abs() / (1e-9 + 1e-3 * q.abs())
}

/// Each stop's worst reading in W0 of the closed form, and what it was.
fn offsets(stops: &[f64]) -> Vec<(f64, f64, String)> {
    let mut e = Engine::load(&board()).expect("loads");
    let mut out = Vec::new();
    for &t in stops {
        if let Err(er) = e.advance(t) {
            panic!("advance to {t} s failed: {} ({}); so far {out:?}", er.code, er.message);
        }
        let (v, i) = (e.voltages().to_vec(), e.currents().to_vec());
        let (tv, ti) = truth(t);
        let (mut worst, mut what) = (0.0f64, String::new());
        for n in 1..3 {
            let x = w0v(v[n], tv[n]);
            if x > worst {
                (worst, what) = (x, format!("v{n} {:.12} truth {:.12}", v[n], tv[n]));
            }
        }
        for d in 0..4 {
            let x = w0i(i[d], ti[d]);
            if x > worst {
                (worst, what) = (x, format!("i{d} {:.6e} truth {:.6e}", i[d], ti[d]));
            }
        }
        out.push((t, worst, what));
    }
    out
}

/// Three host cadences, each of which cut the last CC step past the setpoint
/// by more than 1.5 pV (2.3, 2.6 and 4.4 pV): the first has a stop 0.6 ps
/// after the exit (19.7594564 us).
const CADENCES: [&[f64]; 3] = [
    &[1.5e-5, 1.9759457e-5, 2e-5, 1e-4],
    &[1e-5, 1.5e-5, 1.976e-5, 2e-5, 1e-4, 1e-3],
    &[1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 2e-5, 1e-4, 1e-3],
];

#[test]
fn a_cc_exit_through_a_capacitor_loop_of_no_esr_leaves_the_supply_sourcing_and_reads_the_closed_form() {
    for stops in CADENCES {
        let offs = offsets(stops);
        for (t, w, what) in &offs {
            assert!(*w <= 1.0, "stops {stops:?}, at {t} s: {w:.3} W0 off the closed form ({what}); all: {offs:?}");
        }
    }
}

// ---- the tie's limits (`stepped::tie_rail_loops`, `hub.rs`'s `exit_tie`) ----
//
// Each is pinned by what the engine reads against the same build with the
// tie off (`tie_hook::off`, a test-only switch beside the code): bit for bit,
// the tie leaving the exit as the build before it read it. And by what the
// tie itself judged (`tie_hook::judged`, `exits`), so that a board that stops
// reaching the exit it is built for fails on that, not by agreeing.
use crate::solve::stepped::tie_hook;

/// Every reading after each stop, as bits, or the decline that ended the run.
fn bits(p: &Program, stops: &[f64]) -> Vec<Result<Vec<u64>, String>> {
    let mut e = Engine::load(p).expect("loads");
    let mut out = Vec::new();
    for &t in stops {
        if let Err(er) = e.advance(t) {
            out.push(Err(format!("{} ({})", er.code, er.message)));
            break;
        }
        out.push(Ok(e.voltages().iter().chain(e.currents()).map(|x| x.to_bits()).collect()));
    }
    out
}

/// The readings with the tie on and with it off, and what the tie judged and
/// where the exits fell (of the run with it on).
fn on_and_off(p: &Program, stops: &[f64]) -> (Vec<Result<Vec<u64>, String>>, Vec<Result<Vec<u64>, String>>, Vec<(f64, f64, bool, bool)>, Vec<(bool, bool)>) {
    tie_hook::reset();
    let on = bits(p, stops);
    let (judged, exits) = (tie_hook::judged(), tie_hook::exits());
    let off = {
        let _off = tie_hook::off();
        bits(p, stops)
    };
    (on, off, judged, exits)
}

#[test]
fn a_cc_exit_past_the_cuts_overshoot_keeps_its_charge() {
    // Cadence [1.8e-7, 2e-5, 1e-4]: the exit's loop charges hold the rail
    // 2.1e-7 V past the setpoint, 41 times the bound `EXIT_OVER_REL` sets
    // (5.2e-9 V): the CC phase's own overshoot, which CV pulls back. The
    // first, unbounded form tied it (202/5 moved by 3.1 V, 517/5 by 100 uV).
    let (on, off, judged, _) = on_and_off(&board(), &[1.8e-7, 2e-5, 1e-4]);
    let far: Vec<_> = judged.iter().filter(|j| j.0 > j.1).collect();
    assert!(!far.is_empty(), "the board must reach a loop past the bound; judged {judged:?}");
    assert!(far.iter().all(|j| !j.3), "a loop past the bound was tied: {far:?}");
    assert!(on.iter().all(|r| r.is_ok()), "{on:?}");
    assert_eq!(on, off, "the readings parted from the tie-off build; judged {judged:?}");
}

#[test]
fn a_cc_exit_short_of_the_seeds_rail_keeps_its_charge() {
    // Cadence [4.996e-6, 1.9759457e-5, 2e-5]: the first exit's loop charges
    // hold the rail 1.5e-4 V short of the setpoint the seed holds it at. A
    // charge short of the rail is the circuit's own.
    let (on, off, judged, _) = on_and_off(&board(), &[4.996e-6, 1.9759457e-5, 2e-5]);
    let short: Vec<_> = judged.iter().filter(|j| j.0 <= 0.0).collect();
    assert!(!short.is_empty(), "the board must reach a loop short of the rail; judged {judged:?}");
    assert!(short.iter().all(|j| !j.3), "a loop short of the rail was tied: {short:?}");
    assert_eq!(on, off, "the readings parted from the tie-off build; judged {judged:?}");
}

/// A 5 V, 0.1 A supply charging a 1 mF rail capacitor (esr 5 ohm) in CC
/// against a 2.2 ohm load to a pulse source that rises to 8 V at 5 ms (the
/// back-driven edge of `rail_tests`'s FXI board), and the loop of
/// esr-free capacitors through the rail the tie is for.
fn edge_board() -> Program {
    let devices = vec![
        dev(kinds::BENCH_PSU, 0, &[1, 0], vec![5.0, 0.1]),
        dev(kinds::CAPACITOR, 1, &[1, 0], vec![1e-3, 5.0, 0.0]),
        Device { kind: kinds::PULSE_SOURCE, component_index: 2, pins: vec![pin(2), pin(0)], params: vec![0.0, 8.0, 5e-3, 0.0, 0.0, 2e-4, 20e-3], blob: vec![0.0] },
        dev(kinds::RESISTOR, 3, &[2, 1], vec![2.2]),
        dev(kinds::CAPACITOR, 4, &[1, 3], vec![7.8e-6, 0.0, 0.0]),
        dev(kinds::CAPACITOR, 5, &[3, 0], vec![6.9e-7, 0.0, 0.0]),
    ];
    Program { carry: Vec::new(), node_count: 4, options: Options::default(), digital_slot_count: 1, channel_slot_count: 0, devices }
}

#[test]
fn a_cc_exit_at_a_source_edge_is_not_tied() {
    // The pulse's edge at 5 ms back-drives the rail through the capacitor's
    // esr past the setpoint: the exit falls at a step of the draws
    // (`rail_jump`), where the charges are the circuit's.
    let (on, off, _, exits) = on_and_off(&edge_board(), &[4e-3, 5e-3]);
    let at_edge: Vec<_> = exits.iter().filter(|e| e.0).collect();
    assert!(!at_edge.is_empty(), "the board must exit CC at the edge; exits {exits:?}");
    assert!(at_edge.iter().all(|e| !e.1), "the tie was armed at a source edge: {exits:?}");
    assert_eq!(on, off, "the readings parted from the tie-off build");
}

#[test]
fn a_cc_exit_in_exact_rail_mode_is_not_tied() {
    // Exact mode holds the rail in a partition (no hub), so no exit arms the
    // tie and no partition is judged (the call's own `exact` guard is
    // unreachable through the engine, so this pins the mode, not that line;
    // its mutation does not fail here); the readings (and where a cadence
    // declines, as the first does at its exit) are the tie-off build's.
    let mut p = board();
    p.options.no_rail_shortcut = true;
    for stops in CADENCES {
        let (on, off, judged, exits) = on_and_off(&p, stops);
        assert!(judged.is_empty() && exits.is_empty() && tie_hook::calls() == 0, "exact mode reached the tie: judged {judged:?}, exits {exits:?}");
        assert_eq!(on, off, "stops {stops:?}: the readings parted from the tie-off build");
    }
}
