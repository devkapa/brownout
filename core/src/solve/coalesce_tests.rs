//! Move coalescing (core/README.md, Rails point 3, "Rail moves"): the rail
//! loop settles on the exact rail as before; a move's actions run once the
//! loop settles, and only when the settled rail is more than the budget
//! `rail_act_eps` from the rail they last ran on, or when the loop read the
//! rail exactly. Each test runs one board with coalescing off (every move
//! acts, the behaviour before it) and on, in one build.

use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

fn pin(n: usize) -> PinRecord {
    PinRecord { node: n, flags: 0, open_hash: 0, dead_hash: 0 }
}
fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob }
}
fn r(ci: usize, a: usize, b: usize, ohm: f64) -> Device {
    dev(kinds::RESISTOR, ci, &[a, b], vec![ohm], vec![])
}
fn c(ci: usize, a: usize, b: usize, f: f64) -> Device {
    dev(kinds::CAPACITOR, ci, &[a, b], vec![f, 0.0, 0.0], vec![])
}
fn red_led(ci: usize, a: usize, k: usize) -> Device {
    dev(kinds::LED, ci, &[a, k], vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0], vec![])
}
/// A Li-ion-like cell (3.41 to 4.88 V) on node 1.
fn cell_li(rint: f64, cap_ah: f64) -> Device {
    dev(kinds::BATTERY_PACK, 0, &[1, 0], vec![1.0, rint, 1.0, 0.0, 2.0, 0.05, cap_ah, 1.0, 0.0], vec![2.0, 0.0, 3.4074, 1.0, 4.8837])
}
/// An NE555 on the rail (node 1), its `vccMin` given.
fn n555(ci: usize, trig: usize, out: usize, ctrl: usize, thr: usize, dis: usize, slot: usize, vcc_min: f64) -> Device {
    dev(kinds::NE555, ci, &[1, 0, trig, out, 1, ctrl, thr, dis], vec![vcc_min, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![slot as f64, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
}

/// R132's fast-draining board without its thermal records: a 10 uAh,
/// 0.148 ohm cell feeding a 555 astable (RA 2438, RB 1014 ohm, 68 nF) whose
/// output drives 150 ohm and a red LED, a second LED through 330 ohm from
/// the rail and a 220 ohm load. Each 555 flip steps the draw by about 19 mA,
/// so the rail moves at every flip, and the cell drains about 1.8 V/s.
fn board(vcc_min: f64) -> Program {
    let devices = vec![
        cell_li(0.148, 10e-6),
        n555(1, 3, 4, 6, 3, 2, 8, vcc_min),
        r(2, 1, 2, 2438.0),
        r(3, 2, 3, 1014.0),
        c(4, 3, 0, 6.8e-8),
        c(5, 6, 0, 1e-8),
        r(6, 4, 5, 150.0),
        red_led(7, 5, 0),
        r(8, 1, 7, 330.0),
        red_led(9, 7, 0),
        r(10, 1, 0, 220.0),
    ];
    Program { carry: Vec::new(), node_count: 8, options: Options::default(), digital_slot_count: 64, channel_slot_count: 0, devices }
}

const SLOT: f64 = 8.0;

fn load(p: &Program, eps: f64) -> Engine {
    let mut e = Engine::load(p).unwrap();
    e.debug_set_rail_act_eps(eps);
    e.trace_enable(true);
    e
}

/// The settled rail and the rail the actions last ran on.
fn rails(e: &mut Engine) -> (f64, f64) {
    let hub = e.debug_solver_concrete_mut().hub.as_ref().expect("a hub rail");
    (hub.state.u, hub.state.u_act)
}

/// The 555 output's edges `(t, value)` since the last call.
fn edges(e: &mut Engine) -> Vec<(f64, f64)> {
    let t: Vec<(f64, f64)> = e.trace().chunks(3).filter(|c| c[1] == SLOT).map(|c| (c[0], c[2])).collect();
    e.trace_clear();
    t
}

/// Runs to `end` at stops of `h`, checking at every stop that the published
/// rail is the rail the actions last ran on and within `eps` of the settled
/// one; returns the edges and the worst held offset.
fn run(p: &Program, eps: f64, h: f64, end: f64) -> (Engine, Vec<(f64, f64)>, f64) {
    let mut e = load(p, eps);
    let (mut out, mut worst) = (Vec::new(), 0.0f64);
    let n = (end / h).round() as usize;
    for k in 1..=n {
        e.advance(k as f64 * h).unwrap();
        out.extend(edges(&mut e));
        let (u, u_act) = rails(&mut e);
        let published = e.voltages()[1];
        assert_eq!(published.to_bits(), u_act.to_bits(), "at {} s the published rail {published} is not the acted one {u_act}", e.time());
        let off = (u - u_act).abs();
        if eps >= 0.0 {
            assert!(off <= eps, "at {} s the published rail is {off} V from the settled one, over the budget {eps}", e.time());
        } else {
            assert_eq!(off, 0.0, "with coalescing off every move acts");
        }
        worst = worst.max(off);
    }
    (e, out, worst)
}

#[test]
fn coalescing_acts_once_a_loop_and_holds_moves_within_the_budget() {
    // Off, every move acts (stat 29 = stat 14). On at 0 V, the moves of one
    // rail loop act once; at 1e-5 V the moves within the budget of the last
    // action do not act at all.
    let p = board(2.0);
    let (off, _, _) = run(&p, -1.0, 1e-3, 0.02);
    let (zero, _, _) = run(&p, 0.0, 1e-3, 0.02);
    let (band, _, held) = run(&p, 1e-5, 1e-3, 0.02);
    assert_eq!(off.stat(29), off.stat(14));
    assert!(off.stat(14) > 100.0, "{} moves: the board must move its rail", off.stat(14));
    assert!(zero.stat(29) < zero.stat(14), "eps 0: {} acts of {} moves", zero.stat(29), zero.stat(14));
    // Most of this board's moves are a 555 crossing's, which act (each
    // reads the rail exactly): only the others hold.
    assert!(band.stat(29) < zero.stat(29), "eps 1e-5: {} acts against {} at eps 0", band.stat(29), zero.stat(29));
    assert!(held > 0.0, "the budget never held a move");
}

#[test]
fn a_555_crossing_is_not_delayed_past_its_allowance() {
    // By hand: a held aim stands on a rail at most eps from the settled one;
    // the timing node's target and the 555's thresholds move with the rail
    // by factors in [0, 1], so a crossing moves by at most 2 eps over the
    // node's slope there. The slowest crossing is the charge through RA + RB
    // to 2/3 Vcc: (Vcc / 3) / ((RA + RB) C) = 1.6 / 2.35e-4 = 6.8e3 V/s at
    // 4.8 V (5.8e3 at 4.1 V): 2 x 1e-5 / 5.8e3 = 3.4e-9 s. A crossing itself
    // acts (its node lands on a threshold that moved with the rail's path),
    // so the measured shifts sit far inside.
    let p = board(2.0);
    let (_, a, _) = run(&p, -1.0, 1e-3, 0.05);
    let (_, b, _) = run(&p, 1e-5, 1e-3, 0.05);
    assert!(a.len() > 400, "{} edges", a.len());
    assert_eq!(a.len(), b.len(), "an edge lost or gained");
    let worst = a.iter().zip(&b).map(|(x, y)| {
        assert_eq!(x.1, y.1);
        (x.0 - y.0).abs()
    });
    let worst = worst.fold(0.0f64, f64::max);
    assert!(worst <= 3.4e-9, "worst edge shift {worst} s over the 3.4e-9 s allowance");
}

#[test]
fn a_power_flip_is_seen_at_once() {
    // The 555's power window ends at 0.9 vccMin = 4.32 V: the draining cell
    // crosses it, the chip stops and its output falls. A power flip reads
    // the rail exactly, so its loop acts: the 555's last edges match the run
    // with coalescing off to rounding (the edge where power fails included).
    let p = board(4.8);
    let (_, a, _) = run(&p, -1.0, 1e-3, 0.5);
    let (_, b, _) = run(&p, 1e-5, 1e-3, 0.5);
    let last = a.last().copied().expect("edges");
    assert!(last.0 < 0.45, "the rail must cross the 555's window before the end: last edge at {} s", last.0);
    assert_eq!(a.len(), b.len(), "an edge lost or gained");
    let (x, y) = (a.last().unwrap(), b.last().unwrap());
    // Unpowered, the output reads NaN.
    assert_eq!(x.1.to_bits(), y.1.to_bits());
    // Found on the exact rail at its `Rail` event, the flip is not held;
    // what moves it is the draw's history, the 555's earlier edges each
    // within 3.4e-9 s (above): measured 3.3e-12 s.
    assert!((x.0 - y.0).abs() <= 1e-10, "the power-fail edge at {} s against {} s", y.0, x.0);
}

#[test]
fn a_snapshot_carries_the_held_rail_and_runs_on_bit_for_bit() {
    let p = board(2.0);
    let mut e = load(&p, 1e-5);
    let mut held_at_snapshot = 0;
    for k in 1..=20 {
        e.advance(k as f64 * 1e-3).unwrap();
        let (u, u_act) = rails(&mut e);
        if u.to_bits() == u_act.to_bits() {
            continue;
        }
        held_at_snapshot += 1;
        let mut copy = e.snapshot();
        let mut go = e.snapshot();
        for j in 1..=5 {
            let t = k as f64 * 1e-3 + j as f64 * 2e-4;
            copy.advance(t).unwrap();
            go.advance(t).unwrap();
            let (va, vb) = (copy.voltages().to_vec(), go.voltages().to_vec());
            assert!(va.iter().zip(&vb).all(|(x, y)| x.to_bits() == y.to_bits()));
        }
        assert_eq!(rails(&mut copy), rails(&mut go));
    }
    assert!(held_at_snapshot > 0, "no stop held a move: the test means nothing");
}

#[test]
fn a_rolled_back_window_restores_the_held_rail() {
    let p = board(2.0);
    let mut e = load(&p, 1e-5);
    e.advance(5e-3).unwrap();
    let hub = e.debug_solver_concrete_mut().hub.as_mut().unwrap();
    let before = hub.state.clone();
    hub.begin_window();
    hub.state.u_act += 1e-6;
    hub.state.act_until = 1.0;
    hub.rollback_window();
    assert_eq!(hub.state, before);
}

#[test]
fn a_carried_reload_publishes_the_rail_it_settles_on() {
    // The carry holds no lag: a reload solves its rail and acts on it, and
    // from there holds within the budget again.
    let p = board(2.0);
    let mut a = load(&p, 1e-5);
    a.advance(0.01).unwrap();
    let mut q = p.clone();
    q.options.start_time = a.time();
    q.carry = a.carry_take().unwrap().to_vec();
    let mut b = load(&q, 1e-5);
    let (u, u_act) = rails(&mut b);
    assert_eq!(u.to_bits(), u_act.to_bits(), "a reload starts on the rail it settles on");
    for k in 1..=10 {
        b.advance(0.01 + k as f64 * 1e-3).unwrap();
        let (u, u_act) = rails(&mut b);
        assert!((u - u_act).abs() <= 1e-5);
        assert_eq!(b.voltages()[1].to_bits(), u_act.to_bits());
    }
}

