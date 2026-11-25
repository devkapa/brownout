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


/// R127's board 25 (`crossing_tests::property_board(25)`): a 555
/// monostable (RT 9241 ohm, 915 nF) fired by a 38 Hz clock, on a 0.109 Ah,
/// 1.48 ohm cell at 46 % with 13.6 kohm across the rail.
fn monostable_on_a_cell() -> Program {
    let devices = vec![
        dev(kinds::BATTERY_PACK, 0, &[1, 0], vec![1.0, 1.477753555571446, 1.0, 0.0, 2.0, 0.05, 0.10925759971414718, 0.45754411553890756, 0.0], vec![2.0, 0.0, 3.4074, 1.0, 4.8837]),
        r(1, 1, 0, 13620.925221109494),
        n555(2, 5, 4, 6, 3, 3, 8, 2.0),
        r(3, 1, 3, 9241.414456884317),
        c(4, 3, 0, 9.150985659803093e-7),
        c(5, 6, 0, 1e-8),
        dev(kinds::CLOCK, 6, &[5, 0], vec![37.98432881095158, 0.9, 5.0, 0.0, 0.0], vec![1.0]),
    ];
    Program { carry: Vec::new(), node_count: 7, options: Options::default(), digital_slot_count: 64, channel_slot_count: 0, devices }
}

#[test]
fn a_held_move_whose_path_turned_acts() {
    // A solve can turn the rail's path with no move of its value: the clock
    // fires the monostable, its timing capacitor starts drawing through RT,
    // and the cell's coupled path turns from -2.1e-5 V/s to +0.077 V/s
    // while the rail moves 3 pV. The timing capacitor follows the path its
    // aim took; held on the old one past the next clock edge (2.6 ms), the
    // rail it charged from stood 0.17 mV low and the pulse ended 7.3e-8 s
    // late at any budget above 0 (R127's board 25, 1.46e-6 of its period).
    // By hand: a held aim stands on a path within eps of the settled one
    // until the next action, and an aim made in between on one within eps
    // of that, so the capacitor's drive is at most 2 eps off; the pulse ends
    // at 2/3 of the rail, where the capacitor rises at (u / 3) / (RT C) =
    // 1.358 / 8.457e-3 = 160.6 V/s, so its end moves at most 2 eps / 160.6
    // V/s: 1.25e-11 s at 1e-9 V, 1.25e-9 s at 1e-7 V.
    let p = monostable_on_a_cell();
    let h = 1.0 / 60.0;
    let (_, a, _) = run(&p, -1.0, h, 5.0 * h);
    assert_eq!(a.len(), 6, "the monostable's edges");
    for (eps, allowance) in [(1e-9, 1.25e-11), (1e-7, 1.25e-9)] {
        let (_, b, _) = run(&p, eps, h, 5.0 * h);
        assert_eq!(a.len(), b.len(), "eps {eps:e}: an edge lost or gained");
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(x.1, y.1);
            assert!((x.0 - y.0).abs() <= allowance, "eps {eps:e}: an edge at {} s against {} s off, over {allowance:e} s", y.0, x.0);
        }
    }
}

#[test]
fn a_hub_with_a_stepped_partition_does_not_coalesce() {
    // A stepped hub partition reads the rail as the boundary of its steps;
    // a held rail changes its step sequence and an oscillator's flips with
    // it (the sweeps' 153/5 and 174/2), so such a hub acts on every move:
    // the board runs bit for bit alike with the budget on and off. The
    // board is the fast-draining 555 board with a junction and a capacitor
    // in one partition off the rail (a nonlinear capacitive partition: its
    // own stepped integrator).
    let mut p = board(2.0);
    p.devices.extend([r(11, 1, 8, 470.0), c(12, 8, 0, 2.2e-6), red_led(13, 8, 0)]);
    p.node_count = 9;
    let mut e = load(&p, 1e-5);
    let s = e.debug_solver_concrete_mut();
    let part = s.local_index[8].expect("node 8 is dynamic").0;
    assert_eq!(s.partition_kinds[part], super::PartitionKind::Stepped, "the junction's partition steps");
    assert!(!s.coalescing(), "a hub with a stepped partition does not coalesce");
    let h = 1e-3;
    let (off, a, _) = run(&p, -1.0, h, 0.02);
    for eps in [0.0, 1e-7, 1e-5] {
        let (on, b, held) = run(&p, eps, h, 0.02);
        assert_eq!(held, 0.0, "eps {eps:e}: a move held");
        assert_eq!(on.stat(29), on.stat(14), "eps {eps:e}: every move acts");
        assert_eq!(a.len(), b.len(), "eps {eps:e}: an edge lost or gained");
        for (x, y) in a.iter().zip(&b) {
            assert_eq!((x.0.to_bits(), x.1.to_bits()), (y.0.to_bits(), y.1.to_bits()), "eps {eps:e}: an edge moved");
        }
        let (va, vb) = (off.voltages(), on.voltages());
        assert!(va.iter().zip(vb).all(|(x, y)| x.to_bits() == y.to_bits()), "eps {eps:e}: a reading moved");
        assert_eq!(off.stat(14), on.stat(14));
    }
}

#[test]
fn a_hub_carrying_a_winding_does_not_coalesce() {
    // P0-MAG: an inductor's partition is stepped whatever else it holds (R56
    // 6), so a battery hub carrying one, off the rail behind 470 ohm, acts on
    // every move with coalescing switched on, as any hub with a stepped
    // partition (`a_hub_with_a_stepped_partition_does_not_coalesce`): the board
    // runs bit for bit alike with the budget on and off.
    let mut p = board(2.0);
    p.devices.extend([r(11, 1, 8, 470.0), dev(kinds::INDUCTOR, 12, &[8, 0], vec![10e-3, 5.0, 0.0], vec![])]);
    p.node_count = 9;
    let mut e = load(&p, 1e-5);
    let s = e.debug_solver_concrete_mut();
    let part = s.local_index[8].expect("node 8 is dynamic").0;
    assert_eq!(s.partition_kinds[part], super::PartitionKind::Stepped, "the winding's partition steps");
    assert!(!s.coalescing(), "a hub carrying a winding does not coalesce");
    let h = 1e-3;
    let (off, a, _) = run(&p, -1.0, h, 0.02);
    for eps in [0.0, 1e-7, 1e-5] {
        let (on, b, held) = run(&p, eps, h, 0.02);
        assert_eq!(held, 0.0, "eps {eps:e}: a move held");
        assert_eq!(on.stat(29), on.stat(14), "eps {eps:e}: every move acts");
        assert_eq!(a.len(), b.len(), "eps {eps:e}: an edge lost or gained");
        for (x, y) in a.iter().zip(&b) {
            assert_eq!((x.0.to_bits(), x.1.to_bits()), (y.0.to_bits(), y.1.to_bits()), "eps {eps:e}: an edge moved");
        }
        let (va, vb) = (off.voltages(), on.voltages());
        assert!(va.iter().zip(vb).all(|(x, y)| x.to_bits() == y.to_bits()), "eps {eps:e}: a reading moved");
    }
}

/// The coalescing board with, apart from the rail, a self-heating NTC behind
/// its own 5 V source (node 8 -> 100 ohm -> node 9 -> thermistor, 0.2 s body):
/// a thermistor on the rail's partition declines (R56 5's port, W1h H4c), so
/// this is the only way one rides on a battery board.
fn board_with_a_thermistor(cache_off: bool) -> Program {
    let mut p = board(2.0);
    p.devices.extend([
        dev(kinds::VOLTAGE_SOURCE, 11, &[8, 0], vec![5.0], vec![]),
        r(12, 8, 9, 100.0),
        dev(kinds::THERMISTOR, 13, &[9, 0], vec![100.0, 3950.0, 0.007, 0.2, 25.0], vec![]),
    ]);
    p.node_count = 10;
    p.options.no_solution_cache = cache_off;
    p
}

#[test]
fn a_thermistor_on_the_rail_declines_and_one_apart_from_it_runs_alike_with_coalescing_on_and_off() {
    // P0-MAG: a hub carrying a thermistor never coalesces because it never
    // loads (the thermistor on the rail's partition falls back to classic); one
    // in its own partition beside the battery hub leaves the hub's coalescing
    // as it was, and the board reads bit for bit alike with the budget on and
    // off, the body temperature included.
    let mut on_rail = board(2.0);
    on_rail.devices.extend([r(11, 1, 8, 470.0), dev(kinds::THERMISTOR, 12, &[8, 0], vec![1000.0, 3950.0, 0.007, 15.0, 25.0], vec![])]);
    on_rail.node_count = 9;
    assert_eq!(Engine::load(&on_rail).err().map(|e| e.code), Some(crate::error::ERR_UNSUPPORTED_TOPOLOGY));
    let p = board_with_a_thermistor(false);
    let mut base = load(&board(2.0), 1e-5);
    let mut with = load(&p, 1e-5);
    assert_eq!(with.debug_solver_concrete_mut().coalescing(), base.debug_solver_concrete_mut().coalescing(), "the thermistor apart leaves the hub's coalescing as it was");
    let h = 1e-3;
    let (mut off, a, _) = run(&p, -1.0, h, 0.05);
    for eps in [0.0, 1e-5] {
        let (mut on, b, _) = run(&p, eps, h, 0.05);
        assert_eq!(a.len(), b.len(), "eps {eps:e}: an edge lost or gained");
        let body = |e: &mut Engine| e.device_state(13).unwrap()[0];
        let (t_off, t_on) = (body(&mut off), body(&mut on));
        assert!(t_off > 25.5, "the body warmed: {t_off}");
        let (va, vb) = (off.voltages(), on.voltages());
        assert_eq!(va[9].to_bits(), vb[9].to_bits(), "eps {eps:e}: the thermistor's node moved");
        assert_eq!(t_on.to_bits(), t_off.to_bits(), "eps {eps:e}: the body moved");
    }
}

#[test]
fn a_self_heating_thermistor_on_a_battery_board_reads_the_same_with_the_solution_cache_on_and_off() {
    // The law-keyed solution cache keys a static partition's stamped laws; a
    // thermistor's R(T) moves with its body, but its partition is always
    // stepped (R56 5), so no value of it enters a cached stamp. The battery
    // board (whose static partitions the cache serves) reads the same either
    // way, the thermistor's body and current within R130's thermal tolerance
    // and W0.
    let run_to = |cache_off: bool| {
        let mut e = Engine::load(&board_with_a_thermistor(cache_off)).unwrap();
        let mut out = Vec::new();
        for k in 1..=10 {
            e.advance(k as f64 * 0.05).unwrap();
            out.push((e.device_state(13).unwrap()[0], e.currents()[13], e.voltages().to_vec()));
        }
        out
    };
    let (on, off) = (run_to(false), run_to(true));
    for (k, ((t_on, i_on, v_on), (t_off, i_off, v_off))) in on.iter().zip(&off).enumerate() {
        let rise = (t_off - 25.0).abs();
        assert!(k == 0 || rise > 0.5, "the body warms: {t_off}");
        assert!((t_on - t_off).abs() <= 1e-3 + 1e-3 * rise, "sample {k}: T {t_on} cached against {t_off}");
        assert!((i_on - i_off).abs() <= 1e-9 + 1e-3 * i_off.abs(), "sample {k}: I {i_on} against {i_off}");
        for (n, (a, b)) in v_on.iter().zip(v_off).enumerate() {
            assert!((a - b).abs() <= 100e-6 + 1e-3 * b.abs(), "sample {k} node {n}: {a} against {b}");
        }
    }
}

/// P0-MAGV V6: a thermistor apart from a battery hub, on the board and the numbers the porters did not use (12 V and
/// 47 ohm behind a 4.7 uF, a 470 ohm / 4100 K body with a 0.1 s lag), under the law-keyed cache on and off and
/// coalescing off and at 1e-7 V: the thermistor's node, its body temperature and its current agree within the
/// stepped controller's own bar (100 uV + 1e-3 relative; 1e-3 K + 1e-3 relative; 1 nA + 1e-3 relative).
#[test]
fn magv_a_thermistor_beside_a_battery_hub_reads_alike_with_the_cache_and_coalescing_on_and_off() {
    let mut p = board(2.0);
    p.devices.extend([
        dev(kinds::VOLTAGE_SOURCE, 11, &[8, 0], vec![12.0], vec![]),
        r(12, 8, 9, 47.0),
        dev(kinds::THERMISTOR, 13, &[9, 0], vec![470.0, 4100.0, 0.01, 0.1, 25.0], vec![]),
        c(14, 9, 0, 4.7e-6),
    ]);
    p.node_count = 10;
    let mut runs = Vec::new();
    for (cache, eps) in [(true, -1.0), (false, -1.0), (true, 1e-7), (false, 1e-7)] {
        let mut e = load(&p, eps);
        e.debug_set_law_keyed_cache(cache);
        let mut samples = Vec::new();
        for k in 1..=20 {
            e.advance(k as f64 * 0.01).unwrap();
            samples.push((e.voltages()[9], e.device_state(13).unwrap()[0], e.currents()[13]));
        }
        runs.push(samples);
    }
    assert!(runs[0].last().unwrap().1 > 26.0, "the body warms: {}", runs[0].last().unwrap().1);
    let mut bitwise = true;
    for (which, run) in runs.iter().enumerate().skip(1) {
        for (k, (a, b)) in runs[0].iter().zip(run).enumerate() {
            bitwise &= a.0.to_bits() == b.0.to_bits() && a.1.to_bits() == b.1.to_bits() && a.2.to_bits() == b.2.to_bits();
            assert!((a.0 - b.0).abs() <= 100e-6 + 1e-3 * a.0.abs(), "run {which} sample {k}: node {} against {}", b.0, a.0);
            assert!((a.1 - b.1).abs() <= 1e-3 + 1e-3 * (a.1 - 25.0).abs(), "run {which} sample {k}: T {} against {}", b.1, a.1);
            assert!((a.2 - b.2).abs() <= 1e-9 + 1e-3 * a.2.abs(), "run {which} sample {k}: I {} against {}", b.2, a.2);
        }
    }
    eprintln!("MAGV thermistor beside hub: bitwise equal across the four runs: {bitwise}");
}
