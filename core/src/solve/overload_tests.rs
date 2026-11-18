//! W1h H1c: the potentiometer's two-segment overload (ruling R56 2 (a)) and
//! the resistor array's per-channel overload, through a real `Engine`.
//!
//! Every expected number is classic's rule worked by hand for the circuit
//! (`commitResistiveOverloadStress` and `resistorArrayModel.updateFailures`,
//! passives.ts): a segment's ratio is `(dV^2 / rSeg) / (pMax max(rSeg /
//! (rCw + rCcw), 0.25))`, the rate `max(0, max(r_cw, r_ccw) - 1)` (tie to cw),
//! threshold 1, recovery 1; an array channel is an ordinary resistor at the
//! per-channel rating. A dynamic case is integrated here on the circuit's own
//! analytic node trajectory, by classic's rule at each step, never from core
//! output.

use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};
use crate::solve::static_part::NODE_RSHUNT_G as GS;
use crate::solve::PartitionSolver;

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob }
}

fn source(ci: usize, pos: usize, v: f64) -> Device {
    dev(kinds::VOLTAGE_SOURCE, ci, &[pos, 0], vec![v], vec![])
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    dev(kinds::RESISTOR, ci, &[a, b], vec![r], vec![])
}

fn capacitor(ci: usize, a: usize, b: usize, c: f64) -> Device {
    dev(kinds::CAPACITOR, ci, &[a, b], vec![c, 0.0, 0.0], vec![])
}

fn pot(ci: usize, cw: usize, wiper: usize, ccw: usize, r_cw: f64, r_ccw: f64, p_max: f64) -> Device {
    dev(kinds::POTENTIOMETER, ci, &[cw, wiper, ccw], vec![r_cw, r_ccw, p_max], vec![])
}

fn array(ci: usize, pins: [usize; 8], r: f64, p_max: f64) -> Device {
    dev(kinds::RESISTOR_ARRAY, ci, &pins, vec![r, p_max], vec![0.0])
}

fn program(node_count: usize, devices: Vec<Device>, channels: usize, options: Options) -> Program {
    Program { node_count, options, digital_slot_count: 0, channel_slot_count: channels, devices, carry: Vec::new() }
}

/// Classic's pot rule at one set of segment voltages: `(rate, power,
/// limit)` of the larger ratio, tie to cw.
fn pot_rule(d_cw: f64, d_ccw: f64, r_cw: f64, r_ccw: f64, p_max: f64) -> (f64, f64, f64) {
    let (p_cw, p_ccw) = (d_cw * d_cw / r_cw, d_ccw * d_ccw / r_ccw);
    let track = r_cw + r_ccw;
    let (l_cw, l_ccw) = (p_max * (r_cw / track).max(0.25), p_max * (r_ccw / track).max(0.25));
    let (cw, ccw) = (p_cw / l_cw, p_ccw / l_ccw);
    if cw >= ccw {
        ((cw - 1.0).max(0.0), p_cw, l_cw)
    } else {
        ((ccw - 1.0).max(0.0), p_ccw, l_ccw)
    }
}

/// Records of a channel as `[device, code, pin, since, value, limit, aux]`.
fn records(e: &mut Engine, channel: u32) -> Vec<[f64; 7]> {
    e.records(channel).unwrap().chunks_exact(7).map(|c| c.try_into().unwrap()).collect()
}

#[track_caller]
fn close(what: &str, got: f64, want: f64, rel: f64) {
    assert!((got - want).abs() <= rel * want.abs(), "{what}: got {got:e}, want {want:e} (rel {rel:e})");
}

// ---- the potentiometer ---------------------------------------------------

#[test]
fn a_pot_on_fixed_nodes_latches_by_its_larger_ratio_and_then_carries_nothing() {
    // cw on 5 V, ccw on 0 V, the wiper on a fixed `w` V: every pin fixed,
    // so the drive is constant and the latch is at 1 / rate.
    for (label, w, r_cw, r_ccw) in [
        ("cw over its share", 2.0, 30.0, 70.0),
        ("ccw over its share", 4.5, 30.0, 70.0),
        ("cw at the 0.25 floor", 4.0, 10.0, 90.0),
        ("a tie goes to cw", 2.5, 50.0, 50.0),
    ] {
        let p = program(3, vec![source(0, 1, 5.0), source(1, 2, w), pot(2, 1, 2, 0, r_cw, r_ccw, 0.2)], 0, Options::default());
        let (rate, value, limit) = pot_rule(5.0 - w, w, r_cw, r_ccw, 0.2);
        assert!(rate > 0.0, "{label}: setup");
        let mut e = Engine::load(&p).unwrap();
        close(&format!("{label}: elementI is cw to wiper"), e.currents()[2], (5.0 - w) / r_cw, 1e-15);
        let t_x = 1.0 / rate;
        e.advance(0.999 * t_x).unwrap();
        assert!(records(&mut e, 0).is_empty(), "{label}: nothing yet");
        e.advance(1.5 * t_x).unwrap();
        let r = records(&mut e, 0);
        assert_eq!(r.len(), 1, "{label}: {r:?}");
        assert_eq!((r[0][0], r[0][1], r[0][2], r[0][6]), (2.0, 1.0, -1.0, 0.0), "{label}: resistor_overload, no pin");
        close(&format!("{label}: since"), r[0][3], t_x, 1e-12);
        close(&format!("{label}: value"), r[0][4], value, 1e-15);
        close(&format!("{label}: limit"), r[0][5], limit, 1e-15);
        assert_eq!(e.currents()[2], 0.0, "{label}: the open pot carries nothing");
        assert!(e.currents()[1].abs() < 1e-9, "{label}: the wiper source supplies only its node's shunt: {}", e.currents()[1]);
    }
}

#[test]
fn a_pot_in_a_static_partition_latches_at_the_hand_solved_rate() {
    // Wiper free, loaded by 20 ohm: v = (5/30) / (1/30 + 1/70 + 1/20 + GS).
    let p = program(3, vec![source(0, 1, 5.0), pot(1, 1, 2, 0, 30.0, 70.0, 0.2), resistor(2, 2, 0, 20.0)], 0, Options::default());
    let v = (5.0 / 30.0) / (1.0 / 30.0 + 1.0 / 70.0 + 1.0 / 20.0 + GS);
    let (rate, value, limit) = pot_rule(5.0 - v, v, 30.0, 70.0, 0.2);
    let mut e = Engine::load(&p).unwrap();
    close("wiper", e.voltages()[2], v, 1e-12);
    e.advance(2.0 / rate).unwrap();
    let r = records(&mut e, 0);
    assert_eq!(r.len(), 1, "{r:?}");
    close("since", r[0][3], 1.0 / rate, 1e-12);
    close("value", r[0][4], value, 1e-12);
    close("limit", r[0][5], limit, 1e-15);
    // Open: the load alone holds the wiper, at 0 (its shunt only).
    assert!(e.voltages()[2].abs() < 1e-9, "the wiper floats to 0: {}", e.voltages()[2]);
    assert_eq!(e.currents()[1], 0.0);
}

/// The closed-form RC case: cw on 5 V, ccw on 0 V, the wiper into 1 mF.
/// `v(t) = V_th (1 - e^(-t/tau))`, `G = 1/10 + 1/90 + GS`, `V_th = 0.5/G`,
/// `tau = C/G`. cw starts at 50 times its floored share (10 ohm, 0.05 W)
/// and falls as the wiper charges; ccw climbs to 1.25 times its share. The
/// larger ratio changes hands once, and the latch falls seconds later, on
/// ccw: classic's rule integrated by midpoint steps of 1 us.
fn rc_pot(options: Options) -> (Program, f64) {
    let c = 1e-3;
    let p = program(3, vec![source(0, 1, 5.0), pot(1, 1, 2, 0, 10.0, 90.0, 0.2), capacitor(2, 2, 0, c)], 0, options);
    let g = 0.1 + 1.0 / 90.0 + GS;
    let (v_th, tau) = (0.5 / g, c / g);
    let h = 1e-6;
    let mut s = 0.0;
    let mut t = 0.0;
    let mut handed = false;
    loop {
        let m = t + 0.5 * h;
        let v = v_th * -(-m / tau).exp_m1();
        let (rate, _, limit) = pot_rule(5.0 - v, v, 10.0, 90.0, 0.2);
        handed |= limit != 0.05;
        let next = if rate > 0.0 { s + rate * h } else { (s - h).max(0.0) };
        if next >= 1.0 {
            assert!(handed, "setup: the latch falls after the hand-over");
            return (p, t + h * (1.0 - s) / (next - s));
        }
        s = next;
        t += h;
        assert!(t < 10.0, "setup: it latches");
    }
}

#[test]
fn a_pot_in_a_closed_form_partition_latches_at_the_integral_across_the_hand_over() {
    let (p, want) = rc_pot(Options::default());
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.99 * want).unwrap();
    assert!(records(&mut e, 0).is_empty(), "nothing before {want}");
    e.advance(want + 0.5).unwrap();
    let r = records(&mut e, 0);
    assert_eq!(r.len(), 1, "{r:?}");
    close("closed form since", r[0][3], want, 1e-7);
    close("ccw's share", r[0][5], 0.2 * 0.9, 1e-15);
}

/// The stepped solver integrates by classic's backward rule (the step's end
/// rate held over the step, README W1g point 2), so its latch is classic's
/// step quantisation, not the closed form's. What it must equal is the same
/// rule on the same steps: 20 V on cw (10 ohm), ccw 90 ohm, the wiper into
/// 10 mF, 5 W: cw's share 1.25 W (floored) starts at ratio 32 and latches in
/// the decay, ccw's 4.5 W share is never passed (3.6 W at most), so the pot's
/// rate is cw's alone, exactly a 10 ohm resistor's at 1.25 W. Its twin is that
/// resistor and a 90 ohm one rated 4.5 W, the same matrix: the two latch at
/// the same instant, to rounding.
fn stepped_twins(options: Options, supply: Device, extra: Vec<Device>) -> (f64, f64) {
    let rating = |d: usize, p_max: f64, r: f64| dev(kinds::RATING, d, &[], vec![d as f64, p_max, r], vec![]);
    let mut pot_p = program(3, vec![supply.clone(), pot(1, 1, 2, 0, 10.0, 90.0, 5.0), capacitor(2, 2, 0, 1e-2)], 0, options);
    let mut twin = program(3, vec![supply, resistor(1, 1, 2, 10.0), capacitor(2, 2, 0, 1e-2), resistor(3, 2, 0, 90.0), rating(1, 1.25, 10.0), rating(3, 4.5, 90.0)], 0, options);
    pot_p.devices.extend(extra.iter().cloned());
    twin.devices.extend(extra);
    let latch = |p: &Program| {
        let mut e = Engine::load(p).unwrap();
        e.advance(1.0).unwrap();
        let r = records(&mut e, 0);
        assert_eq!(r.len(), 1, "{r:?}");
        assert_eq!(r[0][0], 1.0, "{r:?}");
        r[0][3]
    };
    (latch(&pot_p), latch(&twin))
}

#[test]
fn a_pot_in_every_partition_kind_latches_as_its_resistor_twin() {
    let stepped = Options { force_stepped: true, ..Options::default() };
    let exact = Options { no_rail_shortcut: true, ..Options::default() };
    let psu = |i_limit: f64| dev(kinds::BENCH_PSU, 0, &[1, 0], vec![20.0, i_limit], vec![]);
    for (label, options) in [("closed form", Options::default()), ("stepped", stepped), ("exact", exact), ("stepped, trapezoid", Options { integration: 1, ..stepped })] {
        let (pot, twin) = stepped_twins(options, source(0, 1, 20.0), vec![]);
        close(&format!("{label}: the pot latches with its twin"), pot, twin, 1e-12);
        // A bench PSU in CV: the RC is a hub partition.
        let (pot, twin) = stepped_twins(options, psu(10.0), vec![]);
        close(&format!("{label}, PSU in CV: the pot latches with its twin"), pot, twin, 1e-12);
    }
    // In CC on a rail capacitor the rail drives the RC (R21, R29), and the
    // re-anchors are each device's own, a tenth of W0 apart at most: the twin
    // within W0.
    for (label, options) in [("closed form", Options::default()), ("stepped", stepped), ("exact", exact)] {
        let (pot, twin) = stepped_twins(options, psu(1.0), vec![capacitor(9, 1, 0, 1e-4)]);
        close(&format!("{label}, PSU in CC: the pot latches with its twin"), pot, twin, 1e-3);
    }
}

#[test]
fn an_array_channel_in_a_stepped_partition_latches_as_its_resistor_twin() {
    // As the closed-form channel case below, against a plain rated 10 ohm.
    for (label, options) in [("stepped", Options { force_stepped: true, ..Options::default() }), ("exact", Options { no_rail_shortcut: true, ..Options::default() })] {
        let arr = program(3, vec![source(0, 1, 5.0), array(1, [1, 2, 0, 0, 0, 0, 0, 0], 10.0, 0.125), capacitor(2, 2, 0, 0.05)], 4, options);
        let twin = program(3, vec![source(0, 1, 5.0), resistor(1, 1, 2, 10.0), capacitor(2, 2, 0, 0.05), dev(kinds::RATING, 1, &[], vec![1.0, 0.125, 10.0], vec![])], 0, options);
        let latch = |p: &Program| {
            let mut e = Engine::load(p).unwrap();
            e.advance(1.0).unwrap();
            let r = records(&mut e, 0);
            assert_eq!(r.len(), 1, "{label}: {r:?}");
            r[0][3]
        };
        close(&format!("{label}: channel 1 latches with its twin"), latch(&arr), latch(&twin), 1e-12);
    }
}

#[test]
fn a_knob_move_re_drives_a_pot_on_fixed_nodes() {
    // As the first case's cw (rate 4) until 0.1 s, S = 0.4; then 50/50:
    // ratios 9/50/0.1 = 1.8 and 4/50/0.1 = 0.8, rate 0.8, so 0.6 more takes
    // 0.75 s: the latch at 0.85 s, with the new segment's power.
    let p = program(3, vec![source(0, 1, 5.0), source(1, 2, 2.0), pot(2, 1, 2, 0, 30.0, 70.0, 0.2)], 0, Options::default());
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.1).unwrap();
    e.set_params(&[(2, 0, 50.0), (2, 1, 50.0)]).unwrap();
    close("elementI after the move", e.currents()[2], 3.0 / 50.0, 1e-15);
    e.advance(1.0).unwrap();
    let r = records(&mut e, 0);
    assert_eq!(r.len(), 1, "{r:?}");
    close("since", r[0][3], 0.85, 1e-12);
    close("value", r[0][4], 0.18, 1e-15);
    close("limit", r[0][5], 0.1, 1e-15);
    // A move on an open pot keeps it open.
    e.set_params(&[(2, 0, 30.0), (2, 1, 70.0)]).unwrap();
    assert_eq!(e.currents()[2], 0.0);
    e.reset_failures().unwrap();
    close("a reset restamps it at the new knob", e.currents()[2], 3.0 / 30.0, 1e-15);
}

// ---- the resistor array --------------------------------------------------

/// 100 ohm, 0.125 W per channel: ch1 5 V across (0.25 W, rate 1, 1 s), ch2
/// 6 V (0.36 W, rate 1.88), ch3 3 V and ch4 1 V under the rating.
fn four_channels() -> Program {
    let nodes = [1, 0, 1, 4, 2, 0, 3, 0];
    program(5, vec![source(0, 1, 5.0), source(1, 2, 3.0), source(2, 3, 1.0), source(3, 4, -1.0), array(4, nodes, 100.0, 0.125)], 4, Options::default())
}

#[test]
fn an_array_latches_channel_by_channel_and_a_latched_channel_reads_zero() {
    let mut e = Engine::load(&four_channels()).unwrap();
    let (i1, i2, i3, i4) = (0.05, 0.06, 0.03, 0.01);
    for (c, want) in [i1, i2, i3, i4].into_iter().enumerate() {
        close(&format!("channel {c}"), e.channel_slots()[c], want, 1e-15);
    }
    e.advance(2.0).unwrap();
    let r = records(&mut e, 0);
    assert_eq!(r.len(), 2, "{r:?}");
    // Latch order: ch2 first. `pin` names the channel, 0-based (`ch<pin + 1>`).
    assert_eq!((r[0][0], r[0][1], r[0][2]), (4.0, 1.0, 1.0), "ch2: {:?}", r[0]);
    close("ch2 since", r[0][3], 1.0 / 1.88, 1e-12);
    close("ch2 power", r[0][4], 0.36, 1e-14);
    assert_eq!(r[0][5], 0.125);
    assert_eq!((r[1][0], r[1][1], r[1][2]), (4.0, 1.0, 0.0), "ch1: {:?}", r[1]);
    close("ch1 since", r[1][3], 1.0, 1e-12);
    close("ch1 power", r[1][4], 0.25, 1e-14);
    assert_eq!(&e.channel_slots()[..2], &[0.0, 0.0], "latched channels read 0");
    close("ch3 still conducts", e.channel_slots()[2], i3, 1e-15);
    close("elementI sums the live channels", e.currents()[4], i3 + i4, 1e-15);
    // The supply of node 4 carries only its shunt once ch2 is open.
    assert!(e.currents()[3].abs() < 1e-9, "{}", e.currents()[3]);
    // A reset closes them again, and they latch again as from 0.
    e.reset_failures().unwrap();
    assert!(records(&mut e, 0).is_empty());
    close("ch1 back", e.channel_slots()[0], i1, 1e-15);
    let t = e.time();
    e.advance(t + 0.9).unwrap();
    let r = records(&mut e, 0);
    assert_eq!(r.len(), 1, "{r:?}");
    close("ch2 again", r[0][3], t + 1.0 / 1.88, 1e-12);
}

#[test]
fn an_array_channel_in_a_closed_form_partition_latches_at_the_closed_form_time() {
    // ch1 from 5 V into 50 mF through 10 ohm (tau 0.5 s): P = 2.5 e^(-2t/tau)
    // W at 0.125 W, so S = 20 (tau/2) (1 - e^(-2t/tau)) - t; the other
    // channels are open-ended (both pins on node 0).
    let tau = 10.0 * 0.05;
    let nodes = [1, 2, 0, 0, 0, 0, 0, 0];
    let p = program(3, vec![source(0, 1, 5.0), array(1, nodes, 10.0, 0.125), capacitor(2, 2, 0, 0.05)], 4, Options::default());
    let s = |t: f64| 20.0 * tau / 2.0 * -(-2.0 * t / tau).exp_m1() - t;
    let (mut lo, mut hi) = (0.0, tau / 2.0 * 20.0f64.ln());
    assert!(s(hi) > 1.0, "setup");
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if s(mid) < 1.0 { lo = mid } else { hi = mid }
    }
    let want = 0.5 * (lo + hi);
    let mut e = Engine::load(&p).unwrap();
    e.advance(1.0).unwrap();
    let r = records(&mut e, 0);
    assert_eq!(r.len(), 1, "{r:?}");
    assert_eq!(r[0][2], 0.0, "ch1");
    // The node shunt moves tau by 1e-12 relative.
    close("since", r[0][3], want, 1e-9);
    assert_eq!(e.channel_slots()[0], 0.0);
}


#[test]
fn two_arrays_and_a_pot_keep_their_slow_keys_apart() {
    // Array A (device 1) idle on every channel (5 V across 1 kohm, 25 mW of
    // 125), a rated pot (device 2) under its shares, and array B (device 3)
    // with ch3 alone over its rating: 100 ohm from 5 V into node 2, 10 ohm to
    // ground, so v2 = 5/11 V, ch3 carries (50/11)^2/100 = 0.2066 W, rate
    // 0.2066/0.125 - 1, and once open node 2 falls to 0. Each array's four
    // channels have keys of their own (`array_channel_key`): were B's A's,
    // B's ch3 would share A's ch3 integrator and its latch would open A's
    // ch3 stamp instead of its own, leaving node 2 where it was.
    let a = array(1, [1, 0, 1, 0, 1, 0, 1, 0], 1000.0, 0.125);
    let b = dev(kinds::RESISTOR_ARRAY, 3, &[0, 0, 0, 0, 1, 2, 0, 0], vec![100.0, 0.125], vec![4.0]);
    let p = program(3, vec![source(0, 1, 5.0), a, pot(2, 1, 1, 0, 100.0, 100.0, 5.0), b, resistor(4, 2, 0, 10.0)], 8, Options::default());
    let v2 = (5.0 / 100.0) / (1.0 / 100.0 + 1.0 / 10.0 + GS);
    let power = (5.0 - v2) * (5.0 - v2) / 100.0;
    let t_x = 1.0 / (power / 0.125 - 1.0);
    let mut e = Engine::load(&p).unwrap();
    close("node 2 before", e.voltages()[2], v2, 1e-12);
    e.advance(0.999 * t_x).unwrap();
    assert!(records(&mut e, 0).is_empty(), "nothing before {t_x}");
    e.advance(t_x + 1.0).unwrap();
    let r = records(&mut e, 0);
    assert_eq!(r.len(), 1, "only B's ch3: {r:?}");
    assert_eq!((r[0][0], r[0][1], r[0][2]), (3.0, 1.0, 2.0), "{:?}", r[0]);
    close("B's ch3 since", r[0][3], t_x, 1e-12);
    close("B's ch3 power", r[0][4], power, 1e-12);
    assert!(e.voltages()[2].abs() < 1e-9, "B's own ch3 opened: node 2 at {}", e.voltages()[2]);
    let slots = e.channel_slots().to_vec();
    for (c, &i) in slots[..4].iter().enumerate() {
        close(&format!("A's ch{} still conducts", c + 1), i, 0.005, 1e-15);
    }
    assert_eq!(&slots[4..], &[0.0, 0.0, 0.0, 0.0], "B: ch3 open, the rest idle");
    close("A's elementI", e.currents()[1], 0.02, 1e-15);
    assert_eq!(e.currents()[3], 0.0, "B carries nothing");
}

// ---- P0-SG: the overloads under the 1.0.0 line's carry and snapshot -------

/// `p` reloaded at `t` with `e`'s carry: program v4's block, at `t`.
fn reload(e: &mut Engine, p: &Program) -> Engine {
    let mut q = p.clone();
    q.carry = e.carry_take().unwrap().to_vec();
    q.options.start_time = e.time();
    Engine::load(&q).unwrap()
}

/// W2c: an array's record is its four channels' slow layouts. Carried at 0.6
/// s (ch2 latched at 1/1.88 s, ch1 at S = 0.6 of 1), the reload keeps ch2
/// open and latches ch1 at 1 s as the run that never reloaded does.
#[test]
fn a_carried_reload_keeps_an_arrays_channel_latches_and_stress() {
    let p = four_channels();
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.6).unwrap();
    let buf = e.carry_take().unwrap().to_vec();
    let slow: Vec<(usize, usize)> = crate::carry::records(&buf).unwrap().iter().filter(|r| r.family == crate::carry::FAMILY_SLOW).map(|r| (r.device, r.values.len())).collect();
    assert_eq!(slow, vec![(4, 4 * crate::carry::slow::LEN)], "one record for the array, its four channels");
    let mut r = reload(&mut e, &p);
    assert_eq!(&r.channel_slots()[..2], &[0.05, 0.0], "ch1 conducting, ch2 carried open");
    r.advance(2.0).unwrap();
    e.advance(2.0).unwrap();
    let (got, want) = (records(&mut r, 0), records(&mut e, 0));
    assert_eq!(got.len(), 2, "{got:?}");
    assert_eq!(&got[1][..3], &[4.0, 1.0, 0.0], "ch1 latches on the reload");
    close("ch1 since, carried", got[1][3], want[1][3], 1e-12);
    assert_eq!(r.channel_slots(), e.channel_slots());
}

/// W2c: a pot's stress (one slow device) and its latch carry by its own
/// record; the reload latches it when the run that never reloaded does.
#[test]
fn a_carried_reload_keeps_a_pots_stress_and_its_latch() {
    let p = program(3, vec![source(0, 1, 5.0), source(1, 2, 2.0), pot(2, 1, 2, 0, 30.0, 70.0, 0.2)], 0, Options::default());
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.1).unwrap();
    let mut r = reload(&mut e, &p);
    r.advance(1.0).unwrap();
    e.advance(1.0).unwrap();
    let (got, want) = (records(&mut r, 0), records(&mut e, 0));
    assert_eq!(got.len(), 1, "{got:?}");
    close("since, carried", got[0][3], want[0][3], 1e-12);
    // And once open, a reload carries it open.
    let mut again = reload(&mut r, &p);
    assert_eq!(again.currents()[2], 0.0, "carried open");
    again.advance(1.5).unwrap();
    assert_eq!(again.currents()[2], 0.0);
}

/// W2a: a snapshot holds the channel latches, the open mask the readout
/// reads and the stress; restored, the engine latches as before.
#[test]
fn a_snapshot_restores_an_arrays_latches_and_its_open_mask() {
    let mut e = Engine::load(&four_channels()).unwrap();
    e.advance(0.6).unwrap();
    let snap = e.snapshot();
    e.advance(2.0).unwrap();
    let after = (records(&mut e, 0), e.channel_slots().to_vec(), e.currents().to_vec());
    e.restore(&snap);
    assert_eq!(&e.channel_slots()[..2], &[0.05, 0.0], "back to ch1 conducting, ch2 open");
    e.advance(2.0).unwrap();
    assert_eq!((records(&mut e, 0), e.channel_slots().to_vec(), e.currents().to_vec()), after);
}

// ---- P0-SG2: a failed advance puts the latch's stamp back ------------------

/// A doomed instant that latches device `dev` at `t_x` (the slow event the kernel would run, then the settle's resolve) and
/// rolls back, against a twin that never had it. The doomed run must have really latched (the device reads nothing inside
/// the window) and the rollback must bring the slow state back; what this proves is the third piece, the STAMP: a live
/// edit after the rollback re-solves the partition from the stamp the rollback left, and it must be the conducting one.
#[track_caller]
fn latch_rolled_back_then_a_live_edit_matches_the_twin(p: &Program, dev: usize, key: usize, after: f64, t_x: f64, edit: &[(u32, u32, f64)]) {
    let (mut doomed, mut twin) = (Engine::load(p).unwrap(), Engine::load(p).unwrap());
    for e in [&mut doomed, &mut twin] {
        e.advance(t_x * 0.5).unwrap();
    }
    let before = doomed.currents().to_vec();
    let now = t_x + 1e-9;
    {
        let s = doomed.debug_solver_concrete_mut();
        let cp = s.begin_instant();
        s.handle_slow_event(crate::events::DeviceId(key as u32), now).unwrap();
        s.slow_settle(now, &[]).unwrap();
        s.resolve_dirty(&[], now, &[]).unwrap();
        s.prepare_readout();
        assert!((s.currents()[dev] - after).abs() < 1e-12, "the doomed instant must have latched the device, or the test proves nothing about its stamp");
        s.rollback_instant(cp);
    }
    assert_eq!(doomed.currents(), before.as_slice(), "the readout goes back");
    for e in [&mut doomed, &mut twin] {
        e.set_params(edit).unwrap();
    }
    assert_ne!(twin.currents()[dev], after, "setup: the edited device still conducts in the twin");
    let (got, want): (Vec<u64>, Vec<u64>) = (doomed.currents().iter().map(|x| x.to_bits()).collect(), twin.currents().iter().map(|x| x.to_bits()).collect());
    assert_eq!(got, want, "after the rollback a live edit solves with the stamp the failed instant left");
    // And it latches at the twin's time.
    for e in [&mut doomed, &mut twin] {
        e.advance(t_x * 4.0).unwrap();
    }
    assert_eq!(records(&mut doomed, 0), records(&mut twin, 0));
    assert_eq!(doomed.currents(), twin.currents());
}

#[test]
fn a_rolled_back_pot_or_trimmer_latch_puts_its_stamp_back() {
    for kind in [kinds::POTENTIOMETER, kinds::TRIMMER] {
        let mut d = pot(2, 1, 2, 0, 30.0, 70.0, 0.2);
        d.kind = kind;
        let p = program(3, vec![source(0, 1, 5.0), source(1, 2, 2.0), d], 0, Options::default());
        let (rate, _, _) = pot_rule(3.0, 2.0, 30.0, 70.0, 0.2);
        // The wiper source moves 2 V to 2.5 V: the segment currents change, and still a pot at this rating is not yet open.
        latch_rolled_back_then_a_live_edit_matches_the_twin(&p, 2, 2, 0.0, 1.0 / rate, &[(1, 0, 2.5)]);
    }
}

#[test]
fn a_rolled_back_array_channel_latch_puts_its_stamp_back() {
    // ch2 (6 V over 100 ohm, 0.36 W against 0.125 W) latches at 1 / 1.88 s; ch1 is still fine.
    let p = four_channels();
    latch_rolled_back_then_a_live_edit_matches_the_twin(&p, 4, super::array_channel_key(&p, 4, 1), 0.05 + 0.03 + 0.01, 1.0 / 1.88, &[(1, 0, 6.5)]);
}
