//! W2b piece 4: light means, the charge (core/README.md, "Charge"; W2 design
//! 2.4, ruling R57 D4 (a)). Each device's charge is exact in the model of
//! the solver that publishes its current: a static or fixed device its held
//! current times the time it held, a closed form's its analytic integral, a
//! stepped partition's its integrator's own quadrature (so a capacitor's is
//! `C dV` to rounding), and a source's the KCL sum of what leaves the nodes
//! it fixes.

use super::observe_tests::{astable, cc_rail, cell_rail, dev, edges, led_rc, pin, program, rc, rc_ring, stepped, wave, STEPS};
use super::static_part::NODE_RSHUNT_G as G;
use crate::devices::led::GMIN_D;
use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Program};

/// Every take's `[t0, t1]` and the per-slot sums of `p` charged on
/// `devices` through `steps` (a take after each), checking the spans join.
fn charge(p: &Program, devices: &[u32], steps: &[f64]) -> (Engine, Vec<f64>) {
    let mut e = Engine::load(p).unwrap();
    e.charge_arm(devices).unwrap();
    let mut t_prev = e.time();
    let mut q: Vec<f64> = Vec::new();
    for &s in steps {
        e.advance(s).unwrap();
        let b = e.take_charge().to_vec();
        assert_eq!((b[0], b[1]), (t_prev, s), "each take covers the span since the last");
        t_prev = s;
        if q.is_empty() {
            q = vec![0.0; b.len() - 2];
        }
        assert_eq!(b.len() - 2, q.len(), "one slot per device and channel");
        for (k, &x) in b[2..].iter().enumerate() {
            q[k] += x;
        }
    }
    (e, q)
}

fn close(name: &str, got: f64, want: f64, tol: f64) {
    assert!((got - want).abs() <= tol, "{name}: {got:e} against {want:e} (off {:e}, allowed {tol:e})", (got - want).abs());
}

/// The RC's capacitor node: `v (1 - e^(-t / tau))` with the shunts.
fn rc_truth() -> (f64, f64) {
    let rg = 1000.0 * G;
    (5.0 / (1.0 + rg), 1e-3 / (1.0 + rg))
}

#[test]
fn a_closed_form_rcs_charge_is_its_analytic_integral() {
    let (e, q) = charge(&rc(), &[0, 1, 2], &STEPS);
    let t = 5e-3;
    let (vinf, tau) = rc_truth();
    let v2 = vinf * (1.0 - (-t / tau).exp());
    // The capacitor: C dV.
    close("capacitor", q[2], 1e-6 * v2, 1e-12 * 1e-6 * v2);
    // The resistor: the integral of its exponential, by hand.
    let hand = (5.0 * t - vinf * (t - tau * (1.0 - (-t / tau).exp()))) / 1000.0;
    close("resistor", q[1], hand, 1e-12 * hand);
    // The source by KCL: the resistor's charge and the shunt at its node,
    // with the sign it publishes its current by.
    let i = e.currents();
    let sign = (i[0] / -(i[1] + G * 5.0)).signum();
    close("published sign", i[0], -sign * (i[1] + G * 5.0), 1e-15);
    close("source", q[0], -sign * (q[1] + G * 5.0 * t), 1e-12 * q[1].abs());
}

#[test]
fn a_stepped_rcs_charge_is_its_integrators_quadrature() {
    let (vinf, tau) = rc_truth();
    for trap in [false, true] {
        let name = if trap { "trapezoidal" } else { "default" };
        let (e, q) = charge(&stepped(rc(), trap), &[0, 1, 2], &STEPS);
        let t = 5e-3;
        let v2 = e.voltages()[2];
        // The integrator moves the capacitor's charge exactly C dV.
        close(&format!("{name}: capacitor"), q[2], 1e-6 * v2, 1e-12 * 1e-6 * v2);
        // KCL at its node, every point of the quadrature: the resistor's is
        // the capacitor's and the shunt's (at most G 5 V t).
        close(&format!("{name}: resistor against KCL"), q[1], q[2], G * 5.0 * t + 1e-12 * q[1]);
        let i = e.currents();
        let sign = (i[0] / -(i[1] + G * 5.0)).signum();
        close(&format!("{name}: source"), q[0], -sign * (q[1] + G * 5.0 * t), 1e-12 * q[1].abs());
        // And within the steps' accepted error of the analytic integral.
        let hand = (5.0 * t - vinf * (t - tau * (1.0 - (-t / tau).exp()))) / 1000.0;
        close(&format!("{name}: resistor against the exponential"), q[1], hand, 1e-4 * hand);
    }
}

#[test]
fn a_tr_bdf2_partitions_charge_is_its_stage_quadrature() {
    // led_rc: 0 the clock (node 1), 1 1k 1-2, 2 1 uF 2-0, 3 the LED 2-3,
    // 4 100 ohm 3-0, stepped by TR-BDF2 by default (R98) and by the
    // trapezoid under the option.
    for trap in [false, true] {
        let name = if trap { "trapezoid" } else { "TR-BDF2" };
        let mut p = led_rc();
        p.options.integration = u8::from(trap);
        // Ending between edges: at an edge the re-seed publishes the node off
        // its capacitor's charge by the seed's own companion.
        let (e, q) = charge(&p, &[0, 1, 2, 3, 4], &[0.37e-3, 1e-3, 1.7e-3, 2.4e-3]);
        let t = 2.4e-3;
        let v2 = e.voltages()[2];
        close(&format!("{name}: capacitor"), q[2], 1e-6 * v2, 1e-12 * 1e-6 * v2);
        // KCL at the capacitor's node and the LED's cathode, each point of
        // the quadrature: the shunts and the junction's GMIN are the slack.
        // An iterated partition's points satisfy KCL to its Newton
        // tolerance, not to rounding (a point's residual measured up to 5e-9
        // A while the LED carries 1.7 mA; the charges 2e-7 to 5e-7 apart).
        let slack = (G + GMIN_D) * 5.0 * t;
        close(&format!("{name}: node 2"), q[1], q[2] + q[3], slack + 1e-6 * q[1]);
        close(&format!("{name}: node 3"), q[3], q[4], slack + 1e-6 * q[3]);
        assert!(q[3] > 1e-6, "{name}: the LED conducts ({:e} C)", q[3]);
        // The clock by KCL at its node: the shunt holds 5 V while it is high
        // (half of the 2.5 ms).
        let i = e.currents();
        let sign = if i[1].abs() > 0.0 { (i[0] / -(i[1] + G * e.voltages()[1])).signum() } else { -1.0 };
        close(&format!("{name}: clock"), q[0], -sign * (q[1] + G * 5.0 * 1.4e-3), 1e-12 * q[1].abs());
    }
}

/// A 1 kHz clock into an RGB LED's red anode through 330 ohm and its green
/// anode through 1k, its blue anode and common cathode at ground: a static,
/// iterated partition whose channels hold between the clock's edges.
fn rgb_clock() -> Program {
    let mut p = program(4, vec![
        wave(kinds::CLOCK, 0, 1, 0, vec![1000.0, 0.5, 5.0, 0.0, 0.0]),
        dev(kinds::RESISTOR, 1, 1, 2, vec![330.0]),
        dev(kinds::RESISTOR, 2, 1, 3, vec![1000.0]),
        Device { kind: kinds::RGB_LED, component_index: 3, pins: vec![pin(2), pin(3), pin(0), pin(0)], params: vec![1.8, 2.0, 3.0, 0.02, 0.02, 0.0], blob: vec![0.0] },
    ]);
    p.channel_slot_count = 3;
    p
}

#[test]
fn a_static_partitions_charge_and_channels_hold_between_its_instants() {
    let (_, q) = charge(&rgb_clock(), &[1, 3, 2], &STEPS);
    assert_eq!(q.len(), 1 + 4 + 1, "the resistor, the LED and its three channels, the resistor");
    // What each slot holds while the clock is high and while it is low.
    let read = |t: f64| {
        let mut e = Engine::load(&rgb_clock()).unwrap();
        e.advance(t).unwrap();
        let (i, c) = (e.currents().to_vec(), e.channel_slots().to_vec());
        [i[1], i[3], c[0], c[1], c[2], i[2]]
    };
    let (hi, lo) = (read(0.25e-3), read(0.75e-3));
    assert!(hi[2] > 1e-3 && hi[3] > 1e-4, "red and green light while high: {hi:?}");
    for k in 0..6 {
        let hand = (hi[k] + lo[k]) * 2.5e-3;
        close(&format!("slot {k}"), q[k], hand, 1e-12 * hand.abs().max(1e-30));
    }
}

#[test]
fn a_ramped_lines_static_dividers_and_closed_forms_charge_exactly_and_their_sources_by_kcl() {
    // edges: 0 the clock (node 1), 1 1k 1-2, 2 1 uF 2-0 (a closed form
    // re-aimed at each edge), 3 the pulse ramp (node 3), 4 2k 3-0 (across
    // fixed nodes, the ramp moving one), 5 2k 1-4 and 6 10k 4-0 (a static
    // divider stepping with the clock).
    let t = 5e-3;
    let (e, q) = charge(&edges(), &[0, 1, 2, 3, 4, 5, 6], &STEPS);
    // The ramp's integral from its own samples every 1 us (its corners fall
    // on that grid, so the trapezoid is exact).
    let mut probe = Engine::load(&edges()).unwrap();
    probe.acquire(&[3], 0.0, 1e-6, 0).unwrap();
    probe.advance(t).unwrap();
    let b = probe.acquire_take().to_vec();
    let n = b[0] as usize;
    let vals = &b[crate::observe::TAKE_HEADER + 1 + n..];
    let ramp: f64 = (1..n).map(|k| 0.5 * (vals[k - 1] + vals[k]) * 1e-6).sum();
    assert!(ramp > 0.01, "the ramp ran ({ramp})");
    close("ramped line", q[4], ramp / 2000.0, 1e-12 * ramp / 2000.0);
    // The static divider: its high and low currents times their times.
    let read = |at: f64| {
        let mut e = Engine::load(&edges()).unwrap();
        e.advance(at).unwrap();
        e.currents().to_vec()
    };
    let (hi, lo) = (read(0.25e-3), read(0.75e-3));
    for k in [5, 6] {
        let hand = (hi[k] + lo[k]) * 2.5e-3;
        close(&format!("divider {k}"), q[k], hand, 1e-12 * hand);
    }
    // The closed form: C dV, and its resistor by KCL at its node.
    let v2 = e.voltages()[2];
    close("capacitor", q[2], 1e-6 * v2, 1e-12 * 1e-6 * v2.abs());
    close("its resistor", q[1], q[2], G * 5.0 * t + 1e-12 * q[1].abs());
    // The sources by KCL: the clock at node 1 (5 V for half the run), the
    // pulse at node 3 (the ramp's integral).
    let i = e.currents();
    let sign = |s: usize, others: f64, v: f64| (i[s] / -(others + G * v)).signum();
    assert!(i[0].abs() > 0.0, "the clock carries current at the end");
    let s0 = sign(0, i[1] + i[5], e.voltages()[1]);
    // The pulse is at 0 V and carries nothing at the end: it publishes its
    // current as the clock does (both from their node to ground).
    let s3 = s0;
    close("clock", q[0], -s0 * (q[1] + q[5] + G * 5.0 * 2.5e-3), 1e-12 * (q[1].abs() + q[5].abs()));
    close("pulse", q[3], -s3 * (q[4] + G * ramp), 1e-12 * q[4].abs());
}

#[test]
fn a_charged_run_is_the_uncharged_run_bit_for_bit() {
    for (name, p) in [("rc", rc()), ("edges", edges()), ("stepped rc", stepped(rc(), false)), ("led rc", led_rc()), ("555", astable()), ("ring", rc_ring()), ("cc rail", cc_rail()), ("cell rail", cell_rail()), ("cell rail, stepped", stepped(cell_rail(), false)), ("rgb", rgb_clock())] {
        let mut armed = Engine::load(&p).unwrap();
        let all: Vec<u32> = (0..p.devices.len() as u32).collect();
        armed.charge_arm(&all).unwrap();
        let mut plain = Engine::load(&p).unwrap();
        for &s in &STEPS {
            armed.advance(s).unwrap();
            plain.advance(s).unwrap();
            let _ = armed.take_charge();
            let bits = |e: &Engine| (e.voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), e.currents().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), e.channel_slots().iter().map(|v| v.to_bits()).collect::<Vec<_>>());
            assert_eq!(bits(&armed), bits(&plain), "{name} at {s}");
            assert_eq!(armed.stat(4).to_bits(), plain.stat(4).to_bits(), "{name}: the same steps at {s}");
        }
    }
}

#[test]
fn a_rolled_back_instant_adds_no_charge() {
    // A stepped RC whose partition is forced onto 1.25 ms inside a window
    // that rolls back: the step it committed there goes with it.
    let p = stepped(rc(), true);
    let mut e = Engine::load(&p).unwrap();
    e.charge_arm(&[1, 2]).unwrap();
    e.advance(1e-3).unwrap();
    let solver = e.debug_solver_mut();
    let token = solver.begin_instant();
    solver.sync_dynamic_to(1.25e-3).unwrap();
    solver.rollback_instant(token);
    e.advance(2e-3).unwrap();
    let with = e.take_charge().to_vec();
    let (_, without) = charge(&p, &[1, 2], &[1e-3, 2e-3]);
    let v2 = e.voltages()[2];
    close("capacitor", with[3], 1e-6 * v2, 1e-12 * 1e-6 * v2);
    close("resistor", with[2], without[0], 1e-12 * without[0]);
}

#[test]
fn arming_takes_and_disarming_follow_the_engine_time() {
    let mut e = Engine::load(&rc()).unwrap();
    assert_eq!(e.take_charge(), &[0.0, 0.0], "nothing armed: the engine's time, no slot");
    assert!(e.charge_arm(&[3]).is_err(), "device 3 does not exist");
    e.advance(1e-3).unwrap();
    e.charge_arm(&[2]).unwrap();
    e.advance(2e-3).unwrap();
    let b = e.take_charge().to_vec();
    assert_eq!((b[0], b[1], b.len()), (1e-3, 2e-3, 3), "armed at 1 ms: the span starts there");
    let (vinf, tau) = rc_truth();
    let v = |t: f64| vinf * (1.0 - (-t / tau).exp());
    close("capacitor from the arming", b[2], 1e-6 * (v(2e-3) - v(1e-3)), 1e-11 * 1e-6 * v(1e-3));
    assert_eq!(e.take_charge(), &[2e-3, 2e-3, 0.0], "taken: cleared, the next span from 2 ms");
    e.charge_arm(&[]).unwrap();
    assert_eq!(e.take_charge(), &[2e-3, 2e-3], "disarmed");
}

#[test]
fn a_555_and_a_cc_rail_charge_by_kcl() {
    // The 555's OUT current into its 1k load (node 5).
    let (e, q) = charge(&astable(), &[4, 5, 0], &[0.37e-3, 1e-3]);
    assert!(q[1].abs() > 1e-7, "the load conducts ({:e})", q[1]);
    let i = e.currents();
    let sign = (i[4] / (i[5] + G * e.voltages()[5])).signum();
    close("555 OUT", sign * q[0], q[1], G * 5.0 * 1e-3 + 1e-9 * q[1].abs());
    // The PSU through CC into CV: its charge against what leaves its rail.
    let t = [2e-3, 10e-3, 30e-3, 60e-3];
    let (e, q) = charge(&cc_rail(), &[0, 1, 2, 4], &t);
    let i = e.currents();
    let sign = (i[0] / -(i[1] + i[2] + i[4] + G * e.voltages()[1])).signum();
    close("PSU", q[0], -sign * (q[1] + q[2] + q[3]), G * 5.0 * 60e-3 + 1e-9 * q[1].abs());
}
