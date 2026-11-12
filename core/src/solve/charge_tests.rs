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
        let slots = b.len() - 2 - 2 * devices.len();
        if q.is_empty() {
            q = vec![0.0; slots];
        }
        assert_eq!(slots, q.len(), "one slot per device and channel");
        assert!(b[2 + slots..2 + slots + devices.len()].iter().all(|&f| f == s), "after advance every charge is final to t1: {:?}", &b[2 + slots..]);
        for (k, &x) in b[2..2 + slots].iter().enumerate() {
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
    assert_eq!((b[0], b[1], b.len(), b[3], b[4]), (1e-3, 2e-3, 5, 2e-3, 0.0), "armed at 1 ms: the span starts there; final to t1; KCL not approximate");
    let (vinf, tau) = rc_truth();
    let v = |t: f64| vinf * (1.0 - (-t / tau).exp());
    close("capacitor from the arming", b[2], 1e-6 * (v(2e-3) - v(1e-3)), 1e-11 * 1e-6 * v(1e-3));
    assert_eq!(e.take_charge(), &[2e-3, 2e-3, 0.0, 2e-3, 0.0], "taken: cleared, the next span from 2 ms");
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

// ---- piece 5: yields, snapshots, failed advances, live edits ----------------

use crate::kernel::Budget;

/// One take of both: `(samples, charge)`, each the take's raw buffer.
fn take_both(e: &mut Engine) -> (Vec<f64>, Vec<f64>) {
    (e.acquire_take().to_vec(), e.take_charge().to_vec())
}

/// `p` with the observer on `nodes` at 100 kHz and the charge on every
/// device, advanced through `steps`: by `advance`, or by `advance_for` on a
/// clock that moves 1 ms a read against a 2 ms budget (a yield every few
/// instants), taking at each host step's end only, or at each yield too.
fn yielding(p: &Program, nodes: &[u32], steps: &[f64], how: &str) -> (Vec<(Vec<f64>, Vec<f64>)>, u32) {
    let mut e = Engine::load(p).unwrap();
    e.acquire(nodes, 0.0, 1e-5, 0).unwrap();
    let all: Vec<u32> = (0..p.devices.len() as u32).collect();
    e.charge_arm(&all).unwrap();
    let (mut takes, mut yields) = (Vec::new(), 0);
    for &s in steps {
        if how == "advance" {
            e.advance(s).unwrap();
        } else {
            let mut now = 0.0;
            let mut clock = || {
                now += 1.0;
                now
            };
            while !e.advance_for(s, Some(Budget { clock: &mut clock, budget_ms: 2.0, check_every: 1 })).unwrap() {
                yields += 1;
                if how == "take at yields" {
                    takes.push(take_both(&mut e));
                }
            }
        }
        takes.push(take_both(&mut e));
    }
    (takes, yields)
}

#[test]
fn a_yielding_run_samples_and_charges_as_one_advance() {
    let steps = [0.37e-3, 1e-3, 1.7e-3];
    let mut partials = 0;
    for (name, p, nodes) in [("led rc", led_rc(), [2u32, 3]), ("555", astable(), [3, 5]), ("edges", edges(), [2, 4]), ("cc rail", cc_rail(), [1, 3]), ("stepped rc", stepped(rc(), true), [1, 2])] {
        let (plain, _) = yielding(&p, &nodes, &steps, "advance");
        let (split, yields) = yielding(&p, &nodes, &steps, "take at host steps");
        assert!(yields > 3, "{name}: the clock yields ({yields})");
        // Taken at the same points, the yields leave no trace.
        let bits = |t: &[(Vec<f64>, Vec<f64>)]| t.iter().map(|(a, b)| (a.iter().map(|x| x.to_bits()).collect::<Vec<_>>(), b.iter().map(|x| x.to_bits()).collect::<Vec<_>>())).collect::<Vec<_>>();
        assert_eq!(bits(&plain), bits(&split), "{name}");
        // Taken at every yield too: the samples are the same in order, the
        // charges sum to the same, each slot's span joins, and a stepped
        // device's charge says how far it is final.
        let (each, _) = yielding(&p, &nodes, &steps, "take at yields");
        let n = nodes.len();
        let samples = |t: &[(Vec<f64>, Vec<f64>)]| {
            let mut out = vec![Vec::new(); n + 1];
            for (b, _) in t {
                let count = b[0] as usize;
                let h = crate::observe::TAKE_HEADER + n;
                out[0].extend_from_slice(&b[h..h + count]);
                for j in 0..n {
                    out[j + 1].extend_from_slice(&b[h + count * (1 + j)..h + count * (2 + j)]);
                }
            }
            out
        };
        assert_eq!(samples(&each), samples(&plain), "{name}: the samples");
        let d = p.devices.len();
        let slots = plain[0].1.len() - 2 - 2 * d;
        let total = |t: &[(Vec<f64>, Vec<f64>)]| (0..slots).map(|k| t.iter().map(|(_, c)| c[2 + k]).sum::<f64>()).collect::<Vec<f64>>();
        let (want, got) = (total(&plain), total(&each));
        let scale = want.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        for k in 0..slots {
            assert!((got[k] - want[k]).abs() <= 1e-12 * scale, "{name}: slot {k} {} against {}", got[k], want[k]);
        }
        let mut partial = 0;
        let mut t_prev = 0.0;
        for (_, c) in &each {
            assert_eq!(c[0], t_prev, "{name}: the spans join");
            t_prev = c[1];
            for &f in &c[2 + slots..2 + slots + d] {
                assert!(f <= c[1], "{name}: final {f} past t1 {}", c[1]);
                partial += usize::from(f < c[1]);
            }
        }
        partials += partial;
    }
    // Where two stepped partitions step apart (the 555's timing and CV
    // capacitors), a yield after one's commit finds the other's pending.
    assert!(partials > 0, "a take at a yield finds a step pending");
}

#[test]
fn a_restore_brings_back_the_observer_and_the_charge() {
    // Decided: the observer's queue and the charge's sums are engine state,
    // in the snapshot; a host that restores retakes from there.
    for (name, p) in [("led rc", led_rc()), ("edges", edges()), ("cc rail", cc_rail())] {
        let mut e = Engine::load(&p).unwrap();
        e.acquire(&[2], 0.0, 1e-5, 0).unwrap();
        let all: Vec<u32> = (0..p.devices.len() as u32).collect();
        e.charge_arm(&all).unwrap();
        e.advance(0.6e-3).unwrap();
        let snap = e.snapshot();
        e.advance(1.3e-3).unwrap();
        let first = take_both(&mut e);
        e.restore(&snap);
        e.advance(1.3e-3).unwrap();
        let again = take_both(&mut e);
        assert_eq!(first, again, "{name}: the same samples and charge from 0 to 1.3 ms");
    }
}

#[test]
fn a_failed_advance_keeps_what_was_final_and_adds_nothing() {
    // A stepped LED RC whose Newton is made to fail: the advance stops at its
    // last completed instant, the samples and charge already final stay, the
    // failed instant adds nothing, and the run then goes on as one that never
    // failed.
    let p = led_rc();
    let all: Vec<u32> = (0..p.devices.len() as u32).collect();
    let run = |fail: bool| {
        let mut e = Engine::load(&p).unwrap();
        e.acquire(&[2, 3], 0.0, 1e-5, 0).unwrap();
        e.charge_arm(&all).unwrap();
        e.advance(0.5e-3).unwrap();
        let mut takes = vec![take_both(&mut e)];
        if fail {
            let _guard = super::newton::hooks::force_unconverged(u32::MAX);
            assert!(e.advance(1.5e-3).is_err(), "the forced failure");
            takes.push(take_both(&mut e));
        }
        e.advance(2e-3).unwrap();
        takes.push(take_both(&mut e));
        takes
    };
    let (failed, plain) = (run(true), run(false));
    let samples = |t: &[(Vec<f64>, Vec<f64>)]| t.iter().flat_map(|(s, _)| s[crate::observe::TAKE_HEADER + 2..].to_vec()).collect::<Vec<f64>>();
    let times = |t: &[(Vec<f64>, Vec<f64>)]| t.iter().flat_map(|(s, _)| s[crate::observe::TAKE_HEADER + 2..crate::observe::TAKE_HEADER + 2 + s[0] as usize].to_vec()).collect::<Vec<f64>>();
    assert_eq!(times(&failed), times(&plain), "the same sample times");
    assert_eq!(samples(&failed), samples(&plain), "and values, bit for bit");
    let n = 5;
    for k in 0..n {
        let sum = |t: &[(Vec<f64>, Vec<f64>)]| t.iter().map(|(_, c)| c[2 + k]).sum::<f64>();
        let (a, b) = (sum(&failed), sum(&plain));
        assert!((a - b).abs() <= 1e-12 * b.abs().max(1e-12), "device {k}: {a:e} against {b:e}");
    }
    assert!(failed[1].1[1] >= failed[0].1[1] && failed[1].1[1] < 1.5e-3, "the failed take ends where the run stopped: {}", failed[1].1[1]);
}

#[test]
fn a_live_edit_refolds_the_held_currents_at_its_instant() {
    // A divider on a 10 V source set to 4 V at 1 ms: each resistor's charge
    // is its current before the set over 1 ms and after it over 1 ms, the
    // source's by KCL.
    let p = program(3, vec![dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![10.0]), dev(kinds::RESISTOR, 1, 1, 2, vec![1000.0]), dev(kinds::RESISTOR, 2, 2, 0, vec![1000.0])]);
    let mut e = Engine::load(&p).unwrap();
    e.charge_arm(&[1, 2, 0]).unwrap();
    e.advance(1e-3).unwrap();
    let before = e.currents().to_vec();
    e.set_params(&[(0, 0, 4.0)]).unwrap();
    let after = e.currents().to_vec();
    assert!((after[1] - 0.4 * before[1]).abs() < 1e-9, "the set moved the divider: {} then {}", before[1], after[1]);
    e.advance(2e-3).unwrap();
    let c = e.take_charge().to_vec();
    for (slot, d) in [(2, 1), (3, 2), (4, 0)] {
        let hand = (before[d] + after[d]) * 1e-3;
        close(&format!("device {d}"), c[slot], hand, 1e-12 * hand.abs());
    }
}

// ---- piece 6: the approximations, against fine references (W0) -------------

/// Each device's charge over `steps` and over host steps of `dt` to the
/// same end, and the worst gap as a share of W0 for a current (1 nA plus
/// 1e-3 of its mean) over the run.
fn against_fine_charge(name: &str, p: &Program, devices: &[u32], steps: &[f64], dt: f64) -> f64 {
    let (_, q) = charge(p, devices, steps);
    let t = *steps.last().unwrap();
    let n = (t / dt).round() as usize;
    let fine: Vec<f64> = (1..=n).map(|k| if k == n { t } else { k as f64 * dt }).collect();
    let (_, qf) = charge(p, devices, &fine);
    let mut worst = 0.0f64;
    for k in 0..q.len() {
        let w0 = (1e-9 + 1e-3 * (qf[k] / t).abs()) * t;
        let share = (q[k] - qf[k]).abs() / w0;
        eprintln!("{name}: slot {k}: {:e} against the fine {:e}: {share:.3e} W0", q[k], qf[k]);
        worst = worst.max(share);
    }
    worst
}

#[test]
fn an_unmodelled_moving_rail_folded_held_stays_within_w0() {
    // Ruling (a)'s rail: a cell whose loads step, moving between its solves
    // with nothing to model it; the charge holds it at its last solve.
    let p = stepped(cell_rail(), false);
    let w = against_fine_charge("cell rail, stepped", &p, &[0, 1, 2, 3, 4], &STEPS, 0.5e-6);
    assert!(w <= 1.0, "within W0: {w}");
    // The fold alone, in the same run: the static divider's lower resistor
    // (node 3 to ground, on the rail's line) against the trapezoid of its
    // node's samples every 1 us, which the observer reads linear between
    // the rail's solves.
    for t in [5e-3, 1.0 / 60.0] {
        let mut e = Engine::load(&p).unwrap();
        e.acquire(&[3], 0.0, 1e-6, 0).unwrap();
        e.charge_arm(&[4]).unwrap();
        e.advance(t).unwrap();
        let b = e.acquire_take().to_vec();
        let n = b[0] as usize;
        let v = &b[crate::observe::TAKE_HEADER + 1 + n..];
        let tl = (n - 1) as f64 * 1e-6;
        let linear: f64 = (1..n).map(|k| 0.5 * (v[k - 1] + v[k]) * 1e-6).sum::<f64>() / 1000.0 + v[n - 1] * (t - tl) / 1000.0;
        let held = e.take_charge()[2];
        let share = (held - linear).abs() / ((1e-9 + 1e-3 * (linear / t).abs()) * t);
        eprintln!("held rail against linear to {t}: {held:e} against {linear:e}: {share:.3e} W0");
        assert!(share <= 1e-6, "the fold follows the rail linear between its solves: {share} W0");
    }
}

/// `cc_rail` without its rail capacitor and limited to 5 mA, stepped: in CC
/// the rail is set by the stepped hub partition's draws alone, solved with
/// each step, until the 10 uF charges and the PSU leaves CC.
fn cc_rail_no_cap() -> Program {
    let mut p = cc_rail();
    p.devices.remove(1);
    p.devices[0].params[1] = 0.005;
    p.options.force_stepped = true;
    p
}

#[test]
fn a_stepped_hub_on_a_cc_rail_with_no_capacitor_read_at_its_step_end_rail_stays_within_w0() {
    // 0 the PSU, 1 1k 1-2, 2 1k 2-0, 3 1k 1-3, 4 10 uF 3-0.
    // The PSU, the stepped resistor and the capacitor within W0 of the fine
    // reference (each read at the rail its step solved with). The divider's
    // charge is the KCL rest of the PSU's, both runs' (its line folds at the
    // stepped partition's points so that KCL holds): it is off the fine run
    // by what the PSU and the stepped resistor are (the coarse run leaves CC
    // at its own instant), within W0 of that.
    let p = cc_rail_no_cap();
    let steps = [0.37e-3, 1e-3, 2e-3, 5e-3, 20e-3];
    let t = 20e-3;
    let (_, q) = charge(&p, &[0, 1, 2, 3, 4], &steps);
    let fine: Vec<f64> = (1..=40000).map(|k| if k == 40000 { t } else { k as f64 * 0.5e-6 }).collect();
    let (_, qf) = charge(&p, &[0, 1, 2, 3, 4], &fine);
    let w0 = |x: f64| (1e-9 + 1e-3 * (x / t).abs()) * t;
    for k in [0, 3, 4] {
        assert!((q[k] - qf[k]).abs() <= w0(qf[k]), "slot {k}: {:e} against the fine {:e}", q[k], qf[k]);
    }
    let rest = (q[0] - qf[0]).abs() + (q[3] - qf[3]).abs();
    for k in [1, 2] {
        assert!((q[k] - qf[k]).abs() <= rest + w0(qf[k]), "divider slot {k}: {:e} against the fine {:e}, the PSU's and the stepped resistor's gaps {rest:e}", q[k], qf[k]);
    }
}

#[test]
fn a_psus_cc_charge_matches_the_kcl_of_its_rail_with_no_capacitor() {
    // The PSU's charge (its own -dir iLimit while in CC) against the KCL of
    // what leaves its rail: the rail's static line folds at the stepped
    // partition's points with its weights, as its draws do (was 1.29 W0 with
    // the line linear between the rail's solves).
    let p = cc_rail_no_cap();
    let (_, q) = charge(&p, &[0, 1, 3], &[0.37e-3, 1e-3, 2e-3, 5e-3, 20e-3]);
    let t = 20e-3;
    let kcl = q[1] + q[2];
    let share = (q[0].abs() - kcl - G * 5.0 * t).abs() / ((1e-9 + 1e-3 * (kcl / t).abs()) * t);
    assert!(share <= 1.0, "within W0: {share}");
}

#[test]
fn a_closed_form_on_a_draining_cell_charges_within_w0() {
    // The cell's coupled path: the closed form's held and curved terms are
    // integrated by Gauss-Legendre.
    let w = against_fine_charge("cell rail", &cell_rail(), &[0, 1, 2, 3, 4], &STEPS, 0.5e-6);
    assert!(w <= 1.0, "within W0: {w}");
}

#[test]
fn two_stepped_hubs_on_a_cc_rail_with_no_capacitor_fold_its_lines_linear_between_its_solves() {
    // A second 1k into 10 uF off the rail (node 4): two stepped hub
    // partitions, each solving the rail against the other's draw held, so
    // no one rail trajectory carries the lines; they fold linear between the
    // rail's solves (`charge_rail_carrier`). Each device within W0 of the
    // fine reference, and the PSU's KCL within the measured gap.
    let mut p = cc_rail_no_cap();
    p.node_count = 5;
    p.devices.push(dev(kinds::RESISTOR, 6, 1, 4, vec![1000.0]));
    p.devices.push(dev(kinds::CAPACITOR, 7, 4, 0, vec![10e-6, 0.0, 0.0]));
    let steps = [0.37e-3, 1e-3, 2e-3, 5e-3, 20e-3];
    let t = 20e-3;
    let all = [0, 1, 2, 3, 4, 5, 6];
    let (_, q) = charge(&p, &all, &steps);
    let fine: Vec<f64> = (1..=40000).map(|k| if k == 40000 { t } else { k as f64 * 0.5e-6 }).collect();
    let (_, qf) = charge(&p, &all, &fine);
    let w0 = |x: f64| (1e-9 + 1e-3 * (x / t).abs()) * t;
    let kcl = q[1] + q[3] + q[5];
    let gap = (q[0].abs() - kcl - G * 5.0 * t).abs() / w0(kcl);
    // Measured: KCL 1.88 W0; the stepped capacitors' own charges (exact
    // `C dV` of the coarse run) are 2.1-2.3 W0 from the fine run's: the
    // steps' coupling error (each holds the other's draw), not the charge's.
    // This guards the measurement; one rail trajectory would need the steps
    // solved together.
    assert!(gap <= 2.0, "the PSU's KCL within the coupling's gap: {gap} W0");
    // Said loudly: each take's flag on the PSU's charge, and stat 28.
    let mut e = Engine::load(&p).unwrap();
    e.charge_arm(&all).unwrap();
    let mut flagged = 0;
    for &s in &steps {
        e.advance(s).unwrap();
        let b = e.take_charge().to_vec();
        let flags = &b[b.len() - all.len()..];
        assert!(flags[1..].iter().all(|&f| f == 0.0), "only a source is flagged: {flags:?}");
        flagged += usize::from(flags[0] == 1.0);
    }
    assert!(flagged > 0, "the PSU's charge says its KCL is approximate");
    assert!(e.stat(28) > 0.0, "stat 28 counts the intervals: {}", e.stat(28));
    // One stepped hub carries the rail: nothing approximate.
    let mut one = Engine::load(&cc_rail_no_cap()).unwrap();
    one.charge_arm(&[0, 1, 3]).unwrap();
    one.advance(20e-3).unwrap();
    let b = one.take_charge().to_vec();
    assert_eq!(b[b.len() - 3..], [0.0, 0.0, 0.0], "one stepped hub: exact KCL");
    assert_eq!(one.stat(28), 0.0);
    assert!((q[0] - qf[0]).abs() <= w0(qf[0]), "the PSU's CC charge");
}

// ---- W2bV fixes ---------------------------------------------------------------

/// W2bV finding 1: a CC PSU (8 V, `i_limit`) holding `c_rail` on its rail
/// (node 1), 150 ohm across it, and 100 ohm into 2.2 uF (node 2) feeding a
/// red LED and 60 ohm (node 3): one stepped hub partition stepping with the
/// rail capacitor as a companion on its rail row (ruling R68).
fn cc_rail_cap_led(c_rail: f64, i_limit: f64) -> Program {
    let mut p = program(4, vec![
        super::observe_tests::psu_dev(0, 1, 8.0, i_limit),
        dev(kinds::CAPACITOR, 1, 1, 0, vec![c_rail, 0.0, 0.0]),
        dev(kinds::RESISTOR, 2, 1, 0, vec![150.0]),
        dev(kinds::RESISTOR, 3, 1, 2, vec![100.0]),
        dev(kinds::CAPACITOR, 4, 2, 0, vec![2.2e-6, 0.0, 0.0]),
        Device { kind: kinds::LED, component_index: 5, pins: vec![pin(2), pin(3)], params: vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0], blob: vec![] },
        dev(kinds::RESISTOR, 6, 3, 0, vec![60.0]),
    ]);
    p.options.force_stepped = false;
    p
}

#[test]
fn a_cc_rail_capacitors_charge_is_c_dv_and_its_psu_the_kcl_of_its_rail_with_one_stepped_hub() {
    // The rail capacitor's charge against `C dV` of its own readout, and the
    // PSU's (its `-dir iLimit t` while in CC) against what leaves the rail.
    // Each in one advance, and the last (trapezoidal) through several takes.
    for (c_rail, i_limit, trap) in [(47e-6, 0.02, false), (10e-6, 0.05, false), (220e-6, 0.01, false), (47e-6, 0.02, true)] {
        for t in [2e-3, 20e-3] {
            let mut p = cc_rail_cap_led(c_rail, i_limit);
            p.options.integration = u8::from(trap);
            let v0 = Engine::load(&p).unwrap().voltages()[1];
            let steps: Vec<f64> = if trap { [0.37e-3, 1e-3, 1.7e-3, 5e-3, 13e-3, 20e-3].into_iter().filter(|&s| s < t).chain([t]).collect() } else { vec![t] };
            let (e, q) = charge(&p, &[0, 1, 2, 3], &steps);
            let b: Vec<f64> = std::iter::repeat_n(0.0, 2).chain(q.iter().copied()).collect();
            let q = &b[2..6];
            let v = e.voltages()[1];
            let name = format!("{c_rail:e} F, {i_limit} A, trapezoid {trap}, to {t}");
            close(&format!("{name}: rail capacitor"), q[1], c_rail * (v - v0), 1e-9 * c_rail * v);
            let kcl = q[1] + q[2] + q[3];
            let w0 = (1e-9 + 1e-3 * (kcl / t).abs()) * t;
            let share = (q[0].abs() - kcl - G * 8.0 * t).abs() / w0;
            eprintln!("{name}: KCL {share:.3e} W0");
            assert!(share <= 1e-2, "{name}: the PSU's KCL within 0.01 W0: {share}");
        }
    }
    // Nothing approximate: one stepped hub carries the rail.
    let mut e = Engine::load(&cc_rail_cap_led(47e-6, 0.02)).unwrap();
    e.charge_arm(&[0, 1, 2, 3]).unwrap();
    e.advance(20e-3).unwrap();
    let b = e.take_charge().to_vec();
    assert_eq!(b[b.len() - 4..], [0.0; 4], "no kcl_approximate flag");
    assert_eq!(e.stat(28), 0.0);
}

#[test]
fn two_stepped_hubs_on_a_cc_rail_with_capacitors_charge_them_c_dv_and_say_the_kcl_is_approximate() {
    // A second 137 ohm into 4.4 uF and a red LED off the rail: two stepped
    // hub partitions, each stepping the rail capacitor as its companion
    // against the other's draw held. Each solve after a commit re-aims the
    // CC segment from the charge the step handed it: the capacitor's charge
    // takes that jump (was 9.4e-4 of `C dV` off), so it is `C dV` of its
    // readout. No one rail trajectory closes the PSU's KCL: flagged and
    // counted, within the steps' coupling gap (measured up to 0.52 W0).
    let mut p = cc_rail_cap_led(47e-6, 0.02);
    p.node_count = 6;
    p.devices.push(dev(kinds::RESISTOR, 7, 1, 4, vec![137.0]));
    p.devices.push(dev(kinds::CAPACITOR, 8, 4, 0, vec![4.4e-6, 0.0, 0.0]));
    p.devices.push(Device { kind: kinds::LED, component_index: 9, pins: vec![pin(4), pin(5)], params: vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0], blob: vec![] });
    p.devices.push(dev(kinds::RESISTOR, 10, 5, 0, vec![60.0]));
    let v0 = Engine::load(&p).unwrap().voltages()[1];
    for steps in [vec![2e-3], vec![20e-3], vec![0.37e-3, 1e-3, 5e-3, 20e-3]] {
        let t = *steps.last().unwrap();
        let (e, q) = charge(&p, &[0, 1, 2, 3, 7], &steps);
        let v = e.voltages()[1];
        close(&format!("to {t} in {}: rail capacitor", steps.len()), q[1], 47e-6 * (v - v0), 1e-9 * 47e-6 * v);
        let kcl = q[1] + q[2] + q[3] + q[4];
        let gap = (q[0].abs() - kcl - G * 8.0 * t).abs() / ((1e-9 + 1e-3 * (kcl / t).abs()) * t);
        eprintln!("to {t}: KCL {gap:.3e} W0");
        assert!(gap <= 1.0, "the PSU's KCL within the coupling's gap: {gap} W0");
    }
    let mut e = Engine::load(&p).unwrap();
    e.charge_arm(&[0, 1]).unwrap();
    e.advance(5e-3).unwrap();
    let b = e.take_charge().to_vec();
    assert_eq!(b[b.len() - 2..], [1.0, 0.0], "the PSU's charge says its KCL is approximate");
    assert!(e.stat(28) > 0.0, "stat 28 counts the intervals");
}

#[test]
fn a_device_armed_alone_charges_as_it_does_armed_with_everything() {
    // W2bV finding 2: a resistor across a CC rail with no capacitor, armed
    // alone, read 0 (its carrier's steps were kept only when a device of
    // that partition or a source was armed too).
    let mut two = cc_rail_cap_led(47e-6, 0.02);
    two.devices.remove(1);
    two.devices.push(dev(kinds::RESISTOR, 7, 1, 4, vec![137.0]));
    two.devices.push(dev(kinds::CAPACITOR, 8, 4, 0, vec![4.4e-6, 0.0, 0.0]));
    two.node_count = 5;
    let boards = [
        ("cc rail, no capacitor, stepped", cc_rail_no_cap()),
        ("cc rail, capacitor, led", cc_rail_cap_led(47e-6, 0.02)),
        ("cc rail, no capacitor, led", { let mut p = cc_rail_cap_led(47e-6, 0.02); p.devices.remove(1); p }),
        ("cc rail, no capacitor, two hubs", two),
        ("cc rail", cc_rail()),
        ("cell rail, stepped", stepped(cell_rail(), false)),
    ];
    let steps = [0.37e-3, 1e-3, 5e-3];
    for (name, p) in boards {
        let all: Vec<u32> = (0..p.devices.len() as u32).collect();
        let (_, qa) = charge(&p, &all, &steps);
        for &d in &all {
            let (_, q) = charge(&p, &[d], &steps);
            assert_eq!(qa.len(), all.len(), "{name}: no channel slots");
            let k = d as usize;
            assert!((q[0] - qa[k]).abs() <= 1e-12 * qa[k].abs(), "{name}: device {d} alone {:e} against {:e} with everything", q[0], qa[k]);
        }
    }
}
