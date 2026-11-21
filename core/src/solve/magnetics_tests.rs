//! W1h H4b: the inductive state in the stepped solver (spec 7.5, 7.2.3, D35;
//! rulings R56 6-8), through a real `Engine`.
//!
//! Every expected number is the circuit's own closed form, worked by hand: a
//! series RL's step response `i(t) = (V/R)(1 - exp(-t R/L))`, an inductor
//! straight across a source `i(t) = V/dcr + (0 - V/dcr) exp(-t dcr/L)` (or the
//! ramp `V t / L` with no dcr), never core output. A stepped trajectory is
//! advanced in short horizons (tau / 2000), so its steps
//! are short and what is tested is the stamp, not the controller's error
//! (which TS holds against classic's, test/core/magnetics-engine.test.ts), and
//! held to W0 (100 uV + 1e-3 relative on a voltage, 1 nA + 1e-3 relative on a
//! current); a closed form read at a time to 1e-9 relative.

use crate::engine::Engine;
use crate::error::ERR_UNSUPPORTED_TOPOLOGY;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob: vec![] }
}

fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
    dev(kinds::VOLTAGE_SOURCE, ci, &[pos, neg], vec![v])
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    dev(kinds::RESISTOR, ci, &[a, b], vec![r])
}

fn inductor(ci: usize, a: usize, b: usize, l: f64, dcr: f64, core_loss: f64) -> Device {
    dev(kinds::INDUCTOR, ci, &[a, b], vec![l, dcr, core_loss])
}

fn program(node_count: usize, devices: Vec<Device>, options: Options) -> Program {
    Program { node_count, options, digital_slot_count: 0, channel_slot_count: 0, devices, carry: Vec::new() }
}

fn trap() -> Options {
    Options { integration: 1, ..Options::default() }
}

#[track_caller]
fn w0_v(what: &str, got: f64, want: f64) {
    assert!((got - want).abs() <= 100e-6 + 1e-3 * want.abs(), "{what}: got {got:e}, want {want:e}");
}

#[track_caller]
fn w0_i(what: &str, got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-9 + 1e-3 * want.abs(), "{what}: got {got:e}, want {want:e}");
}

/// Advances `e` to `t` in horizons of at most `dt`.
fn advance_fine(e: &mut Engine, t: f64, dt: f64) {
    let mut now = e.time();
    while now < t {
        now = (now + dt).min(t);
        e.advance(now).unwrap();
    }
}

#[track_caller]
fn exact(what: &str, got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-15 + 1e-9 * want.abs(), "{what}: got {got:e}, want {want:e}");
}

/// 5 V -> 100 ohm -> node 2 -> 10 mH (dcr, core loss) -> ground: tau = L/(R + dcr).
fn series_rl(dcr: f64, core_loss: f64, options: Options) -> Program {
    program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 100.0), inductor(2, 2, 0, 10e-3, dcr, core_loss)], options)
}

#[test]
fn a_series_rl_follows_its_step_response_with_backward_euler_and_the_trapezoid() {
    for options in [Options::default(), trap()] {
        let mut e = Engine::load(&series_rl(0.0, 0.0, options)).unwrap();
        // Q-O29: the load seed carries no current, so the node starts at the supply.
        w0_v("node 2 at load", e.voltages()[2], 5.0);
        w0_i("inductor at load", e.currents()[2], 0.0);
        let tau = 10e-3 / 100.0;
        for t in [0.25 * tau, tau, 3.0 * tau, 10.0 * tau] {
            advance_fine(&mut e, t, tau / 2000.0);
            let i = 0.05 * (1.0 - (-t / tau).exp());
            w0_v(&format!("node 2 at {t:e} (integration {})", options.integration), e.voltages()[2], 5.0 - 100.0 * i);
            w0_i(&format!("elementI at {t:e}"), e.currents()[2], i);
            w0_i(&format!("resistor at {t:e}"), e.currents()[1], i);
            // The source delivers the winding current (KCL through the node).
            w0_i(&format!("source at {t:e}"), e.currents()[0], -i);
        }
    }
}

#[test]
fn the_winding_resistance_and_the_core_loss_shunt_shape_the_response() {
    // dcr 25 ohm: tau = L / 125, final current 5 / 125. Core loss 10 kohm across
    // the inductor: elementI = winding + V / 10k (spec 7.5), the node settles at
    // the divider of 100 against (25 || 10k).
    let mut e = Engine::load(&series_rl(25.0, 10_000.0, Options::default())).unwrap();
    e.advance(0.02).unwrap();
    let r_par = 25.0 * 10_000.0 / (25.0 + 10_000.0);
    let v2 = 5.0 * r_par / (100.0 + r_par);
    w0_v("node 2 settled", e.voltages()[2], v2);
    w0_i("elementI settled", e.currents()[2], v2 / 25.0 + v2 / 10_000.0);
    let state = e.device_state(2).unwrap().to_vec();
    assert_eq!(state.len(), 2, "[i, v]");
    w0_i("device state i (the winding alone)", state[0], v2 / 25.0);
    exact("device state v", state[1], e.voltages()[2]);
}

#[test]
fn an_inductor_across_a_source_is_a_closed_form_read_at_the_time() {
    // No partition holds it (both ends fixed, R56 6): with a winding
    // resistance the current rises to V/dcr; without, it ramps as V t / L.
    for (dcr, t) in [(2.0, 3e-3), (0.0, 7e-4)] {
        let p = program(2, vec![source(0, 1, 0, 5.0), inductor(1, 1, 0, 10e-3, dcr, 0.0)], Options::default());
        let mut e = Engine::load(&p).unwrap();
        exact("at load", e.currents()[1], 0.0);
        e.advance(t).unwrap();
        let want = if dcr > 0.0 { (5.0 / dcr) * (1.0 - (-t * dcr / 10e-3).exp()) } else { 5.0 * t / 10e-3 };
        exact(&format!("dcr {dcr}"), e.currents()[1], want);
        exact(&format!("dcr {dcr}: the source carries it"), e.currents()[0], -want);
        let state = e.device_state(1).unwrap().to_vec();
        exact("device state i", state[0], want);
        exact("device state v", state[1], 5.0);
    }
}

#[test]
fn a_self_shorted_inductor_stamps_nothing_and_reads_zero() {
    let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 100.0), resistor(2, 2, 0, 100.0), inductor(3, 2, 2, 1e-3, 0.0, 0.0)], Options::default());
    let mut e = Engine::load(&p).unwrap();
    e.advance(1e-3).unwrap();
    exact("divider", e.voltages()[2], 2.5);
    assert_eq!(e.currents()[3], 0.0);
}

#[test]
fn an_inductor_on_a_hub_rail_or_across_a_moving_fixed_pair_declines() {
    // R56 6: a pin on the bench PSU's rail node falls back to classic.
    let psu = dev(kinds::BENCH_PSU, 0, &[1, 0], vec![5.0, 1.0]);
    let on_rail = program(3, vec![psu, inductor(1, 1, 2, 1e-3, 0.0, 0.0), resistor(2, 2, 0, 100.0)], Options::default());
    assert_eq!(Engine::load(&on_rail).err().map(|e| e.code), Some(ERR_UNSUPPORTED_TOPOLOGY));
    // A fixed pair whose voltage a pulse ramp moves: its ramp response is not ported.
    let pulse = dev(kinds::PULSE_SOURCE, 0, &[1, 0], vec![0.0, 5.0, 1e-3, 1e-4, 1e-4, 1e-3, 4e-3]);
    let mut pulse = pulse;
    pulse.blob = vec![0.0];
    let ramped = Program { node_count: 2, options: Options::default(), digital_slot_count: 1, channel_slot_count: 0, carry: Vec::new(), devices: vec![pulse, inductor(1, 1, 0, 1e-3, 1.0, 0.0)] };
    assert_eq!(Engine::load(&ramped).err().map(|e| e.code), Some(ERR_UNSUPPORTED_TOPOLOGY));
}

#[test]
fn an_inductor_partition_is_stepped_never_closed_form() {
    // One inductor and resistors only: a ClosedForm partition would have no
    // capacitor to integrate; it must be stepped (R56 6) and take steps.
    let mut e = Engine::load(&series_rl(0.0, 0.0, Options::default())).unwrap();
    e.advance(1e-3).unwrap();
    assert!(e.stat(10) > 0.0, "stepped steps: {}", e.stat(10));
    assert_eq!(e.stat(8), 0.0, "no closed-form segment");
}

#[test]
fn a_switch_opening_on_an_inductor_keeps_its_current_continuous() {
    // 5 V -> 100 ohm -> node 2 -> 10 mH -> ground, with 1 kohm from node 2 to
    // ground behind a closed switch. At 1 ms (10 tau of the 100 ohm loop, so
    // the winding carries 50 mA), the feed resistor's switch opens: the
    // winding current is continuous, so it now flows through the 1 kohm and
    // node 2 jumps to -50 mA * 1 kohm = -50 V, then decays with tau = L/1k.
    let switch = dev(kinds::SWITCH, 4, &[1, 3], vec![1.0]);
    let p = program(4, vec![source(0, 1, 0, 5.0), resistor(1, 3, 2, 100.0), inductor(2, 2, 0, 10e-3, 0.0, 0.0), resistor(3, 2, 0, 1000.0), switch], Options::default());
    let mut e = Engine::load(&p).unwrap();
    e.advance(1e-3).unwrap();
    // Settled: the inductor shorts node 2, so the winding carries 5/100 (tau =
    // L / (100 || 1k) = 0.11 ms, so 1 ms is past 9 tau).
    let i_settled = 0.05 * (1.0 - (-1e-3f64 / (10e-3 / (100.0 * 1000.0 / 1100.0))).exp());
    w0_i("winding before the opening", e.device_state(2).unwrap()[0], i_settled);
    e.set_params(&[(4, 0, 0.0)]).unwrap();
    let tau = 10e-3 / 1000.0;
    for dt in [0.5 * tau, 2.0 * tau] {
        advance_fine(&mut e, 1e-3 + dt, tau / 2000.0);
        let i = i_settled * (-dt / tau).exp();
        w0_i(&format!("winding {dt:e} after the opening"), e.device_state(2).unwrap()[0], i);
        w0_v(&format!("node 2 {dt:e} after the opening"), e.voltages()[2], -1000.0 * i);
    }
}

#[test]
fn an_inductor_in_a_hub_partition_follows_the_rail_in_both_rail_modes() {
    // A 5 V bench PSU (1 A limit, CV throughout) -> 100 ohm -> node 2 -> 10 mH
    // with a 10 kohm core loss -> ground: the inductor sits in the rail's hub
    // partition (not on the rail itself), and steps with the rail held.
    for exact_mode in [false, true] {
        let psu = dev(kinds::BENCH_PSU, 0, &[1, 0], vec![5.0, 1.0]);
        let options = Options { no_rail_shortcut: exact_mode, ..Options::default() };
        let p = program(3, vec![psu, resistor(1, 1, 2, 100.0), inductor(2, 2, 0, 10e-3, 0.0, 10_000.0)], options);
        let mut e = Engine::load(&p).unwrap();
        let tau = 10e-3 / (100.0 * 10_000.0 / 10_100.0);
        for t in [0.5 * tau, 2.0 * tau] {
            advance_fine(&mut e, t, tau / 2000.0);
            // The winding sees the Thevenin source 5 * 10k/10.1k behind 100 || 10k.
            let (v_th, r_th) = (5.0 * 10_000.0 / 10_100.0, 100.0 * 10_000.0 / 10_100.0);
            let i = (v_th / r_th) * (1.0 - (-t / tau).exp());
            let v2 = v_th - r_th * i;
            w0_i(&format!("winding at {t:e}, exact {exact_mode}"), e.device_state(2).unwrap()[0], i);
            w0_v(&format!("node 2 at {t:e}, exact {exact_mode}"), e.voltages()[2], v2);
            w0_i(&format!("elementI at {t:e}, exact {exact_mode}"), e.currents()[2], i + v2 / 10_000.0);
        }
    }
}

// ---- coupled_inductor (spec 7.16) -------------------------------------------

/// A 5 V step through `r1` into winding 1 (a1 node 2, b1 ground), winding 2
/// (a2 node 3, b2 ground) loaded by `r2`. Records `[l1, l2, k, dcr1, dcr2]`.
fn transformer(l1: f64, l2: f64, k: f64, r1: f64, r2: f64, options: Options) -> Program {
    program(4, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, r1), dev(kinds::COUPLED_INDUCTOR, 2, &[2, 0, 3, 0], vec![l1, l2, k, 0.0, 0.0]), resistor(3, 3, 0, r2)], options)
}

/// The windings' currents by RK4 on `L di/dt = v`, `L = [[l1, M], [M, l2]]`,
/// `v1 = 5 - r1 i1`, `v2 = -r2 i2` (winding 2's current leaves a2 into the
/// winding, so the load returns it), at 1e-9 s: an independent reference.
fn transformer_reference(l1: f64, l2: f64, k: f64, r1: f64, r2: f64, t: f64) -> (f64, f64) {
    let m = k * (l1 * l2).sqrt();
    let det = l1 * l2 - m * m;
    let f = |i: [f64; 2]| {
        let (v1, v2) = (5.0 - r1 * i[0], -r2 * i[1]);
        [(l2 * v1 - m * v2) / det, (l1 * v2 - m * v1) / det]
    };
    let h = 1e-9;
    let mut i = [0.0, 0.0];
    let steps = (t / h).round() as usize;
    for _ in 0..steps {
        let k1 = f(i);
        let k2 = f([i[0] + 0.5 * h * k1[0], i[1] + 0.5 * h * k1[1]]);
        let k3 = f([i[0] + 0.5 * h * k2[0], i[1] + 0.5 * h * k2[1]]);
        let k4 = f([i[0] + h * k3[0], i[1] + h * k3[1]]);
        i = [i[0] + h / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]), i[1] + h / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])];
    }
    (i[0], i[1])
}

#[test]
fn a_coupled_inductor_steps_both_windings_with_backward_euler_and_the_trapezoid() {
    // 1 mH : 4 mH (turns 1:2), k = 0.95, 10 ohm feed, 1 kohm load.
    let (l1, l2, k, r1, r2) = (1e-3, 4e-3, 0.95, 10.0, 1000.0);
    for options in [Options::default(), trap()] {
        let mut e = Engine::load(&transformer(l1, l2, k, r1, r2, options)).unwrap();
        for t in [2e-6, 1e-5, 5e-5] {
            advance_fine(&mut e, t, 1e-8);
            let (i1, i2) = transformer_reference(l1, l2, k, r1, r2, t);
            let state = e.device_state(2).unwrap().to_vec();
            assert_eq!(state.len(), 4, "[i1, i2, v1, v2]");
            let what = format!("at {t:e} (integration {})", options.integration);
            w0_i(&format!("i1 {what}"), state[0], i1);
            w0_i(&format!("i2 {what}"), state[1], i2);
            w0_v(&format!("v1 {what}"), state[2], 5.0 - r1 * i1);
            w0_v(&format!("v2 {what}"), state[3], -r2 * i2);
            exact(&format!("v2 is the published node 3 {what}"), state[3], e.voltages()[3]);
            // elementI is winding 1's current (Q-O78).
            exact(&format!("elementI {what}"), e.currents()[2], state[0]);
            // The supply carries winding 1's current, KCL through the feed.
            w0_i(&format!("source {what}"), e.currents()[0], -i1);
        }
    }
}

#[test]
fn a_coupled_inductor_with_no_free_pin_or_a_pin_on_the_hub_rail_declines() {
    let all_fixed = program(3, vec![source(0, 1, 0, 5.0), source(1, 2, 0, 1.0), dev(kinds::COUPLED_INDUCTOR, 2, &[1, 0, 2, 0], vec![1e-3, 1e-3, 0.9, 0.0, 0.0])], Options::default());
    assert_eq!(Engine::load(&all_fixed).err().map(|e| e.code), Some(ERR_UNSUPPORTED_TOPOLOGY));
    let psu = dev(kinds::BENCH_PSU, 0, &[1, 0], vec![5.0, 1.0]);
    let on_rail = program(4, vec![psu, dev(kinds::COUPLED_INDUCTOR, 1, &[1, 2, 3, 0], vec![1e-3, 1e-3, 0.9, 0.0, 0.0]), resistor(2, 2, 0, 10.0), resistor(3, 3, 0, 10.0)], Options::default());
    assert_eq!(Engine::load(&on_rail).err().map(|e| e.code), Some(ERR_UNSUPPORTED_TOPOLOGY));
}

// ---- P0-MAG: the line's machinery (capacitor-current work, snapshot, carry) --

fn capacitor(ci: usize, a: usize, b: usize, c: f64) -> Device {
    dev(kinds::CAPACITOR, ci, &[a, b], vec![c, 0.0, 0.0])
}

/// A carry block of one capacitor's charge (`crate::carry`'s record).
fn carried_charge(device: usize, vc: f64) -> Vec<f64> {
    let mut out = Vec::new();
    crate::carry::push(&mut out, device, crate::carry::FAMILY_CAPACITOR, &[vc]);
    out
}

/// An undriven LC tank (1 uF and 1 mH from node 1 to ground) charged to 1 V by
/// a carried charge: its energy `C v^2 / 2 + L i^2 / 2`.
fn tank_energy(e: &mut Engine) -> f64 {
    let v = e.voltages()[1];
    let i = e.device_state(1).unwrap()[0];
    0.5 * 1e-6 * v * v + 0.5 * 1e-3 * i * i
}

#[test]
fn an_undriven_lc_tanks_energy_never_grows_over_1e5_periods() {
    // An LC mode is on the imaginary axis: the extrapolated trapezoid and the
    // extrapolated TR-BDF2 (CAPERR2, CAPERR4) were derived for real rates, and
    // the latter's |E| reaches 1.075 near the axis, so such a partition stays
    // on the plain step (`linear_trap_ok`, `fastest_rate_bound`). The plain
    // trapezoid's amplification on the axis is 1, its first backward Euler
    // step's below: the energy may only fall. Sampled every 1000 periods
    // (period 2 pi sqrt(LC) = 198.7 us), under either INTEGRATION option.
    for options in [Options::default(), trap()] {
        let mut p = program(2, vec![capacitor(0, 1, 0, 1e-6), inductor(1, 1, 0, 1e-3, 0.0, 0.0)], options);
        p.carry = carried_charge(0, 1.0);
        let mut e = Engine::load(&p).unwrap();
        crate::solve::stepped::EXTRAPOLATED.with(|c| c.set(0));
        crate::solve::stepped::EXTRAP_TRBDF2.with(|c| c.set(0));
        let e0 = tank_energy(&mut e);
        w0_v("the carried charge at load", e.voltages()[1], 1.0);
        let period = 2.0 * std::f64::consts::PI * (1e-3f64 * 1e-6).sqrt();
        let mut worst = 0.0f64;
        for k in 1..=100 {
            e.advance(k as f64 * 1000.0 * period).unwrap();
            let energy = tank_energy(&mut e);
            worst = worst.max(energy / e0);
            assert!(energy <= e0 * (1.0 + 1e-9), "integration {}: energy {energy:e} at {k}000 periods against {e0:e} at load", options.integration);
        }
        assert_eq!(crate::solve::stepped::EXTRAPOLATED.with(|c| c.get()), 0, "no extrapolated trapezoid step");
        assert_eq!(crate::solve::stepped::EXTRAP_TRBDF2.with(|c| c.get()), 0, "no extrapolated TR-BDF2 step");
        assert!(worst > 0.0);
    }
}

#[test]
#[ignore = "P0-MAG finding, reported, not loosened: the step-doubling controller's per-step tolerance accumulates to 2.40 W0 by the 7th extreme (0.47, 0.93, 1.37, 1.77, 2.11, 2.34, 2.40, 2.30 W0 at extremes 1-8, ~25 steps a half period); needs a ruling (ship-logs/p0-mag/NOTES.md)"]
fn a_series_rlc_ringdown_matches_its_closed_form_at_every_peak() {
    // 1 uF charged to 5 V (carried) discharging through 10 ohm and 1 mH:
    // alpha = R / 2L = 5000 /s, w0 = 1/sqrt(LC) = 31623 rad/s, wd =
    // sqrt(w0^2 - alpha^2); the capacitor's voltage is V0 e^(-alpha t) (cos wd t
    // + (alpha/wd) sin wd t), held to W0 at its extremes (where a phase error
    // costs nothing to first order) over the first eight half periods, under
    // either INTEGRATION option, on the engine's own controller.
    for options in [Options::default(), trap()] {
        let mut p = program(3, vec![capacitor(0, 1, 0, 1e-6), resistor(1, 1, 2, 10.0), inductor(2, 2, 0, 1e-3, 0.0, 0.0)], options);
        p.carry = carried_charge(0, 5.0);
        let mut e = Engine::load(&p).unwrap();
        crate::solve::stepped::EXTRAPOLATED.with(|c| c.set(0));
        crate::solve::stepped::EXTRAP_TRBDF2.with(|c| c.set(0));
        let (alpha, w0) = (10.0 / (2.0 * 1e-3), 1.0 / (1e-3f64 * 1e-6).sqrt());
        let wd = (w0 * w0 - alpha * alpha).sqrt();
        let v = |t: f64| 5.0 * (-alpha * t).exp() * ((wd * t).cos() + alpha / wd * (wd * t).sin());
        // The extremes of v: dv/dt = -V0 (w0^2 / wd) e^(-alpha t) sin wd t = 0.
        let mut worst = (0.0f64, 0usize);
        for k in 1..=8 {
            let t = k as f64 * std::f64::consts::PI / wd;
            e.advance(t).unwrap();
            let r = (e.voltages()[1] - v(t)).abs() / (100e-6 + 1e-3 * v(t).abs());
            eprintln!("RLC integration {} extreme {k}: got {:.9} want {:.9} ({r:.3} W0), steps {}", options.integration, e.voltages()[1], v(t), e.stat(10));
            if r > worst.0 {
                worst = (r, k);
            }
        }
        assert!(worst.0 <= 1.0, "integration {}: extreme {} is {:.3} W0 off the closed form", options.integration, worst.1, worst.0);
        assert_eq!(crate::solve::stepped::EXTRAPOLATED.with(|c| c.get()) + crate::solve::stepped::EXTRAP_TRBDF2.with(|c| c.get()), 0, "no extrapolated step");
    }
}

#[test]
fn the_same_rc_without_its_winding_does_extrapolate() {
    // The control for the two tests above: the ringdown's resistor and
    // capacitor with the winding replaced by a 10 ohm return take
    // extrapolated steps, so the counters the tests read do count.
    // Forced stepped: one capacitor alone is a closed form.
    let mut p = program(3, vec![capacitor(0, 1, 0, 1e-6), resistor(1, 1, 2, 10.0), resistor(2, 2, 0, 10.0)], Options { force_stepped: true, ..trap() });
    p.carry = carried_charge(0, 5.0);
    let mut e = Engine::load(&p).unwrap();
    crate::solve::stepped::EXTRAPOLATED.with(|c| c.set(0));
    crate::solve::stepped::EXTRAP_TRBDF2.with(|c| c.set(0));
    e.advance(1e-4).unwrap();
    assert!(crate::solve::stepped::EXTRAPOLATED.with(|c| c.get()) + crate::solve::stepped::EXTRAP_TRBDF2.with(|c| c.get()) > 0, "an RC partition extrapolates");
}

#[test]
fn a_snapshot_carries_the_winding_currents_and_their_history() {
    // A series RL and a transformer mid-transient: a run restored from a
    // snapshot replays the run that went on, bit for bit, in every reading
    // and in every winding's device state.
    let mut rl = series_rl(25.0, 10_000.0, trap());
    rl.devices.extend(transformer(1e-3, 4e-3, 0.95, 200.0, 1000.0, trap()).devices.into_iter().skip(1).map(|mut d| {
        d.component_index += 10;
        for p in &mut d.pins {
            if p.node >= 2 {
                p.node += 1;
            }
        }
        d
    }));
    rl.node_count += 2;
    let mut e = Engine::load(&rl).unwrap();
    e.advance(3e-5).unwrap();
    let snap = e.snapshot();
    e.advance(1e-4).unwrap();
    let (v, i, s2, s4) = (e.voltages().to_vec(), e.currents().to_vec(), e.device_state(2).unwrap().to_vec(), e.device_state(4).unwrap().to_vec());
    let mut back = Engine::load(&rl).unwrap();
    back.restore(&snap);
    back.advance(1e-4).unwrap();
    let bits = |x: &[f64]| x.iter().map(|f| f.to_bits()).collect::<Vec<_>>();
    assert_eq!(bits(back.voltages()), bits(&v));
    assert_eq!(bits(back.currents()), bits(&i));
    assert_eq!(bits(back.device_state(2).unwrap()), bits(&s2), "the inductor's [i, v]");
    assert_eq!(bits(back.device_state(4).unwrap()), bits(&s4), "the coupled inductor's [i1, i2, v1, v2]");
    assert!(s2[0].abs() > 1e-6 && s4[0].abs() > 1e-6 && s4[1].abs() > 1e-9, "every winding carries current: {s2:?} {s4:?}");
}

/// The carry a reload of `p` at `t` takes, seeded into the program it loads.
fn reloaded(p: &Program, t: f64) -> (Engine, Engine) {
    let mut e = Engine::load(p).unwrap();
    e.advance(t).unwrap();
    let carry = e.carry_take().unwrap().to_vec();
    let mut q = p.clone();
    q.carry = carry;
    q.options.start_time = t;
    (e, Engine::load(&q).unwrap())
}

#[test]
fn a_reload_that_keeps_the_part_carries_each_winding_current() {
    // Classic's load() carries `inds` by id (SE:3077-3117): an inductor's
    // winding current, a coupled inductor's pair as one, and the fixed-pair
    // inductor's too. The reloaded engine starts from the taken currents and
    // follows the run that went on to W0 (its first step is backward Euler,
    // the run's was the trapezoid). Without the carry the winding would
    // restart at 0 (Q-O29).
    let mut p = series_rl(25.0, 10_000.0, trap());
    p.devices.extend(transformer(1e-3, 4e-3, 0.95, 200.0, 1000.0, trap()).devices.into_iter().skip(1).map(|mut d| {
        d.component_index += 10;
        for pin in &mut d.pins {
            if pin.node >= 2 {
                pin.node += 1;
            }
        }
        d
    }));
    p.devices.push(inductor(20, 1, 0, 10e-3, 2.0, 0.0));
    p.node_count += 2;
    let t = 3e-5;
    let (mut on, mut back) = reloaded(&p, t);
    let fixed = 5.0 / 2.0 * -(-t * 2.0 / 10e-3f64).exp_m1();
    for d in [2u32, 4, 6] {
        let (a, b) = (on.device_state(d).unwrap().to_vec(), back.device_state(d).unwrap().to_vec());
        assert!(a[0].abs() > 1e-6, "device {d} carries current: {a:?}");
        let windings = if d == 4 { 2 } else { 1 };
        for w in 0..windings {
            assert_eq!(a[w].to_bits(), b[w].to_bits(), "device {d} winding {w} at the reload");
        }
    }
    exact("the fixed-pair inductor's carried current", back.device_state(6).unwrap()[0], fixed);
    for t2 in [5e-5, 1e-4, 3e-4] {
        on.advance(t2).unwrap();
        back.advance(t2).unwrap();
        for d in [2u32, 4, 6] {
            let (a, b) = (on.device_state(d).unwrap().to_vec(), back.device_state(d).unwrap().to_vec());
            let windings = if d == 4 { 2 } else { 1 };
            for w in 0..windings {
                w0_i(&format!("device {d} winding {w} at {t2:e}"), b[w], a[w]);
            }
        }
    }
}
