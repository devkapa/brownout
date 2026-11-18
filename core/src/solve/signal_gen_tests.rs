//! W1h H3b step 1: `signal_gen`'s three forms (spec 8.12.2) for the piecewise
//! constant shapes (`dc`, `square`, `noise`, a `pulse` under W1f Q5's
//! threshold), through a real `Engine`.
//!
//! Every expected number is worked by hand from the circuit and the shape's
//! level (`waveform::noise_unit` for a noise draw, H3a's port held against
//! classic's golden values): a Norton generator into a load reads `Vt R/(R +
//! r)`, an RC behind it the exponential of its Thevenin equivalent. Levels
//! change at the event instants, which come from the index (Q-O68): one ns
//! either side of an edge reads the old and the new level. Voltages are held
//! to W0 (100 uV + 1e-3 relative), currents to 1 nA + 1e-3 relative, a static
//! divider to 1e-9 relative (its only error is the node shunt).

use crate::engine::Engine;
use crate::error::ERR_UNSUPPORTED_TOPOLOGY;
use crate::kinds::{self, signal_gen_param as sg};
use crate::program::{Device, Options, PinRecord, Program};
use crate::waveform;

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob }
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    dev(kinds::RESISTOR, ci, &[a, b], vec![r], vec![])
}

fn capacitor(ci: usize, a: usize, b: usize, c: f64) -> Device {
    dev(kinds::CAPACITOR, ci, &[a, b], vec![c, 0.0, 0.0], vec![])
}

/// A generator with the catalog's defaults but for `shape`, `amplitude`,
/// `offset`, `frequency`, and the form's `enabled` and `rSource`.
fn generator(ci: usize, pos: usize, neg: usize, shape: u32, amplitude: f64, offset: f64, frequency: f64, r_source: f64) -> Device {
    let mut params = vec![0.0; sg::COUNT];
    params[sg::WAVEFORM] = shape as f64;
    params[sg::AMPLITUDE] = amplitude;
    params[sg::OFFSET] = offset;
    params[sg::FREQUENCY] = frequency;
    params[sg::DUTY] = 0.5;
    params[sg::TR] = 1e-6;
    params[sg::TF] = 1e-6;
    params[sg::PW] = 5e-4;
    params[sg::SEED] = 1.0;
    params[sg::ENABLED] = 1.0;
    params[sg::R_SOURCE] = r_source;
    dev(kinds::SIGNAL_GEN, ci, &[pos, neg], params, vec![0.0])
}

/// A 0/5 V square (amplitude 2.5 about 2.5), 1 kHz, 50 % duty: high on
/// [0, 0.5 ms), low on [0.5, 1 ms).
fn square(ci: usize, pos: usize, neg: usize, r_source: f64) -> Device {
    generator(ci, pos, neg, sg::SHAPE_SQUARE, 2.5, 2.5, 1000.0, r_source)
}

fn program(node_count: usize, devices: Vec<Device>, options: Options) -> Program {
    Program { node_count, options, digital_slot_count: 0, channel_slot_count: 0, devices, carry: Vec::new() }
}

#[track_caller]
fn w0_v(what: &str, got: f64, want: f64) {
    assert!((got - want).abs() <= 100e-6 + 1e-3 * want.abs(), "{what}: got {got:e}, want {want:e}");
}

#[track_caller]
fn w0_i(what: &str, got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-9 + 1e-3 * want.abs(), "{what}: got {got:e}, want {want:e}");
}

#[track_caller]
fn tight(what: &str, got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-12 + 1e-9 * want.abs(), "{what}: got {got:e}, want {want:e}");
}

fn at(e: &mut Engine, t: f64) -> &mut Engine {
    e.advance(t).unwrap();
    e
}

/// Advances `e` to `t` in horizons of at most `dt`.
fn advance_fine(e: &mut Engine, t: f64, dt: f64) {
    let mut now = e.time();
    while now < t {
        now = (now + dt).min(t);
        e.advance(now).unwrap();
    }
}

/// The square's level at `t`, away from an edge.
fn square_level(t: f64) -> f64 {
    if (t * 1000.0).fract() < 0.5 {
        5.0
    } else {
        0.0
    }
}

#[test]
fn a_norton_square_into_a_resistor_reads_the_divided_level_and_steps_at_its_edge_instants() {
    // Node 1: generator (50 ohm) to ground, 1 kohm to ground.
    let mut e = Engine::load(&program(2, vec![square(0, 1, 0, 50.0), resistor(1, 1, 0, 1000.0)], Options::default())).unwrap();
    let divided = |vt: f64| vt * 1000.0 / 1050.0;
    tight("at load", e.voltages()[1], divided(5.0));
    for &(t, vt) in &[(0.25e-3, 5.0), (0.5e-3 - 1e-9, 5.0), (0.5e-3, 0.0), (0.75e-3, 0.0), (1e-3 - 1e-9, 0.0), (1e-3, 5.0), (1.25e-3, 5.0), (2.5e-3, 0.0), (7.25e-3, 5.0)] {
        let e = at(&mut e, t);
        tight(&format!("node at {t:e}"), e.voltages()[1], divided(vt));
        // Q-O67: the current out of `pos`, positive while delivering.
        tight(&format!("elementI at {t:e}"), e.currents()[0], vt / 1050.0);
        tight(&format!("load current at {t:e}"), e.currents()[1], divided(vt) / 1000.0);
    }
}

#[test]
fn an_ideal_square_fixes_its_node_and_reads_the_branch_current_negative_while_delivering() {
    for r_source in [0.0, -10.0] {
        let mut e = Engine::load(&program(2, vec![square(0, 1, 0, r_source), resistor(1, 1, 0, 1000.0)], Options::default())).unwrap();
        for &(t, vt) in &[(0.0, 5.0), (0.25e-3, 5.0), (0.5e-3 - 1e-9, 5.0), (0.5e-3, 0.0), (1e-3, 5.0), (3.75e-3, 0.0)] {
            let e = at(&mut e, t);
            assert_eq!(e.voltages()[1], vt, "rSource {r_source}: node at {t:e}");
            tight(&format!("rSource {r_source}: elementI at {t:e}"), e.currents()[0], -vt / 1000.0);
        }
    }
}

#[test]
fn a_disabled_generator_stamps_nothing_and_reads_no_current() {
    for r_source in [50.0, 0.0] {
        let mut g = square(0, 1, 0, r_source);
        g.params[sg::ENABLED] = 0.0;
        // A 5 V supply through 1 kohm to the generator's node: disabled, the
        // node reads the supply (the shunt aside), as nothing loads it.
        let devices = vec![g, dev(kinds::VOLTAGE_SOURCE, 1, &[2, 0], vec![5.0], vec![]), resistor(2, 2, 1, 1000.0), capacitor(3, 1, 0, 1e-9)];
        let mut e = Engine::load(&program(3, devices, Options::default())).unwrap();
        for t in [1e-4, 0.6e-3, 1.2e-3] {
            let e = at(&mut e, t);
            w0_v(&format!("rSource {r_source}: node at {t:e}"), e.voltages()[1], 5.0);
            assert_eq!(e.currents()[0], 0.0, "rSource {r_source}: elementI at {t:e}");
        }
    }
}

/// Generator (50 ohm) into 1 kohm || 1 uF: Thevenin `Vt 1000/1050` behind
/// `50 || 1000`, so the node follows `Vth + (v0 - Vth) exp(-dt/tau)` from each
/// edge, tau = 47.6 us, ten time constants to a half period.
fn rc_behind_a_square(options: Options) -> Program {
    program(2, vec![square(0, 1, 0, 50.0), resistor(1, 1, 0, 1000.0), capacitor(2, 1, 0, 1e-6)], options)
}

fn rc_node(t: f64) -> f64 {
    let tau = (50.0 * 1000.0 / 1050.0) * 1e-6;
    let vth = |vt: f64| vt * 1000.0 / 1050.0;
    // Settled at each edge to within exp(-10.5): start the exponential from
    // the previous level's settled value.
    let half = 0.5e-3;
    let edge = (t / half).floor() * half;
    let now = square_level(t);
    let before = if now == 5.0 { 0.0 } else { 5.0 };
    vth(now) + (vth(before) - vth(now)) * (-(t - edge) / tau).exp()
}

#[test]
fn a_norton_square_drives_a_closed_form_rc_through_its_thevenin_equivalent() {
    let mut e = Engine::load(&rc_behind_a_square(Options::default())).unwrap();
    for t in [1.0e-3 + 10e-6, 1.0e-3 + 47.6e-6, 1.0e-3 + 200e-6, 1.5e-3 + 1e-6, 1.5e-3 + 60e-6, 2.0e-3 + 30e-6, 4.5e-3 + 100e-6] {
        let e = at(&mut e, t);
        w0_v(&format!("node at {t:e}"), e.voltages()[1], rc_node(t));
        let vt = square_level(t);
        w0_i(&format!("elementI at {t:e}"), e.currents()[0], (vt - rc_node(t)) / 50.0);
    }
}

#[test]
fn a_norton_square_drives_a_stepped_rc_through_the_same_equivalent() {
    for integration in [0u8, 1] {
        let options = Options { force_stepped: true, integration, ..Options::default() };
        let mut e = Engine::load(&rc_behind_a_square(options)).unwrap();
        e.advance(1.0e-3).unwrap();
        // Horizons of tau / 2000 keep the steps short, so what is tested is
        // the stamp, not the controller's error.
        for t in [1.0e-3 + 10e-6, 1.0e-3 + 47.6e-6, 1.0e-3 + 200e-6, 1.5e-3 + 1e-6, 1.5e-3 + 60e-6] {
            advance_fine(&mut e, t, 25e-9);
            w0_v(&format!("integration {integration}: node at {t:e}"), e.voltages()[1], rc_node(t));
        }
    }
}

#[test]
fn a_norton_drive_in_a_hub_partition_moves_the_rail_current() {
    // A 10 V PSU with a 1 A limit (a hub, in CV) on node 1, 1 kohm to node 2,
    // and the generator (1 kohm) from node 2 to ground: V2 = (10 + Vt) / 2.
    let devices = vec![dev(kinds::BENCH_PSU, 0, &[1, 0], vec![10.0, 1.0], vec![]), resistor(1, 1, 2, 1000.0), square(2, 2, 0, 1000.0)];
    let mut e = Engine::load(&program(3, devices, Options::default())).unwrap();
    for &(t, vt) in &[(0.25e-3, 5.0), (0.75e-3, 0.0), (1.25e-3, 5.0), (3.6e-3, 0.0)] {
        let e = at(&mut e, t);
        w0_v(&format!("node 2 at {t:e}"), e.voltages()[2], (10.0 + vt) / 2.0);
        w0_i(&format!("the PSU at {t:e}"), e.currents()[0], -(10.0 - (10.0 + vt) / 2.0) / 1000.0);
        w0_i(&format!("elementI at {t:e}"), e.currents()[2], (vt - (10.0 + vt) / 2.0) / 1000.0);
    }
}

#[test]
fn a_generator_with_a_pin_on_the_hub_rail_declines() {
    for (r_source, enabled) in [(50.0, 1.0), (0.0, 1.0), (50.0, 0.0)] {
        let mut g = square(2, 1, 0, r_source);
        g.params[sg::ENABLED] = enabled;
        let devices = vec![dev(kinds::BENCH_PSU, 0, &[1, 0], vec![10.0, 1.0], vec![]), resistor(1, 1, 0, 1000.0), g];
        let err = Engine::load(&program(2, devices, Options::default())).err().expect("a generator on the rail declines");
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY, "rSource {r_source}, enabled {enabled}: {}", err.message);
    }
}


// ---- step 2: the shapes linear between their events --------------------------------------------------------------

/// The shapes that move between their events: a 0-5 V sine at 1 kHz from 0.2 ms; the four linear ones, a 0-5 V
/// triangle and ramp at 1 kHz, a PWL 0 V -> 5 V over 1 ms then flat, and a 1 kHz pulse with 0.1 ms ramps (resolved:
/// above W1f Q5's threshold).
fn linear_shapes(r_source: f64) -> Vec<(&'static str, Device)> {
    let mut pwl = generator(0, 1, 0, sg::SHAPE_PWL, 0.0, 0.0, 1000.0, r_source);
    pwl.blob = vec![2.0, 0.0, 0.0, 1e-3, 5.0];
    let mut pulse = generator(0, 1, 0, sg::SHAPE_PULSE, 2.5, 2.5, 1000.0, r_source);
    (pulse.params[sg::TR], pulse.params[sg::TF], pulse.params[sg::PW]) = (1e-4, 1e-4, 3e-4);
    let mut sine = generator(0, 1, 0, sg::SHAPE_SINE, 2.5, 2.5, 1000.0, r_source);
    (sine.params[sg::PHASE_DEG], sine.params[sg::DELAY]) = (90.0, 0.2e-3);
    vec![
        // Holds 5 V (its t' = 0 value at 90 degrees) until 0.2 ms, then runs.
        ("sine", sine),
        ("triangle", generator(0, 1, 0, sg::SHAPE_TRIANGLE, 2.5, 2.5, 1000.0, r_source)),
        ("ramp", generator(0, 1, 0, sg::SHAPE_RAMP, 2.5, 2.5, 1000.0, r_source)),
        ("pwl", pwl),
        ("pulse", pulse),
    ]
}

/// Sample instants away from every corner and reset of the shapes above.
const LINEAR_SAMPLES: [f64; 9] = [0.03e-3, 0.17e-3, 0.26e-3, 0.44e-3, 0.61e-3, 0.83e-3, 0.97e-3, 1.33e-3, 2.71e-3];

#[test]
fn an_ideal_moving_shape_moves_its_node_along_classics_waveform_between_its_events() {
    for (name, g) in linear_shapes(0.0) {
        let (params, blob) = (g.params.clone(), g.blob.clone());
        // A divider on the node: a partition reading a ramping node is stepped and reads it at each instant.
        let mut e = Engine::load(&program(3, vec![g, resistor(1, 1, 2, 1000.0), resistor(2, 2, 0, 1000.0)], Options::default())).unwrap();
        for t in LINEAR_SAMPLES {
            let v = waveform::signal_gen_voltage(&params, &blob, t);
            let e = at(&mut e, t);
            assert!((e.voltages()[1] - v).abs() <= 1e-12, "{name}: node at {t:e}: got {}, classic {v}", e.voltages()[1]);
            tight(&format!("{name}: divider at {t:e}"), e.voltages()[2], v / 2.0);
            w0_i(&format!("{name}: elementI at {t:e}"), e.currents()[0], -v / 2000.0);
        }
    }
}

#[test]
fn a_norton_moving_shape_drives_its_load_as_vt_behind_rsource_from_either_fixed_terminal() {
    for neg_grounded in [true, false] {
        for (name, mut g) in linear_shapes(50.0) {
            let (params, blob) = (g.params.clone(), g.blob.clone());
            // pos on node 1 and neg grounded, or pos grounded and neg on node 1 (the drive inverted, G7).
            if !neg_grounded {
                g.pins = vec![pin(0), pin(1)];
            }
            let mut e = Engine::load(&program(2, vec![g, resistor(1, 1, 0, 1000.0)], Options::default())).unwrap();
            let sign = if neg_grounded { 1.0 } else { -1.0 };
            for t in LINEAR_SAMPLES {
                let vt = waveform::signal_gen_voltage(&params, &blob, t);
                let node = sign * vt * 1000.0 / 1050.0;
                let e = at(&mut e, t);
                assert!((e.voltages()[1] - node).abs() <= 1e-12 + 1e-9 * node.abs(), "{name}, neg grounded {neg_grounded}: node at {t:e}: got {}, want {node}", e.voltages()[1]);
                // Q-O67: `(Vt - (V(pos) - V(neg))) / rSource`, positive while delivering.
                let v_pd = if neg_grounded { e.voltages()[1] } else { -e.voltages()[1] };
                tight(&format!("{name}, neg grounded {neg_grounded}: elementI at {t:e}"), e.currents()[0], (vt - v_pd) / 50.0);
                // The internal node and the two parts the rewrite appends are the solver's own: the readout is the
                // program's two nodes and two devices, the record's index where the reader put it.
                assert_eq!((e.voltages().len(), e.currents().len()), (2, 2), "the readout's lengths");
            }
        }
    }
}

#[test]
fn a_norton_moving_drive_with_both_terminals_free_declines() {
    let mut g = generator(0, 1, 2, sg::SHAPE_TRIANGLE, 2.5, 2.5, 1000.0, 50.0);
    g.pins = vec![pin(1), pin(2)];
    let devices = vec![g, resistor(1, 1, 0, 1000.0), resistor(2, 2, 0, 1000.0)];
    let err = Engine::load(&program(3, devices, Options::default())).err().expect("declines");
    assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY, "{}", err.message);
}

/// A PWL from 0 V at t = 0 to 5 V at 1 ms, then flat, behind `r` into 1 kohm || 1 uF from an empty capacitor: the
/// Thevenin line `a + b t` (a = 0, b = 5000 Vth/5 per second) through `Rth` into `C` reads `b (t - tau) + b tau
/// exp(-t/tau)`, then settles from its 1 ms value toward `Vth(5)`.
fn pwl_rc(t: f64, r: f64) -> f64 {
    let (rth, k) = (r * 1000.0 / (r + 1000.0), 1000.0 / (r + 1000.0));
    let (tau, b) = (rth * 1e-6, 5000.0 * k);
    let ramp = |t: f64| b * (t - tau) + b * tau * (-t / tau).exp();
    if t <= 1e-3 {
        ramp(t)
    } else {
        5.0 * k + (ramp(1e-3) - 5.0 * k) * (-(t - 1e-3) / tau).exp()
    }
}

#[test]
fn a_pwl_line_drives_a_closed_form_rc_exactly_and_a_stepped_one_to_w0() {
    for r_source in [50.0, 0.0] {
        for force_stepped in [false, true] {
            let mut g = generator(0, 1, 0, sg::SHAPE_PWL, 0.0, 0.0, 1000.0, r_source);
            g.blob = vec![2.0, 0.0, 0.0, 1e-3, 5.0];
            // The ideal form behind a 50 ohm resistor of its own, so both forms see the same circuit.
            let (node, devices) = if r_source > 0.0 {
                (1, vec![g, resistor(1, 1, 0, 1000.0), capacitor(2, 1, 0, 1e-6)])
            } else {
                (2, vec![g, resistor(1, 1, 2, 50.0), resistor(2, 2, 0, 1000.0), capacitor(3, 2, 0, 1e-6)])
            };
            let options = Options { force_stepped, ..Options::default() };
            let mut e = Engine::load(&program(3, devices, options)).unwrap();
            for t in [20e-6, 0.1e-3, 0.5e-3, 0.9e-3, 1.0e-3 + 30e-6, 1.2e-3, 2e-3] {
                if force_stepped {
                    advance_fine(&mut e, t, 25e-9);
                    w0_v(&format!("rSource {r_source}, stepped: node at {t:e}"), e.voltages()[node], pwl_rc(t, 50.0));
                } else {
                    let e = at(&mut e, t);
                    tight(&format!("rSource {r_source}, closed form: node at {t:e}"), e.voltages()[node], pwl_rc(t, 50.0));
                }
            }
        }
    }
}

#[test]
fn a_pwl_step_of_two_knots_at_one_time_takes_the_later_value() {
    for r_source in [50.0, 0.0] {
        let mut g = generator(0, 1, 0, sg::SHAPE_PWL, 0.0, 0.0, 1000.0, r_source);
        // 0 V to 5 V over 1 ms, a vertical step to 2 V at 1 ms, 2 V to 0 V over the next 1 ms.
        g.blob = vec![4.0, 0.0, 0.0, 1e-3, 5.0, 1e-3, 2.0, 2e-3, 0.0];
        let divided = |v: f64| if r_source > 0.0 { v * 1000.0 / 1050.0 } else { v };
        let mut e = Engine::load(&program(2, vec![g, resistor(1, 1, 0, 1000.0)], Options::default())).unwrap();
        for &(t, v) in &[(0.5e-3, 2.5), (1e-3 - 1e-9, 5.0 - 5e-6), (1e-3, 2.0), (1.5e-3, 1.0), (3e-3, 0.0)] {
            let e = at(&mut e, t);
            assert!((e.voltages()[1] - divided(v)).abs() <= 1e-9, "rSource {r_source}: node at {t:e}: got {}, want {}", e.voltages()[1], divided(v));
        }
    }
}

/// A 74ls04 whose `1a` reads node 1 and whose `1y` drives node 2 into 10 kohm; the other inputs grounded.
fn inverter_on(ci: usize, vcc: usize) -> Device {
    inverter_with_vih(ci, vcc, 2.0)
}

/// [`inverter_on`] with its own VIH (0.8, VIL's, makes the input one threshold).
fn inverter_with_vih(ci: usize, vcc: usize, vih: f64) -> Device {
    let mut params = vec![0.0; kinds::ic_param::COUNT];
    params[kinds::ic_param::VCC_MIN] = 4.75;
    params[kinds::ic_param::VCC_MAX] = 5.25;
    params[kinds::ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    params[kinds::ic_param::VIL_FIXED] = 1.0;
    params[kinds::ic_param::VIL] = 0.8;
    params[kinds::ic_param::VIH_FIXED] = 1.0;
    params[kinds::ic_param::VIH] = vih;
    params[kinds::ic_param::NOMINAL] = 5.0;
    params[kinds::ic_param::IO_MAX] = 0.008;
    params[kinds::ic_param::DELAY] = 20e-9;
    let mut blob = vec![0.0];
    blob.extend((0..12).map(|_| 0.0));
    let logic = [1, 2, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8];
    let mut pins = vec![pin(vcc), pin(0)];
    pins.extend(logic.into_iter().map(pin));
    dev(kinds::IC_74LS04, ci, &[], params, blob).with_pins(pins)
}

trait WithPins {
    fn with_pins(self, pins: Vec<PinRecord>) -> Self;
}

impl WithPins for Device {
    fn with_pins(mut self, pins: Vec<PinRecord>) -> Self {
        self.pins = pins;
        self
    }
}

#[test]
fn a_reader_crosses_its_threshold_on_every_line_of_a_triangle_corner_after_corner() {
    // An ideal 0-5 V triangle at 1 kHz on the inverter's input: 10 V/ms up from 0 at t = 0, down from 5 V at
    // 0.5 ms. This TTL input reads high above VIL (0.8 V), crossed rising at 80 us + 1000 n us and falling at
    // 920 us + 1000 n us; the output follows 20 ns later. The falling line starts at a corner, line to line, where
    // a watch scheduled only when a ramp starts or ends would never be set.
    let devices = vec![
        generator(0, 1, 0, sg::SHAPE_TRIANGLE, 2.5, 2.5, 1000.0, 0.0),
        dev(kinds::VOLTAGE_SOURCE, 1, &[3, 0], vec![5.0], vec![]),
        inverter_on(2, 3),
        resistor(3, 2, 0, 10_000.0),
    ];
    let mut p = program(9, devices, Options::default());
    p.digital_slot_count = 6;
    let mut e = Engine::load(&p).unwrap();
    let mut flips = Vec::new();
    let mut was = e.voltages()[2] > 2.5;
    assert!(was, "the input starts low");
    for k in 1..=2000u32 {
        let high = at(&mut e, f64::from(k) * 1e-6).voltages()[2] > 2.5;
        if high != was {
            flips.push(k);
            was = high;
        }
    }
    assert_eq!(flips, [81, 921, 1081, 1921], "the microsecond each flip is first seen");
}

#[test]
fn noise_holds_each_buckets_draw_from_its_boundary_one_ps_early() {
    for r_source in [50.0, 0.0] {
        let mut g = generator(0, 1, 0, sg::SHAPE_NOISE, 1.0, 2.0, 1000.0, r_source);
        g.params[sg::SEED] = 7.0;
        g.params[sg::DELAY] = 2e-4;
        let divided = |v: f64| if r_source > 0.0 { v * 1000.0 / 1050.0 } else { v };
        let level = |bucket: f64| 2.0 + waveform::noise_unit(7.0, bucket);
        let mut e = Engine::load(&program(2, vec![g, resistor(1, 1, 0, 1000.0)], Options::default())).unwrap();
        // Before the delay the output holds bucket 0's draw.
        tight("at load", e.voltages()[1], divided(level(0.0)));
        tight("in the delay", at(&mut e, 1e-4).voltages()[1], divided(level(0.0)));
        for k in 0..30 {
            let boundary = (2e-4 + (k + 1) as f64 * 1e-4) - 1e-12;
            let e = at(&mut e, boundary - 1e-13);
            tight(&format!("rSource {r_source}: just before boundary {k}"), e.voltages()[1], divided(level(k as f64)));
            let e = at(e, boundary);
            tight(&format!("rSource {r_source}: at boundary {k}"), e.voltages()[1], divided(level((k + 1) as f64)));
        }
    }
}

#[test]
fn a_pulse_under_the_step_threshold_steps_at_its_ramp_midpoints() {
    // tr = tf = 1 ns at 1 kHz with pw 0.2 ms: both ramps are midpoint steps
    // (W1f Q5), so the level is v2 on [tr/2, tr + pw + tf/2) and v1 otherwise.
    for r_source in [50.0, 0.0] {
        let mut g = generator(0, 1, 0, sg::SHAPE_PULSE, 2.0, 1.0, 1000.0, r_source);
        (g.params[sg::TR], g.params[sg::TF], g.params[sg::PW]) = (1e-9, 1e-9, 2e-4);
        let divided = |v: f64| if r_source > 0.0 { v * 1000.0 / 1050.0 } else { v };
        let mut e = Engine::load(&program(2, vec![g, resistor(1, 1, 0, 1000.0)], Options::default())).unwrap();
        for &(t, v) in &[(0.1e-3, 3.0), (0.2e-3 + 1.4e-9, 3.0), (0.2e-3 + 1.6e-9, -1.0), (0.5e-3, -1.0), (1e-3 + 0.4e-9, -1.0), (1e-3 + 0.6e-9, 3.0), (1.1e-3, 3.0), (1.3e-3, -1.0)] {
            let e = at(&mut e, t);
            tight(&format!("rSource {r_source}: node at {t:e}"), e.voltages()[1], divided(v));
        }
    }
}

#[test]
fn dc_holds_its_offset_with_no_event() {
    for r_source in [50.0, 0.0] {
        let g = generator(0, 1, 0, sg::SHAPE_DC, 9.0, 3.3, 1000.0, r_source);
        let divided = if r_source > 0.0 { 3.3 * 1000.0 / 1050.0 } else { 3.3 };
        let mut e = Engine::load(&program(2, vec![g, resistor(1, 1, 0, 1000.0)], Options::default())).unwrap();
        assert_eq!(e.next_event_time(), f64::INFINITY, "rSource {r_source}: dc schedules nothing");
        tight("at 1 s", at(&mut e, 1.0).voltages()[1], divided);
    }
}

#[test]
fn a_norton_drive_across_two_fixed_nodes_moves_the_supplys_current_at_its_edges() {
    // A 5 V source on node 1 and the generator (100 ohm) from node 1 to
    // ground: both pins fixed, so only the KCL walk sees it. The generator
    // sources (Vt - 5)/100 into node 1, the supply the rest.
    let devices = vec![dev(kinds::VOLTAGE_SOURCE, 0, &[1, 0], vec![5.0], vec![]), generator(1, 1, 0, sg::SHAPE_SQUARE, 5.0, 5.0, 1000.0, 100.0)];
    let mut e = Engine::load(&program(2, devices, Options::default())).unwrap();
    for &(t, vt) in &[(0.25e-3, 10.0), (0.75e-3, 0.0), (1.25e-3, 10.0)] {
        let e = at(&mut e, t);
        tight(&format!("elementI at {t:e}"), e.currents()[1], (vt - 5.0) / 100.0);
        tight(&format!("the supply at {t:e}"), e.currents()[0], (vt - 5.0) / 100.0);
    }
}

#[test]
fn two_stacked_clocks_key_each_static_partition_on_the_level_of_the_clock_it_reads() {
    // Clock A (device 0, 1 Hz) on node 2 above node 1; clock B (device 1,
    // 1 kHz) fixes node 1 from ground, so the forest resolves B before A. A
    // divider on node 1 alone reads B: 2.5 V while B is high, 0 V low. Keyed
    // on A's level (A is device 0, B the forest's first source), the divider
    // served B's high solve again at B's falling edge.
    let clock = |ci: usize, pos: usize, neg: usize, freq: f64, slot: f64| dev(kinds::CLOCK, ci, &[pos, neg], vec![freq, 0.5, 5.0, 0.0, 0.0], vec![slot]);
    let devices = vec![clock(0, 2, 1, 1.0, 0.0), clock(1, 1, 0, 1000.0, 1.0), resistor(2, 1, 3, 1000.0), resistor(3, 3, 0, 1000.0), resistor(4, 2, 0, 1000.0)];
    let mut p = program(4, devices, Options::default());
    p.digital_slot_count = 2;
    let mut e = Engine::load(&p).unwrap();
    for &(t, vb) in &[(0.25e-3, 5.0), (0.75e-3, 0.0), (1.25e-3, 5.0), (1.75e-3, 0.0)] {
        let e = at(&mut e, t);
        tight(&format!("divider at {t:e}"), e.voltages()[3], vb / 2.0);
        tight(&format!("clock A's node at {t:e}"), e.voltages()[2], vb + 5.0);
    }
}

// ---- step 2: the sine ---------------------------------------------------------------------------------------------

#[test]
fn a_sine_drives_a_stepped_rc_along_its_driven_response() {
    // 2.5 V at 200 Hz about 0 into 1 kohm and 1 uF (tau = 1 ms) from rest: v = A/sqrt(1 + (w tau)^2) (sin(w t -
    // phi) + sin(phi) exp(-t/tau)), phi = atan(w tau). A partition that reads a sine is stepped; horizons of 0.25
    // us keep the steps short (backward Euler's own error is then about h/2 dv/dt, 5e-5 V), so what is tested is
    // the boundary read at each step's instant.
    for r_source in [0.0, 1000.0] {
        let g = generator(0, 1, 0, sg::SHAPE_SINE, 2.5, 0.0, 200.0, r_source);
        let devices = if r_source > 0.0 { vec![g, capacitor(1, 1, 0, 1e-6)] } else { vec![g, resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0, 1e-6)] };
        let node = if r_source > 0.0 { 1 } else { 2 };
        let mut e = Engine::load(&program(3, devices, Options::default())).unwrap();
        let (w, tau) = (2.0 * std::f64::consts::PI * 200.0, 1e-3);
        let phi = (w * tau).atan();
        let v = |t: f64| 2.5 / (1.0 + (w * tau).powi(2)).sqrt() * ((w * t - phi).sin() + phi.sin() * (-t / tau).exp());
        for t in [0.3e-3, 1.1e-3, 2.4e-3, 3.7e-3] {
            advance_fine(&mut e, t, 0.25e-6);
            w0_v(&format!("rSource {r_source}: node at {t:e}"), e.voltages()[node], v(t));
        }
    }
}

/// An ideal sine of amplitude 0.4 V at 1 kHz on the 74ls04's `1a`, peaking at `peak`, the output's slot traced.
/// The input reads high above VIL (0.8 V, a crossing lands 1e-9 V past it), so a peak above it is a short hump
/// over the threshold each period: two crossings close together, inside any step.
fn hump(peak: f64) -> Engine {
    let devices = vec![
        generator(0, 1, 0, sg::SHAPE_SINE, 0.4, peak - 0.4, 1000.0, 0.0),
        dev(kinds::VOLTAGE_SOURCE, 1, &[3, 0], vec![5.0], vec![]),
        inverter_on(2, 3),
        resistor(3, 2, 0, 10_000.0),
    ];
    let mut p = program(9, devices, Options::default());
    p.digital_slot_count = 6;
    let mut e = Engine::load(&p).unwrap();
    e.trace_enable(true);
    e.trace_clear();
    e
}

/// The times slot 0 (`1y`) changes in `e`'s trace.
fn flips(e: &Engine) -> Vec<f64> {
    e.trace().chunks_exact(3).filter(|c| c[1] == 0.0).map(|c| c[0]).collect()
}

#[test]
fn every_short_hump_of_a_sine_over_a_threshold_is_crossed_twice_inside_one_advance() {
    // Peaks 0.1 mV and 1 uV over VIL: humps of 14 us and 1.4 us, both far inside classic's 25 us step at 1 kHz,
    // read in one advance of ten periods. Each hump is two crossings, so the output pulses low once per period, at
    // the times the sine's own roots give (+ 20 ns of gate delay).
    for over in [1e-4, 1e-6] {
        let mut e = hump(0.8 + over);
        e.advance(10e-3).unwrap();
        let got = flips(&e);
        assert_eq!(got.len(), 20, "over {over:e}: two flips a period, got {got:?}");
        // sin x = s for v = 0.8 + 1e-9: up at asin(s), down at pi - asin(s), x = 2 pi 1000 t.
        let s = ((0.8 + 1e-9) - (0.8 + over - 0.4)) / 0.4;
        let w = 2.0 * std::f64::consts::PI * 1000.0;
        for (n, pair) in got.chunks_exact(2).enumerate() {
            let (up, down) = ((s.asin() + 2.0 * std::f64::consts::PI * n as f64) / w, (std::f64::consts::PI - s.asin() + 2.0 * std::f64::consts::PI * n as f64) / w);
            assert!((pair[0] - (up + 20e-9)).abs() < 1e-9, "over {over:e}, period {n}: low at {:e}, the root at {up:e}", pair[0]);
            assert!((pair[1] - (down + 20e-9)).abs() < 1e-9, "over {over:e}, period {n}: high at {:e}, the root at {down:e}", pair[1]);
            assert!(pair[1] > pair[0]);
        }
    }
}

#[test]
fn a_sine_whose_peak_stays_short_of_the_crossing_wakes_no_reader() {
    // A peak 0.5 nV over VIL never reaches the crossing 1 nV past it: no event, so the reader stays as it was.
    // Short of VIL by 1 uV: nothing to cross at all. A tangent touch of the crossing itself is the double root,
    // where the reader reads classic's value as the rounding leaves it.
    for peak in [0.8 + 0.5e-9, 0.8 - 1e-6] {
        let mut e = hump(peak);
        e.advance(10e-3).unwrap();
        assert!(flips(&e).is_empty(), "peak {peak}: {:?}", flips(&e));
    }
}

#[test]
fn the_sine_crossing_roots_are_every_solution_in_the_span() {
    // A 1 kHz sine of amplitude 2 about 1 V, phase 30 degrees, from 0.25 ms: v = 1 + 2 sin(2 pi 1000 (t - 0.25 ms)
    // + pi/6), five periods to 5.25 ms.
    let mut g = generator(0, 1, 0, sg::SHAPE_SINE, 2.0, 1.0, 1000.0, 0.0);
    (g.params[sg::PHASE_DEG], g.params[sg::DELAY]) = (30.0, 0.25e-3);
    let p = &g.params;
    let v = |t: f64| waveform::signal_gen_voltage(p, &g.blob, t);
    for (target, rising) in [(2.3, true), (2.3, false), (-0.5, true), (-0.5, false), (0.999, true)] {
        let roots: Vec<f64> = waveform::sine_crossings(p, 0.0, 5.25e-3, target, rising).into_iter().map(|(t, _)| t).collect();
        // A dense scan of strict sign changes on the same side finds the same count.
        let mut scan = 0;
        let mut prev = v(0.25e-3) - target;
        for k in 1..=500_000 {
            let now = v(0.25e-3 + k as f64 * 1e-8) - target;
            if (rising && prev < 0.0 && now >= 0.0) || (!rising && prev > 0.0 && now <= 0.0) {
                scan += 1;
            }
            prev = now;
        }
        assert_eq!(roots.len(), 5, "{target} rising {rising}: {roots:?}");
        assert_eq!(roots.len(), scan, "{target} rising {rising}");
        for &t in &roots {
            assert!((v(t) - target).abs() < 1e-9, "{target} rising {rising}: v({t:e}) = {}", v(t));
            assert_eq!(v(t + 1e-9) > v(t - 1e-9), rising, "{target}: slope at {t:e}");
        }
    }
    // The extremes are tangent touches: the double root, once a period on each side's list, on the extreme.
    for (target, rising) in [(3.0, true), (3.0, false), (-1.0, true), (-1.0, false)] {
        let roots: Vec<f64> = waveform::sine_crossings(p, 0.0, 5.25e-3, target, rising).into_iter().map(|(t, _)| t).collect();
        assert_eq!(roots.len(), 5, "{target} rising {rising}: {roots:?}");
        for &t in &roots {
            assert!((v(t) - target).abs() < 1e-9, "{target}: v({t:e}) = {}", v(t));
        }
    }
    // Nothing for a level the sine never reaches, nor before the delay.
    assert!(waveform::sine_crossings(p, 0.0, 5e-3, 3.5, true).is_empty());
    assert!(waveform::sine_crossings(p, 0.0, 0.25e-3, 2.3, true).is_empty());
}

#[test]
fn a_sine_stacked_with_another_waveform_or_supplying_an_ic_declines() {
    let clock = dev(kinds::CLOCK, 1, &[2, 0], vec![100.0, 0.5, 1.0, 0.0, 0.0], vec![0.0]);
    let mut p = program(3, vec![generator(0, 1, 2, sg::SHAPE_SINE, 1.0, 0.0, 1000.0, 0.0), clock, resistor(2, 1, 0, 1000.0)], Options::default());
    p.digital_slot_count = 1;
    let err = Engine::load(&p).err().expect("a sine on a clock declines");
    assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY, "{}", err.message);
    let mut p = program(9, vec![generator(0, 3, 0, sg::SHAPE_SINE, 0.5, 5.0, 1000.0, 0.0), inverter_on(1, 3), resistor(2, 2, 0, 10_000.0)], Options::default());
    p.digital_slot_count = 6;
    let err = Engine::load(&p).err().expect("an IC supplied from a sine declines");
    assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY, "{}", err.message);
}

#[test]
fn a_crossing_late_in_a_run_lands_where_classics_value_is_past_the_threshold() {
    // A 1 MHz sine of 2.5 V about 2.5 V from 50 s: there an ulp of time (7e-15 s) is about 1e-7 V of the sine, a
    // hundred times the 1 nV margin a crossing lands past its threshold by, so a root as `asin` and the time
    // arithmetic round it can fall short (half of them do, by up to 5e-8 V). Each is moved on until classic's
    // value there is past the threshold, and the output flips twice every period, on time.
    let mut g = generator(0, 1, 0, sg::SHAPE_SINE, 2.5, 2.5, 1e6, 0.0);
    g.params[sg::DELAY] = 50.0;
    let devices = vec![g, dev(kinds::VOLTAGE_SOURCE, 1, &[3, 0], vec![5.0], vec![]), inverter_on(2, 3), resistor(3, 2, 0, 10_000.0)];
    let mut p = program(9, devices, Options::default());
    p.digital_slot_count = 6;
    let mut e = Engine::load(&p).unwrap();
    e.advance(50.0).unwrap();
    e.trace_enable(true);
    e.trace_clear();
    e.advance(50.0 + 20e-6).unwrap();
    let got = flips(&e);
    assert_eq!(got.len(), 40, "{got:?}");
    // Each flip a period after the one two before it: a root that fell short is read only at some later event,
    // tens of ns late (the run without the move read 1.639, 2.020 and 2.639 us where 1.552, 1.988, 2.552 are due).
    for k in 2..got.len() {
        assert!((got[k] - got[k - 2] - 1e-6).abs() < 1e-12, "flip {k} at {:e}, the one two before at {:e}", got[k] - 50.0, got[k - 2] - 50.0);
    }
}

// ---- H3bV follow-ups ----------------------------------------------------------------------------------------------

/// A sine on node `pin` of a one-threshold 74ls04 (VIL = VIH = 0.8 V) from `start`, the output's slot traced.
fn one_threshold(devices: Vec<Device>, input: usize, start: f64) -> Engine {
    let mut inv = inverter_with_vih(20, 3, 0.8);
    inv.pins[2] = pin(input);
    let mut all = devices;
    all.extend([dev(kinds::VOLTAGE_SOURCE, 21, &[3, 0], vec![5.0], vec![]), inv, resistor(22, 2, 0, 10_000.0)]);
    // Nodes 1-8 are the inverter's (1a, 1y, its vcc, the other gates' outputs); 9 is free for a stack.
    let mut p = program(10, all, Options { start_time: start, ..Options::default() });
    p.digital_slot_count = 6;
    let mut e = Engine::load(&p).unwrap();
    e.trace_enable(true);
    e.trace_clear();
    e
}

#[test]
fn a_crossing_whose_root_sits_ulps_before_a_span_end_keeps_its_event_past_it() {
    // A 1 MHz sine of 0.3 V from 1000 s (START_TIME and the delay), phase 30 degrees, its offset set so that the
    // rising crossing of VIL falls right at each period boundary, where each watch span ends: the root as asin
    // rounds it sits a few ulps before the span's end, and classic's own value gets past 0.8 V only after it. The
    // event lands past the span's end, and the next span lists only the roots after it, so it is kept where it lands:
    // 22 flips over 12 periods (one fall and one rise a period, the first period's rise at its start already in
    // force). Bounded by the span instead, every one was lost.
    let mut g = generator(0, 1, 0, sg::SHAPE_SINE, 0.3, 0.6499999581410719, 1e6, 0.0);
    (g.params[sg::PHASE_DEG], g.params[sg::DELAY]) = (30.0, 1000.0);
    let mut e = one_threshold(vec![g], 1, 1000.0);
    e.advance(1000.0 + 12e-6).unwrap();
    assert_eq!(flips(&e).len(), 22, "{:?}", flips(&e).iter().map(|t| t - 1000.0).collect::<Vec<_>>());
}

/// The sine `1 + 0.5 sin(2 pi 1000 t)` from 0 as the node a one-threshold input reads, built three ways, and the
/// times it crosses 0.8 V + 1 nV rising (falling: - 1 nV) over three periods, as `asin` gives them.
fn read_three_ways() -> (Vec<(&'static str, Engine)>, Vec<f64>) {
    let w = 2.0 * std::f64::consts::PI * 1000.0;
    let mut want = Vec::new();
    for n in 0..3 {
        let base = 2.0 * std::f64::consts::PI * n as f64;
        // Falls through 0.8 - 1 nV at pi - asin(s_down), rises through 0.8 + 1 nV at 2 pi + asin(s_up).
        let (s_down, s_up): (f64, f64) = (((0.8 - 1e-9) - 1.0) / 0.5, ((0.8 + 1e-9) - 1.0) / 0.5);
        want.push((base + std::f64::consts::PI - s_down.asin()) / w);
        want.push((base + 2.0 * std::f64::consts::PI + s_up.asin()) / w);
    }
    // Plain: pos on the node, neg grounded (sign +1, c = 0).
    let plain = generator(0, 1, 0, sg::SHAPE_SINE, 0.5, 1.0, 1000.0, 0.0);
    // Inverted: pos grounded and neg on the node (sign -1): the node is -v, so v = -1 - 0.5 sin, phase 180.
    let mut inverted = generator(0, 0, 1, sg::SHAPE_SINE, 0.5, -1.0, 1000.0, 0.0);
    inverted.params[sg::PHASE_DEG] = 180.0;
    // Stacked on a 1 V source (c = 1): v = 0.5 sin, its neg on the source's node 9.
    let stacked = generator(0, 1, 9, sg::SHAPE_SINE, 0.5, 0.0, 1000.0, 0.0);
    let engines = vec![
        ("plain", one_threshold(vec![plain], 1, 0.0)),
        ("pos on ground, neg read", one_threshold(vec![inverted], 1, 0.0)),
        ("stacked on 1 V", one_threshold(vec![stacked, dev(kinds::VOLTAGE_SOURCE, 1, &[9, 0], vec![1.0], vec![])], 1, 0.0)),
    ];
    (engines, want)
}

#[test]
fn a_sine_read_through_its_negative_terminal_or_on_a_constant_crosses_where_its_node_does() {
    let (engines, want) = read_three_ways();
    for (name, mut e) in engines {
        e.advance(3e-3).unwrap();
        let got = flips(&e);
        assert_eq!(got.len(), want.len(), "{name}: {got:?}");
        for (g, w) in got.iter().zip(&want) {
            assert!((g - (w + 20e-9)).abs() < 1e-9, "{name}: flip at {g:e}, the root at {w:e} (+ 20 ns)");
        }
    }
}

#[test]
fn stacked_ideal_squares_key_each_static_divider_on_its_own_stack() {
    // Three ideal square generators stacked (A on B on C, C from ground), placed in every device order, each node
    // read by a static divider: a divider keys its cache on the levels of the sources under its node, so after any
    // edge it reads its own stack, never an entry solved at another level (the level bits of an ideal generator).
    let freqs = [10_000.0, 1_000.0, 3_000.0];
    let level = |which: usize| 5.0 + which as f64;
    for perm in [[0usize, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]] {
        let make = |which: usize, ci: usize| {
            let (pos, neg) = [(1, 0), (2, 1), (3, 2)][which];
            generator(ci, pos, neg, sg::SHAPE_SQUARE, level(which) / 2.0, level(which) / 2.0, freqs[which], 0.0)
        };
        let mut devices: Vec<Device> = perm.iter().enumerate().map(|(ci, &w)| make(w, ci)).collect();
        for (k, n) in [1usize, 2, 3].into_iter().enumerate() {
            devices.push(resistor(10 + 2 * k, n, 4 + k, 1000.0));
            devices.push(resistor(11 + 2 * k, 4 + k, 0, 1000.0));
        }
        let mut e = Engine::load(&program(7, devices, Options::default())).unwrap();
        let high = |which: usize, t: f64| {
            let phase = (t * freqs[which]).fract();
            (0.1..0.4).contains(&phase).then_some(true).or_else(|| (0.6..0.9).contains(&phase).then_some(false))
        };
        let mut t = 0.0;
        for k in 0..400 {
            t += 7.3e-6 + (k % 5) as f64 * 1.1e-6;
            e.advance(t).unwrap();
            let Some(hs) = (0..3).map(|w| high(w, t)).collect::<Option<Vec<bool>>>() else { continue };
            let lv = |w: usize| if hs[w] { level(w) } else { 0.0 };
            let stacks = [lv(0), lv(0) + lv(1), lv(0) + lv(1) + lv(2)];
            for (k2, stack) in stacks.into_iter().enumerate() {
                tight(&format!("order {perm:?}, t {t:e}: divider {k2}"), e.voltages()[4 + k2], stack / 2.0);
            }
        }
    }
}

// ---- P0-SG: signal_gen under the 1.0.0 line's carry, snapshot and charge ---

/// Every shape class and form into an RC (closed form, or stepped under
/// `force_stepped`), each run to `t1` straight and reloaded at `t0` with W2c's
/// carry: the generator's state is a function of time (its schedule), so the
/// reload carries only the capacitor, and both runs read within W0 at `t1`
/// (the reload re-solves at `t0`, so not bit for bit). A W2a snapshot taken at
/// `t0` and restored runs on bit for bit.
#[test]
fn a_generator_reloads_with_its_capacitor_carried_and_restores_bit_for_bit() {
    let (t0, t1) = (0.73e-3, 1.9e-3);
    for shape in [sg::SHAPE_SQUARE, sg::SHAPE_SINE, sg::SHAPE_TRIANGLE, sg::SHAPE_RAMP, sg::SHAPE_PULSE] {
        for r_source in [0.0, 50.0] {
            for stepped in [false, true] {
                let options = Options { force_stepped: stepped, ..Options::default() };
                let p = program(3, vec![generator(0, 1, 0, shape, 2.0, 2.5, 1000.0, r_source), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0, 1e-6)], options);
                let what = format!("shape {shape} rSource {r_source} stepped {stepped}");
                let mut straight = Engine::load(&p).unwrap();
                straight.advance(t0).unwrap();
                let snap = straight.snapshot();
                let mut q = p.clone();
                q.carry = straight.carry_take().unwrap().to_vec();
                q.options.start_time = t0;
                let mut reloaded = Engine::load(&q).unwrap();
                straight.advance(t1).unwrap();
                reloaded.advance(t1).unwrap();
                w0_v(&format!("{what}: the capacitor node"), reloaded.voltages()[2], straight.voltages()[2]);
                w0_i(&format!("{what}: the generator's current"), reloaded.currents()[0], straight.currents()[0]);
                let mut restored = Engine::load(&p).unwrap();
                restored.restore(&snap);
                restored.advance(t1).unwrap();
                assert_eq!(restored.voltages(), straight.voltages(), "{what}: restored");
                assert_eq!(restored.currents(), straight.currents(), "{what}: restored");
            }
        }
    }
}

/// W2b's charge of a generator and its load, armed from load: the capacitor's
/// is `C dV` and the generator's the KCL of what leaves its node (ideal form)
/// or its own current integrated by its partition (Norton), so with nothing
/// else on the node it is the resistor's, with Q-O67's sign by form.
#[test]
fn a_generators_charge_balances_its_loads() {
    for shape in [sg::SHAPE_SQUARE, sg::SHAPE_SINE, sg::SHAPE_TRIANGLE] {
        for r_source in [0.0, 50.0] {
            let p = program(3, vec![generator(0, 1, 0, shape, 2.0, 2.5, 1000.0, r_source), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0, 1e-6)], Options::default());
            let mut e = Engine::load(&p).unwrap();
            let v0 = e.voltages()[2];
            e.charge_arm(&[0, 1, 2]).unwrap();
            e.advance(2.2e-3).unwrap();
            let q = e.take_charge().to_vec();
            let (qg, qr, qc) = (q[2], q[3], q[4]);
            let what = format!("shape {shape} rSource {r_source}");
            let span = 2.2e-3;
            let bound = |x: f64| 1e-9 * span + 1e-3 * x.abs();
            assert!((qc - 1e-6 * (e.voltages()[2] - v0)).abs() <= bound(qc), "{what}: C dV {qc:e}");
            assert!((qr - qc).abs() <= bound(qr), "{what}: the resistor feeds the capacitor alone: {qr:e} vs {qc:e}");
            // Q-O67, classic's own: the ideal form publishes the branch
            // current pos -> neg through it (negative while delivering, as a
            // voltage_source), the Norton form the current out of `pos`.
            let sign = if r_source > 0.0 { -1.0 } else { 1.0 };
            assert!((qg + sign * qr).abs() <= bound(qr), "{what}: the generator delivers what the resistor carries: {qg:e} vs {qr:e}");
        }
    }
}
