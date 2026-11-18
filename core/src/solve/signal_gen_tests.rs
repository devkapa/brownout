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

#[test]
fn a_shape_that_moves_between_its_events_declines_until_step_2() {
    let mut pwl = generator(0, 1, 0, sg::SHAPE_PWL, 0.0, 0.0, 1000.0, 50.0);
    pwl.blob = vec![2.0, 0.0, 0.0, 1e-3, 5.0];
    let mut slow_pulse = generator(0, 1, 0, sg::SHAPE_PULSE, 2.5, 2.5, 1000.0, 50.0);
    slow_pulse.params[sg::TR] = 1e-4;
    for g in [
        generator(0, 1, 0, sg::SHAPE_SINE, 2.5, 2.5, 1000.0, 50.0),
        generator(0, 1, 0, sg::SHAPE_TRIANGLE, 2.5, 2.5, 1000.0, 0.0),
        generator(0, 1, 0, sg::SHAPE_RAMP, 2.5, 2.5, 1000.0, 50.0),
        pwl,
        slow_pulse,
    ] {
        let shape = g.params[sg::WAVEFORM];
        let err = Engine::load(&program(2, vec![g, resistor(1, 1, 0, 1000.0)], Options::default())).err().expect("declines");
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY, "shape {shape}: {}", err.message);
    }
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
