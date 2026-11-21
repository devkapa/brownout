//! W1h H1b: the variable resistive kinds and the current source through every
//! partition kind (static, closed form, stepped, a hub partition on a bench
//! PSU rail, and exact mode), driven through a real `Engine`.
//!
//! Every expected number is the spec's formula worked by hand for the circuit
//! (spec 7.6, 7.7, 7.9, 8.13, 10.10): nodal analysis over the circuit's
//! conductances, the node shunt included (`GS`, README W0 point 2: it moves
//! these kilohm circuits by nanovolts), never core output. A dynamic partition is read at a steady state
//! many time constants out, where its fixed point is the DC solution; the
//! current source's ramp into a capacitor is exact for every integrator.

use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};
use crate::solve::static_part::NODE_RSHUNT_G as GS;

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

fn pot(ci: usize, cw: usize, wiper: usize, ccw: usize, r_cw: f64, r_ccw: f64) -> Device {
    dev(kinds::POTENTIOMETER, ci, &[cw, wiper, ccw], vec![r_cw, r_ccw, 0.0], vec![])
}

fn current_source(ci: usize, pos: usize, neg: usize, i: f64, r_parallel: f64) -> Device {
    dev(kinds::CURRENT_SOURCE, ci, &[pos, neg], vec![i, r_parallel], vec![])
}

/// The partition kinds a case is run through. The supply on node 1 is device
/// 0: an ideal 5 V source, or a 5 V bench PSU (1 A limit, so it stays in CV)
/// whose rail makes every partition on node 1 a hub partition.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
    Static,
    ClosedForm,
    Stepped,
    Hub,
    HubCap,
    Exact,
}

const MODES: [Mode; 6] = [Mode::Static, Mode::ClosedForm, Mode::Stepped, Mode::Hub, Mode::HubCap, Mode::Exact];

/// A capacitor this large on the probe node makes its partition dynamic and
/// settles inside the run: every case's Thevenin resistance there is at most
/// about 10 kohm, so tau <= 1 ms against a 100 ms run.
const PROBE_C: f64 = 1e-7;
const RUN_S: f64 = 0.1;

/// `devices` (none of which is device 0) behind the mode's supply, with the
/// probe capacitor on `probe` for the dynamic modes.
fn build(mode: Mode, node_count: usize, probe: usize, devices: Vec<Device>, channels: usize) -> Program {
    let supply = match mode {
        Mode::Hub | Mode::HubCap | Mode::Exact => dev(kinds::BENCH_PSU, 0, &[1, 0], vec![5.0, 1.0], vec![]),
        _ => dev(kinds::VOLTAGE_SOURCE, 0, &[1, 0], vec![5.0], vec![]),
    };
    let mut all = vec![supply];
    all.extend(devices);
    if matches!(mode, Mode::ClosedForm | Mode::Stepped | Mode::HubCap | Mode::Exact) {
        all.push(capacitor(all.len(), probe, 0, PROBE_C));
    }
    let options = Options { force_stepped: mode == Mode::Stepped, no_rail_shortcut: mode == Mode::Exact, ..Options::default() };
    Program { node_count, options, digital_slot_count: 0, channel_slot_count: channels, devices: all, carry: Vec::new() }
}

fn run(p: &Program) -> Engine {
    let mut e = Engine::load(p).unwrap_or_else(|err| panic!("load: {err:?}"));
    e.advance(RUN_S).unwrap_or_else(|err| panic!("advance: {err:?}"));
    e
}

/// Within `abs` plus 1e-9 relative: tight enough that a dropped stamp, a lost
/// injection or the wrong published contact is off by orders of magnitude.
#[track_caller]
fn close(what: &str, mode: Mode, got: f64, want: f64, abs: f64) {
    assert!((got - want).abs() <= abs + 1e-9 * want.abs(), "{what} ({mode:?}): got {got:e}, want {want:e}");
}

/// The dynamic solvers' steady state sits within the stepped error control of
/// the fixed point, not bit on it.
fn v_tol(mode: Mode) -> f64 {
    if mode == Mode::Static || mode == Mode::Hub {
        1e-9
    } else {
        1e-6
    }
}

// ---- the guard (H1aV note 3) ---------------------------------------------

/// Every kind name in `CORE_IMPLEMENTED_KINDS` (src/core/kinds.ts), read from
/// the TS source itself so a kind added there without a solver arm fails here.
fn ts_implemented_kinds() -> Vec<String> {
    let src = include_str!("../../../src/core/kinds.ts");
    let start = src.find("export const CORE_IMPLEMENTED_KINDS").expect("kinds.ts declares CORE_IMPLEMENTED_KINDS");
    let body = &src[start..];
    let body = &body[body.find("new Set([").expect("a Set literal") + "new Set([".len()..];
    let body = &body[..body.find("]);").expect("the Set literal ends")];
    body.lines()
        .map(|l| l.trim())
        .filter(|l| l.starts_with('"'))
        .map(|l| l.trim_end_matches(',').trim_matches('"').to_string())
        .collect()
}

#[test]
fn every_implemented_kind_has_a_solver_placement() {
    // The H1b kinds are routed whether or not they are in the TS set (all
    // six are since H1c gave the pot, trimmer and array their overloads).
    let h1b = ["potentiometer", "trimmer", "resistor_array", "ldr", "push_dpdt", "current_source"];
    let mut names = ts_implemented_kinds();
    assert!(names.len() > 40 && names.iter().any(|n| n == "resistor"), "the parse found the set: {names:?}");
    names.extend(h1b.iter().map(|s| s.to_string()));
    for name in &names {
        // `ground` compiles to no record; a bead is a `resistor` record and a
        // crystal `capacitor` and `inductor` records around an internal node
        // (R56 3).
        let codes = match name.as_str() {
            "ground" => continue,
            "ferrite_bead" => vec![kinds::RESISTOR],
            "crystal" => vec![kinds::CAPACITOR, kinds::INDUCTOR],
            _ => vec![(0..=255u32).find(|&c| kinds::shape(c).is_some_and(|s| s.name == name)).unwrap_or_else(|| panic!("{name}: no Rust kind code"))],
        };
        for code in codes {
            assert!(super::placement(code).is_some(), "{name} (code {code}) reaches no arm of the solver build");
        }
    }
    // The kinds no item has routed yet stay out, so this test can fail.
    for code in [kinds::THERMISTOR] {
        assert_eq!(super::placement(code), None, "code {code}");
    }
    // W1h H3b: a signal_gen's form picks its placement per device.
    let mut g = crate::program::Device { kind: kinds::SIGNAL_GEN, component_index: 0, pins: vec![], params: vec![0.0; kinds::signal_gen_param::COUNT], blob: vec![0.0] };
    let mut form = |enabled: f64, r_source: f64| {
        (g.params[kinds::signal_gen_param::ENABLED], g.params[kinds::signal_gen_param::R_SOURCE]) = (enabled, r_source);
        super::placement_of(&g)
    };
    assert_eq!(form(0.0, 50.0), None, "disabled");
    assert_eq!(form(1.0, 0.0), Some(super::Placement::Source), "ideal");
    assert_eq!(form(1.0, -1.0), Some(super::Placement::Source), "a negative rSource is ideal");
    assert_eq!(form(1.0, 50.0), Some(super::Placement::Linear(crate::devices::Linear::Live)), "Norton");
}

// ---- potentiometer / trimmer (spec 7.7) ------------------------------------

/// 5 V on cw (node 1), ccw on ground, the wiper (node 2) loaded by 10 kohm.
fn pot_divider(mode: Mode, kind: u32, r_cw: f64, r_ccw: f64) -> Program {
    let mut p = pot(1, 1, 2, 0, r_cw, r_ccw);
    p.kind = kind;
    build(mode, 3, 2, vec![p, resistor(2, 2, 0, 10_000.0)], 0)
}

fn pot_wiper(r_cw: f64, r_ccw: f64) -> f64 {
    let g = 1.0 / r_cw + 1.0 / r_ccw + 1.0 / 10_000.0 + GS;
    5.0 / r_cw / g
}

#[test]
fn a_pot_divides_its_supply_and_publishes_its_cw_to_wiper_current_in_every_partition_kind() {
    for kind in [kinds::POTENTIOMETER, kinds::TRIMMER] {
        for mode in MODES {
            let e = run(&pot_divider(mode, kind, 3000.0, 7000.0));
            let v = pot_wiper(3000.0, 7000.0);
            close("wiper", mode, e.voltages()[2], v, v_tol(mode));
            // elementI is cw -> wiper only (spec 7.7), never the track current.
            close("pot elementI", mode, e.currents()[1], (5.0 - v) / 3000.0, v_tol(mode) / 1000.0);
            close("load", mode, e.currents()[2], v / 10_000.0, v_tol(mode) / 1000.0);
            // The supply delivers the whole track and load current, and node
            // 1's own shunt.
            close("supply", mode, e.currents()[0], -((5.0 - v) / 3000.0 + 5.0 * GS), v_tol(mode) / 1000.0);
        }
    }
}

#[test]
fn a_pot_knob_move_rewrites_both_segments_in_one_batch() {
    for mode in MODES {
        let mut e = run(&pot_divider(mode, kinds::POTENTIOMETER, 3000.0, 7000.0));
        e.set_params(&[(1, 0, 8000.0), (1, 1, 2000.0)]).unwrap();
        e.advance(2.0 * RUN_S).unwrap();
        let v = pot_wiper(8000.0, 2000.0);
        close("wiper after the move", mode, e.voltages()[2], v, v_tol(mode));
        close("pot elementI after the move", mode, e.currents()[1], (5.0 - v) / 8000.0, v_tol(mode) / 1000.0);
    }
}

// ---- ldr (spec 7.9) --------------------------------------------------------

/// 5 V through `r_top` into a node loaded by `r_bottom` and the shunt.
fn divider(r_top: f64, r_bottom: f64) -> f64 {
    5.0 / r_top / (1.0 / r_top + 1.0 / r_bottom + GS)
}

/// The two-node ladder 5 V -g1- a -g2- b, `ga` and `gb` (plus the shunt) to
/// ground: `(v_a, v_b)` by Cramer's rule.
fn ladder(g1: f64, g2: f64, ga: f64, gb: f64) -> (f64, f64) {
    let (a11, a12, b1) = (g1 + g2 + ga + GS, -g2, 5.0 * g1);
    let (a21, a22) = (-g2, g2 + gb + GS);
    let det = a11 * a22 - a12 * a21;
    (b1 * a22 / det, -a21 * b1 / det)
}

fn ldr_divider(mode: Mode, r: f64) -> Program {
    build(mode, 3, 2, vec![dev(kinds::LDR, 1, &[1, 2], vec![r], vec![]), resistor(2, 2, 0, 4700.0)], 0)
}

#[test]
fn an_ldr_is_a_resistor_at_its_live_resistance_in_every_partition_kind() {
    for mode in MODES {
        let mut e = run(&ldr_divider(mode, 15_000.0));
        let v = divider(15_000.0, 4700.0);
        close("ldr divider", mode, e.voltages()[2], v, v_tol(mode));
        close("ldr elementI", mode, e.currents()[1], (5.0 - v) / 15_000.0, v_tol(mode) / 1000.0);
        // setEnvironment writes the new lux's resistance as live param 0.
        e.set_params(&[(1, 0, 500.0)]).unwrap();
        e.advance(2.0 * RUN_S).unwrap();
        let v = divider(500.0, 4700.0);
        close("ldr divider in the light", mode, e.voltages()[2], v, v_tol(mode));
        close("ldr elementI in the light", mode, e.currents()[1], (5.0 - v) / 500.0, v_tol(mode) / 1000.0);
    }
}

// ---- push_dpdt (spec 7.6) --------------------------------------------------

/// Pole 1: c1 on the 5 V supply, l1 and r1 each loaded (1 kohm, 2 kohm) to
/// ground. Pole 2: c2 fed from the supply through 1 kohm (node 6), l2 and r2
/// loaded by 4.7 kohm and 10 kohm. Pins l1 c1 r1 l2 c2 r2 = 2 1 3 4 6 5.
fn dpdt_board(mode: Mode, position: f64) -> Program {
    let devices = vec![
        dev(kinds::PUSH_DPDT, 1, &[2, 1, 3, 4, 6, 5], vec![position], vec![]),
        resistor(2, 2, 0, 1000.0),
        resistor(3, 3, 0, 2000.0),
        resistor(4, 4, 0, 4700.0),
        resistor(5, 5, 0, 10_000.0),
        resistor(6, 1, 6, 1000.0),
    ];
    build(mode, 7, 2, devices, 0)
}

/// The throw voltage through a closed 1 mohm contact into `r_load`, and the
/// pole-2 throw voltage behind its 1 kohm feed (c2 a node of its own).
fn dpdt_expect(r1_load: f64, r2_load: f64) -> (f64, f64) {
    let v1 = divider(0.001, r1_load);
    let (_, v2) = ladder(1.0 / 1000.0, 1.0 / 0.001, 0.0, 1.0 / r2_load);
    (v1, v2)
}

#[test]
fn a_dpdt_closes_one_throw_per_pole_and_publishes_pole_one_in_every_partition_kind() {
    for mode in MODES {
        let mut e = run(&dpdt_board(mode, 0.0));
        let (v1, v2) = dpdt_expect(1000.0, 4700.0);
        close("l1", mode, e.voltages()[2], v1, v_tol(mode));
        close("r1 (open)", mode, e.voltages()[3], 0.0, v_tol(mode));
        close("l2", mode, e.voltages()[4], v2, v_tol(mode));
        close("r2 (open)", mode, e.voltages()[5], 0.0, v_tol(mode));
        // Pole 1 only, common to its active throw (spec 7.6).
        close("dpdt elementI", mode, e.currents()[1], v1 / 1000.0, 1e-6);
        e.set_params(&[(1, 0, 1.0)]).unwrap();
        e.advance(2.0 * RUN_S).unwrap();
        let (v1, v2) = dpdt_expect(2000.0, 10_000.0);
        close("r1", mode, e.voltages()[3], v1, v_tol(mode));
        close("l1 (open)", mode, e.voltages()[2], 0.0, v_tol(mode));
        close("r2", mode, e.voltages()[5], v2, v_tol(mode));
        close("dpdt elementI at position 1", mode, e.currents()[1], v1 / 2000.0, 1e-6);
    }
}

// ---- current_source (spec 8.13) --------------------------------------------

/// The supply feeds node 2 through 1 kohm; a current source with `pos` on
/// ground and `neg` on node 2 draws 2 mA out of node 2 (it injects `+I` at
/// `pos`, `-I` at `neg`), a second one with a 10 kohm shunt pushes 1 mA into
/// node 3, which a 4.7 kohm resistor ties to node 2, and a third straight
/// across the supply (both ends fixed, or on the rail) draws 0.5 mA.
fn current_board(mode: Mode) -> Program {
    let devices = vec![
        resistor(1, 1, 2, 1000.0),
        current_source(2, 0, 2, 2e-3, 0.0),
        current_source(3, 3, 0, 1e-3, 10_000.0),
        resistor(4, 3, 2, 4700.0),
        current_source(5, 0, 1, 0.5e-3, 0.0),
    ];
    build(mode, 4, 2, devices, 0)
}

/// Nodal analysis of `current_board`: two unknowns, solved by Cramer's rule.
fn current_expect() -> (f64, f64) {
    let (g1, g4, gs) = (1.0 / 1000.0, 1.0 / 4700.0, 1.0 / 10_000.0);
    // Node 2: (5 - v2) g1 + (v3 - v2) g4 - 2 mA = 0; node 3: 1 mA - v3 gs - (v3 - v2) g4 = 0.
    let (a11, a12, b1) = (g1 + g4 + GS, -g4, 5.0 * g1 - 2e-3);
    let (a21, a22, b2) = (-g4, g4 + gs + GS, 1e-3);
    let det = a11 * a22 - a12 * a21;
    ((b1 * a22 - a12 * b2) / det, (a11 * b2 - a21 * b1) / det)
}

#[test]
fn a_current_source_injects_its_current_and_publishes_minus_i_plus_its_shunt_in_every_partition_kind() {
    for mode in MODES {
        let e = run(&current_board(mode));
        let (v2, v3) = current_expect();
        close("node 2", mode, e.voltages()[2], v2, v_tol(mode));
        close("node 3", mode, e.voltages()[3], v3, v_tol(mode));
        // elementI is -I, plus the shunt's V(pos - neg)/rParallel (spec 8.13).
        close("sink elementI", mode, e.currents()[2], -2e-3, 1e-12);
        close("shunted elementI", mode, e.currents()[3], -1e-3 + v3 / 10_000.0, v_tol(mode) / 1000.0);
        close("across-the-supply elementI", mode, e.currents()[5], -0.5e-3, 1e-12);
        // The supply carries the feed resistor's current plus the 0.5 mA one
        // (KCL at node 1 through the injection).
        close("supply", mode, e.currents()[0], -((5.0 - v2) / 1000.0 + 0.5e-3 + 5.0 * GS), v_tol(mode) / 1000.0);
    }
}

#[test]
fn a_current_source_ramps_a_capacitor_exactly_in_closed_form_and_stepped_partitions() {
    // 1 mA into 1 uF on node 2, nothing else on the node but the shunt: dv/dt
    // = I/C, 1 V per ms, exact for BE and trapezoidal alike.
    for stepped in [false, true] {
        let p = Program {
            node_count: 3,
            options: Options { force_stepped: stepped, ..Options::default() },
            digital_slot_count: 0,
            channel_slot_count: 0,
            devices: vec![
                dev(kinds::VOLTAGE_SOURCE, 0, &[1, 0], vec![5.0], vec![]),
                current_source(1, 2, 0, 1e-3, 0.0),
                capacitor(2, 2, 0, 1e-6),
            ],
            carry: Vec::new(),
        };
        let mut e = Engine::load(&p).unwrap();
        e.advance(2e-3).unwrap();
        close("ramp", if stepped { Mode::Stepped } else { Mode::ClosedForm }, e.voltages()[2], 2.0, 1e-6);
        close("capacitor current", Mode::Stepped, e.currents()[2], 1e-3, 1e-9);
    }
}

#[test]
fn a_live_current_write_moves_the_injection() {
    for mode in MODES {
        let mut e = run(&current_board(mode));
        // The sink to 1 mA: node 2 rises.
        e.set_params(&[(2, 0, 1e-3)]).unwrap();
        e.advance(2.0 * RUN_S).unwrap();
        let (g1, g4, gs) = (1.0 / 1000.0, 1.0 / 4700.0, 1.0 / 10_000.0);
        let (a11, a12, b1) = (g1 + g4 + GS, -g4, 5.0 * g1 - 1e-3);
        let (a21, a22, b2) = (-g4, g4 + gs + GS, 1e-3);
        let v2 = (b1 * a22 - a12 * b2) / (a11 * a22 - a12 * a21);
        close("node 2 after the write", mode, e.voltages()[2], v2, v_tol(mode));
        close("sink elementI after the write", mode, e.currents()[2], -1e-3, 1e-12);
    }
}

// ---- resistor_array (spec 10.10) -------------------------------------------

/// Channel 1: supply -1 kohm- node 2 -ch1- ground (Static or, with the probe,
/// dynamic). Channel 2: supply -ch2- node 3, 10 kohm and a capacitor (always
/// a closed-form or stepped partition of its own) to ground. Channel 3: across
/// the supply (no partition). Channel 4: both pins on ground (absent ids). All
/// channels 10 kohm; channel slots 0..3.
fn array_board(mode: Mode) -> Program {
    let devices = vec![
        resistor(1, 1, 2, 1000.0),
        dev(kinds::RESISTOR_ARRAY, 2, &[2, 0, 1, 3, 1, 0, 0, 0], vec![10_000.0, 0.0], vec![0.0]),
        resistor(3, 3, 0, 10_000.0),
        capacitor(4, 3, 0, PROBE_C),
    ];
    build(mode, 4, 2, devices, 4)
}

#[test]
fn an_array_solves_each_channel_where_it_sits_and_publishes_four_signed_channels_and_their_sum() {
    for mode in MODES {
        let e = run(&array_board(mode));
        let v2 = divider(1000.0, 10_000.0);
        let v3 = divider(10_000.0, 10_000.0);
        close("channel 1 node", mode, e.voltages()[2], v2, v_tol(mode));
        close("channel 2 node", mode, e.voltages()[3], v3, 1e-6);
        let want = [v2 / 10_000.0, (5.0 - v3) / 10_000.0, 5.0 / 10_000.0, 0.0];
        let channels = e.channel_slots();
        for (ch, &w) in want.iter().enumerate() {
            close(&format!("channel {}", ch + 1), mode, channels[ch], w, 1e-9);
        }
        close("array elementI", mode, e.currents()[2], want.iter().sum(), 1e-9);
        // The channels are never joined: channel 2's partition is not channel 1's.
        close("channel 1 feed", mode, e.currents()[1], (5.0 - v2) / 1000.0, 1e-9);
    }
}

// ---- P0-SG: the H1 kinds under the 1.0.0 line's charge and snapshot ------

/// Every board above, each kind on it, and the channel count each device
/// adds to a charge take (an array's four).
fn w1h_boards(mode: Mode) -> Vec<(&'static str, Program)> {
    vec![
        ("pot", pot_divider(mode, kinds::POTENTIOMETER, 3000.0, 7000.0)),
        ("trimmer", pot_divider(mode, kinds::TRIMMER, 1.0, 9999.0)),
        ("ldr", ldr_divider(mode, 15_000.0)),
        ("dpdt", dpdt_board(mode, 1.0)),
        ("current source", current_board(mode)),
        ("array", array_board(mode)),
    ]
}

/// W2b's charge of every device but the supply, armed in steady state: each
/// device's charge is its published current times the span and an array's
/// channels are its channel slots' (the take carries four channels for it, as
/// `CoreEngine.armCharge` counts its channel slots), within W0's charge bound.
#[test]
fn the_charge_of_each_h1_kind_is_its_published_current_and_an_arrays_four_channels() {
    const SPAN: f64 = 0.02;
    for mode in MODES {
        for (name, p) in w1h_boards(mode) {
            let mut e = run(&p);
            let devices: Vec<u32> = (1..p.devices.len() as u32).collect();
            e.charge_arm(&devices).unwrap();
            e.advance(RUN_S + SPAN).unwrap();
            let take = e.take_charge().to_vec();
            let channels: Vec<usize> = devices.iter().map(|&d| if p.devices[d as usize].kind == kinds::RESISTOR_ARRAY { 4 } else { 0 }).collect();
            let slots: usize = channels.iter().map(|c| 1 + c).sum();
            assert_eq!(take.len(), 2 + slots + 2 * devices.len(), "{name} ({mode:?}): one slot per device and channel");
            let (mut at, currents, ch) = (2, e.currents().to_vec(), e.channel_slots().to_vec());
            for (k, &d) in devices.iter().enumerate() {
                let i = currents[d as usize];
                close(&format!("{name} device {d} charge"), mode, take[at], i * SPAN, (1e-9 + 1e-3 * i.abs()) * SPAN);
                for c in 0..channels[k] {
                    let want = ch[p.devices[d as usize].blob[0] as usize + c] * SPAN;
                    close(&format!("{name} channel {c} charge"), mode, take[at + 1 + c], want, (1e-9 + 1e-3 * (want / SPAN).abs()) * SPAN);
                }
                at += 1 + channels[k];
            }
        }
    }
}

/// W2a's snapshot holds a live kind's moved params: restored, the engine runs
/// on as one that never saw the later move; and a moved pot is the same
/// engine bit for bit whichever copy ran.
#[test]
fn a_snapshot_keeps_each_live_h1_kinds_params_and_restores_them() {
    for mode in MODES {
        for (name, p, device, param, value) in [
            ("pot", pot_divider(mode, kinds::POTENTIOMETER, 3000.0, 7000.0), 1u32, 0u32, 6000.0),
            ("trimmer", pot_divider(mode, kinds::TRIMMER, 3000.0, 7000.0), 1, 0, 6000.0),
            ("ldr", ldr_divider(mode, 15_000.0), 1, 0, 2000.0),
            ("dpdt", dpdt_board(mode, 1.0), 1, 0, 0.0),
            ("current source", current_board(mode), 2, 0, 1e-3),
        ] {
            let mut e = run(&p);
            let snap = e.snapshot();
            e.set_params(&[(device, param, value)]).unwrap();
            e.advance(2.0 * RUN_S).unwrap();
            let moved = (e.voltages().to_vec(), e.currents().to_vec());
            e.restore(&snap);
            let mut never = run(&p);
            never.advance(2.0 * RUN_S).unwrap();
            e.advance(2.0 * RUN_S).unwrap();
            assert_eq!(e.voltages(), never.voltages(), "{name} ({mode:?}): the restored engine never saw the move");
            assert_eq!(e.currents(), never.currents(), "{name} ({mode:?})");
            let mut again = run(&p);
            again.set_params(&[(device, param, value)]).unwrap();
            again.advance(2.0 * RUN_S).unwrap();
            assert_eq!((again.voltages().to_vec(), again.currents().to_vec()), moved, "{name} ({mode:?}): the move replays bit for bit");
        }
    }
}

// ---- P0-SG2: a carried reload across the stateless H1 kinds ----------------

/// `p` reloaded at `e`'s time with `e`'s carry (program v4's block), as the host's reload does.
fn reloaded(e: &mut Engine, p: &Program) -> Engine {
    let mut q = p.clone();
    q.carry = e.carry_take().unwrap().to_vec();
    q.options.start_time = e.time();
    Engine::load(&q).unwrap()
}

/// An ldr, a push_dpdt and a current source hold no slow state of their own: what a reload must keep for them is the charge
/// of every capacitor on their partitions and the value of the live param the host edited (it reloads with the circuit's
/// edited value). Mid-transient, in every partition kind: the reloaded engine reads what the one that never reloaded does.
#[test]
fn a_carried_reload_runs_an_ldr_a_dpdt_and_a_current_source_on_as_the_run_that_never_reloaded() {
    for mode in MODES {
        for (name, p, device, param, value) in [
            ("ldr", ldr_divider(mode, 15_000.0), 1usize, 0usize, 2000.0),
            ("dpdt", dpdt_board(mode, 1.0), 1, 0, 0.0),
            ("current source", current_board(mode), 2, 0, 1e-3),
        ] {
            let mut edited = p.clone();
            edited.devices[device].params[param] = value;
            let mut never = Engine::load(&p).unwrap();
            let mut moved = Engine::load(&p).unwrap();
            for e in [&mut never, &mut moved] {
                e.advance(2e-4).unwrap();
                e.set_params(&[(device as u32, param as u32, value)]).unwrap();
                e.advance(4e-4).unwrap();
            }
            let before = never.voltages().to_vec();
            assert!(before.iter().any(|v| v.abs() > 0.1), "{name} ({mode:?}): setup, something is charged");
            let mut r = reloaded(&mut moved, &edited);
            assert_eq!(r.time(), 4e-4);
            for e in [&mut never, &mut r] {
                e.advance(8e-4).unwrap();
            }
            for (k, (got, want)) in r.voltages().iter().zip(never.voltages()).enumerate() {
                assert!((got - want).abs() <= 2e-4, "{name} ({mode:?}): node {k} {got} vs {want}");
            }
            for (k, (got, want)) in r.currents().iter().zip(never.currents()).enumerate() {
                assert!((got - want).abs() <= 1e-6 + 1e-3 * want.abs(), "{name} ({mode:?}): device {k} {got:e} vs {want:e}");
            }
        }
    }
}

/// A live edit the solve cannot take (the ldr's resistance so small its conductance is not finite; a source current so large the node voltage overflows)
/// is refused with `-4` and rolls back with its instant: the live param is back, the readouts are as they were, and the run goes on as a twin that
/// never saw the edit (README `bo_engine_set_params`: all or nothing).
#[test]
fn a_live_edit_the_solve_cannot_take_rolls_back_with_its_instant_for_the_ldr_and_the_current_source() {
    for mode in MODES {
        for (name, p, device, param, value) in [
            ("ldr", ldr_divider(mode, 15_000.0), 1u32, 0u32, 5e-324),
            ("current source", current_board(mode), 2, 0, 1e308),
        ] {
            // A closed-form or stepped partition integrates the overflowing drive without a non-finite stamp (it saturates), so only
            // the algebraic partitions refuse it; the ldr's conductance is non-finite in every kind.
            if name == "current source" && !matches!(mode, Mode::Static | Mode::Hub) {
                continue;
            }
            let (mut doomed, mut twin) = (Engine::load(&p).unwrap(), Engine::load(&p).unwrap());
            for e in [&mut doomed, &mut twin] {
                e.advance(2e-4).unwrap();
            }
            let before = (doomed.voltages().to_vec(), doomed.currents().to_vec());
            let err = doomed.set_params(&[(device, param, value)]).expect_err(&format!("{name} ({mode:?}): the edit must fail the solve"));
            assert_eq!(err.code, crate::error::ERR_SOLVE_FAILED, "{name} ({mode:?}): {err:?}");
            assert_eq!((doomed.voltages().to_vec(), doomed.currents().to_vec()), before, "{name} ({mode:?}): the readouts are as they were");
            assert_eq!(doomed.time(), twin.time());
            for t in [4e-4, 1e-3, 2e-3] {
                doomed.advance(t).unwrap();
                twin.advance(t).unwrap();
                assert_eq!(doomed.voltages(), twin.voltages(), "{name} ({mode:?}): the run goes on as the twin's at {t}");
                assert_eq!(doomed.currents(), twin.currents(), "{name} ({mode:?}) at {t}");
            }
        }
    }
}
