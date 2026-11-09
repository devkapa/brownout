//! W1e worker E2b: the LED kinds through the whole solver, end to end
//! (README, "Kernel semantics: LEDs and displays (W1e)"), driven through a
//! real `Engine`.
//!
//! Every expected number is an independent computation, never core output:
//! `drled-sidecars.mjs` solves each one-node circuit by bisection on its own
//! KCL (the model's equations, GMIN and node shunt included), the seg7 case
//! by nested bisection, and the RC-fed LED integrates the partition's own ODE
//! with an adaptive Dormand-Prince RK45 at rtol 1e-11. The printed digits are
//! copied here at full precision, and tolerances are the contract's (1 nV to
//! 1 nA on the analytic circuits, W0's 100 uV + 1e-3 relative on a stepped
//! trajectory).

use crate::engine::Engine;
use crate::error::ERR_SOLVE_FAILED;
use crate::kinds::{self, ic_param};
use crate::program::{Device, Options, PinRecord, Program, PIN_FLAG_OPEN};
use crate::solve::newton::hooks;
use crate::solve::{PartitionSolver, PartitionedSolver};

// ============================================================================
// Program-building helpers.
// ============================================================================

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

/// A pin the compiler flagged `OPEN`: it sits on the datum node.
fn open_pin() -> PinRecord {
    PinRecord { node: 0, flags: PIN_FLAG_OPEN, open_hash: 0, dead_hash: 0 }
}

fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
    Device { kind: kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
}

fn capacitor(ci: usize, a: usize, b: usize, c: f64) -> Device {
    Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![c, 0.0, 0.0], blob: vec![] }
}

fn switch_dev(ci: usize, a: usize, b: usize, closed: f64) -> Device {
    Device { kind: kinds::SWITCH, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![closed], blob: vec![] }
}

/// A `led` with the catalog red's params: `[vf, n, iRated, isSet, is, burnt]`.
fn led_vf(ci: usize, anode: usize, cathode: usize, vf: f64, burnt: f64) -> Device {
    Device { kind: kinds::LED, component_index: ci, pins: vec![pin(anode), pin(cathode)], params: vec![vf, 2.0, 0.02, 0.0, 0.0, burnt], blob: vec![] }
}

fn red_led(ci: usize, anode: usize, cathode: usize) -> Device {
    led_vf(ci, anode, cathode, 1.8, 0.0)
}

/// `rgb_led`: pins `r_a g_a b_a com_k`, params `[vfR, vfG, vfB, iRated,
/// ifMax, burnt]`, blob `[channelBase]`.
fn rgb(ci: usize, r_a: usize, g_a: usize, b_a: usize, com_k: usize, base: usize, burnt: f64) -> Device {
    Device {
        kind: kinds::RGB_LED,
        component_index: ci,
        pins: vec![pin(r_a), pin(g_a), pin(b_a), pin(com_k)],
        params: vec![1.8, 2.0, 3.0, 0.02, 0.02, burnt],
        blob: vec![base as f64],
    }
}

/// `bicolor_led`: pins `a1 a2 k`, params `[vf1, vf2, iRated, ifMax, burnt]`.
fn bicolor(ci: usize, a1: usize, a2: usize, k: usize, base: usize, burnt: f64) -> Device {
    Device {
        kind: kinds::BICOLOR_LED,
        component_index: ci,
        pins: vec![pin(a1), pin(a2), pin(k)],
        params: vec![1.8, 2.0, 0.02, 0.02, burnt],
        blob: vec![base as f64],
    }
}

/// `seg7_cc`/`seg7_ca`: pins `a b c d e f g dp com com2`, params `[vf, iRated]`.
fn seg7(kind: u32, ci: usize, pins: Vec<PinRecord>, base: usize) -> Device {
    assert_eq!(pins.len(), 10);
    Device { kind, component_index: ci, pins, params: vec![2.0, 0.02], blob: vec![base as f64] }
}

fn program(node_count: usize, channel_slot_count: usize, devices: Vec<Device>) -> Program {
    Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count, devices }
}

fn at_temperature(mut p: Program, ambient_c: f64) -> Program {
    p.options.ambient_c = ambient_c;
    p
}

fn load(p: &Program) -> Engine {
    Engine::load(p).expect("the program loads")
}

/// The accuracy of a polished one-node LED solve. The polish (LEDs point
/// 6.4) is one untested iteration from a passing iterate, so what it
/// publishes is the fixed point to the README's 0.39 uV bound for the worst
/// case measured. On these circuits the worst measured are 4.8e-8 V (the
/// LED-then-resistor chain, which also gives 9e-9 A) and 9e-10 A (the white LED); the unpolished
/// iterate is off by up to 1.4e-4 V (the seeded-fault test in `static_part`),
/// which 1e-7 V tells apart. Tighter needs a second polish, which the
/// contract does not take.
const V_TOL: f64 = 1e-7;
const I_TOL: f64 = 2e-8;

fn assert_close(name: &str, got: f64, want: f64, tol: f64) {
    assert!((got - want).abs() <= tol, "{name}: got {got:e}, expected {want:e} (tolerance {tol:e}, off by {:e})", (got - want).abs());
}

/// `5 V --R-- anode --LED-- gnd`: device 0 the source, 1 the resistor, 2 the
/// LED; node 1 the rail, node 2 the anode.
fn one_node_led(r: f64, vf: f64) -> Program {
    program(3, 0, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, r), led_vf(2, 2, 0, vf, 0.0)])
}

// ============================================================================
// One-node circuits against the analytic sidecar (F1-F4).
// ============================================================================

#[test]
fn f1_a_red_led_on_5v_through_220_ohm_matches_the_sidecar_and_the_source_current_matches_the_resistor() {
    let e = load(&one_node_led(220.0, 1.8));
    // drled-sidecars.mjs F1, full precision.
    assert_close("anode", e.voltages()[2], 1.783894180976496, V_TOL);
    assert_close("LED elementI", e.currents()[2], 0.014618662810175494, I_TOL);
    assert_close("resistor current", e.currents()[1], 0.014618662813743199, I_TOL);
    assert_close("source current", e.currents()[0], -0.014618662813743199, 1e-9);
    assert!((-e.currents()[0] - e.currents()[1]).abs() < 1e-9, "the source carries what the resistor does");
    assert_eq!(e.stat(0), 1.0, "one partition");
    assert_eq!(e.stat(1), 1.0, "one static solve");
    assert!(e.stat(11) >= 2.0, "a static Newton solve is counted: {}", e.stat(11));
}

#[test]
fn f2_each_catalog_colour_on_5v_through_150_ohm_matches_its_sidecar() {
    // (vf, anode volts, LED amps) from drled-sidecars.mjs F2. Green sits at
    // its own Vf: I = iRated - Is.
    for (name, vf, va, i_led) in [
        ("red", 1.8, 1.8032638843613227, 0.021311574100651327),
        ("green", 2.0, 1.999999999989896, 0.01999999999606729),
        ("blue", 3.0, 2.9796844372806204, 0.013468770412169845),
        ("white", 3.2, 3.1744747047199144, 0.012170168628851591),
    ] {
        let e = load(&one_node_led(150.0, vf));
        assert_close(&format!("{name} anode"), e.voltages()[2], va, V_TOL);
        assert_close(&format!("{name} current"), e.currents()[2], i_led, I_TOL);
    }
}

#[test]
fn f3_the_ambient_temperature_moves_the_operating_point_the_way_the_law_says() {
    // drled-sidecars.mjs F3: red (vf 1.8) on 5 V through 1 kohm.
    for (t, va) in [(-40.0, 1.855657027765698), (25.0, 1.7072999230803072), (85.0, 1.5711458932521007), (125.0, 1.4807736652005992)] {
        let e = load(&at_temperature(one_node_led(1000.0, 1.8), t));
        assert_close(&format!("anode at {t} C"), e.voltages()[2], va, V_TOL);
    }
}

#[test]
fn f4_a_led_across_a_tenth_of_an_ohm_runs_on_the_tangent_past_vsat() {
    // The tangent line against the load line, in closed form: vSat is
    // 2.056925791 V and the answer is 2.447 V, well past it.
    let e = load(&one_node_led(0.1, 1.8));
    assert_close("anode", e.voltages()[2], 2.4474294816491375, V_TOL);
    assert_close("LED current", e.currents()[2], 25.525705183503767, 25.5 * 1e-9);
}

// ============================================================================
// Multi-junction kinds (F5-F7) and their channel slots.
// ============================================================================

#[test]
fn f5_a_bicolor_led_with_its_cathode_on_the_datum_solves_two_partitions_and_publishes_two_channels() {
    // a1 through 150 ohm and a2 through 330 ohm from 5 V, k on the datum.
    let p = program(4, 2, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 150.0),
        resistor(2, 1, 3, 330.0),
        bicolor(3, 2, 3, 0, 0, 0.0),
    ]);
    let e = load(&p);
    assert_eq!(e.stat(0), 2.0, "a1 and a2 are two partitions, since k is fixed and the union is per junction");
    assert_close("a1", e.voltages()[2], 1.8032638843613227, V_TOL);
    assert_close("a2", e.voltages()[3], 1.9601628491068799, V_TOL);
    assert_close("channel a1", e.channel_slots()[0], 0.021311574100651327, I_TOL);
    assert_close("channel a2", e.channel_slots()[1], 0.009211627726058816, I_TOL);
    assert_close("elementI is the larger", e.currents()[3], 0.021311574100651327, I_TOL);
    assert_eq!(e.currents()[3], e.channel_slots()[0], "and it is the same number as the larger channel");
}

#[test]
fn f6_an_rgb_led_publishes_three_channels_and_the_running_maximum() {
    // r_a and g_a through 150 ohm, b_a through 1 Gohm, com_k on the datum.
    let p = program(5, 3, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 150.0),
        resistor(2, 1, 3, 150.0),
        resistor(3, 1, 4, 1e9),
        rgb(4, 2, 3, 4, 0, 0, 0.0),
    ]);
    let e = load(&p);
    assert_eq!(e.stat(0), 3.0);
    assert_close("channel r", e.channel_slots()[0], 0.021311574100651327, I_TOL);
    assert_close("channel g", e.channel_slots()[1], 0.01999999999606729, I_TOL);
    assert_close("channel b", e.channel_slots()[2], 2.806444725934853e-9, I_TOL);
    assert_close("elementI", e.currents()[4], 0.021311574100651327, I_TOL);
}

/// `seg7_cc` showing "7": `a b c` through 330 ohm from 5 V, `d` and `e` and
/// `dp` unwired (`OPEN`), `f` and `g` wired straight to the datum (reverse
/// biased against the raised common), `com` to the datum through 10 ohm.
fn seg7_digit_7() -> Program {
    let pins = vec![pin(2), pin(3), pin(4), open_pin(), open_pin(), pin(0), pin(0), open_pin(), pin(5), open_pin()];
    program(6, 8, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 330.0),
        resistor(2, 1, 3, 330.0),
        resistor(3, 1, 4, 330.0),
        resistor(4, 5, 0, 10.0),
        seg7(kinds::SEG7_CC, 5, pins, 0),
    ])
}

#[test]
fn f7_a_seg7_cc_showing_7_shares_one_partition_and_matches_the_sidecar() {
    let e = load(&seg7_digit_7());
    assert_eq!(e.stat(0), 1.0, "every segment joins the free common");
    assert_close("common", e.voltages()[5], 0.25368616447233483, V_TOL);
    for node in [2, 3, 4] {
        assert_close("anode", e.voltages()[node], 2.209452190047295, V_TOL);
    }
    for (channel, want) in [(0, 0.00845620548053996), (1, 0.00845620548053996), (2, 0.00845620548053996)] {
        assert_close("lit segment", e.channel_slots()[channel], want, I_TOL);
    }
    // d, e, dp are OPEN, f and g are reverse biased: all four read exactly 0.
    for channel in [3, 4, 5, 6, 7] {
        assert_eq!(e.channel_slots()[channel], 0.0, "channel {channel}");
    }
    assert_close("elementI is the sum in channel order", e.currents()[5], 0.02536861644161988, 3.0 * I_TOL);
}

#[test]
fn a_seg7_ca_on_a_fixed_common_solves_each_segment_in_its_own_partition_beside_its_ic_driver() {
    // Common anode on the 5 V rail, segment `a` through 330 ohm to a 74LS04
    // output that is LOW (R_out 156.25 ohm to the datum), the other segments
    // unwired. The junction runs from a fixed anode to a free cathode, so it
    // sits in the segment's own partition with the resistor and the driver.
    // Bisection on the chain's KCL: drled e2b-values3.mjs.
    let mut pins: Vec<PinRecord> = (0..10).map(|_| open_pin()).collect();
    pins[0] = pin(3);
    pins[8] = pin(1);
    let mut p = program(9, 8, vec![
        source(0, 1, 0, 5.0),
        ls04(1, 1, 0, [1, 2, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], 0, 0.0),
        resistor(2, 2, 3, 330.0),
        seg7(kinds::SEG7_CA, 3, pins, 0),
    ]);
    p.digital_slot_count = 6;
    let e = load(&p);
    assert_eq!(e.stat(0), 6.0, "the segment's partition and the five other gate outputs");
    assert_close("cathode", e.voltages()[3], 3.0594255339733873, V_TOL);
    assert_close("driver pin", e.voltages()[2], 0.9831058912753734, V_TOL);
    assert_close("channel a", e.channel_slots()[0], 0.0062918777062643495, I_TOL);
    assert_close("elementI", e.currents()[3], 0.0062918777062643495, I_TOL);
    for channel in 1..8 {
        assert_eq!(e.channel_slots()[channel], 0.0);
    }
}

// ============================================================================
// Burnt devices, unwired displays, the com2 fallback (F11, F12).
// ============================================================================

#[test]
fn f11_a_burnt_led_stamps_nothing_reads_zero_and_leaves_its_partition_linear() {
    let e = load(&program(3, 0, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 220.0), led_vf(2, 2, 0, 1.8, 1.0)]));
    assert_eq!(e.currents()[2], 0.0);
    // No junction: the anode floats up to the rail through the resistor.
    assert_close("anode", e.voltages()[2], 5.0, 1e-6);
    assert_eq!(e.stat(11), 0.0, "a linear partition takes the direct solve, no Newton iteration");
}

#[test]
fn f11_a_burnt_rgb_or_bicolor_led_has_nan_channels_and_a_zero_element_current() {
    let e = load(&program(5, 5, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 150.0),
        resistor(2, 1, 3, 150.0),
        resistor(3, 1, 4, 150.0),
        rgb(4, 2, 3, 4, 0, 0, 1.0),
        bicolor(5, 2, 3, 0, 3, 1.0),
    ]));
    for channel in 0..5 {
        assert!(e.channel_slots()[channel].is_nan(), "channel {channel} is absent, not 0");
    }
    assert_eq!(e.currents()[4], 0.0);
    assert_eq!(e.currents()[5], 0.0);
    assert_close("nothing loads the anode", e.voltages()[2], 5.0, 1e-6);
}

#[test]
fn f12_a_seg7_with_nothing_wired_reads_eight_zero_channels() {
    let p = program(2, 8, vec![source(0, 1, 0, 5.0), seg7(kinds::SEG7_CC, 1, (0..10).map(|_| open_pin()).collect(), 0)]);
    let e = load(&p);
    assert_eq!(e.channel_slots(), &[0.0; 8]);
    assert_eq!(e.currents()[1], 0.0);
    assert_eq!(e.stat(11), 0.0);
}

#[test]
fn f12_a_seg7_whose_com_is_open_stamps_through_com2() {
    // Segment `a` from 5 V through 330 ohm; `com` OPEN, `com2` on node 3 with
    // 100 ohm to the datum.
    let mut pins: Vec<PinRecord> = (0..10).map(|_| open_pin()).collect();
    pins[0] = pin(2);
    pins[9] = pin(3);
    let p = program(4, 8, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 330.0), resistor(2, 3, 0, 100.0), seg7(kinds::SEG7_CC, 3, pins, 0)]);
    let e = load(&p);
    let i_a = e.channel_slots()[0];
    assert!(i_a > 5e-3 && i_a < 8e-3, "segment a conducts through com2: {i_a}");
    assert_close("elementI equals the one lit segment", e.currents()[3], i_a, 1e-15);
    for channel in 1..8 {
        assert_eq!(e.channel_slots()[channel], 0.0);
    }
}

// ============================================================================
// An LED across two fixed nodes (no partition), and the source KCL.
// ============================================================================

#[test]
fn an_led_straight_across_an_ideal_source_belongs_to_no_partition_and_loads_the_source() {
    let e = load(&program(2, 0, vec![source(0, 1, 0, 5.0), red_led(1, 1, 0)]));
    assert_eq!(e.stat(0), 0.0, "no free node, no partition");
    // F4b (drled-sidecars.mjs): I(5 V) past vSat on the tangent.
    assert_close("LED current I(5)", e.currents()[1], 172.97491714278573, 172.97 * 1e-9);
    // The branch current with GMIN and the node shunt enters the source KCL.
    assert_close("source current", e.currents()[0], -172.97491714279573, 172.97 * 1e-9);
    assert_eq!(e.stat(11), 0.0, "constant, so nothing is iterated");
}

#[test]
fn a_led_with_its_anode_on_the_rail_carries_the_source_current_through_the_kcl_fold() {
    // 5 V -> LED (anode on the rail, cathode free) -> 330 ohm -> datum. The
    // source's only path is the junction, so its current is the junction's
    // branch current (GMIN included) plus the rail's own shunt, and no
    // resistor is in the KCL at the rail at all. Bisection on the chain's
    // KCL: drled e2b-values3.mjs.
    let e = load(&program(3, 0, vec![source(0, 1, 0, 5.0), red_led(1, 1, 2), resistor(2, 2, 0, 330.0)]));
    assert_close("cathode", e.voltages()[2], 3.236614078441296, V_TOL);
    assert_close("LED elementI", e.currents()[1], 0.009807921451295398, I_TOL);
    assert_close("source current", e.currents()[0], -0.009807921458058784, I_TOL);
}

// ============================================================================
// The solution cache with nonlinear partitions (LEDs point 7).
// ============================================================================

/// `5 V -- switch -- 1 kohm -- LED`, device 1 the switch.
fn switched_led(closed: f64) -> Program {
    program(4, 0, vec![source(0, 1, 0, 5.0), switch_dev(1, 1, 2, closed), resistor(2, 2, 3, 1000.0), red_led(3, 3, 0)])
}

#[test]
fn toggling_a_switch_in_an_led_partition_misses_the_cache_and_repeating_the_same_value_hits_it() {
    let mut e = load(&switched_led(1.0));
    assert_eq!(e.stat(7), 0.0);
    let closed_v = e.voltages()[3];
    e.set_params(&[(1, 0, 0.0)]).unwrap();
    let open_v = e.voltages()[3];
    e.set_params(&[(1, 0, 1.0)]).unwrap();
    e.set_params(&[(1, 0, 0.0)]).unwrap();
    e.set_params(&[(1, 0, 1.0)]).unwrap();
    assert_eq!(e.stat(7), 0.0, "every real toggle bumps the epoch, so none can hit a pre-toggle entry");
    assert!(open_v < 1e-6, "open: the LED node falls to 0 V, {open_v}");
    assert_close("closed again lands where it started", e.voltages()[3], closed_v, 1e-6);
    // Setting the value it already has changes nothing, so the epoch stands
    // and the same key comes round again: a hit, with no new iteration.
    let iterations = e.stat(11);
    e.set_params(&[(1, 0, 1.0)]).unwrap();
    assert_eq!(e.stat(7), 1.0);
    assert_eq!(e.stat(11), iterations, "a hit runs no Newton iteration");
}

/// The IC blob: the first digital slot (6 for these two kinds), then one flag
/// per logic pin, `FLOATING_INPUT` on the inputs (the even-indexed pins of the
/// `1a 1y 2a 2y ...` layout).
fn inverter_blob(slot_base: usize) -> Vec<f64> {
    std::iter::once(slot_base as f64).chain((0..12).map(|i| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 })).collect()
}

/// A 74HC14 on `vcc`/`gnd`: `pins` are its logic pins `1a 1y 2a 2y ...`.
fn hc14(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12], slot_base: usize, delay: f64) -> Device {
    let mut params = vec![0.0; ic_param::COUNT];
    params[ic_param::VCC_MIN] = 2.0;
    params[ic_param::VCC_MAX] = 6.0;
    params[ic_param::FAM_FLAGS] = kinds::FAM_CMOS_RATIO as f64;
    params[ic_param::SCHMITT] = 1.0;
    params[ic_param::VT_PLUS] = 2.7;
    params[ic_param::VT_MINUS] = 1.6;
    params[ic_param::NOMINAL] = 5.0;
    params[ic_param::IO_MAX] = 0.02;
    params[ic_param::DELAY] = delay;
    let mut device_pins = vec![pin(vcc), pin(gnd)];
    device_pins.extend(pins.iter().map(|&n| pin(n)));
    Device { kind: kinds::IC_74HC14, component_index: ci, pins: device_pins, params, blob: inverter_blob(slot_base) }
}

/// A 74LS04 (R_out 156.25 ohm at 5 V) on `vcc`/`gnd`.
fn ls04(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12], slot_base: usize, delay: f64) -> Device {
    let mut params = vec![0.0; ic_param::COUNT];
    params[ic_param::VCC_MIN] = 4.75;
    params[ic_param::VCC_MAX] = 5.25;
    params[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    params[ic_param::VIL_FIXED] = 1.0;
    params[ic_param::VIL] = 0.8;
    params[ic_param::VIH_FIXED] = 1.0;
    params[ic_param::VIH] = 2.0;
    params[ic_param::NOMINAL] = 5.0;
    params[ic_param::IO_MAX] = 0.008;
    params[ic_param::DELAY] = delay;
    let mut device_pins = vec![pin(vcc), pin(gnd)];
    device_pins.extend(pins.iter().map(|&n| pin(n)));
    Device { kind: kinds::IC_74LS04, component_index: ci, pins: device_pins, params, blob: inverter_blob(slot_base) }
}

/// The 74HC14 relaxation oscillator (`1y` on node 2, X on node 3, R = 10 kohm,
/// C = 100 nF) whose output feeds two 74LS04 gates on the same rail: gate 1
/// drives `330 ohm + red LED` (nodes 4, 5) and gate 2 drives a plain 1 kohm
/// (node 6). Nothing is ever set through `bo_engine_set_params`: the level
/// changes come only from the oscillator. `no_solution_cache` is the option.
pub(super) fn clocked_led(no_solution_cache: bool) -> Program {
    let mut p = program(29, 0, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 2, 3, 10_000.0),
        capacitor(2, 3, 0, 100e-9),
        hc14(3, 1, 0, [3, 2, 0, 20, 0, 21, 0, 22, 0, 23, 0, 24], 0, 15e-9),
        ls04(4, 1, 0, [2, 4, 2, 6, 0, 25, 0, 26, 0, 27, 0, 28], 6, 10e-9),
        resistor(5, 4, 5, 330.0),
        red_led(6, 5, 0),
        resistor(7, 6, 0, 1000.0),
    ]);
    p.options.no_solution_cache = no_solution_cache;
    p.digital_slot_count = 12;
    p
}

/// Node 4 (gate 1's output) is HIGH when it sits above 2.5 V.
fn gate_1_is_high(e: &Engine) -> bool {
    e.voltages()[4] > 2.5
}

#[test]
fn an_ic_output_alternating_into_r_plus_led_hits_the_cache_from_the_third_flip_and_publishes_the_stored_bits() {
    let mut e = load(&clocked_led(false));
    let mut high_bits: Option<(u64, u64)> = None;
    let mut low_bits: Option<(u64, u64)> = None;
    let mut iterations_after_both_seen = None;
    let mut flips = 0;
    let mut last_high = gate_1_is_high(&e);
    let mut t = 0.0;
    // 8 ms is about 9 periods of the oscillator: 18 output flips.
    while t < 8e-3 {
        t += 25e-6;
        e.advance(t).unwrap();
        let high = gate_1_is_high(&e);
        if high != last_high {
            flips += 1;
            last_high = high;
        }
        let bits = (e.voltages()[4].to_bits(), e.voltages()[5].to_bits());
        let slot = if high { &mut high_bits } else { &mut low_bits };
        match slot {
            None => *slot = Some(bits),
            Some(first) => assert_eq!(*first, bits, "t={t}: the LED partition must publish the same bits for the same drive"),
        }
        if high_bits.is_some() && low_bits.is_some() && iterations_after_both_seen.is_none() {
            iterations_after_both_seen = Some(e.stat(11));
        }
    }
    assert!(flips >= 10, "the oscillator kept flipping the gate: {flips}");
    // Each level was solved once; every later flip replayed the stored vector.
    assert_eq!(Some(e.stat(11)), iterations_after_both_seen, "no Newton iteration after the first two flips");
    assert!(e.stat(7) >= (flips - 2) as f64, "a hit on every flip after the first two: {} hits, {flips} flips", e.stat(7));
}

#[test]
fn no_solution_cache_agrees_within_a_microvolt_for_the_led_and_bit_for_bit_for_the_linear_partition() {
    let mut cached = load(&clocked_led(false));
    let mut uncached = load(&clocked_led(true));
    let mut t = 0.0;
    while t < 4e-3 {
        t += 37e-6;
        cached.advance(t).unwrap();
        uncached.advance(t).unwrap();
        for node in [4, 5] {
            assert_close(&format!("t={t} node {node}"), cached.voltages()[node], uncached.voltages()[node], 1e-6);
        }
        assert_eq!(cached.voltages()[6].to_bits(), uncached.voltages()[6].to_bits(), "t={t}: the linear partition stays bit-identical");
    }
    assert_eq!(uncached.stat(7), 0.0, "NO_SOLUTION_CACHE never hits");
    assert!(cached.stat(7) > 0.0);
    assert!(uncached.stat(11) > cached.stat(11), "the uncached run re-iterates on every flip");
}

// ============================================================================
// The combined loop: a free IC supply node beside a junction (LEDs point 9).
// ============================================================================

/// `5 V --100 ohm-- vcc(2)`, a 74HC14 on that supply whose input is LOW so
/// gate 1's output (node 3) drives HIGH through R_out, then `330 ohm` and a
/// red LED. R_out follows the supply, so the partition iterates the IC
/// driver and the junction in one loop. Device 3 is the IC, 4 the switch.
fn free_supply_led(delay: f64) -> Program {
    let mut p = program(15, 0, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 100.0),
        hc14(2, 2, 0, [0, 3, 0, 10, 0, 11, 0, 12, 0, 13, 0, 14], 0, delay),
        resistor(3, 3, 4, 330.0),
        red_led(4, 4, 0),
    ]);
    p.digital_slot_count = 6;
    p
}

#[test]
fn a_74hc14_on_a_supply_fed_through_100_ohm_driving_r_plus_led_converges_and_bypasses_the_cache() {
    let mut e = load(&free_supply_led(0.0));
    // The fixed point of "R_out follows vS": bisection on the supply voltage
    // (drled e2b-values2.mjs), R_out = clamp(vS / (4 * 0.02), 50, 1000).
    // R_out follows the supply, and each iteration of that feedback shrinks
    // the error only by about 0.02 (100 ohm x 6.7 mA x 12.5 ohm/V over the
    // 384 ohm chain), so one polish leaves the published iterate ~1e-7 V from
    // the fixed point (measured: 1.2e-7 V on the output pin, the worst node).
    // That is 1000x inside W0's tolerance and 40x tighter than the unpolished
    // iterate (5.5e-6 V on the same pin, `the_polish_tightens_the_combined_loop`).
    let (fp_v, fp_i) = (5e-7, 2e-9);
    assert_close("supply", e.voltages()[2], 4.327401881089489, fp_v);
    assert_close("anode", e.voltages()[4], 1.7440027947231074, fp_v);
    assert_close("output pin", e.voltages()[3], 3.963576585699746, fp_v);
    assert_close("LED current", e.currents()[4], 0.006725981181289687, fp_i);
    assert!(e.stat(11) >= 2.0);
    // A partition holding a free IC supply never reads or writes the cache,
    // whatever else it holds: re-solving it with nothing changed is a miss.
    let iterations = e.stat(11);
    e.advance(1e-3).unwrap();
    assert_eq!(e.stat(7), 0.0);
    assert_eq!(e.stat(11), iterations, "quiet: nothing re-solved it");
}

#[test]
fn the_polish_tightens_the_combined_loop() {
    let worst = |e: &Engine| {
        [(2, 4.327401881089489), (3, 3.963576585699746), (4, 1.7440027947231074)].iter().map(|&(n, want)| (e.voltages()[n] - want).abs()).fold(0.0f64, f64::max)
    };
    let polished = worst(&load(&free_supply_led(0.0)));
    let unpolished = {
        let _guard = hooks::skip_polish();
        worst(&load(&free_supply_led(0.0)))
    };
    assert!(unpolished > 10.0 * polished, "unpolished {unpolished:e} V vs polished {polished:e} V");
}

#[test]
fn a_dirty_free_supply_partition_with_a_junction_re_iterates_every_time_and_never_hits() {
    // The same circuit with a switch in the LED branch, so a repeated
    // `set_params` re-solves the partition with an unchanged epoch.
    let mut p = free_supply_led(0.0);
    p.devices.push(switch_dev(5, 4, 15, 1.0));
    p.devices[4] = red_led(4, 15, 0);
    p.node_count = 16;
    // the switch joins node 4 to node 15, the LED anode.
    let mut e = load(&p);
    let before = e.stat(11);
    e.set_params(&[(5, 0, 1.0)]).unwrap();
    e.set_params(&[(5, 0, 1.0)]).unwrap();
    assert_eq!(e.stat(7), 0.0, "no hit, even though the key repeats");
    assert!(e.stat(11) > before, "each re-solve iterated again");
}

// ============================================================================
// Stepped partitions with junctions (LEDs point 8), against an RK45 sidecar.
// ============================================================================

/// `5 V -- switch (1 mohm) -- n2 -- 1 kohm -- n3`, `n3` to the datum through
/// 100 uF and a red LED: the capacitor makes the partition stepped, the LED
/// makes it nonlinear. Device 1 is the switch. `trap` picks the integrator.
fn rc_fed_led(trap: bool) -> Program {
    let mut p = program(4, 0, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 1, 2, 1.0),
        resistor(2, 2, 3, 1000.0),
        capacitor(3, 3, 0, 100e-6),
        red_led(4, 3, 0),
    ]);
    p.options.integration = if trap { 1 } else { 0 };
    p
}

fn rc_fed_led_follows_the_rk45_reference(trap: bool) {
    let mut e = load(&rc_fed_led(trap));
    assert_eq!(e.stat(0), 1.0);
    assert_eq!(e.stat(8), 0.0, "never closed-form");
    // Charging from vc = 0 (README W1b point 5), then the sidecar's own ODE:
    // 5 V through 1.000001 kohm, the LED law, GMIN and every node shunt.
    // W0's voltage tolerance is 100 uV + 1e-3 relative. The trapezoidal
    // controller stays inside it at every instant here (worst 0.13 mV of a
    // 1.7 mV allowance). Backward Euler is first order: the step-doubling
    // controller bounds each step's error, not their sum, and mid-transient
    // it drifts to 3.4x that tolerance (measured 3.4 mV at 0.02 s, 2.5 mV at
    // 10 ms after opening), which is the same controller classic runs, so
    // BE gets 5x and the settled instants are held to the plain tolerance.
    let slack = if trap { 1.0 } else { 5.0 };
    let tol = |v: f64| slack * (100e-6 + 1e-3 * v.abs());
    let settled_tol = |v: f64| 100e-6 + 1e-3 * v.abs();
    for (t, want, settled) in [(0.02, 0.9063454088471131, false), (0.05, 1.7070668766329435, false), (0.1, 1.70729987247632, true)] {
        e.advance(t).unwrap();
        let allowed = if settled { settled_tol(want) } else { tol(want) };
        assert_close(&format!("charging at {t}"), e.voltages()[3], want, allowed);
    }
    e.advance(0.25).unwrap();
    assert_close("settled", e.voltages()[3], 1.7072998724828414, 1e-9);
    // Open the switch: the capacitor now discharges through the LED alone.
    e.set_params(&[(1, 0, 0.0)]).unwrap();
    for (dt, want) in [(0.001, 1.6818551586752215), (0.01, 1.6043988495728865), (0.05, 1.5275699998047132), (0.2, 1.457514937255462)] {
        e.advance(0.25 + dt).unwrap();
        assert_close(&format!("discharging {dt} s after opening"), e.voltages()[3], want, tol(want));
    }
    assert!(e.stat(10) > 0.0, "it stepped");
    assert!(e.stat(11) >= e.stat(10), "every step ran Newton");
}

#[test]
fn f10_an_rc_fed_led_under_backward_euler_follows_the_rk45_reference() {
    rc_fed_led_follows_the_rk45_reference(false);
}

#[test]
fn f10_an_rc_fed_led_under_trapezoidal_follows_the_rk45_reference() {
    rc_fed_led_follows_the_rk45_reference(true);
}

// ============================================================================
// Failure and rollback (LEDs points 6.3 and 16, W1b point 7).
// ============================================================================

#[test]
fn a_static_partition_that_cannot_converge_fails_the_toggle_with_minus_4_and_changes_nothing() {
    // An rgb LED on a switched supply: its channels and elementI are
    // published state that the failed instant must leave alone.
    let p = program(5, 3, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 1, 2, 1.0),
        resistor(2, 2, 3, 150.0),
        resistor(3, 2, 4, 150.0),
        rgb(4, 3, 4, 0, 0, 0, 0.0),
    ]);
    let mut e = load(&p);
    let (voltages, currents, channels) = (e.voltages().to_vec(), e.currents().to_vec(), e.channel_slots().to_vec());
    assert!(channels[0] > 1e-3);
    {
        let _guard = hooks::force_unconverged(u32::MAX);
        let err = e.set_params(&[(1, 0, 0.0)]).unwrap_err();
        assert_eq!(err.code, ERR_SOLVE_FAILED);
        assert!(err.message.contains("did not converge"), "{}", err.message);
    }
    assert_eq!(e.voltages(), voltages.as_slice(), "voltages rolled back");
    assert_eq!(e.currents(), currents.as_slice(), "currents rolled back");
    assert_eq!(e.channel_slots(), channels.as_slice(), "channels rolled back");
    // The toggle was undone too: the engine still works, and now succeeds.
    e.set_params(&[(1, 0, 0.0)]).unwrap();
    assert!(e.channel_slots()[0] < 1e-9);
}

#[test]
fn a_failed_advance_reports_the_last_completed_instant_and_keeps_its_readouts() {
    // The clocked circuit again, with the LED behind gate 1 an rgb's red
    // channel, so a channel value is state a failed instant must not move.
    // Uncached, so every level change runs the Newton solve the hook breaks
    // (with the cache, both levels were stored in the first nanoseconds and
    // every later flip would just replay them).
    let mut p = clocked_led(true);
    p.channel_slot_count = 3;
    p.devices[6] = rgb(6, 5, 0, 0, 0, 0, 0.0);
    p.devices[6].pins[1] = open_pin();
    p.devices[6].pins[2] = open_pin();
    let mut e = load(&p);
    e.advance(0.5e-3).unwrap();
    let mut control = load(&p);
    let err = {
        let _guard = hooks::force_unconverged(u32::MAX);
        e.advance(1.5e-3).unwrap_err()
    };
    assert_eq!(err.code, ERR_SOLVE_FAILED);
    let reached = e.time();
    assert!(reached >= 0.5e-3 && reached < 1.5e-3, "the clock reports the last completed instant, not the requested one: {reached}");
    control.advance(reached).unwrap();
    assert_eq!(e.channel_slots(), control.channel_slots(), "the channels are those of the last completed instant");
    for node in [4, 5] {
        assert_eq!(e.voltages()[node].to_bits(), control.voltages()[node].to_bits(), "node {node}");
    }
}

#[test]
fn the_channel_array_is_part_of_the_dynamic_snapshot() {
    let p = program(5, 3, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 1, 2, 1.0),
        resistor(2, 2, 3, 150.0),
        resistor(3, 2, 4, 150.0),
        rgb(4, 3, 4, 0, 0, 0, 0.0),
    ]);
    let mut solver = PartitionedSolver::build(&p).unwrap();
    let all: Vec<crate::events::PartitionId> = (0..solver.partition_count() as u32).map(crate::events::PartitionId).collect();
    solver.resolve_dirty(&all, 0.0, &[]).unwrap();
    solver.prepare_readout();
    let lit = solver.channels().to_vec();
    assert!(lit[0] > 1e-3);
    let snapshot = (solver.snapshot(), solver.dynamic_snapshot());
    let touched = solver.apply_params(&[(1, 0, 0.0)]);
    solver.resolve_dirty(&touched, 0.0, &[]).unwrap();
    solver.prepare_readout();
    assert!(solver.channels()[0] < 1e-9, "the toggle darkened it");
    solver.restore(snapshot.0);
    solver.dynamic_restore(snapshot.1);
    solver.prepare_readout();
    assert_eq!(solver.channels(), lit.as_slice());
}

#[test]
fn the_engine_copies_an_rgb_leds_channels_into_its_slots_after_every_event_that_moves_them() {
    let p = program(5, 3, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 1, 2, 0.0),
        resistor(2, 2, 3, 150.0),
        resistor(3, 2, 4, 150.0),
        rgb(4, 3, 4, 0, 0, 0, 0.0),
    ]);
    let mut e = load(&p);
    assert_eq!(e.channel_slots(), &[0.0, 0.0, 0.0], "open: dark, and readable straight after load");
    e.set_params(&[(1, 0, 1.0)]).unwrap();
    // The switch adds its 1 mohm contact to each 150 ohm leg.
    assert_close("r after the toggle", e.channel_slots()[0], 0.021311574100651327, 1e-6);
    assert_close("g after the toggle", e.channel_slots()[1], 0.02, 1e-6);
    assert_eq!(e.channel_slots()[2], 0.0);
}
