//! The load-time guard of a bench supply's current-limit rail the core reads
//! wrong (`rail_guard`, mechanism M3 of the current-limit diagnosis). A
//! default-mode `bench_psu` hub that has a rail capacitor (to a fixed node), an
//! esr-0 capacitor from the rail into a stepped hub partition, an esr-0
//! capacitor loop through the rail and a closed-form hub partition declines at
//! load with `CC_RAIL_REASON` (a `-4`); every board short of one of those
//! ingredients, an unlimited supply, a battery rail and exact rail mode load.
//!
//! The minimal board is the diagnosis's (ship-logs/ccd, `m3truth.py`): in
//! current limit the core reads its rail 0.0295 V at 10 us against a true
//! 2.6135 V, and a capacitor current of millions of amps on a larger board.
//! The board the broad form of this guard declined by mistake is the stepped
//! variant of `rail_tests`'s coupled-closed-form test: a supply in current
//! limit the core reads within 1e-4 V of the closed form.

use super::*;
use crate::engine::Engine;
use crate::error::ERR_SOLVE_FAILED;
use crate::program::{Device, Options, PinRecord};

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn psu(ci: usize, pos: usize, neg: usize, v: f64, i_limit: f64) -> Device {
    Device { kind: kinds::BENCH_PSU, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v, i_limit], blob: vec![] }
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
}

fn capacitor(ci: usize, a: usize, b: usize, c: f64, esr: f64) -> Device {
    Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![c, esr, 0.0], blob: vec![] }
}

fn program(node_count: usize, devices: Vec<Device>) -> Program {
    Program { carry: Vec::new(), node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
}

fn exact(mut p: Program) -> Program {
    p.options.no_rail_shortcut = true;
    p
}

/// The diagnosis's minimal M3 board: the supply on node 1 (5.164233 V,
/// 0.66476 A), C10 rail-ground, C15 rail-n3 and C50 n3-ground (n3 holds two
/// capacitors: a stepped hub partition), C21 rail-n2 and R20 n2-ground (one:
/// a closed-form hub partition). `esr_rail` is C10's esr, `esr_feed` C15's.
fn m3_with(esr_rail: f64, esr_feed: f64) -> Program {
    program(
        4,
        vec![
            psu(0, 1, 0, 5.164233, 0.66476),
            capacitor(1, 1, 0, 1.857e-6, esr_rail),
            capacitor(2, 1, 3, 78.12e-6, esr_feed),
            capacitor(3, 3, 0, 0.6926e-6, 0.0),
            capacitor(4, 2, 1, 0.1388e-6, 0.0),
            resistor(5, 2, 0, 384631.0),
        ],
    )
}

fn m3_minimal() -> Program {
    m3_with(0.0, 0.0)
}

/// A stepped hub partition that hangs off the rail through a resistor (no
/// capacitor from the rail into it), esr > 0 on the rail capacitor: the shape
/// the broad form declined and the narrow form does not (a known gap if the
/// core reads such a board wrong; the sweep's 274/3 and 418/5 are missed too).
fn stepped_behind_a_resistor() -> Program {
    program(
        4,
        vec![
            psu(0, 1, 0, 5.0, 0.03),
            capacitor(1, 1, 0, 1e-6, 0.5),
            resistor(2, 1, 3, 100.0),
            capacitor(3, 3, 0, 1e-6, 0.0),
            capacitor(4, 3, 0, 2.2e-6, 0.0),
            resistor(5, 1, 2, 220.0),
            capacitor(6, 2, 0, 3.3e-6, 0.0),
        ],
    )
}

fn without(mut p: Program, device: usize) -> Program {
    p.devices.remove(device);
    p
}

fn declined(p: &Program) -> BoError {
    match Engine::load(p) {
        Err(e) => e,
        Ok(_) => panic!("the board loaded"),
    }
}

fn assert_declines(p: &Program, what: &str) {
    let err = declined(p);
    assert_eq!(err.code, ERR_SOLVE_FAILED, "{what}: {}", err.message);
    assert!(err.message.starts_with(CC_RAIL_REASON), "{what}: {}", err.message);
    let built = PartitionedSolver::build(p).err().expect("the solver build declines too");
    assert!(built.message.starts_with(CC_RAIL_REASON), "{what}: {}", built.message);
}

fn assert_loads(p: &Program, what: &str) {
    if let Err(e) = Engine::load(p) {
        panic!("{what}: declined: {} {}", e.code, e.message);
    }
}

#[test]
fn the_minimal_m3_board_declines_at_load_with_its_own_reason() {
    assert_declines(&m3_minimal(), "minimal M3");
}

#[test]
fn the_decline_says_what_the_board_has_that_the_core_misreads_and_that_classic_runs_it() {
    let m = declined(&m3_minimal()).message;
    assert!(m.contains("bench_psu (device 0)"), "{m}");
    assert!(m.contains("rail capacitor"), "{m}");
    assert!(m.contains("esr-0 capacitor loop through the rail"), "{m}");
    assert!(m.contains("stepped"), "{m}");
    assert!(m.contains("closed-form"), "{m}");
    assert!(m.contains("current limit"), "{m}");
    assert!(m.contains("classic"), "{m}");
}

#[test]
fn a_supply_in_current_limit_the_core_reads_right_loads_whether_its_partition_is_stepped_or_closed_form() {
    // `an_rc_driven_by_an_ic_across_its_power_floor_follows_the_coupled_closed_form` holds the core
    // within 1e-4 V of the coupled closed form on both variants. The stepped one has a rail
    // capacitor, a stepped hub partition and a closed-form one, and no capacitor from the rail
    // into the stepped partition: the broad form declined it, sending it to classic's backward
    // Euler, 3.4 W0 off that closed form.
    for stepped in [false, true] {
        assert_loads(&super::rail_tests::an_rc_driven_by_an_ic_across_its_power_floor(stepped), if stepped { "coupled closed form, stepped" } else { "coupled closed form" });
    }
}

#[test]
fn a_stepped_partition_fed_through_a_resistor_loads() {
    // The broad form declined this; the narrow form needs the capacitor from the rail into the
    // stepped partition (a known gap of the guard, not a claim that the core reads it right).
    assert_loads(&stepped_behind_a_resistor(), "stepped behind a resistor");
}

#[test]
fn the_narrow_form_needs_the_stepped_partitions_rail_capacitor_to_be_esr_0() {
    assert_declines(&m3_with(0.0, 0.0), "esr 0 on both");
    assert_loads(&m3_with(0.0, 0.5), "esr on the capacitor into the stepped partition");
}

#[test]
fn the_narrow_form_needs_the_esr_0_loop_through_the_rail() {
    // C10 (rail-ground) with an esr leaves C15 (rail-n3) and C50 (n3-ground) esr 0 but no closed
    // loop of esr-0 capacitors through the rail.
    assert_loads(&m3_with(0.5, 0.0), "no esr-0 loop through the rail");
}

#[test]
fn exact_rail_mode_runs_both_boards_it_is_right_on_them() {
    assert_loads(&exact(m3_minimal()), "exact minimal");
    assert_loads(&exact(stepped_behind_a_resistor()), "exact resistor-fed");
}

#[test]
fn a_board_short_of_any_one_ingredient_loads() {
    // No rail capacitor (C10).
    assert_loads(&without(m3_minimal(), 1), "no rail capacitor");
    // No closed-form hub partition (C21 and R20).
    let mut p = without(m3_minimal(), 5);
    p = without(p, 4);
    assert_loads(&p, "no closed-form partition");
    // No stepped hub partition (C15, C50).
    let mut p = without(m3_minimal(), 3);
    p = without(p, 2);
    assert_loads(&p, "no stepped partition");
    // The same with the resistor-fed shape.
    assert_loads(&without(stepped_behind_a_resistor(), 1), "resistor-fed, no rail capacitor");
    let mut p = without(stepped_behind_a_resistor(), 6);
    p = without(p, 5);
    assert_loads(&p, "resistor-fed, no closed-form partition");
}

#[test]
fn an_unlimited_supply_has_no_rail_and_loads() {
    let mut p = m3_minimal();
    p.devices[0].params[1] = 0.0;
    assert_loads(&p, "iLimit 0");
}

#[test]
fn only_the_bench_supplys_rail_is_guarded() {
    use crate::kinds::battery_param;
    let mut params = vec![0.0; battery_param::COUNT];
    params[battery_param::OCV_SCALE] = 1.0;
    params[battery_param::R_BASE] = 0.1;
    params[battery_param::TEMP_FACTOR] = 1.0;
    params[battery_param::DEP_P] = 2.0;
    params[battery_param::DEP_K] = 0.05;
    params[battery_param::CAPACITY_AH] = 1.0;
    params[battery_param::SOC0] = 1.0;
    let mut p = m3_minimal();
    p.devices[0] = Device { kind: kinds::BATTERY_PACK, component_index: 0, pins: vec![pin(1), pin(0)], params, blob: vec![1.0, 0.5, 5.0] };
    assert_loads(&p, "a battery rail");
    let mut p = m3_minimal();
    p.devices[0] = Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![5.0], blob: vec![] };
    assert_loads(&p, "an ideal source");
}
