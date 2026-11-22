//! The load-time guard of a bench supply's current-limit rail the core reads
//! wrong (`rail_guard`, mechanism M3 of the current-limit diagnosis). A
//! default-mode `bench_psu` hub that has a rail capacitor and any stepped hub
//! partition on its rail, beside a closed-form hub partition, declines at load
//! with `CC_RAIL_REASON` (a `-4`); every board short of one of the four
//! ingredients, an unlimited supply, a battery rail and exact rail mode load.
//!
//! The minimal board is the diagnosis's (ship-logs/ccd, `m3truth.py`): in
//! current limit the core reads its rail 0.0295 V at 10 us against a true
//! 2.6135 V, and a capacitor current of millions of amps on a larger board.

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
/// a closed-form hub partition).
fn m3_minimal() -> Program {
    program(
        4,
        vec![
            psu(0, 1, 0, 5.164233, 0.66476),
            capacitor(1, 1, 0, 1.857e-6, 0.0),
            capacitor(2, 1, 3, 78.12e-6, 0.0),
            capacitor(3, 3, 0, 0.6926e-6, 0.0),
            capacitor(4, 2, 1, 0.1388e-6, 0.0),
            resistor(5, 2, 0, 384631.0),
        ],
    )
}

/// The broad form's other shape: the stepped partition hangs off the rail
/// through a resistor (no capacitor from the rail into it), esr > 0 on the
/// rail capacitor.
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
    assert!(m.contains("current limit"), "{m}");
    assert!(m.contains("classic"), "{m}");
}

#[test]
fn any_stepped_hub_partition_beside_a_closed_form_one_and_a_rail_capacitor_declines() {
    assert_declines(&stepped_behind_a_resistor(), "stepped behind a resistor");
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
