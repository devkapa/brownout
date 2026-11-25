//! CAPERR4V (V5): bit hashes of boards the change must not touch (junction, hub battery with a rail capacitor,
//! exact mode), and of a linear ladder it must (control). Run on this branch and on wip/v1: the hashes of the
//! first three must be equal. Uses nothing the change added.

use crate::engine::Engine;
use crate::kinds::battery_param;
use crate::program::{Device, Options, PinRecord};
use crate::solve::{kinds, Program};

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}
fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob: vec![] }
}
fn battery(ci: usize, pos: usize, neg: usize, e: f64, r: f64) -> Device {
    let mut params = vec![0.0; battery_param::COUNT];
    params[battery_param::OCV_SCALE] = 1.0;
    params[battery_param::R_BASE] = r;
    params[battery_param::TEMP_FACTOR] = 1.0;
    params[battery_param::DEP_P] = 2.0;
    params[battery_param::DEP_K] = 0.05;
    params[battery_param::CAPACITY_AH] = 1.0;
    params[battery_param::SOC0] = 1.0;
    Device { kind: kinds::BATTERY_PACK, component_index: ci, pins: vec![pin(pos), pin(neg)], params, blob: vec![1.0, 0.5, e] }
}
fn prog(node_count: usize, devices: Vec<Device>, f: impl Fn(&mut Options)) -> Program {
    let mut options = Options::default();
    f(&mut options);
    Program { carry: Vec::new(), node_count, options, digital_slot_count: 0, channel_slot_count: 0, devices }
}

fn hash_run(p: &Program, frames: &[f64]) -> u64 {
    let mut e = Engine::load(p).unwrap();
    let mut h: u64 = 0xcbf29ce484222325;
    let mut mix = |x: f64| {
        h ^= x.to_bits();
        h = h.wrapping_mul(0x100000001b3);
    };
    for &t in frames {
        e.advance(t).unwrap();
        e.voltages().iter().for_each(|&x| mix(x));
        e.currents().iter().for_each(|&x| mix(x));
    }
    h
}

#[test]
fn v5_hashes() {
    let mut frames: Vec<f64> = vec![];
    let mut t = 0.0;
    for k in 0..80 {
        t += [1e-3, 0.37e-3, 2.5e-3, 1e-5, 4e-4][k % 5];
        frames.push(t);
    }
    // junction: 5 V into 100 ohm, 1 uF, an LED on the cap node (a stepped partition with a junction)
    let led = prog(3, vec![dev(kinds::VOLTAGE_SOURCE, 0, &[1, 0], vec![5.0]), dev(kinds::RESISTOR, 1, &[1, 2], vec![100.0]), dev(kinds::CAPACITOR, 2, &[2, 0], vec![1e-6, 0.0, 0.0]), dev(kinds::LED, 3, &[2, 0], vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0])], |_| {});
    // hub: a cell with a rail capacitor and an RC load
    let hub = prog(4, vec![battery(0, 1, 0, 5.0, 1.0), dev(kinds::CAPACITOR, 1, &[1, 0], vec![1e-5, 0.0, 0.0]), dev(kinds::RESISTOR, 2, &[1, 2], vec![1000.0]), dev(kinds::CAPACITOR, 3, &[2, 0], vec![1e-6, 0.0, 0.0]), dev(kinds::RESISTOR, 4, &[1, 3], vec![5.0]), dev(kinds::CAPACITOR, 5, &[3, 0], vec![2e-6, 0.0, 0.0])], |_| {});
    // exact mode: the 5 ohm ladder with the rail shortcut off
    let ladder = |f: fn(&mut Options)| {
        prog(
            4,
            vec![dev(kinds::VOLTAGE_SOURCE, 0, &[1, 0], vec![3.3]), dev(kinds::RESISTOR, 1, &[1, 2], vec![5.0]), dev(kinds::RESISTOR, 2, &[2, 0], vec![1000.0]), dev(kinds::RESISTOR, 3, &[2, 3], vec![1000.0]), dev(kinds::CAPACITOR, 4, &[2, 0], vec![1e-6, 0.0, 0.0]), dev(kinds::CAPACITOR, 5, &[3, 0], vec![1e-6, 0.0, 0.0])],
            f,
        )
    };
    eprintln!("V5HASH junction {:016x}", hash_run(&led, &frames));
    eprintln!("V5HASH hub {:016x}", hash_run(&hub, &frames));
    eprintln!("V5HASH exact-ladder {:016x}", hash_run(&ladder(|o| o.no_rail_shortcut = true), &frames));
    eprintln!("V5HASH control-ladder {:016x}", hash_run(&ladder(|_| {}), &frames));
}
