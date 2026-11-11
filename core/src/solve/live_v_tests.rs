//! W2c piece 5: a voltage source's `V` set live (ruling R57 D12 (a); core/
//! README.md, "Live V"). `bo_engine_set_params` on a `voltage_source`'s
//! param 0 is a source edge on its fixed node at the engine's time: every
//! node the source fixes, and every source stacked on it, takes its new
//! value; what reads them re-solves as at any discontinuity. Each case is
//! held to classic's answer to the same edit, a carrying reload (W2c's seed):
//! digital identical, analog within W0.

use super::slow_tests::ic;
use crate::carry;
use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn dev(kind: u32, ci: usize, a: usize, b: usize, params: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: vec![pin(a), pin(b)], params, blob: vec![] }
}

fn program(node_count: usize, digital: usize, devices: Vec<Device>) -> Program {
    Program { carry: Vec::new(), node_count, options: Options::default(), digital_slot_count: digital, channel_slot_count: 0, devices }
}

fn w0_volts(got: f64, want: f64) -> bool {
    (got - want).abs() <= 1e-4 + 1e-3 * got.abs().max(want.abs())
}

fn w0_amps(got: f64, want: f64) -> bool {
    (got - want).abs() <= 1e-9 + 1e-3 * got.abs().max(want.abs())
}

/// `p` run to `t_set`, source `src` set to `v` there, and run on through
/// `after`; against the same run reloaded at `t_set` onto `p` with `v`,
/// carrying. Digital slots bit for bit, voltages and currents within W0.
fn live_against_reload(name: &str, p: &Program, src: usize, v: f64, t_set: f64, after: &[f64]) {
    live_param_against_reload(name, p, src, 0, v, t_set, after);
}

/// [`live_against_reload`] for any live param.
fn live_param_against_reload(name: &str, p: &Program, device: usize, param: usize, v: f64, t_set: f64, after: &[f64]) {
    let mut live = Engine::load(p).unwrap();
    live.advance(t_set).unwrap();
    let carry = live.carry_take().unwrap().to_vec();
    live.set_params(&[(device as u32, param as u32, v)]).unwrap_or_else(|e| panic!("{name}: set: {} {}", e.code, e.message));
    let mut q = p.clone();
    q.devices[device].params[param] = v;
    q.options.start_time = t_set;
    q.carry = carry;
    let mut reload = Engine::load(&q).unwrap();
    for (k, &t) in std::iter::once(&t_set).chain(after).enumerate() {
        if k > 0 {
            live.advance(t).unwrap();
            reload.advance(t).unwrap();
        }
        assert_eq!(live.digital_slots().iter().map(|x| x.to_bits()).collect::<Vec<_>>(), reload.digital_slots().iter().map(|x| x.to_bits()).collect::<Vec<_>>(), "{name} at {t}: the slots");
        for (n, (&g, &w)) in live.voltages().iter().zip(reload.voltages()).enumerate() {
            assert!(w0_volts(g, w), "{name} at {t}: node {n} {g} against the reload's {w}");
        }
        for (d, (&g, &w)) in live.currents().iter().zip(reload.currents()).enumerate() {
            assert!(w0_amps(g, w), "{name} at {t}: device {d} {g} A against the reload's {w} A");
        }
    }
}

#[test]
fn a_live_voltage_runs_on_as_a_carrying_reload_onto_the_new_voltage() {
    // 5 V through 1k into 1 uF (closed form; stepped when forced), set to
    // 2 V at 2 ms, mid-charge.
    let rc = program(3, 0, vec![dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]), dev(kinds::RESISTOR, 1, 1, 2, vec![1000.0]), dev(kinds::CAPACITOR, 2, 2, 0, vec![1e-6, 0.0, 0.0])]);
    live_against_reload("rc", &rc, 0, 2.0, 2e-3, &[2e-3 + 1e-6, 2.5e-3, 4e-3, 10e-3]);
    let mut stepped = rc.clone();
    stepped.options.force_stepped = true;
    live_against_reload("rc stepped", &stepped, 0, 2.0, 2e-3, &[2e-3 + 1e-6, 2.5e-3, 4e-3, 10e-3]);
    // A source stacked on the edited one: 1 V on top of V, into the same RC.
    let stacked = program(4, 0, vec![
        dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]),
        dev(kinds::VOLTAGE_SOURCE, 1, 2, 1, vec![1.0]),
        dev(kinds::RESISTOR, 2, 2, 3, vec![1000.0]),
        dev(kinds::CAPACITOR, 3, 3, 0, vec![1e-6, 0.0, 0.0]),
    ]);
    live_against_reload("stacked", &stacked, 0, 3.0, 2e-3, &[2e-3 + 1e-6, 4e-3, 10e-3]);
    // A capacitor straight across the source (rule 2a), with esr: re-aimed.
    let pair = program(2, 0, vec![dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]), dev(kinds::CAPACITOR, 1, 1, 0, vec![1e-6, 10.0, 0.0]), dev(kinds::RESISTOR, 2, 1, 0, vec![100.0])]);
    live_against_reload("fixed pair", &pair, 0, 3.0, 1e-3, &[1e-3 + 1e-6, 1.01e-3, 2e-3]);
}

#[test]
fn a_live_voltage_reaches_an_ic_that_reads_it() {
    // A 10k/10k divider from the edited source into a 74LS04 (5 V of its
    // own), its output into 10k: 5 V reads 2.5 V (HIGH, output LOW); set to
    // 1 V it reads 0.5 V (LOW), so the output goes HIGH after its delay.
    let p = program(6, 6, vec![
        dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]),
        dev(kinds::RESISTOR, 1, 1, 2, vec![1e4]),
        dev(kinds::RESISTOR, 2, 2, 0, vec![1e4]),
        dev(kinds::VOLTAGE_SOURCE, 3, 3, 0, vec![5.0]),
        ic(kinds::IC_74LS04, 4, 3, 0, [2, 4, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5], 1e-8, 0),
        dev(kinds::RESISTOR, 5, 4, 0, vec![1e4]),
    ]);
    live_against_reload("74LS04", &p, 0, 1.0, 1e-3, &[1e-3 + 5e-9, 1e-3 + 2e-8, 2e-3]);
    let mut e = Engine::load(&p).unwrap();
    e.advance(1e-3).unwrap();
    assert!(e.voltages()[4] < 0.5, "LOW before");
    e.set_params(&[(0, 0, 1.0)]).unwrap();
    e.advance(1e-3 + 2e-8).unwrap();
    assert!(e.voltages()[4] > 2.0, "HIGH after the delay: {}", e.voltages()[4]);
}

#[test]
fn a_live_voltage_that_makes_a_cold_resistor_stressable_turns_it_hot() {
    // 10 ohm across the source, rated 3 W: at 5 V at most 2.5 W, so cold. Set
    // to 7 V it dissipates 4.9 W, stress rate 4.9 / 3 - 1, and latches that
    // long after the set: 1 / 0.6333 = 1.5789 s.
    let p = program(2, 0, vec![
        dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]),
        dev(kinds::RESISTOR, 1, 1, 0, vec![10.0]),
        Device { kind: kinds::RATING, component_index: 1, pins: vec![], params: vec![1.0, 3.0, 10.0], blob: vec![] },
    ]);
    let mut e = Engine::load(&p).unwrap();
    e.advance(1.0).unwrap();
    assert!(e.records(0).unwrap().is_empty());
    e.set_params(&[(0, 0, 7.0)]).unwrap();
    e.advance(4.0).unwrap();
    let r = e.records(0).unwrap().to_vec();
    assert_eq!(r.len(), 7, "latched");
    let hand = 1.0 + 1.0 / (4.9 / 3.0 - 1.0);
    assert!((r[3] - hand).abs() <= 1e-9 * hand, "at {} against {hand}", r[3]);
}

#[test]
fn a_live_voltage_is_validated_and_rolls_back_with_its_instant() {
    let p = program(3, 0, vec![dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]), dev(kinds::RESISTOR, 1, 1, 2, vec![1000.0]), dev(kinds::CAPACITOR, 2, 2, 0, vec![1e-6, 0.0, 0.0])]);
    let mut e = Engine::load(&p).unwrap();
    e.advance(1e-3).unwrap();
    assert_eq!(e.set_params(&[(0, 0, f64::NAN)]).unwrap_err().code, -8, "a non-finite V");
    assert_eq!(e.set_params(&[(0, 1, 3.0)]).unwrap_err().code, -8, "no param 1");
    assert_eq!(e.set_params(&[(1, 0, 3.0)]).unwrap_err().code, -8, "a resistor is still no live param");
    e.set_params(&[(0, 0, 3.0)]).unwrap();
    assert_eq!(e.voltages()[1], 3.0, "the node takes the new value at once");
    let _ = carry::VERSION;
}

#[test]
fn a_live_voltage_whose_instant_fails_rolls_back_whole() {
    // The source powers a 74LS04 whose 1Y feeds its own 1A with no delay:
    // unpowered at 0 V, a livelock once powered. The set's instant fails and
    // rolls back: the source, the node it fixes, the static solver's folded
    // base (a 1k divider from the source), a source stacked on it and
    // everything else as before, so
    // the run goes on as a twin's that never tried.
    let p = program(7, 6, vec![
        dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![0.0]),
        ic(kinds::IC_74LS04, 1, 1, 0, [2, 2, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5], 0.0, 0),
        dev(kinds::RESISTOR, 2, 5, 0, vec![1e4]),
        dev(kinds::RESISTOR, 3, 1, 3, vec![1e3]),
        dev(kinds::RESISTOR, 4, 3, 0, vec![1e3]),
        dev(kinds::VOLTAGE_SOURCE, 5, 4, 1, vec![1.0]),
        dev(kinds::SWITCH, 6, 3, 6, vec![0.0]),
        dev(kinds::RESISTOR, 7, 6, 0, vec![1e3]),
    ]);
    let mut e = Engine::load(&p).unwrap();
    let mut twin = Engine::load(&p).unwrap();
    e.advance(1e-3).unwrap();
    twin.advance(1e-3).unwrap();
    let before: Vec<u64> = e.voltages().iter().chain(e.currents()).map(|v| v.to_bits()).collect();
    let err = e.set_params(&[(0, 0, 5.0)]).unwrap_err();
    assert!(err.code == -7 || err.code == -4, "the instant fails: {} {}", err.code, err.message);
    assert_eq!(e.voltages().iter().chain(e.currents()).map(|v| v.to_bits()).collect::<Vec<u64>>(), before, "as before the set");
    // A switch in the divider re-solves its partition from its base.
    e.set_params(&[(6, 0, 1.0)]).unwrap();
    twin.set_params(&[(6, 0, 1.0)]).unwrap();
    for t in [1.5e-3, 3e-3] {
        e.advance(t).unwrap();
        twin.advance(t).unwrap();
        assert_eq!(e.voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), twin.voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), "at {t}");
    }
    // And a set that succeeds afterwards still moves the stacked node.
    e.set_params(&[(0, 0, 0.5)]).unwrap();
    assert!((e.voltages()[3] - 0.5 / 3.0).abs() < 1e-6 && (e.voltages()[4] - 1.5).abs() < 1e-12, "{:?}", e.voltages());
}

#[test]
fn each_switch_kind_set_live_runs_on_as_a_carrying_reload() {
    // The other live params, held to the same answer (the W2c plan's test
    // 4): each switch kind closing 1k from 5 V into 1 uF mid-charge of a
    // second 10k path, and a 74LS04 reading the capacitor, in default mode
    // and stepped.
    let board = |sw: Device| {
        program(6, 6, vec![
            dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]),
            dev(kinds::RESISTOR, 1, 1, 2, vec![1e4]),
            dev(kinds::CAPACITOR, 2, 2, 0, vec![1e-6, 0.0, 0.0]),
            sw,
            dev(kinds::RESISTOR, 4, 3, 2, vec![1e3]),
            ic(kinds::IC_74LS04, 5, 1, 0, [2, 4, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5], 1e-8, 0),
        ])
    };
    let cases: Vec<(&str, Device, usize, f64)> = vec![
        ("switch", dev(kinds::SWITCH, 3, 1, 3, vec![0.0]), 0, 1.0),
        ("push_button", dev(kinds::PUSH_BUTTON, 3, 1, 3, vec![0.0]), 0, 1.0),
        ("spdt_switch", Device { kind: kinds::SPDT_SWITCH, component_index: 3, pins: vec![pin(1), pin(0), pin(3)], params: vec![0.0], blob: vec![] }, 0, 1.0),
        ("dip_switch", Device { kind: kinds::DIP_SWITCH, component_index: 3, pins: vec![pin(1), pin(3), pin(0), pin(0)], params: vec![2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], blob: vec![] }, 1, 1.0),
    ];
    for (name, sw, param, v) in cases {
        let p = board(sw);
        live_param_against_reload(name, &p, 3, param, v, 3e-3, &[3e-3 + 2e-8, 3e-3 + 1e-6, 4e-3, 8e-3]);
        let mut stepped = p.clone();
        stepped.options.force_stepped = true;
        live_param_against_reload(&format!("{name} stepped"), &stepped, 3, param, v, 3e-3, &[3e-3 + 2e-8, 3e-3 + 1e-6, 4e-3, 8e-3]);
    }
}
