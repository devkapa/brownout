//! W2b piece 1: the observer on fixed, static and closed-form nodes (core/
//! README.md, "Acquisition"; rulings R57 D1-D2 (a), R104). A sample is the
//! model's value at its grid time, read off what the solver holds, so a
//! probed run is the unprobed run bit for bit, a closed-form node's samples
//! are its exact curve, and a sample on an instant reads after it.

use crate::engine::Engine;
use crate::kinds;
use crate::observe::TAKE_HEADER;
use crate::program::{Device, Options, PinRecord, Program};

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn dev(kind: u32, ci: usize, a: usize, b: usize, params: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: vec![pin(a), pin(b)], params, blob: vec![] }
}

fn program(node_count: usize, devices: Vec<Device>) -> Program {
    Program { carry: Vec::new(), node_count, options: Options::default(), digital_slot_count: 2, channel_slot_count: 0, devices }
}

/// A waveform source: its blob holds its digital slot.
fn wave(kind: u32, ci: usize, a: usize, slot: usize, params: Vec<f64>) -> Device {
    Device { blob: vec![slot as f64], ..dev(kind, ci, a, 0, params) }
}

/// 5 V through 1k into 1 uF, every node shunted by `NODE_RSHUNT_G` (1e-12
/// S): the capacitor (node 2) at `v (1 - e^(-t / tau))`, `v = 5 / (1 + R g)`,
/// `tau = R C / (1 + R g)`.
fn rc() -> Program {
    program(3, vec![dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]), dev(kinds::RESISTOR, 1, 1, 2, vec![1000.0]), dev(kinds::CAPACITOR, 2, 2, 0, vec![1e-6, 0.0, 0.0])])
}

/// A 1 kHz clock (5 V high, half duty) through 1k into 1 uF and through a
/// 2k/10k divider, and a 1 ms pulse ramp (0 to 5 V from 0.25 ms) into 2k: a
/// fixed node with edges, a closed-form node re-aimed at each, a static node
/// that steps with them, and a ramping fixed node.
fn edges() -> Program {
    program(6, vec![
        wave(kinds::CLOCK, 0, 1, 0, vec![1000.0, 0.5, 5.0, 0.0, 0.0]),
        dev(kinds::RESISTOR, 1, 1, 2, vec![1000.0]),
        dev(kinds::CAPACITOR, 2, 2, 0, vec![1e-6, 0.0, 0.0]),
        wave(kinds::PULSE_SOURCE, 3, 3, 1, vec![0.0, 5.0, 0.25e-3, 1e-3, 1e-3, 2e-3, 10e-3]),
        dev(kinds::RESISTOR, 4, 3, 0, vec![2000.0]),
        dev(kinds::RESISTOR, 5, 1, 4, vec![2000.0]),
        dev(kinds::RESISTOR, 6, 4, 0, vec![10000.0]),
    ])
}

struct Take {
    overflowed: bool,
    dropped: f64,
    max_span: f64,
    t: Vec<f64>,
    /// `v[j][s]`: probed node `j` at sample `s`.
    v: Vec<Vec<f64>>,
}

fn take(e: &mut Engine, n: usize) -> Take {
    let b = e.acquire_take().to_vec();
    let count = b[0] as usize;
    assert_eq!(b.len(), TAKE_HEADER + count * (1 + n), "the take's length");
    let t = b[TAKE_HEADER..TAKE_HEADER + count].to_vec();
    let v = (0..n).map(|j| b[TAKE_HEADER + count * (1 + j)..TAKE_HEADER + count * (2 + j)].to_vec()).collect();
    Take { overflowed: b[2] != 0.0, dropped: b[3], max_span: b[1], t, v }
}

/// Runs `p` with the observer on `nodes` (10 kHz unless `interval`) through
/// the host steps `steps`, taking after each; every sample in order.
fn acquire(p: &Program, nodes: &[u32], interval: f64, steps: &[f64]) -> (Vec<f64>, Vec<Vec<f64>>) {
    let mut e = Engine::load(p).unwrap();
    e.acquire(nodes, 0.0, interval, 0).unwrap();
    let (mut t, mut v) = (Vec::new(), vec![Vec::new(); nodes.len()]);
    for &s in steps {
        e.advance(s).unwrap();
        let k = take(&mut e, nodes.len());
        assert!(!k.overflowed && k.dropped == 0.0 && k.max_span == 0.0);
        t.extend(k.t);
        for (j, col) in k.v.into_iter().enumerate() {
            v[j].extend(col);
        }
    }
    (t, v)
}

/// Host steps that land on no grid time, on some, and on instants.
const STEPS: [f64; 7] = [0.37e-3, 1e-3, 1.5e-3, 2.05e-3, 1.0 / 60.0 * 0.2, 4e-3, 5e-3];

#[test]
fn a_closed_form_node_samples_its_exact_curve() {
    let (t, v) = acquire(&rc(), &[2, 1], 1e-4, &STEPS);
    assert_eq!(t.len(), 51, "0, 0.1 ms, .. 5 ms");
    for (s, &ts) in t.iter().enumerate() {
        assert_eq!(ts, s as f64 * 1e-4, "the grid is origin + k * interval, never summed");
        let rg = 1000.0 * super::static_part::NODE_RSHUNT_G;
        let hand = 5.0 / (1.0 + rg) * (1.0 - (-ts / (1e-3 / (1.0 + rg))).exp());
        assert!((v[0][s] - hand).abs() <= 1e-12 * 5.0, "at {ts}: {} against {hand}", v[0][s]);
        assert_eq!(v[1][s], 5.0);
    }
}

#[test]
fn a_sample_is_the_readout_an_advance_to_its_time_gives() {
    // Each sample against a twin advanced to exactly that time (the clamp's
    // readout): the clock's edges at every 0.5 ms read after the edge (D2),
    // the capacitor is continuous through them, the ramp and the static node
    // it feeds follow it between instants.
    let nodes = [1, 2, 3, 4];
    let (t, v) = acquire(&edges(), &nodes, 1e-4, &STEPS);
    assert_eq!(t.len(), 51);
    for (s, &ts) in t.iter().enumerate() {
        let mut twin = Engine::load(&edges()).unwrap();
        twin.advance(ts).unwrap();
        for (j, &n) in nodes.iter().enumerate() {
            let want = twin.voltages()[n as usize];
            assert!((v[j][s] - want).abs() <= 1e-12 * (1.0 + want.abs()), "node {n} at {ts}: {} against the readout's {want}", v[j][s]);
        }
    }
    // The clock's edges fall on the grid: 5 x 0.1 ms is 0.5 ms exactly.
    assert_eq!((t[5], t[10]), (0.5e-3, 1e-3));
    assert_eq!(v[0][5], 0.0, "the falling edge at 0.5 ms reads low");
    assert_eq!(v[0][10], 5.0, "the rising edge at 1 ms reads high");
    assert_eq!(v[3][5].abs(), 0.0, "the divider stepped down with it");
    assert!((v[3][10] - 5.0 * 10.0 / 12.0).abs() <= 1e-6, "and up: {}", v[3][10]);
    let ramp = 5.0 * (t[7] - 0.25e-3) / 1e-3;
    assert!((v[2][7] - ramp).abs() <= 1e-12, "the ramp at 0.7 ms: {} against {ramp}", v[2][7]);
}

#[test]
fn a_probed_run_is_the_unprobed_run_bit_for_bit() {
    for (name, p) in [("rc", rc()), ("edges", edges())] {
        let mut probed = Engine::load(&p).unwrap();
        let mut plain = Engine::load(&p).unwrap();
        probed.acquire(&[1, 2], 0.0, 1e-5, 0).unwrap();
        for &s in &STEPS {
            probed.advance(s).unwrap();
            plain.advance(s).unwrap();
            let bits = |x: &[f64]| x.iter().map(|v| v.to_bits()).collect::<Vec<_>>();
            assert_eq!(bits(probed.voltages()), bits(plain.voltages()), "{name} at {s}");
            assert_eq!(bits(probed.currents()), bits(plain.currents()), "{name} at {s}");
            for i in 0..64 {
                assert_eq!(probed.stat(i).to_bits(), plain.stat(i).to_bits(), "{name} at {s}: stat {i}");
            }
        }
    }
}

#[test]
fn a_full_queue_keeps_the_oldest_and_counts_what_it_dropped() {
    let mut e = Engine::load(&rc()).unwrap();
    e.acquire(&[2], 0.0, 1e-4, 5).unwrap();
    e.advance(1e-3).unwrap();
    let k = take(&mut e, 1);
    assert!(k.overflowed);
    assert_eq!(k.t, (0..5).map(|k| k as f64 * 1e-4).collect::<Vec<_>>());
    assert_eq!(k.dropped, 6.0, "0.5 ms .. 1 ms");
    e.advance(1.25e-3).unwrap();
    let k = take(&mut e, 1);
    assert!(!k.overflowed);
    assert_eq!(k.t.len(), 2, "1.1 and 1.2 ms after the take");
}

#[test]
fn arming_starts_at_the_engine_time_and_rearming_or_disarming_drops_the_queue() {
    let mut e = Engine::load(&rc()).unwrap();
    e.advance(1.05e-3).unwrap();
    e.acquire(&[2], 0.0, 1e-4, 0).unwrap();
    e.advance(1.3e-3).unwrap();
    let k = take(&mut e, 1);
    assert_eq!(k.t.len(), 2, "1.1 and 1.2 ms: none before the engine's time");
    assert!((k.t[0] - 1.1e-3).abs() < 1e-18);
    e.advance(1.5e-3).unwrap();
    e.acquire(&[1, 2], 0.0, 1e-4, 0).unwrap();
    e.advance(1.6e-3).unwrap();
    let k = take(&mut e, 2);
    assert_eq!(k.t.len(), 2, "1.5 and 1.6 ms; what the first arming held is gone");
    e.acquire(&[], 0.0, 0.0, 0).unwrap();
    e.advance(2e-3).unwrap();
    assert_eq!(e.acquire_take(), &[0.0; TAKE_HEADER], "disarmed");
    assert_eq!(e.acquire(&[3], 0.0, 1e-4, 0).unwrap_err().code, -8, "node out of range");
    assert_eq!(e.acquire(&[1], f64::NAN, 1e-4, 0).unwrap_err().code, -8);
    assert_eq!(e.acquire(&[1], 0.0, -1e-4, 0).unwrap_err().code, -8);
    assert_eq!(e.acquire(&[1], 0.0, f64::INFINITY, 0).unwrap_err().code, -8);
}

#[test]
fn a_rolled_back_instant_reads_no_sample() {
    let p = rc();
    let mut e = Engine::load(&p).unwrap();
    e.advance(1e-3).unwrap();
    let solver = e.debug_solver_mut();
    solver.arm_observer(Some(crate::observe::Observer::new(vec![2], 0.0, 1e-4, 100, 1e-3)));
    solver.observe(1.25e-3, true);
    let token = solver.begin_instant();
    solver.observe(1.55e-3, true);
    solver.rollback_instant(token);
    let mut out = Vec::new();
    assert!(solver.take_samples(&mut out));
    assert_eq!(out[0], 3.0, "1, 1.1 and 1.2 ms; the window's reads went back with it");
    let token = solver.begin_instant();
    solver.observe(1.55e-3, true);
    solver.end_instant(token);
    assert!(solver.take_samples(&mut out));
    assert_eq!(out[0], 3.0, "1.3 .. 1.5 ms read again, and kept");
}
