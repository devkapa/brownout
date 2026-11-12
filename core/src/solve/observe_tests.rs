//! W2b piece 1: the observer on fixed, static and closed-form nodes (core/
//! README.md, "Acquisition"; rulings R57 D1-D2 (a), R104). A sample is the
//! model's value at its grid time, read off what the solver holds, so a
//! probed run is the unprobed run bit for bit, a closed-form node's samples
//! are its exact curve, and a sample on an instant reads after it.

use crate::engine::Engine;
use crate::kinds;
use crate::observe::TAKE_HEADER;
use crate::program::{Device, Options, PinRecord, Program};

pub(super) fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

pub(super) fn dev(kind: u32, ci: usize, a: usize, b: usize, params: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: vec![pin(a), pin(b)], params, blob: vec![] }
}

pub(super) fn program(node_count: usize, devices: Vec<Device>) -> Program {
    Program { carry: Vec::new(), node_count, options: Options::default(), digital_slot_count: 2, channel_slot_count: 0, devices }
}

/// A waveform source: its blob holds its digital slot.
pub(super) fn wave(kind: u32, ci: usize, a: usize, slot: usize, params: Vec<f64>) -> Device {
    Device { blob: vec![slot as f64], ..dev(kind, ci, a, 0, params) }
}

/// 5 V through 1k into 1 uF, every node shunted by `NODE_RSHUNT_G` (1e-12
/// S): the capacitor (node 2) at `v (1 - e^(-t / tau))`, `v = 5 / (1 + R g)`,
/// `tau = R C / (1 + R g)`.
pub(super) fn rc() -> Program {
    program(3, vec![dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]), dev(kinds::RESISTOR, 1, 1, 2, vec![1000.0]), dev(kinds::CAPACITOR, 2, 2, 0, vec![1e-6, 0.0, 0.0])])
}

/// A 1 kHz clock (5 V high, half duty) through 1k into 1 uF and through a
/// 2k/10k divider, and a 1 ms pulse ramp (0 to 5 V from 0.25 ms) into 2k: a
/// fixed node with edges, a closed-form node re-aimed at each, a static node
/// that steps with them, and a ramping fixed node.
pub(super) fn edges() -> Program {
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

#[derive(Debug)]
struct Take {
    /// Per probed node, the samples that fell back to linear.
    fallbacks: Vec<f64>,
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
    assert_eq!(b.len(), TAKE_HEADER + n + count * (1 + n), "the take's length");
    let h = TAKE_HEADER + n;
    let t = b[h..h + count].to_vec();
    let v = (0..n).map(|j| b[h + count * (1 + j)..h + count * (2 + j)].to_vec()).collect();
    Take { fallbacks: b[TAKE_HEADER..h].to_vec(), overflowed: b[2] != 0.0, dropped: b[3], max_span: b[1], t, v }
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
        assert!(!k.overflowed && k.dropped == 0.0 && k.fallbacks.iter().all(|&f| f == 0.0), "{k:?}");
        t.extend(k.t);
        for (j, col) in k.v.into_iter().enumerate() {
            v[j].extend(col);
        }
    }
    (t, v)
}

/// Host steps that land on no grid time, on some, and on instants.
pub(super) const STEPS: [f64; 7] = [0.37e-3, 1e-3, 1.5e-3, 2.05e-3, 1.0 / 60.0 * 0.2, 4e-3, 5e-3];

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
    // Stepped boards too: their dense output, and an iterated partition's
    // scratch solves, leave no trace.
    for (name, p, nodes) in [("rc", rc(), [1, 2]), ("edges", edges(), [1, 2]), ("stepped rc", stepped(rc(), false), [1, 2]), ("led rc", led_rc(), [2, 3]), ("555", astable(), [3, 4]), ("ring", rc_ring(), [2, 5]), ("cc rail", cc_rail(), [1, 3]), ("cell rail, stepped", stepped(cell_rail(), false), [1, 2])] {
        let mut probed = Engine::load(&p).unwrap();
        let mut plain = Engine::load(&p).unwrap();
        probed.acquire(&nodes, 0.0, 1e-6, 0).unwrap();
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
    solver.observe(1.25e-3, true, 1e-3);
    let token = solver.begin_instant();
    solver.observe(1.55e-3, true, 1e-3);
    solver.rollback_instant(token);
    let mut out = Vec::new();
    assert!(solver.take_samples(&mut out));
    assert_eq!(out[0], 3.0, "1, 1.1 and 1.2 ms; the window's reads went back with it");
    let token = solver.begin_instant();
    solver.observe(1.55e-3, true, 1e-3);
    solver.end_instant(token);
    assert!(solver.take_samples(&mut out));
    assert_eq!(out[0], 3.0, "1.3 .. 1.5 ms read again, and kept");
}

// ---- piece 2: stepped dense output (R104) -----------------------------------

use super::PartitionKind;
use super::stepped::{ABS_VOLTAGE, REL_TOL};

/// The step's own accepted error at `v`: the controller's tolerance.
fn step_tol(v: f64) -> f64 {
    ABS_VOLTAGE + REL_TOL * v.abs()
}

struct Dense {
    t: Vec<f64>,
    v: Vec<Vec<f64>>,
    /// Per probed node, every committed step's points (`Stepped::debug_knots`).
    knots: Vec<Vec<Vec<(f64, f64, bool)>>>,
    /// Per probed node, the TR-BDF2 stage ends among them.
    stages: Vec<usize>,
    max_span: f64,
    fallbacks: Vec<f64>,
}

/// `p` probed on `nodes` at `interval` through `steps`, every committed step
/// kept; each probed node must sit in a stepped partition.
fn dense(p: &Program, nodes: &[u32], interval: f64, steps: &[f64]) -> Dense {
    let mut e = Engine::load(p).unwrap();
    for &n in nodes {
        let s = e.debug_solver();
        let kind = s.local_index[n as usize].map(|(q, _)| s.partition_kinds[q]);
        assert_eq!(kind, Some(PartitionKind::Stepped), "node {n} is stepped");
    }
    e.acquire(nodes, 0.0, interval, 0).unwrap();
    e.debug_solver_concrete_mut().stepped.keep_dense = true;
    let (mut t, mut v, mut max_span) = (Vec::new(), vec![Vec::new(); nodes.len()], 0.0f64);
    let mut fallbacks = vec![0.0; nodes.len()];
    for &s in steps {
        e.advance(s).unwrap();
        let k = take(&mut e, nodes.len());
        max_span = max_span.max(k.max_span);
        fallbacks.iter_mut().zip(&k.fallbacks).for_each(|(a, b)| *a += b);
        t.extend(k.t);
        for (j, col) in k.v.into_iter().enumerate() {
            v[j].extend(col);
        }
    }
    // The clamp commits every partition to the host's time, so none is
    // withheld: every grid time to the last step.
    let last = *steps.last().unwrap();
    let expected = (0u64..).take_while(|&k| k as f64 * interval <= last).count();
    assert_eq!(t.len(), expected, "every sample to {last}");
    let knots = nodes.iter().map(|&n| e.debug_solver().stepped.debug_knots(n as usize)).collect();
    let stages = nodes.iter().map(|&n| e.debug_solver().stepped.debug_stage_knots(n as usize)).collect();
    Dense { t, v, knots, stages, max_span, fallbacks }
}

/// Every sample of probed node `j` against `truth`: within the larger error
/// of the two points of its step around it, plus the step's own tolerance
/// (R104). Returns the worst sample error, the worst error linear
/// interpolation between the same points would make, and how many samples
/// lay in a Hermite interval.
fn check_against(d: &Dense, j: usize, name: &str, truth: &dyn Fn(f64) -> f64) -> (f64, f64, usize) {
    let (mut worst, mut worst_linear, mut hermite) = (0.0f64, 0.0f64, 0);
    let mut over: Vec<String> = Vec::new();
    for (s, &ts) in d.t.iter().enumerate() {
        // The last segment that starts at or before the sample (as the
        // sampler reads), and its interval around it.
        let seg = d.knots[j].iter().rev().find(|seg| seg[0].0 <= ts && ts <= seg.last().unwrap().0).unwrap_or_else(|| panic!("{name}: no step holds {ts}"));
        // A seed is a point (its instant only).
        let b = seg.iter().position(|k| ts <= k.0).unwrap().max(1).min(seg.len() - 1);
        let (a, b) = if seg.len() == 1 { (seg[0], seg[0]) } else { (seg[b - 1], seg[b]) };
        let got = d.v[j][s];
        let want = truth(ts);
        let err = (got - want).abs();
        let ends = (a.1 - truth(a.0)).abs().max((b.1 - truth(b.0)).abs());
        if err > ends + step_tol(want) {
            over.push(format!("{ts}: {got} against {want} ({err:e}; its step's ends {ends:e}, tolerance {:e}, {:.2}x)", step_tol(want), (err - ends) / step_tol(want)));
        }
        let linear = if b.0 > a.0 { a.1 + (b.1 - a.1) * (ts - a.0) / (b.0 - a.0) } else { b.1 };
        worst = worst.max(err);
        worst_linear = worst_linear.max((linear - want).abs());
        hermite += usize::from(b.2);
    }
    assert!(over.is_empty(), "{name}: {} of {} samples past their step's error:\n{}", over.len(), d.t.len(), over.join("\n"));
    (worst, worst_linear, hermite)
}

/// A fine reference: `p` advanced through every time in `times` with host
/// steps of at most `dt` between them (each clamp commits, so every step is
/// at most `dt`); node `n` at each.
fn fine(p: &Program, n: usize, times: &[f64], dt: f64) -> Vec<(f64, f64)> {
    let mut times = times.to_vec();
    times.sort_by(f64::total_cmp);
    times.dedup();
    let mut e = Engine::load(p).unwrap();
    let mut out = Vec::with_capacity(times.len());
    for &t in &times {
        while e.time() + dt < t {
            let next = e.time() + dt;
            e.advance(next).unwrap();
        }
        e.advance(t).unwrap();
        out.push((t, e.voltages()[n]));
    }
    out
}

fn lookup(table: &[(f64, f64)]) -> impl Fn(f64) -> f64 + '_ {
    move |t| table[table.binary_search_by(|x| x.0.total_cmp(&t)).unwrap_or_else(|_| panic!("no reference at {t}"))].1
}

pub(super) fn stepped(mut p: Program, trap: bool) -> Program {
    p.options.force_stepped = true;
    p.options.integration = u8::from(trap);
    p
}

#[test]
fn a_stepped_rc_samples_between_its_commits_within_each_steps_error() {
    let rg = 1000.0 * super::static_part::NODE_RSHUNT_G;
    let truth = |t: f64| 5.0 / (1.0 + rg) * (1.0 - (-t / (1e-3 / (1.0 + rg))).exp());
    // A linear partition steps by the trapezoid under either option (R98:
    // TR-BDF2 is the nonlinear partitions' backward Euler).
    for trap in [false, true] {
        let name = if trap { "INTEGRATION trapezoidal" } else { "INTEGRATION default" };
        let d = dense(&stepped(rc(), trap), &[2], 1e-5, &STEPS);
        assert_eq!(d.t.len(), 501, "{name}: every 10 us to 5 ms, none withheld");
        let (worst, linear, hermite) = check_against(&d, 0, name, &truth);
        assert!(hermite > 400, "{name}: {hermite} samples in Hermite intervals");
        assert!(d.max_span > 1e-5, "{name}: steps longer than the grid ({})", d.max_span);
        // Second order between the points, as the integrator: closer than
        // linear interpolation through the same points.
        assert!(worst < 0.5 * linear, "{name}: worst {worst:e} against linear's {linear:e}");
    }
}

/// A 1 kHz clock through 1k into 1 uF, a red LED and 100 ohm across it: a
/// nonlinear stepped partition, which steps by TR-BDF2 by default (R98).
pub(super) fn led_rc() -> Program {
    let mut p = program(4, vec![
        wave(kinds::CLOCK, 0, 1, 0, vec![1000.0, 0.5, 5.0, 0.0, 0.0]),
        dev(kinds::RESISTOR, 1, 1, 2, vec![1000.0]),
        dev(kinds::CAPACITOR, 2, 2, 0, vec![1e-6, 0.0, 0.0]),
        Device { kind: kinds::LED, component_index: 3, pins: vec![pin(2), pin(3)], params: vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0], blob: vec![] },
        dev(kinds::RESISTOR, 4, 3, 0, vec![100.0]),
    ]);
    p.options.force_stepped = true;
    p
}

#[test]
fn an_led_rc_samples_its_tr_bdf2_steps_within_each_steps_error_of_a_fine_reference() {
    for trap in [false, true] {
        let mut p = led_rc();
        p.options.integration = u8::from(trap);
        let name = if trap { "LED RC, trapezoid" } else { "LED RC, TR-BDF2" };
        let d = against_fine(name, &p, &[2, 3], 1e-5, &[1e-3, 1.7e-3, 2.5e-3], 0.25e-6);
        if trap {
            assert_eq!(d.stages[0], 0, "{name}: no stage");
        } else {
            assert!(d.stages[0] > 10, "{name}: its steps log their trapezoidal stage ({})", d.stages[0]);
        }
    }
}

/// A 1 kHz clock through 1k into a ring of three 100 nF capacitors joined
/// by 2k resistors, one with a 1k/1k divider in it (node 5, no capacitor's
/// plate): a linear stepped partition of three capacitors.
pub(super) fn rc_ring() -> Program {
    program(6, vec![
        wave(kinds::CLOCK, 0, 1, 0, vec![1000.0, 0.5, 5.0, 0.0, 0.0]),
        dev(kinds::RESISTOR, 1, 1, 2, vec![1000.0]),
        dev(kinds::RESISTOR, 2, 2, 3, vec![2000.0]),
        dev(kinds::RESISTOR, 3, 3, 4, vec![2000.0]),
        dev(kinds::RESISTOR, 4, 4, 5, vec![1000.0]),
        dev(kinds::RESISTOR, 5, 5, 2, vec![1000.0]),
        dev(kinds::CAPACITOR, 6, 2, 0, vec![100e-9, 0.0, 0.0]),
        dev(kinds::CAPACITOR, 7, 3, 0, vec![100e-9, 0.0, 0.0]),
        dev(kinds::CAPACITOR, 8, 4, 0, vec![100e-9, 0.0, 0.0]),
    ])
}

/// An NE555 astable on 5 V: 1k to DIS (4), 10k to THR/TRIG (3), 10 nF there,
/// OUT (5) into 1k, CV (6) with 10 nF; period about 0.146 ms. Stepped
/// (`FORCE_STEPPED`: one capacitor per partition is a closed form).
pub(super) fn astable() -> Program {
    let mut p = program(7, vec![
        dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]),
        dev(kinds::RESISTOR, 1, 1, 4, vec![1000.0]),
        dev(kinds::RESISTOR, 2, 4, 3, vec![10000.0]),
        dev(kinds::CAPACITOR, 3, 3, 0, vec![10e-9, 0.0, 0.0]),
        super::ic_tests::ne555_dev(4, 1, 0, [3, 5, 1, 6, 3, 4]),
        dev(kinds::RESISTOR, 5, 5, 0, vec![1000.0]),
        dev(kinds::CAPACITOR, 6, 6, 0, vec![10e-9, 0.0, 0.0]),
    ]);
    p.digital_slot_count = 8;
    p.options.force_stepped = true;
    p
}

/// `p` probed on `nodes`, every sample and every step point against a fine
/// reference (host steps of `dt`).
fn against_fine(name: &str, p: &Program, nodes: &[u32], interval: f64, steps: &[f64], dt: f64) -> Dense {
    let d = dense(p, nodes, interval, steps);
    for (j, &n) in nodes.iter().enumerate() {
        let mut times = d.t.clone();
        times.extend(d.knots[j].iter().flatten().map(|k| k.0));
        let table = fine(p, n as usize, &times, dt);
        let (worst, linear, hermite) = check_against(&d, j, &format!("{name} node {n}"), &lookup(&table));
        eprintln!("{name} node {n}: {} samples ({hermite} Hermite), worst {worst:e} V, linear through the same points {linear:e} V", d.t.len());
    }
    d
}

#[test]
fn an_rc_ring_samples_within_each_steps_error_of_a_fine_reference() {
    for trap in [false, true] {
        let mut p = rc_ring();
        p.options.integration = u8::from(trap);
        let _ = against_fine(if trap { "ring, trapezoid" } else { "ring, TR-BDF2" }, &p, &[2, 4, 5], 1e-5, &[1e-3, 2.5e-3, 1.0 / 60.0 * 0.2], 0.25e-6);
    }
}

#[test]
fn a_555_samples_within_each_steps_error_of_a_fine_reference() {
    for trap in [false, true] {
        let mut p = astable();
        p.options.integration = u8::from(trap);
        let _ = against_fine(if trap { "555, trapezoid" } else { "555, TR-BDF2" }, &p, &[3, 4, 6], 2e-6, &[0.3e-3, 0.61e-3, 1e-3], 0.05e-6);
    }
}

#[test]
fn a_solve_that_fails_falls_back_to_linear_and_is_counted() {
    // Every dense-output solve made to fail: the LED's cathode (node 3, no
    // capacitor's plate) reads linear where it was solved, each such sample
    // counted on its channel; the capacitor's node (2) needs no solve.
    let run = |fail: bool| {
        super::stepped::FAIL_DENSE_SOLVE.with(|f| f.set(fail));
        let d = dense(&led_rc(), &[2, 3], 1e-5, &[1e-3, 2.5e-3]);
        super::stepped::FAIL_DENSE_SOLVE.with(|f| f.set(false));
        d
    };
    let (good, failed) = (run(false), run(true));
    assert_eq!(good.fallbacks, vec![0.0, 0.0]);
    assert_eq!(failed.fallbacks[0], 0.0, "the plate is Hermite");
    assert!(failed.fallbacks[1] > 200.0, "nearly every cathode sample fell back: {}", failed.fallbacks[1]);
    assert_eq!(good.t, failed.t);
    let same = good.v[1].iter().zip(&failed.v[1]).filter(|(a, b)| a.to_bits() == b.to_bits()).count();
    assert_eq!(same as f64, good.t.len() as f64 - failed.fallbacks[1], "only the fallen-back samples differ");
    assert_eq!(good.v[0], failed.v[0]);
}

// ---- piece 3: the rail model (Rails point 4) --------------------------------

pub(super) fn psu_dev(ci: usize, pos: usize, v: f64, i_limit: f64) -> Device {
    Device { kind: kinds::BENCH_PSU, component_index: ci, pins: vec![pin(pos), pin(0)], params: vec![v, i_limit], blob: vec![] }
}

pub(super) fn cell_dev(ci: usize, pos: usize, e: f64, r: f64) -> Device {
    use crate::kinds::battery_param as b;
    let mut params = vec![0.0; b::COUNT];
    params[b::OCV_SCALE] = 1.0;
    params[b::R_BASE] = r;
    params[b::TEMP_FACTOR] = 1.0;
    params[b::DEP_A] = 0.5;
    params[b::DEP_P] = 2.0;
    params[b::DEP_K] = 0.05;
    params[b::CAPACITY_AH] = 1e-4;
    params[b::SOC0] = 1.0;
    Device { kind: kinds::BATTERY_PACK, component_index: ci, pins: vec![pin(pos), pin(0)], params, blob: vec![2.0, 0.0, 0.0, 1.0, e] }
}

/// A 5 V PSU limited to 20 mA charging 100 uF on its rail (node 1) through
/// CC into CV, a 1k/1k divider off the rail (node 2, static on the hub) and
/// 1k into 10 uF (node 3, a closed form the rail drives).
pub(super) fn cc_rail() -> Program {
    program(4, vec![
        psu_dev(0, 1, 5.0, 0.02),
        dev(kinds::CAPACITOR, 1, 1, 0, vec![100e-6, 0.0, 0.0]),
        dev(kinds::RESISTOR, 2, 1, 2, vec![1000.0]),
        dev(kinds::RESISTOR, 3, 2, 0, vec![1000.0]),
        dev(kinds::RESISTOR, 4, 1, 3, vec![1000.0]),
        dev(kinds::CAPACITOR, 5, 3, 0, vec![10e-6, 0.0, 0.0]),
    ])
}

/// A small cell (9 V, 2 ohm, 0.1 mAh, generic depletion) draining into 100
/// ohm and 100 uF (node 2) and a 1k/1k divider (node 3): its rail (node 1)
/// moves with its charge between events.
pub(super) fn cell_rail() -> Program {
    program(4, vec![
        cell_dev(0, 1, 9.0, 2.0),
        dev(kinds::RESISTOR, 1, 1, 2, vec![100.0]),
        dev(kinds::CAPACITOR, 2, 2, 0, vec![100e-6, 0.0, 0.0]),
        dev(kinds::RESISTOR, 3, 1, 3, vec![1000.0]),
        dev(kinds::RESISTOR, 4, 3, 0, vec![1000.0]),
    ])
}

#[test]
fn the_rail_and_what_it_feeds_sample_as_an_advance_to_their_time_reads_them() {
    for (name, p, nodes) in [("cc rail", cc_rail(), [1u32, 2, 3]), ("cc rail, stepped", stepped(cc_rail(), false), [1, 2, 3]), ("cell rail", cell_rail(), [1, 2, 3])] {
        // The rail's model runs across the run: the CC segment, the cell's
        // coupled path.
        let mut e = Engine::load(&p).unwrap();
        for t in [0.5e-3, 2e-3, 4.5e-3] {
            e.advance(t).unwrap();
            let st = &e.debug_solver().hub.as_ref().unwrap().state;
            assert!(p.options.force_stepped || st.cap_seg.is_some() || st.cell_path.is_some(), "{name} at {t}: a rail model runs");
        }
        let (t, v) = acquire(&p, &nodes, 1e-4, &STEPS);
        assert_eq!(t.len(), 51, "{name}: every sample to 5 ms");
        // A stepped run's twin, clamped at the sample's time, is another
        // trajectory: there each sample is held to a fine reference no
        // farther than that readout is, plus the step's tolerance (stepped
        // hub partitions sit a few mV off it on this rail, readouts and
        // samples alike, before W2b).
        let stepped = p.options.force_stepped;
        let fines: Vec<Vec<(f64, f64)>> = if stepped { nodes.iter().map(|&n| fine(&p, n as usize, &t, 0.5e-6)).collect() } else { Vec::new() };
        for (s, &ts) in t.iter().enumerate() {
            let mut twin = Engine::load(&p).unwrap();
            twin.advance(ts).unwrap();
            for (j, &n) in nodes.iter().enumerate() {
                let want = twin.voltages()[n as usize];
                if stepped {
                    let f = fines[j][s].1;
                    assert!((v[j][s] - f).abs() <= (want - f).abs() + step_tol(f), "{name} node {n} at {ts}: {} against the fine {f} (the readout {want})", v[j][s]);
                } else {
                    assert!((v[j][s] - want).abs() <= step_tol(want), "{name} node {n} at {ts}: {} against the readout's {want}", v[j][s]);
                }
            }
        }
    }
}

#[test]
fn a_rail_its_stepped_draws_move_between_solves_samples_linear_between_its_solves() {
    // A cell whose loads step (FORCE_STEPPED): the rail moves with their
    // draws between its solves, which no model holds there. Ruling (a): the
    // rail is linear between its solves, its samples waiting for the next
    // one, as a stepped partition's wait for its next commit; so are the hub
    // partitions that read it. Each sample within a fine reference (host
    // steps of 0.5 us) as near as the readout an advance there gives, plus
    // the step's tolerance; none falls back.
    let p = stepped(cell_rail(), false);
    let nodes = [1u32, 2, 3];
    let (t, v) = acquire(&p, &nodes, 1e-4, &STEPS);
    assert_eq!(t.len(), 51, "every sample to 5 ms");
    for (j, &n) in nodes.iter().enumerate() {
        let table = fine(&p, n as usize, &t, 0.5e-6);
        for (s, &ts) in t.iter().enumerate() {
            let mut twin = Engine::load(&p).unwrap();
            twin.advance(ts).unwrap();
            let (f, want) = (table[s].1, twin.voltages()[n as usize]);
            assert!((v[j][s] - f).abs() <= (want - f).abs() + step_tol(f), "node {n} at {ts}: {} against the fine {f} (the readout {want})", v[j][s]);
        }
    }
}

#[test]
fn a_sample_at_a_host_edits_time_reads_after_the_edit() {
    // W2bV finding 5 (D2): an edit at `t` is an instant at `t`, and a sample
    // on an instant reads the state after it. The clamp of `advance(t)` had
    // read the sample at `t` before the host set the source there, so it
    // kept the old value; it is read again after the edit. Taken before the
    // edit, it went out with the value the run had then.
    // 0 a 5 V source (node 1), 1 1k 1-2, 2 1k 2-0: node 2 at half the source.
    let p = program(3, vec![dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]), dev(kinds::RESISTOR, 1, 1, 2, vec![1000.0]), dev(kinds::RESISTOR, 2, 2, 0, vec![1000.0])]);
    let mut e = Engine::load(&p).unwrap();
    e.acquire(&[2], 0.0, 1e-3, 0).unwrap();
    e.advance(2e-3).unwrap();
    e.set_params(&[(0, 0, 10.0)]).unwrap();
    e.advance(4e-3).unwrap();
    let k = take(&mut e, 1);
    assert_eq!(k.t, vec![0.0, 1e-3, 2e-3, 3e-3, 4e-3]);
    let half = |v: f64| v / 2.0;
    for (s, &want) in [half(5.0), half(5.0), half(10.0), half(10.0), half(10.0)].iter().enumerate() {
        assert!((k.v[0][s] - want).abs() <= 1e-6, "at {}: {} against {want}", k.t[s], k.v[0][s]);
    }
    // A take between the clamp and the edit delivered the sample at `t`.
    let mut e = Engine::load(&p).unwrap();
    e.acquire(&[2], 0.0, 1e-3, 0).unwrap();
    e.advance(2e-3).unwrap();
    let before = take(&mut e, 1);
    e.set_params(&[(0, 0, 10.0)]).unwrap();
    e.advance(4e-3).unwrap();
    let after = take(&mut e, 1);
    assert_eq!((before.t.last().copied(), after.t.first().copied()), (Some(2e-3), Some(3e-3)));
    assert!((before.v[0][2] - 2.5).abs() <= 1e-6);
    // Not taken, an edit away from the grid leaves the samples alone, and a
    // full queue's dropped sample at `t` is not counted twice.
    let mut e = Engine::load(&p).unwrap();
    e.acquire(&[2], 0.0, 1e-3, 2).unwrap();
    e.advance(2e-3).unwrap();
    e.set_params(&[(0, 0, 10.0)]).unwrap();
    e.advance(2.5e-3).unwrap();
    e.set_params(&[(0, 0, 6.0)]).unwrap();
    e.advance(4e-3).unwrap();
    let k = take(&mut e, 1);
    assert_eq!(k.t, vec![0.0, 1e-3]);
    assert_eq!(k.dropped, 3.0, "2, 3 and 4 ms dropped once each");
}

#[test]
fn a_step_planned_before_arming_samples_by_its_capacitors_hermite() {
    // W2bV finding 6 (a): armed while a step was in flight (after a yield),
    // the samples in that step read linear between its ends (it kept no
    // points), up to 1.4 W0 off a fine reference (11.4 W0 off a run armed
    // from the start). Its ends' capacitor currents give the Hermite its
    // plates read; an iterated partition's other nodes stay linear there
    // (their scratch solves need the step's own context, gone by then). 5 V through 470 ohm into 220 nF (node 2) and
    // a red LED to 100 ohm and 47 nF (node 3): a TR-BDF2 partition whose
    // probed nodes are both plates.
    use crate::kernel::Budget;
    let mut p = program(4, vec![
        dev(kinds::VOLTAGE_SOURCE, 0, 1, 0, vec![5.0]),
        dev(kinds::RESISTOR, 1, 1, 2, vec![470.0]),
        Device { kind: kinds::LED, component_index: 2, pins: vec![pin(2), pin(3)], params: vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0], blob: vec![] },
        dev(kinds::RESISTOR, 3, 3, 0, vec![100.0]),
        dev(kinds::CAPACITOR, 4, 2, 0, vec![2.2e-7, 0.0, 0.0]),
        dev(kinds::CAPACITOR, 5, 3, 0, vec![4.7e-8, 0.0, 0.0]),
    ]);
    p.options.force_stepped = true;
    let run = |arm_after: Option<u32>| {
        let mut e = Engine::load(&p).unwrap();
        if arm_after.is_none() {
            e.acquire(&[2, 3], 1e-5, 3.1e-6, 0).unwrap();
        }
        let mut now = 0.0;
        let mut clock = || {
            now += 1.0;
            now
        };
        let mut yields = 0;
        while !e.advance_for(2e-3, Some(Budget { clock: &mut clock, budget_ms: 2.0, check_every: 1 })).unwrap() {
            yields += 1;
            if Some(yields) == arm_after {
                e.acquire(&[2, 3], 1e-5, 3.1e-6, 0).unwrap();
            }
        }
        assert!(arm_after.is_none_or(|a| a <= yields), "armed after {arm_after:?} of {yields} yields");
        take(&mut e, 2)
    };
    let all = run(None);
    let mut worst = 0.0f64;
    for arm in [3, 11, 25] {
        let late = run(Some(arm));
        assert!(!late.t.is_empty());
        let k0 = all.t.iter().position(|&t| t == late.t[0]).expect("the same grid");
        for (s, &t) in late.t.iter().enumerate().take(40) {
            assert_eq!(all.t[k0 + s], t);
            for j in 0..2 {
                let (got, want) = (late.v[j][s], all.v[j][k0 + s]);
                let share = (got - want).abs() / (1e-4 + 1e-3 * want.abs());
                worst = worst.max(share);
            }
        }
    }
    // W0 for a voltage as the verifier read it, 100 uV + 1e-3 |v|. Measured:
    // 0.49 (the cathode's turn-on curve, which one Hermite across a TR-BDF2
    // step overshoots by 50 uV); linear between the ends, 11.4.
    eprintln!("armed mid-step against armed from the start: worst {worst:.3e} W0");
    assert!(worst <= 1.0, "the first step's samples follow its capacitors: {worst} W0");
}
