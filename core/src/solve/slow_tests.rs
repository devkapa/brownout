//! W1g through the kernel and the engine (core/README.md, "Kernel semantics:
//! thermal, failures and warnings (W1g)"). Every expected value comes from
//! the model's closed form, from classic's probe F/C numbers in the W1g
//! draft (section 10), or from classic's own constants; none from this
//! implementation.

use super::*;
use crate::devices::ics;
use crate::engine::Engine;
use crate::kernel::{DeviceEvaluator, DriverRequest, Kernel, KernelError};
use crate::kinds::ic_param;
use crate::program::{Device, Options, PinRecord};
use crate::slow::thermal::THERMAL_BAND_C;

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn two(kind: u32, ci: usize, a: usize, b: usize, params: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: vec![pin(a), pin(b)], params, blob: vec![] }
}

fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
    two(kinds::VOLTAGE_SOURCE, ci, pos, neg, vec![v])
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    two(kinds::RESISTOR, ci, a, b, vec![r])
}

fn capacitor(ci: usize, a: usize, b: usize, c: f64) -> Device {
    two(kinds::CAPACITOR, ci, a, b, vec![c, 0.0, 0.0])
}

fn switch(ci: usize, a: usize, b: usize, closed: f64) -> Device {
    two(kinds::SWITCH, ci, a, b, vec![closed])
}

fn fuse(ci: usize, a: usize, b: usize, r_normal: f64, i_rating: f64) -> Device {
    two(kinds::FUSE, ci, a, b, vec![r_normal, i_rating])
}

fn ptc(ci: usize, a: usize, b: usize, r_normal: f64, i_hold: f64) -> Device {
    two(kinds::PTC_FUSE, ci, a, b, vec![r_normal, i_hold])
}

/// A red LED as the catalog's: `vf` 1.8, `n` 2, `iRated` (and `ifMax`) 20 mA.
fn red_led(ci: usize, anode: usize, cathode: usize) -> Device {
    two(kinds::LED, ci, anode, cathode, vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0])
}

fn rating(device: usize, p_max: f64, r_power: f64) -> Device {
    Device { kind: kinds::RATING, component_index: device, pins: vec![], params: vec![device as f64, p_max, r_power], blob: vec![] }
}

fn thermal(device: usize, r_theta: f64, tau_s: f64) -> Device {
    Device { kind: kinds::THERMAL, component_index: device, pins: vec![], params: vec![device as f64, r_theta, tau_s, 0.0, 0.0, 0.0], blob: vec![] }
}

/// LS-TTL IC params, as `ic_tests`' helper: `vil` 0.8, `vih` 2.0 fixed, at 5 V
/// nominal; `io_max` 0.008 is `R_out = 156.25` ohm at 5 V.
fn ls_params(delay: f64) -> Vec<f64> {
    let mut p = vec![0.0; ic_param::COUNT];
    p[ic_param::VCC_MIN] = 4.75;
    p[ic_param::VCC_MAX] = 5.25;
    p[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    p[ic_param::VIL_FIXED] = 1.0;
    p[ic_param::VIL] = 0.8;
    p[ic_param::VIH_FIXED] = 1.0;
    p[ic_param::VIH] = 2.0;
    p[ic_param::NOMINAL] = 5.0;
    p[ic_param::IO_MAX] = 0.008;
    p[ic_param::DELAY] = delay;
    p
}

/// An IC of `kind` on `vcc`/`gnd` with its logic pins on `logic`, digital
/// slots from `slot`.
fn ic(kind: u32, ci: usize, vcc: usize, gnd: usize, logic: [usize; 12], delay: f64, slot: usize) -> Device {
    let mut pins = vec![pin(vcc), pin(gnd)];
    pins.extend(logic.iter().map(|&n| pin(n)));
    let mut blob = vec![slot as f64];
    blob.extend(std::iter::repeat_n(0.0, 12));
    Device { kind, component_index: ci, pins, params: ls_params(delay), blob }
}

/// A one-cell battery at `e` volts with internal resistance `r` (a flat
/// OCV table), full.
fn battery(ci: usize, pos: usize, neg: usize, e: f64, r: f64) -> Device {
    let mut params = vec![0.0; kinds::battery_param::COUNT];
    params[kinds::battery_param::OCV_SCALE] = 1.0;
    params[kinds::battery_param::R_BASE] = r;
    params[kinds::battery_param::TEMP_FACTOR] = 1.0;
    params[kinds::battery_param::DEP_P] = 2.0;
    params[kinds::battery_param::DEP_K] = 0.05;
    params[kinds::battery_param::CAPACITY_AH] = 1.0;
    params[kinds::battery_param::SOC0] = 1.0;
    Device { kind: kinds::BATTERY_PACK, component_index: ci, pins: vec![pin(pos), pin(neg)], params, blob: vec![1.0, 0.5, e] }
}

fn psu(ci: usize, pos: usize, neg: usize, v: f64, i_limit: f64) -> Device {
    two(kinds::BENCH_PSU, ci, pos, neg, vec![v, i_limit])
}

/// Every PTC trip time (channel 2's `since`) up to `end`, advancing in steps
/// of `step`.
fn ptc_trips(p: &Program, step: f64, end: f64) -> Vec<f64> {
    let mut e = Engine::load(p).unwrap();
    let (mut trips, mut t) = (Vec::new(), 0.0f64);
    while t < end {
        t = (t + step).min(end);
        e.advance(t).unwrap();
        let rec = e.records(2).unwrap().to_vec();
        if rec.len() == 7 && trips.last() != Some(&rec[3]) {
            trips.push(rec[3]);
        }
    }
    trips
}

fn program(node_count: usize, devices: Vec<Device>) -> Program {
    Program { node_count, options: Options::default(), digital_slot_count: 16, channel_slot_count: 0, devices }
}

struct Quiet;
impl DeviceEvaluator for Quiet {
    fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
        Vec::new()
    }
}

/// The real kernel and IC evaluator over a concrete solver, so a test can
/// read the slow state as well as the ABI's view of it.
struct Rig {
    k: Kernel<PartitionedSolver, Box<dyn DeviceEvaluator>>,
}

impl Rig {
    fn load(p: &Program) -> Rig {
        let solver = PartitionedSolver::build(p).expect("builds");
        let all: Vec<PartitionId> = (0..solver.partition_count() as u32).map(PartitionId).collect();
        let mut k = Kernel::new(solver, Box::new(Quiet) as Box<dyn DeviceEvaluator>);
        let ev = ics::register(&mut k, p);
        let readers = ev.reader_ids();
        k.set_evaluator(Box::new(ev));
        k.settle(p.options.start_time, &all, &readers).expect("settles");
        k.solver_mut().prepare_readout();
        Rig { k }
    }

    fn advance(&mut self, t: f64) {
        self.try_advance(t).unwrap_or_else(|e| panic!("advance({t}): {e:?}"));
    }

    fn try_advance(&mut self, t: f64) -> Result<(), KernelError> {
        let r = self.k.advance(t);
        self.k.solver_mut().prepare_readout();
        r
    }

    fn s(&self) -> &PartitionedSolver {
        self.k.solver()
    }

    fn v(&self, node: usize) -> f64 {
        self.s().voltages()[node]
    }

    fn i(&self, device: usize) -> f64 {
        self.s().currents()[device]
    }

    fn records(&mut self, channel: u32) -> Vec<[f64; 7]> {
        let now = self.k.time();
        let mut out = Vec::new();
        assert!(self.k.solver_mut().collect_records(channel, now, &mut out));
        out.chunks_exact(7).map(|c| c.try_into().unwrap()).collect()
    }

    fn state(&self, device: usize) -> Vec<f64> {
        let mut out = Vec::new();
        self.s().device_state(device, self.k.time(), &mut out).expect("a device with a state");
        out
    }

    fn stats(&self) -> [f64; 4] {
        self.s().slow_stats()
    }
}

fn close(got: f64, want: f64, tol: f64, what: &str) {
    assert!((got - want).abs() <= tol, "{what}: got {got}, want {want} (tolerance {tol:e})");
}

/// The LED's operating point on `r` ohm from `v` at law `law`, by bisection
/// on the law (the model's own answer; a diode and a resistor have no closed
/// form).
fn led_operating_point(law: &led::JunctionLaw, r: f64, v: f64) -> (f64, f64) {
    let (mut lo, mut hi) = (0.0, v);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if v - r * law.current(mid) - mid > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let vd = 0.5 * (lo + hi);
    (vd, law.current(vd))
}

// ---- resistor, fuse, LED, PTC ---------------------------------------------------

/// 5 V across 50 ohm, `p_max` 0.25 W: 0.5 W, twice the rating, so the dwell
/// integrator runs at rate 1 and latches at exactly t = 1 (probe F
/// `resistor2x`, analytic; classic 1.0017965 at a step end).
fn resistor_2x() -> Program {
    program(2, vec![source(0, 1, 0, 5.0), resistor(1, 1, 0, 50.0), rating(1, 0.25, 50.0)])
}

/// Cold 668/1's rated resistor alone: a pulse holds node 1 at 9.8 V and
/// drops it to -1.4 V at 150 ms; 84.7 ohm (31 mW, `rPower` its own R) from
/// node 1 to 8.6 uF, stepped. The capacitor discharges through it from 11.2
/// V with tau = 0.73 ms: `S = (P0/pMax) (tau/2) (1 - pMax/P0) - t*` with `P0
/// = 11.2^2/84.7 = 1.48 W`, `t* = (tau/2) ln(P0/pMax)` the time over its
/// rating: 0.0157, far from the latch at 1.
fn rated_rc_under_an_edge(force_stepped: bool) -> Program {
    let mut p = program(3, vec![
        Device { kind: kinds::PULSE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![9.8, -1.4, 0.15, 0.0, 0.0, 1.0, 10.0], blob: vec![0.0] },
        resistor(1, 1, 2, 84.7),
        capacitor(2, 2, 0, 8.6e-6),
        rating(1, 0.031, 84.7),
    ]);
    p.options.force_stepped = force_stepped;
    p
}

#[test]
fn a_stepped_rated_resistor_idle_until_an_edge_is_charged_from_the_edge() {
    // Defect B (cold 668/1): an idle visit (R38 (a)) writes nothing, so the
    // integrator's `t0` stayed at the visit before the capacitor settled,
    // and the backward rule held the rate read just after the edge (47/s)
    // over the whole quiet stretch back to it: S = 7 at the edge, the
    // resistor burnt there. The rate the edge brings starts at the edge.
    let tau: f64 = 84.7 * 8.6e-6;
    let p0 = 11.2f64 * 11.2 / 84.7;
    let t_star = tau / 2.0 * (p0 / 0.031).ln();
    let want = p0 / 0.031 * tau / 2.0 * (1.0 - 0.031 / p0) - t_star;
    for (name, p) in [("stepped", rated_rc_under_an_edge(true)), ("exact mode", { let mut p = rated_rc_under_an_edge(true); p.options.no_rail_shortcut = true; p })] {
        let mut rig = Rig::load(&p);
        let mut peak: f64 = 0.0;
        for t in [0.1, 0.15].into_iter().chain((1..=40).map(|k| 0.15 + k as f64 * 1e-4)).chain([0.16, 0.2]) {
            rig.k.advance(t).unwrap();
            peak = peak.max(rig.s().slow.stress(1).unwrap().value_at(t));
        }
        let s = rig.s();
        eprintln!("{name}: S peaks at {peak:.5}, by hand {want:.5}; failed {}", s.slow.is_failed(1));
        assert!(!s.slow.is_failed(1), "{name}: R1 burnt (S {:?})", s.slow.stress(1));
        assert!(peak > 0.25 * want && peak < 4.0 * want, "{name}: S peaks at {peak} against {want} by hand");
    }
}

#[test]
fn a_resistor_at_twice_its_rating_on_a_fixed_pair_latches_at_one_second_and_then_carries_nothing() {
    let mut rig = Rig::load(&resistor_2x());
    close(rig.i(1), 0.1, 1e-15, "0.1 A at load");
    rig.advance(0.999);
    assert!(rig.records(0).is_empty(), "nothing has latched yet");
    rig.advance(1.5);
    let r = rig.records(0);
    assert_eq!(r.len(), 1, "{r:?}");
    let [dev, code, pin, since, value, limit, aux] = r[0];
    assert_eq!((dev, code, pin, aux), (1.0, 1.0, -1.0, 0.0), "resistor_overload, no pin");
    close(since, 1.0, 1e-12, "latch time");
    close(value, 0.5, 1e-12, "P at the latch");
    assert_eq!(limit, 0.25);
    // The open: no node voltage moved (both are fixed), so only the `full`
    // recompute and the republished KCL can make these zero.
    assert_eq!(rig.i(1), 0.0, "the open resistor carries nothing");
    assert!(rig.i(0).abs() < 1e-9, "the source supplies only node 1's shunt: {}", rig.i(0));
    // The latch log has the latch, once.
    let log = rig.records(3);
    assert_eq!(log.len(), 1);
    close(log[0][3], 1.0, 1e-12, "the log's since is the latch time");
    assert!(rig.records(3).is_empty(), "collecting the log empties it");
    assert!(rig.stats()[2] >= 1.0, "stat 23 counts the latch");
}

#[test]
fn a_fuse_at_twice_its_rating_latches_at_a_tenth_of_a_second_and_opens() {
    // 0.1 V across a 0.1 ohm fuse rated 0.5 A: 1 A, rate (1/0.5)^2 - 1 = 3,
    // threshold 0.3, so t = 0.1 (probe F `fuse2x`: classic 0.1018).
    let mut rig = Rig::load(&program(2, vec![source(0, 1, 0, 0.1), fuse(1, 1, 0, 0.1, 0.5)]));
    close(rig.i(1), 1.0, 1e-12, "1 A through the intact fuse");
    rig.advance(0.2);
    let r = rig.records(0);
    assert_eq!(r.len(), 1);
    assert_eq!((r[0][0], r[0][1], r[0][2]), (1.0, 2.0, -1.0), "fuse_tripped");
    close(r[0][3], 0.1, 1e-12, "latch time");
    close(r[0][4], 1.0, 1e-12, "I at the latch");
    assert_eq!(r[0][5], 0.5);
    assert_eq!(rig.i(1), 0.0, "a tripped fuse is open");
}

#[test]
fn an_overdriven_led_latches_at_the_analytic_time() {
    // Probe F `ledOver`: a red LED on 50 ohm from 5 V, 62.824 mA against
    // `ifMax` 20 mA: r = 3.1412, rate r^2 - 1 = 8.8670/s, threshold 24, so
    // t = 2.7066528 s analytic (classic 2.7066605 at a 64 us step end).
    let law = led::JunctionLaw::at(1.8, 0.02, 2.0, None, 25.0);
    let (_, i) = led_operating_point(&law, 50.0, 5.0);
    let ratio = i / 0.02;
    let t_x = 24.0 / (ratio * ratio - 1.0);
    close(t_x, 2.7066528393, 1e-9, "the model's latch time");
    let mut rig = Rig::load(&program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 50.0), red_led(2, 2, 0)]));
    rig.advance(2.7);
    assert!(rig.records(0).is_empty());
    rig.advance(3.0);
    let r = rig.records(0);
    assert_eq!(r.len(), 1);
    assert_eq!((r[0][0], r[0][1], r[0][2]), (2.0, 3.0, -1.0), "led_failed");
    close(r[0][3], 2.7066528393, 1e-9, "latch time");
    close(r[0][4], i, 1e-9, "peakI");
    assert_eq!(r[0][5], 0.02);
    assert_eq!(rig.i(2), 0.0, "a failed LED reads 0, as a burnt one");
    // With the junction gone the node floats up to the source through 50 ohm.
    close(rig.v(2), 5.0, 1e-6, "the anode after the open");
}

#[test]
fn a_safe_led_driven_over_its_limit_by_a_switch_fails_from_that_instant() {
    // A red LED behind 1k carries 3.2 mA, far under `ifMax`, so its visits
    // change nothing. Closing a switch puts 50 ohm (plus the contact's 1 mohm)
    // in parallel at t = 1: only the anode node moves, and the damage clock
    // starts there, so the latch lands at 1 + 24 / (r^2 - 1) as for a plain
    // 50 ohm feed of that resistance.
    let law = led::JunctionLaw::at(1.8, 0.02, 2.0, None, 25.0);
    let r_eq = 1.0 / (1.0 / 50.001 + 1.0 / 1000.0);
    let (_, i) = led_operating_point(&law, r_eq, 5.0);
    let ratio = i / 0.02;
    let t_x = 1.0 + 24.0 / (ratio * ratio - 1.0);
    let mut e = Engine::load(&program(4, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 1000.0), red_led(2, 2, 0), resistor(3, 1, 3, 50.0), switch(4, 3, 2, 0.0)])).unwrap();
    e.advance(1.0).unwrap();
    assert!(e.records(0).unwrap().is_empty());
    e.set_params(&[(4, 0, 1.0)]).unwrap();
    e.advance(t_x - 0.01).unwrap();
    assert!(e.records(0).unwrap().is_empty(), "not before {t_x}");
    e.advance(t_x + 0.01).unwrap();
    let r = e.records(0).unwrap();
    assert_eq!(r.len(), 7, "{r:?}");
    assert_eq!((r[0], r[1], r[2]), (2.0, 3.0, -1.0), "led_failed");
    close(r[3], t_x, 1e-6, "latch time");
}

#[test]
fn a_ptc_at_twice_its_hold_trips_and_recovers_two_seconds_later() {
    // Probe F `ptc`: 0.5 V across 0.5 ohm, 1 A against `iHold` 0.5 A: S' =
    // 1 - 0.25 = 0.75 to 3 * 0.25 * 0.1 = 0.075, a trip at 0.1 s. Tripped it is
    // 100 ohm, 5 mA, cool, so it recovers 2.0 s after the trip, and trips
    // again 0.1 s after that.
    let mut rig = Rig::load(&program(2, vec![source(0, 1, 0, 0.5), ptc(1, 1, 0, 0.5, 0.5)]));
    rig.advance(0.05);
    let st = rig.state(1);
    assert_eq!(st[0], 0.0, "not tripped");
    close(st[1], 0.75 * 0.05, 1e-15, "tripStress");
    assert_eq!(st[2], 0.0);
    close(rig.i(1), 1.0, 1e-12, "1 A");
    rig.advance(1.0);
    let st = rig.state(1);
    assert_eq!(st[0], 1.0, "tripped");
    assert_eq!(st[1], 0.0, "the trip zeroes S");
    close(st[2], 0.9, 1e-12, "cool from the trip instant: Rt = 1.0 - 0.1");
    close(rig.i(1), 0.005, 1e-15, "5 mA through 100 ohm");
    let tripped = rig.records(2);
    assert_eq!(tripped.len(), 1);
    assert_eq!((tripped[0][0], tripped[0][1], tripped[0][2]), (1.0, 1.0, -1.0));
    close(tripped[0][3], 0.1, 1e-12, "since the trip");
    assert!(rig.records(0).is_empty() && rig.records(1).is_empty(), "a PTC trip is never a failure or a warning");
    rig.advance(2.099);
    assert_eq!(rig.state(1)[0], 1.0, "still tripped just before 2.1 s");
    rig.advance(2.15);
    let st = rig.state(1);
    assert_eq!(st[0], 0.0, "recovered at 2.1 s");
    close(st[1], 0.75 * 0.05, 1e-12, "integrating again from the recovery");
    close(rig.i(1), 1.0, 1e-12, "1 A again");
    assert!(rig.records(2).is_empty());
    rig.advance(2.19);
    assert_eq!(rig.state(1)[0], 0.0);
    rig.advance(2.21);
    assert_eq!(rig.state(1)[0], 1.0, "tripped again at 2.2 s");
    close(rig.records(2)[0][3], 2.2, 1e-12, "the second trip's time");
}

#[test]
fn a_ptc_on_a_current_limited_psu_cycles_at_the_analytic_times_however_the_host_advances() {
    // G2bv v6e: a 0.5 V PSU limited to 0.8 A across a 0.5 ohm PTC (iHold 0.5
    // A) runs in CC at 0.8 A: S' = 0.64 - 0.25 = 0.39 to 0.075, a trip at
    // 0.075 / 0.39 s. Tripped it carries 5 mA and recovers 2.0 s later, back
    // into CC. A trip changes a fixed-pair stamp on the rail, so the rail's
    // own loop must re-run at that instant, not at the next publish.
    let p = program(2, vec![psu(0, 1, 0, 0.5, 0.8), ptc(1, 1, 0, 0.5, 0.5)]);
    let period = 2.0 + 0.075 / 0.39;
    let want: Vec<f64> = (0..4).map(|k| 0.075 / 0.39 + k as f64 * period).collect();
    for step in [0.37, 0.01, 1.0 / 60.0] {
        let got = ptc_trips(&p, step, 3.0 * period + 0.5);
        assert_eq!(got.len(), 4, "{step} s steps: {got:?}");
        for (g, w) in got.iter().zip(&want) {
            close(*g, *w, 1e-9, &format!("trip at {step} s steps: {got:?}"));
        }
    }
}

#[test]
fn a_ptc_on_a_psu_in_cv_trips_recovers_and_trips_again_at_the_probe_times_in_any_steps() {
    // G2bv v6c: probe F `ptc` on a 0.5 V PSU limited to 5 A (CV): trips at
    // 0.1 s, recovers at 2.1 s, trips again at 2.2 s, however the host
    // advances. The recovery's own event is due inside the settle that a
    // publish ran; its re-drive must not wait for the next instant.
    for src in [psu(0, 1, 0, 0.5, 5.0), source(0, 1, 0, 0.5)] {
        let p = program(2, vec![src, ptc(1, 1, 0, 0.5, 0.5)]);
        for step in [0.01, 0.37, 1.0 / 60.0] {
            let got = ptc_trips(&p, step, 4.5);
            let want = [0.1, 2.2, 4.3];
            assert_eq!(got.len(), 3, "{step} s steps: {got:?}");
            for (g, w) in got.iter().zip(&want) {
                close(*g, *w, 1e-9, &format!("trip at {step} s steps: {got:?}"));
            }
        }
    }
}

#[test]
fn a_failed_led_across_a_source_leaves_the_source_carrying_the_shunt_alone() {
    // README W1g point 12: a failed LED stamps no junction, GMIN included, so
    // the source KCL walk must skip it: with both of its nodes fixed, the
    // source carries the node shunt's 2 V * NODE_RSHUNT_G alone (contract
    // point 2), not GMIN_D * 2 V on top.
    let mut rig = Rig::load(&program(2, vec![source(0, 1, 0, 2.0), red_led(1, 1, 0)]));
    rig.advance(0.05);
    assert_eq!(rig.records(0).len(), 1, "the LED failed");
    assert_eq!(rig.i(0).to_bits(), (-(static_part::NODE_RSHUNT_G * 2.0)).to_bits(), "the source's current: {:e}", rig.i(0));
    assert_eq!(rig.i(1), 0.0, "the LED's current");
}

#[test]
fn a_rolled_back_rearm_leaves_the_junction_memo_serving_the_law_the_twin_has() {
    // G2bv v19, README W1g point 4: a doomed window re-arms a fixed-pair LED
    // (its law moves to `T_hold`, `law_gen` + 1) and republishes, which stores
    // the source current under the new generation in the junction memo. The
    // rollback brings the generation back, so the next re-arm reuses that
    // number for another law: the memo must be cleared with the rollback, or
    // a debug build's incremental-current check (and the readout) serve the
    // doomed law. The source current then matches a twin's bit for bit.
    let p = program(2, vec![source(0, 1, 0, 1.6), red_led(1, 1, 0), thermal(1, 560.0, 2.0)]);
    let (mut doomed, mut twin) = (Rig::load(&p), Rig::load(&p));
    for rig in [&mut doomed, &mut twin] {
        rig.advance(0.5);
    }
    assert_eq!(doomed.i(0).to_bits(), twin.i(0).to_bits());
    {
        let s = doomed.k.solver_mut();
        let cp = s.begin_instant();
        assert!(s.handle_slow_event(DeviceId(1), 0.5).unwrap().is_empty());
        s.resolve_dirty(&[], 0.5, &[]).unwrap();
        s.prepare_readout();
        s.rollback_instant(cp);
    }
    for k in 1..=60 {
        let t = 0.5 + 0.0125 * k as f64;
        doomed.advance(t);
        twin.advance(t);
        assert_eq!(doomed.i(0).to_bits(), twin.i(0).to_bits(), "the source current at {t} s");
    }
}

/// The first `t` at which `s(t) = 1`, by bisection on `[0, t_peak]` where
/// `s` rises; `None` when it never gets there.
fn first_unity(s: impl Fn(f64) -> f64, t_peak: f64) -> Option<f64> {
    if !(s(t_peak) >= 1.0) {
        return None;
    }
    let (mut lo, mut hi) = (0.0, t_peak);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if s(mid) < 1.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(0.5 * (lo + hi))
}

/// 10 ohm (pMax 0.5 W) from the rail to 0.1 F: the RC of G2bv's v1, fed by
/// `src` on node 1.
fn rail_rc(src: Device, extra: Vec<Device>) -> Program {
    let mut devices = vec![src, resistor(1, 1, 2, 10.0), capacitor(2, 2, 0, 0.1)];
    devices.extend(extra);
    program(3, devices)
}

fn latch_of(p: &Program, dev: f64, until: f64) -> Option<f64> {
    let mut e = Engine::load(p).unwrap();
    e.advance(until).unwrap();
    e.records(0).unwrap().chunks(7).find(|c| c[0] == dev).map(|c| c[3])
}

#[test]
fn an_rc_resistor_on_a_battery_latches_at_the_closed_form_time_for_any_internal_resistance() {
    // G2bv v1 (ruling R21): a battery E = 5 V with internal r feeding 10 ohm
    // into 0.1 F. The cell is the rail and the RC a closed-form partition the
    // rail drives: i = E/(R+r) e^(-t/((R+r)C)), P = i^2 R, S' = P/pMax - 1,
    // so S(t) = k tau/2 (1 - e^(-2t/tau)) - t with k = E^2 R/((R+r)^2 pMax).
    // The latch is within W0 (1e-3 relative) at every r.
    for r in [0.0f64, 1e-4, 1e-2, 0.1, 0.5] {
        let rr = 10.0 + r;
        let tau = rr * 0.1;
        let k = 25.0 * 10.0 / (rr * rr * 0.5);
        let want = first_unity(|t: f64| k * tau / 2.0 * (1.0 - (-2.0 * t / tau).exp()) - t, tau / 2.0 * k.ln()).expect("it latches");
        let got = latch_of(&rail_rc(battery(0, 1, 0, 5.0, r), vec![rating(1, 0.5, 10.0)]), 1.0, 1.5).expect("latched");
        close(got, want, 1e-3 * want, &format!("latch time at r = {r}"));
    }
}

#[test]
fn an_rc_resistor_on_a_battery_heats_by_the_closed_form_power_for_any_internal_resistance() {
    // As above with a thermal record (R_theta 100 K/W, tau 0.5 s) and no
    // rating: T' = (25 + R_theta P(t) - T)/tau_th, integrated here with RK4
    // on the exact P(t). The readout is within 1e-3 of the rise plus 1e-3 C.
    for r in [0.0f64, 1e-4, 1e-2, 0.1, 0.5] {
        let rr = 10.0 + r;
        let p_at = |t: f64| {
            let i = 5.0 / rr * (-t / (rr * 0.1)).exp();
            i * i * 10.0
        };
        let mut e = Engine::load(&rail_rc(battery(0, 1, 0, 5.0, r), vec![thermal(1, 100.0, 0.5)])).unwrap();
        let (mut temp, mut t, dt) = (25.0f64, 0.0f64, 1e-5);
        for check in [0.1, 0.3, 0.6, 1.2] {
            while t < check - 1e-12 {
                let f = |t: f64, y: f64| (25.0 + 100.0 * p_at(t) - y) / 0.5;
                let k1 = f(t, temp);
                let k2 = f(t + dt / 2.0, temp + dt / 2.0 * k1);
                let k3 = f(t + dt / 2.0, temp + dt / 2.0 * k2);
                let k4 = f(t + dt, temp + dt * k3);
                temp += dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
                t += dt;
            }
            e.advance(check).unwrap();
            let got = e.device_state(1).unwrap()[0];
            close(got, temp, 1e-3 * (temp - 25.0) + 1e-3, &format!("T at {check} s, r = {r}"));
        }
    }
}

/// A PSU (`v`, `il`) with a rail capacitor `c1`, `r` from the rail to `c2`:
/// the latch time of `r` (pMax `p_max`) by RK4 on the circuit itself, CC
/// until the rail reaches `v`, then CV; `S' = P/pMax - 1`, never below 0.
fn cc_rail_cap_latch(v: f64, il: f64, c1: f64, r: f64, c2: f64, p_max: f64) -> Option<f64> {
    let dt = 1e-6;
    let (mut u, mut w, mut s, mut t) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let deriv = |u: f64, w: f64| -> (f64, f64) {
        let ib = (u - w) / r;
        (if u < v { (il - ib) / c1 } else { 0.0 }, ib / c2)
    };
    while t < 5.0 {
        let (k1u, k1w) = deriv(u, w);
        let (k2u, k2w) = deriv(u + 0.5 * dt * k1u, w + 0.5 * dt * k1w);
        let (k3u, k3w) = deriv(u + 0.5 * dt * k2u, w + 0.5 * dt * k2w);
        let (k4u, k4w) = deriv(u + dt * k3u, w + dt * k3w);
        let p0 = (u - w).powi(2) / r;
        u = (u + dt / 6.0 * (k1u + 2.0 * k2u + 2.0 * k3u + k4u)).min(v);
        w += dt / 6.0 * (k1w + 2.0 * k2w + 2.0 * k3w + k4w);
        let p1 = (u - w).powi(2) / r;
        s = (s + dt * (0.5 * (p0 + p1) / p_max - 1.0)).max(0.0);
        t += dt;
        if s >= 1.0 {
            return Some(t);
        }
    }
    None
}

#[test]
fn an_rc_resistor_on_a_current_limited_psu_with_a_rail_cap_latches_within_w0() {
    // A 5 V PSU limited to 0.3 A charging a rail capacitor and, through 10
    // ohm, 0.1 F: the rail rises in CC, then holds in CV. The RC is a
    // closed-form partition on the rail's coupled path.
    for c1 in [0.01, 0.001] {
        let want = cc_rail_cap_latch(5.0, 0.3, c1, 10.0, 0.1, 0.3).expect("it latches");
        let p = program(3, vec![psu(0, 1, 0, 5.0, 0.3), capacitor(1, 1, 0, c1), resistor(2, 1, 2, 10.0), capacitor(3, 2, 0, 0.1), rating(2, 0.3, 10.0)]);
        let got = latch_of(&p, 2.0, 5.0).expect("latched");
        close(got, want, 1e-3 * want, &format!("latch time with a {c1} F rail cap"));
    }
}

#[test]
fn a_rolled_back_re_anchor_leaves_the_run_as_a_twins() {
    // R21: a re-anchor writes the device's drives and schedules the next
    // under a new token. A doomed window that re-anchors (an off-schedule
    // `Slow` event, then the settle) and rolls back must leave the run
    // exactly as a twin's: the latch, the readout and the thermal state.
    let p = rail_rc(battery(0, 1, 0, 5.0, 0.1), vec![rating(1, 0.5, 10.0), thermal(1, 100.0, 0.5)]);
    let (mut doomed, mut twin) = (Rig::load(&p), Rig::load(&p));
    for rig in [&mut doomed, &mut twin] {
        rig.advance(0.1);
    }
    let drivers: Vec<crate::drivers::Driver> = Vec::new();
    {
        let s = doomed.k.solver_mut();
        let cp = s.begin_instant();
        s.handle_slow_anchor(DeviceId(1), 0.1).unwrap();
        s.slow_settle(0.1, &drivers).unwrap();
        s.rollback_instant(cp);
    }
    for t in [0.2, 0.3, 0.42, 0.5, 1.0] {
        doomed.advance(t);
        twin.advance(t);
        assert_eq!(doomed.state(1), twin.state(1), "at {t} s");
        assert_eq!(doomed.records(0), twin.records(0), "at {t} s");
        assert_eq!(doomed.i(1).to_bits(), twin.i(1).to_bits(), "at {t} s");
    }
}

fn slow_events_by(p: &Program, t_end: f64) -> (f64, Engine) {
    let mut e = Engine::load(p).unwrap();
    e.advance(t_end).unwrap();
    (e.stat(22), e)
}

#[test]
fn a_fast_rc_on_a_current_limited_ramp_re_anchors_only_for_the_rails_curvature() {
    // G2bv2's storm (R21, R29): 5 V, 0.1 A into 1 mF ramps the rail at 100
    // V/s; a rated 10 ohm into 0.1 uF (tau 1 us) on it tracks the rail, so
    // its own voltage is one exponential with the lag `S tau` in its
    // asymptote (`L = 0`). Only the rail's curvature re-anchors it: a
    // handful in 20 ms, not one per pin-tolerance of rail travel (61,000)
    // nor one per 2 ns (9.3 M).
    let p = program(3, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-3), resistor(2, 1, 2, 10.0), capacitor(3, 2, 0, 1e-7), rating(2, 0.25, 10.0)]);
    let (n, e) = slow_events_by(&p, 0.02);
    assert!(e.voltages()[1] > 1.9, "the rail ramped: {}", e.voltages()[1]);
    assert!(n <= 100.0, "{n} re-anchors");
}

#[test]
fn a_device_that_does_not_track_the_rail_re_anchors_per_tenth_of_w0_of_its_own_growth() {
    // The rail (5 V, 0.1 A into 10 mF) drives 10 ohm into node 2 (1 mF) and
    // a rated 30 ohm from node 2 to ground: the device's voltage grows with
    // the rail (`L = g S`), which one exponential cannot follow. Matched to
    // its slope, the anchor drops `L (δ - (1 - e^(fδ))/|f|)`, at most `|L f|
    // δ²/2`: so in 1 s it re-anchors about 1,500 times (measured 1,525; the
    // band below is ±10 % of that), not the 10^4 per e-fold of its voltage a
    // dropped `Lδ` would cost. Half the bound or ten times the relative
    // tolerance moves it out of the band. It is rated 0.5 W: 5 V across it
    // would be 0.83 W, so R38 (a)'s bound does not make it cold (a cold
    // device is never visited), and its 3.75 V at most is 0.47 W, under it.
    let p = program(3, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-2), resistor(2, 1, 2, 10.0), capacitor(3, 2, 0, 1e-3), resistor(4, 2, 0, 30.0), rating(4, 0.5, 30.0)]);
    let (n, _) = slow_events_by(&p, 1.0);
    assert!((1370.0..=1680.0).contains(&n), "{n} re-anchors");
}

#[test]
fn a_quiet_device_on_a_moving_rail_is_never_re_anchored() {
    // R30's fallback: a 1k resistor rated 10 W straight across a CC rail
    // whose coupled path (an RC on it) has modes, so its drive is held for
    // `tol / (|σ| S)` at a time. Far under its rating it is quiet: its drive
    // writes nothing, and nothing re-anchors it (a re-anchor per deadline
    // would be some 100,000 events in 0.5 s). Rated 25.2 mW, it is not cold
    // (R38 (a): the widened 5 V bound is 25.6 mW), but 5 V is 25 mW.
    let p = program(3, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-2), resistor(2, 1, 2, 10.0), capacitor(3, 2, 0, 1e-3), resistor(4, 1, 0, 1000.0), rating(4, 0.0252, 1000.0)]);
    let (n, _) = slow_events_by(&p, 0.5);
    assert_eq!(n, 0.0, "{n} slow events");
}

#[test]
fn re_anchors_never_come_closer_than_two_nanoseconds() {
    // 10 A into 1 uF slews the rail at 1e7 V/s: the tolerance alone would
    // re-anchor the rated RC every 0.5 ps. The floor (2 NO_PROGRESS_DT) caps
    // the ramp's 0.5 us at 250 re-anchors.
    let p = program(3, vec![psu(0, 1, 0, 5.0, 10.0), capacitor(1, 1, 0, 1e-6), resistor(2, 1, 2, 10.0), capacitor(3, 2, 0, 1e-9), rating(2, 100.0, 10.0)]);
    let (n, _) = slow_events_by(&p, 1e-3);
    assert!(n <= 1.2 * 5e-7 / 2e-9, "{n} re-anchors");
}

#[test]
fn the_re_anchor_deadlines_are_the_bounds_own_roots() {
    // R21's deadline is the largest δ inside the bound; each case's root is
    // solved here by hand. A linear segment: `|a| S δ + |dx| W min(δ²/2,
    // δ/|f|)`.
    use super::closed_form::{largest_within, linear_anchor_bound, linear_anchor_deadline, CurvedBound};
    let tol = 1e-4;
    // Inside 2/|f| = 0.02 s: (W/2) δ² + a S δ = tol.
    let (a, s, w) = (0.5, 10.0, 20.0);
    let got = largest_within(|d| linear_anchor_bound(a, s, 1.0, w, -100.0, d), tol);
    let want = (-a * s + (a * a * s * s + 2.0 * w * tol).sqrt()) / w;
    assert!(want < 0.02);
    close(got, want, 1e-9 * want, "quadratic branch");
    // Past 2/|f| = 2 us, rail-free (a = 0): W δ/|f| = tol.
    let got = largest_within(|d| linear_anchor_bound(0.0, 7.0, 1.0, 1e3, -1e6, d), tol);
    close(got, tol * 1e6 / 1e3, 1e-9 * 0.1, "linear branch");
    // The closed form the solver uses agrees with both roots.
    close(linear_anchor_deadline(a, s, 1.0, w, -100.0, tol), want, 1e-9 * want, "closed form, quadratic");
    close(linear_anchor_deadline(0.0, 7.0, 1.0, 1e3, -1e6, tol), 0.1, 1e-10, "closed form, linear");
    // A curved segment, flat (`a1 = we = 0`) and far into its segment
    // (`τn |f| >> 1`, so `|J'| <= 1/|f|`): `(a0 S + |dx| W/|f| + |D|/tau) δ`.
    let b = CurvedBound { tau_n: 1.0, s_sum: 3.0, w_sum: 40.0, we: 0.0, a0: 0.25, a1: 0.0, adx: 2.0, vm: 4.0, dvc: 1.0, f: -1e3, d_now: 2e-6 };
    let got = largest_within(|d| b.at(d), tol);
    let want = tol / (0.25 * 3.0 + 2.0 * 40.0 / 1e3 + 2e-6 * 1e3);
    close(got, want, 1e-9 * want, "curved, flat");
}

/// G2bv2's `p_for`: the `pMax` at which an RC of time constant `tau` fed at
/// peak power `p_peak` ends its stress at `s_inf` (it latches iff `s_inf >=
/// 1`): `S_inf = tau (k - 1 - ln k) / 2` with `k = p_peak / pMax`.
fn p_max_for(p_peak: f64, tau: f64, s_inf: f64) -> f64 {
    let f = |k: f64| tau * (k - 1.0 - k.ln()) / 2.0;
    let (mut lo, mut hi) = (1.0f64 + 1e-12, 1e18f64);
    for _ in 0..300 {
        let mid = (lo * hi).sqrt();
        if f(mid) < s_inf {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    p_peak / hi
}

/// A PSU (`v`, `il`) with a rail capacitor `c1` and `r` into `c2`: the
/// latch time of `r` (pMax `p_max`) by RK4 on the circuit (`dt`), CC until
/// the rail reaches `v`, then CV, with the stress rule itself: `S' = P/pMax
/// - 1` while that is positive, else `-1` (the resistor's recovery), never
/// below 0.
fn psu_rc_latch(v: f64, il: f64, c1: f64, r: f64, c2: f64, p_max: f64, t_end: f64, dt: f64) -> Option<f64> {
    let (mut u, mut w, mut s, mut t) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let deriv = |u: f64, w: f64| -> (f64, f64) {
        let ib = (u - w) / r;
        (if u < v { (il - ib) / c1 } else { 0.0 }, ib / c2)
    };
    let rate = |u: f64, w: f64| {
        let x = (u - w).powi(2) / r / p_max - 1.0;
        if x > 0.0 { x } else { -1.0 }
    };
    while t < t_end {
        let (k1u, k1w) = deriv(u, w);
        let (k2u, k2w) = deriv(u + 0.5 * dt * k1u, w + 0.5 * dt * k1w);
        let (k3u, k3w) = deriv(u + 0.5 * dt * k2u, w + 0.5 * dt * k2w);
        let (k4u, k4w) = deriv(u + dt * k3u, w + dt * k3w);
        let (u0, w0) = (u, w);
        u = (u + dt / 6.0 * (k1u + 2.0 * k2u + 2.0 * k3u + k4u)).min(v);
        w += dt / 6.0 * (k1w + 2.0 * k2w + 2.0 * k3w + k4w);
        let s1 = (s + dt * 0.5 * (rate(u0, w0) + rate(u, w))).max(0.0);
        if s1 >= 1.0 {
            // Linear inside the step.
            return Some(t + dt * (1.0 - s) / (s1 - s));
        }
        s = s1;
        t += dt;
    }
    None
}

#[test]
fn rc_resistors_on_slow_current_limited_ramps_latch_within_w0() {
    // G2bv2's three slow-ramp cases (ruling R29): the resistor's voltage is a
    // small difference of a rail and a node that both follow it, so a
    // tolerance on each pin is no tolerance on it. pMax ends the stress at
    // 1.5 (p_max_for), so each latches once.
    for (il, c1, r, c2) in [(0.05f64, 0.1f64, 1.0f64, 0.01f64), (0.05, 0.01, 10.0, 1e-3), (0.3, 0.1, 1.0, 0.01)] {
        let v = 5.0f64;
        let p_max = p_max_for((v * v / r).min(il * il * r), r * c2, 1.5);
        let t_end = (3.0 * (r * c2 + v * c1 / il)).clamp(2e-3, 4.0);
        let want = psu_rc_latch(v, il, c1, r, c2, p_max, t_end, 1e-5).expect("it latches");
        let p = program(3, vec![psu(0, 1, 0, v, il), capacitor(1, 1, 0, c1), resistor(2, 1, 2, r), capacitor(3, 2, 0, c2), rating(2, p_max, r)]);
        let got = latch_of(&p, 2.0, t_end).expect("latched");
        close(got, want, 1e-3 * want, &format!("latch at il {il}, C1 {c1}, R {r}, C2 {c2}"));
    }
}

/// The time `S` first reaches `threshold` for `S' = k i² - c` (`S` held at
/// 0 while that is negative) with `i = i_inf (1 - e^(-t/tau))`: from `t1`,
/// where the rate turns positive, `S(t) = k i_inf² (G(t) - G(t1)) - c (t -
/// t1)`, `G(s) = s + 2 tau e^(-s/tau) - (tau/2) e^(-2s/tau)`.
fn charging_latch(i_inf: f64, tau: f64, k: f64, c: f64, threshold: f64) -> f64 {
    let g = |s: f64| s + 2.0 * tau * (-s / tau).exp() - tau / 2.0 * (-2.0 * s / tau).exp();
    let t1 = tau * (1.0 / (1.0 - (c / k).sqrt() / i_inf)).ln();
    let stress = |t: f64| k * i_inf * i_inf * (g(t) - g(t1)) - c * (t - t1);
    let (mut lo, mut hi) = (t1, t1 + 100.0 * tau);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if stress(mid) < threshold {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

#[test]
fn static_devices_on_a_current_limited_rail_cap_latch_at_the_closed_form_time_however_the_host_advances() {
    // G2bv2's w7b (ruling R30): a 6 V PSU limited to 0.8 A into a 10 mF rail
    // capacitor feeds a static series pair. The rail stays in CC (0.8 A
    // times the pair stays under 6 V), so it is one exponential and so is
    // every static node on it: the device's drive is exact, whatever the
    // host's chunks. The fuse (0.05 ohm, 0.5 A, with 2 ohm): S' = (i/0.5)² -
    // 1 to 0.3. The PTC (0.5 ohm, hold 0.5 A, with 4 ohm): S' = i² - 0.25 to
    // 0.075.
    let fuse_case = program(3, vec![psu(0, 1, 0, 6.0, 0.8), capacitor(1, 1, 0, 0.01), fuse(2, 1, 2, 0.05, 0.5), resistor(3, 2, 0, 2.0)]);
    let ptc_case = program(3, vec![psu(0, 1, 0, 6.0, 0.8), capacitor(1, 1, 0, 0.01), ptc(2, 1, 2, 0.5, 0.5), resistor(3, 2, 0, 4.0)]);
    let fuse_want = charging_latch(0.8, 2.05 * 0.01, 4.0, 1.0, 0.3);
    let ptc_want = charging_latch(0.8, 4.5 * 0.01, 1.0, 0.25, 0.075);
    close(fuse_want, 0.23528, 1e-5, "the fuse's closed form");
    close(ptc_want, 0.28638, 1e-5, "the PTC's closed form");
    for (name, p, channel, want) in [("fuse", &fuse_case, 0u32, fuse_want), ("ptc", &ptc_case, 2, ptc_want)] {
        let mut first = None;
        for step in [0.01, 1.0 / 60.0, 0.1, 0.37, 1.0] {
            let mut e = Engine::load(p).unwrap();
            let mut t = 0.0f64;
            while t < 1.0 {
                t = (t + step).min(1.0);
                e.advance(t).unwrap();
            }
            let got = e.records(channel).unwrap().chunks(7).find(|c| c[0] == 2.0).map(|c| c[3]).expect("it latched");
            close(got, want, 1e-3 * want, &format!("{name} at {step} s steps"));
            let first = *first.get_or_insert(got);
            close(got, first, 1e-9 * want, &format!("{name} at {step} s steps against 10 ms steps"));
        }
    }
}

#[test]
fn a_static_fuse_takes_the_cv_rails_constant_once_the_cc_segment_ends() {
    // R30 across a segment change: the PSU (6 V, 0.8 A, 10 mF) into 10 ohm
    // (a 0.05 ohm fuse rated 0.5 A, then 9.95 ohm) would rise to 8 V, so CC
    // ends at 6 V, t_c = 0.1 ln 4, and the rail holds there: the fuse's
    // exponential ends with the segment and its current stays at 0.6 A, S' =
    // 0.44. The latch is t_c + (0.3 - S(t_c)) / 0.44, however the host
    // advances.
    let p = program(3, vec![psu(0, 1, 0, 6.0, 0.8), capacitor(1, 1, 0, 0.01), fuse(2, 1, 2, 0.05, 0.5), resistor(3, 2, 0, 9.95)]);
    let tau = 0.1;
    let t_c = tau * 4f64.ln();
    let g = |s: f64| s + 2.0 * tau * (-s / tau).exp() - tau / 2.0 * (-2.0 * s / tau).exp();
    let t1 = tau * (1.0f64 / 0.375).ln();
    let s_c = 2.56 * (g(t_c) - g(t1)) - (t_c - t1);
    assert!(s_c < 0.3, "it latches in CV");
    let want = t_c + (0.3 - s_c) / 0.44;
    for step in [0.01, 1.0 / 60.0, 0.37, 1.0] {
        let mut e = Engine::load(&p).unwrap();
        let mut t = 0.0f64;
        while t < 1.0 {
            t = (t + step).min(1.0);
            e.advance(t).unwrap();
        }
        let got = e.records(0).unwrap().chunks(7).find(|c| c[0] == 2.0).map(|c| c[3]).expect("it latched");
        close(got, want, 1e-6 * want, &format!("the latch at {step} s steps"));
    }
}

#[test]
fn a_static_fuse_follows_the_rails_new_segment_when_a_rail_load_switches_in() {
    // R30's re-drive at a segment change: the CC rail (6 V, 0.8 A, 10 mF)
    // feeds a 0.05 ohm fuse (0.5 A) and 1.95 ohm. At 50 ms a switch puts 4
    // ohm straight across the rail: the rail is continuous, no node of the
    // fuse's partition moves at that instant, but the segment is new (tau
    // 20 ms to 13.3 ms, 1.6 V to 1.067 V). The fuse's current is u/2; its
    // latch (S' = (i/0.5)² - 1 to 0.3) by quadrature of that, in any steps.
    let p = program(4, vec![psu(0, 1, 0, 6.0, 0.8), capacitor(1, 1, 0, 0.01), fuse(2, 1, 2, 0.05, 0.5), resistor(3, 2, 0, 1.95), switch(4, 1, 3, 0.0), resistor(5, 3, 0, 4.0)]);
    // The closed switch's contact is 1 mohm, in series with the 4 ohm.
    let (t_sw, r_par) = (0.05, 2.0 * 4.001 / 6.001);
    let u_sw = 1.6 * (1.0 - (-t_sw / 0.02f64).exp());
    let u = |t: f64| if t <= t_sw { 1.6 * (1.0 - (-t / 0.02).exp()) } else { 0.8 * r_par + (u_sw - 0.8 * r_par) * (-(t - t_sw) / (0.01 * r_par)).exp() };
    let rate = |t: f64| { let i = u(t) / 2.0; (i / 0.5) * (i / 0.5) - 1.0 };
    let (mut st, mut t, h) = (0.0f64, 0.0f64, 1e-6);
    let want = loop {
        let (a, m, b) = (rate(t), rate(t + h / 2.0), rate(t + h));
        let ds = h / 6.0 * (a + 4.0 * m + b);
        let next = if st + ds > 0.0 || a > 0.0 { (st + ds).max(0.0) } else { 0.0 };
        if next >= 0.3 {
            break t + h * (0.3 - st) / (next - st);
        }
        st = next;
        t += h;
        assert!(t < 5.0, "it latches");
    };
    for step in [0.01, 1.0 / 60.0, 0.37, 1.0] {
        let mut e = Engine::load(&p).unwrap();
        e.advance(t_sw).unwrap();
        e.set_params(&[(4, 0, 1.0)]).unwrap();
        let mut t = t_sw;
        while t < 3.0 {
            t = (t + step).min(3.0);
            e.advance(t).unwrap();
        }
        let got = e.records(0).unwrap().chunks(7).find(|c| c[0] == 2.0).map(|c| c[3]).expect("it latched");
        close(got, want, 1e-5 * want, &format!("the latch at {step} s steps"));
    }
}

/// A `il`-limited 5 V PSU charging a `c1` rail capacitor with `r` straight
/// across the rail: the rail `u(t) = il r (1 - e^(-t/(r c1)))` in CC (written
/// with `expm1`, so no `r` cancels), until it reaches 5 V at `t_c`, then 5 V.
fn hi_r_rail(r: f64, c1: f64, il: f64) -> (impl Fn(f64) -> f64, f64) {
    let t_c = -r * c1 * (-5.0 / (il * r)).ln_1p();
    (move |t: f64| if t < t_c { -il * r * (-t / (r * c1)).exp_m1() } else { 5.0 }, t_c)
}

/// Composite Simpson of `f` over `[a, b]` in `n` (even) intervals.
fn simpson(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let h = (b - a) / n as f64;
    let inner: f64 = (1..n).map(|k| f(a + k as f64 * h) * if k % 2 == 1 { 4.0 } else { 2.0 }).sum();
    (f(a) + inner + f(b)) * h / 3.0
}

#[test]
fn a_rated_resistor_across_a_cc_rail_capacitor_latches_at_its_closed_form_time_at_any_resistance() {
    // W1gV3 (R30's exact path): `r` straight across a CC rail capacitor,
    // rated so that 5 V is twice its pMax. The rail's segment is `u0 +
    // slope φ_k(t - t0)` with `k = 1/(r c1)`, whose asymptote `il r` is far
    // past 5 V for a large `r`: `a + b e^(-s/tau)` then cancels (1.1e-3 late
    // at 1e9, 14x early at 1e10). The latch is `t_c + (1 - S(t_c))` (rate 1
    // in CV), `S(t_c)` by quadrature of the ramp, within the gate's latch
    // tolerance (1e-9 s plus 1e-9) however the host advances.
    for r in [1e6f64, 1e8, 1e9, 1e10, 1e12] {
        for (c1, il) in [(1e-3, 0.1), (1e-2, 0.5)] {
            let p_max = 12.5 / r;
            let (u, t_c) = hi_r_rail(r, c1, il);
            let rate = |t: f64| u(t) * u(t) / (r * p_max) - 1.0;
            let t1 = -r * c1 * (-(p_max * r).sqrt() / (il * r)).ln_1p();
            let want = t_c + (1.0 - simpson(rate, t1, t_c, 200_000));
            let p = program(2, vec![psu(0, 1, 0, 5.0, il), capacitor(1, 1, 0, c1), resistor(2, 1, 0, r), rating(2, p_max, r)]);
            for step in [0.01, 1.0 / 60.0, 0.37, 1.5] {
                let mut e = Engine::load(&p).unwrap();
                let mut t = 0.0f64;
                while t < 1.5 {
                    t = (t + step).min(1.5);
                    e.advance(t).unwrap();
                }
                let got = e.records(0).unwrap().chunks(7).find(|c| c[0] == 2.0).map(|c| c[3]).expect("it latched");
                close(got, want, 1e-9 + 1e-9 * want, &format!("r = {r:e}, c1 = {c1}, il = {il}, {step} s steps"));
            }
        }
    }
}

/// The latch of the rated device on a CC rail, by RK4 on the rail `u` (and
/// `w`, a second state the load may carry) in steps of `dt`: `deriv(u, w)`
/// gives the load's draw from the rail and `w'`, `volts(u, w)` the device's
/// voltage. CC until the rail reaches `v`, then CV; `S' = V^2/(r pMax) - 1`,
/// floored at 0.
#[allow(clippy::too_many_arguments)]
fn cc_rail_rk4_latch(v: f64, il: f64, c1: f64, r: f64, p_max: f64, end: f64, dt: f64, deriv: impl Fn(f64, f64) -> (f64, f64), volts: impl Fn(f64, f64) -> f64) -> Option<f64> {
    let (mut u, mut w, mut s) = (0.0f64, 0.0f64, 0.0f64);
    let f = |u: f64, w: f64| {
        let (draw, dw) = deriv(u, w);
        (if u < v { (il - draw) / c1 } else { 0.0 }, dw)
    };
    for k in 1..=(end / dt) as u64 {
        let (k1u, k1w) = f(u, w);
        let (k2u, k2w) = f(u + 0.5 * dt * k1u, w + 0.5 * dt * k1w);
        let (k3u, k3w) = f(u + 0.5 * dt * k2u, w + 0.5 * dt * k2w);
        let (k4u, k4w) = f(u + dt * k3u, w + dt * k3w);
        let p0 = volts(u, w).powi(2) / r;
        u = (u + dt / 6.0 * (k1u + 2.0 * k2u + 2.0 * k3u + k4u)).min(v);
        w += dt / 6.0 * (k1w + 2.0 * k2w + 2.0 * k3w + k4w);
        let p1 = volts(u, w).powi(2) / r;
        s = (s + dt * (0.5 * (p0 + p1) / p_max - 1.0)).max(0.0);
        if s >= 1.0 {
            return Some(k as f64 * dt);
        }
    }
    None
}

#[test]
fn a_rated_resistor_across_a_cc_rail_on_its_coupled_modes_latches_within_w0() {
    // R30's exact form is the rail's own CC segment, `u0 + slope φ_k`. With a
    // closed-form partition on the rail (10 ohm into 0.1 F) the rail and the
    // RC advance on their coupled modes (ruling R15 (a)), which that form
    // leaves out, so a device across the rail is held and re-anchored there,
    // its voltage to 1e-4 of itself (`DEVICE_TOL_REL`). 20 ohm rated 0.1 W
    // across a 5 V, 0.3 A PSU's 10 mF rail capacitor latches in CC, against
    // RK4 on the circuit, within twice that however the host advances (the
    // exact form, its modes dropped, was 4.6e-4 off at 0.37 s steps).
    let (v, il, c1, r2, c2, rx, pm) = (5.0, 0.3, 0.01, 10.0, 0.1, 20.0, 0.1);
    let want = cc_rail_rk4_latch(v, il, c1, rx, pm, 3.0, 1e-6, |u, w| ((u - w) / r2 + u / rx, (u - w) / (r2 * c2)), |u, _| u).expect("it latches");
    let p = program(3, vec![psu(0, 1, 0, v, il), capacitor(1, 1, 0, c1), resistor(2, 1, 2, r2), capacitor(3, 2, 0, c2), resistor(4, 1, 0, rx), rating(4, pm, rx)]);
    for step in [0.01, 1.0 / 60.0, 0.37, 3.0] {
        let mut e = Engine::load(&p).unwrap();
        let mut t = 0.0f64;
        while t < 3.0 {
            t = (t + step).min(3.0);
            e.advance(t).unwrap();
        }
        let got = e.records(0).unwrap().chunks(7).find(|c| c[0] == 4.0).map(|c| c[3]).expect("it latched");
        close(got, want, 2e-4 * want, &format!("the latch at {step} s steps"));
    }
}

#[test]
fn a_rated_resistor_into_an_led_on_a_cc_rail_is_held_on_the_leds_curve_and_latches_within_w0() {
    // R30's exact form takes each static hub node on its line `a + s u`; that
    // is the node only in a linear hub partition. Behind an LED the line is
    // the curve's tangent at the hub's last solve, so a device there is held
    // and re-anchored instead (R30): it is re-anchored while it accumulates,
    // where the exact form would schedule nothing. A 5 V PSU limited to 20 mA
    // into a 1 mF rail capacitor feeds 200 ohm rated 30 mW into a red LED
    // (16 mA at 5 V): the rail ramps slowly at the top, where the resistor's
    // voltage is the rail less the LED's drop. Against RK4 on the circuit
    // (the LED's operating point by bisection on its law), within W0 however
    // the host advances.
    let (v, il, c1, rx, pm) = (5.0, 0.02, 1e-3, 200.0, 0.03);
    let law = led::JunctionLaw::at(1.8, 0.02, 2.0, None, 25.0);
    let point = |u: f64| {
        let (mut lo, mut hi) = (0.0, u.max(0.0));
        for _ in 0..64 {
            let mid = 0.5 * (lo + hi);
            if u - rx * law.current(mid) - mid > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let vd = 0.5 * (lo + hi);
        (vd, law.current(vd))
    };
    let want = cc_rail_rk4_latch(v, il, c1, rx, pm, 5.0, 1e-5, |u, _| (point(u).1, 0.0), |u, _| u - point(u).0).expect("it latches");
    let p = program(3, vec![psu(0, 1, 0, v, il), capacitor(1, 1, 0, c1), resistor(2, 1, 2, rx), red_led(3, 2, 0), rating(2, pm, rx)]);
    for step in [0.01, 1.0 / 60.0, 0.37, 5.0] {
        let mut e = Engine::load(&p).unwrap();
        let mut t = 0.0f64;
        while t < 5.0 {
            t = (t + step).min(5.0);
            e.advance(t).unwrap();
        }
        let got = e.records(0).unwrap().chunks(7).find(|c| c[0] == 2.0).map(|c| c[3]).expect("it latched");
        close(got, want, 1e-3 * want, &format!("the latch at {step} s steps"));
        // Stat 22 counts the slow events fired: the latch, and the re-anchors.
        assert!(e.stat(22) > 10.0, "the resistor is held and re-anchored on the curve ({} slow events)", e.stat(22));
    }
}

#[test]
fn a_heated_resistor_across_a_cc_rail_capacitor_reads_its_closed_form_temperature_at_any_resistance() {
    // As above with a thermal record (100 K/W, 0.5 s) and no rating: T' =
    // (25 + 100 u²/r - T)/0.5, by RK4 on the exact rail. R30's exact path is
    // exact (point 3), so it is held far inside the readout tolerance: 1e-7
    // of the rise plus 1e-12 C, where the cancelling form read below ambient.
    for r in [1e3f64, 1e6, 1e9, 1e10, 1e12, 1e14] {
        let (c1, il) = (1e-2, 0.5);
        let (u, _) = hi_r_rail(r, c1, il);
        let p = program(2, vec![psu(0, 1, 0, 5.0, il), capacitor(1, 1, 0, c1), resistor(2, 1, 0, r), thermal(2, 100.0, 0.5)]);
        let mut e = Engine::load(&p).unwrap();
        // The rise above 25 C, so the reference keeps its digits at any r;
        // step `k` is at `k dt`, so the time cannot drift by a step.
        let (mut rise, mut k, dt) = (0.0f64, 0u32, 1e-5);
        let f = |t: f64, y: f64| (100.0 * u(t) * u(t) / r - y) / 0.5;
        for check in [0.01, 0.05, 0.2, 1.0, 3.0] {
            while (k as f64) * dt < check - 0.5 * dt {
                let t = k as f64 * dt;
                let k1 = f(t, rise);
                let k2 = f(t + dt / 2.0, rise + dt / 2.0 * k1);
                let k3 = f(t + dt / 2.0, rise + dt / 2.0 * k2);
                let k4 = f(t + dt, rise + dt * k3);
                rise += dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
                k += 1;
            }
            e.advance(check).unwrap();
            let got = e.device_state(2).unwrap()[0] - 25.0;
            close(got, rise, 1e-7 * rise + 1e-12, &format!("the rise at {check} s, r = {r:e}"));
        }
    }
}

#[test]
fn an_rc_with_a_ptc_behind_on_a_battery_latches_alike_however_the_host_advances() {
    // G2bv2's "battery rc + ptc behind" (5.5e-9 between chunkings in round
    // 5): a rated 20 ohm into 50 mF on a 6 V, 0.2 ohm battery, a PTC (1 ohm,
    // hold 0.2 A) and 30 ohm behind it. The rated resistor tracks the rail
    // (R29), and a closed-form device is visited only at its own events,
    // re-aims and anchors, so a host's chunk end that publishes the
    // partition changes nothing.
    let p = program(4, vec![battery(0, 1, 0, 6.0, 0.2), resistor(1, 1, 2, 20.0), capacitor(2, 2, 0, 0.05), ptc(3, 2, 3, 1.0, 0.2), resistor(4, 3, 0, 30.0), rating(1, 0.3, 20.0)]);
    let run = |step: f64| {
        let mut e = Engine::load(&p).unwrap();
        let mut t = 0.0f64;
        while t < 6.0 {
            t = (t + step).min(6.0);
            e.advance(t).unwrap();
        }
        let r = e.records(0).unwrap().to_vec();
        assert_eq!(r.len(), 7, "the resistor latched: {r:?}");
        (r[3], r[4], e.device_state(3).unwrap().to_vec())
    };
    let base = run(0.01);
    for step in [0.37, 1.0 / 60.0, 6.0] {
        let (t, v, st) = run(step);
        close(t, base.0, 1e-11 * base.0, &format!("the latch time at {step} s steps"));
        close(v, base.1, 1e-11 * base.1, &format!("the latch's power at {step} s steps"));
        for (a, b) in st.iter().zip(&base.2) {
            close(*a, *b, 1e-11 * (1.0 + b.abs()), &format!("the PTC's state at {step} s steps"));
        }
    }
}

#[test]
fn reset_failures_restores_the_resistor_which_relatches_one_second_later() {
    // Probe F `reset`: 0.1 A again after the reset, and a re-latch 1.0 s after
    // it.
    let mut e = Engine::load(&resistor_2x()).unwrap();
    e.advance(1.2).unwrap();
    assert_eq!(e.currents()[1], 0.0);
    e.reset_failures().unwrap();
    close(e.time(), 1.2, 0.0, "a reset does not advance time");
    close(e.currents()[1], 0.1, 1e-15, "0.1 A flows again");
    assert!(e.records(0).unwrap().is_empty(), "every latch cleared");
    e.advance(2.199).unwrap();
    assert!(e.records(0).unwrap().is_empty());
    e.advance(2.5).unwrap();
    let r = e.records(0).unwrap().to_vec();
    assert_eq!(r.len(), 7);
    close(r[3], 2.2, 1e-12, "re-latched 1.0 s after the reset");
    // The log kept the first latch across the reset, once per key.
    let log = e.records(3).unwrap().to_vec();
    assert_eq!(log.len(), 7);
    close(log[3], 1.0, 1e-12, "the first latch of the key since the last take");
}

// ---- thermal ----------------------------------------------------------------

#[test]
fn a_thermal_resistor_heats_by_the_one_pole() {
    // Probe F `resHeat`: 0.25 W into R_theta 340, tau 8: T(4 s) = 110 -
    // 85 e^-0.5 = 58.44489392442616 (classic within 5.5e-12).
    let mut rig = Rig::load(&program(2, vec![source(0, 1, 0, 5.0), resistor(1, 1, 0, 100.0), thermal(1, 340.0, 8.0)]));
    assert_eq!(rig.state(1), vec![25.0, 0.25, 0.0], "ambient at load, 0.25 W");
    rig.advance(4.0);
    let st = rig.state(1);
    close(st[0], 58.44489392442616, 1e-9, "T at 4 s");
    assert_eq!(st[1], 0.25);
    assert_eq!(st[2], 0.0, "no shutdown profile");
}

/// Probe F `ledHeat`: a red LED with an identity, 150 ohm from 5 V.
fn led_heat(no_cache: bool) -> Program {
    let mut p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 150.0), red_led(2, 2, 0), thermal(2, 560.0, 2.0)]);
    p.options.no_solution_cache = no_cache;
    p
}

#[test]
fn a_self_heating_led_follows_probe_f_ledheat() {
    // Classic: 21.3117 mA at 1 ms, 21.5875 mA and 46.1619 C at 10 s.
    // Temperatures within 1e-3 of the rise above ambient plus 1e-3 C.
    let mut rig = Rig::load(&led_heat(false));
    rig.advance(1e-3);
    close(rig.i(2), 0.0213117, 5e-8, "the current at 1 ms");
    rig.advance(10.0);
    let st = rig.state(2);
    let rise = 46.1619 - 25.0;
    close(st[0], 46.1619, 1e-3 * rise + 1e-3, "T at 10 s");
    close(rig.i(2), 0.0215875, 2e-7, "the current at 10 s");
    assert!(rig.stats()[3] > 0.0, "stat 24 counts the re-arms");
    // Re-arms follow the band: about one per 0.01 C of the 21 C rise.
    let rearms = rig.stats()[3];
    assert!(rearms > 0.5 * rise / THERMAL_BAND_C && rearms < 2.0 * rise / THERMAL_BAND_C, "{rearms} re-arms");
}

#[test]
fn after_a_rearm_the_partition_re_solves_as_a_cacheless_solve_does() {
    // The cache key carries no law: without the clear at each re-arm, the
    // partition's next solve would hit its entry from the old law and the
    // current would stay at its ambient value.
    let (mut cached, mut bare) = (Rig::load(&led_heat(false)), Rig::load(&led_heat(true)));
    for k in 1..=20 {
        let t = 0.5 * k as f64;
        cached.advance(t);
        bare.advance(t);
        close(cached.v(2), bare.v(2), 1e-6, &format!("the anode at {t} s"));
        close(cached.i(2), bare.i(2), 1e-9, &format!("the LED current at {t} s"));
    }
    assert!(cached.i(2) - 0.0213117 > 2e-4, "the current followed the law up: {}", cached.i(2));
}

#[test]
fn a_rearm_with_unchanged_terminal_voltages_reads_the_new_law_through_the_memo() {
    // A junction straight across 1.9 V: its voltage never moves, so the memo
    // keyed on the two voltages alone would serve the ambient law's current
    // after a re-arm. Keyed on `law_gen` too, the static solver's KCL reads
    // the new one.
    let p = program(2, vec![source(0, 1, 0, 1.9), red_led(1, 1, 0)]);
    let mut s = static_part::StaticPartitionSolver::build(&p).unwrap();
    s.resolve_dirty(&[], 0.0, &[]).unwrap();
    let ambient = led::JunctionLaw::at(1.8, 0.02, 2.0, None, 25.0);
    let i_ambient = ambient.current(1.9) + led::GMIN_D * 1.9;
    close(-s.currents()[0], i_ambient + static_part::NODE_RSHUNT_G * 1.9, 1e-18, "the source at ambient");
    let hot = led::led_law_at(&p.devices[1], 40.0).unwrap();
    s.mark_device_changed(1, static_part::StampState { failed: false, tripped: false, law: Some(hot), law_gen: 1 });
    s.resolve_dirty(&[], 0.0, &[]).unwrap();
    let i_hot = hot.current(1.9) + led::GMIN_D * 1.9;
    assert!((i_hot - i_ambient).abs() > 1e-4, "the law moved the current");
    close(-s.currents()[0], i_hot + static_part::NODE_RSHUNT_G * 1.9, 1e-18, "the source at the held law");
}

#[test]
fn an_led_across_two_volts_fails_fast_then_cools_to_ambient() {
    // `failedOpenLedCircuit`: a red LED with an identity straight across 2 V
    // carries about 1 A, so it fails within 0.02 s; then it dissipates
    // nothing, and its temperature relaxes to ambient.
    let mut rig = Rig::load(&program(2, vec![source(0, 1, 0, 2.0), red_led(1, 1, 0), thermal(1, 560.0, 2.0)]));
    rig.advance(0.02);
    let r = rig.records(0);
    assert_eq!(r.len(), 1, "latched before 0.02 s");
    assert!(r[0][3] < 0.02);
    rig.advance(1.0);
    let st1 = rig.state(1);
    assert_eq!(st1[1], 0.0, "P = 0 once failed");
    assert!(st1[0] > 25.0);
    rig.advance(1.5);
    assert!(rig.state(1)[0] < st1[0], "T falls");
    rig.advance(20.0);
    close(rig.state(1)[0], 25.0, 1e-3, "back to ambient after 20 s");
    assert_eq!(rig.i(1), 0.0);
    assert!(rig.i(0).abs() < 1e-9, "the source supplies nothing once the LED is open: {}", rig.i(0));
}

// ---- output_sag ----------------------------------------------------------------

/// A 74LS00 with 1A and 1B grounded, so 1Y is HIGH, into 100 ohm: `5 * 100 /
/// (100 + 156.25) = 1.9512195` V against `vih` 2.0 (probe F `sag`).
fn ls00_sag() -> Program {
    program(4, vec![source(0, 1, 0, 5.0), ic(kinds::IC_74LS00, 1, 1, 0, [0, 0, 2, 0, 0, 3, 0, 0, 3, 0, 0, 3], 1e-8, 0), resistor(2, 2, 0, 100.0)])
}

#[test]
fn a_sagging_74ls00_output_latches_one_window_after_load() {
    let mut rig = Rig::load(&ls00_sag());
    assert!(rig.records(0).is_empty(), "none at load: pending from START_TIME");
    rig.advance(0.5e-8);
    assert!(rig.records(0).is_empty(), "still inside the window");
    rig.advance(2e-8);
    let r = rig.records(0);
    assert_eq!(r.len(), 1, "{r:?}");
    let [dev, code, pin, since, value, limit, aux] = r[0];
    assert_eq!((dev, code, pin, aux), (1.0, 4.0, 4.0, 1.0), "output_sag on 1y (pin record 4), HIGH");
    close(since, 2e-8, 0.0, "a latched sag's since is the collect time");
    // 1.9512195 V: 5 V through R_out = 156.25 ohm into 100 ohm and the node's
    // shunt.
    let g = 1.0 / 156.25;
    close(value, 5.0 * g / (g + 0.01 + static_part::NODE_RSHUNT_G), 1e-12, "1.9512195 V");
    assert_eq!(limit, 2.0);
    let log = rig.records(3);
    assert_eq!(log.len(), 1);
    close(log[0][3], 1e-8, 1e-20, "latched at START_TIME + 1e-8, as classic's first step");
}

#[test]
fn a_load_switched_onto_a_quiet_high_output_latches_one_window_later() {
    // ls00_sag with the 100 ohm behind an open switch: 1Y sits HIGH and
    // unloaded, so every scan leaves it `Ok`. Closing the switch moves only
    // 1Y's node (no driver changes), and the sag starts at that instant: the
    // static publish must reach the scan however the cell was left.
    let mut e = Engine::load(&program(5, vec![source(0, 1, 0, 5.0), ic(kinds::IC_74LS00, 1, 1, 0, [0, 0, 2, 0, 0, 3, 0, 0, 3, 0, 0, 3], 1e-8, 0), switch(2, 2, 4, 0.0), resistor(3, 4, 0, 100.0)])).unwrap();
    e.advance(1e-3).unwrap();
    assert!(e.records(0).unwrap().is_empty(), "unloaded, no sag");
    e.set_params(&[(2, 0, 1.0)]).unwrap();
    e.advance(1e-3 + 0.5e-8).unwrap();
    assert!(e.records(0).unwrap().is_empty(), "still inside the window");
    e.advance(1e-3 + 2e-8).unwrap();
    let r = e.records(0).unwrap();
    assert_eq!(r.len(), 7, "{r:?}");
    assert_eq!((r[0], r[1], r[2], r[6]), (1.0, 4.0, 4.0, 1.0), "output_sag on 1y, HIGH");
    assert!(r[4] < 2.0, "below vih: {}", r[4]);
    let log = e.records(3).unwrap();
    assert_eq!(log.len(), 7);
    close(log[3], 1e-3 + 1e-8, 1e-18, "latched one window after the switch closed");
}

#[test]
fn a_load_switched_onto_an_output_with_a_resistive_supply_latches_one_window_later() {
    // As above, with the 74LS00's VCC fed through 1 ohm: its supply is a free
    // node, so each scan reads its power afresh. Closing the load moves 1Y's
    // node and VCC's, and the sag starts at that instant.
    let mut e = Engine::load(&program(6, vec![source(0, 5, 0, 5.0), ic(kinds::IC_74LS00, 1, 1, 0, [0, 0, 2, 0, 0, 3, 0, 0, 3, 0, 0, 3], 1e-8, 0), switch(2, 2, 4, 0.0), resistor(3, 4, 0, 100.0), resistor(4, 5, 1, 1.0)])).unwrap();
    e.advance(1e-3).unwrap();
    assert!(e.records(0).unwrap().is_empty(), "unloaded, no sag");
    e.set_params(&[(2, 0, 1.0)]).unwrap();
    e.advance(1e-3 + 0.5e-8).unwrap();
    assert!(e.records(0).unwrap().is_empty(), "still inside the window");
    e.advance(1e-3 + 2e-8).unwrap();
    let r = e.records(0).unwrap();
    assert_eq!(r.len(), 7, "{r:?}");
    assert_eq!((r[0], r[1], r[2], r[6]), (1.0, 4.0, 4.0, 1.0), "output_sag on 1y, HIGH");
    let log = e.records(3).unwrap();
    close(log[3], 1e-3 + 1e-8, 1e-18, "latched one window after the switch closed");
}

#[test]
fn a_flip_of_an_output_tied_to_ground_clears_its_sag_at_the_commit() {
    // 74LS04 1Y tied to ground: commanded HIGH it sags (0 V against vih 2.0)
    // and latches; its node never moves. Switching 1A high commits 1Y LOW
    // 1e-8 s later, where 0 V is no sag: the commit alone must reach the scan,
    // and the latch clears there.
    let mut e = Engine::load(&program(10, vec![source(0, 1, 0, 5.0), ic(kinds::IC_74LS04, 1, 1, 0, [2, 0, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9], 1e-8, 0), switch(2, 1, 2, 0.0), resistor(3, 2, 0, 1000.0)])).unwrap();
    e.advance(1e-3).unwrap();
    let pins = |e: &mut Engine| e.records(0).unwrap().chunks(7).map(|r| (r[2], r[6])).collect::<Vec<_>>();
    assert_eq!(pins(&mut e), vec![(3.0, 1.0)], "1y latched HIGH");
    e.set_params(&[(2, 0, 1.0)]).unwrap();
    e.advance(1e-3 + 0.5e-8).unwrap();
    assert_eq!(pins(&mut e), vec![(3.0, 1.0)], "before the commit");
    e.advance(1e-3 + 2e-8).unwrap();
    assert_eq!(pins(&mut e), vec![], "cleared at the commit");
}

/// A 74LS04 whose 1A is pulled down through 1k and switched to 5 V (device
/// 2), with `delay`; 1Y drives `load` (a device between node 3 and others),
/// and 2Y, HIGH, drives 10 ohm to 1.4 V on node 4 (a permanent HIGH sag:
/// 1.4 + 3.6 * 10 / 166.25 = 1.6165 V).
fn ls04_rig(delay: f64, load: Vec<Device>, sw: f64) -> Program {
    let mut devices = vec![
        source(0, 1, 0, 5.0),
        ic(kinds::IC_74LS04, 1, 1, 0, [2, 3, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9], delay, 0),
        switch(2, 1, 2, sw),
        resistor(3, 2, 0, 1000.0),
        source(4, 4, 0, 1.4),
        resistor(5, 5, 4, 10.0),
    ];
    devices.extend(load);
    program(12, devices)
}

#[test]
fn a_direction_flip_unpublishes_a_latched_sag_until_the_new_direction_has_held_a_window() {
    // 1Y into 10 ohm to 1.4 V sags both ways: HIGH at 1.6165 V (< 2.0), LOW at
    // 1.4 * 156.25 / 166.25 = 1.3158 V (> 0.8). Both outputs latch at 1e-8 in
    // pin order. A flip of 1Y's commanded direction unpublishes its latch at
    // the commit; it re-latches once LOW has held for 1e-8 s, at the end of
    // the order.
    let mut e = Engine::load(&ls04_rig(1e-8, vec![resistor(6, 3, 4, 10.0)], 0.0)).unwrap();
    e.advance(1e-3).unwrap();
    let order = |e: &mut Engine| e.records(0).unwrap().chunks(7).map(|r| (r[2], r[6])).collect::<Vec<_>>();
    assert_eq!(order(&mut e), vec![(3.0, 1.0), (5.0, 1.0)], "1y then 2y, both HIGH");
    e.set_params(&[(2, 0, 1.0)]).unwrap();
    // The inverter commits LOW 1e-8 s after the toggle.
    e.advance(1e-3 + 1.5e-8).unwrap();
    assert_eq!(order(&mut e), vec![(5.0, 1.0)], "1y unpublished while LOW is pending");
    e.advance(1e-3 + 1.9e-8).unwrap();
    assert_eq!(order(&mut e), vec![(5.0, 1.0)], "not yet: the window runs from the commit");
    e.advance(1e-3 + 2.1e-8).unwrap();
    assert_eq!(order(&mut e), vec![(5.0, 1.0), (3.0, 0.0)], "back, LOW, at the end of the order");
}

#[test]
fn a_window_and_a_flipping_commit_at_one_instant_latch_nothing_in_either_heap_order() {
    // README W1g point 9: the window event only marks the window due; the
    // settle decides after the instant's delta cycle, where the flip has
    // restarted it. Both orders put the window's end and the commit at
    // exactly 1e-8.
    //
    // Window first: 1Y sags HIGH from load (window event queued at load); a
    // toggle at t = 0 commits LOW at 1e-8, queued later.
    let mut a = Engine::load(&ls04_rig(1e-8, vec![resistor(6, 3, 4, 10.0)], 0.0)).unwrap();
    a.records(3).unwrap();
    a.set_params(&[(2, 0, 1.0)]).unwrap();
    // Commit first: 1Y is unloaded at load; one batch at t = 0 closes its load
    // (the sag, and its window event, start at that instant's settle) and
    // toggles the input (the commit, queued inside the instant's deltas).
    let mut b = Engine::load(&ls04_rig(1e-8, vec![switch(6, 3, 10, 0.0), resistor(7, 10, 4, 10.0)], 0.0)).unwrap();
    b.records(3).unwrap();
    assert!(b.records(0).unwrap().chunks(7).all(|r| r[2] != 3.0), "1y does not sag unloaded");
    b.set_params(&[(6, 0, 1.0), (2, 0, 1.0)]).unwrap();
    for (name, e) in [("window first", &mut a), ("commit first", &mut b)] {
        e.advance(1.5e-8).unwrap();
        let ch0: Vec<f64> = e.records(0).unwrap().chunks(7).map(|r| r[2]).collect();
        let ch3: Vec<f64> = e.records(3).unwrap().chunks(7).map(|r| r[2]).collect();
        assert!(!ch0.contains(&3.0), "{name}: no zero-length latch in the record: {ch0:?}");
        assert!(!ch3.contains(&3.0), "{name}: nor in the latch log: {ch3:?}");
        // LOW then holds a whole window from the flip, and latches.
        e.advance(2.5e-8).unwrap();
        let ch3: Vec<(f64, f64)> = e.records(3).unwrap().chunks(7).map(|r| (r[2], r[6])).collect();
        assert_eq!(ch3.iter().filter(|r| r.0 == 3.0).copied().collect::<Vec<_>>(), vec![(3.0, 0.0)], "{name}: LOW latches at 2e-8");
    }
}

#[test]
fn on_a_capacitive_edge_at_100_pf_low_edges_latch_and_high_edges_do_not() {
    // Probe C at 100 pF: 1Y into 100 pF through R_out = 156.25 ohm, tau =
    // 15.6 ns. Rising, it crosses vih 2.0 at tau ln(5/3) = 8.0 ns, inside the
    // window; falling, vil 0.8 at tau ln(5/0.8) = 28.6 ns, past it. So the
    // LOW edges latch and the HIGH ones (the power-on edge included) do not.
    let mut e = Engine::load(&ls04_rig(1e-8, vec![capacitor(6, 3, 0, 100e-12)], 0.0)).unwrap();
    e.advance(1e-3).unwrap();
    assert!(e.records(3).unwrap().chunks(7).all(|r| r[2] != 3.0), "the power-on HIGH edge does not latch");
    for k in 0..3 {
        let t = 1e-3 * (2 * k + 1) as f64;
        e.advance(t).unwrap();
        e.set_params(&[(2, 0, 1.0)]).unwrap();
        e.advance(t + 1e-4).unwrap();
        let low: Vec<[f64; 7]> = e.records(3).unwrap().chunks(7).map(|r| r.try_into().unwrap()).filter(|r: &[f64; 7]| r[2] == 3.0).collect();
        assert_eq!(low.len(), 1, "edge {k}: the LOW edge latches");
        assert_eq!(low[0][6], 0.0);
        close(low[0][3], t + 2e-8, 1e-12, "a window after the commit");
        e.advance(t + 1e-3).unwrap();
        e.set_params(&[(2, 0, 0.0)]).unwrap();
        e.advance(t + 1e-3 + 1e-4).unwrap();
        assert!(e.records(3).unwrap().chunks(7).all(|r| r[2] != 3.0), "edge {k}: the HIGH edge does not latch");
        assert!(e.records(0).unwrap().chunks(7).all(|r| r[2] != 3.0), "edge {k}: and the LOW latch has cleared");
    }
}

#[test]
fn a_latch_that_clears_within_one_advance_is_in_the_latch_log() {
    // 1Y into 100 ohm to ground sags only HIGH. Latched at 1e-8; a toggle at
    // t = 0 commits LOW 1 us later, which clears it, all inside one advance.
    let mut e = Engine::load(&ls04_rig(1e-6, vec![resistor(6, 3, 0, 100.0)], 0.0)).unwrap();
    e.set_params(&[(2, 0, 1.0)]).unwrap();
    e.advance(1e-3).unwrap();
    assert!(e.records(0).unwrap().chunks(7).all(|r| r[2] != 3.0), "cleared by the end of the advance");
    let log: Vec<[f64; 7]> = e.records(3).unwrap().chunks(7).map(|r| r.try_into().unwrap()).filter(|r: &[f64; 7]| r[2] == 3.0).collect();
    assert_eq!(log.len(), 1, "but the log saw it");
    close(log[0][3], 1e-8, 1e-20, "with its latch time");
    assert_eq!(log[0][6], 1.0, "HIGH");
    assert!(e.records(3).unwrap().is_empty(), "collecting empties it");
}

// ---- stats ----------------------------------------------------------------------

#[test]
fn stats_21_to_24_move() {
    let mut e = Engine::load(&led_heat(false)).unwrap();
    let before: Vec<f64> = (21..=24).map(|i| e.stat(i)).collect();
    e.advance(1.0).unwrap();
    assert!(e.stat(21) > before[0], "slow interval closes");
    assert!(e.stat(22) > before[1], "slow events fired");
    assert!(e.stat(24) > before[3], "junction re-arms");
    let mut r = Engine::load(&resistor_2x()).unwrap();
    r.advance(1.5).unwrap();
    assert_eq!(r.stat(23), 1.0, "one latch");
    assert!(e.stat(27).is_nan(), "past the table");
}

// ---- rollback ------------------------------------------------------------------

/// The rollback rig: the 2x resistor across the source (device 1, a fixed
/// pair, latching at 1 s), a self-heating LED on 150 ohm (device 3, in a
/// static partition), and the 74LS00's sagging 1Y (device 4).
fn rollback_rig() -> Program {
    program(
        6,
        vec![
            source(0, 1, 0, 5.0),
            resistor(1, 1, 0, 50.0),
            resistor(2, 1, 2, 150.0),
            red_led(3, 2, 0),
            ic(kinds::IC_74LS00, 4, 1, 0, [0, 0, 3, 0, 0, 4, 0, 0, 4, 0, 0, 4], 1e-8, 0),
            resistor(5, 3, 0, 100.0),
            rating(1, 0.25, 50.0),
            thermal(3, 560.0, 2.0),
        ],
    )
}

/// Everything W1g's rollback must restore, as bits: the latch order and
/// log, the resistor's stress, the LED's thermal entry (`T_hold`), law and
/// `law_gen` wherever its law is read, every `output_sag` cell, the failed
/// flags, and the LED partition's cache.
fn slow_fingerprint(s: &PartitionedSolver) -> Vec<String> {
    let mut out = vec![format!("latched {:?}", s.slow.latched()), format!("log {:?}", s.slow.latch_log())];
    out.push(format!("stress {:?}", s.slow.stress(1)));
    out.push(format!("thermal {:?}", s.slow.thermal(3)));
    out.push(format!("law_gen {} {}", s.slow.law_gen(1), s.slow.law_gen(3)));
    out.push(format!("failed {} {}", s.slow.is_failed(1), s.slow.is_failed(3)));
    out.push(format!("junctions {:?}", s.junctions));
    out.push(format!("led law {:?}", s.led_devices.iter().map(|e| e.junctions.clone()).collect::<Vec<_>>()));
    out.push(format!("static {:?}", s.static_solver.junction_laws()));
    out.push(format!("sag {:?}", (0..s.slow.sag_len()).map(|c| s.slow.sag_cell(c).state).collect::<Vec<_>>()));
    out.push(format!("cache {:?}", s.static_solver.cache_bits()));
    out
}

#[test]
fn an_instant_that_latches_re_arms_and_resets_then_fails_restores_every_slow_bit() {
    // README W1g point 15. A doomed window: the resistor's latch (the latch
    // order, the log, the failed flag, the stress, a fixed-pair republish),
    // the LED's re-arm (`T_hold`, its law, `law_gen`, a re-solve that stores
    // a cache entry under the new law), a reset (every sag cell cleared and
    // rescanned, the latches cleared) and a settle, then a rollback. Every
    // slow bit must come back, and the run must go on exactly as a twin that
    // never had the window.
    let p = rollback_rig();
    let (mut doomed, mut twin) = (Rig::load(&p), Rig::load(&p));
    for rig in [&mut doomed, &mut twin] {
        rig.advance(0.5);
        rig.records(3);
    }
    let before = slow_fingerprint(doomed.s());
    let drivers: Vec<crate::drivers::Driver> = (0..4).map(|i| *doomed.k.driver(crate::events::DriverId(i))).collect();
    {
        let s = doomed.k.solver_mut();
        let cp = s.begin_instant();
        let mut dirty = s.handle_slow_event(DeviceId(1), 0.5).unwrap();
        dirty.extend(s.handle_slow_event(DeviceId(3), 0.5).unwrap());
        assert!(s.slow.is_failed(1), "the latch happened");
        dirty.sort_unstable();
        dirty.dedup();
        s.resolve_dirty(&dirty, 0.5, &drivers).unwrap();
        let reset = s.reset_failures(0.5).unwrap();
        s.resolve_dirty(&reset, 0.5, &drivers).unwrap();
        s.slow_settle(0.5, &drivers).unwrap();
        assert_ne!(slow_fingerprint(s), before, "the window changed slow state");
        s.rollback_instant(cp);
    }
    let after = slow_fingerprint(doomed.s());
    for (a, b) in before.iter().zip(&after) {
        assert_eq!(a, b, "a rollback restores it bit for bit");
    }
    for k in 1..=40 {
        let t = 0.5 + 0.05 * k as f64;
        doomed.advance(t);
        twin.advance(t);
        let bits = |r: &Rig| (r.s().voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), r.s().currents().iter().map(|v| v.to_bits()).collect::<Vec<_>>());
        assert_eq!(bits(&doomed), bits(&twin), "at {t} s");
        assert_eq!(doomed.records(0), twin.records(0), "at {t} s");
        assert_eq!(doomed.records(3), twin.records(3), "at {t} s");
        assert_eq!(doomed.state(3), twin.state(3), "at {t} s");
    }
}

#[test]
fn a_failed_instant_that_scanned_a_released_pin_leaves_its_later_sag_seen() {
    // A failed instant's scan must leave nothing behind: here the doomed
    // window scans 1Y with its driver reset (no target), and after the
    // rollback 1Y is HIGH again; closing a load onto it moves only its node,
    // and must start its window as in a twin that never had the instant.
    let p = program(5, vec![source(0, 1, 0, 5.0), ic(kinds::IC_74LS00, 1, 1, 0, [0, 0, 2, 0, 0, 3, 0, 0, 3, 0, 0, 3], 1e-8, 0), switch(2, 2, 4, 0.0), resistor(3, 4, 0, 100.0)]);
    let (mut doomed, mut twin) = (Rig::load(&p), Rig::load(&p));
    for rig in [&mut doomed, &mut twin] {
        rig.advance(1e-3);
    }
    let mut drivers: Vec<crate::drivers::Driver> = (0..4).map(|i| *doomed.k.driver(crate::events::DriverId(i))).collect();
    drivers[0].reset();
    {
        let s = doomed.k.solver_mut();
        let cp = s.begin_instant();
        s.driver_changing(0);
        s.driver_committed(0);
        assert!(s.slow_settle(1e-3, &drivers).unwrap().is_none());
        s.rollback_instant(cp);
    }
    for rig in [&mut doomed, &mut twin] {
        rig.k.apply_params_as_event(&[(2, 0, 1.0)]).unwrap();
        rig.advance(1e-3 + 2e-8);
    }
    assert_eq!(twin.records(0).len(), 1, "the twin latches 1y");
    assert_eq!(doomed.records(0), twin.records(0));
    assert_eq!(doomed.records(3), twin.records(3));
}

#[test]
fn a_failed_instant_that_tripped_a_ptc_leaves_it_as_a_twins() {
    // A doomed window trips the PTC early (its entry, its stamp, channel 2's
    // `since`); after the rollback its state reads as a twin's, before and
    // after the real trip at 0.1 s. (`since` itself is written at every trip
    // and read only while tripped, so its undo cannot show here.)
    let p = program(2, vec![source(0, 1, 0, 0.5), ptc(1, 1, 0, 0.5, 0.5)]);
    let (mut doomed, mut twin) = (Rig::load(&p), Rig::load(&p));
    for rig in [&mut doomed, &mut twin] {
        rig.advance(0.05);
    }
    {
        let s = doomed.k.solver_mut();
        let cp = s.begin_instant();
        s.handle_slow_event(DeviceId(1), 0.05).unwrap();
        assert_eq!(s.slow.ptc(1).map(|p| p.tripped), Some(true), "the window tripped it");
        s.rollback_instant(cp);
    }
    assert_eq!(doomed.state(1), twin.state(1), "right after the rollback");
    for t in [0.09, 0.11, 1.0, 2.5] {
        doomed.advance(t);
        twin.advance(t);
        assert_eq!(doomed.state(1), twin.state(1), "at {t} s");
        assert_eq!(doomed.records(0), twin.records(0), "at {t} s");
    }
}

/// A 74LS04 with no delay whose 1Y feeds a fuse (rated 1 mA, carrying 4.3 mA
/// into 1k, so it trips at about 17 ms) and, through a switch, its own 1A:
/// with the fuse open the loop is broken, and a reset that restores it
/// closes a zero-delay ring.
fn ring_behind_a_fuse() -> Program {
    program(
        12,
        vec![
            source(0, 1, 0, 5.0),
            ic(kinds::IC_74LS04, 1, 1, 0, [2, 3, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9], 0.0, 0),
            fuse(2, 3, 4, 0.1, 0.001),
            resistor(3, 4, 0, 1000.0),
            switch(4, 4, 2, 0.0),
            resistor(5, 2, 0, 10000.0),
        ],
    )
}

#[test]
fn a_reset_whose_re_solve_fails_leaves_the_engine_exactly_as_it_was() {
    // README W1g point 13 and Q6: un-failing the fuse closes a zero-delay
    // ring, the reset's delta cycle livelocks (-7), and the reset rolls back
    // with its instant.
    let run = |reset_attempt: bool| {
        let mut e = Engine::load(&ring_behind_a_fuse()).unwrap();
        e.advance(0.05).unwrap();
        e.set_params(&[(4, 0, 1.0)]).unwrap();
        e.advance(0.06).unwrap();
        let snapshot = |e: &mut Engine| {
            (
                e.time().to_bits(),
                e.voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
                e.currents().iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
                e.records(0).unwrap().iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
                e.digital_slots().iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
            )
        };
        let before = snapshot(&mut e);
        assert_eq!(before.3.len(), 7, "the fuse has tripped");
        if reset_attempt {
            let err = e.reset_failures().unwrap_err();
            assert_eq!(err.code, crate::error::ERR_DELTA_LIVELOCK, "{}", err.message);
            assert_eq!(snapshot(&mut e), before, "exactly as it was");
        }
        e.advance(0.1).unwrap();
        (snapshot(&mut e), e.records(3).unwrap().to_vec())
    };
    assert_eq!(run(true), run(false), "and it goes on as an engine that never tried");
}


#[test]
fn the_led_fast_path_is_exact_at_its_threshold() {
    // At or under the safe voltage the damage rate the full evaluation gives
    // is exactly 0, and just over it the current exceeds `ifMax`: the visit's
    // shortcut skips the exponential without changing a single drive.
    for (vf, n, i_rated) in [(1.8, 2.0, 0.02), (2.0, 1.0, 0.05), (3.2, 2.0, 0.02), (1.8, 0.5, 0.001)] {
        let law = led::JunctionLaw::at(vf, i_rated, n, None, 25.0);
        let v = settle::safe_voltage(&law, i_rated);
        assert!(law.current(v) <= i_rated, "vf {vf}: under the limit at the safe voltage");
        assert!(law.current(v + 4.0 * f64::EPSILON * v.abs().max(1.0)) > i_rated, "vf {vf}: the largest such voltage, to a few ulp");
        for k in 0..=1000 {
            let u = -5.0 + (v + 5.0) * k as f64 / 1000.0;
            let peak = led::peak_from_currents(kinds::LED, &[law.current(u)]);
            assert_eq!(crate::slow::failures::led_failed(peak, i_rated).rate, 0.0, "vf {vf} at {u} V");
        }
    }
    let law = led::JunctionLaw::at(1.8, 0.02, 2.0, None, 25.0);
    assert_eq!(settle::safe_voltage(&law, 0.0), f64::INFINITY, "no limit: always safe");
}


#[test]
fn a_resistor_charging_a_capacitor_latches_at_the_closed_form_time() {
    // README W1g point 5, closed-form segments: 5 V through 50 ohm (rated
    // 0.25 W) into 1 F. V_R(t) = 5 e^{-t/50}, so the rate is 2 e^{-0.04 t} - 1
    // and S(t) = 50 (1 - e^{-0.04 t}) - t, which reaches 1 at the root below
    // (bisection on the model's own closed form).
    let s = |t: f64| 50.0 * (1.0 - (-0.04 * t).exp()) - t;
    let (mut lo, mut hi) = (0.5, 2.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if s(mid) < 1.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let t_x = 0.5 * (lo + hi);
    let mut rig = Rig::load(&program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 50.0), capacitor(2, 2, 0, 1.0), rating(1, 0.25, 50.0)]));
    assert_eq!(rig.s().partition_kinds[0], PartitionKind::ClosedForm);
    rig.advance(t_x - 1e-6);
    assert!(rig.records(0).is_empty());
    rig.advance(t_x + 1e-6);
    let r = rig.records(0);
    assert_eq!(r.len(), 1);
    close(r[0][3], t_x, 1e-9 + 1e-9 * t_x, "the closed-form latch time");
    let v_r = 5.0 * (-t_x / 50.0f64).exp();
    close(r[0][4], v_r * v_r / 50.0, 1e-9, "P at the latch, on the trajectory");
    assert_eq!(rig.i(1), 0.0);
}

#[test]
fn a_self_heating_led_in_a_stepped_partition_holds_the_last_steps_temperature() {
    // README W1g point 4, stepped partitions: `T_hold := T` after every
    // committed step, with no event and no re-seed. An LED and 100 nF behind
    // 150 ohm from 5 V is stepped (a junction and a capacitor); its held
    // temperature follows the one-pole one step behind, so the current at
    // 10 s is the heated law's, as in the static case.
    let mut rig = Rig::load(&program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 150.0), red_led(2, 2, 0), capacitor(3, 2, 0, 100e-9), thermal(2, 560.0, 2.0)]));
    assert_eq!(rig.s().partition_kinds[0], PartitionKind::Stepped);
    rig.advance(10.0);
    let st = rig.state(2);
    close(st[0], 46.1619, 1e-3 * 21.1619 + 1e-3, "T at 10 s, as the static probe");
    let th = rig.s().slow.thermal(2).unwrap();
    assert!((th.t_hold - st[0]).abs() < 0.05, "T_hold lags T by at most a step's heating: {} against {}", th.t_hold, st[0]);
    assert!(!th.cfg.holds_junction, "a stepped LED schedules no boundary");
    assert!(rig.stats()[3] > 100.0, "re-armed at committed steps");
    // Within W0's current tolerance of classic's 21.5875 mA: the stepped
    // partition's held law lags a step, as classic's does.
    close(rig.i(2), 0.0215875, 1e-9 + 1e-3 * 0.0215875, "the heated current");
}

/// A 5 V cell (0.9 ohm, a one-point table) with a red LED straight from its
/// rail to ground, which it overdrives until it fails, and 1k beside it.
fn led_on_a_cell(exact: bool) -> Program {
    let mut params = vec![0.0; kinds::battery_param::COUNT];
    params[kinds::battery_param::OCV_SCALE] = 1.0;
    params[kinds::battery_param::R_BASE] = 0.9;
    params[kinds::battery_param::TEMP_FACTOR] = 1.0;
    params[kinds::battery_param::DEP_P] = 2.0;
    params[kinds::battery_param::DEP_K] = 0.05;
    params[kinds::battery_param::CAPACITY_AH] = 1.0;
    params[kinds::battery_param::SOC0] = 1.0;
    let cell = Device { kind: kinds::BATTERY_PACK, component_index: 0, pins: vec![pin(1), pin(0)], params, blob: vec![1.0, 0.5, 5.0] };
    let mut p = program(2, vec![cell, red_led(1, 1, 0), resistor(2, 1, 0, 1000.0)]);
    p.options.no_rail_shortcut = exact;
    p
}

#[test]
fn a_failed_led_straight_from_the_rail_to_a_fixed_node_draws_nothing() {
    // README W1g point 12: a failed LED stamps no junction at all, GMIN
    // included. Outside exact mode it is a direct device of the hub, whose
    // draw `direct_at` sums; in exact mode the rail's own KCL
    // (`exact_hub_current`) reads it. Either must see only the resistor and
    // the node's shunt, in the order they are summed.
    let mut rig = Rig::load(&led_on_a_cell(false));
    rig.advance(0.1);
    assert_eq!(rig.records(0).iter().map(|r| r[1]).collect::<Vec<_>>(), vec![3.0], "the LED failed");
    let u = rig.v(1);
    let gc = 1.0 / 1000.0;
    let (i, g) = rig.s().direct_at(u);
    assert_eq!(i.to_bits(), (static_part::NODE_RSHUNT_G * u + gc * (u - 0.0)).to_bits(), "the draw: the resistor and the shunt alone");
    assert_eq!(g.to_bits(), (static_part::NODE_RSHUNT_G + gc).to_bits(), "its slope");

    let mut rig = Rig::load(&led_on_a_cell(true));
    rig.advance(0.1);
    assert_eq!(rig.records(0).iter().map(|r| r[1]).collect::<Vec<_>>(), vec![3.0], "the LED failed in exact mode");
    let u = rig.v(1);
    let leaving = static_part::NODE_RSHUNT_G * u + gc * (u - 0.0);
    let held = rig.s().exact.as_ref().unwrap().held;
    assert_eq!(held.to_bits(), leaving.to_bits(), "exact mode's draw: the resistor and the shunt alone");
}






// ---- R38 (a): cold devices ------------------------------------------------------

/// A 0/5 V clock of `freq` Hz (half duty) from `pos` to `neg`.
fn clock_src(ci: usize, pos: usize, neg: usize, freq: f64) -> Device {
    Device { kind: kinds::CLOCK, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![freq, 0.5, 5.0, 0.0, 0.0], blob: vec![0.0] }
}

/// The span bound of `p`, from its own partition build.
fn span_of(p: &Program) -> Option<(f64, f64)> {
    let pb = partition::build(p).expect("builds");
    super::settle::voltage_span_bound(p, &pb.resolved, &pb.fixed, &pb.live, &pb.kinds)
}

/// `p` with every rating's `pMax` at 0: the same circuit with no rated
/// resistor, which W1g's slow physics leaves alone.
fn unrated(p: &Program) -> Program {
    let mut devices = p.devices.clone();
    for d in &mut devices {
        if d.kind == kinds::RATING {
            d.params[kinds::rating_param::P_MAX] = 0.0;
        }
    }
    program(p.node_count, devices)
}

/// Which slow devices (by device index) loaded cold.
fn cold_devices(p: &Program) -> Vec<usize> {
    let rig = Rig::load(p);
    rig.s().slow_model.devs.iter().filter(|d| d.cold).map(|d| d.device).collect()
}

#[test]
fn the_span_bound_covers_every_source_and_declines_a_capacitor_that_could_pump() {
    // R38 (a): `hi - lo` over the potentials the sources can give a fixed
    // node or the rail, and the most any node's own can be.
    let r = |a, b| resistor(9, a, b, 1e3);
    assert_eq!(span_of(&program(2, vec![source(0, 1, 0, 5.0), r(1, 0)])), Some((5.0, 5.0)));
    // Stacked sources add; one below the datum widens the span downward.
    assert_eq!(span_of(&program(3, vec![source(0, 1, 0, 5.0), source(1, 2, 1, 3.0), r(2, 0)])), Some((8.0, 8.0)));
    assert_eq!(span_of(&program(3, vec![source(0, 1, 0, 5.0), source(1, 0, 2, 3.0), r(1, 2)])), Some((8.0, 5.0)));
    // A clock's two levels, a battery's greatest OCV, an ideal PSU's value.
    assert_eq!(span_of(&program(2, vec![clock_src(0, 1, 0, 1e3), r(1, 0)])), Some((5.0, 5.0)));
    assert_eq!(span_of(&program(2, vec![battery(0, 1, 0, 9.0, 0.1), r(1, 0)])), Some((9.0, 9.0)));
    assert_eq!(span_of(&program(2, vec![psu(0, 1, 0, 12.0, 0.0), capacitor(1, 1, 0, 1e-3), r(1, 0)])), Some((12.0, 12.0)));
    // A current-limited PSU's rail is a solve, which no bound may lean on.
    assert_eq!(span_of(&program(2, vec![psu(0, 1, 0, 12.0, 0.5), capacitor(1, 1, 0, 1e-3), r(1, 0)])), None);
    // A capacitor from a free node to node 0 keeps the bound; one between
    // two free nodes, or from a free node to a moving one, can pump a node
    // past every source, and there is no bound.
    assert!(span_of(&program(3, vec![source(0, 1, 0, 5.0), r(1, 2), capacitor(2, 2, 0, 1e-6)])).is_some());
    assert_eq!(span_of(&program(4, vec![source(0, 1, 0, 5.0), r(1, 2), capacitor(2, 2, 3, 1e-6), resistor(3, 3, 0, 1e3)])), None);
    assert_eq!(span_of(&program(3, vec![clock_src(0, 1, 0, 1e3), capacitor(1, 1, 2, 1e-6), r(2, 0)])), None);
}

#[test]
fn a_resistance_in_series_through_a_bare_node_carries_its_share_of_the_span() {
    // 5 V into 1 kohm, a node nothing else touches, then 1 mohm into 1 uF:
    // the 1 mohm carries at most the span over 1000.001 ohm, times itself.
    let p = program(4, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 1e3), resistor(2, 2, 3, 1e-3), capacitor(3, 3, 0, 1e-6)]);
    let pb = partition::build(&p).unwrap();
    let (span, v_abs) = super::settle::voltage_span_bound(&p, &pb.resolved, &pb.fixed, &pb.live, &pb.kinds).unwrap();
    let mut pins_on = vec![0u32; p.node_count];
    let mut devs_on = vec![Vec::new(); p.node_count];
    for (e, d) in p.devices.iter().enumerate() {
        for q in &d.pins {
            pins_on[q.node] += 1;
            devs_on[q.node].push(e as u32);
        }
    }
    let bound = super::settle::resistance_volts_bound(&p, 2, &pins_on, &devs_on, &pb.fixed, span, v_abs);
    close(bound, 1e-3 * (5.0 / 1000.001 + static_part::NODE_RSHUNT_G * 5.0), 1e-18, "the 1 mohm's share");
    // The 1 kohm's walk ends at the capacitor's node: it carries the share
    // of a 1000.001 ohm chain too, nearly the whole span.
    close(super::settle::resistance_volts_bound(&p, 1, &pins_on, &devs_on, &pb.fixed, span, v_abs), 5.0 * 1e3 / 1000.001 + 1e3 * static_part::NODE_RSHUNT_G * 5.0, 1e-12, "the 1 kohm's share");
    // A capacitor on the middle node takes current off the chain: the walk
    // stops there and the bound is the span.
    let mut devices = p.devices.clone();
    devices.push(capacitor(4, 2, 0, 1e-9));
    let q = program(4, devices);
    pins_on[2] += 1;
    devs_on[2].push(4);
    devs_on[0].push(4);
    assert_eq!(super::settle::resistance_volts_bound(&q, 2, &pins_on, &devs_on, &pb.fixed, span, v_abs), span);
}

/// A 555-style timing chain on a 1 kHz clock, rated as the compiler rates
/// every resistor (0.25 W): 1 kohm into a bare node, 1 mohm into 1 uF, and
/// 220 ohm straight across the clock. Every one is cold: 5 V can put 0.114 W
/// at most across 220 ohm, and the 1 mohm carries its share of the chain.
fn cold_timing_chain() -> Program {
    program(
        4,
        vec![
            clock_src(0, 1, 0, 1e3),
            resistor(1, 1, 2, 1e3),
            resistor(2, 2, 3, 1e-3),
            capacitor(3, 3, 0, 1e-6),
            resistor(4, 1, 0, 220.0),
            rating(1, 0.25, 1e3),
            rating(2, 0.25, 1e-3),
            rating(4, 0.25, 220.0),
        ],
    )
}

#[test]
fn cold_resistors_are_never_visited_and_run_as_the_unrated_circuit() {
    // R38 (a): a cold device's integrator stays at 0 under its idle drive at
    // every voltage the circuit can reach, so no publish notes it and it
    // schedules nothing: no close (stat 21), no slow event (22), and the run
    // is the unrated circuit's, event for event and bit for bit.
    let p = cold_timing_chain();
    assert_eq!(cold_devices(&p), vec![1, 2, 4]);
    let (mut e, mut twin) = (Engine::load(&p).unwrap(), Engine::load(&unrated(&p)).unwrap());
    for k in 1..=50 {
        let t = k as f64 * 2e-3;
        e.advance(t).unwrap();
        twin.advance(t).unwrap();
        assert_eq!(e.voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), twin.voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), "voltages at {t}");
    }
    assert_eq!((e.stat(21), e.stat(22)), (0.0, 0.0), "no close and no slow event");
    assert_eq!(e.stat(4), twin.stat(4), "the unrated twin's events");
    assert!(e.records(0).unwrap().is_empty());
    // At its limit a device is not cold: 5 V across 100 ohm rated 0.25 W is
    // exactly its pMax, and the widened bound puts it over.
    assert_eq!(cold_devices(&program(2, vec![source(0, 1, 0, 5.0), resistor(1, 1, 0, 100.0), rating(1, 0.25, 100.0)])), Vec::<usize>::new());
    // A thermal identity keeps a device hot: its readout is the interval's
    // power, which only a visit keeps.
    assert_eq!(cold_devices(&program(2, vec![source(0, 1, 0, 5.0), resistor(1, 1, 0, 1e3), rating(1, 0.25, 1e3), thermal(1, 100.0, 0.5)])), Vec::<usize>::new());
}

#[test]
fn an_led_whose_series_resistance_keeps_it_under_its_limit_is_cold() {
    // R38 (a): a red LED (1.8 V, 20 mA) behind 220 ohm from 5 V, as the
    // SAP-1's 83: over 20 mA its junction is past its safe voltage and the
    // resistor holds 4.4 V, more than 5 V can give, so it never fails and is
    // never visited; the run is the same as when it is visited at every
    // publish (100 ohm: 2 V + 1.8 V fits under 5 V, so that LED is hot).
    let lit = |r: f64| program(4, vec![clock_src(0, 1, 0, 1e3), resistor(1, 1, 2, r), red_led(2, 2, 0), resistor(3, 1, 3, 1e3), red_led(4, 3, 0)]);
    assert_eq!(cold_devices(&lit(220.0)), vec![2, 4]);
    assert_eq!(cold_devices(&lit(100.0)), vec![4]);
    // A capacitor on the node between them breaks the chain.
    let mut capped = lit(220.0).devices;
    capped.push(capacitor(5, 2, 0, 1e-9));
    assert_eq!(cold_devices(&program(4, capped)), vec![4]);
    let mut e = Engine::load(&lit(220.0)).unwrap();
    e.advance(0.05).unwrap();
    assert_eq!((e.stat(21), e.stat(22)), (0.0, 0.0), "no close and no slow event");
}

#[test]
fn a_closed_form_resistor_whose_segments_stay_under_its_level_stores_no_drive() {
    // R38 (a): 10 ohm rated 0.25 W (rate positive above 1.58 V) is not cold,
    // since 5 V could put 2.5 W across it, but in this RC on a 1 kHz clock it
    // carries tens of millivolts: each re-aim's segment stays under its level
    // for good, so the visit stores nothing (stat 21), as the idle drive it
    // would replace bleeds the same, and schedules nothing. The node behind
    // it has three pins, so no series walk makes it cold.
    let p = program(
        4,
        vec![
            clock_src(0, 1, 0, 1e3),
            resistor(1, 1, 2, 1e3),
            capacitor(2, 2, 0, 1e-6),
            resistor(3, 2, 3, 10.0),
            resistor(4, 3, 0, 1e3),
            resistor(5, 3, 1, 1e3),
            rating(3, 0.25, 10.0),
        ],
    );
    assert!(!cold_devices(&p).contains(&3));
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.05).unwrap();
    assert!(e.stat(8) > 50.0, "the RC re-aims at every clock edge ({} segments)", e.stat(8));
    assert_eq!((e.stat(21), e.stat(22)), (0.0, 0.0), "no stored drive and no slow event");
}

#[test]
fn no_cold_device_ever_sees_more_than_its_bound_on_random_circuits() {
    // R38 (a)'s soundness, on small random circuits of the kinds the bound
    // covers (sources stacked or not, clocks, resistors, LEDs, capacitors to
    // node 0, closed switches): at every frame, each cold resistor's voltage
    // is within the widened bound it was classified at, each cold LED carries
    // at most its 20 mA, and no cold device ever latches. Ratings are drawn about the power the span
    // could put across each resistor, so both classes occur.
    let mut seed = 0x9E37_79B9_7F4A_7C15u64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let (mut cold_seen, mut hot_seen, mut cold_leds) = (0, 0, 0);
    for case in 0..60 {
        let nodes = 3 + (next() * 4.0) as usize;
        let mut devices = vec![source(0, 1, 0, 1.0 + 11.0 * next())];
        if next() < 0.5 {
            devices.push(clock_src(devices.len(), 2, 1, 200.0 + 2e3 * next()));
        }
        let first_free = devices.len() + 1;
        let mut rated = Vec::new();
        for _ in 0..(3 + (next() * 6.0) as usize) {
            let (a, b) = ((next() * nodes as f64) as usize, (next() * nodes as f64) as usize);
            if a == b {
                continue;
            }
            let ci = devices.len();
            match (next() * 10.0) as u32 {
                0 => devices.push(red_led(ci, a, b)),
                1 => devices.push(capacitor(ci, a.max(b), 0, 1e-7 * (1.0 + 100.0 * next()))),
                2 => devices.push(switch(ci, a, b, 1.0)),
                _ => {
                    let r = 10f64.powf(-3.0 + 7.0 * next());
                    devices.push(resistor(ci, a, b, r));
                    rated.push((ci, r));
                }
            }
        }
        let _ = first_free;
        // An LED or two behind a resistor through a bare node, as on a
        // display: cold or hot by the resistor's size.
        let mut extra = 0;
        for _ in 0..(next() * 3.0) as usize {
            let k = nodes + extra;
            extra += 1;
            let ci = devices.len();
            let r = 30.0 + 2e3 * next();
            devices.push(resistor(ci, 1 + (next() * (nodes - 1) as f64) as usize, k, r));
            rated.push((ci, r));
            devices.push(red_led(ci + 1, k, 0));
        }
        let nodes = nodes + extra;
        let span_guess = 20.0;
        for &(ci, r) in &rated {
            devices.push(rating(ci, span_guess * span_guess / r * 10f64.powf(-3.0 + 3.0 * next()), r));
        }
        let p = program(nodes.max(3), devices);
        let Ok(solver) = PartitionedSolver::build(&p) else { continue };
        let Some((span, v_abs)) = ({
            let pb = partition::build(&p).unwrap();
            super::settle::voltage_span_bound(&p, &pb.resolved, &pb.fixed, &pb.live, &pb.kinds)
        }) else {
            continue;
        };
        let (span_w, abs_w) = (span * (1.0 + super::settle::COLD_MARGIN_REL) + super::settle::COLD_MARGIN_V, v_abs * (1.0 + super::settle::COLD_MARGIN_REL) + super::settle::COLD_MARGIN_V);
        let pb = partition::build(&p).unwrap();
        let mut pins_on = vec![0u32; p.node_count];
        let mut devs_on = vec![Vec::new(); p.node_count];
        for (e, d) in p.devices.iter().enumerate() {
            for q in &d.pins {
                pins_on[q.node] += 1;
                if devs_on[q.node].last() != Some(&(e as u32)) {
                    devs_on[q.node].push(e as u32);
                }
            }
        }
        // A cold resistor's bound is its voltage; a cold LED's is its
        // current, `ifMax` (marked by a negative entry).
        let cold: Vec<(usize, f64)> = solver
            .slow_model
            .devs
            .iter()
            .filter(|d| d.cold)
            .map(|d| if d.kind == kinds::LED { (d.device, -0.02) } else { (d.device, super::settle::resistance_volts_bound(&p, d.device, &pins_on, &devs_on, &pb.fixed, span_w, abs_w)) })
            .collect();
        let law = led::JunctionLaw::at(1.8, 0.02, 2.0, None, 25.0);
        cold_seen += cold.len();
        cold_leds += cold.iter().filter(|c| c.1 < 0.0).count();
        hot_seen += solver.slow_model.devs.iter().filter(|d| !d.cold && d.kind == kinds::RESISTOR && d.p_max > 0.0).count();
        let Ok(mut e) = Engine::load(&p) else { continue };
        for k in 1..=40 {
            if e.advance(k as f64 * 5e-4).is_err() {
                break;
            }
            for &(dev, bound) in &cold {
                let pins = &p.devices[dev].pins;
                let v = e.voltages()[pins[0].node] - e.voltages()[pins[1].node];
                if bound < 0.0 {
                    let i = law.current(v);
                    assert!(i.abs() <= -bound, "case {case}: cold LED {dev} carries {i} A over its 20 mA");
                } else {
                    assert!(v.abs() <= bound, "case {case}: cold device {dev} at {v} V over its bound {bound}");
                }
            }
        }
        let latched: Vec<f64> = e.records(0).unwrap().chunks(7).map(|c| c[0]).collect();
        assert!(cold.iter().all(|(d, _)| !latched.contains(&(*d as f64))), "case {case}: a cold device latched");
    }
    assert!(cold_seen > 30 && hot_seen > 30 && cold_leds > 5, "both classes occur ({cold_seen} cold, {cold_leds} of them LEDs, {hot_seen} hot)");
}

/// The failure records (as bits) of `p` run through `script` (advance to
/// each time, then set that entry's params), with cold devices and as its
/// visited twin, every device hot.
fn records_cold_and_hot(p: &Program, script: &[(f64, Vec<(u32, u32, f64)>)]) -> (Vec<u64>, Vec<u64>) {
    let run = || {
        let mut e = Engine::load(p).unwrap();
        for (t, params) in script {
            e.advance(*t).unwrap();
            if !params.is_empty() {
                e.set_params(params).unwrap();
            }
        }
        e.records(0).unwrap().iter().map(|v| v.to_bits()).collect::<Vec<_>>()
    };
    let cold = run();
    let hot = {
        let _guard = super::settle::cold_hooks::no_cold();
        run()
    };
    (cold, hot)
}

/// A 555 with its supply and output on `rail`, both its timing pins on one
/// node (`5`) and its discharge on node 3, as W1gV4's minimal 2.
fn ne555_on(ci: usize, rail: usize) -> Device {
    let mut params = vec![0.0; ic_param::COUNT];
    params[ic_param::VCC_MIN] = 2.0;
    params[ic_param::VCC_MAX] = 18.0;
    params[ic_param::NOMINAL] = 5.0;
    Device { kind: kinds::NE555, component_index: ci, pins: [rail, 0, 0, 5, 5, 3, 0, rail].iter().map(|&n| pin(n)).collect(), params, blob: vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] }
}

#[test]
fn a_load_on_a_current_limited_rail_latches_alike_cold_or_visited_wherever_the_rail_is_published() {
    // R38 (a), W1gV4 finding 1: the bound once held a current-limited PSU's
    // rail to [0, V], but the rail is a solve. (a) Default mode, a 555 on an
    // 11.29 V / 0.21 A rail that a switch shorts and then opens: the rail was
    // published at 209.5 V, and the 1 kohm load rated 0.25 W (0.13 W at
    // 11.29 V) latched at 0.306 s when visited and never when cold. (b) Exact
    // mode, a 5 V / 0.1 A PSU back-driven by 9 V through 10 ohm: the rail was
    // published at 9.901 V, and the 1 kohm rated 85 mW (25 mW at 5 V) latched
    // at 6.52 s when visited and never when cold. However the rail solve
    // comes out, the cold run must be the visited run.
    let mut a = program(6, vec![psu(0, 1, 0, 11.2858, 0.2095), switch(1, 1, 0, 1.0), ne555_on(2, 1), resistor(3, 1, 0, 1e3), rating(3, 0.25, 1e3)]);
    a.options.no_rail_shortcut = false;
    let script: Vec<(f64, Vec<(u32, u32, f64)>)> = std::iter::once((0.3, vec![(1, 0, 0.0)])).chain((1..=7).map(|k| (0.3 + 0.1 * k as f64, vec![]))).collect();
    let (cold, hot) = records_cold_and_hot(&a, &script);
    assert_eq!(cold, hot, "default mode: the cold run latches as the visited one");
    assert_eq!(cold_devices(&a), Vec::<usize>::new(), "no device on a current-limited rail is cold");

    let mut b = program(3, vec![psu(0, 1, 0, 5.0, 0.1), source(1, 2, 0, 9.0), resistor(2, 1, 2, 10.0), resistor(3, 1, 0, 1e3), rating(3, 0.085, 1e3)]);
    b.options.no_rail_shortcut = true;
    let (cold, hot) = records_cold_and_hot(&b, &[(10.0, vec![])]);
    assert_eq!(cold, hot, "exact mode: the cold run latches as the visited one");
    assert_eq!(cold_devices(&b), Vec::<usize>::new(), "no device on a current-limited rail is cold");
}

#[test]
fn a_trapezoidal_ring_past_the_supply_latches_its_load_cold_or_visited_and_backward_euler_keeps_it_cold() {
    // R38 (a), W1gV4 finding 2: the trapezoidal rule has no maximum
    // principle. A capacitor charged from 12 V through an LED (1.63 V, 31
    // mA, as fuzz seed 860's; a stiff edge, so the first steps ring) lands near 15.8 V and holds it, discharging
    // through 100 Mohm for seconds. The 100 Mohm rated for 12.5 V (just over
    // the widened 12 V span) latched at 1.76 s when visited and never when
    // cold; now no trapezoidal program has a bound. Under backward Euler the
    // node never passes the supply and the same resistor stays cold.
    let rc = |trap: bool| {
        let mut p = program(3, vec![source(0, 1, 0, 12.0), two(kinds::LED, 1, 1, 2, vec![1.63, 1.85, 0.031, 0.0, 0.0, 0.0]), capacitor(2, 2, 0, 5.4e-7), resistor(3, 2, 0, 1e8), rating(3, 12.5 * 12.5 / 1e8, 1e8)]);
        p.options.integration = trap as u8;
        p
    };
    let script: Vec<(f64, Vec<(u32, u32, f64)>)> = (1..=10).map(|k| (0.5 * k as f64, vec![])).collect();
    let trap = rc(true);
    let (cold, hot) = records_cold_and_hot(&trap, &script);
    assert!(!hot.is_empty(), "the ring past the supply latches the visited resistor");
    assert_eq!(cold, hot, "the cold run latches as the visited one");
    assert_eq!(span_of(&trap), None, "no bound under the trapezoidal rule");
    assert_eq!(cold_devices(&trap), Vec::<usize>::new());

    let be = rc(false);
    assert_eq!(cold_devices(&be), vec![3], "backward Euler keeps it cold");
    let mut e = Engine::load(&be).unwrap();
    for &(t, _) in &script {
        e.advance(t).unwrap();
        assert!(e.voltages()[2] <= 12.0, "backward Euler holds the node under its supply ({} V at {t})", e.voltages()[2]);
    }
    // Ruling R71 (b): the LED latches where its stress crosses, 5.8 ns in,
    // as 12 V across it into the empty capacitor drives it (1990 A); latched
    // at the end of that step from the step's end current, it never did.
    let latched: Vec<f64> = e.records(0).unwrap().chunks(7).map(|c| c[0]).collect();
    assert!(!latched.contains(&3.0), "the 100 Mohm stays intact: {latched:?}");
}

#[test]
fn the_cold_test_reads_the_rate_at_the_span_widened_by_its_margin() {
    // R38 (a): 5 V across 100 ohm is 0.25 W; widened by 1 % plus 10 mV it is
    // 5.06 V, 0.256 W. Rated 0.2502 W the resistor is safe at the bare span
    // but not at the widened one, so it is hot: the margin is what stands
    // between a published voltage a step's error has lifted past the span and
    // a missed latch. Rated 0.2561 W it is safe at the widened span: cold.
    let at = |p_max: f64| cold_devices(&program(2, vec![source(0, 1, 0, 5.0), resistor(1, 1, 0, 100.0), rating(1, p_max, 100.0)]));
    assert_eq!(at(0.2502), Vec::<usize>::new(), "inside the margin: hot");
    assert_eq!(at(0.2561), vec![1], "past the margin: cold");
}

#[test]
fn a_kind_the_bound_does_not_cover_gets_no_bound() {
    // W1gV4: kind 8 is the reserved potentiometer code. The reader rejects
    // it, but a hand-built Program can hold it, and the bound must decline a
    // kind it has not been shown to cover rather than skip it.
    let pot = Device { kind: kinds::POTENTIOMETER, component_index: 2, pins: vec![pin(1), pin(2)], params: vec![1e3], blob: vec![] };
    let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 2, 0, 1e3), pot]);
    assert_eq!(span_of(&p), None, "an unknown kind must decline the bound");
}

// ---- ACC-E: the driver mirror across a fixed-pair republish ----------------

/// A 5 V source (node 1), a resistor at twice its rating on the fixed pair
/// 1-0 (it latches open at t = 1 s and its slow event republishes the fixed
/// devices), and a 74LS04 whose 1A (node 2) is pulled up to the rail through
/// 1 kohm and whose 1Y sits on node 3, held at 2.5 V by a second source: a
/// driver on a fixed node, so no partition re-solves when it commits, and the
/// one reading that shows its level is that source's current. (The other five
/// gates read HIGH and pull ground LOW, which draws nothing.) The load
/// settle's first pass finds 1A LOW (nothing has solved yet) and commits 1Y
/// HIGH; its re-solve pulls 1A up, and the second pass commits 1Y LOW. The
/// switch (2-0, open) is what makes a later kernel resolve.
fn an_inverter_beside_a_latching_resistor() -> Program {
    program(
        4,
        vec![
            source(0, 1, 0, 5.0),
            resistor(1, 1, 0, 50.0),
            rating(1, 0.25, 50.0),
            ic(kinds::IC_74LS04, 3, 1, 0, [2, 3, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0], 1e-8, 0),
            resistor(4, 1, 2, 1e3),
            switch(5, 2, 0, 0.0),
            source(6, 3, 0, 2.5),
        ],
    )
}

/// The entries in which the solver's copy of the driver table differs from
/// the kernel's.
fn lagging_entries(rig: &Rig) -> Vec<usize> {
    let s = rig.s();
    (0..s.driver_snapshot.len()).filter(|&i| !same_driver(&s.driver_snapshot[i], rig.k.driver(crate::events::DriverId(i as u32)))).collect()
}

#[test]
fn a_fixed_pair_republish_leaves_a_commit_the_solver_has_not_resolved_for_the_next_resolve() {
    // Cold 64 tpl 5 (hub 2) in a debug build: "driver 0 changed without
    // `driver_changing`". The settle's last commit lands on a fixed-node
    // driver and no resolve follows it, so the kernel's notice for 1Y waits.
    // The latch's republish resolved with the solver's own (older) table,
    // which used the notice up; the next kernel resolve then found 1Y
    // changed behind its back, and a release build kept 1Y HIGH for good.
    let p = an_inverter_beside_a_latching_resistor();
    let mut rig = Rig::load(&p);
    assert_eq!(rig.k.driver(crate::events::DriverId(0)).committed(), Some(crate::events::Mode::Low), "1A is pulled up, 1Y is LOW");
    let behind = lagging_entries(&rig);
    assert!(behind.iter().all(|i| rig.s().stale.contains(&(*i as u32))), "every entry the solver's table lags in is noted: {behind:?}");
    // Past the latch (a republish), then a resolve the kernel makes.
    rig.advance(1.5);
    assert!(rig.records(0).iter().any(|r| r[0] == 1.0 && r[1] == 1.0), "the resistor latched");
    let behind = lagging_entries(&rig);
    assert!(behind.iter().all(|i| rig.s().stale.contains(&(*i as u32))), "the republish kept every notice it could not deliver: {behind:?}");
    rig.k.apply_params_as_event(&[(5, 0, 1.0)]).unwrap();
    rig.advance(1.6);
    assert_eq!(rig.k.driver(crate::events::DriverId(0)).committed(), Some(crate::events::Mode::High), "1A is grounded, 1Y is HIGH");
    assert_eq!(lagging_entries(&rig), Vec::<usize>::new(), "the solver's table after the toggle");
}

#[test]
fn a_fixed_node_driver_the_load_settle_commits_last_is_in_the_solvers_table_and_its_kcl() {
    // The settle's last pass commits 1Y LOW and dirties no partition (its
    // node is fixed): without a resolve after it, the published currents and
    // the solver's table kept 1Y HIGH until some later event resolved
    // (32 mA on the 2.5 V source here).
    let p = an_inverter_beside_a_latching_resistor();
    let rig = Rig::load(&p);
    assert_eq!(lagging_entries(&rig), Vec::<usize>::new(), "the solver's table after load");
    let mut twin = Rig::load(&p);
    let table: Vec<crate::drivers::Driver> = (0..twin.s().driver_snapshot.len()).map(|i| *twin.k.driver(crate::events::DriverId(i as u32))).collect();
    twin.k.solver_mut().resolve_dirty(&[], 0.0, &table).unwrap();
    twin.k.solver_mut().prepare_readout();
    assert_eq!(rig.i(6).to_bits(), twin.i(6).to_bits(), "the 2.5 V source's current at load: {} against {}", rig.i(6), twin.i(6));
}
