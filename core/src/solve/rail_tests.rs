//! W1f's rails and sources through the real kernel (core/README.md, "Kernel
//! semantics: rails and sources (W1f)"). Every expected value comes from the
//! model's closed form, a probe of classic, or fixed classic's stated rule,
//! never from this implementation; each test says which.

use super::*;
use crate::devices::{ics, led};
use crate::devices::sources::Regime;
use crate::engine::Engine;
use crate::error::{BoError, ERR_UNSUPPORTED_TOPOLOGY};
use crate::kernel::{DeviceEvaluator, DriverRequest, Kernel, KernelError};
use crate::kinds::{battery_param, ic_param};
use crate::program::{Device, Options, PinRecord};
use crate::rails;

// ---- builders ---------------------------------------------------------------

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
    Device { kind: kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
}

fn capacitor(ci: usize, a: usize, b: usize, c: f64, esr: f64) -> Device {
    Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![c, esr, 0.0], blob: vec![] }
}

fn switch(ci: usize, a: usize, b: usize, closed: bool) -> Device {
    Device { kind: kinds::SWITCH, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![if closed { 1.0 } else { 0.0 }], blob: vec![] }
}

fn led(ci: usize, anode: usize, cathode: usize) -> Device {
    // vf 2.0 V, n 2, 20 mA rated, no Is override, not burnt.
    Device { kind: kinds::LED, component_index: ci, pins: vec![pin(anode), pin(cathode)], params: vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0], blob: vec![] }
}

fn anchor(ci: usize, node: usize) -> Device {
    Device { kind: kinds::SECTION_ANCHOR, component_index: ci, pins: vec![pin(node)], params: vec![], blob: vec![] }
}

/// A battery whose OCV is the constant `e` (a one-point table) and whose
/// resistance is the constant `r` (no depletion), 1 Ah.
fn battery(ci: usize, pos: usize, neg: usize, e: f64, r: f64) -> Device {
    battery_with(ci, pos, neg, r, 0.0, 1.0, 1.0, vec![1.0, 0.5, e])
}

/// A battery with the generic shape (depletion 0.5, knee 0.05), the generic
/// table `[(0, 0), (1, e)]`.
fn generic_battery(ci: usize, pos: usize, neg: usize, e: f64, r: f64, capacity_ah: f64) -> Device {
    battery_with(ci, pos, neg, r, 0.5, capacity_ah, 1.0, vec![2.0, 0.0, 0.0, 1.0, e])
}

fn battery_with(ci: usize, pos: usize, neg: usize, r: f64, dep_a: f64, capacity_ah: f64, soc0: f64, table: Vec<f64>) -> Device {
    let mut params = vec![0.0; battery_param::COUNT];
    params[battery_param::OCV_SCALE] = 1.0;
    params[battery_param::R_BASE] = r;
    params[battery_param::TEMP_FACTOR] = 1.0;
    params[battery_param::DEP_A] = dep_a;
    params[battery_param::DEP_P] = 2.0;
    params[battery_param::DEP_K] = 0.05;
    params[battery_param::CAPACITY_AH] = capacity_ah;
    params[battery_param::SOC0] = soc0;
    Device { kind: kinds::BATTERY_PACK, component_index: ci, pins: vec![pin(pos), pin(neg)], params, blob: table }
}

fn psu(ci: usize, pos: usize, neg: usize, v: f64, i_limit: f64) -> Device {
    Device { kind: kinds::BENCH_PSU, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v, i_limit], blob: vec![] }
}

fn clock(ci: usize, pos: usize, neg: usize, freq: f64, duty: f64, slot: usize) -> Device {
    Device { kind: kinds::CLOCK, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![freq, duty, 5.0, 0.0, 0.0], blob: vec![slot as f64] }
}

fn pulse(ci: usize, pos: usize, neg: usize, params: [f64; 7], slot: usize) -> Device {
    Device { kind: kinds::PULSE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: params.to_vec(), blob: vec![slot as f64] }
}

/// A `74ls00` (12 logic pins `1a 1b 1y ... 4y`), LS-TTL params, `delay` its
/// propagation delay, digital slots from `slot`.
fn ls00(ci: usize, vcc: usize, gnd: usize, logic: [usize; 12], delay: f64, slot: usize) -> Device {
    let mut pins = vec![pin(vcc), pin(gnd)];
    pins.extend(logic.iter().map(|&n| pin(n)));
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
    let mut blob = vec![slot as f64];
    blob.extend((0..12).map(|i| if i % 3 == 2 { 0.0 } else { kinds::PIN_FLOATING_INPUT as f64 }));
    Device { kind: kinds::IC_74LS00, component_index: ci, pins, params, blob }
}

fn program(node_count: usize, devices: Vec<Device>) -> Program {
    let digital = devices.iter().map(|d| match d.kind {
        kinds::IC_74LS00 => d.blob[0] as usize + 4,
        kinds::CLOCK | kinds::CLOCK_GEN | kinds::PULSE_SOURCE | kinds::PULSE_GEN => d.blob[0] as usize + 1,
        _ => 0,
    });
    let digital_slot_count = digital.max().unwrap_or(0);
    Program { node_count, options: Options::default(), digital_slot_count, channel_slot_count: 0, devices }
}

fn exact(mut p: Program) -> Program {
    p.options.no_rail_shortcut = true;
    p
}

struct NoDevices;
impl DeviceEvaluator for NoDevices {
    fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
        Vec::new()
    }
}

/// The real kernel with the real IC evaluator over a concrete
/// [`PartitionedSolver`], so a test can read the rail's own state.
struct Rig {
    k: Kernel<PartitionedSolver, Box<dyn DeviceEvaluator>>,
}

impl Rig {
    fn load(p: &Program) -> Result<Rig, BoError> {
        Rig::load_carrying(p, &[])
    }

    /// [`Rig::load`] with capacitor `device` loaded charged to `vc` (its
    /// `pins[0]` minus its `pins[1]`) for each `(device, vc)`, as a reload
    /// that carries its charge will load it (W1b point 5: W2). Only a
    /// capacitor across fixed or rail nodes is supported.
    fn load_carrying(p: &Program, charges: &[(usize, f64)]) -> Result<Rig, BoError> {
        let mut solver = PartitionedSolver::build(p)?;
        for &(device, vc) in charges {
            let cap = solver.fixed_pair_capacitors.iter_mut().find(|c| c.device_index == device).expect("a capacitor across fixed or rail nodes");
            cap.vc0 = vc;
        }
        let all: Vec<PartitionId> = (0..solver.partition_count() as u32).map(PartitionId).collect();
        let mut k = Kernel::new(solver, Box::new(NoDevices) as Box<dyn DeviceEvaluator>);
        let ev = ics::register(&mut k, p);
        let readers = ev.reader_ids();
        k.set_evaluator(Box::new(ev));
        k.settle(p.options.start_time, &all, &readers).map_err(kernel_error)?;
        k.solver_mut().prepare_readout();
        Ok(Rig { k })
    }

    fn advance(&mut self, t: f64) -> Result<(), KernelError> {
        let r = self.k.advance(t);
        self.k.solver_mut().prepare_readout();
        r
    }

    fn v(&self, node: usize) -> f64 {
        self.k.solver().voltages()[node]
    }

    fn i(&self, device: usize) -> f64 {
        self.k.solver().currents()[device]
    }

    fn s(&self) -> &PartitionedSolver {
        self.k.solver()
    }

    fn hub(&self) -> &hub::Hub {
        self.s().hub.as_ref().expect("a hub rail")
    }

    fn u(&self) -> f64 {
        self.hub().state.u
    }

    fn state(&self, device: usize) -> Vec<f64> {
        let mut out = Vec::new();
        self.s().device_state(device, self.k.time(), &mut out).expect("a device with state");
        out
    }
}

fn kernel_error(e: KernelError) -> BoError {
    match e {
        KernelError::Livelock { at } => BoError::delta_livelock(format!("livelock at {at}")),
        KernelError::SolveFailed(m) => BoError::solve_failed(m),
    }
}

fn close(got: f64, want: f64, tol: f64, what: &str) {
    assert!((got - want).abs() <= tol, "{what}: got {got:.12e}, want {want:.12e}, off by {:.3e} (tolerance {tol:.1e})", (got - want).abs());
}

fn rel(got: f64, want: f64, tol: f64, what: &str) {
    close(got, want, tol * want.abs(), what);
}

// ---- Rails point 3: sensitivity ------------------------------------------------

#[test]
fn a_divider_off_the_rail_has_the_closed_form_sensitivity_draw_and_slope() {
    // Ra = 100 (rail to x), Rb = 300 (x to ground) on a 5 V, 1 ohm battery:
    // s = Rb/(Ra+Rb), I = u_p/(Ra+Rb), g = 1/(Ra+Rb) (draft, F1's first test).
    let (ra, rb) = (100.0, 300.0);
    let p = program(3, vec![battery(0, 1, 0, 5.0, 1.0), resistor(1, 1, 2, ra), resistor(2, 2, 0, rb)]);
    let rig = Rig::load(&p).unwrap();
    let pid = rig.s().local_index[2].unwrap().0;
    let hub = rig.s().static_solver.hub_solve(pid).expect("the divider is a static hub partition");
    let (x, s, i, g, u_p) = hub;
    rel(s[0], rb / (ra + rb), 1e-9, "s");
    rel(i, u_p / (ra + rb), 1e-9, "I");
    rel(g, 1.0 / (ra + rb), 1e-9, "g");
    // With x's node shunt the divider is exact: x = u (1/Ra) / (1/Ra + 1/Rb + 1e-12).
    let ratio = (1.0 / ra) / (1.0 / ra + 1.0 / rb + static_part::NODE_RSHUNT_G);
    rel(x[0], u_p * ratio, 1e-12, "x at u_p");
    rel(s[0], ratio, 1e-12, "s with the shunt");
    // The rail itself, and the published node, rail-corrected (Rails point 3).
    let u = 5.0 * (ra + rb) / (ra + rb + 1.0);
    rel(rig.u(), u, 1e-10, "u");
    rel(rig.v(1), rig.u(), 0.0, "the rail node publishes u");
    rel(rig.v(2), rig.u() * ratio, 1e-12, "x + s (u - u_p)");
    // The battery's current: -(u/(Ra+Rb) + the node shunts) into its +.
    rel(rig.i(0), -(u / (ra + rb)) - 1e-12 * (u + rig.v(2)), 1e-9, "battery current");
}

#[test]
fn a_linear_rail_solves_in_one_pass_and_agrees_with_exact_mode() {
    // E 5, r 1, loads of 100 and 50 ohm in two static partitions (each a
    // divider of two halves): u = 5 * (100||50) / ((100||50) + 1) = 500/103.
    let devices = || {
        vec![
            battery(0, 1, 0, 5.0, 1.0),
            resistor(1, 1, 2, 50.0),
            resistor(2, 2, 0, 50.0),
            resistor(3, 1, 3, 25.0),
            resistor(4, 3, 0, 25.0),
        ]
    };
    let rig = Rig::load(&program(4, devices())).unwrap();
    let want = 500.0 / 103.0;
    // The node shunts move it by a part in 1e11.
    rel(rig.u(), want, 1e-10, "u");
    assert_eq!(rig.hub().stats.moves, 1, "one rail solve moves the rail from its open-circuit guess, and the linear partitions keep their correction");
    let ex = Rig::load(&exact(program(4, devices()))).unwrap();
    close(rig.v(1), ex.v(1), 1e-12 * want, "default against exact mode");
    close(rig.v(2), ex.v(2), 1e-12 * want, "node 2 against exact mode");
    close(rig.v(3), ex.v(3), 1e-12 * want, "node 3 against exact mode");
}

// ---- Rails point 4: rail events ------------------------------------------------

#[test]
fn a_coupled_rc_through_the_rail_stays_within_two_drift_widths_of_the_coupled_closed_form() {
    // E 5, r 1, R 1k from the rail to x, C 1 uF from x to ground (draft F1):
    // the capacitor charges through R + r, tau' = 1.001 ms, vc = 5 (1 -
    // e^(-t/tau')), u = 5 - (5 - vc) r/(R + r).
    let (e, r, big_r, c) = (5.0, 1.0, 1000.0, 1e-6);
    let p = program(3, vec![battery(0, 1, 0, e, r), resistor(1, 1, 2, big_r), capacitor(2, 2, 0, c, 0.0)]);
    let mut rig = Rig::load(&p).unwrap();
    let tau = (big_r + r) * c;
    let coupled = |t: f64| {
        let vc = e * (1.0 - (-t / tau).exp());
        (vc, e - (e - vc) * r / (big_r + r))
    };
    let bound = 2.0 * rails::RAIL_DRIFT_V;
    let (mut worst_vc, mut worst_u): (f64, f64) = (0.0, 0.0);
    for k in 1..=50 {
        let t = 5.0 * tau * k as f64 / 50.0;
        rig.advance(t).unwrap();
        let (vc, u) = coupled(t);
        close(rig.v(2), vc, bound, &format!("vc at {t:e}"));
        close(rig.v(1), u, bound, &format!("u at {t:e}"));
        worst_vc = worst_vc.max((rig.v(2) - vc).abs());
        worst_u = worst_u.max((rig.v(1) - u).abs());
        if k == 10 {
            // The draft's sample at t = tau'.
            close(rig.v(2), 3.160603, 1e-6 + bound, "vc(tau')");
            close(rig.v(1), 4.998162, 1e-6 + bound, "u(tau')");
        }
    }
    eprintln!("coupled RC: worst |vc - coupled| {worst_vc:.3e}, worst |u - coupled| {worst_u:.3e}, rail events {}", rig.hub().stats.events);
}

#[test]
fn the_power_gate_flips_at_the_rail_event_that_puts_u_on_the_threshold() {
    // E 4.279, r 5, R 1k from the rail to a 1 uF capacitor, a 74LS00 on the
    // rail with its outputs unloaded and its inputs tied to vcc (draft F1), so
    // powered or not it draws nothing from the rail. The rail starts at
    // E R/(R + r) = 4.25771, below 0.9 * 4.75, and the coupled closed form
    // crosses 4.275 at tau' ln(r E / ((E - 4.275)(R + r))), tau' = (R + r) C.
    let (e, r, big_r, c) = (4.279, 5.0, 1000.0, 1e-6);
    let p = program(7, vec![battery(0, 1, 0, e, r), resistor(1, 1, 2, big_r), capacitor(2, 2, 0, c, 0.0), ls00(3, 1, 0, [1, 1, 3, 1, 1, 4, 1, 1, 5, 1, 1, 6], 10e-9, 0)]);
    let mut rig = Rig::load(&p).unwrap();
    close(rig.u(), e * big_r / (big_r + r), 1e-9, "u at load");
    close(rig.u(), 4.25771, 1e-5, "the draft's u at load");
    let th = 0.9 * 4.75;
    let powered = |rig: &Rig| {
        let ic = &rig.s().ics[0].1;
        digital::power::powered(ic, rig.v(1), rig.v(0))
    };
    assert!(!powered(&rig));
    let mut last_off = rig.u();
    let (t_on, u_on) = loop {
        let t = rig.k.next_event_time();
        assert!(t < 3e-3, "the rail never reached the power window");
        rig.advance(t).unwrap();
        if powered(&rig) {
            break (t, rig.u());
        }
        last_off = rig.u();
    };
    let eta = rails::REACHED_REL * th;
    assert!(last_off < th, "unpowered before");
    // Aimed two reached-widths past the threshold (Rails point 4: "with one
    // moving term the event lands on the threshold exactly").
    close(u_on, th + 2.0 * eta, eta, "the event lands u on the threshold");
    let tau = (big_r + r) * c;
    let t_coupled = tau * ((r * e) / ((e - th) * (big_r + r))).ln();
    let slope = (e - th) / tau;
    close(t_on, t_coupled, rails::RAIL_DRIFT_V / slope, "the power event against the coupled crossing");
    close(t_coupled, 1.68020e-3, 5e-8, "the draft's coupled crossing");
    eprintln!("power window: event at {t_on:.9e}, coupled {t_coupled:.9e}, off by {:.3e} s (bound {:.3e}), u - th {:.3e}", (t_on - t_coupled).abs(), rails::RAIL_DRIFT_V / slope, u_on - th);
    // Powered, the gate reads its inputs high and holds 1y low.
    rig.advance(t_on + 1e-6).unwrap();
    assert!(powered(&rig));
    assert!(rig.v(3) < 0.5, "1y low once powered: {}", rig.v(3));
}

/// A 74LS00's powered rail on a cell of EMF `e` and 5 ohm with 1y loaded by
/// `r_load` (every output HIGH, inputs on the datum): `u = e - 5 u / (R_out +
/// r_load)` with `R_out = u / (4 ioMax)` (W1c point 1), solved by fixed point.
fn ls00_rail_powered(e: f64, r_load: f64) -> f64 {
    let mut u = e;
    for _ in 0..200 {
        u = e - 5.0 * u / (u / (4.0 * 0.008) + r_load);
    }
    u
}

#[test]
fn an_ic_whose_own_draw_sags_the_rail_back_across_its_power_window_is_held_off() {
    // Formerly `w1f-power-gate-pingpong`'s first pin (F5, W1c point 1a):
    // E 4.279, r 5, a 74LS00 on the rail with its inputs on the datum, so
    // every output drives HIGH, and 1y into 1 kohm. Unpowered it draws
    // nothing and the rail is E, above 0.9 * 4.75 = 4.275 V; powered, 1y's
    // draw sags the rail to about 4.2602 V, below it. No power state is
    // consistent, so the rail loop's passes alternate (which declined the
    // load at its 50th pass before F5). The probes find the gate starving
    // itself: held powered its supply reads 4.2602 V, unpowered; held
    // unpowered, E. So it is held off with the rail at E, and its release
    // is the floor plus the drop its draw caused plus two reached widths,
    // 4.2938 V: past E, so it stays off for good on this cell.
    let p = program(6, vec![battery(0, 1, 0, 4.279, 5.0), ls00(1, 1, 0, [0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0, 5], 10e-9, 0), resistor(2, 2, 0, 1000.0)]);
    let mut rig = Rig::load(&p).unwrap();
    let hold = rig.s().power_hold(1);
    assert_eq!(hold.kind, digital::power::HoldKind::Off, "{hold:?}");
    let floor = 0.9 * 4.75;
    let u_on = ls00_rail_powered(4.279, 1000.0);
    close(u_on, 4.2602, 1e-4, "the probed powered rail");
    let release = hold.release.expect("a release level");
    assert!(release.rising);
    close(release.level, floor + (rig.u() - u_on) + 2.0 * rails::REACHED_REL * floor, 1e-9, "the release level");
    assert_eq!(rig.s().hold_counts(), [1.0, 0.0]);
    rig.advance(1e-3).unwrap();
    close(rig.u(), 4.279, 1e-9, "the rail at E, the gate off");
    assert!(rig.v(2).abs() < 1e-6, "1y unpowered, pulled to the datum by 1 kohm: {}", rig.v(2));
    assert_eq!(rig.s().power_hold(1).kind, digital::power::HoldKind::Off);
    assert_eq!(rig.s().hold_counts(), [1.0, 0.0], "never released: E is short of the release level");
    // The control: with 1y unloaded the gate holds its supply, and loads
    // powered with no hold.
    let p = program(6, vec![battery(0, 1, 0, 4.279, 5.0), ls00(1, 1, 0, [0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0, 5], 10e-9, 0), resistor(2, 2, 0, 1e9)]);
    let mut e = Engine::load(&p).unwrap();
    e.advance(1e-6).unwrap();
    assert!(e.voltages()[2] > 4.0, "1y HIGH: {}", e.voltages()[2]);
    assert_eq!((e.stat(25), e.stat(26)), (0.0, 0.0));
}

/// The ping-pong's CC start-up form (HG1): a {5 V, 20 mA} PSU ramps a 1 uF
/// rail capacitor with 100 ohm into 10 uF beside it, and a 74LS00's 1y
/// drives an LED and 330 ohm. Near the floor the RC takes about 18 mA, so
/// once the gate powers its LED's ~5 mA turns the rail back under the floor.
fn cc_ramp_into_an_ic_floor(c_rail: f64, i_limit: f64, load: Device) -> Program {
    program(5, vec![
        psu(0, 1, 0, 5.0, i_limit),
        ls00(1, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], 10e-9, 0),
        load,
        resistor(3, 3, 0, 330.0),
        capacitor(4, 1, 0, c_rail, 0.0),
        resistor(5, 1, 4, 100.0),
        capacitor(6, 4, 0, 1e-5, 0.0),
    ])
}

/// Runs `p` to `t_end` an event at a time, at most `cap` events: the error
/// it ends on, if any, the events taken, the time and the voltages.
fn run_events(p: &Program, t_end: f64, cap: u32) -> (Option<BoError>, u32, f64, Vec<f64>) {
    let mut e = Engine::load(p).unwrap();
    for n in 1..=cap {
        let t = e.next_event_time().min(t_end);
        if let Err(err) = e.advance(t) {
            return (Some(err), n, e.time(), e.voltages().to_vec());
        }
        if t >= t_end {
            return (None, n, t, e.voltages().to_vec());
        }
    }
    panic!("no -4 and no progress within {cap} events: stuck at t = {:.15e}, rail {:.12} V", e.time(), e.voltages()[1]);
}

/// Runs `p` event by event to `t_end` under the real kernel, at most `cap`
/// events: the error it ends on, the events taken, and IC `d`'s holds as
/// `(held, freed)` intervals (an open one ends at `t_end`).
fn hold_intervals(p: &Program, t_end: f64, cap: u32, d: usize) -> (Option<KernelError>, u32, Vec<(f64, f64)>, Rig) {
    let mut rig = Rig::load(p).unwrap();
    let (mut spans, mut since) = (Vec::new(), (rig.s().power_hold(d).kind == digital::power::HoldKind::Off).then_some(0.0));
    for n in 1..=cap {
        let t = rig.k.next_event_time().min(t_end);
        if let Err(err) = rig.advance(t) {
            return (Some(err), n, spans, rig);
        }
        let held = rig.s().power_hold(d).kind == digital::power::HoldKind::Off;
        match (since, held) {
            (None, true) => since = Some(t),
            (Some(t0), false) => {
                spans.push((t0, t));
                since = None;
            }
            _ => {}
        }
        if t >= t_end {
            spans.extend(since.map(|t0| (t0, t_end)));
            return (None, n, spans, rig);
        }
    }
    panic!("no progress within {cap} events: stuck at t = {:.15e}", rig.k.time());
}

#[test]
fn a_cc_ramp_whose_ic_load_turns_the_rail_back_under_its_floor_holds_the_gate_off_until_the_supply_carries_it() {
    // Formerly `w1f-power-gate-pingpong`'s across-instant pin (HG1): a {5 V,
    // 20 mA} PSU ramps a 1 uF rail capacitor (esr 0) with 100 ohm into 10 uF
    // beside it, and a 74LS00's 1y drives an LED and 330 ohm. The rail cannot
    // jump, so each power flip turns the segment's slope (+1,818 V/s
    // unpowered, -3,241 V/s powered: the LED's 5.06 mA against the 1.82 mA
    // the rail capacitor gets), and the power-window events crossed the
    // floor 5.3 and 9.4 fs apart, declining at 424 events before F5 (HG1's
    // count) and, with the drop rule alone, at 10,377 (the drop is 0 on such
    // a rail: ruling R44's case). Now (ruling R44 (a)) the gate, held off at
    // the floor crossing, is released only where powered at its edge it
    // would still let the segment rise: on this board when the rail leaves
    // CC, the RC's draw never falling far enough first. It powers once, on
    // a rail that carries it, and the run reaches 3 ms.
    let p = cc_ramp_into_an_ic_floor(1e-6, 0.02, led(2, 2, 3));
    let (err, n, spans, rig) = hold_intervals(&p, 3e-3, 50_000, 1);
    eprintln!("CC ramp into the floor: {err:?} after {n} events; holds {spans:?}; counts {:?}", rig.s().hold_counts());
    assert!(err.is_none(), "{err:?}");
    assert_eq!(spans.len(), 1, "one hold, no cycle: {spans:?}");
    close(spans[0].0, 1.4421592086e-3, 1e-9, "held at the floor crossing");
    assert!(spans[0].1 - spans[0].0 > 1e-5, "held until the supply carries it: {spans:?}");
    assert!(n < 2_000, "{n} events");
    assert!(rig.u() > 0.9 * 4.75 && rig.v(2) > 3.0, "powered by 3 ms: rail {} V, 1y {} V", rig.u(), rig.v(2));
    // One advance, as a frame asks for it, ends the same way.
    let mut e = Engine::load(&p).unwrap();
    e.advance(3e-3).unwrap();
    // The controls: the rail capacitor takes about i / 11 of the limit on
    // the ramp, 1.8 mA at 20 mA, which outruns 10 kohm on 1y, and at 60 mA
    // outruns the LED.
    for (i_limit, load) in [(0.02, resistor(2, 2, 3, 1e4)), (0.06, led(2, 2, 3))] {
        let (err, n, t, v) = run_events(&cc_ramp_into_an_ic_floor(1e-6, i_limit, load), 3e-3, 50_000);
        assert!(err.is_none(), "{i_limit} A: {err:?}");
        assert_eq!(t, 3e-3);
        assert!(v[1] > 0.9 * 4.75 && v[2] > 3.0, "{i_limit} A: powered by 3 ms, rail {} V, 1y {} V", v[1], v[2]);
        eprintln!("control at {i_limit} A: {n} events to 3 ms");
    }
    // A rail that a clock swings across the floor is no ping-pong, however
    // often it flips: 90 ohm from the rail to a 1 MHz clock draws (u - 2.5) /
    // 90 on average, the 20 mA limit at 4.3 V, and the 1 uF rail capacitor
    // swings about 14 mV each half period, so the rail crosses 4.275 V on
    // each one while its mean passes the floor. Each crossing comes from the
    // edge before it, not from the gate's own draw.
    let p = program(5, vec![
        psu(0, 1, 0, 5.0, 0.02),
        ls00(1, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], 10e-9, 0),
        capacitor(2, 1, 0, 1e-6, 0.0),
        resistor(3, 1, 3, 90.0),
        clock(4, 3, 0, 1e6, 0.5, 4),
    ]);
    let mut e = Engine::load(&p).unwrap();
    let floor = 0.9 * 4.75;
    let (mut flips, mut above, mut n) = (0u32, false, 0u32);
    while e.time() < 1.5e-3 {
        n += 1;
        assert!(n <= 10_000, "no progress: stuck at t = {:.15e}", e.time());
        let t = e.next_event_time().min(1.5e-3);
        e.advance(t).unwrap_or_else(|err| panic!("a clocked rail across the floor at {t:.12e}: {}", err.message));
        if (e.voltages()[1] > floor) != above {
            above = !above;
            flips += 1;
        }
    }
    assert!(flips > rails::RAIL_LOOP_MAX, "{flips} crossings of the floor");
    eprintln!("clocked rail: {flips} crossings of the floor in {n} events to 1.5 ms");
}

/// HG1's CC ramp into a 74LS00's floor, with a second 74LS00 on a fixed 5 V
/// supply whose 1a and 1b read the first gate's 1y, its own propagation delay
/// `delay` (NP1).
fn cc_floor_with_a_reader(delay: f64, i_limit: f64) -> Program {
    program(8, vec![
        psu(0, 1, 0, 5.0, i_limit),
        ls00(1, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], 10e-9, 0),
        led(2, 2, 3),
        resistor(3, 3, 0, 330.0),
        capacitor(4, 1, 0, 1e-6, 0.0),
        resistor(5, 1, 4, 100.0),
        capacitor(6, 4, 0, 1e-5, 0.0),
        source(7, 5, 0, 5.0),
        ls00(8, 5, 0, [2, 2, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0], delay, 4),
    ])
}

#[test]
fn a_zero_delay_reader_of_a_ping_ponging_gate_reads_it_held_off_then_powered() {
    // The ping-pong above, but each power flip swings 1y across a second
    // gate's threshold, and with no delay its output commits in that same
    // instant (which ends HG1's count, `end_gate_chase`): on 076be49 the
    // `Rail` events alternated 5.3 and 9.4 fs apart past any event cap, and
    // until F5 the no-progress guard declined it with -4. Now the first gate
    // is held off as above and the reader follows it: 1y low, its own 1y
    // high while the gate is held, then the reverse once it powers, by 3 ms.
    let p = cc_floor_with_a_reader(0.0, 0.02);
    let (err, n, spans, rig) = hold_intervals(&p, 3e-3, 50_000, 1);
    eprintln!("zero-delay reader: {err:?} after {n} events; holds {spans:?}");
    assert!(err.is_none(), "{err:?}");
    assert_eq!(spans.len(), 1, "{spans:?}");
    close(spans[0].0, 1.4421592086e-3, 1e-9, "held at the floor crossing");
    assert!(rig.v(2) > 3.0 && rig.v(6) < 0.5, "1y HIGH, the reader's 1y LOW: {} V, {} V", rig.v(2), rig.v(6));
    let mut e = Engine::load(&p).unwrap();
    e.advance(3e-3).unwrap();
    // The controls: with the reader's own 10 ns delay (HG1's count runs,
    // nothing commits) the gate is held the same way; at 60 mA the rail
    // outruns the LED and nothing is held.
    let (err, _, spans, _) = hold_intervals(&cc_floor_with_a_reader(10e-9, 0.02), 3e-3, 50_000, 1);
    assert!(err.is_none() && spans.len() == 1, "{err:?} {spans:?}");
    let (err, n, t, v) = run_events(&cc_floor_with_a_reader(0.0, 0.06), 3e-3, 50_000);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(t, 3e-3);
    assert!(v[2] > 3.0 && v[6] < 0.5, "1y HIGH, the reader's 1y LOW: {} V, {} V", v[2], v[6]);
    eprintln!("control at 60 mA: {n} events to 3 ms");
}

/// A 5 V generic cell of `rb` ohm and `cap` Ah draining through 100 ohm,
/// with a 1 nF capacitor charged from its rail through 50 ohm (`gate` None),
/// or through a gate's own output (`Some(high)`: 1y into the capacitor, the
/// switch on 1a/1b open, so 1y rises at load, or closed until the host opens
/// it). F1r13's draining-cell probe.
fn fast_inrush_cell(rb: f64, cap: f64, gate: Option<bool>) -> Program {
    let mut devs = vec![generic_battery(0, 1, 0, 5.0, rb, cap), resistor(1, 1, 0, 100.0)];
    match gate {
        None => {
            devs.extend([resistor(2, 1, 2, 50.0), capacitor(3, 2, 0, 1e-9, 0.0)]);
            no_cache(program(3, devs))
        }
        Some(closed) => {
            devs.extend([gate_io(2, 1, 0, [4, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0.024, 3.0, 6.0), source(3, 5, 0, 5.0), switch(4, 5, 4, closed), resistor(5, 4, 0, 1000.0), capacitor(7, 2, 0, 1e-9, 0.0)]);
            let mut reader = gate_io(8, 1, 0, [2, 2, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0.024, 3.0, 6.0);
            reader.blob[0] = 4.0;
            devs.push(reader);
            let mut p = no_cache(program(7, devs));
            p.digital_slot_count = p.digital_slot_count.max(16);
            p
        }
    }
}

#[test]
fn a_fast_rc_inrush_off_a_5_ohm_cell_runs_to_the_end_on_a_coarse_host() {
    // R6 (a): the no-progress guard never fires on a legitimate run. A 1 nF
    // capacitor charging through 50 ohm (or a gate's 50 ohm output) off a
    // 5 ohm cell sags the rail by about 4 V at once and pulls it back with a
    // 50 ns time constant: about 9e6 V/s at first, so its RAIL_DRIFT_V (10 uV)
    // events land about 1 ps apart and more closely still on the gate's
    // curved draw. Tens of thousands of them sit inside NO_PROGRESS_DT of
    // each other, every one moving the rail and the capacitor on along the
    // inrush; at e211c55 the guard declined each of F1r13's 36 such boards
    // with -4 in its first microsecond. Each runs here in 1 us frames to
    // 100 us, the rail back near the cell's EMF and the capacitor charged.
    for (name, gate) in [("50 ohm into 1 nF", None), ("1y rising at load into 1 nF", Some(false)), ("1y rising when the host opens 1a", Some(true))] {
        for (rb, cap) in [(5.0, 1e-7), (5.0, 1e-3)] {
            let name = format!("rb {rb} cap {cap:e}: {name}");
            let p = fast_inrush_cell(rb, cap, gate);
            let mut rig = Rig::load(&p).unwrap_or_else(|e| panic!("{name}: load {}", e.message));
            let mut t = 0.0;
            if gate == Some(true) {
                t = 1e-6;
                rig.advance(t).unwrap_or_else(|e| panic!("{name}: before the switch: {e:?}"));
                toggle(&mut rig, 4, false).unwrap();
            }
            let events = rig.k.stats().events_processed;
            while t < 100e-6 {
                t += 1e-6;
                rig.advance(t).unwrap_or_else(|e| panic!("{name}: {e:?} at {:.6e} (asked {t:.1e}), rail {:.9} V", rig.k.time(), rig.u()));
            }
            let vc = rig.v(2);
            assert!(rig.u() > 4.5 && (vc - rig.u()).abs() < 0.5, "{name}: rail {} V, capacitor {vc} V", rig.u());
            eprintln!("{name}: {} events to 100 us, rail {:.6} V, capacitor {vc:.6} V", rig.k.stats().events_processed - events, rig.u());
        }
    }
}

#[test]
fn a_cc_ramp_through_a_series_capacitor_into_a_short_runs_to_the_setpoint() {
    // R6 (a), G2bv's seed 400 (FXF): {9.176 V, 0.326 A} -> 2.62 uF from the
    // rail to a node held by 0.26 ohm (and 49301 ohm) to ground. In CC the
    // limit flows through the capacitor into the short: the node sits at
    // I R_p and the capacitor charges at I / C, so u = I R_p + I t / C
    // (124 kV/s) until the setpoint, at t = (V - I R_p) C / I = 73.1 us.
    // Its RAIL_DRIFT_V events land 0.08 ns apart while the short's node,
    // still, turned by rounding at most of them: the no-progress guard
    // declined it with -4 at 1.16 us.
    let (v, i, c, r_s, r_l) = (9.176, 0.326, 2.62e-6, 0.26, 49301.0);
    let r_p = 1.0 / (1.0 / r_s + 1.0 / r_l);
    let p = program(3, vec![psu(0, 1, 0, v, i), capacitor(1, 1, 2, c, 0.0), resistor(2, 2, 0, r_s), resistor(3, 2, 0, r_l)]);
    let mut rig = Rig::load(&p).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc, "CC from load");
    for t in [5e-6, 30e-6, 60e-6] {
        rig.advance(t).unwrap_or_else(|e| panic!("{e:?} at {:.9e} (asked {t:e}), rail {:.9} V", rig.k.time(), rig.u()));
        // Within W0: between drift events the RC charges at the rail of
        // the last one, through 0.26 ohm, so the charge lags (3.7e-5 V at
        // 5 us, as P7's does above).
        let want = i * r_p + i * t / c;
        close(rig.u(), want, 1e-4 + 1e-3 * want, &format!("u({t:e})"));
        close(rig.v(2), i * r_p, 1e-4 + 1e-3 * i * r_p, &format!("the short's node at {t:e}"));
    }
    let t_set = (v - i * r_p) * c / i;
    rig.advance(100e-6).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cv, "CV past the setpoint at {t_set:e}");
    assert_eq!(rig.u(), v);
}

/// The frozen tab (FXF, G2bv's seed 177 hub 2): `{9.362 V, 0.3 A}` into an
/// LED charging 29 mF beside 10 kohm. The LED and capacitor step (a stepped
/// hub partition); in CC the rail is the LED's drop above the capacitor.
///
/// The LED is rated 0.35 A, above the 0.3 A limit, with its `Is` pinned to
/// [`led`]'s 20 mA law, so its I-V curve is that LED's but W1g's `led_failed`
/// never latches: at 20 mA the 0.3 A drive (rate 15^2 - 1 = 224 against 24)
/// fails the LED at 0.107 s, the circuit opens, and the CC ramp these tests
/// follow stops there.
fn cc_led_into_a_big_capacitor() -> Program {
    let mut power_led = led(1, 1, 2);
    power_led.params[kinds::led_param::I_RATED] = 0.35;
    power_led.params[kinds::led_param::IS_SET] = 1.0;
    power_led.params[kinds::led_param::IS] = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0).is;
    program(3, vec![psu(0, 1, 0, 9.362, 0.3), power_led, capacitor(2, 2, 0, 29e-3, 0.0), resistor(3, 2, 0, 1e4)])
}

#[test]
fn a_cc_psu_charging_a_capacitor_through_an_led_steps_at_its_own_time_constants() {
    // A stepped hub partition stepped with the CC rail held over the step,
    // so the 1 uV CC re-solve band (RAIL_CC_RESOLVE_V) re-solved it each
    // time the rail rose 1 uV (10 V/s here: every 1e-7 s), each re-solve a
    // discontinuity restarting the step at POST_EVENT_H: 2.7e6 steps and
    // 27 s for 0.1 s of G2bv's board. Stepped with the rail as an unknown
    // (stepped::CcRail), the capacitor's constant-current ramp sets the
    // step. In CC the LED takes the limit less what the capacitor's 10 kohm
    // and the shunts take, so C dv/dt = I - v / R: v = I R (1 - e^(-t/RC)).
    let (i, r, c) = (0.3, 1e4, 29e-3);
    let mut e = Engine::load(&cc_led_into_a_big_capacitor()).unwrap();
    let mut ex = Engine::load(&exact(cc_led_into_a_big_capacitor())).unwrap();
    for t in [1e-3, 0.03, 0.1, 0.3] {
        e.advance(t).unwrap();
        ex.advance(t).unwrap();
        let want = i * r * (1.0 - (-t / (r * c)).exp());
        // The node shunts (1e-12 S on two nodes, the LED's GMIN_D) take
        // under 1e-11 A of the limit.
        close(e.voltages()[2], want, 1e-4 + 1e-3 * want, &format!("the capacitor at {t}"));
        for n in [1, 2] {
            let (got, x) = (e.voltages()[n], ex.voltages()[n]);
            close(got, x, 1e-4 + 1e-3 * x.abs(), &format!("node {n} at {t} against exact mode"));
        }
    }
    assert_eq!(e.records(0).unwrap(), &[] as &[f64], "the power LED stays intact, so the closed form above holds");
    let steps = e.stat(10);
    assert!(steps < 2000.0, "{steps} steps to 0.3 s");
    eprintln!("CC LED into 29 mF: {steps} steps to 0.3 s (exact mode {}), u {:.9} V, capacitor {:.9} V", ex.stat(10), e.voltages()[1], e.voltages()[2]);
}

/// G2bv's seed 396 (FXF): an 11.919 V cell of 0.872 ohm with 7.04 uF from
/// the rail into a stepped partition of LEDs, and a switch (device 12) that
/// closes onto a node one LED's drop below the rail at 0.128 s.
///
/// Each LED is a power LED with the seed's I-V law: its `Is` pinned to the
/// law its `i_rated` gives (bit for bit at 25 C), and rated 1 kA, above the
/// loop's 330 A. At the seed's ratings W1g's `led_failed` latched devices 10
/// (0.143 s) and 5 (0.448 s) and opened them, so the later checkpoints
/// compared open-LED circuits, not the loop this test is about.
fn cell_switch_into_an_led_loop() -> Program {
    let led_with = |ci: usize, a: usize, b: usize, vf: f64, n: f64, i_rated: f64| Device {
        kind: kinds::LED,
        component_index: ci,
        pins: vec![pin(a), pin(b)],
        params: vec![vf, n, 1e3, 1.0, led::JunctionLaw::at(vf, i_rated, n, None, 25.0).is, 0.0],
        blob: vec![],
    };
    program(
        4,
        vec![
            battery(0, 1, 0, 11.919, 0.871754491687756),
            resistor(1, 2, 0, 62781.73917223233),
            resistor(2, 3, 0, 2114.98690486819),
            led_with(3, 0, 1, 3.1804672682735617, 1.5309588747700547, 0.018282449928599544),
            resistor(4, 1, 0, 568.6193329458822),
            led_with(5, 1, 2, 2.5624835880969785, 2.457724114643147, 0.021406929309895958),
            switch(6, 3, 2, false),
            resistor(7, 2, 0, 93.7821532084478),
            capacitor(8, 2, 1, 7.036846324138248e-6, 0.0),
            resistor(9, 3, 0, 617.7597723368291),
            led_with(10, 1, 3, 1.6963381742955792, 2.253206847498009, 0.009201902080047068),
            resistor(11, 1, 1, 541.3846328576267),
            switch(12, 2, 3, false),
            resistor(13, 3, 0, 130.59505884211214),
            resistor(14, 1, 0, 223.89969371503818),
        ],
    )
}

#[test]
fn a_switch_closing_an_led_loop_through_a_rail_capacitor_settles_the_cell_rail() {
    // Ruling R25 (a), G2bv's seed 396. Closing device 12 puts the LED from
    // the rail to node 3 across the capacitor's held charge: a 330 A loop
    // through the LED, the switch and the capacitor back into the rail,
    // whose draw on the cell nets to a fraction of an amp. The stepped
    // partition's seed (SEED_H, the capacitor as 7e6 S) stopped at Newton's
    // tolerance there, its draw +-0.45 A off and different at each rail, so
    // the rail loop alternated the rail by 0.38 V and declined with -4 after
    // 50 passes. Polished, the seed's draw is the solution's and the loop
    // settles; afterwards the board agrees with exact mode within half of
    // W0's tolerance, and runs faster than real time.
    let mut e = Engine::load(&cell_switch_into_an_led_loop()).unwrap();
    let mut ex = Engine::load(&exact(cell_switch_into_an_led_loop())).unwrap();
    let t_close = 0.12846868322174154;
    let wall = std::time::Instant::now();
    for eng in [&mut e, &mut ex] {
        eng.advance(t_close).unwrap();
        eng.set_params(&[(12, 0, 1.0)]).unwrap_or_else(|err| panic!("closing the switch: {} {}", err.code, err.message));
    }
    for t in [t_close + 1e-4, 0.13, 0.2, 0.5, 1.0] {
        e.advance(t).unwrap_or_else(|err| panic!("default at {t}: {} {}", err.code, err.message));
        ex.advance(t).unwrap();
        for n in 1..4 {
            let (got, x) = (e.voltages()[n], ex.voltages()[n]);
            close(got, x, 0.5 * (1e-4 + 1e-3 * x.abs()), &format!("node {n} at {t} against exact mode"));
        }
    }
    assert_eq!(e.records(0).unwrap(), &[] as &[f64], "the power LEDs stay intact, so the loop above is the one compared");
    assert_eq!(ex.records(0).unwrap(), &[] as &[f64], "and in exact mode");
    let secs = wall.elapsed().as_secs_f64();
    assert!(secs < 1.0, "{secs} s of wall time for 1 s simulated (both engines)");
    eprintln!("seed 396: {} steps to 1 s, {secs:.3} s wall with exact mode", e.stat(10));
}

#[test]
fn a_cc_exit_inside_a_step_lands_within_the_crossing_tolerance() {
    // The CC step's exit cut (Rails point 4, the CC step): the step whose
    // CC rail passes the setpoint is cut where the rail reaches it, to
    // CROSS_TOLERANCE_V. At the setpoint the draw is the limit, so the
    // capacitor stands where the LED's drop at the limit puts the rail on
    // V: I_LED(V - v) + GMIN_D (V - v) + NODE_RSHUNT_G V = I (the LED's own
    // law, by bisection), and the exit publishes it within the tolerance of
    // the rail cut (the LED's current is the limit's on both sides, so the
    // capacitor moves with the rail one for one). Steps reach H_MAX on this
    // ramp (10 V/s): a cut short of the tolerance lands mV off, and the
    // whole step re-solved in CV landed 16 mV off.
    let (v, i) = (9.362, 0.3);
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let f = |vc: f64| law.current(v - vc) + led::GMIN_D * (v - vc) + static_part::NODE_RSHUNT_G * v - i;
    let (mut lo, mut hi) = (0.0, v);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let want = 0.5 * (lo + hi);
    let mut rig = Rig::load(&cc_led_into_a_big_capacitor()).unwrap();
    let t_exit = cc_exit(&mut rig, 2.0);
    // The cut's rail and the rail loop's can part by the step solve's own
    // residual (1e-8 V), and a cut a hair short exits at the next step,
    // H_MIN later: at most one H_MIN of the 10 V/s ramp more.
    let ramp = i / 29e-3;
    close(rig.v(2), want, 2.0 * stepped::CROSS_TOLERANCE_V + ramp * stepped::H_MIN, &format!("the capacitor at the CC exit ({t_exit:.9} s)"));
}

/// FXEFv's frozen tab (FXF round 3, its `cc 50 tpl=9`): two stepped LED and
/// capacitor branches on one CC PSU (no rail capacitor), the second
/// switched on at 36.7 ms.
fn cc_two_stepped_branches() -> Program {
    let led_with = |ci: usize, a: usize, b: usize, vf: f64, n: f64, i_rated: f64| Device {
        kind: kinds::LED,
        component_index: ci,
        pins: vec![pin(a), pin(b)],
        params: vec![vf, n, i_rated, 0.0, 0.0, 0.0],
        blob: vec![],
    };
    program(
        7,
        vec![
            psu(0, 1, 0, 7.074682306731746, 0.04969714478669725),
            resistor(1, 1, 2, 80.8610427002485),
            led_with(2, 2, 3, 1.8308812442799438, 2.127961042214763, 0.009595530326038746),
            capacitor(3, 3, 0, 0.0024925171639779437, 0.0),
            resistor(4, 3, 0, 26681.76879544293),
            switch(5, 1, 4, false),
            resistor(6, 4, 5, 157.01184906156382),
            led_with(7, 5, 6, 2.5360816505906425, 2.377752197542115, 0.01811977062511097),
            capacitor(8, 6, 0, 0.0001874256015905851, 0.0),
            resistor(9, 6, 0, 20602.80889538028),
        ],
    )
}

#[test]
fn two_stepped_partitions_on_one_cc_rail_keep_their_steps() {
    // Each stepped partition's CC step holds the other's draw where it last
    // committed, so each commit moves the rail past the other's 1 uV band
    // and the rail loop re-solves it. That re-solve restarted its step at
    // POST_EVENT_H as a discontinuity would: 2.77e6 steps and 18.8 s for
    // this 0.1 s. A re-solve for the rail's band alone keeps the step the
    // controller chose (stepped::SteppedSolver::reaim); the coupling step
    // control makes 758 steps of it.
    let mut e = Engine::load(&cc_two_stepped_branches()).unwrap();
    let mut ex = Engine::load(&exact(cc_two_stepped_branches())).unwrap();
    for eng in [&mut e, &mut ex] {
        eng.advance(0.036663502769636445).unwrap();
        eng.set_params(&[(5, 0, 1.0)]).unwrap();
    }
    for t in [0.04, 0.06, 0.1] {
        e.advance(t).unwrap();
        ex.advance(t).unwrap();
        // Exact mode steps the two branches as one partition, and is itself
        // 3.4x W0 off a fine reference on this board (FXEFv's `cc`); each
        // CC step here holds the other branch's draw where it last
        // committed, a first-order coupling error, now kept within the
        // rail's W0 per step (`stepped::coupling_ratio`, ruling R39 (1)):
        // 3.44x W0 from exact mode at worst (the second capacitor at 60 ms;
        // 5.1x before). A known divergence at its measured worst (README,
        // Known follow-ups (W1g), R32); the backward-Euler controller's own
        // error is R27/R31's.
        for n in 1..7 {
            let (got, x) = (e.voltages()[n], ex.voltages()[n]);
            close(got, x, 3.5 * (1e-4 + 1e-3 * x.abs()), &format!("node {n} at {t} against exact mode"));
        }
    }
    let steps = e.stat(10);
    assert!(steps < 5000.0, "{steps} steps to 0.1 s (exact mode {})", ex.stat(10));
}

/// FXEFv's `cc` boards (`CC_PWR=1 cc 21 22 tpl=10` and `tpl=12`, seed 21):
/// a {8.78 V, 21.6 mA} PSU (no rail capacitor) into two or three LED and
/// capacitor branches, every one stepped, all on from load. The LEDs are
/// the generator's power LEDs (rated 3 A, so W1g's overdrive latch leaves
/// them alone).
fn cc_seed21_branches(three: bool) -> Program {
    let led_with = |ci: usize, a: usize, b: usize, vf: f64, n: f64| Device {
        kind: kinds::LED,
        component_index: ci,
        pins: vec![pin(a), pin(b)],
        params: vec![vf, n, 3.0, 0.0, 0.0, 0.0],
        blob: vec![],
    };
    let mut devices = vec![
        psu(0, 1, 0, 8.78053248388062, 0.02156505523783257),
        resistor(1, 1, 2, 155.75945726224964),
        capacitor(3, 3, 0, 9.95847677612162e-5, 0.0),
        resistor(4, 3, 0, 23942.725056979652),
        resistor(5, 1, 5, 10.553772652131514),
        capacitor(7, 6, 0, 0.00020878353973358566, 0.0),
    ];
    if three {
        devices.extend([
            led_with(2, 2, 3, 1.7162410990045835, 1.500439376584047),
            led_with(6, 5, 6, 2.2665203674115038, 2.1191992875223002),
            resistor(8, 6, 0, 340.19526150114996),
            resistor(9, 1, 8, 28.94286290705404),
            led_with(10, 8, 9, 2.902616250203949, 1.684399560150239),
            capacitor(11, 9, 0, 1.559478163216429e-5, 0.0),
            resistor(12, 9, 0, 6402.3979397654375),
        ]);
    } else {
        devices.extend([
            led_with(2, 2, 3, 1.9567797208840794, 1.8287000353661065),
            led_with(6, 5, 6, 1.600703002534475, 2.2410807957596885),
            resistor(8, 6, 0, 10946.769571022789),
        ]);
    }
    devices.sort_by_key(|d| d.component_index);
    program(if three { 10 } else { 7 }, devices)
}

/// The worst node of `got` against `want`, sample by sample, in W0 units.
fn worst_w0(got: &[Vec<f64>], want: &[Vec<f64>]) -> f64 {
    got.iter().zip(want).flat_map(|(g, w)| g.iter().zip(w).map(|(x, y)| (x - y).abs() / (1e-4 + 1e-3 * y.abs()))).fold(0.0, f64::max)
}

/// Every node voltage of `p` at each of `samples`, the engine advanced in
/// host slices of at most `slice` (0: straight to each sample), with the
/// wall time and the steps taken.
fn sampled(p: &Program, samples: &[f64], slice: f64) -> (Vec<Vec<f64>>, f64, f64) {
    let mut e = Engine::load(p).unwrap();
    let wall = std::time::Instant::now();
    let mut out = Vec::new();
    let mut t = 0.0;
    for &s in samples {
        while t < s {
            t = if slice > 0.0 { (t + slice).min(s) } else { s };
            e.advance(t).unwrap_or_else(|err| panic!("{} at {t}: {}", err.code, err.message));
        }
        out.push(e.voltages().to_vec());
    }
    (out, wall.elapsed().as_secs_f64(), e.stat(10))
}

#[test]
fn stepped_partitions_sharing_a_cc_rail_bound_the_rail_move_across_each_step() {
    // Ruling R39 (1), R37 (a). With two or three stepped branches on one CC
    // rail each CC step holds the others' draws where they last committed,
    // and since FXF round 3 each keeps the step its controller chose: the
    // rail moved across a step by far more than W0, an error the step
    // doubling cannot see, 39x and 43x W0 off a fine reference on these
    // boards (W1gV3). Each step now keeps the rail's move within W0
    // (`stepped::coupling_ratio`), so the coupling adds at most a little to
    // the backward-Euler controller's own error: exact mode, which steps
    // the branches and the rail as one partition with the same controller
    // (no coupling), is that floor here. The fine reference is exact mode
    // under the trapezoid in 2 us host slices (scipy's Radau agrees with it
    // to 0.00 W0 on W1gV3's board).
    let samples: Vec<f64> = (1..=100).map(|k| k as f64 * 1e-3).collect();
    for three in [false, true] {
        let name = if three { "three branches" } else { "two branches" };
        let mut fine = exact(cc_seed21_branches(three));
        fine.options.integration = 1;
        let (want, _, _) = sampled(&fine, &samples, 2e-6);
        let (floor, _, _) = sampled(&exact(cc_seed21_branches(three)), &samples, 0.0);
        let (got, secs, steps) = sampled(&cc_seed21_branches(three), &samples, 0.0);
        let (err, floor_err) = (worst_w0(&got, &want), worst_w0(&floor, &want));
        eprintln!("{name}: {err:.3} W0 off the fine reference (exact mode {floor_err:.3}), {steps} steps, {secs:.4} s wall");
        assert!(err <= floor_err + 1.0, "{name}: {err} W0 off the fine reference, the controller's floor {floor_err} W0");
        // FXF round 3's freeze (each re-solve restarting at POST_EVENT_H)
        // took 1.1e6 steps for this 0.1 s; 3,682 and 3,024 steps run in
        // 0.08 s alone in a release build, faster than real time. The time
        // itself is not asserted: on the suite's parallel threads this Mac
        // took 0.15-0.17 s of wall and of thread CPU time for the same steps
        // (its efficiency cores), so the bound is on the steps, with 35 %
        // headroom, and the CC sweeps (`cc`, FXEFv) measure the wall time.
        assert!(steps < 5_000.0, "{name}: {steps} steps to 0.1 s");
    }
}

/// A power LED (`led`'s law, rated 0.35 A so W1g's overdrive latch leaves
/// it alone at a CC limit of tens of mA).
fn power_led(ci: usize, anode: usize, cathode: usize) -> Device {
    let mut d = led(ci, anode, cathode);
    d.params[kinds::led_param::I_RATED] = 0.35;
    d.params[kinds::led_param::IS_SET] = 1.0;
    d.params[kinds::led_param::IS] = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0).is;
    d
}

/// Two stepped LED and capacitor branches on one CC PSU (`{7 V, 30 mA}`, no
/// rail capacitor): branch A from the rail, branch B from a 74LS00's `1y`
/// (inputs grounded, so high while powered) whose supply is the rail. In CC
/// the rail rides on A's capacitor, from 4.5 V at load (inside the gate's
/// 4.275-5.775 V window, and 5 V with the gate off: no ping-pong at load)
/// up through 5.775 V, where the gate powers off and B goes dark (the rail
/// jumps 0.5 V up as A takes B's 5 mA, so it stays off), and on to the 7 V
/// setpoint, where the PSU leaves CC.
fn cc_two_branches_with_a_power_flip_and_an_exit() -> Program {
    program(
        7,
        vec![
            psu(0, 1, 0, 7.0, 0.03),
            resistor(1, 1, 2, 100.0),
            power_led(2, 2, 3),
            capacitor(3, 3, 0, 1e-3, 0.0),
            resistor(4, 3, 0, 1000.0),
            ls00(5, 1, 0, [0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0], 10e-9, 0),
            resistor(6, 4, 5, 330.0),
            power_led(7, 5, 6),
            capacitor(8, 6, 0, 1e-4, 0.0),
            resistor(9, 6, 0, 1000.0),
        ],
    )
}

#[test]
fn a_kept_cc_step_restarts_at_a_power_flip_and_at_the_cc_exit() {
    // FXF round 3 keeps a stepped partition's step when the rail loop
    // re-solves it only because the CC rail moved past its band
    // (`SteppedSolver::reaim`). A rail IC's power flip and the CC exit are
    // discontinuities of the partition's own stamp, so those re-solves
    // restart at POST_EVENT_H (W1gV3's seeds: keeping the step across
    // either passed every suite).
    let p = cc_two_branches_with_a_power_flip_and_an_exit();
    let mut rig = Rig::load(&p).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc, "CC from load");
    let (a, b) = (rig.s().partition_of_node(3).unwrap().0 as usize, rig.s().partition_of_node(6).unwrap().0 as usize);
    assert_ne!(a, b, "two partitions");
    let wall = std::time::Instant::now();
    let mut t = 0.0;
    while t < 0.2 {
        t += 1e-3;
        rig.advance(t).unwrap_or_else(|e| panic!("{e:?} at {:.9e} (asked {t:e}), rail {:.9} V", rig.k.time(), rig.u()));
    }
    let secs = wall.elapsed().as_secs_f64();
    assert_eq!(rig.hub().state.reg, Regime::Cv, "CV at 0.2 s, rail {} V", rig.u());
    let log = &rig.s().stepped.resolve_log;
    // Both branches are re-aimed in CC: each commit of one moves the rail
    // past the other's 1 uV band.
    for (name, q) in [("A", a), ("B", b)] {
        let kept = log.iter().filter(|r| r.partition == q && r.cc && r.kept).count();
        assert!(kept > 10, "{name}: {kept} re-aims in CC");
    }
    // The flip: the first re-solve of B after load with the rail past the
    // gate's 5.775 V edge is the pass that found it, and B restarts there.
    // (The load's re-solves read the boundary before the first rail solve,
    // the setpoint.)
    let edge = 5.25 * 1.1;
    let flip = log.iter().find(|r| r.partition == b && r.now > 0.0 && r.u > edge).expect("the rail passes the gate's edge in CC");
    assert!(flip.cc, "the flip comes while the PSU is in CC");
    assert!(log.iter().any(|r| r.partition == b && r.now == flip.now && !r.kept), "B keeps its step across its gate's power flip at {:.9} s", flip.now);
    // The exit: the first re-solve against no CC rail after the flip, where
    // every stepped hub partition re-solves at the setpoint, and restarts.
    // (The rail loop's `!cc_exit` guard is implied today: the band rule is
    // read after the regime change, and CV's is the curvature rule, which
    // re-aims nothing. This pins the restart against either changing.)
    let exit = log.iter().find(|r| r.now > flip.now && !r.cc).expect("a CC exit after the flip");
    for (name, q) in [("A", a), ("B", b)] {
        assert!(log.iter().any(|r| r.partition == q && r.now == exit.now && !r.cc && !r.kept), "{name} keeps its step across the CC exit at {:.9} s", exit.now);
    }
    eprintln!("flip at {:.6} s, CC exit at {:.6} s, {} steps, {secs:.3} s wall", flip.now, exit.now, rig.s().stepped.steps_taken());
}

#[test]
fn a_cc_step_whose_load_is_switched_out_leaves_cc_at_the_step_start() {
    // FXF's own regression (G2bv's seed 99 hub 2): {9.582 V, 2 A} held in
    // CC by a short across the rail, with 21.8 mF and 1 mF from the rail
    // into a stepped partition. Opening the short drops the draws far below
    // the limit: the CC step's rail is past the setpoint however short the
    // step, so the exit is at the step's start. Cutting the step toward a
    // crossing that was not there made steps of 1e-12 s and hung the host.
    let devices = || {
        vec![
            psu(0, 1, 0, 9.582, 1.999),
            resistor(1, 2, 0, 4330.0),
            resistor(2, 3, 0, 3136.0),
            switch(3, 0, 1, true),
            capacitor(4, 1, 2, 21.8e-3, 0.0),
            capacitor(5, 3, 1, 1e-3, 0.0),
            resistor(6, 2, 3, 0.39),
        ]
    };
    let mut e = Engine::load(&program(4, devices())).unwrap();
    let mut ex = Engine::load(&exact(program(4, devices()))).unwrap();
    for eng in [&mut e, &mut ex] {
        eng.advance(0.1).unwrap();
        eng.set_params(&[(3, 0, 0.0)]).unwrap();
    }
    let steps = e.stat(10);
    for t in [0.1 + 1e-4, 0.11, 0.2] {
        e.advance(t).unwrap();
        ex.advance(t).unwrap();
        for n in 1..4 {
            let (got, x) = (e.voltages()[n], ex.voltages()[n]);
            close(got, x, 1e-4 + 1e-3 * x.abs(), &format!("node {n} at {t} against exact mode"));
        }
    }
    let steps = e.stat(10) - steps;
    assert!(steps < 2000.0, "{steps} steps over the 0.1 s after the short opened");
    assert_eq!(e.voltages()[1], 9.582, "CV at the setpoint");
}

#[test]
fn a_gigahertz_clock_into_an_rc_and_gates_runs_for_any_number_of_edges() {
    // R6 (a): a clock has no frequency cap, so one at 1 GHz or 10 GHz is a
    // legitimate board. Its half periods (0.5 and 0.05 ns) fall inside
    // NO_PROGRESS_DT, and every edge turns its node back, so without a rule
    // for them the guard counted each edge and declined after 10,000 of them
    // (5 us at 1 GHz). A source's edges are scheduled input, not the circuit
    // chasing itself. Here it drives 1 kohm into 1 nF and 74LS00 gates read
    // the clock and the capacitor, for 40,000 and 400,000 edges.
    for freq in [1e9, 1e10] {
        let p = program(6, vec![
            clock(0, 1, 0, freq, 0.5, 0),
            resistor(1, 1, 2, 1000.0),
            capacitor(2, 2, 0, 1e-9, 0.0),
            source(3, 3, 0, 5.0),
            ls00(4, 3, 0, [1, 1, 4, 2, 2, 5, 0, 0, 0, 0, 0, 0], 10e-9, 1),
        ]);
        let mut e = Engine::load(&p).unwrap();
        let t_end = 20e-6;
        let mut t = 0.0;
        while t < t_end {
            t += 1e-6;
            e.advance(t).unwrap_or_else(|err| panic!("{freq:e} Hz at {:.9e} (asked {t:.1e}): {}", e.time(), err.message));
        }
        let edges = e.stat(19);
        assert!(edges >= 2.0 * freq * t_end - 2.0, "{freq:e} Hz: {edges} edges");
        assert!((e.voltages()[2] - 2.5).abs() < 0.1, "{freq:e} Hz: the capacitor at the clock's mean, {} V", e.voltages()[2]);
        eprintln!("{freq:e} Hz: {edges} edges to {t_end} s, capacitor {:.6} V, 2y {:.3} V", e.voltages()[2], e.voltages()[5]);
    }
}

/// V11's board B1070: a {5 V, 0.454 mA} PSU from 3.5 V on 47 uF (esr 0); a
/// gate (io 16 mA) whose 1y rises into 91 nF once the host opens 1a/1b at
/// `t_sw`, a reader of 1y, and a second gate driving 1.17 uF through 1 kohm.
/// Returns the program and `t_sw`.
fn cc_exit_stall_board() -> (Program, f64) {
    let (c_a, io) = (9.093449790570233e-8, 0.016);
    let mut reader = gate_io(7, 1, 0, [2, 2, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 3.0, 6.0);
    reader.blob[0] = 4.0;
    let mut b = gate_io(8, 1, 0, [0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 3.0, 6.0);
    b.blob[0] = 8.0;
    let devs = vec![
        psu(0, 1, 0, 5.0, 4.543528850749108e-4),
        capacitor(1, 1, 0, 4.6999878867252346e-5, 0.0),
        gate_io(2, 1, 0, [4, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 3.0, 6.0),
        source(3, 5, 0, 5.0),
        switch(4, 5, 4, true),
        resistor(5, 4, 0, 1000.0),
        capacitor(6, 2, 0, c_a, 0.0),
        reader,
        b,
        resistor(9, 7, 8, 1000.0),
        capacitor(10, 8, 0, 1.173756250879161e-6, 0.0),
    ];
    let mut p = no_cache(program(9, devs));
    p.digital_slot_count = p.digital_slot_count.max(16);
    (p, (20.0 * c_a / gate_g(3.5, io)).min(200e-6))
}

/// [`cc_exit_stall_board`] loaded at 3.5 V with 1y released, up to just past
/// `t_sw`.
fn cc_exit_stall_rig() -> (Rig, f64) {
    let (p, t_sw) = cc_exit_stall_board();
    let mut rig = Rig::load_carrying(&p, &[(1, 3.5)]).unwrap();
    rig.advance(t_sw).unwrap();
    toggle(&mut rig, 4, false).unwrap();
    rig.advance(t_sw + 2e-8).unwrap();
    (rig, t_sw + 2e-8)
}

#[test]
fn a_cc_exit_that_pinned_the_rail_now_runs_on_a_coarse_host() {
    // V11's B1070 (R6 (a), R15 (a)): on a host advancing 612 us a frame, the
    // frame over the CC exit at about 169 ms never returned under round 12's
    // drift cap: the rail sat at 4.99999999999925837 V while Rail events came
    // 1.14 ps apart. Round 15's coupled CC path leaves no such stall, so the
    // board must now run to the end on the coarse host; the no-progress guard's
    // own decline of that shape is pinned by the kernel's mock tests. On a
    // thread given a minute, so a regression fails the test rather than
    // hanging it.
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let (mut rig, mut t) = cc_exit_stall_rig();
        let result = loop {
            t += 6.119836310668826e-4;
            if let Err(e) = rig.advance(t) {
                break Err((e, rig.k.time(), rig.u()));
            }
            if t > 0.25 {
                break Ok(rig.u());
            }
        };
        let _ = tx.send(result);
    });
    match rx.recv_timeout(std::time::Duration::from_secs(60)) {
        Ok(Ok(u)) => assert!((u - 5.0).abs() < 1e-6, "ran to 250 ms off the setpoint: rail {u:.17} V"),
        Ok(Err((e, at, u))) => panic!("declined at {at:.12e}, rail {u:.17} V: {e:?}"),
        Err(_) => panic!("the frame over the CC exit did not return within a minute: hung"),
    }
    // Stepped event by event, the same board passes the exit without
    // stalling (about 64,500 events to 250 ms), so the guard must leave it
    // running; under a cap that turns a hang into a failure.
    let (mut rig, t0) = cc_exit_stall_rig();
    let t_end = 0.25;
    let mut n = 0u32;
    while rig.k.time() < t_end {
        n += 1;
        assert!(n <= 2_000_000, "no end within {n} events: at {:.15e}, rail {:.17} V", rig.k.time(), rig.u());
        let t = rig.k.next_event_time().min(t_end).max(t0);
        rig.advance(t).unwrap_or_else(|e| panic!("event by event at {t:.12e}: {e:?}"));
    }
    eprintln!("B1070 event by event: {n} events to {t_end} s, rail {:.9} V", rig.u());
}

// ---- Rails point 5: bench PSU regimes ------------------------------------------

#[test]
fn cc_charging_through_a_resistor_follows_probe_p7_and_leaves_cc_at_the_setpoint() {
    // {5 V, 0.1 A} -> 10 ohm -> 100 uF (probe P7). CC from load: vc = 1000 t,
    // u = vc + 1, -0.1 A; the draw at 5 V is back at the limit at vc = 4.0
    // (t = 4 ms, fixed classic's exit); then vc = 5 - e^(-(t - 4 ms)/1 ms).
    let p = program(3, vec![psu(0, 1, 0, 5.0, 0.1), resistor(1, 1, 2, 10.0), capacitor(2, 2, 0, 1e-4, 0.0)]);
    let mut rig = Rig::load(&p).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc, "CC from load");
    // The draw at vc = 0 is u / R plus the rail node's shunt.
    close(rig.u(), 0.1 / (0.1 + static_part::NODE_RSHUNT_G), 1e-12, "u at load");
    rig.advance(1e-3).unwrap();
    // Between rail events the partition charges at the rail of the last
    // event, so the charge lags by half a drift width per interval: a
    // relative deficit of RAIL_DRIFT_V / (2 R iLimit) = 5e-6.
    let lag = rails::RAIL_DRIFT_V / (2.0 * 10.0 * 0.1);
    let (vc1, u1, i1) = (rig.v(2), rig.v(1), rig.i(0));
    close(vc1, 1.0, 1.0 * lag + 1e-9, "vc(1 ms)");
    close(u1, 2.0, 1.0 * lag + rails::RAIL_DRIFT_V, "u(1 ms)");
    assert_eq!(i1, -0.1, "CC publishes -iLimit exactly");
    let mut exit = None;
    while exit.is_none() {
        let t = rig.k.next_event_time();
        assert!(t < 4.1e-3, "CC never ended");
        rig.advance(t).unwrap();
        if rig.hub().state.reg == Regime::Cv {
            exit = Some((t, rig.v(2)));
        }
    }
    let (t_exit, vc_exit) = exit.unwrap();
    close(vc_exit, 4.0, 1e-9, "the exit at vc = 4.0, where the draw at 5 V is 0.1 A");
    close(t_exit, 4e-3, 4e-3 * lag + 1e-12, "the exit time");
    rig.advance(4.05e-3).unwrap();
    let want = 5.0 - (-0.05f64).exp();
    let (vc2, i2) = (rig.v(2), rig.i(0));
    // The exit's lag carries over as a time shift of the CV exponential.
    let shift = (t_exit - 4e-3) * (-0.05f64).exp() * 1000.0;
    close(vc2, want, shift.abs() + 1e-9, "vc(4.05 ms)");
    close(i2, -(5.0 - want) / 10.0, shift.abs() / 10.0 + 1e-9, "I(4.05 ms)");
    eprintln!(
        "P7: vc(1 ms) {vc1:.9} (off {:.3e}), u(1 ms) {u1:.9}, I {i1}; exit at {t_exit:.9e} (vc {vc_exit:.12}); vc(4.05 ms) {vc2:.9} (want {want:.9}, off {:.3e}), I {i2:.9} (want {:.9}); rail events {}",
        (vc1 - 1.0).abs(),
        (vc2 - want).abs(),
        -(5.0 - want) / 10.0,
        rig.hub().stats.events
    );
}

/// The root of `I_LED(u) + GMIN_D u + NODE_RSHUNT_G u = i` by bisection on
/// the LED's own law (the model's equation for a direct LED on a CC rail).
fn led_rail_at(i: f64) -> f64 {
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let f = |u: f64| law.current(u) + led::GMIN_D * u + static_part::NODE_RSHUNT_G * u - i;
    let (mut lo, mut hi) = (0.0, 5.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    0.5 * (lo + hi)
}

#[test]
fn cc_with_an_led_straight_across_solves_the_led_law_at_the_limit() {
    // {5 V, 0.1 A} with an LED from the rail to ground and no resistor: the
    // LED is a direct device, so the CC rail solves I_LED(u) = iLimit in the
    // rail's own Newton (Rails points 2 and 5); exact mode solves the same.
    let devices = || vec![psu(0, 1, 0, 5.0, 0.1), led(1, 1, 0)];
    let rig = Rig::load(&program(2, devices())).unwrap();
    let want = led_rail_at(0.1);
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    close(rig.u(), want, rails::RAIL_CC_RESOLVE_V, "u against the LED law");
    close(rig.u(), want, 1e-11, "u to the rail Newton's tolerance");
    assert_eq!(rig.i(0), -0.1);
    let ex = Rig::load(&exact(program(2, devices()))).unwrap();
    assert_eq!(rig.state(0), vec![2.0], "reg 2");
    assert_eq!(ex.state(0), vec![2.0], "exact mode: reg 2");
    close(ex.v(1), want, newton::NEWTON_ABS, "exact mode's rail against the LED law, to its Newton's tolerance");
    eprintln!("CC LED: u {:.15}, exact {:.15}, law {want:.15}", rig.u(), ex.v(1));
}

/// Steps `rig` event by event until its PSU leaves CC, before `t_max`, and
/// returns the instant.
fn cc_exit(rig: &mut Rig, t_max: f64) -> f64 {
    loop {
        let t = rig.k.next_event_time();
        assert!(t < t_max, "CC never ended before {t_max:e}");
        rig.advance(t).unwrap();
        if rig.hub().state.reg == Regime::Cv {
            return t;
        }
    }
}

#[test]
fn a_discharged_rail_capacitor_puts_the_psu_in_cc_at_load_and_the_rail_ramps_at_the_limit() {
    // Probe P9's circuit: {5 V, 0.1 A} with 100 uF straight across (Rails
    // point 5, "A rail capacitor below the setpoint is demand"; contract
    // test (a)). The capacitor starts at 0 V (W1b point 5), 5 V short of the
    // setpoint: its draw there is unbounded at esr 0 and 100 A at esr 0.05,
    // so the PSU is in CC from load and the rail is the capacitor's closed
    // form (Rails point 4): its charge q rises from 0 V, C dq/dt = iLimit - G
    // u with G the rail node's shunt, 1000 V/s, and the rail sits esr I_cap
    // above it (manager ruling 2026-10-07, F1 finding C: classic reads 0.005
    // V at load and 2.005 V at 2 ms at esr 0.05). It leaves CC where the rail
    // reaches 5 V, at 5 ms less esr C (exactly to first order in G; probe P9:
    // classic stays in CC, -0.1 A, up to 5.0 V, and fixed classic leaves CC
    // between 5.0000 and 5.0001 ms at esr 0, within 1e-9 V of this rail at
    // every instant the equivalence harness compares). A negative setpoint
    // ramps the other way.
    for (v, esr) in [(5.0, 0.0), (5.0, 0.05), (-5.0, 0.0), (-5.0, 0.05)] {
        let what = format!("V {v}, esr {esr}");
        let dir: f64 = if v < 0.0 { -1.0 } else { 1.0 };
        let (c, i_limit) = (1e-4, 0.1);
        let g = static_part::NODE_RSHUNT_G;
        let dl = dir * i_limit;
        // u = alpha q + alpha esr dl, q from 0 at the rate k = alpha G / C.
        let alpha = 1.0 / (1.0 + g * esr);
        let k = alpha * g / c;
        let ode = |t: f64| alpha * (dl / g) * -(-k * t).exp_m1() + alpha * esr * dl;
        let p = program(2, vec![psu(0, 1, 0, v, i_limit), capacitor(1, 1, 0, c, esr)]);
        let mut rig = Rig::load(&p).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cc, "{what}: CC from load");
        assert_eq!(rig.state(0), vec![2.0], "{what}: reg 2");
        close(rig.u(), ode(0.0), 1e-15, &format!("{what}: the rail starts esr iLimit above the capacitor's charge"));
        assert_eq!(rig.i(0), -dl, "{what}: CC publishes -dir iLimit exactly");
        close(rig.i(1), dl - g * rig.v(1), 1e-15, &format!("{what}: the capacitor takes the limit"));
        for t in [1e-3, 2e-3, 4e-3] {
            rig.advance(t).unwrap();
            assert_eq!(rig.hub().state.reg, Regime::Cc, "{what}: CC at {t:e}");
            close(rig.v(1), ode(t), 1e-12, &format!("{what}: u at {t:e}"));
            close(rig.v(1), dir * (1000.0 * t + esr * i_limit), 1e-9, &format!("{what}: 1000 V/s at {t:e}, over the esr drop"));
            assert_eq!(rig.i(0), -dl, "{what}: the limit at {t:e}");
            close(rig.i(1), dl - g * rig.v(1), 1e-15, &format!("{what}: C dq/dt at {t:e}"));
        }
        let t_exit = cc_exit(&mut rig, 6e-3);
        // The rail reaches the setpoint where the ODE does, the event aimed
        // two reached-widths past it (Rails point 4).
        let t_ode = -(-(v / alpha - esr * dl) * g / dl).ln_1p() / k;
        close(t_exit, t_ode, 3.0 * rails::REACHED_REL * 5.0 * c / i_limit, &format!("{what}: the exit"));
        close(t_exit, (5.0 - esr * i_limit) * c / i_limit, 1e-12, &format!("{what}: 5 ms less esr C"));
        assert_eq!(rig.v(1), v, "{what}: CV at the setpoint");
        if esr > 0.0 {
            // Its charge one esr drop short of the rail, the capacitor carries
            // its CC current on through the exit and relaxes with esr C (the
            // drop is recovered from the charge, to an ulp of 5 V over esr).
            close(rig.i(1), dl - g * v, 1e-13, &format!("{what}: continuous through the exit"));
        }
        rig.advance(10e-3).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cv, "{what}: and it stays CV");
        assert_eq!(rig.v(1), v);
        assert_eq!(rig.i(1), 0.0, "{what}: the capacitor charged at the rail carries nothing");
        close(rig.i(0), -v * g, 1e-24, &format!("{what}: the PSU supplies the shunt"));
        eprintln!("P9 {what}: exit at {t_exit:.15e} (ODE {t_ode:.15e}), rail events {}, moves {}", rig.hub().stats.events, rig.hub().stats.moves);
    }
}

#[test]
fn a_rail_capacitor_to_a_waveform_source_node_is_declined() {
    // Rails point 7 (README 7c94056): an edge on a rail capacitor's far node
    // does not solve the rail, and the CC segment ignores that node moving,
    // so the load is -3 until it is handled: on a PSU or a battery, the
    // capacitor either way round, on the clock's node or on a source stacked
    // on it, in exact mode alike. Across the clock alone, or from the rail to
    // ground, a capacitor loads (rule 2a; a rail capacitor).
    let clocked = |hub: Device, cap: Device| vec![hub, clock(1, 2, 0, 1000.0, 0.5, 0), cap, resistor(3, 1, 0, 1000.0)];
    for hub in [psu(0, 1, 0, 5.0, 0.1), battery(0, 1, 0, 5.0, 0.01)] {
        for cap in [capacitor(2, 1, 2, 1e-4, 0.0), capacitor(2, 2, 1, 1e-4, 0.05)] {
            let m = declines(clocked(hub.clone(), cap.clone()), 3);
            assert!(m.contains("capacitor (component 2) from the hub rail to a waveform source's node"), "{m}");
        }
    }
    let stacked = vec![psu(0, 1, 0, 5.0, 0.1), clock(1, 2, 0, 1000.0, 0.5, 0), source(2, 3, 2, 1.0), capacitor(3, 1, 3, 1e-4, 0.0), resistor(4, 1, 0, 1000.0)];
    let m = declines(stacked, 4);
    assert!(m.contains("waveform source's node"), "a source stacked on the clock: {m}");
    let ex = PartitionedSolver::build(&exact(program(3, clocked(psu(0, 1, 0, 5.0, 0.1), capacitor(2, 1, 2, 1e-4, 0.0)))));
    assert_eq!(ex.err().map(|e| e.code), Some(ERR_UNSUPPORTED_TOPOLOGY), "exact mode");
    let fine = vec![psu(0, 1, 0, 5.0, 0.1), clock(1, 2, 0, 1000.0, 0.5, 0), capacitor(2, 2, 0, 1e-4, 0.0), capacitor(3, 1, 0, 1e-4, 0.0), resistor(4, 1, 0, 1000.0)];
    assert!(PartitionedSolver::build(&program(3, fine)).is_ok());
}

#[test]
fn the_cc_rail_sits_esr_eff_i_cap_above_the_charge_of_two_rail_capacitors() {
    // {5 V, 0.1 A} with 100 uF at esr 0.05 and 300 uF at esr 0.02 across it,
    // both discharged (Rails point 4; manager ruling 2026-10-07, F1 finding
    // C). Past their esr C transients (5 and 6 us, not modelled) they ramp
    // together, each taking its share C_k / C_rail of the current (25 and 75
    // mA), and the rail sits esr_eff I_cap above their C-weighted charge,
    // esr_eff = (0.05 1e-8 + 0.02 9e-8) / 1.6e-7 = 14.375 milliohm, while the
    // charge rises at 0.1 / 400 uF = 250 V/s. The rail reaches 5 V with the
    // charge esr_eff I_cap short of it, and leaves CC there: each capacitor
    // one esr drop short of the rail, carrying its share on, relaxing with
    // its own esr C.
    let (c1, e1, c2, e2, i_limit) = (1e-4, 0.05, 3e-4, 0.02, 0.1);
    let c_rail = c1 + c2;
    let esr_eff = (e1 * c1 * c1 + e2 * c2 * c2) / (c_rail * c_rail);
    let p = program(2, vec![psu(0, 1, 0, 5.0, i_limit), capacitor(1, 1, 0, c1, e1), capacitor(2, 1, 0, c2, e2)]);
    let mut rig = Rig::load(&p).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    close(rig.hub().esr_eff, 0.014375, 1e-15, "esr_eff");
    close(rig.u(), esr_eff * i_limit, 1e-15, "the rail at load: the offset alone");
    for t in [2e-3, 10e-3] {
        rig.advance(t).unwrap();
        close(rig.v(1), i_limit * t / c_rail + esr_eff * i_limit, 1e-9, &format!("u at {t:e}"));
        close(rig.i(1), i_limit * c1 / c_rail, 1e-11, &format!("the 100 uF's share at {t:e}"));
        close(rig.i(2), i_limit * c2 / c_rail, 1e-11, &format!("the 300 uF's share at {t:e}"));
        assert_eq!(rig.i(0), -i_limit);
    }
    let t_exit = cc_exit(&mut rig, 30e-3);
    close(t_exit, (5.0 - esr_eff * i_limit) * c_rail / i_limit, 1e-11, "the exit, where the rail reaches 5 V");
    assert_eq!(rig.v(1), 5.0);
    close(rig.i(1), i_limit * c1 / c_rail, 1e-11, "the 100 uF through the exit");
    close(rig.i(2), i_limit * c2 / c_rail, 1e-11, "the 300 uF through the exit");
    rig.advance(t_exit + e1 * c1).unwrap();
    close(rig.i(1), i_limit * c1 / c_rail * (-1.0f64).exp(), 1e-9, "the 100 uF one esr C later");
    rig.advance(t_exit + 10e-3).unwrap();
    assert_eq!((rig.i(1), rig.i(2)), (0.0, 0.0), "both settled at the rail");
    close(rig.i(0), -5.0 * static_part::NODE_RSHUNT_G, 1e-24, "the PSU supplies the shunt");
}

#[test]
fn a_load_step_into_cc_drops_the_rail_by_the_capacitors_esr_drop_at_once() {
    // {5 V, 0.1 A} with 100 uF at esr 0.05 loaded charged to 5 V: CV. A
    // switch adds 25 ohm (0.2 A at 5 V): CC at the toggle (Rails point 5),
    // and the capacitor supplies the difference, so the rail steps down by
    // its esr drop at once (Rails point 4): u = q + esr (iLimit - G u) with
    // the charge q at 5 V, u(0+) = (5 + 0.05 0.1) / (1 + 0.05 G) = 4.99501 V
    // for G = 1 / 25.001 (the switch's milliohm). It falls from there toward
    // iLimit / G = 2.5 V at the rate G / (C (1 + G esr)), the PSU at the limit.
    let (c, esr, r, i_limit) = (1e-4, 0.05, 25.0, 0.1);
    let p = program(3, vec![psu(0, 1, 0, 5.0, i_limit), capacitor(1, 1, 0, c, esr), switch(2, 1, 2, false), resistor(3, 2, 0, r)]);
    let mut rig = Rig::load_carrying(&p, &[(1, 5.0)]).unwrap();
    rig.advance(1e-3).unwrap();
    assert_eq!((rig.hub().state.reg, rig.v(1)), (Regime::Cv, 5.0));
    toggle(&mut rig, 2, true).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc, "CC at the load step");
    // The load as the rail sees it: the rail node's shunt, and the switch in
    // series with 25 ohm and node 2's shunt.
    let g_s = static_part::NODE_RSHUNT_G;
    let (g_sw, g_load) = (1.0 / 0.001, 1.0 / r + g_s);
    let g = g_s + g_sw * g_load / (g_sw + g_load);
    let u0 = (5.0 + esr * i_limit) / (1.0 + g * esr);
    close(rig.v(1), u0, 1e-11, "the rail one esr drop below the charge");
    close(u0, 4.995010, 1e-6, "4.99501 V");
    close(rig.i(1), i_limit - g * u0, 1e-11, "the capacitor supplies the rest");
    assert_eq!(rig.i(0), -i_limit);
    let (u_inf, k) = (i_limit / g, g / (c * (1.0 + g * esr)));
    for dt in [1e-3, 5e-3] {
        rig.advance(1e-3 + dt).unwrap();
        close(rig.v(1), u_inf + (u0 - u_inf) * (-k * dt).exp(), 1e-9, &format!("u {dt:e} into CC"));
        assert_eq!(rig.i(0), -i_limit);
    }
}

#[test]
fn a_fast_inrush_drives_a_closed_form_hub_partition_by_the_known_rail() {
    // {0.1 V, 1 A} with 100 nF across the rail and an RC from it (110 kohm,
    // 10 uF, a closed-form hub partition). The discharged rail capacitor
    // puts the PSU in CC at load, and the rail ramps at about 1e7 V/s to 0.1
    // V at 10 ns. While the rail is that known closed form the RC follows it
    // exactly (Rails point 4; F1r3 finding B, option (i)), so the rail
    // solves at its own thresholds only, each a line on its segment: the
    // scan widths of 20 mV, the last of them the exit. (Round 2 re-aimed the
    // RC at every RAIL_DRIFT_V, 10,000 events here and 500,000 per 5 V; and
    // when the closed-form term shared the distance, events closed on the
    // first scan threshold geometrically until the next was under the
    // kernel's 1e-15 s floor.)
    let (r, c) = (110e3, 1e-5);
    let p = program(3, vec![psu(0, 1, 0, 0.1, 1.0), capacitor(1, 1, 0, 1e-7, 0.0), resistor(2, 1, 2, r), capacitor(3, 2, 0, c, 0.0)]);
    let mut rig = Rig::load(&p).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    let t_exit = cc_exit(&mut rig, 1e-7);
    // C du/dt = 1 A less the RC's draw, under 1 uA: 10 ns to 0.1 V.
    close(t_exit, 1e-8, 1e-14, "the exit");
    // The RC's draw moves by under 1e-11 A in 10 ns: the drift bound adds no
    // event to the scan widths.
    let events = rig.hub().stats.events;
    assert!(events <= 6, "the scan widths and the exit: {events}");
    // From then on the RC charges toward 0.1 V with its node shunt, half the
    // ramp late (a linear ramp's lag, to second order in ramp/tau).
    let g = 1.0 / r + static_part::NODE_RSHUNT_G;
    let (x_inf, tau) = (0.1 / (r * g), c / g);
    for t in [1e-3, 0.1] {
        rig.advance(t).unwrap();
        let want = x_inf * -(-(t - t_exit / 2.0) / tau).exp_m1();
        close(rig.v(2), want, 1e-12 + 1e-9 * want, &format!("the RC at {t:e}"));
    }
    eprintln!("inrush: exit at {t_exit:.15e}, rail events {}, moves {}", rig.hub().stats.events, rig.hub().stats.moves);
}

#[test]
fn leaving_cc_every_hub_partition_solves_again_at_the_setpoint() {
    // {5 V, 1 A} with 100 nF across it, discharged: CC from load, the rail
    // ramping at about 1e7 V/s to 5 V (Rails points 4 and 5). On the rail, a
    // 74LS00 whose first gate has its inputs grounded: powered from 4.275 V,
    // its output is HIGH through R_out = vs / 0.032 into 10 kohm. That
    // partition is linear but for R_out, so between band re-solves it
    // carries a first-order correction, about 1 mV off its own law over the
    // ramp's last 0.6 V (F1 finding D: 1.1 nA of PSU current in corpus 09).
    // Leaving CC, every hub partition solves again at 5 V (Rails point 5), so
    // the board reads as the same board loaded charged, in CV throughout.
    let devices = || vec![psu(0, 1, 0, 5.0, 1.0), capacitor(1, 1, 0, 1e-7, 0.0), ls00(2, 1, 0, [0, 0, 2, 1, 1, 3, 1, 1, 4, 1, 1, 5], 10e-9, 0), resistor(3, 2, 0, 1e4)];
    let mut rig = Rig::load(&program(6, devices())).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    let t_exit = cc_exit(&mut rig, 1e-6);
    let mut reference = Rig::load_carrying(&program(6, devices()), &[(1, 5.0)]).unwrap();
    assert_eq!(reference.hub().state.reg, Regime::Cv);
    let t = t_exit + 1e-6;
    rig.advance(t).unwrap();
    reference.advance(t).unwrap();
    let pid = rig.s().local_index[2].unwrap().0;
    let (_, _, _, _, u_p) = rig.s().static_solver.hub_solve(pid).expect("a static hub partition");
    assert_eq!(u_p, 5.0, "solved again at the setpoint");
    // R_out's law at 5 V: 156.25 ohm into 10 kohm, with the node's shunt.
    let g_out = 1.0 / 156.25;
    close(rig.v(2), 5.0 * g_out / (g_out + 1e-4 + static_part::NODE_RSHUNT_G), 1e-12, "the output on its law");
    close(rig.v(2), reference.v(2), 1e-12, "the output, as loaded charged");
    close(rig.i(0), reference.i(0), 1e-15, "the PSU current, as loaded charged");
    eprintln!("CC exit: at {t_exit:.6e}, output {:.12} (reference {:.12}), PSU {:.6e} A (reference {:.6e})", rig.v(2), reference.v(2), rig.i(0), reference.i(0));
}

/// The coupled closed form of a CC rail capacitor `c_r` charged at `i` with
/// the rail node's shunt, an RC (`r`, `c` and its node's shunt) hanging off
/// the rail, both from 0 V: `(u(t), v(t))`. Each of the two real modes is
/// integrated in closed form, `y(t) = sum E1(l, t) P_l (i / c_r, 0)` with the
/// spectral projectors `P_l`, so nothing cancels at the slow mode.
fn coupled_rc(i: f64, c_r: f64, r: f64, c: f64, t: f64) -> (f64, f64) {
    let g_s = static_part::NODE_RSHUNT_G;
    let (a11, a21, a22) = (-(g_s + 1.0 / r) / c_r, 1.0 / (r * c), -(1.0 / r + g_s) / c);
    let tr = a11 + a22;
    let det = g_s * (g_s + 2.0 / r) / (c_r * c);
    let l1 = 0.5 * (tr - (tr * tr - 4.0 * det).sqrt());
    let l2 = det / l1;
    let b = i / c_r;
    let (e1, e2) = (rails::exp_integral(l1, t), rails::exp_integral(l2, t));
    let p1 = ((a11 - l2) * b / (l1 - l2), a21 * b / (l1 - l2));
    let p2 = ((a11 - l1) * b / (l2 - l1), a21 * b / (l2 - l1));
    (p1.0 * e1 + p2.0 * e2, p1.1 * e1 + p2.1 * e2)
}

#[test]
fn a_closed_form_hub_partition_follows_a_slow_inrush_as_the_coupled_closed_form() {
    // {5 V, 0.1 A} with 100 uF across the rail: a 5 ms inrush at 1000 V/s
    // (Rails point 4). An RC hangs off the rail (1 kohm, 1 uF, a closed-form
    // hub partition), lagging it by about 1 V, and a NAND with Schmitt inputs
    // on the rail reads the RC's node: vt- = 0.9 u / 5 and vt+ = 1.7 u / 5
    // ride the rail, and the node crosses vt- near 0.41 ms and vt+ near 0.9
    // ms. The pair is a coupled linear system with a closed form
    // (`coupled_rc`). Driven by the rail's own closed form, the RC is exact
    // against the segment; the segment holds the RC's draw where it stood,
    // its drift bounded to RAIL_DRIFT_V between solves and charged back at
    // each (F1r3 finding B, option (i)). So the rail and the RC stay on the
    // coupled closed form, and each crossing lands where the closed form
    // meets its moving threshold.
    let (i, c_r, r, c) = (0.1, 1e-4, 1e3, 1e-6);
    let devices = vec![psu(0, 1, 0, 5.0, i), capacitor(1, 1, 0, c_r, 0.0), resistor(2, 1, 2, r), capacitor(3, 2, 0, c, 0.0), schmitt_nand(4, 1, [2, 2, 3, 0, 0, 4, 0, 0, 5, 0, 0, 6], 0)];
    let mut rig = Rig::load(&program(7, devices)).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    // Each crossing of the closed form, and the time the node takes there to
    // move one drift width against its threshold: the rail the RC follows
    // trails the coupled one by under a drift width between solves.
    let cross = |frac: f64| {
        let g = |t: f64| {
            let (u, v) = coupled_rc(i, c_r, r, c, t);
            v - frac * u
        };
        let (mut lo, mut hi) = (1e-6, 4e-3);
        assert!(g(lo) < 0.0 && g(hi) > 0.0);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if g(mid) >= 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        let rate = (g(hi + 1e-7) - g(hi - 1e-7)) / 2e-7;
        (hi, rails::RAIL_DRIFT_V / rate)
    };
    let ((want_minus, tol_minus), (want_plus, tol_plus)) = (cross(0.9 / 5.0), cross(1.7 / 5.0));
    let mut seen = Vec::new();
    let mut count = rig.s().closed_form.crossings();
    let (mut worst_u, mut worst_v): (f64, f64) = (0.0, 0.0);
    loop {
        let t = rig.k.next_event_time();
        if t > 4.5e-3 {
            break;
        }
        rig.advance(t).unwrap();
        let (u, v) = coupled_rc(i, c_r, r, c, t);
        worst_u = worst_u.max((rig.v(1) - u).abs());
        worst_v = worst_v.max((rig.v(2) - v).abs());
        let n = rig.s().closed_form.crossings();
        if n > count {
            seen.push(t);
            count = n;
        }
    }
    let events = rig.hub().stats.events;
    eprintln!(
        "slow inrush: crossings at {seen:?} (closed form {want_minus:.9e}, {want_plus:.9e}); worst |u - closed form| {worst_u:.3e}, |v - closed form| {worst_v:.3e} over {events} rail events"
    );
    assert_eq!(seen.len(), 2, "vt- then vt+, nothing else: {seen:?}");
    close(seen[0], want_minus, tol_minus, "the node crosses vt- where the closed form does");
    close(seen[1], want_plus, tol_plus, "the node crosses vt+ where the closed form does");
    assert!(worst_u <= rails::RAIL_DRIFT_V && worst_v <= rails::RAIL_DRIFT_V, "on the coupled closed form within one drift width");
}

#[test]
fn leaving_cc_ends_the_drive_of_a_watched_closed_form_partition_even_without_a_move() {
    // {5 V, 0.1 A} with 100 uF across the rail, discharged: CC from load, a
    // 5 ms inrush at 1000 V/s (Rails points 4 and 5). A fixed 5 V source
    // charges 100 uF through 1 kohm (tau 100 ms), a closed-form partition that
    // does not read the rail, and a NAND with Schmitt inputs on the rail
    // reads its node: in CC its thresholds ride the rail (vt- = 0.9 u / 5,
    // vt+ = 1.7 u / 5) and outrun the node, so no crossing is pending. From
    // the exit on they stand at 0.9 and 1.7 V, which the node crosses rising
    // at tau ln(5 / 4.1) and tau ln(5 / 3.3), long after it. One run takes
    // the exit as it comes, a move of the rail onto the setpoint. The other
    // first solves the rail within RAIL_MOVE_EPS below the setpoint, so the
    // exit does not move it: the partition's drive must end with the segment
    // all the same (F1r3 R3-2), or it keeps waiting on thresholds still
    // riding a ramp that has stopped, and both crossings are lost.
    let devices = || {
        vec![
            psu(0, 1, 0, 5.0, 0.1),
            capacitor(1, 1, 0, 1e-4, 0.0),
            source(2, 3, 0, 5.0),
            resistor(3, 3, 2, 1e3),
            capacitor(4, 2, 0, 1e-4, 0.0),
            schmitt_nand(5, 1, [2, 2, 4, 0, 0, 5, 0, 0, 6, 0, 0, 7], 0),
        ]
    };
    let v = 5.0;
    let (r, c, g_s) = (1e3, 1e-4, static_part::NODE_RSHUNT_G);
    let (x_inf, tau) = (5.0 / (1.0 + r * g_s), c * r / (1.0 + r * g_s));
    for force in [false, true] {
        let what = if force { "an exit with no move" } else { "an exit that moves the rail" };
        let mut rig = Rig::load(&program(8, devices())).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cc, "{what}: CC from load");
        let p = rig.s().local_index[2].unwrap().0;
        assert!(rig.hub().reaim_cf.contains(&p) && !rig.hub().cf_hubs.contains(&p), "{what}: a watched closed form that does not read the rail");
        if force {
            // Up to the last rail event below the setpoint, then a solve
            // placed on the segment's line one RAIL_MOVE_EPS / 2 below it. The
            // segment is the one in force after that event: a scan re-aims it
            // at the draws there, which moves its line's arrival at the
            // setpoint by far more than the 5e-16 s this solve sits before it.
            let t_star = loop {
                assert_eq!(rig.hub().state.reg, Regime::Cc, "{what}: still CC");
                let seg = rig.hub().state.cap_seg.clone().expect("the CC segment");
                // The segment's own inverse: past k dt = RAIL_CAP_LINEAR it is
                // the exponential, already 1e-11 V under its line here.
                let d = v - 0.5 * rails::RAIL_MOVE_EPS - seg.u0;
                let t_star = seg.t0 + -(-d * seg.k / seg.slope).ln_1p() / seg.k;
                let t = rig.k.next_event_time();
                if t >= t_star {
                    break t_star;
                }
                rig.advance(t).unwrap();
            };
            rig.advance(t_star).unwrap();
            let held = rig.u();
            assert!(v - held > 0.0 && v - held <= rails::RAIL_MOVE_EPS, "{what}: the rail {held:.15} within RAIL_MOVE_EPS below the setpoint");
            // The exit is due within a few femtoseconds; an IC's own event
            // may come first, and none of them moves the rail.
            for _ in 0..4 {
                if rig.hub().state.reg == Regime::Cv {
                    break;
                }
                let t = rig.k.next_event_time();
                assert!(t - t_star < 1e-12, "{what}: the exit is due at once, not at {t:e}");
                rig.advance(t).unwrap();
                assert_eq!(rig.u().to_bits(), held.to_bits(), "{what}: an event before the exit does not move the rail");
            }
            assert_eq!(rig.hub().state.reg, Regime::Cv, "{what}: the exit");
            assert_eq!(rig.u().to_bits(), held.to_bits(), "{what}: and it does not move the rail");
        }
        let mut seen = Vec::new();
        let mut count = rig.s().closed_form.crossings();
        loop {
            let t = rig.k.next_event_time();
            if t > 50e-3 {
                break;
            }
            rig.advance(t).unwrap();
            let n = rig.s().closed_form.crossings();
            if n > count {
                seen.push(t);
                count = n;
            }
        }
        assert_eq!(rig.hub().state.reg, Regime::Cv, "{what}: CV");
        // The thresholds where they stand, at the rail as published. A
        // closed-form crossing is due where the node reaches the threshold
        // itself, and landing puts the node CROSS_NUDGE_V past it
        // (closed_form.rs's doc): the exponential goes on from there, so vt+
        // is reached from vt- plus the nudge.
        let u = rig.v(1);
        let (lo, hi) = (0.9 * u / 5.0, 1.7 * u / 5.0);
        let t_lo = -tau * (-lo / x_inf).ln_1p();
        let t_hi = t_lo + tau * ((x_inf - lo - closed_form::CROSS_NUDGE_V) / (x_inf - hi)).ln();
        assert_eq!(seen.len(), 2, "{what}: vt- then vt+, each once: {seen:?}");
        close(seen[0], t_lo, 1e-12, &format!("{what}: the node crosses vt- where it stands"));
        close(seen[1], t_hi, 1e-12, &format!("{what}: the node crosses vt+ where it stands"));
        assert_eq!(rig.v(4), 0.0, "{what}: 1y fell once the input read HIGH");
    }
}

#[test]
fn a_rail_capacitor_charged_to_the_setpoint_at_load_stays_cv() {
    // The same circuit with the capacitor loaded at 5 V, as a reload that
    // carries its charge loads it (contract test (b)): no deficit, so no
    // draw at the setpoint beyond the shunt, and no false entry. At esr 0.05
    // too, where its charge is the exponential's, already at dV.
    for esr in [0.0, 0.05] {
        let p = program(2, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, esr)]);
        let mut rig = Rig::load_carrying(&p, &[(1, 5.0)]).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cv, "esr {esr}: CV at load");
        assert_eq!(rig.u(), 5.0);
        assert_eq!(rig.i(1), 0.0, "esr {esr}: the capacitor carries nothing");
        rig.advance(10e-3).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cv, "esr {esr}: still CV");
        assert_eq!((rig.v(1), rig.i(1)), (5.0, 0.0));
        close(rig.i(0), -5.0 * static_part::NODE_RSHUNT_G, 1e-24, "the PSU supplies the shunt");
        assert_eq!((rig.hub().stats.moves, rig.hub().stats.events), (0, 0), "esr {esr}: the rail never moved");
    }
    // Charged short of the setpoint behind a large esr, the draw it adds is
    // the deficit over esr: 2 V over 100 ohm, 20 mA. Beside 100 ohm (50 mA)
    // that is inside the limit; beside 60 ohm (83 mA, inside it alone) it is
    // over it, and the PSU enters CC at load.
    for (load, cc) in [(100.0, false), (60.0, true)] {
        let p = program(2, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 100.0), resistor(2, 1, 0, load)]);
        let rig = Rig::load_carrying(&p, &[(1, 3.0)]).unwrap();
        assert_eq!(rig.hub().state.reg == Regime::Cc, cc, "beside {load} ohm");
        if !cc {
            close(rig.i(1), 0.02, 1e-15, "the capacitor charges through its esr");
        }
    }
}

#[test]
fn a_slight_overload_holds_cc_at_4_99_v_without_flicker() {
    // {5 V, 0.1 A} into 49.9 ohm draws 0.1002 A at 5 V: CC at 0.1 * 49.9 =
    // 4.99 V, and the draw at the setpoint stays over the limit, so CC holds
    // (fixed classic, `fix/psu-cc-exit`; before it, classic flickered every
    // step, probe P12).
    let p = program(2, vec![psu(0, 1, 0, 5.0, 0.1), resistor(1, 1, 0, 49.9)]);
    let mut rig = Rig::load(&p).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    let want = 0.1 / (1.0 / 49.9 + static_part::NODE_RSHUNT_G);
    rel(rig.u(), want, 1e-12, "u");
    rel(rig.u(), 4.99, 1e-9, "4.99 V");
    let moves = rig.hub().stats.moves;
    rig.advance(50e-3).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    assert_eq!(rig.state(0), vec![2.0], "reg 2");
    rel(rig.v(1), want, 1e-12, "u after 50 ms");
    assert_eq!(rig.i(0), -0.1);
    assert_eq!(rig.hub().stats.events, 0, "nothing moves the rail: no rail event");
    assert_eq!(rig.hub().stats.moves, moves, "and no solve moved it again");
}

#[test]
fn a_rail_capacitor_psu_enters_cc_at_the_load_step_and_holds_the_limit() {
    // {5 V, 0.1 A} with 100 uF across the rail and 100 ohm (50 mA at 5 V).
    // The capacitor loads discharged, so the PSU is in CC from load (Rails
    // point 5) and the rail rises from 0 V toward u_inf = 0.1 * 100 = 10 V
    // with tau = C / G = 10 ms, reaching 5 V at tau ln 2 = 6.93 ms, where it
    // leaves CC. A switch adds 50 ohm at 10 ms: the draw at 5 V is 0.15 A,
    // so CC at the toggle (fixed classic, `fix/psu-railcap-cc-entry`; probes
    // P10 and P11 held CV there). In CC the rail is the capacitor's closed
    // form, u_inf = 0.1 * 33.33 = 3.333 V, tau = C / G = 3.333 ms, the PSU
    // at -0.1 A.
    let (c, r1, r2) = (1e-4, 100.0, 50.0);
    let p = program(4, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, c, 0.0), resistor(2, 1, 0, r1), switch(3, 1, 2, false), resistor(4, 2, 0, r2)]);
    let mut rig = Rig::load(&p).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc, "the discharged capacitor's inrush");
    assert_eq!(rig.u(), 0.0);
    let g0 = 1.0 / r1 + static_part::NODE_RSHUNT_G;
    let (u_inf0, tau0) = (0.1 / g0, c / g0);
    let t_inrush = cc_exit(&mut rig, 8e-3);
    // Aimed two reached-widths past 5 V, where the rail rises at (0.1 - 5 G) / C.
    let overshoot = 3.0 * rails::REACHED_REL * 5.0 * c / (0.1 - 5.0 * g0);
    close(t_inrush, tau0 * (u_inf0 / (u_inf0 - 5.0)).ln(), overshoot, "the inrush ends where the rail reaches 5 V");
    let t1 = 10e-3;
    rig.advance(t1).unwrap();
    assert_eq!((rig.hub().state.reg, rig.v(1)), (Regime::Cv, 5.0));
    toggle(&mut rig, 3, true).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc, "CC at the load step");
    // A closed switch is 1 milliohm (devices/switches.rs).
    let g = 1.0 / r1 + 1.0 / (r2 + 0.001) + static_part::NODE_RSHUNT_G;
    let (u_inf, tau) = (0.1 / g, c / g);
    let cc = |t: f64| u_inf + (5.0 - u_inf) * (-(t - t1) / tau).exp();
    for k in 1..=6 {
        let t = t1 + tau * k as f64;
        rig.advance(t).unwrap();
        rel(rig.v(1), cc(t), 1e-9, &format!("u at {k} tau"));
        assert_eq!(rig.i(0), -0.1, "the PSU holds the limit");
        rel(rig.i(1), c * (u_inf - 5.0) / tau * (-(t - t1) / tau).exp(), 1e-6, "the rail capacitor carries C du/dt");
    }
    // 3.333378 + 1.666622 / e, the switch's milliohm included.
    close(cc(t1 + tau), 3.946494, 1e-6, "the closed form at one tau");
    // Removing the overload: the draw at 5 V is back within the limit, but a
    // rail capacitor leaves CC only when the rail reaches the setpoint: u
    // heads for 0.1 * 100 = 10 V with tau' = 10 ms and crosses 5 V there.
    let t2 = rig.k.time();
    let u2 = rig.v(1);
    toggle(&mut rig, 3, false).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc, "still CC below the setpoint");
    let g2 = 1.0 / r1 + static_part::NODE_RSHUNT_G;
    let (u_inf2, tau2) = (0.1 / g2, c / g2);
    let t_exit = t2 + tau2 * ((u_inf2 - u2) / (u_inf2 - 5.0)).ln();
    let mut exit = None;
    while exit.is_none() {
        let t = rig.k.next_event_time();
        assert!(t < t2 + 10e-3, "CC never ended");
        rig.advance(t).unwrap();
        if rig.hub().state.reg == Regime::Cv {
            exit = Some(t);
        }
    }
    let t = exit.unwrap();
    close(t, t_exit, 1e-12 * t_exit.max(1e-3) + 1e-15, "the exit where the rail reaches 5 V");
    assert_eq!(rig.v(1), 5.0);
    rig.advance(t + 1e-3).unwrap();
    rel(rig.i(0), -(5.0 * g2), 1e-9, "CV: the load alone");
    eprintln!("P10/P11: entry at {t1:e}, u(t1 + tau) {:.9}, exit at {t:.12e} (closed form {t_exit:.12e})", cc(t1 + tau));
}

// ---- Rails point 3: the 25 mV band (W1f Q6) ------------------------------------

#[test]
fn an_led_across_a_0_9_ohm_battery_agrees_with_exact_mode() {
    // Straight across: the LED is a direct device, exact in both modes.
    let devices = || vec![battery(0, 1, 0, 5.0, 0.9), led(1, 1, 0)];
    let rig = Rig::load(&program(2, devices())).unwrap();
    let ex = Rig::load(&exact(program(2, devices()))).unwrap();
    let (dv, di) = ((rig.u() - ex.v(1)).abs(), (rig.i(0) - ex.i(0)).abs());
    eprintln!("Q6 direct: u {:.12} exact {:.12} (dv {dv:.3e}); I {:.12} exact {:.12} (di {di:.3e})", rig.u(), ex.v(1), rig.i(0), ex.i(0));
    close(rig.u(), ex.v(1), 1e-9, "u against exact mode");
    close(rig.i(0), ex.i(0), 1e-9, "the battery current against exact mode");
    // Through a 1 ohm resistor: the LED sits in a nonlinear static hub
    // partition, which keeps its first-order correction inside the 25 mV
    // band. W0's tolerance is 100 uV + 1e-3 relative, 1 nA + 1e-3 relative.
    let devices = || vec![battery(0, 1, 0, 5.0, 0.9), led(1, 1, 2), resistor(2, 2, 0, 1.0)];
    let rig = Rig::load(&program(3, devices())).unwrap();
    let ex = Rig::load(&exact(program(3, devices()))).unwrap();
    let dv = (0..3).map(|n| (rig.v(n) - ex.v(n)).abs()).fold(0.0, f64::max);
    let di = (rig.i(0) - ex.i(0)).abs();
    eprintln!("Q6 partition: u {:.12} exact {:.12}; x {:.12} exact {:.12}; I {:.12} exact {:.12}; worst dv {dv:.3e}, di {di:.3e}", rig.u(), ex.v(1), rig.v(2), ex.v(2), rig.i(0), ex.i(0));
    for n in 1..3 {
        close(rig.v(n), ex.v(n), 1e-4 + 1e-3 * ex.v(n).abs(), &format!("node {n} within W0's tolerance"));
    }
    close(rig.i(0), ex.i(0), 1e-9 + 1e-3 * ex.i(0).abs(), "current within W0's tolerance");
}

/// A cell (device 0) into an LED (rail 1 to node 2) and 220 ohm.
fn led_cell(cell: Device) -> Program {
    program(3, vec![cell, led(1, 1, 2), resistor(2, 2, 0, 220.0)])
}

/// The rail, the LED's cathode, the battery's draw and its discharged charge
/// at 0.1 s and 1 s. The host advances to each `instants` in turn.
fn led_cell_run(p: &Program, instants: &[f64]) -> Vec<[f64; 4]> {
    let mut e = Engine::load(p).unwrap();
    let mut out = Vec::new();
    for &t in instants {
        e.advance(t).unwrap();
        if [0.1, 1.0].iter().any(|&s| (t - s).abs() < 1e-12) {
            let discharged = e.device_state(0).unwrap()[4];
            out.push([e.voltages()[1], e.voltages()[2], -e.currents()[0], discharged]);
        }
    }
    out
}

/// Classic's cadence by hand (SO:141-156) at its step `h`: E and r from the
/// SoC at each step's start, the circuit's static solve with the cell as E
/// behind r, then SoC charged with that solve's draw over `h`. The same
/// readings as [`led_cell_run`].
fn led_cell_reference(cell: &Device, h: f64) -> Vec<[f64; 4]> {
    let model = crate::devices::battery::BatteryModel::from_record(&cell.params, &cell.blob);
    let mut state = crate::devices::battery::BatteryState::initial(&model);
    let mut out = Vec::new();
    let mut k = 0u32;
    for t in [0.1, 1.0] {
        let mut last = [0.0; 3];
        while k < (t / h).round() as u32 {
            last = led_cell_classic_step(&model, &mut state, h);
            k += 1;
        }
        out.push([last[0], last[1], last[2], state.discharged]);
    }
    out
}

/// One step of classic's cadence over `h`: the static solve with E and r
/// from `state`'s SoC, then SoC charged with its draw. The rail, the
/// cathode and the draw.
fn led_cell_classic_step(model: &crate::devices::battery::BatteryModel, state: &mut crate::devices::battery::BatteryState, h: f64) -> [f64; 3] {
    let (e, r) = (model.emf(state.soc), model.resistance(state.soc));
    let p = if r > 0.0 {
        program(4, vec![source(0, 3, 0, e), resistor(3, 3, 1, r), led(1, 1, 2), resistor(2, 2, 0, 220.0)])
    } else {
        program(3, vec![source(0, 1, 0, e), led(1, 1, 2), resistor(2, 2, 0, 220.0)])
    };
    let solve = Engine::load(&p).unwrap();
    let (u, x) = (solve.voltages()[1], solve.voltages()[2]);
    let draw = x / 220.0;
    *state = model.advance(state, draw * h).state;
    [u, x, draw]
}

/// [`led_cell_run`] through the kernel itself, with the instants at which
/// the cell's partition committed its steps.
fn led_cell_run_steps(p: &Program, instants: &[f64]) -> (Vec<[f64; 4]>, Vec<f64>) {
    let mut rig = Rig::load(p).unwrap();
    let mut out = Vec::new();
    for &t in instants {
        rig.advance(t).unwrap();
        if [0.1, 1.0].iter().any(|&s| (t - s).abs() < 1e-12) {
            out.push([rig.v(1), rig.v(2), -rig.i(0), rig.state(0)[4]]);
        }
    }
    let cell = rig.s().exact.as_ref().unwrap().partition;
    let steps = rig.s().stepped.commit_log.iter().filter(|&&(p, _)| p == cell).map(|&(_, t)| t).collect();
    (out, steps)
}

/// [`led_cell_reference`] at the step sequence `steps` (each step's end; the
/// first starts at 0) instead of a fixed step: what the cell would read had
/// each step taken E and r from the SoC at its start.
fn led_cell_reference_at(cell: &Device, steps: &[f64]) -> Vec<[f64; 4]> {
    let model = crate::devices::battery::BatteryModel::from_record(&cell.params, &cell.blob);
    let mut state = crate::devices::battery::BatteryState::initial(&model);
    let mut out = Vec::new();
    let (mut k, mut t0) = (0, 0.0);
    for t in [0.1, 1.0] {
        let mut last = [0.0; 3];
        while k < steps.len() && steps[k] <= t + 1e-15 {
            last = led_cell_classic_step(&model, &mut state, steps[k] - t0);
            t0 = steps[k];
            k += 1;
        }
        assert!((t0 - t).abs() < 1e-12, "a step ends at each reading: {t0} for {t}");
        out.push([last[0], last[1], last[2], state.discharged]);
    }
    out
}

/// Runs `p` (a cell of `capacity_ah` as device 0 on rail 1, the LED to node 2
/// and 220 ohm, plus nothing that loads either node) in exact mode for each
/// host and checks every reading against [`led_cell_reference_at`] at the
/// cell's own steps, with W0's tolerances.
fn assert_tracks_soc_at_own_steps(name: &str, p: &dyn Fn(Device) -> Program, r: f64, capacity_ah: f64) {
    let every_ms: Vec<f64> = (1..=1000).map(|k| k as f64 * 1e-3).collect();
    let cell = generic_battery(0, 1, 0, 5.0, r, capacity_ah);
    let mut checks = Vec::new();
    for (host, instants) in [("1 ms", &every_ms[..]), ("jumps", &[0.1, 1.0][..])] {
        let (run, steps) = led_cell_run_steps(&exact(p(cell.clone())), instants);
        let reference = led_cell_reference_at(&cell, &steps);
        // Not vacuous: the rail fell by tenths of a volt.
        assert!(reference[0][0] - reference[1][0] > 0.1, "{name}: the rail falls: {reference:?}");
        for (k, t) in [0.1, 1.0].into_iter().enumerate() {
            let tol = |want: f64, q: usize| match q {
                0 | 1 => 1e-4 + 1e-3 * want.abs(),
                2 => 1e-9 + 1e-3 * want.abs(),
                _ => (1e-9 + 1e-3 * want / t) * t,
            };
            let off: Vec<f64> = (0..4).map(|q| (run[k][q] - reference[k][q]) / tol(reference[k][q], q)).collect();
            eprintln!("{name} r {r} cap {capacity_ah:e} host {host} t {t} ({} steps): rail {:.3} tol, cathode {:.3}, draw {:.3}, discharged {:.3}", steps.len(), off[0], off[1], off[2], off[3]);
            for (q, what) in ["the rail", "the LED cathode", "the draw", "the discharged charge"].into_iter().enumerate() {
                checks.push((run[k][q], reference[k][q], tol(reference[k][q], q), format!("{name}, r {r}, {capacity_ah:e} Ah, host {host}: {what} at {t} s")));
            }
        }
    }
    // Every reading printed before the first failure.
    for (got, want, tol, what) in checks {
        close(got, want, tol, &what);
    }
}

/// [`led_cell`] plus a 74LS00 powered from its own RC node (5 V through 1 k
/// into 1 uF, node 3), its 1a input on the rail: the watch on the rail takes
/// its supply from another stepped partition. `load` puts a resistor of that
/// value from 1y (node 5) to ground.
fn led_cell_watched(cell: Device, load: Option<f64>) -> Program {
    let mut devices = vec![
        cell,
        led(1, 1, 2),
        resistor(2, 2, 0, 220.0),
        source(3, 4, 0, 5.0),
        resistor(4, 4, 3, 1e3),
        capacitor(5, 3, 0, 1e-6, 0.0),
        ls00(6, 3, 0, [1, 3, 5, 3, 3, 6, 3, 3, 7, 3, 3, 8], 10e-9, 0),
    ];
    if let Some(r) = load {
        devices.push(resistor(7, 5, 0, r));
    }
    program(9, devices)
}

#[test]
fn exact_mode_battery_steps_take_e_and_r_from_the_soc_at_their_start() {
    // FX2 (Rails point 9): each step of the cell's partition takes E and r
    // from the SoC at the step's start, so SoC must be charged with the
    // step just committed before the next one is planned. The reference is
    // classic's cadence by hand at the controller's OWN step sequence
    // (`led_cell_reference_at`), not classic at 1 ms: at 10 ms steps the
    // step's own error is 1-2 tolerances, which is classic's at that step
    // and no defect. A one-step lag shows clearly only on a fast-draining
    // cell: 25 uAh at about 13 mA is some 15 % a second.
    for r in [0.9, 0.0] {
        assert_tracks_soc_at_own_steps("bare", &led_cell, r, 2.5e-5);
    }
}

/// [`led_cell`] plus a 74LS00 powered from the rail, its 1a input on an RC
/// node (5 V through 1 k into 1 uF, node 3): the cell's partition supplies
/// a watch in another stepped partition.
fn led_cell_supplying(cell: Device) -> Program {
    led_cell_supplying_rc(cell, 1e-6)
}

/// [`led_cell_supplying`] with `c` for the RC's 1 uF.
fn led_cell_supplying_rc(cell: Device, c: f64) -> Program {
    program(
        9,
        vec![
            cell,
            led(1, 1, 2),
            resistor(2, 2, 0, 220.0),
            source(3, 4, 0, 5.0),
            resistor(4, 4, 3, 1e3),
            capacitor(5, 3, 0, c, 0.0),
            ls00(6, 1, 0, [3, 1, 5, 1, 1, 6, 1, 1, 7, 1, 1, 8], 10e-9, 0),
        ],
    )
}

/// Whether the cell's partition (rail 1) and the RC's (node 3) are both
/// stepped and distinct, so one's watch takes its supply from the other.
fn assert_cell_and_rc_apart(p: &Program) {
    let pb = crate::partition::build(p).unwrap();
    let (cell_p, rc_p) = (pb.local_index[1].unwrap().0, pb.local_index[3].unwrap().0);
    assert!(cell_p != rc_p && pb.kinds[rc_p] == PartitionKind::Stepped && pb.kinds[cell_p] == PartitionKind::Stepped, "two stepped partitions");
}

#[test]
fn exact_mode_battery_steps_take_e_and_r_from_the_soc_at_their_start_under_a_watch() {
    // FX2: as the test above, with the rail read by a 74LS00 powered from
    // another stepped partition (FX1v's board), so the cell's partition has a
    // supply to bring to each plan's instant. Its step is planned after the
    // SoC charge all the same. The IC loads neither node the reference
    // solves.
    assert_cell_and_rc_apart(&exact(led_cell_watched(generic_battery(0, 1, 0, 5.0, 0.9, 2.5e-5), None)));
    for r in [0.9, 0.0] {
        assert_tracks_soc_at_own_steps("watched", &|cell| led_cell_watched(cell, None), r, 2.5e-5);
    }
}

#[test]
fn exact_mode_battery_steps_take_e_and_r_from_the_soc_at_their_start_supplying_a_watch() {
    // FX2: the other way round, a 74LS00 powered from the rail reading the
    // RC node. Each step the RC's partition takes first brings the cell's to
    // its instant (`sync_supplies`), and that step of the cell's, too, is
    // planned after the SoC charge, with the RC's plan after it, reading its
    // trial. The lag is in the RC's first milliseconds, where its steps are
    // many, so the cell drains faster here, 10 uAh. The IC loads neither
    // node the reference solves.
    assert_cell_and_rc_apart(&exact(led_cell_supplying(generic_battery(0, 1, 0, 5.0, 0.9, 1e-5))));
    for r in [0.9, 0.0] {
        assert_tracks_soc_at_own_steps("supplying", &led_cell_supplying, r, 1e-5);
    }
}

#[test]
fn deferring_the_battery_plan_keeps_the_supply_catch_up_on_the_pre_event_drivers() {
    // FX2: when the cell's step waits for the SoC charge, the supply its
    // watch reads is still brought to the event's instant at once, with the
    // driver table in force BEFORE the event (finding 5), exactly as when
    // the plan is not held back. On FX1v's board with 1y loaded by 10 k,
    // the solver is called as at an event 3 ms after the host's last
    // advance (so the RC's partition is 3 ms behind) whose post-event table
    // flips 1y. Deferred and undeferred must agree bit for bit; a catch-up
    // that read the post-event table would differ.
    let p = exact(led_cell_watched(generic_battery(0, 1, 0, 5.0, 0.9, 1e-4), Some(1e4)));
    let rig_at = |defer: bool| {
        let mut rig = Rig::load(&p).unwrap();
        rig.advance(0.05).unwrap();
        if !defer {
            rig.k.solver_mut().stepped.set_defer(None);
        }
        rig
    };
    let (cell_p, supply_p) = {
        let rig = rig_at(true);
        (rig.s().exact.as_ref().unwrap().partition, rig.s().local_index[3].unwrap().0)
    };
    let pre: Vec<Driver> = rig_at(true).s().driver_snapshot.to_vec();
    let y = pre.iter().position(|d| d.node == 5).expect("1y's driver");
    let mut post = pre.clone();
    let flip = if post[y].committed() == Some(Mode::High) { Mode::Low } else { Mode::High };
    let (_, mode, token) = post[y].request(flip, 0.05, &crate::drivers::DelaySet::uniform(0.0)).expect("a level change schedules a commit");
    assert!(post[y].apply_commit(mode, token));
    let now = 0.05 + 3e-3;
    // The stepped state right after the resolve and once the held plans are
    // made, the readers, and the published voltages.
    let run = |defer: bool, catchup: &[Driver]| {
        let mut rig = rig_at(defer);
        let s = rig.k.solver_mut();
        assert_eq!(s.stepped.time_of(supply_p), Some(0.05), "the supply is behind the event");
        let readers = s.stepped.resolve_dirty(&[cell_p], now, catchup, &post, &s.live_params, &mut s.voltages, &mut s.currents).unwrap();
        let held = s.stepped.snapshot().digest();
        s.stepped.plan_deferred(&post, &s.live_params, &s.voltages).unwrap();
        assert_eq!(s.stepped.time_of(supply_p), Some(now), "the supply was brought to the event");
        (held, s.stepped.snapshot().digest(), readers, s.voltages.iter().map(|v| v.to_bits()).collect::<Vec<_>>())
    };
    let (held, deferred, readers, voltages) = run(true, &pre);
    let (planned, undeferred, readers_undeferred, voltages_undeferred) = run(false, &pre);
    assert_ne!(held, planned, "the cell's plan was held back");
    assert_eq!(deferred, undeferred, "the held plan, once made, is the one made at once");
    assert_eq!((readers, voltages), (readers_undeferred, voltages_undeferred));
    let (_, wrong, _, _) = run(true, &post);
    assert_ne!(deferred, wrong, "not vacuous: the supply's catch-up reads 1y");
}

#[test]
fn a_plan_reading_the_held_battery_trial_is_made_after_it_whatever_the_held_order() {
    // FX2's ordering (Rails point 9), read from the DEPENDENT partition: on
    // `led_cell_supplying` with a slow RC (1 k into 100 uF, still charging
    // through the 74LS00's threshold at 50 ms) the RC's watch takes its
    // supply from the cell's partition, so its plan reads the cell's trial,
    // which comes only after the SoC charge. A host clamp past the cell's
    // trial catches the cell up and holds its plan, so the RC's plan must
    // be held too and made after the cell's, whichever order the two were
    // held in. The solver restamps the cell at its commit whether or not the
    // plan is held (FX5), and the owner charges nothing in between here, so
    // every order must give the plans made at once with no deferral, bit
    // for bit, and the RC's next step must then publish the same RC node at
    // the same instant with the same readers. Swept over instants, since an RC plan reads
    // the cell's trial only when that trial ends inside the RC's step.
    let p = exact(led_cell_supplying_rc(generic_battery(0, 1, 0, 5.0, 0.9, 1e-4), 1e-4));
    assert_cell_and_rc_apart(&p);
    let base = 0.05;
    let rig_at = |defer: bool| {
        let mut rig = Rig::load(&p).unwrap();
        rig.advance(base).unwrap();
        if !defer {
            rig.k.solver_mut().stepped.set_defer(None);
        }
        rig
    };
    let (cell_p, rc_p) = {
        let rig = rig_at(true);
        (rig.s().exact.as_ref().unwrap().partition, rig.s().local_index[3].unwrap().0)
    };
    let mut read_the_trial = 0;
    for k in 1..=12 {
        let now = base + k as f64 * 0.37e-3;
        // `orders`: the held lists to plan from in turn, empty for the one
        // the clamp made. The stepped state once the held plans are made,
        // then the RC node, the RC's instant and the readers after its next
        // step.
        let run = |defer: bool, orders: &[&[usize]]| {
            let mut rig = rig_at(defer);
            let s = rig.k.solver_mut();
            let drivers: Vec<Driver> = s.driver_snapshot.to_vec();
            s.stepped.sync_to(now, &drivers, &s.live_params, &mut s.voltages, &mut s.currents).unwrap();
            if defer {
                assert_eq!(*s.stepped.deferred_mut(), vec![cell_p, rc_p], "at {now}: the cell's plan and then the RC's are held");
            }
            for order in orders {
                *s.stepped.deferred_mut() = order.to_vec();
                s.stepped.plan_deferred(&drivers, &s.live_params, &s.voltages).unwrap();
            }
            s.stepped.plan_deferred(&drivers, &s.live_params, &s.voltages).unwrap();
            let planned = s.stepped.snapshot().digest();
            let readers = s.stepped.handle_step(rc_p, now, &drivers, &s.live_params, &mut s.voltages, &mut s.currents).unwrap();
            s.stepped.plan_deferred(&drivers, &s.live_params, &s.voltages).unwrap();
            (planned, s.voltages[3].to_bits(), s.stepped.time_of(rc_p).unwrap().to_bits(), readers)
        };
        let at_once = run(false, &[]);
        for orders in [&[][..], &[&[cell_p, rc_p][..]], &[&[rc_p, cell_p][..]]] {
            assert_eq!(run(true, orders), at_once, "at {now}, held as {orders:?}: the plans, then the RC node, its instant and the readers after its next step");
        }
        // An RC plan made before the cell's reads no trial of the cell's.
        let early = run(true, &[&[rc_p], &[cell_p]]);
        if (early.1, early.2) != (at_once.1, at_once.2) {
            read_the_trial += 1;
        }
    }
    assert!(read_the_trial > 0, "not vacuous: at some instant the RC's next step reads the cell's trial");
}

#[test]
fn a_rollback_after_a_held_battery_plan_drops_the_hold() {
    // FX2's deferral (Rails point 9) inside an instant that then fails: a
    // resolve holds the cell's plan, and something after it in the same
    // call fails before the plans are made, so the instant rolls back. The
    // hold must go with it: the solver as the twin that never tried, and
    // going on as it does.
    let p = exact(led_cell(generic_battery(0, 1, 0, 5.0, 0.9, 1e-4)));
    let (mut failed, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
    failed.advance(0.05).unwrap();
    twin.advance(0.05).unwrap();
    let before = state_digest(failed.s());
    {
        let s = failed.k.solver_mut();
        let cell_p = s.exact.as_ref().unwrap().partition;
        let checkpoint = s.begin_instant();
        let drivers: Vec<Driver> = s.driver_snapshot.to_vec();
        s.begin_dynamic_writes_list(&[cell_p]);
        s.stepped.resolve_dirty(&[cell_p], 0.07, &drivers, &drivers, &s.live_params, &mut s.voltages, &mut s.currents).unwrap();
        s.end_dynamic_writes();
        assert_eq!(*s.stepped.deferred_mut(), vec![cell_p], "the cell's plan is held");
        s.rollback_instant(checkpoint);
    }
    assert_eq!(state_digest(failed.s()), before, "every piece of state");
    for t in [0.06, 0.07, 0.1] {
        failed.advance(t).unwrap();
        twin.advance(t).unwrap();
        assert_eq!(state_digest(failed.s()), state_digest(twin.s()), "it goes on like the twin at {t}");
    }
}

#[test]
fn exact_mode_random_hosts_leave_no_plan_held() {
    // FX1v's fuzz: a cell, an LED and 220 ohm, a switch to a further 100 ohm;
    // random advances (same-instant and 1 ps ones among them) and switch
    // toggles. A held-back plan must never outlive its call (`end_window`'s
    // debug assertion), and the rail keeps falling once the host stops
    // toggling. FX2 adds the board again with an RC node and two 74LS00s,
    // one powered from the rail reading the RC, one from the RC reading the
    // rail, so plans held behind the cell's run both ways.
    let devices = || vec![generic_battery(0, 1, 0, 5.0, 0.9, 1e-4), led(1, 1, 2), resistor(2, 2, 0, 220.0), switch(3, 1, 3, false), resistor(4, 3, 0, 100.0)];
    let mut crossed = devices();
    crossed.extend([
        source(5, 4, 0, 5.0),
        resistor(6, 4, 5, 1e3),
        capacitor(7, 5, 0, 1e-6, 0.0),
        ls00(8, 1, 0, [5, 1, 6, 1, 1, 7, 1, 1, 8, 1, 1, 9], 10e-9, 0),
        ls00(9, 5, 0, [1, 5, 10, 5, 5, 11, 5, 5, 12, 5, 5, 13], 10e-9, 4),
    ]);
    let boards = [exact(program(4, devices())), exact(program(14, crossed))];
    let mut seed: u64 = 0x9E3779B97F4A7C15;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    // The IC board's rounds are some six times the cost: ten of them.
    for (round, p) in (0..40).map(|r| (r, &boards[0])).chain((0..10).map(|r| (r, &boards[1]))) {
        let mut e = Engine::load(p).unwrap();
        let mut t = 0.0;
        let mut closed = false;
        for _ in 0..300 {
            let r = rnd();
            if r < 0.3 {
                closed = !closed;
                e.set_params(&[(3, 0, if closed { 1.0 } else { 0.0 })]).unwrap();
            } else if r < 0.4 {
                e.advance(t).unwrap();
            } else if r < 0.5 {
                t += 1e-12;
                e.advance(t).unwrap();
            } else {
                t += rnd() * if r < 0.8 { 3e-3 } else { 5e-2 };
                e.advance(t).unwrap();
            }
        }
        let v0 = e.voltages()[1];
        e.advance(t + 0.3).unwrap();
        let v1 = e.voltages()[1];
        e.advance(t + 0.6).unwrap();
        let v2 = e.voltages()[1];
        assert!(v0 > v1 && v1 > v2, "round {round}: the rail must keep falling: {v0} {v1} {v2}");
    }
}

#[test]
fn exact_mode_steps_a_battery_partition_with_nothing_else_to_step() {
    // FX1 (Rails point 9, the owner's rule of 2026-10-15): a {5 V, 0.9 ohm,
    // 0.1 mAh} generic cell into an LED and 220 ohm, no capacitor. About 14
    // mA drains some 4 % a second and the OCV falls with SoC, so the rail
    // falls by tenths of a volt in 1 s. In exact mode the merged partition
    // is stepped, E and r held from the SoC at each step's start. The
    // reference is classic's cadence by hand at classic's step in F3b2's
    // probe, 1 ms (`led_cell_reference`): independent of both rail paths.
    // The host advances every 1 ms, and then as F3b2's TS test does, to 0.1
    // s and 1 s only, where the core steps at its own controller (up to
    // H_MAX, 10 ms). The tolerances are W0's (100 uV + 1e-3 relative, 1 nA
    // + 1e-3 relative) and the harness's battery-drain one, (1 nA + 1e-3
    // |I|avg) T. With `r = 0` the rail row is `u = E(soc)`, which moves
    // with SoC the same way, so it is stepped too.
    let every_ms: Vec<f64> = (1..=1000).map(|k| k as f64 * 1e-3).collect();
    for r in [0.9, 0.0] {
        let cell = generic_battery(0, 1, 0, 5.0, r, 1e-4);
        let reference = led_cell_reference(&cell, 1e-3);
        for (host, instants) in [("1 ms", &every_ms[..]), ("jumps", &[0.1, 1.0][..])] {
            let run = led_cell_run(&exact(led_cell(cell.clone())), instants);
            for (k, t) in [0.1, 1.0].into_iter().enumerate() {
                let ([u, x, i, q], [ru, rx, ri, rq]) = (run[k], reference[k]);
                eprintln!("FX1 r {r} host {host} t {t}: rail {u:.9} ref {ru:.9} ({:.3} tol); cathode {x:.9} ref {rx:.9} ({:.3} tol); draw {i:.9e} ref {ri:.9e}; discharged {q:.9e} ref {rq:.9e}", (u - ru) / (1e-4 + 1e-3 * ru), (x - rx) / (1e-4 + 1e-3 * rx));
                let what = |name: &str| format!("r {r}, host {host}: {name} at {t} s");
                close(u, ru, 1e-4 + 1e-3 * ru.abs(), &what("the rail"));
                close(x, rx, 1e-4 + 1e-3 * rx.abs(), &what("the LED cathode"));
                close(i, ri, 1e-9 + 1e-3 * ri.abs(), &what("the draw"));
                close(q, rq, (1e-9 + 1e-3 * rq / t) * t, &what("the discharged charge"));
            }
        }
        // Not vacuous: the rail fell by tenths of a volt.
        assert!(reference[0][0] - reference[1][0] > 0.1, "the rail falls: {reference:?}");
    }
}

/// V4X's board (Rails point 9): the cell on the rail powers a 74LS00 whose
/// 1y = !(1a 1b), 1b on the rail and 1a a 100 Hz clock on its own section
/// (node 5), into 330 ohm to ground. Each clock edge commits 1y's driver 10
/// ns later inside the cell's partition. With `clk` the 1a input is held at
/// that level instead, for the reference's static solve.
fn nand_load(cell: Device, clk: Option<f64>) -> Vec<Device> {
    let input = match clk {
        Some(v) => source(3, 5, 0, v),
        None => clock(3, 5, 0, 100.0, 0.5, 4),
    };
    vec![cell, ls00(1, 1, 0, [5, 1, 2, 1, 1, 6, 1, 1, 7, 1, 1, 8], 10e-9, 0), resistor(2, 2, 0, 330.0), input]
}

/// V4X's first board: an LED from the rail and 220 ohm into the clock's
/// node 3, so each edge moves the cell's partition's boundary (a source
/// edge, caught up in a call of its own).
fn clocked_led(cell: Device, clk: Option<f64>) -> Vec<Device> {
    let input = match clk {
        Some(v) => source(3, 3, 0, v),
        None => clock(3, 3, 0, 100.0, 0.5, 0),
    };
    vec![cell, led(1, 1, 2), resistor(2, 2, 3, 220.0), input]
}

/// The boards' 100 Hz clock: high for the first half of each period.
fn clock_level(t: f64) -> f64 {
    if (t * 100.0).fract() < 0.5 {
        5.0
    } else {
        0.0
    }
}

/// One step of the hand reference over `h`: E and r from the SoC at the
/// step's start, `board`'s static solve with the cell as E behind r (on
/// node 9) and its clock held at `clk`, then SoC charged with that solve's
/// draw over `h`. The draw.
fn clocked_board_step(board: fn(Device, Option<f64>) -> Vec<Device>, model: &crate::devices::battery::BatteryModel, state: &mut crate::devices::battery::BatteryState, h: f64, clk: f64) -> f64 {
    let (e, r) = (model.emf(state.soc), model.resistance(state.soc));
    let mut devices = board(source(0, 1, 0, e), Some(clk));
    if r > 0.0 {
        devices[0] = source(0, 9, 0, e);
        devices.push(resistor(4, 9, 1, r));
    }
    let solve = Engine::load(&program(10, devices)).unwrap();
    let draw = -solve.currents()[0];
    *state = model.advance(state, draw * h).state;
    draw
}

#[test]
fn exact_mode_bills_each_battery_step_at_its_own_current_across_an_event() {
    // V4X (Rails point 9: SoC advances per committed step with the committed
    // current). On the 74LS00 board a driver commit inside the cell's
    // partition makes one resolve both catch the partition up to the event
    // with the drivers in force before it and re-solve it with the ones
    // after; the caught-up step must be charged at its own current, not at
    // the re-solve's (which charged 0.43 too much with a jumping host). The
    // LED board's edges move the partition's boundary instead. The
    // reference charges each of the cell's own committed steps (the
    // controller's sequence, from the commit log) at the static solve of
    // that step, the clock read at the step's middle less the board's delay.
    // Every host must land on its own reference, and on the 10 us host's
    // within a tenth of the harness's battery-drain tolerance, (1 nA + 1e-3
    // |I|avg) T.
    let reads = [0.0975, 0.1025, 0.2975];
    let boards: [(&str, fn(Device, Option<f64>) -> Vec<Device>, usize, f64); 2] = [("74LS00", nand_load, 5, 10e-9), ("LED", clocked_led, 3, 0.0)];
    for (name, board, clock_node, delay) in boards {
        for r in [0.9, 0.0] {
            let cell = generic_battery(0, 1, 0, 5.0, r, 1e-4);
            let model = crate::devices::battery::BatteryModel::from_record(&cell.params, &cell.blob);
            let p = exact(program(9, board(cell.clone(), None)));
            let mut by_host = Vec::new();
            for (host, frame) in [("10 us", 1e-5), ("1 ms", 1e-3), ("jumps", 0.0)] {
                let mut rig = Rig::load(&p).unwrap();
                let mut got = Vec::new();
                let mut k = 0u64;
                for &tr in &reads {
                    while frame > 0.0 && (k + 1) as f64 * frame < tr - 1e-12 {
                        k += 1;
                        rig.advance(k as f64 * frame).unwrap();
                    }
                    rig.advance(tr).unwrap();
                    assert_eq!(rig.v(clock_node), clock_level(tr), "{name}: the clock's phase at {tr}");
                    got.push(rig.state(0)[4]);
                }
                let cell_p = rig.s().exact.as_ref().unwrap().partition;
                let steps: Vec<f64> = rig.s().stepped.commit_log.iter().filter(|&&(q, _)| q == cell_p).map(|&(_, t)| t).collect();
                let mut state = crate::devices::battery::BatteryState::initial(&model);
                let (mut t0, mut j) = (0.0, 0);
                let mut want = Vec::new();
                let mut charge = 0.0;
                for &tr in &reads {
                    while j < steps.len() && steps[j] <= tr + 1e-15 {
                        let mid = 0.5 * (t0 + steps[j]);
                        let draw = clocked_board_step(board, &model, &mut state, steps[j] - t0, clock_level(mid - delay));
                        charge += draw * (steps[j] - t0);
                        t0 = steps[j];
                        j += 1;
                    }
                    assert!((t0 - tr).abs() < 1e-12, "{name}: a step ends at each reading: {t0} for {tr}");
                    want.push((state.discharged, charge / tr));
                }
                for (j, &tr) in reads.iter().enumerate() {
                    let (q, (rq, iavg)) = (got[j], want[j]);
                    let tol = (1e-9 + 1e-3 * iavg) * tr;
                    eprintln!("FX3 {name} r {r} host {host} t {tr} ({} steps): discharged {q:.9e} ref {rq:.9e}, off {:.3e} = {:.4} tol", steps.len(), q - rq, (q - rq) / tol);
                }
                by_host.push((host, got, want));
            }
            for (host, got, want) in &by_host {
                for (j, &tr) in reads.iter().enumerate() {
                    let (rq, iavg) = want[j];
                    // The same arithmetic as the core's, to the precision of
                    // a step's Newton solves, which stop unpolished
                    // (`newton::NEWTON_ABS` + `NEWTON_REL` of the node) where
                    // the reference's static solve polishes. Each solved half
                    // step is billed (ruling R5), the first half's solve
                    // starting from the last commit: up to 9 parts in 1e6
                    // of the draw on the long steps of the 74LS00 board.
                    let what = |against: &str| format!("{name}, r {r}, host {host}: the discharged charge at {tr} s against {against}");
                    close(got[j], rq, 2e-5 * rq, &what("its own steps"));
                    close(got[j], by_host[0].1[j], 0.1 * (1e-9 + 1e-3 * iavg) * tr, &what("the 10 us host"));
                }
            }
        }
    }
}

/// The current exact mode's source supplies at a solution `(x, caps)` of the
/// rail's partition solved with `stamp` (Rails point 9: "the Norton branch
/// current, or the replaced row's KCL residual"): the KCL of the rail node
/// over every other device of `p` (a resistor at the solved voltages, a
/// capacitor at its solved series current, the node's shunt), and with a
/// Norton its branch current `inject - g u` as well.
fn source_current_at(rig: &Rig, p: &Program, x: &[f64], caps: &[(usize, f64)], stamp: &ExactStamp) -> (f64, Option<f64>) {
    let e = rig.s().exact.as_ref().unwrap();
    let v = |node: usize| match rig.s().local_index[node] {
        Some((q, l)) if q == e.partition => x[l],
        _ => rig.v(node),
    };
    let u = v(e.node);
    let mut leaving = static_part::NODE_RSHUNT_G * u;
    for (d, dev) in p.devices.iter().enumerate() {
        let (a, b) = (dev.pins[0].node, dev.pins[1].node);
        let sign = match (d == e.device, a == e.node, b == e.node) {
            (false, true, _) => 1.0,
            (false, _, true) => -1.0,
            _ => continue,
        };
        leaving += sign
            * match dev.kind {
                kinds::RESISTOR => (v(a) - v(b)) / dev.params[0],
                kinds::CAPACITOR => caps.iter().find(|&&(i, _)| i == d).expect("the rail's capacitors are in its partition").1,
                k => panic!("no rule for kind {k} on the rail"),
            };
    }
    let norton = match stamp.kind {
        ExactKind::Norton { g, inject } => Some(inject - g * u),
        _ => None,
    };
    (leaving, norton)
}

/// A solved sub-step, `(start, end, x, caps)`.
type SubStep<'a> = (f64, f64, &'a [f64], &'a [(usize, f64)]);

/// Committed step `c`'s solved sub-steps: the accepted pair's two half
/// steps, or the one step.
fn sub_steps(c: &stepped::ExactCommit) -> Vec<SubStep<'_>> {
    match &c.mid {
        Some((tm, xm, cm)) => vec![(c.t0, *tm, &xm[..], &cm[..]), (*tm, c.t, &c.x[..], &c.caps[..])],
        None => vec![(c.t0, c.t, &c.x[..], &c.caps[..])],
    }
}

/// What [`assert_each_step_charged_at_its_committed_current`] counted.
struct BillCheck {
    steps: usize,
    /// Calls that charged more than one step.
    multi: usize,
    /// Steps charged as a step-doubling pair.
    pairs: usize,
    /// Steps that charged the cell (a negative charge).
    charging: usize,
    /// The charge the source delivered under two other rules, for
    /// comparison: each committed step at its end current times its length,
    /// and each solved sub-step at its end current times its length.
    end_rule: f64,
    end_per_sub_step: f64,
}

/// Checks every battery step exact mode charged on `rig` (Rails point 9,
/// ruling R5): one charge per committed step of the rail's partition, at
/// that step's end, equal to the sum over the step's solved sub-steps of
/// the source's current ([`source_current_at`]) in the integrator's own
/// quadrature: under backward Euler the sub-step's end current times its
/// length, under the trapezoid the mean of its start and end currents, the
/// start being the last sub-step's end (a trapezoidal step never follows a
/// re-solve); and the cell's state at `at`, the last step's end, the
/// model's from those charges, step by step.
fn assert_each_step_charged_at_its_committed_current(rig: &Rig, p: &Program, at: f64, what: &str) -> BillCheck {
    let e = rig.s().exact.as_ref().unwrap();
    let commits = &rig.s().stepped.exact_commit_log;
    let bills: Vec<(f64, f64)> = rig.s().bill_log.iter().flatten().copied().collect();
    let cell = &p.devices[e.device];
    let model = crate::devices::battery::BatteryModel::from_record(&cell.params, &cell.blob);
    let mut state = crate::devices::battery::BatteryState::initial(&model);
    let mut t0 = p.options.start_time;
    let mut end_rule = 0.0;
    let mut end_per_sub_step = 0.0;
    let mut last: Option<f64> = None;
    assert_eq!(bills.len(), commits.len(), "{what}: one charge per committed step");
    for (k, (c, &(t, bill))) in commits.iter().zip(&bills).enumerate() {
        assert_eq!(t, c.t, "{what}: charge {k} ends at its step's end");
        assert_eq!(c.t0, t0, "{what}: step {k} starts where the last ended");
        let mut want = 0.0;
        let mut scale = 0.0;
        for (s0, s1, x, caps) in sub_steps(c) {
            let (kcl, norton) = source_current_at(rig, p, x, caps, &c.stamp);
            if let Some(norton) = norton {
                close(kcl, norton, 1e-9 * kcl.abs() + 1e-12, &format!("{what}: step {k}, sub-step to {s1}, the rail's KCL against the Norton branch current"));
            }
            let i = norton.unwrap_or(kcl);
            want += match (c.trap, last) {
                (true, Some(i0)) => 0.5 * (i0 + i) * (s1 - s0),
                (true, None) => panic!("{what}: a trapezoidal step {k} first"),
                (false, _) => i * (s1 - s0),
            };
            last = Some(i);
            end_per_sub_step += i * (s1 - s0);
            scale += i.abs() * (s1 - s0);
            if s1 == c.t {
                end_rule += i * (c.t - c.t0);
            }
        }
        close(bill, want, 1e-9 * scale + 1e-12 * (c.t - c.t0), &format!("{what}: step {k} at {t}, the charge"));
        state = model.advance(&state, bill).state;
        t0 = t;
    }
    assert_eq!(t0, at, "{what}: the last step ends at the reading");
    let mut st = Vec::new();
    rig.s().device_state(e.device, at, &mut st).expect("the cell's state");
    close(st[4], state.discharged, 1e-12 * state.discharged.abs() + 1e-18, &format!("{what}: the discharged charge"));
    close(st[5], state.rejected, 1e-12 * state.rejected.abs() + 1e-18, &format!("{what}: the rejected charge"));
    BillCheck {
        steps: commits.len(),
        multi: rig.s().bill_log.iter().filter(|call| call.len() > 1).count(),
        pairs: commits.iter().filter(|c| c.mid.is_some()).count(),
        charging: bills.iter().filter(|b| b.1 < 0.0).count(),
        end_rule,
        end_per_sub_step,
    }
}

/// A cell on the rail (node 1) loading a rail capacitor `c` to ground, 1 k,
/// and a series `c` from the rail into 100 ohm (node 2), which switch 5
/// (open) puts 20 ohm across (node 3).
fn rail_cap_load(cell: Device, c: f64) -> Program {
    program(4, vec![cell, capacitor(1, 1, 0, c, 0.0), resistor(2, 1, 0, 1e3), capacitor(3, 1, 2, c, 0.0), resistor(4, 2, 0, 100.0), switch(5, 2, 3, false), resistor(6, 3, 0, 20.0)])
}

/// `p` loaded and run on a host of `frame` (0: a call straight to each
/// instant) to 1 ms, switch `sw` closed there as one event, and on to 3 ms.
fn run_closing_at_1ms(p: &Program, frame: f64, sw: usize) -> Rig {
    let mut rig = Rig::load(p).unwrap();
    let to = |rig: &mut Rig, from: f64, end: f64| {
        let mut k = 1;
        while frame > 0.0 && from + k as f64 * frame < end - 1e-12 {
            rig.advance(from + k as f64 * frame).unwrap();
            k += 1;
        }
        rig.advance(end).unwrap();
    };
    to(&mut rig, 0.0, 1e-3);
    rig.k.apply_params_as_event(&[(sw, 0, 1.0)]).unwrap();
    rig.k.solver_mut().prepare_readout();
    to(&mut rig, 1e-3, 3e-3);
    rig
}

#[test]
fn exact_mode_bills_each_battery_step_at_its_committed_current_with_a_rail_capacitor() {
    // FX3v (Rails point 9, ruling R5): a step's bill is the source's charge
    // over the sub-steps the solver took, each at the source's current at
    // its own solution with the capacitor companions it was solved with.
    // The committed step is the second half of the step-doubling pair, so a
    // capacitor touching the rail node carries the half step's companion
    // current; billing the full step's companion at that solution charged a
    // 5 V, 0.9 ohm cell with a 10 uF rail capacitor 18.6 mA on an inrush
    // step that published 15.6 mA. The rail capacitor is the Norton's case;
    // the series capacitor's current enters the rail row, so it is the
    // replaced row's case (r = 0) too. Every host, at 10 uF and 100 nF.
    for c in [1e-5, 1e-7] {
        for r in [0.9, 0.0] {
            let p = exact(rail_cap_load(generic_battery(0, 1, 0, 5.0, r, 1e-4), c));
            for (host, frame) in [("10 us", 1e-5), ("1 ms", 1e-3), ("jumps", 0.0)] {
                let rig = run_closing_at_1ms(&p, frame, 5);
                let what = format!("C {c:e}, r {r}, host {host}");
                let check = assert_each_step_charged_at_its_committed_current(&rig, &p, rig.k.time(), &what);
                assert!(check.pairs * 2 > check.steps, "{what}: not vacuous, most steps are pairs ({} of {})", check.pairs, check.steps);
            }
        }
    }
}

/// [`rail_cap_load`] with a 9 V source (node 4) through 10 ohm into the rail
/// as well, which pushes current back into the cell: it charges.
fn charged_rail_cap_load(cell: Device, c: f64) -> Program {
    let mut p = rail_cap_load(cell, c);
    p.node_count = 5;
    p.devices.extend([source(7, 4, 0, 9.0), resistor(8, 4, 1, 10.0)]);
    p
}

#[test]
fn exact_mode_bills_a_charging_cell_its_negative_current() {
    // FX3v (Rails point 9): a cell the board drives current back into draws
    // a negative current, and each step is billed that, as the model's
    // `rejected` charge; a bill clamped at 0 lost it. 9 V through 10 ohm
    // against 5 V puts about 0.4 A into the cell, Norton and replaced row,
    // with the capacitors' inrush and the switch's event on top.
    for c in [1e-5, 1e-7] {
        for r in [0.9, 0.0] {
            let p = exact(charged_rail_cap_load(generic_battery(0, 1, 0, 5.0, r, 1e-4), c));
            for (host, frame) in [("10 us", 1e-5), ("jumps", 0.0)] {
                let rig = run_closing_at_1ms(&p, frame, 5);
                let what = format!("charging, C {c:e}, r {r}, host {host}");
                let check = assert_each_step_charged_at_its_committed_current(&rig, &p, rig.k.time(), &what);
                assert!(check.charging * 2 > check.steps, "{what}: not vacuous, most steps charge the cell ({} of {})", check.charging, check.steps);
                assert!(rig.state(0)[5] > 0.0, "{what}: the cell refuses the charge as rejected");
            }
        }
    }
}

#[test]
fn exact_mode_bills_every_step_of_a_call_that_commits_several() {
    // FX3v (Rails point 9: SoC advances per committed step): the horizon
    // clamp brings the rail's partition up to the instant in as many steps
    // as its controller takes, all in one call, and that call must charge
    // each of them at its own current, in order. The kernel's own event
    // loop commits one step per call, so the clamp is called directly, from
    // load to 3 ms; the steps' currents fall as the capacitors charge, so
    // charging the call at any one of them is wrong. A discharging and a
    // charging cell, Norton and replaced row.
    for (name, board) in [("discharging", rail_cap_load as fn(Device, f64) -> Program), ("charging", charged_rail_cap_load)] {
        for c in [1e-5, 1e-7] {
            for r in [0.9, 0.0] {
                let p = exact(board(generic_battery(0, 1, 0, 5.0, r, 1e-4), c));
                let mut rig = Rig::load(&p).unwrap();
                rig.k.solver_mut().sync_dynamic_to(3e-3).unwrap();
                let what = format!("{name}, C {c:e}, r {r}, one clamp to 3 ms");
                let check = assert_each_step_charged_at_its_committed_current(&rig, &p, 3e-3, &what);
                let longest = rig.s().bill_log.iter().map(Vec::len).max().unwrap_or(0);
                assert!(check.multi > 0 && longest > 10, "{what}: not vacuous, one call charges many steps ({} steps, longest call {longest})", check.steps);
            }
        }
    }
}

/// A fast cell (2.5e-5 Ah, 90 mC) on the rail (node 1) into a rail
/// capacitor `c` and 100 ohm: up to 50 mA, so it gives up over a quarter of
/// its charge in 0.6 s and E and r move at every step.
fn draining_rail_cap(r: f64, c: f64, integration: u8) -> Program {
    let mut p = exact(program(2, vec![generic_battery(0, 1, 0, 5.0, r, 2.5e-5), capacitor(1, 1, 0, c, 0.0), resistor(2, 1, 0, 100.0)]));
    p.options.integration = integration;
    p
}

/// One committed step of [`draining_rail_cap`] by hand.
#[derive(Debug, Clone, Copy)]
struct DrainStep {
    t: f64,
    /// The source's stamp: E and r (r = 0 a replaced row).
    e: f64,
    r: f64,
    /// The rail and the cell's current at the step's end.
    u: f64,
    draw: f64,
    state: crate::devices::battery::BatteryState,
}

/// Classic's cadence by hand (Rails point 9) on [`draining_rail_cap`] under
/// backward Euler, at the run's own sub-steps `commits` (each committed
/// step's start, its accepted pair's midpoint if any, its end): each
/// committed step takes E and r from the SoC at its start, solves each
/// sub-step's one node, `(g + 1/R + C/h + shunt) u = g E + C/h vc` behind a
/// Norton `g = 1/r` or `u = E` for a replaced row, and charges SoC with its
/// sub-steps' end currents times their lengths.
fn draining_rail_cap_reference(p: &Program, commits: &[stepped::ExactCommit]) -> Vec<DrainStep> {
    let cell = &p.devices[0];
    let (c, load) = (p.devices[1].params[0], p.devices[2].params[0]);
    let model = crate::devices::battery::BatteryModel::from_record(&cell.params, &cell.blob);
    let mut state = crate::devices::battery::BatteryState::initial(&model);
    let mut vc = 0.0;
    let mut out = Vec::new();
    for step in commits {
        let (e, r) = (model.emf(state.soc), model.resistance(state.soc));
        let ends: Vec<f64> = step.mid.iter().map(|m| m.0).chain([step.t]).collect();
        let (mut s0, mut q, mut u, mut draw) = (step.t0, 0.0, 0.0, 0.0);
        for s1 in ends {
            let gc = c / (s1 - s0);
            if r > 0.0 {
                u = (e / r + gc * vc) / (1.0 / r + 1.0 / load + gc + static_part::NODE_RSHUNT_G);
                draw = (e - u) / r;
            } else {
                u = e;
                draw = gc * (u - vc) + u / load + static_part::NODE_RSHUNT_G * u;
            }
            q += draw * (s1 - s0);
            vc = u;
            s0 = s1;
        }
        state = model.advance(&state, q).state;
        out.push(DrainStep { t: step.t, e, r, u, draw, state });
    }
    out
}

/// The rail, the cell's draw, its SoC and its discharged charge.
fn drain_readings(rig: &Rig) -> [f64; 4] {
    let st = rig.state(0);
    [rig.v(1), -rig.i(0), st[0], st[4]]
}

#[test]
fn exact_mode_takes_e_and_r_per_step_inside_a_call_that_commits_several() {
    // FX5 (Rails point 9: E and r "per step from the SoC at the step's
    // start"): the horizon clamp brings the rail's partition from load to
    // 0.6 s in one call, many steps; the kernel's event loop takes the same
    // span one step per call. Both take each step's stamp from the SoC its
    // earlier steps left, so their steps, rail, draw and SoC agree to
    // rounding, and both match the hand-stepped reference at their own
    // steps. Stamping every step of the call from the SoC at its first
    // step's start held E at the full cell's: the rail read 4.955 V at 0.6
    // s against 3.567 V (10 uF, r 0.9, backward Euler). Norton and
    // replaced row, 10 uF and 100 nF, backward Euler and the trapezoid (the
    // reference is backward Euler's).
    let tend = 0.6;
    for integration in [0u8, 1] {
        for c in [1e-5, 1e-7] {
            for r in [0.9, 0.0] {
                let p = draining_rail_cap(r, c, integration);
                let what = format!("integration {integration}, C {c:e}, r {r}");
                let mut one = Rig::load(&p).unwrap();
                one.k.solver_mut().sync_dynamic_to(tend).unwrap();
                one.k.solver_mut().prepare_readout();
                let mut each = Rig::load(&p).unwrap();
                each.advance(tend).unwrap();
                let longest = |rig: &Rig| rig.s().bill_log.iter().map(Vec::len).max().unwrap_or(0);
                assert!(longest(&one) > 10, "{what}: not vacuous, the clamp commits many steps in one call ({})", longest(&one));
                assert_eq!(longest(&each), 1, "{what}: the event loop commits one step per call");
                let (a, b) = (drain_readings(&one), drain_readings(&each));
                eprintln!(
                    "FX5 {what}: {} vs {} steps; one call against one per call: rail {:.3e}, draw {:.3e}, soc {:.3e}, discharged {:.3e}",
                    one.s().stepped.exact_commit_log.len(),
                    each.s().stepped.exact_commit_log.len(),
                    a[0] - b[0],
                    a[1] - b[1],
                    a[2] - b[2],
                    a[3] - b[3]
                );
                for (q, quantity) in ["the rail", "the draw", "the SoC", "the discharged charge"].into_iter().enumerate() {
                    close(a[q], b[q], 1e-12 * b[q].abs(), &format!("{what}: {quantity} at {tend} s, one call against one step per call"));
                }
                // Not vacuous: the cell lost a quarter of its charge.
                assert!(b[2] < 0.75, "{what}: the cell drains: SoC {}", b[2]);
                for (name, rig) in [("one call", &one), ("one step per call", &each)] {
                    let commits = &rig.s().stepped.exact_commit_log;
                    if integration == 0 {
                        let reference = draining_rail_cap_reference(&p, commits);
                        for (k, (c, want)) in commits.iter().zip(&reference).enumerate() {
                            let at = format!("{what}, {name}: step {k} to {}", want.t);
                            match c.stamp.kind {
                                ExactKind::Norton { g, inject } => {
                                    close(g, 1.0 / want.r, 1e-12 / want.r, &format!("{at}, the Norton conductance"));
                                    close(inject, want.e / want.r, 1e-12 * want.e / want.r, &format!("{at}, the Norton current"));
                                }
                                ExactKind::Row { value } => close(value, want.e, 1e-12 * want.e, &format!("{at}, the row's E")),
                                ExactKind::Inject { .. } => panic!("{at}: a cell is never a current source"),
                            }
                            close(c.x[rig.s().local_index[1].unwrap().1], want.u, 1e-11 * want.u, &format!("{at}, the rail"));
                        }
                        let last = reference.last().unwrap();
                        assert_eq!(last.t, tend, "{what}, {name}: the last step ends at the reading");
                        let r = drain_readings(rig);
                        close(r[0], last.u, 1e-11 * last.u, &format!("{what}, {name}: the rail at {tend} s against the reference"));
                        close(r[1], last.draw, 1e-9 * last.draw, &format!("{what}, {name}: the draw at {tend} s against the reference"));
                        close(r[2], last.state.soc, 1e-11 * last.state.soc, &format!("{what}, {name}: the SoC at {tend} s against the reference"));
                        close(r[3], last.state.discharged, 1e-11 * last.state.discharged, &format!("{what}, {name}: the discharged charge at {tend} s against the reference"));
                    }
                }
            }
        }
    }
}

#[test]
fn a_rollback_after_a_catch_up_that_restamped_puts_the_stamp_back() {
    // FX5 (Rails points 8 and 9): a catch-up inside an instant (W1b rule 6)
    // commits many of the cell's steps, each restamping from the SoC it
    // leaves, and the instant then fails. The stamp goes back to the SoC
    // the rollback restores, and the solver goes on as the twin that never
    // tried, through a later call that commits several steps as well.
    let p = draining_rail_cap(0.9, 1e-5, 0);
    let (mut failed, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
    failed.advance(0.05).unwrap();
    twin.advance(0.05).unwrap();
    let before = state_digest(failed.s());
    {
        let s = failed.k.solver_mut();
        let cell_p = s.exact.as_ref().unwrap().partition;
        let stamp = s.exact.as_ref().unwrap().stamp();
        assert_eq!(s.stepped.exact_stamp(), Some(stamp), "the call starts at the owner's stamp");
        let checkpoint = s.begin_instant();
        let drivers: Vec<Driver> = s.driver_snapshot.to_vec();
        s.begin_dynamic_writes_list(&[cell_p]);
        s.stepped.resolve_dirty(&[cell_p], 0.2, &drivers, &drivers, &s.live_params, &mut s.voltages, &mut s.currents).unwrap();
        s.end_dynamic_writes();
        let charged = s.stepped.take_bills().count();
        assert!(charged > 5, "not vacuous: the catch-up commits several steps ({charged})");
        assert_ne!(s.stepped.exact_stamp(), Some(stamp), "each step restamped");
        s.rollback_instant(checkpoint);
        assert_eq!(s.stepped.exact_stamp(), Some(stamp), "the stamp is the restored SoC's");
    }
    assert_eq!(state_digest(failed.s()), before, "every piece of state");
    for t in [0.1, 0.3] {
        failed.k.solver_mut().sync_dynamic_to(t).unwrap();
        twin.k.solver_mut().sync_dynamic_to(t).unwrap();
        assert_eq!(state_digest(failed.s()), state_digest(twin.s()), "it goes on like the twin to {t}");
    }
    assert!(failed.s().bill_log.iter().any(|call| call.len() > 5), "a later call commits several steps");
}

#[test]
fn exact_mode_bills_conserve_charge_with_the_integrator() {
    // Ruling D5 (Rails point 9): the bill conserves charge with the
    // integrator. A cell into capacitors alone, a rail capacitor `c` and 100
    // ohm into a second `c` (node 2), delivers exactly the charge the
    // capacitors hold, `c u + c v2`, plus what the nodes' 1e-12 S shunts
    // leak (each sub-step's end voltages times its length), under backward
    // Euler and the trapezoid. Each sub-step's end current times its length
    // was off by up to 7.3e-2 under the trapezoid, and the end current times
    // the whole committed step by up to 1.9e-2 under backward Euler.
    for integration in [0u8, 1] {
        for c in [1e-5, 1e-7] {
            for r in [0.9, 0.0] {
                let mut p = exact(program(3, vec![battery(0, 1, 0, 5.0, r), capacitor(1, 1, 0, c, 0.0), resistor(2, 1, 2, 100.0), capacitor(3, 2, 0, c, 0.0)]));
                p.options.integration = integration;
                for host in ["10 us", "jumps", "one clamp"] {
                    let mut rig = Rig::load(&p).unwrap();
                    let tend = 3e-3;
                    match host {
                        "10 us" => (1..=300).for_each(|k| rig.advance(k as f64 * 1e-5).unwrap()),
                        "jumps" => [1e-4, 1e-3, tend].iter().for_each(|&t| rig.advance(t).unwrap()),
                        _ => {
                            rig.k.solver_mut().sync_dynamic_to(tend).unwrap();
                        }
                    }
                    let what = format!("integration {integration}, C {c:e}, r {r}, host {host}");
                    let check = assert_each_step_charged_at_its_committed_current(&rig, &p, tend, &what);
                    let local = |node: usize| rig.s().local_index[node].unwrap().1;
                    let shunt: f64 = rig
                        .s()
                        .stepped
                        .exact_commit_log
                        .iter()
                        .flat_map(sub_steps)
                        .map(|(s0, s1, x, _)| static_part::NODE_RSHUNT_G * (x[local(1)] + x[local(2)]) * (s1 - s0))
                        .sum();
                    let held = c * rig.v(1) + c * rig.v(2) + shunt;
                    let q = rig.state(0)[4];
                    eprintln!(
                        "FX4 {what}: drawn against held {:.3e}; end current x sub-step {:.3e}; end current x step {:.3e}",
                        (q - held) / held,
                        (check.end_per_sub_step - held) / held,
                        (check.end_rule - held) / held
                    );
                    assert!(check.pairs * 2 > check.steps, "{what}: not vacuous, most steps are pairs ({} of {})", check.pairs, check.steps);
                    close(q, held, 1e-10 * held, &format!("{what}: the charge drawn against the charge held"));
                }
            }
        }
    }
}


// ---- Rails point 9: exact mode's start-up with a rail capacitor ------------------

/// The instants of the first millisecond the start-up tests read.
const START_UP: [f64; 6] = [2.5e-6, 1e-5, 1e-4, 2.5e-4, 5e-4, 1e-3];

#[test]
fn exact_mode_starts_a_discharged_rail_capacitor_in_cc_and_ramps_it_at_the_limit() {
    // Probe P9's circuit, {5 V, 0.1 A} with 100 uF straight across, in exact
    // mode. The capacitor loads at 0 V, so its draw at the setpoint is
    // unbounded at esr 0 and 100 A at esr 0.05 (Rails point 5), and the
    // regime rules run at load as at every rail solve: the PSU is in CC from
    // load, as in default mode and fixed classic (whose 1 ps seed sees the
    // charging current). The rail is then the capacitor charging at the
    // limit, the P9 ODE of the default-mode test above, which backward Euler
    // integrates to rounding (a constant current but for the 1e-12 S shunt).
    // The load's own solve is classic's 1 ps seed, so the rail there is
    // iLimit 1e-12 / C above the ODE. Exact mode used to start in CV: its
    // first step charged the capacitor to the setpoint in one step, so at esr
    // 0 the rail read 5 V from 2.5 us on and never entered CC, and at esr
    // 0.05 it entered CC one step late, 0.01 V ahead of the ramp.
    for (v, esr) in [(5.0, 0.0), (5.0, 0.05), (-5.0, 0.0), (-5.0, 0.05)] {
        let what = format!("V {v}, esr {esr}");
        let dir: f64 = if v < 0.0 { -1.0 } else { 1.0 };
        let (c, i_limit) = (1e-4, 0.1);
        let g = static_part::NODE_RSHUNT_G;
        let dl = dir * i_limit;
        let alpha = 1.0 / (1.0 + g * esr);
        let k = alpha * g / c;
        let ode = |t: f64| alpha * (dl / g) * -(-k * t).exp_m1() + alpha * esr * dl;
        let devices = || vec![psu(0, 1, 0, v, i_limit), capacitor(1, 1, 0, c, esr)];
        let mut ex = Rig::load(&exact(program(2, devices()))).unwrap();
        let mut def = Rig::load(&program(2, devices())).unwrap();
        close(ex.v(1), ode(0.0), dl.abs() * 1e-12 / c * 1.001, &format!("{what}: the rail at load, the seed's 1 ps of charge above the ODE"));
        assert_eq!(ex.state(0), vec![2.0], "{what}: CC from load");
        assert_eq!(ex.i(0), -dl, "{what}: CC publishes -dir iLimit at load");
        for t in START_UP {
            ex.advance(t).unwrap();
            def.advance(t).unwrap();
            let at = format!("{what} at {t:e}");
            assert_eq!(ex.state(0), vec![2.0], "{at}: CC");
            close(ex.v(1), ode(t), 1e-12, &format!("{at}: the rail on the ODE"));
            close(ex.v(1), def.v(1), 1e-12, &format!("{at}: the rail on default mode's"));
            assert_eq!(ex.i(0), -dl, "{at}: the PSU at the limit");
            close(ex.i(1), dl - g * ode(t), 1e-12, &format!("{at}: the capacitor takes the limit"));
            close(ex.i(1), def.i(1), 1e-12, &format!("{at}: the capacitor's current as default mode's"));
        }
    }
}

/// F1r6's boards: {5 V, 1.873 mA} with 1 uF across the rail and an LED with
/// 330 ohm to ground (node 2).
fn led_board(esr: f64) -> Vec<Device> {
    vec![psu(0, 1, 0, 5.0, 0.001873), capacitor(1, 1, 0, 1e-6, esr), led(2, 1, 2), resistor(3, 2, 0, 330.0)]
}

/// INF1's board: {5 V, 20 mA} with 1 uF across the rail (device 6), a
/// 74LS00 on it whose gate 1 reads node 2, and 100 ohm from the rail into 10
/// uF at node 2; the switch to 50 ohm stays open.
fn inf1_board() -> Vec<Device> {
    vec![
        psu(0, 1, 0, 5.0, 0.02),
        resistor(1, 1, 2, 100.0),
        capacitor(2, 2, 0, 1e-5, 0.0),
        switch(3, 1, 3, false),
        resistor(4, 3, 0, 50.0),
        ls00(5, 1, 0, [2, 2, 5, 0, 0, 6, 0, 0, 7, 0, 0, 8], 10e-9, 0),
        capacitor(6, 1, 0, 1e-6, 0.0),
    ]
}

/// Classic at one instant of [`START_UP`]: the rail, node 2 and the rail
/// capacitor's current.
type ClassicAt = (f64, [f64; 3], [f64; 3]);

/// F1r6's boards in classic (HeadlessRunner, scratch /tmp/wasmport/ex1/cls/):
/// at its minimum fixed step, 1e-8 s (the reference), then adaptive.
const LED_BOARD_CLASSIC: [ClassicAt; 6] = [
    (2.5e-6, [0.004682500, 0.0, 1.873000e-3], [0.004682500, 0.0, 1.873000e-3]),
    (1e-5, [0.018730000, 0.0, 1.873000e-3], [0.018730000, 0.0, 1.873000e-3]),
    (1e-4, [0.187300000, 0.0, 1.873000e-3], [0.187300000, 0.0, 1.873000e-3]),
    (2.5e-4, [0.468250000, 0.0, 1.873000e-3], [0.468250000, 0.0, 1.873000e-3]),
    (5e-4, [0.936499999, 0.000000007, 1.873000e-3], [0.936499998, 0.000000007, 1.873000e-3]),
    (1e-3, [1.859282686, 0.083707300, 1.619342e-3], [1.857447774, 0.082573283, 1.622778e-3]),
];
const LED_BOARD_ESR_CLASSIC: [ClassicAt; 6] = [
    (2.5e-6, [0.004776150, 0.0, 1.873000e-3], [0.004776150, 0.0, 1.873000e-3]),
    (1e-5, [0.018823650, 0.0, 1.873000e-3], [0.018823650, 0.0, 1.873000e-3]),
    (1e-4, [0.187393650, 0.0, 1.873000e-3], [0.187393650, 0.0, 1.873000e-3]),
    (2.5e-4, [0.468343650, 0.0, 1.873000e-3], [0.468343650, 0.0, 1.873000e-3]),
    (5e-4, [0.936593649, 0.000000007, 1.873000e-3], [0.936593648, 0.000000007, 1.873000e-3]),
    (1e-3, [1.859351861, 0.083750167, 1.619212e-3], [1.857516023, 0.082615356, 1.622650e-3]),
];
const INF1_BOARD_CLASSIC: [ClassicAt; 6] = [
    (2.5e-6, [0.049378258, 0.000062174, 1.950684e-2], [0.049292929, 0.000070707, 1.950778e-2]),
    (1e-5, [0.190347843, 0.000965216, 1.810617e-2], [0.189658809, 0.001034119, 1.811375e-2]),
    (1e-4, [1.284477320, 0.071552268, 7.870749e-3], [1.276656159, 0.072334384, 7.956782e-3]),
    (2.5e-4, [2.001756148, 0.299824385, 2.980682e-3], [1.993719929, 0.300628007, 3.069081e-3]),
    (5e-4, [2.555226433, 0.744477357, 1.892509e-3], [2.552187303, 0.744781270, 1.925940e-3]),
    (1e-3, [3.471046757, 1.652895324, 1.818486e-3], [3.470867676, 1.652913232, 1.820456e-3]),
];

#[test]
fn exact_mode_starts_a_rail_capacitor_under_a_load_where_classic_does() {
    // A rail capacitor under a load in CC has no closed form to read, so the
    // reference is classic at its minimum step (default mode lands within
    // 1e-4 V of it). Exact mode steps the board with classic's controller
    // (Rails point 9), so its step error is classic's kind: both run up to
    // 8 mV off the reference on INF1's inrush. Exact mode passes where it is
    // no further from the reference than W0's tolerance (100 uV + 1e-3
    // relative, 1 nA + 1e-3 relative) plus classic's own adaptive error, on
    // the rail, node 2 and the rail capacitor's current at every instant of
    // the first millisecond. Exact mode used to start the capacitor at the
    // setpoint: INF1's board read 3.033 V and 1.197 V at 0.5 ms (reference
    // 2.555 and 0.744), and the capacitor's current had the opposite sign
    // (-7.3e-3 A against 1.873e-3 on F1r6's board at 2.5 us).
    let boards: [(&str, usize, Vec<Device>, usize, &[ClassicAt; 6]); 3] = [
        ("F1r6, esr 0", 3, led_board(0.0), 1, &LED_BOARD_CLASSIC),
        ("F1r6, esr 0.05", 3, led_board(0.05), 1, &LED_BOARD_ESR_CLASSIC),
        ("INF1", 9, inf1_board(), 6, &INF1_BOARD_CLASSIC),
    ];
    for (name, n, devices, cap, classic) in boards {
        let mut ex = Rig::load(&exact(program(n, devices.clone()))).unwrap();
        let mut def = Rig::load(&program(n, devices)).unwrap();
        for &(t, reference, adaptive) in classic {
            ex.advance(t).unwrap();
            def.advance(t).unwrap();
            let at = format!("{name} at {t:e}");
            let got = [ex.v(1), ex.v(2), ex.i(cap)];
            eprintln!("{at}: exact {got:?}, default {:?}, reference {reference:?}", [def.v(1), def.v(2), def.i(cap)]);
            for (k, what) in ["the rail", "node 2", "the rail capacitor's current"].iter().enumerate() {
                let w0 = if k < 2 { 1e-4 } else { 1e-9 } + 1e-3 * reference[k].abs();
                close(got[k], reference[k], w0 + (adaptive[k] - reference[k]).abs(), &format!("{at}: {what}"));
            }
            assert_eq!(ex.state(0), def.state(0), "{at}: the regime");
            assert_eq!(ex.i(0), def.i(0), "{at}: the PSU current");
        }
    }
}

#[test]
fn exact_mode_starts_a_battery_rail_capacitor_discharged_and_charges_it_through_r() {
    // A battery has no regime, so the load's solve changes nothing there: a
    // discharged 1 uF rail capacitor across a 5 V cell with 100 ohm starts at
    // 0 V, as classic's does. With r = 0.9 the rail charges through the
    // cell's resistance, a Thevenin RC toward 5 * 100 / 100.9 V with tau =
    // (0.9 || 100) * 1 uF = 0.89 us, which default mode does not model (Rails
    // point 7: it reads the settled rail from load); with r = 0 the row holds
    // the rail at 5 V from load. From 100 us on the two modes agree within
    // W0's tolerance.
    for r in [0.9, 0.0] {
        let devices = || vec![battery(0, 1, 0, 5.0, r), capacitor(1, 1, 0, 1e-6, 0.0), resistor(2, 1, 0, 100.0)];
        let mut ex = Rig::load(&exact(program(2, devices()))).unwrap();
        let mut def = Rig::load(&program(2, devices())).unwrap();
        if r > 0.0 {
            // The 1 ps seed: E/r into the capacitor's C/1e-12 S.
            close(ex.v(1), 5.0 / r * 1e-12 / 1e-6, 1e-9, "r 0.9: the rail at load, at the discharged capacitor");
        } else {
            assert_eq!(ex.v(1), 5.0, "r 0: the rail at E from load");
        }
        for t in START_UP.into_iter().filter(|&t| t >= 1e-4) {
            ex.advance(t).unwrap();
            def.advance(t).unwrap();
            let at = format!("r {r} at {t:e}");
            close(ex.v(1), def.v(1), 1e-4 + 1e-3 * def.v(1).abs(), &format!("{at}: the rail"));
            close(ex.i(0), def.i(0), 1e-9 + 1e-3 * def.i(0).abs(), &format!("{at}: the cell's current"));
            close(ex.i(1), def.i(1), 1e-9, &format!("{at}: the rail capacitor's current"));
        }
    }
}

#[test]
fn exact_mode_enters_cc_at_a_load_step_that_lands_on_a_committed_step() {
    // The same rule at a later solve: {5 V, 0.1 A} with 100 uF and 100 ohm,
    // settled in CV, and 50 ohm switched in at 10 ms, where the host's
    // advance has just committed the rail's partition. The draw at 5 V is
    // 0.15 A, so the PSU is in CC at the toggle (Rails point 5, as default
    // mode's `a_rail_capacitor_psu_enters_cc_at_the_load_step_and_holds_the_
    // limit`), and the rail follows the capacitor's closed form, u_inf = 0.1
    // * 33.33 V with tau = C / G, within W0's tolerance over the first 100 us
    // (past that, backward Euler's step error at classic's cadence shows).
    // Exact mode used to publish -0.15 A in CV there, and enter CC one step
    // later.
    let devices = || vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0), resistor(2, 1, 0, 100.0), switch(3, 1, 2, false), resistor(4, 2, 0, 50.0)];
    let mut ex = Rig::load(&exact(program(4, devices()))).unwrap();
    let t1 = 10e-3;
    ex.advance(t1).unwrap();
    assert_eq!((ex.state(0), ex.v(1)), (vec![0.0], 5.0), "CV at the setpoint before the toggle");
    let p = ex.s().exact.as_ref().unwrap().partition;
    assert_eq!(ex.s().stepped.committed(p).map(|(t, _)| t), Some(t1), "the advance committed the rail's partition at the toggle's instant");
    toggle(&mut ex, 3, true).unwrap();
    assert_eq!(ex.state(0), vec![2.0], "CC at the load step");
    assert_eq!(ex.i(0), -0.1, "the PSU at the limit at the load step");
    // The seed's 1 ps of discharge.
    close(ex.v(1), 5.0, 0.05 * 1e-12 / 1e-4 * 1.001, "the rail continuous through the load step");
    let g = 1.0 / 100.0 + 1.0 / (50.0 + 0.001) + static_part::NODE_RSHUNT_G;
    let (u_inf, tau) = (0.1 / g, 1e-4 / g);
    for t in [t1 + 1e-6, t1 + 1e-4] {
        ex.advance(t).unwrap();
        let want = u_inf + (5.0 - u_inf) * (-(t - t1) / tau).exp();
        assert_eq!(ex.state(0), vec![2.0], "CC at {t:e}");
        close(ex.v(1), want, 1e-4 + 1e-3 * want, &format!("the rail at {t:e} within W0's tolerance of the closed form"));
    }
}

// ---- Rails point 3: the static cache --------------------------------------------

/// Closes or opens switch `device` as one `set_params` event.
fn toggle(rig: &mut Rig, device: usize, closed: bool) -> Result<(), KernelError> {
    let r = rig.k.apply_params_as_event(&[(device, 0, if closed { 1.0 } else { 0.0 })]);
    rig.k.solver_mut().prepare_readout();
    r
}

#[test]
fn a_power_flip_inside_one_band_never_replays_the_other_power_state() {
    // A gate whose power threshold, 0.9 * 4.5 = 4.05 V, sits inside the band
    // [4.025, 4.075), on a rail that starts at E R/(R + r) = 4.04988 behind a
    // 1 uF RC and rises to E = 4.06 through it. The open-circuit guess E is in
    // the same band and powers the gate; the solved rail does not. Unpowered,
    // the gate stamps nothing (W1c point 1), so its outputs read 0 V. The
    // cell's 0.01 ohm keeps the powered gate's own draw (its outputs' node
    // shunts, 16 pA) from pulling the rail back under the threshold it just
    // crossed: through 5 ohm that is 8e-11 V, more than the rail event lands
    // past it, and the rail loop ping-pongs at the threshold to -4
    // (w1f-power-gate-pingpong), as it did here with the cache off.
    let mut gate = ls00(3, 1, 0, [0, 0, 4, 0, 0, 5, 0, 0, 6, 0, 0, 7], 10e-9, 0);
    gate.params[ic_param::VCC_MIN] = 4.5;
    let (e, r, big_r, c) = (4.06, 0.01, 4.0, 1e-6);
    for cache in [true, false] {
        let mut p = program(8, vec![battery(0, 1, 0, e, r), resistor(1, 1, 2, big_r), capacitor(2, 2, 0, c, 0.0), gate.clone()]);
        p.options.no_solution_cache = !cache;
        let mut rig = Rig::load(&p).unwrap();
        let powered = |rig: &Rig| {
            let ic = &rig.s().ics[0].1;
            digital::power::powered(ic, rig.v(1), rig.v(0))
        };
        let band = |u: f64| (u / rails::RAIL_BUCKET_V).round();
        assert_eq!(band(e), band(rig.u()), "one band: {e} and {}", rig.u());
        assert!(!powered(&rig), "unpowered at load: u {}", rig.u());
        assert!((4..8).all(|n| rig.v(n) < 1e-6), "unpowered outputs read 0 V at load: {:?}", (4..8).map(|n| rig.v(n)).collect::<Vec<_>>());
        rig.advance(10e-3).unwrap();
        assert!(powered(&rig), "cache {cache}: powered once the capacitor charges: u {}", rig.u());
        assert!((4..8).all(|n| rig.v(n) > 3.0), "cache {cache}: powered outputs read high: {:?}", (4..8).map(|n| rig.v(n)).collect::<Vec<_>>());
    }
}

#[test]
fn a_hub_partition_hits_its_cache_within_the_band_and_misses_past_it() {
    // A 1 kHz clock drives a 74LS00 gate whose output, high, sinks about 20
    // mA into 100 ohm from a 2 ohm battery: the rail steps about 39 mV each
    // half period. An LED with 1 kohm from the rail is a nonlinear static
    // hub partition, re-solved past the 25 mV band at every edge (Rails
    // point 3), and keyed by the band round(u / 50 mV).
    let circuit = |e: f64| program(8, vec![battery(0, 1, 0, e, 2.0), clock(1, 2, 0, 1000.0, 0.5, 0), ls00(2, 1, 0, [2, 2, 3, 0, 0, 4, 0, 0, 5, 0, 0, 6], 10e-9, 1), resistor(3, 3, 0, 100.0), led(4, 1, 7), resistor(5, 7, 0, 1000.0)]);
    let band = |u: f64| (u / rails::RAIL_BUCKET_V).round();
    // The LED partition's last solve rail, and the rail, after each edge.
    let walk = |e: f64, cache: bool| -> Vec<(f64, f64, f64)> {
        let mut rig = Rig::load(&circuit(e)).unwrap();
        rig.k.solver_mut().set_no_solution_cache(!cache);
        let p = rig.s().local_index[7].unwrap().0;
        let mut out = Vec::new();
        for k in 0..6 {
            rig.advance(0.25e-3 + k as f64 * 0.5e-3).unwrap();
            let (_, _, _, _, u_p) = rig.s().static_solver.hub_solve(p).unwrap();
            out.push((u_p, rig.u(), rig.v(7)));
        }
        out
    };
    // Two bands: E = 5 puts the rail near 4.994 (band 100) with the gate low
    // and 4.955 (band 99) with it high. The load solved at the guess E; each
    // rail's first visit misses, every later one hits an entry within 25 mV.
    let a = walk(5.0, true);
    eprintln!("cache, two bands: {a:?}");
    assert_ne!(band(a[0].1), band(a[1].1));
    assert_eq!(a[0].0, 5.0, "solved at the open-circuit guess, the rail 6 mV below it");
    // A miss solves at the rail of its pass; the rail then moves by the
    // partition's own new draw, nanovolts.
    close(a[1].0, a[1].1, 1e-6, "the high rail's first visit misses and solves at u");
    assert_eq!(a[2].0, 5.0, "the low rail hits the load's entry, 6 mV away");
    for k in 3..6 {
        assert_eq!(a[k].0, a[k - 2].0, "edge {k} hits");
    }
    // Cache on against off: a hit's replay x_p + s (u - u_p) against a fresh
    // solve at u.
    let off = walk(5.0, false);
    let worst = a.iter().zip(&off).map(|(x, y)| (x.1 - y.1).abs().max((x.2 - y.2).abs())).fold(0.0, f64::max);
    eprintln!("cache on against off: worst {worst:.3e} V");
    assert!(worst <= 1e-6, "cache on against off within the rail-correction bound: {worst:e}");
    // One band: E = 5.076 puts both rails in band 101, 39 mV apart. The
    // entry of the other rail is past 25 mV, so each rail's first visit
    // misses although the key matches; later visits find their own entry.
    let b = walk(5.076, true);
    eprintln!("cache, one band: {b:?}");
    assert_eq!(band(b[1].1), band(b[2].1));
    close(b[1].0, b[1].1, 1e-6, "the high rail's first visit misses");
    close(b[2].0, b[2].1, 1e-6, "the low rail misses the high rail's entry, 39 mV away in its band");
    assert!((b[2].0 - b[1].0).abs() > rails::RAIL_RESOLVE_V);
    for k in 3..6 {
        assert_eq!(b[k].0, b[k - 2].0, "edge {k} hits its own rail's entry");
    }
}

#[test]
fn a_reader_of_a_static_hub_partition_flips_at_the_next_scan_within_the_scan_band() {
    // A {5 V, 10 ohm} battery charges 10 uF through 100 ohm, so the rail rises
    // from 4.545 V toward 5 V (tau' = 1.1 ms). A 1.8k/1k divider off the rail
    // is a static hub partition; a NAND with Schmitt inputs (1.7 V up),
    // supplied apart, reads its midpoint, which reaches 1.7 V near u = 4.76 V.
    // The reader is queued only at a rail scan, each RAIL_SCAN_V of rail
    // (Rails point 3), so it flips at most RAIL_SCAN_V of rail, plus the move
    // that crossed it, late, and stat 20 counts the flip.
    let p = program(
        9,
        vec![
            battery(0, 1, 0, 5.0, 10.0),
            resistor(1, 1, 2, 100.0),
            capacitor(2, 2, 0, 1e-5, 0.0),
            resistor(3, 1, 3, 1800.0),
            resistor(4, 3, 0, 1000.0),
            source(5, 4, 0, 5.0),
            schmitt_nand(6, 4, [3, 3, 5, 0, 0, 6, 0, 0, 7, 0, 0, 8], 0),
        ],
    );
    let mut e = Engine::load(&p).unwrap();
    assert!(e.voltages()[3] < 1.7 && e.digital_slots()[0] == 1.0, "the midpoint starts below 1.7 V, 1y high");
    let mut flip = None;
    while flip.is_none() {
        let t = e.next_event_time();
        assert!(t < 10e-3, "1y never fell");
        e.advance(t).unwrap();
        if e.digital_slots()[0] == 0.0 {
            flip = Some((t, e.voltages()[1], e.voltages()[3]));
        }
    }
    let (t, u, x) = flip.unwrap();
    // The divider is linear in the rail, so its midpoint reached 1.7 V at u
    // (1.7 / x).
    let u_cross = u * 1.7 / x;
    let late = u - u_cross;
    eprintln!("scan: flip at {t:.6e}, u {u:.6} (crossed at {u_cross:.6}, {late:.3e} V late), scans {}, flips {}, moves {}, rail events {}", e.stat(17), e.stat(20), e.stat(14), e.stat(18));
    assert!(x >= 1.7, "never early: {x}");
    assert!(late <= rails::RAIL_SCAN_V + rails::RAIL_DRIFT_V, "at most a scan band and a move late: {late:e}");
    assert_eq!(e.stat(20), 1.0, "the flip is counted once");
    assert!(e.stat(17) >= 10.0, "the rail rose through about 11 scan bands: {}", e.stat(17));
}

// ---- Rails point 1: hub classification -------------------------------------------

fn declines(devices: Vec<Device>, node_count: usize) -> String {
    match PartitionedSolver::build(&program(node_count, devices)) {
        Ok(_) => panic!("loaded; Rails point 1 declines it"),
        Err(e) => {
            assert_eq!(e.code, ERR_UNSUPPORTED_TOPOLOGY, "{}", e.message);
            e.message
        }
    }
}

#[test]
fn hub_classification_declines_what_rails_point_1_declines() {
    let m = declines(vec![battery(0, 1, 0, 5.0, 1.0), battery(1, 2, 0, 3.0, 1.0), resistor(2, 1, 0, 100.0), resistor(3, 2, 0, 100.0)], 3);
    assert!(m.contains("more than one hub rail (W1 limit)"), "{m}");
    let m = declines(vec![battery(0, 1, 0, 5.0, 1.0), psu(1, 2, 0, 3.0, 0.1), resistor(2, 1, 0, 100.0), resistor(3, 2, 0, 100.0)], 3);
    assert!(m.contains("more than one hub rail (W1 limit)"), "a battery and a limited PSU: {m}");
    // `neg` on a node joined to the datum only through a resistor.
    let m = declines(vec![battery(0, 1, 2, 5.0, 1.0), resistor(1, 1, 2, 100.0), resistor(2, 2, 0, 100.0)], 3);
    assert!(m.contains("return is not fixed at 0 V"), "{m}");
    // A voltage_source stacked on the rail.
    // A voltage_source stacked on the rail: the rail is not fixed, so the
    // propagation never reaches the source, which is floating.
    let m = declines(vec![battery(0, 1, 0, 5.0, 1.0), source(1, 2, 1, 1.0), resistor(2, 2, 0, 100.0)], 3);
    assert!(m.contains("floating"), "{m}");
    // A battery in parallel with a voltage_source.
    let m = declines(vec![battery(0, 1, 0, 5.0, 1.0), source(1, 1, 0, 5.0), resistor(2, 1, 0, 100.0)], 2);
    assert!(m.contains("fixed by an ideal source"), "{m}");
    // A clock (an ideal waveform source) on the rail.
    let m = declines(vec![battery(0, 1, 0, 5.0, 1.0), clock(1, 1, 0, 1000.0, 0.5, 0), resistor(2, 1, 0, 100.0)], 2);
    assert!(m.contains("fixed by an ideal source"), "{m}");
    // An ideal source joined to the datum only through resistors is still
    // floating (Q-O39; the anchor covers only isolated sections).
    let m = declines(vec![source(0, 1, 2, 5.0), resistor(1, 1, 0, 100.0), resistor(2, 2, 0, 100.0)], 3);
    assert!(m.contains("floating"), "{m}");
    // {5 V, 0.9 ohm} with 1000 uF across it: r C_rail = 0.9 ms (F4's
    // bat-reservoir-declined).
    let m = declines(vec![battery(0, 1, 0, 5.0, 0.9), capacitor(1, 1, 0, 1e-3, 0.0), resistor(2, 1, 0, 100.0)], 2);
    assert!(m.contains("time constant"), "{m}");
    // 100 nF is 90 ns: it loads.
    assert!(PartitionedSolver::build(&program(2, vec![battery(0, 1, 0, 5.0, 0.9), capacitor(1, 1, 0, 1e-7, 0.0), resistor(2, 1, 0, 100.0)])).is_ok());
}

#[test]
fn an_anchored_isolated_battery_loads_and_reads_0_v_at_its_return() {
    // Section A: 5 V across 1 kohm from the datum. Section B, joined to A by
    // nothing: a {5 V, 1 ohm} battery into 100 ohm, its `neg` anchored (code
    // 26, Rails point 1). Its return reads 0 V and the rail E R/(R + r).
    let p = program(5, vec![source(0, 3, 0, 5.0), resistor(1, 3, 0, 1000.0), battery(2, 1, 2, 5.0, 1.0), resistor(3, 1, 2, 100.0), anchor(4, 2)]);
    let rig = Rig::load(&p).unwrap();
    assert_eq!(rig.v(2), 0.0, "the anchored return");
    rel(rig.u(), 5.0 * 100.0 / 101.0, 1e-10, "the rail");
    rel(rig.i(2), -5.0 / 101.0, 1e-9, "the battery's current");
    rel(rig.v(3), 5.0, 0.0, "section A");
    // Without the anchor its return is a free node: declined.
    let p = program(5, vec![source(0, 3, 0, 5.0), resistor(1, 3, 0, 1000.0), battery(2, 1, 2, 5.0, 1.0), resistor(3, 1, 2, 100.0)]);
    assert_eq!(PartitionedSolver::build(&p).err().map(|e| e.code), Some(ERR_UNSUPPORTED_TOPOLOGY));
}

// ---- Rails point 8: rollback -------------------------------------------------------

type DynamicSnapshot = ((Vec<f64>, Vec<f64>), closed_form::Snapshot, stepped::Snapshot, Vec<(f64, EventKind)>, Option<f64>, Rc<Vec<Driver>>, Vec<f64>, bool, f64);

/// The solver's state, as the whole-copy snapshot captures it: published
/// arrays, the dynamic partitions, the queued events and everything of W1f.
fn state_digest(s: &PartitionedSolver) -> String {
    let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
    let (voltages, currents) = s.snapshot();
    let dynamic = s.dynamic_snapshot().downcast::<(DynamicSnapshot, hub::W1fSnapshot)>().expect("PartitionedSolver's dynamic snapshot");
    let ((_, closed, stepped, pending, _, _, _, _, _), w1f) = *dynamic;
    format!("{:?}\n{:?}\n{}\n{}\n{pending:?}\n{}", bits(&voltages), bits(&currents), closed.digest(), stepped.digest(), w1f.digest())
}

/// A 74LS00 whose gate 1 is a zero-delay inverter on node `a` (`1a` = `1y`)
/// enabled by node `b` on `1b`: with `b` high the gate toggles its own input
/// in the same instant, forever (`-7`).
fn oscillator(ci: usize, vcc: usize, a: usize, b: usize, spare: [usize; 3]) -> Device {
    ls00(ci, vcc, 0, [a, b, a, 0, 0, spare[0], 0, 0, spare[1], 0, 0, spare[2]], 0.0, 0)
}

/// Two identical rigs to `t1`; the first closes switch `sw` there, which
/// livelocks, and must be left exactly as the twin that never did, and go on
/// as it does.
fn rollback_against_a_twin(p: &Program, sw: usize, t1: f64, t2: f64, what: &str) -> (Rig, Rig) {
    let (mut failed, mut twin) = (Rig::load(p).unwrap(), Rig::load(p).unwrap());
    failed.advance(t1).unwrap();
    twin.advance(t1).unwrap();
    let before = state_digest(failed.s());
    assert_eq!(before, state_digest(twin.s()), "{what}: the twins agree before");
    let hub_before = failed.hub().state.clone();
    let r = toggle(&mut failed, sw, true);
    assert!(matches!(r, Err(KernelError::Livelock { .. })), "{what}: the instant livelocks: {r:?}");
    assert_eq!(failed.hub().state, hub_before, "{what}: u, u_scan, reg, the battery, the held draws, the pending rail event and the running sums");
    assert_eq!(state_digest(failed.s()), before, "{what}: every piece of state");
    assert_eq!(failed.k.next_event_time().to_bits(), twin.k.next_event_time().to_bits(), "{what}: the next event");
    failed.advance(t2).unwrap();
    twin.advance(t2).unwrap();
    assert_eq!(state_digest(failed.s()), state_digest(twin.s()), "{what}: it goes on like the twin");
    (failed, twin)
}

#[test]
fn a_livelocked_instant_on_a_battery_rail_restores_the_rail_soc_and_pending_event_bitwise() {
    // A {5 V, 1 ohm, 1e-5 Ah} battery drains through an RC (a closed-form hub
    // partition, so a rail event is always pending) and 100 ohm; the switch
    // pulls `1b` of a zero-delay inverter to the rail.
    let p = battery_rollback_circuit(vec![]);
    let (failed, _) = rollback_against_a_twin(&p, 5, 1.5e-3, 3e-3, "battery");
    let st = &failed.hub().state;
    assert!(st.battery.soc < 1.0 && st.event_t.is_some(), "the rail drained and has an event pending: {st:?}");
}

/// The battery rollback circuit: a {5 V, 1 ohm, 1e-5 Ah} battery drains
/// through an RC (a hub partition, so a rail event is always pending) and
/// 100 ohm; the switch (device 5) pulls `1b` of a zero-delay inverter to the
/// rail. `extra` joins it.
fn battery_rollback_circuit(extra: Vec<Device>) -> Program {
    let mut devices = vec![
        battery_with(0, 1, 0, 1.0, 0.0, 1e-5, 1.0, vec![2.0, 0.0, 0.0, 1.0, 5.0]),
        resistor(1, 1, 2, 1000.0),
        capacitor(2, 2, 0, 1e-6, 0.0),
        resistor(3, 1, 0, 100.0),
        oscillator(4, 1, 3, 4, [5, 6, 7]),
        switch(5, 1, 4, false),
        resistor(6, 4, 0, 1000.0),
    ];
    devices.extend(extra);
    program(8, devices)
}

#[test]
fn a_livelocked_instant_restores_a_stepped_hub_partition_and_a_rail_capacitor() {
    // The same instant with the RC forced stepped (key 4): its commits, its
    // pending trial's factors and rail are restored with the rest.
    let mut p = battery_rollback_circuit(vec![]);
    p.options.force_stepped = true;
    let (failed, _) = rollback_against_a_twin(&p, 5, 1.5e-3, 3e-3, "stepped");
    let pid = failed.s().local_index[2].unwrap().0;
    assert_eq!(failed.s().partition_kinds[pid], PartitionKind::Stepped);
    // With 10 nF across the rail (r C_rail = 10 ns): a rule-2a capacitor on
    // a live node, re-aimed at every rail move, the failed one's included.
    let p = battery_rollback_circuit(vec![capacitor(7, 1, 0, 1e-8, 0.0)]);
    let (failed, _) = rollback_against_a_twin(&p, 5, 1.5e-3, 3e-3, "rail capacitor");
    assert!(failed.s().live_caps);
}

#[test]
fn a_livelocked_instant_in_exact_mode_restores_the_battery_and_its_stamp() {
    let p = exact(battery_rollback_circuit(vec![]));
    let (mut failed, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
    failed.advance(1.5e-3).unwrap();
    twin.advance(1.5e-3).unwrap();
    let before = state_digest(failed.s());
    let ex = failed.s().exact.as_ref().unwrap();
    let saved = (ex.reg, ex.battery, ex.t_last, ex.held);
    assert!(saved.1.soc < 1.0, "the battery drained in exact mode");
    assert!(matches!(toggle(&mut failed, 5, true), Err(KernelError::Livelock { .. })));
    let ex = failed.s().exact.as_ref().unwrap();
    assert_eq!((ex.reg, ex.battery, ex.t_last, ex.held), saved, "reg, the battery, t_last and the held draw");
    assert_eq!(state_digest(failed.s()), before);
    failed.advance(3e-3).unwrap();
    twin.advance(3e-3).unwrap();
    assert_eq!(state_digest(failed.s()), state_digest(twin.s()), "it goes on like the twin");
}

/// Steps `rig` event by event until an instant livelocks, and asserts the
/// solver is then exactly as the last completed instant left it.
fn livelock_by_events(rig: &mut Rig, t_max: f64, what: &str) -> f64 {
    loop {
        let before = state_digest(rig.s());
        let next = rig.k.next_event_time();
        assert!(next < t_max, "{what}: nothing livelocked");
        match rig.advance(next) {
            Ok(()) => continue,
            Err(e) => {
                assert!(matches!(e, KernelError::Livelock { .. }), "{what}: {e:?}");
                assert_eq!(state_digest(rig.s()), before, "{what}: the solver as the last completed instant left it");
                assert_eq!(rig.k.next_event_time(), next, "{what}: the failed instant is due again");
                return next;
            }
        }
    }
}

#[test]
fn a_livelocked_source_edge_restores_the_source_and_its_pending_event() {
    // A clock (from 0.25 ms) drives `1b` of the zero-delay inverter on the
    // battery rail: its first rising edge livelocks. The failed instant
    // applied the edge (level, slot, edge counter, version, the next edge's
    // event): the rollback puts the solver back as the last completed instant
    // left it. In exact mode the rail's partition also committed a step
    // there, which advanced the battery.
    let devices = || {
        let mut c = clock(5, 4, 0, 1000.0, 0.5, 4);
        c.params[4] = 2.5e-4;
        vec![
            battery_with(0, 1, 0, 1.0, 0.0, 1e-5, 1.0, vec![2.0, 0.0, 0.0, 1.0, 5.0]),
            resistor(1, 1, 2, 1000.0),
            capacitor(2, 2, 0, 1e-6, 0.0),
            resistor(3, 1, 0, 100.0),
            oscillator(4, 1, 3, 4, [5, 6, 7]),
            c,
        ]
    };
    let mut rig = Rig::load(&program(8, devices())).unwrap();
    assert_eq!(livelock_by_events(&mut rig, 1e-3, "default"), 2.5e-4, "at the clock's rising edge");
    let mut rig = Rig::load(&exact(program(8, devices()))).unwrap();
    assert_eq!(livelock_by_events(&mut rig, 1e-3, "exact mode"), 2.5e-4);
}

#[test]
fn a_failed_instant_puts_back_whether_the_rail_is_due_to_solve() {
    // A NAND on the battery rail inverts a clock into an LED partition on
    // the rail. Its commit marks the rail due to solve (a rail IC's driver,
    // Rails point 2) before the LED partition's solve, made to fail here
    // (-4), so the rail loop never runs. The rollback must put back the rail
    // as not due, as it was.
    let mut c = clock(1, 2, 0, 1000.0, 0.5, 4);
    c.params[4] = 2.5e-4;
    let p = program(8, vec![battery(0, 1, 0, 5.0, 1.0), c, ls00(2, 1, 0, [2, 2, 3, 0, 0, 4, 0, 0, 5, 0, 0, 6], 10e-9, 0), led(3, 3, 7), resistor(4, 7, 0, 1000.0)]);
    let mut rig = Rig::load(&p).unwrap();
    rig.advance(2.5e-4 + 5e-9).unwrap();
    assert!(!rig.hub().dirty);
    let before = state_digest(rig.s());
    let fault = newton::hooks::force_unconverged(u32::MAX);
    let r = rig.advance(2.5e-4 + 2e-8);
    drop(fault);
    assert!(matches!(r, Err(KernelError::SolveFailed(_))), "{r:?}");
    assert!(!rig.hub().dirty, "not due, as before the instant");
    assert_eq!(state_digest(rig.s()), before);
}

#[test]
fn a_livelocked_instant_that_entered_cc_restores_cv() {
    // {5 V, 0.1 A} into 51.5 ohm draws 97 mA (CV). Closing the switch adds
    // 1 kohm to the rail, 102 mA: CC at 4.90 V, where the zero-delay
    // inverter it enables livelocks. The rollback restores CV.
    let p = program(
        6,
        vec![psu(0, 1, 0, 5.0, 0.1), resistor(1, 1, 0, 51.5), oscillator(2, 1, 2, 3, [4, 5, 0]), switch(3, 1, 3, false), resistor(4, 3, 0, 1000.0)],
    );
    let (failed, _) = rollback_against_a_twin(&p, 3, 1e-3, 2e-3, "psu");
    assert_eq!(failed.hub().state.reg, Regime::Cv);
    // The control: the same switch with the inverter disabled enters CC.
    let p = program(6, vec![psu(0, 1, 0, 5.0, 0.1), resistor(1, 1, 0, 51.5), switch(3, 1, 3, false), resistor(4, 3, 0, 1000.0)]);
    let mut control = Rig::load(&p).unwrap();
    toggle(&mut control, 2, true).unwrap();
    assert_eq!(control.hub().state.reg, Regime::Cc, "the failed instant did enter CC");
}

// ---- Sources points 2-5 -------------------------------------------------------------

/// Advances `e` event by event to `t_end`, returning `(time, node value,
/// slot)` after each instant.
fn walk_events(e: &mut Engine, node: usize, slot: usize, t_end: f64) -> Vec<(f64, f64, f64)> {
    let mut out = Vec::new();
    loop {
        let t = e.next_event_time();
        if t > t_end {
            break;
        }
        e.advance(t).unwrap();
        out.push((t, e.voltages()[node], e.digital_slots()[slot]));
    }
    e.advance(t_end).unwrap();
    out
}

#[test]
fn a_clock_publishes_its_edges_as_events_and_its_slot_at_every_instant() {
    // 1 kHz, duty 0.5, 5 V, from START_TIME = 0 (Sources point 2): the rising
    // edge at 0 is in the load level, then falls at n ms + 0.5 ms and rises
    // at n ms, each an event (stat 19), the slot 1 while HIGH.
    let p = program(2, vec![clock(0, 1, 0, 1000.0, 0.5, 0), resistor(1, 1, 0, 1000.0)]);
    let mut e = Engine::load(&p).unwrap();
    assert_eq!((e.voltages()[1], e.digital_slots()[0]), (5.0, 1.0), "HIGH at load");
    let walked = walk_events(&mut e, 1, 0, 10.25e-3);
    let want: Vec<(f64, f64, f64)> = (1..=20).map(|k| if k % 2 == 1 { ((k / 2) as f64 * 1e-3 + 5e-4, 0.0, 0.0) } else { ((k / 2) as f64 * 1e-3, 5.0, 1.0) }).collect();
    assert_eq!(walked, want);
    assert_eq!(e.stat(19), 20.0, "stat 19 counts the edges");
    // The load and the node's shunt.
    rel(e.currents()[0], -(5e-3 + 5.0 * static_part::NODE_RSHUNT_G), 1e-12, "the clock supplies its load while HIGH");
    // A source stacked on the clock moves with it (Rails point 7).
    let p = program(3, vec![clock(0, 1, 0, 1000.0, 0.5, 0), source(1, 2, 1, 1.0), resistor(2, 2, 0, 1000.0)]);
    let mut e = Engine::load(&p).unwrap();
    assert_eq!(e.voltages()[2], 6.0);
    e.advance(0.75e-3).unwrap();
    assert_eq!(e.voltages()[2], 1.0, "LOW plus the stacked volt");
    // A negative delay is a phase shift: falls at 0.25 ms, rises at 0.75 ms.
    let mut c = clock(0, 1, 0, 1000.0, 0.5, 0);
    c.params[4] = -2.5e-4;
    let mut e = Engine::load(&program(2, vec![c, resistor(1, 1, 0, 1000.0)])).unwrap();
    assert_eq!(e.voltages()[1], 5.0);
    let walked = walk_events(&mut e, 1, 0, 1e-3);
    assert_eq!(walked, vec![(2.5e-4, 0.0, 0.0), (7.5e-4, 5.0, 1.0)]);
}

#[test]
fn a_clamped_fall_on_its_successors_instant_applies_first_and_the_clock_reads_high() {
    // 1 kHz, duty 0.9999999999999999: the fall of cycle 18 computes to
    // 0.019000000000000003, past the rise at 0.019, and is clamped onto it
    // (Sources point 2). The fall keeps its place before the rise, so the
    // instant ends HIGH; the reverse order would leave the clock LOW for a
    // whole period.
    let p = program(2, vec![clock(0, 1, 0, 1000.0, 0.9999999999999999, 0), resistor(1, 1, 0, 1000.0)]);
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.0185).unwrap();
    let edges = e.stat(19);
    assert_eq!(e.voltages()[1], 5.0);
    assert_eq!(e.next_event_time(), 0.019, "the clamped fall is due at the rise's instant");
    e.advance(0.019).unwrap();
    assert_eq!(e.stat(19) - edges, 2.0, "the fall and the rise, at one instant");
    assert_eq!((e.voltages()[1], e.digital_slots()[0]), (5.0, 1.0), "HIGH after the instant");
    e.advance(0.0195).unwrap();
    assert_eq!(e.voltages()[1], 5.0);
}

#[test]
fn a_pulse_steps_at_its_ramp_midpoints() {
    // {v1 0, v2 5, td 0, tr 1 us, tf 1 us, pw 500 us, per 1 ms}: 1 us ramps
    // against an event timescale of 498 us are midpoint steps (Sources point
    // 3): up at 0.5 us, down at 501.5 us, up at 1000.5 us (F2's schedule).
    let p = program(2, vec![pulse(0, 1, 0, [0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3], 0), resistor(1, 1, 0, 1000.0)]);
    let mut e = Engine::load(&p).unwrap();
    assert_eq!((e.voltages()[1], e.digital_slots()[0]), (0.0, 0.0));
    let walked = walk_events(&mut e, 1, 0, 1.2e-3);
    assert_eq!(walked, vec![(5e-7, 5.0, 1.0), (5.015e-4, 0.0, 0.0), (1.0005e-3, 5.0, 1.0)]);
    assert_eq!(e.stat(19), 3.0);
}

/// A 74LS00 with Schmitt inputs (0.9 V and 1.7 V), supplied from `vcc`.
fn schmitt_nand(ci: usize, vcc: usize, logic: [usize; 12], slot: usize) -> Device {
    let mut d = ls00(ci, vcc, 0, logic, 10e-9, slot);
    d.params[ic_param::SCHMITT] = 1.0;
    d.params[ic_param::VT_PLUS] = 1.7;
    d.params[ic_param::VT_MINUS] = 0.9;
    d
}

#[test]
fn a_resolved_ramp_is_a_linear_boundary_with_stepped_readers_and_an_exact_crossing() {
    // {v1 0, v2 5, td 0, tr 1 ms, tf 1 ms, pw 4 ms, per 10 ms}: 1 ms ramps
    // against a 4 ms event timescale are resolved (Sources point 3; F4's
    // pulse-ramp-1ms). An RC of 1 kohm and 1 uF reads the node, and a NAND
    // with Schmitt inputs reads it directly. FORCE_STEPPED keeps the RC on the
    // stepped path a nonlinear reader takes (a linear one is closed-form, D3
    // (b), tested below).
    let devices = || {
        vec![
            pulse(0, 1, 0, [0.0, 5.0, 0.0, 1e-3, 1e-3, 4e-3, 1e-2], 0),
            resistor(1, 1, 2, 1000.0),
            capacitor(2, 2, 0, 1e-6, 0.0),
            source(3, 3, 0, 5.0),
            schmitt_nand(4, 3, [1, 1, 4, 0, 0, 5, 0, 0, 6, 0, 0, 7], 1),
        ]
    };
    let stepped = |devices: Vec<Device>| {
        let mut p = program(8, devices);
        p.options.force_stepped = true;
        p
    };
    let rig = Rig::load(&stepped(devices())).unwrap();
    let p = rig.s().local_index[2].unwrap().0;
    assert_eq!(rig.s().partition_kinds[p], PartitionKind::Stepped, "a stepped partition reading a resolved ramp");
    let pulse_voltage = |t: f64| if t <= 1e-3 { 5.0 * t / 1e-3 } else if t <= 5e-3 { 5.0 } else if t <= 6e-3 { 5.0 - 5.0 * (t - 5e-3) / 1e-3 } else { 0.0 };
    let exact = |t: f64| if t <= 1e-3 { 5000.0 * (t - 1e-3 + 1e-3 * (-t / 1e-3).exp()) } else { 5.0 - 5.0 * (1.0 - (-1.0f64).exp()) * (-(t - 1e-3) / 1e-3).exp() };
    let mut e = Engine::load(&stepped(devices())).unwrap();
    // At every instant the node is pulseVoltage's value, and where the RC
    // committed one step since the last instant (stat 10), it is the
    // step-doubling controller's accepted result: two backward Euler half
    // steps, each with the boundary read at its own end (Rails point 7,
    // Sources point 3), vc' = (vc + (h / tau) v(t)) / (1 + h / tau); one
    // plain step at the minimum step. A corner restarts the partition: its
    // published nodes come from W1b's 1 ps backward Euler seed solve, and its
    // capacitor goes on from the step's own value.
    let t_cross = (1.7 + closed_form::CROSS_NUDGE_V) / 5000.0;
    let (mut t_prev, mut vc_prev, mut steps_prev) = (0.0, 0.0, e.stat(10));
    let (mut fall, mut worst_be, mut worst_exact, mut checked): (Option<f64>, f64, f64, u32) = (None, 0.0, 0.0, 0);
    loop {
        let t = e.next_event_time();
        if t > 3e-3 {
            break;
        }
        e.advance(t).unwrap();
        let (v1, vc) = (e.voltages()[1], e.voltages()[2]);
        close(v1, pulse_voltage(t), 1e-12, &format!("the node at {t:e}"));
        let be_step = |vc: f64, h: f64, t_end: f64| (vc + (h / 1e-3) * pulse_voltage(t_end)) / (1.0 + h / 1e-3);
        let h = t - t_prev;
        let state = if h <= 2.01 * stepped::H_MIN { be_step(vc_prev, h, t) } else { be_step(be_step(vc_prev, h / 2.0, t_prev + h / 2.0), h / 2.0, t) };
        let corner = [1e-3, 5e-3, 6e-3].contains(&t);
        let be = if corner { be_step(state, 1e-12, t) } else { state };
        worst_exact = worst_exact.max((vc - exact(t)).abs());
        if e.stat(10) - steps_prev == 1.0 {
            worst_be = worst_be.max((vc - be).abs());
            close(vc, be, 1e-9, &format!("one BE step to {t:e}"));
            checked += 1;
        }
        if fall.is_none() && e.digital_slots()[1] == 0.0 {
            fall = Some(t);
        }
        // The next step goes on from the capacitor's state, which a corner's
        // seed does not move.
        (t_prev, vc_prev, steps_prev) = (t, if corner && e.stat(10) - steps_prev == 1.0 { state } else { vc }, e.stat(10));
    }
    // The gate's input crosses 1.7 V on the ramp at (1.7 + 1e-9) / 5000 s,
    // exactly; 1y falls one 10 ns delay later.
    close(fall.expect("1y fell on the ramp"), t_cross + 10e-9, 1e-15, "1y falls a delay after the exact crossing");
    assert!(checked >= 50, "most instants were one step: {checked}");
    eprintln!("ramp RC: {checked} single steps, worst against the BE recursion {worst_be:.3e}, against the exact continuous response {worst_exact:.3e}; steps {}", e.stat(10));
    e.advance(5.6e-3).unwrap();
    assert_eq!(e.digital_slots()[0], 0.0, "the slot back at 0 past the fall's midpoint");
    // A capacitor straight across the ramp node is declined (rule 2a).
    let mut d = devices();
    d.push(capacitor(5, 1, 0, 1e-6, 0.0));
    assert_eq!(PartitionedSolver::build(&program(8, d)).err().map(|e| e.code), Some(ERR_UNSUPPORTED_TOPOLOGY));
}

// ---- Sources point 3, R3 (b): a linear closed-form reader of a resolved ramp ----

/// `pulseVoltage`'s trapezoid, written out from the README's definition
/// (Sources point 3), for `per > 0` and `td >= 0`.
fn trapezoid(p: [f64; 7], t: f64) -> f64 {
    let [v1, v2, td, tr, tf, pw, per] = p;
    if t < td {
        return v1;
    }
    let s = (t - td) % per;
    if s < tr {
        v1 + (v2 - v1) * s / tr
    } else if s < tr + pw {
        v2
    } else if s < tr + pw + tf {
        v2 + (v1 - v2) * (s - tr - pw) / tf
    } else {
        v1
    }
}

/// Every corner of the trapezoid in `(0, t_end)`.
fn trapezoid_corners(p: [f64; 7], t_end: f64) -> Vec<f64> {
    let [_, _, td, tr, tf, pw, per] = p;
    let mut out = Vec::new();
    let mut c = 0.0;
    while td + c * per < t_end {
        out.extend([0.0, tr, tr + pw, tr + pw + tf].iter().map(|o| td + c * per + o).filter(|&x| x > 0.0 && x < t_end));
        c += 1.0;
    }
    out
}

/// One conductance into the capacitor's node from a stack of trapezoids, each
/// added (`1.0`) or subtracted (`-1.0`): a source stacked on another adds its
/// value with its `-` on the lower node, and subtracts it with its `+` there.
type Feed<'a> = (f64, &'a [(f64, [f64; 7])]);

fn feed_value(terms: &[(f64, [f64; 7])], t: f64) -> f64 {
    terms.iter().map(|&(sign, p)| sign * trapezoid(p, t)).sum()
}

/// The exact response of a capacitor `c` from its node to ground, the node
/// fed through each conductance `g` from its stack and shunted by
/// `NODE_RSHUNT_G`, from `vc = 0` at `t = 0`. With `G` the total conductance
/// and `veq = Σ g v / G`, every piece between two corners has `veq` linear
/// with slope `q`, and there `vc = veq − q τ + (vc0 − veq0 + q τ) e^(−s/τ)`,
/// `τ = c / G`.
fn pwl_rc(inputs: &[Feed], c: f64, t: f64) -> f64 {
    let big_g = inputs.iter().map(|(g, _)| g).sum::<f64>() + static_part::NODE_RSHUNT_G;
    let tau = c / big_g;
    let veq = |t: f64| inputs.iter().map(|&(g, terms)| g * feed_value(terms, t)).sum::<f64>() / big_g;
    let mut corners: Vec<f64> = inputs.iter().flat_map(|&(_, terms)| terms.iter().flat_map(move |&(_, p)| trapezoid_corners(p, t))).collect();
    corners.sort_by(f64::total_cmp);
    corners.push(t);
    let (mut t0, mut vc) = (0.0, 0.0);
    for t1 in corners {
        if t1 <= t0 {
            continue;
        }
        let (a, b) = (veq(t0), veq(t1));
        let q = (b - a) / (t1 - t0);
        vc = b - q * tau + (vc - a + q * tau) * (-(t1 - t0) / tau).exp();
        t0 = t1;
    }
    vc
}

/// The capacitor's current at `t` on the exact response: `Σ g (v − vc) −
/// g_shunt vc`.
fn pwl_rc_current(inputs: &[Feed], c: f64, t: f64) -> f64 {
    let vc = pwl_rc(inputs, c, t);
    inputs.iter().map(|&(g, terms)| g * (feed_value(terms, t) - vc)).sum::<f64>() - static_part::NODE_RSHUNT_G * vc
}

/// The instant `pwl_rc` crosses `level` in `(lo, hi)`, to the last bit.
fn pwl_rc_crossing(inputs: &[Feed], c: f64, level: f64, mut lo: f64, mut hi: f64) -> f64 {
    let rising = pwl_rc(inputs, c, lo) < level;
    assert!(rising == (pwl_rc(inputs, c, hi) > level), "the bracket holds a crossing");
    loop {
        let mid = lo + 0.5 * (hi - lo);
        if !(mid > lo && mid < hi) {
            return hi;
        }
        if (pwl_rc(inputs, c, mid) >= level) == rising {
            hi = mid;
        } else {
            lo = mid;
        }
    }
}

/// How far a closed-form crossing may sit from the exact response's: every
/// crossing lands its node `CROSS_NUDGE_V` past the threshold (pec3's rule),
/// and the earlier landings, decaying, shift a later root by at most two
/// nudges over the node's slope there.
fn nudge_window(inputs: &[Feed], c: f64, t: f64) -> f64 {
    2.0 * closed_form::CROSS_NUDGE_V / (pwl_rc_current(inputs, c, t) / c).abs()
}

/// W0's tolerance: volts 100 uV plus 1e-3 relative, amps 1 nA plus 1e-3.
fn w0_volts(got: f64, want: f64, what: &str) {
    close(got, want, 1e-4 + 1e-3 * want.abs(), what);
}

fn w0_amps(got: f64, want: f64, what: &str) {
    close(got, want, 1e-9 + 1e-3 * want.abs(), what);
}

/// A `74hc14` (Schmitt, CMOS ratio: `vt+ = 2.7 vS / 5`, `vt- = 1.6 vS / 5`,
/// 15 ns) on `vcc` and the datum, 1a on `pin_1a`, every other input on the
/// datum, 1y (slot `slot`) and the other outputs each on their own node from
/// `first_out` on.
fn hc14(ci: usize, vcc: usize, pin_1a: usize, first_out: usize, slot: usize) -> Device {
    let logic = [pin_1a, first_out, 0, first_out + 1, 0, first_out + 2, 0, first_out + 3, 0, first_out + 4, 0, first_out + 5];
    let mut pins = vec![pin(vcc), pin(0)];
    pins.extend(logic.iter().map(|&n| pin(n)));
    let mut params = vec![0.0; ic_param::COUNT];
    params[ic_param::VCC_MIN] = 2.0;
    params[ic_param::VCC_MAX] = 6.0;
    params[ic_param::FAM_FLAGS] = kinds::FAM_CMOS_RATIO as f64;
    params[ic_param::SCHMITT] = 1.0;
    params[ic_param::VT_PLUS] = 2.7;
    params[ic_param::VT_MINUS] = 1.6;
    params[ic_param::NOMINAL] = 5.0;
    params[ic_param::IO_MAX] = 0.02;
    params[ic_param::DELAY] = HC14_DELAY;
    let mut blob = vec![slot as f64];
    blob.extend((0..12).map(|i| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }));
    Device { kind: kinds::IC_74HC14, component_index: ci, pins, params, blob }
}

const HC14_DELAY: f64 = 15e-9;

/// The instants `slot` changes value up to `t_end`, walking `e` event by
/// event.
fn slot_flips(e: &mut Engine, slot: usize, t_end: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut last = e.digital_slots()[slot];
    loop {
        let t = e.next_event_time();
        if t > t_end {
            break;
        }
        e.advance(t).unwrap();
        let now = e.digital_slots()[slot];
        if now != last {
            out.push((t, now));
            last = now;
        }
    }
    out
}

/// The README's example: `{v1 0, v2 5, td 0, tr 1 ms, tf 1 ms, pw 4 ms, per
/// 10 ms}` (F4's pulse-ramp-1ms).
const RAMP_1MS: [f64; 7] = [0.0, 5.0, 0.0, 1e-3, 1e-3, 4e-3, 1e-2];

#[test]
fn a_linear_rc_reading_a_resolved_ramp_is_the_exact_trapezoid_response() {
    // 1 kohm and 1 uF from the pulse node (Sources point 3, R3 (b)): the
    // closed form driven by each ramp's straight line between its corners is
    // the exact piecewise-linear-input response at every instant, inside each
    // ramp and after it, at W0's tolerance (stepped backward Euler is 2.1e-2 V
    // off it over 3 ms).
    let p = program(3, vec![pulse(0, 1, 0, RAMP_1MS, 0), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0, 1e-6, 0.0)]);
    let inputs: [Feed; 1] = [(1e-3, &[(1.0, RAMP_1MS)])];
    let mut e = Engine::load(&p).unwrap();
    let (mut worst_v, mut worst_i): (f64, f64) = (0.0, 0.0);
    for k in 1..=600 {
        let t = k as f64 * 2e-5;
        e.advance(t).unwrap();
        let (vc, ic) = (e.voltages()[2], e.currents()[2]);
        let (want_v, want_i) = (pwl_rc(&inputs, 1e-6, t), pwl_rc_current(&inputs, 1e-6, t));
        worst_v = worst_v.max((vc - want_v).abs());
        worst_i = worst_i.max((ic - want_i).abs());
        w0_volts(vc, want_v, &format!("vc at {t:e}"));
        w0_amps(ic, want_i, &format!("the capacitor's current at {t:e}"));
        w0_amps(e.currents()[1], want_i + static_part::NODE_RSHUNT_G * want_v, &format!("the resistor's current at {t:e}"));
    }
    eprintln!("ramp RC against the exact trapezoid response over 12 ms: worst {worst_v:.3e} V, {worst_i:.3e} A; segments {}", e.stat(8));
    close(worst_v, 0.0, 1e-9, "the closed form is the exact response, to rounding");
    let rig = Rig::load(&p).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    assert_eq!(rig.s().partition_kinds[part], PartitionKind::ClosedForm, "a linear closed-form reader of a resolved ramp stays closed-form");
    // Ramps stacked on the ramp (Rails point 7): node 3 is node 1 plus a 2 V,
    // 0.5 ms pulse (its `-` on node 1), node 4 is node 3 less a 1 V, 2 ms one
    // (its `+` on node 3), and the RC reads node 4, its drive the sum of
    // whichever ramps run, each corner of each aiming it again.
    let up = [0.0, 2.0, 0.7e-3, 0.5e-3, 0.5e-3, 3e-3, 1e-2];
    let down = [0.0, 1.0, 0.2e-3, 2e-3, 2e-3, 4e-3, 1e-2];
    let p = program(5, vec![pulse(0, 1, 0, RAMP_1MS, 0), pulse(1, 3, 1, up, 1), pulse(2, 3, 4, down, 2), resistor(3, 4, 2, 1000.0), capacitor(4, 2, 0, 1e-6, 0.0)]);
    let inputs: [Feed; 1] = [(1e-3, &[(1.0, RAMP_1MS), (1.0, up), (-1.0, down)])];
    let mut e = Engine::load(&p).unwrap();
    let mut worst: f64 = 0.0;
    for k in 1..=600 {
        let t = k as f64 * 2e-5;
        e.advance(t).unwrap();
        let want = pwl_rc(&inputs, 1e-6, t);
        close(e.voltages()[4], feed_value(inputs[0].1, t), 1e-12, &format!("the top of the stack at {t:e}"));
        worst = worst.max((e.voltages()[2] - want).abs());
        w0_volts(e.voltages()[2], want, &format!("vc on the stack at {t:e}"));
        w0_amps(e.currents()[4], pwl_rc_current(&inputs, 1e-6, t), &format!("the capacitor's current on the stack at {t:e}"));
    }
    eprintln!("ramp RC on a stack of three ramps: worst {worst:.3e} V");
    close(worst, 0.0, 1e-9, "the stack's drive is exact, to rounding");
}

#[test]
fn a_ramp_corner_between_two_of_the_partitions_events_keeps_the_drive_exact() {
    // Two resolved ramps feed one capacitor: A `{0, 5, 0, 1 ms, 1 ms, 4 ms,
    // 10 ms}` through 1 kohm and B `{0, 3, 0.4 ms, 1.5 ms, 1 ms, 3 ms, 10
    // ms}` through 2 kohm, 1 uF to ground, and a 74hc14 watches the node. B's
    // corner at 0.4 ms lands partway through A's ramp, A's end at 1 ms
    // partway through B's, B's end at 1.9 ms between the hc14's rising and
    // falling crossings, and the fall's corners interleave the same way: each
    // corner aims the partition again from its exact charge, with the slope
    // of every ramp then running (R3 (b), the coordinator's required case).
    let b_params = [0.0, 3.0, 0.4e-3, 1.5e-3, 1e-3, 3e-3, 1e-2];
    let mut p = program(
        12,
        vec![
            pulse(0, 1, 0, RAMP_1MS, 0),
            pulse(1, 3, 0, b_params, 1),
            resistor(2, 1, 2, 1000.0),
            resistor(3, 3, 2, 2000.0),
            capacitor(4, 2, 0, 1e-6, 0.0),
            source(5, 4, 0, 5.0),
            hc14(6, 4, 2, 5, 2),
        ],
    );
    p.digital_slot_count = 8;
    let inputs: [Feed; 2] = [(1e-3, &[(1.0, RAMP_1MS)]), (5e-4, &[(1.0, b_params)])];
    let c = 1e-6;
    let mut e = Engine::load(&p).unwrap();
    let mut worst: f64 = 0.0;
    for k in 1..=450 {
        let t = k as f64 * 2e-5;
        e.advance(t).unwrap();
        let want = pwl_rc(&inputs, c, t);
        worst = worst.max((e.voltages()[2] - want).abs());
        w0_volts(e.voltages()[2], want, &format!("vc at {t:e}"));
        w0_amps(e.currents()[4], pwl_rc_current(&inputs, c, t), &format!("the capacitor's current at {t:e}"));
    }
    // 1y falls a delay after 1a crosses 2.7 V rising, and rises a delay
    // after it crosses 1.6 V falling.
    let rise = pwl_rc_crossing(&inputs, c, 2.7, 1.0e-3, 1.9e-3);
    let fall = pwl_rc_crossing(&inputs, c, 1.6, 5.9e-3, 9e-3);
    let mut e = Engine::load(&p).unwrap();
    let flips = slot_flips(&mut e, 2, 9e-3);
    eprintln!("two ramps, corners mid-segment: worst {worst:.3e} V over 9 ms; crossings at {rise:.12e} and {fall:.12e}, 1y flips {flips:?}");
    assert_eq!(flips.len(), 2, "1y falls once and rises once: {flips:?}");
    close(flips[0].0, rise + HC14_DELAY, nudge_window(&inputs, c, rise), "1y falls a delay after the exact rising crossing");
    close(flips[1].0, fall + HC14_DELAY, nudge_window(&inputs, c, fall), "1y rises a delay after the exact falling crossing");
    // The four landings (1.6 and 2.7 V each way) are all it carries.
    close(worst, 0.0, 2.0 * closed_form::CROSS_NUDGE_V, "the closed form is the exact response, but for the crossings' nudges");
    let rig = Rig::load(&p).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    assert_eq!(rig.s().partition_kinds[part], PartitionKind::ClosedForm);
}

#[test]
fn a_reader_pin_on_the_rc_node_crosses_at_the_exact_instant_inside_the_ramp() {
    // 100 ohm and 1 uF (tau 0.1 ms) on the 1 ms ramp, a 74hc14's 1a on the
    // capacitor's node: it crosses 2.7 V inside the rise and 1.6 V inside the
    // fall (vc lags the ramp by about 0.5 V), each on the exact response, and
    // 1y follows a delay later (Sources point 3, R3 (b): crossings on a driven
    // node are exact, within the nudges of the crossings before them).
    let mut p = program(10, vec![pulse(0, 1, 0, RAMP_1MS, 0), resistor(1, 1, 2, 100.0), capacitor(2, 2, 0, 1e-6, 0.0), source(3, 3, 0, 5.0), hc14(4, 3, 2, 4, 1)]);
    p.digital_slot_count = 7;
    let inputs: [Feed; 1] = [(1e-2, &[(1.0, RAMP_1MS)])];
    let c = 1e-6;
    let rise = pwl_rc_crossing(&inputs, c, 2.7, 0.1e-3, 1e-3);
    let fall = pwl_rc_crossing(&inputs, c, 1.6, 5.1e-3, 6e-3);
    let mut e = Engine::load(&p).unwrap();
    let flips = slot_flips(&mut e, 1, 8e-3);
    eprintln!("hc14 on the RC node: crossings at {rise:.12e} and {fall:.12e}, 1y flips {flips:?}");
    assert_eq!(flips.len(), 2, "1y falls once and rises once: {flips:?}");
    assert_eq!(flips[0].1, 0.0);
    close(flips[0].0, rise + HC14_DELAY, nudge_window(&inputs, c, rise), "1y falls a delay after the exact crossing inside the rise");
    close(flips[1].0, fall + HC14_DELAY, nudge_window(&inputs, c, fall), "1y rises a delay after the exact crossing inside the fall");
}

#[test]
fn a_nonlinear_or_coupled_reader_of_a_resolved_ramp_stays_stepped() {
    // On the same ramp (Sources point 3, R3 (b)): an LED beside the capacitor
    // (nonlinear), a second capacitor (two), and a 74hc14 supplied from the
    // ramp with its input on the RC node (its threshold moves with the ramp)
    // keep the reader stepped; the plain RC is closed-form.
    let kind_of = |devices: Vec<Device>, slots: usize| {
        let mut p = program(10, devices);
        p.digital_slot_count = slots;
        let rig = Rig::load(&p).unwrap();
        let part = rig.s().local_index[2].unwrap().0;
        rig.s().partition_kinds[part]
    };
    let rc = || vec![pulse(0, 1, 0, RAMP_1MS, 0), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0, 1e-6, 0.0)];
    assert_eq!(kind_of(rc(), 1), PartitionKind::ClosedForm, "the linear RC");
    let mut with_led = rc();
    with_led.push(led(3, 2, 0));
    assert_eq!(kind_of(with_led, 1), PartitionKind::Stepped, "an LED reader");
    let mut two_caps = rc();
    two_caps.extend([resistor(3, 2, 3, 1000.0), capacitor(4, 3, 0, 1e-6, 0.0)]);
    assert_eq!(kind_of(two_caps, 1), PartitionKind::Stepped, "a two-capacitor reader");
    let mut ramp_supplied = rc();
    ramp_supplied.push(hc14(3, 1, 2, 4, 1));
    assert_eq!(kind_of(ramp_supplied, 7), PartitionKind::Stepped, "an IC on the ramp reading the RC node");
}

#[test]
fn a_failed_instant_mid_ramp_restores_the_ramp_drive() {
    // The RC on the 1 ms ramp, with a switch from its node to 10 kohm and a
    // zero-delay inverter (vcc a 5 V source) whose enable a second switch
    // pulls high. Closing both at 0.4 ms, inside the rise, aims the RC under
    // the ramp's drive and then livelocks (-7): the rollback puts back the
    // drive it was aimed under with the rest, and the RC goes on exactly as
    // a twin that never closed them (Sources point 3, R3 (b); W1b point 7).
    let p = program(
        9,
        vec![
            pulse(0, 1, 0, RAMP_1MS, 0),
            resistor(1, 1, 2, 1000.0),
            capacitor(2, 2, 0, 1e-6, 0.0),
            switch(3, 2, 6, false),
            resistor(4, 6, 0, 10_000.0),
            source(5, 3, 0, 5.0),
            oscillator(6, 3, 4, 5, [7, 8, 0]),
            switch(7, 3, 5, false),
            resistor(8, 5, 0, 1000.0),
        ],
    );
    let (mut failed, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
    let part = failed.s().local_index[2].unwrap().0;
    assert_eq!(failed.s().partition_kinds[part], PartitionKind::ClosedForm);
    failed.advance(4e-4).unwrap();
    twin.advance(4e-4).unwrap();
    let before = state_digest(failed.s());
    assert_eq!(before, state_digest(twin.s()));
    let r = failed.k.apply_params_as_event(&[(3, 0, 1.0), (7, 0, 1.0)]);
    failed.k.solver_mut().prepare_readout();
    assert!(matches!(r, Err(KernelError::Livelock { .. })), "the instant livelocks: {r:?}");
    assert_eq!(state_digest(failed.s()), before, "every piece of state, the ramp drive included");
    assert_eq!(failed.k.next_event_time().to_bits(), twin.k.next_event_time().to_bits(), "the next event");
    for t in [7e-4, 1.5e-3, 5.5e-3] {
        failed.advance(t).unwrap();
        twin.advance(t).unwrap();
        assert_eq!(state_digest(failed.s()), state_digest(twin.s()), "it goes on like the twin, at {t:e}");
    }
    close(failed.v(2), pwl_rc(&[(1e-3, &[(1.0, RAMP_1MS)])], 1e-6, 5.5e-3), 1e-12, "and on the exact response");
}

// ---- C ABI: device state and stats 14, 16-20 ---------------------------------------

#[test]
fn device_state_has_the_readme_layouts_and_reading_it_changes_nothing() {
    // {5 V, 0.9 ohm} generic (OCV 5 soc, 1 Ah) into 200 ohm (F4's
    // bat-generic-200r, probe Q1): u = 5 * 200/200.9 = 4.9776008. The draw is
    // 5 soc / 200.9 A, so soc = e^(-5 t / (200.9 * 3600)) exactly; after 1 s
    // 0.99999309 (classic, h = 1e-3: 0.99999308669).
    let p = program(2, vec![generic_battery(0, 1, 0, 5.0, 0.9, 1.0), resistor(1, 1, 0, 200.0)]);
    let (mut read, mut quiet) = (Engine::load(&p).unwrap(), Engine::load(&p).unwrap());
    rel(read.voltages()[1], 5.0 * 200.0 / 200.9, 1e-10, "u at load");
    assert_eq!(read.device_state(0).unwrap(), &[1.0, 5.0, 0.9, 3600.0, 0.0, 0.0], "soc, OCV, R, remaining C, discharged, rejected at load");
    for k in 1..=10 {
        read.advance(k as f64 * 0.1).unwrap();
        quiet.advance(k as f64 * 0.1).unwrap();
        // A read projects SoC to now; twice is the same, and it commits nothing.
        let a = read.device_state(0).unwrap().to_vec();
        assert_eq!(read.device_state(0).unwrap(), &a[..]);
    }
    let state = read.device_state(0).unwrap().to_vec();
    let soc = (-5.0 / (200.9 * 3600.0f64)).exp();
    eprintln!("device state after 1 s: {state:?} (soc exact {soc:.12}); rail moves {}, events {}", read.stat(14), read.stat(18));
    close(state[0], soc, 1e-11, "soc against the exact ODE");
    close(state[0], 0.99999309, 1e-8, "the draft's soc");
    rel(state[1], 5.0 * state[0], 1e-15, "OCV");
    rel(state[3], 3600.0 * state[0], 1e-12, "remaining capacity");
    close(state[4], 3600.0 * (1.0 - state[0]), 1e-9, "discharged");
    assert_eq!(state[5], 0.0);
    assert_eq!(read.voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), quiet.voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), "reading changed nothing");
    assert_eq!(quiet.device_state(0).unwrap(), &state[..]);
    // A resistor has W1g's thermal layout, empty without a code 91 record.
    assert_eq!(read.device_state(1).unwrap(), &[] as &[f64]);
    assert_eq!(read.device_state(2).err().map(|e| e.code), Some(crate::error::ERR_INVALID_ARGUMENT), "out of range");
    // A limited PSU: reg; an unlimited one: an empty layout.
    let mut e = Engine::load(&program(2, vec![psu(0, 1, 0, 5.0, 0.1), resistor(1, 1, 0, 10.0)])).unwrap();
    assert_eq!(e.device_state(0).unwrap(), &[2.0], "reg 2 (probe P5: 1 V, -0.1 A)");
    rel(e.voltages()[1], 1.0, 1e-9, "probe P5's rail");
    let mut e = Engine::load(&program(2, vec![psu(0, 1, 0, 5.0, 0.0), resistor(1, 1, 0, 10.0)])).unwrap();
    assert_eq!(e.device_state(0).unwrap(), &[] as &[f64]);
}

#[test]
fn stats_14_and_16_to_20_count_the_rail_and_the_sources_and_stay_0_in_exact_mode() {
    // The coupled RC of the rail-event test, and a clock on its own section.
    let devices = || vec![battery(0, 1, 0, 5.0, 1.0), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0, 1e-6, 0.0), clock(3, 3, 0, 1000.0, 0.5, 0), resistor(4, 3, 0, 1000.0)];
    let mut e = Engine::load(&program(4, devices())).unwrap();
    e.advance(5e-3).unwrap();
    let stats: Vec<f64> = (14..=20).filter(|&i| i != 15).map(|i| e.stat(i)).collect();
    eprintln!("stats 14, 16-20 after 5 ms: {stats:?}");
    assert!(e.stat(14) > 100.0 && e.stat(18) > 100.0, "rail moves and events");
    assert!(e.stat(16) >= 1.0, "sensitivities");
    assert_eq!(e.stat(19), 10.0, "ten clock edges");
    // Since round 14 the horizon clamp solves a cell whose closed-form
    // partitions follow its path (ruling R15 (a)): one more move, at 5 ms.
    assert_eq!(e.stat(14), e.stat(18) + 2.0, "every event moved the rail, and the load and the clamp did once each");
    let mut x = Engine::load(&exact(program(4, devices()))).unwrap();
    x.advance(5e-3).unwrap();
    for i in [14, 16, 17, 18, 20] {
        assert_eq!(x.stat(i), 0.0, "stat {i} in exact mode");
    }
    assert_eq!(x.stat(19), 10.0, "source edges count in exact mode too");
    close(x.voltages()[1], e.voltages()[1], 2.0 * rails::RAIL_DRIFT_V, "exact mode against the rail");
}

// ---- F1r4: the CC rail capacitor (F1v23 findings 1-5) -------------------------

/// A Schmitt NAND (0.9 V and 1.7 V at 5 V) with `vcc` and `gnd` on any node.
fn schmitt_on(ci: usize, vcc: usize, gnd: usize, logic: [usize; 12], slot: usize) -> Device {
    let mut d = ls00(ci, vcc, gnd, logic, 10e-9, slot);
    d.params[ic_param::SCHMITT] = 1.0;
    d.params[ic_param::VT_PLUS] = 1.7;
    d.params[ic_param::VT_MINUS] = 0.9;
    d
}

/// A NAND with the generic fallback levels `max(0.8, 0.3 vs)` and `max(2.0,
/// 0.6 vs)` (neither fixed), powered from 1.8 V.
fn generic_nand(ci: usize, vcc: usize, gnd: usize, logic: [usize; 12], slot: usize) -> Device {
    let mut d = ls00(ci, vcc, gnd, logic, 10e-9, slot);
    d.params[ic_param::VIL_FIXED] = 0.0;
    d.params[ic_param::VIH_FIXED] = 0.0;
    d.params[ic_param::VCC_MIN] = 2.0;
    d
}

/// An RK4 reference of a CC rail capacitor (`c_r`, `esr`) charged at `dl`
/// with an RC (`r`, `c`) off the rail and every node's shunt: `(t, u, v)`
/// every `dt` up to `t_end`.
fn inrush_reference(dl: f64, c_r: f64, esr: f64, r: f64, c: f64, dt: f64, t_end: f64) -> Vec<(f64, f64, f64)> {
    let gs = static_part::NODE_RSHUNT_G;
    let rail = |q: f64, v: f64| (q + esr * dl + esr * v / r) / (1.0 + esr * gs + esr / r);
    let f = |q: f64, v: f64| {
        let u = rail(q, v);
        ((dl - gs * u - (u - v) / r) / c_r, ((u - v) / r - gs * v) / c)
    };
    let (mut q, mut v, mut t) = (0.0, 0.0, 0.0);
    let mut out = vec![(0.0, rail(0.0, 0.0), 0.0)];
    for _ in 0..(t_end / dt).round() as usize {
        let k1 = f(q, v);
        let k2 = f(q + 0.5 * dt * k1.0, v + 0.5 * dt * k1.1);
        let k3 = f(q + 0.5 * dt * k2.0, v + 0.5 * dt * k2.1);
        let k4 = f(q + dt * k3.0, v + dt * k3.1);
        q += dt / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0);
        v += dt / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1);
        t += dt;
        out.push((t, rail(q, v), v));
    }
    out
}

/// The first instant after 0 at which `g(u, v)` changes sign in the
/// reference, interpolated.
fn reference_crossing(refr: &[(f64, f64, f64)], g: impl Fn(f64, f64) -> f64) -> Option<f64> {
    for w in refr.windows(2) {
        let (a, b) = (g(w[0].1, w[0].2), g(w[1].1, w[1].2));
        if a != 0.0 && (a < 0.0) != (b < 0.0) && w[0].0 > 0.0 {
            return Some(w[0].0 + (w[1].0 - w[0].0) * a / (a - b));
        }
    }
    None
}

#[test]
fn leaving_cc_with_esr_holds_the_setpoint_without_re_entering() {
    // F1v23 finding 1 (ruled (a), 2026-10-15). {5 V, 0.1 A} with 100 uF at
    // esr 0.02-0.1; a switch adds R to ground at ts, in CC. At the exit the
    // capacitors take what the limit leaves of the CV draw at the setpoint
    // (Rails point 5), so the next CV -> CC test reads the limit itself:
    // the PSU stays CV. On dfd3a03 42 of these 96 re-entered CC in the
    // exit's instant at u = 5 V and left 4/10/20 us late.
    let mut flick = Vec::new();
    let mut n = 0;
    for r in [50.0, 75.0, 100.0, 120.0, 150.0, 200.0, 330.0, 1e3] {
        for esr in [0.02, 0.05, 0.1] {
            for ts in [0.5e-3, 1e-3, 2e-3, 3e-3] {
                n += 1;
                let p = program(3, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, esr), switch(2, 1, 2, false), resistor(3, 2, 0, r)]);
                let mut rig = Rig::load(&p).unwrap();
                rig.advance(ts).unwrap();
                toggle(&mut rig, 2, true).unwrap();
                let mut exit = None;
                loop {
                    let t = rig.k.next_event_time();
                    if t > 0.03 {
                        break;
                    }
                    rig.advance(t).unwrap();
                    let reg = rig.hub().state.reg;
                    // A rail in CC at the setpoint has left CC and entered
                    // again within one instant.
                    if reg == Regime::Cc && rig.u() > 5.0 - 1e-9 {
                        flick.push(format!("R {r} esr {esr} toggle {ts:e}: CC at u = {} at {t:.9e}", rig.u()));
                        break;
                    }
                    if exit.is_none() && reg == Regime::Cv {
                        exit = Some(t);
                    } else if exit.is_some() && reg == Regime::Cc {
                        flick.push(format!("R {r} esr {esr} toggle {ts:e}: CC again at {t:.9e} after the exit at {:.9e}", exit.unwrap()));
                        break;
                    }
                    if exit.is_some_and(|e| t > e + 1e-3) {
                        break;
                    }
                }
                // 50 ohm draws the limit itself at 5 V: it never leaves.
                if exit.is_some() {
                    assert_eq!(rig.v(1), 5.0);
                }
            }
        }
    }
    assert!(flick.is_empty(), "{} of {n}:\n{}", flick.len(), flick.join("\n"));
}

#[test]
fn a_load_step_in_cc_with_esr_leaves_at_the_closed_form_instant() {
    // F1v23 finding 1's trace. {5 V, 0.1 A} with 100 uF at esr 0 and 0.05;
    // at 2 ms a switch (1 mohm) adds 100 ohm: the rail steps down by the esr
    // drop and heads for iLimit / G = 10 V, so it leaves CC at 5 V at the
    // closed form's instant (Rails point 4). On dfd3a03 esr 0.05 re-entered
    // at the exit and left 10 us late.
    for esr in [0.0, 0.05] {
        let (c, il, r) = (1e-4, 0.1, 100.0);
        let p = program(3, vec![psu(0, 1, 0, 5.0, il), capacitor(1, 1, 0, c, esr), switch(2, 1, 2, false), resistor(3, 2, 0, r)]);
        let mut rig = Rig::load(&p).unwrap();
        rig.advance(2e-3).unwrap();
        let gs = static_part::NODE_RSHUNT_G;
        let q = rig.v(1) - esr * (il - gs * rig.v(1));
        toggle(&mut rig, 2, true).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cc);
        let g = gs + 1.0 / (1e-3 + 1.0 / (1.0 / r + gs));
        let alpha = 1.0 / (1.0 + g * esr);
        let u0 = alpha * (q + esr * il);
        close(rig.v(1), u0, 1e-9, &format!("esr {esr}: the step at the load"));
        let (u_inf, k) = (il / g, alpha * g / c);
        let want_exit = 2e-3 - ((5.0 - u_inf) / (u0 - u_inf)).ln() / k;
        let t_exit = cc_exit(&mut rig, 20e-3);
        close(t_exit, want_exit, 1e-10, &format!("esr {esr}: the exit"));
        rig.advance(t_exit + 1e-3).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cv, "esr {esr}: no re-entry");
        assert_eq!(rig.v(1), 5.0);
    }
}

#[test]
fn a_load_step_landing_on_the_exit_still_enters_cc() {
    // Ruling (a) over (b): the exit forbids no re-entry, only rounding. As
    // above at esr 0.05, the PSU leaves CC with its capacitors taking
    // iLimit - 50 mA; a second switch adds 1 kohm at the exit's own instant.
    // The draw at the setpoint, 55 mA, is inside the limit, but with the
    // capacitors' 50 mA it is 0.105 A, past it: CC again at once, from the
    // capacitors' charge, the rail one esr drop of the capacitor current's
    // change (5 mA) below the setpoint.
    let devices = |r_extra: f64| program(5, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.05), switch(2, 1, 2, false), resistor(3, 2, 0, 100.0), switch(4, 1, 3, false), resistor(5, 3, 0, r_extra)]);
    let mut probe = Rig::load(&devices(1e3)).unwrap();
    probe.advance(2e-3).unwrap();
    toggle(&mut probe, 2, true).unwrap();
    let t_exit = cc_exit(&mut probe, 20e-3);
    let mut rig = Rig::load(&devices(1e3)).unwrap();
    rig.advance(2e-3).unwrap();
    toggle(&mut rig, 2, true).unwrap();
    rig.advance(t_exit).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cv, "left CC at {t_exit:e}");
    assert_eq!(rig.k.time(), t_exit);
    toggle(&mut rig, 4, true).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc, "the load step on the exit enters CC");
    assert_eq!(rig.i(0), -0.1);
    let g = 1.0 / (1e-3 + 100.0) + 1.0 / (1e-3 + 1e3);
    close(rig.v(1), 5.0 - 0.05 * 5.0 * (g - 1.0 / (1e-3 + 100.0)) / (1.0 + 0.05 * g), 1e-6, "one esr drop below the setpoint");
}

#[test]
fn a_slow_inrush_crosses_each_schmitt_threshold_once_at_any_esr() {
    // F1v23 finding 2. CC rail capacitors at esr 0-5 (both polarities) with
    // an RC off the rail and a Schmitt NAND on the rail reading the RC node:
    // the node crosses each threshold once. The rail solve a crossing owes
    // includes the esr offset's step with the draws (Rails point 4); on
    // dfd3a03 esr 2 and 5 re-fired crossings (3 and 5 for 2 thresholds).
    let cases: [(f64, f64, f64, f64, f64, f64); 14] = [
        (5.0, 0.1, 1e-4, 1.0, 1e3, 1e-6),
        (5.0, 0.1, 1e-4, 2.0, 1e3, 1e-6),
        (5.0, 0.1, 1e-4, 2.0, 100.0, 1e-5),
        (5.0, 0.1, 1e-4, 5.0, 100.0, 1e-5),
        (-5.0, 0.1, 1e-4, 2.0, 1e3, 1e-6),
        (5.0, 0.01, 1e-3, 2.0, 100.0, 1e-4),
        (5.0, 0.1, 1e-4, 0.0, 1e3, 1e-6),
        (5.0, 0.1, 1e-4, 0.05, 1e3, 1e-6),
        (5.0, 0.1, 1e-4, 0.5, 1e3, 1e-6),
        (5.0, 0.1, 1e-4, 0.05, 100.0, 1e-5),
        (-5.0, 0.1, 1e-4, 0.0, 1e3, 1e-6),
        (-5.0, 0.1, 1e-4, 0.05, 1e3, 1e-6),
        (5.0, 0.01, 1e-3, 0.0, 100.0, 1e-4),
        (5.0, 0.01, 1e-3, 0.05, 100.0, 1e-4),
    ];
    let mut bad = Vec::new();
    for (v, il, c_r, esr, r, c) in cases {
        let t_end = 0.9 * v.abs() * c_r / il;
        let nand = if v > 0.0 { schmitt_on(4, 1, 0, [2, 2, 3, 0, 0, 4, 0, 0, 5, 0, 0, 6], 0) } else { schmitt_on(4, 0, 1, [2, 2, 3, 1, 1, 4, 1, 1, 5, 1, 1, 6], 0) };
        let mut rig = Rig::load(&program(7, vec![psu(0, 1, 0, v, il), capacitor(1, 1, 0, c_r, esr), resistor(2, 1, 2, r), capacitor(3, 2, 0, c, 0.0), nand])).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cc);
        let dl = if v < 0.0 { -il } else { il };
        let refr = inrush_reference(dl, c_r, esr, r, c, 1e-8_f64.min(t_end / 2e6), t_end);
        // A threshold as a fraction of u: vt / 5 with vcc on the rail, 1 - vt
        // / 5 with gnd on it.
        let fr = |vt: f64| if v > 0.0 { vt / 5.0 } else { 1.0 - vt / 5.0 };
        let want = [fr(0.9), fr(1.7)].iter().filter(|&&f| reference_crossing(&refr, |u, x| x - f * u).is_some()).count();
        let mut seen = 0;
        let mut count = rig.s().closed_form.crossings();
        loop {
            let t = rig.k.next_event_time();
            if t > t_end || rig.hub().state.reg == Regime::Cv {
                break;
            }
            rig.advance(t).unwrap();
            let n = rig.s().closed_form.crossings();
            if n > count {
                seen += 1;
                count = n;
            }
        }
        if seen != want {
            bad.push(format!("V {v} il {il} esr {esr} R {r}: {seen} crossings, want {want}"));
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

/// The LED branch current at rail `u`: the LED from the rail to `m`, `r` to
/// ground, solved for `m` by bisection.
fn led_branch_at(u: f64, r: f64) -> f64 {
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let gs = static_part::NODE_RSHUNT_G;
    let f = |m: f64| law.current(u - m) + led::GMIN_D * (u - m) - m / r - gs * m;
    let (mut lo, mut hi) = (-1.0, u.max(0.0) + 1.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let m = 0.5 * (lo + hi);
    law.current(u - m) + led::GMIN_D * (u - m)
}

#[test]
fn an_led_hub_partition_follows_its_law_through_the_cc_ramp() {
    // F1v23 finding 3 (ruled (a), 2026-10-15). {5 V, 0.1 A} ramping 100 uF,
    // an LED and 330 ohm off the rail (a nonlinear static hub partition):
    // it solves again at RAIL_CC_CAP_RESOLVE_V of rail motion, so its
    // published current is its own law at the published rail within W0's
    // tolerance (1 nA + 1e-3), through the knee too. On dfd3a03 the 25 mV
    // band left it 1.6% off from 1.6 to 2.2 ms (155.1 vs 152.7 uA).
    let (c, il, r) = (1e-4, 0.1, 330.0);
    let mut rig = Rig::load(&program(3, vec![psu(0, 1, 0, 5.0, il), capacitor(1, 1, 0, c, 0.0), led(2, 1, 2), resistor(3, 2, 0, r)])).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    let gs = static_part::NODE_RSHUNT_G;
    let dudt = |u: f64| (il - gs * u - led_branch_at(u, r)) / c;
    let (mut u, mut t, dt) = (0.0f64, 0.0f64, 1e-7);
    for k in 1..=60 {
        let ts = k as f64 * 1e-4;
        while t < ts - 1e-12 {
            let k1 = dudt(u);
            let k2 = dudt(u + 0.5 * dt * k1);
            let k3 = dudt(u + 0.5 * dt * k2);
            let k4 = dudt(u + dt * k3);
            u += dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
            t += dt;
        }
        if u >= 5.0 {
            break;
        }
        rig.advance(ts).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cc);
        close(rig.v(1), u, 1e-4 + 1e-3 * u.abs(), &format!("u at {ts:e} against the reference"));
        let law = led_branch_at(rig.v(1), r);
        close(rig.i(2), law, 1e-9 + 1e-3 * law.abs(), &format!("I_LED at {ts:e} against its law at the published rail"));
    }
}

#[test]
fn leaving_the_cc_ramp_an_led_partition_agrees_with_cache_off() {
    // The CC exit re-solves every hub partition at the setpoint (Rails point
    // 5). The 1 mV band of the ramp stores cache entries close below the
    // setpoint, so a hit there would carry its first-order error past the
    // exit, where cache off solves exactly: they must agree (LEDs point 7,
    // 1e-6 V; corpus 06-08 parted by 1.3e-5 V before the exit's re-solve
    // bypassed the band).
    let devices = || vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0), led(2, 1, 2), resistor(3, 2, 0, 330.0)];
    let mut on = Rig::load(&program(3, devices())).unwrap();
    let mut off_p = program(3, devices());
    off_p.options.no_solution_cache = true;
    let mut off = Rig::load(&off_p).unwrap();
    let t_exit = cc_exit(&mut on, 20e-3);
    assert_eq!(cc_exit(&mut off, 20e-3), t_exit);
    for t in [t_exit, t_exit + 1e-3] {
        on.advance(t).unwrap();
        off.advance(t).unwrap();
        close(on.v(2), off.v(2), 1e-9, &format!("the LED's cathode at {t:e}, cache on against off"));
    }
}

/// Steps `rig` while its PSU is in CC up to `t_end` and returns the first
/// event after `t_from` at which node `n` is at or below `th` of the
/// published voltage of node `sup`.
fn first_below(rig: &mut Rig, n: usize, sup: usize, th: &dyn Fn(f64) -> f64, t_from: f64, t_end: f64) -> Option<f64> {
    loop {
        let t = rig.k.next_event_time();
        if t > t_end || rig.hub().state.reg == Regime::Cv {
            return None;
        }
        rig.advance(t).unwrap();
        if t > t_from && rig.v(n) <= th(rig.v(sup)) {
            return Some(t);
        }
    }
}

#[test]
fn a_generic_threshold_past_its_kink_rises_past_the_node_at_the_instant() {
    // F1v23 finding 4. {5 V, 10 mA} into 100 uF: the rail ramps at 100 V/s.
    // A generic NAND on the rail reads node 2, held at `level` by a divider
    // and capacitor off a fixed 5 V source (a partition that does not read
    // the rail, aimed under it for its watch). vil = max(0.8, 0.3 u) rises
    // past the node just after its kink at 2.667 V, vih = max(2.0, 0.6 u)
    // past 3.333 V. The kinks are rail events that split the threshold's
    // step (Rails point 4); on dfd3a03 the crossings within the 4 mV of
    // threshold above each kink (20 mV of rail) were missed and seen at the
    // next scan, up to 117 us late. The rail is il t / C_r and the node
    // has settled (tau 1 ms); the 12-35 ns left is the gate's own draw.
    let (il, c_r) = (0.01, 1e-4);
    let mut bad = Vec::new();
    for (k, slope) in [(0.8, 0.3), (2.0, 0.6)] {
        for j in 1..=19 {
            let level = k + 0.0005 * j as f64;
            let (r1, r2) = (1e3, 1e3 * level / (5.0 - level));
            let devices = vec![psu(0, 1, 0, 5.0, il), capacitor(1, 1, 0, c_r, 0.0), source(2, 3, 0, 5.0), resistor(3, 3, 2, r1), resistor(4, 2, 0, r2), capacitor(5, 2, 0, 1e-6, 0.0), generic_nand(6, 1, 0, [2, 2, 4, 0, 0, 5, 0, 0, 6, 0, 0, 7], 0)];
            let mut rig = Rig::load(&program(8, devices)).unwrap();
            let th = |u: f64| f64::max(k, slope * u);
            let seen = first_below(&mut rig, 2, 1, &th, 0.01, 0.045);
            let want = (rig.v(2) / slope) * c_r / il;
            if !seen.is_some_and(|s| (s - want).abs() <= 1e-7) {
                bad.push(format!("level {level}: seen {seen:?}, want {want:.9e}"));
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

#[test]
fn a_gate_on_a_static_hub_node_is_crossed_where_its_supply_meets_the_node() {
    // F1v23 finding 4, the second half. As above with a Schmitt NAND whose
    // vcc is node 4, a divider off the rail (a static hub partition,
    // nonlinear for the free IC supply): vt- = 0.9 vs / 5 rises past node 2
    // held at `level`. The node's partition is aimed under the rail as for
    // an IC on the rail, with the supply moving on its published line
    // (Rails point 3). On dfd3a03 the threshold stood still between the
    // supply's re-solves: up to 9.3 us late.
    let (il, c_r) = (0.01, 1e-4);
    let mut bad = Vec::new();
    for j in 1..=19 {
        let level = 0.6 + 0.01 * j as f64;
        let (r1, r2) = (1e3, 1e3 * level / (5.0 - level));
        let devices = vec![
            psu(0, 1, 0, 5.0, il),
            capacitor(1, 1, 0, c_r, 0.0),
            source(2, 3, 0, 5.0),
            resistor(3, 3, 2, r1),
            resistor(4, 2, 0, r2),
            capacitor(5, 2, 0, 1e-6, 0.0),
            resistor(6, 1, 4, 100.0),
            resistor(7, 4, 0, 1e4),
            schmitt_on(8, 4, 0, [2, 2, 5, 0, 0, 6, 0, 0, 7, 0, 0, 8], 0),
        ];
        let mut rig = Rig::load(&program(9, devices)).unwrap();
        let th = |vs: f64| 0.9 * vs / 5.0;
        let seen = first_below(&mut rig, 2, 4, &th, 0.01, 0.049);
        // The rail charges c_r at il less the divider's draw: u(t) = il R
        // (1 - e^(-t / (R c_r))), and vs = u 1e4 / R.
        let r = 1e4 + 100.0;
        let u_at = 5.0 * rig.v(2) / 0.9 * r / 1e4;
        let want = -r * c_r * (1.0 - u_at / (il * r)).ln();
        if !seen.is_some_and(|s| (s - want).abs() <= 1e-8) {
            bad.push(format!("level {level}: seen {seen:?}, want {want:.9e}"));
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

#[test]
fn a_powered_gate_on_an_inrush_rc_sees_each_crossing_at_its_instant() {
    // Found under finding 4: {5 V, 0.1 A} ramping 100 uF, an RC off the rail
    // and a powered generic NAND on the rail reading it. A crossing whose
    // owed rail step is past the nudge solves the rail first; the solve
    // leaves the node on its threshold at the instant, a root the new aim
    // cannot accept, so the crossing lands there and then (Rails point 4).
    // On dfd3a03 10 of these 42 crossings were dropped and the reads waited
    // for the next scan, 1.2 to 18.8 us late.
    let (il, c_r, c) = (0.1, 1e-4, 1e-6);
    let vil = |vs: f64| f64::max(0.8, 0.3 * vs);
    let vih = |vs: f64| f64::max(2.0, 0.6 * vs);
    let mut bad = Vec::new();
    let rs = (0..21).map(|k| 3.3e3 + 20.0 * k as f64).chain((0..21).map(|k| 1.4e3 + 10.0 * k as f64));
    for r in rs {
        let devices = vec![psu(0, 1, 0, 5.0, il), capacitor(1, 1, 0, c_r, 0.0), resistor(2, 1, 2, r), capacitor(3, 2, 0, c, 0.0), generic_nand(4, 1, 0, [2, 2, 3, 0, 0, 4, 0, 0, 5, 0, 0, 6], 0)];
        let t_end = 4.9e-3;
        let refr = inrush_reference(il, c_r, 0.0, r, c, 1e-8, t_end);
        let want = [reference_crossing(&refr, |u, v| v - vil(u)), reference_crossing(&refr, |u, v| v - vih(u))];
        let mut rig = Rig::load(&program(7, devices)).unwrap();
        let mut seen = [None, None];
        loop {
            let t = rig.k.next_event_time();
            if t > t_end || rig.hub().state.reg == Regime::Cv {
                break;
            }
            rig.advance(t).unwrap();
            for (k, th) in [&vil as &dyn Fn(f64) -> f64, &vih].iter().enumerate() {
                if seen[k].is_none() && rig.v(2) >= th(rig.v(1)) {
                    seen[k] = Some(t);
                }
            }
        }
        for (s, w) in seen.iter().zip(&want) {
            let ok = match (s, w) {
                (Some(s), Some(w)) => (s - w).abs() <= 1e-9,
                (None, None) => true,
                _ => false,
            };
            if !ok {
                bad.push(format!("R {r}: seen {s:?}, want {w:?}"));
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

#[test]
fn the_cc_segment_holds_the_coupled_closed_form_between_events() {
    // F1v23 finding 5. {5 V, 0.1 A} ramping 100 uF with a fast RC off the
    // rail (10 ohm, 10 uF, tau 0.1 ms, settling to 10 mA). Sampled between
    // events, not only at them (an advance solves the rail). When the
    // segment held the RC's draw where it stood it strayed 2.2e-4 V between
    // the scan events, and 4.8e-6 V with the DriftBound watch solving again
    // before the draw's motion owed RAIL_DRIFT_V; since round 14 (ruling R15
    // (a)) it is the coupled path itself, exact to rounding, and the old
    // bound is what it is held to here.
    let (i, c_r, r, c) = (0.1, 1e-4, 10.0, 1e-5);
    let mut rig = Rig::load(&program(4, vec![psu(0, 1, 0, 5.0, i), capacitor(1, 1, 0, c_r, 0.0), resistor(2, 1, 2, r), capacitor(3, 2, 0, c, 0.0)])).unwrap();
    let mut worst: f64 = 0.0;
    let mut t = rig.k.time();
    while t < 4.5e-3 {
        let next = rig.k.next_event_time().min(4.5e-3);
        let seg = rig.hub().state.cap_seg.clone().expect("CC with a rail capacitor");
        for k in 1..=8 {
            let ts = t + (next - t) * k as f64 / 8.0;
            worst = worst.max((seg.at(ts) - coupled_rc(i, c_r, r, c, ts).0).abs());
        }
        rig.advance(next).unwrap();
        t = next;
    }
    assert!(worst <= rails::RAIL_DRIFT_V, "the segment strayed {worst:e} from the coupled closed form");
}

#[test]
fn a_rollback_in_cc_with_a_driven_rc_and_esr_restores_the_rail() {
    // F1v23's missing rollback case (Rails point 8). CC with 100 uF at esr
    // 0.05; a driven RC; an inverter on a fixed 5 V that livelocks once the
    // switch puts the rail on its input. The failed instant restores the
    // segment, the drive and the drift charge, as a twin that never toggled.
    let p = program(
        9,
        vec![
            psu(0, 1, 0, 5.0, 0.1),
            capacitor(1, 1, 0, 1e-4, 0.05),
            resistor(2, 1, 2, 1e3),
            capacitor(3, 2, 0, 1e-6, 0.0),
            source(4, 8, 0, 5.0),
            oscillator(5, 8, 3, 4, [5, 6, 7]),
            switch(6, 1, 4, false),
            resistor(7, 4, 0, 1e3),
        ],
    );
    let (failed, _) = rollback_against_a_twin(&p, 6, 3e-3, 4e-3, "CC rail capacitor");
    assert_eq!(failed.hub().state.reg, Regime::Cc);
    assert!(failed.hub().state.cap_seg.is_some());
}

// ---- F1r5: the curvature band (Rails point 3) ---------------------------------

/// The LED from the rail to `m`, `r` to ground, solved exactly at rail `u`
/// with the GMIN and node shunt the partition stamps: `(m, the LED's law
/// current)`.
fn led_cathode_at(u: f64, r: f64) -> (f64, f64) {
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let gs = static_part::NODE_RSHUNT_G;
    let f = |m: f64| law.current(u - m) + led::GMIN_D * (u - m) - m / r - gs * m;
    let (mut lo, mut hi) = (-1.0, u.max(0.0) + 1.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let m = 0.5 * (lo + hi);
    (m, law.current(u - m))
}

/// Steps `rig` through every event up to `t_end`, publishing also at eight
/// instants between consecutive events and just before each, and calls
/// `look` at each.
fn sample_between_events(rig: &mut Rig, t_end: f64, mut look: impl FnMut(&Rig, f64)) {
    let mut t = rig.k.time();
    while t < t_end {
        let next = rig.k.next_event_time().min(t_end);
        for k in 1..=8 {
            let ts = if k == 8 { next - 1e-6 * (next - t) } else { t + (next - t) * k as f64 / 8.0 };
            if ts > t {
                rig.advance(ts).unwrap();
                look(rig, ts);
            }
        }
        rig.advance(next).unwrap();
        look(rig, next);
        t = next;
    }
}

#[test]
fn an_led_sitting_on_its_knee_stays_inside_its_curvature_bound() {
    // Coordinator's requirement (1). {5 V, 78 uA} into 1 uF, an LED and 330
    // ohm off the rail: the rail ramps until the LED takes the limit, which
    // is its knee (g = 1 / 2R, 78 uA, where the branch current's curvature
    // peaks), and sits there. At every published instant, and between
    // events too, the first-order correction's true error in the cathode,
    // the resistor's current and the LED's current is within W0's tolerance
    // and within the bound the band was sized from, evaluated over |u - u_p|.
    // A point value of that bound (its curvature at u_p) is exceeded on the
    // way up the knee, so this test fails an estimate that is one.
    let (r, il, c) = (330.0, 78e-6, 1e-6);
    let mut rig = Rig::load(&program(3, vec![psu(0, 1, 0, 5.0, il), capacitor(1, 1, 0, c, 0.0), led(2, 1, 2), resistor(3, 2, 0, r)])).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let p = rig.s().local_index[2].unwrap().0;
    let r_ext = 1.0 / (1.0 / r + static_part::NODE_RSHUNT_G + led::GMIN_D);
    let (mut worst, mut worst_point, mut worst_w0): (f64, f64, f64) = (0.0, 0.0, 0.0);
    let mut at_knee = 0;
    sample_between_events(&mut rig, 40e-3, |rig, t| {
        assert_eq!(rig.hub().state.reg, Regime::Cc, "at {t:e}");
        let (x, s, _, _, u_p) = rig.s().static_solver.hub_solve(p).unwrap();
        let u = rig.v(1);
        let b = (u - u_p).abs();
        let (m, i_led) = led_cathode_at(u, r);
        let errs = [(rig.v(2) - m).abs(), (rig.i(3) - m / r).abs(), (rig.i(2) - i_led).abs()];
        let tols = [1e-4 + 1e-3 * m.abs(), 1e-9 + 1e-3 * (m / r).abs(), 1e-9 + 1e-3 * i_led.abs()];
        for (e, tol) in errs.iter().zip(&tols) {
            worst_w0 = worst_w0.max(e / tol);
        }
        let v = u_p - x[0];
        let dv = 1.0 - s[0];
        let port = curvature::Port::single(law, v, dv, r_ext, dv * (1.0 + r_ext * curvature::slope(&law, v)));
        if (r_ext * curvature::slope(&law, v) - 0.5).abs() < 0.25 {
            at_knee += 1;
        }
        // The sensitivity is exact at the solve (V5 finding 2), so the
        // correction errs by the remainder alone. Below the floors the
        // published values only round.
        let floor = [1e-14, 1e-17, 1e-17];
        let est = |pb: &curvature::PortBound| {
            let e = 0.5 * b * b;
            [pb.k_v * e, (pb.k_a * e).min(pb.k_v * e / r), (pb.g_hi * pb.k_v * e).min((pb.g_hi * dv * dv / law.vtn + pb.k_a) * e)]
        };
        let (over, point) = (est(&port.bound(b)), est(&port.bound(0.0)));
        for k in 0..3 {
            worst = worst.max(errs[k] / (over[k] + floor[k]));
            worst_point = worst_point.max(errs[k] / (point[k] + floor[k]));
        }
    });
    assert!(at_knee > 50, "only {at_knee} samples at the knee");
    assert!(worst_w0 <= 1.0, "an error {worst_w0} of W0's tolerance");
    assert!(worst <= 1.0, "the true error is {worst} of the bound");
    assert!(worst_point > 1.0, "the point estimate held ({worst_point}): this circuit would not catch one");
}

#[test]
fn a_linear_hub_partition_keeps_the_flat_band() {
    // Rails point 3: a linear hub partition's first-order correction is
    // exact up to rounding, so its band is RAIL_RESOLVE_V in every regime:
    // a divider off a CC ramp ({5 V, 0.1 A} into 100 uF), and off a battery.
    let mut rig = Rig::load(&program(3, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0), resistor(2, 1, 2, 1e3), resistor(3, 2, 0, 1e3)])).unwrap();
    let p = rig.s().local_index[2].unwrap().0;
    let mut seen = 0;
    while rig.hub().state.reg == Regime::Cc {
        let t = rig.k.next_event_time();
        rig.advance(t).unwrap();
        assert_eq!(rig.s().static_solver.hub_band(p), Some(rails::RAIL_RESOLVE_V), "at {t:e}");
        seen += 1;
    }
    assert!(seen > 100, "{seen} events in the ramp");
    let rig = Rig::load(&program(3, vec![battery(0, 1, 0, 5.0, 0.5), resistor(1, 1, 2, 1e3), resistor(2, 2, 0, 1e3)])).unwrap();
    let p = rig.s().local_index[2].unwrap().0;
    assert_eq!(rig.s().static_solver.hub_band(p), Some(rails::RAIL_RESOLVE_V));
}

#[test]
fn an_led_far_past_its_knee_solves_again_far_less_often_than_a_millivolt() {
    // {5 V, 0.1 A} ramping 100 uF, an LED and 100 ohm off the rail. From 3 V
    // on the LED carries 10 mA and more, R g > 20, and its curvature band
    // reaches RAIL_RESOLVE_V: the partition solves again about every 25 mV
    // of the 2 V left, where the flat 1 mV band solved it 2,000 times.
    let mut rig = Rig::load(&program(3, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0), led(2, 1, 2), resistor(3, 2, 0, 100.0)])).unwrap();
    let p = rig.s().local_index[2].unwrap().0;
    let (mut solves, mut last, mut narrowest) = (0, f64::NAN, f64::INFINITY);
    while rig.hub().state.reg == Regime::Cc {
        let t = rig.k.next_event_time();
        rig.advance(t).unwrap();
        let (_, _, _, _, u_p) = rig.s().static_solver.hub_solve(p).unwrap();
        if u_p >= 3.0 && u_p.to_bits() != last.to_bits() {
            solves += 1;
            narrowest = narrowest.min(rig.s().static_solver.hub_band(p).unwrap());
        }
        last = u_p;
    }
    assert!(solves <= 2 * (2.0 / rails::RAIL_RESOLVE_V) as usize, "{solves} solves above 3 V");
    assert_eq!(narrowest, rails::RAIL_RESOLVE_V, "the band above 3 V");
}

#[test]
fn in_cc_without_a_rail_capacitor_the_cache_answers_only_within_the_cc_band() {
    // Coordinator, review of fd5f3b8. {5 V, 5 mA} with no rail capacitor:
    // the draws set the rail, so a nonlinear hub partition solves again
    // past RAIL_CC_RESOLVE_V (Rails point 3). An LED and 330 ohm off the
    // rail, and a 1 kohm load the switch puts on it: each toggle moves the
    // rail, and the LED partition's cache entry from before must not answer
    // within 25 mV, or it is handed back past its band and never solves
    // again. Cache on and off agree within LEDs point 7's 1e-6 V.
    let devices = || vec![psu(0, 1, 0, 5.0, 5e-3), led(1, 1, 2), resistor(2, 2, 0, 330.0), switch(3, 1, 3, false), resistor(4, 3, 0, 1e3)];
    let mut on = Rig::load(&program(4, devices())).unwrap();
    let mut off_p = program(4, devices());
    off_p.options.no_solution_cache = true;
    let mut off = Rig::load(&off_p).unwrap();
    assert_eq!(on.hub().state.reg, Regime::Cc);
    for (k, closed) in [true, false, true, false, true].into_iter().enumerate() {
        let t = 1e-3 * (k + 1) as f64;
        on.advance(t).unwrap();
        off.advance(t).unwrap();
        toggle(&mut on, 3, closed).unwrap();
        toggle(&mut off, 3, closed).unwrap();
        assert_eq!(on.hub().state.reg, Regime::Cc);
        close(on.v(1), off.v(1), 1e-6, &format!("the rail after toggle {k}, cache on against off"));
        close(on.v(2), off.v(2), 1e-6, &format!("the LED's cathode after toggle {k}, cache on against off"));
    }
}

/// The flickers of `leaving_cc_with_esr_holds_the_setpoint_without_re_entering`
/// over the given esr values and loads: each re-entry of CC after its exit.
fn exit_flickers(esrs: &[f64], loads: &[f64], c: f64) -> (Vec<String>, usize) {
    let mut flick = Vec::new();
    let mut n = 0;
    for &r in loads {
        for &esr in esrs {
            for ts in [0.5e-3, 1e-3, 2e-3, 3e-3] {
                n += 1;
                let p = program(3, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, c, esr), switch(2, 1, 2, false), resistor(3, 2, 0, r)]);
                let mut rig = Rig::load(&p).unwrap();
                rig.advance(ts).unwrap();
                toggle(&mut rig, 2, true).unwrap();
                let mut exit = None;
                loop {
                    let t = rig.k.next_event_time();
                    if t > 0.03 {
                        break;
                    }
                    rig.advance(t).unwrap();
                    let reg = rig.hub().state.reg;
                    if reg == Regime::Cc && rig.u() > 5.0 - 1e-9 {
                        flick.push(format!("R {r} esr {esr} toggle {ts:e}: CC at u = {} at {t:.9e}", rig.u()));
                        break;
                    }
                    if exit.is_none() && reg == Regime::Cv {
                        exit = Some(t);
                    } else if exit.is_some() && reg == Regime::Cc {
                        flick.push(format!("R {r} esr {esr} toggle {ts:e}: CC again at {t:.9e} after the exit at {:.9e}", exit.unwrap()));
                        break;
                    }
                    if exit.is_some_and(|e| t > e + 1e-3) {
                        break;
                    }
                }
            }
        }
    }
    (flick, n)
}

#[test]
fn leaving_cc_at_a_small_esr_ties_the_capacitors_to_the_limit() {
    // Review of fd5f3b8: the exit's tie (TIE_STEPS) on its own. At esr 1e-4
    // to 1e-3 ohm a last-bit change of a capacitor's charge moves its draw
    // by ulp(5 V) / esr, 9e-12 A and more, past the re-entry band (REACHED_REL
    // of the limit, 1e-12 A), so only the tie's steps keep the PSU in CV.
    let (flick, n) = exit_flickers(&[1e-4, 2e-4, 5e-4, 1e-3], &[75.0, 100.0, 150.0, 330.0], 1e-4);
    assert!(flick.is_empty(), "{} of {n}:\n{}", flick.len(), flick.join("\n"));
}

#[test]
fn a_re_solve_at_the_exit_draws_does_not_re_enter_cc() {
    // Review of fd5f3b8: the re-entry band (REACHED_REL past the limit,
    // Rails point 5) on its own. {5 V, 0.1 A} with 100 uF at esr 0.02-0.1
    // leaves CC after a load step; in the exit's instant an LED branch off
    // the rail is opened and closed again, so its partition solves afresh
    // (cache off) at the draws the exit tied the capacitors to. The fresh
    // Newton solve rounds the draw by a few bits, which the tie never saw:
    // only the band keeps the PSU in CV.
    let mut flick = Vec::new();
    let mut n = 0;
    for r in [75.0, 100.0, 120.0, 150.0, 200.0, 330.0] {
        for esr in [0.02, 0.05, 0.1] {
            for ts in [0.5e-3, 1e-3, 2e-3, 3e-3] {
                n += 1;
                let mut p = program(5, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, esr), switch(2, 1, 2, false), resistor(3, 2, 0, r), led(4, 1, 3), resistor(5, 3, 4, 330.0), switch(6, 4, 0, true)]);
                p.options.no_solution_cache = true;
                let mut rig = Rig::load(&p).unwrap();
                rig.advance(ts).unwrap();
                toggle(&mut rig, 2, true).unwrap();
                let mut t_exit = None;
                while t_exit.is_none() && rig.k.next_event_time() < 0.03 {
                    let t = rig.k.next_event_time();
                    rig.advance(t).unwrap();
                    t_exit = (rig.hub().state.reg == Regime::Cv).then_some(t);
                }
                // A load that draws the limit at 5 V never leaves.
                let Some(t_exit) = t_exit else { continue };
                toggle(&mut rig, 6, false).unwrap();
                toggle(&mut rig, 6, true).unwrap();
                assert_eq!(rig.k.time(), t_exit);
                if rig.hub().state.reg == Regime::Cc {
                    flick.push(format!("R {r} esr {esr} toggle {ts:e}: CC again at the exit {t_exit:.9e}"));
                }
            }
        }
    }
    assert!(flick.is_empty(), "{} of {n}:\n{}", flick.len(), flick.join("\n"));
}

#[test]
fn a_rollback_across_a_cc_exit_restores_the_cache_band() {
    // Review of fd5f3b8 (Rails point 8). {5 V, 5 mA} with no rail capacitor
    // and an LED and 330 ohm off the rail through a switch: CC, where the
    // cache answers within RAIL_CC_RESOLVE_V. One event opens that switch,
    // which leaves CC, and enables a zero-delay oscillator on a fixed 5 V,
    // which livelocks: the rollback restores the CC band with the regime,
    // as the twin that never took the event has it, and both go on alike
    // through the load steps that follow.
    let p = program(
        12,
        vec![
            psu(0, 1, 0, 5.0, 5e-3),
            led(1, 1, 2),
            resistor(2, 2, 3, 330.0),
            switch(3, 3, 0, true),
            source(4, 5, 0, 5.0),
            oscillator(5, 5, 6, 7, [8, 9, 10]),
            switch(6, 5, 7, false),
            resistor(7, 7, 0, 1e3),
            switch(8, 1, 11, false),
            resistor(9, 11, 0, 1e3),
        ],
    );
    let (mut failed, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
    failed.advance(1e-3).unwrap();
    twin.advance(1e-3).unwrap();
    assert_eq!(failed.hub().state.reg, Regime::Cc);
    let cc = rails::BandRule::Flat(rails::RAIL_CC_RESOLVE_V);
    assert_eq!(failed.s().static_solver.hub_cache_rule(), cc);
    let r = failed.k.apply_params_as_event(&[(3, 0, 0.0), (6, 0, 1.0)]);
    failed.k.solver_mut().prepare_readout();
    assert!(matches!(r, Err(KernelError::Livelock { .. })), "the instant livelocks: {r:?}");
    assert_eq!(failed.hub().state.reg, Regime::Cc);
    assert_eq!(failed.s().static_solver.hub_cache_rule(), cc, "the cache band after the rollback");
    assert_eq!(state_digest(failed.s()), state_digest(twin.s()));
    for (k, closed) in [true, false, true].into_iter().enumerate() {
        let t = 2e-3 + 1e-3 * k as f64;
        failed.advance(t).unwrap();
        twin.advance(t).unwrap();
        toggle(&mut failed, 8, closed).unwrap();
        toggle(&mut twin, 8, closed).unwrap();
        assert_eq!(state_digest(failed.s()), state_digest(twin.s()), "after load step {k}");
    }
}

#[test]
fn between_events_the_cc_ramp_stays_inside_each_partitions_band() {
    // Review of fd5f3b8: the band in the rail-event thresholds (Rails point
    // 4). {5 V, 0.1 A} ramping 100 uF, an LED and 330 ohm off the rail. The
    // rail between two events is the segment's own closed form, which a
    // host read at any instant there would publish; each nonlinear hub
    // partition's u_p +- band is a threshold of that segment, so the rail
    // never strays past a partition's band before an event solves it again.
    // Read off the segment between host advances, without advancing.
    let mut rig = Rig::load(&program(3, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0), led(2, 1, 2), resistor(3, 2, 0, 330.0)])).unwrap();
    let p = rig.s().local_index[2].unwrap().0;
    let (mut worst, mut narrow) = (0.0f64, 0);
    let mut t = rig.k.time();
    while rig.hub().state.reg == Regime::Cc {
        let next = rig.k.next_event_time();
        let seg = rig.hub().state.cap_seg.clone().expect("CC with a rail capacitor");
        let (_, _, _, _, u_p) = rig.s().static_solver.hub_solve(p).unwrap();
        let band = rig.s().static_solver.hub_band(p).unwrap();
        if band < rails::RAIL_RESOLVE_V {
            narrow += 1;
        }
        for k in 1..=16 {
            let ts = t + (next - t) * k as f64 / 16.0;
            // An event lands within two reached-widths past a threshold.
            let reach = 4.0 * rails::REACHED_REL * (u_p.abs() + band).max(1.0);
            worst = worst.max((seg.at(ts) - u_p).abs() / (band + reach));
        }
        rig.advance(next).unwrap();
        t = next;
    }
    assert!(narrow > 50, "only {narrow} intervals under a narrow band");
    assert!(worst <= 1.0, "the rail strayed {worst} of a band past its partition's solve");
}

#[test]
fn a_coupled_cc_segment_owes_no_step_at_a_solve_whatever_its_esr() {
    // Review of fd5f3b8 checked the step a rail solve owed the CC segment
    // that froze its closed-form draws (`owed_rail_step`): the drifted
    // charge and the esr offset's share of the draws' own motion. Since round
    // 14 (ruling R15 (a)) the segment is the rail and its partitions' coupled
    // path, the esr offset moving with the draws inside it, so a solve owes
    // nothing: {5 V, 0.1 A} ramping 100 uF at esr 0.02-0.5 with an RC off the
    // rail (1 kohm, 1 uF), the rail a solve finds between events is the
    // segment's own to rounding.
    let mut checked = 0;
    let mut worst = 0.0f64;
    for esr in [0.02, 0.05, 0.1, 0.2, 0.5] {
        let mut rig = Rig::load(&program(3, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, esr), resistor(2, 1, 2, 1e3), capacitor(3, 2, 0, 1e-6, 0.0)])).unwrap();
        let mut t = rig.k.time();
        while rig.hub().state.reg == Regime::Cc && t < 4e-3 {
            let next = rig.k.next_event_time();
            let ts = 0.5 * (t + next);
            let seg = rig.hub().state.cap_seg.clone().expect("CC with a rail capacitor");
            assert!(seg.modes.is_some(), "esr {esr}: the coupled path");
            assert!(rig.s().owed_rail_step(ts).is_none(), "esr {esr}: nothing owed");
            rig.advance(ts).unwrap();
            worst = worst.max((rig.u() - seg.at(ts)).abs());
            checked += 1;
            t = rig.k.time();
        }
    }
    assert!(checked > 50, "{checked} solves between events");
    assert!(worst <= 1e-11, "a solve stepped the rail {worst:e} off its coupled path");
}

/// Two LEDs in series from the rail through `m1` to `m2`, `r` to ground,
/// solved exactly at rail `u` with GMIN and the node shunts: `(m1, m2)`.
fn two_leds_at(u: f64, r: f64) -> (f64, f64) {
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let (gs, gmin) = (static_part::NODE_RSHUNT_G, led::GMIN_D);
    let branch = |v: f64| law.current(v) + gmin * v;
    let bisect = |mut lo: f64, mut hi: f64, f: &dyn Fn(f64) -> f64| {
        for _ in 0..90 {
            let mid = 0.5 * (lo + hi);
            if f(mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    };
    // Given m2, the lower LED carries m2's load, which fixes m1; the upper
    // one's surplus falls as m2 rises.
    let m1_of = |m2: f64| bisect(m2 - 1.0, u + 1.0, &|m1| m2 / r + gs * m2 - branch(m1 - m2));
    let m2 = bisect(-1.0, u + 1.0, &|m2| {
        let m1 = m1_of(m2);
        branch(u - m1) - (m2 / r + gs * m2) - gs * m1
    });
    (m1_of(m2), m2)
}

#[test]
fn two_junctions_in_one_hub_partition_stay_inside_w0_through_the_ramp() {
    // The several-junction bound (Rayleigh's resistance interval, |v'| <= 1;
    // Rails point 3). {5 V, 0.1 A} ramping 100 uF, two LEDs in series and
    // 100 ohm off the rail, both through their knees: at every published
    // instant, between events too, the middle and lower nodes, the
    // resistor's and both LEDs' currents agree with the exact solution at
    // the published rail within W0's tolerance.
    let r = 100.0;
    let mut rig = Rig::load(&program(4, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0), led(2, 1, 2), led(3, 2, 3), resistor(4, 3, 0, r)])).unwrap();
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let p = rig.s().local_index[2].unwrap().0;
    assert_eq!(rig.s().local_index[3].unwrap().0, p, "one partition");
    let (mut worst, mut narrow) = (0.0f64, 0);
    sample_between_events(&mut rig, 4.9e-3, |rig, _| {
        if rig.hub().state.reg != Regime::Cc {
            return;
        }
        if rig.s().static_solver.hub_band(p).is_some_and(|b| b < rails::RAIL_RESOLVE_V) {
            narrow += 1;
        }
        let u = rig.v(1);
        let (m1, m2) = two_leds_at(u, r);
        let (i1, i2) = (law.current(u - m1), law.current(m1 - m2));
        for (got, want, abs) in [(rig.v(2), m1, 1e-4), (rig.v(3), m2, 1e-4), (rig.i(4), m2 / r, 1e-9), (rig.i(2), i1, 1e-9), (rig.i(3), i2, 1e-9)] {
            worst = worst.max((got - want).abs() / (abs + 1e-3 * want.abs()));
        }
    });
    assert!(narrow > 20, "only {narrow} samples under a narrow band");
    assert!(worst <= 1.0, "an error {worst} of W0's tolerance");
}

// ---- F1r6: the curvature band made conservative (Rails point 3) ---------------

fn led_law(ci: usize, anode: usize, cathode: usize, vf: f64, n: f64) -> Device {
    Device { kind: kinds::LED, component_index: ci, pins: vec![pin(anode), pin(cathode)], params: vec![vf, n, 0.02, 0.0, 0.0, 0.0], blob: vec![] }
}

/// Bisection on a residual that falls as its argument rises.
fn falling_root(mut lo: f64, mut hi: f64, f: impl Fn(f64) -> f64) -> f64 {
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// An LED between the rail at `u` and node `m` (from the rail when
/// `from_rail`, into it otherwise), `r` from `m` to ground, solved exactly
/// with the GMIN and node shunt the partition stamps: `(m, the LED's law
/// current)`.
fn led_off_rail_at(law: &led::JunctionLaw, u: f64, r: f64, from_rail: bool) -> (f64, f64) {
    let gs = static_part::NODE_RSHUNT_G;
    let v = |m: f64| if from_rail { u - m } else { m - u };
    let into_m = |m: f64| if from_rail { law.current(v(m)) + led::GMIN_D * v(m) } else { -(law.current(v(m)) + led::GMIN_D * v(m)) };
    let span = u.abs() + 1.0;
    let m = falling_root(-span, span, |m| into_m(m) - m / r - gs * m);
    (m, law.current(v(m)))
}

/// A 74LS00 output held high on the rail at `u` (R_out = u / (4 io_max),
/// io_max 8 mA, ground at 0) into node `o`, an LED from `o` to `c`, `r` from
/// `c` to ground, solved exactly with GMIN and the node shunts: `(o, c, the
/// LED's law current)`.
fn ls00_led_at(law: &led::JunctionLaw, u: f64, r: f64) -> (f64, f64, f64) {
    let gs = static_part::NODE_RSHUNT_G;
    let g_out = 4.0 * 0.008 / u;
    let branch = |v: f64| law.current(v) + led::GMIN_D * v;
    // Given c, the LED carries c's load, which fixes o; o's own balance
    // falls as c rises.
    let o_of = |c: f64| {
        let load = c / r + gs * c;
        c + falling_root(-10.0, 10.0, |v| load - branch(v))
    };
    let c = falling_root(-1.0, u + 1.0, |c| {
        let o = o_of(c);
        g_out * (u - o) - gs * o - (c / r + gs * c)
    });
    let o = o_of(c);
    (o, c, law.current(o - c))
}

/// The worst ratio of a published first-order correction's true error to
/// the bound its band was sized from, over every published instant of
/// `rig` up to `t_end` (between events too, or at `uniform` even steps)
/// where `check` holds. `exact(u)` is the exact solution at rail `u`: the
/// partition's free nodes (in local order) and its first LED's current,
/// whose voltage `led_v(u, nodes)` reads off the nodes. The
/// correction is taken from the exact solution at `u_p` with the published
/// sensitivity, so the solve's own error stays out of it: what remains is
/// the sensitivity's error and the second-order remainder.
#[allow(clippy::too_many_arguments)]
fn correction_against_its_bound(rig: &mut Rig, part: usize, t_end: f64, uniform: Option<usize>, law: led::JunctionLaw, led_v: impl Fn(f64, &[f64]) -> f64, check: impl Fn(&Rig, f64) -> bool, exact: impl Fn(f64) -> (Vec<f64>, f64)) -> (f64, usize, String) {
    let (mut worst, mut seen, mut at) = (0.0f64, 0usize, String::new());
    let mut look = |rig: &Rig, t: f64| {
        let Some((_, s, _, _, u_p)) = rig.s().static_solver.hub_solve(part) else { return };
        let u = rig.v(1);
        if !check(rig, u_p) || !check(rig, u) {
            return;
        }
        let last = curvature::hook::of(rig.s().local_index.iter().position(|e| e.is_some_and(|(q, _)| q == part)).unwrap()).expect("a curvature band was sized");
        assert_eq!(last.u_p.to_bits(), u_p.to_bits(), "the hook holds this solve's bound (cache off)");
        let (du, b) = (u - u_p, (u - u_p).abs());
        let ((xp, _), (xu, iu)) = (exact(u_p), exact(u));
        let lin: Vec<f64> = xp.iter().zip(&s).map(|(x, s)| x + s * du).collect();
        let bj = last.junction(0, b);
        for (l, (got, want)) in lin.iter().zip(&xu).enumerate() {
            let node = rig.s().local_index.iter().position(|e| *e == Some((part, l))).unwrap();
            let bv = last.node_at(node, b);
            let r = (got - want).abs() / (bv + 1e-14);
            if r > worst {
                worst = r;
                at = format!("node {l} t {t:e} u_p {u_p} b {b:e}: off by {:e}, bound {bv:e}", (got - want).abs());
            }
        }
        let v_lin = led_v(u, &lin);
        // The nodes' rounding floor carried through the law: a voltage ulp
        // is 4e-17 A at 0.1 S, more than a bound in a step of 1e-10 V.
        let floor = 0.5 * (law.current(v_lin + 1e-14) - law.current(v_lin - 1e-14)).abs() + 1e-17;
        let r = (law.current(v_lin) - iu).abs() / (bj + floor);
        if r > worst {
            worst = r;
            at = format!("LED t {t:e} u_p {u_p} b {b:e}: off by {:e}, bound {bj:e}", (law.current(v_lin) - iu).abs());
        }
        seen += 1;
    };
    match uniform {
        Some(steps) => {
            for k in 1..=steps {
                let t = t_end * k as f64 / steps as f64;
                rig.advance(t).unwrap();
                look(rig, t);
            }
        }
        None => sample_between_events(rig, t_end, look),
    }
    (worst, seen, at)
}

#[test]
fn the_published_sensitivity_leaves_only_the_bounded_remainder() {
    // V5 finding 2. The correction x + s (u - u_p) is first order only if s
    // is the exact derivative at the solve: a sensitivity through the last
    // Newton iteration's Jacobian, over a forward difference, errs by up to
    // 4.2e-4, a first-order error the second-order bound does not cover.
    // A draining battery (2 or 0.5 ohm, 1e-7 Ah, 100 ohm on the rail) with
    // an LED and 10 kohm or 33 ohm off it, and a -5 V CC ramp with an LED
    // into the rail: at every published instant the correction, from the
    // exact solution at u_p, is within the bound its band was sized from.
    let mut worst = (0.0f64, String::new());
    for (vf, n, r, rb) in [(1.8, 1.0, 1e4, 2.0), (2.0, 1.0, 33.0, 0.5), (2.0, 2.0, 330.0, 0.5)] {
        let law = led::JunctionLaw::at(vf, 0.02, n, None, 25.0);
        let mut p = program(3, vec![generic_battery(0, 1, 0, 5.0, rb, 1e-7), resistor(1, 1, 0, 100.0), led_law(2, 1, 2, vf, n), resistor(3, 2, 0, r)]);
        p.options.no_solution_cache = true;
        let mut rig = Rig::load(&p).unwrap();
        let part = rig.s().local_index[2].unwrap().0;
        let (w, seen, at) = correction_against_its_bound(&mut rig, part, 40e-3, Some(4000), law, |u, x| u - x[0], |_, _| true, |u| {
            let (m, i) = led_off_rail_at(&law, u, r, true);
            (vec![m], i)
        });
        assert!(seen > 3000, "{seen} samples");
        if w > worst.0 {
            worst = (w, format!("battery vf {vf} n {n} R {r} rb {rb}: {at}"));
        }
    }
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let mut p = program(3, vec![psu(0, 1, 0, -5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0), led(2, 2, 1), resistor(3, 0, 2, 330.0)]);
    p.options.no_solution_cache = true;
    let mut rig = Rig::load(&p).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    let (w, seen, at) = correction_against_its_bound(&mut rig, part, 10e-3, None, law, |u, x| x[0] - u, |rig, _| rig.hub().state.reg == Regime::Cc, |u| {
        let (m, i) = led_off_rail_at(&law, u, 330.0, false);
        (vec![m], i)
    });
    assert!(seen > 100, "{seen} samples");
    if w > worst.0 {
        worst = (w, format!("-5 V ramp: {at}"));
    }
    eprintln!("sensitivity: worst {:.4} of the bound at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "a correction {} of its bound at {}", worst.0, worst.1);
}

#[test]
fn an_ic_driven_led_stays_inside_its_curvature_bound() {
    // V5 finding 3, ruling R4 (a). A 74LS00 output held high on a CC ramp
    // ({5 V, 0.1 A} into 100 uF) drives an LED and 330 ohm. Its R_out = vs /
    // (4 io_max) moves with the rail, so the partition's response to the
    // rail is curved by the driver as well as by the junction: far past the
    // knee the driver's curvature is all of it. At every published instant
    // with the gate powered, the first-order correction of both nodes and of
    // the LED's current is within the bound its band was sized from.
    let mut worst = (0.0f64, String::new());
    for vf in [2.0, 4.0] {
        let law = led::JunctionLaw::at(vf, 0.02, 2.0, None, 25.0);
        let mut p = program(15, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0), ls00(2, 1, 0, [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], 1e-8, 0), led_law(3, 4, 14, vf, 2.0), resistor(4, 14, 0, 330.0)]);
        p.options.no_solution_cache = true;
        let mut rig = Rig::load(&p).unwrap();
        let part = rig.s().local_index[4].unwrap().0;
        assert_eq!(rig.s().local_index[14].unwrap(), (part, 1), "o then c, one partition");
        let powered = |u: f64| u >= 0.9 * 4.75 && u <= 1.1 * 5.25;
        let (w, seen, at) = correction_against_its_bound(&mut rig, part, 10e-3, None, law, |_, x| x[0] - x[1], |rig, u| rig.hub().state.reg == Regime::Cc && powered(u), |u| {
            let (o, c, i) = ls00_led_at(&law, u, 330.0);
            (vec![o, c], i)
        });
        assert!(seen > 50, "{seen} samples");
        if w > worst.0 {
            worst = (w, format!("vf {vf}: {at}"));
        }
    }
    eprintln!("IC-driven LED: worst {:.4} of the bound at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "a correction {} of its bound at {}", worst.0, worst.1);
}

#[test]
fn two_leds_in_parallel_stay_inside_the_several_junction_bound() {
    // V5 finding 1: the several-junction bound (Rayleigh's resistance
    // interval, |a| bounded by the sensitivity network's terminals) had no
    // test that the single-junction formula per junction (R from the
    // Jacobian less its own slope, a from its sensitivity at the solve) would
    // fail. Two identical LEDs in parallel, `r` from the rail: each sees the
    // other in its rest, and as the rail falls the other's slope falls with
    // its own, so each one's R and a grow past their values at the solve,
    // most where R g is near 1 for each. The per-junction formula then
    // undershoots the remainder below u_p (about 2 % at R g = 1, the band
    // the junction currents allow). {5 V, 0.1 A} ramping 100 uF, and a
    // draining battery; at every published instant the correction of the
    // shared node and an LED's current is within the bound.
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let exact = |u: f64, r: f64| {
        let gs = static_part::NODE_RSHUNT_G;
        let m = falling_root(-1.0, u.abs() + 1.0, |m| (u - m) / r - 2.0 * (law.current(m) + led::GMIN_D * m) - gs * m);
        (vec![m], law.current(m))
    };
    let mut worst = (0.0f64, String::new());
    for r in [100.0, 330.0] {
        for (k, base) in [(0, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0)]), (1, vec![generic_battery(0, 1, 0, 5.0, 0.5, 1e-7), resistor(1, 1, 0, 100.0)])] {
            let mut devs = base;
            devs.extend([resistor(2, 1, 2, r), led(3, 2, 0), led(4, 2, 0)]);
            let mut p = program(3, devs);
            p.options.no_solution_cache = true;
            let mut rig = Rig::load(&p).unwrap();
            let part = rig.s().local_index[2].unwrap().0;
            let (t_end, uniform) = if k == 0 { (6e-3, None) } else { (40e-3, Some(4000)) };
            let (w, seen, at) = correction_against_its_bound(&mut rig, part, t_end, uniform, law, |_, x| x[0], |rig, _| k == 1 || rig.hub().state.reg == Regime::Cc, |u| exact(u, r));
            assert!(seen > 100, "{seen} samples");
            if w > worst.0 {
                worst = (w, format!("R {r} {}: {at}", if k == 0 { "ramp" } else { "battery" }));
            }
        }
    }
    eprintln!("parallel LEDs: worst {:.4} of the bound at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "a correction {} of its bound at {}", worst.0, worst.1);
}


/// The rail and the far node `(v1, v2)` of a PSU in CC at `i` on a rail
/// capacitor `c_r`, loaded by `r_l` and by `r` into `c`, `t` after `(v10,
/// v20)`: the coupled linear closed form `x_inf + e^(A t) (x0 - x_inf)`, the
/// exponential by Sylvester's formula. At rest `c` takes nothing, so both
/// nodes settle at `i r_l`.
fn cc_two_caps(i: f64, c_r: f64, r_l: f64, r: f64, c: f64, (v10, v20): (f64, f64), t: f64) -> (f64, f64) {
    let a = [[-(1.0 / r_l + 1.0 / r) / c_r, 1.0 / (r * c_r)], [1.0 / (r * c), -1.0 / (r * c)]];
    let (tr, det) = (a[0][0] + a[1][1], a[0][0] * a[1][1] - a[0][1] * a[1][0]);
    let disc = (tr * tr / 4.0 - det).sqrt();
    let (l1, l2) = (tr / 2.0 + disc, tr / 2.0 - disc);
    let (e1, e2) = ((l1 * t).exp(), (l2 * t).exp());
    let m = |row: usize, col: usize| {
        let id = if row == col { 1.0 } else { 0.0 };
        e1 * (a[row][col] - l2 * id) / (l1 - l2) + e2 * (a[row][col] - l1 * id) / (l2 - l1)
    };
    let x_inf = i * r_l;
    let d = (v10 - x_inf, v20 - x_inf);
    (x_inf + m(0, 0) * d.0 + m(0, 1) * d.1, x_inf + m(1, 0) * d.0 + m(1, 1) * d.1)
}

/// INF1 (FX45v's default-mode failure): the bench PSU at 5 V and 20 mA with
/// a 1 uF rail capacitor and a 74LS00 on the rail; switching in 50 ohm at 8
/// ms sends it to CC, and the rail falls through the IC's power floor
/// (4.275 V).
#[test]
fn a_cc_rail_capacitor_falling_through_an_ics_power_floor_follows_the_coupled_closed_form() {
    // The power-window event lands the rail tens of nV under the floor,
    // where the IC is unpowered. The sensitivity residual at u + RAIL_DELTA
    // judged it powered, so each high output took `1/R_out` over its
    // shunt-only node: a rail slope of about -2.4e14 S each, a CC segment
    // with k < 0, and "node 1: published voltage -inf". Held at u, the
    // unpowered gate draws nothing and the rail is the linear closed form.
    let mut rig = Rig::load(&program(9, vec![
        psu(0, 1, 0, 5.0, 0.02),
        resistor(1, 1, 2, 100.0),
        capacitor(2, 2, 0, 1e-5, 0.0),
        switch(3, 1, 3, false),
        resistor(4, 3, 0, 50.0),
        ls00(5, 1, 0, [2, 2, 5, 0, 0, 6, 0, 0, 7, 0, 0, 8], 10e-9, 0),
        capacitor(6, 1, 0, 1e-6, 0.0),
    ]))
    .unwrap();
    rig.advance(0.008).unwrap();
    let x0 = (rig.v(1), rig.v(2));
    rig.k.apply_params_as_event(&[(3, 0, 1.0)]).unwrap();
    rig.k.solver_mut().prepare_readout();
    for k in 1..=60 {
        let t = 0.008 + 1e-4 * k as f64;
        rig.advance(t).unwrap_or_else(|e| panic!("advance to {t}: {e:?}"));
        let (v1, v2) = cc_two_caps(0.02, 1e-6, 50.0, 100.0, 1e-5, x0, t - 0.008);
        close(rig.v(1), v1, 1e-4, &format!("the rail at {t}"));
        close(rig.v(2), v2, 1e-4, &format!("the far node at {t}"));
        close(rig.i(0), -0.02, 1e-12, &format!("the PSU current at {t}"));
    }
}

/// INF1's circuit with the gate's power floor crossed by a junction hub
/// partition, which takes its sensitivity on the exact path (`explicit_u`):
/// a 74LS00 output held high drives an LED and 330 ohm. Before the floor
/// the LED's current is the exact solution to W0's tolerance; once the rail
/// falls under the floor the gate drives nothing and both nodes follow the
/// coupled closed form from there.
#[test]
fn an_ic_driven_led_partition_across_its_power_floor_keeps_the_exact_sensitivity_finite() {
    // The partition's stamps and its IC rail terms judge the gate's power at
    // the solve's u. Judged a part in 1e6 above it, a rail just under the
    // floor gives the unpowered solution the powered gate's slopes: a draw
    // slope of about 2 mS the rail does not have, against the 20 mS load.
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let mut rig = Rig::load(&program(9, vec![
        psu(0, 1, 0, 5.0, 0.06),
        resistor(1, 1, 2, 100.0),
        capacitor(2, 2, 0, 1e-5, 0.0),
        switch(3, 1, 3, false),
        resistor(4, 3, 0, 50.0),
        ls00(5, 1, 0, [0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0], 10e-9, 0),
        capacitor(6, 1, 0, 1e-6, 0.0),
        led(7, 5, 6),
        resistor(8, 6, 0, 330.0),
    ]))
    .unwrap();
    let part = rig.s().local_index[5].unwrap().0;
    assert_eq!(rig.s().local_index[6].unwrap(), (part, 1), "o then c, one partition");
    rig.advance(0.008).unwrap();
    let floor = 0.9 * 4.75;
    rig.k.apply_params_as_event(&[(3, 0, 1.0)]).unwrap();
    rig.k.solver_mut().prepare_readout();
    let mut t = 0.008;
    let mut powered_samples = 0;
    while rig.v(1) >= floor {
        let u = rig.v(1);
        let (_, _, i) = ls00_led_at(&law, u, 330.0);
        close(rig.i(7), i, curvature::A_ABS + curvature::REL * i.abs(), &format!("the powered LED at {t}, rail {u}"));
        powered_samples += 1;
        t += 2e-7;
        assert!(t < 0.0081, "the rail never reached the floor");
        rig.advance(t).unwrap_or_else(|e| panic!("advance to {t}: {e:?}"));
    }
    assert!(powered_samples > 5, "{powered_samples} samples above the floor");
    let (t0, x0) = (t, (rig.v(1), rig.v(2)));
    for k in 1..=60 {
        let t = t0 + 1e-4 * k as f64;
        rig.advance(t).unwrap_or_else(|e| panic!("advance to {t}: {e:?}"));
        let (v1, v2) = cc_two_caps(0.06, 1e-6, 50.0, 100.0, 1e-5, x0, t - t0);
        close(rig.v(1), v1, 1e-4, &format!("the rail at {t}"));
        close(rig.v(2), v2, 1e-4, &format!("the far node at {t}"));
        close(rig.i(7), 0.0, curvature::A_ABS, &format!("the unpowered LED at {t}"));
        close(rig.i(0), -0.06, 1e-12, &format!("the PSU current at {t}"));
    }
}

/// The rail's direct draw across a power floor (`Hub::direct_at`): INF1's
/// circuit at 60 mA, 1 kohm into the 10 uF, with the 74LS00's 1y driven
/// high onto a node a 0 V source holds, so the gate's output branch runs
/// from the rail to a fixed node and belongs to no partition. Powered it
/// draws `vs / R_out = 4 io_max` = 32 mA; under the floor nothing, and the
/// rail follows the coupled closed form. The start-up inrush crosses the
/// floor rising with 24 mA to spare over the gate's draw: with less, the
/// gate's own draw turns the rail back under the floor at once
/// (`w1f-power-gate-pingpong`).
#[test]
fn a_direct_ic_draw_across_its_power_floor_follows_the_coupled_closed_form() {
    // The draw's slope is a difference at u + RAIL_DELTA. With the gate's
    // power judged there too, a rail the power event landed just under the
    // floor differenced 0 against 32 mA: 3.2e4 S of slope the rail does not
    // have, which held it on the floor.
    let mut rig = Rig::load(&program(9, vec![
        psu(0, 1, 0, 5.0, 0.06),
        resistor(1, 1, 2, 1000.0),
        capacitor(2, 2, 0, 1e-5, 0.0),
        switch(3, 1, 3, false),
        resistor(4, 3, 0, 50.0),
        ls00(5, 1, 0, [0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0], 10e-9, 0),
        capacitor(6, 1, 0, 1e-6, 0.0),
        source(7, 4, 0, 0.0),
    ]))
    .unwrap();
    assert!(rig.s().local_index[4].is_none(), "1y on a fixed node");
    rig.advance(0.008).unwrap();
    close(rig.i(7), 0.032, 1e-9, "the powered gate's draw");
    let floor = 0.9 * 4.75;
    rig.k.apply_params_as_event(&[(3, 0, 1.0)]).unwrap();
    rig.k.solver_mut().prepare_readout();
    let mut t = 0.008;
    while rig.v(1) >= floor {
        t += 2e-7;
        assert!(t < 0.0081, "the rail never reached the floor");
        rig.advance(t).unwrap_or_else(|e| panic!("advance to {t}: {e:?}"));
    }
    let (t0, x0) = (t, (rig.v(1), rig.v(2)));
    for k in 1..=60 {
        let t = t0 + 1e-4 * k as f64;
        rig.advance(t).unwrap_or_else(|e| panic!("advance to {t}: {e:?}"));
        let (v1, v2) = cc_two_caps(0.06, 1e-6, 50.0, 1000.0, 1e-5, x0, t - t0);
        close(rig.v(1), v1, 1e-4, &format!("the rail at {t}"));
        close(rig.v(2), v2, 1e-4, &format!("the far node at {t}"));
        close(rig.i(7), 0.0, 1e-9, &format!("the unpowered gate's draw at {t}"));
        close(rig.i(0), -0.06, 1e-12, &format!("the PSU current at {t}"));
    }
}

/// The forward-difference sensitivity of a closed-form and of a stepped hub
/// partition across a power floor: the direct-draw circuit with 1y driving
/// 1 kohm into 1 uF (one capacitor: closed form), or on through 1 kohm into
/// another 1 uF (two: stepped). Under the floor the gate drives nothing,
/// the partition draws nothing from the rail, and the rail follows the
/// coupled closed form.
#[test]
fn an_rc_driven_by_an_ic_across_its_power_floor_follows_the_coupled_closed_form() {
    for stepped in [false, true] {
        let mut devs = vec![
            psu(0, 1, 0, 5.0, 0.06),
            resistor(1, 1, 2, 1000.0),
            capacitor(2, 2, 0, 1e-5, 0.0),
            switch(3, 1, 3, false),
            resistor(4, 3, 0, 50.0),
            ls00(5, 1, 0, [0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0], 10e-9, 0),
            capacitor(6, 1, 0, 1e-6, 0.0),
            resistor(7, 4, 5, 1000.0),
            capacitor(8, 5, 0, 1e-6, 0.0),
        ];
        if stepped {
            devs.extend([resistor(9, 5, 6, 1000.0), capacitor(10, 6, 0, 1e-6, 0.0)]);
        }
        let what = if stepped { "stepped" } else { "closed form" };
        let mut rig = Rig::load(&program(9, devs)).unwrap();
        let part = rig.s().local_index[5].unwrap().0;
        assert_eq!(rig.s().local_index[4].unwrap().0, part, "{what}: 1y and the RC, one partition");
        rig.advance(0.008).unwrap();
        let floor = 0.9 * 4.75;
        rig.k.apply_params_as_event(&[(3, 0, 1.0)]).unwrap();
        rig.k.solver_mut().prepare_readout();
        let mut t = 0.008;
        while rig.v(1) >= floor {
            t += 2e-7;
            assert!(t < 0.0081, "{what}: the rail never reached the floor");
            rig.advance(t).unwrap_or_else(|e| panic!("{what}: advance to {t}: {e:?}"));
        }
        let (t0, x0) = (t, (rig.v(1), rig.v(2)));
        // Unpowered, the RC keeps its charge: 1y carries no current, so it
        // reads the first capacitor, and two equal capacitors share theirs.
        let charge = if stepped { 0.5 * (rig.v(5) + rig.v(6)) } else { rig.v(5) };
        for k in 1..=60 {
            let t = t0 + 1e-4 * k as f64;
            rig.advance(t).unwrap_or_else(|e| panic!("{what}: advance to {t}: {e:?}"));
            let (v1, v2) = cc_two_caps(0.06, 1e-6, 50.0, 1000.0, 1e-5, x0, t - t0);
            close(rig.v(1), v1, 1e-4, &format!("{what}: the rail at {t}"));
            close(rig.v(2), v2, 1e-4, &format!("{what}: the far node at {t}"));
            close(rig.i(0), -0.06, 1e-12, &format!("{what}: the PSU current at {t}"));
            close(rig.v(4), rig.v(5), 1e-4, &format!("{what}: 1y at {t}"));
            let held = if stepped { 0.5 * (rig.v(5) + rig.v(6)) } else { rig.v(5) };
            close(held, charge, 1e-4, &format!("{what}: the RC's charge at {t}"));
        }
    }
}

#[test]
fn a_floating_pull_aimed_at_the_rail_inside_an_led_partition_stays_inside_its_bound() {
    // Round 6's untested term: a 74LS00's open 1a (TTL: 1 Mohm) pulled high,
    // to its vcc on the rail, is the only thing driving an LED to ground.
    // The pull stamps a conductance to the datum and an injection `g u`, so
    // the residual's rail column misses it and `explicit_u` adds `-g` to
    // its row. A {5 V, 0.1 A} CC ramp into 100 uF; at every published
    // instant with the gate powered, the correction of the node and of the
    // LED's current is within the bound its band was sized from, against
    // the exact (bisection) solution `g (u - m) = I(m) + GMIN m + gs m`.
    let t_end = 10e-3;
    let open_hash = (0u32..).find(|&h| (0..=1).all(|b| digital::floating::floating_high(h, 0.01 * b as f64, true))).unwrap();
    let mut gate = ls00(2, 1, 0, [3, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1e-8, 0);
    gate.pins[2] = PinRecord { node: 3, flags: crate::program::PIN_FLAG_OPEN, open_hash, dead_hash: 0 };
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let g = 1e-6;
    let exact = |u: f64| {
        let gs = static_part::NODE_RSHUNT_G;
        let m = falling_root(-1.0, u + 1.0, |m| g * (u - m) - law.current(m) - led::GMIN_D * m - gs * m);
        (vec![m], law.current(m))
    };
    let mut p = program(5, vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0), gate, led(3, 3, 0)]);
    p.options.no_solution_cache = true;
    let mut rig = Rig::load(&p).unwrap();
    let part = rig.s().local_index[3].unwrap().0;
    assert!(rig.s().static_solver.hub_solve(part).is_some(), "the pulled LED is a hub partition");
    let powered = |u: f64| u >= 0.9 * 4.75 && u <= 1.1 * 5.25;
    let (w, seen, at) = correction_against_its_bound(&mut rig, part, t_end, None, law, |_, x| x[0], |rig, u| rig.hub().state.reg == Regime::Cc && powered(u), exact);
    eprintln!("pulled LED: worst {w:.4} of the bound over {seen} samples at {at}");
    assert!(seen > 50, "{seen} samples");
    assert!(w <= 1.0, "a correction {w} of its bound at {at}");
}

#[test]
fn branches_in_series_with_junctions_stay_inside_their_bound() {
    // F1r7's tightening: a resistor whose end node holds nothing but
    // junctions carries their current (KCL), so it errs by no more than
    // they do; the partition-wide branch bound, the sum of every curvature
    // injection, had held a resistor behind an LED that is off to the
    // driver's whole curvature. A 74LS00 output held high on a {5 V, 0.1 A}
    // CC ramp into 100 uF drives three such branches: 330 ohm into an LED,
    // 330 ohm into a reversed (off) LED, and 150 ohm into two LEDs in
    // parallel; and, as the control the rule must not take, 330 ohm into a
    // reversed LED and 1 kohm to ground, whose node holds a second branch.
    // At every published instant with the gate powered, each resistor's
    // corrected current and every node are within the bound, against the
    // exact (bisection) solution with GMIN and the shunts.
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let (r1, r2, r3, r4, r5) = (330.0, 330.0, 150.0, 330.0, 1000.0);
    let gs = static_part::NODE_RSHUNT_G;
    let f = |v: f64| law.current(v) + led::GMIN_D * v;
    let root = |mut lo: f64, mut hi: f64, g: &dyn Fn(f64) -> f64| {
        for _ in 0..90 {
            let mid = 0.5 * (lo + hi);
            if g(mid) > 0.0 { lo = mid } else { hi = mid }
        }
        0.5 * (lo + hi)
    };
    let cs = |o: f64| {
        let c1 = root(-1.0, o + 1.0, &|c| (o - c) / r1 - f(c) - gs * c);
        let c2 = root(-1.0, o + 1.0, &|c| (o - c) / r2 + f(-c) - gs * c);
        let c3 = root(-1.0, o + 1.0, &|c| (o - c) / r3 - 2.0 * f(c) - gs * c);
        let c4 = root(-1.0, o + 1.0, &|c| (o - c) / r4 + f(-c) - gs * c - c / r5);
        (c1, c2, c3, c4)
    };
    let exact = |u: f64| {
        let g_out = 4.0 * 0.008 / u;
        let o = root(-1.0, u + 1.0, &|o| {
            let (c1, c2, c3, c4) = cs(o);
            g_out * (u - o) - gs * o - (o - c1) / r1 - (o - c2) / r2 - (o - c3) / r3 - (o - c4) / r4
        });
        let (c1, c2, c3, c4) = cs(o);
        [o, c1, c2, c3, c4]
    };
    let mut p = program(8, vec![
        psu(0, 1, 0, 5.0, 0.1),
        capacitor(1, 1, 0, 1e-4, 0.0),
        ls00(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1e-8, 0),
        resistor(3, 2, 3, r1),
        led(4, 3, 0),
        resistor(5, 2, 4, r2),
        led(6, 0, 4),
        resistor(7, 2, 5, r3),
        led(8, 5, 0),
        led(9, 5, 0),
        resistor(10, 2, 6, r4),
        led(11, 0, 6),
        resistor(12, 6, 0, r5),
    ]);
    p.options.no_solution_cache = true;
    let mut rig = Rig::load(&p).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    let local: Vec<usize> = [2, 3, 4, 5, 6].iter().map(|&n| {
        let (q, l) = rig.s().local_index[n].unwrap();
        assert_eq!(q, part, "one partition");
        l
    }).collect();
    let powered = |u: f64| u >= 0.9 * 4.75 && u <= 1.1 * 5.25;
    let (mut worst, mut at, mut seen) = (0.0f64, String::new(), 0usize);
    sample_between_events(&mut rig, 10e-3, |rig, t| {
        let Some((_, s, _, _, u_p)) = rig.s().static_solver.hub_solve(part) else { return };
        let u = rig.v(1);
        if rig.hub().state.reg != Regime::Cc || !powered(u_p) || !powered(u) {
            return;
        }
        let last = curvature::hook::of(2).expect("a curvature band was sized");
        assert_eq!(last.u_p.to_bits(), u_p.to_bits(), "the hook holds this solve's bound (cache off)");
        let (du, b) = (u - u_p, (u - u_p).abs());
        let (xp, xu) = (exact(u_p), exact(u));
        let lin: Vec<f64> = (0..5).map(|k| xp[k] + s[local[k]] * du).collect();
        for k in 0..5 {
            let bv = last.node_at([2, 3, 4, 5, 6][k], b);
            let r = (lin[k] - xu[k]).abs() / (bv + 1e-14);
            if r > worst {
                worst = r;
                at = format!("node {} t {t:e} b {b:e}: off by {:e}, bound {bv:e}", [2, 3, 4, 5, 6][k], (lin[k] - xu[k]).abs());
            }
        }
        for (dev, (ka, kb), r, series) in [(3, (0, 1), r1, true), (5, (0, 2), r2, true), (7, (0, 3), r3, true), (10, (0, 4), r4, false), (12, (4, 5), r5, false)] {
            let (bb, in_series) = last.branch(dev, b);
            assert_eq!(in_series, series, "resistor {dev} in series with junctions");
            let v = |x: &[f64], k: usize| if k < 5 { x[k] } else { 0.0 };
            let err = ((v(&lin, ka) - v(&lin, kb)) / r - (v(&xu, ka) - v(&xu, kb)) / r).abs();
            let rr = err / (bb + 1e-14 / r);
            if rr > worst {
                worst = rr;
                at = format!("resistor {dev} t {t:e} b {b:e}: off by {err:e}, bound {bb:e}");
            }
        }
        seen += 1;
    });
    eprintln!("series branches: worst {worst:.4} of the bound over {seen} samples at {at}");
    assert!(seen > 50, "{seen} samples");
    assert!(worst <= 1.0, "a correction {worst} of its bound at {at}");
}

// ---- Rails point 3: chip outputs and supplies on the rail (R7 (a)) -----------

/// Newton on `f` with a central-difference Jacobian, each step held to 0.2
/// V, until three steps in a row move under 1e-13 V.
fn newton_fd(x0: Vec<f64>, f: &dyn Fn(&[f64]) -> Vec<f64>) -> Vec<f64> {
    let n = x0.len();
    let mut x = x0;
    let mut small = 0;
    for _ in 0..400 {
        let r = f(&x);
        let mut j = vec![0.0; n * n];
        for c in 0..n {
            let h = 1e-7 * (1.0 + x[c].abs());
            let (mut xp, mut xm) = (x.clone(), x.clone());
            xp[c] += h;
            xm[c] -= h;
            let (rp, rm) = (f(&xp), f(&xm));
            for k in 0..n {
                j[k * n + c] = (rp[k] - rm[k]) / (2.0 * h);
            }
        }
        let mut dx: Vec<f64> = r.iter().map(|v| -v).collect();
        crate::lu::lu_factor(j, n).unwrap().solve(&mut dx).unwrap();
        let m = dx.iter().fold(0.0f64, |a, b| a.max(b.abs()));
        let scale = if m > 0.2 { 0.2 / m } else { 1.0 };
        for k in 0..n {
            x[k] += scale * dx[k];
        }
        if m < 1e-13 {
            small += 1;
            if small >= 3 {
                break;
            }
        }
    }
    x
}

/// A gate's output conductance at supply `vs`: `1 / R_out`, `R_out = vs / (4
/// io_max)` held to [50, 1000] ohm (spec §1.3).
fn gate_g(vs: f64, io: f64) -> f64 {
    1.0 / (vs / (4.0 * io)).clamp(50.0, 1000.0)
}

/// A 74LS00-shaped gate with output drive `io` and supply window `[lo, hi]`.
fn gate_io(ci: usize, vcc: usize, gnd: usize, logic: [usize; 12], io: f64, lo: f64, hi: f64) -> Device {
    let mut d = ls00(ci, vcc, gnd, logic, 1e-8, 0);
    d.params[ic_param::IO_MAX] = io;
    d.params[ic_param::VCC_MIN] = lo;
    d.params[ic_param::VCC_MAX] = hi;
    d
}

/// V6's supplies: a {5 V, 0.1 A} CC ramp into 100 uF, or a 5 V generic cell
/// (0.5 ohm, 1e-7 Ah) draining through 100 ohm.
fn d7_supply(ramp: bool) -> Vec<Device> {
    if ramp {
        vec![psu(0, 1, 0, 5.0, 0.1), capacitor(1, 1, 0, 1e-4, 0.0)]
    } else {
        vec![generic_battery(0, 1, 0, 5.0, 0.5, 1e-7), resistor(1, 1, 0, 100.0)]
    }
}

/// `p` with the solution cache off, so each hub solve sizes its band afresh
/// for the bound hook.
fn no_cache(mut p: Program) -> Program {
    p.options.no_solution_cache = true;
    p
}

/// A board for [`d7_published_against_exact`]: its free nodes, resistors
/// `(device, a, b, r)` and LEDs `(device, anode, cathode)`, the exact
/// solution at rail `u` (the nodes, in `nodes` order, and the draw from the
/// rail), and the supply window `(node, lo)` the sampling stays above.
struct D7Board {
    name: String,
    p: Program,
    nodes: Vec<usize>,
    res: Vec<(usize, usize, usize, f64)>,
    leds: Vec<(usize, usize, usize)>,
    exact: Box<dyn Fn(f64) -> (Vec<f64>, f64)>,
    supply: (usize, f64, f64),
    /// Rail capacitors charged at load (`Rig::load_carrying`).
    carry: Vec<(usize, f64)>,
}

/// The worst ratio, over every published instant of `b` (between events
/// on the CC ramp while in CC; 4,000 even steps on the battery, until the
/// chip's supply nears its floor), of a published node voltage's, resistor
/// or LED current's or draw's error against the exact solution at the
/// published rail to W0's tolerance; and of the first-order correction's
/// true error (from the exact solution at `u_p`, the published `s`) to the
/// bound its band was sized from (infinite where none was). `(W0 ratio, bound
/// ratio, samples, solves, where)`.
fn d7_published_against_exact(b: &D7Board, ramp: bool) -> (f64, f64, usize, usize, String) {
    d7_published_against_exact_at(b, ramp, &[])
}

/// [`d7_published_against_exact`], publishing first at each of `early`,
/// instants that fall closer to a solve than any event does.
fn d7_published_against_exact_at(b: &D7Board, ramp: bool, early: &[f64]) -> (f64, f64, usize, usize, String) {
    let mut rig = Rig::load_carrying(&b.p, &b.carry).unwrap();
    let part = rig.s().local_index[b.nodes[0]].unwrap().0;
    let local: Vec<usize> = b.nodes.iter().map(|&n| {
        let (q, l) = rig.s().local_index[n].unwrap();
        assert_eq!(q, part, "{}: one partition", b.name);
        l
    }).collect();
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let (mut w0, mut wb, mut seen, mut solves, mut at) = (0.0f64, 0.0f64, 0usize, 0usize, String::new());
    let mut last_up = f64::NAN;
    let mut unbounded = 0usize;
    let (sup, lo, hi) = b.supply;
    let mut look = |rig: &Rig, t: f64| -> bool {
        let u = rig.v(1);
        let v_sup = if sup == 1 { u } else { rig.v(sup) };
        if v_sup < lo + 0.02 && !ramp {
            return false;
        }
        let Some((_, s, i_p, g_p, u_p)) = rig.s().static_solver.hub_solve(part) else { return true };
        if ramp && rig.hub().state.reg != Regime::Cc {
            return true;
        }
        let (xp, dp) = (b.exact)(u_p);
        let sup_p = if sup == 1 { u_p } else { xp[b.nodes.iter().position(|&n| n == sup).unwrap()] };
        if !(v_sup >= lo && v_sup <= hi && sup_p >= lo && sup_p <= hi) {
            return true;
        }
        if u_p.to_bits() != last_up.to_bits() {
            solves += 1;
            last_up = u_p;
        }
        let (du, bw) = (u - u_p, (u - u_p).abs());
        let (xu, du_exact) = (b.exact)(u);
        let lin: Vec<f64> = (0..b.nodes.len()).map(|k| xp[k] + s[local[k]] * du).collect();
        let last = curvature::hook::of(b.nodes[0]).filter(|l| l.u_p.to_bits() == u_p.to_bits());
        // Every solve of a curved partition sizes a bound, and the hook holds
        // this board's alone: a sample without one would hold nothing.
        unbounded += usize::from(last.as_ref().is_none_or(|l| !l.bounded()));
        let at_node = |x: &[f64], uu: f64, node: usize| if node == 0 { 0.0 } else if node == 1 { uu } else { x[b.nodes.iter().position(|&n| n == node).unwrap()] };
        let mut note = |what: String, published: f64, corrected: f64, want: f64, bound: f64, tol: f64| {
            let r0 = (published - want).abs() / tol;
            if r0 > w0 {
                w0 = r0;
                at = format!("{what} t {t:e} u {u:.6} u_p {u_p:.6}: published {published:e}, exact {want:e}, tolerance {tol:e}");
            }
            wb = wb.max((corrected - want).abs() / bound);
        };
        for (k, &n) in b.nodes.iter().enumerate() {
            let bv = last.as_ref().map_or(f64::INFINITY, |l| l.node_at(n, bw));
            note(format!("node {n}"), rig.v(n), lin[k], xu[k], bv + 1e-14 * (1.0 + xu[k].abs()), 1e-4 + 1e-3 * xu[k].abs());
        }
        for &(dev, na, nb, r) in &b.res {
            let want = (at_node(&xu, u, na) - at_node(&xu, u, nb)) / r;
            let corrected = (at_node(&lin, u, na) - at_node(&lin, u, nb)) / r;
            let bb = last.as_ref().map_or(f64::INFINITY, |l| l.branch(dev, bw).0);
            note(format!("resistor {dev}"), rig.i(dev), corrected, want, bb + 2e-14 * (1.0 + u.abs()) / r, 1e-9 + 1e-3 * want.abs());
        }
        for (j, &(dev, na, nk)) in b.leds.iter().enumerate() {
            let want = law.current(at_node(&xu, u, na) - at_node(&xu, u, nk));
            let v_lin = at_node(&lin, u, na) - at_node(&lin, u, nk);
            let floor = 0.5 * (law.current(v_lin + 1e-14) - law.current(v_lin - 1e-14)).abs() + 1e-17;
            let bj = last.as_ref().map_or(f64::INFINITY, |l| l.junction(j, bw));
            note(format!("LED {dev}"), rig.i(dev), law.current(v_lin), want, bj + floor, 1e-9 + 1e-3 * want.abs());
        }
        let bd = last.as_ref().map_or(f64::INFINITY, |l| l.draw(bw));
        note("draw".to_string(), i_p + g_p * du, dp + g_p * du, du_exact, bd + 1e-14 * (g_p.abs() * (1.0 + u.abs()) + du_exact.abs()), 1e-9 + 1e-3 * du_exact.abs());
        seen += 1;
        true
    };
    for &t in early {
        rig.advance(t).unwrap();
        look(&rig, t);
    }
    if ramp {
        sample_between_events(&mut rig, 10e-3, |rig, t| {
            look(rig, t);
        });
    } else {
        for k in 1..=4000 {
            let t = 40e-3 * k as f64 / 4000.0;
            rig.advance(t).unwrap();
            if !look(&rig, t) {
                break;
            }
        }
    }
    assert_eq!(unbounded, 0, "{}: {unbounded} of {seen} samples had no bound", b.name);
    (w0, wb, seen, solves, at)
}

/// V6's linear gate board: 1y held high into `r` to ground, or held low
/// sinking `r` from the rail (node 2), the gate's supply on the rail.
fn d7_gate_into_r(io: f64, lo: f64, hi: f64, r: f64, high: bool, ramp: bool) -> D7Board {
    let mut devs = d7_supply(ramp);
    if high {
        devs.extend([gate_io(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, lo, hi), resistor(3, 2, 0, r)]);
    } else {
        devs.extend([gate_io(2, 1, 0, [1, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, lo, hi), resistor(3, 1, 2, r)]);
    }
    let gs = static_part::NODE_RSHUNT_G;
    let exact = move |u: f64| {
        let g = gate_g(u, io);
        if high {
            let o = g * u / (g + 1.0 / r + gs);
            (vec![o], g * (u - o))
        } else {
            let o = (u / r) / (g + 1.0 / r + gs);
            (vec![o], (u - o) / r)
        }
    };
    D7Board {
        name: format!("{} gate io {io} into {r} ohm, {}", if high { "high" } else { "low" }, if ramp { "CC ramp" } else { "battery" }),
        p: no_cache(program(4, devs)),
        nodes: vec![2],
        res: vec![if high { (3, 2, 0, r) } else { (3, 1, 2, r) }],
        leds: vec![],
        exact: Box::new(exact),
        supply: (1, 0.9 * lo, 1.1 * hi),
        carry: Vec::new(),
    }
}

/// A gate supplied from the rail through `rs` (node 2, a free chip
/// supply), its 1y high (node 4) into an LED and `r` (node 5), or into `r`
/// alone.
fn d7_gate_fed_through(rs: f64, r: f64, with_led: bool) -> D7Board {
    let io = 0.008;
    let mut devs = d7_supply(false);
    devs.extend([resistor(2, 1, 2, rs), gate_io(3, 2, 0, [0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 4.75, 5.25)]);
    let gs = static_part::NODE_RSHUNT_G;
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let jf = move |v: f64| law.current(v) + led::GMIN_D * v;
    if with_led {
        devs.extend([led(4, 4, 5), resistor(5, 5, 0, r)]);
        let exact = move |u: f64| {
            let x = newton_fd(vec![u - 0.1, u - 0.4, 0.5 * u], &|x: &[f64]| {
                let (v, o, c) = (x[0], x[1], x[2]);
                let g = gate_g(v, io);
                vec![(v - u) / rs + g * (v - o) + gs * v, g * (o - v) + jf(o - c) + gs * o, -jf(o - c) + c / r + gs * c]
            });
            let d = (u - x[0]) / rs;
            (x, d)
        };
        D7Board {
            name: format!("gate fed through {rs} ohm, 1y into an LED and {r} ohm"),
            p: no_cache(program(6, devs)),
            nodes: vec![2, 4, 5],
            res: vec![(2, 1, 2, rs), (5, 5, 0, r)],
            leds: vec![(4, 4, 5)],
            exact: Box::new(exact),
            supply: (2, 0.9 * 4.75, 1.1 * 5.25),
            carry: Vec::new(),
        }
    } else {
        devs.push(resistor(4, 4, 0, r));
        let exact = move |u: f64| {
            let x = newton_fd(vec![u - 0.1, u - 0.4], &|x: &[f64]| {
                let (v, o) = (x[0], x[1]);
                let g = gate_g(v, io);
                vec![(v - u) / rs + g * (v - o) + gs * v, g * (o - v) + o / r + gs * o]
            });
            let d = (u - x[0]) / rs;
            (x, d)
        };
        D7Board {
            name: format!("gate fed through {rs} ohm, 1y into {r} ohm"),
            p: no_cache(program(5, devs)),
            nodes: vec![2, 4],
            res: vec![(2, 1, 2, rs), (4, 4, 0, r)],
            leds: vec![],
            exact: Box::new(exact),
            supply: (2, 0.9 * 4.75, 1.1 * 5.25),
            carry: Vec::new(),
        }
    }
}

#[test]
fn a_rail_supplied_gate_output_follows_the_rail_within_w0() {
    // V6 finding 1, ruling R7 (a). A gate whose supply is the rail drives
    // 22 or 220 ohm (held high into ground, or low under the rail): R_out =
    // vs / (4 io_max) follows the rail, so the partition, linear in its own
    // nodes, is curved in the rail. Kept to its first-order correction it
    // drifted 3-30x W0 off on the {5 V, 0.1 A} CC ramp and 2.6-130x on a
    // draining cell (one solve at 5 V carried down 0.7 V). At every
    // published instant with the gate powered, each published node voltage,
    // resistor current and the draw is within W0's tolerance of the exact
    // solution at the published rail, and the correction is within the
    // bound its band was sized from.
    let mut worst = (0.0f64, 0.0f64, String::new());
    for ramp in [true, false] {
        for (io, lo, hi) in [(0.008, 4.75, 5.25), (0.024, 4.5, 5.5)] {
            for r in [22.0, 220.0] {
                for high in [true, false] {
                    let b = d7_gate_into_r(io, lo, hi, r, high, ramp);
                    let (w0, wb, seen, solves, at) = d7_published_against_exact(&b, ramp);
                    eprintln!("{}: W0 {w0:.4}, bound {wb:.4}, {seen} samples, {solves} solves; worst at {at}", b.name);
                    assert!(seen > 50, "{}: {seen} samples", b.name);
                    if w0.max(wb) > worst.0.max(worst.1) {
                        worst = (w0, wb, format!("{}: {at}", b.name));
                    }
                }
            }
        }
    }
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.2);
    assert!(worst.1 <= 1.0, "a correction {} of its bound at {}", worst.1, worst.2);
}

#[test]
fn a_gate_fed_from_the_rail_through_a_resistor_follows_the_rail_within_w0() {
    // V6 finding 2, ruling R7 (a). A gate supplied from a draining cell
    // through 10 or 47 ohm (a free chip supply in the hub partition) drives
    // an LED and a resistor, or a resistor alone. Its R_out follows its own
    // supply node, so the partition is no conductance network and had no
    // curvature bound: on the flat 25 mV band it drifted 1.4-2.8x W0 off.
    // At every published instant with the gate powered, each published node
    // voltage, resistor and LED current and the draw is within W0's
    // tolerance of the exact solution at the published rail, and the
    // correction is within the bound its band was sized from.
    let mut worst = (0.0f64, 0.0f64, String::new());
    for (rs, r, with_led) in [(10.0, 330.0, true), (10.0, 100.0, true), (47.0, 330.0, true), (10.0, 47.0, false), (22.0, 220.0, false)] {
        let b = d7_gate_fed_through(rs, r, with_led);
        let (w0, wb, seen, solves, at) = d7_published_against_exact(&b, false);
        eprintln!("{}: W0 {w0:.4}, bound {wb:.4}, {seen} samples, {solves} solves; worst at {at}", b.name);
        assert!(seen > 20, "{}: {seen} samples", b.name);
        if w0.max(wb) > worst.0.max(worst.1) {
            worst = (w0, wb, format!("{}: {at}", b.name));
        }
    }
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.2);
    assert!(worst.1 <= 1.0, "a correction {} of its bound at {}", worst.1, worst.2);
}

#[test]
fn a_resistor_into_a_node_holding_a_junction_and_a_gate_pin_is_not_in_series_with_it() {
    // V6 finding 3 (seed S3). 1y held high drives 330 ohm into node 3, which
    // holds an LED to ground and 2y held low: the resistor's end node holds a
    // junction and an IC pin, whose driver carries current too, so KCL there
    // does not tie the resistor's current to the junction's and the series
    // rule (F1r7's tightening) must not take it. Taken, its bound fell to
    // 1e-14 A of a 5e-8 A error. On the {5 V, 0.1 A} CC ramp and a draining
    // cell, every published value is within W0's tolerance and the
    // correction within the bound its band was sized from.
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let jf = move |v: f64| law.current(v) + led::GMIN_D * v;
    let gs = static_part::NODE_RSHUNT_G;
    let mut worst = (0.0f64, 0.0f64, String::new());
    for ramp in [true, false] {
        for (io, lo, hi) in [(0.008, 4.75, 5.25), (0.024, 4.5, 5.5)] {
            let r = 330.0;
            let mut devs = d7_supply(ramp);
            devs.extend([gate_io(2, 1, 0, [0, 0, 2, 1, 1, 3, 0, 0, 0, 0, 0, 0], io, lo, hi), resistor(3, 2, 3, r), led(4, 3, 0)]);
            let exact = move |u: f64| {
                let g = gate_g(u, io);
                let x = newton_fd(vec![u - 0.2, 1.8], &|x: &[f64]| {
                    let (o, a) = (x[0], x[1]);
                    vec![g * (o - u) + (o - a) / r + gs * o, (a - o) / r + g * a + jf(a) + gs * a]
                });
                let d = g * (u - x[0]);
                (x, d)
            };
            let b = D7Board {
                name: format!("1y into an LED's node with 2y low, io {io}, {}", if ramp { "CC ramp" } else { "battery" }),
                p: no_cache(program(4, devs)),
                nodes: vec![2, 3],
                res: vec![(3, 2, 3, r)],
                leds: vec![(4, 3, 0)],
                exact: Box::new(exact),
                supply: (1, 0.9 * lo, 1.1 * hi),
                carry: Vec::new(),
            };
            let rig = Rig::load(&b.p).unwrap();
            let (_, in_series) = curvature::hook::of(2).expect("a curvature band was sized").branch(3, 1e-3);
            assert!(!in_series, "{}: the resistor is no branch in series with the junction", b.name);
            drop(rig);
            let (w0, wb, seen, solves, at) = d7_published_against_exact(&b, ramp);
            eprintln!("{}: W0 {w0:.4}, bound {wb:.4}, {seen} samples, {solves} solves; worst at {at}", b.name);
            assert!(seen > 50, "{}: {seen} samples", b.name);
            if w0.max(wb) > worst.0.max(worst.1) {
                worst = (w0, wb, format!("{}: {at}", b.name));
            }
        }
    }
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.2);
    assert!(worst.1 <= 1.0, "a correction {} of its bound at {}", worst.1, worst.2);
}

#[test]
fn a_stepped_hub_partition_holding_a_rail_supplied_gate_or_a_rail_fed_supply_follows_the_rail_within_w0() {
    // Ruling R7 (a) in a stepped hub partition, on V6's draining cell (5 V,
    // 0.5 ohm, 1e-7 Ah, 100 ohm). (a) A gate on the rail, 1y high through r
    // into node 3 and on through r into node 4, each holding 1 F (two
    // capacitors: stepped); (b) a gate fed from the rail through 10 ohm
    // (node 2), 1y (node 4) through r into node 5 holding 1 F (a free
    // supply with a capacitor: stepped). The farads hold their nodes over
    // the run, so at every published instant with the gate powered the
    // exact solution at the published rail is the static one with them at
    // their published voltages; each published node voltage and resistor
    // current is within W0's tolerance of it. A stepped hub partition
    // commits a step at every host horizon, its sensitivity taken afresh
    // there, so this held on 076be49 too: it guards the curved stepped path
    // (a band for (a), the exact sensitivity and the supply bound for (b)).
    let io = 0.008;
    let gs = static_part::NODE_RSHUNT_G;
    let mut worst = (0.0f64, String::new());
    for r in [47.0, 220.0] {
        for fed in [false, true] {
            let mut devs = d7_supply(false);
            if fed {
                devs.extend([resistor(2, 1, 2, 10.0), gate_io(3, 2, 0, [0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 4.75, 5.25), resistor(4, 4, 5, r), capacitor(5, 5, 0, 1.0, 0.0)]);
            } else {
                devs.extend([gate_io(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 4.75, 5.25), resistor(3, 2, 3, r), capacitor(4, 3, 0, 1.0, 0.0), resistor(5, 3, 4, r), capacitor(6, 4, 0, 1.0, 0.0)]);
            }
            let mut rig = Rig::load(&program(6, devs)).unwrap();
            let out = if fed { 4 } else { 2 };
            let part = rig.s().local_index[out].unwrap().0;
            assert_eq!(rig.s().partition_kinds[part], PartitionKind::Stepped, "a stepped hub partition");
            let mut seen = 0;
            for k in 1..=4000 {
                let t = 40e-3 * k as f64 / 4000.0;
                rig.advance(t).unwrap();
                let u = rig.v(1);
                let v_sup = if fed { rig.v(2) } else { u };
                if v_sup < 0.9 * 4.75 + 0.02 {
                    break;
                }
                // (node, published, exact), (resistor, published, exact).
                let checks: Vec<(String, f64, f64, bool)> = if fed {
                    let v5 = rig.v(5);
                    let x = newton_fd(vec![u - 0.1, u - 0.4], &|x: &[f64]| {
                        let (v, o) = (x[0], x[1]);
                        let g = gate_g(v, io);
                        vec![(v - u) / 10.0 + g * (v - o) + gs * v, g * (o - v) + (o - v5) / r + gs * o]
                    });
                    vec![("node 2".into(), rig.v(2), x[0], true), ("node 4".into(), rig.v(4), x[1], true), ("resistor 2".into(), rig.i(2), (u - x[0]) / 10.0, false), ("resistor 4".into(), rig.i(4), (x[1] - v5) / r, false)]
                } else {
                    let v3 = rig.v(3);
                    let g = gate_g(u, io);
                    let o = (g * u + v3 / r) / (g + 1.0 / r + gs);
                    vec![("node 2".into(), rig.v(2), o, true), ("resistor 3".into(), rig.i(3), (o - v3) / r, false)]
                };
                for (what, got, want, volts) in checks {
                    let tol = if volts { 1e-4 + 1e-3 * want.abs() } else { 1e-9 + 1e-3 * want.abs() };
                    let ratio = (got - want).abs() / tol;
                    if ratio > worst.0 {
                        worst = (ratio, format!("{} r {r}: {what} t {t:e} u {u:.6}: published {got:e}, exact {want:e}", if fed { "fed supply" } else { "gate on the rail" }));
                    }
                }
                seen += 1;
            }
            eprintln!("{} r {r}: {seen} samples", if fed { "fed supply" } else { "gate on the rail" });
            assert!(seen > 20, "{seen} samples");
        }
    }
    eprintln!("stepped: worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}

#[test]
fn a_failed_resolve_of_the_rail_partition_in_exact_mode_leaves_no_solve_behind() {
    // V6 finding 4 (EX1). In exact mode a resolve that re-solves the rail's
    // partition marks it solved before the solve, and the rail loop at the
    // end of the call takes the mark (the regime rules, and a battery's SoC,
    // run for a solve of the rail's partition). A solve that fails returns
    // before the rail loop, so the instant rolled back with the mark still
    // set, and the next event that runs the rail loop with no resolve of the
    // rail's partition took it for one. A {5 V, 0.1 A} PSU (static rail
    // partition) or a 5 V, 1 ohm, 1e-5 Ah cell (stepped) feeds an LED and
    // 330 ohm, and a switch adds 1 kohm; a separate 5 V source drives a
    // stepped RC (1 kohm, 1 uF, twice), whose steps are such events. Closing
    // the switch with Newton made to fail is rolled back with the mark
    // cleared, and the board goes on exactly as its twin that never closed
    // it: the PSU's last regime instant stayed where its last solve left it.
    for psu_rail in [true, false] {
        let supply = if psu_rail { psu(0, 1, 0, 5.0, 0.1) } else { generic_battery(0, 1, 0, 5.0, 1.0, 1e-5) };
        let p = exact(program(8, vec![
            supply,
            led(1, 1, 2),
            resistor(2, 2, 0, 330.0),
            switch(3, 2, 3, false),
            resistor(4, 3, 0, 1000.0),
            source(5, 4, 0, 5.0),
            resistor(6, 4, 5, 1000.0),
            capacitor(7, 5, 0, 1e-6, 0.0),
            resistor(8, 5, 6, 1000.0),
            capacitor(9, 6, 0, 1e-6, 0.0),
        ]));
        let (mut failed, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
        let (rail_p, rc_p) = (failed.s().local_index[1].unwrap().0, failed.s().local_index[5].unwrap().0);
        let rail_kind = if psu_rail { PartitionKind::Static } else { PartitionKind::Stepped };
        assert_eq!(failed.s().partition_kinds[rail_p], rail_kind, "the rail's partition");
        assert_eq!(failed.s().partition_kinds[rc_p], PartitionKind::Stepped, "the RC is stepped");
        failed.advance(1e-3).unwrap();
        twin.advance(1e-3).unwrap();
        let before = state_digest(failed.s());
        let fault = newton::hooks::force_unconverged(u32::MAX);
        let r = toggle(&mut failed, 3, true);
        drop(fault);
        assert!(matches!(r, Err(KernelError::SolveFailed(_))), "{r:?}");
        assert_eq!(state_digest(failed.s()), before, "the failed instant is rolled back");
        assert!(!failed.s().exact_solved, "no solve of the rail's partition is left marked");
        for k in 1..=20 {
            let t = 1e-3 + 1e-4 * k as f64;
            failed.advance(t).unwrap();
            twin.advance(t).unwrap();
            let (a, b) = (failed.s().exact.as_ref().unwrap(), twin.s().exact.as_ref().unwrap());
            assert_eq!((a.reg, a.battery, a.t_last.to_bits(), a.held.to_bits()), (b.reg, b.battery, b.t_last.to_bits(), b.held.to_bits()), "psu {psu_rail} at {t:e}: the regime, the battery, the last regime instant and the held draw");
            assert_eq!(state_digest(failed.s()), state_digest(twin.s()), "psu {psu_rail} at {t:e}");
        }
    }
}

#[test]
fn a_closed_form_hub_partition_holding_a_rail_supplied_gate_follows_the_rail_within_w0() {
    // V8's high finding, ruling R8 (a). A gate on the rail drives 1y high
    // through r into node 3, which holds 1 F to ground: one capacitor, so the
    // partition is closed-form. R_out = vs / (4 io_max) follows the rail, so
    // the segment's x0 + dx0/du (u - u_seg) is a first-order correction that
    // was never re-solved: 11-25x W0 at 8 mA, and 174x across R_out's 50 ohm
    // clamp kink at 24 mA, on a draining cell. At every published instant
    // with the gate powered the exact solution at the published rail is the
    // static one with node 3 at its published voltage (the capacitor a
    // source); node 2 and the resistor's current are within W0's tolerance
    // of it, on a draining cell and on a {5 V, 0.1 A} CC ramp. A farad holds
    // node 3 near 0; 10 uF charges it within the run, where the segment's
    // sensitivity follows `vc` (`dx` moves with R_out too).
    let gs = static_part::NODE_RSHUNT_G;
    let mut worst = (0.0f64, String::new());
    for (io, r, c) in [(0.008, 22.0, 1.0), (0.008, 220.0, 1.0), (0.024, 47.0, 1.0), (0.008, 47.0, 1e-5), (0.024, 22.0, 1e-5)] {
        for ramp in [false, true] {
            let mut devs = d7_supply(ramp);
            devs.extend([gate_io(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 4.0, 6.0), resistor(3, 2, 3, r), capacitor(4, 3, 0, c, 0.0)]);
            let mut rig = Rig::load(&no_cache(program(4, devs))).unwrap();
            let part = rig.s().local_index[2].unwrap().0;
            assert_eq!(rig.s().partition_kinds[part], PartitionKind::ClosedForm, "a closed-form hub partition");
            let name = format!("io {io} r {r} c {c} {}", if ramp { "CC ramp" } else { "battery" });
            let mut seen = 0;
            // A draining cell is followed until 0.1 V above the gate's power
            // floor, where its power gate ping-pongs (F5's class).
            let mut look = |rig: &Rig, t: f64| -> bool {
                let u = rig.v(1);
                if !(u >= 0.9 * 4.0 + 0.1) {
                    return ramp;
                }
                if ramp && rig.hub().state.reg != Regime::Cc {
                    return true;
                }
                let v3 = rig.v(3);
                let g = gate_g(u, io);
                let o = (g * u + v3 / r) / (g + 1.0 / r + gs);
                for (what, got, want, tol) in [("node 2", rig.v(2), o, 1e-4 + 1e-3 * o.abs()), ("resistor 3", rig.i(3), (o - v3) / r, 1e-9 + 1e-3 * ((o - v3) / r).abs())] {
                    let ratio = (got - want).abs() / tol;
                    if ratio > worst.0 {
                        worst = (ratio, format!("{name}: {what} t {t:e} u {u:.6}: published {got:e}, exact {want:e}"));
                    }
                }
                seen += 1;
                true
            };
            if ramp {
                sample_between_events(&mut rig, 10e-3, |rig, t| {
                    look(rig, t);
                });
            } else {
                for k in 1..=4000 {
                    let t = 40e-3 * k as f64 / 4000.0;
                    rig.advance(t).unwrap();
                    if !look(&rig, t) {
                        break;
                    }
                }
            }
            eprintln!("{name}: {seen} samples, band {:?}, segments {}", rig.s().closed_form.hub_band(part), rig.s().closed_form.segments_built());
            assert!(seen > 50, "{name}: {seen} samples");
        }
    }
    eprintln!("closed-form gate RC: worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}

#[test]
fn the_power_gate_pingpong_board_through_5_ohm_is_held_off_until_its_rail_carries_it() {
    // The original board of `a_power_flip_inside_one_band_never_replays_the_
    // other_power_state` (a 4.06 V cell with 5 ohm, 1 kohm into 1 uF, a gate
    // whose 4.05 V power threshold the rail crosses as the capacitor
    // charges), which declined with -4 from 076be49 to F5
    // (`w1f-power-gate-pingpong`): powered, the gate's four HIGH outputs
    // draw 4.05 V through their node shunts (1.6e-11 A), whose 8e-11 V
    // across 5 ohm puts the rail back under a threshold the event landed
    // two reached widths (8.1e-12 V) past. Now (W1c point 1a) the probes
    // find it starving itself there, so it is held off until the rail
    // passes the threshold by that drop and two reached widths, which the
    // charging capacitor brings about 9 ps later; it then powers and drives
    // its outputs high to 10 ms, cache on or off.
    let mut gate = ls00(3, 1, 0, [0, 0, 4, 0, 0, 5, 0, 0, 6, 0, 0, 7], 10e-9, 0);
    gate.params[ic_param::VCC_MIN] = 4.5;
    let (e, r, big_r, c) = (4.06, 5.0, 1000.0, 1e-6);
    let floor = 0.9 * 4.5;
    let tau = (big_r + r) * c;
    // The rail with the gate off, `E - r (E - vc) / (R + r)`, reaches `u` at
    // this time (the shunts' 1e-12 relative aside).
    let rail_at = |u: f64| tau * (e / (e - (e - (e - u) * (big_r + r) / r))).ln();
    for cache in [true, false] {
        let mut p = program(8, vec![battery(0, 1, 0, e, r), resistor(1, 1, 2, big_r), capacitor(2, 2, 0, c, 0.0), gate.clone()]);
        p.options.no_solution_cache = !cache;
        let mut rig = Rig::load(&p).unwrap();
        let (mut held, mut freed) = (None, None);
        while rig.k.time() < 10e-3 {
            let t = rig.k.next_event_time().min(10e-3);
            rig.advance(t).unwrap_or_else(|err| panic!("cache {cache} at {t:e}: {err:?}"));
            let hold = rig.s().power_hold(3);
            if held.is_none() && hold.kind == digital::power::HoldKind::Off {
                held = Some((t, hold.release.expect("a release level")));
            }
            if held.is_some() && freed.is_none() && hold.is_free() {
                freed = Some(t);
            }
        }
        let (t_held, release) = held.expect("held off at the threshold");
        let t_freed = freed.expect("released");
        close(t_held, rail_at(floor), 1e-9, "held where the rail reaches the threshold");
        assert!(release.rising && release.level > floor + 2.0 * rails::REACHED_REL * floor && release.level < floor + 1e-9, "{release:?}");
        close(t_freed, rail_at(release.level), 1e-9, "released where the rail reaches its release level");
        eprintln!("cache {cache}: held at {t_held:.12e}, release {:.6e} V past the threshold, freed {:.3e} s later", release.level - floor, t_freed - t_held);
        assert!(t_freed - t_held > 5e-12 && t_freed - t_held < 2e-11, "cache {cache}: {:e} s", t_freed - t_held);
        assert_eq!(rig.s().hold_counts(), [1.0, 1.0], "cache {cache}");
        assert!(rig.u() > floor && (rig.v(4) - rig.u()).abs() < 1e-6, "cache {cache}: powered, 1y HIGH at 10 ms: rail {} V, 1y {} V", rig.u(), rig.v(4));
    }
}

/// V8's probe P3: two gates share a supply fed from the rail through `rs`
/// (node 2, a free chip supply): A's 1y (node 4) high into `r1`, B's 1y
/// (node 5) low sinking `r2` from the rail, drives `io_a` and `io_b`, on a
/// {5 V, 0.1 A} CC ramp whose rail capacitor starts at 4.3 V (inside both
/// gates' window, 3-6 V).
fn two_gates_on_a_fed_supply(rs: f64, r1: f64, r2: f64, io_a: f64, io_b: f64) -> D7Board {
    let gs = static_part::NODE_RSHUNT_G;
    let mut devs = d7_supply(true);
    let mut gate_b = gate_io(5, 2, 0, [2, 2, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0], io_b, 3.0, 6.0);
    gate_b.blob[0] = 4.0;
    devs.extend([resistor(2, 1, 2, rs), gate_io(3, 2, 0, [0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0], io_a, 3.0, 6.0), resistor(4, 4, 0, r1), gate_b, resistor(6, 1, 5, r2)]);
    let exact = move |u: f64| {
        let x = newton_fd(vec![u - 0.1, u - 0.4, 0.3], &|x: &[f64]| {
            let (v, o, q) = (x[0], x[1], x[2]);
            let (ga, gb) = (gate_g(v, io_a), gate_g(v, io_b));
            vec![(v - u) / rs + ga * (v - o) + gs * v, ga * (o - v) + o / r1 + gs * o, gb * q + (q - u) / r2 + gs * q]
        });
        let d = (u - x[0]) / rs + (u - x[2]) / r2;
        (x, d)
    };
    let mut p = no_cache(program(6, devs));
    p.digital_slot_count = p.digital_slot_count.max(16);
    D7Board {
        name: format!("two gates rs {rs} r1 {r1} r2 {r2} io {io_a}/{io_b}"),
        p,
        nodes: vec![2, 4, 5],
        res: vec![(2, 1, 2, rs), (4, 4, 0, r1), (6, 1, 5, r2)],
        leds: vec![],
        exact: Box::new(exact),
        supply: (2, 2.7, 6.6),
        carry: vec![(1, 4.3)],
    }
}

#[test]
fn two_gates_on_one_fed_supply_on_a_precharged_cc_ramp_stay_inside_their_supply_bound() {
    // V8's medium finding (probe P3, `two_gates_on_a_fed_supply`). The
    // supply bound (`curvature::supply_band`) sizes the band; with its
    // validation skipped this board ran 2.89x W0. At every published instant
    // in CC with the supply inside its window, each published node voltage,
    // resistor current and the draw is within W0's tolerance of the exact
    // solution at the published rail, and the correction is within the
    // bound its band was sized from, which every sample must have (the hook
    // holds this board's bounds alone).
    let mut worst = (0.0f64, 0.0f64, String::new());
    for rs in [10.0, 47.0] {
        for (r1, r2) in [(47.0, 47.0), (220.0, 22.0), (1000.0, 100.0)] {
            for (io_a, io_b) in [(0.008, 0.008), (0.024, 0.008), (0.008, 0.024)] {
                let b = two_gates_on_a_fed_supply(rs, r1, r2, io_a, io_b);
                let (w0, wb, seen, solves, at) = d7_published_against_exact(&b, true);
                eprintln!("{}: W0 {w0:.4}, bound {wb:.4}, {seen} samples, {solves} solves; worst at {at}", b.name);
                assert!(seen > 50, "{}: {seen} samples", b.name);
                if w0.max(wb) > worst.0.max(worst.1) {
                    worst = (w0, wb, format!("{}: {at}", b.name));
                }
            }
        }
    }
    eprintln!("two gates on a fed supply: worst W0 {:.4}, bound {:.4} at {}", worst.0, worst.1, worst.2);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.2);
    assert!(worst.1 <= 1.0, "a correction {} of its bound at {}", worst.1, worst.2);
}

#[test]
fn two_gates_on_one_fed_supply_stay_inside_their_supply_bound_a_small_rail_step_from_the_solve() {
    // G2b's finding on V8's P3 board (`two_gates_on_a_fed_supply`): sampled
    // at 1e-8 s, 3.4e-6 V up the ramp from the solve at 4.3 V, the
    // correction's true error was 32x its bound (126x at 1e-9 s, 184x on rs
    // 47, r1 22, r2 1000). The iterated solve that a free supply takes
    // passed Newton's tolerance 1.0e-4 V short of the solution (`R_out` is
    // restamped at the last iterate's supply, so it converges linearly), and
    // `s`, exact at that iterate, missed the solution's slope by 1.25e-5
    // V/V: a first-order error, all of it at a small step, against a bound
    // that was second-order alone. Sampled at instants from 1e-11 s to 1e-7
    // s after load (a rail step of about 5e-9 V to 5e-5 V), and on through
    // the ramp, every correction is within its bound, and every published
    // value within W0's tolerance.
    let early = [1e-11, 1e-10, 3e-10, 1e-9, 3e-9, 1e-8, 3e-8, 1e-7];
    let mut worst = (0.0f64, 0.0f64, String::new());
    for rs in [5.0, 10.0, 22.0, 47.0, 100.0] {
        for (r1, r2) in [(47.0, 47.0), (220.0, 22.0), (1000.0, 100.0), (22.0, 1000.0)] {
            for (io_a, io_b) in [(0.008, 0.008), (0.024, 0.008), (0.008, 0.024)] {
                // Powered, these hold the fed supply under the gates' 3 V
                // floor (1.9-2.6 V at 4.3 V), so the load's iterated solve
                // ping-pongs at it and declines (-4, F5's class), as on
                // fb96acb.
                if (rs == 47.0 && r1 == 22.0 && io_a == 0.024) || (rs == 100.0 && (r1 == 47.0 || r1 == 22.0)) {
                    continue;
                }
                let b = two_gates_on_a_fed_supply(rs, r1, r2, io_a, io_b);
                let (w0, wb, seen, solves, at) = d7_published_against_exact_at(&b, true, &early);
                eprintln!("{}: W0 {w0:.4}, bound {wb:.4}, {seen} samples, {solves} solves; worst at {at}", b.name);
                assert!(seen > 50, "{}: {seen} samples", b.name);
                if wb > worst.1 {
                    worst = (w0, wb, b.name.clone());
                }
                assert!(w0 <= 1.0, "{}: {w0} of W0's tolerance at {at}", b.name);
            }
        }
    }
    eprintln!("two gates on a fed supply, a small step from the solve: worst bound {:.4} at {}", worst.1, worst.2);
    assert!(worst.1 <= 1.0, "a correction {} of its bound at {}", worst.1, worst.2);
}

/// FXBv's worst board: one gate (drive `io`, window `win`) on a supply fed
/// from the rail through `rs` (node 2), its 1y (node 3) high into `r` to
/// ground, on the draining battery or the CC ramp precharged to `v0`.
fn one_gate_on_a_fed_supply(rs: f64, r: f64, io: f64, win: (f64, f64), ramp: bool, v0: f64) -> D7Board {
    let gs = static_part::NODE_RSHUNT_G;
    let mut devs = d7_supply(ramp);
    devs.extend([resistor(2, 1, 2, rs), gate_io(3, 2, 0, [0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, win.0, win.1), resistor(4, 3, 0, r)]);
    let exact = move |u: f64| {
        let x = newton_fd(vec![u - 0.3, 0.3], &|x: &[f64]| {
            let (v, o) = (x[0], x[1]);
            let g = gate_g(v, io);
            vec![(v - u) / rs + g * (v - o) + gs * v, g * (o - v) + o / r + gs * o]
        });
        let d = (u - x[0]) / rs;
        (x, d)
    };
    let mut p = no_cache(program(5, devs));
    p.digital_slot_count = p.digital_slot_count.max(8);
    D7Board {
        name: format!("one gate rs {rs} r {r} io {io} win {win:?} {}", if ramp { "ramp" } else { "battery" }),
        p,
        nodes: vec![2, 3],
        res: vec![(2, 1, 2, rs), (4, 3, 0, r)],
        leds: vec![],
        exact: Box::new(exact),
        supply: (2, 0.9 * win.0, 1.1 * win.1),
        carry: if ramp { vec![(1, v0)] } else { Vec::new() },
    }
}

/// Over every published instant of `b` (as [`d7_published_against_exact_at`]
/// samples them), the worst distance of the solve itself from the exact
/// solution at its rail, in volts, and the worst ratio of a PUBLISHED node
/// voltage's, resistor or LED current's or draw's error against the exact
/// solution to the bound its band was sized from: the band's guarantee as
/// the reader sees it, the solve's own offset included. A solve that formed
/// no bound (a chip's `R_out` clamp at hand: the narrowest band, which
/// re-solves at every move of the rail) is counted, its offset checked, and
/// its samples left out of the ratio. `(offset, ratio, samples, unbounded
/// samples, where)`.
fn d7_published_within_bound(b: &D7Board, ramp: bool, early: &[f64]) -> (f64, f64, usize, usize, String) {
    let mut rig = Rig::load_carrying(&b.p, &b.carry).unwrap();
    let part = rig.s().local_index[b.nodes[0]].unwrap().0;
    let local: Vec<usize> = b.nodes.iter().map(|&n| rig.s().local_index[n].unwrap().1).collect();
    let (mut offset, mut ratio, mut seen, mut unbounded, mut at) = (0.0f64, 0.0f64, 0usize, 0usize, String::new());
    let (sup, lo, hi) = b.supply;
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let mut look = |rig: &Rig, t: f64| -> bool {
        let u = rig.v(1);
        let v_sup = if sup == 1 { u } else { rig.v(sup) };
        if v_sup < lo + 0.02 && !ramp {
            return false;
        }
        let Some((x, _, i_p, g_p, u_p)) = rig.s().static_solver.hub_solve(part) else { return true };
        if ramp && rig.hub().state.reg != Regime::Cc {
            return true;
        }
        let (xp, _) = (b.exact)(u_p);
        let sup_p = if sup == 1 { u_p } else { xp[b.nodes.iter().position(|&n| n == sup).unwrap()] };
        if !(v_sup >= lo && v_sup <= hi && sup_p >= lo && sup_p <= hi) {
            return true;
        }
        for (k, &l) in local.iter().enumerate() {
            offset = offset.max((x[l] - xp[k]).abs());
        }
        let (du, bw) = (u - u_p, (u - u_p).abs());
        let (xu, d_exact) = (b.exact)(u);
        let Some(last) = curvature::hook::of(b.nodes[0]).filter(|l| l.u_p.to_bits() == u_p.to_bits()) else {
            panic!("{}: a sample at t {t:e} without its solve's record", b.name);
        };
        if !last.bounded() {
            unbounded += 1;
            return true;
        }
        let at_node = |x: &[f64], uu: f64, node: usize| if node == 0 { 0.0 } else if node == 1 { uu } else { x[b.nodes.iter().position(|&n| n == node).unwrap()] };
        let mut note = |what: String, published: f64, want: f64, bound: f64| {
            let r = (published - want).abs() / bound;
            if r > ratio {
                ratio = r;
                at = format!("{what} t {t:e} u {u:.6} u_p {u_p:.6}: published {published:e}, exact {want:e}, bound {bound:e}");
            }
        };
        for (k, &n) in b.nodes.iter().enumerate() {
            note(format!("node {n}"), rig.v(n), xu[k], last.node_at(n, bw) + 1e-14 * (1.0 + xu[k].abs()));
        }
        for &(dev, na, nb, r) in &b.res {
            let want = (at_node(&xu, u, na) - at_node(&xu, u, nb)) / r;
            note(format!("resistor {dev}"), rig.i(dev), want, last.branch(dev, bw).0 + 2e-14 * (1.0 + u.abs()) / r);
        }
        for (j, &(dev, na, nk)) in b.leds.iter().enumerate() {
            let v = at_node(&xu, u, na) - at_node(&xu, u, nk);
            // The current a rounding of the junction voltage moves.
            let floor = 0.5 * (law.current(v + 1e-14) - law.current(v - 1e-14)).abs() + 1e-17;
            note(format!("LED {dev}"), rig.i(dev), law.current(v), last.junction(j, bw) + floor);
        }
        // The draw is a sum of currents out of the rail, each the rail's
        // value against a node's through a conductance, so the engine's and
        // the exact solution's each round by a few ulps of `u` times every
        // conductance on the rail, however small the sum (LEDs off: 2e-12
        // A of 7e-3 A terms), and 2e-15 (9 ulps) covers both.
        let rail_g: f64 = b.p.devices.iter().map(|d| match d.kind {
            kinds::RESISTOR if d.pins.iter().any(|p| p.node == 1) && d.pins.iter().any(|p| p.node != 1) => 1.0 / d.params[0],
            kinds::IC_74LS00 if d.pins[0].node == 1 => gate_g(u, d.params[ic_param::IO_MAX]),
            kinds::LED if d.pins.iter().any(|p| p.node == 1) => {
                let v = at_node(&xu, u, d.pins[0].node) - at_node(&xu, u, d.pins[1].node);
                (law.current(v + 1e-6) - law.current(v - 1e-6)) / 2e-6 + led::GMIN_D
            }
            _ => 0.0,
        }).sum();
        note("draw".to_string(), i_p + g_p * du, d_exact, last.draw(bw) + 1e-14 * (g_p.abs() * (1.0 + u.abs()) + d_exact.abs()) + 2e-15 * u.abs() * rail_g);
        seen += 1;
        true
    };
    for &t in early {
        rig.advance(t).unwrap();
        look(&rig, t);
    }
    if ramp {
        sample_between_events(&mut rig, 10e-3, |rig, t| {
            look(rig, t);
        });
    } else {
        for k in 1..=4000 {
            let t = 40e-3 * k as f64 / 4000.0;
            rig.advance(t).unwrap();
            if !look(&rig, t) {
                break;
            }
        }
    }
    (offset, ratio, seen, unbounded, at)
}

#[test]
fn a_fed_supply_publishes_its_solution_so_its_band_holds_what_a_reader_sees() {
    // FXBv's finding, ruled R19 (a). The iterated solve a free supply takes
    // restamps `R_out` at the last iterate's supply, converging linearly, and
    // stopped at Newton's tolerance: on its worst board (rs 167, a 37 ohm
    // load, window 1-6 V, the battery) the published supply sat 9.6e-4 V
    // from the solution, 0.514 of W0, an offset the band's budget never
    // counted (its bound measures the correction from the solution). Polished
    // to the solution, every solve is within rounding of it, and every
    // published node voltage, resistor current and draw is within the bound
    // its band was sized from, from 1e-11 s after load (a rail step of about
    // 5e-9 V on the ramp) on through the run.
    let early = [1e-11, 1e-10, 1e-9, 1e-8, 1e-7];
    let rs_worst = 167.2799911696775;
    let mut boards = vec![
        (one_gate_on_a_fed_supply(rs_worst, 37.0, 0.008, (1.0, 6.0), false, 0.0), false),
        (one_gate_on_a_fed_supply(rs_worst, 37.0, 0.008, (1.0, 6.0), true, 4.3), true),
        (one_gate_on_a_fed_supply(126.28578229518071, 28.0, 0.004, (2.0, 6.0), true, 4.410348), true),
    ];
    for (rs, r1, r2) in [(47.0, 22.0, 1000.0), (10.0, 47.0, 47.0)] {
        boards.push((two_gates_on_a_fed_supply(rs, r1, r2, 0.008, 0.008), true));
    }
    let mut worst = (0.0f64, 0.0f64, String::new());
    for (b, ramp) in &boards {
        let (offset, ratio, seen, unbounded, at) = d7_published_within_bound(b, *ramp, &early);
        eprintln!("{}: offset {offset:e} V, published {ratio:.4} of its bound, {seen} samples; worst at {at}", b.name);
        assert!(seen > 50 && unbounded == 0, "{}: {seen} samples, {unbounded} without a bound", b.name);
        assert!(offset <= 1e-13, "{}: a solve {offset:e} V from the solution", b.name);
        if ratio > worst.1 {
            worst = (offset, ratio, format!("{}: {at}", b.name));
        }
    }
    eprintln!("a fed supply's published values: worst {:.4} of the bound at {}", worst.1, worst.2);
    assert!(worst.1 <= 1.0, "a published value {} of its bound at {}", worst.1, worst.2);
    // The seeded fault: without the polish the worst board's solve is the
    // passing iterate again, 9e-4 V from the solution. Its band counts that
    // distance now (its rounding floor reads the residual, R23 (i)), so it
    // narrows rather than breaks, and the refinement that floor sets off
    // takes the solve to 6e-6 V: the offset is what fails.
    let _fault = newton::hooks::cap_hub_polish(0);
    let (offset, ratio, _, _, _) = d7_published_within_bound(&boards[0].0, false, &early);
    assert!(offset > 1e-9, "unpolished: a solve {offset:e} V from the solution, published {ratio} of its bound");
}

#[test]
fn a_fed_supply_polish_that_runs_out_of_steps_fails_the_solve_instead_of_publishing_short() {
    // The polish is bounded: on the worst board the first solve needs four
    // Newton steps (9.7e-4 V, 3.5e-7, 4.5e-14, then rounding), so capped at
    // two it stops short, and the solve fails cleanly (-4) as an iterated
    // solve that never passed does; capped at four it arrives.
    let b = one_gate_on_a_fed_supply(167.2799911696775, 37.0, 0.008, (1.0, 6.0), false, 0.0);
    {
        let _cap = newton::hooks::cap_hub_polish(2);
        let e = Rig::load(&b.p).err().expect("a polish cut short fails the load's solve");
        assert_eq!(e.code, crate::error::ERR_SOLVE_FAILED, "{}", e.message);
        assert!(e.message.contains("hub polish"), "{}", e.message);
    }
    let _cap = newton::hooks::cap_hub_polish(4);
    let rig = Rig::load(&b.p).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    let (x, _, _, _, u_p) = rig.s().static_solver.hub_solve(part).unwrap();
    let (xp, _) = (b.exact)(u_p);
    let l = rig.s().local_index[2].unwrap().1;
    assert!((x[l] - xp[0]).abs() <= 1e-13, "supply {:e} V from the solution", (x[l] - xp[0]).abs());
}

/// A gate on the rail (drive `io`, supply window `win`), its 1y (node 2)
/// high into `r`, an LED (node 3) and `rl` to ground (node 4): FXC's sweep
/// board at `io` 0.016 into 1000 and 47 ohm.
fn rail_gate_into_an_led(io: f64, win: (f64, f64), r: f64, rl: f64, ramp: bool) -> D7Board {
    let gs = static_part::NODE_RSHUNT_G;
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let jf = move |v: f64| law.current(v) + led::GMIN_D * v;
    let mut devs = d7_supply(ramp);
    devs.extend([gate_io(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, win.0, win.1), resistor(3, 2, 3, r), led(4, 3, 4), resistor(5, 4, 0, rl)]);
    let exact = move |u: f64| {
        let g = gate_g(u, io);
        let x = newton_fd(vec![u - 0.2, u - 0.4, 0.4 * u], &|x: &[f64]| {
            let (o, a, c) = (x[0], x[1], x[2]);
            vec![g * (o - u) + (o - a) / r + gs * o, (a - o) / r + jf(a - c) + gs * a, -jf(a - c) + c / rl + gs * c]
        });
        let d = g * (u - x[0]);
        (x, d)
    };
    D7Board {
        name: format!("rail gate io {io} into {r}, an LED and {rl}, {}", if ramp { "CC ramp" } else { "battery" }),
        p: no_cache(program(5, devs)),
        nodes: vec![2, 3, 4],
        res: vec![(3, 2, 3, r), (5, 4, 0, rl)],
        leds: vec![(4, 3, 4)],
        exact: Box::new(exact),
        supply: (1, 0.9 * win.0, 1.1 * win.1),
        carry: Vec::new(),
    }
}

/// A gate on the rail, its 1y (node 2) high through `r` into node 3,
/// clamped by an LED to ground and by a reverse one to the rail (cathode on
/// the rail), and loaded by `rl` to ground.
fn rail_gate_clamped_by_leds(io: f64, win: (f64, f64), r: f64, rl: f64, ramp: bool) -> D7Board {
    let gs = static_part::NODE_RSHUNT_G;
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let jf = move |v: f64| law.current(v) + led::GMIN_D * v;
    let mut devs = d7_supply(ramp);
    devs.extend([gate_io(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, win.0, win.1), resistor(3, 2, 3, r), led(4, 3, 0), led(5, 3, 1), resistor(6, 3, 0, rl)]);
    let exact = move |u: f64| {
        let g = gate_g(u, io);
        let x = newton_fd(vec![u - 0.2, 0.4 * u], &|x: &[f64]| {
            let (o, m) = (x[0], x[1]);
            vec![g * (o - u) + (o - m) / r + gs * o, (m - o) / r + jf(m) + jf(m - u) + m / rl + gs * m]
        });
        // The rail feeds the gate and takes the reverse clamp's current.
        let d = g * (u - x[0]) - jf(x[1] - u);
        (x, d)
    };
    D7Board {
        name: format!("rail gate io {io} through {r} into two LED clamps and {rl}, {}", if ramp { "CC ramp" } else { "battery" }),
        p: no_cache(program(4, devs)),
        nodes: vec![2, 3],
        res: vec![(3, 2, 3, r), (6, 3, 0, rl)],
        leds: vec![(4, 3, 0), (5, 3, 1)],
        exact: Box::new(exact),
        supply: (1, 0.9 * win.0, 1.1 * win.1),
        carry: Vec::new(),
    }
}

/// Two LEDs in series from the rail through `r` (nodes 2 and 3), with no
/// chip: a hub partition curved by its junctions alone.
fn an_led_chain_on_the_rail(r: f64, ramp: bool) -> D7Board {
    let gs = static_part::NODE_RSHUNT_G;
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let jf = move |v: f64| law.current(v) + led::GMIN_D * v;
    let mut devs = d7_supply(ramp);
    devs.extend([resistor(2, 1, 2, r), led(3, 2, 3), led(4, 3, 0)]);
    let exact = move |u: f64| {
        let x = newton_fd(vec![0.8 * u, 0.4 * u], &|x: &[f64]| {
            let (a, m) = (x[0], x[1]);
            vec![(a - u) / r + jf(a - m) + gs * a, -jf(a - m) + jf(m) + gs * m]
        });
        let d = (u - x[0]) / r;
        (x, d)
    };
    D7Board {
        name: format!("two LEDs from the rail through {r}, {}", if ramp { "CC ramp" } else { "battery" }),
        p: no_cache(program(4, devs)),
        nodes: vec![2, 3],
        res: vec![(2, 1, 2, r)],
        leds: vec![(3, 2, 3), (4, 3, 0)],
        exact: Box::new(exact),
        supply: (1, 1.0, 6.0),
        carry: Vec::new(),
    }
}

/// A gate on the rail, its 1y (node 2) high through `r` into two LEDs in
/// series (nodes 3 and 4) and `rl` to ground (node 5).
fn rail_gate_into_an_led_chain(io: f64, win: (f64, f64), r: f64, rl: f64, ramp: bool) -> D7Board {
    let gs = static_part::NODE_RSHUNT_G;
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let jf = move |v: f64| law.current(v) + led::GMIN_D * v;
    let mut devs = d7_supply(ramp);
    devs.extend([gate_io(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, win.0, win.1), resistor(3, 2, 3, r), led(4, 3, 4), led(5, 4, 5), resistor(6, 5, 0, rl)]);
    let exact = move |u: f64| {
        let g = gate_g(u, io);
        let x = newton_fd(vec![u - 0.2, u - 0.3, 0.5 * u, 0.1 * u], &|x: &[f64]| {
            let (o, a, m, c) = (x[0], x[1], x[2], x[3]);
            vec![
                g * (o - u) + (o - a) / r + gs * o,
                (a - o) / r + jf(a - m) + gs * a,
                -jf(a - m) + jf(m - c) + gs * m,
                -jf(m - c) + c / rl + gs * c,
            ]
        });
        let d = g * (u - x[0]);
        (x, d)
    };
    D7Board {
        name: format!("rail gate io {io} through {r} into two LEDs and {rl}, {}", if ramp { "CC ramp" } else { "battery" }),
        p: no_cache(program(6, devs)),
        nodes: vec![2, 3, 4, 5],
        res: vec![(3, 2, 3, r), (6, 5, 0, rl)],
        leds: vec![(4, 3, 4), (5, 4, 5)],
        exact: Box::new(exact),
        supply: (1, 0.9 * win.0, 1.1 * win.1),
        carry: Vec::new(),
    }
}

#[test]
fn every_iterated_hub_solve_publishes_its_solution_so_its_band_holds_what_a_reader_sees() {
    // R20 (a). A hub partition curved by a junction (with or without a chip
    // on the rail) is iterated by Newton, which stopped at its tolerance
    // plus one polish iteration: on FXC's sweep board (a 16 mA gate into
    // 1000 ohm, an LED and 47 ohm) the published LED current sat 2.0e-10 A
    // from the solution's, 6.5e5x the bound its band was sized from at a
    // rail step of 3e-9 V. Polished to the solution, every solve is within
    // rounding of it, and every published node voltage, resistor and LED
    // current and draw is within its bound, from 1e-11 s after load on
    // through the run, on the CC ramp and on the draining battery.
    let early = [1e-11, 1e-10, 1e-9, 1e-8, 1e-7];
    let mut boards = Vec::new();
    for ramp in [true, false] {
        boards.push((rail_gate_into_an_led(0.016, (4.0, 6.0), 1000.0, 47.0, ramp), ramp));
        boards.push((rail_gate_into_an_led(0.008, (1.0, 6.0), 220.0, 330.0, ramp), ramp));
        boards.push((rail_gate_clamped_by_leds(0.008, (2.0, 6.0), 100.0, 1000.0, ramp), ramp));
        boards.push((rail_gate_into_an_led_chain(0.016, (2.0, 6.0), 22.0, 47.0, ramp), ramp));
        boards.push((an_led_chain_on_the_rail(150.0, ramp), ramp));
    }
    let mut worst = (0.0f64, 0.0f64, String::new());
    for (b, ramp) in &boards {
        let (offset, ratio, seen, unbounded, at) = d7_published_within_bound(b, *ramp, &early);
        eprintln!("{}: offset {offset:e} V, published {ratio:.4} of its bound, {seen} samples ({unbounded} without a bound); worst at {at}", b.name);
        assert!(seen - unbounded > 50, "{}: {seen} samples, {unbounded} without a bound", b.name);
        assert!(offset <= 1e-13, "{}: a solve {offset:e} V from the solution", b.name);
        if ratio > worst.1 {
            worst = (offset, ratio, format!("{}: {at}", b.name));
        }
    }
    eprintln!("an iterated hub solve's published values: worst {:.4} of the bound at {}", worst.1, worst.2);
    assert!(worst.1 <= 1.0, "a published value {} of its bound at {}", worst.1, worst.2);
    // The seeded fault: without the polish FXC's board publishes the
    // passing iterate again, 6e-9 V from the solution. Its band counts that
    // distance now (its rounding floor reads the residual, R23 (i)), so it
    // narrows rather than breaks: the offset is what fails.
    let _fault = newton::hooks::cap_hub_polish(0);
    let (offset, ratio, _, _, _) = d7_published_within_bound(&boards[0].0, true, &early);
    assert!(offset > 1e-9, "unpolished: a solve {offset:e} V from the solution, published {ratio} of its bound");
}

#[test]
fn a_stepped_hub_partition_holding_a_rail_supplied_gate_solves_again_when_a_load_step_moves_the_rail() {
    // V8's low finding (seed Sa_stepped: a stepped partition curved only when
    // iterated). A gate on a 5 V, 1 ohm cell drives 1y high through 47 ohm
    // into node 3 and on through 47 ohm into node 4, each holding 1 F (two
    // capacitors: stepped, linear in its own nodes, curved in the rail
    // through R_out). A switch puts 5 ohm across the rail at an instant that
    // commits no step of the gate's partition, so the rail steps about 0.8 V
    // and the partition is republished at its first-order correction: a
    // partition holding a rail-supplied pin solves again past its band
    // (Rails point 3), and right after each toggle node 2 and the resistor's
    // current are within W0's tolerance of the exact solution at the
    // published rail with node 3 at its published voltage. Kept to the
    // correction, node 2 errs by f'' du^2 / 2, about 5x W0.
    let (io, r) = (0.008, 47.0);
    let gs = static_part::NODE_RSHUNT_G;
    let devs = vec![
        battery(0, 1, 0, 5.0, 1.0),
        gate_io(1, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 4.0, 6.0),
        resistor(2, 2, 3, r),
        capacitor(3, 3, 0, 1.0, 0.0),
        resistor(4, 3, 4, r),
        capacitor(5, 4, 0, 1.0, 0.0),
        switch(6, 1, 5, false),
        resistor(7, 5, 0, 5.0),
    ];
    let mut rig = Rig::load(&program(6, devs)).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    assert_eq!(rig.s().partition_kinds[part], PartitionKind::Stepped, "a stepped hub partition");
    let mut worst = (0.0f64, String::new());
    for k in 1..=6 {
        let t = 1e-3 * k as f64;
        rig.advance(t).unwrap();
        let before = rig.u();
        toggle(&mut rig, 6, k % 2 == 1).unwrap();
        let u = rig.u();
        assert!((u - before).abs() > 0.5, "the load step moves the rail: {before} -> {u}");
        let v3 = rig.v(3);
        let g = gate_g(u, io);
        let o = (g * u + v3 / r) / (g + 1.0 / r + gs);
        for (what, got, want, tol) in [("node 2", rig.v(2), o, 1e-4 + 1e-3 * o.abs()), ("resistor 2", rig.i(2), (o - v3) / r, 1e-9 + 1e-3 * ((o - v3) / r).abs())] {
            let ratio = (got - want).abs() / tol;
            if ratio > worst.0 {
                worst = (ratio, format!("{what} at {t:e} u {u:.6}: published {got:e}, exact {want:e}"));
            }
        }
    }
    eprintln!("stepped load step: worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}

#[test]
fn a_draw_that_no_stamp_can_carry_sizes_no_band_and_the_partition_stays_within_w0() {
    // V8's cost finding. The draw check took a draw that cannot flow as if
    // it were a branch current: (a) a gate on the rail, 1y high into nothing
    // but another gate's inputs, so the rail's current through its driver
    // can only leave through the node shunts (1e-12 S: the draw is the
    // shunts' current, its error theirs), sized a 6.8 mV band on a draining
    // cell; (b) a gate whose gnd is the rail and whose vcc is a fixed 9.6 V,
    // 1y high toward vcc into 220 ohm, has no stamp on the rail at all (its
    // draw is 0 at every rail), and re-solved over a thousand times up a CC
    // ramp. Each is classed from its stamps (`DrawRule`), never from the
    // draw's size; with it, every published value is within W0's tolerance
    // of the exact solution and the correction within its bound.
    let gs = static_part::NODE_RSHUNT_G;
    let io = 0.008;
    let mut rows = Vec::new();
    for ramp in [false, true] {
        // (a) Node 2: 1y of a gate on the rail, read by 1a/1b of a second.
        let mut devs = d7_supply(ramp);
        let mut reader = gate_io(3, 1, 0, [2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 4.0, 6.0);
        reader.blob[0] = 4.0;
        devs.extend([gate_io(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 4.0, 6.0), reader]);
        let mut p = no_cache(program(4, devs));
        p.digital_slot_count = p.digital_slot_count.max(16);
        let exact = move |u: f64| {
            let g = gate_g(u, io);
            let o = g * u / (g + gs);
            (vec![o], gs * o)
        };
        rows.push((curvature::DrawRule::ShuntOnly, D7Board { name: format!("1y into gate inputs, {}", if ramp { "CC ramp" } else { "battery" }), p, nodes: vec![2], res: vec![], leds: vec![], exact: Box::new(exact), supply: (1, 3.6, 6.6), carry: Vec::new() }));
        // (b) Node 2: 1y of a gate whose gnd is the rail, its vcc node 3 at
        // 9.6 V, inputs on the rail (low), into 220 ohm to the datum.
        let (vcc, r) = (9.6, 220.0);
        let mut devs = d7_supply(ramp);
        devs.extend([source(2, 3, 0, vcc), gate_io(3, 3, 1, [1, 1, 2, 1, 1, 0, 1, 1, 0, 1, 1, 0], io, 4.0, 6.5), resistor(4, 2, 0, r)]);
        let exact = move |u: f64| {
            let g = gate_g(vcc - u, io);
            (vec![g * vcc / (g + 1.0 / r + gs)], 0.0)
        };
        let carry = if ramp { vec![(1, 2.7)] } else { Vec::new() };
        rows.push((curvature::DrawRule::Zero, D7Board { name: format!("gnd-on-rail 1y high into {r} ohm, {}", if ramp { "CC ramp" } else { "battery" }), p: no_cache(program(4, devs)), nodes: vec![2], res: vec![(4, 2, 0, r)], leds: vec![], exact: Box::new(exact), supply: (1, 2.6, 5.5), carry }));
    }
    let mut worst = (0.0f64, 0.0f64, String::new());
    for (rule, b) in &rows {
        let ramp = b.name.ends_with("CC ramp");
        let (w0, wb, seen, solves, at) = d7_published_against_exact(b, ramp);
        assert_eq!(curvature::hook::of(2).and_then(|l| l.draw_rule()), Some(*rule), "{}: the draw's rule at its last solve", b.name);
        eprintln!("{}: W0 {w0:.4}, bound {wb:.4}, {seen} samples, {solves} solves; worst at {at}", b.name);
        assert!(seen > 50, "{}: {seen} samples", b.name);
        if w0.max(wb) > worst.0.max(worst.1) {
            worst = (w0, wb, format!("{}: {at}", b.name));
        }
    }
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.2);
    assert!(worst.1 <= 1.0, "a correction {} of its bound at {}", worst.1, worst.2);
}

// ---- INV2: a rail root on a driver's R_out kink (Rails point 3) -------------

/// A 0.9 ohm battery, a gate with io_max 24 mA (a 74LS245's: `R_out =
/// max(50, u / 96 mA)` kinks at u = 4.8 V) holding an LED with 100 ohm high
/// off the rail (a nonlinear static hub partition), `base` ohm on the rail,
/// and a 1 kHz clock whose inverse (a 74LS00 output) loads the rail with
/// `rl` ohm in the clock's low half.
fn kink_board(base: f64, rl: f64, no_cache: bool) -> Program {
    let mut gate = ls00(1, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], 10e-9, 0);
    gate.params[ic_param::IO_MAX] = 0.024;
    let mut p = program(6, vec![
        battery(0, 1, 0, 5.0, 0.9),
        gate,
        led(2, 2, 3),
        resistor(3, 3, 0, 100.0),
        resistor(4, 1, 0, base),
        clock(5, 4, 0, 1000.0, 0.5, 8),
        ls00(6, 1, 0, [4, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0], 10e-9, 4),
        resistor(7, 5, 0, rl),
    ]);
    p.options.no_solution_cache = no_cache;
    p
}

#[test]
fn a_rail_root_just_past_a_driver_kink_settles_between_two_cache_entries_across_it() {
    // INV2 (the SAP-1 on a battery, -4 at 2.897 s with the rail at 4.8 V).
    // The LED partition's curvature band stops at the kink, so its solve on
    // the clock's high half, 19 mV above it, has a band reaching down to
    // exactly 4.8 V. On the low half the rail's root sits 10-80 nV above the
    // kink: that solve's first-order correction (concave there, R_out
    // growing with u) puts the rail just below 4.8 V, where a fresh solve,
    // band up to the kink, puts it back above, inside the first entry's
    // band. Replaying the two cache entries alternated until the rail loop's
    // 50 passes ran out. Cache off, the same board settles; cache on must
    // too, within W0's tolerance of it. The loads are fitted with the cache
    // off, so the root lands in that window whatever the model's last digits.
    let kink = 4.0 * 0.024 * 50.0;
    let rail_at = |base: f64, rl: f64, no_cache: bool, t: f64| -> Result<(f64, Rig), KernelError> {
        let mut rig = Rig::load(&kink_board(base, rl, no_cache)).unwrap();
        rig.advance(t)?;
        Ok((rig.u(), rig))
    };
    // A secant on one resistance for a rail target, cache off.
    let fit = |x0: f64, x1: f64, target: f64, f: &dyn Fn(f64) -> f64| -> f64 {
        let (mut x0, mut x1) = (x0, x1);
        let (mut f0, mut f1) = (f(x0) - target, f(x1) - target);
        for _ in 0..40 {
            if f1.abs() < 1e-13 || f1 == f0 {
                break;
            }
            let x2 = x1 - f1 * (x1 - x0) / (f1 - f0);
            (x0, f0, x1) = (x1, f1, x2);
            f1 = f(x1) - target;
        }
        x1
    };
    let (high, low) = (0.25e-3, 0.75e-3);
    let base = fit(26.0, 27.0, kink + 0.019, &|b| rail_at(b, 1e6, true, high).unwrap().0);
    let (_, rig) = rail_at(base, 1e6, false, high).unwrap();
    let led_p = rig.s().local_index[2].unwrap().0;
    let u_p = rig.s().static_solver.hub_solve(led_p).unwrap().4;
    let band = rig.s().static_solver.hub_band(led_p).expect("the LED partition is sized by its curvature");
    assert!(u_p - kink > 0.018 && (u_p - band - kink).abs() < 1e-9, "the high half's band reaches the kink: u_p {u_p} band {band}");
    let rl0 = fit(60.0, 70.0, kink, &|r| rail_at(base, r, true, low).unwrap().0);
    for nv in [10.0, 20.0, 40.0, 80.0] {
        let target = kink + nv * 1e-9;
        let rl = fit(rl0, rl0 * 1.0001, target, &|r| rail_at(base, r, true, low).unwrap().0);
        let (want, _) = rail_at(base, rl, true, low).unwrap();
        assert!((want - target).abs() < 1e-11, "the fit: {want} for {target}");
        let (got, _) = rail_at(base, rl, false, low).unwrap_or_else(|e| panic!("the root {nv} nV past the kink: {e:?}"));
        assert!((got - want).abs() <= 1e-4 + 1e-3 * want.abs(), "{nv} nV: cache on {got}, off {want}");
    }
}

#[test]
fn a_failed_instant_rolls_back_a_closed_form_segment_built_again_past_its_band() {
    // Ruling R8 (a): a curved closed-form hub partition past its band builds
    // its segment again at the present rail, and a failed instant must take
    // that build back with the rest (the segment, its rail and band, and the
    // aim's re-aimed `dx`, `di` and rail distance). A gate on a 5 V, 1 ohm
    // cell drives 1y high through 47 ohm into 1 F (closed-form); an LED with
    // 330 ohm off the rail is a nonlinear static hub partition. Closing a
    // switch onto 5 ohm steps the rail about 0.8 V: the segment builds again,
    // then the LED's Newton is made to fail. The instant rolls back to the
    // state before it, and the board goes on exactly as its twin that never
    // closed the switch.
    let p = program(7, vec![
        battery(0, 1, 0, 5.0, 1.0),
        gate_io(1, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0.008, 4.0, 6.0),
        resistor(2, 2, 3, 47.0),
        capacitor(3, 3, 0, 1.0, 0.0),
        switch(4, 1, 4, false),
        resistor(5, 4, 0, 5.0),
        led(6, 1, 5),
        resistor(7, 5, 0, 330.0),
    ]);
    let (mut failed, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
    let part = failed.s().local_index[2].unwrap().0;
    assert_eq!(failed.s().partition_kinds[part], PartitionKind::ClosedForm, "a closed-form hub partition");
    assert!(failed.s().closed_form.hub_band(part).is_some(), "curved: it holds a pin of a gate on the rail");
    failed.advance(1e-3).unwrap();
    twin.advance(1e-3).unwrap();
    let before = state_digest(failed.s());
    let built = failed.s().closed_form.segments_built();
    let fault = newton::hooks::force_unconverged(u32::MAX);
    let r = toggle(&mut failed, 4, true);
    drop(fault);
    assert!(matches!(r, Err(KernelError::SolveFailed(_))), "{r:?}");
    assert!(failed.s().closed_form.segments_built() > built, "the failed instant built the segment again (stat 8 keeps it)");
    assert_eq!(state_digest(failed.s()), before, "the failed instant is rolled back");
    for k in 1..=20 {
        let t = 1e-3 + 1e-4 * k as f64;
        failed.advance(t).unwrap();
        twin.advance(t).unwrap();
        assert_eq!(state_digest(failed.s()), state_digest(twin.s()), "at {t:e}");
    }
}

// ---- Rails point 4: the CC rail is the exact coupled solution (D10) ----------

/// `x' = A x + b` from `x0` after `t`, exactly, for a 2x2 `A` with real,
/// distinct eigenvalues `l1`, `l2` (Sylvester: `e^(At) = (e^(l1 t)(A - l2) -
/// e^(l2 t)(A - l1)) / (l1 - l2)`). The references below are the circuits'
/// own linear ODEs solved this way, never the core's segment.
fn lin2(a: [[f64; 2]; 2], b: [f64; 2], x0: [f64; 2], t: f64) -> [f64; 2] {
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    let x_inf = [(a[0][1] * b[1] - a[1][1] * b[0]) / det, (a[1][0] * b[0] - a[0][0] * b[1]) / det];
    let tr = a[0][0] + a[1][1];
    let disc = (tr * tr / 4.0 - det).sqrt();
    let (l1, l2) = (tr / 2.0 + disc, tr / 2.0 - disc);
    let d = [x0[0] - x_inf[0], x0[1] - x_inf[1]];
    let shifted = |l: f64| [(a[0][0] - l) * d[0] + a[0][1] * d[1], a[1][0] * d[0] + (a[1][1] - l) * d[1]];
    let (p, q) = (shifted(l2), shifted(l1));
    let (e1, e2) = ((l1 * t).exp(), (l2 * t).exp());
    [x_inf[0] + (e1 * p[0] - e2 * q[0]) / (l1 - l2), x_inf[1] + (e1 * p[1] - e2 * q[1]) / (l1 - l2)]
}

/// The root of a rising `f` in `[lo, hi]`, to the last bit.
fn rising_root(f: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    assert!(f(lo) < 0.0 && f(hi) >= 0.0, "no crossing in [{lo:e}, {hi:e}]");
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) < 0.0 { lo = mid } else { hi = mid }
    }
    hi
}

/// How many W0 tolerances (amps: 1 nA plus 1e-3 relative) `got` is off `want`.
fn amp_tols(got: f64, want: f64) -> f64 {
    (got - want).abs() / (1e-9 + 1e-3 * want.abs())
}

/// How many W0 tolerances (volts: 100 uV plus 1e-3 relative) `got` is off `want`.
fn volt_tols(got: f64, want: f64) -> f64 {
    (got - want).abs() / (1e-4 + 1e-3 * want.abs())
}

#[test]
fn fixture_45s_rail_is_the_exact_coupled_solution_through_both_exits_and_their_esr_c_windows() {
    // Fixture 45: {5 V, 0.1 A}, 100 ohm on the rail, 25 ohm behind a switch
    // (closed 80-110 ms), 1 mF at esr 0.05 discharged. In CC the capacitor's
    // charge vc obeys C vc' = I_cap = (iLimit - G vc) / (1 + esr G) and the
    // rail is u = (vc + esr iLimit) / (1 + esr G) (F4's coupled form, D10);
    // it leaves CC where u reaches 5 V, and in CV the capacitor relaxes to 5
    // V with esr C = 50 us (rule 2a). G is the rail's load as the core and
    // classic both see it: the node shunts, and the switch's 1 mOhm contact
    // in series with 25 ohm. (The sidecar F4 read D10 from had the switch
    // ideal: 8.2e-5 V on the rail, the second exit 1.1 us late and 2.2 % of
    // the capacitor's current inside its window.)
    let (v, il, c, esr) = (5.0, 0.1, 1e-3, 0.05);
    let p = program(3, vec![psu(0, 1, 0, v, il), resistor(1, 1, 0, 100.0), switch(2, 1, 2, false), resistor(3, 2, 0, 25.0), capacitor(4, 1, 0, c, esr)]);
    let gs = static_part::NODE_RSHUNT_G;
    let g_open = gs + 0.01;
    let g_load = 1.0 / 25.0 + gs;
    let g_closed = g_open + 1000.0 * g_load / (1000.0 + g_load);
    // The exact pieces: (t0, cc, vc0, G).
    let mut pieces: Vec<(f64, bool, f64, f64)> = Vec::new();
    let mut exits = Vec::new();
    let (mut vc, mut cc) = (0.0f64, true);
    for (t0, t1, g) in [(0.0, 0.08, g_open), (0.08, 0.11, g_closed), (0.11, 0.17, g_open)] {
        if !cc && g * v + (v - vc).max(0.0) / esr > il {
            cc = true;
        }
        let mut t = t0;
        while t < t1 {
            pieces.push((t, cc, vc, g));
            if cc {
                let (k, inf, vc_x) = (g / (c * (1.0 + esr * g)), il / g, v * (1.0 + esr * g) - esr * il);
                let exit = if vc < vc_x && inf > vc_x { t - ((vc_x - vc) / (vc - inf)).ln_1p() / k } else { f64::INFINITY };
                if exit <= t1 {
                    (vc, cc, t) = (vc_x, false, exit);
                    exits.push(exit);
                } else {
                    (vc, t) = (inf + (vc - inf) * (-k * (t1 - t)).exp(), t1);
                }
            } else {
                (vc, t) = (v - (v - vc) * (-(t1 - t) / (esr * c)).exp(), t1);
            }
        }
    }
    assert_eq!(exits.len(), 2);
    let exact = |t: f64| {
        let &(t0, cc, vc0, g) = pieces.iter().rev().find(|p| p.0 <= t).unwrap();
        if cc {
            let (k, inf) = (g / (c * (1.0 + esr * g)), il / g);
            let vc = inf + (vc0 - inf) * (-k * (t - t0)).exp();
            ((vc + esr * il) / (1.0 + esr * g), (il - g * vc) / (1.0 + esr * g))
        } else {
            let vc = v - (v - vc0) * (-(t - t0) / (esr * c)).exp();
            (v, (v - vc) / esr)
        }
    };
    let mut rig = Rig::load(&p).unwrap();
    let (mut worst_v, mut worst_i, mut worst_exit): (f64, f64, f64) = (0.0, 0.0, 0.0);
    let mut sample = |rig: &mut Rig, t: f64| {
        rig.advance(t).unwrap();
        let (u, i) = exact(t);
        worst_v = worst_v.max((rig.v(1) - u).abs());
        worst_i = worst_i.max(amp_tols(rig.i(4), i));
        close(rig.v(1), u, 1e-10, &format!("the rail at {t:e}"));
        assert!(amp_tols(rig.i(4), i) < 1e-3, "the capacitor at {t:e}: {:.6e} against {i:.6e}", rig.i(4));
    };
    // Every 100 us (off the phase boundaries), and every 1 us for 50 us past each exit.
    let mut t_next = 0.5e-4;
    for (t_end, toggle_to) in [(0.08, Some(true)), (0.11, Some(false)), (0.17, None)] {
        while t_next < t_end {
            if let Some(&x) = exits.iter().find(|&&x| x > rig.k.time() && x <= t_next) {
                let t_exit = cc_exit(&mut rig, x + 1e-6);
                worst_exit = worst_exit.max((t_exit - x).abs());
                close(t_exit, x, 1e-11, "the exit, where the exact rail reaches 5 V");
                for k in 1..=50 {
                    sample(&mut rig, x + 1e-6 * k as f64);
                }
            }
            if t_next > rig.k.time() {
                sample(&mut rig, t_next);
            }
            t_next += 1e-4;
        }
        if let Some(closed) = toggle_to {
            rig.advance(t_end).unwrap();
            toggle(&mut rig, 2, closed).unwrap();
        }
    }
    eprintln!("fixture 45: worst rail {worst_v:.3e} V, capacitor {worst_i:.3e} W0 tolerances, exits off by {worst_exit:.3e} s");
}

#[test]
fn a_cc_rail_capacitor_with_esr_and_a_closed_form_rc_hub_partition_follow_their_coupled_ode() {
    // {5 V, 0.1 A}, 1 mF at esr 0.05 and 100 ohm on the rail, and 1 k into
    // 10 uF from it (a closed-form hub partition), all discharged. In CC the
    // two charges are one linear system: the rail u = (vc + esr (iLimit +
    // gr vx)) / (1 + esr g0) with g0 every conductance at the rail node, C
    // vc' = (u - vc) / esr, Cx vx' = gr (u - vx) - gs vx. Since round 14
    // (ruling R15 (a), Rails point 4's coupled path) the rail and the RC
    // advance together between solves on that system's own modes
    // (`rails::coupled_modes`), so both are its solution to rounding
    // (measured 5e-14 V each, the exit 2e-13 s off, the rail capacitor's
    // current 4e-6 of W0). The bounds below are round 10's, from when the
    // RC's draw was held frozen between rail events and re-aimed every
    // RAIL_DRIFT_V; they still hold. It leaves CC where u reaches 5 V; then
    // the rail capacitor relaxes with esr C = 50 us and the RC charges from
    // the fixed 5 V.
    let (v, il, c, esr, rl, r, cx) = (5.0, 0.1, 1e-3, 0.05, 100.0, 1000.0, 1e-5);
    let gs = static_part::NODE_RSHUNT_G;
    let gr = 1.0 / r;
    let g0 = gs + 1.0 / rl + gr;
    let k0 = 1.0 + esr * g0;
    let a = [[(1.0 / k0 - 1.0) / (esr * c), (gr / k0) / c], [gr / k0 / cx, (gr * esr * gr / k0 - gr - gs) / cx]];
    let b = [(il / k0) / c, gr * esr * il / k0 / cx];
    let rail = |x: [f64; 2]| (x[0] + esr * (il + gr * x[1])) / k0;
    let t_x = rising_root(|t| rail(lin2(a, b, [0.0, 0.0], t)) - v, 0.0, 0.2);
    let x_x = lin2(a, b, [0.0, 0.0], t_x);
    let exact = |t: f64| {
        if t < t_x {
            let x = lin2(a, b, [0.0, 0.0], t);
            let u = rail(x);
            (u, x[1], (u - x[0]) / esr)
        } else {
            let vc = v - (v - x_x[0]) * (-(t - t_x) / (esr * c)).exp();
            let vx_inf = v * gr / (gr + gs);
            (v, vx_inf + (x_x[1] - vx_inf) * (-(t - t_x) * (gr + gs) / cx).exp(), (v - vc) / esr)
        }
    };
    let p = program(3, vec![psu(0, 1, 0, v, il), capacitor(1, 1, 0, c, esr), resistor(2, 1, 0, rl), resistor(3, 1, 2, r), capacitor(4, 2, 0, cx, 0.0)]);
    let mut rig = Rig::load(&p).unwrap();
    assert_eq!(rig.s().partition_kinds[rig.s().local_index[2].unwrap().0], PartitionKind::ClosedForm, "the RC is a closed-form hub partition");
    let (mut worst_u, mut worst_x, mut worst_i): (f64, f64, f64) = (0.0, 0.0, 0.0);
    let mut sample = |rig: &mut Rig, t: f64| {
        rig.advance(t).unwrap();
        let (u, vx, i) = exact(t);
        worst_u = worst_u.max((rig.v(1) - u).abs());
        worst_x = worst_x.max((rig.v(2) - vx).abs());
        worst_i = worst_i.max(amp_tols(rig.i(1), i));
        close(rig.v(1), u, 2e-6, &format!("the rail at {t:e}"));
        close(rig.v(2), vx, 3e-6, &format!("the RC at {t:e}"));
        // Inside the window the frozen draw's exit lag (49 ns) was 0.99 W0
        // tolerances of the capacitor's current at its start (F1r10's
        // report); on the coupled path the exit is on time to rounding.
        assert!(amp_tols(rig.i(1), i) < 1.0, "the rail capacitor at {t:e}: {:.6e} against {i:.6e}", rig.i(1));
    };
    let mut t = 0.5e-3;
    while t < t_x - 1e-6 {
        sample(&mut rig, t);
        t += 1e-3;
    }
    let t_exit = cc_exit(&mut rig, t_x + 1e-6);
    // RAIL_DRIFT_V over the rail's slope is the frozen draw's bound on the
    // exit; the coupled path lands it 2e-13 s off.
    let slope = (il - g0 * v + gr * x_x[1]) / (c * k0);
    close(t_exit, t_x, rails::RAIL_DRIFT_V / slope, "the exit");
    for k in 1..=50 {
        sample(&mut rig, t_x + 1e-6 * k as f64);
    }
    for t in [0.08, 0.1] {
        sample(&mut rig, t);
    }
    eprintln!("cap + RC: worst rail {worst_u:.3e} V, RC {worst_x:.3e} V, rail capacitor {worst_i:.3e} W0 tolerances, exit off by {:.3e} s, rail events {}", (t_exit - t_x).abs(), rig.hub().stats.events);
}

#[test]
fn two_rail_capacitors_with_different_esr_follow_the_coupled_slow_mode_and_their_rail_stays_within_w0_through_the_fast_one() {
    // {5 V, 0.1 A}, 100 uF at esr 0.05 and 300 uF at esr 0.02, 100 ohm, all
    // discharged. Exactly, the charges are two states, (u - vc_k) / esr_k =
    // C_k vc_k', with u = (vc1 / e1 + vc2 / e2 + iLimit) / (1 / e1 + 1 / e2 +
    // G): a slow mode, and a fast one (tau about 5 us) in which the two
    // capacitors redistribute the current between them. The core's CC rail is
    // one exponential, the slow mode, with the capacitors' C-weighted charge
    // and esr_eff = sum(esr_k C_k^2) / C_rail^2 (Rails point 4); the
    // redistribution transient is not modelled, so inside it only the rail is
    // held, to W0 (it is 8e-6 V off at 0.5 us, where capacitor 1's current is
    // 11 % off). Past it the slow mode is checked to the bounds measured.
    let (c1, e1, c2, e2, rl, v, il) = (1e-4, 0.05, 3e-4, 0.02, 100.0, 5.0, 0.1);
    let g = static_part::NODE_RSHUNT_G + 1.0 / rl;
    let den = 1.0 / e1 + 1.0 / e2 + g;
    let rail = |x: [f64; 2]| (x[0] / e1 + x[1] / e2 + il) / den;
    let a = [[(1.0 / (e1 * den) - 1.0) / (e1 * c1), 1.0 / (e2 * den) / (e1 * c1)], [1.0 / (e1 * den) / (e2 * c2), (1.0 / (e2 * den) - 1.0) / (e2 * c2)]];
    let b = [il / den / (e1 * c1), il / den / (e2 * c2)];
    let t_x = rising_root(|t| rail(lin2(a, b, [0.0, 0.0], t)) - v, 1e-3, 0.1);
    let p = program(2, vec![psu(0, 1, 0, v, il), capacitor(1, 1, 0, c1, e1), capacitor(2, 1, 0, c2, e2), resistor(3, 1, 0, rl)]);
    let mut rig = Rig::load(&p).unwrap();
    let (mut fast_v, mut slow_v, mut slow_i): (f64, f64, f64) = (0.0, 0.0, 0.0);
    let mut times: Vec<f64> = (1..=100).map(|k| 0.5e-6 * k as f64).collect();
    times.extend((1..=27).map(|k| 1e-3 * k as f64));
    for &t in &times {
        rig.advance(t).unwrap();
        assert_eq!(rig.hub().state.reg, Regime::Cc, "CC at {t:e}");
        let x = lin2(a, b, [0.0, 0.0], t);
        let u = rail(x);
        let (i1, i2) = ((u - x[0]) / e1, (u - x[1]) / e2);
        assert!(volt_tols(rig.v(1), u) < 1.0, "the rail at {t:e}: {:.9e} against {u:.9e}", rig.v(1));
        fast_v = fast_v.max(volt_tols(rig.v(1), u));
        if t >= 100e-6 {
            // Twenty fast time constants on: the slow mode alone.
            slow_v = slow_v.max((rig.v(1) - u).abs());
            slow_i = slow_i.max(amp_tols(rig.i(1), i1)).max(amp_tols(rig.i(2), i2));
            close(rig.v(1), u, 1e-8, &format!("the slow mode's rail at {t:e}"));
            assert!(amp_tols(rig.i(1), i1) < 0.05 && amp_tols(rig.i(2), i2) < 0.05, "the slow mode's currents at {t:e}: {:.6e}, {:.6e} against {i1:.6e}, {i2:.6e}", rig.i(1), rig.i(2));
        }
    }
    let t_exit = cc_exit(&mut rig, t_x + 1e-6);
    close(t_exit, t_x, 1e-9, "the exit, where the exact rail reaches 5 V");
    eprintln!("two caps: worst rail {fast_v:.3e} W0 tolerances overall; past 100 us rail {slow_v:.3e} V, currents {slow_i:.3e} W0 tolerances; exit off by {:.3e} s", (t_exit - t_x).abs());
}

// ---- V9 MEDIUM 1: a CC-driven curved segment's rail coupling (Rails point 4) -

/// V9's P1d board: a gate on a `{5 V, il}` PSU ramping 100 uF from 4.3 V in
/// CC, its 1a/1b on node 4 (1 kohm to ground, a switch to a fixed 5 V at node
/// 5, closed at load when `closed`), 1y (node 2) into `c`: straight on node 2
/// when `r` is 0, else through `r` (device 6) on node 3. A second gate on the
/// rail reads node 2 (its 1y on node 6) when `reader`, so the partition
/// schedules crossings there. R_out follows the rail, so the partition is a
/// curved closed-form hub partition driven by the CC rail's closed form.
/// Returns the program, the capacitor's node and its device.
fn cc_gate_into_c(il: f64, io: f64, r: f64, c: f64, closed: bool, reader: bool) -> (Program, usize, usize) {
    let mut devs = vec![
        psu(0, 1, 0, 5.0, il),
        capacitor(1, 1, 0, 1e-4, 0.0),
        gate_io(2, 1, 0, [4, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 3.0, 6.0),
        source(3, 5, 0, 5.0),
        switch(4, 5, 4, closed),
        resistor(5, 4, 0, 1000.0),
    ];
    let (node, dev) = if r > 0.0 {
        devs.push(resistor(6, 2, 3, r));
        devs.push(capacitor(7, 3, 0, c, 0.0));
        (3, 7)
    } else {
        devs.push(capacitor(7, 2, 0, c, 0.0));
        (2, 6)
    };
    if reader {
        let mut gate = gate_io(8, 1, 0, [2, 2, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 3.0, 6.0);
        gate.blob[0] = 4.0;
        devs.push(gate);
    }
    let mut p = no_cache(program(7, devs));
    p.digital_slot_count = p.digital_slot_count.max(16);
    (p, node, dev)
}

/// Node 2 of [`cc_gate_into_c`] at rail `u` with the capacitor at `vc`, and
/// 1y's current out of the rail (0 while low).
fn cc_gate_node2(u: f64, vc: f64, io: f64, r: f64, high: bool) -> (f64, f64) {
    let gs = static_part::NODE_RSHUNT_G;
    let (g, t) = (gate_g(u, io), if high { u } else { 0.0 });
    let v2 = if r > 0.0 { (g * t + vc / r) / (g + 1.0 / r + gs) } else { vc };
    (v2, if high { g * (u - v2) } else { 0.0 })
}

/// The board's own ODE (RK4) from `(u, vc)` at `t0` to `t1` in steps of at
/// most `h`: `C_rail u' = il - gs u - i_rail` and `c vc' = i_c - gs vc`, with
/// node 2 at its static solution, `i_c` the current into `c` (1y's, or `r`'s)
/// and `i_rail` 1y's current out of the rail. The ICs draw nothing else.
#[allow(clippy::too_many_arguments)]
fn cc_gate_into_c_rk4(x: [f64; 2], t0: f64, t1: f64, h: f64, il: f64, io: f64, r: f64, c: f64, high: bool) -> [f64; 2] {
    let gs = static_part::NODE_RSHUNT_G;
    let rhs = |x: [f64; 2]| {
        let (u, vc) = (x[0], x[1]);
        let (v2, i_rail) = cc_gate_node2(u, vc, io, r, high);
        let i_c = if r > 0.0 { (v2 - vc) / r } else { gate_g(u, io) * (if high { u } else { 0.0 } - vc) };
        [(il - gs * u - i_rail) / 1e-4, (i_c - gs * vc) / c]
    };
    let n = ((t1 - t0) / h).ceil().max(1.0) as usize;
    let dt = (t1 - t0) / n as f64;
    let mut x = x;
    let add = |a: [f64; 2], b: [f64; 2], s: f64| [a[0] + s * b[0], a[1] + s * b[1]];
    for _ in 0..n {
        let k1 = rhs(x);
        let k2 = rhs(add(x, k1, 0.5 * dt));
        let k3 = rhs(add(x, k2, 0.5 * dt));
        let k4 = rhs(add(x, k3, dt));
        x = [x[0] + dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0, x[1] + dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0];
    }
    x
}

#[test]
fn a_cc_driven_curved_segment_follows_its_coupled_ode_on_a_coarse_host() {
    // V9 MEDIUM 1 (probes P1c/P1d). Under a CC rail capacitor a curved
    // closed-form segment follows the rail's closed form between its aims;
    // its current `i0(u) + di(u) vc` couples the rail and `vc` through
    // `di1du (u - u_a) vc`. Held at the aim's `vc`, that term is a constant
    // drive: after a discharge `vc` settled near `di1du (u - u_a) vc_a /
    // |di|` instead of 0 (0.64 mV, 6.39x W0, at 1 mA, 1 nF, a host every 64
    // us; its current 4411x). The crossing scheduler holds a curved node's
    // sensitivity at `vc_a` too, so a crossing on node 2 behind `r` lands
    // its capacitor off its trajectory. A host that advances only every
    // `gap` leaves the segment on one aim between frames. At each frame, the
    // rail, the capacitor's voltage and current and node 2 are within W0's
    // tolerance of the board's own ODE (RK4, the rail capacitor and `c`),
    // from the published state just after 1y switched. Both directions: 1y
    // falling (the switch closes) discharges `c`; 1y rising charges it along
    // the ramp. The 0.1 A rows also catch a segment band that checks only one
    // end of `vc`'s range (seed S8, 158x at 16 us).
    let mut worst = (0.0f64, String::new());
    for (il, io, r, c, gap, fall) in [
        (0.001, 0.008, 0.0, 1e-9, 64e-6, true),
        (0.001, 0.008, 0.0, 1e-9, 16e-6, true),
        (0.01, 0.008, 0.0, 1e-9, 64e-6, true),
        (0.001, 0.016, 0.0, 1e-8, 256e-6, true),
        (0.1, 0.008, 0.0, 1e-9, 16e-6, true),
        (0.1, 0.008, 0.0, 1e-8, 16e-6, true),
        (0.1, 0.008, 0.0, 3e-8, 16e-6, true),
        (0.03, 0.008, 0.0, 1e-7, 64e-6, true),
        (0.1, 0.008, 150.0, 1e-6, 16e-6, true),
        (0.1, 0.008, 47.0, 1e-7, 16e-6, true),
        (0.001, 0.008, 0.0, 1e-9, 64e-6, false),
        (0.1, 0.008, 0.0, 1e-8, 16e-6, false),
        (0.1, 0.008, 150.0, 1e-6, 16e-6, false),
    ] {
        let name = format!("il {il} io {io} r {r} c {c:e} gap {gap:e} {}", if fall { "falling" } else { "rising" });
        let (p, cn, cd) = cc_gate_into_c(il, io, r, c, !fall, true);
        let mut rig = Rig::load_carrying(&p, &[(1, 4.3)]).unwrap();
        let part = rig.s().local_index[cn].unwrap().0;
        assert_eq!(rig.s().partition_kinds[part], PartitionKind::ClosedForm, "{name}: a closed-form hub partition");
        let tau = (r + 1.0 / gate_g(4.3, io)) * c;
        let t_sw = (20.0 * tau).min(200e-6);
        rig.advance(t_sw).unwrap();
        toggle(&mut rig, 4, fall).unwrap();
        // 1y switches one propagation delay (10 ns) after the switch.
        let t0 = t_sw + 2e-8;
        rig.advance(t0).unwrap();
        assert!(rig.s().closed_form.hub_band(part).is_some() && rig.s().closed_form.is_driven(part), "{name}: curved and driven");
        let (mut t, mut x) = (t0, [rig.u(), rig.v(cn)]);
        let (mut frames, crossed) = (0, rig.s().closed_form.crossings());
        let mut row = [0.0f64; 4];
        while frames < 200 {
            let t1 = t + gap;
            x = cc_gate_into_c_rk4(x, t, t1, tau / 40.0, il, io, r, c, !fall);
            rig.advance(t1).unwrap();
            t = t1;
            if rig.hub().state.reg != Regime::Cc {
                break;
            }
            frames += 1;
            let (u, vc) = (rig.u(), x[1]);
            let gs = static_part::NODE_RSHUNT_G;
            let (v2, _) = cc_gate_node2(x[0], vc, io, r, !fall);
            let i_c = if r > 0.0 { (v2 - vc) / r } else { gate_g(x[0], io) * (if fall { 0.0 } else { x[0] } - vc) } - gs * vc;
            for (k, (what, ratio, got, want)) in [("rail", volt_tols(u, x[0]), u, x[0]), ("vc", volt_tols(rig.v(cn), vc), rig.v(cn), vc), ("node 2", volt_tols(rig.v(2), v2), rig.v(2), v2), ("current", amp_tols(rig.i(cd), i_c), rig.i(cd), i_c)].into_iter().enumerate() {
                row[k] = row[k].max(ratio);
                if ratio > worst.0 {
                    worst = (ratio, format!("{name}: {what} at {t:.4e} u {u:.6}: published {got:.6e}, ode {want:.6e}"));
                }
            }
        }
        assert!(frames >= 10, "{name}: {frames} frames in CC");
        eprintln!("{name}: {frames} frames, rail events {}, crossings {}; W0 tolerances: rail {:.4}, vc {:.4}, node 2 {:.4}, current {:.4}", rig.hub().stats.events, rig.s().closed_form.crossings() - crossed, row[0], row[1], row[2], row[3]);
    }
    eprintln!("cc-driven coupling: worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}

#[test]
fn a_cc_driven_discharge_settles_at_zero_between_coarse_frames() {
    // V9's P1d as it stands: the board of `a_cc_driven_curved_segment_
    // follows_its_coupled_ode_on_a_coarse_host` without the reader, 1y
    // falling into `c` straight on node 2, 20 time constants into the CC
    // ramp, then a host every `gap`. Past 100 time constants the true `vc`
    // is 0 to e^-100; at every such frame the published `vc` is within W0's
    // tolerance of it. Held at the aim's `vc`, the rail coupling left it at
    // 0.64 mV (6.39x W0) at 1 mA and 64 us; a segment band that checks only
    // one end of `vc`'s range (seed S8) left 158x at 0.1 A and 16 us.
    let mut worst = (0.0f64, String::new());
    for (il, io, c, gap) in [(0.001, 0.008, 1e-9, 64e-6), (0.001, 0.008, 1e-9, 16e-6), (0.1, 0.008, 1e-9, 16e-6), (0.01, 0.008, 1e-9, 64e-6), (0.001, 0.016, 1e-8, 1e-3)] {
        let (p, cn, _) = cc_gate_into_c(il, io, 0.0, c, false, false);
        let mut rig = Rig::load_carrying(&p, &[(1, 4.3)]).unwrap();
        let tau = c / gate_g(4.3, io);
        let t_sw = 20.0 * tau;
        rig.advance(t_sw).unwrap();
        toggle(&mut rig, 4, true).unwrap();
        let (mut t, mut seen) = (t_sw, 0);
        for _ in 0..200 {
            t += gap;
            rig.advance(t).unwrap();
            if rig.hub().state.reg != Regime::Cc {
                break;
            }
            if t - t_sw > 100.0 * tau {
                let ratio = rig.v(cn).abs() / 1e-4;
                if ratio > worst.0 {
                    worst = (ratio, format!("il {il} io {io} c {c:e} gap {gap:e}: vc {:.6e} at {t:.4e} u {:.6}", rig.v(cn), rig.u()));
                }
                seen += 1;
            }
        }
        assert!(seen >= 5, "il {il} c {c:e} gap {gap:e}: {seen} frames");
    }
    eprintln!("CC-driven discharge: worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}

#[test]
fn a_curved_segments_aim_adds_no_coupling_error_once_earlier_aims_used_a_tolerance_up() {
    // V11 finding 5. Each aim of a CC-driven curved segment may add to the
    // coupling error earlier aims left only what they left of each
    // quantity's tolerance (`couple_band`). 1y high (through `r`) charges
    // `c` from 0 along the 4.3 V rail's CC ramp, and its current falls
    // toward 0, where W0's tolerance is its absolute 1 nA: the error the
    // inrush's aims left, decaying with the segment, is past that before it
    // is gone (the 768-board sweep's first such boards, whose host first
    // looks a microsecond in). An aim that met it still spent the whole
    // tolerance again (`tol * pace`): 165 checks in that sweep, the
    // inherited bound alone up to 1.34x the tolerance. Every check that
    // meets a tolerance used up must now fail, and end the aim there, unless
    // the aim adds nothing. The board itself stays within W0 of its own ODE
    // (RK4) at every frame.
    for r in [22.0, 0.0] {
        let (il, io, c, gap) = (0.001, 0.008, 1e-9, 1e-6);
        let (p, cn, cd) = cc_gate_into_c(il, io, r, c, false, true);
        let mut rig = Rig::load_carrying(&p, &[(1, 4.3)]).unwrap();
        let _ = closed_form::hook::take();
        let tau = (r + 1.0 / gate_g(4.3, io)) * c;
        let (mut t, mut x) = (0.0, [rig.u(), rig.v(cn)]);
        let mut worst = (0.0f64, String::new());
        for _ in 0..40 {
            let t1 = t + gap;
            x = cc_gate_into_c_rk4(x, t, t1, tau / 40.0, il, io, r, c, true);
            rig.advance(t1).unwrap();
            t = t1;
            assert_eq!(rig.hub().state.reg, Regime::Cc, "r {r}: CC throughout");
            let gs = static_part::NODE_RSHUNT_G;
            let (v2, _) = cc_gate_node2(x[0], x[1], io, r, true);
            let i_c = if r > 0.0 { (v2 - x[1]) / r } else { gate_g(x[0], io) * (x[0] - x[1]) } - gs * x[1];
            for (what, ratio) in [("rail", volt_tols(rig.u(), x[0])), ("vc", volt_tols(rig.v(cn), x[1])), ("node 2", volt_tols(rig.v(2), v2)), ("current", amp_tols(rig.i(cd), i_c))] {
                if ratio > worst.0 {
                    worst = (ratio, format!("{what} at {t:.4e}"));
                }
            }
        }
        let spent = closed_form::hook::take();
        eprintln!("r {r}: {spent:?}, rail events {}, worst {:.4} of W0 at {}", rig.hub().stats.events, worst.0, worst.1);
        assert!(spent.exhausted > 0, "r {r}: the inrush's aims used a tolerance up");
        assert_eq!(spent.granted, 0, "r {r}: an aim added coupling error to a tolerance earlier aims had used up");
        assert!(worst.0 <= 1.0, "r {r}: {} of W0's tolerance at {}", worst.0, worst.1);
    }
}

#[test]
fn a_cc_ramps_drift_leaves_a_curved_segments_published_quantities_within_w0_on_a_coarse_host() {
    // R12 (F1r11's sweep), held since round 14 by ruling R15 (a). When a CC
    // rail capacitor's segment held the closed-form draws where they stood
    // between two solves, the rail it drove a curved partition with drifted
    // off the board's, the partition's capacitor followed the drifting rail,
    // and the solve at the frame put the rail back but not `vc`: 1y's
    // tracking current erred by its slope in `vc` times the drift (at 10 uV
    // and R_out 50 ohm, 200 nA against W0's 1 nA; 160x at 1 us frames). Now
    // the rail and the partition advance on their coupled path, and only the
    // curved segment's bilinear term is left, which `couple_band` bounds.
    // These are the sweep's worst boards (1y rising, `c` charged along the
    // ramp just after its own transient moved the draws) and others of the
    // 83 over W0: every drive strength, both `r` kinds and long frames. At
    // each frame the rail, `vc`, node 2, the capacitor's current and `r`'s
    // are within W0's tolerance of the board's own ODE (RK4) from the
    // published state just after 1y switched.
    let mut worst = (0.0f64, String::new());
    for (il, io, r, c, gap) in [
        (0.001, 0.024, 0.0, 1e-9, 1e-6),
        (0.01, 0.024, 0.0, 1e-9, 1e-6),
        (0.1, 0.024, 0.0, 1e-9, 1e-6),
        (0.001, 0.024, 0.0, 1e-7, 64e-6),
        (0.001, 0.024, 22.0, 1e-9, 1e-6),
        (0.001, 0.024, 0.0, 1e-7, 256e-6),
        (0.001, 0.008, 150.0, 1e-8, 64e-6),
        (0.001, 0.008, 1000.0, 1e-8, 64e-6),
        (0.1, 0.024, 0.0, 1e-9, 16e-6),
        (0.01, 0.008, 0.0, 1e-9, 1e-6),
    ] {
        let name = format!("il {il} io {io} r {r} c {c:e} gap {gap:e}");
        let (p, cn, cd) = cc_gate_into_c(il, io, r, c, true, true);
        let mut rig = Rig::load_carrying(&p, &[(1, 4.3)]).unwrap();
        let part = rig.s().local_index[cn].unwrap().0;
        let tau = (r + 1.0 / gate_g(4.3, io)) * c;
        let t_sw = (20.0 * tau).min(200e-6);
        rig.advance(t_sw).unwrap();
        toggle(&mut rig, 4, false).unwrap();
        // 1y rises one propagation delay (10 ns) after the switch opens.
        let t0 = t_sw + 2e-8;
        rig.advance(t0).unwrap();
        assert!(rig.s().closed_form.hub_band(part).is_some() && rig.s().closed_form.is_driven(part), "{name}: curved and driven");
        let (mut t, mut x) = (t0, [rig.u(), rig.v(cn)]);
        let (mut frames, events) = (0, rig.hub().stats.events);
        let mut row = [0.0f64; 5];
        while frames < 300 {
            let t1 = t + gap;
            x = cc_gate_into_c_rk4(x, t, t1, tau / 40.0, il, io, r, c, true);
            rig.advance(t1).unwrap();
            t = t1;
            if rig.hub().state.reg != Regime::Cc {
                break;
            }
            frames += 1;
            let (u, vc) = (rig.u(), x[1]);
            let gs = static_part::NODE_RSHUNT_G;
            let (v2, _) = cc_gate_node2(x[0], vc, io, r, true);
            let i_r = if r > 0.0 { (v2 - vc) / r } else { gate_g(x[0], io) * (x[0] - vc) };
            let mut checks = vec![("rail", volt_tols(u, x[0]), u, x[0]), ("vc", volt_tols(rig.v(cn), vc), rig.v(cn), vc), ("node 2", volt_tols(rig.v(2), v2), rig.v(2), v2), ("current", amp_tols(rig.i(cd), i_r - gs * vc), rig.i(cd), i_r - gs * vc)];
            if r > 0.0 {
                checks.push(("r's current", amp_tols(rig.i(6), i_r), rig.i(6), i_r));
            }
            for (k, (what, ratio, got, want)) in checks.into_iter().enumerate() {
                row[k] = row[k].max(ratio);
                if ratio > worst.0 {
                    worst = (ratio, format!("{name}: {what} at {t:.4e} u {u:.6}: published {got:.6e}, ode {want:.6e}"));
                }
            }
        }
        assert!(frames >= 10, "{name}: {frames} frames in CC");
        eprintln!("{name}: {frames} frames, rail events {}; W0 tolerances: rail {:.4}, vc {:.4}, node 2 {:.4}, current {:.4}, r's {:.4}", rig.hub().stats.events - events, row[0], row[1], row[2], row[3], row[4]);
    }
    eprintln!("cc ramp drift: worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}

#[test]
fn a_curved_segment_at_its_equilibrium_on_a_draining_cell_publishes_its_resistor_current_within_w0() {
    // V9 MEDIUM 2 (probe P2). A gate on a draining 5 V cell (0.5 ohm, 1e-7
    // Ah, 100 ohm load) drives 1y high through `r` into `c` (a curved
    // closed-form hub partition). A small `c` sits at its equilibrium, `vc`
    // near the rail, where the segment's sensitivities' `vc` terms (`dx1du
    // vc`, `di1du vc`, `g1 vc`) are as large as they get: without them (seed
    // S1) the resistor's current there ran 1362x W0, which the farad of
    // `a_closed_form_hub_partition_holding_a_rail_supplied_gate_follows_the_
    // rail_within_w0` (vc near 0) cannot see. Every 2 us for 2 ms, node 2
    // and the resistor's current are within W0's tolerance of the exact
    // solution at the published rail with node 3 at its published voltage
    // (the capacitor a source).
    let gs = static_part::NODE_RSHUNT_G;
    let mut worst = (0.0f64, String::new());
    for (io, r, c) in [(0.008, 22.0, 1e-7), (0.008, 68.0, 1e-6)] {
        let mut devs = d7_supply(false);
        devs.extend([gate_io(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 3.0, 6.0), resistor(3, 2, 3, r), capacitor(4, 3, 0, c, 0.0)]);
        let mut rig = Rig::load(&no_cache(program(4, devs))).unwrap();
        let part = rig.s().local_index[3].unwrap().0;
        assert_eq!(rig.s().partition_kinds[part], PartitionKind::ClosedForm, "a closed-form hub partition");
        let name = format!("io {io} r {r} c {c:e}");
        let mut seen = 0;
        for k in 1..=1000 {
            let t = 2e-6 * k as f64;
            rig.advance(t).unwrap();
            let u = rig.v(1);
            if u < 0.9 * 3.0 + 0.1 {
                break;
            }
            let v3 = rig.v(3);
            let g = gate_g(u, io);
            let o = (g * u + v3 / r) / (g + 1.0 / r + gs);
            for (what, ratio, got, want) in [("node 2", volt_tols(rig.v(2), o), rig.v(2), o), ("resistor", amp_tols(rig.i(3), (o - v3) / r), rig.i(3), (o - v3) / r)] {
                if ratio > worst.0 {
                    worst = (ratio, format!("{name}: {what} at {t:.4e} u {u:.6} vc {v3:.6}: published {got:.6e}, exact {want:.6e}"));
                }
            }
            seen += 1;
        }
        assert!(seen > 500, "{name}: {seen} samples");
    }
    eprintln!("gate RC at equilibrium on a draining cell: worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}

#[test]
fn a_curved_segment_keeps_its_capacitor_inside_the_range_its_band_was_sized_over() {
    // V9 LOW. A curved closed-form segment's band is sized at the two ends of
    // the capacitor voltages it can reach (`curvature::segment_band`): from
    // `vc` at the build toward its asymptote, which moves with the rail. The
    // reach took the asymptote's motion as `di0du / di`, the sensitivity at
    // `vc = 0`, where it is `(di0du + di1du inf) / di`; and a driver on a
    // clamped R_out piece is affine up to its kink, a band wider than the
    // reach was sized for. A gate on a draining 5 V cell (0.5 ohm, 1e-7 Ah,
    // 100 ohm) drives 1y high through `r` into `c`; at 16 mA R_out sits on
    // its 50 ohm clamp below 3.2 V, and `vc` ran 0.137 V past its range. At
    // every sample down to the gate's power floor, and on a {5 V, 0.1 A} CC
    // ramp, the published `vc` is inside the range of the segment it is
    // published from.
    let mut worst = (0.0f64, String::new());
    for (ramp, io, r, c) in [(false, 0.016, 22.0, 1e-7), (false, 0.008, 22.0, 1e-7), (false, 0.008, 150.0, 1e-6), (true, 0.008, 22.0, 1e-6), (true, 0.016, 22.0, 1e-7)] {
        let mut devs = d7_supply(ramp);
        devs.extend([gate_io(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 3.0, 6.0), resistor(3, 2, 3, r), capacitor(4, 3, 0, c, 0.0)]);
        let carry = if ramp { vec![(1, 4.3)] } else { Vec::new() };
        let mut rig = Rig::load_carrying(&no_cache(program(4, devs)), &carry).unwrap();
        let part = rig.s().local_index[3].unwrap().0;
        let name = format!("{} io {io} r {r} c {c:e}", if ramp { "CC ramp" } else { "cell" });
        let mut seen = 0;
        for k in 1..=1500 {
            let t = 4e-6 * k as f64;
            rig.advance(t).unwrap();
            let u = rig.v(1);
            if u < 0.9 * 3.0 + 0.1 || (ramp && rig.hub().state.reg != Regime::Cc) {
                break;
            }
            let Some((lo, hi)) = rig.s().closed_form.hub_range(part).filter(|r| r.0.is_finite() && r.1.is_finite()) else { continue };
            let vc = rig.v(3);
            let out = (lo - vc).max(vc - hi);
            if out > worst.0 {
                worst = (out, format!("{name}: vc {vc:.6} at {t:.4e} u {u:.6}, range [{lo:.6}, {hi:.6}]"));
            }
            seen += 1;
        }
        assert!(seen > 50, "{name}: {seen} samples");
    }
    eprintln!("capacitor against its range: worst {:.3e} V outside at {}", worst.0, worst.1);
    assert!(worst.0 <= 1e-12, "{:.3e} V outside the range at {}", worst.0, worst.1);
}

#[test]
fn a_host_stepping_nanoseconds_through_a_cc_ramp_is_not_a_stall() {
    // V9 MEDIUM 3 (NP1's class, a false decline). A {5 V, 10 mA} PSU ramps
    // 100 uF from 4.3 V in CC; a gate on the rail holds 1y low into 1 nF.
    // A host advancing every few nanoseconds re-solves the rail at each
    // horizon, and each solve supersedes the pending `Rail` event with a new
    // version due at the same threshold; at u = 4.32 V about 60,000 of those
    // stale versions popped within 1 ps of one another, each run as an
    // instant, and the no-progress guard counted them and declined with -4.
    // A superseded event is no instant: the run goes on to 300 us in 3 and 7
    // ns steps, past 4.32 V, still in CC.
    let (p, cn, _) = cc_gate_into_c(0.01, 0.008, 0.0, 1e-9, false, false);
    let mut rig = Rig::load_carrying(&p, &[(1, 4.3)]).unwrap();
    let t_sw = 20.0 * 1e-9 / gate_g(4.3, 0.008);
    rig.advance(t_sw).unwrap();
    toggle(&mut rig, 4, true).unwrap();
    let (mut t, mut k) = (t_sw, 0u64);
    while t < 3e-4 {
        t += if k % 2 == 0 { 3e-9 } else { 7e-9 };
        k += 1;
        if let Err(e) = rig.advance(t) {
            panic!("advancing to {t:.9e} (step {k}) at u {:.9}: {e:?}", rig.u());
        }
    }
    assert_eq!(rig.hub().state.reg, Regime::Cc, "still in CC");
    assert!(rig.u() > 4.32 && rig.v(cn).abs() < 1e-4, "the rail ramped on (u {}) and 1 nF stayed discharged ({})", rig.u(), rig.v(cn));
}

/// The exact solution of `x' = A x + b` from `x0` after `t`, for a real `A`
/// with distinct real eigenvalues: `x0 + f(A) (A x0 + b)`, `f(λ) = (e^(λt) -
/// 1) / λ` (Sylvester). Taken about `x0`, never through the equilibrium, which
/// the node shunts put a gigavolt out on a CC rail.
fn lin2_about(a: [[f64; 2]; 2], b: [f64; 2], x0: [f64; 2], t: f64) -> [f64; 2] {
    let (tr, det) = (a[0][0] + a[1][1], a[0][0] * a[1][1] - a[0][1] * a[1][0]);
    let l1 = 0.5 * tr - (0.25 * tr * tr - det).sqrt();
    let l2 = det / l1;
    let f = |l: f64| if l == 0.0 { t } else { (l * t).exp_m1() / l };
    let (f1, f2, w) = (f(l1), f(l2), l1 - l2);
    let m = |i: usize, j: usize| {
        let id = if i == j { 1.0 } else { 0.0 };
        (f1 * (a[i][j] - l2 * id) - f2 * (a[i][j] - l1 * id)) / w
    };
    let r = [a[0][0] * x0[0] + a[0][1] * x0[1] + b[0], a[1][0] * x0[0] + a[1][1] * x0[1] + b[1]];
    [x0[0] + m(0, 0) * r[0] + m(0, 1) * r[1], x0[1] + m(1, 0) * r[0] + m(1, 1) * r[1]]
}

#[test]
fn a_cc_ramps_drift_leaves_a_linear_closed_form_rcs_published_quantities_within_w0_on_a_coarse_host() {
    // R13 (a) (F1r12's 108 boards), held since round 14 by ruling R15 (a). A
    // {5 V, iLimit} PSU ramps 100 uF from 4.3 V in CC; `r` from the rail into
    // `c` is a linear closed-form hub partition. When the rail's segment held
    // its draw where it stood at its aim, the board's rail drifted off the
    // rail that drove the partition until the drift watch solved it again,
    // and the solve put the rail back but not `vc`: `c`'s current erred by up
    // to 10 uV over 50 ohm, 200 nA against W0's 1 nA where it had settled
    // (99x at 16 us frames). Now the two are one linear system advanced on
    // its own modes, exact to rounding. These are 8 of the 27 boards that
    // were over W0, every iLimit and `r`. At each frame the rail, `vc`, and
    // `r`'s and `c`'s currents are within W0's tolerance of the board's own
    // linear ODE, solved exactly from the published state at load.
    let gs = static_part::NODE_RSHUNT_G;
    let mut worst = (0.0f64, String::new());
    for (il, r, c, gap) in [
        (0.001, 50.0, 1e-8, 16e-6),
        (0.001, 150.0, 1e-7, 64e-6),
        (0.001, 50.0, 1e-7, 64e-6),
        (0.001, 1000.0, 1e-8, 64e-6),
        (0.01, 50.0, 1e-8, 16e-6),
        (0.01, 150.0, 1e-9, 1e-6),
        (0.1, 50.0, 1e-9, 16e-6),
        (0.001, 50.0, 1e-6, 64e-6),
    ] {
        let name = format!("il {il} r {r} c {c:e} gap {gap:e}");
        let p = no_cache(program(3, vec![psu(0, 1, 0, 5.0, il), capacitor(1, 1, 0, 1e-4, 0.0), resistor(2, 1, 2, r), capacitor(3, 2, 0, c, 0.0)]));
        let mut rig = Rig::load_carrying(&p, &[(1, 4.3)]).unwrap();
        let part = rig.s().local_index[2].unwrap().0;
        assert_eq!(rig.s().partition_kinds[part], PartitionKind::ClosedForm, "{name}: a closed-form hub partition");
        assert!(rig.s().closed_form.hub_band(part).is_none(), "{name}: linear");
        let a = [[-(gs + 1.0 / r) / 1e-4, 1.0 / (r * 1e-4)], [1.0 / (r * c), -(1.0 / r + gs) / c]];
        let (mut t, mut x) = (0.0, [rig.u(), rig.v(2)]);
        let (mut frames, events) = (0, rig.hub().stats.events);
        let mut row = [0.0f64; 4];
        while frames < 300 {
            x = lin2_about(a, [il / 1e-4, 0.0], x, gap);
            t += gap;
            rig.advance(t).unwrap();
            if rig.hub().state.reg != Regime::Cc {
                break;
            }
            frames += 1;
            let i_r = (x[0] - x[1]) / r;
            let checks = [("rail", volt_tols(rig.u(), x[0]), rig.u(), x[0]), ("vc", volt_tols(rig.v(2), x[1]), rig.v(2), x[1]), ("r's current", amp_tols(rig.i(2), i_r), rig.i(2), i_r), ("c's current", amp_tols(rig.i(3), i_r - gs * x[1]), rig.i(3), i_r - gs * x[1])];
            for (k, (what, ratio, got, want)) in checks.into_iter().enumerate() {
                row[k] = row[k].max(ratio);
                if ratio > worst.0 {
                    worst = (ratio, format!("{name}: {what} at {t:.4e} u {:.6}: published {got:.6e}, exact {want:.6e}", rig.u()));
                }
            }
        }
        assert!(frames >= 10, "{name}: {frames} frames in CC");
        eprintln!("{name}: {frames} frames, rail events {}; W0 tolerances: rail {:.4}, vc {:.4}, r's {:.4}, c's {:.4}", rig.hub().stats.events - events, row[0], row[1], row[2], row[3]);
    }
    eprintln!("cc ramp drift, linear: worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}

/// `x' = A x + b` by RK4 from `x` over `t` in steps of at most `h`.
fn rk4_linear(a: &[Vec<f64>], b: &[f64], x: &[f64], t: f64, h: f64) -> Vec<f64> {
    let n = x.len();
    let rhs = |x: &[f64]| (0..n).map(|i| b[i] + (0..n).map(|j| a[i][j] * x[j]).sum::<f64>()).collect::<Vec<f64>>();
    let add = |x: &[f64], k: &[f64], s: f64| x.iter().zip(k).map(|(p, q)| p + s * q).collect::<Vec<f64>>();
    let steps = (t / h).ceil().max(1.0) as usize;
    let dt = t / steps as f64;
    let mut x = x.to_vec();
    for _ in 0..steps {
        let k1 = rhs(&x);
        let k2 = rhs(&add(&x, &k1, 0.5 * dt));
        let k3 = rhs(&add(&x, &k2, 0.5 * dt));
        let k4 = rhs(&add(&x, &k3, dt));
        for i in 0..n {
            x[i] += dt * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
        }
    }
    x
}

#[test]
fn two_rcs_heavier_than_the_rail_capacitor_follow_their_coupled_path_within_w0() {
    // Ruling R15 (a). A {5 V, 1 mA} PSU ramps 1 uF from 4.3 V in CC with two
    // closed-form hub partitions off the rail: 2 uF behind 50 ohm, twice the
    // rail's own capacitance, and 100 nF behind 1 kohm, both discharged at
    // load. Each one's draw moves the rail as much as the rail moves it:
    // holding a partition's draw while the rail moved, or feeding each the
    // other's frozen response once (a single Picard pass, whose error here
    // is the capacitance ratio, 2, of what it corrects), is no path at all.
    // The rail and both partitions advance on the system's own modes, so at
    // every frame the rail, both `vc` and both resistors' currents are within
    // W0's tolerance of the board's linear ODE (RK4), and only the rail's
    // thresholds (the scan widths) solve it between frames.
    let gs = static_part::NODE_RSHUNT_G;
    let (il, cr, r1, c1, r2, c2) = (1e-3, 1e-6, 50.0, 2e-6, 1e3, 1e-7);
    let p = no_cache(program(5, vec![psu(0, 1, 0, 5.0, il), capacitor(1, 1, 0, cr, 0.0), resistor(2, 1, 2, r1), capacitor(3, 2, 0, c1, 0.0), resistor(4, 1, 3, r2), capacitor(5, 3, 0, c2, 0.0)]));
    let mut rig = Rig::load_carrying(&p, &[(1, 4.3)]).unwrap();
    for n in [2, 3] {
        let part = rig.s().local_index[n].unwrap().0;
        assert_eq!(rig.s().partition_kinds[part], PartitionKind::ClosedForm, "node {n}: a closed-form hub partition");
    }
    assert!(rig.hub().state.cap_seg.as_ref().is_some_and(|s| s.modes.is_some()), "the coupled path");
    let a = vec![
        vec![-(gs + 1.0 / r1 + 1.0 / r2) / cr, 1.0 / (r1 * cr), 1.0 / (r2 * cr)],
        vec![1.0 / (r1 * c1), -(1.0 / r1 + gs) / c1, 0.0],
        vec![1.0 / (r2 * c2), 0.0, -(1.0 / r2 + gs) / c2],
    ];
    let b = [il / cr, 0.0, 0.0];
    let mut x = vec![rig.u(), rig.v(2), rig.v(3)];
    let (mut t, gap, events) = (0.0, 2e-6, rig.hub().stats.events);
    let mut worst = (0.0f64, String::new());
    let mut frames = 0;
    while frames < 300 {
        x = rk4_linear(&a, &b, &x, gap, 1e-9);
        t += gap;
        rig.advance(t).unwrap();
        if rig.hub().state.reg != Regime::Cc {
            break;
        }
        frames += 1;
        let (i1, i2) = ((x[0] - x[1]) / r1, (x[0] - x[2]) / r2);
        let checks = [
            ("rail", volt_tols(rig.u(), x[0]), rig.u(), x[0]),
            ("vc1", volt_tols(rig.v(2), x[1]), rig.v(2), x[1]),
            ("vc2", volt_tols(rig.v(3), x[2]), rig.v(3), x[2]),
            ("r1's current", amp_tols(rig.i(2), i1), rig.i(2), i1),
            ("r2's current", amp_tols(rig.i(4), i2), rig.i(4), i2),
        ];
        for (what, ratio, got, want) in checks {
            if ratio > worst.0 {
                worst = (ratio, format!("{what} at {t:.4e}: published {got:.9e}, ode {want:.9e}"));
            }
        }
    }
    let events = rig.hub().stats.events - events;
    eprintln!("two heavy RCs on a 1 uF CC rail: {frames} frames, rail events {events}, worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(frames >= 100, "{frames} frames in CC");
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}

/// A draining-cell drift board (the test below) at `[soc, vc]`: `(u, i_c,
/// i_cell)`, `u = E(soc) - R(soc) i_cell` solved by Newton, `i_cell` the node
/// shunt's, the 100 ohm load's and the partition's draw, `i_c` the current
/// into `c` before its node shunt. `gate`: 1y's drive and whether it is high,
/// else `r` from the rail into `c`.
fn cell_board(m: &crate::devices::battery::BatteryModel, x: [f64; 2], gate: Option<(f64, bool)>, r: f64) -> (f64, f64, f64) {
    let gs = static_part::NODE_RSHUNT_G;
    let (e, rb, vc) = (m.emf(x[0]), m.resistance(x[0]), x[1]);
    let draw = |u: f64| -> (f64, f64) {
        match gate {
            None => ((gs + 1.0 / 100.0) * u + (u - vc) / r, (u - vc) / r),
            Some((io, high)) => {
                let (v2, i_rail) = cc_gate_node2(u, vc, io, r, high);
                let i_c = if r > 0.0 { (v2 - vc) / r } else { gate_g(u, io) * (if high { u } else { 0.0 } - vc) };
                ((gs + 1.0 / 100.0) * u + i_rail, i_c)
            }
        }
    };
    let f = |u: f64| u - e + rb * draw(u).0;
    // Linear in `u` with no gate: one step from anywhere.
    let mut u = if gate.is_none() { (e + rb * vc / r) / (1.0 + rb * (gs + 1.0 / 100.0 + 1.0 / r)) } else { e };
    for _ in 0..60 {
        let du = f(u) / ((f(u + 1e-7) - f(u - 1e-7)) / 2e-7);
        u -= du;
        if du.abs() < 1e-15 {
            break;
        }
    }
    let (i, i_c) = draw(u);
    (u, i_c, i)
}

/// [`cell_board`]'s ODE, `soc' = -i_cell / Q` and `c vc' = i_c - gs vc`, by
/// RK4 from `t0` to `t1` in steps of at most `h`.
fn cell_board_rk4(m: &crate::devices::battery::BatteryModel, x: [f64; 2], t0: f64, t1: f64, h: f64, gate: Option<(f64, bool)>, r: f64, c: f64) -> [f64; 2] {
    let rhs = |x: [f64; 2]| {
        let (_, i_c, i) = cell_board(m, x, gate, r);
        [-i / m.capacity_coulombs(), (i_c - static_part::NODE_RSHUNT_G * x[1]) / c]
    };
    let n = ((t1 - t0) / h).ceil().max(1.0) as usize;
    let dt = (t1 - t0) / n as f64;
    let add = |a: [f64; 2], b: [f64; 2], s: f64| [a[0] + s * b[0], a[1] + s * b[1]];
    let mut x = x;
    for _ in 0..n {
        let k1 = rhs(x);
        let k2 = rhs(add(x, k1, 0.5 * dt));
        let k3 = rhs(add(x, k2, 0.5 * dt));
        let k4 = rhs(add(x, k3, dt));
        x = [x[0] + dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0, x[1] + dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0];
    }
    x
}

#[test]
fn a_draining_cells_drift_leaves_its_closed_form_hub_partitions_published_quantities_within_w0_on_a_coarse_host() {
    // R13 (a), the battery rail (F1r13's 672 boards), held since round 14 by
    // ruling R15 (a). A 5 V generic cell (0.5 ohm) drains through 100 ohm;
    // `r` from the rail into `c` (linear), or a gate on the rail driving 1y
    // high into `c` through `r` (curved), is a closed-form hub partition.
    // When it was held at the rail of its aim between solves while SoC and
    // its own draw moved the cell's rail, which its drift watch let go 10 uV,
    // `vc` kept up to that times `r`'s pull over `c`'s, and a settled current
    // through 50 ohm erred by about 200 nA against W0's 1 nA (90x). Since
    // round 15 (ruling R16 (B), `rails::cell_modes`) the rail and the
    // partition advance together between solves on the cell's coupled path,
    // the partition's own pull on the cell and the drain's line inside it
    // (a curved segment adding its transient's share), and the rail solves
    // again once the path has moved RAIL_DRIFT_V; every board here runs on
    // that path at every frame (worst 0.09 of W0). Round 14's drive (its own
    // pull in its rate, the drain's line as a ramp) is now only the
    // fallback for couplings the path declines. These are the probe's
    // worst boards, a fast and a slow drain, linear and curved. At each frame
    // the rail, `vc`, the load's, `r`'s and `c`'s currents (and node 2) are
    // within W0's tolerance of the board's own ODE (RK4 at a fortieth of the
    // time constant) from the published state.
    let gs = static_part::NODE_RSHUNT_G;
    let mut worst = (0.0f64, String::new());
    for (cap, gate, r, c) in [
        (1e-5, None, 50.0, 1e-9),
        (1e-5, None, 50.0, 1e-8),
        (1e-5, None, 150.0, 1e-8),
        (1e-7, None, 50.0, 1e-8),
        (1e-5, Some(0.024), 0.0, 1e-9),
        (1e-5, Some(0.024), 150.0, 1e-9),
        (1e-5, Some(0.008), 0.0, 1e-9),
    ] {
        let name = format!("cap {cap:e} {} r {r} c {c:e}", gate.map_or("linear".to_string(), |io| format!("gate io {io}")));
        let cell = generic_battery(0, 1, 0, 5.0, 0.5, cap);
        let m = crate::devices::battery::BatteryModel::from_record(&cell.params, &cell.blob);
        let mut devs = vec![cell, resistor(1, 1, 0, 100.0)];
        let (cn, cd, rd) = match gate {
            None => {
                devs.extend([resistor(2, 1, 2, r), capacitor(3, 2, 0, c, 0.0)]);
                (2, 3, Some(2))
            }
            Some(io) => {
                devs.extend([gate_io(2, 1, 0, [4, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 3.0, 6.0), source(3, 5, 0, 5.0), switch(4, 5, 4, true), resistor(5, 4, 0, 1000.0)]);
                let at = if r > 0.0 {
                    devs.extend([resistor(6, 2, 3, r), capacitor(7, 3, 0, c, 0.0)]);
                    (3, 7, Some(6))
                } else {
                    devs.push(capacitor(7, 2, 0, c, 0.0));
                    (2, 6, None)
                };
                // A second gate reads 1y.
                let mut reader = gate_io(8, 1, 0, [2, 2, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, 3.0, 6.0);
                reader.blob[0] = 4.0;
                devs.push(reader);
                at
            }
        };
        let mut p = no_cache(program(if gate.is_some() { 7 } else { 3 }, devs));
        p.digital_slot_count = p.digital_slot_count.max(16);
        let mut rig = Rig::load(&p).unwrap();
        let part = rig.s().local_index[cn].unwrap().0;
        assert_eq!(rig.s().partition_kinds[part], PartitionKind::ClosedForm, "{name}: a closed-form hub partition");
        assert_eq!(rig.s().closed_form.hub_band(part).is_some(), gate.is_some(), "{name}: curved with the gate");
        let g = gate.map(|io| (io, true));
        let tau = gate.map_or(r, |io| r + 1.0 / gate_g(4.5, io)) * c;
        let mut t = 0.0;
        if gate.is_some() {
            // The switch opens the gate's inputs; 1y rises one propagation
            // delay (10 ns) later.
            t = 20.0 * tau;
            rig.advance(t).unwrap();
            toggle(&mut rig, 4, false).unwrap();
            t += 2e-8;
            rig.advance(t).unwrap();
        }
        let mut x = [rig.state(0)[0], rig.v(cn)];
        let (mut frames, events) = (0, rig.hub().stats.events);
        while frames < 300 {
            let t1 = t + 1e-6;
            x = cell_board_rk4(&m, x, t, t1, tau / 40.0, g, r, c);
            rig.advance(t1).unwrap();
            t = t1;
            frames += 1;
            let (u, i_c, _) = cell_board(&m, x, g, r);
            let mut checks = vec![("rail", volt_tols(rig.u(), u), rig.u(), u), ("vc", volt_tols(rig.v(cn), x[1]), rig.v(cn), x[1]), ("load", amp_tols(rig.i(1), u / 100.0), rig.i(1), u / 100.0), ("c's current", amp_tols(rig.i(cd), i_c - gs * x[1]), rig.i(cd), i_c - gs * x[1])];
            if let Some(rd) = rd {
                checks.push(("r's current", amp_tols(rig.i(rd), i_c), rig.i(rd), i_c));
            }
            if let Some(io) = gate {
                let v2 = cc_gate_node2(u, x[1], io, r, true).0;
                checks.push(("node 2", volt_tols(rig.v(2), v2), rig.v(2), v2));
            }
            for (what, ratio, got, want) in checks {
                if ratio > worst.0 {
                    worst = (ratio, format!("{name}: {what} at {t:.4e} u {:.6}: published {got:.6e}, ode {want:.6e}", rig.u()));
                }
            }
        }
        eprintln!("{name}: {frames} frames, rail events {}", rig.hub().stats.events - events);
    }
    eprintln!("draining cell drift: worst {:.4} of W0 at {}", worst.0, worst.1);
    assert!(worst.0 <= 1.0, "{} of W0's tolerance at {}", worst.0, worst.1);
}



// ---- FXE (rulings R23 (i), (ii) and the keep-best stop rule) ----------------

/// A double-double number, `hi + lo` with `|lo| <= ulp(hi) / 2`: about 32
/// significant digits, enough to stand as the exact solution of a hub
/// partition whose `f64` conditioning leaves its solve 1e-6 V off (a
/// GMIN-held cluster) or W0 off (1e-3 and 1e10 ohm in one partition).
#[derive(Clone, Copy, Debug)]
struct Dd(f64, f64);

fn dd_two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let bb = s - a;
    (s, (a - (s - bb)) + (b - bb))
}

fn dd_quick(a: f64, b: f64) -> Dd {
    let s = a + b;
    Dd(s, b - (s - a))
}

impl Dd {
    fn of(x: f64) -> Dd {
        Dd(x, 0.0)
    }

    fn f(self) -> f64 {
        self.0 + self.1
    }

    fn scale(self, k: f64) -> Dd {
        // Exact for a power of two.
        Dd(self.0 * k, self.1 * k)
    }

    fn exp(self) -> Dd {
        if self.0 < -700.0 {
            return Dd(0.0, 0.0);
        }
        const LN2: Dd = Dd(6.931471805599452862e-1, 2.319046813846299558e-17);
        let k = (self.0 / LN2.0).round();
        let r = (self - LN2 * Dd::of(k)).scale(1.0 / 1024.0);
        let (mut term, mut sum) = (Dd::of(1.0), Dd::of(1.0));
        for i in 1..=14 {
            term = term * r / Dd::of(i as f64);
            sum = sum + term;
        }
        for _ in 0..10 {
            sum = sum * sum;
        }
        sum.scale(2f64.powi(k as i32))
    }

    fn min(self, o: Dd) -> Dd {
        if (self - o).f() <= 0.0 { self } else { o }
    }
}

impl std::ops::Add for Dd {
    type Output = Dd;
    fn add(self, o: Dd) -> Dd {
        let (s, e) = dd_two_sum(self.0, o.0);
        let (t, f) = dd_two_sum(self.1, o.1);
        let r = dd_quick(s, e + t);
        dd_quick(r.0, r.1 + f)
    }
}

impl std::ops::Neg for Dd {
    type Output = Dd;
    fn neg(self) -> Dd {
        Dd(-self.0, -self.1)
    }
}

impl std::ops::Sub for Dd {
    type Output = Dd;
    fn sub(self, o: Dd) -> Dd {
        self + (-o)
    }
}

impl std::ops::Mul for Dd {
    type Output = Dd;
    fn mul(self, o: Dd) -> Dd {
        let p = self.0 * o.0;
        let e = self.0.mul_add(o.0, -p);
        dd_quick(p, e + (self.0 * o.1 + self.1 * o.0))
    }
}

impl std::ops::Div for Dd {
    type Output = Dd;
    fn div(self, o: Dd) -> Dd {
        let q1 = self.0 / o.0;
        let r = self - o * Dd::of(q1);
        let q2 = r.0 / o.0;
        let r = r - o * Dd::of(q2);
        let q3 = r.0 / o.0;
        dd_quick(q1, q2) + Dd::of(q3)
    }
}

/// The junction law of `led(..)` in double-double: `I(v)`, GMIN excluded,
/// and its slope.
fn dd_led(v: Dd) -> (Dd, f64) {
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let (is, vtn, vsat) = (Dd::of(law.is), Dd::of(law.vtn), Dd::of(law.vsat));
    let c = v.min(vsat);
    let e = (c / vtn).exp();
    let past = if (v - vsat).f() > 0.0 { v - vsat } else { Dd::of(0.0) };
    let i = is * (e - Dd::of(1.0)) + (is * e / vtn) * past;
    (i, (is * e / vtn).f())
}

/// One element of an FXE board: a resistor, an LED (anode, cathode), the
/// output (high) of a gate whose supply is the rail and whose drive is
/// `io`, or an open switch (which stamps nothing).
#[derive(Clone, Copy, Debug)]
enum FxEl {
    R(usize, usize, f64),
    Led(usize, usize),
    Gate(usize, f64),
    Sw(usize, usize),
}

/// A hub board on the rail (node 1) of V6's supplies: its elements, its
/// free nodes, the window of its gates' supply.
struct FxBoard {
    name: String,
    p: Program,
    els: Vec<FxEl>,
    nodes: Vec<usize>,
    win: (f64, f64),
    ramp: bool,
}

fn fx_board(name: &str, ramp: bool, els: Vec<FxEl>, win: (f64, f64)) -> FxBoard {
    let mut devs = d7_supply(ramp);
    let mut top = 1;
    for (k, e) in els.iter().enumerate() {
        let ci = 2 + k;
        devs.push(match *e {
            FxEl::R(a, b, r) => resistor(ci, a, b, r),
            FxEl::Led(a, c) => led(ci, a, c),
            FxEl::Gate(o, io) => gate_io(ci, 1, 0, [0, 0, o, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, win.0, win.1),
            FxEl::Sw(a, b) => switch(ci, a, b, false),
        });
        top = top.max(match *e {
            FxEl::R(a, b, _) | FxEl::Led(a, b) | FxEl::Sw(a, b) => a.max(b),
            FxEl::Gate(o, _) => o,
        });
    }
    let mut p = no_cache(program(top + 1, devs));
    p.digital_slot_count = p.digital_slot_count.max(8);
    FxBoard { name: format!("{name}, {}", if ramp { "CC ramp" } else { "battery" }), p, els, nodes: (2..=top).collect(), win, ramp }
}

/// An FXE board's exact solution at rail `u`, rounded once from
/// double-double: its node voltages (in `nodes` order) and each element's
/// current (a resistor's and an LED's, GMIN excluded, `a` to `b`; a gate's
/// out of the rail).
struct FxExact {
    x: Vec<f64>,
    cur: Vec<f64>,
    /// Each element's whole current (an LED's GMIN included), unrounded.
    full: Vec<Dd>,
}

/// The exact solution of `b` at rail `u`, its gate unpowered when `held`
/// (W1c point 1a) or outside its window.
fn fx_exact(b: &FxBoard, u: f64, held: bool) -> FxExact {
    let n = b.nodes.len();
    let at = |node: usize| b.nodes.iter().position(|&m| m == node);
    let powered = !held && u >= b.win.0 * 0.9 && u <= b.win.1 * 1.1;
    let gate_g = |io: f64| Dd::of(1.0) / (Dd::of(u) / Dd::of(4.0 * io)).min(Dd::of(1000.0)).max_dd(Dd::of(50.0));
    let volt = |x: &[Dd], node: usize| if node == 0 { Dd::of(0.0) } else if node == 1 { Dd::of(u) } else { x[at(node).unwrap()] };
    // Each element's current a -> b (into the node for a gate) and its slope.
    let current = |x: &[Dd], e: &FxEl| -> (Dd, f64) {
        match *e {
            FxEl::R(a, c, r) => ((volt(x, a) - volt(x, c)) / Dd::of(r), 1.0 / r),
            FxEl::Led(a, c) => {
                let v = volt(x, a) - volt(x, c);
                let (i, g) = dd_led(v);
                (i + Dd::of(led::GMIN_D) * v, g + led::GMIN_D)
            }
            FxEl::Gate(o, io) if powered => {
                let g = gate_g(io);
                (g * (Dd::of(u) - volt(x, o)), g.f())
            }
            _ => (Dd::of(0.0), 0.0),
        }
    };
    let ends = |e: &FxEl| -> (usize, usize) {
        match *e {
            FxEl::R(a, c, _) | FxEl::Led(a, c) => (a, c),
            FxEl::Gate(o, _) => (1, o),
            FxEl::Sw(a, c) => (a, c),
        }
    };
    let mut x: Vec<Dd> = vec![Dd::of(0.5 * u); n];
    let mut small = 0;
    for _ in 0..400 {
        let mut f = vec![Dd::of(0.0); n];
        let mut j = vec![0.0; n * n];
        for k in 0..n {
            f[k] = Dd::of(static_part::NODE_RSHUNT_G) * x[k];
            j[k * n + k] += static_part::NODE_RSHUNT_G;
        }
        for e in &b.els {
            let (i, g) = current(&x, e);
            let (a, c) = ends(e);
            let (la, lc) = (at(a), at(c));
            if let Some(la) = la {
                f[la] = f[la] + i;
            }
            if let Some(lc) = lc {
                f[lc] = f[lc] - i;
            }
            for (p, sp) in [(la, 1.0), (lc, -1.0)] {
                for (q, sq) in [(la, 1.0), (lc, -1.0)] {
                    if let (Some(p), Some(q)) = (p, q) {
                        j[p * n + q] += sp * sq * g;
                    }
                }
            }
        }
        let mut dx: Vec<f64> = f.iter().map(|v| -v.f()).collect();
        crate::lu::lu_factor(j, n).unwrap().solve(&mut dx).unwrap();
        let m = dx.iter().fold(0.0f64, |a, v| a.max(v.abs()));
        let damp = if m > 0.2 { 0.2 / m } else { 1.0 };
        for k in 0..n {
            x[k] = x[k] + Dd::of(damp * dx[k]);
        }
        // Converged to double-double, give or take the conditioning: a
        // few steps more once the step is far below any floor.
        if m < 1e-27 * (1.0 + u) {
            small += 1;
            if small >= 3 {
                break;
            }
        }
    }
    let full: Vec<Dd> = b.els.iter().map(|e| current(&x, e).0).collect();
    let cur = b
        .els
        .iter()
        .zip(&full)
        .map(|(e, i)| {
            match *e {
                // The LED's published current excludes GMIN.
                FxEl::Led(a, c) => dd_led(volt(&x, a) - volt(&x, c)).0.f(),
                _ => i.f(),
            }
        })
        .collect();
    FxExact { x: x.iter().map(|v| v.f()).collect(), cur, full }
}

impl Dd {
    fn max_dd(self, o: Dd) -> Dd {
        if (self - o).f() >= 0.0 { self } else { o }
    }
}

/// Over every published instant of `b` (the CC ramp between events, the
/// battery's `steps` even steps over 40 ms, with `early` first), the worst
/// ratio of a published node voltage's, resistor or LED current's or
/// draw's distance from the exact solution's to the bound its band was
/// sized from, with no allowance beyond the exact value's own rounding;
/// and the worst distance
/// of a hub solve itself (its node voltages and the currents they give,
/// formed exactly) from the exact solution at its rail, in W0. Every hub
/// partition of the board is read. Instants whose rail and solve sit on two
/// sides of a gate's power edge are left out. `(bound ratio, solve W0,
/// samples, where, solve where)`.
fn fx_published_within_bound(b: &FxBoard, early: &[f64], steps: usize) -> (f64, f64, usize, String, String) {
    let mut rig = Rig::load(&b.p).unwrap();
    // Each hub partition: its free nodes (as indices into `b.nodes`) and
    // local indices, and its elements (by index into `b.els`).
    let mut parts: Vec<(usize, Vec<usize>, Vec<usize>, Vec<usize>)> = Vec::new();
    for (k, &n) in b.nodes.iter().enumerate() {
        let Some((p, l)) = rig.s().local_index[n] else { continue };
        match parts.iter_mut().find(|e| e.0 == p) {
            Some(e) => {
                e.1.push(k);
                e.2.push(l);
            }
            None => parts.push((p, vec![k], vec![l], Vec::new())),
        }
    }
    for (k, el) in b.els.iter().enumerate() {
        let ends = match *el {
            FxEl::R(a, c, _) | FxEl::Led(a, c) | FxEl::Sw(a, c) => [a, c],
            FxEl::Gate(o, _) => [o, o],
        };
        if let Some(e) = parts.iter_mut().find(|e| ends.iter().any(|&n| rig.s().local_index[n].is_some_and(|(p, _)| p == e.0))) {
            e.3.push(k);
        }
    }
    let (mut ratio, mut solve_w0, mut seen, mut at, mut solve_at) = (0.0f64, 0.0f64, 0usize, String::new(), String::new());
    let mut last_up = vec![f64::NAN; parts.len()];
    let powered = |u: f64| u >= b.win.0 * 0.9 && u <= b.win.1 * 1.1;
    let tol_v = |v: f64| 1e-4 + 1e-3 * v.abs();
    let tol_i = |i: f64| 1e-9 + 1e-3 * i.abs();
    let mut look = |rig: &Rig, t: f64| {
        let u = rig.v(1);
        // A gate held off (W1c point 1a: on a draining cell, at its floor,
        // its own draw would take the rail under it) is unpowered whatever
        // the rail reads, and so is its exact solution. Device `2 + k` is
        // element `k` (`fx_board`).
        let held = b.els.iter().enumerate().any(|(k, e)| matches!(e, FxEl::Gate(..)) && rig.s().power_hold(2 + k).kind == digital::power::HoldKind::Off);
        let powered = |u: f64| !held && powered(u);
        for (pi, (part, idx, local, els)) in parts.iter().enumerate() {
            let Some((x, _, i_p, g_p, u_p)) = rig.s().static_solver.hub_solve(*part) else { continue };
            if powered(u) != powered(u_p) {
                continue;
            }
            if u_p.to_bits() != last_up[pi].to_bits() {
                last_up[pi] = u_p;
                // The solve itself against the exact solution at its rail.
                let e = fx_exact(b, u_p, held);
                let v_at = |node: usize| -> Dd {
                    if node == 0 {
                        Dd::of(0.0)
                    } else if node == 1 {
                        Dd::of(u_p)
                    } else {
                        let k = b.nodes.iter().position(|&m| m == node).unwrap();
                        match idx.iter().position(|&q| q == k) {
                            Some(j) => Dd::of(x[local[j]]),
                            None => Dd::of(e.x[k]),
                        }
                    }
                };
                let mut w = |what: String, d: f64, tol: f64| {
                    if d / tol > solve_w0 {
                        solve_w0 = d / tol;
                        solve_at = format!("{what} u_p {u_p:.9}: {d:e} off");
                    }
                };
                for (j, &k) in idx.iter().enumerate() {
                    w(format!("node {}", b.nodes[k]), (x[local[j]] - e.x[k]).abs(), tol_v(e.x[k]));
                }
                for &k in els {
                    match b.els[k] {
                        FxEl::R(a, c, r) => w(format!("resistor {}", k + 2), (((v_at(a) - v_at(c)) / Dd::of(r)) - Dd::of(e.cur[k])).f().abs(), tol_i(e.cur[k])),
                        FxEl::Led(a, c) => w(format!("LED {}", k + 2), (dd_led(v_at(a) - v_at(c)).0 - Dd::of(e.cur[k])).f().abs(), tol_i(e.cur[k])),
                        _ => {}
                    }
                }
            }
            let Some(last) = curvature::hook::of(b.nodes[idx[0]]).filter(|l| l.u_p.to_bits() == u_p.to_bits() && l.bounded()) else { continue };
            let bw = (u - u_p).abs();
            let e = fx_exact(b, u, held);
            let mut note = |what: String, published: f64, want: f64, bound: f64| {
                let r = ((published - want).abs() - f64::EPSILON * want.abs()).max(0.0) / bound;
                if r > ratio {
                    ratio = r;
                    at = format!("{what} t {t:e} u {u:.9} u_p {u_p:.9}: published {published:e}, exact {want:e}, bound {bound:e}");
                }
            };
            for &k in idx {
                note(format!("node {}", b.nodes[k]), rig.v(b.nodes[k]), e.x[k], last.node_at(b.nodes[k], bw));
            }
            let mut j = 0;
            // The exact draw: every element current leaving the rail.
            let mut draw = Dd::of(0.0);
            for &k in els {
                let ends = match b.els[k] {
                    FxEl::R(a, c, _) | FxEl::Led(a, c) | FxEl::Sw(a, c) => (a, c),
                    FxEl::Gate(o, _) => (1, o),
                };
                if ends.0 == 1 {
                    draw = draw + e.full[k];
                }
                if ends.1 == 1 {
                    draw = draw - e.full[k];
                }
                match b.els[k] {
                    FxEl::R(..) => note(format!("resistor {}", k + 2), rig.i(k + 2), e.cur[k], last.branch(k + 2, bw).0),
                    FxEl::Led(..) => {
                        note(format!("LED {}", k + 2), rig.i(k + 2), e.cur[k], last.junction(j, bw));
                        j += 1;
                    }
                    _ => {}
                }
            }
            if last.draw_rule() != Some(curvature::DrawRule::Zero) {
                note("draw".to_string(), i_p + g_p * (u - u_p), draw.f(), last.draw(bw));
            }
            seen += 1;
        }
    };
    for &t in early {
        rig.advance(t).unwrap();
        look(&rig, t);
    }
    if b.ramp {
        sample_between_events(&mut rig, 10e-3, |rig, t| look(rig, t));
    } else {
        for k in 1..=steps {
            let t = 40e-3 * k as f64 / steps as f64;
            if rig.advance(t).is_err() {
                break;
            }
            look(&rig, t);
        }
    }
    (ratio, solve_w0, seen, at, solve_at)
}

/// FXDv's GMIN-held floating clusters (ruling R23 (i)): nodes 3 and 4
/// between an LED off node 2, an open switch, a reversed LED and an LED to
/// ground, joined by `r34`, beside a divider off the rail (`k5`); a rail
/// gate through `r` into an LED clamp to ground and one to the rail, which
/// floats once the gate is unpowered (`k1`).
fn fx_cluster(r12: f64, r20: f64, r34: f64, led24: bool, ramp: bool) -> FxBoard {
    let mut els = vec![FxEl::R(1, 2, r12), FxEl::R(2, 0, r20), FxEl::Led(3, 2), FxEl::Sw(3, 0), FxEl::Led(0, 3), FxEl::R(3, 4, r34), FxEl::Led(4, 0)];
    if led24 {
        els.push(FxEl::Led(2, 4));
    }
    fx_board(&format!("a floating cluster: {r12} and {r20} off the rail, {r34} between two LEDs{}", if led24 { ", an LED from node 2" } else { "" }), ramp, els, (0.0, 0.0))
}

fn fx_clamped_gate(io: f64, r: f64, win: (f64, f64), ramp: bool) -> FxBoard {
    fx_board(&format!("a rail gate io {io} through {r} into two LED clamps"), ramp, vec![FxEl::Gate(2, io), FxEl::R(2, 3, r), FxEl::Led(3, 0), FxEl::Led(3, 1)], win)
}

/// `b` (a gate on a fed supply) loaded on a PSU of `v` volts: the rail
/// static, the hub solved at load.
fn fx_on_psu(b: &mut D7Board, v: f64) -> Result<Rig, BoError> {
    b.p.devices[0] = psu(0, 1, 0, v, 20.0);
    b.p.devices[1] = resistor(1, 1, 0, 1e9);
    Rig::load(&b.p)
}

/// The power-edge hairline of a gate on a fed supply (its supply node 2
/// through `rs` off the rail, the rest of `b` behind its output): the PSU
/// volts `V*` at which its exact powered solution's supply sits on the
/// power floor, `0.9 lo`, and every load at `V* + d` over a ladder of `d`
/// from 1e-15 to 1e-4 V either way. Returns `(loads, failed, worst solve
/// offset in volts, where)` over the loads that published, each against
/// the exact solution of the state that holds there: powered from `V*` up,
/// unpowered (the supply at the rail less the shunt's share, the rest at
/// 0 V) below, each where its supply is inside (outside) the window `[0.9
/// lo, 1.1 hi]`, `hi` 6 V, to within 1e-6 V. Between the floor and `V*`
/// neither holds, and a published solve is wrong whichever it is held to;
/// `-4` is the answer there (FXDv: 49-66 % of CC-ramp fed-supply boards,
/// ruling R23 (iii)).
fn fx_power_hairline(b: &mut D7Board, rs: f64, lo: f64) -> (usize, usize, f64, String) {
    let floor = 0.9 * lo;
    let (mut a, mut z) = (floor, 20.0);
    assert!((b.exact)(z).0[0] >= floor, "{}: no power edge below 20 V", b.name);
    for _ in 0..200 {
        let mid = 0.5 * (a + z);
        if (b.exact)(mid).0[0] < floor {
            a = mid;
        } else {
            z = mid;
        }
    }
    let v_star = z;
    let mut deltas = vec![0.0];
    for j in 0..45 {
        let d = 1e-15 * 10f64.powf(j as f64 * 0.25);
        deltas.extend([d, -d]);
    }
    let (mut loads, mut failed, mut worst, mut at) = (0usize, 0usize, 0.0f64, String::new());
    for d in deltas {
        loads += 1;
        match fx_on_psu(b, v_star + d) {
            Err(e) => {
                assert_eq!(e.code, crate::error::ERR_SOLVE_FAILED, "{}", e.message);
                failed += 1;
            }
            Ok(rig) => {
                let part = rig.s().local_index[b.nodes[0]].unwrap().0;
                let (x, _, _, _, u_p) = rig.s().static_solver.hub_solve(part).unwrap();
                // The states that hold here, to rounding of the floor.
                let (powered, _) = (b.exact)(u_p);
                let off: Vec<f64> = b.nodes.iter().map(|&n| if n == 2 { u_p / (1.0 + rs * static_part::NODE_RSHUNT_G) } else { 0.0 }).collect();
                let mut states = Vec::new();
                let top = 1.1 * 6.0;
                if powered[0] >= floor - 1e-6 && powered[0] <= top + 1e-6 {
                    states.push(powered);
                }
                // A gate held off (W1c point 1a: between the floor and V*,
                // powered it starves its own supply) holds the unpowered state
                // wherever its supply sits.
                let gate = b.p.devices.iter().position(|d| d.kind == kinds::IC_74LS00 && d.pins[0].node == b.nodes[0]).expect("the fed gate");
                let held = rig.s().power_hold(gate).kind == digital::power::HoldKind::Off;
                if held || off[0] <= floor + 1e-6 || off[0] >= top - 1e-6 {
                    states.push(off);
                }
                let published: Vec<f64> = b.nodes.iter().map(|&n| x[rig.s().local_index[n].unwrap().1]).collect();
                let dist = states.iter().map(|xs| published.iter().zip(xs).fold(0.0f64, |a, (p, q)| a.max((p - q).abs()))).fold(f64::INFINITY, f64::min);
                if dist > worst {
                    worst = dist;
                    at = format!("V* {d:+e}: {dist:e} V from the solution of every state that holds there ({})", states.len());
                }
            }
        }
    }
    (loads, failed, worst, at)
}

/// A gate whose supply (node 2) is fed from the rail through `rs`, its 1y
/// (node 3) high through `r` into an LED (node 4) to ground: FXDv's
/// power-edge hairline board.
fn fx_fed_gate_into_an_led(rs: f64, io: f64, r: f64, lo: f64) -> D7Board {
    let gs = static_part::NODE_RSHUNT_G;
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let jf = move |v: f64| law.current(v) + led::GMIN_D * v;
    let mut devs = d7_supply(false);
    devs.extend([resistor(2, 1, 2, rs), gate_io(3, 2, 0, [0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, lo, 6.0), resistor(4, 3, 4, r), led(5, 4, 0)]);
    let exact = move |u: f64| {
        let x = newton_fd(vec![u - 0.1, 0.5 * u, 0.3 * u], &|x: &[f64]| {
            let (v, o, a) = (x[0], x[1], x[2]);
            let g = gate_g(v, io);
            vec![(v - u) / rs + g * (v - o) + gs * v, g * (o - v) + (o - a) / r + gs * o, (a - o) / r + jf(a) + gs * a]
        });
        let d = (u - x[0]) / rs;
        (x, d)
    };
    let mut p = no_cache(program(5, devs));
    p.digital_slot_count = p.digital_slot_count.max(8);
    D7Board {
        name: format!("a gate fed through {rs} (io {io}, lo {lo}) into {r} and an LED"),
        p,
        nodes: vec![2, 3, 4],
        res: vec![(2, 1, 2, rs), (4, 3, 4, r)],
        leds: vec![(5, 4, 0)],
        exact: Box::new(exact),
        supply: (2, 0.9 * lo, 6.6),
        carry: Vec::new(),
    }
}

/// A gate whose supply (node 2) is fed from the rail through `rs`, its 1y
/// (node 3) high into `r` to ground and into two LEDs in series to ground
/// (node 4 between them), which stay off below 4 V: node 4 is held by
/// their GMIN alone, so the partition's rounding floor is large.
fn fx_fed_gate_into_r_and_an_off_chain(rs: f64, io: f64, r: f64, lo: f64) -> D7Board {
    let gs = static_part::NODE_RSHUNT_G;
    let law = led::JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0);
    let jf = move |v: f64| law.current(v) + led::GMIN_D * v;
    let mut devs = d7_supply(false);
    devs.extend([resistor(2, 1, 2, rs), gate_io(3, 2, 0, [0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0], io, lo, 6.0), resistor(4, 3, 0, r), led(5, 3, 4), led(6, 4, 0)]);
    let exact = move |u: f64| {
        let x = newton_fd(vec![u - 0.1, 0.5 * u, 0.25 * u], &|x: &[f64]| {
            let (v, o, a) = (x[0], x[1], x[2]);
            let g = gate_g(v, io);
            vec![(v - u) / rs + g * (v - o) + gs * v, g * (o - v) + o / r + jf(o - a) + gs * o, -jf(o - a) + jf(a) + gs * a]
        });
        let d = (u - x[0]) / rs;
        (x, d)
    };
    let mut p = no_cache(program(5, devs));
    p.digital_slot_count = p.digital_slot_count.max(8);
    D7Board {
        name: format!("a gate fed through {rs} (io {io}, lo {lo}) into {r} and two LEDs off"),
        p,
        nodes: vec![2, 3, 4],
        res: vec![(2, 1, 2, rs), (4, 3, 0, r)],
        leds: vec![(5, 3, 4), (6, 4, 0)],
        exact: Box::new(exact),
        supply: (2, 0.9 * lo, 6.6),
        carry: Vec::new(),
    }
}

/// FXDv's extreme-ratio boards (ruling R23 (ii)): resistors from 1e-3 to
/// 1e10 ohm in one partition, with LEDs and open switches (`k8 s1017`, its
/// polished solve 70 W0 from the exact one at bda1839, 18 W0 before the
/// polish), and the 0.05..1e8 and 1e-3..1e9 families' worst (`k4 s1002`,
/// 1.3 W0; `k9 s1011`, a ladder, 0.04 W0).
fn fx_extreme(which: u32, ramp: bool) -> FxBoard {
    use FxEl::{Led, Sw, R};
    let (name, els) = match which {
        0 => (
            "k8 s1017: 1e-3 to 3e9 ohm, five LEDs, two open switches",
            vec![
                R(1, 2, 1.22514421094097123e1),
                R(2, 8, 2.78324059714798633e0),
                R(2, 4, 2.83431818593903732e9),
                Sw(3, 8),
                R(3, 4, 7.99657143686693071e-3),
                R(4, 2, 6.18989618499603510e8),
                R(4, 9, 4.52717979286214802e6),
                Led(6, 4),
                Led(6, 5),
                R(6, 7, 1.03325941760135077e-3),
                Led(0, 6),
                R(7, 6, 6.14217350471706712e5),
                Sw(7, 6),
                Led(5, 7),
                R(8, 3, 3.87057194601742958e4),
                R(8, 3, 2.83677629330803569e1),
                Led(8, 0),
                R(9, 1, 4.24281797683692873e7),
            ],
        ),
        1 => (
            "k4 s1002: 0.06 to 1.4e7 ohm, six LEDs",
            vec![
                R(1, 2, 3.68785200933061788e4),
                Led(2, 3),
                R(3, 4, 1.40215703507334981e7),
                R(3, 4, 6.10248508535342973e-2),
                R(3, 4, 1.22722025606106827e6),
                Led(0, 4),
                Led(3, 4),
                Led(4, 5),
                Led(5, 2),
                Led(5, 0),
            ],
        ),
        _ => (
            "k9 s1011: a ladder of 3e-3 to 4e8 ohm, three LEDs",
            vec![
                R(1, 2, 4.20141960479364038e8),
                R(2, 3, 1.92077431507161127e4),
                R(3, 4, 3.79383561545533444e-3),
                R(4, 5, 2.88564524287502093e-3),
                R(5, 0, 3.19467604428716600e8),
                Led(0, 3),
                Led(4, 0),
                Led(5, 0),
            ],
        ),
    };
    fx_board(name, ramp, els, (0.0, 0.0))
}

#[test]
fn a_stalled_hub_polish_keeps_the_iterate_with_the_smaller_residual() {
    // Ruling R23 (keep-best). Where its steps stop contracting a polish's
    // steps are rounding, and bda1839 applied the last one regardless: on
    // GMIN-held clusters and extreme-ratio partitions 5-20 % of such exits
    // ended more than 2x further from the solution than the iterate that
    // entered (FXDv). Now every stalled polish publishes a residual (each
    // node's KCL imbalance over its own conductance, volts) no larger than
    // the entering iterate's. (A polish that arrives by a step within 16
    // ulps of its largest voltage takes it: nothing smaller moves a value.)
    let mut boards = Vec::new();
    for ramp in [false, true] {
        boards.push(fx_cluster(2.29888839893743125e2, 6.07826902542198582e1, 1.07865330249350109e1, false, ramp));
        boards.push(fx_cluster(4.86101407852637749e3, 8.55780208410779778e3, 6.63255153855991164e0, true, ramp));
        boards.push(fx_extreme(1, ramp));
        boards.push(fx_extreme(2, ramp));
    }
    let (mut stalls, mut worst) = (0usize, (0.0f64, String::new()));
    for b in &boards {
        let _norms = newton::hooks::record_polish_norms();
        let mut rig = Rig::load(&b.p).unwrap();
        if b.ramp {
            sample_between_events(&mut rig, 10e-3, |_, _| {});
        } else {
            for k in 1..=2000 {
                if rig.advance(40e-3 * k as f64 / 2000.0).is_err() {
                    break;
                }
            }
        }
        for (m_in, m_out, _) in newton::hooks::polish_norms() {
            stalls += 1;
            let over = m_out / m_in;
            if over > worst.0 {
                worst = (over, format!("{}: entering {m_in:e} V, published {m_out:e} V", b.name));
            }
        }
    }
    eprintln!("{stalls} stalled polishes; the published residual at worst {:.4} of the entering one's at {}", worst.0, worst.1);
    assert!(stalls > 500, "{stalls} stalled polishes");
    assert!(worst.0 <= 1.0, "a polish published a residual {} of the entering one's at {}", worst.0, worst.1);
}

#[test]
fn a_published_solve_at_a_power_edge_is_a_solution_of_the_state_it_publishes() {
    // Ruling R23 (keep-best), FXDv's seed: loosening the polish's rounding
    // floor (`step_rounding`) a millionfold passed every test. At the power
    // edge of a gate on a fed supply (its supply on 0.9 lo), between the
    // floor and the rail `V*` where the powered solution's supply sits on
    // it, no state holds: unpowered the supply would sit at the rail, so
    // the polish ping-pongs across the edge and stalls, which only the
    // floor turns into `-4` where no iterate it evaluated is a solution.
    // Every load from V* - 1e-4 V to V* + 1e-4 V either fails `-4` (the
    // power edge's own failures, ruling R23 (iii), F5's) or publishes a
    // solution of a state that holds there: within 1e-12 V on a gate into a
    // load and two LEDs held off (a node held by GMIN alone, so the
    // rounding floor is large: loosened, it publishes stalled iterates up to
    // 1.6e-9 V off), and within 1e-6 V (0.01 W0) where the gate drives an
    // LED alone (the iterate a stall keeps sits up to 7e-9 V off, inside the
    // polish's rounding floor of 2.2e-7 V there).
    let mut boards = vec![
        (fx_fed_gate_into_r_and_an_off_chain(150.0, 0.004, 1000.0, 1.0), 150.0, 1.0, 1e-12),
        (fx_fed_gate_into_r_and_an_off_chain(60.0, 0.004, 200.0, 3.0), 60.0, 3.0, 1e-12),
        (fx_fed_gate_into_r_and_an_off_chain(20.0, 0.004, 200.0, 2.0), 20.0, 2.0, 1e-12),
        (fx_fed_gate_into_an_led(40.94005929443432, 0.016, 72.39521148039215, 1.0), 40.94005929443432, 1.0, 1e-6),
        (fx_fed_gate_into_an_led(260.10858729957744, 0.004, 56.385405604028385, 1.0), 260.10858729957744, 1.0, 1e-6),
        (one_gate_on_a_fed_supply(5.632455568787591, 60.754768860798805, 0.004, (2.0, 6.0), false, 0.0), 5.632455568787591, 2.0, 1e-12),
    ];
    for (b, rs, lo, tol) in boards.iter_mut() {
        let (loads, failed, worst, at) = fx_power_hairline(b, *rs, *lo);
        eprintln!("{}: {failed} of {loads} loads failed -4; the published worst {worst:e} V from a solution ({at})", b.name);
        assert!(loads - failed >= 30, "{}: {failed} of {loads} failed", b.name);
        assert!(worst <= *tol, "{}: {at}", b.name);
    }
}

#[test]
fn a_floating_cluster_publishes_within_its_bound_its_rounding_floor_included() {
    // Ruling R23 (i). A GMIN-held cluster (nodes between off LEDs and an
    // open switch; a gate's output once it is unpowered) is held by 1e-12
    // S, so the residual's rounding moves its solve by up to 1e-6 V (7e-3
    // W0), which no band counted: published 1.8e-6 V from the solution
    // against a bound of 1e-14 V and less (FXDv; bda1839: published values
    // off a bound of exactly 0). Every published node voltage, resistor and
    // LED current and draw is now within its bound against the exact
    // (double-double) solution, with no allowance beyond the exact value's
    // own rounding, the rounding floor added (`curvature::Floors`).
    let early = [1e-11, 1e-10, 1e-9, 1e-8, 1e-7];
    let mut worst = (0.0f64, String::new());
    for ramp in [false, true] {
        for b in [
            fx_cluster(2.29888839893743125e2, 6.07826902542198582e1, 1.07865330249350109e1, false, ramp),
            fx_cluster(4.86101407852637749e3, 8.55780208410779778e3, 6.63255153855991164e0, true, ramp),
            fx_clamped_gate(8e-3, 2.19160003795985254e1, (4.0, 6.0), ramp),
        ] {
            let (ratio, w0, seen, at, sat) = fx_published_within_bound(&b, &early, 2000);
            eprintln!("{}: published {ratio:.4} of its bound over {seen} samples (at {at}); solve {w0:.2e} of W0 ({sat})", b.name);
            assert!(seen > 50, "{}: {seen} samples", b.name);
            if ratio > worst.0 {
                worst = (ratio, format!("{}: {at}", b.name));
            }
        }
    }
    assert!(worst.0 <= 1.0, "published {} of its bound at {}", worst.0, worst.1);
}

#[test]
fn extreme_resistance_ratios_are_refined_to_a_hundredth_of_w0() {
    // Ruling R23 (ii). With 1e-3 and 1e10 ohm in one partition the stamp's
    // residual rounds by ulps of 1e3 A terms, which a GMIN-held cluster's
    // 1e-12 S multiply into volts: k8 s1017's solve sat 70 W0 from the
    // exact solution at bda1839. Where the rounding floor passes 0.01 of
    // W0 the solve is refined with the residual summed element by element:
    // every hub solve of these boards is within 0.01 W0 of the exact
    // (double-double) solution, and every published value within its
    // bound. The refinement runs only there.
    let early = [1e-11, 1e-10, 1e-9, 1e-8, 1e-7];
    let (mut worst_w0, mut worst_ratio) = ((0.0f64, String::new()), (0.0f64, String::new()));
    // The draining battery only: k8 s1017's every solve is refined, and the
    // CC ramp's samples between events run to 1e5.
    for ramp in [false] {
        for which in 0..3 {
            let b = fx_extreme(which, ramp);
            let _refined = newton::hooks::record_refinements();
            let (ratio, w0, seen, at, sat) = fx_published_within_bound(&b, &early, 400);
            let refined = newton::hooks::refinements().len();
            eprintln!("{}: solve {w0:.2e} of W0 ({sat}); published {ratio:.4} of its bound over {seen} samples (at {at}); {refined} refinements", b.name);
            assert!(seen > 50, "{}: {seen} samples", b.name);
            if which == 0 {
                assert!(refined > 0, "{}: never refined", b.name);
            }
            if w0 > worst_w0.0 {
                worst_w0 = (w0, format!("{}: {sat}", b.name));
            }
            if ratio > worst_ratio.0 {
                worst_ratio = (ratio, format!("{}: {at}", b.name));
            }
        }
    }
    assert!(worst_w0.0 <= 0.01, "a solve {} of W0 from the exact one at {}", worst_w0.0, worst_w0.1);
    assert!(worst_ratio.0 <= 1.0, "published {} of its bound at {}", worst_ratio.0, worst_ratio.1);
}

/// A ladder of `n` nodes off the rail: `r` from the rail to node 2, `r`
/// between each node and the next, an LED from each to ground. One hub
/// partition of `n` nodes, curved by its `n` junctions.
fn fx_led_ladder(n: usize, r: f64, ramp: bool) -> Program {
    let mut devs = d7_supply(ramp);
    let mut ci = 2;
    for k in 0..n {
        devs.push(resistor(ci, if k == 0 { 1 } else { 1 + k }, 2 + k, r));
        devs.push(led(ci + 1, 2 + k, 0));
        ci += 2;
    }
    program(2 + n, devs)
}

#[test]
fn a_hub_solves_rounding_floor_costs_three_solves_not_a_column_each() {
    // FXEFv: every curved hub solve formed its rounding floor (R23 (i)) from
    // every column of `J^-1`, `n + 1` solves through its factors, O(n^3): a
    // 128-node hub on a CC ramp ran 3.3x bda1839's wall time on the same
    // solves. A conductance network's floor takes three (the distance,
    // `J^-1` of the rounding at the solve and of its rate), O(n^2), and
    // stays an upper bound (`curvature::conductance_matrix`).
    let p = fx_led_ladder(64, 47.0, true);
    let (before, stall_before) = (newton::hooks::floor_solves(), newton::hooks::stall_solves());
    let mut rig = Rig::load(&p).unwrap();
    for k in 1..=200 {
        rig.advance(4e-3 * k as f64 / 200.0).unwrap();
    }
    let (floors, solves) = newton::hooks::floor_solves();
    let (floors, solves) = (floors - before.0, solves - before.1);
    let stall = newton::hooks::stall_solves() - stall_before;
    let part = rig.s().local_index[2].unwrap().0;
    assert_eq!(rig.s().static_solver.hub_solve(part).unwrap().0.len(), 64, "one 64-node hub");
    eprintln!("{floors} floors, {solves} solves through their factors; {stall} for stalled polishes' rounding");
    assert!(floors > 20, "{floors} floors");
    // A stalled polish forms its own rounding floor (`n` solves) only where
    // its step is above the relative stall floor; FXE round 1 formed it at
    // every stall.
    assert!(stall <= floors, "{stall} solves for stalled polishes' rounding over {floors} hub solves");
    assert!(solves <= 3 * floors, "{solves} solves for {floors} floors: {:.1} each", solves as f64 / floors as f64);
}

#[test]
fn a_large_hub_publishes_within_its_flow_bounded_floor() {
    // FXE round 2. A hub of at least `FLOW_MIN_N` nodes forms its rounding
    // floor by flow (three solves, `curvature::floors`), which bounds a
    // branch's share of the rounding by all of it rather than by its own
    // column: an upper bound still. On a 12-node LED ladder and a
    // 10-node GMIN-held cluster beside a divider, every published node
    // voltage, resistor and LED current and draw is within its bound
    // against the exact (double-double) solution, node by node.
    use FxEl::{Led, Sw, R};
    let mut ladder = vec![R(1, 2, 47.0)];
    for k in 2..=13 {
        if k < 13 {
            ladder.push(R(k, k + 1, 47.0 + k as f64));
        }
        ladder.push(Led(k, 0));
    }
    let mut cluster = vec![R(1, 2, 229.9), R(2, 0, 60.8), Led(3, 2), Sw(3, 0), Led(0, 3)];
    for k in 3..11 {
        cluster.push(R(k, k + 1, 10.8 + k as f64));
    }
    cluster.push(Led(11, 0));
    let early = [1e-11, 1e-10, 1e-9, 1e-8, 1e-7];
    let mut worst = (0.0f64, String::new());
    for (name, els) in [("a 12-node LED ladder", ladder), ("a 10-node floating cluster", cluster)] {
        let b = fx_board(name, false, els, (0.0, 0.0));
        let before = newton::hooks::floor_solves();
        let (ratio, w0, seen, at, sat) = fx_published_within_bound(&b, &early, 200);
        let (floors, solves) = newton::hooks::floor_solves();
        eprintln!("{}: published {ratio:.4} of its bound over {seen} samples (at {at}); solve {w0:.2e} of W0 ({sat}); {} solves for {} floors", b.name, solves - before.1, floors - before.0);
        assert!(seen > 50, "{}: {seen} samples", b.name);
        assert!(solves - before.1 <= 3 * (floors - before.0), "{}: not taken by flow", b.name);
        if ratio > worst.0 {
            worst = (ratio, format!("{}: {at}", b.name));
        }
    }
    assert!(worst.0 <= 1.0, "published {} of its bound at {}", worst.0, worst.1);
}

// ---- FXH: a PSU never holds its rail past its setpoint ------------------------

/// W1gV4's min2: a `{11.2858 V, 0.2095 A}` PSU, a closed switch shorting
/// its rail, and an `ne555` on the rail (TRIG and THR grounded, OUT driving
/// RST, DIS on the rail), plus a resistor `load` from the rail when given.
/// The short holds the PSU in CC at load with the 555 unpowered; opened, the
/// 555 powers up with OUT low (RST, on OUT, holds it there), so its DIS
/// transistor loads the rail.
fn psu_555_short(load: Option<f64>, exact_mode: bool) -> Program {
    let mut p555 = vec![0.0; ic_param::COUNT];
    p555[ic_param::VCC_MIN] = 2.0;
    p555[ic_param::VCC_MAX] = 18.0;
    p555[ic_param::NOMINAL] = 5.0;
    let mut devices = vec![
        psu(0, 1, 0, 11.2858, 0.2095),
        switch(1, 1, 0, true),
        Device { kind: kinds::NE555, component_index: 2, pins: [1usize, 0, 0, 5, 5, 3, 0, 1].map(pin).to_vec(), params: p555, blob: vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] },
    ];
    if let Some(r) = load {
        devices.push(resistor(3, 1, 0, r));
    }
    let mut p = Program { node_count: 6, options: Options::default(), digital_slot_count: 16, channel_slot_count: 0, devices };
    p.options.no_rail_shortcut = exact_mode;
    p
}

/// The rail at 0.4-0.7 s after the short opens at 0.3 s.
fn rail_after_the_short_opens(p: &Program) -> Vec<f64> {
    let mut e = Engine::load(p).unwrap();
    e.advance(0.3).unwrap();
    e.set_params(&[(1, 0, 0.0)]).unwrap();
    (1..=4)
        .map(|k| {
            e.advance(0.3 + 0.1 * k as f64).unwrap();
            e.voltages()[1]
        })
        .collect()
}

#[test]
fn a_cc_rail_whose_ic_powers_off_past_the_setpoint_is_no_cc_solution() {
    // W1gV4's bug A: with the switch opened, the CC rail solved from the
    // short's 0.2 mV, where the 555 is unpowered, so it draws nothing and the
    // rail went to iLimit over the node shunt, 2.1e11 V (209.5 V with a 1k
    // load), where the 555 is past its window and draws nothing still. The
    // draw at the setpoint (the 555's DIS) stayed over the limit, so the
    // draw test held CC there. A CC solution past the setpoint is no current
    // limit (Rails point 5: CC exits when the CC solution reaches the
    // setpoint), and a PSU never drives its rail past its setpoint. Exact
    // mode, which solves the rail with the 555 as one Newton, is the
    // reference.
    for load in [None, Some(1000.0)] {
        let got = rail_after_the_short_opens(&psu_555_short(load, false));
        let want = rail_after_the_short_opens(&psu_555_short(load, true));
        for (g, w) in got.iter().zip(&want) {
            assert!(*g <= 11.2858, "load {load:?}: the rail at {g} V, past the 11.2858 V setpoint");
            close(*g, *w, 1e-4 + 1e-3 * w.abs(), &format!("load {load:?}: default against exact mode"));
        }
    }
}

/// W1gV4's min4: a `{5 V, 0.1 A}` PSU, a 9 V ideal source behind 10 ohm and
/// a 1k load on the rail, with a rail capacitor when given.
fn psu_backfed(rail_cap: bool, exact_mode: bool) -> Program {
    let mut devices = vec![psu(0, 1, 0, 5.0, 0.1), source(1, 2, 0, 9.0), resistor(2, 1, 2, 10.0), resistor(3, 1, 0, 1000.0)];
    if rail_cap {
        devices.push(capacitor(4, 1, 0, 1e-6, 0.0));
    }
    let p = program(3, devices);
    if exact_mode {
        exact(p)
    } else {
        p
    }
}

#[test]
fn a_psu_sinking_past_its_limit_stays_cv_in_exact_mode_as_in_default() {
    // W1gV4's bug B: the 9 V source pushes (9 - 5)/10 = 0.4 A into the rail
    // and the load takes 5 mA, so the draw at the setpoint is -0.395 A, past
    // the limit against the PSU's own direction. The CC solution, (0.1 +
    // 0.9) / (0.1 + 0.001) = 9.901 V, is past the setpoint, so the compliance
    // clamp keeps CV (Rails point 5), as classic and default mode do: the
    // rail at 5 V and the PSU sinking 0.395 A (elementI = -I). Exact mode
    // entered CC on |draw| alone and held 9.901 V; with a rail capacitor it
    // alternated CC and CV each step, the capacitor's current growing to
    // kiloamps.
    for rail_cap in [false, true] {
        for exact_mode in [false, true] {
            let what = format!("rail capacitor {rail_cap}, exact mode {exact_mode}");
            let mut e = Engine::load(&psu_backfed(rail_cap, exact_mode)).unwrap();
            for t in [1e-3, 0.1, 1.0] {
                e.advance(t).unwrap();
                close(e.voltages()[1], 5.0, 1e-9, &format!("{what}: the rail at {t} s"));
                close(e.currents()[0], 0.395, 1e-9, &format!("{what}: the PSU's current at {t} s"));
                assert_eq!(e.device_state(0).unwrap(), &[Regime::Cv.code()], "{what}: CV at {t} s");
            }
        }
    }
}

#[test]
fn a_gate_output_held_far_above_its_supply_keeps_a_band_on_a_cc_ramp() {
    // W1gV4's frozen seed 84 (tpl 1, hub 2), reduced: a `{10 V, 0.1 A}` PSU
    // ramps 1 uF in CC, and a 74LS00 on the rail (inputs grounded) drives
    // 1y into a 9 V source through 10 ohm. While the gate is powered (4.275
    // to 5.775 V) 1y sits near 8.7 V, twice its supply, so its node follows
    // the rail at a slope bound past 2 (each driver's conductance falls with
    // the supply, and its far end is 4.4 to 8.7 V away). The band's
    // sensitivity bound assumed a fixed 2, failed to close, and the
    // partition took the narrowest band, 1e-12 V: on a rail ramping at 1e5
    // V/s every event came 1 fs after the last, and the run froze at 4.275 V
    // (each instant moved the rail, so no guard declined it). The ramp
    // reaches 10 V at about 0.1 ms; a host stepping event by event gets
    // there within the instants a normal ramp takes (885 with a 5 V source).
    let devices = vec![
        psu(0, 1, 0, 10.0, 0.1),
        capacitor(1, 1, 0, 1e-6, 0.0),
        ls00(2, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1e-8, 0),
        resistor(3, 2, 3, 10.0),
        source(4, 3, 0, 9.0),
    ];
    let mut e = Engine::load(&program(4, devices)).unwrap();
    let (t_end, mut instants) = (2e-4, 0);
    while e.time() < t_end && instants < 20_000 {
        let next = e.next_event_time().min(t_end);
        e.advance(next).unwrap();
        instants += 1;
    }
    assert!(e.time() >= t_end, "frozen at {:e} s after {instants} instants, the rail at {} V", e.time(), e.voltages()[1]);
    close(e.voltages()[1], 10.0, 1e-9, "the rail in CV");
}

#[test]
fn an_exact_mode_cc_solution_past_the_setpoint_keeps_cv() {
    // W1gV4's cold seed 839 (tpl 6, hub 2, exact mode), reduced: a `{5 V,
    // 20 mA}` PSU and a 74LS00 on the rail driving 47 ohm from 1y (inputs
    // grounded). Powered, the gate draws 27.8 mA at the setpoint and more
    // than 20 mA anywhere in its 4.275-5.775 V window; below it and above
    // it it draws nothing. So the only CC solution is the node shunt's,
    // 0.02 A / 1e-12 S = 2e10 V, where the gate is past its window: exact
    // mode entered CC on the draw and held 2e10 V for good (static), or
    // published it every other step (with a capacitor, stepped). That CC
    // solution is past the setpoint, so the compliance clamp keeps CV (Rails
    // point 5), as classic's does: the rail at 5 V, the PSU over its limit.
    // (Default mode declines this board at load, `-4`: the power-gate
    // ping-pong, F5's.)
    for with_cap in [false, true] {
        let mut devices = vec![psu(0, 1, 0, 5.0, 0.02), ls00(1, 1, 0, [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1e-8, 0), resistor(2, 2, 0, 47.0)];
        if with_cap {
            devices.push(capacitor(3, 2, 0, 1e-6, 0.0));
        }
        let mut e = Engine::load(&exact(program(3, devices))).unwrap();
        let mut t = 0.0;
        while t < 0.01 {
            t += 2.5e-4;
            e.advance(t).unwrap();
            let what = format!("capacitor {with_cap}, t {t:.4} s");
            assert!(e.voltages()[1] <= 5.0, "{what}: the rail at {} V, past the setpoint", e.voltages()[1]);
            assert_eq!(e.device_state(0).unwrap(), &[Regime::Cv.code()], "{what}: CV");
            close(e.voltages()[1], 5.0, 1e-12, &what);
        }
    }
}

// ---- FXI: a back-driven CC exit keeps the rail capacitors' charge (bug D) ----

/// A `{5 V, 0.1 A}` PSU ramping a 1 mF rail capacitor (esr 5 ohm) in CC
/// against a 2.2 ohm load to a pulse source at 0 V, which rises to 8 V at 5
/// ms for 0.2 ms; with `rc_load`, a 1k + 1 uF RC on the rail too (a hub
/// partition, so the exit's re-solves take a second pass of the rail loop).
fn back_driven_rail_cap(rc_load: bool, exact_mode: bool) -> Program {
    let mut devices = vec![
        psu(0, 1, 0, 5.0, 0.1),
        capacitor(1, 1, 0, 1e-3, 5.0),
        pulse(2, 2, 0, [0.0, 8.0, 5e-3, 0.0, 0.0, 2e-4, 20e-3], 0),
        resistor(3, 2, 1, 2.2),
    ];
    if rc_load {
        devices.push(resistor(4, 1, 3, 1000.0));
        devices.push(capacitor(5, 3, 0, 1e-6, 0.0));
    }
    let p = program(4, devices);
    if exact_mode {
        exact(p)
    } else {
        p
    }
}

/// The PSU's rail (node 1) every `dt` to `t_end`; stops at the first error,
/// which it returns with the time it was met.
fn rail_samples(p: &Program, dt: f64, t_end: f64) -> (Vec<(f64, f64)>, Option<(f64, BoError)>) {
    let mut e = Engine::load(p).unwrap();
    let (mut out, mut t) = (Vec::new(), 0.0);
    while t < t_end {
        t = (t + dt).min(t_end);
        if let Err(err) = e.advance(t) {
            return (out, Some((t, err)));
        }
        out.push((t, e.voltages()[1]));
    }
    (out, None)
}

#[test]
fn a_cc_exit_a_back_driven_edge_forces_keeps_the_rail_capacitors_charge() {
    // FXI, bug D, reduced. In CC the capacitor charges from 0 V at the limit
    // against the 2.2 ohm load and holds the rail near 0.19 V by 5 ms. The
    // pulse's rising edge back-drives the rail by (8 - 5) / 2.2 = 1.36 A at
    // the setpoint, so the CC solution at the instant (the capacitor's 0.11 V
    // charge plus the drop its current takes across 5 ohm) is 5.73 V, past
    // the setpoint: CV, the PSU sinking (ruling R40). The exit's tie held
    // the capacitor at V - esr (iLimit - draw(V)) = -2.29 V, as if the rail
    // had ramped to the setpoint with the supply current continuous; the
    // pulse's fall then entered CC from that charge and published -0.47 V. A
    // capacitor's charge is continuous: it keeps the 0.11 V and charges
    // toward 5 V through its esr while the pulse is high, as in classic and
    // exact mode (classic: the rail at 5.000000 V at 5.1 ms and 0.244587 V at
    // 5.2 ms, its minimum 0.153484 V; this probe's TS script is the
    // reviewer's, at 1e-4 s steps). With the RC load the exit's re-solve
    // took a second pass, whose CV -> CC test read the capacitor's 0.98 A
    // demand against the 1.36 A back-drive, entered CC on |I| = 0.38 A past
    // the limit, and tied the capacitor at -2.29 V again on the exit that
    // followed: a back-driven total keeps CV (Rails point 5's clamp).
    for rc_load in [false, true] {
        let (got, err) = rail_samples(&back_driven_rail_cap(rc_load, false), 1e-4, 0.06);
        let (want, err_exact) = rail_samples(&back_driven_rail_cap(rc_load, true), 1e-4, 0.06);
        assert!(err.is_none() && err_exact.is_none(), "RC load {rc_load}: {err:?} {err_exact:?}");
        for (&(t, g), &(_, w)) in got.iter().zip(&want) {
            let what = format!("RC load {rc_load}, t {t:.4} s");
            assert!((0.0..=5.0).contains(&g), "{what}: the rail at {g} V, outside [0, 5]");
            close(g, w, 5e-4, &format!("{what}: default against exact mode"));
        }
        let at = |t: f64| got.iter().find(|s| (s.0 - t).abs() < 1e-9).unwrap().1;
        close(at(5.1e-3), 5.0, 1e-12, &format!("RC load {rc_load}: CV while the pulse back-drives the rail"));
        close(at(5.2e-3), 0.2446, 3e-3, &format!("RC load {rc_load}: CC again from the capacitor's own charge after the fall"));
    }
}

fn raw(kind: u32, ci: usize, pins: &[usize], params: &[f64], blob: &[f64]) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params: params.to_vec(), blob: blob.to_vec() }
}

/// W1gV4's cold fuzz seed 704 (tpl 0, hub 2), without its ratings: a `{5.058
/// V, 0.164 A}` PSU with a 2.8 mF rail capacitor (esr 1.44 ohm), a -1.6 to
/// 8.8 V pulse source on node 3 (and a 2.8 V source stacked on it) whose
/// rising edge at 7.17 ms back-drives the rail by 19.8 A through an LED
/// (dev 21) and a 0.07 ohm fuse (dev 9), and a 74LS00 with two outputs on the
/// rail.
fn fuzz_board_704(exact_mode: bool) -> Program {
    let devices = vec![
        raw(21, 0, &[1, 0], &[5.057803645242242, 0.1643490365417486], &[]),
        raw(24, 1, &[3, 0], &[-1.6482243571471615, 8.772181009178164, 0.007167128715436583, 0.0, 0.0, 0.0010967034831029113, 0.3489404885979338], &[8.0]),
        raw(2, 2, &[4, 3], &[2.8207735079366767], &[]),
        raw(1, 3, &[2, 0], &[105987.3367292531], &[]),
        raw(1, 4, &[5, 0], &[15142.52266554762], &[]),
        raw(1, 5, &[6, 0], &[4193.48134804897], &[]),
        raw(50, 6, &[1, 0, 1, 1, 5, 0, 0, 1, 0, 0, 1, 0, 0, 5], &[4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.6100802728800438e-7], &[0.0; 13]),
        raw(1, 7, &[5, 0], &[196.50994043032895], &[]),
        raw(1, 8, &[1, 1], &[94.33945966859177], &[]),
        raw(10, 9, &[1, 6], &[0.07141015036884849, 0.6918195233027702], &[]),
        raw(30, 10, &[1, 5], &[1.6479173590043614, 1.9660987089501443, 0.020233545639578816, 0.0, 0.0, 0.0], &[]),
        raw(1, 11, &[1, 1], &[29.490278448878296], &[]),
        raw(30, 12, &[5, 2], &[3.0244475548048952, 2.3743550561623117, 0.01149533899561269, 0.0, 0.0, 0.0], &[]),
        raw(1, 13, &[1, 5], &[1545.7703692649864], &[]),
        raw(1, 14, &[3, 2], &[2.1906054679620595], &[]),
        raw(1, 15, &[2, 1], &[102.37750757175063], &[]),
        raw(3, 16, &[1, 0], &[0.002777433066412672, 1.4425789352517004, 0.0], &[]),
        raw(30, 17, &[4, 2], &[3.2314399819533524, 1.7346332886951767, 0.004606310734245304, 0.0, 0.0, 0.0], &[]),
        raw(1, 18, &[4, 4], &[155.42080789087976], &[]),
        raw(1, 19, &[2, 5], &[29644.923607048462], &[]),
        raw(30, 20, &[0, 6], &[2.393607971456686, 2.3129841578257255, 0.038234714867376965, 0.0, 0.0, 0.0], &[]),
        raw(30, 21, &[3, 6], &[1.9229336677436781, 2.2647670741600114, 0.030417092569347222, 0.0, 0.0, 0.0], &[]),
        raw(1, 22, &[3, 3], &[159.27187461695368], &[]),
        raw(1, 23, &[3, 0], &[1.2529850049883364], &[]),
    ];
    let mut p = Program { node_count: 7, options: Options::default(), digital_slot_count: 32, channel_slot_count: 0, devices };
    p.options.no_rail_shortcut = exact_mode;
    p
}

/// W1gV4's cold fuzz seed 985 (tpl 4, hub 2), without its ratings: a `{4.230
/// V, 82 mA}` PSU with a 98 uF rail capacitor (esr 3.73 ohm), a 0 to 5.87 V
/// clock on node 3 tied to the rail by a closed switch (dev 14, toggled at
/// 0.458 s), and an ne555 on the rail.
fn fuzz_board_985(exact_mode: bool) -> Program {
    let devices = vec![
        raw(21, 0, &[1, 0], &[4.229565420957204, 0.08204819285851947], &[]),
        raw(22, 1, &[3, 0], &[4.5523553977778155, 0.2456431034458153, 5.866016623637785, 0.0, 0.0], &[8.0]),
        raw(1, 2, &[2, 0], &[36287.47800787177], &[]),
        raw(1, 3, &[4, 0], &[81539.85296266287], &[]),
        raw(1, 4, &[5, 0], &[499069.779500247], &[]),
        raw(1, 5, &[6, 0], &[26481.85738586337], &[]),
        raw(1, 6, &[7, 0], &[106623.31252877027], &[]),
        raw(3, 7, &[1, 0], &[9.815673390615174e-5, 3.7267111161942257, 0.0], &[]),
        raw(40, 8, &[1, 0, 4, 7, 7, 6, 0, 1], &[2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], &[9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        raw(1, 9, &[1, 1], &[81341.97342773926], &[]),
        raw(1, 10, &[1, 0], &[22012.613787986375], &[]),
        raw(3, 11, &[0, 0], &[4.6712382127812963e-7, 0.0, 0.0], &[]),
        raw(1, 12, &[7, 3], &[299.07270451094774], &[]),
        raw(1, 13, &[0, 3], &[2053.7398377339387], &[]),
        raw(4, 14, &[1, 3], &[1.0], &[]),
        raw(1, 15, &[3, 0], &[208.7922685096376], &[]),
        raw(1, 16, &[1, 2], &[14.53887732518689], &[]),
        raw(1, 17, &[7, 5], &[52339.41310490844], &[]),
        raw(1, 18, &[0, 3], &[55103.10480804695], &[]),
        raw(1, 19, &[0, 6], &[16833.06131455854], &[]),
        raw(1, 20, &[7, 1], &[16735.44716992589], &[]),
        raw(4, 21, &[7, 2], &[0.0], &[]),
        raw(1, 22, &[2, 0], &[3660.6313878993333], &[]),
        raw(1, 23, &[2, 3], &[162.05324192171082], &[]),
        raw(1, 24, &[1, 5], &[18.918788164939212], &[]),
    ];
    let mut p = Program { node_count: 8, options: Options::default(), digital_slot_count: 32, channel_slot_count: 0, devices };
    p.options.no_rail_shortcut = exact_mode;
    p
}

#[test]
fn the_fuzz_boards_whose_back_driven_exit_published_a_negative_rail_stay_within_the_supply() {
    // FXI, bug D, as the fuzz found it. Seed 704: the pulse's rising edge
    // back-drives the rail by 19.8 A at the setpoint while the PSU ramps its
    // capacitor in CC at 0.57 V; the exit's tie held the capacitor at -23.7
    // V, and 56 us later, when the LED on the path burnt open, CC re-entered
    // from that charge and published -22.4 V (and an LED from the return to
    // the fuse's far end burnt, forward-biased by the negative rail). Seed 985: a clock tied
    // to the rail by a closed switch back-drives it by 1636 A on each rising
    // edge; the tie held the capacitor at -6093 V and the rail published as
    // low as -5960 V. A PSU's rail lies between 0 and its setpoint unless a
    // source back-drives it (and then the CV supply sinks, ruling R40):
    // sampled at 0.1 ms the rail stays in [0, V] and follows exact mode.
    // Seed 704 ends at 0.37 s: at 0.3845 s its 74LS00's power gate
    // ping-pongs on the CC rail (`-4`, F5's class; exact mode declines at
    // 0.381 s too). Its rail differs from exact mode by a few mV because the
    // modes part on the 0.07 ohm fuse: exact mode trips it at the edge, on a
    // step's end current, where default mode integrates the 19.8 A for the
    // 56 us the LED lasts (0.05 of the 0.3 latch). Seed 985 ends at 0.45 s:
    // the switch opening at 0.458 s is `-4` (the CV -> CC clamp's CC
    // candidate, a rail Newton across the 555's power window, does not
    // converge: the class the board met at 1.0068 s before the fix).
    let v_704 = 5.057803645242242;
    let (got, err) = rail_samples(&fuzz_board_704(false), 1e-4, 0.37);
    assert!(err.is_none(), "seed 704: {err:?}");
    for &(t, u) in &got {
        assert!((0.0..=v_704).contains(&u), "seed 704: the rail at {u} V at {t:.4} s, outside [0, V]");
    }
    let (want, _) = rail_samples(&fuzz_board_704(true), 1e-4, 0.37);
    for t in [0.1, 0.2, 0.3] {
        let at = |s: &[(f64, f64)]| s.iter().find(|x| (x.0 - t).abs() < 1e-9).unwrap().1;
        close(at(&got), at(&want), 5e-3, &format!("seed 704: default against exact mode at {t} s"));
    }
    let v_985 = 4.229565420957204;
    let (got, err) = rail_samples(&fuzz_board_985(false), 1e-4, 0.45);
    let (want, err_exact) = rail_samples(&fuzz_board_985(true), 1e-4, 0.45);
    assert!(err.is_none() && err_exact.is_none(), "seed 985: {err:?} {err_exact:?}");
    for (&(t, g), &(_, w)) in got.iter().zip(&want) {
        assert!((0.0..=v_985).contains(&g), "seed 985: the rail at {g} V at {t:.4} s, outside [0, V]");
        close(g, w, 1e-4, &format!("seed 985 at {t:.4} s: default against exact mode"));
    }
}

// ---- F5b: W1c point 1a on the fuzz boards ----------------------------------

/// A fuzz board's device: plain pins (no flags, no hashes).
fn fz(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob }
}

/// A fuzz board as its sweep ran it, event by event: a host advancing to
/// each of `stops`, toggling each
/// switch of `toggles` (param 0 between 0 and 1) once it reaches its time.
/// Returns the rig, every change of IC `d`'s power hold with the time and
/// its supply then, `(t, hold, supply)`, and the first error with the time
/// it left the engine at.
fn hold_trace(p: &Program, toggles: &[(f64, usize)], stops: &[f64], d: usize) -> (Rig, Vec<(f64, digital::power::PowerHold, f64)>, Option<(KernelError, f64)>) {
    let mut rig = Rig::load(p).unwrap_or_else(|e| panic!("load: {}", e.message));
    let supply = |rig: &Rig| {
        let dev = &p.devices[d];
        rig.v(dev.pins[0].node) - rig.v(dev.pins[1].node)
    };
    let mut trace = vec![(0.0, rig.s().power_hold(d), supply(&rig))];
    // Event by event between the stops, so each hold and release is seen at
    // its own instant.
    let mut err = None;
    let mut state: Vec<f64> = p.devices.iter().map(|d| d.params.first().copied().unwrap_or(0.0)).collect();
    let mut ev = 0;
    'run: for &stop in stops {
        while rig.k.time() < stop {
            let t = rig.k.next_event_time().min(stop);
            if let Err(e) = rig.advance(t) {
                err = Some((e, rig.k.time()));
                break 'run;
            }
            let hold = rig.s().power_hold(d);
            if hold != trace.last().unwrap().1 {
                trace.push((t, hold, supply(&rig)));
            }
        }
        while ev < toggles.len() && toggles[ev].0 <= stop {
            let dv = toggles[ev].1;
            state[dv] = 1.0 - state[dv];
            let result = rig.k.apply_params_as_event(&[(dv, 0, state[dv])]);
            rig.k.solver_mut().prepare_readout();
            if let Err(e) = result {
                err = Some((e, rig.k.time()));
                break 'run;
            }
            ev += 1;
        }
    }
    (rig, trace, err)
}

/// The trace's holds `(time, release level, supply)` and releases `(time,
/// supply)`, in order.
fn holds_and_releases(trace: &[(f64, digital::power::PowerHold, f64)]) -> (Vec<(f64, f64, f64)>, Vec<(f64, f64)>) {
    let (mut held, mut freed) = (Vec::new(), Vec::new());
    for w in trace.windows(2) {
        let ((_, before, _), (t, hold, supply)) = (w[0], w[1]);
        // Off to Off is the same hold at a new level or instant (a retry
        // that moved it, ruling R51 (b); a hold dropped and taken again
        // within one instant), not another.
        if hold.kind == digital::power::HoldKind::Off && before.kind != digital::power::HoldKind::Off {
            held.push((t, hold.release.expect("a release level").level, supply));
        } else if before.kind == digital::power::HoldKind::Off && hold.is_free() {
            freed.push((t, supply));
        }
    }
    if trace[0].1.kind == digital::power::HoldKind::Off {
        held.insert(0, (0.0, trace[0].1.release.expect("a release level").level, trace[0].2));
    }
    (held, freed)
}

#[test]
fn a_g2bv_gate_starving_its_cell_at_load_is_held_off_and_again_after_a_toggle() {
    // G2bv's bc_ic seed 20 hub 1 (F5D: "1a, within an instant"), which
    // declined its load: a 74LS00 on a 4.2774 V cell of 0.236 ohm, 2y and
    // 3y (node 6) high into 1588 and 4703 ohm, 4y (node 3) high against 309
    // ohm to the rail. Powered, its draw takes the rail under its 4.275 V
    // floor; held off, the cell's EMF is above it. So it loads held off,
    // its release the floor plus that drop and two reached widths: above
    // the EMF, never reached. A toggle (`set_params`) drops the hold, and
    // the same instant takes it again (W1c point 1a).
    let p = { let mut p = program(7, g2bv_bc_ic_20_hub1()); p.digital_slot_count = 16; p };
    let stops = [0.00010054129745299623, 0.001267074627819937, 0.19185406847861228, 0.44552757564924855, 0.44644865932111427, 0.5];
    let (rig, trace, err) = hold_trace(&p, &[(0.44552757564924855, 9)], &stops, 6);
    assert!(err.is_none(), "{err:?}");
    let (held, freed) = holds_and_releases(&trace);
    eprintln!("bc_ic 20: held {held:?}, freed {freed:?}");
    let e = 4.277440234153144;
    assert_eq!(held.len(), 1, "{trace:?}");
    assert_eq!(held[0].0, 0.0, "held at the load");
    assert!(held[0].1 > e && held[0].1 < e + 0.01, "the release level past the EMF: {}", held[0].1);
    assert!(freed.is_empty());
    assert_eq!(rig.s().power_hold(6).kind, digital::power::HoldKind::Off);
    assert_eq!(rig.s().hold_counts(), [2.0, 0.0], "the load's hold and the toggle's");
    assert!(rig.u() > 0.9 * 4.75 && rig.v(6).abs() < 1e-6, "rail {} V above the floor, the gate's outputs dark: {} V", rig.u(), rig.v(6));
}

#[test]
fn a_rail_newton_alternating_a_555s_power_gate_holds_it_off_until_the_rail_passes_its_release() {
    // The cold fuzz's seed 237 tpl 3 (F5D: "1a in the rail Newton"), which
    // declined at 47.4 us: a {5.03 V, 62 mA} PSU ramping 1.6 uF (esr 24
    // mohm), a 555 on the rail with its output (node 6) driving an LED to
    // a -3.35 V source. Its 1.8 V power-on edge sits where the 555's draw
    // takes the rail Newton's iterates back under it (1.79988 <-> 1.80000
    // V). The rail Newton's flip holds it off at the edge, its drop 1.19e-4
    // V; the rail, rising at about 37,000 V/s with it off, reaches the
    // release level as a `Rail` event (Rails point 4's release threshold)
    // some 3.2 ns later, where it powers: its draw through the capacitor's
    // esr takes the rail back down by the drop, to within a few reached
    // widths of the edge, still inside, and the ramp carries it.
    let p = { let mut p = program(7, cold_237_tpl3()); p.options.integration = 1; p.digital_slot_count = 32; p };
    let stops = [0.006272880129900816, 0.006486738525025257, 0.006639124636807362, 0.010253136567057732, 0.010513169565513698, 0.03044452364163602, 0.034443802061474564, 0.04971881825306094, 0.049992873499283136, 0.06374826736193694, 0.06994028776299031, 0.07];
    let (rig, trace, err) = hold_trace(&p, &[], &stops, 11);
    assert!(err.is_none(), "{err:?}");
    let (held, freed) = holds_and_releases(&trace);
    eprintln!("cold 237: held {held:?}, freed {freed:?}");
    assert_eq!((held.len(), freed.len()), (1, 1), "{trace:?}");
    let (t_held, level, s_held) = held[0];
    let (t_freed, s_freed) = freed[0];
    let eta = rails::REACHED_REL * 1.8;
    assert!((s_held - 1.8).abs() < 1e-6 && level > 1.8 + 2.0 * eta && level < 1.8 + 1e-3, "held at the edge: supply {s_held}, release {level}");
    assert!(t_held > 4.7e-5 && t_held < 4.9e-5, "{t_held}");
    assert!(t_freed > t_held && t_freed - t_held < 1e-8, "released {:e} s later", t_freed - t_held);
    // Powered at the release, the supply sits the drop under it: inside the
    // window by a few reached widths, the release's own two (D1 (a)) on top
    // of the one or two its `Rail` event lands past the level by. Without
    // them it would sit only that landing past the edge (3.6e-12 V, two
    // widths, against 7.2e-12 V), one rail Newton tolerance from starving
    // again.
    assert!(s_freed - 1.8 >= 3.0 * eta && s_freed - 1.8 < 5.0 * eta, "the supply {:e} V past the edge once powered", s_freed - 1.8);
    assert_eq!(rig.s().hold_counts(), [1.0, 1.0]);
}

#[test]
fn an_iterated_partition_alternating_a_555s_power_gate_holds_it_off_while_its_load_is_there() {
    // The cold fuzz's seed 12 tpl 4 (F5D: "probably 1a, iterated static
    // partition with a resistor-fed IC supply"), which declined at 14.7 ms:
    // a 555 whose vcc (node 2) is fed from a {12.43 V, 0.46 A} PSU through
    // 144 ohm, its OUT on node 3, which a 42.6 Hz clock (7.07 V for 14.7 ms,
    // then -0.29 V to 23.4 ms) holds, with 270 uF. When the clock falls,
    // OUT high into it collapses the 555's own supply under its 1.8 V floor;
    // unpowered, the supply is the rail's. The partition's Newton flips the
    // gate each iterate; held off, the drop is 11 V and the release level,
    // 12.87 V, past the PSU. Ruling R51 (b): each re-solve retries it, as
    // classic's H8 does at each host step, and the clock's rise at 23.4 ms
    // takes the load away, so it powers there (until F5k it stayed dark to
    // the end). Powered, its latch meets NE555 point 11's zero-delay loop,
    // which declines `-7` at that instant (ruling R49 (d), classic's
    // fallback), as it would have at the load had it powered then.
    let p = { let mut p = program(8, cold_12_tpl4()); p.digital_slot_count = 32; p };
    let stops = [0.006272879619865001, 0.006418236135522505, 0.006653719691401212, 0.04368244956136634, 0.043851961234653535, 0.06987367313960807, 0.07058263333052203, 0.096299810615255, 0.11091739978284508, 0.13294549546641507, 0.15];
    let (rig, trace, err) = hold_trace(&p, &[], &stops, 15);
    let (held, freed) = holds_and_releases(&trace);
    eprintln!("cold 12: held {held:?}, freed {freed:?}, {err:?}");
    assert_eq!(held.len(), 1, "{trace:?}");
    let (t_held, level, s_held) = held[0];
    assert!(t_held > 0.0146 && t_held < 0.0148, "{t_held}");
    assert!(level > 12.434588688285583 && s_held < level, "a release level past the PSU: supply {s_held}, release {level}");
    let period = 1.0 / 42.647402311824166;
    match err {
        Some((KernelError::Livelock { at }, _)) => assert!((at - period).abs() < 1e-6, "declined at the clock's rise, {at} s"),
        other => panic!("the latch loop's -7 at the clock's rise: {other:?}"),
    }
    assert_eq!(rig.s().hold_counts(), [1.0, 1.0], "released at the retry");
}

/// A fuzz board as its sweep ran it, through the engine: an `advance` to
/// each of `stops`, then each switch of `toggles` due by then toggled
/// (param 0 between 0 and 1). The first error, with the time it left.
fn sweep_run(p: &Program, toggles: &[(f64, usize)], stops: &[f64]) -> Result<Engine, (BoError, f64)> {
    let mut e = Engine::load(p).map_err(|er| (er, 0.0))?;
    let mut state: Vec<f64> = p.devices.iter().map(|d| d.params.first().copied().unwrap_or(0.0)).collect();
    let mut ev = 0;
    for &stop in stops {
        e.advance(stop).map_err(|er| (er, e.time()))?;
        while ev < toggles.len() && toggles[ev].0 <= stop {
            let d = toggles[ev].1;
            state[d] = 1.0 - state[d];
            e.set_params(&[(d as u32, 0, state[d])]).map_err(|er| (er, e.time()))?;
            ev += 1;
        }
    }
    Ok(e)
}

#[test]
fn a_555_whose_output_feeds_its_own_supply_converges_at_a_low_supply() {
    // The cold fuzz's seed 1120 tpl 2 (F5d's D7 search), declined `-4` at
    // 13.4 us: a 555 fed through 30.9 kohm from a CC rail its own OUT pin
    // sits on, DIS on its vcc. Its OUT stage's conductance to vcc is `f(vs)
    // / 20` (NE555 point 7), steep at a low supply; solved with that
    // conductance frozen at each iterate, the partition's iteration only
    // contracted by 0.89 a step (vs 1.80, 0.10, ..., then 0.77, 0.24, 0.67,
    // ... toward 0.41 V) and hit its 50-iteration cap. Newton with the
    // conductance's derivative in the Jacobian converges, as the hub
    // polish's does, and the power hold then holds the 555 off (W1c point
    // 1a): powered it pulls its supply to 0.41 V through DIS.
    let board = |exact_mode: bool| { let mut p = program(6, cold_1120_tpl2()); p.digital_slot_count = 32; p.options.integration = 1; p.options.no_rail_shortcut = exact_mode; p };
    let toggles = [(0.41324546951082397, 14), (0.601000495581227, 14), (0.6806880874994053, 14)];
    let stops = [1.35e-5, 0.1, 0.41324546951082397, 0.601000495581227, 0.6806880874994053, 1.2];
    match sweep_run(&board(false), &toggles, &stops) {
        Ok(e) => assert!(e.stat(25) >= 1.0, "held off at least once"),
        Err((er, t)) => panic!("{} at {t}: {}", er.code, er.message),
    }
    // Exact mode's stepped seed alternates the same gate (powered at a 2.31 V
    // supply, 0.16 V after), and a stepped partition's Newton runs no hold
    // (W1c point 1a): its `-4` stands, at the same instant as before.
    let Err((er, t)) = sweep_run(&board(true), &toggles, &stops) else { panic!("exact mode's stepped H8 now passes: update this pin") };
    assert_eq!(er.code, -4);
    assert!((t - 1.023e-5).abs() < 1e-8 && er.message.starts_with("stepped: Newton did not converge"), "{t} {}", er.message);
}

#[test]
fn a_555_whose_supply_switch_opens_converges_without_a_hold() {
    // The cold fuzz's seed 1325 tpl 0 (F5d's D7 search), declined `-4` at
    // the switch toggle at 0.613 s: opening it leaves the 555's vcc (node 6)
    // fed through 510 ohm from DIS's node. Solved with the OUT and DIS
    // conductances frozen at each iterate, the partition 2-cycled between a
    // powered iterate (vs 2.56 V) and an unpowered one (1.75 V), so the power
    // hold ran; its powered probe converged at 1.98 V, inside the window,
    // and with no IC starving there the procedure declined. That probe is a
    // solution of the free partition: Newton with the conductances'
    // derivatives finds it directly. Exact mode's step declined there too.
    let board = |exact_mode: bool| { let mut p = program(8, cold_1325_tpl0()); p.digital_slot_count = 32; p.options.no_rail_shortcut = exact_mode; p };
    let toggles = [(0.6132293487216949, 7)];
    let stops = [
        0.006272876767779332, 0.006480661829038629, 0.03440297140027209, 0.0380641329025449, 0.044002088431166535, 0.06665239476678818, 0.10654709267762458, 0.11408661311212073, 0.12116016198788658, 0.12564713556801815, 0.14598381797610285, 0.14664850923070183,
        0.1468188098690709, 0.16056491591787245, 0.1646468664631822, 0.2066528843471837, 0.20724479979408952, 0.20747836626119665, 0.2564271719827684, 0.2976674142574927, 0.2979597497153942, 0.2990134639035969, 0.32152737436969925, 0.32193917016753537, 0.3222475190937351,
        0.37084977750817055, 0.4134040971516132, 0.41438822960946525, 0.46138471825377536, 0.46595783237299987, 0.4661448109338016, 0.46647074150173456, 0.4689111511755701, 0.4861494398218669, 0.4866372989934134, 0.488466984380709, 0.491985592965016, 0.5373769607134146,
        0.553044905348276, 0.5579314518689056, 0.5610904955350021, 0.5843675623509273, 0.5992511162198577, 0.6132293487216949, 0.7,
    ];
    for exact_mode in [false, true] {
        let p = board(exact_mode);
        let stats = |e: &Engine| [e.stat(25), e.stat(26)];
        let before = sweep_run(&p, &toggles, &stops[..stops.len() - 2]).map(|e| stats(&e)).unwrap_or_else(|(er, t)| panic!("exact={exact_mode}: {} at {t}: {}", er.code, er.message));
        match sweep_run(&p, &toggles, &stops) {
            // No hold taken at the toggle (default mode holds and releases
            // once earlier, as at the head).
            Ok(e) => assert_eq!(stats(&e), before, "exact={exact_mode}"),
            Err((er, t)) => panic!("exact={exact_mode}: {} at {t}: {}", er.code, er.message),
        }
    }
}

#[test]
fn the_fed_gate_on_a_cc_ramp_is_held_at_its_floor_and_released_at_the_next_re_solve() {
    // FXB's extra board, FXDv's "k6 s39 ramp" (declined at step 226 of
    // 3,000, 0.9 ms): a gate fed from a {5 V, 0.1 A} ramp into 100 uF
    // through 8.43 ohm (node 2), its 1y into 791 ohm and an LED. At its 0.9 V
    // floor the polish of its supply's partition ping-pongs across the edge.
    // Held off there (W1c point 1a), it is released at the partition's
    // first re-solve past its release level (on a static partition, the
    // release is checked at each re-solve): the rail band's 25 mV later,
    // 25 us on this ramp.
    let mut devs = d7_supply(true);
    devs.extend([resistor(2, 1, 2, 8.433658904889839), gate_io(3, 2, 0, [0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0.004, 1.0, 6.0), resistor(4, 3, 4, 791.0671698769225), led(5, 4, 0)]);
    let mut p = no_cache(program(5, devs));
    p.digital_slot_count = p.digital_slot_count.max(8);
    let stops: Vec<f64> = (1..=3000).map(|k| 12e-3 * k as f64 / 3000.0).collect();
    let (rig, trace, err) = hold_trace(&p, &[], &stops, 3);
    assert!(err.is_none(), "{err:?}");
    let (held, freed) = holds_and_releases(&trace);
    eprintln!("k6 s39: held {held:?}, freed {freed:?}");
    assert_eq!((held.len(), freed.len()), (1, 1), "{trace:?}");
    let ((t_held, level, s_held), (t_freed, s_freed)) = (held[0], freed[0]);
    assert!((s_held - 0.9).abs() < 1e-6 && level > 0.9 && level < 0.9 + 1e-6, "held at the floor: supply {s_held}, release {level}");
    assert!((t_held - 0.9e-3).abs() < 1e-6, "{t_held}");
    assert!(s_freed >= level && t_freed > t_held && t_freed - t_held < 3e-5, "released at {t_freed} (supply {s_freed})");
    assert!(rig.v(2) > 4.0 && rig.v(3) > 2.0, "powered and high at 12 ms: supply {} V, 1y {} V", rig.v(2), rig.v(3));
}

#[test]
fn the_fxh_boards_that_declined_at_load_load_with_their_gate_held_off() {
    // FXH's new declines (cold seeds 785 tpl 0 and 830 tpl 3, both `{V,
    // iLimit}` PSUs): a 74LS00 on the rail whose power flip each rail-loop
    // pass flips the PSU's regime too (CC <-> CV, the rail between -10.05
    // and 4.70 V on 785, 2.11 and 5.03 V on 830), so the load declined.
    // Held powered, its supply collapses in CC: it starves itself, and
    // loads held off. 785's PSU then sits in CV at its setpoint, 4.699 V;
    // 830's limits at 3.26 V with the gate off, under its floor (a part
    // whose supply reads low either way is held too, ruling D2 (a)). Each
    // release level is past the setpoint, so neither is released over 50
    // ms; 785's toggle at 18 ms drops the hold, and the toggle's instant
    // takes it again.
    for (name, p, toggles, stops, d, v) in [
        ("785 tpl 0", { let mut p = program(8, cold_785_tpl0()); p.digital_slot_count = 32; p }, vec![(0.01840803191451679, 16)], vec![0.006272880216392937, 0.01840803191451679, 0.02495905461262398, 0.05], 12, 4.698635237910),
        ("830 tpl 3", { let mut p = program(7, cold_830_tpl3()); p.digital_slot_count = 32; p }, vec![], vec![0.0062728803007134695, 0.006590779933999605, 0.02225723242285205, 0.05], 8, 5.032450386993),
    ] {
        let (rig, trace, err) = hold_trace(&p, &toggles, &stops, d);
        assert!(err.is_none(), "{name}: {err:?}");
        let (held, freed) = holds_and_releases(&trace);
        eprintln!("{name}: held {held:?}, freed {freed:?}, counts {:?}", rig.s().hold_counts());
        assert_eq!(held[0].0, 0.0, "{name}: held at the load");
        assert!(freed.is_empty() && held.iter().all(|h| h.1 > v), "{name}: release levels past the setpoint {v}: {held:?}");
        assert_eq!(rig.s().hold_counts(), [toggles.len() as f64 + 1.0, 0.0], "{name}");
        assert!(rig.u() <= v + 1e-9, "{name}: the rail {} V", rig.u());
    }
}

#[test]
fn the_fxdv_edge_board_loads_held_off_between_its_floor_and_its_carrying_rail() {
    // FXDv's power-edge sweep (`fxdv_edgeband` k2): a gate (window 3-6 V,
    // ioMax 16 mA) whose supply (node 2) hangs off a stiff PSU through 5.36
    // ohm, its 1y into 48.7 ohm. Every load from 2.7001 V to 2.8467 V
    // declined: powered its 1y draw takes its supply under the 2.7 V floor.
    // Each now loads with the gate held off, its supply at the rail (less
    // the node shunt's share), and its release level the floor plus that
    // drop and two reached widths, past the rail: never reached. Above
    // 2.8467 V it carries itself and loads powered with no hold.
    let board = |v: f64| {
        let mut p = no_cache(program(4, vec![psu(0, 1, 0, v, 20.0), resistor(1, 1, 0, 1e9), resistor(2, 1, 2, 5.36), gate_io(3, 2, 0, [0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0.016, 3.0, 6.0), resistor(4, 3, 0, 48.7)]));
        p.digital_slot_count = p.digital_slot_count.max(8);
        p
    };
    for v in [2.7001, 2.75, 2.84] {
        let rig = Rig::load(&board(v)).unwrap_or_else(|e| panic!("{v} V: {}", e.message));
        let hold = rig.s().power_hold(3);
        assert_eq!(hold.kind, digital::power::HoldKind::Off, "{v} V");
        close(rig.v(2), v, 1e-9, &format!("{v} V: the supply at the rail"));
        assert!(rig.v(3).abs() < 1e-6, "{v} V: 1y dark");
        assert!(hold.release.unwrap().level > v, "{v} V: {hold:?}");
    }
    let rig = Rig::load(&board(2.86)).unwrap();
    assert!(rig.s().power_hold(3).is_free() && rig.v(2) > 2.7 && rig.v(3) > 1.0, "2.86 V: powered, supply {} V, 1y {} V", rig.v(2), rig.v(3));
    assert_eq!(rig.s().hold_counts(), [0.0, 0.0]);
}

/// tpl=3 hub=2 e=5.032 psu ilim=0.062 V2=-3.35 SW4-5(0)+R5-0=266 LED6-2+R1-6=2596 C1-0=1.6e-6 555 vcc1 gnd0 [6, 1, 1, 6, 0, 0] SW6-4(1)+R4-0=36 R4-5=30.8 exact=false span=1.2e1
/// options: Options { integration: 1, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 7
/// toggles [(0.2538321494839014, 6), (0.7156499115983714, 17), (0.77843404319527, 6)]
/// ERR advance to 0.006272880129900816 at t 4.744769960698e-5: rail: the rail Newton did not converge within 50 iterations
fn cold_237_tpl3() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![5.032283712497381, 0.0624090419716193], vec![]),
        fz(2, 1, &[2, 0], vec![-3.3526961194572937], vec![]),
        fz(1, 2, &[3, 0], vec![70272.8000456936], vec![]),
        fz(1, 3, &[4, 0], vec![13354.719005664909], vec![]),
        fz(1, 4, &[5, 0], vec![271665.5368311562], vec![]),
        fz(1, 5, &[6, 0], vec![172932.70075604186], vec![]),
        fz(4, 6, &[4, 5], vec![0.0], vec![]),
        fz(1, 7, &[5, 0], vec![265.7078397145019], vec![]),
        fz(30, 8, &[6, 2], vec![2.393827904419805, 2.269320847980911, 0.018193723652586924, 0.0, 0.0, 0.0], vec![]),
        fz(1, 9, &[1, 6], vec![2595.698870452701], vec![]),
        fz(3, 10, &[1, 0], vec![1.6387355562614893e-6, 0.02374653780097165, 0.0], vec![]),
        fz(40, 11, &[1, 0, 6, 1, 1, 6, 0, 0], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 12, &[1, 0], vec![43553.62314192138], vec![]),
        fz(1, 13, &[0, 0], vec![5058.019598667813], vec![]),
        fz(3, 14, &[0, 0], vec![1.8680443805564595e-7, 0.0, 0.0], vec![]),
        fz(1, 15, &[1, 3], vec![834.9825057226262], vec![]),
        fz(1, 16, &[3, 4], vec![8344.683954650249], vec![]),
        fz(4, 17, &[6, 4], vec![1.0], vec![]),
        fz(1, 18, &[4, 0], vec![36.16031100042536], vec![]),
        fz(1, 19, &[4, 5], vec![30.788119774492785], vec![]),
        fz(90, 7, &[], vec![7.0, 0.2008457024821117, 265.7078397145019], vec![]),
        fz(90, 9, &[], vec![9.0, 0.09404392727682356, 2595.698870452701], vec![]),
        fz(90, 12, &[], vec![12.0, 0.00014427389762369818, 43553.62314192138], vec![]),
        fz(90, 13, &[], vec![13.0, 0.026110869025297915, 5058.019598667813], vec![]),
        fz(90, 15, &[], vec![15.0, 0.0006229133837456091, 834.9825057226262], vec![]),
        fz(90, 18, &[], vec![18.0, 0.00944959256290787, 36.16031100042536], vec![]),
        fz(90, 19, &[], vec![19.0, 0.14999542647824116, 30.788119774492785], vec![]),
    ]
}

/// tpl=4 hub=2 e=12.435 psu ilim=0.458 CLK3 -0.3..7.1 C3-0=2.6e-7 R3-4=1229.3 LED4-0+R1-4=152 C3-0=2.7e-4 R6-7=16794.5 SW5-6(1)+R6-0=30 555 vcc2 gnd0 [6, 3, 2, 4, 4, 1] LED4-5+R1-4=31 R4-3=717.3 exact=false span=1.2e1
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 8
/// toggles [(0.726518064166707, 13), (0.8039048447848541, 13)]
/// ERR advance to 0.04368244956136634 at t 1.469704221435e-2: partition 0 (1 node(s)): Newton did not converge within 50 iterations
fn cold_12_tpl4() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![12.434588688285583, 0.4575726818390958], vec![]),
        fz(22, 1, &[3, 0], vec![42.647402311824166, 0.6282912521373276, 7.069653443862801, -0.29054133390678727, 0.0], vec![8.0]),
        fz(1, 2, &[2, 0], vec![586250.4385286777], vec![]),
        fz(1, 3, &[4, 0], vec![14181.802531581461], vec![]),
        fz(1, 4, &[5, 0], vec![1296.6007360361198], vec![]),
        fz(1, 5, &[6, 0], vec![6009.607747559662], vec![]),
        fz(1, 6, &[7, 0], vec![5487.11251627804], vec![]),
        fz(3, 7, &[3, 0], vec![2.6017239069042806e-7, 0.0, 0.0], vec![]),
        fz(1, 8, &[3, 4], vec![1229.2795190953807], vec![]),
        fz(30, 9, &[4, 0], vec![2.0391708025181674, 1.9530375831782436, 0.01034668892259119, 0.0, 0.0, 0.0], vec![]),
        fz(1, 10, &[1, 4], vec![151.56499619749388], vec![]),
        fz(3, 11, &[3, 0], vec![0.00027193837893659114, 0.0, 0.0], vec![]),
        fz(1, 12, &[6, 7], vec![16794.5036047125], vec![]),
        fz(4, 13, &[5, 6], vec![1.0], vec![]),
        fz(1, 14, &[6, 0], vec![29.960765255159092], vec![]),
        fz(40, 15, &[2, 0, 6, 3, 2, 4, 4, 1], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 16, &[2, 1], vec![144.42399433878745], vec![]),
        fz(1, 17, &[1, 4], vec![1831.4031043148443], vec![]),
        fz(3, 18, &[4, 0], vec![2.1821725802011662e-7, 0.0, 0.0], vec![]),
        fz(1, 19, &[3, 5], vec![51.36602119409897], vec![]),
        fz(30, 20, &[4, 5], vec![1.8151462559678644, 2.1494323498494436, 0.021354434014062573, 0.0, 0.0, 0.0], vec![]),
        fz(1, 21, &[1, 4], vec![30.73890448554215], vec![]),
        fz(1, 22, &[4, 3], vec![717.3374325430739], vec![]),
        fz(90, 8, &[], vec![8.0, 0.0002474966530743104, 1229.2795190953807], vec![]),
        fz(90, 10, &[], vec![10.0, 0.003402518897282526, 151.56499619749388], vec![]),
        fz(90, 12, &[], vec![12.0, 0.00015670410762658456, 16794.5036047125], vec![]),
        fz(90, 14, &[], vec![14.0, 0.048806308419276856, 29.960765255159092], vec![]),
        fz(90, 16, &[], vec![16.0, 0.0022432594918316653, 144.42399433878745], vec![]),
        fz(90, 17, &[], vec![17.0, 0.004689202167196722, 1831.4031043148443], vec![]),
        fz(90, 19, &[], vec![19.0, 2.5035405037013048, 51.36602119409897], vec![]),
        fz(90, 21, &[], vec![21.0, 0.2751832854536309, 30.73890448554215], vec![]),
        fz(90, 22, &[], vec![22.0, 0.2396166905698217, 717.3374325430739], vec![]),
    ]
}

/// tpl=2 hub=2 e=4.920 psu ilim=0.064 555 vcc4 gnd0 [2, 1, 3, 5, 1, 4] LED3-4 C0-0=1.5e-7 SW0-2(0)+R2-0=97 R4-3=83100.5 R1-5=9.7 span=12
/// options: trapezoid; digital_slot_count 32 node_count 6
/// toggles [(0.41324546951082397, 14), (0.601000495581227, 14), (0.6806880874994053, 14)]
fn cold_1120_tpl2() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![4.919813081257697, 0.06422823184384589], vec![]),
        fz(1, 1, &[2, 0], vec![1621.2718893138422], vec![]),
        fz(1, 2, &[3, 0], vec![2397.397399638335], vec![]),
        fz(1, 3, &[4, 0], vec![40121.13778668658], vec![]),
        fz(1, 4, &[5, 0], vec![35238.55077529291], vec![]),
        fz(40, 5, &[4, 0, 2, 1, 3, 5, 1, 4], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 6, &[4, 4], vec![5279.234468491432], vec![]),
        fz(1, 7, &[4, 1], vec![30886.293024395007], vec![]),
        fz(3, 8, &[1, 0], vec![2.2158174428755075e-7, 0.0, 0.0], vec![]),
        fz(1, 9, &[1, 1], vec![85.97288883696424], vec![]),
        fz(1, 10, &[0, 1], vec![2922.2966863273396], vec![]),
        fz(30, 11, &[3, 4], vec![3.005352562049808, 1.742146566268996, 0.005953065042841878, 0.0, 0.0, 0.0], vec![]),
        fz(3, 12, &[0, 0], vec![1.4567680570576984e-7, 0.12685877358451655, 0.0], vec![]),
        fz(1, 13, &[2, 0], vec![549.3706091249462], vec![]),
        fz(4, 14, &[0, 2], vec![0.0], vec![]),
        fz(1, 15, &[2, 0], vec![96.61088950213237], vec![]),
        fz(1, 16, &[4, 3], vec![83100.47188631297], vec![]),
        fz(1, 17, &[1, 5], vec![9.71640748808111], vec![]),
        fz(90, 6, &[], vec![6.0, 0.0017731272930075598, 5279.234468491432], vec![]),
        fz(90, 7, &[], vec![7.0, 1.3214763198567896e-5, 30886.293024395007], vec![]),
        fz(90, 9, &[], vec![9.0, 0.05402837727657123, 85.97288883696424], vec![]),
        fz(90, 15, &[], vec![15.0, 0.013157392561162534, 96.61088950213237], vec![]),
        fz(90, 16, &[], vec![16.0, 1.1561470750484285e-5, 83100.47188631297], vec![]),
        fz(90, 17, &[], vec![17.0, 7.204855684324213, 9.71640748808111], vec![]),
    ]
}

/// tpl=0 hub=2 e=7.270 psu ilim=0.069 V2=-3.95 SW3-6(1)+R6-0=43 555 vcc6 gnd0 [3, 2, 4, 6, 1, 4] R4-1=60363.7 F2-5=4.44/0.28 LED1-5+R1-1=1565 C1-0=5.9e-4 LED3-2+R1-3=57 span=12
/// options: default; digital_slot_count 32 node_count 8
/// toggles [(0.6132293487216949, 7)]
fn cold_1325_tpl0() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![7.270217999288251, 0.06853528685274506], vec![]),
        fz(2, 1, &[0, 2], vec![-3.9480110144696248], vec![]),
        fz(1, 2, &[3, 0], vec![17927.736465463717], vec![]),
        fz(1, 3, &[4, 0], vec![425942.82621893083], vec![]),
        fz(1, 4, &[5, 0], vec![68876.33381574621], vec![]),
        fz(1, 5, &[6, 0], vec![1155.6332825473146], vec![]),
        fz(1, 6, &[7, 0], vec![8183.941804003441], vec![]),
        fz(4, 7, &[3, 6], vec![1.0], vec![]),
        fz(1, 8, &[6, 0], vec![42.83384507667242], vec![]),
        fz(40, 9, &[6, 0, 3, 2, 4, 6, 1, 4], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 10, &[6, 4], vec![509.96405976607474], vec![]),
        fz(1, 11, &[4, 1], vec![804.4665292582611], vec![]),
        fz(3, 12, &[1, 0], vec![2.6185663685936847e-6, 0.0, 0.0], vec![]),
        fz(1, 13, &[2, 4], vec![119.80128517249696], vec![]),
        fz(1, 14, &[4, 1], vec![60363.744445343604], vec![]),
        fz(1, 15, &[1, 5], vec![19387.009125813216], vec![]),
        fz(10, 16, &[2, 5], vec![4.43695307462206, 0.282002451222728], vec![]),
        fz(30, 17, &[1, 5], vec![1.6193140140033933, 2.208334563638683, 0.03897740944372803, 0.0, 0.0, 0.0], vec![]),
        fz(1, 18, &[1, 1], vec![1565.1938432065601], vec![]),
        fz(3, 19, &[1, 0], vec![0.0005940317553685062, 0.0, 0.0], vec![]),
        fz(30, 20, &[3, 2], vec![3.3237763170220314, 1.6464963983528853, 0.02467490927476345, 0.0, 0.0, 0.0], vec![]),
        fz(1, 21, &[1, 3], vec![56.76867957123571], vec![]),
        fz(1, 22, &[3, 2], vec![1301.909184654966], vec![]),
        fz(90, 8, &[], vec![8.0, 0.00672371243155751, 42.83384507667242], vec![]),
        fz(90, 10, &[], vec![10.0, 0.031154085664473176, 509.96405976607474], vec![]),
        fz(90, 11, &[], vec![11.0, 0.011559379280864642, 804.4665292582611], vec![]),
        fz(90, 13, &[], vec![13.0, 0.5558929144002336, 119.80128517249696], vec![]),
        fz(90, 14, &[], vec![14.0, 0.0020163745295198555, 60363.744445343604], vec![]),
        fz(90, 18, &[], vec![18.0, 0.03917389377034387, 1565.1938432065601], vec![]),
        fz(90, 21, &[], vec![21.0, 0.33842795881290666, 56.76867957123571], vec![]),
    ]
}

/// seed 20 hub=1 e=4.277 r=0.236 IC SW5-4(0)+R4-0=125 C0-4=1.17e-4 R4-5=29.7
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 16 node_count 7
/// toggles [(0.44552757564924855, 9)] resets []
fn g2bv_bc_ic_20_hub1() -> Vec<Device> {
    vec![
        fz(20, 0, &[1, 0], vec![1.0, 0.2359746390621206, 1.0, 0.0, 2.0, 0.05, 1.0, 1.0, 0.0], vec![1.0, 0.5, 4.277440234153144]),
        fz(1, 1, &[2, 0], vec![8169.826763026575], vec![]),
        fz(1, 2, &[3, 0], vec![4978.782842137732], vec![]),
        fz(1, 3, &[4, 0], vec![40295.55222654737], vec![]),
        fz(1, 4, &[5, 0], vec![170385.8384458016], vec![]),
        fz(1, 5, &[6, 0], vec![4703.109940985932], vec![]),
        fz(50, 6, &[1, 0, 3, 1, 6, 0, 0, 3, 0, 0, 3, 0, 0, 6], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.3641153272136766e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 7, &[6, 0], vec![1587.8675069059157], vec![]),
        fz(1, 8, &[3, 1], vec![308.7042287061871], vec![]),
        fz(4, 9, &[5, 4], vec![0.0], vec![]),
        fz(1, 10, &[4, 0], vec![125.39044642469158], vec![]),
        fz(3, 11, &[0, 4], vec![0.00011719543557716233, 0.0, 0.0], vec![]),
        fz(1, 12, &[4, 5], vec![29.7326372318558], vec![]),
    ]
}


/// tpl=0 hub=2 e=4.699 psu ilim=0.022 V2=-0.65 CLK3 0.0..2.7 R6-4=2477.2 SW2-3(1)+R3-0=50 C0-6=2.1e-4 R3-1=306.4 74LS00 vcc1 gnd0 in5,7 out3,5 R7-0=113639.4 SW0-5(1)+R5-0=359 LED3-2+R3-3=1763 C3-0=3.2e-5 P7-5 exact=false span=1.2e1
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 8
/// toggles [(0.01840803191451679, 16), (0.4294995383338517, 8), (0.8515432372531131, 16), (0.9361844041441881, 8), (0.984110775576708, 16)]
/// ERR load: rail: the rail loop did not settle within 50 passes
fn cold_785_tpl0() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![4.6986352379097385, 0.0224080962134651], vec![]),
        fz(2, 1, &[2, 0], vec![-0.6472895151846014], vec![]),
        fz(22, 2, &[3, 0], vec![2.2225169323243628, 0.3302223480133346, 2.6923237130559348, 0.0, 0.0], vec![8.0]),
        fz(1, 3, &[4, 0], vec![38725.28183211784], vec![]),
        fz(1, 4, &[5, 0], vec![33010.78201015608], vec![]),
        fz(1, 5, &[6, 0], vec![2940.791478302445], vec![]),
        fz(1, 6, &[7, 0], vec![236828.43574410884], vec![]),
        fz(1, 7, &[6, 4], vec![2477.1901409763395], vec![]),
        fz(4, 8, &[2, 3], vec![1.0], vec![]),
        fz(1, 9, &[3, 0], vec![49.86313691266805], vec![]),
        fz(3, 10, &[0, 6], vec![0.00020739826170311886, 0.0, 0.0], vec![]),
        fz(1, 11, &[3, 1], vec![306.39624757098414], vec![]),
        fz(50, 12, &[1, 0, 5, 7, 3, 0, 0, 5, 0, 0, 5, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 4.0997027213537435e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 13, &[3, 6], vec![256.4019899053665], vec![]),
        fz(1, 14, &[5, 0], vec![70.37765453290346], vec![]),
        fz(1, 15, &[7, 0], vec![113639.36908703843], vec![]),
        fz(4, 16, &[0, 5], vec![1.0], vec![]),
        fz(1, 17, &[5, 0], vec![359.2485883074942], vec![]),
        fz(30, 18, &[3, 2], vec![2.918255369660087, 2.0356134343839525, 0.020741351040892617, 0.0, 0.0, 0.0], vec![]),
        fz(1, 19, &[3, 3], vec![1762.7684407704048], vec![]),
        fz(3, 20, &[3, 0], vec![3.1662513665532644e-5, 0.0, 0.0], vec![]),
        fz(1, 21, &[2, 3], vec![230.3495713460602], vec![]),
        fz(13, 22, &[7, 5], vec![0.13436614554678622, 0.07246446569779841], vec![]),
        fz(90, 7, &[], vec![7.0, 0.00011626177295752732, 2477.1901409763395], vec![]),
        fz(90, 9, &[], vec![9.0, 0.05733803209954079, 49.86313691266805], vec![]),
        fz(90, 11, &[], vec![11.0, 0.1887874285982294, 306.39624757098414], vec![]),
        fz(90, 13, &[], vec![13.0, 0.029933484890954754, 256.4019899053665], vec![]),
        fz(90, 14, &[], vec![14.0, 0.004980589081934987, 70.37765453290346], vec![]),
        fz(90, 15, &[], vec![15.0, 1.956317982121651e-5, 113639.36908703843], vec![]),
        fz(90, 17, &[], vec![17.0, 0.004824604075116485, 359.2485883074942], vec![]),
        fz(90, 19, &[], vec![19.0, 0.003944198862336364, 1762.7684407704048], vec![]),
    ]
}

/// tpl=3 hub=2 e=5.032 psu ilim=0.061 V2=3.43 LED4-3+R1-4=1796 74LS00 vcc1 gnd0 in2,0 out2,3 SW0-5(1)+R5-0=71 R3-1=110.7 LED6-0 SW1-2(0)+R2-0=46 R6-4=4989.0 R1-4=21.8 SW3-5(1)+R5-0=94 LED0-2+R0-0=31 exact=false span=1.2e1
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 7
/// toggles [(0.10061352023320264, 20), (0.19698806894745943, 11), (0.45088136135248097, 20), (1.039448424072327, 16), (1.1051088564106837, 16)]
/// ERR load: rail: the rail loop did not settle within 50 passes
fn cold_830_tpl3() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![5.032450386993319, 0.06091321705131145], vec![]),
        fz(2, 1, &[2, 0], vec![3.433591124553459], vec![]),
        fz(1, 2, &[3, 0], vec![1498.6166111379346], vec![]),
        fz(1, 3, &[4, 0], vec![1197.7689637237856], vec![]),
        fz(1, 4, &[5, 0], vec![3228.904742582944], vec![]),
        fz(1, 5, &[6, 0], vec![7714.894472290962], vec![]),
        fz(30, 6, &[4, 3], vec![2.5835970637091763, 1.7963230902191438, 0.02760832503000144, 0.0, 0.0, 0.0], vec![]),
        fz(1, 7, &[1, 4], vec![1795.65980870523], vec![]),
        fz(50, 8, &[1, 0, 2, 0, 2, 0, 0, 3, 0, 0, 3, 0, 0, 2], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.3117093235179186e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 9, &[2, 3], vec![2193.9018993128116], vec![]),
        fz(1, 10, &[3, 3], vec![2431.2047826609914], vec![]),
        fz(4, 11, &[0, 5], vec![1.0], vec![]),
        fz(1, 12, &[5, 0], vec![70.74845246806566], vec![]),
        fz(1, 13, &[3, 1], vec![110.74331156557903], vec![]),
        fz(30, 14, &[6, 0], vec![1.6725954721020677, 2.0817777066683005, 0.025021474381575665, 0.0, 0.0, 0.0], vec![]),
        fz(1, 15, &[3, 5], vec![414.6610334109262], vec![]),
        fz(4, 16, &[1, 2], vec![0.0], vec![]),
        fz(1, 17, &[2, 0], vec![46.039152458289756], vec![]),
        fz(1, 18, &[6, 4], vec![4988.952275821875], vec![]),
        fz(1, 19, &[1, 4], vec![21.791530100445204], vec![]),
        fz(4, 20, &[3, 5], vec![1.0], vec![]),
        fz(1, 21, &[5, 0], vec![93.53702483711008], vec![]),
        fz(30, 22, &[0, 2], vec![2.7881303560890545, 2.1180573610353983, 0.0292305309118783, 0.0, 0.0, 0.0], vec![]),
        fz(1, 23, &[0, 0], vec![31.069654400738116], vec![]),
        fz(90, 7, &[], vec![7.0, 0.029719603785865387, 1795.65980870523], vec![]),
        fz(90, 9, &[], vec![9.0, 0.029898455171604717, 2193.9018993128116], vec![]),
        fz(90, 10, &[], vec![10.0, 0.014966827633910763, 2431.2047826609914], vec![]),
        fz(90, 12, &[], vec![12.0, 0.12419952316098057, 70.74845246806566], vec![]),
        fz(90, 13, &[], vec![13.0, 1.4234479635167772, 110.74331156557903], vec![]),
        fz(90, 17, &[], vec![17.0, 5.499791863284933, 46.039152458289756], vec![]),
        fz(90, 18, &[], vec![18.0, 0.0045201447093539465, 4988.952275821875], vec![]),
        fz(90, 19, &[], vec![19.0, 0.37763298873628515, 21.791530100445204], vec![]),
        fz(90, 21, &[], vec![21.0, 1.5118164053726753, 93.53702483711008], vec![]),
        fz(90, 23, &[], vec![23.0, 0.16679804321612787, 31.069654400738116], vec![]),
    ]
}

/// The hold intervals `(held, freed)` of a [`hold_trace`] (an open one ends
/// at `t_end`).
fn trace_spans(trace: &[(f64, digital::power::PowerHold, f64)], t_end: f64) -> Vec<(f64, f64)> {
    let mut spans = Vec::new();
    let mut since = (trace[0].1.kind == digital::power::HoldKind::Off).then_some(trace[0].0);
    for &(t, hold, _) in &trace[1..] {
        match (since, hold.kind == digital::power::HoldKind::Off) {
            (None, true) => since = Some(t),
            (Some(t0), false) => {
                spans.push((t0, t));
                since = None;
            }
            _ => {}
        }
    }
    spans.extend(since.map(|t0| (t0, t_end)));
    spans
}

#[test]
fn the_hg1_fuzz_board_holds_its_gate_until_the_ramp_carries_it() {
    // G2bv's fz_ic seed 275 hub 2 (HG1, `gate_flips` at 4.275 V), which
    // declined at 1.3905 s: a {8.10 V, 0.385 A} PSU with 32.9 and 55.7 uF on
    // its rail (esr 0), a 74LS00 on it. After the switch toggle at 1.3901 s
    // the rail ramps back up through the gate's floor, where its draw turns
    // the segment back. Held off there (ruling R44 (a)), it would be released
    // only where powered at its edge the segment still rose; on this board
    // the limit never leaves it that, so each toggle's `set_params` drops the
    // hold and the next pass through the floor takes it again: five holds,
    // no hold/release cycle, 538 events to 3 s.
    let p = { let mut p = program(7, g2bv_fz_ic_275_hub2()); p.digital_slot_count = 16; p };
    let (rig, trace, err) = hold_trace(&p, &[(1.272630061902458, 10), (1.3901187325138928, 14), (2.219621774815895, 10), (2.3726728547854523, 8), (2.6198704533388346, 8), (2.823733970122995, 14), (2.9515980361023977, 14)], &[0.0001005412851729187, 0.0007895698576060186, 0.0011385180074891042, 0.0014955531390017994, 0.008742282512901723, 0.013441818616058119, 0.3413522952387019, 0.595897902723136, 0.7729237222644616, 0.7816529562171904, 0.8251142608707076, 1.1997709379672226, 1.2003926080936536, 1.2221591591911647, 1.226424894467912, 1.2287976179283164, 1.2351929048652155, 1.272630061902458, 1.2857769401881944, 1.2898492890622748, 1.312196653998578, 1.3153038609479852, 1.328313890403436, 1.331078503270592, 1.3323320647144672, 1.3345495519770676, 1.3901187325138928, 1.3925843080284819, 1.3942702516817582, 1.4834704479059608, 1.4926798949992932, 1.4928734885816275, 1.493177198700087, 1.5182004589425633, 1.5229088092535445, 1.5519882497027657, 1.5524312072936755, 1.7500867347305875, 1.760391847602593, 1.7983773057493389, 1.8107357089873901, 1.8223739406882635, 1.8224747068836138, 1.8235109916715213, 1.8293589937154362, 1.829553975160383, 1.8615065468379255, 1.9460438563176246, 1.9518525475748025, 1.9683703530087338, 2.1239814218603263, 2.219621774815895, 2.2825722453041997, 2.2829780089207383, 2.284419254508861, 2.353792547939792, 2.3540727176891734, 2.3726728547854523, 2.392317791298873, 2.392769084983172, 2.394320023634597, 2.395411812003443, 2.4012839152440377, 2.513954423886269, 2.5155071842455685, 2.515668254446695, 2.515857553201643, 2.6198704533388346, 2.6202214605827883, 2.6468979977732436, 2.6551294244338224, 2.6567981652860904, 2.823733970122995, 2.8317468303723583, 2.8430228246571843, 2.8575313428409324, 2.9515980361023977, 2.956319361438573, 2.986799039331321, 2.9872506284843294, 3.0], 4);
    let spans = trace_spans(&trace, 3.0);
    eprintln!("fz_ic 275: {err:?}; holds {spans:?}; counts {:?}; events {}", rig.s().hold_counts(), rig.k.stats().events_processed);
    assert!(err.is_none(), "{err:?}");
    assert!(spans.iter().all(|&(a, b)| b - a > 1e-3), "no hold shorter than the rail's own time constants: {spans:?}");
    // Each toggle drops the hold (`set_params`), and the ramp's next pass
    // through the floor takes it again: no hold/release cycle.
    assert!(rig.s().hold_counts()[0] <= 8.0, "{:?}", rig.s().hold_counts());
}

#[test]
fn a_gate_hovering_at_its_floor_on_a_cc_rail_is_held_once() {
    // G2bv's bc_ic seed 97 hub 2: a {11.2 V, 0.421 A} PSU in CC with no rail
    // capacitor, 3.16 mF from the rail to a node with 15.6 ohm to ground, and
    // a 74LS00 on the rail whose 1y drives that node through 49 ohm. The rail
    // creeps up through the gate's 4.275 V floor at the capacitor's pace, and
    // powered the gate takes it 0.11 V back. Two faults made that a cycle of
    // 11,195 holds and releases in 6 ms (0.02x real time): the closed-form
    // partition's segment key did not carry the hold, so a probe re-used the
    // segment stamped the other way and saw no drop, and a release was taken
    // on the rail loop's passes in passing rather than on the settled rail.
    // Now it is held once, 34.7 to 39.1 ms, and freed when the rail carries it.
    // (Its 1a, 1b and 1y share a node: powered, the gate rings at its own 81
    // ns delay, some 12,000 events a millisecond, which no hold touches.)
    let p = { let mut p = program(6, g2bv_bc_ic_97_hub2()); p.digital_slot_count = 16; p };
    let stops = [0.00010054129372012221, 0.0003467103846392178, 0.0004605220067872004, 0.05];
    let (rig, trace, err) = hold_trace(&p, &[], &stops, 10);
    let spans = trace_spans(&trace, 0.05);
    eprintln!("bc_ic 97: {err:?}; holds {spans:?}; counts {:?}; events {}", rig.s().hold_counts(), rig.k.stats().events_processed);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(rig.s().hold_counts(), [1.0, 1.0]);
    assert_eq!(spans.len(), 1);
    assert!(spans[0].0 > 0.0346 && spans[0].0 < 0.0348 && spans[0].1 > 0.039 && spans[0].1 < 0.0393, "{spans:?}");
    assert!(rig.u() > 0.9 * 4.75, "rail {} V", rig.u());
}

#[test]
fn a_gate_tied_to_its_own_input_is_held_at_load_then_rings_at_its_own_delay() {
    // G2bv's fz_ic seed 401 hub 2, which declined at its load: a {5.83 V,
    // 0.44 A} PSU in CC, a 74LS00 whose 1b and 1y share node 5. It loads held
    // off (its draw takes the rail under its floor) and stays so until the
    // switch toggle at 0.44 s drops the hold; powered then with 1a high, 1y
    // feeds its own input and rings at the gate's 27 ns delay, 18 MHz, two
    // events an edge: the run is slower than real time from there by the
    // circuit's own doing, not the hold's.
    let p = { let mut p = program(7, g2bv_fz_ic_401_hub2()); p.digital_slot_count = 16; p };
    let (rig, trace, err) = hold_trace(&p, &[(0.44015570685572775, 13)], &[0.00010054129153157535, 0.0003458489221323154, 0.0005618221415494645, 0.0007140772565774002, 0.004709368587030973, 0.016874565396198055, 0.018490089942201665, 0.026414640575132733, 0.027084402744392, 0.2382891458762392, 0.2393829186074303, 0.33486761095951917, 0.33498479477918414, 0.3353630577252437, 0.44015570685572775], 10);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(trace_spans(&trace, 0.44)[0].0, 0.0, "held at the load");
    assert_eq!(rig.s().hold_counts(), [1.0, 0.0], "dropped by the toggle, never cycled");
    let mut rig = rig;
    let (t0, mut edges, mut last) = (rig.k.time(), 0, rig.v(5) > 3.0);
    for k in 1..=200 {
        rig.advance(t0 + k as f64 * 1e-9).unwrap();
        if (rig.v(5) > 3.0) != last {
            last = !last;
            edges += 1;
        }
    }
    eprintln!("fz_ic 401: {edges} edges of 1y in 200 ns");
    assert!((6..=9).contains(&edges), "1y rings at about 2 x 27 ns a period: {edges} edges in 200 ns");
}

/// seed 97 hub=2 e=11.214 ilim=0.421 R5-4=81.7 SW5-0(0)+R0-0=104 R1-0=16.2 R3-4=31.4 IC R5-2=49.0 C1-2=3.16e-3 R0-2=15.6
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 16 node_count 6
/// toggles [(1.3393742423690518, 6), (2.7978206249650617, 6)] resets []
fn g2bv_bc_ic_97_hub2() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![11.213895031120575, 0.42055500594646733], vec![]),
        fz(1, 1, &[2, 0], vec![33881.81866815495], vec![]),
        fz(1, 2, &[3, 0], vec![2755.9469043071645], vec![]),
        fz(1, 3, &[4, 0], vec![6341.763697887451], vec![]),
        fz(1, 4, &[5, 0], vec![2017.279549842803], vec![]),
        fz(1, 5, &[5, 4], vec![81.67779194855792], vec![]),
        fz(4, 6, &[5, 0], vec![0.0], vec![]),
        fz(1, 7, &[0, 0], vec![104.02083318002894], vec![]),
        fz(1, 8, &[1, 0], vec![16.24375077320078], vec![]),
        fz(1, 9, &[3, 4], vec![31.438742186261308], vec![]),
        fz(50, 10, &[1, 0, 5, 5, 5, 0, 0, 2, 0, 0, 2, 0, 0, 5], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 8.111414590724573e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 11, &[5, 0], vec![120.27494562243916], vec![]),
        fz(1, 12, &[2, 1], vec![496.47046284123473], vec![]),
        fz(1, 13, &[5, 2], vec![49.03908580255002], vec![]),
        fz(3, 14, &[1, 2], vec![0.003163696267321141, 0.0, 0.0], vec![]),
        fz(1, 15, &[0, 2], vec![15.553145840323737], vec![]),
    ]
}

/// seed 401 hub=2 e=5.834 ilim=0.442 SW3-1(1)+R1-0=12 P4-5=0.91 P4-2=1.72 IC SW2-3(1)+R3-0=98 C3-1=2.06e-3
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 16 node_count 7
/// toggles [(0.44015570685572775, 13), (0.4872017451092161, 6), (1.28231178430037, 13), (2.3066926693934438, 13)] resets []
fn g2bv_fz_ic_401_hub2() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![5.833680415864328, 0.4416205865116221], vec![]),
        fz(1, 1, &[2, 0], vec![17622.05456493839], vec![]),
        fz(1, 2, &[3, 0], vec![5377.263519243568], vec![]),
        fz(1, 3, &[4, 0], vec![9253.40703554229], vec![]),
        fz(1, 4, &[5, 0], vec![109291.70698686843], vec![]),
        fz(1, 5, &[6, 0], vec![2199.8560622387677], vec![]),
        fz(4, 6, &[3, 1], vec![1.0], vec![]),
        fz(1, 7, &[1, 0], vec![11.931721114368015], vec![]),
        fz(1, 8, &[4, 5], vec![0.9133364283707476], vec![]),
        fz(1, 9, &[4, 2], vec![1.7156985347279319], vec![]),
        fz(50, 10, &[1, 0, 3, 5, 5, 0, 0, 6, 0, 0, 6, 0, 0, 5], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 2.717393732964803e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 11, &[5, 0], vec![630.4651796322628], vec![]),
        fz(1, 12, &[6, 1], vec![103.73493073749172], vec![]),
        fz(4, 13, &[2, 3], vec![1.0], vec![]),
        fz(1, 14, &[3, 0], vec![97.61422778082371], vec![]),
        fz(3, 15, &[3, 1], vec![0.00205894450326118, 0.0, 0.0], vec![]),
    ]
}
/// seed 275 hub=2 e=8.095 ilim=0.385 IC C1-0=3.29e-5 SW4-0(1)+R0-0=22 SW2-3(1)+R3-0=184 R1-2=3970 F4-0=0.44 SW3-1(1)+R1-0=12 C0-1=5.57e-5
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 16 node_count 5
/// toggles [(1.272630061902458, 10), (1.3901187325138928, 14), (2.219621774815895, 10), (2.3726728547854523, 8), (2.6198704533388346, 8), (2.823733970122995, 14), (2.9515980361023977, 14)] resets []
fn g2bv_fz_ic_275_hub2() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![8.094717445814506, 0.3845712863130265], vec![]),
        fz(1, 1, &[2, 0], vec![173574.86251306813], vec![]),
        fz(1, 2, &[3, 0], vec![112520.54007451961], vec![]),
        fz(1, 3, &[4, 0], vec![7418.156349028272], vec![]),
        fz(50, 4, &[1, 0, 1, 1, 3, 0, 0, 3, 0, 0, 3, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 6.859798610844554e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 5, &[3, 0], vec![353.2943086646341], vec![]),
        fz(1, 6, &[3, 1], vec![157.94274935630074], vec![]),
        fz(3, 7, &[1, 0], vec![3.294723434183485e-5, 0.0, 0.0], vec![]),
        fz(4, 8, &[4, 0], vec![1.0], vec![]),
        fz(1, 9, &[0, 0], vec![21.86126716827705], vec![]),
        fz(4, 10, &[2, 3], vec![1.0], vec![]),
        fz(1, 11, &[3, 0], vec![183.91515164101776], vec![]),
        fz(1, 12, &[1, 2], vec![3970.1407174723504], vec![]),
        fz(1, 13, &[4, 0], vec![0.43973610276635844], vec![]),
        fz(4, 14, &[3, 1], vec![1.0], vec![]),
        fz(1, 15, &[1, 0], vec![12.042350635818496], vec![]),
        fz(3, 16, &[0, 1], vec![5.57000337370709e-5, 0.0, 0.0], vec![]),
    ]
}

// ---- F5c: R28, a capacitor on a CC rail with no rail capacitor ----------

/// A fuzz board as its sweep ran it: a host advancing to each of `stops`,
/// toggling each switch of `toggles` once it reaches its time. Returns the
/// rig and the first error with the time it left the engine at.
fn run_stops(p: &Program, toggles: &[(f64, usize)], stops: &[f64]) -> (Rig, Option<(KernelError, f64)>) {
    let mut rig = Rig::load(p).unwrap_or_else(|e| panic!("load: {}", e.message));
    let mut state: Vec<f64> = p.devices.iter().map(|d| d.params.first().copied().unwrap_or(0.0)).collect();
    let mut ev = 0;
    for &stop in stops {
        if let Err(e) = rig.advance(stop) {
            let t = rig.k.time();
            return (rig, Some((e, t)));
        }
        while ev < toggles.len() && toggles[ev].0 <= stop {
            let dv = toggles[ev].1;
            state[dv] = 1.0 - state[dv];
            let result = rig.k.apply_params_as_event(&[(dv, 0, state[dv])]);
            rig.k.solver_mut().prepare_readout();
            if let Err(e) = result {
                let t = rig.k.time();
                return (rig, Some((e, t)));
            }
            ev += 1;
        }
    }
    (rig, None)
}

#[test]
fn a_g2bv_capacitor_on_a_cc_rail_seeds_at_the_rail_it_solves_with() {
    // G2bv's bc seed 453 hub 2 (F5D: "R28 seed rounding"), which declined at
    // 0.95 ms: an {8.39 V, 119 mA} PSU in CC with no rail capacitor, 213 uF
    // from the rail (node 1) to node 3, which LEDs and ohms tie to the rest;
    // stepped (the sweep's force_stepped). Each stepped re-solve seeds at
    // 1 ps, where the capacitor is a 2e8 S conductance into the rail, so the
    // seed's Newton tolerance became amps of draw and the rail loop moved
    // the rail 1e-6 to 2e-3 V a pass until its 50-pass `-4`. Under the CC
    // step's rail-as-unknown (Rails point 4) the seed solves the rail too,
    // and the run reaches the sweep's 3 s; the rail stays in [0, V].
    let mut p = program(6, g2bv_bc_453_hub2());
    p.options.force_stepped = true;
    p.digital_slot_count = 16;
    let stops = STOPS_453;
    let (rig, err) = run_stops(&p, &[(1.992454544220535, 8)], &stops);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(rig.k.time(), 3.0);
    assert!((0.0..=8.389274738481177).contains(&rig.u()), "the rail at {} V", rig.u());
}

#[test]
fn a_g2bv_capacitor_on_a_cc_rail_loads_at_the_rail_it_solves_with() {
    // G2bv's bc_ic seed 345 hub 2 (F5D: "R28 seed rounding"), which declined
    // its load: a {8.25 V, 208 mA} PSU in CC with no rail capacitor, 325 uF
    // from the rail to node 5 (64 ohm to the LEDs' node 4), a 74LS00 on the
    // rail. The load's seed is the same 1 ps solve as 453's re-solves.
    let mut p = program(7, g2bv_bc_ic_345_hub2());
    p.digital_slot_count = 16;
    let stops = STOPS_345;
    let (rig, err) = run_stops(&p, &[(1.356093623107948, 17)], &stops);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(rig.k.time(), 3.0);
    assert!((0.0..=8.251163385474932).contains(&rig.u()), "the rail at {} V", rig.u());
}

const STOPS_453: [f64; 82] = [0.0001005412890247001, 0.0020486456811428006, 0.0021620567116171414, 0.004167044332630853, 0.010086225945659784, 0.010234110689627481, 0.011083821408639873, 0.06326491214580818, 0.06571149610749605, 0.07179909530985325, 0.07836895021263249, 0.2572453725257941, 0.2573623282315845, 0.25779174459044335, 0.26247325731095644, 0.2952311026840107, 0.31281464391610664, 0.3133359557402932, 0.31388086151773653, 0.31413639805554594, 0.315047645114208, 0.3153014868027775, 0.31543503118242583, 0.5728342099837793, 0.5851494557963632, 0.5972385210509734, 0.7309468806729855, 0.7881086868916378, 0.7890023380878298, 0.7891766560115344, 0.8237371169119325, 1.0016615925754146, 1.096167576142008, 1.2005331806272324, 1.2127919864319743, 1.2226972814701624, 1.2238035614359857, 1.2292697209148309, 1.3645778627797196, 1.368791509122223, 1.4962674294309724, 1.4989698825477182, 1.51055795202623, 1.6027909430423262, 1.610899286188153, 1.639755509151849, 1.6417302199984132, 1.6554484361718649, 1.6563413973707375, 1.8635659457291553, 1.992454544220535, 2.151819239993597, 2.3927270277512576, 2.3943354876954075, 2.3949028740455427, 2.3956168557307533, 2.5950978982250854, 2.7223968871741233, 2.725004173136593, 2.7651708856036263, 2.7658121136924203, 2.777733379282435, 2.778157176199307, 2.811246388387834, 2.8115773860051254, 2.812083671720204, 2.8408303017178804, 2.8576353605433953, 2.8736636805677787, 2.874134526848015, 2.874422646316318, 2.8905629995494566, 2.891000506739132, 2.9175167235639314, 2.9188359842132234, 2.918998919308797, 2.93392633676662, 2.9341983587866927, 2.936318039241165, 2.9364880659355546, 2.9367969375932423, 3.0];
const STOPS_345: [f64; 89] = [0.00010054128257875011, 0.015516735039395338, 0.01563024834871797, 0.08821110342064188, 0.08854134082522593, 0.10567393324678193, 0.10580114418327866, 0.10660525403135836, 0.106974464791995, 0.10710683774002967, 0.10760995433150236, 0.20731184896125587, 0.20804981074972, 0.20824298581913306, 0.20846893909141714, 0.20927400568854454, 0.20985720446445139, 0.2929065539665082, 0.5399160387234524, 0.6027532406764107, 0.6029897075226733, 0.8538938930786757, 0.8576352331391416, 0.8585059372269214, 0.9095735333635412, 1.017273594590784, 1.2289223439177972, 1.2373575429503882, 1.2389928542551274, 1.2414173325514635, 1.3182407916756975, 1.3550309927023165, 1.3556343101830242, 1.356093623107948, 1.356476324286028, 1.356580431745361, 1.356793737818573, 1.3587094510135653, 1.3590224359449796, 1.4050899928248286, 1.406046788391974, 1.7517329655672742, 1.759945329452676, 1.7621402195637978, 1.8819959763227012, 1.8822378891840212, 1.8896470212200926, 1.89306694226731, 1.8985114931234253, 1.8987802363891635, 1.9949778996053906, 2.002230786970315, 2.0315114213871466, 2.032389097277278, 2.0388268190668417, 2.039216810935022, 2.041193738175997, 2.041523926595729, 2.0689825102096386, 2.078045742303034, 2.080921100402751, 2.0918706961014952, 2.3438101946267795, 2.3439848897532554, 2.3442627279713237, 2.3444001414879807, 2.347031864684135, 2.3797574389017315, 2.386705419144287, 2.422971140943671, 2.425603632303634, 2.4257409480000756, 2.6227789209063355, 2.6228815684200737, 2.6231185402383295, 2.624431767112084, 2.6344208064073924, 2.6347232536891703, 2.6695178009315184, 2.712124035561082, 2.7156317547196758, 2.7193768348349576, 2.7200704856414357, 2.7203109059520747, 2.730966338797728, 2.795302110411459, 2.916580843969291, 2.9167951790416566, 3.0];

/// seed 453 hub=2 e=8.389 ilim=0.119 LED0-3 C3-1=2.13e-4 R2-3=10.8 SW5-4(1)+R4-0=331 LED4-5+R1-4=32 C5-0=4.26e-4 LED2-1+R1-2=596 R2-3=7.5 stepped
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: true, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 16 node_count 6
/// toggles [(1.992454544220535, 8)] resets []
fn g2bv_bc_453_hub2() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![8.389274738481177, 0.11932898630442335], vec![]),
        fz(1, 1, &[2, 0], vec![9538.317920658634], vec![]),
        fz(1, 2, &[3, 0], vec![16663.403961873224], vec![]),
        fz(1, 3, &[4, 0], vec![27680.52542526922], vec![]),
        fz(1, 4, &[5, 0], vec![20648.46695907391], vec![]),
        fz(30, 5, &[0, 3], vec![2.703675598429488, 2.2517743232242813, 0.007791270736355658, 0.0, 0.0, 0.0], vec![]),
        fz(3, 6, &[3, 1], vec![0.0002134091716088516, 0.0, 0.0], vec![]),
        fz(1, 7, &[2, 3], vec![10.80573824928048], vec![]),
        fz(4, 8, &[5, 4], vec![1.0], vec![]),
        fz(1, 9, &[4, 0], vec![330.99782953069223], vec![]),
        fz(30, 10, &[4, 5], vec![3.0843277217893648, 2.4644891814717194, 0.019259583719938394, 0.0, 0.0, 0.0], vec![]),
        fz(1, 11, &[1, 4], vec![31.8565540004344], vec![]),
        fz(3, 12, &[5, 0], vec![0.0004257956172432858, 0.0, 0.0], vec![]),
        fz(30, 13, &[2, 1], vec![2.538346297787883, 2.2543464258188055, 0.023903634689225392, 0.0, 0.0, 0.0], vec![]),
        fz(1, 14, &[1, 2], vec![595.6304606974892], vec![]),
        fz(1, 15, &[2, 3], vec![7.549763479782209], vec![]),
    ]
}
/// seed 345 hub=2 e=8.251 ilim=0.208 LED3-4+R1-3=58 LED2-4+R1-2=1004 R4-5=63.7 R6-2=18.6 R3-0=351.7 IC C1-5=3.25e-4 SW0-4(1)+R4-0=30
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 16 node_count 7
/// toggles [(1.356093623107948, 17)] resets []
fn g2bv_bc_ic_345_hub2() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![8.251163385474932, 0.20768671027202937], vec![]),
        fz(1, 1, &[2, 0], vec![7571.242796625596], vec![]),
        fz(1, 2, &[3, 0], vec![15647.64773768837], vec![]),
        fz(1, 3, &[4, 0], vec![14514.112226490512], vec![]),
        fz(1, 4, &[5, 0], vec![2856.8022632379675], vec![]),
        fz(1, 5, &[6, 0], vec![72119.14378823017], vec![]),
        fz(30, 6, &[3, 4], vec![1.7094737965181794, 1.831920572043295, 0.01097343224620406, 0.0, 0.0, 0.0], vec![]),
        fz(1, 7, &[1, 3], vec![58.4462285974141], vec![]),
        fz(30, 8, &[2, 4], vec![1.9641552841860321, 2.283731023266931, 0.005770991396296044, 0.0, 0.0, 0.0], vec![]),
        fz(1, 9, &[1, 2], vec![1004.2045898004918], vec![]),
        fz(1, 10, &[4, 5], vec![63.67358287734673], vec![]),
        fz(1, 11, &[6, 2], vec![18.600613860612206], vec![]),
        fz(1, 12, &[3, 0], vec![351.6596273445374], vec![]),
        fz(50, 13, &[1, 0, 2, 1, 2, 0, 0, 3, 0, 0, 3, 0, 0, 2], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 6.109751872707034e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 14, &[2, 0], vec![509.34284830143264], vec![]),
        fz(1, 15, &[3, 1], vec![234.66491763496472], vec![]),
        fz(3, 16, &[1, 5], vec![0.0003245809960210505, 0.0, 0.0], vec![]),
        fz(4, 17, &[0, 4], vec![1.0], vec![]),
        fz(1, 18, &[4, 0], vec![30.259984384557136], vec![]),
    ]
}

// ---- F5b round 3: R45 ------------------------------------------------------

/// A 5 V / 0.5 A bench PSU (CV) with a 74LS00 on the rail (dev 1) whose 1y
/// drives an LED through 330 ohm, so the probe's re-solve of the gate's
/// partition iterates (an LED is nonlinear).
fn psu_with_a_gate_driving_an_led() -> Program {
    let devs = vec![psu(0, 1, 0, 5.0, 0.5), ls00(1, 1, 0, [0, 0, 2, 1, 1, 3, 1, 1, 4, 1, 1, 5], 10e-9, 0), resistor(2, 2, 6, 330.0), led(3, 6, 0)];
    no_cache(program(7, devs))
}

/// Runs the hold procedure on the loaded board for its gate (dev 1) as the
/// rail loop would after it alternated, with nothing starving: returns its
/// result, the voltages before and after, and the rig.
fn hold_on_a_self_carrying_gate(p: &Program) -> (Result<(), String>, Vec<f64>, Rig) {
    let mut rig = Rig::load(p).unwrap();
    let before = rig.k.solver().voltages().to_vec();
    let drivers = Rc::clone(&rig.s().driver_snapshot);
    let now = rig.k.time();
    let mut readers = Vec::new();
    let result = rig.k.solver_mut().hold_on_rail(now, &drivers, &mut readers, &[1], rails::loop_unsettled());
    (result, before, rig)
}

#[test]
fn a_probe_every_alternating_gate_carries_itself_in_is_accepted_when_its_state_passes_the_residual_check() {
    // Ruling R45 (a), the accepting half. The gate carries itself in the
    // probe (nothing starves) and every rail IC's free verdict is the one
    // it was probed at: the probe's state is the instant's solution, and it
    // is accepted only after the ordinary convergence check passes on it,
    // the partitions re-solved from it with the holds as they were moving
    // by less than Newton's tolerance. No hold is taken and nothing moves.
    let (result, before, rig) = hold_on_a_self_carrying_gate(&psu_with_a_gate_driving_an_led());
    assert_eq!(result, Ok(()), "accepted");
    assert!(rig.s().power_hold(1).is_free(), "the gate is not held");
    assert_eq!(rig.s().hold_counts(), [0.0, 0.0]);
    for (n, (&got, &want)) in rig.k.solver().voltages().iter().zip(&before).enumerate() {
        close(got, want, 1e-4 + 1e-3 * want.abs(), &format!("node {n} after the accepted probe"));
    }
    assert!(rig.v(6) > 1.5 && rig.v(6) < 2.5, "the LED conducts: {} V", rig.v(6));
}

#[test]
fn a_probe_whose_state_fails_the_residual_check_is_not_accepted() {
    // Ruling R45 (a), the rejecting half: the same board, with the check's
    // re-solve forced unconverged (`hold::FailProbeResidual`, the way the
    // seeded faults of `newton::hooks` are). The procedure fails as before
    // the acceptance existed (`-4`, the loop's own message), not with a
    // state nothing checked.
    let _fault = hold::FailProbeResidual::arm();
    let (result, _, _) = hold_on_a_self_carrying_gate(&psu_with_a_gate_driving_an_led());
    assert_eq!(result, Err(rails::loop_unsettled()), "rejected");
}

// ---- F5b round 3: R47 ------------------------------------------------------

/// W1gV4's cold fuzz seed 226 (tpl 4, hub 2), as the R42 probe ran it: a
/// `{8.99 V, 91 mA}` PSU in CC with a 0.16 uF rail capacitor (esr 0.14 ohm), a
/// -2.0 to 7.3 V pulse source on node 3, a 0.92 mF capacitor (dev 5) from
/// node 5 to it, 18 and 341 ohm from the rail to node 5, and an LED (dev 9)
/// from the pulse node to the rail, with two more from node 5.
fn fuzz_board_226(exact_mode: bool) -> Program {
    let devices = vec![
        raw(21, 0, &[1, 0], &[8.99034184622614, 0.09111661676821328], &[]),
        raw(24, 1, &[3, 0], &[-1.9527258264000453, 7.308766396980163, 0.01893220528935032, 0.0, 0.0, 0.009305650872797054, 0.19921641172523472], &[8.0]),
        raw(1, 2, &[2, 0], &[122506.80201624568], &[]),
        raw(1, 3, &[4, 0], &[1093.219335695193], &[]),
        raw(1, 4, &[5, 0], &[4380.848127479292], &[]),
        raw(3, 5, &[5, 3], &[0.0009175201504092653, 0.0, 0.0], &[]),
        raw(1, 6, &[0, 3], &[8.202037322025507], &[]),
        raw(30, 7, &[5, 2], &[3.079375955633612, 1.8422305628842202, 0.019754226651359424, 0.0, 0.0, 0.0], &[]),
        raw(1, 8, &[1, 5], &[18.17514619869535], &[]),
        raw(30, 9, &[3, 1], &[2.0789318408039135, 1.7000933281455102, 0.009481792577495464, 0.0, 0.0, 0.0], &[]),
        raw(1, 10, &[3, 3], &[16.73431368060318], &[]),
        raw(3, 11, &[0, 0], &[1.2990481818868812e-5, 0.0, 0.0], &[]),
        raw(3, 12, &[1, 0], &[1.5999164502748763e-7, 0.1412537798088652, 0.0], &[]),
        raw(30, 13, &[5, 3], &[1.931350470642555, 1.860095725987525, 0.01572824136645284, 0.0, 0.0, 0.0], &[]),
        raw(1, 14, &[1, 5], &[341.19836764024944], &[]),
        raw(3, 15, &[5, 0], &[0.0144928188878566, 3.4588400262396184, 0.0], &[]),
    ];
    let mut p = Program { node_count: 6, options: Options::default(), digital_slot_count: 32, channel_slot_count: 0, devices };
    p.options.no_rail_shortcut = exact_mode;
    p
}

#[test]
#[ignore = "R47: seed 226's LED from a pulse node to a CC rail; diagnosis pending (perf round 3)"]
fn seed_226_default_mode_stays_within_w0_of_a_fine_reference() {
    // Ruling R47: not fixed in F5. Default mode is 19 W0 off a fine
    // reference at 5 ms and 720-860 W0 at 27-33 ms (the rail at 5.12 V against
    // 2.78 V at 28.2 ms); exact mode with a horizon of 1 us, the reference,
    // is the board's physics. The suspect is the LED from the pulse node to
    // the CC rail (the pulse's edge reaches the rail through it); the
    // diagnosis is perf round 3's. This pins the target.
    let samples = [0.005, 0.01, 0.0272, 0.0282, 0.0327];
    let mut fine = exact(fuzz_board_226(true));
    fine.options.integration = 1;
    let (want, _, _) = sampled(&fine, &samples, 1e-6);
    let (got, _, _) = sampled(&fuzz_board_226(false), &samples, 0.0);
    let offs: Vec<f64> = (0..samples.len()).map(|k| worst_w0(&got[k..=k], &want[k..=k])).collect();
    for (k, &t) in samples.iter().enumerate() {
        eprintln!("seed 226 at {t} s: {:.1} W0 off the fine reference; rail {:.4} V against {:.4} V", offs[k], got[k][1], want[k][1]);
    }
    for (k, &t) in samples.iter().enumerate() {
        assert!(offs[k] <= 1.0, "seed 226 at {t} s: {:.1} W0 off the fine reference (rail {:.4} V against {:.4} V)", offs[k], got[k][1], want[k][1]);
    }
}

#[test]
fn a_cc_rail_whose_load_opens_leaves_cc_for_the_setpoint() {
    // The cold fuzz's seed 22 tpl 4 (F5D: "rail at 3.1e10 V"; ruling D5 (b),
    // F5f): a {6.32 V, 63 mA} PSU in CC into a 0.1 V pulse source through a
    // 0.26 ohm fuse rated 10.5 mA, the rail's only other load a reverse LED.
    // The fuse trips some 6 ms in; with its load gone the CC rail heads for
    // iLimit over the node shunt, 63 mA / 2e-12 S = 3.1e10 V, where the
    // LED's 3e-16 A rounds a different way at each iterate and the rail
    // Newton, one ulp (3.8e-6 V) apart, cannot meet its 1e-12 V tolerance:
    // the advance declined with `-4`. The supply holds its setpoint there:
    // CV at 6.32 V from the trip on, the rail never past it.
    let p = { let mut p = program(8, cold_22_tpl4()); p.options.integration = 1; p.digital_slot_count = 32; p };
    let v = 6.322361965212574;
    let stops = [0.006272879633087429, 0.013345117839129006, 0.015494695988619472, 0.024315328917644118, 0.05064003930122574, 0.05490012932844035, 0.09847041802644906, 0.11261489541743265];
    let mut e = Engine::load(&p).unwrap();
    for &t in &stops {
        e.advance(t).unwrap_or_else(|er| panic!("advance to {t}: {} at {}: {}", er.code, e.time(), er.message));
        let u = e.voltages()[1];
        assert!(u <= v + 1e-9, "the rail {u} V at {t}, past the setpoint {v}");
    }
    assert_eq!(e.currents()[10], 0.0, "the fuse tripped");
    assert_eq!(e.voltages()[1], v, "CV at the setpoint once the load opened");
}

/// tpl=4 hub=2 e=6.322 psu ilim=0.063 V2=2.81 PULSE3 0.1..8.4 tr=2e-4 R4-5=10291.7 C3-6=8.9e-5 LED0-1 F1-3=0.26/0.01 R6-2=20.1 LED5-4 SW5-3(1)+R3-0=17 exact=false span=8.4
/// options: Options { integration: 1, .. } digital_slot_count 32 node_count 8
/// toggles [(0.42852457705507285, 13), (0.7194396106642458, 13), (1.0510772579537433, 13)]
/// ERR advance to 0.013345117839129006 at t 6.272879633087429e-3: rail: the rail Newton did not converge within 50 iterations
fn cold_22_tpl4() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![6.322361965212574, 0.06299239375166817], vec![]),
        fz(2, 1, &[2, 0], vec![2.8141896380534313], vec![]),
        fz(24, 2, &[3, 0], vec![0.10461017875902145, 8.392659606829216, 0.012172834348593155, 0.00019498313165868688, 0.00019498313165868688, 0.027646839705626, 0.0684597476858559], vec![8.0]),
        fz(1, 3, &[4, 0], vec![136212.16444055017], vec![]),
        fz(1, 4, &[5, 0], vec![3433.651644971659], vec![]),
        fz(1, 5, &[6, 0], vec![6515.598989611513], vec![]),
        fz(1, 6, &[7, 0], vec![335473.78092745814], vec![]),
        fz(1, 7, &[4, 5], vec![10291.662113429844], vec![]),
        fz(3, 8, &[3, 6], vec![8.898602240469258e-5, 0.0, 0.0], vec![]),
        fz(30, 9, &[0, 1], vec![1.9728609744022425, 2.4020290663757944, 0.024963565203299995, 0.0, 0.0, 0.0], vec![]),
        fz(10, 10, &[1, 3], vec![0.2621076353578133, 0.010493551323249091], vec![]),
        fz(1, 11, &[6, 2], vec![20.062657159606093], vec![]),
        fz(30, 12, &[5, 4], vec![2.1676349986528507, 2.3452118466706255, 0.01761062464588532, 0.0, 0.0, 0.0], vec![]),
        fz(4, 13, &[5, 3], vec![1.0], vec![]),
        fz(1, 14, &[3, 0], vec![17.021970939781387], vec![]),
        fz(90, 7, &[], vec![7.0, 1.4460096379689398e-5, 10291.662113429844], vec![]),
        fz(90, 11, &[], vec![11.0, 0.942324895972889, 20.062657159606093], vec![]),
        fz(90, 14, &[], vec![14.0, 0.011706195361967153, 17.021970939781387], vec![]),
    ]
}

#[test]
fn a_555_with_no_consistent_latch_on_a_cc_rail_declines_with_minus_7_in_both_modes() {
    // The cold fuzz's seed 1356 tpl 6 (F5e; ruling R49 (d)): a 555 astable
    // on a {13.5 V, 43 mA} PSU whose output load (47 ohm and an LED) drags
    // the CC rail to 5.1 V while OUT is high. The capacitor at 3.40 V sits
    // in the band where no latch state is consistent: high, THR passes 2/3
    // of the dragged rail and resets it; low, the rail returns to CV at
    // 13.5 V and TRIG is under a third of that, which sets it. Default mode
    // reports the zero-delay loop as `-7` (NE555 point 11). Exact mode's
    // instant reset and set the latch and then held it high on a rail its
    // re-solve put 5.5e-5 V above the step's, within W0, so each step
    // re-planned the reset at its start and NP2's guard declined the crawl
    // with `-4`. The latch's own evidence is the same in both modes: it
    // returned, within one instant, to the value it opened the instant with.
    for exact in [false, true] {
        let p = {
            let mut p = program(7, cold_1356_tpl6());
            p.options.integration = 1;
            p.options.no_rail_shortcut = exact;
            p.digital_slot_count = 32;
            p
        };
        let mut e = Engine::load(&p).unwrap();
        let er = e.advance(0.006272876838681287).unwrap_err();
        assert_eq!(er.code, -7, "exact {exact}: {} at {}: {}", er.code, e.time(), er.message);
        // The instant in the band, declined whole: the engine stays at the
        // last one it completed.
        let at: f64 = er.message.rsplit("t = ").next().unwrap().parse().unwrap();
        assert!((0.003535..0.003539).contains(&at) && e.time() < at, "exact {exact}: declined at {at}, left at {}", e.time());
    }
}

/// tpl=6 hub=2 e=13.524 psu ilim=0.043 555-astable R1=5162 R2=15471 C=1.6e-7 Rl=47 exact=true span=13.52
/// options: Options { integration: 1, no_rail_shortcut: true, .. } digital_slot_count 32 node_count 7
/// ERR (27ea36c) exact advance -4 at 3.535969 ms: the rail loop did not settle within 50 passes (NP2);
/// default advance -7 at 3.538572 ms
fn cold_1356_tpl6() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![13.52366996898624, 0.043183751760289955], vec![]),
        fz(40, 1, &[1, 0, 3, 4, 1, 6, 3, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 2, &[1, 2], vec![5161.817727538946], vec![]),
        fz(1, 3, &[2, 3], vec![15471.35201144846], vec![]),
        fz(3, 4, &[3, 0], vec![1.5590167310697523e-7, 0.0, 0.0], vec![]),
        fz(3, 5, &[6, 0], vec![1e-8, 0.0, 0.0], vec![]),
        fz(1, 6, &[4, 5], vec![47.433374949121045], vec![]),
        fz(30, 7, &[5, 0], vec![2.42780965809063, 2.098288374276283, 0.026192850590442324, 0.0, 0.0, 0.0], vec![]),
        fz(1, 8, &[4, 0], vec![2704.4036600522195], vec![]),
        fz(90, 2, &[], vec![2.0, 0.0024330930088748894, 5161.817727538946], vec![]),
        fz(90, 3, &[], vec![3.0, 0.0002900081775833328, 15471.35201144846], vec![]),
        fz(90, 6, &[], vec![6.0, 0.6260429703218926, 47.433374949121045], vec![]),
        fz(90, 8, &[], vec![8.0, 0.010885009222683432, 2704.4036600522195], vec![]),
    ]
}

// ---- F5h: the CC hub seed's draw at the power its solve held ----------

#[test]
fn a_cc_hub_seed_publishes_its_draw_at_the_power_its_solve_held() {
    // G2bv's bc_ic seed 178 hub 2 (F5V finding 1), stepped: an {8.29 V,
    // 66 mA} PSU in CC with no rail capacitor and a 74LS00 on the rail whose
    // output sinks the LED string 33 ohm off it. Powered, the gate pulls the
    // rail to 1.69 V; unpowered, the string alone sits at 4.94 V: no
    // consistent state, which the hold settles (F5b, H8). The seed solved
    // the rail with the gate's power judged at the loop's rail but
    // published its draw judged at its own (unpowered: 22.6 mA against the
    // 66 mA limit), so the loop's rail never moved, no verdict flipped, and
    // the partition re-seeded to the same 1.69 V for 50 passes (-4 at
    // 0.13 ms from 7279cb9; one hold and the sweep's 3 s at c213495).
    let mut p = program(8, g2bv_bc_ic_178_hub2());
    p.options.force_stepped = true;
    p.digital_slot_count = 16;
    let (rig, err) = run_stops(&p, &[(2.643426447688165, 15)], &STOPS_178);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(rig.k.time(), 3.0);
    assert!((0.0..=8.293754669060064).contains(&rig.u()), "the rail at {} V", rig.u());
    assert_eq!(rig.s().hold_counts()[0], 1.0, "one hold");
}

#[test]
fn a_cc_hub_seed_near_a_power_edge_lets_the_rail_move() {
    // G2bv's bc_ic seed 6 hub 2 (F5V finding 1): a {4.91 V, 59 mA} PSU in CC
    // with no rail capacitor, a 74LS00 powered from the rail with its inputs
    // tied to it. At 18 ms its seed solved the rail to 4.27668 V with the
    // gate judged at the loop's 4.24114 V and published 99.9 mA there,
    // judged at 4.27668 V: the loop solved 4.24114 V again for 50 passes.
    let mut p = program(5, g2bv_bc_ic_6_hub2());
    p.digital_slot_count = 16;
    let (rig, err) = run_stops(&p, &[], &STOPS_6);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(rig.k.time(), 3.0);
    assert!((0.0..=4.905614234877831).contains(&rig.u()), "the rail at {} V", rig.u());
}

const STOPS_178: [f64; 74] = [0.00010054130234204333, 0.000516264847582394, 0.090392234498523, 0.10668152744895201, 0.1450627147874467, 0.17031197592202071, 0.17487762727093611, 0.18100154969582236, 0.18133492723077077, 0.18961575573563005, 0.26655087427727586, 0.271356026034515, 0.3098144029736797, 0.3100926534416213, 0.3118951252525391, 0.3126693272898205, 0.31300001061894384, 0.33965524080934884, 0.6047560175957156, 0.6053731992925906, 0.7208836966686086, 0.7233672092747313, 0.7238416992579619, 0.7272574889870183, 0.7717136517000949, 0.7847125562472044, 0.7876048983101962, 0.7914382392707884, 0.7949980436001027, 0.7951112011525082, 0.7952935975653151, 0.7957167440397214, 0.8029012507415543, 0.8128232474663165, 0.8227787421276438, 0.8284612296881646, 1.1161665578971245, 1.1276859838043451, 1.1303183527993368, 1.4067649397950517, 1.6639558115985498, 1.9853725519132555, 1.9859905033744367, 1.9877668027388968, 2.0592386513260297, 2.059614559586607, 2.189381065646793, 2.2383397596650805, 2.270497895314913, 2.2771481284224495, 2.2848750074307214, 2.2859014613830086, 2.2898392698426884, 2.356843622025454, 2.3581089361890184, 2.3586116010897786, 2.4721766107323346, 2.558743261402498, 2.5616893646386854, 2.593218194224725, 2.643426447688165, 2.6592427096336317, 2.6692198039002615, 2.669488027412533, 2.6722061669906, 2.6883727914208, 2.6889349558050353, 2.764236043607216, 2.7644337419723892, 2.838130425375827, 2.838426897400348, 2.8541546102818667, 2.854339967757926, 3.0];
const STOPS_6: [f64; 80] = [0.00010054129832127842, 0.003392132757056072, 0.006788740164450711, 0.00854465704586226, 0.008709500832538523, 0.3507977993362406, 0.35092328174099646, 0.4510841802059878, 0.6170622835297674, 0.6228982173315674, 0.6232122768408935, 0.6287424899512699, 0.8042809183612456, 0.8077411661701315, 0.8081694339749722, 0.810368703446426, 0.811385888416755, 0.8178183706108872, 0.8180676685672675, 0.8780219805189872, 1.106919722255374, 1.1359583339654762, 1.1372721984348433, 1.1415014644874224, 1.141842789883125, 1.203483146770121, 1.4241840750060364, 1.425419401877855, 1.4267422428386898, 1.4304815063925236, 1.4558367939749415, 1.4567347063256268, 1.5628612215397333, 1.563095940680276, 1.5765263721638176, 1.5774862317649447, 1.5781861084818407, 1.5803031754350219, 1.6041259747201573, 1.615436987601587, 1.615911286623937, 1.6499486081840478, 1.6500524980785685, 1.6541688898013174, 1.6741133650174866, 1.6742398813932395, 1.720992318223476, 1.8088209720826525, 1.8090682748245441, 1.810272168235282, 1.8418713633351362, 1.8585309724135661, 1.9608433681132695, 1.962281555423242, 1.9679180310751199, 1.968071036580936, 1.9685372268226469, 2.0752704076721513, 2.098370379653878, 2.1031962092366334, 2.1095539984924683, 2.2149951503693064, 2.2667091537261346, 2.3098323097361027, 2.316263569901875, 2.387499502320899, 2.400568143190981, 2.4566439782967064, 2.4581069250216356, 2.4741024076334934, 2.479194046834902, 2.739927000219146, 2.7411325077401134, 2.7463136053933392, 2.7468314816801134, 2.9434183893614647, 2.9557104045112492, 2.9608141999024578, 2.9612520889299088, 3.0];

/// seed 178 hub=2 e=8.294 ilim=0.066 C0-6=9.25e-6 R6-0=5.8 LED4-5+R1-4=33 R5-6=11.1 IC SW5-4(0)+R4-0=42 stepped
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: true, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 16 node_count 8
/// toggles [(2.643426447688165, 15)] resets []
fn g2bv_bc_ic_178_hub2() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![8.293754669060064, 0.06608919005373863], vec![]),
        fz(1, 1, &[2, 0], vec![57812.62880621811], vec![]),
        fz(1, 2, &[3, 0], vec![34246.38463068881], vec![]),
        fz(1, 3, &[4, 0], vec![150802.9034871621], vec![]),
        fz(1, 4, &[5, 0], vec![13911.656587445708], vec![]),
        fz(1, 5, &[6, 0], vec![53929.6050900096], vec![]),
        fz(1, 6, &[7, 0], vec![2572.673525685172], vec![]),
        fz(3, 7, &[0, 6], vec![9.24922082734711e-6, 0.0, 0.0], vec![]),
        fz(1, 8, &[6, 0], vec![5.793474683060875], vec![]),
        fz(30, 9, &[4, 5], vec![1.841393306055568, 1.5655410823934672, 0.013379888504367273, 0.0, 0.0, 0.0], vec![]),
        fz(1, 10, &[1, 4], vec![33.04229359906569], vec![]),
        fz(1, 11, &[5, 6], vec![11.070825524435474], vec![]),
        fz(50, 12, &[1, 0, 7, 4, 5, 0, 0, 2, 0, 0, 2, 0, 0, 5], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.0778205649508968e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 13, &[5, 0], vec![318.0888736429107], vec![]),
        fz(1, 14, &[2, 1], vec![2480.7686937008857], vec![]),
        fz(4, 15, &[5, 4], vec![0.0], vec![]),
        fz(1, 16, &[4, 0], vec![41.74305715308631], vec![]),
    ]
}
/// seed 6 hub=2 e=4.906 ilim=0.059 C2-0=6.95e-4 IC C4-0=2.80e-4 LED1-2+R1-1=51 R0-4=7.3 LED4-3 R0-3=5.4
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 16 node_count 5
/// toggles [] resets []
fn g2bv_bc_ic_6_hub2() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![4.905614234877831, 0.05861666615693717], vec![]),
        fz(1, 1, &[2, 0], vec![4646.264747857403], vec![]),
        fz(1, 2, &[3, 0], vec![11903.502963889012], vec![]),
        fz(1, 3, &[4, 0], vec![31004.127870970446], vec![]),
        fz(3, 4, &[2, 0], vec![0.0006953588849245298, 0.0, 0.0], vec![]),
        fz(50, 5, &[1, 0, 1, 1, 4, 0, 0, 2, 0, 0, 2, 0, 0, 4], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.6925803248793534e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 6, &[4, 0], vec![2636.8498618366584], vec![]),
        fz(1, 7, &[2, 1], vec![228.73328259782895], vec![]),
        fz(3, 8, &[4, 0], vec![0.00028001110667238787, 0.0, 0.0], vec![]),
        fz(30, 9, &[1, 2], vec![2.715532701744019, 1.5725643222720707, 0.01590001739478545, 0.0, 0.0, 0.0], vec![]),
        fz(1, 10, &[1, 1], vec![50.87434601771005], vec![]),
        fz(1, 11, &[0, 4], vec![7.318227303508357], vec![]),
        fz(30, 12, &[4, 3], vec![2.336609950310507, 1.6757798373338635, 0.028482310223686195, 0.0, 0.0, 0.0], vec![]),
        fz(1, 13, &[0, 3], vec![5.432088525361593], vec![]),
    ]
}

#[test]
fn a_cc_hub_seed_with_a_capacitor_path_off_the_rail_publishes_the_draw_its_rail_row_holds() {
    // The cold fuzz's seed 603 tpl 3 (F5V finding 1), stepped: a {4.14 V,
    // 1.21 A} PSU in CC with no rail capacitor, 1.33 mF from the rail to
    // node 4 and 5.5 uF from node 4 to ground, so the cluster has no charge
    // balance (`rail_capacitors`) and the seed's draw is read through 1e9 S
    // of `C / SEED_H`. At the 0.2523 s toggle the seed's rail row held its
    // draw at 0.9065994 A at its rail; read from the stamp, a rounding of the
    // node's ulp, it published 5.2e-7 A more, the loop put the rail 1.4 uV
    // under the seed's (past the 1 uV band) and the seed re-solved to the
    // same rail for 50 passes (-4 from 7279cb9, none at c213495).
    let p = { let mut p = program(6, cold_603_tpl3()); p.options.force_stepped = true; p.digital_slot_count = 32; p };
    let stops: [f64; 63] = [0.006272880961604103, 0.008508452821013713, 0.011217868098216567, 0.0125412255119006, 0.013444238725033149, 0.03702659377668277, 0.05788268974900449, 0.05870385714073755, 0.09210910261960445, 0.09226560649009774, 0.10359510748129593, 0.11034919348846739, 0.1105179261206954, 0.11527576650868628, 0.11819911599505084, 0.1304572290561621, 0.13144035258059136, 0.13180694787231517, 0.13303907099276227, 0.14723247478286433, 0.15369111747628245, 0.156341829952502, 0.15689051388253816, 0.15701396550600577, 0.15803408049462367, 0.18140949366651932, 0.181901264883631, 0.18342461596947754, 0.20399654490380195, 0.20862953070545406, 0.218149491720171, 0.21832905219164528, 0.22135884978623582, 0.22239756226820792, 0.23010464913939946, 0.23058510236805418, 0.23137867253636427, 0.23733657071343633, 0.23772411488837686, 0.2388932556660436, 0.23902816066930138, 0.2523268991706088, 0.25916652000082796, 0.2600784077874062, 0.26474678021055736, 0.3038126908855331, 0.3506750727726682, 0.351358571990953, 0.35161259987140514, 0.35177342991127986, 0.35229411167494656, 0.3524700798521884, 0.3547458528603408, 0.35513501530943636, 0.35580932234804735, 0.3561867751952288, 0.3595227308713596, 0.3614402252828668, 0.36491583324673255, 0.371935746269125, 0.3733334230677414, 0.3744830143200952, 0.3756964450308541];
    let (rig, err) = run_stops(&p, &[(0.2523268991706088, 11), (0.3524700798521884, 13), (0.371935746269125, 11)], &stops);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(rig.k.time(), 0.3756964450308541);
    assert!((0.0..=4.137436982877272).contains(&rig.u()), "the rail at {} V", rig.u());
}

/// tpl=3 hub=2 e=4.137 psu ilim=1.211 SW4-1(0)+R1-0=11 R4-0=19053.4 C4-1=1.3e-3 LED4-2+R1-4=2759 SW1-0(1)+R0-0=3503 SW3-4(1)+R4-0=61 C4-0=5.5e-6 C0-0=1.1e-2 span=4.137436982877272
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: true, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 6
/// toggles [(0.2523268991706088, 11), (0.3524700798521884, 13), (0.371935746269125, 11), (0.5488892808666742, 5), (0.8262538818018857, 13), (1.1118390257918258, 11)]
fn cold_603_tpl3() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![4.137436982877272, 1.2110731156833958], vec![]),
        fz(1, 1, &[2, 0], vec![58127.37598100101], vec![]),
        fz(1, 2, &[3, 0], vec![21716.09147703718], vec![]),
        fz(1, 3, &[4, 0], vec![63045.63553141906], vec![]),
        fz(1, 4, &[5, 0], vec![2522.9032194273514], vec![]),
        fz(4, 5, &[4, 1], vec![0.0], vec![]),
        fz(1, 6, &[1, 0], vec![10.590442680438601], vec![]),
        fz(1, 7, &[4, 0], vec![19053.3768383018], vec![]),
        fz(3, 8, &[4, 1], vec![0.0013341401938250428, 0.0, 0.0], vec![]),
        fz(30, 9, &[4, 2], vec![2.720500881246087, 2.0089022365125486, 0.03859502919794719, 0.0, 0.0, 0.0], vec![]),
        fz(1, 10, &[1, 4], vec![2759.0451643389533], vec![]),
        fz(4, 11, &[1, 0], vec![1.0], vec![]),
        fz(1, 12, &[0, 0], vec![3502.567887094168], vec![]),
        fz(4, 13, &[3, 4], vec![1.0], vec![]),
        fz(1, 14, &[4, 0], vec![60.55583149229431], vec![]),
        fz(3, 15, &[4, 0], vec![5.542224222197012e-6, 3.5568806717747345, 0.0], vec![]),
        fz(3, 16, &[0, 0], vec![0.011324558313765654, 0.0, 0.0], vec![]),
        fz(90, 6, &[], vec![6.0, 0.5990363889635428, 10.590442680438601], vec![]),
        fz(90, 7, &[], vec![7.0, 0.00011291462134938728, 19053.3768383018], vec![]),
        fz(90, 10, &[], vec![10.0, 0.005916066248696526, 2759.0451643389533], vec![]),
        fz(90, 12, &[], vec![12.0, 2.010485616448936e-5, 3502.567887094168], vec![]),
        fz(90, 14, &[], vec![14.0, 0.009383917249005861, 60.55583149229431], vec![]),
    ]
}

#[test]
fn a_cc_exit_ties_a_capacitor_with_no_esr_at_the_setpoint() {
    // The cold fuzz's seed 654 tpl 5 (F5V, F5h): a {5.00 V, 1.12 A} PSU with
    // 12 uF straight across the rail (no esr) beside 4.6 mF through 0.27 ohm.
    // At a CC exit the setpoint was within RAIL_MOVE_EPS of the CC rail, so
    // the rail node was never published at it; the exit's tie then set the
    // esr-free capacitor to that stale node difference, below the setpoint,
    // and the CV->CC test read the gap as unbounded demand: the next instants
    // crawled one ulp apart between CC and CV until the rail loop's cap (-4
    // at 1.19933 s). A capacitor with no esr is tied where the setpoint puts
    // it, as the rail is.
    let p = { let mut p = program(6, cold_654_tpl5()); p.options.integration = 1; p.digital_slot_count = 32; p };
    let stops: [f64; 148] = [0.006272881060637559, 0.006591874094186496, 0.006738369798626037, 0.01140702575290447, 0.01426541570164304, 0.014494664955489392, 0.015845785419983334, 0.01604297253667917, 0.016438762835408082, 0.01694575147421005, 0.022649428698003057, 0.02878348869319501, 0.029723295666523927, 0.03362152774810005, 0.056120405102267845, 0.056229037375842415, 0.0725332780157821, 0.09744006763523005, 0.10000702248153023, 0.10247727226228409, 0.11400881604452087, 0.11420789527825603, 0.11456675316695472, 0.132036707462793, 0.13215333428541956, 0.13314856946508996, 0.1735960941288972, 0.17880237000823784, 0.20233255736292138, 0.20273060554438915, 0.20650303005521584, 0.210097432859445, 0.21125785297540947, 0.21194824409116625, 0.21236037913567732, 0.21257167696074925, 0.2143008803746772, 0.21515195493065617, 0.238197387178014, 0.2417489640715959, 0.24263016337478488, 0.27519041846173053, 0.27583175012136746, 0.3041944321362515, 0.3472853045887351, 0.3710382925502926, 0.3771699428146706, 0.37755157586408883, 0.3776649625939152, 0.37805049052767786, 0.37869952414921493, 0.38881824909745627, 0.3921186165874907, 0.3922430325869847, 0.4318378140798058, 0.43313208309999685, 0.43502070768173773, 0.4665922437066058, 0.49696122820564226, 0.4975049592904818, 0.5036637225410135, 0.5100107018458228, 0.5103263405428516, 0.5104448958744789, 0.5322078830317314, 0.5729954376478771, 0.573366169926861, 0.5735434152973795, 0.5744108985185017, 0.5756527434944542, 0.5762016580713385, 0.5775099789975596, 0.5780666419508907, 0.5812528912830027, 0.5813559918570157, 0.6186440865366273, 0.6191726812478239, 0.6621121374575532, 0.662219553428663, 0.6625228128919308, 0.704541274882496, 0.7191587413915297, 0.721870295683001, 0.7249738548269464, 0.7256339553455277, 0.7258714122915849, 0.7423166736650905, 0.7823511441505642, 0.7837148653368945, 0.7838555159946443, 0.7888559199629311, 0.7934599011306834, 0.7935851828807118, 0.8228386668695195, 0.8251574881177077, 0.8263666049622582, 0.8265915033646667, 0.8331405956540121, 0.8364759942627226, 0.8366736377837456, 0.8371085617969939, 0.845293952886047, 0.8454543233568703, 0.8456046153517915, 0.8458809980289612, 0.8461208104394647, 0.8667772615602513, 0.8671529799111006, 0.8986848692310039, 0.8991467326488936, 0.8992668336881126, 0.9003791469536602, 0.9052156567779707, 0.9370791712584148, 0.9382189336827093, 0.9402973673020084, 0.9405623311214611, 0.9466822120630486, 0.9851200944072771, 0.985380388540387, 0.9860332004496556, 0.9916966594641506, 1.00032869057813, 1.0016847447306398, 1.0047930604974566, 1.0049483954954213, 1.0052015709959516, 1.0069855087633708, 1.051445293858752, 1.0518333304858911, 1.0523505618884832, 1.053516825650964, 1.1017904675290167, 1.1024345712071053, 1.1124746686117384, 1.1131820655242126, 1.1132921477607274, 1.113597025878502, 1.1137810135798154, 1.116083225369127, 1.1185253280794436, 1.1190815339648412, 1.122016569423452, 1.161203305333806, 1.1875671808278914, 1.1880933144367918, 1.1883124925563644, 1.2];
    let (rig, err) = run_stops(&p, &[(0.4665922437066058, 10), (1.1875671808278914, 10)], &stops);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(rig.k.time(), 1.2);
}

/// tpl=5 hub=2 e=5.003 psu ilim=1.122 PULSE3 2.7..11.0 tr=0e0 74LS00 vcc1 gnd0 in5,4 out3,4 C1-5=4.6e-3 C0-1=6.1e-6 SW3-5(1)+R5-0=2249 R1-5=2391.0 R2-4=233.2 C5-4=1.4e-6 C0-3=5.3e-3 C1-0=1.2e-5 span=11.043285499156779
/// options: Options { integration: 1, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 6
/// toggles [(0.4665922437066058, 10), (1.1875671808278914, 10)]
fn cold_654_tpl5() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![5.0030280621067735, 1.1215150954150948], vec![]),
        fz(24, 1, &[3, 0], vec![2.687812442540313, 11.043285499156779, 0.04914266660941855, 0.0, 0.0, 0.0066869289288260594, 0.08215597281589736], vec![8.0]),
        fz(1, 2, &[2, 0], vec![377705.1222457586], vec![]),
        fz(1, 3, &[4, 0], vec![108375.60843446481], vec![]),
        fz(1, 4, &[5, 0], vec![600595.7940900992], vec![]),
        fz(50, 5, &[1, 0, 5, 4, 3, 0, 0, 4, 0, 0, 4, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 6.670854528022206e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 6, &[3, 4], vec![571.8550058333984], vec![]),
        fz(1, 7, &[4, 1], vec![2539.2299008753234], vec![]),
        fz(3, 8, &[1, 5], vec![0.004618898311568497, 0.2747588372699416, 0.0], vec![]),
        fz(3, 9, &[0, 1], vec![6.113218162272266e-6, 0.0, 0.0], vec![]),
        fz(4, 10, &[3, 5], vec![1.0], vec![]),
        fz(1, 11, &[5, 0], vec![2249.1060683524665], vec![]),
        fz(1, 12, &[1, 5], vec![2391.020671754964], vec![]),
        fz(1, 13, &[2, 4], vec![233.23941796291263], vec![]),
        fz(1, 14, &[3, 2], vec![27936.19868708654], vec![]),
        fz(3, 15, &[5, 4], vec![1.422830139860396e-6, 0.0, 0.0], vec![]),
        fz(3, 16, &[0, 3], vec![0.005261793936319887, 0.0, 0.0], vec![]),
        fz(1, 17, &[1, 2], vec![8173.766803621111], vec![]),
        fz(3, 18, &[1, 0], vec![1.204128509750762e-5, 0.0, 0.0], vec![]),
        fz(90, 6, &[], vec![6.0, 0.002568762397785921, 571.8550058333984], vec![]),
        fz(90, 7, &[], vec![7.0, 0.0001526007875003483, 2539.2299008753234], vec![]),
        fz(90, 11, &[], vec![11.0, 0.0030463309179397276, 2249.1060683524665], vec![]),
        fz(90, 12, &[], vec![12.0, 0.01343277287240647, 2391.020671754964], vec![]),
        fz(90, 13, &[], vec![13.0, 0.008476157452195739, 233.23941796291263], vec![]),
    ]
}

// ---- F5i: a hold taken or released in a failing instant ------------------

/// Advances `rig` event by event up to `until`, stopping at the first
/// instant where IC `d`'s hold changes: that instant's time, or `None`.
fn advance_to_hold_change(rig: &mut Rig, until: f64, d: usize) -> Option<f64> {
    let from = rig.s().power_hold(d);
    while rig.k.time() < until {
        let t = rig.k.next_event_time().min(until);
        rig.advance(t).unwrap_or_else(|e| panic!("{e:?} at {}", rig.k.time()));
        if rig.s().power_hold(d) != from {
            return Some(t);
        }
    }
    None
}

/// Rule 8's journal for the hold table (W1c point 1a; Rails point 8): the
/// instant where IC `d` of `p` first takes or drops a hold after its load
/// (found event by event up to `until`) is failed at each of its nonlinear
/// solves in turn (`newton::hooks`), from the first on. Every failure
/// leaves `d`'s hold as it was before the instant, and the instant run
/// again changes it as an unfailed run does. Returns how many of the
/// instant's solves failed it.
fn a_failed_hold_instant_rolls_back(p: &Program, until: f64, d: usize) -> u32 {
    let mut rig = Rig::load(p).unwrap();
    let t_change = advance_to_hold_change(&mut rig, until, d).expect("the board changes the hold");
    let changed = rig.s().power_hold(d);
    let mut failed = 0;
    for skip in 0.. {
        let mut rig = Rig::load(p).unwrap();
        while rig.k.next_event_time() < t_change {
            let t = rig.k.next_event_time();
            rig.advance(t).unwrap();
        }
        let (before, hold, episode) = (rig.k.time(), rig.s().power_hold(d), rig.s().episode(d));
        let result = {
            let _fault = newton::hooks::force_unconverged_after(skip, u32::MAX);
            rig.advance(t_change)
        };
        if result.is_ok() {
            break;
        }
        failed += 1;
        assert_eq!(rig.k.time(), before, "the failed instant left no time");
        assert_eq!(rig.s().power_hold(d), hold, "skip {skip}: the failed instant's change undone");
        assert_eq!(rig.s().episode(d), episode, "skip {skip}: the episode (ruling R51's guard) undone with it");
        rig.advance(t_change).unwrap_or_else(|e| panic!("skip {skip}: the instant run again: {e:?}"));
        assert_eq!(rig.s().power_hold(d), changed, "skip {skip}: the instant run again changes the hold as unfailed");
    }
    failed
}

#[test]
fn a_rail_hold_taken_in_an_instant_that_then_fails_is_undone_with_it() {
    // The owner's table (`PartitionedSolver::set_power_hold`), journaled at
    // first touch in the instant's window: cold 237 tpl 3's 555, held off by
    // the rail loop at 47.4 us. A later solve of the same instant failing
    // rolls the hold back with everything else; left in place, the chip
    // would stay held through an instant that never happened.
    let p = { let mut p = program(7, cold_237_tpl3()); p.options.integration = 1; p.digital_slot_count = 32; p };
    let failed = a_failed_hold_instant_rolls_back(&p, 0.07, 11);
    assert!(failed >= 3, "the hold's instant has the probes' solves and more: {failed}");
}

#[test]
fn a_static_release_in_an_instant_that_then_fails_is_undone_with_it() {
    // The static solver's own copy (`StaticPartitionSolver::set_local_hold`),
    // journaled in its window. A 74LS00 fed from a pulse source through 30
    // ohm, 1y high into 60 ohm: at the pulse's 4.5 V its draw takes its
    // supply under the 4.275 V floor and it loads held off; at 5.5 V (1 ms)
    // its iterated partition passes the release and solves it powered
    // (`release_in`). That solve failing undoes the release in the copy as
    // well as in the owner's table: left freed in the copy alone, the run
    // again would solve the gate powered while the owner holds it off.
    let logic = [0, 0, 3, 4, 4, 5, 0, 0, 6, 0, 0, 7];
    let devs = vec![
        pulse(0, 2, 0, [4.5, 5.5, 1e-3, 1e-6, 1e-6, 4e-3, 1e-2], 0),
        source(1, 4, 0, 5.0),
        resistor(2, 2, 1, 30.0),
        ls00(3, 1, 0, logic, 10e-9, 1),
        resistor(4, 3, 0, 60.0),
        resistor(5, 4, 5, 1000.0),
    ];
    let p = program(8, devs);
    assert_eq!(Rig::load(&p).unwrap().s().power_hold(3).kind, digital::power::HoldKind::Off, "held at the load");
    let failed = a_failed_hold_instant_rolls_back(&p, 2e-3, 3);
    assert!(failed >= 1, "{failed}");
}

#[test]
fn a_retry_taken_in_an_instant_that_then_fails_is_armed_again() {
    // F5V2 finding 6: the retry's arming (ruling R51 (b)) is journaled where
    // the retry takes it, in the owner (`take_retry`, the rail) and in the
    // static solver (`release_in`), and rolled back with the instant. F5V's
    // stale-hold boards are released only through it (their release level
    // lies past what the supply reaches unpowered): `s1`'s gate on an
    // iterated static partition, `s1b`'s on a cell's rail. Each solve of
    // the release's instant failed in turn leaves the hold, and the instant
    // run again releases it; an arming left taken by the failed instant
    // would leave the gate dark until the next host step.
    // An LED and 1 k from P to ground give the release's instant (P's ramp
    // end) a Newton solve after the retry's, in a partition of its own,
    // for the failure to land on.
    // Each solve on from one is failed in turn (a failed probe asks nothing, so
    // the instant can pass with the hold kept); every instant that fails
    // must leave the hold and be released when run again.
    for cell in [false, true] {
        let mut p = stale_hold_board(cell);
        p.node_count = 9;
        p.devices.extend([led(6, 4, 8), resistor(7, 8, 0, 1e3)]);
        let mut rig = Rig::load(&p).unwrap();
        let t_change = advance_to_hold_change(&mut rig, 2e-3, 2).expect("released at the pulse's edge");
        let mut failed = 0;
        for skip in 0..40 {
            let mut rig = Rig::load(&p).unwrap();
            while rig.k.next_event_time() < t_change {
                let t = rig.k.next_event_time();
                rig.advance(t).unwrap();
            }
            let result = {
                let _fault = newton::hooks::force_unconverged_after(skip, u32::MAX);
                rig.advance(t_change)
            };
            if result.is_ok() {
                continue;
            }
            failed += 1;
            assert_eq!(rig.s().power_hold(2).kind, digital::power::HoldKind::Off, "cell {cell} skip {skip}: the failed instant left the hold");
            rig.advance(t_change).unwrap_or_else(|e| panic!("cell {cell} skip {skip}: the instant run again: {e:?}"));
            assert!(rig.s().power_hold(2).is_free(), "cell {cell} skip {skip}: the instant run again releases the gate");
        }
        assert!(failed >= 1, "cell {cell}: no solve of the release's instant failed it");
    }
}

#[test]
fn the_dynamic_snapshot_carries_each_ics_episode() {
    // F5V2 finding 6: a rail retry that frees nothing puts the solver back
    // at its checkpoint (`keep_held`), and a chip freed there that at once
    // alternated again had its release counted toward its episode (ruling
    // R54 (a)) before the passes showed it was none. The checkpoint's
    // dynamic snapshot carries the episodes, so that release is undone with
    // the rest.
    let mut rig = Rig::load(&stale_hold_board(true)).unwrap();
    assert_eq!(rig.s().power_hold(2).kind, digital::power::HoldKind::Off, "held at the load");
    let before = rig.s().episode(2);
    let snapshot = rig.s().dynamic_snapshot();
    rig.k.solver_mut().release_episode(2, 1e-9).unwrap();
    assert_ne!(rig.s().episode(2), before, "a release counted");
    rig.k.solver_mut().dynamic_restore(snapshot);
    assert_eq!(rig.s().episode(2), before, "and undone with the snapshot");
}

/// A 4.6 V cell of `r` ohm with two 74LS00s on its rail: dev 1 (floor
/// 4.275 V) with 1y high into 60 ohm, and dev 2, unloaded, its floor raised
/// to 4.41 V (`VCC_MIN` 4.9 V).
fn a_cell_with_a_loaded_gate_and_a_higher_floor(r: f64) -> Program {
    let mut high = ls00(2, 1, 0, [0, 0, 4, 0, 0, 5, 0, 0, 6, 0, 0, 7], 10e-9, 4);
    high.params[ic_param::VCC_MIN] = 4.9;
    let devs = vec![battery(0, 1, 0, 4.6, r), ls00(1, 1, 0, [0, 0, 2, 0, 0, 8, 0, 0, 8, 0, 0, 8], 10e-9, 0), high, resistor(3, 2, 0, 60.0)];
    no_cache(program(9, devs))
}

#[test]
fn a_probe_that_moves_another_rail_ic_across_its_floor_is_not_accepted() {
    // Ruling R45 (a), the consistency half: a probe in which every
    // alternating IC carries itself is the instant's solution only if every
    // other rail IC's free verdict there is the reading it was probed at.
    // Dev 1 held off, the rail sits at the cell's 4.6 V and dev 2 (floor
    // 4.41 V) reads powered. Probed powered, dev 1 carries itself, but its
    // draw takes the rail to about 4.35 V, under dev 2's floor, where dev 2
    // was probed powered: not a solution of the free circuit, so it is
    // rejected (`-4`, the loop's message) whatever the residual re-solve
    // from it would find.
    let p = a_cell_with_a_loaded_gate_and_a_higher_floor(11.0);
    let mut rig = Rig::load(&p).unwrap();
    let drivers = Rc::clone(&rig.s().driver_snapshot);
    let now = rig.k.time();
    let mut readers = Vec::new();
    let held = digital::power::PowerHold { kind: digital::power::HoldKind::Off, release: Some(digital::power::Release { level: 10.0, rising: true, margin: 0.0, since: 0.0 }) };
    let seed: Vec<usize> = rig.k.solver_mut().set_power_hold(1, held).into_iter().map(|p| p.0 as usize).collect();
    assert!(matches!(rig.k.solver_mut().rail_passes(now, &drivers, &mut readers, seed), Ok(hold::Passes::Settled(_))));
    assert!(rig.u() > 4.59 && rig.s().ic_powered(2), "dev 1 held off: the rail {} V, dev 2 powered", rig.u());
    // The rail dev 1 would leave, powered: between the two floors.
    let on = Rig::load(&p).unwrap();
    assert!(on.u() > 4.275 && on.u() < 4.41 && on.s().ic_powered(1) && !on.s().ic_powered(2), "free: the rail {} V, dev 1 powered, dev 2 not", on.u());
    let result = rig.k.solver_mut().hold_on_rail(now, &drivers, &mut readers, &[1], rails::loop_unsettled());
    assert_eq!(result, Err(rails::loop_unsettled()), "rejected");
}

// ---- F5k: ruling R51 (b), the classic-style retry ---------------------------

/// The verifier's stale-hold board (F5V finding 3): a 74LS00 whose 1y
/// (inputs low, so high) drives 60 ohm to a pulse source P, its 2a and 2b
/// on P and its 2y pulled to 5 V through 1 k. P is 0 V to 1 ms, then 5 V
/// for 4 ms (10 ms period). Its supply: `s1` a 5 V source through 100 ohm
/// (an iterated static partition), `s1b` a 4.6 V cell of 20 ohm (the rail).
/// Nodes: 1 vcc, 2 the 5 V source, 3 1y, 4 P, 5 2y.
fn stale_hold_board(cell: bool) -> Program {
    let logic = [0, 0, 3, 4, 4, 5, 0, 0, 6, 0, 0, 7];
    let mut devs = if cell {
        vec![battery(0, 1, 0, 4.6, 20.0), source(1, 2, 0, 5.0)]
    } else {
        vec![source(0, 2, 0, 5.0), resistor(1, 2, 1, 100.0)]
    };
    devs.extend([ls00(2, 1, 0, logic, 10e-9, 0), resistor(3, 3, 4, 60.0), resistor(4, 2, 5, 1000.0), pulse(5, 4, 0, [0.0, 5.0, 1e-3, 1e-6, 1e-6, 4e-3, 1e-2], 4)]);
    program(8, devs)
}

#[test]
fn a_gate_held_at_load_is_retried_and_powers_once_its_load_is_gone() {
    // Ruling R51 (b). At the load 1y's draw into P at 0 V takes the
    // gate's supply under its 4.275 V floor and its absence lets it
    // recover: it loads held off. At 1 ms P rises to 5 V and the draw is
    // gone, but the release level D1 (a) set from the load's drop (the
    // supply held unpowered less held powered, 1.4 V on `s1`) lies past
    // what the supply can ever reach unpowered (the source's 5 V): F5V's
    // stale hold, the gate dark for good. Classic's H8 holds a chip for one
    // host step and retries it powered at the next, so it powers as soon
    // as it can carry itself. Each re-solve retries a held chip the same
    // way. Classic (drafts/f5k-logs/s1r.mts, the board in classic's terms,
    // the cell as its Thevenin pair) at 1.5 and 3 ms: `s1` vcc 5 V, 2y
    // 0.675676 V; `s1b` vcc 4.635577 V, 1y 4.893268 V, 2y 0.632661 V.
    for (cell, vcc, y, y2) in [(false, 5.0, 5.0, 0.675676), (true, 4.635577, 4.893268, 0.632661)] {
        let p = stale_hold_board(cell);
        let mut e = Engine::load(&p).unwrap_or_else(|er| panic!("load: {}", er.message));
        assert_eq!(e.stat(25), 1.0, "cell {cell}: held off at the load");
        for t in [1.5e-3, 3e-3] {
            e.advance(t).unwrap_or_else(|er| panic!("cell {cell}: {} at {}", er.message, e.time()));
            let v = e.voltages();
            eprintln!("cell {cell} t={t}: vcc {} 1y {} P {} 2y {} holds {} released {}", v[1], v[3], v[4], v[5], e.stat(25), e.stat(26));
            w0_volts(v[1], vcc, "vcc");
            w0_volts(v[3], y, "1y");
            w0_volts(v[5], y2, "2y, the gate powered");
        }
        assert_eq!((e.stat(25), e.stat(26)), (1.0, 1.0), "cell {cell}: one hold, released at the edge");
    }
}

// ---- F5k: ruling R51, the hold-episode guard --------------------------------

/// The cold fuzz's seed 552 tpl 2: a {4.83 V, 38 mA} PSU in CC with no rail
/// capacitor, a 74LS00 on the rail (1a, 1b on it, 1y and 4y on node 6, 13.8
/// ohm off the rail with an LED and 4.1 uF to ground).
/// tpl=2 hub=2 e=4.826 psu ilim=0.038 74LS00 vcc1 gnd0 in1,1 out6,4 LED6-2+R1-6=14 R4-6=1662.9 LED5-0 R1-2=49.8 R4-1=51.0 LED5-3+R1-5=734 C6-0=4.1e-6 R4-5=6754.6
fn cold_552_tpl2() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.826259699151881, 0.03804516202394984], vec![]),
        fz(1, 1, &[2, 0], vec![77574.53515198552], vec![]),
        fz(1, 2, &[3, 0], vec![4011.1312235366486], vec![]),
        fz(1, 3, &[4, 0], vec![450538.84788693767], vec![]),
        fz(1, 4, &[5, 0], vec![73300.64670255948], vec![]),
        fz(1, 5, &[6, 0], vec![95939.07719910401], vec![]),
        fz(50, 6, &[1, 0, 1, 1, 6, 0, 0, 4, 0, 0, 4, 0, 0, 6], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 2.074484901825081e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 7, &[6, 6], vec![1429.2778309284438], vec![]),
        fz(1, 8, &[4, 3], vec![1375.492622534435], vec![]),
        fz(30, 9, &[6, 2], vec![1.7958977390178201, 2.0602008419730833, 0.004565304048337132, 0.0, 0.0, 0.0], vec![]),
        fz(1, 10, &[1, 6], vec![13.781715670965971], vec![]),
        fz(1, 11, &[4, 6], vec![1662.8715674848859], vec![]),
        fz(30, 12, &[5, 0], vec![3.176398841094245, 2.055626278765814, 0.026599384648517813, 0.0, 0.0, 0.0], vec![]),
        fz(1, 13, &[1, 2], vec![49.84731026432964], vec![]),
        fz(1, 14, &[4, 1], vec![50.97499049956404], vec![]),
        fz(30, 15, &[5, 3], vec![1.852115415644077, 2.328633659650766, 0.022184204205591848, 0.0, 0.0, 0.0], vec![]),
        fz(1, 16, &[1, 5], vec![733.5129340211519], vec![]),
        fz(3, 17, &[6, 0], vec![4.056872790221788e-6, 0.0, 0.0], vec![]),
        fz(1, 18, &[4, 0], vec![403.57527016552956], vec![]),
        fz(1, 19, &[4, 5], vec![6754.598768588724], vec![]),
        fz(90, 7, &[], vec![7.0, 0.0010593839059236617, 1429.2778309284438], vec![]),
        fz(90, 8, &[], vec![8.0, 0.0015414086696669824, 1375.492622534435], vec![]),
        fz(90, 10, &[], vec![10.0, 0.03149230942183751, 13.781715670965971], vec![]),
        fz(90, 11, &[], vec![11.0, 0.0005093002689384947, 1662.8715674848859], vec![]),
        fz(90, 13, &[], vec![13.0, 0.0663473987017302, 49.84731026432964], vec![]),
        fz(90, 14, &[], vec![14.0, 0.0009810704368777255, 50.97499049956404], vec![]),
        fz(90, 16, &[], vec![16.0, 0.00023034065864443334, 733.5129340211519], vec![]),
        fz(90, 19, &[], vec![19.0, 0.0007796107412845672, 6754.598768588724], vec![]),
    ];
    let mut p = program(7, devices);
    p.options.integration = 1;
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_gate_sliding_along_its_power_edge_declines_after_fifty_short_holds_in_one_episode() {
    // Rulings R51, R54 (a), the permanent safety net. Cold 552 tpl 2's gate
    // slides at its 4.275 V floor: powered, its outputs into node 6 take the
    // CC rail under it; held off, the rail comes back over it, and the
    // retry finds it carrying itself again, 70 ns powered for every 10 ns
    // held. Each hold is H8's, and right, but the chip has no state of its
    // own to settle in: 76,652 holds and releases in the sweep's first 6.3
    // ms, a 16 s advance (a frozen tab). Each hold is far shorter than the
    // 6.3 ms host step and each release the chip's own (no other driver
    // changes), so all count toward one episode, and the 51st release
    // declines `-4`, the board's classic fallback as at c5b1a13.
    let mut e = Engine::load(&cold_552_tpl2()).unwrap_or_else(|er| panic!("load: {}", er.message));
    let err = e.advance(0.00627288085463002).expect_err("declined, not crawled through");
    assert_eq!(err.code, -4, "{}", err.message);
    assert!(err.message.contains("held 51 times in one sliding episode"), "{}", err.message);
    assert!(e.time() < 7e-4, "declined within the first episode, at {} s", e.time());
    assert_eq!((e.stat(25), e.stat(26)), (hold::EPISODE_MAX as f64 + 1.0, hold::EPISODE_MAX as f64), "the 51st hold kept, its release the declined instant");
}

#[test]
fn an_episode_counts_only_short_holds_its_chip_released_itself() {
    // Ruling R54 (a), the guard's own rule (hold::Episode): a hold counts at
    // its release when it lasted less than the host step and no foreign
    // driver change caused the release; a hold of a host step or more, or a
    // foreign-caused release, ends the episode. A retry moving a held
    // chip's level is the same hold, and so is the procedure re-running
    // over a chip it holds.
    let step = 1e-3;
    let e = hold::Episode::default().held(1.0).released(1.0 + 10e-9, step, false);
    assert_eq!(e.count, 1, "10 ns held in a 1 ms host step");
    let e = e.held(1.0 + 1e-6).held(1.5).released(1.0 + 2e-6, step, false);
    assert_eq!(e.count, 2, "held again with no release between: the same hold, from 1 us");
    assert_eq!(e.held(2.0).released(2.0 + 1.5 * step, step, false).count, 0, "held over a host step: classic's cadence, a new episode");
    assert_eq!(e.held(2.0).released(2.0 + 10e-9, step, true).count, 0, "released by a foreign change: a new episode");
    assert_eq!(e.released(3.0, step, false), e, "a release with no hold in place changes nothing");
    let mut e = hold::Episode::default();
    for k in 0..=hold::EPISODE_MAX {
        e = e.held(k as f64).released(k as f64 + 0.5, step * 1e3, false);
        assert_eq!(e.check(3).is_err(), k + 1 > hold::EPISODE_MAX, "{k}");
    }
    // The foreign test is ruling R49 (d)'s: a commit by another device or a
    // waveform edge, never the chip's own.
    let own = hold::Changes::None.note(7);
    assert!(!own.foreign_to(7) && own.foreign_to(3));
    assert!(own.note(7).note(2).foreign_to(7), "a second device's commit");
    assert!(!hold::Changes::None.foreign_to(7));
}

/// F5V2's pulse-fed gate: a 4.5 <-> 5.5 V pulse (1 us edges, 4 ms high
/// from 1 ms, 10 ms period) through 30 ohm to a 74LS00's vcc, its 1y (inputs
/// low, so high) into 60 ohm to ground. Nodes: 1 vcc, 2 the feed, 3 1y,
/// 4 a 5 V source pulling 2y through 1 k at 5.
fn pulse_fed_gate() -> Program {
    let logic = [0, 0, 3, 4, 4, 5, 0, 0, 6, 0, 0, 7];
    program(8, vec![
        pulse(0, 2, 0, [4.5, 5.5, 1e-3, 1e-6, 1e-6, 4e-3, 1e-2], 0),
        source(1, 4, 0, 5.0),
        resistor(2, 2, 1, 30.0),
        ls00(3, 1, 0, logic, 10e-9, 1),
        resistor(4, 3, 0, 60.0),
        resistor(5, 4, 5, 1000.0),
    ])
}

#[test]
fn a_gate_fed_from_a_pulse_is_held_each_low_phase_and_runs() {
    // Ruling R54 (a), F5V2 finding 1. At 4.5 V through 30 ohm the gate's
    // draw takes its supply under 4.275 V: held off each low phase, 6 ms,
    // and freed by the pulse's rising edge, a foreign change, so every
    // phase is an episode of its own. R51's ratio rule chained them (each
    // back within 100 holds' lengths) and declined at the 51st, 0.4955 s.
    // At 0.5 ms host steps each hold is also longer than a host step; at
    // 20 ms only the foreign edge ends the episode.
    for step in [5e-4f64, 2e-2] {
        let mut e = Engine::load(&pulse_fed_gate()).unwrap_or_else(|er| panic!("load: {}", er.message));
        let n = (1.0 / step).round() as usize;
        for k in 1..=n {
            let t = k as f64 * step;
            e.advance(t).unwrap_or_else(|er| panic!("step {step}: {} at {} after {} holds", er.message, e.time(), e.stat(25)));
        }
        // Held at load and at each of the 100 falling edges to 0.995 s, and
        // freed at each of the 100 rising edges.
        assert_eq!((e.stat(25), e.stat(26)), (101.0, 100.0), "step {step}: a hold each low phase, each released");
    }
}

#[test]
fn the_stale_hold_boards_run_a_second_of_pulses() {
    // Ruling R54 (a), F5V2 finding 1 on F5V's stale-hold boards: the gate is
    // held each time P falls to 0 V and freed when it rises, a foreign
    // change, so each hold is an episode of its own; R51's ratio rule
    // declined both at 0.4955 s.
    for cell in [false, true] {
        let mut e = Engine::load(&stale_hold_board(cell)).unwrap_or_else(|er| panic!("load: {}", er.message));
        for k in 1..=2000 {
            let t = k as f64 * 5e-4;
            e.advance(t).unwrap_or_else(|er| panic!("cell {cell}: {} at {}", er.message, e.time()));
        }
        assert_eq!((e.stat(25), e.stat(26)), (101.0, 100.0), "cell {cell}: a hold each low phase, each released");
    }
}

/// Cold 577 tpl 5 (default mode): a {12.2 V, 47 mA} PSU, a 555 on the
/// rail, capacitors from the rail to nodes 2 and 7.
fn cold_577_tpl5() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![12.229934285445824, 0.04701034017161009], vec![]),
        fz(1, 1, &[2, 0], vec![8867.255616775888], vec![]),
        fz(1, 2, &[3, 0], vec![94901.52053035072], vec![]),
        fz(1, 3, &[4, 0], vec![7888.674607743629], vec![]),
        fz(1, 4, &[5, 0], vec![600276.8690632748], vec![]),
        fz(1, 5, &[6, 0], vec![884237.6824883647], vec![]),
        fz(1, 6, &[7, 0], vec![30951.67166504169], vec![]),
        fz(40, 7, &[1, 0, 1, 1, 5, 3, 4, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 8, &[1, 2], vec![240.32709970848802], vec![]),
        fz(1, 9, &[2, 4], vec![1151.491575093083], vec![]),
        fz(3, 10, &[4, 0], vec![5.365225726517978e-7, 0.0, 0.0], vec![]),
        fz(1, 11, &[1, 0], vec![58.27272691211567], vec![]),
        fz(3, 12, &[3, 5], vec![1.229036906705404e-6, 0.0, 0.0], vec![]),
        fz(3, 13, &[4, 6], vec![0.00015552971733438634, 1.345026098002157, 0.0], vec![]),
        fz(4, 14, &[4, 1], vec![1.0], vec![]),
        fz(1, 15, &[1, 0], vec![590.94922733906], vec![]),
        fz(3, 16, &[2, 1], vec![0.013370194859060885, 0.0, 0.0], vec![]),
        fz(3, 17, &[1, 7], vec![0.0008042963679718378, 0.0, 0.0], vec![]),
        fz(1, 18, &[3, 6], vec![7636.565386312634], vec![]),
        fz(3, 19, &[1, 2], vec![6.306011191925995e-7, 0.0, 0.0], vec![]),
        fz(1, 20, &[6, 2], vec![1221.9898456979784], vec![]),
        fz(3, 21, &[7, 6], vec![0.0013550115520314285, 0.4070283063693921, 0.0], vec![]),
        fz(1, 22, &[4, 7], vec![2.6895533828654035], vec![]),
        fz(90, 8, &[], vec![8.0, 0.007496503973939393, 240.32709970848802], vec![]),
        fz(90, 9, &[], vec![9.0, 0.1127859043451141, 1151.491575093083], vec![]),
        fz(90, 11, &[], vec![11.0, 0.5756742027176894, 58.27272691211567], vec![]),
        fz(90, 15, &[], vec![15.0, 0.0006952367891127764, 590.94922733906], vec![]),
        fz(90, 18, &[], vec![18.0, 0.0007437938566566663, 7636.565386312634], vec![]),
        fz(90, 22, &[], vec![22.0, 27.221379455186806, 2.6895533828654035], vec![]),
    ];
    let mut p = program(8, devices);
    p.digital_slot_count = 32;
    p
}

/// Cold 670 tpl 2, forced to default mode: a {4.72 V, 32 mA} PSU, 1.26 mF
/// from the rail to node 5, a 74LS00 on the rail whose 1y and 4y are the
/// rail itself.
fn cold_670_tpl2_default() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.720394607974314, 0.032309252292146054], vec![]),
        fz(1, 1, &[2, 0], vec![88473.26199596563], vec![]),
        fz(1, 2, &[3, 0], vec![34768.670387193124], vec![]),
        fz(1, 3, &[4, 0], vec![10182.76491880738], vec![]),
        fz(1, 4, &[5, 0], vec![126779.54890656004], vec![]),
        fz(1, 5, &[3, 4], vec![4.61151881030905], vec![]),
        fz(3, 6, &[1, 5], vec![0.0012594497438993583, 0.0, 0.0], vec![]),
        fz(3, 7, &[5, 0], vec![1.2837342652815031e-6, 0.2042274790789823, 0.0], vec![]),
        fz(30, 8, &[1, 0], vec![3.106996054717552, 1.7477641650620048, 0.03827393492257751, 0.0, 0.0, 0.0], vec![]),
        fz(1, 9, &[1, 1], vec![595.6878122216177], vec![]),
        fz(50, 10, &[1, 0, 5, 2, 1, 0, 0, 3, 0, 0, 3, 0, 0, 1], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 2.070409865862511e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 11, &[1, 3], vec![308.1867248609299], vec![]),
        fz(1, 12, &[3, 1], vec![1816.2982678747258], vec![]),
        fz(1, 13, &[5, 4], vec![10023.706993073263], vec![]),
        fz(1, 14, &[3, 2], vec![160.20876539445462], vec![]),
        fz(30, 15, &[2, 5], vec![2.702427208103991, 2.2004357739998106, 0.02971469951400689, 0.0, 0.0, 0.0], vec![]),
        fz(1, 16, &[1, 2], vec![174.88927436568574], vec![]),
        fz(1, 17, &[4, 5], vec![4732.936116204729], vec![]),
        fz(1, 18, &[4, 2], vec![20195.27416812104], vec![]),
        fz(90, 5, &[], vec![5.0, 0.3140951715350074, 4.61151881030905], vec![]),
        fz(90, 9, &[], vec![9.0, 0.04520187842907025, 595.6878122216177], vec![]),
        fz(90, 11, &[], vec![11.0, 0.05141360438304322, 308.1867248609299], vec![]),
        fz(90, 12, &[], vec![12.0, 0.007608179221505972, 1816.2982678747258], vec![]),
        fz(90, 16, &[], vec![16.0, 0.010627700053332191, 174.88927436568574], vec![]),
    ];
    let mut p = program(6, devices);
    p.options.integration = 1;
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_chip_chattering_at_its_edge_in_ten_nanosecond_holds_declines_within_its_first_episode() {
    // Ruling R54 (a), F5V2 finding 2. Each of these chips is held 10 ns and
    // runs about 1 us powered before its draw takes its supply out again:
    // R51's ratio rule (back within 100 holds' lengths) saw a new episode
    // at each hold, and the boards took 10,000 to 13,000 holds and 3 to 4.5
    // s to hit the sweep's wall cap. Every hold is far shorter than the host
    // step and released by the chip's own supply, so the 51st release
    // declines `-4`, within the first host step.
    for (name, p, first) in [("577 tpl 5", cold_577_tpl5(), 0.006272880912080727), ("670 tpl 2", cold_670_tpl2_default(), 0.006272881096655422)] {
        let mut e = Engine::load(&p).unwrap_or_else(|er| panic!("{name}: load: {}", er.message));
        let err = e.advance(first).expect_err("declined, not crawled through");
        assert_eq!(err.code, -4, "{name}: {}", err.message);
        assert!(err.message.contains("held 51 times in one sliding episode"), "{name}: {}", err.message);
        // Stat 25 counts holds taken, a probe re-holding a chip already held
        // too; the episode counts each hold once.
        assert!(e.stat(25) <= 2.0 * (hold::EPISODE_MAX as f64 + 1.0), "{name}: declined at the first episode's end, {} holds", e.stat(25));
    }
}

// ---- F5m: HG1 across a hub partition's own steps ----------------------------

/// Cold 301 tpl 2: a {4.70 V, 136 mA} PSU, a 555 on the rail (its window
/// from 1.8 V), the rail switched onto node 2 (2.4 mF from node 3, 1.5 uF
/// to ground), so a stepped hub partition steps with the rail on CC.
fn cold_301_tpl2() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.699251780737138, 0.1360561939952043], vec![]),
        fz(1, 1, &[2, 0], vec![2517.771189205941], vec![]),
        fz(1, 2, &[3, 0], vec![378548.0920676974], vec![]),
        fz(1, 3, &[4, 0], vec![49481.381163273276], vec![]),
        fz(1, 4, &[2, 1], vec![15032.654471028625], vec![]),
        fz(40, 5, &[1, 0, 2, 2, 4, 2, 2, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 6, &[1, 2], vec![3252.1449925274337], vec![]),
        fz(1, 7, &[2, 2], vec![1617.2471390535895], vec![]),
        fz(3, 8, &[2, 0], vec![1.522762840475241e-6, 0.0, 0.0], vec![]),
        fz(1, 9, &[2, 2], vec![288.4053834275334], vec![]),
        fz(1, 10, &[0, 3], vec![339.07432549069006], vec![]),
        fz(30, 11, &[1, 4], vec![3.063767807001338, 2.254233334874512, 0.03165390111805066, 0.0, 0.0, 0.0], vec![]),
        fz(1, 12, &[1, 1], vec![251.55074129441036], vec![]),
        fz(30, 13, &[2, 0], vec![3.197788212742436, 1.8806789593259228, 0.03784577826860894, 0.0, 0.0, 0.0], vec![]),
        fz(3, 14, &[0, 4], vec![0.0033197818229961924, 0.4040618106713327, 0.0], vec![]),
        fz(4, 15, &[1, 2], vec![1.0], vec![]),
        fz(1, 16, &[2, 0], vec![31.855583707434604], vec![]),
        fz(3, 17, &[3, 0], vec![0.0023980142451702035, 0.0, 0.0], vec![]),
        fz(1, 18, &[2, 3], vec![1230.7387074428268], vec![]),
        fz(1, 19, &[1, 2], vec![2934.0465005261735], vec![]),
        fz(1, 20, &[1, 2], vec![18023.107279949378], vec![]),
        fz(90, 4, &[], vec![4.0, 9.549272217678113e-5, 15032.654471028625], vec![]),
        fz(90, 6, &[], vec![6.0, 0.00538895572266153, 3252.1449925274337], vec![]),
        fz(90, 7, &[], vec![7.0, 0.0037770571531698196, 1617.2471390535895], vec![]),
        fz(90, 9, &[], vec![9.0, 0.006887733917138283, 288.4053834275334], vec![]),
        fz(90, 12, &[], vec![12.0, 0.010738374913095827, 251.55074129441036], vec![]),
        fz(90, 16, &[], vec![16.0, 0.05023120368993972, 31.855583707434604], vec![]),
    ];
    let mut p = program(5, devices);
    p.options.integration = 1;
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_chip_flipped_at_each_step_of_the_hub_partition_it_draws_from_is_held() {
    // W1c point 1a across instants (HG1), F5V2 finding 3. The 555 sits at
    // its 1.8 V floor on a CC rail that a stepped hub partition carries:
    // a step that starts with the rail over 1.8 V powers it and ends under
    // (its draw), the next starts unpowered and ends over, at 10 ns steps.
    // Only `Rail` events counted toward HG1, so that ping-pong ran free
    // between the holds the rail loop took: 49 holds and an 11.5 s first
    // advance (6.3 ms), each hold followed by 0.24 s of 10 ns steps. Each
    // hub step now counts as a `Rail` event does; the chip is held at the
    // second flip back, and its 10 ns holds decline within the first
    // episode (ruling R54 (a)), the board's classic fallback as at c5b1a13
    // (-4 at 66 us).
    let mut e = Engine::load(&cold_301_tpl2()).unwrap_or_else(|er| panic!("load: {}", er.message));
    let err = e.advance(0.006272879).expect_err("declined, not stepped through at 10 ns");
    assert_eq!(err.code, -4, "{}", err.message);
    assert!(err.message.contains("held 51 times in one sliding episode"), "{}", err.message);
    assert!(e.time() < 1e-4, "declined within the first episode, at {} s", e.time());
}

// ---- F5m: a retry that frees nothing restores bit for bit -----------------

/// Cold 182 tpl 0: a {4.82 V, 145 mA} PSU, a 555 on the rail held off from 0.17 s on and never
/// released (its switch, device 13, toggles at 0.17 and 0.26 s).
fn cold_182_tpl0() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.82391955261206, 0.14470139061705575], vec![]),
        fz(1, 1, &[2, 0], vec![21341.10759536071], vec![]),
        fz(1, 2, &[3, 0], vec![84910.1743357632], vec![]),
        fz(1, 3, &[4, 0], vec![225501.70521977055], vec![]),
        fz(1, 4, &[5, 0], vec![28806.865083234894], vec![]),
        fz(1, 5, &[1, 2], vec![1.747283555018954], vec![]),
        fz(40, 6, &[1, 0, 5, 5, 5, 3, 0, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 7, &[1, 2], vec![5974.4736388104975], vec![]),
        fz(1, 8, &[2, 0], vec![12904.332650180246], vec![]),
        fz(3, 9, &[0, 0], vec![6.532160081325276e-7, 0.0, 0.0], vec![]),
        fz(1, 10, &[5, 0], vec![61.371320562763586], vec![]),
        fz(1, 11, &[0, 3], vec![103.82029707728188], vec![]),
        fz(1, 12, &[3, 4], vec![67.615697881173], vec![]),
        fz(4, 13, &[2, 3], vec![1.0], vec![]),
        fz(1, 14, &[3, 0], vec![12.194728986041477], vec![]),
        fz(30, 15, &[3, 0], vec![2.7297057141689, 1.683199681331419, 0.007639621518160501, 0.0, 0.0, 0.0], vec![]),
        fz(1, 16, &[1, 3], vec![24.954636662482386], vec![]),
        fz(90, 5, &[], vec![5.0, 0.026636099111645904, 1.747283555018954], vec![]),
        fz(90, 7, &[], vec![7.0, 2.1705399257336346e-5, 5974.4736388104975], vec![]),
        fz(90, 8, &[], vec![8.0, 8.270780028369148e-5, 12904.332650180246], vec![]),
        fz(90, 10, &[], vec![10.0, 0.0009525140276869148, 61.371320562763586], vec![]),
        fz(90, 12, &[], vec![12.0, 0.07335863992493184, 67.615697881173], vec![]),
        fz(90, 14, &[], vec![14.0, 2.68345986476719, 12.194728986041477], vec![]),
        fz(90, 16, &[], vec![16.0, 0.2396639661486624, 24.954636662482386], vec![]),
    ];
    let mut p = program(6, devices);
    p.options.integration = 0;
    p.digital_slot_count = 32;
    p
}

/// Cold 196 tpl 1: a {8.34 V, 132 mA} PSU, a clock, and a 555 whose supply, node
/// 6, hangs 32.7 ohm off a static partition: held off there, never released.
fn cold_196_tpl1() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![8.335302951158798, 0.13150633326105143], vec![]),
        fz(22, 1, &[3, 0], vec![2.6008084627826156, 0.8205630059042611, 10.880874921096378, 0.0, 0.0], vec![8.0]),
        fz(1, 2, &[2, 0], vec![114878.18238656467], vec![]),
        fz(1, 3, &[4, 0], vec![7799.633539194564], vec![]),
        fz(1, 4, &[5, 0], vec![442076.2244467679], vec![]),
        fz(1, 5, &[6, 0], vec![53065.50262844789], vec![]),
        fz(1, 6, &[6, 5], vec![32.68097190528945], vec![]),
        fz(1, 7, &[3, 0], vec![194.00547260509674], vec![]),
        fz(40, 8, &[6, 0, 1, 5, 6, 6, 3, 0], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 9, &[6, 0], vec![105.99001801685802], vec![]),
        fz(1, 10, &[0, 3], vec![25165.620198424596], vec![]),
        fz(3, 11, &[3, 0], vec![3.721079953434243e-8, 0.0, 0.0], vec![]),
        fz(1, 12, &[5, 3], vec![1903.7707655920356], vec![]),
        fz(1, 13, &[0, 5], vec![503.547643512332], vec![]),
        fz(1, 14, &[5, 0], vec![70344.81805455378], vec![]),
        fz(30, 15, &[4, 5], vec![3.230533522708651, 1.759887150160364, 0.012980736768439189, 0.0, 0.0, 0.0], vec![]),
        fz(1, 16, &[1, 4], vec![64.4640751352851], vec![]),
        fz(90, 6, &[], vec![6.0, 0.041306886082873015, 32.68097190528945], vec![]),
        fz(90, 7, &[], vec![7.0, 0.00305128671037237, 194.00547260509674], vec![]),
        fz(90, 9, &[], vec![9.0, 0.02479140635094927, 105.99001801685802], vec![]),
        fz(90, 10, &[], vec![10.0, 0.00042131993818982864, 25165.620198424596], vec![]),
        fz(90, 12, &[], vec![12.0, 0.05096307531833872, 1903.7707655920356], vec![]),
        fz(90, 13, &[], vec![13.0, 0.0023506586257629318, 503.547643512332], vec![]),
        fz(90, 14, &[], vec![14.0, 5.903766542545458e-6, 70344.81805455378], vec![]),
        fz(90, 16, &[], vec![16.0, 0.2788284246193524, 64.4640751352851], vec![]),
    ];
    let mut p = program(7, devices);
    p.options.integration = 1;
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_retry_that_frees_nothing_leaves_the_run_bit_for_bit_as_it_was() {
    // F5V2 finding 5 (ruling R51 (b): the retry only asks). Each board runs
    // twice, with no retry at all and with every retry probing a held chip
    // and freeing nothing; the published voltages must agree to the bit at
    // every host step. The rail's retry put the hub's running sums back
    // re-summed (Rails point 2's recompute after a whole restore, ulps from
    // the incremental ones: 182 tpl 0 off by 1e-13 V from 0.26 s), and the
    // clamp re-solved a static partition holding a held chip only to probe
    // it (196 tpl 1 off by 1e-15 V from the first host step).
    let toggles: &[(f64, usize)] = &[(0.16964862719279233, 13), (0.2595604491700335, 13)];
    for (name, p, stops, toggles) in [("182 tpl 0", cold_182_tpl0(), vec![0.006272879998957345, 0.013318265673895324, 0.013519568466705188, 0.014317009684649685, 0.040378550111770425, 0.07057049561482419, 0.09297134995164474, 0.09315771753629373, 0.09565666438180485, 0.10153405682478835, 0.10202583796940948, 0.10220933460419031, 0.12443019075624726, 0.13250932588946368, 0.1327202125679568, 0.13501474161943483, 0.13578591735295228, 0.13615201829900628, 0.1401459363269185, 0.14910358004921206, 0.1500050127261678, 0.15063784184044718, 0.15162406913993892, 0.16266398231785245, 0.16964862719279233, 0.19049156196271452, 0.20392892161124826, 0.22540840453808006, 0.22591006094867933, 0.2262459710478473, 0.22804201127754886, 0.23074629780535588, 0.23598804876616586, 0.2398857751954551, 0.2595604491700335, 0.3], toggles), ("196 tpl 1", cold_196_tpl1(), vec![0.006272880040702927, 0.009518924590976103, 0.009902844739601756, 0.010256506809549005, 0.010737349902114918, 0.02], &[][..])] {
        let run = |seam: hold::RetrySeam| -> Vec<Vec<u64>> {
            let _seam = seam.arm();
            let mut e = Engine::load(&p).unwrap_or_else(|er| panic!("{name}: load: {}", er.message));
            let mut state = 1.0;
            let mut out = Vec::new();
            for &t in &stops {
                e.advance(t).unwrap_or_else(|er| panic!("{name} {seam:?}: {} at {}", er.message, e.time()));
                for &(at, d) in toggles {
                    if at == t {
                        state = 1.0 - state;
                        e.set_params(&[(d as u32, 0, state)]).unwrap_or_else(|er| panic!("{name}: toggle: {}", er.message));
                    }
                }
                out.push(e.voltages().iter().map(|v| v.to_bits()).collect());
            }
            assert!(e.stat(25) >= 1.0 && e.stat(26) == 0.0, "{name} {seam:?}: a hold never released ({}, {})", e.stat(25), e.stat(26));
            out
        };
        let (none, asks) = (run(hold::RetrySeam::NoRetry), run(hold::RetrySeam::AsksOnly));
        for (k, (a, b)) in none.iter().zip(&asks).enumerate() {
            assert_eq!(a, b, "{name}: the run with retries that free nothing left the run without them at host step {k} (t = {})", stops[k]);
        }
    }
}

// ---- F5l: a CC step whose gate's stamps move with the rail -------------

/// seed 425 hub=2 e=9.579 ilim=0.133 SW4-2(1)+R2-0=41 R2-0=164.9 R3-5=35.0 SW3-2(1)+R2-0=267 IC R2-1=22.6 C5-1=3.68e-2 stepped
/// options: force_stepped, digital_slot_count 16, node_count 6
fn g2bv_bc_ic_425_hub2() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![9.57914262231615, 0.13316460518608836], vec![]),
        fz(1, 1, &[2, 0], vec![42364.783803100094], vec![]),
        fz(1, 2, &[3, 0], vec![55600.195235159736], vec![]),
        fz(1, 3, &[4, 0], vec![5029.3936874087285], vec![]),
        fz(1, 4, &[5, 0], vec![128318.71955948745], vec![]),
        fz(4, 5, &[4, 2], vec![1.0], vec![]),
        fz(1, 6, &[2, 0], vec![41.328596501266986], vec![]),
        fz(1, 7, &[2, 0], vec![164.91981218453046], vec![]),
        fz(1, 8, &[3, 5], vec![34.951871519416585], vec![]),
        fz(4, 9, &[3, 2], vec![1.0], vec![]),
        fz(1, 10, &[2, 0], vec![267.01684198769414], vec![]),
        fz(50, 11, &[1, 0, 3, 4, 5, 0, 0, 5, 0, 0, 5, 0, 0, 5], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 6.122846891794423e-8], vec![0.0; 13]),
        fz(1, 12, &[5, 0], vec![504.5170591280707], vec![]),
        fz(1, 13, &[5, 1], vec![1845.7881477704784], vec![]),
        fz(1, 14, &[2, 1], vec![22.559592527301515], vec![]),
        fz(3, 15, &[5, 1], vec![0.03676457413127092, 0.0, 0.0], vec![]),
    ];
    let mut p = program(6, devices);
    p.options.force_stepped = true;
    p.digital_slot_count = 16;
    p
}

#[test]
fn a_cc_step_solves_the_rail_its_gates_stamps_move_with() {
    // G2bv's bc_ic seed 425 hub 2, stepped: a {9.58 V, 133 mA} PSU in CC
    // with no rail capacitor, and a 74LS00 on the rail driving node 5, which
    // 36.8 mF holds to the rail. Powered, the gate's draw takes the rail
    // under its 4.275 V floor; held off, the rail sits at 5.26 V and the
    // capacitor charges, until about 0.25 s, where classic powers the gate
    // (H8). A CC step solves the rail with the partition, and the gate's
    // stamps (its R_out, `vS / 4 ioMax`) move with that rail; solved in one
    // linear pass about the step's start, a seed or a step from 5.29 V
    // landed the rail 8.5 mV over the one its own stamps hold there, the
    // retry freed the gate at 31.9 ms, and from then on every step's draw
    // took the rail under the floor, flipped the gate twice and restarted
    // the step: 10 ns steps, 47 ms of the sweep in its 10 s cap. Iterated,
    // the CC step holds the gate off until the retry after classic powers
    // it (0.2485-0.249 s; here one stepped commit later). Classic's values
    // (`HeadlessRunner`, drafts/f5l-logs/c425.mts) at 32 ms and 0.3 s.
    let p = g2bv_bc_ic_425_hub2();
    let w0 = |got: f64, want: f64| (got - want).abs() <= 1e-4 + 1e-3 * want.abs();
    let (rig, _, err) = hold_trace(&p, &[], &[0.032], 11);
    assert!(err.is_none(), "{err:?}");
    assert_eq!(rig.s().hold_counts(), [1.0, 0.0], "held from the load, as classic");
    assert!(w0(rig.u(), 5.287001), "the rail at {} V", rig.u());
    let (rig, trace, err) = hold_trace(&p, &[], &[0.3], 11);
    assert!(err.is_none(), "{err:?}");
    let (held, freed) = holds_and_releases(&trace);
    assert_eq!(held.len(), 1, "{held:?}");
    assert_eq!(freed.len(), 1, "{freed:?}");
    assert!((0.248..=0.253).contains(&freed[0].0), "freed at {} s", freed[0].0);
    assert!(w0(rig.u(), 4.331270), "the rail at {} V", rig.u());
}

/// The cold fuzz's seed 34 tpl 0: a {11.86 V, 94 mA} PSU, a 555 on the rail
/// (window 1.8-19.8 V) and a switch that at 27.8 ms puts 48.6 ohm on it.
/// tpl=0 hub=2 e=11.859 psu ilim=0.094 R2-4=75133.9 F4-0=0.05/0.21 R4-2=112.6 555 vcc1 gnd0 [4, 1, 4, 3, 3, 2] R0-3=21723.3 SW1-2(0)+R2-0=49
fn cold_34_tpl0() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![11.858903734048935, 0.0935614640161109], vec![]),
        fz(1, 1, &[2, 0], vec![1051.3582663382858], vec![]),
        fz(1, 2, &[3, 0], vec![454974.1230634683], vec![]),
        fz(1, 3, &[4, 0], vec![18572.010663921657], vec![]),
        fz(1, 4, &[2, 4], vec![75133.90810676213], vec![]),
        fz(10, 5, &[4, 0], vec![0.05251947962084081, 0.20622319478239332], vec![]),
        fz(1, 6, &[4, 2], vec![112.57726229434861], vec![]),
        fz(40, 7, &[1, 0, 4, 1, 4, 3, 3, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 8, &[1, 2], vec![21447.69038196634], vec![]),
        fz(1, 9, &[2, 3], vec![162.3150848648091], vec![]),
        fz(3, 10, &[3, 0], vec![5.408949459272955e-6, 0.0, 0.0], vec![]),
        fz(1, 11, &[1, 1], vec![632.7649876171475], vec![]),
        fz(1, 12, &[0, 3], vec![21723.292413162922], vec![]),
        fz(4, 13, &[1, 2], vec![0.0], vec![]),
        fz(1, 14, &[2, 0], vec![48.64505484843414], vec![]),
        fz(90, 4, &[], vec![4.0, 3.743579247739981e-6, 75133.90810676213], vec![]),
        fz(90, 6, &[], vec![6.0, 0.03902788757862344, 112.57726229434861], vec![]),
        fz(90, 8, &[], vec![8.0, 0.0013934350970086493, 21447.69038196634], vec![]),
        fz(90, 9, &[], vec![9.0, 0.036731763414938105, 162.3150848648091], vec![]),
        fz(90, 11, &[], vec![11.0, 0.0038389639281661793, 632.7649876171475], vec![]),
        fz(90, 12, &[], vec![12.0, 0.0004027918811657714, 21723.292413162922], vec![]),
        fz(90, 14, &[], vec![14.0, 2.4942039463936685, 48.64505484843414], vec![]),
    ];
    let mut p = program(5, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_held_chip_is_retried_once_per_host_step() {
    // Ruling R51 (b) at classic's cadence. Cold 34 tpl 0's switch closes at
    // 27.8 ms and the 555 starves the PSU's CC rail: powered it drags it to
    // 0.53 V, held off it climbs from 2.56 V by 10 uV a `Rail` event, every
    // 20 ns to 70 us. Its supply never reaches the window again within the
    // run. Retried at each of those re-solves, the probe paid two more rail
    // passes per event (79,000 failed retries, the board 0.12 s -> 0.6 s);
    // once per host step it pays two per `advance`.
    let mut e = Engine::load(&cold_34_tpl0()).unwrap_or_else(|er| panic!("load: {}", er.message));
    let stops = [0.006272879661111714, 0.027755330060653026, 0.03946023509167783, 0.03975728524363152, 0.03987809344734865, 0.041087485294586786, 0.044213706364126304, 0.05088267414388208];
    for &t in &stops {
        e.advance(t).unwrap_or_else(|er| panic!("{} at {}", er.message, e.time()));
        if t == stops[1] {
            e.set_params(&[(13, 0, 1.0)]).unwrap_or_else(|er| panic!("toggle: {}", er.message));
        }
    }
    let (rail_events, moves) = (e.stat(18), e.stat(14));
    assert_eq!((e.stat(25), e.stat(26)), (1.0, 0.0), "held from the toggle on");
    assert!(rail_events > 1000.0, "the held rail's climb: {rail_events} rail events");
    assert!(moves <= rail_events + 4.0 * stops.len() as f64, "a retry's rail passes at each event: {moves} moves for {rail_events} rail events");
}

/// Cold 348 tpl 2: a {4.72 V, 52 mA} PSU, a 74LS00 from the rail to node 5
/// (a capacitor's, so a dynamic partition), LEDs off the rail.
fn cold_348_tpl2() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.719736475607921, 0.05152945530038952], vec![]),
        fz(2, 1, &[2, 0], vec![-3.273475035521793], vec![]),
        fz(1, 2, &[3, 0], vec![127022.80270546685], vec![]),
        fz(1, 3, &[4, 0], vec![1831.712570292335], vec![]),
        fz(1, 4, &[5, 0], vec![2890.342124557235], vec![]),
        fz(30, 5, &[1, 3], vec![2.71961588286353, 2.2919567441261295, 0.027467401259479395, 0.0, 0.0, 0.0], vec![]),
        fz(1, 6, &[1, 1], vec![29.466282894389277], vec![]),
        fz(50, 7, &[1, 5, 1, 1, 2, 0, 0, 2, 0, 0, 2, 0, 0, 2], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 5.257354985593194e-8], vec![0.0; 13]),
        fz(1, 8, &[2, 5], vec![1773.6239697162114], vec![]),
        fz(1, 9, &[2, 3], vec![50.42310552217961], vec![]),
        fz(3, 10, &[5, 0], vec![0.00040838095935055024, 0.0, 0.0], vec![]),
        fz(3, 11, &[4, 5], vec![3.4850363960034684e-5, 0.029939290842767433, 0.0], vec![]),
        fz(1, 12, &[5, 3], vec![23168.02197052867], vec![]),
        fz(1, 13, &[5, 2], vec![5.109387329338594], vec![]),
        fz(1, 14, &[0, 4], vec![1018.1968477815329], vec![]),
        fz(30, 15, &[5, 1], vec![2.2206429163494095, 1.7089728206921646, 0.03468189382104983, 0.0, 0.0, 0.0], vec![]),
        fz(90, 6, &[], vec![6.0, 0.1409502231390421, 29.466282894389277], vec![]),
        fz(90, 8, &[], vec![8.0, 0.018510792425421595, 1773.6239697162114], vec![]),
        fz(90, 9, &[], vec![9.0, 0.00930525426503431, 50.42310552217961], vec![]),
        fz(90, 13, &[], vec![13.0, 0.36613893511709994, 5.109387329338594], vec![]),
        fz(90, 14, &[], vec![14.0, 0.00021872118298664679, 1018.1968477815329], vec![]),
    ];
    let mut p = program(6, devices);
    p.digital_slot_count = 32;
    p
}

/// Cold 696 tpl 2: a {5.30 V, 212 mA} PSU with 164 nF on the rail, a 555 on
/// it (window 1.8-19.8 V) whose draw, powered, turns the CC segment down.
fn cold_696_tpl2() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![5.300722651192141, 0.21209461081928244], vec![]),
        fz(2, 1, &[0, 2], vec![-7.3665463086539225], vec![]),
        fz(1, 2, &[3, 0], vec![6922.180438083536], vec![]),
        fz(1, 3, &[4, 0], vec![873903.1987586675], vec![]),
        fz(40, 4, &[1, 0, 1, 1, 0, 4, 1, 3], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 5, &[1, 3], vec![753.538473800529], vec![]),
        fz(1, 6, &[3, 1], vec![530.2199993377301], vec![]),
        fz(3, 7, &[1, 0], vec![1.635152068328234e-7, 0.0, 0.0], vec![]),
        fz(1, 8, &[1, 0], vec![2259.6694352455756], vec![]),
        fz(1, 9, &[3, 4], vec![1942.3118286434553], vec![]),
        fz(10, 10, &[1, 3], vec![0.4062854019360682, 0.5978469550130745], vec![]),
        fz(30, 11, &[2, 3], vec![2.353948837455101, 2.398021699688035, 0.005653295596555875, 0.0, 0.0, 0.0], vec![]),
        fz(1, 12, &[2, 2], vec![797.9412638471375], vec![]),
        fz(1, 13, &[2, 4], vec![48376.06599381678], vec![]),
        fz(1, 14, &[3, 0], vec![56927.35012328186], vec![]),
        fz(1, 15, &[4, 2], vec![448.744170219939], vec![]),
        fz(90, 5, &[], vec![5.0, 0.0046813492608382586, 753.538473800529], vec![]),
        fz(90, 6, &[], vec![6.0, 0.009280720114834263, 530.2199993377301], vec![]),
        fz(90, 8, &[], vec![8.0, 0.023943067709369565, 2259.6694352455756], vec![]),
        fz(90, 12, &[], vec![12.0, 0.06635468193071649, 797.9412638471375], vec![]),
        fz(90, 14, &[], vec![14.0, 5.03453034213674e-6, 56927.35012328186], vec![]),
    ];
    let mut p = program(5, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn the_host_steps_retry_reaches_only_static_partitions_and_a_release_that_alternates_at_once_is_none() {
    // Once per host step (ruling R51 (b)) the horizon clamp re-solves the
    // static partitions holding an IC held off. Cold 348 tpl 2's gate has
    // its ground on a capacitor's node, a dynamic partition the static
    // solver numbers but never solves: re-solved there from its all-zero
    // matrix it declined `-4` (singular) at the first host step. Cold 696
    // tpl 2's 555 carries itself in the retry's probe, but once free its
    // draw turns the CC rail capacitor's segment down (ruling R44 (a)) and
    // it alternates within the instant: freed and held again at each host
    // step, its holds chained to the guard's 51st at 0.40 s. Such a release
    // is none, and it stays held. Each runs the sweep's first 0.45 s.
    for (name, p, stops) in [
        ("348 tpl 2", cold_348_tpl2(), vec![0.006272879216757055, 0.0064176329678814455, 0.0072205984454715546, 0.047106003259173905, 0.08131269210781472, 0.10521657801372807, 0.2, 0.45]),
        ("696 tpl 2", cold_696_tpl2(), vec![0.006272881183404639, 0.006966306565007355, 0.009417531962006231, 0.010957541973997885, 0.02355459038011193, 0.026963756076411785, 0.03524749073592549, 0.04725975647383719, 0.047506684947997695, 0.06415119105970056, 0.07339441021920648, 0.0763856964974442, 0.0767216336286122, 0.07806925916008774, 0.07818709350463077, 0.07893846719838202, 0.08079720773635315, 0.08142740199172127, 0.1278637102858161, 0.12803623625819735, 0.1456503375690544, 0.16540303969681716, 0.17130347133357166, 0.17149465284851506, 0.18829395345581199, 0.22891522085945265, 0.24780265473450278, 0.25011757017665937, 0.25709668299691135, 0.25722908427919655, 0.2652023898240226, 0.2868545263940312, 0.2871097340013043, 0.2922164944666682, 0.29513873100719434, 0.30867664648499343, 0.32653383759677823, 0.3392771365981063, 0.3772691209535698, 0.3863825499058944, 0.3867281888454588, 0.38951243662268603, 0.3910711920431147, 0.3913576042444156, 0.39346624956021864, 0.3957817570250845, 0.3960732013741232, 0.396477561189899, 0.3967531855218919, 0.4240738864233175, 0.4320792030926026, 0.43239352394159325, 0.45]),
    ] {
        let mut e = Engine::load(&p).unwrap_or_else(|er| panic!("{name}: load: {}", er.message));
        for &t in &stops {
            e.advance(t).unwrap_or_else(|er| panic!("{name}: {} at {}", er.message, e.time()));
        }
        assert!(e.stat(25) <= 2.0, "{name}: {} holds", e.stat(25));
    }
}
