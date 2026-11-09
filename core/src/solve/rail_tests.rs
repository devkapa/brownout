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
    // own decline of that shape is pinned by the kernel's mock tests. The run
    // is held to an event budget rather than to wall time (a loaded host made
    // a minute's budget fail in a debug build), as 3254/0's test is.
    let (mut rig, mut t) = cc_exit_stall_rig();
    let start = rig.k.stats().events_processed;
    loop {
        t += 6.119836310668826e-4;
        rig.advance(t).unwrap_or_else(|e| panic!("declined at {:.12e}, rail {:.17} V: {e:?}", rig.k.time(), rig.u()));
        if t > 0.25 {
            break;
        }
    }
    let events = rig.k.stats().events_processed - start;
    eprintln!("B1070 on the coarse host: {events} events to 250 ms");
    assert!((rig.u() - 5.0).abs() < 1e-6, "ran to 250 ms off the setpoint: rail {:.17} V", rig.u());
    // 7,000 events at 43c9e2b; the stall's Rail events came 1.14 ps apart.
    assert!(events < 70_000, "{events} events to 250 ms on the coarse host");
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
    // Exact mode's stepped seed alternates the same gate (powered at a 2.31 V
    // supply, 0.16 V after); its Newton now runs the hold too (REF,
    // `stepped::hold_around`), where it declined `-4` at 10.23 us.
    for exact_mode in [false, true] {
        match sweep_run(&board(exact_mode), &toggles, &stops) {
            Ok(e) => assert!(e.stat(25) >= 1.0, "exact {exact_mode}: held off at least once"),
            Err((er, t)) => panic!("exact {exact_mode}: {} at {t}: {}", er.code, er.message),
        }
    }
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
    // Now it is held once and freed when the rail carries it: 35.0 to 39.5 ms
    // since its partition steps with the CC rail (rulings R59 (1) (B), R63
    // (2); 34.7 to 39.1 on the closed form's crawl, whose rail the fine
    // reference puts at the floor at 34.7 ms, where exact mode stops).
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
    assert!(spans[0].0 > 0.0349 && spans[0].0 < 0.0351 && spans[0].1 > 0.0394 && spans[0].1 < 0.0396, "{spans:?}");
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
    // 0.61 ms before ruling R90 (a); its stress cap on the gate's stepped
    // partition (an LED near its rating on node 6) takes shorter steps
    // through the slide, so the 51 holds come by 0.89 ms.
    assert!(e.time() < 1e-3, "declined within the first episode, at {} s", e.time());
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
    // once per host step it pays two per `advance`. (Since R59 (1) (B) the
    // climbing partition steps with the CC rail: no `Rail` events, the rail
    // moved once per committed step, and the retries' passes on top.)
    let mut e = Engine::load(&cold_34_tpl0()).unwrap_or_else(|er| panic!("load: {}", er.message));
    let stops = [0.006272879661111714, 0.027755330060653026, 0.03946023509167783, 0.03975728524363152, 0.03987809344734865, 0.041087485294586786, 0.044213706364126304, 0.05088267414388208];
    for &t in &stops {
        e.advance(t).unwrap_or_else(|er| panic!("{} at {}", er.message, e.time()));
        if t == stops[1] {
            e.set_params(&[(13, 0, 1.0)]).unwrap_or_else(|er| panic!("toggle: {}", er.message));
        }
    }
    let (rail_events, moves, steps) = (e.stat(18), e.stat(14), e.stat(10));
    assert_eq!((e.stat(25), e.stat(26)), (1.0, 0.0), "held from the toggle on");
    assert!(rail_events + steps > 50.0, "the held rail's climb: {rail_events} rail events, {steps} steps");
    assert!(moves <= rail_events + steps + 4.0 * stops.len() as f64, "a retry's rail passes at each event: {moves} moves for {rail_events} rail events and {steps} steps");
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

// ---- F5n: a 555's crossing against its CC rail ------------------------------

/// Cold 304 tpl 4 with its 4-6 switch open from the load, as the sweep's
/// toggle at 0.568 s leaves it: a `{5.27 V, 0.213 A}` PSU with no rail
/// capacitor, a 555 on the rail whose output is its own trigger (node 6,
/// 0.7 uF and 58 ohm to ground), its threshold on node 4 above two thirds
/// of the rail: it oscillates at 6.1 us from 4 ms, its output's load
/// taking the PSU into CC each low phase.
fn cold_304_tpl4_switch_open() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![5.2655128000042835, 0.21345953892981578], vec![]),
        fz(24, 1, &[3, 0], vec![-0.6549675638365855, 7.126998315260018, 0.004341408137726716, 0.0, 0.0, 0.017687053568537208, 0.13881918572510682], vec![8.0]),
        fz(2, 2, &[4, 3], vec![4.564892363720713], vec![]),
        fz(1, 3, &[2, 0], vec![460739.3073719522], vec![]),
        fz(1, 4, &[5, 0], vec![3271.1903360417946], vec![]),
        fz(1, 5, &[6, 0], vec![13184.966403064442], vec![]),
        fz(40, 6, &[1, 0, 6, 6, 2, 5, 4, 1], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 7, &[1, 1], vec![404.11405318823387], vec![]),
        fz(1, 8, &[1, 4], vec![149.767769862216], vec![]),
        fz(3, 9, &[4, 0], vec![2.775711698661513e-8, 0.0, 0.0], vec![]),
        fz(1, 10, &[6, 2], vec![121.32768347328833], vec![]),
        fz(3, 11, &[4, 0], vec![0.002507519258645126, 0.0, 0.0], vec![]),
        fz(30, 12, &[1, 4], vec![2.509748773473368, 2.256251739010313, 0.037513405458630994, 0.0, 0.0, 0.0], vec![]),
        fz(1, 13, &[1, 1], vec![26.531694208442694], vec![]),
        fz(1, 14, &[5, 6], vec![1090.9876182680619], vec![]),
        fz(30, 15, &[2, 6], vec![1.8912337219092163, 2.0362011275480176, 0.03775881780216939, 0.0, 0.0, 0.0], vec![]),
        fz(1, 16, &[1, 2], vec![404.1263633350451], vec![]),
        fz(1, 17, &[3, 5], vec![4.552762642533429], vec![]),
        fz(13, 18, &[3, 1], vec![0.15525272900742773, 0.047158656164903376], vec![]),
        fz(3, 19, &[6, 0], vec![6.999023568767726e-7, 0.21915706333178064, 0.0], vec![]),
        fz(4, 20, &[4, 6], vec![0.0], vec![]),
        fz(1, 21, &[6, 0], vec![58.08042003295962], vec![]),
        fz(30, 22, &[1, 2], vec![2.00045951467954, 1.558250344975702, 0.02547802372840287, 0.0, 0.0, 0.0], vec![]),
        fz(1, 23, &[1, 1], vec![774.2791402353771], vec![]),
        fz(1, 24, &[4, 5], vec![6.128352554157261], vec![]),
        fz(90, 7, &[], vec![7.0, 0.0007970148407519956, 404.11405318823387], vec![]),
        fz(90, 8, &[], vec![8.0, 0.6527049610425459, 149.767769862216], vec![]),
        fz(90, 10, &[], vec![10.0, 0.013235839817347543, 121.32768347328833], vec![]),
        fz(90, 13, &[], vec![13.0, 5.509775652361266, 26.531694208442694], vec![]),
        fz(90, 14, &[], vec![14.0, 0.14997575343365246, 1090.9876182680619], vec![]),
        fz(90, 16, &[], vec![16.0, 0.15715389918262151, 404.1263633350451], vec![]),
        fz(90, 17, &[], vec![17.0, 2.7052012112395736, 4.552762642533429], vec![]),
        fz(90, 21, &[], vec![21.0, 0.7340592066517114, 58.08042003295962], vec![]),
        fz(90, 23, &[], vec![23.0, 0.0007707332494949089, 774.2791402353771], vec![]),
        fz(90, 24, &[], vec![24.0, 23.68191074150624, 6.128352554157261], vec![]),
    ];
    let mut p = program(7, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_crossing_inside_a_cc_step_is_aimed_at_the_threshold_on_the_steps_own_rail() {
    // On a CC rail the stepped partition solves the rail as an unknown
    // (`stepped::CcRail`), and the 555's thresholds are fractions of it.
    // The crossing was found and refined against the rail as published at
    // the step's start: each refined step landed where the threshold had
    // been, the commit's rail solve moved it on, and eight steps, each a
    // fifth or less of the last, closed on every crossing of the low phase
    // (2,120 steps in 0.5 ms, 26 a period). Aimed at each trial's own rail,
    // one step lands it (20 a period); the period is unchanged (6.142 us
    // both ways; the fine reference's 6.125 us).
    // Ruling R71: without the two LEDs (15, 22), which burnt at 4.34 ms only
    // from the backward rule's charge back to their last visit (S 0 -> 48 at
    // an edge, 11,266/s); intact, the 555 ring declines at 4.344 ms in every
    // mode (NE555 point 11, a separate item). There, aimed at the published
    // rail, 28.0 steps a period; at each trial's own, 22.0.
    let mut p = cold_304_tpl4_switch_open();
    p.devices.retain(|d| !(d.kind == 30 && (d.component_index == 15 || d.component_index == 22)));
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.005).unwrap();
    let (s0, c0) = (e.stat(10), e.stat(13));
    for k in 1..=2 {
        e.advance(0.005 + k as f64 * 5e-4).unwrap();
    }
    let (steps, commits) = (e.stat(10) - s0, e.stat(13) - c0);
    let periods = commits / 4.0;
    eprintln!("5-6 ms: {steps} steps, {commits} commits, {:.2} steps a period", steps / periods);
    assert!(periods > 150.0, "oscillating: {commits} commits");
    assert!(steps / periods < 24.0, "{steps} steps over {periods} periods");
}

/// Cold 674 tpl 4: a `{10.58 V, 44 mA}` PSU with no rail capacitor, a
/// -2.2..7.9 V clock on node 3 through an LED to the rail, a 555 on the
/// rail whose THR and DIS sit in a stepped hub partition; trapezoid.
fn cold_674_tpl4() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![10.57768299388643, 0.04407705384523474], vec![]),
        fz(2, 1, &[0, 2], vec![-6.718705375382749], vec![]),
        fz(22, 2, &[3, 0], vec![82.24619002585541, 0.3079502701145943, 7.868261040906934, -2.2259392050102225, 0.0], vec![8.0]),
        fz(1, 3, &[4, 0], vec![146192.33403225866], vec![]),
        fz(1, 4, &[5, 0], vec![328070.9272781424], vec![]),
        fz(1, 5, &[6, 0], vec![469851.8818275567], vec![]),
        fz(1, 6, &[7, 0], vec![20521.342287454077], vec![]),
        fz(1, 7, &[1, 7], vec![7.023796218757453], vec![]),
        fz(3, 8, &[2, 5], vec![1.157721812433039e-6, 0.0, 0.0], vec![]),
        fz(4, 9, &[7, 4], vec![0.0], vec![]),
        fz(1, 10, &[4, 0], vec![118.77857406962261], vec![]),
        fz(1, 11, &[2, 7], vec![35.97411733858808], vec![]),
        fz(4, 12, &[6, 7], vec![0.0], vec![]),
        fz(1, 13, &[7, 0], vec![420.8333564702835], vec![]),
        fz(30, 14, &[1, 3], vec![2.0672682044045088, 2.4177662280284267, 0.025276962645213532, 0.0, 0.0, 0.0], vec![]),
        fz(1, 15, &[1, 1], vec![2695.6276078238393], vec![]),
        fz(1, 16, &[7, 2], vec![1958.1035547537406], vec![]),
        fz(30, 17, &[6, 5], vec![1.6462258209947083, 2.108257630935631, 0.025000573006314596, 0.0, 0.0, 0.0], vec![]),
        fz(40, 18, &[1, 0, 4, 5, 1, 6, 7, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 19, &[1, 2], vec![757.5560158810392], vec![]),
        fz(1, 20, &[2, 7], vec![982.9792177338045], vec![]),
        fz(3, 21, &[7, 0], vec![1.3932123167885662e-7, 0.0, 0.0], vec![]),
        fz(1, 22, &[5, 2], vec![436.51226727439683], vec![]),
        fz(90, 7, &[], vec![7.0, 0.049311835363249304, 7.023796218757453], vec![]),
        fz(90, 10, &[], vec![10.0, 2.064556550123816, 118.77857406962261], vec![]),
        fz(90, 11, &[], vec![11.0, 0.9534821766778961, 35.97411733858808], vec![]),
        fz(90, 13, &[], vec![13.0, 0.5653571923610491, 420.8333564702835], vec![]),
        fz(90, 15, &[], vec![15.0, 0.00034811013162001556, 2695.6276078238393], vec![]),
        fz(90, 16, &[], vec![16.0, 0.004042912580414882, 1958.1035547537406], vec![]),
        fz(90, 19, &[], vec![19.0, 0.046999492775666875, 757.5560158810392], vec![]),
        fz(90, 20, &[], vec![20.0, 0.007628582468226875, 982.9792177338045], vec![]),
        fz(90, 22, &[], vec![22.0, 0.0013739525041225659, 436.51226727439683], vec![]),
    ];
    let mut p = program(8, devices);
    p.digital_slot_count = 32;
    p.options.integration = 1;
    p
}

#[test]
fn a_crossing_whose_step_starts_off_its_published_rail_is_refined_on_that_rail() {
    // The clock's edge at 12.2 ms moves the published CC rail from -0.03 to
    // 7.8 V before the stepped partition's own CC solution, which at `h -> 0`
    // lies elsewhere: against each trial's own rail the 555's threshold read
    // as passed at every trial, the bisection took the step to `h / 2^40`,
    // and the advance never reached its horizon. Such a refinement falls
    // back to the published rail.
    let mut e = Engine::load(&cold_674_tpl4()).unwrap();
    for t in [0.0062728811248002015, 0.03998576024840161, 0.05] {
        e.advance(t).unwrap_or_else(|er| panic!("{} at {}", er.message, e.time()));
    }
    assert!(e.stat(10) < 5_000.0, "{} steps to 50 ms", e.stat(10));
}

// ---- F5p: ruling R59 (1) (B), the CC kind switch -----------------------------

/// tpl=3 hub=2 e=11.610 psu ilim=1.311 P4-1 C5-0=1.5e-2 R1-0=7031.6 C3-0=2.0e-6 R0-3=81.3 C4-0=5.8e-6 R0-3=41.8 74LS00 vcc1 gnd0 in0,5 out4,5 LED3-5+R1-3=35 R3-5=1.2 span=11.609561585688706
/// options: default, digital_slot_count 32 node_count 6; no toggles.
fn cold_742_tpl3() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![11.609561585688706, 1.3111584852033025], vec![]),
        fz(1, 1, &[2, 0], vec![664458.1190985516], vec![]),
        fz(1, 2, &[3, 0], vec![302924.36781416985], vec![]),
        fz(1, 3, &[4, 0], vec![1843.766479283641], vec![]),
        fz(1, 4, &[5, 0], vec![2813.098491588092], vec![]),
        fz(13, 5, &[4, 1], vec![0.4544463363598449, 0.07493446948293848], vec![]),
        fz(3, 6, &[5, 0], vec![0.015287114460682977, 0.0, 0.0], vec![]),
        fz(1, 7, &[1, 0], vec![7031.58752937688], vec![]),
        fz(3, 8, &[3, 0], vec![2.0028330936510225e-6, 0.0, 0.0], vec![]),
        fz(1, 9, &[0, 3], vec![81.2607746004466], vec![]),
        fz(3, 10, &[4, 0], vec![5.755342862865539e-6, 0.0, 0.0], vec![]),
        fz(1, 11, &[0, 3], vec![41.78674247287521], vec![]),
        fz(50, 12, &[1, 0, 0, 5, 4, 0, 0, 5, 0, 0, 5, 0, 0, 4], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 3.8617551054817935e-8], vec![0.0; 13]),
        fz(1, 13, &[4, 1], vec![559.6562593015271], vec![]),
        fz(1, 14, &[5, 2], vec![91.22972745062744], vec![]),
        fz(30, 15, &[3, 5], vec![3.0818388961845447, 2.2580775218765092, 0.0050046726686661155, 0.0, 0.0, 0.0], vec![]),
        fz(1, 16, &[1, 3], vec![34.74947927162091], vec![]),
        fz(1, 17, &[3, 5], vec![1.16432390070543], vec![]),
        fz(90, 7, &[], vec![7.0, 0.007103669718218067, 7031.58752937688], vec![]),
        fz(90, 9, &[], vec![9.0, 0.0240178024952031, 81.2607746004466], vec![]),
        fz(90, 11, &[], vec![11.0, 0.013101351844056908, 41.78674247287521], vec![]),
        fz(90, 13, &[], vec![13.0, 0.09582043795568322, 559.6562593015271], vec![]),
        fz(90, 14, &[], vec![14.0, 1.7725998004659973, 91.22972745062744], vec![]),
        fz(90, 16, &[], vec![16.0, 0.2816914522937691, 34.74947927162091], vec![]),
        fz(90, 17, &[], vec![17.0, 135.59641780709114, 1.16432390070543], vec![]),
    ];
    let mut p = program(6, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_closed_form_partition_carrying_a_cc_rail_with_no_rail_capacitor_steps_with_it_in_cc() {
    // Ruling R59 (1) (B), family A. Cold 742 tpl 3: a `{11.6 V, 1.31 A}` PSU
    // with no rail capacitor, in CC from the load, and node 4's 5.8 uF
    // capacitor 0.45 ohm (a PTC) from the rail, a closed-form hub partition.
    // In CC the rail is the capacitor plus 0.45 ohm of the limit, so it
    // ramps at I / C, 0.23 V/us, for 50 us; the closed form held the rail
    // it was aimed at and the rail re-solved every RAIL_DRIFT_V of its own
    // motion: 2.2M events, 13 s for the sweep's first 6.3 ms advance. In CC
    // the partition steps with the rail as an unknown (`stepped::CcRail`) at
    // its own RC scale. Within W0 of the fine reference (exact mode, 1 us
    // host slices) through the ramp; at the sweep's later stops the 15 mF
    // stepped partition's own backward-Euler error is 1.1-1.5 W0 in either
    // mode, so there, as in R39's test, within W0 of exact mode's floor at
    // the same host stops.
    let p = cold_742_tpl3();
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.006272881262171642).unwrap();
    assert!(e.stat(4) < 20_000.0, "{} events to the sweep's first stop", e.stat(4));
    let mut rig = Rig::load(&p).unwrap();
    let (part, _) = rig.s().local_index[4].unwrap();
    assert!(rig.s().cc_switch.contains(&part), "node 4's partition switches with the regime");
    assert_eq!(rig.hub().state.reg, Regime::Cc);
    rig.advance(1e-5).unwrap();
    assert_eq!(rig.s().partition_kinds[part], PartitionKind::Stepped, "stepped once the CC rail crawls (R60 (3))");
    let samples = [1e-5, 3e-5, 6e-5, 0.006272881262171642, 0.013412616779032716, 0.05];
    let (want, _, _) = sampled(&exact(cold_742_tpl3()), &samples, 1e-6);
    let (got, _, _) = sampled(&p, &samples, 0.0);
    let (floor, _, _) = sampled(&exact(cold_742_tpl3()), &samples, 0.0);
    for k in 0..samples.len() {
        let (off, floor_off) = (worst_w0(&got[k..=k], &want[k..=k]), worst_w0(&floor[k..=k], &want[k..=k]));
        eprintln!("742 tpl 3 at {} s: {off:.3} W0 off the fine reference (exact mode {floor_off:.3})", samples[k]);
        assert!(off <= 1.0 || off <= floor_off + 1.0, "at {} s: {off:.2} W0 off the fine reference (floor {floor_off:.2}): {:?} against {:?}", samples[k], got[k], want[k]);
    }
    assert!(worst_w0(&got[..3], &want[..3]) <= 1.0, "the ramp itself within W0");
}

/// {5 V, 0.1 A} with no rail capacitor: 100 ohm from the rail to node 2,
/// 10 uF from node 2 to ground (a closed-form hub partition, `cc_switch`),
/// and switch 3 (open) from the rail to node 3, 20 ohm from there to
/// ground. CV while it is open (50 mA at most); closed it draws 250 mA, CC,
/// where the rail is `(0.1 + vc / 100) / 0.06` and `vc` rises toward 2 V
/// with `tau` 1.2 ms. `extra` joins it.
fn cc_switch_board(extra: Vec<Device>) -> Program {
    let mut devices = vec![psu(0, 1, 0, 5.0, 0.1), resistor(1, 1, 2, 100.0), capacitor(2, 2, 0, 1e-5, 0.0), switch(3, 1, 3, false), resistor(4, 3, 0, 20.0)];
    devices.extend(extra);
    let nodes = devices.iter().flat_map(|d| d.pins.iter().map(|p| p.node)).max().unwrap() + 1;
    program(nodes, devices)
}

/// The board's node 2 from a host that toggles switch 3 at each of
/// `toggles` and reads every node at each of `samples`, advancing in
/// slices of at most `slice` (0: straight to each stop).
fn cc_switch_samples(p: &Program, toggles: &[(f64, bool)], samples: &[f64], slice: f64) -> Vec<Vec<f64>> {
    let mut e = Engine::load(p).unwrap();
    let mut stops: Vec<(f64, Option<bool>)> = toggles.iter().map(|&(t, c)| (t, Some(c))).chain(samples.iter().map(|&t| (t, None))).collect();
    stops.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (mut t, mut out) = (0.0, Vec::new());
    for (s, toggle) in stops {
        while t < s {
            t = if slice > 0.0 { (t + slice).min(s) } else { s };
            e.advance(t).unwrap_or_else(|err| panic!("{} at {t}: {}", err.code, err.message));
        }
        match toggle {
            Some(c) => e.set_params(&[(3, 0, if c { 1.0 } else { 0.0 })]).unwrap_or_else(|err| panic!("{} toggling at {t}: {}", err.code, err.message)),
            None => out.push(e.voltages().to_vec()),
        }
    }
    out
}

#[test]
fn a_cc_entry_hands_the_closed_forms_charge_to_the_stepped_copy() {
    // R59 (1) (B): at the CC entry the stepped copy starts from the closed
    // form's `vc(now)`, so node 2 (the capacitor, no esr) does not move at
    // the switch. Started from the copy's own parked state it read 0 V there
    // (the copy never ran), a 1.97 V jump. (The trigger at 0: promoted at
    // the CC entry, R60 (3)'s crawl aside.)
    let _k = hub::crawl_hook::set(Some(0));
    let mut rig = Rig::load(&cc_switch_board(vec![])).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    assert_eq!(rig.s().cc_switch, vec![part]);
    rig.advance(5e-4).unwrap();
    assert_eq!((rig.hub().state.reg, rig.s().partition_kinds[part]), (Regime::Cv, PartitionKind::ClosedForm));
    let vc = rig.v(2);
    close(vc, 5.0 * (1.0 - (-0.5f64).exp()), 1e-6, "vc(0.5 ms), CV");
    toggle(&mut rig, 3, true).unwrap();
    assert_eq!((rig.hub().state.reg, rig.s().partition_kinds[part]), (Regime::Cc, PartitionKind::Stepped), "the closed switch puts the PSU in CC");
    close(rig.v(2), vc, 1e-9, "node 2 across the switch");
    // The rail meets the limit with the capacitor where it was.
    close((rig.u() - vc) / 100.0 + rig.v(3) / 20.0, 0.1, 1e-8, "the draw at the entry's rail");
}

#[test]
fn a_cc_exit_hands_the_stepped_charge_back_to_the_closed_form() {
    // R59 (1) (B): at the CC exit the closed form resumes from the stepped
    // copy's `vc(now)`. Resumed on its own segment from the entry (aimed
    // there, 1.5 ms earlier, at the CC rail the entry solved) it read
    // 1.98855 V there against 1.98981, 12 W0.
    let mut rig = Rig::load(&cc_switch_board(vec![])).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    rig.advance(5e-4).unwrap();
    toggle(&mut rig, 3, true).unwrap();
    rig.advance(2e-3).unwrap();
    assert_eq!(rig.s().partition_kinds[part], PartitionKind::Stepped);
    let vc = rig.v(2);
    toggle(&mut rig, 3, false).unwrap();
    assert_eq!((rig.hub().state.reg, rig.s().partition_kinds[part]), (Regime::Cv, PartitionKind::ClosedForm), "the opened switch leaves CC");
    close(rig.v(2), vc, 1e-9, "node 2 across the switch");
    close(rig.u(), 5.0, 1e-12, "the CV rail");
}

#[test]
fn a_cc_switch_board_runs_within_w0_of_the_fine_reference_through_both_switches() {
    // The board switched into CC and out, against exact mode in 1 us host
    // slices: within W0 at every sample on both sides of each switch and
    // through the CC stint, or, as in R39's test, within W0 of exact mode's
    // own error at the same host stops. (A second stint entered at 3 ms
    // from vc = 3.89 V, the capacitor discharging into the CC rail, ends
    // 4.1 W0 off at 3.99 ms against exact mode's 1.1: the CC step's own
    // backward-Euler error at its controller's steps, 46 against exact
    // mode's 135 over the run, which `stepped::CcRail` already had for a
    // stepped hub partition; not this switch's.)
    let toggles = [(5e-4, true), (2e-3, false)];
    let samples = [4e-4, 5.1e-4, 1e-3, 1.9e-3, 2.001e-3, 2.01e-3, 2.5e-3, 4e-3];
    let p = cc_switch_board(vec![]);
    let want = cc_switch_samples(&exact(cc_switch_board(vec![])), &toggles, &samples, 1e-6);
    let floor = cc_switch_samples(&exact(cc_switch_board(vec![])), &toggles, &samples, 0.0);
    let got = cc_switch_samples(&p, &toggles, &samples, 0.0);
    for k in 0..samples.len() {
        let (off, floor_off) = (worst_w0(&got[k..=k], &want[k..=k]), worst_w0(&floor[k..=k], &want[k..=k]));
        eprintln!("cc switch board at {} s: {off:.3} W0 off the fine reference (exact mode {floor_off:.3})", samples[k]);
        assert!(off <= 1.0 || off <= floor_off + 1.0, "at {} s: {off:.2} W0 off the fine reference (floor {floor_off:.2}): {:?} against {:?}", samples[k], got[k], want[k]);
    }
}

#[test]
fn a_rolled_back_cc_entry_or_exit_restores_the_kind_and_the_state() {
    // R59 (1) (B), Rails point 8: an instant that switched the partition's
    // kind and then failed leaves it as the twin that never took the
    // instant: its kind, both copies' state and the pending events. The
    // event closes (then opens) the switch, which enters (leaves) CC, and
    // enables a zero-delay oscillator on a fixed 5 V, which livelocks.
    // The trigger at 0: the kind switches at the CC entry (R60 (3) aside).
    let _k = hub::crawl_hook::set(Some(0));
    let p = cc_switch_board(vec![source(5, 4, 0, 5.0), oscillator(6, 4, 5, 6, [7, 8, 9]), switch(7, 4, 6, false), resistor(8, 6, 0, 1e3)]);
    let (mut failed, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
    let part = failed.s().local_index[2].unwrap().0;
    for (t, closed, kind) in [(5e-4, true, PartitionKind::ClosedForm), (2e-3, false, PartitionKind::Stepped)] {
        failed.advance(t).unwrap();
        twin.advance(t).unwrap();
        let before = state_digest(failed.s());
        assert_eq!(before, state_digest(twin.s()), "the twins agree before");
        assert_eq!(failed.s().partition_kinds[part], kind);
        let r = failed.k.apply_params_as_event(&[(3, 0, if closed { 1.0 } else { 0.0 }), (7, 0, 1.0)]);
        failed.k.solver_mut().prepare_readout();
        assert!(matches!(r, Err(KernelError::Livelock { .. })), "the instant livelocks: {r:?}");
        assert_eq!(failed.s().partition_kinds[part], kind, "the kind the instant found");
        assert_eq!(state_digest(failed.s()), before, "every piece of state");
        assert_eq!(failed.k.next_event_time().to_bits(), twin.k.next_event_time().to_bits(), "the next event");
        // Both take the switch alone, and go on alike.
        toggle(&mut failed, 3, closed).unwrap();
        toggle(&mut twin, 3, closed).unwrap();
        assert_ne!(failed.s().partition_kinds[part], kind, "the switch alone switches the kind");
        failed.advance(t + 2e-4).unwrap();
        twin.advance(t + 2e-4).unwrap();
        assert_eq!(state_digest(failed.s()), state_digest(twin.s()), "it goes on like the twin");
    }
}


/// tpl=3 hub=2 e=11.017 psu ilim=0.097 V2=-1.91 74LS00 vcc4 gnd0 in5,0 out1,0 C6-3=1.9e-7 SW1-6(1)+R6-0=2893 span=11.017203710490474
fn cold_794_tpl3() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![11.017203710490474, 0.09731248418656985], vec![]),
        fz(2, 1, &[0, 2], vec![-1.9130032647655177], vec![]),
        fz(1, 2, &[3, 0], vec![1241.7956231301166], vec![]),
        fz(1, 3, &[4, 0], vec![4000.3546919888026], vec![]),
        fz(1, 4, &[5, 0], vec![262116.5738109504], vec![]),
        fz(1, 5, &[6, 0], vec![2078.7962621837546], vec![]),
        fz(50, 6, &[4, 0, 5, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 2.4806769510188443e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 7, &[1, 4], vec![116.17373165447268], vec![]),
        fz(1, 8, &[0, 3], vec![62.305324706115236], vec![]),
        fz(3, 9, &[6, 3], vec![1.9372546992087074e-7, 0.0, 0.0], vec![]),
        fz(4, 10, &[1, 6], vec![1.0], vec![]),
        fz(1, 11, &[6, 0], vec![2893.31809477976], vec![]),
        fz(90, 7, &[], vec![7.0, 0.38720362085734644, 116.17373165447268], vec![]),
        fz(90, 8, &[], vec![8.0, 0.15765786712810706, 62.305324706115236], vec![]),
        fz(90, 11, &[], vec![11.0, 0.013024491059154694, 2893.31809477976], vec![]),
    ];
    let mut p = program(7, devices);
    p.digital_slot_count = 32;
    p
}

/// tpl=2 hub=2 e=4.790 psu ilim=0.058 V2=9.69 74LS00 vcc1 gnd0 in3,4 out0,4 LED0-1+R0-0=116 R0-2=22626.8 R0-1=1630.6 C4-0=1.2e-5 C6-3=4.9e-3 span=9.686918810827134
fn cold_230_tpl2() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.7895928406358665, 0.057946407746576306], vec![]),
        fz(2, 1, &[2, 0], vec![9.686918810827134], vec![]),
        fz(1, 2, &[3, 0], vec![5410.252982843795], vec![]),
        fz(1, 3, &[4, 0], vec![1504.6424698142757], vec![]),
        fz(1, 4, &[5, 0], vec![190307.6763461144], vec![]),
        fz(1, 5, &[6, 0], vec![12210.48184730575], vec![]),
        fz(50, 6, &[1, 0, 3, 4, 0, 0, 0, 4, 0, 0, 4, 0, 0, 0], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 3.81459829974826e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 7, &[0, 2], vec![325.06946353944704], vec![]),
        fz(1, 8, &[4, 5], vec![1381.0276419239983], vec![]),
        fz(30, 9, &[0, 1], vec![1.946727242733705, 2.220559434300754, 0.022183122057487726, 0.0, 0.0, 0.0], vec![]),
        fz(1, 10, &[0, 0], vec![116.20810406270883], vec![]),
        fz(1, 11, &[0, 2], vec![22626.834415103014], vec![]),
        fz(1, 12, &[0, 1], vec![1630.5836327744385], vec![]),
        fz(3, 13, &[4, 0], vec![1.2327988001576903e-5, 0.0, 0.0], vec![]),
        fz(1, 14, &[1, 5], vec![2149.561908510973], vec![]),
        fz(3, 15, &[6, 3], vec![0.004940783759334168, 0.0, 0.0], vec![]),
        fz(90, 7, &[], vec![7.0, 0.018764788533395884, 325.06946353944704], vec![]),
        fz(90, 8, &[], vec![8.0, 0.002629977616967989, 1381.0276419239983], vec![]),
        fz(90, 10, &[], vec![10.0, 0.0016796939546747593, 116.20810406270883], vec![]),
        fz(90, 11, &[], vec![11.0, 0.00013522289582969471, 22626.834415103014], vec![]),
        fz(90, 12, &[], vec![12.0, 0.0010275453326120548, 1630.5836327744385], vec![]),
    ];
    let mut p = program(7, devices);
    p.digital_slot_count = 32;
    p
}

/// tpl=5 hub=2 e=5.326 psu ilim=0.085 V2=-3.54 C2-3=1.1e-6 C3-0=7.4e-5 74LS00 vcc1 gnd0 in5,3 out3,4 R5-0=164.1 C2-4=1.0e-6 span=8.862602086217157
fn cold_527_tpl5() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![5.326123393256066, 0.08520110182928953], vec![]),
        fz(2, 1, &[2, 0], vec![-3.536478692961091], vec![]),
        fz(1, 2, &[3, 0], vec![7468.046982501495], vec![]),
        fz(1, 3, &[4, 0], vec![399106.7266990096], vec![]),
        fz(1, 4, &[5, 0], vec![1171.738260236386], vec![]),
        fz(3, 5, &[2, 3], vec![1.105046700596323e-6, 0.01325885827059807, 0.0], vec![]),
        fz(3, 6, &[3, 0], vec![7.439781567682745e-5, 0.0, 0.0], vec![]),
        fz(50, 7, &[1, 0, 5, 3, 3, 0, 0, 4, 0, 0, 4, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 3.343813056587388e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 8, &[3, 2], vec![124.8537036911802], vec![]),
        fz(1, 9, &[4, 4], vec![69.62253856221633], vec![]),
        fz(1, 10, &[5, 0], vec![164.11374515863153], vec![]),
        fz(3, 11, &[2, 4], vec![1.0148448311073916e-6, 0.0, 0.0], vec![]),
        fz(1, 12, &[0, 4], vec![2104.728126043158], vec![]),
        fz(90, 8, &[], vec![8.0, 0.007577646760766647, 124.8537036911802], vec![]),
        fz(90, 9, &[], vec![9.0, 0.0023199660085589918, 69.62253856221633], vec![]),
        fz(90, 10, &[], vec![10.0, 0.49965919186909724, 164.11374515863153], vec![]),
    ];
    let mut p = program(6, devices);
    p.digital_slot_count = 32;
    p
}

/// Runs `p` on the sweep's host `stops`, toggling switch `d` (param 0) at
/// each of `toggles` once the host reaches it; the first error, with the
/// time it left the engine at.
fn run_cold(p: &Program, toggles: &[(f64, usize)], stops: &[f64]) -> Result<Engine, String> {
    let mut e = Engine::load(p).map_err(|er| format!("load {} {}", er.code, er.message))?;
    let mut state: Vec<(usize, f64)> = Vec::new();
    let mut k = 0;
    for &t in stops {
        e.advance(t).map_err(|er| format!("advance {} at {}: {}", er.code, e.time(), er.message))?;
        while k < toggles.len() && toggles[k].0 <= t {
            let d = toggles[k].1;
            let v = match state.iter_mut().find(|s| s.0 == d) {
                Some(s) => { s.1 = 1.0 - s.1; s.1 }
                None => { let v = 1.0 - p.devices[d].params[0]; state.push((d, v)); v }
            };
            e.set_params(&[(d as u32, 0, v)]).map_err(|er| format!("set_params {} at {t}: {}", er.code, er.message))?;
            k += 1;
        }
    }
    Ok(e)
}

#[test]
fn a_cc_seed_at_a_rail_where_a_held_gate_has_no_supply_has_a_finite_sensitivity() {
    // Cold 230 tpl 2 (527 tpl 5 too, at 10.7 ms): a 74LS00 on a CC rail with no rail
    // capacitor, its output in a stepped hub partition. The CC seed solves
    // the rail with the partition, and its draw pulled it below the gate's
    // ground (u = -0.68 V in 230's seed), where the supply difference reads
    // 0; the gate's power is judged at the loop's rail (`hold_power_at`), so
    // it still stamps its 25 ohm output, whose conductance is the constant
    // piece of its supply curve. That piece's rail derivative, `-b / vs^2`
    // with `b = 0`, was evaluated as 0 / 0: a NaN sensitivity, `-4` at load.
    // The derivative of a constant piece is 0 at any supply.
    run_cold(&cold_230_tpl2(), &[], &[0.006272880109317727, 0.013326498829160302, 0.014475158088294663, 0.05]).unwrap_or_else(|e| panic!("230 tpl 2: {e}"));
}

#[test]
fn a_cc_exit_in_the_instant_that_entered_cc_leaves_the_partition_stepped_until_a_later_event() {
    // Cold 794 tpl 3: {11.0 V, 97 mA}, no rail capacitor, 0.19 uF across
    // nodes 6 and 3 (a closed-form hub partition, `cc_switch`). At 13.04 us
    // the stepped copy's CC solve puts the rail at the setpoint, CV, and the
    // closed form resumed there drew past the limit, CC: the kind switched
    // 51 times in the one instant and the rail loop gave up (`-4`). A
    // partition promoted in an instant stays stepped through it; the next
    // event demotes it if the PSU is still in CV (R59 (1) (B) hysteresis).
    let e = run_cold(&cold_794_tpl3(), &[(0.015596114684644283, 10)], &[0.0062728802366896095, 0.007777438227485965, 0.015596114684644283, 0.016132343116063375, 0.03796558824037464, 0.05]);
    if let Err(e) = e {
        panic!("794 tpl 3: {e}");
    }
}

#[test]
fn a_cc_closed_form_steps_only_once_its_rail_crawls() {
    // Rulings R60 (3), R63 (2). The switch board closed at 0.5 ms: CC, the
    // capacitor carrying the rail 33 mV toward 2 V with `tau` 1.2 ms, a
    // `Rail` event every 10 uV of it. It is closed-form at the entry and
    // steps from the first rail event, whose move over its interval puts the
    // rail's pace past W0 per time constant (the crawl count, at most
    // `CC_CRAWL_EVENTS`, would have come later), the charge handed across.
    let mut rig = Rig::load(&cc_switch_board(vec![])).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    rig.advance(5e-4).unwrap();
    toggle(&mut rig, 3, true).unwrap();
    assert_eq!((rig.hub().state.reg, rig.s().partition_kinds[part]), (Regime::Cc, PartitionKind::ClosedForm), "closed-form at the CC entry");
    let e0 = rig.hub().stats.events;
    while rig.s().partition_kinds[part] == PartitionKind::ClosedForm {
        let t = rig.k.next_event_time();
        assert!(t < 1e-3, "no crawl found by 1 ms");
        rig.advance(t).unwrap();
    }
    let t = rig.k.time();
    let events = rig.hub().stats.events - e0;
    assert_eq!(events, 1, "promoted at the crawl's first rail event");
    close(rig.v(2), rig.s().closed_form.vc_now(part, t).unwrap(), 1e-9, "node 2 across the promotion");
    // A CC rail its closed form barely moves: 100 kohm to 10 uF behind a
    // 20 ohm load in CC (`tau` 1 s). Its rail moves R_load / R times the
    // capacitor's 0.2 V over 0.2 s, 40 uV, four events: it stays closed-form
    // (the R59 prototype stepped it from the entry).
    let p = program(4, vec![psu(0, 1, 0, 5.0, 0.1), resistor(1, 1, 2, 1e5), capacitor(2, 2, 0, 1e-5, 0.0), switch(3, 1, 3, true), resistor(4, 3, 0, 20.0)]);
    let mut rig = Rig::load(&p).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    assert!(rig.s().cc_switch.contains(&part));
    rig.advance(0.2).unwrap();
    eprintln!("slow CC board: {} rail events, kind {:?}, steps {}", rig.hub().stats.events, rig.s().partition_kinds[part], rig.s().stepped_steps());
    assert_eq!((rig.hub().state.reg, rig.s().partition_kinds[part]), (Regime::Cc, PartitionKind::ClosedForm), "never crawled");
    assert!(rig.hub().stats.events < hub::CC_CRAWL_EVENTS as u64);
}

#[test]
fn a_cc_rail_crawling_under_a_stepped_partition_stays_within_w0_of_the_fine_reference() {
    // Ruling R60 (4), G2bv's bc_ic seed 97 hub 2 (family A): 3.16 mF from a
    // CC rail with no rail capacitor, the rail creeping at the capacitor's
    // pace toward a 74LS00's floor. Stepped with the rail (R59 (1) (B)), its
    // steps at the stepped solver's own tolerance ran 1.48 W0 off the fine
    // reference (exact mode, 1 us host steps) over 30-34.7 ms, the rail 4 mV
    // low; exact mode at the same host stops is 1.50 off and 0.09 at 100 us
    // host steps, so the host cadence lets the steps grow (D6). A promoted
    // partition's CC step error is held `CC_STEP_TOL_SCALE` times tighter:
    // 0.70 W0, 64 steps against 40.
    let b = || { let mut p = program(6, g2bv_bc_ic_97_hub2()); p.digital_slot_count = 16; p };
    let samples: Vec<f64> = (0..=9).map(|k| 0.030 + 0.0005 * k as f64).chain([0.0347]).collect();
    let (want, _, _) = sampled(&exact(b()), &samples, 1e-6);
    let stops = [0.00010054129372012221, 0.0003467103846392178, 0.0004605220067872004];
    let mut e = Engine::load(&b()).unwrap();
    let mut worst: f64 = 0.0;
    for &s in &stops {
        e.advance(s).unwrap();
    }
    for (k, &s) in samples.iter().enumerate() {
        e.advance(s).unwrap();
        worst = worst.max(worst_w0(&[e.voltages().to_vec()], &want[k..=k]));
    }
    eprintln!("bc_ic 97: {worst:.3} W0 off the fine reference over 30-34.7 ms, {} steps", e.stat(10));
    assert!(worst <= 1.0, "{worst:.3} W0 off the fine reference");
}
/// tpl=5 hub=2 e=7.565 psu ilim=0.165 R2-0=1122.5 C0-5=2.4e-3 C2-1=2.2e-6 SW1-5(1)+R5-0=189 span=7.564565524781408
fn cold_677_tpl5() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![7.564565524781408, 0.16481947944548236], vec![]),
        fz(1, 1, &[2, 0], vec![1774.3603103610976], vec![]),
        fz(1, 2, &[3, 0], vec![570141.1479501962], vec![]),
        fz(1, 3, &[4, 0], vec![988861.5535183402], vec![]),
        fz(1, 4, &[5, 0], vec![26239.84588231391], vec![]),
        fz(1, 5, &[2, 0], vec![1122.4846265053593], vec![]),
        fz(3, 6, &[0, 5], vec![0.0024211253084653866, 0.0, 0.0], vec![]),
        fz(3, 7, &[2, 1], vec![2.1930046168815875e-6, 0.0, 0.0], vec![]),
        fz(4, 8, &[1, 5], vec![1.0], vec![]),
        fz(1, 9, &[5, 0], vec![189.17100109376014], vec![]),
        fz(90, 5, &[], vec![5.0, 0.0006140458982237856, 1122.4846265053593], vec![]),
        fz(90, 9, &[], vec![9.0, 0.04683206193458161, 189.17100109376014], vec![]),
    ];
    let mut p = program(6, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_load_switched_off_a_cc_rail_leaves_cc_at_the_switch() {
    // Cold 677 tpl 5: {7.56 V, 165 mA}, no rail capacitor; the switch from
    // the rail charges 2.4 mF (node 5, 189 ohm across it) in CC, and 2.2 uF
    // from the rail to node 2 (1.12 kohm to ground) carries the rail. At
    // 78.94 ms the switch opens: the rest draws a few mA at the setpoint, so
    // the PSU leaves CC at the switch (the fine reference: 7.5646 V there).
    let stops = [0.0062728811181171394, 0.011106993384281238, 0.011322953395826995, 0.011834470099438776, 0.01233693176553175, 0.013194973258403004, 0.014036283416555679, 0.014710983240353128, 0.03775409661270685, 0.0389724476464377, 0.04462674479283428, 0.04742245189835752, 0.04791251210039341, 0.04893083427134086, 0.0525974462585554, 0.05528146562750171, 0.05823269777715527, 0.058356311835162024, 0.07893848215151009];
    let e = run_cold(&cold_677_tpl5(), &[(0.07893848215151009, 8)], &stops).unwrap_or_else(|e| panic!("677 tpl 5: {e}"));
    close(e.voltages()[1], 7.564565524781408, 1e-4 + 1e-3 * 7.564565524781408, "the rail at the switch");
}
/// tpl=3 hub=2 e=5.024 psu ilim=0.026 V2=-6.32 C4-3=8.0e-7 R3-4=4.1 C2-0=1.6e-4 SW1-3(1)+R3-0=216 span=6.317608046450156
fn cold_75_tpl3() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![5.024009637607736, 0.026496295412856984], vec![]),
        fz(2, 1, &[0, 2], vec![-6.317608046450156], vec![]),
        fz(1, 2, &[3, 0], vec![8616.47032068873], vec![]),
        fz(1, 3, &[4, 0], vec![1013.5623103799135], vec![]),
        fz(1, 4, &[5, 0], vec![1286.1594095452047], vec![]),
        fz(3, 5, &[4, 3], vec![7.964078236822556e-7, 0.01141208585422171, 0.0], vec![]),
        fz(1, 6, &[3, 4], vec![4.138926041343154], vec![]),
        fz(3, 7, &[2, 0], vec![0.00016087837200354314, 0.0, 0.0], vec![]),
        fz(4, 8, &[1, 3], vec![1.0], vec![]),
        fz(1, 9, &[3, 0], vec![216.2909858301283], vec![]),
        fz(90, 6, &[], vec![6.0, 3.5737332998716207, 4.138926041343154], vec![]),
        fz(90, 9, &[], vec![9.0, 0.02351636225530671, 216.2909858301283], vec![]),
    ];
    let mut p = program(6, devices);
    p.options.integration = 1;
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_cc_exit_inside_a_horizon_clamp_steps_on_in_cv() {
    // Cold 75 tpl 3: {5.02 V, 26 mA}, no rail capacitor; 0.8 uF and 4.1 ohm
    // and 160 uF behind the switch (a `cc_switch` partition, stepped once its
    // rail crawls). At 0.3105 s its CC rail reaches the setpoint inside one
    // host advance. The horizon clamp steps it to the host's time with the
    // CC rail it was handed: each step was cut at the setpoint, the next
    // began in CC again, 19,393 steps of 20-40 ns in one 1.9 ms advance.
    // Once a step of the clamp leaves CC at its cut, the rest of the clamp
    // steps on in CV; the rail loop after it judges the regime.
    let stops = [0.006272879754842847, 0.00848064914786476, 0.008755727396472172, 0.045373527410157155, 0.06274331074818437, 0.0930288786114757, 0.09391230397663634, 0.09613978254352885, 0.11521889268836158, 0.1169227172556522, 0.11892304169877559, 0.11915944412089538, 0.16806931209804682, 0.20853725478047835, 0.20924457038396038, 0.22750630001394725, 0.22765838628755417, 0.22800180733389527, 0.22900689682185624, 0.2472591254663561, 0.24818096928705943, 0.28755792577392003, 0.2879162427112212, 0.29544673278915945, 0.29635115942674856, 0.2979422710021157, 0.31049165447559385, 0.3124182183801322, 0.3125];
    let mut e = Engine::load(&cold_75_tpl3()).unwrap();
    for &t in &stops {
        let before = e.stat(10);
        e.advance(t).unwrap_or_else(|er| panic!("{} at {}: {}", er.code, e.time(), er.message));
        assert!(e.stat(10) - before < 2_000.0, "{} steps in the advance to {t}", e.stat(10) - before);
    }
}
/// tpl=4 hub=2 e=5.368 psu ilim=1.164 PULSE3 -2.7..9.8 tr=0e0 555 vcc2 gnd0 [3, 1, 1, 5, 2, 0] R4-3=1.9 F4-2=0.74/0.62 R0-2=4.6 C3-5=1.0e-7 C2-1=1.3e-2 span=12.45842047380064
fn cold_72_tpl4() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![5.368197763625233, 1.1643787092961784], vec![]),
        fz(24, 1, &[3, 0], vec![-2.6576602934370257, 9.800760180363614, 0.0063877126823550295, 0.0, 0.0, 0.005346738495807022, 0.22941104323127617], vec![8.0]),
        fz(1, 2, &[2, 0], vec![5771.555035399707], vec![]),
        fz(1, 3, &[4, 0], vec![19983.665856854375], vec![]),
        fz(1, 4, &[5, 0], vec![35214.23416671305], vec![]),
        fz(40, 5, &[2, 0, 3, 1, 1, 5, 2, 0], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 6, &[2, 0], vec![1780.910847002465], vec![]),
        fz(1, 7, &[0, 2], vec![3275.3061127125998], vec![]),
        fz(3, 8, &[2, 0], vec![4.12971922334916e-8, 0.0, 0.0], vec![]),
        fz(1, 9, &[1, 5], vec![340.00159473362675], vec![]),
        fz(1, 10, &[3, 1], vec![537.5038819845298], vec![]),
        fz(1, 11, &[4, 3], vec![1.856955189027227], vec![]),
        fz(10, 12, &[4, 2], vec![0.7392063015686207, 0.6186105546830768], vec![]),
        fz(1, 13, &[0, 2], vec![4.556055974881248], vec![]),
        fz(1, 14, &[1, 2], vec![149.57406315428537], vec![]),
        fz(3, 15, &[3, 5], vec![1.0280941045132062e-7, 0.0, 0.0], vec![]),
        fz(3, 16, &[2, 1], vec![0.012985384227155158, 0.0, 0.0], vec![]),
        fz(1, 17, &[0, 2], vec![23518.995172271967], vec![]),
        fz(90, 6, &[], vec![6.0, 0.00018413708421505297, 1780.910847002465], vec![]),
        fz(90, 7, &[], vec![7.0, 0.0009526717996846093, 3275.3061127125998], vec![]),
        fz(90, 9, &[], vec![9.0, 0.7009222735005726, 340.00159473362675], vec![]),
        fz(90, 11, &[], vec![11.0, 21.093375225983795, 1.856955189027227], vec![]),
        fz(90, 13, &[], vec![13.0, 40.733271535141725, 4.556055974881248], vec![]),
    ];
    let mut p = program(6, devices);
    p.digital_slot_count = 32;
    p
}


#[test]
fn a_closed_form_on_a_cc_rail_another_partition_moves_steps_with_it() {
    // Ruling R63 (2). Cold 72 tpl 4: {5.37 V, 1.16 A}, no rail capacitor,
    // 13 mF from the rail to node 2 (a stepped partition) moving the CC rail
    // 90 mV/ms; node 5 (0.1 uF to a pulse's node, 340 ohm to the rail, a
    // `cc_switch` closed form, `tau` 34 us) follows the rail. The rail moves
    // only at the stepped partition's commits, about a millisecond apart, and
    // no `Rail` event re-aims node 5 between them: no crawl, but node 5 was
    // aimed at a rail a commit old, 0.083 V low at 6.27 ms (95 W0 off the
    // fine reference; stepped with the rail from the entry, 5.5). A closed
    // form whose rail moves more than W0 within one of its time constants
    // steps. Within W0 of the stepped run at the sweep's host stops.
    let stops = [0.006272879757094588, 0.006961352915624008, 0.009458814460263459, 0.01153130615462656, 0.05808507802599983, 0.058830097853103336, 0.0595741505624736, 0.05967846412462022, 0.06266170036641795, 0.07953558078788245, 0.07978378149045819, 0.08023133162152682, 0.08310892147936291, 0.08479671467796533, 0.08490146648731667, 0.09560781787769408, 0.09792275902071916, 0.0980450874315556, 0.09825769978442092, 0.1];
    let run = |k: Option<u32>| {
        let _k = hub::crawl_hook::set(k);
        let mut e = Engine::load(&cold_72_tpl4()).unwrap();
        stops.iter().map(|&t| { e.advance(t).unwrap_or_else(|er| panic!("{} at {t}: {}", er.code, er.message)); e.voltages().to_vec() }).collect::<Vec<Vec<f64>>>()
    };
    let (lazy, stepped) = (run(None), run(Some(0)));
    for k in 0..stops.len() {
        let off = worst_w0(&lazy[k..=k], &stepped[k..=k]);
        assert!(off <= 1.0, "at {} s: {off:.2} W0 off the stepped run: {:?} against {:?}", stops[k], lazy[k], stepped[k]);
    }
}

/// {5 V, 0.1 A} with no rail capacitor: 100 ohm from the rail to node 2,
/// 10 uF from node 2 to ground (a `cc_switch` hub partition), 20 ohm from
/// the rail to node 3, where a one-shot pulse holds 5 V but for 0 V from
/// 1 ms for 30.0371 ms: CC through the low phase (the capacitor settles at
/// 2 V), CV from its end, an internal event.
fn cc_pulse_board(force_stepped: bool) -> Program {
    let mut p = program(4, vec![psu(0, 1, 0, 5.0, 0.1), resistor(1, 1, 2, 100.0), capacitor(2, 2, 0, 1e-5, 0.0), resistor(3, 1, 3, 20.0), pulse(4, 3, 0, [5.0, 0.0, 1e-3, 0.0, 0.0, 0.0300371, 1.0], 0)]);
    p.options.force_stepped = force_stepped;
    p
}

#[test]
fn a_cc_exit_at_an_internal_event_brings_a_stepped_partition_to_it_on_the_cc_rail_it_was_on() {
    // F5pV: the pulse's end (31.0371 ms) leaves CC. Node 2's stepped copy
    // (promoted in the stint; stepped from load under FORCE_STEPPED) last
    // committed 3 ms before it, settled at 2 V, and was brought to the edge
    // after the rail moved: demoted (`SteppedSolver::demote`) or re-solved
    // at the exit, it stepped those 3 ms at the 5 V setpoint and read
    // 4.834 V just after the edge against the fine reference's 2.003 (1346
    // W0). On the CC rail its pending step was planned on it stays at 2 V
    // to the edge. Within W0 of the fine reference (exact mode, 1 us host
    // slices), or of exact mode's own error at the same host stops.
    let samples = [1.5e-3, 0.0280371, 0.0310381, 0.0310471, 0.0311371, 0.0330371];
    let (want, _, _) = sampled(&exact(cc_pulse_board(false)), &samples, 1e-6);
    let (floor, _, _) = sampled(&exact(cc_pulse_board(false)), &samples, 0.0);
    for force_stepped in [false, true] {
        let (got, _, _) = sampled(&cc_pulse_board(force_stepped), &samples, 0.0);
        for k in 0..samples.len() {
            let (off, floor_off) = (worst_w0(&got[k..=k], &want[k..=k]), worst_w0(&floor[k..=k], &want[k..=k]));
            eprintln!("cc pulse board (force_stepped {force_stepped}) at {} s: {off:.3} W0 off the fine reference (exact mode {floor_off:.3}), node 2 {:.6} against {:.6}", samples[k], got[k][2], want[k][2]);
            assert!(off <= 1.0 || off <= floor_off + 1.0, "force_stepped {force_stepped} at {} s: {off:.2} W0 off the fine reference (floor {floor_off:.2}): {:?} against {:?}", samples[k], got[k], want[k]);
        }
    }
}

/// The cc pulse board with a second capacitor behind node 2 (1 uF, 1 kohm
/// to ground), which makes node 2's partition stepped in default mode too.
fn cc_pulse_board_two_caps(force_stepped: bool) -> Program {
    let mut p = cc_pulse_board(force_stepped);
    p.devices.extend([capacitor(5, 2, 4, 1e-6, 0.0), resistor(6, 4, 0, 1000.0)]);
    p.node_count = 5;
    p
}

#[test]
fn a_cc_entry_at_an_internal_event_brings_a_stepped_partition_to_it_on_the_rail_it_was_on() {
    // Ruling R66 (1), the exit's mirror. The pulse's start (1 ms) puts the
    // PSU in CC. Node 2's stepped partition (stepped from load under
    // FORCE_STEPPED, or for its two capacitors) last committed before the
    // edge and was brought to it after the rail had left the 5 V setpoint
    // for the CC rail: it stepped its gap at the CC rail and read 2.883 V at
    // 1.5 ms against the fine reference's 2.765 (41.3 W0, as at da167d2).
    // On the setpoint it stood at to the edge, within W0 of the fine
    // reference (exact mode, 1 us host slices), or of exact mode's own error
    // at the same host stops.
    let samples = [1.001e-3, 1.01e-3, 1.1e-3, 1.5e-3, 3e-3, 0.0280371];
    for (name, board) in [("two capacitors", cc_pulse_board_two_caps as fn(bool) -> Program), ("one capacitor", cc_pulse_board)] {
        let (want, _, _) = sampled(&exact(board(false)), &samples, 1e-6);
        let (floor, _, _) = sampled(&exact(board(false)), &samples, 0.0);
        for force_stepped in [false, true] {
            let p = board(force_stepped);
            let (got, _, _) = sampled(&p, &samples, 0.0);
            for k in 0..samples.len() {
                let (off, floor_off) = (worst_w0(&got[k..=k], &want[k..=k]), worst_w0(&floor[k..=k], &want[k..=k]));
                eprintln!("{name} (force_stepped {force_stepped}) at {} s: {off:.3} W0 off the fine reference (exact mode {floor_off:.3}), node 2 {:.6} against {:.6}", samples[k], got[k][2], want[k][2]);
                assert!(off <= 1.0 || off <= floor_off + 1.0, "{name}, force_stepped {force_stepped} at {} s: {off:.2} W0 off the fine reference (floor {floor_off:.2}): {:?} against {:?}", samples[k], got[k], want[k]);
            }
        }
    }
}

/// The cc pulse board with a second 20 ohm load from the rail to node 4,
/// whose pulse pulls it to 0 V from `at`. At the first pulse's end (`at`
/// 31.0371 ms) one load leaves as the other takes its place and the PSU
/// stays in CC at the same rail; at 20 ms the second load joins the first
/// and the CC rail drops from 3.5 V to 1.23 V.
fn cc_pulse_board_second_load(force_stepped: bool, at: f64) -> Program {
    let mut p = cc_pulse_board(force_stepped);
    p.devices.extend([resistor(5, 1, 4, 20.0), pulse(6, 4, 0, [5.0, 0.0, at, 0.0, 0.0, 1.0, 10.0], 1)]);
    p.node_count = 5;
    p.digital_slot_count = 2;
    p
}

#[test]
fn a_cc_rail_stepped_at_a_source_edge_leaves_a_stepped_capacitor_where_it_was() {
    // Defect D (F5rV): at the first pulse's end a second pulse puts an equal
    // load on the rail, two source edges at one instant with no regime
    // change. The rail stays at its CC value and node 2's 10 uF with it; in
    // default mode its stepped partition (promoted in the stint) was
    // re-solved at the passes' 1.05 V between the two edges and stepped its
    // 3 ms gap there: 1.17 V just after the edge against the fine
    // reference's 3.5 (647 W0), 44 W0 still 3 ms later. With the second
    // load alone (20 ms), FORCE_STEPPED kept the trial it had planned before
    // the edge, which the next host stop cut and stepped again from its
    // start on the new rail: 1.46 V against 3.50 (566 W0). Within W0 of the
    // fine reference (exact mode, 1 us host slices), or of exact mode's own
    // error at the same host stops.
    let t_end = 1e-3 + (0.0 + 0.0300371);
    for at in [t_end, 0.02] {
        let samples = [0.0181, at + 1e-6, at + 1e-5, at + 1e-4, at + 1e-3, at + 3e-3, 0.040];
        let (want, _, _) = sampled(&exact(cc_pulse_board_second_load(false, at)), &samples, 1e-6);
        let (floor, _, _) = sampled(&exact(cc_pulse_board_second_load(false, at)), &samples, 0.0);
        for force_stepped in [false, true] {
            let (got, _, _) = sampled(&cc_pulse_board_second_load(force_stepped, at), &samples, 0.0);
            for k in 0..samples.len() {
                let (off, floor_off) = (worst_w0(&got[k..=k], &want[k..=k]), worst_w0(&floor[k..=k], &want[k..=k]));
                eprintln!("second load at {at} (force_stepped {force_stepped}) at {} s: {off:.3} W0 off the fine reference (exact mode {floor_off:.3}), node 2 {:.6} against {:.6}", samples[k], got[k][2], want[k][2]);
                assert!(off <= 1.0 || off <= floor_off + 1.0, "second load at {at}, force_stepped {force_stepped} at {} s: {off:.2} W0 off the fine reference (floor {floor_off:.2}): {:?} against {:?}", samples[k], got[k], want[k]);
            }
        }
    }
}

#[test]
fn a_demoted_partition_draws_on_the_rail_as_its_closed_form_alone() {
    // R59 (1) (B): a demotion that brings the stepped copy to `now` commits
    // steps, each queueing its draw for the rail; drained after the closed
    // form's own (`drain_contributions` takes the stepped queue last), the
    // last of them stood as the closed-form partition's contribution, a
    // stepped line `i + g (u - u_p)` counted beside the closed form's own
    // term: the cc pulse board's rail read 12 mA more draw than its devices
    // at 5 V from the pulse's end on. The rail's draw is its devices'.
    let mut rig = Rig::load(&cc_pulse_board(false)).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    for t in [0.0280371, 0.0310381, 0.0330371] {
        rig.advance(t).unwrap();
        let u = rig.u();
        let draw = (u - rig.v(2)) / 100.0 + (u - rig.v(3)) / 20.0;
        let seen = rig.s().rail_draw_at(t, u);
        eprintln!("at {t} s ({:?}, {:?}): the rail sees {seen:.9} A against the devices' {draw:.9}", rig.s().partition_kinds[part], rig.hub().state.reg);
        close(seen, draw, 1e-9 + 1e-6 * draw.abs(), "the rail's draw");
    }
    assert_eq!(rig.s().partition_kinds[part], PartitionKind::ClosedForm, "demoted at the pulse's end");
}

#[test]
fn a_rolled_back_cc_exit_at_an_internal_event_restores_the_partition_it_brought_there() {
    // Rails point 8: the cc pulse board, and a second pulse that enables a
    // zero-delay oscillator at the very instant the first one's end leaves
    // CC, which livelocks. Node 2's stepped copy (promoted in the stint, or
    // stepped from load under FORCE_STEPPED), brought to the exit on its CC
    // rail (`catch_up_on_cc_rail`), then demoted or re-solved in CV, is put
    // back with everything else as the twin that never took the instant
    // left it.
    let t_end = 1e-3 + (0.0 + 0.0300371);
    for force_stepped in [false, true] {
        let mut p = program(10, vec![
            psu(0, 1, 0, 5.0, 0.1),
            resistor(1, 1, 2, 100.0),
            capacitor(2, 2, 0, 1e-5, 0.0),
            resistor(3, 1, 3, 20.0),
            pulse(4, 3, 0, [5.0, 0.0, 1e-3, 0.0, 0.0, 0.0300371, 1.0], 0),
            source(5, 4, 0, 5.0),
            pulse(6, 5, 0, [0.0, 5.0, t_end, 0.0, 0.0, 1.0, 2.0], 1),
            ls00(7, 4, 0, [6, 5, 6, 0, 0, 7, 0, 0, 8, 0, 0, 9], 0.0, 2),
        ]);
        p.options.force_stepped = force_stepped;
        let (mut failed, twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
        let mut twin = twin;
        let part = failed.s().local_index[2].unwrap().0;
        failed.advance(0.0280371).unwrap();
        twin.advance(0.0280371).unwrap();
        let before = state_digest(failed.s());
        assert_eq!(before, state_digest(twin.s()), "force_stepped {force_stepped}: the twins agree before");
        assert_eq!((failed.hub().state.reg, failed.s().partition_kinds[part]), (Regime::Cc, PartitionKind::Stepped));
        let r = failed.advance(t_end + 1e-6);
        assert!(matches!(r, Err(KernelError::Livelock { at }) if at == t_end), "force_stepped {force_stepped}: the pulse's end livelocks: {r:?}");
        assert_eq!(failed.s().partition_kinds[part], PartitionKind::Stepped, "force_stepped {force_stepped}: the kind the instant found");
        assert_eq!(state_digest(failed.s()), before, "force_stepped {force_stepped}: every piece of state");
        assert_eq!(failed.k.next_event_time().to_bits(), twin.k.next_event_time().to_bits(), "force_stepped {force_stepped}: the next event");
    }
}

#[test]
fn the_crawl_hook_is_restored_when_its_guard_drops() {
    // A test that sets the trigger leaves it as it found it for the next
    // test on the same thread (`--test-threads=1`), even when it panics.
    {
        let _k = hub::crawl_hook::set(Some(0));
        assert_eq!(hub::crawl_hook::events(), 0);
        {
            let _inner = hub::crawl_hook::set(Some(7));
            assert_eq!(hub::crawl_hook::events(), 7);
        }
        assert_eq!(hub::crawl_hook::events(), 0, "the inner guard restores the outer setting");
    }
    assert_eq!(hub::crawl_hook::events(), hub::CC_CRAWL_EVENTS, "the default once the guard is gone");
    let _ = std::panic::catch_unwind(|| {
        let _k = hub::crawl_hook::set(Some(0));
        panic!("a failing test");
    });
    assert_eq!(hub::crawl_hook::events(), hub::CC_CRAWL_EVENTS, "restored through a panic");
}

#[test]
fn a_rolled_back_cc_entry_at_an_internal_event_restores_the_partition_it_brought_there() {
    // Rails point 8, ruling R66 (1): the two-capacitor cc pulse board, and a
    // second pulse that enables a zero-delay oscillator at the very instant
    // the first one's start enters CC, which livelocks. Node 2's stepped
    // partition, brought to the entry on the setpoint it stood at
    // (`SteppedSolver::catch_up`), is put back with everything else as the
    // twin that never took the instant left it.
    for force_stepped in [false, true] {
        let mut p = cc_pulse_board_two_caps(force_stepped);
        p.devices.extend([source(7, 5, 0, 5.0), pulse(8, 6, 0, [0.0, 5.0, 1e-3, 0.0, 0.0, 1.0, 2.0], 1), ls00(9, 5, 0, [7, 6, 7, 0, 0, 8, 0, 0, 9, 0, 0, 10], 0.0, 2)]);
        p.node_count = 11;
        p.digital_slot_count = 6;
        let (mut failed, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
        let part = failed.s().local_index[2].unwrap().0;
        // Both to the last event before the edge, so the instant that fails
        // is the edge's own.
        for rig in [&mut failed, &mut twin] {
            while rig.k.next_event_time() < 1e-3 {
                let t = rig.k.next_event_time();
                rig.advance(t).unwrap();
            }
        }
        assert!(failed.s().stepped.committed(part).unwrap().0 < 1e-3, "force_stepped {force_stepped}: the partition is short of the edge");
        let before = state_digest(failed.s());
        assert_eq!(before, state_digest(twin.s()), "force_stepped {force_stepped}: the twins agree before");
        assert_eq!((failed.hub().state.reg, failed.s().partition_kinds[part]), (Regime::Cv, PartitionKind::Stepped));
        let r = failed.advance(1e-3 + 1e-6);
        assert!(matches!(r, Err(KernelError::Livelock { at }) if at == 1e-3), "force_stepped {force_stepped}: the pulse's start livelocks: {r:?}");
        assert_eq!(state_digest(failed.s()), before, "force_stepped {force_stepped}: every piece of state");
        assert_eq!(failed.k.next_event_time().to_bits(), twin.k.next_event_time().to_bits(), "force_stepped {force_stepped}: the next event");
    }
}

/// tpl=5 hub=2 e=12.603 psu ilim=1.883 V2=6.64 CLK3 0.0..10.1 stacked4-3 1.2 C2-4=5.1e-3 P7-6 C2-3=1.1e-6 C3-2=1.4e-3 C7-1=1.8e-6 C0-5=1.5e-3 C1-5=7.5e-7 SW0-3(1)+R3-0=38 R1-6=15080.0 R1-3=2472.8 C5-4=2.0e-6 C1-6=1.2e-3
fn cold_574_tpl5() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![12.602831553761094, 1.8827872565849404], vec![]),
        fz(2, 1, &[2, 0], vec![6.6375304609725205], vec![]),
        fz(22, 2, &[3, 0], vec![17.297159248234106, 0.3534570618442351, 10.05567971675732, 0.0, 0.0], vec![8.0]),
        fz(2, 3, &[4, 3], vec![1.1503919030790386], vec![]),
        fz(1, 4, &[5, 0], vec![4711.643635044132], vec![]),
        fz(1, 5, &[6, 0], vec![15728.301741998961], vec![]),
        fz(1, 6, &[7, 0], vec![131284.945047971], vec![]),
        fz(3, 7, &[2, 4], vec![0.005127116066070482, 0.0, 0.0], vec![]),
        fz(13, 8, &[7, 6], vec![0.6712406185574047, 0.05017157453116978], vec![]),
        fz(3, 9, &[2, 3], vec![1.0868224423893482e-6, 0.015691113787803947, 0.0], vec![]),
        fz(3, 10, &[3, 2], vec![0.0013537362227502058, 0.0, 0.0], vec![]),
        fz(3, 11, &[7, 1], vec![1.8110415021511686e-6, 0.0, 0.0], vec![]),
        fz(3, 12, &[0, 5], vec![0.001499178943121283, 0.10317948135208961, 0.0], vec![]),
        fz(3, 13, &[1, 5], vec![7.519945258799222e-7, 0.0, 0.0], vec![]),
        fz(4, 14, &[0, 3], vec![1.0], vec![]),
        fz(1, 15, &[3, 0], vec![37.55344887494819], vec![]),
        fz(1, 16, &[1, 6], vec![15079.971435248293], vec![]),
        fz(1, 17, &[1, 3], vec![2472.761810397683], vec![]),
        fz(3, 18, &[5, 4], vec![2.025703910926431e-6, 0.0, 0.0], vec![]),
        fz(3, 19, &[1, 6], vec![0.0011840960047857835, 0.266342515322754, 0.0], vec![]),
        fz(90, 15, &[], vec![15.0, 0.05094478304263869, 37.55344887494819], vec![]),
        fz(90, 16, &[], vec![16.0, 3.336716843859078e-5, 15079.971435248293], vec![]),
        fz(90, 17, &[], vec![17.0, 0.012878156008332309, 2472.761810397683], vec![]),
    ];
    let mut p = program(8, devices);
    p.options.integration = 1;
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_partition_the_cc_entrys_catch_up_brings_to_the_entry_restarts_there() {
    // Ruling R66 (1). Cold 574 tpl 5: a CC entry at an internal event finds
    // a stepped hub partition whose pending trial ends at or before it; the
    // entry's catch-up commits that trial, and nothing planned the next, so
    // its `Step` event, still current, committed an empty trial (an index
    // panic in `step_sens`). Each partition the catch-up brings there
    // restarts at the entry, as at the exit.
    let stops = [0.006272880877141529, 0.0065927161668331785, 0.028976581614620318, 0.03553340541767844, 0.03698489478563283, 0.03901939250962618, 0.03991764293395217, 0.041271473887937844, 0.04335513173190618, 0.044337405090577084, 0.06947386681851733, 0.07123793317893146, 0.08841718232174085, 0.09182557291355894, 0.0943951507204556, 0.10422962131724346, 0.10530899188220319, 0.10542614807069964, 0.11049686736026144, 0.1445664128049566, 0.1842653151444662, 0.21422145514361743, 0.21506580900902345, 0.22663140602920362, 0.22940150856396527, 0.24083371462914918, 0.24284869237573473, 0.24320594023225206, 0.2433738608765407, 0.2530618018968624, 0.25385456921175353, 0.2547089514806105, 0.2551795896448584, 0.2553390498729086, 0.26390885075550385, 0.26597085831493583, 0.2662632641543127, 0.2690951199292167, 0.2707020403282692, 0.2724425605181134, 0.2739380883344346, 0.275636267511459, 0.2757441057945907, 0.27676400762637865, 0.27738983890471747, 0.3048682492823358, 0.3064496119541417, 0.30670732184507626, 0.30767511906567774, 0.307923329563767, 0.31650227610470705, 0.3179876336303704, 0.3180902253015938, 0.31861989438403127, 0.31880642021693695, 0.3191112189930113, 0.31923664034197746, 0.3207268780167979, 0.3455119958016315, 0.35122919965398564, 0.3527034963396514, 0.35285492273389735, 0.35387789812656595, 0.35525155709773204, 0.38344468100401163, 0.4046108419665314, 0.40531640259070945, 0.4054878471693596, 0.4055899122397323, 0.4061745636644941, 0.4160692768379551, 0.46267071196050097, 0.4638372242823546, 0.4642640595365132, 0.47179999070178674, 0.48642219398730163, 0.4907815468142124, 0.49092615333946227, 0.49108483026962063, 0.5016999107930417, 0.5018611399360423, 0.5197543654853523, 0.5357903717294508, 0.5360605955246178, 0.5361768450784467, 0.5374347983618939, 0.539529357865102, 0.5399507992463187, 0.5424389689694512, 0.5899005863701755, 0.5904900934336084, 0.5963172167538294, 0.5967046337295367, 0.6007206096537955, 0.6092686846920152, 0.644748978957318, 0.6545745069012455, 0.6604988803807474, 0.6615036525512134, 0.6616141477039709, 0.6903372916550937, 0.6904485983445836, 0.7124508598300596, 0.7128151912990386, 0.718733127888563, 0.7225306552600174, 0.7387556677500934, 0.7490736139561662, 0.7522313486629078, 0.7609420801660539, 0.7826603174339363, 0.782848177020686, 0.7829643667580773, 0.7839798744580058, 0.7864731833839732, 0.7936907989124025, 0.8019890682418442, 0.8372500697186027, 0.8819703444207859, 0.9137239136359017, 0.9155254298193632, 0.9160121321096205, 0.9162579300052882, 0.916454087304285, 0.9223516595771097, 0.934667936701408, 0.9362609527798573, 0.9452662925429353, 0.9517241413110135, 0.9522569372398517, 0.9613659576270696, 0.9617674415130101, 0.9643792673522906, 0.9951309845208647, 0.9979132929832992, 1.000725363113518, 1.0058396498610622, 1.0227008536519822, 1.0279231071210482, 1.0626795627680892, 1.0637386614818067, 1.070420631368488, 1.0705472363376314, 1.0875917492522307, 1.1187719175213746, 1.122083995447408, 1.122439804105945, 1.149227026869476, 1.149946253280575, 1.1506086266511395, 1.1510427584929321, 1.1514469826527027, 1.1873760094238954, 1.19688749429246, 1.2];
    run_cold(&cold_574_tpl5(), &[(0.10422962131724346, 14), (0.24083371462914918, 14), (0.9643792673522906, 14)], &stops).unwrap_or_else(|e| panic!("574 tpl 5: {e}"));
}

#[test]
fn a_partition_caught_up_before_the_rail_loops_early_return_restarts_there() {
    // The rail loop's early return for an alternating rail IC (W1c point 1a)
    // comes after a regime change's catch-up and before the re-solves that
    // restart what it brought to `now`: one left at `now` with its spent
    // trial's `Step` event still current committed an empty trial there
    // (cold 574 tpl 5's index panic, on a path the restart did not cover).
    // The two-capacitor cc pulse board with a 74LS00 on the rail, the
    // return forced once right after the CC entry's catch-up at 1 ms.
    let mut p = cc_pulse_board_two_caps(false);
    p.devices.push(ls00(7, 1, 0, [5, 5, 6, 0, 0, 7, 0, 0, 8, 0, 0, 9], 1e-8, 1));
    p.node_count = 10;
    p.digital_slot_count = 5;
    let mut rig = Rig::load(&p).unwrap();
    let part = rig.s().local_index[2].unwrap().0;
    rig.advance(9e-4).unwrap();
    assert_eq!((rig.hub().state.reg, rig.s().partition_kinds[part]), (Regime::Cv, PartitionKind::Stepped));
    hub::alternation_hook::arm();
    rig.advance(3e-3).unwrap_or_else(|e| panic!("{e:?}"));
    assert!(!hub::alternation_hook::take(), "the early return was taken after the entry's catch-up");
    let seen = hub::alternation_hook::seen().expect("the early return recorded the caught-up partitions");
    assert!(!seen.is_empty() && seen.iter().all(|&planned| planned), "each partition the catch-up brought to the entry has a trial planned when the early return leaves: {seen:?}");
    assert_eq!(rig.hub().state.reg, Regime::Cc);
}

// ---- ACC-E: a retry at the clamp frees a chip only under the regime rules ----

/// Cold 621 tpl 2 (hub 2): a 4.963 V PSU limited to 99.1 mA, 94.3 mA of it
/// in a 52.6 ohm resistor on the rail, and a 74ls00 on the rail whose 2y and
/// 3y drive node 3 (73 ohm from the rail) and whose 1y, LOW, sits on the rail.
/// Powered, the chip's draw puts the PSU in CC under its window; off, the PSU
/// is in CV: it starves itself and is held off at load (W1c point 1a).
fn cold_621_2() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.963186548120814, 0.09913892461723416], vec![]),
        fz(1, 1, &[2, 0], vec![179806.30875360098], vec![]),
        fz(1, 2, &[3, 0], vec![224557.2294754236], vec![]),
        fz(1, 3, &[4, 0], vec![288252.56747104926], vec![]),
        fz(50, 4, &[1, 0, 3, 4, 1, 0, 0, 3, 0, 0, 3, 0, 0, 1], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.1709540611253739e-8], vec![0.0; 13]),
        fz(1, 5, &[1, 0], vec![52.62785413717453], vec![]),
        fz(1, 6, &[3, 1], vec![125.89334044307894], vec![]),
        fz(1, 7, &[4, 2], vec![788.5087169483519], vec![]),
        fz(1, 8, &[1, 3], vec![214.47346987669798], vec![]),
        fz(1, 9, &[3, 4], vec![173.22569930665583], vec![]),
        fz(1, 10, &[0, 1], vec![54934.23864874996], vec![]),
        fz(30, 11, &[3, 4], vec![2.3804250756327083, 2.269744551659818, 0.02453319387718482, 0.0, 0.0, 0.0], vec![]),
        fz(1, 12, &[1, 3], vec![894.6488234871266], vec![]),
    ];
    Program { node_count: 5, options: Options::default(), digital_slot_count: 32, channel_slot_count: 0, devices }
}

#[test]
fn a_chip_starving_its_own_rail_stays_held_at_the_host_steps_retry_in_both_modes() {
    // The fine reference (exact mode, 1 us steps): the clamp's static retry
    // probed the chip powered with the PSU in CV, where the rail cannot
    // move, found it carried itself and freed it, and nothing ran the regime
    // rules on that solve, so the PSU delivered 126 mA in CV against its
    // 99.1 mA limit until the circuit changed; default mode keeps it held.
    let i_limit = 0.09913892461723416;
    let mut published = Vec::new();
    for p in [cold_621_2(), exact(cold_621_2())] {
        let mode = if p.options.no_rail_shortcut { "exact" } else { "default" };
        let mut rig = Rig::load(&p).unwrap();
        assert!(!rig.s().power_hold(4).is_free(), "{mode}: held off at load");
        for k in 1..=5 {
            rig.advance(k as f64 * 1e-6).unwrap();
            assert!(rig.i(0).abs() <= i_limit, "{mode}: the PSU past its limit at {k} us: {} A", rig.i(0));
            assert!(!rig.s().power_hold(4).is_free(), "{mode}: held off at {k} us");
        }
        published.push((rig.s().voltages().to_vec(), rig.i(0)));
    }
    for (n, (a, b)) in published[0].0.iter().zip(&published[1].0).enumerate() {
        close(*b, *a, 1e-4 + 1e-3 * a.abs(), &format!("node {n}, exact against default"));
    }
    close(published[1].1, published[0].1, 1e-9 + 1e-3 * published[0].1.abs(), "the PSU's current, exact against default");
}


// ---- Rails point 5: the CC operating point on a non-monotone draw (ruling G) --

/// A 74LS00 (device 2) on the rail (node 1) of a PSU `{v, i_limit}` with
/// `r_rail` from the rail to node `r_to`, its inputs `a` (1A, 1B) and 1Y on
/// node `y`, every other pin on ground (a driver on ground stamps nothing).
fn window_chip_rail(v: f64, i_limit: f64, r_rail: f64, r_to: usize, a: [usize; 2], y: usize, extra: Vec<Device>) -> Program {
    let mut devices = vec![psu(0, 1, 0, v, i_limit), resistor(1, 1, r_to, r_rail), ls00(2, 1, 0, [a[0], a[1], y, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1e-8, 0)];
    devices.extend(extra);
    program(5, devices)
}

fn exact_regime(rig: &Rig) -> Regime {
    rig.s().exact.as_ref().map_or_else(|| rig.hub().state.reg, |e| e.reg)
}

#[test]
fn a_chip_whose_draw_takes_its_rail_past_the_limit_only_inside_its_window_is_held_off_at_its_power_edge() {
    // Defect G (ruled 2026-10-31; ACC-EV's probe). PSU 5 V, 50 mA; 125 ohm
    // on the rail; a 74LS00 on the rail with 1A and 1B pulled up through 1k
    // and 100k (node 3, 4), so 1Y LOW, and 1Y on the rail itself. The draw
    // curve by hand: unpowered, u/125 and the pull-ups' 5 nA (1k into 1e9);
    // powered, inside [0.9 * 4.75, 1.1 * 5.25] = [4.275, 5.775] V, 1Y LOW
    // sinks u / R_out with R_out = u / (4 ioMax), 4 * 8 mA = 32 mA more. At
    // the setpoint 40 + 32 = 72 mA, past the limit: CC. Coming down from 5 V
    // the powered draw u/125 + 32 mA stays above 66.2 mA to the window's
    // floor, and under it the unpowered draw is at most 34.2 mA: there is
    // no CC operating point at or below the setpoint, the draw crossing the
    // limit only across the chip's power edge, which is the power hold's
    // case. The chip held off, the PSU is in CV at 5 V delivering 5/125 +
    // 5/(1e9 + 1e3) = 40.000005 mA (the node shunts' 15 pA aside). The CC
    // Newton's root past the setpoint, unpowered at 6.25 V above the
    // window, is no operating point of a supply coming down from its
    // setpoint: before the fix both modes held CV there at 72 mA.
    let board = || window_chip_rail(5.0, 0.05, 125.0, 0, [3, 4], 1, vec![resistor(3, 1, 3, 1e3), resistor(4, 1, 4, 1e5), resistor(5, 3, 0, 1e9)]);
    let want = 5.0 / 125.0 + 5.0 / (1e9 + 1e3);
    for (mode, p) in [("default", board()), ("exact", exact(board()))] {
        let mut rig = Rig::load(&p).unwrap();
        for k in 0..=3 {
            if k > 0 {
                rig.advance(k as f64 * 1e-3).unwrap();
            }
            let at = format!("{mode} at {k} ms");
            close(rig.v(1), 5.0, 1e-12, &format!("{at}: the rail at the setpoint"));
            close(rig.i(0), -want, 1e-10, &format!("{at}: the PSU's current"));
            assert_eq!(exact_regime(&rig), Regime::Cv, "{at}");
            assert!(!rig.s().power_hold(2).is_free(), "{at}: the chip is held off");
        }
    }
}

#[test]
fn a_cc_entry_takes_the_highest_operating_point_at_or_below_the_setpoint() {
    // Ruling G's two-root case. PSU 5.7 V, 80 mA; 50 ohm from the rail to a
    // 0 / 5.7 V clock (node 3); a 74LS00 on the rail with 1A and 1B on
    // ground, so 1Y HIGH, and 1Y held at Vs = 5.10625 V by an ideal source
    // (node 2). 1Y's HIGH stage, R_out = u / (4 ioMax) = u / 0.032, draws
    // 0.032 (u - Vs) / u from the rail while the chip is powered, inside
    // [4.275, 5.775] V. Clock high, the 50 ohm carries nothing at the
    // setpoint and the stage 3.3 mA: CV, the chip powered and 1Y HIGH.
    // Clock low, by hand: 114 + 3.3 mA at the setpoint, past the limit: CC.
    // Coming down from 5.7 V, powered, u/50 + 0.032 - 0.1634/u = 0.08
    // gives u^2 - 2.4 u - 8.17 = 0, u = 1.2 + sqrt(9.61) = 4.3 V, inside
    // the window: the first root reached, the operating point. Unpowered
    // under the window the draw is u/50, a second root at 0.08 * 50 = 4 V,
    // consistent too. The powered draw is concave, so the CC Newton from
    // the setpoint steps past its root, to 4.208 V, under the window, and
    // from there reaches 4 V: before the fix both modes settled there.
    // Clock high again, the CC solution is past the setpoint: CV.
    // The chip takes digital slots 0-3, the clock 4.
    let mut ck = clock(4, 3, 0, 1000.0, 0.5, 4);
    ck.params[2] = 5.7;
    let board = || window_chip_rail(5.7, 0.08, 50.0, 3, [0, 0], 2, vec![source(3, 2, 0, 5.10625), ck.clone()]);
    let want = 4.3;
    for (mode, p) in [("default", board()), ("exact", exact(board()))] {
        let mut rig = Rig::load(&p).unwrap();
        let (mut low, mut high) = (0, 0);
        for k in 1..=12 {
            let t = k as f64 * 0.25e-3 + 0.1e-3;
            rig.advance(t).unwrap();
            let at = format!("{mode} at {:.2} ms, clock {}", t * 1e3, rig.v(3));
            if rig.v(3) < 1.0 {
                low += 1;
                close(rig.v(1), want, 1e-9, &format!("{at}: the rail at the highest root"));
                close(rig.i(0), -0.08, 1e-10, &format!("{at}: the PSU at its limit"));
                assert_eq!(exact_regime(&rig), Regime::Cc, "{at}");
                assert!(digital::power::powered(&rig.s().ics[0].1, rig.v(1), rig.v(0)), "{at}: the chip is powered there");
            } else {
                high += 1;
                close(rig.v(1), 5.7, 1e-12, &format!("{at}: CV at the setpoint"));
                assert_eq!(exact_regime(&rig), Regime::Cv, "{at}");
            }
        }
        assert!(low >= 4 && high >= 4, "{mode}: both phases sampled");
    }
}

#[test]
fn a_cc_rail_below_a_chips_window_stays_in_cc_when_the_chip_would_feed_it_at_the_setpoint() {
    // Defect C (cold 364 tpl 2). PSU 5 V, 40 mA; 100 ohm on the rail; a
    // 74LS00 on the rail with 1A and 1B on ground, so 1Y HIGH, and 1Y on a
    // 0 / 10 V clock. By hand, clock low: powered at the setpoint 1Y's HIGH
    // stage sinks u / R_out = 4 ioMax = 32 mA into the clock, 82 mA in all:
    // CC. Coming down, the powered draw u/100 + 32 mA is past the limit
    // through the window and the unpowered 42.75 mA still is at its floor
    // (4.275 V); the unpowered root is 0.04 * 100 = 4 V, under the window:
    // CC at 4 V, the chip off. Clock high, from there: the unpowered draw
    // is unchanged, so the CC rail stays at 4 V, the chip off. A CV state
    // exists too (at 5 V, powered, 1Y feeds the rail 0.032 (10 - 5)/5 = 32
    // mA, an 18 mA draw within the limit), but the CC rail never reaches
    // the window to power the chip: the CC solution is short of the
    // setpoint. Before the fix the default exit read the draw at the
    // setpoint, the powered chip's 18 mA, and left CC at each clock-high
    // phase; exact mode, the rail test alone, stayed.
    // The chip takes digital slots 0-3, the clock 4.
    let mut ck = clock(3, 2, 0, 1000.0, 0.5, 4);
    ck.params[2] = 10.0;
    let board = || window_chip_rail(5.0, 0.04, 100.0, 0, [0, 0], 2, vec![ck.clone()]);
    for (mode, p) in [("default", board()), ("exact", exact(board()))] {
        let mut rig = Rig::load(&p).unwrap();
        let (mut low_seen, mut high_after_low) = (false, 0);
        for k in 1..=16 {
            rig.advance(k as f64 * 0.25e-3 + 0.1e-3).unwrap();
            let at = format!("{mode} at {:.2} ms, clock {}", (k as f64 * 0.25e-3 + 0.1e-3) * 1e3, rig.v(2));
            if rig.v(2) < 1.0 {
                low_seen = true;
            }
            if !low_seen {
                continue;
            }
            if rig.v(2) > 9.0 {
                high_after_low += 1;
            }
            close(rig.v(1), 4.0, 1e-9, &format!("{at}: the CC rail at the unpowered root"));
            close(rig.i(0), -0.04, 1e-10, &format!("{at}: the PSU at its limit"));
            assert_eq!(exact_regime(&rig), Regime::Cc, "{at}");
        }
        assert!(low_seen && high_after_low >= 4, "{mode}: both phases sampled ({high_after_low} high after a low)");
    }
}

#[test]
fn a_cc_exit_whose_cv_draw_is_past_the_limit_enters_again_at_once() {
    // Cold 267 tpl 2's exit, minimal. PSU 5 V, 50 mA; 50 ohm from the rail
    // to a 0 / 5 V clock (node 2); a 74LS00 on the rail with 1A, 1B, 2A, 2B
    // on the rail, so 1Y and 2Y LOW, both on the rail: powered, inside
    // [4.275, 5.775] V, each sinks u / R_out = 4 ioMax = 32 mA. By hand,
    // clock low: 100 + 64 mA at the setpoint: CC; coming down, the powered
    // draw u/50 + 64 mA is past the limit through the window and the
    // unpowered u/50 still is at its floor (85.5 mA), so the root is the
    // unpowered 0.05 * 50 = 2.5 V: CC there, the chip off. Clock high, from
    // there: unpowered, (u - 5)/50 = 0.05 puts the CC rail at 7.5 V, past
    // the setpoint: CV, where the powered chip draws 64 mA, past the limit,
    // so CC again at the same instant, coming down from 5 V: powered,
    // (u - 5)/50 + 0.064 = 0.05 at u = 4.3 V, inside the window: CC at
    // 4.3 V, the chip powered. Before the fix exact mode stayed in CV at
    // 64 mA through the clock-high phase (nothing re-solved its rail), and
    // default mode, its loop settled after the exit, did too. Exact mode
    // lands 1.55 mV high (a LOW stage sinking from its own supply, its
    // R_out moving with the iterate), within W0, which is the tolerance.
    // The chip takes digital slots 0-3, the clock 4.
    let w0 = |v: f64| 1e-4 + 1e-3 * v;
    let mut ck = clock(3, 2, 0, 1000.0, 0.5, 4);
    ck.params[2] = 5.0;
    let board = || {
        let devices = vec![psu(0, 1, 0, 5.0, 0.05), resistor(1, 1, 2, 50.0), ls00(2, 1, 0, [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0], 1e-8, 0), ck.clone()];
        program(3, devices)
    };
    for (mode, p) in [("default", board()), ("exact", exact(board()))] {
        let mut rig = Rig::load(&p).unwrap();
        let (mut low, mut high) = (0, 0);
        for k in 1..=12 {
            let t = k as f64 * 0.25e-3 + 0.1e-3;
            rig.advance(t).unwrap();
            let at = format!("{mode} at {:.2} ms, clock {}", t * 1e3, rig.v(2));
            let on = digital::power::powered(&rig.s().ics[0].1, rig.v(1), rig.v(0));
            if rig.v(2) < 1.0 {
                low += 1;
                close(rig.v(1), 2.5, w0(2.5), &format!("{at}: the unpowered CC root"));
                assert!(!on, "{at}: the chip off");
            } else {
                high += 1;
                close(rig.v(1), 4.3, w0(4.3), &format!("{at}: the powered CC root"));
                assert!(on, "{at}: the chip powered");
            }
            close(rig.i(0), -0.05, 1e-10, &format!("{at}: the PSU at its limit"));
            assert_eq!(exact_regime(&rig), Regime::Cc, "{at}");
        }
        assert!(low >= 4 && high >= 4, "{mode}: both phases sampled");
    }
}

#[test]
fn a_landing_holds_a_chip_whose_other_pins_sit_on_the_rail_or_fixed_nodes() {
    // The verifier's repros of ruling G's landing (2026-11-01). Each chip's
    // pins are all on the rail or a fixed node, so its hold dirties no
    // partition. By hand, as in the probe: powered, inside [4.275, 5.775]
    // V, each LOW output on the rail sinks u / R_out = 4 ioMax = 32 mA.
    // (a) PSU 5 V, 50 mA, 125 ohm on the rail, 1A and 1B on the rail, 1Y on
    // it: 40 + 32 = 72 mA at the setpoint, at least 66.2 mA through the
    // window, at most 34.2 mA under it: the landing, the chip held off, CV
    // at 5 V delivering 5/125 = 40 mA. (b) The same with 1A and 1B on a 5 V
    // source. (c) PSU 5 V, 45 mA, 50 ohm from the rail to a 5 V source, 1A,
    // 1B, 1Y, 2A, 2B, 2Y on the rail: 64 mA at the setpoint, (u - 5)/50 +
    // 64 mA >= 49.5 mA through the window, below it (u - 5)/50 < 0: held
    // off, CV at 5 V, nothing delivered (the node shunt's 5 pA). Before the
    // fix default mode stayed CV at 72, 72 and 64 mA, its chip free.
    let boards: [(&str, fn() -> Program, f64); 3] = [
        ("(a)", || window_chip_rail(5.0, 0.05, 125.0, 0, [1, 1], 1, vec![]), 5.0 / 125.0),
        ("(b)", || window_chip_rail(5.0, 0.05, 125.0, 0, [3, 3], 1, vec![source(3, 3, 0, 5.0)]), 5.0 / 125.0),
        ("(c)", || window_chip_rail(5.0, 0.045, 50.0, 2, [1, 1], 1, vec![source(3, 2, 0, 5.0)]), 0.0),
    ];
    for (name, board, want) in boards {
        // (c) has 2A, 2B, 2Y on the rail too.
        let board = || {
            let mut p = board();
            if name == "(c)" {
                p.devices[2] = ls00(2, 1, 0, [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0], 1e-8, 0);
            }
            p
        };
        for (mode, p) in [("default", board()), ("exact", exact(board()))] {
            let mut rig = Rig::load(&p).unwrap();
            for k in 0..=3 {
                if k > 0 {
                    rig.advance(k as f64 * 1e-3).unwrap();
                }
                let at = format!("{name} {mode} at {k} ms");
                close(rig.v(1), 5.0, 1e-12, &format!("{at}: the rail at the setpoint"));
                close(rig.i(0), -want, 1e-10, &format!("{at}: the PSU's current"));
                assert_eq!(exact_regime(&rig), Regime::Cv, "{at}");
                assert!(!rig.s().power_hold(2).is_free(), "{at}: the chip is held off");
            }
        }
    }
}

#[test]
fn a_cc_exit_that_lands_on_a_power_edge_solves_the_rail_with_the_chip_held() {
    // The verifier's clock repro. PSU 5 V, 45 mA; 50 ohm from the rail to a
    // 0 / 5 V clock (node 2); a 74LS00 with 1A, 1B, 1Y, 2A, 2B, 2Y on the
    // rail (two LOW outputs sinking 32 mA each while powered, inside [4.275,
    // 5.775] V). By hand, clock low: 100 + 64 mA at the setpoint: CC;
    // powered u/50 + 64 mA is past the limit through the window, unpowered
    // 85.5 mA at its floor, so the root is the unpowered 0.045 * 50 = 2.25
    // V, the chip off. Clock high: unpowered, the CC rail (u - 5)/50 = 0.045
    // is 7.25 V, past the setpoint; at the setpoint the powered chip draws
    // 64 mA, past the limit; coming down, (u - 5)/50 + 64 mA >= 49.5 mA
    // through the window and (u - 5)/50 < 0 under it: a landing, the chip
    // held off, CV at 5 V with nothing delivered. Before the fix default
    // mode published the CC rail at 2.25 V with 45 mA from the PSU while
    // the resistor fed 55 mA into the rail.
    // The chip takes digital slots 0-3, the clock 4.
    let mut ck = clock(3, 2, 0, 1000.0, 0.5, 4);
    ck.params[2] = 5.0;
    let board = || program(3, vec![psu(0, 1, 0, 5.0, 0.045), resistor(1, 1, 2, 50.0), ls00(2, 1, 0, [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0], 1e-8, 0), ck.clone()]);
    for (mode, p) in [("default", board()), ("exact", exact(board()))] {
        let mut rig = Rig::load(&p).unwrap();
        let (mut low, mut high) = (0, 0);
        for k in 1..=12 {
            let t = k as f64 * 0.25e-3 + 0.1e-3;
            rig.advance(t).unwrap();
            let at = format!("{mode} at {:.2} ms, clock {}", t * 1e3, rig.v(2));
            // The rail's KCL: what the PSU delivers is what the resistor
            // takes, the chip off either way (the node shunt aside).
            close(-rig.i(0), rig.i(1), 1e-10, &format!("{at}: KCL at the rail"));
            assert!(!digital::power::powered(&rig.s().ics[0].1, rig.v(1), rig.v(0)) || !rig.s().power_hold(2).is_free(), "{at}: the chip off or held");
            if rig.v(2) < 1.0 {
                low += 1;
                close(rig.v(1), 2.25, 1e-9, &format!("{at}: the unpowered CC root"));
                close(rig.i(0), -0.045, 1e-10, &format!("{at}: the PSU at its limit"));
                assert_eq!(exact_regime(&rig), Regime::Cc, "{at}");
            } else {
                high += 1;
                close(rig.v(1), 5.0, 1e-12, &format!("{at}: CV at the setpoint"));
                close(rig.i(0), 0.0, 1e-10, &format!("{at}: nothing delivered"));
                assert_eq!(exact_regime(&rig), Regime::Cv, "{at}");
                assert!(!rig.s().power_hold(2).is_free(), "{at}: the chip held off");
            }
        }
        assert!(low >= 4 && high >= 4, "{mode}: both phases sampled");
    }
}

#[test]
fn a_power_edge_at_the_setpoint_is_a_landing() {
    // A PSU whose setpoint is the 74LS00's window floor, 0.9 * 4.75 V (the
    // edge computed as the window test computes it), 50 mA; 125 ohm on the
    // rail; 1A, 1B pulled up (1k, 100k), 1Y on the rail. By hand: at the
    // setpoint the chip is powered (the floor is inside the window), u/125
    // + 32 mA = 66.2 mA, past the limit; any rail below it is under the
    // floor, the chip off, u/125 <= 34.2 mA: the draw crosses the limit at
    // the setpoint itself, a landing: the chip held off, CV at the setpoint
    // delivering v/125 + v/(1e9 + 1e3). Before the fix the edge at the
    // setpoint was dropped and default mode settled in CC a few 1e-13 V
    // under it.
    let v = 4.75 * 0.9;
    let board = || window_chip_rail(v, 0.05, 125.0, 0, [3, 4], 1, vec![resistor(3, 1, 3, 1e3), resistor(4, 1, 4, 1e5), resistor(5, 3, 0, 1e9)]);
    let want = v / 125.0 + v / (1e9 + 1e3);
    for (mode, p) in [("default", board()), ("exact", exact(board()))] {
        let mut rig = Rig::load(&p).unwrap();
        for k in 0..=2 {
            if k > 0 {
                rig.advance(k as f64 * 1e-3).unwrap();
            }
            let at = format!("{mode} at {k} ms");
            close(rig.v(1), v, 1e-12, &format!("{at}: the rail at the setpoint"));
            close(rig.i(0), -want, 1e-10, &format!("{at}: the PSU's current"));
            assert_eq!(exact_regime(&rig), Regime::Cv, "{at}");
            assert!(!rig.s().power_hold(2).is_free(), "{at}: the chip is held off");
        }
    }
}

// ---- R68: stepped hub partitions on a CC rail with a rail capacitor ------

/// Every node voltage of `p` at each of `samples`, the host advancing in
/// slices of at most `slice` (0: straight to each sample) and setting each
/// `(t, device, value)` of `toggles` once it reaches `t` (each toggle time
/// is a host stop).
fn sampled_toggling(p: &Program, samples: &[f64], slice: f64, toggles: &[(f64, u32, f64)]) -> Vec<Vec<f64>> {
    let mut e = Engine::load(p).unwrap_or_else(|err| panic!("load: {} {}", err.code, err.message));
    let mut out = Vec::new();
    let (mut t, mut ev) = (0.0f64, 0usize);
    for &s in samples {
        while t < s {
            let mut next = if slice > 0.0 { (t + slice).min(s) } else { s };
            if ev < toggles.len() && toggles[ev].0 <= next {
                next = toggles[ev].0;
            }
            e.advance(next).unwrap_or_else(|err| panic!("{} at {next}: {}", err.code, err.message));
            t = next;
            while ev < toggles.len() && toggles[ev].0 <= t {
                e.set_params(&[(toggles[ev].1, 0, toggles[ev].2)]).unwrap_or_else(|err| panic!("toggle at {t}: {} {}", err.code, err.message));
                ev += 1;
            }
        }
        out.push(e.voltages().to_vec());
    }
    out
}

/// Ruling R68's board, after the cold fuzz's seed 347 tpl 5: a {3.08 V,
/// 332 mA} PSU with a 1.75 uF rail capacitor (esr 4.74 ohm); one stepped hub
/// partition of 562 uF on node 3 (switch 7 to the rail, switch 8 to ground)
/// and 12.6 mF from node 4 to the rail, a second of 0.67 uF (esr 19 mohm)
/// and 1.59 uF from node 6 to the rail, with 2.7 kohm to ground.
fn railcap_two_stepped() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![3.082145317580972, 0.3322620969472558], vec![]),
        fz(3, 1, &[0, 1], vec![1.7542530827047273e-6, 4.737497116656414, 0.0], vec![]),
        fz(3, 2, &[0, 3], vec![0.0005624182743411655, 0.0, 0.0], vec![]),
        fz(1, 3, &[3, 0], vec![11.623632299319503], vec![]),
        fz(3, 4, &[4, 1], vec![0.012643318391908064, 0.2512880456384814, 0.0], vec![]),
        fz(1, 5, &[3, 4], vec![3.756401430443622], vec![]),
        fz(1, 6, &[4, 0], vec![437778.79028702184], vec![]),
        fz(4, 7, &[1, 3], vec![1.0], vec![]),
        fz(4, 8, &[0, 3], vec![0.0], vec![]),
        fz(3, 9, &[6, 1], vec![6.653681533285178e-7, 0.01866792533169424, 0.0], vec![]),
        fz(3, 10, &[6, 1], vec![1.5923609781871292e-6, 4.227183789546719, 0.0], vec![]),
        fz(1, 11, &[6, 0], vec![2685.7512596001907], vec![]),
    ]
}

/// [`railcap_two_stepped`] under the trapezoid, with `one`: node 6's
/// capacitors to ground, so one stepped hub partition is on the rail.
fn railcap_board(one: bool) -> Program {
    let mut devices = railcap_two_stepped();
    if one {
        devices[9].pins[1] = pin(0);
        devices[10].pins[1] = pin(0);
    }
    let mut p = program(7, devices);
    p.options.integration = 1;
    p
}

/// [`railcap_board`]'s host: the start-up inrush in CC, switch 7 opening at
/// 25 ms, and switch 8 shorting node 3's 562 uF at 30.0001 ms, which drags
/// the rail into CC through the 12.6 mF.
const RAILCAP_SAMPLES: [f64; 14] = [0.002, 0.004, 0.006, 0.01, 0.02, 0.03, 0.0300004, 0.030001, 0.03001, 0.0301, 0.031, 0.035, 0.04, 0.05];
const RAILCAP_TOGGLES: [(f64, u32, f64); 2] = [(0.025, 7, 0.0), (0.0300001, 8, 1.0)];

/// The worst node's distance from the fine reference (exact mode in 1 us
/// host slices) at each of [`RAILCAP_SAMPLES`], default mode advanced
/// straight to each.
fn railcap_offsets(one: bool) -> Vec<f64> {
    let want = sampled_toggling(&exact(railcap_board(one)), &RAILCAP_SAMPLES, 1e-6, &RAILCAP_TOGGLES);
    let got = sampled_toggling(&railcap_board(one), &RAILCAP_SAMPLES, 0.0, &RAILCAP_TOGGLES);
    (0..RAILCAP_SAMPLES.len()).map(|k| worst_w0(&got[k..=k], &want[k..=k])).collect()
}

#[test]
fn a_stepped_hub_partition_steps_a_cc_rail_capacitor_with_it() {
    // Ruling R68 (Rails point 4, the CC step). With a rail capacitor the CC
    // step's rail was a boundary, the capacitors' segment, and the stepped
    // partition's draw was held on it linearised: 562 uF on a 1 mohm contact
    // and 12.6 mF from the rail stepped against a rail that moved with them.
    // Stepping the capacitors with the partition, the rail one of its
    // unknowns, default mode keeps to the fine reference as exact mode does
    // (within 0.4 W0 at every sample).
    let offs = railcap_offsets(true);
    for (k, &t) in RAILCAP_SAMPLES.iter().enumerate() {
        assert!(offs[k] <= 1.0, "at {t} s: {:.2} W0 off the fine reference (all: {offs:.2?})", offs[k]);
    }
}

#[test]
fn two_stepped_hub_partitions_on_a_cc_rail_capacitor_keep_near_the_fine_reference() {
    // Ruling R68: cold 347 tpl 5's shape, both stepped partitions on a CC
    // rail with a capacitor. Each held the other's and its own draw
    // linearised on the capacitors' segment: thousands of W0 off the fine
    // reference in the start-up inrush and hundreds after the switch's CC
    // entry. Each now steps the capacitors with it, the other's draw held
    // where it last committed, which is the coupling error of two stepped
    // partitions on one CC rail (rulings R32, R37, R39; perf round 3), up to
    // 21 W0 a microsecond after the short, where exact mode is within 0.8.
    let offs = railcap_offsets(false);
    for (k, &t) in RAILCAP_SAMPLES.iter().enumerate() {
        let bound = if t < 0.03 { 5.0 } else { 25.0 };
        assert!(offs[k] <= bound, "at {t} s: {:.2} W0 off the fine reference, over {bound} (all: {offs:.2?})", offs[k]);
    }
}

#[test]
fn a_rolled_back_instant_restores_the_cc_rail_capacitors_charge_a_stepped_commit_hands_over() {
    // Ruling R68, Rails point 8. In the start-up inrush in CC, a horizon
    // clamp brings node 3's stepped partition through steps that carry the
    // rail capacitors with it, each commit keeping their charge and handing
    // the latest to the rail solve. A rollback puts each partition's
    // committed charge and the hand-off back with everything else, and the
    // run goes on as a twin's that never took the window.
    let p = railcap_board(false);
    let (mut rig, mut twin) = (Rig::load(&p).unwrap(), Rig::load(&p).unwrap());
    let part = rig.s().local_index[3].unwrap().0;
    rig.advance(2e-3).unwrap();
    twin.advance(2e-3).unwrap();
    assert_eq!(rig.hub().state.reg, Regime::Cc, "the inrush is in CC");
    let before = state_digest(rig.s());
    let charge_before = rig.s().stepped.debug_rail_charge(part);
    let s = rig.k.solver_mut();
    let token = s.begin_instant();
    s.sync_dynamic_to(2.5e-3).unwrap();
    let during = s.stepped.debug_rail_charge(part);
    assert!(during.1.is_some_and(|(t, _, _)| t == 2.5e-3), "the clamp's last commit keeps the charge: {during:?}");
    assert_ne!(during, charge_before, "the window moved the charges");
    s.rollback_instant(token);
    assert_eq!(rig.s().stepped.debug_rail_charge(part), charge_before, "the charges");
    assert_eq!(state_digest(rig.s()), before, "every piece of state");
    rig.advance(4e-3).unwrap();
    twin.advance(4e-3).unwrap();
    assert_eq!(state_digest(rig.s()), state_digest(twin.s()), "it goes on like the twin");
}

#[test]
fn a_rail_ic_at_its_power_edge_on_a_cc_rail_capacitor_is_held_as_on_the_segment() {
    // Ruling R68, the cold fuzz's 216 tpl 5: a {13.0 V, 27 mA} PSU in CC
    // ramps 30 uF (esr 0) toward a 74LS00's 4.275 V floor; the chip's output
    // node, 1.7 mF, is a stepped hub partition stepping the rail capacitor
    // with it. At the floor the chip, powered, pulls the rail back under it.
    // A re-solve's catch-up judged the chip's power at the rail published
    // after the falling edge that ended its step, so the step carried the
    // rail back over the edge, and the chip flipped at every instant 1e-14 s
    // apart, never twice within one: 3.2M steps for 7 ms, no hold. Caught up
    // on the rail its trial was solved on, the passes alternate at the edge
    // and the chip is held off there, as on the segment before R68 (base:
    // one hold at 5.32 ms, 2,040 steps to 6.3 ms).
    let mut p = program(8, cold_216_tpl5());
    p.digital_slot_count = 32;
    let mut e = Engine::load(&p).unwrap();
    let mut t = 0.0f64;
    while t < 7.3e-3 {
        t = (t + 1e-4).min(7.3e-3);
        e.advance(t).unwrap_or_else(|er| panic!("advance to {t}: {} {}", er.code, er.message));
        assert!(e.stat(10) < 20_000.0, "{} steps by {t} s: the chip chatters at its floor", e.stat(10));
    }
    assert!(e.stat(25) >= 1.0, "the chip held off at its floor");
}

#[test]
fn a_cc_step_on_a_rail_capacitor_is_cut_at_a_rail_ics_power_edge() {
    // Ruling R68, the cold fuzz's 505 tpl 5: a {13.4 V, 311 mA} PSU in CC
    // with 0.4 uF of rail capacitor (esr 0.145 ohm) and a 74LS00 on the rail;
    // a stepped LED and capacitor partition steps the capacitor with it.
    // Its gate judged at the step's start rail, one 2.6 ms step carried the
    // rail from 3.94 V to 4.41 V, 0.13 V past the chip's 4.275 V floor; the
    // powered chip then pulled the rail back at 3e5 V/s, and the rail Newton
    // failed at the floor with no verdict alternating (`-4` at 0.3919 s).
    // The step is cut at the floor, as at the setpoint, so the chip's power
    // flips at that commit's rail solve, and the run goes on.
    let p = { let mut p = program(5, cold_505_tpl5()); p.digital_slot_count = 32; p };
    let stops: [f64; STOPS_505.len()] = STOPS_505;
    let mut e = Engine::load(&p).unwrap();
    for &t in &stops {
        e.advance(t).unwrap_or_else(|er| panic!("advance to {t}: {} at {}: {}", er.code, e.time(), er.message));
    }
}

const STOPS_505: [f64; 60] = [0.006272879571408905, 0.0072785054032582005, 0.017876457538686583, 0.018062886736138067, 0.018340502711687582, 0.02336229275905534, 0.03731104997307824, 0.037618554404920436, 0.038439311441490744, 0.03908783214516494, 0.039386454964519674, 0.04715109307293887, 0.056699242152804606, 0.056824490802280254, 0.05888527331802057, 0.059124089362100486, 0.05928655907634512, 0.0987567251478853, 0.11209473731118556, 0.1566669504946821, 0.1575207447526295, 0.18580802788194342, 0.18951967976761797, 0.19077110226964777, 0.1916870068860743, 0.20621913239321335, 0.20691562034439767, 0.20727761824740132, 0.20741492218813548, 0.20752859482385222, 0.20779185173384274, 0.22299673055917238, 0.2236406810111467, 0.22947657715238745, 0.23198271164301645, 0.23341168368260118, 0.2343051786433487, 0.2516176401706743, 0.25616393068783705, 0.2933665858089527, 0.3276966916619526, 0.3278697129857167, 0.32808620367788355, 0.3284254096619142, 0.33115954603093894, 0.33949753737674004, 0.37231679693597775, 0.37399489161947114, 0.3741727172839677, 0.3761215318740903, 0.3773743730054749, 0.37767444041233944, 0.4172932478090943, 0.4278969012092656, 0.42841611805988244, 0.42883785451457257, 0.43275199815425797, 0.43332230116377113, 0.43353983556620085, 0.44447736049582365];

/// tpl=5 hub=2 e=13.022 psu ilim=0.027 V2=-7.43 PULSE3 -3.0..4.2 tr=2e-3 stacked4-3 2.5 R5-2=6113.5 R0-3=1.2 C0-5=1.7e-3 C1-0=3.0e-5 74LS00 vcc1 gnd0 in5,3 out5,0 R1-6=33.5 R3-4=28.8 R6-2=39578.2 SW4-5(1)+R5-0=140 span=20.455439953003236
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 8
/// toggles [(0.8599502792753119, 20)]
fn cold_216_tpl5() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![13.022448609674765, 0.026969676605768433], vec![]),
        fz(2, 1, &[2, 0], vec![-7.432991343328469], vec![]),
        fz(24, 2, &[3, 0], vec![-2.968349505555712, 4.228886889594543, 0.02847267005934285, 0.0017746688539685936, 0.0017746688539685936, 0.036700583859148636, 0.06301243758242027], vec![8.0]),
        fz(2, 3, &[4, 3], vec![2.5210816149320805], vec![]),
        fz(1, 4, &[5, 0], vec![10422.235275104867], vec![]),
        fz(1, 5, &[6, 0], vec![5089.06296809635], vec![]),
        fz(1, 6, &[7, 0], vec![105125.12983996692], vec![]),
        fz(1, 7, &[5, 2], vec![6113.542508018208], vec![]),
        fz(1, 8, &[0, 3], vec![1.2175029497422107], vec![]),
        fz(3, 9, &[0, 5], vec![0.001675603874519194, 0.0, 0.0], vec![]),
        fz(3, 10, &[1, 0], vec![3.0143493907957308e-5, 0.0, 0.0], vec![]),
        fz(50, 11, &[1, 0, 5, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 5], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 2.472812137154975e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 12, &[5, 1], vec![2938.006353196718], vec![]),
        fz(1, 13, &[0, 6], vec![1988.358426732496], vec![]),
        fz(1, 14, &[0, 1], vec![37327.46685158523], vec![]),
        fz(1, 15, &[1, 6], vec![33.50806366250361], vec![]),
        fz(1, 16, &[3, 4], vec![28.75253917551141], vec![]),
        fz(1, 17, &[7, 1], vec![358.9163713879824], vec![]),
        fz(1, 18, &[4, 3], vec![1049.0389364685514], vec![]),
        fz(1, 19, &[6, 2], vec![39578.18287361168], vec![]),
        fz(4, 20, &[4, 5], vec![1.0], vec![]),
        fz(1, 21, &[5, 0], vec![139.64008852306267], vec![]),
        fz(90, 7, &[], vec![7.0, 0.0008244001389211244, 6113.542508018208], vec![]),
        fz(90, 8, &[], vec![8.0, 14.775985163116061, 1.2175029497422107], vec![]),
        fz(90, 12, &[], vec![12.0, 0.0017929403182408447, 2938.006353196718], vec![]),
        fz(90, 13, &[], vec![13.0, 0.14420072311943027, 1988.358426732496], vec![]),
        fz(90, 15, &[], vec![15.0, 0.8942729157353687, 33.50806366250361], vec![]),
        fz(90, 16, &[], vec![16.0, 1.0757562946313903, 28.75253917551141], vec![]),
        fz(90, 19, &[], vec![19.0, 0.0009110801375696161, 39578.18287361168], vec![]),
        fz(90, 21, &[], vec![21.0, 0.17020571822982822, 139.64008852306267], vec![]),
    ]
}

/// tpl=5 hub=2 e=13.403 psu ilim=0.311 V2=7.97 C1-0=4.0e-7 R3-0=1.2 74LS00 vcc1 gnd0 in1,2 out3,2 LED4-0+R1-4=43 C4-1=2.8e-7 P1-4 C4-0=1.7e-3 C1-4=1.6e-6 C0-2=2.4e-7 span=21.36939837303977
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 5
/// toggles []
fn cold_505_tpl5() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![13.402957769250968, 0.31102727029811983], vec![]),
        fz(2, 1, &[0, 2], vec![7.966440603788801], vec![]),
        fz(1, 2, &[3, 0], vec![45603.34004136992], vec![]),
        fz(1, 3, &[4, 0], vec![671357.3964771514], vec![]),
        fz(3, 4, &[1, 0], vec![4.031401863880018e-7, 0.1448226438041977, 0.0], vec![]),
        fz(1, 5, &[3, 0], vec![1.1529353501851427], vec![]),
        fz(50, 6, &[1, 0, 1, 2, 3, 0, 0, 2, 0, 0, 2, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 6.292790469859957e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 7, &[3, 2], vec![829.6892202896048], vec![]),
        fz(1, 8, &[2, 3], vec![1101.1836187547399], vec![]),
        fz(30, 9, &[4, 0], vec![3.3292130156609434, 1.7720828664490131, 0.03807182756102721, 0.0, 0.0, 0.0], vec![]),
        fz(1, 10, &[1, 4], vec![43.49417018480507], vec![]),
        fz(3, 11, &[4, 1], vec![2.7883493113459534e-7, 0.0, 0.0], vec![]),
        fz(13, 12, &[1, 4], vec![0.1498781000777963, 0.7618258418259335], vec![]),
        fz(3, 13, &[4, 0], vec![0.0017422766538027425, 0.0, 0.0], vec![]),
        fz(3, 14, &[1, 4], vec![1.6422396677494e-6, 0.0, 0.0], vec![]),
        fz(3, 15, &[0, 2], vec![2.3732978532103394e-7, 0.0, 0.0], vec![]),
        fz(90, 5, &[], vec![5.0, 4.770819857641739, 1.1529353501851427], vec![]),
        fz(90, 7, &[], vec![7.0, 0.015483497912294173, 829.6892202896048], vec![]),
        fz(90, 8, &[], vec![8.0, 0.10740117494282157, 1101.1836187547399], vec![]),
        fz(90, 10, &[], vec![10.0, 0.08348089265895527, 43.49417018480507], vec![]),
    ]
}

#[test]
fn a_rail_capacitor_loop_through_a_stepped_partition_keeps_the_rail_on_its_switch() {
    // Ruling R68, the cold fuzz's 581 tpl 5: a {12.35 V, 35 mA} PSU in CC
    // whose rail a closed switch (1 mohm) ties to a fixed -10.69 V node, 2.3
    // mF across that switch and 22 uF to ground; a stepped partition hangs
    // 7.6 mF from the rail to a node 1.8 uF holds to ground (a capacitor loop
    // through the rail), beside a 555 on the rail. After a 555 event the
    // rail solved before the catch-up's commit, and the re-seed took that
    // frozen estimate of the capacitors' charge, 6 uV off the partition's
    // own: the loop turned it into 4.5 A, which the trapezoid reflected at
    // every step, and the rail sat up to 2.35 mV below the node the switch
    // holds it at (the fine reference: never below it). The seed takes the
    // latest commit's charge, and the rail stays on its switch.
    let p = { let mut p = program(6, cold_581_tpl5()); p.options.integration = 1; p.digital_slot_count = 32; p };
    let v2 = -10.692059456946101;
    let mut e = Engine::load(&p).unwrap();
    for &t in &STOPS_581 {
        e.advance(t).unwrap_or_else(|er| panic!("advance to {t}: {} {}", er.code, er.message));
        let u = e.voltages()[1];
        assert!(u >= v2 - 1e-4, "the rail at {u} V at {t} s, {:.2e} V below the node its switch holds it at", v2 - u);
    }
}

const STOPS_581: [f64; 159] = [0.006272880903073908, 0.01113267594955776, 0.01957020525564175, 0.03763243552995939, 0.05465333910305036, 0.05805194121447384, 0.05839371322009491, 0.0636490324683268, 0.0655806117888315, 0.06568482079259369, 0.06794850824776758, 0.07539398321999564, 0.07554294698205018, 0.07581042056533828, 0.07639743353063506, 0.07650898912417448, 0.07694873951594532, 0.077588471389285, 0.07794290816896733, 0.07983550359105632, 0.09740273448689196, 0.09852474556066702, 0.11308447765897386, 0.11437754442941998, 0.1149492463832852, 0.11591684220361788, 0.13541387435262328, 0.13566239778691072, 0.14068460126602286, 0.14825511443343634, 0.14917335741571927, 0.15046956630838626, 0.1526020832249455, 0.15279688120259813, 0.15864293984790442, 0.15886988174793593, 0.15992734461959365, 0.16086639207190367, 0.161024395346723, 0.1869483625340615, 0.18739561649038786, 0.1878993828186588, 0.1950739796942984, 0.1955397888674969, 0.19585151705228696, 0.1959988104011165, 0.20677487866966673, 0.2245799038003945, 0.2289626607974781, 0.27371363906643753, 0.27886447538627274, 0.30471298597494473, 0.3050436679894259, 0.305259476279998, 0.3054288706367639, 0.3342174035345849, 0.3343649596349184, 0.33488268187153414, 0.33514175836886195, 0.33566053198861195, 0.33652334524521793, 0.36279279383915547, 0.3759508357915635, 0.3760946998681795, 0.38082829891741105, 0.39101289783450355, 0.4036944986390005, 0.41928954016241715, 0.4235985202746962, 0.43297664169695826, 0.44531605311431033, 0.445731204996931, 0.44915583633014355, 0.4498750498929623, 0.450055936542663, 0.45430523386227684, 0.4546977327918623, 0.4594551533042145, 0.469426950350428, 0.48425237809726296, 0.48573314940808576, 0.4962736829250874, 0.5096428621944392, 0.5463222281266975, 0.5472951915662534, 0.5496678572266223, 0.5497716818359579, 0.5692352021450174, 0.5940532164242823, 0.6334400494930552, 0.6341800073857087, 0.6406688507354097, 0.6734037755109588, 0.6912106628787511, 0.7091599007920022, 0.715383279766472, 0.7272563130629344, 0.7360478449649418, 0.7365707368701967, 0.7459821648152626, 0.750141954954762, 0.7522710146940683, 0.7696081100567205, 0.7699060056710554, 0.7700443471089582, 0.7786332669917162, 0.778750321274665, 0.7789011073804565, 0.7807698289530678, 0.7808751087530468, 0.8299618888310879, 0.830637472188818, 0.8359460125756414, 0.8720294981436804, 0.8850042393289244, 0.8880564652326893, 0.888301689734015, 0.8884777829784505, 0.9088512991847896, 0.9147915649139727, 0.9164182955507595, 0.9178515198060165, 0.9200131561161826, 0.9201196678224557, 0.9421634164451697, 0.9510479606732842, 0.953613236150933, 0.9685100591005242, 0.9686268637291654, 0.9845844075398119, 1.0034675955261771, 1.0077193501665955, 1.0078899290528915, 1.008421573501843, 1.0087324261868424, 1.0178372582350905, 1.026655356766963, 1.0451945033988215, 1.0453850132925973, 1.0740749007337786, 1.1159363609445265, 1.117028773453567, 1.119487500434228, 1.1342941408490226, 1.1346640006023057, 1.1348342523410229, 1.1351465031360592, 1.1602669514270387, 1.1626969576452972, 1.171818574081458, 1.1735445232227883, 1.1793079048665507, 1.1797811511551783, 1.1800359235022633, 1.1823562236835037, 1.187737079657078, 1.1879357771080772, 1.1880371352598253, 1.2];

/// tpl=5 hub=2 e=12.350 psu ilim=0.035 V2=10.69 C5-0=1.8e-6 C1-0=2.2e-5 C1-2=2.3e-3 555 vcc1 gnd0 [1, 1, 5, 4, 4, 3] C0-4=2.7e-7 R1-2=5.4 SW1-2(1)+R2-0=386 C1-5=7.6e-3 R0-5=48203.8 C1-3=3.6e-4 span=23.042158621509657
/// options: Options { integration: 1, no_solution_cache: false, no_rail_shortcut: false, force_stepped: true, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 6
/// toggles [(0.05465333910305036, 15), (0.16086639207190367, 15)]
fn cold_581_tpl5() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![12.350099164563556, 0.03455615312550691], vec![]),
        fz(2, 1, &[0, 2], vec![10.692059456946101], vec![]),
        fz(1, 2, &[3, 0], vec![982405.229513162], vec![]),
        fz(1, 3, &[4, 0], vec![467834.5356529982], vec![]),
        fz(1, 4, &[5, 0], vec![46934.0157988618], vec![]),
        fz(3, 5, &[5, 0], vec![1.817800177522958e-6, 0.0, 0.0], vec![]),
        fz(3, 6, &[1, 0], vec![2.245689452361714e-5, 0.0, 0.0], vec![]),
        fz(3, 7, &[1, 2], vec![0.0023091726542618287, 0.0, 0.0], vec![]),
        fz(40, 8, &[1, 0, 1, 1, 5, 4, 4, 3], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 9, &[1, 3], vec![93520.42288142613], vec![]),
        fz(1, 10, &[3, 4], vec![406.4323002987852], vec![]),
        fz(3, 11, &[4, 0], vec![3.120940315131073e-8, 0.0, 0.0], vec![]),
        fz(1, 12, &[1, 0], vec![540.8134620759187], vec![]),
        fz(3, 13, &[0, 4], vec![2.7165252929058973e-7, 0.0, 0.0], vec![]),
        fz(1, 14, &[1, 2], vec![5.436856105452832], vec![]),
        fz(4, 15, &[1, 2], vec![1.0], vec![]),
        fz(1, 16, &[2, 0], vec![385.616342937228], vec![]),
        fz(3, 17, &[1, 5], vec![0.007648038513469115, 0.0, 0.0], vec![]),
        fz(1, 18, &[0, 3], vec![43566.54401104649], vec![]),
        fz(1, 19, &[3, 5], vec![35797.199146586085], vec![]),
        fz(1, 20, &[0, 5], vec![48203.8322706819], vec![]),
        fz(3, 21, &[1, 3], vec![0.00035646447345021964, 0.0, 0.0], vec![]),
        fz(90, 9, &[], vec![9.0, 6.838378525599318e-5, 93520.42288142613], vec![]),
        fz(90, 10, &[], vec![10.0, 0.20173067881293985, 406.4323002987852], vec![]),
        fz(90, 12, &[], vec![12.0, 0.3742723420024521, 540.8134620759187], vec![]),
        fz(90, 14, &[], vec![14.0, 177.71332160572175, 5.436856105452832], vec![]),
        fz(90, 16, &[], vec![16.0, 0.010611153649083278, 385.616342937228], vec![]),
        fz(90, 20, &[], vec![20.0, 0.004419246789533757, 48203.8322706819], vec![]),
    ]
}


#[test]
fn a_chip_powered_only_inside_its_window_on_a_rail_capacitor_is_held_off_as_without_one() {
    // Ruling G given a PSU with a rail capacitor (R68). Defect G's board (5
    // V, 50 mA; 125 ohm; a 74LS00 on the rail sinking 32 mA through 1Y only
    // inside [4.275, 5.775] V) with 10 uF (esr 0) on the rail. By hand: from
    // load the capacitor charges in CC at 50 mA less u/125, the chip off; at
    // its 4.275 V floor, powered, the draw is 66.2 mA, past the limit, and
    // off 34.2 mA: the chip starves itself there and is held off, and the
    // rail climbs to the setpoint, CV at 5 / 125 + 5 / (1e9 + 1e3) =
    // 40.000005 mA, as without the capacitor. Its retry finds 72 mA at the
    // setpoint, which CV cannot deliver: G's entry enters CC (the Newton's
    // candidate, past the setpoint on the chip-off draw, kept CV at 72 mA),
    // and a release passed on the way back up waits for the draw at the
    // setpoint to fit (R44 (a)'s threshold, read at the setpoint in CV):
    // freed at each crossing, cold 696 tpl 2's 555 was held 51 times in a
    // host step. The PSU never delivers past its limit.
    let board = window_chip_rail(5.0, 0.05, 125.0, 0, [3, 4], 1, vec![resistor(3, 1, 3, 1e3), resistor(4, 1, 4, 1e5), resistor(5, 3, 0, 1e9), capacitor(6, 1, 0, 10e-6, 0.0)]);
    let want = 5.0 / 125.0 + 5.0 / (1e9 + 1e3);
    let mut rig = Rig::load(&board).unwrap();
    for k in 1..=24 {
        let t = k as f64 * 0.25e-3;
        rig.advance(t).unwrap();
        assert!(-rig.i(0) <= 0.05 * (1.0 + 1e-9), "at {t} s the PSU delivers {} A past its 50 mA limit", -rig.i(0));
        if t >= 3e-3 {
            close(rig.v(1), 5.0, 1e-12, &format!("at {t} s: the rail at the setpoint"));
            close(rig.i(0), -want, 1e-10, &format!("at {t} s: the PSU's current"));
            assert_eq!(rig.hub().state.reg, Regime::Cv, "at {t} s");
            assert!(!rig.s().power_hold(2).is_free(), "at {t} s: the chip is held off");
        }
    }
}

/// The cold fuzz's boards whose 555 sits at its power floor on a CC rail
/// capacitor, run as the sweep runs them: each to 1.2 s.
fn railcap_floor_board(devices: Vec<Device>, nodes: usize, stops: &[f64], toggles: &[(f64, u32, f64)], what: &str) {
    let mut p = program(nodes, devices);
    p.digital_slot_count = 32;
    let mut e = Engine::load(&p).unwrap_or_else(|er| panic!("{what}: load: {} {}", er.code, er.message));
    let mut ev = 0;
    for &t in stops {
        e.advance(t).unwrap_or_else(|er| panic!("{what}: advance to {t}: {} at {}: {}", er.code, e.time(), er.message));
        while ev < toggles.len() && toggles[ev].0 <= t {
            e.set_params(&[(toggles[ev].1, 0, toggles[ev].2)]).unwrap();
            ev += 1;
        }
    }
}

#[test]
fn a_555_at_its_floor_on_a_cc_rail_capacitor_is_held_once_and_the_runs_complete() {
    // Ruling G given a PSU with a rail capacitor (R68). Cold 175 tpl 2 (3.8
    // uF), 696 tpl 2 (0.16 uF) and 627 tpl 4 (17 nF): a 555 on the rail whose
    // powered draw at the setpoint is past the limit. With the Newton's
    // candidate 175/2 declined at 0.798 s (the rail loop did not settle);
    // with G's entry alone each released its 555 at every crossing of its
    // release level and was held 51 times within a host step (the hold
    // episode's guard, `-4`). Held off while its powered draw at the
    // setpoint is past the limit, each 555 is held once and the runs reach
    // 1.2 s.
    railcap_floor_board(cold_175_tpl2_devices(), 7, &[0.006272879978984825, 0.006737256308170404, 0.038139641656497435, 0.04154750969351386, 0.041981687418143336, 0.042282020737404044, 0.04446548328535588, 0.04504935689956483, 0.05726417845589198, 0.10110338257874799, 0.14096638524514044, 0.1538781103253384, 0.15541585779436878, 0.19379853965400207, 0.19904704398077355, 0.21871570765888051, 0.22040488704267083, 0.23077962201215618, 0.23147669889061756, 0.23682007658125745, 0.24359863432451603, 0.24581044567054333, 0.24605793747451885, 0.26932953218650935, 0.26982857249566305, 0.29165774589882193, 0.323822478710669, 0.3245326395441571, 0.3359707272657522, 0.3395328772132661, 0.38864253968945234, 0.3889310914409459, 0.3894919304334255, 0.39124262951114225, 0.39141701781214466, 0.40143331551698935, 0.41628392037537565, 0.41647245933017124, 0.45280568546069694, 0.45323486667323953, 0.45527047578136115, 0.46254312537984915, 0.5042073814245781, 0.5052973211239591, 0.5536249130446808, 0.5600010651701359, 0.5601060934342678, 0.5620067865442132, 0.5624945716973923, 0.5628349088039665, 0.5662623283733159, 0.5776876980861431, 0.5782974178367883, 0.5858075376675697, 0.6146390778376243, 0.6163169542386913, 0.6166733936188559, 0.616998527942831, 0.617274097698689, 0.6211551453936076, 0.640385855888215, 0.6441264267123612, 0.6565684797438297, 0.6569960945273576, 0.6575463582125846, 0.6576758945837408, 0.6844762175685779, 0.684582582692559, 0.684817685888256, 0.6890160046023218, 0.693237305007297, 0.6955773492910368, 0.6965257941055804, 0.696654378809123, 0.7143571810475812, 0.7216829291135866, 0.7373435559888114, 0.754328481018915, 0.7848754630079678, 0.7852831311355024, 0.786396101193285, 0.8017427412366293, 0.802277048255857, 0.808092112091967, 0.8087158657433096, 0.8474199007621832, 0.8477465687221756, 0.8750504749682919, 0.8751507639377251, 0.8765261726391869, 0.8787537976809756, 0.8911437383909047, 0.8912449818991913, 0.9119844818578474, 0.9121377152095659, 0.912309607357835, 0.9128487774533108, 0.9483353977040864, 0.948530967053169, 0.9502433696033153, 0.9507873101047768, 0.9567400255275331, 0.9594387500826398, 0.9596677520782007, 0.9942583260564374, 1.0053272603519443, 1.0099697492115463, 1.0146894857869517, 1.0148388795848955, 1.016948984812883, 1.0170970624758415, 1.04920397558034, 1.0493790056935663, 1.0523782422610515, 1.0627500084062378, 1.073109372477166, 1.0736604276213615, 1.0870665604165186, 1.0872205960412311, 1.0879135530647468, 1.0880441702052972, 1.0907640331175135, 1.090944961310115, 1.092364821284739, 1.1250166891717657, 1.136415533234253, 1.1534394131003634, 1.197167881792808, 1.1973242684882521, 1.1996982000167258, 1.2], &[], "175 tpl 2");
    railcap_floor_board(cold_696_tpl2_devices(), 5, &[0.006272881183404639, 0.006966306565007355, 0.009417531962006231, 0.010957541973997885, 0.02355459038011193, 0.026963756076411785, 0.03524749073592549, 0.04725975647383719, 0.047506684947997695, 0.06415119105970056, 0.07339441021920648, 0.0763856964974442, 0.0767216336286122, 0.07806925916008774, 0.07818709350463077, 0.07893846719838202, 0.08079720773635315, 0.08142740199172127, 0.1278637102858161, 0.12803623625819735, 0.1456503375690544, 0.16540303969681716, 0.17130347133357166, 0.17149465284851506, 0.18829395345581199, 0.22891522085945265, 0.24780265473450278, 0.25011757017665937, 0.25709668299691135, 0.25722908427919655, 0.2652023898240226, 0.2868545263940312, 0.2871097340013043, 0.2922164944666682, 0.29513873100719434, 0.30867664648499343, 0.32653383759677823, 0.3392771365981063, 0.3772691209535698, 0.3863825499058944, 0.3867281888454588, 0.38951243662268603, 0.3910711920431147, 0.3913576042444156, 0.39346624956021864, 0.3957817570250845, 0.3960732013741232, 0.396477561189899, 0.3967531855218919, 0.4240738864233175, 0.4320792030926026, 0.43239352394159325, 0.45247178112870723, 0.45906585528861965, 0.45934622990247115, 0.4638487786137009, 0.4728329167219261, 0.47311241225309164, 0.473960215269201, 0.4740611241551731, 0.47773209657392424, 0.47883481561986774, 0.47899502194103233, 0.5058493547346157, 0.5068517048299418, 0.5070328878186144, 0.5073627449116349, 0.5155501551514791, 0.5164746998781291, 0.5424558631820314, 0.5539316461150392, 0.5989877867006879, 0.5992644722328755, 0.6220764553814448, 0.6221917871463515, 0.6484452905573084, 0.6485476200606125, 0.6512305884686025, 0.6519572571101679, 0.6526325499793951, 0.6711029307162208, 0.6741383087505061, 0.6837378195054654, 0.6842202104834562, 0.6873195423529334, 0.6895884529461211, 0.6897352603469128, 0.7102857482090696, 0.7335487371280908, 0.7359996624500226, 0.7572778558913686, 0.7753091208859357, 0.7892972575327339, 0.7922189983926099, 0.7936004432002328, 0.7959513378986545, 0.7971050526833895, 0.816094455548527, 0.8266710755896289, 0.8284596868301134, 0.8300907135643512, 0.8338455854345832, 0.851084716583123, 0.8523598848248947, 0.8526827047214832, 0.860575137181093, 0.8750675322996903, 0.892051051289009, 0.9238646102765796, 0.9245136125560721, 0.9246422438374858, 0.9255307319398658, 0.9754067734999342, 0.9778683953578068, 0.9782142330329157, 0.9805620418918208, 1.014210892222082, 1.0294811420264873, 1.0318459352673102, 1.0323735996964483, 1.0326660424974268, 1.0430840085038013, 1.0432013812153194, 1.045405754866649, 1.0468611770543035, 1.0471365411515372, 1.055467884710807, 1.0615342280699196, 1.1046550513486346, 1.1077281800061642, 1.139112503451449, 1.1471620721446683, 1.1474698864245099, 1.1477566390842864, 1.151405114753475, 1.1535995582020542, 1.153846149377815, 1.1592111537948364, 1.1610874515489649, 1.1612965734487264, 1.169286366043265, 1.171227558640489, 1.1718417191340897, 1.1721311598437594, 1.1724161972261458, 1.1729681692741354, 1.173922015196817, 1.1741205750770491, 1.184052249992642, 1.1843930975530832, 1.1855259903438418, 1.1859488870010375, 1.1919555510036834, 1.2], &[], "696 tpl 2");
    railcap_floor_board(cold_627_tpl4_devices(), 6, &[0.006272881016766573, 0.05623259616441002, 0.05664913219758152, 0.05704805905461277, 0.06647733163621133, 0.06670729340603633, 0.09779689458687386, 0.0999033416907044, 0.10104853189513649, 0.10449849357542228, 0.10895390827942177, 0.1125766280827211, 0.11469606040192862, 0.13241814745632544, 0.13581213053659347, 0.1365990894495477, 0.14029598965907022, 0.16620871703307097, 0.16686129870132388, 0.1971102969843148, 0.2083438101180934, 0.2140314899014415, 0.21502222581080524, 0.22084533813039128, 0.2243565941554369, 0.2623222576493554, 0.30510959781143765, 0.30522595161635047, 0.33853756363635545, 0.34084710413295477, 0.34280414351273736, 0.34306507676600556, 0.3434984852487767, 0.38358494348731403, 0.38375877395037405, 0.4222641464092727, 0.433321932845347, 0.4338523448905136, 0.43396925693799865, 0.44886578589874393, 0.44969275239733364, 0.4499429700336419, 0.45305846852083814, 0.49911023110372726, 0.5054030497156946, 0.5069132563003466, 0.5074517347016072, 0.52822508390344, 0.5367946163904193, 0.53708479615216, 0.5380148583276968, 0.5397608981808637, 0.5417488528298878, 0.5513117387650649, 0.557336225352163, 0.5575380510559441, 0.5732368176604477, 0.575832959565935, 0.579493043461084, 0.5796096030730169, 0.5807725997103055, 0.5810736311490378, 0.6108733666846701, 0.633696118006043, 0.6339015950557203, 0.6346261067602708, 0.6461864685717434, 0.6478131497448482, 0.6518560917799253, 0.6522625187432419, 0.6527730136800899, 0.6588221130326025, 0.7074536033927173, 0.7075547948233323, 0.7077559146065809, 0.7108166684124186, 0.7128565393576757, 0.7364918593883033, 0.7694517504077503, 0.776441425685768, 0.776927866522161, 0.8246991932050719, 0.8606056962007163, 0.8610609653973494, 0.8656335113054539, 0.8658588482832231, 0.8869364059573257, 0.9249113499509588, 0.9261906512518973, 0.9282139110325576, 0.9285314812508769, 0.9536439035101283, 0.9539367472877169, 0.9625583472572937, 0.9629534872108801, 0.9636272723926842, 0.9659238168964163, 0.9801584292372287, 0.980322391623014, 0.9966563642746553, 1.014746902014974, 1.0157224631532593, 1.0158812018385852, 1.0166662895786631, 1.0168326681054214, 1.0170533579220133, 1.0172047495707217, 1.05634417957352, 1.058229582983137, 1.0597487861687773, 1.0961381280217837, 1.097827821540921, 1.1094091177827705, 1.113830729378578, 1.1186110356039705, 1.1202097954691805, 1.1452397790672935, 1.146796588148531, 1.1533633054719903, 1.1674529229540787, 1.1676138331595411, 1.169306322429446, 1.184768680260098, 1.1849417096829853, 1.1853879852812252, 1.187809885281425, 1.1879795227065086, 1.1881332064254435, 1.188245179702328, 1.1895629527107985, 1.1914271809532544, 1.1920535684950222, 1.2], &[(0.557336225352163, 14, 0.0), (0.6108733666846701, 14, 1.0), (0.633696118006043, 14, 0.0)], "627 tpl 4");
}

/// tpl=2 hub=2 e=5.279 psu ilim=0.125 V2=3.29 LED6-2+R1-6=768 555 vcc1 gnd0 [6, 3, 5, 5, 1, 1] R0-1=80444.0 C6-0=1.0e-4 C4-2=5.7e-6 LED0-1+R0-0=11 C4-0=6.9e-5 span=5.279251491871787
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: true, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 7
/// toggles []
fn cold_175_tpl2_devices() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![5.279251491871787, 0.12486913735539018], vec![]),
        fz(2, 1, &[2, 0], vec![3.2944469335300433], vec![]),
        fz(1, 2, &[3, 0], vec![523113.99131483736], vec![]),
        fz(1, 3, &[4, 0], vec![441731.46301296854], vec![]),
        fz(1, 4, &[5, 0], vec![400887.03122832946], vec![]),
        fz(1, 5, &[6, 0], vec![19551.037108783705], vec![]),
        fz(30, 6, &[6, 2], vec![2.490487895713666, 2.133341615049267, 0.005941553299019336, 0.0, 0.0, 0.0], vec![]),
        fz(1, 7, &[1, 6], vec![768.0287173218338], vec![]),
        fz(40, 8, &[1, 0, 6, 3, 5, 5, 1, 1], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 9, &[1, 1], vec![259.9129858404332], vec![]),
        fz(1, 10, &[1, 1], vec![22798.751989056218], vec![]),
        fz(3, 11, &[1, 0], vec![3.845883733983537e-6, 0.0, 0.0], vec![]),
        fz(1, 12, &[3, 0], vec![2705.033195768211], vec![]),
        fz(1, 13, &[5, 6], vec![16770.726136343015], vec![]),
        fz(1, 14, &[3, 6], vec![22693.85635324758], vec![]),
        fz(1, 15, &[0, 1], vec![80444.03115180125], vec![]),
        fz(1, 16, &[6, 3], vec![1832.7466798794446], vec![]),
        fz(3, 17, &[6, 0], vec![0.00010041305989192043, 0.0, 0.0], vec![]),
        fz(3, 18, &[4, 2], vec![5.709328797531033e-6, 0.0, 0.0], vec![]),
        fz(30, 19, &[0, 1], vec![1.8202604658110737, 1.799465313988287, 0.008247942771208824, 0.0, 0.0, 0.0], vec![]),
        fz(1, 20, &[0, 0], vec![11.194632856996742], vec![]),
        fz(1, 21, &[4, 5], vec![3718.5413539250926], vec![]),
        fz(3, 22, &[4, 0], vec![6.8848737667698e-5, 0.0, 0.0], vec![]),
        fz(1, 23, &[0, 1], vec![671.9597403622046], vec![]),
        fz(90, 7, &[], vec![7.0, 0.0023589339771113043, 768.0287173218338], vec![]),
        fz(90, 9, &[], vec![9.0, 0.20249870632200995, 259.9129858404332], vec![]),
        fz(90, 10, &[], vec![10.0, 9.903850432653147e-5, 22798.751989056218], vec![]),
        fz(90, 12, &[], vec![12.0, 2.3342260829871632e-5, 2705.033195768211], vec![]),
        fz(90, 15, &[], vec![15.0, 0.0002399021656463534, 80444.03115180125], vec![]),
        fz(90, 20, &[], vec![20.0, 0.025490775773001514, 11.194632856996742], vec![]),
    ]
}

/// tpl=2 hub=2 e=5.301 psu ilim=0.212 V2=-7.37 555 vcc1 gnd0 [1, 1, 0, 4, 1, 3] F1-3=0.41/0.60 LED2-3+R2-2=798 R3-0=56927.4 span=7.3665463086539225
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 5
/// toggles []
fn cold_696_tpl2_devices() -> Vec<Device> {
    vec![
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
    ]
}

/// tpl=4 hub=2 e=9.003 psu ilim=0.078 CLK3 0.4..7.4 555 vcc1 gnd3 [3, 3, 1, 3, 1, 5] LED3-0+R3-3=861 R5-3=64876.3 C4-0=2.8e-3 SW4-3(1)+R3-0=37 P1-5 R1-2=60.0 LED0-3+R0-0=909 LED5-2 span=9.002652064682454
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 6
/// toggles [(0.557336225352163, 14), (0.6108733666846701, 14), (0.633696118006043, 14)]
fn cold_627_tpl4_devices() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![9.002652064682454, 0.07842457417248008], vec![]),
        fz(22, 1, &[3, 0], vec![11.170381444935689, 0.38874085736449426, 7.358788763747977, 0.406113937571952, 0.0], vec![8.0]),
        fz(1, 2, &[2, 0], vec![6419.094329868755], vec![]),
        fz(1, 3, &[4, 0], vec![10410.13120021588], vec![]),
        fz(1, 4, &[5, 0], vec![13819.797840276742], vec![]),
        fz(40, 5, &[1, 3, 3, 3, 1, 3, 1, 5], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 6, &[1, 5], vec![1536.8923756235156], vec![]),
        fz(1, 7, &[5, 1], vec![26837.57402236184], vec![]),
        fz(3, 8, &[1, 0], vec![1.7142691704725365e-8, 0.0, 0.0], vec![]),
        fz(1, 9, &[3, 4], vec![178.61467778880706], vec![]),
        fz(30, 10, &[3, 0], vec![1.8632162806099972, 1.740412493310314, 0.026398833096468073, 0.0, 0.0, 0.0], vec![]),
        fz(1, 11, &[3, 3], vec![861.0395061216053], vec![]),
        fz(1, 12, &[5, 3], vec![64876.34263329658], vec![]),
        fz(3, 13, &[4, 0], vec![0.0028204082362427297, 0.0, 0.0], vec![]),
        fz(4, 14, &[4, 3], vec![1.0], vec![]),
        fz(1, 15, &[3, 0], vec![36.541607442338986], vec![]),
        fz(1, 16, &[0, 5], vec![1011.579616198879], vec![]),
        fz(13, 17, &[1, 5], vec![0.6038554451489084, 0.32511714268017733], vec![]),
        fz(1, 18, &[1, 2], vec![60.04088277596672], vec![]),
        fz(30, 19, &[0, 3], vec![1.85046544523346, 2.215310839517028, 0.022160237419898924, 0.0, 0.0, 0.0], vec![]),
        fz(1, 20, &[0, 0], vec![908.7530823473618], vec![]),
        fz(30, 21, &[5, 2], vec![3.1690758042803058, 1.7981780760612027, 0.019934775063414917, 0.0, 0.0, 0.0], vec![]),
        fz(90, 6, &[], vec![6.0, 0.00011141787162135056, 1536.8923756235156], vec![]),
        fz(90, 7, &[], vec![7.0, 0.0029175595606077387, 26837.57402236184], vec![]),
        fz(90, 9, &[], vec![9.0, 0.006418615673997487, 178.61467778880706], vec![]),
        fz(90, 11, &[], vec![11.0, 0.14930806568382282, 861.0395061216053], vec![]),
        fz(90, 12, &[], vec![12.0, 0.0002497372969288061, 64876.34263329658], vec![]),
        fz(90, 15, &[], vec![15.0, 0.6805894678323418, 36.541607442338986], vec![]),
        fz(90, 18, &[], vec![18.0, 0.03491576178552989, 60.04088277596672], vec![]),
        fz(90, 20, &[], vec![20.0, 0.08455322243772491, 908.7530823473618], vec![]),
    ]
}

#[test]
fn an_led_draining_a_cc_rail_capacitor_carries_the_stress_its_own_drain_gives() {
    // Ruling R68, the cold fuzz's 84 tpl 1: a {10.65 V, 92 mA} PSU with 2.65
    // uF (esr 0) on the rail and an LED (vf 2.68 V, n 2.27, ifMax 28.06 mA,
    // no series resistance) from the rail to a clock's node. At the clock's
    // fall to -3.31 V the LED takes 789.6 A from the charged capacitor; past
    // its knee its law is a line of 71.5 S, so the capacitor drains on a
    // single exponential, tau = C / g = 37.1 ns, to -0.563 V. By hand the
    // damage integral of the drain is (I0 / ifMax)^2 tau / 2 = 14.71 against
    // the latch's 24 (led_failed, rate (I / ifMax)^2 - 1): the LED survives
    // the first fall, and over the low phase its 86.6 mA adds 0.01. The
    // second fall needs 9.25 more, reached at e^(-2t/tau) = 1 - 9.25 / 14.71,
    // where the drain has brought the rail to -0.563 + 11.21 e^(-t/tau) =
    // 6.26 V: the LED opens there and the rail climbs from it. Exact mode, the
    // fine reference, latched at the first fall: its stepped rail partition
    // took the step's end rate, 7.9e8 per second, over the whole 2.07 ms
    // step that ended at the edge (W1g point 5's quantization), an
    // integral of 1.6e6. The default rail before R68 (the setpoint held in
    // CV at the edge) latched there too.
    let mut p = program(6, cold_84_tpl1());
    p.digital_slot_count = 32;
    let mut rig = Rig::load(&p).unwrap();
    rig.advance(0.0021).unwrap();
    let s1 = rig.s().slow.stress(10).map(|s| s.value_at(0.0021)).unwrap();
    assert!(!rig.s().slow.is_failed(10), "the LED survives the first fall");
    close(s1, 14.71, 0.02 * 14.71, "the first drain's damage integral");
    close(rig.v(1), -0.563, 0.01, "the rail an LED drop over the clock's node");
    rig.advance(0.0054).unwrap();
    assert!(!rig.s().slow.is_failed(10), "still intact before the second fall");
    rig.advance(0.0055).unwrap();
    assert!(rig.s().slow.is_failed(10), "the second fall burns it");
    let t_fall = rig.s().slow.stress(10).map(|s| s.t0).unwrap();
    assert!(t_fall < 0.0055, "latched at {t_fall}");
}

/// tpl=1 hub=2 e=10.647 psu ilim=0.092 CLK3 -3.3..9.5 stacked4-3 1.1 C1-0=2.7e-6 R2-3=22.1 R2-3=84817.4 R3-5=8.1 R4-5=104.8 LED1-3+R1-1=123 74LS00 vcc1 gnd0 in5,3 out5,0 span=13.96067493276152
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 6
/// toggles []
fn cold_84_tpl1() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![10.647486522334537, 0.0923007229999578], vec![]),
        fz(22, 1, &[3, 0], vec![292.4270639400117, 0.6067396404390497, 9.512811525710633, -3.3131884104269833, 0.0], vec![8.0]),
        fz(2, 2, &[4, 3], vec![1.0658932999473136], vec![]),
        fz(1, 3, &[2, 0], vec![11183.931130067276], vec![]),
        fz(1, 4, &[5, 0], vec![3653.7769026175633], vec![]),
        fz(3, 5, &[1, 0], vec![2.654722241109152e-6, 0.0, 0.0], vec![]),
        fz(1, 6, &[2, 3], vec![22.13610172025595], vec![]),
        fz(1, 7, &[2, 3], vec![84817.38849207966], vec![]),
        fz(1, 8, &[3, 5], vec![8.115792481404343], vec![]),
        fz(1, 9, &[4, 5], vec![104.78146855709932], vec![]),
        fz(30, 10, &[1, 3], vec![2.6844845184488806, 2.2666570131156947, 0.02805793538661563, 0.0, 0.0, 0.0], vec![]),
        fz(1, 11, &[1, 1], vec![123.25751103521999], vec![]),
        fz(50, 12, &[1, 0, 5, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 5], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.5215933255689305e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 13, &[5, 1], vec![258.85338726766065], vec![]),
        fz(1, 14, &[0, 1], vec![131.22013358699968], vec![]),
        fz(90, 6, &[], vec![6.0, 0.10039255154394434, 22.13610172025595], vec![]),
        fz(90, 7, &[], vec![7.0, 1.144579996054986e-5, 84817.38849207966], vec![]),
        fz(90, 8, &[], vec![8.0, 29.322249323215566, 8.115792481404343], vec![]),
        fz(90, 9, &[], vec![9.0, 0.004031042336012474, 104.78146855709932], vec![]),
        fz(90, 11, &[], vec![11.0, 0.04323459645416291, 123.25751103521999], vec![]),
        fz(90, 13, &[], vec![13.0, 0.2251908463627935, 258.85338726766065], vec![]),
        fz(90, 14, &[], vec![14.0, 0.003338164034755094, 131.22013358699968], vec![]),
    ]
}

/// Cold 482 tpl 4 (ruling R70) run as the sweep runs it, the rail at each
/// of `at`.
fn cold_482_rail(exact_mode: bool, at: &[f64]) -> Vec<f64> {
    let mut p = program(5, cold_482_tpl4());
    p.options.force_stepped = true;
    p.digital_slot_count = 32;
    if exact_mode {
        p = exact(p);
    }
    let mut e = Engine::load(&p).unwrap();
    let stops: [f64; 174] = [0.006272879523874057, 0.012412271948377768, 0.012528317268689065, 0.018408923456882432, 0.019345974824282033, 0.019612906676866496, 0.031218077038142264, 0.04376215273942862, 0.04416467898139638, 0.05594939440566254, 0.07533198233624208, 0.07888737325107964, 0.07919724051874279, 0.08011801553600362, 0.08030492082035544, 0.085503575843058, 0.09434759508891336, 0.12784628024283307, 0.12885960628037776, 0.13719614828936066, 0.14382792403718578, 0.1454242658892979, 0.1456801638870943, 0.14684714884773747, 0.1507832457728929, 0.15141340419834823, 0.1559279704542733, 0.20541803882120785, 0.24090481608124162, 0.24516127846648175, 0.245334144645241, 0.27936616781801316, 0.2816351475290895, 0.2819255379407827, 0.29138429910570895, 0.3126931347528943, 0.35319741209825023, 0.3541893691607316, 0.3548860734402846, 0.3661607311190105, 0.3663238187819686, 0.36639181369082807, 0.36694692210663105, 0.3692637024106552, 0.3693927494412495, 0.374228015953136, 0.37499050210073626, 0.3796460293653142, 0.38212582641489845, 0.42936956619685296, 0.43002267745223827, 0.43054550270689396, 0.43123926796939815, 0.43199557971015196, 0.43308000238112004, 0.4766941497730408, 0.47735442472374134, 0.4937309232971509, 0.49939367090187603, 0.49960927458157023, 0.5032723511236836, 0.503412647565242, 0.5038332384333996, 0.5053965631310094, 0.5055867394201828, 0.5087751972720775, 0.5094296430281484, 0.5095741598933129, 0.5219318741647693, 0.5278104245878644, 0.5292724514768359, 0.5412857099764011, 0.5416278751681141, 0.544168931689411, 0.5522177181459947, 0.5635746187475017, 0.5802449847143053, 0.5806022171725509, 0.598313315455535, 0.5984998230339097, 0.5997626231309076, 0.6017594598800152, 0.602293876189981, 0.6464684685358049, 0.646591692608136, 0.6468136571582461, 0.6550279161822433, 0.6630285160445724, 0.6653993619600314, 0.6693009104499813, 0.6707441731264153, 0.6751336916359344, 0.6763215149849022, 0.6948737146008922, 0.6987684531853258, 0.7169818581864273, 0.718593335158148, 0.7251603668806493, 0.7283401051072289, 0.7379531147652653, 0.7426073385569902, 0.7560084559639546, 0.7590633471647963, 0.7705896911583276, 0.7708459941986021, 0.7808134986578842, 0.7810792280792902, 0.8012126205415933, 0.8014254384838743, 0.8015737219417354, 0.824525998705598, 0.8279515045603405, 0.8288214050004286, 0.8292537229876062, 0.8433932952694639, 0.8434996361046219, 0.8437470171602771, 0.8480538110034029, 0.8559643990109206, 0.8561246766890488, 0.8568139439064969, 0.8592084930664139, 0.8601070941383088, 0.8650805472709621, 0.865234073578485, 0.8653676161399267, 0.8670921433805497, 0.8681321645970119, 0.869524167815423, 0.8808999527870099, 0.8967110895896362, 0.9018035452024994, 0.9020945331994755, 0.9027931142839448, 0.9051983233687657, 0.9053021329839148, 0.919982505917073, 0.9216093029412679, 0.922912337746953, 0.9230345091205124, 0.9235580436535118, 0.9295875537886162, 0.9298642563389333, 0.9744522434624334, 0.9895953436926982, 0.9976276517883502, 0.9977958829271243, 0.9990749652934836, 1.0090691315352036, 1.0091981709522362, 1.010353096194347, 1.0148772391254255, 1.0181597638352324, 1.023650333556795, 1.0258131646830793, 1.0355453618476096, 1.0542123256840308, 1.0593844511546993, 1.0640088368404865, 1.0728552170251124, 1.077584872963798, 1.0783254548646353, 1.0967978524724995, 1.0970150010326554, 1.0982814100803737, 1.1011104824352245, 1.1041454500452859, 1.1190407202482866, 1.1483538412743324, 1.1493054700798877, 1.186341461104113, 1.1869338711266828, 1.1874948492423163, 1.2];
    let toggles: [(f64, u32, f64); 8] = [(0.012412271948377768, 6, 0.0), (0.14382792403718578, 10, 0.0), (0.36639181369082807, 6, 1.0), (0.5032723511236836, 8, 0.0), (0.5053965631310094, 6, 0.0), (0.8559643990109206, 8, 1.0), (0.9295875537886162, 10, 1.0), (0.9744522434624334, 8, 0.0)];
    let (mut ev, mut out) = (0, Vec::new());
    for &t in &stops {
        e.advance(t).unwrap_or_else(|er| panic!("advance to {t}: {} {}", er.code, er.message));
        while ev < toggles.len() && toggles[ev].0 <= t {
            e.set_params(&[(toggles[ev].1, 0, toggles[ev].2)]).unwrap();
            ev += 1;
        }
        if at.contains(&t) {
            out.push(e.voltages()[1]);
        }
    }
    out
}

/// Ruling R70's hand values for cold 482 tpl 4: from 0.3669 s (where every
/// tree agrees) a {11.05 V, 42.66 mA} PSU in CC into 588.6 uF (302.0 + 286.7
/// uF, esr 0), 95.63 ohm to a 240.5 Hz clock's node (4.50 / -2.70 V), and a
/// 74LS00 whose 1Y and 4Y, HIGH, drive that node from the rail through
/// `R_out = clamp(u / 0.032, 50, 1000)` while powered, inside [4.275,
/// 5.775] V: `C du/dt = I - (u - v3) / R - chip(u, v3)`, integrated at 0.2
/// us (`acc-g2/ode482.py`, model A), the chip powered whenever its window
/// says so: 4.1714 V at 0.98960 s, 4.2516 V at 1.2 s.
const COLD_482_AT: [f64; 2] = [0.9895953436926982, 1.2];
const COLD_482_HAND: [f64; 2] = [4.171428, 4.251644];

#[test]
fn cold_482_tpl4_exact_mode_follows_the_hand_rail() {
    // Exact mode, the rail and the chip one partition, slides through the
    // chip's floor as the hand integration does: within W0 of it.
    for (u, (t, want)) in cold_482_rail(true, &COLD_482_AT).into_iter().zip(COLD_482_AT.iter().zip(COLD_482_HAND)) {
        close(u, want, 1e-4 + 1e-3 * want, &format!("exact mode, the rail at {t} s"));
    }
}

#[test]
#[ignore = "R70: default mode's hold cadence on a CC rail capacitor (the R44 class); known item"]
fn cold_482_tpl4_default_mode_follows_the_hand_rail() {
    // Known item (ruling R70). Default mode holds the 74LS00 off at its
    // floor (the passes alternate there, W1c point 1a) and frees it only at
    // R44 (a)'s draw threshold or a host step's retry, so the capacitors
    // carry the rail up past the floor with the chip off: 4.29 V and 4.67 V
    // where the chip, powered inside its window, holds it at 4.17 and 4.25
    // (model B of the hand integration, the chip off whenever powering it
    // would turn the rail down, gives 4.53 and 4.62). The fix is a hold that
    // lets the chip slide at its edge (R53's duty hold), not R68's step.
    for (u, (t, want)) in cold_482_rail(false, &COLD_482_AT).into_iter().zip(COLD_482_AT.iter().zip(COLD_482_HAND)) {
        close(u, want, 1e-4 + 1e-3 * want, &format!("default mode, the rail at {t} s"));
    }
}

/// tpl=4 hub=2 e=11.045 psu ilim=0.043 V2=-6.94 CLK3 -2.7..4.5 LED2-1 C1-0=3.0e-4 SW3-1(1)+R1-0=7 SW1-4(1)+R4-0=887 SW3-4(1)+R4-0=993 C4-0=8.2e-4 R4-0=246.9 C0-1=2.9e-4 74LS00 vcc1 gnd0 in0,3 out3,1 LED0-1+R0-0=11 C4-0=9.4e-4 span=13.742390062059124
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: true, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 5
/// toggles [(0.012412271948377768, 6), (0.14382792403718578, 10), (0.36639181369082807, 6), (0.5032723511236836, 8), (0.5053965631310094, 6), (0.8559643990109206, 8), (0.9295875537886162, 10), (0.9744522434624334, 8)]
fn cold_482_tpl4() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![11.045175895089358, 0.04266451859547341], vec![]),
        fz(2, 1, &[0, 2], vec![-6.943904798877482], vec![]),
        fz(22, 2, &[3, 0], vec![240.50718574088432, 0.45359874929345734, 4.503240762000619, -2.697214166969767, 0.0], vec![8.0]),
        fz(1, 3, &[4, 0], vec![21100.330743748636], vec![]),
        fz(30, 4, &[2, 1], vec![3.019387729239728, 1.8266488862088588, 0.028356499998380528, 0.0, 0.0, 0.0], vec![]),
        fz(3, 5, &[1, 0], vec![0.0003019196554823646, 0.017756036250419105, 0.0], vec![]),
        fz(4, 6, &[3, 1], vec![1.0], vec![]),
        fz(1, 7, &[1, 0], vec![6.617081827964158], vec![]),
        fz(4, 8, &[1, 4], vec![1.0], vec![]),
        fz(1, 9, &[4, 0], vec![887.1909401513195], vec![]),
        fz(4, 10, &[3, 4], vec![1.0], vec![]),
        fz(1, 11, &[4, 0], vec![993.4999163276094], vec![]),
        fz(3, 12, &[4, 0], vec![0.0008158330397103426, 0.0, 0.0], vec![]),
        fz(1, 13, &[4, 0], vec![246.90016158909214], vec![]),
        fz(3, 14, &[0, 1], vec![0.0002866988805255068, 0.0, 0.0], vec![]),
        fz(50, 15, &[1, 0, 0, 3, 3, 0, 0, 1, 0, 0, 1, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 9.174107314946013e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 16, &[3, 1], vec![56.073951299094], vec![]),
        fz(1, 17, &[1, 4], vec![95.62687998160986], vec![]),
        fz(30, 18, &[0, 1], vec![2.8408549966168204, 1.8332298824149202, 0.01422995385836658, 0.0, 0.0, 0.0], vec![]),
        fz(1, 19, &[0, 0], vec![11.35105520744054], vec![]),
        fz(1, 20, &[4, 2], vec![15454.490044496439], vec![]),
        fz(3, 21, &[4, 0], vec![0.0009350098792573034, 0.0, 0.0], vec![]),
        fz(90, 7, &[], vec![7.0, 0.060299739842273666, 6.617081827964158], vec![]),
        fz(90, 9, &[], vec![9.0, 0.3243969594387307, 887.1909401513195], vec![]),
        fz(90, 11, &[], vec![11.0, 0.008837830702239091, 993.4999163276094], vec![]),
        fz(90, 13, &[], vec![13.0, 0.025494433082430008, 246.90016158909214], vec![]),
        fz(90, 16, &[], vec![16.0, 0.05497262303980982, 56.073951299094], vec![]),
        fz(90, 17, &[], vec![17.0, 0.4652254677155526, 95.62687998160986], vec![]),
        fz(90, 19, &[], vec![19.0, 0.7095611534943612, 11.35105520744054], vec![]),
    ]
}

/// tpl=4 hub=2 e=10.933 psu ilim=0.968 CLK3 0.9..5.9 C5-0=1.1e-7 R7-3=15.4 F1-7=0.17/0.09 C1-2=2.2e-3 LED6-3+R1-6=943 R4-2=2.2 R2-1=197798.3 74LS00 vcc1 gnd0 in2,4 out2,7 SW5-6(0)+R6-0=1240 R6-4=66.6 SW0-7(1)+R7-0=21; exact mode
fn cold_39_tpl4() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![10.933286069379529, 0.9684681458752474], vec![]),
        fz(22, 1, &[3, 0], vec![24.69012126470536, 0.24569337194886948, 5.876795877236651, 0.9029359146706, 0.0], vec![8.0]),
        fz(1, 2, &[2, 0], vec![84079.32405240185], vec![]),
        fz(1, 3, &[4, 0], vec![67903.00466951706], vec![]),
        fz(1, 4, &[5, 0], vec![4378.458207121376], vec![]),
        fz(1, 5, &[6, 0], vec![7660.232934161665], vec![]),
        fz(1, 6, &[7, 0], vec![15701.465251996993], vec![]),
        fz(3, 7, &[5, 0], vec![1.1052814462456185e-7, 0.0, 0.0], vec![]),
        fz(1, 8, &[7, 3], vec![15.360232755371694], vec![]),
        fz(10, 9, &[1, 7], vec![0.16963240310714417, 0.0854298245055415], vec![]),
        fz(3, 10, &[1, 2], vec![0.002168065767600623, 0.0, 0.0], vec![]),
        fz(30, 11, &[6, 3], vec![2.281369834862081, 2.2508941490716845, 0.03800053357995761, 0.0, 0.0, 0.0], vec![]),
        fz(1, 12, &[1, 6], vec![942.829606302069], vec![]),
        fz(1, 13, &[4, 2], vec![2.150492269391616], vec![]),
        fz(1, 14, &[2, 1], vec![197798.3382206931], vec![]),
        fz(50, 15, &[1, 0, 2, 4, 2, 0, 0, 7, 0, 0, 7, 0, 0, 2], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 7.613498786080187e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 16, &[2, 2], vec![119.59921047457372], vec![]),
        fz(1, 17, &[7, 6], vec![58.41222708854561], vec![]),
        fz(4, 18, &[5, 6], vec![0.0], vec![]),
        fz(1, 19, &[6, 0], vec![1239.9216711781662], vec![]),
        fz(1, 20, &[6, 4], vec![66.6090932858312], vec![]),
        fz(4, 21, &[0, 7], vec![1.0], vec![]),
        fz(1, 22, &[7, 0], vec![21.143596086964518], vec![]),
        fz(90, 8, &[], vec![8.0, 0.016442244677005374, 15.360232755371694], vec![]),
        fz(90, 12, &[], vec![12.0, 0.02212694646133105, 942.829606302069], vec![]),
        fz(90, 13, &[], vec![13.0, 0.6589638124975878, 2.150492269391616], vec![]),
        fz(90, 14, &[], vec![14.0, 0.0005286040479022892, 197798.3382206931], vec![]),
        fz(90, 16, &[], vec![16.0, 0.5015236520415853, 119.59921047457372], vec![]),
        fz(90, 17, &[], vec![17.0, 0.005005184070772641, 58.41222708854561], vec![]),
        fz(90, 19, &[], vec![19.0, 0.07830107949285799, 1239.9216711781662], vec![]),
        fz(90, 20, &[], vec![20.0, 0.010596900968222132, 66.6090932858312], vec![]),
        fz(90, 22, &[], vec![22.0, 0.04647557905085862, 21.143596086964518], vec![]),
    ];
    let mut p = program(8, devices);
    p.options.no_rail_shortcut = true;
    p.digital_slot_count = 32;
    p
}

/// Each failure record's device and latch time after running `p` to `end`
/// through `stops`, or in host slices of `slice` when it is positive.
fn latches(p: &Program, stops: &[f64], slice: f64, end: f64) -> Vec<(usize, f64)> {
    let mut e = Engine::load(p).unwrap();
    let mut t: f64 = 0.0;
    let mut k = 0;
    while t < end {
        t = if slice > 0.0 { (t + slice).min(end) } else { stops.get(k).copied().unwrap_or(end).min(end) };
        k += 1;
        e.advance(t).unwrap_or_else(|er| panic!("{} at {t}: {}", er.code, er.message));
    }
    e.records(0).unwrap().chunks(7).map(|c| (c[0] as usize, c[3])).collect()
}

#[test]
fn a_stepped_device_latches_where_its_stress_crosses_inside_a_step() {
    // Ruling R71 (b). Cold 39 tpl 4 (exact mode): rated R20 (0.66 W on 10.6
    // mW) and R17 (0.18 W on 5 mW) share a stepped partition. R20 latched
    // at the end of the step its stress crossed in, 2.1 ms after the fine
    // reference's crossing (19.85 ms), and R17, at 34/s until then, latched
    // with it at 22.0 ms; the fine reference's R17 is down to 6 mW once R20
    // is open and latches at 95.7 ms. The same devices latch in the same
    // order, each within 0.5 ms of the fine reference (exact mode, 1 us host
    // slices) or 10 % of its time: R17 then runs 26 % over its rating, 0.26/s,
    // and turns R20's latch 70 us late (its drive held from the step before)
    // into 7 ms early.
    let stops = [0.00627287967254526, 0.016690206583740867, 0.0170375405909559, 0.017349086141227935, 0.03879249608408733, 0.043625293415744414, 0.04500685709789586, 0.0524890189823995, 0.05265925519389161, 0.0959884882405498, 0.09954981585820734, 0.09985143728957276];
    let p = cold_39_tpl4();
    let want = latches(&p, &[], 1e-6, 0.1);
    let got = latches(&p, &stops, 0.0, 0.1);
    eprintln!("fine {want:?}\ngot  {got:?}");
    assert_eq!(got.iter().map(|l| l.0).collect::<Vec<_>>(), want.iter().map(|l| l.0).collect::<Vec<_>>(), "the devices latched, in order");
    for (g, w) in got.iter().zip(&want) {
        assert!((g.1 - w.1).abs() <= 5e-4_f64.max(0.1 * w.1), "device {} latched at {} against {}", g.0, g.1, w.1);
    }
}

/// tpl=4 hub=2 e=6.111 psu ilim=0.033 CLK3 0.0..8.2 C4-0=3.4e-5 555 vcc1 gnd0 [3, 3, 1, 4, 6, 7] R6-7=20033.8 C3-0=2.1e-7 C0-3=4.0e-7 R0-1=20.1 LED6-3+R1-6=2564 C1-0=5.7e-6 R4-5=25.4 SW0-3(1)+R3-0=6 span=8.160452348801016
/// options: Options { integration: 1, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 8
/// toggles [(0.3254916072076612, 23), (1.1098892139986414, 23), (1.1920327808300362, 23)]
fn cold_507_tpl4_dev() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![6.110964258203888, 0.032971578272426916], vec![]),
        fz(22, 1, &[3, 0], vec![30.331350339428877, 0.2574818098436931, 8.160452348801016, 0.0, 0.0], vec![8.0]),
        fz(1, 2, &[2, 0], vec![41321.66871885181], vec![]),
        fz(1, 3, &[4, 0], vec![449465.19480459753], vec![]),
        fz(1, 4, &[5, 0], vec![52335.203311963065], vec![]),
        fz(1, 5, &[6, 0], vec![62313.92155031614], vec![]),
        fz(1, 6, &[7, 0], vec![7440.6761389539815], vec![]),
        fz(3, 7, &[4, 0], vec![3.3749559355324406e-5, 0.0, 0.0], vec![]),
        fz(40, 8, &[1, 0, 3, 3, 1, 4, 6, 7], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 9, &[1, 7], vec![293.2083758083417], vec![]),
        fz(1, 10, &[7, 6], vec![8309.814722462592], vec![]),
        fz(3, 11, &[6, 0], vec![4.1218012772140167e-7, 0.0, 0.0], vec![]),
        fz(1, 12, &[3, 7], vec![330.575703630589], vec![]),
        fz(1, 13, &[6, 7], vec![20033.84947330294], vec![]),
        fz(1, 14, &[7, 6], vec![48270.35610720781], vec![]),
        fz(3, 15, &[3, 0], vec![2.1087576498729285e-7, 0.0, 0.0], vec![]),
        fz(1, 16, &[4, 6], vec![3250.4019241438664], vec![]),
        fz(3, 17, &[0, 3], vec![3.9684429074772786e-7, 0.0, 0.0], vec![]),
        fz(1, 18, &[0, 1], vec![20.141858433856143], vec![]),
        fz(30, 19, &[6, 3], vec![2.7986823911453116, 2.4923681507892668, 0.03736956562301265, 0.0, 0.0, 0.0], vec![]),
        fz(1, 20, &[1, 6], vec![2563.7471243276555], vec![]),
        fz(3, 21, &[1, 0], vec![5.735486362429675e-6, 0.0, 0.0], vec![]),
        fz(1, 22, &[4, 5], vec![25.439301295085983], vec![]),
        fz(4, 23, &[0, 3], vec![1.0], vec![]),
        fz(1, 24, &[3, 0], vec![6.187583217051827], vec![]),
        fz(90, 9, &[], vec![9.0, 0.00047985441686489855, 293.2083758083417], vec![]),
        fz(90, 10, &[], vec![10.0, 0.00025072927196634554, 8309.814722462592], vec![]),
        fz(90, 12, &[], vec![12.0, 0.0144286459006432, 330.575703630589], vec![]),
        fz(90, 13, &[], vec![13.0, 0.000700845240240721, 20033.84947330294], vec![]),
        fz(90, 18, &[], vec![18.0, 0.3647540562764839, 20.141858433856143], vec![]),
        fz(90, 20, &[], vec![20.0, 0.001281935757022394, 2563.7471243276555], vec![]),
        fz(90, 22, &[], vec![22.0, 1.4552438201559343, 25.439301295085983], vec![]),
        fz(90, 24, &[], vec![24.0, 0.033023898442256144, 6.187583217051827], vec![]),
    ]
}

/// tpl=2 hub=2 e=5.349 psu ilim=0.198 V2=8.60 C3-0=2.5e-3 R2-0=82228.8 C2-0=3.4e-3 SW3-4(1)+R4-0=7 555 vcc1 gnd0 [1, 4, 5, 2, 1, 4] P4-1 LED3-5+R1-3=1695 LED0-4+R0-0=17 R1-2=8446.9 R5-0=1.6 span=13.95377623038553
/// options: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 } digital_slot_count 32 node_count 6
/// toggles [(0.2572420874394904, 8)]
fn cold_55_tpl2_dev() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![5.3492357050722905, 0.19834564145007297], vec![]),
        fz(2, 1, &[0, 2], vec![8.60454052531324], vec![]),
        fz(1, 2, &[3, 0], vec![1416.1518798553122], vec![]),
        fz(1, 3, &[4, 0], vec![13956.243237242461], vec![]),
        fz(1, 4, &[5, 0], vec![5418.783256125151], vec![]),
        fz(3, 5, &[3, 0], vec![0.002451199070188693, 0.0, 0.0], vec![]),
        fz(1, 6, &[2, 0], vec![82228.81012716988], vec![]),
        fz(3, 7, &[2, 0], vec![0.0033715286425486136, 0.0, 0.0], vec![]),
        fz(4, 8, &[3, 4], vec![1.0], vec![]),
        fz(1, 9, &[4, 0], vec![7.052586568880892], vec![]),
        fz(40, 10, &[1, 0, 1, 4, 5, 2, 1, 4], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 11, &[1, 4], vec![2338.0025282828856], vec![]),
        fz(1, 12, &[4, 1], vec![739.8772438040638], vec![]),
        fz(3, 13, &[1, 0], vec![1.5689696594381792e-8, 0.0, 0.0], vec![]),
        fz(1, 14, &[4, 4], vec![327.26372788107744], vec![]),
        fz(13, 15, &[4, 1], vec![0.6362336862897945, 0.012095189234610391], vec![]),
        fz(1, 16, &[2, 4], vec![47983.0207241084], vec![]),
        fz(30, 17, &[3, 5], vec![2.1361035259660164, 2.3141408665244807, 0.023847034029166353, 0.0, 0.0, 0.0], vec![]),
        fz(1, 18, &[1, 3], vec![1695.3741168443169], vec![]),
        fz(1, 19, &[3, 4], vec![16269.422364064012], vec![]),
        fz(30, 20, &[0, 4], vec![3.1913560774879057, 1.7703926526296039, 0.026447246568378758, 0.0, 0.0, 0.0], vec![]),
        fz(1, 21, &[0, 0], vec![17.18607506618329], vec![]),
        fz(1, 22, &[1, 2], vec![8446.91013171194], vec![]),
        fz(1, 23, &[5, 0], vec![1.619732044714196], vec![]),
        fz(90, 6, &[], vec![6.0, 0.00015392461021031858, 82228.81012716988], vec![]),
        fz(90, 9, &[], vec![9.0, 1.6532381237051093, 7.052586568880892], vec![]),
        fz(90, 11, &[], vec![11.0, 0.08118038687391589, 2338.0025282828856], vec![]),
        fz(90, 12, &[], vec![12.0, 0.007824935827833796, 739.8772438040638], vec![]),
        fz(90, 14, &[], vec![14.0, 0.46901696396039405, 327.26372788107744], vec![]),
        fz(90, 18, &[], vec![18.0, 0.018159214364218208, 1695.3741168443169], vec![]),
        fz(90, 21, &[], vec![21.0, 15.979166136012056, 17.18607506618329], vec![]),
        fz(90, 22, &[], vec![22.0, 8.583065830776106e-5, 8446.91013171194], vec![]),
        fz(90, 23, &[], vec![23.0, 107.03694557254273, 1.619732044714196], vec![]),
    ]
}

#[test]
fn a_stepped_resistor_on_a_clock_latches_where_its_stress_integral_reaches_one() {
    // ACC-A, cold 507 tpl 4: rated R7-6 (8.31 kohm, 0.25 mW) in a stepped hub
    // partition on a clock. The high phase's rate was charged back over each
    // low-phase step after the re-seed at its edge, and R7-6 latched at
    // 0.239 s. By hand its integral reaches 0.44 at 0.2 s and the latch at
    // 0.4006 s (the switch closes at 0.3255 s). Within 1 ms of that at 5 ms
    // host stops, either mode.
    let close_at = 0.3254916072076612;
    for exact_mode in [false, true] {
        let mut p = program(8, cold_507_tpl4_dev());
        p.options.integration = 1;
        p.digital_slot_count = 32;
        p.options.no_rail_shortcut = exact_mode;
        let mut stops: Vec<f64> = (1..=90).map(|k| k as f64 * 0.005).collect();
        stops.push(close_at);
        stops.sort_by(|a, b| a.total_cmp(b));
        let mut e = Engine::load(&p).unwrap();
        for &t in &stops {
            e.advance(t).unwrap();
            if t == close_at {
                let v = 1.0 - p.devices[23].params[0];
                e.set_params(&[(23, 0, v)]).unwrap();
            }
        }
        let r7_6 = e.records(0).unwrap().chunks(7).find(|c| c[0] == 10.0).map(|c| c[3]);
        eprintln!("507/4 exact {exact_mode}: R7-6 latched at {r7_6:?}, by hand 0.4006");
        assert!(r7_6.is_some_and(|t| (t - 0.4006).abs() <= 1e-3), "exact {exact_mode}: R7-6 latched at {r7_6:?}");
    }
}

#[test]
fn a_stepped_ptc_trips_where_its_stress_reaches_the_threshold_whatever_the_host_step() {
    // ACC-A, cold 55 tpl 2: the PTC (0.636 ohm, hold 12.1 mA) carries the CC
    // rail's 198 mA less 1.1 mA elsewhere into 2.45 mF: by hand S' = I^2 -
    // Ih^2 = 0.0387/s reaches 0.3 Ih^2 = 4.39e-5 at 1.135 ms. It tripped at
    // the host's step, 1.14, 1.2 and 2.0 ms at 10 us, 0.1 and 1 ms hosts.
    // Within 10 us of the hand time at every host step in both modes (each
    // step closes the PTC at its own end voltages and the rail its commit
    // found: read as published after the rail loop moved it, default mode
    // tripped 0.105 ms late at 1 ms hosts).
    for (exact_mode, slice, tol) in [(true, 1e-5, 1e-5), (true, 1e-4, 1e-5), (true, 1e-3, 1e-5), (false, 1e-5, 1e-5), (false, 1e-4, 1e-5), (false, 1e-3, 1e-5)] {
        let mut p = program(6, cold_55_tpl2_dev());
        p.digital_slot_count = 32;
        p.options.no_rail_shortcut = exact_mode;
        let mut e = Engine::load(&p).unwrap();
        let mut t: f64 = 0.0;
        while t < 0.0025 {
            t = (t + slice).min(0.0025);
            e.advance(t).unwrap();
        }
        let trip = e.records(2).unwrap().chunks(7).find(|c| c[0] == 15.0).map(|c| c[3]);
        eprintln!("55/2 exact {exact_mode}, {slice} s hosts: tripped at {trip:?}, by hand 1.135e-3");
        assert!(trip.is_some_and(|t| (t - 1.135e-3).abs() <= tol), "exact {exact_mode}, {slice} s hosts: tripped at {trip:?}");
    }
}

#[test]
fn exact_mode_charges_cold_84_tpl1s_led_its_drain_and_not_the_steps_end_rate() {
    // R71 (b): exact mode, the fine reference, latched the LED at the first
    // fall (the end rate, 7.9e8 per second, over the 2.07 ms step ending at
    // the edge). With the step's own end drive, its seeded edge and the latch
    // inside the step, it survives the first fall (S 12.3 by its BE steps
    // across the 37 ns drain, by hand 14.71, the latch at 24) and burns at
    // the second, as default mode does.
    let mut p = program(6, cold_84_tpl1());
    p.digital_slot_count = 32;
    p.options.no_rail_shortcut = true;
    let mut rig = Rig::load(&p).unwrap();
    rig.advance(0.0021).unwrap();
    assert!(!rig.s().slow.is_failed(10), "the LED survives the first fall");
    rig.advance(0.0054).unwrap();
    assert!(!rig.s().slow.is_failed(10), "still intact before the second fall");
    rig.advance(0.0055).unwrap();
    assert!(rig.s().slow.is_failed(10), "the second fall burns it");
}

/// tpl=3 hub=2 e=13.685 psu ilim=0.082 SW1-3(1)+R3-0=1550 R3-1=1463.4 LED3-4+R1-3=979 C4-0=1.8e-3 C5-0=1.4e-3 R4-5=257.7
fn cold_675_tpl3() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![13.684902848374957, 0.08161949607297277], vec![]),
        fz(1, 1, &[2, 0], vec![281466.365342136], vec![]),
        fz(1, 2, &[3, 0], vec![236475.01481167166], vec![]),
        fz(1, 3, &[4, 0], vec![84875.99148766494], vec![]),
        fz(1, 4, &[5, 0], vec![738786.6382091894], vec![]),
        fz(1, 5, &[6, 0], vec![54290.934422395025], vec![]),
        fz(4, 6, &[1, 3], vec![1.0], vec![]),
        fz(1, 7, &[3, 0], vec![1550.3322954002194], vec![]),
        fz(1, 8, &[3, 1], vec![1463.3971383123571], vec![]),
        fz(30, 9, &[3, 4], vec![3.0660076479583918, 2.3847008349690926, 0.01299316214029995, 0.0, 0.0, 0.0], vec![]),
        fz(1, 10, &[1, 3], vec![978.8840905575431], vec![]),
        fz(3, 11, &[4, 0], vec![0.0018388727366312652, 0.0, 0.0], vec![]),
        fz(3, 12, &[5, 0], vec![0.0013660329587277412, 0.0, 0.0], vec![]),
        fz(1, 13, &[4, 5], vec![257.6903587652661], vec![]),
        fz(90, 7, &[], vec![7.0, 0.044767533012357086, 1550.3322954002194], vec![]),
        fz(90, 8, &[], vec![8.0, 0.0005065528999520752, 1463.3971383123571], vec![]),
        fz(90, 10, &[], vec![10.0, 0.0010837273211185066, 978.8840905575431], vec![]),
        fz(90, 13, &[], vec![13.0, 0.2449639729867886, 257.6903587652661], vec![]),
    ]
}

/// tpl=3 hub=2 e=2.100 psu ilim=0.040 F0-1=1.44/0.02 R4-7=28.9 R6-1=115.2 C2-0=3.1e-5 555 vcc6 gnd0 [2, 6, 5, 4, 7, 3] R1-4=2727.8 R3-6=1794.4 P7-6
fn cold_656_tpl3() -> Vec<Device> {
    vec![
        fz(21, 0, &[1, 0], vec![2.0997551283597415, 0.03993181995397193], vec![]),
        fz(1, 1, &[2, 0], vec![70499.12728765282], vec![]),
        fz(1, 2, &[3, 0], vec![104322.4772122887], vec![]),
        fz(1, 3, &[4, 0], vec![2680.476161387651], vec![]),
        fz(1, 4, &[5, 0], vec![32788.0180586396], vec![]),
        fz(1, 5, &[6, 0], vec![156820.77936240626], vec![]),
        fz(1, 6, &[7, 0], vec![405527.29783161636], vec![]),
        fz(10, 7, &[0, 1], vec![1.4402208057314996, 0.02038315706881638], vec![]),
        fz(1, 8, &[4, 7], vec![28.911521433020177], vec![]),
        fz(1, 9, &[6, 1], vec![115.16380451031051], vec![]),
        fz(3, 10, &[2, 0], vec![3.109340611974017e-5, 3.315715846515984, 0.0], vec![]),
        fz(40, 11, &[6, 0, 2, 6, 5, 4, 7, 3], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 12, &[6, 3], vec![37505.864107821646], vec![]),
        fz(1, 13, &[3, 7], vec![20757.966784553708], vec![]),
        fz(3, 14, &[7, 0], vec![6.936969678706147e-8, 0.0, 0.0], vec![]),
        fz(1, 15, &[6, 3], vec![1279.3335181110078], vec![]),
        fz(1, 16, &[1, 4], vec![2727.7631154965725], vec![]),
        fz(1, 17, &[3, 6], vec![1794.363266277742], vec![]),
        fz(13, 18, &[7, 6], vec![0.570414384883887, 0.127126112188416], vec![]),
        fz(90, 8, &[], vec![8.0, 0.056515929711860466, 28.911521433020177], vec![]),
        fz(90, 9, &[], vec![9.0, 0.00023305991130195925, 115.16380451031051], vec![]),
        fz(90, 12, &[], vec![12.0, 2.93155864453981e-6, 37505.864107821646], vec![]),
        fz(90, 13, &[], vec![13.0, 3.97714993323673e-6, 20757.966784553708], vec![]),
        fz(90, 15, &[], vec![15.0, 1.1961440614496167e-5, 1279.3335181110078], vec![]),
        fz(90, 16, &[], vec![16.0, 0.00015047479414880447, 2727.7631154965725], vec![]),
        fz(90, 17, &[], vec![17.0, 0.0001101907431417095, 1794.363266277742], vec![]),
    ]
}

#[test]
fn stepped_resistors_idle_until_a_switch_opens_latch_on_the_stress_after_it() {
    // REF, cold 675 tpl 3: the rail-node 3 switch opens at 0.2958 s and puts
    // R8 (1463 ohm, 0.507 mW) and R10 (979 ohm, 1.08 mW), idle since load,
    // across 3.78 V. By hand R8 runs at 18.2/s and latches at 0.3508 s;
    // then node 3 falls to 8.39 V and R10, at 0.683, runs at 25.4/s and
    // latches at 0.3633 s. Both latched at the toggle, the post-toggle rate
    // charged over [0, 0.2958] (S = 5.4). Within 1 ms of the hand times.
    let toggle = 0.2958471421807681;
    for exact_mode in [false, true] {
        let mut p = program(7, cold_675_tpl3());
        p.digital_slot_count = 32;
        p.options.no_rail_shortcut = exact_mode;
        let mut e = Engine::load(&p).unwrap();
        e.advance(toggle).unwrap();
        let v = 1.0 - p.devices[6].params[0];
        e.set_params(&[(6, 0, v)]).unwrap();
        e.advance(0.4).unwrap();
        let rec = e.records(0).unwrap().to_vec();
        let at = |d: f64| rec.chunks(7).find(|c| c[0] == d).map(|c| c[3]);
        let (r8, r10) = (at(8.0), at(10.0));
        eprintln!("675/3 exact {exact_mode}: R8 {r8:?} (by hand 0.3508), R10 {r10:?} (0.3633)");
        assert!(r8.is_some_and(|t| (t - 0.3508).abs() <= 1e-3), "exact {exact_mode}: R8 at {r8:?}");
        assert!(r10.is_some_and(|t| (t - 0.3633).abs() <= 1e-3), "exact {exact_mode}: R10 at {r10:?}");
    }
}

#[test]
fn stepped_resistors_idle_until_a_fuse_burns_take_no_damage_from_before_it() {
    // REF, cold 656 tpl 3: a fuse from the rail to ground holds the {2.10 V,
    // 40 mA} PSU in CC at 57 mV until it latches at 0.105865 s. R9 (115 ohm,
    // 0.233 mW) and R16 (2728 ohm, 0.150 mW), idle until then, latched at the
    // fuse with S = 76.7 and 4.86, their post-burn rates charged from load.
    // By hand their damage at the burn is 0 and grows from it at R9's 155/s
    // at most. Neither latches at the burn instant. Just past it the 555 at
    // node 6 reaches its window's floor through R9: at 1.8 V its OUT pair
    // draws 22 mA where R9 gives 1.7 mA, so it has no consistent power state.
    // Default mode holds it at its stepped partition's seed (W1c point 1a),
    // retries it as its supply passes the release level, and declines through
    // ruling R54 (a)'s guard (51 short holds); it failed the seed's Newton,
    // with no hold. Exact mode's own hold is REF's item.
    for exact_mode in [false, true] {
        let mut p = program(8, cold_656_tpl3());
        p.digital_slot_count = 32;
        p.options.no_rail_shortcut = exact_mode;
        let mut e = Engine::load(&p).unwrap();
        e.advance(0.10587).unwrap();
        let rec: Vec<(f64, f64)> = e.records(0).unwrap().chunks(7).map(|c| (c[0], c[3])).collect();
        eprintln!("656/3 exact {exact_mode}: {rec:?}");
        assert!(rec.iter().any(|r| r.0 == 7.0), "exact {exact_mode}: the fuse latched");
        assert!(!rec.iter().any(|r| r.0 == 9.0 || r.0 == 16.0), "exact {exact_mode}: R9 or R16 latched at the fuse: {rec:?}");
        if !exact_mode {
            let err = e.advance(0.2).expect_err("the 555 slides along its power edge");
            assert!(err.code == -4 && err.message.contains("sliding episode"), "the episode guard: {} {}", err.code, err.message);
        }
    }
}

#[test]
fn exact_mode_searches_a_stepped_rail_partition_and_holds_a_chip_at_its_landing() {
    // REF (1), ruling R69: exact mode's CC entry skipped a stepped rail
    // partition. PSU 5 V, 50 mA; 125 ohm on the rail; a 74LS00 with 1A, 1B
    // and 1Y on the rail; 1 k from the rail to 1 uF (node 3, from 0 V), which
    // makes the rail's partition stepped. By hand: powered, inside [4.275,
    // 5.775] V, 1Y LOW sinks 32 mA, so at the setpoint the draw is 40 + 32 +
    // (5 - vc)/1k >= 72 mA, past the limit; through the window it stays at
    // or above 4.275/125 + 32 = 66.2 mA, and under the floor, the chip off,
    // it is at most 34.2 + 4.3 = 38.5 mA: the draw meets the limit only
    // across the power edge, the landing. The chip held off, the PSU is in
    // CV at 5 V delivering 5/125 + (5 - vc)/1k, the capacitor charging on
    // tau = 1 ms: vc = 5 (1 - e^(-t/tau)). Before the fix exact mode held
    // CV with the chip free, 72 mA and more through a 50 mA supply.
    let board = || window_chip_rail(5.0, 0.05, 125.0, 0, [1, 1], 1, vec![resistor(3, 1, 3, 1e3), capacitor(4, 3, 0, 1e-6, 0.0)]);
    let mut rig = Rig::load(&exact(board())).unwrap();
    let p = rig.s().exact.as_ref().unwrap().partition;
    assert_ne!(rig.s().partition_kinds[p], PartitionKind::Static, "the rail's partition steps");
    for k in 1..=8 {
        let t = k as f64 * 0.5e-3;
        rig.advance(t).unwrap();
        let at = format!("at {:.1} ms", t * 1e3);
        close(rig.v(1), 5.0, 1e-12, &format!("{at}: CV at the setpoint"));
        assert_eq!(exact_regime(&rig), Regime::Cv, "{at}");
        assert!(!rig.s().power_hold(2).is_free(), "{at}: the chip is held off");
        let vc = 5.0 * (1.0 - (-t / 1e-3).exp());
        close(rig.v(3), vc, 0.02 * 5.0, &format!("{at}: the capacitor charging on 1 ms"));
        // The rail's KCL with the chip off: the resistor and the RC branch.
        close(rig.i(0), -(5.0 / 125.0 + (5.0 - rig.v(3)) / 1e3), 1e-9, &format!("{at}: the PSU's current"));
        assert!(-rig.i(0) <= 0.05 * (1.0 + 1e-9), "{at}: within the limit, {} A", -rig.i(0));
    }
}

/// Cold fuzz 84 tpl 1 as its sweep ran it (`f5e gen 84 1`): a {10.65 V,
/// 92 mA} PSU with 2.65 uF on the rail and an LED from the rail to a 292 Hz
/// clock's node (-3.31 / 9.51 V), no series resistance.
fn ref_cold_84_tpl1() -> Program {
    let mut p = program(
        6,
        vec![
            fz(21, 0, &[1, 0], vec![10.647486522334537, 0.0923007229999578], vec![]),
            fz(22, 1, &[3, 0], vec![292.4270639400117, 0.6067396404390497, 9.512811525710633, -3.3131884104269833, 0.0], vec![8.0]),
            fz(2, 2, &[4, 3], vec![1.0658932999473136], vec![]),
            fz(1, 3, &[2, 0], vec![11183.931130067276], vec![]),
            fz(1, 4, &[5, 0], vec![3653.7769026175633], vec![]),
            fz(3, 5, &[1, 0], vec![2.654722241109152e-6, 0.0, 0.0], vec![]),
            fz(1, 6, &[2, 3], vec![22.13610172025595], vec![]),
            fz(1, 7, &[2, 3], vec![84817.38849207966], vec![]),
            fz(1, 8, &[3, 5], vec![8.115792481404343], vec![]),
            fz(1, 9, &[4, 5], vec![104.78146855709932], vec![]),
            fz(30, 10, &[1, 3], vec![2.6844845184488806, 2.2666570131156947, 0.02805793538661563, 0.0, 0.0, 0.0], vec![]),
            fz(1, 11, &[1, 1], vec![123.25751103521999], vec![]),
            fz(50, 12, &[1, 0, 5, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 5], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.5215933255689305e-7], vec![0.0; 13]),
            fz(1, 13, &[5, 1], vec![258.85338726766065], vec![]),
            fz(1, 14, &[0, 1], vec![131.22013358699968], vec![]),
            fz(90, 6, &[], vec![6.0, 0.10039255154394434, 22.13610172025595], vec![]),
            fz(90, 7, &[], vec![7.0, 1.144579996054986e-5, 84817.38849207966], vec![]),
            fz(90, 8, &[], vec![8.0, 29.322249323215566, 8.115792481404343], vec![]),
            fz(90, 9, &[], vec![9.0, 0.004031042336012474, 104.78146855709932], vec![]),
            fz(90, 11, &[], vec![11.0, 0.04323459645416291, 123.25751103521999], vec![]),
            fz(90, 13, &[], vec![13.0, 0.2251908463627935, 258.85338726766065], vec![]),
            fz(90, 14, &[], vec![14.0, 0.003338164034755094, 131.22013358699968], vec![]),
        ],
    );
    p.digital_slot_count = 32;
    p
}

#[test]
fn exact_mode_closes_a_stepped_step_that_ends_at_a_clock_edge_with_its_own_end_drive() {
    // Not fail-first on the tree it landed on: B's trail (8785c63) had
    // already fixed both modes, so this pins the exact-mode case only.
    // REF (4), cold 84 tpl 1 in exact mode, the LED in the stepped rail
    // partition. By hand (ruling R68's analysis): at the clock's first fall,
    // to -3.31 V, the LED (vf 2.68 V, n 2.27, ifMax 28.06 mA) takes 789.6 A
    // from the rail capacitor charged to 10.65 V; past its knee its law is a
    // line of 71.5 S, so the capacitor drains on tau = C / g = 37.1 ns to
    // -0.563 V, the PSU's 92 mA aside. The damage integral of that drain is
    // (I0 / ifMax)^2 tau / 2 = 14.71, under the latch's 24 (led_failed, rate
    // (I / ifMax)^2 - 1): the LED survives the first fall. Over the high
    // phase before it, the LED at the setpoint less the clock's 9.51 V, 1.14
    // V, is far under its knee: no damage. Before the fix the LED, idle
    // since the load, was never closed (its integrator stood at t = 0), and
    // the first step to drive it, the fall's, closed with its end rate, 7.9e8
    // per second, over the whole 2.07 ms: latched there with 1.6e6.
    let mut rig = Rig::load(&exact(ref_cold_84_tpl1())).unwrap();
    let p = rig.s().exact.as_ref().unwrap().partition;
    assert_ne!(rig.s().partition_kinds[p], PartitionKind::Static, "the rail's partition steps");
    let fall = 0.6067396404390497 / 292.4270639400117;
    rig.advance(fall - 1e-6).unwrap();
    let s0 = rig.s().slow.stress(10).map_or(0.0, |s| s.value_at(fall - 1e-6));
    close(s0, 0.0, 1e-9, "no damage over the high phase");
    rig.advance(fall + 2e-6).unwrap();
    assert!(!rig.s().slow.is_failed(10), "the LED survives the first fall");
    // Each step holds the mean of its start and end drives (rulings R80
    // (a), R84 (B)); the first after the fall keeps its end rate. The window
    // is 10 % either side for this board only (R84): the drain the core
    // steps is no single exponential, the trapezoidal step ringing on it
    // (node 5 at -2.90, -2.53, -3.02 V while the rail capacitor falls 8.30,
    // 5.26, 4.06, 2.50 V), so any two-point mean misses the hand
    // exponential's integral: 15.86 measured with ruling R90 (a)'s step cap
    // (16.08 without it), 12.3 under classic's backward rule. The ringing is
    // the trapezoidal step's own, which no stress-side cap removes.
    let s1 = rig.s().slow.stress(10).map(|s| s.value_at(fall + 2e-6)).unwrap();
    assert!((s1 - 14.71).abs() <= 0.10 * 14.71, "the first drain's damage integral {s1} against 14.71 by hand");
}

/// Cold fuzz 789 tpl 3 as its sweep ran it (`f5e gen 789 3`), to 0.3 s (its
/// first toggle is at 0.49 s).
fn ref_cold_789_tpl3() -> Program {
    let mut p = program(
        6,
        vec![
            fz(21, 0, &[1, 0], vec![5.702815114093951, 0.10709034605756618], vec![]),
            fz(2, 1, &[2, 0], vec![-2.1223983582616253], vec![]),
            fz(22, 2, &[3, 0], vec![6.175657925034204, 0.13632525323120667, 11.637408847739295, 0.0, 0.0], vec![8.0]),
            fz(1, 3, &[4, 0], vec![74866.59775382324], vec![]),
            fz(1, 4, &[5, 0], vec![692863.7348067253], vec![]),
            fz(30, 5, &[0, 3], vec![1.8144324704895112, 2.3694818343536106, 0.015605014551019997, 0.0, 0.0, 0.0], vec![]),
            fz(1, 6, &[0, 0], vec![17.50282632860529], vec![]),
            fz(1, 7, &[1, 0], vec![310.0258546740256], vec![]),
            fz(50, 8, &[1, 0, 1, 2, 5, 0, 0, 3, 0, 0, 3, 0, 0, 5], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 7.982905264005229e-7], vec![0.0; 13]),
            fz(1, 9, &[5, 1], vec![618.691378274752], vec![]),
            fz(1, 10, &[3, 0], vec![158.8410539324055], vec![]),
            fz(3, 11, &[5, 0], vec![0.004441243309138362, 0.862470934417167, 0.0], vec![]),
            fz(4, 12, &[0, 2], vec![0.0], vec![]),
            fz(1, 13, &[2, 0], vec![226.01169158568942], vec![]),
            fz(1, 14, &[5, 2], vec![12329.602924622965], vec![]),
            fz(3, 15, &[3, 0], vec![0.0001454098095747667, 0.0, 0.0], vec![]),
            fz(1, 16, &[4, 5], vec![2450.045329825011], vec![]),
            fz(4, 17, &[0, 4], vec![1.0], vec![]),
            fz(1, 18, &[4, 0], vec![24.10011433800014], vec![]),
            fz(90, 6, &[], vec![6.0, 4.008861975871273, 17.50282632860529], vec![]),
            fz(90, 7, &[], vec![7.0, 0.03208795361041518, 310.0258546740256], vec![]),
            fz(90, 9, &[], vec![9.0, 0.050235466518358, 618.691378274752], vec![]),
            fz(90, 10, &[], vec![10.0, 0.1978743367396583, 158.8410539324055], vec![]),
            fz(90, 13, &[], vec![13.0, 0.004105486433659165, 226.01169158568942], vec![]),
            fz(90, 18, &[], vec![18.0, 0.5190131069074609, 24.10011433800014], vec![]),
        ],
    );
    p.digital_slot_count = 32;
    p
}

#[test]
fn exact_mode_holds_a_gate_sourcing_into_a_clock_low_on_a_stepped_rail_partition() {
    // REF (1), cold 789 tpl 3 (ruling R69's pin). PSU 5.703 V, 107 mA; 310
    // ohm on the rail; a 74LS00 on it whose 3Y and 4Y (HIGH: inputs on
    // ground) drive a 0 / 11.64 V clock's node (158.8 ohm, 145 uF and an LED
    // there), and whose 1Y, 4Y drive node 5, which holds 4.44 mF and 618.7
    // ohm back to the rail: a stepped rail partition. By hand, clock low
    // (22.1 to 161.9 ms): the two HIGH outputs source 64 mA into it, and with
    // node 5's charging current the draw at the setpoint is 147 mA, past the
    // limit, and stays past it through the window [4.275, 5.775] V; under its
    // floor, the chip off, it is V/310 + (V - v5)/618.7, under 30 mA: a
    // landing. The chip held off, the PSU is in CV at the setpoint
    // delivering V/310.03 + (V - v5)/618.69 (26.99 mA at 38.6 ms, v5 = 0.385
    // V). Clock high again, the chip carries itself in CV and is free at the
    // next host step's retry.
    // Before the fix exact mode, the fine reference, held CV with the chip
    // free at 147.3 mA (682 W0 against default mode at 0.16 s).
    let v = 5.702815114093951;
    let mut rig = Rig::load(&exact(ref_cold_789_tpl3())).unwrap();
    let p = rig.s().exact.as_ref().unwrap().partition;
    assert_ne!(rig.s().partition_kinds[p], PartitionKind::Static, "the rail's partition steps");
    for t in [0.03, 0.0386, 0.06, 0.1, 0.15] {
        rig.advance(t).unwrap();
        let at = format!("clock low, {:.1} ms", t * 1e3);
        assert!(rig.v(3) < 1e-9, "{at}: the clock is low");
        close(rig.v(1), v, 1e-12, &format!("{at}: CV at the setpoint"));
        assert_eq!(exact_regime(&rig), Regime::Cv, "{at}");
        assert!(!rig.s().power_hold(8).is_free(), "{at}: the chip is held off");
        let want = v / 310.0258546740256 + (v - rig.v(5)) / 618.691378274752;
        rel(-rig.i(0), want, 1e-6, &format!("{at}: the PSU's current, the chip off"));
        assert!(-rig.i(0) < 0.03, "{at}: well within the limit");
    }
    // A held chip is retried once per host step (ruling R51 (b)): 1 ms ones.
    for k in 151..=175 {
        rig.advance(k as f64 * 1e-3).unwrap();
    }
    assert!(rig.v(3) > 11.0, "the clock is high");
    assert!(rig.s().power_hold(8).is_free(), "clock high: the chip is free again");
    assert_eq!(exact_regime(&rig), Regime::Cv);
    assert!(-rig.i(0) <= 0.10709034605756618, "clock high: within the limit");
}

#[test]
fn exact_mode_holds_a_landing_whose_powered_probe_has_no_cc_solution() {
    // REF (2), cold 794 tpl 4. PSU 11.965 V, 455 mA; a 555 on the rail with
    // TRIG and THR on a -9.05 V node and DIS, OUT on a clock's node (-2.04 /
    // 7.87 V, 16.76 Hz, high for 44.1 ms of 59.7); 59.6 ohm and 611 ohm from
    // the rail to the clock's node (the 611 ohm, rated 1.5 mW, burns at the
    // first low phase). By hand, clock low: powered, the 555 draws 1.05 A at
    // the setpoint, past the limit, and the draw stays past it down its
    // window to 1.8 V; under the floor, the 555 off, it is (u + 2.04)/59.6,
    // 64 mA at the floor: a landing. The 555 held off, the PSU is in CV at
    // the setpoint delivering (11.965 + 2.037)/59.61 = 234.9 mA. The powered
    // probe of the hold procedure (the 555 held on, the PSU in CC) has no
    // solution its Newton reaches; before the fix exact mode then kept the
    // CC solution and its clamp's CV, 1.277 A against the 455 mA limit,
    // through every low phase.
    let board = || {
    let mut p = program(
        7,
        vec![
            fz(21, 0, &[1, 0], vec![11.96504207843962, 0.4552508446732851], vec![]),
            fz(2, 1, &[0, 2], vec![9.046538307982303], vec![]),
            fz(22, 2, &[3, 0], vec![16.75594445426356, 0.739333047096713, 7.868200537211464, -2.0372010795942597, 0.0], vec![8.0]),
            fz(1, 3, &[4, 0], vec![46566.41285113872], vec![]),
            fz(1, 4, &[5, 0], vec![2849.7521078903405], vec![]),
            fz(1, 5, &[6, 0], vec![3662.158505283313], vec![]),
            fz(40, 6, &[1, 0, 2, 2, 1, 0, 3, 3], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            fz(1, 7, &[1, 3], vec![611.0370664295151], vec![]),
            fz(1, 8, &[3, 3], vec![4376.957214293443], vec![]),
            fz(3, 9, &[3, 0], vec![1.0551435497994568e-6, 0.0, 0.0], vec![]),
            fz(1, 10, &[2, 2], vec![2150.5584869417153], vec![]),
            fz(1, 11, &[1, 3], vec![59.610495360161075], vec![]),
            fz(1, 12, &[4, 6], vec![2994.110322297871], vec![]),
            fz(90, 7, &[], vec![7.0, 0.0015265366482540331, 611.0370664295151], vec![]),
            fz(90, 8, &[], vec![8.0, 0.0047824577690048385, 4376.957214293443], vec![]),
            fz(90, 10, &[], vec![10.0, 0.041619826286586226, 2150.5584869417153], vec![]),
            fz(90, 11, &[], vec![11.0, 0.6257606828479856, 59.610495360161075], vec![]),
        ],
    );
    p.digital_slot_count = 32;
    p
    };
    let v = 11.96504207843962;
    for (mode, p) in [("default", board()), ("exact", exact(board()))] {
        let mut rig = Rig::load(&p).unwrap();
        for t in [0.046, 0.05, 0.058, 0.106, 0.118] {
            rig.advance(t).unwrap();
            let at = format!("{mode}, clock low, {:.1} ms", t * 1e3);
            assert!(rig.v(3) < -2.0, "{at}: the clock is low");
            assert!(rig.s().slow.is_failed(7), "{at}: the 611 ohm burnt");
            close(rig.v(1), v, 1e-12, &format!("{at}: CV at the setpoint"));
            assert_eq!(exact_regime(&rig), Regime::Cv, "{at}");
            assert!(!rig.s().power_hold(6).is_free(), "{at}: the 555 is held off");
            rel(-rig.i(0), (v - rig.v(3)) / 59.610495360161075, 1e-6, &format!("{at}: the PSU's current, 234.9 mA"));
        }
    }
}

#[test]
fn exact_mode_holds_a_555_sliding_along_its_power_edge_and_burns_nothing_at_the_fuse() {
    // REF, cold 656 tpl 3 in exact mode. A {2.1 V, 39.9 mA} PSU shorted by a
    // 1.44 ohm, 20.4 mA fuse from ground to the rail: CC at 0.0575 V until
    // the fuse latches (about 0.107 s), then CV at 2.1 V. A 555 takes its
    // supply (node 6) through 115.2 ohm from the rail, with 69 nF behind a
    // 0.57 ohm PTC on node 7. By hand: powered at its 1.8 V floor, its OUT
    // Thevenin pair (40 ohm to each supply pin there) draws vs / 80 = 22 mA, far
    // past the (2.0 - 1.8) / 115 = 1.7 mA its feed can carry there, so it
    // pulls its supply under the floor within a microsecond of the 69 nF;
    // unpowered, node 6 recovers on 115 ohm * 69 nF = 8 us: it slides along
    // its power edge for good, the case W1c point 1a holds and ruling R54
    // (a)'s episode guard declines. The 115 ohm (pMax 0.23 mW) and the 2.73
    // k (0.15 mW) carry nothing until the fuse opens, so neither can latch
    // at that instant: before the fix (stale t0) both latched there, their
    // post-burn rates charged from t = 0 (S = 76.7 and 4.86), and the 555
    // lost its feed; with them intact the stepped seed's Newton then failed
    // outright at the edge (-4) instead of holding the chip.
    let mut p = program(8, vec![
        fz(21, 0, &[1, 0], vec![2.0997551283597415, 0.03993181995397193], vec![]),
        fz(1, 1, &[2, 0], vec![70499.12728765282], vec![]),
        fz(1, 2, &[3, 0], vec![104322.4772122887], vec![]),
        fz(1, 3, &[4, 0], vec![2680.476161387651], vec![]),
        fz(1, 4, &[5, 0], vec![32788.0180586396], vec![]),
        fz(1, 5, &[6, 0], vec![156820.77936240626], vec![]),
        fz(1, 6, &[7, 0], vec![405527.29783161636], vec![]),
        fz(10, 7, &[0, 1], vec![1.4402208057314996, 0.02038315706881638], vec![]),
        fz(1, 8, &[4, 7], vec![28.911521433020177], vec![]),
        fz(1, 9, &[6, 1], vec![115.16380451031051], vec![]),
        fz(3, 10, &[2, 0], vec![3.109340611974017e-5, 3.315715846515984, 0.0], vec![]),
        fz(40, 11, &[6, 0, 2, 6, 5, 4, 7, 3], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 12, &[6, 3], vec![37505.864107821646], vec![]),
        fz(1, 13, &[3, 7], vec![20757.966784553708], vec![]),
        fz(3, 14, &[7, 0], vec![6.936969678706147e-8, 0.0, 0.0], vec![]),
        fz(1, 15, &[6, 3], vec![1279.3335181110078], vec![]),
        fz(1, 16, &[1, 4], vec![2727.7631154965725], vec![]),
        fz(1, 17, &[3, 6], vec![1794.363266277742], vec![]),
        fz(13, 18, &[7, 6], vec![0.570414384883887, 0.127126112188416], vec![]),
        fz(90, 8, &[], vec![8.0, 0.056515929711860466, 28.911521433020177], vec![]),
        fz(90, 9, &[], vec![9.0, 0.00023305991130195925, 115.16380451031051], vec![]),
        fz(90, 12, &[], vec![12.0, 2.93155864453981e-6, 37505.864107821646], vec![]),
        fz(90, 13, &[], vec![13.0, 3.97714993323673e-6, 20757.966784553708], vec![]),
        fz(90, 15, &[], vec![15.0, 1.1961440614496167e-5, 1279.3335181110078], vec![]),
        fz(90, 16, &[], vec![16.0, 0.00015047479414880447, 2727.7631154965725], vec![]),
        fz(90, 17, &[], vec![17.0, 0.0001101907431417095, 1794.363266277742], vec![]),
    ]);
    p.digital_slot_count = 32;
    let mut rig = Rig::load(&exact(p)).unwrap();
    let mut failure = None;
    for k in 1..=200 {
        if let Err(e) = rig.advance(k as f64 * 1e-3) {
            failure = Some(format!("{e:?}"));
            break;
        }
        if rig.s().slow.is_failed(7) {
            assert!(!rig.s().slow.is_failed(9) && !rig.s().slow.is_failed(16), "at {k} ms: the 115 ohm and the 2.73 k outlive the fuse's instant");
        }
    }
    assert!(rig.s().slow.is_failed(7), "the fuse latched");
    if let Some(e) = failure {
        assert!(e.contains("power hold") && e.contains("device 11"), "only the sliding-episode guard declines: {e}");
    }
    assert!(rig.s().hold_counts()[0] > 0.0, "the 555 was held at its edge");
}

#[test]
fn a_landing_whose_powered_probe_sits_in_cv_past_the_limit_holds_the_chip_off_in_both_modes() {
    // Ruling R74, cold 471 tpl 2 and 673 tpl 0. Each 74LS00 sits on the rail
    // with HIGH outputs into a node held below ground (471: 2Y, 3Y into a
    // -4.6 V source, 1Y, 4Y into node 3 behind 98.7 and 263.5 ohm to it; 673:
    // 2Y, 3Y into ground, 1Y, 4Y into node 4), drivers that sit in a hub
    // partition. By hand: powered, inside [4.275, 5.775] V, the chip draws
    // 190 mA (471) and 76 mA (673) at the setpoint, far past the 55 and 23.5
    // mA limits, and stays past them down its window; under its floor, off,
    // it draws nothing: on 471 the rail holds only the chip, 0 mA; on 673
    // the rail feeds 1443 ohm to node 3 and 62.7 k to ground, (V - v3)/1443
    // + V/62695 = 5.87 mA. A landing at the floor: the chip held off, CV at
    // the setpoint delivering 0 and 5.87 mA, which exact mode, its trials
    // solving the partitions at each rail, already gave. Before the fix
    // default mode's linearised curve kept the chip's partition draw under
    // its floor (64 mA on 471), placed the CC root past the setpoint, and the
    // clamp held CV at 190 and 76 mA.
    let b471 = || {
        let mut p = program(7, vec![
        fz(21, 0, &[1, 0], vec![4.751886091509621, 0.054961332725668104], vec![]),
        fz(2, 1, &[2, 0], vec![-4.596049646086812], vec![]),
        fz(1, 2, &[3, 0], vec![494452.0913392493], vec![]),
        fz(1, 3, &[4, 0], vec![77429.4854565049], vec![]),
        fz(1, 4, &[5, 0], vec![78194.52499104275], vec![]),
        fz(1, 5, &[6, 0], vec![2675.744432386028], vec![]),
        fz(1, 6, &[2, 3], vec![98.66153046803586], vec![]),
        fz(50, 7, &[1, 0, 5, 2, 3, 0, 0, 2, 0, 0, 2, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 3.3711834227987145e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 8, &[3, 2], vec![263.508620325762], vec![]),
        fz(1, 9, &[2, 0], vec![102.06709251544831], vec![]),
        fz(3, 10, &[0, 0], vec![0.008790905891188373, 0.1361737485557175, 0.0], vec![]),
        fz(1, 11, &[3, 5], vec![13563.322422657602], vec![]),
        fz(30, 12, &[3, 6], vec![2.645442738804019, 1.9851176644289632, 0.03894033893409811, 0.0, 0.0, 0.0], vec![]),
        fz(90, 6, &[], vec![6.0, 0.05757475196045802, 98.66153046803586], vec![]),
        fz(90, 8, &[], vec![8.0, 0.019609209561770004, 263.508620325762], vec![]),
        fz(90, 9, &[], vec![9.0, 0.010313525627732776, 102.06709251544831], vec![]),
        fz(90, 11, &[], vec![11.0, 0.00012125106711201618, 13563.322422657602], vec![]),
        ]);
        p.digital_slot_count = 32;
        p
    };
    let b673 = || {
        let mut p = program(5, vec![
        fz(21, 0, &[1, 0], vec![4.714141438666053, 0.023491781958795588], vec![]),
        fz(2, 1, &[0, 2], vec![10.238988506030267], vec![]),
        fz(1, 2, &[3, 0], vec![4147.990303747157], vec![]),
        fz(1, 3, &[4, 0], vec![1080.9277279137743], vec![]),
        fz(30, 4, &[3, 2], vec![3.2578847823938304, 1.5013889507842921, 0.012701189387349803, 0.0, 0.0, 0.0], vec![]),
        fz(1, 5, &[2, 0], vec![177.34950648254926], vec![]),
        fz(1, 6, &[0, 4], vec![835.2238021718816], vec![]),
        fz(1, 7, &[3, 4], vec![16.917609314085325], vec![]),
        fz(1, 8, &[3, 1], vec![1442.958403632793], vec![]),
        fz(50, 9, &[1, 0, 0, 2, 4, 0, 0, 0, 0, 0, 0, 0, 0, 4], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 4.678551333515387e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 10, &[4, 2], vec![253.00725842672114], vec![]),
        fz(1, 11, &[0, 3], vec![232.7966819363799], vec![]),
        fz(1, 12, &[0, 1], vec![62695.06832422875], vec![]),
        fz(4, 13, &[1, 4], vec![0.0], vec![]),
        fz(1, 14, &[4, 0], vec![110.03868315949899], vec![]),
        fz(1, 15, &[3, 2], vec![318.1845945203245], vec![]),
        fz(90, 5, &[], vec![5.0, 0.0025215526595527963, 177.34950648254926], vec![]),
        fz(90, 6, &[], vec![6.0, 0.005376102258413996, 835.2238021718816], vec![]),
        fz(90, 7, &[], vec![7.0, 0.05482817867166562, 16.917609314085325], vec![]),
        fz(90, 8, &[], vec![8.0, 0.16905584605281831, 1442.958403632793], vec![]),
        fz(90, 10, &[], vec![10.0, 0.018138173683196954, 253.00725842672114], vec![]),
        fz(90, 11, &[], vec![11.0, 0.016138968002100827, 232.7966819363799], vec![]),
        fz(90, 12, &[], vec![12.0, 0.0001867517222310781, 62695.06832422875], vec![]),
        fz(90, 14, &[], vec![14.0, 0.2748100324643442, 110.03868315949899], vec![]),
        fz(90, 15, &[], vec![15.0, 0.032473750757841695, 318.1845945203245], vec![]),
        ]);
        p.digital_slot_count = 32;
        p
    };
    let (v471, v673) = (4.751886091509621, 4.714141438666053);
    for (mode, exact_mode) in [("default", false), ("exact", true)] {
        let pick = |p: Program| if exact_mode { exact(p) } else { p };
        let mut r471 = Rig::load(&pick(b471())).unwrap();
        let mut r673 = Rig::load(&pick(b673())).unwrap();
        // To 0.1 s: on 673 the 253 ohm from node 4 to the 10.2 V source (6.8
        // V across, 182 mW on 18 mW) burns at about 0.11 s and moves v3.
        for k in 0..=5 {
            let t = k as f64 * 0.02;
            if k > 0 {
                r471.advance(t).unwrap();
                r673.advance(t).unwrap();
            }
            let at = format!("{mode} at {t:.2} s");
            close(r471.v(1), v471, 1e-12, &format!("{at}, 471: CV at the setpoint"));
            assert_eq!(exact_regime(&r471), Regime::Cv, "{at}, 471");
            assert!(!r471.s().power_hold(7).is_free(), "{at}, 471: the chip held off");
            close(r471.i(0), 0.0, 1e-9, &format!("{at}, 471: nothing delivered"));
            close(r673.v(1), v673, 1e-12, &format!("{at}, 673: CV at the setpoint"));
            assert_eq!(exact_regime(&r673), Regime::Cv, "{at}, 673");
            assert!(!r673.s().power_hold(9).is_free(), "{at}, 673: the chip held off");
            let want = (v673 - r673.v(3)) / 1442.958403632793 + v673 / 62695.06832422875;
            rel(-r673.i(0), want, 1e-6, &format!("{at}, 673: the PSU's current, the chip off"));
            close(-r673.i(0), 0.00587, 0.0001, &format!("{at}, 673: 5.87 mA"));
        }
    }
}

#[test]
fn a_landing_that_each_clock_rise_recreates_declines_by_the_episode_guard() {
    // Ruling R76 (b), cold 360 tpl 3, on the R53 list of known declines. A
    // {13.57 V, 90 mA} PSU; a 74LS00 with vcc on the rail and its ground pin
    // on node 4, which 0.52 uF couples to a -2.7 / 7.2 V, 141.5 Hz clock's
    // node. At each clock rise the passes settle CV at 138.6 mA, past the
    // limit, with the gate's floor edge (its ground plus 4.275 V, 12.71 V)
    // under the setpoint; solved with the rail just above that edge the
    // gate still draws 140 mA (the capacitor holds its ground pin at the
    // instant), so the R74 landing holds it off. It is free again at the next
    // host step, once the capacitor has lifted its ground pin past its 5.775
    // V ceiling, and held again at the next rise: each hold shorter than a
    // host step, the episode guard (rulings R51, R54 (a)) declines at the
    // 51st, at 0.3604 s. cf1822c ran it to the end, CV at 138.6 mA past the
    // limit at each rise; no edge-recreation exception is made (R76).
    let mut p = program(7, vec![
        fz(21, 0, &[1, 0], vec![13.56831830455205, 0.0899977823668638], vec![]),
        fz(22, 1, &[3, 0], vec![141.5065553275765, 0.7890347499709739, 7.155483571452099, -2.651931692289305, 0.0], vec![8.0]),
        fz(1, 2, &[2, 0], vec![733231.4588983348], vec![]),
        fz(1, 3, &[4, 0], vec![3239.42258066058], vec![]),
        fz(1, 4, &[5, 0], vec![253108.83426089876], vec![]),
        fz(1, 5, &[6, 0], vec![3531.880989138582], vec![]),
        fz(3, 6, &[0, 5], vec![4.3765694311503106e-5, 0.0, 0.0], vec![]),
        fz(3, 7, &[4, 2], vec![5.182119295011528e-7, 0.0, 0.0], vec![]),
        fz(4, 8, &[6, 3], vec![1.0], vec![]),
        fz(1, 9, &[3, 0], vec![58.6667009582244], vec![]),
        fz(1, 10, &[5, 4], vec![125.42986408656836], vec![]),
        fz(1, 11, &[6, 0], vec![1.6834310474624419], vec![]),
        fz(1, 12, &[3, 1], vec![3967.9963460410227], vec![]),
        fz(1, 13, &[2, 6], vec![19.054717277859613], vec![]),
        fz(50, 14, &[1, 4, 5, 5, 6, 0, 0, 6, 0, 0, 6, 0, 0, 6], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 8.205458880913924e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 15, &[6, 3], vec![273.9179801394844], vec![]),
        fz(1, 16, &[6, 4], vec![284.5026685144939], vec![]),
        fz(1, 17, &[4, 5], vec![525.9392291225133], vec![]),
        fz(3, 18, &[6, 5], vec![1.514196511782389e-6, 0.0, 0.0], vec![]),
        fz(4, 19, &[5, 0], vec![1.0], vec![]),
        fz(1, 20, &[0, 0], vec![82.90049821089357], vec![]),
        fz(1, 21, &[2, 3], vec![732.1053013942415], vec![]),
        fz(90, 9, &[], vec![9.0, 1.6619880934246645, 58.6667009582244], vec![]),
        fz(90, 10, &[], vec![10.0, 0.40917973189663, 125.42986408656836], vec![]),
        fz(90, 11, &[], vec![11.0, 13.9973619476481, 1.6834310474624419], vec![]),
        fz(90, 12, &[], vec![12.0, 0.00021690767417563642, 3967.9963460410227], vec![]),
        fz(90, 13, &[], vec![13.0, 0.06680377745880885, 19.054717277859613], vec![]),
        fz(90, 15, &[], vec![15.0, 0.002488637972988142, 273.9179801394844], vec![]),
        fz(90, 16, &[], vec![16.0, 0.07364110329693237, 284.5026685144939], vec![]),
        fz(90, 17, &[], vec![17.0, 0.0012481278815312088, 525.9392291225133], vec![]),
        fz(90, 20, &[], vec![20.0, 0.9771690809602595, 82.90049821089357], vec![]),
    ]);
    p.digital_slot_count = 32;
    p.options.integration = 1;
    let mut rig = Rig::load(&p).unwrap();
    let mut declined = None;
    for k in 1..=250 {
        if let Err(e) = rig.advance(k as f64 * 2e-3) {
            declined = Some((format!("{e:?}"), rig.k.time()));
            break;
        }
    }
    let (e, t) = declined.expect("the episode guard declines");
    assert!(e.contains("power hold") && e.contains("device 14") && e.contains("51 times"), "{e}");
    assert!((0.358..0.3605).contains(&t), "declined after the instant at {t} s");
}

/// tpl=4 hub=2 e=7.316 psu ilim=0.023 PULSE3 -2.9..9.4 tr=4e-4 R3-1=34379.3 LED5-1+R1-5=18 C4-0=1.7e-6 R2-5=26737.0 R0-1=258.9 LED2-1+R1-2=977 R4-5=21084.2 F3-5=0.34/0.72 P5-2
fn cold_798_tpl4() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![7.316343526897107, 0.022696405804929983], vec![]),
        fz(24, 1, &[3, 0], vec![-2.910049949110422, 9.428119422513031, 0.030825375737027517, 0.000446139644632424, 0.000446139644632424, 0.030150160958612095, 0.06963023939913855], vec![8.0]),
        fz(1, 2, &[2, 0], vec![1126.18351137929], vec![]),
        fz(1, 3, &[4, 0], vec![27486.992052156547], vec![]),
        fz(1, 4, &[5, 0], vec![659862.5684880756], vec![]),
        fz(1, 5, &[3, 1], vec![34379.290033810015], vec![]),
        fz(30, 6, &[5, 1], vec![2.7518611162330906, 1.5221408959345513, 0.011804779912483042, 0.0, 0.0, 0.0], vec![]),
        fz(1, 7, &[1, 5], vec![18.052246303017483], vec![]),
        fz(3, 8, &[4, 0], vec![1.7310551548989753e-6, 0.061523764202301344, 0.0], vec![]),
        fz(1, 9, &[2, 5], vec![26737.00832477482], vec![]),
        fz(1, 10, &[0, 1], vec![258.87184502461054], vec![]),
        fz(30, 11, &[2, 1], vec![2.0649196166332278, 1.7482912179700998, 0.016040085068305156, 0.0, 0.0, 0.0], vec![]),
        fz(1, 12, &[1, 2], vec![977.3516586724628], vec![]),
        fz(1, 13, &[4, 5], vec![21084.23204229505], vec![]),
        fz(10, 14, &[3, 5], vec![0.3363374532195785, 0.7248383940123352], vec![]),
        fz(13, 15, &[5, 2], vec![4.8173628388346135, 0.01759179327952475], vec![]),
        fz(90, 5, &[], vec![5.0, 9.3553901509546e-6, 34379.290033810015], vec![]),
        fz(90, 7, &[], vec![7.0, 0.07110845907259333, 18.052246303017483], vec![]),
        fz(90, 9, &[], vec![9.0, 0.0024668602371752378, 26737.00832477482], vec![]),
        fz(90, 10, &[], vec![10.0, 0.03521388231474889, 258.87184502461054], vec![]),
        fz(90, 12, &[], vec![12.0, 0.015523798766164896, 977.3516586724628], vec![]),
        fz(90, 13, &[], vec![13.0, 0.00021296390160255787, 21084.23204229505], vec![]),
    ];
    let mut p = program(6, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_stepped_fuse_from_a_ramping_pulse_reads_the_ramp_where_its_step_ended() {
    // R71 (b)'s trail, cold 798 tpl 4: fuse F3-5 (0.336 ohm, 0.725 A) from a
    // pulse's node, which ramps from 9.4 V to -2.9 V over 0.4 ms, into a
    // stepped node 5. The trail read the pulse's node where the last publish
    // left it, 7.6 V from where the step ended, and blew the fuse at the
    // ramp's end on 34 mA. It carries at most 0.13 A (the fine reference
    // never blows it); R12 latches at 0.6140 s.
    let stops = [0.006272880227543187, 0.0065910248589287155, 0.014867916614019297, 0.02247516916867357, 0.02477913165122169, 0.06954196850921748, 0.07111182541939857, 0.10247305019576619, 0.10280356819085222, 0.10738325191977782, 0.1077867901845644, 0.1221920717506905, 0.12236204363621518, 0.12260746380415856, 0.12334174424164744, 0.1273048136490475, 0.13212906277969547, 0.14581643851602924, 0.14597254109246852, 0.1833211549829557, 0.18355077234677847, 0.183670941709475, 0.21897758796911235, 0.21968120316069029, 0.22780037862693625, 0.2303290091889369, 0.25237995312471795, 0.2530020183132669, 0.2544114124005052, 0.2552386079349576, 0.2557602912338444, 0.2561546390532273, 0.2595876832572605, 0.2597924154606735, 0.26293016115752604, 0.263382939612516, 0.28364187871818897, 0.29503525537631187, 0.3076887148005353, 0.3083140399727918, 0.3180467362570134, 0.3379927294836021, 0.3386843652176448, 0.341978542142703, 0.34501322866469314, 0.3454261116054774, 0.3459144100958994, 0.34608727590496435, 0.3462155919078771, 0.34799208337265547, 0.3481198766088865, 0.38745497806425394, 0.3896072068661334, 0.4086229470367869, 0.4089817918234879, 0.41728150928509256, 0.44348291361006736, 0.4438642963611351, 0.45];
    let mut e = Engine::load(&cold_798_tpl4()).unwrap();
    for &t in stops.iter().filter(|&&t| t <= 0.45) {
        e.advance(t).unwrap();
    }
    let rec: Vec<(f64, f64)> = e.records(0).unwrap().chunks(7).map(|c| (c[0], c[3])).collect();
    assert!(!rec.iter().any(|r| r.0 == 14.0), "the fuse blew: {rec:?}");
}

/// tpl=4 hub=2 e=4.916 psu ilim=0.040 PULSE3 -1.6..10.6 tr=0e0 C4-3=2.4e-4 C3-0=6.6e-5 LED4-2+R1-4=13 LED1-2+R1-1=52 C4-0=1.4e-2 LED2-3+R1-2=1311 F4-0=0.06/0.01 R2-3=3202.8 R3-2=17.0 R2-3=7.7
fn cold_705_tpl4() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.916107385635344, 0.03962118111120531], vec![]),
        fz(24, 1, &[3, 0], vec![-1.6416609897440881, 10.617920975897578, 0.038872893824295845, 0.0, 0.0, 0.04833680298614762, 0.32884525852496216], vec![8.0]),
        fz(1, 2, &[2, 0], vec![153527.51309561517], vec![]),
        fz(1, 3, &[4, 0], vec![411493.39679297584], vec![]),
        fz(3, 4, &[4, 3], vec![0.00023528173670769268, 0.01819986966610647, 0.0], vec![]),
        fz(3, 5, &[3, 0], vec![6.56059467238488e-5, 0.4946704301998454, 0.0], vec![]),
        fz(30, 6, &[4, 2], vec![2.485669089197824, 2.197006589086898, 0.03820580298438778, 0.0, 0.0, 0.0], vec![]),
        fz(1, 7, &[1, 4], vec![13.491734424644703], vec![]),
        fz(30, 8, &[1, 2], vec![2.2235299076190413, 1.6254620564214588, 0.028778600628238376, 0.0, 0.0, 0.0], vec![]),
        fz(1, 9, &[1, 1], vec![52.27725941661463], vec![]),
        fz(3, 10, &[4, 0], vec![0.013870561125491902, 0.04293419706513406, 0.0], vec![]),
        fz(30, 11, &[2, 3], vec![2.541572977259681, 2.1394498441332317, 0.013471906836003342, 0.0, 0.0, 0.0], vec![]),
        fz(1, 12, &[1, 2], vec![1310.87157515496], vec![]),
        fz(10, 13, &[4, 0], vec![0.06048395171629919, 0.014396870408935359], vec![]),
        fz(1, 14, &[2, 3], vec![3202.829326028936], vec![]),
        fz(1, 15, &[3, 2], vec![16.99814947709048], vec![]),
        fz(1, 16, &[2, 3], vec![7.697683972110706], vec![]),
        fz(90, 7, &[], vec![7.0, 0.023536446923698745, 13.491734424644703], vec![]),
        fz(90, 9, &[], vec![9.0, 1.1746562013511053, 52.27725941661463], vec![]),
        fz(90, 12, &[], vec![12.0, 0.025852242312701192, 1310.87157515496], vec![]),
        fz(90, 14, &[], vec![14.0, 0.016595722568064138, 3202.829326028936], vec![]),
        fz(90, 15, &[], vec![15.0, 7.865202634479338, 16.99814947709048], vec![]),
        fz(90, 16, &[], vec![16.0, 4.034628859101567, 7.697683972110706], vec![]),
    ];
    let mut p = program(6, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_stepped_device_on_the_rail_reads_the_rail_its_step_ended_on() {
    // R71 (b)'s trail, cold 705 tpl 4: the {4.92 V, 40 mA} PSU's CC rail
    // feeds R1-4 (13.5 ohm, 23.5 mW) and LED R8 (rail to node 2, ifMax 28.8
    // mA). Neither can carry more than the PSU's 40 mA: R1-4 at most 21.6 mW,
    // under its rating, and R8 at most 0.93/s, 26 s to its latch. The trail
    // read the rail as published after the pulse's rising edge, against
    // node 2 where its step ended before it (6.5 V), and latched R8 at
    // 37 ms; at acc-int the idle charge burnt R1-4 at 38.9 ms in both modes.
    // Neither latches; the fuse F4-0 does at load.
    let stops = [0.006272881200294258, 0.029185926317624438, 0.05090243353528483, 0.052432113074980094, 0.08641903179429013, 0.08676530483479943, 0.08845295232412471, 0.0966232194545737, 0.09676842900230036, 0.09764008595168698, 0.10200518723718602, 0.11187738231765108, 0.1250742335256922, 0.13596823500616897, 0.13612843922774423, 0.13718821982788126, 0.14428401983642283, 0.14779926983463293, 0.14806166704325105, 0.16117471937143518, 0.16441480437867267, 0.20747018675647133, 0.24524729216692318, 0.2497592019341911, 0.2506962616770701, 0.2827002676400363, 0.2850710912527529, 0.3031130939916753, 0.30434149265748023, 0.30473876147337786, 0.30484544036110944, 0.30702501231104895, 0.3077118642680218, 0.319701751635851, 0.320389339252393, 0.32143809234294113, 0.3399029943415494, 0.3400522633522544, 0.3496253789453974, 0.34990166714691906, 0.35088075272360636, 0.35537435103266524, 0.35783817213369123, 0.35810593131497903, 0.35950832694542834, 0.3599517891281955, 0.3601492199313351, 0.40380979432386105, 0.4041095830145945, 0.4248252630142262, 0.43516419842477766, 0.437757521867536, 0.47332291881049976, 0.4941195143223547, 0.4951099097819948, 0.4987837702401886, 0.49982306667163023, 0.5117738086529908, 0.5304884589995156, 0.5337168345779831, 0.5374129762370196, 0.5814377696080736, 0.5853161842820207, 0.5855930078962507, 0.6123469540166411, 0.6180670711233387, 0.6562068288179699, 0.6771571832801375, 0.6774591407943499, 0.6784058978990182, 0.6786746550919907, 0.6870703825433593, 0.6874254892306085, 0.6901181636343875, 0.6952404247038165, 0.6972513788504672, 0.6993011769091048, 0.7292268436948669, 0.7771728405548967, 0.7773304652492941, 0.7793462674055572, 0.7795604505748617, 0.7803101956359613, 0.7830346938701106, 0.7946227648230703, 0.7950622337371467, 0.8];
    let mut e = Engine::load(&cold_705_tpl4()).unwrap();
    for &t in &stops {
        e.advance(t).unwrap();
    }
    let rec: Vec<(f64, f64)> = e.records(0).unwrap().chunks(7).map(|c| (c[0], c[3])).collect();
    assert!(rec.iter().any(|r| r.0 == 13.0), "the fuse latched: {rec:?}");
    assert!(!rec.iter().any(|r| r.0 == 7.0 || r.0 == 8.0), "R1-4 or R8 latched: {rec:?}");
}

/// tpl=2 hub=2 e=5.234 psu ilim=0.275 LED1-2+R1-1=1355 C5-0=2.3e-3 LED1-5+R1-1=71
fn cold_535_tpl2() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![5.234432841933891, 0.27456469733480177], vec![]),
        fz(1, 1, &[2, 0], vec![141011.80857252545], vec![]),
        fz(1, 2, &[3, 0], vec![31707.640364078485], vec![]),
        fz(1, 3, &[4, 0], vec![1013.0749703674163], vec![]),
        fz(1, 4, &[5, 0], vec![602620.7626514741], vec![]),
        fz(30, 5, &[1, 2], vec![1.9216721394959964, 2.4560626778575756, 0.0340512703373638, 0.0, 0.0, 0.0], vec![]),
        fz(1, 6, &[1, 1], vec![1355.213844442966], vec![]),
        fz(3, 7, &[5, 0], vec![0.0023060961643505047, 0.023372750461450608, 0.0], vec![]),
        fz(30, 8, &[1, 5], vec![3.370604628158895, 2.4818603748546586, 0.006854894315031227, 0.0, 0.0, 0.0], vec![]),
        fz(1, 9, &[1, 1], vec![70.5361128726511], vec![]),
        fz(90, 6, &[], vec![6.0, 0.0013142563652867277, 1355.213844442966], vec![]),
        fz(90, 9, &[], vec![9.0, 0.02293015100366537, 70.5361128726511], vec![]),
    ];
    let mut p = program(6, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_stepped_latch_predicted_from_a_falling_rate_waits_for_the_steps_it_falls_in() {
    // Ruling R71 (b), cold 535 tpl 2: LED R8 (vf 3.37 V, n 2.48, ifMax 6.85
    // mA, no series resistance) charges 2.31 mF from a {5.23 V, 0.275 A} PSU.
    // By hand it carries the 0.2746 A limit (1603/s) until the capacitor
    // reaches 5.234 - 3.605 = 1.629 V at 13.7 ms, S = 21.96; in CV its current
    // falls an e-fold per 64 mV of charge, adding C vtn I0 / ifMax^2 = 0.86:
    // 22.8, under the latch's 24. Its latch, scheduled from the CC rate held
    // forward, fired at 14.97 ms inside a step whose own drive had fallen.
    let stops = [0.00627288078824777, 0.016829457476524014, 0.058843896577463285, 0.058976488413401716, 0.059103567309001626, 0.05970527509700228, 0.07520090991509062, 0.07780697361405664, 0.08073770949696749, 0.08757943996501401, 0.09266788130346024, 0.09857249569377252, 0.1];
    let mut rig = Rig::load(&cold_535_tpl2()).unwrap();
    let mut peak: f64 = 0.0;
    for &t in &stops {
        rig.advance(t).unwrap();
        peak = peak.max(rig.s().slow.stress(8).map_or(0.0, |s| s.value_at(t)));
    }
    assert!(!rig.s().slow.is_failed(8), "the LED latched (S peaked at {peak})");
    assert!((peak - 22.8).abs() <= 0.05 * 22.8, "S peaks at {peak}, by hand 22.8");
}

fn cold_757_tpl0() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![12.55536651140569, 0.07175978736463227], vec![]),
        fz(1, 1, &[2, 0], vec![193087.79499371018], vec![]),
        fz(1, 2, &[3, 0], vec![18521.038002834935], vec![]),
        fz(1, 3, &[4, 0], vec![116743.52822351363], vec![]),
        fz(1, 4, &[3, 4], vec![9.5030121689896], vec![]),
        fz(30, 5, &[1, 2], vec![2.6589948197773783, 1.6162173702828613, 0.035422923785642726, 0.0, 0.0, 0.0], vec![]),
        fz(1, 6, &[1, 1], vec![88.70770747871579], vec![]),
        fz(1, 7, &[3, 2], vec![139.84254079217789], vec![]),
        fz(10, 8, &[1, 2], vec![0.6363705875911793, 0.520692898999944], vec![]),
        fz(3, 9, &[1, 0], vec![0.0011258032507684215, 3.48657095217715, 0.0], vec![]),
        fz(40, 10, &[1, 0, 1, 2, 0, 1, 0, 4], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 11, &[1, 4], vec![38191.54118418172], vec![]),
        fz(1, 12, &[4, 0], vec![50399.407580564475], vec![]),
        fz(3, 13, &[0, 0], vec![8.395189886874198e-8, 0.0, 0.0], vec![]),
        fz(1, 14, &[2, 0], vec![776.4179254184078], vec![]),
        fz(1, 15, &[0, 2], vec![416.3016266812649], vec![]),
        fz(3, 16, &[0, 0], vec![4.8537797738921665e-6, 0.0, 0.0], vec![]),
        fz(13, 17, &[4, 0], vec![0.05495852872765491, 0.037060356438221025], vec![]),
        fz(30, 18, &[2, 0], vec![2.6282409780138005, 2.0950830697546556, 0.017529062722669603, 0.0, 0.0, 0.0], vec![]),
        fz(1, 19, &[1, 2], vec![18.49418242129856], vec![]),
        fz(3, 20, &[2, 0], vec![0.004944123567185785, 0.0, 0.0], vec![]),
        fz(1, 21, &[2, 0], vec![35684.191910148096], vec![]),
        fz(90, 4, &[], vec![4.0, 0.03317655608161077, 9.5030121689896], vec![]),
        fz(90, 6, &[], vec![6.0, 0.006333577366600456, 88.70770747871579], vec![]),
        fz(90, 7, &[], vec![7.0, 0.0033964198577820584, 139.84254079217789], vec![]),
        fz(90, 11, &[], vec![11.0, 0.0012476403385135381, 38191.54118418172], vec![]),
        fz(90, 12, &[], vec![12.0, 7.176592215921902e-5, 50399.407580564475], vec![]),
        fz(90, 14, &[], vec![14.0, 0.13346510582182602, 776.4179254184078], vec![]),
        fz(90, 19, &[], vec![19.0, 0.34002880010579595, 18.49418242129856], vec![]),
    ];
    let mut p = program(5, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_rail_ic_chased_across_its_own_partitions_restart_steps_is_held() {
    // Ruling R82 (B'), cold 757 tpl 0: a 555 held in reset (RESET at 0) on
    // a {12.56 V, 71.76 mA} PSU's rail with 1.13 mF (3.49 ohm) on it; its OUT
    // pin, LOW, sinks 82.8 mA from node 2 (0.64 ohm fuse and 4.94 mF to the
    // rail's side), past the limit. By hand, powered, the rail settles at
    // about 1.41 V, under the chip's 1.8 V floor, and off, node 2's loads
    // take about 6.6 mA, so the rail climbs past it: no power state holds,
    // and the rail slides at 1.8 V with the chip powered x = (71.76 - 6.6) /
    // 82.8 = 0.79 of the time (R53's class). Exact mode declines it at
    // 0.1723 s through ruling R54 (a)'s guard. In default mode the rail
    // capacitor's CC step carried the rail across the edge, and after each
    // flip the partition restarted at H_MIN, so about 13 steps that flipped
    // nothing lay between flips: each ended the chase count, no hold ran, and
    // the chip chattered at ~170 us for as long as it ran (6,805 steps in a
    // 45 ms host step).
    let stops = [0.006272881300476146, 0.011116023500639817, 0.012362155978723055, 0.037731840680085206, 0.03788763379837714, 0.053120364878471986, 0.05323040595778528, 0.07025570911480136, 0.07068761116523894, 0.0708919775775236, 0.08304046345640247, 0.08449035536229421, 0.08933790419438906, 0.0896527633248559, 0.08980863754495291, 0.11830896264699073, 0.12138748064854432, 0.12754703876223325, 0.127694654382418, 0.12783188484706418, 0.15140082694744983, 0.15514026862023755, 0.16204267260765454, 0.16419524959068535, 0.17745987266455407, 0.18930446044733726, 0.19010673195265432, 0.1978444980497105, 0.19863167834988738, 0.19900938133228965, 0.2];
    let mut rig = Rig::load(&cold_757_tpl0()).unwrap();
    for &t in &stops {
        if let Err(err) = rig.advance(t) {
            let at = rig.k.time();
            let msg = format!("{err:?}");
            assert!(msg.contains("held 51 times in one sliding episode"), "{msg}");
            assert!((at - 0.1723).abs() <= 0.01, "declined at {at} s, exact mode at 0.1723 s");
            assert!((rig.u() - 1.8).abs() <= 1e-4 + 1e-3 * 1.8, "the rail at {} V, by hand on the 555's 1.8 V floor", rig.u());
            return;
        }
    }
    panic!("the 555 chattered at its power edge to {} s with {} holds", rig.k.time(), rig.s().hold_stats()[0]);
}

fn cold_108_tpl5() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![11.3202882887201, 0.03459077386322441], vec![]),
        fz(1, 1, &[2, 0], vec![268661.33594287507], vec![]),
        fz(1, 2, &[3, 0], vec![13401.875266756893], vec![]),
        fz(1, 3, &[4, 0], vec![42379.72563549372], vec![]),
        fz(1, 4, &[5, 0], vec![698676.4113034616], vec![]),
        fz(1, 5, &[6, 0], vec![135156.75598749088], vec![]),
        fz(3, 6, &[1, 3], vec![7.714598469201561e-7, 0.0, 0.0], vec![]),
        fz(1, 7, &[4, 0], vec![33766.93589489301], vec![]),
        fz(3, 8, &[3, 5], vec![6.592198340546241e-6, 0.0, 0.0], vec![]),
        fz(50, 9, &[1, 0, 3, 3, 3, 0, 0, 1, 0, 0, 1, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.7951945369129708e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 10, &[3, 1], vec![404.4181104053365], vec![]),
        fz(1, 11, &[1, 5], vec![48.78236835368356], vec![]),
        fz(3, 12, &[1, 4], vec![0.010129635396223535, 0.19263911685269025, 0.0], vec![]),
        fz(30, 13, &[0, 5], vec![2.842892286888211, 1.528311681120797, 0.039684320515477906, 0.0, 0.0, 0.0], vec![]),
        fz(1, 14, &[0, 0], vec![35.426427739042936], vec![]),
        fz(3, 15, &[5, 2], vec![3.601981866910648e-5, 0.06428047465974664, 0.0], vec![]),
        fz(3, 16, &[6, 0], vec![1.9308361169247496e-5, 0.01461543153104114, 0.0], vec![]),
        fz(1, 17, &[6, 5], vec![118.61938041262384], vec![]),
        fz(90, 7, &[], vec![7.0, 4.571267934805499e-5, 33766.93589489301], vec![]),
        fz(90, 10, &[], vec![10.0, 0.0024178672389142362, 404.4181104053365], vec![]),
        fz(90, 11, &[], vec![11.0, 4.35356122116446, 48.78236835368356], vec![]),
        fz(90, 14, &[], vec![14.0, 0.01584240439083085, 35.426427739042936], vec![]),
    ];
    let mut p = program(7, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_rail_ic_flipping_off_a_rail_capacitor_is_not_counted_across_its_steps() {
    // Ruling R82 (B')'s pin, cold 108 tpl 5: a 74LS00 on a {11.32 V, 34.6
    // mA} PSU's rail with no rail capacitor, the rail climbing through the
    // chip's 4.275 V floor on its way to CV at 11.32 V. From 0.90 ms the
    // chip's flips at its floor throw the rail to 7.1 V and back, with up to
    // 15 steps that flip nothing between them; counting through those steps
    // off a rail capacitor's segment declined this board at 0.9 ms. Exact
    // mode runs it, the rail in CV at 11.3203 V at 10 ms.
    let mut rig = Rig::load(&cold_108_tpl5()).unwrap();
    for t in [0.006272879839431231, 0.006418348458282123, 0.00949764155495048, 0.009882438911885259, 0.01] {
        rig.advance(t).unwrap_or_else(|e| panic!("{e:?} at {:.9} s", rig.k.time()));
    }
    assert!((rig.u() - 11.3203).abs() <= 1e-4 + 1e-3 * 11.3203, "the rail at {} V, exact mode 11.3203 V", rig.u());
}


fn cold_3254_tpl0() -> (Program, Vec<(f64, usize)>, Vec<f64>) {
    let mut p = program(6, vec![
    fz(21, 0, &[1, 0], vec![11.246220343398445, 0.4551772607240805], vec![]),
    fz(1, 1, &[2, 0], vec![284150.1753338189], vec![]),
    fz(1, 2, &[3, 0], vec![9897.83225390066], vec![]),
    fz(1, 3, &[4, 0], vec![481665.1512480828], vec![]),
    fz(1, 4, &[5, 0], vec![2717.5253242449144], vec![]),
    fz(40, 5, &[1, 0, 3, 3, 1, 2, 4, 4], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    fz(1, 6, &[1, 4], vec![3945.3370616899874], vec![]),
    fz(1, 7, &[4, 4], vec![137.6327248849708], vec![]),
    fz(3, 8, &[4, 0], vec![1.6330449037676564e-8, 0.0, 0.0], vec![]),
    fz(1, 9, &[3, 1], vec![2130.5603626075554], vec![]),
    fz(3, 10, &[2, 0], vec![0.00027155851581905916, 0.0, 0.0], vec![]),
    fz(1, 11, &[5, 1], vec![572.269355419224], vec![]),
    fz(1, 12, &[1, 0], vec![4525.935851725097], vec![]),
    fz(4, 13, &[5, 2], vec![1.0], vec![]),
    fz(1, 14, &[2, 0], vec![6.584718622861545], vec![]),
    fz(3, 15, &[1, 3], vec![0.0003746264249957443, 0.0, 0.0], vec![]),
    fz(1, 16, &[5, 2], vec![152688.93386831973], vec![]),
    fz(13, 17, &[1, 2], vec![0.28969149883573164, 0.02550757355942012], vec![]),
    fz(1, 18, &[3, 2], vec![5.029719918830595], vec![]),
    fz(3, 19, &[5, 2], vec![6.108838013635083e-5, 2.371434043813692, 0.0], vec![]),
    fz(30, 20, &[1, 4], vec![1.6523220639367115, 2.006618782544971, 0.021317977505823846, 0.0, 0.0, 0.0], vec![]),
    fz(1, 21, &[1, 1], vec![1593.354199626653], vec![]),
    fz(1, 22, &[5, 0], vec![14608.432648436725], vec![]),
    fz(90, 6, &[], vec![6.0, 6.411551618139749e-5, 3945.3370616899874], vec![]),
    fz(90, 7, &[], vec![7.0, 0.0052610079827370496, 137.6327248849708], vec![]),
    fz(90, 9, &[], vec![9.0, 0.0005665751438239242, 2130.5603626075554], vec![]),
    fz(90, 11, &[], vec![11.0, 0.03314459408302718, 572.269355419224], vec![]),
    fz(90, 12, &[], vec![12.0, 0.016386842938351843, 4525.935851725097], vec![]),
    fz(90, 14, &[], vec![14.0, 0.13698282367016573, 6.584718622861545], vec![]),
    fz(90, 16, &[], vec![16.0, 3.4475139601517186e-5, 152688.93386831973], vec![]),
    fz(90, 18, &[], vec![18.0, 0.1219394773547773, 5.029719918830595], vec![]),
    fz(90, 21, &[], vec![21.0, 0.001585964050464631, 1593.354199626653], vec![]),
    fz(90, 22, &[], vec![22.0, 0.000142890460572345, 14608.432648436725], vec![]),
    ]);
    p.digital_slot_count = 32;
    p.options.integration = 0;
    p.options.force_stepped = false;
    let toggles: Vec<(f64, usize)> = vec![(0.268150882995015, 13), (1.1104149809983563, 13)];
    let stops: Vec<f64> = vec![0.006272882304178112, 0.012823525565177877, 0.013541170980079701, 0.02465293041873677, 0.028923865714929033, 0.029302355916161114, 0.029506481508895555, 0.02977509163763609, 0.03031285539192511, 0.04295642939408964, 0.05168230728731764, 0.05183030091569954, 0.05235882669737041, 0.053414008964893414, 0.05844602269927233, 0.06011399341525381, 0.0605189582792592, 0.06071846481777225, 0.06973270929022515, 0.06994614517391336, 0.07749356123398495, 0.07868157029454456, 0.08472512365416775, 0.08489501434168648, 0.09098097952787793, 0.09377157437734732, 0.09732135266939068, 0.0986045023665298, 0.10939262659337588, 0.11072759124514905, 0.11099360085645693, 0.11109474478110061, 0.11119870609988029, 0.117138637026125, 0.12805063200156458, 0.13435096191532053, 0.13663561726611406, 0.13699121918091287, 0.13736211401938317, 0.17008652863207616, 0.1704060704626443, 0.17460186863997348, 0.186894945256729, 0.2057312329348928, 0.2061212261211575, 0.2099029530557235, 0.21046805550100117, 0.21353800305952475, 0.22353416675879317, 0.2279387301540179, 0.22881863487157328, 0.22914692756913582, 0.22953503379111853, 0.24527798692657307, 0.24544253452832024, 0.268150882995015, 0.2685248747888842, 0.30813352715280046, 0.3093924165168472, 0.3439296213173371, 0.3444878289421121, 0.3593507880570135, 0.378301577659042, 0.41295117460447905, 0.41329708278167043, 0.4247648598873834, 0.4252745703298649, 0.4254761831314866, 0.4306902207026152, 0.4318290782300953, 0.4350818052065058, 0.4378164838956515, 0.4380168885985817, 0.44189422835479225, 0.45769384155390314, 0.47815539865372086, 0.47932513008709665, 0.47949711297038167, 0.49845342485011795, 0.5308742747178923, 0.5340462544067739, 0.5455881149362168, 0.5705667049976189, 0.6015782649730197, 0.6105129140705196, 0.6134981660271844, 0.6556345648271322, 0.6585217214483023, 0.6590510633219221, 0.6591524776009698, 0.6611442239808121, 0.6623757107456157, 0.6632077553008157, 0.6638675107228921, 0.6658183782321939, 0.666054481464249, 0.6662081572464302, 0.677325465485507, 0.6775703627621606, 0.6780190218856317, 0.6799692150961106, 0.6801652012358935, 0.6822364560775906, 0.7265546635809104, 0.7336334014039504, 0.7341869473954892, 0.7390672966809088, 0.7393311026761152, 0.7398588432251397, 0.7418117336997042, 0.7419313467027967, 0.7425269140522809, 0.7458687568633516, 0.7459832913523767, 0.7503055796763382, 0.7530287405354558, 0.7960665504026587, 0.8328356579346917, 0.87152403044653, 0.8737599457748357, 0.8775711544744417, 0.8779030833712935, 0.8786240114499474, 0.8787494795292969, 0.9156629958861552, 0.9162985535100091, 0.924110822757011, 0.9403032867730153, 0.9406359605801973, 0.9431441779986227, 0.9519653622528825, 0.9531585944389401, 0.9760587345198813, 0.9762976996019883, 0.9948652744818263, 0.9973752737063667, 0.9977269627470655, 1.032305869580562, 1.0329149774312982, 1.03302278964017, 1.0629470926201148, 1.0635488620993185, 1.0664475892761776, 1.0667107525782313, 1.0670050476072457, 1.0829773128133313, 1.0830924970115465, 1.083410316349095, 1.0882964007396267, 1.101269503788342, 1.1017151788516288, 1.1019148844779887, 1.1104149809983563, 1.1108240424864686, 1.1112792868569974, 1.1124191878432839, 1.1125472993429806, 1.1137293609279948, 1.1278541134053632, 1.1299534302444267, 1.1304397982843621, 1.1453340769271232, 1.1530139937277784, 1.153653199765726, 1.1537768983315253, 1.1938280466161726, 1.1942201352608826, 1.1943752706128126, 1.194585079346648, 1.2];
    (p, toggles, stops)
}

#[test]
fn a_catch_up_whose_crossing_lands_at_its_start_still_reaches_its_horizon() {
    // REFV's hang, cold 3254 tpl 0 in default mode (on 8785c63 and both REF
    // commits). A stepped hub partition caught up to an event on a CC rail
    // reads each watch's threshold from the published rail, which the
    // catch-up does not move, while its steps move the rail's own partition:
    // the 555's comparator node sat past its threshold at each step's start,
    // the crossing bisected to steps of about 1e-16 s, and the catch-up, 85
    // ps from its horizon at 5.73 ms, never got there: the first `advance`
    // never returned. With no catch-up step shorter than the shortest step,
    // that advance returns (in about 3 s of this board's crawl; what follows
    // the crawl is reported, not pinned here).
    // A budget the run can be held to in any build: the first advance takes
    // 79,198 steps and 167,237 events with the floor (stats 10 and 4).
    let (p, toggles, stops) = cold_3254_tpl0();
    let first: Vec<f64> = stops.iter().copied().take(1).collect();
    let e = sweep_run(&p, &toggles, &first).unwrap_or_else(|(e, t)| panic!("{} at {t}: {}", e.code, e.message));
    assert!(e.stat(10) < 2e5 && e.stat(4) < 4e5, "steps {} and events {} within the budget", e.stat(10), e.stat(4));
}


fn ref_b471(extra: Vec<Device>) -> Program {
    let mut p = program(7, vec![
        fz(21, 0, &[1, 0], vec![4.751886091509621, 0.054961332725668104], vec![]),
        fz(2, 1, &[2, 0], vec![-4.596049646086812], vec![]),
        fz(1, 2, &[3, 0], vec![494452.0913392493], vec![]),
        fz(1, 3, &[4, 0], vec![77429.4854565049], vec![]),
        fz(1, 4, &[5, 0], vec![78194.52499104275], vec![]),
        fz(1, 5, &[6, 0], vec![2675.744432386028], vec![]),
        fz(1, 6, &[2, 3], vec![98.66153046803586], vec![]),
        fz(50, 7, &[1, 0, 5, 2, 3, 0, 0, 2, 0, 0, 2, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 3.3711834227987145e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 8, &[3, 2], vec![263.508620325762], vec![]),
        fz(1, 9, &[2, 0], vec![102.06709251544831], vec![]),
        fz(3, 10, &[0, 0], vec![0.008790905891188373, 0.1361737485557175, 0.0], vec![]),
        fz(1, 11, &[3, 5], vec![13563.322422657602], vec![]),
        fz(30, 12, &[3, 6], vec![2.645442738804019, 1.9851176644289632, 0.03894033893409811, 0.0, 0.0, 0.0], vec![]),
        fz(90, 6, &[], vec![6.0, 0.05757475196045802, 98.66153046803586], vec![]),
        fz(90, 8, &[], vec![8.0, 0.019609209561770004, 263.508620325762], vec![]),
        fz(90, 9, &[], vec![9.0, 0.010313525627732776, 102.06709251544831], vec![]),
        fz(90, 11, &[], vec![11.0, 0.00012125106711201618, 13563.322422657602], vec![]),
    ]);
    p.devices.extend(extra);
    p.digital_slot_count = 32;
    p
}

#[test]
fn r74_walks_down_the_edges_and_frees_the_chips_above_where_the_rail_lands() {
    // REFV's probe T1e: cold 471 tpl 2 (its 74LS00, device 7, sourcing into a
    // -4.6 V node: 190 mA at the 4.752 V setpoint of a 55 mA PSU) plus a
    // second 74LS00 (device 17) with every pin on ground and a 5.0 V floor,
    // so a window [4.5, 5.775] V and a draw of nothing. Its edge, 4.5 V, is
    // the highest under the setpoint. By hand, as the rail sags it browns
    // out first (nothing changes: it drew nothing), then device 7 at 4.275 V,
    // under which nothing is drawn: the CC point is device 7's edge. Held
    // off there, the PSU is in CV at the setpoint delivering 0 mA, and device
    // 17, whose edge lies above where the rail landed, is powered again.
    // Exact mode, its trials solving the partitions at each rail, holds
    // device 7 alone. Before the fix default mode declined at load.
    for (mode, exact_mode) in [("default", false), ("exact", true)] {
        let mut b = ls00(17, 1, 0, [0; 12], 1e-8, 8);
        b.params[ic_param::VCC_MIN] = 5.0;
        let p = ref_b471(vec![b]);
        let mut rig = Rig::load(&if exact_mode { exact(p) } else { p }).unwrap_or_else(|e| panic!("{mode}: load {e:?}"));
        for k in 0..=4 {
            if k > 0 {
                rig.advance(k as f64 * 0.01).unwrap_or_else(|e| panic!("{mode}: {e:?}"));
            }
            let at = format!("{mode} at {} ms", 10 * k);
            close(rig.v(1), 4.751886091509621, 1e-12, &format!("{at}: CV at the setpoint"));
            assert_eq!(exact_regime(&rig), Regime::Cv, "{at}");
            close(rig.i(0), 0.0, 1e-9, &format!("{at}: nothing delivered"));
            assert!(!rig.s().power_hold(7).is_free(), "{at}: the culprit held off");
            assert!(rig.s().power_hold(17).is_free(), "{at}: the chip above the landing free");
        }
    }
}


fn cold_1986_tpl0() -> (Program, Vec<(f64, usize)>, Vec<f64>) {
    let mut p = program(7, vec![
    fz(21, 0, &[1, 0], vec![4.8867463521225085, 0.0214728538967793], vec![]),
    fz(2, 1, &[0, 2], vec![3.0566701917890082], vec![]),
    fz(24, 2, &[3, 0], vec![0.991173364806758, 4.5566559890519125, 0.034654796981481044, 0.0, 0.0, 0.03702702192297119, 0.1116594851956823], vec![8.0]),
    fz(1, 3, &[4, 0], vec![1004.2838744196536], vec![]),
    fz(1, 4, &[5, 0], vec![387004.26018526533], vec![]),
    fz(1, 5, &[6, 0], vec![91245.7104637708], vec![]),
    fz(4, 6, &[6, 0], vec![1.0], vec![]),
    fz(1, 7, &[0, 0], vec![2244.1707287391473], vec![]),
    fz(3, 8, &[6, 0], vec![1.0031466396907978e-7, 0.0, 0.0], vec![]),
    fz(3, 9, &[0, 0], vec![6.954943726412405e-7, 0.0, 0.0], vec![]),
    fz(1, 10, &[6, 2], vec![68.15727720730737], vec![]),
    fz(1, 11, &[5, 6], vec![10795.61036779299], vec![]),
    fz(10, 12, &[5, 2], vec![1.4881035750798193, 0.9342083200892071], vec![]),
    fz(1, 13, &[1, 0], vec![63678.204504779744], vec![]),
    fz(50, 14, &[1, 0, 2, 0, 5, 0, 0, 3, 0, 0, 3, 0, 0, 5], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 6.996841739527289e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    fz(1, 15, &[5, 6], vec![1032.1364021540458], vec![]),
    fz(1, 16, &[3, 0], vec![647.2983352820522], vec![]),
    fz(1, 17, &[3, 2], vec![32.2538200048518], vec![]),
    fz(90, 7, &[], vec![7.0, 5.623314350198672e-5, 2244.1707287391473], vec![]),
    fz(90, 10, &[], vec![10.0, 0.027610984268032487, 68.15727720730737], vec![]),
    fz(90, 11, &[], vec![11.0, 4.018460613030943e-5, 10795.61036779299], vec![]),
    fz(90, 13, &[], vec![13.0, 8.892659607433069e-5, 63678.204504779744], vec![]),
    fz(90, 15, &[], vec![15.0, 0.08138492965780203, 1032.1364021540458], vec![]),
    fz(90, 16, &[], vec![16.0, 0.0068285602735905, 647.2983352820522], vec![]),
    fz(90, 17, &[], vec![17.0, 0.01769009722134774, 32.2538200048518], vec![]),
    ]);
    p.digital_slot_count = 32;
    p.options.integration = 1;
    p.options.force_stepped = false;
    let toggles: Vec<(f64, usize)> = vec![(0.12176476287406653, 6), (0.3375230430794596, 6), (0.4975878607207562, 6)];
    let stops: Vec<f64> = vec![0.006272878315969919, 0.039053661031856216, 0.039587929898541416, 0.055582046143667116, 0.06445388981072296, 0.06865869973053526, 0.06956810502544665, 0.0701060057085617, 0.0994402337770437, 0.12176476287406653, 0.12195922950877115, 0.12209275191181494, 0.14848011811607387, 0.18492937178825156, 0.20275816893596504, 0.20395584963445798, 0.216857264075338, 0.2242991902979167, 0.22478833449703703, 0.2449760050617346, 0.2505702921501145, 0.25067812802816963, 0.25081861992729354, 0.27367174976973524, 0.3230763139466005, 0.3236155890652665, 0.32620806026051774, 0.3344374593132042, 0.33462451400363175, 0.3347705095742189, 0.3361626042905809, 0.336610359070558, 0.3375230430794596, 0.33818413096226996, 0.35830891309261914, 0.3587333378292127, 0.3645524869338136, 0.37636323059735216, 0.3891786429964177, 0.40203195756314636, 0.4022241237942444, 0.4023507308385056, 0.41323351428418653, 0.4150847957560882, 0.4177508877570866, 0.44961153005027504, 0.44982942541183035, 0.45134468871044814, 0.46412596167808956, 0.46454892349983296, 0.4648083243630516, 0.46533478158066177, 0.4665853650970894, 0.46691964137451863, 0.47731000443257543, 0.4939094004704731, 0.4975878607207562, 0.5088582774068496, 0.5118264932574365, 0.5127483800824659, 0.5259737186504947, 0.5261731108047144, 0.5270122611348498, 0.5405159814787663, 0.540629447997466, 0.5431152112304453, 0.5464446494686298, 0.5470441508524689, 0.5495338157982573, 0.5814880853682152, 0.5824909916609562, 0.5926802857237751, 0.5928154966778645, 0.594366598237228, 0.5946809292296322, 0.5960268417844788, 0.6276226410772671, 0.6433534814855612, 0.6441726366053819, 0.6456612194448824, 0.6464995518993366, 0.6496088975373155, 0.65000254558652, 0.6506541779384729, 0.6986761058254453, 0.6989281922607982, 0.6995556149835177, 0.7013847043718886, 0.739569448898801, 0.7438984224881721, 0.7455294823478481, 0.7461393552401376, 0.7523865913631522, 0.7527712151501617, 0.7528914297660981, 0.7532016896308679, 0.7584091976799182, 0.7585831744350028, 0.7593875467400196, 0.7663138975004475, 0.8091204778620077, 0.8095604170964402, 0.8184836640532125, 0.8290168408844414, 0.8373603857424282, 0.8445097047315975, 0.8638740434346842, 0.8643590383604568, 0.8746428539883043, 0.8807474824351509, 0.8810284723870406, 0.8813180778341678, 0.9018796043846267, 0.9020870764290633, 0.9112938465512022, 0.9130444892461798, 0.9345081163055889, 0.9347273885999949, 0.9349507519182654, 0.9496240002410199, 0.95019013978608, 0.9676307126026663, 1.0113381701567548, 1.0178486327949237, 1.0187869362838344, 1.0271268601318173, 1.0282797042847853, 1.0327490000688027, 1.0333711855987762, 1.0341178205858845, 1.0386206412433778, 1.0387660317372716, 1.0388676432067605, 1.0395612114864186, 1.073024219368843, 1.0738043213375068, 1.1114405794652145, 1.1138833725373767, 1.14736023602682, 1.1498989164164573, 1.1629051862341473, 1.1773687577333931, 1.177902135813081, 1.178032421081348, 1.1794128551578533, 1.1950462503063604, 1.2];
    (p, toggles, stops)
}

#[test]
fn r74_a_landing_whose_powered_probe_cannot_settle_holds_the_chip() {
    // REFV, cold 1986 tpl 0 in default mode: OK at 98cbdfd, declined `-4` at
    // 68d7896 at a switch toggle (0.4976 s, "the rail loop did not settle").
    // A {4.887 V, 21 mA} PSU, a 74LS00 on the rail with outputs into nodes a
    // 3.06 V source and a pulse hold below its rail. At the toggle R74's
    // landing probed the gate held powered: CV at the setpoint past the
    // limit, CC at 1.73 V under it, the passes alternating the two with no
    // chip flipping. A powered probe that cannot settle is no operating point
    // with the gate powered either: it starves, read under its edge, and is
    // held off, as exact mode holds it (4 holds). By hand, held off the PSU
    // delivers within its limit at every sampled instant.
    let (p, toggles, stops) = cold_1986_tpl0();
    let e = sweep_run(&p, &toggles, &stops).unwrap_or_else(|(e, t)| panic!("{} at {t}: {}", e.code, e.message));
    assert!(e.stat(25) >= 1.0, "the gate held off");
}


#[test]
fn a_retry_frees_an_innocent_chip_held_beside_the_culprit_at_the_same_edge() {
    // REFV's probe T1b: cold 471 tpl 2's culprit (device 7) plus an
    // identical 74LS00 (device 17, window [4.275, 5.775] V, the same edge)
    // whose 1Y, inputs on ground and so HIGH, drives 1 k to ground (node 7).
    // The landing holds both. By hand the innocent one carries itself: with
    // the culprit off the rail is CV at 4.752 V, and its HIGH output into 1 k
    // gives 4.14 V, 4.14 mA, within the 55 mA limit. A retry that probes the
    // held chips together fails on the culprit's overload and kept both off
    // for good; each probed on its own, the innocent one is free at the next
    // host step's retry.
    for (mode, exact_mode) in [("default", false), ("exact", true)] {
        let b = ls00(17, 1, 0, [0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1e-8, 8);
        let mut p = ref_b471(vec![b, resistor(18, 7, 0, 1e3)]);
        p.node_count = 8;
        let mut rig = Rig::load(&if exact_mode { exact(p) } else { p }).unwrap_or_else(|e| panic!("{mode}: load {e:?}"));
        for k in 1..=200 {
            rig.advance(k as f64 * 1e-3).unwrap_or_else(|e| panic!("{mode}: {e:?}"));
        }
        let at = format!("{mode} at 200 ms");
        assert!(!rig.s().power_hold(7).is_free(), "{at}: the culprit held off");
        assert!(rig.s().power_hold(17).is_free(), "{at}: the innocent chip free");
        close(rig.v(7), 4.14, 0.01, &format!("{at}: its HIGH output into 1 k"));
        close(-rig.i(0), rig.v(7) / 1e3, 1e-6, &format!("{at}: the PSU delivers the 1 k's current"));
    }
}


fn cold_559_tpl0() -> (Program, Vec<(f64, usize)>, Vec<f64>) {
    let mut p = program(8, vec![
    fz(21, 0, &[1, 0], vec![6.82148517461291, 0.02744343564693486], vec![]),
    fz(2, 1, &[0, 2], vec![-3.447555511847309], vec![]),
    fz(22, 2, &[3, 0], vec![77.75843409368349, 0.1532909655474362, 5.720372002063975, -1.1232499945415584, 0.0], vec![8.0]),
    fz(1, 3, &[4, 0], vec![3414.076274801499], vec![]),
    fz(1, 4, &[5, 0], vec![4603.464686826747], vec![]),
    fz(1, 5, &[6, 0], vec![3441.7912779681706], vec![]),
    fz(1, 6, &[7, 0], vec![9944.26050359521], vec![]),
    fz(1, 7, &[6, 7], vec![31.717378562436856], vec![]),
    fz(30, 8, &[0, 6], vec![2.8442902304899236, 1.9962176804936107, 0.03242012245843129, 0.0, 0.0, 0.0], vec![]),
    fz(1, 9, &[0, 0], vec![398.3149499231084], vec![]),
    fz(10, 10, &[2, 3], vec![0.631184379027539, 0.08269112855711004], vec![]),
    fz(30, 11, &[1, 4], vec![2.444513044917242, 2.4682050969684397, 0.004145541058086976, 0.0, 0.0, 0.0], vec![]),
    fz(1, 12, &[1, 1], vec![2438.5254439643445], vec![]),
    fz(1, 13, &[5, 1], vec![80.82981534225667], vec![]),
    fz(1, 14, &[2, 0], vec![24938.668883756738], vec![]),
    fz(40, 15, &[1, 0, 5, 7, 6, 0, 7, 1], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    fz(1, 16, &[1, 1], vec![327.28293409638275], vec![]),
    fz(1, 17, &[1, 7], vec![4068.9776798069756], vec![]),
    fz(3, 18, &[7, 0], vec![1.33001945374868e-6, 0.0, 0.0], vec![]),
    fz(1, 19, &[7, 6], vec![208.30013824429537], vec![]),
    fz(3, 20, &[1, 0], vec![0.0003801853076044016, 0.0, 0.0], vec![]),
    fz(90, 7, &[], vec![7.0, 0.003980112945073713, 31.717378562436856], vec![]),
    fz(90, 9, &[], vec![9.0, 0.04230930705734778, 398.3149499231084], vec![]),
    fz(90, 12, &[], vec![12.0, 0.003761036258254046, 2438.5254439643445], vec![]),
    fz(90, 13, &[], vec![13.0, 0.5910499580882643, 80.82981534225667], vec![]),
    fz(90, 14, &[], vec![14.0, 3.2156308096705946e-5, 24938.668883756738], vec![]),
    fz(90, 16, &[], vec![16.0, 0.06306789772228794, 327.28293409638275], vec![]),
    fz(90, 17, &[], vec![17.0, 0.001003993668794642, 4068.9776798069756], vec![]),
    fz(90, 19, &[], vec![19.0, 0.000750537860197, 208.30013824429537], vec![]),
    ]);
    p.digital_slot_count = 32;
    p.options.integration = 0;
    p.options.force_stepped = false;
    let toggles: Vec<(f64, usize)> = vec![];
    let stops: Vec<f64> = vec![0.0062728808434102425, 0.006744357641509285, 0.04604942185360206, 0.04863838805755794, 0.048774397958224545, 0.05428613408979253, 0.08639614081770303, 0.08654201271038474, 0.0867517681796459, 0.09434631801663479, 0.09665844860063455, 0.10118841766255325, 0.10142610745628707, 0.11340656588124004, 0.114338194933015, 0.11890242959325976, 0.1195500358981922, 0.11978576670317097, 0.11998340910254421, 0.13080697737817742, 0.16716075045624024, 0.16793703526378673, 0.17199936646661357, 0.172468765349972, 0.1728911451055386, 0.17323525814487883, 0.18062926447679095, 0.18093226285485842, 0.18108428073984184, 0.20299520457307285, 0.21667364445870116, 0.21681830450434905, 0.21692176054214063, 0.23405094037552032, 0.2491495084091149, 0.2494008872885095, 0.25133807468598807, 0.2573985283563046, 0.2687257829550583, 0.27252442829673723, 0.27401275995995295, 0.2942602929079507, 0.29479195007836284, 0.30358354091006984, 0.3045081418320409, 0.30557387134254815, 0.31049271867199163, 0.3109929949203516, 0.337314860719146, 0.36730623097528925, 0.367492464441942, 0.37211091359914056, 0.3729600029317298, 0.41302174379336837, 0.4203698339591645, 0.42067670067687674, 0.42089112945871315, 0.4246422443828818, 0.44415004345455483, 0.48073421035468944, 0.4808552473774065, 0.5016055888991673, 0.5045689493470747, 0.5201034797067425, 0.5206437719659585, 0.5220510121605763, 0.5223418649765913, 0.5231431851962718, 0.523317405590831, 0.5240020357023335, 0.5504323525353039, 0.5580397176283233, 0.5586902386086067, 0.5595589739563015, 0.5600087251339375, 0.5601335491868368, 0.5646378726135576, 0.5657262411918652, 0.56790569716211, 0.5690543683451675, 0.5698856775017881, 0.5731513418514298, 0.6169606893972126, 0.6173627074999039, 0.6174766499067164, 0.6184183958469026, 0.6193566257853959, 0.6206102617167495, 0.6207352715502354, 0.6209024463100801, 0.6519904795958686, 0.6667776551735678, 0.6669693962451633, 0.6679856615812371, 0.6687565952944522, 0.6697822919346454, 0.6719333041771339, 0.6721619024605012, 0.6786014174891939, 0.6802111870617021, 0.6832322278100943, 0.6887314069787532, 0.6890479664104866, 0.689150848803334, 0.6921494770763945, 0.6926134782364969, 0.6929163231557539, 0.6951113270265822, 0.6957199811315283, 0.6960451482610817, 0.700190072892249, 0.7005262822813643, 0.7292111182711758, 0.7548237736413504, 0.757446046947421, 0.8026339147150103, 0.8029702348701044, 0.8083392595409207, 0.8375594340672226, 0.8377665777158889, 0.8379311813781939, 0.841683270297885, 0.8435728751210765, 0.862668943054271, 0.8637832216416776, 0.8753128800375212, 0.8794478593054902, 0.8804657869682194, 0.88217609292356, 0.8875713474756209, 0.8878135082600507, 0.9368393170220117, 0.937170550745586, 0.937281209429872, 0.9412285169219382, 0.9417536242060938, 0.9419321733477731, 0.9505523839312703, 0.9787079122064506, 0.9845019851291609, 0.9930055910747865, 1.017032130083633, 1.031252153813453, 1.039100905752488, 1.0405386629205147, 1.0411354767727135, 1.0702955402942531, 1.1169016059223018, 1.1170284311100263, 1.1275565358606752, 1.1283652151454828, 1.128482439455916, 1.1433709225562323, 1.1448588192777833, 1.1472555532798812, 1.1475197808208941, 1.1528435314174756, 1.155038155047801, 1.1566679360000833, 1.156933903005455, 1.1575312675477754, 1.1576725441285616, 1.1582280395777336, 1.1802102888193042, 1.1806672923345567, 1.2];
    (p, toggles, stops)
}

fn cold_175_tpl2_sweep() -> (Program, Vec<(f64, usize)>, Vec<f64>) {
    let mut p = program(7, vec![
    fz(21, 0, &[1, 0], vec![5.279251491871787, 0.12486913735539018], vec![]),
    fz(2, 1, &[2, 0], vec![3.2944469335300433], vec![]),
    fz(1, 2, &[3, 0], vec![523113.99131483736], vec![]),
    fz(1, 3, &[4, 0], vec![441731.46301296854], vec![]),
    fz(1, 4, &[5, 0], vec![400887.03122832946], vec![]),
    fz(1, 5, &[6, 0], vec![19551.037108783705], vec![]),
    fz(30, 6, &[6, 2], vec![2.490487895713666, 2.133341615049267, 0.005941553299019336, 0.0, 0.0, 0.0], vec![]),
    fz(1, 7, &[1, 6], vec![768.0287173218338], vec![]),
    fz(40, 8, &[1, 0, 6, 3, 5, 5, 1, 1], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    fz(1, 9, &[1, 1], vec![259.9129858404332], vec![]),
    fz(1, 10, &[1, 1], vec![22798.751989056218], vec![]),
    fz(3, 11, &[1, 0], vec![3.845883733983537e-6, 0.0, 0.0], vec![]),
    fz(1, 12, &[3, 0], vec![2705.033195768211], vec![]),
    fz(1, 13, &[5, 6], vec![16770.726136343015], vec![]),
    fz(1, 14, &[3, 6], vec![22693.85635324758], vec![]),
    fz(1, 15, &[0, 1], vec![80444.03115180125], vec![]),
    fz(1, 16, &[6, 3], vec![1832.7466798794446], vec![]),
    fz(3, 17, &[6, 0], vec![0.00010041305989192043, 0.0, 0.0], vec![]),
    fz(3, 18, &[4, 2], vec![5.709328797531033e-6, 0.0, 0.0], vec![]),
    fz(30, 19, &[0, 1], vec![1.8202604658110737, 1.799465313988287, 0.008247942771208824, 0.0, 0.0, 0.0], vec![]),
    fz(1, 20, &[0, 0], vec![11.194632856996742], vec![]),
    fz(1, 21, &[4, 5], vec![3718.5413539250926], vec![]),
    fz(3, 22, &[4, 0], vec![6.8848737667698e-5, 0.0, 0.0], vec![]),
    fz(1, 23, &[0, 1], vec![671.9597403622046], vec![]),
    fz(90, 7, &[], vec![7.0, 0.0023589339771113043, 768.0287173218338], vec![]),
    fz(90, 9, &[], vec![9.0, 0.20249870632200995, 259.9129858404332], vec![]),
    fz(90, 10, &[], vec![10.0, 9.903850432653147e-5, 22798.751989056218], vec![]),
    fz(90, 12, &[], vec![12.0, 2.3342260829871632e-5, 2705.033195768211], vec![]),
    fz(90, 15, &[], vec![15.0, 0.0002399021656463534, 80444.03115180125], vec![]),
    fz(90, 20, &[], vec![20.0, 0.025490775773001514, 11.194632856996742], vec![]),
    ]);
    p.digital_slot_count = 32;
    p.options.integration = 0;
    p.options.force_stepped = true;
    let toggles: Vec<(f64, usize)> = vec![];
    let stops: Vec<f64> = vec![0.006272879978984825, 0.006737256308170404, 0.038139641656497435, 0.04154750969351386, 0.041981687418143336, 0.042282020737404044, 0.04446548328535588, 0.04504935689956483, 0.05726417845589198, 0.10110338257874799, 0.14096638524514044, 0.1538781103253384, 0.15541585779436878, 0.19379853965400207, 0.19904704398077355, 0.21871570765888051, 0.22040488704267083, 0.23077962201215618, 0.23147669889061756, 0.23682007658125745, 0.24359863432451603, 0.24581044567054333, 0.24605793747451885, 0.26932953218650935, 0.26982857249566305, 0.29165774589882193, 0.323822478710669, 0.3245326395441571, 0.3359707272657522, 0.3395328772132661, 0.38864253968945234, 0.3889310914409459, 0.3894919304334255, 0.39124262951114225, 0.39141701781214466, 0.40143331551698935, 0.41628392037537565, 0.41647245933017124, 0.45280568546069694, 0.45323486667323953, 0.45527047578136115, 0.46254312537984915, 0.5042073814245781, 0.5052973211239591, 0.5536249130446808, 0.5600010651701359, 0.5601060934342678, 0.5620067865442132, 0.5624945716973923, 0.5628349088039665, 0.5662623283733159, 0.5776876980861431, 0.5782974178367883, 0.5858075376675697, 0.6146390778376243, 0.6163169542386913, 0.6166733936188559, 0.616998527942831, 0.617274097698689, 0.6211551453936076, 0.640385855888215, 0.6441264267123612, 0.6565684797438297, 0.6569960945273576, 0.6575463582125846, 0.6576758945837408, 0.6844762175685779, 0.684582582692559, 0.684817685888256, 0.6890160046023218, 0.693237305007297, 0.6955773492910368, 0.6965257941055804, 0.696654378809123, 0.7143571810475812, 0.7216829291135866, 0.7373435559888114, 0.754328481018915, 0.7848754630079678, 0.7852831311355024, 0.786396101193285, 0.8017427412366293, 0.802277048255857, 0.808092112091967, 0.8087158657433096, 0.8474199007621832, 0.8477465687221756, 0.8750504749682919, 0.8751507639377251, 0.8765261726391869, 0.8787537976809756, 0.8911437383909047, 0.8912449818991913, 0.9119844818578474, 0.9121377152095659, 0.912309607357835, 0.9128487774533108, 0.9483353977040864, 0.948530967053169, 0.9502433696033153, 0.9507873101047768, 0.9567400255275331, 0.9594387500826398, 0.9596677520782007, 0.9942583260564374, 1.0053272603519443, 1.0099697492115463, 1.0146894857869517, 1.0148388795848955, 1.016948984812883, 1.0170970624758415, 1.04920397558034, 1.0493790056935663, 1.0523782422610515, 1.0627500084062378, 1.073109372477166, 1.0736604276213615, 1.0870665604165186, 1.0872205960412311, 1.0879135530647468, 1.0880441702052972, 1.0907640331175135, 1.090944961310115, 1.092364821284739, 1.1250166891717657, 1.136415533234253, 1.1534394131003634, 1.197167881792808, 1.1973242684882521, 1.1996982000167258, 1.2];
    (p, toggles, stops)
}

#[test]
fn r74_leaves_a_rail_capacitor_psu_to_r68() {
    // R74's landing skips a PSU with a rail capacitor, where an edge landing
    // is R68/R69's (`cv_past_limit_landing`). Under R74 v2 without that
    // exclusion, cold 559 tpl 0's 555 behind 380 uF slid to the hold-episode
    // guard at 96.7 ms (exact mode holds nothing there) and cold 175 tpl 2
    // declined at its load. Since the edge walk and the probe-failure rule
    // both run either way; this guards them, and the exclusion's only effect
    // left on cold 1-800 is one hold more on 542 tpl 4.
    let (p, toggles, stops) = cold_559_tpl0();
    let run = sweep_run(&p, &toggles, &stops);
    assert!(run.is_ok(), "559/0: {:?}", run.err().map(|(e, t)| (e.message, t)));
    let (p, toggles, stops) = cold_175_tpl2_sweep();
    let early: Vec<f64> = stops.iter().copied().filter(|&t| t < 0.5).collect();
    let run = sweep_run(&p, &toggles, &early);
    assert!(run.is_ok(), "175/2 to 0.5 s: {:?}", run.err().map(|(e, t)| (e.message, t)));
}

/// tpl=5 hub=2 e=4.896 psu ilim=0.035 R2-1=60.6 LED4-0+R1-4=1413 LED2-1+R1-2=95 C4-2=3.1e-5 555 vcc1 gnd3 [3, 2, 0, 0, 4, 2] R4-1=88257.2 span=4.896156382894526
fn cold_7_tpl5() -> (Program, Vec<(f64, usize)>, Vec<f64>) {
    let mut p = program(5, vec![
    fz(21, 0, &[1, 0], vec![4.896156382894526, 0.03481635222318897], vec![]),
    fz(1, 1, &[2, 0], vec![538092.6737587971], vec![]),
    fz(1, 2, &[3, 0], vec![45913.19071839569], vec![]),
    fz(1, 3, &[4, 0], vec![218593.60191678582], vec![]),
    fz(1, 4, &[2, 1], vec![60.55601922368702], vec![]),
    fz(30, 5, &[4, 0], vec![1.8226952901802467, 1.8027771914612698, 0.018335341625788135, 0.0, 0.0, 0.0], vec![]),
    fz(1, 6, &[1, 4], vec![1413.1919505396436], vec![]),
    fz(30, 7, &[2, 1], vec![3.3688753823171913, 2.3978954764807834, 0.03771864871801506, 0.0, 0.0, 0.0], vec![]),
    fz(1, 8, &[1, 2], vec![94.74345945425831], vec![]),
    fz(3, 9, &[4, 2], vec![3.1103677994643145e-5, 0.0, 0.0], vec![]),
    fz(40, 10, &[1, 3, 3, 2, 0, 0, 4, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    fz(1, 11, &[1, 2], vec![2454.067175745275], vec![]),
    fz(1, 12, &[2, 4], vec![2440.5136036827544], vec![]),
    fz(3, 13, &[4, 0], vec![5.232301086340363e-7, 0.0, 0.0], vec![]),
    fz(1, 14, &[2, 2], vec![127.91134428699795], vec![]),
    fz(1, 15, &[0, 3], vec![119.91113041581153], vec![]),
    fz(1, 16, &[4, 1], vec![88257.19412692077], vec![]),
    fz(90, 4, &[], vec![4.0, 0.004768333450176877, 60.55601922368702], vec![]),
    fz(90, 6, &[], vec![6.0, 0.0011178569493674587, 1413.1919505396436], vec![]),
    fz(90, 8, &[], vec![8.0, 0.0012908352218015651, 94.74345945425831], vec![]),
    fz(90, 11, &[], vec![11.0, 0.0028278308783430245, 2454.067175745275], vec![]),
    fz(90, 12, &[], vec![12.0, 0.00011228671844498051, 2440.5136036827544], vec![]),
    fz(90, 14, &[], vec![14.0, 0.0006964935066691465, 127.91134428699795], vec![]),
    fz(90, 16, &[], vec![16.0, 1.9462740271229835e-5, 88257.19412692077], vec![]),
    ]);
    p.digital_slot_count = 32;
    p.options.integration = 0;
    p.options.force_stepped = false;
    p.options.no_rail_shortcut = true;
    let toggles: Vec<(f64, usize)> = vec![];
    let stops: Vec<f64> = vec![0.006272879599356148, 0.016698232994375808, 0.017516190773174106, 0.0210471593856631, 0.023700230091663114, 0.024218131306263185, 0.024882135080202174, 0.03190567475933873, 0.04152108682251448, 0.05457620614236116, 0.05475715862700885, 0.05502787837056468, 0.055240284308500524, 0.05547824214556055, 0.07018945334296087, 0.0753977036576992, 0.12439770398421504, 0.12631407096851402, 0.1361452360811551, 0.14576710069909615, 0.15032218991751076, 0.19816592540611697, 0.19834690583834919, 0.20593281067285235, 0.2083771599560474, 0.21564673292461814, 0.2157682989750026, 0.2179475877035891, 0.24408607702474405, 0.27639781167971944, 0.2766101985117782, 0.28246624787966285, 0.29573011299706525, 0.3350859294116795, 0.34997695130920736, 0.3501330474580613, 0.3522708607445432, 0.3524906992305994, 0.35284515848589443, 0.3725745212446662, 0.38040783334599104, 0.3817365920457004, 0.38644551396020443, 0.387134109977099, 0.3878971920972979, 0.38817521584546505, 0.4100797172690344, 0.4200051814485588, 0.4239704430629162, 0.4627118314281926, 0.463580804949956, 0.4637568774931365, 0.46428892125919324, 0.46460570083330316, 0.5060520618833959, 0.5069657621020269, 0.5093670443037324, 0.5245240567083429, 0.5247070694646409, 0.5276435076057755, 0.5307324331507823, 0.5310691422905622, 0.5314280728839472, 0.5478298906524353, 0.5481526986423813, 0.5482593446831252, 0.5502036687883113, 0.5504408750686187, 0.5675366183253524, 0.5814001919522828, 0.5955311437525788, 0.5956676660037171, 0.5958708903598392, 0.6078366984620305, 0.6106199036233864, 0.610939937655525, 0.611958452910949, 0.6133352170989014, 0.6140937378966043, 0.6191562272926154, 0.6361388717987714, 0.6612415215824481, 0.6680100380410444, 0.6685978443997784, 0.6691392451115941, 0.6694410728212956, 0.6705220978914267, 0.7200733295064959, 0.7245323898250954, 0.726953424571807, 0.7456694220626813, 0.746003218821703, 0.7461350060224803, 0.7463898540183249, 0.7473012297628313, 0.7568624729649742, 0.757371933544934, 0.758272618338216, 0.7603637235626823, 0.7615028421233028, 0.7772583083674716, 0.7892158507158582, 0.7907885750027731, 0.8115975849622982, 0.8165678351180219, 0.8169986838349484, 0.8175254944885001, 0.8209769953646282, 0.8300300472207452, 0.8408091039031471, 0.8503092482609652, 0.8511694300792721, 0.8512756623441988, 0.8591622006132519, 0.8843081715272795, 0.8941721442202377, 0.924451649707872, 0.9249053809527986, 0.9470096888069099, 0.9473486168564398, 0.9474564258075704, 0.9488098403130102, 0.9604982138946541, 0.9608462665285806, 0.9630989002009397, 0.9740013853555531, 0.9798422178448776, 0.9804917082190705, 1.0115417610952397, 1.0192853464760072, 1.0382895217714463, 1.038823721910161, 1.0415240357173854, 1.0435119902449685, 1.043822391136619, 1.0448156088528395, 1.0517638886270122, 1.0737416586681734, 1.0836540392333291, 1.083755621284031, 1.0997093196546408, 1.1014211992832883, 1.1017387310839661, 1.1410524359221088, 1.1543152236715215, 1.1544943809191999, 1.1549554440007437, 1.1554863983517034, 1.1719531997646644, 1.1725953329057452, 1.1979430919459675, 1.2];
    (p, toggles, stops)
}

#[test]
fn exact_mode_enters_a_stepped_seeds_hold_in_the_owners_table_after_a_regime_re_solve() {
    // Cold 7 tpl 5 in exact mode: a 555 on the rail with its ground pin on
    // node 3, in the stepped rail partition. The regime transition's
    // re-solve of that partition (`exact_resolve`) seeds it, and the seed's
    // hold procedure holds the 555 off (`stepped::hold_around`). Left in the
    // stepped solver's copy and never entered in the owner's table, the hold
    // was dropped at the next table handover and taken again: 156 holds and
    // releases over the run, where one hold is kept.
    let (p, toggles, stops) = cold_7_tpl5();
    let e = sweep_run(&p, &toggles, &stops).unwrap_or_else(|(e, t)| panic!("{} at {t}: {}", e.code, e.message));
    assert!(e.stat(25) >= 1.0 && e.stat(25) <= 2.0, "held once and kept: {} holds", e.stat(25));
}

fn cold_2575_tpl2() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.984117245919062, 0.10655500310025036], vec![]),
        fz(24, 1, &[3, 0], vec![-2.776143448201199, 10.62271198229284, 0.019224965471389778, 0.0011146330296212504, 0.0011146330296212504, 0.00211290312286571, 0.08966155041286154], vec![8.0]),
        fz(1, 2, &[2, 0], vec![820463.1551430054], vec![]),
        fz(1, 3, &[4, 0], vec![22519.658657749274], vec![]),
        fz(1, 4, &[5, 0], vec![669956.6952925198], vec![]),
        fz(1, 5, &[6, 0], vec![4633.8280209985505], vec![]),
        fz(40, 6, &[1, 0, 1, 2, 6, 6, 1, 1], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 7, &[1, 1], vec![565.4538523477618], vec![]),
        fz(1, 8, &[1, 1], vec![34661.853714269266], vec![]),
        fz(3, 9, &[1, 0], vec![1.1883016115495022e-8, 0.0, 0.0], vec![]),
        fz(1, 10, &[2, 1], vec![105.69729298224514], vec![]),
        fz(1, 11, &[0, 6], vec![6490.686997769565], vec![]),
        fz(4, 12, &[2, 0], vec![0.0], vec![]),
        fz(1, 13, &[0, 0], vec![3107.326817419344], vec![]),
        fz(4, 14, &[1, 3], vec![1.0], vec![]),
        fz(1, 15, &[3, 0], vec![2364.6610521719144], vec![]),
        fz(13, 16, &[3, 0], vec![0.7612055434997812, 0.04342360439622982], vec![]),
        fz(3, 17, &[1, 6], vec![2.693740383515899e-7, 0.39884509959352876, 0.0], vec![]),
        fz(1, 18, &[4, 0], vec![2.6582292915898424], vec![]),
        fz(1, 19, &[3, 4], vec![157180.1611373908], vec![]),
        fz(1, 20, &[4, 6], vec![2350.2658937432475], vec![]),
        fz(4, 21, &[2, 3], vec![0.0], vec![]),
        fz(1, 22, &[3, 0], vec![78.53652641416919], vec![]),
        fz(1, 23, &[4, 5], vec![17619.584895627977], vec![]),
        fz(90, 7, &[], vec![7.0, 0.020638933360439853, 565.4538523477618], vec![]),
        fz(90, 8, &[], vec![8.0, 0.01021098792962712, 34661.853714269266], vec![]),
        fz(90, 10, &[], vec![10.0, 0.1092148261682321, 105.69729298224514], vec![]),
        fz(90, 11, &[], vec![11.0, 0.007282598521730152, 6490.686997769565], vec![]),
        fz(90, 13, &[], vec![13.0, 0.00022862519904999048, 3107.326817419344], vec![]),
        fz(90, 15, &[], vec![15.0, 0.14653657477295207, 2364.6610521719144], vec![]),
        fz(90, 18, &[], vec![18.0, 13.28565863651707, 2.6582292915898424], vec![]),
        fz(90, 19, &[], vec![19.0, 9.1675114392305e-6, 157180.1611373908], vec![]),
        fz(90, 22, &[], vec![22.0, 1.1492992570160048, 78.53652641416919], vec![]),
        fz(90, 23, &[], vec![23.0, 0.00011275247802067702, 17619.584895627977], vec![]),
    ];
    let mut p = program(7, devices);
    p.digital_slot_count = 32;
    p
}

const STOPS_2575: [f64; 144] = [0.006272885454268746, 0.006722358845361651, 0.030082069747425742, 0.030305514505074526, 0.07737467669015351, 0.11079122496282161, 0.11101267854173187, 0.1113951420967517, 0.11164419206590313, 0.11180195028027584, 0.1138701213278603, 0.11650259652189927, 0.11675684593067406, 0.11692578059784811, 0.12231022624887829, 0.12242676381306818, 0.12269457187038708, 0.14557142082448843, 0.1514403471524137, 0.15217171434011573, 0.153632621076611, 0.15632653220215822, 0.17836178888619977, 0.1799941324399469, 0.18047972075070431, 0.22265137577819227, 0.25672253328407624, 0.2968355431585014, 0.3064171231383485, 0.30688352703536487, 0.3166121668293984, 0.33316069978529217, 0.35593598078893585, 0.35632858407926943, 0.3567093322188216, 0.3568610316481533, 0.3570258849223718, 0.3588561046044093, 0.3674662495607346, 0.37175636193375877, 0.37317703863544827, 0.3795858435485576, 0.3797855597249306, 0.38146413740463214, 0.4016293832812307, 0.40393609255256335, 0.4053301890226898, 0.44492194354769654, 0.4735156336506627, 0.4860672140295934, 0.5054636814031529, 0.5056213307976068, 0.506186254598784, 0.5063994240091841, 0.5065156050996727, 0.5186636110113361, 0.5200976434439157, 0.5274607808928731, 0.5378025079205805, 0.5667890865375645, 0.5670166441290978, 0.5678225549865811, 0.5687631669913571, 0.5836424522695971, 0.5848768777581916, 0.5902366435220326, 0.614503823816892, 0.6429047511219942, 0.6432247246476697, 0.6470889473414827, 0.6490163406287742, 0.6762247114388868, 0.6795320793458005, 0.6800198505890974, 0.6803932584303227, 0.6817750765335221, 0.6819841119665235, 0.6847669906360809, 0.6853687843254429, 0.6895029689998767, 0.6984460485300185, 0.7016778913941555, 0.7087958014950311, 0.7154846718803052, 0.7183619292769835, 0.7218445496130511, 0.7223110340740124, 0.7247842218978595, 0.725051741895515, 0.7262447841499476, 0.7265708656173511, 0.7268138907606634, 0.7275653545693996, 0.7384622599982188, 0.7499971678142786, 0.7691818876925283, 0.7694417255295235, 0.8026818300912854, 0.8029622039682913, 0.8047022924910198, 0.8057310874381503, 0.8064638171716066, 0.8076940084282942, 0.8156975243194418, 0.8165081417132876, 0.8166318682116716, 0.8286683076718543, 0.8308657692853977, 0.8312159213788277, 0.8315494819225858, 0.8685473107410446, 0.8691493498018293, 0.8695113574669172, 0.9054047813416228, 0.9340903172156757, 0.9521403437392756, 0.9543873550382468, 0.9656266456197006, 0.9670293630256126, 0.9674565417937, 0.9678220131197381, 0.9726532211607167, 1.0017706854196933, 1.0022022420650438, 1.009710339541874, 1.0104472581284287, 1.0335355896152079, 1.0359573122741632, 1.0372701531874613, 1.0384282283459234, 1.0389323403255502, 1.0500829132535774, 1.0991813283619407, 1.1022758598967872, 1.1025908131810065, 1.1049535032499194, 1.1050610457213186, 1.1330034529780417, 1.137007465821038, 1.1371086660730299, 1.1711438348268193, 1.1935460904267732, 1.1945409348701024, 1.2];

#[test]
fn a_cc_rail_freed_by_a_switch_holds_its_555_off_at_the_power_edge_once_per_toggle() {
    // Rulings R89 (a) and R91 (a), cold 2575 tpl 2: a {4.984 V, 106.6 mA}
    // PSU in CC, its rail tied to a pulse source at -2.776 V through switch
    // 14 until the switch opens at 30.08 ms. Then the limit charges the 11.9
    // nF rail capacitor at 106.6 mA / 11.9 nF = 9.0e6 V/s toward the
    // setpoint. A 555 has VCC, TRIG, THR and DIS on the rail: by hand it
    // powers at its 1.8 V edge (0.9 x VCC_MIN 2 V), 0.51 us after the
    // toggle ((1.8 + 2.776) / 9.0e6 s), and powered, THR at VCC puts OUT LOW
    // and DIS on, which load the rail past the limit (classic, with the edge
    // lowered to 0.9 V, holds the rail at 0.96 V powered). By ruling G (with
    // R68) the 555 is held off there, once per episode, and the rail goes on
    // to the setpoint; each later toggle starts a new episode. Exact mode
    // declines at 30.0846 ms (R54 (a)'s guard) and classic stalls at 1.81 V:
    // the default-versus-exact split is an R53/R70 item. Before the fix the
    // hub partitions' trials, planned at the toggle's instant before the
    // edit, survived it, and the host's next stop clamped the rail 0.22 ms
    // with the 555's power judged at -2.776 V: never on, the rail ran up the
    // capacitor past the setpoint to 749.6 V, and the Rail events then chased
    // it for 82M events in one advance.
    let p = cold_2575_tpl2();
    let (v_set, edge, d555) = (4.984117245919062, 1.8, 6);
    let toggles = [(0.030082069747425742, 14), (0.3166121668293984, 21), (1.009710339541874, 21), (1.0359573122741632, 12)];
    let mut rig = Rig::load(&p).unwrap();
    let mut state: Vec<f64> = p.devices.iter().map(|d| d.params.first().copied().unwrap_or(0.0)).collect();
    let (mut ev, mut holds) = (0, Vec::new());
    for &stop in &STOPS_2575 {
        rig.advance(stop).unwrap_or_else(|e| panic!("{e:?} at {}", rig.k.time()));
        assert!(rig.u() <= v_set + 1e-9, "the rail at {} V at {stop} s, past the {v_set} V setpoint", rig.u());
        // The hold the last toggle's episode took, read at the next stop.
        if ev > 0 && holds.len() < ev {
            let hold = rig.s().power_hold(d555);
            let release = hold.release.unwrap_or_else(|| panic!("the 555 free at {stop} s after toggle {}", ev - 1));
            assert_eq!(hold.kind, digital::power::HoldKind::Off);
            assert!((release.level - edge).abs() <= 1e-9 && release.rising, "held at {release:?}, by hand at the {edge} V edge");
            let after = release.since - toggles[ev - 1].0;
            assert!(after > 0.0 && after <= 1e-6, "held {after} s after toggle {}", ev - 1);
            holds.push(after);
        }
        while ev < toggles.len() && toggles[ev].0 <= stop {
            let d = toggles[ev].1;
            state[d] = 1.0 - state[d];
            rig.k.apply_params_as_event(&[(d, 0, state[d])]).unwrap_or_else(|e| panic!("toggle {d} at {stop}: {e:?}"));
            rig.k.solver_mut().prepare_readout();
            ev += 1;
        }
    }
    assert_eq!((ev, holds.len()), (toggles.len(), toggles.len()), "every toggle applied, each with its hold");
    assert_eq!(rig.s().hold_stats(), [toggles.len() as f64, 0.0], "one hold per toggle and no release: {holds:?}");
    assert!((rig.k.time() - 1.2).abs() < 1e-12, "the run reached 1.2 s");
    // The first hold by hand: (1.8 + 2.776) V at 9.0e6 V/s, 0.51 us after the
    // switch opened.
    assert!((holds[0] - 5.1e-7).abs() <= 0.05e-6, "the first hold {} s after the toggle, by hand 0.51 us", holds[0]);
}

/// tpl=1 hub=2 e=9.038 psu ilim=0.189 V2=-2.24 F2-0=0.06/0.64 R2-4=8388.0 R0-2=16.0 C4-0=3.0e-6 P3-0 R1-2=1.1 R2-3=43338.1 SW0-4(0)+R4-0=160 555 vcc1 gnd0 [2, 4, 1, 2, 1, 2] R1-4=21.9 R4-3=391.7 R0-1=6.2 R1-2=5.9 R3-4=203.8 R4-0=1.4 span=9.038178759358374
fn cold_1838_tpl1() -> (Program, Vec<(f64, usize)>, Vec<f64>) {
    let mut p = program(5, vec![
        fz(21, 0, &[1, 0], vec![9.038178759358374, 0.1886199776479608], vec![]),
        fz(2, 1, &[0, 2], vec![-2.235542083308246], vec![]),
        fz(1, 2, &[3, 0], vec![62129.52679061059], vec![]),
        fz(1, 3, &[4, 0], vec![10721.20835458154], vec![]),
        fz(10, 4, &[2, 0], vec![0.0627123172794748, 0.6396085285823838], vec![]),
        fz(1, 5, &[2, 4], vec![8388.005535454064], vec![]),
        fz(1, 6, &[0, 2], vec![16.026440631397108], vec![]),
        fz(3, 7, &[4, 0], vec![3.0365775649267877e-6, 0.0, 0.0], vec![]),
        fz(13, 8, &[3, 0], vec![0.3165191463907781, 0.7946375484519316], vec![]),
        fz(1, 9, &[1, 2], vec![1.0541636073007479], vec![]),
        fz(1, 10, &[2, 3], vec![43338.11828517909], vec![]),
        fz(4, 11, &[0, 4], vec![0.0], vec![]),
        fz(1, 12, &[4, 0], vec![159.82419439786543], vec![]),
        fz(40, 13, &[1, 0, 2, 4, 1, 2, 1, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 14, &[1, 2], vec![83138.45869818091], vec![]),
        fz(1, 15, &[2, 1], vec![36997.72548599595], vec![]),
        fz(3, 16, &[1, 0], vec![2.2922775466410644e-7, 0.0, 0.0], vec![]),
        fz(1, 17, &[4, 1], vec![636.9224478823085], vec![]),
        fz(1, 18, &[1, 4], vec![21.875088976298834], vec![]),
        fz(1, 19, &[4, 3], vec![391.74873660307725], vec![]),
        fz(1, 20, &[0, 1], vec![6.176189366136723], vec![]),
        fz(1, 21, &[1, 2], vec![5.891373734988909], vec![]),
        fz(1, 22, &[3, 4], vec![203.7637394318904], vec![]),
        fz(1, 23, &[4, 0], vec![1.4204624827579135], vec![]),
        fz(90, 5, &[], vec![5.0, 0.00011104339588934994, 8388.005535454064], vec![]),
        fz(90, 6, &[], vec![6.0, 0.32291149187341306, 16.026440631397108], vec![]),
        fz(90, 9, &[], vec![9.0, 0.6804444928174624, 1.0541636073007479], vec![]),
        fz(90, 10, &[], vec![10.0, 0.0018233268484194789, 43338.11828517909], vec![]),
        fz(90, 12, &[], vec![12.0, 0.024250597078358596, 159.82419439786543], vec![]),
        fz(90, 14, &[], vec![14.0, 0.00011590197565282543, 83138.45869818091], vec![]),
        fz(90, 15, &[], vec![15.0, 0.00012873756009805108, 36997.72548599595], vec![]),
        fz(90, 17, &[], vec![17.0, 0.05315352769220552, 636.9224478823085], vec![]),
        fz(90, 18, &[], vec![18.0, 0.06733231106878386, 21.875088976298834], vec![]),
        fz(90, 19, &[], vec![19.0, 0.03350264714193744, 391.74873660307725], vec![]),
        fz(90, 20, &[], vec![20.0, 7.631500994637584, 6.176189366136723], vec![]),
        fz(90, 21, &[], vec![21.0, 1.9760956833902152, 5.891373734988909], vec![]),
        fz(90, 22, &[], vec![22.0, 0.0022579876823534436, 203.7637394318904], vec![]),
        fz(90, 23, &[], vec![23.0, 66.0522583593123, 1.4204624827579135], vec![]),
    ]);
    p.digital_slot_count = 32;
    (p, vec![(0.4810911852304546, 11)], vec![0.006272877922672752, 0.0065830355553143675, 0.0074409128450704165, 0.008419724729443388, 0.03846897546721881, 0.03898147777673475, 0.04029400655673618, 0.040866784295230295, 0.05861920057748794, 0.06982393264317256, 0.07033688639110738, 0.09056255362344044, 0.09092023671274244, 0.1169639299136157, 0.11751067560389039, 0.11802619278772375, 0.12293015468768474, 0.12656075777070291, 0.12691538865302016, 0.12945180416473184, 0.1343790533932548, 0.13449971399963337, 0.1409559690696507, 0.14166169315251853, 0.16096182037588352, 0.1611543358230243, 0.16165315132857697, 0.16177635940126145, 0.1619326428852814, 0.1792261048548625, 0.17942647966985237, 0.20167401781952132, 0.22582850188638737, 0.2261848906269315, 0.22947436784329034, 0.23226960324638812, 0.23281593311849588, 0.27465717470164286, 0.2780477345232049, 0.2797759127707064, 0.2799388963496314, 0.2813249067747996, 0.2818958437705419, 0.2848737731660255, 0.29664368461082596, 0.3230869975524387, 0.32328921363619084, 0.3509979144118802, 0.3657251127619349, 0.39404087206591004, 0.39883522279290584, 0.40093094574421156, 0.42089961793920294, 0.42122738342959726, 0.42149251254907366, 0.426963851742771, 0.4275026680971968, 0.42822220861186316, 0.443344977160614, 0.4440015761471034, 0.44539701308809493, 0.4461138543893673, 0.44699434136536087, 0.44962698598418266, 0.4508825107869241, 0.45435749819993376, 0.4545631177179382, 0.45605205907783347, 0.45680191720192825, 0.46471219053471247, 0.46481977137333136, 0.46514123664288193, 0.46956683322787873, 0.4740665703782788, 0.4810911852304546, 0.49684186616462506, 0.5135866854808602, 0.5191619652897942, 0.5378402519965905, 0.538426488720747, 0.5447733420189673, 0.5706883562388942, 0.5718913481558302, 0.6053654555765822, 0.605669539676143, 0.6058199960343957, 0.6069681666662698, 0.6072238581151079, 0.6092005910604106, 0.6093167369659557, 0.6101795920980541, 0.6111772434991882, 0.6163137916790911, 0.623244957152144, 0.6242111973682665, 0.6302092343313705, 0.6408356388640224, 0.6868737384543642, 0.6871213814768871, 0.6872879027875186, 0.6890536364460107, 0.6907602000079464, 0.6913365504427115, 0.7067714321329545, 0.7137938344630965, 0.7138977574850903, 0.7465292927807842, 0.7470458798162811, 0.7805048173492567, 0.7819799769756206, 0.7863028476268464, 0.7864675613245161, 0.7877218589522919, 0.80510194241779, 0.8056207406093908, 0.8059466134757225, 0.8093678672703827, 0.8161490108903859, 0.8174890498015316, 0.8336470064651188, 0.8355866899328154, 0.8520325329647838, 0.8741635893214501, 0.8743199920323446, 0.8745875708810075, 0.8748189148658394, 0.8845110858051677, 0.8847154939297424, 0.8860761941008093, 0.9196750886199698, 0.9200871122422988, 0.9477638879731541, 0.9541206146651996, 0.9695825479765098, 0.9702085437168949, 0.9712199972328406, 0.9921200519889347, 0.9957665455176753, 1.0084764928113552, 1.0087189152964806, 1.0094585334497714, 1.01349997934667, 1.014079235893828, 1.0171399886560584, 1.0541715157534461, 1.055062445643716, 1.0551988947398354, 1.0555027967787995, 1.0558015855564231, 1.0559433051015021, 1.0562645447227874, 1.0591842279871873, 1.0597582333655755, 1.0700716887894166, 1.071273753392158, 1.071934669867629, 1.0721005489574038, 1.072638759204678, 1.0727450124524571, 1.0892463227732314, 1.0972106233294245, 1.1450670352337333, 1.1483812885869216, 1.1499366226769523, 1.1500545668400302, 1.1806763532502242, 1.2])
}

/// tpl=5 hub=2 e=5.271 psu ilim=0.046 V2=3.54 555 vcc1 gnd0 [4, 3, 3, 2, 4, 5] SW2-4(1)+R4-0=387 R3-5=143.1 C0-1=4.9e-3 R2-4=9.9 span=5.270583419353784
fn cold_2548_tpl5() -> (Program, Vec<(f64, usize)>, Vec<f64>) {
    let mut p = program(6, vec![
        fz(21, 0, &[1, 0], vec![5.270583419353784, 0.04560418898693066], vec![]),
        fz(2, 1, &[2, 0], vec![3.5444468737343957], vec![]),
        fz(1, 2, &[3, 0], vec![54124.67229606257], vec![]),
        fz(1, 3, &[4, 0], vec![11722.523360837218], vec![]),
        fz(1, 4, &[5, 0], vec![125036.14834329223], vec![]),
        fz(40, 5, &[1, 0, 4, 3, 3, 2, 4, 5], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 6, &[1, 5], vec![1451.7194544449098], vec![]),
        fz(1, 7, &[5, 4], vec![426.1399501315587], vec![]),
        fz(3, 8, &[4, 0], vec![1.5327471986819734e-6, 0.0, 0.0], vec![]),
        fz(1, 9, &[3, 1], vec![1515.538411530625], vec![]),
        fz(4, 10, &[2, 4], vec![1.0], vec![]),
        fz(1, 11, &[4, 0], vec![386.93401295396995], vec![]),
        fz(1, 12, &[3, 5], vec![143.11902464018866], vec![]),
        fz(3, 13, &[0, 1], vec![0.004882413953692385, 0.0, 0.0], vec![]),
        fz(1, 14, &[2, 4], vec![9.900968347762008], vec![]),
        fz(1, 15, &[5, 3], vec![1331.6029419025076], vec![]),
        fz(90, 6, &[], vec![6.0, 0.0002304880556184839, 1451.7194544449098], vec![]),
        fz(90, 7, &[], vec![7.0, 0.016759577179403275, 426.1399501315587], vec![]),
        fz(90, 9, &[], vec![9.0, 0.02196551810784746, 1515.538411530625], vec![]),
        fz(90, 11, &[], vec![11.0, 0.029016303549898415, 386.93401295396995], vec![]),
        fz(90, 12, &[], vec![12.0, 0.001134488126943947, 143.11902464018866], vec![]),
        fz(90, 14, &[], vec![14.0, 0.19775525557804474, 9.900968347762008], vec![]),
    ]);
    p.digital_slot_count = 32;
    (p, vec![(0.5587307916987848, 10), (0.7258738709720594, 10), (1.1289110385931995, 10)], vec![0.00627288417468781, 0.009345194127096196, 0.013461209371189109, 0.04591948507275472, 0.06901610959838697, 0.08510949854904287, 0.10073912451799966, 0.10096462866220063, 0.10118140611564727, 0.13900479340852462, 0.13941085990020824, 0.14573950999554283, 0.14629897479500167, 0.1480554141083486, 0.14852575336411636, 0.14982298959711862, 0.15155331005615993, 0.16103964527457457, 0.1656232077943176, 0.17378721939163408, 0.17475495495371476, 0.17589454692416312, 0.21261604421630342, 0.24024352770008037, 0.24063537589965725, 0.24090000112805587, 0.2875312536956903, 0.28765095311281513, 0.2878184859470967, 0.28839979807090677, 0.2885202359691093, 0.289637503515954, 0.2898349340550851, 0.2901399753083409, 0.32707622644383844, 0.32751525263095355, 0.3279159153055835, 0.32813165890741525, 0.3284376776257342, 0.35587705620523236, 0.35795546682850754, 0.3638974133045557, 0.36413469929750397, 0.3673342915444422, 0.36748103787903663, 0.3676977724122154, 0.3678892769011583, 0.3681326598541484, 0.3737274442366686, 0.3738293237136612, 0.3755695294141925, 0.3764610829077447, 0.3766532231199138, 0.37686471585842735, 0.3769892724431306, 0.40261632810904163, 0.4518545499579383, 0.45220578717206955, 0.4538393143785291, 0.4543187738947217, 0.48035873597015805, 0.48098914088085987, 0.49689613340269, 0.497615853463733, 0.5071671020316566, 0.5133024268352636, 0.5155625256883754, 0.5157128002994075, 0.5587307916987848, 0.6048415204783117, 0.6066539271154384, 0.6067631176013111, 0.6090412465608539, 0.6232612234130372, 0.6328077808988549, 0.6402241294662816, 0.6403871613627874, 0.6453827075998735, 0.6527970759647789, 0.6554747778883148, 0.6614501892045032, 0.6618602487155637, 0.6621495776619365, 0.6674776888094994, 0.6680428424421597, 0.66870704283178, 0.6688382689885497, 0.6705510108060592, 0.6708153769659337, 0.6720661208688716, 0.6864799447608383, 0.6868968286768475, 0.7199757350932766, 0.7202631792858518, 0.7258738709720594, 0.7341099087453461, 0.7355693053170653, 0.7357362274826167, 0.73674283439845, 0.75274770726183, 0.7531967480449364, 0.7534088079636553, 0.7535179390285908, 0.7536591123347787, 0.779794208056135, 0.7799993286159939, 0.7988644181246647, 0.7993365808307656, 0.8005411484897926, 0.8016532637275813, 0.8105321714241882, 0.8274415129924708, 0.8533153238042401, 0.8538477392123358, 0.8906129340884036, 0.9068401818306424, 0.907590609476769, 0.9119815602373343, 0.9122462942677675, 0.9130663093767423, 0.9132106439624429, 0.9165942544428921, 0.9186183227144933, 0.919324207670255, 0.9196433647719058, 0.9247023917138981, 0.9291520919039948, 0.9293556277123151, 0.9308838503565195, 0.9398979622659989, 0.940082548060601, 0.9407315938816995, 0.94109555816568, 0.9512936492677643, 0.9514517218187973, 0.979502966903414, 1.0274351684479515, 1.0328677101122847, 1.0346956681986412, 1.0350176202200512, 1.0353776374637162, 1.0623870810876705, 1.0643457622205728, 1.0655810171968174, 1.0696907571472967, 1.0710894662226733, 1.095640937659783, 1.1214400041773787, 1.1286034127286082, 1.1289110385931995, 1.1312576876560927, 1.131682176728038, 1.1404731534514188, 1.1411480370846008, 1.1424211996992104, 1.1428640134735248, 1.142990241493031, 1.1908718459007972, 1.1911087971888388, 1.1941054768983703, 1.194551222792447, 1.2])
}

#[test]
fn rated_devices_on_a_settled_rail_segment_re_anchor_on_its_decayed_rates() {
    // Ruling R21's re-anchors, on two cold-fuzz boards whose CC rail settles
    // on a closed-form hub partition. Cold 1838 tpl 1: a {9.04 V, 189 mA}
    // PSU in CC at 2.026 V through 21.9 and 637 ohm rated resistors into a
    // 3 uF node; its segment's modes (27,302 V/s at k = 3.3e8 and 5.8e6 /s,
    // cancelling) had decayed within microseconds, but the bounds summed
    // them as aimed, and the two resistors re-anchored every 2 ns
    // (REDRIVE_MIN_DT): 16.7M events in a 15.8 ms host step, 167 s to 1.2
    // s. Cold 2548 tpl 5: the rail creeps at 8 V/s through a 555's 1.8 V
    // edge on a 1.5 ns segment, and the curved anchor let the node's rail
    // part (29 uV) decay with that segment's exponential, so it dropped it
    // whole within a few ns: re-anchors every 1-2 ns, 30M events a host
    // step. Bounded by the rates as they stand at the anchor and with that
    // part held, each board runs its 1.2 s in under 50,000 events.
    for (name, (p, toggles, stops)) in [("1838 tpl 1", cold_1838_tpl1()), ("2548 tpl 5", cold_2548_tpl5())] {
        let e = sweep_run(&p, &toggles, &stops).unwrap_or_else(|(e, t)| panic!("{name}: {} at {t}: {}", e.code, e.message));
        assert!((e.time() - 1.2).abs() < 1e-12, "{name} reached {}", e.time());
        assert!(e.stat(4) < 50_000.0, "{name}: {} events to 1.2 s", e.stat(4));
    }
}

fn cold_104_tpl6() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![8.687150581769643, 1.180118773796576], vec![]),
        fz(40, 1, &[1, 0, 3, 4, 1, 6, 3, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 2, &[1, 2], vec![2191.237251268685], vec![]),
        fz(1, 3, &[2, 3], vec![852.654212038412], vec![]),
        fz(3, 4, &[3, 0], vec![1.0154847829101675e-7, 0.0, 0.0], vec![]),
        fz(3, 5, &[6, 0], vec![1e-8, 0.0, 0.0], vec![]),
        fz(1, 6, &[4, 5], vec![1023.6384695918451], vec![]),
        fz(30, 7, &[5, 0], vec![3.2276682801520256, 2.08777834869681, 0.005037545679024358, 0.0, 0.0, 0.0], vec![]),
        fz(1, 8, &[4, 0], vec![16.86101837494531], vec![]),
        fz(90, 2, &[], vec![2.0, 0.002365034093071564, 2191.237251268685], vec![]),
        fz(90, 3, &[], vec![3.0, 0.0003943049059350338, 852.654212038412], vec![]),
        fz(90, 6, &[], vec![6.0, 0.010711382735221682, 1023.6384695918451], vec![]),
        fz(90, 8, &[], vec![8.0, 1.9439833317903972, 16.86101837494531], vec![]),
    ];
    let mut p = program(8, devices);
    p.digital_slot_count = 32;
    p.options.no_rail_shortcut = true;
    p.options.integration = 1;
    p
}

const COLD_104_TPL6_STOPS: &[f64] = &[0.006272879830282593, 0.006960822680210238, 0.03071029095223489, 0.041479051031954955, 0.04726268692453734, 0.05045957180685165, 0.05211002263275876, 0.05232404780116879, 0.05255293663601682, 0.07666730054396512, 0.08352946186073991, 0.08393607249334734, 0.09363782624136749, 0.09392697172623517, 0.09577386995266314, 0.09764821222649518, 0.09820365539657783, 0.09998862057558866, 0.10069605184992905, 0.14];

fn cold_282_tpl6() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![13.292070515256059, 0.7573386050355044], vec![]),
        fz(40, 1, &[1, 0, 3, 4, 1, 6, 3, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 2, &[1, 2], vec![278.08852894571385], vec![]),
        fz(1, 3, &[2, 3], vec![2173.1937015167937], vec![]),
        fz(3, 4, &[3, 0], vec![6.172520612032082e-6, 0.0, 0.0], vec![]),
        fz(3, 5, &[6, 0], vec![1e-8, 0.0, 0.0], vec![]),
        fz(1, 6, &[4, 5], vec![21.68480848123426], vec![]),
        fz(30, 7, &[5, 0], vec![1.6415888198857833, 1.595416792681203, 0.035732733875483286, 0.0, 0.0, 0.0], vec![]),
        fz(1, 8, &[4, 0], vec![582.7161311356624], vec![]),
        fz(90, 2, &[], vec![2.0, 0.04362890187070934, 278.08852894571385], vec![]),
        fz(90, 3, &[], vec![3.0, 0.0008651100548062045, 2173.1937015167937], vec![]),
        fz(90, 6, &[], vec![6.0, 2.5837621206687995, 21.68480848123426], vec![]),
        fz(90, 8, &[], vec![8.0, 0.0006899432167843668, 582.7161311356624], vec![]),
    ];
    let mut p = program(7, devices);
    p.digital_slot_count = 32;
    p.options.no_rail_shortcut = true;
    p.options.integration = 1;
    p
}

const COLD_282_TPL6_STOPS: &[f64] = &[0.00627287906594856, 0.007759280870621779, 0.019473317143096153, 0.019582017040386174, 0.028210273278552245, 0.04418788592979969, 0.04972810566492482, 0.059732951588839414, 0.06424379020161758, 0.06736620232207068, 0.06816774509004235, 0.07190852553600559, 0.07673162631625222, 0.12];

fn cold_7_tpl6() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![4.921430677373768, 1.8943974603642248], vec![]),
        fz(40, 1, &[1, 0, 3, 4, 1, 6, 3, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 2, &[1, 2], vec![12141.990038265814], vec![]),
        fz(1, 3, &[2, 3], vec![1065.4650562552722], vec![]),
        fz(3, 4, &[3, 0], vec![5.1806354724289366e-6, 0.0, 0.0], vec![]),
        fz(3, 5, &[6, 0], vec![1e-8, 0.0, 0.0], vec![]),
        fz(1, 6, &[4, 5], vec![385.22426114176847], vec![]),
        fz(30, 7, &[5, 0], vec![2.0481761097158655, 1.8315076018960104, 0.007398767899840754, 0.0, 0.0, 0.0], vec![]),
        fz(90, 2, &[], vec![2.0, 0.00013698247125092347, 12141.990038265814], vec![]),
        fz(90, 3, &[], vec![3.0, 0.027755271666707194, 1065.4650562552722], vec![]),
        fz(90, 6, &[], vec![6.0, 0.00038101217437008783, 385.22426114176847], vec![]),
    ];
    let mut p = program(7, devices);
    p.digital_slot_count = 32;
    p.options.no_rail_shortcut = true;
    p.options.integration = 0;
    p
}

const COLD_7_TPL6_STOPS: &[f64] = &[0.006272879599356148, 0.016698232994375808, 0.017516190773174106, 0.0210471593856631, 0.023700230091663114, 0.024218131306263185, 0.024882135080202174, 0.03190567475933873, 0.04152108682251448, 0.05457620614236116, 0.05475715862700885, 0.05502787837056468, 0.055240284308500524, 0.05547824214556055, 0.07018945334296087, 0.0753977036576992, 0.12439770398421504, 0.12631407096851402, 0.1361452360811551, 0.14576710069909615, 0.15032218991751076, 0.19816592540611697, 0.19834690583834919, 0.20593281067285235, 0.2083771599560474, 0.21564673292461814, 0.2157682989750026, 0.2179475877035891, 0.24408607702474405, 0.27639781167971944, 0.2766101985117782, 0.28246624787966285, 0.29573011299706525, 0.33];

fn cold_570_tpl6() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![13.647403951331636, 0.048202624113554284], vec![]),
        fz(40, 1, &[1, 0, 3, 4, 1, 6, 3, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 2, &[1, 2], vec![2891.6365905004805], vec![]),
        fz(1, 3, &[2, 3], vec![6742.850827704528], vec![]),
        fz(3, 4, &[3, 0], vec![1.3945407714519867e-6, 0.0, 0.0], vec![]),
        fz(3, 5, &[6, 0], vec![1e-8, 0.0, 0.0], vec![]),
        fz(1, 6, &[4, 5], vec![572.6007815138817], vec![]),
        fz(30, 7, &[5, 0], vec![3.343020326655086, 2.055198801293253, 0.03362483906776208, 0.0, 0.0, 0.0], vec![]),
        fz(90, 2, &[], vec![2.0, 0.0044231187020022465, 2891.6365905004805], vec![]),
        fz(90, 3, &[], vec![3.0, 0.00028827904257487045, 6742.850827704528], vec![]),
        fz(90, 6, &[], vec![6.0, 0.16471244672539137, 572.6007815138817], vec![]),
    ];
    let mut p = program(8, devices);
    p.digital_slot_count = 32;
    p.options.no_rail_shortcut = true;
    p.options.integration = 1;
    p
}

const COLD_570_TPL6_STOPS: &[f64] = &[0.006272880886146131, 0.007785429794059074, 0.01542749630025457, 0.01633988555851327, 0.016683463569004787, 0.018343443761369482, 0.01853056749373825, 0.02600908587079125, 0.06855045350713991, 0.07705821184659765, 0.07764466706352305, 0.08324367010259509, 0.08338214093907599, 0.10876267700558855, 0.1150744200194748, 0.11618454136287167, 0.12382987425673352, 0.13];

/// tpl=6 hub=2 e=6.565 psu ilim=1.482 555-astable R1=8435 R2=4561 C=1.1e-7 Rl=1988 span=6.564756609058284
fn cold_178_tpl6() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![6.564756609058284, 1.4821403148193875], vec![]),
        fz(40, 1, &[1, 0, 3, 4, 1, 6, 3, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 2, &[1, 2], vec![8435.22878545403], vec![]),
        fz(1, 3, &[2, 3], vec![4560.734511392359], vec![]),
        fz(3, 4, &[3, 0], vec![1.0503712439356632e-7, 0.0, 0.0], vec![]),
        fz(3, 5, &[6, 0], vec![1e-8, 0.0, 0.0], vec![]),
        fz(1, 6, &[4, 5], vec![1987.7464371959259], vec![]),
        fz(30, 7, &[5, 0], vec![3.329325427496355, 1.561370934608731, 0.008451322468125525, 0.0, 0.0, 0.0], vec![]),
        fz(90, 2, &[], vec![2.0, 0.00035084273758161853, 8435.22878545403], vec![]),
        fz(90, 3, &[], vec![3.0, 0.003149644863725417, 4560.734511392359], vec![]),
        fz(90, 6, &[], vec![6.0, 6.531809582376052e-5, 1987.7464371959259], vec![]),
    ];
    let mut p = program(7, devices);
    p.digital_slot_count = 32;
    p.options.no_rail_shortcut = true;
    p.options.integration = 1;
    p
}

const COLD_178_TPL6_STOPS: &[f64] = &[0.006272879989810924, 0.039591561527473854, 0.04113242362252593, 0.04633288529402665, 0.05542991846571776, 0.08113995033884838, 0.08569411954555575, 0.09714763891750725, 0.09724974479107461, 0.09792784365749811, 0.12789199038934745, 0.14168849343511414, 0.14178851039003526, 0.15279472456816426, 0.15312907148430938, 0.1532662803867128, 0.15594854418210113, 0.20170412184231412, 0.20264987603553103, 0.20297265660972424, 0.20382095005012635, 0.20560042372019133, 0.20610236116676267, 0.21947976437656874, 0.23895084510022446, 0.23910701037912654, 0.2544443522190506, 0.2562223735270833, 0.260055508460714, 0.27968606715899097, 0.2798280147801551, 0.28];

/// tpl=6 hub=2 e=7.584 psu ilim=0.027 555-astable R1=6845 R2=835 C=5.8e-7 Rl=82 span=7.5844459169690355
fn cold_323_tpl6() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![7.5844459169690355, 0.027121965452500285], vec![]),
        fz(40, 1, &[1, 0, 3, 4, 1, 6, 3, 2], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 2, &[1, 2], vec![6845.37563705009], vec![]),
        fz(1, 3, &[2, 3], vec![835.2602122402577], vec![]),
        fz(3, 4, &[3, 0], vec![5.847523636981566e-7, 0.0, 0.0], vec![]),
        fz(3, 5, &[6, 0], vec![1e-8, 0.0, 0.0], vec![]),
        fz(1, 6, &[4, 5], vec![81.58419136195944], vec![]),
        fz(30, 7, &[5, 0], vec![3.2427480293450888, 1.926600546108804, 0.023737020421771927, 0.0, 0.0, 0.0], vec![]),
        fz(90, 2, &[], vec![2.0, 0.0005770620323295561, 6845.37563705009], vec![]),
        fz(90, 3, &[], vec![3.0, 0.01502685191084184, 835.2602122402577], vec![]),
        fz(90, 6, &[], vec![6.0, 0.044641478816532666, 81.58419136195944], vec![]),
    ];
    let mut p = program(7, devices);
    p.digital_slot_count = 32;
    p.options.no_rail_shortcut = true;
    p.options.integration = 1;
    p
}

const COLD_323_TPL6_STOPS: &[f64] = &[0.006272879160256394, 0.05535092949592646, 0.10481230682950853, 0.11527294127011199, 0.13544230657419612, 0.1425184897702758, 0.1430545754801801, 0.16002370994072068, 0.16023833387438932, 0.16060010380984183, 0.20942365577760064, 0.2152741227732425, 0.21599672967339395, 0.21612024791932158, 0.2206646723288724, 0.22110219405915874, 0.22135086809875998, 0.2216684394091824, 0.23415642820515228, 0.2704899644231742, 0.2977073677208505, 0.29811775975065585, 0.3010068864724767, 0.30416087621197707, 0.31040177427821053, 0.3106633360106082, 0.3310106651986903, 0.3325610115143301, 0.3326618071969605, 0.3347095677795212, 0.3353067679494328, 0.33670368614452495, 0.36268031185999594, 0.3713116843243565, 0.3720017772842537, 0.39989290396976696, 0.4412440546801241, 0.44835175881455724, 0.44976867855233726, 0.4615625580715273, 0.46179490661104244, 0.4646512067578881, 0.46532793650421117, 0.4815804354168739, 0.4875946419101712, 0.4878116690534695, 0.4887082049480184, 0.5189565934364115, 0.5201173732165757, 0.5203391317274425, 0.5205264675245741, 0.5217735236526293, 0.5244804095808212, 0.5312238569747123, 0.5314712897571585, 0.5413219275857487, 0.541610489526552, 0.5475880191776681, 0.5512251967416646, 0.5573142640570334, 0.5580215105762409, 0.5600907967508648, 0.5649443085461386, 0.566841549084585, 0.5692310526156734, 0.569401336138474, 0.5701583221717537, 0.5708068313753576, 0.5735420438033615, 0.5767734193897418, 0.5915915735114808, 0.5918026544146443, 0.5922037175113217, 0.5929514263626646, 0.5956957268876544, 0.5960872673719217, 0.5962164838441434, 0.6033210820255037, 0.6094056508646937, 0.6101126769911732, 0.6228340927553545, 0.6232389986915935, 0.6236327585024883, 0.6243003502488595, 0.6247764401610721, 0.6257838205491483, 0.6262084665884904, 0.6264889785226121, 0.6500336677514003, 0.6531587383552486, 0.6533801316354629, 0.6674118036471551, 0.6691834261360949, 0.670244150166756, 0.6707592756269632, 0.7040013121485482, 0.7043032219530623, 0.7045324467799459, 0.7127101973108994, 0.7292615379750079, 0.729573427725888, 0.7492836811574229, 0.7566033224982766, 0.7731435588104643, 0.7755603411532712, 0.7812792431451344, 0.803423119957695, 0.8069902951405226, 0.8157998510602946, 0.8160156440523355, 0.8165006399631444, 0.827975730253516, 0.8282946966699762, 0.8286594833372558, 0.8289322512507298, 0.8314958790528281, 0.8319313404549481, 0.8357850968670762, 0.8482016205382965, 0.848478644568373, 0.8486504401497689, 0.8561378876501377, 0.859840598840099, 0.8600023971079562, 0.9093589503452719, 0.9294147049020834, 0.9430137722903487, 0.9436067761329283, 0.9687279411715883, 0.9694349467626846, 0.977996279091322, 0.9784250691656596, 1.003857107413984, 1.0040384670465305, 1.0101925645617762, 1.0103602925744821, 1.03];

/// The resistor latches (device, time) an exact-mode 555 astable board records
/// over `stops`.
fn resistor_latches(p: &Program, stops: &[f64]) -> Vec<(usize, f64)> {
    let mut e = Engine::load(p).unwrap_or_else(|er| panic!("load: {}", er.message));
    for &t in stops {
        e.advance(t).unwrap_or_else(|er| panic!("advance to {t}: {} {}", er.code, er.message));
    }
    e.records(0).unwrap().chunks(7).filter(|c| c[1] == 1.0).map(|c| (c[0] as usize, c[3])).collect()
}

#[test]
fn a_stepped_resistors_stress_is_the_mean_of_its_steps_start_and_end_states() {
    // Rulings R80 (a), R84 (B): a stepped device's stress over a committed
    // step is the mean of the drive at the step's own start state and at its
    // end state (the logarithmic mean of the ratio the rate is taken from).
    // Classic's backward rule held the end rate over the step; on a
    // 555 astable's timing resistors, which take most of their stress in the
    // discharge's exponential spikes, it under-counted by 5-17 % in exact
    // mode (the rail's partition stepped), where default mode's closed form
    // matched the hand integral. Hand values from an independent model of
    // each board (ideal supply, thresholds at Vcc/3 and 2 Vcc/3, DIS 10 ohm,
    // S' = max(0, P/Pmax - 1), a latched resistor open):
    // - 104 tpl 6 (R1 2191, R2 853 ohm rated 0.39 mW, 0.1 uF, 8.69 V): R2
    //   carries 3.6-6.6 mA through each discharge and burns at 0.072553 s,
    //   freezing the capacitor; R1 then burns at 0.128156 s on the DIS
    //   current. The backward rule burnt R2 at 0.0851 s.
    // - 282 tpl 6 (R1 278, R2 2173 ohm, 6.2 uF, 13.29 V): R2 at 0.044604 s,
    //   R1 at 0.107587 s; the backward rule burnt R2 at 0.0499 s, in another
    //   phase, and R1 never.
    // - 7 tpl 6 (R1 12142, R2 1065 ohm, 5.2 uF, 4.92 V, Euler): R1 at
    //   0.307912 s; the backward rule at 0.3240 s.
    // - 570 tpl 6 (R1 2892, R2 6743 ohm, 1.4 uF, 13.65 V): R2 at 0.062948 s,
    //   R1 at 0.119089 s; the backward rule burnt R2 at 0.0691 s and R1 never.
    // - 178 tpl 6 (R1 8435 ohm, 0.11 uF, 6.56 V): R1 at 0.270519 s; its ratio
    //   falls through the zero level in each charging step, and read at the
    //   level that step accrued throughout and never bled: 0.2561 s.
    // - 323 tpl 6 (R1 6845 ohm on a 27 mA supply its 82 ohm output load holds
    //   in CC, which the ideal-supply hand model lacks): R1 at 1.0227 s in the
    //   1 us fine reference; 0.7825 s read at the level, 1.1806 s backward.
    let boards: [(&str, Program, &[f64], &[(usize, f64)]); 6] = [
        ("104/6", cold_104_tpl6(), COLD_104_TPL6_STOPS, &[(3, 0.072553), (2, 0.128156)]),
        ("282/6", cold_282_tpl6(), COLD_282_TPL6_STOPS, &[(3, 0.044604), (2, 0.107587)]),
        ("7/6", cold_7_tpl6(), COLD_7_TPL6_STOPS, &[(2, 0.307912)]),
        ("570/6", cold_570_tpl6(), COLD_570_TPL6_STOPS, &[(3, 0.062948), (2, 0.119089)]),
        ("178/6", cold_178_tpl6(), COLD_178_TPL6_STOPS, &[(2, 0.270519)]),
        ("323/6", cold_323_tpl6(), COLD_323_TPL6_STOPS, &[(2, 1.0227)]),
    ];
    for (name, p, stops, hand) in boards {
        let got = resistor_latches(&p, stops);
        for &(d, t) in hand {
            let at = got.iter().find(|r| r.0 == d).map(|r| r.1);
            assert!(at.is_some_and(|a| (a - t).abs() <= 0.02 * t), "{name}: R{} (device {d}) burns at {at:?}, by hand {t} s", d - 1);
        }
    }
}

/// tpl=5 hub=2 e=2.338 psu ilim=0.097 C1-0=8.2e-6 C1-2=1.8e-5 C1-4=7.2e-3 C1-3=2.1e-6 C1-0=1.5e-7 F1-2=0.08/0.98 SW4-0(0)+R0-0=184 C0-1=1.2e-7
fn cold_474_tpl5() -> Program {
    let devices = vec![
        fz(21, 0, &[1, 0], vec![2.3375291345409845, 0.09678055859145739], vec![]),
        fz(1, 1, &[2, 0], vec![7180.2199110408665], vec![]),
        fz(1, 2, &[3, 0], vec![345814.27213022014], vec![]),
        fz(1, 3, &[4, 0], vec![19525.064293459738], vec![]),
        fz(3, 4, &[1, 0], vec![8.155629243426333e-6, 0.0, 0.0], vec![]),
        fz(3, 5, &[1, 2], vec![1.836345948900337e-5, 0.0, 0.0], vec![]),
        fz(1, 6, &[2, 4], vec![607.8206046545533], vec![]),
        fz(3, 7, &[1, 4], vec![0.007205932911217215, 0.0, 0.0], vec![]),
        fz(3, 8, &[1, 3], vec![2.144561582044065e-6, 0.0, 0.0], vec![]),
        fz(3, 9, &[1, 0], vec![1.5440391025338377e-7, 0.0, 0.0], vec![]),
        fz(10, 10, &[1, 2], vec![0.08054189918308306, 0.9788101989363357], vec![]),
        fz(4, 11, &[4, 0], vec![0.0], vec![]),
        fz(1, 12, &[0, 0], vec![183.51265257714326], vec![]),
        fz(3, 13, &[0, 1], vec![1.229388233457951e-7, 0.0, 0.0], vec![]),
        fz(90, 12, &[], vec![12.0, 0.00035864224151590314, 183.51265257714326], vec![]),
    ];
    let mut p = program(5, devices);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_stepped_steps_start_state_reads_its_outside_nodes_where_the_step_before_ended() {
    // Ruling R80 (a), cold 474 tpl 5: a 0.98 ohm fuse rated 80.5 mA from a
    // {2.34 V, 96.8 mA} PSU's rail to node 2, which feeds 7.2 kohm to ground
    // and 608 ohm into the 7.2 mF that hangs from the rail; 18 uF across the
    // fuse. By hand the fuse never carries more than 2.34 V over the two
    // paths' 560 ohm, 4.2 mA, 5 % of its rating: its rate is 0 throughout,
    // and the fine reference never stresses it. A step's start state read
    // the rail as published when the step committed, where other
    // partitions' rail solves had moved it since the step began: 0.26 V
    // across the fuse against node 2 as the step began, 0.27 A, and the
    // fuse tripped at 0.138 s. The start state is the step before's end.
    let stops: &[f64] = &[0.00627287950500977, 0.007757064900799632, 0.007881150628536771, 0.00816013172635659, 0.009035029419257412, 0.020888221258013227, 0.04482961454967979, 0.04550677873296601, 0.05217013656613313, 0.07120611375942745, 0.11266777803099516, 0.11885725268952782, 0.16072453444507465, 0.17863163009750702, 0.1794031621633488, 0.18253635560309564, 0.1830587183835147, 0.1906137107148383, 0.2];
    let mut e = Engine::load(&cold_474_tpl5()).unwrap_or_else(|er| panic!("load: {}", er.message));
    for &t in stops {
        e.advance(t).unwrap_or_else(|er| panic!("advance to {t}: {} {}", er.code, er.message));
    }
    let rec: Vec<(f64, f64)> = e.records(0).unwrap().chunks(7).map(|c| (c[0], c[3])).collect();
    assert!(!rec.iter().any(|r| r.0 == 10.0), "the fuse tripped: {rec:?}");
}

#[test]
fn a_stepped_restore_puts_back_each_partitions_end_reads() {
    // A step's start state is its partition's outside reads where the step
    // before ended (`Partition::end_reads`, ruling R80 (a)); a restored
    // snapshot must put them back with the rest, or the next step starts
    // from where the abandoned future left the rail (R80V). Cold 474 tpl 5:
    // its fuse's partition reads the rail.
    let mut rig = Rig::load(&cold_474_tpl5()).unwrap();
    rig.advance(0.0209).unwrap();
    let snap = rig.s().stepped.snapshot();
    let before = snap.digest();
    assert!(before.contains("end_reads: [("), "the fuse's partition has end reads to keep: {before}");
    rig.advance(0.05).unwrap();
    assert_ne!(rig.s().stepped.snapshot().digest(), before, "later steps moved the end reads");
    rig.k.solver_mut().stepped.restore(snap);
    assert_eq!(rig.s().stepped.snapshot().digest(), before);
}

#[test]
fn a_stepped_led_under_its_rating_carries_its_current_ratio() {
    // R80V: a stepped LED whose junctions all sit under the voltage where
    // they carry ifMax takes the fast path (rate 0) without computing its
    // peak current, and a step whose current crossed ifMax inside it read
    // that end at the level: accruing all of the step and bleeding none
    // (+0.820 for +0.116 on q from 3 to 0.5). Cold 535 tpl 2's LED R8 (ifMax
    // 6.85 mA) is in CV by 14 ms and its current falls under ifMax at 35 ms by
    // hand (I0 / (1 + I0 t / (C n vt)), I0 = 0.2746 A, C n vt = 1.474e-4): at
    // 0.1 s its drive carries (I / ifMax)^2 under 1, not the unknown -1.
    let stops = [0.00627288078824777, 0.016829457476524014, 0.058843896577463285, 0.058976488413401716, 0.059103567309001626, 0.05970527509700228, 0.07520090991509062, 0.07780697361405664, 0.08073770949696749, 0.08757943996501401, 0.09266788130346024, 0.09857249569377252, 0.1];
    let mut rig = Rig::load(&cold_535_tpl2()).unwrap();
    for &t in &stops {
        rig.advance(t).unwrap();
    }
    let drive = rig.s().slow.stress(8).expect("the LED's integrator").drive;
    let crate::slow::stress::StressDrive::Constant { rate, q, .. } = drive else { panic!("a constant drive: {drive:?}") };
    assert_eq!(rate, 0.0, "under its rating");
    assert!((0.0..1.0).contains(&q), "the drive carries its ratio: q = {q}");
}

#[test]
fn a_stepped_leds_cv_tail_at_long_host_steps_is_stepped_finely_enough_not_to_latch() {
    // Ruling R90 (a), cold 535 tpl 2: the LED's CV tail falls from 0.2746 A
    // like I0 / (1 + I0 t / (C n vt)), about 16-fold in its first 3 ms, and
    // adds 0.86 to S = 21.96 by hand, under the latch's 24. At host steps
    // of 50 ms and more (the app's 10x speed runs 1/6 s batches) the
    // stepped partition took one long step down that fall, whose mean of
    // the two ends' ratios over-counted it (q from 1604 to 97 at the ends)
    // and latched the LED at 19.45 ms. The step a stepped stressed device
    // may take is held to a ratio change of e^0.5 (`STRESS_STEP_RATIO`).
    for dt in [0.05, 0.1, 1.0 / 6.0] {
        let mut rig = Rig::load(&cold_535_tpl2()).unwrap();
        let mut t: f64 = 0.0;
        while t < 0.2 - 1e-12 {
            t = (t + dt).min(0.2);
            rig.advance(t).unwrap();
        }
        assert!(!rig.s().slow.is_failed(8), "host steps of {dt} s: the LED latched");
        let s = rig.s().slow.stress(8).map_or(0.0, |s| s.value_at(0.2));
        assert!((s - (22.8 - 2.0 * (0.2 - 0.0347))).abs() <= 0.05 * 22.8, "host steps of {dt} s: S at 0.2 s {s}, by hand 22.47");
    }
}


/// Cold 535 tpl 2 at host steps of 50 ms to 0.2 s, with the per-step state a
/// stress cut could disturb: the stepped partitions' digest (their `version`
/// counters aside), the published arrays, the LED's damage integral and the
/// latch log.
fn run_535_at_50ms(force: Option<(usize, f64, f64)>) -> Vec<String> {
    stepped::FORCE_REACH.with(|f| f.set(force));
    stepped::STRESS_CUT_LOG.with(|l| l.borrow_mut().clear());
    let mut rig = Rig::load(&cold_535_tpl2()).unwrap();
    let mut t: f64 = 0.0;
    let mut states = Vec::new();
    while t < 0.2 - 1e-12 {
        t = (t + 0.05).min(0.2);
        rig.advance(t).unwrap();
        let digest = rig.s().stepped.snapshot().digest();
        let digest: String = digest.split("version: ").enumerate().map(|(k, part)| if k == 0 { part.to_string() } else { part.trim_start_matches(|c: char| c.is_ascii_digit()).to_string() }).collect();
        let v: Vec<u64> = rig.s().voltages.iter().map(|x| x.to_bits()).collect();
        let c: Vec<u64> = rig.s().currents.iter().map(|x| x.to_bits()).collect();
        let s8 = rig.s().slow.stress(8).map(|s| s.value_at(t).to_bits());
        states.push(format!("{t} {digest} {v:?} {c:?} {s8:?} {}", rig.s().slow.is_failed(8)));
    }
    stepped::FORCE_REACH.with(|f| f.set(None));
    states
}

/// The stress cuts logged so far, grouped by the step's start: each start's
/// cut lengths in order.
fn cuts_by_start() -> Vec<(f64, usize, Vec<f64>)> {
    let log = stepped::STRESS_CUT_LOG.with(|l| l.borrow().clone());
    let mut out: Vec<(f64, usize, Vec<f64>)> = Vec::new();
    for (p, t, _, cut) in log {
        match out.iter_mut().find(|e| e.0 == t && e.1 == p) {
            Some(e) => e.2.push(cut),
            None => out.push((t, p, vec![cut])),
        }
    }
    out
}

#[test]
fn a_stress_cut_leaves_the_state_a_plan_at_its_reach_would_have() {
    // Ruling R90 (a): a trial cut for a stressed device's ratio
    // (`plan_step`'s stress probe) must leave catch-up, readers, watches and
    // every later step as if the shorter step had been planned in the first
    // place. Cold 535 tpl 2 at 50 ms host steps cuts its LED's partition at
    // the CC exit (13.625 ms) and after; at the first later commit whose
    // cuts settle the trial under e^0.5 (not at the cap), a second run plans
    // that commit at the length the cuts ended with, and must match the
    // first state for state to 0.2 s.
    let cut = run_535_at_50ms(None);
    let starts = cuts_by_start();
    let &(t, p, ref cuts) = starts.iter().skip(1).find(|e| e.2.len() < 4).expect("a later commit whose cuts settle under the ratio");
    let forced = run_535_at_50ms(Some((p, t, *cuts.last().unwrap())));
    assert!(!cuts_by_start().iter().any(|e| e.0 == t), "planned at that length, the trial needed no cut");
    assert_eq!(cut, forced);
}

#[test]
fn a_stress_cut_stops_at_its_cap_on_a_ratio_that_keeps_falling() {
    // Cold 84 tpl 1 (exact mode) at the clock's first fall: the LED drains
    // the rail capacitor on tau = 37 ns from 790 A, and the drain's steps
    // ring, so a trial cut in proportion to its ratio's change still changes
    // by more than e^0.5: a plan cuts `STRESS_CUTS` (4) times, each shorter,
    // and goes on; none cuts more, and the run gets past the fall.
    stepped::STRESS_CUT_LOG.with(|l| l.borrow_mut().clear());
    let mut rig = Rig::load(&exact(ref_cold_84_tpl1())).unwrap();
    let fall = 0.6067396404390497 / 292.4270639400117;
    rig.advance(fall + 2e-6).unwrap();
    let starts = cuts_by_start();
    assert!(starts.iter().any(|e| e.2.len() == 4), "a plan cuts to the cap: {starts:?}");
    assert!(starts.iter().all(|e| e.2.len() <= 4 && e.2.windows(2).all(|w| w[1] < w[0])), "{starts:?}");
}




#[test]
fn the_stepped_stress_probe_reads_each_kinds_ratio_as_its_drives_do() {
    // Ruling R90 (a): the planner's probe and the slow pass's drives read a
    // stepped device's stress ratio through the same formulas
    // (`failures::resistor_q`, `fuse_q`, `led_q`, `PtcParams::
    // untripped_constant`); at the published voltages they agree for each
    // kind: resistors (cold 104 tpl 6), an LED (535 tpl 2), a PTC (55 tpl
    // 2) and a fuse (474 tpl 5).
    let mut p55 = program(6, cold_55_tpl2_dev());
    p55.digital_slot_count = 32;
    let boards: [(&str, Program, f64); 4] = [("104/6", cold_104_tpl6(), 0.01), ("535/2", cold_535_tpl2(), 0.005), ("55/2", p55, 0.0005), ("474/5", cold_474_tpl5(), 0.03)];
    let mut kinds_seen = std::collections::BTreeSet::new();
    for (name, p, t) in boards {
        let mut rig = Rig::load(&p).unwrap();
        rig.advance(t).unwrap();
        let rows = rig.s().probe_agreement(t);
        assert!(!rows.is_empty(), "{name}: rated stepped devices");
        for (kind, q, pq) in rows {
            kinds_seen.insert(kind);
            assert!((q - pq).abs() <= 1e-12 * q.abs().max(1.0), "{name}: kind {kind} drives q {q}, probe {pq}");
        }
    }
    for kind in [kinds::RESISTOR, kinds::FUSE, kinds::LED, kinds::PTC_FUSE] {
        assert!(kinds_seen.contains(&kind), "kind {kind} checked: {kinds_seen:?}");
    }
}

/// A 0.2 s ramp from 0 to 6 V into an LED (vf 2 V, ifMax 20 mA) charging
/// 1 mF: dark until the ramp passes the LED's knee, then carrying C dV/dt =
/// 30 mA, past its rating, from a partition nothing watched.
fn ramp_fed_led() -> Program {
    let mut p = program(3, vec![
        fz(kinds::PULSE_SOURCE, 0, &[1, 0], vec![0.0, 6.0, 0.0, 0.2, 0.2, 10.0, 20.0], vec![0.0]),
        fz(kinds::LED, 1, &[1, 2], vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0], vec![]),
        fz(kinds::CAPACITOR, 2, &[2, 0], vec![1e-3, 0.0, 0.0], vec![]),
        fz(kinds::RESISTOR, 3, &[2, 0], vec![1e4], vec![]),
    ]);
    p.digital_slot_count = 32;
    p
}

#[test]
fn a_device_going_stressed_from_an_unwatched_partition_is_cut_on_the_step_that_takes_it() {
    // Ruling R90 (a), R80V2 A: the ramp-fed LED's partition carries no
    // stressed device until the ramp passes the LED's knee; each trial is
    // read from its own end (`plan_step_cut`), whatever the last commit
    // watched, and at 10 ms host steps the LED's damage tracks a run at
    // 0.1 ms within 3 %.
    let s_at = |dt: f64| {
        stepped::STRESS_CUT_LOG.with(|l| l.borrow_mut().clear());
        let mut rig = Rig::load(&ramp_fed_led()).unwrap();
        let mut t: f64 = 0.0;
        while t < 0.2 - 1e-12 {
            t = (t + dt).min(0.2);
            rig.advance(t).unwrap();
        }
        let cuts = stepped::STRESS_CUT_LOG.with(|l| l.borrow().len());
        (rig.s().slow.stress(1).map_or(0.0, |s| s.value_at(0.2)), rig.s().slow.is_failed(1), cuts)
    };
    let (coarse, failed_c, _) = s_at(0.01);
    let (fine, failed_f, _) = s_at(1e-4);
    assert_eq!(failed_c, failed_f);
    assert!(fine > 0.0 && (coarse - fine).abs() <= 0.03 * fine, "S at 0.2 s: {coarse} at 10 ms hosts, {fine} at 0.1 ms");
}

#[test]
fn exact_modes_deferred_rail_plan_is_cut_like_any_other() {
    // Ruling R90 (a), R80V2 C and D: exact mode plans its rail partition
    // only once the owner has charged the step's SoC (`plan_deferred`),
    // after every other path; the stress probe sits in `plan_step` itself,
    // so cold 84 tpl 1's drain (its LED in the stepped rail partition) is
    // cut there, the first step after the fall's re-seed read from the
    // seed. That first step starts at the shortest step, under any cut.
    let fall = 0.6067396404390497 / 292.4270639400117;
    stepped::STRESS_CUT_PATHS.with(|l| l.borrow_mut().clear());
    let mut rig = Rig::load(&exact(ref_cold_84_tpl1())).unwrap();
    rig.advance(fall + 2e-6).unwrap();
    let paths = stepped::STRESS_CUT_PATHS.with(|l| l.borrow().clone());
    assert!(!paths.is_empty() && paths.iter().all(|c| c.2), "the rail partition's deferred plans are cut: {paths:?}");
}





