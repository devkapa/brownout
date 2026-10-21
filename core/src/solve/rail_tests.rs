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

#[test]
fn an_ic_whose_own_draw_sags_the_rail_back_across_its_power_window_is_minus_4_today() {
    // Known divergence `w1f-power-gate-pingpong` (Equivalence harness rules,
    // W1g's list): E 4.279, r 5, a 74LS00 on the rail with its inputs on the
    // datum, so every output drives HIGH, and 1y into 1 kohm. Unpowered it
    // draws nothing and the rail is E, above 0.9 * 4.75 = 4.275 V; powered,
    // 1y's draw (about 3.8 mA through R_out and 1 kohm) sags the rail to
    // about 4.26 V, below it. No power state is consistent, so the rail loop
    // alternates until its 50 passes and the load fails with -4 (the TS side
    // falls back to classic, which resolves it with H8's rule). This pins
    // today's result until that rule is ported into the rail loop.
    let p = program(6, vec![battery(0, 1, 0, 4.279, 5.0), ls00(1, 1, 0, [0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0, 5], 10e-9, 0), resistor(2, 2, 0, 1000.0)]);
    let e = Engine::load(&p).err().expect("no consistent power state: the load fails");
    assert_eq!(e.code, crate::error::ERR_SOLVE_FAILED, "{}", e.message);
    assert!(e.message.contains(&format!("the rail loop did not settle within {} passes", rails::RAIL_LOOP_MAX)), "{}", e.message);
    // The control: with 1y unloaded the gate holds its supply, and loads.
    let p = program(6, vec![battery(0, 1, 0, 4.279, 5.0), ls00(1, 1, 0, [0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0, 5], 10e-9, 0), resistor(2, 2, 0, 1e9)]);
    let mut e = Engine::load(&p).unwrap();
    e.advance(1e-6).unwrap();
    assert!(e.voltages()[2] > 4.0, "1y HIGH: {}", e.voltages()[2]);
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

#[test]
fn a_cc_ramp_whose_ic_load_turns_the_rail_back_under_its_floor_is_minus_4_today() {
    // Known divergence `w1f-power-gate-pingpong` again, across instants: with
    // a rail capacitor the rail cannot jump, so each power flip turns the
    // segment's slope (+1,818 V/s unpowered, -3,241 V/s powered: the LED's
    // 5.06 mA against the 1.82 mA the rail capacitor gets), and the next
    // power-window event lands the rail two reached widths (8.55e-12 V) past
    // the floor the other way, 5.3 and 9.4 fs later. Without the count below
    // that alternated from event 2,081 on, at 1.4421620096 ms, every event a
    // `Rail` event, 15 fs a pair: 3 ms lay about 2e11 events away, a hang in
    // the browser's worker. Now it fails as the capacitor-less ping-pong does.
    // Up to the floor the gate draws nothing and the board is linear: the
    // rail reaches 4.275 V at 1.4421592086 ms (the exact solution of its two
    // capacitors from 0 V). Since round 14 the rail and the 100 ohm RC
    // advance on their coupled path (ruling R15 (a)), so the crossing lands
    // there; with the RC's draw frozen between solves it landed 2.8 ns late
    // (R13's drift width put it there at about 26,700 events).
    let p = cc_ramp_into_an_ic_floor(1e-6, 0.02, led(2, 2, 3));
    let (err, n, t, _) = run_events(&p, 3e-3, 50_000);
    let err = err.expect("the ping-pong fails");
    assert_eq!(err.code, crate::error::ERR_SOLVE_FAILED, "{}", err.message);
    assert!(err.message.contains(&format!("the rail loop did not settle within {} passes", rails::RAIL_LOOP_MAX)), "{}", err.message);
    close(t, 1.4421592086e-3, 1e-9, "the floor crossing");
    eprintln!("ping-pong on a CC ramp: -4 after {n} events at {t:.12e}: {}", err.message);
    // One advance, as a frame asks for it, ends the same way.
    let mut e = Engine::load(&p).unwrap();
    let err = e.advance(3e-3).expect_err("the ping-pong fails");
    assert_eq!(err.code, crate::error::ERR_SOLVE_FAILED, "{}", err.message);
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
fn a_zero_delay_reader_of_a_ping_ponging_gate_is_minus_4_not_a_hang() {
    // The ping-pong above, but each power flip swings 1y across the second
    // gate's threshold, and with no delay its output commits in that same
    // instant. That re-solve ends the gate-flip count (end_gate_chase, which
    // a clocked load needs), so on 076be49 the Rail events alternated 5.3
    // and 9.4 fs apart from 1.442162 ms on, past any event cap, and one
    // advance to 3 ms never returned. The no-progress guard ends it with the
    // ping-pong's own -4, at the exact floor crossing (the test above), within
    // the event count R13's drift width took to reach it (26,700).
    let p = cc_floor_with_a_reader(0.0, 0.02);
    let (err, n, t, _) = run_events(&p, 3e-3, 26_700 + crate::kernel::NO_PROGRESS_MAX + 10);
    let err = err.expect("the ping-pong fails");
    assert_eq!(err.code, crate::error::ERR_SOLVE_FAILED, "{}", err.message);
    assert_eq!(err.message, rails::loop_unsettled());
    close(t, 1.4421592086e-3, 1e-9, "the floor crossing");
    eprintln!("zero-delay reader: -4 after {n} events at {t:.12e}");
    let mut e = Engine::load(&p).unwrap();
    let err = e.advance(3e-3).expect_err("one advance ends the same way");
    assert_eq!(err.message, rails::loop_unsettled());
    close(e.time(), 1.4421592086e-3, 1e-9, "time stays at the last completed instant");
    // A second advance declines again rather than hanging or resuming.
    let err = e.advance(3e-3).expect_err("still stuck");
    assert_eq!(err.message, rails::loop_unsettled());
    // The controls: with the reader's own 10 ns delay the flips cancel its
    // transitions, nothing commits, and HG1's count declines; at 60 mA the
    // rail outruns the LED and the run completes.
    let (err, _, _, _) = run_events(&cc_floor_with_a_reader(10e-9, 0.02), 3e-3, 50_000);
    assert_eq!(err.expect("HG1's count").message, rails::loop_unsettled());
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
fn cc_led_into_a_big_capacitor() -> Program {
    program(3, vec![psu(0, 1, 0, 9.362, 0.3), led(1, 1, 2), capacitor(2, 2, 0, 29e-3, 0.0), resistor(3, 2, 0, 1e4)])
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
    let steps = e.stat(10);
    assert!(steps < 2000.0, "{steps} steps to 0.3 s");
    eprintln!("CC LED into 29 mF: {steps} steps to 0.3 s (exact mode {}), u {:.9} V, capacitor {:.9} V", ex.stat(10), e.voltages()[1], e.voltages()[2]);
}

/// G2bv's seed 396 (FXF): an 11.919 V cell of 0.872 ohm with 7.04 uF from
/// the rail into a stepped partition of LEDs, and a switch (device 12) that
/// closes onto a node one LED's drop below the rail at 0.128 s.
fn cell_switch_into_an_led_loop() -> Program {
    let led_with = |ci: usize, a: usize, b: usize, vf: f64, n: f64, i_rated: f64| Device {
        kind: kinds::LED,
        component_index: ci,
        pins: vec![pin(a), pin(b)],
        params: vec![vf, n, i_rated, 0.0, 0.0, 0.0],
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
    // controller chose (stepped::SteppedSolver::reaim).
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
        // committed, a first-order coupling error: 5.1x W0 from exact mode
        // at worst (the second capacitor at 0.1 s), 6.4x off the reference.
        for n in 1..7 {
            let (got, x) = (e.voltages()[n], ex.voltages()[n]);
            close(got, x, 6.0 * (1e-4 + 1e-3 * x.abs()), &format!("node {n} at {t} against exact mode"));
        }
    }
    let steps = e.stat(10);
    assert!(steps < 5000.0, "{steps} steps to 0.1 s (exact mode {})", ex.stat(10));
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
    // A resistor has no device state (W1g adds its thermal layout).
    assert_eq!(read.device_state(1).err().map(|e| e.code), Some(crate::error::ERR_INVALID_ARGUMENT));
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
fn the_power_gate_pingpong_board_through_5_ohm_still_declines() {
    // The original board of `a_power_flip_inside_one_band_never_replays_the_
    // other_power_state` (a 4.06 V cell with 5 ohm, 1 kohm into 1 uF, a gate
    // whose 4.05 V power threshold the rail crosses inside one cache band),
    // kept here so the change stays visible: it passed on 076be49 only via a
    // cache hit, and with the cache on or off the rail now ping-pongs at the
    // threshold (the powered gate's own draw through 5 ohm pulls the rail
    // back under it) and the run declines with -4. This is
    // w1f-power-gate-pingpong, F5's class (the power-gate hold channel): F5
    // must flip this assertion to a run that reaches 10 ms powered with its
    // outputs high.
    let mut gate = ls00(3, 1, 0, [0, 0, 4, 0, 0, 5, 0, 0, 6, 0, 0, 7], 10e-9, 0);
    gate.params[ic_param::VCC_MIN] = 4.5;
    let (e, r, big_r, c) = (4.06, 5.0, 1000.0, 1e-6);
    for cache in [true, false] {
        let mut p = program(8, vec![battery(0, 1, 0, e, r), resistor(1, 1, 2, big_r), capacitor(2, 2, 0, c, 0.0), gate.clone()]);
        p.options.no_solution_cache = !cache;
        let mut rig = Rig::load(&p).unwrap();
        let err = rig.advance(10e-3).expect_err("the power gate ping-pongs at its threshold (F5)");
        assert!(matches!(&err, KernelError::SolveFailed(m) if *m == rails::loop_unsettled()), "cache {cache}: {err:?}");
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

fn fx_exact(b: &FxBoard, u: f64) -> FxExact {
    let n = b.nodes.len();
    let at = |node: usize| b.nodes.iter().position(|&m| m == node);
    let powered = u >= b.win.0 * 0.9 && u <= b.win.1 * 1.1;
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
        for (pi, (part, idx, local, els)) in parts.iter().enumerate() {
            let Some((x, _, i_p, g_p, u_p)) = rig.s().static_solver.hub_solve(*part) else { continue };
            if powered(u) != powered(u_p) {
                continue;
            }
            if u_p.to_bits() != last_up[pi].to_bits() {
                last_up[pi] = u_p;
                // The solve itself against the exact solution at its rail.
                let e = fx_exact(b, u_p);
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
            let e = fx_exact(b, u);
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
                if off[0] <= floor + 1e-6 || off[0] >= top - 1e-6 {
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
