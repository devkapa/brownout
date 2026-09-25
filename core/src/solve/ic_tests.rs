//! W1c step 2 integration tests: the solver/kernel side of the digital core,
//! exercised end to end through a REAL [`PartitionedSolver`] and a REAL
//! [`Kernel`], with a small test-only [`DeviceEvaluator`] ([`IcHarness`])
//! that implements an IC honestly from the step-1 functions
//! (`digital::levels::read`, `digital::combinational::evaluate`,
//! `digital::delay::driver_requests`) — never a mock of the digital core
//! itself, only of the wiring `engine.rs` will eventually do for a real
//! program (out of this step's scope; see the worker brief).
//!
//! Every `Program`/`Device` here is built by hand (never through
//! `program.rs`'s byte-format reader, which this step does not touch), so
//! `kinds::shape`'s own `check` never runs against them — the shapes below
//! must already match `kinds.rs`'s IC table exactly (pin/param/blob counts),
//! the same discipline `digital::mod`'s own `#[cfg(test)]` fixtures follow.

use super::{PartitionSolver, PartitionedSolver, Threshold};
use crate::digital::{self, IcPin, IcRecord};
use crate::drivers::Driver;
use crate::engine::Engine;
use crate::events::{DeviceId, DriverId, Mode, PartitionId};
use crate::kernel::{DeviceEvaluator, DriverRequest, Kernel, KernelError};
use crate::kinds::{self, ic_param};
use crate::program::{Device, Options, PinRecord, Program, PIN_FLAG_OPEN};

// ============================================================================
// Program-building helpers.
// ============================================================================

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn open_pin(node: usize, open_hash: u32) -> PinRecord {
    PinRecord { node, flags: PIN_FLAG_OPEN, open_hash, dead_hash: 0 }
}

fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
    Device { kind: kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
}

fn capacitor(ci: usize, a: usize, b: usize, c: f64, esr: f64, leak: f64) -> Device {
    Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![c, esr, leak], blob: vec![] }
}

fn switch_dev(ci: usize, a: usize, b: usize, closed: f64) -> Device {
    Device { kind: kinds::SWITCH, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![closed], blob: vec![] }
}

fn spdt(ci: usize, com: usize, b: usize, c: usize, sel: f64) -> Device {
    Device { kind: kinds::SPDT_SWITCH, component_index: ci, pins: vec![pin(com), pin(b), pin(c)], params: vec![sel], blob: vec![] }
}

fn program(node_count: usize, devices: Vec<Device>) -> Program {
    Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
}

/// A from-scratch FNV-1a, used only to build the SAME `openHash`/`deadHash`
/// prefix the compiler would ship (`digital::floating`'s own module doc:
/// "the FNV-1a state already advanced over the id/pin prefix"), so a test
/// that names a real component/pin id exercises the real hash, not a
/// hand-picked number — mirrors `digital::floating`'s own private test
/// helper of the same name exactly (that one is not `pub`, so it cannot be
/// reused from here).
fn fnv1a(s: &str) -> u32 {
    s.bytes().fold(0x811c_9dc5u32, |h, b| (h ^ b as u32).wrapping_mul(0x0100_0193))
}

/// Standard LS-TTL IC params (core/README.md's "IC params"): fixed
/// `vil`/`vih`, no Schmitt, `nominal = 5`. `io_max`/`delay` are the two
/// figures that actually vary kind to kind (README's own r_out/delay
/// examples), so every caller supplies them.
fn ls_ttl_params(io_max: f64, delay: f64) -> Vec<f64> {
    let mut p = vec![0.0; ic_param::COUNT];
    p[ic_param::VCC_MIN] = 4.75;
    p[ic_param::VCC_MAX] = 5.25;
    p[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    p[ic_param::VIL_FIXED] = 1.0;
    p[ic_param::VIL] = 0.8;
    p[ic_param::VIH_FIXED] = 1.0;
    p[ic_param::VIH] = 2.0;
    p[ic_param::NOMINAL] = 5.0;
    p[ic_param::IO_MAX] = io_max;
    p[ic_param::DELAY] = delay;
    p
}

/// A `74ls04` (hex inverter) device: `pins[i]` names the node for logic pin
/// `i` (`icPinMap` order `1a 1y 2a 2y 3a 3y 4a 4y 5a 5y 6a 6y`), `open[i]`
/// whether it carries `PIN_FLAG_OPEN`, `open_hash[i]` its open-key prefix
/// (only consulted when `open[i]`). io_max 0.008 -> R_out 156.25 ohm at 5V
/// (`digital::power`'s own `r_out_at_5v` test), matching the README's own
/// LS-TTL gate figure.
#[allow(clippy::too_many_arguments)]
fn ls04(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12], open: [bool; 12], open_hash: [u32; 12], io_max: f64, delay: f64) -> Device {
    let mut device_pins = vec![pin(vcc), pin(gnd)];
    for i in 0..12 {
        device_pins.push(if open[i] { open_pin(pins[i], open_hash[i]) } else { pin(pins[i]) });
    }
    // `1a 1y 2a 2y ...`: even indices are inputs (FLOATING_INPUT), odd are
    // outputs (no flag).
    let blob_flags: Vec<f64> = (0..12).map(|i| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }).collect();
    let mut blob = vec![0.0];
    blob.extend(blob_flags);
    Device { kind: kinds::IC_74LS04, component_index: ci, pins: device_pins, params: ls_ttl_params(io_max, delay), blob }
}

/// A `74ls00` (quad 2-input NAND) device: `pins[i]` names logic pin `i`
/// (`1a 1b 1y 2a 2b 2y 3a 3b 3y 4a 4b 4y`).
#[allow(clippy::too_many_arguments)]
fn ls00(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12], open: [bool; 12], open_hash: [u32; 12], io_max: f64, delay: f64) -> Device {
    let mut device_pins = vec![pin(vcc), pin(gnd)];
    for i in 0..12 {
        device_pins.push(if open[i] { open_pin(pins[i], open_hash[i]) } else { pin(pins[i]) });
    }
    // `1a 1b 1y 2a 2b 2y ...`: pins at position (i % 3 != 2) are inputs.
    let blob_flags: Vec<f64> = (0..12).map(|i| if i % 3 != 2 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }).collect();
    let mut blob = vec![0.0];
    blob.extend(blob_flags);
    Device { kind: kinds::IC_74LS00, component_index: ci, pins: device_pins, params: ls_ttl_params(io_max, delay), blob }
}

/// HC-CMOS-ratio Schmitt IC params (core/README.md's "IC params"): the
/// `74hc14`'s own catalog figures (spec/w1-datasheet-delays.md, and the
/// worked example in "Kernel semantics: the digital core (W1c)", point 4:
/// "Schmitt thresholds at 5V are 1.6/2.7 V"). `vtPlus`/`vtMinus` ship
/// UNSCALED (at `nominal`), exactly as `digital::power::ic_power` expects to
/// scale them itself by `s = vS/nominal`.
fn hc14_params(io_max: f64, delay: f64) -> Vec<f64> {
    let mut p = vec![0.0; ic_param::COUNT];
    p[ic_param::VCC_MIN] = 2.0;
    p[ic_param::VCC_MAX] = 6.0;
    p[ic_param::FAM_FLAGS] = kinds::FAM_CMOS_RATIO as f64;
    p[ic_param::SCHMITT] = 1.0;
    p[ic_param::VT_PLUS] = 2.7;
    p[ic_param::VT_MINUS] = 1.6;
    p[ic_param::NOMINAL] = 5.0;
    p[ic_param::IO_MAX] = io_max;
    p[ic_param::DELAY] = delay;
    p
}

/// A `74hc14` (Schmitt hex inverter) device: same `1a 1y 2a 2y ...` pin
/// layout as `ls04`. `io_max = 0.02` gives R_out = 62.5 ohm at 5V
/// (`digital::power`'s own `r_out_at_5v` golden value, matching the README's
/// worked example exactly).
#[allow(clippy::too_many_arguments)]
fn hc14(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12], open: [bool; 12], open_hash: [u32; 12], io_max: f64, delay: f64) -> Device {
    let mut device_pins = vec![pin(vcc), pin(gnd)];
    for i in 0..12 {
        device_pins.push(if open[i] { open_pin(pins[i], open_hash[i]) } else { pin(pins[i]) });
    }
    let blob_flags: Vec<f64> = (0..12).map(|i| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }).collect();
    let mut blob = vec![0.0];
    blob.extend(blob_flags);
    Device { kind: kinds::IC_74HC14, component_index: ci, pins: device_pins, params: hc14_params(io_max, delay), blob }
}

// ============================================================================
// IcHarness: a test-only DeviceEvaluator built honestly from the step-1
// functions, never a mock of the digital core itself.
// ============================================================================

struct IcEntry {
    device_index: usize,
    record: IcRecord,
    /// Driver ids for each `combinational::output_capable(kind)` pin, in
    /// that same order — `digital::delay::driver_requests`'s own contract.
    output_drivers: Vec<DriverId>,
    /// One Schmitt-hysteresis slot per logic pin (`digital::levels::read`'s
    /// `schmitt_state`); unused (never written) by a non-Schmitt kind.
    schmitt_state: Vec<bool>,
}

struct IcHarness {
    ics: Vec<IcEntry>,
}

impl IcHarness {
    fn new(ics: Vec<IcEntry>) -> Self {
        IcHarness { ics }
    }
}

impl DeviceEvaluator for IcHarness {
    /// Contract point 4: "An unpowered IC reads nothing" — this simply
    /// never calls `digital::levels::read` at all in that case, matching
    /// `digital::power::ic_power(..).powered` exactly, the same gate a real
    /// production evaluator would need. Every logic pin is read every call
    /// (§1.4's own reader shape: "an IC reads its supply nodes and every
    /// logic pin's node"), fed straight into `combinational::evaluate` and
    /// `delay::driver_requests` — nothing here re-derives what those
    /// functions already decide.
    fn evaluate(&mut self, reader: DeviceId, now: f64, _acting_on_edges: bool, voltages: &[f64]) -> Vec<DriverRequest> {
        let entry = self.ics.iter_mut().find(|e| e.device_index == reader.0 as usize).unwrap_or_else(|| panic!("IcHarness: no IC registered for reader {reader:?}"));
        let ic = &entry.record;
        let power = digital::power::ic_power(ic, voltages[ic.vcc.node], voltages[ic.gnd.node]);
        if !power.powered {
            return Vec::new();
        }
        let levels: Vec<bool> = ic
            .logic
            .iter()
            .zip(entry.schmitt_state.iter_mut())
            .map(|(p, st)| digital::levels::read(ic, &power, p, voltages[p.node], now, st))
            .collect();
        let result = digital::combinational::evaluate(ic.kind, &levels);
        digital::delay::driver_requests(ic, &entry.output_drivers, &result)
    }
}

/// Registers every combinational IC device in `program` against `kernel`:
/// creates a [`Driver`] for each of its `output_capable` pins (via
/// `PartitionSolver::partition_of_node`, so a driver on a fixed node
/// correctly gets `None` — contract point 4), and returns the
/// [`IcEntry`] list [`IcHarness::new`] expects. Called AFTER
/// `Kernel::new` (drivers need a live kernel to assign ids) but BEFORE the
/// harness itself is installed via `Kernel::set_evaluator` — `DriverId`s are
/// stable once assigned (`Kernel::add_driver` never renumbers), so building
/// the real evaluator only after every driver exists is safe and simpler
/// than threading ids back in afterward.
fn register_ics(kernel: &mut Kernel<PartitionedSolver, IcHarness>, program: &Program) -> Vec<IcEntry> {
    let mut entries = Vec::new();
    for (device_index, device) in program.devices.iter().enumerate() {
        if !digital::is_combinational_ic(device.kind) {
            continue;
        }
        let ic = IcRecord::from_device(device);
        let mut output_drivers = Vec::new();
        for &i in digital::combinational::output_capable(ic.kind) {
            let p: &IcPin = &ic.logic[i];
            let partition = kernel.solver().partition_of_node(p.node);
            let driver = Driver::new_ic_output(p.node, ic.vcc.node, ic.gnd.node, device_index, p.open_collector);
            output_drivers.push(kernel.add_driver(driver, partition));
        }
        let schmitt_state = vec![false; ic.logic.len()];
        entries.push(IcEntry { device_index, record: ic, output_drivers, schmitt_state });
    }
    entries
}

/// Builds the solver, kernel and harness for `program`, registers every IC,
/// and returns `(kernel, all_partitions, all_readers)` ready for
/// `Kernel::settle`.
fn build(program: &Program) -> (Kernel<PartitionedSolver, IcHarness>, Vec<PartitionId>, Vec<DeviceId>) {
    let solver = PartitionedSolver::build(program).expect("program must load");
    let mut kernel = Kernel::new(solver, IcHarness::new(Vec::new()));
    let entries = register_ics(&mut kernel, program);
    let all_readers: Vec<DeviceId> = entries.iter().map(|e| DeviceId(e.device_index as u32)).collect();
    let all_partitions: Vec<PartitionId> = (0..kernel.solver().partition_count() as u32).map(PartitionId).collect();
    kernel.set_evaluator(IcHarness::new(entries));
    (kernel, all_partitions, all_readers)
}

fn driver_for(kernel: &Kernel<PartitionedSolver, IcHarness>, program: &Program, device_index: usize, output_logic_index: usize) -> DriverId {
    // Recomputed the same way `register_ics` assigned them: drivers are
    // added in device order, and within a device in `output_capable` order
    // — so the Nth IC's Kth output driver is the (sum of every earlier IC's
    // output count + K)th driver added overall.
    let mut id = 0u32;
    for (di, device) in program.devices.iter().enumerate() {
        if !digital::is_combinational_ic(device.kind) {
            continue;
        }
        let kind = device.kind;
        let out = digital::combinational::output_capable(kind);
        if di == device_index {
            let pos = out.iter().position(|&i| i == output_logic_index).expect("output_logic_index must be output-capable");
            return DriverId(id + pos as u32);
        }
        id += out.len() as u32;
    }
    let _ = kernel;
    panic!("device {device_index} is not a registered IC in this program");
}

// ============================================================================
// (a) a 74ls04 on fixed 5V rails.
// ============================================================================

#[test]
fn a1_74ls04_input_pulled_low_through_1k_gives_a_high_output_into_a_10k_load_divider() {
    // 0 = gnd, 1 = vcc (5V, fixed), 2 = 1a (through 1k to gnd), 3 = 1y
    // (through 10k to gnd — the load). Every other gate's input ties
    // straight to gnd (deterministic LOW, unused); its output gets its own
    // isolated node.
    let p = program(9, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 2, 0, 1_000.0),
        resistor(2, 3, 0, 10_000.0),
        ls04(3, 1, 0, [2, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 3, 1); // logic index 1 = "1y".
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "input pulled to 0V (LOW) must invert to a HIGH output");

    // node2 (1a) reads exactly 0V: only a 1k resistor to the fixed datum.
    assert_eq!(kernel.solver().voltages()[2], 0.0);

    // The load divider: R_out = clamp(5/(4*0.008), 50, 1000) = 156.25 ohm
    // (digital::power's own r_out_at_5v golden value), so
    // v(1y) = 5 * 10000/(10000+156.25) exactly, up to the universal 1e-12 S
    // shunt's negligible contribution.
    let expected = 5.0 * 10_000.0 / (10_000.0 + 156.25);
    assert!((kernel.solver().voltages()[3] - expected).abs() < 1e-6, "got {}, expected {expected}", kernel.solver().voltages()[3]);
}

#[test]
fn a2_74ls04_input_tied_to_vcc_gives_a_low_output() {
    // 1a tied straight to the vcc node itself: reads 5V, well above vih.
    let p = program(9, vec![
        source(0, 1, 0, 5.0),
        ls04(1, 1, 0, [1, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 1, 1);
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "input tied HIGH must invert to a LOW output");
}

// ============================================================================
// (b) two inverters in a chain: a switch toggle on the first input commits
// at +20ns and +40ns.
// ============================================================================

#[test]
fn b_two_chained_inverters_commit_at_plus_20ns_and_plus_40ns() {
    // 0 = gnd, 1 = vcc (5V fixed), 2 = 1a (through an spdt: b=gnd, c=vcc),
    // 3 = 1y = 2a (shared node — gate1 feeds gate2 directly), 4 = 2y.
    let p = program(9, vec![
        source(0, 1, 0, 5.0),
        spdt(1, 2, 0, 1, 0.0), // sel=0 selects b=gnd: 1a starts LOW.
        ls04(2, 1, 0, [2, 3, 3, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y1 = driver_for(&kernel, &p, 2, 1); // 1y
    let y2 = driver_for(&kernel, &p, 2, 3); // 2y
    assert_eq!(kernel.driver(y1).committed(), Some(Mode::High), "1a LOW -> 1y HIGH at settle");
    assert_eq!(kernel.driver(y2).committed(), Some(Mode::Low), "2a (=1y) HIGH -> 2y LOW at settle");

    let t0 = kernel.time();
    kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap(); // sel=1 selects c=vcc: 1a -> HIGH.

    // Nothing commits AT t0: the request is a real (20ns), not zero-delay,
    // transition.
    assert_eq!(kernel.driver(y1).committed(), Some(Mode::High), "1y's commit is still in flight, not yet due");

    kernel.advance(t0 + 20e-9).unwrap();
    assert_eq!(kernel.driver(y1).committed(), Some(Mode::Low), "1y commits at +20ns");
    assert_eq!(kernel.driver(y2).committed(), Some(Mode::Low), "2y has not fired yet: its own +20ns has not elapsed");

    kernel.advance(t0 + 40e-9).unwrap();
    assert_eq!(kernel.driver(y2).committed(), Some(Mode::High), "2y commits at +40ns, 20ns after 1y");
}

// ============================================================================
// (c) an open 74ls00 input reads classic's golden verdict.
// ============================================================================

#[test]
fn c_open_74ls00_input_reads_the_golden_verdict_and_nands_with_a_driven_low_input() {
    // u1:1b in bucket 0 is HIGH for TTL (digital::floating's own golden
    // vector). 1a is driven LOW (tied to gnd); 1b is open, floating_input,
    // with the real "u1:1b:" prefix.
    let prefix = fnv1a("u1:1b:");
    let p = program(9, vec![
        source(0, 1, 0, 5.0),
        ls00(1, 1, 0, [0, 2, 3, 0, 0, 4, 0, 0, 5, 0, 0, 6], [false, true, false, false, false, false, false, false, false, false, false, false], [0, prefix, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 1, 2); // 1y
    // 1a=LOW(0), 1b=HIGH(1, from the golden hash) -> NAND(0,1) = 1 = HIGH.
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "NAND(1a=0, 1b=golden-HIGH) = HIGH");
}

// ============================================================================
// (d) a power gate: the IC's vcc through a switch.
// ============================================================================

#[test]
fn d_power_gate_holds_a_due_commit_while_unpowered_and_applies_it_once_repowered() {
    // 0 = gnd, 1 = 5V rail (fixed), 2 = vcc (free, fed through a switch from
    // the rail), 3 = 1y (tracked), 4 = 1a (free, through an spdt: b=gnd,
    // c=the fixed 5V rail — an INDEPENDENT level source from the power
    // switch, so toggling it never touches vcc), 5 = a shared dump for
    // every unused gate (inputs tied LOW, so they all agree on HIGH).
    //
    // The pending transition below is deliberately raised by a REAL input
    // change (1a's own switch), not `Kernel::request_driver` directly: a
    // synthetic request has no input state behind it, so the reader's own
    // re-evaluation on the first powered pass (contract point 9: "the
    // partition is solved with the pre-outage committed levels") would see
    // 1a unchanged and re-request the STILL-committed level, which the
    // inertial FSM correctly reads as a glitch-back that CANCELS whatever
    // is pending (`drivers::fsm`'s own "a glitch shorter than the delay is
    // swallowed" row) — the same cancellation a real glitch deserves, just
    // not what this test means to exercise. Driving the pending transition
    // from a real, still-in-effect input change means that same
    // re-evaluation instead re-requests the SAME target already pending,
    // which the FSM leaves alone ("no reschedule").
    let p = program(6, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 2, 1, 1.0), // vcc power switch, closed at load.
        spdt(2, 4, 0, 1, 0.0),    // 1a selector: sel=0 selects gnd (LOW).
        ls04(3, 2, 0, [4, 3, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 3, 1); // 1y
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "powered at settle: 1a LOW -> 1y HIGH");

    let t0 = kernel.time();
    // 1a -> HIGH while still powered: a real (20ns) transition request,
    // 1y -> LOW, not yet applied.
    kernel.solver().validate_params(&[(2, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(2, 0, 1.0)]).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "the request is in flight, not yet due");
    assert_eq!(kernel.driver(y).pending(), Some(crate::drivers::Pending { target: Mode::Low, due: t0 + 20e-9 }));

    // Cut power before that commit becomes due.
    kernel.solver().validate_params(&[(1, 0, 0.0)]).unwrap();
    kernel.apply_params_as_event(&[(1, 0, 0.0)]).unwrap();
    assert!(!kernel.solver().ic_powered(3), "the IC must read unpowered once its vcc node has no path to the rail");
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "cutting power alone must not touch the committed level");
    assert_eq!(kernel.driver(y).pending(), Some(crate::drivers::Pending { target: Mode::Low, due: t0 + 20e-9 }), "cutting power must not touch the in-flight transition either");

    kernel.advance(t0 + 20e-9).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "the due commit must be HELD, not applied, while unpowered");
    assert_eq!(
        kernel.driver(y).pending(),
        Some(crate::drivers::Pending { target: Mode::Low, due: t0 + 20e-9 }),
        "the in-flight transition stays frozen while held"
    );
    // "opening it leaves no driver stamps": with the IC unpowered, its
    // output node has no stamp at all -- only the universal 1e-12 S shunt,
    // so it reads essentially 0V despite still being committed HIGH.
    assert!(kernel.solver().voltages()[3].abs() < 1e-6, "got {}", kernel.solver().voltages()[3]);

    // Re-power: the held commit must apply in the delta after power
    // returns.
    kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "the held commit must apply once the IC is powered again");
}

// ============================================================================
// (e) the fixed point.
// ============================================================================

#[test]
fn e_fixed_point_vcc_through_10ohm_output_low_into_100ohm_to_vcc_matches_a_hand_iteration() {
    // 0 = gnd (fixed), 1 = 5V rail (fixed), 2 = vcc (free, 10 ohm from the
    // rail), 3 = 1y (free, 100 ohm to vcc — not to gnd). 1a tied to the 5V
    // rail (HIGH, deterministic): 1y is always driven LOW once powered.
    let p = program(9, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 2, 1, 10.0),
        resistor(2, 3, 2, 100.0),
        ls04(3, 2, 0, [1, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 3, 1);
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "1a tied HIGH -> 1y settles LOW once powered");

    // A hand fixed-point iteration over the SAME two-node network, driven by
    // the SAME `digital::power::ic_power` formula, independent of
    // `iterate_solve`'s own implementation.
    let ic = IcRecord::from_device(&p.devices[3]);
    let (g_src, g_link, shunt) = (1.0 / 10.0, 1.0 / 100.0, 1e-12);
    let mut vcc = 0.0f64;
    let mut vy = 0.0f64;
    for _ in 0..50 {
        let power = digital::power::ic_power(&ic, vcc, 0.0);
        let g_out = if power.powered { 1.0 / power.r_out } else { 0.0 };
        let a11 = g_src + shunt + g_link;
        let a12 = -g_link;
        let a21 = -g_link;
        let a22 = g_link + shunt + g_out;
        let b1 = g_src * 5.0;
        let b2 = 0.0;
        let det = a11 * a22 - a12 * a21;
        let new_vcc = (b1 * a22 - a12 * b2) / det;
        let new_vy = (a11 * b2 - a21 * b1) / det;
        let dvcc = (new_vcc - vcc).abs();
        let dvy = (new_vy - vy).abs();
        vcc = new_vcc;
        vy = new_vy;
        if dvcc <= 1e-6 + 1e-3 * vcc.abs() && dvy <= 1e-6 + 1e-3 * vy.abs() {
            break;
        }
    }

    assert!((kernel.solver().voltages()[2] - vcc).abs() < 1e-6, "vcc: got {}, hand iteration {vcc}", kernel.solver().voltages()[2]);
    assert!((kernel.solver().voltages()[3] - vy).abs() < 1e-6, "vy: got {}, hand iteration {vy}", kernel.solver().voltages()[3]);
}

// ============================================================================
// (f) a bucket boundary where the open-pin verdict flips.
// ============================================================================

#[test]
fn f_a_bucket_boundary_where_the_open_pin_verdict_flips_resolves_exactly_at_the_boundary() {
    let prefix = fnv1a("u1:1a:");
    // Search for a bucket boundary where the golden verdict actually flips,
    // using the real hash (`digital::floating::floating_high`) rather than
    // assuming one.
    let ttl = true;
    let mut before_bucket = 0u64;
    let mut before_verdict = digital::floating::floating_high(prefix, 0.005, ttl);
    let mut found = None;
    for b in 1..200_000u64 {
        let t = (b as f64) * 0.01 + 0.005;
        let v = digital::floating::floating_high(prefix, t, ttl);
        if v != before_verdict {
            found = Some((before_bucket, b));
            break;
        }
        before_bucket = b;
        before_verdict = v;
    }
    let (b_before, b_after) = found.expect("a flip must exist within the searched range");
    let verdict_before = digital::floating::floating_high(prefix, (b_before as f64) * 0.01 + 0.005, ttl);
    let verdict_after = digital::floating::floating_high(prefix, (b_after as f64) * 0.01 + 0.005, ttl);
    assert_ne!(verdict_before, verdict_after);

    let start_time = (b_before as f64) * 0.01 + 0.001;
    let boundary = digital::floating::next_bucket_boundary(start_time);
    assert_eq!(digital::floating::bucket(boundary), b_after);

    // 0 = gnd, 1 = vcc (5V fixed), 2 = 1a (OPEN, floating_input, the real
    // prefix), 3 = 1y (isolated, no load). Zero delay so the flip commits
    // exactly at the boundary, with nothing else in flight.
    let mut opts = Options::default();
    opts.start_time = start_time;
    let mut p = program(9, vec![
        source(0, 1, 0, 5.0),
        ls04(1, 1, 0, [2, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [true, false, false, false, false, false, false, false, false, false, false, false], [prefix, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0.008, 0.0),
    ]);
    p.options = opts;
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(start_time, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 1, 1);
    let expected_before = if verdict_before { Mode::Low } else { Mode::High };
    assert_eq!(kernel.driver(y).committed(), Some(expected_before), "before the boundary: NOT(golden verdict)");

    kernel.advance(boundary).unwrap();
    let expected_after = if verdict_after { Mode::Low } else { Mode::High };
    assert_eq!(kernel.driver(y).committed(), Some(expected_after), "exactly at the boundary: the flipped verdict must have re-resolved");
}

// ============================================================================
// (g) rollback: a held commit, and the bucket schedule.
// ============================================================================
//
// Both tests below share a 3-gate ring built from a SEPARATE 74ls04 (fixed
// rails, delay 0): `1y->2a, 2y->3a`, and `3y` feeds back to `1a` through an
// spdt whose `b` throw is a fixed LOW instead. With `sel=0` the loop is
// broken (`1a` reads a deterministic LOW, the ring settles quietly, exactly
// like every other gate in these tests); with `sel=1` it closes an ODD
// inversion loop at zero propagation delay, which can never reach a quiet
// fixed point — the same shape as `kernel.rs`'s own oscillating-pair
// livelock test, but three real IC gates instead of a mock evaluator, so
// closing the switch is a reliable, on-demand way to make an
// `apply_params_as_event` call fail without needing any synthetic
// oscillator machinery of its own.

#[test]
fn g1_rollback_restores_a_held_commit_the_failed_attempts_own_retry_touched() {
    // Same power-gate shape as test (d) (1a driven through its own spdt, so
    // the pending transition comes from a real, still-in-effect input
    // change — see that test's doc comment on why), plus the ring:
    // 0=gnd, 1=5V rail, 2=vcc (power-gated), 3=1y (tracked), 4=1a
    // (power-gated IC, via its own spdt), 5=dump for the power-gated IC's
    // OTHER 5 gates, 6=ring com/1a, 7=ring y1 (=2a), 8=ring y2 (=3a), 9=ring
    // y3, 10=a SEPARATE dump for the ring's other 3 gates. The two ICs'
    // dumps must NOT be the same node: the ring is always powered, and
    // sharing a node would let its drivers keep that node (and, through the
    // power-gated IC's own 2y..6y driver stamps into it, `vcc` itself) near
    // 5V even after the power switch opens — an unwanted backfeed the real
    // circuit this models never has, since a real gate's output never wires
    // straight back to its own supply pin.
    let p = program(11, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 2, 1, 1.0), // power switch, closed at load.
        spdt(2, 4, 0, 1, 0.0),    // 1a selector: sel=0 selects gnd (LOW).
        ls04(3, 2, 0, [4, 3, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5], [false; 12], [0; 12], 0.008, 20e-9),
        spdt(4, 6, 0, 9, 0.0), // ring-arming switch: sel=0 (b=gnd) breaks the loop.
        ls04(5, 1, 0, [6, 7, 7, 8, 8, 9, 0, 10, 0, 10, 0, 10], [false; 12], [0; 12], 0.008, 0.0),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 3, 1); // the power-gated IC's 1y.
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High));

    let t0 = kernel.time();
    kernel.solver().validate_params(&[(2, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(2, 0, 1.0)]).unwrap(); // 1a -> HIGH while still powered.
    assert_eq!(kernel.driver(y).pending(), Some(crate::drivers::Pending { target: Mode::Low, due: t0 + 20e-9 }));

    kernel.solver().validate_params(&[(1, 0, 0.0)]).unwrap();
    kernel.apply_params_as_event(&[(1, 0, 0.0)]).unwrap(); // unpower.
    kernel.advance(t0 + 20e-9).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "held, not applied, while unpowered");
    assert_eq!(kernel.driver(y).pending(), Some(crate::drivers::Pending { target: Mode::Low, due: t0 + 20e-9 }));

    // Re-power AND arm the ring in ONE call: the retry re-queues the held
    // commit (touching `held_commits`) while the ring's zero-delay loop
    // never settles -- the whole instant fails.
    kernel.solver().validate_params(&[(1, 0, 1.0), (4, 0, 1.0)]).unwrap();
    let err = kernel.apply_params_as_event(&[(1, 0, 1.0), (4, 0, 1.0)]).unwrap_err();
    assert!(matches!(err, KernelError::Livelock { .. }), "the ring must never settle once closed: {err:?}");

    // Rolled back: the driver's own committed/pending state is exactly as
    // it was (an ordinary part of the rollback), but the REAL question is
    // whether `held_commits` itself survived -- proven by a SECOND, clean
    // re-power (ring left alone this time) still delivering the commit.
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High));
    assert_eq!(kernel.driver(y).pending(), Some(crate::drivers::Pending { target: Mode::Low, due: t0 + 20e-9 }));

    kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "the held commit must still be there after the failed attempt's rollback, and now applies");
}

#[test]
fn g2_rollback_restores_the_bucket_schedule_the_failed_attempts_own_scheduling_touched() {
    // 0=gnd, 1=5V rail (fixed), 2=vcc (power-gated, free), 3=1a (OPEN,
    // floating_input, the real "u1:1a:" prefix), 4=1y, 5=a dump for the
    // power-gated IC's OTHER 5 gates, 6=ring com/1a, 7=ring y1 (=2a), 8=ring
    // y2 (=3a), 9=ring y3, 10=a SEPARATE dump for the ring's other 3 gates
    // (test (g1)'s own doc comment on why the two ICs must not share one).
    let prefix = fnv1a("u1:1a:");
    let p = program(11, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 2, 1, 0.0), // power switch, OPEN at load: no bucket watch yet.
        ls04(
            2,
            2,
            0,
            [3, 4, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5],
            [true, false, false, false, false, false, false, false, false, false, false, false],
            [prefix, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            0.008,
            20e-9,
        ),
        spdt(3, 6, 0, 9, 0.0), // ring-arming switch, broken at load.
        ls04(4, 1, 0, [6, 7, 7, 8, 8, 9, 0, 10, 0, 10, 0, 10], [false; 12], [0; 12], 0.008, 0.0),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let before_next = kernel.next_event_time();
    assert_eq!(before_next, f64::INFINITY, "unpowered: no bucket watch yet, nothing else pending");

    // Power the bucket IC AND arm the ring in ONE call: the resolve that
    // powers it schedules a fresh bucket boundary (touching
    // `PartitionedSolver::next_bucket`) while the ring livelocks.
    kernel.solver().validate_params(&[(1, 0, 1.0), (3, 0, 1.0)]).unwrap();
    let err = kernel.apply_params_as_event(&[(1, 0, 1.0), (3, 0, 1.0)]).unwrap_err();
    assert!(matches!(err, KernelError::Livelock { .. }));

    // Rolled back: the bucket IC must still read exactly as unpowered/
    // unscheduled as before this failed attempt.
    assert_eq!(kernel.next_event_time(), f64::INFINITY, "the failed attempt's own scheduling must not survive rollback");
    assert!(!kernel.solver().ic_powered(2));

    // A clean, second power-up (ring left alone) must now schedule a fresh
    // bucket boundary correctly -- proving `next_bucket`'s dedup flag was
    // not left in some corrupted state by the earlier rollback.
    kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap();
    assert!(kernel.next_event_time().is_finite(), "a fresh bucket boundary must now be scheduled");
}

// ============================================================================
// W1c step 2b: IC driver/pull stamps in closed-form and stepped partitions
// (`partition.rs`'s former decline lifted). Every test below exercises the
// REAL kernel/solver/harness round trip, exactly like (a)-(g) above, but with
// the IC's own driver or pull pin landing in a dynamic partition instead of a
// `Static` one.
// ============================================================================

// ---- (h) an inverter driving an RC: the exact closed form with R + R_out --

#[test]
fn h_inverter_output_driving_an_rc_matches_the_exact_closed_form_with_r_plus_r_out() {
    // 0=gnd, 1=vcc (5V fixed), 2=1y (the driver's own node, R_out couples it
    // to vcc while committed HIGH), 3=X (1y --R-- X --C-- gnd). 1a and every
    // other gate's input tie to gnd (LOW, deterministic); the other 5 gates'
    // outputs each get their own isolated node -- none of this ever touches
    // the RC partition (vcc/gnd fixed, so "no IC pin joins anything" other
    // than through the plain R union, README's own point 2).
    let (r, c) = (10_000.0, 100e-9);
    let p = program(9, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 2, 3, r),
        capacitor(2, 3, 0, c, 0.0, 0.0),
        ls04(3, 1, 0, [0, 2, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_x, _) = pb.local_index[3].expect("X is a free node");
    assert_eq!(pb.kinds[p_x], crate::partition::PartitionKind::ClosedForm, "one capacitor, fixed rails: ClosedForm");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 3, 1); // 1y.
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "1a LOW -> 1y HIGH");

    // R_out = clamp(5/(4*0.008), 50, 1000) = 156.25 ohm (digital::power's own
    // r_out_at_5v golden value, matching a1's own LS-TTL gate figure).
    let r_out = 156.25;
    let tau = (r + r_out) * c;
    // vinf is exactly 5V up to the universal 1e-12 S node shunt's negligible
    // deviation (closed_form.rs's own tests quantify this at ~R*NODE_RSHUNT_G,
    // here about (10_000+156.25)*1e-12 ~ 1e-8, well under this test's bound).
    for &t in &[1e-5, 5e-5, 2e-4, 1e-3] {
        kernel.advance(t).unwrap();
        let expected = 5.0 * (1.0 - (-t / tau).exp());
        let got = kernel.solver().voltages()[3];
        let rel = (got - expected).abs() / expected.abs();
        assert!(rel < 1e-6, "t={t}: got {got} expected {expected} rel {rel}");
    }
}

// ---- (i)/(j) the canonical 74HC14 relaxation oscillator, fixed 5V rails ---
//
// The inverter's own output drives R = 10 kOhm into C = 100 nF at node X,
// and X is the same inverter's own input (`1a`) -- the textbook Schmitt
// relaxation oscillator. Schmitt thresholds at 5V are 1.6/2.7 V (this
// module's own `hc14_params`) and R_out = 62.5 ohm (`digital::power`'s
// `r_out_at_5v` golden value for `io_max = 0.02`); `delay = 15e-9` (the
// README's own "plus two 15 ns delays"). The period is
// `tau*ln((5-1.6)/(5-2.7)) + tau*ln(2.7/1.6)` with `tau = (R + 62.5)*C`, plus
// two 15 ns delays for the two inertial commits every cycle -- verified
// below against the driver's own committed-level transitions, measured from
// the SECOND full cycle on: the very first charge leg starts from `vc = 0`
// at load (contract point 5), not from the steady orbit's own `vt_minus`, so
// only the periods AFTER that first, atypically-long leg are the steady,
// analytically-predicted ones.

/// Node/device layout shared by the closed-form and `FORCE_STEPPED` variants
/// below: `0` = gnd, `1` = vcc (5V fixed), `2` = `1y` (the driver's own
/// node), `3` = X (`1y --R-- X --C-- gnd`, and X = `1a`, the SAME inverter's
/// own input -- the feedback). Every other gate's input ties to gnd (LOW,
/// deterministic); the other 5 gates' outputs each get their own isolated
/// node, never touching this partition (vcc/gnd fixed: "no IC pin joins
/// anything" beyond the plain R union, README's own point 2). Returns
/// `(program, x_node, ic_device_index, y_output_logic_index)`.
fn hc14_oscillator_program(force_stepped: bool) -> (Program, usize, usize, usize) {
    let (r, c) = (10_000.0, 100e-9);
    let mut opts = Options::default();
    opts.force_stepped = force_stepped;
    let mut p = program(9, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 2, 3, r),
        capacitor(2, 3, 0, c, 0.0, 0.0),
        hc14(3, 1, 0, [3, 2, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], [0; 12], 0.02, 15e-9),
    ]);
    p.options = opts;
    (p, 3, 3, 1)
}

/// The Schmitt watch: both directions, since a decaying segment's own
/// asymptote can head toward EITHER threshold depending on which way the
/// driver is currently committed -- `schedule_crossing`/`find_crossing`
/// (closed-form/stepped) only ever produce a candidate for whichever
/// direction the CURRENT trajectory actually reaches, so registering both up
/// front (fixed rails: the thresholds never move) is exactly the "reader
/// registers its live thresholds" contract point 4 describes, minus the
/// live re-registration a moving rail would need (irrelevant here).
fn watch_hc14_schmitt(kernel: &mut Kernel<PartitionedSolver, IcHarness>, x_node: usize, ic_device_index: usize) {
    assert!(kernel.solver_mut().watch_node(
        x_node,
        DeviceId(ic_device_index as u32),
        Box::new(|| vec![Threshold { value: 2.7, rising: true }, Threshold { value: 1.6, rising: false }]),
    ));
}

/// The task's own hand-computed formula: `tau*ln((5-1.6)/(5-2.7)) +
/// tau*ln(2.7/1.6)` with `tau = (R + 62.5)*C`, plus two 15 ns delays --
/// treating each propagation delay as flat dead time tacked onto an
/// otherwise-exact cycle that starts and ends precisely on 1.6/2.7.
fn hc14_oscillator_naive_period() -> f64 {
    let (r, r_out, c): (f64, f64, f64) = (10_000.0, 62.5, 100e-9);
    let tau = (r + r_out) * c;
    let charge = tau * ((5.0f64 - 1.6) / (5.0 - 2.7)).ln();
    let discharge = tau * (2.7 / 1.6f64).ln();
    charge + discharge + 2.0 * 15e-9
}

/// The EXACT period a real (closed-form, delay-respecting) Schmitt
/// oscillator produces, correcting what the naive formula above leaves out:
/// during each 15 ns propagation delay the capacitor keeps evolving along
/// the STILL-committed segment (the driver has not flipped yet — worker
/// R6b's own contract point 1: "A driver commit already dirties the
/// partition... must re-aim the segment at now"), so the NEXT leg does not
/// actually start precisely on the threshold; it starts wherever that extra
/// 15 ns of the OLD segment left it. This is real physics (a genuine
/// consequence of a nonzero propagation delay on a Schmitt oscillator), not
/// a discretisation artifact of this core's own solver — the correction
/// below is itself closed-form (the RC's own exponential is exact), so the
/// crossings this test measures must match IT to near machine precision. The
/// gap to the naive formula above is small (order `1e-5` relative at this
/// test's own R/C/delay) but real, and the test below checks it explicitly
/// rather than assuming a number.
fn hc14_oscillator_exact_period() -> f64 {
    let (r, r_out, c): (f64, f64, f64) = (10_000.0, 62.5, 100e-9);
    let tau = (r + r_out) * c;
    let d = 15e-9;
    let decay = (-d / tau).exp();
    // X keeps discharging toward 0 for `d` past the 1.6 crossing before the
    // driver actually commits HIGH: `v0_charge = 0 + (1.6 - 0) * decay`.
    let v0_charge = 1.6 * decay;
    let t_charge = tau * ((5.0f64 - v0_charge) / (5.0 - 2.7)).ln();
    // X keeps charging toward 5 for `d` past the 2.7 crossing before the
    // driver actually commits LOW: `v0_discharge = 5 + (2.7 - 5) * decay`.
    let v0_discharge = 5.0 + (2.7 - 5.0) * decay;
    let t_discharge = tau * (v0_discharge / 1.6).ln();
    2.0 * d + t_charge + t_discharge
}

/// Drives `kernel` through `commits_wanted` driver commits (HIGH or LOW),
/// recording `(time, mode)` for each. When `crossing_tol` is `Some`, every
/// OTHER due event in between (one that did not itself change the driver)
/// must be a threshold crossing: the published voltage at `x_node` must land
/// within that tolerance of 1.6 or 2.7 ("the crossing instants land exactly
/// on the thresholds") -- true for the closed-form solver, where EVERY event
/// it ever schedules against a registered watch IS a real crossing. `None`
/// skips that check: the stepped solver's OWN due events are mostly ordinary
/// step-doubling commits, not crossings (`stepped.rs`'s `Step` event fires on
/// every accepted step, not only the one the crossing refinement targets),
/// so "not near a threshold" is the expected, common case there, not a bug.
/// Advances via `next_event_time` repeatedly rather than a fixed step, so
/// this exercises the REAL scheduled events one at a time, exactly as
/// `bo_engine_advance` would when a host steps to `next_event_time` itself.
fn drive_oscillator(kernel: &mut Kernel<PartitionedSolver, IcHarness>, x_node: usize, y: DriverId, commits_wanted: usize, crossing_tol: Option<f64>) -> Vec<(f64, Mode)> {
    let mut last_mode = kernel.driver(y).committed();
    let mut commits = Vec::new();
    while commits.len() < commits_wanted {
        let t = kernel.next_event_time();
        assert!(t.is_finite(), "the oscillator must always have a next event scheduled");
        kernel.advance(t).unwrap();
        let mode = kernel.driver(y).committed();
        if mode != last_mode {
            commits.push((kernel.time(), mode.expect("a driver that just changed must be committed to something")));
            last_mode = mode;
        } else if let Some(tol) = crossing_tol {
            let v = kernel.solver().voltages()[x_node];
            assert!((v - 2.7).abs() < tol || (v - 1.6).abs() < tol, "a due event that did not commit the driver must be a threshold crossing landing on 1.6 or 2.7, got v={v}");
        }
    }
    commits
}

/// Asserts every FULL period (same-mode commit to same-mode commit, two
/// transitions apart — `commits[i]` to `commits[i+2]`) is within `rel_tol` of
/// `expected_period`, starting from index 1: `commits[0]` is the END of the
/// atypical FIRST charge leg (`vc = 0` at load, not the steady orbit's own
/// `vt_minus`), so the period `commits[0] -> commits[2]` would still include
/// that one atypically-long leg. Every period starting from `commits[1]`
/// onward begins and ends exactly on a Schmitt threshold (the previous
/// commit's own crossing-nudged landing spot), so it is genuinely the
/// steady, analytically-predicted cycle.
fn assert_periods_match(commits: &[(f64, Mode)], expected_period: f64, rel_tol: f64) {
    assert!(commits.len() >= 4, "need at least two full periods to check, got {} commits", commits.len());
    for w in commits.windows(2) {
        assert_ne!(w[0].1, w[1].1, "consecutive commits must alternate HIGH/LOW");
    }
    let mut checked = 0;
    for i in 1..commits.len() - 2 {
        let (t0, m0) = commits[i];
        let (t2, m2) = commits[i + 2];
        assert_eq!(m0, m2, "two commits apart must be the same mode");
        let period = t2 - t0;
        let rel = (period - expected_period).abs() / expected_period;
        assert!(rel < rel_tol, "period {t0}->{t2} ({m0:?}) = {period}, expected {expected_period}, rel {rel}");
        checked += 1;
    }
    assert!(checked >= 2, "expected at least two full periods checked, got {checked}");
}

#[test]
fn i_hc14_oscillator_closed_form_period_matches_the_analytic_formula_and_crossings_land_on_the_thresholds() {
    let (p, x_node, ic_index, y_logic) = hc14_oscillator_program(false);
    let pb = crate::partition::build(&p).unwrap();
    let (p_x, _) = pb.local_index[x_node].expect("X is a free node");
    assert_eq!(pb.kinds[p_x], crate::partition::PartitionKind::ClosedForm);

    let (mut kernel, all_partitions, all_readers) = build(&p);
    watch_hc14_schmitt(&mut kernel, x_node, ic_index);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, ic_index, y_logic);
    let commits = drive_oscillator(&mut kernel, x_node, y, 8, Some(1e-6));

    // The exact, delay-corrected period, to nine decimal digits: this is
    // what the closed-form solver's own exact crossings and re-aims (worker
    // R6b's own contract point 1) must reproduce, since nothing about them
    // is approximate. The residual few `1e-9`s beyond that are the LU
    // solve's own floating-point rounding (`closed_form.rs`'s own tests hold
    // themselves to the identical `< 1e-9`/`< 3e-9` bar for the same reason).
    assert_periods_match(&commits, hc14_oscillator_exact_period(), 1e-6);

    // The task's own hand-computed formula, "within 1e-6 relative" as
    // stated -- except a real Schmitt oscillator with a NONZERO propagation
    // delay does not actually hit that: `hc14_oscillator_exact_period`'s own
    // doc comment on why (the capacitor keeps moving for a whole 15 ns past
    // each threshold before the driver's commit actually lands, which the
    // naive formula's flat "+2*15ns dead time" does not model). Checked here
    // explicitly, at a bound wide enough for that real, derived gap (order
    // 1e-5 at this R/C/delay) rather than silently dropped: the naive
    // formula is still the right one for a delay-free comparison, and
    // matches to the precision that comparison actually supports.
    let naive = hc14_oscillator_naive_period();
    let exact = hc14_oscillator_exact_period();
    let gap = (exact - naive).abs() / naive;
    assert!(gap < 5e-5, "naive vs. exact period gap {gap} is larger than the propagation-delay correction should ever produce");
    assert_periods_match(&commits, naive, 1e-4);
}

#[test]
fn j_hc14_oscillator_force_stepped_stays_within_the_stepped_solvers_accuracy() {
    // The IDENTICAL circuit, `FORCE_STEPPED` (Decision 14; core/README.md's
    // Options table key 4): the same single-capacitor partition now runs the
    // general BE/trap step-doubling path instead of the exact closed form.
    // Bound used: 2e-2 relative, the SAME bound `solve/stepped.rs`'s own
    // `two_capacitor_ladder_converges_to_the_eigendecomposition_reference`
    // test holds a numerically-converged stepped trajectory to against an
    // independent reference ("a numerically-converged stepped result, not
    // the single-capacitor closed form's ~1e-9 bar") -- the step-doubling
    // controller's own per-step acceptance threshold (`stepped::REL_TOL =
    // 1e-3`) bounds a SINGLE step's local voltage/charge error, but the
    // crossing TIME this test measures compounds many such steps' local
    // errors across two full RC legs per period, so the period bound is
    // deliberately looser than the per-step one, not the tightest number
    // this run happens to hit.
    let (p, x_node, ic_index, y_logic) = hc14_oscillator_program(true);
    let pb = crate::partition::build(&p).unwrap();
    let (p_x, _) = pb.local_index[x_node].expect("X is a free node");
    assert_eq!(pb.kinds[p_x], crate::partition::PartitionKind::Stepped, "FORCE_STEPPED reclassifies the single-capacitor partition");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    watch_hc14_schmitt(&mut kernel, x_node, ic_index);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, ic_index, y_logic);
    let commits = drive_oscillator(&mut kernel, x_node, y, 8, None);
    // Compared against the EXACT (delay-corrected) period, not the naive
    // formula: at a 2e-2 bound the ~1e-5 gap between the two (this module's
    // own `hc14_oscillator_exact_period` doc comment) is immaterial either
    // way, but the exact one is the physically correct reference.
    assert_periods_match(&commits, hc14_oscillator_exact_period(), 2e-2);
}

// ---- (k) a free-rail IC in a stepped partition -----------------------

#[test]
fn k_free_rail_ic_in_a_stepped_partition_converges_and_a_supply_dip_unpowers_it_at_the_crossing() {
    // 0=gnd, 1=5V rail (fixed), 2=vcc (free, fed via 10 ohm from the rail
    // plus a 1uF decoupling capacitor), 3=a dump junction (free, 30 ohm from
    // vcc, normally open to gnd), 4..9=the six Y outputs (free too: vcc is
    // free, so every free output-capable pin unions with it, README's own
    // point 2 -- this partition ends up with 8 free nodes, all Stepped).
    // A's tie to gnd (LOW, deterministic; unused by this test).
    let p = program(10, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 10.0),             // R_feed: 5V rail -> vcc.
        capacitor(2, 2, 0, 1e-6, 0.0, 0.0),   // C_decouple, on vcc.
        resistor(3, 2, 3, 30.0),              // R_dump: vcc -> the dump junction.
        switch_dev(4, 3, 0, 0.0),             // S_dump, OPEN at load.
        ls04(5, 2, 0, [0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_vcc, _) = pb.local_index[2].expect("vcc is free");
    assert_eq!(pb.kinds[p_vcc], crate::partition::PartitionKind::Stepped, "a free IC supply node forces Stepped (contract point 3)");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    let ic_index = 5;
    let (lo, _hi) = digital::levels::power_window(&IcRecord::from_device(&p.devices[ic_index]));
    assert!((lo - 4.75 * 0.9).abs() < 1e-9, "LS-TTL power window floor");
    // The power-window watch (contract point 2's own "a supply pin through
    // the power window... If gnd is fixed, that is a threshold on the vcc
    // node"): both directions registered up front, so the SAME watch catches
    // both the initial power-up (rising) and the later dip (falling).
    assert!(kernel.solver_mut().watch_node(2, DeviceId(ic_index as u32), Box::new(move || vec![
        Threshold { value: lo, rising: true },
        Threshold { value: lo, rising: false },
    ])));
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    // At load: vc=0 at the decoupling capacitor (contract point 5), so vcc
    // starts at 0V -- well below the power window. Settling itself must not
    // fail: the per-step Newton fixed point (deliverable point 1) converges
    // even while unpowered, when R_out simply does not apply at all.
    assert!(!kernel.solver().ic_powered(ic_index), "vcc=0V at load: unpowered");

    // Charge up (tau = R_feed*C = 10us, dump switch still open) until vcc
    // clears the power window -- proving the free-rail Newton iteration
    // converges as R_out switches in partway through the charge.
    let mut powered_at = None;
    for _ in 0..2000 {
        let t = kernel.next_event_time();
        assert!(t.is_finite(), "vcc must always have a next scheduled event while charging");
        kernel.advance(t).unwrap();
        if kernel.solver().ic_powered(ic_index) {
            powered_at = Some(kernel.time());
            break;
        }
    }
    let powered_at = powered_at.expect("vcc must cross into the power window while charging");
    let v_at_power_on = kernel.solver().voltages()[2];
    assert!((v_at_power_on - lo).abs() < 1e-6, "the power-up crossing should land within a tight tolerance of the window's own threshold, got {v_at_power_on}");

    // Let it settle comfortably above the threshold (steady state ~5V, dump
    // switch still open) before disturbing it.
    kernel.advance(powered_at + 200e-6).unwrap();
    assert!(kernel.solver().ic_powered(ic_index));
    let steady_v = kernel.solver().voltages()[2];
    assert!(steady_v > 4.9, "expected vcc to have settled close to 5V by now, got {steady_v}");

    // Close the dump switch: a new, LOWER steady state (5*30/(30+10) =
    // 3.75V) sits below the power window, so vcc must dip through it.
    kernel.solver().validate_params(&[(4, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(4, 0, 1.0)]).unwrap();
    assert!(kernel.solver().ic_powered(ic_index), "closing the dump switch alone must not immediately unpower it");

    let mut unpowered_at = None;
    for _ in 0..2000 {
        let t = kernel.next_event_time();
        assert!(t.is_finite(), "vcc must always have a next scheduled event while dipping");
        kernel.advance(t).unwrap();
        if !kernel.solver().ic_powered(ic_index) {
            unpowered_at = Some(kernel.time());
            break;
        }
    }
    let unpowered_at = unpowered_at.expect("vcc must dip below the power window once the dump switch is closed");
    let v_at_power_off = kernel.solver().voltages()[2];
    assert!((v_at_power_off - lo).abs() < 1e-6, "the dip must unpower the IC exactly at the crossing, got vcc={v_at_power_off}");
    assert!(unpowered_at > powered_at);
}

// ---- (l) a bucket-boundary re-aim in a closed-form partition, open input -

#[test]
fn l_bucket_boundary_reaim_in_a_closed_form_partition_with_an_open_floating_input() {
    // 0=gnd, 1=vcc (5V fixed), 2=X (an OPEN, FLOATING_INPUT pin -- 2a -- AND
    // the partition's one capacitor, both on the SAME node), 3..8=the other
    // five gates' outputs (isolated dumps; their A's tie to gnd).
    let prefix = fnv1a("u1:2a:");
    let ttl = true;
    // Find a real bucket boundary where the golden verdict actually flips
    // (test (f)'s own search loop), so this exercises the real hash, not an
    // assumed one.
    let mut before_bucket = 0u64;
    let mut before_verdict = digital::floating::floating_high(prefix, 0.005, ttl);
    let mut found = None;
    for b in 1..200_000u64 {
        let t = (b as f64) * 0.01 + 0.005;
        let v = digital::floating::floating_high(prefix, t, ttl);
        if v != before_verdict {
            found = Some((before_bucket, b));
            break;
        }
        before_bucket = b;
        before_verdict = v;
    }
    let (b_before, b_after) = found.expect("a flip must exist within the searched range");
    let verdict_before = digital::floating::floating_high(prefix, (b_before as f64) * 0.01 + 0.005, ttl);
    let verdict_after = digital::floating::floating_high(prefix, (b_after as f64) * 0.01 + 0.005, ttl);
    assert_ne!(verdict_before, verdict_after);

    let start_time = (b_before as f64) * 0.01 + 0.001; // 1ms into the "before" bucket: 9ms of runway before the boundary.
    let boundary = digital::floating::next_bucket_boundary(start_time);
    assert_eq!(digital::floating::bucket(boundary), b_after);

    // TTL pull: g = 1/1MOhm. tau = C/g = 1e-9/1e-6 = 1ms -- three time
    // constants (3ms) comfortably fit within the 9ms of runway on each side
    // of the boundary this test uses.
    let (c, pull_g): (f64, f64) = (1e-9, 1e-6);
    let tau: f64 = c / pull_g;

    let mut opts = Options::default();
    opts.start_time = start_time;
    let mut p = program(9, vec![
        source(0, 1, 0, 5.0),
        capacitor(1, 2, 0, c, 0.0, 0.0),
        ls04(
            2,
            1,
            0,
            [0, 3, 2, 4, 0, 5, 0, 6, 0, 7, 0, 8],
            [false, false, true, false, false, false, false, false, false, false, false, false],
            [0, 0, prefix, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            0.008,
            20e-9,
        ),
    ]);
    p.options = opts;

    let pb = crate::partition::build(&p).unwrap();
    let (p_x, _) = pb.local_index[2].expect("X is a free node");
    assert_eq!(pb.kinds[p_x], crate::partition::PartitionKind::ClosedForm, "one capacitor, fixed rails: ClosedForm even with the open pull sharing its node");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(start_time, &all_partitions, &all_readers).unwrap();

    let target_before = if verdict_before { 5.0 } else { 0.0 };
    let target_after = if verdict_after { 5.0 } else { 0.0 };

    // The universal node shunt (core/README.md's W0 point 2, 1e-12 S) sits
    // in parallel with the pull itself here (1e-6 S, TTL): a THIRD path the
    // naive target-only formula below does not include, deviating from it by
    // about `NODE_RSHUNT_G / pull_g` ~ 1e-6 relative — `closed_form.rs`'s own
    // "exact against the shunt-inclusive model" test quantifies the identical
    // effect for a resistor-fed RC; a MUCH weaker pull conductance (1MOhm,
    // vs. a plain resistor's typical few kOhm) makes it proportionally
    // larger here, though still far below any value this fixture could
    // mistake for a real bug.
    let shunt_tol = 1e-5;

    // Three time constants into the "before" bucket: X has moved
    // meaningfully toward `target_before`, well before the boundary.
    kernel.advance(start_time + 3.0 * tau).unwrap();
    let x_before = kernel.solver().voltages()[2];
    let expected_before = target_before * (1.0 - (-3.0f64).exp());
    if target_before == 0.0 {
        assert!(x_before.abs() < 1e-6, "expected X to stay near 0V toward a LOW pull, got {x_before}");
    } else {
        let rel = (x_before - expected_before).abs() / expected_before.abs();
        assert!(rel < shunt_tol, "before the boundary: got {x_before} expected {expected_before} rel {rel}");
    }

    // Advance exactly to the boundary: the Bucket event fires, the pull's
    // target flips, and the partition must re-aim (deliverable point 1's
    // first bullet: "a bucket boundary re-dirties pulls... must re-aim the
    // segment at now") -- landing continuously (no jump) on the OLD
    // trajectory's own value at this instant.
    kernel.advance(boundary).unwrap();
    let x_at_boundary = kernel.solver().voltages()[2];
    let expected_at_boundary = target_before * (1.0 - (-(boundary - start_time) / tau).exp());
    if target_before == 0.0 {
        assert!(x_at_boundary.abs() < 1e-6, "expected X to still be near 0V exactly at the boundary, got {x_at_boundary}");
    } else {
        let rel = (x_at_boundary - expected_at_boundary).abs() / expected_at_boundary.abs();
        assert!(rel < shunt_tol, "at the boundary (continuity, still the OLD target): got {x_at_boundary} expected {expected_at_boundary} rel {rel}");
    }

    // Three time constants past the boundary: X must now be heading toward
    // `target_after`, from `x_at_boundary`, on the NEW segment.
    kernel.advance(boundary + 3.0 * tau).unwrap();
    let x_after = kernel.solver().voltages()[2];
    let expected_after = target_after + (x_at_boundary - target_after) * (-3.0f64).exp();
    let rel = (x_after - expected_after).abs() / expected_after.abs().max(1e-6);
    assert!(rel < shunt_tol, "after the boundary (the NEW target): got {x_after} expected {expected_after} rel {rel}");
    assert_ne!(target_before, target_after, "the whole point of this fixture is that the verdict actually flips");
}

// ---- (m) rollback across a failed instant with crossing watches live -----

#[test]
fn m_rollback_across_a_failed_instant_leaves_a_live_crossing_watch_exactly_as_it_was() {
    // The same 74HC14 oscillator as (i)/(j) (0=gnd, 1=vcc, 2=1y, 3=X), PLUS
    // (g1)/(g2)'s own 3-gate zero-delay ring (9=ring com/1a, 10=ring y1,
    // 11=ring y2, 12=ring y3, 13=a dump for the ring's other 3 gates — a
    // SEPARATE node from the oscillator's own dumps, that test's own doc
    // comment on why), armed by its spdt (device 4). Device 6 is a switch
    // from X (node 3) to a fresh, otherwise-unconnected node (14), OPEN at
    // load and left OPEN: it stamps nothing (an open contact stamps
    // nothing, "open stamps nothing"), so it never perturbs the RC's own
    // dynamics, but `partition::build` still unions it into the oscillator's
    // partition unconditionally (a switch's topology is decided once, at
    // load, regardless of open/closed state) and setting it to the value it
    // ALREADY holds still dirties and re-aims that segment
    // (`PartitionedSolver::apply_params`'s unconditional `touched.insert`,
    // proven directly in `solve::mod`'s own
    // `a_switch_toggle_via_set_params_style_dispatch_reaims_and_cancels_the_old_crossing`),
    // which is what makes this a real test of ROLLING BACK a segment/
    // schedule that a failed instant genuinely touched, not merely one that
    // happened to sit untouched nearby.
    let (r, c) = (10_000.0, 100e-9);
    let p = program(15, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 2, 3, r),
        capacitor(2, 3, 0, c, 0.0, 0.0),
        hc14(3, 1, 0, [3, 2, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], [0; 12], 0.02, 15e-9),
        spdt(4, 9, 0, 12, 0.0), // ring-arming switch: sel=0 (b=gnd) breaks the loop.
        ls04(5, 1, 0, [9, 10, 10, 11, 11, 12, 0, 13, 0, 13, 0, 13], [false; 12], [0; 12], 0.008, 0.0),
        switch_dev(6, 3, 14, 0.0), // S_osc, OPEN at load, stamps nothing.
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_x, _) = pb.local_index[3].expect("X is a free node");
    assert_eq!(pb.kinds[p_x], crate::partition::PartitionKind::ClosedForm);

    let (mut kernel, all_partitions, all_readers) = build(&p);
    watch_hc14_schmitt(&mut kernel, 3, 3);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 3, 1); // the oscillator's own 1y.
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "1a LOW (vc=0 at load) -> 1y HIGH");

    // Advance partway into the first charge leg (well short of the ~390us
    // analytic charge time), so X is a genuine, nonzero, mid-charge value —
    // not merely its untouched initial 0 — with a real crossing already
    // scheduled against it.
    kernel.advance(150e-6).unwrap();
    let before_v = kernel.solver().voltages()[3];
    assert!(before_v > 0.1 && before_v < 2.7, "expected a partially-charged, still-below-threshold X, got {before_v}");
    let before_driver = kernel.driver(y).committed();
    let before_next = kernel.next_event_time();
    assert!(before_next.is_finite(), "a crossing must already be scheduled");

    // Arm the ring AND re-aim the oscillator (S_osc's own no-op close) in
    // ONE event: the ring never settles, failing the whole instant, AFTER
    // the oscillator's own segment has already been re-aimed and its
    // crossing rescheduled earlier in the SAME delta cycle.
    kernel.solver().validate_params(&[(4, 0, 1.0), (6, 0, 0.0)]).unwrap();
    let err = kernel.apply_params_as_event(&[(4, 0, 1.0), (6, 0, 0.0)]).unwrap_err();
    assert!(matches!(err, KernelError::Livelock { .. }), "the ring must never settle once armed: {err:?}");

    // Rolled back exactly: the published X voltage (bit-for-bit — no time
    // passed, `apply_params_as_event` never moves the clock), the driver's
    // committed level, AND the crossing's own schedule (proof that
    // `ClosedFormSolver`'s segment/version — and, W1c step 2b,
    // `PartitionedSolver::driver_snapshot` — actually rolled back, not just
    // the plain voltages/currents array: a stale version left behind would
    // make the OLD heap entry `is_current` again this same instant, which
    // does not fail the round trip, but pushing a WRONG replacement would).
    assert_eq!(kernel.solver().voltages()[3].to_bits(), before_v.to_bits(), "X must roll back bit-for-bit");
    assert_eq!(kernel.driver(y).committed(), before_driver);
    assert_eq!(kernel.next_event_time(), before_next, "the live crossing's own schedule must roll back exactly, not be lost or replaced");

    // And the system stays fully FUNCTIONAL afterward, not merely frozen:
    // driving it onward (`drive_oscillator`, exactly like (i)/(j)) must still
    // reach a real commit, with every intermediate due event landing on a
    // Schmitt threshold. `before_next` itself need not be the RISING-2.7
    // crossing specifically: every non-open logic pin's thresholds are
    // watched in BOTH directions (`PartitionedSolver::register_ic_read_pin_watches`'s
    // own doc comment on why the "extra" direction — X rising THROUGH 1.6 on
    // its way to 2.7 — is a real, harmless intermediate wake-up here, not a
    // sign anything rolled back incorrectly: it re-evaluates the reader,
    // finds no actual level change yet, and immediately re-schedules the
    // real 2.7 crossing from there). What must hold is that the driver still
    // reaches its next real commit, correctly, from the exact rolled-back
    // state.
    let commits = drive_oscillator(&mut kernel, 3, y, 1, Some(1e-6));
    assert_eq!(commits[0].1, Mode::Low, "the surviving schedule must still lead to 1y committing LOW next");
}

// ============================================================================
// W1c step 2b follow-up: the end-to-end proof through the PUBLIC path, with
// NO hand registration. `PartitionedSolver::build`'s own
// `register_ic_read_pin_watches` (called from `Engine::load` via
// `PartitionedSolver::build`, before `devices::ics::IcEvaluator` even exists)
// is what makes this work — unlike every test above, nothing here ever calls
// `watch_node`, `driver_for`, or touches a `Driver`/`Kernel` directly.
// ============================================================================

/// Drives `engine` through `commits_wanted` digital-slot-`slot` transitions
/// (skipping a `NaN`-to-`NaN` non-change, which is never a real transition),
/// recording `(time, value)` for each. Mirrors `drive_oscillator`'s own
/// `next_event_time`/`advance` loop, but through `Engine`'s public API only.
fn drive_oscillator_via_engine(engine: &mut Engine, slot: usize, commits_wanted: usize) -> Vec<(f64, f64)> {
    let mut last = engine.digital_slots()[slot];
    let mut commits = Vec::new();
    let mut guard = 0u32;
    while commits.len() < commits_wanted {
        guard += 1;
        assert!(guard < 10_000, "the oscillator must keep progressing toward {commits_wanted} commits");
        let t = engine.next_event_time();
        assert!(t.is_finite(), "the oscillator must always have a next event scheduled");
        engine.advance(t).unwrap();
        let now = engine.digital_slots()[slot];
        let changed = now.is_nan() != last.is_nan() || (!now.is_nan() && now != last);
        if changed {
            commits.push((engine.time(), now));
            last = now;
        }
    }
    commits
}

/// [`assert_periods_match`]'s own logic, against a digital-slot `f64` value
/// (1.0/0.0) instead of a `Mode`.
fn assert_periods_match_f64(commits: &[(f64, f64)], expected_period: f64, rel_tol: f64) {
    assert!(commits.len() >= 4, "need at least two full periods to check, got {} commits", commits.len());
    for w in commits.windows(2) {
        assert_ne!(w[0].1, w[1].1, "consecutive commits must alternate 1.0/0.0");
    }
    let mut checked = 0;
    for i in 1..commits.len() - 2 {
        let (t0, v0) = commits[i];
        let (t2, v2) = commits[i + 2];
        assert_eq!(v0, v2, "two commits apart must be the same level");
        let period = t2 - t0;
        let rel = (period - expected_period).abs() / expected_period;
        assert!(rel < rel_tol, "period {t0}->{t2} ({v0}) = {period}, expected {expected_period}, rel {rel}");
        checked += 1;
    }
    assert!(checked >= 2, "expected at least two full periods checked, got {checked}");
}

#[test]
fn n_hc14_oscillator_through_engine_load_closed_form_needs_no_hand_registration() {
    let (mut p, _x_node, _ic_index, _y_logic) = hc14_oscillator_program(false);
    // OUT_HEX2 (`74hc14`'s output-capable pins, in `icPinMap` order) is
    // `[1, 3, 5, 7, 9, 11]` — "1y" (logic index 1) is its FIRST entry, so its
    // own digital slot is `slot_base + 0 = 0`, NOT the logic-pin index 1
    // (`driver_for`'s own "output_logic_index" — a different number,
    // POSITION-searched against this same list — is what that index means
    // elsewhere in this file; a digital slot is a position within
    // `output_capable`, never a logic-pin index directly).
    const SLOT_1Y: usize = 0;
    p.digital_slot_count = 6;
    let mut engine = Engine::load(&p).expect("program must load through the real Engine, with no hand-registered watch");
    engine.trace_enable(true);

    let commits = drive_oscillator_via_engine(&mut engine, SLOT_1Y, 8);
    assert_periods_match_f64(&commits, hc14_oscillator_exact_period(), 1e-6);

    // The trace carries the SAME transitions, at the SAME instants
    // (`bo_engine_trace_*`'s own contract: "record every digital slot
    // change").
    let trace = engine.trace();
    assert_eq!(trace.len() % 3, 0);
    let traced_slot0: Vec<(f64, f64)> = trace.chunks_exact(3).filter(|c| c[1] as usize == SLOT_1Y).map(|c| (c[0], c[2])).collect();
    assert_eq!(traced_slot0.len(), commits.len(), "the trace must carry exactly the same slot-0 transitions");
    for (a, b) in traced_slot0.iter().zip(commits.iter()) {
        assert_eq!(a.0, b.0, "traced transition time must match");
        assert_eq!(a.1, b.1, "traced transition value must match");
    }
}

#[test]
fn o_hc14_oscillator_through_engine_load_force_stepped_needs_no_hand_registration() {
    let (mut p, _x_node, _ic_index, _y_logic) = hc14_oscillator_program(true);
    const SLOT_1Y: usize = 0; // see (n)'s own comment on why this is 0, not the logic-pin index 1.
    p.digital_slot_count = 6;
    let mut engine = Engine::load(&p).expect("program must load through the real Engine, with no hand-registered watch");

    let commits = drive_oscillator_via_engine(&mut engine, SLOT_1Y, 8);
    assert_periods_match_f64(&commits, hc14_oscillator_exact_period(), 2e-2);
}
