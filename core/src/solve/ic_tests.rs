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

/// Like [`ls04`], but pin index 1 ("1y") ALSO carries `PIN_FLOATING_INPUT`
/// in its blob, on top of whatever `open`/`open_hash` the caller gives it —
/// a pin that is simultaneously output-capable (`combinational::
/// output_capable(IC_74LS04)` names index 1 regardless of blob content) and
/// a `FLOATING_INPUT`, the exact shape the W1d P1 floating-pull-stamp bug
/// needs (real parts: the 74ls173 q pins, 74ls189 /o pins, 28c16/28c256 io
/// pins, 74ls245 a/b pins — none of them a plain inverter, but this is the
/// smallest device this test module already knows how to drive and read
/// that can still reproduce the bug's shape).
#[allow(clippy::too_many_arguments)]
fn ls04_with_open_capable_output(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12], open: [bool; 12], open_hash: [u32; 12], io_max: f64, delay: f64) -> Device {
    let mut device_pins = vec![pin(vcc), pin(gnd)];
    for i in 0..12 {
        device_pins.push(if open[i] { open_pin(pins[i], open_hash[i]) } else { pin(pins[i]) });
    }
    let blob_flags: Vec<f64> = (0..12).map(|i| if i % 2 == 0 || i == 1 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }).collect();
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
        // This harness is a step-1/2a truth-table and power-gating tool, not
        // a `lastReads` tracker: it always passes `None` here, which is a
        // legitimate contract state (delay.rs's "no identified cause, so
        // param 12" row for the 157), not a mock of the cause-detection rule
        // -- no test in this file depends on strobe-vs-param-12 timing.
        digital::delay::driver_requests(ic, &entry.output_drivers, &result, &levels, None)
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
        Box::new(|_v: &[f64]| vec![Threshold { value: 2.7, rising: true }, Threshold { value: 1.6, rising: false }]),
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
    assert!(kernel.solver_mut().watch_node(2, DeviceId(ic_index as u32), Box::new(move |_v: &[f64]| vec![
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

// ============================================================================
// W1c review-fix regression tests (worker R7). Each proves the finding it
// names FAILS against the pre-fix code (verified by hand: reverting the
// named fix and re-running the single test reproduces the failure) and
// PASSES against the fix.
// ============================================================================

// ---- finding 1: the horizon clamp discards sync_dynamic_to's readers -----

#[test]
fn finding1_case1_fine_grained_1us_advances_still_catch_an_unwatched_reader_on_a_free_supply() {
    // The finding's own case 1, structurally: an IC reader with NO
    // registered crossing watch at all (contract point 4, amended: an IC
    // whose supply sits in a DYNAMIC partition relies purely on the stepped
    // re-read, never a watch), so there is no pre-existing "crossing
    // refinement" safety net to fall back on -- unlike a fixed-rail reader
    // (which `register_ic_read_pin_watches` DOES watch, and whose own
    // `plan_and_schedule` keeps re-arming a correctly refined `Step` after
    // every horizon clamp regardless of this finding's bug, so a
    // single-node fixed-rail variant of this test does not actually
    // discriminate old code from fixed code -- verified by hand). 0=gnd,
    // 1=5V rail (fixed), 2=vcc (free: 10 ohm feed + 1uF decoupling cap,
    // test (k)'s own pattern -- forces the partition Stepped, contract
    // point 3), 3=1a (a bare 100k tap off vcc, in the SAME Stepped
    // partition, no capacitor of its own and no current path elsewhere, so
    // it tracks vcc's own charge exactly). 1y drives a 10k load.
    let p = program(10, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 10.0),
        capacitor(2, 2, 0, 1e-6, 0.0, 0.0),
        resistor(3, 2, 3, 100_000.0),
        ls04(4, 2, 0, [3, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_vcc, _) = pb.local_index[2].expect("vcc is free");
    let (p_a, _) = pb.local_index[3].expect("1a is free");
    assert_eq!(pb.kinds[p_vcc], crate::partition::PartitionKind::Stepped, "a free IC supply forces Stepped (contract point 3)");
    assert_eq!(p_a, p_vcc, "1a must share vcc's own (dynamic) partition, so its supply is NOT load-stable and gets no watch (findings 2/6's own eligibility rule)");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 4, 1); // 1y
    // vc=0 at load (contract point 5): unpowered until vcc (and 1a, which
    // tracks it exactly) charges into the power window.
    assert_eq!(kernel.driver(y).committed(), None, "unpowered at load: vcc starts at 0V");

    let dt = 1e-6; // 1us: with the bug, the horizon clamp's own changed-node
                   // reader report (`report_changed_ic_readers`, the ONLY
                   // mechanism available here, with no watch to fall back
                   // on) is discarded on every one of these calls.
    let mut t = 0.0;
    let mut flipped_at = None;
    while flipped_at.is_none() && t < 5e-3 {
        t += dt;
        kernel.advance(t).unwrap();
        if kernel.driver(y).committed() == Some(Mode::Low) {
            flipped_at = Some(kernel.time());
        }
    }
    let flipped_at = flipped_at.expect(
        "1y must flip within 5ms of fine-grained 1us advances -- finding 1's discarded horizon-clamp readers leave an unwatched, dynamic-supply IC reader stale forever",
    );

    // 1a tracks vcc exactly (no drop across the 100k: nothing else draws
    // current through it), so the IC powers on exactly when vcc clears the
    // power window (vcc_min*0.9 = 4.275V for LS-TTL) -- already well past
    // vih=2.0V by then, so 1y commits LOW at essentially that same instant
    // (plus the 20ns inertial delay), quantized to this test's own 1us grid.
    let tau = 10.0 * 1e-6;
    let t_power_on = -tau * (1.0 - 4.75 * 0.9 / 5.0f64).ln();
    let expected = t_power_on + 20e-9;
    assert!((flipped_at - expected).abs() < 2.0 * dt, "1y flipped at {flipped_at}, expected ~{expected} (within one grid step)");
}

#[test]
fn finding1_case2_gated_oscillator_advances_correctly_under_1us_host_steps() {
    // The finding's own case 2 (fixture 19): a 74HC14 whose vcc is free
    // (reached only through a closed switch from a fixed 5V rail, no added
    // R/C of its own -- a Static supply would be the OTHER finding, 2/6;
    // here vcc's OWN free-ness, plus the union rule joining every free
    // output-capable pin of the SAME ic with it, pulls {vcc, 1y, X} into ONE
    // partition, promoted to Stepped by contract point 3's rule). Driven by
    // 1us host steps only (never `next_event_time`), exactly the finding's
    // own reproduction shape.
    let (r, c) = (10_000.0, 100e-9);
    let p = program(10, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 1, 2, 1.0), // power switch, CLOSED at load: vcc free but ~5V.
        resistor(2, 3, 4, r),
        capacitor(3, 4, 0, c, 0.0, 0.0),
        hc14(4, 2, 0, [4, 3, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9], [false; 12], [0; 12], 0.02, 15e-9),
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_vcc, _) = pb.local_index[2].expect("vcc is free");
    assert_eq!(pb.kinds[p_vcc], crate::partition::PartitionKind::Stepped, "a free IC supply forces Stepped (contract point 3)");
    let (p_x, _) = pb.local_index[4].expect("X is free");
    assert_eq!(p_x, p_vcc, "the union rule must place X (a free output-capable-fed node) in the SAME partition as vcc");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    let ic_index = 4;
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, ic_index, 1); // 1y

    let dt = 1e-6;
    let mut t = 0.0;
    let mut first_commit = None;
    let last_mode = kernel.driver(y).committed();
    while first_commit.is_none() && t < 5e-3 {
        t += dt;
        kernel.advance(t).unwrap();
        let mode = kernel.driver(y).committed();
        if mode != last_mode {
            first_commit = Some((kernel.time(), mode.expect("a driver that just changed must be committed to something")));
        }
    }
    let (first_t, first_mode) = first_commit.expect(
        "the gated oscillator must flip within 5ms of fine-grained 1us advances -- finding 1's discarded horizon-clamp readers leave it frozen forever",
    );
    assert_eq!(first_mode, Mode::Low, "X charging past 2.7V must commit 1y LOW");

    // The first charge leg, from vc=0 at load (contract point 5): X heads
    // from 0V toward 5V and crosses vt+ = 2.7V at tau*ln(5/(5-2.7)), plus
    // one 15ns propagation delay -- `hc14_oscillator_exact_period`'s own
    // first-leg derivation, before any prior commit exists to correct for.
    let tau = (r + 62.5) * c;
    let expected_first = tau * (5.0f64 / (5.0 - 2.7)).ln() + 15e-9;
    let rel = (first_t - expected_first).abs() / expected_first;
    assert!(rel < 2e-2, "first commit at {first_t}, expected ~{expected_first} (rel {rel})");
}

// ---- findings 2/6: a closed-form read pin's watch needs a Static-supply
// eligibility rule and a re-aim when that supply moves -------------------

#[test]
fn finding2_6_closed_form_ic_input_is_watched_when_its_supply_sits_in_a_static_partition() {
    // The findings' own scenario (fixture 17-style): 0=gnd, 1=5V rail
    // (fixed), 2=vcc (free, through a closed switch -- a `Static` partition
    // holding a free IC supply, `StaticPartitionSolver::iterate_solve`'s own
    // territory, never promoted since it has no capacitor), 3=1a (RC: 10k
    // from the rail, 1uF to gnd -- its OWN ClosedForm partition, since a
    // plain read pin is never unioned with its IC's supply), 4=1y
    // (unloaded), 5..9=the other five gates' isolated dumps. A Schmitt part
    // (74hc14), not a plain 74ls04: a plain pin's dead band is a genuine
    // hash-based coin flip (`digital::floating::floating_high`), which would
    // make this test's own expected-time formula meaningless.
    let p = program(10, vec![
        source(0, 1, 0, 5.0),
        switch_dev(1, 1, 2, 1.0), // vcc power switch, CLOSED at load.
        resistor(2, 1, 3, 10_000.0),
        capacitor(3, 3, 0, 1e-6, 0.0, 0.0),
        hc14(4, 2, 0, [3, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9], [false; 12], [0; 12], 0.02, 15e-9),
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_vcc, _) = pb.local_index[2].expect("vcc is free");
    assert_eq!(pb.kinds[p_vcc], crate::partition::PartitionKind::Static, "no capacitor of its own: stays Static (contract point 3's `iterate_solve` path)");
    let (p_a, _) = pb.local_index[3].expect("1a is free");
    assert_eq!(pb.kinds[p_a], crate::partition::PartitionKind::ClosedForm, "a plain read pin is never unioned with its IC's supply");
    assert_ne!(p_vcc, p_a, "vcc and 1a must be genuinely different partitions for this to test findings 2/6 at all");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 4, 1); // 1y
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "1a at 0V (LOW) -> 1y HIGH at settle");

    // Driven purely by `next_event_time`/`advance`: if 1a's read pin were
    // never watched (findings 2/6's own bug -- `register_ic_read_pin_watches`
    // requires BOTH supply nodes literally fixed), `next_event_time` would
    // stay `+Infinity` forever and 1y would never move, since nothing else
    // in this circuit perturbs anything on its own.
    let mut guard = 0;
    loop {
        let t = kernel.next_event_time();
        assert!(t.is_finite(), "1a's crossing watch must be registered even though vcc sits in a Static, not fixed, partition (findings 2/6)");
        kernel.advance(t).unwrap();
        guard += 1;
        assert!(guard < 100, "must reach a commit well within a handful of events");
        if kernel.driver(y).committed() == Some(Mode::Low) {
            break;
        }
    }
    let tau = 10_000.0 * 1e-6;
    let expected = -tau * (1.0 - 2.7 / 5.0f64).ln() + 15e-9;
    let rel = (kernel.time() - expected).abs() / expected;
    assert!(rel < 1e-3, "1y committed LOW at {}, expected ~{expected} (rel {rel})", kernel.time());
}

#[test]
fn finding2_6_reaiming_a_static_supply_move_updates_a_scaled_thresholds_watch() {
    // Isolates the SECOND half of findings 2/6's fix (re-aiming, not merely
    // registering): a 74hc14 (CMOS_RATIO family -- its vt-/vt+ SCALE with
    // the live supply, contract's scaling formula) whose vcc is selected, by
    // an spdt, between two different FIXED sources (2.5V and 5V) -- still a
    // free, Static-partition supply, never fixed. 1a sits on its OWN RC (10k
    // from a THIRD, independent fixed 5V rail, tau=1ms), unrelated to vcc,
    // so ONLY the watched THRESHOLD moves when vcc is re-selected, never the
    // RC's own trajectory -- isolating the re-aim's effect on the threshold
    // value from any electrical coupling.
    //
    // 0=gnd, 1=5V rail (feeds 1a's RC, fixed), 2=2.5V source (fixed),
    // 3=vcc (free: spdt selects 2=2.5V initially, then 1=5V), 4=X(=1a, RC:
    // R from rail(1), C to gnd), 5=1y, 6..10=the other five gates' dumps.
    let (r, c) = (10_000.0, 100e-9);
    let p = program(11, vec![
        source(0, 1, 0, 5.0),
        source(1, 2, 0, 2.5),
        spdt(2, 3, 2, 1, 0.0), // com=vcc(3), b=2.5V(node2), c=5V(node1); sel=0 selects b.
        resistor(3, 1, 4, r),
        capacitor(4, 4, 0, c, 0.0, 0.0),
        hc14(5, 3, 0, [4, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10], [false; 12], [0; 12], 0.02, 15e-9),
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_vcc, _) = pb.local_index[3].expect("vcc is free");
    assert_eq!(pb.kinds[p_vcc], crate::partition::PartitionKind::Static, "no capacitor of its own");
    let (p_x, _) = pb.local_index[4].expect("X is free");
    assert_eq!(pb.kinds[p_x], crate::partition::PartitionKind::ClosedForm);
    assert_ne!(p_vcc, p_x);

    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
    assert!((kernel.solver().voltages()[3] - 2.5).abs() < 1e-6, "vcc must start at 2.5V (sel=0)");
    assert!(kernel.solver().ic_powered(5), "2.5V is within the hc14's power window (vcc_min=2.0)");

    let y = driver_for(&kernel, &p, 5, 1); // 1y
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "X at 0V -> 1y HIGH at settle");

    // Toggle vcc to 5V well before EITHER candidate threshold (1.35V at
    // vS=2.5, 2.7V at vS=5) so the re-aim's effect, not electrical timing
    // luck, decides which one the schedule ends up honoring. (This also
    // exercises the SAME mechanism a stale, still-pending crossing's own
    // natural re-aim would eventually self-correct through on its own --
    // `ClosedFormSolver::handle_crossing` always recomputes fresh thresholds
    // at its own end -- so this test's real bar is that the schedule ends
    // up honoring vS=5's 2.7V, not 2.5's 1.35V, regardless of exactly which
    // path got it there; `PartitionedSolver::add_watch_dependent_partitions`
    // is what makes that happen the instant the supply moves, rather than
    // only whenever some unrelated crossing next happens to fire.)
    let tau = r * c;
    let t_1_35 = tau * (5.0f64 / (5.0 - 1.35)).ln();
    let t_2_7 = tau * (5.0f64 / (5.0 - 2.7)).ln();
    assert!(t_1_35 < t_2_7);
    kernel.advance(0.1 * t_1_35).unwrap();
    kernel.solver().validate_params(&[(2, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(2, 0, 1.0)]).unwrap(); // sel=1 selects c=5V.
    assert!((kernel.solver().voltages()[3] - 5.0).abs() < 1e-6, "vcc must now read 5V");

    let mut guard = 0;
    loop {
        let t = kernel.next_event_time();
        assert!(t.is_finite());
        kernel.advance(t).unwrap();
        guard += 1;
        assert!(guard < 100);
        if kernel.driver(y).committed() == Some(Mode::Low) {
            break;
        }
    }
    let commit_t = kernel.time();
    let rel_to_2_7 = (commit_t - t_2_7).abs() / t_2_7;
    let rel_to_1_35 = (commit_t - t_1_35).abs() / t_1_35;
    assert!(
        rel_to_2_7 < 2e-2,
        "1y committed LOW at {commit_t}, expected ~{t_2_7} (the FRESH vt+=2.7V threshold after the re-aim); rel to stale 1.35V threshold ({t_1_35}) was {rel_to_1_35} -- a stale, registration-time-captured threshold would fire far too early"
    );
    assert!(rel_to_1_35 > 0.1, "the commit must NOT land near the stale 1.35V-threshold time ({t_1_35}), got {commit_t}");
}

// ---- findings 3/8: an output pin on a fixed non-datum node must load its
// free supply's partition -------------------------------------------------

#[test]
fn finding3_8_static_output_pin_on_a_fixed_node_loads_its_free_supply_and_balances_kcl() {
    // v1=5V through 20 ohm to vcc(free, Static); 1a..6a tied to gnd so 1y is
    // HIGH; 1y wired straight to v2=3V (a SECOND fixed node). Before the
    // fix, `stamp_ics_into` never considered this ic for vcc's own
    // partition at all (no driver/pull pin of its own was free-in-partition
    // there), so vcc solved as if unloaded (5V exactly) while
    // `recompute_source_currents` still billed v2 for the R_out current
    // anyway -- a KCL-violating combination.
    let p = program(9, vec![
        source(0, 1, 0, 5.0), // v1
        source(1, 3, 0, 3.0), // v2
        resistor(2, 1, 2, 20.0),
        ls04(3, 2, 0, [0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 3, 1); // 1y
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "1a LOW -> 1y HIGH");

    let vcc = kernel.solver().voltages()[2];
    // Independently verified against the real classic engine (the review's
    // own VERIFIER section): V(vcc) = 4.7630997508717305. A tolerance wide
    // enough for this core's own universal per-node shunt (absent from
    // classic) and Newton convergence, but two orders of magnitude tighter
    // than the bug's own 0.237V error (broken: exactly 5.0V, unloaded).
    let expected_vcc = 4.7630997508717305;
    let rel = (vcc - expected_vcc).abs() / expected_vcc;
    assert!(rel < 1e-3, "V(vcc) = {vcc}, expected ~{expected_vcc} (rel {rel})");

    // KCL: with only these two sources (plus negligible universal-shunt
    // noise), everything leaving v1 must return through v2.
    let currents = kernel.solver().currents();
    let (i_v1, i_v2) = (currents[0], currents[1]);
    assert!(i_v1.abs() > 1e-4, "I(v1) must be a real, nonzero loading current now that the coupling is stamped, got {i_v1}");
    assert!((i_v1 + i_v2).abs() < 1e-5, "I(v1) + I(v2) must balance (KCL), got i_v1={i_v1} i_v2={i_v2}");
}

#[test]
fn finding3_8_dynamic_output_pin_on_a_fixed_node_loads_a_stepped_free_supply() {
    // `stamp_ics_into_dyn`'s twin of the static test above (findings 3/8
    // cite both call sites): vcc free through 10 ohm plus a 1uF decoupling
    // cap (promoted to Stepped, contract point 3), 1y wired straight to a
    // SECOND fixed node (3V).
    let p = program(9, vec![
        source(0, 1, 0, 5.0),
        source(1, 3, 0, 3.0),
        resistor(2, 1, 2, 10.0),
        capacitor(3, 2, 0, 1e-6, 0.0, 0.0),
        ls04(4, 2, 0, [0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_vcc, _) = pb.local_index[2].expect("vcc is free");
    assert_eq!(pb.kinds[p_vcc], crate::partition::PartitionKind::Stepped, "a free IC supply with its own capacitor is promoted to Stepped");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 4, 1); // 1y
    // vc=0 at load (contract point 5): vcc starts at 0V, unpowered, and 1y
    // has nothing committed until vcc charges into the power window (test
    // (k)'s own pattern) -- unlike the static test above, where vcc has no
    // capacitor and is already at its steady value from the first solve.
    assert_eq!(kernel.driver(y).committed(), None, "unpowered at load: vcc starts at 0V");
    kernel.advance(500e-6).unwrap(); // 50 time constants (10 ohm * 1uF): fully settled.
    assert!(kernel.solver().ic_powered(4), "vcc must have charged into the power window by 500us");
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "1a LOW -> 1y HIGH once powered");

    let currents = kernel.solver().currents();
    let (i_v1, i_v2) = (currents[0], currents[1]);
    assert!(i_v1.abs() > 1e-4, "I(v1) must be a real, nonzero loading current, got {i_v1}");
    assert!((i_v1 + i_v2).abs() < 1e-5, "I(v1) + I(v2) must balance (KCL) once settled, got i_v1={i_v1} i_v2={i_v2}");

    let vcc = kernel.solver().voltages()[2];
    assert!(vcc < 4.99, "vcc must sag below the unloaded 5V once the pin's own loading is stamped, got {vcc}");
}

// ---- finding 9: an output on the datum node stamps nothing, draws no
// source current ------------------------------------------------------------

#[test]
fn finding9_output_on_the_datum_node_stamps_nothing_and_draws_no_source_current() {
    // 0=gnd (the datum), 1=vcc (5V fixed). 1a=gnd (LOW) -> 1y=HIGH, wired
    // straight to the datum node itself -- classic's `_stampDigitalOutput`
    // early-return case ("If the output node is below 0, it does nothing").
    let p = program(7, vec![
        source(0, 1, 0, 5.0),
        ls04(1, 1, 0, [0, 0, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 1, 1); // 1y
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "1a LOW -> 1y HIGH");

    // Without the guard, `recompute_source_currents` folds `1/R_out *
    // (V(0) - V(vcc))` into the source's KCL unconditionally for every
    // committed driver -- here that would be -5V/156.25ohm = -0.032A. With
    // it, a driver on the datum node contributes nothing, matching classic's
    // I(vsrc) (RSHUNT noise only, ~1e-11A).
    let i_vsrc = kernel.solver().currents()[0];
    assert!(i_vsrc.abs() < 1e-8, "I(vsrc) should be near zero (shunt noise only), got {i_vsrc}");
}

// ---- W1d P1: an open FLOATING_INPUT pin's pull stamps to the datum, never
// to `ic.vcc.node`/`ic.gnd.node` directly (the bug: routing its return
// current through a real supply node let it leak into that supply's own
// published source current) -----------------------------------------------

#[test]
fn w1d_p1_static_driven_open_floating_output_leaves_supply_current_clean() {
    // 74ls173 q2-q4's own shape, reproduced on the smallest device this
    // module can drive and read (`ls04_with_open_capable_output`): 1a tied
    // straight to vcc (HIGH from load, vcc fixed), so 1y (NOT gate) commits
    // LOW -- driven to gnd through R_out. 1y is ALSO open and
    // FLOATING_INPUT, with `u1:1b`'s own golden hash (digital::floating's
    // golden vector) selecting HIGH at bucket 0: the pull's target is vcc,
    // the OPPOSITE of the driven level, exactly the repro's own shape ("the
    // driver holds the pin LOW" while "the hash selects HIGH").
    //
    // Before the fix, `ic_pull_target` returned `ic.vcc.node` as the pull's
    // target, so `recompute_source_currents` folded `g*(V(1y)-V(vcc))` into
    // `leaving[vcc]` -- and vcc IS `source(0)`'s own fixed child node here,
    // so that phantom term reached `currents[0]` directly: with g=1e-6
    // (TTL) and V(1y)~0, roughly `-1e-6*(0-5) = 5e-6` A of current that
    // never actually flows through the supply. After the fix, the pull
    // returns through the datum instead, so `currents[0]` carries only the
    // universal per-node shunt's noise.
    let prefix = fnv1a("u1:1b:");
    let p = program(8, vec![
        source(0, 1, 0, 5.0), // vcc, fixed.
        ls04_with_open_capable_output(
            1,
            1,
            0,
            [1, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7],
            [false, true, false, false, false, false, false, false, false, false, false, false],
            [0, prefix, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            0.008,
            20e-9,
        ),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 1, 1); // 1y
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "1a HIGH (tied to vcc) -> 1y LOW");

    // The pin voltage is unchanged from before the fix: the conductance
    // half of the pull's stamp (`a[l,l] += g`) is identical either way, and
    // `target_v` is numerically exactly `V(vcc)`, so the injected RHS term
    // (`g*target_v` now, `g*V(vcc)` via a real node before) is identical
    // too. R_out (156.25 ohm) dominates the pull (1 Mohm), so 1y sits close
    // to LOW despite the pull's own HIGH target.
    let g_out = 1.0 / 156.25;
    let g_pull = 1e-6;
    let expected_v = g_pull * 5.0 / (g_out + g_pull);
    let v_pin = kernel.solver().voltages()[2];
    assert!((v_pin - expected_v).abs() / expected_v < 1e-2, "V(1y) = {v_pin}, expected ~{expected_v} (unaffected by the fix)");

    let i_vcc = kernel.solver().currents()[0];
    assert!(i_vcc.abs() < 1e-8, "I(vcc) must show only the universal per-node shunt's noise, no phantom pull leakage, got {i_vcc}");
}

#[test]
fn w1d_p1_dynamic_driven_open_floating_output_pull_stamps_to_the_datum() {
    // `stamp_ics_into_dyn`'s own counterpart of the static test above: a
    // capacitor on 1y's own (otherwise plain) node forces it into a
    // dynamic (ClosedForm, since both of this IC's supply nodes stay fixed
    // -- `stamp_ics_into_dyn`'s own doc comment) partition, so the pull's
    // Norton branch folds through `stamp_two_terminal`'s general
    // `g*(va-vb)+ieq` convention instead of `fold_pull`'s direct matrix
    // poke -- the OTHER stamp site this fix touches.
    let prefix = fnv1a("u1:1b:");
    let p = program(8, vec![
        source(0, 1, 0, 5.0), // vcc, fixed.
        capacitor(1, 2, 0, 1e-9, 0.0, 0.0), // forces 1y's own node dynamic.
        ls04_with_open_capable_output(
            2,
            1,
            0,
            [1, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7],
            [false, true, false, false, false, false, false, false, false, false, false, false],
            [0, prefix, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            0.008,
            20e-9,
        ),
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_pin, _) = pb.local_index[2].expect("1y's own node is free (the capacitor keeps it out of derive_fixed_nodes)");
    assert_ne!(pb.kinds[p_pin], crate::partition::PartitionKind::Static, "the capacitor must force 1y's own node into a dynamic partition");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
    let y = driver_for(&kernel, &p, 2, 1); // 1y
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "1a HIGH (tied to vcc) -> 1y LOW");

    kernel.advance(2e-5).unwrap(); // >> R_out*C's own time constant (156.25ohm * 1nF ~ 156ns).

    // Same analytic steady state as the static test above (the stamp site
    // differs, the physics does not).
    let g_out = 1.0 / 156.25;
    let g_pull = 1e-6;
    let expected_v = g_pull * 5.0 / (g_out + g_pull);
    let v_pin = kernel.solver().voltages()[2];
    assert!((v_pin - expected_v).abs() / expected_v < 1e-2, "V(1y) = {v_pin}, expected ~{expected_v}");

    let i_vcc = kernel.solver().currents()[0];
    assert!(i_vcc.abs() < 1e-6, "I(vcc) must show no phantom pull leakage once 1y's own node is dynamic either, got {i_vcc}");
}

#[test]
fn w1d_p1_undriven_open_floating_input_settles_at_pull_target_with_zero_current() {
    // A pure, non-output-capable open input (1a, plain `ls04`): this shape
    // never showed the bug (nothing else drives current through the same
    // node, so `i = g*(V(pin)-V(target))` was already exactly 0 either way
    // -- `V(pin)` settles to `target_v` exactly since the pull is the ONLY
    // thing stamped there). Checked anyway as a no-regression guard on the
    // ordinary case `ic_pull_target`'s callers exercise far more often than
    // the driven one above.
    let prefix = fnv1a("u1:1b:");
    let p = program(9, vec![
        source(0, 1, 0, 5.0),
        ls04(1, 1, 0, [2, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [true, false, false, false, false, false, false, false, false, false, false, false], [prefix, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    // `u1:1b`'s golden hash selects HIGH at bucket 0 (digital::floating's
    // own golden vector), so 1a settles at vcc (5V), not gnd.
    let v_1a = kernel.solver().voltages()[2];
    assert!((v_1a - 5.0).abs() < 1e-4, "V(1a) = {v_1a}, expected ~5V (the pull's own HIGH target, nothing else stamped on this node)");

    let i_vcc = kernel.solver().currents()[0];
    assert!(i_vcc.abs() < 1e-8, "an undriven open input's pull current must return entirely through its own IC, never the supply source, got {i_vcc}");
}

// ---- finding 7: a commit on a fixed-node driver must re-publish source
// currents at that instant --------------------------------------------------

#[test]
fn finding7_a_commit_on_a_fixed_node_driver_republishes_the_source_current_immediately() {
    // 0=gnd, 1=vcc (5V fixed, ALSO 1y's own node -- wired straight to the
    // rail), 2=1a (via an spdt: b=gnd, c=vcc). `add_driver`'s own doc
    // comment: a driver whose own node is fixed belongs to no partition, so
    // its commit alone never dirties anything for the ordinary delta cycle
    // to re-solve -- exactly the gap finding 7 describes.
    let p = program(8, vec![
        source(0, 1, 0, 5.0),
        spdt(1, 2, 0, 1, 0.0), // sel=0 selects b=gnd: 1a starts LOW.
        ls04(2, 1, 0, [2, 1, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7], [false; 12], [0; 12], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 2, 1); // 1y
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "1a LOW -> 1y HIGH (targets its own vcc rail: no extra current)");
    assert!(kernel.solver().currents()[0].abs() < 1e-8, "HIGH onto its own rail draws no extra current at settle");

    // 1a -> HIGH: 1y commits LOW after 20ns, now targeting gnd instead of
    // vcc -- a REAL current (5V/R_out) must appear in the vsrc's published
    // current exactly at that commit, not one event later.
    let t0 = kernel.time();
    kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap();
    kernel.advance(t0 + 20e-9).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "1y must have committed LOW by now");
    let i_after_low = kernel.solver().currents()[0];
    assert!((i_after_low - (-0.032)).abs() < 1e-3, "I(vsrc) must reflect the new LOW commit's R_out-to-gnd current immediately, got {i_after_low}");

    // Switch back: 1y commits HIGH again (back to its own rail), and the
    // published current must revert to ~0 at THAT SAME instant, not stay
    // stale at -0.032 until some unrelated event happens to dirty a
    // partition (finding 7's own "stays exactly one commit stale").
    let t1 = kernel.time();
    kernel.solver().validate_params(&[(1, 0, 0.0)]).unwrap();
    kernel.apply_params_as_event(&[(1, 0, 0.0)]).unwrap();
    kernel.advance(t1 + 20e-9).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "1y must have committed HIGH again by now");
    let i_after_high = kernel.solver().currents()[0];
    assert!(i_after_high.abs() < 1e-3, "I(vsrc) must revert to ~0 immediately once 1y is back on its own rail, got {i_after_high}");
}

// ---- finding 5: a stepped catch-up must integrate with the pre-event
// driver stamps, not the ones the triggering commit just installed --------

#[test]
fn finding5_stepped_catchup_integrates_with_the_driver_level_in_force_before_the_commit() {
    // 1a on a closed-form RC (5V, 10k, 1uF, tau=10ms) drives 1y, which feeds
    // a 2-cap ladder (1y --1k-- n1 --1uF-- gnd, n1 --1k-- n2 --1uF-- gnd:
    // decision 14's "two or more capacitors", naturally Stepped). A 74hc14
    // (Schmitt, deterministic crossing at vt_plus=2.7V -- a plain pin's dead
    // band is a genuine hash coin flip, useless for an analytic expected
    // time). A SINGLE `advance` call spans from load past 1a's own
    // closed-form crossing (~7.77ms) and 1y's resulting commit: the ladder's
    // own two-stage settling is slow enough that it has NOT yet reached
    // steady state by then (verified by hand: V(n1) reads ~4.7706V right at
    // the crossing, still charging), but its own step-doubling controller
    // still relaxes to larger steps once the charge stops changing much
    // from one step to the next -- leaving it idle for a while before the
    // commit lands, exactly the gap the finding describes ("bounded by
    // H_MAX=10ms"). Whatever V(n1) actually is at the crossing, the
    // PHYSICAL decay 1us later is bounded by tau=(R_out+1k)*1uF~1.06ms, at
    // most a few mV -- not the multi-volt collapse toward 0V the pre-fix bug
    // produces by integrating the WHOLE idle gap with 1y's NEW (LOW) level
    // instead of the one actually in force throughout it.
    let p = program(11, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 10_000.0),
        capacitor(2, 2, 0, 1e-6, 0.0, 0.0),
        resistor(3, 3, 4, 1_000.0),
        capacitor(4, 4, 0, 1e-6, 0.0, 0.0),
        resistor(5, 4, 5, 1_000.0),
        capacitor(6, 5, 0, 1e-6, 0.0, 0.0),
        hc14(7, 1, 0, [2, 3, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10], [false; 12], [0; 12], 0.02, 15e-9),
    ]);
    let pb = crate::partition::build(&p).unwrap();
    let (p_ladder, _) = pb.local_index[3].expect("1y is free");
    assert_eq!(pb.kinds[p_ladder], crate::partition::PartitionKind::Stepped, "two capacitors: Stepped by decision 14");
    assert_eq!(pb.local_index[4].unwrap().0, p_ladder, "n1 must share the ladder's own partition");

    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    let y = driver_for(&kernel, &p, 7, 1); // 1y
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "1a at 0V -> 1y HIGH at settle");

    let tau_rc = 10_000.0 * 1e-6;
    let t_cross = -tau_rc * (1.0 - 2.7 / 5.0f64).ln();
    let sample_at = t_cross + 15e-9 + 1e-6; // 1us past 1y's own commit.

    // ONE call: the ladder's own natural step schedule, 1a's closed-form
    // crossing, and 1y's resulting commit ALL happen inside it, with no
    // intervening horizon clamp to artificially re-sync the ladder right up
    // to the commit first (which would hide the bug).
    kernel.advance(sample_at).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "1a crossing vih must have committed 1y LOW well before {sample_at}");

    let v_n1 = kernel.solver().voltages()[4];
    // Hand-verified (by tracing the same circuit event by event): V(n1) is
    // ~4.7706V right at the crossing and decays by only a few mV over the
    // following 1us with the fix (~4.766V). The pre-fix bug instead
    // integrates the whole pre-commit idle gap with the NEW (LOW) driver,
    // landing at ~3.87V here -- comfortably below this bound.
    assert!(
        v_n1 > 4.6,
        "V(n1) = {v_n1} just 1us after 1y's commit -- physically it can only have moved a few mV from its ~4.77V value at the crossing (tau~1.06ms); a value this far below means the catch-up integrated the pre-commit gap with the NEW (LOW) driver level instead of the old one"
    );
}

// ---- finding 10/8: bo_engine_next_digital_due_time ignores every event
// kind except a live driver Commit -------------------------------------------

#[test]
fn finding8_next_digital_due_time_ignores_a_scheduled_crossing() {
    // `build()` already auto-registers X's watch through the PRODUCTION
    // `register_ic_read_pin_watches` (both supply nodes are fixed here), on
    // top of `watch_hc14_schmitt`'s own hand registration -- so, exactly
    // like test (m)'s own comment on this ("every non-open logic pin's
    // thresholds are watched in BOTH directions... re-evaluates the reader,
    // finds no actual level change yet, and immediately re-schedules"), the
    // very FIRST scheduled Crossing here is the harmless "X rising through
    // 1.6 on its way to 2.7" wake-up, not a real digital transition. The
    // SECOND is the real one (2.7 rising).
    let (p, x_node, ic_index, y_logic) = hc14_oscillator_program(false);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    watch_hc14_schmitt(&mut kernel, x_node, ic_index);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
    let y = driver_for(&kernel, &p, ic_index, y_logic);

    assert!(kernel.next_event_time().is_finite(), "a Crossing must be scheduled after settle");
    assert_eq!(kernel.next_digital_due_time(), f64::INFINITY, "no driver commit is pending yet -- only a Crossing is, which must not count");

    let t_bookkeeping = kernel.next_event_time();
    kernel.advance(t_bookkeeping).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "the 1.6-rising wake-up requests nothing: X is still below vt_plus");
    assert_eq!(kernel.next_digital_due_time(), f64::INFINITY, "a bare Crossing wake-up that requested nothing must not report a pending digital due time");

    let t_cross = kernel.next_event_time();
    kernel.advance(t_cross).unwrap();
    let due = kernel.next_digital_due_time();
    assert!(due.is_finite() && due > t_cross, "a driver commit must now be pending, after the (real) 2.7 crossing's own propagation delay");
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "not committed yet: the delay has not elapsed");
    kernel.advance(due).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "the driver commit next_digital_due_time reported must be the real one");
}

#[test]
fn finding8_next_digital_due_time_ignores_a_scheduled_step() {
    let (p, x_node, ic_index, _y_logic) = hc14_oscillator_program(true); // FORCE_STEPPED
    let (mut kernel, all_partitions, all_readers) = build(&p);
    watch_hc14_schmitt(&mut kernel, x_node, ic_index);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    assert!(kernel.next_event_time().is_finite(), "a Step must be scheduled (the stepped partition's own bookkeeping)");
    assert_eq!(kernel.next_digital_due_time(), f64::INFINITY, "a scheduled Step must not count as a pending digital due time");
}

#[test]
fn finding8_next_digital_due_time_ignores_a_scheduled_bucket() {
    // (c)'s own fixture, with 1a driven by an spdt instead of tied straight
    // to gnd, so it can be toggled to raise a genuine driver Commit.
    let prefix = fnv1a("u1:1b:");
    let p = program(9, vec![
        source(0, 1, 0, 5.0),
        spdt(1, 2, 0, 1, 0.0), // 1a selector: sel=0 selects gnd (LOW) at load.
        ls00(2, 1, 0, [2, 8, 3, 0, 0, 4, 0, 0, 5, 0, 0, 6], [false, true, false, false, false, false, false, false, false, false, false, false], [0, prefix, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0.008, 20e-9),
    ]);
    let (mut kernel, all_partitions, all_readers) = build(&p);
    kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

    assert!(kernel.next_event_time().is_finite(), "a Bucket boundary must be scheduled for the open floating input");
    assert_eq!(kernel.next_digital_due_time(), f64::INFINITY, "a scheduled Bucket must not count as a pending digital due time");

    let t0 = kernel.time();
    kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
    kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap(); // sel=1 selects vcc: 1a -> HIGH.
    let due = kernel.next_digital_due_time();
    assert!(due.is_finite(), "a real driver commit must now be pending");
    let y = driver_for(&kernel, &p, 2, 2); // 1y (NAND output).
    assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "not committed yet");
    kernel.advance(due).unwrap();
    assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "1a HIGH, 1b golden-HIGH -> NAND -> LOW");
    let _ = t0;
}




// ============================================================================
// W1e follow-up, gap 1: a watched read pin whose IC supply is free in a
// Stepped partition (core/README.md, W1c point 4 and NE555 point 5). Every
// test loads through the real `Engine`, so nothing is hand-registered, and
// every expected instant comes from an ODE integrated here at a fixed 2 us
// RK4 step (independent of the core's own controller), never from the core.
// ============================================================================

/// `(t_cross, tolerance)` for the first instant a two-state linear circuit's
/// `y[1] - k * y[0]` goes from negative to non-negative: the moment node 1 (a
/// read pin) meets a threshold that is `k` times node 0 (the supply), and the
/// time equivalent of W0's tolerance on that threshold (100 uV + 1e-3
/// relative) at the crossing slope. `kinks` are instants the circuit changes,
/// landed on exactly.
fn model_crossing(deriv: impl Fn(f64, [f64; 2]) -> [f64; 2], k: f64, kinks: &[f64]) -> (f64, f64) {
    let rk4 = |t: f64, y: [f64; 2], h: f64| -> [f64; 2] {
        let add = |a: [f64; 2], b: [f64; 2], s: f64| [a[0] + s * b[0], a[1] + s * b[1]];
        let k1 = deriv(t, y);
        let k2 = deriv(t + h / 2.0, add(y, k1, h / 2.0));
        let k3 = deriv(t + h / 2.0, add(y, k2, h / 2.0));
        let k4 = deriv(t + h, add(y, k3, h));
        [0, 1].map(|i| y[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
    };
    let (mut t, mut y) = (0.0f64, [0.0f64; 2]);
    let mut prev = y[1] - k * y[0];
    while t < 0.2 {
        let mut h = 2e-6;
        if let Some(&kink) = kinks.iter().find(|&&kk| kk > t + 1e-15 && kk < t + h) {
            h = kink - t;
        }
        y = rk4(t, y, h);
        t += h;
        let cur = y[1] - k * y[0];
        if prev < 0.0 && cur >= 0.0 {
            let t_cross = t - h * cur / (cur - prev);
            let dy = deriv(t, y);
            let slope = (dy[1] - k * dy[0]).abs();
            return (t_cross, (1e-4 + 1e-3 * k * y[0]) / slope);
        }
        prev = cur;
    }
    panic!("the model never crosses");
}

/// Advances `engine` event by event until digital slot `slot` reads `to`,
/// returning that instant, or `None` when `limit` passes first. A partition
/// that is never watched schedules only its own steps, so a missed crossing
/// shows here as `None` or a late instant, never as a hang.
fn instant_slot_reads(engine: &mut Engine, slot: usize, to: f64, limit: f64) -> Option<f64> {
    loop {
        let t = engine.next_event_time();
        if !t.is_finite() || t > limit {
            return None;
        }
        engine.advance(t).unwrap();
        if engine.digital_slots()[slot] == to {
            return Some(engine.time());
        }
    }
}

/// A `74hc14` (Schmitt, CMOS ratio: `vt+ = 2.7 * vS / 5`) on `vcc`/gnd with
/// 1a on `pin_1a` and every other input on the datum; 1y (slot 0) and the
/// other five outputs each on their own free node from `first_out` on.
fn hc14_on(ci: usize, vcc: usize, pin_1a: usize, first_out: usize) -> Device {
    hc14(ci, vcc, 0, [pin_1a, first_out, 0, first_out + 1, 0, first_out + 2, 0, first_out + 3, 0, first_out + 4, 0, first_out + 5], [false; 12], [0; 12], 0.02, 15e-9)
}

const HC14_DELAY: f64 = 15e-9;

/// Loads `p` under each integration method (`INTEGRATION` 1, trapezoidal, then
/// 0, backward Euler), lets `prepare` run any stimulus, and checks the first
/// instant `slot` reads `to`: the pin (`pin_node`) must be on a threshold
/// `k * V(supply_node)` taken from the supply as it is AT that instant, to
/// W0's tolerance (100 uV + 1e-3 relative), and the instant must match the
/// model's crossing `t_model` (the flip is `delay` after it). The first is
/// the requirement and holds under either method, because it compares the
/// core with itself; the second also carries the stepped controller's
/// integration error (README, NE555 point 5: "exact timing needs closed
/// form"), which trapezoidal keeps inside the tolerance and backward Euler,
/// at about 1% here, does not, so under it the bound is 2%.
#[allow(clippy::too_many_arguments)]
fn assert_flips_at_the_model_crossing(
    mut p: Program,
    prepare: impl Fn(&mut Engine),
    slot: usize,
    to: f64,
    delay: f64,
    (pin_node, supply_node, k): (usize, usize, f64),
    (t_model, tol): (f64, f64),
    what: &str,
) {
    for integration in [1u8, 0] {
        p.options.integration = integration;
        let mut engine = Engine::load(&p).unwrap();
        prepare(&mut engine);
        let flipped = instant_slot_reads(&mut engine, slot, to, 0.2).unwrap_or_else(|| panic!("{what} (integration {integration}): the read pin's crossing was missed"));
        let threshold = k * engine.voltages()[supply_node];
        let miss = (engine.voltages()[pin_node] - threshold).abs();
        let allowed = 1e-4 + 1e-3 * threshold;
        assert!(miss <= allowed, "{what} (integration {integration}): at the flip the pin is {miss} V from the threshold its supply gives at that instant (allowed {allowed})");
        let t_cross = flipped - delay;
        let bound = if integration == 1 { tol } else { 0.02 * t_model };
        assert!((t_cross - t_model).abs() <= bound, "{what} (integration {integration}): flipped for a crossing at {t_cross}, the model puts it at {t_model} (bound {bound})");
    }
}

#[test]
fn gap1_a_read_pin_in_the_supplys_own_stepped_partition_flips_at_the_moving_thresholds_crossing() {
    // 0=gnd, 1=5 V rail, 2=vcc (5k from the rail, 1 uF to gnd), 3=1a (20k
    // from vcc, 1 uF to gnd), 4..9 the six outputs. One partition with two
    // capacitors: Stepped. vcc is still rising when 1a meets vt+ = 0.54 vcc
    // (4.36 V at 20.7 ms), so a threshold read from the start of the step (or
    // never watched at all) lands the flip late or not at all.
    let mut p = program(10, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 5_000.0),
        capacitor(2, 2, 0, 1e-6, 0.0, 0.0),
        resistor(3, 2, 3, 20_000.0),
        capacitor(4, 3, 0, 1e-6, 0.0, 0.0),
        hc14_on(5, 2, 3, 4),
    ]);
    p.digital_slot_count = 6;
    let pb = crate::partition::build(&p).unwrap();
    assert_eq!(pb.local_index[2].unwrap().0, pb.local_index[3].unwrap().0, "vcc and 1a share a partition");
    assert_eq!(pb.kinds[pb.local_index[3].unwrap().0], crate::partition::PartitionKind::Stepped);

    let model = model_crossing(|_, y| [((5.0 - y[0]) / 5_000.0 - (y[0] - y[1]) / 20_000.0) / 1e-6, (y[0] - y[1]) / 20_000.0 / 1e-6], 0.54, &[]);
    assert_flips_at_the_model_crossing(p, |_| {}, 0, 0.0, HC14_DELAY, (3, 2, 0.54), model, "same partition, supply moving");
}

#[test]
fn gap1_a_closed_form_pin_beside_a_quiet_stepped_supply_is_promoted_and_flips_at_the_analytic_time() {
    // 1a on its own RC from the rail (10k, 1 uF; closed-form by count) while
    // vcc sits in another partition behind 10 ohm and 1 uF, settled long
    // before 1a reaches vt+ = 2.7 V at -tau ln(1 - 2.7/5) = 7.77 ms.
    let mut p = program(10, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 10.0),
        capacitor(2, 2, 0, 1e-6, 0.0, 0.0),
        resistor(3, 1, 3, 10_000.0),
        capacitor(4, 3, 0, 1e-6, 0.0, 0.0),
        hc14_on(5, 2, 3, 4),
    ]);
    p.digital_slot_count = 6;
    let pb = crate::partition::build(&p).unwrap();
    let (p_a, _) = pb.local_index[3].unwrap();
    assert_ne!(p_a, pb.local_index[2].unwrap().0, "1a and vcc are different partitions");
    assert_eq!(pb.kinds[p_a], crate::partition::PartitionKind::Stepped, "promoted from closed-form");

    let tau = 10_000.0 * 1e-6;
    let t_model = -tau * (1.0 - 2.7 / 5.0f64).ln();
    // 2.7 V is 0.54 * 5 V; the slope of 1a there is (5 - 2.7) / tau.
    let tol = (1e-4 + 1e-3 * 2.7) / ((5.0 - 2.7) / tau);
    assert_flips_at_the_model_crossing(p, |_| {}, 0, 0.0, HC14_DELAY, (3, 2, 0.54), (t_model, tol), "another partition, supply quiet");
}

#[test]
fn gap1_a_read_pin_in_another_partition_meets_a_threshold_that_is_still_moving() {
    // Same, with the supply on a 5 ms time constant and 1a on 20 ms: they
    // meet at 14.2 ms with vcc at 4.71 V and still rising 0.058 V/ms, so vt+
    // moves 0.031 V/ms against 1a's 0.12 V/ms. A threshold taken from the
    // supply's last published value is up to a whole step stale.
    let mut p = program(10, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 5_000.0),
        capacitor(2, 2, 0, 1e-6, 0.0, 0.0),
        resistor(3, 1, 3, 20_000.0),
        capacitor(4, 3, 0, 1e-6, 0.0, 0.0),
        hc14_on(5, 2, 3, 4),
    ]);
    p.digital_slot_count = 6;
    let pb = crate::partition::build(&p).unwrap();
    assert_ne!(pb.local_index[2].unwrap().0, pb.local_index[3].unwrap().0);

    let model = model_crossing(|_, y| [(5.0 - y[0]) / 5_000.0 / 1e-6, (5.0 - y[1]) / 20_000.0 / 1e-6], 0.54, &[]);
    assert_flips_at_the_model_crossing(p, |_| {}, 0, 0.0, HC14_DELAY, (3, 2, 0.54), model, "another partition, supply moving");
}

#[test]
fn gap1_a_supply_re_solve_re_aims_the_dependent_watch_to_an_earlier_crossing() {
    // A quiet supply (10 ohm and 1 uF) and 1a on 100k and 1 uF, whose steps
    // have grown long. A 10 ohm dump on vcc closes at 31.1 ms: vcc drops to
    // 2.5 V within microseconds, vt+ falls from 2.7 V to 1.35 V, and 1a meets
    // it at 31.47 ms, 0.37 ms later and inside the step its partition had
    // planned before the dump. That crossing was aimed at the old supply, so
    // 1a's partition must be re-planned when vcc is re-solved, or 1y goes LOW
    // at the end of the planned step, 1.5 ms late. The run after the dump is
    // one `advance`, because a host's `advance` ends by bringing every
    // partition to its instant and would re-plan 1a anyway.
    let mut p = program(11, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 10.0),
        capacitor(2, 2, 0, 1e-6, 0.0, 0.0),
        resistor(3, 1, 3, 100_000.0),
        capacitor(4, 3, 0, 1e-6, 0.0, 0.0),
        resistor(5, 2, 10, 10.0),
        switch_dev(6, 10, 0, 0.0),
        hc14_on(7, 2, 3, 4),
    ]);
    p.digital_slot_count = 6;
    p.options.integration = 1;
    let t_switch = 31.1e-3;
    let (t_model, tol) = model_crossing(
        |t, y| {
            let dump = if t >= t_switch { y[0] / 10.001 } else { 0.0 };
            [((5.0 - y[0]) / 10.0 - dump) / 1e-6, (5.0 - y[1]) / 100_000.0 / 1e-6]
        },
        0.54,
        &[t_switch],
    );
    assert!(t_model > t_switch, "the model crosses after the dump");
    let mut engine = Engine::load(&p).unwrap();
    engine.trace_enable(true);
    engine.advance(t_switch).unwrap();
    assert_eq!(engine.digital_slots()[0], 1.0, "1y is HIGH until 1a crosses");
    engine.set_params(&[(6, 0, 1.0)]).unwrap();
    engine.advance(0.1).unwrap();
    let flipped = engine.trace().chunks_exact(3).find(|c| c[0] > t_switch && c[1] as usize == 0 && c[2] == 0.0).map(|c| c[0]).expect("1y never went LOW");
    assert!((flipped - HC14_DELAY - t_model).abs() <= tol, "1y went LOW at {flipped}, the model puts 1a's crossing at {t_model} (tolerance {tol})");
}

/// An `ne555` with no catalog window (2-18 V): pins in the record's order,
/// `logic` in TRIG/OUT/RST/CV/THR/DIS order, digital slot base 0.
fn ne555_dev(ci: usize, vcc: usize, gnd: usize, logic: [usize; 6]) -> Device {
    let mut p = vec![0.0; ic_param::COUNT];
    p[ic_param::VCC_MIN] = 2.0;
    p[ic_param::VCC_MAX] = 18.0;
    p[ic_param::NOMINAL] = 5.0;
    let mut pins = vec![pin(vcc), pin(gnd)];
    pins.extend(logic.map(pin));
    Device { kind: kinds::NE555, component_index: ci, pins, params: p, blob: vec![0.0; 7] }
}

#[test]
fn gap1_a_555_thr_pin_in_another_partition_resets_at_the_moving_thresholds_crossing() {
    // THR on 20k/1 uF from the rail; the 555's supply behind 5k and 1 uF
    // (5 ms). RESET fires when THR meets vUpper = (2/3) vcc, at 21.4 ms with
    // vcc at 4.93 V and still rising. TRIG and RST sit on the rail, so the
    // 555 powers on with its default HIGH latch and stays there until THR.
    // OUT and DIS are on the datum, which stamps nothing (NE555 point 7): a
    // 555's idle draw (4.9 mA) through 5k would leave no powered operating
    // point, and vcc would be a plain RC the model can integrate.
    let mut p = program(6, vec![
        source(0, 1, 0, 5.0),
        resistor(1, 1, 2, 5_000.0),
        capacitor(2, 2, 0, 1e-6, 0.0, 0.0),
        resistor(3, 1, 3, 20_000.0),
        capacitor(4, 3, 0, 1e-6, 0.0, 0.0),
        ne555_dev(5, 2, 0, [1, 0, 1, 0, 3, 0]),
    ]);
    p.digital_slot_count = 1;
    let pb = crate::partition::build(&p).unwrap();
    assert_ne!(pb.local_index[2].unwrap().0, pb.local_index[3].unwrap().0);
    assert_eq!(pb.kinds[pb.local_index[3].unwrap().0], crate::partition::PartitionKind::Stepped, "THR's RC is promoted");

    let model = model_crossing(|_, y| [(5.0 - y[0]) / 5_000.0 / 1e-6, (5.0 - y[1]) / 20_000.0 / 1e-6], 2.0 / 3.0, &[]);
    assert_flips_at_the_model_crossing(p, |_| {}, 0, 0.0, 0.0, (3, 2, 2.0 / 3.0), model, "555 THR, another partition, supply moving");
}
