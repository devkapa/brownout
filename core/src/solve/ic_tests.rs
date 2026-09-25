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

use super::{PartitionSolver, PartitionedSolver};
use crate::digital::{self, IcPin, IcRecord};
use crate::drivers::Driver;
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
