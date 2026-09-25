//! The production combinational-IC [`DeviceEvaluator`] (core/README.md,
//! "Kernel semantics: the digital core (W1c)") and its registration against
//! a live [`Kernel`]. Owned by W1c worker I1.
//!
//! This file adds no digital logic of its own: every IC is read and
//! evaluated straight through the step-1 functions this crate already
//! carries — `digital::power::ic_power`, `digital::levels::read`,
//! `digital::combinational::evaluate`, `digital::delay::driver_requests` —
//! exactly the way `solve::ic_tests`'s test-only `IcHarness` (step 2a)
//! already proved out end to end against a real `Kernel`/`PartitionedSolver`
//! pair. The two things a PRODUCTION evaluator needs beyond that harness are
//! here: stat 12 (IC evaluations, counted even for an instant that later
//! rolls back — core/README.md's "A failed advance": "stats keep
//! failed-attempt counts") and [`DeviceEvaluator::snapshot`]/[`restore`]
//! for the per-pin Schmitt state a failed instant must undo.
//!
//! IC output stages are not behind `devices::stamp`/`devices::current` at
//! all (`devices/mod.rs`'s own module doc) — a `Driver`'s stamp is the
//! partition solver's job (`solve::static_part`, R6b's file), not this
//! module's; this file only creates the `Driver`s and answers what they
//! should request each delta.

use crate::digital::{self, IcRecord};
use crate::drivers::Driver;
use crate::events::{DeviceId, DriverId, Mode};
use crate::kernel::{DeviceEvaluator, DriverRequest, Kernel};
use crate::program::Program;
use crate::solve::PartitionSolver;

/// One registered IC: its typed record, the driver ids for its
/// `combinational::output_capable` pins (in that same order — also this
/// IC's digital-slot order, `IcRecord::slot_base + i` for output-capable pin
/// `i`, core/README.md's "A digital slot belongs to each output-capable pin
/// in pin order"), and one Schmitt-hysteresis flag per LOGIC pin
/// (`digital::levels::read`'s `schmitt_state` — unused, never written, by a
/// non-Schmitt kind, but every pin still gets a slot so `evaluate`'s
/// `zip` stays positional and needs no per-pin "does this kind even have
/// Schmitt" branch of its own).
#[derive(Debug, Clone)]
struct IcEntry {
    device_index: usize,
    record: IcRecord,
    output_drivers: Vec<DriverId>,
    schmitt_state: Vec<bool>,
}

/// The production combinational-IC evaluator. One instance covers every IC
/// in a loaded program — `register` below builds it in one pass, alongside
/// creating each IC's output drivers, which is why the two are wired
/// together rather than split across two calls (`solve::ic_tests`'s own
/// `register_ics`/`IcHarness::new` split exists only because that test-only
/// harness is built AFTER the drivers it needs ids from already exist; this
/// module's `register` does the same thing in one function since nothing
/// else here needs the intermediate list).
#[derive(Debug)]
pub struct IcEvaluator {
    ics: Vec<IcEntry>,
    /// Stat 12 (IC evaluations, core/README.md's "Stat indices"). Bumped at
    /// the top of every [`DeviceEvaluator::evaluate`] call, powered or not —
    /// "evaluation" here is the kernel's own vocabulary (a reader actually
    /// being asked to evaluate this delta), not classic's cache-miss-only
    /// `digitalEvaluationCount`, which the contract explicitly says this
    /// need not match (point 7: "need not equal classic's
    /// digitalEvaluationCount (D27)") — this crate has no combinational
    /// signature cache to miss against. Deliberately NOT part of
    /// [`DeviceEvaluator::snapshot`]/[`restore`]: a rolled-back instant's
    /// evaluations still happened and still count, exactly like
    /// `kernel::KernelStats` itself sits outside `Kernel::advance`'s own
    /// snapshot/restore envelope.
    evaluations: u64,
}

impl IcEvaluator {
    /// The device ids `Kernel::settle`'s `all_readers` needs — every
    /// registered IC, in registration (= program device) order. `engine.rs`
    /// calls this once, right after [`register`], to build the reader list
    /// load settling requires (W1c contract point 2: "settle with every IC
    /// as a reader").
    pub fn reader_ids(&self) -> Vec<DeviceId> {
        self.ics.iter().map(|e| DeviceId(e.device_index as u32)).collect()
    }
}

impl DeviceEvaluator for IcEvaluator {
    /// Contract point 4 ("An unpowered IC reads nothing") and point 7
    /// (evaluate the truth table, map the result onto driver requests). This
    /// function adds no rules of its own: every branch below is a direct
    /// call into `digital::power`/`digital::levels`/`digital::combinational`/
    /// `digital::delay`, exactly as `solve::ic_tests::IcHarness` (the
    /// test-only sibling this mirrors) already does.
    fn evaluate(&mut self, reader: DeviceId, now: f64, _acting_on_edges: bool, voltages: &[f64]) -> Vec<DriverRequest> {
        self.evaluations += 1;
        let entry = self
            .ics
            .iter_mut()
            .find(|e| e.device_index == reader.0 as usize)
            .unwrap_or_else(|| panic!("IcEvaluator: no IC registered for reader {reader:?}"));
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

    /// One `Vec<bool>` per registered IC, in the same order as `self.ics` —
    /// every pin's Schmitt-hysteresis flag `evaluate` may have written this
    /// instant (core/README.md's "A failed advance": rolled back along with
    /// everything else the attempt touched). `evaluations` is deliberately
    /// NOT here — see its own doc comment.
    fn snapshot(&self) -> Box<dyn std::any::Any> {
        Box::new(self.ics.iter().map(|e| e.schmitt_state.clone()).collect::<Vec<Vec<bool>>>())
    }

    fn restore(&mut self, snapshot: Box<dyn std::any::Any>) {
        let states = *snapshot
            .downcast::<Vec<Vec<bool>>>()
            .expect("IcEvaluator::restore: snapshot was not produced by IcEvaluator::snapshot");
        debug_assert_eq!(states.len(), self.ics.len(), "one Schmitt-state vector per registered IC");
        for (entry, state) in self.ics.iter_mut().zip(states) {
            entry.schmitt_state = state;
        }
    }

    /// Contract point 10 ("Readouts"): a digital slot holds the pin's
    /// committed level while driven (1.0 HIGH, 0.0 LOW — an `OPEN_COLLECTOR`
    /// pin released HIGH reads exactly like a push-pull HIGH here, since its
    /// `Driver::committed()` is `Some(High)` regardless of whether the
    /// solver's own stamp then releases it or drives it — point 8's
    /// electrical distinction, not a readout one) and `NaN` while the IC is
    /// unpowered or the driver is uncommitted/reset (`None`) or mid-HIZ
    /// (`Some(Hiz)`, unreachable in practice — `digital::delay::driver_requests`
    /// never requests it — but handled rather than assumed impossible).
    ///
    /// Deliberately reads straight off `drivers`/`solver.ic_powered` with no
    /// re-evaluation of pins or truth tables: the committed level this
    /// reports is exactly what the ordinary delta cycle's `evaluate` calls
    /// already settled on this instant, so recomputing it here would be
    /// redundant (and would wrongly mutate `schmitt_state` a second time,
    /// since `digital::levels::read`'s `commit = true` write happens on
    /// every call). Every output-capable pin of every registered IC is
    /// reported every call, whether or not it actually changed — cheap at
    /// this crate's current IC counts — and left for the caller (`engine.rs`'s
    /// `Readout`) to dedupe against its own last-published value before
    /// deciding whether a trace entry is warranted, the same "raw write, the
    /// reader decides if it's a change" split `Engine::set_digital_slot`
    /// already uses for every other digital-slot writer.
    fn on_settled(&mut self, _now: f64, drivers: &[Driver], solver: &dyn PartitionSolver) -> Vec<(usize, f64)> {
        let mut out = Vec::new();
        for entry in &self.ics {
            let powered = solver.ic_powered(entry.device_index);
            for (i, &driver_id) in entry.output_drivers.iter().enumerate() {
                let slot = entry.record.slot_base + i;
                let value = if !powered {
                    f64::NAN
                } else {
                    match drivers[driver_id.0 as usize].committed() {
                        Some(Mode::High) => 1.0,
                        Some(Mode::Low) => 0.0,
                        Some(Mode::Hiz) | None => f64::NAN,
                    }
                };
                out.push((slot, value));
            }
        }
        out
    }

    fn ic_evaluations(&self) -> f64 {
        self.evaluations as f64
    }
}

/// Registers every combinational IC device in `program` against `kernel`:
/// one [`Driver`] per `combinational::output_capable` pin (contract point 2
/// wiring), created in pin order and handed `kernel.solver().partition_of_node`
/// so a driver whose own node happens to be fixed correctly gets `None`
/// (contract point 4). Returns the [`IcEvaluator`] ready to install with
/// [`Kernel::set_evaluator`] — called AFTER `Kernel::new` (a driver needs a
/// live kernel to assign an id) but BEFORE the evaluator itself is installed,
/// mirroring `solve::ic_tests::register_ics`'s identical ordering constraint
/// for the exact same reason: `DriverId`s are stable once assigned
/// (`Kernel::add_driver` never renumbers), so building the real evaluator
/// only after every driver exists is simpler than threading ids back in
/// afterward.
///
/// Generic over `S`/`E` (not the concrete `PartitionedSolver`/`IcEvaluator`
/// pair) so `engine.rs` can call this against its own boxed
/// `Kernel<Box<dyn PartitionSolver>, Box<dyn DeviceEvaluator>>` — the only
/// two things this function needs from `kernel` (`Kernel::solver`,
/// `Kernel::add_driver`) are on the trait, not the concrete type.
pub fn register<S: PartitionSolver, E: DeviceEvaluator>(kernel: &mut Kernel<S, E>, program: &Program) -> IcEvaluator {
    let mut ics = Vec::new();
    for (device_index, device) in program.devices.iter().enumerate() {
        if !digital::is_combinational_ic(device.kind) {
            continue;
        }
        let record = IcRecord::from_device(device);
        let mut output_drivers = Vec::with_capacity(digital::combinational::output_capable(record.kind).len());
        for &i in digital::combinational::output_capable(record.kind) {
            let p = &record.logic[i];
            let partition = kernel.solver().partition_of_node(p.node);
            let driver = Driver::new_ic_output(p.node, record.vcc.node, record.gnd.node, device_index, p.open_collector);
            output_drivers.push(kernel.add_driver(driver, partition));
        }
        let schmitt_state = vec![false; record.logic.len()];
        ics.push(IcEntry { device_index, record, output_drivers, schmitt_state });
    }
    IcEvaluator { ics, evaluations: 0 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::events::PartitionId;
    use crate::kinds::{self, ic_param};
    use crate::program::{Device, Options, PinRecord, PIN_FLAG_OPEN};
    use crate::solve::PartitionedSolver;

    // ---- tiny program-building helpers, matching solve::ic_tests's own
    // conventions (this file owns no shared test-helper module) ------------

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn open_pin(node: usize) -> PinRecord {
        PinRecord { node, flags: PIN_FLAG_OPEN, open_hash: 0, dead_hash: 0 }
    }

    fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }

    fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }

    fn spdt(ci: usize, com: usize, b: usize, c: usize, sel: f64) -> Device {
        Device { kind: kinds::SPDT_SWITCH, component_index: ci, pins: vec![pin(com), pin(b), pin(c)], params: vec![sel], blob: vec![] }
    }

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

    /// A `74ls04` device: `pins[i]` names logic pin `i` (`1a 1y 2a 2y ...`).
    /// `open[i]` marks it `PIN_FLAG_OPEN`.
    fn ls04(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12], open: [bool; 12], io_max: f64, delay: f64) -> Device {
        let mut device_pins = vec![pin(vcc), pin(gnd)];
        for i in 0..12 {
            device_pins.push(if open[i] { open_pin(pins[i]) } else { pin(pins[i]) });
        }
        let blob_flags: Vec<f64> = (0..12).map(|i| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }).collect();
        let mut blob = vec![0.0];
        blob.extend(blob_flags);
        Device { kind: kinds::IC_74LS04, component_index: ci, pins: device_pins, params: ls_ttl_params(io_max, delay), blob }
    }

    fn program(node_count: usize, digital_slot_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count, channel_slot_count: 0, devices }
    }

    fn build(program: &Program) -> (Kernel<PartitionedSolver, IcEvaluator>, Vec<PartitionId>, Vec<DeviceId>) {
        let solver = PartitionedSolver::build(program).expect("program must load");
        let mut kernel = Kernel::new(solver, IcEvaluator { ics: Vec::new(), evaluations: 0 });
        let evaluator = register(&mut kernel, program);
        let all_readers = evaluator.reader_ids();
        let all_partitions: Vec<PartitionId> = (0..kernel.solver().partition_count() as u32).map(PartitionId).collect();
        kernel.set_evaluator(evaluator);
        (kernel, all_partitions, all_readers)
    }

    #[test]
    fn settle_publishes_every_output_capable_pins_slot_with_the_committed_level() {
        // 0=gnd, 1=vcc (5V fixed), 2=1a (through 1k to gnd -> LOW -> 1y HIGH),
        // 3=1y (slot 1, isolated).
        let p = program(9, 6, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 2, 0, 1_000.0),
            ls04(2, 1, 0, [2, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], 0.008, 20e-9),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

        let writes = kernel.take_digital_writes();
        // slot_base is 0 (the only IC in this program); slot 1 = "1y" (the
        // second output-capable pin position, index 1 of OUT_HEX2).
        let slot1 = writes.iter().rev().find(|&&(_, slot, _)| slot == 1).expect("slot 1 (1y) must have been published");
        assert_eq!(slot1.2, 1.0, "1a pulled LOW -> 1y committed HIGH -> slot reads 1.0");
        // `Kernel::settle`'s own three-step process (frozen fixed point,
        // then exactly one edge pass) evaluates every reader at least once,
        // possibly several times across the fixed-point's own passes — the
        // load-bearing claim here is just "at least one", not an exact count
        // this test has no business pinning to `settle`'s internals.
        assert!(kernel.evaluator().ic_evaluations() >= 1.0, "settle must have evaluated the one reader at least once");
    }

    #[test]
    fn an_unpowered_ic_publishes_nan_on_every_slot() {
        // 0=gnd, 1=1y (isolated) — vcc (node 2) left floating/unpowered: no
        // source ties it to anything, so `ic_power` reads it as OPEN.
        let p = program(9, 6, vec![ls04(0, 2, 0, [3, 1, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8], [false; 12], 0.008, 20e-9)]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

        let writes = kernel.take_digital_writes();
        let slot1 = writes.iter().rev().find(|&&(_, slot, _)| slot == 1).expect("slot 1 (1y) must have been published");
        assert!(slot1.2.is_nan(), "unpowered IC must publish NaN, got {}", slot1.2);
    }

    #[test]
    fn a_livelocked_instant_still_counts_its_evaluations_but_queues_no_digital_writes() {
        // A 3-gate ring, zero propagation delay, armed by its own spdt: with
        // sel=0 the loop is broken (com reads a deterministic LOW through
        // `b`), so the ring settles quietly like any other gate; flipping
        // sel to 1 selects `c` (fed back from the ring's own last output),
        // closing an odd inversion loop that can never reach a quiet fixed
        // point — the same shape solve::ic_tests's g1/g2 use to force
        // `apply_params_as_event` to fail on demand.
        // 0=gnd, 1=vcc (5V fixed), 2=ring com/1a, 3=y1(=2a), 4=y2(=3a),
        // 5=y3, 6=dump for the other 3 gates.
        let p = program(9, 6, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 5, 0.0), // sel=0 selects b=gnd: loop broken at load.
            ls04(2, 1, 0, [2, 3, 3, 4, 4, 5, 0, 6, 0, 6, 0, 6], [false; 12], 0.008, 0.0),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        kernel.take_digital_writes(); // drain the load-time settle's own writes.
        let evaluations_before = kernel.evaluator().ic_evaluations();
        assert!(evaluations_before >= 1.0, "load settling must have evaluated the ring at least once");

        kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
        let err = kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap_err();
        assert!(matches!(err, crate::kernel::KernelError::Livelock { .. }), "the closed ring must never settle: {err:?}");

        // "Stats keep failed-attempt counts" (core/README.md, "A failed
        // advance"): the livelocked attempt's own evaluate() calls still
        // ran and still count, even though the whole instant rolled back.
        assert!(kernel.evaluator().ic_evaluations() > evaluations_before, "a livelocked instant's evaluations must not be rolled back");
        // But `publish_settled` only ever runs after an instant that
        // reached a quiet fixed point (kernel.rs's own doc comment on
        // `pending_digital_writes`) — this one never did, so nothing was
        // ever queued for it.
        assert!(kernel.take_digital_writes().is_empty(), "a livelocked instant must publish no digital-slot writes at all");
    }
}
