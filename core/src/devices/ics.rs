//! The production IC [`DeviceEvaluator`] (core/README.md, "Kernel semantics:
//! the digital core (W1c)" / "...: sequential ICs (W1d)" / "...: memories
//! (W1d)") and its registration against a live [`Kernel`]. W1c combinational
//! dispatch owned by worker I1; the W1d family dispatch (this file's own
//! `Family` enum, the sequential/memory branches of `evaluate`/`on_settled`/
//! `snapshot`/`restore`/`seed_memory`/`take_eeprom_updates`, and widening
//! `register` to every IC kind) is W1d worker K1's kernel-infrastructure
//! wiring — the real per-kind register/memory logic behind
//! `digital::sequential`/`digital::memory` is D1/D2's.
//!
//! This file adds no digital logic of its own for the combinational path:
//! every combinational IC is read and evaluated straight through the step-1
//! functions this crate already carries — `digital::power::ic_power`,
//! `digital::levels::read`, `digital::combinational::evaluate`,
//! `digital::delay::driver_requests` — exactly the way `solve::ic_tests`'s
//! test-only `IcHarness` (step 2a) already proved out end to end against a
//! real `Kernel`/`PartitionedSolver` pair. For a sequential or memory IC, the
//! read/power steps are identical but the truth table is replaced by a call
//! into that IC's own family evaluator (`digital::sequential::evaluate` /
//! `digital::memory::evaluate`), which returns a [`digital::FamilyOutput`]
//! this file maps onto `DriverRequest`s and digital-slot writes instead of
//! `digital::delay::driver_requests`'s job for the combinational family.
//!
//! IC output stages are not behind `devices::stamp`/`devices::current` at
//! all (`devices/mod.rs`'s own module doc) — a `Driver`'s stamp is the
//! partition solver's job (`solve::static_part`, R6b's file), not this
//! module's; this file only creates the `Driver`s and answers what they
//! should request each delta.

use crate::digital::{self, IcRecord};
use crate::drivers::{DelaySet, Driver, DriverStage};
use crate::events::{DeviceId, DriverId, Mode};
use crate::kernel::{DeviceEvaluator, DriverRequest, Kernel};
use crate::kinds;
use crate::program::{Device, Program};
use crate::solve::PartitionSolver;

/// Which family of evaluator an IC's own state belongs to (core/README.md's
/// W1d dispatcher): combinational keeps its per-LOGIC-pin Schmitt state
/// (unchanged from W1c); sequential and memory each carry one state struct
/// per device, built once at registration from the device's own record (a
/// memory's also from its ROM words, when it has any).
#[derive(Debug, Clone)]
enum Family {
    Combinational { schmitt_state: Vec<bool> },
    Sequential(digital::sequential::SeqState),
    Memory(digital::memory::MemState),
    /// NE555 point 1: its own family, one latch bit per device. Dispatched
    /// specially in `evaluate`/`on_settled` (this file's own doc comment) --
    /// it shares `IcEntry::family_slots`' single-slot publish path with
    /// Sequential/Memory, but reads no logic level and writes no
    /// `last_reads` at all.
    Ne555(digital::ne555::Ne555State),
}

/// One registered IC: its typed record, the driver ids for its
/// `digital::output_capable` pins (in that same order — also this IC's
/// digital-slot order for the COMBINATIONAL family only, `IcRecord::slot_base
/// + i` for output-capable pin `i`, core/README.md's "A digital slot belongs
/// to each output-capable pin in pin order"; a sequential/memory IC's slots
/// use a DIFFERENT indexing — see `family_slots`' own doc comment), and its
/// family state.
#[derive(Debug, Clone)]
struct IcEntry {
    device_index: usize,
    record: IcRecord,
    output_drivers: Vec<DriverId>,
    family: Family,
    /// This IC's logic-pin reads at its own previous POWERED evaluation
    /// (W1d contract, delays point 3 / sequential point 4 / memory point 7):
    /// `None` before the first one. Kept for every family, not only
    /// sequential/memory: the combinational delay tail (the 74ls157's
    /// `strobe`, the 74ls245's `oeEnable`/`oeDisable`) uses the identical
    /// cause-detection rule, read from here by `evaluate`'s combinational
    /// arm and passed into `digital::delay::driver_requests`.
    last_reads: Option<Vec<bool>>,
    /// Sequential/memory only: this device's digital-slot readout from its
    /// LAST evaluation (`digital::FamilyOutput::slots`), published by
    /// `on_settled` verbatim (sequential contract point 8 / memories point
    /// 6: "a slot holds the evaluator's view at its latest evaluation, not
    /// the committed pin level"). Sized to the kind's own digital-slot count
    /// at registration (`kinds::shape(kind).digital_slots`) — a DIFFERENT
    /// count, and a different meaning per index, from `output_drivers.len()`
    /// (a 161 has 5 output-capable pins — `qa..qd rco` — but 5 slots keyed
    /// `<id> qa qb qc qd`: the bare count has no driver of its own, and
    /// `rco` has no slot at all). Left empty (and unused) for a
    /// combinational entry, whose slots instead come from driver levels,
    /// exactly as W1c already did.
    family_slots: Vec<f64>,
}

/// The production IC evaluator. One instance covers every IC in a loaded
/// program — `register` below builds it in one pass, alongside creating
/// each IC's output drivers, which is why the two are wired together rather
/// than split across two calls (`solve::ic_tests`'s own `register_ics`/
/// `IcHarness::new` split exists only because that test-only harness is
/// built AFTER the drivers it needs ids from already exist; this module's
/// `register` does the same thing in one function since nothing else here
/// needs the intermediate list).
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

/// Maps one family evaluator's [`digital::FamilyOutput`] onto the
/// `DriverRequest`s the kernel applies: one `Level` per output-capable pin
/// driver (in the same order `out.requests` carries — the interface's own
/// contract) and one `Timer` per pending deadline the evaluation scheduled
/// (a memory's pending-write completion). The W1d counterpart of
/// `digital::delay::driver_requests` for the combinational family — like
/// that function since W1d point 4 ("no IC requests a reset any more"),
/// there is no `None`/`Reset` arm here at all: every entry in `out.requests`
/// is already a concrete `Mode` (`Hiz` included).
fn family_requests(device_index: usize, output_drivers: &[DriverId], out: &digital::FamilyOutput) -> Vec<DriverRequest> {
    debug_assert_eq!(output_drivers.len(), out.requests.len(), "one driver per output-capable pin, in the same order");
    let mut reqs: Vec<DriverRequest> = output_drivers
        .iter()
        .zip(out.requests.iter())
        .map(|(&driver, &(level, delays))| DriverRequest::Level { driver, level, delays })
        .collect();
    reqs.extend(out.timers.iter().map(|&(at, tag)| DriverRequest::Timer { device: DeviceId(device_index as u32), at, tag }));
    reqs
}

/// A memory device's ROM image, decoded from its blob tail (core/README.md,
/// "ROM image": blob is `[slotBase, pinFlags x L, word x W]`) — empty for a
/// 74ls189 (no ROM: `kinds::eeprom_words` is 0 for that kind).
fn rom_words(device: &Device, kind: u32, logic_len: usize) -> Vec<u32> {
    let words_from = 1 + logic_len;
    let words_count = kinds::eeprom_words(kind);
    device.blob[words_from..words_from + words_count].iter().map(|&w| w as u32).collect()
}

impl DeviceEvaluator for IcEvaluator {
    /// Contract point 4 ("An unpowered IC reads nothing") and point 7
    /// (evaluate the truth table, map the result onto driver requests) for
    /// the combinational family; W1d sequential point 3 / memories points
    /// 3-4 for the other two. Every branch below is a direct call into
    /// `digital::power`/`digital::levels` plus, per family,
    /// `digital::combinational`+`digital::delay` or
    /// `digital::sequential`/`digital::memory` — no digital logic lives in
    /// this function itself.
    fn evaluate(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64]) -> Vec<DriverRequest> {
        self.evaluations += 1;
        let entry = self
            .ics
            .iter_mut()
            .find(|e| e.device_index == reader.0 as usize)
            .unwrap_or_else(|| panic!("IcEvaluator: no IC registered for reader {reader:?}"));
        let ic = &entry.record;

        // NE555 point 1: dispatched before any `levels::read`/power-gate/
        // `last_reads` bookkeeping the other three families share below --
        // it has no logic-pin read concept of its own ("no logic level...
        // it publishes its latch instead"). `digital::ne555::evaluate`
        // already re-derives power itself (`None` while unpowered), so this
        // branch does its own power gate rather than sharing the one below.
        if let Family::Ne555(state) = &mut entry.family {
            return match digital::ne555::evaluate(ic, state, voltages, acting_on_edges) {
                None => {
                    // Point 10: unpowered -- the slot goes NaN and nothing is
                    // requested (drivers keep their frozen committed level
                    // through the kernel's own power-loss hold).
                    for s in entry.family_slots.iter_mut() {
                        *s = f64::NAN;
                    }
                    Vec::new()
                }
                Some(out_high) => {
                    // Point 12: the slot holds `outHigh` at every completed
                    // instant while powered.
                    entry.family_slots[0] = if out_high { 1.0 } else { 0.0 };
                    // Point 4: OUT HIGH/LOW as the result, DIS LOW when the
                    // result is 0 or HIGH (released) when it is 1 -- the SAME
                    // `Mode` for both drivers -- with DelaySet::uniform(0):
                    // no output lag, no propagation delay.
                    let level = if out_high { Mode::High } else { Mode::Low };
                    let delays = DelaySet::uniform(0.0);
                    vec![
                        DriverRequest::Level { driver: entry.output_drivers[0], level, delays },
                        DriverRequest::Level { driver: entry.output_drivers[1], level, delays },
                    ]
                }
            };
        }

        let power = digital::power::ic_power(ic, voltages[ic.vcc.node], voltages[ic.gnd.node]);
        if !power.powered {
            // W1c contract point 9 / W1d sequential point 3 ("an unpowered
            // device reads, writes and requests nothing, so its state and
            // lastClk freeze") / memories points 3-4: a sequential/memory
            // slot goes NaN while unpowered. The combinational path already
            // returned nothing here before W1d and still does — its slots
            // come from driver levels at `on_settled` time, not from a
            // stored readout, so there is nothing of its own to blank here.
            if !matches!(entry.family, Family::Combinational { .. }) {
                for s in entry.family_slots.iter_mut() {
                    *s = f64::NAN;
                }
            }
            return Vec::new();
        }

        // Every logic pin's read, in pin order (W1c contract point 4).
        // Schmitt hysteresis state only exists for the combinational
        // family (`kinds::seq_check` rejects `schmitt = 1` on every W1d
        // kind), so a sequential/memory read uses a throwaway per-pin
        // scratch slot instead of a stored one.
        let levels: Vec<bool> = if let Family::Combinational { schmitt_state } = &mut entry.family {
            ic.logic
                .iter()
                .zip(schmitt_state.iter_mut())
                .map(|(p, st)| digital::levels::read(ic, &power, p, voltages[p.node], now, st))
                .collect()
        } else {
            ic.logic
                .iter()
                .map(|p| {
                    let mut scratch = false;
                    digital::levels::read(ic, &power, p, voltages[p.node], now, &mut scratch)
                })
                .collect()
        };

        // "lastReads is written at every powered evaluation, frozen-settle
        // passes included" (W1d delays point 3) — for every family, not
        // only sequential/memory (this entry's own doc comment on the
        // field).
        let last_reads = entry.last_reads.clone();
        entry.last_reads = Some(levels.clone());

        match &mut entry.family {
            Family::Combinational { .. } => {
                let result = digital::combinational::evaluate(ic.kind, &levels);
                digital::delay::driver_requests(ic, &entry.output_drivers, &result, &levels, last_reads.as_deref())
            }
            Family::Sequential(state) => {
                let out = digital::sequential::evaluate(ic, state, &levels, last_reads.as_deref(), acting_on_edges, now);
                for (i, v) in entry.family_slots.iter_mut().enumerate() {
                    *v = out.slots.get(i).copied().unwrap_or(f64::NAN);
                }
                family_requests(entry.device_index, &entry.output_drivers, &out)
            }
            Family::Memory(state) => {
                let out = digital::memory::evaluate(ic, state, &levels, last_reads.as_deref(), acting_on_edges, now);
                for (i, v) in entry.family_slots.iter_mut().enumerate() {
                    *v = out.slots.get(i).copied().unwrap_or(f64::NAN);
                }
                family_requests(entry.device_index, &entry.output_drivers, &out)
            }
            // Returned above, before any read or `last_reads` write.
            Family::Ne555(_) => unreachable!("a 555 is dispatched at the top of `evaluate`"),
        }
    }

    /// Every piece of per-device mutable state `evaluate` (or `seed_memory`)
    /// may have written this instant — the family state, `last_reads` and
    /// `family_slots`, one tuple per registered IC in `self.ics` order
    /// (core/README.md's "A failed advance": rolled back along with
    /// everything else the attempt touched). `evaluations` is deliberately
    /// NOT here — see its own doc comment.
    fn snapshot(&self) -> Box<dyn std::any::Any> {
        Box::new(self.ics.iter().map(|e| (e.family.clone(), e.last_reads.clone(), e.family_slots.clone())).collect::<Vec<(Family, Option<Vec<bool>>, Vec<f64>)>>())
    }

    fn restore(&mut self, snapshot: Box<dyn std::any::Any>) {
        let states = *snapshot
            .downcast::<Vec<(Family, Option<Vec<bool>>, Vec<f64>)>>()
            .expect("IcEvaluator::restore: snapshot was not produced by IcEvaluator::snapshot");
        debug_assert_eq!(states.len(), self.ics.len(), "one state tuple per registered IC");
        for (entry, (family, last_reads, family_slots)) in self.ics.iter_mut().zip(states) {
            entry.family = family;
            entry.last_reads = last_reads;
            entry.family_slots = family_slots;
        }
    }

    /// Contract point 10 ("Readouts") for the combinational family: a
    /// digital slot holds the pin's committed level while driven (1.0 HIGH,
    /// 0.0 LOW — an `OPEN_COLLECTOR` pin released HIGH reads exactly like a
    /// push-pull HIGH here, since its `Driver::committed()` is `Some(High)`
    /// regardless of whether the solver's own stamp then releases it or
    /// drives it — point 8's electrical distinction, not a readout one) and
    /// `NaN` while the IC is unpowered or the driver is uncommitted/reset
    /// (`None`) or mid-HIZ (`Some(Hiz)`, unreachable in practice —
    /// `digital::delay::driver_requests` never requests it — but handled
    /// rather than assumed impossible).
    ///
    /// W1d sequential point 8 / memories point 6 for the other two
    /// families: a slot holds the evaluator's OWN view from its latest
    /// evaluation (`family_slots`, already NaN while unpowered — see
    /// `Self::evaluate`'s own doc comment), not a driver's committed level
    /// at all — `powered` is still re-checked here too, defensively, for
    /// parity with the combinational branch's identical belt-and-suspenders
    /// read of live power state rather than trusting a value computed
    /// earlier in the same instant.
    fn on_settled(&mut self, _now: f64, drivers: &[Driver], solver: &dyn PartitionSolver) -> Vec<(usize, f64)> {
        let mut out = Vec::new();
        for entry in &self.ics {
            let powered = solver.ic_powered(entry.device_index);
            match &entry.family {
                Family::Combinational { .. } => {
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
                Family::Sequential(_) | Family::Memory(_) | Family::Ne555(_) => {
                    for (i, &value) in entry.family_slots.iter().enumerate() {
                        out.push((entry.record.slot_base + i, if powered { value } else { f64::NAN }));
                    }
                }
            }
        }
        out
    }

    fn ic_evaluations(&self) -> f64 {
        self.evaluations as f64
    }

    /// `bo_engine_seed_memory`: routes to `digital::memory::seed` for a
    /// registered 74ls189, declines every other kind (a wrong device index
    /// is `Err` too — `Engine::seed_memory` already checks the index is in
    /// range before this is ever called, so a miss here only means the
    /// device at that index is not one `register` gave a `Family::Memory`
    /// state at all, which cannot happen for a valid 74ls189 but is handled
    /// rather than assumed).
    fn seed_memory(&mut self, device: usize, words: &[f64]) -> Result<(), String> {
        let entry = match self.ics.iter_mut().find(|e| e.device_index == device) {
            Some(e) => e,
            None => return Err(format!("seed_memory: no IC registered for device {device}")),
        };
        if entry.record.kind != kinds::IC_74LS189 {
            return Err("seed_memory: not a memory".to_string());
        }
        let record = entry.record.clone();
        match &mut entry.family {
            Family::Memory(state) => digital::memory::seed(&record, state, words),
            _ => Err("seed_memory: not a memory".to_string()),
        }
    }

    /// `bo_engine_take_eeprom_updates`: collects and clears every dirty
    /// 28c16/28c256 image (`digital::memory::take_update`), packed
    /// `deviceIndex, W, word x W` per dirty EEPROM, in device (registration)
    /// order. A 74ls189 is a `Family::Memory` too but is never a source
    /// here — `digital::memory::take_update`'s own state never marks one
    /// dirty (only a completed EEPROM write does, memories section point 5),
    /// so calling it uniformly for every memory entry costs nothing and
    /// needs no kind check of its own.
    fn take_eeprom_updates(&mut self) -> Vec<f64> {
        let mut out = Vec::new();
        for entry in &mut self.ics {
            if let Family::Memory(state) = &mut entry.family {
                if let Some(words) = digital::memory::take_update(state) {
                    out.push(entry.device_index as f64);
                    out.push(words.len() as f64);
                    out.extend(words.iter().map(|&w| w as f64));
                }
            }
        }
        out
    }
}

/// Registers every IC device in `program` against `kernel` — combinational,
/// sequential and memory alike (W1d widened this from combinational-only):
/// one [`Driver`] per `digital::output_capable` pin (contract point 2
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
        if !digital::is_ic(device.kind) {
            continue;
        }
        let record = IcRecord::from_device(device);
        let outputs = digital::output_capable(record.kind);
        let mut output_drivers = Vec::with_capacity(outputs.len());
        for &i in outputs {
            let p = &record.logic[i];
            let partition = kernel.solver().partition_of_node(p.node);
            let mut driver = Driver::new_ic_output(p.node, record.vcc.node, record.gnd.node, device_index, p.open_collector);
            // W1a point 2 (amended): a 555's OUT/DIS pin carries its own
            // stage instead of the default `Logic` one, so
            // `solve::ic_driver_branches` dispatches to
            // `digital::ne555::branches` for it (NE555 point 7). `outputs`
            // is `digital::NE555_OUT, digital::NE555_DIS` in that order for
            // this kind (`output_capable`'s own table), so `i` names exactly
            // which stage applies.
            if digital::is_ne555(record.kind) {
                driver.stage = if i == digital::NE555_OUT { DriverStage::Ne555Out } else { DriverStage::Ne555Dis };
            }
            output_drivers.push(kernel.add_driver(driver, partition));
        }

        let slot_count = kinds::shape(record.kind).map_or(0, |s| (s.digital_slots)(record.logic.len() + 2));
        let family = if digital::is_sequential_ic(record.kind) {
            Family::Sequential(digital::sequential::SeqState::power_on(&record))
        } else if digital::is_memory_ic(record.kind) {
            let rom = rom_words(device, record.kind, record.logic.len());
            Family::Memory(digital::memory::MemState::power_on(&record, &rom))
        } else if digital::is_ne555(record.kind) {
            Family::Ne555(digital::ne555::Ne555State::power_on())
        } else {
            Family::Combinational { schmitt_state: vec![false; record.logic.len()] }
        };
        let family_slots = vec![f64::NAN; slot_count];

        ics.push(IcEntry { device_index, record, output_drivers, family, last_reads: None, family_slots });
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

    /// A W1d sequential/memory device with every logic pin tied to `gnd`
    /// (node 0) and no ROM contents (a 74ls161/173/74hc74/189) or an
    /// all-0xFF image (an EEPROM, `atob("")`'s own fallback) — this
    /// module's own dispatcher-wiring tests need nothing more specific than
    /// "loads, is powered, and every family evaluator is reachable".
    /// `logic_len` is the kind's own `L` (README's sequential/memory pin
    /// table); `params` is the kind's full param vector (13 IC params, the
    /// delay tail, and, for an EEPROM, `tWC`/`sdp`).
    fn w1d_ic(kind: u32, ci: usize, vcc: usize, gnd: usize, logic_len: usize, params: Vec<f64>, rom_words: usize) -> Device {
        let device_pins = std::iter::once(pin(vcc)).chain(std::iter::once(pin(gnd))).chain((0..logic_len).map(|_| pin(gnd))).collect();
        let mut blob = vec![0.0]; // slotBase
        blob.extend(std::iter::repeat(0.0).take(logic_len)); // pin flags, all clear
        blob.extend(std::iter::repeat(0.0).take(rom_words)); // ROM words (0xFFFFFFFF-free stub image)
        Device { kind, component_index: ci, pins: device_pins, params, blob }
    }

    fn w1d_params(delay_pairs: usize, extra: &[f64]) -> Vec<f64> {
        let mut p = vec![0.0; ic_param::COUNT];
        p[ic_param::VCC_MIN] = 4.5;
        p[ic_param::VCC_MAX] = 5.5;
        p[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
        p[ic_param::VIL_FIXED] = 1.0;
        p[ic_param::VIL] = 0.8;
        p[ic_param::VIH_FIXED] = 1.0;
        p[ic_param::VIH] = 2.0;
        p[ic_param::NOMINAL] = 5.0;
        p.extend(std::iter::repeat(20e-9).take(2 * delay_pairs));
        p.extend_from_slice(extra);
        p
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

    // ---- W1d: the dispatcher end to end, one of each new kind ---------------

    /// `device` with its digital slots starting at `base` (blob[0]).
    fn at_slot(mut device: Device, base: usize) -> Device {
        device.blob[0] = base as f64;
        device
    }

    #[test]
    fn a_program_holding_one_of_each_w1d_kind_publishes_each_familys_power_on_readout() {
        // Every logic pin ties to gnd and every device is powered from one
        // 5 V source, so every family evaluator is reached (not the
        // unpowered NaN branch). With every pin LOW: the 161's /clr clears
        // (count 0), the 173 holds its power-on 0, the 74hc74's /PRE and /CLR
        // are both asserted (Q = Q-bar = 1 on both flip-flops), the 189 is
        // selected in write mode showing mem[0] = 0xF inverted (all 0), and
        // the 28c16 is in read mode showing byte 0 of an all-zero image.
        let p = program(2, 25, vec![
            source(0, 1, 0, 5.0),
            at_slot(w1d_ic(kinds::IC_74LS161, 1, 1, 0, 14, w1d_params(4, &[]), 0), 0),
            at_slot(w1d_ic(kinds::IC_74LS173, 2, 1, 0, 14, w1d_params(4, &[]), 0), 5),
            at_slot(w1d_ic(kinds::IC_74HC74, 3, 1, 0, 12, w1d_params(2, &[]), 0), 9),
            at_slot(w1d_ic(kinds::IC_74LS189, 4, 1, 0, 14, w1d_params(5, &[]), 0), 13),
            at_slot(w1d_ic(kinds::IC_28C16, 5, 1, 0, 22, w1d_params(5, &[10e-3, 0.0]), 2048 / 4), 17),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        assert_eq!(all_readers.len(), 5, "every W1d kind must register as a reader, exactly like a W1c IC");
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

        let mut slots = vec![f64::NAN; 25];
        for (_, slot, value) in kernel.take_digital_writes() {
            slots[slot] = value;
        }
        let expected = [
            0.0, 0.0, 0.0, 0.0, 0.0, // 161: count, qa..qd
            0.0, 0.0, 0.0, 0.0, // 173: q1..q4
            1.0, 1.0, 1.0, 1.0, // 74hc74: q1 q1_n q2 q2_n
            0.0, 0.0, 0.0, 0.0, // 189: /o1../o4
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 28c16: io0..io7
        ];
        assert_eq!(slots, expected);
        assert!(kernel.evaluator().ic_evaluations() >= 5.0, "settle must have evaluated every one of the 5 new readers at least once");
    }

    #[test]
    fn seed_memory_declines_a_non_189_kind_and_seeds_a_real_189() {
        let p = program(2, 20, vec![
            source(0, 1, 0, 5.0),
            w1d_ic(kinds::IC_74HC74, 1, 1, 0, 12, w1d_params(2, &[]), 0),
            at_slot(w1d_ic(kinds::IC_74LS189, 2, 1, 0, 14, w1d_params(5, &[]), 0), 4),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

        let wrong_kind = kernel.evaluator_mut().seed_memory(1, &[0.0; 16]).unwrap_err();
        assert!(wrong_kind.contains("not a memory"), "a 74hc74 must never reach digital::memory::seed: {wrong_kind}");
        assert!(kernel.evaluator_mut().seed_memory(2, &[16.0; 16]).is_err(), "a nibble outside [0, 15] must be refused");
        kernel.evaluator_mut().seed_memory(2, &[0.0; 16]).expect("a real 74ls189 must accept a valid seed");
    }

    #[test]
    fn take_eeprom_updates_returns_empty_with_no_dirty_eeprom() {
        let p = program(2, 20, vec![
            source(0, 1, 0, 5.0),
            w1d_ic(kinds::IC_28C16, 1, 1, 0, 22, w1d_params(5, &[10e-3, 0.0]), 2048 / 4),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();

        let updates = kernel.evaluator_mut().take_eeprom_updates();
        assert!(updates.is_empty(), "no write has completed, so no EEPROM is dirty");
    }
}
