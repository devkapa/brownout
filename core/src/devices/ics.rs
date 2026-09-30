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
#[derive(Debug)]
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

impl Clone for Family {
    fn clone(&self) -> Self {
        match self {
            Family::Combinational { schmitt_state } => Family::Combinational { schmitt_state: schmitt_state.clone() },
            Family::Sequential(state) => Family::Sequential(state.clone()),
            Family::Memory(state) => Family::Memory(state.clone()),
            Family::Ne555(state) => Family::Ne555(state.clone()),
        }
    }

    /// Reuses the Schmitt vector's storage when both sides are combinational,
    /// which is what lets the rollback journal save a pre-image without
    /// allocating once its pool has warmed up.
    fn clone_from(&mut self, source: &Self) {
        match (&mut *self, source) {
            (Family::Combinational { schmitt_state: to }, Family::Combinational { schmitt_state: from }) => to.clone_from(from),
            _ => *self = source.clone(),
        }
    }
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
    /// Whether the open rollback window's journal already holds this IC's
    /// pre-image.
    journaled: bool,
    /// The bits of every digital slot this IC owns (`record.slot_base + i`)
    /// as `on_settled` last handed them to the kernel; see
    /// [`IcEvaluator::on_settled`].
    published: Vec<u64>,
}

/// One IC's pre-image, taken before the first thing an instant wrote to it:
/// exactly the state `snapshot` used to clone for every IC at every instant.
#[derive(Debug)]
struct Saved {
    at: usize,
    family: Family,
    last_reads: Option<Vec<bool>>,
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
    /// `device_index -> position in `ics``, `u32::MAX` for a device that is
    /// not a registered IC: `evaluate` used to find its entry by scanning
    /// every IC on every call.
    position_of_device: Vec<u32>,
    /// Whether a rollback window is open (`begin_instant`..`end_instant`/
    /// `rollback_instant`): only then does a write to an IC save its
    /// pre-image first.
    journaling: bool,
    /// The pre-images of the ICs written in the open window, one each
    /// (`IcEntry::journaled`).
    journal: Vec<Saved>,
    /// Pre-image records from finished windows, kept with their storage so
    /// the next window's saves allocate nothing.
    spare: Vec<Saved>,
    /// Scratch for one evaluation's pin reads.
    levels: Vec<bool>,
    /// Whether `on_settled` has published yet: its first call hands the
    /// kernel every slot, later ones only what changed.
    published_once: bool,
}

impl IcEvaluator {
    fn new(ics: Vec<IcEntry>) -> Self {
        let device_count = ics.iter().map(|e| e.device_index + 1).max().unwrap_or(0);
        let mut position_of_device = vec![u32::MAX; device_count];
        for (at, entry) in ics.iter().enumerate() {
            position_of_device[entry.device_index] = at as u32;
        }
        IcEvaluator {
            ics,
            evaluations: 0,
            position_of_device,
            journaling: false,
            journal: Vec::new(),
            spare: Vec::new(),
            levels: Vec::new(),
            published_once: false,
        }
    }

    /// The position in `ics` of the IC at program device `device`.
    fn position(&self, device: usize) -> Option<usize> {
        match self.position_of_device.get(device) {
            Some(&p) if p != u32::MAX => Some(p as usize),
            _ => None,
        }
    }

    /// Saves IC `at`'s pre-image into the open window's journal, once per
    /// window and before the first write. Callers check `self.journaling`.
    fn save(&mut self, at: usize) {
        let entry = &mut self.ics[at];
        if entry.journaled {
            return;
        }
        entry.journaled = true;
        let saved = match self.spare.pop() {
            Some(mut saved) => {
                saved.at = at;
                saved.family.clone_from(&entry.family);
                saved.last_reads.clone_from(&entry.last_reads);
                saved.family_slots.clone_from(&entry.family_slots);
                saved
            }
            None => Saved { at, family: entry.family.clone(), last_reads: entry.last_reads.clone(), family_slots: entry.family_slots.clone() },
        };
        self.journal.push(saved);
    }

    /// Closes the window, keeping every write: the pre-images go back to the
    /// pool.
    fn end_window(&mut self) {
        for saved in self.journal.drain(..) {
            self.ics[saved.at].journaled = false;
            self.spare.push(saved);
        }
        self.journaling = false;
    }

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
fn family_requests_into(device_index: usize, output_drivers: &[DriverId], out: &digital::FamilyOutput, reqs: &mut Vec<DriverRequest>) {
    debug_assert_eq!(output_drivers.len(), out.requests.len(), "one driver per output-capable pin, in the same order");
    reqs.extend(output_drivers.iter().zip(out.requests.iter()).map(|(&driver, &(level, delays))| DriverRequest::Level { driver, level, delays }));
    reqs.extend(out.timers.iter().map(|&(at, tag)| DriverRequest::Timer { device: DeviceId(device_index as u32), at, tag }));
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
        let mut out = Vec::new();
        self.evaluate_into(reader, now, acting_on_edges, voltages, &mut out);
        out
    }

    /// The body of [`Self::evaluate`], appending to the kernel's reusable
    /// request buffer. Everything it needs per call (the pin reads, the
    /// previous reads' storage) is reused rather than allocated: this runs
    /// tens of thousands of times per simulated second.
    fn evaluate_into(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64], out: &mut Vec<DriverRequest>) {
        self.evaluations += 1;
        let at = self.position(reader.0 as usize).unwrap_or_else(|| panic!("IcEvaluator: no IC registered for reader {reader:?}"));
        if self.journaling {
            self.save(at);
        }
        let entry = &mut self.ics[at];
        let ic = &entry.record;

        // NE555 point 1: dispatched before any `levels::read`/power-gate/
        // `last_reads` bookkeeping the other three families share below --
        // it has no logic-pin read concept of its own ("no logic level...
        // it publishes its latch instead"). `digital::ne555::evaluate`
        // already re-derives power itself (`None` while unpowered), so this
        // branch does its own power gate rather than sharing the one below.
        if let Family::Ne555(state) = &mut entry.family {
            match digital::ne555::evaluate(ic, state, voltages, acting_on_edges) {
                None => {
                    // Point 10: unpowered -- the slot goes NaN and nothing is
                    // requested (drivers keep their frozen committed level
                    // through the kernel's own power-loss hold).
                    for s in entry.family_slots.iter_mut() {
                        *s = f64::NAN;
                    }
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
                    out.push(DriverRequest::Level { driver: entry.output_drivers[0], level, delays });
                    out.push(DriverRequest::Level { driver: entry.output_drivers[1], level, delays });
                }
            }
            return;
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
            return;
        }

        // Every logic pin's read, in pin order (W1c contract point 4).
        // Schmitt hysteresis state only exists for the combinational
        // family (`kinds::seq_check` rejects `schmitt = 1` on every W1d
        // kind), so a sequential/memory read uses a throwaway per-pin
        // scratch slot instead of a stored one.
        let levels = &mut self.levels;
        levels.clear();
        if let Family::Combinational { schmitt_state } = &mut entry.family {
            levels.extend(ic.logic.iter().zip(schmitt_state.iter_mut()).map(|(p, st)| digital::levels::read(ic, &power, p, voltages[p.node], now, st)));
        } else {
            levels.extend(ic.logic.iter().map(|p| {
                let mut scratch = false;
                digital::levels::read(ic, &power, p, voltages[p.node], now, &mut scratch)
            }));
        }

        // "lastReads is written at every powered evaluation, frozen-settle
        // passes included" (W1d delays point 3) — for every family, not
        // only sequential/memory (this entry's own doc comment on the
        // field). The previous reads are taken out for the evaluation (which
        // only ever sees them as its `last_reads` argument) and their storage
        // is refilled with this evaluation's reads afterwards.
        let previous = entry.last_reads.take();

        match &mut entry.family {
            Family::Combinational { .. } => {
                let result = digital::combinational::evaluate(ic.kind, levels);
                digital::delay::driver_requests_into(ic, &entry.output_drivers, &result, levels, previous.as_deref(), out);
            }
            Family::Sequential(state) => {
                let result = digital::sequential::evaluate(ic, state, levels, previous.as_deref(), acting_on_edges, now);
                for (i, v) in entry.family_slots.iter_mut().enumerate() {
                    *v = result.slots.get(i).copied().unwrap_or(f64::NAN);
                }
                family_requests_into(entry.device_index, &entry.output_drivers, &result, out);
            }
            Family::Memory(state) => {
                let result = digital::memory::evaluate(ic, state, levels, previous.as_deref(), acting_on_edges, now);
                for (i, v) in entry.family_slots.iter_mut().enumerate() {
                    *v = result.slots.get(i).copied().unwrap_or(f64::NAN);
                }
                family_requests_into(entry.device_index, &entry.output_drivers, &result, out);
            }
            // Returned above, before any read or `last_reads` write.
            Family::Ne555(_) => unreachable!("a 555 is dispatched at the top of `evaluate`"),
        }

        let mut reads = previous.unwrap_or_default();
        reads.clear();
        reads.extend_from_slice(levels);
        entry.last_reads = Some(reads);
    }

    /// Every piece of per-device mutable state `evaluate` (or `seed_memory`)
    /// may have written this instant — the family state, `last_reads` and
    /// `family_slots`, one tuple per registered IC in `self.ics` order
    /// (core/README.md's "A failed advance": rolled back along with
    /// everything else the attempt touched). `evaluations` is deliberately
    /// NOT here — see its own doc comment. The kernel no longer calls this
    /// per instant (it uses [`Self::begin_instant`]'s journal, which saves
    /// only the ICs an instant wrote); it stays as the whole-state form of
    /// the same rollback.
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

    /// Opens the journal: from here every write to an IC saves that IC's
    /// pre-image first (once), so a completed instant, the common case,
    /// costs a copy of what it touched rather than of every IC.
    fn begin_instant(&mut self) -> Box<dyn std::any::Any> {
        assert!(!self.journaling && self.journal.is_empty(), "rollback windows do not nest");
        self.journaling = true;
        Box::new(())
    }

    fn end_instant(&mut self, _checkpoint: Box<dyn std::any::Any>) {
        self.end_window();
    }

    /// Puts every journaled IC back to its pre-image. The ICs the instant
    /// never wrote are untouched, which is what makes this equal to
    /// restoring a whole-state snapshot.
    fn rollback_instant(&mut self, _checkpoint: Box<dyn std::any::Any>) {
        for mut saved in self.journal.drain(..) {
            let entry = &mut self.ics[saved.at];
            std::mem::swap(&mut entry.family, &mut saved.family);
            std::mem::swap(&mut entry.last_reads, &mut saved.last_reads);
            std::mem::swap(&mut entry.family_slots, &mut saved.family_slots);
            entry.journaled = false;
            self.spare.push(saved);
        }
        self.journaling = false;
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
    ///
    /// Only what changed is returned: the first call hands the kernel every
    /// slot, every later one just the slots whose bits differ from what this
    /// method last returned for them. The engine that drains these writes
    /// (`Engine::apply_digital_writes`) starts from an all-NaN slot array
    /// that only these writes ever change, so a slot re-written to the value
    /// (or NaN payload) it already holds is a no-op there, for the slot array
    /// and for the trace alike; leaving those out saves building, queueing
    /// and replaying every IC's every slot at every completed instant.
    fn on_settled(&mut self, _now: f64, drivers: &[Driver], solver: &dyn PartitionSolver) -> Vec<(usize, f64)> {
        let mut out = Vec::new();
        let voltages = solver.voltages();
        let all = !self.published_once;
        self.published_once = true;
        for entry in &mut self.ics {
            // The same verdict `PartitionSolver::ic_powered` gives (it reads
            // the same record's supply nodes off the same published
            // voltages), without its scan for the IC among all of them.
            let powered = digital::power::powered(&entry.record, voltages[entry.record.vcc.node], voltages[entry.record.gnd.node]);
            match &entry.family {
                Family::Combinational { .. } => {
                    for (i, &driver_id) in entry.output_drivers.iter().enumerate() {
                        let value = if !powered {
                            f64::NAN
                        } else {
                            match drivers[driver_id.0 as usize].committed() {
                                Some(Mode::High) => 1.0,
                                Some(Mode::Low) => 0.0,
                                Some(Mode::Hiz) | None => f64::NAN,
                            }
                        };
                        if all || value.to_bits() != entry.published[i] {
                            entry.published[i] = value.to_bits();
                            out.push((entry.record.slot_base + i, value));
                        }
                    }
                }
                Family::Sequential(_) | Family::Memory(_) | Family::Ne555(_) => {
                    for (i, &value) in entry.family_slots.iter().enumerate() {
                        let value = if powered { value } else { f64::NAN };
                        if all || value.to_bits() != entry.published[i] {
                            entry.published[i] = value.to_bits();
                            out.push((entry.record.slot_base + i, value));
                        }
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
        let at = match self.position(device) {
            Some(at) => at,
            None => return Err(format!("seed_memory: no IC registered for device {device}")),
        };
        if self.journaling {
            self.save(at);
        }
        let entry = &mut self.ics[at];
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
            // A driver on a fixed non-datum node has no partition of its own,
            // yet its stamp lands in the partition of whichever supply node is
            // free (W1c point 8), so its commit must dirty that one: without
            // it the free supply keeps the old stamp until something else
            // re-solves it. `partition::build` unions the two supplies when
            // both are free and such a pin exists, so `vcc` first and `gnd`
            // second name one partition either way. A pin on the datum stamps
            // nothing and, with both supplies fixed, nothing is left to dirty.
            let partition = kernel.solver().partition_of_node(p.node).or_else(|| {
                if p.node == 0 {
                    return None;
                }
                kernel.solver().partition_of_node(record.vcc.node).or_else(|| kernel.solver().partition_of_node(record.gnd.node))
            });
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

        // The digital slots this IC hands `on_settled`, in slot order: one per
        // output-capable pin for a combinational IC, `family_slots`' own count
        // for the others.
        let published_len = if matches!(family, Family::Combinational { .. }) { output_drivers.len() } else { family_slots.len() };
        ics.push(IcEntry { device_index, record, output_drivers, family, last_reads: None, family_slots, journaled: false, published: vec![0; published_len] });
    }
    IcEvaluator::new(ics)
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
        let mut kernel = Kernel::new(solver, IcEvaluator::new(Vec::new()));
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

    /// The whole per-IC evaluator state, as text: what `snapshot` captures
    /// (family state, `last_reads`, `family_slots`, in registration order).
    fn state_dump(kernel: &Kernel<PartitionedSolver, IcEvaluator>) -> String {
        let boxed = kernel.evaluator().snapshot();
        let states = boxed.downcast::<Vec<(Family, Option<Vec<bool>>, Vec<f64>)>>().expect("IcEvaluator::snapshot's own shape");
        format!("{states:?}")
    }

    #[test]
    fn an_instant_that_fails_after_touching_several_ics_rolls_every_one_back_and_leaves_no_journal() {
        // The evaluator no longer snapshots every IC before every instant: it
        // journals each IC's pre-image the first time the instant writes it.
        // A ring of three inverters closed by a live param oscillates at zero
        // delay and never settles (the shape `a_livelocked_instant_still_
        // counts_its_evaluations_but_queues_no_digital_writes` uses), and a
        // 74hc74 and a 74ls161 clocked off two of its nodes see hundreds of
        // edges inside that one doomed instant, so their registers, edge
        // history and readouts all move before it fails. After the failure
        // every IC must be exactly what it was, and a run that then goes on
        // must match a control that never made the attempt.
        //
        // 0=gnd, 1=vcc (5 V), 2=ring com/1a, 3=y1(=2a), 4=y2(=3a), 5=y3.
        let wire = |mut d: Device, logic: &[(usize, usize)]| {
            for &(i, node) in logic {
                d.pins[2 + i] = pin(node);
            }
            d
        };
        let devices = || {
            vec![
                source(0, 1, 0, 5.0),
                spdt(1, 2, 0, 5, 0.0), // sel = 0 selects b = gnd: loop broken.
                ls04(2, 1, 0, [2, 3, 3, 4, 4, 5, 0, 6, 0, 6, 0, 6], [false; 12], 0.008, 0.0),
                // 74hc74: /clr1, /pre1, /pre2, /clr2 on vcc (never asserted),
                // unit 1 clocked by the ring's y3, unit 2 by y1.
                at_slot(wire(w1d_ic(kinds::IC_74HC74, 3, 1, 0, 12, w1d_params(2, &[]), 0), &[(0, 1), (3, 1), (8, 1), (11, 1), (2, 5), (9, 3)]), 6),
                // 74ls161: /clr, /load, enp, ent on vcc, clocked by y2.
                at_slot(wire(w1d_ic(kinds::IC_74LS161, 4, 1, 0, 14, w1d_params(4, &[]), 0), &[(0, 1), (2, 1), (3, 1), (4, 1), (1, 4)]), 10),
            ]
        };
        let p = program(7, 15, devices());
        let (mut doomed, all_partitions, all_readers) = build(&p);
        let (mut control, ..) = build(&p);
        for k in [&mut doomed, &mut control] {
            k.settle(0.0, &all_partitions, &all_readers).unwrap();
            k.take_digital_writes();
        }

        let before = state_dump(&doomed);
        assert_eq!(before, state_dump(&control));
        let evaluations_before = doomed.evaluator().ic_evaluations();
        kernel_validate(&doomed);
        let err = doomed.apply_params_as_event(&[(1, 0, 1.0)]).unwrap_err();
        assert!(matches!(err, crate::kernel::KernelError::Livelock { .. }), "the closed ring must never settle: {err:?}");
        assert!(doomed.evaluator().ic_evaluations() - evaluations_before > 500.0, "the doomed instant must have evaluated the clocked ICs hundreds of times");

        assert_eq!(before, state_dump(&doomed), "every IC is back to its pre-image");
        assert!(doomed.take_digital_writes().is_empty(), "a failed instant publishes nothing");
        assert!(!doomed.evaluator().journaling && doomed.evaluator().journal.is_empty(), "the journal is closed and empty");
        assert!(doomed.evaluator().ics.iter().all(|e| !e.journaled), "no IC is left marked as journaled");

        // Going on from here must match a control that never attempted it: the
        // same evaluator state, and the same slot writes.
        for k in [&mut doomed, &mut control] {
            k.advance(1e-6).unwrap();
        }
        assert_eq!(state_dump(&control), state_dump(&doomed));
        assert_eq!(control.take_digital_writes(), doomed.take_digital_writes());
    }

    #[test]
    fn an_advance_that_fails_mid_run_rolls_the_ics_back_and_fails_the_same_way_again() {
        // Same rollback, reached through `Kernel::advance` instead of a param
        // event, with the doomed instant born from a queued delayed commit:
        // an enable inverter (20 ns) releases a NAND that closes a zero-delay
        // ring, so the instant at t = 20 ns oscillates forever while a 74hc74
        // and a 74ls161 clocked off the ring count edges. The failure must
        // leave every IC as it was, and (proving the pending commit and the
        // driver tokens came back too) trying again must fail the same way at
        // the same instant, with the same state left behind.
        //
        // 0=gnd, 1=vcc (5 V), 2=sel (spdt com), 3=enable, 4=NAND y, 5=inv y,
        // 6=inv y (fed back to the NAND), 7=dump for every unused output.
        let ls00 = |ci: usize, pins: [usize; 12]| {
            let mut device_pins = vec![pin(1), pin(0)];
            device_pins.extend(pins.iter().map(|&n| pin(n)));
            let mut blob = vec![0.0];
            blob.extend(std::iter::repeat(0.0).take(12));
            Device { kind: kinds::IC_74LS00, component_index: ci, pins: device_pins, params: ls_ttl_params(0.008, 0.0), blob }
        };
        let wire = |mut d: Device, logic: &[(usize, usize)]| {
            for &(i, node) in logic {
                d.pins[2 + i] = pin(node);
            }
            d
        };
        let p = program(8, 25, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 1.0), // sel = 1 selects c = vcc: enable LOW at load.
            at_slot(ls04(2, 1, 0, [2, 3, 0, 7, 0, 7, 0, 7, 0, 7, 0, 7], [false; 12], 0.008, 20e-9), 0),
            at_slot(ls00(3, [3, 6, 4, 0, 0, 7, 0, 0, 7, 0, 0, 7]), 6),
            at_slot(ls04(4, 1, 0, [4, 5, 5, 6, 0, 7, 0, 7, 0, 7, 0, 7], [false; 12], 0.008, 0.0), 10),
            at_slot(wire(w1d_ic(kinds::IC_74HC74, 5, 1, 0, 12, w1d_params(2, &[]), 0), &[(0, 1), (3, 1), (8, 1), (11, 1), (2, 6)]), 16),
            at_slot(wire(w1d_ic(kinds::IC_74LS161, 6, 1, 0, 14, w1d_params(4, &[]), 0), &[(0, 1), (2, 1), (3, 1), (4, 1), (1, 5)]), 20),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        kernel.take_digital_writes();

        // sel -> 0 makes the enable inverter's output rise, 20 ns from now:
        // this event itself settles (nothing is due yet), and queues the commit.
        kernel.solver().validate_params(&[(1, 0, 0.0)]).unwrap();
        kernel.apply_params_as_event(&[(1, 0, 0.0)]).unwrap();
        kernel.take_digital_writes();
        let time_before = kernel.time();
        let state_before = state_dump(&kernel);

        let evaluations_before = kernel.evaluator().ic_evaluations();
        let first = kernel.advance(1e-6).unwrap_err();
        let crate::kernel::KernelError::Livelock { at } = first else { panic!("expected a livelock, got {first:?}") };
        assert!((at - 20e-9).abs() < 1e-12, "the ring closes when the enable commits, at 20 ns: {at}");
        assert!(kernel.evaluator().ic_evaluations() - evaluations_before > 500.0, "the doomed instant must have evaluated the ring and the clocked ICs hundreds of times");
        assert_eq!(kernel.time(), time_before, "the clock stays at the last completed instant");
        assert_eq!(state_before, state_dump(&kernel), "every IC is back to its pre-image");
        assert!(kernel.take_digital_writes().is_empty(), "a failed instant publishes nothing");
        assert!(!kernel.evaluator().journaling && kernel.evaluator().journal.is_empty() && kernel.evaluator().ics.iter().all(|e| !e.journaled), "the journal is closed and empty");

        // The commit that was due is still queued, so it fails again, identically.
        let second = kernel.advance(1e-6).unwrap_err();
        assert_eq!(second, first);
        assert_eq!(state_before, state_dump(&kernel));
        assert!(kernel.take_digital_writes().is_empty());
    }

    #[test]
    fn a_failed_instant_between_two_completed_ones_leaves_the_engines_slots_and_trace_as_a_run_that_never_tried_it() {
        // Through the real `Engine`, so this covers what `on_settled` and the
        // engine's slot array do together. `on_settled` hands the kernel only
        // the slots that changed since it last returned them, which is exact
        // because the shadow it compares against (`IcEntry::published`) sits
        // OUTSIDE the rollback window: it moves only when an instant
        // completes, exactly like the engine's slot array. Were it inside, a
        // failed instant would leave the shadow and the array disagreeing and
        // the next completed instant would drop a real change.
        //
        // Completed instants change a slot (an inverter with a 20 ns delay
        // follows a switch). Then a zero-delay ring is closed by another
        // switch, with a 74hc74 counting its edges: that instant livelocks and
        // is rolled back. Then another instant completes and changes the same
        // slot again. The control run never closes the ring.
        //
        // 0=gnd, 1=vcc, 2=switch X com, 3=inverter X out, 4=ring com/1a,
        // 5=y1, 6=y2, 7=y3, 8=dump for every unused output.
        let wire = |mut d: Device, logic: &[(usize, usize)]| {
            for &(i, node) in logic {
                d.pins[2 + i] = pin(node);
            }
            d
        };
        let p = program(9, 16, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 0.0), // switch X: sel 0 selects gnd, sel 1 selects vcc.
            at_slot(ls04(2, 1, 0, [2, 3, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8], [false; 12], 0.008, 20e-9), 0),
            spdt(3, 4, 0, 7, 0.0), // switch R: sel 1 closes the ring.
            at_slot(ls04(4, 1, 0, [4, 5, 5, 6, 6, 7, 0, 8, 0, 8, 0, 8], [false; 12], 0.008, 0.0), 6),
            at_slot(wire(w1d_ic(kinds::IC_74HC74, 5, 1, 0, 12, w1d_params(2, &[]), 0), &[(0, 1), (3, 1), (8, 1), (11, 1), (2, 6)]), 12),
        ]);
        let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<u64>>();
        let readout = |e: &Engine| (bits(e.digital_slots()), bits(e.trace()), bits(e.voltages()), e.time().to_bits());

        let run = |attempt_the_failing_instant: bool| {
            let mut e = Engine::load(&p).unwrap();
            e.trace_enable(true);
            // Completed instants: X selects vcc and the inverter's output
            // falls 20 ns later, then X selects gnd and it rises again. The
            // slot is HIGH going into the failing instant, so a shadow that
            // wrongly lost track of it would treat the next fall as no change.
            e.set_params(&[(1, 0, 1.0)]).unwrap();
            e.advance(e.time() + 1e-6).unwrap();
            assert_eq!(e.digital_slots()[0], 0.0, "the inverter's output fell: a slot changed in a completed instant");
            e.set_params(&[(1, 0, 0.0)]).unwrap();
            e.advance(e.time() + 1e-6).unwrap();
            assert_eq!(e.digital_slots()[0], 1.0, "and rose again");
            // X selects vcc again: the fall is queued for 20 ns from now, and
            // nothing has published it yet.
            e.set_params(&[(1, 0, 1.0)]).unwrap();
            if attempt_the_failing_instant {
                let before = readout(&e);
                let err = e.set_params(&[(3, 0, 1.0)]).unwrap_err();
                assert_eq!(err.code, crate::error::ERR_DELTA_LIVELOCK, "the closed ring never settles: {}", err.message);
                assert_eq!(before, readout(&e), "the failed instant left the slot array, trace, voltages and clock as they were");
            }
            // The first instant to complete after the failure is the one that
            // lands the fall. The engine's slot array must show it and the
            // trace record it: a shadow that had lost track of the HIGH the
            // slot holds would call this fall no change and drop it.
            e.advance(e.time() + 1e-6).unwrap();
            assert_eq!(e.digital_slots()[0], 0.0, "the slot fell in the first instant to complete after the failure");
            readout(&e)
        };
        let control = run(false);
        let with_failure = run(true);
        assert_eq!(control, with_failure, "slots, trace, voltages and time are bit-identical to a run that never attempted the failed instant");
        assert!(!control.1.is_empty(), "the trace recorded the two changes, so the comparison is not vacuous");
    }

    #[test]
    fn a_rejected_seed_memory_leaves_the_ic_state_and_the_journal_as_they_were() {
        // `Kernel::seed_memory` opens a rollback window, and the evaluator's
        // `seed_memory` saves the IC's pre-image before it decides whether to
        // accept. A rejection ends the window through `abandon_window`, which
        // rolls the evaluator back and clears the journal marks; nothing else
        // may be left changed or marked.
        let p = program(2, 20, vec![
            source(0, 1, 0, 5.0),
            w1d_ic(kinds::IC_74HC74, 1, 1, 0, 12, w1d_params(2, &[]), 0),
            at_slot(w1d_ic(kinds::IC_74LS189, 2, 1, 0, 14, w1d_params(5, &[]), 0), 4),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        kernel.take_digital_writes();
        let before = state_dump(&kernel);
        let evaluations = kernel.evaluator().ic_evaluations();
        let clean = |k: &Kernel<PartitionedSolver, IcEvaluator>| {
            assert!(!k.evaluator().journaling && k.evaluator().journal.is_empty(), "the journal is closed and empty");
            assert!(k.evaluator().ics.iter().all(|e| !e.journaled), "no IC is left marked as journaled");
        };

        // A 74hc74 is not a memory: `save` has run for it by the time it is refused.
        match kernel.seed_memory(DeviceId(1), &[0.0; 16]) {
            Err(crate::kernel::SeedMemoryError::Rejected(msg)) => assert!(msg.contains("not a memory"), "{msg}"),
            other => panic!("expected a rejection, got {other:?}"),
        }
        assert_eq!(before, state_dump(&kernel));
        clean(&kernel);

        // A real 74ls189 given a nibble outside [0, 15]: also refused.
        assert!(matches!(kernel.seed_memory(DeviceId(2), &[16.0; 16]), Err(crate::kernel::SeedMemoryError::Rejected(_))));
        assert_eq!(before, state_dump(&kernel));
        clean(&kernel);
        assert!(kernel.take_digital_writes().is_empty(), "a rejected seed publishes nothing");
        assert_eq!(kernel.evaluator().ic_evaluations(), evaluations, "a rejection runs no delta cycle");

        // The next, valid seed still works, and this time the state changes and the journal closes again.
        kernel.seed_memory(DeviceId(2), &[3.0; 16]).unwrap();
        assert_ne!(before, state_dump(&kernel), "an accepted seed changes the memory");
        clean(&kernel);
    }

    fn kernel_validate(kernel: &Kernel<PartitionedSolver, IcEvaluator>) {
        kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
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

    // ---- W1e follow-up, gap 2: a driver on a fixed node, its supply free ----
    //
    // The driver's own node is fixed, so `partition_of_node` gives it no
    // partition, yet its `1/R_out` stamp lands in whichever supply partition
    // is free. Every case loads through the real `Engine` and moves the
    // driver with a switch, so the only thing that can re-solve the supply at
    // the commit is the commit's own dirty mark. `io_max = 0` gives a constant
    // 50 ohm R_out, so every expected voltage is a linear hand solution.

    use crate::engine::Engine;

    const R_OUT: f64 = 50.0;

    /// Runs `engine`'s only pending change: `set_params` at the current
    /// instant, then to just before and to the commit, `delay` later, and
    /// returns the voltages of `nodes` at each.
    fn across_the_commit(engine: &mut Engine, triples: &[(u32, u32, f64)], delay: f64, nodes: &[usize]) -> (Vec<f64>, Vec<f64>) {
        let t0 = engine.time();
        engine.set_params(triples).unwrap();
        engine.advance(t0 + 0.5 * delay).unwrap();
        let before: Vec<f64> = nodes.iter().map(|&n| engine.voltages()[n]).collect();
        engine.advance(t0 + delay).unwrap();
        let at: Vec<f64> = nodes.iter().map(|&n| engine.voltages()[n]).collect();
        (before, at)
    }

    #[test]
    fn gap2_an_ic_output_on_a_fixed_node_moves_a_free_vcc_at_the_commit() {
        // 0=gnd, 1=5 V rail, 2=4.5 V source (1y wired to it), 3=vcc (1k from
        // the rail), 4=1a (spdt: sel 0 to gnd, 1 to the rail).
        let p = program(5, 6, vec![
            source(0, 1, 0, 5.0),
            source(1, 2, 0, 4.5),
            resistor(2, 1, 3, 1000.0),
            spdt(3, 4, 0, 1, 0.0),
            ls04(4, 3, 0, [4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [false; 12], 0.0, 20e-9),
        ]);
        let (s, rv, f) = (5.0, 1000.0, 4.5);
        // 1a LOW -> 1y HIGH: R_out from the 4.5 V source into vcc, in parallel
        // with the feed. 1a HIGH -> 1y LOW: R_out goes to gnd, vcc sees only the feed.
        let high = (s / rv + f / R_OUT) / (1.0 / rv + 1.0 / R_OUT);
        let mut engine = Engine::load(&p).unwrap();
        assert!((engine.voltages()[3] - high).abs() < 1e-6, "at load vcc = {}, hand solution {high}", engine.voltages()[3]);
        assert!((engine.currents()[1] - (high - f) / R_OUT).abs() < 1e-6, "the 4.5 V source absorbs what vcc sends back");

        let (before, at) = across_the_commit(&mut engine, &[(3, 0, 1.0)], 20e-9, &[3]);
        assert!((before[0] - high).abs() < 1e-6, "vcc must not move before the commit, got {}", before[0]);
        assert!((at[0] - s).abs() < 1e-6, "vcc = {} at the commit, hand solution {s}: the stamp left with 1y's HIGH", at[0]);
        assert!((engine.currents()[1] - (-f / R_OUT)).abs() < 1e-6, "the 4.5 V source now feeds 1y LOW into gnd: {}", engine.currents()[1]);

        let (_, back) = across_the_commit(&mut engine, &[(3, 0, 0.0)], 20e-9, &[3]);
        assert!((back[0] - high).abs() < 1e-6, "back to HIGH, vcc = {}, hand solution {high}", back[0]);
    }

    #[test]
    fn gap2_an_ic_output_on_a_fixed_node_moves_a_free_gnd_at_the_commit() {
        // vcc on the rail, gnd free (10 ohm to the datum): only 1y LOW stamps
        // into gnd's partition. 0=datum, 1=5 V rail, 2=1.5 V source, 3=IC
        // gnd, 4=1a.
        let p = program(5, 6, vec![
            source(0, 1, 0, 5.0),
            source(1, 2, 0, 1.5),
            resistor(2, 3, 0, 10.0),
            spdt(3, 4, 0, 1, 0.0),
            ls04(4, 1, 3, [4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [false; 12], 0.0, 20e-9),
        ]);
        let low_gnd = (1.5 / R_OUT) / (1.0 / 10.0 + 1.0 / R_OUT);
        let mut engine = Engine::load(&p).unwrap();
        assert!(engine.voltages()[3].abs() < 1e-6, "1y HIGH stamps into the rail, gnd sits at the datum");
        let (before, at) = across_the_commit(&mut engine, &[(3, 0, 1.0)], 20e-9, &[3]);
        assert!(before[0].abs() < 1e-6);
        assert!((at[0] - low_gnd).abs() < 1e-6, "gnd = {} at the commit, hand solution {low_gnd}", at[0]);
    }

    #[test]
    fn gap2_with_both_supplies_free_one_commit_re_solves_both_sides() {
        // vcc 10 ohm from the rail, gnd 10 ohm to the datum, 1y on the 1.5 V
        // source. HIGH loads vcc (4.4167 V, gnd 0); LOW loads gnd (5 V, 0.25 V).
        // The stamp leaves one supply and lands in the other in one commit.
        let p = program(6, 6, vec![
            source(0, 1, 0, 5.0),
            source(1, 2, 0, 1.5),
            resistor(2, 1, 3, 10.0),
            resistor(3, 4, 0, 10.0),
            spdt(4, 5, 0, 1, 0.0),
            ls04(5, 3, 4, [5, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [false; 12], 0.0, 20e-9),
        ]);
        let pb = crate::partition::build(&p).unwrap();
        assert_eq!(pb.local_index[3].unwrap().0, pb.local_index[4].unwrap().0, "vcc and gnd share the partition the driver dirties");
        let high_vcc = (5.0 / 10.0 + 1.5 / R_OUT) / (1.0 / 10.0 + 1.0 / R_OUT);
        let low_gnd = (1.5 / R_OUT) / (1.0 / 10.0 + 1.0 / R_OUT);
        let mut engine = Engine::load(&p).unwrap();
        assert!((engine.voltages()[3] - high_vcc).abs() < 1e-6 && engine.voltages()[4].abs() < 1e-6);
        let (_, at) = across_the_commit(&mut engine, &[(4, 0, 1.0)], 20e-9, &[3, 4]);
        assert!((at[0] - 5.0).abs() < 1e-6, "vcc = {} at the commit, hand solution 5", at[0]);
        assert!((at[1] - low_gnd).abs() < 1e-6, "gnd = {} at the commit, hand solution {low_gnd}", at[1]);
    }

    /// An `ne555` with no catalog window (2-18 V), pins in the record's
    /// order, `logic` in TRIG/OUT/RST/CV/THR/DIS order, digital slot base 0.
    fn ne555_dev(ci: usize, vcc: usize, gnd: usize, logic: [usize; 6]) -> Device {
        let mut params = vec![0.0; ic_param::COUNT];
        params[ic_param::VCC_MIN] = 2.0;
        params[ic_param::VCC_MAX] = 18.0;
        params[ic_param::NOMINAL] = 5.0;
        let mut pins = vec![pin(vcc), pin(gnd)];
        pins.extend(logic.map(pin));
        Device { kind: kinds::NE555, component_index: ci, pins, params, blob: vec![0.0; 7] }
    }

    #[test]
    fn gap2_a_555_out_on_a_fixed_node_moves_a_free_vcc_at_the_commit() {
        // 0=gnd, 1=5 V rail, 2=4.5 V source (OUT wired to it), 3=vcc (1k from
        // the rail), 4=TRIG and THR (spdt: sel 0 to gnd sets, 1 to the rail
        // resets), RST on the rail, CV and DIS on gnd. DIS on a free node
        // would join vcc's partition, and its own commit (LOW with the reset)
        // would re-solve vcc and hide a missing dirty mark on OUT's.
        // OUT stamps f/20 S to vcc and (1-f)/20 S to gnd with
        // f = 1 - min(0.5, 0.1/vS) HIGH and min(0.5, 0.1/vS) LOW (NE555 point
        // 7), and vS is vcc itself, so each vcc is the fixed point of
        // vcc = (S/Rv + g F) / (1/Rv + g + 1e-12), g = f/20.
        let p = program(5, 1, vec![
            source(0, 1, 0, 5.0),
            source(1, 2, 0, 4.5),
            resistor(2, 1, 3, 1000.0),
            spdt(3, 4, 0, 1, 0.0),
            ne555_dev(4, 3, 0, [4, 2, 1, 0, 4, 0]),
        ]);
        let vcc_with_out = |high: bool| {
            let mut n = 4.5f64;
            for _ in 0..200 {
                let f = if high { 1.0 - (0.1 / n).min(0.5) } else { (0.1 / n).min(0.5) };
                let g = f / 20.0;
                n = (5.0 / 1000.0 + g * 4.5) / (1.0 / 1000.0 + g + 1e-12);
            }
            n
        };
        let (high, low) = (vcc_with_out(true), vcc_with_out(false));
        assert!((high - low).abs() > 0.1, "the two states must differ for this to test anything: {high} vs {low}");

        let mut engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 1.0, "TRIG on gnd sets OUT HIGH at load");
        assert!((engine.voltages()[3] - high).abs() < 1e-4, "at load vcc = {}, hand solution {high}", engine.voltages()[3]);

        // OUT, like every 555 commit, lands in the same instant as the
        // comparator that requested it: THR above 2/3 vS resets the latch.
        let t0 = engine.time();
        engine.set_params(&[(3, 0, 1.0)]).unwrap();
        engine.advance(t0).unwrap();
        assert_eq!(engine.digital_slots()[0], 0.0, "THR on the rail resets OUT LOW");
        assert!((engine.voltages()[3] - low).abs() < 1e-4, "vcc = {} at the commit, hand solution {low}", engine.voltages()[3]);
    }
}
