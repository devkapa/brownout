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

use crate::digital::power::{IcPower, PowerHold};
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
            Family::Sequential(state) => Family::Sequential(*state),
            Family::Memory(state) => Family::Memory(state.clone()),
            Family::Ne555(state) => Family::Ne555(state.clone()),
        }
    }

    /// The rollback journal refills a pre-image of the same variant every
    /// time (an IC's family never changes). The Schmitt vector's storage is
    /// reused, and a register's or 555's state is a plain copy. A memory's is
    /// overwritten whole by its derived `clone` (a 189's 16 nibbles are
    /// copied; an EEPROM's image is an `Rc`, so the pre-image shares it
    /// copy-on-write, as the old pool did), not refilled in place.
    fn clone_from(&mut self, source: &Self) {
        match (&mut *self, source) {
            (Family::Combinational { schmitt_state: to }, Family::Combinational { schmitt_state: from }) => to.clone_from(from),
            (Family::Sequential(to), Family::Sequential(from)) => *to = *from,
            (Family::Ne555(to), Family::Ne555(from)) => *to = *from,
            (Family::Memory(to), Family::Memory(from)) => to.clone_from(from),
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
    record: crate::shared::Shared<IcRecord>,
    output_drivers: crate::shared::Shared<[DriverId]>,
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
    /// `rco` has no slot at all). For a combinational entry it is sized the
    /// same way (the kind's slot count, one per output-capable pin) but is
    /// all NaN and never written or read: its slots instead come from driver
    /// levels, exactly as W1c already did.
    family_slots: Vec<f64>,
    /// Whether the open rollback window's journal already holds this IC's
    /// pre-image (`saved`).
    journaled: bool,
    /// The pre-image, valid while `journaled`.
    saved: Saved,
    /// Whether an evaluation can write anything of this IC but its
    /// `last_reads`, so whether the pre-image must carry `family` and
    /// `family_slots` too. Not the case for a combinational IC without
    /// hysteresis: it keeps no per-pin state (`Family::Combinational`'s
    /// `schmitt_state` is written only by a Schmitt part's reads) and writes
    /// no slot readout (its `family_slots` stay the NaN vector `register`
    /// made, unread), so those two never change.
    saves_state: bool,
    /// The bits of every digital slot this IC owns (`record.slot_base + i`)
    /// as `on_settled` last handed them to the kernel; see
    /// [`IcEvaluator::on_settled`].
    published: Vec<u64>,
    /// Whether this IC was powered at the last `on_settled` call, kept exact
    /// (`IcEvaluator::on_settled`'s supply scan moves it whenever a supply
    /// node's voltage does), so a powered check costs no arithmetic there.
    powered: bool,
    /// Whether this IC is on `IcEvaluator::dirty`, the list of ICs whose
    /// published slots the next `on_settled` must re-derive.
    dirty: bool,
    /// The supply voltages' bits `power` was derived from, and the result:
    /// `digital::power::ic_power` is a pure function of them, so an
    /// evaluation whose supply has not moved (almost all of them) reuses it.
    power_key: Option<(u64, u64)>,
    power: IcPower,
}

/// One node some IC takes its supply from (`vcc` or `gnd`), with the ICs that
/// do and the bits of the voltage `on_settled` last saw on it. A supply
/// almost never moves, so watching these few nodes replaces asking every IC
/// whether it is still powered.
#[derive(Clone, Debug)]
struct SupplyNode {
    node: usize,
    bits: u64,
    ics: Vec<u32>,
}

impl IcEntry {
    /// Appends this IC's changed slots to `out` and records them as
    /// published; `all` publishes every one regardless (the first call).
    fn publish(&mut self, drivers: &[Driver], all: bool, out: &mut Vec<(usize, f64)>) {
        slot_writes(&self.family, &self.output_drivers, &self.family_slots, self.record.slot_base, &mut self.published, self.powered, drivers, all, out);
    }
}

/// The slots of one IC whose value differs from the bits in `published` (or
/// every one, when `all`), in slot order, with `published` updated to match.
///
/// Contract point 10 ("Readouts") for the combinational family: a digital
/// slot holds the pin's committed level while driven (1.0 HIGH, 0.0 LOW — an
/// `OPEN_COLLECTOR` pin released HIGH reads exactly like a push-pull HIGH
/// here, since its `Driver::committed()` is `Some(High)` regardless of
/// whether the solver's own stamp then releases it or drives it — point 8's
/// electrical distinction, not a readout one) and `NaN` while the IC is
/// unpowered or the driver is uncommitted/reset (`None`) or mid-HIZ
/// (`Some(Hiz)`, unreachable in practice — `digital::delay::driver_requests`
/// never requests it — but handled rather than assumed impossible).
///
/// W1d sequential point 8 / memories point 6 for the other families: a slot
/// holds the evaluator's OWN view from its latest evaluation (`family_slots`,
/// already NaN while unpowered — see `IcEvaluator::evaluate`'s own doc
/// comment), not a driver's committed level at all — `powered` is still
/// applied here too, defensively, for parity with the combinational branch's
/// identical belt-and-suspenders read of live power state rather than
/// trusting a value computed earlier in the same instant.
#[allow(clippy::too_many_arguments)]
fn slot_writes(
    family: &Family,
    output_drivers: &[DriverId],
    family_slots: &[f64],
    slot_base: usize,
    published: &mut [u64],
    powered: bool,
    drivers: &[Driver],
    all: bool,
    out: &mut Vec<(usize, f64)>,
) {
    match family {
        Family::Combinational { .. } => {
            for (i, &driver_id) in output_drivers.iter().enumerate() {
                let value = if !powered {
                    f64::NAN
                } else {
                    match drivers[driver_id.0 as usize].committed() {
                        Some(Mode::High) => 1.0,
                        Some(Mode::Low) => 0.0,
                        Some(Mode::Hiz) | None => f64::NAN,
                    }
                };
                if all || value.to_bits() != published[i] {
                    published[i] = value.to_bits();
                    out.push((slot_base + i, value));
                }
            }
        }
        Family::Sequential(_) | Family::Memory(_) | Family::Ne555(_) => {
            for (i, &value) in family_slots.iter().enumerate() {
                let value = if powered { value } else { f64::NAN };
                if all || value.to_bits() != published[i] {
                    published[i] = value.to_bits();
                    out.push((slot_base + i, value));
                }
            }
        }
    }
}

/// One IC's pre-image, taken before the first thing an instant wrote to it:
/// exactly the state `snapshot` used to clone for every IC at every instant.
/// It lives inside the IC's own entry (`IcEntry::saved`) and is only
/// meaningful while `IcEntry::journaled` is set, so taking one refills
/// storage that is already there instead of moving a record in and out of a
/// pool.
#[derive(Debug, Clone)]
struct Saved {
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
#[derive(Debug, Clone)]
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
    position_of_device: crate::shared::Shared<[u32]>,
    /// Whether a rollback window is open (`begin_instant`..`end_instant`/
    /// `rollback_instant`): only then does a write to an IC save its
    /// pre-image first.
    journaling: bool,
    /// The positions in `ics` of the ICs written in the open window, one
    /// each, whose pre-images are in `IcEntry::saved` (`IcEntry::journaled`).
    journal: Vec<u32>,
    /// The 555s whose latch flipped once in the open window (since its last
    /// latch baseline, [`DeviceEvaluator::new_latch_baseline`]), by position
    /// in `ics`, each with whether another device's driver has changed since
    /// that flip. A second flip with none between returns the latch to the
    /// state that flipped it, its own output the only cause: the zero-delay
    /// loop of NE555 point 11 ([`DeviceEvaluator::zero_delay_loop`]). A
    /// second flip after another device changed answers that change (a
    /// delta-cycle hazard into TRIG or RST) and counts as a first flip
    /// again. Empty outside a window: load settling keeps the pass cap
    /// alone.
    latch_flips: Vec<(u32, bool)>,
    /// Whether a 555 in the open window flipped back.
    latch_loop: bool,
    /// Scratch for one evaluation's pin reads.
    levels: Vec<bool>,
    /// Scratch for a combinational evaluation's truth-table result.
    results: Vec<Option<bool>>,
    /// Scratch for a sequential or memory evaluation's result.
    family_out: digital::FamilyOutput,
    /// Whether `on_settled` has published yet: its first call hands the
    /// kernel every slot, later ones only what changed.
    published_once: bool,
    /// Positions in `ics` whose published slots `on_settled` must re-derive:
    /// an IC joins when a driver of its own commits or resets
    /// ([`DeviceEvaluator::driver_changed`]), when one that keeps its readout
    /// in `family_slots` is evaluated, and when its supply crosses its power
    /// window. `IcEntry::dirty` marks membership.
    dirty: Vec<u32>,
    /// `DriverId -> position in `ics``, `u32::MAX` for a driver no IC owns.
    driver_owner: crate::shared::Shared<[u32]>,
    /// Every distinct supply node of every IC; see [`SupplyNode`].
    supplies: crate::shared::Shared<[SupplyNode]>,
    /// The solver's power-hold table as the last `on_settled` saw it (empty
    /// while nothing was held): an IC whose hold changed is re-checked, as
    /// one whose supply moved is.
    seen_holds: Vec<PowerHold>,
    /// How many ICs `on_settled_into`'s incremental path has looked at, for
    /// the tests that pin its cost to what changed.
    #[cfg(test)]
    visited: u64,
}


impl IcEvaluator {
    fn new(ics: Vec<IcEntry>) -> Self {
        let device_count = ics.iter().map(|e| e.device_index + 1).max().unwrap_or(0);
        let mut position_of_device = vec![u32::MAX; device_count];
        for (at, entry) in ics.iter().enumerate() {
            position_of_device[entry.device_index] = at as u32;
        }
        let driver_count = ics.iter().flat_map(|e| e.output_drivers.iter()).map(|d| d.0 as usize + 1).max().unwrap_or(0);
        let mut driver_owner = vec![u32::MAX; driver_count];
        let mut supply_uses: Vec<(usize, u32)> = Vec::new();
        for (at, entry) in ics.iter().enumerate() {
            for driver in &entry.output_drivers {
                driver_owner[driver.0 as usize] = at as u32;
            }
            supply_uses.push((entry.record.vcc.node, at as u32));
            supply_uses.push((entry.record.gnd.node, at as u32));
        }
        supply_uses.sort_unstable();
        supply_uses.dedup();
        let mut supplies: Vec<SupplyNode> = Vec::new();
        for (node, at) in supply_uses {
            match supplies.last_mut() {
                Some(last) if last.node == node => last.ics.push(at),
                _ => supplies.push(SupplyNode { node, bits: 0, ics: vec![at] }),
            }
        }
        IcEvaluator {
            ics,
            evaluations: 0,
            position_of_device: position_of_device.into(),
            journaling: false,
            journal: Vec::new(),
            latch_flips: Vec::new(),
            latch_loop: false,
            levels: Vec::new(),
            results: Vec::new(),
            family_out: digital::FamilyOutput::default(),
            published_once: false,
            dirty: Vec::new(),
            driver_owner: driver_owner.into(),
            supplies: supplies.into(),
            seen_holds: Vec::new(),
            #[cfg(test)]
            visited: 0,
        }
    }

    /// Adds IC `at` to the list `on_settled` re-derives, once.
    fn mark_dirty(&mut self, at: usize) {
        let entry = &mut self.ics[at];
        if !entry.dirty {
            entry.dirty = true;
            self.dirty.push(at as u32);
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
        entry.saved.last_reads.clone_from(&entry.last_reads);
        if entry.saves_state {
            entry.saved.family.clone_from(&entry.family);
            entry.saved.family_slots.clone_from(&entry.family_slots);
        }
        self.journal.push(at as u32);
    }

    /// Closes the window, keeping every write: the pre-images are simply no
    /// longer valid.
    fn end_window(&mut self) {
        for &at in &self.journal {
            self.ics[at as usize].journaled = false;
        }
        self.journal.clear();
        self.journaling = false;
        self.latch_flips.clear();
        self.latch_loop = false;
    }

    /// [`DeviceEvaluator::evaluate`]'s body. `live`, when given, is the
    /// kernel's driver table: a request `Driver::request_is_noop` calls inert
    /// is left out of `out` instead of built, and (kernel-side) never
    /// journaled, applied or counted. Without it every request is kept, which
    /// is what load settling's `evaluate` wants.
    ///
    /// `holds` is the solver's power-hold table: a held-off IC reads and
    /// requests nothing (W1c point 1a), exactly as an unpowered one.
    fn evaluate_impl(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64], holds: &[PowerHold], live: Option<&[Driver]>, out: &mut Vec<DriverRequest>) {
        self.evaluations += 1;
        let at = self.position(reader.0 as usize).unwrap_or_else(|| panic!("IcEvaluator: no IC registered for reader {reader:?}"));
        if self.journaling {
            self.save(at);
        }
        let entry = &mut self.ics[at];
        let ic = &entry.record;
        // Read only when some IC is held: this runs at every evaluation.
        let hold = if holds.is_empty() { PowerHold::FREE } else { digital::power::hold_of(holds, entry.device_index) };
        // A combinational IC's readout comes from its drivers' committed
        // levels and its power (`on_settled`), neither of which an evaluation
        // writes; the other families keep theirs in `family_slots`. Whether an
        // evaluation changed one is tracked as it writes them, and puts the IC
        // on the list `on_settled` re-derives.
        let mut slots_changed = false;

        // NE555 point 1: dispatched before any `levels::read`/power-gate/
        // `last_reads` bookkeeping the other three families share below --
        // it has no logic-pin read concept of its own ("no logic level...
        // it publishes its latch instead"). `digital::ne555::evaluate`
        // already re-derives power itself (`None` while unpowered), so this
        // branch does its own power gate rather than sharing the one below.
        if let Family::Ne555(state) = &mut entry.family {
            let stored = state.out_high;
            let result = digital::ne555::evaluate(ic, state, voltages, acting_on_edges, hold);
            // NE555 point 11: a latch written back to the value it already
            // left in this window has no consistent state here.
            if self.journaling && state.out_high != stored {
                match self.latch_flips.iter_mut().find(|f| f.0 == at as u32) {
                    Some((_, false)) => self.latch_loop = true,
                    Some(entry) => entry.1 = false,
                    None => self.latch_flips.push((at as u32, false)),
                }
            }
            match result {
                None => {
                    // Point 10: unpowered -- the slot goes NaN and nothing is
                    // requested (drivers keep their frozen committed level
                    // through the kernel's own power-loss hold).
                    for s in entry.family_slots.iter_mut() {
                        slots_changed |= set_slot(s, f64::NAN);
                    }
                }
                Some(out_high) => {
                    // Point 12: the slot holds `outHigh` at every completed
                    // instant while powered.
                    slots_changed |= set_slot(&mut entry.family_slots[0], if out_high { 1.0 } else { 0.0 });
                    // Point 4: OUT HIGH/LOW as the result, DIS LOW when the
                    // result is 0 or HIGH (released) when it is 1 -- the SAME
                    // `Mode` for both drivers -- with DelaySet::uniform(0):
                    // no output lag, no propagation delay.
                    let level = if out_high { Mode::High } else { Mode::Low };
                    let delays = DelaySet::uniform(0.0);
                    push_level(out, live, entry.output_drivers[0], level, delays);
                    push_level(out, live, entry.output_drivers[1], level, delays);
                }
            }
            if slots_changed && !entry.dirty {
                entry.dirty = true;
                self.dirty.push(at as u32);
            }
            return;
        }

        let (v_vcc, v_gnd) = (voltages[ic.vcc.node], voltages[ic.gnd.node]);
        let key = (v_vcc.to_bits(), v_gnd.to_bits());
        if entry.power_key != Some(key) {
            entry.power = digital::power::ic_power(ic, v_vcc, v_gnd);
            entry.power_key = Some(key);
        }
        // The cache holds the free verdict; a hold overrides it (W1c point
        // 1a).
        let mut power = entry.power;
        if !hold.is_free() {
            digital::power::hold_onto(&mut power, ic, v_vcc, v_gnd, hold);
        }
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
                    slots_changed |= set_slot(s, f64::NAN);
                }
                if slots_changed && !entry.dirty {
                    entry.dirty = true;
                    self.dirty.push(at as u32);
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
        if power.schmitt.is_none() {
            // No hysteresis state to consult or write: one loop for every pin.
            digital::levels::read_plain_into(ic, &power, voltages, now, levels);
        } else if let Family::Combinational { schmitt_state } = &mut entry.family {
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
                digital::combinational::evaluate_into(ic.kind, levels, &mut self.results);
                digital::delay::driver_requests_into(ic, &entry.output_drivers, &self.results, levels, previous.as_deref(), live, out);
            }
            Family::Sequential(state) => {
                digital::sequential::evaluate_into(ic, state, levels, previous.as_deref(), acting_on_edges, now, &mut self.family_out);
                slots_changed = store_slots(&mut entry.family_slots, &self.family_out.slots);
                family_requests_into(entry.device_index, &entry.output_drivers, &self.family_out, live, out);
            }
            Family::Memory(state) => {
                digital::memory::evaluate_into(ic, state, levels, previous.as_deref(), acting_on_edges, now, &mut self.family_out);
                slots_changed = store_slots(&mut entry.family_slots, &self.family_out.slots);
                family_requests_into(entry.device_index, &entry.output_drivers, &self.family_out, live, out);
            }
            // Returned above, before any read or `last_reads` write.
            Family::Ne555(_) => unreachable!("a 555 is dispatched at the top of `evaluate`"),
        }
        if slots_changed && !entry.dirty {
            entry.dirty = true;
            self.dirty.push(at as u32);
        }

        let mut reads = previous.unwrap_or_default();
        reads.clear();
        reads.extend_from_slice(levels);
        entry.last_reads = Some(reads);
    }

    /// What [`DeviceEvaluator::on_settled_into`] would append if it looked at
    /// every IC: the reference its incremental form is asserted against.
    #[cfg(test)]
    fn full_scan(&self, drivers: &[Driver], voltages: &[f64], holds: &[PowerHold]) -> Vec<(usize, f64)> {
        let mut out = Vec::new();
        for entry in &self.ics {
            let powered = entry_verdict(entry, voltages, holds);
            let mut published = entry.published.clone();
            slot_writes(&entry.family, &entry.output_drivers, &entry.family_slots, entry.record.slot_base, &mut published, powered, drivers, false, &mut out);
        }
        out
    }

    /// The device ids `Kernel::settle`'s `all_readers` needs — every
    /// registered IC, in registration (= program device) order. `engine.rs`
    /// calls this once, right after [`register`], to build the reader list
    /// load settling requires (W1c contract point 2: "settle with every IC
    /// as a reader").
    /// W2c's seed (program v4's `CARRY` block), at load before the settle:
    /// each carried IC's family state, its last reads (so the settle's edge
    /// pass sees an edge the reload made, against the level it had), its
    /// slot readout, and its output drivers, returned for the kernel to seed
    /// (`(driver, committed, pending (target, due))`), with the timers the
    /// state needs (an EEPROM write in flight). An EEPROM carried from
    /// another image starts afresh, drivers included, as classic's does. A
    /// record whose counts do not fit the IC is `Err`.
    #[allow(clippy::type_complexity)]
    pub fn carry_seed(&mut self, block: &[f64]) -> Result<(Vec<(DriverId, Option<Mode>, Option<(Mode, f64)>)>, Vec<(DeviceId, f64, u64)>), String> {
        use crate::carry;
        let (mut drivers, mut timers) = (Vec::new(), Vec::new());
        for r in carry::records(block)? {
            if !carry::known(&r) || r.family != carry::FAMILY_IC {
                continue;
            }
            let Some(entry) = self.ics.iter_mut().find(|e| e.device_index == r.device) else { continue };
            let seed = carry::ic::decode(entry.record.kind, r.values)?;
            let what = format!("carry: IC device {}", r.device);
            if seed.reads.as_ref().is_some_and(|reads| reads.len() != entry.record.logic.len()) {
                return Err(format!("{what}: {} reads for {} logic pins", seed.reads.as_ref().unwrap().len(), entry.record.logic.len()));
            }
            if seed.slots.len() != entry.family_slots.len() || seed.drivers.len() != entry.output_drivers.len() {
                return Err(format!("{what}: {} slots and {} drivers for {} and {}", seed.slots.len(), seed.drivers.len(), entry.family_slots.len(), entry.output_drivers.len()));
            }
            match &mut entry.family {
                Family::Combinational { schmitt_state } => {
                    if seed.family[0] != schmitt_state.len() as f64 {
                        return Err(format!("{what}: {} Schmitt levels for {} pins", seed.family[0], schmitt_state.len()));
                    }
                    for (b, &v) in schmitt_state.iter_mut().zip(&seed.family[1..]) {
                        *b = carry::ic::bit_of(v)?;
                    }
                }
                Family::Sequential(state) => state.carry_seed(seed.family)?,
                Family::Memory(state) => {
                    if !digital::memory::carry_seed(state, seed.family)? {
                        continue;
                    }
                    if let Some((at, tag)) = digital::memory::carried_timer(state) {
                        timers.push((DeviceId(entry.device_index as u32), at, tag));
                    }
                }
                Family::Ne555(state) => state.out_high = carry::ic::bit_of(seed.family[0])?,
            }
            entry.last_reads = seed.reads;
            entry.family_slots.copy_from_slice(seed.slots);
            drivers.extend(entry.output_drivers.iter().zip(seed.drivers).map(|(&id, (committed, pending))| (id, committed, pending)));
        }
        Ok((drivers, timers))
    }

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
/// is already a concrete `Mode` (`Hiz` included). A `Level` the driver would
/// ignore (`live`, see [`push_level`]) is left out; a `Timer` never is.
fn family_requests_into(device_index: usize, output_drivers: &[DriverId], out: &digital::FamilyOutput, live: Option<&[Driver]>, reqs: &mut Vec<DriverRequest>) {
    debug_assert_eq!(output_drivers.len(), out.requests.len(), "one driver per output-capable pin, in the same order");
    for (&driver, &(level, delays)) in output_drivers.iter().zip(out.requests.iter()) {
        push_level(reqs, live, driver, level, delays);
    }
    reqs.extend(out.timers.iter().map(|&(at, tag)| DriverRequest::Timer { device: DeviceId(device_index as u32), at, tag }));
}

/// Appends a `Level` request for `driver`, unless `live` (the kernel's driver
/// table) shows the driver would ignore it (`Driver::request_is_noop`).
fn push_level(out: &mut Vec<DriverRequest>, live: Option<&[Driver]>, driver: DriverId, level: Mode, delays: DelaySet) {
    if live.is_some_and(|live| live[driver.0 as usize].request_is_noop(level)) {
        return;
    }
    out.push(DriverRequest::Level { driver, level, delays });
}

/// Writes `value` into a family slot, reporting whether its bits changed
/// (a NaN over a NaN of the same payload is no change).
fn set_slot(slot: &mut f64, value: f64) -> bool {
    let changed = slot.to_bits() != value.to_bits();
    *slot = value;
    changed
}

/// Stores an evaluation's slot readout into `family_slots` (a slot the
/// evaluation left out reads as NaN), reporting whether any changed.
fn store_slots(family_slots: &mut [f64], readout: &[f64]) -> bool {
    let mut changed = false;
    for (i, v) in family_slots.iter_mut().enumerate() {
        changed |= set_slot(v, readout.get(i).copied().unwrap_or(f64::NAN));
    }
    changed
}

/// A memory device's ROM image, decoded from its blob tail (core/README.md,
/// "ROM image": blob is `[slotBase, pinFlags x L, word x W]`) — empty for a
/// 74ls189 (no ROM: `kinds::eeprom_words` is 0 for that kind).
fn rom_words(device: &Device, kind: u32, logic_len: usize) -> Vec<u32> {
    let words_from = 1 + logic_len;
    let words_count = kinds::eeprom_words(kind);
    device.blob[words_from..words_from + words_count].iter().map(|&w| w as u32).collect()
}

/// IC `entry`'s power verdict at `voltages` under `holds` (W1c points 1
/// and 1a).
#[inline]
fn entry_verdict(entry: &IcEntry, voltages: &[f64], holds: &[PowerHold]) -> bool {
    let ic = &entry.record;
    digital::power::verdict(ic, voltages[ic.vcc.node], voltages[ic.gnd.node], digital::power::hold_of(holds, entry.device_index))
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
        self.evaluate_held(reader, now, acting_on_edges, voltages, &[])
    }

    /// [`Self::evaluate`] under the solver's hold table (load settling).
    fn evaluate_held(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64], holds: &[PowerHold]) -> Vec<DriverRequest> {
        let mut out = Vec::new();
        self.evaluate_impl(reader, now, acting_on_edges, voltages, holds, None, &mut out);
        out
    }

    /// The body of [`Self::evaluate`], appending to the kernel's reusable
    /// request buffer and leaving out the requests `drivers` (the kernel's
    /// driver table) would ignore. Everything it needs per call (the pin
    /// reads, the previous reads' storage, the truth-table and family
    /// results) is reused rather than allocated: this runs tens of thousands
    /// of times per simulated second.
    fn evaluate_into(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64], holds: &[PowerHold], drivers: &[Driver], out: &mut Vec<DriverRequest>) -> bool {
        self.evaluate_impl(reader, now, acting_on_edges, voltages, holds, Some(drivers), out);
        self.latch_loop
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
        // Whatever the snapshot held may differ from what was last published.
        for at in 0..self.ics.len() {
            self.mark_dirty(at);
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
        for &at in &self.journal {
            let entry = &mut self.ics[at as usize];
            std::mem::swap(&mut entry.last_reads, &mut entry.saved.last_reads);
            if entry.saves_state {
                std::mem::swap(&mut entry.family, &mut entry.saved.family);
                std::mem::swap(&mut entry.family_slots, &mut entry.saved.family_slots);
            }
            entry.journaled = false;
        }
        self.journal.clear();
        self.journaling = false;
        self.latch_flips.clear();
        self.latch_loop = false;
    }

    fn zero_delay_loop(&self) -> bool {
        self.latch_loop
    }

    fn new_latch_baseline(&mut self) {
        self.latch_flips.clear();
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
    fn on_settled(&mut self, now: f64, drivers: &[Driver], solver: &dyn PartitionSolver) -> Vec<(usize, f64)> {
        let mut out = Vec::new();
        self.on_settled_into(now, drivers, solver, solver.power_holds(), &mut out);
        out
    }

    /// [`Self::on_settled`]'s body, appending to the kernel's reusable buffer.
    ///
    /// A slot's value is a function of three things: for a combinational IC
    /// its drivers' committed levels and its power, for the others their
    /// `family_slots` and their power. So only an IC one of those touched
    /// since the last call needs looking at, which is what keeps this
    /// proportional to what changed and not to the number of ICs: `dirty`
    /// collects the ICs whose driver committed or reset
    /// ([`DeviceEvaluator::driver_changed`]) and the family ICs that were
    /// evaluated, and the supply scan below adds the ICs whose power verdict
    /// flipped. The writes come out in IC order, as a scan over every IC
    /// produced them. Under `cfg(test)` every call also runs the scan over
    /// every IC and asserts the two agree, so a missed mark fails whichever
    /// test hits it.
    fn on_settled_into(&mut self, _now: f64, drivers: &[Driver], solver: &dyn PartitionSolver, holds: &[PowerHold], out: &mut Vec<(usize, f64)>) {
        let voltages = solver.voltages();
        if !self.published_once {
            self.published_once = true;
            for supply in &mut self.supplies {
                supply.bits = voltages[supply.node].to_bits();
            }
            self.seen_holds.clear();
            self.seen_holds.extend_from_slice(holds);
            self.dirty.clear();
            for entry in &mut self.ics {
                entry.dirty = false;
                entry.powered = entry_verdict(entry, voltages, holds);
                entry.publish(drivers, true, out);
            }
            return;
        }

        #[cfg(test)]
        let (first_write, expected) = (out.len(), self.full_scan(drivers, voltages, holds));

        // A hold changes the verdict with no supply moving (W1c point 1a):
        // the ICs whose hold changed since the last call are re-checked. Both
        // tables are empty whenever nothing is or was held.
        if holds != self.seen_holds.as_slice() {
            for (at, entry) in self.ics.iter_mut().enumerate() {
                let d = entry.device_index;
                if digital::power::hold_of(holds, d) == digital::power::hold_of(&self.seen_holds, d) {
                    continue;
                }
                let powered = entry_verdict(entry, voltages, holds);
                if powered != entry.powered {
                    entry.powered = powered;
                    if !entry.dirty {
                        entry.dirty = true;
                        self.dirty.push(at as u32);
                    }
                }
            }
            self.seen_holds.clear();
            self.seen_holds.extend_from_slice(holds);
        }

        // The power verdict only depends on the two supply voltages, so an IC
        // is re-checked only when one of its supply nodes moved, and joins
        // `dirty` only when the verdict itself flipped.
        for supply in &mut self.supplies {
            let bits = voltages[supply.node].to_bits();
            if bits == supply.bits {
                continue;
            }
            supply.bits = bits;
            for &at in &supply.ics {
                let entry = &mut self.ics[at as usize];
                let powered = entry_verdict(entry, voltages, holds);
                if powered != entry.powered {
                    entry.powered = powered;
                    if !entry.dirty {
                        entry.dirty = true;
                        self.dirty.push(at);
                    }
                }
            }
        }

        if !self.dirty.is_empty() {
            self.dirty.sort_unstable();
            for &at in &self.dirty {
                let entry = &mut self.ics[at as usize];
                entry.dirty = false;
                entry.publish(drivers, false, out);
                #[cfg(test)]
                {
                    self.visited += 1;
                }
            }
            self.dirty.clear();
        }

        #[cfg(test)]
        {
            let bits = |writes: &[(usize, f64)]| writes.iter().map(|&(slot, v)| (slot, v.to_bits())).collect::<Vec<_>>();
            assert_eq!(bits(&out[first_write..]), bits(&expected), "on_settled's incremental writes differ from a scan over every IC: a change was not marked");
        }
    }

    /// A driver's committed level moved: the IC that owns it has a slot to
    /// re-derive (`on_settled_into`). A driver no IC owns is ignored.
    fn driver_changed(&mut self, driver: DriverId) {
        let owner = self.driver_owner.get(driver.0 as usize).copied().unwrap_or(u32::MAX);
        if owner != u32::MAX {
            self.mark_dirty(owner as usize);
        }
        // Another device's change between a 555's two flips is a cause of
        // its own (NE555 point 11); the 555's OUT and DIS are not.
        for flip in self.latch_flips.iter_mut().filter(|f| f.0 != owner) {
            flip.1 = true;
        }
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

    fn clone_box(&self) -> Box<dyn DeviceEvaluator> {
        Box::new(self.clone())
    }

    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        crate::snapshot::HeapBytes::heap_bytes(self, w)
    }

    /// One IC record per entry (`crate::carry`'s IC layout): `[kind, tag,
    /// family values..., reads, read x reads, slots, slot x slots, drivers,
    /// (committed, pending target, pending due) x drivers]`, `reads` being
    /// `-1` before the IC's first powered evaluation.
    fn carry_take(&self, drivers: &[crate::drivers::Driver], out: &mut Vec<f64>) {
        use crate::carry::ic;
        let mut entries: Vec<&IcEntry> = self.ics.iter().collect();
        entries.sort_by_key(|e| e.device_index);
        let mut v = Vec::new();
        for entry in entries {
            v.clear();
            v.push(f64::from(entry.record.kind));
            let tag_at = v.len();
            v.push(0.0);
            v[tag_at] = match &entry.family {
                Family::Combinational { schmitt_state } => {
                    v.push(schmitt_state.len() as f64);
                    v.extend(schmitt_state.iter().map(|&b| ic::bit(b)));
                    ic::COMBINATIONAL
                }
                Family::Sequential(state) => {
                    state.carry_values(&mut v);
                    ic::SEQUENTIAL
                }
                Family::Memory(state) => digital::memory::carry_values(state, &mut v),
                Family::Ne555(state) => {
                    v.push(ic::bit(state.out_high));
                    ic::NE555
                }
            };
            match &entry.last_reads {
                Some(reads) => {
                    v.push(reads.len() as f64);
                    v.extend(reads.iter().map(|&b| ic::bit(b)));
                }
                None => v.push(-1.0),
            }
            v.push(entry.family_slots.len() as f64);
            v.extend_from_slice(&entry.family_slots);
            v.push(entry.output_drivers.len() as f64);
            for id in entry.output_drivers.iter() {
                let d = &drivers[id.0 as usize];
                let (target, due) = d.pending().map_or((-1.0, f64::NAN), |p| (ic::mode(Some(p.target)), p.due));
                v.extend([ic::mode(d.committed()), target, due]);
            }
            crate::carry::push(out, entry.device_index, crate::carry::FAMILY_IC, &v);
        }
    }

    #[cfg(test)]
    fn debug_output_drivers(&self, device: usize) -> Vec<DriverId> {
        self.ics.iter().find(|e| e.device_index == device).map(|e| e.output_drivers.to_vec()).unwrap_or_default()
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
        let saved = Saved { family: family.clone(), last_reads: None, family_slots: family_slots.clone() };
        let saves_state = !matches!(family, Family::Combinational { .. }) || record.schmitt.is_some();
        ics.push(IcEntry {
            device_index,
            record: record.into(),
            output_drivers: output_drivers.into(),
            family,
            last_reads: None,
            family_slots,
            journaled: false,
            saved,
            saves_state,
            published: vec![0; published_len],
            powered: false,
            dirty: false,
            power_key: None,
            power: IcPower { powered: false, v_gnd: 0.0, vs: 0.0, r_out: 0.0, vil: 0.0, vih: 0.0, schmitt: None, weight: 1.0 },
        });
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
        Program { carry: Vec::new(), node_count, options: Options::default(), digital_slot_count, channel_slot_count: 0, devices }
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
        // W1g: the outputs tied together on node 7 sag from load, and their
        // `output_sag` windows run out at 1e-8 s, an instant of their own
        // before the ring closes. Complete it first, so the doomed instant is
        // the next one.
        kernel.advance(19e-9).unwrap();
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
    fn a_555_latch_that_flips_back_within_one_window_is_a_zero_delay_loop() {
        // NE555 point 11's evidence (ruling R49 (d)): reset, then set, inside
        // one rollback window. Not across windows, not after a latch due at
        // once (`new_latch_baseline`), and not outside a window (load
        // settling keeps the delta cap alone). 0=gnd, 1=vcc (5 V), 2=TRIG,
        // 3=THR, 4=OUT, 5=DIS.
        let p = program(6, 1, vec![source(0, 1, 0, 5.0), ne555_dev(1, 1, 0, [2, 4, 1, 0, 3, 5])]);
        let solver = PartitionedSolver::build(&p).expect("program must load");
        let mut kernel = Kernel::new(solver, IcEvaluator::new(Vec::new()));
        let mut ev = register(&mut kernel, &p);
        let reader = DeviceId(1);
        // THR past 2/3 resets; TRIG under 1/3 sets (TRIG beats THR).
        let reset = [0.0, 5.0, 2.0, 4.0, 0.0, 0.0];
        let set = [0.0, 5.0, 1.0, 4.0, 0.0, 0.0];
        let flip_back = |ev: &mut IcEvaluator, baseline: bool| {
            let w = ev.begin_instant();
            ev.evaluate(reader, 0.0, true, &reset);
            if baseline {
                ev.new_latch_baseline();
            }
            ev.evaluate(reader, 0.0, true, &set);
            let found = ev.zero_delay_loop();
            ev.end_instant(w);
            assert!(!ev.zero_delay_loop(), "the evidence ends with its window");
            found
        };
        assert!(flip_back(&mut ev, false), "reset then set in one window");
        assert!(!flip_back(&mut ev, true), "a latch due at once between them starts the evidence afresh");
        // One flip per window, back and forth: an astable's two thresholds.
        for v in [&reset, &set, &reset] {
            let w = ev.begin_instant();
            ev.evaluate(reader, 0.0, true, v);
            assert!(!ev.zero_delay_loop());
            ev.end_instant(w);
        }
        // A frozen pass writes no latch; outside a window nothing is kept.
        let w = ev.begin_instant();
        ev.evaluate(reader, 0.0, false, &set);
        ev.evaluate(reader, 0.0, true, &set);
        assert!(!ev.zero_delay_loop(), "a frozen pass is no flip");
        ev.rollback_instant(w);
        ev.evaluate(reader, 0.0, true, &set);
        ev.evaluate(reader, 0.0, true, &reset);
        assert!(!ev.zero_delay_loop(), "no window, no evidence");
    }

    #[test]
    fn a_zero_delay_hazard_into_a_555_settles_its_instant_rather_than_declining() {
        // A delta-cycle hazard, not a loop: one edge reaches a zero-delay
        // NAND by two paths of unequal depth (directly, and through a NAND
        // wired as an inverter), so its output glitches low for one delta.
        // The glitch is the 555's TRIG: the latch sets on it, and once the
        // glitch ends THR (on the supply) takes it back. The latch flips and
        // flips back within one instant, but another device (the NAND)
        // changed between the two flips, so its own output did not cause
        // them: the instant settles with the latch low, as classic's does.
        // 0=gnd, 1=vcc (5 V), 2=A (spdt: sel 0 gnd, 1 vcc), 3=inverted A,
        // 4=NAND(A, !A) = TRIG, 5=OUT, 6=DIS, 7 and 8 dump unused outputs.
        let mut gates = ls00(2, 1, 0, [2, 2, 3, 2, 3, 4, 0, 0, 7, 0, 0, 8], 0.0);
        gates.blob[0] = 1.0;
        let p = program(9, 5, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 0.0),
            gates,
            ne555_dev(3, 1, 0, [4, 5, 1, 0, 1, 6]),
        ]);
        let mut engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 0.0, "THR on the supply resets the latch at load, TRIG high");
        engine.set_params(&[(1, 0, 1.0)]).unwrap_or_else(|e| panic!("the hazard's instant declined: {} {}", e.code, e.message));
        assert_eq!(engine.digital_slots()[0], 0.0, "the glitch's set is taken back by THR at the same instant");
        assert!(engine.voltages()[4] > 5.0 / 3.0, "the NAND's output is high again: {}", engine.voltages()[4]);
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
    // ---- on_settled visits only the ICs that changed -----------------------

    /// A 74ls04 with only `1a` and `1y` wired (`1a` on `a`, `1y` on `y`); every
    /// other input on gnd and every other output on `dump`.
    fn inverter(ci: usize, vcc: usize, a: usize, y: usize, dump: usize, delay: f64, slot_base: usize) -> Device {
        at_slot(ls04(ci, vcc, 0, [a, y, 0, dump, 0, dump, 0, dump, 0, dump, 0, dump], [false; 12], 0.008, delay), slot_base)
    }

    #[test]
    fn a_supply_that_leaves_its_power_window_blanks_a_combinational_ic_with_nothing_else_moving() {
        // Nothing evaluates a combinational IC into a slot change, and no
        // driver of it commits, when its supply collapses: the power verdict
        // flipping is the only thing that moves its slot, so it is the only
        // thing that can put the IC on the list `on_settled` re-derives.
        //
        // 0=gnd, 1=rail (5 V), 2=vcc (spdt: sel 0 to the rail, 1 to gnd),
        // 3=1a (gnd through a resistor), 4=1y, 5=dump.
        let p = program(6, 6, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 1, 0, 0.0),
            resistor(2, 3, 0, 1_000.0),
            inverter(3, 2, 3, 4, 5, 20e-9, 0),
        ]);
        let mut engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 1.0, "1a LOW, powered: 1y HIGH");

        engine.set_params(&[(1, 0, 1.0)]).unwrap();
        engine.advance(engine.time() + 1e-6).unwrap();
        assert!(engine.voltages()[2].abs() < 0.1, "vcc collapsed to {}", engine.voltages()[2]);
        assert!(engine.digital_slots()[0].is_nan(), "unpowered: the slot goes NaN, though no driver committed anything");

        engine.set_params(&[(1, 0, 0.0)]).unwrap();
        engine.advance(engine.time() + 1e-6).unwrap();
        assert_eq!(engine.digital_slots()[0], 1.0, "powered again: the committed level shows again");
    }

    #[test]
    fn a_counters_readout_follows_its_clock_edges_after_load() {
        // A family IC's slots change only by evaluation, so an evaluation is
        // what puts it on the list `on_settled` re-derives. A 161 counting
        // edges from a switch, its enables and clear/load released on the rail.
        //
        // 0=gnd, 1=rail, 2=clock (spdt: sel 0 to gnd, 1 to the rail).
        let wire = |mut d: Device, logic: &[(usize, usize)]| {
            for &(i, node) in logic {
                d.pins[2 + i] = pin(node);
            }
            d
        };
        let p = program(3, 5, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 0.0),
            wire(w1d_ic(kinds::IC_74LS161, 2, 1, 0, 14, w1d_params(4, &[]), 0), &[(0, 1), (1, 2), (2, 1), (3, 1), (4, 1)]),
        ]);
        let mut engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 0.0);
        for expected in 1..=3 {
            engine.set_params(&[(1, 0, 1.0)]).unwrap();
            engine.advance(engine.time() + 1e-6).unwrap();
            assert_eq!(engine.digital_slots()[0], expected as f64, "the rising edge counted");
            engine.set_params(&[(1, 0, 0.0)]).unwrap();
            engine.advance(engine.time() + 1e-6).unwrap();
            assert_eq!(engine.digital_slots()[0], expected as f64, "the falling edge holds it");
        }
    }

    #[test]
    fn the_writes_of_one_instant_come_out_in_ic_order_whatever_order_the_commits_landed() {
        // Two inverters fall at the same instant, the second one's request
        // queued first (a longer delay, started earlier), so the commits apply
        // second-IC-first. `on_settled` must still return the writes the way a
        // scan over the ICs in order does: the first IC's slot, then the
        // second's.
        //
        // 0=gnd, 1=rail, 2=A's input (spdt), 3=A's output, 4=B's input (spdt),
        // 5=B's output, 6=dump. A falls 10 ns after its input rises, B 20 ns.
        let p = program(7, 12, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 0.0),
            spdt(2, 4, 0, 1, 0.0),
            inverter(3, 1, 2, 3, 6, 10e-9, 0),
            inverter(4, 1, 4, 5, 6, 20e-9, 6),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        kernel.take_digital_writes();

        kernel.solver().validate_params(&[(2, 0, 1.0)]).unwrap();
        kernel.apply_params_as_event(&[(2, 0, 1.0)]).unwrap();
        kernel.advance(10e-9).unwrap();
        kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
        kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap();
        kernel.take_digital_writes();
        kernel.advance(20e-9).unwrap();
        let writes = kernel.take_digital_writes();
        let at_20ns: Vec<(usize, f64)> = writes.iter().filter(|w| w.0 == 20e-9).map(|w| (w.1, w.2)).collect();
        assert_eq!(at_20ns, vec![(0, 0.0), (6, 0.0)], "A's 1y (slot 0) then B's 1y (slot 6), both fallen: {writes:?}");
    }

    #[test]
    fn a_quiet_instant_looks_at_no_ic_and_a_commit_looks_at_only_the_ic_it_moved() {
        // Three independent inverters, one switch on the first. Nothing
        // commits at the instant the switch moves (the output is 20 ns away)
        // and the settled instant 20 ns later commits one output, so one IC
        // is looked at over the whole run and the others never.
        //
        // 0=gnd, 1=rail, 2=switch, 3=first 1y, 4=second 1a, 5=second 1y,
        // 6=third 1a, 7=third 1y, 8=dump.
        let p = program(9, 18, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 0.0),
            inverter(2, 1, 2, 3, 8, 20e-9, 0),
            inverter(3, 1, 4, 5, 8, 20e-9, 6),
            inverter(4, 1, 6, 7, 8, 20e-9, 12),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        kernel.take_digital_writes();
        assert_eq!(kernel.evaluator().visited, 0, "the first publish is the one that looks at everything, and is not counted");

        kernel.solver().validate_params(&[(1, 0, 1.0)]).unwrap();
        kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap();
        assert_eq!(kernel.evaluator().visited, 0, "the switch moved, nothing committed");
        assert!(kernel.take_digital_writes().is_empty());

        kernel.advance(40e-9).unwrap();
        assert_eq!(kernel.evaluator().visited, 1, "one commit, one IC looked at");
        let writes = kernel.take_digital_writes();
        assert_eq!(writes.iter().map(|w| (w.1, w.2)).collect::<Vec<_>>(), vec![(0, 0.0)], "only the first inverter's 1y changed");
        assert!(kernel.evaluator().dirty.is_empty() && kernel.evaluator().ics.iter().all(|e| !e.dirty), "the list is consumed");

        kernel.advance(80e-9).unwrap();
        assert_eq!(kernel.evaluator().visited, 1, "a quiet stretch looks at nothing");
    }
    // ---- the journal's pre-image, one IC of every family --------------------

    /// Program with one IC of every kind that has its own evaluator state — a
    /// Schmitt inverter, a plain inverter, the three registers, the RAM, an
    /// EEPROM and a 555 — each logic pin on a node of its own so a test can
    /// set every read by hand, all supplied from node 1 (5 V) and node 0.
    fn one_of_every_family() -> (Program, usize) {
        // Returns the program and the node of the 555's first logic pin (TRIG;
        // RST and THR follow at +2 and +4).
        let mut next_node = 2;
        let mut fresh = |count: usize| -> Vec<usize> {
            let nodes: Vec<usize> = (next_node..next_node + count).collect();
            next_node += count;
            nodes
        };
        let mut next_slot = 0;
        let mut slots = |count: usize| {
            let base = next_slot;
            next_slot += count;
            base
        };
        let own_pins = |mut d: Device, nodes: Vec<usize>| {
            for (i, node) in nodes.into_iter().enumerate() {
                d.pins[2 + i] = pin(node);
            }
            d
        };
        let mut devices = vec![source(0, 1, 0, 5.0)];
        let ls04_nodes = fresh(12);
        devices.push(at_slot(ls04(1, 1, 0, ls04_nodes.try_into().unwrap(), [false; 12], 0.008, 20e-9), slots(6)));
        let hc14_nodes = fresh(12);
        let mut hc14 = ls04(2, 1, 0, hc14_nodes.try_into().unwrap(), [false; 12], 0.02, 15e-9);
        hc14.kind = kinds::IC_74HC14;
        hc14.params[ic_param::SCHMITT] = 1.0;
        hc14.params[ic_param::VT_PLUS] = 2.7;
        hc14.params[ic_param::VT_MINUS] = 1.6;
        devices.push(at_slot(hc14, slots(6)));
        devices.push(at_slot(own_pins(w1d_ic(kinds::IC_74LS161, 3, 1, 0, 14, w1d_params(4, &[]), 0), fresh(14)), slots(5)));
        devices.push(at_slot(own_pins(w1d_ic(kinds::IC_74LS173, 4, 1, 0, 14, w1d_params(4, &[]), 0), fresh(14)), slots(4)));
        devices.push(at_slot(own_pins(w1d_ic(kinds::IC_74HC74, 5, 1, 0, 12, w1d_params(2, &[]), 0), fresh(12)), slots(4)));
        devices.push(at_slot(own_pins(w1d_ic(kinds::IC_74LS189, 6, 1, 0, 14, w1d_params(5, &[]), 0), fresh(14)), slots(4)));
        devices.push(at_slot(own_pins(w1d_ic(kinds::IC_28C16, 7, 1, 0, 22, w1d_params(5, &[10e-3, 0.0]), 2048 / 4), fresh(22)), slots(8)));
        let ne555_nodes = fresh(6);
        let first_555_node = ne555_nodes[0];
        let mut ne555 = ne555_dev(8, 1, 0, ne555_nodes.try_into().unwrap());
        ne555.blob[0] = slots(1) as f64;
        devices.push(ne555);
        (program(next_node, next_slot, devices), first_555_node)
    }

    /// The evaluator state one IC's rollback has to restore.
    fn ic_state(evaluator: &IcEvaluator, at: usize) -> String {
        let e = &evaluator.ics[at];
        format!("{:?}", (&e.family, &e.last_reads, &e.family_slots))
    }

    #[test]
    fn every_familys_pre_image_comes_back_from_a_rolled_back_window_and_a_completed_window_keeps_its_writes() {
        let (p, first_555_node) = one_of_every_family();
        let node_count = p.node_count;
        let (mut kernel, ..) = build(&p);
        let evaluator = kernel.evaluator_mut();
        let readers: Vec<DeviceId> = evaluator.reader_ids();
        assert_eq!(readers.len(), 8);

        // Every logic pin's node set to a random rail, the supplies as wired.
        let mut seed = 0x9e37_79b9u32;
        let mut scramble = |volts: &mut Vec<f64>| {
            for v in volts.iter_mut().skip(2) {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                *v = if seed & 0x100 != 0 { 5.0 } else { 0.0 };
            }
        };
        let mut volts = vec![0.0; node_count];
        volts[1] = 5.0;
        let mut now = 0.0;
        let mut requests = Vec::new();
        let mut run = |evaluator: &mut IcEvaluator, volts: &mut Vec<f64>, now: &mut f64, passes: usize, script: &mut dyn FnMut(&mut Vec<f64>)| {
            for _ in 0..passes {
                script(volts);
                *now += 1e-6;
                for &reader in &readers {
                    requests.clear();
                    evaluator.evaluate_impl(reader, *now, true, volts, &[], None, &mut requests);
                }
            }
        };

        // Away from power-on first, with no window open, so the state a
        // window saves is not the one the entries were registered with. The
        // last pass resets the 555's latch (TRIG, THR and RST all HIGH), which
        // the window's own last pass sets again, so the latch is known to
        // differ across the window whatever the random passes between did.
        run(evaluator, &mut volts, &mut now, 40, &mut scramble);
        run(evaluator, &mut volts, &mut now, 1, &mut |volts: &mut Vec<f64>| {
            for offset in [0, 2, 4] {
                volts[first_555_node + offset] = 5.0;
            }
        });
        let before: Vec<String> = (0..readers.len()).map(|at| ic_state(evaluator, at)).collect();

        // The same scripted passes, once rolled back and once kept. The
        // script is a copy of the generator's state, so both see one sequence.
        let script_seed = seed;
        let scripted = |seed: u32| {
            let mut seed = seed;
            let mut pass = 0;
            move |volts: &mut Vec<f64>| {
                for v in volts.iter_mut().skip(2) {
                    seed ^= seed << 13;
                    seed ^= seed >> 17;
                    seed ^= seed << 5;
                    *v = if seed & 0x100 != 0 { 5.0 } else { 0.0 };
                }
                pass += 1;
                if pass == 30 {
                    // TRIG and THR LOW, RST HIGH: the latch sets.
                    volts[first_555_node] = 0.0;
                    volts[first_555_node + 2] = 5.0;
                    volts[first_555_node + 4] = 0.0;
                }
            }
        };

        let mut volts_a = volts.clone();
        let mut now_a = now;
        evaluator.begin_instant();
        run(evaluator, &mut volts_a, &mut now_a, 30, &mut scripted(script_seed));
        let written: Vec<String> = (0..readers.len()).map(|at| ic_state(evaluator, at)).collect();
        for at in 0..readers.len() {
            assert_ne!(before[at], written[at], "IC {at}: the window's evaluations must have changed its state, or the rollback below proves nothing");
        }
        // Each family's own state (not only its reads and slots) moved in at
        // least one IC of that family.
        evaluator.rollback_instant(Box::new(()));
        for at in 0..readers.len() {
            assert_eq!(before[at], ic_state(evaluator, at), "IC {at} is back to its pre-image");
        }
        assert!(!evaluator.journaling && evaluator.journal.is_empty() && evaluator.ics.iter().all(|e| !e.journaled));

        // The same passes again, this time completed: the writes stay, and
        // they are what the rolled-back run had written.
        let mut volts_b = volts.clone();
        let mut now_b = now;
        evaluator.begin_instant();
        run(evaluator, &mut volts_b, &mut now_b, 30, &mut scripted(script_seed));
        evaluator.end_instant(Box::new(()));
        for at in 0..readers.len() {
            assert_eq!(written[at], ic_state(evaluator, at), "IC {at}: a completed window keeps what it wrote, and replaying from the restored pre-image reproduces it");
        }
        assert!(!evaluator.journaling && evaluator.journal.is_empty() && evaluator.ics.iter().all(|e| !e.journaled));
    }
    #[test]
    fn evaluate_into_leaves_out_exactly_the_requests_the_driver_would_ignore() {
        // At a settled state every level an IC asks for is one its driver
        // already holds, so `evaluate_into` (given the driver table) returns
        // no `Level`; `evaluate` (without one) returns them all. Put one
        // driver's opposite level in flight and its request, and only its
        // request, is one the driver no longer ignores.
        let (p, _) = one_of_every_family();
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        let volts = kernel.solver().voltages().to_vec();
        let driver_count: u32 = kernel.evaluator().ics.iter().map(|e| e.output_drivers.len() as u32).sum();
        let drivers: Vec<Driver> = (0..driver_count).map(|i| *kernel.driver(DriverId(i))).collect();
        let levels_of = |requests: &[DriverRequest]| -> Vec<(DriverId, Mode)> {
            requests
                .iter()
                .filter_map(|r| match r {
                    DriverRequest::Level { driver, level, .. } => Some((*driver, *level)),
                    _ => None,
                })
                .collect()
        };

        let mut levels_checked = 0;
        for reader in all_readers {
            let now = 1e-6;
            let asked = levels_of(&kernel.evaluator_mut().evaluate(reader, now, true, &volts));
            let mut filtered = Vec::new();
            kernel.evaluator_mut().evaluate_into(reader, now, true, &volts, &[], &drivers, &mut filtered);
            assert!(levels_of(&filtered).is_empty(), "{reader:?}: at a settled state nothing is worth asking, got {filtered:?}");

            for &(driver, level) in &asked {
                let mut perturbed = drivers.clone();
                let opposite = if level == Mode::High { Mode::Low } else { Mode::High };
                perturbed[driver.0 as usize].request(opposite, now, &DelaySet::uniform(1e-9));
                filtered.clear();
                kernel.evaluator_mut().evaluate_into(reader, now, true, &volts, &[], &perturbed, &mut filtered);
                assert_eq!(levels_of(&filtered), vec![(driver, level)], "{reader:?}: only the disturbed driver's request survives");
                levels_checked += 1;
            }
        }
        assert_eq!(levels_checked, driver_count as usize, "every driver of every family was checked");
    }

    #[test]
    fn the_cached_power_state_follows_each_supply_pin_and_never_goes_stale() {
        // `evaluate_impl` reuses an IC's `ic_power` while neither supply
        // voltage's bits moved. Every pair below is checked against a fresh
        // `ic_power`, and the sequence moves each supply on its own (the key
        // must hold both), repeats a pair (a hit), goes unpowered and comes
        // back. The Schmitt part's thresholds and every part's output
        // resistance scale with the supply, so a stale entry is visible.
        let (p, _) = one_of_every_family();
        let node_count = p.node_count;
        let (mut kernel, ..) = build(&p);
        let evaluator = kernel.evaluator_mut();
        let readers = evaluator.reader_ids();
        let mut volts = vec![0.0; node_count];
        let mut out = Vec::new();
        let mut distinct = std::collections::HashSet::new();
        let mut unpowered = 0;
        let supplies = [(5.0, 0.0), (5.0, 0.0), (5.0, 0.4), (4.6, 0.4), (4.6, 0.4), (0.0, 0.0), (5.0, 0.0), (5.5, 0.5), (3.0, 0.0), (5.0, 0.0), (5.0, 0.0)];
        let mut now = 0.0;
        for &(v_vcc, v_gnd) in &supplies {
            volts[1] = v_vcc;
            volts[0] = v_gnd;
            now += 1e-6;
            for &reader in &readers {
                out.clear();
                evaluator.evaluate_impl(reader, now, true, &volts, &[], None, &mut out);
                let at = evaluator.position(reader.0 as usize).unwrap();
                let entry = &evaluator.ics[at];
                if matches!(entry.family, Family::Ne555(_)) {
                    continue; // dispatched before any power derivation
                }
                let fresh = digital::power::ic_power(&entry.record, v_vcc, v_gnd);
                assert_eq!(format!("{:?}", entry.power), format!("{fresh:?}"), "IC {at} with vcc {v_vcc}, gnd {v_gnd}");
                distinct.insert(format!("{:?}", entry.power));
                unpowered += usize::from(!entry.power.powered);
            }
        }
        assert!(unpowered > 0, "an unpowered verdict was cached and checked");
        assert!(distinct.len() > 8, "the supplies changed what was derived: {} distinct states", distinct.len());
    }

    // ---- F5a: the power hold (W1c point 1a) -----------------------------

    /// A `74ls00`: `pins[i]` names logic pin `i` (`1a 1b 1y 2a 2b 2y ...`).
    fn ls00(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12], delay: f64) -> Device {
        let mut device_pins = vec![pin(vcc), pin(gnd)];
        device_pins.extend(pins.iter().map(|&n| pin(n)));
        let mut blob = vec![0.0];
        blob.extend((0..12).map(|i| if i % 3 != 2 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }));
        Device { kind: kinds::IC_74LS00, component_index: ci, pins: device_pins, params: ls_ttl_params(0.008, delay), blob }
    }

    #[test]
    fn a_held_off_ic_on_an_ideal_supply_stamps_reads_and_commits_nothing_and_its_hold_rolls_back() {
        use crate::digital::power::PowerHold;
        // A hold makes an IC unpowered whatever its supply reads (W1c point
        // 1a), here on an ideal 5 V supply the procedure itself never holds
        // (its draw cannot move it), so only the hold can be what changes.
        //
        // 0=gnd, 1=vcc, 2=1a=1b (spdt: sel 0 to gnd, 1 to vcc), 3=1y between
        // two 10k (2.5 V with nothing stamped), 4=dump.
        let ic = 4usize;
        let p = program(5, 4, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 0.0),
            resistor(2, 1, 3, 10_000.0),
            resistor(3, 3, 0, 10_000.0),
            ls00(4, 1, 0, [2, 2, 3, 0, 0, 4, 0, 0, 4, 0, 0, 4], 20e-9),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        let mut slots = [f64::NAN; 4];
        let drain = |kernel: &mut Kernel<PartitionedSolver, IcEvaluator>, slots: &mut [f64; 4]| {
            for (_, slot, v) in kernel.take_digital_writes() {
                slots[slot] = v;
            }
        };
        drain(&mut kernel, &mut slots);
        assert!(kernel.solver().voltages()[3] > 4.5, "1a LOW: 1y driven HIGH, {}", kernel.solver().voltages()[3]);
        assert_eq!(slots[0], 1.0);

        // Inputs HIGH: 1y LOW is requested, due 20 ns on.
        kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap();
        drain(&mut kernel, &mut slots);
        let y = kernel.evaluator().ics[0].output_drivers[0];
        assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "the LOW is still in flight");
        let reads = ic_state(kernel.evaluator(), 0);

        // Held off at this instant: no stamp (the divider alone, which a
        // cached powered solution would not give), no read, NaN slots.
        let hits = kernel.solver().cache_hits();
        kernel.hold_power_as_event(ic, PowerHold::off()).unwrap();
        drain(&mut kernel, &mut slots);
        assert!(!kernel.solver().ic_powered(ic));
        // The divider and the node shunt alone: 2.5 V less 12.5 nV.
        let divider = kernel.solver().voltages()[3];
        assert!((divider - 2.5).abs() < 1e-7, "a held IC stamps nothing: {divider}");
        assert_eq!(kernel.solver().cache_hits(), hits, "the powered solution must not answer for a held IC");
        assert!(slots.iter().all(|s| s.is_nan()), "every slot of a held IC is NaN: {slots:?}");

        // The LOW falls due while held: held, not applied (point 9).
        kernel.advance(1e-6).unwrap();
        drain(&mut kernel, &mut slots);
        assert_eq!(kernel.driver(y).committed(), Some(Mode::High), "a held IC's commit is held");
        assert_eq!(kernel.solver().voltages()[3], divider);
        assert_eq!(ic_state(kernel.evaluator(), 0), reads, "a held IC reads nothing");
        assert!(slots.iter().all(|s| s.is_nan()));

        // The table rolls back with a failed instant and with a snapshot.
        let solver = kernel.solver_mut();
        let window = solver.begin_instant();
        solver.set_power_hold(ic, PowerHold::FREE);
        assert_eq!(solver.power_hold(ic), PowerHold::FREE);
        solver.rollback_instant(window);
        assert_eq!(solver.power_hold(ic), PowerHold::off(), "rollback restores the hold");
        let snap = solver.dynamic_snapshot();
        solver.set_power_hold(ic, PowerHold::FREE);
        solver.dynamic_restore(snap);
        assert_eq!(solver.power_hold(ic), PowerHold::off(), "a restored snapshot carries the hold");
        assert!(!solver.ic_powered(ic));

        // `resetFailures` drops every hold: the IC reads at once and the held
        // LOW applies.
        kernel.reset_failures_as_event().unwrap();
        drain(&mut kernel, &mut slots);
        assert_eq!(kernel.solver().power_hold(ic), PowerHold::FREE);
        assert!(kernel.solver().ic_powered(ic));
        kernel.advance(2e-6).unwrap();
        drain(&mut kernel, &mut slots);
        assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "the held commit applied once released");
        assert!(kernel.solver().voltages()[3] < 0.3, "1y driven LOW, {}", kernel.solver().voltages()[3]);
        assert_eq!(slots[0], 0.0);
        // One hold taken; the two releases are the rolled-back ones above
        // (stats keep a failed attempt's counts); the drop is not a release.
        assert_eq!(kernel.solver().hold_stats(), [1.0, 2.0]);
    }

    #[test]
    fn a_sliding_ic_drives_at_its_duty_reads_and_commits_and_its_duty_rolls_back() {
        // Ruling R92 (Q4): a chip sliding along its power edge (Duty(x)) stays
        // powered, its logic running, and drives its outputs at x times its
        // powered conductance (Filippov's average with the unpowered stamp,
        // which is nothing). On an ideal 5 V supply only the hold can change
        // anything.
        //
        // 0=gnd, 1=vcc, 2=1a=1b (spdt: sel 0 to gnd), 3=1y between two 10k,
        // 4=dump. 1a LOW: 1y HIGH through R_out = 156.25 ohm (io_max 8 mA at
        // 5 V) to vcc.
        use crate::digital::power::PowerHold;
        let ic = 4usize;
        let p = program(5, 4, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 0.0),
            resistor(2, 1, 3, 10_000.0),
            resistor(3, 3, 0, 10_000.0),
            ls00(4, 1, 0, [2, 2, 3, 0, 0, 4, 0, 0, 4, 0, 0, 4], 20e-9),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        let mut slots = [f64::NAN; 4];
        let drain = |kernel: &mut Kernel<PartitionedSolver, IcEvaluator>, slots: &mut [f64; 4]| {
            for (_, slot, v) in kernel.take_digital_writes() {
                slots[slot] = v;
            }
        };
        drain(&mut kernel, &mut slots);
        let free = kernel.solver().voltages()[3];
        // By hand: the node sees 5 V through 10k and through 156.25 / x ohm,
        // and 0 V through 10k (the node shunt's nanovolts aside).
        let by_hand = |x: f64| {
            let (g, g_out) = (1e-4, x / 156.25);
            5.0 * (g + g_out) / (2.0 * g + g_out)
        };
        assert!((free - by_hand(1.0)).abs() < 1e-6, "free: {free} against {}", by_hand(1.0));

        let x = 0.25;
        kernel.hold_power_as_event(ic, PowerHold::duty(x)).unwrap();
        drain(&mut kernel, &mut slots);
        assert!(kernel.solver().ic_powered(ic), "a sliding chip is powered");
        let sliding = kernel.solver().voltages()[3];
        assert!((sliding - by_hand(x)).abs() < 1e-6, "drive at its duty: {sliding} against {}", by_hand(x));
        assert_eq!(slots[0], 1.0, "its logic reads and publishes");
        // Each duty is its own stamp: a solution cached at one never answers
        // for another.
        for x2 in [0.5, 0.25, 1.0 / 3.0] {
            kernel.hold_power_as_event(ic, PowerHold::duty(x2)).unwrap();
            let v = kernel.solver().voltages()[3];
            assert!((v - by_hand(x2)).abs() < 1e-6, "duty {x2}: {v} against {}", by_hand(x2));
        }
        kernel.hold_power_as_event(ic, PowerHold::duty(x)).unwrap();

        // Its logic runs: inputs HIGH, the LOW commits 20 ns on, driven at x
        // (a `set_params` drops every hold, so the duty is set again before
        // the LOW falls due).
        kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap();
        kernel.hold_power_as_event(ic, PowerHold::duty(x)).unwrap();
        kernel.advance(1e-6).unwrap();
        drain(&mut kernel, &mut slots);
        let y = kernel.evaluator().ics[0].output_drivers[0];
        assert_eq!(kernel.driver(y).committed(), Some(Mode::Low), "a sliding chip commits");
        let low = kernel.solver().voltages()[3];
        let low_by_hand = 5.0 * 1e-4 / (2.0 * 1e-4 + x / 156.25);
        assert!((low - low_by_hand).abs() < 1e-6, "LOW at its duty: {low} against {low_by_hand}");
        assert_eq!(slots[0], 0.0);

        // The duty rolls back with a failed instant and with a snapshot.
        let solver = kernel.solver_mut();
        let window = solver.begin_instant();
        solver.set_power_hold(ic, PowerHold::duty(0.5));
        solver.rollback_instant(window);
        assert_eq!(solver.power_hold(ic), PowerHold::duty(x), "rollback restores the duty");
        let snap = solver.dynamic_snapshot();
        solver.set_power_hold(ic, PowerHold::FREE);
        solver.dynamic_restore(snap);
        assert_eq!(solver.power_hold(ic), PowerHold::duty(x), "a restored snapshot carries the duty");
    }

    #[test]
    fn the_kernels_hold_flag_reads_the_table_through_every_change_rollback_and_restore() {
        // F5q: the kernel skips the hold table on this flag, so a stale
        // `false` would evaluate a held IC as a free one.
        use crate::digital::power::PowerHold;
        use crate::solve::PartitionSolver;
        let ic = 4usize;
        let p = program(5, 4, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 0.0),
            resistor(2, 1, 3, 10_000.0),
            resistor(3, 3, 0, 10_000.0),
            ls00(4, 1, 0, [2, 2, 3, 0, 0, 4, 0, 0, 4, 0, 0, 4], 20e-9),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        let flag = kernel.solver().holds_flag().expect("the partitioned solver shares its flag");
        let agrees = |solver: &PartitionedSolver| flag.get() == !solver.power_holds().is_empty();
        assert!(!flag.get() && agrees(kernel.solver()));
        kernel.hold_power_as_event(ic, PowerHold::off()).unwrap();
        assert!(flag.get() && agrees(kernel.solver()), "a hold sets it");
        let solver = kernel.solver_mut();
        let window = solver.begin_instant();
        solver.set_power_hold(ic, PowerHold::FREE);
        assert!(!flag.get() && agrees(solver), "a release clears it");
        solver.rollback_instant(window);
        assert!(flag.get() && agrees(solver), "a rollback sets it back");
        let snap = solver.dynamic_snapshot();
        solver.set_power_hold(ic, PowerHold::FREE);
        assert!(!flag.get());
        solver.dynamic_restore(snap);
        assert!(flag.get() && agrees(solver), "a restored snapshot sets it back");
        kernel.reset_failures_as_event().unwrap();
        assert!(!flag.get() && agrees(kernel.solver()), "dropping every hold clears it");
    }

    #[test]
    fn a_cloned_kernel_gets_its_own_holds_flag_linked_into_its_own_solver() {
        // W2a: `derive(Clone)` would alias the `Rc<Cell<bool>>` the kernel
        // and the solver share, so a hold taken in a snapshot would flip the
        // live kernel's flag (and the reverse).
        use crate::digital::power::PowerHold;
        use crate::solve::PartitionSolver;
        let ic = 4usize;
        let p = program(5, 4, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 2, 0, 1, 0.0),
            resistor(2, 1, 3, 10_000.0),
            resistor(3, 3, 0, 10_000.0),
            ls00(4, 1, 0, [2, 2, 3, 0, 0, 4, 0, 0, 4, 0, 0, 4], 20e-9),
        ]);
        let (mut kernel, all_partitions, all_readers) = build(&p);
        kernel.settle(0.0, &all_partitions, &all_readers).unwrap();
        let mut copy = kernel.clone();
        let flag = kernel.solver().holds_flag().unwrap();
        let copy_flag = copy.solver().holds_flag().unwrap();
        assert!(!std::rc::Rc::ptr_eq(&flag, &copy_flag), "the clone must get a cell of its own");
        assert!(std::rc::Rc::ptr_eq(copy.holds_flag_cell().unwrap(), &copy_flag), "the clone's kernel must read its own solver's cell");
        copy.hold_power_as_event(ic, PowerHold::off()).unwrap();
        assert!(copy_flag.get() && !flag.get(), "a hold in the clone leaves the original's flag alone");
        kernel.hold_power_as_event(ic, PowerHold::off()).unwrap();
        copy.reset_failures_as_event().unwrap();
        assert!(flag.get() && !copy_flag.get(), "and the reverse");
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Family {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Family::Combinational { schmitt_state } => crate::snapshot::HeapBytes::heap_bytes(schmitt_state, w),
            Family::Sequential(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            Family::Memory(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            Family::Ne555(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for IcEntry {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.record, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.output_drivers, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.family, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_reads, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.family_slots, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.journaled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.saved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.saves_state, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.published, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.powered, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dirty, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.power_key, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.power, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SupplyNode {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.bits, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ics, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Saved {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.family, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_reads, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.family_slots, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for IcEvaluator {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ics, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.evaluations, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.position_of_device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.journaling, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.journal, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.latch_flips, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.latch_loop, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.levels, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.results, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.family_out, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.published_once, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dirty, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.driver_owner, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.supplies, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seen_holds, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.visited, w); }
        n
    }
}
