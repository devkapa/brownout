//! The engine: `Engine::load`/`Engine::advance` wire the event kernel
//! (`kernel.rs`) to concrete state through [`solve::PartitionSolver`].
//! `Engine` is a plain, safe Rust type with no `extern "C"` on it anywhere;
//! `abi.rs` is the only place that touches raw pointers or a handle table.
//!
//! The static partition solver (`partition::build`'s topology, the dense LU
//! solve per partition, the KCL current pass) lived here through W1a as a
//! single self-contained struct, and moved to `solve::static_part` in W1b
//! (plan row B1) once real partitioning/caching/switch-family stamping
//! existed to justify the module. `Engine`'s own `load`/`advance` stay thin:
//! build the static topology, hand it to a `Kernel`, and let the kernel's
//! settle/advance own every event-kernel concern (contract point 6, decision
//! 8; contract point 3, decision 4). No W0/W1b kind (`resistor`,
//! `voltage_source`, the switch family) ever creates a `Driver`, so for a
//! program with no digital kind nothing is ever dirty beyond the one full
//! solve `settle` performs at load — every W0 behaviour and test stays
//! exactly as it was. W1c's combinational ICs (`devices::ics`) are the first
//! kind that does: `Engine::load` registers one `Driver` per output-capable
//! pin and installs `devices::ics::IcEvaluator` as the kernel's real
//! evaluator before settling, and every `advance`/`set_params` call drains
//! `Kernel::take_digital_writes` into this struct's own `Readout` afterward
//! (`Self::apply_digital_writes`), at each write's own recorded instant.
//!
//! `#[cfg(test)]` at the bottom adds a synthetic driver/device pair (gated
//! out of every real build) that proves `Engine::advance` actually routes
//! through the kernel end to end: a driver commit scheduled for a future
//! time changes a node voltage through the static solver, and the kernel's
//! own counters record it.

#[cfg(test)]
use crate::drivers::Driver;
use crate::devices::ics;
use crate::error::BoError;
use crate::events::{DeviceId, PartitionId};
use crate::kernel::{DeviceEvaluator, DriverRequest, Kernel, KernelError, SeedMemoryError};
use crate::program::{switch_value, Program};
use crate::readout::Readout;
use crate::solve::PartitionSolver;
use crate::solve::PartitionedSolver;

/// `bo_engine_advance` rejects a `t_end` this far or more behind the current
/// time (contract, point 6). A plain `t_end < time` would re-reject a caller
/// re-issuing the same `t_end` it just advanced to, purely from float noise.
const TIME_EPS: f64 = 1e-15;

pub const STAT_PARTITIONS: u32 = 0;
pub const STAT_STATIC_SOLVES: u32 = 1;
pub const STAT_LARGEST_PARTITION: u32 = 2;
pub const STAT_FIXED_NODES: u32 = 3;
// Indices 4, 5, 6 and 13 (events processed, delta passes, maximum delta
// depth, driver commits) come live from `Kernel::stats`; 7 (solution-cache
// hits), 8 (closed-form segments), 9 (crossings), 10 (stepped steps) and 11
// (Newton iterations) come live from the partition solver (W1b step 2); 12
// (IC evaluations) comes live from the kernel's own evaluator
// (`kernel::DeviceEvaluator::ic_evaluations`, W1c) — see `Engine::stat`. 14
// (rail moves) stays reserved at 0 until W1f builds that machinery;
// `Engine::stat` still answers it (as 0), not `NaN`, so a caller can't
// mistake "not implemented yet" for "unknown index" — that stays reserved
// for indices past the contract's table (>= `STAT_COUNT`).
pub const STAT_MULTI_CAPACITOR_LINEAR_PARTITIONS: u32 = 15;
/// W1b's report to the orchestrator: index 15 is NOT in core/README.md's
/// "Stat indices" table yet (that file is off limits to this worker) — it is
/// exposed here ahead of the contract being updated to name it, per the W1
/// plan row B2's "W1b reports the count of multi-capacitor linear
/// partitions in the corpus". The `bo_engine_stat` export list itself is
/// unaffected either way (it already accepts any `u32` index).
const STAT_COUNT: usize = 16;

/// The placeholder evaluator every `Engine::load` installs a `Kernel` with
/// FIRST, before any `Driver` exists — `devices::ics::register` needs a live
/// `Kernel` to assign driver ids through (`Kernel::add_driver`), so building
/// the real `devices::ics::IcEvaluator` is necessarily a two-step "construct
/// the kernel, then register, then `Kernel::set_evaluator`" sequence, exactly
/// like `solve::ic_tests`'s own `IcHarness` build order. A program with no
/// IC (or no digital kind at all, W0/W1b) never swaps this out, so it is
/// also, still, `Engine`'s real evaluator for every such program — not just
/// a load-time scaffold.
#[derive(Debug)]
struct NoDevices;

impl DeviceEvaluator for NoDevices {
    fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
        Vec::new()
    }
}

/// Boxed rather than the concrete `Kernel<StaticPartitionSolver, NoDevices>`
/// so `Engine`'s `#[cfg(test)]` harness can swap in a mock solver/evaluator
/// pair through `Kernel::set_solver`/`set_evaluator` and prove
/// `Engine::advance` propagates a livelock end to end — `StaticPartitionSolver`
/// itself never produces one (it reports no readers at all, so `advance`'s
/// delta cycle always settles after the one pass that applies a commit), so
/// that proof needs a solver capable of a different shape at runtime. The
/// extra vtable indirection this costs every call is a deliberate, currently
/// cheap trade for that testability; W1b's real solvers may revisit it.
type EngineKernel = Kernel<Box<dyn PartitionSolver>, Box<dyn DeviceEvaluator>>;

/// Maps a [`KernelError`] onto the `BoError` `Engine::load`/`Engine::advance`
/// return: `Livelock` to `-7` (contract, "Kernel semantics: events", point 3;
/// `ERR_DELTA_LIVELOCK`), `SolveFailed` to `-4` (`ERR_SOLVE_FAILED`) with the
/// solver's own message carried through unchanged.
fn map_kernel_error(e: KernelError) -> BoError {
    match e {
        KernelError::Livelock { at } => BoError::delta_livelock(format!("delta-cycle livelock at t = {at}")),
        KernelError::SolveFailed(message) => BoError::solve_failed(message),
    }
}

// No `derive(Debug)`: `EngineKernel`'s solver/evaluator are trait objects
// (`Box<dyn PartitionSolver>`/`Box<dyn DeviceEvaluator>`, needed so the
// `#[cfg(test)]` harness below can swap them — see `EngineKernel`'s doc
// comment), and neither trait requires `Debug` of its implementors. A manual,
// minimal impl follows instead: `Result::unwrap_err` on a `Result<Engine,
// BoError>` needs `Engine: Debug` for the success case's panic message, which
// several tests below exercise (`Engine::load(&p).unwrap_err()`).
pub struct Engine {
    node_count: usize,
    device_count: usize,
    kernel: EngineKernel,
    stats: [f64; STAT_COUNT],
    /// The program's options, kept so `set_option` has something to update.
    /// W1a's static kernel does not yet read any of these back into the
    /// solve (that arrives with the partition and digital sub-waves); this
    /// only stores what a live `bo_engine_set_option` call is allowed to
    /// change and validates against.
    options: crate::program::Options,
    readout: Readout,
    /// The last `bo_engine_take_eeprom_updates` call's own buffer
    /// (`Self::take_eeprom_updates`'s doc comment) — kept here so
    /// `Self::eeprom_updates`'s pointer stays valid "until the next call on
    /// this engine" (the contract's own promise for every such pointer),
    /// exactly like `Readout`'s digital/channel slot arrays.
    eeprom_updates: Vec<f64>,
}

impl std::fmt::Debug for Engine {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Engine")
            .field("node_count", &self.node_count)
            .field("device_count", &self.device_count)
            .field("time", &self.kernel.time())
            .field("voltages", &self.voltages())
            .field("currents", &self.currents())
            .finish_non_exhaustive()
    }
}

impl Engine {
    pub fn node_count(&self) -> u32 {
        self.node_count as u32
    }

    pub fn device_count(&self) -> u32 {
        self.device_count as u32
    }

    pub fn time(&self) -> f64 {
        self.kernel.time()
    }

    pub fn voltages(&self) -> &[f64] {
        self.kernel.solver().voltages()
    }

    pub fn currents(&self) -> &[f64] {
        self.kernel.solver().currents()
    }

    /// Counters (core/README.md, "Stat indices"). Indices 0-3 and 15 are the
    /// load-time snapshot (partition classification never changes after
    /// load); 4, 5, 6 and 13 (events processed, delta passes, maximum delta
    /// depth, driver commits) come live from the kernel; 7-12 (solution-cache
    /// hits, closed-form segments, crossings, stepped steps, Newton
    /// iterations, IC evaluations) come live from the partition solver and
    /// the kernel's own evaluator; 14 is reserved at `0.0` until its
    /// machinery lands (W1f). Unknown `i` (>= `STAT_COUNT`) is `NaN`.
    pub fn stat(&self, i: u32) -> f64 {
        match i {
            4 => self.kernel.stats().events_processed as f64,
            5 => self.kernel.stats().delta_passes as f64,
            6 => self.kernel.stats().max_delta_depth as f64,
            7 => self.kernel.solver().cache_hits(),
            8 => self.kernel.solver().closed_form_segments(),
            9 => self.kernel.solver().crossings(),
            10 => self.kernel.solver().stepped_steps(),
            11 => self.kernel.solver().newton_iterations(),
            12 => self.kernel.evaluator().ic_evaluations(),
            13 => self.kernel.stats().driver_commits as f64,
            _ => self.stats.get(i as usize).copied().unwrap_or(f64::NAN),
        }
    }

    /// Time of the earliest pending event, `+Infinity` when none
    /// (core/README.md, `bo_engine_next_event_time`). `&mut self` because
    /// finding the true next event means permanently discarding any stale,
    /// cancelled `Commit` sitting ahead of it on the heap (`Kernel::next_event_time`).
    pub fn next_event_time(&mut self) -> f64 {
        self.kernel.next_event_time()
    }

    /// Time of the earliest pending driver commit, `+Infinity` when none
    /// (core/README.md, `bo_engine_next_digital_due_time`; finding 10/8).
    /// `&self`, not `&mut`: see `Kernel::next_digital_due_time`'s own doc
    /// comment on why this one never needs to permanently discard anything
    /// to answer correctly.
    pub fn next_digital_due_time(&self) -> f64 {
        self.kernel.next_digital_due_time()
    }

    pub fn digital_slots(&self) -> &[f64] {
        self.readout.digital_slots()
    }

    pub fn digital_slot_count(&self) -> u32 {
        self.readout.digital_slots().len() as u32
    }

    pub fn channel_slots(&self) -> &[f64] {
        self.readout.channel_slots()
    }

    pub fn channel_slot_count(&self) -> u32 {
        self.readout.channel_slots().len() as u32
    }

    pub fn trace_enable(&mut self, on: bool) {
        self.readout.trace_enable(on);
    }

    pub fn trace(&self) -> &[f64] {
        self.readout.trace()
    }

    pub fn trace_clear(&mut self) {
        self.readout.trace_clear();
    }

    /// Writes `value` into digital slot `slot` at time `t`, recording a
    /// trace entry only when the value actually changed
    /// (`bo_engine_trace_enable`'s contract: "Record every digital slot
    /// change", not every write attempt — a slot re-written to the value it
    /// already held is not a change, and classic itself rebuilds
    /// `digitalState` from scratch every pass, so a re-affirmed value is the
    /// ordinary case, not the exception). NaN stands for "absent" (contract,
    /// "Readouts"), so absent-to-absent is never a change either, even
    /// though `NaN != NaN` under `==`. `t` is a parameter, not always
    /// `self.kernel.time()`, because [`Self::apply_digital_writes`] replays
    /// entries at THEIR OWN recorded instant — possibly earlier than the
    /// kernel's current time, when `advance` completed several instants in
    /// one call (W1c contract point 3/10: the harness aligns digital traces
    /// on each change's exact instant, not on whenever `Engine` regains
    /// control).
    fn publish_digital(&mut self, t: f64, slot: usize, value: f64) {
        let current = self.readout.digital_slots()[slot];
        let changed = current.is_nan() != value.is_nan() || (!current.is_nan() && current != value);
        if changed {
            self.readout.record(t, slot, value);
        }
        self.readout.digital_slots_mut()[slot] = value;
    }

    /// Writes `value` into digital slot `slot` at the CURRENT kernel time —
    /// the harness/test entry point ([`Self::publish_digital`]'s own doc
    /// comment covers the general rule; every real digital-slot write goes
    /// through [`Self::apply_digital_writes`] instead, which knows each
    /// write's own instant).
    pub fn set_digital_slot(&mut self, slot: usize, value: f64) {
        let t = self.kernel.time();
        self.publish_digital(t, slot, value);
    }

    /// Drains every `(t, slot, value)` write the kernel's evaluator queued
    /// via [`crate::kernel::DeviceEvaluator::on_settled`] since the last
    /// call (`Kernel::take_digital_writes`) and republishes each one at its
    /// own recorded instant. Called after `Engine::load`'s settle and after
    /// every `Engine::advance`/`Engine::set_params` call — REGARDLESS of
    /// whether that call itself succeeded: `advance` can complete several
    /// instants before a LATER one livelocks or fails to solve, and those
    /// earlier instants' writes are real, already-committed state (core/
    /// README.md, "A failed advance": "the engine keeps the state of the
    /// last instant it completed"), not part of the doomed attempt that
    /// follows them — nothing is ever queued for the doomed instant itself
    /// (`Kernel::pending_digital_writes`'s own doc comment), so draining
    /// unconditionally is exactly correct, never a source of spurious writes
    /// from a failed call.
    fn apply_digital_writes(&mut self) {
        for (t, slot, value) in self.kernel.take_digital_writes() {
            self.publish_digital(t, slot, value);
        }
    }

    /// Changes an option after load (core/README.md, `bo_engine_set_option`).
    /// Keys 2 and 3 (the exactness switches) take effect at the next event;
    /// keys 1 and 4-7 are load-time only and are always
    /// `ERR_INVALID_ARGUMENT` here, same as any key this build does not
    /// recognize — `START_TIME` (7) names the instant the kernel's clock
    /// already started at, which cannot be moved after the fact any more
    /// than `INTEGRATION` or `AMBIENT_C` can, and `FORCE_STEPPED` (4) is
    /// load-time only for the same reason: `partition::build` folds it into
    /// each partition's `PartitionKind` (Decision 14) exactly once, at load,
    /// and nothing ever re-partitions or migrates a partition between
    /// `solve::closed_form::ClosedFormSolver` and `solve::stepped::SteppedSolver`
    /// afterward — a post-load `set_option(4, ...)` that merely recorded the
    /// new value here would return success while every already-built
    /// partition kept solving exactly as before, silently comparing the
    /// closed form against itself instead of switching to BE/trap. This is a
    /// distinct error code from a bad option *in the program itself*
    /// (`ERR_MALFORMED` at load, `program::parse_option`) — the bytes were
    /// fine at load time; this is a live call misusing the ABI.
    pub fn set_option(&mut self, key: u32, value: f64) -> Result<(), BoError> {
        match key {
            2 => {
                let off = Self::require_switch("NO_SOLUTION_CACHE", value)?;
                self.options.no_solution_cache = off;
                // Propagated into the concrete solver (`solve::static_part`'s
                // solution cache, W1 plan row B1): this generic `Engine`
                // struct only stores the option for validation/round-trip,
                // never reads it back into a solve itself.
                self.kernel.solver_mut().set_no_solution_cache(off);
                Ok(())
            }
            3 => {
                self.options.no_rail_shortcut = Self::require_switch("NO_RAIL_SHORTCUT", value)?;
                Ok(())
            }
            1 | 4 | 5 | 6 | 7 => Err(BoError::invalid_argument(format!(
                "option key {key} is load-time only and cannot be changed after load"
            ))),
            _ => Err(BoError::invalid_argument(format!("unknown option key {key}"))),
        }
    }

    fn require_switch(name: &str, value: f64) -> Result<bool, BoError> {
        switch_value(value)
            .ok_or_else(|| BoError::invalid_argument(format!("option {name}: value must be 0 or 1, got {value}")))
    }

    /// Advances to `t_end` (absolute seconds), through the event kernel's
    /// two-phase delta cycle (contract point 3, decision 4). The non-finite
    /// and backwards-time checks run first and stay entirely `Engine`'s
    /// concern (`KernelError` has no ABI code of its own, by design —
    /// `kernel.rs` does not depend on `error.rs`): a plain `t_end < time`
    /// check alone would accept `NaN` (every comparison against it is
    /// false), which would poison the clock and permanently disable this
    /// same guard on every later call, and would accept `+Infinity` too,
    /// then reject every later finite advance as "backwards". Neither is a
    /// real point in time, so both are `-6` here, before the kernel ever
    /// sees `t_end`.
    pub fn advance(&mut self, t_end: f64) -> Result<(), BoError> {
        if !t_end.is_finite() {
            return Err(BoError::time_backwards("t_end must be finite".to_string()));
        }
        if t_end < self.kernel.time() - TIME_EPS {
            return Err(BoError::time_backwards(format!(
                "advance({t_end}) is before the current time {}",
                self.kernel.time()
            )));
        }
        let result = self.kernel.advance(t_end);
        // Drained unconditionally, before mapping/returning the error: see
        // `Self::apply_digital_writes`'s own doc comment on why a failed
        // `advance` can still carry real, already-completed instants' writes.
        self.apply_digital_writes();
        result.map_err(map_kernel_error)
    }

    /// `bo_engine_set_params` (owner decision 1): `(deviceIndex, paramIndex,
    /// value)` triples for a switch-family kind's live params
    /// (`kinds::shape(code).live_params`). All or nothing — every triple is
    /// validated against the current program before any of them is applied
    /// (`solve::PartitionSolver::validate_params`, which the concrete
    /// `solve::static_part::StaticPartitionSolver` backs with per-kind
    /// `kinds.rs` `check` calls) — then applied as ONE event at the current
    /// time: the params change, the affected partitions go dirty, and the
    /// delta cycle runs at that instant (`Kernel::apply_params_as_event`),
    /// with all other state kept.
    ///
    /// An empty batch is trivially valid and changes nothing; it returns
    /// `Ok(())` without touching the kernel at all, rather than running a
    /// vacuous "event" with no partitions to re-solve.
    ///
    /// A validation failure (an out-of-range device/param index, or a value
    /// the kind's `check` rejects) is `ERR_INVALID_ARGUMENT` (-8), per the
    /// contract, with nothing changed. A downstream solve failure or
    /// livelock while applying an otherwise-valid batch (a toggle that makes
    /// the circuit unsolvable) surfaces as the ordinary `-4`/`-7` those
    /// conditions already carry — see `Kernel::apply_params_as_event`'s doc
    /// comment for why this method treats that case as "nothing changes"
    /// too, a reading of "all or nothing" the contract does not spell out
    /// explicitly.
    pub fn set_params(&mut self, triples: &[(u32, u32, f64)]) -> Result<(), BoError> {
        if triples.is_empty() {
            return Ok(());
        }
        let triples: Vec<(usize, usize, f64)> = triples.iter().map(|&(d, p, v)| (d as usize, p as usize, v)).collect();
        self.kernel.solver().validate_params(&triples).map_err(BoError::invalid_argument)?;
        let result = self.kernel.apply_params_as_event(&triples);
        // See `Engine::advance`'s identical call: drained unconditionally.
        self.apply_digital_writes();
        result.map_err(map_kernel_error)
    }

    /// `bo_engine_seed_memory` (W1d, owner decision 2): validates `device` is
    /// a real device-record index (`-8` otherwise — an unknown/out-of-range
    /// kind or a bad word is the `DeviceEvaluator::seed_memory` hook's own
    /// job, also mapped to `-8` below), then applies it as one event at the
    /// current time exactly like `Self::set_params` (`Kernel::seed_memory`,
    /// mirroring `Kernel::apply_params_as_event`): a hook rejection changes
    /// nothing and maps to `-8`; a downstream solve failure/livelock while
    /// re-evaluating the device rolls the contents back with the rest of the
    /// instant and maps to `-4`/`-7` as usual. Every digital-slot write the
    /// kernel queued is published afterward regardless of outcome, exactly
    /// like `advance`/`set_params` (`Self::apply_digital_writes`'s own doc
    /// comment).
    pub fn seed_memory(&mut self, device: u32, words: &[f64]) -> Result<(), BoError> {
        if device as usize >= self.device_count {
            return Err(BoError::invalid_argument(format!("seed_memory: device index {device} out of range")));
        }
        let result = self.kernel.seed_memory(DeviceId(device), words);
        // See `Engine::advance`'s identical call: drained unconditionally,
        // regardless of outcome.
        self.apply_digital_writes();
        match result {
            Ok(()) => Ok(()),
            Err(SeedMemoryError::Rejected(msg)) => Err(BoError::invalid_argument(msg)),
            Err(SeedMemoryError::Kernel(e)) => Err(map_kernel_error(e)),
        }
    }

    /// `bo_engine_take_eeprom_updates` (W1d): collects and clears every
    /// dirty EEPROM image (`DeviceEvaluator::take_eeprom_updates`), packed
    /// `deviceIndex, W, word x W` per dirty EEPROM, in device order
    /// (core/README.md's own wording). Kept on `Engine` (not just returned)
    /// so `Self::eeprom_updates`'s pointer stays valid "until the next call
    /// on this engine" — the ABI's `bo_engine_eeprom_updates_ptr` reads that
    /// accessor, never this one, so reading the pointer never re-drains the
    /// evaluator's dirty set a second time.
    pub fn take_eeprom_updates(&mut self) -> &[f64] {
        self.eeprom_updates = self.kernel.evaluator_mut().take_eeprom_updates();
        &self.eeprom_updates
    }

    /// The buffer from the last [`Self::take_eeprom_updates`] call.
    pub fn eeprom_updates(&self) -> &[f64] {
        &self.eeprom_updates
    }

    /// Loads a program from scratch: build the static topology, register
    /// every IC's output drivers (combinational, sequential and memory
    /// alike — W1d widened this from combinational-only) and install the
    /// real digital evaluator (`devices::ics::register`, W1c contract point
    /// 2), then
    /// settle at `START_TIME` (decision 8, and decision 4's option key 7 —
    /// the engine clock and the kernel both start there, never at a
    /// hard-coded 0) with every IC as a reader (contract point 2's own
    /// wording) — for a program with no IC (or no digital kind at all,
    /// W0/W1b), `all_readers` is empty and this is exactly the one dense LU
    /// solve per partition, plus the KCL current pass, `Engine::load` always
    /// ran. Never panics on a well-formed `Program` — `Program::parse` is
    /// what turns malformed bytes into `BoError` before this ever runs.
    pub fn load(program: &Program) -> Result<Engine, BoError> {
        let solver = PartitionedSolver::build(program)?;
        let partition_count = solver.partition_count();
        let static_solve_count = solver.static_solve_count();
        let largest_partition = solver.largest_partition();
        let fixed_count = solver.fixed_count();
        let multi_capacitor_linear_partitions = solver.multi_capacitor_linear_partitions();
        let all_partitions: Vec<PartitionId> = (0..partition_count as u32).map(PartitionId).collect();

        let mut kernel = Kernel::new(Box::new(solver) as Box<dyn PartitionSolver>, Box::new(NoDevices) as Box<dyn DeviceEvaluator>);
        // `ics::register` needs a live kernel to assign driver ids through
        // (`Kernel::add_driver`), so the real evaluator can only be built —
        // and installed via `set_evaluator` — AFTER `Kernel::new`, exactly
        // like `solve::ic_tests`'s own `IcHarness` build order (that
        // module's own doc comment).
        let ic_evaluator = ics::register(&mut kernel, program);
        let all_readers = ic_evaluator.reader_ids();
        kernel.set_evaluator(Box::new(ic_evaluator) as Box<dyn DeviceEvaluator>);
        kernel.settle(program.options.start_time, &all_partitions, &all_readers).map_err(map_kernel_error)?;

        let mut stats = [0.0; STAT_COUNT];
        stats[STAT_PARTITIONS as usize] = partition_count as f64;
        stats[STAT_STATIC_SOLVES as usize] = static_solve_count as f64;
        stats[STAT_LARGEST_PARTITION as usize] = largest_partition as f64;
        stats[STAT_FIXED_NODES as usize] = fixed_count as f64;
        stats[STAT_MULTI_CAPACITOR_LINEAR_PARTITIONS as usize] = multi_capacitor_linear_partitions;

        let mut engine = Engine {
            node_count: program.node_count,
            device_count: program.devices.len(),
            kernel,
            stats,
            options: program.options,
            readout: Readout::new(program.digital_slot_count, program.channel_slot_count),
            eeprom_updates: Vec::new(),
        };
        // The settle above already queued every IC's initial digital-slot
        // state on the kernel side (`Kernel::settle`'s own `publish_settled`
        // call) — drain it into this freshly-built `readout` now, so a
        // caller reading `digital_slots()` right after `load()` sees it
        // without a separate `advance()` first.
        engine.apply_digital_writes();
        Ok(engine)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::DelaySet;
    use crate::error::{ERR_INVALID_ARGUMENT, ERR_SOLVE_FAILED, ERR_TIME_BACKWARDS, ERR_UNSUPPORTED_TOPOLOGY};
    use crate::events::{DriverId, Mode};
    use crate::kinds;
    use crate::program::{Device, Options, PinRecord};

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn resistor(component_index: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: kinds::RESISTOR, component_index, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }

    fn source(component_index: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: kinds::VOLTAGE_SOURCE, component_index, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }

    fn switch_dev(component_index: usize, a: usize, b: usize, closed: f64) -> Device {
        Device { kind: kinds::SWITCH, component_index, pins: vec![pin(a), pin(b)], params: vec![closed], blob: vec![] }
    }

    fn capacitor(component_index: usize, a: usize, b: usize, c: f64, esr: f64, leak: f64) -> Device {
        Device { kind: kinds::CAPACITOR, component_index, pins: vec![pin(a), pin(b)], params: vec![c, esr, leak], blob: vec![] }
    }

    fn program(node_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    }

    #[test]
    fn divider_reads_5v_at_the_midpoint_with_correct_currents() {
        // node0 = datum, node1 = source +, node2 = midpoint.
        let p = program(3, vec![
            source(0, 1, 0, 10.0),
            resistor(1, 1, 2, 1000.0),
            resistor(2, 2, 0, 1000.0),
        ]);
        let e = Engine::load(&p).unwrap();
        assert!((e.voltages()[2] - 5.0).abs() < 1e-6, "got {}", e.voltages()[2]);
        assert!((e.currents()[1] - 0.005).abs() < 1e-6, "R1 current {}", e.currents()[1]);
        assert!((e.currents()[2] - 0.005).abs() < 1e-6, "R2 current {}", e.currents()[2]);
        // Current into the source's + terminal is negative while it delivers
        // power: -5 mA.
        assert!((e.currents()[0] - (-0.005)).abs() < 1e-6, "source current {}", e.currents()[0]);
    }

    #[test]
    fn source_fixing_its_negative_terminal() {
        // pins = (datum +, node1 -): node1 ends up at -V.
        let p = program(2, vec![source(0, 0, 1, 7.0)]);
        let e = Engine::load(&p).unwrap();
        assert!((e.voltages()[1] - (-7.0)).abs() < 1e-9);
    }

    #[test]
    fn chained_sources_with_a_load_on_each_node_have_kcl_correct_currents() {
        // datum -> A at 5V -> B at 5+3=8V, with a 1k load from A and B each
        // to the datum, so both loads pull from A's own branch.
        let p = program(3, vec![
            source(0, 1, 0, 5.0),  // fixes node1 (A) = 5V
            source(1, 2, 1, 3.0),  // fixes node2 (B) = A + 3 = 8V
            resistor(2, 1, 0, 1000.0), // load at A
            resistor(3, 2, 0, 1000.0), // load at B
        ]);
        let e = Engine::load(&p).unwrap();
        assert!((e.voltages()[1] - 5.0).abs() < 1e-9);
        assert!((e.voltages()[2] - 8.0).abs() < 1e-9);

        let i_load_a = e.currents()[2];
        let i_load_b = e.currents()[3];
        assert!((i_load_a - 0.005).abs() < 1e-9);
        assert!((i_load_b - 0.008).abs() < 1e-9);

        // KCL at B: source_b supplies exactly the load at B (B's + terminal
        // is node2, since pins = (2,1)).
        let i_source_b = e.currents()[1];
        assert!((i_source_b - (-i_load_b)).abs() < 1e-6, "source_b {}", i_source_b);

        // KCL at A: source_a supplies A's own load AND everything source_b
        // draws through A (source_b's - terminal sits at A, so source_b
        // pulls +i_source_b out of the circuit at A, i.e. draws -i_source_b
        // from A into itself). A's + terminal is node1 (pins = (1,0)).
        let i_source_a = e.currents()[0];
        let expected = -(i_load_a + (-i_source_b));
        assert!((i_source_a - expected).abs() < 1e-6, "source_a {} expected {}", i_source_a, expected);
    }

    #[test]
    fn two_ideal_sources_in_parallel_is_unsupported_topology() {
        let p = program(2, vec![source(0, 1, 0, 5.0), source(1, 1, 0, 5.0)]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY);
    }

    #[test]
    fn a_source_with_both_terminals_on_the_same_node_is_unsupported_topology() {
        // Shorted to itself, and never fixed by anything else: floating.
        let p = program(2, vec![source(0, 1, 1, 5.0)]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY);
    }

    #[test]
    fn a_source_shorted_to_itself_after_being_otherwise_fixed_is_unsupported_topology() {
        // node1 gets fixed by a normal source, then a second source shorted
        // node1-to-node1 finds both its terminals already fixed.
        let p = program(2, vec![source(0, 1, 0, 5.0), source(1, 1, 1, 3.0)]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY);
    }

    #[test]
    fn a_floating_source_is_unsupported_topology() {
        // node1/node2 are a pair, but neither ever reaches the datum.
        let p = program(3, vec![source(0, 1, 2, 5.0)]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_UNSUPPORTED_TOPOLOGY);
    }

    #[test]
    fn an_isolated_net_with_no_elements_reads_zero_volts_and_does_not_fail() {
        let p = program(2, vec![]);
        let e = Engine::load(&p).unwrap();
        assert_eq!(e.voltages()[1], 0.0);
        assert_eq!(e.stat(STAT_PARTITIONS), 1.0);
    }

    #[test]
    fn two_independent_partitions_solve_separately() {
        let p = program(5, vec![
            source(0, 1, 0, 10.0),
            resistor(1, 1, 2, 1000.0),
            resistor(2, 2, 0, 1000.0),
            source(3, 3, 0, 20.0),
            resistor(4, 3, 4, 500.0),
            resistor(5, 4, 0, 1500.0),
        ]);
        let e = Engine::load(&p).unwrap();
        assert!((e.voltages()[2] - 5.0).abs() < 1e-6);
        assert!((e.voltages()[4] - 15.0).abs() < 1e-6);
        assert_eq!(e.stat(STAT_PARTITIONS), 2.0);
    }

    #[test]
    fn a_resistor_with_both_ends_fixed_has_a_computed_current_and_no_stamp() {
        let p = program(3, vec![
            source(0, 1, 0, 10.0),
            source(1, 2, 0, 4.0),
            resistor(2, 1, 2, 1000.0),
        ]);
        let e = Engine::load(&p).unwrap();
        // No free nodes at all: zero partitions.
        assert_eq!(e.stat(STAT_PARTITIONS), 0.0);
        assert!((e.currents()[2] - 0.006).abs() < 1e-9);
    }

    #[test]
    fn stats_report_partitions_solves_largest_partition_and_fixed_count() {
        let p = program(4, vec![
            source(0, 1, 0, 10.0),
            resistor(1, 1, 2, 1000.0),
            resistor(2, 2, 3, 1000.0),
            resistor(3, 3, 0, 1000.0),
        ]);
        let e = Engine::load(&p).unwrap();
        assert_eq!(e.stat(STAT_PARTITIONS), 1.0);
        assert_eq!(e.stat(STAT_STATIC_SOLVES), 1.0);
        assert_eq!(e.stat(STAT_LARGEST_PARTITION), 2.0);
        assert_eq!(e.stat(STAT_FIXED_NODES), 2.0); // node0 + node1
        assert!(e.stat(99).is_nan());
    }

    #[test]
    fn advance_rejects_moving_time_backwards_and_accepts_forward_motion() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        e.advance(1.0).unwrap();
        assert_eq!(e.time(), 1.0);
        let err = e.advance(0.5).unwrap_err();
        assert_eq!(err.code, -6);
        e.advance(2.0).unwrap();
        assert_eq!(e.time(), 2.0);
    }

    #[test]
    fn advance_rejects_non_finite_t_end_instead_of_poisoning_the_clock() {
        // Before this fix: `t_end < self.time - TIME_EPS` is false for every
        // comparison against NaN, so NaN was accepted and became the new
        // clock — after which every future comparison against it was also
        // false, permanently disabling the backwards-time guard. +Infinity
        // was accepted too, then rejected every later finite advance as
        // "backwards". Neither is a real point in time.
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        e.advance(1.0).unwrap();

        let err = e.advance(f64::NAN).unwrap_err();
        assert_eq!(err.code, ERR_TIME_BACKWARDS);
        assert_eq!(err.message, "t_end must be finite");
        assert_eq!(e.time(), 1.0, "a rejected advance must not touch the clock");

        let err = e.advance(f64::INFINITY).unwrap_err();
        assert_eq!(err.code, ERR_TIME_BACKWARDS);
        assert_eq!(e.time(), 1.0);

        // The clock must still be usable afterwards.
        e.advance(2.0).unwrap();
        assert_eq!(e.time(), 2.0);
    }

    #[test]
    fn load_fails_with_solve_failed_when_a_published_voltage_overflows_to_infinity() {
        // Two 1e308 V sources chained in series: fixed-node propagation adds
        // their voltages in plain f64 arithmetic (1e308 + 1e308 overflows a
        // finite double), and no LU pivot is ever involved, so the old
        // pivot-only check let this through as a converged load with an
        // Infinity readout.
        let p = program(3, vec![
            source(0, 1, 0, 1e308),
            source(1, 2, 1, 1e308),
        ]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_SOLVE_FAILED);
        assert!(err.message.contains("node 2"), "message was: {}", err.message);
    }

    #[test]
    fn load_fails_with_solve_failed_when_a_published_current_is_non_finite() {
        // A hand-built both-fixed resistor with R = 0 (the reader now
        // rejects this in a real program, but `Engine::load` is tested
        // directly on hand-built `Program`s here, the same way `abi.rs` will
        // call it after parsing). Both terminals are fixed, so this resistor
        // is never stamped into a matrix — its current is Ohm's law on two
        // already-known, finite voltages, dividing by zero. Listed first
        // (device 0) so it is unambiguously the "first offending device" the
        // message names, rather than a source whose current only went
        // non-finite by KCL propagation from this same resistor.
        let p = program(3, vec![
            resistor(0, 1, 2, 0.0),
            source(1, 1, 0, 10.0),
            source(2, 2, 0, 4.0),
        ]);
        let err = Engine::load(&p).unwrap_err();
        assert_eq!(err.code, ERR_SOLVE_FAILED);
        assert!(err.message.contains("device 0"), "message was: {}", err.message);
    }

    #[test]
    fn load_solves_a_long_series_chain_without_stack_overflow() {
        // The end-to-end counterpart of
        // `partition::tests::union_find_handles_a_million_node_chain_without_recursing`,
        // through the real `Engine::load` path (parse -> fixed-node derivation ->
        // partitioning -> solve), where the old recursive `find` actually
        // lived (the partition-grouping loop's first `find` call walked the
        // whole chain). A chain anywhere near the 1,000,000-node ceiling
        // needs a dense partition matrix of many gigabytes, which a native
        // test cannot allocate; this uses a chain long enough to exercise the
        // same code path with a real solve, at a size the dense LU can
        // actually factor in a test.
        const CHAIN_NODES: usize = 800; // nodes 1..=CHAIN_NODES, one free partition
        let mut devices = Vec::with_capacity(CHAIN_NODES - 1);
        for i in 1..CHAIN_NODES {
            devices.push(resistor(i, i, i + 1, 1000.0));
        }
        let p = program(CHAIN_NODES + 1, devices);
        let e = Engine::load(&p).unwrap();
        assert_eq!(e.stat(STAT_PARTITIONS), 1.0);
        assert_eq!(e.stat(STAT_LARGEST_PARTITION), CHAIN_NODES as f64);
        for (node, v) in e.voltages().iter().enumerate() {
            assert!(v.is_finite(), "node {node} voltage was not finite: {v}");
        }
    }

    /// Section 3 of the W1b review, contract point 2: a capacitor whose two
    /// pins land on the SAME free node must stamp nothing (its companion
    /// stamps cancel exactly, classic's own `stampConductance` behavior for
    /// `i==j`) and must not count toward that node's partition
    /// classification. Before this fix, `partition::build` counted it
    /// anyway, wrongly classifying the node `ClosedForm`; its branch-current
    /// row in `closed_form::build_segment` was then all zero (the
    /// capacitor's `+1`/`-1` stamps to the SAME local index cancel exactly,
    /// and `esr -= 0` when `esr == 0` leaves nothing else in that row) — a
    /// singular pivot, so `bo_engine_load` returned `-4` where classic loads
    /// the circuit normally. `esr` is irrelevant either way, since a
    /// self-shorted capacitor never reaches the matrix at all; checked at 0
    /// (the review's own repro) and a nonzero value, and under both default
    /// classification and `FORCE_STEPPED` (which must not be able to pull a
    /// partition with no REAL capacitor into the stepped path).
    #[test]
    fn a_capacitor_shorted_to_itself_on_a_free_node_loads_like_classic_not_singular() {
        for &esr in &[0.0, 0.5] {
            for &force_stepped in &[false, true] {
                // node0 = gnd, node1 = source+, node2 (n1) = free: 1k from
                // the source, a capacitor with BOTH pins on n1, 1k to gnd.
                let mut options = Options::default();
                options.force_stepped = force_stepped;
                let p = Program {
                    node_count: 3,
                    options,
                    digital_slot_count: 0,
                    channel_slot_count: 0,
                    devices: vec![
                        source(0, 1, 0, 5.0),
                        resistor(1, 1, 2, 1000.0),
                        capacitor(2, 2, 2, 1e-6, esr, 0.0),
                        resistor(3, 2, 0, 1000.0),
                    ],
                };
                let e = Engine::load(&p)
                    .unwrap_or_else(|err| panic!("esr={esr} force_stepped={force_stepped}: load failed with {} ({})", err.code, err.message));
                // classic's own shunt-inclusive divider (the universal
                // 1e-12 S node shunt pulls it very slightly below a bare
                // 2.5V — the same tolerance `closed_form`'s own tests use
                // against that model).
                assert!((e.voltages()[2] - 2.5).abs() < 1e-6, "esr={esr} force_stepped={force_stepped}: n1 = {}", e.voltages()[2]);
                assert_eq!(e.currents()[2], 0.0, "esr={esr} force_stepped={force_stepped}: c1 must publish exactly 0");
            }
        }
    }

    #[test]
    fn a_capacitor_with_both_pins_on_gnd_loads_fine_and_publishes_zero_current() {
        let p = program(1, vec![capacitor(0, 0, 0, 1e-6, 0.0, 0.0)]);
        let e = Engine::load(&p).unwrap();
        assert_eq!(e.currents()[0], 0.0);
    }

    // --- v2 additions: next_event_time, slots, set_option, trace, stats ---

    #[test]
    fn next_event_time_is_infinity_in_the_static_kernel() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        assert_eq!(e.next_event_time(), f64::INFINITY);
    }

    // ---- decision 4: option key 7, START_TIME ----------------------------

    #[test]
    fn load_starts_the_engine_clock_and_the_kernel_at_start_time() {
        // core/README.md, "Options": START_TIME is "the absolute time the
        // loaded program starts at" -- the facade passes its carried-over
        // simTime through here on a reload, so the kernel's clock must start
        // there, not at a hard-coded 0.
        let mut p = program(1, vec![]);
        p.options.start_time = 12.5;
        let e = Engine::load(&p).unwrap();
        assert_eq!(e.time(), 12.5);
    }

    #[test]
    fn set_option_rejects_start_time_after_load_with_a_message_naming_it() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        let err = e.set_option(7, 3.0).unwrap_err();
        assert_eq!(err.code, ERR_INVALID_ARGUMENT);
        assert!(err.message.contains('7'), "message was: {}", err.message);
    }

    // ---- "a failed advance": rollback to the last completed instant ------

    #[test]
    fn a_failed_advance_with_a_solve_failure_keeps_the_readouts_and_time_of_the_last_completed_instant() {
        // core/README.md, "A failed advance": on `-4` the engine keeps the
        // state of the last instant it *completed*. Proven here through the
        // real `StaticPartitionSolver`, which reaches exactly the
        // half-written-`voltages` case no mock-only coverage can: it writes
        // `voltages` partition by partition, so node1's freshly (and
        // validly) solved voltage from *this same, ultimately failed* delta
        // pass must not survive either.
        //
        // node1 and node2 are each their own singleton partition (joined to
        // the datum only by their own resistor, never to each other); node3
        // is a fixed 10V rail.
        let p = program(4, vec![
            source(0, 3, 0, 10.0),
            resistor(1, 1, 0, 1000.0),
            resistor(2, 2, 0, 1000.0),
        ]);
        let mut e = Engine::load(&p).unwrap();
        assert_eq!(e.time(), 0.0);
        let voltages_before = e.voltages().to_vec();

        // d1 is an ordinary driver; d2's conductance is infinite, which
        // stamps a non-finite matrix entry and yields a non-finite published
        // voltage on its own partition (contract, "W0 kernel semantics",
        // point 4: "-4" either from a non-finite pivot or, as here, a
        // finite-but-infinite-conductance stamp).
        let d1 = e.kernel.add_driver(Driver::new(1, 3, 0, 1.0, 1.0, 0.0), Some(PartitionId(0)));
        let d2 = e.kernel.add_driver(Driver::new(2, 3, 0, f64::INFINITY, 1.0, 0.0), Some(PartitionId(1)));
        // Both first requests, so both land at exactly `now` -- due at the
        // same instant, one delta pass resolves both partitions: partition 0
        // (d1) first, successfully; partition 1 (d2) second, and fails.
        e.kernel.request_driver(d1, Mode::High, 5.0, 0.0);
        e.kernel.request_driver(d2, Mode::High, 5.0, 0.0);

        let err = e.advance(5.0).unwrap_err();
        assert_eq!(err.code, ERR_SOLVE_FAILED);

        assert_eq!(e.time(), 0.0, "time must roll back to the last completed instant -- load, since this advance never completed");
        assert_eq!(
            e.voltages(),
            voltages_before.as_slice(),
            "node1's freshly (and validly) solved voltage must not survive partition 1's failure in the same, ultimately-failed delta"
        );
    }

    #[test]
    fn digital_slots_start_absent_and_channel_slots_start_zero_sized_from_the_program() {
        let mut p = program(1, vec![]);
        p.digital_slot_count = 3;
        p.channel_slot_count = 2;
        let e = Engine::load(&p).unwrap();
        assert_eq!(e.digital_slot_count(), 3);
        assert!(e.digital_slots().iter().all(|v| v.is_nan()), "{:?}", e.digital_slots());
        assert_eq!(e.channel_slot_count(), 2);
        assert_eq!(e.channel_slots(), &[0.0, 0.0]);
    }

    #[test]
    fn set_option_accepts_keys_2_and_3_after_load() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        assert!(e.set_option(2, 1.0).is_ok());
        assert!(e.set_option(3, 0.0).is_ok());
    }

    #[test]
    fn set_option_rejects_load_time_only_keys_after_load() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        for key in [1, 4, 5, 6, 7] {
            let err = e.set_option(key, 0.0).unwrap_err();
            assert_eq!(err.code, ERR_INVALID_ARGUMENT, "key {key}");
        }
    }

    /// Section 5 of the W1b review: `FORCE_STEPPED` after load used to
    /// return success but never actually move a partition between the
    /// closed-form and stepped solvers (`partition::build` only folds it in
    /// once, at load). Contract: key 4 is load-time only, exactly like keys
    /// 1/5/6/7, and the message must name it so a caller can tell which
    /// option it tried to change live.
    #[test]
    fn set_option_rejects_force_stepped_after_load() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        let err = e.set_option(4, 1.0).unwrap_err();
        assert_eq!(err.code, ERR_INVALID_ARGUMENT);
        assert!(err.message.contains('4'), "message was: {}", err.message);
    }

    #[test]
    fn set_option_rejects_unknown_keys() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        for key in [0, 8, 999] {
            let err = e.set_option(key, 0.0).unwrap_err();
            assert_eq!(err.code, ERR_INVALID_ARGUMENT, "key {key}");
        }
    }

    #[test]
    fn set_option_rejects_an_out_of_range_value_for_an_accepted_key() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        let err = e.set_option(2, 2.0).unwrap_err();
        assert_eq!(err.code, ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn trace_is_off_by_default_and_round_trips_enable_and_clear() {
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        assert!(e.trace().is_empty());
        e.trace_enable(true);
        e.trace_clear();
        assert!(e.trace().is_empty(), "clearing an empty trace stays empty");
    }

    #[test]
    fn stat_indices_4_through_14_are_zero_for_a_device_less_circuit() {
        // Nothing in a W0/W1a-only program ever creates a driver or a
        // reader, so `settle` at load has nothing to commit and `advance`'s
        // delta cycle never runs at all (no event is ever pending) — every
        // one of these stays at its `0.0` initializer, live counters
        // included, not because they are unimplemented but because there is
        // truly nothing for them to count.
        let p = program(1, vec![]);
        let mut e = Engine::load(&p).unwrap();
        e.advance(10.0).unwrap();
        for i in 4..=14 {
            assert_eq!(e.stat(i), 0.0, "stat({i})");
        }
        // Index 15 (W1b step 2's multi-capacitor-partition count, not yet in
        // the ABI contract table — see `STAT_MULTI_CAPACITOR_LINEAR_PARTITIONS`)
        // is a real, defined stat now, `0.0` for a capacitor-less program,
        // not `NaN`; only an index past `STAT_COUNT` still means "unknown".
        assert_eq!(e.stat(STAT_MULTI_CAPACITOR_LINEAR_PARTITIONS), 0.0);
        assert!(e.stat(99).is_nan());
    }

    #[test]
    fn set_digital_slot_records_a_trace_entry_only_on_an_actual_change() {
        let mut p = program(1, vec![]);
        p.digital_slot_count = 1;
        let mut e = Engine::load(&p).unwrap();
        e.trace_enable(true);

        // Absent (NaN) to a real value: a change.
        e.set_digital_slot(0, 1.0);
        assert_eq!(e.trace(), &[0.0, 0.0, 1.0]);

        // Same value again: not a change.
        e.set_digital_slot(0, 1.0);
        assert_eq!(e.trace(), &[0.0, 0.0, 1.0], "re-writing the same value must not grow the trace");

        // A real value back to absent: a change.
        e.set_digital_slot(0, f64::NAN);
        assert_eq!(e.trace().len(), 6);
        assert_eq!(&e.trace()[3..5], &[0.0, 0.0]);
        assert!(e.trace()[5].is_nan());

        // Absent to absent: not a change, even though NaN != NaN.
        e.set_digital_slot(0, f64::NAN);
        assert_eq!(e.trace().len(), 6, "absent-to-absent must not count as a change");
    }

    // --- phase 2: proving `Engine::advance` really routes through the kernel

    /// A synthetic driver/device pair, gated out of every real build, whose
    /// only purpose is to prove `Engine::advance` routes a scheduled driver
    /// commit through `Kernel::advance` into `StaticPartitionSolver` and back
    /// out through `Engine::voltages`/`Engine::stat` — the same wiring a real
    /// W1c/W1d kind will use once one exists. No production kind creates a
    /// `Driver` yet, so this is the only place one is ever added to an
    /// `Engine`'s kernel.
    #[test]
    fn advance_routes_a_scheduled_driver_commit_through_the_static_solver_and_the_stats_count_it() {
        // node0 = datum (fixed 0V), node1 = free (pulled to 0V by a 1k
        // resistor to the datum until the driver acts), node2 = a fixed 10V
        // rail via a voltage_source. The driver's hi_node is node2, lo_node
        // is node0 -- both fixed, so `StaticPartitionSolver` can stamp it.
        let p = program(3, vec![
            source(0, 2, 0, 10.0),
            resistor(1, 1, 0, 1000.0),
        ]);
        let mut e = Engine::load(&p).unwrap();

        // Before the driver exists at all: node1 sits at 0V, and every live
        // counter is untouched by load/settle (a device-less load).
        assert!((e.voltages()[1] - 0.0).abs() < 1e-9);
        assert_eq!(e.stat(4), 0.0, "events processed");
        assert_eq!(e.stat(5), 0.0, "delta passes");
        assert_eq!(e.stat(13), 0.0, "driver commits");

        // node1 is W0's only free node, so it is partition 0 by construction
        // (`StaticPartitionSolver::build`'s first-seen order).
        let d0 = e.kernel.add_driver(Driver::new(1, 2, 0, 1.0, 1.0, 0.0), Some(PartitionId(0)));

        // A first-ever request for this driver always schedules at exactly
        // the `now` it is given, regardless of `delay` (drivers::fsm's
        // "first observation has zero delay" row) -- so requesting it
        // directly at t = 5.0 is how a *first* commit lands at a specific
        // future time without any preceding commit to redirect away from.
        e.kernel.request_driver(d0, Mode::High, 5.0, 0.0);
        assert_eq!(e.next_event_time(), 5.0);

        // Nothing has happened yet: advancing to just before the scheduled
        // time must not move the voltage or the counters.
        e.advance(4.0).unwrap();
        assert!((e.voltages()[1] - 0.0).abs() < 1e-9);
        assert_eq!(e.stat(4), 0.0);

        e.advance(5.0).unwrap();

        // Driving HIGH toward a 10V rail through a 1 ohm driver conductance,
        // competing with the 1k pulldown to the datum, pulls node1 close to
        // 10V (a simple two-resistor divider: 10 * 1000 / 1001).
        let expected = 10.0 * 1000.0 / 1001.0;
        assert!((e.voltages()[1] - expected).abs() < 1e-6, "got {}", e.voltages()[1]);

        // One Commit event, popped once, that actually changed the driver's
        // committed level -- the real kernel counters, not a stub.
        assert_eq!(e.stat(4), 1.0, "events processed");
        assert!(e.stat(5) >= 1.0, "at least one delta pass");
        assert!(e.stat(6) >= 1.0, "max delta depth");
        assert_eq!(e.stat(13), 1.0, "driver commits");
    }

    #[test]
    fn livelock_through_engine_advance_returns_delta_livelock() {
        // `StaticPartitionSolver` itself can never livelock (it never
        // reports a reader, so `advance`'s delta cycle always settles after
        // the pass that applies a commit — there is nothing for W1a's static
        // kernel to loop on). Proving `Engine::advance` still propagates a
        // livelock end to end, the way a real zero-delay loop eventually will
        // once W1c's digital kinds exist, needs a solver/evaluator pair that
        // actually cycles; `Kernel::set_solver`/`set_evaluator` swap
        // `Engine`'s kernel to exactly the mock pair phase 1's own
        // `kernel::tests::an_oscillating_zero_delay_pair_hits_the_cap_...`
        // test used, reached here through the real `Engine::advance` path
        // and `BoError` mapping instead of `Kernel::advance` directly.
        struct AlwaysAffects;
        impl PartitionSolver for AlwaysAffects {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                Ok(if dirty.is_empty() { Vec::new() } else { vec![DeviceId(0)] })
            }
            fn voltages(&self) -> &[f64] {
                &[]
            }
            fn currents(&self) -> &[f64] {
                &[]
            }
            fn restore(&mut self, _snapshot: (Vec<f64>, Vec<f64>)) {
                // No state to restore.
            }
        }
        struct OscillatingEvaluator {
            driver: DriverId,
            next: Mode,
        }
        impl DeviceEvaluator for OscillatingEvaluator {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                let level = self.next;
                self.next = if level == Mode::High { Mode::Low } else { Mode::High };
                vec![DriverRequest::Level { driver: self.driver, level, delays: DelaySet::uniform(0.0) }]
            }
        }

        let p = program(2, vec![resistor(0, 1, 0, 1000.0)]);
        let mut e = Engine::load(&p).unwrap();
        let d0 = e.kernel.add_driver(Driver::new(1, 0, 0, 1.0, 1.0, 0.0), Some(PartitionId(0)));
        e.kernel.set_solver(Box::new(AlwaysAffects));
        e.kernel.set_evaluator(Box::new(OscillatingEvaluator { driver: d0, next: Mode::High }));
        e.kernel.request_driver(d0, Mode::Low, 0.0, 0.0);

        let err = e.advance(0.0).unwrap_err();
        assert_eq!(err.code, -7);
        assert!(err.message.contains("livelock"), "message was: {}", err.message);
        // Never returned silently: the clock stays put at the stuck instant.
        assert_eq!(e.time(), 0.0);
    }

    // --- W1b: bo_engine_set_params (owner decision 1) ----------------------

    #[test]
    fn set_params_with_an_empty_batch_is_a_no_op() {
        let p = program(3, vec![source(0, 1, 0, 10.0), switch_dev(1, 1, 2, 0.0)]);
        let mut e = Engine::load(&p).unwrap();
        let before = e.voltages().to_vec();
        e.set_params(&[]).unwrap();
        assert_eq!(e.voltages(), before.as_slice());
    }

    #[test]
    fn set_params_rejects_an_invalid_triple_with_invalid_argument_and_changes_nothing() {
        let p = program(3, vec![source(0, 1, 0, 10.0), switch_dev(1, 1, 2, 0.0)]);
        let mut e = Engine::load(&p).unwrap();
        let before = e.voltages().to_vec();

        // Device 0 is the voltage_source: it has no live params at all.
        let err = e.set_params(&[(0, 0, 1.0)]).unwrap_err();
        assert_eq!(err.code, ERR_INVALID_ARGUMENT);
        assert_eq!(e.voltages(), before.as_slice(), "an invalid batch must change nothing");

        // Device 1 is the switch: param 0 exists, but 2.0 fails
        // `kinds::single_switch_check`.
        let err = e.set_params(&[(1, 0, 2.0)]).unwrap_err();
        assert_eq!(err.code, ERR_INVALID_ARGUMENT);
        assert_eq!(e.voltages(), before.as_slice());
    }

    #[test]
    fn set_params_is_all_or_nothing_one_bad_triple_among_good_ones_changes_nothing() {
        let p = program(4, vec![
            source(0, 1, 0, 10.0),
            switch_dev(1, 1, 2, 0.0),
            switch_dev(2, 1, 3, 0.0),
        ]);
        let mut e = Engine::load(&p).unwrap();
        let before = e.voltages().to_vec();

        // First triple valid (close device 1), second invalid (device 2's
        // value out of range): the whole batch must be rejected, and device
        // 1 must NOT have been toggled either.
        let err = e.set_params(&[(1, 0, 1.0), (2, 0, 2.0)]).unwrap_err();
        assert_eq!(err.code, ERR_INVALID_ARGUMENT);
        assert_eq!(e.voltages(), before.as_slice(), "a later bad triple must undo nothing, because nothing was ever applied");
    }

    #[test]
    fn set_params_applies_a_valid_toggle_as_one_event_and_resolves_immediately() {
        // 10V -- 1kΩ resistor -- node2 -- switch -- datum: a resistor in
        // series with the switch, so the switch carries a meaningful,
        // boundable current instead of ~0 (as it would directly across the
        // ideal source, where nothing else limits it).
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 2, 0, 0.0)]);
        let mut e = Engine::load(&p).unwrap();
        // Open: node2 has no path to the datum at all except the switch
        // (which stamps nothing while open), so it floats to essentially the
        // full 10V (nothing pulls current through the resistor).
        assert!((e.voltages()[2] - 10.0).abs() < 1e-6, "got {}", e.voltages()[2]);

        e.set_params(&[(2, 0, 1.0)]).unwrap();
        // Closed: node2 is now shorted to the datum through the switch's
        // 1 mΩ contact, dominated by the 1kΩ feed from the rail, so it reads
        // ~0V and ~10mA now flows -- the point of this test is that the
        // toggle actually took effect immediately, without a further
        // `advance`.
        assert!(e.voltages()[2].abs() < 1e-3, "got {}", e.voltages()[2]);
        assert!((e.currents()[2] - 0.01).abs() < 1e-4, "closing the switch must publish ~10mA through it, got {}", e.currents()[2]);
    }

    #[test]
    fn set_params_toggle_re_solving_repeatedly_hits_the_solution_cache() {
        let p = program(3, vec![source(0, 1, 0, 10.0), switch_dev(1, 1, 2, 1.0)]);
        let mut e = Engine::load(&p).unwrap();
        assert_eq!(e.stat(7), 0.0);

        // Toggling to the SAME value the switch is already at still mutates
        // (bumps the epoch) and re-solves as one event; a second identical
        // toggle after that re-solve is what actually repeats the same
        // conditions and can hit the cache.
        e.set_params(&[(1, 0, 0.0)]).unwrap();
        e.set_params(&[(1, 0, 0.0)]).unwrap();
        // Both calls change the same one partition to the same electrical
        // state (open), so the second's resolve must reuse the first's
        // cached solution rather than re-running the LU solve.
        assert!(e.stat(7) >= 1.0, "expected at least one cache hit, got {}", e.stat(7));
    }

    #[test]
    fn horizon_clamp_advance_readouts_are_solved_values_at_t_not_interpolations() {
        // FORCE_STEPPED (a single capacitor, Decision 14's override) so the
        // horizon clamp actually has integration work to do — a closed-form
        // partition is exact at any `t` with no clamp needed at all (see
        // `PartitionSolver::sync_dynamic_to`'s doc comment), so it alone
        // would not distinguish "landed exactly on t_end" from "already
        // correct regardless".
        let mut options = Options::default();
        options.force_stepped = true;
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = Program {
            node_count: 3,
            options,
            digital_slot_count: 0,
            channel_slot_count: 0,
            devices: vec![source(0, 1, 0, v0), resistor(1, 1, 2, r), capacitor(2, 2, 0, c, 0.0, 0.0)],
        };
        let mut e = Engine::load(&p).unwrap();

        // An "ugly" time that will not, in general, land on a step-doubling
        // controller's own natural step boundary.
        let t = 4.37e-4;
        e.advance(t).unwrap();
        assert_eq!(e.time(), t);

        let tau = r * c;
        let expected = v0 * (1.0 - (-t / tau).exp());
        let got = e.voltages()[2];
        // The step-doubling controller's own local tolerance is REL_TOL =
        // 1e-3; a horizon-clamped landing exactly at `t` should track the
        // analytic answer to within a small multiple of that, not the much
        // larger gap an un-clamped "wherever the last committed step
        // happened to land" readout would show (up to ACTIVE_H_MAX = 2ms
        // away for this circuit's time constant).
        let rel = (got - expected).abs() / expected.abs();
        assert!(rel < 1e-2, "t={t}: got {got} expected {expected} rel {rel}");

        // Advancing again to the SAME instant must be idempotent: nothing
        // new is due, so the horizon clamp re-publishes the identical
        // solved value, not a fresh integration step from it.
        e.advance(t).unwrap();
        assert_eq!(e.voltages()[2], got);
    }

    // ---- W1d: the dispatcher through the real Engine::load entry point -----

    fn w1d_params(delay_pairs: usize, extra: &[f64]) -> Vec<f64> {
        let mut p = vec![0.0; kinds::ic_param::COUNT];
        p[kinds::ic_param::VCC_MIN] = 4.5;
        p[kinds::ic_param::VCC_MAX] = 5.5;
        p[kinds::ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
        p[kinds::ic_param::VIL_FIXED] = 1.0;
        p[kinds::ic_param::VIL] = 0.8;
        p[kinds::ic_param::VIH_FIXED] = 1.0;
        p[kinds::ic_param::VIH] = 2.0;
        p[kinds::ic_param::NOMINAL] = 5.0;
        p.extend(std::iter::repeat(20e-9).take(2 * delay_pairs));
        p.extend_from_slice(extra);
        p
    }

    /// A W1d sequential/memory device with every logic pin tied to `gnd` and
    /// no ROM contents — `devices::ics::tests`'s identical helper, rebuilt
    /// here rather than shared (this crate's own convention: each test
    /// module owns its tiny fixtures, `devices::ics`'s own doc comment).
    fn w1d_ic(kind: u32, ci: usize, vcc: usize, gnd: usize, logic_len: usize, params: Vec<f64>, rom_words: usize) -> Device {
        let device_pins = std::iter::once(pin(vcc)).chain(std::iter::once(pin(gnd))).chain((0..logic_len).map(|_| pin(gnd))).collect();
        let mut blob = vec![0.0];
        blob.extend(std::iter::repeat(0.0).take(logic_len));
        blob.extend(std::iter::repeat(0.0).take(rom_words));
        Device { kind, component_index: ci, pins: device_pins, params, blob }
    }

    #[test]
    fn engine_load_registers_and_settles_one_of_each_w1d_kind_with_the_stubs_publishing_nan() {
        // The DELIVER-list proof that the dispatcher (devices::ics::register
        // and IcEvaluator) is wired all the way through the real
        // `Engine::load` entry point, not just through a hand-built
        // `Kernel`/`PartitionedSolver` pair (devices::ics::tests's own
        // version of this same fixture) -- one each of 161, 173, 74hc74, 189
        // and 28c16, all powered from a shared 5V fixed supply.
        let p = Program {
            node_count: 2,
            options: Options::default(),
            digital_slot_count: 100,
            channel_slot_count: 0,
            devices: vec![
                source(0, 1, 0, 5.0),
                w1d_ic(kinds::IC_74LS161, 1, 1, 0, 14, w1d_params(4, &[]), 0),
                w1d_ic(kinds::IC_74LS173, 2, 1, 0, 14, w1d_params(4, &[]), 0),
                w1d_ic(kinds::IC_74HC74, 3, 1, 0, 12, w1d_params(2, &[]), 0),
                w1d_ic(kinds::IC_74LS189, 4, 1, 0, 14, w1d_params(5, &[]), 0),
                w1d_ic(kinds::IC_28C16, 5, 1, 0, 22, w1d_params(5, &[10e-3, 0.0]), 2048 / 4),
            ],
        };
        let e = Engine::load(&p).expect("a program holding every W1d kind must load through the real Engine::load entry point");
        assert!(!e.digital_slots().is_empty());
        for (slot, &value) in e.digital_slots().iter().enumerate() {
            assert!(value.is_nan(), "slot {slot} must read NaN while the W1d family evaluators are stubs, got {value}");
        }
    }
}
