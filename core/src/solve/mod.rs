//! Partition solvers. W1a defined only the partition-solver trait (worker
//! A1) plus the first real implementation, `StaticPartitionSolver`, adapting
//! the W0 static kernel. W1b step 1 (row B1) moved that implementation to
//! `static_part`, added the switch family's fresh-every-resolve stamping and
//! the solution cache, and gave this trait the `bo_engine_set_params` (owner
//! decision 1) and cache-control hooks every default-implemented method
//! below exists for.
//!
//! W1b step 2 (rows B2/B3) adds the two dynamic partition kinds
//! (`closed_form`, `stepped`, sharing the linear per-partition stamping
//! helpers and [`DynShape`] defined here) and [`PartitionedSolver`], which
//! dispatches every [`PartitionSolver`] call across the three partition
//! kinds `partition::build` can now produce (Static/ClosedForm/Stepped,
//! Decision 14) by [`crate::partition::PartitionKind`]. Every addition to
//! the trait below is a DEFAULT-provided method: `StaticPartitionSolver` and
//! every `#[cfg(test)]` mock solver in `kernel.rs`/`engine.rs` (which predate
//! W1b step 2 and know nothing about dynamic partitions) keep compiling
//! unchanged, because a solver with no dynamic partitions has nothing to say
//! through any of them.

pub mod closed_form;
mod curvature;
mod hold;
mod hub;
pub(crate) mod live;
mod settle;
pub mod newton;
pub mod static_part;
pub mod stepped;

#[cfg(test)]
mod ic_tests;
#[cfg(test)]
mod led_tests;
#[cfg(test)]
mod rail_tests;
#[cfg(test)]
mod slow_tests;
#[cfg(test)]
mod carry_tests;
#[cfg(test)]
mod snapshot_tests;
#[cfg(test)]
mod window_tests;

use std::collections::{BTreeSet, HashMap};
use std::rc::Rc;

use crate::devices::{self, led};
use crate::digital;
use crate::drivers::{Driver, DriverStage};
use crate::error::BoError;
use crate::events::{DeviceId, EventKind, Mode, PartitionId, Version};
use crate::kinds;
use crate::partition::{self, PartitionKind};
use crate::program::Program;

/// What re-solving a set of partitions changed, from the kernel's point of
/// view: the readers whose watched inputs moved. `kernel.rs`'s delta cycle
/// (contract, "Kernel semantics: events", point 3) calls
/// [`resolve_dirty`](PartitionSolver::resolve_dirty) once per delta, with
/// every partition a commit marked dirty this pass, and evaluates exactly
/// the readers it returns — each at most once, even if several of its
/// inputs live in different partitions this same call re-solved (the kernel
/// deduplicates across the whole call's result, not per partition).
///
/// A real implementation (W1b's `static_part.rs`, `closed_form.rs`,
/// `stepped.rs`) decides "moved" against its own tolerance or solution-cache
/// band; a partition that resolves to the same values it already held
/// reports no readers at all, which is how a network without zero-delay
/// loops reaches a fixed point in a bounded number of deltas.
pub trait PartitionSolver {
    /// Re-solves every partition named in `dirty` at time `now` and returns
    /// the readers to queue for one evaluation this delta, or `Err` with a
    /// message if any dirty partition's solve failed (a singular or
    /// non-finite pivot, or a non-finite published value) — the kernel-level
    /// counterpart of `ERR_SOLVE_FAILED`; `engine.rs` maps the message onto
    /// `BoError::solve_failed`. `dirty` carries no duplicates the kernel
    /// didn't already collapse, but a partition may legitimately appear that
    /// the implementation already considers clean (a driver commit that
    /// didn't actually change the partition's inputs); the implementation is
    /// free to no-op such an entry and report nothing for it.
    ///
    /// `drivers` is the kernel's full driver table (every driver that
    /// exists, not only ones in `dirty` partitions) — a solver needs it to
    /// read a committed driver's electrical contribution (its node,
    /// hi/lo rails, conductance and levels) when it stamps a partition that
    /// driver's output lives in; `kernel.rs` never interprets a driver's
    /// electrical fields itself.
    fn resolve_dirty(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver]) -> Result<Vec<DeviceId>, String>;

    /// [`Self::resolve_dirty`], appending the readers to `out` (the kernel's
    /// reused buffer) instead of returning a fresh vector: the kernel calls
    /// this at every delta pass, and the allocation and growth of a vector
    /// per call was a measurable share of the SAP-1's cost (F5q). `out` is
    /// untouched on `Err`. The default forwards to `resolve_dirty`.
    fn resolve_dirty_into(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver], out: &mut Vec<DeviceId>) -> Result<(), String> {
        out.extend(self.resolve_dirty(dirty, now, drivers)?);
        Ok(())
    }

    /// Every node's currently published voltage, node 0 = 0
    /// (`bo_engine_node_voltages`'s contract). Whatever the solve method
    /// (static, closed-form, stepped), this is the one array `Engine`
    /// publishes through the ABI, so it lives on the trait rather than only
    /// on a concrete implementation — `engine.rs` reads it through
    /// `Kernel::solver` without needing to know which implementation is
    /// behind it.
    fn voltages(&self) -> &[f64];

    /// Every device's currently published current, in record order
    /// (`bo_engine_device_currents`'s contract). Current as of the last
    /// [`Self::prepare_readout`]; a solver that defers part of it may assert
    /// that nothing is owed.
    fn currents(&self) -> &[f64];

    /// Brings `currents()` and `channels()` up to date with the last publish.
    /// `Engine` calls it at the end of `load`, `advance`, `set_params` and
    /// `seed_memory`, on success and failure alike, before anything reads
    /// them. A solver that computes derived values (source, LED, 555
    /// currents) only when asked, because they are pure functions of the
    /// published state, does that work here. Default: nothing owed.
    fn prepare_readout(&mut self) {}

    /// Owned copies of every array [`Self::resolve_dirty`] can mutate —
    /// today exactly `voltages()` and `currents()`, the only two this trait
    /// exposes at all. `Kernel::advance` takes one of these before each
    /// instant's delta cycle and, if that instant never reaches a quiet
    /// fixed point (a livelock or a solve failure), hands it back to
    /// [`Self::restore`] to undo whatever partial work that doomed attempt
    /// did (core/README.md, "A failed advance": the engine keeps the state
    /// of the last instant it *completed*). This matters even for a plain
    /// solve failure, not only a livelock: `StaticPartitionSolver::resolve_dirty`
    /// writes `voltages` partition by partition and only recomputes
    /// `currents` after every dirty partition solves, so a mid-loop error on
    /// partition k leaves the array a mix of this attempt's new values
    /// (partitions before k) and the previous instant's (partitions at or
    /// after k) — exactly the half-written state a restore must undo.
    ///
    /// Given a default in terms of `voltages`/`currents` because every
    /// implementation already has to provide those; only [`Self::restore`]
    /// needs a per-implementation body, since nothing here can reach into a
    /// concrete solver's private storage to write them back. A solver that
    /// fills part of `currents()` in later (`PartitionedSolver`) overrides
    /// this to copy its raw arrays, and keeps the flag saying what is still
    /// owed in [`Self::dynamic_snapshot`].
    fn snapshot(&self) -> (Vec<f64>, Vec<f64>) {
        (self.voltages().to_vec(), self.currents().to_vec())
    }

    /// Restores exactly what a prior [`Self::snapshot`] call captured.
    fn restore(&mut self, snapshot: (Vec<f64>, Vec<f64>));

    /// The published channel array, one current (amps) per channel slot
    /// (README, LEDs point 13): the readout's channel slots are a copy of it.
    /// `NaN` marks a slot whose `getElementChannelI` entry is absent. Default
    /// empty: a solver with no LED-family device (every mock) publishes no
    /// channel, and `Engine` then leaves its own slots as they are.
    fn channels(&self) -> &[f64] {
        &[]
    }

    /// Toggles the solution-cache exactness switch (`NO_SOLUTION_CACHE`,
    /// core/README.md's Options table, key 2) after load. Default a no-op:
    /// only a solver that actually caches (`static_part::StaticPartitionSolver`)
    /// needs to do anything; a solver kind that never caches at all — every
    /// mock in `kernel.rs`'s own tests included — is unaffected by
    /// definition.
    fn set_no_solution_cache(&mut self, _off: bool) {}

    /// Solution-cache hits so far (contract stat `7`). Default `0.0`: only a
    /// solver that actually caches reports anything else.
    fn cache_hits(&self) -> f64 {
        0.0
    }

    /// Closed-form segments built so far (contract stat `8`). Default `0.0`:
    /// only [`PartitionedSolver`] (which owns any `ClosedForm` partition)
    /// reports anything else.
    fn closed_form_segments(&self) -> f64 {
        0.0
    }

    /// Threshold crossings landed so far (contract stat `9`).
    fn crossings(&self) -> f64 {
        0.0
    }

    /// Committed stepped-partition steps so far (contract stat `10`).
    fn stepped_steps(&self) -> f64 {
        0.0
    }

    /// INT-ACC (R81): how many times a trapezoidal stepped partition fell
    /// back to backward Euler on a ring (stat `27`).
    fn trap_fallbacks(&self) -> f64 {
        0.0
    }

    /// Newton iterations run so far, summed across every stepped step and,
    /// from W1e, every iteration of an iterated static partition, the polish
    /// included (contract stat `11`).
    fn newton_iterations(&self) -> f64 {
        0.0
    }

    /// Validates one `bo_engine_set_params` batch of `(deviceIndex,
    /// paramIndex, value)` triples WITHOUT applying anything (owner
    /// decision 1's "all or nothing": every triple is checked before any is
    /// applied). `Err` names the first problem found. The default rejects
    /// any non-empty batch — `engine.rs` never calls this with an empty one
    /// (it short-circuits first) — since only a solver with mutable
    /// per-device parameters (`static_part::StaticPartitionSolver`) can have
    /// a live one at all.
    fn validate_params(&self, triples: &[(usize, usize, f64)]) -> Result<(), String> {
        let _ = triples;
        Err("this partition solver has no live parameters".to_string())
    }

    /// Applies an already-`validate_params`-checked batch: mutates every
    /// named device's param, bumps whatever epoch the solver's own solution
    /// cache keys on, and returns every partition touched (deduplicated) —
    /// no electrical re-solve here. The caller (`Kernel::apply_params_as_event`)
    /// runs that as one kernel event (owner decision 1: "applied as ONE
    /// event at the current time"). Default a no-op, paired with the default
    /// `validate_params` above, which never lets a non-empty batch reach
    /// here in the first place.
    fn apply_params(&mut self, triples: &[(usize, usize, f64)]) -> Vec<PartitionId> {
        let _ = triples;
        Vec::new()
    }

    /// Reads a device's current live-parameter value, for
    /// `Kernel::apply_params_as_event` to save a pre-image before applying a
    /// batch — so a downstream solve failure or livelock can be undone by
    /// re-applying the old values, the same "keeps the state of the last
    /// completed instant" guarantee `Kernel::advance` already gives a
    /// failed advance. `None` for a device index this solver does not track
    /// live parameters for at all; every triple `validate_params` accepted
    /// is guaranteed to have `Some` here.
    fn param_value(&self, device_index: usize, param_index: usize) -> Option<f64> {
        let _ = (device_index, param_index);
        None
    }

    // ---- W1b step 2: dynamic partitions (closed-form/stepped) -------------
    //
    // Every method below is default-provided and a no-op/empty for a solver
    // with no dynamic partitions at all, so `StaticPartitionSolver` and every
    // pre-existing `#[cfg(test)]` mock solver (`kernel.rs`, `engine.rs`) need
    // no changes to keep satisfying this trait. Only `PartitionedSolver`
    // (this module) overrides them.

    /// Extra dynamic-partition state a snapshot/restore pair must carry
    /// beyond `voltages()`/`currents()`: a closed-form partition's committed
    /// segment anchor (`vc0`, `t0`) and its solved segment coefficients, or a
    /// stepped partition's committed capacitor charge(s), last-accepted
    /// series current(s), pending-step state and step-size controller state —
    /// core/README.md's "A failed advance" and the W1b brief's "A failed
    /// advance rolls back dynamic state too (committed vc)". Opaque to
    /// `Kernel`, which only ever round-trips the box through
    /// [`Self::dynamic_restore`] and never downcasts it — only the concrete
    /// implementor that produced it does, on the way back in. `Box<dyn Any>`
    /// rather than a fixed shape (a second `Vec<f64>`, say) because a
    /// closed-form/stepped partition's state is not a flat float array by
    /// nature (per-partition vectors of varying length, a `segReady`
    /// boolean, per-partition version counters); forcing it through one
    /// would cost this trait's only two implementors with anything to say
    /// here (`PartitionedSolver` is the only one) an artificial encode/decode
    /// step for no reader that ever needs the bytes themselves. Default
    /// `Box::new(())`: a solver with no dynamic partitions has nothing to
    /// carry, and `()` downcasts trivially back to itself if anything ever
    /// mistakenly tried.
    fn dynamic_snapshot(&self) -> Box<dyn std::any::Any> {
        Box::new(())
    }

    /// Restores exactly what a prior [`Self::dynamic_snapshot`] call
    /// captured. Default a no-op, paired with the default snapshot above.
    fn dynamic_restore(&mut self, snapshot: Box<dyn std::any::Any>) {
        let _ = snapshot;
    }

    /// The kernel is about to change the COMMITTED level of driver `idx` in
    /// its table (a commit applied, or a reset), which it does not tell the
    /// solver otherwise: a solver that keeps a copy of the table
    /// (`PartitionedSolver`'s `driver_snapshot`, as of the last
    /// `resolve_dirty`) uses this to refresh only the entries that moved, not
    /// to copy the whole table at every resolve. The committed level is all
    /// the solver reads of a driver that changes (a request only touches the
    /// pending transition and the token), so a change of those needs no call.
    /// Called before every such change, whether or not a window is open. A
    /// kernel that never calls it loses nothing: the solver then copies the
    /// table, as it always did.
    fn driver_changing(&mut self, idx: usize) {
        let _ = idx;
    }

    /// The kernel just changed the COMMITTED level of driver `idx` (a commit
    /// that changed it, or a reset of a committed driver): unlike
    /// [`Self::driver_changing`], only for a change that happened. W1g point
    /// 9: a commit can flip a pin's commanded direction without moving a
    /// node, so the settle scans its `output_sag` cell.
    fn driver_committed(&mut self, idx: usize) {
        let _ = idx;
    }

    /// An `advance` call is about to run the host step `[t_start, t_end]`:
    /// its length is the hold-episode guard's yardstick (ruling R54 (a)).
    fn begin_host_step(&mut self, t_start: f64, t_end: f64) {
        let _ = (t_start, t_end);
    }

    /// A current `Slow` event whose token carries [`crate::events::SLOW_ANCHOR`]
    /// (W1g ruling R21): re-drives the device's rail-driven segment from its
    /// exact value at `now` and fires nothing.
    fn handle_slow_anchor(&mut self, device: DeviceId, now: f64) -> Result<Vec<PartitionId>, String> {
        let _ = (device, now);
        Ok(Vec::new())
    }

    /// The solver's side of the kernel's per-instant rollback window: the
    /// checkpoint a failed instant goes back to. The defaults take exactly
    /// what the kernel took before this method existed (`snapshot` plus
    /// `dynamic_snapshot`) and restore it in the same order, so a solver that
    /// does not override them behaves as it always did. An override may
    /// capture only what the instant can touch, as long as
    /// `rollback_instant` leaves the solver bit for bit where
    /// `begin_instant` found it.
    fn begin_instant(&mut self) -> Box<dyn std::any::Any> {
        Box::new((self.snapshot(), self.dynamic_snapshot()))
    }

    /// The instant completed: the checkpoint is no longer needed.
    fn end_instant(&mut self, checkpoint: Box<dyn std::any::Any>) {
        let _ = checkpoint;
    }

    /// The instant failed: back to the checkpoint.
    fn rollback_instant(&mut self, checkpoint: Box<dyn std::any::Any>) {
        let (snapshot, dynamic) = *checkpoint
            .downcast::<((Vec<f64>, Vec<f64>), Box<dyn std::any::Any>)>()
            .expect("a checkpoint made by this solver's begin_instant");
        self.restore(snapshot);
        self.dynamic_restore(dynamic);
    }

    /// Drains every `(time, EventKind::Crossing | EventKind::Step)` a dynamic
    /// partition has asked to be scheduled since the last drain — a fresh
    /// re-aim from [`Self::resolve_dirty`], [`Self::handle_dynamic_event`] or
    /// [`Self::sync_dynamic_to`] each produce at most one, replacing whatever
    /// that same partition had pending before (its version already bumped, so
    /// the stale entry still on `Kernel`'s heap is recognisably dead —
    /// [`Self::is_current`]). Every entry here has already passed the
    /// crossing-acceptance rule (`kernel::accepts_crossing`) where relevant
    /// (a closed-form re-aim applies it itself before ever producing one,
    /// core/README.md's "Hard-won rules"); `Kernel` pushes each verbatim onto
    /// its own heap and never re-checks it. Default empty.
    fn take_scheduled(&mut self) -> Vec<(f64, EventKind)> {
        Vec::new()
    }

    /// Whether a `Crossing`/`Step` event's carried `version` still matches
    /// `partition`'s current solve epoch (events.rs's lazy-deletion
    /// contract): `is_crossing` selects which of the two epochs (a partition
    /// can be re-aimed and re-planned independently is moot in W1b, since a
    /// partition is only ever one dynamic kind at a time, but the flag keeps
    /// `Kernel` from needing to know that). Default `false`: a solver with no
    /// dynamic partitions never has a live entry to confirm, matching the
    /// fact that nothing ever pushes a `Crossing`/`Step` against it in the
    /// first place.
    fn is_current(&self, partition: PartitionId, is_crossing: bool, version: Version) -> bool {
        let _ = (partition, is_crossing, version);
        false
    }

    /// Applies a due `Crossing` (`is_crossing = true`) or `Step`
    /// (`is_crossing = false`) event that [`Self::is_current`] already
    /// confirmed is live, at `now`: pec3.mjs's `_analyticCross`/`_commitStep`
    /// plus its immediate re-aim/replan. Returns the readers to queue this
    /// delta, exactly like [`Self::resolve_dirty`] — `Kernel` calls this
    /// INSTEAD of `resolve_dirty` for a partition whose own scheduled event is
    /// what became due (nothing marked it dirty). Default empty: unreachable
    /// for a solver [`Self::is_current`] never confirms anything against.
    fn handle_dynamic_event(&mut self, partition: PartitionId, now: f64, is_crossing: bool) -> Result<Vec<DeviceId>, String> {
        let _ = (partition, now, is_crossing);
        Ok(Vec::new())
    }

    /// [`Self::handle_dynamic_event`], appending the readers to `out`, as
    /// [`Self::resolve_dirty_into`] (a crossing is the SAP-1's most frequent
    /// event). `out` is untouched on `Err`.
    fn handle_dynamic_event_into(&mut self, partition: PartitionId, now: f64, is_crossing: bool, out: &mut Vec<DeviceId>) -> Result<(), String> {
        out.extend(self.handle_dynamic_event(partition, now, is_crossing)?);
        Ok(())
    }

    /// The horizon clamp (core/README.md, "Kernel semantics: events", point
    /// 5): brings every dynamic partition's PUBLISHED `voltages()`/`currents()`
    /// entries exactly to `t_end`, so a readout right after `bo_engine_advance`
    /// is a solved value, never an interpolation. The two dynamic kinds need
    /// different amounts of work to get there:
    /// - **Stepped**: takes however many bounded BE/trap steps land it
    ///   exactly on `t_end` (pec3.mjs's `_syncDynamic`, called with `now =
    ///   t_end`) — the same real integration work an ordinary step does, just
    ///   horizon-bounded.
    /// - **Closed-form**: its node voltages are already the exact trajectory
    ///   at any `t`, so there is no INTEGRATION to do — but the value SITTING
    ///   in the shared `voltages()`/`currents()` arrays was only ever written
    ///   at the last re-aim/crossing time, which is generally still `< t_end`.
    ///   This still has to evaluate `_vcAt(t_end)` and re-publish every node
    ///   and the capacitor's current from it — cheap (no LU solve, no
    ///   iteration, just the closed-form expression), but not a no-op.
    ///
    /// Called by `Kernel::advance` once, after its event loop has drained
    /// everything up to and including `t_end`, inside the same
    /// snapshot/rollback envelope as any other instant — a solve failure here
    /// (a stepped partition's forced step fails to converge) is reported and
    /// rolled back exactly like a mid-`advance` one. Default a no-op
    /// returning no readers: nothing for a solver with no dynamic partitions
    /// to do.
    fn sync_dynamic_to(&mut self, t_end: f64) -> Result<Vec<DeviceId>, String> {
        let _ = t_end;
        Ok(Vec::new())
    }

    // ---- W1c step 2: the digital core's solver-side hooks -----------------
    //
    // Every method below is default-provided, matching the pattern above: a
    // solver with no notion of an IC (every implementor before this step)
    // needs no changes to keep satisfying this trait.

    /// Which partition `node` belongs to, or `None` when it names no free
    /// node this solver tracks a partition for at all — a fixed node
    /// (contract point 4: "a driver whose node is fixed belongs to no
    /// partition"), or an out-of-range one. Used to assign a newly-created
    /// [`crate::drivers::Driver`] its `Kernel::add_driver` partition (`None`
    /// there is exactly this method's `None`, threaded straight through).
    /// Default `None`: a solver with no partition concept of its own (every
    /// `#[cfg(test)]` mock in `kernel.rs`) answers every node this way.
    fn partition_of_node(&self, node: usize) -> Option<PartitionId> {
        let _ = node;
        None
    }

    /// Whether the IC at device index `device` is currently powered
    /// (`digital::power::verdict`: its power hold's, else the window test on
    /// the latest PUBLISHED voltages) — the kernel's power-loss hold (contract point 9)
    /// calls this to decide whether a due `Commit` for one of that IC's
    /// drivers applies now or is held. Default `true`: "true for a
    /// non-IC" — every driver through W1b has `Driver::device ==
    /// usize::MAX`, for which the kernel never even calls this, but a
    /// solver with no IC concept at all (every mock) answering `true`
    /// unconditionally is the correct, permissive default regardless.
    fn ic_powered(&self, device: usize) -> bool {
        let _ = device;
        true
    }

    /// The floating-bucket boundary handler (W1c contract point 5/6):
    /// called by `Kernel::apply_due_events` when a popped `EventKind::Bucket`
    /// fires. Returns `(partitions to mark dirty, readers to evaluate this
    /// delta)` — every partition holding an `OPEN` `FLOATING_INPUT` pin of a
    /// powered IC (its pull target may have flipped, so it needs an ordinary
    /// re-solve through `PartitionSolver::resolve_dirty`, folded into THIS
    /// delta's dirty set) and every IC that either holds such a pin or reads
    /// a pin in its dead band (added directly to the readers this delta
    /// evaluates, exactly like a `Crossing`/`Step`'s own directly-affected
    /// readers — its verdict can change from the bucket alone, with no
    /// voltage having moved for `resolve_dirty` to notice on its own).
    /// Default `(Vec::new(), Vec::new())`: a solver with no IC never
    /// schedules a `Bucket` in the first place (see
    /// [`Self::take_scheduled`]'s own doc comment), so this is unreachable
    /// for one in practice.
    fn bucket_boundary(&mut self, now: f64) -> (Vec<PartitionId>, Vec<DeviceId>) {
        let _ = now;
        (Vec::new(), Vec::new())
    }

    // ---- W1f: rails and sources ---------------------------------------------
    //
    // Default-provided like every hook above: a solver with no rail and no
    // waveform source (every mock) never schedules either event.

    /// Whether a popped `Rail { version }` is the rail's pending event
    /// (README, Rails point 4).
    fn rail_current(&self, version: Version) -> bool {
        let _ = version;
        false
    }

    /// A due `Rail` event: SoC advances, the closed-form draws refresh, the
    /// rail solves, the regime rules run, and the next event is scheduled.
    /// Returns `(partitions to re-solve, readers)` like `bucket_boundary`, or
    /// a solve failure.
    fn rail_event(&mut self, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
        let _ = now;
        Ok((Vec::new(), Vec::new()))
    }

    /// Whether a popped `Source { device, version }` is that source's next
    /// edge (Sources point 5).
    fn source_current(&self, device: DeviceId, version: Version) -> bool {
        let _ = (device, version);
        false
    }

    /// A due waveform source event (Sources points 2 and 3): an edge, where
    /// the source's nodes take their new value, the slot updates and the next
    /// edge is scheduled; or, on a resolved ramp, a reader's threshold
    /// crossing. Returns `(partitions to re-solve, readers)`.
    fn source_edge(&mut self, device: DeviceId, version: Version, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
        let _ = (device, version, now);
        Ok((Vec::new(), Vec::new()))
    }

    /// One device's runtime state (C ABI, `bo_engine_device_state`) at `now`,
    /// written into `out`; `None` for a kind with none. Never changes the
    /// simulation.
    fn device_state(&self, device: usize, now: f64, out: &mut Vec<f64>) -> Option<usize> {
        let _ = (device, now, out);
        None
    }

    /// Whether a resolve since the last call moved the rail by more than
    /// 1e-9 V or changed a regime (Sources point 4), clearing the note.
    fn take_settle_change(&mut self) -> bool {
        false
    }

    /// The waveform sources' digital slots, `(slot, value)` (Readouts).
    fn source_slots(&self, out: &mut Vec<(usize, f64)>) {
        let _ = out;
    }

    /// Stats 14 and 16-20: rail moves, sensitivity computations, rail scans,
    /// rail events, source edges, scan flips.
    fn rail_stats(&self) -> [f64; 6] {
        [0.0; 6]
    }

    /// The node the hub rail publishes, whose `Rail` events exist to move
    /// it: the kernel's no-progress guard counts a `Rail`-only instant that
    /// left it where it was ([`crate::kernel::NO_PROGRESS_MAX`]).
    fn rail_node(&self) -> Option<usize> {
        None
    }

    // ---- W1g: thermal, failures and warnings --------------------------------
    //
    // Default-provided like every hook above (README W1g point 17): a solver
    // with no slow device (every mock) settles nothing, schedules no `Slow`
    // event and has no records.

    /// The settle at the end of an instant (README W1g points 2 and 14), run
    /// by the kernel once the delta cycle has quiesced: closes and re-drives
    /// the slow devices of every partition published this instant, runs the
    /// `output_sag` scan, and returns the partitions a latch due at once
    /// dirtied (another delta cycle runs at `now`, then this again). `None`
    /// when it scheduled nothing and dirtied nothing (most instants), so the
    /// kernel has nothing to drain.
    fn slow_settle(&mut self, now: f64, drivers: &[Driver]) -> Result<Option<Vec<PartitionId>>, String> {
        let _ = (now, drivers);
        Ok(None)
    }

    /// Whether a popped `Slow { device, token }` still names the device's
    /// pending event (its token has not moved on).
    fn slow_is_current(&self, device: DeviceId, token: u32) -> bool {
        let _ = (device, token);
        false
    }

    /// A due, current `Slow` event (README W1g point 14): closes the device to
    /// `now` and applies the latch, re-arm, PTC trip or recovery, or marks an
    /// `output_sag` window due for the settle. Returns the partitions dirtied.
    fn handle_slow_event(&mut self, device: DeviceId, now: f64) -> Result<Vec<PartitionId>, String> {
        let _ = (device, now);
        Ok(Vec::new())
    }

    /// `resetFailures` (README W1g point 13), applied by the kernel as one
    /// event at `now`: returns the partitions whose stamps returned.
    fn reset_failures(&mut self, now: f64) -> Result<Vec<PartitionId>, String> {
        let _ = now;
        Ok(Vec::new())
    }

    /// One `bo_engine_records` channel at `now`, 7 f64 a record, into `out`
    /// (cleared first). `false` for an unknown channel. Channels 0-2 project
    /// without committing anything; channel 3 is a take.
    fn collect_records(&mut self, channel: u32, now: f64, out: &mut Vec<f64>) -> bool {
        let _ = now;
        out.clear();
        channel <= 3
    }

    /// The W1g device state (thermal, PTC) of `device` at `now`, or `None`
    /// for a device with no W1g layout.
    fn slow_device_state(&self, device: usize, now: f64, out: &mut Vec<f64>) -> Option<usize> {
        let _ = (device, now, out);
        None
    }

    /// Stats 21-24: slow interval closes, slow events fired, latches,
    /// junction re-arms (README W1g point 18).
    fn slow_stats(&self) -> [f64; 4] {
        [0.0; 4]
    }

    /// The power-hold table (W1c point 1a), device-indexed, as the evaluator
    /// judges an IC's power by it ([`crate::kernel::DeviceEvaluator`]'s
    /// `holds` argument): empty while no IC is held. Default empty, a solver
    /// with no holds.
    fn power_holds(&self) -> &[digital::power::PowerHold] {
        &[]
    }

    /// A flag that reads `true` exactly while [`Self::power_holds`] is not
    /// empty, shared with the kernel at its construction: the kernel reads
    /// it instead of calling through the trait object at every delta pass
    /// and instant (the SAP-1 never holds; F5q). `None`, the default, has
    /// the kernel call [`Self::power_holds`] every time.
    fn holds_flag(&self) -> Option<Rc<std::cell::Cell<bool>>> {
        None
    }

    /// Drops every power hold, at a successful `set_params`, `seed_memory`
    /// or `resetFailures` (the circuit itself changed), returning the
    /// partitions whose stamps change and the ICs released, which read at
    /// once. Called by the kernel inside the event's window, so a failed
    /// event rolls the drop back.
    fn drop_power_holds(&mut self) -> (Vec<PartitionId>, Vec<DeviceId>) {
        (Vec::new(), Vec::new())
    }

    /// Power holds taken and released (D8).
    fn hold_stats(&self) -> [f64; 2] {
        [0.0; 2]
    }

    /// Test builds: sets one IC's hold as the hold procedure would, returning
    /// the partitions to re-solve ([`crate::kernel::Kernel::hold_power_as_event`]).
    #[cfg(test)]
    fn hold_power(&mut self, device: usize, hold: digital::power::PowerHold) -> Vec<PartitionId> {
        let _ = (device, hold);
        unimplemented!("this solver takes no power hold")
    }

    /// A boxed deep copy, for W2a's snapshots (the dyn-clone pattern).
    /// [`PartitionedSolver`], the one solver `Engine::load` installs,
    /// overrides it; the test mocks never snapshot, so the default panics.
    fn clone_box(&self) -> Box<dyn PartitionSolver> {
        unimplemented!("this solver is never snapshotted")
    }

    /// The heap this solver owns, for `bo_engine_snapshot_bytes`'s walk
    /// ([`crate::snapshot::HeapBytes`]). Default none: the mocks never
    /// snapshot.
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let _ = w;
        0
    }

    /// Empties the test builds' instrumentation logs, which grow for the
    /// whole run, so a size check measures the engine's own state.
    #[cfg(test)]
    fn clear_test_logs(&mut self) {}

    /// Appends the carry records (`crate::carry`) of the state this solver
    /// owns, at `now`: capacitors, a battery or PSU on the rail, and the slow
    /// physics with their latches. Default none.
    fn carry_take(&self, now: f64, out: &mut Vec<f64>) {
        let _ = (now, out);
    }
}

impl Clone for Box<dyn PartitionSolver> {
    fn clone(&self) -> Self {
        (**self).clone_box()
    }
}

/// So `Kernel<Box<dyn PartitionSolver>, _>` satisfies the trait bound
/// directly: `engine.rs`'s test-only harness swaps the whole solver to prove
/// `Engine::advance` propagates a livelock end to end, which needs a solver
/// type that can vary at runtime behind `Engine`'s otherwise-concrete
/// `kernel` field.
impl PartitionSolver for Box<dyn PartitionSolver> {
    fn resolve_dirty(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
        (**self).resolve_dirty(dirty, now, drivers)
    }

    fn resolve_dirty_into(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver], out: &mut Vec<DeviceId>) -> Result<(), String> {
        (**self).resolve_dirty_into(dirty, now, drivers, out)
    }

    fn voltages(&self) -> &[f64] {
        (**self).voltages()
    }

    fn currents(&self) -> &[f64] {
        (**self).currents()
    }

    // Forwarded, not left to the default: the default reads `self.currents()`,
    // which `PartitionedSolver` forbids while its readout pass is owed (see
    // its own `snapshot`).
    fn snapshot(&self) -> (Vec<f64>, Vec<f64>) {
        (**self).snapshot()
    }

    fn restore(&mut self, snapshot: (Vec<f64>, Vec<f64>)) {
        (**self).restore(snapshot)
    }

    fn prepare_readout(&mut self) {
        (**self).prepare_readout()
    }

    fn channels(&self) -> &[f64] {
        (**self).channels()
    }

    fn set_no_solution_cache(&mut self, off: bool) {
        (**self).set_no_solution_cache(off)
    }

    fn cache_hits(&self) -> f64 {
        (**self).cache_hits()
    }

    fn closed_form_segments(&self) -> f64 {
        (**self).closed_form_segments()
    }

    fn crossings(&self) -> f64 {
        (**self).crossings()
    }

    fn stepped_steps(&self) -> f64 {
        (**self).stepped_steps()
    }

    fn trap_fallbacks(&self) -> f64 {
        (**self).trap_fallbacks()
    }

    fn newton_iterations(&self) -> f64 {
        (**self).newton_iterations()
    }

    fn validate_params(&self, triples: &[(usize, usize, f64)]) -> Result<(), String> {
        (**self).validate_params(triples)
    }

    fn apply_params(&mut self, triples: &[(usize, usize, f64)]) -> Vec<PartitionId> {
        (**self).apply_params(triples)
    }

    fn param_value(&self, device_index: usize, param_index: usize) -> Option<f64> {
        (**self).param_value(device_index, param_index)
    }

    fn dynamic_snapshot(&self) -> Box<dyn std::any::Any> {
        (**self).dynamic_snapshot()
    }

    fn dynamic_restore(&mut self, snapshot: Box<dyn std::any::Any>) {
        (**self).dynamic_restore(snapshot)
    }

    fn driver_changing(&mut self, idx: usize) {
        (**self).driver_changing(idx)
    }

    fn driver_committed(&mut self, idx: usize) {
        (**self).driver_committed(idx)
    }

    fn begin_host_step(&mut self, t_start: f64, t_end: f64) {
        (**self).begin_host_step(t_start, t_end)
    }

    fn handle_slow_anchor(&mut self, device: DeviceId, now: f64) -> Result<Vec<PartitionId>, String> {
        (**self).handle_slow_anchor(device, now)
    }

    fn begin_instant(&mut self) -> Box<dyn std::any::Any> {
        (**self).begin_instant()
    }

    fn end_instant(&mut self, checkpoint: Box<dyn std::any::Any>) {
        (**self).end_instant(checkpoint)
    }

    fn rollback_instant(&mut self, checkpoint: Box<dyn std::any::Any>) {
        (**self).rollback_instant(checkpoint)
    }

    fn take_scheduled(&mut self) -> Vec<(f64, EventKind)> {
        (**self).take_scheduled()
    }

    fn is_current(&self, partition: PartitionId, is_crossing: bool, version: Version) -> bool {
        (**self).is_current(partition, is_crossing, version)
    }

    fn handle_dynamic_event(&mut self, partition: PartitionId, now: f64, is_crossing: bool) -> Result<Vec<DeviceId>, String> {
        (**self).handle_dynamic_event(partition, now, is_crossing)
    }

    fn handle_dynamic_event_into(&mut self, partition: PartitionId, now: f64, is_crossing: bool, out: &mut Vec<DeviceId>) -> Result<(), String> {
        (**self).handle_dynamic_event_into(partition, now, is_crossing, out)
    }

    fn sync_dynamic_to(&mut self, t_end: f64) -> Result<Vec<DeviceId>, String> {
        (**self).sync_dynamic_to(t_end)
    }

    fn partition_of_node(&self, node: usize) -> Option<PartitionId> {
        (**self).partition_of_node(node)
    }

    fn ic_powered(&self, device: usize) -> bool {
        (**self).ic_powered(device)
    }

    fn bucket_boundary(&mut self, now: f64) -> (Vec<PartitionId>, Vec<DeviceId>) {
        (**self).bucket_boundary(now)
    }

    fn rail_current(&self, version: Version) -> bool {
        (**self).rail_current(version)
    }

    fn rail_event(&mut self, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
        (**self).rail_event(now)
    }

    fn source_current(&self, device: DeviceId, version: Version) -> bool {
        (**self).source_current(device, version)
    }

    fn source_edge(&mut self, device: DeviceId, version: Version, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
        (**self).source_edge(device, version, now)
    }

    fn device_state(&self, device: usize, now: f64, out: &mut Vec<f64>) -> Option<usize> {
        (**self).device_state(device, now, out)
    }

    fn take_settle_change(&mut self) -> bool {
        (**self).take_settle_change()
    }

    fn source_slots(&self, out: &mut Vec<(usize, f64)>) {
        (**self).source_slots(out)
    }

    fn rail_stats(&self) -> [f64; 6] {
        (**self).rail_stats()
    }

    fn rail_node(&self) -> Option<usize> {
        (**self).rail_node()
    }

    fn slow_settle(&mut self, now: f64, drivers: &[Driver]) -> Result<Option<Vec<PartitionId>>, String> {
        (**self).slow_settle(now, drivers)
    }

    fn slow_is_current(&self, device: DeviceId, token: u32) -> bool {
        (**self).slow_is_current(device, token)
    }

    fn handle_slow_event(&mut self, device: DeviceId, now: f64) -> Result<Vec<PartitionId>, String> {
        (**self).handle_slow_event(device, now)
    }

    fn reset_failures(&mut self, now: f64) -> Result<Vec<PartitionId>, String> {
        (**self).reset_failures(now)
    }

    fn collect_records(&mut self, channel: u32, now: f64, out: &mut Vec<f64>) -> bool {
        (**self).collect_records(channel, now, out)
    }

    fn slow_device_state(&self, device: usize, now: f64, out: &mut Vec<f64>) -> Option<usize> {
        (**self).slow_device_state(device, now, out)
    }

    fn slow_stats(&self) -> [f64; 4] {
        (**self).slow_stats()
    }

    fn power_holds(&self) -> &[digital::power::PowerHold] {
        (**self).power_holds()
    }

    fn holds_flag(&self) -> Option<Rc<std::cell::Cell<bool>>> {
        (**self).holds_flag()
    }

    fn clone_box(&self) -> Box<dyn PartitionSolver> {
        (**self).clone_box()
    }

    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        std::mem::size_of_val::<dyn PartitionSolver>(&**self) + (**self).heap_bytes(w)
    }

    #[cfg(test)]
    fn clear_test_logs(&mut self) {
        (**self).clear_test_logs()
    }

    fn carry_take(&self, now: f64, out: &mut Vec<f64>) {
        (**self).carry_take(now, out)
    }

    fn drop_power_holds(&mut self) -> (Vec<PartitionId>, Vec<DeviceId>) {
        (**self).drop_power_holds()
    }

    fn hold_stats(&self) -> [f64; 2] {
        (**self).hold_stats()
    }

    #[cfg(test)]
    fn hold_power(&mut self, device: usize, hold: digital::power::PowerHold) -> Vec<PartitionId> {
        (**self).hold_power(device, hold)
    }
}

// ============================================================================
// Shared dynamic-partition (closed-form/stepped) support.
// ============================================================================
//
// A `ClosedFormPartition`/`SteppedPartition` (their own modules) both stamp
// the SAME capacitor-independent linear network — resistors, a capacitor's
// own leakage, and switch-family contacts — and differ only in how they
// additionally treat the capacitor(s) themselves (the exact `M_SOURCE`
// branch-current unknown for closed form, vs. a BE/trap companion for
// stepped). [`DynShape`] and the `stamp_*`/`compile_dyn_shape` functions
// below are that shared half, so neither module repeats it.

/// One reader's current threshold on a watched node (the crossing-watch API,
/// W1 plan row B2, used by both `closed_form` and `stepped`): `value` in
/// volts, `rising` selects which direction counts as a crossing (`true`: now
/// below, heading above; `false`: the reverse).
#[derive(Debug, Clone, Copy)]
pub struct Threshold {
    pub value: f64,
    pub rising: bool,
}

/// A registered reader's threshold function: called fresh every time a
/// dynamic partition re-aims/re-plans (the whole point of a re-aim is that
/// the trajectory changed, so a stale threshold snapshot would miss a moved
/// comparator reference or a Schmitt band) to get whatever it currently
/// wants watched on its node. `Fn`, not `FnMut`: a reader reports its OWN
/// current reference, it does not need to mutate anything to do so.
///
/// Takes the full, currently-published voltage array (finding 2/6): a
/// no-arg version cannot see anything beyond what it closed over at
/// registration time, which is wrong the instant a watched IC's supply is
/// allowed to be anything other than a truly fixed node (contract point 4,
/// amended) -- a supply sitting in a `Static` partition can move whenever
/// that partition is dirtied, and the threshold must move with it. Reading
/// straight out of the published array (rather than being handed just the
/// two supply voltages) keeps this a single, uniform shape regardless of
/// which node(s) a given reader's own thresholds happen to depend on.
pub type ThresholdFn = Box<dyn Fn(&[f64]) -> Vec<Threshold>>;

/// A registered [`ThresholdFn`] as a partition stores it: shared, so a
/// partition clones (W2a's snapshot) without copying a closure, which a
/// `Box<dyn Fn>` cannot do. It is never mutated after registration.
pub type SharedThresholdFn = Rc<dyn Fn(&[f64]) -> Vec<Threshold>>;

/// One of a device's own pins, already resolved against a SPECIFIC dynamic
/// partition: `local` is `Some(index)` into that partition's own node list
/// when the pin is free and belongs to it, `None` when the pin is a non-free
/// node, whose voltage a stamp reads through the live boundary ([`Bnd`]) at
/// `node` (W1f, Rails point 7: no fixed-node voltage is baked at build any
/// more, since the rail and a waveform source's nodes move). A pin can never
/// resolve to a DIFFERENT partition's free node: every device
/// `compile_dyn_shape` considers has every one of its free pins in the
/// partition being compiled, since `partition::build`'s union-find
/// guarantees a device's free pins share one partition — for a switch-family
/// device, by unioning every one of its free pins together directly
/// (`partition.rs`'s module doc comment), not merely each contact's own two
/// ends, precisely so a pin can never land here resolved against the wrong
/// partition. `compile_dyn_shape`'s own `resolve` enforces this as a hard
/// error rather than trusting it silently.
#[derive(Debug, Clone, Copy)]
pub(crate) struct DynPin {
    pub local: Option<usize>,
    pub node: usize,
}

/// The live boundary a stamp reads every non-free node through (README,
/// Rails point 7): the published voltage array, whose non-free entries are
/// the boundary values, with four overrides that only the rail and the
/// waveform sources need.
/// - `rail`: the rail node read at another voltage than the published one,
///   for the sensitivity residuals at `u + RAIL_DELTA` (Rails point 3).
/// - `ext`: the rail node stamped as one more unknown at a local index, so
///   one residual stamp yields both the partition's residual and, in the
///   extra row, the current the partition draws from the rail.
/// - `ramps`: the nodes under a resolved pulse ramp, evaluated at the step's
///   own instant (Sources point 3).
/// - `power_u`: the rail at which an IC whose `vcc` or `gnd` is the `ext`
///   rail is judged powered, held at the solve's `u` for the sensitivity
///   residuals: power flips only at a rail event (Rails point 4), so the
///   slope is the present power state's. Judged at `u + RAIL_DELTA` instead,
///   a rail that landed on a falling power floor, within `RAIL_DELTA` under
///   it, differenced an unpowered IC against a powered one: `1/R_out` over
///   an output's shunt-only node, about -2.4e14 S per output for a 74LS00
///   at 4.275 V, which sent a CC rail capacitor's segment to infinity.
/// A partition that reads none of these sees exactly the values it baked
/// before W1f, bit for bit: the published array holds each constant fixed
/// node's propagated value and nothing writes it.
#[derive(Clone, Copy)]
pub(crate) struct Bnd<'a> {
    values: &'a [f64],
    ext: Option<(usize, usize)>,
    ramps: Option<(&'a live::RampEval, f64)>,
    power_u: Option<f64>,
    /// The solver's hold table (device-indexed; empty while nothing is
    /// held), which every IC stamp's power verdict consults (W1c point 1a).
    holds: &'a [digital::power::PowerHold],
}

impl<'a> Bnd<'a> {
    pub(crate) fn new(values: &'a [f64]) -> Self {
        Bnd { values, ext: None, ramps: None, power_u: None, holds: &[] }
    }

    /// Judges every IC's power under `holds` (the solver's table).
    pub(crate) fn with_holds(mut self, holds: &'a [digital::power::PowerHold]) -> Self {
        self.holds = holds;
        self
    }

    /// The hold of the IC at device index `device`.
    #[inline]
    pub(crate) fn hold(&self, device: usize) -> digital::power::PowerHold {
        digital::power::hold_of(self.holds, device)
    }

    pub(crate) fn with_ext(mut self, node: usize, local: usize) -> Self {
        self.ext = Some((node, local));
        self
    }

    /// Judges the power of an IC on the `ext` rail at `u` whatever the rail
    /// unknown reads (a sensitivity's residuals).
    pub(crate) fn hold_power_at(mut self, u: f64) -> Self {
        self.power_u = Some(u);
        self
    }

    /// `pin`'s voltage for an IC power test: the held rail for the `ext`
    /// rail when one is held, else [`pin_voltage`].
    #[inline]
    pub(crate) fn power_voltage(&self, pin: &DynPin, x: &[f64]) -> f64 {
        match (self.power_u, self.ext) {
            (Some(u), Some((rail, _))) if pin.local.is_none() && pin.node == rail => u,
            _ => pin_voltage(pin, x, self),
        }
    }

    /// The boundary at instant `t` for the nodes a resolved ramp moves; the
    /// rest read their published value. `None` leaves the ramps out.
    pub(crate) fn at_time(mut self, ramps: Option<&'a live::RampEval>, t: f64) -> Self {
        self.ramps = ramps.filter(|r| r.any_ramping()).map(|r| (r, t));
        self
    }

    #[inline]
    pub(crate) fn get(&self, node: usize) -> f64 {
        if let Some((ramps, t)) = self.ramps {
            if let Some(v) = ramps.value(node, t, self.values) {
                return v;
            }
        }
        self.values[node]
    }

    /// `pin`'s local index, the rail's included when it is stamped as an
    /// unknown.
    #[inline]
    pub(crate) fn local(&self, pin: &DynPin) -> Option<usize> {
        match (pin.local, self.ext) {
            (Some(l), _) => Some(l),
            (None, Some((rail, l))) if rail == pin.node => Some(l),
            _ => None,
        }
    }
}

/// A resistor or switch-family device touching this partition, kept by
/// device index (for `PartitionedSolver`'s live-parameter dispatch) and kind
/// (for `devices::stamp`) rather than a precomputed conductance, so a
/// switch's current parameters are read fresh on every stamp — the same
/// "never cached the way a resistor's stamp is baked in" rule
/// `devices/switches.rs` already states for the static solver.
#[derive(Debug, Clone)]
pub(crate) struct DynStamped {
    pub device_index: usize,
    pub kind: u32,
    pub pins: Vec<DynPin>,
    /// The device's params AT COMPILE TIME — correct forever for a resistor
    /// (no live params), and only a fallback for a switch-family device
    /// whose CURRENT value is looked up in `PartitionedSolver`'s canonical
    /// `live_params` map instead wherever the caller has one (every call
    /// site in `closed_form`/`stepped` does).
    pub params: Vec<f64>,
}

/// The one capacitor in a `ClosedForm` partition, or one of several in a
/// `Stepped` one. Immutable after load (`capacitor`'s `live_params` is
/// empty, core/README.md's record-layout table), so — unlike
/// [`DynStamped`] — `c`/`esr`/`leak_g` are simply read directly, never
/// looked up live.
#[derive(Debug, Clone, Copy)]
pub(crate) struct DynCapacitor {
    pub device_index: usize,
    pub pins: [DynPin; 2],
    pub c: f64,
    pub esr: f64,
    /// `1/leakage`, or `0.0` for "no leakage" (classic's infinite parallel
    /// resistance) — already resolved so callers never re-derive it.
    pub leak_g: f64,
}

/// One STAMPED LED junction (anode and cathode nodes differ), resolved
/// against the one partition that holds it (README, LEDs point 4): each end is
/// a local index when free (necessarily in this partition, by the junction
/// union) or a fixed voltage. Its companion depends on the Newton iterate, so
/// unlike a [`DynStamped`] entry it is stamped fresh from `x` every
/// iteration ([`stamp_junctions`]).
#[derive(Debug, Clone, Copy)]
pub(crate) struct DynJunction {
    pub anode: DynPin,
    pub cathode: DynPin,
    pub law: led::JunctionLaw,
}

/// A dynamic partition's whole capacitor-independent-plus-capacitor
/// compiled shape: every device `compile_dyn_shape` found touching it.
#[derive(Debug, Clone)]
pub(crate) struct DynShape {
    /// Node ids, in the SAME order `partition::build` assigned local indices
    /// — `nodes[l]` is the global node id at local index `l`.
    pub nodes: Vec<usize>,
    pub n: usize,
    pub stamped: Vec<DynStamped>,
    pub capacitors: Vec<DynCapacitor>,
    /// Every stamped LED junction touching this partition, whichever device
    /// owns it (W1e). Never non-empty for a `ClosedForm` partition
    /// (`closed_form::ClosedFormSolver::build` asserts it).
    pub junctions: Vec<DynJunction>,
}

/// Resolves `node` against partition `p_idx`: its local index when free in
/// `p_idx`, the node itself (read through the live boundary) when it is not
/// free. A node free in a DIFFERENT partition is a partitioning regression,
/// not a value a fixed-0V fallback should ever paper over (`DynPin`'s doc
/// comment).
pub(crate) fn resolve_pin(node: usize, p_idx: usize, local_index: &[Option<(usize, usize)>], fixed: &[Option<f64>]) -> DynPin {
    match local_index[node] {
        Some((p, l)) if p == p_idx => DynPin { local: Some(l), node },
        Some((p, _)) => unreachable!(
            "compile_dyn_shape: node {node} is free but lives in partition {p}, not the partition {p_idx} being compiled — \
             partition::build's union-find guarantees a device's free pins share one partition (partition.rs's module doc \
             comment); this is a partitioning regression, not a value a fixed-0V fallback should ever paper over"
        ),
        None => {
            assert!(fixed[node].is_some(), "a node absent from local_index must be fixed");
            DynPin { local: None, node }
        }
    }
}

/// The partition a junction belongs to: the one holding its free end, or
/// `None` for a junction that is not stamped or sits across two fixed nodes
/// (README, LEDs point 4: it belongs to no partition). By the junction union
/// a junction with two free ends has both in one partition.
pub(crate) fn junction_partition(j: &led::Junction, local_index: &[Option<(usize, usize)>]) -> Option<usize> {
    if !j.stamped() {
        return None;
    }
    local_index[j.anode].or(local_index[j.cathode]).map(|(p, _)| p)
}

/// `j` resolved against partition `p_idx`, or `None` when it is not stamped
/// or does not touch that partition.
pub(crate) fn place_junction(j: &led::Junction, p_idx: usize, local_index: &[Option<(usize, usize)>], fixed: &[Option<f64>]) -> Option<DynJunction> {
    if junction_partition(j, local_index) != Some(p_idx) {
        return None;
    }
    Some(DynJunction { anode: resolve_pin(j.anode, p_idx, local_index, fixed), cathode: resolve_pin(j.cathode, p_idx, local_index, fixed), law: j.law })
}

/// Stamps every junction's companion about the iterate `x` (the partition's
/// local solution vector) into `a`/`b`: conductance `gd + GMIN_D` and Norton
/// offset `id - gd * vPrev`, so the branch current anode to cathode is `g *
/// (vA - vK) + ieq` (LEDs point 2, `stamp_two_terminal`'s convention).
pub(crate) fn stamp_junctions(junctions: &[DynJunction], size: usize, a: &mut [f64], b: &mut [f64], x: &[f64], bnd: &Bnd) {
    for j in junctions {
        let v_prev = pin_voltage(&j.anode, x, bnd) - pin_voltage(&j.cathode, x, bnd);
        let (g, ieq) = j.law.companion(v_prev);
        stamp_two_terminal(size, a, b, &j.anode, &j.cathode, g, ieq, bnd);
    }
}

/// A stamped junction's branch current at the published voltages, anode to
/// cathode, GMIN included: `I(dV) + GMIN_D * dV` (LEDs point 12), which at
/// convergence equals the companion current classic's source branch carries.
/// This is what enters the source KCL; `elementI` excludes GMIN.
pub(crate) fn junction_branch_current(j: &led::Junction, voltages: &[f64]) -> f64 {
    let v = voltages[j.anode] - voltages[j.cathode];
    j.law.current(v) + led::GMIN_D * v
}

/// Compiles partition `p_idx`'s devices into a [`DynShape`]: every resistor,
/// switch-family device or capacitor with at least one pin whose free node
/// lands in `p_idx` (per `local_index`) — never one whose pins are ALL
/// fixed or land in a DIFFERENT partition. Called once per dynamic partition
/// at build time by `closed_form::build`/`stepped::build`.
pub(crate) fn compile_dyn_shape(p_idx: usize, nodes: &[usize], local_index: &[Option<(usize, usize)>], fixed: &[Option<f64>], program: &Program) -> DynShape {
    let resolve = |node: usize| -> DynPin { resolve_pin(node, p_idx, local_index, fixed) };
    let touches_partition = |device: &crate::program::Device| -> bool {
        device.pins.iter().any(|pin| matches!(local_index[pin.node], Some((pp, _)) if pp == p_idx))
    };

    let mut stamped = Vec::new();
    let mut capacitors = Vec::new();
    let mut junctions = Vec::new();
    for (d, device) in program.devices.iter().enumerate() {
        if !touches_partition(device) {
            continue;
        }
        match device.kind {
            // An LED-family device is placed junction by junction (README,
            // LEDs point 4), never by owner: some of its pins may sit in
            // another partition, or be fixed, with that junction belonging
            // elsewhere.
            kinds::LED | kinds::RGB_LED | kinds::BICOLOR_LED | kinds::SEG7_CC | kinds::SEG7_CA => {
                junctions.extend(led::junctions(d, device, program.options.ambient_c).iter().filter_map(|j| place_junction(j, p_idx, local_index, fixed)));
            }
            // W1g: a fuse and a PTC stamp as a resistor does; their slow state
            // reaches the stamp through `live_params` (`devices::slow_flag`).
            kinds::RESISTOR | kinds::FUSE | kinds::PTC_FUSE | kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH => {
                let pins = device.pins.iter().map(|p| resolve(p.node)).collect();
                stamped.push(DynStamped { device_index: d, kind: device.kind, pins, params: device.params.clone() });
            }
            kinds::CAPACITOR => {
                if device.pins[0].node == device.pins[1].node {
                    // Self-shorted (contract point 2): stamps nothing and
                    // does not count toward classification, so it must not
                    // appear as this (or any) partition's capacitor either —
                    // `partition::build`'s `capacitor_count` already agrees
                    // by skipping it, which is what a `ClosedForm`
                    // partition's `debug_assert_eq!(shape.capacitors.len(),
                    // 1)` depends on staying true.
                    continue;
                }
                let pins = [resolve(device.pins[0].node), resolve(device.pins[1].node)];
                let leak = device.params[2];
                capacitors.push(DynCapacitor {
                    device_index: d,
                    pins,
                    c: device.params[0],
                    esr: device.params[1],
                    leak_g: if leak > 0.0 { 1.0 / leak } else { 0.0 },
                });
            }
            _ => {}
        }
    }
    DynShape { nodes: nodes.to_vec(), n: nodes.len(), stamped, capacitors, junctions }
}

/// pec3.mjs's `stampH`/`stamp2` two-terminal companion stamp: the branch
/// current flowing `pin_a -> pin_b` is `g*(v_a - v_b) + ieq` — every linear
/// two-terminal contribution a dynamic partition ever stamps uses this same
/// convention, whether it is a plain conductance (`ieq = 0.0`: a resistor, a
/// capacitor's leakage, a switch contact) or a capacitor's BE/trap companion
/// (`ieq` = its `-Ihist`, `devices::passives::capacitor_be_companion`/
/// `capacitor_trap_companion`). `size` may exceed `a`/`b`'s node count (the
/// closed-form segment solve adds one extra branch-current row/column).
#[allow(clippy::too_many_arguments)]
pub(crate) fn stamp_two_terminal(size: usize, a: &mut [f64], b: &mut [f64], pin_a: &DynPin, pin_b: &DynPin, g: f64, ieq: f64, bnd: &Bnd) {
    if g == 0.0 && ieq == 0.0 {
        return;
    }
    match (bnd.local(pin_a), bnd.local(pin_b)) {
        (Some(la), Some(lb)) => {
            a[la * size + la] += g;
            a[lb * size + lb] += g;
            a[la * size + lb] -= g;
            a[lb * size + la] -= g;
            b[la] -= ieq;
            b[lb] += ieq;
        }
        (Some(la), None) => {
            a[la * size + la] += g;
            b[la] += g * bnd.get(pin_b.node) - ieq;
        }
        (None, Some(lb)) => {
            a[lb * size + lb] += g;
            b[lb] += g * bnd.get(pin_a.node) + ieq;
        }
        (None, None) => {}
    }
}

/// What the dynamic solvers need of W1f's topology at build: the rail node
/// (outside exact mode), which partitions read it, and which non-free nodes
/// each partition reads that can move after load (README, Rails points 2 and
/// 7).
#[derive(Clone, Copy)]
pub(crate) struct LiveTopology<'a> {
    pub rail: Option<usize>,
    /// The PSU's rail node in either mode (`rail` is `None` in exact mode,
    /// where the rail is a node of its own partition, not the hub's).
    pub rail_node: Option<usize>,
    pub hub: &'a [bool],
    pub reads: &'a [Vec<usize>],
    pub live: &'a [bool],
}

impl<'a> LiveTopology<'a> {
    pub(crate) fn of(pb: &'a partition::PartitionBuild) -> Self {
        LiveTopology { rail: pb.rail.filter(|t| !t.exact).map(|t| t.rail), rail_node: pb.rail.map(|t| t.rail), hub: &pb.hub, reads: &pb.reads, live: &pb.live }
    }

    /// Partition `p`'s live nodes, ascending.
    pub(crate) fn live_reads(&self, p: usize) -> Vec<usize> {
        self.reads[p].iter().copied().filter(|&n| self.live[n]).collect()
    }
}

/// `A z - b` for one stamp of size `size` at `z`.
pub(crate) fn stamp_residual(size: usize, z: &[f64], stamp: impl FnOnce(&mut [f64], &mut [f64])) -> Vec<f64> {
    let mut a = vec![0.0; size * size];
    let mut b = vec![0.0; size];
    stamp(&mut a, &mut b);
    (0..size).map(|k| a[k * size..(k + 1) * size].iter().zip(z).map(|(aij, zj)| aij * zj).sum::<f64>() - b[k]).collect()
}

/// Exact mode's hub source, stamped into the rail's own partition (README,
/// Rails point 9): a battery with `r > 0` as a Norton (`1/r` from the rail to
/// the return at 0 V, `E/r` injected), a battery with `r = 0` or a PSU in CV
/// as the rail row replaced by `u = E` or `u = V`, a PSU in CC as `dir
/// iLimit` injected into the kept KCL row. Set by the owner before every
/// call, from the hub's state at the step's start.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ExactStamp {
    pub partition: usize,
    pub local: usize,
    pub kind: ExactKind,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum ExactKind {
    Norton { g: f64, inject: f64 },
    Row { value: f64 },
    Inject { current: f64 },
}

impl ExactKind {
    /// A cell's stamp at `soc`: E and r from the model there.
    pub(crate) fn battery(m: &crate::devices::battery::BatteryModel, soc: f64) -> ExactKind {
        let (e, r) = (m.emf(soc), m.resistance(soc));
        if r > 0.0 {
            ExactKind::Norton { g: 1.0 / r, inject: e / r }
        } else {
            ExactKind::Row { value: e }
        }
    }
}

impl ExactStamp {
    /// Stamps the source into a partition's `size x size` system, after
    /// every other stamp (the row replacement must see the whole row).
    pub(crate) fn apply(&self, size: usize, a: &mut [f64], b: &mut [f64]) {
        let l = self.local;
        match self.kind {
            ExactKind::Norton { g, inject } => {
                a[l * size + l] += g;
                b[l] += inject;
            }
            ExactKind::Row { value } => {
                a[l * size..(l + 1) * size].fill(0.0);
                a[l * size + l] = 1.0;
                b[l] = value;
            }
            ExactKind::Inject { current } => b[l] += current,
        }
    }
}

/// A hub partition's rail sensitivity after a solve at rail `u_p` (README,
/// Rails point 3): its draw `i` from the rail, the slope `g = di/du`, and the
/// node sensitivities `s = dx/du` of its own unknowns.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RailSens {
    pub i: f64,
    pub g: f64,
    pub s: Vec<f64>,
    /// The residual stamp's matrix at the solve, `(sys + 1)^2` with the rail
    /// as the last unknown: the curvature band reads the partition's
    /// Jacobian from it (`curvature::hub_band`).
    pub jac: Vec<f64>,
    /// With `explicit_u` or `extra` (a partition the curvature band
    /// covers): the residual `r(x, u)` of the partition's own rows at the
    /// solve. Any distance left between the solve and the solution
    /// (rounding, once polished, `static_part::polish_hub`) makes `s`, exact
    /// at `x`, miss the solution's own slope by a first-order amount the
    /// supply bound reads from this (`curvature::supply_band`), and sets the
    /// solve's rounding floor (`curvature::Floors`). Empty otherwise.
    pub res: Vec<f64>,
    /// With `res`: each row's gross terms at the solve, `sum_j |A_kj z_j| +
    /// |b_k|` over the partition's rows and then the rail's, which a
    /// residual evaluation rounds by a few ulps of (`curvature::Floors`).
    pub gross: Vec<f64>,
    /// With `res`: each row's terms in `s`, `|F_u,k| + sum_j |J_kj s_j|`
    /// over the partition's rows and then the rail's (`F_u` the residual's
    /// rail derivative), which `s` and the draw's slope round by a few ulps
    /// of (`curvature::Floors`).
    pub gross_s: Vec<f64>,
    /// With `res`: the factors of the exact Jacobian at the solve (`jac`'s
    /// first `sys` rows and columns) that `s` was solved through, for the
    /// rounding floor and the refinement (`static_part::solve_hub`).
    pub lu: Option<crate::lu::LuFactorization>,
}

/// Three residual stamps and one re-solve (P3:437-448): `F0 = r(x, u)`,
/// `F1 = r(x, u + RAIL_DELTA)`, `s = -A^-1 (F1 - F0) / RAIL_DELTA` through the
/// partition's own factors `lu`, `I0 = I(x, u)`, `I1 = I(x + s RAIL_DELTA, u +
/// RAIL_DELTA)`, `g = (I1 - I0) / RAIL_DELTA`.
///
/// `explicit_u` (a partition the curvature band covers,
/// `curvature::hub_band`): `s` is the derivative of the solution at `x`
/// itself instead, so the published correction errs by its second-order
/// remainder alone (V5 finding 2: the last Newton iteration's Jacobian, a
/// step behind `x`, and the forward difference left `s` 4.2e-4 off, a
/// first-order error). `F0`'s own matrix, from stamps linearised about `x`,
/// is the exact Jacobian in `x` and, in its rail column, in `u` as far as
/// a stamp reaches the rail through a conductance; `explicit_u` adds the
/// rest of `dF/du` (rows `0..=sys`), the IC stamps that follow the rail
/// otherwise (`curvature::IcRail::explicit_u`). `s = -J^-1 dF/du` through
/// `J` factored afresh, and `g = dI/dx s + dI/du` from the same matrix: no
/// difference is taken, so `s` is exact to rounding. `lu` is not read.
///
/// `stamp_ext(z, a, b)` stamps the partition's `sys` unknowns plus the rail as
/// unknown `sys` (`size = sys + 1`), every nonlinear element linearised about
/// `z` itself, so `A z - b` is the true residual: rows `0..sys` are `r(x,
/// u)`, and row `sys` is the KCL of the rail node over this partition's
/// stamps alone, which is the current the partition draws from the rail.
///
/// Each difference `F(z') - F(z)` is formed as `(A' - A) z' + A (z' - z) -
/// (b' - b)`, with `z' - z` the intended step itself: a linear stamp is the
/// same at both points, so it contributes `A (z' - z)` exactly instead of
/// the difference of two residuals the size of its own current. Subtracting
/// whole residuals leaves their rounding, a part in 1e16 of each, in a step
/// of 1e-6: through a closed switch (1000 S) on a 5 V rail that is about
/// 1e-6 S of slope, a part in 1e5 of a typical rail.
///
/// `extra` (a partition holding a free IC supply, `curvature::supply_band`):
/// the rest of the exact `dF/dz` over every unknown and the rail
/// (`curvature::IcSupply::extra_jacobian`, `(sys + 1)^2`), taken the same
/// way; the returned `jac` then holds it.
#[allow(clippy::too_many_arguments)]
pub(crate) fn rail_sensitivity(sys: usize, x: &[f64], u: f64, lu: &crate::lu::LuFactorization, explicit_u: Option<&[f64]>, extra: Option<&[f64]>, mut stamp_ext: impl FnMut(&[f64], &mut [f64], &mut [f64])) -> Result<RailSens, String> {
    let size = sys + 1;
    let mut stamp_at = |z: &[f64]| -> (Vec<f64>, Vec<f64>) {
        let mut a = vec![0.0; size * size];
        let mut b = vec![0.0; size];
        stamp_ext(z, &mut a, &mut b);
        (a, b)
    };
    let difference = |(a0, b0): &(Vec<f64>, Vec<f64>), (a1, b1): &(Vec<f64>, Vec<f64>), z1: &[f64], dz: &[f64], k: usize| -> f64 {
        let row = k * size;
        let mut acc = -(b1[k] - b0[k]);
        for j in 0..size {
            let (x0, x1) = (a0[row + j], a1[row + j]);
            acc += (x1 - x0) * z1[j] + x0 * dz[j];
        }
        acc
    };
    let delta = crate::rails::RAIL_DELTA;
    let mut z0: Vec<f64> = x[..sys].to_vec();
    z0.push(u);
    let mut at0 = stamp_at(&z0);
    let i0 = at0.0[sys * size..].iter().zip(&z0).map(|(a, z)| a * z).sum::<f64>() - at0.1[sys];
    let factored = explicit_u.is_some() || extra.is_some();
    let res: Vec<f64> = if factored { (0..sys).map(|k| at0.0[k * size..(k + 1) * size].iter().zip(&z0).map(|(a, z)| a * z).sum::<f64>() - at0.1[k]).collect() } else { Vec::new() };
    let gross: Vec<f64> = if factored { (0..size).map(|k| at0.0[k * size..(k + 1) * size].iter().zip(&z0).map(|(a, z)| (a * z).abs()).sum::<f64>() + at0.1[k].abs()).collect() } else { Vec::new() };
    if let Some(extra) = extra {
        for (a, e) in at0.0.iter_mut().zip(extra) {
            *a += e;
        }
    }
    if explicit_u.is_some() || extra.is_some() {
        let fu: Vec<f64> = (0..size).map(|k| at0.0[k * size + sys] + explicit_u.map_or(0.0, |e| e[k])).collect();
        let mut jac = vec![0.0; sys * sys];
        for r in 0..sys {
            jac[r * sys..(r + 1) * sys].copy_from_slice(&at0.0[r * size..r * size + sys]);
        }
        let singular = |_| "rail sensitivity: singular or non-finite pivot".to_string();
        let mut s: Vec<f64> = fu[..sys].iter().map(|f| -f).collect();
        let lu_j = crate::lu::lu_factor(jac, sys).map_err(singular)?;
        lu_j.solve(&mut s).map_err(singular)?;
        let g = (0..sys).map(|k| at0.0[sys * size + k] * s[k]).sum::<f64>() + fu[sys];
        let gross_s: Vec<f64> = (0..size).map(|k| fu[k].abs() + (0..sys).map(|j| (at0.0[k * size + j] * s[j]).abs()).sum::<f64>()).collect();
        if !(i0.is_finite() && g.is_finite() && s.iter().all(|v| v.is_finite())) {
            return Err("rail sensitivity: a non-finite draw, slope or sensitivity".to_string());
        }
        return Ok(RailSens { i: i0, g, s, jac: at0.0, res, gross, gross_s, lu: Some(lu_j) });
    }
    let mut z1 = z0.clone();
    z1[sys] = u + delta;
    let at1 = stamp_at(&z1);
    let mut dz = vec![0.0; size];
    dz[sys] = delta;
    let mut s: Vec<f64> = (0..sys).map(|k| -difference(&at0, &at1, &z1, &dz, k) / delta).collect();
    lu.solve(&mut s).map_err(|_| "rail sensitivity: singular or non-finite pivot".to_string())?;
    let mut z2 = z1;
    for k in 0..sys {
        z2[k] = x[k] + s[k] * delta;
        dz[k] = s[k] * delta;
    }
    let at2 = stamp_at(&z2);
    let g = difference(&at0, &at2, &z2, &dz, sys) / delta;
    if !(i0.is_finite() && g.is_finite() && s.iter().all(|v| v.is_finite())) {
        return Err("rail sensitivity: a non-finite draw, slope or sensitivity".to_string());
    }
    Ok(RailSens { i: i0, g, s, jac: at0.0, res, gross, gross_s: Vec::new(), lu: None })
}

/// The current flowing `pin_a -> pin_b` through a branch stamped by
/// [`stamp_two_terminal`] with the same `(g, ieq)`, given both terminals'
/// resolved voltages.
pub(crate) fn branch_current(g: f64, ieq: f64, va: f64, vb: f64) -> f64 {
    g * (va - vb) + ieq
}

/// One capacitor terminal's contribution to an `M_SOURCE`-style branch-
/// current row/column `r` (`sign = 1.0` for `pins[0]`, `-1.0` for `pins[1]`):
/// `A[r,local] += sign; A[local,r] += sign` when free, `b[r] -= sign*fixed`
/// when fixed (pec3.mjs `:386-388`). Shared by `closed_form::build_segment`
/// (one capacitor, swept over `vc=0/1`) and `stepped::solve_initial_x`
/// (every capacitor in the partition, evaluated once at its own current
/// committed `vc`) — both replace a capacitor with an ideal source in series
/// with its `esr` to solve the REST of the network exactly, just for
/// different reasons.
pub(crate) fn stamp_branch_terminal(a: &mut [f64], b: &mut [f64], size: usize, r: usize, pin: &DynPin, sign: f64, bnd: &Bnd) {
    match bnd.local(pin) {
        Some(l) => {
            a[l * size + r] += sign;
            a[r * size + l] += sign;
        }
        None => {
            b[r] -= sign * bnd.get(pin.node);
        }
    }
}

/// A resolved [`DynPin`]'s current voltage, given the partition's own local
/// solution vector `x` (only valid for entries `0..n`).
pub(crate) fn pin_voltage(pin: &DynPin, x: &[f64], bnd: &Bnd) -> f64 {
    match bnd.local(pin) {
        Some(l) => x[l],
        None => bnd.get(pin.node),
    }
}

/// A resolved [`DynPin`]'s current voltage read from the GLOBAL, PUBLISHED
/// `voltages` array (`shape.nodes[l]` for a free pin) rather than a local
/// solution vector — used once a partition's own free nodes have already
/// been written into the shared array (`closed_form`/`stepped` publishing
/// their own capacitor's terminal voltages for its leakage/series current,
/// after already having written every node this same call).
pub(crate) fn pin_voltage_global(pin: &DynPin, voltages: &[f64]) -> f64 {
    voltages[pin.node]
}

/// Looks up a [`DynStamped`] device's CURRENT params: `live_params` (keyed by
/// device index — `PartitionedSolver`'s single canonical, mutable copy of
/// every switch-family device's live state) when present, `stamped.params`
/// (the load-time snapshot, correct forever for a resistor) otherwise.
pub(crate) fn live_params_for<'a>(stamped: &'a DynStamped, live_params: &'a HashMap<usize, Vec<f64>>) -> &'a [f64] {
    live_params.get(&stamped.device_index).map(|v| v.as_slice()).unwrap_or(&stamped.params)
}

/// Stamps every resistor, capacitor-leakage and switch-contact conductance
/// in `shape` — the partition's capacitor-companion-INDEPENDENT linear part
/// — into `a`/`b` (`size x size` / `size`, `size >= shape.n`), plus the node
/// shunt (core/README.md, "W0 kernel semantics", point 2) on every local
/// row. Shared by `closed_form`'s segment solve and `stepped`'s BE/trap
/// companion solve, which differ only in how they additionally stamp the
/// capacitor(s) themselves on top of this.
pub(crate) fn stamp_linear_base(shape: &DynShape, size: usize, a: &mut [f64], b: &mut [f64], live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd) {
    for l in 0..shape.n {
        a[l * size + l] += static_part::NODE_RSHUNT_G;
    }
    linear_elements(shape, live_params, |pa, pb, g| stamp_two_terminal(size, a, b, pa, pb, g, 0.0, bnd));
}

/// [`stamp_linear_base`]'s conductances (the node shunts aside), handed to
/// `each` as `(pin_a, pin_b, g)`: the stamp itself, or a hub refinement's
/// residual (`static_part::hub_residual_exact`), so the two walk the same
/// branches.
pub(crate) fn linear_elements(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, mut each: impl FnMut(&DynPin, &DynPin, f64)) {
    for s in &shape.stamped {
        let params = live_params_for(s, live_params);
        for c in devices::stamp(s.kind, s.pins.len(), params) {
            each(&s.pins[c.pin_a], &s.pins[c.pin_b], c.conductance);
        }
    }
    for cap in &shape.capacitors {
        if cap.leak_g != 0.0 {
            each(&cap.pins[0], &cap.pins[1], cap.leak_g);
        }
    }
}

/// Publishes every resistor's and switch-family device's OWN current
/// (core/README.md's "Currents" rules — a resistor is `(Va-Vb)/R`, a switch
/// kind is the sum of its own closed contacts' currents) for every
/// [`DynStamped`] device in `shape`, given the ALREADY-PUBLISHED global
/// `voltages` (every free node in `shape` must already be written into it —
/// both `closed_form` and `stepped` call this only after their own node-
/// voltage publish). Shared by both dynamic solvers: neither published this
/// at all before W1b's own review caught it — `PartitionedSolver`'s
/// `recompute_source_currents` only ever reads a device's current back OUT
/// (to fold into a `voltage_source`'s KCL), it never computes one for a
/// device that isn't itself a source, so a resistor/switch inside a dynamic
/// partition had no path to `bo_engine_device_currents` at all until this
/// existed.
pub(crate) fn publish_stamped_currents(shape: &DynShape, voltages: &[f64], live_params: &HashMap<usize, Vec<f64>>, currents: &mut [f64]) {
    for s in &shape.stamped {
        let params = live_params_for(s, live_params);
        let mut total = 0.0;
        for c in devices::stamp(s.kind, s.pins.len(), params) {
            let (va, vb) = (pin_voltage_global(&s.pins[c.pin_a], voltages), pin_voltage_global(&s.pins[c.pin_b], voltages));
            total += c.conductance * (va - vb);
        }
        currents[s.device_index] = total;
    }
}

// ============================================================================
// W1c step 2: shared IC-stamping helpers.
// ============================================================================
//
// Step 2a left every combinational IC confined to a `Static` partition (a
// `-3` at load otherwise — `partition.rs`'s own doc comment on the decline
// step 2b later lifts). Step 2b's own job is exactly that lift:
// `closed_form::ClosedFormSolver` and `stepped::SteppedSolver` now stamp an
// IC's driver/pull footprint too ([`stamp_ics_into_dyn`], below), whenever it
// lands in one of THEIR partitions instead of a `Static` one. Every
// per-partition stamp site — `static_part::StaticPartitionSolver`'s own LU
// stamp, `closed_form`/`stepped`'s dynamic ones, and
// `PartitionedSolver::recompute_source_currents` (the global KCL fold, needed
// even for a driver/pull pin whose OWN node happens to be fixed — contract
// point 4) — needs to answer the same two questions: "where does this
// committed driver currently stamp, and at what conductance" and "where does
// this open floating-input pin's pull currently stamp, and at what
// conductance". Defined once, here, so no module re-derives
// `digital::power`/`digital::floating`'s rules a second time.

/// Every IC in `program` — combinational, sequential or memory alike (W1d
/// widened this from combinational-only, `digital::is_ic`) — by device index
/// (`digital::IcRecord::from_device` is safe unconditionally — see
/// `partition.rs`'s identical call for why). Every driver/pull stamp site
/// below (`ic_driver_branches`, `ic_pull_target`, the watch registration and
/// bucket-watch machinery) is already generic over `IcRecord`, so this one
/// filter change is what makes a W1d IC's electrical footprint (power,
/// floating pulls, driver stamps, crossing-watch exclusion, stepped
/// classification) solver-visible; only the DEVICE EVALUATION (its truth
/// table or register/memory logic) differs, and that lives in
/// `devices::ics`, never here.
pub(crate) fn collect_ics(program: &Program) -> Vec<(usize, digital::IcRecord)> {
    program
        .devices
        .iter()
        .enumerate()
        .filter(|(_, d)| digital::is_ic(d.kind))
        .map(|(i, d)| (i, digital::IcRecord::from_device(d)))
        .collect()
}

/// Where a committed IC output driver currently stamps, and at what
/// conductance(s) (contract point 8, and NE555 point 7 for a 555's OUT/DIS
/// stage): at most two `(target, g)` branches — a plain `Logic` stage only
/// ever produces one (the pin's own node to `ic.vcc.node` at `1/R_out` while
/// committed HIGH, unless `OPEN_COLLECTOR`, which is released instead — or
/// to `ic.gnd.node` while committed LOW), while a 555's OUT stage can
/// produce two simultaneously (its complementary-conductance Thevenin split,
/// `digital::ne555::branches`). Both entries `None` while unpowered, HIZ,
/// uncommitted, a released `OPEN_COLLECTOR` HIGH, or the driver's own pin
/// sits on the datum node (finding 9; NE555 point 7 "a pin on node 0 stamps
/// nothing"): classic's `_stampDigitalOutput`/`_stampRailReferencedOutput`
/// both return immediately for a pin on the reference net (`node < 0`), and
/// node 0 is this core's own datum (core/README.md: "node 0 is the datum")
/// — the same pin-on-ground case, just numbered differently. Guarded once
/// here, in the shared helper every caller (`recompute_source_currents`,
/// `static_part::stamp_ics_into`, `stamp_ics_into_dyn`) goes through, rather
/// than at each call site, per this function's own "defined once" doc
/// comment above (the other two callers never reach node 0 today, since
/// they gate on `in_partition(driver.node)` and node 0 is never a partition
/// member — but a THIRD caller should not have to remember that
/// independently).
pub(crate) fn ic_driver_branches(power: &digital::power::IcPower, driver: &Driver, ic: &digital::IcRecord) -> [Option<(usize, f64)>; 2] {
    if !power.powered || driver.node == 0 {
        return [None, None];
    }
    match driver.stage {
        DriverStage::Logic => {
            let branch = match driver.committed() {
                Some(Mode::High) if !driver.open_collector => Some((ic.vcc.node, 1.0 / power.r_out)),
                Some(Mode::Low) => Some((ic.gnd.node, 1.0 / power.r_out)),
                _ => None,
            };
            [branch, None]
        }
        DriverStage::Ne555Out | DriverStage::Ne555Dis => digital::ne555::branches(driver.stage, driver.committed(), power, driver, ic),
    }
}

/// The conductances [`ic_driver_branches`] stamps for `driver`, slot by
/// slot, as curves in the IC's supply difference (Rails point 3).
pub(crate) fn ic_driver_curves(driver: &Driver, ic: &digital::IcRecord) -> [Option<digital::power::SupplyCurve>; 2] {
    match driver.stage {
        DriverStage::Logic => match driver.committed() {
            Some(Mode::High) if !driver.open_collector => [Some(digital::power::r_out_curve(ic)), None],
            Some(Mode::Low) => [Some(digital::power::r_out_curve(ic)), None],
            _ => [None, None],
        },
        DriverStage::Ne555Out | DriverStage::Ne555Dis => digital::ne555::branch_curves(driver.stage, driver.committed()),
    }
}

/// The stamps of [`stamp_ics_into_dyn`] in hub partition `p_idx` that move
/// with the rail (unknown `rail_local` of `bnd`'s ext) other than through
/// their conductance to it: every driver branch of an IC with a supply on
/// the rail, its conductance a curve in that supply, and every floating
/// pull whose target is the rail. The same walk as the stamp, so the two
/// see the same branches; `x` is read for nothing but the supplies, which
/// are never free here (a partition holding a free IC supply is not
/// covered by the curvature band).
#[allow(clippy::too_many_arguments)]
pub(crate) fn ic_rail_terms(now: f64, drivers: &[Driver], ics: &[(usize, digital::IcRecord)], node_owner: &HashMap<usize, (usize, usize)>, bnd: &Bnd, p_idx: usize, x: &[f64], rail_local: usize) -> curvature::IcRail {
    let mut out = curvature::IcRail::default();
    let resolve = |node: usize| -> DynPin {
        match node_owner.get(&node) {
            Some(&(p, l)) if p == p_idx => DynPin { local: Some(l), node },
            _ => DynPin { local: None, node },
        }
    };
    let in_partition = |node: usize| matches!(node_owner.get(&node), Some(&(p, _)) if p == p_idx);
    // Only a driver on one of this partition's nodes stamps a branch the
    // band reads (no supply is free here): one pass over the drivers.
    let local: Vec<&Driver> = drivers.iter().filter(|d| in_partition(d.node)).collect();
    for (_, ic) in ics {
        for pin in [&ic.vcc, &ic.gnd].into_iter().chain(&ic.logic) {
            if in_partition(pin.node) {
                out.pinned.push(node_owner[&pin.node].1);
            }
        }
    }
    for (device_index, ic) in ics {
        let touches = local.iter().any(|d| d.device == *device_index)
            || ic.logic.iter().any(|pin| pin.open && pin.floating_input && !pin.open_high && in_partition(pin.node))
            || in_partition(ic.vcc.node)
            || in_partition(ic.gnd.node);
        if !touches {
            continue;
        }
        let (vcc_pin, gnd_pin) = (resolve(ic.vcc.node), resolve(ic.gnd.node));
        let on_rail = |pin: &DynPin| bnd.local(pin) == Some(rail_local);
        let sigma = match (on_rail(&vcc_pin), on_rail(&gnd_pin)) {
            (true, false) => 1.0,
            (false, true) => -1.0,
            _ => {
                // A chip off the rail that stamps here stamps toward its own
                // supplies: somewhere for the draw to go.
                out.sinks = true;
                continue;
            }
        };
        // Judged as the stamp judges it (`Bnd::hold_power_at`): the terms
        // are the derivative within the present power state.
        let mut power = digital::power::ic_power(ic, pin_voltage(&vcc_pin, x, bnd), pin_voltage(&gnd_pin, x, bnd));
        power.powered = digital::power::verdict(ic, bnd.power_voltage(&vcc_pin, x), bnd.power_voltage(&gnd_pin, x), bnd.hold(*device_index));
        if !power.powered {
            continue;
        }
        for driver in local.iter().filter(|d| d.device == *device_index) {
            let curves = ic_driver_curves(driver, ic);
            for (branch, curve) in ic_driver_branches(&power, driver, ic).into_iter().zip(curves) {
                let Some((target, _)) = branch else { continue };
                let far = resolve(target);
                let to_rail = on_rail(&far);
                out.touches_rail |= to_rail;
                out.sinks |= !to_rail;
                let Some(curve) = curve else { continue };
                let o = node_owner[&driver.node].1;
                out.drivers.push(curvature::RailDriver { o, to_rail, t: pin_voltage(&far, x, bnd), sigma, vs: power.vs, curve, window: digital::levels::power_window(ic) });
            }
        }
        for pin in &ic.logic {
            if !in_partition(pin.node) {
                continue;
            }
            if let Some((g, _)) = ic_pull_target(ic, &power, pin, now) {
                // The target is `gnd + vs` (the vcc pin) when the pull is
                // high, `gnd` when low: it follows the rail when that pin is
                // the rail.
                let high = digital::floating::floating_high(pin.open_hash, now, ic.ttl);
                let follows = if high { sigma > 0.0 } else { sigma < 0.0 };
                if follows {
                    out.pulls.push((node_owner[&pin.node].1, g));
                    out.touches_rail = true;
                } else {
                    out.sinks = true;
                }
            }
        }
    }
    out
}

/// The stamps of [`stamp_ics_into_dyn`] in a hub partition `p_idx` holding a
/// free IC supply that are not a fixed conductance (`curvature::IcSupply`),
/// the rail stamped as unknown `rail_local` of `bnd`'s ext: every driver
/// branch whose IC's supply is not fixed, its conductance a curve in that
/// supply, and every floating pull whose target, its IC's `vcc` or `gnd`, is
/// not. The same walk as the stamp, with power judged as the stamp judges
/// it.
#[allow(clippy::too_many_arguments)]
pub(crate) fn ic_supply_terms(now: f64, drivers: &[Driver], ics: &[(usize, digital::IcRecord)], node_owner: &HashMap<usize, (usize, usize)>, bnd: &Bnd, p_idx: usize, x: &[f64]) -> curvature::IcSupply {
    let mut out = curvature::IcSupply::default();
    let resolve = |node: usize| -> DynPin {
        match node_owner.get(&node) {
            Some(&(p, l)) if p == p_idx => DynPin { local: Some(l), node },
            _ => DynPin { local: None, node },
        }
    };
    let in_partition = |node: usize| matches!(node_owner.get(&node), Some(&(p, _)) if p == p_idx);
    let end = |pin: &DynPin| match bnd.local(pin) {
        Some(l) => curvature::End::Z(l),
        None => curvature::End::Fixed(bnd.get(pin.node)),
    };
    for (device_index, ic) in ics {
        let touches = drivers.iter().any(|d| d.device == *device_index && in_partition(d.node))
            || ic.logic.iter().any(|pin| pin.open && pin.floating_input && !pin.open_high && in_partition(pin.node))
            || in_partition(ic.vcc.node)
            || in_partition(ic.gnd.node);
        if !touches {
            continue;
        }
        let (vcc_pin, gnd_pin) = (resolve(ic.vcc.node), resolve(ic.gnd.node));
        let (vcc, gnd) = (end(&vcc_pin), end(&gnd_pin));
        let mut power = digital::power::ic_power(ic, pin_voltage(&vcc_pin, x, bnd), pin_voltage(&gnd_pin, x, bnd));
        power.powered = digital::power::verdict(ic, bnd.power_voltage(&vcc_pin, x), bnd.power_voltage(&gnd_pin, x), bnd.hold(*device_index));
        if !power.powered {
            continue;
        }
        let supply_fixed = matches!((vcc, gnd), (curvature::End::Fixed(_), curvature::End::Fixed(_)));
        for driver in drivers.iter().filter(|d| d.device == *device_index) {
            let curves = ic_driver_curves(driver, ic);
            for (branch, curve) in ic_driver_branches(&power, driver, ic).into_iter().zip(curves) {
                let (Some((target, _)), Some(curve)) = (branch, curve) else { continue };
                if supply_fixed || !(in_partition(driver.node) || in_partition(target)) {
                    continue;
                }
                out.drivers.push(curvature::SupplyDriver { a: end(&resolve(driver.node)), f: end(&resolve(target)), vcc, gnd, curve });
            }
        }
        for pin in &ic.logic {
            if !in_partition(pin.node) || ic_pull_target(ic, &power, pin, now).is_none() {
                continue;
            }
            let high = digital::floating::floating_high(pin.open_hash, now, ic.ttl);
            let target = if high { vcc } else { gnd };
            if let curvature::End::Z(_) = target {
                out.pulls.push(curvature::SupplyPull { pin: node_owner[&pin.node].1, target, g: ic_pull_target(ic, &power, pin, now).unwrap().0 });
            }
        }
    }
    out
}

/// Where an open `FLOATING_INPUT` pin's floating pull currently stamps, and
/// at what conductance (contract point 6): exactly `digital::floating::pull`'s
/// own `(conductance, absolute target volts)` Norton branch — `g` from the
/// pin's node to the DATUM (node 0), plus the injection `g * target_v` into
/// the pin's own row, never a conductance to `ic.gnd.node`/`ic.vcc.node`
/// directly.
///
/// This module used to fold the pull as `g` from the pin to whichever supply
/// node `target_v` numerically equals (`ic.vcc.node` while HIGH, `ic.gnd.node`
/// while LOW) on the argument that node voltages cannot tell the two stamps
/// apart — true, but SOURCE CURRENTS can: routing the branch through a real
/// supply node makes its return current fold into that node's own KCL, so
/// `recompute_source_currents`'s walk attributes a phantom few microamps of
/// it to whatever voltage source ultimately fixed that rail — current a
/// pull's own physical resistor-to-a-Norton-equivalent never draws from any
/// real source, since it is not actually wired to the supply rail at all
/// (classic's `_stampFloatingDigitalInputs` never touches the supply source
/// either: `mna.add(node, node, g); mna.addB(node, target * g)` is a shunt
/// to the reference net, not a branch to `power.vcc`/`power.gnd`'s own
/// component). Stamping to the datum instead makes every call site match
/// classic's current bookkeeping exactly, not merely its node voltages.
///
/// `None` when `digital::floating::pull` itself declines (unpowered, not
/// open, not `FLOATING_INPUT`, or `OPEN_HIGH`).
pub(crate) fn ic_pull_target(ic: &digital::IcRecord, power: &digital::power::IcPower, pin: &digital::IcPin, t: f64) -> Option<(f64, f64)> {
    digital::floating::pull(ic, power, pin, t)
}

/// Whether ANY powered IC in `ics` currently holds an open `FLOATING_INPUT`
/// pin, or reads a non-Schmitt pin in its dead band (contract point 5/6):
/// the condition that makes the solver schedule (or keep scheduled) a
/// floating-bucket boundary. Both categories are exactly the pins whose read
/// consults `digital::floating::floating_high` at all — a Schmitt pin's own
/// "in between" case holds `*schmitt_state` instead (`digital::levels::read`'s
/// own doc comment), never touching the floating model, so it never needs a
/// bucket watch.
pub(crate) fn any_powered_ic_needs_bucket_watch(ics: &[(usize, digital::IcRecord)], voltages: &[f64], holds: &[digital::power::PowerHold]) -> bool {
    for (device_index, ic) in ics {
        // C8: a 555 takes no part in the floating model at all (it holds no
        // `FLOATING_INPUT` pin and reads nothing through it), so it must
        // never be considered here — its own TRIG/THR sit inside the
        // generic dead band for most of an astable's period, and without
        // this exclusion that would schedule a pointless bucket every 10 ms
        // forever.
        if digital::is_ne555(ic.kind) {
            continue;
        }
        let power = digital::power::ic_power_held(ic, voltages[ic.vcc.node], voltages[ic.gnd.node], digital::power::hold_of(holds, *device_index));
        if !power.powered {
            continue;
        }
        for pin in &ic.logic {
            if pin.open && pin.floating_input && !pin.open_high {
                return true;
            }
            if !pin.open && power.schmitt.is_none() {
                let v = voltages[pin.node] - power.v_gnd;
                if v > power.vil && v < power.vih {
                    return true;
                }
            }
        }
    }
    false
}

/// The node -> IC-reader index (`static_part.rs`'s own `node_to_ic_readers`,
/// mirrored here for `closed_form`/`stepped`, W1c step 2b's own contract
/// point 4 addition: "resolves of dynamic partitions must also report IC
/// readers of nodes that changed, as static does"): every combinational IC's
/// `vcc`, `gnd` and every logic pin, mapped to its own device index.
/// Program-wide, not filtered to any one partition — a node a given dynamic
/// solver's own partitions never touch simply never gets consulted, the same
/// harmless unused entry static's own copy already accepts.
pub(crate) fn build_ic_reader_index(ics: &[(usize, digital::IcRecord)]) -> HashMap<usize, Vec<usize>> {
    let mut index: HashMap<usize, Vec<usize>> = HashMap::new();
    for (device_index, ic) in ics {
        for &node in &[ic.vcc.node, ic.gnd.node] {
            index.entry(node).or_default().push(*device_index);
        }
        for pin in &ic.logic {
            index.entry(pin.node).or_default().push(*device_index);
        }
    }
    index
}

/// Stamps every combinational IC's driver/pull contribution that touches
/// dynamic partition `p_idx` into its local `size x size` linear system —
/// the `ClosedForm`/`Stepped` counterpart of `static_part.rs`'s own
/// `stamp_ics_into` (W1c step 2b, lifting `partition.rs`'s former decline):
/// generalized over `resolve`, a [`DynPin`]-producing closure, rather than
/// that module's own `fold_local`, since a dynamic partition's stamp target
/// goes through [`stamp_two_terminal`], not a direct matrix index. `touches`
/// mirrors `static_part.rs::stamp_ics_into`'s own per-item test exactly,
/// deliberately NOT assuming an IC's whole driver/pull footprint shares one
/// partition: that is only guaranteed when at least one of its supply nodes
/// is free (`partition.rs`'s union rule then joins every free driver/pull pin
/// with it); when both are fixed, each driver/pull pin is placed purely by
/// its own, independent wiring and can land in a different partition from
/// its siblings (README's W1c contract point 2: "no IC pin joins anything").
///
/// `x` is the CURRENT local solution vector — only ever non-empty (and only
/// ever actually indexed by [`pin_voltage`]) for a `Stepped` partition mid
/// Newton iteration on a free IC supply node (contract point 3). Every IC
/// that ever touches a `ClosedForm` partition is guaranteed, by
/// `partition::build`'s own promotion rule (a `ClosedForm` partition holding
/// a free IC supply is reclassified `Stepped` outright), to have BOTH supply
/// nodes fixed, so `resolve` never returns `local: Some(_)` for either one
/// there and `closed_form.rs` always passes `&[]`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn stamp_ics_into_dyn(
    size: usize,
    a: &mut [f64],
    b: &mut [f64],
    now: f64,
    drivers: &[Driver],
    ics: &[(usize, digital::IcRecord)],
    node_owner: &HashMap<usize, (usize, usize)>,
    bnd: &Bnd,
    p_idx: usize,
    x: &[f64],
) {
    ic_elements(now, drivers, ics, node_owner, bnd, p_idx, x, |pa, pb, g, ieq| stamp_two_terminal(size, a, b, pa, pb, g, ieq, bnd));
}

/// [`stamp_ics_into_dyn`]'s walk, handing each two-terminal branch it
/// stamps (`pin_a`, `pin_b`, `g`, `ieq`, [`stamp_two_terminal`]'s
/// convention) to `each`: the stamp itself, or a hub refinement's residual
/// (`static_part::hub_residual_exact`), which sums the same branches'
/// currents element by element so the two can never disagree on what an IC
/// stamps.
#[allow(clippy::too_many_arguments)]
pub(crate) fn ic_elements(now: f64, drivers: &[Driver], ics: &[(usize, digital::IcRecord)], node_owner: &HashMap<usize, (usize, usize)>, bnd: &Bnd, p_idx: usize, x: &[f64], mut each: impl FnMut(&DynPin, &DynPin, f64, f64)) {
    // No IC, nothing to stamp (the loop below would not run).
    if ics.is_empty() {
        return;
    }
    // A node that is not free in `p_idx` is read through the live boundary:
    // partition.rs's union rule guarantees it is not free in another
    // partition.
    let resolve = |node: usize| -> DynPin {
        match node_owner.get(&node) {
            Some(&(p, l)) if p == p_idx => DynPin { local: Some(l), node },
            _ => DynPin { local: None, node },
        }
    };
    // `in_partition(node)` is `node_owner[node].0 == p_idx`. The `touches`
    // test below asks it about every driver, floating pin and supply node of
    // every IC, and a SipHash lookup for each dominated this function. The
    // owned nodes are gathered once instead (one pass over `node_owner`,
    // which costs about as much as a lookup per eight drivers, hence the
    // bound); the answers are the same either way. The two scratch vectors
    // live between calls so the function allocates nothing.
    let (mut owned, mut driver_touched) = STAMP_ICS_SCRATCH.with(|c| c.take());
    owned.clear();
    driver_touched.clear();
    let by_list = node_owner.len() <= 4 * (drivers.len() + 4);
    if by_list {
        owned.extend(node_owner.iter().filter(|(_, &(p, _))| p == p_idx).map(|(&n, _)| n));
        owned.sort_unstable();
    }
    let in_partition = |node: usize| {
        if by_list {
            owned.binary_search(&node).is_ok()
        } else {
            matches!(node_owner.get(&node), Some(&(p, _)) if p == p_idx)
        }
    };

    // Which ICs have a driver on one of this partition's nodes (their
    // positions in `ics`, sorted), in one pass over the drivers rather than
    // one pass per IC. Valid when `ics` is strictly ascending by device
    // index (as `collect_ics` builds it), so a driver's IC is found by
    // binary search; any other order falls back to the per-IC scan below.
    let ics_sorted = ics.windows(2).all(|w| w[0].0 < w[1].0);
    if ics_sorted {
        for d in drivers {
            if in_partition(d.node) {
                if let Ok(pos) = ics.binary_search_by_key(&d.device, |(device_index, _)| *device_index) {
                    driver_touched.push(pos);
                }
            }
        }
        driver_touched.sort_unstable();
    }

    for (pos, (device_index, ic)) in ics.iter().enumerate() {
        // `touches` also fires when this IC's own free supply node lives in
        // `p_idx`, even if every one of its driver/pull pins is fixed
        // elsewhere (findings 3/8's dynamic-partition twin): an output pin
        // wired straight onto a fixed/ideal node still loads its IC's free
        // `vcc`/`gnd` through `1/R_out`, and that belongs in whichever
        // partition holds the free end.
        let by_driver = if ics_sorted { driver_touched.binary_search(&pos).is_ok() } else { drivers.iter().any(|d| d.device == *device_index && in_partition(d.node)) };
        let touches = by_driver
            || ic.logic.iter().any(|pin| pin.open && pin.floating_input && !pin.open_high && in_partition(pin.node))
            || in_partition(ic.vcc.node)
            || in_partition(ic.gnd.node);
        if !touches {
            continue;
        }
        let vcc_pin = resolve(ic.vcc.node);
        let gnd_pin = resolve(ic.gnd.node);
        let mut power = digital::power::ic_power(ic, pin_voltage(&vcc_pin, x, bnd), pin_voltage(&gnd_pin, x, bnd));
        power.powered = digital::power::verdict(ic, bnd.power_voltage(&vcc_pin, x), bnd.power_voltage(&gnd_pin, x), bnd.hold(*device_index));
        if !power.powered {
            continue;
        }
        for driver in drivers {
            if driver.device != *device_index {
                continue;
            }
            for branch in ic_driver_branches(&power, driver, ic) {
                let Some((target, g)) = branch else { continue };
                // Findings 3/8: stamp whenever EITHER end is free-in-`p_idx`
                // (the driver's own pin, or its commit target), not only the
                // driver's own pin as before — that was the whole bug, a
                // fixed-pin output never loaded its IC's own free supply.
                // Unlike `static_part.rs`'s `fold_local` (which silently
                // no-ops when neither end is free-in-`p_idx`), `resolve`
                // above PANICS on a node that is free but NOT fixed and NOT
                // in `p_idx` (the "one supply free, the other pin placed
                // entirely independently" case this function's own doc
                // comment describes), so this guard must run BEFORE
                // `resolve` is ever called on either node, not merely guard
                // the stamp itself. This is still no narrower than
                // necessary: when `target` is free, the union rule
                // guarantees `driver.node` is fixed or ALSO free-in-`p_idx`
                // (never free in some OTHER partition), so `resolve` never
                // panics for a case this guard lets through. Each of a 555
                // OUT's up-to-two branches is folded independently (W1a
                // point 2's "each caller folds every branch").
                if in_partition(driver.node) || in_partition(target) {
                    each(&resolve(driver.node), &resolve(target), g, 0.0);
                }
            }
        }
        for pin in &ic.logic {
            if !in_partition(pin.node) {
                continue;
            }
            if let Some((g, target_v)) = ic_pull_target(ic, &power, pin, now) {
                // The pull's Norton branch runs to the DATUM (node 0, always
                // fixed at 0.0 — never a partition member, so this is never
                // `resolve(0)`'s free arm), not to a supply node (the bug
                // `ic_pull_target`'s own doc comment explains). Folded through
                // `stamp_two_terminal`'s general `g*(va-vb)+ieq` convention:
                // with `vb` pinned at 0, `ieq = -g*target_v` makes the branch
                // current into the pin exactly `g*V(pin) - g*target_v`,
                // classic's own `mna.add(node,node,g); mna.addB(node,
                // target*g)` (`_stampFloatingDigitalInputs`) restated as a
                // two-terminal stamp instead of a direct matrix/RHS poke.
                let datum = DynPin { local: None, node: 0 };
                each(&resolve(pin.node), &datum, g, -g * target_v);
            }
        }
    }
    STAMP_ICS_SCRATCH.with(|c| c.replace((owned, driver_touched)));
}

thread_local! {
    /// `stamp_ics_into_dyn`'s working vectors (the partition's node list and
    /// the positions of the ICs with a driver in it), kept between calls.
    static STAMP_ICS_SCRATCH: std::cell::RefCell<(Vec<usize>, Vec<usize>)> = const { std::cell::RefCell::new((Vec::new(), Vec::new())) };
}

// ============================================================================
// PartitionedSolver: the composite W1b step 2 solver.
// ============================================================================
//
// One `PartitionSolver` covering every partition kind `partition::build` can
// produce: a `Static` partition's electrical solve stays entirely
// `static_part::StaticPartitionSolver`'s (untouched by this module beyond
// which devices it is even handed, `static_part.rs`'s own module doc), a
// `ClosedForm` one is `closed_form::ClosedFormSolver`'s, a `Stepped` one is
// `stepped::SteppedSolver`'s. This struct's own job is purely dispatch and
// the pieces that genuinely cross partition boundaries:
// - the SHARED `voltages`/`currents` arrays every `bo_engine_*` readout
//   reads (`closed_form`/`stepped` write directly into them; a `Static`
//   resolve's results are copied in from `StaticPartitionSolver`'s own,
//   separately-owned arrays afterward, [`Self::copy_in_static`]);
// - a `voltage_source`'s published current, which needs the FULL, global
//   picture of every OTHER device's current at its fixed neighbour nodes
//   ([`Self::recompute_source_currents`]) — `StaticPartitionSolver`'s own
//   version of this same KCL walk only ever saw its own (now filtered-down)
//   resistor/switch list, so it is authoritative only for a purely static
//   program; this solver's version is authoritative for every program;
// - a switch-family device's live parameters ([`Self::live_params`]), a
//   SINGLE canonical, mutable copy this solver owns outright — not
//   duplicated per sub-solver — that `closed_form`/`stepped` are simply
//   handed by reference at stamp time, and `StaticPartitionSolver`'s OWN
//   separate copy (for a `Static`-owned switch) is kept in sync by
//   [`PartitionSolver::apply_params`]'s dispatcher;
// - a capacitor across two FIXED, distinct nodes (core/README.md's rule 2a)
//   — `partition::build` excludes it from every partition (it belongs to
//   none), so it is never handed to `closed_form`/`stepped` either;
//   [`FixedPairCapacitor`]/[`Self::publish_fixed_pair_capacitors`] are its
//   only home.

/// Which sub-solver owns a device (by its free pin's partition, or
/// `Static` for a device with no free pin at all — the same "belongs to no
/// dynamic partition" rule `static_part.rs`'s `owned_by_static` applies).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Owner {
    Static,
    ClosedForm(usize),
    Stepped(usize),
}

/// A capacitor across two FIXED, distinct nodes (core/README.md's rule 2a —
/// the ordinary decoupling capacitor straight across an ideal supply).
/// Belongs to no partition at all (`partition::build` excludes it from every
/// `capacitor_count`), so `compile_dyn_shape` never touches it either: no
/// free pin means it never "touches" any partition by that function's own
/// `touches_partition` test. Unlike a `ClosedForm` partition's `Segment`,
/// this device needs no re-aiming, no LU solve, and no watched-crossing
/// machinery at all — its terminal voltage `dV = V(pins[0]) - V(pins[1])` is
/// fixed FOREVER (a `voltage_source` has no live params, `kinds.rs`'s
/// `live_params` table), so its whole trajectory is one pure, closed-form
/// function of absolute time; `current_at` recomputes it fresh on every
/// call rather than carrying any committed `vc`/anchor state of its own.
/// That also means nothing here needs its own entry in
/// [`PartitionSolver::dynamic_snapshot`]/`dynamic_restore`: its only mutable
/// trace is the published `currents[device_index]` entry, already covered by
/// the plain `voltages`/`currents` snapshot every `PartitionSolver` provides
/// (contract point 7 — "rolls back with the instant").
#[derive(Debug, Clone, Copy)]
struct FixedPairCapacitor {
    device_index: usize,
    /// `(pins[0], pins[1])`'s nodes.
    nodes: (usize, usize),
    /// `V(pins[0]) - V(pins[1])`: constant on constant nodes, re-aimed at
    /// each change of a live one (README, Rails point 7).
    d_v: f64,
    c: f64,
    esr: f64,
    /// `1/leakage`, or `0.0` for "none" — same convention as `DynCapacitor::leak_g`.
    leak_g: f64,
    /// The charge's anchor: `vc0` at `t0` (0 at `START_TIME` until a live
    /// node moves).
    vc0: f64,
    t0: f64,
}

impl FixedPairCapacitor {
    /// Rule 2a's exact trajectory: from `vc = 0` at `START_TIME`, `vc(t) =
    /// dV + (0 - dV) * exp(-(t - START_TIME) / (esr*C))` for `esr > 0`, so
    /// the series current `(dV - vc)/esr` reduces to `(dV/esr) *
    /// exp(-(t-START_TIME)/(esr*C))` — computed directly in that form
    /// (rather than via an intermediate `vc`) to avoid the catastrophic
    /// cancellation `dV - vc` would suffer once `vc` has relaxed close to
    /// `dV`. `esr == 0`: `vc = dV` from the load instant on, so the series
    /// current is exactly `0` (the contract's own words — not `dV*exp(..)/0`,
    /// which would be `NaN`/`inf`). Either way, the leakage term `dV/leakage`
    /// (`0` when there is none) is constant for as long as `dV` is,
    /// i.e. forever. Current flows `pins[0] -> pins[1]`, the same convention
    /// `DynCapacitor`'s published current and `PartitionedSolver::cap_devices`'s
    /// KCL folding already use.
    fn current_at(&self, now: f64) -> f64 {
        // With the load's anchor (`vc0 = 0`, `t0 = START_TIME`) this is the
        // pre-W1f expression bit for bit: `d_v - 0.0` is `d_v`.
        let series = if self.esr > 0.0 { ((self.d_v - self.vc0) / self.esr) * (-(now - self.t0) / (self.esr * self.c)).exp() } else { 0.0 };
        series + self.leak_g * self.d_v
    }

    /// `vc` at `now` (`dV` at once when `esr = 0`).
    fn vc_at(&self, now: f64) -> f64 {
        if self.esr > 0.0 {
            self.d_v + (self.vc0 - self.d_v) * (-(now - self.t0) / (self.esr * self.c)).exp()
        } else {
            self.d_v
        }
    }

    /// The charge a bench PSU's CV -> CC test reads (Rails point 5): `vc` for
    /// `esr > 0`. At `esr = 0` it is the anchor, the charge the capacitor
    /// was loaded with (W1b point 5) until a live node first moves, and from
    /// then on `dV`, which rule 2a jumps it to at once ([`Self::reaim`]): so
    /// in CV only a capacitor discharged at load is short of the setpoint.
    fn charge(&self, now: f64) -> f64 {
        if self.esr > 0.0 {
            self.vc_at(now)
        } else {
            self.vc0
        }
    }

    /// A live node moved (Rails point 7): `vc` is continuous and the new `dV`
    /// is its target. At `esr = 0` the anchor records the jump to it, which
    /// nothing else reads (the series current and `vc` there are `0` and
    /// `dV` whatever the anchor), so that [`Self::charge`] holds it.
    fn reaim(&mut self, now: f64, d_v: f64) {
        if d_v.to_bits() == self.d_v.to_bits() {
            return;
        }
        self.vc0 = if self.esr > 0.0 { self.vc_at(now) } else { d_v };
        self.t0 = now;
        self.d_v = d_v;
    }

    /// Takes `vc` as its charge at `now`, heading for `dV`: where a PSU
    /// leaving CC carried it (Rails point 4).
    fn hold_at(&mut self, now: f64, d_v: f64, vc: f64) {
        self.vc0 = vc;
        self.t0 = now;
        self.d_v = d_v;
    }
}

/// Limits under which the readout passes are deferred to the first read
/// ([`PartitionedSolver::lazy_publish_ok`]). A node voltage above
/// `LAZY_V_LIMIT`, a coefficient above `LAZY_G_LIMIT` or a current above
/// `LAZY_I_LIMIT` selects the eager passes, so they only need to keep every
/// product and sum of at most `LAZY_TERM_LIMIT` terms inside the f64 range:
/// `1e6 * (1e12 * 2e9)` and `1e6 * 1e100` both are, by a wide margin.
const LAZY_V_LIMIT: f64 = 1e9;
const LAZY_G_LIMIT: f64 = 1e12;
const LAZY_I_LIMIT: f64 = 1e100;
const LAZY_TERM_LIMIT: usize = 1_000_000;

/// Whether `law.current(v)` and the KCL's `current(v) + GMIN_D * v` are at
/// most [`LAZY_I_LIMIT`] for every `|v| <= 2 * LAZY_V_LIMIT`. Below `vsat` the
/// exponential is at most `exp(vsat / vtn)`; above it the tangent adds
/// `is * exp(vsat / vtn) / vtn` per volt. A non-positive or non-finite `vtn`
/// (the exponent could then grow without limit) or a bound that overflows
/// fails, and `!(x <= LIMIT)` also fails a NaN.
fn junction_current_bounded(law: &led::JunctionLaw) -> bool {
    if !(law.vtn > 0.0 && law.vtn.is_finite() && law.is.is_finite() && law.vsat.is_finite()) {
        return false;
    }
    let e_sat = (law.vsat / law.vtn).exp();
    let span = 2.0 * LAZY_V_LIMIT + law.vsat.abs();
    let bound = law.is.abs() * (e_sat + 1.0) + (law.is.abs() * e_sat / law.vtn) * span + led::GMIN_D * 2.0 * LAZY_V_LIMIT;
    bound <= LAZY_I_LIMIT
}

/// A resistor or switch-family device, program-wide, kept for the
/// source-current KCL walk ([`PartitionedSolver::recompute_source_currents`])
/// and for copying a `Static`-owned one's current out of
/// `StaticPartitionSolver` after it resolves.
#[derive(Clone)]
struct KclDevice {
    device_index: usize,
    kind: u32,
    nodes: Vec<usize>,
    #[cfg_attr(not(any(test, debug_assertions)), allow(dead_code, reason = "read by the debug-build check and the tests"))]
    owner: Owner,
    /// `devices::stamp(kind, nodes.len(), <its current params>)` resolved to
    /// `(node, node, conductance)`, kept so the KCL walk allocates nothing
    /// and looks no params up. A resistor's is fixed at load; a switch's is
    /// rebuilt by [`PartitionedSolver::refresh_kcl_contributions`] whenever
    /// `apply_params` writes one of its live params (the only writer).
    contributions: Vec<(usize, usize, f64)>,
}

impl KclDevice {
    fn resolve_contributions(kind: u32, nodes: &[usize], params: &[f64]) -> Vec<(usize, usize, f64)> {
        devices::stamp(kind, nodes.len(), params).into_iter().map(|c| (nodes[c.pin_a], nodes[c.pin_b], c.conductance)).collect()
    }
}

/// One LED-family device's readout inputs (README, LEDs point 13): what
/// `PartitionedSolver::publish_led_devices` needs to compute its `elementI`
/// and channel values from the published voltages. The device is not one
/// partition's stamp (its junctions can sit in several, or across two fixed
/// nodes), so this pass is solver-wide, like `recompute_source_currents`.
#[derive(Clone)]
struct LedDevice {
    device_index: usize,
    /// A copy of the record: `led::readout` reads its kind and `burnt` flag.
    device: crate::program::Device,
    /// Its junctions, stamped or not, in channel order.
    junctions: Vec<led::Junction>,
    /// The first of its channel slots and how many it owns (0 for `led`).
    channel_base: usize,
    channel_count: usize,
}

impl LedDevice {
    /// Whether its `getElementChannelI` entry is absent: `burnt` in W1e
    /// (`seg7` has no such flag, Q-O22), and `led_failed` from W1g.
    fn absent(&self) -> bool {
        let p = &self.device.params;
        match self.device.kind {
            kinds::LED => p[kinds::led_param::BURNT] != 0.0,
            kinds::RGB_LED => p[kinds::rgb_param::BURNT] != 0.0,
            kinds::BICOLOR_LED => p[kinds::bicolor_param::BURNT] != 0.0,
            _ => false,
        }
    }
}

/// A switch-family device's dispatch metadata for
/// `PartitionSolver::validate_params`/`apply_params`/`param_value`.
#[derive(Clone)]
struct SwitchInfo {
    kind: u32,
    pin_count: usize,
    owner: Owner,
}

/// Raises `bound` to cover `|x|`; a NaN, which no comparison orders, raises
/// it to infinity, so a bound never hides one.
fn fold_bound(bound: &mut f64, x: f64) {
    let a = x.abs();
    if !(a <= *bound) {
        *bound = if a.is_nan() { f64::INFINITY } else { a };
    }
}

/// Whether two driver entries are the same in every field the solver reads (a
/// debug build's check that the mirror agrees with the kernel's table: the
/// pending transition and the token, which only a request changes, are not
/// among them), and the tests' check of the same in either build.
#[cfg(any(debug_assertions, test))]
fn same_driver(a: &Driver, b: &Driver) -> bool {
    a.node == b.node
        && a.hi_node == b.hi_node
        && a.lo_node == b.lo_node
        && a.conductance.to_bits() == b.conductance.to_bits()
        && a.hi_level.to_bits() == b.hi_level.to_bits()
        && a.lo_level.to_bits() == b.lo_level.to_bits()
        && a.device == b.device
        && a.open_collector == b.open_collector
        && a.stage == b.stage
        && a.committed() == b.committed()
}

/// See [`PartitionedSolver::watch_dependents`].
fn build_watch_dependents(ics: &[(usize, digital::IcRecord)], local_index: &[Option<(usize, usize)>], kinds: &[PartitionKind]) -> Vec<(u32, Vec<(bool, usize)>)> {
    let mut out = Vec::new();
    for (i, (_, ic)) in ics.iter().enumerate() {
        let supply_moves = [ic.vcc.node, ic.gnd.node]
            .iter()
            .any(|&n| matches!(local_index[n], Some((p, _)) if matches!(kinds[p], PartitionKind::Static | PartitionKind::Stepped)));
        if !supply_moves {
            continue;
        }
        let mut deps = Vec::new();
        for (logic_index, pin) in ic.logic.iter().enumerate() {
            if !partition::is_watched_read_pin(ic, logic_index) {
                continue;
            }
            if let Some((p, _)) = local_index[pin.node] {
                match kinds[p] {
                    PartitionKind::ClosedForm => deps.push((true, p)),
                    PartitionKind::Stepped => deps.push((false, p)),
                    PartitionKind::Static => {}
                }
            }
        }
        if !deps.is_empty() {
            out.push((i as u32, deps));
        }
    }
    out
}

/// What a dynamic (closed-form or stepped) partition publishes: the nodes it
/// writes into `voltages`, and the devices it writes into `currents` (its
/// capacitors' and its stamped resistors' and switches'), the capacitors also
/// listed on their own for the readout's magnitude bound.
#[derive(Clone, Debug, Default)]
struct DynOut {
    nodes: Vec<usize>,
    devices: Vec<usize>,
    caps: Vec<usize>,
}

impl DynOut {
    fn of(shape: &DynShape) -> Self {
        let caps: Vec<usize> = shape.capacitors.iter().map(|c| c.device_index).collect();
        let mut devices = caps.clone();
        devices.extend(shape.stamped.iter().map(|s| s.device_index));
        DynOut { nodes: shape.nodes.clone(), devices, caps }
    }
}

/// The checkpoint [`PartitionedSolver::begin_instant`] hands the kernel. The
/// solver keeps what a rollback needs itself (undo logs and journals, filled
/// as the instant writes), so the token carries nothing, and being zero-sized
/// it costs no allocation.
struct WindowToken;

/// [`PartitionedSolver::holds_flag`]'s cell. The kernel holds the other
/// reference to it, so a clone must not share it: the copy gets a cell of
/// its own with the same value, and `Kernel`'s `Clone` takes that one.
#[derive(Debug)]
struct HoldsFlag(Rc<std::cell::Cell<bool>>);


impl crate::snapshot::HeapBytes for HoldsFlag {
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        crate::snapshot::HeapBytes::heap_bytes(&self.0, w)
    }
}

impl Clone for HoldsFlag {
    fn clone(&self) -> Self {
        HoldsFlag(Rc::new(std::cell::Cell::new(self.0.get())))
    }
}

/// The composite solver `engine.rs`'s `Engine::load` builds and boxes as
/// `Box<dyn PartitionSolver>` (`solve::PartitionedSolver::build`).
#[derive(Clone)]
pub struct PartitionedSolver {
    voltages: Vec<f64>,
    /// The published device currents. The slots the readout pass derives
    /// (source, 555, LED and fixed-pair capacitor currents) may be stale
    /// until [`PartitionSolver::prepare_readout`] runs the pass; reading
    /// `currents()` while that is owed is a bug (`debug_assert`).
    currents: Vec<f64>,
    resolved: crate::shared::Shared<[partition::ResolvedSource]>,
    local_index: crate::shared::Shared<[Option<(usize, usize)>]>,
    partition_kinds: crate::shared::Shared<[PartitionKind]>,
    /// Every node whose owning partition is `Static`: the nodes
    /// [`Self::copy_in_static`] keeps in step with `StaticPartitionSolver`'s
    /// own array (it copies the ones that solver changed; a debug build checks
    /// all of these on every call. Never a dynamic node, which
    /// `StaticPartitionSolver` never writes at all after `static_part.rs`'s
    /// W1b step 2 device filter).
    #[cfg_attr(not(any(test, debug_assertions)), allow(dead_code, reason = "read by the debug-build check and the tests"))]
    static_nodes: crate::shared::Shared<[usize]>,
    static_solver: static_part::StaticPartitionSolver,
    closed_form: closed_form::ClosedFormSolver,
    stepped: stepped::SteppedSolver,
    /// Every resistor and switch-family device, program-wide (this module's
    /// doc comment).
    kcl_devices: Vec<KclDevice>,
    /// The `device_index` of every `kcl_devices` entry a `Static` partition
    /// owns: the currents [`Self::copy_in_static`] keeps in step with the
    /// static solver's (it copies the ones that solver recomputed; a debug
    /// build checks all of these on every call).
    #[cfg_attr(not(any(test, debug_assertions)), allow(dead_code, reason = "read by the debug-build check and the tests"))]
    static_kcl_devices: crate::shared::Shared<[usize]>,
    /// device index -> dispatch metadata, for every switch-family device
    /// only (a resistor/capacitor/voltage_source never takes a live param).
    switches: crate::shared::Shared<HashMap<usize, SwitchInfo>>,
    /// The single canonical, mutable copy of every switch-family device's
    /// CURRENT params (this module's doc comment).
    live_params: HashMap<usize, Vec<f64>>,
    /// Every capacitor device, program-wide: `(device_index, pins[0] node,
    /// pins[1] node)`, for folding its ALREADY-PUBLISHED current into
    /// [`Self::recompute_source_currents`]'s `leaving` array. Includes a
    /// [`FixedPairCapacitor`] too (its `device_index` appears in both lists)
    /// — this one only cares about the node ids for the KCL fold, not which
    /// list computed the current sitting in `currents[device_index]`.
    cap_devices: crate::shared::Shared<[(usize, usize, usize)]>,
    /// Every capacitor across two fixed, distinct nodes (rule 2a) —
    /// `partition::build` excludes these from every partition, so this is
    /// the only place anything ever publishes their current.
    fixed_pair_capacitors: Vec<FixedPairCapacitor>,
    /// `program.options.start_time` (owner decision 8/option key 7): every
    /// [`FixedPairCapacitor`] anchors its `vc = 0` here, exactly like a
    /// closed-form/stepped partition's capacitor does (core/README.md point
    /// 5), so it is kept alongside them rather than threaded through as a
    /// parameter from every one of [`Self::publish_fixed_pair_capacitors`]'s
    /// three call sites.
    start_time: f64,
    /// Every combinational IC, program-wide (this module's "shared
    /// IC-stamping helpers" section) — used by
    /// [`Self::recompute_source_currents`]'s driver/pull KCL fold (contract
    /// point 4's "fold driver and pull currents into the source KCL at
    /// fixed nodes") and by [`Self::maybe_schedule_bucket`]/`bucket_boundary`.
    /// Both of those walk every IC's global voltages/driver state directly
    /// (never through a per-partition solver), so — unlike
    /// [`KclDevice`]/[`SwitchInfo`] — this list needs no `Owner` bookkeeping
    /// of its own even now that an IC's driver/pull footprint can land in
    /// any partition kind (W1c step 2b lifted `partition::build`'s former
    /// `-3` decline): `static_part.rs`, `closed_form.rs` and `stepped.rs`
    /// each keep their OWN copy of this same list (`solve::collect_ics`),
    /// used only for the driver/pull stamps that touch THEIR OWN partitions.
    ics: crate::shared::Shared<[(usize, digital::IcRecord)]>,
    /// The currently-scheduled floating-bucket boundary time, or `None` —
    /// the dedup flag contract point 6 calls for ("the solver schedules...
    /// ONE boundary event... deduplicated"). Part of "the bucket schedule"
    /// contract point 7 names as state a failed instant must roll back
    /// ([`PartitionSolver::dynamic_snapshot`]/`dynamic_restore`).
    next_bucket: Option<f64>,
    /// Crossing/step events a dynamic partition asked to be scheduled since
    /// the last [`PartitionSolver::take_scheduled`] drain.
    pending_schedule: Vec<(f64, EventKind)>,
    largest_partition: usize,
    /// Contract stat 15 (not yet in the ABI table — see
    /// `partition::PartitionBuild::multi_capacitor_partitions`'s doc
    /// comment).
    multi_capacitor_linear_partitions: f64,
    /// Every stamped LED junction, program-wide, whichever partition holds it
    /// or none: folded into [`Self::recompute_source_currents`]'s KCL (LEDs
    /// point 12).
    junctions: crate::shared::Shared<[led::Junction]>,
    /// Every LED-family device, for the device pass
    /// ([`Self::publish_led_devices`]).
    led_devices: crate::shared::Shared<[LedDevice]>,
    /// The published channel array, one slot per program channel slot
    /// (`PartitionSolver::channels`). Rolls back with a failed instant, in
    /// [`PartitionSolver::dynamic_snapshot`] (LEDs point 16).
    channels: Vec<f64>,
    /// Whether `currents`' derived slots and `channels` are stale: a publish
    /// point found the lazy readout provably identical to the eager one
    /// ([`Self::lazy_publish_ok`]) and left the compute to
    /// [`PartitionSolver::prepare_readout`]. Rolls back with a failed
    /// instant, in [`PartitionSolver::dynamic_snapshot`], together with
    /// `derived_now` and `driver_snapshot`, the other inputs the deferred
    /// compute needs.
    derived_dirty: bool,
    /// The `now` of the publish that set `derived_dirty` (a floating pull's
    /// target depends on time).
    derived_now: f64,
    /// Whether every coefficient of the readout pass is bounded well inside
    /// the f64 range, so that with bounded voltages no derived value can
    /// overflow (see [`Self::lazy_static_ok`]). Fixed at load.
    lazy_static_ok: bool,
    /// Capacitors whose current a partition solver publishes (every one but a
    /// [`FixedPairCapacitor`]), for [`Self::lazy_publish_ok`]'s magnitude scan.
    partition_cap_devices: crate::shared::Shared<[usize]>,
    /// `leaving`/`extra` arrays the KCL walk reuses (zeroed at each use), so
    /// a compute allocates nothing.
    kcl_scratch: (crate::shared::Scratch<f64>, crate::shared::Scratch<f64>),
    /// device index -> position in `ics` (`u32::MAX`: not an IC), so
    /// [`PartitionSolver::ic_powered`] does not walk the list per call.
    ic_position: crate::shared::Shared<[u32]>,
    /// device index -> position in `kcl_devices`, for
    /// [`Self::refresh_kcl_contributions`].
    kcl_position: crate::shared::Shared<HashMap<usize, usize>>,
    /// The previous `driver_snapshot` table. Each resolve swaps it with
    /// `driver_snapshot`, so the table `stepped.resolve_dirty` needs as its
    /// pre-commit input is already in hand, and the new `driver_snapshot` is
    /// written over the older buffer in place instead of a fresh allocation.
    /// Meaningless between resolves.
    pre_commit_drivers: Rc<Vec<Driver>>,
    /// A clone of the kernel's full driver table, as of the most recent
    /// [`PartitionSolver::resolve_dirty`] call (W1c step 2b) — that trait
    /// method is the only one the kernel ever calls WITH a fresh `drivers`
    /// slice; [`PartitionSolver::handle_dynamic_event`]/`sync_dynamic_to`
    /// take none, because before this step no IC driver/pull ever touched a
    /// dynamic partition at all, so `closed_form`/`stepped` never needed to
    /// know a driver's committed state to restamp one. Now that they do
    /// (`partition.rs`'s decline is lifted), this is what stands in: a
    /// driver's committed level only ever changes via a `Commit` event, and
    /// every `Commit` marks its own partition dirty, which always drives a
    /// `resolve_dirty` call with the CURRENT `drivers` in the SAME delta —
    /// so by the time a LATER, unrelated `Crossing`/`Step`/horizon-clamp call
    /// can run, this snapshot is never stale. Included in
    /// [`PartitionSolver::dynamic_snapshot`]/`dynamic_restore` (rule 7): a
    /// failed instant's own livelock passes can call `resolve_dirty` many
    /// times with an oscillating driver table before the whole attempt is
    /// rolled back, and without restoring this too, a `Crossing`/`Step` that
    /// fires on some LATER, otherwise-unrelated instant (with no
    /// intervening `resolve_dirty` of its own — a due `Crossing`/`Step` with
    /// nothing else dirty that same delta) would restamp against the failed
    /// attempt's last, never-committed driver states instead of the
    /// correctly-restored ones.
    driver_snapshot: Rc<Vec<Driver>>,
    /// Upper bounds on `|v|` over every published voltage and on `|i|` over
    /// every partition-published capacitor current, which is all
    /// [`Self::lazy_publish_ok`] asks of them. A write raises them
    /// ([`Self::fold_voltage`]), so they only ever exceed the true maxima;
    /// a rescan sets them exact; a rollback, which brings back values written
    /// before them, sets them to infinity so the next publish rescans.
    v_hint: f64,
    i_hint: f64,
    /// The devices the readout pass writes (fixed-pair capacitors, sources,
    /// 555s, LED devices), for the window's undo log.
    derived_devices: crate::shared::Shared<[usize]>,
    /// Per dynamic partition, by global index (`DynOut::default()` for the
    /// rest): what it publishes.
    dyn_out: crate::shared::Shared<[DynOut]>,
    /// Global indices of the closed-form and of the stepped partitions.
    cf_parts: crate::shared::Shared<[usize]>,
    stepped_parts: crate::shared::Shared<[usize]>,
    /// The dynamic partitions the current call may have written, by global
    /// index: logged before the call, folded into the bounds and checked
    /// after it.
    written: Vec<u32>,
    /// The classified dirty ids of one `resolve_dirty` call, reused.
    static_ids: crate::shared::Scratch<PartitionId>,
    /// The readers `resolve_dirty_into` and `handle_dynamic_event_into`
    /// collect before handing them on, reused across calls (F5q).
    readers_buf: crate::shared::Scratch<DeviceId>,
    cf_ids: crate::shared::Scratch<usize>,
    st_ids: crate::shared::Scratch<usize>,
    /// Per IC (position in `ics`): the ids of its drivers, ascending, and the
    /// driver-table length they were derived for; the table only grows at
    /// load and a driver's `device` never changes.
    ic_driver_ids: crate::shared::Shared<[Vec<u32>]>,
    ic_driver_ids_len: usize,
    /// The ICs (positions in `ics`, ascending) that can add partitions in
    /// [`Self::add_watch_dependent_partitions`]: a supply node free in a
    /// `Static` or `Stepped` partition and at least one watched read pin on a
    /// closed-form or stepped one, with those pins' partitions in pin order
    /// as `(is_closed_form, global index)`.
    watch_dependents: crate::shared::Shared<[(u32, Vec<(bool, usize)>)]>,

    // ---- the rollback window ([`PartitionSolver::begin_instant`]) ----------
    window: bool,
    /// `(slot, old bits)` of every `voltages`/`currents` slot the window has
    /// written, in write order; a rollback undoes them newest first.
    v_log: Vec<(u32, u64)>,
    i_log: Vec<(u32, u64)>,
    /// Which dynamic partitions' published slots are already in the logs, and
    /// whether the derived slots and channels are.
    dyn_logged: Vec<bool>,
    dyn_logged_list: Vec<u32>,
    derived_logged: bool,
    /// The whole-value state the window found: it changes too rarely, or is
    /// too small, to be worth a log.
    w_pending_schedule: Vec<(f64, EventKind)>,
    w_next_bucket: Option<f64>,
    w_driver_snapshot: Option<Rc<Vec<Driver>>>,
    w_derived: (bool, f64),
    w_channels: Vec<f64>,

    // ---- the driver table's mirror ([`PartitionSolver::driver_changing`]) ---
    /// Whether the kernel has ever called `driver_changing`, and whether this
    /// solver relies on it: it starts to at the first resolve or window end
    /// after that, when no window is open.
    mirror_seen: bool,
    mirror_live: bool,
    /// Whether `driver_snapshot` equals the kernel's table as of the last
    /// resolve, apart from the entries in `stale`. False until a resolve has
    /// copied the table whole.
    mirror_valid: bool,
    /// Driver indices the kernel has changed since the last resolve.
    stale: Vec<u32>,
    /// `(index, entry before)` of what the current resolve refreshed, so the
    /// pre-commit table can be rebuilt from it when a stepped partition needs
    /// it.
    call_patches: Vec<(u32, Driver)>,
    /// The same for the whole window, and where in it a whole-table copy was
    /// made (entries before it are undone on that copy's predecessor, which
    /// `w_driver_snapshot` holds).
    d_log: Vec<(u32, Driver)>,
    d_log_split: usize,
    w_stale: Vec<u32>,
    /// Whether the open window rolls the table back through `d_log` (the
    /// mirror is live and valid at its start) or by holding its `Rc`.
    window_mirror: bool,
    /// Test builds keep the shared arrays from when the window opened and
    /// check, at its end, that undoing the logs reproduces them (see the
    /// static solver's field of the same name).
    #[cfg(test)]
    w_saved: Option<(Vec<f64>, Vec<f64>, Vec<f64>)>,
    /// Test builds record the battery steps exact mode charges, `(its end,
    /// the charge)`, one list per call that charged.
    #[cfg(test)]
    pub(crate) bill_log: Vec<Vec<(f64, f64)>>,

    // ---- W1f: the live boundary and the hub rail ([`hub`]) ---------------
    /// The waveform sources and the propagation forest, shared with the
    /// stepped solver while a ramp runs. A window holds the `Rc` it found.
    wave: Rc<live::RampEval>,
    w_wave: Option<Rc<live::RampEval>>,
    /// waveform source -> the forest entries under it, resolution order;
    /// and every partition reading a node under it.
    wave_entries: crate::shared::Shared<[Vec<u32>]>,
    wave_parts: crate::shared::Shared<[Vec<usize>]>,
    /// node -> the partitions, the watched closed-form or stepped partitions
    /// (through an IC supplied from it), the ICs and the rule-2a capacitors
    /// that read it. Filled for live nodes only.
    node_parts: crate::shared::Shared<[Vec<u32>]>,
    node_watch_parts: crate::shared::Shared<[Vec<u32>]>,
    node_ic_readers: crate::shared::Shared<[Vec<u32>]>,
    node_caps: crate::shared::Shared<[Vec<u32>]>,
    /// Whether a rule-2a capacitor sits on a live node (its anchor is then
    /// instant state).
    live_caps: bool,
    w_caps: Vec<(f64, f64, f64)>,
    /// node -> not free (fixed, or the rail outside exact mode).
    fixed_node: crate::shared::Shared<[bool]>,
    /// device -> its kind code.
    device_kinds: crate::shared::Shared<[u32]>,
    /// The hub rail outside exact mode, and exact mode's hub.
    hub: Option<Box<hub::Hub>>,
    exact: Option<Box<hub::ExactHub>>,
    /// Ruling R59 (1) (B): the closed-form hub partitions that step while the
    /// PSU is in CC (`partition::PartitionBuild::cc_switch`), ascending, and
    /// the window's log of their kind switches, `(partition, kind before)`
    /// in order (a rollback undoes them newest first).
    cc_switch: crate::shared::Shared<[usize]>,
    kind_log: Vec<(u32, PartitionKind)>,
    /// The switch's own state (`hub::CcCrawl`), and its value when the
    /// window opened.
    cc_crawl: hub::CcCrawl,
    w_cc_crawl: hub::CcCrawl,
    /// Stat 19.
    source_edges: u64,
    /// Whether this call re-solved exact mode's rail partition (static, or a
    /// stepped one's seed), which runs the regime rules even with no step
    /// committed since the last (Rails point 5: at load, the only solve).
    exact_solved: bool,
    /// Set by R74's landing (`rail_loop`) for the `hold_on_rail` it calls,
    /// which takes it.
    landing_hold: bool,

    // ---- W1g: thermal, failures and warnings ([`settle`]) -----------------
    /// All slow state (README W1g point 1), journaled per instant.
    slow: crate::slow::SlowState,
    /// The load-time tables the settle reads: which devices are slow, how
    /// each is driven, and the `output_sag` cells.
    slow_model: crate::shared::Shared<settle::SlowModel>,
    /// What happened since the last settle: the partitions published, the
    /// drivers that changed, the devices to re-drive.
    slow_scratch: settle::SlowScratch,
    /// Stats 21-24. Not rolled back, as every stat.
    slow_counts: [u64; 4],

    // ---- F5: the power-hold table (W1c point 1a) --------------------------
    /// One [`digital::power::PowerHold`] per device index (an IC's only
    /// meaningful), empty until the first hold: device-indexed rather than
    /// by position in `ics`, because the stamps judge power over the
    /// partition solvers' own IC lists, which are subsets. Shared with the
    /// static solver; a change replaces it (`Rc::make_mut`), so a window
    /// holds the table it found as the driver table's `Rc` is held.
    holds: Rc<Vec<digital::power::PowerHold>>,
    /// The entries not free: while 0, every verdict is the window test and
    /// every hot path pays one branch (the SAP-1 never holds).
    holds_active: u32,
    /// Ruling R90 (a): bumped by every failure latch, PTC trip or recovery,
    /// reset and rolled-back instant, the events that change which stepped
    /// devices the stress probes cover (`refresh_stress_probes`). A count of
    /// failed and tripped devices could not key them: one PTC tripping as
    /// another recovers leaves the counts where they were (R80V3).
    probe_version: u64,
    /// The `probe_version` the probes were last built at; `Some(u64::MAX)`
    /// once a build found no rated stepped device and none failed or
    /// tripped, so none can become one.
    probes_at: Option<u64>,
    /// `holds_active > 0`, as the kernel reads it
    /// ([`PartitionSolver::holds_flag`]); set with the sub-solvers' copies
    /// ([`Self::sync_power_holds`]), so it follows every change, rollback
    /// and restore of the table. Derived, so not journaled itself.
    holds_flag: HoldsFlag,
    /// The window's journal of the table, `holds_active` and `episodes`,
    /// taken at the first touch of either (Rails point 8).
    w_holds: Option<(Rc<Vec<digital::power::PowerHold>>, u32, Rc<Vec<hold::Episode>>, bool)>,
    /// Holds taken and released (D8, classic's `_holdOffCounts.accepted`).
    /// Not rolled back, as every stat; a drop at `set_params`,
    /// `seed_memory` or `resetFailures` is not a release.
    hold_counts: [u64; 2],
    /// Each IC's sliding episode, device-indexed, empty until the first hold
    /// (ruling R51, the hold-episode guard: [`hold::Episode`]). Rolled back
    /// with the table; dropped with every hold where the circuit changes.
    episodes: Rc<Vec<hold::Episode>>,
    /// Whether this host step's retry of the rail's ICs held off is still to
    /// come (ruling R51 (b), classic's H8 cadence: [`Self::retry_on_rail`]):
    /// set at load and at each horizon clamp, the end of a host step
    /// (`advance`), taken by the first retry after it. Journaled with the
    /// table (Rails point 8).
    retry_armed: bool,
    /// Set only inside the horizon clamp: the rail loop retries there even
    /// when no pass solved.
    at_clamp: bool,
    /// Set only inside a source edge and a re-solve of dirty partitions:
    /// what the rail sees changed at `now`, so a rail move there is a step,
    /// not the draws' drift (`rail_passes` brings the stepped hub
    /// partitions to it on the rail they were planned on).
    rail_jump: bool,
    /// Set by a host's live edit (`apply_params`) and cleared after the rail
    /// loop that edit's instant runs (and at the next host step): on a CC rail capacitor the edit moves
    /// the segment, not the rail, and a stepped hub partition already at the
    /// instant plans its trial again on the new one (ruling R89 (a),
    /// `rail_passes`).
    host_edit: bool,
    /// The length of the host step (`advance` call) running, the guard's
    /// yardstick for a short hold (ruling R54 (a)); set by the kernel at
    /// each call, infinite before the first.
    host_step: f64,
    /// The open instant's foreign driver changes (ruling R54 (a)): a release
    /// one of them caused ends its IC's episode. Cleared as each instant
    /// opens, so nothing rolls it back.
    changes: hold::Changes,
}

impl PartitionedSolver {
    /// Each table this solver shares with snapshots, by address (W2a tests).
    #[cfg(test)]
    pub(crate) fn shared_tables(&self) -> Vec<(&'static str, usize)> {
        vec![("resolved", crate::shared::Shared::addr(&self.resolved)), ("local_index", crate::shared::Shared::addr(&self.local_index)), ("partition_kinds", crate::shared::Shared::addr(&self.partition_kinds)), ("static_nodes", crate::shared::Shared::addr(&self.static_nodes)), ("static_kcl_devices", crate::shared::Shared::addr(&self.static_kcl_devices)), ("switches", crate::shared::Shared::addr(&self.switches)), ("cap_devices", crate::shared::Shared::addr(&self.cap_devices)), ("ics", crate::shared::Shared::addr(&self.ics)), ("junctions", crate::shared::Shared::addr(&self.junctions)), ("led_devices", crate::shared::Shared::addr(&self.led_devices)), ("partition_cap_devices", crate::shared::Shared::addr(&self.partition_cap_devices)), ("ic_position", crate::shared::Shared::addr(&self.ic_position)), ("kcl_position", crate::shared::Shared::addr(&self.kcl_position)), ("derived_devices", crate::shared::Shared::addr(&self.derived_devices)), ("dyn_out", crate::shared::Shared::addr(&self.dyn_out)), ("cf_parts", crate::shared::Shared::addr(&self.cf_parts)), ("stepped_parts", crate::shared::Shared::addr(&self.stepped_parts)), ("ic_driver_ids", crate::shared::Shared::addr(&self.ic_driver_ids)), ("watch_dependents", crate::shared::Shared::addr(&self.watch_dependents)), ("wave_entries", crate::shared::Shared::addr(&self.wave_entries)), ("wave_parts", crate::shared::Shared::addr(&self.wave_parts)), ("node_parts", crate::shared::Shared::addr(&self.node_parts)), ("node_watch_parts", crate::shared::Shared::addr(&self.node_watch_parts)), ("node_ic_readers", crate::shared::Shared::addr(&self.node_ic_readers)), ("node_caps", crate::shared::Shared::addr(&self.node_caps)), ("fixed_node", crate::shared::Shared::addr(&self.fixed_node)), ("device_kinds", crate::shared::Shared::addr(&self.device_kinds)), ("cc_switch", crate::shared::Shared::addr(&self.cc_switch)), ("slow_model", crate::shared::Shared::addr(&self.slow_model))]
    }
}

impl PartitionedSolver {
    pub fn build(program: &Program) -> Result<Self, BoError> {
        #[cfg(test)]
        curvature::hook::clear();
        let pb = partition::build(program)?;
        // The dynamic solvers read W1f's topology as they build.
        let closed_form = closed_form::ClosedFormSolver::build(program, &pb.partitions, &pb.local_index, &pb.fixed, &pb.kinds, LiveTopology::of(&pb));
        // Ruling R59 (1) (B): a `cc_switch` partition is compiled by both
        // dynamic solvers and runs in the one its kind names, the other
        // copy parked (`sync_cc_kinds`). It loads closed-form.
        let cc_switch: Vec<usize> = (0..pb.kinds.len()).filter(|&p| pb.cc_switch[p]).collect();
        let mut stepped = if cc_switch.is_empty() {
            stepped::SteppedSolver::build(program, &pb.partitions, &pb.local_index, &pb.fixed, &pb.kinds, LiveTopology::of(&pb))
        } else {
            let kinds: Vec<PartitionKind> = pb.kinds.iter().zip(&pb.cc_switch).map(|(&k, &sw)| if sw { PartitionKind::Stepped } else { k }).collect();
            stepped::SteppedSolver::build(program, &pb.partitions, &pb.local_index, &pb.fixed, &kinds, LiveTopology::of(&pb))
        };
        for &p in &cc_switch {
            stepped.set_parked(p, true);
        }
        let partition::PartitionBuild { partitions, local_index, fixed, resolved, kinds: partition_kinds, multi_capacitor_partitions, rail, live, reads, hub, .. } = pb;

        let node_count = program.node_count;
        let device_count = program.devices.len();
        let mut voltages = vec![0.0; node_count];
        for (node, v) in fixed.iter().enumerate() {
            if let Some(v) = v {
                voltages[node] = *v;
            }
        }
        let currents = vec![0.0; device_count];

        let static_nodes: Vec<usize> =
            (0..node_count).filter(|&n| matches!(local_index[n], Some((p, _)) if partition_kinds[p] == PartitionKind::Static)).collect();

        let owner_of = |device: &crate::program::Device| -> Owner {
            match device.pins.iter().find_map(|p| local_index[p.node].map(|(pi, _)| (pi, partition_kinds[pi]))) {
                Some((p, PartitionKind::ClosedForm)) => Owner::ClosedForm(p),
                Some((p, PartitionKind::Stepped)) => Owner::Stepped(p),
                _ => Owner::Static, // no free pin, or its free pin(s) are Static-owned.
            }
        };

        let mut switches = HashMap::new();
        let mut live_params = HashMap::new();
        let mut kcl_devices = Vec::new();
        let mut cap_devices = Vec::new();
        let mut fixed_pair_capacitors = Vec::new();

        for (d, device) in program.devices.iter().enumerate() {
            match device.kind {
                kinds::RESISTOR | kinds::FUSE | kinds::PTC_FUSE => {
                    let nodes: Vec<usize> = device.pins.iter().map(|p| p.node).collect();
                    let contributions = KclDevice::resolve_contributions(device.kind, &nodes, &device.params);
                    kcl_devices.push(KclDevice { device_index: d, kind: device.kind, nodes, owner: owner_of(device), contributions });
                }
                kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH => {
                    let nodes: Vec<usize> = device.pins.iter().map(|p| p.node).collect();
                    let pin_count = nodes.len();
                    let owner = owner_of(device);
                    let contributions = KclDevice::resolve_contributions(device.kind, &nodes, &device.params);
                    kcl_devices.push(KclDevice { device_index: d, kind: device.kind, nodes, owner, contributions });
                    switches.insert(d, SwitchInfo { kind: device.kind, pin_count, owner });
                    live_params.insert(d, device.params.clone());
                }
                kinds::CAPACITOR => {
                    // A self-shorted capacitor (contract point 2) folds to a
                    // no-op in `recompute_source_currents`'s `leaving[u] +=
                    // i; leaving[w] -= i;` anyway (u == w cancels whatever
                    // `i` is), but it is excluded here too, on principle: it
                    // never has a published current to fold in the first
                    // place (`compile_dyn_shape` never counts it as any
                    // partition's capacitor, and `static_part.rs` never
                    // stamps a capacitor at all), so it has no business in
                    // this KCL device list either.
                    let (u, w) = (device.pins[0].node, device.pins[1].node);
                    if u == w {
                        continue;
                    }
                    cap_devices.push((d, u, w));
                    if local_index[u].is_none() && local_index[w].is_none() {
                        // Rule 2a: both terminals are fixed, distinct nodes
                        // (`partition::build` already excluded this device
                        // from every partition's capacitor count on exactly
                        // this test) — a decoupling capacitor straight
                        // across an ideal supply. `fixed[..]` is guaranteed
                        // `Some` for a node `local_index` reports as fixed
                        // (`PartitionBuild`'s own doc comment on the two
                        // fields).
                        let leak = device.params[2];
                        fixed_pair_capacitors.push(FixedPairCapacitor {
                            device_index: d,
                            nodes: (u, w),
                            d_v: fixed[u].expect("a fixed node always carries a voltage") - fixed[w].expect("a fixed node always carries a voltage"),
                            c: device.params[0],
                            esr: device.params[1],
                            leak_g: if leak > 0.0 { 1.0 / leak } else { 0.0 },
                            vc0: 0.0,
                            t0: program.options.start_time,
                        });
                    }
                }
                _ => {}
            }
        }

        let mut junctions = Vec::new();
        let mut led_devices = Vec::new();
        for (d, device) in program.devices.iter().enumerate() {
            if !matches!(device.kind, kinds::LED | kinds::RGB_LED | kinds::BICOLOR_LED | kinds::SEG7_CC | kinds::SEG7_CA) {
                continue;
            }
            let device_junctions = led::junctions(d, device, program.options.ambient_c);
            junctions.extend(device_junctions.iter().filter(|j| j.stamped()).copied());
            let channel_count = (kinds::shape(device.kind).expect("a program the reader accepted has known kinds").channel_slots)(device.pins.len());
            // The blob's first value is the first channel slot for a kind
            // with channel slots, and there is no blob at all for `led`.
            let channel_base = if channel_count > 0 { device.blob[0] as usize } else { 0 };
            led_devices.push(LedDevice { device_index: d, device: device.clone(), junctions: device_junctions, channel_base, channel_count });
        }

        let largest_partition = partitions.iter().map(|p| p.len()).max().unwrap_or(0);
        let ics = collect_ics(program);

        let static_solver = static_part::StaticPartitionSolver::build(program)?;

        let _ = device_count; // documents intent; `currents` is already sized from it above.

        let static_kcl_devices: Vec<usize> = kcl_devices.iter().filter(|kd| kd.owner == Owner::Static).map(|kd| kd.device_index).collect();
        let kcl_position: HashMap<usize, usize> = kcl_devices.iter().enumerate().map(|(i, kd)| (kd.device_index, i)).collect();
        let mut ic_position = vec![u32::MAX; device_count];
        for (i, (d, _)) in ics.iter().enumerate() {
            ic_position[*d] = i as u32;
        }
        let fixed_pair_device: BTreeSet<usize> = fixed_pair_capacitors.iter().map(|c| c.device_index).collect();
        let partition_cap_devices: Vec<usize> = cap_devices.iter().map(|&(d, _, _)| d).filter(|d| !fixed_pair_device.contains(d)).collect();
        let channel_slot_count = program.channel_slot_count;
        let readout_term_count = node_count + device_count + 4 * program.devices.iter().map(|d| d.pins.len()).sum::<usize>();

        let mut dyn_out: Vec<DynOut> = (0..partition_kinds.len()).map(|_| DynOut::default()).collect();
        let (mut cf_parts, mut stepped_parts) = (Vec::new(), Vec::new());
        for (p, kind) in partition_kinds.iter().enumerate() {
            match kind {
                PartitionKind::ClosedForm => {
                    dyn_out[p] = DynOut::of(closed_form.shape(p).expect("a closed-form partition has a compiled shape"));
                    cf_parts.push(p);
                    // Either solver may write it.
                    if cc_switch.contains(&p) {
                        stepped_parts.push(p);
                    }
                }
                PartitionKind::Stepped => {
                    dyn_out[p] = DynOut::of(stepped.shape(p).expect("a stepped partition has a compiled shape"));
                    stepped_parts.push(p);
                }
                PartitionKind::Static => {}
            }
        }
        let mut derived_devices: Vec<usize> = fixed_pair_capacitors.iter().map(|c| c.device_index).collect();
        derived_devices.extend(resolved.iter().map(|r| r.device_index));
        derived_devices.extend(ics.iter().filter(|(_, ic)| digital::is_ne555(ic.kind)).map(|(d, _)| *d));
        derived_devices.extend(led_devices.iter().map(|e| e.device_index));
        let watch_dependents = build_watch_dependents(&ics, &local_index, &partition_kinds);
        let partition_total = partition_kinds.len();
        let ic_count = ics.len();

        let mut solver = PartitionedSolver {
            voltages,
            currents,
            resolved: resolved.into(),
            local_index: local_index.into(),
            partition_kinds: partition_kinds.into(),
            static_nodes: static_nodes.into(),
            static_solver,
            closed_form,
            stepped,
            kcl_devices: kcl_devices.into(),
            static_kcl_devices: static_kcl_devices.into(),
            switches: switches.into(),
            live_params,
            cap_devices: cap_devices.into(),
            fixed_pair_capacitors: fixed_pair_capacitors.into(),
            start_time: program.options.start_time,
            ics: ics.into(),
            next_bucket: None,
            pending_schedule: Vec::new(),
            largest_partition,
            multi_capacitor_linear_partitions: multi_capacitor_partitions as f64,
            junctions: junctions.into(),
            led_devices: led_devices.into(),
            channels: vec![0.0; channel_slot_count],
            derived_dirty: false,
            derived_now: 0.0,
            lazy_static_ok: false,
            partition_cap_devices: partition_cap_devices.into(),
            kcl_scratch: (crate::shared::Scratch::default(), crate::shared::Scratch::default()),
            ic_position: ic_position.into(),
            kcl_position: kcl_position.into(),
            pre_commit_drivers: Rc::new(Vec::new()),
            driver_snapshot: Rc::new(Vec::new()),
            v_hint: f64::INFINITY,
            i_hint: f64::INFINITY,
            derived_devices: derived_devices.into(),
            dyn_out: dyn_out.into(),
            cf_parts: cf_parts.into(),
            stepped_parts: stepped_parts.into(),
            written: Vec::new(),
            static_ids: crate::shared::Scratch::default(),
            readers_buf: crate::shared::Scratch::default(),
            cf_ids: crate::shared::Scratch::default(),
            st_ids: crate::shared::Scratch::default(),
            ic_driver_ids: (vec![Vec::new(); ic_count]).into(),
            ic_driver_ids_len: usize::MAX,
            watch_dependents: watch_dependents.into(),
            window: false,
            v_log: Vec::new(),
            i_log: Vec::new(),
            dyn_logged: vec![false; partition_total],
            dyn_logged_list: Vec::new(),
            derived_logged: false,
            w_pending_schedule: Vec::new(),
            w_next_bucket: None,
            w_driver_snapshot: None,
            w_derived: (false, 0.0),
            w_channels: Vec::new(),
            mirror_seen: false,
            mirror_live: false,
            mirror_valid: false,
            stale: Vec::new(),
            call_patches: Vec::new(),
            d_log: Vec::new(),
            d_log_split: 0,
            w_stale: Vec::new(),
            window_mirror: false,
            #[cfg(test)]
            w_saved: None,
            #[cfg(test)]
            bill_log: Vec::new(),
            wave: Rc::new(live::RampEval::default()),
            w_wave: None,
            wave_entries: Vec::new().into(),
            wave_parts: Vec::new().into(),
            node_parts: Vec::new().into(),
            node_watch_parts: Vec::new().into(),
            node_ic_readers: Vec::new().into(),
            node_caps: Vec::new().into(),
            live_caps: false,
            w_caps: Vec::new(),
            fixed_node: fixed.iter().map(|f| f.is_some()).collect(),
            device_kinds: program.devices.iter().map(|d| d.kind).collect(),
            hub: None,
            exact: None,
            source_edges: 0,
            exact_solved: false,
            landing_hold: false,
            slow: crate::slow::SlowState::new(device_count),
            slow_model: settle::SlowModel::default().into(),
            slow_scratch: settle::SlowScratch::default(),
            slow_counts: [0; 4],
            holds: Rc::new(Vec::new()),
            holds_active: 0,
            probe_version: 0,
            probes_at: None,
            holds_flag: HoldsFlag(Rc::new(std::cell::Cell::new(false))),
            w_holds: None,
            hold_counts: [0; 2],
            episodes: Rc::new(Vec::new()),
            retry_armed: true,
            at_clamp: false,
            rail_jump: false,
            host_edit: false,
            host_step: f64::INFINITY,
            changes: hold::Changes::None,
            cc_switch: cc_switch.into(),
            kind_log: Vec::new(),
            cc_crawl: hub::CcCrawl::default(),
            w_cc_crawl: hub::CcCrawl::default(),
        };
        solver.wire_w1f(program, rail, &live, &reads, &hub, &partitions);
        solver.wire_w1g(program, &live, &fixed);
        solver.lazy_static_ok = solver.compute_lazy_static_ok(readout_term_count);
        solver.rescan_bounds();
        // Nothing reads the static solver's own source currents: this solver
        // computes them from the whole circuit (`recompute_source_currents`).
        solver.static_solver.set_source_walk_skippable(true);
        #[cfg(debug_assertions)]
        {
            let mut owned: Vec<usize> = solver.static_solver.stamp_devices().collect();
            owned.sort_unstable();
            let mut listed = solver.static_kcl_devices.clone();
            listed.sort_unstable();
            debug_assert_eq!(owned, listed, "the static solver owns exactly the devices `copy_in_static` copies");
        }
        solver.register_ic_read_pin_watches();
        // After the watches: they decide whether another partition supplies
        // the battery's (`SteppedSolver::set_defer`).
        if let Some(e) = solver.exact.as_ref() {
            if let crate::rails::HubKind::Battery(m) = &e.kind {
                let (p, model, state) = (e.partition, m.clone(), e.battery);
                solver.stepped.set_defer(Some(p));
                solver.stepped.set_cell(model, state);
            }
        }
        Ok(solver)
    }

    /// W1c contract point 4's own reader rule, closing the production gap
    /// R6b's own W1c step 2b report left open: "A reader whose pin sits on a
    /// closed-form or stepped partition registers that pin's live thresholds
    /// (vil/vih, or the scaled vtMinus/vtPlus...) with the partition's
    /// crossing watch, so it is re-evaluated at the crossing instant... not
    /// at the next unrelated event." Nothing in production ever called
    /// [`Self::watch_node`] before this — `devices::ics::IcEvaluator`
    /// (I1's own production `DeviceEvaluator`) reads pins and drives outputs,
    /// but has no channel back into the solver to register a watch of its
    /// own, and nothing else in the kernel/evaluator pair ever will either
    /// (this crate's `DeviceEvaluator` trait has no such hook). So this
    /// solver registers every eligible watch itself, once, at load — the
    /// only place that already has both a `PartitionSolver` to register
    /// against and the IC list to register for.
    ///
    /// Only a NON-OPEN, NON-OUTPUT-CAPABLE logic pin. An open pin's read
    /// never depends on its node voltage at all (`digital::levels::watch_thresholds`'s
    /// own `Vec::new()` for that case), so it would be a pure no-op, harmless
    /// but pointless to register. An output-capable pin is actively WRONG to
    /// register (this method's own body has the full account: a driven
    /// output sits pinned near its own rail by `1/R_out`, so solving
    /// BACKWARD for `vc` from its own near-flat equation, the way
    /// `ClosedFormSolver::handle_crossing` does for whichever node the
    /// crossing landed on, is numerically unstable and can publish every
    /// other node in the segment at a physically meaningless value).
    ///
    /// A pin on a `ClosedForm` partition only when BOTH this IC's supply
    /// nodes are load-time STABLE ([`Self::supply_is_load_stable`]: fixed, or
    /// free in a `Static` partition — contract point 4, amended by finding
    /// 2/6): a `ClosedForm` partition itself is guaranteed never to hold a
    /// free IC supply node (`partition::build`'s own promotion rule
    /// reclassifies any partition that does to `Stepped`), but a PLAIN read
    /// pin (unlike an output-capable or open-floating-input one) is never
    /// unioned with its own IC's supply at all, so it can land in a
    /// `ClosedForm`/`Stepped` partition while its OWN ic's supply sits, free,
    /// in some entirely different partition. A `Static` partition's voltage
    /// is exactly as load-time-stable as a truly fixed node BETWEEN dirtying
    /// events (a switch toggle, a driver commit), and
    /// [`Self::add_watch_dependent_partitions`] re-aims every watch that
    /// depends on one whenever such an event actually moves it. A supply free
    /// in a `Stepped` partition moves continuously, which a closed-form
    /// crossing (solved once, for one threshold) cannot follow, so
    /// `partition::build` promotes the `ClosedForm` partition of such a pin
    /// to `Stepped` (W1e follow-up, gap 1). A pin still `ClosedForm` here
    /// therefore has a stable supply, and one on a `Stepped` partition is
    /// watched WHATEVER its supply, as a 555's is (NE555 point 5). Its
    /// thresholds then follow the supply inside `stepped`: taken from each
    /// trial's own state when the supply is in the same partition, and from
    /// the supply partition's pending trial when it is another one
    /// ([`stepped::SteppedSolver::add_supply_dependency`]).
    ///
    /// Both directions are registered for every threshold value
    /// `watch_thresholds` returns, rather than only the direction each one
    /// is individually meaningful for (a plain pin's `vil` only truly matters
    /// falling, into LOW; `vih` only truly matters rising, into HIGH — the
    /// other direction of each merely re-enters the dead band, still worth
    /// noticing since a Schmitt-free pin's dead-band read is itself
    /// time-varying, `digital::floating::floating_high` on the dead key): an
    /// extra evaluation an unreached direction would cost is harmless (the
    /// closed-form/stepped crossing search itself discards any threshold a
    /// segment's own asymptote never reaches, `closed_form::schedule_crossing`/
    /// `stepped::find_crossing`), and registering unconditionally avoids
    /// re-deriving which of a Schmitt part's two directions versus a plain
    /// part's four actually matter.
    fn register_ic_read_pin_watches(&mut self) {
        for i in 0..self.ics.len() {
            let (device_index, ic) = self.ics[i].clone();
            // NE555 point 5 replaces W1c point 4's generic per-pin watch
            // entirely for code 40: it has no `vil`/`vih`/Schmitt read at
            // all (`digital::levels::watch_thresholds` would find nothing to
            // watch there anyway), and its own eligibility rule differs from
            // every other IC's (`Self::register_ne555_watches`'s own doc
            // comment: a 555 is watched on a Stepped partition even with a
            // dynamic supply, unlike a generic IC).
            if digital::is_ne555(ic.kind) {
                self.register_ne555_watches(device_index, &ic);
                continue;
            }
            let supply_stable = self.supply_is_load_stable(ic.vcc.node) && self.supply_is_load_stable(ic.gnd.node);
            // An output-capable pin is NEVER a valid watch target, on top of
            // simply being meaningless as an INPUT (`digital::levels::read`
            // is called on it too, since `IcRecord::logic` carries every pin
            // uniformly — this crate's own `devices::ics::IcEvaluator`'s doc
            // comment — but nothing ever CONSUMES that particular boolean:
            // `digital::combinational::evaluate` only reads the pins each
            // kind's own truth table names). It is actively WRONG to watch:
            // a driven output sits pinned near its own rail by `1/R_out` (a
            // strong, low-impedance path), so its voltage barely depends on
            // this partition's `vc` at all — `ClosedFormSolver::handle_crossing`
            // solves BACKWARD for `vc` from the crossed node's own affine
            // equation (`(cross_v - x0[cross_local]) / dx[cross_local]`),
            // and a near-zero `dx` there makes that solve numerically
            // unstable, publishing a `vc` (hence every OTHER node in the
            // segment) far from anything physically meaningful. Confirmed by
            // running the 74HC14 oscillator through the real `Engine`: with
            // an output pin watched too, the driver's own commit (`1/R_out`
            // to the OPPOSITE rail) makes that SAME output pin race back
            // across its threshold within nanoseconds, hijacking every
            // later re-aim before the real input ever gets anywhere near ITS
            // own threshold — the oscillator livelocks in a burst of
            // sub-microsecond crossings and its output never actually
            // flips again.
            let power = digital::power::ic_power(&ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node]);
            for (logic_index, pin) in ic.logic.iter().enumerate() {
                if !partition::is_watched_read_pin(&ic, logic_index) {
                    continue;
                }
                let kind = self.local_index.get(pin.node).copied().flatten().and_then(|(p, _)| self.partition_kinds.get(p).copied());
                let switch = self.local_index[pin.node].is_some_and(|(p, _)| self.cc_switch.contains(&p));
                let eligible = match kind {
                    Some(PartitionKind::Stepped) => true,
                    Some(PartitionKind::ClosedForm) => supply_stable,
                    _ => false,
                };
                if !eligible && !switch {
                    continue;
                }
                let thresholds = digital::levels::watch_thresholds(&ic, &power, pin);
                if thresholds.is_empty() {
                    continue;
                }
                let reader = DeviceId(device_index as u32);
                let node = pin.node;
                // Recomputed fresh on every call from the LIVE voltages
                // array (finding 2/6), not the `thresholds` snapshot just
                // above -- that one only decides whether this pin is worth
                // registering AT ALL (a non-open pin's `watch_thresholds` is
                // never actually empty today, but the check is cheap
                // insurance against a future kind where it could be). A
                // `Static`-supply IC's thresholds can move after load, when
                // `Self::add_watch_dependent_partitions` re-aims this watch
                // following a re-solve of the partition holding `vcc`/`gnd`;
                // a fixed-supply IC's never move, so recomputing costs one
                // redundant `ic_power` call and produces the identical
                // answer every time.
                let pin_for_watch = *pin;
                let vcc_node = ic.vcc.node;
                let gnd_node = ic.gnd.node;
                let make = || -> ThresholdFn {
                    let ic_for_watch = ic.clone();
                    Box::new(move |voltages: &[f64]| {
                        let power = digital::power::ic_power(&ic_for_watch, voltages[vcc_node], voltages[gnd_node]);
                        digital::levels::watch_thresholds(&ic_for_watch, &power, &pin_for_watch)
                            .iter()
                            .flat_map(|&value| [Threshold { value, rising: true }, Threshold { value, rising: false }])
                            .collect()
                    })
                };
                if eligible {
                    self.watch_node(node, reader, make());
                }
                if kind == Some(PartitionKind::Stepped) {
                    self.stepped.add_supply_dependency(node, ic.vcc.node);
                    self.stepped.add_supply_dependency(node, ic.gnd.node);
                }
                // Its stepped copy watches the pin too (eligible there
                // whatever the supply), for while it steps (R59 (1) (B)).
                if switch {
                    self.stepped.watch_node(node, reader, make());
                    self.stepped.add_supply_dependency(node, ic.vcc.node);
                    self.stepped.add_supply_dependency(node, ic.gnd.node);
                }
            }
        }
    }

    /// NE555 point 5: registers TRIG/RST/THR crossing watches, both
    /// directions, for the 555 at `device_index` — the "Moving thresholds"
    /// paragraph's own eligibility rule, which is NOT
    /// [`Self::supply_is_load_stable`] applied uniformly the way a generic
    /// IC's is (`Self::register_ic_read_pin_watches`'s own gate): a 555's
    /// read pin is watched whenever ITS OWN partition is `Stepped`
    /// (regardless of whether its supply sits in that same dynamic
    /// partition or a `Static` one — "a supply node on a dynamic partition:
    /// that partition is stepped, and the watches are registered all the
    /// same"), and watched on a `ClosedForm` partition only when both
    /// supply nodes are load-stable (fixed or `Static` — the same
    /// numerical-instability concern as any other IC: a `ClosedForm`
    /// partition never holds a free IC supply at all, `partition::build`'s
    /// promotion rule, so this half of the rule can never actually diverge
    /// from the generic one in practice, but is spelled out here to match
    /// the contract's own wording rather than silently relying on that
    /// invariant). A pin that is open, or whose node is fixed (no partition
    /// at all: [`Self::watch_node`] simply returns `false` for it, so
    /// registering unconditionally is harmless), is naturally excluded by
    /// the `match` below rather than a separate check — unlike a generic
    /// IC's watch, a 555's own `pin.open` is NOT excluded on principle
    /// (NE555 point 2: "an unwired RST reads its node voltage... Rust
    /// reads OPEN on pins 8 and 1 only", never on a read pin), though in
    /// practice an `OPEN` read pin's node is always isolated (G8's rule)
    /// and so never actually lands on a `Stepped`/`ClosedForm` partition
    /// anyway.
    fn register_ne555_watches(&mut self, device_index: usize, ic: &digital::IcRecord) {
        let supply_stable = self.supply_is_load_stable(ic.vcc.node) && self.supply_is_load_stable(ic.gnd.node);
        for &logic_index in &[digital::NE555_TRIG, digital::NE555_RST, digital::NE555_THR] {
            let node = ic.logic[logic_index].node;
            let kind = self.local_index.get(node).copied().flatten().and_then(|(p, _)| self.partition_kinds.get(p).copied());
            let switch = self.local_index[node].is_some_and(|(p, _)| self.cc_switch.contains(&p));
            let eligible = match kind {
                Some(PartitionKind::Stepped) => true,
                Some(PartitionKind::ClosedForm) => supply_stable,
                _ => false,
            };
            let reader = DeviceId(device_index as u32);
            let (vcc_node, gnd_node) = (ic.vcc.node, ic.gnd.node);
            let make = || -> ThresholdFn { Box::new(move |voltages: &[f64]| digital::ne555::watch_thresholds(logic_index, voltages[vcc_node], voltages[gnd_node])) };
            if eligible {
                self.watch_node(node, reader, make());
            }
            if kind == Some(PartitionKind::Stepped) {
                self.stepped.add_supply_dependency(node, vcc_node);
                self.stepped.add_supply_dependency(node, gnd_node);
            }
            // Its stepped copy's watch (R59 (1) (B)), as for any IC.
            if switch {
                self.stepped.watch_node(node, reader, make());
                self.stepped.add_supply_dependency(node, vcc_node);
                self.stepped.add_supply_dependency(node, gnd_node);
            }
        }
    }

    /// Whether `node` is stable enough between dirtying events to serve as
    /// the basis for a crossing watch's thresholds (contract point 4,
    /// amended by finding 2/6): a genuinely fixed node, or one that is free
    /// but lives in a `Static` partition. Neither moves except when
    /// something dirties it (a switch toggle, a driver commit) — unlike a
    /// `ClosedForm`/`Stepped` partition's own node, which moves continuously
    /// between re-solves, the numerical-instability hazard
    /// `Self::register_ic_read_pin_watches`'s own doc comment describes for
    /// an output-capable pin applies here too.
    fn supply_is_load_stable(&self, node: usize) -> bool {
        match self.local_index[node] {
            None => true,
            Some((p, _)) => matches!(self.partition_kinds.get(p), Some(PartitionKind::Static)),
        }
    }

    /// Finding 2/6's re-aim path: for every IC watched only because one (or
    /// both) of its supply nodes sits in a `Static` partition, forces its
    /// dependent `ClosedForm`/`Stepped` partitions (the ones holding one of
    /// its currently-registered read-pin watches) into `cf_ids`/`st_ids`
    /// whenever `static_ids` (the partitions `Self::resolve_dirty` is about
    /// to re-solve this pass) contains that supply's own partition — even
    /// though nothing in THEIR OWN topology changed. Their own trajectory
    /// segment/plan comes out identical (nothing electrical there moved),
    /// but the re-aim's `schedule_crossing`/`plan_step` runs the reader's
    /// [`ThresholdFn`] again, which now sees the freshly-resolved supply
    /// voltage (published into `self.voltages` by the static resolve that
    /// just ran) instead of a stale one, so a crossing this IC's OWN watch
    /// depends on gets correctly re-scheduled instead of staying aimed at
    /// whatever the supply used to be.
    ///
    /// The same holds for a supply in a `Stepped` partition that this pass
    /// re-solves (W1e follow-up, gap 1): its trial is discarded and planned
    /// afresh under the new stamps, and a dependent partition's crossing was
    /// aimed at the discarded one (`stepped`'s supply traces). Such a
    /// dependent is itself re-solved, which can make it a supply in turn, so
    /// this repeats until nothing more joins.
    fn add_watch_dependent_partitions(&self, static_ids: &[PartitionId], cf_ids: &mut Vec<usize>, st_ids: &mut Vec<usize>) {
        #[cfg(test)]
        let (mut reference_cf, mut reference_st) = (cf_ids.clone(), st_ids.clone());
        loop {
            let before = (cf_ids.len(), st_ids.len());
            let supply_touched = |node: usize, st_ids: &[usize]| -> bool {
                let Some((p, _)) = self.local_index[node] else { return false };
                match self.partition_kinds.get(p) {
                    Some(PartitionKind::Static) => static_ids.iter().any(|pid| pid.0 as usize == p),
                    Some(PartitionKind::Stepped) => st_ids.contains(&p),
                    _ => false,
                }
            };
            // Only the ICs `watch_dependents` lists can add anything: any other
            // has no supply in a partition that moves, or no pin to re-aim.
            for (ic_pos, deps) in &self.watch_dependents {
                let ic = &self.ics[*ic_pos as usize].1;
                if !(supply_touched(ic.vcc.node, st_ids) || supply_touched(ic.gnd.node, st_ids)) {
                    continue;
                }
                for &(_, p) in deps {
                    // A `cc_switch` partition's kind is the present one.
                    if self.partition_kinds[p] == PartitionKind::ClosedForm {
                        if !cf_ids.contains(&p) {
                            cf_ids.push(p);
                        }
                    } else if !st_ids.contains(&p) {
                        st_ids.push(p);
                    }
                }
            }
            if (cf_ids.len(), st_ids.len()) == before {
                break;
            }
        }
        #[cfg(test)]
        {
            self.add_watch_dependent_partitions_by_scanning_every_ic(static_ids, &mut reference_cf, &mut reference_st);
            assert_eq!((&reference_cf, &reference_st), (&*cf_ids, &*st_ids), "the dependents list must add the same partitions, in the same order, as the scan of every IC");
        }
    }

    /// What [`Self::add_watch_dependent_partitions`] was before it went
    /// through `watch_dependents`, kept verbatim as the oracle every test-build
    /// call checks it against.
    #[cfg(test)]
    fn add_watch_dependent_partitions_by_scanning_every_ic(&self, static_ids: &[PartitionId], cf_ids: &mut Vec<usize>, st_ids: &mut Vec<usize>) {
        loop {
            let before = (cf_ids.len(), st_ids.len());
            let supply_touched = |node: usize, st_ids: &[usize]| -> bool {
                let Some((p, _)) = self.local_index[node] else { return false };
                match self.partition_kinds.get(p) {
                    Some(PartitionKind::Static) => static_ids.iter().any(|pid| pid.0 as usize == p),
                    Some(PartitionKind::Stepped) => st_ids.contains(&p),
                    _ => false,
                }
            };
            for (_, ic) in &self.ics {
                if !(supply_touched(ic.vcc.node, st_ids) || supply_touched(ic.gnd.node, st_ids)) {
                    continue;
                }
                for (logic_index, pin) in ic.logic.iter().enumerate() {
                    if !partition::is_watched_read_pin(ic, logic_index) {
                        continue;
                    }
                    if let Some((p, _)) = self.local_index[pin.node] {
                        match self.partition_kinds.get(p) {
                            Some(PartitionKind::ClosedForm) if !cf_ids.contains(&p) => cf_ids.push(p),
                            Some(PartitionKind::Stepped) if !st_ids.contains(&p) => st_ids.push(p),
                            _ => {}
                        }
                    }
                }
            }
            if (cf_ids.len(), st_ids.len()) == before {
                break;
            }
        }
    }

    /// Contract stat 0: every partition, of any kind.
    pub fn partition_count(&self) -> usize {
        self.partition_kinds.len()
    }

    /// Contract stat 1: `Static`-classified partitions only — each one
    /// actually gets one dense LU solve at load, exactly as W0's "static
    /// solves" meant before dynamic partitions existed at all. A `ClosedForm`
    /// partition's segment build and a `Stepped` partition's first plan are
    /// their own kind of "solve" but not this one, since it names the SAME
    /// static-partition mechanism the pre-W1b contract already described.
    pub fn static_solve_count(&self) -> usize {
        self.partition_kinds.iter().filter(|k| **k == PartitionKind::Static).count()
    }

    /// Contract stat 2: the largest partition (any kind) by node count.
    pub fn largest_partition(&self) -> usize {
        self.largest_partition
    }

    /// Contract stat 3: every fixed node, node 0 included.
    pub fn fixed_count(&self) -> usize {
        let rail = self.hub.as_ref().map(|h| h.node);
        self.local_index.iter().enumerate().filter(|(n, x)| x.is_none() && Some(*n) != rail).count()
    }

    pub fn multi_capacitor_linear_partitions(&self) -> f64 {
        self.multi_capacitor_linear_partitions
    }

    pub fn closed_form_segments(&self) -> f64 {
        self.closed_form.segments_built()
    }

    pub fn crossings(&self) -> f64 {
        self.closed_form.crossings()
    }

    pub fn stepped_steps(&self) -> f64 {
        self.stepped.steps_taken()
    }

    pub fn newton_iterations(&self) -> f64 {
        self.stepped.newton_iterations() + self.static_solver.newton_iterations()
    }

    /// The crossing-watch API's entry point (W1 plan row B2): registers
    /// `reader` against `thresholds` on `node`, dispatching to whichever
    /// dynamic sub-solver owns it. `false` for a node this solver has no
    /// dynamic-partition record of at all (a fixed node, or a `Static`
    /// partition's — see `closed_form`/`stepped`'s own `watch_node` doc
    /// comment for why only a dynamic partition's node is ever a valid
    /// target).
    pub fn watch_node(&mut self, node: usize, reader: DeviceId, thresholds: ThresholdFn) -> bool {
        match self.local_index.get(node).copied().flatten() {
            Some((p, _)) => match self.partition_kinds.get(p) {
                Some(PartitionKind::ClosedForm) => self.closed_form.watch_node(node, reader, thresholds),
                Some(PartitionKind::Stepped) => self.stepped.watch_node(node, reader, thresholds),
                _ => false,
            },
            None => false,
        }
    }

    /// Brings the shared arrays up to date with what the static solver just
    /// resolved: the nodes whose bits moved and the devices whose current it
    /// recomputed, and nothing else. That is all that can differ: the static
    /// solver's node voltages and its devices' currents are, bit for bit, what
    /// the shared arrays hold after every publish (they start equal, and both
    /// are rolled back together), so a slot the resolve did not change
    /// already agrees. The debug build checks that claim on every call.
    fn copy_in_static(&mut self) -> Result<(), String> {
        // F5b: a hold the static solver took or released (W1c point 1a);
        // past ruling R51's guard the call fails once all is copied.
        let held = self.take_static_holds();
        let static_voltages = self.static_solver.voltages();
        for &node in self.static_solver.changed_nodes() {
            let new = static_voltages[node];
            let old_bits = self.voltages[node].to_bits();
            if new.to_bits() != old_bits {
                if self.window {
                    self.v_log.push((node as u32, old_bits));
                }
                self.voltages[node] = new;
                // W1g point 2: what this node's move concerns the settle with.
                settle::note_static_node(&mut self.slow_scratch, &self.slow_model, node);
            }
            fold_bound(&mut self.v_hint, new);
        }
        let static_currents = self.static_solver.currents();
        for d in self.static_solver.recomputed_devices() {
            let new = static_currents[d];
            let old_bits = self.currents[d].to_bits();
            if new.to_bits() != old_bits {
                if self.window {
                    self.i_log.push((d as u32, old_bits));
                }
                self.currents[d] = new;
            }
        }
        #[cfg(debug_assertions)]
        {
            for &node in &self.static_nodes {
                debug_assert_eq!(self.voltages[node].to_bits(), static_voltages[node].to_bits(), "node {node}: the shared voltage drifted from the static solver's");
            }
            for &d in &self.static_kcl_devices {
                debug_assert_eq!(self.currents[d].to_bits(), static_currents[d].to_bits(), "device {d}: the shared current drifted from the static solver's");
            }
        }
        held
    }

    /// Names the dynamic partitions the coming call may write: `cf` (global indices) or every closed-form
    /// partition, and every stepped one when `stepped` (a stepped call can
    /// bring the partitions that supply it along). Their published slots are
    /// logged for the window before the call writes them, and
    /// [`Self::end_dynamic_writes`] folds what they then hold into the bounds.
    fn begin_dynamic_writes(&mut self, cf: &[usize], all_cf: bool, stepped: bool) {
        self.written.clear();
        if all_cf {
            self.written.extend(self.cf_parts.iter().map(|&p| p as u32));
        } else {
            self.written.extend(cf.iter().map(|&p| p as u32));
        }
        if stepped {
            self.written.extend(self.stepped_parts.iter().map(|&p| p as u32));
        }
        self.log_written();
        self.note_written_published(0);
    }

    /// [`Self::begin_dynamic_writes`] for a list of dynamic partitions, added
    /// to the ones already named in this call (the rail loop writes more
    /// after the call's own resolves, W1f).
    fn begin_dynamic_writes_list(&mut self, parts: &[usize]) {
        let from = self.written.len();
        self.written.extend(parts.iter().map(|&p| p as u32));
        self.log_written();
        self.note_written_published(from);
    }

    fn log_written(&mut self) {
        if self.window {
            for i in 0..self.written.len() {
                let p = self.written[i] as usize;
                if !self.dyn_logged[p] {
                    self.dyn_logged[p] = true;
                    self.dyn_logged_list.push(p as u32);
                    for &n in &self.dyn_out[p].nodes {
                        self.v_log.push((n as u32, self.voltages[n].to_bits()));
                    }
                    for &d in &self.dyn_out[p].devices {
                        self.i_log.push((d as u32, self.currents[d].to_bits()));
                    }
                }
            }
        }
    }

    /// Folds the voltages and capacitor currents the dynamic partitions named
    /// by the last [`Self::begin_dynamic_writes`] hold into the bounds.
    fn end_dynamic_writes(&mut self) {
        for &p in &self.written {
            let out = &self.dyn_out[p as usize];
            for &n in &out.nodes {
                fold_bound(&mut self.v_hint, self.voltages[n]);
            }
            for &d in &out.caps {
                fold_bound(&mut self.i_hint, self.currents[d]);
            }
        }
    }

    /// The exact maxima of `|v|` over every voltage and `|i|` over every
    /// partition-published capacitor current, by a full scan (a NaN is
    /// infinite here).
    fn rescan_bounds(&mut self) {
        (self.v_hint, self.i_hint) = self.scan_bounds();
    }

    fn scan_bounds(&self) -> (f64, f64) {
        let (mut v, mut i) = (0.0, 0.0);
        for &x in &self.voltages {
            fold_bound(&mut v, x);
        }
        for &d in &self.partition_cap_devices {
            fold_bound(&mut i, self.currents[d]);
        }
        (v, i)
    }

    /// Publishes every [`FixedPairCapacitor`]'s current at `now`
    /// (core/README.md's rule 2a: "publishes... at every advance, horizon
    /// clamp and set_params instant"). The eager form ran unconditionally
    /// from all three of those points ([`PartitionSolver::resolve_dirty`],
    /// `handle_dynamic_event`, `sync_dynamic_to`); it now runs inside
    /// [`Self::compute_derived`], at each of them or on the first read after
    /// them, with the `now` of the last one — mirroring
    /// [`Self::recompute_source_currents`]'s own "recomputed unconditionally"
    /// contract just below, and for the same reason: this device belongs to
    /// no partition, so nothing ever marks it "dirty" the way a driver commit
    /// marks a real partition; recomputing it fresh from `now` on every call
    /// is what stands in for that. MUST run before
    /// `recompute_source_currents`, which folds this method's freshly
    /// published `currents[device_index]` into the KCL of whichever source(s)
    /// fixed this capacitor's two nodes (`self.cap_devices` already lists it,
    /// core/README.md rule 2a's own last bullet).
    fn publish_fixed_pair_capacitors(&self, currents: &mut [f64], now: f64) {
        for cap in &self.fixed_pair_capacitors {
            currents[cap.device_index] = cap.current_at(now);
        }
        // While a PSU with a rail capacitor is in CC, each rail capacitor
        // carries its share `C dq/dt` of the capacitors' current, `q` their
        // charge in the rail's closed form (Rails point 4).
        if let Some(hub) = self.hub.as_ref() {
            if let Some(seg) = &hub.state.cap_seg {
                let dq = seg.charge_slope_at(now);
                for &(k, _) in &hub.caps {
                    let cap = &self.fixed_pair_capacitors[k as usize];
                    let sign = if cap.nodes.0 == hub.node { 1.0 } else { -1.0 };
                    currents[cap.device_index] = sign * cap.c * dq + cap.leak_g * (self.voltages[cap.nodes.0] - self.voltages[cap.nodes.1]);
                }
            }
        }
    }

    /// The authoritative `voltage_source` current computation for the WHOLE
    /// program (this module's doc comment): rebuilds the KCL `leaving`
    /// array from every resistor/switch contact (`devices::stamp`, live
    /// params, cached in [`KclDevice::contributions`]) plus every capacitor's
    /// ALREADY-PUBLISHED total current, then
    /// walks `resolved` children-before-parents exactly as
    /// `static_part::StaticPartitionSolver::recompute_currents` does for a
    /// purely static program. Always run, even with nothing dirty this call
    /// (a live-parameter change elsewhere can still move a source's
    /// current), mirroring that method's own "recomputed unconditionally"
    /// contract. `now` and `drivers` are only for the W1c fold below — a
    /// call with nothing IC-relevant to say (`handle_dynamic_event`/
    /// `sync_dynamic_to`, since an IC can never touch a dynamic partition
    /// this step) simply passes `&[]` and whatever `now`/`t_end` it already
    /// has; every OTHER driver/pull site (this partition's own static solve)
    /// is unaffected, since `self.ics` alone decides what there is to fold.
    ///
    /// W1c contract point 4's last bullet ("fold driver and pull currents
    /// into the source KCL at fixed nodes... IC devices publish elementI
    /// 0"): an IC output driver or floating pull is not in `self.kcl_devices`
    /// at all (only a resistor/switch is), so without this it would
    /// contribute NOTHING to a source's published current even when its own
    /// pin sits on a fixed rail (a driver `partition_of_node` reports `None`
    /// for) and draws real current from it. `self.voltages` is already the
    /// fully-published, POST-solve array by the time this runs (every call
    /// site calls it last), so both `ic_driver_branches`/`ic_pull_target`'s
    /// power computation and the actual `g*(Va-Vb)` current read off
    /// already-final values, exactly like every other term in `leaving`
    /// above them.
    fn recompute_source_currents(&self, currents: &mut [f64], scratch: &mut (Vec<f64>, Vec<f64>), now: f64, drivers: &[Driver]) {
        // Scratch (W2a): empty between uses, so each starts zeroed here, as
        // the `fill` it replaces did on a buffer kept at node-count length.
        let (leaving, extra) = scratch;
        leaving.resize(self.voltages.len(), 0.0);
        extra.resize(self.voltages.len(), 0.0);
        for kd in &self.kcl_devices {
            for &(u, w, conductance) in &kd.contributions {
                let i = conductance * (self.voltages[u] - self.voltages[w]);
                leaving[u] += i;
                leaving[w] -= i;
            }
        }
        for (node, v) in self.voltages.iter().enumerate().skip(1) {
            leaving[node] += static_part::NODE_RSHUNT_G * v;
        }
        for &(d, u, w) in &self.cap_devices {
            let i = currents[d];
            leaving[u] += i;
            leaving[w] -= i;
        }
        // W1e, LEDs point 12: every stamped junction's branch current, anode
        // to cathode, whether it sits in a partition or across two fixed
        // nodes. The datum end feeds no source (`leaving[0]` is never read).
        for j in &self.junctions {
            // A failed LED stamps no junction at all, GMIN included (W1g
            // point 12).
            if self.slow.may_have_failed() && self.slow.is_failed(j.device) {
                continue;
            }
            let i = junction_branch_current(j, &self.voltages);
            leaving[j.anode] += i;
            leaving[j.cathode] -= i;
        }
        for (pos, (device_index, ic)) in self.ics.iter().enumerate() {
            let power = digital::power::ic_power_held(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node], self.power_hold(*device_index));
            if !power.powered {
                continue;
            }
            for &id in &self.ic_driver_ids[pos] {
                let driver = &drivers[id as usize];
                for branch in ic_driver_branches(&power, driver, ic) {
                    let Some((target, g)) = branch else { continue };
                    let i = g * (self.voltages[driver.node] - self.voltages[target]);
                    leaving[driver.node] += i;
                    leaving[target] -= i;
                }
            }
            for pin in &ic.logic {
                if let Some((g, target_v)) = ic_pull_target(ic, &power, pin, now) {
                    // Into the datum (node 0), not `ic.vcc.node`/`ic.gnd.node`
                    // (`ic_pull_target`'s own doc comment: this is the fix).
                    // `leaving[0]` is never read by the `resolved` walk below
                    // (node 0 is that walk's root, never a `child_node`), so
                    // the symmetric term is simply omitted rather than added
                    // to a slot nothing consumes — unlike the driver loop
                    // above, whose `target` is a real, possibly-fixed node.
                    let i = g * (self.voltages[pin.node] - target_v);
                    leaving[pin.node] += i;
                }
            }
        }
        let cc_hub = self.cc_hub_current();
        for r in self.resolved.iter().rev() {
            let total = leaving[r.child_node] + extra[r.child_node];
            let child_is_pos = !r.parent_is_pos;
            let mut i_s = if child_is_pos { -total } else { total };
            // A PSU in CC publishes `-dir iLimit` exactly (Rails point 5).
            if let Some((device, i)) = cc_hub {
                if device == r.device_index {
                    i_s = i;
                }
            }
            currents[r.device_index] = i_s;
            let contribution = if r.parent_is_pos { i_s } else { -i_s };
            extra[r.parent_node] += contribution;
        }
    }

    /// `(device, -dir iLimit)` while the hub PSU is in CC.
    fn cc_hub_current(&self) -> Option<(usize, f64)> {
        use crate::devices::sources::{direction, Regime};
        use crate::rails::HubKind;
        if let Some(hub) = self.hub.as_ref() {
            if let (HubKind::Psu { v, i_limit }, Regime::Cc) = (&hub.kind, hub.state.reg) {
                return Some((hub.device, -direction(*v) * i_limit));
            }
        }
        if let Some(e) = self.exact.as_ref() {
            if let (HubKind::Psu { v, i_limit }, Regime::Cc) = (&e.kind, e.reg) {
                return Some((e.device, -direction(*v) * i_limit));
            }
        }
        None
    }

    /// NE555 point 8 / C11: publishes each 555's `elementI`, the current
    /// delivered out of pin 3, in the global post-pass beside
    /// [`Self::recompute_source_currents`] (every resolve, crossing, step and
    /// horizon clamp), from the committed OUT level and the published
    /// voltages, so it follows a moving supply or load. 0 while unpowered or
    /// while OUT has never committed. A 555 is not in `kcl_devices` and no
    /// other pass writes its `currents` slot, so this is its only writer.
    fn publish_ne555_currents(&self, currents: &mut [f64], drivers: &[Driver]) {
        for (pos, (device_index, ic)) in self.ics.iter().enumerate() {
            if !digital::is_ne555(ic.kind) {
                continue;
            }
            let power = digital::power::ic_power_held(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node], self.power_hold(*device_index));
            let committed = self.ic_driver_ids[pos].iter().map(|&id| &drivers[id as usize]).find(|d| d.stage == DriverStage::Ne555Out).and_then(|d| d.committed());
            currents[*device_index] = match (power.powered, committed) {
                (true, Some(level @ (Mode::High | Mode::Low))) => digital::ne555::element_current(
                    level == Mode::High,
                    power.vs,
                    self.voltages[ic.logic[digital::NE555_OUT].node],
                    self.voltages[ic.gnd.node],
                    self.voltages[ic.vcc.node],
                ),
                _ => 0.0,
            };
        }
    }

    /// The LED device pass (README, LEDs point 13): each LED-family device's
    /// `elementI` and channel values, from the published voltages and its
    /// `burnt` flag, into `currents` and the channel array. Run beside
    /// [`Self::recompute_source_currents`] by [`Self::compute_derived`]. A
    /// pure function of those voltages, so the same numbers come out however
    /// often it runs, which is what lets the whole readout pass wait for the
    /// first read (`publish_readout`) instead of memoising it per device.
    fn publish_led_devices(&self, currents: &mut [f64], channels: &mut [f64]) {
        for e in &self.led_devices {
            let mut element_i = 0.0;
            let own = &mut channels[e.channel_base..e.channel_base + e.channel_count];
            // W1g point 12: the real `led_failed` flag, which takes `burnt`'s
            // path (`elementI` 0, `NaN` channels).
            led::readout(&e.device, &e.junctions, &self.voltages, self.slow.is_failed(e.device_index), &mut element_i, own);
            currents[e.device_index] = element_i;
        }
    }

    /// Re-derives `ic_driver_ids` when the driver table's length is not the one
    /// they were derived for (a debug build re-derives and compares on every
    /// call, like the static solver's own driver index).
    fn ensure_ic_driver_ids(&mut self, drivers: &[Driver]) {
        let derive = |ic_position: &[u32], ics: usize| -> Vec<Vec<u32>> {
            let mut ids = vec![Vec::new(); ics];
            for (id, d) in drivers.iter().enumerate() {
                if let Some(&pos) = ic_position.get(d.device) {
                    if pos != u32::MAX {
                        ids[pos as usize].push(id as u32);
                    }
                }
            }
            ids
        };
        if self.ic_driver_ids_len != drivers.len() {
            self.ic_driver_ids = (derive(&self.ic_position, self.ics.len())).into();
            self.ic_driver_ids_len = drivers.len();
        }
        #[cfg(debug_assertions)]
        debug_assert!(derive(&self.ic_position, self.ics.len()) == self.ic_driver_ids, "the driver table changed without changing its length");
    }

    /// The four readout passes, in the order the eager publish ran them: the
    /// fixed-pair capacitors' currents (which the KCL fold reads), the
    /// sources', the 555s' and the LED devices'. The arrays they write are
    /// taken out of `self` for the call, because the passes read the rest of
    /// it.
    fn compute_derived(&mut self, now: f64) {
        // Inside a window this is the eager path (the deferred one runs at
        // `prepare_readout`, between windows): its writes need logging.
        if self.window && !self.derived_logged {
            self.derived_logged = true;
            for &d in &self.derived_devices {
                self.i_log.push((d as u32, self.currents[d].to_bits()));
            }
            self.w_channels.clone_from(&self.channels);
        }
        let drivers = Rc::clone(&self.driver_snapshot);
        self.ensure_ic_driver_ids(&drivers);
        let mut currents = std::mem::take(&mut self.currents);
        let mut channels = std::mem::take(&mut self.channels);
        let mut scratch = (self.kcl_scratch.0.take(), self.kcl_scratch.1.take());
        self.publish_fixed_pair_capacitors(&mut currents, now);
        self.recompute_source_currents(&mut currents, &mut scratch, now, &drivers);
        self.publish_ne555_currents(&mut currents, &drivers);
        self.publish_led_devices(&mut currents, &mut channels);
        self.currents = currents;
        self.channels = channels;
        self.kcl_scratch.0.put(scratch.0);
        self.kcl_scratch.1.put(scratch.1);
    }

    /// The tail every publish point ends its readout with: the eager passes
    /// (`compute_derived`) when a derived value could be non-finite, else
    /// only a note that they are owed.
    ///
    /// Why deferring cannot change a result: the passes are pure functions of
    /// the published state at the publish point, and nothing changes that
    /// state without another publish, or a rollback that restores it and the
    /// flag together. The one thing an eager pass can do that a deferred one
    /// cannot is make `check_finite` fail the instant on a derived value that
    /// overflowed to infinity or NaN. [`Self::lazy_publish_ok`] proves that
    /// cannot happen, or the pass runs here, exactly as before.
    ///
    /// Returns whether the passes were deferred, which also means every node
    /// voltage was found finite (`check_finite` need not scan them again).
    fn publish_readout(&mut self, now: f64) -> bool {
        if self.lazy_publish_ok(now) {
            self.derived_now = now;
            self.derived_dirty = true;
            true
        } else {
            self.compute_derived(now);
            self.derived_dirty = false;
            false
        }
    }

    /// Whether every value `compute_derived` would produce is provably
    /// finite, so `check_finite` could not tell a deferred publish from an
    /// eager one. Sufficient conditions, all cheap:
    /// - `lazy_static_ok`: every coefficient the passes multiply by is
    ///   bounded (`compute_lazy_static_ok`);
    /// - every node voltage is at most [`LAZY_V_LIMIT`] in magnitude (a NaN
    ///   or infinite one fails the test, and `check_finite` then reports it
    ///   exactly as before, after the eager passes ran);
    /// - every partition-published capacitor current is at most
    ///   [`LAZY_I_LIMIT`] (both are read off `v_hint` and `i_hint`, which every
    ///   write keeps covering, not off a scan of the arrays);
    /// - `now >= start_time`, so a fixed-pair capacitor's exponential is at
    ///   most 1.
    /// A term is then at most `max(G * 2 * V, I)` and there are at most
    /// [`LAZY_TERM_LIMIT`] of them, far inside the f64 range. Failing any
    /// of these merely selects the eager path.
    fn lazy_publish_ok(&mut self, now: f64) -> bool {
        if !self.lazy_static_ok || !(now >= self.start_time) {
            return false;
        }
        // The claim the bounds rest on, checked on every call of a debug
        // build: no write has left a value above them.
        #[cfg(debug_assertions)]
        {
            let (v, i) = self.scan_bounds();
            debug_assert!(v <= self.v_hint && i <= self.i_hint, "a write left |v| = {v} or |i| = {i} above the bounds {} and {}", self.v_hint, self.i_hint);
        }
        if !(self.v_hint <= LAZY_V_LIMIT && self.i_hint <= LAZY_I_LIMIT) {
            // The bounds only ever over-estimate (they cover every write and
            // are lifted by a rollback), so this is the scan that finds out
            // whether the values really exceed the limits, and leaves them
            // exact.
            self.rescan_bounds();
        }
        self.v_hint <= LAZY_V_LIMIT && self.i_hint <= LAZY_I_LIMIT
    }

    /// The load-time half of [`Self::lazy_publish_ok`]: whether every
    /// coefficient is bounded. Conductances of resistors are `1/R`
    /// (`R > 0` is all `kinds` checks, so `1/R` can overflow); a switch's
    /// is `0` or `1/0.001` whatever its live params (`switches::stamp`);
    /// an IC driver's, a pull's and a 555's are constants or `1/r_out` with
    /// `r_out >= 50` (`digital::power`); a junction's current is bounded by
    /// its law; a fixed-pair capacitor's by its constants.
    fn compute_lazy_static_ok(&self, term_count: usize) -> bool {
        if term_count > LAZY_TERM_LIMIT {
            return false;
        }
        for kd in &self.kcl_devices {
            if !kd.contributions.iter().all(|&(_, _, g)| g.abs() <= LAZY_G_LIMIT) {
                return false;
            }
        }
        for e in &self.led_devices {
            if !e.junctions.iter().all(|j| junction_current_bounded(&j.law)) {
                return false;
            }
        }
        self.fixed_pair_capacitors.iter().all(|cap| {
            // A capacitor on a live node re-aims at any `dV` the bounded
            // voltages allow, from any charge it held (Rails point 7).
            let live = self.fixed_node.len() > cap.nodes.0.max(cap.nodes.1) && (self.node_caps.get(cap.nodes.0).is_some_and(|v| !v.is_empty()) || self.node_caps.get(cap.nodes.1).is_some_and(|v| !v.is_empty()));
            let series_bound = if cap.esr > 0.0 { if live { 4.0 * LAZY_V_LIMIT / cap.esr } else { (cap.d_v / cap.esr).abs() } } else { 0.0 };
            let leak_term = (cap.leak_g * cap.d_v).abs();
            // The exponent is `-(now - start) / (esr * C)`, at most 0 for
            // `now >= start` and never NaN when `esr * C` is a positive
            // finite number.
            let time_constant_ok = cap.esr == 0.0 || (cap.esr * cap.c > 0.0 && (cap.esr * cap.c).is_finite());
            cap.d_v.abs() <= 2.0 * LAZY_V_LIMIT && series_bound <= LAZY_I_LIMIT && leak_term <= LAZY_I_LIMIT && time_constant_ok
        })
    }

    /// Rebuilds the cached KCL contributions of every switch-family device in
    /// `triples`, after `apply_params` wrote their live params.
    fn refresh_kcl_contributions(&mut self, triples: &[(usize, usize, f64)]) {
        for &(device_index, _, _) in triples {
            let (Some(&pos), Some(params)) = (self.kcl_position.get(&device_index), self.live_params.get(&device_index)) else {
                continue;
            };
            let kd = &mut self.kcl_devices[pos];
            kd.contributions = KclDevice::resolve_contributions(kd.kind, &kd.nodes, params);
        }
    }

    fn drain_scheduled(&mut self) {
        for (t, p, v) in self.closed_form.drain_scheduled() {
            self.pending_schedule.push((t, EventKind::Crossing { partition: PartitionId(p as u32), version: v }));
        }
        for (t, p, v) in self.stepped.drain_scheduled() {
            self.pending_schedule.push((t, EventKind::Step { partition: PartitionId(p as u32), version: v }));
        }
    }

    /// W1c contract point 5/6: schedules a floating-bucket boundary at
    /// `digital::floating::next_bucket_boundary(now)` if any powered IC
    /// currently needs one and none is already pending (`self.next_bucket`,
    /// the dedup flag) — called after every point that just published fresh
    /// voltages ([`PartitionSolver::resolve_dirty`], `handle_dynamic_event`,
    /// `sync_dynamic_to`), and again from [`Self::bucket_boundary`] itself so
    /// the chain keeps going even on a delta whose `bucket_boundary` call
    /// returns no partitions to re-solve (nothing would otherwise call
    /// `resolve_dirty`, and with it this method, that same delta).
    fn maybe_schedule_bucket(&mut self, now: f64) {
        if self.next_bucket.is_some() {
            return;
        }
        if !any_powered_ic_needs_bucket_watch(&self.ics, &self.voltages, self.power_holds_slice()) {
            return;
        }
        let boundary = digital::floating::next_bucket_boundary(now);
        self.next_bucket = Some(boundary);
        self.pending_schedule.push((boundary, EventKind::Bucket));
    }

    /// [`Self::check_finite`] for a deferred publish, which changed no derived
    /// slot and no channel, and left every voltage within `LAZY_V_LIMIT`: the
    /// only values that can be non-finite are the currents the dynamic
    /// partitions just wrote (the static solver checks its own, and every
    /// other slot passed the check when it was written or is a snapshot of
    /// such a value). One that is sends the call to the full scan, so the
    /// error names the same device it always did.
    fn check_finite_written(&self) -> Result<(), String> {
        for &p in &self.written {
            for &d in &self.dyn_out[p as usize].devices {
                if !self.currents[d].is_finite() {
                    return self.check_finite(true);
                }
            }
        }
        Ok(())
    }

    fn check_finite(&self, voltages_known_finite: bool) -> Result<(), String> {
        if !voltages_known_finite {
            if let Some((node, v)) = self.voltages.iter().enumerate().find(|(_, v)| !v.is_finite()) {
                return Err(format!("node {node}: published voltage {v} is not finite"));
            }
        }
        if let Some((device, i)) = self.currents.iter().enumerate().find(|(_, i)| !i.is_finite()) {
            return Err(format!("device {device}: published current {i} is not finite"));
        }
        // A `NaN` channel is an absent entry (README, Readouts), legitimate
        // only on a device that is absent; any other non-finite channel is a
        // failed solve.
        for e in &self.led_devices {
            // Absent: `burnt`, or W1g's `led_failed` (README W1g point 12).
            let absent = e.absent() || self.slow.is_failed(e.device_index);
            for (c, v) in self.channels[e.channel_base..e.channel_base + e.channel_count].iter().enumerate() {
                if !v.is_finite() && !(v.is_nan() && absent) {
                    return Err(format!("device {}: published channel {c} value {v} is not finite", e.device_index));
                }
            }
        }
        Ok(())
    }
}

impl PartitionedSolver {
    /// Undoes the logs on copies of the shared arrays, newest first, and
    /// checks they come out as the arrays the window found (test builds only).
    #[cfg(test)]
    fn check_logs_reproduce_the_window_start(&mut self) {
        let Some((v0, i0, c0)) = self.w_saved.take() else { return };
        let (mut v, mut i) = (self.voltages.clone(), self.currents.clone());
        for &(node, bits) in self.v_log.iter().rev() {
            v[node as usize] = f64::from_bits(bits);
        }
        for &(device, bits) in self.i_log.iter().rev() {
            i[device as usize] = f64::from_bits(bits);
        }
        // The channels are saved whole, the first time the eager pass writes
        // them; a window that never ran it did not write them.
        let c = if self.derived_logged { self.w_channels.clone() } else { self.channels.clone() };
        for (n, (a, b)) in v.iter().zip(&v0).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "node {n}: voltage changed in the window without being logged");
        }
        for (d, (a, b)) in i.iter().zip(&i0).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "device {d}: current changed in the window without being logged");
        }
        for (k, (a, b)) in c.iter().zip(&c0).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "channel {k}: changed in the window without being logged");
        }
    }

    /// Starts relying on the kernel's `driver_changing` notifications once it
    /// has sent one and no window is open (a window that began without them
    /// keeps rolling the table back its own way to its end).
    fn activate_mirror_if_seen(&mut self) {
        if !self.window && self.mirror_seen && !self.mirror_live {
            self.mirror_live = true;
            self.mirror_valid = false;
            self.stale.clear();
        }
    }

    /// Brings `driver_snapshot` up to the kernel's table `drivers`, as it
    /// stood at this resolve. Returns whether that was done in place from the
    /// entries the kernel reported changed, in which case the table it
    /// replaced is only recoverable from `call_patches`
    /// ([`Self::rebuild_pre_commit_table`]); otherwise the whole table was
    /// copied and the one it replaced is `pre_commit_drivers`.
    ///
    /// The whole copy is a swap with `pre_commit_drivers`: the old table is
    /// only needed for this one call, and swapping leaves the older buffer to
    /// be overwritten in place with `drivers`. That buffer is still shared
    /// only when an instant's checkpoint holds it (a second resolve within the
    /// instant), and then a copy is the price.
    ///
    /// Finding 5 (W1b rule 6, amended): the replaced table is what
    /// `stepped.resolve_dirty` hands its pre-`now` catch-up
    /// (`SteppedSolver::sync_partition`), the one that was actually in force for
    /// that whole span, not the one THIS commit just produced.
    fn refresh_driver_snapshot(&mut self, drivers: &[Driver]) -> bool {
        self.activate_mirror_if_seen();
        self.call_patches.clear();
        // A resolve handed this solver's own table (the slow republish,
        // `settle.rs`'s `republish_fixed`) has nothing to refresh from: it is
        // the table as of the last resolve, without the commits the kernel
        // announced since. Draining `stale` against it would mark those as
        // delivered, and the solver would keep their old levels until each
        // changed again (cold 64 tpl 5, hub 2). In place with no patches,
        // so a stepped catch-up in this call sees this same table as the
        // one in force before it.
        if std::ptr::eq(drivers.as_ptr(), self.driver_snapshot.as_ptr()) && drivers.len() == self.driver_snapshot.len() {
            return true;
        }
        let in_place = self.mirror_live
            && self.mirror_valid
            && self.driver_snapshot.len() == drivers.len()
            && (!self.window || self.window_mirror);
        if in_place {
            // `make_mut`, not `get_mut`: a table something else holds (a
            // `dynamic_snapshot()` taken since the last refresh) must not
            // change under its holder, so it is cloned first, and that copy
            // plus the stale entries is the kernel's current table all the
            // same. When nothing holds it, as in every run today, this is the
            // in-place write and costs nothing extra.
            let table = Rc::make_mut(&mut self.driver_snapshot);
            for idx in self.stale.drain(..) {
                let old = table[idx as usize];
                self.call_patches.push((idx, old));
                if self.window {
                    self.d_log.push((idx, old));
                }
                table[idx as usize] = drivers[idx as usize];
            }
            #[cfg(debug_assertions)]
            for (id, (mine, theirs)) in table.iter().zip(drivers).enumerate() {
                debug_assert!(same_driver(mine, theirs), "driver {id} changed without `driver_changing` being called first");
            }
            return true;
        }
        if self.window && self.window_mirror && self.w_driver_snapshot.is_none() {
            // The first whole copy of a window that rolls the table back
            // through `d_log`: hold the table it replaces, and remember which
            // entries of `d_log` are about that one.
            self.w_driver_snapshot = Some(Rc::clone(&self.driver_snapshot));
            self.d_log_split = self.d_log.len();
        }
        std::mem::swap(&mut self.driver_snapshot, &mut self.pre_commit_drivers);
        match Rc::get_mut(&mut self.driver_snapshot) {
            Some(table) => {
                table.clear();
                table.extend_from_slice(drivers);
            }
            None => self.driver_snapshot = Rc::new(drivers.to_vec()),
        }
        self.stale.clear();
        self.mirror_valid = self.mirror_live;
        false
    }

    /// The table `driver_snapshot` held before this resolve refreshed it in
    /// place: the current one with the refreshed entries put back.
    fn rebuild_pre_commit_table(&mut self) {
        if Rc::get_mut(&mut self.pre_commit_drivers).is_none() {
            self.pre_commit_drivers = Rc::new(Vec::new());
        }
        let pre = Rc::get_mut(&mut self.pre_commit_drivers).expect("just made unique");
        pre.clear();
        pre.extend_from_slice(&self.driver_snapshot);
        for &(idx, old) in self.call_patches.iter().rev() {
            pre[idx as usize] = old;
        }
    }

    /// Drops the solver-level window state (the sub-solvers close their own).
    /// The hold table as the verdict reads it: empty while nothing is held.
    #[inline]
    pub(crate) fn power_holds_slice(&self) -> &[digital::power::PowerHold] {
        if self.holds_active == 0 {
            &[]
        } else {
            &self.holds
        }
    }

    /// The hold of the IC at device index `device` (free for any other).
    #[inline]
    pub(crate) fn power_hold(&self, device: usize) -> digital::power::PowerHold {
        digital::power::hold_of(self.power_holds_slice(), device)
    }

    /// Sets the hold of the IC at device index `device`, journaled in an
    /// open window, and returns the partitions its stamps touch (its supply,
    /// pin and driver nodes'), which the caller re-solves: the hold changes
    /// what they stamp. A transition to `Off` counts a hold, `Off` to free a
    /// release.
    #[cfg_attr(not(test), allow(dead_code, reason = "the hold procedure (F5b) is the first caller outside the tests"))]
    pub(crate) fn set_power_hold(&mut self, device: usize, hold: digital::power::PowerHold) -> Vec<PartitionId> {
        use digital::power::HoldKind;
        let old = self.power_hold(device);
        if old.kind == HoldKind::Off && hold.is_free() {
            self.hold_counts[1] += 1;
        } else if hold.kind == HoldKind::Off && old.kind != HoldKind::Off {
            self.hold_counts[0] += 1;
        }
        self.write_power_hold(device, hold)
    }

    /// Drops every hold, uncounted (the circuit itself changed: W1c point
    /// 1a), returning the partitions to re-solve.
    fn release_every_hold(&mut self) -> (Vec<PartitionId>, Vec<DeviceId>) {
        let (mut dirty, mut readers) = (Vec::new(), Vec::new());
        if !self.episodes.is_empty() {
            self.journal_holds();
            self.episodes = Rc::new(Vec::new());
        }
        if self.holds_active == 0 {
            return (dirty, readers);
        }
        for device in 0..self.holds.len() {
            if !self.holds[device].is_free() {
                dirty.extend(self.write_power_hold(device, digital::power::PowerHold::FREE));
                readers.push(DeviceId(device as u32));
            }
        }
        dirty.sort_unstable();
        dirty.dedup();
        (dirty, readers)
    }

    /// Journals the hold table, `holds_active`, the episodes and the retry's
    /// arming at their first touch in an open window (Rails point 8).
    fn journal_holds(&mut self) {
        if self.window && self.w_holds.is_none() {
            self.w_holds = Some((Rc::clone(&self.holds), self.holds_active, Rc::clone(&self.episodes), self.retry_armed));
        }
    }

    /// The horizon clamp's static retry (ruling R51 (b)): the iterated
    /// partitions holding an IC held off solve again, each retrying it,
    /// unless this host step retried already.
    fn retry_static_holds(&mut self, t_end: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>) -> Result<(), String> {
        if !self.at_clamp {
            return Ok(());
        }
        #[cfg(test)]
        if hold::RetrySeam::get() == hold::RetrySeam::NoRetry {
            return Ok(());
        }
        // The static solver numbers every partition, its own and the rest.
        let mut held = self.static_solver.held_partitions(t_end);
        held.retain(|p| self.partition_kinds[p.0 as usize] == PartitionKind::Static);
        // Re-solved only where the retry changes something: a forced re-solve
        // that frees nothing moved the partition by ulps (F5V2 finding 5).
        held.retain(|p| self.static_solver.clamp_retry_changes(p.0 as usize, t_end, drivers));
        if held.is_empty() {
            return Ok(());
        }
        self.static_solver.set_inputs_bounded(false);
        // In exact mode the rail sits in a static partition: a retry that
        // re-solves it is a solve of the rail's partition, so the regime rules
        // run on it (`exact_after_publish`, from the clamp's rail loop). Without
        // them a chip freed in a CV probe, where the rail cannot move, stayed
        // free with the PSU past its limit (cold 621 tpl 2: 126 mA against
        // 99.1 mA for 60 ms of the fine reference).
        if let Some(e) = self.exact.as_ref() {
            if held.iter().any(|p| p.0 as usize == e.partition) {
                self.exact_solved = true;
            }
        }
        self.static_solver.resolve_dirty_into(&held, t_end, drivers, readers)?;
        self.copy_in_static()
    }

    /// Arms the next host step's rail retry (the horizon clamp ended this
    /// one), journaled.
    fn arm_retry(&mut self) {
        if !self.retry_armed {
            // The clamp arms after its own retries, so the journal matters
            // only when the clamp then fails (its ramp re-solve or delta
            // pass): rolled back, the flag must read as the step left it, or
            // asking again would retry at the clamp where the declined call
            // did not. No board in the tests reaches a clamp failing after
            // a retry was taken in its step (F5V2 finding 6, mutation 8).
            self.journal_holds();
            self.retry_armed = true;
        }
    }

    /// Takes this host step's rail retry, journaled.
    pub(super) fn take_retry(&mut self) {
        if self.retry_armed {
            self.journal_holds();
            self.retry_armed = false;
        }
    }

    /// IC `device`'s sliding episode (ruling R51's guard).
    pub(crate) fn episode(&self, device: usize) -> hold::Episode {
        self.episodes.get(device).copied().unwrap_or_default()
    }

    /// Sets IC `device`'s episode, journaled; `Err` (the instant declines,
    /// `-4`) once it passes [`hold::EPISODE_MAX`] holds.
    pub(crate) fn set_episode(&mut self, device: usize, episode: hold::Episode) -> Result<(), String> {
        // Never the window's first touch today: every episode change follows
        // a write of the same IC's hold in the same instant (a probe, the
        // held solve or the release), which journals the episodes with the
        // table, so no test can see this call (F5V2 finding 6, mutation 6).
        // Kept so an episode change can never escape the journal.
        self.journal_holds();
        let n = self.device_kinds.len();
        let table = Rc::make_mut(&mut self.episodes);
        if table.len() < n {
            table.resize(n, hold::Episode::default());
        }
        table[device] = episode;
        episode.check(device)
    }

    fn write_power_hold(&mut self, device: usize, hold: digital::power::PowerHold) -> Vec<PartitionId> {
        let pos = self.ic_position.get(device).copied().filter(|&i| i != u32::MAX);
        let Some(pos) = pos else {
            debug_assert!(hold.is_free(), "only an IC takes a power hold");
            return Vec::new();
        };
        let old = self.power_hold(device);
        if old == hold {
            return Vec::new();
        }
        self.journal_holds();
        let n = self.device_kinds.len();
        let table = Rc::make_mut(&mut self.holds);
        if table.len() < n {
            table.resize(n, digital::power::PowerHold::FREE);
        }
        table[device] = hold;
        match (old.is_free(), hold.is_free()) {
            (true, false) => self.holds_active += 1,
            (false, true) => self.holds_active -= 1,
            _ => {}
        }
        self.sync_power_holds();
        let ic = &self.ics[pos as usize].1;
        let mut dirty: Vec<PartitionId> = [ic.vcc.node, ic.gnd.node].into_iter().chain(ic.logic.iter().map(|p| p.node)).filter_map(|n| self.partition_of_node(n)).collect();
        dirty.sort_unstable();
        dirty.dedup();
        dirty
    }

    /// Hands the partition solvers the table they judge their stamps (and
    /// the static one its cache keys) by.
    fn sync_power_holds(&mut self) {
        let on = self.holds_active > 0;
        self.holds_flag.0.set(on);
        self.static_solver.set_power_holds(Rc::clone(&self.holds), on);
        self.closed_form.set_power_holds(Rc::clone(&self.holds), on);
        self.stepped.set_power_holds(Rc::clone(&self.holds), on);
    }

    /// Holds taken and released (D8).
    #[cfg_attr(not(test), allow(dead_code, reason = "read through `PartitionSolver::hold_stats`"))]
    pub(crate) fn hold_counts(&self) -> [f64; 2] {
        [self.hold_counts[0] as f64, self.hold_counts[1] as f64]
    }

    fn close_window(&mut self) {
        self.window = false;
        self.w_holds = None;
        self.v_log.clear();
        self.i_log.clear();
        for p in self.dyn_logged_list.drain(..) {
            self.dyn_logged[p as usize] = false;
        }
        self.derived_logged = false;
        self.w_driver_snapshot = None;
        self.d_log.clear();
        self.w_stale.clear();
        self.activate_mirror_if_seen();
    }

    /// What every entry point that publishes does last: run the readout
    /// passes now or note that they are owed, queue the crossings and steps
    /// the partitions asked for, and check what was written is finite.
    fn finish_publish(&mut self, now: f64) -> Result<(), String> {
        let lazy = self.publish_readout(now);
        self.drain_scheduled();
        self.maybe_schedule_bucket(now);
        if lazy {
            self.check_finite_written()
        } else {
            self.check_finite(false)
        }
    }

    /// An error from a publishing entry point leaves the bounds unproven (the
    /// call may have written past a limit before it failed), so the next
    /// publish scans; the kernel rolls the instant back anyway.
    fn on_error<T>(&mut self, result: Result<T, String>) -> Result<T, String> {
        if result.is_err() {
            self.v_hint = f64::INFINITY;
            self.i_hint = f64::INFINITY;
        }
        result
    }

    /// [`PartitionSolver::resolve_dirty`]'s body, with the readers appended to
    /// `readers`. The three id lists are the caller's, taken out of `self` so
    /// the calls below can borrow the rest of it.
    fn resolve_dirty_with(
        &mut self,
        static_ids: &[PartitionId],
        cf_ids: &mut Vec<usize>,
        st_ids: &mut Vec<usize>,
        now: f64,
        drivers: &[Driver],
        readers: &mut Vec<DeviceId>,
    ) -> Result<(), String> {
        // `StaticPartitionSolver::resolve_dirty` recomputes every resistor/
        // switch current UNCONDITIONALLY, even for a device with no
        // partition at all (both terminals fixed) — so it (and the copy-in
        // that reads its result) must run every call, not only when
        // `static_ids` is non-empty, exactly mirroring that method's own
        // "recomputed unconditionally" contract (`static_part.rs`'s doc
        // comment on `resolve_dirty`). `closed_form`/`stepped` have no such
        // side effect for an empty id list, so calling them unconditionally
        // too is simply a cheap no-op, kept for symmetry.
        // W1c step 2b: keep a copy of the full driver table for
        // `handle_dynamic_event`/`sync_dynamic_to` (below, and this struct's
        // own `driver_snapshot` doc comment on why they need one at all now
        // that an IC can touch a dynamic partition). Unconditional, exactly
        // like `copy_in_static`/`recompute_source_currents` below it: a
        // caller only ever hands this the FULL table anyway (never filtered
        // to `dirty`'s own partitions), so there is no "nothing changed"
        // shortcut to take.
        let refreshed_in_place = self.refresh_driver_snapshot(drivers);
        self.ensure_ic_driver_ids(drivers);
        // W1f: a running ramp's nodes at this instant, before anything reads them.
        self.refresh_ramps(now);
        if self.exact.is_some() {
            self.push_exact_stamp();
        }

        self.static_solver.set_inputs_bounded(self.lazy_static_ok && self.v_hint <= LAZY_V_LIMIT);
        if let Some(e) = self.exact.as_ref() {
            self.exact_solved = static_ids.iter().any(|pid| pid.0 as usize == e.partition) || st_ids.contains(&e.partition);
        }
        self.static_solver.resolve_dirty_into(static_ids, now, drivers, readers)?;
        self.copy_in_static()?;
        // Finding 2/6: a watched IC's supply can sit in a `Static`
        // partition, which only just moved (if it is among `static_ids`) —
        // force a re-aim of every closed-form/stepped partition that
        // depends on one, even though nothing in THEIR OWN topology
        // changed, so the watch's thresholds (recomputed from the fresh
        // supply voltage `copy_in_static` just published) actually get
        // reconsidered instead of staying aimed at a stale target.
        if !static_ids.is_empty() || !st_ids.is_empty() {
            self.add_watch_dependent_partitions(static_ids, cf_ids, st_ids);
        }
        if refreshed_in_place && !st_ids.is_empty() {
            self.rebuild_pre_commit_table();
        }
        self.begin_dynamic_writes(cf_ids, false, !st_ids.is_empty());
        self.sync_rail_drive();
        self.closed_form.resolve_dirty_into(cf_ids, now, drivers, &self.live_params, &mut self.voltages, &mut self.currents, readers)?;
        // A stepped hub partition re-solved here is brought to `now` on the
        // CC rail its pending trial was planned on (`rail_passes`'
        // `rail_jump`): the rail's draws are already this instant's.
        if !st_ids.is_empty() && self.hub.as_ref().is_some_and(|h| h.state.reg == crate::devices::sources::Regime::Cc) {
            self.stepped.catch_up_on_cc_rail(st_ids, now, &self.pre_commit_drivers, &self.live_params, &self.voltages)?;
        }
        self.sync_stepped_cc(now);
        let stepped = self.stepped.resolve_dirty(st_ids, now, &self.pre_commit_drivers, drivers, &self.live_params, &mut self.voltages, &mut self.currents);
        self.stepped.cc = None;
        readers.extend(stepped?);
        self.take_stepped_holds()?;
        self.end_dynamic_writes();
        // W1f (Rails point 3): the rail loop runs inside this one call.
        self.rail_loop(now, drivers, readers)?;
        self.finish_publish(now)
    }
}

impl PartitionedSolver {
    /// [`PartitionSolver::handle_dynamic_event_into`]'s body, collecting
    /// the readers in `readers`.
    fn handle_dynamic_event_with(&mut self, partition: PartitionId, now: f64, is_crossing: bool, readers: &mut Vec<DeviceId>) -> Result<(), String> {
        let p = partition.0 as usize;
        // A hub partition's own step or crossing moves the rail as a `Rail`
        // event does: a rail IC whose gate it flips back at the next is
        // chasing its own draw across instants, HG1's ping-pong (W1c point
        // 1a), and is counted as `Rail` events are (a stepped partition at
        // H_MIN on a CC rail flipped a gate every 10 ns step: G2bv's fz_ic 6
        // hub 2, cold 204 tpl 5, 301 tpl 2). Any other partition's event
        // changed what the rail sees, and ends the count. A PSU in CV holds
        // its rail at the setpoint, which no partition moves: there the
        // count costs nothing (the SAP-1's case).
        let counted = |h: &hub::Hub| !h.rail_ics.is_empty() && h.contrib.get(p).is_some_and(|c| c.active) && (h.psu().is_none() || h.state.reg == crate::devices::sources::Regime::Cc);
        let gates = self.hub.as_ref().filter(|h| counted(h)).map(|_| self.rail_gates());
        if gates.is_none() {
            self.end_gate_chase();
        }
        if self.exact.is_some() {
            self.push_exact_stamp();
        }
        self.refresh_ramps(now);
        // A crossing under a known rail lands against the solved rail; if
        // that solve re-aimed the partition, the crossing is rescheduled
        // under it and nothing lands here.
        if is_crossing {
            let early = self.solve_rail_before_crossing(p, now, readers);
            if self.on_error(early)? {
                let published = self.finish_publish(now);
                self.on_error(published)?;
                return Ok(());
            }
        }
        if is_crossing {
            self.begin_dynamic_writes(&[p], false, false);
        } else {
            self.begin_dynamic_writes(&[], false, true);
        }
        self.sync_rail_drive();
        if is_crossing {
            let result = self.closed_form.handle_crossing_into(p, now, &self.driver_snapshot, &self.live_params, &mut self.voltages, &mut self.currents, readers);
            self.on_error(result)?;
        } else {
            self.sync_stepped_cc(now);
            let stepped = self.stepped.handle_step(p, now, &self.driver_snapshot, &self.live_params, &mut self.voltages, &mut self.currents);
            self.stepped.cc = None;
            readers.extend(self.on_error(stepped)?);
            let held = self.take_stepped_holds();
            self.on_error(held)?;
        }
        self.end_dynamic_writes();
        if !is_crossing {
            let retried = self.retry_stepped_holds(now, false, readers);
            self.on_error(retried)?;
        }
        // W1f: a closed-form crossing or a stepped commit of a hub partition
        // changes its contribution, so the rail solves at it (Rails point 4).
        let drivers = Rc::clone(&self.driver_snapshot);
        let rail = self.rail_loop(now, &drivers, readers);
        self.on_error(rail)?;
        if let Some(a) = gates.and_then(|g| self.count_gate_flip(g, !is_crossing)) {
            let result = self.hold_on_rail(now, &drivers, readers, &a, crate::rails::loop_unsettled());
            self.on_error(result)?;
            self.hub.as_mut().unwrap().state.gate_flips = 0;
        }
        // W1c step 2b lifted `partition.rs`'s former decline, so an IC CAN
        // now touch a dynamic partition: `handle_crossing`/`handle_step`
        // above already restamped its driver/pull contribution correctly
        // (using `self.driver_snapshot` — that field's own doc comment on
        // why it, not a fresh `drivers` slice, is what's available here).
        // What this call still needs to do is exactly the KCL fold for a
        // driver/pull pin whose OWN node is FIXED (contract point 4's last
        // bullet: the rail's published current keeps moving continuously as
        // a dynamic partition's own free-node voltage evolves, even between
        // driver commits) — `self.driver_snapshot` is the right table for
        // that too, not `&[]`: an empty slice would silently stop
        // republishing every IC driver's contribution to a fixed rail's own
        // current on every crossing/step in between actual commits.
        let published = self.finish_publish(now);
        self.on_error(published)?;
        Ok(())
    }
}

impl PartitionSolver for PartitionedSolver {
    fn resolve_dirty(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
        let mut out = Vec::new();
        self.resolve_dirty_into(dirty, now, drivers, &mut out)?;
        Ok(out)
    }

    fn resolve_dirty_into(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver], out: &mut Vec<DeviceId>) -> Result<(), String> {
        let mut static_ids = self.static_ids.take();
        let mut cf_ids = self.cf_ids.take();
        let mut st_ids = self.st_ids.take();
        static_ids.clear();
        cf_ids.clear();
        st_ids.clear();
        for &pid in dirty {
            let p = pid.0 as usize;
            match self.partition_kinds.get(p) {
                Some(PartitionKind::Static) => static_ids.push(pid),
                Some(PartitionKind::ClosedForm) => cf_ids.push(p),
                Some(PartitionKind::Stepped) => st_ids.push(p),
                None => {}
            }
        }
        // F5b: a commit does not end HG1's count (`end_gate_chase`): the
        // drivers that commit while a rail gate ping-pongs answer it (its own
        // outputs, a zero-delay reader), they do not drive the rail.
        let mut readers = self.readers_buf.take();
        readers.clear();
        self.rail_jump = true;
        let result = self.resolve_dirty_with(&static_ids, &mut cf_ids, &mut st_ids, now, drivers, &mut readers);
        self.rail_jump = false;
        self.static_ids.put(static_ids);
        self.cf_ids.put(cf_ids);
        self.st_ids.put(st_ids);
        if result.is_ok() {
            out.extend_from_slice(&readers);
        }
        self.readers_buf.put(readers);
        self.on_error(result)
    }

    fn voltages(&self) -> &[f64] {
        &self.voltages
    }

    fn currents(&self) -> &[f64] {
        debug_assert!(!self.derived_dirty, "currents() read while the readout pass is owed: call prepare_readout() first");
        &self.currents
    }

    fn channels(&self) -> &[f64] {
        debug_assert!(!self.derived_dirty, "channels() read while the readout pass is owed: call prepare_readout() first");
        &self.channels
    }

    /// Runs the readout pass if a publish left one pending. The pass reads
    /// only state a failed instant restores (`voltages`, the
    /// partition-published capacitor currents, `live_params`,
    /// `driver_snapshot`, `derived_now`), so what it computes here is bit for
    /// bit what the eager publish would have computed at the publish point.
    fn prepare_readout(&mut self) {
        if self.derived_dirty {
            self.derived_dirty = false;
            self.compute_derived(self.derived_now);
        }
    }

    /// The raw arrays, not [`Self::currents`]: the pass is never run for a
    /// snapshot. A slot it still owes is captured stale;
    /// [`PartitionSolver::dynamic_snapshot`] captures the flag that says so,
    /// and the kernel restores both.
    fn snapshot(&self) -> (Vec<f64>, Vec<f64>) {
        (self.voltages.clone(), self.currents.clone())
    }

    fn restore(&mut self, snapshot: (Vec<f64>, Vec<f64>)) {
        self.voltages = snapshot.0;
        self.currents = snapshot.1;
        // Values from before the bounds were last exact are back.
        self.v_hint = f64::INFINITY;
        self.i_hint = f64::INFINITY;
    }

    fn set_no_solution_cache(&mut self, off: bool) {
        self.static_solver.set_no_solution_cache(off);
    }

    fn cache_hits(&self) -> f64 {
        self.static_solver.cache_hits()
    }

    fn closed_form_segments(&self) -> f64 {
        PartitionedSolver::closed_form_segments(self)
    }

    fn crossings(&self) -> f64 {
        PartitionedSolver::crossings(self)
    }

    fn stepped_steps(&self) -> f64 {
        PartitionedSolver::stepped_steps(self)
    }

    fn trap_fallbacks(&self) -> f64 {
        self.stepped.trap_fallbacks()
    }

    fn newton_iterations(&self) -> f64 {
        PartitionedSolver::newton_iterations(self)
    }

    fn validate_params(&self, triples: &[(usize, usize, f64)]) -> Result<(), String> {
        for &(device_index, param_index, value) in triples {
            if !value.is_finite() {
                return Err(format!("device {device_index} param {param_index}: value {value} is not finite"));
            }
            let info = self.switches.get(&device_index).ok_or_else(|| format!("device {device_index}: has no live parameters"))?;
            let shape = kinds::shape(info.kind).expect("a tracked switch device always has a known kind code");
            if !shape.live_params.contains(&param_index) {
                return Err(format!("device {device_index} ({}): param {param_index} is not live", shape.name));
            }
            let params = self.live_params.get(&device_index).expect("every switch in `switches` has a `live_params` entry");
            if param_index >= params.len() {
                return Err(format!("device {device_index} ({}): param index {param_index} out of range", shape.name));
            }
            let mut new_params = params.clone();
            new_params[param_index] = value;
            // Only switch-family kinds have live params, and none carries a blob.
            (shape.check)(info.pin_count, &new_params, &[]).map_err(|msg| format!("device {device_index} ({}): {msg}", shape.name))?;
        }
        Ok(())
    }

    fn apply_params(&mut self, triples: &[(usize, usize, f64)]) -> Vec<PartitionId> {
        self.host_edit = true;
        let mut touched = BTreeSet::new();
        let mut static_triples = Vec::new();
        for &(device_index, param_index, value) in triples {
            if let Some(params) = self.live_params.get_mut(&device_index) {
                params[param_index] = value;
            }
            if let Some(info) = self.switches.get(&device_index) {
                match info.owner {
                    Owner::Static => static_triples.push((device_index, param_index, value)),
                    Owner::ClosedForm(p) | Owner::Stepped(p) => {
                        touched.insert(PartitionId(p as u32));
                    }
                }
            }
        }
        if !static_triples.is_empty() {
            touched.extend(self.static_solver.apply_params(&static_triples));
        }
        self.refresh_kcl_contributions(triples);
        // A switch contact from the rail to a fixed node is a direct device,
        // which no partition re-solve reports; any other switch reaches the
        // rail through its partition's new contribution.
        if let Some(hub) = self.hub.as_mut() {
            if triples.iter().any(|&(d, _, _)| hub.direct_devices.binary_search(&d).is_ok()) {
                hub.dirty = true;
            }
        }
        touched.into_iter().collect()
    }

    fn param_value(&self, device_index: usize, param_index: usize) -> Option<f64> {
        self.live_params.get(&device_index)?.get(param_index).copied()
    }

    /// Contract point 7 ("A failed advance"/"A failed set_params"): beyond
    /// `closed_form`/`stepped`'s own committed state, this must ALSO carry
    /// `static_solver`'s own private `(voltages, currents)` — distinct from
    /// this struct's PUBLISHED `self.voltages`/`self.currents` that
    /// `PartitionSolver::snapshot` already round-trips, but written to
    /// unconditionally by `StaticPartitionSolver::resolve_dirty` BEFORE that
    /// method's own finiteness check can fail the whole instant
    /// (`static_part.rs`'s doc comment on `resolve_dirty`), so a doomed
    /// attempt's half-solved values would otherwise sit in `static_solver`
    /// forever and reappear the next time ANY partition resolves (`
    /// copy_in_static` unconditionally copies every `Static` node out of it,
    /// not just the ones a given call actually re-solved) — and
    /// `pending_schedule`, since `resolve_dirty`/`handle_dynamic_event`/
    /// `sync_dynamic_to` all drain `closed_form`/`stepped`'s own `scheduled`
    /// queues into it BEFORE their `check_finite` call can fail, so a doomed
    /// attempt's crossing/step entries are already sitting here, not in
    /// those (never-snapshotted, always-empty-by-the-time-a-caller-sees-them)
    /// queues themselves. Restoring `pending_schedule` back to what it held
    /// before this instant started — always empty in practice, since
    /// `Kernel::drain_scheduled` empties it again after every successful
    /// resolve within the SAME instant — is what stops a stale entry from a
    /// failed attempt surviving to be pushed onto the kernel's heap by some
    /// later, unrelated, successful call — and, W1c step 2b, `driver_snapshot`
    /// itself (that field's own doc comment: without it, a `Crossing`/`Step`
    /// that fires on a LATER instant with no `resolve_dirty` of its own could
    /// restamp an IC against the failed attempt's last, never-committed
    /// driver states) — and, W1e, the published channel array (LEDs point
    /// 16): the device pass writes it after the partitions publish, so an
    /// earlier delta of a doomed instant has already changed it by the time a
    /// later one fails.
    fn dynamic_snapshot(&self) -> Box<dyn std::any::Any> {
        let base = (
            (
                self.static_solver.snapshot(),
                self.closed_form.snapshot(),
                self.stepped.snapshot(),
                self.pending_schedule.clone(),
                self.next_bucket,
                Rc::clone(&self.driver_snapshot),
                self.channels.clone(),
                self.derived_dirty,
                self.derived_now,
            ),
            self.w1f_snapshot(),
        );
        // F5: the hold table rides along only while an IC is held, so a
        // snapshot with none keeps the shape it always had; restoring that
        // shape means no hold.
        if self.holds_active == 0 && self.episodes.is_empty() {
            Box::new(base)
        } else {
            Box::new((base, (Rc::clone(&self.holds), self.holds_active, Rc::clone(&self.episodes))))
        }
    }

    fn dynamic_restore(&mut self, snapshot: Box<dyn std::any::Any>) {
        type Base = ((Vec<f64>, Vec<f64>), closed_form::Snapshot, stepped::Snapshot, Vec<(f64, EventKind)>, Option<f64>, Rc<Vec<Driver>>, Vec<f64>, bool, f64);
        type Holds = (Rc<Vec<digital::power::PowerHold>>, u32, Rc<Vec<hold::Episode>>);
        let snapshot = match snapshot.downcast::<((Base, hub::W1fSnapshot), Holds)>() {
            Ok(boxed) => {
                let (inner, (holds, holds_active, episodes)) = *boxed;
                (self.holds, self.holds_active, self.episodes) = (holds, holds_active, episodes);
                Box::new(inner) as Box<dyn std::any::Any>
            }
            Err(other) => {
                (self.holds, self.holds_active, self.episodes) = (Rc::new(Vec::new()), 0, Rc::new(Vec::new()));
                other
            }
        };
        self.sync_power_holds();
        if let Ok(boxed) = snapshot.downcast::<(Base, hub::W1fSnapshot)>() {
            let (base, w1f) = *boxed;
            self.w1f_restore(w1f);
            let (static_snap, cf, st, pending, next_bucket, driver_snapshot, channels, derived_dirty, derived_now) = base;
            self.static_solver.restore(static_snap);
            self.sync_hub_cache_band();
            self.closed_form.restore(cf);
            self.stepped.restore(st);
            self.pending_schedule = pending;
            self.next_bucket = next_bucket;
            self.driver_snapshot = driver_snapshot;
            self.channels = channels;
            self.derived_dirty = derived_dirty;
            self.derived_now = derived_now;
            self.v_hint = f64::INFINITY;
            self.i_hint = f64::INFINITY;
            // The table just came from a snapshot, not from a resolve.
            self.stale.clear();
            self.mirror_valid = false;
        }
    }

    fn driver_changing(&mut self, idx: usize) {
        self.mirror_seen = true;
        if self.mirror_live {
            self.stale.push(idx as u32);
        }
        // W1f: a commit of an IC on the rail can change a direct device's
        // draw (Rails point 2), which no partition re-solve reports.
        if let Some(hub) = self.hub.as_mut() {
            if !hub.dirty {
                if let Some(d) = self.driver_snapshot.get(idx) {
                    if self.ic_position.get(d.device).is_some_and(|&p| p != u32::MAX && hub.rail_ics.contains(&p)) {
                        hub.dirty = true;
                    }
                }
            }
        }
    }

    #[inline]
    fn driver_committed(&mut self, idx: usize) {
        self.note_driver_change(idx);
        // Ruling R54 (a): a commit is a foreign change to every IC but the
        // one that owns the driver.
        let owner = self.driver_snapshot.get(idx).map_or(u32::MAX, |d| if d.device == usize::MAX { u32::MAX } else { d.device as u32 });
        self.changes = self.changes.note(owner);
    }

    fn begin_host_step(&mut self, t_start: f64, t_end: f64) {
        self.host_step = t_end - t_start;
        // A rolled-back edit restores its params through `apply_params`
        // with no rail solve after it: nothing of it reaches the next host
        // step.
        self.host_edit = false;
    }

    /// Opens the window: nothing is copied. From here every slot of the
    /// shared arrays that the instant writes is logged first (the static
    /// solver logs its own), and each dynamic partition's state is saved
    /// before its first change (`closed_form`/`stepped`'s journals), so that
    /// a completed instant ([`Self::end_instant`]) costs nothing to drop and a
    /// failed one ([`Self::rollback_instant`]) puts back only what it changed.
    /// The few whole values the instant can change (the schedule queue, the
    /// bucket flag, the driver table's `Rc`, the readout flags) are kept
    /// directly. The kernel opens no nested windows.
    fn begin_instant(&mut self) -> Box<dyn std::any::Any> {
        assert!(!self.window, "rollback windows do not nest");
        self.window = true;
        self.changes = hold::Changes::None;
        self.v_log.clear();
        self.i_log.clear();
        self.static_solver.begin_window();
        self.closed_form.begin_window();
        self.stepped.begin_window();
        self.kind_log.clear();
        self.w_cc_crawl = self.cc_crawl;
        self.w_pending_schedule.clone_from(&self.pending_schedule);
        self.w_next_bucket = self.next_bucket;
        // No `activate_mirror_if_seen` here: a window that opens on a solver
        // not yet relying on the notifications holds its table its own way to
        // its end (activating now would leave the mirror unproven, so
        // `window_mirror` would be false either way), and `close_window`
        // activates it when the window ends.
        self.window_mirror = self.mirror_live && self.mirror_valid;
        self.d_log.clear();
        self.d_log_split = 0;
        if self.window_mirror {
            // Refreshed in place, so rolled back through the entries it
            // changes; the window itself holds no copy of the table.
            self.w_stale.clone_from(&self.stale);
            self.w_driver_snapshot = None;
        } else {
            self.w_driver_snapshot = Some(Rc::clone(&self.driver_snapshot));
        }
        self.w_derived = (self.derived_dirty, self.derived_now);
        // W1g point 15: the slow state's own journal.
        self.slow_begin_instant();
        // W1f (Rails point 8): the rail's whole state, its contribution
        // journal, the sources and the rule-2a anchors on live nodes.
        if let Some(hub) = self.hub.as_mut() {
            hub.begin_window();
        }
        if let Some(e) = self.exact.as_mut() {
            e.w_saved = Some((e.reg, e.battery, e.t_last, e.held));
        }
        self.w_wave = Some(Rc::clone(&self.wave));
        if self.live_caps {
            self.w_caps.clear();
            self.w_caps.extend(self.fixed_pair_capacitors.iter().map(|c| (c.d_v, c.vc0, c.t0)));
        }
        #[cfg(test)]
        {
            self.w_saved = Some((self.voltages.clone(), self.currents.clone(), self.channels.clone()));
        }
        Box::new(WindowToken)
    }

    fn end_instant(&mut self, checkpoint: Box<dyn std::any::Any>) {
        checkpoint.downcast::<WindowToken>().expect("a checkpoint made by this solver's begin_instant");
        debug_assert!(self.window, "end_instant without an open window");
        #[cfg(test)]
        self.check_logs_reproduce_the_window_start();
        self.close_window();
        self.static_solver.end_window();
        self.closed_form.end_window();
        self.stepped.end_window();
        self.kind_log.clear();
        self.slow_end_instant();
        if let Some(hub) = self.hub.as_mut() {
            hub.end_window();
        }
        if let Some(e) = self.exact.as_mut() {
            e.w_saved = None;
        }
        self.w_wave = None;
    }

    /// Undoes the logs newest first (a slot written twice ends at its first
    /// pre-image), then puts back the journals and the whole values.
    fn rollback_instant(&mut self, checkpoint: Box<dyn std::any::Any>) {
        checkpoint.downcast::<WindowToken>().expect("a checkpoint made by this solver's begin_instant");
        debug_assert!(self.window, "rollback_instant without an open window");
        for &(node, bits) in self.v_log.iter().rev() {
            self.voltages[node as usize] = f64::from_bits(bits);
        }
        for &(device, bits) in self.i_log.iter().rev() {
            self.currents[device as usize] = f64::from_bits(bits);
        }
        if self.derived_logged {
            self.channels.clone_from(&self.w_channels);
        }
        // The static solver sets its own `full`: the next resolve recomputes
        // and checks everything, since the arrays just went back.
        self.static_solver.rollback_window();
        self.closed_form.rollback_window();
        self.stepped.rollback_window();
        // R59 (1) (B): the kinds the window found (each solver's journal put
        // the state back; this puts back which copy runs).
        while let Some((p, kind)) = self.kind_log.pop() {
            self.set_kind(p as usize, kind);
        }
        self.cc_crawl = self.w_cc_crawl;
        self.pending_schedule.clone_from(&self.w_pending_schedule);
        self.next_bucket = self.w_next_bucket;
        if self.window_mirror {
            // Entries refreshed in place go back through their log; if a
            // whole copy was made in between, first the table it replaced
            // (which the entries before it are about).
            let restored_whole = self.w_driver_snapshot.is_some();
            if let Some(table) = self.w_driver_snapshot.take() {
                self.driver_snapshot = table;
            }
            let undo = if restored_whole { self.d_log_split } else { self.d_log.len() };
            let table = Rc::make_mut(&mut self.driver_snapshot);
            for &(idx, old) in self.d_log[..undo].iter().rev() {
                table[idx as usize] = old;
            }
            self.stale.clone_from(&self.w_stale);
        } else {
            self.driver_snapshot = self.w_driver_snapshot.take().expect("begin_instant saved the driver table");
            // The kernel's notes since are about a table this one no longer
            // follows: the next resolve copies it whole.
            self.stale.clear();
            self.mirror_valid = false;
        }
        (self.derived_dirty, self.derived_now) = self.w_derived;
        // W1f (Rails point 8).
        if let Some(hub) = self.hub.as_mut() {
            hub.rollback_window();
        }
        self.sync_hub_cache_band();
        if let Some(e) = self.exact.as_mut() {
            if let Some((reg, battery, t_last, held)) = e.w_saved.take() {
                (e.reg, e.battery, e.t_last, e.held) = (reg, battery, t_last, held);
            }
        }
        // A resolve that failed after marking the rail's partition solved
        // never reached the rail loop that takes the mark; left set, the next
        // event's rail loop would run the regime rules for a solve that did
        // not happen.
        self.exact_solved = false;
        if self.exact.is_some() {
            self.push_exact_stamp();
        }
        if let Some(wave) = self.w_wave.take() {
            self.wave = wave;
            self.sync_ramps();
            for (w, src) in self.wave.sources.iter().enumerate() {
                self.static_solver.set_wave_level(w, src.level_index);
            }
        }
        if self.live_caps {
            for (cap, &(d_v, vc0, t0)) in self.fixed_pair_capacitors.iter_mut().zip(&self.w_caps) {
                (cap.d_v, cap.vc0, cap.t0) = (d_v, vc0, t0);
            }
        }
        // W1g point 15: the slow state, then every stamp it set this instant.
        self.slow_rollback_instant();
        // F5 (Rails point 8): the hold table the window found.
        if let Some((holds, active, episodes, armed)) = self.w_holds.take() {
            self.holds = holds;
            self.holds_active = active;
            self.episodes = episodes;
            self.retry_armed = armed;
            self.sync_power_holds();
        }
        // Values from before the bounds were last exact are back.
        self.v_hint = f64::INFINITY;
        self.i_hint = f64::INFINITY;
        #[cfg(test)]
        if let Some((v0, i0, c0)) = self.w_saved.take() {
            let same = |a: &[f64], b: &[f64]| a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits());
            assert!(same(&self.voltages, &v0) && same(&self.currents, &i0) && same(&self.channels, &c0), "a rollback left a shared array changed");
        }
        self.close_window();
    }

    fn take_scheduled(&mut self) -> Vec<(f64, EventKind)> {
        std::mem::take(&mut self.pending_schedule)
    }

    fn is_current(&self, partition: PartitionId, is_crossing: bool, version: Version) -> bool {
        let p = partition.0 as usize;
        if is_crossing {
            self.closed_form.is_current(p, version)
        } else {
            self.stepped.is_current(p, version)
        }
    }

    fn handle_dynamic_event(&mut self, partition: PartitionId, now: f64, is_crossing: bool) -> Result<Vec<DeviceId>, String> {
        let mut out = Vec::new();
        self.handle_dynamic_event_into(partition, now, is_crossing, &mut out)?;
        Ok(out)
    }

    fn handle_dynamic_event_into(&mut self, partition: PartitionId, now: f64, is_crossing: bool, out: &mut Vec<DeviceId>) -> Result<(), String> {
        let mut readers = self.readers_buf.take();
        readers.clear();
        let result = self.handle_dynamic_event_with(partition, now, is_crossing, &mut readers);
        if result.is_ok() {
            out.extend_from_slice(&readers);
        }
        self.readers_buf.put(readers);
        result
    }

    fn sync_dynamic_to(&mut self, t_end: f64) -> Result<Vec<DeviceId>, String> {
        let ramps_moved = self.refresh_ramps(t_end);
        self.begin_dynamic_writes(&[], true, true);
        self.closed_form.publish_at(t_end, &self.live_params, &mut self.voltages, &mut self.currents);
        self.sync_stepped_cc(t_end);
        let result = self.stepped.sync_to(t_end, &self.driver_snapshot, &self.live_params, &mut self.voltages, &mut self.currents);
        self.stepped.cc = None;
        let mut readers = self.on_error(result)?;
        let held = self.take_stepped_holds();
        self.on_error(held)?;
        self.end_dynamic_writes();
        let retried = self.retry_stepped_holds(t_end, true, &mut readers);
        self.on_error(retried)?;
        // W1f: the stepped hub partitions the clamp committed contribute, and
        // a CC rail capacitor is published live (Rails point 4). The clamp
        // ends the host's step: every IC held off is retried powered there,
        // as classic's H8 retries a held chip at its next host step (ruling
        // R51 (b)), the static ones first, so the rail sees their draws.
        let drivers = Rc::clone(&self.driver_snapshot);
        self.at_clamp = self.holds_active > 0;
        let rail = self.retry_static_holds(t_end, &drivers, &mut readers).and_then(|()| self.rail_at_clamp(t_end, &drivers, &mut readers));
        self.at_clamp = false;
        self.on_error(rail)?;
        // The next host step retries again.
        self.arm_retry();
        self.static_solver.arm_retry();
        // A running ramp's nodes moved to `t_end`: the static solver publishes
        // what they feed, and their readers read them there (Sources point 3).
        if ramps_moved {
            for node in 0..self.wave.ramping.len() {
                if self.wave.ramping[node] {
                    readers.extend(self.node_ic_readers[node].iter().map(|&d| DeviceId(d)));
                }
            }
            self.static_solver.set_inputs_bounded(false);
            let result = self.static_solver.resolve_dirty_into(&[], t_end, &drivers, &mut readers);
            self.on_error(result)?;
            let held = self.copy_in_static();
            self.on_error(held)?;
        }
        // Same reasoning as `handle_dynamic_event` above: the horizon clamp
        // re-publishes a dynamic partition's exact state at `t_end`, and a
        // fixed rail's own current must track that continuously, not only at
        // the instants a driver actually commits.
        let published = self.finish_publish(t_end);
        self.on_error(published)?;
        Ok(readers)
    }

    fn partition_of_node(&self, node: usize) -> Option<PartitionId> {
        self.local_index.get(node).copied().flatten().map(|(p, _)| PartitionId(p as u32))
    }

    fn ic_powered(&self, device: usize) -> bool {
        match self.ic_position.get(device).filter(|&&i| i != u32::MAX).map(|&i| &self.ics[i as usize]) {
            Some(&(device_index, ref ic)) => digital::power::verdict(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node], self.power_hold(device_index)),
            // Not a tracked IC at all (a non-IC driver, `Driver::device ==
            // usize::MAX`, or an out-of-range index): "true for a non-IC"
            // (the trait default's own doc comment) — the kernel only ever
            // calls this for `device != usize::MAX` in the first place, but
            // answering permissively here rather than panicking keeps this
            // method total.
            None => true,
        }
    }

    /// W1c contract point 6's third bullet: partitions to mark dirty are
    /// every one holding an open `FLOATING_INPUT` pin of a CURRENTLY powered
    /// IC (its pull target may have flipped — `partition.rs`'s union rule
    /// guarantees that pin's node has a partition whenever it is free, which
    /// it always is for a pull-eligible pin: an open pin is never a fixed
    /// node in practice, but `partition_of_node`'s `None` is simply skipped
    /// here rather than assumed impossible); readers are every powered IC
    /// that either holds such a pin or currently reads a pin in its dead
    /// band, exactly [`any_powered_ic_needs_bucket_watch`]'s own per-IC test
    /// applied here so the caller learns WHICH ICs, not just whether any do.
    fn bucket_boundary(&mut self, now: f64) -> (Vec<PartitionId>, Vec<DeviceId>) {
        self.next_bucket = None;
        let mut dirty = BTreeSet::new();
        let mut readers = BTreeSet::new();
        for (device_index, ic) in &self.ics {
            // C8, as in `any_powered_ic_needs_bucket_watch`: a 555 takes no
            // part in the floating model, so a bucket boundary never dirties
            // or wakes it.
            if digital::is_ne555(ic.kind) {
                continue;
            }
            let power = digital::power::ic_power_held(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node], self.power_hold(*device_index));
            if !power.powered {
                continue;
            }
            let mut needs_reader = false;
            for pin in &ic.logic {
                if pin.open && pin.floating_input && !pin.open_high {
                    needs_reader = true;
                    if let Some((p, _)) = self.local_index.get(pin.node).copied().flatten() {
                        dirty.insert(PartitionId(p as u32));
                    }
                } else if !pin.open && power.schmitt.is_none() {
                    let v = self.voltages[pin.node] - power.v_gnd;
                    if v > power.vil && v < power.vih {
                        needs_reader = true;
                    }
                }
            }
            if needs_reader {
                readers.insert(DeviceId(*device_index as u32));
            }
        }
        self.maybe_schedule_bucket(now);
        (dirty.into_iter().collect(), readers.into_iter().collect())
    }

    fn rail_current(&self, version: Version) -> bool {
        self.hub.as_ref().is_some_and(|h| h.state.version == version && h.state.event_t.is_some())
    }

    fn rail_event(&mut self, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
        self.rail_event_at(now)
    }

    fn source_current(&self, device: DeviceId, version: Version) -> bool {
        let d = device.0 as usize;
        match self.wave.wave_of.get(d) {
            Some(&w) if w != u32::MAX => {
                let src = &self.wave.sources[w as usize];
                if version & live::WATCH_BIT != 0 {
                    src.ramp.is_some() && version & !live::WATCH_BIT == src.watch_gen
                } else {
                    src.version == version && src.next.is_some()
                }
            }
            _ => false,
        }
    }

    fn source_edge(&mut self, device: DeviceId, version: Version, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
        self.end_gate_chase();
        if version & live::WATCH_BIT != 0 {
            self.ramp_crossing_at(device.0 as usize, now)
        } else {
            // A waveform edge is a foreign change to every IC (ruling R54 (a)).
            self.changes = hold::Changes::Many;
            self.rail_jump = true;
            let result = self.source_edge_at(device.0 as usize, now);
            self.rail_jump = false;
            result
        }
    }

    fn device_state(&self, device: usize, now: f64, out: &mut Vec<f64>) -> Option<usize> {
        self.slow_device_state(device, now, out).or_else(|| self.hub_device_state(device, now, out))
    }

    fn take_settle_change(&mut self) -> bool {
        self.hub.as_mut().is_some_and(|h| std::mem::take(&mut h.state.settle_changed))
    }

    fn source_slots(&self, out: &mut Vec<(usize, f64)>) {
        self.wave_slots(out);
    }

    fn rail_stats(&self) -> [f64; 6] {
        let sens = (self.static_solver.sensitivities() + self.closed_form.sensitivities() + self.stepped.sensitivities()) as f64;
        match self.hub.as_ref() {
            Some(h) => [h.stats.moves as f64, sens, h.stats.scans as f64, h.stats.events as f64, self.source_edges as f64, h.stats.flips as f64],
            None => [0.0, sens, 0.0, 0.0, self.source_edges as f64, 0.0],
        }
    }

    fn rail_node(&self) -> Option<usize> {
        self.hub.as_ref().map(|h| h.node)
    }

    fn slow_settle(&mut self, now: f64, drivers: &[Driver]) -> Result<Option<Vec<PartitionId>>, String> {
        let result = self.settle_slow(now, drivers);
        self.on_error(result)
    }

    fn slow_is_current(&self, device: DeviceId, token: u32) -> bool {
        // The top bit marks a re-anchor (R21); the rest is the device's token.
        (device.0 as usize) < self.slow.device_count() && (self.slow.token(device.0 as usize) ^ token) & !crate::events::SLOW_ANCHOR == 0
    }

    fn handle_slow_anchor(&mut self, device: DeviceId, now: f64) -> Result<Vec<PartitionId>, String> {
        let result = self.slow_anchor(device.0 as usize, now);
        self.on_error(result)
    }

    fn handle_slow_event(&mut self, device: DeviceId, now: f64) -> Result<Vec<PartitionId>, String> {
        let result = self.slow_event(device.0 as usize, now);
        self.on_error(result)
    }

    fn reset_failures(&mut self, now: f64) -> Result<Vec<PartitionId>, String> {
        let result = self.reset_slow(now);
        self.on_error(result)
    }

    fn collect_records(&mut self, channel: u32, now: f64, out: &mut Vec<f64>) -> bool {
        self.slow_records(channel, now, out)
    }

    fn slow_device_state(&self, device: usize, now: f64, out: &mut Vec<f64>) -> Option<usize> {
        self.slow_state_of(device, now, out)
    }

    fn slow_stats(&self) -> [f64; 4] {
        self.slow_counts.map(|c| c as f64)
    }

    fn power_holds(&self) -> &[digital::power::PowerHold] {
        self.power_holds_slice()
    }

    fn holds_flag(&self) -> Option<Rc<std::cell::Cell<bool>>> {
        Some(Rc::clone(&self.holds_flag.0))
    }

    fn clone_box(&self) -> Box<dyn PartitionSolver> {
        Box::new(self.clone())
    }

    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        crate::snapshot::HeapBytes::heap_bytes(self, w)
    }

    fn carry_take(&self, now: f64, out: &mut Vec<f64>) {
        self.carry_records(now, out);
    }

    #[cfg(test)]
    fn clear_test_logs(&mut self) {
        self.bill_log.clear();
        self.stepped.commit_log.clear();
        self.stepped.resolve_log.clear();
        self.stepped.exact_commit_log.clear();
    }

    fn drop_power_holds(&mut self) -> (Vec<PartitionId>, Vec<DeviceId>) {
        self.release_every_hold()
    }

    fn hold_stats(&self) -> [f64; 2] {
        self.hold_counts()
    }

    #[cfg(test)]
    fn hold_power(&mut self, device: usize, hold: digital::power::PowerHold) -> Vec<PartitionId> {
        self.set_power_hold(device, hold)
    }
}

#[cfg(test)]
mod partitioned_solver_tests {
    use super::*;
    use crate::drivers::Driver;
    use crate::kernel::Kernel;
    use crate::program::{Device, Options, PinRecord};

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
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
    fn capacitor(ci: usize, a: usize, b: usize, c: f64, esr: f64, leak: f64) -> Device {
        Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![c, esr, leak], blob: vec![] }
    }
    fn spdt(ci: usize, com: usize, b: usize, c: usize, sel: f64) -> Device {
        Device { kind: kinds::SPDT_SWITCH, component_index: ci, pins: vec![pin(com), pin(b), pin(c)], params: vec![sel], blob: vec![] }
    }
    fn dip(ci: usize, pins: Vec<usize>, k: usize, sw: [f64; 8]) -> Device {
        let mut params = vec![k as f64];
        params.extend(sw);
        Device { kind: kinds::DIP_SWITCH, component_index: ci, pins: pins.into_iter().map(pin).collect(), params, blob: vec![] }
    }
    fn program(node_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    }
    fn all_partitions(solver: &PartitionedSolver) -> Vec<PartitionId> {
        (0..solver.partition_count() as u32).map(PartitionId).collect()
    }
    struct NoDevices;
    impl crate::kernel::DeviceEvaluator for NoDevices {
        fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<crate::kernel::DriverRequest> {
            Vec::new()
        }
    }

    // ---- review section 1: a switch-family contact through a fixed pin ---
    //
    // An spdt_switch's `com` (or a dip_switch's `a` side) can sit on a fixed
    // pin, so the contact itself never has two free ends to union. `b` and
    // `c` (or every `b_i`) must still land in ONE partition (contract point
    // 1's third bullet, `partition.rs`'s module doc comment), or the old
    // single-owner dispatch attributes the whole device to whichever free
    // pin it finds first and `compile_dyn_shape::resolve` fabricates 0V for
    // the other one — the review's "5000 A through a contact it never
    // stamped" bug.

    /// com on a 5V source (fixed), b through 10k + 1uF to gnd (the
    /// partition's one counted capacitor), c through a bare 10k to gnd.
    fn spdt_partition_program(sel: f64) -> Program {
        program(4, vec![
            source(0, 1, 0, 5.0),
            spdt(1, 1, 2, 3, sel),
            resistor(2, 2, 0, 10_000.0),
            capacitor(3, 2, 0, 1e-6, 0.0, 0.0),
            resistor(4, 3, 0, 10_000.0),
        ])
    }

    #[test]
    fn spdt_com_on_a_fixed_source_puts_b_and_c_in_one_closed_form_partition_and_selecting_c_at_load_reads_half_a_milliamp() {
        // sel=1 selects c: com fixed to b/c through no free-free contact
        // ever unions the old way, so this also proves the classification.
        let p = spdt_partition_program(1.0);
        let pb = partition::build(&p).unwrap();
        let (p_b, _) = pb.local_index[2].expect("b is a free node");
        let (p_c, _) = pb.local_index[3].expect("c is a free node");
        assert_eq!(p_b, p_c, "b and c must share one partition even though com is fixed");
        assert_eq!(pb.kinds[p_b], PartitionKind::ClosedForm, "exactly one counted capacitor (on b) classifies the merged partition ClosedForm");

        let mut solver = PartitionedSolver::build(&p).unwrap();
        let dirty = all_partitions(&solver);
        solver.resolve_dirty(&dirty, 0.0, &[]).unwrap();
        solver.prepare_readout();
        assert!((solver.voltages()[3] - 5.0).abs() < 1e-3, "v(c) = {}", solver.voltages()[3]);
        // 5V across ~10k (the 1 milliohm contact is negligible), not the
        // 5000A a fabricated 0V fixed boundary on c would have produced.
        assert!((solver.currents()[1] - 5e-4).abs() < 1e-5, "spdt current = {}", solver.currents()[1]);
    }

    #[test]
    fn spdt_selecting_c_by_set_params_reads_near_5v_and_half_a_milliamp_not_5000_amps() {
        let p = spdt_partition_program(0.0); // sel=0 selects b at load.
        let solver = PartitionedSolver::build(&p).unwrap();
        let all = all_partitions(&solver);
        let mut kernel = Kernel::new(solver, NoDevices);
        kernel.settle(0.0, &all, &[]).unwrap();

        kernel.apply_params_as_event(&[(1, 0, 1.0)]).unwrap(); // device 1 = spdt, param 0 = sel.

        kernel.solver_mut().prepare_readout();
        assert!((kernel.solver().voltages()[3] - 5.0).abs() < 1e-3, "v(c) = {}", kernel.solver().voltages()[3]);
        assert!((kernel.solver().currents()[1] - 5e-4).abs() < 1e-5, "spdt current = {}", kernel.solver().currents()[1]);
    }

    /// a1 and a2 both on a 5V rail (the SAME fixed node), b1 through 10k +
    /// 1uF to gnd, b2 through a bare 10k to gnd.
    fn dip_partition_program(sw1: f64, sw2: f64) -> Program {
        program(4, vec![
            source(0, 1, 0, 5.0),
            dip(1, vec![1, 2, 1, 3], 2, [sw1, sw2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            resistor(2, 2, 0, 10_000.0),
            capacitor(3, 2, 0, 1e-6, 0.0, 0.0),
            resistor(4, 3, 0, 10_000.0),
        ])
    }

    #[test]
    fn dip_a_pins_on_a_fixed_rail_puts_b1_and_b2_in_one_closed_form_partition_and_sw2_closed_at_load_reads_half_a_milliamp() {
        let p = dip_partition_program(0.0, 1.0); // sw1 open, sw2 closed at load.
        let pb = partition::build(&p).unwrap();
        let (p_b1, _) = pb.local_index[2].expect("b1 is a free node");
        let (p_b2, _) = pb.local_index[3].expect("b2 is a free node");
        assert_eq!(p_b1, p_b2, "b1 and b2 must share one partition even though a1/a2 are both fixed to the rail");
        assert_eq!(pb.kinds[p_b1], PartitionKind::ClosedForm);

        let mut solver = PartitionedSolver::build(&p).unwrap();
        let dirty = all_partitions(&solver);
        solver.resolve_dirty(&dirty, 0.0, &[]).unwrap();
        assert!((solver.voltages()[3] - 5.0).abs() < 1e-3, "v(b2) = {}", solver.voltages()[3]);
        solver.prepare_readout();
        assert!((solver.currents()[1] - 5e-4).abs() < 1e-5, "dip current = {}", solver.currents()[1]);
    }

    #[test]
    fn dip_sw2_closed_by_set_params_reads_near_5v_and_half_a_milliamp_not_5000_amps() {
        let p = dip_partition_program(0.0, 0.0); // both open at load.
        let solver = PartitionedSolver::build(&p).unwrap();
        let all = all_partitions(&solver);
        let mut kernel = Kernel::new(solver, NoDevices);
        kernel.settle(0.0, &all, &[]).unwrap();

        kernel.apply_params_as_event(&[(1, 2, 1.0)]).unwrap(); // device 1 = dip, param 2 = sw2.

        assert!((kernel.solver().voltages()[3] - 5.0).abs() < 1e-3, "v(b2) = {}", kernel.solver().voltages()[3]);
        kernel.solver_mut().prepare_readout();
        assert!((kernel.solver().currents()[1] - 5e-4).abs() < 1e-5, "dip current = {}", kernel.solver().currents()[1]);
    }

    /// node0=gnd, node1=source(+, 5V), node2=RC midpoint. A switch in
    /// parallel with the resistor (node1-node2) changes the surrounding
    /// resistive network when toggled, but must not move the capacitor's own
    /// terminal voltage (esr=0, so that terminal EQUALS `vc` exactly,
    /// mathematically continuous by construction) even a single bit.
    fn rc_with_parallel_switch_program() -> Program {
        program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 1, 2, 0.0), capacitor(3, 2, 0, 1e-6, 0.0, 0.0)])
    }

    /// `test/fixtures/core/13-rc-switch-toggle.sim.json`'s exact topology: the
    /// switch sits IN SERIES before the resistor (not in parallel with it, as
    /// `a_switch_toggle_does_not_step_the_capacitors_terminal_voltage` above
    /// uses), so the partition has TWO free nodes (the switch/resistor
    /// junction, and the capacitor's own pin) — proving the continuity fix
    /// (`closed_form::fix_branch_terminals_exactly`) holds for that shape
    /// too, not only the simplest one-free-node case. Driven through 40
    /// repeated 1ms horizon-clamp calls before and after the toggle, exactly
    /// like `test/core/equivalence/stimuli.ts`'s `replayStimuliOnCore` fixed-
    /// step replay (never a single jump), since that is the ACTUAL path the
    /// TS-side equivalence suite drives a stimulus phase through.
    #[test]
    fn a_series_switch_toggle_leaves_a_two_free_node_partitions_capacitor_pin_bit_exact() {
        let p = program(4, vec![source(0, 1, 0, 5.0), switch_dev(1, 1, 2, 1.0), resistor(2, 2, 3, 100000.0), capacitor(3, 3, 0, 1e-6, 0.0, 0.0)]);
        let mut solver = PartitionedSolver::build(&p).unwrap();
        solver.resolve_dirty(&[PartitionId(0)], 0.0, &[]).unwrap();
        for step in 1..=40 {
            solver.sync_dynamic_to(step as f64 * 1e-3).unwrap();
        }
        let before = solver.voltages()[3];
        assert!(before > 1.0 && before < 2.0, "expected a partially-charged node, got {before}");

        let touched = solver.apply_params(&[(1, 0, 0.0)]); // open the switch, mirroring `{closed: 0}`.
        assert_eq!(touched, vec![PartitionId(0)]);
        solver.resolve_dirty(&touched, 40e-3, &[]).unwrap();
        let after = solver.voltages()[3];
        assert_eq!(before.to_bits(), after.to_bits(), "the capacitor's own pin (esr=0, its other terminal grounded) must not step across the toggle: before={before} after={after}");

        // And it stays frozen afterward (no path to the source remains, only
        // the universal node shunt's ~1e6 s decay — utterly negligible over
        // 40 more ms).
        for step in 1..=40 {
            solver.sync_dynamic_to(40e-3 + step as f64 * 1e-3).unwrap();
        }
        assert!((solver.voltages()[3] - after).abs() < 1e-6, "expected the held charge to stay essentially frozen for 40ms, moved by {}", solver.voltages()[3] - after);
    }

    #[test]
    fn a_switch_toggle_does_not_step_the_capacitors_terminal_voltage() {
        let p = rc_with_parallel_switch_program();
        let mut solver = PartitionedSolver::build(&p).unwrap();
        solver.resolve_dirty(&[PartitionId(0)], 0.0, &[]).unwrap();

        // Advance (via the same horizon-clamp path `Kernel::advance` uses)
        // to a time partway through the charge, so `vc` is a nontrivial,
        // nonzero value when the toggle lands — not merely unchanged because
        // it never moved off its initial 0.
        solver.sync_dynamic_to(5e-4).unwrap();
        let before = solver.voltages()[2];
        assert!(before > 0.1 && before < 5.0, "expected a partially-charged node, got {before}");

        // Toggle the switch (device index 2) closed, exactly as
        // `Kernel::apply_params_as_event` would dispatch a live
        // `bo_engine_set_params` call: mutate, then resolve the touched
        // partition(s) at the current time.
        let touched = solver.apply_params(&[(2, 0, 1.0)]);
        assert_eq!(touched, vec![PartitionId(0)]);
        solver.resolve_dirty(&touched, 5e-4, &[]).unwrap();

        let after = solver.voltages()[2];
        assert_eq!(before.to_bits(), after.to_bits(), "a capacitor's terminal voltage (esr=0) must not step across a re-aim: before={before} after={after}");
    }

    #[test]
    fn a_switch_toggle_via_set_params_style_dispatch_reaims_and_cancels_the_old_crossing() {
        let p = rc_with_parallel_switch_program();
        let solver = PartitionedSolver::build(&p).unwrap();
        struct NoDevices;
        impl crate::kernel::DeviceEvaluator for NoDevices {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<crate::kernel::DriverRequest> {
                Vec::new()
            }
        }
        let mut kernel = Kernel::new(solver, NoDevices);
        kernel.settle(0.0, &[PartitionId(0)], &[]).unwrap();

        // Register a watcher on the RC node (device index 2, the switch, is
        // reused here only as an arbitrary `DeviceId` stand-in — no real
        // reader kind exists in W1b) so a crossing actually gets scheduled.
        assert!(kernel.solver_mut().watch_node(2, DeviceId(0), Box::new(|_v: &[f64]| vec![Threshold { value: 3.0, rising: true }])));
        // Re-aim through the KERNEL (not the solver directly) so the
        // resulting schedule is actually DRAINED onto its heap
        // (`Kernel::drain_scheduled`, called from `apply_params_as_event`'s
        // `run_deltas`) — setting the switch to the value it already holds
        // still dirties the partition (`PartitionedSolver::apply_params`
        // always reports the touched partition) without changing anything
        // electrically, so this purely "picks up" the newly-registered
        // watcher.
        kernel.apply_params_as_event(&[(2, 0, 0.0)]).unwrap();

        let before_next = kernel.next_event_time();
        assert!(before_next.is_finite(), "expected a crossing to be scheduled");

        // The toggle (`bo_engine_set_params`'s dispatch, `Engine::set_params`
        // -> `Kernel::apply_params_as_event`) re-aims the partition, which
        // must cancel the old crossing.
        let validate_ok = kernel.solver().validate_params(&[(2, 0, 1.0)]);
        assert!(validate_ok.is_ok());
        kernel.apply_params_as_event(&[(2, 0, 1.0)]).unwrap();

        // The stale crossing must no longer be current — `next_event_time`
        // silently drops it and reports whatever the NEW re-aim scheduled
        // (or infinity, if the new segment's asymptote no longer reaches the
        // threshold).
        let after_next = kernel.next_event_time();
        assert_ne!(before_next, after_next, "the re-aim must have cancelled the stale crossing, not left it in place");
    }

    #[test]
    fn a_failed_advance_rolls_back_dynamic_state_including_committed_vc() {
        let p = rc_with_parallel_switch_program();
        let solver = PartitionedSolver::build(&p).unwrap();

        // A driver assigned to the RC partition's own id, oscillating
        // forever: every commit marks partition 0 dirty, forcing a real
        // re-aim (bumping the segment's version) each delta pass, without
        // ever letting the instant settle — a livelock, exactly like
        // `kernel.rs`'s own `an_oscillating_zero_delay_pair_hits_the_cap_...`
        // test, but against the REAL dynamic solver instead of a mock, so
        // this also proves `dynamic_snapshot`/`dynamic_restore` are actually
        // wired into `Kernel::advance`'s rollback.
        use crate::events::Mode::{High, Low};
        struct OscillatingEvaluator {
            driver: crate::events::DriverId,
            next: crate::events::Mode,
        }
        impl crate::kernel::DeviceEvaluator for OscillatingEvaluator {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<crate::kernel::DriverRequest> {
                let level = self.next;
                self.next = if level == High { Low } else { High };
                vec![crate::kernel::DriverRequest::Level { driver: self.driver, level, delays: crate::drivers::DelaySet::uniform(0.0) }]
            }
        }
        let d0 = crate::events::DriverId(0);
        let mut kernel = Kernel::new(solver, OscillatingEvaluator { driver: d0, next: High });
        kernel.settle(0.0, &[PartitionId(0)], &[]).unwrap();

        // A watcher (returning no thresholds — it exists purely to make this
        // partition report a reader on every resolve) so the oscillating
        // evaluator actually gets invoked at all: `resolve_dirty`'s returned
        // readers come only from `part.watches`, and with none registered
        // the delta cycle would settle after the very first commit with
        // nothing left to oscillate.
        kernel.solver_mut().watch_node(2, DeviceId(0), Box::new(|_v: &[f64]| Vec::<Threshold>::new()));

        // Advance for real first, so `vc`/the published node voltage is a
        // genuine nonzero mid-charge value, not merely its untouched initial
        // 0 — a rollback that "changes nothing" either way would not prove
        // anything. The segment built once at `settle` (version 1) is never
        // re-aimed by this plain advance (no crossing was scheduled: the
        // watcher above reports no thresholds, and the horizon clamp's own
        // `ClosedFormSolver::publish_at` is a pure read that never bumps the
        // version).
        kernel.advance(5e-4).unwrap();
        let committed_voltage = kernel.solver().voltages()[2];
        assert!(committed_voltage > 0.1 && committed_voltage < 5.0);
        let current_version = 1u64;
        assert!(kernel.solver().is_current(PartitionId(0), true, current_version));

        assert_eq!(kernel.add_driver(Driver::new(1, 100, 101, 1.0, 1.0, 0.0), Some(PartitionId(0))), d0);
        kernel.request_driver(d0, Low, kernel.time(), 0.0);

        let err = kernel.advance(kernel.time()).unwrap_err();
        assert!(matches!(err, crate::kernel::KernelError::Livelock { .. }));

        // Rolled all the way back: time, the published voltage AND the
        // segment's own version (proof that `dynamic_restore`, not just the
        // plain `voltages`/`currents` snapshot, actually ran).
        assert_eq!(kernel.solver().voltages()[2].to_bits(), committed_voltage.to_bits());
        assert!(kernel.solver().is_current(PartitionId(0), true, current_version), "the segment's version must be rolled back too, not left at whatever the livelock's last re-aim bumped it to");
    }

    // ---- review section 4: rollback leaks ---------------------------------

    /// Section 4 of the W1b review, contract point 7 ("A failed advance"
    /// also covers a failed `set_params`): a failed instant must restore
    /// EVERY piece of solver state the attempt touched, not just the
    /// published voltages/currents `PartitionedSolver::restore` already
    /// covers. `StaticPartitionSolver::resolve_dirty` writes its OWN
    /// internal `voltages`/`currents` — and recomputes EVERY static
    /// device's current unconditionally, even ones outside this call's own
    /// dirty set — BEFORE its finiteness check can fail the whole instant,
    /// so without also rolling that internal state back, a doomed attempt's
    /// half-solved values sit there until ANY later, unrelated, successful
    /// event's own unconditional recompute republishes them as if they were
    /// real (`copy_in_static` copies every `Static` node out of it, not
    /// just the ones that call actually re-solved).
    #[test]
    fn a_failed_set_params_does_not_leak_static_solver_state_into_a_later_unrelated_event() {
        // device0=source0(1e306V), device1=R1(1ohm, node1->node2), device2=
        // S1(node2-gnd, open), device3=S2(across the source, node1-gnd,
        // open), device4=source1(5V), device5=R2(1k, node3->node4), device6
        // =S3(node4-gnd, open). node2 and node4 are two SEPARATE static
        // partitions (no free-free contact ever joins them).
        let p = program(5, vec![
            source(0, 1, 0, 1e306),
            resistor(1, 1, 2, 1.0),
            switch_dev(2, 2, 0, 0.0),
            switch_dev(3, 1, 0, 0.0),
            source(4, 3, 0, 5.0),
            resistor(5, 3, 4, 1000.0),
            switch_dev(6, 4, 0, 0.0),
        ]);
        let solver = PartitionedSolver::build(&p).unwrap();
        let all = all_partitions(&solver);
        let mut kernel = Kernel::new(solver, NoDevices);
        kernel.settle(0.0, &all, &[]).unwrap();

        let v2_before = kernel.solver().voltages()[2];
        kernel.solver_mut().prepare_readout();
        let i1_before = kernel.solver().currents()[1];

        // Close S1 and S2 together as one event: S2 shorts the 1e306V
        // source to the datum through its own 1 milliohm contact,
        // overflowing THAT device's published current to +Infinity.
        let err = kernel.apply_params_as_event(&[(2, 0, 1.0), (3, 0, 1.0)]).unwrap_err();
        assert!(matches!(err, crate::kernel::KernelError::SolveFailed(_)));

        kernel.solver_mut().prepare_readout();
        assert_eq!(kernel.solver().voltages()[2], v2_before, "the published node2 voltage must roll back");
        assert_eq!(kernel.solver().currents()[1], i1_before, "R1's published current must roll back too");

        // An UNRELATED event, on a different partition entirely (S3,
        // node4): must not republish node2's doomed, S1-closed voltage —
        // nor the R1 current that stale voltage would imply.
        kernel.apply_params_as_event(&[(6, 0, 1.0)]).unwrap();
        kernel.solver_mut().prepare_readout();
        assert_eq!(kernel.solver().voltages()[2], v2_before, "an unrelated successful set_params must not resurrect the failed attempt's node2 voltage");
        assert_eq!(kernel.solver().currents()[1], i1_before, "...or R1's doomed current either");
    }

    /// Section 4 of the W1b review, contract point 7: "any crossing or step
    /// event the attempt scheduled" must not survive a rollback either. Two
    /// independent `ClosedForm` partitions, A and B, dirtied by the SAME
    /// event: A has a watcher registered, so re-aiming it schedules a real
    /// crossing (drained into `PartitionedSolver::pending_schedule` by that
    /// same call's own `drain_scheduled`, run BEFORE `check_finite`). B's
    /// own switch sits directly between a free node and a 1e306V fixed one
    /// (`com`-style, like the review's section 1/4 fixtures); closing it
    /// stamps `g * fixed = 1000 * 1e306 = 1e309` into `build_segment`'s own
    /// RHS, overflowing to `+Infinity` — `lu::LuFactorization::solve` checks
    /// only the PIVOTs (finite by construction; only the right-hand side
    /// overflows), so `build_segment` itself returns `Ok` with a poisoned
    /// segment, and it is `PartitionedSolver::check_finite`, at the very
    /// end, that actually fails the whole event — AFTER `drain_scheduled`
    /// already moved A's crossing into `pending_schedule`. Without the
    /// `pending_schedule` rollback, that stale entry survives into the next
    /// successful, unrelated re-aim's own `drain_scheduled` sweep.
    #[test]
    fn a_crossing_scheduled_when_a_different_partition_fails_in_the_same_event_is_not_pushed_later() {
        let p = program(5, vec![
            source(0, 1, 0, 5.0),
            switch_dev(1, 1, 2, 0.0), // A's own re-aim trigger, parallel with R1.
            resistor(2, 1, 2, 1000.0),
            capacitor(3, 2, 0, 1e-6, 0.0, 0.0),
            source(4, 3, 0, 1e306),
            switch_dev(5, 3, 4, 0.0), // B's own switch: fixed node3 (1e306V) directly to free node4.
            capacitor(6, 4, 0, 1e-6, 0.0, 0.0),
        ]);
        let pb = partition::build(&p).unwrap();
        let (a_idx, _) = pb.local_index[2].expect("A's node is free");
        let (b_idx, _) = pb.local_index[4].expect("B's node is free");
        assert_eq!(pb.kinds[a_idx], PartitionKind::ClosedForm);
        assert_eq!(pb.kinds[b_idx], PartitionKind::ClosedForm);

        // CONTROL: the identical topology and sequence, but B's switch is
        // never closed at all — exactly the crossing schedule a clean,
        // leak-free re-aim must produce. Compared against below rather than
        // a hand-picked magic number, since the exact analytic crossing
        // time is not the point of this test (`closed_form`'s own tests
        // already cover that); whether a stale entry distorts it is.
        let control_solver = PartitionedSolver::build(&p).unwrap();
        let all = all_partitions(&control_solver);
        let mut control = Kernel::new(control_solver, NoDevices);
        control.settle(0.0, &all, &[]).unwrap();
        assert!(control.solver_mut().watch_node(2, DeviceId(0), Box::new(|_v: &[f64]| vec![Threshold { value: 3.0, rising: true }])));
        control.apply_params_as_event(&[(1, 0, 0.0)]).unwrap(); // the same "no-op" re-aim as below.
        let control_next = control.next_event_time();
        assert!(control_next.is_finite(), "the control kernel must schedule a real crossing");

        let solver = PartitionedSolver::build(&p).unwrap();
        let mut kernel = Kernel::new(solver, NoDevices);
        kernel.settle(0.0, &all, &[]).unwrap();
        assert!(kernel.solver_mut().watch_node(2, DeviceId(0), Box::new(|_v: &[f64]| vec![Threshold { value: 3.0, rising: true }])));

        // Close A's switch (re-aims A, scheduling a real crossing) AND B's
        // switch (overflows B's segment) as ONE event: A's crossing is
        // already drained into `pending_schedule` by the time B's poisoned
        // voltage is caught.
        let err = kernel.apply_params_as_event(&[(1, 0, 1.0), (5, 0, 1.0)]).unwrap_err();
        assert!(matches!(err, crate::kernel::KernelError::SolveFailed(_)));

        // The SAME "no-op" re-aim the control kernel used (setting A's
        // switch to the value it already holds still dirties the partition
        // and re-aims it, `PartitionedSolver::apply_params`'s own
        // unconditional `touched.insert`). If the failed attempt's crossing
        // leaked into `pending_schedule`, THIS re-aim's own
        // `drain_scheduled` sweeps it up alongside the fresh, legitimate one
        // — `next_event_time` would then report the stale (much earlier,
        // fast-segment) time instead of matching the control kernel's clean
        // one.
        kernel.apply_params_as_event(&[(1, 0, 0.0)]).unwrap();
        let next = kernel.next_event_time();
        assert_eq!(next, control_next, "a crossing scheduled during the failed attempt must not survive to distort a later, otherwise-identical re-aim's own schedule");
    }

    // ---- rule 2a: a capacitor across two fixed nodes -----------------------
    //
    // core/README.md's rule 2a: a decoupling capacitor straight across an
    // ideal supply belongs to no partition (both its terminals are fixed,
    // distinct nodes) but is not `-3` — it evolves on its own from the fixed
    // `dV` between its terminals. `fixed_pair_program` below is exactly that
    // shape: `node1` fixed at `v0` by `source`, `node0` = gnd, `capacitor`
    // straight across them.

    fn fixed_pair_program(v0: f64, c: f64, esr: f64, leak: f64) -> Program {
        program(2, vec![source(0, 1, 0, v0), capacitor(1, 1, 0, c, esr, leak)])
    }

    #[test]
    fn fixed_pair_capacitor_esr_matches_the_exact_exponential_within_1e9_relative() {
        let (v0, esr, c) = (5.0, 0.5, 1e-6);
        let p = fixed_pair_program(v0, c, esr, 0.0);
        let mut solver = PartitionedSolver::build(&p).unwrap();
        solver.resolve_dirty(&[], 0.0, &[]).unwrap();

        solver.prepare_readout();
        let expected_at_load = v0 / esr; // dV/esr, exactly (the contract's own "5/0.5=10").
        let at_load = solver.currents()[1];
        assert!((at_load - expected_at_load).abs() / expected_at_load.abs() < 1e-9, "at load: got {at_load} expected {expected_at_load}");

        let tau = esr * c; // 0.5 us.
        for &t in &[1e-6, 5e-6, 20e-6] {
            solver.sync_dynamic_to(t).unwrap();
            solver.prepare_readout();
            let got = solver.currents()[1];
            let expected = (v0 / esr) * (-t / tau).exp();
            let rel = (got - expected).abs() / expected.abs();
            assert!(rel < 1e-9, "t={t}: got {got} expected {expected} rel {rel}");
        }
    }

    #[test]
    fn fixed_pair_capacitor_esr_zero_gives_zero_series_current_from_load_on() {
        // esr=0: vc=dV from the load instant on (core/README.md rule 2a), so
        // the series current (dV-vc)/esr is exactly 0 — not the classic
        // known-divergence artifacts (`C*dV/1e-12` at load, `C*dV/h` on the
        // first step) the contract explicitly calls out as classic's OWN
        // step-size quirk, never the core's.
        let p = fixed_pair_program(5.0, 1e-6, 0.0, 0.0);
        let mut solver = PartitionedSolver::build(&p).unwrap();
        solver.resolve_dirty(&[], 0.0, &[]).unwrap();
        solver.prepare_readout();
        assert_eq!(solver.currents()[1], 0.0, "esr=0: series current must be exactly 0 at load");

        solver.sync_dynamic_to(1e-3).unwrap();
        solver.prepare_readout();
        assert_eq!(solver.currents()[1], 0.0, "esr=0: series current must stay exactly 0 (vc=dV) at any later time");
    }

    #[test]
    fn fixed_pair_capacitor_publishes_series_plus_leakage_current() {
        // Matches the contract's own worked example: 5V/1uF/esr=0.5,
        // leakage=1e6 -> steady c1 = 5e-6 A once the series term decays away.
        let (v0, esr, c, leak) = (5.0, 0.5, 1e-6, 1e6);
        let p = fixed_pair_program(v0, c, esr, leak);
        let mut solver = PartitionedSolver::build(&p).unwrap();
        solver.resolve_dirty(&[], 0.0, &[]).unwrap();

        solver.prepare_readout();
        let expected_leak = v0 / leak;
        let expected_series_at_load = v0 / esr;
        let got = solver.currents()[1];
        assert!((got - (expected_series_at_load + expected_leak)).abs() < 1e-9, "at load: got {got} expected series {expected_series_at_load} + leak {expected_leak}");

        solver.sync_dynamic_to(1e-2).unwrap(); // ~2e4 time constants: series fully decayed.
        solver.prepare_readout();
        let steady = solver.currents()[1];
        assert!((steady - expected_leak).abs() < 1e-9, "steady state should be exactly the leakage term, got {steady} expected {expected_leak}");
    }

    #[test]
    fn fixed_pair_capacitors_current_enters_the_fixing_sources_kcl() {
        let (v0, esr, c) = (5.0, 0.5, 1e-6);
        let p = fixed_pair_program(v0, c, esr, 0.0);
        let mut solver = PartitionedSolver::build(&p).unwrap();
        solver.resolve_dirty(&[], 0.0, &[]).unwrap();

        // node1 (the source's + terminal) also carries the universal node
        // shunt (W0 point 2), included in every KCL sum — `recompute_source_
        // currents`'s own `leaving` walk folds both this and the capacitor's
        // published current together before deriving the source's current.
        solver.prepare_readout();
        let cap_i = solver.currents()[1];
        let shunt = static_part::NODE_RSHUNT_G;
        let expected_source_i = -(shunt * v0 + cap_i);
        assert!(
            (solver.currents()[0] - expected_source_i).abs() < 1e-9,
            "source current {} does not fold in the capacitor's {cap_i} (expected {expected_source_i})",
            solver.currents()[0]
        );
    }

    #[test]
    fn fixed_pair_capacitor_start_time_5_matches_start_time_0() {
        let (v0, esr, c) = (5.0, 0.5, 1e-6);
        let build_at = |start_time: f64| {
            let mut options = Options::default();
            options.start_time = start_time;
            let p = Program { node_count: 2, options, digital_slot_count: 0, channel_slot_count: 0, devices: vec![source(0, 1, 0, v0), capacitor(1, 1, 0, c, esr, 0.0)] };
            let mut solver = PartitionedSolver::build(&p).unwrap();
            solver.resolve_dirty(&[], start_time, &[]).unwrap();
            solver.prepare_readout();
            solver
        };
        let mut s0 = build_at(0.0);
        let mut s5 = build_at(5.0);
        assert_eq!(
            s0.currents()[1].to_bits(),
            s5.currents()[1].to_bits(),
            "the load-instant current must match bit-for-bit regardless of START_TIME — both anchor vc=0 at their own START_TIME"
        );

        // And the trajectory itself tracks the same shifted exponential
        // later, not merely at the load instant — a RELATIVE, not bit-exact,
        // comparison: `now - start_time` for START_TIME=5 is not bit-
        // identical to the same delta computed from START_TIME=0, an
        // ordinary floating-point subtraction artifact, not a bug.
        s0.sync_dynamic_to(5e-6).unwrap();
        s5.sync_dynamic_to(5.0 + 5e-6).unwrap();
        s0.prepare_readout();
        s5.prepare_readout();
        let rel = (s5.currents()[1] - s0.currents()[1]).abs() / s0.currents()[1].abs();
        assert!(rel < 1e-9, "5us after their own START_TIME: START_TIME=0 got {} START_TIME=5 got {} rel {rel}", s0.currents()[1], s5.currents()[1]);
    }

    #[test]
    fn a_fixed_pair_capacitors_current_rolls_back_with_the_instant() {
        // Rule 2a's last bullet ("it rolls back with the rest of the
        // instant") — this device carries no `dynamic_snapshot` state of its
        // own (`FixedPairCapacitor`'s doc comment: it is a pure function of
        // `now`), so its only mutable trace is `currents[device_index]`,
        // already covered by the plain snapshot/restore `Kernel::advance`
        // wraps every instant in. Proven directly against that same pair of
        // calls, exactly as `kernel.rs`'s `advance` uses them.
        let (v0, esr, c) = (5.0, 0.5, 1e-6);
        let p = fixed_pair_program(v0, c, esr, 0.0);
        let mut solver = PartitionedSolver::build(&p).unwrap();
        solver.resolve_dirty(&[], 0.0, &[]).unwrap();
        solver.prepare_readout();
        let at_load = solver.currents()[1];
        assert!((at_load - v0 / esr).abs() < 1e-9);

        let snapshot = solver.snapshot();
        solver.sync_dynamic_to(1e-5).unwrap(); // ~20 time constants later: fully decayed.
        solver.prepare_readout();
        let decayed = solver.currents()[1];
        assert!(decayed.abs() < 1e-6, "expected the series current to have decayed to ~0 by 1e-5s, got {decayed}");

        solver.restore(snapshot);
        assert_eq!(
            solver.currents()[1].to_bits(),
            at_load.to_bits(),
            "a rollback must restore the fixed-pair capacitor's published current to its pre-attempt value, not leave the decayed one in place"
        );
    }

    // ---- deferred readout (`publish_readout`, `prepare_readout`) ------------

    /// One program with every kind of derived value that needs no IC: a
    /// source whose current folds in a resistor, a switch, three capacitors'
    /// currents (an RC, a fixed-pair one) and two LED junctions; an RGB LED
    /// (channels) across the fixed rail; a plain LED inside the RC's
    /// partition.
    fn readout_program() -> Program {
        let led = |ci: usize, anode: usize, cathode: usize| Device {
            kind: kinds::LED,
            component_index: ci,
            pins: vec![pin(anode), pin(cathode)],
            params: vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0],
            blob: vec![],
        };
        let rgb = Device {
            kind: kinds::RGB_LED,
            component_index: 7,
            pins: vec![pin(1), pin(1), pin(1), pin(0)],
            params: vec![2.0, 2.1, 3.0, 0.02, 0.03, 0.0],
            blob: vec![0.0],
        };
        Program {
            node_count: 3,
            options: Options::default(),
            digital_slot_count: 0,
            channel_slot_count: 3,
            devices: vec![
                source(0, 1, 0, 5.0),
                resistor(1, 1, 2, 1000.0),
                capacitor(2, 2, 0, 1e-6, 0.0, 0.0),
                switch_dev(3, 1, 2, 0.0),
                capacitor(4, 1, 0, 1e-6, 0.5, 1e7),
                rgb,
                led(6, 2, 0),
            ],
        }
    }

    /// The program above plus the readout inputs that only an IC has: a
    /// 74ls04 whose `1a` is an open floating-input pin on the RAIL (its pull
    /// enters the source KCL, and depends on time), whose `1y` drives an RC
    /// and whose `2y` drives the rail itself (a driver folded into the
    /// source KCL); and an ne555 whose OUT level (the latch, read through
    /// the driver table) sets its `elementI`. Devices: 0 source, 1 ls04, 2 R,
    /// 3 C, 4 ne555, 5 and 6 its load and DIS resistors, 7 a fixed-pair C.
    fn ic_readout_program(open_hash: u32) -> Program {
        let flags = |i: usize| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 };
        let mut ls04_pins = vec![pin(1), pin(0)];
        for (i, node) in [1usize, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0].into_iter().enumerate() {
            ls04_pins.push(if i == 0 { PinRecord { node, flags: crate::program::PIN_FLAG_OPEN, open_hash, dead_hash: 0 } } else { pin(node) });
        }
        let mut ls04_params = vec![0.0; kinds::ic_param::COUNT];
        ls04_params[kinds::ic_param::VCC_MIN] = 4.75;
        ls04_params[kinds::ic_param::VCC_MAX] = 5.25;
        ls04_params[kinds::ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
        ls04_params[kinds::ic_param::VIL_FIXED] = 1.0;
        ls04_params[kinds::ic_param::VIL] = 0.8;
        ls04_params[kinds::ic_param::VIH_FIXED] = 1.0;
        ls04_params[kinds::ic_param::VIH] = 2.0;
        ls04_params[kinds::ic_param::NOMINAL] = 5.0;
        ls04_params[kinds::ic_param::IO_MAX] = 0.008;
        ls04_params[kinds::ic_param::DELAY] = 20e-9;
        let mut ls04_blob = vec![0.0];
        ls04_blob.extend((0..12).map(flags));
        let ls04 = Device { kind: kinds::IC_74LS04, component_index: 1, pins: ls04_pins, params: ls04_params, blob: ls04_blob };

        let mut ne555_params = vec![0.0; kinds::ic_param::COUNT];
        ne555_params[kinds::ic_param::VCC_MIN] = 2.0;
        ne555_params[kinds::ic_param::VCC_MAX] = 18.0;
        ne555_params[kinds::ic_param::NOMINAL] = 5.0;
        // `8 1` then TRIG, OUT, RST, CV, THR, DIS.
        let ne555_pins = [1, 0, 0, 3, 1, 0, 0, 4].into_iter().map(pin).collect();
        let ne555 = Device { kind: kinds::NE555, component_index: 4, pins: ne555_pins, params: ne555_params, blob: vec![0.0; 7] };

        Program {
            node_count: 5,
            options: Options::default(),
            digital_slot_count: 0,
            channel_slot_count: 0,
            devices: vec![
                source(0, 1, 0, 5.0),
                ls04,
                resistor(2, 2, 0, 1000.0),
                capacitor(3, 2, 0, 1e-6, 0.0, 0.0),
                ne555,
                resistor(5, 3, 0, 1000.0),
                resistor(6, 4, 1, 10000.0),
                capacitor(7, 1, 0, 1e-6, 0.5, 1e7),
            ],
        }
    }

    /// The `ic_readout_program`'s driver table: the ls04's `1y` and `2y`, the
    /// 555's OUT and DIS.
    fn ic_readout_drivers() -> Vec<Driver> {
        let mut out = Driver::new_ic_output(3, 1, 0, 4, false);
        out.stage = DriverStage::Ne555Out;
        let mut dis = Driver::new_ic_output(4, 1, 0, 4, false);
        dis.stage = DriverStage::Ne555Dis;
        vec![Driver::new_ic_output(2, 1, 0, 1, false), Driver::new_ic_output(1, 1, 0, 1, false), out, dis]
    }

    /// Commits `levels[i]` on `drivers[i]` at `now`, where it changes.
    fn commit_levels(drivers: &mut [Driver], levels: [Mode; 4], now: f64) {
        for (d, level) in drivers.iter_mut().zip(levels) {
            if d.committed() != Some(level) {
                let (_, mode, token) = d.request(level, now, &crate::drivers::DelaySet::uniform(0.0)).expect("a level change schedules a commit");
                assert!(d.apply_commit(mode, token));
            }
        }
    }

    /// Every published number of `s`, as bits, after bringing its readout up
    /// to date.
    fn published_bits(s: &mut PartitionedSolver) -> Vec<u64> {
        s.prepare_readout();
        s.voltages().iter().chain(s.currents()).chain(s.channels()).map(|v| v.to_bits()).collect()
    }

    /// A lazy solver and its twin with the deferral forced off (the eager
    /// passes at every publish point: the reference).
    fn lazy_and_eager(p: &Program) -> (PartitionedSolver, PartitionedSolver) {
        let lazy = PartitionedSolver::build(p).unwrap();
        let mut eager = PartitionedSolver::build(p).unwrap();
        assert!(lazy.lazy_static_ok, "this program's coefficients are all bounded");
        eager.lazy_static_ok = false;
        (lazy, eager)
    }

    #[test]
    fn a_deferred_readout_matches_the_eager_one_bit_for_bit_and_rolls_back_with_the_instant() {
        let p = readout_program();
        let (mut lazy, mut eager) = lazy_and_eager(&p);
        let all = all_partitions(&lazy);

        for s in [&mut lazy, &mut eager] {
            s.resolve_dirty(&all, 0.0, &[]).unwrap();
        }
        assert!(lazy.derived_dirty, "the lazy solver owes its readout until it is prepared");
        assert!(!eager.derived_dirty);
        assert_eq!(published_bits(&mut lazy), published_bits(&mut eager), "at load");
        assert!(!lazy.derived_dirty, "preparing runs the owed pass");
        assert!(lazy.currents()[0] != 0.0 && lazy.channels().iter().any(|&c| c > 0.0), "the readout must be non-trivial");

        for t in [1e-4, 3e-4, 1e-3] {
            for s in [&mut lazy, &mut eager] {
                s.sync_dynamic_to(t).unwrap();
            }
            assert!(lazy.derived_dirty);
            assert_eq!(published_bits(&mut lazy), published_bits(&mut eager), "after a horizon clamp at {t}");
        }

        // A rollback taken while the readout is still owed (nothing has read
        // it since the last publish), then one taken after a read.
        for read_before_snapshot in [false, true] {
            for s in [&mut lazy, &mut eager] {
                s.sync_dynamic_to(2e-3).unwrap();
            }
            let expected = published_bits(&mut eager);
            if read_before_snapshot {
                assert_eq!(published_bits(&mut lazy), expected);
            }
            assert_eq!(lazy.derived_dirty, !read_before_snapshot);
            let (snapshot, dynamic) = (lazy.snapshot(), lazy.dynamic_snapshot());

            // The doomed attempt: a later publish, a toggled switch, and a read.
            let touched = lazy.apply_params(&[(3, 0, 1.0)]);
            lazy.resolve_dirty(&touched, 2.5e-3, &[]).unwrap();
            lazy.sync_dynamic_to(3e-3).unwrap();
            assert_ne!(published_bits(&mut lazy), expected, "the attempt must have changed the published state");

            lazy.restore(snapshot);
            lazy.dynamic_restore(dynamic);
            lazy.apply_params(&[(3, 0, 0.0)]);
            assert_eq!(published_bits(&mut lazy), expected, "a rollback must restore the readout it took (read first: {read_before_snapshot})");
        }

        // A live-param change publishes its new readout.
        for s in [&mut lazy, &mut eager] {
            let touched = s.apply_params(&[(3, 0, 1.0)]);
            s.resolve_dirty(&touched, 2e-3, &[]).unwrap();
        }
        assert_eq!(published_bits(&mut lazy), published_bits(&mut eager), "after closing the switch");
    }

    #[test]
    fn a_deferred_readout_with_ics_and_a_555_matches_the_eager_one_across_commits_time_and_a_rolled_back_instant() {
        use crate::events::Mode::{High, Low};
        // A floating-pull hash whose coin comes up heads, tails, heads, tails
        // over the 10 ms buckets at these four times (far apart, since
        // neighbouring buckets' coins are nearly equal): the rail's source
        // current then differs between them by the pull, so a wrong
        // `derived_now` shows.
        let times = [0.003, 0.173, 0.427, 0.913];
        let open_hash = (0u32..100_000)
            .find(|&h| times.iter().map(|&t| digital::floating::floating_high(h, t, true)).collect::<Vec<_>>() == [true, false, true, false])
            .expect("some hash flips at each of these times");
        let p = ic_readout_program(open_hash);
        let (mut lazy, mut eager) = lazy_and_eager(&p);
        let all = all_partitions(&lazy);
        let mut drivers = ic_readout_drivers();
        let mut source_currents = Vec::new();
        fn check(lazy: &mut PartitionedSolver, eager: &mut PartitionedSolver, what: &str, log: &mut Vec<u64>) {
            assert_eq!(published_bits(lazy), published_bits(eager), "{what}");
            log.push(lazy.currents()[0].to_bits());
        }

        for s in [&mut lazy, &mut eager] {
            s.resolve_dirty(&all, 0.0, &drivers).unwrap();
        }
        assert!(lazy.derived_dirty && !eager.derived_dirty);
        check(&mut lazy, &mut eager, "at load, every driver uncommitted", &mut source_currents);

        // A commit between publishes, then time alone, then a toggle.
        commit_levels(&mut drivers, [High, Low, High, Low], times[0]);
        for s in [&mut lazy, &mut eager] {
            s.resolve_dirty(&all, times[0], &drivers).unwrap();
        }
        check(&mut lazy, &mut eager, "after the first commits", &mut source_currents);
        assert!(lazy.currents()[4] != 0.0, "the 555 delivers current while OUT is committed HIGH");
        for s in [&mut lazy, &mut eager] {
            s.sync_dynamic_to(times[1]).unwrap();
        }
        check(&mut lazy, &mut eager, "after a horizon clamp into the next bucket", &mut source_currents);
        commit_levels(&mut drivers, [Low, High, Low, High], times[2]);
        for s in [&mut lazy, &mut eager] {
            s.resolve_dirty(&all, times[2], &drivers).unwrap();
        }
        check(&mut lazy, &mut eager, "after every commit toggled", &mut source_currents);
        let distinct: std::collections::BTreeSet<u64> = source_currents.iter().copied().collect();
        assert_eq!(distinct.len(), source_currents.len(), "each step must have moved the source current: {source_currents:?}");

        // A doomed instant of two delta passes with commit changes, rolled
        // back, taken while the readout is owed and again after a read.
        for read_before_snapshot in [false, true] {
            for s in [&mut lazy, &mut eager] {
                s.sync_dynamic_to(times[2] + 1e-3).unwrap();
            }
            let expected = published_bits(&mut eager);
            if read_before_snapshot {
                assert_eq!(published_bits(&mut lazy), expected);
            }
            assert_eq!(lazy.derived_dirty, !read_before_snapshot);
            let (snapshot, dynamic) = (lazy.snapshot(), lazy.dynamic_snapshot());

            let levels = |table: &[Driver]| table.iter().map(|d| d.committed()).collect::<Vec<_>>();
            let before = levels(&drivers);
            let mut doomed = drivers.clone();
            commit_levels(&mut doomed, [High, High, High, Low], times[3]);
            lazy.resolve_dirty(&all, times[3], &doomed).unwrap();
            assert!(
                std::rc::Rc::strong_count(&lazy.pre_commit_drivers) > 1,
                "the table the snapshot holds is now the pre-commit one: the next resolve must copy over the other buffer, not write into it"
            );
            assert_eq!((levels(&lazy.driver_snapshot), levels(&lazy.pre_commit_drivers)), (levels(&doomed), before));
            let first = levels(&doomed);
            commit_levels(&mut doomed, [Low, Low, Low, High], times[3]);
            lazy.resolve_dirty(&all, times[3], &doomed).unwrap();
            assert_eq!((levels(&lazy.driver_snapshot), levels(&lazy.pre_commit_drivers)), (levels(&doomed), first));
            lazy.sync_dynamic_to(times[3] + 1e-3).unwrap();
            assert_ne!(published_bits(&mut lazy), expected, "the attempt must have changed the published state");

            lazy.restore(snapshot);
            lazy.dynamic_restore(dynamic);
            assert_eq!(published_bits(&mut lazy), expected, "a rollback must restore the readout it took (read first: {read_before_snapshot})");
        }

        // Both go on from there, from the same state.
        commit_levels(&mut drivers, [High, Low, High, Low], times[3]);
        for s in [&mut lazy, &mut eager] {
            s.resolve_dirty(&all, times[3], &drivers).unwrap();
        }
        check(&mut lazy, &mut eager, "after the rolled-back instants, at the next publish", &mut source_currents);
    }

    #[test]
    fn a_coefficient_that_could_overflow_selects_the_eager_readout() {
        // 1/R for R = 1e-320 is infinite, and a bound on it cannot be proved.
        let p = program(2, vec![source(0, 1, 0, 5.0), resistor(1, 1, 0, 1e-320)]);
        assert!(!PartitionedSolver::build(&p).unwrap().lazy_static_ok);
        let p = program(2, vec![source(0, 1, 0, 5.0), resistor(1, 1, 0, 1000.0)]);
        assert!(PartitionedSolver::build(&p).unwrap().lazy_static_ok);
        // A node voltage past the limit selects it at run time.
        let mut s = PartitionedSolver::build(&p).unwrap();
        s.resolve_dirty(&[], 0.0, &[]).unwrap();
        assert!(s.derived_dirty);
        s.voltages[1] = 2.0 * LAZY_V_LIMIT;
        // What every write site does with the value it writes.
        fold_bound(&mut s.v_hint, s.voltages[1]);
        assert!(!s.lazy_publish_ok(0.0));
    }

    #[test]
    #[should_panic(expected = "call prepare_readout() first")]
    fn reading_the_currents_while_the_readout_is_owed_is_a_bug_that_fails_loudly() {
        let p = program(2, vec![source(0, 1, 0, 5.0), resistor(1, 1, 0, 1000.0)]);
        let mut s = PartitionedSolver::build(&p).unwrap();
        s.resolve_dirty(&[], 0.0, &[]).unwrap();
        let _ = s.currents();
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Threshold {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rising, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DynPin {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.local, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DynStamped {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pins, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.params, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DynCapacitor {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pins, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.esr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.leak_g, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DynJunction {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.anode, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cathode, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.law, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DynShape {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.n, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stamped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.capacitors, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.junctions, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for ExactStamp {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.partition, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.local, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for ExactKind {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            ExactKind::Norton { g, inject } => crate::snapshot::HeapBytes::heap_bytes(g, w) + crate::snapshot::HeapBytes::heap_bytes(inject, w),
            ExactKind::Row { value } => crate::snapshot::HeapBytes::heap_bytes(value, w),
            ExactKind::Inject { current } => crate::snapshot::HeapBytes::heap_bytes(current, w),
        }
    }
}

impl crate::snapshot::HeapBytes for RailSens {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.jac, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.res, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.gross, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.gross_s, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.lu, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Owner {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Owner::Static => 0,
            Owner::ClosedForm(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            Owner::Stepped(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for FixedPairCapacitor {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.d_v, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.esr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.leak_g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for KclDevice {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.owner, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.contributions, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for LedDevice {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.junctions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.channel_base, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.channel_count, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SwitchInfo {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pin_count, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.owner, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DynOut {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.devices, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.caps, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PartitionedSolver {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.voltages, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.currents, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.resolved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.local_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.partition_kinds, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.static_nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.static_solver, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.closed_form, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stepped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kcl_devices, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.static_kcl_devices, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.switches, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.live_params, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cap_devices, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.fixed_pair_capacitors, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.start_time, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ics, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.next_bucket, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending_schedule, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.largest_partition, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.multi_capacitor_linear_partitions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.junctions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.led_devices, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.channels, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.derived_dirty, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.derived_now, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.lazy_static_ok, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.partition_cap_devices, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kcl_scratch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ic_position, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kcl_position, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pre_commit_drivers, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.driver_snapshot, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_hint, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i_hint, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.derived_devices, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dyn_out, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cf_parts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stepped_parts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.written, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.static_ids, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.readers_buf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cf_ids, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.st_ids, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ic_driver_ids, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ic_driver_ids_len, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.watch_dependents, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.window, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_log, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i_log, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dyn_logged, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dyn_logged_list, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.derived_logged, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_pending_schedule, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_next_bucket, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_driver_snapshot, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_derived, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_channels, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mirror_seen, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mirror_live, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mirror_valid, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stale, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.call_patches, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.d_log, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.d_log_split, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_stale, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.window_mirror, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_saved, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.bill_log, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.wave, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_wave, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.wave_entries, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.wave_parts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_parts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_watch_parts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_ic_readers, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_caps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.live_caps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_caps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.fixed_node, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device_kinds, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.exact, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc_switch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind_log, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc_crawl, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_cc_crawl, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.source_edges, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.exact_solved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.landing_hold, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slow, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slow_model, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slow_scratch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slow_counts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds_active, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.probe_version, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.probes_at, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds_flag, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_holds, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hold_counts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.episodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.retry_armed, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.at_clamp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_jump, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.host_edit, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.host_step, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.changes, w); }
        n
    }
}
