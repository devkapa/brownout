//! The event kernel: the event loop, delta cycles and load settling
//! (core/README.md, "Kernel semantics: events"). Owned by W1a worker A1.
//!
//! `Kernel<S, E>` is generic over two traits so this module stays standalone
//! this phase (no `Program`, no concrete devices): [`solve::PartitionSolver`]
//! re-solves whatever partitions a driver commit marked dirty and reports
//! which readers need re-evaluating, and [`DeviceEvaluator`] runs one reader
//! and reports the driver-level changes it wants. `Kernel` itself owns the
//! only two things every wave shares regardless of which devices exist yet:
//! the event heap ([`events::EventHeap`]) and the driver table
//! ([`drivers::Driver`]).
//!
//! Two entry points:
//! - [`Kernel::advance`] processes every event up to `t_end` through the
//!   two-phase delta cycle (contract point 3, decision 4): apply due
//!   commits, re-solve, evaluate affected readers once, repeat until the
//!   instant is quiet or the pass cap trips [`KernelError::Livelock`].
//! - [`Kernel::settle`] is decision 8's load-time fixed point: edge history
//!   frozen, then exactly one real edge pass, then idempotent follow-ups.
//!
//! `engine.rs` wires this in behind `StaticPartitionSolver` (the W0 static
//! kernel adapted to [`solve::PartitionSolver`]) and a no-op
//! [`DeviceEvaluator`], mapping [`KernelError`] onto `BoError` (`Livelock` to
//! `-7`, `SolveFailed` to `-4`).

use std::collections::BTreeSet;

use crate::drivers::Driver;
use crate::events::{DeviceId, DriverId, EventHeap, EventKind, Mode, PartitionId, Token, Version};
use crate::solve::PartitionSolver;

/// The per-instant delta-pass cap (contract point 3: "More than 1000 deltas
/// at one instant is an error"). Also used as [`Kernel::settle`]'s pass cap:
/// the contract does not name a separate figure for settling, and an
/// unconverging settle is the same failure mode (a network with a zero-delay
/// loop) as an unconverging delta cycle, so it is guarded the same way
/// rather than left uncapped or given an arbitrary smaller number.
pub const DELTA_CAP: u32 = 1000;

/// The crossing-acceptance margin (contract point 4; "Hard-won rules":
/// rounding can put a node back exactly on its threshold and re-schedule it
/// forever if accepted at `t == now`).
pub const CROSSING_ACCEPT_EPS: f64 = 1e-15;

/// Everything that can go wrong inside the kernel itself. This is
/// deliberately not `error::BoError` (kept independent of `error.rs` so this
/// module has no dependency on it): `engine.rs` maps both variants onto a
/// `BoError`.
#[derive(Debug, Clone, PartialEq)]
pub enum KernelError {
    /// [`DELTA_CAP`] (or, during [`Kernel::settle`], the same cap applied to
    /// settle passes) was exceeded without reaching a quiet state: a
    /// zero-delay loop that never settles. `at` is the simulation time this
    /// happened at. Never returned silently — pec3.mjs's `delta < 1000` loop
    /// (`:766`) simply exits and carries on as if converged; this is the
    /// mistake the contract calls out by name and this type exists to make
    /// impossible to repeat. Maps to ABI `-7`.
    Livelock { at: f64 },
    /// A [`solve::PartitionSolver::resolve_dirty`] call failed (a singular or
    /// non-finite pivot, or a non-finite published value) — the kernel-level
    /// counterpart of `ERR_SOLVE_FAILED`. Maps to ABI `-4`.
    SolveFailed(String),
}

/// The kernel's own live counters (contract, "Stat indices" 4, 5, 6 and 13 —
/// 7-12 and 14 belong to machinery this phase does not build). Only
/// [`Kernel::advance`]'s real delta cycle updates these: [`Kernel::settle`]'s
/// passes are a distinct, load-time-only procedure (decision 8), and the
/// prototype itself draws the same line — `init()`'s own inline commit
/// handling never calls `_applyCommit`, the function that increments
/// `stats.commits`/`stats.events`, so settling was never counted there
/// either. `engine.rs` reads these directly for `bo_engine_stat`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct KernelStats {
    /// Every event popped from the heap during `advance`, whatever kind or
    /// however stale (contract stat `4`).
    pub events_processed: u64,
    /// Every delta-cycle pass `advance` has run, across every instant
    /// (contract stat `5`).
    pub delta_passes: u64,
    /// The most delta passes any single instant has needed so far (contract
    /// stat `6`).
    pub max_delta_depth: u64,
    /// Every `Commit` that actually changed a driver's committed level
    /// (contract stat `13`) — a token match whose mode was already current
    /// does not count, matching the prototype's own `if (d.mode === ev.mode)
    /// return;` guard ahead of `stats.commits++`.
    pub driver_commits: u64,
}

/// One driver-level change a reader's evaluation produced this delta.
/// `DeviceEvaluator` implementations return these instead of touching a
/// `Driver` directly, so `Kernel` stays the sole owner of the driver table
/// and the event heap.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DriverRequest {
    /// Request `driver` reach `level` through the inertial FSM
    /// (`drivers::fsm::request`), with propagation delay `delay` — the
    /// reader's own kind's datasheet or catalog figure. The kernel neither
    /// knows nor needs to know what it is; it is carried on the request
    /// rather than stored on `Driver` itself. During [`Kernel::settle`],
    /// `delay` is ignored and treated as zero throughout (decision 8;
    /// pec3-w1's `_seeding` forces every commit to zero delay while the
    /// network settles at load, not only on a driver's first observation).
    Level { driver: DriverId, level: Mode, delay: f64 },
    /// The reader did not drive `driver` at all this evaluation (a
    /// tri-stated bus, a direction flip on a bidirectional part): drop its
    /// delay history outright (`drivers::fsm::reset`) instead of
    /// transitioning it through the ordinary delayed path, so the next
    /// request against it is again a first observation (spec 2.4's cleanup
    /// pass, "drops the triad for any pin missing from `immediate`").
    Reset { driver: DriverId },
}

/// Evaluates one reader (a device with at least one input) once per delta.
///
/// `acting_on_edges` is `false` only while [`Kernel::settle`] is freezing
/// edge history (decision 8): an implementation must still drive its
/// outputs at their current committed state, but must not treat any input
/// transition as an edge, and must not advance whatever edge-history state
/// it keeps for itself (a `last_clk`-shaped flag). The kernel has no
/// visibility into a reader's internal state and cannot enforce "exactly one
/// edge pass" on its behalf; that guarantee comes entirely from an
/// implementation only ever updating its own edge-history bookkeeping when
/// `acting_on_edges` is `true` — the kernel's contract is only to call
/// `evaluate` with `false` for every settle pass before the first `true`
/// pass, and with `true` for every call after that, including every
/// ordinary delta during `advance`.
///
/// `voltages` is the solver's currently-published node-voltage array
/// (`solve::PartitionSolver::voltages`, W1c contract point 4: "an IC reads
/// its supply nodes and every logic pin's node"), handed straight through so
/// an implementation needs no back-channel of its own to the solver just to
/// read a pin.
pub trait DeviceEvaluator {
    fn evaluate(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64]) -> Vec<DriverRequest>;

    /// An owned copy of every piece of state this evaluator would need to
    /// undo a failed instant (core/README.md, "A failed advance") — Schmitt
    /// per-pin hysteresis state, register contents, anything else an
    /// evaluation mutates in place. `Box<dyn Any>`, exactly like
    /// [`solve::PartitionSolver::dynamic_snapshot`], for the same reason:
    /// each evaluator's own state has its own shape, and only the concrete
    /// implementation that produced the box ever downcasts it back. Default
    /// `Box::new(())`: an evaluator with no mutable state of its own (every
    /// one through W1b, `engine.rs`'s `NoDevices` included) has nothing to
    /// roll back.
    fn snapshot(&self) -> Box<dyn std::any::Any> {
        Box::new(())
    }

    /// Restores exactly what a prior [`Self::snapshot`] call captured.
    /// Default a no-op, paired with the default snapshot above.
    fn restore(&mut self, snapshot: Box<dyn std::any::Any>) {
        let _ = snapshot;
    }
}

/// So `Kernel<_, Box<dyn DeviceEvaluator>>` satisfies the trait bound
/// directly — see `solve::PartitionSolver`'s matching impl for why
/// `engine.rs` needs this.
impl DeviceEvaluator for Box<dyn DeviceEvaluator> {
    fn evaluate(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64]) -> Vec<DriverRequest> {
        (**self).evaluate(reader, now, acting_on_edges, voltages)
    }

    fn snapshot(&self) -> Box<dyn std::any::Any> {
        (**self).snapshot()
    }

    fn restore(&mut self, snapshot: Box<dyn std::any::Any>) {
        (**self).restore(snapshot)
    }
}

/// Whether a candidate threshold-crossing time is far enough past `now` to
/// accept (contract point 4). Exposed standalone (not only via
/// [`Kernel::schedule_crossing`]) because a closed-form solver (W1b) applies
/// the same rule when it re-aims a segment, before it ever reaches a heap.
pub fn accepts_crossing(candidate_t: f64, now: f64) -> bool {
    candidate_t > now + CROSSING_ACCEPT_EPS
}

/// The event kernel. Generic over the partition-solver and device-evaluation
/// traits so this module has no dependency on `Program` or any concrete
/// device (this phase's scope); see the module doc.
#[derive(Debug)]
pub struct Kernel<S: PartitionSolver, E: DeviceEvaluator> {
    heap: EventHeap,
    drivers: Vec<Driver>,
    /// Parallel to `drivers`: which partition each driver's node belongs to.
    /// Kept off `Driver` itself (whose fields are the ones core/README.md's
    /// W1a brief names — node, supply nodes, conductance, levels, mode,
    /// pending token, no partition) because partition membership is a
    /// property of current topology, owned by whatever assigns partitions
    /// (W1b's `partition.rs`), not an intrinsic fact about a driver.
    /// `None` for a driver whose own node is fixed (W1c contract point 4's
    /// "a driver whose node is fixed belongs to no partition"): it has
    /// nothing for a partition re-solve to do, so a commit against it never
    /// dirties anything here — its electrical contribution reaches the
    /// published state purely through the solver's own KCL fold
    /// (`solve::PartitionedSolver::recompute_source_currents`), not through
    /// [`solve::PartitionSolver::resolve_dirty`].
    driver_partition: Vec<Option<PartitionId>>,
    time: f64,
    solver: S,
    evaluator: E,
    stats: KernelStats,
    /// A `Commit` popped for a driver whose IC is currently unpowered (W1c
    /// contract point 9, owner decision 8/Q-O4): held here instead of
    /// applied, so the driver's committed level and any in-flight transition
    /// stay exactly as they were while the outage lasts. Retried onto the
    /// heap, at the current time, after every re-solve
    /// ([`Self::retry_held_commits`]) once [`solve::PartitionSolver::ic_powered`]
    /// reports the owning IC powered again — the ordinary stale-token check
    /// on the next pop is what makes a commit superseded WHILE held (a fresh
    /// request bumped the driver's token in the meantime) drop silently
    /// instead of resurrecting a stale level.
    held_commits: Vec<(DriverId, Mode, Token)>,
}

impl<S: PartitionSolver, E: DeviceEvaluator> Kernel<S, E> {
    pub fn new(solver: S, evaluator: E) -> Self {
        Kernel {
            heap: EventHeap::new(),
            drivers: Vec::new(),
            driver_partition: Vec::new(),
            time: 0.0,
            solver,
            evaluator,
            stats: KernelStats::default(),
            held_commits: Vec::new(),
        }
    }

    pub fn time(&self) -> f64 {
        self.time
    }

    /// The kernel's live counters (contract stats 4, 5, 6, 13); `engine.rs`
    /// reads these directly for `bo_engine_stat`.
    pub fn stats(&self) -> &KernelStats {
        &self.stats
    }

    /// Read access to the partition solver — `engine.rs` uses this to reach
    /// whatever state a concrete solver publishes (node voltages, device
    /// currents), which this generic module has no reason to know the shape
    /// of.
    pub fn solver(&self) -> &S {
        &self.solver
    }

    /// Mutable access to the partition solver — `engine.rs`'s
    /// `Engine::set_option` uses this to propagate `NO_SOLUTION_CACHE`
    /// (`PartitionSolver::set_no_solution_cache`) into whatever solver is
    /// actually behind the kernel, without this generic module needing to
    /// know that cache exists at all.
    pub fn solver_mut(&mut self) -> &mut S {
        &mut self.solver
    }

    /// Replaces the partition solver outright, keeping every driver, the
    /// heap and the live counters as they were. `engine.rs`'s `#[cfg(test)]`
    /// harness uses this to swap in a mock solver and prove `Engine::advance`
    /// propagates a livelock end to end; a real host could equally use it to
    /// move from a placeholder solver to a fully compiled one.
    pub fn set_solver(&mut self, solver: S) {
        self.solver = solver;
    }

    /// Replaces the device evaluator outright, for the same reason as
    /// [`Self::set_solver`].
    pub fn set_evaluator(&mut self, evaluator: E) {
        self.evaluator = evaluator;
    }

    /// The time of the earliest *pending* event, `+Infinity` when none —
    /// `bo_engine_next_event_time`'s contract (ABI v2). A cancelled inertial
    /// transition (`drivers::fsm`'s glitch-cancellation row: the token bump
    /// is the whole cancellation, and the `Commit` already in the heap for
    /// the old token is left exactly where it is — events.rs's lazy-deletion
    /// contract) is not pending; nothing will happen when it is popped. This
    /// peeks past any number of such stale entries and discards them for
    /// good — the same fate a real `advance` would give them on pop — rather
    /// than reporting a time nothing actually happens at, which would leave
    /// a host's step-clamp landing on a dead instant. Takes `&mut self`
    /// because discarding a stale entry mutates the heap; a stale-blind
    /// `&self` peek (the previous implementation) cannot tell a cancelled
    /// commit from a live one without popping it.
    pub fn next_event_time(&mut self) -> f64 {
        loop {
            let ev = match self.heap.pop() {
                Some(ev) => ev,
                None => return f64::INFINITY,
            };
            let live = match ev.kind {
                EventKind::Commit { driver, token, .. } => self.drivers[driver.0 as usize].token() == token,
                // A dynamic partition's own solve epoch (events.rs's
                // lazy-deletion contract): stale the instant a re-aim/replan
                // bumps the version this event was scheduled under (a switch
                // toggle, a driver commit, the horizon clamp's forced
                // resync, or the crossing/step this event itself represents
                // already having fired once under an older version).
                EventKind::Crossing { partition, version } => self.solver.is_current(partition, true, version),
                EventKind::Step { partition, version } => self.solver.is_current(partition, false, version),
                // Nothing in W1a-W1b schedules a `Timer` yet; treat it as
                // live rather than guess at a validation rule with no
                // producer to test it against.
                EventKind::Timer { .. } => true,
                // A `Bucket` is never stale by construction (events.rs's own
                // doc comment on the variant: the solver deduplicates so at
                // most one is ever pending) -- always live.
                EventKind::Bucket => true,
            };
            if live {
                let t = ev.time;
                self.heap.push_event(ev);
                return t;
            }
            // Stale: already popped, and never pushed back -- gone for good,
            // exactly as a real pop through `apply_due_commits` would drop
            // it.
        }
    }

    pub fn driver(&self, id: DriverId) -> &Driver {
        &self.drivers[id.0 as usize]
    }

    /// Registers a new driver as belonging to `partition`, returning its id.
    /// Topology assignment (which nodes, which partition) is the caller's
    /// job; the kernel only needs the partition id to mark dirty when this
    /// driver later commits a change. `partition` is `None` for a driver
    /// whose own node is fixed (`solve::PartitionSolver::partition_of_node`
    /// reports it so — W1c contract point 4): such a driver still commits
    /// and stamps into the solver's KCL, it simply never dirties a partition
    /// re-solve of its own.
    pub fn add_driver(&mut self, driver: Driver, partition: Option<PartitionId>) -> DriverId {
        let id = DriverId(self.drivers.len() as u32);
        self.drivers.push(driver);
        self.driver_partition.push(partition);
        id
    }

    /// Requests `level` for `driver` at `now`, exactly as a `DeviceEvaluator`
    /// would via `DriverRequest::Level`, but callable directly for a
    /// stimulus that does not originate from a reader's own evaluation — a
    /// harness toggle, or W1b's decision-1 parameter-update event. Goes
    /// through the same FSM and event-scheduling path `advance`'s delta
    /// cycle itself uses.
    pub fn request_driver(&mut self, driver: DriverId, level: Mode, now: f64, delay: f64) {
        self.apply_request(DriverRequest::Level { driver, level, delay }, now, None);
    }

    /// Schedules a partition threshold crossing at `candidate_t`, or rejects
    /// it under the acceptance rule (contract point 4). Returns whether it
    /// was accepted and scheduled.
    pub fn schedule_crossing(&mut self, partition: PartitionId, version: Version, candidate_t: f64) -> bool {
        if !accepts_crossing(candidate_t, self.time) {
            return false;
        }
        self.heap.push(candidate_t, EventKind::Crossing { partition, version });
        true
    }

    /// Applies one `DriverRequest`, pushing a `Commit` event onto the heap
    /// when the driver's FSM decides to schedule one. `delay_override`, when
    /// set, replaces whatever delay a `Level` request carries — used by
    /// [`Self::run_settle_pass`] to force zero delay throughout (decision 8).
    ///
    /// Returns the driver's partition when a `Reset` actually changed
    /// something (contract, "Kernel semantics: events", point 2: "a reset
    /// ... marks its partition dirty for the next re-solve; readers of that
    /// partition are then re-evaluated") — a `Level` request never returns
    /// one, because its effect (if any) is a `Commit` event that will mark
    /// the partition dirty itself, through the ordinary heap/`apply_due_commits`
    /// path, once it becomes due. A `Reset` has no due time of its own: unlike
    /// a commit, it takes effect the instant it is requested (`Driver::reset`),
    /// so this is the only signal a caller gets that it needs to fold the
    /// partition into whatever dirty set feeds the next re-solve.
    fn apply_request(&mut self, req: DriverRequest, now: f64, delay_override: Option<f64>) -> Option<PartitionId> {
        match req {
            DriverRequest::Level { driver, level, delay } => {
                let delay = delay_override.unwrap_or(delay);
                let idx = driver.0 as usize;
                if let Some((due, mode, token)) = self.drivers[idx].request(level, now, delay) {
                    self.heap.push(due, EventKind::Commit { driver, mode, token });
                }
                None
            }
            DriverRequest::Reset { driver } => {
                let idx = driver.0 as usize;
                // Only a driver that was actually committed to something
                // changes by being reset (finding 0: a never-committed
                // driver going through `reset()` is not an observable
                // change, so it must not force a re-solve of anything).
                let was_committed = self.drivers[idx].committed().is_some();
                self.drivers[idx].reset();
                if was_committed { self.driver_partition[idx] } else { None }
            }
        }
    }

    /// Pops and applies every event due at `self.time` (FIFO within that
    /// instant — the heap's own ordering already guarantees this):
    /// - a `Commit` folds its driver's partition into the returned `dirty`
    ///   set, for the caller to hand to `PartitionSolver::resolve_dirty`
    ///   (unchanged from W1a);
    /// - a `Crossing`/`Step` still current (`PartitionSolver::is_current`,
    ///   events.rs's lazy-deletion contract) is applied DIRECTLY through
    ///   `PartitionSolver::handle_dynamic_event` — not folded into `dirty`,
    ///   since the partition's own scheduled event becoming due already IS
    ///   the electrical update (pec3.mjs's `_analyticCross`/`_commitStep`),
    ///   with nothing left for a later `resolve_dirty` call to redo — and its
    ///   readers are returned directly in `directly_affected`;
    /// - a stale `Crossing`/`Step` (version no longer current) and a `Timer`
    ///   (no producer before W1c's floating buckets) are silently dropped,
    ///   exactly as a stale `Commit` already is via `apply_commit`'s own
    ///   token check.
    ///
    /// Also returns how many events were popped and how many commits
    /// actually changed a driver's level — raw counts for the *caller* to
    /// fold into `self.stats` (or not: `run_settle_pass` calls this too, and
    /// deliberately does not count settling toward the same stats — see
    /// `KernelStats`'s doc comment).
    fn apply_due_events(&mut self) -> Result<(BTreeSet<PartitionId>, BTreeSet<DeviceId>, u64, u64), KernelError> {
        let mut dirty = BTreeSet::new();
        let mut directly_affected = BTreeSet::new();
        let mut popped = 0u64;
        let mut changed = 0u64;
        while self.heap.peek_time() <= self.time {
            let ev = self.heap.pop().expect("peek_time <= time implies an event is present");
            popped += 1;
            match ev.kind {
                EventKind::Commit { driver, mode, token } => {
                    let idx = driver.0 as usize;
                    // A stale token (superseded/cancelled since this event
                    // was scheduled) is checked FIRST, before ever consulting
                    // power state: a commit that no longer means anything
                    // must drop silently exactly as it always has, never sit
                    // in `held_commits` forever waiting for a power state
                    // that was never going to make it live again.
                    if self.drivers[idx].token() != token {
                        // Stale: dropped, same as `Driver::apply_commit`
                        // would report (mirrored here, not delegated to it,
                        // because that call also clears `pending` on an
                        // Unchanged match -- a distinction that does not
                        // apply to a token mismatch either way).
                    } else if self.drivers[idx].device != usize::MAX && !self.solver.ic_powered(self.drivers[idx].device) {
                        // W1c contract point 9: held, not applied. The
                        // driver's committed level and in-flight transition
                        // stay exactly as they are; `Self::retry_held_commits`
                        // re-queues this once the IC is powered again.
                        self.held_commits.push((driver, mode, token));
                    } else if self.drivers[idx].apply_commit(mode, token) {
                        if let Some(p) = self.driver_partition[idx] {
                            dirty.insert(p);
                        }
                        changed += 1;
                    }
                }
                EventKind::Crossing { partition, version } => {
                    if self.solver.is_current(partition, true, version) {
                        let readers = self.solver.handle_dynamic_event(partition, self.time, true).map_err(KernelError::SolveFailed)?;
                        directly_affected.extend(readers);
                    }
                }
                EventKind::Step { partition, version } => {
                    if self.solver.is_current(partition, false, version) {
                        let readers = self.solver.handle_dynamic_event(partition, self.time, false).map_err(KernelError::SolveFailed)?;
                        directly_affected.extend(readers);
                    }
                }
                EventKind::Timer { .. } => {}
                EventKind::Bucket => {
                    let (dirty_partitions, readers) = self.solver.bucket_boundary(self.time);
                    dirty.extend(dirty_partitions);
                    directly_affected.extend(readers);
                }
            }
        }
        Ok((dirty, directly_affected, popped, changed))
    }

    /// Re-queues every held commit (W1c contract point 9) whose owning IC
    /// `solve::PartitionSolver::ic_powered` now reports powered again, at
    /// `self.time`, so it becomes due and is applied by the very next delta
    /// pass's [`Self::apply_due_events`] -- including its ordinary
    /// stale-token check, which is what lets a commit superseded WHILE held
    /// drop silently instead of resurrecting a stale level. Called after
    /// every re-solve, both from [`Self::run_deltas`] and
    /// [`Self::run_settle_pass`], since a hold can only end as a RESULT of a
    /// re-solve publishing a new supply voltage.
    fn retry_held_commits(&mut self) {
        if self.held_commits.is_empty() {
            return;
        }
        let mut still_held = Vec::new();
        for (driver, mode, token) in self.held_commits.drain(..) {
            let device = self.drivers[driver.0 as usize].device;
            if device != usize::MAX && self.solver.ic_powered(device) {
                self.heap.push(self.time, EventKind::Commit { driver, mode, token });
            } else {
                still_held.push((driver, mode, token));
            }
        }
        self.held_commits = still_held;
    }

    /// Pushes every `Crossing`/`Step` a dynamic partition asked to be
    /// scheduled since the last drain (`PartitionSolver::take_scheduled`)
    /// onto this kernel's own heap — the acceptance rule has already been
    /// applied by the solver itself before it ever produced one (that
    /// method's own doc comment), so every entry here is pushed verbatim.
    fn drain_scheduled(&mut self) {
        for (at, kind) in self.solver.take_scheduled() {
            self.heap.push(at, kind);
        }
    }

    /// Runs the two-phase delta cycle at the instant already set in
    /// `self.time`, to a quiet fixed point or [`KernelError::Livelock`] after
    /// [`DELTA_CAP`] passes (contract point 3, decision 4):
    /// 1. apply every due commit at `t`, in `(t, seq)` order, plus
    ///    `seed_dirty` on the very first pass only;
    /// 2. re-solve the partitions that changed, then evaluate each affected
    ///    reader exactly once;
    /// 3. a request with zero delay lands back at `t` and is picked up by
    ///    phase 1 of the next delta pass — "zero-delay outputs form the next
    ///    delta"; a `Reset` (finding 0) has no due time of its own to land
    ///    back at, so [`Self::apply_request`]'s returned partition is carried
    ///    forward the same way, into the *next* pass's dirty set, instead.
    ///
    /// `seed_dirty` exists for [`Self::settle`]'s hand-off from its one edge
    /// pass (decision 2, contract point 6 step 3): that pass's `Reset`
    /// requests are otherwise invisible to this function, since they neither
    /// sit on the heap (so `apply_due_commits` cannot find them) nor arrive
    /// through a delta pass of this function's own (they happened before this
    /// call even started). Every ordinary [`Self::advance`] call seeds with
    /// nothing.
    ///
    /// `count_stats` gates every `self.stats` update in here: `true` from
    /// [`Self::advance`], `false` from [`Self::settle`]'s hand-off, which is
    /// still load-time bookkeeping, not a live delta cycle (`KernelStats`'s
    /// doc comment: only `advance`'s real delta cycle counts towards these,
    /// the same line `run_settle_pass` already draws for its own passes).
    fn run_deltas(&mut self, mut seed_dirty: BTreeSet<PartitionId>, count_stats: bool) -> Result<(), KernelError> {
        let mut settled = false;
        let mut depth = 0u64;
        let mut carried_dirty = BTreeSet::new();
        for _delta in 0..DELTA_CAP {
            depth += 1;
            if count_stats {
                self.stats.delta_passes += 1;
            }

            // Phase 1: apply every due commit at this instant, plus whatever
            // a `Reset` in the previous pass (or, on the very first pass,
            // `settle`'s hand-off) marked dirty. A due `Crossing`/`Step` is
            // applied directly here too (`apply_due_events`'s doc comment),
            // contributing straight to `affected` below rather than to
            // `dirty` — its own partition's electrical update already
            // happened.
            let (mut dirty, mut affected, popped, changed) = self.apply_due_events()?;
            dirty.append(&mut seed_dirty);
            dirty.append(&mut carried_dirty);
            if count_stats {
                self.stats.events_processed += popped;
                self.stats.driver_commits += changed;
            }

            // Phase 2: re-solve every DRIVER-dirtied partition, folding its
            // affected readers into the same set `apply_due_events` already
            // seeded from any `Crossing`/`Step` this pass handled directly —
            // a set, not a plain concatenation, so a reader named more than
            // once (it watches nodes in more than one dirty partition, or
            // both a commit AND a crossing landed on it this same delta)
            // still runs only once.
            if !dirty.is_empty() {
                let dirty: Vec<PartitionId> = dirty.into_iter().collect();
                affected.extend(self.solver.resolve_dirty(&dirty, self.time, &self.drivers).map_err(KernelError::SolveFailed)?);
            }
            self.drain_scheduled();
            // W1c contract point 9: a hold can only end as a result of the
            // re-solve just above publishing a fresh supply voltage, so this
            // runs right after it, every pass, regardless of whether `dirty`
            // was empty this particular pass.
            self.retry_held_commits();

            // Phase 3: this instant is quiet once nothing was affected and
            // nothing else is due at it (a re-solve could in principle
            // schedule something at exactly `self.time` once a real solver
            // exists; this is checked, not assumed, the same way pec3.mjs's
            // `advance` does). A held commit just retried by the call above
            // lands back on the heap at exactly `self.time`, so this check
            // sees it too: "held commits ... apply in the next delta."
            if affected.is_empty() && self.heap.peek_time() > self.time {
                settled = true;
                break;
            }

            // Phase 4: evaluate each affected reader exactly once.
            for reader in affected {
                let requests = self.evaluator.evaluate(reader, self.time, true, self.solver.voltages());
                for req in requests {
                    if let Some(p) = self.apply_request(req, self.time, None) {
                        carried_dirty.insert(p);
                    }
                }
            }
        }

        if count_stats && depth > self.stats.max_delta_depth {
            self.stats.max_delta_depth = depth;
        }
        if !settled {
            return Err(KernelError::Livelock { at: self.time });
        }
        Ok(())
    }

    /// Processes every event with `t <= t_end` through [`Self::run_deltas`],
    /// then sets the clock to `t_end` (contract point 5).
    ///
    /// **A failed advance** (core/README.md): if a given instant's delta
    /// cycle never reaches a quiet fixed point, this instant is rolled back
    /// in full — the heap, every driver and the solver's published state are
    /// all put back exactly as they were before it began, and `self.time`
    /// stays at the previous, fully-settled instant — before the error is
    /// returned. Without this, a livelock's up-to-`DELTA_CAP` passes (each
    /// individually a legitimate, successful re-solve; that is exactly why a
    /// livelock is not itself a `SolveFailed`) would leave `self.time` and
    /// the solver's arrays at whatever the *last* of those passes happened
    /// to land on — an arbitrary point mid-oscillation, not "the state of
    /// the last instant it completed" the contract promises. The snapshot is
    /// taken once per instant, before its first delta pass, which is exactly
    /// the state as of the previous instant's completion (nothing runs
    /// between one instant finishing and the next one's `self.time = t`).
    pub fn advance(&mut self, t_end: f64) -> Result<(), KernelError> {
        loop {
            let t = self.heap.peek_time();
            if t > t_end {
                break;
            }

            let previous_time = self.time;
            let heap_snapshot = self.heap.clone();
            let drivers_snapshot = self.drivers.clone();
            let solver_snapshot = self.solver.snapshot();
            let dynamic_snapshot = self.solver.dynamic_snapshot();
            let held_commits_snapshot = self.held_commits.clone();
            let evaluator_snapshot = self.evaluator.snapshot();

            self.time = t;
            if let Err(e) = self.run_deltas(BTreeSet::new(), true) {
                self.time = previous_time;
                self.heap = heap_snapshot;
                self.drivers = drivers_snapshot;
                self.solver.restore(solver_snapshot);
                self.solver.dynamic_restore(dynamic_snapshot);
                self.held_commits = held_commits_snapshot;
                self.evaluator.restore(evaluator_snapshot);
                return Err(e);
            }
        }

        // The horizon clamp (contract point 5): bring every dynamic
        // partition's published state exactly to `t_end`, even though
        // nothing on the heap is due there — a closed-form partition's
        // trajectory is exact at any `t` but was only last PUBLISHED at its
        // latest re-aim/crossing time, and a stepped partition's committed
        // step generally lands short of `t_end` too
        // (`PartitionSolver::sync_dynamic_to`'s own doc comment). Wrapped in
        // the identical snapshot/rollback envelope as any other instant: a
        // stepped partition's forced step can fail to converge exactly as an
        // ordinary one can.
        let previous_time = self.time;
        let heap_snapshot = self.heap.clone();
        let drivers_snapshot = self.drivers.clone();
        let solver_snapshot = self.solver.snapshot();
        let dynamic_snapshot = self.solver.dynamic_snapshot();
        let held_commits_snapshot = self.held_commits.clone();
        let evaluator_snapshot = self.evaluator.snapshot();

        self.time = t_end;
        match self.solver.sync_dynamic_to(t_end) {
            Ok(_readers) => {
                // No reader concept is exercised by the horizon clamp today
                // (W1c+): a dynamic partition's own crossing/step handling
                // already ran through the ordinary delta machinery above for
                // everything up to `t_end`; the clamp only re-publishes a
                // state that was already correct, so nothing new needs
                // evaluating. Kept as a `Result` (not `()`) so a future
                // caller that DOES need to react to a horizon-clamp-only
                // change has somewhere to plug in.
                self.drain_scheduled();
                Ok(())
            }
            Err(e) => {
                self.time = previous_time;
                self.heap = heap_snapshot;
                self.drivers = drivers_snapshot;
                self.solver.restore(solver_snapshot);
                self.solver.dynamic_restore(dynamic_snapshot);
                self.held_commits = held_commits_snapshot;
                self.evaluator.restore(evaluator_snapshot);
                Err(KernelError::SolveFailed(e))
            }
        }
    }

    /// Applies a `bo_engine_set_params` batch as ONE event at the current
    /// time (owner decision 1, core/README.md's `bo_engine_set_params`):
    /// mutates the solver's device state
    /// ([`solve::PartitionSolver::apply_params`]), marks every partition it
    /// touched dirty, and runs one delta cycle right now — without moving
    /// the heap or the clock.
    ///
    /// The caller (`engine.rs`) must call
    /// `self.solver().validate_params(triples)` FIRST and map that failure
    /// to `-8`: this method only handles what happens once validation has
    /// already passed, because turning a solver-level validation `String`
    /// into the right `BoError` variant is `engine.rs`'s job, not this
    /// `Program`/`BoError`-free module's (the same line `advance`/`settle`
    /// already draw against `engine.rs`).
    ///
    /// Rolls back exactly like [`Self::advance`]'s "a failed advance" if the
    /// resulting delta cycle does not reach a quiet fixed point: a batch
    /// `validate_params` already accepted can still make the circuit
    /// unsolvable (closing a switch that shorts two rails, say), and the
    /// contract's "all or nothing" reads most naturally as covering that
    /// outcome too, not only the pre-application checks — so the rollback
    /// here undoes the param mutation itself (by re-applying the pre-image
    /// `validate_params`'s own success already guarantees exists,
    /// `PartitionSolver::param_value`), on top of the heap/drivers/solver
    /// snapshot `advance` already takes. This is a judgment call beyond
    /// what the contract states outright; the orchestrator can override it.
    pub fn apply_params_as_event(&mut self, triples: &[(usize, usize, f64)]) -> Result<(), KernelError> {
        let heap_snapshot = self.heap.clone();
        let drivers_snapshot = self.drivers.clone();
        let solver_snapshot = self.solver.snapshot();
        let dynamic_snapshot = self.solver.dynamic_snapshot();
        let held_commits_snapshot = self.held_commits.clone();
        let evaluator_snapshot = self.evaluator.snapshot();
        let old_values: Vec<(usize, usize, f64)> = triples
            .iter()
            .map(|&(d, p, _)| (d, p, self.solver.param_value(d, p).expect("validate_params must run and succeed before apply_params_as_event")))
            .collect();

        let dirty: BTreeSet<PartitionId> = self.solver.apply_params(triples).into_iter().collect();

        // A device every one of whose pins sits on an already-fixed node
        // names no partition at all (mirrors a resistor with both ends
        // fixed: no matrix solve is needed), but its published current can
        // still have just changed. `resolve_dirty` recomputes every
        // device's current unconditionally, even when handed an empty
        // slice, so call it directly here rather than going through
        // `run_deltas`, whose "nothing dirty, nothing to do" shortcut exists
        // for `advance` skipping a resolve when nothing is actually pending
        // — which does not apply here, since a live param really did just
        // change.
        let result = if dirty.is_empty() {
            self.solver.resolve_dirty(&[], self.time, &self.drivers).map(|_| ()).map_err(KernelError::SolveFailed)
        } else {
            self.run_deltas(dirty, true)
        };

        if let Err(e) = result {
            self.heap = heap_snapshot;
            self.drivers = drivers_snapshot;
            self.solver.restore(solver_snapshot);
            self.solver.dynamic_restore(dynamic_snapshot);
            self.held_commits = held_commits_snapshot;
            self.evaluator.restore(evaluator_snapshot);
            self.solver.apply_params(&old_values);
            return Err(e);
        }
        Ok(())
    }

    /// One settle pass: evaluate every reader in `all_readers`
    /// unconditionally (not just ones a solver flagged — nothing has run yet
    /// at the top of the very first pass, so there is nothing to flag by),
    /// apply whatever that produces (settling forces zero delay throughout,
    /// so every resulting `Commit` is due immediately), then re-solve.
    /// `resolve_all_once` is drained on its first `true`: the very first
    /// pass of a settle phase has no dirty partitions yet either, so it
    /// re-solves everything instead of nothing — the counterpart of
    /// classic's first seed solve and pec3's initial sweep over every
    /// partition. That first call happens even when `all_partitions` is
    /// empty (a circuit with no free node at all still needs its solver's
    /// one-time bookkeeping — for `StaticPartitionSolver`, publishing device
    /// currents computed purely from fixed voltages, which no partition
    /// solve would otherwise trigger). Returns whether anything actually
    /// changed, or the solver's error.
    ///
    /// Deliberately does not fold `apply_due_commits`'s counts into
    /// `self.stats`: see `KernelStats`'s doc comment.
    fn run_settle_pass(
        &mut self,
        all_partitions: &[PartitionId],
        all_readers: &[DeviceId],
        acting_on_edges: bool,
        resolve_all_once: &mut bool,
    ) -> Result<bool, KernelError> {
        // Finding 0: a `Reset` has no due time to be picked up by
        // `apply_due_commits` below, so it needs its own dirty set, folded in
        // alongside the ordinary commit-driven one -- both for what gets
        // re-solved this pass and for whether this pass counts as a change
        // at all (a settle pass whose only effect is a `Reset` must not be
        // mistaken for a fixed point).
        let mut reset_dirty = BTreeSet::new();
        for &reader in all_readers {
            let requests = self.evaluator.evaluate(reader, self.time, acting_on_edges, self.solver.voltages());
            for req in requests {
                if let Some(p) = self.apply_request(req, self.time, Some(0.0)) {
                    reset_dirty.insert(p);
                }
            }
        }
        let (mut dirty, directly_affected, _popped, commits_changed) = self.apply_due_events()?;
        let changed = commits_changed > 0 || !reset_dirty.is_empty() || !directly_affected.is_empty();
        dirty.append(&mut reset_dirty);
        let is_first_pass = *resolve_all_once;
        let to_resolve: Vec<PartitionId> = if is_first_pass {
            *resolve_all_once = false;
            all_partitions.to_vec()
        } else {
            dirty.into_iter().collect()
        };
        if is_first_pass || !to_resolve.is_empty() {
            // The returned reader list is intentionally unused here: this
            // pass already re-evaluates every reader unconditionally, the
            // same way pec3's `init()` calls `ic.eval()` over every IC each
            // settle pass. The affected-reader optimisation only pays off
            // once `advance`'s delta cycle has a converged baseline to
            // compare against. This first pass is also where every dynamic
            // partition (a `ClosedForm`/`Stepped` id, folded into
            // `all_partitions` exactly like a `Static` one) builds its FIRST
            // segment/plan (the `segReady` guard; `Partition::new`'s pending
            // state) and schedules its first `Crossing`/`Step` —
            // `Self::drain_scheduled` below is what gets that onto the heap
            // at all; without it a dynamic partition would sit forever with
            // a pending trial nothing ever pops.
            self.solver
                .resolve_dirty(&to_resolve, self.time, &self.drivers)
                .map_err(KernelError::SolveFailed)?;
        }
        self.drain_scheduled();
        // W1c contract point 9, mirroring `run_deltas`'s own call: a hold
        // can only end as a result of the resolve just above.
        self.retry_held_commits();
        Ok(changed)
    }

    /// Runs [`Self::run_settle_pass`] to a fixed point (no pass changes
    /// anything), or [`KernelError::Livelock`] after [`DELTA_CAP`] passes.
    fn run_to_fixed_point(
        &mut self,
        all_partitions: &[PartitionId],
        all_readers: &[DeviceId],
        acting_on_edges: bool,
        mut resolve_all_once: bool,
    ) -> Result<(), KernelError> {
        for _pass in 0..DELTA_CAP {
            if !self.run_settle_pass(all_partitions, all_readers, acting_on_edges, &mut resolve_all_once)? {
                return Ok(());
            }
        }
        Err(KernelError::Livelock { at: self.time })
    }

    /// Settles the network at `START_TIME` = `now` (decision 8, the W1-0
    /// spike's `pec3-w1.mjs` `init()`, and finding 1's fix to it), for
    /// `bo_engine_load`. `all_partitions` and `all_readers` are the complete
    /// lists for the loaded program — settling has no "dirty" set to start
    /// from, unlike an ordinary delta, so its first pass must consider
    /// everything.
    ///
    /// Exactly the contract's three steps (point 6), no more:
    /// 1. **Frozen fixed point, no delays.** Every `evaluate` call gets
    ///    `acting_on_edges = false`: devices drive their current state but
    ///    must not act on an edge or advance their own edge-history
    ///    bookkeeping. [`Self::run_to_fixed_point`] forces every resulting
    ///    request to zero delay regardless of what it asks for (decision 8)
    ///    and iterates until nothing changes — finding the consistent initial
    ///    state, not simulating time.
    /// 2. **Exactly one edge pass.** One call to `evaluate` per reader in
    ///    `all_readers`, `acting_on_edges = true` — never looped, never
    ///    forced to zero delay. A clock already HIGH at this point gives
    ///    classic's single "phantom" rising edge (Q-O3), because every
    ///    device's edge-history slot is still at its power-up default,
    ///    untouched throughout step 1.
    /// 3. **Ordinary simulation from here, never a further settle pass.**
    ///    Whatever step 2 requested — each with its *real* delay, per the
    ///    contract, which is why step 2 does not go through
    ///    [`Self::run_settle_pass`] at all — is hardware exactly like a
    ///    driver commit born from an `advance` delta: it is scheduled onto
    ///    the heap on its own real due time (immediately, only for a
    ///    zero-delay reader) and drained by the ordinary delta cycle
    ///    ([`Self::run_deltas`], the same machinery `advance` itself uses),
    ///    seeded with whatever step 2's requests reset outright (finding 0:
    ///    a `Reset` has no heap event of its own to make `run_deltas` see it
    ///    otherwise). This is exactly what rules out the W1-0 spike's
    ///    RAM-at-load bug: re-running the edge-*acting* pass at zero delay
    ///    (the previous implementation's bug — [`Self::run_to_fixed_point`]
    ///    with `acting_on_edges = true`, looped) let a reconvergent static
    ///    hazard (e.g. `/WE = NAND(A, NOT A)`) glitch through several
    ///    zero-delay passes and register as a spurious edge; a real,
    ///    nonzero delay lets the ordinary inertial FSM swallow that same
    ///    glitch, exactly as it does after load (decision 2).
    pub fn settle(&mut self, now: f64, all_partitions: &[PartitionId], all_readers: &[DeviceId]) -> Result<(), KernelError> {
        self.time = now;
        self.run_to_fixed_point(all_partitions, all_readers, false, true)?;

        let mut edge_pass_dirty = BTreeSet::new();
        for &reader in all_readers {
            let requests = self.evaluator.evaluate(reader, now, true, self.solver.voltages());
            for req in requests {
                if let Some(p) = self.apply_request(req, now, None) {
                    edge_pass_dirty.insert(p);
                }
            }
        }

        self.run_deltas(edge_pass_dirty, false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::Driver;
    use crate::events::Mode::{High, Low};

    // ---- mocks --------------------------------------------------------

    struct NoOpSolver;
    impl PartitionSolver for NoOpSolver {
        fn resolve_dirty(&mut self, _dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
            Ok(Vec::new())
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
        // W1b step 2: `PartitionSolver`'s default `is_current` is `false`
        // (nothing to confirm a `Crossing`/`Step` against for a solver with
        // no dynamic partitions at all), but the tests below push a
        // `Crossing` directly via `Kernel::schedule_crossing` to exercise
        // ITS OWN acceptance rule, independent of any real dynamic solver —
        // so this mock reports every version current, exactly matching this
        // module's pre-W1b-step-2 assumption that nothing here invalidates
        // one.
        fn is_current(&self, _partition: PartitionId, _is_crossing: bool, _version: Version) -> bool {
            true
        }
    }

    struct NoOpEvaluator;
    impl DeviceEvaluator for NoOpEvaluator {
        fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
            Vec::new()
        }
    }

    fn driver(node: usize) -> Driver {
        Driver::new(node, 100, 101, 1.0, 1.0, 0.0)
    }

    // ---- advance: basic wiring -----------------------------------------

    #[test]
    fn a_committed_change_marks_its_partition_dirty_and_the_solver_sees_it() {
        struct RecordingSolver {
            seen: Vec<Vec<PartitionId>>,
        }
        impl PartitionSolver for RecordingSolver {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                self.seen.push(dirty.to_vec());
                Ok(Vec::new())
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

        let mut k = Kernel::new(RecordingSolver { seen: Vec::new() }, NoOpEvaluator);
        let d0 = k.add_driver(driver(1), Some(PartitionId(7)));
        k.apply_request(DriverRequest::Level { driver: d0, level: High, delay: 5.0 }, 0.0, None);

        k.advance(5.0).unwrap();

        assert_eq!(k.solver.seen, vec![vec![PartitionId(7)]]);
        assert_eq!(k.driver(d0).committed(), Some(High));
        assert_eq!(k.time(), 5.0);
    }

    #[test]
    fn advance_leaves_time_untouched_beyond_t_end_when_nothing_is_pending() {
        let mut k = Kernel::new(NoOpSolver, NoOpEvaluator);
        k.advance(10.0).unwrap();
        assert_eq!(k.time(), 10.0);
        assert_eq!(k.next_event_time(), f64::INFINITY);
    }

    // ---- "a reader evaluated once per delta even when several of its
    // inputs change in the same delta" ----------------------------------

    #[test]
    fn a_reader_watching_two_dirty_partitions_is_evaluated_once_this_delta() {
        // The solver reports the same reader once per dirty partition it was
        // handed -- a solver that watches nodes across partitions could
        // plausibly do this. The kernel must still call `evaluate` on it
        // exactly once, deduplicating across the whole delta's result.
        struct DuplicatingSolver;
        impl PartitionSolver for DuplicatingSolver {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                Ok(dirty.iter().map(|_| DeviceId(0)).collect())
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
        struct CountingEvaluator {
            calls: Vec<(DeviceId, bool)>,
        }
        impl DeviceEvaluator for CountingEvaluator {
            fn evaluate(&mut self, reader: DeviceId, _now: f64, acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                self.calls.push((reader, acting_on_edges));
                Vec::new()
            }
        }

        let mut k = Kernel::new(DuplicatingSolver, CountingEvaluator { calls: Vec::new() });
        let d0 = k.add_driver(driver(1), Some(PartitionId(0)));
        let d1 = k.add_driver(driver(2), Some(PartitionId(1)));
        // Both commits become due at the same instant, so phase 1 of one
        // delta pass marks both partitions dirty in a single batch.
        k.apply_request(DriverRequest::Level { driver: d0, level: High, delay: 5.0 }, 0.0, None);
        k.apply_request(DriverRequest::Level { driver: d1, level: High, delay: 5.0 }, 0.0, None);

        k.advance(5.0).unwrap();

        assert_eq!(k.evaluator.calls, vec![(DeviceId(0), true)]);
    }

    // ---- "a zero-delay chain across 3 deltas" --------------------------

    #[test]
    fn a_zero_delay_combinational_chain_propagates_across_three_deltas() {
        // partition N dirty -> reader N affected, one to one.
        struct ChainSolver;
        impl PartitionSolver for ChainSolver {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                Ok(dirty.iter().map(|p| DeviceId(p.0)).collect())
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
        // reader 0 and 1 immediately retarget the next driver in the chain
        // at zero delay; reader 2 is the end of the chain and requests
        // nothing further.
        struct ChainEvaluator {
            drivers: [DriverId; 3],
            calls: Vec<(DeviceId, f64, bool)>,
        }
        impl DeviceEvaluator for ChainEvaluator {
            fn evaluate(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                self.calls.push((reader, now, acting_on_edges));
                let next = reader.0 as usize + 1;
                if next < self.drivers.len() {
                    vec![DriverRequest::Level { driver: self.drivers[next], level: High, delay: 0.0 }]
                } else {
                    Vec::new()
                }
            }
        }

        let mut k = Kernel::new(ChainSolver, ChainEvaluator { drivers: [DriverId(0), DriverId(0), DriverId(0)], calls: Vec::new() });
        let d0 = k.add_driver(driver(1), Some(PartitionId(0)));
        let d1 = k.add_driver(driver(2), Some(PartitionId(1)));
        let d2 = k.add_driver(driver(3), Some(PartitionId(2)));
        k.evaluator.drivers = [d0, d1, d2];
        // Kick off the chain: driver 0 commits at t = 0.
        k.apply_request(DriverRequest::Level { driver: d0, level: High, delay: 0.0 }, 0.0, None);

        k.advance(0.0).unwrap();

        assert_eq!(
            k.evaluator.calls,
            vec![(DeviceId(0), 0.0, true), (DeviceId(1), 0.0, true), (DeviceId(2), 0.0, true)],
            "each link fires in its own delta pass, all at the same instant"
        );
        assert_eq!(k.driver(d2).committed(), Some(High), "the level must have reached the far end of the chain");
    }

    // ---- livelock detection ---------------------------------------------

    #[test]
    fn an_oscillating_zero_delay_pair_hits_the_cap_and_returns_livelock_not_a_silent_exit() {
        struct AlwaysAffects(DeviceId);
        impl PartitionSolver for AlwaysAffects {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                Ok(if dirty.is_empty() { Vec::new() } else { vec![self.0] })
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
                self.next = if level == High { Low } else { High };
                vec![DriverRequest::Level { driver: self.driver, level, delay: 0.0 }]
            }
        }

        // `add_driver` assigns ids sequentially from 0, so the first driver
        // added is known to be `DriverId(0)` before it exists -- needed here
        // because the mock evaluator must be constructed (naming the driver
        // it will oscillate) before the kernel that will assign that id.
        let d0 = DriverId(0);
        let mut k = Kernel::new(AlwaysAffects(DeviceId(0)), OscillatingEvaluator { driver: d0, next: High });
        assert_eq!(k.add_driver(driver(1), Some(PartitionId(0))), d0);
        k.apply_request(DriverRequest::Level { driver: d0, level: Low, delay: 0.0 }, 0.0, None);

        let err = k.advance(0.0).unwrap_err();
        assert_eq!(err, KernelError::Livelock { at: 0.0 });
        // The clock must stay at the stuck instant, never silently advance
        // as if the pass had converged.
        assert_eq!(k.time(), 0.0);
    }

    // ---- finding 0: DriverRequest::Reset dirties its partition -----------

    #[test]
    fn reset_of_a_committed_driver_marks_its_partition_dirty_for_the_next_delta_and_re_evaluates_its_readers() {
        // A `Reset` used to drop the driver's stamp (`Driver::reset`)
        // without marking anything dirty or scheduling anything: the
        // driver's partition was never re-solved and its readers never
        // re-evaluated, so a published node voltage kept the stale stamp
        // until an unrelated commit happened to touch the same partition.
        struct RecordingSolver {
            seen: Vec<Vec<PartitionId>>,
        }
        impl PartitionSolver for RecordingSolver {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                self.seen.push(dirty.to_vec());
                // Reader 0 is the "enable" logic (watches partition 0);
                // reader 1 is the bus's own reader (watches partition 1) --
                // exactly the mapping a real bus/enable topology would give.
                let mut affected = Vec::new();
                if dirty.contains(&PartitionId(0)) {
                    affected.push(DeviceId(0));
                }
                if dirty.contains(&PartitionId(1)) {
                    affected.push(DeviceId(1));
                }
                Ok(affected)
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

        // Reader 0 resets the bus the first time it runs (an enable logic
        // that tri-states the bus once), and does nothing after. Reader 1
        // just counts how many times the bus's own reader is evaluated.
        struct Evaluator {
            bus: DriverId,
            reset_sent: bool,
            reader1_calls: u32,
        }
        impl DeviceEvaluator for Evaluator {
            fn evaluate(&mut self, reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                match reader.0 {
                    0 => {
                        if self.reset_sent {
                            Vec::new()
                        } else {
                            self.reset_sent = true;
                            vec![DriverRequest::Reset { driver: self.bus }]
                        }
                    }
                    1 => {
                        self.reader1_calls += 1;
                        Vec::new()
                    }
                    other => unreachable!("no reader {other} in this test"),
                }
            }
        }

        let mut k = Kernel::new(RecordingSolver { seen: Vec::new() }, Evaluator { bus: DriverId(0), reset_sent: false, reader1_calls: 0 });
        let bus = k.add_driver(driver(1), Some(PartitionId(1)));
        let enable = k.add_driver(driver(2), Some(PartitionId(0)));
        k.evaluator.bus = bus;

        // The bus is committed HIGH well before the reset -- and its own
        // reader is naturally evaluated once from that first commit.
        k.request_driver(bus, High, 0.0, 0.0);
        k.advance(0.0).unwrap();
        assert_eq!(k.driver(bus).committed(), Some(High));
        let reader1_calls_from_the_initial_commit = k.evaluator.reader1_calls;

        // At t = 1.0, an unrelated commit on `enable` dirties partition 0,
        // whose reader (device 0) issues the bus's Reset.
        k.request_driver(enable, High, 1.0, 0.0);
        k.advance(1.0).unwrap();

        assert_eq!(k.driver(bus).committed(), None, "the reset must actually take effect");
        // Partition 1 (the bus's) must be re-solved in the delta *after* the
        // one where the reset was requested -- a Reset carries no due time
        // of its own to land back at the same delta the way a zero-delay
        // commit would.
        assert_eq!(
            &k.solver.seen[1..],
            &[vec![PartitionId(0)], vec![PartitionId(1)]],
            "partition 0 (enable) resolves first, requesting the reset; partition 1 (the bus) must be re-solved next: {:?}",
            k.solver.seen
        );
        assert_eq!(
            k.evaluator.reader1_calls,
            reader1_calls_from_the_initial_commit + 1,
            "the bus's reader must be re-evaluated exactly once after the reset"
        );
    }

    #[test]
    fn settle_counts_a_reset_as_a_change_not_a_fixed_point() {
        // Finding 0's settle half: `run_settle_pass` used to count only
        // `apply_due_commits`'s changed-commit count, so a pass whose only
        // effect was a `Reset` reported `changed = false` -- the fixed point
        // ended right there, and the reset's partition was never re-solved
        // at all (not even once), leaving the stale, pre-reset stamp behind.
        struct RecordingSolver {
            seen: Vec<Vec<PartitionId>>,
        }
        impl PartitionSolver for RecordingSolver {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                self.seen.push(dirty.to_vec());
                Ok(Vec::new())
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
        // Commits the driver HIGH on the first call, then resets it on the
        // second, then goes quiet.
        struct CommitThenReset {
            driver: DriverId,
            calls: u32,
        }
        impl DeviceEvaluator for CommitThenReset {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                self.calls += 1;
                match self.calls {
                    1 => vec![DriverRequest::Level { driver: self.driver, level: High, delay: 0.0 }],
                    2 => vec![DriverRequest::Reset { driver: self.driver }],
                    _ => Vec::new(),
                }
            }
        }

        let d0 = DriverId(0);
        let mut k = Kernel::new(RecordingSolver { seen: Vec::new() }, CommitThenReset { driver: d0, calls: 0 });
        assert_eq!(k.add_driver(driver(1), Some(PartitionId(3))), d0);

        k.settle(0.0, &[PartitionId(3)], &[DeviceId(0)]).unwrap();

        assert_eq!(k.driver(d0).committed(), None, "the reset must have actually applied");
        // Partition 3 must be re-solved once for the initial commit and
        // again for the reset -- if the reset pass were (wrongly) treated as
        // a fixed point, the second resolve would never happen at all.
        let partition3_resolves = k.solver.seen.iter().filter(|p| **p == vec![PartitionId(3)]).count();
        assert!(
            partition3_resolves >= 2,
            "the reset's partition must be re-solved, not skipped as a false fixed point: {:?}",
            k.solver.seen
        );
    }

    // ---- decision 8: settle -----------------------------------------------

    #[test]
    fn settle_freezes_edges_then_runs_exactly_one_edge_pass() {
        // A clock input that is already HIGH at load (classic's default
        // outHigh, Q-O3): the register must not count it as an edge while
        // frozen, and must count it exactly once after.
        struct EdgeCountingRegister {
            last_clk: bool,
            edges: u32,
        }
        impl DeviceEvaluator for EdgeCountingRegister {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                let clk_high = true;
                if acting_on_edges {
                    if !self.last_clk && clk_high {
                        self.edges += 1;
                    }
                    self.last_clk = clk_high;
                }
                Vec::new()
            }
        }

        let mut k = Kernel::new(NoOpSolver, EdgeCountingRegister { last_clk: false, edges: 0 });
        k.settle(0.0, &[], &[DeviceId(0)]).unwrap();

        assert_eq!(k.evaluator.edges, 1, "exactly one edge pass, never one per settle iteration");
    }

    #[test]
    fn settle_resolves_every_partition_on_its_first_pass_with_nothing_yet_marked_dirty() {
        struct RecordingSolver {
            seen: Vec<Vec<PartitionId>>,
        }
        impl PartitionSolver for RecordingSolver {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                self.seen.push(dirty.to_vec());
                Ok(Vec::new())
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

        let mut k = Kernel::new(RecordingSolver { seen: Vec::new() }, NoOpEvaluator);
        k.settle(0.0, &[PartitionId(0), PartitionId(1)], &[]).unwrap();

        assert_eq!(k.solver.seen.first(), Some(&vec![PartitionId(0), PartitionId(1)]), "the first settle pass has no dirty set to start from, so it must resolve everything");
    }

    #[test]
    fn an_oscillating_settle_hits_the_cap_and_returns_livelock() {
        // `settle`'s passes evaluate every reader in `all_readers`
        // unconditionally (there is no dirty set to filter by on the very
        // first pass, and it never switches to the affected-reader mode
        // `advance` uses), so the solver's own report plays no part here:
        // `NoOpSolver` is enough. The oscillation comes entirely from the
        // mock reader always retargeting its driver, every pass, forever.
        struct OscillatingEvaluator {
            driver: DriverId,
            next: Mode,
        }
        impl DeviceEvaluator for OscillatingEvaluator {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                let level = self.next;
                self.next = if level == High { Low } else { High };
                // The requested delay does not matter: settle forces zero
                // delay throughout regardless of what is asked for.
                vec![DriverRequest::Level { driver: self.driver, level, delay: 123.0 }]
            }
        }

        let d0 = DriverId(0);
        let mut k = Kernel::new(NoOpSolver, OscillatingEvaluator { driver: d0, next: High });
        assert_eq!(k.add_driver(driver(1), Some(PartitionId(0))), d0);

        let err = k.settle(0.0, &[], &[DeviceId(0)]).unwrap_err();
        assert_eq!(err, KernelError::Livelock { at: 0.0 });
    }

    // ---- finding 1: settle's load-time hazard must not survive real delays

    #[test]
    fn settle_never_lets_a_reconvergent_hazard_after_the_phantom_edge_write_ram_at_load() {
        // Finding 1: the previous `settle` ran its edge-acting pass to a
        // fixed point at forced zero delay (the old decision-8 code's second
        // `run_to_fixed_point` call), so a static hazard through
        // reconvergent logic fed by the phantom edge -- `/WE = NAND(Q, NOT
        // Q)`, spec 2.4's classic phantom-edge scenario -- got to glitch
        // through several zero-delay passes and register as a fresh rising
        // edge on a later pass: the W1-0 spike's RAM-at-load bug. The fix
        // (contract point 6) is exactly one edge pass, then real delays
        // drained by the ordinary delta cycle, which lets the inertial FSM
        // swallow the same hazard exactly as it does after load (decision 2).
        //
        // Topology (one driver/partition each, real, nonzero delays):
        //   Q  (partition 0): a register's output, phantom-edge driven.
        //   N  (partition 1): an inverter, N = !Q.
        //   WE (partition 2): a NAND, /WE = !(Q && N) -- ideally always HIGH,
        //     since Q and N always disagree, EXCEPT for the moment right
        //     after Q changes and before N has caught up.
        // Readers: 0 = register (drives Q), 1 = inverter (drives N),
        // 2 = NAND (drives WE), 3 = RAM (drives nothing; watches WE, writes
        // on a LOW->HIGH transition, `last_we` seeded HIGH so the steady
        // power-up state is not itself mistaken for a write).
        use std::cell::RefCell;
        use std::rc::Rc;

        const REG: u32 = 0;
        const INV: u32 = 1;
        const NAND: u32 = 2;
        const RAM: u32 = 3;

        struct Shadow {
            q: bool,
            n: bool,
            we: bool,
        }

        struct HazardSolver {
            shadow: Rc<RefCell<Shadow>>,
            q: DriverId,
            n: DriverId,
            we: DriverId,
        }
        impl PartitionSolver for HazardSolver {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                // A real solver publishes voltages; this mock's equivalent is
                // refreshing the shadow every reader reads its inputs from,
                // from the real, just-applied driver commits.
                {
                    let mut s = self.shadow.borrow_mut();
                    s.q = drivers[self.q.0 as usize].committed() == Some(High);
                    s.n = drivers[self.n.0 as usize].committed() == Some(High);
                    s.we = drivers[self.we.0 as usize].committed() == Some(High);
                }
                let mut affected = Vec::new();
                for &p in dirty {
                    match p.0 {
                        0 => {
                            affected.push(DeviceId(INV));
                            affected.push(DeviceId(NAND));
                        }
                        1 => affected.push(DeviceId(NAND)),
                        2 => affected.push(DeviceId(RAM)),
                        other => panic!("no partition {other} in this test"),
                    }
                }
                Ok(affected)
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

        struct HazardEvaluator {
            shadow: Rc<RefCell<Shadow>>,
            q: DriverId,
            n: DriverId,
            we: DriverId,
            reg_fired: bool,
            last_we: bool,
            writes: u32,
        }
        impl DeviceEvaluator for HazardEvaluator {
            fn evaluate(&mut self, reader: DeviceId, _now: f64, acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                match reader.0 {
                    REG => {
                        // CLK tied HIGH, `last_clk` defaults to 0 (classic's
                        // phantom rising edge, Q-O3): fires exactly once,
                        // only once edges go live. The delay it names does
                        // not actually matter for *this* commit -- it is
                        // this driver's first-ever request, which the FSM
                        // always schedules at `now` regardless of delay --
                        // but a real register's clock-to-Q figure belongs
                        // here regardless, for whatever fires after the
                        // first observation.
                        if acting_on_edges && !self.reg_fired {
                            self.reg_fired = true;
                            vec![DriverRequest::Level { driver: self.q, level: High, delay: 10.0 }]
                        } else {
                            Vec::new()
                        }
                    }
                    INV => {
                        let q = self.shadow.borrow().q;
                        vec![DriverRequest::Level { driver: self.n, level: if q { Low } else { High }, delay: 2.0 }]
                    }
                    NAND => {
                        let (q, n) = {
                            let s = self.shadow.borrow();
                            (s.q, s.n)
                        };
                        vec![DriverRequest::Level { driver: self.we, level: if q && n { Low } else { High }, delay: 5.0 }]
                    }
                    RAM => {
                        let we = self.shadow.borrow().we;
                        if !self.last_we && we {
                            self.writes += 1;
                        }
                        self.last_we = we;
                        Vec::new()
                    }
                    other => unreachable!("no reader {other} in this test"),
                }
            }
        }

        // Seeded to the values Q/N/WE will actually settle to (N = !Q with
        // Q = 0; WE = !(Q && N) = 1): a reader consulting the shadow before
        // any real commit lands sees exactly what that commit will produce,
        // so there is no artificial bounce from this mock's own bookkeeping
        // standing in for "undriven."
        let shadow = Rc::new(RefCell::new(Shadow { q: false, n: true, we: true }));

        let mut k = Kernel::new(
            HazardSolver { shadow: Rc::clone(&shadow), q: DriverId(0), n: DriverId(1), we: DriverId(2) },
            HazardEvaluator { shadow: Rc::clone(&shadow), q: DriverId(0), n: DriverId(1), we: DriverId(2), reg_fired: false, last_we: true, writes: 0 },
        );
        let q = k.add_driver(driver(1), Some(PartitionId(0)));
        let n = k.add_driver(driver(2), Some(PartitionId(1)));
        let we = k.add_driver(driver(3), Some(PartitionId(2)));
        assert_eq!((q, n, we), (DriverId(0), DriverId(1), DriverId(2)));

        k.settle(0.0, &[PartitionId(0), PartitionId(1), PartitionId(2)], &[DeviceId(REG), DeviceId(INV), DeviceId(NAND), DeviceId(RAM)]).unwrap();

        // The settle instant itself only drains what is due *at* it (real,
        // nonzero delays keep N's and WE's corrective commits pending); the
        // cascade plays out through ordinary simulation from here, exactly
        // as the contract's step 3 says.
        k.advance(100.0).unwrap();

        assert_eq!(k.evaluator.writes, 0, "a settling hazard must never look like a real write: RAM was written at load");
        assert_eq!(k.driver(we).committed(), Some(High), "/WE must settle back to its steady HIGH, the glitch never actually committed");
    }

    // ---- crossing acceptance --------------------------------------------

    #[test]
    fn accepts_crossing_requires_strictly_more_than_the_epsilon() {
        assert!(!accepts_crossing(10.0, 10.0));
        assert!(!accepts_crossing(10.0 + CROSSING_ACCEPT_EPS, 10.0), "exactly now + eps is still rejected: the rule is strictly greater than");
        // 1e-15 is below a single f64 ULP at magnitude 10.0, so a margin
        // that must be unambiguously *past* it needs a coarser step than
        // doubling CROSSING_ACCEPT_EPS itself would give.
        assert!(accepts_crossing(10.0 + 1e-12, 10.0));
    }

    #[test]
    fn schedule_crossing_rejects_at_or_before_the_epsilon_and_does_not_touch_the_heap() {
        let mut k = Kernel::new(NoOpSolver, NoOpEvaluator);
        k.time = 10.0;

        assert!(!k.schedule_crossing(PartitionId(0), 1, 10.0));
        assert_eq!(k.next_event_time(), f64::INFINITY, "a rejected crossing must not be scheduled at all");

        assert!(k.schedule_crossing(PartitionId(0), 1, 10.0 + 1e-12));
        assert_eq!(k.next_event_time(), 10.0 + 1e-12);
    }

    // ---- finding 2: next_event_time skips stale (cancelled) commits ------

    #[test]
    fn next_event_time_skips_a_cancelled_commits_stale_heap_entry() {
        // A glitch shorter than the delay cancels the in-flight transition
        // by bumping the token only (drivers::fsm's lazy-deletion contract)
        // -- the `Commit` event already in the heap for the old token is
        // left exactly where it is. `next_event_time` must not report that
        // dead slot as the next pending event (core/README.md: "Time of the
        // earliest pending event, +Infinity when none").
        let mut k = Kernel::new(NoOpSolver, NoOpEvaluator);
        let d0 = k.add_driver(driver(1), Some(PartitionId(0)));

        // First observation: committed LOW at t = 0.
        k.request_driver(d0, Low, 0.0, 0.0);
        k.advance(0.0).unwrap();
        assert_eq!(k.driver(d0).committed(), Some(Low));

        // Request HIGH with a real delay: schedules a Commit 20ns out.
        k.request_driver(d0, High, 1e-9, 20e-9);
        assert_eq!(k.next_event_time(), 1e-9 + 20e-9);

        // Glitch back to LOW before it is due: cancels the pending
        // transition (token bump), leaving the stale Commit sitting in the
        // heap for good.
        k.request_driver(d0, Low, 2e-9, 20e-9);

        assert_eq!(
            k.next_event_time(),
            f64::INFINITY,
            "nothing is actually pending: the stale Commit must be skipped, not reported"
        );
        // The skip is a real pop, not a masked peek: calling it again must
        // not somehow "rediscover" the entry.
        assert_eq!(k.next_event_time(), f64::INFINITY);
    }

    #[test]
    fn next_event_time_peeks_past_a_stale_entry_without_losing_the_live_one_under_it() {
        // Two drivers: the first's commit is cancelled (stale, token
        // mismatch) and sorts earlier in the heap than the second's live
        // one. Skipping the stale entry must not lose track of the live
        // event still sitting behind it, and must not disturb its own FIFO
        // position among any other event at that same time.
        let mut k = Kernel::new(NoOpSolver, NoOpEvaluator);
        let stale = k.add_driver(driver(1), Some(PartitionId(0)));
        let live = k.add_driver(driver(2), Some(PartitionId(1)));

        k.request_driver(stale, Low, 0.0, 0.0);
        k.request_driver(live, Low, 0.0, 0.0);
        k.advance(0.0).unwrap();

        // `stale`'s pending HIGH (due at 5.0) gets cancelled by a glitch back
        // to LOW; `live`'s pending HIGH (due at 10.0) is left alone.
        k.request_driver(stale, High, 1.0, 4.0);
        k.request_driver(live, High, 1.0, 9.0);
        k.request_driver(stale, Low, 2.0, 4.0);

        assert_eq!(k.next_event_time(), 10.0, "the live event behind the stale one must still be found");

        k.advance(10.0).unwrap();
        assert_eq!(k.driver(live).committed(), Some(High));
        assert_eq!(k.driver(stale).committed(), Some(Low), "the cancelled transition must never have taken effect");
    }

    // ---- "a failed advance": rollback to the last completed instant ------

    #[test]
    fn a_livelocked_instant_rolls_back_entirely_to_the_previous_instants_state() {
        // core/README.md, "A failed advance": on `-7` (or `-4`) the engine
        // keeps the state of the last instant it *completed*, and reports
        // that instant's time -- never a later, doomed instant's partial (and,
        // being an oscillation, essentially arbitrary) work. This proves the
        // rollback covers everything a delta cycle can mutate: the solver's
        // published state, every driver's commit, the event heap, and the
        // reported time -- not merely the time value on its own.
        #[derive(Clone)]
        struct RecordingSolver {
            // A mock "solve": whatever driver 0 is committed to, so this has
            // real, observable state to snapshot and restore, the same way
            // `StaticPartitionSolver`'s `voltages` does.
            voltages: Vec<f64>,
        }
        impl PartitionSolver for RecordingSolver {
            fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
                if dirty.is_empty() {
                    return Ok(Vec::new());
                }
                self.voltages = vec![match drivers[0].committed() {
                    Some(High) => 1.0,
                    Some(Low) => 0.0,
                    _ => -1.0,
                }];
                Ok(vec![DeviceId(0)])
            }
            fn voltages(&self) -> &[f64] {
                &self.voltages
            }
            fn currents(&self) -> &[f64] {
                &[]
            }
            fn restore(&mut self, snapshot: (Vec<f64>, Vec<f64>)) {
                self.voltages = snapshot.0;
            }
        }
        struct OscillatingEvaluator {
            driver: DriverId,
            next: Mode,
        }
        impl DeviceEvaluator for OscillatingEvaluator {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                let level = self.next;
                self.next = if level == High { Low } else { High };
                vec![DriverRequest::Level { driver: self.driver, level, delay: 0.0 }]
            }
        }

        let d0 = DriverId(0);
        let mut k = Kernel::new(RecordingSolver { voltages: vec![-1.0] }, OscillatingEvaluator { driver: d0, next: High });
        assert_eq!(k.add_driver(driver(1), Some(PartitionId(0))), d0);

        // First instant settles normally: one commit to HIGH at t = 5.0.
        k.request_driver(d0, High, 5.0, 0.0);
        k.advance(5.0).unwrap();
        assert_eq!(k.time(), 5.0);
        assert_eq!(k.driver(d0).committed(), Some(High));
        assert_eq!(k.solver.voltages, vec![1.0]);

        // Fresh oscillator for the doomed second instant, so its behavior
        // does not depend on whatever phase the first instant's evaluator
        // happened to end on (it settled after one real toggle, above).
        k.set_evaluator(OscillatingEvaluator { driver: d0, next: High });

        // A later request retargets it, due at t = 10.0 -- once that commit
        // applies, the oscillating evaluator never lets the partition settle.
        k.request_driver(d0, Low, 10.0, 0.0);
        let err = k.advance(20.0).unwrap_err();
        assert_eq!(err, KernelError::Livelock { at: 10.0 });

        // Every piece of "the last completed instant"'s state must be back
        // exactly as it was after the t = 5.0 advance -- not the t = 10.0
        // attempt's partial (and arbitrary) work.
        assert_eq!(k.time(), 5.0, "time must roll back to the last instant that actually settled");
        assert_eq!(k.driver(d0).committed(), Some(High), "the doomed instant's commit must be undone");
        assert_eq!(k.solver.voltages, vec![1.0], "the solver's published state must roll back too");
        // The request that triggered the doomed instant predates it (it was
        // issued between the two `advance` calls, not by the doomed
        // instant's own work), so it must still be pending afterward -- the
        // rollback undoes the instant's own work, not history before it.
        assert_eq!(k.next_event_time(), 10.0, "the still-pending commit that was about to run must not be lost");
    }
}
