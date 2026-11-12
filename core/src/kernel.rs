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

use crate::drivers::{DelaySet, Driver};
use crate::events::{DeviceId, DriverId, Event, EventHeap, EventKind, Mode, PartitionId, Seq, Token, Version};
use crate::digital::power::PowerHold;
use crate::solve::PartitionSolver;

/// The per-instant delta-pass cap (contract point 3: "More than 1000 deltas
/// at one instant is an error"). Also used as [`Kernel::settle`]'s pass cap:
/// the contract does not name a separate figure for settling, and an
/// unconverging settle is the same failure mode (a network with a zero-delay
/// loop) as an unconverging delta cycle, so it is guarded the same way
/// rather than left uncapped or given an arbitrary smaller number.
pub const DELTA_CAP: u32 = 1000;

/// The no-progress guard (coordinator ruling R6 (a)): completed instants
/// each within this of the one before form a run, and inside a run each is
/// judged by what it did to the state ([`NO_PROGRESS_MAX`]). Time alone
/// cannot tell a livelock from fast dynamics: a 50 ns RC inrush off a 5 ohm
/// cell lands some 28,000 rail drift events 0.7 ps apart (NP2), while V11's
/// CC-exit stall (B1070) repeats a rail that does not move every 1.14 ps and
/// the power-gate ping-pong every 5 and 9 fs. Since the state decides, the
/// window only has to hold a livelock's instants: 1 ns is a decade under the
/// stepped solver's smallest step (`stepped::H_MIN`, 1e-8 s), and a clock's
/// edges inside it count as progress. The longest run of instants inside it
/// measured on a legitimate board was 82,763 (the inrush); the most
/// no-progress instants one run counted, 3 (NP2, 2026-10-16).
pub const NO_PROGRESS_DT: f64 = 1e-9;

/// Within a run of completed instants (see [`NO_PROGRESS_DT`]), an instant
/// made no progress if it moved no published node voltage, turned one back
/// (moved it against its own last move in the run), or ran only `Rail`
/// events and left the rail node where it was, a move being a change of more
/// than `RAIL_MOVE_EPS` ([`move_sign`]) (a `Rail` event exists to move
/// the rail: B1070's other nodes crept on by their own trajectories while
/// the rail sat still). An instant that applied a waveform source's edge
/// progressed whatever it turned, and each node's direction starts afresh
/// after it: the edges are the board's scheduled input, not the circuit
/// chasing itself, and a clock has no frequency cap, so one at 10 GHz runs
/// indefinitely. Once this many have made no progress, the next instant
/// inside the run declines with `-4` and the rail ping-pong's own message
/// ([`crate::rails::loop_unsettled`]), before it runs. A livelock repeats
/// states, and a node that returns to a value must turn back on the way, so
/// any cycle counts at least one instant a period (two between source edges),
/// and a crawl that moves nothing counts every instant. Instants that
/// progress neither count nor reset it, so a cycle cannot hide its turns
/// between them; only an instant outside the window ends the run. A counter
/// with one floor, the rounding of a published value ([`move_sign`]): past
/// it the sign of each change is exact, and it changes nothing in a run it
/// does not stop. At microseconds an instant, a livelock declines well
/// inside a second.
pub const NO_PROGRESS_MAX: u32 = 10_000;

/// Ruling R86 (d): the events one host step (one `advance` target, across
/// the yields of a budgeted advance) may take before it declines with `-7`,
/// a deterministic backstop that holds with or without a host clock. The
/// busiest legitimate advance of the cold fuzz (1-4200, every template)
/// takes 956k events (866 tpl 0); the six boards that froze took 30M to 82M.
pub const HOST_STEP_EVENT_CAP: u64 = 20_000_000;

/// Ruling R86 (c): a budgeted run that makes less than this much simulated
/// time per second of the host's wall time, over [`PROGRESS_WINDOW_MS`] of
/// wall time spent inside budgeted advances, declines with `-7`. The slow
/// legitimate boards of the cold fuzz bottom out at 0.003-0.03 s/s, for the
/// 1-2 s of their start-up inrush; the freezes that remained ran at 4e-4 to
/// 9e-4 s/s for 50-200 s (2707 tpl 0, 4199 tpl 5, 894 tpl 0).
pub const PROGRESS_FLOOR: f64 = 1e-2;
pub const PROGRESS_WINDOW_MS: f64 = 30_000.0;

/// How many instants a budgeted advance runs between reads of the host
/// clock (ruling R86, addition (c): one import call a check, not one an
/// event).
pub const YIELD_CHECK_EVERY: u32 = 64;

/// The most one read of the host clock adds to the busy time, beside four
/// budgets (R86V finding 3): a read-to-read gap longer than both is a pause
/// (a suspended laptop, a backgrounded tab), not work. The check stride keeps
/// a read within about one budget plus one instant, and 250 ms is above the
/// slowest legitimate instant the cold fuzz measured (the 2 s start-up of
/// 3055 tpl 5 runs about 45,000 events, some 50 us each), so no real work is
/// cut; a cut can only make the floor slower to fire, never fire it.
pub const BUSY_READ_CAP_MS: f64 = 250.0;

/// What a budgeted advance ([`Kernel::advance_budgeted`]) reached.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Advanced {
    /// `t_end`, its horizon clamp included.
    Reached,
    /// The budget ran out between two instants: the state is the last
    /// completed instant's, short of `t_end`, and an advance to the same
    /// `t_end` goes on with the same host step.
    Yielded,
}

/// A wall budget for one call (ruling R86): the host's clock in
/// milliseconds, read every `check_every` instants, and how long the call
/// may run. A clock reading that is not finite turns the budget off.
pub struct Budget<'a> {
    pub clock: &'a mut dyn FnMut() -> f64,
    pub budget_ms: f64,
    pub check_every: u32,
}

/// The host step in flight: its target, the event count it began at, and
/// its span in simulated seconds (the cap without a clock scales with it).
#[derive(Debug, Clone, Copy)]
struct HostStep {
    t_end: f64,
    events_at_start: u64,
    span: f64,
}

/// One budgeted call's clock: when it began and was last read, the
/// instants since, and whether one has run in the call.
struct Reads {
    start: f64,
    last: f64,
    instants: u32,
    ran: bool,
}

/// The direction a published node voltage moved from `before` to `after`
/// for the no-progress guard ([`NO_PROGRESS_MAX`]): 0 for a change of at
/// most `RAIL_MOVE_EPS` (1e-12 V), which is no move, as it is none of the
/// rail's (Rails point 3, P3:468). A node the circuit holds still is
/// published from terms at the board's scale, so its value turns by their
/// rounding (G2bv's seed 400: a CC ramp into a series capacitor and a
/// 0.26 ohm short, the short's node at 0.0833 V turning by 1e-15 to 4e-15 V
/// at most of its 0.08 ns `Rail` events while the rail rose 1e-5 V at
/// each, declined at 1.16 us). Counting those as turns read a node at rest
/// as a cycle. A livelock is still caught: a cycle within the floor moves
/// nothing, so it counts at every instant.
#[inline]
fn move_sign(before: f64, after: f64) -> i8 {
    let d = after - before;
    if d > crate::rails::RAIL_MOVE_EPS {
        1
    } else if d < -crate::rails::RAIL_MOVE_EPS {
        -1
    } else {
        0
    }
}

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
    /// zero-delay loop that never settles; or, sooner, an evaluator's latch
    /// flipped back within the instant ([`DeviceEvaluator::zero_delay_loop`]),
    /// the same loop's own evidence. `at` is the simulation time this
    /// happened at. Never returned silently — pec3.mjs's `delta < 1000` loop
    /// (`:766`) simply exits and carries on as if converged; this is the
    /// mistake the contract calls out by name and this type exists to make
    /// impossible to repeat. Maps to ABI `-7`.
    Livelock { at: f64 },
    /// A [`solve::PartitionSolver::resolve_dirty`] call failed (a singular or
    /// non-finite pivot, or a non-finite published value) — the kernel-level
    /// counterpart of `ERR_SOLVE_FAILED`. Maps to ABI `-4`.
    SolveFailed(String),
    /// Ruling R86: the host step ran past [`HOST_STEP_EVENT_CAP`] events, or
    /// a budgeted run made less than [`PROGRESS_FLOOR`] of simulated time per
    /// wall second over [`PROGRESS_WINDOW_MS`]. Maps to ABI `-7`, the
    /// livelock's code: the circuit chases itself without getting anywhere.
    NoProgress { at: f64, message: String },
}

/// The outcome of a failed [`Kernel::seed_memory`] call: either the
/// [`DeviceEvaluator::seed_memory`] hook itself declined (`engine.rs` maps
/// this to ABI `-8`, invalid argument — the wrong kind, a wrong word count,
/// or a bad value), or the delta cycle the seed then triggered failed the
/// ordinary way (mapped exactly like any other [`KernelError`]).
#[derive(Debug, Clone, PartialEq)]
pub enum SeedMemoryError {
    Rejected(String),
    Kernel(KernelError),
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
#[derive(Debug, Clone, PartialEq)]
pub enum DriverRequest {
    /// Request `driver` reach `level` through the inertial FSM
    /// (`drivers::fsm::request`), with the reader's own kind's full
    /// datasheet [`DelaySet`] — the kernel neither knows nor needs to know
    /// which pair it picks from; that is `Driver::request`'s job
    /// (`drivers::fsm::pick`). During [`Kernel::settle`]'s frozen fixed
    /// point, every set is ignored and treated as all-zero throughout
    /// (decision 8; pec3-w1's `_seeding` forces every commit to zero delay
    /// while the network settles at load, not only on a driver's first
    /// observation).
    Level { driver: DriverId, level: Mode, delays: DelaySet },
    /// The reader did not drive `driver` at all this evaluation (a
    /// tri-stated bus, a direction flip on a bidirectional part): drop its
    /// delay history outright (`drivers::fsm::reset`) instead of
    /// transitioning it through the ordinary delayed path, so the next
    /// request against it is again a first observation (spec 2.4's cleanup
    /// pass, "drops the triad for any pin missing from `immediate`"). Only a
    /// W1c combinational IC still requests this (the HARD RULE: its
    /// behaviour stays bit-identical); W1d point 4: "no IC requests a reset
    /// any more" past that.
    Reset { driver: DriverId },
    /// A device-owned deadline becoming due (W1d, "Kernel semantics: events
    /// (W1a)" point 2 and the memories section's write-completion point):
    /// pushes `EventKind::Timer { device, tag }` onto the heap at `at`. A
    /// popped `Timer` folds `device` into that delta's directly-affected
    /// readers (`Self::apply_due_events`), so the device that scheduled it
    /// is evaluated in the very delta its `Timer` pops — the EEPROM's
    /// pending-write completion is the one producer today.
    Timer { device: DeviceId, at: f64, tag: u64 },
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

    /// [`Self::evaluate`], appending its requests to `out` instead of
    /// returning a fresh `Vec`: the kernel calls this once per affected
    /// reader per delta pass with one reusable buffer, so an evaluator that
    /// can fill it directly saves an allocation per call. `drivers` is the
    /// kernel's driver table, current as of this call: an evaluator may leave
    /// out a `Level` request for a driver that ignores it
    /// ([`Driver::request_is_noop`]), which the kernel would drop unseen
    /// anyway. The default forwards to `evaluate`, so an implementation that
    /// only defines `evaluate` (every test mock) behaves exactly as before.
    ///
    /// **A condition on every batch.** That equivalence holds only because
    /// the table is read once, before the batch is applied, while the kernel
    /// judges each request against the table as the batch before it left it.
    /// The two agree when no driver is named by a `Level` and by any other
    /// `Level` or `Reset` of the same batch (a first request that changes the
    /// driver, or a reset that clears it, would change what the second is
    /// judged against). Every evaluator in this crate meets it (one `Level`
    /// per output-capable pin, each pin its own driver; a 555's OUT and DIS
    /// are two drivers), and the kernel asserts it in debug builds
    /// ([`Kernel::run_deltas`]).
    ///
    /// `holds` is the solver's power-hold table
    /// ([`crate::solve::PartitionSolver::power_holds`], W1c point 1a): an IC
    /// held unpowered reads and requests nothing whatever its supply reads.
    ///
    /// Returns [`Self::zero_delay_loop`] as it stands after the call: the
    /// kernel reads the latch evidence off its evaluations rather than with
    /// a call through the trait object per delta pass (F5q).
    fn evaluate_into(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64], holds: &[PowerHold], drivers: &[Driver], out: &mut Vec<DriverRequest>) -> bool {
        let _ = drivers;
        out.extend(self.evaluate_held(reader, now, acting_on_edges, voltages, holds));
        self.zero_delay_loop()
    }

    /// [`Self::evaluate`] under the solver's power-hold table, the form the
    /// kernel calls where it keeps every request (load settling). The
    /// default ignores the table, which every evaluator with no IC (every
    /// test mock) may.
    fn evaluate_held(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64], holds: &[PowerHold]) -> Vec<DriverRequest> {
        let _ = holds;
        self.evaluate(reader, now, acting_on_edges, voltages)
    }

    /// Opens the rollback window for one instant (core/README.md, "A failed
    /// advance"): the kernel calls it before the instant's first delta pass
    /// and then exactly one of [`Self::end_instant`] (the instant completed)
    /// or [`Self::rollback_instant`] (it failed), handing back what this
    /// returned. The default takes a whole [`Self::snapshot`], the behaviour
    /// this trait always had; an evaluator whose state is large next to what
    /// one instant touches overrides all three to journal only the state it
    /// actually writes, so a completed instant (the overwhelmingly common
    /// case) pays for what it changed and not for a copy of everything.
    fn begin_instant(&mut self) -> Box<dyn std::any::Any> {
        self.snapshot()
    }

    /// The instant opened by [`Self::begin_instant`] completed: nothing to
    /// undo any more. Default drops the snapshot.
    fn end_instant(&mut self, checkpoint: Box<dyn std::any::Any>) {
        let _ = checkpoint;
    }

    /// The instant opened by [`Self::begin_instant`] failed: put back
    /// exactly the state it had. Default restores the snapshot.
    fn rollback_instant(&mut self, checkpoint: Box<dyn std::any::Any>) {
        self.restore(checkpoint)
    }

    /// Whether an evaluation inside the open window found a latch with no
    /// consistent state (NE555 point 11): it flipped and then flipped back
    /// within the instant with no other device's driver changing between,
    /// so its own output moved its inputs across both of its thresholds.
    /// With the instant's solves deterministic the state it returned to is
    /// the one that flipped it, so the cycle never ends (a flip back after
    /// another device changed, a delta-cycle hazard, is no evidence), and the kernel declines the instant as the zero-delay loop it
    /// is (`-7`, W1c's rule) without waiting for [`DELTA_CAP`]. That holds
    /// even where the second state's solve differs from the first's within
    /// tolerance and lets the cycle quiet (exact mode's stepped CC rail),
    /// which would otherwise re-open the loop at each step after it. The
    /// default (no latch) never finds one.
    fn zero_delay_loop(&self) -> bool {
        false
    }

    /// A latch due at once (a slow device's, W1g point 14) changed the
    /// circuit inside the instant: a latch that flips back after it answers
    /// a new circuit, not its own output, so the evidence of
    /// [`Self::zero_delay_loop`] starts afresh. Default: nothing kept.
    fn new_latch_baseline(&mut self) {}

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

    /// Called once after every instant this kernel COMPLETES with a quiet
    /// fixed point — never for one that livelocks or fails to solve and
    /// rolls back (W1c contract point 3, "Digital slots": "Slots and trace
    /// roll back with a failed instant" falls straight out of this never
    /// being called for one). [`Kernel::advance`] calls it once per instant
    /// in its own event loop and once more after the horizon clamp,
    /// [`Kernel::settle`] once at the end of load settling, and
    /// [`Kernel::apply_params_as_event`] once after its delta cycle — every
    /// point core/README.md's point 3 names ("at EVERY completed instant,
    /// not just at the end of advance").
    ///
    /// `drivers` is the kernel's full driver table and `solver` is the live
    /// partition solver, given as a trait object so this generic module
    /// commits to nothing about which concrete solver is behind `S` — W1c's
    /// digital-slot readout needs exactly two things from them (a driver's
    /// committed level, and its owning IC's current power state via
    /// [`crate::solve::PartitionSolver::ic_powered`]), neither of which
    /// `Kernel` itself has any notion of "a digital slot" to interpret.
    /// Returns `(slot, value)` pairs for the caller (`engine.rs`) to publish
    /// at this instant's time — an implementation with no slot-shaped
    /// readout (every evaluator before W1c) returns nothing. Default empty.
    fn on_settled(&mut self, now: f64, drivers: &[Driver], solver: &dyn PartitionSolver) -> Vec<(usize, f64)> {
        let _ = (now, drivers, solver);
        Vec::new()
    }

    /// [`Self::on_settled`], appending to `out` (the kernel's reusable
    /// buffer, cleared before the call) instead of returning a fresh `Vec`:
    /// the kernel calls this once per completed instant, so an evaluator that
    /// can fill it directly saves an allocation per instant. The default
    /// forwards to `on_settled`, so an implementation that only defines that
    /// (every test mock) behaves exactly as before. `holds` is the solver's
    /// power-hold table, as for [`Self::evaluate_into`]: a held IC's slots
    /// read `NaN`.
    fn on_settled_into(&mut self, now: f64, drivers: &[Driver], solver: &dyn PartitionSolver, holds: &[PowerHold], out: &mut Vec<(usize, f64)>) {
        let _ = holds;
        out.extend(self.on_settled(now, drivers, solver));
    }

    /// The kernel changed `driver`'s COMMITTED level: a commit that moved it
    /// (`Driver::apply_commit` returned `true`) or a reset that dropped a
    /// committed one. Called at the moment it happens, inside or outside a
    /// rollback window, and never for a request, a stale commit or a reset of
    /// a never-committed driver, none of which changes what a driver reads
    /// as. An evaluator whose [`Self::on_settled`] readout depends on driver
    /// levels uses it to visit only the devices that moved instead of every
    /// one; a rolled-back change is reported too (the kernel does not undo the
    /// call), which costs the evaluator one extra look and nothing else.
    /// Default: ignore.
    fn driver_changed(&mut self, driver: DriverId) {
        let _ = driver;
    }

    /// IC evaluations so far (contract stat 12) — "stats keep failed-attempt
    /// counts" (core/README.md, "A failed advance"), so an implementation
    /// must count every [`Self::evaluate`] call here regardless of whether
    /// the instant it happened in is later rolled back, exactly like
    /// [`KernelStats`] itself is never included in `Kernel::advance`'s own
    /// snapshot/restore envelope. Default `0.0`: only W1c's IC evaluator
    /// counts anything.
    fn ic_evaluations(&self) -> f64 {
        0.0
    }

    /// `bo_engine_seed_memory` (W1d, owner decision 2): mutate this
    /// evaluator's own state for `device` (a device-record index) to hold
    /// `words`, or decline with a message naming why (the wrong kind, a
    /// wrong word count, a bad value). `Err` means NOTHING changed —
    /// [`Kernel::seed_memory`] never runs a delta cycle at all when this
    /// returns one, so a well-behaved implementation must validate `words`
    /// in full before mutating anything (core/README.md: "validated in full
    /// before anything changes"). Default: every device declines — only an
    /// evaluator that actually owns a memory (`devices::ics::IcEvaluator`)
    /// overrides this.
    fn seed_memory(&mut self, device: usize, words: &[f64]) -> Result<(), String> {
        let _ = (device, words);
        Err("not a memory".to_string())
    }

    /// `bo_engine_take_eeprom_updates` (W1d): collects and clears every
    /// dirty EEPROM image this evaluator owns, packed `deviceIndex, W, word
    /// x W` per dirty EEPROM (core/README.md's own wording), in device
    /// order. Default empty: only an evaluator that actually owns an EEPROM
    /// overrides this.
    fn take_eeprom_updates(&mut self) -> Vec<f64> {
        Vec::new()
    }

    /// A boxed deep copy, for W2a's snapshots (the dyn-clone pattern: a
    /// trait object cannot require `Clone` itself). Every evaluator
    /// `Engine::load` installs overrides this; the test mocks never
    /// snapshot, so the default panics rather than make each one clonable.
    fn clone_box(&self) -> Box<dyn DeviceEvaluator> {
        unimplemented!("this evaluator is never snapshotted")
    }

    /// The heap this evaluator owns, for `bo_engine_snapshot_bytes`'s walk
    /// ([`crate::snapshot::HeapBytes`]). Default none: the mocks never
    /// snapshot.
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let _ = w;
        0
    }

    /// Appends W2c's carry records (`crate::carry`) of the devices this
    /// evaluator owns, their output drivers read from `drivers`. Default
    /// none.
    fn carry_take(&self, drivers: &[Driver], out: &mut Vec<f64>) {
        let _ = (drivers, out);
    }

    /// The output drivers of `device`, in pin order (tests).
    #[cfg(test)]
    fn debug_output_drivers(&self, device: usize) -> Vec<DriverId> {
        let _ = device;
        Vec::new()
    }
}

impl Clone for Box<dyn DeviceEvaluator> {
    fn clone(&self) -> Self {
        (**self).clone_box()
    }
}

/// So `Kernel<_, Box<dyn DeviceEvaluator>>` satisfies the trait bound
/// directly — see `solve::PartitionSolver`'s matching impl for why
/// `engine.rs` needs this.
impl DeviceEvaluator for Box<dyn DeviceEvaluator> {
    fn evaluate(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64]) -> Vec<DriverRequest> {
        (**self).evaluate(reader, now, acting_on_edges, voltages)
    }

    fn evaluate_into(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64], holds: &[PowerHold], drivers: &[Driver], out: &mut Vec<DriverRequest>) -> bool {
        (**self).evaluate_into(reader, now, acting_on_edges, voltages, holds, drivers, out)
    }

    fn evaluate_held(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, voltages: &[f64], holds: &[PowerHold]) -> Vec<DriverRequest> {
        (**self).evaluate_held(reader, now, acting_on_edges, voltages, holds)
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

    fn zero_delay_loop(&self) -> bool {
        (**self).zero_delay_loop()
    }

    fn new_latch_baseline(&mut self) {
        (**self).new_latch_baseline()
    }

    fn snapshot(&self) -> Box<dyn std::any::Any> {
        (**self).snapshot()
    }

    fn restore(&mut self, snapshot: Box<dyn std::any::Any>) {
        (**self).restore(snapshot)
    }

    fn on_settled(&mut self, now: f64, drivers: &[Driver], solver: &dyn PartitionSolver) -> Vec<(usize, f64)> {
        (**self).on_settled(now, drivers, solver)
    }

    fn on_settled_into(&mut self, now: f64, drivers: &[Driver], solver: &dyn PartitionSolver, holds: &[PowerHold], out: &mut Vec<(usize, f64)>) {
        (**self).on_settled_into(now, drivers, solver, holds, out)
    }

    fn driver_changed(&mut self, driver: DriverId) {
        (**self).driver_changed(driver)
    }

    fn ic_evaluations(&self) -> f64 {
        (**self).ic_evaluations()
    }

    fn seed_memory(&mut self, device: usize, words: &[f64]) -> Result<(), String> {
        (**self).seed_memory(device, words)
    }

    fn take_eeprom_updates(&mut self) -> Vec<f64> {
        (**self).take_eeprom_updates()
    }

    fn clone_box(&self) -> Box<dyn DeviceEvaluator> {
        (**self).clone_box()
    }

    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        std::mem::size_of_val::<dyn DeviceEvaluator>(&**self) + (**self).heap_bytes(w)
    }

    fn carry_take(&self, drivers: &[Driver], out: &mut Vec<f64>) {
        (**self).carry_take(drivers, out)
    }

    #[cfg(test)]
    fn debug_output_drivers(&self, device: usize) -> Vec<DriverId> {
        (**self).debug_output_drivers(device)
    }
}

/// For a `Box<dyn DeviceEvaluator>` field (`WithSources`): the walk goes
/// through the trait.
impl crate::snapshot::HeapBytes for dyn DeviceEvaluator {
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        DeviceEvaluator::heap_bytes(self, w)
    }
}

/// Whether `requests` (one reader's batch) names every driver it names in a
/// `Level` exactly once: no other `Level` and no `Reset` for it. The
/// condition `DeviceEvaluator::evaluate_into` documents, under which dropping
/// an inert `Level` against the table as it stood before the batch equals the
/// kernel skipping it against the table as the batch before it left it.
fn one_request_per_level_driver(requests: &[DriverRequest]) -> bool {
    let named = |r: &DriverRequest| match *r {
        DriverRequest::Level { driver, .. } => Some((driver, true)),
        DriverRequest::Reset { driver } => Some((driver, false)),
        DriverRequest::Timer { .. } => None,
    };
    requests.iter().enumerate().all(|(i, r)| match named(r) {
        None => true,
        Some((driver, is_level)) => requests[..i].iter().filter_map(named).all(|(other, other_is_level)| other != driver || !(is_level || other_is_level)),
    })
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
    /// The solver's [`PartitionSolver::holds_flag`], taken at construction:
    /// `Some(false)` says the hold table is empty without a call.
    holds_flag: Option<std::rc::Rc<std::cell::Cell<bool>>>,
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
    /// `(t, slot, value)` triples [`DeviceEvaluator::on_settled`] has
    /// reported since the last [`Self::take_digital_writes`] drain — the
    /// kernel-side half of W1c contract point 10's "Digital slots ... at
    /// EVERY completed instant": `Engine` (which owns the actual slot array
    /// and the harness trace, neither of which this `Program`/device-free
    /// module knows the shape of) drains this after every top-level call and
    /// republishes each entry at ITS OWN recorded `t`, not at whatever time
    /// `Engine` happens to regain control — `advance`'s own event loop can
    /// complete several instants inside one call, and each one's slot writes
    /// belong to ITS instant for the trace to align on (the harness's own
    /// requirement, W1c contract point 3). Never part of any snapshot/restore
    /// envelope: nothing is ever pushed here for an instant that fails and
    /// rolls back, since [`Self::publish_settled`] only ever runs after one
    /// that already committed — so there is nothing to undo.
    pending_digital_writes: Vec<(f64, usize, f64)>,
    /// Whether a rollback window ([`Self::open_window`]) is open: only then
    /// do pops and driver writes leave the undo records below.
    windowed: bool,
    /// Every event popped since the window opened, so a rollback can put it
    /// back ([`EventHeap::rollback`]). Replaces cloning the whole heap before
    /// every instant.
    heap_popped: Vec<Event>,
    /// The pre-image of every driver written since the window opened, one per
    /// driver (`driver_journaled` marks the ones already saved): what a
    /// rollback writes back. Replaces cloning the whole driver table before
    /// every instant, when an instant writes a handful of them.
    driver_journal: Vec<(u32, Driver)>,
    driver_journaled: Vec<bool>,
    /// A committed driver on a fixed node was reset since the delta cycle
    /// last resolved: like a commit there (finding 7), it dirties no
    /// partition, yet the next pass must resolve so the source KCL and the
    /// solver's driver table take it. Set and taken within one instant; a
    /// rollback clears it.
    fixed_reset: bool,
    /// Scratch buffers the delta cycle reuses so it allocates nothing per
    /// pass once they have grown to size: the partitions to re-solve, the
    /// readers to evaluate (each kept sorted and duplicate-free, which is the
    /// order the `BTreeSet`s they replace iterated in), and one reader's
    /// requests.
    dirty_buf: crate::shared::Scratch<PartitionId>,
    affected_buf: crate::shared::Scratch<DeviceId>,
    requests_buf: crate::shared::Scratch<DriverRequest>,
    /// The slot writes one `DeviceEvaluator::on_settled_into` call fills,
    /// reused across instants.
    settled_buf: crate::shared::Scratch<(usize, f64)>,
    /// The no-progress guard ([`NO_PROGRESS_MAX`]): the time of the last
    /// completed instant `advance` popped off the heap, and how many
    /// completed instants of the present run (each within [`NO_PROGRESS_DT`]
    /// of the one before) made no progress. Kept across `advance` calls,
    /// since a host stepping event by event makes one instant per call.
    /// Written only once an instant has completed, so one that fails and
    /// rolls back neither counts nor resets it; the horizon clamp is not an
    /// instant off the heap and leaves it alone.
    last_instant: f64,
    stalled: u32,
    /// The published voltages as an instant inside the run began (taken
    /// only for those, so a run of spaced instants copies nothing), and
    /// each node's last move in the run: -1, 1, or 0 for none yet.
    run_before: Vec<f64>,
    run_moves: Vec<i8>,
    /// Whether the instant in progress handled a live `Rail` event, and any
    /// other event: a `Rail`-only instant progresses only if it moved the
    /// rail ([`NO_PROGRESS_MAX`]).
    instant_rail: bool,
    instant_other: bool,
    /// The instant ran a `Slow` event that only re-anchored a drive (R21).
    instant_redrive: bool,
    /// Whether the instant in progress applied a live waveform source edge:
    /// scheduled input, so progress whatever it turned back.
    instant_source: bool,
    /// Ruling R86: the host step in flight ([`HOST_STEP_EVENT_CAP`]), kept
    /// while a budgeted advance yields inside it; the event cap; the wall
    /// time spent inside budgeted advances, and where the progress window
    /// ([`PROGRESS_FLOOR`]) began, in that time and in simulated time.
    host_step: Option<HostStep>,
    pub(crate) event_cap: u64,
    busy_ms: f64,
    window: (f64, f64),
    /// The instants between clock reads, adapted to the time an instant
    /// takes ([`Self::clock_read`]), and how many have run since the last
    /// read, carried across calls (R86V finding 1).
    check_stride: u32,
    since_check: u32,
}

/// What [`Kernel::open_window`] captured for a rollback that the kernel does
/// not keep incrementally: the solver's snapshots and the held commits (the
/// solver's side is another owner's to make cheaper), the evaluator's
/// checkpoint token, and where the heap's sequence counter stood.
struct Window {
    heap_seq: Seq,
    held_commits: Vec<(DriverId, Mode, Token)>,
    /// The solver's own checkpoint (`PartitionSolver::begin_instant`).
    solver: Box<dyn std::any::Any>,
    evaluator: Box<dyn std::any::Any>,
}

/// A deep copy for W2a's snapshots. The one field `derive(Clone)` would get
/// wrong is `holds_flag`: the kernel and its solver share one cell, so the
/// copy takes its flag from the solver's copy (which has a cell of its own,
/// [`crate::solve::PartitionSolver::clone_box`]) instead of aliasing the
/// original's. Written out field by field so a new field cannot be left out.
impl<S: PartitionSolver + Clone, E: DeviceEvaluator + Clone> Clone for Kernel<S, E> {
    fn clone(&self) -> Self {
        let solver = self.solver.clone();
        let holds_flag = solver.holds_flag();
        Kernel {
            solver,
            heap: self.heap.clone(),
            drivers: self.drivers.clone(),
            driver_partition: self.driver_partition.clone(),
            time: self.time.clone(),
            holds_flag,
            evaluator: self.evaluator.clone(),
            stats: self.stats.clone(),
            held_commits: self.held_commits.clone(),
            pending_digital_writes: self.pending_digital_writes.clone(),
            windowed: self.windowed.clone(),
            heap_popped: self.heap_popped.clone(),
            driver_journal: self.driver_journal.clone(),
            driver_journaled: self.driver_journaled.clone(),
            fixed_reset: self.fixed_reset.clone(),
            dirty_buf: self.dirty_buf.clone(),
            affected_buf: self.affected_buf.clone(),
            requests_buf: self.requests_buf.clone(),
            settled_buf: self.settled_buf.clone(),
            last_instant: self.last_instant.clone(),
            stalled: self.stalled.clone(),
            run_before: self.run_before.clone(),
            run_moves: self.run_moves.clone(),
            instant_rail: self.instant_rail.clone(),
            instant_other: self.instant_other.clone(),
            instant_redrive: self.instant_redrive.clone(),
            instant_source: self.instant_source.clone(),
            host_step: self.host_step.clone(),
            event_cap: self.event_cap.clone(),
            busy_ms: self.busy_ms.clone(),
            window: self.window.clone(),
            check_stride: self.check_stride.clone(),
            since_check: self.since_check.clone(),
        }
    }
}

/// The walk for W2a's snapshot bytes: the solver and the evaluator through
/// their traits, the holds flag's cell (shared with the solver) once.
impl<S: PartitionSolver, E: DeviceEvaluator> crate::snapshot::HeapBytes for Kernel<S, E> {
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        PartitionSolver::heap_bytes(&self.solver, w)
            + DeviceEvaluator::heap_bytes(&self.evaluator, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.holds_flag, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.heap, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.drivers, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.driver_partition, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.time, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.stats, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.held_commits, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.pending_digital_writes, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.windowed, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.heap_popped, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.driver_journal, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.driver_journaled, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.fixed_reset, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.dirty_buf, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.affected_buf, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.requests_buf, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.settled_buf, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.last_instant, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.stalled, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.run_before, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.run_moves, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.instant_rail, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.instant_other, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.instant_redrive, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.instant_source, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.host_step, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.event_cap, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.busy_ms, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.window, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.check_stride, w)
            + crate::snapshot::HeapBytes::heap_bytes(&self.since_check, w)
    }
}

impl<S: PartitionSolver, E: DeviceEvaluator> Kernel<S, E> {
    pub fn new(solver: S, evaluator: E) -> Self {
        Kernel {
            holds_flag: solver.holds_flag(),
            heap: EventHeap::new(),
            drivers: Vec::new(),
            driver_partition: Vec::new(),
            time: 0.0,
            solver,
            evaluator,
            stats: KernelStats::default(),
            held_commits: Vec::new(),
            pending_digital_writes: Vec::new(),
            windowed: false,
            heap_popped: Vec::new(),
            driver_journal: Vec::new(),
            driver_journaled: Vec::new(),
            fixed_reset: false,
            dirty_buf: crate::shared::Scratch::default(),
            affected_buf: crate::shared::Scratch::default(),
            requests_buf: crate::shared::Scratch::default(),
            settled_buf: crate::shared::Scratch::default(),
            last_instant: f64::NEG_INFINITY,
            stalled: 0,
            run_before: Vec::new(),
            run_moves: Vec::new(),
            instant_rail: false,
            instant_other: false,
            instant_redrive: false,
            instant_source: false,
            host_step: None,
            event_cap: HOST_STEP_EVENT_CAP,
            busy_ms: 0.0,
            window: (0.0, 0.0),
            check_stride: 1,
            since_check: 0,
        }
    }

    /// Opens the rollback window for one instant (or one event, for
    /// [`Self::apply_params_as_event`]/[`Self::seed_memory`]): everything a
    /// failed one must put back is captured or, for the heap, the drivers
    /// and the evaluator, journaled from here on as it is written. The
    /// window ends with exactly one of [`Self::close_window`] (the instant
    /// completed) or [`Self::roll_back`].
    fn open_window(&mut self) -> Window {
        // A real assert: a nested window would overwrite the journals of the
        // one it sits inside, and a later rollback would silently restore the
        // wrong state.
        assert!(!self.windowed, "rollback windows do not nest");
        self.windowed = true;
        self.heap_popped.clear();
        self.driver_journal.clear();
        Window {
            heap_seq: self.heap.next_seq(),
            held_commits: self.held_commits.clone(),
            solver: self.solver.begin_instant(),
            evaluator: self.evaluator.begin_instant(),
        }
    }

    /// Ends a window whose instant completed: the journals are dropped, the
    /// evaluator's checkpoint released.
    fn close_window(&mut self, window: Window) {
        self.end_journals();
        self.solver.end_instant(window.solver);
        self.evaluator.end_instant(window.evaluator);
    }

    /// Ends a window whose instant failed, restoring what
    /// [`Self::open_window`] captured, in the order the whole-copy snapshots
    /// always restored in: heap, drivers, solver, held commits, evaluator.
    /// The caller restores `self.time` (and anything else of its own).
    fn roll_back(&mut self, window: Window) {
        self.heap.rollback(window.heap_seq, &self.heap_popped);
        for (idx, driver) in self.driver_journal.drain(..) {
            self.drivers[idx as usize] = driver;
            self.driver_journaled[idx as usize] = false;
        }
        self.fixed_reset = false;
        self.heap_popped.clear();
        self.windowed = false;
        self.solver.rollback_instant(window.solver);
        self.held_commits = window.held_commits;
        self.evaluator.rollback_instant(window.evaluator);
    }

    /// Ends a window that changed nothing the kernel journals (a rejected
    /// `seed_memory`, which stops before any delta pass): only the
    /// evaluator's own state can differ, so only it is rolled back.
    fn abandon_window(&mut self, window: Window) {
        self.end_journals();
        self.solver.end_instant(window.solver);
        self.evaluator.rollback_instant(window.evaluator);
    }

    fn end_journals(&mut self) {
        for (idx, _) in self.driver_journal.drain(..) {
            self.driver_journaled[idx as usize] = false;
        }
        self.heap_popped.clear();
        self.windowed = false;
    }

    /// Saves driver `idx`'s current state before its first write inside an
    /// open window. Called ahead of every mutation of `self.drivers[idx]`
    /// (`request`, `reset`, `apply_commit`); a no-op outside a window
    /// (`settle`'s own passes have no rollback).
    fn journal_driver(&mut self, idx: usize) {
        if self.windowed && !self.driver_journaled[idx] {
            self.driver_journaled[idx] = true;
            self.driver_journal.push((idx as u32, self.drivers[idx]));
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
    /// The holds flag the kernel reads, so a test can check which cell it is.
    #[cfg(test)]
    pub(crate) fn holds_flag_cell(&self) -> Option<&std::rc::Rc<std::cell::Cell<bool>>> {
        self.holds_flag.as_ref()
    }

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

    /// Read access to the device evaluator — `engine.rs`'s `Engine::stat`
    /// uses this to reach [`DeviceEvaluator::ic_evaluations`] (contract stat
    /// 12) without this generic module needing to know what an "IC" is,
    /// mirroring [`Self::solver`]'s identical reason for existing.
    pub fn evaluator(&self) -> &E {
        &self.evaluator
    }

    /// Mutable access to the device evaluator — `engine.rs`'s
    /// `Engine::take_eeprom_updates` uses this to reach
    /// [`DeviceEvaluator::take_eeprom_updates`] (a draining call, so it needs
    /// `&mut`) without this generic module needing to know what an EEPROM
    /// is, mirroring [`Self::solver_mut`]'s identical reason for existing.
    pub fn evaluator_mut(&mut self) -> &mut E {
        &mut self.evaluator
    }

    /// Drains every `(t, slot, value)` write [`DeviceEvaluator::on_settled`]
    /// has reported since the last call — `engine.rs` calls this after every
    /// `advance`/`settle`/`apply_params_as_event` (successful or not: a
    /// failed `advance` can still have completed earlier instants within the
    /// same call, and those instants' writes are real, not rolled back — see
    /// [`Self::pending_digital_writes`]'s own doc comment) and republishes
    /// each entry at its OWN recorded time into whatever slot storage it
    /// owns.
    pub fn take_digital_writes(&mut self) -> Vec<(f64, usize, f64)> {
        std::mem::take(&mut self.pending_digital_writes)
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
            if self.event_live(ev.kind) {
                let t = ev.time;
                self.heap.push_event(ev);
                return t;
            }
            // Stale: already popped, and never pushed back -- gone for good,
            // exactly as a real pop through `apply_due_commits` would drop
            // it.
        }
    }

    /// Whether a queued event still means anything: the check each kind's
    /// handler in [`Self::apply_due_events`] makes before acting, so a stale
    /// one is exactly an event that handler would drop.
    #[inline]
    fn event_live(&self, kind: EventKind) -> bool {
        match kind {
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
            // W1f: the rail's and a waveform source's own versions.
            EventKind::Rail { version } => self.solver.rail_current(version),
            EventKind::Source { device, version } => self.solver.source_current(device, version),
            // W1g: a slow device's token (README W1g point 14).
            EventKind::Slow { device, token } => self.solver.slow_is_current(device, token),
        }
    }

    /// The time of the earliest pending driver `Commit` with a still-live
    /// token, `+Infinity` when none -- `bo_engine_next_digital_due_time`'s
    /// contract (finding 10/8): classic's `nextScheduledEventTime()`
    /// counterpart, for a host that wants to clamp its own steps onto
    /// digital due times without single-stepping through a `Crossing`/
    /// `Step`/`Bucket` event a dynamic partition schedules purely for its own
    /// internal bookkeeping ([`Self::next_event_time`] reports ALL of those
    /// too, which is correct for that method's own contract but wrong for a
    /// host that only means to ask "when does a digital output next
    /// change"). A `Commit` currently sitting in [`Self::held_commits`] (the
    /// W1c contract point 9 power-loss hold) does not count: it already left
    /// the heap the instant it became due while unpowered, so it has no
    /// pending time of its own to report until [`Self::retry_held_commits`]
    /// re-queues it, at which point it is due immediately (at `self.time`)
    /// and this method reports that, same as any other due commit. `&self`,
    /// not `&mut`: unlike [`Self::next_event_time`], this never needs to
    /// permanently discard a stale entry to answer correctly -- it can
    /// simply skip one on every call, since [`EventHeap::iter`] never
    /// mutates the heap.
    pub fn next_digital_due_time(&self) -> f64 {
        self.heap
            .iter()
            .filter_map(|ev| match ev.kind {
                EventKind::Commit { driver, token, .. } if self.drivers[driver.0 as usize].token() == token => Some(ev.time),
                _ => None,
            })
            .fold(f64::INFINITY, f64::min)
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
    /// W2c's seed, at load before [`Self::settle`]: driver `id`'s carried
    /// committed level and in-flight transition, whose `Commit` goes on the
    /// heap at its absolute due time.
    pub fn seed_driver(&mut self, id: DriverId, committed: Option<Mode>, pending: Option<(Mode, f64)>) {
        let pending = pending.map(|(target, due)| crate::drivers::Pending { target, due });
        if let (Some(token), Some(p)) = (self.drivers[id.0 as usize].seed(committed, pending), pending) {
            self.heap.push(p.due, EventKind::Commit { driver: id, mode: p.target, token });
        }
    }

    /// W2c's seed: a carried device timer (an EEPROM write completing).
    pub fn seed_timer(&mut self, device: DeviceId, at: f64, tag: u64) {
        self.heap.push(at, EventKind::Timer { device, tag });
    }

    pub fn add_driver(&mut self, driver: Driver, partition: Option<PartitionId>) -> DriverId {
        let id = DriverId(self.drivers.len() as u32);
        self.drivers.push(driver);
        self.driver_partition.push(partition);
        self.driver_journaled.push(false);
        id
    }

    /// Requests `level` for `driver` at `now`, exactly as a `DeviceEvaluator`
    /// would via `DriverRequest::Level`, but callable directly for a
    /// stimulus that does not originate from a reader's own evaluation — a
    /// harness toggle, or W1b's decision-1 parameter-update event. Goes
    /// through the same FSM and event-scheduling path `advance`'s delta
    /// cycle itself uses.
    pub fn request_driver(&mut self, driver: DriverId, level: Mode, now: f64, delay: f64) {
        self.apply_request(DriverRequest::Level { driver, level, delays: DelaySet::uniform(delay) }, now, false);
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

    /// Applies one `DriverRequest`, pushing a `Commit` (or `Timer`) event
    /// onto the heap when appropriate. `zero_delays`, when set, replaces
    /// whatever [`DelaySet`] a `Level` request carries with an all-zero one
    /// — used by [`Self::run_settle_pass`] to force zero delay throughout
    /// (decision 8, delays point 2: "the settle override zeroes the whole
    /// set").
    ///
    /// Returns the driver's partition when a `Reset` actually changed
    /// something (contract, "Kernel semantics: events", point 2: "a reset
    /// ... marks its partition dirty for the next re-solve; readers of that
    /// partition are then re-evaluated") — a `Level` or `Timer` request
    /// never returns one, because its effect (if any) is an event that will
    /// mark the relevant partition dirty (or fold its device into
    /// `directly_affected`) itself, through the ordinary heap/
    /// `apply_due_events` path, once it becomes due. A `Reset` has no due
    /// time of its own: unlike a commit, it takes effect the instant it is
    /// requested (`Driver::reset`), so this is the only signal a caller gets
    /// that it needs to fold the partition into whatever dirty set feeds the
    /// next re-solve.
    fn apply_request(&mut self, req: DriverRequest, now: f64, zero_delays: bool) -> Option<PartitionId> {
        match req {
            DriverRequest::Level { driver, level, delays } => {
                let idx = driver.0 as usize;
                // A request that changes nothing (most of them: a reader
                // re-evaluated because a neighbour moved asks for what it
                // already asked) needs neither the FSM nor a journal entry.
                if self.drivers[idx].request_is_noop(level) {
                    return None;
                }
                // A carried driver is no first observation: classic's seed
                // solve after a load times its transitions by the real
                // delays from the carried slots (W2c).
                let delays = if zero_delays && !self.drivers[idx].seeded() { DelaySet::uniform(0.0) } else { delays };
                self.journal_driver(idx);
                if let Some((due, mode, token)) = self.drivers[idx].request(level, now, &delays) {
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
                self.journal_driver(idx);
                self.solver.driver_changing(idx);
                self.drivers[idx].reset();
                if was_committed {
                    self.solver.driver_committed(idx);
                    self.evaluator.driver_changed(driver);
                    self.fixed_reset |= self.driver_partition[idx].is_none();
                    self.driver_partition[idx]
                } else {
                    None
                }
            }
            DriverRequest::Timer { device, at, tag } => {
                self.heap.push(at, EventKind::Timer { device, tag });
                None
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
    ///
    /// `dirty` and `directly_affected` are appended to, unsorted and possibly
    /// with repeats: the caller sorts and dedups them once before use
    /// (ascending, unique: the order the `BTreeSet`s this used to return
    /// iterated in), which costs one small sort per pass instead of a node
    /// allocation per insert.
    fn apply_due_events(&mut self, dirty: &mut Vec<PartitionId>, directly_affected: &mut Vec<DeviceId>) -> Result<(u64, u64), KernelError> {
        let mut popped = 0u64;
        let mut changed = 0u64;
        while self.heap.peek_time() <= self.time {
            let ev = self.heap.pop().expect("peek_time <= time implies an event is present");
            if self.windowed {
                self.heap_popped.push(ev);
            }
            popped += 1;
            match ev.kind {
                EventKind::Rail { .. } | EventKind::Slow { .. } => {}
                _ => self.instant_other = true,
            }
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
                    } else {
                        self.journal_driver(idx);
                        self.solver.driver_changing(idx);
                        if self.drivers[idx].apply_commit(mode, token) {
                            self.solver.driver_committed(idx);
                            self.evaluator.driver_changed(driver);
                            if let Some(p) = self.driver_partition[idx] {
                                dirty.push(p);
                            }
                            changed += 1;
                        }
                    }
                }
                EventKind::Crossing { partition, version } => {
                    if self.solver.is_current(partition, true, version) {
                        self.solver.handle_dynamic_event_into(partition, self.time, true, directly_affected).map_err(KernelError::SolveFailed)?;
                    }
                }
                EventKind::Step { partition, version } => {
                    if self.solver.is_current(partition, false, version) {
                        self.solver.handle_dynamic_event_into(partition, self.time, false, directly_affected).map_err(KernelError::SolveFailed)?;
                    }
                }
                EventKind::Timer { device, .. } => {
                    // W1d, "Kernel semantics: events (W1a)" point 2: "a
                    // popped Timer adds its device to that delta's affected
                    // readers, so the device is evaluated in the delta its
                    // Timer pops." A `Timer` carries no version to go stale
                    // by (events.rs's own doc comment: no producer needed
                    // one before W1c's floating buckets, and W1d's one
                    // producer — the EEPROM's pending-write completion —
                    // never needs to cancel one either: a stray pop simply
                    // finds nothing left to do, contract point 5 under
                    // "Kernel semantics: memories").
                    directly_affected.push(device);
                }
                EventKind::Bucket => {
                    let (dirty_partitions, readers) = self.solver.bucket_boundary(self.time);
                    dirty.extend(dirty_partitions);
                    directly_affected.extend(readers);
                }
                // W1f (Sources point 5): dispatched like `Bucket`, each a
                // solver method returning the partitions to re-solve and the
                // readers to evaluate; a stale version drops silently.
                EventKind::Rail { version } => {
                    if self.solver.rail_current(version) {
                        self.instant_rail = true;
                        let (dirty_partitions, readers) = self.solver.rail_event(self.time).map_err(KernelError::SolveFailed)?;
                        dirty.extend(dirty_partitions);
                        directly_affected.extend(readers);
                    }
                }
                EventKind::Source { device, version } => {
                    if self.solver.source_current(device, version) {
                        self.instant_source = true;
                        let (dirty_partitions, readers) = self.solver.source_edge(device, version, self.time).map_err(KernelError::SolveFailed)?;
                        dirty.extend(dirty_partitions);
                        directly_affected.extend(readers);
                    }
                }
                // W1g point 14: a current slow event closes its device and
                // applies the latch, re-arm or PTC transition (or marks an
                // `output_sag` window due for the settle); the partitions a
                // stamp change dirtied re-solve in this delta.
                EventKind::Slow { device, token } => {
                    if self.solver.slow_is_current(device, token) {
                        // R21: a re-anchor fires nothing and moves no node.
                        if token & crate::events::SLOW_ANCHOR != 0 {
                            dirty.extend(self.solver.handle_slow_anchor(device, self.time).map_err(KernelError::SolveFailed)?);
                            self.instant_redrive = true;
                        } else {
                            dirty.extend(self.solver.handle_slow_event(device, self.time).map_err(KernelError::SolveFailed)?);
                            self.instant_other = true;
                        }
                    } else {
                        self.instant_other = true;
                    }
                }
            }
        }
        Ok((popped, changed))
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

    /// Calls [`DeviceEvaluator::on_settled`] for the instant just completed
    /// at `self.time` and queues whatever it returns into
    /// [`Self::pending_digital_writes`] — the one call site every one of
    /// [`Self::run_deltas`]'s per-instant success, the horizon clamp's own
    /// success, and [`Self::apply_params_as_event`]'s success routes through
    /// (see each call site's own comment for why THERE, specifically, is "an
    /// instant completed").
    fn publish_settled(&mut self) {
        let now = self.time;
        let mut writes = self.settled_buf.take();
        writes.clear();
        let holds = if self.holds_flag.as_ref().is_none_or(|f| f.get()) { self.solver.power_holds() } else { &[] };
        self.evaluator.on_settled_into(now, &self.drivers, &self.solver, holds, &mut writes);
        self.pending_digital_writes.extend(writes.iter().map(|&(slot, value)| (now, slot, value)));
        self.settled_buf.put(writes);
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
    /// `seed_affected` exists for [`Self::advance`]'s horizon clamp (finding
    /// 1): the readers [`solve::PartitionSolver::sync_dynamic_to`] returns
    /// have no due commit and touch no dirty partition of their own (the
    /// clamp's forced steps already moved their node's published voltage
    /// directly), so they cannot reach [`Self::evaluate`] through `dirty`
    /// the way an ordinary `Crossing`/`Step`'s readers do — they are folded
    /// straight into the first pass's `affected` set instead, exactly as if
    /// a step had landed on them at `t_end`. Like `seed_dirty`, only the
    /// first pass ever sees it: the first pass drains the caller's `Vec` into
    /// `affected`, and every later pass's own (empty) leftover appends as a
    /// no-op.
    ///
    /// `count_stats` gates every `self.stats` update in here: `true` from
    /// [`Self::advance`], `false` from [`Self::settle`]'s hand-off, which is
    /// still load-time bookkeeping, not a live delta cycle (`KernelStats`'s
    /// doc comment: only `advance`'s real delta cycle counts towards these,
    /// the same line `run_settle_pass` already draws for its own passes).
    fn run_deltas(&mut self, mut seed_dirty: Vec<PartitionId>, mut seed_affected: Vec<DeviceId>, count_stats: bool) -> Result<(), KernelError> {
        let mut settled = false;
        let mut depth = 0u64;
        let mut carried_dirty: Vec<PartitionId> = Vec::new();
        // The scratch buffers are taken out of `self` for the whole cycle
        // (the loop below calls `&mut self` methods) and put back at the end;
        // an early `?` return just drops them and the next call regrows them.
        let mut dirty = self.dirty_buf.take();
        let mut affected = self.affected_buf.take();
        let mut requests = self.requests_buf.take();
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
            dirty.clear();
            affected.clear();
            let (popped, changed) = self.apply_due_events(&mut dirty, &mut affected)?;
            dirty.append(&mut seed_dirty);
            dirty.append(&mut carried_dirty);
            affected.append(&mut seed_affected);
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
            //
            // `|| changed > 0` (finding 7): a commit can also land on a
            // driver whose own node is fixed (`driver_partition[idx] ==
            // None`, contract point 4) — no partition needs re-solving, but
            // `resolve_dirty` still needs to run, with `dirty` possibly
            // empty, so it can recompute source currents and refresh
            // `driver_snapshot` from the just-changed `self.drivers`, the
            // same fallback `Self::apply_params_as_event`'s own
            // `dirty.is_empty()` branch already takes for a live-param
            // change on an already-fixed device. Without this, a fixed-node
            // driver's commit left the published source current (and
            // `driver_snapshot`) exactly one commit stale until some
            // unrelated event happened to dirty a partition.
            // A reset of a fixed-node driver (`fixed_reset`) is the same case.
            // The flag is taken before the test, as the settle pass takes it:
            // inside the `||` chain a commit or a dirty partition left it set
            // for the next instant.
            let fixed_reset = std::mem::take(&mut self.fixed_reset);
            if !dirty.is_empty() || changed > 0 || fixed_reset {
                dirty.sort_unstable();
                dirty.dedup();
                self.solver.resolve_dirty_into(&dirty, self.time, &self.drivers, &mut affected).map_err(KernelError::SolveFailed)?;
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
                // W1g point 14: once the cycle is quiet, the slow settle closes
                // and re-drives the devices of every partition this instant
                // published and runs the `output_sag` scan. A latch due at once
                // dirties partitions, and another delta cycle runs at this same
                // instant (under the same pass cap), then this again.
                let Some(slow_dirty) = self.solver.slow_settle(self.time, &self.drivers).map_err(KernelError::SolveFailed)? else {
                    settled = true;
                    break;
                };
                self.drain_scheduled();
                if slow_dirty.is_empty() && self.heap.peek_time() > self.time {
                    settled = true;
                    break;
                }
                carried_dirty.extend(slow_dirty);
                self.evaluator.new_latch_baseline();
                continue;
            }

            // Phase 4: evaluate each affected reader exactly once, in
            // ascending device order.
            affected.sort_unstable();
            affected.dedup();
            // Nothing in this loop sets a hold, and the solver is a trait
            // object: the shared flag says once per pass whether there is a
            // table to fetch per reader, with no call at all (the SAP-1
            // never holds). The latch evidence below comes back with each
            // evaluation.
            let held = self.holds_flag.as_ref().is_none_or(|f| f.get());
            let mut latch_loop = false;
            for &reader in &affected {
                requests.clear();
                let holds = if held { self.solver.power_holds() } else { &[] };
                latch_loop |= self.evaluator.evaluate_into(reader, self.time, true, self.solver.voltages(), holds, &self.drivers, &mut requests);
                debug_assert!(
                    one_request_per_level_driver(&requests),
                    "evaluator returned a batch in which a driver is named by a Level and by another Level or Reset: {requests:?} (see DeviceEvaluator::evaluate_into)"
                );
                for req in requests.drain(..) {
                    if let Some(p) = self.apply_request(req, self.time, false) {
                        carried_dirty.push(p);
                    }
                }
            }
            // A latch that flipped back within this instant has no
            // consistent state (NE555 point 11): the loop would run to the
            // cap, so it is declined now, as the livelock it is. A pass with
            // no reader asks the evaluator itself.
            if latch_loop || (affected.is_empty() && self.evaluator.zero_delay_loop()) {
                break;
            }
        }
        self.dirty_buf.put(dirty);
        self.affected_buf.put(affected);
        self.requests_buf.put(requests);

        if count_stats && depth > self.stats.max_delta_depth {
            self.stats.max_delta_depth = depth;
        }
        if !settled {
            return Err(KernelError::Livelock { at: self.time });
        }
        Ok(())
    }

    /// Whether the instant just completed inside a run moved the published
    /// voltages on from [`Self::run_before`]: at least one changed, none
    /// against its own last move in the run, and the rail if only `Rail`
    /// events ran (see [`NO_PROGRESS_MAX`]). Records each moved node's
    /// direction for the next instant.
    fn progressed(&mut self) -> bool {
        let now = self.solver.voltages();
        if self.run_moves.len() != now.len() {
            self.run_moves.clear();
            self.run_moves.resize(now.len(), 0);
        }
        let (mut moved, mut turned) = (false, false);
        for ((&after, &before), last) in now.iter().zip(&self.run_before).zip(self.run_moves.iter_mut()) {
            let step = move_sign(before, after);
            if step == 0 {
                continue;
            }
            moved = true;
            turned |= *last == -step;
            *last = step;
        }
        let rail_held = self.instant_rail && !self.instant_other && self.solver.rail_node().is_some_and(|n| matches!((self.run_before.get(n), now.get(n)), (Some(&b), Some(&a)) if move_sign(b, a) == 0));
        moved && !turned && !rail_held
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
        self.advance_budgeted(t_end, None).map(|_| ())
    }

    /// [`Self::advance`] under a wall budget (ruling R86): between two
    /// instants, once the host's clock says the call has run `budget_ms`, it
    /// returns [`Advanced::Yielded`] with the state of the last completed
    /// instant, and an advance to the same `t_end` goes on with the same host
    /// step (its span for the hold guard, its event count), so a run split
    /// by yields is the same run, bit for bit, as one without them. It never
    /// yields before an instant has run in the call. Without a budget, or
    /// with a clock that reads no finite time, it is [`Self::advance`].
    pub fn advance_budgeted(&mut self, t_end: f64, mut budget: Option<Budget>) -> Result<Advanced, KernelError> {
        // A host step stays open from its first call until it reaches its
        // `t_end` or an edit ends it: a call to the same `t_end` resumes it,
        // whether or not the yield landed on `t_end` itself (R86V finding 6).
        let resumed = self.host_step.is_some_and(|h| h.t_end.to_bits() == t_end.to_bits());
        if !resumed {
            if t_end > self.time {
                self.solver.begin_host_step(self.time, t_end);
            }
            self.host_step = Some(HostStep { t_end, events_at_start: self.stats.events_processed, span: (t_end - self.time).max(0.0) });
        }
        let mut reads: Option<Reads> = match budget.as_mut() {
            Some(b) if b.budget_ms > 0.0 => {
                let now = (b.clock)();
                now.is_finite().then_some(Reads { start: now, last: now, instants: 0, ran: false })
            }
            _ => None,
        };
        let mut result = self.advance_loop(t_end, &mut budget, &mut reads);
        // The call's end read: its busy time counts, and the progress floor
        // is judged there too, so short host steps of slow instants that
        // never reach a check are judged (R86V finding 1).
        if let (Some(b), Some(r), Ok(Advanced::Reached)) = (budget.as_mut(), reads.as_mut(), &result) {
            if let Err(e) = self.clock_read(b, r) {
                result = Err(e);
            }
        }
        if matches!(result, Ok(Advanced::Reached)) {
            self.host_step = None;
        }
        result
    }

    /// Ends the host step in flight (ruling R86): a host's edit, a memory
    /// seed, an option change or a failure reset between two yields, so the
    /// next advance starts a new one.
    /// The kind of the event at the head of the heap (tests).
    #[cfg(test)]
    pub(crate) fn next_event_kind(&self) -> Option<EventKind> {
        self.heap.peek().map(|e| e.kind)
    }

    pub fn end_host_step(&mut self) {
        self.host_step = None;
    }

    /// The driver table, by driver id (W2c's carry reads its commits).
    pub(crate) fn drivers(&self) -> &[Driver] {
        &self.drivers
    }

    /// Whether the kernel is between two calls (W2a): no instant window open
    /// and no digital write waiting for the engine to publish it. Snapshots
    /// are taken and restored only there.
    pub(crate) fn is_quiescent(&self) -> bool {
        !self.windowed && self.pending_digital_writes.is_empty()
    }

    /// The wall time the budgeted advances have run, R86's busy clock.
    pub(crate) fn busy_ms(&self) -> f64 {
        self.busy_ms
    }

    /// After a W2a restore put the whole kernel back: the busy time is wall
    /// telemetry, so it keeps the live value, and the progress window
    /// re-anchors at (busy now, restored time), as a host edit leaves it to
    /// a fresh judgement, since a restore moves simulated time back while
    /// wall time runs on. The host step, its event count and the check
    /// stride stay as the snapshot had them, with the event counter they
    /// were taken against.
    pub(crate) fn rebase_after_restore(&mut self, busy_ms: f64) {
        self.busy_ms = busy_ms;
        self.window = (busy_ms, self.time);
    }

    /// One read of the host clock inside a budgeted call (ruling R86): adds
    /// the time since the last read to the busy time, at most
    /// [`BUSY_READ_CAP_MS`] or four budgets (a longer gap is a pause, as a
    /// suspended host's is: R86V finding 3); re-anchors on a clock that went
    /// back (R86V finding 4); adapts the check stride to the time an instant
    /// takes; judges the progress floor; and says whether the call's budget
    /// is spent with an instant run in it.
    fn clock_read(&mut self, b: &mut Budget, r: &mut Reads) -> Result<bool, KernelError> {
        let now = (b.clock)();
        if !now.is_finite() {
            return Ok(false);
        }
        let delta = now - r.last;
        let instants = std::mem::take(&mut r.instants);
        if delta < 0.0 {
            r.start = now;
            r.last = now;
            return Ok(false);
        }
        r.last = now;
        let counted = delta.min((4.0 * b.budget_ms).max(BUSY_READ_CAP_MS));
        self.busy_ms += counted;
        if instants > 0 {
            // Shrink the stride at once to what an instant's time allows; let
            // it grow back by a quarter a read, so one fast stretch does not
            // undo it (R86V finding 1: 64 instants of 100 ms against 16 ms).
            let per_instant = counted / instants as f64;
            let stride = self.check_stride as f64;
            if per_instant * stride > b.budget_ms {
                self.check_stride = ((b.budget_ms / (2.0 * per_instant)) as u32).max(1);
            } else if 4.0 * per_instant * stride < b.budget_ms {
                self.check_stride = (self.check_stride + (self.check_stride / 4).max(1)).min(b.check_every.max(1));
            }
        }
        let span = self.busy_ms - self.window.0;
        if span >= PROGRESS_WINDOW_MS {
            let progress = self.time - self.window.1;
            // Judged, the window starts again here either way, so a run that
            // the host goes on with after a decline is judged afresh.
            self.window = (self.busy_ms, self.time);
            if progress / (span / 1e3) < PROGRESS_FLOOR {
                return Err(KernelError::NoProgress {
                    at: self.time,
                    message: format!("the run made {progress:.3e} s of progress in {:.0} s of wall time (t = {}), under the floor of {PROGRESS_FLOOR} s a second", span / 1e3, self.time),
                });
            }
        }
        Ok(r.ran && now - r.start >= b.budget_ms)
    }

    fn advance_loop(&mut self, t_end: f64, budget: &mut Option<Budget>, reads: &mut Option<Reads>) -> Result<Advanced, KernelError> {
        loop {
            // Ruling R94 (a): without a clock, the host step's event cap,
            // `HOST_STEP_EVENT_CAP` x max(1, its span in simulated seconds),
            // checked before the next instant runs, as every decline leaves
            // the last completed instant. With one, only the progress floor
            // governs.
            if reads.is_none() {
                if let Some(h) = self.host_step {
                    let taken = self.stats.events_processed - h.events_at_start;
                    let cap = (self.event_cap as f64 * h.span.max(1.0)) as u64;
                    if taken >= cap {
                        return Err(KernelError::NoProgress {
                            at: self.time,
                            message: format!("the host step to t = {t_end} took {taken} events without reaching it (stopped at t = {})", self.time),
                        });
                    }
                }
            }
            // R86 (a)-(c): yield only here, between instants, at a read of
            // the clock every `check_stride` instants.
            if let (Some(b), Some(r)) = (budget.as_mut(), reads.as_mut()) {
                self.since_check += 1;
                if self.since_check >= self.check_stride {
                    self.since_check = 0;
                    if self.clock_read(b, r)? {
                        return Ok(Advanced::Yielded);
                    }
                }
            }
            // A superseded event at the head is dropped here, between
            // instants, as its handler would drop it inside one: it is no
            // instant, so it neither runs nor counts toward the no-progress
            // guard (V9 MEDIUM 3: a host stepping every few nanoseconds
            // through a CC ramp re-solved the rail at each horizon, each
            // solve superseding the pending `Rail` event with a new version
            // for the same threshold, and some 60,000 stale versions popped
            // within 1 ps of each other and declined with -4). The state is
            // the last completed instant's, as a rollback leaves it, so an
            // event stale now stays stale after any rollback. It is still
            // popped during `advance`, so stat 4 counts it.
            while let Some(ev) = self.heap.peek() {
                if ev.time > t_end || self.event_live(ev.kind) {
                    break;
                }
                self.heap.pop();
                self.stats.events_processed += 1;
            }
            let t = self.heap.peek_time();
            if t > t_end {
                break;
            }

            // Checked before the instant runs, so a decline leaves the state of
            // the last completed instant exactly as it was, as every failed
            // advance does, and asking again declines again.
            let in_run = t - self.last_instant <= NO_PROGRESS_DT;
            if in_run && self.stalled >= NO_PROGRESS_MAX {
                return Err(KernelError::SolveFailed(crate::rails::loop_unsettled()));
            }
            if in_run {
                self.run_before.clear();
                self.run_before.extend_from_slice(self.solver.voltages());
            }
            self.instant_rail = false;
            self.instant_other = false;
            self.instant_source = false;
            self.instant_redrive = false;

            // W2b: the samples before this instant are final on the state
            // the last one left (one exactly at `t` reads after it, D2).
            self.solver.observe(t, false, self.time);
            let previous_time = self.time;
            let window = self.open_window();

            self.time = t;
            if let Err(e) = self.run_deltas(Vec::new(), Vec::new(), true) {
                self.time = previous_time;
                self.roll_back(window);
                return Err(e);
            }
            self.close_window(window);
            if !in_run {
                self.stalled = 0;
                self.run_moves.clear();
            } else if self.instant_source {
                // A source edge turns its node and, after it, whatever it
                // drives: each node's direction starts afresh from here. The
                // count stands, so a livelock elsewhere on the board still
                // accumulates between edges.
                self.run_moves.iter_mut().for_each(|m| *m = 0);
            } else if !self.progressed() && !(self.instant_redrive && !self.instant_other && !self.instant_rail) {
                // An instant that only re-anchored slow drives (W1g ruling
                // R21) moved nothing by design: it counts neither way. Each
                // device's re-anchors are at least `2 NO_PROGRESS_DT` apart,
                // so they cannot make a run of their own.
                self.stalled += 1;
            }
            self.last_instant = t;
            if let Some(r) = reads.as_mut() {
                r.ran = true;
                r.instants += 1;
            }
            // This instant reached a quiet fixed point and was NOT rolled
            // back: exactly "a completed instant" (W1c contract point 3), so
            // its digital-slot readout is published now, at ITS OWN `t`, not
            // deferred to wherever `advance`'s loop or the horizon clamp
            // below eventually stops.
            self.publish_settled();
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
        self.solver.observe(t_end, false, self.time);
        let previous_time = self.time;
        let window = self.open_window();

        self.time = t_end;
        // W1a contract point 5 (amended by finding 1): a stepped partition's
        // forced catch-up steps inside `sync_dynamic_to` can move a node an
        // IC reads, with no `Step`/`Crossing` event of its own ever landing
        // on the heap for the ordinary delta machinery above to pick up —
        // `stepped::sync_to`'s own readers were previously bound and
        // immediately discarded here (`Ok(_readers) =>`), leaving such a
        // reader stale until some unrelated later event happened to
        // re-evaluate it. Those readers are now fed through the same
        // evaluate/apply-request/re-solve machinery `run_deltas` already
        // gives an ordinary due `Step`/`Crossing`'s readers, seeded as this
        // one-pass-or-more delta cycle's initial `affected` set, quiet or
        // `KernelError::Livelock` (`-7`) guarded exactly like any other
        // instant.
        let clamp_result: Result<(), KernelError> = match self.solver.sync_dynamic_to(t_end) {
            // No reader means the clamp genuinely had nothing new to
            // evaluate (the common case: no dynamic partition, or every one
            // of them was already exactly current) — skip `run_deltas`
            // entirely rather than spend a delta pass (and its stats) on a
            // guaranteed no-op, exactly matching what this branch always did
            // before finding 1's fix.
            Ok(readers) if readers.is_empty() => {
                self.drain_scheduled();
                // The clamp published the dynamic partitions at `t_end`, so the
                // slow devices settle there too (W1g point 2).
                self.settle_slow_after(true)
            }
            Ok(readers) => self.run_deltas(Vec::new(), readers, true),
            Err(e) => Err(KernelError::SolveFailed(e)),
        };
        match clamp_result {
            Ok(()) => {
                // The horizon clamp is its own completed "instant" at
                // `t_end` (it has its own snapshot/rollback envelope above,
                // exactly like any other) — a dynamic partition's published
                // voltages moved here even though nothing was due, so a
                // digital reader watching one of its nodes could too. When
                // there WERE readers, `run_deltas` above already drained
                // every scheduled Crossing/Step this pass produced
                // (`drain_scheduled` runs inside it every pass); the empty-
                // reader branch above drains it directly instead, so either
                // way nothing more is needed here before publishing.
                self.close_window(window);
                self.publish_settled();
                // W2b: a sample at `t_end` is this readout (D2).
                self.solver.observe(t_end, true, t_end);
                Ok(Advanced::Reached)
            }
            Err(e) => {
                self.time = previous_time;
                self.roll_back(window);
                Err(e)
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
        let window = self.open_window();
        let old_values: Vec<(usize, usize, f64)> = triples
            .iter()
            .map(|&(d, p, _)| (d, p, self.solver.param_value(d, p).expect("validate_params must run and succeed before apply_params_as_event")))
            .collect();

        let mut dirty: Vec<PartitionId> = self.solver.apply_params(triples);
        // W2c: a voltage source's V is a source edge now.
        let (edge_dirty, edge_readers) = match self.solver.apply_source_params(triples, self.time) {
            Ok(edge) => edge,
            Err(e) => {
                // The window puts the sources and the boundary back.
                self.roll_back(window);
                self.solver.apply_params(&old_values);
                return Err(KernelError::SolveFailed(e));
            }
        };
        dirty.extend(edge_dirty);
        // W1c point 1a: the circuit changed, so every power hold goes (a
        // ping-pong still there re-holds at once), inside the window; a
        // released IC reads at once.
        let (released, mut readers) = self.solver.drop_power_holds();
        readers.extend(edge_readers);
        dirty.extend(released);
        dirty.sort_unstable();
        dirty.dedup();

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
        let result = if dirty.is_empty() && readers.is_empty() {
            self.solver
                .resolve_dirty(&[], self.time, &self.drivers)
                .map(|_| ())
                .map_err(KernelError::SolveFailed)
                .and_then(|()| self.settle_slow_after(true))
        } else {
            self.run_deltas(dirty, readers, true)
        };

        if let Err(e) = result {
            self.roll_back(window);
            // The window puts a voltage source's value and the boundary it
            // fixed back with the rest.
            self.solver.apply_params(&old_values);
            return Err(e);
        }
        self.close_window(window);
        // A host's edit is new input, not the circuit chasing itself: without
        // this, once the guard declined, every later advance would decline
        // too, even after the edit that ends the livelock.
        self.stalled = 0;
        // R86: and a new host step begins. The progress window stands: an
        // edit moves no simulated time (R86V finding 2).
        self.host_step = None;
        // The one event this call applies reached a quiet fixed point and
        // was not rolled back: a completed instant, same as any `advance`
        // delta.
        self.publish_settled();
        Ok(())
    }

    /// W1g's slow settle outside a delta cycle (a step [`Self::run_deltas`]
    /// takes at quiescence): after the horizon clamp or an empty resolve,
    /// which published without running a cycle. A latch due at once runs one.
    fn settle_slow_after(&mut self, count_stats: bool) -> Result<(), KernelError> {
        let Some(dirty) = self.solver.slow_settle(self.time, &self.drivers).map_err(KernelError::SolveFailed)? else {
            return Ok(());
        };
        self.drain_scheduled();
        if dirty.is_empty() && self.heap.peek_time() > self.time {
            return Ok(());
        }
        self.run_deltas(dirty, Vec::new(), count_stats)
    }

    /// `bo_engine_reset_failures` (README W1g point 13) as ONE event at the
    /// current time, the envelope of [`Self::apply_params_as_event`]: the
    /// solver clears every latch, stress integrator, PTC and `output_sag`
    /// window, the partitions whose stamps returned re-solve in a delta cycle
    /// at this instant, and the slow devices settle. It does not advance time.
    /// A failed re-solve (`-4`, `-7`) rolls the reset back with the instant:
    /// the slow state is instant state, so the engine is exactly as it was.
    pub fn reset_failures_as_event(&mut self) -> Result<(), KernelError> {
        // R86: host input between yields ends the host step, as an edit does.
        self.end_host_step();
        let window = self.open_window();
        let result = match self.solver.reset_failures(self.time) {
            Err(e) => Err(KernelError::SolveFailed(e)),
            Ok(mut dirty) => {
                // W1c point 1a, as at `set_params`.
                let (released, readers) = self.solver.drop_power_holds();
                dirty.extend(released);
                dirty.sort_unstable();
                dirty.dedup();
                if dirty.is_empty() && readers.is_empty() {
                    self.solver
                        .resolve_dirty(&[], self.time, &self.drivers)
                        .map(|_| ())
                        .map_err(KernelError::SolveFailed)
                        .and_then(|()| self.settle_slow_after(true))
                } else {
                    self.run_deltas(dirty, readers, true)
                }
            }
        };
        if let Err(e) = result {
            self.roll_back(window);
            return Err(e);
        }
        self.close_window(window);
        // New input, as a host's edit is (see `apply_params_as_event`).
        self.stalled = 0;
        self.publish_settled();
        Ok(())
    }

    /// Test builds: sets one IC's power hold as ONE event at the current time,
    /// the envelope of [`Self::reset_failures_as_event`]: the partitions its
    /// stamps touch re-solve and the IC is evaluated, in a delta cycle at
    /// this instant. The hold procedure (F5b) sets holds inside a failing
    /// solve instead; this is how a test reaches a held IC without one.
    #[cfg(test)]
    pub(crate) fn hold_power_as_event(&mut self, device: usize, hold: PowerHold) -> Result<(), KernelError> {
        let window = self.open_window();
        let dirty = self.solver.hold_power(device, hold);
        if let Err(e) = self.run_deltas(dirty, vec![DeviceId(device as u32)], true) {
            self.roll_back(window);
            return Err(e);
        }
        self.close_window(window);
        self.publish_settled();
        Ok(())
    }

    /// Applies `bo_engine_seed_memory` as ONE event at the current time
    /// (core/README.md's own section, W1d owner decision 2), mirroring
    /// [`Self::apply_params_as_event`]'s shape: snapshot everything first,
    /// mutate the evaluator through [`DeviceEvaluator::seed_memory`] (the
    /// hook decides whether `device` even accepts a seed at all — the
    /// evaluator's own state is restored either way, so a rejection is as
    /// safe as a no-op even if a future implementation ever mutated before
    /// detecting one), then run a delta cycle seeded with `device` as this
    /// pass's sole `directly_affected` reader (contract: "the device is
    /// re-evaluated in a delta cycle there, its readout shows the new word
    /// at once, and an output the new word changes requests it with the
    /// addrO pair"). Rolls back exactly like `apply_params_as_event` on
    /// `-4`/`-7`.
    pub fn seed_memory(&mut self, device: DeviceId, words: &[f64]) -> Result<(), SeedMemoryError> {
        // R86: host input between yields ends the host step, as an edit does.
        self.end_host_step();
        let window = self.open_window();

        if let Err(msg) = self.evaluator.seed_memory(device.0 as usize, words) {
            self.abandon_window(window);
            return Err(SeedMemoryError::Rejected(msg));
        }

        // W1c point 1a, as at `set_params`: an accepted seed drops every
        // power hold, inside the window.
        let (dirty, mut readers) = self.solver.drop_power_holds();
        readers.push(device);
        readers.sort_unstable();
        readers.dedup();
        if let Err(e) = self.run_deltas(dirty, readers, true) {
            self.roll_back(window);
            return Err(SeedMemoryError::Kernel(e));
        }
        self.close_window(window);
        // New input, as in `apply_params_as_event`.
        self.stalled = 0;
        // This instant reached a quiet fixed point and was not rolled back:
        // a completed instant, same as any `advance` delta or
        // `apply_params_as_event`'s own event.
        self.publish_settled();
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
        let mut reset_dirty: Vec<PartitionId> = Vec::new();
        for &reader in all_readers {
            let requests = self.evaluator.evaluate_held(reader, self.time, acting_on_edges, self.solver.voltages(), self.solver.power_holds());
            for req in requests {
                if let Some(p) = self.apply_request(req, self.time, true) {
                    reset_dirty.push(p);
                }
            }
        }
        let mut dirty = Vec::new();
        let mut directly_affected = Vec::new();
        let (_popped, commits_changed) = self.apply_due_events(&mut dirty, &mut directly_affected)?;
        let fixed_reset = std::mem::take(&mut self.fixed_reset);
        let changed = commits_changed > 0 || fixed_reset || !reset_dirty.is_empty() || !directly_affected.is_empty();
        dirty.append(&mut reset_dirty);
        let is_first_pass = *resolve_all_once;
        let to_resolve: Vec<PartitionId> = if is_first_pass {
            *resolve_all_once = false;
            all_partitions.to_vec()
        } else {
            dirty.sort_unstable();
            dirty.dedup();
            dirty
        };
        // `|| commits_changed > 0`: finding 7's rule in `run_deltas`, here for
        // the settle. A commit that lands only on fixed-node drivers dirties
        // no partition, yet the source KCL and the solver's driver table must
        // take it; without the resolve a load whose last settle pass flips
        // such a driver published its old level until some later event
        // resolved (cold 64 tpl 5, hub 2). A reset of one is the same case.
        if is_first_pass || !to_resolve.is_empty() || commits_changed > 0 || fixed_reset {
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
        // W1f (Sources point 4): a rail move over 1e-9 V or a regime change
        // in this pass's resolve changes the fixed point too.
        let rail_changed = self.solver.take_settle_change();
        Ok(changed || rail_changed)
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

        let mut edge_pass_dirty: Vec<PartitionId> = Vec::new();
        for &reader in all_readers {
            let requests = self.evaluator.evaluate_held(reader, now, true, self.solver.voltages(), self.solver.power_holds());
            for req in requests {
                if let Some(p) = self.apply_request(req, now, false) {
                    edge_pass_dirty.push(p);
                }
            }
        }

        self.run_deltas(edge_pass_dirty, Vec::new(), false)?;
        // Load settling is itself one completed "instant" at `now` (the
        // three-step process above is how it gets there, not three separate
        // ones — core/README.md's own "Load settling" point 6 describes it
        // as a single settling procedure): publish once, at the end, exactly
        // like `advance`'s own per-instant call does.
        self.publish_settled();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::{DelaySet, Driver};
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
        k.apply_request(DriverRequest::Level { driver: d0, level: High, delays: DelaySet::uniform(5.0) }, 0.0, false);

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
        k.apply_request(DriverRequest::Level { driver: d0, level: High, delays: DelaySet::uniform(5.0) }, 0.0, false);
        k.apply_request(DriverRequest::Level { driver: d1, level: High, delays: DelaySet::uniform(5.0) }, 0.0, false);

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
                    vec![DriverRequest::Level { driver: self.drivers[next], level: High, delays: DelaySet::uniform(0.0) }]
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
        k.apply_request(DriverRequest::Level { driver: d0, level: High, delays: DelaySet::uniform(0.0) }, 0.0, false);

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
                vec![DriverRequest::Level { driver: self.driver, level, delays: DelaySet::uniform(0.0) }]
            }
        }

        // `add_driver` assigns ids sequentially from 0, so the first driver
        // added is known to be `DriverId(0)` before it exists -- needed here
        // because the mock evaluator must be constructed (naming the driver
        // it will oscillate) before the kernel that will assign that id.
        let d0 = DriverId(0);
        let mut k = Kernel::new(AlwaysAffects(DeviceId(0)), OscillatingEvaluator { driver: d0, next: High });
        assert_eq!(k.add_driver(driver(1), Some(PartitionId(0))), d0);
        k.apply_request(DriverRequest::Level { driver: d0, level: Low, delays: DelaySet::uniform(0.0) }, 0.0, false);

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
                    1 => vec![DriverRequest::Level { driver: self.driver, level: High, delays: DelaySet::uniform(0.0) }],
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

    /// Records each resolve's instant and dirty set.
    struct ResolveLog {
        seen: Vec<(f64, Vec<PartitionId>)>,
    }
    impl PartitionSolver for ResolveLog {
        fn resolve_dirty(&mut self, dirty: &[PartitionId], now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
            self.seen.push((now, dirty.to_vec()));
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

    /// Drives a fixed-node driver HIGH while settling, schedules itself at
    /// t = 1 from the edge pass, and resets the driver there; with
    /// `reset_in_settle` the reset comes in the settle's second pass instead.
    struct FixedNodeReset {
        driver: DriverId,
        reset_in_settle: bool,
        calls: u32,
    }
    impl DeviceEvaluator for FixedNodeReset {
        fn evaluate(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
            self.calls += 1;
            if self.reset_in_settle {
                return match self.calls {
                    1 => vec![DriverRequest::Level { driver: self.driver, level: High, delays: DelaySet::uniform(0.0) }],
                    2 => vec![DriverRequest::Reset { driver: self.driver }],
                    _ => Vec::new(),
                };
            }
            if now == 0.0 && !acting_on_edges {
                vec![DriverRequest::Level { driver: self.driver, level: High, delays: DelaySet::uniform(0.0) }]
            } else if now == 0.0 {
                vec![DriverRequest::Timer { device: reader, at: 1.0, tag: 0 }]
            } else if now == 1.0 {
                vec![DriverRequest::Reset { driver: self.driver }]
            } else {
                Vec::new()
            }
        }
    }

    #[test]
    fn a_reset_of_a_committed_fixed_node_driver_is_resolved_in_a_delta_and_in_the_settle() {
        // Finding 7's rule for a `Reset`: a driver whose own node is fixed
        // dirties no partition, but the source KCL and the solver's driver
        // table must take its reset, so a resolve follows it at its instant.
        for reset_in_settle in [true, false] {
            let d0 = DriverId(0);
            let mut k = Kernel::new(ResolveLog { seen: Vec::new() }, FixedNodeReset { driver: d0, reset_in_settle, calls: 0 });
            assert_eq!(k.add_driver(driver(1), None), d0);
            k.settle(0.0, &[], &[DeviceId(0)]).unwrap();
            if reset_in_settle {
                assert_eq!(k.driver(d0).committed(), None, "the settle's reset applied");
                // The first pass resolves everything and commits; the second
                // resets, and must resolve after it.
                assert!(k.solver.seen.len() >= 2, "a resolve after the settle's reset: {:?}", k.solver.seen);
                continue;
            }
            assert_eq!(k.driver(d0).committed(), Some(High));
            let before = k.solver.seen.len();
            k.advance(2.0).unwrap();
            assert_eq!(k.driver(d0).committed(), None, "the reset at t = 1 applied");
            assert!(k.solver.seen[before..].iter().any(|(t, _)| *t == 1.0), "a resolve at the reset's instant: {:?}", &k.solver.seen[before..]);
        }
    }

    struct ResetAndCommit {
        fixed: DriverId,
        other: DriverId,
        calls: u32,
    }
    impl DeviceEvaluator for ResetAndCommit {
        fn evaluate(&mut self, reader: DeviceId, now: f64, acting_on_edges: bool, _v: &[f64]) -> Vec<DriverRequest> {
            let _ = reader;
            if now == 0.0 && !acting_on_edges {
                return vec![DriverRequest::Level { driver: self.fixed, level: High, delays: DelaySet::uniform(0.0) }];
            }
            if now == 0.0 {
                return vec![DriverRequest::Timer { device: reader, at: 1.0, tag: 0 }];
            }
            if now == 1.0 && self.calls == 0 {
                self.calls += 1;
                return vec![DriverRequest::Reset { driver: self.fixed }, DriverRequest::Level { driver: self.other, level: High, delays: DelaySet::uniform(0.0) }, DriverRequest::Timer { device: reader, at: 3.0, tag: 1 }];
            }
            Vec::new()
        }
    }

    #[test]
    fn a_fixed_node_reset_beside_a_commit_resolves_at_its_own_instant_only() {
        // A fixed-node driver's reset in the same pass as another driver's
        // commit: the commit already brings the resolve, and the reset's flag
        // must be spent there too, not left for the next instant (a timer
        // alone at t = 3), where it ran a resolve nothing asked for and made
        // a rolled-back instant differ from one never attempted.
        let d0 = DriverId(0);
        let d1 = DriverId(1);
        let mut k = Kernel::new(ResolveLog { seen: Vec::new() }, ResetAndCommit { fixed: d0, other: d1, calls: 0 });
        k.add_driver(driver(1), None);
        k.add_driver(driver(2), Some(PartitionId(0)));
        k.settle(0.0, &[PartitionId(0)], &[DeviceId(0)]).unwrap();
        k.advance(2.0).unwrap();
        assert_eq!(k.driver(d0).committed(), None);
        k.advance(4.0).unwrap();
        assert!(!k.solver.seen.iter().any(|(t, _)| *t == 3.0), "resolve at the timer-only instant 3.0");
        assert!(!k.fixed_reset, "fixed_reset leaked past the instant");
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
                vec![DriverRequest::Level { driver: self.driver, level, delays: DelaySet::uniform(123.0) }]
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
                            vec![DriverRequest::Level { driver: self.q, level: High, delays: DelaySet::uniform(10.0) }]
                        } else {
                            Vec::new()
                        }
                    }
                    INV => {
                        let q = self.shadow.borrow().q;
                        vec![DriverRequest::Level { driver: self.n, level: if q { Low } else { High }, delays: DelaySet::uniform(2.0) }]
                    }
                    NAND => {
                        let (q, n) = {
                            let s = self.shadow.borrow();
                            (s.q, s.n)
                        };
                        vec![DriverRequest::Level { driver: self.we, level: if q && n { Low } else { High }, delays: DelaySet::uniform(5.0) }]
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
                vec![DriverRequest::Level { driver: self.driver, level, delays: DelaySet::uniform(0.0) }]
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

    #[test]
    fn a_failed_instant_that_touched_several_drivers_and_heap_events_leaves_the_kernel_as_if_it_never_ran() {
        // The rollback no longer clones the heap and the driver table before
        // every instant: it journals the drivers an instant writes and the
        // events it pops, and truncates the events it pushes. This is the
        // case that has to hold for that to be a rollback and not a hope: one
        // doomed instant writes four drivers many times over (four commits
        // and thousands of re-requests, each pushing and popping heap
        // events), with pending events queued at later times, and afterwards
        // the kernel must be indistinguishable from a control kernel that
        // never attempted it: same drivers (tokens and pending transitions
        // included), same heap sequence counter, same next event, and the
        // same result once both run the same benign evaluator to the end.
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
        // Flips every driver on every call, at zero delay: the partition
        // never settles.
        struct FlipsAllFour {
            next: Mode,
        }
        impl DeviceEvaluator for FlipsAllFour {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                let level = self.next;
                self.next = if level == High { Low } else { High };
                (0..4).map(|d| DriverRequest::Level { driver: DriverId(d), level, delays: DelaySet::uniform(0.0) }).collect()
            }
        }
        struct Benign;
        impl DeviceEvaluator for Benign {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                Vec::new()
            }
        }

        type K = Kernel<AlwaysAffects, Box<dyn DeviceEvaluator>>;
        fn scenario() -> K {
            let mut k: K = Kernel::new(AlwaysAffects, Box::new(Benign));
            for node in 0..4 {
                k.add_driver(driver(node), Some(PartitionId(0)));
                // A first request is a first observation: it commits at once,
                // whatever the delay. Settle them all LOW at t = 0 so the
                // delayed requests below really are delayed.
                k.request_driver(DriverId(node as u32), Low, 0.0, 0.0);
            }
            k.advance(0.0).unwrap();
            // Two commits due together at t = 10 (FIFO between them), one at
            // 15, one beyond the horizon the test advances to.
            k.request_driver(DriverId(0), High, 0.0, 10.0);
            k.request_driver(DriverId(1), High, 0.0, 10.0);
            k.request_driver(DriverId(2), High, 0.0, 15.0);
            k.request_driver(DriverId(3), High, 0.0, 25.0);
            k
        }
        let dump = |k: &K| format!("{:?} seq={} next={} held={:?}", k.drivers, k.heap.next_seq(), k.heap.peek_time(), k.held_commits);

        let mut control = scenario();
        control.advance(20.0).unwrap();

        let mut doomed = scenario();
        doomed.set_evaluator(Box::new(FlipsAllFour { next: Low }));
        let before = dump(&doomed);
        let err = doomed.advance(20.0).unwrap_err();
        assert_eq!(err, KernelError::Livelock { at: 10.0 });
        assert_eq!(doomed.time(), 0.0, "the doomed instant never completed");
        assert_eq!(before, dump(&doomed), "drivers (tokens and pending transitions included), the heap's sequence counter and its next event are back exactly");
        assert!(doomed.driver_journal.is_empty() && doomed.heap_popped.is_empty() && !doomed.windowed, "no journal outlives its window");
        assert!(doomed.driver_journaled.iter().all(|&j| !j), "every journal mark is cleared");
        assert_eq!(doomed.next_event_time(), 10.0, "the commits that were about to run are still queued");

        // Both run the same benign evaluator from here; the whole state must
        // come out identical, which it can only do if the rollback put every
        // driver token and every queued event (FIFO order included) back.
        doomed.set_evaluator(Box::new(Benign));
        doomed.advance(20.0).unwrap();
        assert_eq!(dump(&control), dump(&doomed));
        assert_eq!(control.time(), doomed.time());
        for d in 0..4 {
            assert_eq!(control.driver(DriverId(d)).committed(), doomed.driver(DriverId(d)).committed());
        }
    }

    // ---- W1d: DriverRequest::Timer -----------------------------------------

    #[test]
    fn a_popped_timer_adds_its_device_to_this_deltas_affected_readers() {
        // A `Timer` carries no driver and no partition of its own: it only
        // exists to guarantee that the device which scheduled it gets one
        // more `evaluate` call, in the delta the timer pops, at the
        // timer's own time -- the EEPROM write-completion mechanism (W1d,
        // "Kernel semantics: memories", point 5).
        struct RecordingEvaluator {
            calls: Vec<(DeviceId, f64)>,
        }
        impl DeviceEvaluator for RecordingEvaluator {
            fn evaluate(&mut self, reader: DeviceId, now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                self.calls.push((reader, now));
                Vec::new()
            }
        }

        let mut k = Kernel::new(NoOpSolver, RecordingEvaluator { calls: Vec::new() });
        k.apply_request(DriverRequest::Timer { device: DeviceId(3), at: 5.0, tag: 42 }, 0.0, false);
        // Nothing has happened yet -- the Timer only sits on the heap until
        // its own due time, exactly like a Commit.
        assert!(k.evaluator.calls.is_empty());

        k.advance(5.0).unwrap();

        assert_eq!(k.evaluator.calls, vec![(DeviceId(3), 5.0)], "the device must be evaluated in the delta its own Timer pops");
    }

    #[test]
    fn next_event_time_reports_a_pending_timer_but_next_digital_due_time_does_not() {
        // W1d, "Kernel semantics: events (W1a)" point 2: "Timer events are
        // in the heap snapshot (rollback), counted by next_event_time and
        // NOT by next_digital_due_time."
        let mut k = Kernel::new(NoOpSolver, NoOpEvaluator);
        k.apply_request(DriverRequest::Timer { device: DeviceId(0), at: 7.0, tag: 1 }, 0.0, false);
        assert_eq!(k.next_event_time(), 7.0);
        assert_eq!(k.next_digital_due_time(), f64::INFINITY, "a Timer is not a driver commit");
    }

    // ---- W1d: Kernel::seed_memory -------------------------------------------

    #[test]
    fn seed_memory_rejects_through_the_evaluator_hook_without_touching_the_kernel() {
        struct DecliningEvaluator;
        impl DeviceEvaluator for DecliningEvaluator {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                Vec::new()
            }
            fn seed_memory(&mut self, _device: usize, _words: &[f64]) -> Result<(), String> {
                Err("not a memory".to_string())
            }
        }
        let mut k = Kernel::new(NoOpSolver, DecliningEvaluator);
        let err = k.seed_memory(DeviceId(0), &[1.0; 16]).unwrap_err();
        assert_eq!(err, SeedMemoryError::Rejected("not a memory".to_string()));
        assert_eq!(k.time(), 0.0, "a rejected seed must not run any event at all");
    }

    #[test]
    fn seed_memory_rolls_back_the_evaluator_hooks_own_mutation_on_a_forced_livelock() {
        // Same oscillation shape as `an_oscillating_zero_delay_pair_hits_the_cap_and_returns_livelock_not_a_silent_exit`
        // above, except the trigger is `Kernel::seed_memory` rather than
        // `advance`: the hook itself succeeds (recording `seeded`) before
        // the delta cycle it then runs never settles, so the whole instant
        // -- the hook's own mutation included -- must roll back
        // (core/README.md, `bo_engine_seed_memory`: "-4 or -7 rolls the
        // contents back with the instant").
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
            seeded: Option<f64>,
        }
        impl DeviceEvaluator for OscillatingEvaluator {
            fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
                let level = self.next;
                self.next = if level == High { Low } else { High };
                vec![DriverRequest::Level { driver: self.driver, level, delays: DelaySet::uniform(0.0) }]
            }
            fn seed_memory(&mut self, _device: usize, words: &[f64]) -> Result<(), String> {
                self.seeded = Some(words[0]);
                Ok(())
            }
            fn snapshot(&self) -> Box<dyn std::any::Any> {
                Box::new(self.seeded)
            }
            fn restore(&mut self, snapshot: Box<dyn std::any::Any>) {
                self.seeded = *snapshot.downcast::<Option<f64>>().expect("OscillatingEvaluator::restore: wrong snapshot type");
            }
        }

        let d0 = DriverId(0);
        let mut k = Kernel::new(AlwaysAffects(DeviceId(0)), OscillatingEvaluator { driver: d0, next: High, seeded: None });
        assert_eq!(k.add_driver(driver(1), Some(PartitionId(0))), d0);

        let err = k.seed_memory(DeviceId(0), &[9.0]).unwrap_err();
        assert_eq!(err, SeedMemoryError::Kernel(KernelError::Livelock { at: 0.0 }));
        assert_eq!(k.evaluator().seeded, None, "a rolled-back seed must undo the hook's own mutation too");
        assert_eq!(k.driver(d0).committed(), None, "the oscillator's own doomed commits must roll back too");
    }
    // ---- driver_changed and the request short-circuit ---------------------

    struct ChangeLog {
        changed: Vec<DriverId>,
    }
    impl DeviceEvaluator for ChangeLog {
        fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
            Vec::new()
        }
        fn driver_changed(&mut self, driver: DriverId) {
            self.changed.push(driver);
        }
    }

    #[test]
    fn the_evaluator_is_told_of_exactly_the_committed_moves_and_resets() {
        let mut k = Kernel::new(NoOpSolver, ChangeLog { changed: Vec::new() });
        let d0 = k.add_driver(driver(1), Some(PartitionId(0)));
        let d1 = k.add_driver(driver(2), Some(PartitionId(1)));
        let level = |driver: DriverId, level: Mode, delay: f64| DriverRequest::Level { driver, level, delays: DelaySet::uniform(delay) };

        // A request alone moves nothing that is committed.
        k.apply_request(level(d0, High, 5.0), 0.0, false);
        assert!(k.evaluator().changed.is_empty(), "a request is not a commit");
        // The commit that lands is one.
        k.advance(5.0).unwrap();
        assert_eq!(k.evaluator().changed, vec![d0]);

        // A superseded commit is dropped, and reports nothing: d1's HIGH is
        // cancelled by a reset of a driver that never committed anything.
        k.apply_request(level(d1, High, 5.0), 5.0, false);
        assert_eq!(k.apply_request(DriverRequest::Reset { driver: d1 }, 5.0, false), None, "a never-committed driver's reset marks nothing");
        k.advance(20.0).unwrap();
        assert_eq!(k.evaluator().changed, vec![d0], "neither the reset of a never-committed driver nor the stale commit it left is a change");

        // A reset of a committed driver is one.
        assert_eq!(k.apply_request(DriverRequest::Reset { driver: d0 }, 20.0, false), Some(PartitionId(0)));
        assert_eq!(k.evaluator().changed, vec![d0, d0]);
    }

    #[test]
    fn a_request_that_changes_nothing_leaves_no_journal_entry_and_no_event() {
        let mut k = Kernel::new(NoOpSolver, NoOpEvaluator);
        let d0 = k.add_driver(driver(1), Some(PartitionId(0)));
        let level = |level: Mode| DriverRequest::Level { driver: d0, level, delays: DelaySet::uniform(5.0) };
        k.apply_request(level(High), 0.0, false);
        k.advance(5.0).unwrap();
        assert_eq!(k.driver(d0).committed(), Some(High));

        let window = k.open_window();
        let events = k.heap.len();
        // Already committed HIGH with nothing in flight: inert.
        k.apply_request(level(High), 5.0, false);
        assert!(k.driver_journal.is_empty(), "an inert request saves no pre-image");
        assert_eq!(k.heap.len(), events);

        // A real change is journaled, scheduled, and inert on the second ask.
        k.apply_request(level(Low), 5.0, false);
        assert_eq!(k.driver_journal.len(), 1);
        assert_eq!(k.heap.len(), events + 1);
        k.apply_request(level(Low), 5.0, false);
        assert_eq!((k.driver_journal.len(), k.heap.len()), (1, events + 1), "asking again for the target already in flight is inert too");

        // Rolled back, the driver is exactly as it was.
        k.roll_back(window);
        assert_eq!(k.driver(d0).committed(), Some(High));
        assert_eq!(k.driver(d0).pending(), None);
        assert_eq!(k.heap.len(), events);
    }

    // ---- one request per Level driver in a batch --------------------------

    #[test]
    fn a_driver_named_by_a_level_must_be_named_by_no_other_level_or_reset_in_its_batch() {
        let (a, b) = (DriverId(0), DriverId(1));
        let level = |driver: DriverId, level: Mode| DriverRequest::Level { driver, level, delays: DelaySet::uniform(0.0) };
        let reset = |driver: DriverId| DriverRequest::Reset { driver };
        let timer = DriverRequest::Timer { device: DeviceId(0), at: 1.0, tag: 0 };

        assert!(one_request_per_level_driver(&[]));
        assert!(one_request_per_level_driver(&[level(a, High), level(b, Low), timer.clone(), timer.clone()]), "distinct drivers, any number of timers");
        assert!(one_request_per_level_driver(&[reset(a), reset(a), level(b, High)]), "two resets of one driver are the same reset");
        assert!(!one_request_per_level_driver(&[level(a, High), level(a, High)]), "two levels to one driver, even equal ones");
        assert!(!one_request_per_level_driver(&[level(a, High), level(b, Low), level(a, Low)]), "not adjacent");
        assert!(!one_request_per_level_driver(&[reset(a), level(a, High)]), "a level after a reset is judged against a cleared driver");
        assert!(!one_request_per_level_driver(&[level(a, High), reset(a)]), "and a reset after a level is a second request for it");
    }

    /// A solver whose every dirty partition affects one reader, so a commit
    /// reaches that reader's evaluation.
    struct AffectsOneReader(DeviceId);
    impl PartitionSolver for AffectsOneReader {
        fn resolve_dirty(&mut self, dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
            Ok(if dirty.is_empty() { Vec::new() } else { vec![self.0] })
        }
        fn voltages(&self) -> &[f64] {
            &[]
        }
        fn currents(&self) -> &[f64] {
            &[]
        }
        fn restore(&mut self, _snapshot: (Vec<f64>, Vec<f64>)) {}
    }

    /// An evaluator that returns `batch` for every evaluation.
    struct FixedBatch(Vec<DriverRequest>);
    impl DeviceEvaluator for FixedBatch {
        fn evaluate(&mut self, _reader: DeviceId, _now: f64, _acting_on_edges: bool, _voltages: &[f64]) -> Vec<DriverRequest> {
            self.0.clone()
        }
    }

    /// Runs one instant at which the driver's commit lands and the reader
    /// evaluates to `batch`.
    fn run_one_evaluation_of(batch: Vec<DriverRequest>) -> Result<(), KernelError> {
        let d0 = DriverId(0);
        let mut k = Kernel::new(AffectsOneReader(DeviceId(0)), FixedBatch(batch));
        assert_eq!(k.add_driver(driver(1), Some(PartitionId(0))), d0);
        k.apply_request(DriverRequest::Level { driver: d0, level: High, delays: DelaySet::uniform(0.0) }, 0.0, false);
        k.advance(0.0)
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "named by a Level and by another Level or Reset")]
    fn an_evaluator_that_returns_two_levels_for_one_driver_trips_the_debug_assertion() {
        let level = |level: Mode| DriverRequest::Level { driver: DriverId(0), level, delays: DelaySet::uniform(0.0) };
        // Each is a perfectly good request alone; together `evaluate_into`'s
        // inert-request filter could no longer agree with the kernel's own.
        let _ = run_one_evaluation_of(vec![level(Low), level(High)]);
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "named by a Level and by another Level or Reset")]
    fn an_evaluator_that_resets_a_driver_it_also_requests_a_level_of_trips_the_debug_assertion() {
        let d0 = DriverId(0);
        let _ = run_one_evaluation_of(vec![DriverRequest::Reset { driver: d0 }, DriverRequest::Level { driver: d0, level: High, delays: DelaySet::uniform(0.0) }]);
    }

    #[test]
    fn a_batch_with_one_request_per_driver_is_applied_without_tripping_the_assertion() {
        // The same mock and instant as the two tests above, with a batch that
        // meets the condition: proves their panics come from the batch and
        // not from the harness.
        run_one_evaluation_of(vec![DriverRequest::Level { driver: DriverId(0), level: Low, delays: DelaySet::uniform(5.0) }, DriverRequest::Timer { device: DeviceId(0), at: 9.0, tag: 1 }]).unwrap();
    }

    // ---- the no-progress guard (NP1, ruling R6 (a)) ---------------------

    /// A partition whose every crossing schedules the next one `step` later,
    /// except every `jump_every`-th, which lands `jump` later; its
    /// `fail_at`-th crossing fails once (an instant that rolls back). `done`
    /// counts the crossings that succeeded. With `wave`, it publishes
    /// `wave(done)` as its two node voltages after each; with `rail`, its
    /// events are `Rail` events and node 0 is the rail.
    struct Crawler {
        step: f64,
        jump: f64,
        jump_every: u64,
        fail_at: u64,
        calls: u64,
        done: u64,
        scheduled: Vec<(f64, EventKind)>,
        wave: Option<fn(u64) -> [f64; 2]>,
        v: Vec<f64>,
        rail: bool,
        /// Crawl as `Slow` events, each a re-anchor (`Some(true)`) or a
        /// fired event (`Some(false)`).
        slow: Option<bool>,
    }
    thread_local! {
        /// Every crawl instant on this thread: R86's tests read the host's
        /// wall time off it, so many milliseconds an instant.
        static CRAWLED: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
        /// Host steps a crawler's solver was told of (`begin_host_step`).
        static HOST_STEPS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    }
    fn crawled() -> u64 {
        CRAWLED.with(|c| c.get())
    }
    impl Crawler {
        fn crawl(&mut self, now: f64) -> Result<Vec<DeviceId>, String> {
            CRAWLED.with(|c| c.set(c.get() + 1));
            self.calls += 1;
            if self.calls == self.fail_at {
                return Err("the failing instant".to_string());
            }
            self.done += 1;
            if let Some(wave) = self.wave {
                self.v = wave(self.done).to_vec();
            }
            let dt = if self.done % self.jump_every == 0 { self.jump } else { self.step };
            let kind = if let Some(anchor) = self.slow {
                EventKind::Slow { device: DeviceId(0), token: if anchor { crate::events::SLOW_ANCHOR } else { 0 } }
            } else if self.rail {
                EventKind::Rail { version: 0 }
            } else {
                EventKind::Crossing { partition: PartitionId(0), version: 0 }
            };
            self.scheduled.push((now + dt, kind));
            Ok(Vec::new())
        }
    }
    impl PartitionSolver for Crawler {
        fn resolve_dirty(&mut self, _dirty: &[PartitionId], _now: f64, _drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
            Ok(Vec::new())
        }
        fn voltages(&self) -> &[f64] {
            &self.v
        }
        fn currents(&self) -> &[f64] {
            &[]
        }
        fn restore(&mut self, snapshot: (Vec<f64>, Vec<f64>)) {
            self.v = snapshot.0;
        }
        fn is_current(&self, _partition: PartitionId, _is_crossing: bool, _version: Version) -> bool {
            true
        }
        fn handle_dynamic_event(&mut self, _partition: PartitionId, now: f64, _is_crossing: bool) -> Result<Vec<DeviceId>, String> {
            self.crawl(now)
        }
        fn rail_current(&self, _version: Version) -> bool {
            self.rail
        }
        fn rail_event(&mut self, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
            self.crawl(now).map(|readers| (Vec::new(), readers))
        }
        fn rail_node(&self) -> Option<usize> {
            self.rail.then_some(0)
        }
        fn take_scheduled(&mut self) -> Vec<(f64, EventKind)> {
            std::mem::take(&mut self.scheduled)
        }
        fn param_value(&self, _device_index: usize, _param_index: usize) -> Option<f64> {
            Some(0.0)
        }
        fn slow_is_current(&self, _device: DeviceId, _token: u32) -> bool {
            self.slow.is_some()
        }
        fn handle_slow_event(&mut self, _device: DeviceId, now: f64) -> Result<Vec<PartitionId>, String> {
            self.crawl(now).map(|_| Vec::new())
        }
        fn handle_slow_anchor(&mut self, _device: DeviceId, now: f64) -> Result<Vec<PartitionId>, String> {
            self.crawl(now).map(|_| Vec::new())
        }
        fn begin_host_step(&mut self, _t_start: f64, _t_end: f64) {
            HOST_STEPS.with(|c| c.set(c.get() + 1));
        }
    }

    /// Far enough for any of these crawls to stop by itself if the guard
    /// never fires (200,000 crossings at 5 fs), so a broken guard fails the
    /// test instead of hanging it.
    const T_END: f64 = 1e-3 + 1e-9;

    fn crawler(step: f64, jump: f64, jump_every: u64, fail_at: u64) -> Kernel<Crawler, NoOpEvaluator> {
        let mut k = Kernel::new(Crawler { step, jump, jump_every, fail_at, calls: 0, done: 0, scheduled: Vec::new(), wave: None, v: Vec::new(), rail: false, slow: None }, NoOpEvaluator);
        assert!(k.schedule_crossing(PartitionId(0), 0, 1e-3));
        k
    }

    /// A crawl of 5 fs instants publishing `wave`, as `Rail` events if
    /// `rail`.
    fn waver(wave: fn(u64) -> [f64; 2], rail: bool) -> Kernel<Crawler, NoOpEvaluator> {
        let mut k = Kernel::new(Crawler { step: 5e-15, jump: 5e-15, jump_every: u64::MAX, fail_at: 0, calls: 0, done: 0, scheduled: Vec::new(), wave: Some(wave), v: vec![0.0, 0.0], rail, slow: None }, NoOpEvaluator);
        if rail {
            k.heap.push(1e-3, EventKind::Rail { version: 0 });
        } else {
            assert!(k.schedule_crossing(PartitionId(0), 0, 1e-3));
        }
        k
    }

    // ---- R86: the yielding wall budget -------------------------------------

    /// A fake host clock: each call reads `step_ms` later than the last.
    fn fake_clock(step_ms: f64) -> impl FnMut() -> f64 {
        let mut t = 0.0;
        move || {
            t += step_ms;
            t
        }
    }

    /// A crawl of instants `step` apart, each moving both nodes on the same
    /// way: progress, so the no-progress guard leaves it running.
    fn progressing(step: f64) -> Kernel<Crawler, NoOpEvaluator> {
        let monotone = |n: u64| [n as f64 * 1e-6, -(n as f64) * 1e-3];
        let mut k = Kernel::new(Crawler { step, jump: step, jump_every: u64::MAX, fail_at: 0, calls: 0, done: 0, scheduled: Vec::new(), wave: Some(monotone), v: vec![0.0, 0.0], rail: false, slow: None }, NoOpEvaluator);
        assert!(k.schedule_crossing(PartitionId(0), 0, step));
        k
    }

    #[test]
    fn a_budgeted_advance_yields_between_instants_and_resumes_its_host_step_exactly() {
        // R86 (a): a 10 ms budget on a clock that reads 1 ms per check, one
        // check an instant, yields after about ten instants, at an instant's
        // boundary (the time is the last instant's, no horizon clamp), and
        // the run, resumed to the same `t_end`, ends where the unsplit run
        // does, instant for instant.
        let t_end = 2e-6;
        let mut plain = progressing(1e-9);
        plain.advance(t_end).unwrap();
        let mut k = progressing(1e-9);
        let mut clock = fake_clock(1.0);
        let mut yields = 0;
        loop {
            match k.advance_budgeted(t_end, Some(Budget { clock: &mut clock, budget_ms: 10.0, check_every: 1 })).unwrap() {
                Advanced::Reached => break,
                Advanced::Yielded => {
                    yields += 1;
                    assert!(k.time() < t_end && k.time() == k.last_instant, "yielded at {} off an instant {}", k.time(), k.last_instant);
                }
            }
        }
        assert!(yields >= 100, "{yields} yields");
        assert_eq!(k.time().to_bits(), plain.time().to_bits());
        assert_eq!((k.solver.done, k.stats().events_processed), (plain.solver.done, plain.stats().events_processed));
        assert_eq!(k.solver.v.iter().map(|v| v.to_bits()).collect::<Vec<_>>(), plain.solver.v.iter().map(|v| v.to_bits()).collect::<Vec<_>>());
    }

    #[test]
    fn a_host_step_past_its_event_cap_declines_with_minus_7() {
        // R86 (d), with the budget off as without the clock import (R86
        // addition (a)): re-anchor instants count neither way for the
        // no-progress guard, so a crawl of them runs until the host step's
        // event cap stops it. The cap is 20M events; 1,000 here.
        assert_eq!(HOST_STEP_EVENT_CAP, 20_000_000);
        let mut k = Kernel::new(Crawler { step: 5e-15, jump: 5e-15, jump_every: u64::MAX, fail_at: 0, calls: 0, done: 0, scheduled: Vec::new(), wave: None, v: vec![0.0], rail: false, slow: Some(true) }, NoOpEvaluator);
        k.heap.push(1e-3, EventKind::Slow { device: DeviceId(0), token: crate::events::SLOW_ANCHOR });
        k.event_cap = 1_000;
        let before = k.time();
        match k.advance(T_END).unwrap_err() {
            KernelError::NoProgress { message, .. } => assert!(message.contains("1000 events"), "{message}"),
            e => panic!("{e:?}"),
        }
        assert_eq!(k.solver.done, 1_000, "the cap stops the host step before the instant past it runs");
        assert!(k.time() > before && k.time() < T_END);
        // Asking again declines again: the count is the host step's.
        assert!(matches!(k.advance(T_END), Err(KernelError::NoProgress { .. })));
    }

    /// A budget of `ms` on a clock reading `per_instant` ms for every crawl
    /// instant so far, plus whatever `extra` adds at that count.
    fn instant_clock(per_instant: f64, extra: fn(u64) -> f64) -> impl FnMut() -> f64 {
        let base = crawled();
        move || {
            let n = crawled() - base;
            n as f64 * per_instant + extra(n)
        }
    }

    #[test]
    fn short_host_steps_of_slow_instants_still_meet_the_progress_floor() {
        // R86V finding 1: 60-instant host steps at 250 ms an instant never
        // reached the 64-instant check, and the floor was judged nowhere
        // else; 1e-9 s an instant is 4e-9 s/s. The floor is judged at each
        // call's end read too: the run declines after 30 s of busy time,
        // about 120 instants.
        let mut k = progressing(1e-9);
        let mut clock = instant_clock(250.0, |_| 0.0);
        let mut t = 0.0;
        let err = loop {
            t += 60e-9;
            match k.advance_budgeted(t, Some(Budget { clock: &mut clock, budget_ms: 16.0, check_every: YIELD_CHECK_EVERY })) {
                Ok(_) => assert!(k.solver.done < 1_000, "ran {} instants", k.solver.done),
                Err(e) => break e,
            }
        };
        assert!(matches!(err, KernelError::NoProgress { .. }), "{err:?}");
        assert!(k.solver.done <= 130, "declined after {} instants", k.solver.done);
    }

    #[test]
    fn a_slow_instant_shrinks_the_check_stride_so_a_call_overruns_its_budget_by_an_instant_at_most() {
        // R86V finding 1: at 100 ms an instant a 16 ms budget yielded only at
        // the 64th instant, 6.3 s late. The stride shrinks to what the
        // measured time an instant allows, so every call yields after one
        // instant.
        let mut k = progressing(1e-4);
        let mut clock = instant_clock(100.0, |_| 0.0);
        for _ in 0..20 {
            let before = k.solver.done;
            assert_eq!(k.advance_budgeted(1.0, Some(Budget { clock: &mut clock, budget_ms: 16.0, check_every: YIELD_CHECK_EVERY })).unwrap(), Advanced::Yielded);
            assert_eq!(k.solver.done - before, 1, "a call ran {} instants", k.solver.done - before);
        }
    }

    #[test]
    fn a_host_edit_does_not_restart_the_progress_window() {
        // R86V finding 2: a crawl edited every 10 s of busy time never
        // declined (600 s busy, 60 edits). An edit moves no simulated time:
        // the window stands, and the crawl declines after 30 s.
        let mut k = progressing(1e-9);
        let mut clock = instant_clock(1.0, |_| 0.0);
        let mut next_edit = 10_000.0;
        let err = loop {
            match k.advance_budgeted(1.0, Some(Budget { clock: &mut clock, budget_ms: 16.0, check_every: YIELD_CHECK_EVERY })) {
                Ok(_) => {}
                Err(e) => break e,
            }
            if k.busy_ms >= next_edit {
                k.apply_params_as_event(&[(0, 0, 0.0)]).unwrap();
                next_edit += 10_000.0;
            }
            assert!(k.busy_ms < 60_000.0, "still running after {} ms busy", k.busy_ms);
        };
        assert!(matches!(err, KernelError::NoProgress { .. }), "{err:?}");
        assert!(k.busy_ms <= 31_000.0, "declined after {} ms busy", k.busy_ms);
    }

    #[test]
    fn a_forward_clock_jump_counts_as_a_pause_not_work() {
        // R86V finding 3: one +600 s jump (a suspend) declined a 0.1 s/s
        // board. Each read adds at most BUSY_READ_CAP_MS (or 4 budgets), so
        // the jump is a pause and the board runs to its end.
        let mut k = progressing(1e-4);
        let mut clock = instant_clock(1.0, |n| if n >= 5_000 { 600_000.0 } else { 0.0 });
        while k.advance_budgeted(10.0, Some(Budget { clock: &mut clock, budget_ms: 16.0, check_every: YIELD_CHECK_EVERY })).unwrap() == Advanced::Yielded {}
        assert_eq!(k.time(), 10.0);
    }

    #[test]
    fn a_clock_that_steps_back_re_anchors_and_the_call_still_yields() {
        // R86V finding 4: a clock that went back left busy at 0 and the call
        // ran 999,999 instants without yielding. A negative step re-anchors
        // the call's reads, so it yields one budget after it.
        let mut k = progressing(1e-9);
        let mut clock = instant_clock(1.0, |n| if n >= 100 { -1e9 } else { 0.0 });
        assert_eq!(k.advance_budgeted(1.0, Some(Budget { clock: &mut clock, budget_ms: 200.0, check_every: YIELD_CHECK_EVERY })).unwrap(), Advanced::Yielded);
        assert!(k.solver.done <= 400, "yielded after {} instants", k.solver.done);
        assert!(k.busy_ms > 100.0, "busy {} ms: the reads after the step back count", k.busy_ms);
    }

    #[test]
    fn a_yield_that_lands_on_t_end_resumes_the_same_host_step() {
        // R86V finding 6: a yield right after an instant at `t_end` started
        // a new host step on resume (its event count from zero), since the
        // resume tested `time < t_end`. The host step is resumed by being
        // open for this `t_end`.
        let step = 2f64.powi(-30);
        let t_end = 60.0 * step;
        let mut k = progressing(step);
        let mut clock = instant_clock(1.0, |_| 0.0);
        let mut yielded_on_t_end = false;
        let begun = HOST_STEPS.with(|c| c.get());
        loop {
            match k.advance_budgeted(t_end, Some(Budget { clock: &mut clock, budget_ms: 60.0, check_every: 1 })).unwrap() {
                Advanced::Reached => break,
                Advanced::Yielded => yielded_on_t_end |= k.time() == t_end,
            }
        }
        assert!(yielded_on_t_end, "the test must yield on t_end");
        assert_eq!(HOST_STEPS.with(|c| c.get()) - begun, 1, "one host step, resumed after each yield");
        // The resume after a yield on `t_end` continues the host step's event
        // count: a plain advance (no clock, so the cap applies) to the same
        // `t_end` after that yield counts the 60 events the budgeted calls
        // took, past a cap of 50.
        let mut k = progressing(step);
        k.event_cap = 50;
        let mut clock = instant_clock(1.0, |_| 0.0);
        while k.time() < t_end {
            assert_eq!(k.advance_budgeted(t_end, Some(Budget { clock: &mut clock, budget_ms: 60.0, check_every: 1 })).unwrap(), Advanced::Yielded);
        }
        assert!(matches!(k.advance(t_end), Err(KernelError::NoProgress { .. })), "the host step's count restarted");
    }

    #[test]
    fn a_failure_reset_or_a_memory_seed_between_yields_ends_the_host_step_as_an_edit_does() {
        // R86V finding 5: every host input between two yields (set_params,
        // seed_memory, set_option, reset_failures) ends the host step, so
        // the next advance to the same `t_end` begins a new one.
        for input in ["reset", "seed", "edit"] {
            let mut k = progressing(1e-9);
            let mut clock = instant_clock(1.0, |_| 0.0);
            let begun = HOST_STEPS.with(|c| c.get());
            assert_eq!(k.advance_budgeted(1e-6, Some(Budget { clock: &mut clock, budget_ms: 50.0, check_every: 1 })).unwrap(), Advanced::Yielded);
            match input {
                "reset" => k.reset_failures_as_event().unwrap(),
                "seed" => assert!(k.seed_memory(DeviceId(0), &[]).is_err(), "the no-op evaluator refuses the seed"),
                _ => k.apply_params_as_event(&[(0, 0, 0.0)]).unwrap(),
            }
            assert!(k.host_step.is_none(), "{input} ended the host step");
            k.advance_budgeted(1e-6, Some(Budget { clock: &mut clock, budget_ms: 50.0, check_every: 1 })).unwrap();
            assert_eq!(HOST_STEPS.with(|c| c.get()) - begun, 2, "{input}: a new host step began");
        }
    }

    #[test]
    fn with_a_clock_only_the_floor_governs_and_without_one_the_cap_scales_with_the_span() {
        // Ruling R94 (a): a budgeted call with a clock has no event cap (a
        // legitimate 8.25M-event-a-second board ran into 20M at 0.22 s/s);
        // without a clock the cap is 20M x max(1, the host step's span in
        // simulated seconds). Re-anchor instants count neither way for the
        // no-progress guard, so only the cap or the floor can stop them.
        assert_eq!(HOST_STEP_EVENT_CAP, 20_000_000);
        let anchors = |step: f64| {
            let mut k = Kernel::new(Crawler { step, jump: step, jump_every: u64::MAX, fail_at: 0, calls: 0, done: 0, scheduled: Vec::new(), wave: None, v: vec![0.0], rail: false, slow: Some(true) }, NoOpEvaluator);
            k.heap.push(step, EventKind::Slow { device: DeviceId(0), token: crate::events::SLOW_ANCHOR });
            k.event_cap = 1_000;
            k
        };
        // A clock and 0.1 s/s: 5,000 events past a cap of 1,000.
        let mut k = anchors(1e-4);
        let mut clock = instant_clock(1.0, |_| 0.0);
        while k.advance_budgeted(0.5, Some(Budget { clock: &mut clock, budget_ms: 16.0, check_every: YIELD_CHECK_EVERY })).unwrap() == Advanced::Yielded {}
        assert_eq!(k.solver.done, 5_000);
        // No clock: a 3 s host step stops at 3 x 1,000 events, a 0.5 s one at
        // 1,000, the same count on every run.
        for (t_end, cap) in [(3.0, 3_000), (0.5, 1_000), (3.0, 3_000)] {
            let mut k = anchors(1e-4);
            assert!(matches!(k.advance(t_end), Err(KernelError::NoProgress { .. })));
            assert_eq!(k.solver.done, cap, "host step to {t_end} s");
        }
    }

    #[test]
    fn a_budgeted_run_under_the_progress_floor_for_thirty_seconds_declines_and_one_over_it_runs() {
        // R86 (c): 1e-2 s simulated per wall second, over 30 s of the host's
        // time inside budgeted advances. A crawl of 1 ns instants on a clock
        // reading 1 ms per check makes 1e-6 s/s: after 30 s of reads it
        // declines. One of 0.1 ms instants makes 0.1 s/s and runs to its end.
        let mut k = progressing(1e-9);
        let mut clock = fake_clock(1.0);
        let err = loop {
            match k.advance_budgeted(1.0, Some(Budget { clock: &mut clock, budget_ms: 16.0, check_every: 1 })) {
                Ok(Advanced::Yielded) => {}
                Ok(Advanced::Reached) => panic!("ran to 1 s"),
                Err(e) => break e,
            }
        };
        match err {
            KernelError::NoProgress { message, .. } => assert!(message.contains("in 30 s"), "{message}"),
            e => panic!("{e:?}"),
        }
        // 30 s of reads inside the calls; each 16 ms call reads the clock
        // before its first instant too, so 16 reads to 15 instants.
        assert!((k.busy_ms - 30_000.0).abs() <= 1.0, "declined after {} ms", k.busy_ms);
        assert!((k.solver.done as f64 - 30_000.0 * 15.0 / 16.0).abs() <= 16.0, "declined after {} instants", k.solver.done);
        let mut k = progressing(1e-4);
        let mut clock = fake_clock(1.0);
        while k.advance_budgeted(10.0, Some(Budget { clock: &mut clock, budget_ms: 16.0, check_every: 1 })).unwrap() == Advanced::Yielded {}
        assert_eq!(k.time(), 10.0);
    }

    #[test]
    fn instants_that_only_re_anchor_slow_drives_count_neither_way_and_fired_ones_still_do() {
        // W1g ruling R21: a re-anchor moves no node by design. A crawl of
        // them 5 fs apart (far denser than the solver ever schedules them)
        // runs on; the same crawl of slow events that fired and moved
        // nothing is a stall, and declines.
        let slow = |redrove: bool| {
            let mut k = Kernel::new(Crawler { step: 5e-15, jump: 5e-15, jump_every: u64::MAX, fail_at: 0, calls: 0, done: 0, scheduled: Vec::new(), wave: None, v: vec![0.0], rail: false, slow: Some(redrove) }, NoOpEvaluator);
            k.heap.push(1e-3, EventKind::Slow { device: DeviceId(0), token: if redrove { crate::events::SLOW_ANCHOR } else { 0 } });
            k
        };
        let mut k = slow(true);
        k.advance(1e-3 + 5e-10).unwrap();
        assert!(k.solver.done > 9 * NO_PROGRESS_MAX as u64, "{}", k.solver.done);
        let mut k = slow(false);
        assert_eq!(k.advance(T_END).unwrap_err(), KernelError::SolveFailed(crate::rails::loop_unsettled()));
        assert_eq!(k.solver.done, NO_PROGRESS_MAX as u64 + 1);
    }

    #[test]
    fn a_fast_monotone_burst_runs_however_dense_and_a_cycle_of_any_length_declines() {
        // 100,000 instants 5 fs apart, every one moving both nodes on the
        // same way (the fast RC inrush's shape, NP2): progress, so it runs.
        let monotone = |n: u64| [n as f64 * 1e-6, -(n as f64) * 1e-3];
        let mut k = waver(monotone, false);
        k.advance(1e-3 + 5e-10).unwrap();
        assert!(k.solver.done > 9 * NO_PROGRESS_MAX as u64, "{}", k.solver.done);
        // The same crawl as Rail events moving the rail (node 0): still runs.
        let mut k = waver(monotone, true);
        k.advance(1e-3 + 5e-10).unwrap();
        assert!(k.solver.done > 9 * NO_PROGRESS_MAX as u64, "{}", k.solver.done);
        // A three-state cycle (0, 1, 2, 0, ...) turns node 0 back at two of
        // every three instants, and the third carries it on without
        // resetting the count.
        let mut k = waver(|n| [(n % 3) as f64, 0.0], false);
        assert_eq!(k.advance(T_END).unwrap_err(), KernelError::SolveFailed(crate::rails::loop_unsettled()));
        let n = k.solver.done;
        assert!((3 * NO_PROGRESS_MAX as u64 / 2..=3 * NO_PROGRESS_MAX as u64 / 2 + 3).contains(&n), "{n} instants");
        // B1070's shape: Rail events that leave the rail still while
        // another node creeps on its own trajectory.
        let mut k = waver(|n| [4.99, n as f64 * 1e-11], true);
        assert_eq!(k.advance(T_END).unwrap_err(), KernelError::SolveFailed(crate::rails::loop_unsettled()));
        assert_eq!(k.solver.done, NO_PROGRESS_MAX as u64 + 1);
        // The same creep from a crossing is a trajectory's own progress.
        let mut k = waver(|n| [4.99, n as f64 * 1e-11], false);
        k.advance(1e-3 + 5e-10).unwrap();
        assert!(k.solver.done > 9 * NO_PROGRESS_MAX as u64, "{}", k.solver.done);
    }

    #[test]
    fn a_node_at_rest_in_its_rounding_beside_a_moving_rail_is_progress_and_a_cycle_inside_the_floor_still_declines() {
        // G2bv's seed 400 (FXF): a CC PSU ramping a series capacitor into a
        // 0.26 ohm short moves the rail 1e-5 V a Rail event, 0.08 ns apart,
        // while the short's node, set by the limit alone, sits at 0.0833 V
        // and its published value turns by 1e-15 to 4e-15 V of rounding at
        // most instants: 10,000 such turns declined it at 1.16 us.
        let jitter = |n: u64| [n as f64 * 1e-5, 0.0833 + if n % 2 == 0 { 2e-15 } else { -2e-15 }];
        let mut k = waver(jitter, true);
        k.advance(1e-3 + 5e-10).unwrap();
        assert!(k.solver.done > 9 * NO_PROGRESS_MAX as u64, "{}", k.solver.done);
        // The same rounding with nothing moving on is no progress: a cycle
        // within the floor moves nothing, so it counts every instant.
        let mut k = waver(|n| [4.99 + if n % 2 == 0 { 2e-15 } else { -2e-15 }, 0.0833], false);
        assert_eq!(k.advance(T_END).unwrap_err(), KernelError::SolveFailed(crate::rails::loop_unsettled()));
        assert_eq!(k.solver.done, NO_PROGRESS_MAX as u64 + 1);
        // And a rail that moves only by its rounding is a rail held still
        // (B1070's shape), whatever creeps beside it.
        let mut k = waver(|n| [4.99 + if n % 2 == 0 { 2e-15 } else { -2e-15 }, n as f64 * 1e-11], true);
        assert_eq!(k.advance(T_END).unwrap_err(), KernelError::SolveFailed(crate::rails::loop_unsettled()));
        assert_eq!(k.solver.done, NO_PROGRESS_MAX as u64 + 1);
    }

    #[test]
    fn a_crawl_of_femtosecond_instants_declines_with_the_guards_minus_4_and_keeps_the_last_instant() {
        // 5 fs a crossing clears the 1e-15 s floor every time, as the rail
        // ping-pong's crossings did before W1c point 1a held a starving IC
        // off (F5): only the guard ends such a crawl. The first instant
        // follows nothing, so NO_PROGRESS_MAX + 1 complete and the next
        // declines.
        let mut k = crawler(5e-15, 5e-15, u64::MAX, 0);
        let err = k.advance(T_END).unwrap_err();
        assert_eq!(err, KernelError::SolveFailed(crate::rails::loop_unsettled()));
        let n = NO_PROGRESS_MAX as u64 + 1;
        assert_eq!(k.solver.done, n);
        assert!((k.time() - (1e-3 + (n - 1) as f64 * 5e-15)).abs() < 1e-14, "{}", k.time());
        assert!(k.next_event_time() > k.time(), "the declined instant is still pending");
        // Asking again declines at once and changes nothing.
        let (t, events) = (k.time(), k.stats().events_processed);
        assert_eq!(k.advance(T_END).unwrap_err(), KernelError::SolveFailed(crate::rails::loop_unsettled()));
        assert_eq!((k.time(), k.stats().events_processed, k.solver.done), (t, events, n));
        // A host's edit starts a new run: the crawl goes on until it has
        // stalled the full count again.
        k.apply_params_as_event(&[(0, 0, 1.0)]).unwrap();
        assert_eq!(k.advance(T_END).unwrap_err(), KernelError::SolveFailed(crate::rails::loop_unsettled()));
        assert_eq!(k.solver.done, 2 * n - 1);
    }

    #[test]
    fn an_instant_that_rolls_back_neither_counts_nor_resets_the_guard() {
        // The 500th crossing fails once and rolls back; asked again, it runs.
        // The guard still declines after exactly NO_PROGRESS_MAX + 1
        // completed instants: a reset would let more through, a count of the
        // failed one fewer.
        let mut k = crawler(5e-15, 5e-15, u64::MAX, 500);
        let err = k.advance(T_END).unwrap_err();
        assert_eq!(err, KernelError::SolveFailed("the failing instant".to_string()));
        assert_eq!(k.solver.done, 499);
        let err = k.advance(T_END).unwrap_err();
        assert_eq!(err, KernelError::SolveFailed(crate::rails::loop_unsettled()));
        assert_eq!(k.solver.done, NO_PROGRESS_MAX as u64 + 1);
    }

    #[test]
    fn a_dense_burst_that_moves_on_runs_to_the_end_and_a_burst_cut_short_by_a_step_never_accumulates() {
        // Runs of NO_PROGRESS_MAX instants 1 fs apart (stalls, all of them
        // but the first: nothing moves), each ended by a step past the
        // window: never more than the cap in a run, so it runs to t_end
        // however many there are.
        let jump = 2.0 * NO_PROGRESS_DT;
        let mut k = crawler(1e-15, jump, NO_PROGRESS_MAX as u64, 0);
        k.advance(1e-3 + 3.5 * jump).unwrap();
        assert_eq!(k.solver.done, 4 * NO_PROGRESS_MAX as u64);
        // One instant more in the run and it declines.
        let mut k = crawler(1e-15, jump, NO_PROGRESS_MAX as u64 + 2, 0);
        assert_eq!(k.advance(T_END + 1e-8).unwrap_err(), KernelError::SolveFailed(crate::rails::loop_unsettled()));
        assert_eq!(k.solver.done, NO_PROGRESS_MAX as u64 + 1);
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Advanced {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Advanced::Reached => 0,
            Advanced::Yielded => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for HostStep {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t_end, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.events_at_start, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.span, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for KernelError {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            KernelError::Livelock { at } => crate::snapshot::HeapBytes::heap_bytes(at, w),
            KernelError::SolveFailed(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            KernelError::NoProgress { at, message } => crate::snapshot::HeapBytes::heap_bytes(at, w) + crate::snapshot::HeapBytes::heap_bytes(message, w),
        }
    }
}

impl crate::snapshot::HeapBytes for SeedMemoryError {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            SeedMemoryError::Rejected(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            SeedMemoryError::Kernel(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for KernelStats {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.events_processed, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.delta_passes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.max_delta_depth, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.driver_commits, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DriverRequest {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            DriverRequest::Level { driver, level, delays } => crate::snapshot::HeapBytes::heap_bytes(driver, w) + crate::snapshot::HeapBytes::heap_bytes(level, w) + crate::snapshot::HeapBytes::heap_bytes(delays, w),
            DriverRequest::Reset { driver } => crate::snapshot::HeapBytes::heap_bytes(driver, w),
            DriverRequest::Timer { device, at, tag } => crate::snapshot::HeapBytes::heap_bytes(device, w) + crate::snapshot::HeapBytes::heap_bytes(at, w) + crate::snapshot::HeapBytes::heap_bytes(tag, w),
        }
    }
}
