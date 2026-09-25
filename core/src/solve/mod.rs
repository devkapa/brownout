//! Partition solvers. W1a defined only the partition-solver trait (worker
//! A1) plus the first real implementation, `StaticPartitionSolver`, adapting
//! the W0 static kernel. W1b (row B1) moves that implementation to
//! `static_part`, adds the switch family's fresh-every-resolve stamping and
//! the solution cache, and gives this trait the `bo_engine_set_params`
//! (owner decision 1) and cache-control hooks every default-implemented
//! method below exists for — a solver with no mutable device state or no
//! cache (a future closed-form/stepped solver, or a test mock) can ignore
//! them entirely.

pub mod static_part;

use crate::drivers::Driver;
use crate::events::{DeviceId, PartitionId};

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

    /// Every node's currently published voltage, node 0 = 0
    /// (`bo_engine_node_voltages`'s contract). Whatever the solve method
    /// (static, closed-form, stepped), this is the one array `Engine`
    /// publishes through the ABI, so it lives on the trait rather than only
    /// on a concrete implementation — `engine.rs` reads it through
    /// `Kernel::solver` without needing to know which implementation is
    /// behind it.
    fn voltages(&self) -> &[f64];

    /// Every device's currently published current, in record order
    /// (`bo_engine_device_currents`'s contract).
    fn currents(&self) -> &[f64];

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
    /// concrete solver's private storage to write them back.
    fn snapshot(&self) -> (Vec<f64>, Vec<f64>) {
        (self.voltages().to_vec(), self.currents().to_vec())
    }

    /// Restores exactly what a prior [`Self::snapshot`] call captured.
    fn restore(&mut self, snapshot: (Vec<f64>, Vec<f64>));

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

    fn voltages(&self) -> &[f64] {
        (**self).voltages()
    }

    fn currents(&self) -> &[f64] {
        (**self).currents()
    }

    fn restore(&mut self, snapshot: (Vec<f64>, Vec<f64>)) {
        (**self).restore(snapshot)
    }

    fn set_no_solution_cache(&mut self, off: bool) {
        (**self).set_no_solution_cache(off)
    }

    fn cache_hits(&self) -> f64 {
        (**self).cache_hits()
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
}
