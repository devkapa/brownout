//! The stepped dynamic-partition solver (W1 plan row B3): a multi-capacitor
//! or (once W1c+ adds one) nonlinear partition, integrated by backward-Euler
//! step doubling with a trapezoidal option — pec3.mjs's `_planStep`/
//! `_commitStep`/`_syncDynamic` (`:643-703`), generalized from pec3's
//! BE-only prototype to also support trap (spec §7.2.3/§7.4), since pec3
//! itself never implements trap at all (verified: no occurrence of "trap" in
//! `pec3.mjs`).
//!
//! **Per-step solve.** Each attempt at a step of size `h` replaces every
//! capacitor with its BE or trap companion (`devices::passives::
//! capacitor_be_companion`/`capacitor_trap_companion`, spec §7.4) and solves
//! the resulting system through [`crate::solve::newton`]. A partition with
//! nothing nonlinear in it converges in exactly one iteration (W1 plan row
//! B3); one holding a free IC supply node (W1c) or a stamped LED junction
//! (W1e) iterates, with no polish, exactly as classic solves each step.
//!
//! **A step whose Newton does not converge is rejected, not fatal** (W1e,
//! README LEDs point 8, as classic's host does, HL:609-617 and 819-831): `h`
//! becomes `max(H_MIN, 0.25 h)` and the step-doubling trio is retried, and
//! only a failure at `h <= 1.01 * H_MIN` is `-4`. A crossing-refinement trial
//! that does not converge ends the refinement at the best `h` found so far.
//! A singular or non-finite pivot stays `-4` at once.
//!
//! **Step-doubling error control** (`plan_step`): one step of `h` (coarse)
//! is compared against two steps of `h/2` (fine); the worse of every node
//! voltage's and every capacitor charge's relative error
//! (`ABS_VOLTAGE`/`REL_TOL`) against `nextStepFactor` decides whether to
//! accept and how `h` should move, using classic's constants throughout
//! (`H_MIN`/`H_MAX`/`ACTIVE_H_MAX`/`QUIET_FRACTION`/`REL_TOL`/`ABS_VOLTAGE`,
//! W1 plan row B3).
//!
//! **The BE anchor at a discontinuity** (spec §7.2.3): each partition tracks
//! its own `be_next_step`, starting `true` (the first step is always BE,
//! classic's `load()`) and set again whenever a stamp in the partition
//! changes (a switch toggle, a driver commit — the same trigger that
//! re-aims a closed-form segment); cleared only after a step is ACCEPTED.
//! While set, the WHOLE step-doubling trio (coarse and both halves) uses BE
//! regardless of the `INTEGRATION` option, matching classic's rule that a
//! step immediately after a discontinuity must not use trap for either half
//! (`adaptive-step.ts`'s replay path: "if the snapshot had BE pending, call
//! `markDiscontinuity()`" before the SECOND half too).
//!
//! **Crossings inside a step are REFINED ITERATIVELY** (spec D11): a linear
//! first estimate of the crossing time (as pec3 does) is followed by
//! bisection on `h` itself, re-taking the real BE/trap step at each trial
//! midpoint, until the crossing node lands within [`CROSS_TOLERANCE_V`] of
//! its threshold or the iteration cap is hit — never pec3's single clamp to
//! `[0.02, 0.98]` (`pec3.mjs:681`), which the spec explicitly calls out as
//! not to be ported.

use std::collections::HashMap;

use crate::devices::{self, passives};
use crate::digital;
use crate::drivers::Driver;
use crate::events::DeviceId;
use crate::partition::PartitionKind;
use crate::program::Program;
use crate::solve::newton;
use crate::solve::{
    build_ic_reader_index, collect_ics, compile_dyn_shape, pin_voltage, stamp_ics_into_dyn, stamp_junctions, stamp_linear_base, stamp_two_terminal,
    DynCapacitor, DynShape, Threshold, ThresholdFn,
};

/// Classic's step-doubling constants (W1 plan row B3; core/README.md).
pub const H_MIN: f64 = 1e-8;
pub const H_MAX: f64 = 1e-2;
pub const ACTIVE_H_MAX: f64 = 2e-3;
pub const QUIET_FRACTION: f64 = 0.1;
pub const REL_TOL: f64 = 1e-3;
pub const ABS_VOLTAGE: f64 = 100e-6;
/// The time a step is first given right after a discontinuity (pec3.mjs
/// `POST_EVENT_H`, `:33`) — small enough that the doubling controller
/// re-discovers the right scale within a couple of passes rather than
/// starting from `H_MAX` and immediately over-stepping a fast transient.
pub const POST_EVENT_H: f64 = 1e-8;
/// How closely a refined crossing must land on its threshold (matches
/// `closed_form::CROSS_NUDGE_V`, so a stepped and a closed-form partition
/// under `FORCE_STEPPED` land a crossing to the same precision).
pub const CROSS_TOLERANCE_V: f64 = 1e-9;
/// The bisection cap for iterative crossing refinement (this module's doc
/// comment; spec D11).
const CROSS_REFINE_MAX_ITERS: u32 = 40;

/// Backward Euler or trapezoidal (core/README.md's `INTEGRATION` option,
/// load-time only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Method {
    Be,
    Trap,
}

impl Method {
    pub fn from_option(integration: u8) -> Self {
        if integration == 1 {
            Method::Trap
        } else {
            Method::Be
        }
    }
}

fn next_step_factor(ratio: f64, method: Method) -> f64 {
    if !ratio.is_finite() {
        return 0.2;
    }
    if ratio <= 1e-12 {
        return 2.0;
    }
    let raw = match method {
        // Order 1 (pec3.mjs `nextStepFactor`, `:41-45`).
        Method::Be => 0.9 / ratio.sqrt(),
        // Order 2 (spec §10.14, classic's trap-mode error controller).
        Method::Trap => 0.9 * ratio.powf(-1.0 / 3.0),
    };
    raw.clamp(0.2, 2.0)
}

/// One step's result: the node voltages and every capacitor's new charge and
/// series current (the latter needed as `iPrev` for a LATER trap step, spec
/// §7.4: "`iPrev` must be the last ACCEPTED series current").
#[derive(Debug, Clone)]
struct StepResult {
    x: Vec<f64>,
    vc: Vec<f64>,
    i: Vec<f64>,
}

/// Everything [`take_step`]'s per-Newton-iterate stamp closure needs to
/// restamp this partition's own IC driver/pull contributions afresh, every
/// time (W1c contract point 3: "a partition holding a free IC supply node
/// re-stamps from the latest iterate") — bundled into one struct rather than
/// threaded as a long, repeated parameter list through `take_step`,
/// `solve_initial_x`, `plan_step`, `refine_crossing_h`, all of which need
/// every field here. `now` is the partition's own current time (`part.t`,
/// unchanged for the whole step-doubling trio a single `plan_step` call
/// tries): the floating-pull bucket key does not need finer resolution than
/// that, and it keeps every sub-stamp (coarse, mid, fine, and any bisection
/// trial `refine_crossing_h` takes) using the identical key, exactly as a
/// single `static_part.rs::resolve_dirty` call uses one `now` throughout its
/// own fixed-point iteration.
struct IcStampCtx<'a> {
    now: f64,
    drivers: &'a [Driver],
    ics: &'a [(usize, digital::IcRecord)],
    node_owner: &'a HashMap<usize, (usize, usize)>,
    fixed: &'a [Option<f64>],
    p_idx: usize,
    /// Whether THIS partition's stamp moves with the very voltages the step
    /// is solving for: it holds a free IC supply node (contract point 3,
    /// R_out follows the iterate) or a stamped LED junction (W1e, whose
    /// companion is re-linearised about it). [`take_step`] then runs a real
    /// Newton iteration (`linear = false`) instead of the one-shot linear
    /// solve every other partition needs.
    iterated: bool,
}

/// Why a step's solve produced no result.
#[derive(Debug)]
enum StepError {
    /// The iteration hit its cap without passing (README, LEDs point 8): the
    /// caller rejects the step and retries it smaller, or ends a refinement,
    /// or fails where there is no smaller step to try.
    NoConverge,
    /// A singular or non-finite pivot or iterate: `-4` at once.
    Failed(String),
}

impl StepError {
    /// The message for a caller with no smaller step to retry.
    fn into_message(self) -> String {
        match self {
            StepError::NoConverge => format!("stepped: Newton did not converge within {} iterations", newton::NEWTON_MAX),
            StepError::Failed(message) => message,
        }
    }
}

/// Solves one BE or trap step of size `h` from `(vc_from, x_from)`, with
/// `i_prev` the last ACCEPTED series current (used only by trap). `ic_ctx`
/// carries whatever this partition's own IC driver/pull stamps need
/// (W1c step 2b) — see [`IcStampCtx`]'s own doc comment.
#[allow(clippy::too_many_arguments)]
fn take_step(
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc_from: &[f64],
    x_from: &[f64],
    i_prev: &[f64],
    method: Method,
    newton_iters: &mut u64,
    ic_ctx: &IcStampCtx,
) -> Result<StepResult, StepError> {
    let n = shape.n;
    // `linear = true` short-circuits `newton::solve` to exactly one
    // iteration (a linear stamp could never move on a second pass) — but an
    // IC's driver/pull conductance on a FREE supply node genuinely depends on
    // the iterate itself (R_out is a function of the very voltage being
    // solved for), and so does an LED junction's companion, so those
    // partitions need the real convergence loop instead. No polish: classic
    // solves each step exactly like this (LEDs point 8).
    let linear = !ic_ctx.iterated;
    let result = newton::solve(n, n, x_from, linear, false, |x, a, b| {
        stamp_linear_base(shape, n, a, b, live_params);
        for (i, cap) in shape.capacitors.iter().enumerate() {
            let contrib = companion(cap, h, vc_from[i], i_prev[i], method);
            stamp_two_terminal(n, a, b, &cap.pins[0], &cap.pins[1], contrib.conductance, contrib.injection);
        }
        stamp_ics_into_dyn(n, a, b, ic_ctx.now, ic_ctx.drivers, ic_ctx.ics, ic_ctx.node_owner, ic_ctx.fixed, ic_ctx.p_idx, x);
        stamp_junctions(&shape.junctions, n, a, b, x);
    })
    .map_err(|e| StepError::Failed(format!("stepped: {e}")))?;
    // Counted before the verdict: a rejected step's iterations were run.
    *newton_iters += result.iterations as u64;
    if !linear && !result.converged {
        // `newton::solve` never turns "hit the iteration cap unconverged"
        // into an `Err`; what that means is the caller's call (retry
        // smaller, end a refinement, or fail).
        return Err(StepError::NoConverge);
    }

    let x = result.x;
    let mut vc = vec![0.0; shape.capacitors.len()];
    let mut i_new = vec![0.0; shape.capacitors.len()];
    for (idx, cap) in shape.capacitors.iter().enumerate() {
        let contrib = companion(cap, h, vc_from[idx], i_prev[idx], method);
        let (va, vb) = (pin_voltage(&cap.pins[0], &x), pin_voltage(&cap.pins[1], &x));
        let i = crate::solve::branch_current(contrib.conductance, contrib.injection, va, vb);
        i_new[idx] = i;
        vc[idx] = match method {
            Method::Be => vc_from[idx] + (h / cap.c) * i,
            Method::Trap => vc_from[idx] + (h / (2.0 * cap.c)) * (i + i_prev[idx]),
        };
    }
    Ok(StepResult { x, vc, i: i_new })
}

fn companion(cap: &DynCapacitor, h: f64, vc_prev: f64, i_prev: f64, method: Method) -> devices::Contribution {
    let params = [cap.c, cap.esr, 0.0];
    match method {
        Method::Be => passives::capacitor_be_companion(&params, h, vc_prev),
        Method::Trap => passives::capacitor_trap_companion(&params, h, vc_prev, i_prev),
    }
}

/// The tiny step size classic's own load-time seed uses (spec §10.2 point 5:
/// "Seed solve `_solve(1e-12)`"; pec3.mjs's own `init()` — `const seedDyn =
/// (part) => { this._beStep(part, 1e-12, part.vc, ...); ... }`, called for
/// every dynamic partition before the real event loop starts). Deliberately
/// NOT a true `h -> 0` ideal-source solve (which would pin an `esr = 0`
/// capacitor's own node to EXACTLY its committed `vc`, giving a chain of
/// capacitors downstream of the first one an exact `0` current at `t = 0`
/// whenever they are all uncharged — the very bug this module fixes, just
/// reintroduced one capacitor later): a real, if minuscule, `h` gives the
/// capacitor a large-but-FINITE companion conductance instead, letting a
/// correspondingly tiny but genuinely nonzero current reach whatever sits
/// beyond it, matching what classic's own identical seed produces.
const SEED_H: f64 = 1e-12;

/// The node voltages AND each capacitor's own instantaneous series current
/// consistent with `vc` HELD FIXED (not evolving), via a single tiny BE step
/// (`SEED_H`'s doc comment) rather than a step the step-doubling controller
/// would ever actually take or commit.
///
/// A stepped [`Partition`]'s own `x` is otherwise only ever produced as a
/// COMMITTED step's result; at load, or right after a re-aim, there is a
/// committed `vc` but no step has been taken YET against the CURRENT stamp,
/// so `x` would otherwise still be whatever a stale placeholder
/// (`Partition::new`'s all-zero start) or the OLD topology's last committed
/// step left it as — exactly the bug this exists to fix: `elementI` for a
/// device in a stepped partition read `0` at `t = START_TIME`, right after
/// `bo_engine_load`, because nothing had ever solved for the actual
/// operating point yet.
///
/// The returned series current is likewise used to seed `i_prev`, NOT left
/// at whatever it was before (`0.0` at load; the OLD topology's last
/// accepted value at a later re-aim): classic's own seed solve
/// (`_solve(1e-12)`, spec §10.2 point 5) runs the FULL post-solve pipeline
/// (spec §10.5), which unconditionally republishes `capCurrents = series +
/// leakage` — spec §7.2.4 "capCurrents = elementI = leakage current" is the
/// DC-mode case; the ordinary path's `commitState` "Updates `capCurrents`"
/// unconditionally too, spec §7.4 — even though a genuinely forced-BE step
/// (`_beNextStep`, which load and a discontinuity both set) skips updating
/// `capsI` itself. `i_prev` feeding a LATER trap companion is harmless here:
/// `be_next_step` is set alongside this seed at every call site, so the very
/// next REAL step is forced to BE regardless, never reading it.
///
/// `x_from` is the partition's COMMITTED `x` (W1e, LEDs point 8: classic
/// warm-starts every solve from the previous accepted one), all zeros at
/// load. It is not a step and has no smaller `h` to retry, so a Newton that
/// does not pass there is `-4`. Its iterations are not counted in stat 11.
fn solve_initial_x(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, vc: &[f64], x_from: &[f64], ic_ctx: &IcStampCtx) -> Result<(Vec<f64>, Vec<f64>), String> {
    let i_prev = vec![0.0; shape.capacitors.len()]; // BE never reads `i_prev`; only trap does.
    let mut newton_iters = 0u64;
    let result = take_step(shape, live_params, SEED_H, vc, x_from, &i_prev, Method::Be, &mut newton_iters, ic_ctx).map_err(StepError::into_message)?;
    Ok((result.x, result.i))
}

struct Partition {
    shape: DynShape,
    /// Committed local node voltages (len `n`) — the authoritative internal
    /// copy; `voltages`/`currents` (the shared, published arrays) are
    /// written from this at every commit/sync, never read back from.
    x: Vec<f64>,
    /// Committed capacitor charge(s), len `shape.capacitors.len()`.
    vc: Vec<f64>,
    /// The last ACCEPTED series current per capacitor (spec §7.4's trap
    /// requirement) — updated only on commit, never from a trial.
    i_prev: Vec<f64>,
    t: f64,
    h: f64,
    changing: bool,
    /// Spec §7.2.3's `_beNextStep`: forces BE for the WHOLE next
    /// step-doubling trio, cleared only after that step is accepted.
    be_next_step: bool,
    version: u64,
    has_pend: bool,
    pend_t: f64,
    pend_x: Vec<f64>,
    pend_vc: Vec<f64>,
    pend_i: Vec<f64>,
    watches: HashMap<usize, Vec<(DeviceId, ThresholdFn)>>,
    /// Contract stats 10/11 (this partition's share). NOT rolled back
    /// (core/README.md, "A failed advance").
    steps_taken: u64,
    newton_iterations: u64,
    /// Whether this partition holds a free IC supply node (W1c contract
    /// point 3) — a build-time topology fact (`SteppedSolver::build`
    /// computes it once, from `collect_ics`/`local_index`), not part of any
    /// snapshot/restore: it can never change after load, since
    /// `partition::build`'s own union rule decides it before this solver
    /// even exists.
    holds_free_ic_supply: bool,
}

impl Partition {
    /// Whether the stamp depends on the iterate (`IcStampCtx::iterated`).
    fn iterated(&self) -> bool {
        self.holds_free_ic_supply || !self.shape.junctions.is_empty()
    }

    /// `t0` is `program.options.start_time` (contract point 5: "A stepped
    /// partition's clock starts at START_TIME, so a load integrates nothing
    /// and costs the same at any start time"). Starting the clock at 0
    /// regardless of `t0` was the bug this fixes: `resolve_dirty`'s first
    /// `sync_partition(p_idx, now, ...)` call horizon-clamps to `now`, which
    /// IS `START_TIME` at load — with `t` hardcoded to 0 that call had to
    /// really integrate the whole `[0, START_TIME]` interval with genuine
    /// BE/trap steps before the circuit was considered loaded at all,
    /// contradicting `vc`'s own "Initial vc = 0 at load" a few lines below
    /// (a capacitor starts uncharged AT `START_TIME`, not at some point on
    /// the way to it) and costing load time linear in `START_TIME`.
    fn new(shape: DynShape, t0: f64) -> Self {
        let n = shape.n;
        let nc = shape.capacitors.len();
        Partition {
            shape,
            x: vec![0.0; n],
            vc: vec![0.0; nc], // README: "Initial vc = 0 at load".
            i_prev: vec![0.0; nc],
            t: t0,
            h: POST_EVENT_H,
            changing: true,
            be_next_step: true, // classic's load(): the first step is always BE.
            version: 0,
            has_pend: false,
            pend_t: 0.0,
            pend_x: Vec::new(),
            pend_vc: Vec::new(),
            pend_i: Vec::new(),
            watches: HashMap::new(),
            steps_taken: 0,
            newton_iterations: 0,
            holds_free_ic_supply: false, // overwritten by `SteppedSolver::build` right after construction.
        }
    }
}

/// Contract point 7: includes `scheduled` too, not just each partition's own
/// committed state — see `closed_form::Snapshot`'s doc comment for why a
/// later partition in the SAME dirty batch failing first can leave a stale
/// entry sitting here rather than in `PartitionedSolver::pending_schedule`.
#[derive(Debug, Clone)]
pub struct Snapshot {
    partitions: HashMap<usize, PartitionSnapshot>,
    scheduled: Vec<(f64, usize, u64)>,
}

#[derive(Debug, Clone)]
struct PartitionSnapshot {
    x: Vec<f64>,
    vc: Vec<f64>,
    i_prev: Vec<f64>,
    t: f64,
    h: f64,
    changing: bool,
    be_next_step: bool,
    version: u64,
    has_pend: bool,
    pend_t: f64,
    pend_x: Vec<f64>,
    pend_vc: Vec<f64>,
    pend_i: Vec<f64>,
}

/// Owns every `Stepped` partition (>= 2 capacitors, or a single one under
/// `FORCE_STEPPED` — Decision 14), keyed by GLOBAL partition index.
pub struct SteppedSolver {
    partitions: HashMap<usize, Partition>,
    node_owner: HashMap<usize, (usize, usize)>,
    method: Method,
    /// `Step` events this solver wants scheduled, drained by
    /// [`Self::take_scheduled`] — normally every call, but see
    /// `closed_form::Snapshot`'s doc comment for why it is part of
    /// [`Snapshot`] rather than always empty by the time one is taken.
    scheduled: Vec<(f64, usize, u64)>,
    /// Every combinational IC in the program (W1c step 2b, lifting
    /// `partition.rs`'s former decline): [`take_step`]/`solve_initial_x`
    /// restamp whichever of these touch a given `Stepped` partition fresh on
    /// every trial, mirroring `static_part.rs`'s own `ics` list.
    ics: Vec<(usize, digital::IcRecord)>,
    /// Fixed voltages, node -> `Some(v)` — `stamp_ics_into_dyn`'s own
    /// resolver needs this to place an IC pin whose node is fixed.
    fixed: Vec<Option<f64>>,
    /// node -> every IC device index that reads it (contract point 4's
    /// last bullet: "resolves of dynamic partitions must also report IC
    /// readers of nodes that changed, as static does").
    node_to_ic_readers: HashMap<usize, Vec<usize>>,
}

impl SteppedSolver {
    pub fn build(program: &Program, partitions: &[Vec<usize>], local_index: &[Option<(usize, usize)>], fixed: &[Option<f64>], kinds: &[PartitionKind]) -> Self {
        let ics = collect_ics(program);
        // Contract point 3: a partition holding a FREE `vcc`/`gnd` node of
        // any IC is where R_out moves with the partition's own solved state,
        // computed once here (program-wide, mirroring `capacitor_count`'s own
        // per-partition accumulation in `partition.rs`) rather than re-walked
        // on every step.
        let mut holds_free_ic_supply = vec![false; partitions.len()];
        for (_, ic) in &ics {
            for &node in &[ic.vcc.node, ic.gnd.node] {
                if let Some((p, _)) = local_index[node] {
                    holds_free_ic_supply[p] = true;
                }
            }
        }
        let node_to_ic_readers = build_ic_reader_index(&ics);

        let mut parts = HashMap::new();
        let mut node_owner = HashMap::new();
        for (p_idx, kind) in kinds.iter().enumerate() {
            if *kind != PartitionKind::Stepped {
                continue;
            }
            let shape = compile_dyn_shape(p_idx, &partitions[p_idx], local_index, fixed, program);
            for (l, &node) in shape.nodes.iter().enumerate() {
                node_owner.insert(node, (p_idx, l));
            }
            let mut part = Partition::new(shape, program.options.start_time);
            part.holds_free_ic_supply = holds_free_ic_supply[p_idx];
            parts.insert(p_idx, part);
        }
        SteppedSolver {
            partitions: parts,
            node_owner,
            method: Method::from_option(program.options.integration),
            scheduled: Vec::new(),
            ics,
            fixed: fixed.to_vec(),
            node_to_ic_readers,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.partitions.is_empty()
    }

    pub fn watch_node(&mut self, node: usize, reader: DeviceId, thresholds: ThresholdFn) -> bool {
        let Some(&(p_idx, local)) = self.node_owner.get(&node) else {
            return false;
        };
        let part = self.partitions.get_mut(&p_idx).expect("node_owner and partitions built together");
        part.watches.entry(local).or_default().push((reader, thresholds));
        true
    }

    pub fn steps_taken(&self) -> f64 {
        self.partitions.values().map(|p| p.steps_taken as f64).sum()
    }

    pub fn newton_iterations(&self) -> f64 {
        self.partitions.values().map(|p| p.newton_iterations as f64).sum()
    }

    pub fn is_current(&self, partition: usize, version: u64) -> bool {
        self.partitions.get(&partition).is_some_and(|p| p.version == version)
    }

    /// Whether `partition`'s NEXT planned step is forced to BE by the
    /// discontinuity anchor (spec §7.2.3) — test-only introspection, since
    /// nothing outside this module needs to know the method a specific step
    /// used, only that its NUMBERS came out right.
    #[cfg(test)]
    pub(crate) fn debug_be_next_step(&self, partition: usize) -> Option<bool> {
        self.partitions.get(&partition).map(|p| p.be_next_step)
    }

    /// A switch toggle or driver commit touched `dirty` (core/README.md:
    /// same trigger a closed-form re-aim uses): pec3's `_syncDynamic` — fast-
    /// forward to `now` (discarding any pending trial beyond it), mark the
    /// BE anchor (a discontinuity), publish the synced state, then
    /// immediately plan (and schedule) a fresh step from `now`.
    ///
    /// `catchup_drivers` and `drivers` are deliberately DIFFERENT tables
    /// (finding 5, W1b rule 6 amended): an IC driver commit lands via the
    /// ordinary heap mid-`advance()`, at an arbitrary time with `part.t`
    /// generally still short of `now` by up to a whole step — the catch-up
    /// inside [`Self::sync_partition`] below must integrate that whole
    /// `[part.t, now)` span with the stamps that were ACTUALLY in force
    /// during it, i.e. the driver table from BEFORE this commit
    /// (`catchup_drivers` — `solve::mod::PartitionedSolver::resolve_dirty`'s
    /// own pre-overwrite `driver_snapshot`), not the one this same commit
    /// just produced. Only the catch-up call gets it; `solve_initial_x` and
    /// [`Self::plan_and_schedule`] below correctly keep using the current,
    /// post-commit `drivers` — the discontinuity is real and belongs
    /// exactly at `now`, not smeared across the interval leading up to it.
    /// (`W1b`'s own `apply_params_as_event` never hit this because it only
    /// ever runs right after `advance`'s horizon clamp, where `part.t ==
    /// now` already and the catch-up loop is a no-op either way — see
    /// `Kernel::apply_params_as_event`'s doc comment.)
    pub fn resolve_dirty(
        &mut self,
        dirty: &[usize],
        now: f64,
        catchup_drivers: &[Driver],
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
    ) -> Result<Vec<DeviceId>, String> {
        let mut readers = Vec::new();
        for &p_idx in dirty {
            if !self.partitions.contains_key(&p_idx) {
                continue;
            }
            self.sync_partition(p_idx, now, catchup_drivers, live_params, voltages)?;

            let old_bits: Vec<u64> = self.partitions[&p_idx].shape.nodes.iter().map(|&n| voltages[n].to_bits()).collect();
            let iterated = self.partitions[&p_idx].iterated();
            let ic_ctx = IcStampCtx { now, drivers, ics: &self.ics, node_owner: &self.node_owner, fixed: &self.fixed, p_idx, iterated };
            {
                let part = self.partitions.get_mut(&p_idx).unwrap();
                part.be_next_step = true; // a discontinuity: force BE for the next trio.
                part.h = POST_EVENT_H;
                part.changing = true;
                part.version += 1;
                // Re-solve `x` (and seed `i_prev`) from the CURRENT
                // committed `vc` against the stamp as it stands NOW
                // (`solve_initial_x`'s doc comment): at load this is the
                // only thing that ever gives `x`/`i_prev` a real value at
                // all (`Partition::new`'s placeholder is all zeros); at a
                // later re-aim it keeps them from publishing the OLD
                // topology's stale values next to `live_params`-driven
                // switch/resistor currents that already read the NEW one.
                let (x, i_prev) = solve_initial_x(&part.shape, live_params, &part.vc, &part.x, &ic_ctx)?;
                part.x = x;
                part.i_prev = i_prev;
            }
            Self::publish(self.partitions.get(&p_idx).unwrap(), live_params, voltages, currents);
            readers.extend(self.partitions[&p_idx].watches.values().flatten().map(|(id, _)| *id));
            Self::report_changed_ic_readers(&self.partitions[&p_idx], &old_bits, voltages, &self.node_to_ic_readers, &mut readers);
            self.plan_and_schedule(p_idx, now, drivers, live_params, voltages)?;
        }
        Ok(readers)
    }

    /// A due, still-current `Step` event: commits the pending trial, then
    /// immediately plans (and schedules) the next one.
    pub fn handle_step(
        &mut self,
        partition: usize,
        now: f64,
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
    ) -> Result<Vec<DeviceId>, String> {
        let _ = now; // the due time IS `part.pend_t`; `now` is only a sanity anchor.
        let old_bits: Vec<u64> = self.partitions[&partition].shape.nodes.iter().map(|&n| voltages[n].to_bits()).collect();
        {
            let part = self.partitions.get_mut(&partition).ok_or_else(|| format!("stepped: unknown partition {partition}"))?;
            Self::commit(part);
        }
        Self::publish(self.partitions.get(&partition).unwrap(), live_params, voltages, currents);
        let mut readers: Vec<DeviceId> = self.partitions[&partition].watches.values().flatten().map(|(id, _)| *id).collect();
        Self::report_changed_ic_readers(&self.partitions[&partition], &old_bits, voltages, &self.node_to_ic_readers, &mut readers);
        let commit_time = self.partitions[&partition].t;
        self.plan_and_schedule(partition, commit_time, drivers, live_params, voltages)?;
        Ok(readers)
    }

    /// The horizon clamp's stepped half (`PartitionedSolver::sync_dynamic_to`):
    /// fast-forwards every partition still behind `t_end`, publishes it, and
    /// re-plans from there so the next `advance` has a live pending step.
    pub fn sync_to(&mut self, t_end: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) -> Result<Vec<DeviceId>, String> {
        let ids: Vec<usize> = self.partitions.keys().copied().collect();
        let mut readers = Vec::new();
        for p_idx in ids {
            if self.partitions[&p_idx].t >= t_end - 1e-15 {
                continue;
            }
            let old_bits: Vec<u64> = self.partitions[&p_idx].shape.nodes.iter().map(|&n| voltages[n].to_bits()).collect();
            self.sync_partition(p_idx, t_end, drivers, live_params, voltages)?;
            Self::publish(self.partitions.get(&p_idx).unwrap(), live_params, voltages, currents);
            readers.extend(self.partitions[&p_idx].watches.values().flatten().map(|(id, _)| *id));
            // Contract point 4's last bullet ("resolves of dynamic
            // partitions must also report IC readers of nodes that changed,
            // as static does"), extended here to the horizon clamp too — the
            // coordinator's own follow-up point 2: a Schmitt/plain input on a
            // `Stepped` partition must be re-read after EVERY committed step,
            // including whichever ones the clamp itself takes to land exactly
            // on `t_end` (`sync_partition`'s own doc comment: "however many
            // committed steps it takes"), not only the ones `handle_step`
            // commits one at a time from the ordinary event loop. `Kernel::advance`'s
            // horizon clamp now feeds this call's returned readers through
            // the same evaluate/apply-request/re-solve machinery an ordinary
            // due `Step`/`Crossing`'s readers get (finding 1's fix,
            // `kernel.rs`'s own doc comment on the clamp), so reporting them
            // correctly here is no longer merely keeping this solver's OWN
            // contract complete independent of the caller — it is exactly
            // what makes finding 1's fix work at all. Also checked directly,
            // at the solver level, by
            // `stepped::tests::sync_to_reports_ic_readers_of_changed_nodes`
            // (below).
            Self::report_changed_ic_readers(&self.partitions[&p_idx], &old_bits, voltages, &self.node_to_ic_readers, &mut readers);
            self.plan_and_schedule(p_idx, t_end, drivers, live_params, voltages)?;
        }
        Ok(readers)
    }

    pub fn take_scheduled(&mut self) -> Vec<(f64, usize, u64)> {
        std::mem::take(&mut self.scheduled)
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            partitions: self
                .partitions
                .iter()
                .map(|(&p, part)| {
                    (
                        p,
                        PartitionSnapshot {
                            x: part.x.clone(),
                            vc: part.vc.clone(),
                            i_prev: part.i_prev.clone(),
                            t: part.t,
                            h: part.h,
                            changing: part.changing,
                            be_next_step: part.be_next_step,
                            version: part.version,
                            has_pend: part.has_pend,
                            pend_t: part.pend_t,
                            pend_x: part.pend_x.clone(),
                            pend_vc: part.pend_vc.clone(),
                            pend_i: part.pend_i.clone(),
                        },
                    )
                })
                .collect(),
            scheduled: self.scheduled.clone(),
        }
    }

    pub fn restore(&mut self, snapshot: Snapshot) {
        self.scheduled = snapshot.scheduled;
        for (p, s) in snapshot.partitions {
            if let Some(part) = self.partitions.get_mut(&p) {
                part.x = s.x;
                part.vc = s.vc;
                part.i_prev = s.i_prev;
                part.t = s.t;
                part.h = s.h;
                part.changing = s.changing;
                part.be_next_step = s.be_next_step;
                part.version = s.version;
                part.has_pend = s.has_pend;
                part.pend_t = s.pend_t;
                part.pend_x = s.pend_x;
                part.pend_vc = s.pend_vc;
                part.pend_i = s.pend_i;
            }
        }
    }

    // ---- internals ---------------------------------------------------

    /// Publishes every one of this partition's own currents (core/README.md's
    /// "Currents" rules): each capacitor's series-plus-leakage current from
    /// its last ACCEPTED companion, AND — W1b's own review caught this
    /// missing entirely — every resistor's and switch's OWN current
    /// ([`crate::solve::publish_stamped_currents`]), which nothing else in
    /// `PartitionedSolver` ever computed for a device sitting inside a
    /// `Stepped` partition.
    fn publish(part: &Partition, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) {
        for (l, &node) in part.shape.nodes.iter().enumerate() {
            voltages[node] = part.x[l];
        }
        for (idx, cap) in part.shape.capacitors.iter().enumerate() {
            let (va, vb) = (crate::solve::pin_voltage_global(&cap.pins[0], &part.shape, voltages), crate::solve::pin_voltage_global(&cap.pins[1], &part.shape, voltages));
            let leak_i = cap.leak_g * (va - vb);
            currents[cap.device_index] = part.i_prev[idx] + leak_i;
        }
        crate::solve::publish_stamped_currents(&part.shape, voltages, live_params, currents);
    }

    /// W1c contract point 4's last bullet, mirrored from `static_part.rs`'s
    /// own `node_to_ic_readers` walk (`closed_form.rs` carries the identical
    /// helper): for every one of this partition's own local nodes whose
    /// PUBLISHED bits actually changed this commit/re-aim, extends `readers`
    /// with every IC that reads it — not only the ones with an explicit
    /// crossing watch registered. `old_bits` is `part.shape.nodes`-order,
    /// captured by the caller BEFORE `Self::publish`; a duplicate `readers`
    /// entry (already present from `part.watches`) is harmless — `Kernel`
    /// dedups across a whole delta's readers regardless.
    fn report_changed_ic_readers(part: &Partition, old_bits: &[u64], voltages: &[f64], node_to_ic_readers: &HashMap<usize, Vec<usize>>, readers: &mut Vec<DeviceId>) {
        for (l, &node) in part.shape.nodes.iter().enumerate() {
            if voltages[node].to_bits() != old_bits[l] {
                if let Some(ics) = node_to_ic_readers.get(&node) {
                    readers.extend(ics.iter().map(|&d| DeviceId(d as u32)));
                }
            }
        }
    }

    /// Commits `part`'s pending trial: `i_prev` becomes the trial's OWN
    /// series current (spec: "the last ACCEPTED series current"), `vc`/`x`
    /// become its result, `t` becomes `pend_t`, and the BE anchor clears
    /// (spec §7.2.3: "cleared only after an accepted step").
    fn commit(part: &mut Partition) {
        debug_assert!(part.has_pend, "commit called with nothing pending");
        part.has_pend = false;
        part.t = part.pend_t;
        part.x = std::mem::take(&mut part.pend_x);
        part.vc = std::mem::take(&mut part.pend_vc);
        part.i_prev = std::mem::take(&mut part.pend_i);
        part.be_next_step = false;
        part.steps_taken += 1;
    }

    /// pec3's `_syncDynamic` (`:694-703`), generalized for BE/trap: takes
    /// however many committed steps (each bounded to `horizon`) it takes to
    /// bring `part.t` up to `horizon`, using a pending trial outright when it
    /// already lands within `horizon`.
    fn sync_partition(&mut self, p_idx: usize, horizon: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<(), String> {
        loop {
            let (t, has_pend, pend_t) = {
                let part = &self.partitions[&p_idx];
                (part.t, part.has_pend, part.pend_t)
            };
            if t >= horizon - 1e-15 {
                break;
            }
            if has_pend && pend_t <= horizon + 1e-15 {
                let part = self.partitions.get_mut(&p_idx).unwrap();
                Self::commit(part);
                continue;
            }
            self.plan_step(p_idx, horizon, drivers, live_params, voltages)?;
            let part = self.partitions.get_mut(&p_idx).unwrap();
            Self::commit(part);
        }
        Ok(())
    }

    fn plan_and_schedule(&mut self, p_idx: usize, now: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<(), String> {
        self.plan_step(p_idx, f64::INFINITY, drivers, live_params, voltages)?;
        let part = &self.partitions[&p_idx];
        let _ = now;
        self.scheduled.push((part.pend_t, p_idx, part.version));
        Ok(())
    }

    /// pec3's `_planStep` (`:643-687`): the BE/trap step-doubling controller,
    /// plus iterative crossing refinement (this module's doc comment,
    /// replacing pec3's single-fraction clamp). Leaves a fresh pending trial
    /// on `part`; does not commit it. `voltages` is the solver-wide published
    /// array (finding 2/6), threaded down to [`find_crossing`]'s
    /// [`ThresholdFn`] calls: a watched reader's thresholds can depend on a
    /// node OUTSIDE this partition (its IC's supply, sitting in a `Static`
    /// partition), which `part`'s own local `x0`/`x1` arrays never carry.
    fn plan_step(&mut self, p_idx: usize, horizon: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<(), String> {
        let option_method = self.method;
        let ics = &self.ics;
        let node_owner = &self.node_owner;
        let fixed = &self.fixed;
        let part = self.partitions.get_mut(&p_idx).unwrap();
        let shape_n = part.shape.n;
        let method = if part.be_next_step { Method::Be } else { option_method };
        let vc0 = part.vc.clone();
        let x0 = part.x.clone();
        let i_prev = part.i_prev.clone();
        let ic_ctx = IcStampCtx { now: part.t, drivers, ics, node_owner, fixed, p_idx, iterated: part.iterated() };

        let mut h = part.h.min(if part.changing { ACTIVE_H_MAX } else { H_MAX }).min(horizon - part.t);
        let mut ratio;
        let mut accepted: StepResult;
        loop {
            let attempt = attempt_step(&part.shape, live_params, h, &vc0, &x0, &i_prev, method, shape_n, &mut part.newton_iterations, &ic_ctx);
            match attempt {
                Ok((step, r)) => {
                    ratio = r;
                    if ratio <= 1.0 {
                        accepted = step;
                        break;
                    }
                    h = (h * next_step_factor(ratio, method)).max(H_MIN);
                    part.changing = true;
                }
                // A rejected step (README, LEDs point 8; classic's
                // `nextFactor: 0.25`, HL:609-617): retry the whole trio at a
                // quarter of `h`, and give up only when there is no smaller
                // step to try (classic's `hitMinStep`, HL:827-829).
                Err(StepError::NoConverge) => {
                    if h <= H_MIN * 1.01 {
                        return Err(format!("stepped: Newton did not converge at the minimum step h = {h:e} s"));
                    }
                    h = (0.25 * h).max(H_MIN);
                    part.changing = true;
                }
                Err(StepError::Failed(message)) => return Err(message),
            }
        }

        let order_exp = if method == Method::Trap { 3 } else { 2 };
        if h > H_MIN * 2.01 {
            part.changing = ratio * (H_MAX / h).powi(order_exp) > QUIET_FRACTION;
        }
        part.h = (h * next_step_factor(ratio, method)).clamp(H_MIN, if part.changing { ACTIVE_H_MAX } else { H_MAX });

        // Iterative crossing refinement (this module's doc comment).
        if let Some((local, threshold)) = find_crossing(part, &x0, &accepted.x, voltages) {
            let refined_h = refine_crossing_h(&part.shape, live_params, h, &vc0, &x0, &i_prev, method, local, threshold, &mut part.newton_iterations, &ic_ctx)?;
            h = refined_h;
            // Deterministic: `refined_h` is an `h` a trial (or the accepted
            // step itself) already converged at.
            accepted = take_step(&part.shape, live_params, h, &vc0, &x0, &i_prev, method, &mut part.newton_iterations, &ic_ctx).map_err(StepError::into_message)?;
        }

        part.pend_x = accepted.x;
        part.pend_vc = accepted.vc;
        part.pend_i = accepted.i;
        part.pend_t = part.t + h;
        part.has_pend = true;
        part.version += 1;
        Ok(())
    }
}

/// One attempt at the step-doubling trio at `h` (or, at `h <= 2.01 * H_MIN`,
/// one plain step, which has nothing to compare against): the fine result and
/// its error ratio. `NoConverge` from any of the three solves rejects the
/// whole attempt (`plan_step`).
#[allow(clippy::too_many_arguments)]
fn attempt_step(
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc0: &[f64],
    x0: &[f64],
    i_prev: &[f64],
    method: Method,
    shape_n: usize,
    newton_iters: &mut u64,
    ic_ctx: &IcStampCtx,
) -> Result<(StepResult, f64), StepError> {
    if h <= H_MIN * 2.01 {
        return Ok((take_step(shape, live_params, h, vc0, x0, i_prev, method, newton_iters, ic_ctx)?, 0.0));
    }
    let coarse = take_step(shape, live_params, h, vc0, x0, i_prev, method, newton_iters, ic_ctx)?;
    let mid = take_step(shape, live_params, h / 2.0, vc0, x0, i_prev, method, newton_iters, ic_ctx)?;
    let fine = take_step(shape, live_params, h / 2.0, &mid.vc, &mid.x, &mid.i, method, newton_iters, ic_ctx)?;

    let mut ratio = 0.0f64;
    for k in 0..shape_n {
        let r = rel_error(coarse.x[k], fine.x[k]);
        if r > ratio {
            ratio = r;
        }
    }
    for i in 0..vc0.len() {
        let r = rel_error(coarse.vc[i], fine.vc[i]);
        if r > ratio {
            ratio = r;
        }
    }
    Ok((fine, ratio))
}

fn rel_error(a: f64, b: f64) -> f64 {
    (a - b).abs() / (ABS_VOLTAGE + REL_TOL * a.abs().max(b.abs()))
}

/// pec3's crossing search within a planned step (`:668-680`), linear
/// interpolation between the step's start (`x0`) and its accepted end
/// (`x1`): the EARLIEST (by interpolated fraction) watched threshold this
/// step actually crosses, or `None`.
fn find_crossing(part: &Partition, x0: &[f64], x1: &[f64], voltages: &[f64]) -> Option<(usize, Threshold)> {
    let mut best: Option<(f64, usize, Threshold)> = None;
    for (&local, watchers) in &part.watches {
        let (v0, v1) = (x0[local], x1[local]);
        if v0 == v1 {
            continue;
        }
        for (_, thresholds) in watchers {
            for th in thresholds(voltages) {
                let ok = if th.rising { v0 < th.value && v1 > th.value } else { v0 > th.value && v1 < th.value };
                if !ok {
                    continue;
                }
                let frac = (th.value - v0) / (v1 - v0);
                if best.is_none_or(|(bf, _, _)| frac < bf) {
                    best = Some((frac, local, th));
                }
            }
        }
    }
    best.map(|(_, l, th)| (l, th))
}

/// Bisects `h` (never pec3's single `[0.02, 0.98]` fraction clamp, spec D11)
/// until the crossing node lands within [`CROSS_TOLERANCE_V`] of its
/// threshold (nudged past it, `closed_form::CROSS_NUDGE_V`'s same reasoning
/// applies here too — landing exactly ON a threshold risks re-triggering).
#[allow(clippy::too_many_arguments)]
fn refine_crossing_h(
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc0: &[f64],
    x0: &[f64],
    i_prev: &[f64],
    method: Method,
    local: usize,
    threshold: Threshold,
    newton_iters: &mut u64,
    ic_ctx: &IcStampCtx,
) -> Result<f64, String> {
    let target = threshold.value + if threshold.rising { CROSS_TOLERANCE_V } else { -CROSS_TOLERANCE_V };
    let (mut lo, mut hi) = (0.0f64, h);
    let mut best_h = hi;
    for _ in 0..CROSS_REFINE_MAX_ITERS {
        let mid = 0.5 * (lo + hi);
        let trial = match take_step(shape, live_params, mid, vc0, x0, i_prev, method, newton_iters, ic_ctx) {
            Ok(trial) => trial,
            // A trial that does not converge ends the refinement at the best
            // `h` found so far (README, LEDs point 8).
            Err(StepError::NoConverge) => return Ok(best_h),
            Err(StepError::Failed(message)) => return Err(message),
        };
        let v = trial.x[local];
        let past = if threshold.rising { v >= target } else { v <= target };
        if past {
            hi = mid;
            best_h = mid;
        } else {
            lo = mid;
        }
        if (v - target).abs() < CROSS_TOLERANCE_V {
            return Ok(mid);
        }
    }
    Ok(best_h)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition;
    use crate::program::{Device, Options, PinRecord};

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }
    fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: crate::kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }
    fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: crate::kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }
    fn capacitor(ci: usize, a: usize, b: usize, c: f64, esr: f64, leak: f64) -> Device {
        Device { kind: crate::kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![c, esr, leak], blob: vec![] }
    }
    fn program(node_count: usize, devices: Vec<Device>, options: Options) -> Program {
        Program { node_count, options, digital_slot_count: 0, channel_slot_count: 0, devices }
    }
    fn build(program: &Program) -> SteppedSolver {
        let pb = partition::build(program).unwrap();
        SteppedSolver::build(program, &pb.partitions, &pb.local_index, &pb.fixed, &pb.kinds)
    }

    /// `source --r1-- node1 --c1-- gnd`, `node1 --r2-- node2 --c2-- gnd`:
    /// `analytic.ts`'s `rcLadderVoltagesAt` fixture — two capacitors, one
    /// linear partition, Decision 14's genuinely-`Stepped` case (no
    /// `FORCE_STEPPED` needed).
    fn ladder_program(v: f64, r1: f64, c1: f64, r2: f64, c2: f64) -> Program {
        program(4, vec![source(0, 1, 0, v), resistor(1, 1, 2, r1), capacitor(2, 2, 0, c1, 0.0, 0.0), resistor(3, 2, 3, r2), capacitor(4, 3, 0, c2, 0.0, 0.0)], Options::default())
    }

    /// Port of `test/core/equivalence/analytic.ts`'s `rcLadderVoltagesAt`:
    /// the exact two-capacitor ladder trajectory via eigendecomposition of
    /// the linear ODE `dx/dt = A x + b`, both capacitors starting uncharged.
    /// Kept independent of this module's own numerics (translated from the
    /// TS reference, not derived from anything in `stepped.rs`) so it is a
    /// genuine cross-check, not a tautology.
    fn ladder_reference(v: f64, r1: f64, c1: f64, r2: f64, c2: f64, t: f64) -> (f64, f64) {
        let a11 = -(1.0 / r1 + 1.0 / r2) / c1;
        let a12 = 1.0 / (r2 * c1);
        let a21 = 1.0 / (r2 * c2);
        let a22 = -1.0 / (r2 * c2);
        let trace = a11 + a22;
        let det = a11 * a22 - a12 * a21;
        let discriminant = trace * trace - 4.0 * det;
        assert!(discriminant >= 0.0, "expected real eigenvalues for a passive RC ladder");
        let sqrt_disc = discriminant.sqrt();
        let lambda1 = (trace + sqrt_disc) / 2.0;
        let lambda2 = (trace - sqrt_disc) / 2.0;
        let eigenvector = |lambda: f64| (a12, lambda - a11);
        let e1 = eigenvector(lambda1);
        let e2 = eigenvector(lambda2);
        let det_m = e1.0 * e2.1 - e2.0 * e1.1;
        let k1 = (-v * e2.1 - e2.0 * -v) / det_m;
        let k2 = (e1.0 * -v - -v * e1.1) / det_m;
        let mode1 = k1 * (lambda1 * t).exp();
        let mode2 = k2 * (lambda2 * t).exp();
        (v + mode1 * e1.0 + mode2 * e2.0, v + mode1 * e1.1 + mode2 * e2.1)
    }

    #[test]
    fn two_capacitor_ladder_converges_to_the_eigendecomposition_reference() {
        let (v, r1, c1, r2, c2) = (5.0, 1000.0, 1e-6, 1000.0, 1e-6);
        let p = ladder_program(v, r1, c1, r2, c2);
        let mut st = build(&p);
        assert_eq!(st.partitions.len(), 1, "two capacitors in one linear partition must classify Stepped");
        let mut voltages = vec![0.0, v, 0.0, 0.0];
        let mut currents = vec![0.0; 5];
        st.resolve_dirty(&[0], 0.0, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        // Not testing t=1e-4: node2 is still barely charged there (its own
        // RC lag behind node1), so even a tiny absolute error reads as a
        // large RELATIVE one — a comparison of the numerically-converged
        // step-doubling result against the eigendecomposition reference, not
        // a real accuracy regression.
        for &t in &[5e-4, 1e-3, 3e-3] {
            let (mut vv, mut cc) = (voltages.clone(), currents.clone());
            st.sync_to(t, &[], &HashMap::new(), &mut vv, &mut cc).unwrap();
            let (expected_v1, expected_v2) = ladder_reference(v, r1, c1, r2, c2, t);
            // Decision 14: "within classic's tolerance but not exact" — a
            // numerically-converged stepped result, not the single-capacitor
            // closed form's ~1e-9 bar.
            let rel1 = (vv[2] - expected_v1).abs() / expected_v1.abs();
            let rel2 = (vv[3] - expected_v2).abs() / expected_v2.abs();
            assert!(rel1 < 2e-2, "t={t}: node1 got {} expected {} rel {rel1}", vv[2], expected_v1);
            assert!(rel2 < 2e-2, "t={t}: node2 got {} expected {} rel {rel2}", vv[3], expected_v2);
        }
        assert!(st.steps_taken() > 0.0);
        assert!(st.newton_iterations() > 0.0, "every step must route through Newton (W1 plan row B3), even converging in one iteration");
    }

    /// Section 2 of the W1b review, contract point 5: a stepped partition's
    /// clock starts at `START_TIME`, so a load integrates nothing and costs
    /// the same at any start time — `Partition::new`'s `t` used to be
    /// hardcoded to 0 regardless of `t0`, so `resolve_dirty`'s first
    /// `sync_partition(p_idx, now=START_TIME, ...)` call genuinely
    /// integrated the whole `[0, START_TIME]` interval with real BE/trap
    /// steps before the circuit was even considered loaded. Checked by
    /// comparing `START_TIME=5` against `START_TIME=0` directly: with the
    /// fix, both take zero steps at load and land on bit-identical (both
    /// near-zero, `vc=0`-seeded) node voltages, since `solve_initial_x` is a
    /// pure function of the topology and `vc`, never of `now` itself.
    #[test]
    fn a_stepped_ladder_at_a_nonzero_start_time_comes_up_uncharged_with_zero_steps_at_load() {
        let (v, r1, c1, r2, c2) = (5.0, 1000.0, 1e-6, 1000.0, 1e-6);

        let p0 = ladder_program(v, r1, c1, r2, c2); // START_TIME defaults to 0.
        let mut st0 = build(&p0);
        let mut voltages0 = vec![0.0, v, 0.0, 0.0];
        let mut currents0 = vec![0.0; 5];
        st0.resolve_dirty(&[0], 0.0, &[], &[], &HashMap::new(), &mut voltages0, &mut currents0).unwrap();

        let mut options5 = Options::default();
        options5.start_time = 5.0;
        let p5 = program(4, vec![source(0, 1, 0, v), resistor(1, 1, 2, r1), capacitor(2, 2, 0, c1, 0.0, 0.0), resistor(3, 2, 3, r2), capacitor(4, 3, 0, c2, 0.0, 0.0)], options5);
        let mut st5 = build(&p5);
        let mut voltages5 = vec![0.0, v, 0.0, 0.0];
        let mut currents5 = vec![0.0; 5];
        st5.resolve_dirty(&[0], 5.0, &[], &[], &HashMap::new(), &mut voltages5, &mut currents5).unwrap();

        assert_eq!(st5.steps_taken(), 0.0, "loading at a nonzero START_TIME must not integrate anything");
        for l in [2usize, 3] {
            assert_eq!(voltages5[l].to_bits(), voltages0[l].to_bits(), "node {l}: START_TIME=5 ({}) must match START_TIME=0 ({}) bit-for-bit — both take zero steps at load", voltages5[l], voltages0[l]);
            assert!(voltages5[l].abs() < 1e-3, "node {l} got {}, expected an uncharged-capacitor value, not something charged toward 5V as if it had run since t=0", voltages5[l]);
        }
    }

    /// The `FORCE_STEPPED` single-capacitor counterpart of the ladder test
    /// above: the same zero-steps-at-load, bit-identical-to-START_TIME=0
    /// guarantee must hold for a partition that only becomes `Stepped`
    /// because of the option, not because it genuinely has two capacitors.
    #[test]
    fn a_force_stepped_single_capacitor_at_a_nonzero_start_time_comes_up_uncharged_with_zero_steps() {
        let (v, r, c) = (5.0, 1000.0, 1e-6);
        let build_at = |start_time: f64| {
            let mut options = Options::default();
            options.force_stepped = true;
            options.start_time = start_time;
            let p = program(3, vec![source(0, 1, 0, v), resistor(1, 1, 2, r), capacitor(2, 2, 0, c, 0.0, 0.0)], options);
            let mut st = build(&p);
            let mut voltages = vec![0.0, v, 0.0];
            let mut currents = vec![0.0; 3];
            st.resolve_dirty(&[0], start_time, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
            (st, voltages)
        };
        let (_st0, voltages0) = build_at(0.0);
        let (st5, voltages5) = build_at(5.0);

        assert_eq!(st5.steps_taken(), 0.0, "loading at a nonzero START_TIME under FORCE_STEPPED must not integrate anything");
        assert_eq!(voltages5[2].to_bits(), voltages0[2].to_bits(), "START_TIME=5 ({}) must match START_TIME=0 ({}) bit-for-bit", voltages5[2], voltages0[2]);
        assert!(voltages5[2].abs() < 1e-3, "got {}, expected an uncharged-capacitor value", voltages5[2]);
    }

    /// The single-capacitor `FORCE_STEPPED` case (Decision 14; W1 plan row
    /// B3's "the RC/RL discrete-conservation benchmarks under
    /// `FORCE_STEPPED`"): one committed BE step's accepted charge change
    /// must equal `i*h` exactly (BE's own definition of the companion,
    /// spec §7.4), not merely approximately.
    #[test]
    fn discrete_charge_conservation_under_force_stepped_matches_be_exactly() {
        let mut options = Options::default();
        options.force_stepped = true;
        let (v, r, c) = (5.0, 1000.0, 1e-6);
        let p = program(3, vec![source(0, 1, 0, v), resistor(1, 1, 2, r), capacitor(2, 2, 0, c, 0.0, 0.0)], options);
        let mut st = build(&p);
        let mut voltages = vec![0.0, v, 0.0];
        let mut currents = vec![0.0; 3];
        st.resolve_dirty(&[0], 0.0, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let vc_before = st.partitions[&0].vc[0];
        let h = st.partitions.get(&0).unwrap().h;
        st.plan_step(0, f64::INFINITY, &[], &HashMap::new(), &voltages).unwrap();
        let part = &st.partitions[&0];
        let accepted_h = part.pend_t - 0.0;
        let vc_after = part.pend_vc[0];
        let i_accepted = part.pend_i[0];
        // BE's own definition: vc += (h/C)*i (spec §7.4), so C*dvc must equal
        // i*h bit-for-bit up to how the doubling controller's own h differs
        // from what was FIRST attempted (`h` above is only the initial
        // guess, `accepted_h` is what the doubling loop actually settled on
        // — the conservation law is checked against the latter).
        let dq = c * (vc_after - vc_before);
        let expected_dq = i_accepted * accepted_h;
        assert!((dq - expected_dq).abs() < 1e-15, "dq={dq} expected={expected_dq}");
        let _ = h;
    }

    /// Spec §7.2.3: a discontinuity (a switch toggle or driver commit — here
    /// simulated by re-`resolve_dirty`-ing the same partition, which is
    /// exactly what a live `bo_engine_set_params` toggle does) forces BE for
    /// the WHOLE next step-doubling trio, cleared only once that step is
    /// accepted.
    #[test]
    fn a_discontinuity_forces_the_be_anchor_until_the_next_step_is_accepted() {
        let mut options = Options::default();
        options.force_stepped = true;
        options.integration = 1; // trap
        let (v, r, c) = (5.0, 1000.0, 1e-6);
        let p = program(3, vec![source(0, 1, 0, v), resistor(1, 1, 2, r), capacitor(2, 2, 0, c, 0.0, 0.0)], options);
        let mut st = build(&p);
        let mut voltages = vec![0.0, v, 0.0];
        let mut currents = vec![0.0; 3];

        // `resolve_dirty` is load's own first segment/plan too, so the very
        // first plan is BE regardless (classic's `load()`); accept it, then
        // trigger a SECOND discontinuity explicitly to prove the flag gets
        // RE-SET, not merely left over from load.
        st.resolve_dirty(&[0], 0.0, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert_eq!(st.debug_be_next_step(0), Some(true), "the first step after load must be BE");
        let now = st.partitions[&0].pend_t;
        st.handle_step(0, now, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert_eq!(st.debug_be_next_step(0), Some(false), "an accepted step clears the BE anchor");

        st.resolve_dirty(&[0], now, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert_eq!(st.debug_be_next_step(0), Some(true), "a re-aim (a discontinuity) must set the BE anchor again");
    }

    /// core/README.md's "Currents": every resistor inside a `Stepped`
    /// partition must publish `(Va-Vb)/R` too, not just a `ClosedForm` one's
    /// — checked here against KCL at BOTH ladder nodes, at several times,
    /// exactly as `closed_form`'s own equivalent test does.
    #[test]
    fn resistors_inside_a_stepped_ladder_publish_their_own_current_and_satisfy_kcl() {
        let (v, r1, c1, r2, c2) = (5.0, 1000.0, 1e-6, 1000.0, 1e-6);
        let p = ladder_program(v, r1, c1, r2, c2);
        let mut st = build(&p);
        let mut voltages = vec![0.0, v, 0.0, 0.0];
        let mut currents = vec![0.0; 5];
        st.resolve_dirty(&[0], 0.0, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let shunt = crate::solve::static_part::NODE_RSHUNT_G;
        for &t in &[1e-4, 5e-4, 1e-3, 3e-3] {
            let (mut vv, mut cc) = (voltages.clone(), currents.clone());
            st.sync_to(t, &[], &HashMap::new(), &mut vv, &mut cc).unwrap();

            let expected_r1 = (vv[1] - vv[2]) / r1;
            let expected_r2 = (vv[2] - vv[3]) / r2;
            assert!(cc[1] != 0.0 && cc[3] != 0.0, "t={t}: resistor currents must not be published as a stale/absent 0.0 (r1={} r2={})", cc[1], cc[3]);
            assert!((cc[1] - expected_r1).abs() < 1e-9, "t={t}: r1 got {} expected {}", cc[1], expected_r1);
            assert!((cc[3] - expected_r2).abs() < 1e-9, "t={t}: r2 got {} expected {}", cc[3], expected_r2);

            // KCL at node1 (voltages[2]): r1's current in = c1's current out
            // + r2's current out + the node's own shunt current out.
            let kcl_node1 = cc[1] - cc[2] - cc[3] - shunt * vv[2];
            assert!(kcl_node1.abs() < 1e-9, "t={t}: KCL residual at node1 = {kcl_node1}");
            // KCL at node2 (voltages[3]): r2's current in = c2's current out
            // + the node's own shunt current out.
            let kcl_node2 = cc[3] - cc[4] - shunt * vv[3];
            assert!(kcl_node2.abs() < 1e-9, "t={t}: KCL residual at node2 = {kcl_node2}");
        }
    }

    /// `test/fixtures/core/14-two-capacitor-ladder`: right after
    /// `bo_engine_load` (before any `advance` at all — the FIRST
    /// `resolve_dirty` call IS `Kernel::settle`'s load-time resolve), every
    /// device's current must already be the real value consistent with the
    /// starting operating point (`vc = 0`, README: "Initial vc = 0 at load"),
    /// not the `0.0` a never-solved `Partition::new`'s placeholder `x` would
    /// publish — the exact bug T2's equivalence harness caught: `elementI`
    /// for c1/c2/r2 read `0` at `t = START_TIME` and only became correct
    /// from the next sample on.
    #[test]
    fn a_stepped_ladders_currents_are_correct_at_load_time_before_any_advance() {
        let (v, r1, c1, r2, c2) = (5.0, 1000.0, 1e-6, 1000.0, 1e-6);
        let p = ladder_program(v, r1, c1, r2, c2);
        let mut st = build(&p);
        let mut voltages = vec![0.0, v, 0.0, 0.0];
        let mut currents = vec![0.0; 5];

        // This IS load time: no `sync_to`/`handle_step` has run yet.
        st.resolve_dirty(&[0], 0.0, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        // At vc=0 for both (uncharged) capacitors, both node voltages sit at
        // the resistive divider's value with no capacitor current flowing
        // YET at this exact instant... except the divider itself is exactly
        // `V` at every node once solved (no current has had time to flow
        // into either capacitor at t=0 the way a BE company model would
        // report it, but the INSTANTANEOUS operating point with vc HELD at
        // 0 does draw a real current through R1/R2, which is exactly the
        // value classic and the core's own later samples agree on).
        let expected_r1 = (voltages[1] - voltages[2]) / r1;
        let expected_r2 = (voltages[2] - voltages[3]) / r2;
        assert!(currents[1] != 0.0, "r1's current must not read 0 at load time, got {}", currents[1]);
        assert!(currents[3] != 0.0, "r2's current must not read 0 at load time, got {}", currents[3]);
        assert!((currents[1] - expected_r1).abs() < 1e-9, "r1 got {} expected {}", currents[1], expected_r1);
        assert!((currents[3] - expected_r2).abs() < 1e-9, "r2 got {} expected {}", currents[3], expected_r2);
        assert!(currents[2] != 0.0, "c1's current must not read 0 at load time, got {}", currents[2]);
        assert!(currents[4] != 0.0, "c2's current must not read 0 at load time, got {}", currents[4]);

        let shunt = crate::solve::static_part::NODE_RSHUNT_G;
        let kcl_node1 = currents[1] - currents[2] - currents[3] - shunt * voltages[2];
        assert!(kcl_node1.abs() < 1e-9, "KCL residual at node1 (load time) = {kcl_node1}");
        let kcl_node2 = currents[3] - currents[4] - shunt * voltages[3];
        assert!(kcl_node2.abs() < 1e-9, "KCL residual at node2 (load time) = {kcl_node2}");
    }

    // ---- W1c step 2b's own follow-up: IC readers of changed nodes, after
    // EVERY committed step (`resolve_dirty`, `handle_step` AND `sync_to`) --

    fn ls04_reading(ci: usize, vcc: usize, gnd: usize, a1: usize, dump: usize) -> Device {
        use crate::kinds::ic_param;
        let mut params = vec![0.0; ic_param::COUNT];
        params[ic_param::VCC_MIN] = 4.75;
        params[ic_param::VCC_MAX] = 5.25;
        params[ic_param::FAM_FLAGS] = crate::kinds::FAM_TTL as f64;
        params[ic_param::VIL_FIXED] = 1.0;
        params[ic_param::VIL] = 0.8;
        params[ic_param::VIH_FIXED] = 1.0;
        params[ic_param::VIH] = 2.0;
        params[ic_param::NOMINAL] = 5.0;
        params[ic_param::IO_MAX] = 0.008;
        params[ic_param::DELAY] = 20e-9;
        // `1a 1y 2a 2y 3a 3y 4a 4y 5a 5y 6a 6y`: 1a reads `a1` (the watched
        // node); every other pin ties to `dump` (an isolated, otherwise
        // unconnected node — irrelevant to this test).
        let pins = vec![pin(vcc), pin(gnd), pin(a1), pin(dump), pin(dump), pin(dump), pin(dump), pin(dump), pin(dump), pin(dump), pin(dump), pin(dump), pin(dump), pin(dump)];
        let blob_flags: Vec<f64> = (0..12).map(|i| if i % 2 == 0 { crate::kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }).collect();
        let mut blob = vec![0.0];
        blob.extend(blob_flags);
        Device { kind: crate::kinds::IC_74LS04, component_index: ci, pins, params, blob }
    }

    /// Contract point 4's last bullet ("resolves of dynamic partitions must
    /// also report IC readers of nodes that changed, as static does"), and
    /// the coordinator's own W1c step 2b follow-up point 2: for a `Stepped`
    /// partition this must hold after `resolve_dirty` (the load-time/re-aim
    /// path), `handle_step` (an ordinary committed step) AND `sync_to` (the
    /// horizon clamp's own committed steps) alike — a Schmitt or plain input
    /// on a stepped RC must be re-read every time, not only at an explicit
    /// crossing watch. No `watch_node` call anywhere in this test: the IC
    /// (device 5) reads node2 purely through `node_to_ic_readers`
    /// (`SteppedSolver::build`'s own `collect_ics`/`build_ic_reader_index`),
    /// proving this is the GENERIC fallback the contract calls for, not
    /// something that only works alongside a registered watch.
    #[test]
    fn resolve_dirty_handle_step_and_sync_to_all_report_the_ic_reader_of_a_changed_node() {
        let (v, r1, c1, r2, c2) = (5.0, 1000.0, 1e-6, 1000.0, 1e-6);
        let mut devices = vec![
            source(0, 1, 0, v),
            resistor(1, 1, 2, r1),
            capacitor(2, 2, 0, c1, 0.0, 0.0),
            resistor(3, 2, 3, r2),
            capacitor(4, 3, 0, c2, 0.0, 0.0),
        ];
        devices.push(ls04_reading(5, 1, 0, 2, 4)); // reads node2 (the first RC node), dump = node4.
        let p = program(5, devices, Options::default());
        let mut st = build(&p);
        assert_eq!(st.partitions.len(), 1, "two capacitors: genuinely Stepped");
        let mut voltages = vec![0.0, v, 0.0, 0.0, 0.0];
        let mut currents = vec![0.0; 6];

        // Load-time resolve_dirty: node2 moves from its 0.0 placeholder to
        // the real operating point (`solve_initial_x`'s own seed), so the ic
        // reader (device 5) must be reported even with no watch registered.
        let readers = st.resolve_dirty(&[0], 0.0, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert!(readers.contains(&DeviceId(5)), "resolve_dirty must report the ic reader when node2 changes, got {readers:?}");

        // handle_step: commit one real, ordinary step -- node2 keeps moving.
        let now = st.partitions[&0].pend_t;
        let readers = st.handle_step(0, now, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert!(readers.contains(&DeviceId(5)), "handle_step must report the ic reader when node2 changes, got {readers:?}");

        // sync_to: the horizon clamp, well past several more committed steps.
        let readers = st.sync_to(5e-4, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert!(readers.contains(&DeviceId(5)), "sync_to must report the ic reader when node2 changes, got {readers:?}");
    }

    // ---- W1e: junctions in a stepped partition (LEDs point 8) --------------

    use crate::solve::newton::hooks;

    fn red_led(ci: usize, anode: usize, cathode: usize) -> Device {
        Device { kind: crate::kinds::LED, component_index: ci, pins: vec![pin(anode), pin(cathode)], params: vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0], blob: vec![] }
    }

    /// `5 V --1 kohm-- n2`, `n2` to the datum through 100 uF and a red LED:
    /// one capacitor and one junction, so stepped and nonlinear. Node 2 is
    /// the only free node.
    fn rc_led() -> (SteppedSolver, Vec<f64>, Vec<f64>) {
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0, 100e-6, 0.0, 0.0), red_led(3, 2, 0)], Options::default());
        let mut st = build(&p);
        assert_eq!(st.partitions.len(), 1, "an LED beside one capacitor is stepped, not closed-form");
        let mut voltages = vec![0.0, 5.0, 0.0];
        let mut currents = vec![0.0; 4];
        st.resolve_dirty(&[0], 0.0, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        (st, voltages, currents)
    }

    fn pending_step(st: &SteppedSolver) -> f64 {
        let part = &st.partitions[&0];
        part.pend_t - part.t
    }

    fn plan(st: &mut SteppedSolver, h: f64, voltages: &[f64]) -> Result<(), String> {
        st.partitions.get_mut(&0).unwrap().h = h;
        st.plan_step(0, f64::INFINITY, &[], &HashMap::new(), voltages)
    }

    #[test]
    fn a_junction_makes_every_step_a_real_newton_solve() {
        let (mut st, voltages, _) = rc_led();
        let before = st.newton_iterations();
        plan(&mut st, 1e-4, &voltages).unwrap();
        // The trio is three solves, and a nonlinear one takes at least two
        // iterations each (the second is what shows the first passed).
        assert!(st.newton_iterations() - before >= 6.0, "{}", st.newton_iterations() - before);
    }

    #[test]
    fn a_step_that_does_not_converge_is_retried_at_a_quarter_of_h() {
        let (mut control, voltages, _) = rc_led();
        plan(&mut control, 1e-4, &voltages).unwrap();
        assert_eq!(pending_step(&control), 1e-4, "smooth RC: the first attempt is accepted at the given h");

        let (mut st, voltages, _) = rc_led();
        let _guard = hooks::force_unconverged(1);
        plan(&mut st, 1e-4, &voltages).unwrap();
        assert_eq!(pending_step(&st), 0.25 * 1e-4, "one rejected attempt, then the retry at 0.25 h");
        assert!(st.partitions[&0].changing, "the partition is marked changing");
    }

    #[test]
    fn a_step_fails_only_when_it_cannot_converge_at_the_floor() {
        // h runs 1e-6, 2.5e-7, 6.25e-8, 1.5625e-8, then the floor 1e-8: five
        // attempts. Four rejections leave a fifth that converges at H_MIN.
        {
            let (mut st, voltages, _) = rc_led();
            let _guard = hooks::force_unconverged(4);
            plan(&mut st, 1e-6, &voltages).unwrap();
            assert_eq!(pending_step(&st), H_MIN, "converged at the floor, so no error");
        }
        // Five rejections fail at the floor, having run each attempt's
        // solve (3 forced iterations apiece).
        let (mut st, voltages, _) = rc_led();
        let before = st.newton_iterations();
        let _guard = hooks::force_unconverged(u32::MAX);
        let err = plan(&mut st, 1e-6, &voltages).unwrap_err();
        assert!(err.contains("minimum step"), "{err}");
        assert_eq!(st.newton_iterations() - before, 15.0, "5 attempts x 3 iterations, counted though rejected");
    }

    #[test]
    fn a_non_finite_stamp_is_minus_4_at_once_not_a_rejected_step() {
        // The retry is for Newton non-convergence only. A stamp that goes
        // non-finite fails on the first attempt, with no `0.25 h` loop: a NaN
        // live value for the resistor makes its conductance NaN.
        let (mut st, voltages, _) = rc_led();
        let mut live = HashMap::new();
        live.insert(1usize, vec![f64::NAN]);
        let before = st.newton_iterations();
        let err = st.plan_step(0, f64::INFINITY, &[], &live, &voltages).unwrap_err();
        assert!(!err.contains("minimum step"), "not a rejected step: {err}");
        assert!(st.newton_iterations() - before <= 3.0, "gave up on the first attempt");
    }

    #[test]
    fn a_refinement_trial_that_does_not_converge_ends_the_refinement_at_the_best_h_so_far() {
        let watch = |st: &mut SteppedSolver| {
            // A rising threshold the first 1 ms step crosses: the node charges at
            // about 50 V/s from 0, so it reaches 0.03 V near 0.6 ms.
            assert!(st.watch_node(2, DeviceId(0), Box::new(|_v: &[f64]| vec![Threshold { value: 0.03, rising: true }])));
        };
        let (mut control, voltages, _) = rc_led();
        watch(&mut control);
        plan(&mut control, 1e-3, &voltages).unwrap();
        let refined = pending_step(&control);
        assert!(refined > 5e-4 && refined < 7e-4, "refined to the crossing near 0.6 ms: {refined}");
        assert!((control.partitions[&0].pend_x[0] - 0.03).abs() < 1e-8, "and it lands on the threshold");

        let (mut st, voltages, _) = rc_led();
        watch(&mut st);
        // The trio is three solves; the fourth is the first refinement trial.
        let _guard = hooks::force_unconverged_after(3, 1);
        plan(&mut st, 1e-3, &voltages).unwrap();
        assert_eq!(pending_step(&st), 1e-3, "no trial converged, so the unrefined step stands");
        assert!(st.partitions[&0].pend_x[0] > 0.03, "having stepped past the threshold");
    }

    #[test]
    fn the_re_aim_seed_starts_from_the_committed_x_not_from_zero() {
        let (mut st, mut voltages, mut currents) = rc_led();
        st.sync_to(0.05, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        let committed = st.partitions[&0].x.clone();
        assert!(committed[0] > 1.0, "charged by now: {committed:?}");
        let _guard = hooks::record_initial_guesses();
        // A discontinuity at the instant the partition is already at.
        st.resolve_dirty(&[0], 0.05, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        let guesses = hooks::recorded_initial_guesses();
        assert_eq!(guesses[0], committed, "the seed solve is the first Newton after the catch-up");
    }
}
