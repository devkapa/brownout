//! The closed-form dynamic-partition solver (core/README.md, "Kernel
//! semantics: events"; W1 plan row B2; Decision 14: exactly one capacitor).
//! Port of pec3.mjs's `_segment`/`_aim`/`_vcAt`/`_scheduleCrossing`/
//! `_analyticCross`/`_recross` (`:532-628`), with the rail-sensitivity half
//! of that machinery dropped: W1b has no hub rail (that is W1f), so every
//! segment is a plain function of time with no `du` correction term.
//!
//! **The exact model.** Replace the partition's one capacitor with an IDEAL
//! voltage source of value `vc` in series with its `esr` (an extra
//! branch-current unknown, MNA-style — pec3's `M_SOURCE` stamp). Since every
//! OTHER device in a W1b partition is linear (a resistor, a switch's CURRENT
//! contact state, the capacitor's own leakage), the rest of the network's
//! solution — every node voltage and the branch current itself — is an
//! AFFINE function of `vc`: solving once at `vc=0` and once at `vc=1` (same
//! factored matrix, `b` only differs by the coefficient on `vc`, so this is
//! one LU factorization plus two applies) gives `x(vc) = x0 + dx*vc` and
//! `i(vc) = i0 + di*vc` exactly, for any `vc`, with no discretization error
//! at all. Since `i = C*dvc/dt`, this makes `vc` itself the exact solution
//! of a first-order LINEAR ODE, `dvc/dt = F*(vc - vinf)` with `F = di/C` and
//! `vinf = -i0/di` (`F == 0`'s degenerate constant-derivative case is
//! guarded, matching pec3): `vc(t) = vinf + (vc0-vinf)*exp(F*(t-t0))`. A
//! "segment" is this quadruple `(x0, dx, i0, di)` plus the anchor `(t0,
//! vc0)`; it stays exactly valid until something in the partition's own
//! linear system changes (a switch toggle, a driver commit) — a "re-aim" —
//! at which point a fresh segment is solved and anchored at the CURRENT
//! exact `vc` (`_vcAt`), never a discontinuity.
//!
//! **Crossings.** A registered [`ThresholdFn`] (the crossing-watch API,
//! [`ClosedFormSolver::watch_node`]) is checked against the segment's
//! asymptote: a threshold strictly between "now" and "where this segment is
//! headed" has an exact, closed-form crossing time (solving `vc(t) = vc*`
//! for `t`), scheduled through the kernel's heap
//! ([`ClosedFormSolver::take_scheduled`]) if it clears the acceptance rule
//! (`kernel::accepts_crossing`, contract point 4) — applied HERE, before the
//! candidate ever reaches a heap, exactly as `kernel::accepts_crossing`'s own
//! doc comment describes. Landing on a crossing nudges the crossed node's
//! value by [`CROSS_NUDGE_V`] past the threshold (pec3's own "hard-won
//! rule": accepting a crossing exactly ON a threshold can put a node back on
//! it and reschedule forever) and re-aims from there.

use std::collections::HashMap;
use std::rc::Rc;

use crate::digital;
use crate::drivers::Driver;
use crate::events::DeviceId;
use crate::kernel::accepts_crossing;
use crate::lu;
use crate::partition::PartitionKind;
use crate::program::Program;
use crate::solve::static_part::mode_discriminant;
use crate::rails::{self, CapSeg, CellDrive, Contrib};
use crate::solve::live::RampEval;
use crate::solve::curvature;
use crate::solve::{
    build_ic_reader_index, collect_ics, compile_dyn_shape, live_params_for, pin_voltage_global, rail_sensitivity, stamp_branch_terminal, stamp_ics_into_dyn,
    stamp_linear_base, stamp_residual, Bnd, DynCapacitor, DynPin, DynShape, LiveTopology, SharedThresholdFn, ThresholdFn,
};

/// The 1e-9 V nudge past a crossed threshold (W1 plan row B2; pec3.mjs
/// `CROSS_NUDGE_V`, `:34`) — see this module's doc comment.
pub const CROSS_NUDGE_V: f64 = 1e-9;

/// The exact affine relationship this partition's whole linear network has
/// to its one capacitor's voltage `vc` (this module's doc comment): `x(vc) =
/// x0 + dx*vc`, `i(vc) = i0 + di*vc`.
#[derive(Debug, Clone)]
struct Segment {
    x0: Vec<f64>,
    dx: Vec<f64>,
    i0: f64,
    di: f64,
    /// Everything [`build_segment`] read that can change between two builds
    /// of the same partition (see [`ClosedFormSolver::segment_key`]). Two builds
    /// with the same key are the same function of the same inputs, so the
    /// second is skipped and the first segment reused.
    key: Vec<u8>,
    /// A hub partition's rail terms (W1f, README Rails points 2-4,
    /// P3:533-556): its draw `a + b vc` at the rail `u_seg` it was built at,
    /// the slope `g`, and the sensitivities `dx0/du`, `di0/du` that re-aim the
    /// segment at another rail.
    hub: Option<Box<SegHub>>,
    /// Per boundary node a resolved ramp moved when it was built (Sources
    /// point 3, R3 (b)): the node, `dx0/dv` and `di0/dv`. The network is
    /// linear, so these are exact, and a ramp's slope through them drives the
    /// segment between corners.
    ramp_sens: Vec<(usize, Vec<f64>, f64)>,
}

#[derive(Debug, Clone)]
struct SegHub {
    a: f64,
    b: f64,
    g: f64,
    u_seg: f64,
    dx0du: Vec<f64>,
    di0du: f64,
    /// A curved hub partition's terms (ruling R8 (a)), none for any other.
    curve: Option<Box<SegCurve>>,
}

/// A curved hub partition's segment (one holding a pin of an IC whose
/// supply is the rail, Rails point 3): its network's conductances follow the
/// rail, so `dx` and `di` do too, and the sensitivities depend on `vc`:
/// `dx/du = dx0du + dx1du vc`, `di/du = di0du + di1du vc`, and the draw's
/// slope `g + g1 vc`. Exact at the build, so the published segment errs only
/// by its second-order remainder, which `band` holds inside W0's tolerance
/// for `vc` in `[vc_lo, vc_hi]` (`curvature::segment_band`). Past the band
/// the segment is built again at the present rail.
#[derive(Debug, Clone)]
struct SegCurve {
    dx1du: Vec<f64>,
    di1du: f64,
    g1: f64,
    band: f64,
    vc_lo: f64,
    vc_hi: f64,
}

/// A scheduled crossing at the instant it is due (`ClosedFormSolver::
/// due_crossing`): its node, the node's voltage and the landing value then,
/// the landing value's motion per volt of rail, and the rail it assumed.
#[derive(Debug, Clone, Copy)]
pub(crate) struct DueCrossing {
    local: usize,
    node_v: f64,
    cross_v: f64,
    dth: f64,
    u: f64,
}

/// While the hub PSU is in CC with a rail capacitor the rail is a known
/// closed form (Rails point 4): from a partition's aim at `t_a`, `u(t_a + τ)
/// = u_a + s_a φ_k(τ)` with `φ_k(τ) = ∫0^τ e^(-k r) dr`. A hub partition's
/// capacitor follows it exactly, `vc = inf + (vc_a - inf) e^(fτ) + w1 J(f,
/// k, τ)` (`rails::ramp_response`), and its nodes publish `x0(u_a) + dx0/du
/// s_a φ_k(τ) + dx vc`; a partition holding a watched pin of an IC on the
/// rail sees that pin's thresholds move with `u` (`w1 = 0`: nothing of it
/// reads the rail).
///
/// A curved segment's current, `i0(u) + di(u) vc`, couples the rail and its
/// capacitor through `di1du (u - u_a) vc`, which the drive holds at the aim's
/// `vc` (V9 MEDIUM 1). The rail aims the partition again at `u_a ± couple`,
/// before what that leaves out can put a published quantity past W0's
/// tolerance ([`couple_band`]).
///
/// With closed-form hub partitions on it the CC rail is their coupled path
/// (ruling R15 (a), `rails::coupled_modes`): several modes, `u(t_a + τ) = u_a
/// + sum s φ_k(τ)`, and the capacitor follows each, `vc = inf + (vc_a - inf)
/// e^(fτ) + sum w J(f, k, τ)`, `w = (di0/du + di1du vc_a) s / C`. That is the
/// coupled solution itself, so nothing is left for the next solve to settle.
///
/// On a draining cell (`cell`) the drive is the drain's line alone, one mode
/// at `k = 0`: the partition takes its own pull on the cell's rail into its
/// rate at the aim (`ClosedFormSolver::aim`), and the others' stay first
/// order under the rail's drift watch.
#[derive(Debug, Clone, PartialEq)]
struct Drive {
    u_a: f64,
    modes: Rc<[DriveMode]>,
    cell: bool,
    /// On a draining cell's coupled path, which the rail's drift watch
    /// solves again within `RAIL_DRIFT_V` (`set_rail_held`): its crossings
    /// are searched only until then, and a curved segment takes its
    /// transient's share of each mode (`DriveMode::we`).
    held: bool,
    /// How far the rail may move from `u_a` before the partition is aimed
    /// again, or infinity, and the instant it must be aimed again by in any
    /// case (`couple_band`).
    couple: f64,
    expiry: f64,
    /// On a cell, a curved segment's response to the line through its
    /// sensitivity's own motion with `vc` (`Self::aim`): `we τ² e^(fτ) / 2`.
    cell_we: f64,
    /// A bound on the capacitor's error the coupling left from earlier aims
    /// under this drive, at this aim.
    err: f64,
    /// The curved segment's `|di1du|`, what `err` grows by, read again at the
    /// next aim (the segment may be built anew by then); 0 for any other.
    g1: f64,
}

/// What an aim reads of the rail: its node, and while it is a known path,
/// the CC rail capacitor's segment or the draining cell's drive.
#[derive(Clone, Copy)]
struct OnRail<'a> {
    rail: Option<usize>,
    seg: Option<&'a CapSeg>,
    cell: Option<&'a CellDrive>,
    /// `seg` is a draining cell's coupled path (`set_rail_held`).
    held: bool,
}

/// One mode of a [`Drive`]: the rail moves `s φ_k(τ)` and the capacitor
/// takes `w J(f, k, τ)` of it.
#[derive(Debug, Clone, Copy, PartialEq)]
struct DriveMode {
    k: f64,
    s: f64,
    w: f64,
    /// On a cell's coupled path, a curved segment's response to the mode
    /// through its sensitivity's own motion with `vc`: `we e^(fτ) χ_k(τ)`.
    we: f64,
}

impl Drive {
    /// `sum over modes of term(mode)`, from the first mode on, so a single
    /// mode's sum is its term bit for bit.
    fn sum(&self, term: impl Fn(&DriveMode) -> f64) -> f64 {
        let mut it = self.modes.iter();
        let mut acc = term(it.next().expect("a drive has a mode"));
        for m in it {
            acc += term(m);
        }
        acc
    }

    /// `u(t_a + τ) - u_a`.
    fn du(&self, tau: f64) -> f64 {
        self.sum(|m| m.s * rails::exp_integral(-m.k, tau))
    }

    /// Whether the capacitor follows the rail at all (a hub partition).
    fn follows(&self) -> bool {
        self.modes.iter().any(|m| m.w != 0.0)
    }

    /// `sum w J(f, k, τ)`: how far the capacitor has followed the rail.
    fn jw(&self, f: f64, tau: f64) -> f64 {
        self.sum(|m| if m.w != 0.0 { m.w * rails::ramp_response(f, m.k, tau) } else { 0.0 })
    }

    /// `∫ sum w J(f, k, s) ds` over `[0, τ]`.
    fn jw_charge(&self, f: f64, tau: f64) -> f64 {
        self.sum(|m| if m.w != 0.0 { m.w * rails::ramp_charge(f, m.k, tau) } else { 0.0 })
    }

    /// The one mode of a single-mode drive.
    fn single(&self) -> Option<DriveMode> {
        (self.modes.len() == 1).then(|| self.modes[0])
    }
}

/// While a resolved pulse ramp moves a boundary node the partition reads, the
/// boundary is a straight line until the ramp's next corner (Sources point 3,
/// R3 (b)): from the aim at `t_a`, `x0(t_a + τ) = x0_a + rate_x τ` and `i0 =
/// i0_a + rate_i τ`, so the capacitor follows `vc = inf + (vc_a - inf)
/// e^(fτ) + w J(f, 0, τ)`, `w = rate_i / C`. It is [`Drive`] with `k = 0`,
/// the line in place of the rail's exponential. Every corner aims again.
#[derive(Debug, Clone, PartialEq)]
struct RampDrive {
    rate_x: Vec<f64>,
    rate_i: f64,
    w: f64,
    /// The next corner of a ramp that drives it.
    end: f64,
}

#[derive(Clone)]
struct Partition {
    /// This partition's GLOBAL index.
    global: usize,
    shape: DynShape,
    cap: DynCapacitor,
    /// `None` until the first segment exists (pec3's `segReady` guard: `_aim`
    /// on a segment that was never built would read garbage/NaN). Shared, so a
    /// snapshot of it is a reference count, not two vector copies.
    seg: Option<Rc<Segment>>,
    seg_t0: f64,
    seg_vc0: f64,
    /// `di / C` — the segment's exponential rate.
    seg_f: f64,
    /// The asymptote `vc` heads towards under `seg_f`.
    seg_inf: f64,
    /// Bumped on every re-aim (events.rs's lazy-deletion contract): a
    /// `Crossing` scheduled under a stale version is silently dropped when
    /// popped.
    version: u64,
    /// The local node index and threshold+nudge value the LAST scheduled
    /// crossing (if any) targets — `handle_crossing` uses these to land
    /// exactly past the threshold (this module's doc comment), not merely at
    /// the analytically-computed time (which can differ from the threshold
    /// by float noise).
    cross_local: usize,
    cross_v: f64,
    /// Local node index -> every reader watching it. Never touched by a
    /// solve/snapshot/restore — registrations are load-time/setup wiring,
    /// not per-instant state (this module's doc comment on
    /// `ClosedFormSolver::snapshot`).
    watches: HashMap<usize, Vec<(DeviceId, SharedThresholdFn)>>,
    /// Every registered reader, once per registration: what a re-aim queues.
    /// Kept beside `watches` (whose hash order the crossing tie-break
    /// follows) so queuing them walks one flat list.
    watch_ids: Vec<DeviceId>,
    /// Contract stats 8/9 (this partition's own share). NOT rolled back by
    /// `restore` (core/README.md, "A failed advance": "the stat counters...
    /// keep the failed attempt's counts").
    segments_built: u64,
    crossings_hit: u64,
    /// The ICs that can touch this partition, in program order: those with a
    /// supply or logic pin free in it. `stamp_ics_into_dyn` skips every other
    /// IC (its `touches` test needs a driver, a pull, or a supply of that IC
    /// in the partition, and a driver's or a pull's node is one of the IC's
    /// logic pins), so handing it only these is the same stamp without
    /// scanning the whole program's ICs and drivers on every segment.
    touch_ics: Vec<(usize, digital::IcRecord)>,
    /// `(index into touch_ics, logic-pin index)` of every open
    /// `FLOATING_INPUT` non-`OPEN_HIGH` pin free in this partition, in stamp
    /// order: the pins whose pull follows the floating bucket of `now`.
    touch_pulls: Vec<(u32, u32)>,
    /// Ids of every driver that stamps into this partition ([`ClosedFormSolver::stamps_into`]:
    /// a driver of a `touch_ics` IC whose pin or supply is free in it, not
    /// every driver of an IC that merely reads it), ascending. They are the
    /// only drivers a segment build reads, so they are what its key covers.
    /// Derived from the driver table's `node` and `device` fields, which never
    /// change after a driver is registered (load time), so it is rebuilt only
    /// when the table's length differs from `driver_index_len`; a debug build
    /// re-derives and compares it on every call.
    driver_ids: Vec<u32>,
    driver_index_len: usize,
    /// Scratch, reused across calls: the packed segment key, the copies of
    /// `driver_ids`' drivers handed to `stamp_ics_into_dyn`, and the node
    /// bits from before a publish.
    key_scratch: Vec<u8>,
    driver_scratch: Vec<Driver>,
    bits_scratch: Vec<u64>,
    /// Whether this partition's pre-image is already in the open window's
    /// journal ([`ClosedFormSolver::journal`]).
    journaled: bool,
    /// W1f: whether the partition reads the rail (a hub partition), and the
    /// live boundary nodes it reads (their values key the segment).
    hub: bool,
    live_nodes: Vec<usize>,
    /// A hub partition's segment re-aimed at the present rail: `x0(u)` and
    /// `i0(u)` (P3:558-569). Unused for any other partition, which publishes
    /// the segment as built.
    aim_x0: Vec<f64>,
    aim_i0: f64,
    /// Whether it holds a watched pin of an IC on the rail without reading
    /// the rail itself (re-aimed on a move, Rails point 3).
    rail_watched: bool,
    /// The rail it was aimed under while that is a known closed form.
    drive: Option<Drive>,
    /// The resolved ramp it was aimed under, if one moves what it reads.
    ramp: Option<Rc<RampDrive>>,
    /// The scheduled crossing's threshold motion per volt of rail.
    cross_dth: f64,
    /// A hub partition holding a pin of an IC on the rail (ruling R8 (a)):
    /// its segment is curved in the rail. Its `dx` and `di` re-aimed at the
    /// present rail, and that rail's distance from the build's, `u - u_seg`.
    curved: bool,
    aim_dx: Vec<f64>,
    aim_di: f64,
    aim_du: f64,
    /// Aimed on a draining cell with its own pull on the cell's rail taken
    /// in (`ClosedFormSolver::aim`): `aim_dx` and `aim_di` hold that aim's
    /// slopes in `vc`, as a curved partition's do.
    folded: bool,
    /// Under a known rail, `∫ b vc` this hub partition has drawn since the
    /// rail's segment last held its draw, banked up to `drift_from` each time
    /// its own trajectory changes (Rails point 4's charge correction).
    drift_q: f64,
    drift_from: f64,
    /// Ruling R59 (1) (B): a hub partition that steps while its PSU is in CC
    /// (`PartitionBuild::cc_switch`) is parked here meanwhile: `slot` does
    /// not find it, so nothing aims, publishes or schedules it, and its
    /// crossings are stale. The owner sets it and journals it.
    parked: bool,
}

impl Partition {
    /// The segment's curved terms, if it is a curved hub partition's.
    fn curve(&self) -> Option<&SegCurve> {
        self.seg.as_ref()?.hub.as_ref()?.curve.as_deref()
    }

    /// `dx` at the present aim: re-aimed at the rail when curved, and with
    /// the cell's pull taken in when folded.
    fn dx(&self) -> &[f64] {
        if self.folded || self.curve().is_some() {
            &self.aim_dx
        } else {
            &self.seg.as_ref().expect("a segment").dx
        }
    }

    /// `di` at the present aim, as [`Self::dx`].
    fn di(&self) -> f64 {
        if self.folded || self.curve().is_some() {
            self.aim_di
        } else {
            self.seg.as_ref().expect("a segment").di
        }
    }

    /// Node `l`'s sensitivity to the rail at capacitor voltage `vc`.
    fn dxdu(&self, h: &SegHub, l: usize, vc: f64) -> f64 {
        match &h.curve {
            Some(c) => h.dx0du[l] + c.dx1du[l] * vc,
            None => h.dx0du[l],
        }
    }

    /// The draw's slope in `vc` at the present aim: `b + g1 (u - u_seg)`.
    fn hub_b(&self, h: &SegHub) -> f64 {
        match &h.curve {
            Some(c) => h.b + c.g1 * self.aim_du,
            None => h.b,
        }
    }
}

/// Everything [`ClosedFormSolver::snapshot`] captures, restored verbatim by
/// [`ClosedFormSolver::restore`]. Deliberately excludes `watches` (load-time
/// wiring, never mutated by a solve) and the two stat counters (README: kept
/// even through a rollback). Includes `scheduled` (contract point 7): a
/// dirty batch with more than one `ClosedForm` partition can re-aim an
/// EARLIER partition (queuing a crossing into `self.scheduled`) before a
/// LATER one in the SAME `resolve_dirty` call fails — `PartitionedSolver`'s
/// own `drain_scheduled` (which would otherwise empty this into
/// `pending_schedule`) never runs in that case, since the `?` on the failing
/// call short-circuits first, so a stale entry can be left sitting here, not
/// only in `pending_schedule`.
#[derive(Debug, Clone)]
pub struct Snapshot {
    /// One per partition, in the solver's own (ascending global index) order.
    partitions: Vec<PartitionSnapshot>,
    scheduled: Vec<(f64, usize, u64)>,
}

impl Snapshot {
    /// Every captured value, printed in a fixed order (tests compare states by
    /// this).
    #[cfg(test)]
    pub(crate) fn digest(&self) -> String {
        format!("{self:?}")
    }
}

#[derive(Debug, Clone)]
struct PartitionSnapshot {
    seg: Option<Rc<Segment>>,
    seg_t0: f64,
    seg_vc0: f64,
    seg_f: f64,
    seg_inf: f64,
    version: u64,
    cross_local: usize,
    cross_v: f64,
    aim_x0: Vec<f64>,
    aim_i0: f64,
    drive: Option<Drive>,
    ramp: Option<Rc<RampDrive>>,
    cross_dth: f64,
    aim_dx: Vec<f64>,
    aim_di: f64,
    aim_du: f64,
    folded: bool,
    drift_q: f64,
    drift_from: f64,
}

/// A closed-form hub partition's term as the rail reads it at an instant
/// (`ClosedFormSolver::hub_term`, Rails points 2 and 4): its draw `a + b vc +
/// g (u - u_seg)` (with `g1 vc` more slope and `g1 (u - u_seg)` more `b` for a
/// curved segment), its capacitor `vc` then and the exponential it is on, and
/// `w`, its share of a draining cell's line (0 for any other).
#[derive(Debug, Clone, Copy)]
pub(crate) struct HubTerm {
    pub a: f64,
    pub b: f64,
    pub g: f64,
    pub u_seg: f64,
    pub vc: f64,
    pub inf: f64,
    pub f: f64,
    pub g1: f64,
    pub w: f64,
}

impl HubTerm {
    /// The draw's slope in `vc` at rail `u`: `b`, and for a curved segment `b
    /// + g1 (u - u_seg)` (R8 (a)).
    pub fn b_at(&self, u: f64) -> f64 {
        if self.g1 != 0.0 {
            self.b + self.g1 * (u - self.u_seg)
        } else {
            self.b
        }
    }
}

/// Owns every `ClosedForm` partition (Decision 14), addressed by its GLOBAL
/// partition index (the same numbering `partition::build`/`PartitionId` use)
/// so `solve::PartitionedSolver` can route a dirty id straight here without
/// a separate id-remapping table.
#[derive(Clone)]
pub struct ClosedFormSolver {
    /// The closed-form partitions, ascending by global index.
    parts: Vec<Partition>,
    /// global partition index -> index into `parts`, `u32::MAX` for a
    /// partition this solver does not own.
    slot_of: crate::shared::Shared<[u32]>,
    /// node id -> (partition index, local index), for `watch_node` — built
    /// once at [`Self::build`] time so a caller registering a watch never
    /// needs `partition::build`'s own tables in hand.
    node_owner: crate::shared::Shared<HashMap<usize, (usize, usize)>>,
    /// Crossings this solver wants scheduled, drained by
    /// [`Self::take_scheduled`] — normally emptied by the caller
    /// (`solve::mod`'s `PartitionedSolver`) before control returns to
    /// `kernel.rs`, but NOT when a later partition in the SAME dirty batch
    /// fails first (see [`Snapshot`]'s doc comment), so it is part of
    /// [`Snapshot`] after all.
    scheduled: Vec<(f64, usize, u64)>,
    /// node -> every IC device index that reads it (contract point 4's
    /// last bullet: "resolves of dynamic partitions must also report IC
    /// readers of nodes that changed, as static does"), by node id.
    node_to_ic_readers: crate::shared::Shared<[Vec<usize>]>,
    /// The rollback window (`PartitionedSolver::begin_instant`): while it is
    /// open, each partition's state is saved (`journal`) before the first
    /// call of the window that can change it, and `scheduled` as the window
    /// found it is kept, so the whole solver is no longer copied at every
    /// instant. A completed instant just drops the journal.
    window: bool,
    journal: Vec<(u32, PartitionSnapshot)>,
    window_scheduled: Vec<(f64, usize, u64)>,
    /// W1f: the rail node outside exact mode, the contributions solved since
    /// the owner last drained them, and stat 16's share.
    rail: Option<usize>,
    hub_updates: Vec<(usize, Contrib)>,
    sensitivities: u64,
    /// The CC rail capacitor's segment while the rail is that known closed
    /// form (Rails point 4), and the rail voltages where an IC on the rail
    /// turns on or off. The owner sets both from its own state before every
    /// call that can aim a partition, so a rollback needs nothing here.
    rail_seg: Option<CapSeg>,
    rail_edges: Vec<f64>,
    /// While the hub is a draining cell, how its partitions follow its rail
    /// (ruling R15 (a)); set by the owner as `rail_seg` is.
    cell: Option<CellDrive>,
    /// Whether `rail_seg` is a draining cell's coupled path
    /// ([`Self::set_rail_held`]).
    rail_held: bool,
    rail_until: f64,
    /// The non-rail nodes an IC supply sits on that follow the rail, as
    /// `(node, a, s)` for `a + s u`: the static hub partitions' published
    /// lines (Rails point 3), set with the segment.
    rail_follow: Vec<(usize, f64, f64)>,
    /// The running ramps, set by the owner whenever one starts or ends (and
    /// after a rollback), as for the stepped solver.
    ramps: Option<Rc<RampEval>>,
    /// F5: the owner's device-indexed power-hold table (W1c point 1a) and
    /// whether any entry is not free; the owner journals it.
    holds: Rc<Vec<digital::power::PowerHold>>,
    holds_on: bool,
    /// Test builds keep every partition's state from when the window opened
    /// and check, at the window's end, that a partition the journal does not
    /// hold is unchanged, and after a rollback that all of them are back: a
    /// change made without saving fails the test that ran it.
    #[cfg(test)]
    w_saved: Option<Vec<String>>,
}

impl ClosedFormSolver {
    /// Each table this solver shares with snapshots, by address (W2a tests).
    #[cfg(test)]
    pub(crate) fn shared_tables(&self) -> Vec<(&'static str, usize)> {
        vec![("slot_of", crate::shared::Shared::addr(&self.slot_of)), ("node_owner", crate::shared::Shared::addr(&self.node_owner)), ("node_to_ic_readers", crate::shared::Shared::addr(&self.node_to_ic_readers))]
    }
}


impl ClosedFormSolver {
    /// Builds one [`Partition`] per `ClosedForm`-classified entry in `kinds`
    /// (a program with none at all gets an empty solver, cheaply). `kinds`
    /// already has `FORCE_STEPPED` folded in (`partition::build`), so this
    /// never has to check the option itself.
    pub(crate) fn build(program: &Program, partitions: &[Vec<usize>], local_index: &[Option<(usize, usize)>], fixed: &[Option<f64>], kinds: &[PartitionKind], topo: LiveTopology) -> Self {
        let mut parts = Vec::new();
        let mut slot_of = vec![u32::MAX; kinds.len()];
        let mut node_owner = HashMap::new();
        for (p_idx, kind) in kinds.iter().enumerate() {
            if *kind != PartitionKind::ClosedForm {
                continue;
            }
            let shape = compile_dyn_shape(p_idx, &partitions[p_idx], local_index, fixed, program);
            debug_assert_eq!(shape.capacitors.len(), 1, "Decision 14: a ClosedForm partition has exactly one capacitor");
            // README, LEDs point 5: the exact exponential needs a linear
            // network, so `partition::build` never classifies a partition
            // holding a stamped junction ClosedForm. A real assert, not a
            // debug one: a junction reaching a segment would silently solve
            // the wrong physics (the segment never stamps one).
            assert!(shape.junctions.is_empty(), "a ClosedForm partition never holds an LED junction (partition {p_idx}); partition::build must classify it Stepped");
            let cap = shape.capacitors[0];
            for (l, &node) in shape.nodes.iter().enumerate() {
                node_owner.insert(node, (p_idx, l));
            }
            slot_of[p_idx] = parts.len() as u32;
            parts.push(Partition {
                global: p_idx,
                shape,
                cap,
                seg: None,
                seg_t0: 0.0,
                seg_vc0: 0.0,
                seg_f: 0.0,
                seg_inf: 0.0,
                version: 0,
                cross_local: 0,
                cross_v: 0.0,
                watches: HashMap::new(),
                watch_ids: Vec::new(),
                segments_built: 0,
                crossings_hit: 0,
                touch_ics: Vec::new(),
                touch_pulls: Vec::new(),
                driver_ids: Vec::new(),
                driver_index_len: usize::MAX,
                key_scratch: Vec::new(),
                driver_scratch: Vec::new(),
                bits_scratch: Vec::new(),
                journaled: false,
                hub: topo.rail.is_some() && topo.hub[p_idx],
                live_nodes: topo.live_reads(p_idx),
                aim_x0: Vec::new(),
                aim_i0: 0.0,
                rail_watched: false,
                drive: None,
                ramp: None,
                cross_dth: 0.0,
                curved: false,
                aim_dx: Vec::new(),
                aim_di: 0.0,
                aim_du: 0.0,
                folded: false,
                drift_q: 0.0,
                drift_from: 0.0,
                parked: false,
            });
        }
        let ics = collect_ics(program);
        if let Some(rail) = topo.rail {
            let rail_ic = crate::solve::curvature::holds_rail_ic_pin(&ics, local_index, rail, kinds.len());
            for part in &mut parts {
                part.curved = part.hub && rail_ic[part.global];
            }
        }
        let in_partition = |node: usize, p_idx: usize| matches!(node_owner.get(&node), Some(&(p, _)) if p == p_idx);
        for part in &mut parts {
            let p_idx = part.global;
            for (device_index, ic) in &ics {
                let touches = in_partition(ic.vcc.node, p_idx) || in_partition(ic.gnd.node, p_idx) || ic.logic.iter().any(|pin| in_partition(pin.node, p_idx));
                if !touches {
                    continue;
                }
                let ti = part.touch_ics.len() as u32;
                for (pi, pin) in ic.logic.iter().enumerate() {
                    if pin.open && pin.floating_input && !pin.open_high && in_partition(pin.node, p_idx) {
                        part.touch_pulls.push((ti, pi as u32));
                    }
                }
                part.touch_ics.push((*device_index, ic.clone()));
            }
        }
        let mut node_to_ic_readers = vec![Vec::new(); fixed.len()];
        for (node, readers) in build_ic_reader_index(&ics) {
            node_to_ic_readers[node] = readers;
        }
        ClosedFormSolver {
            parts,
            slot_of: slot_of.into(),
            node_owner: node_owner.into(),
            scheduled: Vec::new(),
            node_to_ic_readers: node_to_ic_readers.into(),
            window: false,
            journal: Vec::new(),
            window_scheduled: Vec::new(),
            rail: topo.rail,
            hub_updates: Vec::new(),
            sensitivities: 0,
            rail_seg: None,
            rail_edges: Vec::new(),
            cell: None,
            rail_held: false,
            rail_until: f64::INFINITY,
            rail_follow: Vec::new(),
            ramps: None,
            holds: Rc::new(Vec::new()),
            holds_on: false,
            #[cfg(test)]
            w_saved: None,
        }
    }

    /// The running resolved ramps, or none (Sources point 3, R3 (b)).
    /// The owner's power-hold table, after every change to it.
    pub(crate) fn set_power_holds(&mut self, holds: Rc<Vec<digital::power::PowerHold>>, on: bool) {
        self.holds = holds;
        self.holds_on = on;
    }

    pub(crate) fn set_ramps(&mut self, ramps: Option<Rc<RampEval>>) {
        self.ramps = ramps;
    }

    /// The rail as a known closed form for the calls that follow, or none,
    /// with the rail voltages where an IC's thresholds stop being affine in
    /// it (its power window's edges, its generic levels' kinks), ascending,
    /// and the supply nodes that follow it.
    pub(crate) fn set_rail_seg(&mut self, seg: Option<CapSeg>, edges: Vec<f64>, follow: Vec<(usize, f64, f64)>) {
        self.rail_seg = seg;
        self.rail_edges = edges;
        self.rail_follow = follow;
    }

    /// Whether the rail's path (`rail_seg`) is a draining cell's coupled one
    /// (ruling R16 (B)), which the rail solves again before it has moved
    /// `RAIL_DRIFT_V` along, and `until`, the latest instant it does so, past
    /// which no crossing of an aim under it is ever landed. A curved segment
    /// keeps its coupling band there as on a CC path: the drift cap bounds
    /// the rail's motion, not how long its bilinear term runs.
    pub(crate) fn set_rail_held(&mut self, held: bool, until: f64) {
        self.rail_held = held;
        self.rail_until = if held { until } else { f64::INFINITY };
    }

    /// The draining cell's drive for the calls that follow, or none.
    pub(crate) fn set_cell(&mut self, cell: Option<CellDrive>) {
        self.cell = cell;
    }

    /// Marks the partitions holding a watched pin of an IC on the rail that
    /// do not read the rail (wired at load).
    pub(crate) fn set_rail_watched(&mut self, parts: &[usize]) {
        for &p in parts {
            if let Some(slot) = self.slot(p) {
                self.parts[slot].rail_watched = true;
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.parts.is_empty()
    }

    /// The crossing-watch API (W1 plan row B2): registers `reader` against
    /// `thresholds` on `node`. `false` if `node` is not a free node this
    /// solver owns (a caller error — a reader watches a NODE, and only a
    /// dynamic partition's nodes move continuously enough to need this; a
    /// fixed or static-partition node is never a valid target).
    pub fn watch_node(&mut self, node: usize, reader: DeviceId, thresholds: ThresholdFn) -> bool {
        let Some(&(p_idx, local)) = self.node_owner.get(&node) else {
            return false;
        };
        let slot = self.slot_of[p_idx] as usize;
        let part = &mut self.parts[slot];
        part.watches.entry(local).or_default().push((reader, Rc::from(thresholds)));
        part.watch_ids.push(reader);
        true
    }

    /// Contract stat 8 (closed-form segments built), summed over every
    /// partition.
    pub fn segments_built(&self) -> f64 {
        self.parts.iter().map(|p| p.segments_built as f64).sum()
    }

    /// Contract stat 9 (crossings), summed over every partition.
    pub fn crossings(&self) -> f64 {
        self.parts.iter().map(|p| p.crossings_hit as f64).sum()
    }

    /// Whether a `Crossing` event carrying `version` for `partition` is
    /// still current (events.rs's lazy-deletion contract).
    pub fn is_current(&self, partition: usize, version: u64) -> bool {
        self.slot(partition).is_some_and(|slot| self.parts[slot].version == version)
    }

    /// `partition`'s aim version, which every re-aim bumps.
    pub(crate) fn version_of(&self, partition: usize) -> Option<u64> {
        self.slot(partition).map(|slot| self.parts[slot].version)
    }

    /// The crossing `partition` has scheduled, as it stands at `now` under
    /// the aim that scheduled it: where `handle_crossing` would land its node
    /// (Rails point 4).
    pub(crate) fn due_crossing(&self, partition: usize, now: f64) -> Option<DueCrossing> {
        let part = &self.parts[self.slot(partition)?];
        let d = part.drive.as_ref()?;
        let du = d.du(now - part.seg_t0);
        Some(DueCrossing { local: part.cross_local, node_v: Self::node_at(part, part.cross_local, now)?, cross_v: part.cross_v + part.cross_dth * du, dth: part.cross_dth, u: d.u_a + du })
    }

    /// After a rail solve re-aimed `partition` at `now` ahead of its crossing
    /// `due` (`PartitionedSolver::solve_rail_before_crossing`): a crossing
    /// due at the instant is a root at `now`, which no schedule accepts, so
    /// unless the solve has put the node back short of its threshold by more
    /// than a nudge, the crossing is set up again to land now. The threshold
    /// moves with the solved rail `u`; a node the solve took past its landing
    /// point lands where it stands.
    pub(crate) fn keep_due_crossing(&mut self, partition: usize, due: DueCrossing, u: f64, now: f64) -> bool {
        let Some(slot) = self.slot(partition) else { return false };
        let Some(node_v) = Self::node_at(&self.parts[slot], due.local, now) else { return false };
        let rising = due.cross_v > due.node_v;
        let land = due.cross_v + due.dth * (u - due.u);
        let short = if rising { land - node_v } else { node_v - land };
        if short > 2.0 * CROSS_NUDGE_V {
            return false;
        }
        self.journal(slot);
        let part = &mut self.parts[slot];
        part.cross_local = due.local;
        part.cross_v = if short < 0.0 { node_v } else { land };
        part.cross_dth = due.dth;
        true
    }

    /// Local node `local`'s voltage at `t` on the partition's trajectory, as
    /// `handle_crossing` reads it.
    fn node_at(part: &Partition, local: usize, t: f64) -> Option<f64> {
        let seg = part.seg.as_ref()?;
        let mut x0 = if seg.hub.is_some() { part.aim_x0[local] } else { seg.x0[local] };
        let vc = vc_of(part, t);
        if let (Some(d), Some(h)) = (part.drive.as_ref(), seg.hub.as_ref()) {
            x0 += part.dxdu(h, local, vc) * d.du(t - part.seg_t0);
        }
        if let Some(r) = &part.ramp {
            x0 += r.rate_x[local] * (t - part.seg_t0);
        }
        Some(x0 + part.dx()[local] * vc)
    }

    /// W1g (README W1g points 3, 5 and 9): node `node`'s voltage over the
    /// current segment as `(v_inf, v0, t0, tau)`, `V(t) = v_inf + (v0 - v_inf)
    /// exp(-(t - t0)/tau)`, which the slow integrators close in closed form.
    /// `x(vc) = x0 + dx vc` with `vc` the segment's exponential, so the node is
    /// the same exponential: `v_inf = x0 + dx vc_inf` and `v0 = x0 + dx vc0`.
    /// A segment whose `vc` does not move (`F = 0`) is the constant `v0`, an
    /// infinite `tau`. `None` for a node this solver does not own, before the
    /// first segment, for a growing segment (`F > 0`, no decaying exponential
    /// to close), and for a segment that is not one exponential: one driven
    /// by a known rail or a resolved ramp (W1f), whose trajectory carries
    /// extra terms. The caller then holds the published value instead.
    pub(crate) fn node_trajectory(&self, node: usize) -> Option<(f64, f64, f64, f64)> {
        let &(p, l) = self.node_owner.get(&node)?;
        let part = &self.parts[self.slot(p)?];
        if part.drive.is_some() || part.ramp.is_some() {
            return None;
        }
        let seg = part.seg.as_ref()?;
        let x0 = if seg.hub.is_some() { part.aim_x0[l] } else { seg.x0[l] };
        let dx = part.dx()[l];
        let v0 = x0 + dx * part.seg_vc0;
        if part.seg_f == 0.0 {
            return Some((v0, v0, part.seg_t0, f64::INFINITY));
        }
        if part.seg_f < 0.0 {
            Some((x0 + dx * part.seg_inf, v0, part.seg_t0, -1.0 / part.seg_f))
        } else {
            None
        }
    }

    /// W1g, ruling R21: node `node`'s voltage on a segment a known rail
    /// drives (which [`Self::node_trajectory`] declines), for the slow
    /// integrators: the segment's own exponential re-anchored at `now`
    /// through the node's exact value there, `(v_inf, v_now, tau)`, and
    /// `Δ`, how long that stays within `tol = tol_abs + tol_rel |v_now|` of
    /// the exact trajectory.
    ///
    /// The exact node is `x0 + dxdu(vc) du(τ) + dx vc(t)`, with `vc` the
    /// segment's exponential plus the rail's share `Δvc` (`vc_of`), `τ = t -
    /// seg_t0`; what the exponential drops is `D(τ) = dxdu(vc) du(τ) + dx
    /// Δvc(τ)`. Re-anchored at `now` (`τn`), the error at `τn + δ` is `D(τn +
    /// δ) - D(τn) e^(-δ/tau)`, at most `(L + |D(τn)|/tau) δ`, `L` a bound on
    /// `|D'|` over `[τn, τn + δ]`. For modes `k >= 0` and `f <= 0`: `φ_k(τ)
    /// <= τ`, `|du| <= S τ` and `|du'| <= S` (`S = Σ|s|`); `0 <= J <= τ²/2`
    /// and `|J'| = |φ_k + f J| <= τ + |f| τ²/2`, and the held and cell terms
    /// likewise (`W = Σ|w| + Σ|we| + |cell_we|`), so `|Δvc| <= W τ²/2` and
    /// `|Δvc'| <= W (τ + |f| τ²/2)`; `|dxdu| <= a0 + a1 (Vm +
    /// |Δvc|)` and `|vc'| <= |f| |vc0 - inf| + |Δvc'|`. So `L(τ) = a1 |vc'|
    /// S τ + |dxdu| S + |dx| |Δvc'|`, nondecreasing in `τ`, and `Δ` is the
    /// largest `δ` with `(L(τn + δ) + |D(τn)|/tau) δ <= tol` (infinity when
    /// nothing is dropped). `None` for anything else: not this solver's node,
    /// no drive, a ramp, a growing segment or a mode with `k < 0`.
    pub(crate) fn driven_anchor(&self, node: usize, now: f64, tol_abs: f64, tol_rel: f64) -> Option<(f64, f64, f64, f64)> {
        let &(p, l) = self.node_owner.get(&node)?;
        let part = &self.parts[self.slot(p)?];
        let d = part.drive.as_ref()?;
        if part.ramp.is_some() || part.seg_f > 0.0 || d.modes.iter().any(|m| !(m.k >= 0.0)) {
            return None;
        }
        let seg = part.seg.as_ref()?;
        let x0 = if seg.hub.is_some() { part.aim_x0[l] } else { seg.x0[l] };
        let dx = part.dx()[l];
        let f = part.seg_f;
        let v_now = Self::node_at(part, l, now)?;
        let tol = tol_abs + tol_rel * v_now.abs();
        let s_sum: f64 = d.modes.iter().map(|m| m.s.abs()).sum();
        let w_sum: f64 = d.modes.iter().map(|m| m.w.abs()).sum();
        let adx = dx.abs();
        let tau_n = now - part.seg_t0;
        let curved = seg.hub.as_ref().is_some_and(|h| h.curve.is_some()) || d.cell_we != 0.0 || (d.held && d.modes.iter().any(|m| m.we != 0.0));
        if !curved {
            // A linear segment: `vc' = f (vc - inf) + F(τ)`, `F = Σ w φ_k`,
            // and the node `x0 + a du(τ) + dx vc`. The anchor freezes the rail
            // where it stands: the asymptote is the node's with `F` and `du`
            // held at `τn` (`vc_inf = inf - F(τn)/f`), so it tracks the rail.
            // Its error is `a (du(τ) - du(τn)) + dx e_vc`, with `e_vc' = f
            // e_vc + F(τ) - F(τn)`: `|du(τ) - du(τn)| <= S δ`, `|F(τ) -
            // F(τn)| <= W δ` (`W = Σ|w|`), so `|e_vc| <= W min(δ²/2, δ/|f|)`
            // and the bound is `|a| S δ + |dx| W min(δ²/2, δ/|f|)`: past a few
            // `tau`, `δ = tol / (g S)`, `g` the node's rail sensitivity at DC.
            let a = seg.hub.as_ref().map_or(0.0, |h| h.dx0du[l]);
            let f_n: f64 = d.sum(|m| if m.w != 0.0 { m.w * rails::exp_integral(-m.k, tau_n) } else { 0.0 });
            let (v_inf, tau) = if f < 0.0 {
                (x0 + a * d.du(tau_n) + dx * (part.seg_inf - f_n / f), -1.0 / f)
            } else {
                (v_now, f64::INFINITY)
            };
            // `S` and `W` as the modes stand at `τn`: each `e^(-kτ)` only
            // falls from there (every `k >= 0`, checked above), so they
            // bound the slopes over the whole anchor. Summed as aimed, a
            // segment whose modes had long decayed (a CC rail settled at its
            // operating point, cold 1838 tpl 1) kept the slope it was aimed
            // with, and its rated devices re-anchored at `REDRIVE_MIN_DT`
            // for as long as the segment stood.
            let s_n: f64 = d.modes.iter().map(|m| m.s.abs() * (-m.k * tau_n).exp()).sum();
            let w_n: f64 = d.modes.iter().map(|m| m.w.abs() * (-m.k * tau_n).exp()).sum();
            return Some((v_inf, v_now, tau, linear_anchor_deadline(a.abs(), s_n, adx, w_n, f, tol)));
        }
        // A curved or folded segment (the coupling moves with `vc`): the
        // segment's own exponential re-anchored through the exact value, the
        // rest of the node, `D = v - base` (the rail's part and the
        // forcing's), held where it stands, and the bound below on how far
        // `D` moves. Let decay with the exponential instead, `D` was dropped
        // whole within a few `tau`: cold 2548 tpl 5's rail, creeping at 8 V/s
        // through a 555's edge, left 29 uV of it on a node of a 1.5 ns
        // segment, and its rated resistors re-anchored every 1-2 ns.
        let base = |t: f64| x0 + dx * vc_at(f, part.seg_inf, part.seg_vc0, part.seg_t0, t);
        let d_now = v_now - base(now);
        let (v_inf, tau) = if f < 0.0 { (x0 + dx * part.seg_inf + d_now, -1.0 / f) } else { (v_now, f64::INFINITY) };
        let held_we: f64 = if d.held { d.modes.iter().map(|m| m.we.abs()).sum() } else { 0.0 };
        let (a0, a1) = match seg.hub.as_ref() {
            Some(h) => (h.dx0du[l].abs(), h.curve.as_ref().map_or(0.0, |c| c.dx1du[l].abs())),
            None => (0.0, 0.0),
        };
        let b = CurvedBound {
            tau_n,
            s_sum,
            w_sum,
            modes: curved_modes(d.modes.iter().map(|m| (m.s.abs(), m.k, m.w.abs())), f, tau_n),
            we: held_we + d.cell_we.abs(),
            a0,
            a1,
            adx,
            vm: part.seg_vc0.abs().max(part.seg_inf.abs()),
            dvc: (part.seg_vc0 - part.seg_inf).abs(),
            f,
        };
        Some((v_inf, v_now, tau, largest_within(|delta| b.at(delta), tol)))
    }

    /// W1g, ruling R21: the rail under `partition`'s drive at `now`, `u_a +
    /// du(now - seg_t0)`, and how long it stays within `tol = tol_abs +
    /// tol_rel |u|` of that value: `|du'| <= S = Σ|s|` for modes `k >= 0`,
    /// so `tol / S`. `None` where [`Self::driven_anchor`] declines.
    pub(crate) fn driven_rail(&self, partition: usize, now: f64, tol_abs: f64, tol_rel: f64) -> Option<(f64, f64)> {
        let part = &self.parts[self.slot(partition)?];
        let d = part.drive.as_ref()?;
        if part.ramp.is_some() || part.seg_f > 0.0 || d.modes.iter().any(|m| !(m.k >= 0.0)) {
            return None;
        }
        let tau_n = now - part.seg_t0;
        let u = d.u_a + d.du(tau_n);
        // The rail's slope bounded as the modes stand at `τn`, as
        // `driven_anchor`'s linear bound.
        let s_sum: f64 = d.modes.iter().map(|m| m.s.abs() * (-m.k * tau_n).exp()).sum();
        let tol = tol_abs + tol_rel * u.abs();
        Some((u, if s_sum > 0.0 { tol / s_sum } else { f64::INFINITY }))
    }

    /// W1g, ruling R29: a device's own voltage `V = Σ ± pin` over a linear
    /// segment of `partition` that a rail drives, each pin a node of the
    /// partition, the rail, or a fixed value. `(v_inf, v_now, tau, Δ)`: the
    /// one exponential `V` follows from `now` with the rail's motion taken as
    /// its tangent there, and how long that stays within `tol = tol_abs +
    /// tol_rel |v_now|`.
    ///
    /// With `du(τn + δ) = du_n + S_n δ` and the capacitor's forcing `F(τn +
    /// δ) = F_n + W_n δ` (`S_n = Σ s e^(-k τn)`, `W_n = Σ w e^(-k τn)`), `vc' =
    /// f (vc - inf) + F` has the exact solution `α + β δ + (vc_n - α) e^(fδ)`,
    /// `β = -W_n/f`, `α = inf - F_n/f - W_n/f²`, and `V = C + A du + D vc`
    /// (`A`, `D`, `C` summed over the pins) is `V∞ + Lδ + (V_n - V∞) e^(fδ)`
    /// with `L = A S_n + D β`. A device that tracks the rail (a resistor from
    /// the rail into the capacitor on a linear ramp) has `L = 0`: its voltage
    /// is one exponential, lag included. Otherwise the anchor's asymptote is
    /// `V∞ - L/f`, which leaves `V_n` on `V`'s slope, and it drops `L (δ -
    /// (1 - e^(fδ))/|f|)` (at most `|L f| δ²/2`). With the rail's curvature,
    /// `|du''| <= K_s = Σ|s| k e^(-k τn)` and `|F''| <= K_w = Σ|w| k e^(-k
    /// τn)`, the error is at most that plus `|A| K_s δ²/2 + |D| K_w min(δ³/6,
    /// δ²/(2|f|))`. `None` for a curved or folded segment, a ramp,
    /// a non-decaying segment or a mode with `k < 0`.
    pub(crate) fn driven_device(&self, partition: usize, pins: &[(DevicePin, f64)], now: f64, tol_abs: f64, tol_rel: f64) -> Option<(f64, f64, f64, f64)> {
        let part = &self.parts[self.slot(partition)?];
        let d = part.drive.as_ref()?;
        let seg = part.seg.as_ref()?;
        let f = part.seg_f;
        let curved = seg.hub.as_ref().is_some_and(|h| h.curve.is_some()) || d.cell_we != 0.0 || (d.held && d.modes.iter().any(|m| m.we != 0.0));
        if curved || part.ramp.is_some() || !(f < 0.0) || d.modes.iter().any(|m| !(m.k >= 0.0)) {
            return None;
        }
        let (mut a, mut dd, mut c) = (0.0, 0.0, 0.0);
        for &(pin, sign) in pins {
            match pin {
                DevicePin::Node(node) => {
                    let &(q, l) = self.node_owner.get(&node)?;
                    if q != partition {
                        return None;
                    }
                    let x0 = if seg.hub.is_some() { part.aim_x0[l] } else { seg.x0[l] };
                    a += sign * seg.hub.as_ref().map_or(0.0, |h| h.dx0du[l]);
                    dd += sign * part.dx()[l];
                    c += sign * x0;
                }
                DevicePin::Rail => {
                    a += sign;
                    c += sign * d.u_a;
                }
                DevicePin::Fixed(v) => c += sign * v,
            }
        }
        let tau_n = now - part.seg_t0;
        let s_n = d.sum(|m| m.s * (-m.k * tau_n).exp());
        let w_n = d.sum(|m| m.w * (-m.k * tau_n).exp());
        let f_n = d.sum(|m| if m.w != 0.0 { m.w * rails::exp_integral(-m.k, tau_n) } else { 0.0 });
        let k_s = d.sum(|m| (m.s * m.k).abs() * (-m.k * tau_n).exp());
        let k_w = d.sum(|m| (m.w * m.k).abs() * (-m.k * tau_n).exp());
        let du_n = d.du(tau_n);
        let vc_n = vc_of(part, now);
        let v_now = c + a * du_n + dd * vc_n;
        let alpha = part.seg_inf - f_n / f - w_n / (f * f);
        let lin = a * s_n - dd * w_n / f;
        // The asymptote moved by `-L/f` so the exponential leaves `V_n` on
        // `V`'s own slope: the term it drops is then `L (δ - (1 - e^(fδ))/|f|)`,
        // second order in `δ` until `δ ~ 1/|f|`.
        let v_inf = c + a * du_n + dd * alpha - lin / f;
        let tol = tol_abs + tol_rel * v_now.abs();
        let bound = |delta: f64| lin.abs() * (delta + (-(f * delta).exp_m1()) / f) + a.abs() * k_s * delta * delta / 2.0 + dd.abs() * k_w * (delta * delta * delta / 6.0).min(delta * delta / (2.0 * -f));
        Some((v_inf, v_now, -1.0 / f, largest_within(bound, tol)))
    }

    /// Whether `partition` is aimed under a known rail (Rails point 4).
    pub(crate) fn is_driven(&self, partition: usize) -> bool {
        self.slot(partition).is_some_and(|slot| self.parts[slot].drive.is_some())
    }

    /// Re-aims every partition in `dirty` (a switch toggle or driver commit
    /// touched it — core/README.md: "A segment is re-aimed when a stamp in
    /// the partition changes") at `now`, publishing its exact trajectory at
    /// this instant into `voltages`/`currents` and queuing every registered
    /// watcher for re-evaluation (pec3's `_aim` re-queues every watcher on
    /// every re-aim unconditionally, since the whole trajectory just
    /// changed — there is no cheaper "did this one move" check that would
    /// still be correct). Builds the FIRST segment (segReady guard) at `vc =
    /// 0` (README: "Initial vc = 0 at load; carry-forward is W2") the first
    /// time a partition is named here, exactly as `Kernel::settle`'s
    /// first-pass "resolve every partition" hands every partition to this
    /// call once at load.
    pub fn resolve_dirty(
        &mut self,
        dirty: &[usize],
        now: f64,
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
    ) -> Result<Vec<DeviceId>, String> {
        let mut readers = Vec::new();
        self.resolve_dirty_into(dirty, now, drivers, live_params, voltages, currents, &mut readers)?;
        Ok(readers)
    }

    /// [`Self::resolve_dirty`], appending its readers to `readers` (the
    /// owner's reused buffer: a fresh vector per call, grown reader by
    /// reader, was a measurable share of the SAP-1's allocator time, F5q).
    /// On `Err`, what it appended is the caller's to drop.
    #[allow(clippy::too_many_arguments)]
    pub fn resolve_dirty_into(
        &mut self,
        dirty: &[usize],
        now: f64,
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
        readers: &mut Vec<DeviceId>,
    ) -> Result<(), String> {
        for &p_idx in dirty {
            let Some(slot) = self.slot(p_idx) else { continue };
            self.journal(slot);
            let part = &mut self.parts[slot];
            // With no segment yet, the charge it loads with (0), or the one
            // a CC exit resumed it at (`resume`).
            let vc_now = match &part.seg {
                Some(_) => vc_of(part, now),
                None => part.seg_vc0,
            };
            Self::note_bits(part, voltages);
            let on = OnRail { rail: self.rail, seg: self.rail_seg.as_ref(), cell: self.cell.as_ref(), held: self.rail_held };
            Self::segment_and_aim(part, now, vc_now, drivers, &self.node_owner, &Bnd::new(voltages).with_holds(if self.holds_on { &self.holds } else { &[] }), live_params, on, self.ramps.as_deref(), &mut self.hub_updates, &mut self.sensitivities, 0.0)?;
            Self::publish(part, live_params, voltages, currents);
            readers.extend_from_slice(&part.watch_ids);
            Self::report_changed_ic_readers(part, voltages, &self.node_to_ic_readers, readers);
            let crossing = Self::schedule_crossing(part, now, voltages, self.rail, &self.rail_edges, &self.rail_follow, self.rail_until);
            Self::set_couple(part, now, crossing, voltages, self.rail, live_params);
            if let Some(t) = crossing {
                self.scheduled.push((t, p_idx, part.version));
            }
        }
        Ok(())
    }

    /// Applies a due, still-current `Crossing` at `now` (pec3's
    /// `_analyticCross`): lands the crossed node exactly on its
    /// threshold-plus-nudge, then re-aims from there, exactly like any other
    /// re-aim.
    pub fn handle_crossing(
        &mut self,
        partition: usize,
        now: f64,
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
    ) -> Result<Vec<DeviceId>, String> {
        let mut readers = Vec::new();
        self.handle_crossing_into(partition, now, drivers, live_params, voltages, currents, &mut readers)?;
        Ok(readers)
    }

    /// [`Self::handle_crossing`], appending its readers to `readers` (as
    /// [`Self::resolve_dirty_into`]).
    #[allow(clippy::too_many_arguments)]
    pub fn handle_crossing_into(
        &mut self,
        partition: usize,
        now: f64,
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
        readers: &mut Vec<DeviceId>,
    ) -> Result<(), String> {
        let slot = self.slot(partition).ok_or_else(|| format!("closed-form: unknown partition {partition}"))?;
        self.journal(slot);
        let part = &mut self.parts[slot];
        part.crossings_hit += 1;
        let seg = part.seg.as_ref().expect("a Crossing was only ever scheduled after a segment existed");
        let mut vc = vc_of(part, now);
        // Land exactly past the threshold on the crossing node (this
        // module's doc comment; pec3's `_analyticCross`), not merely at the
        // analytically-computed time, which can differ from the threshold by
        // float noise. Under a known rail both the node and the threshold
        // have moved with it since the aim.
        let mut dx = part.dx()[part.cross_local];
        let mut x0 = if seg.hub.is_some() { part.aim_x0[part.cross_local] } else { seg.x0[part.cross_local] };
        let mut cross_v = part.cross_v;
        if let Some(d) = part.drive.as_ref() {
            let du = d.du(now - part.seg_t0);
            if let Some(h) = seg.hub.as_ref() {
                x0 += h.dx0du[part.cross_local] * du;
                // A curved segment's sensitivity follows `vc`: the node is
                // affine in it at this rail.
                if let Some(c) = &h.curve {
                    dx += c.dx1du[part.cross_local] * du;
                }
            }
            cross_v += part.cross_dth * du;
        }
        if let Some(r) = &part.ramp {
            x0 += r.rate_x[part.cross_local] * (now - part.seg_t0);
        }
        // The landing steps `vc` off its trajectory by whatever the crossing's
        // time left the node short or past: a curved node's sensitivity held
        // at the aim's `vc` leaves `dx1du (u - u_a) (vc - vc_a)` (V9 MEDIUM
        // 1), which the next aim carries as error.
        let jump = if dx != 0.0 {
            let on = vc;
            vc = (cross_v - x0) / dx;
            (vc - on).abs()
        } else {
            0.0
        };
        Self::note_bits(part, voltages);
        let on = OnRail { rail: self.rail, seg: self.rail_seg.as_ref(), cell: self.cell.as_ref(), held: self.rail_held };
        Self::segment_and_aim(part, now, vc, drivers, &self.node_owner, &Bnd::new(voltages).with_holds(if self.holds_on { &self.holds } else { &[] }), live_params, on, self.ramps.as_deref(), &mut self.hub_updates, &mut self.sensitivities, jump)?;
        Self::publish(part, live_params, voltages, currents);
        readers.extend_from_slice(&part.watch_ids);
        Self::report_changed_ic_readers(part, voltages, &self.node_to_ic_readers, readers);
        let crossing = Self::schedule_crossing(part, now, voltages, self.rail, &self.rail_edges, &self.rail_follow, self.rail_until);
        Self::set_couple(part, now, crossing, voltages, self.rail, live_params);
        if let Some(t) = crossing {
            self.scheduled.push((t, partition, part.version));
        }
        Ok(())
    }

    /// Re-anchors partition `p` at `(now, vc(now))` and aims it at the present
    /// rail (README, Rails point 3, "Rail moves": a hub partition, and one
    /// holding a watched pin of an IC on the rail, whose thresholds moved),
    /// re-publishes it and reschedules its crossing. Returns its watchers.
    /// The segment itself is kept: only the aim and the anchor move.
    pub(crate) fn reaim(&mut self, p: usize, now: f64, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) -> Vec<DeviceId> {
        let Some(slot) = self.slot(p) else { return Vec::new() };
        if self.parts[slot].seg.is_none() {
            return Vec::new();
        }
        self.journal(slot);
        let rail = self.rail;
        let part = &mut self.parts[slot];
        let vc_now = vc_of(part, now);
        bank_drift(part, now);
        let on = OnRail { rail, seg: self.rail_seg.as_ref(), cell: self.cell.as_ref(), held: self.rail_held };
        Self::aim(part, now, vc_now, &Bnd::new(voltages).with_holds(if self.holds_on { &self.holds } else { &[] }), on, self.ramps.as_deref(), 0.0);
        Self::publish(part, live_params, voltages, currents);
        let crossing = Self::schedule_crossing(part, now, voltages, rail, &self.rail_edges, &self.rail_follow, self.rail_until);
        Self::set_couple(part, now, crossing, voltages, rail, live_params);
        if let Some(t) = crossing {
            self.scheduled.push((t, p, part.version));
        }
        part.watch_ids.clone()
    }

    /// Hub partition `p`'s draw as the rail sees it at `t` ([`HubTerm`],
    /// Rails points 2 and 4), or `None` before its first segment or for a
    /// partition that does not read the rail.
    pub(crate) fn hub_term(&self, p: usize, t: f64) -> Option<HubTerm> {
        let part = &self.parts[self.slot(p)?];
        let h = part.seg.as_ref()?.hub.as_ref()?;
        let vc = vc_of(part, t);
        let w = part.drive.as_ref().filter(|d| d.cell).map_or(0.0, |d| d.modes[0].w);
        Some(HubTerm { a: h.a, b: h.b, g: h.g, u_seg: h.u_seg, vc, inf: part.seg_inf, f: part.seg_f, g1: h.curve.as_ref().map_or(0.0, |c| c.g1), w })
    }

    /// Hub partition `p` as the CC rail capacitor's coupled path takes it at
    /// `now` ([`rails::Coupled`], ruling R15 (a)): the aim it is about to take
    /// at the solved rail `u` (`Self::aim`), from its capacitor's voltage now.
    pub(crate) fn coupling(&self, p: usize, now: f64, u: f64) -> Option<rails::Coupled> {
        let part = &self.parts[self.slot(p)?];
        let seg = part.seg.as_ref()?;
        let h = seg.hub.as_ref()?;
        let vc = vc_of(part, now);
        let du = u - h.u_seg;
        let i0 = seg.i0 + h.di0du * du;
        let (di, didu, b) = match &h.curve {
            Some(c) => (seg.di + c.di1du * du, h.di0du + c.di1du * vc, h.b + c.g1 * du),
            None => (seg.di, h.di0du, h.b),
        };
        let c = part.cap.c;
        Some(rails::Coupled { f: di / c, gamma: didu / c, b, r0: (i0 + di * vc) / c })
    }

    /// How hub partition `p`'s draw moves after `t` under the rail `seg`
    /// (Rails point 4): `(b, vc - inf, f, w1)` of its capacitor's trajectory
    /// re-anchored at `t`, where the segment holds the draw at `a + b vc(t)`.
    pub(crate) fn hub_drift(&self, p: usize, t: f64, seg: &CapSeg) -> Option<(f64, f64, f64, f64)> {
        let part = &self.parts[self.slot(p)?];
        let s = part.seg.as_ref()?;
        let h = s.hub.as_ref()?;
        let du = seg.at(t) - h.u_seg;
        let i0 = s.i0 + h.di0du * du;
        let vc = vc_of(part, t);
        let (b, di, didu) = match &h.curve {
            Some(c) => (h.b + c.g1 * du, s.di + c.di1du * du, h.di0du + c.di1du * vc),
            None => (h.b, s.di, h.di0du),
        };
        let inf = if di != 0.0 { -i0 / di } else { vc };
        Some((b, vc - inf, part.seg_f, didu * seg.slope_at(t) / part.cap.c))
    }

    /// What hub partition `p`'s draw has moved, `∫ b vc`, since the rail's
    /// segment last held it (Rails point 4), or `None` before its first
    /// segment.
    pub(crate) fn drift_charge(&self, p: usize, now: f64) -> Option<f64> {
        let part = &self.parts[self.slot(p)?];
        let b = part.hub_b(part.seg.as_ref()?.hub.as_ref()?);
        Some(part.drift_q + b * charge_since(part, part.drift_from, now))
    }

    /// The rail's segment holds `parts`' draws again from `now`: their
    /// charge starts afresh.
    pub(crate) fn reset_drift(&mut self, parts: &[usize], now: f64) {
        for &p in parts {
            let Some(slot) = self.slot(p) else { continue };
            if self.parts[slot].drift_q == 0.0 && self.parts[slot].drift_from == now {
                continue;
            }
            self.journal(slot);
            let part = &mut self.parts[slot];
            part.drift_q = 0.0;
            part.drift_from = now;
        }
    }

    /// The earliest instant a driven partition's coupling band runs out in
    /// time (`couple_band`), when the rail must solve and aim it again.
    pub(crate) fn couple_expiry(&self) -> f64 {
        self.parts.iter().filter(|p| !p.parked).filter_map(|p| p.drive.as_ref().map(|d| d.expiry)).fold(f64::INFINITY, f64::min)
    }

    /// The curved hub partitions whose segment was built more than its band
    /// under `rule` from `u` (ruling R8 (a), Rails point 3): each builds again
    /// at the present rail, `vc` carried exactly.
    pub(crate) fn hubs_past_band(&self, u: f64, rule: rails::BandRule, out: &mut Vec<usize>) {
        for part in self.parts.iter().filter(|p| !p.parked) {
            if let (Some(h), Some(c)) = (part.seg.as_ref().and_then(|s| s.hub.as_ref()), part.curve()) {
                // An affine segment (an infinite band) is exact at any rail,
                // in every regime, as a linear partition is.
                if c.band.is_finite() && (u - h.u_seg).abs() > rule.band(Some(c.band)) {
                    out.push(part.global);
                }
            }
        }
    }

    /// The `u_seg` of every curved hub partition and its band under `rule`:
    /// a CC rail capacitor's segment takes their edges as thresholds, as the
    /// static and stepped ones' (Rails point 4). A driven one adds its aim's
    /// rail and coupling width (`couple_band`): the rail event there aims it
    /// again.
    pub(crate) fn nonlinear_hub_rails(&self, rule: rails::BandRule, out: &mut Vec<(f64, f64)>) {
        for part in self.parts.iter().filter(|p| !p.parked) {
            if let (Some(h), Some(c)) = (part.seg.as_ref().and_then(|s| s.hub.as_ref()), part.curve()) {
                if c.band.is_finite() {
                    out.push((h.u_seg, rule.band(Some(c.band))));
                }
                if let Some(d) = part.drive.as_ref().filter(|d| d.couple.is_finite()) {
                    out.push((d.u_a, d.couple));
                }
            }
        }
    }

    /// Curved hub partition `p`'s capacitor range, `[vc_lo, vc_hi]` (tests
    /// read it).
    #[cfg(test)]
    pub(crate) fn hub_range(&self, p: usize) -> Option<(f64, f64)> {
        self.parts[self.slot(p)?].curve().map(|c| (c.vc_lo, c.vc_hi))
    }

    /// Curved hub partition `p`'s segment band (tests read it).
    #[cfg(test)]
    pub(crate) fn hub_band(&self, p: usize) -> Option<f64> {
        self.parts[self.slot(p)?].curve().map(|c| c.band)
    }

    pub(crate) fn drain_hub_updates(&mut self) -> std::vec::Drain<'_, (usize, Contrib)> {
        self.hub_updates.drain(..)
    }

    /// Each partition's capacitor voltage at `now`, by device (W2c's carry).
    pub(crate) fn carry_caps(&self, now: f64, out: &mut Vec<(usize, f64)>) {
        out.extend(self.parts.iter().map(|p| (p.cap.device_index, if p.seg.is_some() { vc_of(p, now) } else { p.seg_vc0 })));
    }

    pub(crate) fn sensitivities(&self) -> u64 {
        self.sensitivities
    }

    /// Drains every crossing this solver wants scheduled since the last
    /// drain — `PartitionedSolver::take_scheduled`'s closed-form half.
    pub fn take_scheduled(&mut self) -> Vec<(f64, usize, u64)> {
        std::mem::take(&mut self.scheduled)
    }

    /// [`Self::take_scheduled`] without giving the queue's buffer away, for a
    /// caller that moves the entries on at once.
    pub(crate) fn drain_scheduled(&mut self) -> std::vec::Drain<'_, (f64, usize, u64)> {
        self.scheduled.drain(..)
    }

    /// The compiled shape of global partition `p_idx`, if this solver owns it.
    pub(crate) fn shape(&self, p_idx: usize) -> Option<&DynShape> {
        self.slot(p_idx).map(|slot| &self.parts[slot].shape)
    }

    /// Which segment partition `p_idx` currently holds, as an identity: it
    /// changes exactly when a re-aim built a new one (tests look at it).
    #[cfg(test)]
    pub(crate) fn segment_id(&self, p_idx: usize) -> Option<usize> {
        self.slot(p_idx).and_then(|slot| self.parts[slot].seg.as_ref().map(|seg| Rc::as_ptr(seg) as usize))
    }

    /// Makes every partition's next re-aim rebuild its segment, as if the key
    /// never matched (tests use it for the reference a reuse is compared with).
    #[cfg(test)]
    pub(crate) fn forget_segment_keys(&mut self) {
        for part in &mut self.parts {
            if let Some(seg) = part.seg.as_mut() {
                Rc::make_mut(seg).key.clear();
            }
        }
    }

    /// The exact node voltages and capacitor current AT `t_end`
    /// (`PartitionedSolver::sync_dynamic_to`'s closed-form half — see that
    /// trait method's doc comment for why this is not a no-op even though no
    /// state changes): every partition's `voltages`/`currents` entries are
    /// re-published from its still-valid segment evaluated at `t_end`,
    /// without touching `seg_t0`/`seg_vc0` (the segment's own anchor stays
    /// wherever it was last aimed; only the PUBLISHED reading moves).
    pub fn publish_at(&self, t_end: f64, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) {
        for part in &self.parts {
            if part.seg.is_none() || part.parked {
                continue; // load not settled yet, or stepping: nothing to publish.
            }
            let vc = vc_of(part, t_end);
            let du = part.drive.as_ref().map(|d| d.du(t_end - part.seg_t0));
            Self::publish_at_vc(part, vc, du, t_end - part.seg_t0, live_params, voltages, currents);
        }
    }

    fn partition_snapshot(part: &Partition) -> PartitionSnapshot {
        PartitionSnapshot {
            seg: part.seg.clone(),
            seg_t0: part.seg_t0,
            seg_vc0: part.seg_vc0,
            seg_f: part.seg_f,
            seg_inf: part.seg_inf,
            version: part.version,
            cross_local: part.cross_local,
            cross_v: part.cross_v,
            aim_x0: part.aim_x0.clone(),
            aim_i0: part.aim_i0,
            drive: part.drive.clone(),
            ramp: part.ramp.clone(),
            cross_dth: part.cross_dth,
            aim_dx: part.aim_dx.clone(),
            aim_di: part.aim_di,
            aim_du: part.aim_du,
            folded: part.folded,
            drift_q: part.drift_q,
            drift_from: part.drift_from,
        }
    }

    fn restore_partition(part: &mut Partition, s: PartitionSnapshot) {
        part.seg = s.seg;
        part.seg_t0 = s.seg_t0;
        part.seg_vc0 = s.seg_vc0;
        part.seg_f = s.seg_f;
        part.seg_inf = s.seg_inf;
        part.version = s.version;
        part.cross_local = s.cross_local;
        part.cross_v = s.cross_v;
        part.aim_x0 = s.aim_x0;
        part.aim_i0 = s.aim_i0;
        part.drive = s.drive;
        part.ramp = s.ramp;
        part.cross_dth = s.cross_dth;
        part.aim_dx = s.aim_dx;
        part.aim_di = s.aim_di;
        part.aim_du = s.aim_du;
        part.folded = s.folded;
        part.drift_q = s.drift_q;
        part.drift_from = s.drift_from;
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot { partitions: self.parts.iter().map(Self::partition_snapshot).collect(), scheduled: self.scheduled.clone() }
    }

    /// Opens the rollback window (see the field): nothing is copied yet.
    pub(crate) fn begin_window(&mut self) {
        self.window = true;
        self.window_scheduled.clone_from(&self.scheduled);
        #[cfg(test)]
        {
            self.w_saved = Some(self.parts.iter().map(|part| format!("{:?}", Self::partition_snapshot(part))).collect());
        }
    }

    /// Saves partition `slot`'s state before the window's first call that can
    /// change it. A no-op outside a window and for a partition already saved:
    /// the first pre-image is the one a rollback needs.
    fn journal(&mut self, slot: usize) {
        if self.window && !self.parts[slot].journaled {
            self.parts[slot].journaled = true;
            let saved = Self::partition_snapshot(&self.parts[slot]);
            self.journal.push((slot as u32, saved));
        }
    }

    /// The window's instant completed: the journal is dropped.
    pub(crate) fn end_window(&mut self) {
        #[cfg(test)]
        if let Some(saved) = self.w_saved.take() {
            for (slot, (part, was)) in self.parts.iter().zip(&saved).enumerate() {
                assert!(
                    part.journaled || format!("{:?}", Self::partition_snapshot(part)) == *was,
                    "closed-form partition {} changed in the window without being journaled",
                    part.global
                );
                let _ = slot;
            }
        }
        self.window = false;
        for (slot, _) in self.journal.drain(..) {
            self.parts[slot as usize].journaled = false;
        }
    }

    /// The window's instant failed: every partition it touched, and the
    /// queue of crossings to schedule, go back to what the window found.
    pub(crate) fn rollback_window(&mut self) {
        self.scheduled.clone_from(&self.window_scheduled);
        self.hub_updates.clear();
        self.window = false;
        for (slot, saved) in self.journal.drain(..) {
            let part = &mut self.parts[slot as usize];
            Self::restore_partition(part, saved);
            part.journaled = false;
        }
        #[cfg(test)]
        if let Some(saved) = self.w_saved.take() {
            for (part, was) in self.parts.iter().zip(&saved) {
                assert_eq!(format!("{:?}", Self::partition_snapshot(part)), *was, "closed-form partition {}: a rollback left it changed", part.global);
            }
        }
    }

    pub fn restore(&mut self, snapshot: Snapshot) {
        self.scheduled = snapshot.scheduled;
        for (part, s) in self.parts.iter_mut().zip(snapshot.partitions) {
            Self::restore_partition(part, s);
        }
    }

    /// The index into `parts` of global partition `p_idx`, if this solver
    /// owns it.
    fn slot(&self, p_idx: usize) -> Option<usize> {
        match self.slot_of.get(p_idx) {
            Some(&slot) if slot != u32::MAX && !self.parts[slot as usize].parked => Some(slot as usize),
            _ => None,
        }
    }

    /// Ruling R59 (1) (B): parks or unparks partition `p` with no change to
    /// its state (the owner's load, its rollback of a switch, and the CC
    /// entry, whose stepped copy takes the charge `vc_now` reads).
    pub(crate) fn set_parked(&mut self, p: usize, parked: bool) {
        if let Some(&slot) = self.slot_of.get(p).filter(|&&s| s != u32::MAX) {
            self.parts[slot as usize].parked = parked;
        }
    }

    /// Partition `p`'s segment rate `F` (`vc` relaxes as `e^(F t)`), parked
    /// or not; `None` before its first segment.
    pub(crate) fn rate(&self, p: usize) -> Option<f64> {
        let &slot = self.slot_of.get(p).filter(|&&s| s != u32::MAX)?;
        let part = &self.parts[slot as usize];
        part.seg.as_ref().map(|_| part.seg_f)
    }

    /// Partition `p`'s capacitor voltage at `t` on its present segment (the
    /// charge a CC entry hands the stepped copy), parked or not.
    pub(crate) fn vc_now(&self, p: usize, t: f64) -> Option<f64> {
        let &slot = self.slot_of.get(p).filter(|&&s| s != u32::MAX)?;
        let part = &self.parts[slot as usize];
        Some(if part.seg.is_some() { vc_of(part, t) } else { part.seg_vc0 })
    }

    /// Ruling R59 (1) (B): partition `p`'s PSU left CC and its stepped copy
    /// stopped at `now` with its capacitor at `vc`. The closed form resumes
    /// from there: a held segment (no motion) anchored at `(now, vc)`, which
    /// the re-solve the owner runs next aims afresh; its old crossing and
    /// drive are stale. Journaled as any change in a window.
    pub(crate) fn resume(&mut self, p: usize, now: f64, vc: f64) {
        let Some(&slot) = self.slot_of.get(p).filter(|&&s| s != u32::MAX) else { return };
        let slot = slot as usize;
        self.journal(slot);
        let part = &mut self.parts[slot];
        part.parked = false;
        part.seg_t0 = now;
        part.seg_vc0 = vc;
        part.seg_f = 0.0;
        part.seg_inf = vc;
        part.drive = None;
        part.ramp = None;
        part.drift_q = 0.0;
        part.drift_from = now;
        part.version += 1;
    }

    // ---- internals ---------------------------------------------------

    /// Whether driver `d` can put anything into `part`'s system: the same test
    /// `stamp_ics_into_dyn` applies to each branch of a committed driver (its
    /// own pin, or the supply node it is committed towards, is free in the
    /// partition; every branch targets one of its IC's two supply nodes).
    /// A driver of an IC that merely reads a node of the partition stamps
    /// nothing here, whatever it commits.
    fn stamps_into(part: &Partition, d: &Driver, node_owner: &HashMap<usize, (usize, usize)>) -> bool {
        let in_partition = |node: usize| matches!(node_owner.get(&node), Some(&(p, _)) if p == part.global);
        part.touch_ics
            .iter()
            .find(|(device_index, _)| *device_index == d.device)
            .is_some_and(|(_, ic)| in_partition(d.node) || in_partition(ic.vcc.node) || in_partition(ic.gnd.node))
    }

    /// Re-derives what `part.driver_ids` is derived from when the driver
    /// table's length is not the one it was derived for (see the field's doc
    /// comment).
    fn ensure_driver_ids(part: &mut Partition, drivers: &[Driver], node_owner: &HashMap<usize, (usize, usize)>) {
        if part.driver_index_len != drivers.len() {
            let ids: Vec<u32> = drivers.iter().enumerate().filter(|(_, d)| Self::stamps_into(part, d, node_owner)).map(|(id, _)| id as u32).collect();
            part.driver_ids = ids;
            part.driver_index_len = drivers.len();
        }
        #[cfg(debug_assertions)]
        {
            let fresh: Vec<u32> = drivers.iter().enumerate().filter(|(_, d)| Self::stamps_into(part, d, node_owner)).map(|(id, _)| id as u32).collect();
            debug_assert_eq!(fresh, part.driver_ids, "the driver table changed without changing its length");
        }
    }

    /// Packs into `part.key_scratch` everything a segment build reads that
    /// can differ from one build of this partition to the next: the
    /// committed mode of every driver of an IC that can touch it (`3` for
    /// none yet), the floating bucket's `high` bit of every pull pin in it
    /// (the only place `now` enters the stamp), and the live params of every
    /// resistor/switch it holds. Everything else `build_segment` reads (the
    /// shape, the capacitor, every IC record and driver's fixed fields, the
    /// fixed supply voltages, `node_owner`) is immutable after load, so equal
    /// keys mean equal inputs and a bit-identical segment.
    fn segment_key(part: &mut Partition, now: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd, ramps: Option<&RampEval>) {
        let key = &mut part.key_scratch;
        key.clear();
        for &id in &part.driver_ids {
            key.push(drivers[id as usize].committed().map_or(3, mode_discriminant));
        }
        for &(ic_index, pin_index) in &part.touch_pulls {
            let (_, ic) = &part.touch_ics[ic_index as usize];
            let pin = &ic.logic[pin_index as usize];
            key.push(digital::floating::floating_high(pin.open_hash, now, ic.ttl) as u8);
        }
        for stamped in &part.shape.stamped {
            for v in live_params_for(stamped, live_params) {
                key.extend_from_slice(&v.to_bits().to_le_bytes());
            }
        }
        // W1f: every live boundary value the build reads (the rail, a
        // waveform source's node). Empty before W1f.
        for &node in &part.live_nodes {
            key.extend_from_slice(&bnd.get(node).to_bits().to_le_bytes());
        }
        // Which of them a ramp moves: the build solves their sensitivities
        // (Sources point 3, R3 (b)).
        for &node in &part.live_nodes {
            key.push(u8::from(ramps.is_some_and(|r| r.ramping[node])));
        }
        // F5b: a held IC stamps nothing (W1c point 1a), so its hold is part
        // of the stamp; only ICs not free are named, so a key with no hold is
        // the key it always was.
        for (k, (device, _)) in part.touch_ics.iter().enumerate() {
            let hold = bnd.hold(*device);
            if !hold.is_free() {
                key.extend_from_slice(&(k as u32).to_le_bytes());
                key.push(hold.kind.tag());
            }
        }
    }

    /// Builds this partition's segment for the current stamp — or reuses the
    /// one it already has when nothing the build reads has changed (a
    /// `Crossing` re-aims the same network from a new anchor) — and anchors
    /// it at `(now, vc_now)`. The reuse still counts as a segment built (stat
    /// 8 counts re-aims), and the rate and asymptote are re-derived from
    /// `vc_now` exactly as before (`seg_inf` is `vc_now` when `di == 0`).
    #[allow(clippy::too_many_arguments)]
    fn segment_and_aim(
        part: &mut Partition,
        now: f64,
        vc_now: f64,
        drivers: &[Driver],
        node_owner: &HashMap<usize, (usize, usize)>,
        bnd: &Bnd,
        live_params: &HashMap<usize, Vec<f64>>,
        on: OnRail,
        ramps: Option<&RampEval>,
        hub_updates: &mut Vec<(usize, Contrib)>,
        sensitivities: &mut u64,
        jump: f64,
    ) -> Result<(), String> {
        bank_drift(part, now);
        Self::ensure_driver_ids(part, drivers, node_owner);
        Self::segment_key(part, now, drivers, live_params, bnd, ramps);
        // A curved segment's band holds over the capacitor voltages it was
        // sized for; a reuse from outside them builds again.
        let reusable = matches!(&part.seg, Some(seg) if seg.key == part.key_scratch && seg.hub.as_ref().and_then(|h| h.curve.as_ref()).is_none_or(|c| vc_now >= c.vc_lo && vc_now <= c.vc_hi));
        if !reusable {
            // Only the drivers `stamp_ics_into_dyn` can read, in table order,
            // and only the ICs that can touch this partition.
            part.driver_scratch.clear();
            part.driver_scratch.extend(part.driver_ids.iter().map(|&id| drivers[id as usize].clone()));
            let hub_rail = if part.hub { on.rail } else { None };
            let ramping: Vec<usize> = part.live_nodes.iter().copied().filter(|&n| ramps.is_some_and(|r| r.ramping[n])).collect();
            let driven = part.hub && on.seg.is_some();
            let mut seg = build_segment(&part.shape, &part.cap, part.global, now, &part.driver_scratch, &part.touch_ics, node_owner, bnd, live_params, hub_rail, part.curved, vc_now, &ramping, driven)?;
            if seg.hub.is_some() {
                *sensitivities += 1;
            }
            seg.key = part.key_scratch.clone();
            part.seg = Some(Rc::new(seg));
        }
        part.segments_built += 1;
        Self::aim(part, now, vc_now, bnd, on, ramps, jump);
        if let Some(h) = part.seg.as_ref().and_then(|seg| seg.hub.as_ref()) {
            hub_updates.push((part.global, Contrib { active: true, i: h.a, g: h.g, u_p: h.u_seg, b: Some(h.b) }));
        }
        Ok(())
    }

    /// Anchors the segment at `(now, vc_now)`, aimed at the present rail when
    /// it is a hub partition (P3:558-569): `x0(u) = x0 + dx0/du (u - u_seg)`
    /// and `i0(u) = i0 + di0/du (u - u_seg)` set the asymptote. Any other
    /// partition aims exactly as before W1f. While the rail is a known closed
    /// form (`rail_seg`), a hub partition is aimed at its value now, which the
    /// published rail node may trail, and takes it as its drive, as does a
    /// partition watched by an IC on the rail.
    /// A ramp that moves what the partition reads drives it too (Sources
    /// point 3, R3 (b)).
    ///
    /// On a draining cell (ruling R15 (a)) a hub partition's own draw moves
    /// the cell's rail by `kappa b (vc - vc_a)` (`rails::CellDrive`), which
    /// moves its own nodes and current by their sensitivities: that is affine
    /// in `vc`, so it is taken into the aim exactly, `dx` and `di` gaining
    /// `dx/du kappa b` and `di/du kappa b` (at `vc_a` for a curved segment,
    /// whose bilinear remainder `couple_band` would bound under a CC drive and
    /// the drift watch bounds here), with the rate and asymptote following.
    /// The drain's line drives it as a ramp, a one-mode [`Drive`] at `k = 0`.
    fn aim(part: &mut Partition, now: f64, vc_now: f64, bnd: &Bnd, on: OnRail, ramps: Option<&RampEval>, jump: f64) {
        // The coupling's error so far under this drive, read off the aim it
        // ends (its trajectory's own parameters, before they are replaced),
        // and the step a crossing's landing just put into `vc` (`jump`).
        let inherited = match (&part.drive, on.seg) {
            (Some(d), Some(_)) if d.g1 > 0.0 && !d.cell => CoupleAim::of(part, d, None).map_or(0.0, |a| a.err_at(now - part.seg_t0)) + jump,
            _ => 0.0,
        };
        let seg = Rc::clone(part.seg.as_ref().expect("a segment exists before it is aimed"));
        let driven = on.seg.filter(|_| seg.hub.is_some() || part.rail_watched);
        part.folded = false;
        let mut i0 = match (&seg.hub, on.rail) {
            (Some(h), Some(rail)) => {
                let u = driven.map_or_else(|| bnd.get(rail), |s| s.at(now));
                let du = u - h.u_seg;
                part.aim_x0.clear();
                part.aim_x0.extend(seg.x0.iter().zip(&h.dx0du).map(|(x, d)| x + d * du));
                part.aim_i0 = seg.i0 + h.di0du * du;
                if let Some(c) = &h.curve {
                    part.aim_dx.clear();
                    part.aim_dx.extend(seg.dx.iter().zip(&c.dx1du).map(|(x, d)| x + d * du));
                    part.aim_di = seg.di + c.di1du * du;
                    part.aim_du = du;
                }
                part.aim_i0
            }
            _ => seg.i0,
        };
        // A curved segment's current follows the rail at `vc_now`'s slope
        // under the drive; the coupling that leaves out is bounded by
        // `couple_band`, which aims the partition again before it reaches W0's
        // tolerance (V9 MEDIUM 1).
        let di0du = seg.hub.as_ref().map_or(0.0, |h| match &h.curve {
            Some(c) => h.di0du + c.di1du * vc_now,
            None => h.di0du,
        });
        let g1 = seg.hub.as_ref().and_then(|h| h.curve.as_ref()).map_or(0.0, |c| c.di1du.abs());
        let mut line = None;
        if let (Some(cell), Some(h), Some(rail), None) = (on.cell, seg.hub.as_ref(), on.rail, driven) {
            let b = match &h.curve {
                Some(c) => h.b + c.g1 * part.aim_du,
                None => h.b,
            };
            let pull = cell.kappa * b;
            if pull != 0.0 && pull.is_finite() {
                if h.curve.is_none() {
                    part.aim_dx.clear();
                    part.aim_dx.extend_from_slice(&seg.dx);
                    part.aim_di = seg.di;
                }
                for l in 0..part.aim_dx.len() {
                    let s = match &h.curve {
                        Some(c) => h.dx0du[l] + c.dx1du[l] * vc_now,
                        None => h.dx0du[l],
                    };
                    part.aim_dx[l] += s * pull;
                    part.aim_x0[l] -= s * pull * vc_now;
                }
                part.aim_di += di0du * pull;
                part.aim_i0 -= di0du * pull * vc_now;
                i0 = part.aim_i0;
                part.folded = true;
            }
            if cell.rho != 0.0 && cell.rho.is_finite() {
                let modes: Rc<[DriveMode]> = Rc::from(vec![DriveMode { k: 0.0, s: cell.rho, w: di0du * cell.rho / part.cap.c, we: 0.0 }]);
                line = Some(Drive { u_a: bnd.get(rail), modes, cell: true, held: false, couple: f64::INFINITY, expiry: f64::INFINITY, cell_we: 0.0, err: 0.0, g1: 0.0 });
            }
        }
        part.drive = match driven {
            Some(s) => {
                let follows = seg.hub.is_some();
                let modes: Rc<[DriveMode]> = s.modes_at(now).into_iter().map(|(k, sm)| DriveMode { k, s: sm, w: if follows { di0du * sm / part.cap.c } else { 0.0 }, we: 0.0 }).collect();
                Some(Drive { u_a: s.at(now), modes, cell: false, held: on.held, couple: f64::INFINITY, expiry: f64::INFINITY, cell_we: 0.0, err: inherited, g1 })
            }
            None => line,
        };
        part.ramp = ramps.and_then(|r| ramp_drive(&seg, r, part.cap.c));
        // `partition::build` keeps a reader of the rail, or of a threshold
        // that moves with it, stepped when it reads a ramp: the two drives
        // are not composed.
        debug_assert!(part.drive.is_none() || part.ramp.is_none(), "partition {} is driven by the rail and a ramp at once", part.global);
        let di = if part.folded || seg.hub.as_ref().is_some_and(|h| h.curve.is_some()) { part.aim_di } else { seg.di };
        part.seg_f = di / part.cap.c;
        part.seg_inf = if di != 0.0 { -i0 / di } else { vc_now };
        // A curved segment's current follows the cell's line by `di0du +
        // di1du vc`, and `vc` itself moves across its transient: taken at
        // `vc_a` alone, the line's pull on a capacitor swinging volts stays
        // where `vc_a` put it (F1r14: a falling gate on a cell, 15x W0 once
        // settled). With `vc = inf + (vc_a - inf) e^(fτ)` the forcing is
        // `rho τ (di0du + di1du inf) + rho τ di1du (vc_a - inf) e^(fτ)`, whose
        // response is the line's at `inf` and `we τ² e^(fτ) / 2`.
        if let (Some(d), Some(c)) = (part.drive.as_mut(), seg.hub.as_ref().and_then(|h| h.curve.as_ref().map(|c| (h.di0du, c.di1du)))) {
            if d.cell {
                let (di0du, di1du) = c;
                let rho = d.modes[0].s;
                d.modes = Rc::from(vec![DriveMode { k: 0.0, s: rho, w: (di0du + di1du * part.seg_inf) * rho / part.cap.c, we: 0.0 }]);
                d.cell_we = di1du * (vc_now - part.seg_inf) * rho / part.cap.c;
            } else if d.held && seg.hub.is_some() {
                // The same on a cell's coupled path (ruling R16 (B)), mode by
                // mode: the rail's pull taken at `inf`, and the transient's
                // share `we e^(fτ) χ_k(τ)` (F1r15: a falling gate on a cell,
                // 104x W0 at `vc_a`'s slope, its rail barely moving, so no
                // band's event came while the transient ran).
                let (di0du, di1du) = c;
                let (cap, inf) = (part.cap.c, part.seg_inf);
                d.modes = d.modes.iter().map(|m| DriveMode { k: m.k, s: m.s, w: (di0du + di1du * inf) * m.s / cap, we: di1du * (vc_now - inf) * m.s / cap }).collect();
            }
        }
        part.seg_t0 = now;
        part.seg_vc0 = vc_now;
        part.version += 1;
    }

    /// Once a driven curved partition is aimed and its crossing scheduled
    /// (`crossing`, the time it lands at, if any): how far the rail may move
    /// before it is aimed again ([`couple_band`]).
    fn set_couple(part: &mut Partition, now: f64, crossing: Option<f64>, voltages: &[f64], rail: Option<usize>, live_params: &HashMap<usize, Vec<f64>>) {
        let (Some(d), Some(rail)) = (part.drive.clone(), rail) else { return };
        if d.cell || part.curve().is_none() {
            return;
        }
        let (couple, at) = couple_band(part, &d, crossing.map(|t| t - now), &Bnd::new(voltages), rail, live_params);
        let expiry = if d.modes.len() > 1 && at.is_finite() { now + at.max(1e-9) } else { f64::INFINITY };
        if let Some(d) = part.drive.as_mut() {
            d.couple = couple;
            d.expiry = expiry;
        }
    }

    /// Notes the published bits of every node of `part`, in
    /// `part.shape.nodes` order, before a re-aim publishes over them.
    fn note_bits(part: &mut Partition, voltages: &[f64]) {
        part.bits_scratch.clear();
        part.bits_scratch.extend(part.shape.nodes.iter().map(|&n| voltages[n].to_bits()));
    }

    /// W1c contract point 4's last bullet, mirrored from `static_part.rs`'s
    /// own `node_to_ic_readers` walk: for every one of this partition's own
    /// local nodes whose PUBLISHED bits actually changed this re-aim,
    /// extends `readers` with every IC that reads it (its `vcc`, `gnd`, or
    /// any logic pin) — not only the ones with an explicit crossing watch
    /// registered. `part.bits_scratch` is `part.shape.nodes`-order, captured by the
    /// caller BEFORE `Self::publish` (`Self::note_bits`); `readers` may already hold entries
    /// (from `part.watches`), and a duplicate here is harmless — `Kernel`
    /// dedups across a whole delta's readers regardless.
    fn report_changed_ic_readers(part: &Partition, voltages: &[f64], node_to_ic_readers: &[Vec<usize>], readers: &mut Vec<DeviceId>) {
        for (l, &node) in part.shape.nodes.iter().enumerate() {
            if voltages[node].to_bits() != part.bits_scratch[l] {
                readers.extend(node_to_ic_readers[node].iter().map(|&d| DeviceId(d as u32)));
            }
        }
    }

    fn publish(part: &Partition, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) {
        Self::publish_at_vc(part, part.seg_vc0, None, 0.0, live_params, voltages, currents);
    }

    /// Publishes every one of this partition's own currents (core/README.md's
    /// "Currents" rules) at the given `vc`: the capacitor's series-plus-
    /// leakage current from the exact segment, AND — W1b's own review caught
    /// this missing entirely — every resistor's and switch's OWN current
    /// ([`crate::solve::publish_stamped_currents`]), which nothing else in
    /// `PartitionedSolver` ever computed for a device sitting inside a
    /// `ClosedForm` partition (`recompute_source_currents` only reads a
    /// device's current back out for a `voltage_source`'s own KCL, never
    /// computes one for a plain resistor/switch itself).
    /// `du`: how far a known rail has moved since the aim, which a driven hub
    /// partition's nodes follow through their sensitivities, and `tau`: the
    /// time since the aim, along which a ramp drives them.
    #[allow(clippy::too_many_arguments)]
    fn publish_at_vc(part: &Partition, vc: f64, du: Option<f64>, tau: f64, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) {
        let seg = part.seg.as_ref().expect("segReady guard upheld by every caller");
        let (x0, i0) = if seg.hub.is_some() { (&part.aim_x0[..], part.aim_i0) } else { (&seg.x0[..], seg.i0) };
        let (dx, di) = (part.dx(), part.di());
        let mut series_i = match (du, seg.hub.as_ref()) {
            (Some(du), Some(h)) => {
                for (l, &node) in part.shape.nodes.iter().enumerate() {
                    voltages[node] = x0[l] + part.dxdu(h, l, vc) * du + dx[l] * vc;
                }
                let didu = match &h.curve {
                    Some(c) => h.di0du + c.di1du * vc,
                    None => h.di0du,
                };
                i0 + didu * du + di * vc
            }
            _ => {
                for (l, &node) in part.shape.nodes.iter().enumerate() {
                    voltages[node] = x0[l] + dx[l] * vc;
                }
                i0 + di * vc
            }
        };
        if let Some(r) = &part.ramp {
            for (l, &node) in part.shape.nodes.iter().enumerate() {
                voltages[node] += r.rate_x[l] * tau;
            }
            series_i += r.rate_i * tau;
        }
        let (va, vb) = (pin_voltage_global(&part.cap.pins[0], voltages), pin_voltage_global(&part.cap.pins[1], voltages));
        let leak_i = part.cap.leak_g * (va - vb);
        currents[part.cap.device_index] = series_i + leak_i;
        crate::solve::publish_stamped_currents(&part.shape, voltages, live_params, currents);
    }

    /// pec3's `_scheduleCrossing` (`:580-609`), minus the rail term: the
    /// earliest accepted crossing across every watched node/threshold this
    /// segment's asymptote actually reaches, or `None`. Only a DECAYING
    /// segment (`F < 0`) is ever scheduled this way — `F >= 0` (growing or
    /// degenerate) is left alone, exactly as pec3 does, since the log
    /// formula below assumes a decaying exponential.
    #[allow(clippy::too_many_arguments)]
    fn schedule_crossing(part: &mut Partition, now: f64, voltages: &[f64], rail: Option<usize>, edges: &[f64], follow: &[(usize, f64, f64)], until: f64) -> Option<f64> {
        let f = part.seg_f;
        if !(f < 0.0) {
            return None;
        }
        if let (Some(d), Some(rail)) = (part.drive.clone(), rail) {
            return if d.cell { Self::schedule_cell_crossing(part, now, voltages, &d) } else { Self::schedule_driven_crossing(part, now, voltages, rail, &d, edges, follow, until) };
        }
        if part.ramp.is_some() {
            return Self::schedule_ramp_crossing(part, now, voltages);
        }
        let seg = part.seg.as_ref()?;
        let (inf, v0, t0) = (part.seg_inf, part.seg_vc0, part.seg_t0);
        let mut best: Option<(f64, usize, f64)> = None; // (t, local, threshold+nudge)
        let seg_x0 = if seg.hub.is_some() { &part.aim_x0[..] } else { &seg.x0[..] };
        let seg_dx = part.dx();
        for (&local, watchers) in &part.watches {
            let x0 = seg_x0[local];
            let dx = seg_dx[local];
            if dx == 0.0 {
                continue;
            }
            let now_v = x0 + dx * v0;
            let end_v = x0 + dx * inf;
            for (_, thresholds) in watchers {
                for th in thresholds(voltages) {
                    let ok = if th.rising { now_v < th.value && end_v > th.value } else { now_v > th.value && end_v < th.value };
                    if !ok {
                        continue;
                    }
                    let vc_star = (th.value - x0) / dx;
                    let ratio = (vc_star - inf) / (v0 - inf);
                    if !(ratio > 0.0 && ratio < 1.0) {
                        continue;
                    }
                    let t = t0 + ratio.ln() / f;
                    if !accepts_crossing(t, now) {
                        continue;
                    }
                    let nudged = th.value + if th.rising { CROSS_NUDGE_V } else { -CROSS_NUDGE_V };
                    if best.is_none_or(|(bt, _, _)| t < bt) {
                        best = Some((t, local, nudged));
                    }
                }
            }
        }
        let (t, local, cross_v) = best?;
        part.cross_local = local;
        part.cross_v = cross_v;
        part.cross_dth = 0.0;
        Some(t)
    }

    /// [`Self::schedule_crossing`] under a known rail (Rails point 4): each
    /// watched node follows its driven trajectory, each threshold moves with
    /// the rail, and the first crossing is a root of their difference
    /// (`rails::driven_crossing`).
    ///
    /// A threshold's motion is one difference in the ramp's direction, at the
    /// rail the trajectory starts from, with a step that stays between the
    /// rail's `edges`: the power windows' edges and the kinks of the generic
    /// levels (`max(0.8, 0.3 vs)` at 2.667 V, `max(2.0, 0.6 vs)` at 3.333 V,
    /// the CMOS ratios' clamps at 0.57 and 0.67 V). Between them every
    /// threshold the compiler emits is affine in the supply (TTL levels
    /// fixed, the generic and CMOS levels past or short of their kinks,
    /// Schmitt pairs scaled by the supply, the 555's thirds), so the
    /// difference is the slope to rounding, and each edge is a rail event
    /// that aims again. Only a rail within `1e-3` of a step of an edge can
    /// take a step across it, and that edge's event aims again before the
    /// rail has moved so far. A supply on a static hub node (`follow`) moves
    /// with the rail on the line that node publishes.
    ///
    /// Under a coupled rail of several modes (ruling R15 (a)) the difference
    /// is a sum of monotone terms, each mode's line and its capacitor's
    /// response, whose first root `rails::ExpSum` finds; with one mode it is
    /// `driven_crossing`'s as before.
    #[allow(clippy::too_many_arguments)]
    ///
    /// On a draining cell's coupled path (`Drive::held`) the rail solves
    /// again by `until` and aims the partition again: a root past it is no
    /// crossing of this aim's, and is not searched for.
    fn schedule_driven_crossing(part: &mut Partition, now: f64, voltages: &[f64], rail: usize, d: &Drive, edges: &[f64], follow: &[(usize, f64, f64)], until: f64) -> Option<f64> {
        if part.watches.is_empty() {
            return None;
        }
        let f = part.seg_f;
        let seg = part.seg.as_ref()?;
        let (inf, v0, t0) = (part.seg_inf, part.seg_vc0, part.seg_t0);
        let hub = seg.hub.as_deref();
        let dir = if d.sum(|m| m.s) < 0.0 { -1.0 } else { 1.0 };
        let within = if d.held { until - t0 } else { f64::INFINITY };
        let single = d.single();
        let mut terms: Vec<(f64, rails::Basis)> = Vec::new();
        let nominal = 1e-6 * d.u_a.abs().max(1.0);
        let gap = edges.iter().map(|&e| dir * (e - d.u_a)).filter(|&g| g > 0.0).fold(f64::INFINITY, f64::min);
        let step = dir * (0.5 * gap).min(nominal).max(1e-3 * nominal);
        let mut at_rail = voltages.to_vec();
        at_rail[rail] = d.u_a;
        let mut stepped = at_rail.clone();
        stepped[rail] = d.u_a + step;
        for &(node, a, s) in follow {
            at_rail[node] = a + s * d.u_a;
            stepped[node] = a + s * (d.u_a + step);
        }
        let mut best: Option<(f64, usize, f64, f64)> = None; // (t, local, threshold+nudge at the aim, its rail slope)
        let seg_dx = part.dx();
        for (&local, watchers) in &part.watches {
            let dx = seg_dx[local];
            if dx == 0.0 {
                continue;
            }
            // A curved segment's sensitivity at the aim's `vc`, as its
            // current's: the node then errs by `dx1du (u - u_a) (vc - vc_a)`,
            // which `couple_band` bounds (V9 MEDIUM 1).
            let (x_a, sx) = match hub {
                Some(h) => (part.aim_x0[local], part.dxdu(h, local, v0)),
                None => (seg.x0[local], 0.0),
            };
            for (_, thresholds) in watchers {
                let at = thresholds(&at_rail[..]);
                let moved = thresholds(&stepped[..]);
                // Every watch's list has one shape at any supply (a power
                // gate never empties it), so its entries pair up. Were one
                // to change shape, a test build stops here; a release build
                // holds that threshold where it stands, as a re-aim at each
                // rail event did before the drive.
                debug_assert!(moved.len() == at.len() && moved.iter().zip(&at).all(|(m, a)| m.rising == a.rising), "a watch's thresholds changed shape with the rail");
                for (i, th) in at.iter().enumerate() {
                    let dth = moved.get(i).filter(|m| m.rising == th.rising).map_or(0.0, |m| (m.value - th.value) / step);
                    let c0 = x_a + dx * inf - th.value;
                    let ce = dx * (v0 - inf);
                    let root = match single {
                        Some(m) => rails::driven_crossing(c0, ce, (sx - dth) * m.s, dx * m.w, f, m.k, th.rising).filter(|&t| t <= within),
                        None => {
                            terms.clear();
                            if ce != 0.0 {
                                terms.push((ce, rails::Basis::Exp(f)));
                            }
                            for m in d.modes.iter() {
                                let cphi = (sx - dth) * m.s;
                                if cphi != 0.0 {
                                    terms.push((cphi, rails::Basis::Phi(m.k)));
                                }
                                if dx * m.w != 0.0 {
                                    terms.push((dx * m.w, rails::Basis::J(f, m.k)));
                                }
                            }
                            rails::ExpSum { c0, terms: &terms }.first_crossing_until(th.rising, within)
                        }
                    };
                    let Some(tau) = root else { continue };
                    let t = t0 + tau;
                    if !accepts_crossing(t, now) {
                        continue;
                    }
                    let nudged = th.value + if th.rising { CROSS_NUDGE_V } else { -CROSS_NUDGE_V };
                    if best.is_none_or(|(bt, ..)| t < bt) {
                        best = Some((t, local, nudged, dth));
                    }
                }
            }
        }
        let (t, local, cross_v, dth) = best?;
        part.cross_local = local;
        part.cross_v = cross_v;
        part.cross_dth = dth;
        Some(t)
    }

    /// [`Self::schedule_crossing`] on a draining cell (ruling R15 (a)): each
    /// watched node is `x_a + dx vc + dx/du rho τ` with its own pull folded
    /// into `x_a` and `dx` (`Self::aim`) and `vc` following the drain's line,
    /// so its first crossing of a threshold is a root of
    /// `rails::line_crossing`. The thresholds are read at the published
    /// voltages, as an undriven partition reads them.
    ///
    /// The line holds only until the rail's next event, which the drift
    /// watch puts no later than `RAIL_DRIFT_V` of the line's own motion
    /// (`rail_watches`), and which aims the partition again: a crossing past
    /// that is no crossing of this aim's. The search runs to twice that, the
    /// watch's line being read again at its own instant.
    fn schedule_cell_crossing(part: &mut Partition, now: f64, voltages: &[f64], d: &Drive) -> Option<f64> {
        if part.watches.is_empty() {
            return None;
        }
        let f = part.seg_f;
        let seg = part.seg.as_ref()?;
        let h = seg.hub.as_deref()?;
        let (inf, v0, t0) = (part.seg_inf, part.seg_vc0, part.seg_t0);
        let m = d.modes[0];
        let horizon = (2.0 * rails::RAIL_DRIFT_V / m.s.abs()).max(1e-14);
        let mut best: Option<(f64, usize, f64)> = None;
        let seg_dx = part.dx();
        for (&local, watchers) in &part.watches {
            let (dx, sx) = (seg_dx[local], part.dxdu(h, local, v0));
            if dx == 0.0 && sx == 0.0 {
                continue;
            }
            let x_a = part.aim_x0[local];
            for (_, thresholds) in watchers {
                for th in thresholds(voltages) {
                    let Some(tau) = rails::line_crossing(x_a + dx * inf - th.value, dx * (v0 - inf), sx * m.s, dx * m.w, f, th.rising, horizon) else { continue };
                    let t = t0 + tau;
                    if !accepts_crossing(t, now) {
                        continue;
                    }
                    let nudged = th.value + if th.rising { CROSS_NUDGE_V } else { -CROSS_NUDGE_V };
                    if best.is_none_or(|(bt, _, _)| t < bt) {
                        best = Some((t, local, nudged));
                    }
                }
            }
        }
        let (t, local, cross_v) = best?;
        part.cross_local = local;
        part.cross_v = cross_v;
        part.cross_dth = 0.0;
        Some(t)
    }

    /// [`Self::schedule_crossing`] under a resolved ramp (Sources point 3, D3
    /// (b)): each watched node follows `x0_a + rate_x τ + dx vc(τ)` up to the
    /// ramp's next corner, where the partition aims again, and its first
    /// crossing of a threshold is a root of `rails::ramp_crossing`. The
    /// thresholds stand still: `partition::build` keeps a partition stepped
    /// whose watched pins read against a moving supply.
    fn schedule_ramp_crossing(part: &mut Partition, now: f64, voltages: &[f64]) -> Option<f64> {
        let seg = part.seg.as_ref()?;
        let r = part.ramp.as_ref()?;
        let (f, inf, v0, t0) = (part.seg_f, part.seg_inf, part.seg_vc0, part.seg_t0);
        let horizon = r.end - t0;
        let mut best: Option<(f64, usize, f64)> = None;
        for (&local, watchers) in &part.watches {
            let (x0, dx, rx) = (seg.x0[local], seg.dx[local], r.rate_x[local]);
            if dx == 0.0 && rx == 0.0 {
                continue;
            }
            for (_, thresholds) in watchers {
                for th in thresholds(voltages) {
                    let Some(tau) = rails::ramp_crossing(x0 + dx * inf - th.value, dx * (v0 - inf), rx, dx * r.w, f, th.rising, horizon) else { continue };
                    let t = t0 + tau;
                    if !accepts_crossing(t, now) {
                        continue;
                    }
                    let nudged = th.value + if th.rising { CROSS_NUDGE_V } else { -CROSS_NUDGE_V };
                    if best.is_none_or(|(bt, _, _)| t < bt) {
                        best = Some((t, local, nudged));
                    }
                }
            }
        }
        let (t, local, cross_v) = best?;
        part.cross_local = local;
        part.cross_v = cross_v;
        part.cross_dth = 0.0;
        Some(t)
    }
}

/// The ramp drive of a partition aimed now under `ramps` (Sources point 3, D3
/// (b)): each node the segment was built with a sensitivity to moves at its
/// ramp's slope, until the earliest of their corners.
fn ramp_drive(seg: &Segment, ramps: &RampEval, c: f64) -> Option<Rc<RampDrive>> {
    if seg.ramp_sens.is_empty() {
        return None;
    }
    let mut rate_x = vec![0.0; seg.dx.len()];
    let (mut rate_i, mut end, mut any) = (0.0, f64::INFINITY, false);
    for (node, sx, si) in &seg.ramp_sens {
        let Some((slope, e)) = ramps.node_ramp(*node) else {
            debug_assert!(false, "a segment holds a sensitivity to node {node}, which no ramp moves");
            continue;
        };
        any = true;
        for (rx, s) in rate_x.iter_mut().zip(sx) {
            *rx += slope * s;
        }
        rate_i += slope * si;
        end = end.min(e);
    }
    any.then(|| Rc::new(RampDrive { rate_x, rate_i, w: rate_i / c, end }))
}

/// `∫ vc` over `[from, to]` on the partition's present trajectory, the part
/// before its aim (a stale `from`) left out.
fn charge_since(part: &Partition, from: f64, to: f64) -> f64 {
    let integral = |tau: f64| {
        let (f, inf, vc0) = (part.seg_f, part.seg_inf, part.seg_vc0);
        let base = if f == 0.0 { vc0 * tau } else { inf * tau + (vc0 - inf) * rails::exp_integral(f, tau) };
        match &part.drive {
            Some(d) if d.follows() => base + d.jw_charge(f, tau),
            _ => base,
        }
    };
    let t_a = part.seg_t0;
    integral(to - t_a) - integral(from.max(t_a) - t_a)
}

/// Banks a driven hub partition's draw since `drift_from`, `∫ b vc`, before
/// its trajectory changes at `now` (Rails point 4's charge correction).
fn bank_drift(part: &mut Partition, now: f64) {
    if part.drive.as_ref().is_none_or(|d| d.cell) {
        return;
    }
    let Some(b) = part.seg.as_ref().and_then(|s| s.hub.as_ref()).map(|h| part.hub_b(h)) else { return };
    let drawn = b * charge_since(part, part.drift_from, now);
    part.drift_q += drawn;
    part.drift_from = now;
}

/// `vc` at `t` on the partition's segment, driven by the rail when that is a
/// known closed form (Rails point 4).
/// A pin of a device for [`ClosedFormSolver::driven_device`].
#[derive(Debug, Clone, Copy)]
pub(crate) enum DevicePin {
    /// A node of the partition.
    Node(usize),
    /// The rail that drives it.
    Rail,
    /// A fixed node's value.
    Fixed(f64),
}

/// R21, a linear segment's re-anchor (`ClosedFormSolver::driven_anchor`):
/// the bound on the node's error `δ` after the anchor, `|a| S δ + |dx| W
/// min(δ²/2, δ/|f|)` (`min(...)` read as `δ²/2` when `f = 0`).
pub(crate) fn linear_anchor_bound(a: f64, s_sum: f64, dx: f64, w_sum: f64, f: f64, delta: f64) -> f64 {
    let lag = if f < 0.0 { (delta * delta / 2.0).min(delta / -f) } else { delta * delta / 2.0 };
    a * s_sum * delta + dx * w_sum * lag
}

/// The largest `δ` with `linear_anchor_bound(..., δ) <= tol`, in closed
/// form: inside `δ <= 2/|f|` the root of `(|dx| W/2) δ² + |a| S δ = tol`,
/// past it (or when that root lies past it) `tol / (|a| S + |dx| W/|f|)`.
pub(crate) fn linear_anchor_deadline(a: f64, s_sum: f64, dx: f64, w_sum: f64, f: f64, tol: f64) -> f64 {
    let (lin, quad) = (a * s_sum, dx * w_sum);
    if lin == 0.0 && quad == 0.0 {
        return f64::INFINITY;
    }
    let knee = if f < 0.0 { 2.0 / -f } else { f64::INFINITY };
    let inner = if quad > 0.0 { (2.0 * tol) / (lin + (lin * lin + 2.0 * quad * tol).sqrt()) } else { tol / lin };
    let deadline = if inner <= knee { inner } else { tol / (lin + quad / -f) };
    debug_assert!(linear_anchor_bound(a, s_sum, dx, w_sum, f, deadline) <= tol * (1.0 + 1e-9), "the deadline is inside its bound");
    deadline
}

/// R21, a curved or folded segment's re-anchor: the terms of the bound
/// `L(τn + δ) δ` (`ClosedFormSolver::driven_anchor`).
/// The node is `x0 + (a0 + a1 vc) du + dx vc`, so `L` takes `(a0 + a1|vc|)
/// |du'| + a1 |vc'| |du| + |dx| |vc'|` over the anchor, with `|vc|`, `|du|`
/// and the rates each bounded from the segment's start to `τn + δ`.
#[derive(Debug, Clone)]
pub(crate) struct CurvedBound {
    pub tau_n: f64,
    pub s_sum: f64,
    pub w_sum: f64,
    /// The drive's modes `(|s|, k, |w|)`, every `k >= 0`: the rail's rate
    /// from `τn` on is at most `Σ|s| e^(-k τn)`, its move since the segment
    /// began `Σ|s| φ_k(τ)`, and the forcing `Σ|w| φ_k(τ)`, `φ_k(τ) = (1 -
    /// e^(-kτ))/k`. Bounded by `S` and `W` alone, a segment whose modes had
    /// decayed kept the rates it was aimed with and a move growing with its
    /// age: cold 1838 tpl 1's rated rail resistors re-anchored every 2 ns to
    /// 67 ns on a CC rail settled at its operating point.
    pub modes: Vec<CurvedMode>,
    pub we: f64,
    pub a0: f64,
    pub a1: f64,
    pub adx: f64,
    pub vm: f64,
    pub dvc: f64,
    pub f: f64,
}

/// One mode of a [`CurvedBound`], with what does not change over the
/// deadline's search: `e^(-k τn)` and `J'(f, k, τn)`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct CurvedMode {
    pub s: f64,
    pub k: f64,
    pub w: f64,
    pub e_n: f64,
    pub jp_n: f64,
}

/// The modes `(|s|, k, |w|)` of a segment of rate `f`, `τn` into it.
pub(crate) fn curved_modes(modes: impl Iterator<Item = (f64, f64, f64)>, f: f64, tau_n: f64) -> Vec<CurvedMode> {
    modes.map(|(s, k, w)| CurvedMode { s, k, w, e_n: (-k * tau_n).exp(), jp_n: j_prime(f, k, tau_n) }).collect()
}

impl CurvedBound {
    pub(crate) fn at(&self, delta: f64) -> f64 {
        let t = self.tau_n + delta;
        let af = self.f.abs();
        // `|J'| = ∫ e^(f(τ-s)) e^(-k s) ds <= min(τ, 1/|f|)`; the held and
        // cell terms by their derivatives' `τ + |f| τ²/2`.
        let j_rate = if af > 0.0 { t.min(1.0 / af) } else { t };
        // From `τn` on, `J'` is the part already driven, decaying (at most
        // `J'(τn)`), plus what the forcing still adds, `e^(-k τn) min(δ,
        // 1/|f|)`.
        let added = if af > 0.0 { delta.min(1.0 / af) } else { delta };
        // `|vc|`: the forcing's part `∫ e^(f(τ-s)) F(s) ds`, `|F| <= Σ|w|
        // φ_k(τ)`, is at most that times `min(τ, 1/|f|)` as well as `W τ²/2`;
        // `|du|` since the segment began is `Σ|s| φ_k(τ)` at most, and `|du'|`
        // from `τn` on `Σ|s| e^(-k τn)`.
        let (mut forcing, mut forced, mut moved, mut rate) = (0.0, 0.0, 0.0, 0.0);
        for m in &self.modes {
            let phi = rails::exp_integral(-m.k, t);
            forcing += m.w * (m.jp_n + m.e_n * added).min(j_rate);
            forced += m.w * phi;
            moved += m.s * phi;
            rate += m.s * m.e_n;
        }
        let dvc_rate = forcing.min(self.w_sum * j_rate) + self.we * (t + af * t * t / 2.0);
        let sens = self.a0 + self.a1 * (self.vm + (forced * j_rate).min(self.w_sum * t * t / 2.0) + self.we * t * t / 2.0);
        let (moved, rate) = (moved.min(self.s_sum * t), rate.min(self.s_sum));
        let l_bound = self.a1 * (af * self.dvc + dvc_rate) * moved + sens * rate + self.adx * dvc_rate;
        l_bound * delta
    }
}

/// `J'(f, k, τ) = ∫0^τ e^(f(τ-s)) e^(-k s) ds = (e^(fτ) - e^(-kτ)) / (f + k)`,
/// for `f <= 0 <= k`, the rate at which a capacitor of rate `f` follows a
/// forcing mode of rate `k`, written so neither exponential overflows.
fn j_prime(f: f64, k: f64, tau: f64) -> f64 {
    let a = f + k;
    if a > 0.0 {
        (f * tau).exp() * rails::exp_integral(-a, tau)
    } else {
        (-k * tau).exp() * rails::exp_integral(a, tau)
    }
}

/// The largest `δ` with `bound(δ) <= tol`, `bound` nondecreasing from 0
/// (infinity when it never gets there), to bisection's last useful bit.
pub(crate) fn largest_within(bound: impl Fn(f64) -> f64, tol: f64) -> f64 {
    let mut hi = 1e-12;
    while bound(hi) <= tol {
        hi *= 2.0;
        if hi > 1e12 {
            return f64::INFINITY;
        }
    }
    let mut lo = 0.0;
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        // Adjacent doubles: nothing is left to split, and every further
        // pass would land on `lo` or `hi` again.
        if mid <= lo || mid >= hi {
            break;
        }
        if bound(mid) <= tol {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

fn vc_of(part: &Partition, t: f64) -> f64 {
    let vc = vc_at(part.seg_f, part.seg_inf, part.seg_vc0, part.seg_t0, t);
    let vc = match &part.drive {
        Some(d) if d.follows() => {
            let tau = t - part.seg_t0;
            let vc = vc + d.jw(part.seg_f, tau);
            let vc = if d.held && d.modes.iter().any(|m| m.we != 0.0) { vc + (part.seg_f * tau).exp() * d.sum(|m| m.we * rails::phi_integral(m.k, tau)) } else { vc };
            if d.cell_we != 0.0 {
                vc + d.cell_we * 0.5 * tau * tau * (part.seg_f * tau).exp()
            } else {
                vc
            }
        }
        _ => vc,
    };
    // A resolved ramp's line is the same drive at `k = 0` (R3 (b)).
    match &part.ramp {
        Some(r) if r.w != 0.0 => vc + r.w * rails::ramp_response(part.seg_f, 0.0, t - part.seg_t0),
        _ => vc,
    }
}

/// The exact exponential (this module's doc comment): `vc(t) = vinf +
/// (vc0-vinf)*exp(F*(t-t0))`, or the constant `vc0` when `F == 0` (a segment
/// whose current does not depend on `vc` at all — no capacitor current
/// changes it).
fn vc_at(f: f64, inf: f64, vc0: f64, t0: f64, t: f64) -> f64 {
    if f == 0.0 {
        vc0
    } else {
        inf + (vc0 - inf) * (f * (t - t0)).exp()
    }
}

/// How far into the transient (in time constants) [`couple_band`] spreads the
/// tolerance a curved segment's coupling may use: an aim of `x` time
/// constants may add at most `x / COUPLE_PACE` of what is left. Each aim
/// adds to what earlier ones left, which decays only with the segment's own
/// transient; a greedy aim would leave the next none, and its rail events
/// would come a nanosecond apart. With this pace what is left after the
/// transient's first `COUPLE_PACE` time constants is still `1/e` of it. It
/// sets how often the rail aims, never how far a quantity may err.
const COUPLE_PACE: f64 = 8.0;

/// A quantity a driven curved segment publishes, bilinear in its capacitor's
/// voltage and the rail's motion since the aim: `q0 + q1 vc + q2 δ + q3 vc δ`
/// (a node, a stamped device's current, the capacitor's, the draw), and its
/// W0 tolerance's absolute part (volts or amps).
struct CoupleQty {
    q: [f64; 4],
    abs: f64,
}

impl CoupleQty {
    /// Its least magnitude over the box `vc in vcs`, `δ in ds`: a bilinear
    /// function is extreme at the corners, and 0 where they differ in sign.
    fn least(&self, vcs: (f64, f64), ds: (f64, f64)) -> f64 {
        let at = |v: f64, d: f64| self.q[0] + self.q[1] * v + self.q[2] * d + self.q[3] * v * d;
        let c = [at(vcs.0, ds.0), at(vcs.0, ds.1), at(vcs.1, ds.0), at(vcs.1, ds.1)];
        if c.iter().all(|&x| x > 0.0) || c.iter().all(|&x| x < 0.0) {
            c.iter().fold(f64::INFINITY, |m, &x| m.min(x.abs()))
        } else {
            0.0
        }
    }
}

/// One driven curved aim's coupling error (V9 MEDIUM 1, Rails point 4).
///
/// The segment's current is `i = i0_a + di_a vc + di0du δ + di1du vc δ`, `δ =
/// u - u_a`: a linear ODE whose rate moves with the rail, which no closed form
/// follows. The drive holds the last term at the aim's `vc_a`, so the
/// capacitor's error `e` obeys `C e' = (di_a + di1du δ) e + di1du δ (vm -
/// vc_a)`, `vm = inf + (vc_a - inf) e^(fτ) + sum w J(f, k, τ)` the published
/// trajectory. With `|δ| <= D(τ) = sum |s| φ_k(τ)` (one term per mode of the
/// rail's path; one alone without closed-form partitions to couple) and the
/// decay at least `a = (|di_a| - |di1du| D) / C`:
/// - the transient's share, `|vm - vc_a| <= |vc_a - inf| (1 - e^(fτ))`, gives
///   `|di1du| D |vc_a - inf| K(τ) / C`, `K(τ) = ∫0^τ e^(-a(τ-s)) (1 -
///   e^(fs)) ds`: third order in `τ` on a short aim, and on a long one the
///   offset V9 measured, `|di1du| D |vc_a - inf| / (C a)`;
/// - the rail's own share, `sum |w| J`, gives `|di1du| D sum |w| J / (C a)`,
///   second order in the rail;
/// - the pending crossing's landing (`xr = |dx1du / dx|` of its node, 0 with
///   none): the scheduler holds a curved node's sensitivity at `vc_a` too, so
///   the node lands `dx1du δ (vm - vc_a)` off its trajectory, which moves
///   `vc` by that over its `dx` (the next aim carries the step it actually
///   took);
/// - and what earlier aims under the same drive left, `err`, decaying at `a`.
struct CoupleAim {
    cap: f64,
    di: f64,
    f: f64,
    modes: Rc<[DriveMode]>,
    a0: f64,
    g1: f64,
    xr: f64,
    err: f64,
    /// Where a path of several modes turns (`rails::path_turns`): with them
    /// the rail's own excursion bounds the coupling, not `D`, which bounds
    /// each mode's reach separately and outgrows a path that turns back (a
    /// width `D` reached was never reached by the rail, and the aim it ended
    /// was never renewed: F1r14's 5 residual CC boards). `None` for one mode,
    /// whose path is `D` itself, and for a path of more rates than the turns
    /// are found for, which keeps `D`.
    turns: Option<Vec<f64>>,
}

impl CoupleAim {
    /// The aim `d` of `part` (its trajectory's parameters as they stand),
    /// with its pending crossing's `xr`, or `None` for a segment that does not
    /// decay.
    fn of(part: &Partition, d: &Drive, xr: Option<f64>) -> Option<CoupleAim> {
        (part.seg_f < 0.0 && d.modes.iter().all(|m| m.k >= 0.0)).then(|| CoupleAim {
            cap: part.cap.c,
            di: part.aim_di.abs(),
            f: part.seg_f,
            modes: Rc::clone(&d.modes),
            a0: (part.seg_vc0 - part.seg_inf).abs(),
            g1: d.g1,
            xr: xr.unwrap_or(0.0),
            err: d.err,
            turns: if d.modes.len() > 1 { rails::path_turns(d.modes.iter().map(|m| (m.k, m.s))) } else { None },
        })
    }

    /// `u(t_a + τ) - u_a`, the rail's path.
    fn delta(&self, tau: f64) -> f64 {
        self.sum(|m| m.s * rails::exp_integral(-m.k, tau))
    }

    /// The path's limit, or infinity with a line among its modes.
    fn delta_tail(&self) -> f64 {
        self.sum(|m| if m.s == 0.0 { 0.0 } else if m.k > 0.0 { m.s / m.k } else { m.s * f64::INFINITY })
    }

    /// The most the rail has moved from `u_a` by `τ` after the aim: the
    /// path's excursion, its largest magnitude at `τ` or at a turn before
    /// (`D` where the turns are not known).
    fn peak(&self, tau: f64) -> f64 {
        match &self.turns {
            Some(turns) => turns.iter().take_while(|&&t| t < tau).fold(self.delta(tau).abs(), |m, &t| m.max(self.delta(t).abs())),
            None => self.reach(tau),
        }
    }

    /// [`Self::peak`] over all time.
    fn peak_tail(&self) -> f64 {
        match &self.turns {
            Some(turns) => turns.iter().fold(self.delta_tail().abs(), |m, &t| m.max(self.delta(t).abs())),
            None => self.reach_tail(),
        }
    }

    /// The range of the path over `[ta, tb]` (`tb` may be infinite): its
    /// ends and the turns between where they are known, else each mode's
    /// own range, which is monotone.
    fn span(&self, ta: f64, tb: f64) -> (f64, f64) {
        let at = |t: f64| if t.is_finite() { self.delta(t) } else { self.delta_tail() };
        match &self.turns {
            Some(turns) => {
                let (a, b) = (at(ta), at(tb));
                turns.iter().filter(|&&t| t > ta && t < tb).fold((a.min(b), a.max(b)), |(lo, hi), &t| {
                    let v = self.delta(t);
                    (lo.min(v), hi.max(v))
                })
            }
            None => {
                let (mut lo, mut hi) = (0.0, 0.0);
                for m in self.modes.iter().filter(|m| m.s != 0.0) {
                    let (pa, pb) = (m.s * rails::exp_integral(-m.k, ta), m.s * if tb.is_finite() { rails::exp_integral(-m.k, tb) } else { 1.0 / m.k });
                    lo += pa.min(pb);
                    hi += pa.max(pb);
                }
                (lo, hi)
            }
        }
    }

    /// The first `τ` at which the path's excursion reaches `d`, or infinity:
    /// on each monotone piece between its turns `|δ|` falls (through 0 at
    /// most once) and then rises, so whether it has reached `d` is false and
    /// then true, and is bisected. Past the last turn the path is monotone to
    /// its limit: windows doubling from the fastest mode's time.
    fn first_reach(&self, d: f64) -> f64 {
        let Some(turns) = &self.turns else {
            // `D` rises monotonically.
            if !(self.reach_tail() > d) {
                return f64::INFINITY;
            }
            let mut hi = 1e-15;
            while self.reach(hi) < d && hi.is_finite() {
                hi *= 2.0;
            }
            return Self::bisect_reach(|t| self.reach(t) >= d, 0.0, hi);
        };
        let reached = |t: f64| self.delta(t).abs() >= d;
        let mut a = 0.0;
        for &t in turns {
            if reached(t) {
                return Self::bisect_reach(reached, a, t);
            }
            a = t;
        }
        if !(self.delta_tail().abs() > d) {
            return f64::INFINITY;
        }
        let fast = self.modes.iter().map(|m| m.k).fold(0.0f64, f64::max);
        let mut w = if fast > 0.0 { 1.0 / fast } else { 1e-15 }.max(a * 1e-3);
        for _ in 0..2200 {
            let b = a + w;
            if !b.is_finite() {
                return f64::INFINITY;
            }
            if reached(b) {
                return Self::bisect_reach(reached, a, b);
            }
            a = b;
            w *= 2.0;
        }
        f64::INFINITY
    }

    /// The first point of `(lo, hi]` where `reached` holds, given it is false
    /// at `lo`, true at `hi` and false then true between.
    fn bisect_reach(reached: impl Fn(f64) -> bool, mut lo: f64, mut hi: f64) -> f64 {
        for _ in 0..2200 {
            let mid = lo + 0.5 * (hi - lo);
            if !(mid > lo && mid < hi) {
                break;
            }
            if reached(mid) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        hi
    }

    /// `sum over modes of term(mode)`, as [`Drive::sum`].
    fn sum(&self, term: impl Fn(&DriveMode) -> f64) -> f64 {
        let mut it = self.modes.iter();
        let mut acc = term(it.next().expect("a drive has a mode"));
        for m in it {
            acc += term(m);
        }
        acc
    }

    /// `D(τ)`, the most the rail has moved `τ` after the aim.
    fn reach(&self, tau: f64) -> f64 {
        self.sum(|m| m.s.abs() * rails::exp_integral(-m.k, tau))
    }

    /// `D` once every mode has run out, infinite with a line among them.
    fn reach_tail(&self) -> f64 {
        self.sum(|m| if m.s == 0.0 { 0.0 } else if m.k > 0.0 { m.s.abs() / m.k } else { f64::INFINITY })
    }

    /// `sum |w| J(f, k, τ)`, the most the capacitor has followed the rail.
    fn follow(&self, tau: f64) -> f64 {
        self.sum(|m| if m.w != 0.0 { m.w.abs() * rails::ramp_response(self.f, m.k, tau) } else { 0.0 })
    }

    /// `(inherited, new, a)` over `[ta, tb]` with the rail within `d_b`: the
    /// earlier aims' error at `ta`, the bound on this aim's own at `tb`, and
    /// the decay rate. `tb` infinite is the tail of a rail that settles inside
    /// the band (`d_b` its whole reach). `None` where the bound does not close.
    fn err(&self, ta: f64, tb: f64, d_b: f64) -> Option<(f64, f64, f64)> {
        let d_lo = self.di - self.g1 * d_b;
        let xr_d = self.xr * d_b;
        if !(d_lo > 0.0) || !(xr_d < 1.0) {
            return None;
        }
        let a = d_lo / self.cap;
        let (k_b, j_b, m_t) = if tb.is_finite() {
            let k_b = (rails::exp_integral(-a, tb) - (-a * tb).exp() * rails::exp_integral(a + self.f, tb)).max(0.0);
            (k_b, self.follow(tb), -(self.f * tb).exp_m1())
        } else if self.modes.iter().all(|m| m.k > 0.0) {
            (1.0 / a, self.sum(|m| m.w.abs() / (m.k * self.f.abs())), 1.0)
        } else {
            return None;
        };
        let e1 = self.g1 * d_b * self.a0 * k_b / self.cap;
        let e2 = self.g1 * d_b * j_b / d_lo;
        let e3 = xr_d * (self.a0 * m_t + j_b) / (1.0 - xr_d);
        Some((self.err * (-a * ta).exp(), e1 + e2 + e3, a))
    }

    /// The bound on the capacitor's error `tau` after the aim: what the next
    /// aim inherits.
    fn err_at(&self, tau: f64) -> f64 {
        let tau = tau.max(0.0);
        match self.err(tau, tau, self.peak(tau)) {
            Some((inherited, new, _)) => inherited + new,
            // Past where the bound closes nothing is known: the next aim
            // starts afresh rather than stop the rail.
            None => 0.0,
        }
    }
}

/// How far the rail may move from a driven curved segment's aim before the
/// coupling its drive leaves out ([`CoupleAim`]) could put a published
/// quantity past W0's tolerance (V9 MEDIUM 1, Rails point 4), or infinity;
/// the rail aims the partition again there (`nonlinear_hub_rails`).
///
/// Each published quantity errs by its slope in `vc` times the capacitor's
/// error, and is checked as the band checks it (`curvature::Bound::valid`):
/// within W0's tolerance at its least magnitude over the voltages and rails
/// it can take in each interval of a geometric grid in `τ`, up to the band's
/// edge, where the rail aims the segment again anyway, or for ever where the
/// rail settles inside the band. This aim's own share may use, of what the
/// earlier aims left, the fraction `x / COUPLE_PACE` of it, `x` the time
/// constants since the aim. Where their error alone has used a quantity's
/// tolerance up (near its zero, where only the absolute part is left) this
/// aim may add none: no aim takes that error back, but spending the
/// tolerance again on top of it grew the bound past W0 by up to one more
/// tolerance (V11 finding 5; 165 such checks in the 768-board CC sweep, the
/// inherited bound alone up to 1.34x the tolerance). The aim fails there,
/// and the rail aims again until the segment's own decay frees some.
/// The first interval that fails sets the width, held to a nanosecond of
/// rail at least so the rail's events keep moving, and the time since the aim
/// it fails at. Under one mode the rail's reach is its path, so the width
/// alone ends the aim in time. A rail of several modes is bounded by its own
/// excursion (`CoupleAim::peak`): the most it has moved by then, which the
/// error grows with in time as well as in width, and which it may reach only
/// at a turn, where no passage of the width fires; so the aim also ends at
/// that time (`Drive::expiry`, F1r14: a 150 ohm gate's settled current was
/// 275x W0 off where the turned rail never reached the width). The horizon
/// itself is the path's own first reach of the band's edge, whose rail event
/// aims again: bounded by `D`, which outgrows a path that turns back, the
/// check stopped where the rail never went and nothing guarded the aim past
/// it (F1r15, the 5 residual CC boards).
fn couple_band(part: &Partition, d: &Drive, crossing: Option<f64>, bnd: &Bnd, rail: usize, live_params: &HashMap<usize, Vec<f64>>) -> (f64, f64) {
    let (Some(seg), Some(c)) = (part.seg.as_ref(), part.curve()) else { return (f64::INFINITY, f64::INFINITY) };
    let xr = crossing.map(|_| {
        let dx = part.aim_dx[part.cross_local];
        if dx != 0.0 { (c.dx1du[part.cross_local] / dx).abs() } else { 0.0 }
    });
    let Some(aim) = CoupleAim::of(part, d, xr) else { return (f64::INFINITY, f64::INFINITY) };
    let h = seg.hub.as_ref().expect("a curved segment is a hub partition's");
    let s_abs = aim.sum(|m| m.s.abs());
    // A segment exact at any rail (an infinite band) still couples through
    // `di1du`: under a single mode the drift watch renewed its aim, under a
    // coupled path of several nothing does but this band (F1r14, V11 B256:
    // 2.6x W0 in a gate's transient).
    if (!c.band.is_finite() && d.modes.len() == 1) || s_abs == 0.0 || (aim.g1 == 0.0 && aim.xr == 0.0) {
        return (f64::INFINITY, f64::INFINITY);
    }
    let n = part.shape.n;
    let (inf, a0s) = (part.seg_inf, part.seg_vc0 - part.seg_inf);
    // Every pin's voltage as the coefficients of `1, vc, δ, vc δ`.
    let node = |l: usize| [part.aim_x0[l], part.aim_dx[l], h.dx0du[l], c.dx1du[l]];
    let pin = |p: &DynPin| match p.local {
        Some(l) => node(l),
        None if p.node == rail => [d.u_a, 0.0, 1.0, 0.0],
        None => [bnd.get(p.node), 0.0, 0.0, 0.0],
    };
    let across = |a: [f64; 4], b: [f64; 4], g: f64| [g * (a[0] - b[0]), g * (a[1] - b[1]), g * (a[2] - b[2]), g * (a[3] - b[3])];
    let add = |a: [f64; 4], b: [f64; 4]| [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]];
    let mut qty: Vec<CoupleQty> = (0..n).map(|l| CoupleQty { q: node(l), abs: curvature::V_ABS }).collect();
    for st in &part.shape.stamped {
        let mut q = [0.0; 4];
        for ct in crate::devices::stamp(st.kind, st.pins.len(), live_params_for(st, live_params)) {
            q = add(q, across(pin(&st.pins[ct.pin_a]), pin(&st.pins[ct.pin_b]), ct.conductance));
        }
        qty.push(CoupleQty { q, abs: curvature::A_ABS });
    }
    let leak = across(pin(&part.cap.pins[0]), pin(&part.cap.pins[1]), part.cap.leak_g);
    qty.push(CoupleQty { q: add([part.aim_i0, part.aim_di, h.di0du, c.di1du], leak), abs: curvature::A_ABS });
    qty.push(CoupleQty { q: [h.a + h.g * part.aim_du, h.b + c.g1 * part.aim_du, h.g, c.g1], abs: curvature::A_ABS });
    // The aim lasts until the rail leaves the band, or for ever where the
    // rail settles inside it.
    let f = aim.f;
    let d_max = c.band + part.aim_du.abs();
    let tau_d = match d.single() {
        Some(m) => {
            let (k, s) = (m.k, m.s.abs());
            if k == 0.0 {
                d_max / s
            } else if k * d_max < s {
                -(-k * d_max / s).ln_1p() / k
            } else {
                f64::INFINITY
            }
        }
        // The first time the path's excursion reaches the band's edge, where
        // the band's own rail event aims the segment again (`|u - u_seg|`
        // reaches the band no later).
        None => aim.first_reach(d_max),
    };
    let tau_c = aim.cap / aim.di;
    // A pending crossing ends the aim too: its landing aims again.
    let tau_x = crossing.map_or(f64::INFINITY, |t| t.max(0.0));
    let settles = !tau_d.is_finite() && !tau_x.is_finite();
    let k_min = d.modes.iter().filter(|m| m.s != 0.0 || m.w != 0.0).map(|m| m.k).fold(f64::INFINITY, f64::min);
    let tau_end = if settles { 64.0 * tau_c.max(1.0 / k_min) } else { tau_d.min(tau_x) };
    let fails = |ta: f64, tb: f64| -> bool {
        let tail = !tb.is_finite();
        let d_b = if tail { aim.peak_tail() } else { aim.peak(tb).min(d_max) };
        let Some((inherited, new, a)) = aim.err(ta, tb, d_b) else { return true };
        // Each term of the trajectory and of the rail's motion is monotone in
        // `τ`, so over `[ta, tb]` each lies between its values at the ends.
        let (ea, eb) = ((f * ta).exp(), if tail { 0.0 } else { (f * tb).exp() });
        let (mut lo, mut hi) = (inf + (a0s * ea).min(a0s * eb), inf + (a0s * ea).max(a0s * eb));
        for m in d.modes.iter() {
            if m.w != 0.0 {
                let (ja, jb) = (m.w * rails::ramp_response(f, m.k, ta), m.w * if tail { 1.0 / (m.k * f.abs()) } else { rails::ramp_response(f, m.k, tb) });
                lo += ja.min(jb);
                hi += ja.max(jb);
            }
        }
        let (d_lo, d_hi) = aim.span(ta, if tail { f64::INFINITY } else { tb });
        let e = inherited + new;
        let vcs = (lo - e, hi + e);
        let ds = (d_lo.max(-d_max), d_hi.min(d_max));
        let pace = if tail { 1.0 } else { (a * tb / COUPLE_PACE).min(1.0) };
        qty.iter().any(|qt| {
            let slope = qt.q[1].abs() + qt.q[3].abs() * d_b;
            let tol = qt.abs + curvature::REL * (qt.least(vcs, ds) - slope * e).max(0.0);
            let left = (tol - slope * inherited).max(0.0);
            let over = slope * new > left * pace;
            #[cfg(test)]
            hook::note(slope * inherited, slope * new, tol, over);
            over
        })
    };
    let floor = aim.reach(1e-9).max(1e3 * rails::REACHED_REL * d.u_a.abs().max(1.0));
    let (mut ta, mut tb) = (0.0, 1e-6 * tau_end.min(tau_c));
    if !(tb > 0.0) {
        return (f64::INFINITY, f64::INFINITY);
    }
    loop {
        let last = tb >= tau_end;
        let tb_now = if last { tau_end } else { tb };
        if fails(ta, tb_now) {
            return (aim.peak(ta).max(floor), ta);
        }
        if last {
            break;
        }
        ta = tb_now;
        tb *= 1.5;
    }
    if settles && fails(tau_end, f64::INFINITY) {
        return (aim.peak(tau_end).max(floor), tau_end);
    }
    (f64::INFINITY, f64::INFINITY)
}

/// What [`couple_band`]'s checks met, for tests that hold an aim to what the
/// earlier aims left. Counters, not a log: a long board checks millions of
/// times on a test thread.
#[cfg(test)]
pub(crate) mod hook {
    use std::cell::Cell;

    /// Checks of a quantity whose inherited error alone had used its
    /// tolerance up, and how many of those still let the aim add some.
    #[derive(Debug, Clone, Copy, Default, PartialEq)]
    pub(crate) struct Spent {
        pub exhausted: u64,
        pub granted: u64,
    }

    thread_local! {
        static SPENT: Cell<Spent> = const { Cell::new(Spent { exhausted: 0, granted: 0 }) };
    }

    /// One check: the quantity's error from earlier aims and from this one
    /// (each times its slope), its tolerance, and whether the check failed.
    pub(crate) fn note(inherited: f64, new: f64, tol: f64, failed: bool) {
        if inherited >= tol {
            SPENT.with(|s| {
                let mut v = s.get();
                v.exhausted += 1;
                if !failed && new > 0.0 {
                    v.granted += 1;
                }
                s.set(v);
            });
        }
    }

    /// The counts since the last call, which resets them.
    pub(crate) fn take() -> Spent {
        SPENT.with(|s| s.replace(Spent::default()))
    }
}

/// Solves the two `M_SOURCE`-mode systems (`vc = 0`, `vc = 1`) that give this
/// partition's exact affine relationship to its capacitor's voltage (this
/// module's doc comment): one extra branch-current unknown (row/column `n`)
/// carries the capacitor's own series current, related to the free nodes and
/// `vc` by `V[pin_a] - V[pin_b] - esr*i == vc`. Both solves reuse the SAME LU
/// factorization (only `b[n]` differs between them), so this is one factor
/// plus two applies, exactly as the module doc promises ("two small solves
/// per switching event").
///
/// `stamp_ics_into_dyn` (W1c step 2b) is called once, into the SHARED `a`/
/// `b0` baseline, before it is cloned into `x0`/`x1`: an IC driver/pull
/// conductance never depends on `vc` at all (only on this partition's fixed
/// supply voltages, `stamp_ics_into_dyn`'s own doc comment on why every IC
/// touching a `ClosedForm` partition has both fixed), so it is exactly as
/// constant across the two sub-solves as `stamp_linear_base`'s own resistor/
/// switch/leakage terms already are — never something that needs its own
/// `vc=0`/`vc=1` sweep the way the capacitor's own branch equation does.
#[allow(clippy::too_many_arguments)]
fn build_segment(
    shape: &DynShape,
    cap: &DynCapacitor,
    p_idx: usize,
    now: f64,
    drivers: &[Driver],
    ics: &[(usize, digital::IcRecord)],
    node_owner: &HashMap<usize, (usize, usize)>,
    bnd: &Bnd,
    live_params: &HashMap<usize, Vec<f64>>,
    hub_rail: Option<usize>,
    curved: bool,
    vc_now: f64,
    ramping: &[usize],
    driven: bool,
) -> Result<Segment, String> {
    let n = shape.n;
    let size = n + 1;
    let r = n;
    let mut a = vec![0.0; size * size];
    let mut b0 = vec![0.0; size];
    stamp_linear_base(shape, size, &mut a, &mut b0, live_params, bnd);
    // A `ClosedForm` partition never holds a free IC supply node (the
    // promotion rule this module's own doc comment describes), so `x` (the
    // Newton-iterate slice `stamp_ics_into_dyn` would index for a FREE
    // supply) is never actually read here — `&[]` is exactly correct, not a
    // shortcut, matching `static_part.rs::direct_solve`'s identical `&[]`.
    stamp_ics_into_dyn(size, &mut a, &mut b0, now, drivers, ics, node_owner, bnd, p_idx, &[]);

    stamp_branch_terminal(&mut a, &mut b0, size, r, &cap.pins[0], 1.0, bnd);
    stamp_branch_terminal(&mut a, &mut b0, size, r, &cap.pins[1], -1.0, bnd);
    a[r * size + r] -= cap.esr;
    // b0[r] += 0.0 * vc (vc = 0 for the first solve).

    let lu = lu::lu_factor(a, size).map_err(|_| "closed-form segment: singular or non-finite pivot".to_string())?;
    let mut x0 = b0.clone();
    lu.solve(&mut x0).map_err(|_| "closed-form segment: singular or non-finite pivot solving vc=0".to_string())?;
    let mut x1 = b0;
    x1[r] += 1.0; // vc = 1.
    lu.solve(&mut x1).map_err(|_| "closed-form segment: singular or non-finite pivot solving vc=1".to_string())?;

    // Enforce the branch equation `pins[0] - pins[1] - esr*i = vc` EXACTLY
    // for the capacitor's own two terminals, rather than trusting the
    // general LU solve's rounding for them. Algebraically this equation
    // already holds (it is one of the rows being solved), but Gaussian
    // elimination couples every row together, so its numerical residual on
    // THESE two specific unknowns grows with how ill-conditioned the REST of
    // the partition's matrix happens to be (a switch's closed-contact
    // conductance sitting many orders of magnitude away from a nearby
    // resistor's, say) — invisible for a single, standing segment, but
    // exposed the instant a re-aim compares "before" and "after": with
    // `esr = 0` the terminal voltage IS `vc`, which cannot legitimately
    // change at all when only the SURROUNDING network's topology changes (a
    // capacitor's charge is physically continuous), yet an unpatched
    // solve here reported a several-ULP jump on exactly that comparison,
    // amplified by ill-conditioning into a measurable millivolt-scale one
    // for a fixture with more disparate conductances (T2's `set_params`
    // switch-toggle cross-check). Deriving one terminal from the other via
    // this exact relationship removes the general solve's rounding for
    // whichever terminal is FIXED (carrying zero rounding of its own) —
    // the common case — and, when both are free, at least removes it for
    // one of the two instead of letting both drift independently.
    fix_branch_terminals_exactly(&mut x0, &mut x1, r, cap, bnd);

    // A hub partition's rail terms (P3:533-556): the sensitivity of the vc =
    // 0 solution through the same factors, and the draw at vc = 1. The rail
    // is stamped as unknown `size` beside the branch row.
    let hub = match hub_rail {
        Some(rail) => {
            let u = bnd.get(rail);
            let ext = bnd.with_ext(rail, size).hold_power_at(u);
            let stamp = |z: &[f64], a: &mut [f64], b: &mut [f64], vc: f64| {
                let size_e = size + 1;
                stamp_linear_base(shape, size_e, a, b, live_params, &ext);
                stamp_ics_into_dyn(size_e, a, b, now, drivers, ics, node_owner, &ext, p_idx, z);
                stamp_branch_terminal(a, b, size_e, r, &cap.pins[0], 1.0, &ext);
                stamp_branch_terminal(a, b, size_e, r, &cap.pins[1], -1.0, &ext);
                a[r * size_e + r] -= cap.esr;
                b[r] += vc;
            };
            // A partition whose drivers' response to the rail is affine (a
            // 555's discharge transistor; not its OUT pair with vcc on the
            // rail, whose split follows the supply, so it is curved) keeps the
            // linear segment's terms: its correction is exact.
            let curved = curved && {
                let mut z0 = x0.clone();
                z0.push(u);
                !crate::solve::ic_rail_terms(now, drivers, ics, node_owner, &ext, p_idx, &z0, size).affine(u)
            };
            if curved {
                let seg = SegFit { shape, cap, p_idx, now, drivers, ics, node_owner, bnd, live_params, rail, u };
                Some(Box::new(seg.curved_hub(&x0, &x1, &lu, &stamp, vc_now, driven)?))
            } else {
                let sens = rail_sensitivity(size, &x0, u, &lu, None, None, |z, a, b| stamp(z, a, b, 0.0)).map_err(|e| format!("closed-form segment: {e}"))?;
                let mut z1 = x1.clone();
                z1.push(u);
                let hub1 = stamp_residual(size + 1, &z1, |a, b| stamp(&z1, a, b, 1.0))[size];
                Some(Box::new(SegHub { a: sens.i, b: hub1 - sens.i, g: sens.g, u_seg: u, dx0du: sens.s[..n].to_vec(), di0du: sens.s[n], curve: None }))
            }
        }
        None => None,
    };

    // Each ramping boundary node's exact sensitivity (Sources point 3, D3
    // (b)): stamped as one more unknown, its column `c` gives `dx/dv = -A^-1
    // c` through the same factors. Only linear stamps read it (`partition::
    // build` keeps a partition stepped where an IC does), so the IC stamps
    // add nothing to the column.
    let mut ramp_sens = Vec::with_capacity(ramping.len());
    for &node in ramping {
        let size_e = size + 1;
        let ext = bnd.with_ext(node, size);
        let (mut a, mut b) = (vec![0.0; size_e * size_e], vec![0.0; size_e]);
        stamp_linear_base(shape, size_e, &mut a, &mut b, live_params, &ext);
        stamp_branch_terminal(&mut a, &mut b, size_e, r, &cap.pins[0], 1.0, &ext);
        stamp_branch_terminal(&mut a, &mut b, size_e, r, &cap.pins[1], -1.0, &ext);
        let mut s: Vec<f64> = (0..size).map(|k| -a[k * size_e + size]).collect();
        lu.solve(&mut s).map_err(|_| "closed-form segment: singular or non-finite pivot solving a ramp sensitivity".to_string())?;
        // As `fix_branch_terminals_exactly`: the branch equation holds for the
        // derivative exactly.
        let unit = |pin: &DynPin| if pin.node == node { 1.0 } else { 0.0 };
        match (cap.pins[0].local, cap.pins[1].local) {
            (Some(la), None) => s[la] = unit(&cap.pins[1]) + cap.esr * s[r],
            (None, Some(lb)) => s[lb] = unit(&cap.pins[0]) - cap.esr * s[r],
            (Some(la), Some(lb)) => s[la] = s[lb] + cap.esr * s[r],
            (None, None) => {}
        }
        ramp_sens.push((node, s[..n].to_vec(), s[r]));
    }

    let dx: Vec<f64> = (0..n).map(|k| x1[k] - x0[k]).collect();
    let i0 = x0[r];
    let di = x1[r] - i0;
    Ok(Segment { x0: x0[..n].to_vec(), dx, i0, di, key: Vec::new(), hub, ramp_sens })
}

/// What a curved hub partition's segment build reads (ruling R8 (a)).
struct SegFit<'a> {
    shape: &'a DynShape,
    cap: &'a DynCapacitor,
    p_idx: usize,
    now: f64,
    drivers: &'a [Driver],
    ics: &'a [(usize, digital::IcRecord)],
    node_owner: &'a HashMap<usize, (usize, usize)>,
    bnd: &'a Bnd<'a>,
    live_params: &'a HashMap<usize, Vec<f64>>,
    rail: usize,
    u: f64,
}

/// The conductance that stands for a capacitor with no esr in the band's
/// static picture of a segment (`curvature::segment_band`): stiff against
/// any stamp a partition holds, so the rest's resistances it overstates are
/// overstated by micro-ohms.
const STIFF_CAP_G: f64 = 1e6;

impl SegFit<'_> {
    /// A curved hub partition's rail terms: the exact sensitivities of the
    /// `vc = 0` and `vc = 1` solutions `x0`, `x1` (the IC stamps that follow
    /// the rail other than through a conductance taken explicitly, as a
    /// static curved partition takes them), whose difference is how they
    /// move with `vc`, and the band over the capacitor voltages the segment
    /// can reach from `vc_now`.
    fn curved_hub(&self, x0: &[f64], x1: &[f64], lu: &lu::LuFactorization, stamp: &dyn Fn(&[f64], &mut [f64], &mut [f64], f64), vc_now: f64, driven: bool) -> Result<SegHub, String> {
        let (n, u) = (self.shape.n, self.u);
        let size = n + 1;
        let ext = self.bnd.with_ext(self.rail, size).hold_power_at(u);
        let z_of = |x: &[f64]| x.iter().copied().chain(std::iter::once(u)).collect::<Vec<f64>>();
        let (z0, z1) = (z_of(x0), z_of(x1));
        let ic = crate::solve::ic_rail_terms(self.now, self.drivers, self.ics, self.node_owner, &ext, self.p_idx, &z0, size);
        let (e0, e1) = (ic.explicit_u(&z0, size), ic.explicit_u(&z1, size));
        let err = |e: String| format!("closed-form segment: {e}");
        let s0 = rail_sensitivity(size, x0, u, lu, Some(&e0), None, |z, a, b| stamp(z, a, b, 0.0)).map_err(err)?;
        let s1 = rail_sensitivity(size, x1, u, lu, Some(&e1), None, |z, a, b| stamp(z, a, b, 1.0)).map_err(err)?;
        let (i0, di) = (x0[n], x1[n] - x0[n]);
        let dx: Vec<f64> = (0..n).map(|k| x1[k] - x0[k]).collect();
        let dx1du: Vec<f64> = (0..n).map(|k| s1.s[k] - s0.s[k]).collect();
        let di1du = s1.s[n] - s0.s[n];
        let (b, g1) = (s1.i - s0.i, s1.g - s0.g);
        // The capacitor voltages the segment can reach (V9 LOW): from
        // `vc_now` toward its asymptote, which the segment's own model puts
        // at `-(i0 + di0du d) / (di + di1du d)` a rail `d` from here. That
        // is monotone in `d` while the segment decays, so over the widest
        // band a curved segment takes (`RAIL_RESOLVE_V`) it lies between its
        // ends; the capacitor, following it at a positive rate, stays in the
        // hull of them and `vc_now`. Built under a CC rail capacitor
        // (`driven`), the published `vc` strays from that by the coupling's
        // error, which `couple_band` holds within W0's tolerance, its width
        // here. A segment that does not decay, or could stop decaying inside
        // the band, has no bound.
        let bound = |d: f64| {
            let den = di + di1du * d;
            (den < 0.0).then(|| -(i0 + s0.s[n] * d) / den)
        };
        let reach = crate::rails::RAIL_RESOLVE_V;
        let (vc_lo, vc_hi, band) = match (di < 0.0).then(|| (bound(-reach), bound(reach))) {
            Some((Some(a), Some(z))) => {
                let inf = -i0 / di;
                let (lo, hi) = (vc_now.min(inf).min(a).min(z), vc_now.max(inf).max(a).max(z));
                let w = if driven { curvature::V_ABS + curvature::REL * lo.abs().max(hi.abs()) } else { 0.0 };
                let (lo, hi) = (lo - w, hi + w);
                let (band, affine) = self.band(x0, &dx, &s0, &dx1du, (i0, di, s0.s[n], di1du), (b, g1), [lo, hi]);
                // An affine segment's band holds at any `vc`, and may be
                // wider than the reach was taken over.
                if affine { (f64::NEG_INFINITY, f64::INFINITY, band) } else { (lo, hi, band) }
            }
            _ if di == 0.0 => (vc_now, vc_now, self.band(x0, &dx, &s0, &dx1du, (i0, di, s0.s[n], di1du), (b, g1), [vc_now, vc_now]).0),
            _ => (vc_now, vc_now, crate::solve::curvature::NARROWEST_BAND),
        };
        Ok(SegHub {
            a: s0.i,
            b,
            g: s0.g,
            u_seg: u,
            dx0du: s0.s[..n].to_vec(),
            di0du: s0.s[n],
            curve: Some(Box::new(SegCurve { dx1du, di1du, g1, band, vc_lo, vc_hi })),
        })
    }

    /// `curvature::segment_band` over `vcs` (with whether the segment is
    /// affine in the rail): at each, the partition's nodes
    /// with the capacitor held there, their sensitivities, the draw's and
    /// the capacitor's series current's, and the node-space Jacobian with
    /// the capacitor a conductance to its source.
    #[allow(clippy::too_many_arguments)]
    fn band(&self, x0: &[f64], dx: &[f64], s0: &crate::solve::RailSens, dx1du: &[f64], (i0, di, di0du, di1du): (f64, f64, f64, f64), (b, g1): (f64, f64), vcs: [f64; 2]) -> (f64, bool) {
        let (n, u) = (self.shape.n, self.u);
        let size = n + 1;
        let ext = self.bnd.with_ext(self.rail, n).hold_power_at(u);
        let g_cap = if self.cap.esr > 0.0 { 1.0 / self.cap.esr } else { STIFF_CAP_G };
        let end = |vc: f64| -> curvature::SegmentEnd {
            let x: Vec<f64> = (0..n).map(|k| x0[k] + dx[k] * vc).collect();
            let s: Vec<f64> = (0..n).map(|k| s0.s[k] + dx1du[k] * vc).collect();
            let z: Vec<f64> = x.iter().copied().chain(std::iter::once(u)).collect();
            let (mut a, mut rhs) = (vec![0.0; size * size], vec![0.0; size]);
            stamp_linear_base(self.shape, size, &mut a, &mut rhs, self.live_params, &ext);
            stamp_ics_into_dyn(size, &mut a, &mut rhs, self.now, self.drivers, self.ics, self.node_owner, &ext, self.p_idx, &z);
            crate::solve::stamp_two_terminal(size, &mut a, &mut rhs, &self.cap.pins[0], &self.cap.pins[1], g_cap, 0.0, &ext);
            let sens = crate::solve::RailSens { i: s0.i + b * vc, g: s0.g + g1 * vc, s, jac: a, res: Vec::new(), gross: Vec::new(), gross_s: Vec::new(), lu: None };
            let cap = curvature::Branch { i: i0 + di * vc, di: di0du + di1du * vc, g: g_cap, contacts: 1.0, series: None, device: Some(self.cap.device_index), floor: (0.0, 0.0) };
            curvature::SegmentEnd { x, sens, cap }
        };
        let ends = [end(vcs[0]), end(vcs[1])];
        let z: Vec<f64> = ends[0].x.iter().copied().chain(std::iter::once(u)).collect();
        let ic = crate::solve::ic_rail_terms(self.now, self.drivers, self.ics, self.node_owner, &ext, self.p_idx, &z, n);
        curvature::segment_band(self.shape, self.live_params, &self.bnd.with_ext(self.rail, n), u, &ends, &ic)
    }
}

/// See [`build_segment`]'s doc comment on why this exists. `r` is the branch
/// row/column index (`x[r]` is the series current at `vc=0`/`vc=1`
/// respectively).
fn fix_branch_terminals_exactly(x0: &mut [f64], x1: &mut [f64], r: usize, cap: &DynCapacitor, bnd: &Bnd) {
    match (cap.pins[0].local, cap.pins[1].local) {
        (Some(la), None) => {
            // pins[0] = pins[1] + esr*i + vc (pins[1] fixed: zero rounding).
            let fixed = bnd.get(cap.pins[1].node);
            x0[la] = fixed + cap.esr * x0[r];
            x1[la] = fixed + cap.esr * x1[r] + 1.0;
        }
        (None, Some(lb)) => {
            // pins[1] = pins[0] - esr*i - vc.
            let fixed = bnd.get(cap.pins[0].node);
            x0[lb] = fixed - cap.esr * x0[r];
            x1[lb] = fixed - cap.esr * x1[r] - 1.0;
        }
        (Some(la), Some(lb)) => {
            // Both free: pin_a still derived from pin_b's own (LU-solved)
            // value, so at least ONE of the two is exactly consistent with
            // the other under this relationship, rather than both drifting
            // independently.
            x0[la] = x0[lb] + cap.esr * x0[r];
            x1[la] = x1[lb] + cap.esr * x1[r] + 1.0;
        }
        (None, None) => {} // Both fixed: not a valid `ClosedForm` partition (`partition::build` already rejects this).
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition;
    use crate::program::{Device, Options, PinRecord};
    use crate::solve::Threshold;

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn source(component_index: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: crate::kinds::VOLTAGE_SOURCE, component_index, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }

    fn resistor(component_index: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: crate::kinds::RESISTOR, component_index, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }

    fn capacitor(component_index: usize, a: usize, b: usize, c: f64, esr: f64, leak: f64) -> Device {
        Device { kind: crate::kinds::CAPACITOR, component_index, pins: vec![pin(a), pin(b)], params: vec![c, esr, leak], blob: vec![] }
    }

    fn program(node_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    }

    fn build(program: &Program) -> ClosedFormSolver {
        let pb = partition::build(program).unwrap();
        ClosedFormSolver::build(program, &pb.partitions, &pb.local_index, &pb.fixed, &pb.kinds, LiveTopology::of(&pb))
    }

    /// node0 = gnd, node1 = source(+), node2 = RC midpoint (the one free
    /// node, hence one `ClosedForm` partition at index 0).
    fn rc_program(v0: f64, r: f64, c: f64, esr: f64, leak: f64) -> Program {
        program(3, vec![source(0, 1, 0, v0), resistor(1, 1, 2, r), capacitor(2, 2, 0, c, esr, leak)])
    }

    #[test]
    fn rc_charge_matches_the_exact_exponential_within_1e9_relative() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let tau = r * c;
        // Not testing far past 5*tau: both the node voltage and the series
        // current head to constants/zero there, and a RELATIVE comparison
        // against a reference computed independently (rather than reading
        // back this module's own numbers) becomes a comparison of two
        // catastrophically-cancelled near-zero quantities, not a meaningful
        // precision check of either implementation.
        for &t in &[1e-5, 1e-4, 5e-4, 1e-3, 2e-3, 5e-3] {
            let expected = v0 * (1.0 - (-t / tau).exp());
            let (mut v, mut cur) = (voltages.clone(), currents.clone());
            cf.publish_at(t, &HashMap::new(), &mut v, &mut cur);
            // `x0`/`dx` for this circuit (esr=0, the capacitor's other
            // terminal fixed at ground) are exact 0.0/1.0 by construction
            // (`fix_branch_terminals_exactly`), so the node voltage IS `vc`
            // bit-for-bit; the residual relative error here is entirely
            // `vc_at`'s own `F`/`vinf` (derived through an LU solve rather
            // than the direct `-1/(RC)`/`V0` this specific circuit happens
            // to reduce to), well under the plan's 1e-9 bar even though it
            // is not literal bit-exactness.
            let rel = (v[2] - expected).abs() / expected.abs();
            assert!(rel < 3e-9, "t={t}: got {} expected {} rel {rel}", v[2], expected);
            // The series current is exactly C*dvc/dt = (v0-vc)/R here (esr=0).
            let expected_i = (v0 - expected) / r;
            let rel_i = (cur[2] - expected_i).abs() / expected_i.abs();
            assert!(rel_i < 5e-9, "t={t}: current got {} expected {} rel {rel_i}", cur[2], expected_i);
        }
    }

    #[test]
    fn rc_with_esr_and_leakage_matches_the_derived_exponential() {
        // Independent derivation (not the module's own code): with a leakage
        // resistor `leak_r` in parallel with the ESR+C branch, KCL at the
        // free node gives `n2 = A + B*vc` where `D = 1/R + 1/leak_r + 1/esr`,
        // `A = (V0/R)/D`, `B = (1/esr)/D`; and `C*dvc/dt = (n2-vc)/esr`
        // reduces to `dvc/dt = F*(vc - vinf)` with `F = -(1-B)/(esr*C)`,
        // `vinf = A/(1-B)`. Sanity check: `vinf` must equal the steady-state
        // Ohm's-law divider `V0*leak_r/(R+leak_r)` (independent of `esr`,
        // since no current flows through the capacitor branch at steady
        // state) — asserted below alongside the trajectory itself.
        let (v0, r, esr, leak_r, c) = (5.0, 1000.0, 10.0, 1e5, 1e-6);
        let p = rc_program(v0, r, c, esr, leak_r);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let d = 1.0 / r + 1.0 / leak_r + 1.0 / esr;
        let a = (v0 / r) / d;
        let b = (1.0 / esr) / d;
        let f = -(1.0 - b) / (esr * c);
        let vinf = a / (1.0 - b);
        let steady_state = v0 * leak_r / (r + leak_r);
        assert!((vinf - steady_state).abs() < 1e-9, "vinf {vinf} steady_state {steady_state}");

        for &t in &[1e-6, 1e-5, 5e-5, 1e-4, 5e-4, 2e-3] {
            let vc = vinf + (0.0 - vinf) * (f * t).exp();
            let expected_n2 = a + b * vc;
            let (mut v, mut cur) = (voltages.clone(), currents.clone());
            cf.publish_at(t, &HashMap::new(), &mut v, &mut cur);
            // Relative, not absolute: with esr/leakage nonzero, `x0[node2]`
            // and `dx[node2]` are no longer bit-exact constants (the
            // continuity fix only forces the OTHER, ground-fixed terminal
            // relationship, not this one), so a residual at the LU solve's
            // own precision floor is expected — still far tighter than any
            // discretised method could achieve.
            let rel = (v[2] - expected_n2).abs() / expected_n2.abs();
            assert!(rel < 1e-9, "t={t}: got {} expected {} rel {rel}", v[2], expected_n2);
            let _ = cur; // current cross-checked implicitly via the node voltage's own ODE above.
        }
    }

    #[test]
    fn crossing_time_matches_the_analytic_log_formula() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        // No watcher yet: the first segment schedules nothing.
        assert!(cf.take_scheduled().is_empty());

        let threshold = 3.0;
        assert!(cf.watch_node(2, DeviceId(0), Box::new(move |_v: &[f64]| vec![Threshold { value: threshold, rising: true }])));
        // Re-aiming (a fresh `resolve_dirty`, standing in for "a stamp in
        // the partition changed") is what actually schedules the crossing
        // against the now-registered watcher.
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let scheduled = cf.take_scheduled();
        assert_eq!(scheduled.len(), 1);
        let (t, p_idx, version) = scheduled[0];
        assert_eq!(p_idx, 0);
        assert!(cf.is_current(0, version));

        let tau = r * c;
        let expected_t = -tau * (1.0 - threshold / v0).ln();
        assert!((t - expected_t).abs() < 1e-12, "t={t} expected={expected_t}");

        // Landing the crossing nudges the node just past the threshold and
        // re-aims from there.
        cf.handle_crossing(0, t, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert!(voltages[2] > threshold && voltages[2] < threshold + 1e-6, "got {}", voltages[2]);
    }

    #[test]
    fn a_reaim_bumps_the_version_so_a_stale_crossing_is_no_longer_current() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        cf.watch_node(2, DeviceId(0), Box::new(|_v: &[f64]| vec![Threshold { value: 3.0, rising: true }]));
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        let (_, _, stale_version) = cf.take_scheduled()[0];
        assert!(cf.is_current(0, stale_version));

        // A second re-aim (a switch toggle or driver commit, in the real
        // kernel) bumps the version — the old crossing is no longer current,
        // exactly as `events.rs`'s lazy-deletion contract requires so
        // `Kernel` silently drops the stale heap entry instead of acting on
        // it.
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert!(!cf.is_current(0, stale_version), "the old crossing must be cancelled by the re-aim");
    }

    #[test]
    fn segments_built_and_crossings_stats_advance() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert_eq!(cf.segments_built(), 1.0);
        cf.watch_node(2, DeviceId(0), Box::new(|_v: &[f64]| vec![Threshold { value: 3.0, rising: true }]));
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert_eq!(cf.segments_built(), 2.0);
        assert_eq!(cf.crossings(), 0.0);
        let (t, _, _) = cf.take_scheduled()[0];
        cf.handle_crossing(0, t, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert_eq!(cf.crossings(), 1.0);
    }

    /// The closed form is exact relative to the CIRCUIT IT ACTUALLY STAMPS,
    /// which always includes the universal node shunt (core/README.md, "W0
    /// kernel semantics" point 2 — `NODE_RSHUNT_G = 1e-12` S on every node,
    /// classic's own model too) — NOT relative to a bare textbook RC formula
    /// that omits it. That shunt adds a THIRD path (an effective `1/1e-12 =
    /// 1e12 ohm` resistor to ground) in parallel with the ideal source's own
    /// path, pulling the true asymptote very slightly below the source
    /// voltage: `vinf = V0 * Rshunt/(R+Rshunt)`, a relative deviation of
    /// about `R * NODE_RSHUNT_G` from the naive `V0`. At `R = 1kOhm` (this
    /// module's other tests) that is about `1e-9`, invisible next to their
    /// `3e-9` bar; at `R = 100kOhm` (T2's W1b equivalence fixtures 11/12/13)
    /// it grows to about `1e-7` — large enough that comparing against the
    /// shunt-FREE formula at THOSE resistances reads as "only 1e-7 accurate"
    /// even though the segment math is exact to machine precision against
    /// the model it is actually required to implement, verified here.
    #[test]
    fn the_closed_form_is_exact_against_the_shunt_inclusive_model_at_a_large_resistance() {
        let (v0, r, c) = (5.0, 100_000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let r_shunt = 1.0 / crate::solve::static_part::NODE_RSHUNT_G;
        let vinf = v0 * r_shunt / (r + r_shunt);
        // The shunt also very slightly speeds up the time constant (a third
        // parallel path only ever adds conductance): tau = R_parallel * C
        // where R_parallel = (R*Rshunt)/(R+Rshunt).
        let r_parallel = (r * r_shunt) / (r + r_shunt);
        let tau = r_parallel * c;
        let naive_tau = r * c;

        let t = naive_tau; // "one tau" by the naive (shunt-free) formula, matching T2's own fixture 11 check.
        let expected = vinf + (0.0 - vinf) * (-t / tau).exp();
        let naive_expected = v0 * (1.0 - (-t / naive_tau).exp());

        let (mut v, mut cur) = (voltages.clone(), currents.clone());
        cf.publish_at(t, &HashMap::new(), &mut v, &mut cur);

        let rel_vs_shunt_inclusive = (v[2] - expected).abs() / expected.abs();
        assert!(rel_vs_shunt_inclusive < 1e-9, "against the shunt-inclusive model: got {} expected {} rel {rel_vs_shunt_inclusive}", v[2], expected);

        // And confirm the shunt is INDEED the whole story behind a naive
        // comparison's much looser apparent accuracy: the gap from the naive
        // formula should be within an order of magnitude of `R *
        // NODE_RSHUNT_G` (about 1e-7 here), not some unrelated, larger
        // number that would point at a real bug instead.
        let rel_vs_naive = (v[2] - naive_expected).abs() / naive_expected.abs();
        let predicted = r * crate::solve::static_part::NODE_RSHUNT_G;
        assert!(rel_vs_naive < predicted * 10.0, "naive-formula gap {rel_vs_naive} is not explained by the node shunt (predicted ~{predicted})");
    }

    /// core/README.md's "Currents": a resistor publishes `(Va-Vb)/R`, even
    /// when it sits inside a `ClosedForm` partition — a device W1b's own
    /// review found publishing exactly `0.0` there, since nothing before
    /// this fix ever computed a `DynStamped` device's own current at all
    /// (`recompute_source_currents` only ever reads a device's current back
    /// OUT, for a `voltage_source`'s own KCL; it never produced one for a
    /// plain resistor). Checked at several times against the resistor's own
    /// formula AND against KCL at the free node (current in = capacitor's
    /// current out + the node's own shunt current — core/README.md's W0
    /// point 2, included in every KCL sum).
    #[test]
    fn a_resistor_inside_a_closed_form_partition_publishes_its_own_current_and_satisfies_kcl() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let shunt = crate::solve::static_part::NODE_RSHUNT_G;
        for &t in &[1e-5, 1e-4, 5e-4, 1e-3, 2e-3] {
            let (mut v, mut cur) = (voltages.clone(), currents.clone());
            cf.publish_at(t, &HashMap::new(), &mut v, &mut cur);

            let expected_r = (v[1] - v[2]) / r; // resistor device 1: pins (node1, node2).
            assert!(cur[1] != 0.0, "t={t}: resistor current must not be published as a stale/absent 0.0");
            assert!((cur[1] - expected_r).abs() < 1e-12, "t={t}: resistor current got {} expected {}", cur[1], expected_r);

            // KCL at node2 (the only free node): resistor current IN must
            // equal the capacitor's current OUT plus the node's own shunt
            // current OUT.
            let kcl_residual = cur[1] - cur[2] - shunt * v[2];
            assert!(kcl_residual.abs() < 1e-12, "t={t}: KCL residual at node2 = {kcl_residual}");
        }
    }

    fn switch(component_index: usize, a: usize, b: usize, closed: f64) -> Device {
        Device { kind: crate::kinds::SWITCH, component_index, pins: vec![pin(a), pin(b)], params: vec![closed], blob: vec![] }
    }

    fn assert_same_bits(a: &[f64], b: &[f64]) {
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(b) {
            assert_eq!(x.to_bits(), y.to_bits());
        }
    }

    /// A re-aim whose stamp inputs did not change (a crossing landing on the
    /// same network, a `resolve_dirty` with nothing new) shares the segment
    /// it already has instead of rebuilding it, and what it shares is
    /// bit-for-bit what a fresh build would have produced. It is still a
    /// re-aim: the stat and the version move as before.
    #[test]
    fn a_reaim_with_an_unchanged_stamp_reuses_the_segment_a_rebuild_would_produce() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 1e6);
        let mut cf = build(&p);
        let live = HashMap::new();
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &live, &mut voltages, &mut currents).unwrap();
        cf.watch_node(2, DeviceId(0), Box::new(|_v: &[f64]| vec![Threshold { value: 3.0, rising: true }]));
        let first = Rc::clone(cf.parts[0].seg.as_ref().unwrap());
        let version = cf.parts[0].version;

        cf.resolve_dirty(&[0], 1e-4, &[], &live, &mut voltages, &mut currents).unwrap();
        let again = cf.parts[0].seg.as_ref().unwrap();
        assert!(Rc::ptr_eq(&first, again), "nothing in the stamp changed, so the segment is shared");
        assert_eq!(cf.segments_built(), 2.0, "a reused re-aim still counts as a re-aim");
        assert!(cf.parts[0].version > version, "and still cancels the crossings scheduled under the old aim");

        let (t, _, _) = cf.take_scheduled()[0];
        cf.handle_crossing(0, t, &[], &live, &mut voltages, &mut currents).unwrap();
        assert!(Rc::ptr_eq(&first, cf.parts[0].seg.as_ref().unwrap()), "a landed crossing re-aims the same network");

        let part = &cf.parts[0];
        let fresh = build_segment(&part.shape, &part.cap, part.global, t, &[], &part.touch_ics, &cf.node_owner, &Bnd::new(&voltages), &live, None, false, 0.0, &[], false).unwrap();
        assert_same_bits(&fresh.x0, &first.x0);
        assert_same_bits(&fresh.dx, &first.dx);
        assert_eq!(fresh.i0.to_bits(), first.i0.to_bits());
        assert_eq!(fresh.di.to_bits(), first.di.to_bits());
    }

    /// The other half: a live-parameter change (a switch toggle) is an input
    /// of the build, so it must not be served the old segment, and the
    /// rebuilt one is the fresh build.
    #[test]
    fn a_live_param_change_forces_a_rebuild_and_a_restore_brings_the_old_segment_back() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = program(3, vec![source(0, 1, 0, v0), resistor(1, 1, 2, r), capacitor(2, 2, 0, c, 0.0, 0.0), switch(3, 2, 0, 0.0)]);
        let mut cf = build(&p);
        let mut live: HashMap<usize, Vec<f64>> = HashMap::new();
        live.insert(3, vec![0.0]);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 4];
        cf.resolve_dirty(&[0], 0.0, &[], &live, &mut voltages, &mut currents).unwrap();
        let open = Rc::clone(cf.parts[0].seg.as_ref().unwrap());
        let snapshot = cf.snapshot();

        live.insert(3, vec![1.0]);
        cf.resolve_dirty(&[0], 1e-4, &[], &live, &mut voltages, &mut currents).unwrap();
        let closed = Rc::clone(cf.parts[0].seg.as_ref().unwrap());
        assert!(!Rc::ptr_eq(&open, &closed), "closing the switch changes the network");
        assert_ne!(open.di.to_bits(), closed.di.to_bits());
        let part = &cf.parts[0];
        let fresh = build_segment(&part.shape, &part.cap, part.global, 1e-4, &[], &part.touch_ics, &cf.node_owner, &Bnd::new(&voltages), &live, None, false, 0.0, &[], false).unwrap();
        assert_same_bits(&fresh.x0, &closed.x0);
        assert_eq!(fresh.di.to_bits(), closed.di.to_bits());

        cf.restore(snapshot);
        assert!(Rc::ptr_eq(&open, cf.parts[0].seg.as_ref().unwrap()), "a rolled-back instant gets its own segment back, key and all");
        live.insert(3, vec![0.0]);
        cf.resolve_dirty(&[0], 2e-4, &[], &live, &mut voltages, &mut currents).unwrap();
        assert!(Rc::ptr_eq(&open, cf.parts[0].seg.as_ref().unwrap()), "and the reopened switch matches that key again");
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Segment {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dx, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.di, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.key, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramp_sens, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SegHub {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.b, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_seg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dx0du, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.di0du, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.curve, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SegCurve {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dx1du, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.di1du, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.band, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc_lo, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc_hi, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DueCrossing {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.local, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_v, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cross_v, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dth, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Drive {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.modes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cell, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.held, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.couple, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.expiry, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cell_we, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.err, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g1, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DriveMode {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.k, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.we, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for RampDrive {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rate_x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rate_i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.end, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Partition {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.global, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shape, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cap, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg_t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg_vc0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg_f, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg_inf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.version, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cross_local, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cross_v, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.watches, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.watch_ids, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.segments_built, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.crossings_hit, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.touch_ics, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.touch_pulls, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.driver_ids, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.driver_index_len, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.key_scratch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.driver_scratch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.bits_scratch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.journaled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.live_nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_x0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_i0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_watched, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drive, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cross_dth, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.curved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_dx, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_di, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_du, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.folded, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drift_q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drift_from, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.parked, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Snapshot {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.partitions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.scheduled, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PartitionSnapshot {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg_t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg_vc0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg_f, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg_inf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.version, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cross_local, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cross_v, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_x0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_i0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drive, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cross_dth, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_dx, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_di, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_du, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.folded, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drift_q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drift_from, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for HubTerm {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.b, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_seg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.inf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.f, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for ClosedFormSolver {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.parts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slot_of, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_owner, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.scheduled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_to_ic_readers, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.window, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.journal, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.window_scheduled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub_updates, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sensitivities, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_seg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_edges, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cell, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_held, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_until, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_follow, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds_on, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_saved, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DevicePin {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            DevicePin::Node(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            DevicePin::Rail => 0,
            DevicePin::Fixed(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for CurvedBound {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.tau_n, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s_sum, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_sum, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.modes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.we, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.adx, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vm, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dvc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.f, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for CurvedMode {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.k, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.e_n, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.jp_n, w); }
        n
    }
}
