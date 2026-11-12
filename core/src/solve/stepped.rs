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
//! **A watched IC's supply moves the threshold, and is read at the instant.**
//! A watch's thresholds come from its IC's supply, so a crossing is the node
//! meeting a threshold that is itself moving. `find_crossing` and the
//! bisection read the thresholds from the state at each trial's own instant:
//! this partition's nodes from the trial, and a supply in ANOTHER Stepped
//! partition from that partition's pending trial ([`SupplyTrace`], a parabola
//! through its last two commits and the trial's end). So this partition's step
//! never runs past the nearest supply trial's end, and
//! [`SteppedSolver::sync_supplies`] brings each supply partition to the
//! planning instant first, so an IC evaluated at the crossing reads the same
//! supply the crossing was aimed at. Partitions otherwise advance
//! independently, each on its own `Step` events.
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
use std::rc::Rc;

use crate::lu::LuFactorization;
use crate::rails::{self, Contrib};
use crate::solve::live::RampEval;
use crate::solve::{
    build_ic_reader_index, collect_ics, compile_dyn_shape, pin_voltage, rail_sensitivity, stamp_ics_into_dyn, stamp_junctions, stamp_linear_base,
    stamp_residual, stamp_two_terminal, Bnd, DynCapacitor, DynPin, DynShape, ExactKind, ExactStamp, LiveTopology, SharedThresholdFn, ThresholdFn,
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
/// Ruling R60 (4): a `cc_switch` partition's step on a CC rail (`CcRail`)
/// holds its error this many times under `REL_TOL`/`ABS_VOLTAGE`. It stands
/// in for a closed form, exact to the rounding, whose capacitor carries the
/// rail, and the host cadence lets its steps grow: a creep at the
/// capacitor's pace accumulated 1.48 W0 of backward-Euler error over 5 ms
/// (G2bv's bc_ic 97 hub 2, 40 steps; exact mode at the same host stops
/// 1.50); at 8 times it is 0.70 W0 in 64 steps (4 times left 0.96; at 16,
/// 0.50, cold 482 tpl 0's aim landed its gate on its floor, where it slid
/// to the hold guard's `-4`, R51 (c)'s class). A
/// partition stepped from load keeps the controller's own tolerance (every
/// CC step tightened, two branches on one CC rail took 14,270 steps to 0.1
/// s against 3,682).
pub const CC_STEP_TOL_SCALE: f64 = 8.0;
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

/// Classic's `ABS_CURRENT` (`adaptive-step.ts:22`): the absolute part of a
/// current's tolerance in the step-doubling comparison (R77: the capacitor
/// series-current history in a trapezoidal run, classic's `capsI`).
pub const ABS_CURRENT: f64 = 10e-9;

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
    /// The factors of the step's last Newton iteration, kept for a hub
    /// partition's rail sensitivity (README, Rails point 3).
    lu: Option<LuFactorization>,
    /// On a CC rail ([`CcStep`]), the rail the step solved at its end.
    u: Option<f64>,
    /// On a CC rail with capacitors ([`CapStep`]), their C-weighted charge
    /// (as a rail voltage) and current at the step's end.
    rail_q: Option<(f64, f64)>,
    /// A TR-BDF2 step's first stage (R96), for the charge bill's quadrature.
    stage: Option<Box<TrBdf2Stage>>,
}

/// TR-BDF2's `gamma`: the trapezoidal stage ends at `gamma h`, `2 - sqrt(2)`,
/// the value that makes both stages' Jacobians the same multiple of `C / h`
/// and the method L-stable.
const TRBDF2_GAMMA: f64 = 2.0 - std::f64::consts::SQRT_2;

/// The trapezoidal stage of a TR-BDF2 step and the BDF2 stage's history.
#[derive(Debug, Clone)]
struct TrBdf2Stage {
    /// The stage's length, `gamma h`, and its end state.
    h_g: f64,
    x_g: Vec<f64>,
    i_g: Vec<f64>,
    /// The BDF2 stage as a backward Euler solve: its length and history.
    h_2: f64,
    vh: Vec<f64>,
    /// On a CC rail, the rail the stage solved at its end (W2b's charge).
    u_g: Option<f64>,
}

/// W2b: a point of a committed step: its time, nodes, capacitor voltages and
/// currents, and whether the interval ending here is interpolated by cubic
/// Hermite (a trapezoidal step or a TR-BDF2 stage) or linearly (backward
/// Euler).
#[derive(Debug, Clone)]
pub(crate) struct Knot {
    pub(crate) t: f64,
    pub(crate) x: Vec<f64>,
    vc: Vec<f64>,
    pub(crate) i: Vec<f64>,
    hermite: bool,
    /// The end of a TR-BDF2 step's trapezoidal stage.
    stage: bool,
    /// On a CC rail, the rail solved with this point (W2b's charge reads the
    /// step's devices against it).
    u: Option<f64>,
    /// On a CC rail with capacitors, their current at this point (W2bV: the
    /// charge weighs the rail capacitors at the step's points with it, so
    /// their charge is the companion's `C dq` of the step).
    pub(crate) rail_i: Option<f64>,
}

/// W2b piece 4: one committed step of a partition whose devices' charge is
/// armed ([`SteppedSolver::set_charge`]): its start and its knots, and the
/// nodes outside it its devices read, at each point as the step was solved
/// against them (`vals[k * reads.len() + r]`, point 0 the start).
#[derive(Debug, Clone)]
pub(crate) struct ChargeStep {
    pub(crate) p: usize,
    pub(crate) t0: f64,
    pub(crate) x0: Vec<f64>,
    pub(crate) i0: Vec<f64>,
    pub(crate) knots: Vec<Knot>,
    pub(crate) vals: Vec<f64>,
    /// Planned before the charge was armed: no knots were kept, so `knots`
    /// holds its end alone and the step is weighed by its end.
    pub(crate) ends_only: bool,
    /// On a CC rail with capacitors, their current at the step's start (the
    /// trapezoid's `i`, `CapStep`).
    pub(crate) rail_i0: Option<f64>,
}

impl ChargeStep {
    /// Each point's weight in the step's own quadrature, so that `sum w_k
    /// i_k` over a capacitor's currents is exactly how far the integrator
    /// moved its charge (`C dvc`): backward Euler the end current times the
    /// interval, the trapezoid the mean of its ends, and a TR-BDF2 sub-step
    /// `k (gamma h / 2) (i_0 + i_g) + h_2 i_1` (`plan_step`'s bill).
    pub(crate) fn weights(&self) -> Vec<f64> {
        let n = self.knots.len();
        let mut w = vec![0.0; n + 1];
        let t_of = |k: usize| if k == 0 { self.t0 } else { self.knots[k - 1].t };
        let mut j = 1;
        while j <= n {
            let k = &self.knots[j - 1];
            let (ts, te) = (t_of(j - 1), k.t);
            if k.stage && j < n {
                let g = TRBDF2_GAMMA;
                let kk = 1.0 / (g * (2.0 - g));
                let end = self.knots[j].t;
                let half = kk * 0.5 * (te - ts);
                w[j - 1] += half;
                w[j] += half;
                w[j + 1] += (end - ts) * (1.0 - g) / (2.0 - g);
                j += 2;
            } else if k.hermite {
                w[j - 1] += 0.5 * (te - ts);
                w[j] += 0.5 * (te - ts);
                j += 1;
            } else {
                w[j] += te - ts;
                j += 1;
            }
        }
        w
    }

    /// The end of the step.
    pub(crate) fn end(&self) -> f64 {
        self.knots.last().map_or(self.t0, |k| k.t)
    }
}

/// W2b: a stepped node as the observer found it between two instants: its
/// partition, its local index, and whether it reads the rail.
pub(crate) struct Located<'a> {
    p: usize,
    l: usize,
    part: &'a Partition,
    pub(crate) on_rail: bool,
}

/// W2b: a pending trial's knots after its start, and an iterated
/// partition's nodes solved at the grid times within it ([`DenseSeg`]).
#[derive(Debug, Clone, Default)]
pub(crate) struct PendDense {
    knots: Vec<Knot>,
    solved: Vec<(f64, Option<Vec<f64>>)>,
    /// On a CC rail, the rail the trial starts from: its capacitors' charge,
    /// or with none the rail as last solved.
    u0: Option<f64>,
    /// On a CC rail with capacitors, their current at the trial's start.
    rail_i0: Option<f64>,
}

impl PendDense {
    fn clear(&mut self) {
        self.knots.clear();
        self.solved.clear();
    }
}

/// W2b: one committed step of a partition the observer reads, from its
/// start state through its knots, or a seed (no knots: a point).
#[derive(Debug, Clone)]
pub(crate) struct DenseSeg {
    p: usize,
    t0: f64,
    x0: Vec<f64>,
    vc0: Vec<f64>,
    i0: Vec<f64>,
    knots: Vec<Knot>,
    /// Nothing it reads moves within the step (no running ramp, no rail).
    steady: bool,
    /// Steady, linear, with one capacitor: every node an affine map of it.
    affine: bool,
    /// A hub partition's sensitivity to the rail at the step, `(s, u_p)`.
    sens: Option<(Vec<f64>, f64)>,
    /// An iterated partition's nodes solved at the observer's grid times in
    /// the step (`None`: the solve failed, the sample is linear), R104 as
    /// ruled for a node no capacitor's voltage gives.
    solved: Vec<(f64, Option<Vec<f64>>)>,
}

impl DenseSeg {
    fn end(&self) -> f64 {
        self.knots.last().map_or(self.t0, |k| k.t)
    }

    /// Every capacitor's voltage at `t`: Hermite over a Hermite interval,
    /// else linear.
    fn vc_at(&self, shape: &DynShape, t: f64) -> Vec<f64> {
        let (mut ta, mut vca, mut ia) = (self.t0, &self.vc0, &self.i0);
        for k in &self.knots {
            if t <= k.t {
                let tau = k.t - ta;
                if !(tau > 0.0) {
                    return k.vc.clone();
                }
                let s = (t - ta) / tau;
                let (s2, s3) = (s * s, s * s * s);
                return (0..vca.len())
                    .map(|j| {
                        if !k.hermite {
                            return vca[j] + (k.vc[j] - vca[j]) * s;
                        }
                        let c = shape.capacitors[j].c;
                        (2.0 * s3 - 3.0 * s2 + 1.0) * vca[j] + (s3 - 2.0 * s2 + s) * tau * ia[j] / c + (-2.0 * s3 + 3.0 * s2) * k.vc[j] + (s3 - s2) * tau * k.i[j] / c
                    })
                    .collect();
            }
            (ta, vca, ia) = (k.t, &k.vc, &k.i);
        }
        vca.clone()
    }

    fn span(&self) -> f64 {
        self.end() - self.t0
    }

    /// Local node `l` at `t` within the segment, and whether it fell back to
    /// linear where a solve was due.
    fn value(&self, shape: &DynShape, l: usize, t: f64) -> (f64, bool) {
        let plate = plate_of(shape, l);
        if plate.is_none() {
            if let Some((_, solved)) = self.solved.iter().find(|s| s.0 == t) {
                return match solved {
                    Some(x) => (x[l], false),
                    None => (self.interpolate(shape, l, t, None), true),
                };
            }
        }
        (self.interpolate(shape, l, t, plate), false)
    }

    fn interpolate(&self, shape: &DynShape, l: usize, t: f64, plate: Option<(usize, f64)>) -> f64 {
        let (mut ta, mut xa, mut vca, mut ia) = (self.t0, &self.x0, &self.vc0, &self.i0);
        for k in &self.knots {
            if t <= k.t {
                let tau = k.t - ta;
                if !(tau > 0.0) {
                    return k.x[l];
                }
                let s = (t - ta) / tau;
                let linear = xa[l] + (k.x[l] - xa[l]) * s;
                if !k.hermite || !self.steady {
                    return linear;
                }
                // Cubic Hermite on a capacitor's voltage, its slope i / C at
                // each end.
                let (s2, s3) = (s * s, s * s * s);
                let hermite = |j: usize| {
                    let c = shape.capacitors[j].c;
                    (2.0 * s3 - 3.0 * s2 + 1.0) * vca[j] + (s3 - 2.0 * s2 + s) * tau * ia[j] / c + (-2.0 * s3 + 3.0 * s2) * k.vc[j] + (s3 - s2) * tau * k.i[j] / c
                };
                // The node from the charges where that is cheap: a plate of
                // a capacitor with no ESR whose other plate is a fixed node
                // is that capacitor's voltage plus a constant; every node of
                // a linear partition with one capacitor is an affine map of
                // its voltage, which the two ends fix. Otherwise linear in
                // time (an iterated partition's other nodes are solved).
                if let Some((j, sign)) = plate {
                    return xa[l] + sign * (hermite(j) - vca[j]);
                }
                if !self.affine {
                    return linear;
                }
                let dv = k.vc[0] - vca[0];
                if !(dv.abs() > 1e-12 * (1.0 + vca[0].abs())) {
                    return linear;
                }
                return xa[l] + (k.x[l] - xa[l]) * (hermite(0) - vca[0]) / dv;
            }
            (ta, xa, vca, ia) = (k.t, &k.x, &k.vc, &k.i);
        }
        xa[l]
    }
}

// Test builds can make every dense-output solve fail, to check the
// fallback to linear and its count.
#[cfg(test)]
thread_local! { pub(crate) static FAIL_DENSE_SOLVE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) }; }

/// Local node `l` as a plate of a capacitor with no ESR whose other plate is
/// a fixed node: that capacitor, and `1` or `-1` as the node is its first
/// or second plate.
fn plate_of(shape: &DynShape, l: usize) -> Option<(usize, f64)> {
    shape.capacitors.iter().enumerate().find_map(|(j, cap)| match (cap.esr == 0.0, cap.pins[0].local, cap.pins[1].local) {
        (true, Some(a), None) if a == l => Some((j, 1.0)),
        (true, None, Some(b)) if b == l => Some((j, -1.0)),
        _ => None,
    })
}

/// A bench PSU in CC with no rail capacitor, as the stepped hub partitions
/// see it (README, Rails point 3, FXF): the rail is set by the draws alone,
/// `Σ draws(u) = dir·iLimit`, so it moves with this partition's own state
/// (an LED charging a capacitor at the limit drags the rail up with the
/// capacitor). Held as a boundary over a step, it went stale within the step
/// and the 1 µV CC re-solve band re-solved the partition every 1e-7 s, each
/// time a discontinuity restarting the step at `POST_EVENT_H` (0.1 s of such
/// a board took 27 s). So each such partition steps with the rail as an
/// unknown: its own KCL plus the rest of the rail's draw, `a_o + g_o u`,
/// linearised at the last rail solve, equals the limit. The rest is the
/// running sums of every other contribution and the direct devices at the
/// present rail; the rail loop still solves the rail exactly after each
/// commit and re-solves past the band, so this only makes the step's own
/// rail consistent, not the rail's answer.
#[derive(Debug, Clone)]
pub(crate) struct CcRail {
    pub rail: usize,
    /// `dir·iLimit` and the setpoint `V`, where CC ends.
    pub dl: f64,
    pub v: f64,
    /// Every partition's and direct device's draw as `a + g u` at the last
    /// rail solve, and each stepped hub partition's own share of it.
    pub a_all: f64,
    pub g_all: f64,
    pub own: Vec<(usize, f64, f64)>,
    /// The rail capacitors, when there are any (ruling R68): stepped with
    /// the partition as a companion on its rail row ([`CapStep`]).
    pub cap: Option<RailCap>,
    /// With them, the rails at which a rail IC's power flips (its window's
    /// edges, a held one's release level), ascending: a step is cut where
    /// its rail reaches one ([`CcStep::edge_past`]).
    pub edges: Vec<f64>,
}

/// A CC rail's capacitors as a stepped hub partition steps against them
/// (README, Rails point 4, ruling R68): their summed capacitance and
/// `esr_eff`, and the segment their C-weighted charge `q` follows between
/// rail solves, from which a step that does not start at a commit of its
/// own reads it. Held as the segment's frozen form instead, a partition
/// coupled to the rail through microfarads of small esr took the rail's
/// microsecond motion as a linearised draw: the cold fuzz's 347 tpl 5 ran
/// 2824 W0 off a fine reference in its start-up inrush and 239 W0 after a
/// CC entry, where exact mode is within 3.6.
#[derive(Debug, Clone)]
pub(crate) struct RailCap {
    pub c: f64,
    pub esr: f64,
    pub seg: crate::rails::CapSeg,
}

/// [`RailCap`] at a step's start: the charge `q` (as a rail voltage) and
/// the capacitors' current `i` there (the trapezoid's), with `C` and
/// `esr_eff`. Over the step `u = q' + esr i'` and `C q' = C q + h i'`
/// (backward Euler) or `+ h (i + i') / 2` (trapezoid): a companion of
/// conductance `1 / (esr + h / C)` (or `h / 2C`) on the rail row. A seed
/// holds the charge, `u = q + esr i'`, the rail `q` itself at `esr = 0`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct CapStep {
    c: f64,
    esr: f64,
    q: f64,
    i: f64,
}

impl CapStep {
    /// The companion `(g, q_eff)` of the capacitors over `h` (a seed's at
    /// `None`): their current is `g (u - q_eff)`. `None` at a seed with
    /// `esr = 0`, where `u = q`.
    fn companion(&self, h: Option<f64>, method: Method) -> Option<(f64, f64)> {
        match h {
            None => (self.esr > 0.0).then(|| (1.0 / self.esr, self.q)),
            Some(h) => Some(match method {
                Method::Be => (1.0 / (self.esr + h / self.c), self.q),
                Method::Trap => (1.0 / (self.esr + h / (2.0 * self.c)), self.q + h * self.i / (2.0 * self.c)),
            }),
        }
    }

    /// The charge and current at the end of a step over `h` to rail `u`.
    fn end(&self, h: f64, method: Method, u: f64) -> (f64, f64) {
        let (g, q_eff) = self.companion(Some(h), method).expect("a step has a companion");
        let i = g * (u - q_eff);
        let q = match method {
            Method::Be => self.q + h * i / self.c,
            Method::Trap => self.q + h * (i + self.i) / (2.0 * self.c),
        };
        (q, i)
    }
}

impl CcRail {
    /// Partition `p`'s view at its step's start `t`, the rail at `u0`;
    /// `committed` the rail charge `(t, q, i)` where it last stood (its last
    /// commit or seed on such a rail), and `pending` the charge the latest
    /// commit handed the rail solve that has not yet taken it
    /// ([`SteppedSolver::take_rail_charge`]). The charge at `t` is the latest
    /// one there: a pending hand-off at `t`, else the segment's when it was
    /// aimed at `t` (the rail the loop solved after that commit; a
    /// partition's own commit there may have been superseded by another's:
    /// cold 517 tpl 5, two seeded at a rail each solved from its own charge,
    /// 0.02 V and 0.64 V, and one re-solved past its band for the loop's 50
    /// passes), else the partition's own (a segment aimed before `t` is the
    /// frozen form's estimate since, which a commit replaces, and one aimed
    /// after `t` no longer knows `t`). A segment aimed at `t` before a commit
    /// there is that estimate too: cold 581 tpl 5's seed after a 555 event,
    /// whose rail solve came before the catch-up's commit, held the rail
    /// 6 uV off the partition's capacitors, which a capacitor loop through
    /// the rail (7.6 mF to a node 1.8 uF holds to ground) turned into 4.5 A
    /// that the trapezoid then reflected undamped at every step. The current
    /// a trapezoid starts from is the partition's own last step's whenever it
    /// stands there: the segment's slope at its aim is the frozen form's, the
    /// draws' imbalance at the instant.
    fn step(&self, p: usize, u0: f64, t: f64, committed: Option<(f64, f64, f64)>, pending: Option<(f64, f64)>) -> CcStep {
        let (a, g) = self.own.iter().find(|o| o.0 == p).map_or((0.0, 0.0), |o| (o.1, o.2));
        let coupled = self.own.iter().any(|o| o.0 != p);
        let cap = self.cap.as_ref().map(|rc| {
            let at = t.max(rc.seg.t0);
            let own_i = committed.filter(|c| c.0 == t).map(|c| c.2);
            let i_seg = || rc.c * rc.seg.charge_slope_at(at);
            let (q, i) = match (pending.filter(|p| p.0 == t), committed) {
                (Some((_, q)), _) => (q, own_i.unwrap_or_else(i_seg)),
                (None, Some((tc, q, i))) if tc == t && t != rc.seg.t0 => (q, i),
                _ => (rc.seg.charge_at(at), own_i.unwrap_or_else(i_seg)),
            };
            CapStep { c: rc.c, esr: rc.esr, q, i }
        });
        let edge_up = self.edges.iter().copied().find(|&e| e > u0).unwrap_or(f64::INFINITY);
        let edge_dn = self.edges.iter().rev().copied().find(|&e| e < u0).unwrap_or(f64::NEG_INFINITY);
        CcStep { rail: self.rail, dl: self.dl, v: self.v, a_o: self.a_all - a, g_o: self.g_all - g, u0, coupled, seed: false, cv: false, cap, edge_up, edge_dn }
    }
}

/// Ruling R90 (a): one stepped device with a rating, as the planner reads
/// its stress ratio `q` from a trial's own nodes ([`StressProbe::ratio`],
/// the formulas the slow pass's drives use: `failures::resistor_q`,
/// `fuse_q`, `led_q`, `PtcParams::untripped_constant`). The slow pass hands
/// each partition the ones not failed or tripped (`set_stress_probes`).
#[derive(Debug, Clone)]
pub(crate) enum StressProbe {
    Resistor { a: usize, b: usize, r: f64, r_power: f64, p_max: f64 },
    Fuse { a: usize, b: usize, r_normal: f64, i_rating: f64 },
    Led { device: Rc<crate::program::Device>, junctions: Rc<Vec<devices::led::Junction>>, if_max: f64 },
    Ptc { a: usize, b: usize, params: crate::slow::stress::PtcParams },
}

impl StressProbe {
    /// `(q, zero)` with the node voltages read through `v`.
    pub(crate) fn ratio(&self, v: &dyn Fn(usize) -> f64) -> (f64, f64) {
        use crate::slow::failures;
        match self {
            StressProbe::Resistor { a, b, r, r_power, p_max } => (failures::resistor_q(v(*a) - v(*b), *r, *r_power, *p_max), 1.0),
            StressProbe::Fuse { a, b, r_normal, i_rating } => (failures::fuse_q(v(*a) - v(*b), *r_normal, *i_rating), 1.0),
            StressProbe::Led { device, junctions, if_max } => (failures::led_q(devices::led::peak_current_with(device, junctions, v), *if_max), 1.0),
            StressProbe::Ptc { a, b, params } => match params.untripped_constant((v(*a) - v(*b)).abs() / params.resistance(false)) {
                crate::slow::stress::StressDrive::Constant { q, zero, .. } => (q, zero),
                crate::slow::stress::StressDrive::Segment(_) => (-1.0, 1.0),
            },
        }
    }
}

/// Ruling R90 (a): the step a trial from `q0` to `q1` (zero level `zero`)
/// should have been, `h` cut in proportion to its ratio's log change with a
/// margin, when the change passes `STRESS_STEP_RATIO`; `None` within it, or
/// when neither end is at least half the level.
fn stress_cut(changes: impl Iterator<Item = ((f64, f64), (f64, f64))>, h: f64) -> Option<f64> {
    let mut worst: f64 = 0.0;
    for ((q0, zero), (q1, _)) in changes {
        if q0 > 0.0 && q1 > 0.0 && q0.max(q1) >= 0.5 * zero {
            worst = worst.max((q1 / q0).ln().abs());
        }
    }
    (worst > crate::slow::stress::STRESS_STEP_RATIO && h > 2.0 * H_MIN).then(|| (0.8 * h * crate::slow::stress::STRESS_STEP_RATIO / worst).max(H_MIN))
}

/// One committed step of a partition, as [`SteppedSolver::trail`] holds it.
#[derive(Debug, Clone)]
pub(crate) struct Committed {
    pub p: usize,
    pub t0: f64,
    pub t1: f64,
    pub nodes: Vec<(usize, f64)>,
    pub rail: Option<(usize, f64)>,
    /// The step's own start state, as `nodes` and `rail` hold its end
    /// (rulings R80 (a), R84 (B): its stress is the mean of the drives at
    /// the two); `None` for the first step after a re-seed, which keeps its
    /// end rate.
    pub start: Option<Vec<(usize, f64)>>,
}

/// One partition's view of a [`CcRail`]: the rest of the draw without its
/// own share, and the rail `u0` it starts from, at which a rail IC's power
/// gate is judged for the whole step (power flips only at rail solves).
#[derive(Debug, Clone, Copy)]
struct CcStep {
    rail: usize,
    dl: f64,
    v: f64,
    a_o: f64,
    g_o: f64,
    u0: f64,
    /// Another stepped hub partition shares the rail: it holds this one's
    /// draw where this one last committed, so the rail's move across this
    /// step is an error it does not see ([`coupling_ratio`]).
    coupled: bool,
    /// A seed (`SEED_H`): its rail row is the charge balance
    /// ([`ChargeBalance`]) when a capacitor has a pin on the rail.
    seed: bool,
    /// Solved in CV, the rail at `v` (`take_cc_step`'s `cv`): a horizon
    /// clamp's steps after one that left CC (`Partition::cc_exited`).
    cv: bool,
    /// The rail capacitors at the step's start (ruling R68).
    cap: Option<CapStep>,
    /// The nearest power edges above and below `u0` (`CcRail::edges`).
    edge_up: f64,
    edge_dn: f64,
}

impl CcStep {
    /// How far `u` is past the setpoint in the supply's direction (the CC
    /// exit, Rails point 5, is at 0).
    fn beyond(&self, u: f64) -> f64 {
        let dir = if self.v < 0.0 { -1.0 } else { 1.0 };
        dir * u - self.v.abs()
    }

    fn past_setpoint(&self, u: f64) -> bool {
        self.beyond(u) > 0.0
    }

    /// How far `u` is past the nearest rail IC power edge either side of the
    /// step's start (ruling R68); negative while between them.
    fn edge_past(&self, u: f64) -> f64 {
        (u - self.edge_up).max(self.edge_dn - u)
    }
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
#[derive(Clone, Copy)]
struct IcStampCtx<'a> {
    now: f64,
    drivers: &'a [Driver],
    ics: &'a [(usize, digital::IcRecord)],
    node_owner: &'a HashMap<usize, (usize, usize)>,
    /// The live boundary every non-free pin is read through, and the running
    /// ramps a step reads at its own instant (Sources point 3).
    bnd: Bnd<'a>,
    ramps: Option<&'a RampEval>,
    /// Whether the step keeps its factors (a hub partition).
    keep_lu: bool,
    /// Exact mode's hub source, when this is the rail's partition.
    exact: Option<ExactStamp>,
    /// The CC rail this partition steps against ([`CcRail`]).
    cc: Option<CcStep>,
    p_idx: usize,
    /// Whether THIS partition's stamp moves with the very voltages the step
    /// is solving for: it holds a free IC supply node (contract point 3,
    /// R_out follows the iterate) or a stamped LED junction (W1e, whose
    /// companion is re-linearised about it), or, on a CC rail, a pin of an
    /// IC the rail supplies (`Partition::iterated_on`). [`take_step`] then runs a real
    /// Newton iteration (`linear = false`) instead of the one-shot linear
    /// solve every other partition needs.
    iterated: bool,
    /// Whether it holds a free IC supply node, whose stamps' slopes in that
    /// supply [`solve_step`]'s Newton takes into its Jacobian.
    free_supply: bool,
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
    at: f64,
) -> Result<StepResult, StepError> {
    // R96: a backward Euler step of a partition `plan_step` marks (`TRB`)
    // is TR-BDF2: a trapezoidal stage to gamma h, then a BDF2 stage to h.
    // The BDF2 stage is a backward Euler solve over h (1 - g) / (2 - g) from
    // the history vc = k vc_g - m vc_0, k = 1 / (g (2 - g)), m = (1 - g)^2 k,
    // which is BDF2's companion exactly. Second order and L-stable, where
    // backward Euler is first order: its voltages kept to W0 per step left
    // a capacitor's decaying current hundreds of W0 off (fixture 38's
    // supply, 326 W0; cold 360/3's gate ground pin 9.6 W0 after each clock
    // edge). A floor step stays the one plain step (R78 (a)).
    if method == Method::Be && h > H_MIN * 2.01 && TRB.with(|f| f.get()) {
        let g = TRBDF2_GAMMA;
        let a = solve_step(shape, live_params, g * h, vc_from, x_from, i_prev, Method::Trap, newton_iters, ic_ctx, at - h + g * h, false)?;
        let k = 1.0 / (g * (2.0 - g));
        let m = (1.0 - g) * (1.0 - g) * k;
        let vh: Vec<f64> = vc_from.iter().zip(&a.vc).map(|(v0, vg)| k * vg - m * v0).collect();
        let h_2 = h * (1.0 - g) / (2.0 - g);
        let mut b = solve_step(shape, live_params, h_2, &vh, &a.x, &a.i, Method::Be, newton_iters, ic_ctx, at, false)?;
        b.stage = Some(Box::new(TrBdf2Stage { h_g: g * h, x_g: a.x, i_g: a.i, h_2, vh, u_g: a.u }));
        return Ok(b);
    }
    solve_step(shape, live_params, h, vc_from, x_from, i_prev, method, newton_iters, ic_ctx, at, false)
}

// The partition `plan_step` is planning steps by TR-BDF2 (R96): set there,
// read by `take_step`, so its trials, crossing searches and hold retries all
// take the same steps. Cleared by `TrbGuard` however the plan returns.
thread_local! { static TRB: std::cell::Cell<bool> = const { std::cell::Cell::new(false) }; }
/// Clears `TRB` however `plan_step` returns.
struct TrbGuard;
impl Drop for TrbGuard {
    fn drop(&mut self) {
        TRB.with(|f| f.set(false));
    }
}

/// [`take_step`], with `polish` handed to the iterated Newton solve (one more
/// iteration from the passing iterate, `newton::solve`).
#[allow(clippy::too_many_arguments)]
fn solve_step(
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc_from: &[f64],
    x_from: &[f64],
    i_prev: &[f64],
    method: Method,
    newton_iters: &mut u64,
    ic_ctx: &IcStampCtx,
    at: f64,
    polish: bool,
) -> Result<StepResult, StepError> {
    solve_step_noting(shape, live_params, h, vc_from, x_from, i_prev, method, newton_iters, ic_ctx, at, polish, None)
}

/// The ICs with a supply pin free in a partition (device index, record),
/// whose power verdicts a seed's Newton notes, and their flips.
type Noting<'n, 'a> = (&'n [(usize, &'a digital::IcRecord)], &'n mut crate::solve::hold::Flips);

/// [`solve_step`], noting each gated IC's power verdict at every iterate of
/// the plain Newton in `noting` (W1c point 1a's trigger, as the static
/// solver's `newton_in` notes them).
#[allow(clippy::too_many_arguments)]
fn solve_step_noting(
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc_from: &[f64],
    x_from: &[f64],
    i_prev: &[f64],
    method: Method,
    newton_iters: &mut u64,
    ic_ctx: &IcStampCtx,
    at: f64,
    polish: bool,
    mut noting: Option<Noting>,
) -> Result<StepResult, StepError> {
    if let Some(cc) = ic_ctx.cc {
        return take_cc_step(shape, live_params, h, vc_from, x_from, i_prev, method, newton_iters, ic_ctx, at, cc, cc.cv);
    }
    let n = shape.n;
    // The boundary at the step's own end, where backward Euler and the
    // trapezoid both solve the network (a running ramp moves it).
    let bnd = ic_ctx.bnd.at_time(ic_ctx.ramps, at);
    let exact = ic_ctx.exact.filter(|e| e.partition == ic_ctx.p_idx);
    // `linear = true` short-circuits `newton::solve` to exactly one
    // iteration (a linear stamp could never move on a second pass) — but an
    // IC's driver/pull conductance on a FREE supply node genuinely depends on
    // the iterate itself (R_out is a function of the very voltage being
    // solved for), and so does an LED junction's companion, so those
    // partitions need the real convergence loop instead. No polish: classic
    // solves each step exactly like this (LEDs point 8).
    let linear = !ic_ctx.iterated;
    // Classic's loop restamps each IC stamp at the iterate's supply, its
    // slope in that supply left out. Where a partition holding a free IC
    // supply does not converge so at a step with no smaller one to retry
    // (the seed's, or `H_MIN`), the step is solved once more with the
    // slopes in the Jacobian, as the hub polish takes them (W1c point 3;
    // `StaticPartitionSolver::newton_in`'s second try, for the same reasons:
    // cold fuzz 1325 tpl 0's seed declined in exact mode at its switch
    // toggle). Only there: a longer step retries smaller anyway, and a
    // second try at each doubled cold fuzz 238 tpl 3's run.
    let mut result = None;
    for slopes in [false, true] {
        if slopes && (linear || !ic_ctx.free_supply || h > H_MIN * 1.01) {
            break;
        }
        let (r, factors) = newton::solve_keeping_factors(n, n, x_from, linear, polish, |x, a, b| {
            if let (false, Some((gated, flips))) = (slopes, noting.as_mut()) {
                for (k, &(d, ic)) in gated.iter().enumerate() {
                    let v = |node: usize| match ic_ctx.node_owner.get(&node) {
                        Some(&(p, l)) if p == ic_ctx.p_idx => x[l],
                        _ => bnd.get(node),
                    };
                    flips.note(k, digital::power::verdict(ic, v(ic.vcc.node), v(ic.gnd.node), bnd.hold(d)));
                }
            }
            stamp_step(shape, live_params, h, vc_from, i_prev, method, ic_ctx, &bnd, n, x, a, b);
            if slopes {
                let extra = crate::solve::ic_supply_terms(ic_ctx.now, ic_ctx.drivers, ic_ctx.ics, ic_ctx.node_owner, &bnd, ic_ctx.p_idx, x).extra_jacobian(x, n);
                for row in 0..n {
                    for col in 0..n {
                        let j = extra[row * (n + 1) + col];
                        if j != 0.0 && j.is_finite() {
                            a[row * n + col] += j;
                            b[row] += j * x[col];
                        }
                    }
                }
            }
            if let Some(e) = exact {
                e.apply(n, a, b);
            }
        })
        .map_err(|e| StepError::Failed(format!("stepped: {e}")))?;
        // Counted before the verdict: a rejected step's iterations were run.
        *newton_iters += r.iterations as u64;
        if linear || r.converged {
            result = Some((r, factors));
            break;
        }
    }
    // `newton::solve` never turns "hit the iteration cap unconverged" into
    // an `Err`; what that means is the caller's call (retry smaller, end a
    // refinement, or fail).
    let Some((result, factors)) = result else { return Err(StepError::NoConverge) };

    let x = result.x;
    let mut vc = vec![0.0; shape.capacitors.len()];
    let mut i_new = vec![0.0; shape.capacitors.len()];
    for (idx, cap) in shape.capacitors.iter().enumerate() {
        let contrib = companion(cap, h, vc_from[idx], i_prev[idx], method);
        let (va, vb) = (pin_voltage(&cap.pins[0], &x, &bnd), pin_voltage(&cap.pins[1], &x, &bnd));
        i_new[idx] = crate::solve::branch_current(contrib.conductance, contrib.injection, va, vb);
    }
    if h == SEED_H {
        seed_branch_currents(shape, live_params, ic_ctx, &bnd, exact.as_ref(), n, &x, vc_from, &mut i_new);
    }
    for (idx, cap) in shape.capacitors.iter().enumerate() {
        let i = i_new[idx];
        vc[idx] = match method {
            Method::Be => vc_from[idx] + (h / cap.c) * i,
            Method::Trap => vc_from[idx] + (h / (2.0 * cap.c)) * (i + i_prev[idx]),
        };
    }
    Ok(StepResult { x, vc, i: i_new, lu: ic_ctx.keep_lu.then_some(factors), u: None, rail_q: None, stage: None })
}

/// [`take_step`] on a CC rail ([`CcRail`]): the rail is unknown `n`, its row
/// this partition's draw (the residual stamp's rail row, as the sensitivity
/// reads it) plus the rest's `a_o + g_o u`, equal to `dir·iLimit`. A rail
/// that ends past the setpoint is reported as it is: the plan cuts the step
/// at the exit (`plan_step`), and the regime rules leave CC at its commit
/// (Rails point 5). Re-solving the whole step in CV instead put the step's
/// end on a trajectory CV never took (the rail held at `V` from the step's
/// start), whose draw at `V` was over the limit again: the rail loop stayed
/// in CC 1.3e-8 V short of `V`, and the exit landed a whole step (2 ms, 16
/// mV of capacitor) late.
#[allow(clippy::too_many_arguments)]
fn take_cc_step(
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc_from: &[f64],
    x_from: &[f64],
    i_prev: &[f64],
    method: Method,
    newton_iters: &mut u64,
    ic_ctx: &IcStampCtx,
    at: f64,
    cc: CcStep,
    cv: bool,
) -> Result<StepResult, StepError> {
    let n = shape.n;
    let size = n + 1;
    let bnd = ic_ctx.bnd.at_time(ic_ctx.ramps, at).with_ext(cc.rail, n).hold_power_at(cc.u0);
    let linear = !ic_ctx.iterated;
    let balance = (cc.seed && !cv).then(|| rail_capacitors(shape, &bnd)).flatten().map(|(caps, nodes)| ChargeBalance { caps, nodes });
    let cap = cc.cap.filter(|_| !cv);
    let solve = |newton_iters: &mut u64| -> Result<(Vec<f64>, LuFactorization), StepError> {
        let mut z0: Vec<f64> = x_from[..n].to_vec();
        z0.push(if cv { cc.v } else { cc.u0 });
        let stamp = |z: &[f64], a: &mut [f64], b: &mut [f64]| {
            stamp_step(shape, live_params, h, vc_from, i_prev, method, ic_ctx, &bnd, size, z, a, b);
            let row = n * size;
            if cv {
                a[row..row + size].fill(0.0);
                a[row + n] = 1.0;
                b[n] = cc.v;
            } else {
                if let Some(balance) = &balance {
                    b[n] = balance.row(shape, live_params, h, vc_from, ic_ctx, &bnd, z, &mut a[row..row + size]);
                }
                match cap {
                    // A seed on an esr-0 rail capacitor: the rail is its
                    // charge, which an instant does not move.
                    Some(c) if cc.seed && c.companion(None, method).is_none() => {
                        a[row..row + size].fill(0.0);
                        a[row + n] = 1.0;
                        b[n] = c.q;
                    }
                    _ => {
                        a[row + n] += cc.g_o;
                        b[n] += cc.dl - cc.a_o;
                        if let Some((g, q)) = cap.and_then(|c| c.companion((!cc.seed).then_some(h), method)) {
                            a[row + n] += g;
                            b[n] += g * q;
                        }
                    }
                }
            }
        };
        let (result, mut factors) = newton::solve_keeping_factors(size, size, &z0, linear, true, stamp).map_err(|e| StepError::Failed(format!("stepped: {e}")))?;
        *newton_iters += result.iterations as u64;
        if !linear && !result.converged {
            return Err(StepError::NoConverge);
        }
        // Then on to the solution, to rounding: the rail loop solves the rail
        // from this step's draw and compares it with the rail the step ended
        // on, against the 1 µV CC band and against the setpoint. An iterate a
        // Newton tolerance short re-solved the partition every few hundred
        // µs, and one polish iteration still left the rail 1.3e-8 V off the
        // loop's, enough to leave a cut that landed past the setpoint in CC
        // for another step (a 5 ms late exit). Each further step is one
        // Newton iteration about the last (a linear solve of the stamps
        // linearised there); it stops once a step moves nothing beyond a few
        // ulps, at most `POLISH_MORE` of them.
        const POLISH_MORE: u32 = 4;
        let mut z = result.x;
        if !linear {
            for _ in 0..POLISH_MORE {
                let (next, f) = newton::solve_keeping_factors(size, size, &z, true, false, stamp).map_err(|e| StepError::Failed(format!("stepped: {e}")))?;
                *newton_iters += next.iterations as u64;
                let moved = next.x.iter().zip(&z).any(|(a, b)| (a - b).abs() > 4.0 * f64::EPSILON * a.abs().max(b.abs()).max(1.0));
                z = next.x;
                factors = f;
                if !moved {
                    break;
                }
            }
        }
        Ok((z, factors))
    };
    let (mut z, factors) = solve(newton_iters)?;
    let mut vc = vec![0.0; shape.capacitors.len()];
    let mut i_new = vec![0.0; shape.capacitors.len()];
    for (idx, cap) in shape.capacitors.iter().enumerate() {
        let contrib = companion(cap, h, vc_from[idx], i_prev[idx], method);
        let (va, vb) = (pin_voltage(&cap.pins[0], &z, &bnd), pin_voltage(&cap.pins[1], &z, &bnd));
        i_new[idx] = crate::solve::branch_current(contrib.conductance, contrib.injection, va, vb);
    }
    if cc.seed {
        seed_branch_currents(shape, live_params, ic_ctx, &bnd, None, size, &z, vc_from, &mut i_new);
    }
    for (idx, cap) in shape.capacitors.iter().enumerate() {
        let i = i_new[idx];
        vc[idx] = match method {
            Method::Be => vc_from[idx] + (h / cap.c) * i,
            Method::Trap => vc_from[idx] + (h / (2.0 * cap.c)) * (i + i_prev[idx]),
        };
    }
    // The sensitivity solves through the partition's own matrix at the rail
    // it ended on: the node block of the last iteration's, restamped at the
    // solution (the rail's row and column are the step's, not the
    // partition's).
    let lu = if ic_ctx.keep_lu {
        let (mut a, mut b) = (vec![0.0; size * size], vec![0.0; size]);
        stamp_step(shape, live_params, h, vc_from, i_prev, method, ic_ctx, &bnd, size, &z, &mut a, &mut b);
        let block: Vec<f64> = (0..n).flat_map(|r| a[r * size..r * size + n].to_vec()).collect();
        drop(factors);
        Some(crate::lu::lu_factor(block, n).map_err(|_| StepError::Failed("stepped: singular or non-finite pivot in a Newton iteration".to_string()))?)
    } else {
        None
    };
    let u = z[n];
    z.truncate(n);
    let rail_q = cap.filter(|_| !cc.seed).map(|c| c.end(h, method, u));
    Ok(StepResult { x: z, vc, i: i_new, lu, u: Some(u), rail_q, stage: None })
}

/// A seed's capacitor currents in the branch formulation (W2c, the ruling on
/// option (A)): each capacitor an ideal source of its charge `vc` in series
/// with its esr, its current an unknown of the solve, with the rest of the
/// network linearised at the seed's own solution `x` (`size` is `shape.n`,
/// or `shape.n + 1` on a CC rail, whose rail `x[n]` then enters as a known
/// boundary). Through the seed's companion the current is `C / SEED_H`
/// times the node's offset from the charge, so a node an ulp off a settled
/// charge read up to `C / 1 ps x ulp / 2` (0.44 uA for 1 mF at 5 V, 1.9 nA
/// on corpus 35's 10 uF), where the true current is ~0; here it comes out
/// of the solve to the rounding of the currents themselves. Only for a
/// capacitor with a charge: at `vc = 0` (every cold load) the companion's
/// current has no cancellation, and is kept bit for bit. A singular system
/// (a loop of esr-free capacitors) keeps the companion's.
#[allow(clippy::too_many_arguments)]
fn seed_branch_currents(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, ic_ctx: &IcStampCtx, bnd: &Bnd, exact: Option<&crate::solve::ExactStamp>, size: usize, x: &[f64], vc_from: &[f64], i: &mut [f64]) {
    if vc_from.iter().all(|&v| v == 0.0) {
        return;
    }
    let (n, m) = (shape.n, shape.capacitors.len());
    let (mut a, mut b) = (vec![0.0; size * size], vec![0.0; size]);
    stamp_linear_base(shape, size, &mut a, &mut b, live_params, bnd);
    stamp_ics_into_dyn(size, &mut a, &mut b, ic_ctx.now, ic_ctx.drivers, ic_ctx.ics, ic_ctx.node_owner, bnd, ic_ctx.p_idx, x);
    stamp_junctions(&shape.junctions, size, &mut a, &mut b, x, bnd);
    if let Some(e) = exact {
        e.apply(size, &mut a, &mut b);
    }
    // Exact mode's source row (`ExactKind::Row`, a CV rail or an `r = 0`
    // cell) is a constraint, not the rail's KCL: no capacitor current
    // enters it.
    let constraint = exact.filter(|e| matches!(e.kind, crate::solve::ExactKind::Row { .. })).map(|e| e.local);
    let dim = n + m;
    let (mut mna, mut rhs) = (vec![0.0; dim * dim], vec![0.0; dim]);
    for r in 0..n {
        mna[r * dim..r * dim + n].copy_from_slice(&a[r * size..r * size + n]);
        rhs[r] = b[r] - (n..size).map(|c| a[r * size + c] * x[c]).sum::<f64>();
    }
    for (k, cap) in shape.capacitors.iter().enumerate() {
        let row = n + k;
        for (pin, sign) in [(&cap.pins[0], 1.0), (&cap.pins[1], -1.0)] {
            match bnd.local(pin) {
                Some(l) if l < n => {
                    if constraint != Some(l) {
                        mna[l * dim + row] += sign;
                    }
                    mna[row * dim + l] += sign;
                }
                Some(l) => rhs[row] -= sign * x[l],
                None => rhs[row] -= sign * bnd.get(pin.node),
            }
        }
        mna[row * dim + row] = -cap.esr;
        rhs[row] += vc_from[k];
    }
    if let Ok(sol) = crate::lu::solve(mna, dim, rhs) {
        for k in 0..m {
            if vc_from[k] != 0.0 && sol[n + k].is_finite() {
                i[k] = sol[n + k];
            }
        }
    }
}

/// One step's network, every element about `x`: the linear base, each
/// capacitor's companion, the ICs and the junctions, at `size >= shape.n`
/// (the rail is unknown `shape.n` in a residual stamp, Rails point 3).
#[allow(clippy::too_many_arguments)]
fn stamp_step(
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc_from: &[f64],
    i_prev: &[f64],
    method: Method,
    ic_ctx: &IcStampCtx,
    bnd: &Bnd,
    size: usize,
    x: &[f64],
    a: &mut [f64],
    b: &mut [f64],
) {
    stamp_linear_base(shape, size, a, b, live_params, bnd);
    for (i, cap) in shape.capacitors.iter().enumerate() {
        let contrib = companion(cap, h, vc_from[i], i_prev[i], method);
        stamp_two_terminal(size, a, b, &cap.pins[0], &cap.pins[1], contrib.conductance, contrib.injection, bnd);
    }
    stamp_ics_into_dyn(size, a, b, ic_ctx.now, ic_ctx.drivers, ic_ctx.ics, ic_ctx.node_owner, bnd, ic_ctx.p_idx, x);
    stamp_junctions(&shape.junctions, size, a, b, x, bnd);
}

/// The rail's capacitor cluster in a partition: the local nodes the rail
/// (the `ext` unknown `shape.n` of `bnd`) reaches through capacitors alone,
/// and the capacitors among them and the rail ([`ChargeBalance`]). `None`
/// when no capacitor has a pin on the rail, or when one in the cluster has
/// its other pin off the partition (a capacitor to a fixed node: no KCL there
/// to read its current from).
fn rail_capacitors(shape: &DynShape, bnd: &Bnd) -> Option<(Vec<usize>, Vec<usize>)> {
    let n = shape.n;
    // Local index, the rail as `n`, or `None` for a fixed node.
    let end = |pin: &DynPin| bnd.local(pin);
    let mut cluster = vec![false; n + 1];
    cluster[n] = true;
    loop {
        let mut grew = false;
        for cap in &shape.capacitors {
            let (a, b) = (end(&cap.pins[0]), end(&cap.pins[1]));
            for (this, other) in [(a, b), (b, a)] {
                if this.is_some_and(|l| cluster[l]) {
                    match other {
                        Some(l) if !cluster[l] => {
                            cluster[l] = true;
                            grew = true;
                        }
                        Some(_) => {}
                        None => return None,
                    }
                }
            }
        }
        if !grew {
            break;
        }
    }
    let caps: Vec<usize> = shape.capacitors.iter().enumerate().filter(|(_, c)| end(&c.pins[0]).is_some_and(|l| cluster[l])).map(|(i, _)| i).collect();
    if caps.is_empty() {
        return None;
    }
    let nodes = (0..n).filter(|&l| cluster[l]).collect();
    Some((caps, nodes))
}

/// A hub seed's draw by charge balance (README, Rails point 4, R28), and on
/// a CC rail the seed's rail row: the
/// current into the partition from the rail as every branch at the rail but
/// its capacitors, plus each rail capacitor's current read from KCL at its
/// far node, the rest of that node's branches; when the far node holds more
/// capacitors, KCL over the whole cluster the rail reaches through
/// capacitors (cold fuzz 953: 0.71 mF to node 3, 18 mF on to node 2, whose
/// 1.8e10 S read the rounding into node 3's KCL). At `SEED_H` a capacitor is
/// `C / SEED_H` siemens (up to 2e10 S), so in the rail row its companion
/// magnified the solve's rounding: the rail solved with it was off by
/// `eps C / (SEED_H g)`, 2e-6 V for G2bv's 213 uF behind 0.03 S, and a draw
/// read from it moved by 1e-14 V of node times 2e8 S. Held at the rail
/// the loop last solved, such a seed's draw kept the rail loop moving 1e-6
/// to 2e-3 V a pass without settling (G2bv's seed rounding). The sum of the
/// rail row and the cluster's node rows is the same equation with the
/// capacitors' terms cancelled exactly; stamped without them it holds only
/// ordinary conductances.
struct ChargeBalance {
    caps: Vec<usize>,
    nodes: Vec<usize>,
}

impl ChargeBalance {
    /// The rail row of the stamp at `z` without the cluster's capacitors,
    /// plus the cluster's node rows, into `row` and `rhs` (size `shape.n +
    /// 1`): each capacitor's current enters two of them and cancels.
    #[allow(clippy::too_many_arguments)]
    fn row(&self, shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, h: f64, vc_from: &[f64], ic_ctx: &IcStampCtx, bnd: &Bnd, z: &[f64], row: &mut [f64]) -> f64 {
        let n = shape.n;
        let size = n + 1;
        let (mut a, mut b) = (vec![0.0; size * size], vec![0.0; size]);
        stamp_linear_base(shape, size, &mut a, &mut b, live_params, bnd);
        for (i, cap) in shape.capacitors.iter().enumerate() {
            if self.caps.contains(&i) {
                continue;
            }
            // A seed is backward Euler, which reads no previous current.
            let contrib = companion(cap, h, vc_from[i], 0.0, Method::Be);
            stamp_two_terminal(size, &mut a, &mut b, &cap.pins[0], &cap.pins[1], contrib.conductance, contrib.injection, bnd);
        }
        stamp_ics_into_dyn(size, &mut a, &mut b, ic_ctx.now, ic_ctx.drivers, ic_ctx.ics, ic_ctx.node_owner, bnd, ic_ctx.p_idx, z);
        stamp_junctions(&shape.junctions, size, &mut a, &mut b, z, bnd);
        row.copy_from_slice(&a[n * size..(n + 1) * size]);
        let mut rhs = b[n];
        for &l in &self.nodes {
            for (r, x) in row.iter_mut().zip(&a[l * size..(l + 1) * size]) {
                *r += x;
            }
            rhs += b[l];
        }
        rhs
    }

    /// The draw at `z`: the row's residual.
    #[allow(clippy::too_many_arguments)]
    fn draw(&self, shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, h: f64, vc_from: &[f64], ic_ctx: &IcStampCtx, bnd: &Bnd, z: &[f64]) -> f64 {
        let mut row = vec![0.0; shape.n + 1];
        let rhs = self.row(shape, live_params, h, vc_from, ic_ctx, bnd, z, &mut row);
        row.iter().zip(z).map(|(a, z)| a * z).sum::<f64>() - rhs
    }
}

/// Exact mode's source current at a solution `x` of the rail's partition
/// solved over `h` from `(vc_from, i_from)` (Rails point 9): a Norton's
/// branch current, `(E - u)/r`, or else what the rest of the partition draws
/// from the rail node there, that row's KCL residual without the source's
/// own stamp, the network linearised about `x` itself (the nonlinear
/// devices' exact current) with the capacitor companions the solve used. The
/// two agree to the Newton tolerance.
#[allow(clippy::too_many_arguments)]
fn source_current(
    e: &ExactStamp,
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc_from: &[f64],
    i_from: &[f64],
    method: Method,
    ic_ctx: &IcStampCtx,
    bnd: &Bnd,
    x: &[f64],
) -> f64 {
    match e.kind {
        ExactKind::Norton { g, inject } => inject - g * x[e.local],
        ExactKind::Row { .. } | ExactKind::Inject { .. } => {
            let n = shape.n;
            stamp_residual(n, x, |a, b| stamp_step(shape, live_params, h, vc_from, i_from, method, ic_ctx, bnd, n, x, a, b))[e.local]
        }
    }
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
///
/// A hub partition's seed (`keep_lu`) is polished, one Newton iteration past
/// the passing iterate (ruling R25 (a)): its draw is what the rail solves
/// from, and through `C / SEED_H` a capacitor's current magnifies a Newton
/// tolerance's offset into amps. On G2bv's seed 396 a switch closed an LED
/// from the rail across a capacitor's held charge, a 330 A loop back into
/// the rail; the passing iterate's draw was +-0.45 A off, differently at
/// each rail, and the rail loop alternated the rail by 0.38 V until its
/// 50-pass `-4`. Classic's seed is not polished; the polish moves the seed
/// toward the solution of the same equations.
fn solve_initial_x(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, vc: &[f64], x_from: &[f64], ic_ctx: &IcStampCtx) -> Result<StepResult, String> {
    seed_noting(shape, live_params, vc, x_from, ic_ctx, None).map_err(StepError::into_message)
}

fn seed_noting(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, vc: &[f64], x_from: &[f64], ic_ctx: &IcStampCtx, noting: Option<Noting>) -> Result<StepResult, StepError> {
    let i_prev = vec![0.0; shape.capacitors.len()]; // BE never reads `i_prev`; only trap does.
    let mut newton_iters = 0u64;
    solve_step_noting(shape, live_params, SEED_H, vc, x_from, &i_prev, Method::Be, &mut newton_iters, ic_ctx, ic_ctx.now, ic_ctx.keep_lu, noting)
}

/// Both modes (default mode's retry `PartitionedSolver::retry_stepped_holds`,
/// exact mode's `exact_retry_stepped`): the seed of a partition
/// holding a free IC supply with W1c
/// point 1a around it, as the static solver runs it around an iterated
/// partition's Newton (`StaticPartitionSolver::hold_in`). A seed whose Newton
/// fails with gated ICs' verdicts alternating (a chip whose own draw takes
/// its supply out of its window, while its absence lets the supply come
/// back) is solved with those held powered; the ones reading unpowered there
/// starve themselves, are solved held unpowered for the drop, and the seed
/// is solved with them held off. Returns the seed and the holds it took,
/// which the caller enters in its copy and the owner in its table
/// (`PartitionedSolver::take_stepped_holds`); any other failure is the
/// seed's own. Without it a 555 reaching its window's floor through 115
/// ohm from the rail failed the seed's Newton (cold 656 tpl 3, -4) before
/// any hold; held and retried (`PartitionedSolver::retry_stepped_holds`)
/// it slides along the edge and declines through ruling R54 (a)'s guard.
fn seed_holding(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, vc: &[f64], x_from: &[f64], ic_ctx: &IcStampCtx) -> Result<(StepResult, Vec<(usize, digital::power::PowerHold)>), String> {
    if !holds_around(ic_ctx) {
        return solve_initial_x(shape, live_params, vc, x_from, ic_ctx).map(|s| (s, Vec::new()));
    }
    let mut solve = |ctx: &IcStampCtx, noting: Option<Noting>| seed_noting(shape, live_params, vc, x_from, ctx, noting);
    hold_around(ic_ctx, &mut solve).map(|(seed, held)| (seed, held.map_or_else(Vec::new, |h| h.1))).map_err(StepError::into_message)
}

/// Whether a solve under `ic_ctx` runs W1c point 1a around its Newton
/// (`hold_around`): either mode, a partition holding a free IC supply, off
/// a CC rail.
fn holds_around(ic_ctx: &IcStampCtx) -> bool {
    ic_ctx.free_supply && ic_ctx.iterated && ic_ctx.cc.is_none()
}

/// W1c point 1a around one solve of a stepped partition (`solve`, a seed or
/// a step, under the context it is handed), as the static solver runs it
/// around an iterated partition's Newton (`StaticPartitionSolver::hold_in`):
/// a solve whose Newton fails with gated ICs' verdicts alternating (a chip
/// whose own draw takes its supply out of its window while its absence lets
/// it come back) is solved with those held powered; the ones reading
/// unpowered there starve themselves, are solved held unpowered for the
/// drop, and the solve is done with them held off. Returns the solution
/// and, when it held any, the table it solved under and the holds taken;
/// any other failure is the solve's own.
#[allow(clippy::type_complexity)]
fn hold_around(ic_ctx: &IcStampCtx, solve: &mut dyn FnMut(&IcStampCtx, Option<Noting>) -> Result<StepResult, StepError>) -> Result<(StepResult, Option<(Vec<digital::power::PowerHold>, Vec<(usize, digital::power::PowerHold)>)>), StepError> {
    use crate::solve::hold;
    let local = |node: usize| ic_ctx.node_owner.get(&node).is_some_and(|&(p, _)| p == ic_ctx.p_idx);
    let gated: Vec<(usize, &digital::IcRecord)> = ic_ctx.ics.iter().filter(|(_, ic)| local(ic.vcc.node) || local(ic.gnd.node)).map(|(d, ic)| (*d, ic)).collect();
    let mut flips = hold::Flips::new(gated.len());
    match solve(ic_ctx, Some((&gated, &mut flips))) {
        Ok(solved) => return Ok((solved, None)),
        Err(StepError::NoConverge) => {}
        Err(err) => return Err(err),
    }
    let a: Vec<usize> = flips.alternating();
    if a.is_empty() {
        return Err(StepError::NoConverge);
    }
    let supply = |x: &[f64], ic: &digital::IcRecord| {
        let v = |node: usize| match ic_ctx.node_owner.get(&node) {
            Some(&(p, l)) if p == ic_ctx.p_idx => x[l],
            _ => ic_ctx.bnd.get(node),
        };
        v(ic.vcc.node) - v(ic.gnd.node)
    };
    let mut table: Vec<digital::power::PowerHold> = ic_ctx.bnd.holds.to_vec();
    let top = gated.iter().map(|g| g.0).max().unwrap_or(0);
    if table.len() <= top {
        table.resize(top + 1, digital::power::PowerHold::FREE);
    }
    let mut under = |table: &[digital::power::PowerHold]| {
        let ctx = IcStampCtx { bnd: ic_ctx.bnd.with_holds(table), ..*ic_ctx };
        solve(&ctx, None).ok()
    };
    for &k in &a {
        table[gated[k].0] = hold::probe(true);
    }
    let Some(on) = under(&table) else { return Err(StepError::NoConverge) };
    let starving: Vec<(usize, f64)> = a
        .iter()
        .map(|&k| gated[k])
        .filter(|&(_, ic)| !digital::power::powered(ic, supply(&on.x, ic), 0.0))
        .map(|(d, ic)| (d, supply(&on.x, ic)))
        .collect();
    if starving.is_empty() {
        return Err(StepError::NoConverge);
    }
    for &k in &a {
        table[gated[k].0] = ic_ctx.bnd.hold(gated[k].0);
    }
    for &(d, _) in &starving {
        table[d] = hold::probe(false);
    }
    let Some(off) = under(&table) else { return Err(StepError::NoConverge) };
    let mut taken = Vec::new();
    for &(d, v_on) in &starving {
        let ic = gated.iter().find(|g| g.0 == d).unwrap().1;
        let held = hold::held_off(ic, v_on, supply(&off.x, ic), ic_ctx.now);
        table[d] = held;
        taken.push((d, held));
    }
    let Some(solved) = under(&table) else { return Err(StepError::NoConverge) };
    Ok((solved, Some((table, taken))))
}

#[derive(Clone)]
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
    /// INT-ACC (R79): a trapezoidal partition whose step failed its estimate
    /// at the floor rings (a stiff mode the trapezoid does not damp): it
    /// steps by backward Euler until its next discontinuity.
    trap_off: bool,
    pend_trap_off: bool,
    version: u64,
    has_pend: bool,
    pend_t: f64,
    pend_x: Vec<f64>,
    pend_vc: Vec<f64>,
    pend_i: Vec<f64>,
    watches: HashMap<usize, Vec<(DeviceId, SharedThresholdFn)>>,
    /// Contract stats 10/11 (this partition's share). NOT rolled back
    /// (core/README.md, "A failed advance").
    steps_taken: u64,
    /// R81 (1): times this partition fell back from the trapezoid to
    /// backward Euler (stat `27`). NOT rolled back, as `steps_taken` is not.
    trap_fallbacks: u64,
    newton_iterations: u64,
    /// Other Stepped partitions holding a supply node of an IC that watches
    /// one of this partition's nodes (`SteppedSolver::add_supply_dependency`).
    /// A build-time fact, like `holds_free_ic_supply`, so not part of any
    /// snapshot: see [`SteppedSolver::sync_supplies`] for what it drives.
    supply_deps: Vec<usize>,
    /// The state before the last committed step (`prev_t`, `prev_x`), valid
    /// while `has_prev`: a dependent partition interpolates this one's
    /// pending trial with it ([`SupplyTrace`]). Dropped at a discontinuity.
    prev_t: f64,
    prev_x: Vec<f64>,
    has_prev: bool,
    /// The nodes outside it its stepped devices read (and a CC rail), as
    /// its last committed step ended: the next step's start state (ruling
    /// R80 (a)).
    end_reads: Vec<(usize, f64)>,
    /// The longest step its stepped stressed devices let it take (ruling
    /// R90 (a), `SteppedSolver::set_stress_reach`): infinite while none is;
    /// and whether one was stressed at the last commit, without which its
    /// trial is not checked for a replan.
    stress_reach: f64,
    stress_watch: bool,
    /// Whether this partition holds a free IC supply node (W1c contract
    /// point 3) — a build-time topology fact (`SteppedSolver::build`
    /// computes it once, from `collect_ics`/`local_index`), not part of any
    /// snapshot/restore: it can never change after load, since
    /// `partition::build`'s own union rule decides it before this solver
    /// even exists.
    holds_free_ic_supply: bool,
    /// Whether this partition's pre-image is already in the open window's
    /// journal ([`SteppedSolver::journal`]).
    journaled: bool,
    /// W1f: whether the partition reads the rail (a hub partition, README
    /// Rails point 2), a build-time fact.
    hub: bool,
    /// R79 (a): the partition reads the PSU's rail node (`hub`, set only
    /// where the hub solves the rail) or, in exact mode, holds it with the
    /// supply's own regime. It keeps backward Euler: the rail jumps at a
    /// regime change with no discontinuity for the partition, which
    /// backward Euler steps through and the trapezoid carries as a ring
    /// (exact mode's cc pulse board, 13.6 mV off within 1 us of the CC
    /// entry).
    on_rail: bool,
    /// Whether it holds a pin of an IC supplied from the rail, whose driver
    /// curves it in the rail (Rails point 3), a build-time fact.
    rail_ic: bool,
    /// A hub partition's sensitivity at its last committed step (or seed):
    /// it publishes `x + s (u - u_p)` (Rails point 3).
    sens: Option<Rc<StepSens>>,
    /// What the pending trial was solved with, for its sensitivity when it
    /// commits: its factors, the rail and the instant it read, its `h` and
    /// method.
    pend_lu: Option<Rc<LuFactorization>>,
    pend_u: f64,
    pend_now: f64,
    pend_h: f64,
    pend_method: Method,
    /// The CC rail the pending trial was solved on (`None` off one): a CC
    /// exit at an internal event brings the partition to it on this rail
    /// (`SteppedSolver::catch_up_on_cc_rail`).
    pend_cc: Option<CcStep>,
    /// Exact mode's rail partition: the charge the hub source delivers over
    /// the pending trial, from the trial's own solves, so a battery's SoC is
    /// charged with it when the trial commits (Rails point 9).
    pend_charge: f64,
    /// Exact mode's rail partition: the source's current at the committed
    /// step's end, where a trapezoidal step's charge starts (a seed does not
    /// set it: the step after one is backward Euler), and at the pending
    /// trial's end.
    i_src: f64,
    pend_i_src: f64,
    /// On a CC rail with capacitors (ruling R68): their charge and current
    /// at the pending trial's end, and at the last commit `(t, q, i)`, from
    /// which the partition's next step starts when it starts there.
    pend_rail_q: Option<(f64, f64)>,
    rail_q: Option<(f64, f64, f64)>,
    /// Ruling R59 (1) (B): a closed-form hub partition that steps only while
    /// its PSU is in CC (`PartitionBuild::cc_switch`) is parked here in CV:
    /// nothing steps, publishes or schedules it, and its events are stale.
    /// The owner sets it (`SteppedSolver::promote`/`demote`) and journals it.
    parked: bool,
    /// Within a horizon clamp (`sync_partition`): a step of it left CC at
    /// its cut, and the clamp's later steps are solved in CV.
    cc_exited: bool,
    /// Whether it is a `cc_switch` partition (set at its first promotion):
    /// its CC steps take `CC_STEP_TOL_SCALE`.
    cc_switch: bool,
    /// W2b: the pending trial's knots after its start, while the observer
    /// reads the partition ([`SteppedSolver::set_dense`]): each solved
    /// sub-step's end and each TR-BDF2 stage's, the last at `pend_t`.
    pend_dense: PendDense,
    /// Test builds keep the pending trial's first half step, `(its end, x,
    /// capacitor currents)`, for [`ExactCommit`].
    #[cfg(test)]
    pend_mid: MidHalf,
}

#[cfg(test)]
type MidHalf = Option<(f64, Vec<f64>, Vec<f64>)>;

/// A stepped hub partition's sensitivity at a committed step, and its band
/// (Rails point 3, as a static hub partition's `HubSolve`).
#[derive(Debug, Clone, PartialEq)]
struct StepSens {
    s: Vec<f64>,
    i: f64,
    g: f64,
    u_p: f64,
    band: Option<f64>,
}

impl Partition {
    /// Whether the stamp depends on the iterate (`IcStampCtx::iterated`).
    fn iterated(&self) -> bool {
        self.holds_free_ic_supply || !self.shape.junctions.is_empty()
    }

    /// Whether a step solving it on a CC rail (`cc`, the rail one of the
    /// step's unknowns, [`CcRail`]) must iterate: iterated, or holding a
    /// rail-supplied IC pin, whose driver's `R_out` (`vS / 4 ioMax`) and
    /// targets move with that rail. Solved in one linear pass about the
    /// step's start, a seed from 5.29 V landed G2bv's bc_ic 425 hub 2 rail
    /// 8.5 mV over the one its gate's stamps hold there (5.8 mA left at the
    /// gate's output node): the retry freed the gate 0.22 s before classic
    /// does, and every step after it drew the rail under the gate's floor,
    /// flipped it twice and restarted (10 ns steps, F5l).
    fn iterated_on(&self, cc: bool) -> bool {
        self.iterated() || cc && self.rail_ic
    }

    /// Whether its response to the rail is curved, so it solves again past
    /// its band (Rails point 3): iterated, or holding a rail-supplied IC pin.
    fn curved(&self) -> bool {
        self.iterated() || self.rail_ic
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
            trap_off: false,
            pend_trap_off: false,
            version: 0,
            has_pend: false,
            pend_t: 0.0,
            pend_x: Vec::new(),
            pend_vc: Vec::new(),
            pend_i: Vec::new(),
            watches: HashMap::new(),
            steps_taken: 0,
            trap_fallbacks: 0,
            newton_iterations: 0,
            supply_deps: Vec::new(),
            prev_t: t0,
            prev_x: Vec::new(),
            end_reads: Vec::new(),
            stress_reach: f64::INFINITY,
            stress_watch: false,
            has_prev: false,
            holds_free_ic_supply: false, // overwritten by `SteppedSolver::build` right after construction.
            journaled: false,
            hub: false,
            on_rail: false,
            rail_ic: false,
            sens: None,
            pend_lu: None,
            pend_u: 0.0,
            pend_now: t0,
            pend_h: 0.0,
            pend_method: Method::Be,
            pend_cc: None,
            pend_charge: 0.0,
            i_src: 0.0,
            pend_i_src: 0.0,
            pend_rail_q: None,
            rail_q: None,
            parked: false,
            cc_exited: false,
            cc_switch: false,
            pend_dense: PendDense::default(),
            #[cfg(test)]
            pend_mid: None,
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
    rail_charge: Option<(f64, f64)>,
}

impl Snapshot {
    /// Every captured value, printed in partition order (tests compare states
    /// by this; the map's own order is arbitrary).
    #[cfg(test)]
    pub(crate) fn digest(&self) -> String {
        let ordered: std::collections::BTreeMap<_, _> = self.partitions.iter().collect();
        format!("{ordered:?} {:?} {:?}", self.scheduled, self.rail_charge)
    }
}

#[derive(Debug, Clone, Default)]
struct PartitionSnapshot {
    x: Vec<f64>,
    vc: Vec<f64>,
    i_prev: Vec<f64>,
    t: f64,
    h: f64,
    changing: bool,
    be_next_step: bool,
    trap_off: bool,
    pend_trap_off: bool,
    version: u64,
    has_pend: bool,
    pend_t: f64,
    pend_x: Vec<f64>,
    pend_vc: Vec<f64>,
    pend_i: Vec<f64>,
    prev_t: f64,
    prev_x: Vec<f64>,
    has_prev: bool,
    end_reads: Vec<(usize, f64)>,
    stress_reach: f64,
    stress_watch: bool,
    sens: Option<Rc<StepSens>>,
    pend_lu: Option<Rc<LuFactorization>>,
    pend_u: f64,
    pend_now: f64,
    pend_h: f64,
    pend_method: Option<Method>,
    pend_cc: Option<CcStep>,
    pend_charge: f64,
    i_src: f64,
    pend_i_src: f64,
    pend_rail_q: Option<(f64, f64)>,
    rail_q: Option<(f64, f64, f64)>,
    pend_dense: PendDense,
    #[cfg(test)]
    pend_mid: MidHalf,
}

impl PartitionSnapshot {
    /// Becomes a copy of `part`'s state, reusing this snapshot's buffers.
    fn fill_from(&mut self, part: &Partition) {
        self.x.clone_from(&part.x);
        self.vc.clone_from(&part.vc);
        self.i_prev.clone_from(&part.i_prev);
        self.t = part.t;
        self.h = part.h;
        self.changing = part.changing;
        self.be_next_step = part.be_next_step;
        self.trap_off = part.trap_off;
        self.pend_trap_off = part.pend_trap_off;
        self.version = part.version;
        self.has_pend = part.has_pend;
        self.pend_t = part.pend_t;
        self.pend_x.clone_from(&part.pend_x);
        self.pend_vc.clone_from(&part.pend_vc);
        self.pend_i.clone_from(&part.pend_i);
        self.prev_t = part.prev_t;
        self.prev_x.clone_from(&part.prev_x);
        self.has_prev = part.has_prev;
        self.end_reads.clone_from(&part.end_reads);
        self.stress_reach = part.stress_reach;
        self.stress_watch = part.stress_watch;
        self.sens.clone_from(&part.sens);
        self.pend_lu.clone_from(&part.pend_lu);
        self.pend_u = part.pend_u;
        self.pend_now = part.pend_now;
        self.pend_h = part.pend_h;
        self.pend_method = Some(part.pend_method);
        self.pend_cc = part.pend_cc;
        self.pend_charge = part.pend_charge;
        self.i_src = part.i_src;
        self.pend_i_src = part.pend_i_src;
        self.pend_rail_q = part.pend_rail_q;
        self.rail_q = part.rail_q;
        self.pend_dense.clone_from(&part.pend_dense);
        #[cfg(test)]
        self.pend_mid.clone_from(&part.pend_mid);
    }

    /// Puts the W1f fields back into `part`.
    fn restore_w1f(&self, part: &mut Partition) {
        part.sens.clone_from(&self.sens);
        part.pend_lu.clone_from(&self.pend_lu);
        part.pend_u = self.pend_u;
        part.pend_now = self.pend_now;
        part.pend_h = self.pend_h;
        part.pend_method = self.pend_method.unwrap_or(Method::Be);
        part.pend_cc = self.pend_cc;
        part.pend_charge = self.pend_charge;
        part.i_src = self.i_src;
        part.pend_i_src = self.pend_i_src;
        part.pend_rail_q = self.pend_rail_q;
        part.rail_q = self.rail_q;
        part.pend_dense.clone_from(&self.pend_dense);
        #[cfg(test)]
        part.pend_mid.clone_from(&self.pend_mid);
    }
}

/// Owns every `Stepped` partition (>= 2 capacitors, or a single one under
/// `FORCE_STEPPED` — Decision 14), keyed by GLOBAL partition index.
#[derive(Clone)]
pub struct SteppedSolver {
    partitions: HashMap<usize, Partition>,
    node_owner: crate::shared::Shared<HashMap<usize, (usize, usize)>>,
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
    ics: crate::shared::Shared<[(usize, digital::IcRecord)]>,
    /// node -> every IC device index that reads it (contract point 4's
    /// last bullet: "resolves of dynamic partitions must also report IC
    /// readers of nodes that changed, as static does").
    node_to_ic_readers: crate::shared::Shared<HashMap<usize, Vec<usize>>>,
    /// The voltage array a watch's thresholds are read from while a step is
    /// planned ([`ThresholdEnv`]), kept between plans so a plan costs a copy
    /// and not an allocation. No state: it is overwritten before every use.
    threshold_scratch: Vec<f64>,
    /// The rollback window (`PartitionedSolver::begin_instant`): while it is
    /// open, each partition's state is saved (`journal`) before the first
    /// call of the window that can change it, so a completed instant costs
    /// nothing and a failed one puts back only what it touched. `spare` holds
    /// the saved states of earlier windows for reuse, buffers and all.
    window: bool,
    journal: Vec<(usize, PartitionSnapshot)>,
    spare: Vec<PartitionSnapshot>,
    window_scheduled: Vec<(f64, usize, u64)>,
    /// W1f: the rail node outside exact mode, the running ramps (set by the
    /// owner when a resolved ramp starts or ends), exact mode's hub source
    /// (set by the owner before each call), the contributions committed
    /// since the owner last drained them, and stat 16's share.
    rail: Option<usize>,
    ramps: Option<Rc<RampEval>>,
    exact: Option<ExactStamp>,
    /// F5: the owner's device-indexed power-hold table (W1c point 1a) and
    /// whether any entry is not free; the owner journals it.
    holds: Rc<Vec<digital::power::PowerHold>>,
    holds_on: bool,
    /// Holds a seed took in exact mode (`seed_holding`), `(device, hold,
    /// when)`, for the owner to enter in its table after the call; dropped
    /// whenever the owner hands its table over (a rollback among them).
    hold_requests: Vec<(usize, digital::power::PowerHold, f64)>,
    /// Exact mode's battery partition, whose next step is planned only
    /// after the owner has advanced SoC with the step just committed, and
    /// the partitions whose plans are so held back (Rails point 9: E and r
    /// from the SoC at each step's start).
    defer: Option<usize>,
    deferred: Vec<usize>,
    /// Each step of the battery partition committed since the owner last
    /// took them, `(its end, the charge the source delivered over it)`: one
    /// call can commit several (a catch-up before a discontinuity re-solve,
    /// the horizon clamp), and each is charged with its own.
    bills: Vec<(f64, f64)>,
    /// Exact mode's cell: its model (set at load) and its state as of the
    /// last stamp the owner set. Each committed step of the battery
    /// partition is charged here and restamps the source, since one call
    /// can commit several (the horizon clamp, a catch-up) and the owner
    /// charges SoC only after the call; it charges the same bills in the
    /// same order and lands on the same state.
    cell: Option<(devices::battery::BatteryModel, devices::battery::BatteryState)>,
    hub_updates: Vec<(usize, Contrib)>,
    sensitivities: u64,
    /// The CC rail the hub partitions step against while the owner's call
    /// runs ([`CcRail`]); set by the owner before a call that may plan a
    /// step and taken after it, so it is never stale.
    pub(crate) cc: Option<CcRail>,
    /// Set while `catch_up_on_cc_rail` brings a partition to a CC exit: the
    /// CC rail its steps take in place of `cc`'s.
    cc_held: Option<CcStep>,
    /// The rail capacitors' charge `(t, q)` the latest commit of a stepped
    /// hub partition on a CC rail with capacitors stepped them to (ruling
    /// R68), for the rail solve after it in place of the segment's, which
    /// held that partition's draw linearised over the step. Taken by that
    /// solve; saved at the window's start for a rollback.
    rail_charge: Option<(f64, f64)>,
    window_rail_charge: Option<(f64, f64)>,
    /// Set while `sync_partition` steps a partition to its horizon.
    clamp: bool,
    /// The partitions the owner's `resolve_dirty` re-solves only because
    /// the rail moved past their band on a CC rail (no rail capacitor): a
    /// re-aim, not a discontinuity, so each keeps the step its controller
    /// chose. Two stepped LED and capacitor branches on one CC PSU each hold
    /// the other's draw where it last committed, so every commit of one moved
    /// the rail past the other's 1 µV band, and restarting it at
    /// `POST_EVENT_H` made 2.77e6 steps of a 0.1 s board (FXEFv, FXF round
    /// 3). Set by the owner around the call, empty otherwise.
    pub(crate) reaim: Vec<usize>,
    /// Every step committed since the owner last took them (ruling R71 (b),
    /// ACC-A): the partition, the step's start and end, its own end node
    /// voltages and, on a CC rail, its own rail. The owner closes each
    /// stepped slow device's integrators over each step with them, before a
    /// re-seed or a republish at the moved rail overwrites what is published.
    /// Taken at the instant's settle; cleared with a rollback.
    pub(crate) trail: Vec<Committed>,
    /// Ruling R90 (a): each partition's rated stepped devices
    /// (`set_stress_probes`).
    stress_probes: HashMap<usize, Vec<StressProbe>>,
    /// The partitions whose steps go on the trail (the owner's: a stepped slow
    /// device that is not cold sits in them), indexed by partition.
    pub(crate) trail_parts: Vec<bool>,
    /// The nodes outside each such partition its devices read: their values
    /// at the commit, the boundary the step was solved against.
    pub(crate) trail_reads: Vec<Vec<usize>>,
    /// Test builds keep every partition's state from when the window opened
    /// (keyed by partition) and check, at the window's end, that a partition
    /// the journal does not hold is unchanged, and after a rollback that all
    /// of them are back.
    /// W2b: the partitions the observer reads (by index), and each committed
    /// step and seed of theirs since its next sample ([`DenseSeg`]), oldest
    /// first; `w_dense` is how many the open window found.
    dense_on: Vec<bool>,
    /// The partitions with a probed node no capacitor's charge gives: an
    /// iterated one solves at each grid time in its steps.
    dense_solve: Vec<bool>,
    dense: Vec<DenseSeg>,
    w_dense: usize,
    /// The observer's grid `(origin, interval)`, for the solved samples.
    dense_grid: Option<(f64, f64)>,
    /// W2b piece 4: the partitions whose committed steps the charge weighs
    /// (by index), the nodes outside each its charge reads, and each step
    /// committed since the charge last drained them ([`ChargeStep`]);
    /// `w_charge` is how many the open window found.
    charge_on: Vec<bool>,
    charge_reads: Vec<Vec<usize>>,
    charge_steps: Vec<ChargeStep>,
    w_charge: usize,
    /// Test builds can keep every segment (no pruning), to check samples
    /// against the knots around them.
    #[cfg(test)]
    pub(crate) keep_dense: bool,
    #[cfg(test)]
    w_saved: Option<std::collections::BTreeMap<usize, String>>,
    /// Test builds record every committed step, `(partition, instant)`, so a
    /// test can hand-step a reference at the controller's own step sequence.
    #[cfg(test)]
    pub(crate) commit_log: Vec<(usize, f64)>,
    /// Test builds record every re-solve ([`Self::resolve_dirty`]), so a
    /// test can tell which kept their step (`reaim`) and which restarted.
    #[cfg(test)]
    pub(crate) resolve_log: Vec<ResolveRecord>,
    /// Test builds record each committed step of exact mode's rail
    /// partition with the solution it published and the source stamp it was
    /// solved with, so a test can work out the source's current at it.
    #[cfg(test)]
    pub(crate) exact_commit_log: Vec<ExactCommit>,
}

impl SteppedSolver {
    /// Each table this solver shares with snapshots, by address (W2a tests).
    #[cfg(test)]
    pub(crate) fn shared_tables(&self) -> Vec<(&'static str, usize)> {
        vec![("node_owner", crate::shared::Shared::addr(&self.node_owner)), ("ics", crate::shared::Shared::addr(&self.ics)), ("node_to_ic_readers", crate::shared::Shared::addr(&self.node_to_ic_readers))]
    }
}

/// One re-solve of a stepped partition ([`SteppedSolver::resolve_log`]).
#[cfg(test)]
#[derive(Debug, Clone, Copy)]
pub(crate) struct ResolveRecord {
    pub partition: usize,
    pub now: f64,
    /// The published rail when the re-solve ran (0 with no rail).
    pub u: f64,
    /// Whether it kept its step (a re-aim) rather than restarting at
    /// `POST_EVENT_H`.
    pub kept: bool,
    /// Whether it ran against a CC rail ([`CcRail`]).
    pub cc: bool,
}

/// `(device index, series current)` per capacitor.
#[cfg(test)]
pub(crate) type CapCurrents = Vec<(usize, f64)>;

/// One committed step of exact mode's rail partition ([`SteppedSolver::exact_commit_log`]).
#[cfg(test)]
#[derive(Debug, Clone)]
pub(crate) struct ExactCommit {
    pub t0: f64,
    /// The accepted pair's first half step, `(its end, x, caps)`, when the
    /// step was a pair.
    pub mid: Option<(f64, Vec<f64>, CapCurrents)>,
    pub t: f64,
    pub x: Vec<f64>,
    pub caps: CapCurrents,
    pub stamp: ExactStamp,
    /// Whether the step was trapezoidal.
    pub trap: bool,
}

impl SteppedSolver {
    pub(crate) fn build(program: &Program, partitions: &[Vec<usize>], local_index: &[Option<(usize, usize)>], fixed: &[Option<f64>], kinds: &[PartitionKind], topo: LiveTopology) -> Self {
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
        let rail_ic = topo.rail.map(|rail| crate::solve::curvature::holds_rail_ic_pin(&ics, local_index, rail, partitions.len()));

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
            part.hub = topo.rail.is_some() && topo.hub[p_idx];
            part.on_rail = topo.hub[p_idx] || topo.rail_node.is_some_and(|r| local_index[r].is_some_and(|(q, _)| q == p_idx));
            part.rail_ic = part.hub && rail_ic.as_ref().is_some_and(|r| r[p_idx]);
            parts.insert(p_idx, part);
        }
        SteppedSolver {
            partitions: parts,
            node_owner: node_owner.into(),
            method: Method::from_option(program.options.integration),
            scheduled: Vec::new(),
            ics: ics.into(),
            node_to_ic_readers: node_to_ic_readers.into(),
            threshold_scratch: Vec::new(),
            window: false,
            journal: Vec::new(),
            spare: Vec::new(),
            window_scheduled: Vec::new(),
            rail_charge: None,
            window_rail_charge: None,
            rail: topo.rail,
            ramps: None,
            exact: None,
            holds: Rc::new(Vec::new()),
            holds_on: false,
            hold_requests: Vec::new(),
            defer: None,
            deferred: Vec::new(),
            bills: Vec::new(),
            cell: None,
            hub_updates: Vec::new(),
            sensitivities: 0,
            cc: None,
            cc_held: None,
            clamp: false,
            reaim: Vec::new(),
            trail: Vec::new(),
            stress_probes: HashMap::new(),
            trail_parts: Vec::new(),
            trail_reads: Vec::new(),
            dense_on: Vec::new(),
            dense_solve: Vec::new(),
            dense: Vec::new(),
            w_dense: 0,
            dense_grid: None,
            charge_on: Vec::new(),
            charge_reads: Vec::new(),
            charge_steps: Vec::new(),
            w_charge: 0,
            #[cfg(test)]
            keep_dense: false,
            #[cfg(test)]
            w_saved: None,
            #[cfg(test)]
            commit_log: Vec::new(),
            #[cfg(test)]
            resolve_log: Vec::new(),
            #[cfg(test)]
            exact_commit_log: Vec::new(),
        }
    }

    /// The owner's power-hold table, after every change to it.
    pub(crate) fn set_power_holds(&mut self, holds: Rc<Vec<digital::power::PowerHold>>, on: bool) {
        self.holds = holds;
        self.holds_on = on;
        self.hold_requests.clear();
    }

    /// The holds the last calls' seeds took (`seed_holding`).
    pub(crate) fn take_hold_requests(&mut self) -> Vec<(usize, digital::power::PowerHold, f64)> {
        std::mem::take(&mut self.hold_requests)
    }

    /// The running ramps every step reads its boundary through (Sources
    /// point 3); `None` when none runs.
    pub(crate) fn set_ramps(&mut self, ramps: Option<Rc<RampEval>>) {
        self.ramps = ramps;
    }

    /// Exact mode's hub source for the calls that follow (Rails point 9).
    pub(crate) fn set_exact(&mut self, exact: Option<ExactStamp>) {
        self.exact = exact;
    }

    /// Exact mode's battery partition (Rails point 9). A step planned the
    /// moment the last one commits would read the SoC before that step is
    /// charged, so E and r would lag a step behind classic's. Its plan waits
    /// for [`Self::plan_deferred`] instead ([`Self::plan_and_schedule_in`]).
    pub(crate) fn set_defer(&mut self, p: Option<usize>) {
        self.defer = p;
    }

    /// Exact mode's cell, at load ([`Self::cell`]).
    pub(crate) fn set_cell(&mut self, model: devices::battery::BatteryModel, state: devices::battery::BatteryState) {
        self.cell = Some((model, state));
    }

    /// The cell's state the stamp just set came from (with
    /// [`Self::set_exact`]).
    pub(crate) fn set_cell_state(&mut self, state: devices::battery::BatteryState) {
        if let Some(cell) = self.cell.as_mut() {
            cell.1 = state;
        }
    }

    /// The stamp the next step of the rail's partition is solved with.
    pub(crate) fn exact_stamp(&self) -> Option<ExactStamp> {
        self.exact
    }

    /// Plans the steps held back in this call, now that the owner has set
    /// the stamp from the advanced SoC: a partition after the held ones that
    /// supply it, so its plan reads their new trials. Every supply was
    /// brought to the plan's instant when the plan was held back, so nothing
    /// is caught up here.
    pub(crate) fn plan_deferred(&mut self, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<(), String> {
        let mut held = std::mem::take(&mut self.deferred);
        while !held.is_empty() {
            // A supply cycle has no such partition; it keeps the held order.
            let k = held.iter().position(|p| !self.partitions[p].supply_deps.iter().any(|b| b != p && held.contains(b))).unwrap_or(0);
            let p = held.remove(k);
            self.plan_step(p, f64::INFINITY, drivers, live_params, voltages)?;
            let part = &self.partitions[&p];
            self.scheduled.push((part.pend_t, p, part.version));
        }
        Ok(())
    }

    /// The battery partition's steps committed since the last call, oldest
    /// first ([`Self::bills`]).
    pub(crate) fn take_bills(&mut self) -> std::vec::Drain<'_, (f64, f64)> {
        self.bills.drain(..)
    }

    /// The plans held back in this call, in the order they were held (tests
    /// reorder them).
    #[cfg(test)]
    pub(crate) fn deferred_mut(&mut self) -> &mut Vec<usize> {
        &mut self.deferred
    }

    /// Holds `p`'s plan back for [`Self::plan_deferred`].
    fn hold(&mut self, p: usize) {
        if !self.deferred.contains(&p) {
            self.deferred.push(p);
        }
    }

    pub(crate) fn drain_hub_updates(&mut self) -> std::vec::Drain<'_, (usize, Contrib)> {
        self.hub_updates.drain(..)
    }

    /// Each capacitor's committed charge voltage, by device (W2c's carry).
    /// Taken between calls, after the horizon clamp, so at the engine's time.
    pub(crate) fn carry_caps(&self, out: &mut Vec<(usize, f64)>) {
        // A parked `cc_switch` copy's charge is the closed form's to carry.
        for part in self.partitions.values().filter(|p| !p.parked) {
            out.extend(part.shape.capacitors.iter().zip(&part.vc).map(|(c, &vc)| (c.device_index, vc)));
        }
    }

    /// W2c's seed, at load before the first solve: capacitor `device`'s
    /// charge, which that solve seeds the partition's node voltages from
    /// (as from `vc = 0` on a cold load). A parked `cc_switch` copy takes it
    /// too. Whether this solver holds the capacitor.
    pub(crate) fn seed_vc(&mut self, device: usize, vc: f64) -> bool {
        let mut found = false;
        for part in self.partitions.values_mut() {
            if let Some(k) = part.shape.capacitors.iter().position(|c| c.device_index == device) {
                part.vc[k] = vc;
                found = true;
            }
        }
        found
    }

    /// W2b: the partitions the observer reads (`None`: none), from which
    /// each commit and seed is kept as a [`DenseSeg`] until read past.
    pub(crate) fn set_dense(&mut self, nodes: Option<&[usize]>, grid: Option<(f64, f64)>) {
        self.dense.clear();
        self.dense_on.clear();
        self.dense_solve.clear();
        self.dense_grid = grid;
        for &n in nodes.unwrap_or(&[]) {
            let Some(&(p, l)) = self.node_owner.get(&n) else { continue };
            let Some(part) = self.partitions.get(&p) else { continue };
            if self.dense_on.len() <= p {
                self.dense_on.resize(p + 1, false);
                self.dense_solve.resize(p + 1, false);
            }
            self.dense_on[p] = true;
            // Only a node no capacitor's charge gives is solved at the grid
            // times (R104 (b)); a plate reads its Hermite charge.
            self.dense_solve[p] |= plate_of(&part.shape, l).is_none();
        }
        for (&p, part) in self.partitions.iter_mut() {
            if !self.dense_on.get(p).copied().unwrap_or(false) && !self.charge_on.get(p).copied().unwrap_or(false) {
                part.pend_dense.clear();
            }
        }
    }

    /// W2b piece 4: the partitions whose committed steps the charge weighs
    /// (`None`: none), each with the nodes outside it the charge reads beside
    /// its own devices' boundary. Drops any step not yet drained.
    pub(crate) fn set_charge(&mut self, parts: Option<&[(usize, Vec<usize>)]>) {
        self.charge_steps.clear();
        self.charge_on.clear();
        self.charge_reads.clear();
        for (p, extra) in parts.unwrap_or(&[]) {
            let Some(part) = self.partitions.get(p) else { continue };
            if self.charge_on.len() <= *p {
                self.charge_on.resize(p + 1, false);
                self.charge_reads.resize(p + 1, Vec::new());
            }
            self.charge_on[*p] = true;
            let shape = &part.shape;
            let pins = shape.stamped.iter().flat_map(|s| s.pins.iter()).chain(shape.capacitors.iter().flat_map(|c| c.pins.iter())).chain(shape.junctions.iter().flat_map(|j| [&j.anode, &j.cathode]));
            let mut reads: Vec<usize> = pins.filter(|pin| pin.local.is_none()).map(|pin| pin.node).chain(extra.iter().copied()).filter(|n| !shape.nodes.contains(n)).collect();
            reads.sort_unstable();
            reads.dedup();
            self.charge_reads[*p] = reads;
        }
        for (&p, part) in self.partitions.iter_mut() {
            if !self.dense_on.get(p).copied().unwrap_or(false) && !self.charge_on.get(p).copied().unwrap_or(false) {
                part.pend_dense.clear();
            }
        }
    }

    /// The steps committed since the last drain, oldest first, emptied.
    /// Outside a window only (a rollback truncates them).
    pub(crate) fn drain_charge_steps(&mut self, out: &mut Vec<ChargeStep>) {
        debug_assert!(!self.window, "the charge drains between instants");
        out.clear();
        out.append(&mut self.charge_steps);
    }

    /// The nodes outside partition `p` its charge reads.
    pub(crate) fn charge_reads(&self, p: usize) -> &[usize] {
        self.charge_reads.get(p).map_or(&[], |r| &r[..])
    }

    /// Partition `p`'s compiled shape.
    pub(crate) fn shape_of(&self, p: usize) -> Option<&DynShape> {
        self.partitions.get(&p).map(|part| &part.shape)
    }

    /// Partition `p`'s pending step as it commits, from its committed state,
    /// its knots taken from the trial.
    fn dense_seg_taken(&mut self, p: usize) -> DenseSeg {
        let PendDense { mut knots, solved, .. } = std::mem::take(&mut self.partitions.get_mut(&p).expect("a committing partition").pend_dense);
        let part = &self.partitions[&p];
        if knots.is_empty() {
            // Planned before the observer read it: its ends only.
            knots.push(Knot { t: part.pend_t, x: part.pend_x.clone(), vc: part.pend_vc.clone(), i: part.pend_i.clone(), hermite: false, stage: false, u: None, rail_i: None });
        }
        // A linear partition with one capacitor: every node an affine map of
        // its voltage when nothing it reads moves within the step (R104:
        // recomputed from the charge where cheap).
        let steady = !part.hub && self.ramps.is_none();
        let affine = steady && part.shape.capacitors.len() == 1 && !part.iterated();
        let sens = part.sens.as_ref().map(|s| (s.s.clone(), s.u_p));
        DenseSeg { p, t0: part.t, x0: part.x.clone(), vc0: part.vc.clone(), i0: part.i_prev.clone(), knots, solved, steady, affine, sens }
    }

    /// Partition `p`'s pending step as it commits, for the charge: its start,
    /// its knots, and its reads at each point as `plan_step` solved against
    /// them (a running ramp at the point's time, a CC rail at the step's own
    /// rail).
    fn charge_step(&self, p: usize, voltages: &[f64]) -> ChargeStep {
        let part = &self.partitions[&p];
        let mut knots = part.pend_dense.knots.clone();
        let u0 = part.pend_dense.u0;
        let ends_only = knots.is_empty();
        if ends_only {
            knots.push(Knot { t: part.pend_t, x: part.pend_x.clone(), vc: part.pend_vc.clone(), i: part.pend_i.clone(), hermite: false, stage: false, u: None, rail_i: part.pend_rail_q.map(|r| r.1) });
        }
        let reads = self.charge_reads(p);
        // On a CC rail each point at the rail it was solved with (the end's
        // where a point did not report one).
        let rail = match (part.hub, self.rail, part.pend_cc) {
            (true, Some(r), Some(_)) => Some(r),
            _ => None,
        };
        let mut vals = Vec::with_capacity(reads.len() * (knots.len() + 1));
        let points = std::iter::once((part.t, u0)).chain(knots.iter().map(|k| (k.t, k.u)));
        for (t, u) in points {
            let bnd = Bnd::new(voltages).at_time(self.ramps.as_deref(), t);
            vals.extend(reads.iter().map(|&n| match rail {
                Some(r) if r == n => u.unwrap_or(part.pend_u),
                _ => bnd.get(n),
            }));
        }
        ChargeStep { p, t0: part.t, x0: part.x.clone(), i0: part.i_prev.clone(), knots, vals, ends_only, rail_i0: part.pend_dense.rail_i0.filter(|_| !ends_only) }
    }

    /// Drops what no sample at or after `t` needs: each partition's latest
    /// segment stays. Outside a window only (a rollback truncates).
    pub(crate) fn prune_dense(&mut self, t: f64) {
        if self.window || self.dense.is_empty() {
            return;
        }
        #[cfg(test)]
        if self.keep_dense {
            return;
        }
        // Nothing ends before `t` (the common case between two samples).
        if self.dense.iter().all(|seg| seg.end() >= t) {
            return;
        }
        // Each partition's latest segment, by a scan from the newest (few
        // partitions are probed).
        let mut latest: Vec<(usize, usize)> = Vec::new();
        for (k, seg) in self.dense.iter().enumerate().rev() {
            if !latest.iter().any(|&(p, _)| p == seg.p) {
                latest.push((seg.p, k));
            }
        }
        let mut k = 0;
        self.dense.retain(|seg| {
            let keep = seg.end() >= t || latest.contains(&(seg.p, k));
            k += 1;
            keep
        });
    }

    /// Every kept segment of `node`'s partition as its points, `(t, node,
    /// whether the interval ending there is Hermite)`, the start first (tests).
    #[cfg(test)]
    pub(crate) fn debug_knots(&self, node: usize) -> Vec<Vec<(f64, f64, bool)>> {
        let Some(&(p, l)) = self.node_owner.get(&node) else { return Vec::new() };
        self.dense.iter().filter(|s| s.p == p).map(|s| std::iter::once((s.t0, s.x0[l], false)).chain(s.knots.iter().map(|k| (k.t, k.x[l], k.hermite))).collect()).collect()
    }

    /// How many TR-BDF2 stage ends the kept segments of `node`'s partition
    /// hold (tests).
    #[cfg(test)]
    pub(crate) fn debug_stage_knots(&self, node: usize) -> usize {
        let Some(&(p, _)) = self.node_owner.get(&node) else { return 0 };
        self.dense.iter().filter(|s| s.p == p).map(|s| s.knots.iter().filter(|k| k.stage).count()).sum()
    }

    /// Where `node` is, for `node_at_located`: the observer looks it
    /// up once between instants, not at each sample.
    pub(crate) fn locate(&self, node: usize) -> Option<Located<'_>> {
        let &(p, l) = self.node_owner.get(&node)?;
        let part = self.partitions.get(&p)?;
        Some(Located { p, l, part, on_rail: self.rail.is_some() && part.hub })
    }

    /// W2b: `node`'s value at `t` from the committed steps around it (R104:
    /// cubic Hermite on each capacitor's voltage over a trapezoidal step or
    /// TR-BDF2 stage, from its currents at both ends; linear over a backward
    /// Euler step), with the step's span; `None` while its partition has not
    /// committed past `t`, or reads the rail (W2b piece 3).
    pub(crate) fn node_at_located(&self, at: &Located<'_>, t: f64, rail_u: Option<f64>) -> Option<(f64, f64, bool)> {
        let (p, l, part) = (at.p, at.l, at.part);
        // A hub partition publishes its rail-corrected value, `x + s (u -
        // u_p)` with its step's sensitivity (Rails point 3), at the rail's
        // model there.
        let on_rail = |seg: &DenseSeg, x: f64| match (part.hub && self.rail.is_some(), &seg.sens) {
            (false, _) => Some(x),
            (true, Some((s, u_p))) => rail_u.map(|u| x + s[l] * (u - u_p)),
            (true, None) => Some(x),
        };
        let mut latest = true;
        for seg in self.dense.iter().rev().filter(|s| s.p == p) {
            if t > seg.end() {
                // Past the partition's last commit: not final. Past an older
                // one: a later segment started at or before it, so it read t.
                return if latest { None } else { on_rail(seg, seg.value(&part.shape, l, seg.end()).0).map(|v| (v, seg.span(), false)) };
            }
            latest = false;
            if t >= seg.t0 {
                let (v, fallback) = seg.value(&part.shape, l, t);
                return on_rail(seg, v).map(|v| (v, seg.span(), fallback));
            }
        }
        None
    }

    pub(crate) fn sensitivities(&self) -> u64 {
        self.sensitivities
    }

    /// The hub partitions, ascending.
    /// Whether any hub partition steps (W2b: the rail then moves with its
    /// draws between solves).
    /// How many steps the hub partitions have committed (W2b: a rail solve
    /// after one has their draws fresh).
    pub(crate) fn hub_commits(&self) -> u64 {
        self.partitions.values().filter(|p| p.hub).map(|p| p.steps_taken).sum()
    }

    pub(crate) fn any_hub_stepping(&self) -> bool {
        self.partitions.values().any(|p| p.hub && !p.parked)
    }

    pub(crate) fn hub_partitions(&self) -> Vec<usize> {
        let mut out: Vec<usize> = self.partitions.iter().filter(|(_, p)| p.hub && !p.parked).map(|(&p, _)| p).collect();
        out.sort_unstable();
        out
    }

    /// The curved hub partitions whose last committed step is more than
    /// its band under `rule` from `u` (Rails point 3), ascending.
    ///
    /// With `linear_too` (a CC rail with no rail capacitor) a linear one
    /// counts as well: its committed draw line is its step's, the
    /// capacitors as their companions `C / h`, and a rail moved within the
    /// step by another partition's change meets the capacitors as they hold
    /// their charge, not as that line has them. Cold 677 tpl 5: a switch
    /// took a CC load off the rail, and 2.2 uF from the rail to 1.1 kohm,
    /// its line `C / h` = 0.09 S from a 24 us step, read 243 mA at the
    /// setpoint where it draws 4 mA: the PSU stayed in CC, the rail 0.86 V
    /// short, until that step's end. Only a promoted `cc_switch` partition
    /// (which stands in for a closed form re-aimed at every move): every
    /// stepped hub partition re-solved so slid cold 61 tpl 5's gate along
    /// its floor to the hold guard's `-4`.
    pub(crate) fn hubs_past_band(&self, u: f64, rule: rails::BandRule, linear_too: bool) -> Vec<usize> {
        let mut out: Vec<usize> = self
            .partitions
            .iter()
            .filter(|(_, p)| p.hub && !p.parked && (linear_too && p.cc_switch || p.curved()) && p.sens.as_ref().is_some_and(|s| (u - s.u_p).abs() > rule.band(s.band)))
            .map(|(&p, _)| p)
            .collect();
        out.sort_unstable();
        out
    }

    /// The `u_p` of every curved hub partition and its band under `rule`.
    pub(crate) fn nonlinear_hub_rails(&self, rule: rails::BandRule, out: &mut Vec<(f64, f64)>) {
        out.extend(self.partitions.values().filter(|p| p.hub && !p.parked && p.curved()).filter_map(|p| p.sens.as_ref().map(|s| (s.u_p, rule.band(s.band)))));
    }

    /// Re-publishes every hub partition at the present rail (a rail move).
    pub(crate) fn republish_hubs(&self, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) {
        let mut ids: Vec<usize> = self.partitions.iter().filter(|(_, p)| p.hub && !p.parked && p.sens.is_some()).map(|(&p, _)| p).collect();
        ids.sort_unstable();
        for p in ids {
            Self::publish(&self.partitions[&p], self.rail, live_params, voltages, currents);
        }
    }

    /// Brings each of `ids` to `now` with the boundary it has (a waveform
    /// edge is about to move it: W1b rule 6), committing the steps it takes
    /// and publishing nothing; the re-solve that follows publishes.
    pub(crate) fn catch_up(&mut self, ids: &[usize], now: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<(), String> {
        for &p in ids {
            if !self.partitions.get(&p).is_some_and(|part| !part.parked) {
                continue;
            }
            self.journal(p);
            self.sync_partition(p, now, drivers, live_params, voltages)?;
        }
        Ok(())
    }

    /// Ruling R68: each of `ids` committed at `now` with a trial pending is
    /// planned again from `now` against the rail the owner handed (`cc`),
    /// its state, sensitivity and step kept. A CC entry at an instant where
    /// a hub partition already stands left its trial as it was solved
    /// against the rail before the entry as a boundary, and a linear one is
    /// past no band, so it went on to commit that trial in CC: cold 347 tpl
    /// 5's load stepped 10 ns at the 3.08 V setpoint, 30 uC into 562 uF
    /// through a 332 mA limit. A re-seed instead changes the draw's rounding,
    /// which solves the rail again, and a rail capacitor of no esr entered
    /// at the setpoint leaves CC at once there (the rail is its charge):
    /// CC and CV in turn for the loop's 50 passes. Returns the partitions
    /// planned again.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn replan(&mut self, ids: &[usize], now: f64, catchup_drivers: &[Driver], drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64], readers: &mut Vec<DeviceId>) -> Result<Vec<usize>, String> {
        let mut planned = Vec::new();
        for &p in ids {
            if !self.partitions.get(&p).is_some_and(|part| !part.parked && part.has_pend && part.t == now) {
                continue;
            }
            self.journal(p);
            let part = self.partitions.get_mut(&p).unwrap();
            // The trial's `Step` event goes stale with it.
            part.has_pend = false;
            part.version += 1;
            self.plan_and_schedule(p, now, catchup_drivers, drivers, live_params, voltages, currents, readers)?;
            planned.push(p);
        }
        Ok(planned)
    }

    /// A CC exit at an internal event (the PSU left CC at a rail solve at
    /// `now` that an event other than this partition's own step brought
    /// about): each of `ids` still short of `now` whose pending trial was
    /// solved on a CC rail is brought to `now` on that rail, as a waveform
    /// edge brings a partition to it on the boundary it had (W1b rule 6).
    /// Brought there after the rail moved, it stepped the whole gap at the
    /// setpoint (F5pV: 3 ms at 5 V for a capacitor settled at 2 V on the CC
    /// rail, 1346 W0). Commits the steps and publishes nothing, and returns
    /// the partitions it moved; the re-solve or the demotion that follows
    /// starts from them.
    pub(crate) fn catch_up_on_cc_rail(&mut self, ids: &[usize], now: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<Vec<usize>, String> {
        let mut moved = Vec::new();
        for &p in ids {
            let Some(cc) = self.partitions.get(&p).filter(|part| !part.parked && part.has_pend && part.t < now).and_then(|part| part.pend_cc) else { continue };
            self.journal(p);
            self.cc_held = Some(cc);
            let result = self.sync_partition(p, now, drivers, live_params, voltages);
            self.cc_held = None;
            result?;
            moved.push(p);
        }
        Ok(moved)
    }

    /// The rail capacitors' charge the latest stepped commit handed the rail
    /// solve at `now` ([`Self::rail_charge`]), taken once; one handed at an
    /// earlier instant is stale and dropped.
    pub(crate) fn take_rail_charge(&mut self, now: f64) -> Option<f64> {
        self.rail_charge.take().filter(|&(t, _)| t == now).map(|(_, q)| q)
    }

    /// The committed state of partition `p`: its time and its own node
    /// voltages (exact mode reads the rail's partition through it).
    pub(crate) fn committed(&self, p: usize) -> Option<(f64, &[f64])> {
        self.partitions.get(&p).map(|part| (part.t, &part.x[..]))
    }

    /// Ruling R90 (a): the longest step partition `p` may take next, from its
    /// stepped stressed devices (`settle`'s trail close sets it at each
    /// commit, infinite once none is stressed).
    pub(crate) fn set_stress_reach(&mut self, p: usize, reach: f64) {
        if self.partitions.get(&p).is_some_and(|part| part.stress_reach.to_bits() != reach.to_bits()) {
            self.journal(p);
            self.partitions.get_mut(&p).unwrap().stress_reach = reach;
        }
    }

    /// Ruling R90 (a): partition `p`'s rated stepped devices still able to
    /// fail, which its planner reads each trial's stress ratios from.
    pub(crate) fn set_stress_probes(&mut self, p: usize, probes: Vec<StressProbe>) {
        if probes.is_empty() {
            self.stress_probes.remove(&p);
        } else {
            self.stress_probes.insert(p, probes);
        }
    }

    #[cfg(test)]
    pub(crate) fn debug_probe_list(&self, p: usize) -> Vec<StressProbe> {
        self.stress_probes.get(&p).cloned().unwrap_or_default()
    }

    /// Ruling R90 (a): whether partition `p` had a stressed device at its
    /// last commit (`settle`'s trail close sets it); only then is its next
    /// trial checked for a replan, so a board none of whose devices nears its
    /// rating pays nothing for the check.
    pub(crate) fn set_stress_watch(&mut self, p: usize, watch: bool) {
        if self.partitions.get(&p).is_some_and(|part| part.stress_watch != watch) {
            self.journal(p);
            self.partitions.get_mut(&p).unwrap().stress_watch = watch;
        }
    }


    /// Partition `p`'s state at the start of a step from `x0` (ruling R80
    /// (a)): its own nodes there, and its outside reads as the step before
    /// it ended (`Partition::end_reads`, which a re-seed at the start would
    /// have replaced with the seed's own start: that step keeps its end
    /// rate). Read as published when the step commits, a CC rail other
    /// partitions' solves had moved since the step began set 0.26 V across a
    /// 0.98 ohm fuse against the node the step began from (cold 474 tpl 5:
    /// 0.27 A where by hand it never carries 4.2 mA; tripped at 0.138 s); the
    /// trial's own `u0` is the rail it was planned from, a step behind (cold
    /// 535 tpl 2: 3.622 V where the last step had carried the rail to 3.632
    /// V).
    fn start_state(&self, p: usize, x0: &[f64]) -> Vec<(usize, f64)> {
        let part = &self.partitions[&p];
        let mut start: Vec<(usize, f64)> = part.shape.nodes.iter().copied().zip(x0.iter().copied()).collect();
        start.extend(part.end_reads.iter().copied());
        start
    }

    /// Partition `p`'s pending trial as a [`Committed`] would hold it: from
    /// its committed time to the trial's end, at the trial's own end
    /// voltages, its CC rail and its boundary at that end (ruling R71 (b): a
    /// latch predicted inside it is judged on the drive the trial ends with).
    pub(crate) fn pending_end(&self, p: usize, voltages: &[f64]) -> Option<Committed> {
        let part = self.partitions.get(&p).filter(|part| !part.parked && part.has_pend)?;
        let rail = match (part.hub, self.rail, part.pend_cc) {
            (true, Some(r), Some(_)) => Some((r, part.pend_u)),
            _ => None,
        };
        let mut nodes: Vec<(usize, f64)> = part.shape.nodes.iter().copied().zip(part.pend_x.iter().copied()).collect();
        let bnd = Bnd::new(voltages).at_time(self.ramps.as_deref(), part.pend_t);
        nodes.extend(self.trail_reads.get(p).into_iter().flatten().map(|&n| (n, bnd.get(n))));
        let start = part.has_prev.then(|| self.start_state(p, &part.x));
        Some(Committed { p, t0: part.t, t1: part.pend_t, nodes, rail, start })
    }

    /// Where the step partition `p` committed at `now` started: its last
    /// commit's own start, or `now` itself when it was re-solved there after
    /// that commit (a discontinuity: an event's re-solve or a promotion seeds
    /// it at `now` and drops its step, and what it publishes is no step's
    /// end). `None` when it is not at `now`.
    pub(crate) fn step_start(&self, p: usize, now: f64) -> Option<f64> {
        self.partitions.get(&p).filter(|part| !part.parked && part.t == now).map(|part| if part.has_prev { part.prev_t } else { now })
    }

    /// Ruling R59 (1) (B): parks or unparks partition `p` with no change to
    /// its state (the owner's load and its rollback of a switch).
    pub(crate) fn set_parked(&mut self, p: usize, parked: bool) {
        if let Some(part) = self.partitions.get_mut(&p) {
            part.parked = parked;
        }
    }

    /// Ruling R59 (1) (B): partition `p`'s PSU entered CC. It starts stepping
    /// at `now` from its capacitors' voltages `vc` (the closed form's charge
    /// there, so the switch moves no charge): a discontinuity, whose
    /// re-solve, which the owner runs next, seeds `x` from `vc` against the
    /// CC rail. Journaled as any change in a window.
    pub(crate) fn promote(&mut self, p: usize, now: f64, vc: &[f64]) {
        self.journal(p);
        let part = self.partitions.get_mut(&p).expect("a partition the stepped solver compiled");
        debug_assert_eq!(part.vc.len(), vc.len());
        part.parked = false;
        part.cc_switch = true;
        part.vc.copy_from_slice(vc);
        part.t = now;
        part.prev_t = now;
        part.has_prev = false;
        part.has_pend = false;
        part.be_next_step = true;
        part.trap_off = false;
        part.h = POST_EVENT_H;
        part.changing = true;
        // Its steps from the last time it stepped are stale.
        part.version += 1;
        part.sens = None;
        part.pend_lu = None;
        part.i_src = 0.0;
        part.pend_i_src = 0.0;
    }

    /// Ruling R59 (1) (B): partition `p`'s PSU left CC. It is brought to `now`
    /// on the boundary it has (as a waveform edge brings one, W1b rule 6),
    /// parked, and its capacitors' voltages there returned to seed the closed
    /// form with.
    pub(crate) fn demote(&mut self, p: usize, now: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<Vec<f64>, String> {
        self.catch_up(&[p], now, drivers, live_params, voltages)?;
        self.journal(p);
        // The draws its catch-up queued for the rail are a stepped copy's
        // the closed form now replaces: the owner drains this queue after the
        // closed form's, so one left here would stand as the partition's
        // contribution beside the closed form's own term.
        self.hub_updates.retain(|&(q, _)| q != p);
        let part = self.partitions.get_mut(&p).expect("a partition the stepped solver compiled");
        part.parked = true;
        Ok(part.vc.clone())
    }

    /// The sensitivity of hub partition `p` at state `x`, solved with the
    /// step `(h, method)` from `(vc_from, i_prev)` at rail `u_p` and instant
    /// `stamp_now`, through `lu` (README, Rails point 3), its rail ICs'
    /// power judged at `power_u` (`Bnd::hold_power_at`): the rail the solve
    /// it linearises judged them at.
    #[allow(clippy::too_many_arguments)]
    fn step_sens(
        &self,
        p: usize,
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &[f64],
        x: &[f64],
        vc_from: &[f64],
        i_prev: &[f64],
        h: f64,
        method: Method,
        stamp_now: f64,
        at: f64,
        u_p: f64,
        power_u: f64,
        lu: &LuFactorization,
    ) -> Result<StepSens, String> {
        let rail = self.rail.expect("a hub partition implies a rail");
        let part = &self.partitions[&p];
        let n = part.shape.n;
        let ramps = self.ramps.as_deref();
        let ic_ctx = IcStampCtx {
            now: stamp_now,
            drivers,
            ics: &self.ics,
            node_owner: &self.node_owner,
            bnd: Bnd::new(voltages).with_holds(if self.holds_on { &self.holds } else { &[] }),
            ramps,
            keep_lu: false,
            exact: None,
            cc: None,
            p_idx: p,
            iterated: part.iterated(),
            free_supply: part.holds_free_ic_supply,
        };
        let bnd = Bnd::new(voltages).with_holds(if self.holds_on { &self.holds } else { &[] }).at_time(ramps, at).with_ext(rail, n).hold_power_at(power_u);
        let z: Vec<f64> = if part.curved() { x.iter().copied().chain(std::iter::once(u_p)).collect() } else { Vec::new() };
        let ic = (part.curved() && !part.holds_free_ic_supply).then(|| {
            let ic = crate::solve::ic_rail_terms(stamp_now, drivers, &self.ics, &self.node_owner, &bnd, p, &z, n);
            let explicit = ic.explicit_u(&z, n);
            (ic, explicit)
        });
        let supply = part.holds_free_ic_supply.then(|| {
            let ic = crate::solve::ic_supply_terms(stamp_now, drivers, &self.ics, &self.node_owner, &bnd, p, &z);
            let extra = ic.extra_jacobian(&z, n);
            (ic, extra)
        });
        let sens = rail_sensitivity(n, x, u_p, lu, ic.as_ref().map(|(_, e)| &e[..]), supply.as_ref().map(|(_, e)| &e[..]), |z, a, b| stamp_step(&part.shape, live_params, h, vc_from, i_prev, method, &ic_ctx, &bnd, n + 1, z, a, b))
            .map_err(|e| format!("stepped partition {p}: {e}"))?;
        // A stepped solve is not polished to the solution (a known
        // follow-up of R20), so no rounding floor is formed for it.
        let floors = crate::solve::curvature::Floors::default();
        let band = match (ic, supply) {
            _ if !part.curved() => Some(rails::RAIL_RESOLVE_V),
            (Some((ic, _)), _) => crate::solve::curvature::hub_band(&part.shape, live_params, &bnd, x, u_p, &sens, &ic, floors),
            (None, Some((ic, _))) => crate::solve::curvature::supply_band(&part.shape, live_params, &bnd, x, u_p, &sens, &ic, floors),
            (None, None) => None,
        };
        Ok(StepSens { s: sens.s, i: sens.i, g: sens.g, u_p, band })
    }

    /// Commits partition `p`'s pending trial, first taking a hub partition's
    /// sensitivity at it (a stepped hub partition contributes afresh at each
    /// committed step, Rails point 4).
    fn commit_partition(&mut self, p: usize, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<(), String> {
        let part = &self.partitions[&p];
        if part.hub && self.rail.is_some() {
            if let Some(lu) = part.pend_lu.clone() {
                let (x, vc_from, i_prev) = (part.pend_x.clone(), part.vc.clone(), part.i_prev.clone());
                let sens = self.step_sens(p, drivers, live_params, voltages, &x, &vc_from, &i_prev, part.pend_h, part.pend_method, part.pend_now, part.pend_t, part.pend_u, part.pend_u, &lu)?;
                self.sensitivities += 1;
                self.hub_updates.push((p, Contrib { active: true, i: sens.i, g: sens.g, u_p: sens.u_p, b: None }));
                self.partitions.get_mut(&p).unwrap().sens = Some(Rc::new(sens));
            }
        }
        let charge = self.partitions[&p].pend_charge;
        if self.defer == Some(p) {
            self.bills.push((self.partitions[&p].pend_t, charge));
        }
        let reseeded = !self.partitions[&p].has_prev;
        // The charge copies the trial's knots; the observer, last, takes
        // them (the next plan starts afresh).
        if self.charge_on.get(p).copied().unwrap_or(false) {
            let step = self.charge_step(p, voltages);
            self.charge_steps.push(step);
        }
        if self.dense_on.get(p).copied().unwrap_or(false) {
            let seg = self.dense_seg_taken(p);
            self.dense.push(seg);
        }
        Self::commit(self.partitions.get_mut(&p).unwrap());
        if let Some((t, q, _)) = self.partitions[&p].rail_q {
            self.rail_charge = Some((t, q));
        }
        if self.trail_parts.get(p).copied().unwrap_or(false) {
            let part = &self.partitions[&p];
            // On a CC rail the step's own rail; otherwise the rail as the
            // commit found it, before an event at its end moves it (cold 705
            // tpl 4: read as published after a pulse's edge, 6.5 V across an
            // LED from the rail to where its step ended latched it at 37 ms
            // on a 40 mA PSU that cannot stress it).
            let rail = match (part.hub, self.rail, part.pend_cc) {
                (true, Some(r), Some(_)) => Some((r, part.pend_u)),
                _ => None,
            };
            let mut nodes: Vec<(usize, f64)> = part.shape.nodes.iter().copied().zip(part.x.iter().copied()).collect();
            // A running ramp's node at the step's end, as the step read it
            // (cold 798 tpl 4: the fuse from a falling pulse's node read the
            // ramp where the last publish left it, 7.6 V from where the step
            // ended, and blew on 34 mA).
            let bnd = Bnd::new(voltages).at_time(self.ramps.as_deref(), part.t);
            nodes.extend(self.trail_reads[p].iter().map(|&n| (n, bnd.get(n))));
            let start = (!reseeded).then(|| self.start_state(p, &part.prev_x));
            let (t0, t1, own) = (part.prev_t, part.t, part.shape.nodes.len().min(nodes.len()));
            // What the next step starts from outside the partition: this
            // step's end, its CC rail over the boundary's read of it.
            let end_reads: Vec<(usize, f64)> = nodes[own..].iter().copied().chain(rail).collect();
            self.partitions.get_mut(&p).unwrap().end_reads = end_reads;
            self.trail.push(Committed { p, t0, t1, nodes, rail, start });
        }
        #[cfg(test)]
        self.commit_log.push((p, self.partitions[&p].t));
        #[cfg(test)]
        if let Some(stamp) = self.exact.filter(|e| e.partition == p) {
            let part = &self.partitions[&p];
            let caps = |i: &[f64]| part.shape.capacitors.iter().zip(i).map(|(c, &i)| (c.device_index, i)).collect();
            let mid = part.pend_mid.as_ref().map(|(t, x, i)| (*t, x.clone(), caps(i)));
            self.exact_commit_log.push(ExactCommit { t0: part.prev_t, mid, t: part.t, x: part.x.clone(), caps: caps(&part.i_prev), stamp, trap: part.pend_method == Method::Trap });
        }
        // Rails point 9: the step after this one, if this call plans it,
        // takes E and r from the SoC this one leaves.
        if let (Some((model, state)), Some(stamp)) = (self.cell.as_mut(), self.exact.as_mut().filter(|e| e.partition == p)) {
            *state = model.advance(state, charge).state;
            stamp.kind = ExactKind::battery(model, state.soc);
        }
        Ok(())
    }

    pub fn is_empty(&self) -> bool {
        self.partitions.is_empty()
    }

    pub fn watch_node(&mut self, node: usize, reader: DeviceId, thresholds: ThresholdFn) -> bool {
        let Some(&(p_idx, local)) = self.node_owner.get(&node) else {
            return false;
        };
        let part = self.partitions.get_mut(&p_idx).expect("node_owner and partitions built together");
        part.watches.entry(local).or_default().push((reader, Rc::from(thresholds)));
        true
    }

    /// Records that the watch on `node` reads `supply_node` for its
    /// thresholds (`solve::register_ic_read_pin_watches`). Only a supply in
    /// ANOTHER Stepped partition matters: one in the same partition comes out
    /// of every trial step's own solution, and a fixed or Static one moves
    /// only at events, which re-aim the watch (`add_watch_dependent_partitions`).
    pub fn add_supply_dependency(&mut self, node: usize, supply_node: usize) {
        let (Some(&(watcher, _)), Some(&(supplier, _))) = (self.node_owner.get(&node), self.node_owner.get(&supply_node)) else {
            return;
        };
        if watcher == supplier {
            return;
        }
        let part = self.partitions.get_mut(&watcher).expect("node_owner and partitions built together");
        if !part.supply_deps.contains(&supplier) {
            part.supply_deps.push(supplier);
        }
    }

    pub fn steps_taken(&self) -> f64 {
        self.partitions.values().map(|p| p.steps_taken as f64).sum()
    }

    /// INT-ACC (R81): fallbacks to backward Euler taken so far, every
    /// partition's (not rolled back, as `steps_taken` is not).
    /// R79 (a): whether any partition here may step by the trapezoidal
    /// rule, so has no maximum principle (`settle::voltage_span_bound`):
    /// every one under `INTEGRATION` trap, else a linear one off the hub
    /// (`plan_step`'s `trap_run`, off the rail; read without the CC rail's `rail_ic`,
    /// which only takes a partition off it).
    pub(super) fn may_step_trap(&self, option: Method) -> bool {
        !self.partitions.is_empty() && (option == Method::Trap || self.partitions.values().any(|p| !p.iterated() && !p.on_rail))
    }

    pub fn trap_fallbacks(&self) -> f64 {
        self.partitions.values().map(|p| p.trap_fallbacks as f64).sum()
    }

    pub fn newton_iterations(&self) -> f64 {
        self.partitions.values().map(|p| p.newton_iterations as f64).sum()
    }

    pub fn is_current(&self, partition: usize, version: u64) -> bool {
        self.partitions.get(&partition).is_some_and(|p| p.version == version && !p.parked)
    }

    /// Whether `partition`'s NEXT planned step is forced to BE by the
    /// discontinuity anchor (spec §7.2.3) — test-only introspection, since
    /// nothing outside this module needs to know the method a specific step
    /// used, only that its NUMBERS came out right.
    /// Whether `partition` has a pending trial (test-only introspection).
    #[cfg(test)]
    /// The rail charge handed to the rail solve, and partition `p`'s own
    /// last committed one (ruling R68).
    #[cfg(test)]
    pub(crate) fn debug_rail_charge(&self, p: usize) -> (Option<(f64, f64)>, Option<(f64, f64, f64)>) {
        (self.rail_charge, self.partitions.get(&p).and_then(|part| part.rail_q))
    }

    pub(crate) fn debug_has_pend(&self, partition: usize) -> bool {
        self.partitions.get(&partition).is_some_and(|p| p.has_pend)
    }

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
        for p_idx in self.suppliers_first(dirty) {
            if !self.partitions.get(&p_idx).is_some_and(|part| !part.parked) {
                continue;
            }
            self.journal(p_idx);
            // On a CC rail with capacitors (ruling R68) the catch-up steps on
            // the rail its pending trial was solved on, as a CC exit's does
            // (`catch_up_on_cc_rail`): the rail is the capacitors' charge,
            // which those steps carry, and judged at the rail published now
            // they took a rail IC's power from after the change that brought
            // the re-solve. Cold 216 tpl 5: a 74LS00 powered over the step
            // was judged off at the falling edge that ended it, the step
            // carried the rail back over the edge, and the chip flipped at
            // every instant, 1e-14 s apart, never alternating within one.
            let held = self.partitions.get(&p_idx).filter(|part| part.has_pend && part.t < now).and_then(|part| part.pend_cc).filter(|cc| cc.cap.is_some());
            self.cc_held = held;
            let synced = self.sync_partition(p_idx, now, catchup_drivers, live_params, voltages);
            self.cc_held = None;
            synced?;

            let old_bits: Vec<u64> = self.partitions[&p_idx].shape.nodes.iter().map(|&n| voltages[n].to_bits()).collect();
            let hub = self.partitions[&p_idx].hub && self.rail.is_some();
            // On a CC rail the seed solves the rail with the partition, as
            // its steps do (`take_cc_step`; README, Rails point 4, R28): held
            // at the rail the loop last solved, a seed whose stamps curve in
            // the rail (cold fuzz 456: a gate's output on it, 13 ohm and an
            // LED off it) re-solved to the same three rails in turn.
            let cc = match (&self.cc, hub, self.exact) {
                (Some(cc), true, None) => {
                    let part = &self.partitions[&p_idx];
                    Some(CcStep { seed: true, ..cc.step(p_idx, voltages[cc.rail], now, part.rail_q, self.rail_charge) })
                }
                _ => None,
            };
            let ic_ctx = IcStampCtx {
                now,
                drivers,
                ics: &self.ics,
                node_owner: &self.node_owner,
                bnd: Bnd::new(voltages).with_holds(if self.holds_on { &self.holds } else { &[] }),
                ramps: self.ramps.as_deref(),
                keep_lu: hub,
                exact: self.exact,
                cc,
                p_idx,
                iterated: self.partitions[&p_idx].iterated_on(cc.is_some()),
                free_supply: self.partitions[&p_idx].holds_free_ic_supply,
            };
            let seed_sens;
            let seed_u;
            let taken;
            {
                let part = self.partitions.get_mut(&p_idx).unwrap();
                part.be_next_step = true; // a discontinuity: force BE for the next trio.
                part.trap_off = false;
                part.has_prev = false;
                // A re-aim keeps its step (`reaim`); a discontinuity restarts.
                let kept = self.reaim.contains(&p_idx);
                if !kept {
                    part.h = POST_EVENT_H;
                }
                #[cfg(test)]
                self.resolve_log.push(ResolveRecord { partition: p_idx, now, u: self.rail.map_or(0.0, |r| voltages[r]), kept, cc: self.cc.is_some() });
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
                let (seed, holds) = seed_holding(&part.shape, live_params, &part.vc, &part.x, &ic_ctx)?;
                taken = holds;
                // `i_src` stays as it is: the step after a seed is backward
                // Euler, whose bill reads only its end current (`plan_step`).
                part.x = seed.x;
                part.i_prev = seed.i;
                if self.dense_on.get(p_idx).copied().unwrap_or(false) {
                    // A seed is the partition's state from its instant on
                    // (a discontinuity: right-continuous, D2).
                    self.dense.push(DenseSeg { p: p_idx, t0: part.t, x0: part.x.clone(), vc0: part.vc.clone(), i0: part.i_prev.clone(), knots: Vec::new(), solved: Vec::new(), steady: false, affine: false, sens: None });
                }
                // A seed holds the rail capacitors' charge where it stands
                // (ruling R68): the next step starts from it. Read from the
                // segment at a later step's start instead, a step from before
                // the segment's aim took a charge from after it (the cold
                // fuzz's 517 tpl 5 at load: 0.025 V against 0, a step of 20 fs
                // carried the rail to 0.65 V and the loop did not settle).
                if let Some(c) = cc.and_then(|c| c.cap) {
                    part.rail_q = Some((now, c.q, c.i));
                }
                seed_sens = seed.lu;
                seed_u = seed.u;
            }
            // A hub partition's seed is a solve too: its sensitivity at the
            // seed's own system (Rails point 3), at the rail it solved on a
            // CC rail. Its draw is its rail row's on a CC rail, else the
            // charge balance ([`ChargeBalance`]): a magnified draw misjudged
            // the CV -> CC test too
            // (G2bv's bc 453 hub 2 at 17.8 ms, 2.8e-7 A off, CC and CV in
            // turn for 50 passes).
            if let (Some(lu), Some(rail)) = (seed_sens, self.rail.filter(|_| hub)) {
                let part = &self.partitions[&p_idx];
                let (x, vc_from) = (part.x.clone(), part.vc.clone());
                let zeros = vec![0.0; vc_from.len()];
                let u = seed_u.unwrap_or(voltages[rail]);
                // Its rail ICs' power as the seed's solve judged it, at the
                // loop's rail (`take_cc_step`): judged at its own rail, a
                // gate powered in the solve and off at `u` (G2bv's bc_ic 178
                // hub 2: 1.69 V powered, 4.94 V not) published a draw the
                // solve never met, the loop solved its old rail again, no
                // verdict flipped, and the seed re-solved to the same rail
                // for 50 passes. Judged as solved, the draw meets the rail
                // row at `u`, the loop moves the rail there, and the flip
                // re-seeds it as any other (H8).
                let mut sens = self.step_sens(p_idx, drivers, live_params, voltages, &x, &vc_from, &zeros, SEED_H, Method::Be, now, now, u, voltages[rail], &lu)?;
                let bnd = ic_ctx.bnd.at_time(ic_ctx.ramps, now).with_ext(rail, x.len()).hold_power_at(voltages[rail]);
                // With rail capacitors (ruling R68) the row also holds their
                // current at the seed, `(u - q) / esr`; at `esr = 0` the row
                // is the charge alone and the draw is read as off a CC rail.
                let seed_cap = ic_ctx.cc.and_then(|cc| cc.cap).map(|c| c.companion(None, Method::Be).map(|(g, q)| g * (u - q)));
                if let (Some(cc), Some(_), None | Some(Some(_))) = (ic_ctx.cc, seed_u, seed_cap) {
                    // On a CC rail, the draw its rail row held at `u`: the
                    // loop solves the rail from it and the rest's draw, and
                    // so finds `u` again. Read from the stamp instead, its
                    // Newton's last rounding comes in through `C / SEED_H`
                    // where a capacitor path runs from the rail to a fixed
                    // node, which no charge balance reaches (cold fuzz 603
                    // tpl 3: 1.33 mF from the rail to a node 5.5 uF holds,
                    // 5.2e-7 A over the row's draw, the loop's rail 1.4 uV
                    // under the seed's for 50 passes).
                    sens.i = cc.dl - cc.a_o - cc.g_o * u - seed_cap.flatten().unwrap_or(0.0);
                } else if let Some((caps, nodes)) = rail_capacitors(&part.shape, &bnd) {
                    let z: Vec<f64> = x.iter().copied().chain(std::iter::once(u)).collect();
                    sens.i = ChargeBalance { caps, nodes }.draw(&part.shape, live_params, SEED_H, &vc_from, &ic_ctx, &bnd, &z);
                }
                self.sensitivities += 1;
                self.hub_updates.push((p_idx, Contrib { active: true, i: sens.i, g: sens.g, u_p: u, b: None }));
                self.partitions.get_mut(&p_idx).unwrap().sens = Some(Rc::new(sens));
            }
            if !taken.is_empty() {
                // Held in this copy at once, so the plan from the seed steps
                // with them; the owner enters them in its table after the call.
                let table = Rc::make_mut(&mut self.holds);
                for &(d, hold) in &taken {
                    if table.len() <= d {
                        table.resize(d + 1, digital::power::PowerHold::FREE);
                    }
                    table[d] = hold;
                    readers.push(DeviceId(d as u32));
                }
                self.holds_on = true;
                self.hold_requests.extend(taken.iter().map(|&(d, hold)| (d, hold, now)));
            }
            Self::publish(self.partitions.get(&p_idx).unwrap(), self.rail, live_params, voltages, currents);
            readers.extend(self.partitions[&p_idx].watches.values().flatten().map(|(id, _)| *id));
            Self::report_changed_ic_readers(&self.partitions[&p_idx], &old_bits, voltages, &self.node_to_ic_readers, &mut readers);
            self.plan_and_schedule(p_idx, now, catchup_drivers, drivers, live_params, voltages, currents, &mut readers)?;
        }
        Ok(readers)
    }

    /// `dirty` reordered so a partition that supplies another dirty
    /// partition's watch is re-solved first: the dependent's plan then reads
    /// the supply's fresh trial and not the one the discontinuity is about to
    /// discard (`sync_supplies`). Stable otherwise, and a supply cycle simply
    /// keeps the caller's order.
    fn suppliers_first(&self, dirty: &[usize]) -> Vec<usize> {
        let supplies_a_dirty_partition =
            |p: usize| dirty.iter().any(|&q| q != p && self.partitions.get(&q).is_some_and(|part| part.supply_deps.contains(&p)));
        let (first, rest): (Vec<usize>, Vec<usize>) = dirty.iter().copied().partition(|&p| supplies_a_dirty_partition(p));
        first.into_iter().chain(rest).collect()
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
        self.journal(partition);
        let old_bits: Vec<u64> = self.partitions[&partition].shape.nodes.iter().map(|&n| voltages[n].to_bits()).collect();
        if !self.partitions.contains_key(&partition) {
            return Err(format!("stepped: unknown partition {partition}"));
        }
        self.commit_partition(partition, drivers, live_params, voltages)?;
        Self::publish(self.partitions.get(&partition).unwrap(), self.rail, live_params, voltages, currents);
        let mut readers: Vec<DeviceId> = self.partitions[&partition].watches.values().flatten().map(|(id, _)| *id).collect();
        Self::report_changed_ic_readers(&self.partitions[&partition], &old_bits, voltages, &self.node_to_ic_readers, &mut readers);
        let commit_time = self.partitions[&partition].t;
        #[cfg(test)]
        if let Some(reach) = FORCE_REACH.with(|f| f.get()).filter(|&(p, t, _)| p == partition && t == commit_time).map(|f| f.2) {
            self.partitions.get_mut(&partition).unwrap().stress_reach = reach;
        }
        self.plan_and_schedule(partition, commit_time, drivers, drivers, live_params, voltages, currents, &mut readers)?;
        Ok(readers)
    }

    /// The horizon clamp's stepped half (`PartitionedSolver::sync_dynamic_to`):
    /// fast-forwards every partition still behind `t_end`, publishes it, and
    /// re-plans from there so the next `advance` has a live pending step.
    pub fn sync_to(&mut self, t_end: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) -> Result<Vec<DeviceId>, String> {
        let mut ids: Vec<usize> = self.partitions.iter().filter(|(_, part)| !part.parked).map(|(&p, _)| p).collect();
        // A map's order is arbitrary, and a partition's plan now reads the
        // ones that supply it (`sync_supplies`): fix the order so a run does
        // not depend on it.
        ids.sort_unstable();
        let mut readers = Vec::new();
        for p_idx in ids {
            if self.partitions[&p_idx].t >= t_end - 1e-15 {
                continue;
            }
            self.journal(p_idx);
            let old_bits: Vec<u64> = self.partitions[&p_idx].shape.nodes.iter().map(|&n| voltages[n].to_bits()).collect();
            self.sync_partition(p_idx, t_end, drivers, live_params, voltages)?;
            Self::publish(self.partitions.get(&p_idx).unwrap(), self.rail, live_params, voltages, currents);
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
            self.plan_and_schedule(p_idx, t_end, drivers, drivers, live_params, voltages, currents, &mut readers)?;
        }
        Ok(readers)
    }

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
        self.partitions.get(&p_idx).map(|part| &part.shape)
    }

    /// W1g (README W1g point 17): the one setter for partition `p_idx`'s
    /// junctions, each with its law and only the stamped (not failed) ones,
    /// in the order the shape was compiled in. A stepped partition owns its
    /// `DynShape`, so this is how a held temperature after a committed step
    /// or a failed LED reaches its stamps. It plans nothing: the next trial
    /// stamps the new list, which is classic's one-step lag for `T_hold`
    /// (point 4), and a latch also marks the partition dirty, whose re-seed
    /// picks it up.
    pub(crate) fn set_junctions(&mut self, p_idx: usize, junctions: Vec<crate::solve::DynJunction>) {
        if let Some(part) = self.partitions.get_mut(&p_idx) {
            part.shape.junctions = junctions;
        }
    }

    /// W2b piece 4: the instant partition `p_idx`'s last commit reached,
    /// how far its charge is final.
    pub(crate) fn committed_time(&self, p_idx: usize) -> Option<f64> {
        self.partitions.get(&p_idx).map(|part| part.t)
    }

    /// The instant partition `p_idx` has been committed to (tests look at it).
    #[cfg(test)]
    pub(crate) fn time_of(&self, p_idx: usize) -> Option<f64> {
        self.partitions.get(&p_idx).map(|part| part.t)
    }

    /// Opens the rollback window (see the field): nothing is copied yet.
    pub(crate) fn begin_window(&mut self) {
        self.window = true;
        self.w_dense = self.dense.len();
        self.w_charge = self.charge_steps.len();
        self.window_scheduled.clone_from(&self.scheduled);
        self.window_rail_charge = self.rail_charge;
        #[cfg(test)]
        {
            self.w_saved = Some(self.partitions.iter().map(|(&p, part)| (p, Self::state_digest(part))).collect());
        }
    }

    #[cfg(test)]
    fn state_digest(part: &Partition) -> String {
        let mut snap = PartitionSnapshot::default();
        snap.fill_from(part);
        format!("{snap:?}")
    }

    /// Saves partition `p`'s state before the window's first call that can
    /// change it. A no-op outside a window and for a partition already saved:
    /// the first pre-image is the one a rollback needs. Every entry point
    /// that changes a partition calls this first (`resolve_dirty`,
    /// `handle_step`, `sync_to`, `sync_supplies`); `plan_step`, `sync_partition`
    /// and `commit` only ever run on a partition one of them named.
    fn journal(&mut self, p: usize) {
        if !self.window {
            return;
        }
        let Some(part) = self.partitions.get_mut(&p) else { return };
        if part.journaled {
            return;
        }
        part.journaled = true;
        let mut saved = self.spare.pop().unwrap_or_default();
        saved.fill_from(part);
        self.journal.push((p, saved));
    }

    /// The window's instant completed: the journal is dropped (its buffers
    /// kept for the next window).
    pub(crate) fn end_window(&mut self) {
        debug_assert!(self.deferred.is_empty(), "a held-back plan outlived its call: {:?}", self.deferred);
        debug_assert!(self.bills.is_empty(), "a committed battery step was never charged: {:?}", self.bills);
        #[cfg(test)]
        if let Some(saved) = self.w_saved.take() {
            for (p, part) in &self.partitions {
                assert!(part.journaled || Self::state_digest(part) == saved[p], "stepped partition {p} changed in the window without being journaled");
            }
        }
        self.window = false;
        for (p, saved) in self.journal.drain(..) {
            if let Some(part) = self.partitions.get_mut(&p) {
                part.journaled = false;
            }
            self.spare.push(saved);
        }
    }

    /// The window's instant failed: every partition it touched, and the queue
    /// of steps to schedule, go back to what the window found.
    pub(crate) fn rollback_window(&mut self) {
        self.dense.truncate(self.w_dense);
        self.charge_steps.truncate(self.w_charge);
        self.trail.clear();
        self.scheduled.clone_from(&self.window_scheduled);
        self.rail_charge = self.window_rail_charge;
        self.deferred.clear();
        self.bills.clear();
        self.window = false;
        for (p, mut saved) in self.journal.drain(..) {
            if let Some(part) = self.partitions.get_mut(&p) {
                part.journaled = false;
                // Swapped, not moved: the buffers the partition held go to the
                // spare snapshot instead of to the allocator.
                std::mem::swap(&mut part.x, &mut saved.x);
                std::mem::swap(&mut part.vc, &mut saved.vc);
                std::mem::swap(&mut part.i_prev, &mut saved.i_prev);
                part.t = saved.t;
                part.h = saved.h;
                part.changing = saved.changing;
                part.be_next_step = saved.be_next_step;
                part.trap_off = saved.trap_off;
                part.pend_trap_off = saved.pend_trap_off;
                part.version = saved.version;
                part.has_pend = saved.has_pend;
                part.pend_t = saved.pend_t;
                std::mem::swap(&mut part.pend_x, &mut saved.pend_x);
                std::mem::swap(&mut part.pend_vc, &mut saved.pend_vc);
                std::mem::swap(&mut part.pend_i, &mut saved.pend_i);
                part.prev_t = saved.prev_t;
                std::mem::swap(&mut part.prev_x, &mut saved.prev_x);
                part.has_prev = saved.has_prev;
                std::mem::swap(&mut part.end_reads, &mut saved.end_reads);
                part.stress_reach = saved.stress_reach;
                part.stress_watch = saved.stress_watch;
                saved.restore_w1f(part);
            }
            self.spare.push(saved);
        }
        self.hub_updates.clear();
        #[cfg(test)]
        if let Some(saved) = self.w_saved.take() {
            for (p, part) in &self.partitions {
                assert_eq!(Self::state_digest(part), saved[p], "stepped partition {p}: a rollback left it changed");
            }
        }
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
                            trap_off: part.trap_off,
                            pend_trap_off: part.pend_trap_off,
                            version: part.version,
                            has_pend: part.has_pend,
                            pend_t: part.pend_t,
                            pend_x: part.pend_x.clone(),
                            pend_vc: part.pend_vc.clone(),
                            pend_i: part.pend_i.clone(),
                            prev_t: part.prev_t,
                            prev_x: part.prev_x.clone(),
                            has_prev: part.has_prev,
                            end_reads: part.end_reads.clone(),
                            stress_reach: part.stress_reach,
                            stress_watch: part.stress_watch,
                            sens: part.sens.clone(),
                            pend_lu: part.pend_lu.clone(),
                            pend_u: part.pend_u,
                            pend_now: part.pend_now,
                            pend_h: part.pend_h,
                            pend_method: Some(part.pend_method),
                            pend_cc: part.pend_cc,
                            pend_charge: part.pend_charge,
                            i_src: part.i_src,
                            pend_i_src: part.pend_i_src,
                            pend_rail_q: part.pend_rail_q,
                            rail_q: part.rail_q,
                            pend_dense: part.pend_dense.clone(),
                            #[cfg(test)]
                            pend_mid: part.pend_mid.clone(),
                        },
                    )
                })
                .collect(),
            scheduled: self.scheduled.clone(),
            rail_charge: self.rail_charge,
        }
    }

    pub fn restore(&mut self, snapshot: Snapshot) {
        self.trail.clear();
        self.scheduled = snapshot.scheduled;
        self.rail_charge = snapshot.rail_charge;
        self.deferred.clear();
        self.bills.clear();
        for (p, s) in snapshot.partitions {
            if let Some(part) = self.partitions.get_mut(&p) {
                part.x = s.x;
                part.vc = s.vc;
                part.i_prev = s.i_prev;
                part.t = s.t;
                part.h = s.h;
                part.changing = s.changing;
                part.be_next_step = s.be_next_step;
                part.trap_off = s.trap_off;
                part.pend_trap_off = s.pend_trap_off;
                part.version = s.version;
                part.has_pend = s.has_pend;
                part.pend_t = s.pend_t;
                part.pend_x = s.pend_x;
                part.pend_vc = s.pend_vc;
                part.pend_i = s.pend_i;
                part.prev_t = s.prev_t;
                part.prev_x = s.prev_x;
                part.has_prev = s.has_prev;
                part.end_reads = s.end_reads;
                part.stress_reach = s.stress_reach;
                part.stress_watch = s.stress_watch;
                part.sens = s.sens;
                part.pend_lu = s.pend_lu;
                part.pend_u = s.pend_u;
                part.pend_now = s.pend_now;
                part.pend_h = s.pend_h;
                part.pend_method = s.pend_method.unwrap_or(Method::Be);
                part.pend_cc = s.pend_cc;
                part.pend_charge = s.pend_charge;
                part.i_src = s.i_src;
                part.pend_i_src = s.pend_i_src;
                part.pend_rail_q = s.pend_rail_q;
                part.rail_q = s.rail_q;
                part.pend_dense = s.pend_dense;
                #[cfg(test)]
                {
                    part.pend_mid = s.pend_mid;
                }
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
    fn publish(part: &Partition, rail: Option<usize>, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) {
        match (&part.sens, rail) {
            // A hub partition publishes its rail-corrected voltage (Rails
            // point 3).
            (Some(sens), Some(rail)) if part.hub => {
                let du = voltages[rail] - sens.u_p;
                for (l, &node) in part.shape.nodes.iter().enumerate() {
                    voltages[node] = part.x[l] + sens.s[l] * du;
                }
            }
            _ => {
                for (l, &node) in part.shape.nodes.iter().enumerate() {
                    voltages[node] = part.x[l];
                }
            }
        }
        for (idx, cap) in part.shape.capacitors.iter().enumerate() {
            let (va, vb) = (crate::solve::pin_voltage_global(&cap.pins[0], voltages), crate::solve::pin_voltage_global(&cap.pins[1], voltages));
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
        part.prev_t = part.t;
        part.has_prev = true;
        part.t = part.pend_t;
        part.prev_x = std::mem::replace(&mut part.x, std::mem::take(&mut part.pend_x));
        part.vc = std::mem::take(&mut part.pend_vc);
        part.i_prev = std::mem::take(&mut part.pend_i);
        part.i_src = part.pend_i_src;
        part.rail_q = part.pend_rail_q.map(|(q, i)| (part.t, q, i));
        part.be_next_step = false;
        if part.pend_trap_off && !part.trap_off {
            part.trap_fallbacks += 1;
        }
        part.trap_off |= part.pend_trap_off;
        part.steps_taken += 1;
    }

    /// pec3's `_syncDynamic` (`:694-703`), generalized for BE/trap: takes
    /// however many committed steps (each bounded to `horizon`) it takes to
    /// bring `part.t` up to `horizon`, using a pending trial outright when it
    /// already lands within `horizon`.
    fn sync_partition(&mut self, p_idx: usize, horizon: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<(), String> {
        self.clamp = true;
        let result = self.sync_partition_steps(p_idx, horizon, drivers, live_params, voltages);
        self.clamp = false;
        if let Some(part) = self.partitions.get_mut(&p_idx) {
            part.cc_exited = false;
        }
        result
    }

    fn sync_partition_steps(&mut self, p_idx: usize, horizon: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64]) -> Result<(), String> {
        loop {
            let (t, has_pend, pend_t) = {
                let part = &self.partitions[&p_idx];
                (part.t, part.has_pend, part.pend_t)
            };
            if t >= horizon - 1e-15 {
                break;
            }
            if has_pend && pend_t <= horizon + 1e-15 {
                self.commit_partition(p_idx, drivers, live_params, voltages)?;
                continue;
            }
            self.plan_step(p_idx, horizon, drivers, live_params, voltages)?;
            self.commit_partition(p_idx, drivers, live_params, voltages)?;
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn plan_and_schedule(
        &mut self,
        p_idx: usize,
        now: f64,
        catchup_drivers: &[Driver],
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
        readers: &mut Vec<DeviceId>,
    ) -> Result<(), String> {
        let mut visiting = vec![p_idx];
        self.plan_and_schedule_in(p_idx, now, catchup_drivers, drivers, live_params, voltages, currents, readers, &mut visiting)
    }

    #[allow(clippy::too_many_arguments)]
    fn plan_and_schedule_in(
        &mut self,
        p_idx: usize,
        now: f64,
        catchup_drivers: &[Driver],
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
        readers: &mut Vec<DeviceId>,
        visiting: &mut Vec<usize>,
    ) -> Result<(), String> {
        // Exact mode (Rails point 9): the battery partition's plan waits for
        // the SoC charge of the step just committed, and so does every plan
        // that would read its trial, which comes only then
        // (`plan_deferred`). It is held before its supplies are synced, so a
        // supply that reads it in turn (a cycle) is held behind it. The
        // supplies are brought to `now` here all the same: their catch-up
        // must read `catchup_drivers`, the table in force before the event,
        // which only this call has.
        let held = self.defer.is_some() && (self.defer == Some(p_idx) || self.deferred.contains(&p_idx));
        if held {
            self.hold(p_idx);
        }
        self.sync_supplies(p_idx, now, catchup_drivers, drivers, live_params, voltages, currents, readers, visiting)?;
        if held || (self.defer.is_some() && self.partitions[&p_idx].supply_deps.iter().any(|b| self.deferred.contains(b))) {
            self.hold(p_idx);
            return Ok(());
        }
        self.plan_step(p_idx, f64::INFINITY, drivers, live_params, voltages)?;
        let part = &self.partitions[&p_idx];
        self.scheduled.push((part.pend_t, p_idx, part.version));
        Ok(())
    }

    /// Brings every OTHER Stepped partition that supplies one of `p_idx`'s
    /// watched ICs (`supply_deps`) up to `now`, publishes it, and plans it
    /// afresh, before `p_idx` itself is planned or its readers are read.
    ///
    /// Stepped partitions advance independently, each on its own `Step`
    /// events, so at `now` a supply's published voltage is its last commit,
    /// which can be a whole step old. Two things need it current. The plan:
    /// it finds a crossing against the supply's value at the crossing
    /// instant, which it takes from the supply's pending trial
    /// ([`SupplyTrace`]), so that trial must start at `now` and reach past
    /// it. The read: the IC evaluated at this event compares the pin with
    /// thresholds derived from the published supply, and a stale one can
    /// leave a pin the plan landed 1e-9 V past its threshold on the wrong
    /// side of the one the IC actually reads. The supply's own watchers and
    /// the readers of the nodes it moved are added to `readers`, as any
    /// committed step's are. `visiting` stops a cycle (an IC whose supply
    /// sits in the partition of a pin that another IC's supply depends on)
    /// from recursing; the innermost partition then plans against the
    /// trial it already has.
    #[allow(clippy::too_many_arguments)]
    fn sync_supplies(
        &mut self,
        p_idx: usize,
        now: f64,
        catchup_drivers: &[Driver],
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
        readers: &mut Vec<DeviceId>,
        visiting: &mut Vec<usize>,
    ) -> Result<(), String> {
        let deps = self.partitions[&p_idx].supply_deps.clone();
        for b in deps {
            if visiting.contains(&b) {
                continue;
            }
            let part = &self.partitions[&b];
            if part.t >= now - 1e-15 && part.has_pend && part.pend_t > now + 1e-15 {
                continue;
            }
            // Held back in this call, so already at `now` and published; its
            // trial comes after the SoC charge (`plan_and_schedule_in`).
            if self.deferred.contains(&b) {
                debug_assert!(part.t >= now - 1e-15, "a held plan behind its instant");
                continue;
            }
            let old_bits: Vec<u64> = part.shape.nodes.iter().map(|&n| voltages[n].to_bits()).collect();
            self.journal(b);
            visiting.push(b);
            let result = (|| {
                self.sync_partition(b, now, catchup_drivers, live_params, voltages)?;
                Self::publish(&self.partitions[&b], self.rail, live_params, voltages, currents);
                readers.extend(self.partitions[&b].watches.values().flatten().map(|(id, _)| *id));
                Self::report_changed_ic_readers(&self.partitions[&b], &old_bits, voltages, &self.node_to_ic_readers, readers);
                self.plan_and_schedule_in(b, now, catchup_drivers, drivers, live_params, voltages, currents, readers, visiting)
            })();
            visiting.pop();
            result?;
        }
        Ok(())
    }

    /// The Stepped partitions in `p_idx`'s `supply_deps` as trajectories over
    /// their pending trials, for [`plan_step`](Self::plan_step)'s thresholds.
    fn supply_traces(&self, p_idx: usize) -> Vec<SupplyTrace> {
        self.partitions[&p_idx]
            .supply_deps
            .iter()
            .map(|b| {
                let part = &self.partitions[b];
                let (t1, x1) = if part.has_pend { (part.pend_t, part.pend_x.clone()) } else { (part.t, part.x.clone()) };
                let before = (part.has_prev && part.prev_t < part.t).then(|| (part.prev_t, part.prev_x.clone()));
                SupplyTrace { nodes: part.shape.nodes.clone(), before, t0: part.t, x0: part.x.clone(), t1, x1 }
            })
            .collect()
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
        self.plan_step_cut(p_idx, horizon, drivers, live_params, voltages, 0, f64::INFINITY)
    }

    /// [`Self::plan_step`], the `cuts`-th time for a stressed device's ratio
    /// (ruling R90 (a)): a plan whose accepted step changes a stressed
    /// device's ratio by more than e^0.5 is planned again under a horizon
    /// that step's length cut in proportion, at most `STRESS_CUTS` times. It
    /// reads the step as the plan finally takes it, after its CC exit, edge
    /// and crossing cuts (cold 535 tpl 2: the CC trial from its exit held the
    /// LED at 1604 and the exit's CV re-solve over the same 10 ms fell
    /// 16-fold), on every planning path: a Step event's, a re-seed's, a
    /// catch-up's, a clamp's, exact mode's deferred plan. Each cut plans
    /// again from the step length, regime and CC exit the plan started from,
    /// under `cut` (ruling R80V3 2), so a cut step is the step a plan at
    /// that reach makes outright.
    #[allow(clippy::too_many_arguments)]
    fn plan_step_cut(&mut self, p_idx: usize, horizon: f64, drivers: &[Driver], live_params: &HashMap<usize, Vec<f64>>, voltages: &[f64], cuts: u32, cut: f64) -> Result<(), String> {
        let option_method = self.method;
        // Each partition this one's watches take a supply from is a known
        // trajectory only as far as its pending trial reaches, so a step
        // never runs past the nearest trial's end (below): inside it the
        // supply is the trial, interpolated, and a threshold at any instant
        // of the step is one of its values, not the last published one.
        let traces = self.supply_traces(p_idx);
        let ics = &self.ics;
        let node_owner = &self.node_owner;
        let scratch = &mut self.threshold_scratch;
        let rail = self.rail;
        let ramps = self.ramps.as_deref();
        let holds: &[digital::power::PowerHold] = if self.holds_on { &self.holds } else { &[] };
        let exact = self.exact;
        // Only a cell's charge is billed (`commit_partition`); a PSU's never
        // is, so its trial skips the source-current solves.
        let billed = exact.filter(|e| e.partition == p_idx && self.cell.is_some());
        let cc = match (self.cc_held, &self.cc, self.partitions[&p_idx].hub, exact) {
            // A held rail's capacitors go on from where this partition's own
            // last step left them (ruling R68).
            (Some(held), _, true, None) => {
                let part = &self.partitions[&p_idx];
                let cap = held.cap.map(|c| match part.rail_q {
                    Some((tc, q, i)) if tc == part.t => CapStep { q, i, ..c },
                    _ => c,
                });
                Some(CcStep { cap, ..held })
            }
            (None, Some(cc), true, None) => {
                let part = &self.partitions[&p_idx];
                Some(cc.step(p_idx, voltages[cc.rail], part.t, part.rail_q, self.rail_charge))
            }
            _ => None,
        };
        let clamp = self.clamp;
        let probes: &[StressProbe] = self.stress_probes.get(&p_idx).map_or(&[], |v| v.as_slice());
        // Ruling R90 (a): not while an IC this partition stamps is held (W1c
        // point 1a): cut under the hold procedure's table, cold 7 tpl 5's
        // stepped rail partition in exact mode re-seeded under its held 555
        // at every step after.
        let in_p = |n: usize| node_owner.get(&n).is_some_and(|o| o.0 == p_idx);
        #[cfg(test)]
        let forced = FORCE_HELD.with(|f| f.get());
        #[cfg(not(test))]
        let forced: Option<usize> = None;
        let held = |d: usize| forced == Some(d) || holds.get(d).is_some_and(|h| !h.is_free());
        let held_here = !probes.is_empty() && (forced.is_some() || holds.iter().any(|h| !h.is_free())) && ics.iter().any(|(d, ic)| held(*d) && (in_p(ic.vcc.node) || in_p(ic.gnd.node) || ic.logic.iter().any(|pin| in_p(pin.node))));
        let part = self.partitions.get_mut(&p_idx).unwrap();
        // Within one horizon clamp (`sync_partition`) the CC rail it was
        // handed stays until the rail loop after it, so a step after one cut
        // at the exit, or solved in CV, is solved in CV too (`CcStep::cv`).
        // Cut again at each start, cold 75 tpl 3's steps had cut the clamp's
        // 1.9 ms into 19,393 steps of 20-40 ns: its CC solution sat at 24 V
        // a few ns into each, past a 5 V setpoint, and a CC trial of more
        // failed Newton, a quarter of the step each time.
        let cc = cc.map(|c| CcStep { cv: part.cc_exited, ..c });
        let shape_n = part.shape.n;
        // R79 (a): a linear partition (no junction, no IC stamp that follows
        // its own iterate) steps by the trapezoidal rule whatever the
        // INTEGRATION option; the step after a discontinuity stays backward
        // Euler, as classic's trap mode has it. A partition on the PSU's
        // rail keeps backward Euler (`on_rail`): the trapezoid there let a
        // rail's stiff draw ring (761/5, 207/5 on the cold sweep), and the
        // rail jumps at a regime change with no discontinuity for its
        // readers. A partition fallen back (R81 (1),
        // `trap_off`) keeps backward Euler until its next discontinuity,
        // under either option.
        let trap_run = !part.trap_off && (option_method == Method::Trap || (!part.iterated_on(cc.is_some()) && !part.on_rail));
        let mut method = if part.be_next_step || !trap_run { Method::Be } else { Method::Trap };
        // R96, R98: a backward Euler step is TR-BDF2 (`take_step`), except
        // the one after a discontinuity (a CV/CC regime switch is one: exact
        // mode's re-solve and default mode's restart both set
        // `be_next_step`), a step on a CC rail, whose rail capacitors
        // (`CapStep`) and coupling bound have no TR-BDF2 form, and exact
        // mode's cell partition: its E and r are held from the SoC at each
        // step's start, an explicit first-order coupling that TR-BDF2's
        // longer steps stretch (a 5 V, 0.9 ohm cell draining into 100 nF
        // and 100 ohm read its rail 3.8 W0 off the fine reference at 0.6 s
        // against 1.5).
        let cell_partition = self.cell.is_some() && self.exact.is_some_and(|e| e.partition == p_idx);
        let trb = !part.be_next_step && cc.is_none() && !cell_partition;
        TRB.with(|f| f.set(trb));
        let _trb_guard = TrbGuard;
        part.pend_trap_off = false;
        let vc0 = part.vc.clone();
        let x0 = part.x.clone();
        let i_prev = part.i_prev.clone();
        let keep_lu = part.hub && rail.is_some();
        // A step held by W1c point 1a (`hold_around`) solves the rest of its
        // plan under the table it took.
        let held_table: std::cell::OnceCell<Vec<digital::power::PowerHold>> = std::cell::OnceCell::new();
        let mut step_taken: Vec<(usize, digital::power::PowerHold)> = Vec::new();
        let mut ic_ctx = IcStampCtx { now: part.t, drivers, ics, node_owner, bnd: Bnd::new(voltages).with_holds(holds), ramps, keep_lu, exact, cc, p_idx, iterated: part.iterated_on(cc.is_some()), free_supply: part.holds_free_ic_supply };

        let supply_reach = traces.iter().filter(|tr| tr.t1 > part.t + 1e-15).map(|tr| tr.t1 - part.t).fold(f64::INFINITY, f64::min);
        // Ruling R90 (a): a stressed device's ratio may change by e^0.5 at
        // most in a step, a horizon on the step, never the error controller.
        let start_state = (part.h, part.changing, part.cc_exited);
        let mut h = part.h.min(if part.changing { ACTIVE_H_MAX } else { H_MAX }).min(horizon - part.t).min(supply_reach).min(part.stress_reach.min(cut).max(H_MIN));
        let mut ratio;
        let mut accepted: StepResult;
        // The first half of the accepted step-doubling pair, whose end the
        // committed second half starts from; `None` when the accepted step
        // is one solve over `h`.
        let mut half: Option<StepResult>;
        let mut growth: Option<f64> = None;
        loop {
            let attempt = attempt_step(&part.shape, live_params, h, &vc0, &x0, &i_prev, method, trap_run, shape_n, &mut part.newton_iterations, &ic_ctx, part.t);
            match attempt {
                Ok((step, r, mid, ringing, rest)) => {
                    let coupling = coupling_ratio(ic_ctx.cc, &step, mid.as_ref(), method);
                    ratio = r.max(coupling);
                    let mut smooth = rest.max(coupling);
                    if ic_ctx.cc.is_some() && part.cc_switch {
                        ratio *= CC_STEP_TOL_SCALE;
                        smooth *= CC_STEP_TOL_SCALE;
                    }
                    // R81 (3), the ring-aware R77: a trapezoidal step that
                    // fails on its ringing series currents alone (`ringing`,
                    // everything else within tolerance) is taken. A shorter
                    // step does not shrink the trapezoid's `-1` mode, so
                    // failing it shortened the step for good: INTEGRATION
                    // trap's draining cell in exact mode (`exact_mode_takes_
                    // e_and_r_per_step...`) took 15 million steps to 0.6 s.
                    // Classic fails it and crawls; a recorded divergence.
                    if ratio > 1.0 && method == Method::Trap && h > H_MIN * 1.01 && ringing > 1.0 && smooth <= 1.0 {
                        ratio = smooth;
                    }
                    // R81 (1): a trapezoidal step that fails its estimate
                    // even at the floor meets a stiff mode the trapezoid
                    // does not damp, which no shorter step removes: the
                    // partition goes on by backward Euler until its next
                    // discontinuity, under either INTEGRATION option
                    // (counted, stat `27`; a recorded divergence from
                    // classic's trap mode, which crawls). Falling back on a
                    // ring above the floor instead gave up the trapezoid for
                    // good on a board whose inrush rang once (the R68
                    // two-partition board, node 6 at 10 ms 18.8 W0 off the
                    // fine reference against 1.5).
                    if ratio > 1.0 && method == Method::Trap && h <= H_MIN * 1.01 {
                        method = Method::Be;
                        part.pend_trap_off = true;
                        continue;
                    }
                    // At the floor there is no smaller step: it is taken,
                    // and its estimate holds the next one there (R78 (a)).
                    if ratio <= 1.0 || h <= H_MIN * 1.01 {
                        accepted = step;
                        half = mid;
                        // A ring inside R77's tolerance passes, and a shorter
                        // step would not shrink it, so it does not set the
                        // growth; the rest of the estimate does (R88's
                        // divergence: classic feeds the whole capsI ratio to
                        // its next factor, and crawls on 287/3).
                        if ratio <= 1.0 {
                            growth = Some(smooth);
                        }
                        break;
                    }
                    h = (h * next_step_factor(ratio, if trb { Method::Trap } else { method })).max(H_MIN);
                    part.changing = true;
                }
                // A rejected step (README, LEDs point 8; classic's
                // `nextFactor: 0.25`, HL:609-617): retry the whole trio at a
                // quarter of `h`, and give up only when there is no smaller
                // step to try (classic's `hitMinStep`, HL:827-829).
                Err(StepError::NoConverge) => {
                    if h <= H_MIN * 1.01 {
                        // A chip alternating at its power edge
                        // through the smallest step is held there (W1c point
                        // 1a, `hold_around`), as at a seed.
                        if held_table.get().is_none() && holds_around(&ic_ctx) {
                            let (shape, t0) = (&part.shape, part.t);
                            let iters = &mut part.newton_iterations;
                            let mut solve = |ctx: &IcStampCtx, noting: Option<Noting>| solve_step_noting(shape, live_params, h, &vc0, &x0, &i_prev, method, iters, ctx, t0 + h, false, noting);
                            if let Ok((step, Some((table, taken)))) = hold_around(&ic_ctx, &mut solve) {
                                let _ = held_table.set(table);
                                ic_ctx = IcStampCtx { bnd: ic_ctx.bnd.with_holds(held_table.get().unwrap()), ..ic_ctx };
                                step_taken = taken;
                                ratio = 1.0;
                                accepted = step;
                                half = None;
                                break;
                            }
                        }
                        return Err(format!("stepped: Newton did not converge at the minimum step h = {h:e} s"));
                    }
                    h = (0.25 * h).max(H_MIN);
                    part.changing = true;
                }
                Err(StepError::Failed(message)) => return Err(message),
            }
        }

        let order_exp = if method == Method::Trap || trb { 3 } else { 2 };
        if h > H_MIN * 2.01 {
            part.changing = ratio * (H_MAX / h).powi(order_exp) > QUIET_FRACTION;
        }
        part.h = (h * next_step_factor(growth.unwrap_or(ratio), if trb { Method::Trap } else { method })).clamp(H_MIN, if part.changing { ACTIVE_H_MAX } else { H_MAX });

        // The CC exit inside the step (Rails point 5): the step is cut where
        // its CC rail reaches the setpoint, just past it, so the regime rules
        // leave CC at this commit, at the exit's own time
        // rather than up to a whole step (`H_MAX`) late. Regula falsi on the
        // rail's distance past the setpoint (near linear in `h` on a ramp),
        // as a crossing is refined.
        if let (Some(cc), Some(u_hi)) = (ic_ctx.cc, accepted.u) {
            // The bracket starts at the shortest step: a CC rail already past
            // the setpoint there (the draws fell below the limit since the
            // last rail solve, as when a load is switched out) is an exit at
            // the step's start, and the whole step is solved in CV, the rail
            // at `V` (a re-entry is found at the commit, as every regime
            // change of a stepped partition is); cutting toward a start that
            // is no CC solution made steps of 1e-12 s.
            let first = if cc.past_setpoint(u_hi) && h > 2.0 * H_MIN {
                let step = take_step(&part.shape, live_params, H_MIN, &vc0, &x0, &i_prev, method, &mut part.newton_iterations, &ic_ctx, part.t + H_MIN).map_err(StepError::into_message)?;
                Some((cc.beyond(step.u.expect("a CC step reports its rail")), step))
            } else {
                None
            };
            let first = match first {
                Some((f, _)) if f > 0.0 => {
                    accepted = take_cc_step(&part.shape, live_params, h, &vc0, &x0, &i_prev, method, &mut part.newton_iterations, &ic_ctx, part.t + h, cc, true).map_err(StepError::into_message)?;
                    half = None;
                    part.cc_exited = clamp;
                    None
                }
                other => other.map(|(f, _)| f),
            };
            if let Some(f_first) = first {
                let (mut lo, mut f_lo) = (H_MIN, f_first);
                let (mut hi, mut f_hi) = (h, cc.beyond(u_hi));
                // `f_hi` and `f_lo` are the weights regula falsi uses (halved
                // on a stuck side); `past` is the true distance at `hi`.
                let mut past = f_hi;
                let mut best = None;
                let mut side = 0i8;
                for _ in 0..CROSS_REFINE_MAX_ITERS {
                    if past <= CROSS_TOLERANCE_V || hi - lo <= H_MIN * 1e-3 {
                        break;
                    }
                    let frac = (-f_lo / (f_hi - f_lo)).clamp(1e-3, 1.0 - 1e-3);
                    let h_try = lo + frac * (hi - lo);
                    let step = take_step(&part.shape, live_params, h_try, &vc0, &x0, &i_prev, method, &mut part.newton_iterations, &ic_ctx, part.t + h_try).map_err(StepError::into_message)?;
                    let f = cc.beyond(step.u.expect("a CC step reports its rail"));
                    if f > 0.0 {
                        (hi, f_hi, past) = (h_try, f, f);
                        best = Some(step);
                        if side == 1 {
                            f_lo *= 0.5;
                        }
                        side = 1;
                    } else {
                        (lo, f_lo) = (h_try, f);
                        if side == -1 {
                            f_hi *= 0.5;
                        }
                        side = -1;
                    }
                }
                if let Some(step) = best {
                    h = hi;
                    accepted = step;
                    half = None;
                    part.cc_exited = clamp;
                }
            }
        }

        // A rail IC's power edge inside a CC step on rail capacitors (ruling
        // R68): the step is cut where its rail reaches the edge, just past
        // it, as at the setpoint above, so the IC's power flips at that
        // commit's rail solve. The step's rail is the capacitors' charge,
        // which a solve at the instant cannot move back; its gate judged at
        // the step's start for the whole step, cold 505 tpl 5's stepped
        // partition carried the rail 0.47 V past a 74LS00's 4.275 V floor
        // in one 2.6 ms step, and the powered chip's draw then pulled the
        // rail back at 3e5 V/s through a floor the rail Newton could not
        // straddle (`-4`).
        if let (Some(cc), Some(u_hi)) = (ic_ctx.cc.filter(|c| !c.cv && c.cap.is_some()), accepted.u) {
            let f_hi0 = cc.edge_past(u_hi);
            // The bracket starts at the shortest step, as the setpoint's: the
            // step's own rail at its start is the capacitors' charge, which
            // can stand apart from the published `u0` (cold 769 tpl 5: a CC
            // rail at 0.36 mV beside a published setpoint), and a rail past
            // the edge already there is no crossing inside the step.
            let first = if f_hi0 > 0.0 && !cc.past_setpoint(u_hi) && h > 2.0 * H_MIN {
                let step = take_step(&part.shape, live_params, H_MIN, &vc0, &x0, &i_prev, method, &mut part.newton_iterations, &ic_ctx, part.t + H_MIN).map_err(StepError::into_message)?;
                Some(cc.edge_past(step.u.expect("a CC step reports its rail")))
            } else {
                None
            };
            if let Some(f_first) = first.filter(|&f| f < 0.0) {
                let (mut lo, mut f_lo) = (H_MIN, f_first);
                let (mut hi, mut f_hi) = (h, f_hi0);
                let mut past = f_hi;
                let mut best = None;
                let mut side = 0i8;
                for _ in 0..CROSS_REFINE_MAX_ITERS {
                    if past <= CROSS_TOLERANCE_V || hi - lo <= H_MIN * 1e-3 {
                        break;
                    }
                    let frac = (-f_lo / (f_hi - f_lo)).clamp(1e-3, 1.0 - 1e-3);
                    let h_try = lo + frac * (hi - lo);
                    let step = take_step(&part.shape, live_params, h_try, &vc0, &x0, &i_prev, method, &mut part.newton_iterations, &ic_ctx, part.t + h_try).map_err(StepError::into_message)?;
                    let f = cc.edge_past(step.u.expect("a CC step reports its rail"));
                    if f > 0.0 {
                        (hi, f_hi, past) = (h_try, f, f);
                        best = Some(step);
                        if side == 1 {
                            f_lo *= 0.5;
                        }
                        side = 1;
                    } else {
                        (lo, f_lo) = (h_try, f);
                        if side == -1 {
                            f_hi *= 0.5;
                        }
                        side = -1;
                    }
                }
                if let Some(step) = best {
                    h = hi;
                    accepted = step;
                    half = None;
                }
            }
        }

        // Iterative crossing refinement (this module's doc comment).
        if !part.watches.is_empty() {
            let t0 = part.t;
            let mut env = ThresholdEnv::new(&part.shape.nodes, &traces, voltages, scratch, ic_ctx.cc);
            if let Some(crossing) = find_crossing(&part.watches, &x0, t0, &accepted.x, t0 + h, &mut env) {
                let refined_h = refine_crossing_h(
                    &part.shape,
                    live_params,
                    h,
                    &vc0,
                    &x0,
                    &i_prev,
                    method,
                    &part.watches,
                    crossing,
                    t0,
                    &mut env,
                    &mut part.newton_iterations,
                    &ic_ctx,
                )?;
                // A catch-up (`sync_partition`) reads each watch's threshold
                // from the published rail, which it does not move, while its
                // steps move the rail's own partition: a node its step's
                // start carries past that threshold at once (a CC rail's
                // step beside the published one) bisects to a step of
                // ~1e-16 s at every step, and the catch-up never reaches its
                // horizon (cold 3254 tpl 0: 0.1 ns from the horizon, a tab
                // frozen for good). No catch-up step is shorter than the
                // shortest step, as no planned one is: a crossing it cannot
                // act on before its horizon anyway is taken at `H_MIN`.
                let refined_h = if clamp && refined_h < H_MIN { H_MIN.min(h) } else { refined_h };
                h = refined_h;
                // Deterministic: `refined_h` is an `h` a trial (or the accepted
                // step itself) already converged at.
                accepted = take_step(&part.shape, live_params, h, &vc0, &x0, &i_prev, method, &mut part.newton_iterations, &ic_ctx, t0 + h).map_err(StepError::into_message)?;
                half = None;
            }
        }

        // Ruling R90 (a): a stressed device's ratio over the step as taken.
        if !probes.is_empty() && !held_here && held_table.get().is_none() && cuts < STRESS_CUTS && h > 2.0 * H_MIN {
            let t1 = part.t + h;
            let bnd1 = ic_ctx.bnd.at_time(ramps, t1);
            let u1 = accepted.u.filter(|_| part.hub).zip(rail);
            let end = |n: usize| match node_owner.get(&n) {
                Some(&(q, k)) if q == p_idx => accepted.x[k],
                _ => match u1 {
                    Some((u, r)) if r == n => u,
                    _ => bnd1.get(n),
                },
            };
            let q1: Vec<(f64, f64)> = probes.iter().map(|pr| pr.ratio(&end)).collect();
            let stressed = q1.iter().any(|&(q, zero)| q >= 0.5 * zero);
            if stressed || part.stress_watch {
                let bnd0 = ic_ctx.bnd.at_time(ramps, part.t);
                let reads = &part.end_reads;
                let start = |n: usize| match node_owner.get(&n) {
                    Some(&(q, k)) if q == p_idx => x0[k],
                    _ => reads.iter().rev().find(|r| r.0 == n).map_or_else(|| bnd0.get(n), |r| r.1),
                };
                let q0: Vec<(f64, f64)> = probes.iter().map(|pr| pr.ratio(&start)).collect();
                if let Some(cut) = stress_cut(q0.into_iter().zip(q1), h) {
                    let t0 = part.t;
                    #[cfg(test)]
                    STRESS_CUT_LOG.with(|l| l.borrow_mut().push((p_idx, t0, h, cut)));
                    #[cfg(test)]
                    STRESS_CUT_PATHS.with(|l| l.borrow_mut().push((t0, clamp, exact.is_some_and(|e| e.partition == p_idx))));
                    (part.h, part.changing, part.cc_exited) = start_state;
                    return self.plan_step_cut(p_idx, horizon, drivers, live_params, voltages, cuts + 1, cut);
                }
            }
            part.stress_watch = stressed;
        }

        // The source's charge over the trial in the integrator's own
        // quadrature (Rails point 9, ruling R5): over each solved sub-step,
        // the two half steps of the accepted pair or the one step over `h`,
        // its end current times its length under backward Euler and the
        // mean of its start and end currents under the trapezoid, which is
        // exactly how far each capacitor's charge moves, so the bill
        // conserves charge with the integrator.
        if let Some(e) = billed {
            let t0 = part.t;
            let shape = &part.shape;
            let draw = |at: f64, h_sub: f64, vc_from: &[f64], i_from: &[f64], x: &[f64], m: Method| {
                let bnd = ic_ctx.bnd.at_time(ramps, at);
                source_current(&e, shape, live_params, h_sub, vc_from, i_from, m, &ic_ctx, &bnd, x)
            };
            // One solved sub-step from `t_s` (state `vc_s`, `i_s`, source
            // current `i0`) to `t_s + h_sub` (`end`): its charge and end
            // current. A TR-BDF2 sub-step bills in its own quadrature, as
            // each capacitor's charge moves: C (v1 - v0) = h_2 i_1 + k
            // (gamma h / 2) (i_0 + i_g), the trapezoid over its first stage
            // weighted by k, and the BDF2 stage's end current over h_2.
            let sub = |t_s: f64, h_sub: f64, vc_s: &[f64], i_s: &[f64], i0: f64, end: &StepResult| -> (f64, f64) {
                match &end.stage {
                    Some(st) => {
                        let g = TRBDF2_GAMMA;
                        let k = 1.0 / (g * (2.0 - g));
                        let i_g = draw(t_s + st.h_g, st.h_g, vc_s, i_s, &st.x_g, Method::Trap);
                        let i_1 = draw(t_s + h_sub, st.h_2, &st.vh, &st.i_g, &end.x, Method::Be);
                        (k * 0.5 * st.h_g * (i0 + i_g) + st.h_2 * i_1, i_1)
                    }
                    None => {
                        let i_1 = draw(t_s + h_sub, h_sub, vc_s, i_s, &end.x, method);
                        let q = match method {
                            Method::Be => i_1 * h_sub,
                            Method::Trap => 0.5 * (i0 + i_1) * h_sub,
                        };
                        (q, i_1)
                    }
                }
            };
            let i_start = part.i_src;
            (part.pend_charge, part.pend_i_src) = match &half {
                Some(mid) => {
                    let (q_a, i_mid) = sub(t0, h / 2.0, &vc0, &i_prev, i_start, mid);
                    let (q_b, i_end) = sub(t0 + h / 2.0, h / 2.0, &mid.vc, &mid.i, i_mid, &accepted);
                    (q_a + q_b, i_end)
                }
                None => sub(t0, h, &vc0, &i_prev, i_start, &accepted),
            };
        }
        part.pend_dense.clear();
        let dense_on = self.dense_on.get(p_idx).copied().unwrap_or(false);
        if dense_on || self.charge_on.get(p_idx).copied().unwrap_or(false) {
            let t0 = part.t;
            let push_sub = |knots: &mut Vec<Knot>, t_s: f64, vc_s: &[f64], t_e: f64, end: &StepResult| {
                match &end.stage {
                    Some(st) => {
                        // The stage's capacitor voltages from the BDF2 history
                        // `vh = k vc_g - m vc_s`.
                        let g = TRBDF2_GAMMA;
                        let k = 1.0 / (g * (2.0 - g));
                        let m = (1.0 - g) * (1.0 - g) * k;
                        let vc_g = st.vh.iter().zip(vc_s).map(|(vh, v0)| (vh + m * v0) / k).collect();
                        knots.push(Knot { t: t_s + st.h_g, x: st.x_g.clone(), vc: vc_g, i: st.i_g.clone(), hermite: true, stage: true, u: st.u_g, rail_i: None });
                        knots.push(Knot { t: t_e, x: end.x.clone(), vc: end.vc.clone(), i: end.i.clone(), hermite: true, stage: false, u: end.u, rail_i: end.rail_q.map(|r| r.1) });
                    }
                    None => knots.push(Knot { t: t_e, x: end.x.clone(), vc: end.vc.clone(), i: end.i.clone(), hermite: method == Method::Trap, stage: false, u: end.u, rail_i: end.rail_q.map(|r| r.1) }),
                }
            };
            let mut knots = Vec::new();
            match &half {
                Some(mid) => {
                    push_sub(&mut knots, t0, &vc0, t0 + h / 2.0, mid);
                    push_sub(&mut knots, t0 + h / 2.0, &mid.vc, t0 + h, &accepted);
                }
                None => push_sub(&mut knots, t0, &vc0, t0 + h, &accepted),
            }
            // An iterated partition's nodes at each grid time in the step,
            // solved at the capacitors' Hermite voltages there under the
            // step's own boundary and drivers (R104 as ruled, option (b)): a
            // scratch solve, which nothing keeps but its values.
            let mut solved = Vec::new();
            let dense_solve = dense_on && self.dense_solve.get(p_idx).copied().unwrap_or(false);
            if let (true, Some((origin, interval))) = (dense_solve && part.iterated() && !part.hub && ramps.is_none(), self.dense_grid) {
                let seg = DenseSeg { p: p_idx, t0, x0: x0.clone(), vc0: vc0.clone(), i0: i_prev.clone(), knots: knots.clone(), solved: Vec::new(), steady: true, affine: false, sens: None };
                let mut k = ((t0 - origin) / interval).floor().max(0.0) as u64;
                loop {
                    let ts = origin + k as f64 * interval;
                    k += 1;
                    if ts <= t0 {
                        continue;
                    }
                    if ts > t0 + h {
                        break;
                    }
                    let vc_ts = seg.vc_at(&part.shape, ts);
                    let x_lin: Vec<f64> = (0..x0.len()).map(|l| seg.interpolate(&part.shape, l, ts, None)).collect();
                    let ctx = IcStampCtx { now: ts, keep_lu: false, ..ic_ctx };
                    let x = solve_initial_x(&part.shape, live_params, &vc_ts, &x_lin, &ctx).ok().map(|r| r.x);
                    #[cfg(test)]
                    let x = x.filter(|_| !FAIL_DENSE_SOLVE.with(|f| f.get()));
                    solved.push((ts, x));
                }
            }
            let u0 = ic_ctx.cc.map(|c| c.cap.map_or(c.u0, |cap| cap.q));
            let rail_i0 = ic_ctx.cc.and_then(|c| c.cap).filter(|_| ic_ctx.cc.is_some_and(|c| !c.seed)).map(|cap| cap.i);
            part.pend_dense = PendDense { knots, solved, u0, rail_i0 };
        }
        #[cfg(test)]
        {
            part.pend_mid = half.map(|mid| (part.t + h / 2.0, mid.x, mid.i));
        }
        part.pend_x = accepted.x;
        part.pend_vc = accepted.vc;
        part.pend_i = accepted.i;
        part.pend_lu = accepted.lu.map(Rc::new);
        part.pend_u = accepted.u.unwrap_or_else(|| rail.map_or(0.0, |r| voltages[r]));
        part.pend_now = part.t;
        part.pend_h = h;
        part.pend_method = method;
        part.pend_cc = ic_ctx.cc;
        part.pend_rail_q = accepted.rail_q;
        part.pend_t = part.t + h;
        part.has_pend = true;
        part.version += 1;
        let since = part.t;
        if !step_taken.is_empty() {
            // Held in this copy at once, so the steps after it keep them; the
            // owner enters them in its table after the call.
            let table = Rc::make_mut(&mut self.holds);
            for &(d, hold) in &step_taken {
                if table.len() <= d {
                    table.resize(d + 1, digital::power::PowerHold::FREE);
                }
                table[d] = hold;
            }
            self.holds_on = true;
            self.hold_requests.extend(step_taken.iter().map(|&(d, hold)| (d, hold, since)));
        }
        Ok(())
    }
}

/// One attempt at the step-doubling trio at `h`: the result to commit, its
/// error ratio, the first half step the committed second half continues
/// from (`None` when the committed result is the one step over `h`), the
/// ratio of the capacitor currents that ring (R77, `ringing`) and the ratio
/// of everything else (what a shorter step would improve). `NoConverge` from
/// any of the three solves rejects the whole attempt (`plan_step`), except
/// at the floor.
///
/// At the floor (`h <= 2.01 * H_MIN`) the trio is an estimate only (R78
/// (a)): its halves fall below the floor, so the floor step itself is
/// committed as the one plain step it always was, and the estimate decides
/// whether the next step may grow from it; where a half does not converge
/// the floor step stands unchecked.
#[allow(clippy::too_many_arguments)]
fn attempt_step(
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc0: &[f64],
    x0: &[f64],
    i_prev: &[f64],
    method: Method,
    trap_run: bool,
    shape_n: usize,
    newton_iters: &mut u64,
    ic_ctx: &IcStampCtx,
    t0: f64,
) -> Result<(StepResult, f64, Option<StepResult>, f64, f64), StepError> {
    let coarse = take_step(shape, live_params, h, vc0, x0, i_prev, method, newton_iters, ic_ctx, t0 + h)?;
    let at_floor = h <= H_MIN * 2.01;
    let mid = match take_step(shape, live_params, h / 2.0, vc0, x0, i_prev, method, newton_iters, ic_ctx, t0 + h / 2.0) {
        Err(StepError::NoConverge) if at_floor => return Ok((coarse, 0.0, None, 0.0, 0.0)),
        other => other?,
    };
    // The second half starts from the rail capacitors' charge the first
    // left (ruling R68).
    let second = IcStampCtx { cc: ic_ctx.cc.map(|cc| CcStep { cap: cc.cap.zip(mid.rail_q).map(|(c, (q, i))| CapStep { q, i, ..c }), ..cc }), ..*ic_ctx };
    let fine = match take_step(shape, live_params, h / 2.0, &mid.vc, &mid.x, &mid.i, method, newton_iters, &second, t0 + h) {
        Err(StepError::NoConverge) if at_floor => return Ok((coarse, 0.0, None, 0.0, 0.0)),
        other => other?,
    };

    let mut ratio = 0.0f64;
    let mut ringing = 0.0f64;
    for k in 0..shape_n {
        let r = rel_error(coarse.x[k], fine.x[k]);
        if r > ratio {
            ratio = r;
        }
    }
    // Everything but a ring's current: what a shorter step would improve.
    let mut rest = ratio;
    for i in 0..vc0.len() {
        let r = rel_error(coarse.vc[i], fine.vc[i]);
        if r > ratio {
            ratio = r;
        }
        rest = rest.max(r);
        // R77 (a): in a trapezoidal run the series current is the next
        // step's history, compared as classic compares `capsI`
        // (`adaptive-step.ts` estimateStepError), so a ring the charges hide
        // is still caught.
        if trap_run {
            let (ci, fi) = (coarse.i[i], fine.i[i]);
            let ri = (ci - fi).abs() / (ABS_CURRENT + REL_TOL * ci.abs().max(fi.abs()));
            ratio = ratio.max(ri);
            // A ring, not a step too long, read about the current it rides
            // on: over the two halves the trapezoid's undamped `-1` mode
            // steps the current down and back up (`i_prev`, `mid`, `fine` =
            // I+a, I-a, I+a), so the halves' net move is small beside their
            // swing, where a smooth current moves the same way twice.
            if (fine.i[i] - i_prev[i]).abs() < 0.5 * (fine.i[i] - 2.0 * mid.i[i] + i_prev[i]).abs() {
                ringing = ringing.max(ri);
            } else {
                rest = rest.max(ri);
            }
        }
    }
    if let (Some((qc, _)), Some((qf, _))) = (coarse.rail_q, fine.rail_q) {
        ratio = ratio.max(rel_error(qc, qf));
        rest = rest.max(rel_error(qc, qf));
    }
    if at_floor {
        return Ok((coarse, ratio, None, ringing, rest));
    }
    Ok((fine, ratio, Some(mid), ringing, rest))
}

/// The cross-branch coupling on a CC rail (ruling R37 (a), R39 (1)), as a
/// step-doubling ratio for [`next_step_factor`]. Each stepped hub partition
/// on a CC rail with no capacitor steps with the rail as an unknown and every
/// other partition's draw held where it last committed (`CcStep::a_o`), so
/// over this step the others see this partition's draw stale, and it sees
/// theirs: the rail they each solve parts from the true rail by the rail's
/// move across the step, which the step-doubling comparison cannot see (both
/// halves hold the same draws). Kept within W0 of the rail, the coupling
/// error stays within the controller's own: on FXEFv's boards (`cc`, tpl 10
/// and 12) the worst node went from 39 and 43 W0 off a fine reference to 6.5
/// and 9.0, the backward-Euler controller's own floor on those boards, at
/// up to 4.4k steps per 0.1 s (faster than real time). The move is taken as
/// twice the second half step's, since a step's own rail at its start can
/// differ from the published rail by a re-solve's offset (taken whole, the
/// offset alone drove a board to 12k steps against 1.1k). The coupling
/// error is first order in `h`, so it enters as its square for backward
/// Euler (the cube for the trapezoid), which [`next_step_factor`] turns back
/// into `0.9 / move`.
fn coupling_ratio(cc: Option<CcStep>, end: &StepResult, mid: Option<&StepResult>, method: Method) -> f64 {
    let (Some(_), Some(u_end), Some(u_mid)) = (cc.filter(|c| c.coupled), end.u, mid.and_then(|m| m.u)) else {
        return 0.0;
    };
    let moved = 2.0 * (u_end - u_mid).abs() / (ABS_VOLTAGE + REL_TOL * u_end.abs().max(u_mid.abs()));
    match method {
        Method::Be => moved * moved,
        Method::Trap => moved * moved * moved,
    }
}

fn rel_error(a: f64, b: f64) -> f64 {
    (a - b).abs() / (ABS_VOLTAGE + REL_TOL * a.abs().max(b.abs()))
}

/// Another Stepped partition's trajectory over its pending trial: `x0` at
/// `t0` (its last commit) to `x1` at `t1` (the trial's end), or flat when it
/// has no trial, and `before` the state at the commit before that. What a
/// threshold that reads a node of that partition sees at any instant `t` in
/// between: the parabola through the three points (the trial's end alone
/// would be a straight line). A step's error control bounds its END, and a
/// trapezoidal step is second order there, so a straight line between the
/// ends can be off by several millivolts mid-step where the step itself is
/// good to a fraction of one; the parabola is a whole order better. With no
/// `before` (a discontinuity just dropped it, or nothing has committed) it
/// is the line, and the steps right after a discontinuity are short.
struct SupplyTrace {
    nodes: Vec<usize>,
    before: Option<(f64, Vec<f64>)>,
    t0: f64,
    x0: Vec<f64>,
    t1: f64,
    x1: Vec<f64>,
}

impl SupplyTrace {
    fn write(&self, scratch: &mut [f64], t: f64) {
        if self.t1 <= self.t0 {
            for (l, &node) in self.nodes.iter().enumerate() {
                if let Some(slot) = scratch.get_mut(node) {
                    *slot = self.x0[l];
                }
            }
            return;
        }
        let t = t.clamp(self.t0, self.t1);
        let f = (t - self.t0) / (self.t1 - self.t0);
        for (l, &node) in self.nodes.iter().enumerate() {
            let line = self.x0[l] + f * (self.x1[l] - self.x0[l]);
            let value = match &self.before {
                Some((tp, xp)) => {
                    // The parabola is the line plus a term that vanishes at
                    // both ends, `k (t - t0)(t - t1)`, with `k` fixed by the
                    // earlier point `(tp, xp)`.
                    let at_tp = self.x0[l] + (tp - self.t0) / (self.t1 - self.t0) * (self.x1[l] - self.x0[l]);
                    let k = (xp[l] - at_tp) / ((tp - self.t0) * (tp - self.t1));
                    line + k * (t - self.t0) * (t - self.t1)
                }
                None => line,
            };
            if let Some(slot) = scratch.get_mut(node) {
                *slot = value;
            }
        }
    }
}

/// The voltage array a [`ThresholdFn`] is called with while a step is
/// planned: the published array, with this partition's own nodes replaced by
/// the step's state at instant `t` and every supply partition's by its
/// trajectory at `t`. A watched IC's thresholds move with its supply, so
/// reading them from the array as last published (the start of the step, and
/// for another partition possibly older still) puts the crossing where the
/// threshold WAS, not where it is when the node arrives.
///
/// On a CC rail ([`CcStep`]) the rail is the step's own unknown too, and a
/// threshold of an IC on the rail moves with it: the crossing found against
/// the published rail is refined against each trial's own solved rail
/// (`rail`, with the rail the step starts from; `refine_crossing_h`). Refined
/// against the published rail, a 555 on a CC rail (cold 304 tpl 4) had each
/// step land where its threshold had been, and the next commit's rail solve
/// moved the threshold on: eight steps, each a fifth or less of the last,
/// closed on every crossing.
struct ThresholdEnv<'a> {
    nodes: &'a [usize],
    traces: &'a [SupplyTrace],
    scratch: &'a mut Vec<f64>,
    rail: Option<(usize, f64)>,
}

impl<'a> ThresholdEnv<'a> {
    fn new(nodes: &'a [usize], traces: &'a [SupplyTrace], published: &[f64], scratch: &'a mut Vec<f64>, cc: Option<CcStep>) -> Self {
        scratch.clear();
        scratch.extend_from_slice(published);
        ThresholdEnv { nodes, traces, scratch, rail: cc.map(|cc| (cc.rail, cc.u0)) }
    }

    /// The array at instant `t` with this partition at `x`, and on a CC rail
    /// the rail at `u` (a trial's own), or where the step started.
    fn at(&mut self, x: &[f64], t: f64, u: Option<f64>) -> &[f64] {
        for (l, &node) in self.nodes.iter().enumerate() {
            if let Some(slot) = self.scratch.get_mut(node) {
                *slot = x[l];
            }
        }
        for trace in self.traces {
            trace.write(self.scratch, t);
        }
        if let Some((rail, u0)) = self.rail {
            self.scratch[rail] = u.unwrap_or(u0);
        }
        self.scratch
    }
}

/// A watched threshold a step crosses: the watched local node, which of its
/// watchers, and which of that watcher's thresholds.
#[derive(Clone, Copy)]
struct Crossing {
    local: usize,
    watcher: usize,
    index: usize,
    rising: bool,
}

/// pec3's crossing search within a planned step (`:668-680`), linear
/// interpolation between the step's start (`x0` at `t0`) and its accepted end
/// (`x1` at `t1`): the EARLIEST (by interpolated fraction) watched threshold
/// this step actually crosses, or `None`. A threshold is a function of the
/// supply, so it is taken at each end of the step from the supply's value
/// THEN ([`ThresholdEnv`]), and a crossing is the node's distance from it
/// changing sign, whichever of the two moved (a supply sweeping a threshold
/// past a node that stands still is one too).
fn find_crossing(watches: &HashMap<usize, Vec<(DeviceId, SharedThresholdFn)>>, x0: &[f64], t0: f64, x1: &[f64], t1: f64, env: &mut ThresholdEnv) -> Option<Crossing> {
    let mut best: Option<(f64, Crossing)> = None;
    for (&local, watchers) in watches {
        for (w, (_, thresholds)) in watchers.iter().enumerate() {
            let at_start = thresholds(env.at(x0, t0, None));
            let at_end = thresholds(env.at(x1, t1, None));
            for (k, (th0, th1)) in at_start.iter().zip(&at_end).enumerate() {
                let (d0, d1) = (x0[local] - th0.value, x1[local] - th1.value);
                let ok = if th0.rising { d0 < 0.0 && d1 > 0.0 } else { d0 > 0.0 && d1 < 0.0 };
                if !ok {
                    continue;
                }
                let frac = d0 / (d0 - d1);
                if best.is_none_or(|(bf, _)| frac < bf) {
                    best = Some((frac, Crossing { local, watcher: w, index: k, rising: th0.rising }));
                }
            }
        }
    }
    best.map(|(_, c)| c)
}

/// Bisects `h` (never pec3's single `[0.02, 0.98]` fraction clamp, spec D11)
/// until the crossing node lands within [`CROSS_TOLERANCE_V`] of its
/// threshold (nudged past it, `closed_form::CROSS_NUDGE_V`'s same reasoning
/// applies here too — landing exactly ON a threshold risks re-triggering).
/// The threshold is re-read at every trial from the state and the supplies
/// at the trial's own instant `t0 + h`, so it is the one the node meets.
#[allow(clippy::too_many_arguments)]
fn refine_crossing_h(
    shape: &DynShape,
    live_params: &HashMap<usize, Vec<f64>>,
    h: f64,
    vc0: &[f64],
    x0: &[f64],
    i_prev: &[f64],
    method: Method,
    watches: &HashMap<usize, Vec<(DeviceId, SharedThresholdFn)>>,
    crossing: Crossing,
    t0: f64,
    env: &mut ThresholdEnv,
    newton_iters: &mut u64,
    ic_ctx: &IcStampCtx,
) -> Result<f64, String> {
    let thresholds = &watches[&crossing.local][crossing.watcher].1;
    let nudge = if crossing.rising { CROSS_TOLERANCE_V } else { -CROSS_TOLERANCE_V };
    // On a CC rail each trial's own rail first; a start the published rail
    // does not describe (an edge moved it, and the step's own rail at `h ->
    // 0` lies elsewhere: cold 674 tpl 4, 7.8 V published against a CC
    // solution near 0 V) reads as past at every trial, and bisected that
    // to `h / 2^40` the steps never reached the horizon. Then the published
    // rail is the threshold's, as before.
    let passes: &[bool] = if env.rail.is_some() { &[true, false] } else { &[false] };
    let mut best_h = h;
    for &trial_rail in passes {
        let (mut lo, mut hi) = (0.0f64, h);
        best_h = hi;
        for _ in 0..CROSS_REFINE_MAX_ITERS {
            let mid = 0.5 * (lo + hi);
            let trial = match take_step(shape, live_params, mid, vc0, x0, i_prev, method, newton_iters, ic_ctx, t0 + mid) {
                Ok(trial) => trial,
                // A trial that does not converge ends the refinement at the
                // best `h` found so far (README, LEDs point 8).
                Err(StepError::NoConverge) => return Ok(best_h),
                Err(StepError::Failed(message)) => return Err(message),
            };
            let Some(threshold) = thresholds(env.at(&trial.x, t0 + mid, trial.u.filter(|_| trial_rail))).get(crossing.index).copied() else {
                return Ok(best_h);
            };
            let target = threshold.value + nudge;
            let v = trial.x[crossing.local];
            let past = if crossing.rising { v >= target } else { v <= target };
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
        if lo > 0.0 {
            break;
        }
    }
    Ok(best_h)
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
        Program { carry: Vec::new(), node_count, options, digital_slot_count: 0, channel_slot_count: 0, devices }
    }
    fn build(program: &Program) -> SteppedSolver {
        let pb = partition::build(program).unwrap();
        SteppedSolver::build(program, &pb.partitions, &pb.local_index, &pb.fixed, &pb.kinds, LiveTopology::of(&pb))
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

    #[test]
    fn a_threshold_that_reads_another_node_of_the_partition_is_taken_at_the_trial_instant() {
        // The two-capacitor ladder, with node 3 (the far capacitor) watched
        // against half of node 2 (the near one), a threshold that RISES as the
        // step goes on. The crossing must land where node 3 meets half of node
        // 2 as it is at the landing instant, not half of node 2 as it was
        // when the step began.
        let mut st = build(&ladder_program(5.0, 1e3, 1e-6, 1e3, 1e-6));
        let mut voltages = vec![0.0, 5.0, 0.0, 0.0];
        let mut currents = vec![0.0; 5];
        st.resolve_dirty(&[0], 0.0, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert!(st.watch_node(3, DeviceId(0), Box::new(|v: &[f64]| vec![Threshold { value: 0.5 * v[2], rising: true }])));
        let (l2, l3) = (st.node_owner[&2].1, st.node_owner[&3].1);

        let mut landed = None;
        for _ in 0..200 {
            st.plan_step(0, f64::INFINITY, &[], &HashMap::new(), &voltages).unwrap();
            let part = &st.partitions[&0];
            let miss = part.pend_x[l3] - 0.5 * part.pend_x[l2];
            if miss.abs() < 1e-6 {
                landed = Some(miss);
                break;
            }
            let part = st.partitions.get_mut(&0).unwrap();
            SteppedSolver::commit(part);
            for (l, &node) in st.partitions[&0].shape.nodes.iter().enumerate() {
                voltages[node] = st.partitions[&0].x[l];
            }
        }
        let miss = landed.expect("a step must land on the crossing of node 3 with half of node 2");
        assert!(miss.abs() < 3e-9, "node 3 lands {miss} V from half of node 2 at the landing instant, expected within the 1 nV nudge");
    }

    // ---- G1's cross-partition supply tracking (manager ruling 2026-09-29) --

    fn trace(nodes: Vec<usize>, before: Option<(f64, Vec<f64>)>, t0: f64, x0: Vec<f64>, t1: f64, x1: Vec<f64>) -> SupplyTrace {
        SupplyTrace { nodes, before, t0, x0, t1, x1 }
    }

    #[test]
    fn a_supply_trace_with_an_earlier_commit_reads_the_parabola_through_its_three_points() {
        // Two nodes, each a quadratic in t, written as the quadratic itself so
        // the expectation does not come from `write`. Points at tp = 1, t0 = 2,
        // t1 = 4. Node 2: 0.5 t^2 - 3 t + 7 (4.5, 3, 3). Node 4: -0.25 t^2 + 2 t
        // + 1 (2.75, 4, 5). The other slots must stay as they were.
        let qa = |t: f64| 0.5 * t * t - 3.0 * t + 7.0;
        let qb = |t: f64| -0.25 * t * t + 2.0 * t + 1.0;
        let tr = trace(vec![2, 4], Some((1.0, vec![qa(1.0), qb(1.0)])), 2.0, vec![qa(2.0), qb(2.0)], 4.0, vec![qa(4.0), qb(4.0)]);
        let mut scratch = vec![-99.0; 6];
        for &t in &[2.0, 2.25, 2.5, 3.0, 3.5, 3.9, 4.0] {
            tr.write(&mut scratch, t);
            assert!((scratch[2] - qa(t)).abs() < 1e-12, "node 2 at t={t}: {} vs {}", scratch[2], qa(t));
            assert!((scratch[4] - qb(t)).abs() < 1e-12, "node 4 at t={t}: {} vs {}", scratch[4], qb(t));
        }
        for slot in [0, 1, 3, 5] {
            assert_eq!(scratch[slot], -99.0, "slot {slot} is not a supply node and must be left alone");
        }
        // And it is the parabola, not the straight line between the ends: at
        // t = 3 the line gives 3 for node 2 (its ends are both 3), the
        // quadratic 2.5; node 4's line gives 4.5, the quadratic 4.75.
        tr.write(&mut scratch, 3.0);
        assert!((scratch[2] - 2.5).abs() < 1e-12, "{}", scratch[2]);
        assert!((scratch[4] - 4.75).abs() < 1e-12, "{}", scratch[4]);
    }

    #[test]
    fn a_supply_trace_with_no_earlier_commit_is_the_straight_line() {
        // Right after a discontinuity `before` is dropped: 1 V at t = 0 to 3 V at t = 2.
        let tr = trace(vec![1], None, 0.0, vec![1.0], 2.0, vec![3.0]);
        let mut scratch = vec![0.0; 3];
        for &(t, want) in &[(0.0, 1.0), (0.5, 1.5), (1.0, 2.0), (1.5, 2.5), (2.0, 3.0)] {
            tr.write(&mut scratch, t);
            assert!((scratch[1] - want).abs() < 1e-15, "t={t}: {} vs {want}", scratch[1]);
        }
    }

    #[test]
    fn a_supply_trace_with_no_trial_is_flat_at_its_committed_value() {
        // `supply_traces` gives a partition with no pending trial t1 = t0 and x1 = x0.
        let tr = trace(vec![1], Some((1.0, vec![9.0])), 3.0, vec![4.2], 3.0, vec![4.2]);
        let mut scratch = vec![0.0; 2];
        for &t in &[0.0, 1.0, 3.0, 3.5, 9.0] {
            tr.write(&mut scratch, t);
            assert_eq!(scratch[1], 4.2, "t={t}: no trial means the supply is where it was committed, whatever `before` says");
        }
    }

    #[test]
    fn a_supply_trace_clamps_to_its_own_interval_instead_of_extrapolating() {
        let q = |t: f64| 0.5 * t * t - 3.0 * t + 7.0;
        let with_before = trace(vec![1], Some((1.0, vec![q(1.0)])), 2.0, vec![q(2.0)], 4.0, vec![q(4.0)]);
        let line = trace(vec![1], None, 2.0, vec![q(2.0)], 4.0, vec![q(4.0)]);
        let mut scratch = vec![0.0; 2];
        for tr in [&with_before, &line] {
            tr.write(&mut scratch, -5.0);
            assert!((scratch[1] - q(2.0)).abs() < 1e-12, "before t0 the value is x0: {}", scratch[1]);
            tr.write(&mut scratch, 1.0);
            assert!((scratch[1] - q(2.0)).abs() < 1e-12, "before t0 the value is x0, not the parabola's own value at t = 1 ({}): {}", q(1.0), scratch[1]);
            tr.write(&mut scratch, 9.0);
            assert!((scratch[1] - q(4.0)).abs() < 1e-12, "after t1 the value is x1, not the parabola's own value at t = 9 ({}): {}", q(9.0), scratch[1]);
        }
    }

    /// Two independent single-capacitor partitions under `FORCE_STEPPED`: node
    /// 2, a 1 ms RC off the 5 V rail (the supplier), and node 3, a 1 s RC off
    /// the same rail (the watcher). The watch on node 3 reads half of node 2,
    /// so node 2 is the supply its threshold follows. Returns the solver and
    /// the two partition indices, both loaded at t = 0.
    fn supplier_and_watcher(depends: bool) -> (SteppedSolver, usize, usize) {
        let mut options = Options::default();
        options.force_stepped = true;
        let p = program(
            4,
            vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 1e3), capacitor(2, 2, 0, 1e-6, 0.0, 0.0), resistor(3, 1, 3, 1e6), capacitor(4, 3, 0, 1e-6, 0.0, 0.0)],
            options,
        );
        let mut st = build(&p);
        assert_eq!(st.partitions.len(), 2, "two capacitors on two separate free nodes: two partitions");
        let (supplier, watcher) = (st.node_owner[&2].0, st.node_owner[&3].0);
        assert_ne!(supplier, watcher);
        let mut voltages = vec![0.0, 5.0, 0.0, 0.0];
        let mut currents = vec![0.0; 5];
        st.resolve_dirty(&[supplier, watcher], 0.0, &[], &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert!(st.watch_node(3, DeviceId(0), Box::new(|v: &[f64]| vec![Threshold { value: 0.5 * v[2], rising: true }])));
        if depends {
            st.add_supply_dependency(3, 2);
        }
        (st, supplier, watcher)
    }

    #[test]
    fn a_step_never_runs_past_the_end_of_a_supply_trial_however_long_its_own_control_would_take() {
        let voltages = vec![0.0, 5.0, 0.0, 0.0];
        let live = HashMap::new();
        // The supplier's trial: a short step of 100 us from t = 0.
        let plan_supplier = |st: &mut SteppedSolver, supplier: usize| {
            st.partitions.get_mut(&supplier).unwrap().h = 1e-4;
            st.plan_step(supplier, f64::INFINITY, &[], &live, &voltages).unwrap();
            let part = &st.partitions[&supplier];
            assert!(part.has_pend && part.pend_t > 0.0 && part.pend_t <= 1e-4 + 1e-15, "the supplier's trial ends within 100 us: {}", part.pend_t);
            part.pend_t
        };
        // The watcher, asked for its longest active step. A 1 s RC takes that whole step in one go.
        let plan_watcher = |st: &mut SteppedSolver, watcher: usize, h: f64| {
            let part = st.partitions.get_mut(&watcher).unwrap();
            part.h = h;
            part.changing = true;
            st.plan_step(watcher, f64::INFINITY, &[], &live, &voltages).unwrap();
            st.partitions[&watcher].pend_t
        };

        // Without the dependency the watcher takes the whole 2 ms: this is the step the cap has to cut.
        let (mut control, supplier, watcher) = supplier_and_watcher(false);
        let control_supplier_end = plan_supplier(&mut control, supplier);
        let control_end = plan_watcher(&mut control, watcher, ACTIVE_H_MAX);
        assert_eq!(control_end, ACTIVE_H_MAX, "the watcher's own control takes the full active step");
        assert!(control_end > 10.0 * control_supplier_end, "and that is far past the supplier's trial ({control_supplier_end})");

        // With it, the step ends where the supplier's trial ends and not a moment later.
        let (mut st, supplier, watcher) = supplier_and_watcher(true);
        let supplier_end = plan_supplier(&mut st, supplier);
        let watcher_end = plan_watcher(&mut st, watcher, ACTIVE_H_MAX);
        assert!(watcher_end <= supplier_end + 1e-15, "watcher pend_t {watcher_end} must not exceed the supplier's {supplier_end}");
        assert!((watcher_end - supplier_end).abs() < 1e-15, "and it is the cap that ends it, not something shorter: {watcher_end} vs {supplier_end}");

        // A step already shorter than the trial is left alone: the cap only lowers.
        let short = plan_watcher(&mut st, watcher, 1e-5);
        assert_eq!(short, 1e-5);
        assert!(short < supplier_end);
    }
}

#[cfg(test)]
thread_local! {
    /// A test's (partition, instant, reach): that partition's plan at that
    /// commit takes the reach from the start, as a stress cut would leave
    /// it (ruling R90 (a)'s equivalence test).
    pub(crate) static FORCE_REACH: std::cell::Cell<Option<(usize, f64, f64)>> = const { std::cell::Cell::new(None) };
}

/// Ruling R90 (a): how many times one plan cuts its trial for a stressed
/// device's ratio; each cut is shorter, and a ratio that keeps falling
/// faster stops here.
const STRESS_CUTS: u32 = 4;

#[cfg(test)]
thread_local! {
    /// Every cut `plan_step` makes for a stressed device: (partition, the
    /// step's start, the trial's h, the cut h).
    pub(crate) static STRESS_CUT_LOG: std::cell::RefCell<Vec<(usize, f64, f64, f64)>> = const { std::cell::RefCell::new(Vec::new()) };
    /// For each cut: the step's start, whether a catch-up or clamp
    /// (`sync_partition`) planned it, and whether it is exact mode's rail
    /// partition (whose plan is deferred to the owner's SoC charge).
    pub(crate) static STRESS_CUT_PATHS: std::cell::RefCell<Vec<(f64, bool, bool)>> = const { std::cell::RefCell::new(Vec::new()) };
}

#[cfg(test)]
thread_local! {
    /// Ruling R90 (a)'s held-IC seam: an IC device index the planner reads
    /// as held, whatever the owner's table says, so a test can hold one IC
    /// on a board whose supply never browns it out.
    pub(crate) static FORCE_HELD: std::cell::Cell<Option<usize>> = const { std::cell::Cell::new(None) };
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Method {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Method::Be => 0,
            Method::Trap => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for StepResult {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.lu, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stage, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for TrBdf2Stage {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.h_g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x_g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i_g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.h_2, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vh, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_g, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Knot {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hermite, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stage, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_i, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for ChargeStep {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.p, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.knots, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vals, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ends_only, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_i0, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PendDense {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.knots, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.solved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_i0, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DenseSeg {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.p, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.knots, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.steady, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.affine, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sens, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.solved, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for CcRail {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dl, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a_all, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g_all, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.own, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cap, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.edges, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for RailCap {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.esr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for CapStep {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.esr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for StressProbe {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            StressProbe::Resistor { a, b, r, r_power, p_max } => crate::snapshot::HeapBytes::heap_bytes(a, w) + crate::snapshot::HeapBytes::heap_bytes(b, w) + crate::snapshot::HeapBytes::heap_bytes(r, w) + crate::snapshot::HeapBytes::heap_bytes(r_power, w) + crate::snapshot::HeapBytes::heap_bytes(p_max, w),
            StressProbe::Fuse { a, b, r_normal, i_rating } => crate::snapshot::HeapBytes::heap_bytes(a, w) + crate::snapshot::HeapBytes::heap_bytes(b, w) + crate::snapshot::HeapBytes::heap_bytes(r_normal, w) + crate::snapshot::HeapBytes::heap_bytes(i_rating, w),
            StressProbe::Led { device, junctions, if_max } => crate::snapshot::HeapBytes::heap_bytes(device, w) + crate::snapshot::HeapBytes::heap_bytes(junctions, w) + crate::snapshot::HeapBytes::heap_bytes(if_max, w),
            StressProbe::Ptc { a, b, params } => crate::snapshot::HeapBytes::heap_bytes(a, w) + crate::snapshot::HeapBytes::heap_bytes(b, w) + crate::snapshot::HeapBytes::heap_bytes(params, w),
        }
    }
}

impl crate::snapshot::HeapBytes for Committed {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.p, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.start, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for CcStep {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dl, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a_o, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g_o, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.coupled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seed, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cv, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cap, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.edge_up, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.edge_dn, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Partition {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shape, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i_prev, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.h, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.changing, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.be_next_step, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trap_off, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_trap_off, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.version, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.has_pend, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_vc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.watches, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.steps_taken, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trap_fallbacks, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.newton_iterations, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.supply_deps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.prev_t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.prev_x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.has_prev, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.end_reads, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stress_reach, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stress_watch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds_free_ic_supply, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.journaled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.on_rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_ic, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sens, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_lu, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_u, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_now, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_h, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_method, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_cc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_charge, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i_src, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_i_src, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_rail_q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.parked, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc_exited, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc_switch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_dense, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_mid, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for StepSens {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_p, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.band, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Snapshot {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.partitions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.scheduled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_charge, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PartitionSnapshot {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i_prev, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.h, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.changing, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.be_next_step, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trap_off, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_trap_off, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.version, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.has_pend, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_vc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.prev_t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.prev_x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.has_prev, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.end_reads, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stress_reach, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stress_watch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sens, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_lu, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_u, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_now, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_h, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_method, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_cc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_charge, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i_src, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_i_src, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_rail_q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_dense, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.pend_mid, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SteppedSolver {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.partitions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_owner, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.method, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.scheduled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ics, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_to_ic_readers, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.threshold_scratch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.window, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.journal, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.spare, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.window_scheduled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.exact, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds_on, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hold_requests, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.defer, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.deferred, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.bills, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cell, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub_updates, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sensitivities, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc_held, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_charge, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.window_rail_charge, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.clamp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.reaim, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stress_probes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trail_parts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trail_reads, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dense_on, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dense_solve, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dense, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_dense, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dense_grid, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.charge_on, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.charge_reads, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.charge_steps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_charge, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.keep_dense, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_saved, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.commit_log, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.resolve_log, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.exact_commit_log, w); }
        n
    }
}

#[cfg(test)]
impl crate::snapshot::HeapBytes for ResolveRecord {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.partition, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.now, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kept, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc, w); }
        n
    }
}

#[cfg(test)]
impl crate::snapshot::HeapBytes for ExactCommit {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mid, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.caps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stamp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trap, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Crossing {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.local, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.watcher, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rising, w); }
        n
    }
}
