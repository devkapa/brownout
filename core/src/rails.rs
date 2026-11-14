//! Hub rails (F1): core/README.md, "Kernel semantics: rails and sources
//! (W1f)", Rails points 2-6. The positive terminal of the one hub source (a
//! `battery_pack`, or a `bench_psu` with `iLimit > 0`) is torn out of every
//! partition and solved here from the partitions' linearised draws, pec3's
//! numerics (P3:437-506) extended with rail events, battery chemistry and the
//! bench PSU's regimes.
//!
//! This module holds the rail's own state and its scalar mathematics: the
//! rail equation with the direct devices iterated inside it, the regime
//! rules, the SoC integral over an event interval and the time of the next
//! `Rail` event. It knows nothing of partitions beyond the contributions they
//! hand it; `solve::PartitionedSolver` owns the wiring.

use std::rc::Rc;

use crate::devices::battery::{exp_term_charge, BatteryModel, BatteryState};
use crate::devices::sources::{compliance_keeps_cv, direction, regime_step, regime_step_rail_cap, Regime};

/// The finite-difference step of the sensitivity (Rails point 3).
pub const RAIL_DELTA: f64 = 1e-6;
/// The cap of a curvature band (an affine closed-form segment's band is not
/// capped), and the band of a curved hub partition without one outside CC
/// (Rails point 3).
pub const RAIL_RESOLVE_V: f64 = 0.025;
/// The rail band in the static cache key.
pub const RAIL_BUCKET_V: f64 = 0.05;
/// Re-queue the readers of hub partitions once the rail has moved this far
/// since the last scan.
pub const RAIL_SCAN_V: f64 = 0.02;
/// A smaller change of `u` is not a move (P3:468).
pub const RAIL_MOVE_EPS: f64 = 1e-12;
/// The implied rail motion that schedules a rail event (Rails point 4).
pub const RAIL_DRIFT_V: f64 = 1e-5;
/// The re-solve band while a PSU is in CC with no rail capacitor.
pub const RAIL_CC_RESOLVE_V: f64 = 1e-6;
/// The re-solve band while a PSU ramps a rail capacitor in CC (Rails point
/// 3; F1v23 finding 3, ruled 2026-10-15), now only for a curved partition
/// without a band of its own, which none is since every curved kind,
/// a free IC supply included (ruling R7 (a)), sizes one.
pub const RAIL_CC_CAP_RESOLVE_V: f64 = 1e-3;

/// How far the rail may move from a nonlinear hub partition's solve before
/// it solves again (Rails point 3).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BandRule {
    /// Every nonlinear hub partition alike.
    Flat(f64),
    /// Each partition's own curvature band (`solve::curvature`), or
    /// `fallback` for one that has none.
    Curvature { fallback: f64 },
}

impl BandRule {
    /// The band of a partition whose solve gave `curvature`.
    pub fn band(self, curvature: Option<f64>) -> f64 {
        match self {
            BandRule::Flat(b) => b,
            BandRule::Curvature { fallback } => curvature.unwrap_or(fallback),
        }
    }
}

/// The largest battery rail time constant `r * C_rail` the core accepts.
pub const RAIL_TAU_MAX: f64 = 1e-4;
/// The scalar Newton over the direct devices (Rails point 2).
pub const RAIL_NEWTON_TOL: f64 = 1e-12;
pub const RAIL_NEWTON_MAX: u32 = 50;
/// The rail loop's pass cap (Rails point 3).
pub const RAIL_LOOP_MAX: u32 = 50;

/// The error of a rail loop that did not settle within `RAIL_LOOP_MAX`
/// passes (and could not hold a starving IC off, W1c point 1a), and of the
/// no-progress guard's crawl across instants.
pub fn loop_unsettled() -> String {
    format!("rail: the rail loop did not settle within {RAIL_LOOP_MAX} passes")
}
/// The running sums are recomputed from scratch this often (P3:465).
pub const RECOMPUTE_EVERY: u64 = 4096;
/// A settle pass whose rail moved more than this changed the fixed point
/// (Sources point 4, P3:746).
pub const SETTLE_MOVE_V: f64 = 1e-9;
/// The reached-width `eta = REACHED_REL * max(1, |threshold|)` (Rails point
/// 4): a watch aims a threshold two widths past it, on the far side, and
/// counts it reached within one width of that aim, so at `eta` or more past
/// the threshold. A PSU's capacitor demand re-enters CC only past `iLimit +
/// REACHED_REL * max(1, iLimit)` (Rails point 5).
pub const REACHED_REL: f64 = 1e-12;
/// The CC rail with a rail capacitor is a straight line while `G (t - t0) /
/// C_rail` is below this (Rails point 4).
pub const RAIL_CAP_LINEAR: f64 = 1e-9;

/// The hub source.
#[derive(Debug, Clone)]
pub enum HubKind {
    Battery(BatteryModel),
    Psu { v: f64, i_limit: f64 },
}

/// One rail capacitor: a capacitor with one pin on the rail and the other on
/// a fixed node (Rails point 7).
#[derive(Debug, Clone, Copy)]
pub struct RailCap {
    pub device: usize,
    pub c: f64,
}

/// What a hub partition hands the rail after a solve (Rails point 2): its
/// draw `i` from the rail at `u_p`, the slope `g`, and for a closed-form
/// partition the coefficient `b` of its capacitor voltage, so its draw at any
/// instant is `i + b vc(t) + g (u - u_p)`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Contrib {
    pub active: bool,
    pub i: f64,
    pub g: f64,
    pub u_p: f64,
    /// `Some(b)` for a closed-form partition.
    pub b: Option<f64>,
}

/// A closed-form hub partition's draw over an interval, at its start `t0`:
/// `a + b vc(t)` with `vc(t) = inf + (vc0 - inf) e^(f (t - t0)) + w J(f, 0,
/// t - t0)`, and the `g (u - u_p)` term folded into `a` at the rail of `t0`.
/// `w` is its capacitor's share of a draining cell's line (`CellDrive`), 0
/// for any other.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CfTerm {
    pub a: f64,
    pub b: f64,
    pub vc0: f64,
    pub inf: f64,
    pub f: f64,
    pub w: f64,
}

impl CfTerm {
    pub fn vc_after(&self, dt: f64) -> f64 {
        let vc = if self.f == 0.0 { self.vc0 } else { self.inf + (self.vc0 - self.inf) * (self.f * dt).exp() };
        if self.w != 0.0 {
            vc + self.w * ramp_response(self.f, 0.0, dt)
        } else {
            vc
        }
    }

    pub fn current_after(&self, dt: f64) -> f64 {
        self.a + self.b * self.vc_after(dt)
    }

    /// The exact charge over `dt` (Rails point 6).
    pub fn charge(&self, dt: f64) -> f64 {
        let q = exp_term_charge(self.a, self.b, self.inf, self.vc0, self.f, dt);
        if self.w != 0.0 {
            q + self.b * self.w * ramp_charge(self.f, 0.0, dt)
        } else {
            q
        }
    }
}

/// A draining cell's rail as a closed-form hub partition follows it between
/// solves (Rails point 4, ruling R15 (a)): `u = u_c(t) + kappa sum b vc` with
/// `kappa = -r / (1 + r G)`, the cell's resistance `r` against the draws'
/// slope `G`, and `u_c` moving at `rho`, the drain's line (the SoC term of the
/// implied trajectory). With every coupling of one sign and stable, the
/// partitions follow the cell's coupled path built from it (`cell_modes`,
/// ruling R16 (B)); otherwise, the fallback, a partition takes its own
/// `kappa b vc` into its rate exactly and the line as a drive, and the
/// others' terms stay first order. `t0` is the solve's instant: a solve at
/// another one is another drive, and the partitions aim again under it, the
/// line's horizon with them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CellDrive {
    pub t0: f64,
    pub kappa: f64,
    pub rho: f64,
}

/// The CC rail with a rail capacitor (Rails point 4): `u(t) = u_inf + (u0 -
/// u_inf) e^(-k (t - t0))`, and the straight line `u0 + slope (t - t0)` while
/// `k (t - t0) < RAIL_CAP_LINEAR`.
///
/// The state is the capacitors' C-weighted charge `q`, `C_rail dq/dt =
/// I_cap`, and the rail sits `esr_eff I_cap` above it, `I_cap = dir iLimit -
/// (A0 + G u)`. Both are linear in `u`, so `u = alpha q + beta` with `alpha
/// = 1 / (1 + G esr_eff)` and `beta = alpha esr_eff (dir iLimit - A0)`, and
/// `u` is the same exponential as `q`: `k = alpha G / C_rail`. At `esr_eff =
/// 0`, `alpha = 1` and `beta = 0` exactly, so `q` is `u` bit for bit.
///
/// A running ramp that moves the direct devices' draws (a pulse source's
/// edge through a switch onto the rail) moves `A0` at `adot` and the
/// asymptote with it, at `ramp = -adot / G` (R99 (3)). Then `q(τ) = q0 +
/// (slope / alpha - ramp) φ_k(τ) + ramp τ` and `u = alpha q + beta + (1 -
/// alpha) ramp τ`, so `u(τ) = u0 + (slope - alpha ramp) φ_k(τ) + ramp τ`:
/// the exponential settles onto a line the rail follows, not a level it
/// lags. Held frozen instead, cold 2575 tpl 2's rail lagged its 12 kV/s
/// ramp by up to 19 mV between rail solves, as far as a host stop happened
/// to land from the last one.
///
/// With closed-form hub partitions on the rail the segment is their coupled
/// path instead (`modes`, ruling R15 (a)): the rail and their capacitors
/// advance together, so nothing is frozen and nothing is owed at the next
/// solve. The single exponential and the frozen draws remain for a rail with
/// none, and for a coupling the modal form cannot take (`coupled_modes`).
#[derive(Debug, Clone, PartialEq)]
pub struct CapSeg {
    pub t0: f64,
    pub u0: f64,
    pub u_inf: f64,
    pub k: f64,
    /// `alpha (dir iLimit - A0 - G u0) / C_rail`.
    pub slope: f64,
    pub alpha: f64,
    pub beta: f64,
    /// The asymptote's rate under a running ramp, `-adot / G` (V/s); 0
    /// without one. Not with `modes`.
    pub ramp: f64,
    /// `sum b vc` of the closed-form hub partitions at `t0`: the part of
    /// their draws the segment holds where it stood, while their capacitors
    /// move on, so the next solve charges the difference. 0 with `modes`.
    pub cf_frozen: f64,
    /// How many partitions `cf_frozen` sums.
    pub cf_count: u32,
    /// The coupled path, when the segment is one.
    pub modes: Option<Rc<RailModes>>,
}

impl CapSeg {
    /// The capacitors' C-weighted charge, as a rail voltage: continuous where
    /// `u` steps with the draws through the esr offset.
    pub fn charge_at(&self, t: f64) -> f64 {
        match &self.modes {
            Some(m) => m.charge(self.u0, t - self.t0),
            None if self.ramp != 0.0 => (self.at(t) - self.beta - (1.0 - self.alpha) * self.ramp * (t - self.t0)) / self.alpha,
            None => (self.at(t) - self.beta) / self.alpha,
        }
    }

    /// `dq/dt`, so that `C_k dq/dt` is capacitor `k`'s share of `I_cap`.
    pub fn charge_slope_at(&self, t: f64) -> f64 {
        match &self.modes {
            Some(m) => m.charge_slope(t - self.t0),
            None if self.ramp != 0.0 => (self.slope_at(t) - (1.0 - self.alpha) * self.ramp) / self.alpha,
            None => self.slope_at(t) / self.alpha,
        }
    }

    /// The exponential is taken as its increment from `u0`, `slope ∫0^dt
    /// e^(-k s) ds`: `u_inf + (u0 - u_inf) e^(-k dt)` rounds at the scale of
    /// `u_inf`, which on a lightly loaded rail is millions of volts (`iLimit
    /// / G`), so its last bit there is nanovolts of rail.
    pub fn at(&self, t: f64) -> f64 {
        let dt = t - self.t0;
        if let Some(m) = &self.modes {
            return self.u0 + m.du(dt);
        }
        if self.ramp != 0.0 {
            let phi = if self.k * dt < RAIL_CAP_LINEAR { dt } else { exp_integral(-self.k, dt) };
            return self.u0 + (self.slope - self.alpha * self.ramp) * phi + self.ramp * dt;
        }
        if self.k * dt < RAIL_CAP_LINEAR {
            self.u0 + self.slope * dt
        } else {
            self.u0 + self.slope * exp_integral(-self.k, dt)
        }
    }

    /// W2b piece 4: the rail's mean over `[a, b]`, `∫ at / (b - a)`: each
    /// exponential's integral `χ_k`, or the line's midpoint where
    /// [`Self::at`] takes the line.
    pub fn mean(&self, a: f64, b: f64) -> f64 {
        let (ta, tb) = (a - self.t0, b - self.t0);
        let dt = b - a;
        if let Some(m) = &self.modes {
            return self.u0 + m.modes.iter().map(|&(k, s)| s * (phi_integral(k, tb) - phi_integral(k, ta))).sum::<f64>() / dt;
        }
        if self.ramp != 0.0 {
            let phi = if self.k * tb < RAIL_CAP_LINEAR { 0.5 * (ta + tb) } else { (phi_integral(self.k, tb) - phi_integral(self.k, ta)) / dt };
            return self.u0 + (self.slope - self.alpha * self.ramp) * phi + self.ramp * 0.5 * (ta + tb);
        }
        if self.k * tb < RAIL_CAP_LINEAR {
            self.u0 + self.slope * 0.5 * (ta + tb)
        } else {
            self.u0 + self.slope * (phi_integral(self.k, tb) - phi_integral(self.k, ta)) / dt
        }
    }

    pub fn slope_at(&self, t: f64) -> f64 {
        let dt = t - self.t0;
        if let Some(m) = &self.modes {
            return m.slope(dt);
        }
        if self.ramp != 0.0 {
            let e = if self.k * dt < RAIL_CAP_LINEAR { 1.0 } else { (-self.k * dt).exp() };
            return (self.slope - self.alpha * self.ramp) * e + self.ramp;
        }
        if self.k * dt < RAIL_CAP_LINEAR {
            self.slope
        } else {
            self.slope * (-self.k * dt).exp()
        }
    }

    /// The rail's motion at `t`, as the modes a partition aimed then is
    /// driven by: `(k, s)` with `u(t + τ) = u(t) + sum s φ_k(τ)`.
    pub fn modes_at(&self, t: f64) -> Vec<(f64, f64)> {
        match &self.modes {
            Some(m) => {
                let dt = t - self.t0;
                m.modes.iter().map(|&(k, s)| (k, s * (-k * dt).exp())).collect()
            }
            None if self.ramp != 0.0 => vec![(self.k, self.slope_at(t) - self.ramp), (0.0, self.ramp)],
            None => vec![(self.k, self.slope_at(t))],
        }
    }

    /// The segment's own motion as a path a watch takes its passages from,
    /// when a ramp moves it (R99 (3)): an exponential onto a line, which
    /// turns where the two oppose, so [`Motion::Seg`]'s one-way log does not
    /// hold for it.
    pub fn ramp_path(&self) -> Option<RailModes> {
        (self.modes.is_none() && self.ramp != 0.0).then(|| RailModes::new(vec![(self.k, self.slope - self.alpha * self.ramp), (0.0, self.ramp)], self.charge_at(self.t0), None))
    }
}

// ---- the CC rail capacitor coupled to its partitions (ruling R15 (a)) ------

/// A closed-form hub partition as the CC rail capacitor's coupled path takes
/// it, linearised at the aim it is about to take (Rails point 4): its
/// capacitor's rate `f = di / C`, how the rail drives it, `gamma = (di0/du +
/// di1du vc) / C`, how its capacitor moves its draw, `b` (with `g1 (u -
/// u_seg)`), and its capacitor's rate of change there, `r0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coupled {
    pub f: f64,
    pub gamma: f64,
    pub b: f64,
    pub r0: f64,
}

/// The CC rail capacitor's segment coupled to its closed-form hub
/// partitions (ruling R15 (a)): `u(t0 + τ) = u0 + sum s φ_k(τ)` over the
/// coupled system's modes, and the capacitors' charge, which with `esr_eff >
/// 0` lags the rail through the esr pole: `q(t0 + τ) = q0 + r0_q φ_ke(τ) +
/// ce sum s J(-ke, k, τ)`, `(ke, r0_q, ce)` in `esr`. At esr 0 `q` is `u`.
#[derive(Debug, Clone)]
pub struct RailModes {
    pub modes: Vec<(f64, f64)>,
    pub q0: f64,
    pub esr: Option<(f64, f64, f64)>,
    /// The last [`Self::passages`] asked of this path, `(a, b)` by their
    /// bits and its answer: a draining cell's rail solve asks for the drift
    /// cap each way, and the event it schedules on the same path asks again
    /// (BAT-PERF). A function of the fields above, so no part of equality.
    passages_memo: std::cell::Cell<Option<PassagesMemo>>,
}

type PassagesMemo = (Option<u64>, Option<u64>, f64, f64);

impl PartialEq for RailModes {
    fn eq(&self, other: &Self) -> bool {
        self.modes == other.modes && self.q0 == other.q0 && self.esr == other.esr
    }
}

impl RailModes {
    pub fn new(modes: Vec<(f64, f64)>, q0: f64, esr: Option<(f64, f64, f64)>) -> Self {
        RailModes { modes, q0, esr, passages_memo: std::cell::Cell::new(None) }
    }

    /// `u(t0 + τ) - u0`.
    pub fn du(&self, tau: f64) -> f64 {
        let mut it = self.modes.iter();
        let &(k, s) = it.next().expect("a path has a mode");
        let mut d = s * exp_integral(-k, tau);
        for &(k, s) in it {
            d += s * exp_integral(-k, tau);
        }
        d
    }

    pub fn slope(&self, tau: f64) -> f64 {
        self.modes.iter().map(|&(k, s)| s * (-k * tau).exp()).sum()
    }

    pub fn charge(&self, u0: f64, tau: f64) -> f64 {
        match self.esr {
            Some((ke, r0q, ce)) => self.q0 + r0q * exp_integral(-ke, tau) + ce * self.modes.iter().map(|&(k, s)| s * ramp_response(-ke, k, tau)).sum::<f64>(),
            None => u0 + self.du(tau),
        }
    }

    pub fn charge_slope(&self, tau: f64) -> f64 {
        match self.esr {
            Some((ke, r0q, ce)) => r0q * (-ke * tau).exp() + ce * self.modes.iter().map(|&(k, s)| s * ramp_rate(-ke, k, tau)).sum::<f64>(),
            None => self.slope(tau),
        }
    }

    /// The first `τ > 0` at which the rail has moved by `d` (either sign),
    /// or infinity.
    pub fn passage(&self, d: f64) -> f64 {
        self.passages(Some(d), None).0
    }

    /// [`Self::passage`] of `a` and of `b` (infinity for `None`), as two
    /// calls give: the sums differ only in `c0`, so their turning points are
    /// found once. Every caller wants a move each way, at each rail event.
    pub fn passages(&self, a: Option<f64>, b: Option<f64>) -> (f64, f64) {
        let key = (a.map(f64::to_bits), b.map(f64::to_bits));
        if let Some((ka, kb, up, down)) = self.passages_memo.get() {
            if (ka, kb) == key {
                return (up, down);
            }
        }
        let found = self.passages_fresh(a, b);
        self.passages_memo.set(Some((key.0, key.1, found.0, found.1)));
        found
    }

    fn passages_fresh(&self, a: Option<f64>, b: Option<f64>) -> (f64, f64) {
        let terms: Vec<(f64, Basis)> = self.modes.iter().map(|&(k, s)| (s, Basis::Phi(k))).collect();
        let mut zeros = None;
        let mut one = |d: Option<f64>| match d {
            None => f64::INFINITY,
            Some(d) if d == 0.0 => 0.0,
            Some(d) => ExpSum { c0: -d, terms: &terms }.first_crossing_sharing(d > 0.0, &mut zeros).unwrap_or(f64::INFINITY),
        };
        let pa = one(a);
        (pa, one(b))
    }
}

/// The secular function of the coupled CC rail (`coupled_modes`): `F(λ) =
/// sum w / (λ - f) - m(λ)`, evaluated at `λ = origin + delta` with each
/// difference taken from the origin, so a root near a pole keeps its
/// distance from it to the last bit.
struct Secular<'a> {
    /// `(f, w, b r0)` per distinct rate.
    poles: &'a [(f64, f64, f64)],
    esr: f64,
    c_rail: f64,
    alpha: f64,
    k_q: f64,
    lam_e: f64,
}

impl Secular<'_> {
    fn at(&self, origin: f64, delta: f64) -> f64 {
        let mut w = 0.0;
        for &(f, wt, _) in self.poles {
            if wt > 0.0 {
                w += wt / ((origin - f) + delta);
            }
        }
        w - self.m(origin, delta)
    }

    /// `m(λ) = (λ + k_q) / (alpha Q(λ))`, `Q` the esr factor below.
    fn m(&self, origin: f64, delta: f64) -> f64 {
        let lk = (origin + self.k_q) + delta;
        if self.esr > 0.0 {
            lk / (self.alpha * self.esr * ((origin - self.lam_e) + delta))
        } else {
            self.c_rail * lk / (self.alpha * self.alpha)
        }
    }

    /// `Q(λ) = (alpha + esr C_rail (λ + k_q)) / C_rail`.
    fn q(&self, origin: f64, delta: f64) -> f64 {
        if self.esr > 0.0 {
            self.esr * ((origin - self.lam_e) + delta)
        } else {
            self.alpha / self.c_rail
        }
    }

    /// The root of the decreasing `F` between `left` (where it is positive,
    /// or a pole) and `right` (negative, or a pole), as `(origin, delta)`
    /// from the nearer end.
    fn root(&self, left: f64, right: f64) -> Option<(f64, f64)> {
        let half = 0.5 * (right - left);
        if !(half > 0.0) {
            return None;
        }
        let (origin, mut lo, mut hi) = if self.at(left, half) > 0.0 { (right, -half, 0.0) } else { (left, 0.0, half) };
        for _ in 0..2200 {
            let mid = if lo == 0.0 {
                0.5 * hi
            } else if hi == 0.0 {
                0.5 * lo
            } else if lo > 0.0 && hi > 4.0 * lo {
                (lo * hi).sqrt()
            } else if hi < 0.0 && lo < 4.0 * hi {
                -(lo * hi).sqrt()
            } else {
                lo + 0.5 * (hi - lo)
            };
            if !(mid > lo && mid < hi) {
                break;
            }
            if self.at(origin, mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let delta = if lo == 0.0 { hi } else if hi == 0.0 { lo } else { lo + 0.5 * (hi - lo) };
        Some((origin, delta))
    }
}

/// The modes of the CC rail capacitor coupled to the closed-form hub
/// partitions `parts` (ruling R15 (a), Rails point 4), as `(k, s)`: `u(t0 +
/// τ) - u0 = sum s φ_k(τ)`, the exact solution of the linear system the rail
/// and their capacitors form at the solve, or `None` where that system is
/// not one the modal form takes (a partition with `w = -b gamma < 0`, a
/// capacitor that does not decay, an unstable or degenerate mode).
///
/// The system: `C_rail q' = I_cap = dir iLimit - A0 - G u - sum b (vc -
/// vc_a)`, `u = alpha (q + esr_eff (dir iLimit - A0 - sum b vc))`, and each
/// `vc' = f (vc - vc_a) + gamma (u - u0) + r0`; `k_q = alpha G / C_rail`,
/// `r0_q = q'(t0)`. Eliminating the partitions, the rail's transform is
/// `U(s) = R(s) / s`, `R = (r0_q - B Q) / ((s + k_q) / alpha - W Q)`, with `W(s)
/// = sum w / (s - f)`, `B(s) = sum b r0 / (s - f)` and `Q(s) = (alpha +
/// esr_eff C_rail (s + k_q)) / C_rail`. The modes are the zeros of `F = W -
/// m`, `m = (s + k_q) / (alpha Q)`: `W` falls between its poles and `m`
/// rises (with a pole of its own at `lam_e = -k_q - alpha / (esr_eff
/// C_rail)`), so exactly one root lies between each two consecutive poles,
/// one above the last (below 0 while the rail is stable, `F(0) < 0`), and at
/// esr 0 one below the first: as many as the system has states. Each mode's
/// share is the residue `s = (B - r0_q / Q) / F'`. A partition that moves its
/// draw but follows no rail (`w = 0`, `b r0 != 0`) is a forcing at its own
/// rate, its share `b r0 / F(f)`. The shares sum to the rail's slope,
/// `alpha (r0_q - esr_eff sum b r0)`, which is checked.
///
/// The partitions follow the rail by their own closed forms (each its
/// exponential and `gamma s J(f, k, τ)` per mode), which is the coupled
/// solution's partition part: a linear partition is exact to rounding, and
/// what is left is second order, a curved partition's bilinear `di1du (u -
/// u_a) (vc - vc_a)` (bounded by `couple_band`) and the linearisation of the
/// direct devices and static partitions at the solve, as before.
pub fn coupled_modes(c_rail: f64, esr: f64, alpha: f64, k_q: f64, r0_q: f64, parts: &[Coupled]) -> Option<Vec<(f64, f64)>> {
    if !(c_rail > 0.0 && alpha > 0.0 && esr >= 0.0 && k_q >= 0.0 && k_q.is_finite() && r0_q.is_finite()) {
        return None;
    }
    let mut poles: Vec<(f64, f64, f64)> = Vec::with_capacity(parts.len());
    let mut br_total = 0.0;
    for p in parts {
        let w = -p.b * p.gamma;
        let br = p.b * p.r0;
        if !(p.f < 0.0 && p.f.is_finite() && w >= 0.0 && w.is_finite() && br.is_finite()) {
            return None;
        }
        br_total += br;
        match poles.iter_mut().find(|e| e.0 == p.f) {
            Some(e) => {
                e.1 += w;
                e.2 += br;
            }
            None => poles.push((p.f, w, br)),
        }
    }
    poles.sort_by(|a, b| a.0.total_cmp(&b.0));
    let lam_e = if esr > 0.0 { -k_q - alpha / (esr * c_rail) } else { f64::NEG_INFINITY };
    let sec = Secular { poles: &poles, esr, c_rail, alpha, k_q, lam_e };
    let mut cuts: Vec<f64> = poles.iter().filter(|e| e.1 > 0.0).map(|e| e.0).collect();
    if esr > 0.0 {
        if !lam_e.is_finite() || cuts.contains(&lam_e) {
            return None;
        }
        cuts.push(lam_e);
        cuts.sort_by(f64::total_cmp);
    }
    if !(sec.at(0.0, 0.0) < 0.0) {
        return None;
    }
    let mut roots: Vec<(f64, f64)> = Vec::with_capacity(cuts.len() + 1);
    if cuts.is_empty() {
        roots.push((-k_q, 0.0));
    } else {
        if esr == 0.0 {
            let p = cuts[0];
            let mut d = -p.abs().max(1.0);
            let mut n = 0;
            while !(sec.at(p, d) > 0.0) {
                d *= 2.0;
                n += 1;
                if n > 2100 || !d.is_finite() {
                    return None;
                }
            }
            roots.push(sec.root(p + d, p)?);
        }
        for w in cuts.windows(2) {
            roots.push(sec.root(w[0], w[1])?);
        }
        roots.push(sec.root(*cuts.last().unwrap(), 0.0)?);
    }
    let mut out = Vec::with_capacity(roots.len() + poles.len());
    for &(origin, delta) in &roots {
        let (mut db, mut dw) = (0.0, 0.0);
        for &(f, w, br) in &poles {
            let dif = (origin - f) + delta;
            db += br / dif;
            if w > 0.0 {
                dw += w / (dif * dif);
            }
        }
        let q = sec.q(origin, delta);
        let fp = -dw - 1.0 / (c_rail * q * q);
        let s = (db - r0_q / q) / fp;
        out.push((-(origin + delta), s));
    }
    for &(f, w, br) in &poles {
        if w == 0.0 && br != 0.0 {
            let ff = sec.at(f, 0.0);
            if !(ff != 0.0 && ff.is_finite()) {
                return None;
            }
            out.push((-f, br / ff));
        }
    }
    let slope = alpha * (r0_q - esr * br_total);
    let (sum, size) = out.iter().fold((0.0, slope.abs()), |(a, m), &(_, s)| (a + s, m + s.abs()));
    if !out.iter().all(|&(k, s)| k > 0.0 && k.is_finite() && s.is_finite()) || !((sum - slope).abs() <= 1e-9 * size) {
        return None;
    }
    Some(out)
}

/// The draining cell's rail coupled to its closed-form hub partitions `parts`
/// (ruling R16 (B), Rails point 4), as modes `(k, s)` of its path from the
/// solve: `u(t0 + τ) - u0 = sum s φ_k(τ)`, or `None` where the coupling is
/// not one the modal form takes.
///
/// The cell's rail is algebraic: `u = u0 + rho τ + kappa sum b e`, `e = vc -
/// vc(t0)`, `kappa = -r / (1 + r G)` and `rho` the drain's line
/// (`CellDrive`), and each partition, linearised at the aim it takes at the
/// solve ([`Coupled`]), `e' = f e + gamma (u - u0) + r0`. So the partitions
/// are a diagonal system with a rank-one coupling, `M = F + kappa gamma
/// b^T`, and the rail's slope transforms to `(rho / s + kappa B(s)) / D(s)`,
/// `D = 1 - sum c / (s - f)`, `c = kappa b gamma`, `B = sum b r0 / (s - f)`.
/// With every `c` of one sign `sum c / (s - f)` is monotone between its
/// poles, so `D` has one zero between each two consecutive poles and one
/// beyond them (above the last for `c > 0`, which is stable only while
/// `D(0) > 0`; below the first for `c < 0`): as many as the partitions
/// coupled, each found from its nearer pole so a weakly coupled partition's
/// mode keeps its distance from its own rate to the last bit. Each mode's
/// share is its residue, `s = (rho / λ + kappa B(λ)) / D'(λ)`; the drain
/// adds the line `rho / D(0)`, and a partition that moves its draw but
/// follows no rail (`c = 0`) its own rate, `kappa b r0 / D(f)`. The shares
/// sum to the rail's slope, `rho + kappa sum b r0`, which is checked.
///
/// The partitions follow the path by their own closed forms (the coupled
/// solution's partition part, as under a CC rail capacitor's path), each at
/// its own rate `f` with its own pull on the cell in the path.
pub fn cell_modes(kappa: f64, rho: f64, parts: &[Coupled]) -> Option<Vec<(f64, f64)>> {
    if !(kappa <= 0.0 && kappa.is_finite() && rho.is_finite()) {
        return None;
    }
    // `(f, c, b r0)` per distinct rate.
    let mut poles: Vec<(f64, f64, f64)> = Vec::with_capacity(parts.len());
    let mut br_total = 0.0;
    for p in parts {
        let (c, br) = (kappa * p.b * p.gamma, p.b * p.r0);
        if !(p.f < 0.0 && p.f.is_finite() && c.is_finite() && br.is_finite()) {
            return None;
        }
        br_total += br;
        match poles.iter_mut().find(|e| e.0 == p.f) {
            Some(e) => {
                e.1 += c;
                e.2 += br;
            }
            None => poles.push((p.f, c, br)),
        }
    }
    poles.sort_by(|a, b| a.0.total_cmp(&b.0));
    let sign = poles.iter().map(|e| e.1).find(|&c| c != 0.0).map_or(1.0, f64::signum);
    if poles.iter().any(|e| e.1 * sign < 0.0) {
        return None;
    }
    // `D` at `origin + delta`, each difference from the origin.
    let d_at = |origin: f64, delta: f64| 1.0 - poles.iter().filter(|e| e.1 != 0.0).map(|&(f, c, _)| c / ((origin - f) + delta)).sum::<f64>();
    let dd_at = |origin: f64, delta: f64| {
        poles.iter().filter(|e| e.1 != 0.0).map(|&(f, c, _)| {
            let x = (origin - f) + delta;
            c / (x * x)
        }).sum::<f64>()
    };
    let cuts: Vec<f64> = poles.iter().filter(|e| e.1 != 0.0).map(|e| e.0).collect();
    let mut roots: Vec<(f64, f64)> = Vec::with_capacity(cuts.len());
    if !cuts.is_empty() {
        if sign > 0.0 {
            if !(d_at(0.0, 0.0) > 0.0) {
                return None;
            }
            for w in cuts.windows(2) {
                roots.push(cell_root(&d_at, &dd_at, w[0], w[1], true)?);
            }
            // Above the last pole `D` rises from minus infinity to `D(0) > 0`.
            roots.push(cell_root(&d_at, &dd_at, *cuts.last().unwrap(), 0.0, true)?);
        } else {
            // Below the first pole `D` falls from 1 to minus infinity.
            let p = cuts[0];
            let mut d = -p.abs().max(1.0);
            let mut n = 0;
            while !(d_at(p, d) > 0.0) {
                d *= 2.0;
                n += 1;
                if n > 2100 || !d.is_finite() {
                    return None;
                }
            }
            roots.push(cell_root(&d_at, &dd_at, p + d, p, false)?);
            for w in cuts.windows(2) {
                roots.push(cell_root(&d_at, &dd_at, w[0], w[1], false)?);
            }
        }
    }
    let mut out = Vec::with_capacity(roots.len() + 2);
    for &(origin, delta) in &roots {
        let lam = origin + delta;
        let b: f64 = poles.iter().map(|&(f, _, br)| br / ((origin - f) + delta)).sum();
        let s = (rho / lam + kappa * b) / dd_at(origin, delta);
        out.push((-lam, s));
    }
    if rho != 0.0 {
        let d0 = d_at(0.0, 0.0);
        if !(d0 != 0.0 && d0.is_finite()) {
            return None;
        }
        out.push((0.0, rho / d0));
    }
    for &(f, c, br) in &poles {
        if c == 0.0 && br != 0.0 {
            let df = d_at(f, 0.0);
            if !(df != 0.0 && df.is_finite()) {
                return None;
            }
            out.push((-f, kappa * br / df));
        }
    }
    let slope = rho + kappa * br_total;
    let (sum, size) = out.iter().fold((0.0, slope.abs()), |(a, m), &(_, s)| (a + s, m + s.abs()));
    if out.is_empty() || !out.iter().all(|&(k, s)| k >= 0.0 && k.is_finite() && s.is_finite()) || !((sum - slope).abs() <= 1e-9 * size) {
        return None;
    }
    Some(out)
}

/// The zero of `D` ([`cell_modes`]) between `left` and `right` (poles, a
/// finite point below the first pole, or `right = 0` past the last), on
/// which `D` rises (`inc`) or falls monotonically, as `(origin, delta)` from
/// the nearer end: Newton on `delta` kept inside the bracket, halving
/// (geometrically across scales) where it steps out, so a root a pole's
/// width from its pole costs a few steps rather than a bisection to the last
/// bit.
fn cell_root(d_at: &dyn Fn(f64, f64) -> f64, dd_at: &dyn Fn(f64, f64) -> f64, left: f64, right: f64, inc: bool) -> Option<(f64, f64)> {
    let half = 0.5 * (right - left);
    if !(half > 0.0) {
        return None;
    }
    // Left of the root `D` is below 0 where it rises, above where it falls.
    let before = |v: f64| if inc { v < 0.0 } else { v > 0.0 };
    let (origin, mut lo, mut hi) = if before(d_at(left, half)) { (right, -half, 0.0) } else { (left, 0.0, half) };
    let mut t = if lo == 0.0 { 0.5 * hi } else { 0.5 * lo };
    for _ in 0..400 {
        if !(t > lo && t < hi) {
            t = if lo == 0.0 {
                0.5 * hi
            } else if hi == 0.0 {
                0.5 * lo
            } else if lo > 0.0 && hi > 4.0 * lo {
                (lo * hi).sqrt()
            } else if hi < 0.0 && lo < 4.0 * hi {
                -(lo * hi).sqrt()
            } else {
                lo + 0.5 * (hi - lo)
            };
            if !(t > lo && t < hi) {
                break;
            }
        }
        let v = d_at(origin, t);
        if v == 0.0 {
            return Some((origin, t));
        }
        if before(v) {
            lo = t;
        } else {
            hi = t;
        }
        let dv = dd_at(origin, t);
        let next = if dv != 0.0 { t - v / dv } else { f64::NAN };
        if (next - t).abs() <= 4.0 * f64::EPSILON * t.abs() {
            return Some((origin, t));
        }
        t = next;
    }
    let delta = if lo == 0.0 { hi } else if hi == 0.0 { lo } else { lo + 0.5 * (hi - lo) };
    Some((origin, delta))
}

// ---- first crossings of sums of monotone terms ----------------------------

/// A basis function of `τ >= 0`, each monotone there.
#[derive(Debug, Clone, Copy)]
pub enum Basis {
    /// `e^(f τ)`, `f < 0`: falls from 1 to 0.
    Exp(f64),
    /// `φ_k(τ) = ∫0^τ e^(-k r) dr`, `k >= 0`: rises.
    Phi(f64),
    /// `J(f, k, τ)` ([`ramp_response`]), `f < 0 <= k`: rises.
    J(f64, f64),
}

impl Basis {
    fn at(self, tau: f64) -> f64 {
        match self {
            Basis::Exp(f) => (f * tau).exp(),
            Basis::Phi(k) => exp_integral(-k, tau),
            Basis::J(f, k) => ramp_response(f, k, tau),
        }
    }

    fn rate(self, tau: f64) -> f64 {
        match self {
            Basis::Exp(f) => f * (f * tau).exp(),
            Basis::Phi(k) => (-k * tau).exp(),
            Basis::J(f, k) => ramp_rate(f, k, tau),
        }
    }

    /// Its value as `τ` grows without bound.
    fn at_inf(self) -> f64 {
        match self {
            Basis::Exp(_) => 0.0,
            Basis::Phi(k) => {
                if k > 0.0 {
                    1.0 / k
                } else {
                    f64::INFINITY
                }
            }
            Basis::J(f, k) => {
                if k > 0.0 {
                    1.0 / (k * -f)
                } else {
                    f64::INFINITY
                }
            }
        }
    }

    /// Bounds on its rate over `[a, b]`: `e^(f τ)` and `e^(-k τ)` are
    /// monotone, and `J' = ∫0^τ e^(f (τ - s)) e^(-k s) ds` lies between
    /// `e^(-k b) (1 - e^(f a)) / -f` and the least of `(1 - e^(f b)) / -f`
    /// and `φ_k(b)`.
    fn rate_bounds(self, a: f64, b: f64) -> (f64, f64) {
        match self {
            Basis::Exp(f) => (f * (f * a).exp(), f * (f * b).exp()),
            Basis::Phi(k) => ((-k * b).exp(), (-k * a).exp()),
            Basis::J(f, k) => {
                let lo = (-k * b).exp() * -(f * a).exp_m1() / -f;
                let hi = (-(f * b).exp_m1() / -f).min(exp_integral(-k, b));
                (lo.max(0.0), hi)
            }
        }
    }

    /// Its rates: what sets the time scales of a search.
    fn rates(self) -> (f64, f64) {
        match self {
            Basis::Exp(f) => (-f, -f),
            Basis::Phi(k) => (k, k),
            Basis::J(f, k) => ((-f).max(k), (-f).min(k)),
        }
    }
}

/// `c0 + sum a B(τ)` over monotone [`Basis`] terms: a closed-form node less a
/// threshold under a rail of several modes, or the rail's own motion less a
/// level. Each term's range over an interval is its values at the ends, so
/// the sum's range there is bounded, and a search can skip what cannot hold a
/// crossing and solve by Newton where the sum is monotone.
pub struct ExpSum<'a> {
    pub c0: f64,
    pub terms: &'a [(f64, Basis)],
}

impl ExpSum<'_> {
    fn at(&self, tau: f64) -> f64 {
        self.terms.iter().fold(self.c0, |acc, &(a, b)| acc + a * b.at(tau))
    }

    fn rate(&self, tau: f64) -> f64 {
        self.terms.iter().map(|&(a, b)| a * b.rate(tau)).sum()
    }

    /// The range of `dir` times the sum over `[a, b]` (`b` may be infinite).
    fn range(&self, dir: f64, a: f64, b: f64) -> (f64, f64) {
        let (mut lo, mut hi) = (dir * self.c0, dir * self.c0);
        for &(c, basis) in self.terms {
            let ya = dir * c * basis.at(a);
            let yb = dir * c * if b.is_finite() { basis.at(b) } else { basis.at_inf() };
            lo += ya.min(yb);
            hi += ya.max(yb);
        }
        (lo, hi)
    }

    /// The least rate of `dir` times the sum over `[a, b]`.
    fn least_rate(&self, dir: f64, a: f64, b: f64) -> f64 {
        self.terms.iter().map(|&(c, basis)| {
            let (lo, hi) = basis.rate_bounds(a, b);
            let c = dir * c;
            if c >= 0.0 { c * lo } else { c * hi }
        }).sum()
    }

    /// The first `τ > 0` at which the sum crosses 0 upward (`rising`) or
    /// downward: from strictly below to at or past 0, or the reverse. A sum
    /// already past 0 at `τ = 0` must first come back.
    ///
    /// Bounded term by term over a window, a node and a threshold that ride
    /// the same rail cancel only in the sum: a mode's line and the
    /// capacitor's response to it reach millions of volts each where their
    /// difference is millivolts, so no window but the narrowest excluded a
    /// root (F1r14: corpus 07 switch-on 3x its bar, 80 range bounds a
    /// search; on a coupled cell's path the windows' bisection branched
    /// without end, F1r15). Taken by rate ([`Merged`]), the sum's turning
    /// points are the zeros of an exponential polynomial, few and found in
    /// order, and the sum is monotone between them: each piece is one
    /// evaluation at its end and, where it holds the crossing, the same
    /// Newton to the last bit. A sum of more than [`MERGED_MAX`] rates is
    /// searched by windows.
    pub fn first_crossing(&self, rising: bool) -> Option<f64> {
        self.first_crossing_until(rising, f64::INFINITY)
    }

    /// [`Self::first_crossing`] at or before `until`, or `None`: a drive
    /// that is aimed again by then (a draining cell's path, solved again
    /// within `RAIL_DRIFT_V` of its own motion) has no use for a later one,
    /// and its bounds over `[0, until]` dismiss most thresholds at once.
    pub fn first_crossing_until(&self, rising: bool, until: f64) -> Option<f64> {
        match self.merged() {
            Some(m) => self.first_crossing_merged(&m, rising, until),
            None => self.first_crossing_scan(rising).filter(|&t| t <= until),
        }
    }

    /// The sum by rate: `c0 + sum d φ_λ(τ) + sum q ψ_λ(τ)`, `e^(fτ) = 1 + f
    /// φ_(-f)(τ)` and `J(f, k, τ) = (φ_(-f)(τ) - φ_k(τ)) / (f + k)`; a
    /// capacitor within `1e-6` of resonance with a mode (a weakly coupled
    /// partition's own mode of a coupled path sits on its own rate) takes
    /// the confluent form `ψ_λ(τ) = ∫0^τ s e^(-λs) ds` at their mean rate,
    /// whose difference from `J` is second order in their gap (below `1e-12`
    /// of the term), where the split form's coefficients would cancel past
    /// any precision.
    fn merged(&self) -> Option<Merged> {
        let mut m = Merged { c0: self.c0, n: 0, lam: [0.0; MERGED_MAX], d: [0.0; MERGED_MAX], q: [0.0; MERGED_MAX], conf: 0.0 };
        for &(a, b) in self.terms {
            match b {
                Basis::Exp(f) => {
                    if !(f < 0.0) {
                        return None;
                    }
                    m.c0 += a;
                    m.add(-f, a * f, 0.0)?;
                }
                Basis::Phi(k) => {
                    if !(k >= 0.0) {
                        return None;
                    }
                    m.add(k, a, 0.0)?;
                }
                Basis::J(f, k) => {
                    let s = f + k;
                    if !(f < 0.0 && k >= 0.0) {
                        return None;
                    }
                    if s.abs() > 1e-6 * (-f).max(k) {
                        let c = a / s;
                        m.add(-f, c, 0.0)?;
                        m.add(k, -c, 0.0)?;
                    } else {
                        let lam = 0.5 * (-f + k);
                        m.add(lam, 0.0, a)?;
                        m.conf += 1e-9 * (a / (lam * lam)).abs();
                    }
                }
            }
        }
        // Ascending rates, as the sign rule reads them.
        for i in 1..m.n {
            let mut j = i;
            while j > 0 && m.lam[j - 1] > m.lam[j] {
                m.lam.swap(j - 1, j);
                m.d.swap(j - 1, j);
                m.q.swap(j - 1, j);
                j -= 1;
            }
        }
        (m.c0.is_finite() && m.d[..m.n].iter().chain(&m.q[..m.n]).all(|d| d.is_finite())).then_some(m)
    }

    /// [`Self::first_crossing`] on the merged form `m`.
    fn first_crossing_merged(&self, m: &Merged, rising: bool, until: f64) -> Option<f64> {
        self.first_crossing_merged_sharing(m, rising, until, None)
    }

    /// [`Self::first_crossing`], sharing with other sums of the same terms
    /// (`c0` aside) the turning points `zeros` holds, or computing them into
    /// it: they depend on the terms' rates and weights alone.
    fn first_crossing_sharing(&self, rising: bool, zeros: &mut Option<Vec<f64>>) -> Option<f64> {
        match self.merged() {
            Some(m) => self.first_crossing_merged_sharing(&m, rising, f64::INFINITY, Some(zeros)),
            None => self.first_crossing_scan(rising).filter(|&t| t <= f64::INFINITY),
        }
    }

    /// `zeros` is `None` where no other sum shares them: they are then
    /// found for this one alone, with no copy kept.
    fn first_crossing_merged_sharing(&self, m: &Merged, rising: bool, until: f64, zeros: Option<&mut Option<Vec<f64>>>) -> Option<f64> {
        let dir = if rising { 1.0 } else { -1.0 };
        let (lo, hi, margin) = m.bounds(until);
        // Each term at `τ = 0` once, for the sum and its scale alike.
        let (mut at0, mut scale_terms) = (self.c0, 0.0);
        for &(a, b) in self.terms {
            let y = a * b.at(0.0);
            at0 += y;
            scale_terms += y.abs();
        }
        let g0 = dir * at0;
        let scale = self.c0.abs() + scale_terms;
        let eps = 64.0 * f64::EPSILON * scale;
        let (least, most) = if dir > 0.0 { (lo, hi) } else { (-hi, -lo) };
        // Never reaching 0, or (already past it) never coming back.
        if (g0 < 0.0 && most < -margin) || (g0 >= 0.0 && least > -eps + margin) {
            return None;
        }
        let mut turns = match zeros {
            Some(Some(z)) => z.clone(),
            Some(shared) => {
                let mut z = Vec::new();
                exp_zeros(&m.lam[..m.n], &m.d[..m.n], &m.q[..m.n], &mut z);
                *shared = Some(z.clone());
                z
            }
            None => {
                let mut z = Vec::new();
                exp_zeros(&m.lam[..m.n], &m.d[..m.n], &m.q[..m.n], &mut z);
                z
            }
        };
        let slow = m.lam[..m.n].iter().copied().filter(|&l| l > 0.0).fold(f64::INFINITY, f64::min);
        // Past every rate's decay below the smallest double the sum is its
        // line: a turn there is the merged form's own rounding.
        turns.retain(|&t| t * slow < 745.0 && t < until);
        let fast = m.lam[..m.n].iter().copied().fold(0.0f64, f64::max);
        let start = if g0 < 0.0 {
            0.0
        } else {
            // As the windowed search: the first point the reversed sum is
            // past 0 by more than the sum rounds there.
            let shifted = ExpSum { c0: self.c0 + dir * eps, terms: self.terms };
            let t = shifted.reach_pieces(-dir * (m.limit() + dir * eps), &turns, -dir, 0.0, -(g0 + eps), slow, fast, until)?;
            if !(dir * self.at(t) < 0.0) {
                let t1 = t.next_up();
                if !(dir * self.at(t1) < 0.0) {
                    return None;
                }
                t1
            } else {
                t
            }
        };
        let gs = dir * self.at(start);
        self.reach_pieces(dir * m.limit(), &turns, dir, start, gs, slow, fast, until)
    }

    /// The first `τ >= a` at which `dir` times the sum is at or past 0, given
    /// it is `ga < 0` at `a`, on a sum monotone between its turning points
    /// `turns` (ascending) and tending to `dir * lim` (`lim` here already
    /// times `dir`). Past the last turn it is monotone to its limit: windows
    /// doubling from the fastest rate's time find a finite end, and past
    /// every rate's settling (`64 / slow`) the sum is its limit. Nothing
    /// past `until` is searched.
    #[allow(clippy::too_many_arguments)]
    fn reach_pieces(&self, lim: f64, turns: &[f64], dir: f64, a: f64, ga: f64, slow: f64, fast: f64, until: f64) -> Option<f64> {
        let (mut a, mut ga) = (a, ga);
        for &c in turns {
            if !(c > a) {
                continue;
            }
            let gc = dir * self.at(c);
            if gc >= 0.0 {
                return Some(self.newton(dir, a, c, ga, gc));
            }
            a = c;
            ga = gc;
        }
        if !(lim > 0.0) {
            return None;
        }
        let horizon = if slow.is_finite() { a + 128.0 / slow } else { f64::INFINITY };
        let mut w = if fast > 0.0 { 1.0 / fast } else { 1e-9 }.max(a * 1e-3);
        for _ in 0..2200 {
            let b = (a + w).min(until);
            if !(b > a && b.is_finite()) || (a > horizon && lim.is_finite()) {
                return None;
            }
            let gb = dir * self.at(b);
            if gb >= 0.0 {
                return Some(self.newton(dir, a, b, ga, gb));
            }
            if b == until {
                return None;
            }
            a = b;
            ga = gb;
            w *= 2.0;
        }
        None
    }

    /// The windowed search: windows doubling from a thousandth of the
    /// fastest rate's time, each skipped where its range stays below 0, a
    /// window that may hold a root bisected until its range excludes one or
    /// the sum is monotone there.
    fn first_crossing_scan(&self, rising: bool) -> Option<f64> {
        let dir = if rising { 1.0 } else { -1.0 };
        let (mut fast, mut slow) = (0.0f64, f64::INFINITY);
        for &(_, b) in self.terms {
            let (hi, lo) = b.rates();
            fast = fast.max(hi);
            slow = slow.min(lo);
        }
        let h0 = if fast > 0.0 { 1e-3 / fast } else { 1e-9 };
        let g0 = dir * self.at(0.0);
        let start = if g0 < 0.0 {
            0.0
        } else {
            // Below first: the first point the reversed sum is clearly past 0,
            // by more than the sum rounds there, so the root it has just come
            // through is no crossing back of its own last bits.
            let scale = self.c0.abs() + self.terms.iter().map(|&(a, b)| (a * b.at(0.0)).abs()).sum::<f64>();
            let eps = 64.0 * f64::EPSILON * scale;
            let shifted = ExpSum { c0: self.c0 + dir * eps, terms: self.terms };
            let t = shifted.reach(-dir, 0.0, -(g0 + eps), h0, slow)?;
            if !(dir * self.at(t) < 0.0) {
                let t1 = t.next_up();
                if !(dir * self.at(t1) < 0.0) {
                    return None;
                }
                t1
            } else {
                t
            }
        };
        let gs = dir * self.at(start);
        self.reach(dir, start, gs, h0, slow)
    }

    /// The first `τ >= a` at which `dir` times the sum is at or past 0, given
    /// it is `ga < 0` at `a`: windows doubling from `h0`, each skipped where
    /// its range stays below 0, until the sum's range from there on can
    /// never reach it.
    fn reach(&self, dir: f64, a: f64, ga: f64, h0: f64, slow: f64) -> Option<f64> {
        let (mut a, mut ga, mut w) = (a, ga, h0.max(a * 1e-3));
        for _ in 0..4000 {
            if self.range(dir, a, f64::INFINITY).1 < 0.0 {
                return None;
            }
            let b = a + w;
            if !(b > a && b.is_finite()) {
                return None;
            }
            if self.range(dir, a, b).1 >= 0.0 {
                if let Some(t) = self.first_in(dir, a, b, ga, 0) {
                    return Some(t);
                }
            }
            ga = dir * self.at(b);
            a = b;
            w *= 2.0;
            // Past every mode's settling the window may jump to infinity's
            // own bound, which the next pass reads.
            if slow > 0.0 && a > 64.0 / slow && w < a {
                w = a;
            }
        }
        None
    }

    /// The first point of `[a, b]` at which `dir` times the sum is at or past
    /// 0, given it is `ga < 0` at `a`, or `None`.
    fn first_in(&self, dir: f64, a: f64, b: f64, ga: f64, depth: u32) -> Option<f64> {
        if self.range(dir, a, b).1 < 0.0 {
            return None;
        }
        let mid = a + 0.5 * (b - a);
        if !(mid > a && mid < b) || depth > 200 {
            return (dir * self.at(b) >= 0.0).then_some(b);
        }
        if self.least_rate(dir, a, b) > 0.0 {
            let gb = dir * self.at(b);
            return (gb >= 0.0).then(|| self.newton(dir, a, b, ga, gb));
        }
        let gm = dir * self.at(mid);
        if gm >= 0.0 {
            return Some(self.first_in(dir, a, mid, ga, depth + 1).unwrap_or(mid));
        }
        self.first_in(dir, a, mid, ga, depth + 1).or_else(|| self.first_in(dir, mid, b, gm, depth + 1))
    }

    /// The root of `dir` times the sum, increasing on `[a, b]` from `ga < 0`
    /// to `gb >= 0`: Newton kept inside the bracket, to the last bit.
    fn newton(&self, dir: f64, a: f64, b: f64, ga: f64, gb: f64) -> f64 {
        let (mut lo, mut hi) = (a, b);
        let mut t = if gb > ga { a + (b - a) * (-ga / (gb - ga)) } else { a + 0.5 * (b - a) };
        for _ in 0..200 {
            if !(t > lo && t < hi) {
                t = lo + 0.5 * (hi - lo);
                if !(t > lo && t < hi) {
                    break;
                }
            }
            let g = dir * self.at(t);
            if g >= 0.0 {
                hi = t;
            } else {
                lo = t;
            }
            let d = dir * self.rate(t);
            let next = if d > 0.0 { t - g / d } else { f64::NAN };
            let met = !((next - t).abs() > 4.0 * f64::EPSILON * t.abs());
            t = if next > lo && next < hi && next != t {
                next
            } else if !met {
                // Thrown out of a wide bracket (a flat stretch near a turn of
                // the merged search's pieces): bisect.
                lo + 0.5 * (hi - lo)
            } else if g >= 0.0 {
                // Newton has met the root: walk the bracket's last bits.
                t.next_down()
            } else {
                t.next_up()
            };
        }
        hi
    }
}

/// How many distinct rates [`Merged`] holds.
const MERGED_MAX: usize = 16;

/// An [`ExpSum`] by rate: `c0 + sum d φ_λ(τ) + sum q ψ_λ(τ)` over distinct
/// `λ >= 0`, ascending. Its rate `g' = sum (d + q τ) e^(-λτ)` is an
/// exponential polynomial, and the sum is monotone between its zeros
/// ([`exp_zeros`]). The form serves for that structure and for bounds; the
/// sum itself is always evaluated term by term, as [`ExpSum::at`] does.
struct Merged {
    c0: f64,
    n: usize,
    lam: [f64; MERGED_MAX],
    d: [f64; MERGED_MAX],
    q: [f64; MERGED_MAX],
    /// What the confluent forms leave of `J`, at most.
    conf: f64,
}

impl Merged {
    /// Adds `d φ_lam + q ψ_lam`. A rate within `1e-6` of one already held is
    /// taken as that one to first order, `φ_(λ+δ) = φ_λ - δ ψ_λ`: rates a
    /// hair apart (a partition's own mode beside its own rate) would
    /// otherwise turn the sum where every term has long decayed, `1 / δ` on.
    fn add(&mut self, lam: f64, d: f64, q: f64) -> Option<()> {
        if let Some(i) = self.lam[..self.n].iter().position(|&l| l == lam) {
            self.d[i] += d;
            self.q[i] += q;
            return Some(());
        }
        if let Some(i) = self.lam[..self.n].iter().position(|&l| (l - lam).abs() <= 1e-6 * l.max(lam)) {
            let delta = lam - self.lam[i];
            self.d[i] += d;
            self.q[i] += q - d * delta;
            let l = self.lam[i];
            self.conf += 1e-11 * (d / l).abs() + 1e-5 * (q / (l * l)).abs();
            return Some(());
        }
        if self.n == MERGED_MAX {
            return None;
        }
        self.lam[self.n] = lam;
        self.d[self.n] = d;
        self.q[self.n] = q;
        self.n += 1;
        Some(())
    }

    /// Each rate's share at `τ` without bound: `d / λ + q / λ²`, or an
    /// infinity of its sign at `λ = 0`.
    fn end(&self, i: usize) -> f64 {
        let (l, d, q) = (self.lam[i], self.d[i], self.q[i]);
        if l > 0.0 {
            d / l + q / (l * l)
        } else if q != 0.0 {
            q * f64::INFINITY
        } else if d != 0.0 {
            d * f64::INFINITY
        } else {
            0.0
        }
    }

    /// The sum's limit as `τ` grows without bound.
    fn limit(&self) -> f64 {
        (0..self.n).fold(self.c0, |s, i| s + self.end(i))
    }

    /// The sum's range over `[0, until]`, each `φ_λ` in `[0, φ_λ(until)]` and
    /// `ψ_λ` in `[0, ψ_λ(until)]` (`1 / λ` and `1 / λ²` without end), and
    /// how far that rounds: what a resonant pair's large coefficients cancel
    /// to, at 64 ulps of their scale, and what the confluent forms leave out.
    fn bounds(&self, until: f64) -> (f64, f64, f64) {
        let (mut lo, mut hi, mut mag) = (self.c0, self.c0, self.c0.abs());
        for i in 0..self.n {
            let (l, d, q) = (self.lam[i], self.d[i], self.q[i]);
            let (dd, qq) = if until.is_finite() {
                (d * exp_integral(-l, until), q * ramp_response(-l, l, until))
            } else if l > 0.0 {
                (d / l, q / (l * l))
            } else {
                (d * f64::INFINITY, q * f64::INFINITY)
            };
            for part in [dd, qq] {
                if part.is_nan() {
                    continue;
                }
                lo += part.min(0.0);
                hi += part.max(0.0);
                if l > 0.0 {
                    mag += part.abs();
                }
            }
        }
        (lo, hi, 64.0 * f64::EPSILON * mag + self.conf)
    }
}

/// The zeros in `(0, ∞)` of `p(τ) = sum (a + b τ) e^(-μτ)` over distinct
/// ascending `mu`, ascending, appended to `out`. `e^(μ0 τ) p` is monotone
/// between the zeros of its own derivative, which has one degree fewer in
/// all (Rolle), found first; a derivative whose coefficients share a sign
/// has none, and an exponential sum with one sign change among its
/// coefficients has one zero at most (Descartes), so neither recurses.
fn exp_zeros(mu: &[f64], a: &[f64], b: &[f64], out: &mut Vec<f64>) {
    let (mut m, mut ka, mut kb, mut n) = ([0.0; MERGED_MAX], [0.0; MERGED_MAX], [0.0; MERGED_MAX], 0);
    for i in 0..mu.len() {
        if a[i] != 0.0 || b[i] != 0.0 {
            m[n] = mu[i];
            ka[n] = a[i];
            kb[n] = b[i];
            n += 1;
        }
    }
    if n == 0 {
        return;
    }
    if n == 1 {
        if kb[0] != 0.0 {
            let t = -ka[0] / kb[0];
            if t > 0.0 && t.is_finite() {
                out.push(t);
            }
        }
        return;
    }
    let all = |pos: bool| (0..n).all(|i| if pos { ka[i] >= 0.0 && kb[i] >= 0.0 } else { ka[i] <= 0.0 && kb[i] <= 0.0 });
    if all(true) || all(false) {
        return;
    }
    let plain = kb[..n].iter().all(|&x| x == 0.0);
    let changes = (1..n).filter(|&i| (ka[i] > 0.0) != (ka[i - 1] > 0.0)).count();
    // `p e^(μ0 τ) = sum (a + b τ) e^(-ν τ)`, `ν = μ - μ0`, and its derivative.
    let (mut nu, mut da, mut db) = ([0.0; MERGED_MAX], [0.0; MERGED_MAX], [0.0; MERGED_MAX]);
    for i in 0..n {
        nu[i] = m[i] - m[0];
        da[i] = kb[i] - nu[i] * ka[i];
        db[i] = -nu[i] * kb[i];
    }
    let p = |t: f64| (0..n).fold(0.0, |acc, i| acc + (ka[i] + kb[i] * t) * (-nu[i] * t).exp());
    let dp = |t: f64| (0..n).fold(0.0, |acc, i| acc + (da[i] + db[i] * t) * (-nu[i] * t).exp());
    let mut turns = Vec::new();
    if !(plain && changes <= 1) {
        exp_zeros(&nu[..n], &da[..n], &db[..n], &mut turns);
    }
    let p_inf = if kb[0] != 0.0 { kb[0] * f64::INFINITY } else { ka[0] };
    let (mut lo, mut pa) = (0.0, p(0.0));
    for i in 0..=turns.len() {
        let (b, pb) = if i < turns.len() { (turns[i], p(turns[i])) } else { (f64::INFINITY, p_inf) };
        if (pa < 0.0 && pb > 0.0) || (pa > 0.0 && pb < 0.0) {
            // A finite end on the last piece: windows doubling from the
            // fastest decay's time.
            let (mut b, mut pb) = (b, pb);
            if !b.is_finite() {
                let mut w = if nu[n - 1] > 0.0 { 1.0 / nu[n - 1] } else { 1.0 };
                for _ in 0..2200 {
                    b = lo + w;
                    pb = p(b);
                    if !b.is_finite() || pb == 0.0 || (pb > 0.0) != (pa > 0.0) {
                        break;
                    }
                    w *= 2.0;
                }
            }
            if b.is_finite() && (pb == 0.0 || (pb > 0.0) != (pa > 0.0)) {
                out.push(bracketed_root(&p, &dp, lo, b, pa));
            }
        }
        if !b.is_finite() {
            break;
        }
        lo = b;
        pa = pb;
    }
}

/// The turning points in `(0, ∞)` of a rail path `sum s φ_k(τ)` over its
/// modes `(k, s)`, ascending: the zeros of its slope `sum s e^(-kτ)`. `None`
/// for a path of more rates than [`exp_zeros`] takes.
pub(crate) fn path_turns(modes: impl Iterator<Item = (f64, f64)>) -> Option<Vec<f64>> {
    let mut m = Merged { c0: 0.0, n: 0, lam: [0.0; MERGED_MAX], d: [0.0; MERGED_MAX], q: [0.0; MERGED_MAX], conf: 0.0 };
    for (k, s) in modes {
        if !(k >= 0.0 && k.is_finite() && s.is_finite()) {
            return None;
        }
        m.add(k, s, 0.0)?;
    }
    for i in 1..m.n {
        let mut j = i;
        while j > 0 && m.lam[j - 1] > m.lam[j] {
            m.lam.swap(j - 1, j);
            m.d.swap(j - 1, j);
            j -= 1;
        }
    }
    let mut out = Vec::new();
    exp_zeros(&m.lam[..m.n], &m.d[..m.n], &m.q[..m.n], &mut out);
    Some(out)
}

/// The root of `p` in `[a, b]`, `p(a) = pa` and `p(b)` of the other sign (or
/// 0): Newton kept inside the bracket, to a few ulps.
fn bracketed_root(p: &dyn Fn(f64) -> f64, dp: &dyn Fn(f64) -> f64, a: f64, b: f64, pa: f64) -> f64 {
    let up = pa < 0.0;
    let (mut lo, mut hi) = (a, b);
    let mut t = a + 0.5 * (b - a);
    for _ in 0..200 {
        if !(t > lo && t < hi) {
            t = lo + 0.5 * (hi - lo);
            if !(t > lo && t < hi) {
                break;
            }
        }
        let v = p(t);
        if (v >= 0.0) == up {
            hi = t;
        } else {
            lo = t;
        }
        let d = dp(t);
        let next = if d != 0.0 { t - v / d } else { f64::NAN };
        if (next - t).abs() <= 4.0 * f64::EPSILON * t.abs() {
            return t;
        }
        t = next;
    }
    lo + 0.5 * (hi - lo)
}

/// `J'(f, k, τ) = ∫0^τ e^(f (τ - s)) e^(-k s) ds`, the rate of
/// [`ramp_response`], with no growing factor.
pub fn ramp_rate(f: f64, k: f64, tau: f64) -> f64 {
    let d = f + k;
    if d > 0.0 {
        (f * tau).exp() * exp_integral(-d, tau)
    } else {
        (-k * tau).exp() * exp_integral(d, tau)
    }
}

/// The first `τ` in `(0, horizon]` at which `g(τ) = c0 + ce e^(f τ) + cl τ +
/// cj J(f, 0, τ)` crosses 0 upward (`rising`) or downward, for `f < 0`: a
/// node of a partition on a draining cell, its capacitor following the
/// drain's line (`CellDrive`), which holds until the rail's next event,
/// within `horizon`. Each term is monotone, so the sum's range over the
/// horizon is its ends' and most thresholds are dismissed on two values. `g'
/// = (ce f + cj / f) e^(f τ) + (cl - cj / f)` is affine in `e^(f τ)`, so `g`
/// has at most one extremum, found in closed form; each monotone piece with
/// a sign change is solved by Newton kept inside it, from the root the
/// line-free sum would have, until its step is a few ulps.
pub fn line_crossing(c0: f64, ce: f64, cl: f64, cj: f64, f: f64, rising: bool, horizon: f64) -> Option<f64> {
    if !(f < 0.0 && horizon > 0.0 && horizon.is_finite()) {
        return None;
    }
    let dir = if rising { 1.0 } else { -1.0 };
    let g = |t: f64| dir * (c0 + ce * (f * t).exp() + cl * t + cj * ramp_response(f, 0.0, t));
    let dg = |t: f64| dir * ((ce * f + cj / f) * (f * t).exp() + (cl - cj / f));
    let (e_h, j_h) = ((f * horizon).exp(), ramp_response(f, 0.0, horizon));
    let (g_0, g_h) = (dir * (c0 + ce), dir * (c0 + ce * e_h + cl * horizon + cj * j_h));
    let most = dir * c0 + (dir * ce).max(dir * ce * e_h) + (dir * cl * horizon).max(0.0) + (dir * cj * j_h).max(0.0);
    let least = dir * c0 + (dir * ce).min(dir * ce * e_h) + (dir * cl * horizon).min(0.0) + (dir * cj * j_h).min(0.0);
    if most < 0.0 || least >= 0.0 {
        return None;
    }
    let (p, l) = (ce * f + cj / f, cl - cj / f);
    // The extremum, where `e^(f τ) = -l / p` lies in (0, 1), inside the
    // horizon.
    let ext = if p != 0.0 {
        let e = -l / p;
        (e > e_h && e < 1.0).then(|| e.ln() / f)
    } else {
        None
    };
    // A guess at the root: the line-free exponential's.
    let guess = {
        let r = -c0 / ce;
        (ce != 0.0 && r > 0.0 && r < 1.0).then(|| r.ln() / f)
    };
    let mut pieces = [(0.0, g_0, horizon, g_h), (horizon, g_h, horizon, g_h)];
    if let Some(x) = ext {
        let gx = g(x);
        pieces = [(0.0, g_0, x, gx), (x, gx, horizon, g_h)];
    }
    for &(a, ga, b, gb) in &pieces {
        if !(ga < 0.0 && gb >= 0.0) {
            continue;
        }
        // Increasing here: Newton from the guess, kept inside the bracket.
        let (mut lo, mut hi) = (a, b);
        let mut t = guess.filter(|&t| t > a && t < b).unwrap_or(a + (b - a) * (-ga / (gb - ga)));
        for _ in 0..200 {
            if !(t > lo && t < hi) {
                t = lo + 0.5 * (hi - lo);
                if !(t > lo && t < hi) {
                    break;
                }
            }
            let v = g(t);
            if v >= 0.0 {
                hi = t;
            } else {
                lo = t;
            }
            let d = dg(t);
            let next = if d > 0.0 { t - v / d } else { f64::NAN };
            if (next - t).abs() <= 4.0 * f64::EPSILON * t {
                return Some(if v >= 0.0 { t } else { next.max(t.next_up()).min(hi) });
            }
            t = if next > lo && next < hi { next } else { lo + 0.5 * (hi - lo) };
        }
        return Some(hi);
    }
    None
}

/// The rail's instant state, small enough to save whole at the start of
/// every rollback window (Rails point 8).
#[derive(Debug, Clone, PartialEq)]
pub struct RailState {
    pub u: f64,
    pub u_scan: f64,
    pub reg: Regime,
    pub battery: BatteryState,
    /// The instant SoC was last advanced to.
    pub t_last: f64,
    /// The draw of the static, stepped and direct terms, held from the last
    /// rail solve (the SoC integral, Rails point 6).
    pub held_i: f64,
    /// The closed-form terms at the last rail solve.
    pub held_cf: Vec<CfTerm>,
    /// The pending `Rail` event's version (Rails point 4).
    pub version: u64,
    pub event_t: Option<f64>,
    /// While a PSU with a rail capacitor is in CC.
    pub cap_seg: Option<CapSeg>,
    /// While the hub is a draining cell: how its closed-form hub partitions
    /// follow its rail until the next solve.
    pub cell: Option<CellDrive>,
    /// While the hub is a draining cell whose closed-form hub partitions
    /// advance on their coupled path (`cell_modes`, ruling R16 (B)): the
    /// rail's path from the last solve, which they follow as they follow a CC
    /// rail capacitor's segment.
    pub cell_path: Option<CapSeg>,
    /// The latest instant the rail solves again on that path: its first
    /// passage of `RAIL_DRIFT_V` either way, which its watch caps.
    pub cell_until: f64,
    /// Running sums over the static and stepped contributions: `sum (i - g
    /// u_p)` and `sum g`, and how many updates since the last recompute.
    pub a_run: f64,
    pub g_run: f64,
    pub updates: u64,
    /// A settle pass reads and clears this (Sources point 4).
    pub settle_changed: bool,
    /// The power-gate ping-pong across instants (HG1): how many `Rail`
    /// events in a row flipped the rail ICs' power gates, each back to the
    /// gates the one before it flipped from (at the second, W1c point 1a's
    /// hold procedure runs), and the gates before the last of them.
    pub gate_flips: u32,
    pub gates_before: Vec<bool>,
    /// The gates the last of those flips left, and how many of a stepped
    /// hub partition's own steps on a rail capacitor's segment have flipped
    /// nothing since (ruling R82 (B'): they leave the run standing,
    /// `CHASE_REGROWTH` at most).
    pub gates_after: Vec<bool>,
    pub quiet_steps: u32,
    /// The rail ICs (device indices) held off on a CC rail capacitor's
    /// segment, each with the draw its being powered adds, `dir` signed, from
    /// the hold's probes (ruling R44 (a)): such a chip is released only where,
    /// powered at its edge, it would still let the segment rise.
    pub held_draw: Vec<(usize, f64)>,
}

/// The rail's counters (stats 14, 16-18, 20). Never rolled back.
#[derive(Debug, Clone, Copy, Default)]
pub struct RailStats {
    pub moves: u64,
    pub sensitivities: u64,
    pub scans: u64,
    pub events: u64,
    pub flips: u64,
}

/// The direct devices, as one function of the rail voltage: their summed
/// draw and slope at `u` (Rails point 2).
pub trait Direct {
    fn at(&self, u: f64) -> (f64, f64);
}

impl<F: Fn(f64) -> (f64, f64)> Direct for F {
    fn at(&self, u: f64) -> (f64, f64) {
        self(u)
    }
}

/// `A0` and `G` at `u` with the direct devices linearised there.
fn totals(a_p: f64, g_p: f64, direct: &dyn Direct, u: f64) -> (f64, f64) {
    let (i_d, g_d) = direct.at(u);
    (a_p + (i_d - g_d * u), g_p + g_d)
}

/// A scalar Newton on `u` with the direct devices re-linearised each pass:
/// `next(A0, G)` is the rail equation solved for `u` with `A0`, `G` held.
fn newton(u0: f64, a_p: f64, g_p: f64, direct: &dyn Direct, next: impl Fn(f64, f64) -> f64) -> Result<f64, String> {
    let mut u = u0;
    for _ in 0..RAIL_NEWTON_MAX {
        let (a0, g) = totals(a_p, g_p, direct, u);
        let un = next(a0, g);
        if !un.is_finite() {
            return Err(format!("rail: the rail equation gave a non-finite voltage ({un}) from u = {u}"));
        }
        if (un - u).abs() <= RAIL_NEWTON_TOL {
            return Ok(un);
        }
        u = un;
    }
    Err(format!("rail: the rail Newton did not converge within {RAIL_NEWTON_MAX} iterations"))
}

/// The battery rail, `u = (E - r A0) / (1 + r G)`; `r = 0` gives `E`.
pub fn solve_battery(e: f64, r: f64, a_p: f64, g_p: f64, direct: &dyn Direct, u0: f64) -> Result<f64, String> {
    if r == 0.0 {
        return Ok(e);
    }
    newton(u0, a_p, g_p, direct, |a0, g| (e - r * a0) / (1.0 + r * g))
}

/// The settled CC rail of a PSU of setpoint `v`, `u = (dir iLimit - A0) /
/// G`.
///
/// A Newton that cannot settle past the setpoint, with the draw at the
/// setpoint within the limit, gives the setpoint: the CC solution lies at or
/// past it, where the supply's compliance holds the rail, and the regime
/// rules leave CC (or keep CV) there. A load that opens in CC sends the
/// iterates to iLimit over whatever conductance is left, the node shunt's
/// 2e-12 S, where one ulp is millions of times `RAIL_NEWTON_TOL` and a draw
/// that rounds differently at each iterate never settles (F5f: cold fuzz
/// seed 22 tpl 4, a fuse tripping in CC, 3.1e10 V; FXH's bug A is the same
/// CC solve past the setpoint, there converged and left by the exit's
/// compliance test). A Newton that settles is returned as it is, wherever it
/// lands, and one that fails short of the setpoint fails as before, without
/// a look at the setpoint: the direct devices' last iterates are what the
/// power-gate hold's trigger reads (W1c point 1a).
pub fn solve_cc(v: f64, dir_limit: f64, a_p: f64, g_p: f64, direct: &dyn Direct, u0: f64) -> Result<f64, String> {
    let last = std::cell::Cell::new(u0);
    newton(u0, a_p, g_p, direct, |a0, g| {
        last.set((dir_limit - a0) / g);
        last.get()
    })
    .or_else(|err| if compliance_keeps_cv(last.get(), v) && direction(v) * draw_at(a_p, g_p, direct, v) <= dir_limit.abs() { Ok(v) } else { Err(err) })
}

/// How [`cc_entry_rail`]'s error for a landing on a power edge begins.
pub const EDGE_LANDING: &str = "rail: the CC operating point lands on a power edge";

/// The rail of a landing on a power edge, from [`cc_entry_rail`]'s error;
/// `None` for any other error.
pub fn landing_edge(err: &str) -> Option<f64> {
    err.strip_prefix(EDGE_LANDING)?.strip_prefix(" at ")?.parse().ok()
}

/// The CC operating point the CV -> CC clamp reads (Rails point 5, ruling
/// G): the highest rail at or below the setpoint, in the PSU's direction,
/// where the draw meets the limit, the first one reached coming down from
/// the setpoint. `i_at_v` is the draw at the setpoint and `edges` the rails
/// at which a free rail IC's power window flips (`power_edges`).
///
/// The draw is monotone between edges (each direct device's current rises
/// with the rail) but not across one: a chip powered only inside its window
/// draws there and not past it, so the CC Newton from the setpoint can
/// settle past the setpoint on the chip-off draw, where the clamp would keep
/// CV however far the draw at the setpoint is past the limit (defect G: a
/// 74LS00 sinking 32 mA from a 5 V, 50 mA rail held CV at 72 mA, its CC
/// Newton at the chip-off 6.25 V), or step over the segment holding the
/// highest root to a lower one. So the Newton's root stands only when no
/// edge lies between it and the setpoint; otherwise the segments are walked
/// down from the setpoint, each read just inside its lower edge: the first
/// whose draw there is within the limit holds the root, solved there, and an
/// edge whose draw falls within the limit only across it is no operating
/// point at all: the rail lands on that chip's power edge, which is the
/// power hold's case (W1c point 1a). That is an error whose last direct
/// evaluations straddle the edge, so the hold's trigger
/// (`newton_alternating`) reads the chip there flipping at each.
///
/// A draw at the setpoint within the limit in the PSU's direction (a rail
/// back-fed past the limit) has its CC solution past the setpoint, as
/// before: the Newton's, which the clamp keeps CV on.
pub fn cc_entry_rail(v: f64, dir_limit: f64, a_p: f64, g_p: f64, direct: &dyn Direct, i_at_v: f64, edges: &[f64]) -> Result<f64, String> {
    let dir = direction(v);
    let (top, limit) = (v.abs(), dir_limit.abs());
    let first = solve_cc(v, dir_limit, a_p, g_p, direct, v);
    if !(dir * i_at_v > limit) {
        return first;
    }
    // Rails in the PSU's direction (`w = dir u`), where "down" is lower.
    let floor = match first {
        Ok(u) if dir * u <= top => dir * u,
        _ => f64::NEG_INFINITY,
    };
    // An edge at the setpoint itself counts: just below it the draw can
    // already be another power state's (a chip whose window's floor is the
    // setpoint), and dropped, the walk found a CC rail an ulp-scale step
    // under the setpoint.
    let mut below: Vec<f64> = edges.iter().map(|&e| dir * e).filter(|&w| w <= top && w > floor).collect();
    if below.is_empty() && floor > f64::NEG_INFINITY {
        return first;
    }
    below.sort_unstable_by(|a, b| b.total_cmp(a));
    below.dedup();
    let excess = |w: f64| dir * draw_at(a_p, g_p, direct, dir * w) - limit;
    let mut hi = top;
    for e in below {
        // Two reached widths either side, as a rail event lands past a power
        // edge (Rails point 4).
        let d = 2.0 * REACHED_REL * e.abs().max(1.0);
        let (up, dn) = (e + d, e - d);
        if up < hi && excess(up) <= 0.0 {
            return cc_in_segment(dir, dir_limit, a_p, g_p, direct, dir * up, dir * hi);
        }
        // The Newton's root within the edge's own band, under it: the draw
        // falls within the limit at `dn` because of that root, not across a
        // jump, and nothing above it is a root.
        if dn <= floor {
            return first;
        }
        if excess(dn) <= 0.0 {
            // The last three evaluations straddle the edge whether or not
            // `up` was read above (it is not when the band reaches `hi`).
            excess(up);
            excess(dn);
            excess(up);
            return Err(format!("{EDGE_LANDING} at {}", dir * e));
        }
        hi = dn;
    }
    // Below every edge: the Newton's root when it lies there, else the root
    // of the lowest segment, solved from its top. A draw that does not
    // reach the limit there either is one this scalar curve cannot place: a
    // chip whose drivers sit in a hub partition is linearised in `A0` and
    // `G` at its present power, so its edge moves nothing here, and a HIGH
    // stage into a node below ground draws less as the rail rises (cold 461
    // tpl 2). The Newton from the setpoint then stands as before, re-solved
    // so that a failure leaves its own iterates for the hold's trigger, and
    // the rail loop's passes judge the partitions' chips at the rail it
    // moves to.
    // A Newton that cannot reach the lowest segment's root from its top
    // (a HIGH stage into a node below ground draws more as the rail falls,
    // down to its R_out floor: cold 247 tpl 2's chip held powered by the
    // hold's probe has its one root at -2.68 V) has the root bracketed by
    // steps doubling down from the top, and solved in the bracket.
    match first {
        Ok(u) if dir * u <= hi => Ok(u),
        _ => match newton(dir * hi, a_p, g_p, direct, |a0, g| (dir_limit - a0) / g) {
            Ok(u) if dir * u <= hi => Ok(u),
            _ => {
                let (mut above, mut step) = (hi, 1e-3 * hi.abs().max(1.0));
                while step < 1e6 {
                    let w = hi - step;
                    if excess(w) <= 0.0 {
                        return cc_in_segment(dir, dir_limit, a_p, g_p, direct, dir * w, dir * above);
                    }
                    above = w;
                    step *= 2.0;
                }
                solve_cc(v, dir_limit, a_p, g_p, direct, v)
            }
        },
    }
}

/// The root of the draw at the limit between `u_in`, where the draw (in the
/// PSU's direction `dir`) is within it, and `u_out`, where it is past it,
/// with no power edge between (`cc_entry_rail`): the rail Newton from
/// `u_out`, a bisection wherever an iterate leaves the bracket.
fn cc_in_segment(dir: f64, dir_limit: f64, a_p: f64, g_p: f64, direct: &dyn Direct, u_in: f64, u_out: f64) -> Result<f64, String> {
    let (mut u_in, mut u_out) = (u_in, u_out);
    let mut u = u_out;
    // A bisection halves a bracket of volts to the tolerance in about 45
    // steps; the Newton's own cap on top.
    for _ in 0..(RAIL_NEWTON_MAX + 64) {
        let (a0, g) = totals(a_p, g_p, direct, u);
        if dir * (a0 + g * u) <= dir_limit.abs() {
            u_in = u;
        } else {
            u_out = u;
        }
        let mut un = (dir_limit - a0) / g;
        let (lo, hi) = (u_in.min(u_out), u_in.max(u_out));
        if !(un > lo && un < hi) {
            un = 0.5 * (u_in + u_out);
        }
        if (un - u).abs() <= RAIL_NEWTON_TOL || (u_in - u_out).abs() <= RAIL_NEWTON_TOL {
            return Ok(un);
        }
        u = un;
    }
    Err(format!("rail: the CC operating point did not converge between {u_in} and {u_out}"))
}

/// The CC rail over rail capacitors of charge `q` (Rails point 4): `u = q +
/// esr_eff (dir iLimit - draw(u))`, the direct devices exact at `u`.
pub fn solve_cc_cap(q: f64, esr_eff: f64, dir_limit: f64, a_p: f64, g_p: f64, direct: &dyn Direct) -> Result<f64, String> {
    newton(q, a_p, g_p, direct, |a0, g| (q + esr_eff * (dir_limit - a0)) / (1.0 + g * esr_eff))
}

/// The draw at `u`, `A0(u) + G(u) u`, the direct devices evaluated exactly.
pub fn draw_at(a_p: f64, g_p: f64, direct: &dyn Direct, u: f64) -> f64 {
    let (i_d, _) = direct.at(u);
    a_p + g_p * u + i_d
}

/// What the rail capacitors add to the CV -> CC test (Rails point 5, "A rail
/// capacitor below the setpoint is demand"). A rail capacitor is no term of
/// `A0` or `G`, but one whose charge holds the rail short of `V` would draw
/// at the setpoint: `d / esr` for its deficit `d > 0`, without bound at `esr
/// = 0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CapDemand {
    /// The summed draw at `V`, in the PSU's direction: 0 when no capacitor
    /// is below the setpoint, infinite when one at `esr = 0` is.
    pub draw: f64,
    /// The rail the capacitors hold, the charge CC with a rail capacitor
    /// starts from (Rails point 4): their charge shared at the rail, `V - dir
    /// sum(C d) / C_rail` over the capacitors with a deficit, a capacitor at
    /// or above the setpoint counted at `V`. Exactly `V` when none is below
    /// it. The rail itself starts `esr_eff I_cap` above it.
    pub held: f64,
}

impl CapDemand {
    /// No capacitor below the setpoint `v`.
    pub fn none(v: f64) -> CapDemand {
        CapDemand { draw: 0.0, held: v }
    }

    /// The demand of capacitors given as `(C, esr, d)`, `d` each one's
    /// deficit `dir (V - V_f - vc)` with `vc` its charge measured from its
    /// fixed pin `V_f` to the rail, `c_rail` their summed capacitance.
    pub fn of(v: f64, c_rail: f64, caps: impl IntoIterator<Item = (f64, f64, f64)>) -> CapDemand {
        let (mut draw, mut short) = (0.0, 0.0);
        for (c, esr, d) in caps {
            if d > 0.0 {
                draw += if esr > 0.0 { d / esr } else { f64::INFINITY };
                // Weighted by its share of `C_rail`, exactly 1 for a lone
                // capacitor, so it holds the rail at exactly its own charge.
                short += (c / c_rail) * d;
            }
        }
        if short > 0.0 {
            CapDemand { draw, held: v - direction(v) * short }
        } else {
            CapDemand::none(v)
        }
    }
}

/// One regime transition, at most, for a PSU (Rails point 5). `u_rail` is the
/// present rail; `demand` is the rail capacitors' (`CapDemand::none` in CC,
/// where it is not read); `edges` gives the rails where a free rail IC's
/// power flips (`cc_entry_rail`), asked only by the CV -> CC clamp and the
/// CC exit with no rail capacitor. Returns the regime and the CC candidate
/// the clamp read (`u_rail` where none was solved): entering CC with no rail
/// capacitor, that is the CC rail.
#[allow(clippy::too_many_arguments)]
pub fn next_regime(reg: Regime, v: f64, i_limit: f64, rail_cap: bool, a_p: f64, g_p: f64, direct: &dyn Direct, u_rail: f64, demand: CapDemand, edges: &dyn Fn() -> Vec<f64>) -> Result<(Regime, f64), String> {
    let i_at_v = draw_at(a_p, g_p, direct, v);
    // The CC exit with no rail capacitor reads the draw at the setpoint as
    // the CC solution reaching it, which holds only while the draw rises
    // with the rail between the two. Across a free rail IC's power edge it
    // does not: the draw at the setpoint is another power state's (defect
    // C, cold 364 tpl 2: a 74LS00 off on a 3.43 V CC rail, back-feeding its
    // rail through a clock-high output once powered at the 4.881 V setpoint,
    // where its draw is within the limit; the CC solution, chip off, stays
    // below its window). There the rail itself is the test, the clamp the
    // entry reads, as in exact mode.
    if reg == Regime::Cc && !rail_cap && i_limit > 0.0 {
        let dir = direction(v);
        // A CC solution past the setpoint with the draw at the setpoint past
        // the limit is the twin case: the continuation passed the setpoint
        // on another power state's draw (a chip off past its window), and
        // CV there would deliver past the limit. The operating point is the
        // entry's, coming down from the setpoint (ruling G): CC moves
        // there, or CV only when that too reaches the setpoint. Left in CV,
        // a loop pass that re-solved no partition settled there (cold 267
        // tpl 2: 114.8 mA from a 62.7 mA PSU).
        // A search that fails other than by landing on an edge (the hold's
        // case) leaves the exit as it was: a HIGH stage into a node below
        // ground draws less as the rail rises, and the rail Newton from the
        // setpoint can fail on it (cold 536 tpl 2).
        if compliance_keeps_cv(u_rail, v) && dir * i_at_v > i_limit {
            return match cc_entry_rail(v, dir * i_limit, a_p, g_p, direct, i_at_v, &edges()) {
                Ok(u) if !compliance_keeps_cv(u, v) => Ok((Regime::Cc, u)),
                Err(err) if err.starts_with(EDGE_LANDING) => Err(err),
                _ => Ok((Regime::Cv, u_rail)),
            };
        }
        if edges().iter().any(|&e| dir * e > dir * u_rail && dir * e < v.abs()) {
            return Ok((if compliance_keeps_cv(u_rail, v) { Regime::Cv } else { Regime::Cc }, u_rail));
        }
    }
    let (i_at_v, u_cc) = if demand.draw > 0.0 {
        let i = i_at_v + direction(v) * demand.draw;
        // A PSU that left CC holds its capacitors' demand at exactly the
        // limit (the exit's tie, Rails point 5), so only a draw past it by
        // more than the reached-width the exit itself lands within re-enters:
        // never the rounding of a re-solve at the same draws.
        // A total past the limit against the PSU's direction is a source
        // back-driving the rail, which the CV supply sinks (ruling R40): the
        // CC solution at the instant, the capacitors' charge plus their
        // current's drop across their esr, is past the setpoint there, so
        // the clamp keeps CV (FXI, bug D: a pulse's edge through 2.2 ohm
        // back-drove the rail by 16.6 A net of a capacitor's 3.2 A demand,
        // and the clamp, reading the charge alone, entered CC at 6.2 V).
        if reg == Regime::Cv && direction(v) * i <= i_limit + REACHED_REL * i_limit.max(1.0) {
            return Ok((Regime::Cv, u_rail));
        }
        // A capacitor below the setpoint holds the CC solution short of it at
        // the instant, so the compliance clamp, which keeps CV only for a CC
        // solution at the setpoint, reads the rail it holds: the capacitor's
        // draw is the whole reason to enter, and it is no removed load.
        (i, demand.held)
    } else {
        // The CC candidate's settled rail is only read by the CV -> CC clamp.
        // With a rail capacitor the rail lands nowhere at the instant (CC
        // starts from the capacitors' charge and ramps), so ruling G's
        // operating point, an edge landing above all, has no meaning there
        // yet: the Newton's candidate, as before (an open item: entered at a
        // landing, a 555 slid along its floor to the hold-episode guard on
        // cold 175 tpl 2 and 696 tpl 2; held at the instant, the hold's
        // powered probe found the capacitors holding the rail at the
        // setpoint and declined on cold 627 tpl 4).
        let u_cc = if reg != Regime::Cv || !(i_at_v.abs() > i_limit) {
            u_rail
        } else if rail_cap {
            // With a rail capacitor the rail lands nowhere at the instant: CC
            // starts from the capacitors' charge and moves with the draws, so
            // a draw past the limit at the setpoint enters CC wherever the
            // operating point lies, a landing on a power edge included (the
            // capacitors carry the rail down to that edge, where the power
            // hold meets it as on any CC ramp). The Newton's candidate alone
            // could settle past the setpoint on another power state's draw
            // and keep CV past the limit (ruling R68 after G: a 74LS00
            // powered only inside its window on 10 uF held CV at 72 mA from
            // a 50 mA supply).
            match cc_entry_rail(v, direction(v) * i_limit, a_p, g_p, direct, i_at_v, &edges()) {
                Ok(u) => u,
                Err(err) => landing_edge(&err).ok_or(err)?,
            }
        } else {
            cc_entry_rail(v, direction(v) * i_limit, a_p, g_p, direct, i_at_v, &edges())?
        };
        (i_at_v, u_cc)
    };
    let next = if rail_cap { regime_step_rail_cap(reg, i_at_v, u_cc, u_rail, v, i_limit) } else { regime_step(reg, i_at_v, u_cc, u_rail, v, i_limit) };
    Ok((next, u_cc))
}

/// Advances the battery from `t_last` to `t` with the charge the implied rail
/// current delivers over the interval (Rails point 6). Returns the new state.
pub fn advance_soc(model: &BatteryModel, state: &BatteryState, held_i: f64, held_cf: &[CfTerm], dt: f64) -> BatteryState {
    if !(dt > 0.0) {
        return *state;
    }
    let mut q = held_i * dt;
    for term in held_cf {
        q += term.charge(dt);
    }
    model.advance(state, q).state
}

/// The slope of `OCV(soc) * ocvScale` and of `R(soc)` just below `soc`, where
/// a draining cell is heading: within one OCV-table segment `E` is linear in
/// `soc`, so a step that stays inside it gives the exact slope.
pub fn soc_slopes(model: &BatteryModel, soc: f64) -> (f64, f64) {
    let below = model.next_knot_below(soc).unwrap_or(0.0);
    let span = soc - below;
    if !(span > 0.0) {
        return (0.0, 0.0);
    }
    let h = (0.5 * span).min(1e-6);
    let s1 = soc - h;
    ((model.emf(soc) - model.emf(s1)) / h, (model.resistance(soc) - model.resistance(s1)) / h)
}

// ---- the CC rail capacitor as a known drive (Rails point 4) ---------------

/// `(e^z - 1) / z`, 1 at 0.
fn phi1(z: f64) -> f64 {
    if z == 0.0 {
        1.0
    } else {
        z.exp_m1() / z
    }
}

/// `∫0^1 θ^n e^(zθ) dθ`, the n-th derivative of [`phi1`], for `z <= 0`.
fn phi1_deriv(n: u32, z: f64) -> f64 {
    if z.abs() < 1.0 {
        // `Σ z^j / (j! (n + j + 1))`: no cancellation below |z| = 1. A term
        // that reached 0 adds nothing from there on (at `z = 0`, every call
        // of `ramp_response` at `τ = 0`, all but the first).
        let (mut term, mut sum) = (1.0, 0.0);
        for j in 0..30u32 {
            if term == 0.0 {
                break;
            }
            sum += term / f64::from(n + j + 1);
            term *= z / f64::from(j + 1);
        }
        sum
    } else {
        // Up from `φ1` by parts, `I_m = (e^z - m I_(m-1)) / z`: each step
        // shrinks the error once |z| >= 1 and m is small.
        let ez = z.exp();
        let mut i = phi1(z);
        for m in 1..=n {
            i = (ez - f64::from(m) * i) / z;
        }
        i
    }
}

/// `(φ1(x) - φ1(y)) / (x - y)`, `φ1'(x)` at `x = y`, for `x, y <= 0`. Apart,
/// the difference loses at most `1e4` ulps; close, the midpoint series is
/// exact to `O(h^4)` relative.
fn phi1_divided(x: f64, y: f64) -> f64 {
    let scale = x.abs().max(y.abs()).max(1.0);
    if (x - y).abs() > 1e-4 * scale {
        (phi1(x) - phi1(y)) / (x - y)
    } else {
        let (m, h) = (0.5 * (x + y), 0.5 * (x - y));
        phi1_deriv(1, m) + phi1_deriv(3, m) * h * h / 6.0
    }
}

/// `(e^z - 1 - z) / z^2 = ∫0^1 (1 - θ) e^(zθ) dθ`, 1/2 at 0, for `z <= 0`.
fn phi2(z: f64) -> f64 {
    if z.abs() < 1.0 {
        // `Σ z^j / (j + 2)!`.
        let (mut term, mut sum) = (0.5, 0.0);
        for j in 0..30u32 {
            sum += term;
            term *= z / f64::from(j + 3);
        }
        sum
    } else {
        (z.exp_m1() - z) / (z * z)
    }
}

/// `(φ2(x) - φ2(y)) / (x - y)`, as [`phi1_divided`]: `φ2^(n) = I_n - I_(n+1)`.
fn phi2_divided(x: f64, y: f64) -> f64 {
    let scale = x.abs().max(y.abs()).max(1.0);
    if (x - y).abs() > 1e-4 * scale {
        (phi2(x) - phi2(y)) / (x - y)
    } else {
        let (m, h) = (0.5 * (x + y), 0.5 * (x - y));
        let d = |n: u32| phi1_deriv(n, m) - phi1_deriv(n + 1, m);
        d(1) + d(3) * h * h / 6.0
    }
}

/// `χ_k(t) = ∫0^t φ_k(s) ds`, for `k >= 0`: `t² φ2(-k t)`, `t² / 2` at `k =
/// 0`.
pub fn phi_integral(k: f64, t: f64) -> f64 {
    t * t * phi2(-k * t)
}

/// `∫0^t e^(a s) ds`, without cancellation at small `a t`.
pub fn exp_integral(a: f64, t: f64) -> f64 {
    t * phi1(a * t)
}

/// `∫0^t J(f, k, s) ds` (see [`ramp_response`]), for `f <= 0 <= k`.
pub fn ramp_charge(f: f64, k: f64, t: f64) -> f64 {
    t * t * t * phi2_divided(f * t, -k * t)
}

/// `J(f, k, t) = ∫0^t e^(f (t - s)) φ_k(s) ds` with `φ_k(s) = ∫0^s e^(-k r)
/// dr`, for `f <= 0 <= k`: how far a capacitor of rate `f` has followed a
/// rail moving as `φ_k` from a standing start. It is the divided difference
/// of `exp_integral` in its rate, so `f = -k` (the capacitor and the rail at
/// the same rate) needs no case of its own.
pub fn ramp_response(f: f64, k: f64, t: f64) -> f64 {
    t * t * phi1_divided(f * t, -k * t)
}

/// The first point of `(lo, hi]` where `g`, monotone there, has passed 0
/// from below (`up`) or from above, to the last bit.
fn bisect(g: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64, up: bool) -> f64 {
    for _ in 0..2200 {
        let mid = lo + 0.5 * (hi - lo);
        if !(mid > lo && mid < hi) {
            break;
        }
        let v = g(mid);
        if (up && v >= 0.0) || (!up && v <= 0.0) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    hi
}

/// The first `t > 0` at which `g(t) = c0 + ce e^(f t) + cphi φ_k(t) + cj
/// J(f, k, t)` crosses 0 upward (`rising`) or downward, for `f < 0 < k`:
/// a driven capacitor's node less a threshold that moves with the rail.
/// `e^(k t) g'(t) = ce f e^(d t) + cphi + cj ∫0^t e^(d s) ds`, `d = f + k`,
/// is monotone, so `g` has at most one extremum and each of the (at most
/// two) monotone pieces is bisected. Past `64 / min(-f, k)` both modes have
/// decayed to `e^-64`, so `g` there is its asymptote.
pub fn driven_crossing(c0: f64, ce: f64, cphi: f64, cj: f64, f: f64, k: f64, rising: bool) -> Option<f64> {
    if !(f < 0.0 && k > 0.0) {
        return None;
    }
    let g = |t: f64| c0 + ce * (f * t).exp() + cphi * exp_integral(-k, t) + cj * ramp_response(f, k, t);
    // `g'` itself, its sign that of the monotone `e^(k t) g'`, with `J' =
    // ∫0^t e^(f (t - s)) e^(-k s) ds` written so that no factor grows.
    let d = f + k;
    let dg = |t: f64| {
        let dj = if d > 0.0 { (f * t).exp() * exp_integral(-d, t) } else { (-k * t).exp() * exp_integral(d, t) };
        ce * f * (f * t).exp() + cphi * (-k * t).exp() + cj * dj
    };
    let horizon = 64.0 / (-f).min(k);
    let (p0, p1) = (dg(0.0), dg(horizon));
    let mut cuts = vec![0.0];
    if (p0 < 0.0 && p1 > 0.0) || (p0 > 0.0 && p1 < 0.0) {
        cuts.push(bisect(&dg, 0.0, horizon, p0 < 0.0));
    }
    cuts.push(horizon);
    for w in cuts.windows(2) {
        let (g0, g1) = (g(w[0]), g(w[1]));
        if (rising && g0 < 0.0 && g1 > 0.0) || (!rising && g0 > 0.0 && g1 < 0.0) {
            return Some(bisect(&g, w[0], w[1], rising));
        }
    }
    None
}

/// [`driven_crossing`] for a boundary that moves in a straight line, a
/// resolved pulse ramp (Sources point 3, R3 (b)): the first `t` in `(0,
/// t_max]` at which `g(t) = c0 + ce e^(f t) + cl t + cj J(f, 0, t)` crosses
/// 0 upward (`rising`) or downward, for `f < 0`. It is `driven_crossing`'s
/// function at `k = 0`, where `φ_0(t) = t`, and it holds only until the
/// ramp's next corner `t_max`, where the partition aims again. `g'(t) = ce f
/// e^(f t) + cl + cj ∫0^t e^(f s) ds` is affine in `e^(f t)`, so monotone,
/// and `g` again has at most one extremum.
pub fn ramp_crossing(c0: f64, ce: f64, cl: f64, cj: f64, f: f64, rising: bool, t_max: f64) -> Option<f64> {
    if !(f < 0.0 && t_max > 0.0 && t_max.is_finite()) {
        return None;
    }
    let g = |t: f64| c0 + ce * (f * t).exp() + cl * t + cj * ramp_response(f, 0.0, t);
    let dg = |t: f64| ce * f * (f * t).exp() + cl + cj * exp_integral(f, t);
    let (p0, p1) = (dg(0.0), dg(t_max));
    let mut cuts = vec![0.0];
    if (p0 < 0.0 && p1 > 0.0) || (p0 > 0.0 && p1 < 0.0) {
        cuts.push(bisect(&dg, 0.0, t_max, p0 < 0.0));
    }
    cuts.push(t_max);
    for w in cuts.windows(2) {
        let (g0, g1) = (g(w[0]), g(w[1]));
        if (rising && g0 < 0.0 && g1 >= 0.0) || (!rising && g0 > 0.0 && g1 <= 0.0) {
            return Some(bisect(&g, w[0], w[1], rising));
        }
    }
    None
}

/// How far the CC rail with a rail capacitor can drift from its segment while
/// the closed-form draws, which the segment holds at their value when it
/// was aimed, move (Rails point 4). After `t` those draws have moved by at
/// most `min(c_sat, c1 t) + c2 t^2` amps (a capacitor's own exponential at
/// its initial rate, saturating, and the rail's pull on it), and an amp
/// moves the rail by at most `alpha (esr_eff + t / C_rail)`: through the
/// capacitors' charge and their esr.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriftBound {
    pub c_sat: f64,
    pub c1: f64,
    pub c2: f64,
    pub esr_eff: f64,
    pub inv_c_rail: f64,
    pub alpha: f64,
}

impl DriftBound {
    fn at(&self, t: f64) -> f64 {
        self.alpha * (self.c_sat.min(self.c1 * t) + self.c2 * t * t) * (self.esr_eff + t * self.inv_c_rail)
    }

    fn time_to(&self, d: f64) -> f64 {
        if !(self.c1 > 0.0 || self.c2 > 0.0) {
            return f64::INFINITY;
        }
        let mut hi = 1e-18;
        while self.at(hi) < d {
            hi *= 2.0;
            if !hi.is_finite() {
                return f64::INFINITY;
            }
        }
        bisect(|t| self.at(t) - d, 0.0, hi, true)
    }
}

// ---- the next rail event (Rails point 4) ----------------------------------

/// How one moving term moves one threshold function.
#[derive(Debug, Clone)]
pub enum Motion {
    /// `coef * (vc(t) - vc0)` with `vc` an exponential segment.
    Exp { coef: f64, vc0: f64, inf: f64, f: f64 },
    /// A straight line, `rate * (t - t0)`.
    Line { rate: f64 },
    /// The CC rail capacitor's own segment.
    Seg(CapSeg),
    /// The CC rail capacitor's drift from its segment, a bound.
    Drift(DriftBound),
    /// The CC rail capacitor's segment coupled to its partitions: alone in
    /// its watch, which it moves by its own path, rising or falling (ruling
    /// R15 (a)).
    Path(Rc<RailModes>),
}

impl Motion {
    /// The sign of the motion: `+1` up, `-1` down, `0` still.
    fn direction(&self) -> f64 {
        let s = match self {
            &Motion::Exp { coef, vc0, inf, f } => {
                if f == 0.0 || coef == 0.0 {
                    0.0
                } else if f < 0.0 {
                    coef * (inf - vc0)
                } else {
                    coef * (vc0 - inf)
                }
            }
            &Motion::Line { rate } => rate,
            Motion::Seg(seg) => seg.slope,
            Motion::Drift(b) => {
                if b.c1 > 0.0 || b.c2 > 0.0 {
                    1.0
                } else {
                    0.0
                }
            }
            Motion::Path(m) => m.slope(0.0),
        };
        if s > 0.0 {
            1.0
        } else if s < 0.0 {
            -1.0
        } else {
            0.0
        }
    }

    /// The time after `t0` at which the term alone has moved its function by
    /// `d`, or infinity when it never does.
    fn time_to(&self, d: f64) -> f64 {
        match self {
            &Motion::Exp { coef, vc0, inf, f } => {
                let span = (vc0 - inf).abs();
                if coef == 0.0 || f == 0.0 || span == 0.0 {
                    return f64::INFINITY;
                }
                let dv = d / coef.abs();
                if f < 0.0 {
                    if dv >= span {
                        f64::INFINITY
                    } else {
                        (-dv / span).ln_1p() / f
                    }
                } else {
                    (dv / span).ln_1p() / f
                }
            }
            &Motion::Line { rate } => {
                if rate == 0.0 {
                    f64::INFINITY
                } else {
                    d / rate.abs()
                }
            }
            Motion::Seg(seg) => {
                debug_assert!(seg.ramp == 0.0, "a ramping segment is watched as its path");
                if seg.slope == 0.0 {
                    return f64::INFINITY;
                }
                let line = d / seg.slope.abs();
                if seg.k * line < RAIL_CAP_LINEAR {
                    return line;
                }
                let span = (seg.u0 - seg.u_inf).abs();
                if d >= span {
                    f64::INFINITY
                } else {
                    -(-d / span).ln_1p() / seg.k
                }
            }
            Motion::Drift(b) => b.time_to(d),
            Motion::Path(m) => {
                let (up, down) = m.passages(Some(d), Some(-d));
                up.min(down)
            }
        }
    }
}

/// One scalar function of time whose thresholds schedule the rail event.
#[derive(Debug, Clone, Default)]
pub struct Watch {
    pub value: f64,
    pub thresholds: Vec<f64>,
    /// The drift cap (`RAIL_DRIFT_V` for `u*`).
    pub cap: Option<f64>,
    pub terms: Vec<Motion>,
}

impl Watch {
    /// The target a threshold is aimed past: two reached-widths beyond it, on
    /// the far side from `value`, so a landing on it is past it however the
    /// last bit rounds (a gate tested with `>=` or a strict `>` alike).
    fn target(&self, threshold: f64) -> Option<(f64, f64)> {
        let eta = REACHED_REL * threshold.abs().max(1.0);
        let up = self.value < threshold;
        let target = if up { threshold + 2.0 * eta } else { threshold - 2.0 * eta };
        let side = if up { 1.0 } else { -1.0 };
        // Within a reached-width of the target, or already beyond it: reached.
        if side * (target - self.value) <= eta {
            return None;
        }
        Some((target, side))
    }

    /// The earliest time after `t0` (as an offset) at which a term uses its
    /// share of the distance to the nearest threshold in a direction some
    /// term moves.
    pub fn next(&self) -> f64 {
        // A coupled CC rail moves by a path that may turn: the first threshold
        // it reaches is the nearest above or the nearest below, whichever its
        // path reaches first.
        if let [Motion::Path(m)] = &self.terms[..] {
            let (mut above, mut below) = (f64::INFINITY, f64::NEG_INFINITY);
            for &th in &self.thresholds {
                if let Some((target, side)) = self.target(th) {
                    let d = target - self.value;
                    if side > 0.0 {
                        above = above.min(d);
                    } else {
                        below = below.max(d);
                    }
                }
            }
            if let Some(cap) = self.cap {
                above = above.min(cap);
                below = below.max(-cap);
            }
            let (up, down) = m.passages(above.is_finite().then_some(above), below.is_finite().then_some(below));
            return up.min(down);
        }
        let moving: Vec<(&Motion, f64)> = self.terms.iter().map(|m| (m, m.direction())).filter(|(_, d)| *d != 0.0).collect();
        if moving.is_empty() {
            return f64::INFINITY;
        }
        let ups = moving.iter().any(|(_, d)| *d > 0.0);
        let downs = moving.iter().any(|(_, d)| *d < 0.0);
        let mut distance = self.cap.unwrap_or(f64::INFINITY);
        for &th in &self.thresholds {
            if let Some((target, side)) = self.target(th) {
                if (side > 0.0 && ups) || (side < 0.0 && downs) {
                    distance = distance.min((target - self.value).abs());
                }
            }
        }
        if !distance.is_finite() {
            return f64::INFINITY;
        }
        let share = distance / moving.len() as f64;
        moving.iter().map(|(m, _)| m.time_to(share)).fold(f64::INFINITY, f64::min)
    }
}

/// The earliest of every watch's next time, as an absolute instant, or
/// `None` when nothing moves toward a threshold. The event is due at the
/// earliest `t > t0 + 1e-15` (Rails point 4): a threshold some term would
/// reach sooner, as successive events closing on it geometrically do, is
/// still ahead, so its event is the first instant the kernel accepts rather
/// than none, which would leave the rail with no event at all.
pub fn next_event(t0: f64, watches: &[Watch]) -> Option<f64> {
    let dt = watches.iter().map(Watch::next).fold(f64::INFINITY, f64::min);
    if !dt.is_finite() {
        return None;
    }
    let floor = t0 + 1e-15;
    let t = t0 + dt;
    Some(if t > floor { t } else { floor.next_up() })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn none(_u: f64) -> (f64, f64) {
        (0.0, 0.0)
    }

    #[test]
    fn a_linear_battery_rail_is_the_closed_form_in_one_pass() {
        // E = 5, r = 1, loads 100 and 50 ohm as two contributions at u_p = 5:
        // I = u_p/R, g = 1/R, so A0 = 0 and G = 0.03.
        let (a_p, g_p) = (5.0 / 100.0 - 5.0 / 100.0 + 5.0 / 50.0 - 5.0 / 50.0, 1.0 / 100.0 + 1.0 / 50.0);
        let u = solve_battery(5.0, 1.0, a_p, g_p, &none, 5.0).unwrap();
        let want = 5.0 * (100.0 * 50.0 / 150.0) / (100.0 * 50.0 / 150.0 + 1.0);
        assert!((u - want).abs() < 1e-12, "{u} vs {want}");
    }

    #[test]
    fn an_ideal_battery_rail_is_the_emf() {
        assert_eq!(solve_battery(4.2, 0.0, 3.0, 1.0, &none, 0.0).unwrap(), 4.2);
    }

    #[test]
    fn a_direct_device_is_iterated_to_the_newton_tolerance() {
        // A 10 ohm direct resistor to ground on a 1 ohm battery.
        let direct = |u: f64| (u / 10.0, 0.1);
        let u = solve_battery(5.0, 1.0, 0.0, 0.0, &direct, 0.0).unwrap();
        assert!((u - 5.0 * 10.0 / 11.0).abs() < 1e-12);
    }

    #[test]
    fn the_cc_rail_solves_the_draw_to_the_limit() {
        // 0.1 A into 10 ohm: 1 V.
        let u = solve_cc(5.0, 0.1, 0.0, 0.1, &none, 5.0).unwrap();
        assert!((u - 1.0).abs() < 1e-15);
    }

    #[test]
    fn a_cc_rail_newton_past_the_setpoint_that_cannot_settle_gives_the_setpoint() {
        // 63 mA into the node shunt alone, a reverse LED's 3e-16 A rounding
        // one way or the other with the iterate's last bit: the iterates sit
        // one ulp apart at 3.1e10 V (F5f). With the draw at the setpoint
        // within the limit, the setpoint.
        let leak = |u: f64| (if u.to_bits() & 1 == 0 { 3.330669e-16 } else { 3.191891e-16 }, 0.0);
        assert_eq!(solve_cc(6.32, 0.063, 0.0, 2e-12, &leak, 0.12), Ok(6.32));
        // A draw past the limit at the setpoint keeps the Newton's failure.
        let past = |u: f64| if u == 6.32 { (1.0, 0.0) } else { leak(u) };
        assert!(solve_cc(6.32, 0.063, 0.0, 2e-12, &past, 0.12).is_err());
    }

    #[test]
    fn the_cc_entry_takes_the_highest_root_at_or_below_the_setpoint_and_lands_on_a_power_edge_as_an_error() {
        // Ruling G on the scalar draw, mirrored for a negative setpoint. A
        // chip inside [4.275, 5.775] V sinking 32 mA beside 125 ohm, on a 5 V
        // 50 mA PSU: 72 mA at the setpoint, at least 66.2 mA through the
        // window, at most 34.2 mA under it: no root at or below the
        // setpoint, the draw crossing the limit at the window's floor. The
        // Newton from the setpoint settles past it (6.25 V, unpowered).
        for dir in [1.0, -1.0] {
            let window = |u: f64| (4.275..=5.775).contains(&(dir * u));
            let edges = [dir * 4.275, dir * 5.775];
            let sink = move |u: f64| (u / 125.0 + if window(u) { dir * 0.032 } else { 0.0 }, 1.0 / 125.0);
            let i_at_v = sink(dir * 5.0).0;
            assert!((solve_cc(dir * 5.0, dir * 0.05, 0.0, 0.0, &sink, dir * 5.0).unwrap() - dir * 6.25).abs() < 1e-12, "the Newton's root past the setpoint");
            let landed = cc_entry_rail(dir * 5.0, dir * 0.05, 0.0, 0.0, &sink, i_at_v, &edges);
            assert!(landed.as_ref().is_err_and(|e| e.starts_with(EDGE_LANDING)), "dir {dir}: {landed:?}");
            // The last three evaluations straddle the edge: the hold's trigger
            // reads the chip flipping at each.
            let trace = std::cell::RefCell::new(Vec::new());
            let traced = |u: f64| {
                trace.borrow_mut().push(u);
                sink(u)
            };
            let _ = cc_entry_rail(dir * 5.0, dir * 0.05, 0.0, 0.0, &traced, i_at_v, &edges);
            let t = trace.borrow();
            let on: Vec<bool> = t[t.len() - 3..].iter().map(|&u| window(u)).collect();
            assert_eq!(on, [true, false, true], "dir {dir}");
            // Two roots: 50 ohm and a chip feeding 0.1634 / u - 0.032 inside the
            // window, on 5.7 V 80 mA. Powered, u^2 - 2.4 u - 8.17 = 0 at 4.3 V;
            // unpowered, 4 V. The concave powered draw sends the Newton from
            // the setpoint under the window, to the lower root.
            let feed = move |u: f64| {
                let w = dir * u;
                let (i, g) = if window(u) { (w / 50.0 + 0.032 - 0.1634 / w, 1.0 / 50.0 + 0.1634 / (w * w)) } else { (w / 50.0, 1.0 / 50.0) };
                (dir * i, g)
            };
            let i_at_v = feed(dir * 5.7).0;
            assert!((solve_cc(dir * 5.7, dir * 0.08, 0.0, 0.0, &feed, dir * 5.7).unwrap() - dir * 4.0).abs() < 1e-12, "the Newton's lower root");
            let u = cc_entry_rail(dir * 5.7, dir * 0.08, 0.0, 0.0, &feed, i_at_v, &edges).unwrap();
            assert!((u - dir * 4.3).abs() < 1e-12, "dir {dir}: the highest root, got {u}");
            // With no edge between its root and the setpoint, the Newton's root
            // itself, to the bit.
            let linear = |u: f64| (u / 50.0, 1.0 / 50.0);
            let newton_root = solve_cc(dir * 5.7, dir * 0.08, 0.0, 0.0, &linear, dir * 5.7).unwrap();
            assert_eq!(cc_entry_rail(dir * 5.7, dir * 0.08, 0.0, 0.0, &linear, linear(dir * 5.7).0, &[dir * 3.0, dir * 6.0]).unwrap().to_bits(), newton_root.to_bits());
        }
    }

    #[test]
    fn an_edge_at_the_setpoint_lands_a_flat_edge_by_the_root_does_not_and_a_landing_straddles_its_edge() {
        for dir in [1.0, -1.0] {
            // An edge at the setpoint itself: a chip powered from 4.275 V up
            // sinking 32 mA beside 125 ohm, on a 4.275 V, 50 mA PSU: 66.2 mA
            // at the setpoint, 34.2 mA just under it: a landing there.
            let floor = 4.75 * 0.9;
            let window = move |u: f64| (dir * u) >= floor;
            let sink = move |u: f64| (u / 125.0 + if window(u) { dir * 0.032 } else { 0.0 }, 1.0 / 125.0);
            let trace = std::cell::RefCell::new(Vec::new());
            let traced = |u: f64| {
                trace.borrow_mut().push(u);
                sink(u)
            };
            let landed = cc_entry_rail(dir * floor, dir * 0.05, 0.0, 0.0, &traced, sink(dir * floor).0, &[dir * floor]);
            assert!(landed.as_ref().is_err_and(|e| e.starts_with(EDGE_LANDING)), "dir {dir}: an edge at the setpoint is a landing: {landed:?}");
            // Its band reaches the setpoint, so `up` is read only after `dn`:
            // the last three evaluations still straddle it.
            let t = trace.borrow();
            let on: Vec<bool> = t[t.len() - 3..].iter().map(|&u| window(u)).collect();
            assert_eq!(on, [true, false, true], "dir {dir}");
            // A flat edge (nothing jumps there) 1e-13 V above the Newton's
            // root: the root stands, to the bit.
            let linear = |u: f64| (u / 50.0, 1.0 / 50.0);
            let root = solve_cc(dir * 5.7, dir * 0.08, 0.0, 0.0, &linear, dir * 5.7).unwrap();
            let got = cc_entry_rail(dir * 5.7, dir * 0.08, 0.0, 0.0, &linear, linear(dir * 5.7).0, &[root + dir * 1e-13]);
            assert_eq!(got.map(f64::to_bits), Ok(root.to_bits()), "dir {dir}");
        }
    }

    #[test]
    fn a_single_exponential_term_lands_on_its_threshold() {
        // vc from 0 toward 5 at rate -1000/s, f = 2 vc: the threshold 4 is
        // reached at vc = 2, t = ln(5/3)/1000.
        let w = Watch { value: 0.0, thresholds: vec![4.0], cap: None, terms: vec![Motion::Exp { coef: 2.0, vc0: 0.0, inf: 5.0, f: -1000.0 }] };
        let t = w.next();
        let vc = 5.0 - 5.0 * (-1000.0 * t).exp();
        assert!((2.0 * vc - (4.0 + 2.0 * REACHED_REL * 4.0)).abs() < 1e-14, "lands past the threshold by two reached-widths");
    }

    #[test]
    fn a_threshold_behind_every_moving_term_is_ignored() {
        let w = Watch { value: 3.0, thresholds: vec![2.9999999], cap: Some(1e-5), terms: vec![Motion::Line { rate: 1.0 }] };
        assert!((w.next() - 1e-5).abs() < 1e-18, "only the drift cap applies going up");
    }

    #[test]
    fn two_terms_take_half_the_distance_each() {
        let w = Watch { value: 0.0, thresholds: vec![], cap: Some(1e-5), terms: vec![Motion::Line { rate: 1.0 }, Motion::Line { rate: 2.0 }] };
        assert!((w.next() - 0.25e-5).abs() < 1e-18);
    }

    #[test]
    fn a_capacitors_demand_against_a_back_feed_keeps_cv() {
        // FXI, bug D: {5 V, 0.1 A}, a 1 mF capacitor (esr 5 ohm) at 0.11 V,
        // so 0.978 A of demand at the setpoint, and a 2.2 ohm path to an 8 V
        // source, -1.36 A at the setpoint. The total, -0.38 A, is past the
        // limit against the PSU's direction: the CC solution at the instant,
        // `u = 0.11 + 5 (0.1 - (u - 8) / 2.2)`, is 5.74 V, past the
        // setpoint, so the clamp keeps CV. The same
        // capacitor with the source at 0 V (2.27 A + 0.978 A) enters CC. A
        // negative setpoint mirrors both.
        let d = CapDemand::of(5.0, 1e-3, [(1e-3, 5.0, 5.0 - 0.11)]);
        let fed = |u: f64| ((u - 8.0) / 2.2, 1.0 / 2.2);
        assert_eq!(next_regime(Regime::Cv, 5.0, 0.1, true, 0.0, 0.0, &fed, 5.0, d, &Vec::new).unwrap().0, Regime::Cv);
        let grounded = |u: f64| (u / 2.2, 1.0 / 2.2);
        assert_eq!(next_regime(Regime::Cv, 5.0, 0.1, true, 0.0, 0.0, &grounded, 5.0, d, &Vec::new).unwrap().0, Regime::Cc);
        let d = CapDemand::of(-5.0, 1e-3, [(1e-3, 5.0, 5.0 - 0.11)]);
        let fed = |u: f64| ((u + 8.0) / 2.2, 1.0 / 2.2);
        assert_eq!(next_regime(Regime::Cv, -5.0, 0.1, true, 0.0, 0.0, &fed, -5.0, d, &Vec::new).unwrap().0, Regime::Cv);
        let grounded = |u: f64| (u / 2.2, 1.0 / 2.2);
        assert_eq!(next_regime(Regime::Cv, -5.0, 0.1, true, 0.0, 0.0, &grounded, -5.0, d, &Vec::new).unwrap().0, Regime::Cc);
    }

    #[test]
    fn a_rail_capacitor_below_the_setpoint_is_demand_and_one_at_it_adds_nothing() {
        // {5 V, 0.1 A} with only the rail node's shunt: 5e-12 A at V.
        let shunt = |u: f64| (1e-12 * u, 1e-12);
        let step = |demand: CapDemand| next_regime(Regime::Cv, 5.0, 0.1, true, 0.0, 0.0, &shunt, 5.0, demand, &Vec::new).unwrap().0;
        // Discharged, esr 0: unbounded, and the rail it holds is 0 V. The CC
        // candidate without it is 1e11 V, which the clamp alone would keep CV.
        let d = CapDemand::of(5.0, 1e-4, [(1e-4, 0.0, 5.0)]);
        assert_eq!(d, CapDemand { draw: f64::INFINITY, held: 0.0 });
        assert_eq!(step(d), Regime::Cc);
        // Charged to the setpoint, or above it: nothing.
        for deficit in [0.0, -1.0] {
            let d = CapDemand::of(5.0, 1e-4, [(1e-4, 0.0, deficit)]);
            assert_eq!(d, CapDemand::none(5.0), "deficit {deficit}");
            assert_eq!(step(d), Regime::Cv);
        }
        // esr 0.05: 100 A at the setpoint.
        let d = CapDemand::of(5.0, 1e-4, [(1e-4, 0.05, 5.0)]);
        assert!((d.draw - 100.0).abs() < 1e-12);
        assert_eq!(step(d), Regime::Cc);
        // esr 100: 50 mA, inside the limit with no load...
        let d = CapDemand::of(5.0, 1e-4, [(1e-4, 100.0, 5.0)]);
        assert_eq!(step(d), Regime::Cv);
        // ...and over it with 60 ohm (83.3 mA) beside it, although the load
        // alone is inside it.
        let load = |u: f64| (u / 60.0, 1.0 / 60.0);
        assert_eq!(next_regime(Regime::Cv, 5.0, 0.1, true, 0.0, 0.0, &load, 5.0, CapDemand::none(5.0), &Vec::new).unwrap().0, Regime::Cv);
        assert_eq!(next_regime(Regime::Cv, 5.0, 0.1, true, 0.0, 0.0, &load, 5.0, d, &Vec::new).unwrap().0, Regime::Cc);
        // Two capacitors share their charge at the rail; one above the setpoint
        // counts at V: 1 uF short by 4 V and 3 uF at it hold 5 - 1 = 4 V.
        let d = CapDemand::of(5.0, 4e-6, [(1e-6, 0.0, 4.0), (3e-6, 0.0, -2.0)]);
        assert_eq!(d.held, 4.0);
        // A negative setpoint: the deficit is in its direction, and so is the
        // rail it holds.
        let d = CapDemand::of(-5.0, 1e-4, [(1e-4, 0.0, 5.0)]);
        assert_eq!(d.held, 0.0);
        assert_eq!(next_regime(Regime::Cv, -5.0, 0.1, true, 0.0, 0.0, &shunt, -5.0, d, &Vec::new).unwrap().0, Regime::Cc);
    }

    #[test]
    fn a_threshold_closer_than_the_time_floor_still_has_an_event() {
        // Two terms close on 0.02 V geometrically; at 1e7 V/s the rail term's
        // share of a 1e-8 V distance is 5e-16 s away, under the kernel's floor
        // at t0 = 2e-9. The threshold is still ahead, so the event is the
        // first instant the kernel accepts.
        let t0 = 2e-9;
        let w = Watch { value: 0.02 - 1e-8, thresholds: vec![0.02], cap: None, terms: vec![Motion::Line { rate: 1e7 }, Motion::Line { rate: 1.0 }] };
        assert!(w.next() < 1e-15);
        let t = next_event(t0, &[w]).expect("an event");
        assert!(t > t0 + 1e-15 && t == (t0 + 1e-15).next_up(), "{t:e}");
        // Nothing moving toward any threshold: no event.
        let still = Watch { value: 0.0, thresholds: vec![1.0], cap: Some(1e-5), terms: vec![Motion::Line { rate: 0.0 }] };
        assert_eq!(next_event(t0, &[still]), None);
    }

    /// `∫0^t e^(f (t - s)) φ_k(s) ds` by composite Simpson, the reference for
    /// [`ramp_response`].
    fn ramp_response_by_quadrature(f: f64, k: f64, t: f64) -> f64 {
        let n = 20_000;
        let h = t / f64::from(n);
        let g = |s: f64| (f * (t - s)).exp() * exp_integral(-k, s);
        let mut sum = g(0.0) + g(t);
        for i in 1..n {
            sum += g(f64::from(i) * h) * if i % 2 == 1 { 4.0 } else { 2.0 };
        }
        sum * h / 3.0
    }

    #[test]
    fn the_ramp_response_is_its_integral_at_every_rate_and_through_resonance() {
        // Apart, at resonance, a hair off it either side, a nearly still rail,
        // a slow capacitor, a long and a nanosecond horizon.
        let cases = [
            (-1e3, 2e3, 1e-3),
            (-1e3, 1e3, 1e-3),
            (-1e3, 1e3 * (1.0 + 1e-9), 1e-3),
            (-1e3, 1e3 * (1.0 - 1e-6), 2e-3),
            (-1e3, 1e-8, 1e-3),
            (-1e-3, 5.0, 2.0),
            (-50.0, 0.0, 0.1),
            (-2e4, 2e3, 1e-9),
            (-0.9, 2e3, 5e-7),
            (-3e4, 2e3, 5e-7),
        ];
        for (f, k, t) in cases {
            let (got, want) = (ramp_response(f, k, t), ramp_response_by_quadrature(f, k, t));
            assert!((got - want).abs() <= 1e-10 * want.abs(), "J({f}, {k}, {t}) = {got:e}, quadrature {want:e}");
        }
        // The confluent ends in closed form: at resonance `∫0^t s e^(f s) ds`,
        // and on a still rail `(∫0^t e^(f s) ds - t) / f`.
        let (f, t): (f64, f64) = (-1e3, 1.5e-3);
        let resonance = (t * (f * t).exp() - exp_integral(f, t)) / f;
        assert!((ramp_response(f, -f, t) - resonance).abs() <= 1e-13 * resonance.abs());
        let still = (exp_integral(f, t) - t) / f;
        assert!((ramp_response(f, 0.0, t) - still).abs() <= 1e-12 * still.abs());
        // From a standing start it is t^2 / 2.
        assert!((ramp_response(-1.0, 1.0, 1e-9) - 0.5e-18).abs() <= 1e-27);
    }

    #[test]
    fn the_ramp_charge_is_the_integral_of_the_ramp_response() {
        for (f, k, t) in [(-1e3, 2e3, 1e-3), (-1e3, 1e3, 1e-3), (-1e3, 1e3 * (1.0 + 1e-9), 1e-3), (-1e3, 1e-8, 1e-3), (-50.0, 0.0, 0.1), (-0.9, 2e3, 5e-7)] {
            let n = 20_000;
            let h = t / f64::from(n);
            let mut sum = ramp_response(f, k, 0.0) + ramp_response(f, k, t);
            for i in 1..n {
                sum += ramp_response(f, k, f64::from(i) * h) * if i % 2 == 1 { 4.0 } else { 2.0 };
            }
            let (got, want) = (ramp_charge(f, k, t), sum * h / 3.0);
            assert!((got - want).abs() <= 1e-10 * want.abs(), "K({f}, {k}, {t}) = {got:e}, quadrature {want:e}");
        }
        // From a standing start it is t^3 / 6.
        let small = ramp_charge(-1.0, 1.0, 1e-6);
        assert!((small - 1e-18 / 6.0).abs() <= 1e-6 * 1e-18 / 6.0, "{small:e}");
    }

    #[test]
    fn a_driven_crossing_is_the_first_root_on_either_monotone_piece() {
        // A 1 ms RC following a rail ramping from 0 at 1000 V/s (k = 10), against
        // a threshold at 0.34 u: the threshold outruns the node at first, then
        // the node's lag settles under 0.66 u and it crosses upward. A scan at
        // 1 ns finds the same root.
        let (f, k, s_a) = (-1e3, 10.0, 1e3);
        let (cphi, cj) = (-0.34 * s_a, -f * s_a);
        let g = |t: f64| cphi * exp_integral(-k, t) + cj * ramp_response(f, k, t);
        let t = driven_crossing(0.0, 0.0, cphi, cj, f, k, true).expect("a rising crossing");
        assert!(g(t) >= 0.0 && g(t.next_down()) < 0.0, "the first bit past the root: {t:e}");
        // g < 0 from 0 until the root, near 0.9 ms.
        let scan = (500_000..10_000_000u32).map(|i| f64::from(i) * 1e-9).find(|&s| g(s) >= 0.0).unwrap();
        assert!((t - scan).abs() <= 1e-9, "{t:e} against the scan's {scan:e}");
        assert_eq!(driven_crossing(0.0, 0.0, cphi, cj, f, k, false), None, "starting on the threshold and falling away is no crossing");
        // Below a fixed point that a rising threshold later overtakes: up on
        // the first piece, down on the second.
        let (c0, ce, cphi) = (0.4, -0.5, -50.0);
        let g = |t: f64| c0 + ce * (f * t).exp() + cphi * exp_integral(-k, t);
        let up = driven_crossing(c0, ce, cphi, 0.0, f, k, true).unwrap();
        let down = driven_crossing(c0, ce, cphi, 0.0, f, k, false).unwrap();
        assert!(g(up) >= 0.0 && g(up.next_down()) < 0.0 && up < 1e-3, "{up:e}");
        assert!(g(down) <= 0.0 && g(down.next_down()) > 0.0 && down > 1e-3, "{down:e}");
    }

    #[test]
    fn the_drift_bound_reaches_its_budget_at_its_time() {
        let b = DriftBound { c_sat: 1e-3, c1: 1.0, c2: 500.0, esr_eff: 0.05, inv_c_rail: 1e4, alpha: 0.999 };
        let t = b.time_to(RAIL_DRIFT_V);
        assert!((b.at(t) - RAIL_DRIFT_V).abs() <= 1e-12 * RAIL_DRIFT_V && b.at(0.5 * t) < RAIL_DRIFT_V, "{t:e}");
        let still = DriftBound { c_sat: 0.0, c1: 0.0, c2: 0.0, ..b };
        assert_eq!(still.time_to(RAIL_DRIFT_V), f64::INFINITY);
    }

    #[test]
    fn the_cap_segment_is_a_line_then_an_exponential() {
        let seg = CapSeg { t0: 0.0, u0: 0.0, u_inf: 1e11, k: 1e-8, slope: 1000.0, alpha: 1.0, beta: 0.0, ramp: 0.0, cf_frozen: 0.0, cf_count: 0, modes: None };
        assert_eq!(seg.at(1e-3), 1.0);
        let w = Watch { value: 0.0, thresholds: vec![5.0], cap: None, terms: vec![Motion::Seg(seg.clone())] };
        let t = w.next();
        assert!((seg.at(t) - 5.0 - 2.0 * REACHED_REL * 5.0).abs() < 1e-12);
    }

    #[test]
    fn the_cap_segment_rounds_at_the_rail_not_at_its_asymptote() {
        // A 74LS00 on a 100 uF rail at 0.1 A: u_inf near 1.4e7 V, whose last
        // bit is 1.9e-9 V. 20 us on, k dt = 1.4e-9 is past the line, and the
        // rail is the series `slope (dt - k dt^2 / 2 + k^2 dt^3 / 6)` (the
        // next term is 1e-27 of it) to an ulp of 5 V.
        let (u0, slope, k) = (4.98, 1000.0, 7.0e-5);
        let seg = CapSeg { t0: 0.0, u0, u_inf: u0 + slope / k, k, slope, alpha: 1.0, beta: 0.0, ramp: 0.0, cf_frozen: 0.0, cf_count: 0, modes: None };
        let dt = 2e-5;
        assert!(k * dt >= RAIL_CAP_LINEAR);
        let want = u0 + slope * (dt - k * dt * dt / 2.0 + k * k * dt * dt * dt / 6.0);
        assert!((seg.at(dt) - want).abs() <= 2e-15, "{:e} off", seg.at(dt) - want);
        assert!((seg.slope_at(dt) - slope * (1.0 - k * dt)).abs() <= 1e-12, "{:e}", seg.slope_at(dt));
    }

    /// The coupled CC rail's own linear system (`coupled_modes`' doc
    /// comment), integrated by RK4 in steps of at most `h` from rest, read at
    /// each of `times` (ascending): the rail `u = alpha q - alpha esr sum b
    /// vc` and the charge `q`, as increments.
    #[allow(clippy::too_many_arguments)]
    fn coupled_rk4(c_rail: f64, esr: f64, alpha: f64, k_q: f64, r0_q: f64, parts: &[Coupled], times: &[f64], h: f64) -> Vec<(f64, f64)> {
        let n = parts.len();
        let rail = |z: &[f64]| alpha * z[0] - alpha * esr * parts.iter().enumerate().map(|(i, p)| p.b * z[i + 1]).sum::<f64>();
        let rhs = |z: &[f64]| -> Vec<f64> {
            let u = rail(z);
            let mut d = vec![r0_q - k_q * z[0] - (alpha / c_rail) * parts.iter().enumerate().map(|(i, p)| p.b * z[i + 1]).sum::<f64>()];
            d.extend(parts.iter().enumerate().map(|(i, p)| p.f * z[i + 1] + p.gamma * u + p.r0));
            d
        };
        let add = |z: &[f64], k: &[f64], s: f64| z.iter().zip(k).map(|(a, b)| a + s * b).collect::<Vec<f64>>();
        let (mut z, mut t0, mut out) = (vec![0.0; n + 1], 0.0, Vec::new());
        for &t in times {
            let steps = ((t - t0) / h).ceil().max(1.0) as usize;
            let dt = (t - t0) / steps as f64;
            for _ in 0..steps {
                let k1 = rhs(&z);
                let k2 = rhs(&add(&z, &k1, 0.5 * dt));
                let k3 = rhs(&add(&z, &k2, 0.5 * dt));
                let k4 = rhs(&add(&z, &k3, dt));
                for j in 0..=n {
                    z[j] += dt * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]) / 6.0;
                }
            }
            t0 = t;
            out.push((rail(&z), z[0]));
        }
        out
    }

    #[test]
    fn the_coupled_cc_rail_is_its_systems_exact_solution() {
        // A 10 uF rail at 1 mS ramping from a 1 mA limit, against: 1 uF behind
        // 50 ohm (2e4/s, comparable to the rail), 10 nF behind 1 kohm (1e5/s)
        // still settling, and a partition whose draw moves but which follows
        // no rail (a forcing at its own rate); at esr 0 and at 0.05 ohm.
        let gs = 1e-12;
        let rc = |r: f64, c: f64, lag: f64| Coupled { f: -(1.0 / r + gs) / c, gamma: 1.0 / (r * c), b: -1.0 / r, r0: lag / (r * c) };
        let mut parts = vec![rc(50.0, 1e-6, 0.1), rc(1e3, 1e-8, 0.5)];
        parts.push(Coupled { f: -3e3, gamma: 0.0, b: -2e-3, r0: 40.0 });
        let times = [1e-7, 3e-6, 4e-5, 7e-4, 2e-2];
        for esr in [0.0, 0.05] {
            let (c_rail, g) = (1e-5, 1e-3 + 1.0 / 50.0 + 1.0 / 1e3);
            let alpha = 1.0 / (1.0 + g * esr);
            let (k_q, r0_q) = (alpha * g / c_rail, 100.0);
            let modes = coupled_modes(c_rail, esr, alpha, k_q, r0_q, &parts).expect("modes");
            // The rail and two followers are three states; the forcing adds one.
            assert_eq!(modes.len(), 4);
            let m = RailModes::new(modes, 0.0, (esr > 0.0).then(|| (k_q + alpha / (esr * c_rail), r0_q, 1.0 / (esr * c_rail))));
            for (&t, &(u, q)) in times.iter().zip(&coupled_rk4(c_rail, esr, alpha, k_q, r0_q, &parts, &times, 2e-8)) {
                let (gu, gq) = (m.du(t), m.charge(0.0, t));
                assert!((gu - u).abs() <= 1e-8 * u.abs().max(1e-6), "esr {esr} t {t:e}: rail {gu:e} against {u:e}");
                assert!((gq - q).abs() <= 1e-8 * q.abs().max(1e-6), "esr {esr} t {t:e}: charge {gq:e} against {q:e}");
            }
        }
        // A partition whose coupling is not passive is not a modal path.
        assert_eq!(coupled_modes(1e-5, 0.0, 1.0, 100.0, 100.0, &[Coupled { f: -1e3, gamma: 1e3, b: 1e-3, r0: 0.0 }]), None);
    }

    #[test]
    fn the_coupled_cell_rail_is_its_systems_exact_solution() {
        // A 0.5 ohm cell (kappa about -0.49) draining at -2 mV/s, against: a
        // 1 uF behind 50 ohm (2e4/s) and a 10 nF behind 1 kohm (1e5/s), each
        // charging from the rail (b < 0, gamma > 0: c > 0, the rail rising
        // as they fill), a partition weakly coupled to within 1e-6 of its own
        // rate, and one that moves its draw but follows no rail. Against the
        // same system by RK4: `e' = f e + gamma du + r0`, `du = rho τ + kappa
        // sum b e`.
        let gs = 1e-12;
        let rc = |r: f64, c: f64, lag: f64| Coupled { f: -(1.0 / r + gs) / c, gamma: 1.0 / (r * c), b: -1.0 / r, r0: lag / (r * c) };
        let mut parts = vec![rc(50.0, 1e-6, 0.1), rc(1e3, 1e-8, 0.5), Coupled { f: -3e3, gamma: 1e-3, b: -1e-6, r0: 2.0 }];
        parts.push(Coupled { f: -7e2, gamma: 0.0, b: -2e-3, r0: 40.0 });
        let (kappa, rho) = (-0.5 / (1.0 + 0.5 * (1.0 / 50.0 + 1e-3 + 1e-2)), -2e-3);
        let modes = cell_modes(kappa, rho, &parts).expect("modes");
        // Three followers, the line, and the forcing.
        assert_eq!(modes.len(), 5);
        let du = |e: &[f64], t: f64| rho * t + kappa * parts.iter().zip(e).map(|(p, x)| p.b * x).sum::<f64>();
        let times = [1e-7, 3e-6, 4e-5, 7e-4, 2e-2];
        let (mut e, mut t0) = (vec![0.0; parts.len()], 0.0f64);
        let h: f64 = 2e-8;
        for &t in &times {
            let steps = ((t - t0) / h).ceil().max(1.0) as usize;
            let dt = (t - t0) / steps as f64;
            for i in 0..steps {
                let ts = t0 + i as f64 * dt;
                let rhs = |e: &[f64], tt: f64| -> Vec<f64> {
                    let u = du(e, tt);
                    parts.iter().zip(e).map(|(p, x)| p.f * x + p.gamma * u + p.r0).collect()
                };
                let add = |a: &[f64], k: &[f64], s: f64| a.iter().zip(k).map(|(x, y)| x + s * y).collect::<Vec<f64>>();
                let k1 = rhs(&e, ts);
                let k2 = rhs(&add(&e, &k1, 0.5 * dt), ts + 0.5 * dt);
                let k3 = rhs(&add(&e, &k2, 0.5 * dt), ts + 0.5 * dt);
                let k4 = rhs(&add(&e, &k3, dt), ts + dt);
                for j in 0..e.len() {
                    e[j] += dt * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]) / 6.0;
                }
            }
            t0 = t;
            let want = du(&e, t);
            let got: f64 = modes.iter().map(|&(k, s)| s * exp_integral(-k, t)).sum();
            assert!((got - want).abs() <= 1e-8 * want.abs().max(1e-9), "t {t:e}: path {got:e} against {want:e}");
        }
        // Couplings of both signs are no modal path; neither is an unstable one.
        let mixed = [Coupled { f: -1e3, gamma: 1e3, b: -1e-3, r0: 0.0 }, Coupled { f: -2e3, gamma: 1e3, b: 1e-3, r0: 0.0 }];
        assert_eq!(cell_modes(-0.5, 0.0, &mixed), None);
        assert_eq!(cell_modes(-1e3, 0.0, &[Coupled { f: -1.0, gamma: 1e3, b: -1e-3, r0: 0.0 }]), None);
    }

    #[test]
    fn a_sum_of_monotone_terms_crosses_where_a_scan_finds_it() {
        // Two rail modes that turn the sum back: up fast, down slow, through a
        // level twice; and a capacitor that follows them past a threshold.
        let terms = [(2.0, Basis::Phi(1e4)), (-1.0, Basis::Phi(10.0))];
        let s = ExpSum { c0: -2e-5, terms: &terms };
        let t = s.first_crossing(true).expect("up");
        let scan = (1..2_000_000u32).map(|i| f64::from(i) * 1e-9).find(|&x| s.at(x) >= 0.0).unwrap();
        assert!((t - scan).abs() <= 1e-9 && s.at(t) >= 0.0, "{t:e} against {scan:e}");
        let t2 = s.first_crossing(false).expect("back down");
        assert!(t2 > t && s.at(t2) <= 0.0 && s.at(0.5 * (t + t2)) > 0.0, "{t2:e}");
        // An exponential, a mode and a capacitor's response to it: the
        // crossing near 2.2e-4 s, where 1500 φ_5e3 reaches about 0.2.
        let terms = [(-0.7, Basis::Exp(-2e5)), (1500.0, Basis::Phi(5e3)), (2e5, Basis::J(-2e5, 5e3))];
        let s = ExpSum { c0: -0.2, terms: &terms };
        let t = s.first_crossing(true).expect("a crossing");
        let scan = (1..500_000u32).map(|i| f64::from(i) * 1e-9).find(|&x| s.at(x) >= 0.0).unwrap();
        assert!((t - scan).abs() <= 1e-9, "{t:e} against {scan:e}");
        // A level the sum never reaches.
        let terms = [(1.0, Basis::Phi(1e3))];
        assert_eq!(ExpSum { c0: -2e-3, terms: &terms }.first_crossing(true), None);
    }

    #[test]
    fn the_merged_search_finds_the_first_crossing_a_dense_scan_finds() {
        // Sums shaped as a driven node less a threshold riding the same rail
        // (each mode's line and the capacitor's response to it nearly
        // cancelling, as in corpus 07's switch-on), resonant pairs within a
        // few parts per thousand and within 1e-8 (a weakly coupled
        // partition's own mode), a line mode, and levels on either side.
        // The windowed search is no reference here: on such sums its window
        // bounds cancel so badly that its bisection branches without end.
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let (mut found, mut checked) = (0, 0);
        for case in 0..1500 {
            let f = -(10f64).powf(-1.0 + 6.0 * rnd());
            let modes = 1 + (rnd() * 4.0) as usize;
            let mut terms = Vec::new();
            if rnd() < 0.8 {
                terms.push(((rnd() - 0.5) * 2.0, Basis::Exp(f)));
            }
            let (sx, dth, dx) = (rnd(), rnd(), rnd() - 0.2);
            let mut slow = -f;
            for j in 0..modes {
                let k = match (case + j) % 5 {
                    0 => -f * (1.0 + (rnd() - 0.5) * if case % 2 == 0 { 0.01 } else { 1e-8 }),
                    1 => 0.0,
                    _ => (10f64).powf(-2.0 + 7.0 * rnd()),
                };
                if k > 0.0 {
                    slow = slow.min(k);
                }
                let s = (rnd() - 0.4) * (10f64).powf(6.0 * rnd());
                let w = (sx + 0.3 * rnd()) * s * -f;
                terms.push(((sx - dth) * s, Basis::Phi(k)));
                terms.push((dx * w, Basis::J(f, k)));
            }
            let c0 = (rnd() - 0.5) * (10f64).powf(-3.0 + 3.0 * rnd());
            let sum = ExpSum { c0, terms: &terms };
            let scale = |t: f64| c0.abs() + terms.iter().map(|&(c, b)| (c * b.at(t)).abs()).sum::<f64>();
            for rising in [true, false] {
                let dir = if rising { 1.0 } else { -1.0 };
                let g = |t: f64| dir * sum.at(t);
                let got = sum.first_crossing(rising);
                checked += 1;
                // A dense scan up to the crossing (or past every rate's
                // settling): the first sample at or past 0 after one below.
                let end = got.unwrap_or(200.0 / slow).min(1e6);
                let mut armed = g(0.0) < 0.0;
                let mut first = None;
                for i in 1..=40_000 {
                    let x = i as f64 / 40_000.0;
                    let t = end * if i < 20_000 { (10f64).powf(-12.0 * (1.0 - 2.0 * x)) * 0.5 } else { x };
                    let v = g(t);
                    let tol = 1e-9 * scale(t);
                    if !armed {
                        armed = v < -tol;
                    } else if v > tol && t < end * (1.0 - 1e-9) {
                        first = Some(t);
                        break;
                    }
                }
                match got {
                    Some(t) => {
                        found += 1;
                        assert!(g(t) >= 0.0 && g(t.next_down()) < 0.0 || g(t).abs() <= 1e-12 * scale(t), "case {case} rising {rising}: {t:e} is no root {terms:?} c0 {c0:e}");
                        assert!(first.is_none(), "case {case} rising {rising}: crossed at {:e} before {t:e} {terms:?} c0 {c0:e}", first.unwrap());
                    }
                    None => assert!(first.is_none(), "case {case} rising {rising}: missed a crossing at {:e} {terms:?} c0 {c0:e}", first.unwrap()),
                }
            }
        }
        assert!(found > checked / 10, "{found} crossings of {checked}");
    }

    #[test]
    fn a_crossing_on_a_draining_cells_line_is_the_first_root() {
        // A 1 ms RC charging from 0 toward 0.6 against a threshold at 0.5,
        // with lines that: do nothing (one crossing up at ln 6 ms); pull it
        // down past its peak (up near 2 ms, back down near 4.5 ms); pull it
        // down too steeply to reach it (none); and push it up sooner.
        let f = -1e3;
        for (cl, cj) in [(0.0, 0.0), (-20.0, 5.0), (-400.0, 0.0), (30.0, -2e3)] {
            let g = |t: f64| 0.1 - 0.6 * (f * t).exp() + cl * t + cj * ramp_response(f, 0.0, t);
            for rising in [true, false] {
                let got = line_crossing(0.1, -0.6, cl, cj, f, rising, 1e-2);
                let mut prev = g(0.0);
                let mut scan = None;
                for i in 1..1_000_000u32 {
                    let t = f64::from(i) * 1e-8;
                    let v = g(t);
                    if (rising && prev < 0.0 && v >= 0.0) || (!rising && prev > 0.0 && v <= 0.0) {
                        scan = Some(t);
                        break;
                    }
                    prev = v;
                }
                match (got, scan) {
                    (Some(a), Some(b)) => assert!((a - b).abs() <= 1e-8, "cl {cl} cj {cj} rising {rising}: {a:e} against {b:e}"),
                    (a, b) => assert_eq!(a.is_some(), b.is_some(), "cl {cl} cj {cj} rising {rising}: {a:?} against {b:?}"),
                }
            }
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for BandRule {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            BandRule::Flat(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            BandRule::Curvature { fallback } => crate::snapshot::HeapBytes::heap_bytes(fallback, w),
        }
    }
}

impl crate::snapshot::HeapBytes for HubKind {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            HubKind::Battery(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            HubKind::Psu { v, i_limit } => crate::snapshot::HeapBytes::heap_bytes(v, w) + crate::snapshot::HeapBytes::heap_bytes(i_limit, w),
        }
    }
}

impl crate::snapshot::HeapBytes for RailCap {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Contrib {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.active, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_p, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.b, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for CfTerm {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.b, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vc0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.inf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.f, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for CellDrive {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kappa, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rho, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for CapSeg {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_inf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.k, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slope, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.alpha, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.beta, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cf_frozen, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cf_count, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.modes, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Coupled {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.f, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.gamma, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.b, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.r0, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for RailModes {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.modes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.q0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.esr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.passages_memo, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Basis {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Basis::Exp(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            Basis::Phi(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            Basis::J(f0, f1) => crate::snapshot::HeapBytes::heap_bytes(f0, w) + crate::snapshot::HeapBytes::heap_bytes(f1, w),
        }
    }
}

impl crate::snapshot::HeapBytes for RailState {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_scan, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.reg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.battery, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t_last, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.held_i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.held_cf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.version, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.event_t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cap_seg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cell, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cell_path, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cell_until, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a_run, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g_run, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.updates, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.settle_changed, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.gate_flips, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.gates_before, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.gates_after, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.quiet_steps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.held_draw, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for RailStats {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.moves, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sensitivities, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.scans, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.events, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.flips, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for CapDemand {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.draw, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.held, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DriftBound {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c_sat, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c2, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.esr_eff, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.inv_c_rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.alpha, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Motion {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Motion::Exp { coef, vc0, inf, f } => crate::snapshot::HeapBytes::heap_bytes(coef, w) + crate::snapshot::HeapBytes::heap_bytes(vc0, w) + crate::snapshot::HeapBytes::heap_bytes(inf, w) + crate::snapshot::HeapBytes::heap_bytes(f, w),
            Motion::Line { rate } => crate::snapshot::HeapBytes::heap_bytes(rate, w),
            Motion::Seg(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            Motion::Drift(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            Motion::Path(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for Watch {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.thresholds, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cap, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.terms, w); }
        n
    }
}
