//! Stress integrators, PTC integrators and threshold times (G1): core/README.md, W1g kernel semantics, points 5 and 10.
//!
//! Classic's dwell integrator (SE:5510-5531) is `S' = rate` while `rate > 0`
//! and `S' = -recovery` otherwise, floored at 0. The core holds it lazily as
//! `(S at t0, t0, drive)` and finds the threshold time analytically, so a latch
//! is an event at the exact time rather than a step end. Everything here is a
//! pure function of times and drives; the per-kind rate, recovery and
//! threshold come from the caller (G2a).

// A NaN time or stress reads as no time and no stress, so the guards below are
// written `!(x > 0.0)` on purpose; `x <= 0.0` would let a NaN through.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use super::ExpVoltage;

/// Classic's recovery timer length (PA:809): a tripped PTC resets after this
/// much continuous cool time.
pub const PTC_RECOVERY_TIME_S: f64 = 2.0;

/// The safeguarded Newton stops when a step moves the root by less than this
/// fraction of itself (README point 5: 1e-12 relative).
const NEWTON_REL_TOL: f64 = 1e-12;

/// A bisection fallback halves the bracket every other iteration, so this bound
/// is far above what 53 bits of mantissa need.
const NEWTON_MAX_ITER: usize = 200;

/// A positive-rate piece that never ends (`|a|` exactly at the rate's zero
/// level) is cut after this many time constants: the remaining gain is below
/// `exp(-100)` of the piece's.
const UNBOUNDED_TAIL_TAUS: f64 = 100.0;

/// A closed-form drive: over a segment the device's voltage is `V(s) = a +
/// b*exp(-s/tau)` (point 5), and the stress rate is `k*V(s)^2 - c`.
///
/// - resistor: `k = rPower/(R^2*pMax)`, `c = 1`;
/// - fuse: `k = 1/(rNormal*iRating)^2`, `c = 1`;
/// - PTC: `k = 1/R^2`, `c = iHold^2`.
///
/// While the rate is not positive the integrator bleeds at `recovery`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SegmentRate {
    pub k: f64,
    pub c: f64,
    pub recovery: f64,
    pub v: ExpVoltage,
}

/// What a stepped step holds over itself, in order ([`StressDrive::over_step`]):
/// `first` over the first `split` of the step (a fraction of it), `then`
/// over the rest.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepMean {
    pub first: StressDrive,
    pub split: f64,
    pub then: StressDrive,
}

impl StepMean {
    /// One drive over the whole step.
    pub fn whole(drive: StressDrive) -> StepMean {
        StepMean { first: drive, split: 1.0, then: drive }
    }

    /// The instant `first` gives way to `then` in a step from `t0` to `t1`.
    pub fn split_at(&self, t0: f64, t1: f64) -> f64 {
        if self.split >= 1.0 { t1 } else { t0 + self.split * (t1 - t0) }
    }
}

/// What drives a stress integrator from `t0` on.
#[derive(Clone, Copy, Debug)]
pub enum StressDrive {
    /// Static, fixed-pair and one stepped step (the step's end rate held over
    /// the step, SE:5510-5531 backward rule): `S' = rate` when `rate > 0`,
    /// else `S' = -recovery`.
    /// `zero` is the rate's zero level, `rate = q - zero` for the ratio `q`
    /// the kind integrates (1 for a resistor, fuse or LED; the PTC's hold
    /// current squared), which a stepped step's mean needs ([`Self::over_step`]).
    ///
    /// `q` is the ratio itself where the drive knows it, -1 where not (a
    /// sentinel that keeps drives comparable, which change detection needs): an
    /// end at rate 0 does not say how far under `zero` it sits, which a step
    /// whose ratio crosses the level needs ([`Self::over_step`]).
    Constant { rate: f64, recovery: f64, zero: f64, q: f64 },
    /// A closed-form segment.
    Segment(SegmentRate),
}

/// Two drives are the same drive when they integrate the same: `q` only
/// informs a stepped step's mean ([`StressDrive::over_step`]), so a quiet
/// device whose current moves under its rating is not rewritten at every
/// visit for it (a visit writes a drive that changed).
impl PartialEq for StressDrive {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (StressDrive::Constant { rate: a, recovery: b, zero: c, .. }, StressDrive::Constant { rate: x, recovery: y, zero: z, .. }) => a == x && b == y && c == z,
            (StressDrive::Segment(a), StressDrive::Segment(b)) => a == b,
            _ => false,
        }
    }
}

impl StressDrive {
    /// Nothing accumulates: the state of a device that sits at rate 0.
    pub const IDLE: StressDrive = StressDrive::Constant { rate: 0.0, recovery: 0.0, zero: 1.0, q: -1.0 };

    /// Whether `S` can grow under this drive. A reset skips an entry that sits
    /// at 0 under a drive that cannot, so a reset does not touch (and journal)
    /// every quiet device.
    pub fn accumulates(&self) -> bool {
        match self {
            StressDrive::Constant { rate, .. } => *rate > 0.0,
            StressDrive::Segment(seg) => match seg.collapsed() {
                Some(constant) => constant.accumulates(),
                None => seg.k > 0.0,
            },
        }
    }

    /// Whether `S` can grow at any time from `t` on: a closed-form segment
    /// only where its voltage, monotone over the segment, still reaches the
    /// rate's zero level after `t` (ruling R38 (a): a segment that stays under
    /// it bleeds exactly as an idle drive does, so a visit need not store it).
    pub fn accumulates_after(&self, t: f64) -> bool {
        match self {
            StressDrive::Constant { rate, .. } => *rate > 0.0,
            StressDrive::Segment(seg) => match seg.collapsed() {
                Some(constant) => constant.accumulates_after(t),
                None => seg.k > 0.0 && seg.pieces(t - seg.v.t_seg, f64::INFINITY).iter().any(Option::is_some),
            },
        }
    }

    /// `S(t)` from `s0` at `t0`, committing nothing (the readout of point 16
    /// and what `Stress::close` stores).
    pub fn value_after(&self, s0: f64, t0: f64, t: f64) -> f64 {
        if !(t > t0) {
            return s0;
        }
        match self {
            StressDrive::Constant { rate, recovery, .. } => constant_value(s0, *rate, *recovery, t - t0),
            StressDrive::Segment(seg) => match seg.collapsed() {
                Some(constant) => constant.value_after(s0, t0, t),
                None => seg.value_after(s0, t0, t),
            },
        }
    }

    /// The time `S` reaches `threshold`, or `None` if it never does under this
    /// drive. `S0 >= threshold` is due at `t0`: rounding at a tie can leave a
    /// closed value at or past the threshold, and a latch is never skipped.
    pub fn time_to_reach(&self, s0: f64, t0: f64, threshold: f64) -> Option<f64> {
        if s0 >= threshold {
            return Some(t0);
        }
        match self {
            StressDrive::Constant { rate, .. } => constant_latch_time(s0, t0, *rate, threshold),
            StressDrive::Segment(seg) => match seg.collapsed() {
                Some(constant) => constant.time_to_reach(s0, t0, threshold),
                None => seg.latch_time(s0, t0, threshold),
            },
        }
    }

    /// What a stepped step holds over itself (rulings R80 (a), R84 (B)),
    /// from the drives at the step's own start state (`start`) and end state
    /// (`end`), in place of classic's backward rule (the end rate held over
    /// the step, which under-counts a rate that falls through the step: a
    /// 555 astable's timing resistors take most of their stress in the
    /// discharge's exponential spikes, and burnt 5-17 % late).
    ///
    /// The rate is `q - zero` for a ratio `q` (`P/pMax`, `(I/limit)^2`, the
    /// PTC's `I^2`), and it is `q` that relaxes exponentially in time across
    /// an RC step, so where both ends accumulate the step holds the
    /// logarithmic mean of the two `q`s less `zero`. Where only one does, `q`
    /// crosses the level inside the step, at the fraction `f` of the step it
    /// spends above it: the step accrues `LM(q_hi, zero) - zero` over that
    /// stretch and bleeds at the idle end's recovery over the rest, each in
    /// its own order in time (a rising `q` bleeds first, floored at 0 as the
    /// integrator is, then accrues; netted into one slope, a step from S = 0
    /// with `q` from 0.5 to 3 gained 0.116 for 0.503). An end at `q = 0` is a
    /// voltage through zero, where no exponential goes: `q` is read as the
    /// square of a voltage linear over the step. An end whose `q` the drive
    /// does not carry (-1) is read at the level. The mean of the rates
    /// themselves read 0.26 for 0.82 on `q` from 1.001 to 3, and the
    /// trapezoid 2.72 for a CV tail of 0.86 by hand on cold 535 tpl 2. Exact
    /// for `q = A e^(kt)`; for `q = (a + b e^(-t/tau))^2` with `a b >= 0` (a
    /// voltage relaxing to a nonzero level) `ln q` is convex with curvature
    /// at most 1/2 per tau^2, so the mean over-counts by at most `exp((h/tau)^2
    /// / 16) - 1` (1.6 % at h = tau/2; the trapezoid's error on a pure
    /// exponential is `(h/tau)^2 / 3`); a crossing step's split over-counts
    /// more, +20-25 % at h = 2 tau, which no step bound limits yet. Where
    /// neither end accumulates the step bleeds at the mean recovery. A closed
    /// form on either side has nothing to average and keeps `end`.
    pub fn over_step(start: StressDrive, end: StressDrive) -> StepMean {
        let (StressDrive::Constant { rate: r0, recovery: k0, q: q0, .. }, StressDrive::Constant { rate: r1, recovery: k1, zero, .. }) = (start, end) else {
            return StepMean::whole(end);
        };
        let q1 = match end { StressDrive::Constant { q, .. } => q, _ => -1.0 };
        let accrue = |slope: f64| StressDrive::Constant { rate: slope.max(0.0), recovery: k1, zero, q: -1.0 };
        let bleed = |recovery: f64| StressDrive::Constant { rate: 0.0, recovery: recovery.max(0.0), zero, q: -1.0 };
        match (r0 > 0.0, r1 > 0.0) {
            (true, true) => StepMean::whole(accrue(log_mean(r0 + zero, r1 + zero) - zero)),
            (false, false) => StepMean::whole(bleed(0.5 * (k0.max(0.0) + k1.max(0.0)))),
            (falling, _) => {
                let (hi, lo, recovery) = if falling { (r0 + zero, q1, k1) } else { (r1 + zero, q0, k0) };
                let (f, above) = if lo == 0.0 {
                    // q = hi (1 - s)^2 over the stretch from the accruing end:
                    // above the level for 1 - sqrt(zero/hi) of the step.
                    let f = 1.0 - (zero / hi).sqrt();
                    let mean = if f > 0.0 { hi * (1.0 - (1.0 - f).powi(3)) / (3.0 * f) - zero } else { 0.0 };
                    (f, mean)
                } else if lo > 0.0 && lo < zero {
                    ((hi / zero).ln() / (hi / lo).ln(), log_mean(hi, zero) - zero)
                } else {
                    (1.0, log_mean(hi, zero) - zero)
                };
                if f >= 1.0 {
                    StepMean::whole(accrue(above))
                } else if falling {
                    StepMean { first: accrue(above), split: f, then: bleed(recovery) }
                } else {
                    StepMean { first: bleed(recovery), split: 1.0 - f, then: accrue(above) }
                }
            }
        }
    }

    /// The time a bleeding integrator reaches 0: `None` unless it is bleeding
    /// from a positive value at a positive recovery (a closed-form segment
    /// returns `None` too: its entry is dropped lazily at the next close).
    pub fn zero_time(&self, s0: f64, t0: f64) -> Option<f64> {
        match self {
            StressDrive::Constant { rate, recovery, .. } => bleed_zero_time(s0, t0, *rate, *recovery),
            StressDrive::Segment(seg) => seg.collapsed().and_then(|constant| constant.zero_time(s0, t0)),
        }
    }
}

/// Ruling R90 (a): the most a stepped stressed device's ratio may change in
/// one step, `|ln(q1/q0)| <= 0.5`. Under it the step mean over-counts `q =
/// (a + b e^(-t/tau))^2` by at most 1.5 % and a crossing step's split as
/// little (+19-33 % at h = 2 tau unbounded), and a CV tail falling 16-fold
/// in a step (cold 535 tpl 2 at 50 ms host steps, which latched) is taken
/// in a dozen.
pub const STRESS_STEP_RATIO: f64 = 0.5;

impl StressDrive {
    /// Ruling R90 (a): the step this drive's device lets its partition take
    /// next, from its ratio at the start (`start`) and end of the step just
    /// taken, over `h`: the ratio's log rate held, `STRESS_STEP_RATIO` of it;
    /// unbounded where [`Self::ratio_change`] is 0.
    pub fn stress_reach(start: StressDrive, end: StressDrive, h: f64) -> f64 {
        let change = Self::ratio_change(start, end);
        if change > 0.0 && h > 0.0 { STRESS_STEP_RATIO * h / change } else { f64::INFINITY }
    }

    /// Whether this drive's device is near or above its level (ruling R90
    /// (a)): its ratio known and at least half the zero level.
    pub fn stressed(&self) -> bool {
        matches!(*self, StressDrive::Constant { q, zero, .. } if q >= 0.5 * zero)
    }

    /// `|ln(q1/q0)|` across a step from `start` to `end`, 0 unless both ends
    /// know their ratio and either sits within a factor 2 of the zero level
    /// or above it (ruling R90 (a): a device far under its rating does not
    /// limit its partition's steps).
    pub fn ratio_change(start: StressDrive, end: StressDrive) -> f64 {
        let (StressDrive::Constant { q: q0, .. }, StressDrive::Constant { q: q1, zero, .. }) = (start, end) else {
            return 0.0;
        };
        if !(q0 > 0.0 && q1 > 0.0) || q0.max(q1) < 0.5 * zero {
            return 0.0;
        }
        (q1 / q0).ln().abs()
    }
}

/// The logarithmic mean `(b - a) / ln(b / a)` of two positive values, the
/// mean of `A e^(kt)` over a span it runs from `a` to `b`; symmetric. With
/// `a <= b`, `b - a` is exact (Sterbenz) wherever the two are near, and
/// `ln(b / a)` is `ln_1p((b - a) / a)` to a few ulps, so the quotient keeps
/// its digits down to `(b - a) / a` of 1e-8, under which its 0/0 is replaced
/// by the series `a (1 + d/2)`, `d = (b - a) / a`, whose next term `d^2/12`
/// is under 1e-17.
pub fn log_mean(a: f64, b: f64) -> f64 {
    let (a, b) = if a <= b { (a, b) } else { (b, a) };
    let d = (b - a) / a;
    if d < 1e-8 {
        a * (1.0 + 0.5 * d)
    } else {
        (b - a) / d.ln_1p()
    }
}

/// `S` after `dt` under a constant drive. The recovery is clamped at 0 as
/// classic does (`-Math.max(0, recoveryRate)`, SE:5522).
pub fn constant_value(s0: f64, rate: f64, recovery: f64, dt: f64) -> f64 {
    if !(dt > 0.0) {
        return s0;
    }
    if rate > 0.0 {
        s0 + rate * dt
    } else {
        (s0 - recovery.max(0.0) * dt).max(0.0)
    }
}

/// `t_x = t0 + (threshold - S0)/rate` for a constant positive rate.
pub fn constant_latch_time(s0: f64, t0: f64, rate: f64, threshold: f64) -> Option<f64> {
    if s0 >= threshold {
        return Some(t0);
    }
    if rate > 0.0 {
        Some(t0 + (threshold - s0) / rate)
    } else {
        None
    }
}

/// The time a bleed floored at 0 gets there.
pub fn bleed_zero_time(s0: f64, t0: f64, rate: f64, recovery: f64) -> Option<f64> {
    if rate > 0.0 || !(s0 > 0.0) || !(recovery > 0.0) {
        return None;
    }
    Some(t0 + s0 / recovery)
}

/// The set `{s >= s1 : sign*V(s) > level}` as `(start, end)` in segment
/// time, `end` possibly infinite, for `V = a + b*exp(-s/tau)`. `V` is
/// monotone in `s`, so the set is a prefix, a suffix, everything or nothing,
/// decided by `V(s1)` and the limit `a`, and a crossing is solved once,
/// analytically: `e^(-s/tau) = (level - a)/b`, which for a ramp
/// (`ExpVoltage`'s doc) is `1 + (level - v0)/b`, taken with `ln_1p` so that
/// a huge `b` keeps its digits.
fn above(v: &ExpVoltage, sign: f64, level: f64, s1: f64) -> Option<(f64, f64)> {
    let (a, b, tau) = (sign * v.a, sign * v.b, v.tau);
    let v1 = sign * v.at_offset(s1);
    // `b` is nonzero whenever a crossing is needed: with `b == 0` the voltage is
    // the constant `a`, which one of the two branches below already decides.
    let crossing = || {
        if v.ramp {
            (-tau * ((level - sign * v.v0) / b).ln_1p()).max(s1)
        } else {
            (-tau * ((level - a) / b).ln()).max(s1)
        }
    };
    if v1 > level {
        if a >= level {
            Some((s1, f64::INFINITY))
        } else {
            Some((s1, crossing()))
        }
    } else if a > level {
        Some((crossing(), f64::INFINITY))
    } else {
        None
    }
}

/// `g1(z) = z + expm1(-z)`, the integral of `-expm1(-x)` over `[0, z]`, by its
/// series below 1 (it is `z^2/2` to first order, which the direct form
/// would lose to cancellation).
pub(crate) fn g1(z: f64) -> f64 {
    if !(z < 1.0) {
        return z + (-z).exp_m1();
    }
    // sum over n >= 2 of (-z)^n / n!
    let (mut term, mut sum) = (z * z / 2.0, 0.0);
    for n in 3..40 {
        sum += term;
        term *= -z / n as f64;
        if term.abs() <= 1e-18 * sum.abs() {
            break;
        }
    }
    sum + term
}

/// `g2(z) = z + 2 expm1(-z) - expm1(-2z)/2`, the integral of `expm1(-x)^2`
/// over `[0, z]`, by its series below 1 (`z^3/3` to first order).
pub(crate) fn g2(z: f64) -> f64 {
    if !(z < 1.0) {
        return z + 2.0 * (-z).exp_m1() - 0.5 * (-2.0 * z).exp_m1();
    }
    // sum over n >= 2 of (-1)^n (2^n - 2) z^(n+1) / ((n+1) n!)
    let (mut sum, mut pow2, mut zn, mut fact) = (0.0f64, 4.0f64, z * z * z, 2.0f64);
    for n in 2..48 {
        let term = (pow2 - 2.0) * zn / ((n + 1) as f64 * fact) * if n % 2 == 0 { 1.0 } else { -1.0 };
        sum += term;
        if term.abs() <= 1e-18 * sum.abs() {
            break;
        }
        pow2 *= 2.0;
        zn *= z;
        fact *= (n + 1) as f64;
    }
    sum
}

impl SegmentRate {
    /// The constant drive this segment is when its voltage does not relax
    /// (`ExpVoltage::constant_value`): the closed forms below divide by `tau`,
    /// and a latch time of `Some(NaN)` for `tau = 0, b = 0` would be an event
    /// the kernel schedules.
    fn collapsed(&self) -> Option<StressDrive> {
        let volts = self.v.constant_value()?;
        Some(StressDrive::Constant { rate: self.k * volts * volts - self.c, recovery: self.recovery, zero: self.c, q: self.k * volts * volts })
    }

    fn rate_s(&self, s: f64) -> f64 {
        let v = self.v.at_offset(s);
        self.k * v * v - self.c
    }

    /// The rate's value as `s` goes to infinity.
    fn rate_limit(&self) -> f64 {
        self.k * self.v.a * self.v.a - self.c
    }

    /// `{rate > 0}` within `[s1, s2]` (segment time) as at most two disjoint
    /// pieces in increasing order. `rate > 0` is `|V| > L` with `L =
    /// sqrt(c/k)`: the voltage is monotone, so `V > L` and `V < -L` are each
    /// one ray, separated by the zero crossing, and `P` is monotone on each.
    fn pieces(&self, s1: f64, s2: f64) -> [Option<(f64, f64)>; 2] {
        if !(self.k > 0.0 && self.c >= 0.0) {
            return [None, None];
        }
        let level = (self.c / self.k).sqrt();
        let clip = |piece: Option<(f64, f64)>| {
            piece.and_then(|(p, q)| {
                let q = q.min(s2);
                (p < q).then_some((p, q))
            })
        };
        let up = clip(above(&self.v, 1.0, level, s1));
        let down = clip(above(&self.v, -1.0, level, s1));
        match (up, down) {
            (Some(u), Some(d)) => {
                if u.0 <= d.0 {
                    [Some(u), Some(d)]
                } else {
                    [Some(d), Some(u)]
                }
            }
            (Some(x), None) | (None, Some(x)) => [Some(x), None],
            (None, None) => [None, None],
        }
    }

    /// The integral of the rate over `[p, p + d]`, inside one positive piece:
    /// `(k*a^2 - c)*d - 2*k*a*b*tau*E*expm1(-d/tau) - (k*b^2*tau/2)*E^2*
    /// expm1(-2d/tau)` with `E = exp(-p/tau)`. The exponential terms are
    /// differences of exponentials, written with `expm1` so a short interval
    /// loses nothing to cancellation.
    ///
    /// A ramp (`ExpVoltage`'s doc) is integrated from its value at the piece's
    /// start, `V(p + x) = V_p + W·expm1(-x/tau)` with `W = b e^(-p/tau)`:
    /// `(k V_p^2 - c) d - 2 k V_p W tau g1(d/tau) + k W^2 tau g2(d/tau)`, the
    /// `g`s the integrals of `expm1` and its square ([`g1`], [`g2`]), whose
    /// terms are the size of the rate's own over the piece however large `b`.
    fn gain(&self, p: f64, d: f64) -> f64 {
        let ExpVoltage { a, b, tau, .. } = self.v;
        if self.v.ramp {
            let (v_p, w, z) = (self.v.at_offset(p), b * (-p / tau).exp(), d / tau);
            return (self.k * v_p * v_p - self.c) * d - 2.0 * self.k * v_p * w * tau * g1(z) + self.k * w * w * tau * g2(z);
        }
        let e = (-p / tau).exp();
        (self.k * a * a - self.c) * d
            - 2.0 * self.k * a * b * tau * e * (-d / tau).exp_m1()
            - 0.5 * self.k * b * b * tau * e * e * (-2.0 * d / tau).exp_m1()
    }

    /// `S(t)`: bleed through the gaps, gain through the positive pieces.
    fn value_after(&self, s0: f64, t0: f64, t: f64) -> f64 {
        let s1 = t0 - self.v.t_seg;
        let s2 = t - self.v.t_seg;
        let recovery = self.recovery.max(0.0);
        let mut s = s0;
        let mut pos = s1;
        for (p, q) in self.pieces(s1, s2).into_iter().flatten() {
            s = (s - recovery * (p - pos)).max(0.0);
            s += self.gain(p, q - p);
            pos = q;
        }
        (s - recovery * (s2 - pos)).max(0.0)
    }

    /// The first time `S` reaches `threshold`, by walking the positive pieces
    /// in order and solving inside the first whose total gain gets there.
    fn latch_time(&self, s0: f64, t0: f64, threshold: f64) -> Option<f64> {
        let s1 = t0 - self.v.t_seg;
        let recovery = self.recovery.max(0.0);
        let mut s = s0;
        let mut pos = s1;
        for (p, q) in self.pieces(s1, f64::INFINITY).into_iter().flatten() {
            s = (s - recovery * (p - pos)).max(0.0);
            let r_end = if q.is_finite() { self.rate_s(q) } else { self.rate_limit() };
            let q = if q.is_finite() || r_end > 0.0 { q } else { p + UNBOUNDED_TAIL_TAUS * self.v.tau };
            let total = if q.is_finite() { self.gain(p, q - p) } else { f64::INFINITY };
            if s + total >= threshold {
                return Some(self.v.t_seg + p + self.solve_piece(p, q, r_end, s, threshold));
            }
            s += total;
            pos = q;
        }
        None
    }

    /// The offset `d` into the piece `[p, q]` at which `s_start + gain(p, d)`
    /// reaches `threshold`: safeguarded Newton on a function that is monotone
    /// increasing (its derivative is the rate, positive on the piece) and
    /// convex or concave (the rate is monotone), so it is bracketed from both
    /// sides by the rate's own extremes and Newton steps that leave the
    /// bracket fall back to bisection.
    fn solve_piece(&self, p: f64, q: f64, r_end: f64, s_start: f64, threshold: f64) -> f64 {
        let need = threshold - s_start;
        let r_start = self.rate_s(p);
        let (r_lo, r_hi) = if r_start < r_end { (r_start, r_end) } else { (r_end, r_start) };
        let d_max = q - p;
        // gain(d) <= r_hi*d and gain(d) >= r_lo*d on the piece.
        let mut lo = (need / r_hi).min(d_max);
        let mut hi = if r_lo > 0.0 { (need / r_lo).min(d_max) } else { d_max };
        let f = |d: f64| s_start + self.gain(p, d) - threshold;
        if !hi.is_finite() {
            // An unbounded piece whose rate starts at 0: double until it gets there.
            hi = lo.max(f64::MIN_POSITIVE) * 2.0;
            while f(hi) < 0.0 {
                lo = hi;
                hi *= 2.0;
            }
        }
        let mut x = if r_start > 0.0 { (need / r_start).clamp(lo, hi) } else { 0.5 * (lo + hi) };
        for _ in 0..NEWTON_MAX_ITER {
            let fx = f(x);
            if fx == 0.0 {
                return x;
            }
            if fx > 0.0 {
                hi = hi.min(x);
            } else {
                lo = lo.max(x);
            }
            let mut next = x - fx / self.rate_s(p + x);
            if !(next > lo && next < hi) {
                next = lo + 0.5 * (hi - lo);
            }
            if (next - x).abs() <= NEWTON_REL_TOL * next.abs() || hi - lo <= NEWTON_REL_TOL * hi.abs() {
                return next;
            }
            x = next;
        }
        x
    }
}

/// A device's lazy stress integrator: `S` at `t0`, what drives it, and the
/// latch threshold.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stress {
    pub s0: f64,
    pub t0: f64,
    pub drive: StressDrive,
    pub threshold: f64,
}

impl Stress {
    /// Closes the integrator over a stepped step from `t0` to `t1` under
    /// `mean`'s pieces in order, and returns the latch time if one falls
    /// inside it (the integrator is then left at the piece it latches in,
    /// closed to that piece's start).
    pub fn run_step(&mut self, mean: &StepMean, t0: f64, t1: f64) -> Option<f64> {
        self.close(t0);
        let tm = mean.split_at(t0, t1);
        self.drive = mean.first;
        if let Some(t) = self.latch_time().filter(|&t| t <= tm) {
            return Some(t);
        }
        self.close(tm);
        self.drive = mean.then;
        if let Some(t) = self.latch_time().filter(|&t| t <= t1) {
            return Some(t);
        }
        self.close(t1);
        None
    }

    pub fn new(threshold: f64, t0: f64) -> Self {
        Stress { s0: 0.0, t0, drive: StressDrive::IDLE, threshold }
    }

    /// `S(t)` under the current drive, committing nothing.
    pub fn value_at(&self, t: f64) -> f64 {
        self.drive.value_after(self.s0, self.t0, t)
    }

    /// Brings the integrator exactly to `t` with what drove it. The drive stays
    /// in place for the caller to replace.
    pub fn close(&mut self, t: f64) {
        if t > self.t0 {
            self.s0 = self.value_at(t);
            self.t0 = t;
        }
    }

    /// The next latch time under the current drive.
    pub fn latch_time(&self) -> Option<f64> {
        self.drive.time_to_reach(self.s0, self.t0, self.threshold)
    }

    /// `resetFailures` (point 13): every integrator starts again from 0 at the
    /// reset instant. The drive is kept: the voltages have not moved, and a
    /// publish that follows replaces it.
    pub fn reset(&mut self, now: f64) {
        self.s0 = 0.0;
        self.t0 = now;
    }
}

/// A PTC's record values (Fuse and PTC records), floored by the reader.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PtcParams {
    pub r_normal: f64,
    pub i_hold: f64,
}

impl PtcParams {
    /// The trip threshold, in classic's operation order (PA:789).
    pub fn trip_threshold(&self) -> f64 {
        3.0 * self.i_hold * self.i_hold * 0.1
    }

    /// The bleed rate below the hold current (PA:798).
    pub fn bleed_rate(&self) -> f64 {
        self.i_hold * self.i_hold * 0.5
    }

    /// `rNormal`, or `rNormal * 200` while tripped, the product taken first as
    /// classic stamps it (PA:759).
    pub fn resistance(&self, tripped: bool) -> f64 {
        if tripped {
            self.r_normal * 200.0
        } else {
            self.r_normal
        }
    }

    /// Classic's cool test on a current, in its operation order (PA:805-807).
    pub fn is_cool(&self, current: f64) -> bool {
        let tripped_power = current * current * self.r_normal * 200.0;
        let cool_power = self.i_hold * self.i_hold * self.r_normal * 0.1;
        current < self.i_hold / 2.0 && tripped_power < cool_power
    }

    /// The largest current the cool test passes, for a closed-form segment
    /// where the test becomes a voltage level (`rNormal` cancels from the
    /// power clause, which is the stricter one: about `0.02236 * iHold`).
    pub fn cool_current_limit(&self) -> f64 {
        (self.i_hold / 2.0).min((self.i_hold * self.i_hold * 0.1 / 200.0).sqrt())
    }

    /// An untripped PTC at a constant current: `S' = max(0, I^2 - iHold^2)` at or
    /// above the hold current (PA:787-788), else a bleed at `iHold^2 * 0.5`
    /// (PA:798). At exactly the hold current the rate is 0 and nothing bleeds.
    pub fn untripped_constant(&self, current: f64) -> StressDrive {
        if current >= self.i_hold {
            StressDrive::Constant { rate: (current * current - self.i_hold * self.i_hold).max(0.0), recovery: 0.0, zero: self.i_hold * self.i_hold, q: current * current }
        } else {
            StressDrive::Constant { rate: 0.0, recovery: self.bleed_rate(), zero: self.i_hold * self.i_hold, q: current * current }
        }
    }

    /// An untripped PTC in a closed-form segment, `I = |V|/rNormal`. A voltage
    /// that does not relax collapses to a constant rate when the drive is
    /// evaluated (`StressDrive`'s methods).
    pub fn untripped_segment(&self, v: ExpVoltage) -> StressDrive {
        StressDrive::Segment(SegmentRate {
            k: 1.0 / (self.r_normal * self.r_normal),
            c: self.i_hold * self.i_hold,
            recovery: self.bleed_rate(),
            v,
        })
    }

    /// A tripped PTC at a constant current from `now`: cool now and until the
    /// next publish, or not.
    pub fn tripped_constant(&self, current: f64, now: f64) -> PtcDrive {
        if self.is_cool(current) {
            PtcDrive::Cool { from: now, until: f64::INFINITY }
        } else {
            PtcDrive::HOT
        }
    }

    /// A tripped PTC in a closed-form segment: `|V|` stays under the cool level
    /// on one interval, the intersection of `V < level` and `V > -level`, each
    /// a ray. Returned in absolute time and clipped to start at `now`.
    pub fn tripped_segment(&self, v: ExpVoltage, now: f64) -> PtcDrive {
        if let Some(volts) = v.constant_value() {
            return self.tripped_constant(volts.abs() / self.resistance(true), now);
        }
        let level = self.cool_current_limit() * self.resistance(true);
        let s1 = now - v.t_seg;
        let below = above(&v, -1.0, -level, s1);
        let over = above(&v, 1.0, -level, s1);
        match (below, over) {
            (Some((p1, q1)), Some((p2, q2))) => {
                let (p, q) = (p1.max(p2), q1.min(q2));
                // A window that already covers `now` starts at `now` exactly:
                // `t_seg + (now - t_seg)` can land one ulp after it, and
                // `set_drive` would then read an uncool observation and zero
                // `Rt`.
                let from = if p <= s1 { now } else { v.t_seg + p };
                let until = v.t_seg + q;
                if from < until {
                    PtcDrive::Cool { from, until }
                } else {
                    PtcDrive::HOT
                }
            }
            _ => PtcDrive::HOT,
        }
    }
}

/// What drives a PTC integrator from `t0` on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PtcDrive {
    /// Untripped: the stress integrator's drive (trip at `trip_threshold`).
    Stress(StressDrive),
    /// Tripped: the interval of absolute time over which the cool test holds.
    /// The recovery timer counts only inside it and is reset by an uncool
    /// observation. For a constant drive it runs from the publish to infinity.
    Cool { from: f64, until: f64 },
}

impl PtcDrive {
    /// Never cool.
    pub const HOT: PtcDrive = PtcDrive::Cool { from: f64::INFINITY, until: f64::INFINITY };

    /// Cool at the instant `t`.
    pub fn cool_at(&self, t: f64) -> bool {
        match self {
            PtcDrive::Cool { from, until } => *from <= t && t < *until,
            PtcDrive::Stress(_) => false,
        }
    }
}

/// What a PTC does when its event fires (a stamp change, never a failure or a
/// warning record, point 10).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PtcEvent {
    Trip,
    Recover,
}

/// `Rt` after closing `[t0, t]` inside the cool window `[from, until)`: the
/// observation at `t` decides. Not cool at `t` (the window not begun, or over)
/// resets it (PA:815, "restart the recovery timer"); cool at `t` counts the
/// time since the window began or since `t0`, whichever is later.
fn rt_after(rt0: f64, t0: f64, t: f64, from: f64, until: f64) -> f64 {
    if !(from <= t && t < until) {
        0.0
    } else if from <= t0 {
        rt0 + (t - t0)
    } else {
        t - from
    }
}

/// When `Rt` reaches 2.0 inside the window, if it does before the window ends.
fn recovery_time(rt0: f64, t0: f64, from: f64, until: f64) -> Option<f64> {
    let (enter, start) = if from <= t0 { (t0, rt0) } else { (from, 0.0) };
    if !(enter < until) {
        return None;
    }
    let t = enter + (PTC_RECOVERY_TIME_S - start).max(0.0);
    (t < until).then_some(t)
}

/// A PTC's lazy state, classic's `{tripped, tripStress, recoveryTime}`: `S`
/// while untripped, `Rt` while tripped, both held at `t0` with their drive.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PtcEntry {
    pub params: PtcParams,
    pub tripped: bool,
    pub stress: f64,
    pub rt: f64,
    pub t0: f64,
    pub drive: PtcDrive,
}

impl PtcEntry {
    /// A load starts every PTC untripped with nothing accumulated (point 15).
    pub fn new(params: PtcParams, t0: f64) -> Self {
        PtcEntry { params, tripped: false, stress: 0.0, rt: 0.0, t0, drive: PtcDrive::Stress(StressDrive::IDLE) }
    }

    /// `(tripped, S, Rt)` at `t`, committing nothing: the `ptc_fuse` device
    /// state of point 10.
    pub fn projected(&self, t: f64) -> (bool, f64, f64) {
        let mut at = *self;
        at.close(t);
        (at.tripped, at.stress, at.rt)
    }

    /// Brings the integrator exactly to `t` with what drove it.
    pub fn close(&mut self, t: f64) {
        if !(t > self.t0) {
            return;
        }
        if self.tripped {
            self.rt = match self.drive {
                PtcDrive::Cool { from, until } => rt_after(self.rt, self.t0, t, from, until),
                PtcDrive::Stress(_) => 0.0,
            };
        } else if let PtcDrive::Stress(drive) = self.drive {
            self.stress = drive.value_after(self.stress, self.t0, t);
        }
        self.t0 = t;
    }

    /// Replaces the drive after a publish, at the entry's own `t0`. A tripped
    /// PTC that is not cool at `t0` has its timer reset by the observation
    /// (point 10: "`Rt = 0`, set at the publish that observes it").
    pub fn set_drive(&mut self, drive: PtcDrive) {
        self.drive = drive;
        if self.tripped && !drive.cool_at(self.t0) {
            self.rt = 0.0;
        }
    }

    /// [`Stress::run_step`] for an untripped PTC's stress: closes it over a
    /// stepped step under `mean`'s pieces in order and returns the trip time
    /// if one falls inside it.
    pub fn run_step(&mut self, mean: &StepMean, t0: f64, t1: f64) -> Option<f64> {
        self.close(t0);
        let tm = mean.split_at(t0, t1);
        self.set_drive(PtcDrive::Stress(mean.first));
        if let Some((t, PtcEvent::Trip)) = self.next_event().filter(|&(t, _)| t <= tm) {
            return Some(t);
        }
        self.close(tm);
        self.set_drive(PtcDrive::Stress(mean.then));
        if let Some((t, PtcEvent::Trip)) = self.next_event().filter(|&(t, _)| t <= t1) {
            return Some(t);
        }
        self.close(t1);
        None
    }

    /// The next trip or recovery.
    pub fn next_event(&self) -> Option<(f64, PtcEvent)> {
        if self.tripped {
            let PtcDrive::Cool { from, until } = self.drive else { return None };
            recovery_time(self.rt, self.t0, from, until).map(|t| (t, PtcEvent::Recover))
        } else {
            let PtcDrive::Stress(drive) = self.drive else { return None };
            drive.time_to_reach(self.stress, self.t0, self.params.trip_threshold()).map(|t| (t, PtcEvent::Trip))
        }
    }

    /// The trip: `S` and `Rt` zeroed (PA:790-792). The drive is a placeholder
    /// until the re-solve at this instant publishes the tripped current, which
    /// may be cool at once: recovery then starts at the trip instant itself.
    pub fn trip(&mut self, t: f64) {
        self.tripped = true;
        self.stress = 0.0;
        self.rt = 0.0;
        self.t0 = t;
        self.drive = PtcDrive::HOT;
    }

    /// The recovery (PA:810-812): all three cleared.
    pub fn recover(&mut self, t: f64) {
        self.tripped = false;
        self.stress = 0.0;
        self.rt = 0.0;
        self.t0 = t;
        self.drive = PtcDrive::Stress(StressDrive::IDLE);
    }

    /// `resetFailures` (point 13): untripped with nothing accumulated. A tripped
    /// PTC goes back through `recover`, whose stamp change forces a publish
    /// that supplies its new drive; an untripped one keeps the drive its
    /// unmoved voltages still give.
    pub fn reset(&mut self, now: f64) {
        if self.tripped {
            self.recover(now);
        } else {
            self.stress = 0.0;
            self.rt = 0.0;
            self.t0 = now;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The resistor/fuse/LED recoveries and thresholds, as the README's
    /// constants table gives them (SE:120-122 and PA/SC): used only to pin
    /// expected values, the per-kind drives are G2a's.
    const RESISTOR_THRESHOLD: f64 = 1.0;
    const FUSE_THRESHOLD: f64 = 0.3;

    fn seg(k: f64, c: f64, recovery: f64, a: f64, b: f64, tau: f64, t_seg: f64) -> SegmentRate {
        SegmentRate { k, c, recovery, v: ExpVoltage::new(a, b, tau, t_seg) }
    }

    /// The same integrator by midpoint stepping: `S' = rate` where the rate at
    /// the step's midpoint is positive, else the floored bleed. Returns the
    /// first time `S` reaches `threshold` (interpolated inside the step) and
    /// `S` at `t_end`. Uses only the rate formula, none of the closed form.
    fn numeric(seg: &SegmentRate, s0: f64, t0: f64, threshold: f64, t_end: f64, h: f64) -> (Option<f64>, f64) {
        let rate = |t: f64| {
            let v = seg.v.a + seg.v.b * (-(t - seg.v.t_seg) / seg.v.tau).exp();
            seg.k * v * v - seg.c
        };
        let n = ((t_end - t0) / h).round() as u64;
        let mut s = s0;
        let mut hit = None;
        for i in 0..n {
            let t = t0 + i as f64 * h;
            let r = rate(t + 0.5 * h);
            let next = if r > 0.0 { s + r * h } else { (s - seg.recovery * h).max(0.0) };
            if next >= threshold {
                hit = Some(t + h * (threshold - s) / (next - s));
                break;
            }
            s = next;
        }
        (hit, s)
    }

    #[test]
    fn linear_latch_times_come_from_the_model() {
        // Resistor at 2x: P/pMax - 1 = 1, threshold 1: t = 1.0.
        let resistor = StressDrive::Constant { rate: 2.0 - 1.0, recovery: 1.0, zero: 1.0, q: -1.0 };
        assert!((resistor.time_to_reach(0.0, 0.0, RESISTOR_THRESHOLD).unwrap() - 1.0).abs() < 1e-15);
        // Fuse at 2x (1 A through 0.5 A): (I/iRating)^2 - 1 = 3, threshold 0.3: t = 0.1.
        let fuse = StressDrive::Constant { rate: (1.0f64 / 0.5).powi(2) - 1.0, recovery: 0.3, zero: 1.0, q: -1.0 };
        assert!((fuse.time_to_reach(0.0, 0.0, FUSE_THRESHOLD).unwrap() - 0.1).abs() < 1e-15);
        // PTC at 2x hold (1 A, iHold 0.5): I^2 - iHold^2 = 0.75 against 3*0.5*0.5*0.1 = 0.075.
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let drive = ptc.untripped_constant(1.0);
        assert_eq!(drive, StressDrive::Constant { rate: 0.75, recovery: 0.0, zero: 0.25, q: 1.0 });
        assert!((ptc.trip_threshold() - 0.075).abs() < 1e-15);
        assert!((drive.time_to_reach(0.0, 0.0, ptc.trip_threshold()).unwrap() - 0.1).abs() < 1e-14);
        // From a nonzero start and a later t0.
        assert!((resistor.time_to_reach(0.25, 3.0, RESISTOR_THRESHOLD).unwrap() - 3.75).abs() < 1e-15);
        // No rate, no event; already at the threshold is due now.
        assert_eq!(StressDrive::Constant { rate: 0.0, recovery: 1.0, zero: 1.0, q: -1.0 }.time_to_reach(0.5, 0.0, 1.0), None);
        assert_eq!(resistor.time_to_reach(1.0, 4.0, 1.0), Some(4.0));
    }

    #[test]
    fn recovery_bleeds_floor_at_zero_and_find_the_zero_time() {
        // Recoveries 1 (resistor), 0.3 (fuse) and 2 (LED) per second from 0.5.
        for (recovery, after_0_2, zero_at) in [(1.0, 0.3, 0.5), (0.3, 0.44, 0.5 / 0.3), (2.0, 0.1, 0.25)] {
            let drive = StressDrive::Constant { rate: 0.0, recovery, zero: 1.0, q: -1.0 };
            assert!((drive.value_after(0.5, 0.0, 0.2) - after_0_2).abs() < 1e-15, "recovery {recovery}");
            assert_eq!(drive.value_after(0.5, 0.0, zero_at + 1.0), 0.0, "floored at 0");
            assert!((drive.zero_time(0.5, 0.0).unwrap() - zero_at).abs() < 1e-15);
            assert!(!drive.accumulates());
        }
        // A negative recovery never grows the integrator (classic clamps it).
        assert_eq!(StressDrive::Constant { rate: 0.0, recovery: -1.0, zero: 1.0, q: -1.0 }.value_after(0.5, 0.0, 2.0), 0.5);
        // Accumulating drives have no zero time.
        assert_eq!(StressDrive::Constant { rate: 1.0, recovery: 1.0, zero: 1.0, q: -1.0 }.zero_time(0.5, 0.0), None);
        // The PTC bleeds at 0.125 per second at iHold 0.5.
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let bleed = ptc.untripped_constant(0.4);
        assert_eq!(bleed, StressDrive::Constant { rate: 0.0, recovery: 0.125, zero: 0.25, q: 0.4 * 0.4 });
        assert!((bleed.value_after(0.05, 0.0, 0.2) - 0.025).abs() < 1e-15);
    }

    #[test]
    fn close_commits_and_value_at_projects() {
        let mut s = Stress::new(1.0, 2.0);
        s.drive = StressDrive::Constant { rate: 0.5, recovery: 1.0, zero: 1.0, q: -1.0 };
        let before = s;
        assert_eq!(s.value_at(4.0), 1.0);
        assert_eq!(s, before, "a projection never writes");
        s.close(3.0);
        assert_eq!((s.s0, s.t0), (0.5, 3.0));
        s.close(3.0);
        s.close(1.0);
        assert_eq!((s.s0, s.t0), (0.5, 3.0), "closing at or before t0 is a no-op");
        assert_eq!(s.latch_time(), Some(4.0));
    }

    #[test]
    fn reset_zeroes_the_value_and_keeps_the_drive() {
        let mut s = Stress::new(1.0, 2.0);
        s.drive = StressDrive::Constant { rate: 0.5, recovery: 1.0, zero: 1.0, q: -1.0 };
        s.s0 = 0.4;
        s.reset(10.0);
        assert_eq!((s.s0, s.t0), (0.0, 10.0));
        assert_eq!(s.latch_time(), Some(12.0), "integrates from 0 at the reset instant");
    }

    #[test]
    fn closed_form_gain_matches_its_own_derivative() {
        // d(gain)/dd = rate: a central difference over the closed form.
        let s = seg(0.02, 1.0, 1.0, 5.0, 5.0, 1.0, 0.0);
        for (p, d) in [(0.0, 0.3), (0.2, 0.5), (1.0, 2.0)] {
            let h = 1e-6;
            let slope = (s.gain(p, d + h) - s.gain(p, d - h)) / (2.0 * h);
            assert!((slope - s.rate_s(p + d)).abs() < 1e-8, "p {p} d {d}");
        }
        assert_eq!(s.gain(0.4, 0.0), 0.0);
    }

    #[test]
    fn pieces_split_where_the_voltage_crosses_zero() {
        // V falls from 8 toward -4: above L = 3 first, inside the band while it
        // crosses zero, below -L at the end (k = 1/9, c = 1).
        let s = seg(1.0 / 9.0, 1.0, 1.0, -4.0, 12.0, 1.0, 0.0);
        let [first, second] = s.pieces(0.0, f64::INFINITY);
        let (p1, q1) = first.expect("above +L");
        let (p2, q2) = second.expect("below -L");
        assert_eq!(p1, 0.0);
        assert!((q1 - (12.0f64 / 7.0).ln()).abs() < 1e-12, "V = 3 at e^-s = 7/12");
        assert!((p2 - 12.0f64.ln()).abs() < 1e-12, "V = -3 at e^-s = 1/12");
        assert!(q2.is_infinite());
        assert!(q1 < p2);
        // Clipped to a window.
        let [only, none] = s.pieces(0.0, 0.3);
        assert_eq!(only, Some((0.0, 0.3)));
        assert_eq!(none, None);
        // A voltage that never reaches L has no piece, and none when k <= 0.
        assert_eq!(seg(0.01, 1.0, 1.0, 5.0, 0.0, 1.0, 0.0).pieces(0.0, 10.0), [None, None]);
        assert_eq!(seg(0.0, 1.0, 1.0, 50.0, 0.0, 1.0, 0.0).pieces(0.0, 10.0), [None, None]);
    }

    /// One closed-form case against the fine numeric integral: the latch time
    /// within 1e-9 s, and `S` at the latch time at the threshold within the
    /// Newton tolerance. `expect` says whether the case must latch before
    /// `t_end` at all, so a `None` on both sides cannot hide a case that was
    /// meant to exercise the root.
    fn assert_latch_matches_numeric(label: &str, s: &SegmentRate, s0: f64, t0: f64, threshold: f64, t_end: f64, expect: bool) {
        let (hit, _) = numeric(s, s0, t0, threshold, t_end, 2e-6);
        let closed = StressDrive::Segment(*s).time_to_reach(s0, t0, threshold);
        match (closed, hit) {
            (Some(c), Some(n)) => {
                assert!(expect, "{label}: latched at {c}, expected no latch");
                assert!((c - n).abs() < 1e-9, "{label}: closed {c}, numeric {n}");
                let at = StressDrive::Segment(*s).value_after(s0, t0, c);
                assert!((at - threshold).abs() <= 1e-13 * threshold, "{label}: S(t_x) = {at}, threshold {threshold}");
            }
            (None, None) => assert!(!expect, "{label}: expected a latch, got none"),
            (c, n) => panic!("{label}: closed {c:?}, numeric {n:?}"),
        }
    }

    #[test]
    fn closed_form_root_matches_a_numeric_integral() {
        // A resistor of 50 ohm, pMax 1 W: k = rPower/(R^2 pMax) = 0.02, c = 1.
        // V relaxes from 10 V (rate 1) to 5 V (rate -0.5): a decaying overload
        // whose one piece gains 0.352 in all, so 0.34 latches near its very end.
        let decaying = seg(0.02, 1.0, 1.0, 5.0, 5.0, 1.0, 0.0);
        assert_latch_matches_numeric("decaying, early", &decaying, 0.0, 0.0, 0.2, 4.0, true);
        assert_latch_matches_numeric("decaying, at the end of the piece", &decaying, 0.0, 0.0, 0.34, 4.0, true);
        assert_latch_matches_numeric("decaying, out of reach", &decaying, 0.0, 0.0, 5.0, 4.0, false);
        // Rising into an overload (2 V -> 8 V): the piece is a suffix to infinity.
        let rising = seg(0.02, 1.0, 1.0, 8.0, -6.0, 0.5, 0.0);
        assert_latch_matches_numeric("rising", &rising, 0.0, 0.0, 0.3, 6.0, true);
        assert_latch_matches_numeric("rising, high threshold", &rising, 0.0, 0.0, 1.0, 6.0, true);
        // Started mid-segment, with stress carried in and a later t0.
        assert_latch_matches_numeric("mid-segment", &decaying, 0.15, 0.4, 0.22, 4.0, true);
        assert_latch_matches_numeric("mid-segment, out of reach", &decaying, 0.15, 0.4, 0.3, 4.0, false);
        assert_latch_matches_numeric("mid-segment, bleeding first", &rising, 0.2, 0.3, 0.6, 6.0, true);
        // A fuse: rNormal 0.1, iRating 0.5 gives k = 1/(0.1*0.5)^2 = 400, so 0.1 V
        // across it is 1 A (rate 3), settling toward 0.04 V (0.4 A, bleeding at 0.3).
        let fuse = seg(400.0, 1.0, 0.3, 0.04, 0.06, 0.7, 0.0);
        assert_latch_matches_numeric("fuse", &fuse, 0.0, 0.0, FUSE_THRESHOLD, 4.0, true);
        assert_latch_matches_numeric("fuse, high threshold", &fuse, 0.0, 0.0, 0.9, 4.0, true);
    }

    #[test]
    fn closed_form_root_over_a_segment_that_crosses_zero() {
        // V falls from 8 V through 0 toward -4 V with tau 0.5: k = 1/9, c = 1
        // gives |V| > 3. Piece one (V > 3) ends at s = 0.5*ln(12/7) = 0.27 with a
        // gain of 0.625, the voltage is inside the band until s = 0.5*ln 12 =
        // 1.24, and piece two (V < -3) runs to infinity where the rate settles
        // at 16/9 - 1 = 0.778.
        let s = seg(1.0 / 9.0, 1.0, 1.0, -4.0, 12.0, 0.5, 0.0);
        let total_one = s.gain(0.0, 0.5 * (12.0f64 / 7.0).ln());
        assert!((total_one - 0.625).abs() < 0.01, "setup: piece one gains {total_one}");
        // Thresholds inside piece one, early and at its end.
        assert_latch_matches_numeric("first piece", &s, 0.0, 0.0, 0.05, 6.0, true);
        assert_latch_matches_numeric("first piece, late", &s, 0.0, 0.0, 0.6, 6.0, true);
        assert_latch_matches_numeric("first piece, carried stress", &s, 0.3, 0.0, 0.9, 6.0, true);
        // Above piece one's total: the gap bleeds the stress away first and the
        // latch falls in piece two.
        assert_latch_matches_numeric("second piece", &s, 0.0, 0.0, 1.0, 6.0, true);
        assert_latch_matches_numeric("second piece, carried stress", &s, 0.3, 0.0, 1.2, 6.0, true);
        // The mirror image, -8 V toward +4 V, latches at the same instants.
        let mirrored = seg(1.0 / 9.0, 1.0, 1.0, 4.0, -12.0, 0.5, 0.0);
        assert_latch_matches_numeric("mirrored", &mirrored, 0.0, 0.0, 1.0, 6.0, true);
        let a = StressDrive::Segment(s).time_to_reach(0.0, 0.0, 1.0).unwrap();
        let b = StressDrive::Segment(mirrored).time_to_reach(0.0, 0.0, 1.0).unwrap();
        assert!((a - b).abs() < 1e-12, "{a} vs {b}");
        // A segment that starts after time 0, and a t0 inside it.
        let offset = seg(1.0 / 9.0, 1.0, 1.0, -4.0, 12.0, 0.5, 5.0);
        assert_latch_matches_numeric("offset t_seg", &offset, 0.0, 5.0, 1.0, 11.0, true);
        assert_latch_matches_numeric("offset t_seg, t0 inside", &offset, 0.1, 5.2, 0.6, 11.0, true);
    }

    /// Composite Simpson over `[from, to]`: an integral that does not share the
    /// closed form's algebra and has no kink to straddle inside one piece.
    fn simpson(f: impl Fn(f64) -> f64, from: f64, to: f64, n: usize) -> f64 {
        let h = (to - from) / n as f64;
        let mut sum = f(from) + f(to);
        for i in 1..n {
            sum += f(from + i as f64 * h) * if i % 2 == 1 { 4.0 } else { 2.0 };
        }
        sum * h / 3.0
    }

    #[test]
    fn piece_gain_matches_an_independent_integral() {
        let s = seg(1.0 / 9.0, 1.0, 1.0, -4.0, 12.0, 0.5, 0.0);
        let [first, second] = s.pieces(0.0, f64::INFINITY);
        let (p1, q1) = first.unwrap();
        let (p2, _) = second.unwrap();
        let g1 = s.gain(p1, q1 - p1);
        assert!((g1 - simpson(|t| s.rate_s(t), p1, q1, 200_000)).abs() < 1e-12, "piece one gains {g1}");
        // A stretch of piece two, and a window that starts inside a piece.
        let g2 = s.gain(p2, 1.7);
        assert!((g2 - simpson(|t| s.rate_s(t), p2, p2 + 1.7, 200_000)).abs() < 1e-12, "piece two gains {g2}");
        let g3 = s.gain(0.1, 0.12);
        assert!((g3 - simpson(|t| s.rate_s(t), 0.1, 0.22, 200_000)).abs() < 1e-13, "{g3}");
    }

    #[test]
    fn segment_value_follows_the_bleed_and_the_floor() {
        let s = seg(1.0 / 9.0, 1.0, 1.0, -4.0, 12.0, 0.5, 0.0);
        let drive = StressDrive::Segment(s);
        let [first, second] = s.pieces(0.0, f64::INFINITY);
        let (_, q1) = first.unwrap();
        let (p2, _) = second.unwrap();
        let g1 = s.gain(0.0, q1);
        // Inside piece one, past it (bleeding at 1 per second), and the floor.
        assert!((drive.value_after(0.0, 0.0, 0.1) - s.gain(0.0, 0.1)).abs() < 1e-15);
        assert!((drive.value_after(0.0, 0.0, 0.8) - (g1 - (0.8 - q1))).abs() < 1e-14);
        assert_eq!(drive.value_after(0.0, 0.0, 1.0), 0.0, "bled out before the second piece begins at {p2}");
        assert_eq!(drive.value_after(0.01, 0.4, 1.2), 0.0, "a long gap with little stress ends at exactly 0");
        // Carried stress survives the gap partly when it is large.
        let carried = drive.value_after(5.0, 0.0, 1.0);
        assert!((carried - (5.0 + g1 - (1.0 - q1))).abs() < 1e-13, "{carried}");
        // Closing at a split point and again to the end equals one close.
        let mid = drive.value_after(0.0, 0.0, 0.2);
        let split = drive.value_after(mid, 0.2, 3.0);
        assert!((split - drive.value_after(0.0, 0.0, 3.0)).abs() < 1e-13);
        // The fine numeric integral agrees to within the discontinuity it
        // cannot resolve: S' jumps by the recovery rate where the rate crosses
        // zero, so a step straddling it errs by up to recovery*h.
        for (s0, t_end) in [(0.0, 0.15), (0.0, 0.8), (0.0, 1.5), (0.4, 1.0), (0.0, 0.27)] {
            let h = 2e-6;
            let (_, numeric_s) = numeric(&s, s0, 0.0, f64::INFINITY, t_end, h);
            let closed = drive.value_after(s0, 0.0, t_end);
            assert!((closed - numeric_s).abs() < s.recovery * h + 1e-9, "s0 {s0} to {t_end}: closed {closed}, numeric {numeric_s}");
        }
    }

    #[test]
    fn a_degenerate_unbounded_piece_terminates() {
        // |a| exactly at L: the rate decays to 0 and the piece gains a finite
        // amount, so a big threshold finds nothing and a small one a root.
        let s = seg(1.0, 1.0, 1.0, 1.0, 3.0, 1.0, 0.0);
        let total = StressDrive::Segment(s).time_to_reach(0.0, 0.0, 1.0e9);
        assert_eq!(total, None);
        assert!(StressDrive::Segment(s).time_to_reach(0.0, 0.0, 0.5).is_some());
    }

    #[test]
    fn already_at_threshold_latches_at_t0_under_any_drive() {
        let s = seg(0.02, 1.0, 1.0, 5.0, 5.0, 1.0, 0.0);
        assert_eq!(StressDrive::Segment(s).time_to_reach(0.3, 7.0, 0.3), Some(7.0));
    }

    #[test]
    fn untripped_ptc_segment_trip_matches_a_numeric_integral() {
        // A PTC in series with a charging cap: V across it decays from 0.4 V
        // toward 0.2 V. rNormal 0.5, iHold 0.5: k = 1/R^2 = 4, c = 0.25, bleed
        // 0.125. The piece gains 0.0761 in all against the 0.075 threshold, so
        // the trip falls late in it.
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let v = ExpVoltage::new(0.2, 0.2, 0.4, 0.0);
        let StressDrive::Segment(s) = ptc.untripped_segment(v) else { panic!("a segment drive") };
        assert_eq!((s.k, s.c, s.recovery), (4.0, 0.25, 0.125));
        assert_latch_matches_numeric("ptc segment", &s, 0.0, 0.0, ptc.trip_threshold(), 3.0, true);
        // A little below the hold current's stress budget: no trip in the segment.
        assert_latch_matches_numeric("ptc segment, under budget", &s, 0.0, 0.0, 0.08, 3.0, false);
    }

    #[test]
    fn ptc_cool_test_follows_classics_operation_order() {
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        // At 0.5 V the tripped current is 5 mA: cool.
        assert!(ptc.is_cool(0.5 / ptc.resistance(true)));
        // The power clause binds at about 0.02236 * iHold = 0.01118 A.
        let limit = ptc.cool_current_limit();
        assert!((limit - 0.5 * (0.1f64 / 200.0).sqrt()).abs() < 1e-18);
        assert!(ptc.is_cool(0.0111) && !ptc.is_cool(0.0112));
        assert!(ptc.is_cool(limit * (1.0 - 1e-12)) && !ptc.is_cool(limit * (1.0 + 1e-12)));
        // The half-hold clause binds only for a tiny rNormal-free case that
        // cannot occur: the limit is always the power clause.
        assert!(limit < ptc.i_hold / 2.0);
        // A current at or above the hold current is never cool.
        assert!(!ptc.is_cool(0.5) && !ptc.is_cool(1.0));
        assert_eq!(ptc.resistance(false), 0.5);
        assert_eq!(ptc.resistance(true), 100.0);
    }

    #[test]
    fn ptc_trips_bleeds_and_recovers_at_two_seconds_of_cool_time() {
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let mut e = PtcEntry::new(ptc, 0.0);
        // 1 A: trips at 0.1 (probe F `ptc`: classic 0.1018 at its step end).
        e.set_drive(PtcDrive::Stress(ptc.untripped_constant(1.0)));
        let (t_trip, event) = e.next_event().expect("a trip");
        assert_eq!(event, PtcEvent::Trip);
        assert!((t_trip - 0.1).abs() < 1e-14);
        // Halfway there the projection reads 0.0375 and nothing is committed.
        let projected = e.projected(0.05);
        assert_eq!((projected.0, projected.2), (false, 0.0));
        assert!((projected.1 - 0.0375).abs() < 1e-15);
        // The trip zeroes S and Rt. Its re-solve publishes 0.5 V across the
        // tripped 100 ohm: 5 mA, cool at once, so recovery starts at the trip
        // instant itself and fires 2.0 s after it.
        e.trip(t_trip);
        assert_eq!((e.tripped, e.stress, e.rt), (true, 0.0, 0.0));
        e.set_drive(ptc.tripped_constant(0.5 / ptc.resistance(true), t_trip));
        let (t_rec, event) = e.next_event().expect("a recovery");
        assert_eq!(event, PtcEvent::Recover);
        assert!((t_rec - (t_trip + 2.0)).abs() < 1e-15, "recovers at {t_rec}");
        // At the recovery all three clear.
        e.close(t_rec);
        assert!((e.rt - 2.0).abs() < 1e-14);
        e.recover(t_rec);
        assert_eq!((e.tripped, e.stress, e.rt), (false, 0.0, 0.0));
        // A bleed below the hold current: 0.125 per second.
        e.stress = 0.05;
        e.set_drive(PtcDrive::Stress(ptc.untripped_constant(0.4)));
        e.close(t_rec + 0.2);
        assert!((e.stress - 0.025).abs() < 1e-15);
        assert_eq!(e.next_event(), None, "a bleeding PTC schedules nothing");
    }

    #[test]
    fn rt_after_resets_on_an_uncool_observation_and_counts_cool_time() {
        let inf = f64::INFINITY;
        // The window has not begun at t: an uncool observation, so the 1.2 s
        // already counted are lost.
        assert_eq!(rt_after(1.2, 2.0, 3.0, 5.0, inf), 0.0);
        assert_eq!(rt_after(1.2, 2.0, 3.0, inf, inf), 0.0, "hot");
        // The window is over at t (its end is exclusive).
        assert_eq!(rt_after(1.2, 2.0, 3.0, 0.0, 3.0), 0.0);
        assert_eq!(rt_after(1.2, 2.0, 4.0, 0.0, 3.0), 0.0);
        // Cool at t: the counted time carries and the interval adds.
        assert!((rt_after(1.2, 2.0, 3.0, 0.0, inf) - 2.2).abs() < 1e-15);
        // The window began inside the interval: only the time since it began.
        assert!((rt_after(0.7, 2.0, 3.0, 2.5, inf) - 0.5).abs() < 1e-15);
    }

    #[test]
    fn close_resets_the_recovery_timer_through_rt_after() {
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let mut e = PtcEntry::new(ptc, 0.0);
        e.trip(1.0);
        e.set_drive(PtcDrive::Cool { from: 1.0, until: f64::INFINITY });
        e.close(2.2);
        assert!((e.rt - 1.2).abs() < 1e-15, "cool time accumulates");
        // The drive is swapped without `set_drive`, so only `close` can reset
        // the timer: the interval it closes was observed uncool.
        e.drive = PtcDrive::HOT;
        e.close(3.0);
        assert_eq!(e.rt, 0.0);
        // The same for a window that ended inside the interval.
        let mut f = PtcEntry::new(ptc, 0.0);
        f.trip(0.0);
        f.set_drive(PtcDrive::Cool { from: 0.0, until: 1.0 });
        f.close(0.9);
        assert!((f.rt - 0.9).abs() < 1e-15);
        f.close(1.5);
        assert_eq!(f.rt, 0.0);
    }

    #[test]
    fn set_drive_observes_the_publish_and_restarts_the_timer() {
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let mut e = PtcEntry::new(ptc, 0.0);
        e.trip(1.0);
        e.set_drive(PtcDrive::Cool { from: 1.0, until: f64::INFINITY });
        e.close(2.2);
        // A publish observes a current that is no longer cool: Rt = 0 at once.
        e.set_drive(PtcDrive::HOT);
        assert_eq!(e.rt, 0.0);
        assert_eq!(e.next_event(), None, "no recovery while hot");
        // Cool again from 3.0: the timer restarts from there, not from 1.0.
        e.close(3.0);
        e.set_drive(PtcDrive::Cool { from: 3.0, until: f64::INFINITY });
        assert_eq!(e.next_event(), Some((5.0, PtcEvent::Recover)));
        e.close(4.0);
        assert!((e.rt - 1.0).abs() < 1e-15);
        // A publish that keeps it cool keeps the timer.
        e.set_drive(PtcDrive::Cool { from: 3.0, until: f64::INFINITY });
        assert!((e.rt - 1.0).abs() < 1e-15);
    }

    #[test]
    fn a_tripped_window_that_covers_now_starts_at_now_exactly() {
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        // `t_seg + (now - t_seg)` rounds one ulp after `now` here.
        let v = ExpVoltage::new(0.1, 0.2, 0.5, 0.163);
        let (t_seg, now) = (0.163f64, 0.43f64);
        assert!(t_seg + (now - t_seg) > now, "setup: the sum overshoots");
        let drive = ptc.tripped_segment(v, 0.43);
        assert_eq!(drive, PtcDrive::Cool { from: 0.43, until: f64::INFINITY });
        // So a timer that was counting survives the publish.
        let mut e = PtcEntry::new(ptc, 0.0);
        e.trip(0.0);
        e.set_drive(PtcDrive::Cool { from: 0.0, until: f64::INFINITY });
        e.close(0.43);
        e.set_drive(drive);
        assert_eq!(e.rt, 0.43, "set_drive must not read an uncool observation");
        // The same over many segments: whenever |V(now)| is inside the cool
        // level the window starts at `now`, never an ulp after.
        let level = ptc.cool_current_limit() * ptc.resistance(true);
        let mut state = 0x9E37_79B9_7F4A_7C15u64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut inside = 0;
        for _ in 0..5000 {
            let t_seg = next() * 10.0;
            let now = t_seg + next() * 5.0;
            let v = ExpVoltage::new(6.0 * next() - 3.0, 6.0 * next() - 3.0, 0.05 + 3.0 * next(), t_seg);
            if v.at(now).abs() >= level * (1.0 - 1e-9) {
                continue;
            }
            inside += 1;
            match ptc.tripped_segment(v, now) {
                PtcDrive::Cool { from, .. } => assert_eq!(from, now, "t_seg {t_seg} now {now}"),
                other => panic!("{other:?} for a voltage inside the level at {now}"),
            }
        }
        assert!(inside > 500, "the sweep must reach the case: {inside} inside");
    }

    #[test]
    fn tripped_segment_window_matches_a_brute_force_scan() {
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let level = ptc.cool_current_limit() * ptc.resistance(true);
        // Falling into the band, always in it, rising out of it, through it
        // from one side to the other, and never in it.
        for (a, b) in [(-0.3, 1.5), (0.0, 0.8), (0.4, -0.38), (3.0, 1.0), (-0.2, 0.1), (2.0, -1.5), (3.0, -6.0), (-3.0, 6.0)] {
            let v = ExpVoltage::new(a, b, 0.9, 0.0);
            let drive = ptc.tripped_segment(v, 0.0);
            let n = 40_000;
            let dt = 6.0 / f64::from(n);
            let mut first = None;
            let mut last = None;
            for i in 0..n {
                let t = f64::from(i) * dt;
                if v.at(t).abs() < level {
                    first.get_or_insert(t);
                    last = Some(t);
                }
            }
            match (drive, first, last) {
                (PtcDrive::Cool { from, until }, Some(f), Some(l)) => {
                    assert!((from - f).abs() <= dt + 1e-12, "({a}, {b}) from {from} vs {f}");
                    if until.is_finite() {
                        assert!((until - l).abs() <= dt + 1e-12, "({a}, {b}) until {until} vs {l}");
                    } else {
                        assert!(l >= 6.0 - 2.0 * dt, "({a}, {b}) cool to the end of the scan");
                    }
                }
                (d, None, None) => assert_eq!(d, PtcDrive::HOT, "({a}, {b})"),
                (d, f, l) => panic!("({a}, {b}): {d:?} against scan {f:?}..{l:?}"),
            }
        }
    }

    #[test]
    fn segment_recovery_window_is_bounded_by_the_window_end() {
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let mut e = PtcEntry::new(ptc, 0.0);
        e.trip(0.0);
        // Cool on [1, 2.5): only 1.5 s, so no recovery in this segment.
        e.set_drive(PtcDrive::Cool { from: 1.0, until: 2.5 });
        assert_eq!(e.next_event(), None);
        // Cool on [1, 4): recovery at 3.0.
        e.set_drive(PtcDrive::Cool { from: 1.0, until: 4.0 });
        assert_eq!(e.next_event(), Some((3.0, PtcEvent::Recover)));
        // Closing past the window's end leaves the timer reset.
        e.close(5.0);
        assert_eq!(e.rt, 0.0);
        // Time already counted carries in when the window began before t0.
        let mut f = PtcEntry::new(ptc, 0.0);
        f.trip(0.0);
        f.set_drive(PtcDrive::Cool { from: 0.0, until: f64::INFINITY });
        f.close(1.5);
        assert_eq!(f.next_event(), Some((2.0, PtcEvent::Recover)));
        // A timer that has counted and then meets the window's end inside one
        // closed interval is reset by the uncool observation at its end, not
        // carried: cool on [0, 1.8) with 1.5 s counted, observed at 3.0.
        let mut g = PtcEntry::new(ptc, 0.0);
        g.trip(0.0);
        g.set_drive(PtcDrive::Cool { from: 0.0, until: 1.8 });
        g.close(1.5);
        assert_eq!(g.rt, 1.5);
        assert_eq!(g.next_event(), None, "0.5 s short of 2.0 when the window ends");
        g.close(3.0);
        assert_eq!(g.rt, 0.0);
    }

    #[test]
    fn ptc_reset_clears_trip_and_stress() {
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let mut e = PtcEntry::new(ptc, 0.0);
        e.trip(0.1);
        e.set_drive(PtcDrive::Cool { from: 0.1, until: f64::INFINITY });
        e.close(1.0);
        e.reset(1.0);
        assert_eq!((e.tripped, e.stress, e.rt, e.t0), (false, 0.0, 0.0, 1.0));
        // An untripped one with stress on board restarts from 0 and keeps its
        // drive, so it trips 0.1 s after the reset, as classic does (probe F).
        let mut u = PtcEntry::new(ptc, 0.0);
        u.set_drive(PtcDrive::Stress(ptc.untripped_constant(1.0)));
        u.close(0.06);
        assert!(u.stress > 0.04);
        u.reset(0.06);
        assert_eq!((u.tripped, u.stress, u.rt, u.t0), (false, 0.0, 0.0, 0.06));
        let (t, event) = u.next_event().expect("a trip");
        assert_eq!(event, PtcEvent::Trip);
        assert!((t - 0.16).abs() < 1e-14, "re-trips at {t}");
    }

    #[test]
    fn ptc_trip_threshold_keeps_classics_operation_order() {
        // Classic's `3 * iHold * iHold * 0.1` (PA:789) run in node. Each of
        // these iHold values rounds differently under every reassociation
        // (`0.3*i*i`, `3*(i*i)*0.1`, `3*0.1*i*i`, `i*i*0.3`, `i*i*3*0.1`), so a
        // reordering shows as a different bit pattern.
        let cases: [(f64, f64); 4] = [(0.15, 0.006749999999999999), (0.3, 0.026999999999999996), (0.6, 0.10799999999999998), (1.2, 0.43199999999999994)];
        for (i, expected) in cases {
            let ptc = PtcParams { r_normal: 0.5, i_hold: i };
            assert_eq!(ptc.trip_threshold().to_bits(), expected.to_bits(), "iHold {i}");
            for (name, variant) in [("0.3*i*i", 0.3 * i * i), ("3*(i*i)*0.1", 3.0 * (i * i) * 0.1), ("3*0.1*i*i", 3.0 * 0.1 * i * i), ("i*i*0.3", i * i * 0.3), ("i*i*3*0.1", i * i * 3.0 * 0.1)] {
                assert_ne!(variant.to_bits(), expected.to_bits(), "iHold {i}: {name} would not discriminate");
            }
        }
    }

    #[test]
    fn ptc_cool_test_keeps_classics_operation_order_at_its_boundary() {
        // Classic's `current < iHold/2 && current*current*rNormal*200 <
        // iHold*iHold*rNormal*0.1` (PA:805-807) run in node, at currents within
        // an ulp of the boundary where each reassociation of either power
        // changes the answer.
        let cases: [(f64, f64, f64, bool); 6] = [
            (0.3, 0.1, 0.006708203932499369, false),
            (0.3, 0.3, 0.006708203932499369, true),
            (0.5, 0.1, 0.011180339887498949, true),
            (0.5, 0.001, 0.011180339887498949, true),
            (0.5, 0.7, 0.011180339887498947, true),
            (0.9, 0.7, 0.020124611797498106, true),
        ];
        let cool_assoc = |i: f64, h: f64, r: f64| i < h / 2.0 && i * i * r * 200.0 < h * h * (r * 0.1);
        let tripped_assoc = |i: f64, h: f64, r: f64| i < h / 2.0 && i * i * (r * 200.0) < h * h * r * 0.1;
        let r_first = |i: f64, h: f64, r: f64| i < h / 2.0 && i * i * r * 200.0 < r * 0.1 * h * h;
        let no_r = |i: f64, h: f64, r: f64| {
            let _ = r;
            i < h / 2.0 && i * i * 200.0 < h * h * 0.1
        };
        for (h, r, i, expected) in cases {
            let ptc = PtcParams { r_normal: r, i_hold: h };
            assert_eq!(ptc.is_cool(i), expected, "iHold {h} rNormal {r} I {i}");
        }
        for (name, variant) in [("cool power reassociated", &cool_assoc as &dyn Fn(f64, f64, f64) -> bool), ("tripped power reassociated", &tripped_assoc), ("rNormal first", &r_first), ("rNormal cancelled", &no_r)] {
            assert!(cases.iter().any(|&(h, r, i, expected)| variant(i, h, r) != expected), "{name}: no case discriminates it");
        }
    }

    #[test]
    fn a_segment_that_does_not_relax_is_a_constant_drive() {
        // k = 0.08, c = 1: a held 5 V is rate 0.08*25 - 1 = 1. `tau = 0` with
        // `b = 0`, and `tau = INF` with `b = 0`, divided by zero or by infinity
        // and returned `Some(NaN)`; `tau = INF` holds `a + b` and `tau = 0` holds `a`.
        let inf = f64::INFINITY;
        for (a, b, tau) in [(5.0, 0.0, 0.0), (5.0, 0.0, inf), (3.0, 2.0, inf), (5.0, 2.0, 0.0), (5.0, 0.0, 1.0)] {
            let drive = StressDrive::Segment(seg(0.08, 1.0, 1.0, a, b, tau, 0.0));
            assert!(drive.accumulates(), "a {a} b {b} tau {tau}");
            let t = drive.time_to_reach(0.25, 3.0, 1.0).expect("a latch time");
            assert!((t - 3.75).abs() < 1e-14, "a {a} b {b} tau {tau}: {t}");
            let v = drive.value_after(0.25, 3.0, 3.5);
            assert!((v - 0.75).abs() < 1e-14, "a {a} b {b} tau {tau}: {v}");
            assert_eq!(drive.zero_time(0.25, 3.0), None);
        }
        // Held at 1 V the rate is 0.08 - 1 < 0: it bleeds, reaches 0 and
        // schedules nothing.
        for (a, b, tau) in [(1.0, 0.0, 0.0), (1.0, 0.0, inf), (0.5, 0.5, inf)] {
            let drive = StressDrive::Segment(seg(0.08, 1.0, 0.5, a, b, tau, 0.0));
            assert!(!drive.accumulates());
            assert_eq!(drive.time_to_reach(0.25, 3.0, 1.0), None);
            assert!((drive.value_after(0.25, 3.0, 3.2) - 0.15).abs() < 1e-14);
            assert_eq!(drive.value_after(0.25, 3.0, 9.0), 0.0);
            assert!((drive.zero_time(0.25, 3.0).unwrap() - 3.5).abs() < 1e-14);
        }
    }

    #[test]
    fn ptc_segments_that_do_not_relax_collapse_to_the_constant_case() {
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let inf = f64::INFINITY;
        // 0.3 V across the tripped 100 ohm is 3 mA: cool from `now` for good.
        for v in [ExpVoltage::new(0.3, 0.0, inf, 2.0), ExpVoltage::new(0.3, 0.0, 0.0, 2.0), ExpVoltage::new(-0.1, 0.4, inf, 2.0), ExpVoltage::new(0.3, 5.0, 0.0, 2.0)] {
            assert_eq!(ptc.tripped_segment(v, 3.0), PtcDrive::Cool { from: 3.0, until: inf }, "{v:?}");
        }
        // At the segment's own origin `0/0` would be NaN for `tau = 0`.
        assert_eq!(ptc.tripped_segment(ExpVoltage::new(0.3, 0.5, 0.0, 3.0), 3.0), PtcDrive::Cool { from: 3.0, until: inf });
        // 5 V across it is 50 mA: hot.
        assert_eq!(ptc.tripped_segment(ExpVoltage::new(5.0, 0.0, inf, 2.0), 3.0), PtcDrive::HOT);
        // Untripped, 0.5 V across 0.5 ohm is 1 A, which trips 0.1 s on.
        let drive = ptc.untripped_segment(ExpVoltage::new(0.5, 0.0, 0.0, 0.0));
        let t = drive.time_to_reach(0.0, 0.0, ptc.trip_threshold()).expect("a trip time");
        assert!((t - 0.1).abs() < 1e-14, "{t}");
    }

    // ---- a stepped step's mean (rulings R80 (a), R84 (B)) -------------------

    /// `S'` over one step of length 1 for a ratio `q(t)`, by the midpoint rule
    /// on a fine grid: the integral the step's mean stands for.
    fn step_integral(q: impl Fn(f64) -> f64, zero: f64) -> f64 {
        let n = 200_000;
        (0..n).map(|k| (q((k as f64 + 0.5) / n as f64) - zero).max(0.0)).sum::<f64>() / n as f64
    }

    /// `S` gained over a step of 1 from `s0`, the step's ends at rates `r0`
    /// and `r1` (zero level 1, recovery 1, `q` unknown).
    fn step_rate(r0: f64, r1: f64) -> f64 {
        let at = |rate: f64| StressDrive::Constant { rate, recovery: 1.0, zero: 1.0, q: -1.0 };
        step_gain(StressDrive::over_step(at(r0), at(r1)), 100.0)
    }

    /// `S` gained over a step from 0 to 1 under `mean`, from `s0` (threshold
    /// out of reach).
    fn step_gain(mean: StepMean, s0: f64) -> f64 {
        let mut s = Stress { s0, t0: 0.0, drive: StressDrive::IDLE, threshold: 1e9 };
        assert_eq!(s.run_step(&mean, 0.0, 1.0), None);
        s.s0 - s0
    }

    #[test]
    fn a_steps_mean_is_exact_for_a_ratio_exponential_in_time() {
        // A ratio q = A e^(kt) over the step, falling or rising: the step's
        // rate is the integral of q - 1 to the grid's 1e-9. The first is q from
        // 1.001 to 3, where the logarithmic mean of the rates themselves (0.001
        // and 2) reads 0.263 and the trapezoid 1.0005; the last cold 104 tpl
        // 6's discharge spike, q from 94 to 27 (trapezoid 59.5).
        for (q0, q1, by_hand) in [(1.001f64, 3.0f64, 0.8212), (2.0, 30.0, 9.3395), (94.0, 27.0, 52.7092)] {
            let exact = step_integral(|t| q0 * (q1 / q0).powf(t), 1.0);
            let got = step_rate(q0 - 1.0, q1 - 1.0);
            assert!((got - exact).abs() <= 1e-9 * exact.max(1.0), "q {q0} -> {q1}: {got} against {exact}");
            assert!((got - by_hand).abs() <= 1e-4, "q {q0} -> {q1}: {got} against {by_hand} by hand");
        }
    }

    #[test]
    fn a_steps_mean_is_continuous_as_an_end_falls_to_the_zero_level() {
        // An end at rate 0 is taken at the zero level, the limit of an end
        // whose rate falls to 0, so a rate of 1e-12 and of 0 hold the same.
        for r1 in [2.0, 0.5, 300.0] {
            let (at_0, near_0) = (step_rate(0.0, r1), step_rate(1e-12, r1));
            assert!((at_0 - near_0).abs() <= 1e-11, "end rate {r1}: {at_0} at 0, {near_0} at 1e-12");
            assert_eq!(step_rate(r1, 0.0), at_0, "the mean is symmetric in its ends");
        }
    }

    #[test]
    fn a_step_whose_ratio_crosses_the_zero_level_holds_between_the_integral_and_the_trapezoid() {
        // q from 0.9 to 3 crosses 1 inside the step: the end at rate 0 is read
        // at the zero level, so the mean (0.8205) over-counts the integral
        // (0.7487) by the step's stretch under the level, and stays under the
        // trapezoid's 1.0; the same falling, q from 3 to 0.5 (0.5031).
        for (q0, q1) in [(0.9f64, 3.0f64), (3.0, 0.5)] {
            let exact = step_integral(|t| q0 * (q1 / q0).powf(t), 1.0);
            let got = step_rate((q0 - 1.0f64).max(0.0), (q1 - 1.0f64).max(0.0));
            let trapezoid = 0.5 * ((q0 - 1.0f64).max(0.0) + (q1 - 1.0f64).max(0.0));
            assert!(got >= exact && got < trapezoid, "q {q0} -> {q1}: {got}, integral {exact}, trapezoid {trapezoid}");
            assert!((got - 0.8205).abs() <= 1e-4, "{got}");
        }
    }

    #[test]
    fn the_log_mean_of_near_equal_ends_is_their_series() {
        // (b - a) / ln(b / a) is 0/0 at a = b: the series a (1 + d/2 - d^2/12
        // + d^3/24) against the quotient at d = 1e-6, the two sides of the
        // switch at d = 1e-8, and a = b.
        assert_eq!(log_mean(50.0, 50.0), 50.0);
        let d = 1e-6;
        let series = 50.0 * (1.0 + d / 2.0 - d * d / 12.0 + d * d * d / 24.0);
        assert!((log_mean(50.0, 50.0 * (1.0 + d)) - series).abs() <= 4e-16 * 50.0, "{} against {series}", log_mean(50.0, 50.0 * (1.0 + d)));
        for a in [1e-3, 1.0, 50.0, 1e6] {
            let (under, over) = (log_mean(a, a * (1.0 + 0.999e-8)), log_mean(a, a * (1.0 + 1.001e-8)));
            assert!((over - under) / a >= 0.0 && (over - under) / a <= 2e-11 + 4e-16, "a {a}: {under} under the switch, {over} over it");
            assert_eq!(log_mean(a, 3.0 * a), log_mean(3.0 * a, a), "symmetric");
        }
        assert!((log_mean(2.0, 30.0) - 28.0 / 15.0f64.ln()).abs() <= 1e-15 * 10.0);
    }

    #[test]
    fn a_step_that_accumulates_at_neither_end_bleeds_at_the_mean_recovery() {
        let idle = |recovery: f64| StressDrive::Constant { rate: 0.0, recovery, zero: 1.0, q: -1.0 };
        let mean = StressDrive::over_step(idle(1.0), idle(2.0));
        assert_eq!(mean.split, 1.0);
        let StressDrive::Constant { rate, recovery, .. } = mean.first else { panic!("a constant drive") };
        assert_eq!((rate, recovery), (0.0, 1.5));
    }

    #[test]
    fn a_ptcs_constant_drive_carries_its_hold_current_squared_as_its_zero_level() {
        // An untripped PTC's rate is I^2 - I_hold^2: q = I^2, zero I_hold^2.
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let StressDrive::Constant { zero, .. } = ptc.untripped_constant(1.0) else { panic!("a constant drive") };
        assert_eq!(zero, 0.25);
        let StressDrive::Constant { zero, .. } = ptc.untripped_constant(0.1) else { panic!("a constant drive") };
        assert_eq!(zero, 0.25);
    }

    /// `S'` over a step whose ends sit at ratios `q0` and `q1` (zero level 1,
    /// recovery `rec`), as the step's mean drive holds it: the rate, or minus
    /// the recovery it bleeds at.
    fn crossing_slope(q0: f64, q1: f64, rec: f64) -> f64 {
        crossing_gain(q0, q1, rec, 100.0)
    }

    /// `S` gained over a step of 1 from `s0` whose ends sit at ratios `q0`
    /// and `q1` (zero level 1, recovery `rec`).
    fn crossing_gain(q0: f64, q1: f64, rec: f64, s0: f64) -> f64 {
        let at = |q: f64| StressDrive::Constant { rate: (q - 1.0).max(0.0), recovery: rec, zero: 1.0, q };
        step_gain(StressDrive::over_step(at(q0), at(q1)), s0)
    }

    #[test]
    fn a_step_whose_ratio_falls_through_the_zero_level_accrues_to_the_crossing_then_bleeds() {
        // q = 3 (1/6)^t over a step of 1, recovery 1/s: by hand it crosses 1
        // at f = ln 3 / ln 6 = 0.613147, accrues f (2 / ln 3 - 1) = 0.503074
        // up to there and bleeds 1 - f = 0.386853 after: 0.116221. Read at the
        // zero level, the end at rate 0 had it accrue 0.8205 and never bleed
        // (cold 323 tpl 6's R1 burnt 23 % early).
        let exact = step_integral_bleeding(|t| 3.0 * (0.5f64 / 3.0).powf(t), 1.0);
        let got = crossing_slope(3.0, 0.5, 1.0);
        assert!((got - 0.116221).abs() <= 1e-6, "{got}, by hand 0.116221");
        // The grid's midpoint rule steps over the jump at the crossing: 1e-5.
        assert!((got - exact).abs() <= 1e-5, "{got} against the integral {exact}");
    }

    #[test]
    fn a_step_whose_ratio_rises_through_the_zero_level_bleeds_to_the_crossing_then_accrues() {
        // q = 0.9 (10/3)^t: under 1 for 1 - f = 0.087511 of the step, f = ln 3
        // / ln(3/0.9); above it accrues f (2 / ln 3 - 1) = 0.748678: 0.661167.
        let exact = step_integral_bleeding(|t| 0.9 * (3.0f64 / 0.9).powf(t), 1.0);
        let got = crossing_slope(0.9, 3.0, 1.0);
        assert!((got - 0.661167).abs() <= 1e-6, "{got}, by hand 0.661167");
        // The grid's midpoint rule steps over the jump at the crossing: 1e-5.
        assert!((got - exact).abs() <= 1e-5, "{got} against the integral {exact}");
    }

    #[test]
    fn a_step_that_reaches_the_zero_level_at_its_end_accrues_all_of_it() {
        // q from 3 to exactly 1 (and back): the crossing is the step's end, so
        // nothing bleeds: 2 / ln 3 - 1 = 0.820478 either way; and an end a
        // hair under the level meets it.
        for (q0, q1) in [(3.0, 1.0), (1.0, 3.0)] {
            let got = crossing_slope(q0, q1, 1.0);
            assert!((got - 0.820478).abs() <= 1e-6, "q {q0} -> {q1}: {got}");
        }
        assert!((crossing_slope(3.0, 1.0 - 1e-12, 1.0) - 0.820478).abs() <= 1e-6);
    }

    /// As [`step_integral`], with the integrator bleeding at `rec` wherever
    /// the ratio is under 1 (S stays positive over the step).
    fn step_integral_bleeding(q: impl Fn(f64) -> f64, rec: f64) -> f64 {
        let n = 200_000;
        (0..n).map(|k| { let v = q((k as f64 + 0.5) / n as f64); if v > 1.0 { v - 1.0 } else { -rec } }).sum::<f64>() / n as f64
    }

    #[test]
    fn a_rising_crossing_from_the_floor_bleeds_nothing_before_it_accrues() {
        // From S = 0 the bleed before the crossing has nothing to take, so the
        // step gains all it accrues after: q = 0.5 6^t crosses 1 at ln 2 / ln 6
        // and accrues 0.503074 after; q = 0.1 30^t is above 1 for ln 3 / ln 30 and
        // accrues 0.265021. Netted into one slope, the bleed ate the first
        // (0.116) and all of the second (R80V).
        for (q0, by_hand) in [(0.5, 0.503074), (0.1, 0.265021)] {
            let got = crossing_gain(q0, 3.0, 1.0, 0.0);
            assert!((got - by_hand).abs() <= 1e-6, "q {q0} -> 3 from S = 0: {got}, by hand {by_hand}");
        }
    }

    #[test]
    fn a_falling_crossing_latches_in_its_accrual_though_the_step_nets_a_loss() {
        // q = 3 (0.2/3)^t from S = 0.8, threshold 1, recovery 1: it accrues
        // 0.8205/s until f = ln 3 / ln 15 = 0.4057 and bleeds after, a net
        // loss of 0.26 over the step, but S reaches 1 at 0.2 / 0.8205 = 0.2438.
        let at = |q: f64| StressDrive::Constant { rate: (q - 1.0f64).max(0.0), recovery: 1.0, zero: 1.0, q };
        let mean = StressDrive::over_step(at(3.0), at(0.2));
        let mut s = Stress { s0: 0.8, t0: 0.0, drive: StressDrive::IDLE, threshold: 1.0 };
        let t = s.run_step(&mean, 0.0, 1.0).expect("a latch inside the step");
        assert!((t - 0.2 / (2.0 / 3.0f64.ln() - 1.0)).abs() <= 1e-12, "{t}");
    }

    #[test]
    fn an_end_at_zero_ratio_is_a_voltage_through_zero() {
        // q = 3 (1 - t)^2 (a voltage linear to 0): above 1 until 1 - 1/sqrt 3,
        // accruing (1 - 3^-1.5) - (1 - 1/sqrt 3) = 0.384900, then bleeding
        // 1/sqrt 3 = 0.577350 at recovery 1: -0.192450 from a high S. Read as
        // unknown it was 0.820 accrued and no bleed.
        let got = crossing_gain(3.0, 0.0, 1.0, 100.0);
        assert!((got + 0.192450).abs() <= 1e-6, "{got}");
        assert!((crossing_gain(0.0, 3.0, 1.0, 100.0) - got).abs() <= 1e-12, "the same rising");
    }
}

// ---- W2a: heap bytes (generated by the size walk generator; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for SegmentRate {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.k, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.recovery, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for StepMean {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.first, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.split, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.then, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for StressDrive {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            StressDrive::Constant { rate, recovery, zero, q } => crate::snapshot::HeapBytes::heap_bytes(rate, w) + crate::snapshot::HeapBytes::heap_bytes(recovery, w) + crate::snapshot::HeapBytes::heap_bytes(zero, w) + crate::snapshot::HeapBytes::heap_bytes(q, w),
            StressDrive::Segment(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for Stress {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drive, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.threshold, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PtcParams {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.r_normal, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i_hold, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PtcDrive {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            PtcDrive::Stress(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            PtcDrive::Cool { from, until } => crate::snapshot::HeapBytes::heap_bytes(from, w) + crate::snapshot::HeapBytes::heap_bytes(until, w),
        }
    }
}

impl crate::snapshot::HeapBytes for PtcEvent {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            PtcEvent::Trip => 0,
            PtcEvent::Recover => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for PtcEntry {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.params, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.tripped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stress, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rt, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drive, w); }
        n
    }
}
