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

/// What drives a stress integrator from `t0` on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StressDrive {
    /// Static, fixed-pair and one stepped step (the step's end rate held over
    /// the step, SE:5510-5531 backward rule): `S' = rate` when `rate > 0`,
    /// else `S' = -recovery`.
    Constant { rate: f64, recovery: f64 },
    /// A closed-form segment.
    Segment(SegmentRate),
}

impl StressDrive {
    /// Nothing accumulates: the state of a device that sits at rate 0.
    pub const IDLE: StressDrive = StressDrive::Constant { rate: 0.0, recovery: 0.0 };

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

    /// `S(t)` from `s0` at `t0`, committing nothing (the readout of point 16
    /// and what `Stress::close` stores).
    pub fn value_after(&self, s0: f64, t0: f64, t: f64) -> f64 {
        if !(t > t0) {
            return s0;
        }
        match self {
            StressDrive::Constant { rate, recovery } => constant_value(s0, *rate, *recovery, t - t0),
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

    /// The time a bleeding integrator reaches 0: `None` unless it is bleeding
    /// from a positive value at a positive recovery (a closed-form segment
    /// returns `None` too: its entry is dropped lazily at the next close).
    pub fn zero_time(&self, s0: f64, t0: f64) -> Option<f64> {
        match self {
            StressDrive::Constant { rate, recovery } => bleed_zero_time(s0, t0, *rate, *recovery),
            StressDrive::Segment(seg) => seg.collapsed().and_then(|constant| constant.zero_time(s0, t0)),
        }
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

/// The set `{s >= s1 : a + b*exp(-s/tau) > level}` as `(start, end)` in segment
/// time, `end` possibly infinite. `V` is monotone in `s`, so the set is a
/// prefix, a suffix, everything or nothing, decided by `V(s1)` and the limit
/// `a`, and a crossing is solved once, analytically.
fn above(a: f64, b: f64, tau: f64, level: f64, s1: f64) -> Option<(f64, f64)> {
    let v1 = a + b * (-s1 / tau).exp();
    // `b` is nonzero whenever a crossing is needed: with `b == 0` the voltage is
    // the constant `a`, which one of the two branches below already decides.
    let crossing = || (-tau * ((level - a) / b).ln()).max(s1);
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

impl SegmentRate {
    /// The constant drive this segment is when its voltage does not relax
    /// (`ExpVoltage::constant_value`): the closed forms below divide by `tau`,
    /// and a latch time of `Some(NaN)` for `tau = 0, b = 0` would be an event
    /// the kernel schedules.
    fn collapsed(&self) -> Option<StressDrive> {
        let volts = self.v.constant_value()?;
        Some(StressDrive::Constant { rate: self.k * volts * volts - self.c, recovery: self.recovery })
    }

    fn rate_s(&self, s: f64) -> f64 {
        let v = self.v.a + self.v.b * (-s / self.v.tau).exp();
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
        let ExpVoltage { a, b, tau, .. } = self.v;
        let clip = |piece: Option<(f64, f64)>| {
            piece.and_then(|(p, q)| {
                let q = q.min(s2);
                (p < q).then_some((p, q))
            })
        };
        let up = clip(above(a, b, tau, level, s1));
        let down = clip(above(-a, -b, tau, level, s1));
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
    fn gain(&self, p: f64, d: f64) -> f64 {
        let ExpVoltage { a, b, tau, .. } = self.v;
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
            StressDrive::Constant { rate: (current * current - self.i_hold * self.i_hold).max(0.0), recovery: 0.0 }
        } else {
            StressDrive::Constant { rate: 0.0, recovery: self.bleed_rate() }
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
        let below = above(-v.a, -v.b, v.tau, -level, s1);
        let over = above(v.a, v.b, v.tau, -level, s1);
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
        let resistor = StressDrive::Constant { rate: 2.0 - 1.0, recovery: 1.0 };
        assert!((resistor.time_to_reach(0.0, 0.0, RESISTOR_THRESHOLD).unwrap() - 1.0).abs() < 1e-15);
        // Fuse at 2x (1 A through 0.5 A): (I/iRating)^2 - 1 = 3, threshold 0.3: t = 0.1.
        let fuse = StressDrive::Constant { rate: (1.0f64 / 0.5).powi(2) - 1.0, recovery: 0.3 };
        assert!((fuse.time_to_reach(0.0, 0.0, FUSE_THRESHOLD).unwrap() - 0.1).abs() < 1e-15);
        // PTC at 2x hold (1 A, iHold 0.5): I^2 - iHold^2 = 0.75 against 3*0.5*0.5*0.1 = 0.075.
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let drive = ptc.untripped_constant(1.0);
        assert_eq!(drive, StressDrive::Constant { rate: 0.75, recovery: 0.0 });
        assert!((ptc.trip_threshold() - 0.075).abs() < 1e-15);
        assert!((drive.time_to_reach(0.0, 0.0, ptc.trip_threshold()).unwrap() - 0.1).abs() < 1e-14);
        // From a nonzero start and a later t0.
        assert!((resistor.time_to_reach(0.25, 3.0, RESISTOR_THRESHOLD).unwrap() - 3.75).abs() < 1e-15);
        // No rate, no event; already at the threshold is due now.
        assert_eq!(StressDrive::Constant { rate: 0.0, recovery: 1.0 }.time_to_reach(0.5, 0.0, 1.0), None);
        assert_eq!(resistor.time_to_reach(1.0, 4.0, 1.0), Some(4.0));
    }

    #[test]
    fn recovery_bleeds_floor_at_zero_and_find_the_zero_time() {
        // Recoveries 1 (resistor), 0.3 (fuse) and 2 (LED) per second from 0.5.
        for (recovery, after_0_2, zero_at) in [(1.0, 0.3, 0.5), (0.3, 0.44, 0.5 / 0.3), (2.0, 0.1, 0.25)] {
            let drive = StressDrive::Constant { rate: 0.0, recovery };
            assert!((drive.value_after(0.5, 0.0, 0.2) - after_0_2).abs() < 1e-15, "recovery {recovery}");
            assert_eq!(drive.value_after(0.5, 0.0, zero_at + 1.0), 0.0, "floored at 0");
            assert!((drive.zero_time(0.5, 0.0).unwrap() - zero_at).abs() < 1e-15);
            assert!(!drive.accumulates());
        }
        // A negative recovery never grows the integrator (classic clamps it).
        assert_eq!(StressDrive::Constant { rate: 0.0, recovery: -1.0 }.value_after(0.5, 0.0, 2.0), 0.5);
        // Accumulating drives have no zero time.
        assert_eq!(StressDrive::Constant { rate: 1.0, recovery: 1.0 }.zero_time(0.5, 0.0), None);
        // The PTC bleeds at 0.125 per second at iHold 0.5.
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        let bleed = ptc.untripped_constant(0.4);
        assert_eq!(bleed, StressDrive::Constant { rate: 0.0, recovery: 0.125 });
        assert!((bleed.value_after(0.05, 0.0, 0.2) - 0.025).abs() < 1e-15);
    }

    #[test]
    fn close_commits_and_value_at_projects() {
        let mut s = Stress::new(1.0, 2.0);
        s.drive = StressDrive::Constant { rate: 0.5, recovery: 1.0 };
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
        s.drive = StressDrive::Constant { rate: 0.5, recovery: 1.0 };
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
}
