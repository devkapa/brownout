//! `signal_gen` waveform layer (W1h item H3a, core/spec §8.12, Q-O67 to Q-O74, D34): the pure part. No solver
//! code lives here; the Norton drive channel, the ideal form and the sine time function are H3b's, in the
//! F5-owned solver files.
//!
//! - [`signal_gen_voltage`]: classic's `signalGenVoltage` (`waveform.ts:166-200`) for the eight shapes, every
//!   operation in classic's order so a deterministic shape agrees to the bit (`%` is fmod, as JS's). Noise ports
//!   the FNV-1a hash and the Box-Muller variate; `Math.log` and `Math.cos` in V8 and Rust's `f64` can differ in
//!   the last places, and Q-O71 accepts 4 ulp on the variate for it.
//! - [`Schedule`]: the event schedule of Q-O68, in the style of [`crate::devices::sources::Clock`] and
//!   [`crate::devices::sources::Pulse`]. Edges, resets, corners, knots and noise buckets are computed from an
//!   integer index, never accumulated, and each event carries the level it sets: the edge's own, never the
//!   voltage function at the edge time (classic reads either side of an edge instant by float rounding, probe P1f).
//!
//! Param layout: `kinds::signal_gen_param`. The compiler calls classic's `parseSignalGenParams` itself, so every
//! clamp below is classic's, repeated for a hand-built program by [`signal_gen_check`].

use std::f64::consts::PI;
use std::fmt::{self, Write as _};

use crate::devices::sources::{Pulse, PulseEventKind, RampSeg};
use crate::kinds::signal_gen_param::*;

/// Classic updates noise at 10 kSa/s (`SIGNAL_GEN_NOISE_UPDATE_RATE_HZ`) and holds each draw for one bucket.
pub const NOISE_BUCKET: f64 = 1.0 / 10_000.0;
/// Classic's buckets begin 1e-12 s early (`waveform.ts:316`), so a boundary reads the new bucket in the guard step
/// the hosts land on it.
pub const NOISE_GUARD: f64 = 1e-12;

/// The largest edge or cycle index the schedules search for: exact as f64 below 2^52, and an instant this far out
/// is no timeline (as `devices::sources`).
const MAX_INDEX: f64 = 4_503_599_627_370_496.0;
/// JS prints an integer-valued number in decimal below this and in exponent form from it on; the hash is over
/// that spelling.
const JS_DECIMAL_LIMIT: f64 = 1e21;
/// The largest level (volts) or PWL time (seconds) a record may carry: no circuit has a source of 1e100 V, and
/// below it every sum, difference and `swing * elapsed / length` the schedule and its `RampSeg`s form stays finite.
const LEVEL_LIMIT: f64 = 1e100;
/// The delay rule: the schedule's times are `(origin + n P) + offset`, strictly ordered only while the spacing of
/// floats at the delay is a tiny fraction of the period; `P * 2^-20` leaves about a million float steps per period
/// (a 1 kHz generator may start up to about 4e6 s in). Beyond it edge times collide and `first_event_after` can
/// skip an edge, so the record is declined and the circuit stays on classic.
const DELAY_ULP_PER_PERIOD: f64 = 1.0 / 1_048_576.0;

fn whole(v: f64) -> bool {
    v.fract() == 0.0
}

/// The params and the PWL blob of a `signal_gen` record. The blob is
/// `[n, t0, v0, ..., t(n-1), v(n-1)]` with `n <= PWL_MAX_POINTS`, times
/// non-decreasing, and `n = 0` unless `waveform` is `pwl`, so a record has one
/// encoding per value.
///
/// Every value is finite (classic lets a NaN through each clamp and plays it, so the compiler declines
/// `non-finite-<param>` and the reader repeats the rule), `|seed| < 1e21` (the hash is over `String(seed)`), and
/// a `pulse` cycle is whole: `(tr + pw) + tf <= 1/frequency` (Q-O72, summed as classic's `tr + pw + tf`).
pub(crate) fn signal_gen_check(_: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    if let Some(i) = params.iter().position(|v| !v.is_finite()) {
        return Err(format!("signal_gen param {i} must be finite, got {}", params[i]));
    }
    let shape = params[WAVEFORM];
    if !(whole(shape) && (0.0..=SHAPE_NOISE as f64).contains(&shape)) {
        return Err(format!("signal_gen waveform must be an integer in [0, {SHAPE_NOISE}], got {shape}"));
    }
    if params[AMPLITUDE] < 0.0 {
        return Err(format!("signal_gen amplitude must be >= 0, got {}", params[AMPLITUDE]));
    }
    if !(params[FREQUENCY] >= 1e-6) {
        return Err(format!("signal_gen frequency must be >= 1e-6, got {}", params[FREQUENCY]));
    }
    if !(0.01..=0.99).contains(&params[DUTY]) {
        return Err(format!("signal_gen duty must be in [0.01, 0.99], got {}", params[DUTY]));
    }
    if params[DELAY] < 0.0 {
        return Err(format!("signal_gen delay must be >= 0, got {}", params[DELAY]));
    }
    for (name, v) in [("tr", params[TR]), ("tf", params[TF]), ("pw", params[PW])] {
        if !(v >= 1e-9) {
            return Err(format!("signal_gen {name} must be >= 1e-9, got {v}"));
        }
    }
    if !whole(params[SEED]) || params[SEED].abs() >= JS_DECIMAL_LIMIT {
        return Err(format!("signal_gen seed must be an integer below 1e21 in magnitude, got {}", params[SEED]));
    }
    if params[ENABLED] != 0.0 && params[ENABLED] != 1.0 {
        return Err(format!("signal_gen enabled must be 0 or 1, got {}", params[ENABLED]));
    }
    if params[OFFSET].abs() > LEVEL_LIMIT {
        return Err(format!("signal_gen offset must be within {LEVEL_LIMIT:e} V, got {}", params[OFFSET]));
    }
    // Every shape but dc and pwl scales the amplitude into the level (noise draws reach about 6.8 sigma, so the
    // factor is 8): offset +- amplitude, and the swing of a line, stay finite.
    if shape != SHAPE_DC as f64 && shape != SHAPE_PWL as f64 && params[OFFSET].abs() + 8.0 * params[AMPLITUDE] > LEVEL_LIMIT {
        return Err(format!(
            "signal_gen offset {} and amplitude {} must keep every level within {LEVEL_LIMIT:e} V",
            params[OFFSET], params[AMPLITUDE]
        ));
    }
    let lattice_period = match shape as u32 {
        SHAPE_SQUARE | SHAPE_TRIANGLE | SHAPE_RAMP | SHAPE_PULSE => Some(1.0 / params[FREQUENCY]),
        SHAPE_NOISE => Some(NOISE_BUCKET),
        _ => None,
    };
    if let Some(period) = lattice_period {
        let delay = params[DELAY];
        let ulp = f64::from_bits(delay.to_bits() + 1) - delay;
        if ulp > period * DELAY_ULP_PER_PERIOD {
            return Err(format!(
                "signal_gen delay {delay} is too late for a {period} s period: the schedule needs ulp(delay) <= period * 2^-20"
            ));
        }
    }
    if shape == SHAPE_PULSE as f64 {
        let period = 1.0 / params[FREQUENCY];
        if period < (params[TR] + params[PW]) + params[TF] {
            return Err(format!(
                "signal_gen pulse cycle is truncated: tr + pw + tf exceeds the period {period}"
            ));
        }
    }

    let n = blob[0];
    if !(whole(n) && (0.0..=PWL_MAX_POINTS as f64).contains(&n)) {
        return Err(format!("signal_gen PWL point count must be an integer in [0, {PWL_MAX_POINTS}], got {n}"));
    }
    let n = n as usize;
    if blob.len() != 1 + 2 * n {
        return Err(format!("signal_gen PWL blob must hold 1 + 2 * {n} values, got {}", blob.len()));
    }
    if n > 0 && params[WAVEFORM] != SHAPE_PWL as f64 {
        return Err(format!("signal_gen PWL points are only for the pwl shape, got {n} with waveform {shape}"));
    }
    if blob.iter().any(|v| !v.is_finite()) {
        return Err("signal_gen PWL values must be finite".to_string());
    }
    if blob[1..].iter().any(|v| v.abs() > LEVEL_LIMIT) {
        return Err(format!("signal_gen PWL times and values must be within {LEVEL_LIMIT:e}"));
    }
    let times = blob[1..].iter().step_by(2);
    if times.clone().zip(times.skip(1)).any(|(a, b)| b < a) {
        return Err("signal_gen PWL times must be non-decreasing".to_string());
    }
    Ok(())
}

// ---- classic's waveform functions, in classic's operation order -------------------------------------------------

/// `signalGenVoltage(p, t)` (`waveform.ts:166-200`). `params` is a validated record and `blob` its PWL table.
pub fn signal_gen_voltage(params: &[f64], blob: &[f64], t: f64) -> f64 {
    // Before the delay the waveform holds its own t' = 0 value (waveform.ts:170).
    let t_eff = if t < params[DELAY] { 0.0 } else { t - params[DELAY] };
    eval(params, blob, t_eff, t)
}

/// The waveform at its own time `t_eff`; `t` is the absolute time PWL reads (`pwlVoltage(p, t)`, which ignores
/// the delay).
fn eval(p: &[f64], blob: &[f64], t_eff: f64, t: f64) -> f64 {
    let (amplitude, offset, frequency, phase_deg) = (p[AMPLITUDE], p[OFFSET], p[FREQUENCY], p[PHASE_DEG]);
    match p[WAVEFORM] as u32 {
        // Offset only: amplitude is intentionally unused (waveform.ts:173-175).
        SHAPE_DC => offset,
        SHAPE_SINE => offset + amplitude * sine_unit(t_eff, frequency, phase_deg),
        SHAPE_SQUARE => offset + amplitude * square_unit(t_eff, frequency, phase_deg, p[DUTY]),
        SHAPE_TRIANGLE => offset + amplitude * triangle_unit(t_eff, frequency, phase_deg),
        SHAPE_RAMP => offset + amplitude * ramp_unit(t_eff, frequency, phase_deg),
        // Phase is not applied to a pulse (SPICE convention).
        SHAPE_PULSE => offset + amplitude * pulse_unit(t_eff, frequency, p[TR], p[TF], p[PW]),
        // Absolute voltages at absolute time: offset, amplitude and delay are ignored.
        SHAPE_PWL => pwl_voltage(blob, offset, t),
        SHAPE_NOISE => offset + amplitude * noise_unit_at(p[SEED], t_eff),
        _ => f64::NAN,
    }
}

fn sine_unit(t: f64, f: f64, phase_deg: f64) -> f64 {
    let phase_rad = (phase_deg * PI) / 180.0;
    (2.0 * PI * f * t + phase_rad).sin()
}

/// `((phaseDeg / 360) * period + period) % period`. A phase below -360 degrees leaves it negative (fmod keeps the
/// dividend's sign), which the callers' `+ period` then `% period` wraps back.
fn phase_offset(phase_deg: f64, period: f64) -> f64 {
    ((phase_deg / 360.0) * period + period) % period
}

fn square_unit(t: f64, f: f64, phase_deg: f64, duty: f64) -> f64 {
    let period = 1.0 / f;
    let offset = phase_offset(phase_deg, period);
    let shifted = ((t + offset) % period + period) % period;
    if shifted < duty * period {
        1.0
    } else {
        -1.0
    }
}

fn triangle_unit(t: f64, f: f64, phase_deg: f64) -> f64 {
    let period = 1.0 / f;
    let offset = phase_offset(phase_deg, period);
    let pos = ((t + offset) % period + period) % period / period;
    if pos < 0.5 {
        -1.0 + 4.0 * pos
    } else {
        3.0 - 4.0 * pos
    }
}

fn ramp_unit(t: f64, f: f64, phase_deg: f64) -> f64 {
    let period = 1.0 / f;
    let offset = phase_offset(phase_deg, period);
    let pos = ((t + offset) % period + period) % period / period;
    -1.0 + 2.0 * pos
}

fn pulse_unit(t: f64, f: f64, tr: f64, tf: f64, pw: f64) -> f64 {
    let period = 1.0 / f;
    let t_mod = ((t % period) + period) % period;
    if t_mod < tr {
        -1.0 + 2.0 * (t_mod / tr)
    } else if t_mod < tr + pw {
        1.0
    } else if t_mod < tr + pw + tf {
        1.0 - 2.0 * ((t_mod - tr - pw) / tf)
    } else {
        -1.0
    }
}

fn pwl_voltage(blob: &[f64], offset: f64, t: f64) -> f64 {
    let n = blob[0] as usize;
    if n == 0 {
        return offset;
    }
    let (pt, pv) = (|i: usize| blob[1 + 2 * i], |i: usize| blob[2 + 2 * i]);
    if t <= pt(0) {
        return pv(0);
    }
    if t >= pt(n - 1) {
        return pv(n - 1);
    }
    for i in 1..n {
        if t <= pt(i) {
            let frac = (t - pt(i - 1)) / (pt(i) - pt(i - 1));
            return pv(i - 1) + frac * (pv(i) - pv(i - 1));
        }
    }
    pv(n - 1)
}

// ---- noise ---------------------------------------------------------------------------------------------------------

/// 32-bit FNV-1a over the bytes written to it. The strings hashed are ASCII (digits, `-`, `:` and a tag), where
/// UTF-16 code units and bytes are the same numbers.
struct Fnv(u32);

impl fmt::Write for Fnv {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            self.0 ^= b as u32;
            self.0 = self.0.wrapping_mul(0x0100_0193);
        }
        Ok(())
    }
}

/// `String(v)` for an integer-valued number below 1e21: Rust's `{}` prints the same decimal digits, except that
/// it spells negative zero "-0" where JS says "0" (`Math.round(-0.3)` is -0 in classic's seed parse).
fn write_js_integer(h: &mut Fnv, v: f64) {
    if v == 0.0 {
        let _ = h.write_str("0");
    } else {
        let _ = write!(h, "{v}");
    }
}

/// `hashToOpenUnit("<seed>:<bucket>:<tag>")` (`waveform.ts:65-73`): the FNV-1a hash, the integer finalizer, and a
/// map to the open interval (0, 1).
fn hash_to_open_unit(seed: f64, bucket: f64, tag: &str) -> f64 {
    let mut h = Fnv(0x811c_9dc5);
    write_js_integer(&mut h, seed);
    let _ = h.write_str(":");
    write_js_integer(&mut h, bucket);
    let _ = h.write_str(":");
    let _ = h.write_str(tag);
    let mut h = h.0;
    h ^= h >> 16;
    h = h.wrapping_mul(0x7feb_352d);
    h ^= h >> 15;
    h = h.wrapping_mul(0x846c_a68b);
    h ^= h >> 16;
    (h as f64 + 0.5) / 4_294_967_296.0
}

/// The unit-variance Gaussian draw of one noise bucket (`noiseUnit`, `waveform.ts:312-318`): Box-Muller over two
/// hashed uniforms. Deterministic per `(seed, bucket)`. A bucket JS would print in exponent form (1e21 or more,
/// an instant past 1e17 s) or that is not a number returns NaN rather than a draw classic would not make.
pub fn noise_unit(seed: f64, bucket: f64) -> f64 {
    if !(bucket.abs() < JS_DECIMAL_LIMIT) {
        return f64::NAN;
    }
    let u1 = hash_to_open_unit(seed, bucket, "radius");
    let u2 = hash_to_open_unit(seed, bucket, "angle");
    (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
}

/// `noiseUnit(seed, t')`: the bucket is `floor((t' + 1e-12) / 1e-4)`.
fn noise_unit_at(seed: f64, t_eff: f64) -> f64 {
    noise_unit(seed, ((t_eff + NOISE_GUARD) / NOISE_BUCKET).floor())
}

// ---- the schedule (Q-O68) --------------------------------------------------------------------------------------------

/// How the node moves between events, the split of Q-O69.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeClass {
    /// Piecewise constant: the level changes only at events (`dc`, `square`, `noise`, and a `pulse` whose ramps
    /// are all midpoint steps, W1f Q5). The partition stays static and re-solves at each event.
    Constant,
    /// Piecewise linear: between events the value follows a straight line (`triangle`, `ramp`, `pwl` with two
    /// points or more, and a `pulse` with a resolved ramp). Each event's [`SgEvent::run`] is the line to follow.
    Linear,
    /// Nonlinear in `t`: a sine, evaluated at each stamp's own instant. Its one event is the delay boundary.
    Sine,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventKind {
    /// The delay boundary of a continuous shape: the hold ends and the waveform begins.
    Start,
    /// A jump of the level: a square edge, a ramp reset, a pulse midpoint step.
    Edge,
    /// A change of slope at a continuous level: a triangle trough or peak, a pulse ramp's start or end.
    Corner,
    /// A noise bucket boundary.
    Bucket,
    /// A PWL knot.
    Knot,
    /// A resolved pulse ramp's midpoint. `signal_gen` publishes no digital slot, so nothing needs it; it keeps
    /// the index space of `Pulse` and may be ignored.
    Mid,
}

/// One event of the schedule.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SgEvent {
    pub time: f64,
    pub kind: EventKind,
    /// The voltage right after the event: the edge's own level (Q-O68), never the voltage function at `time`.
    pub level: f64,
    /// The straight line the value follows from this event to the next, when there is one. Its `value` is not
    /// classic's expression operand for operand (a few ulps of the swing apart); classic's own is
    /// [`Schedule::voltage`].
    pub run: Option<RampSeg>,
}

/// The source's state at a time: the value, and the line it is on if any.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SgLevel {
    pub value: f64,
    pub run: Option<RampSeg>,
}

/// One `signal_gen` record's event schedule. Event `k` is computed from `k`, never from event `k - 1`, so its
/// time is the same however the kernel reached it. Times never fall with `k`.
///
/// Per shape (`P = 1/frequency`, `origin = delay - phaseOffset`, the headless host's own, `waveform.ts:213`):
/// - `dc`: no events.
/// - `sine`: event 0 at `delay` (`Start`).
/// - `square`: rises at `origin + n P`, falls at `min((origin + n P) + duty P, origin + (n + 1) P)`, those with
///   time `> delay` (an edge at or before it is already the hold level). `n` may start at -1: a phase below -360
///   degrees puts the lattice ahead of the delay. Where an edge rounds onto the delay while classic's t' = 0
///   reading is still the pre-edge side, event 0 is a `Start` at `delay` carrying classic's reading after it
///   ([`Schedule::find_start`]).
/// - `triangle`: event 0 at `delay` (`Start`), then troughs at `origin + n P` and peaks half a period later
///   (never past the next trough), after the delay.
/// - `ramp`: event 0 at `delay` (`Start`, from the hold, or from the dropped reset's line where classic reads a reset), then the resets at `origin + n P`, after the delay.
/// - `pulse`: [`Pulse`]'s schedule with `v1 = offset - amplitude`, `v2 = offset + amplitude`, `td = delay`,
///   `per = P`: midpoint steps for a ramp under W1f Q5's threshold, start/mid/end for a resolved one.
/// - `pwl`: one event per knot at its absolute time (the delay is ignored, Q-O73); two knots at one time are two
///   events in authored order, and the second's level is the one after the step.
/// - `noise`: event `k` is bucket `k + 1`'s boundary, `(delay + (k + 1) * 1e-4) - 1e-12`; bucket 0 holds from
///   the start.
pub struct Schedule<'a> {
    params: &'a [f64],
    blob: &'a [f64],
    shape: u32,
    amplitude: f64,
    offset: f64,
    period: f64,
    duty: f64,
    delay: f64,
    /// `delay - phaseOffset`, for the shapes on a phase lattice.
    origin: f64,
    /// The lattice index of the first event after the delay (square: `2n + h`; triangle: the same; ramp: `n`).
    first_lattice: i64,
    /// The event at `delay` that ends the hold, for the shapes that have one (see [`Schedule::find_start`]).
    start: Option<SgEvent>,
    pulse: Option<Pulse>,
}

impl<'a> Schedule<'a> {
    /// Reads a validated record.
    pub fn new(params: &'a [f64], blob: &'a [f64]) -> Self {
        let shape = params[WAVEFORM] as u32;
        let period = 1.0 / params[FREQUENCY];
        let delay = params[DELAY];
        let origin = delay - phase_offset(params[PHASE_DEG], period);
        let (offset, amplitude) = (params[OFFSET], params[AMPLITUDE]);
        let pulse = (shape == SHAPE_PULSE).then(|| {
            Pulse::from_params(&[
                offset + amplitude * -1.0,
                offset + amplitude * 1.0,
                delay,
                params[TR],
                params[TF],
                params[PW],
                period,
            ])
        });
        let mut sched = Schedule {
            params,
            blob,
            shape,
            amplitude,
            offset,
            period,
            duty: params[DUTY],
            delay,
            origin,
            first_lattice: 0,
            start: None,
            pulse,
        };
        sched.first_lattice = sched.find_first_lattice();
        sched.start = sched.find_start();
        sched
    }

    pub fn class(&self) -> ShapeClass {
        match self.shape {
            SHAPE_SINE => ShapeClass::Sine,
            SHAPE_TRIANGLE | SHAPE_RAMP => ShapeClass::Linear,
            SHAPE_PWL if self.knots() >= 2 => ShapeClass::Linear,
            SHAPE_PULSE => {
                let pulse = self.pulse.as_ref().expect("a pulse record has its pulse");
                if pulse.events_per_cycle() > 0 && pulse.has_resolved_ramp() {
                    ShapeClass::Linear
                } else {
                    ShapeClass::Constant
                }
            }
            _ => ShapeClass::Constant,
        }
    }

    /// Classic's `signalGenVoltage(t)`, to the bit for every shape but noise (Q-O71).
    pub fn voltage(&self, t: f64) -> f64 {
        signal_gen_voltage(self.params, self.blob, t)
    }

    /// The level before any event: the waveform's own value at t' = 0 (and, for a table, its first value).
    fn hold(&self) -> f64 {
        eval(self.params, self.blob, 0.0, f64::NEG_INFINITY)
    }

    fn knots(&self) -> usize {
        self.blob[0] as usize
    }

    fn knot(&self, i: usize) -> (f64, f64) {
        (self.blob[1 + 2 * i], self.blob[2 + 2 * i])
    }

    fn high(&self) -> f64 {
        self.offset + self.amplitude * 1.0
    }

    fn low(&self) -> f64 {
        self.offset + self.amplitude * -1.0
    }

    // -- the phase lattice (square, triangle, ramp) --

    fn two_per_cycle(&self) -> bool {
        self.shape == SHAPE_SQUARE || self.shape == SHAPE_TRIANGLE
    }

    /// The time of lattice index `j`. Square and triangle: `j = 2n + h`, `h = 0` the rise (trough) and `h = 1`
    /// the fall (peak); ramp: `j = n`. Each is computed from `j`, and the second half of a cycle is clamped to
    /// the next cycle's start so times never fall (a `duty` within an ulp of 1 would otherwise put a fall after
    /// its successor, the clock's own case).
    fn lattice_time(&self, j: i64) -> f64 {
        if !self.two_per_cycle() {
            return self.origin + j as f64 * self.period;
        }
        let n = j.div_euclid(2);
        let base = self.origin + n as f64 * self.period;
        if j.rem_euclid(2) == 0 {
            return base;
        }
        let next = self.origin + (n + 1) as f64 * self.period;
        let inner = if self.shape == SHAPE_SQUARE { self.duty * self.period } else { self.period / 2.0 };
        (base + inner).min(next)
    }

    /// The level after lattice index `j`: a rise or trough sets the low level for a triangle and ramp and the
    /// high one for a square; a fall or peak the other.
    fn lattice_level(&self, j: i64) -> f64 {
        let first_half = !self.two_per_cycle() || j.rem_euclid(2) == 0;
        match (self.shape, first_half) {
            (SHAPE_SQUARE, true) => self.high(),
            (SHAPE_SQUARE, false) => self.low(),
            (_, true) => self.low(),
            (_, false) => self.high(),
        }
    }

    /// The first lattice index whose time is strictly after the delay: an event at or before it equals the hold
    /// level already.
    fn find_first_lattice(&self) -> i64 {
        if !matches!(self.shape, SHAPE_SQUARE | SHAPE_TRIANGLE | SHAPE_RAMP) {
            return 0;
        }
        let mut j = if self.two_per_cycle() { -2 } else { -1 };
        for _ in 0..16 {
            if self.lattice_time(j) > self.delay {
                break;
            }
            j += 1;
        }
        j
    }

    /// Whether event 0 is the `Start` at `delay`. Always for the continuous shapes, whose slope or nonlinearity
    /// begins there; for a square or a ramp only where the lattice and classic's t' = 0 reading disagree (below).
    fn has_start(&self) -> bool {
        self.start.is_some()
    }

    /// Lattice event `j` (see [`Schedule::lattice_time`]) with the level it sets and the line that follows.
    fn lattice_event_at(&self, j: i64) -> SgEvent {
        let time = self.lattice_time(j);
        let level = self.lattice_level(j);
        let (kind, run) = match self.shape {
            SHAPE_SQUARE => (EventKind::Edge, None),
            SHAPE_RAMP => {
                let len = self.lattice_time(j + 1) - time;
                (EventKind::Edge, Some(RampSeg { t0: time, len, v_from: level, v_to: self.high() }))
            }
            _ => {
                let len = self.lattice_time(j + 1) - time;
                let v_to = if j.rem_euclid(2) == 0 { self.high() } else { self.low() };
                (EventKind::Corner, Some(RampSeg { t0: time, len, v_from: level, v_to }))
            }
        };
        SgEvent { time, kind, level, run }
    }

    fn lattice_event(&self, k: u64) -> Option<SgEvent> {
        let start = u64::from(self.has_start());
        if k < start || (k - start) as f64 >= MAX_INDEX {
            return None;
        }
        Some(self.lattice_event_at(self.first_lattice + (k - start) as i64))
    }

    /// The event at `delay` that ends the hold, if the shape has one.
    ///
    /// The hold is classic's reading at t' = 0; the state after the delay is classic's own reading in the stretch
    /// that follows, up to the first scheduled event. They differ where an edge's computed time
    /// `(delay - phaseOffset) + n P` rounds onto or below `delay` while classic's `%` wrap still reads the pre-edge
    /// side at t' = 0 (50 Hz, phase 720, delay 0.5: classic is HIGH on [0.5, 0.51), the hold LOW), which
    /// `find_first_lattice` would drop as already in the hold level. The reading is taken at the middle of that
    /// stretch, `after`: a reading one float past the delay is not enough, because `t' + phaseOffset` can absorb
    /// it (3 Hz, phase 90, duty 0.25, delay 0.125 reads the hold there and the real level a few periods of
    /// float spacing later). A stretch with no float between its ends has no reading, and the hold stands.
    /// - `square`: a Start carrying `after`, when it is not the hold level;
    /// - `ramp`: always a Start with the line up to the first reset. It starts at the hold, or at the last
    ///   reset's line where `after` reads below the hold: a rising sawtooth only falls through a reset;
    /// - `triangle`: always, from the hold value to the first corner (a value is continuous through a corner, so
    ///   the hold is right to within the rounding either way);
    /// - `sine`: always, level the hold, no line.
    fn find_start(&self) -> Option<SgEvent> {
        let hold = self.hold();
        let first = self.first_lattice;
        let start = |level: f64, run: Option<RampSeg>| Some(SgEvent { time: self.delay, kind: EventKind::Start, level, run });
        let next = self.lattice_time(first);
        let mid = self.delay + (next - self.delay) / 2.0;
        let after = if mid > self.delay && mid < next { self.voltage(mid) } else { hold };
        match self.shape {
            SHAPE_SINE => start(hold, None),
            SHAPE_SQUARE => (after.to_bits() != hold.to_bits()).then(|| SgEvent { time: self.delay, kind: EventKind::Start, level: after, run: None }),
            SHAPE_TRIANGLE => {
                let len = next - self.delay;
                start(hold, Some(RampSeg { t0: self.delay, len, v_from: hold, v_to: self.lattice_level(first) }))
            }
            SHAPE_RAMP => {
                let reset = after < hold;
                let level = match self.lattice_event_at(first - 1).run {
                    Some(run) if reset => run.value(self.delay),
                    _ => hold,
                };
                start(level, Some(RampSeg { t0: self.delay, len: next - self.delay, v_from: level, v_to: self.high() }))
            }
            _ => None,
        }
    }

    // -- the other shapes --

    fn pulse_event(&self, k: u64) -> Option<SgEvent> {
        let e = self.pulse.as_ref()?.event(k)?;
        let (kind, run) = match e.kind {
            PulseEventKind::Step => (EventKind::Edge, None),
            PulseEventKind::RampStart => (EventKind::Corner, e.ramp),
            PulseEventKind::RampMid => (EventKind::Mid, e.ramp),
            // The ramp is over: flat until the next event.
            PulseEventKind::RampEnd => (EventKind::Corner, None),
        };
        Some(SgEvent { time: e.time, kind, level: e.level, run })
    }

    fn pwl_event(&self, k: u64) -> Option<SgEvent> {
        let i = usize::try_from(k).ok().filter(|&i| i < self.knots())?;
        let (t, v) = self.knot(i);
        let run = (i + 1 < self.knots())
            .then(|| self.knot(i + 1))
            .filter(|&(next_t, _)| next_t > t)
            .map(|(next_t, next_v)| RampSeg { t0: t, len: next_t - t, v_from: v, v_to: next_v });
        Some(SgEvent { time: t, kind: EventKind::Knot, level: v, run })
    }

    fn noise_event(&self, k: u64) -> Option<SgEvent> {
        if k as f64 >= MAX_INDEX {
            return None;
        }
        let bucket = (k + 1) as f64;
        let time = (self.delay + bucket * NOISE_BUCKET) - NOISE_GUARD;
        let level = self.offset + self.amplitude * noise_unit(self.params[SEED], bucket);
        Some(SgEvent { time, kind: EventKind::Bucket, level, run: None })
    }

    /// Event `k`, or `None` when there is no such event.
    pub fn event(&self, k: u64) -> Option<SgEvent> {
        match self.shape {
            SHAPE_SINE => if k == 0 { self.start } else { None },
            SHAPE_SQUARE | SHAPE_TRIANGLE | SHAPE_RAMP => {
                if k == 0 && self.start.is_some() {
                    self.start
                } else {
                    self.lattice_event(k)
                }
            }
            SHAPE_PULSE => self.pulse_event(k),
            SHAPE_PWL => self.pwl_event(k),
            SHAPE_NOISE => self.noise_event(k),
            _ => None,
        }
    }

    /// The first index in `from..from + span` whose event time is `> t0`. The event times decide, not the
    /// arithmetic that estimates where to look.
    fn scan_after(&self, from: u64, span: u64, t0: f64) -> Option<u64> {
        (from..from + span).find(|&k| self.event(k).is_some_and(|e| e.time > t0))
    }

    /// The index of the first event with time `> t0` (`START_TIME` at load), or `None` when there is none (no
    /// events at all, the last event passed, or an index past `MAX_INDEX`).
    pub fn first_event_after(&self, t0: f64) -> Option<u64> {
        match self.shape {
            SHAPE_SINE => (self.delay > t0).then_some(0),
            SHAPE_PULSE => self.pulse.as_ref()?.first_event_after(t0),
            SHAPE_PWL => (0..self.knots() as u64).find(|&k| self.pwl_event(k).is_some_and(|e| e.time > t0)),
            SHAPE_NOISE => {
                let estimate = if t0 > self.delay { ((t0 - self.delay) / NOISE_BUCKET).floor() } else { 0.0 };
                if !(estimate < MAX_INDEX) {
                    return None;
                }
                // Event k is bucket k + 1's boundary: one cycle of margin either way.
                self.scan_after((estimate as u64).saturating_sub(2), 6, t0)
            }
            SHAPE_SQUARE | SHAPE_TRIANGLE | SHAPE_RAMP => {
                if self.has_start() && self.delay > t0 {
                    return Some(0);
                }
                let cycle = if t0 > self.origin { ((t0 - self.origin) / self.period).floor() } else { 0.0 };
                if !(cycle < MAX_INDEX) {
                    return None;
                }
                let per_cycle = if self.two_per_cycle() { 2 } else { 1 };
                let j = (cycle as i64 - 2) * per_cycle;
                let start = u64::from(self.has_start());
                let k = (j - self.first_lattice).max(0) as u64 + start;
                self.scan_after(k, 6 * per_cycle as u64, t0)
            }
            _ => None,
        }
    }

    /// The level at load and the index of the first event still to come (README Sources point 4): the level after
    /// the last event whose time, the same expression as the schedule's, is `<= t0`, and the hold level when
    /// there is none. So every event is either in the load level or scheduled, never both and never neither.
    /// Inside a linear run the value is classic's own `signalGenVoltage(t0)` and `run` the line it is on; a sine's
    /// is always classic's.
    ///
    /// Where no event can be named (a periodic shape past `MAX_INDEX` cycles, where event times no longer
    /// resolve) the level is classic's `signalGenVoltage(t0)` and nothing is scheduled.
    pub fn load_state(&self, t0: f64) -> (SgLevel, Option<u64>) {
        let next = self.first_event_after(t0);
        let by_voltage = |next| (SgLevel { value: self.voltage(t0), run: None }, next);
        let applied = match (self.shape, next) {
            (SHAPE_SINE, next) => return by_voltage(next),
            (_, Some(k)) => k,
            (SHAPE_PWL, None) => self.knots() as u64,
            (SHAPE_DC, None) => 0,
            (SHAPE_PULSE, None) if self.pulse.as_ref().is_some_and(|p| p.events_per_cycle() == 0) => 0,
            (_, None) => return by_voltage(None),
        };
        let level = match applied.checked_sub(1).and_then(|last| self.event(last)) {
            None => SgLevel { value: self.hold(), run: None },
            Some(e) => match e.run {
                Some(run) => {
                    let value = if t0 <= e.time { e.level } else { self.voltage(t0) };
                    SgLevel { value, run: Some(run) }
                }
                None => SgLevel { value: e.level, run: None },
            },
        };
        (level, next)
    }
}

#[cfg(test)]
#[path = "waveform_golden.rs"]
mod waveform_golden;

#[cfg(test)]
#[path = "waveform_tests.rs"]
mod tests;

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for ShapeClass {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            ShapeClass::Constant => 0,
            ShapeClass::Linear => 0,
            ShapeClass::Sine => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for EventKind {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            EventKind::Start => 0,
            EventKind::Edge => 0,
            EventKind::Corner => 0,
            EventKind::Bucket => 0,
            EventKind::Knot => 0,
            EventKind::Mid => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for SgEvent {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.time, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.level, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.run, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SgLevel {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.run, w); }
        n
    }
}
