//! Bench PSU regimes and waveform edge schedules (core/README.md, "Kernel
//! semantics: rails and sources (W1f)", Rails point 5 and Sources points
//! 2-3), as pure functions. No solver code lives here: the rail and the
//! kernel call these, keep the state (`reg`, an edge counter) and schedule
//! the events.
//!
//! Every edge time is computed from an integer index, never accumulated: a
//! clock edge is `delay + n*P` and a pulse event is `td + c*per + offset`.
//! Adding a period to the previous edge would drift by an ulp per edge, and
//! the computed instants are the ones the kernel's events land on.

/// The longest pulse ramp, relative to the source's event timescale, that is
/// a midpoint step (Sources point 3); a longer one is resolved.
pub const RAMP_STEP_RATIO: f64 = 0.01;

/// The largest edge or cycle index the schedules search for: `n` and `c` are
/// exact as f64 below 2^52, and an instant this far out is no timeline.
const MAX_INDEX: f64 = 4_503_599_627_370_496.0;

// ---- bench PSU regimes (Rails point 5) -----------------------------------

/// A current-limited bench PSU's regime. `reg` reads 0 (CV) or 2 (CC).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Regime {
    Cv,
    Cc,
}

impl Regime {
    /// The device-state value: 0 for CV, 2 for CC (classic's `icState.reg`).
    pub fn code(self) -> f64 {
        match self {
            Regime::Cv => 0.0,
            Regime::Cc => 2.0,
        }
    }

    pub fn from_code(code: f64) -> Option<Regime> {
        if code == 0.0 {
            Some(Regime::Cv)
        } else if code == 2.0 {
            Some(Regime::Cc)
        } else {
            None
        }
    }
}

/// `+1`, or `-1` for a negative setpoint: the sign the output and the
/// delivered current follow (SO:228).
pub fn direction(v: f64) -> f64 {
    if v < 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// The compliance clamp of fixed classic (0.6.2, `fix/psu-railcap-cc-entry`,
/// README Rails point 5): a CC candidate whose settled rail has reached the
/// setpoint itself, `dir * u_cc >= |V|`, is no current limit at all, so the
/// source stays CV. There is no tolerance band below `|V|`: a band there held
/// an overloaded supply with a rail capacitor in CV past its limit (probes
/// P10, P11). Non-finite values never clamp.
pub fn compliance_keeps_cv(u_cc: f64, v: f64) -> bool {
    if !u_cc.is_finite() || !v.is_finite() {
        return false;
    }
    direction(v) * u_cc >= v.abs()
}

/// One regime transition at a rail solve, for a PSU with no rail capacitor.
/// At most one transition per call, as the rules run at most once per solve.
/// `i_at_v` is the draw at the setpoint (`A0 + G V`, Rails point 2).
///
/// The two voltages are different quantities, and each rule reads only its
/// own:
/// - `u_cc` is the rail voltage of the CC solution. The CV -> CC clamp tests
///   it. In CV the present rail is `V` itself, so the caller computes the CC
///   candidate; handing in the CV rail instead would make the clamp keep CV
///   forever.
/// - `u_rail` is the present rail. The CC -> CV exit tests it.
///
/// CV enters CC when `|i_at_v| > i_limit`, strictly (SO:319), unless the CC
/// solution is already at the setpoint ([`compliance_keeps_cv`] on `u_cc`).
/// CC exits to CV when the CC solution reaches the setpoint, which for a
/// linear rail is the draw at the setpoint back within the limit, `dir *
/// i_at_v <= i_limit` (fixed classic, `fix/psu-cc-exit`: its commit leaves CC
/// exactly when the solve's own compliance clamp selected CV, so there is one
/// threshold, not two). In CC the present rail `u_rail` is the CC solution,
/// so it exits too once `u_rail` has reached the setpoint itself, the clamp
/// the entry reads: the draw test is that clamp only while the draw rises
/// with the rail, and an IC that powers off past its window does not (FXH,
/// W1gV4's bug A: a 555 whose DIS loads the rail at the setpoint, solved in
/// CC from below its window, held 2.1e11 V, iLimit over the node shunt, in
/// CC). A voltage fraction of the setpoint would re-enter at the exit
/// instant forever (probe P7); the whole setpoint does not, since the entry
/// clamp then keeps CV on the same test.
///
/// An `i_limit` that is not `> 0` is an ideal source: it has no regime.
pub fn regime_step(reg: Regime, i_at_v: f64, u_cc: f64, u_rail: f64, v: f64, i_limit: f64) -> Regime {
    if !(i_limit > 0.0) {
        return Regime::Cv;
    }
    match reg {
        Regime::Cv => {
            if i_at_v.abs() > i_limit && !compliance_keeps_cv(u_cc, v) {
                Regime::Cc
            } else {
                Regime::Cv
            }
        }
        Regime::Cc => {
            if direction(v) * i_at_v <= i_limit || compliance_keeps_cv(u_rail, v) {
                Regime::Cv
            } else {
                Regime::Cc
            }
        }
    }
}

/// [`regime_step`] for a PSU with a rail capacitor, with the same arguments.
/// Entering CC is the same (`u_cc` against the clamp). The exit is different:
/// a jump to `V` would draw without bound into the capacitor, so CC exits
/// only once the present rail has reached the setpoint, `dir * u_rail >=
/// |V|` (probe P9: classic stays in CC, -0.1 A, up to 5.0 V), and the draw
/// at the setpoint is read only to keep CC where it is past the limit (ruling
/// R68 after G: CC just entered at the setpoint).
pub fn regime_step_rail_cap(reg: Regime, i_at_v: f64, u_cc: f64, u_rail: f64, v: f64, i_limit: f64) -> Regime {
    if !(i_limit > 0.0) {
        return Regime::Cv;
    }
    match reg {
        Regime::Cv => regime_step(reg, i_at_v, u_cc, u_rail, v, i_limit),
        Regime::Cc => {
            // A rail at the setpoint whose draw there is past the limit is
            // one CC has just entered at it (the capacitors charged to `V`),
            // which the draws pull down at once: CV there would deliver past
            // the limit (ruling R68 after G; a chip powered only inside its
            // window on 10 uF, entered at its retry, flipped back to CV at
            // 72 mA from a 50 mA supply). A rail that reached the setpoint
            // climbing had its draw within the limit, and leaves.
            if u_rail.is_finite() && v.is_finite() && direction(v) * u_rail >= v.abs() && !(direction(v) * i_at_v > i_limit) {
                Regime::Cv
            } else {
                Regime::Cc
            }
        }
    }
}

// ---- classic's waveform functions (WF:12-57), verbatim -----------------------

/// `clockVoltage`. The core never samples it between edges; it schedules
/// them, and uses this for the level at load and to check the schedule.
/// `%` is fmod, as JS's.
pub fn clock_voltage(t: f64, period: f64, duty: f64, v_high: f64, v_low: f64, delay: f64) -> f64 {
    if t < delay {
        return v_low;
    }
    let phase = ((t - delay) % period) / period;
    if phase < duty {
        v_high
    } else {
        v_low
    }
}

/// `pulseVoltage`: SPICE PULSE, piecewise linear, one-shot unless `per` is
/// finite and positive.
#[allow(clippy::too_many_arguments)]
pub fn pulse_voltage(t: f64, v1: f64, v2: f64, td: f64, tr: f64, tf: f64, pw: f64, per: f64) -> f64 {
    if t < td {
        return v1;
    }
    let elapsed = t - td;
    let t_cyc = if per.is_finite() && per > 0.0 { elapsed % per } else { elapsed };
    if t_cyc < tr {
        return v1 + ((v2 - v1) * t_cyc) / tr;
    }
    if t_cyc < tr + pw {
        return v2;
    }
    let fall = tr + pw;
    if t_cyc < fall + tf {
        return v2 - ((v2 - v1) * (t_cyc - fall)) / tf;
    }
    v1
}

/// The digital slot of a level against the midpoint of the two levels:
/// strictly greater is 1 (SE:6195, 6209).
fn slot_above(level: f64, a: f64, b: f64) -> f64 {
    if level > (a + b) / 2.0 {
        1.0
    } else {
        0.0
    }
}

// ---- clock edges (Sources point 2) ---------------------------------------------

/// One clock edge: the index counts rising and falling edges in time order,
/// `k = 2n` the rising edge of cycle `n` and `k = 2n + 1` its falling edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClockEdge {
    pub time: f64,
    pub rising: bool,
    /// The level after the edge: `vHigh` after a rising edge, `vLow` after a
    /// falling one, never `clockVoltage` of the edge time (fmod can put an
    /// edge instant on the wrong side, probe P8).
    pub level: f64,
    /// The digital slot after the edge.
    pub slot: f64,
}

/// A clock record (codes 22, 23; `[freq, duty, vHigh, vLow, delay]`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Clock {
    period: f64,
    duty: f64,
    v_high: f64,
    v_low: f64,
    delay: f64,
}

impl Clock {
    /// Reads a validated record (`freq >= 1e-6`, so the period is finite). The
    /// period is `1 / freq`, as classic passes it to `clockVoltage` (SE:6194).
    pub fn from_params(params: &[f64]) -> Self {
        Clock { period: 1.0 / params[0], duty: params[1], v_high: params[2], v_low: params[3], delay: params[4] }
    }

    pub fn period(&self) -> f64 {
        self.period
    }

    /// `clockVoltage(t)`, classic's own level, for checking the schedule
    /// against. Between edges it equals the edge-driven level; it differs
    /// only on an edge instant itself, which is why the load level comes from
    /// [`Clock::load_state`] and not from this.
    pub fn level_at(&self, t: f64) -> f64 {
        clock_voltage(t, self.period, self.duty, self.v_high, self.v_low, self.delay)
    }

    /// The digital slot of `level`: 1 when it exceeds `(vHigh + vLow) / 2`.
    pub fn slot_of(&self, level: f64) -> f64 {
        slot_above(level, self.v_high, self.v_low)
    }

    /// Edge `k`, or `None` when there is no such edge.
    /// - `duty >= 1`: `vHigh` from `delay` on, so one rising edge at `delay`
    ///   (`k = 0`);
    /// - `duty <= 0`: `vLow` always, no edge;
    /// - otherwise rising at `delay + n*P` and falling at `min(delay + n*P +
    ///   duty*P, delay + (n + 1)*P)`. `n` starts at 0: an edge before `delay`
    ///   is not one (`vLow` before `delay`), and a negative `delay` is a
    ///   phase shift.
    ///
    /// The `min` keeps the times from falling with `k`. A `duty` with `(1 -
    /// duty)*P` under an ulp of the time makes `delay + n*P + duty*P` round
    /// past the next rise (1 kHz, `duty` 0.9999999999999999: edge 37 computes
    /// to 0.019000000000000003, after edge 38 at 0.019), and the schedule
    /// would then hand the kernel an edge in the past. Both terms come from
    /// `n`, never from a running sum, so [`Clock::first_edge_after`] and
    /// [`Clock::load_state`] read the same times the kernel schedules.
    pub fn edge(&self, k: u64) -> Option<ClockEdge> {
        if !(self.duty > 0.0) {
            return None;
        }
        if self.duty >= 1.0 {
            return (k == 0).then(|| self.make_edge(self.delay, true));
        }
        let n = k / 2;
        let rise = self.delay + n as f64 * self.period;
        if k % 2 == 0 {
            Some(self.make_edge(rise, true))
        } else {
            let fall = rise + self.duty * self.period;
            let next_rise = self.delay + (n + 1) as f64 * self.period;
            Some(self.make_edge(fall.min(next_rise), false))
        }
    }

    fn make_edge(&self, time: f64, rising: bool) -> ClockEdge {
        let level = if rising { self.v_high } else { self.v_low };
        ClockEdge { time, rising, level, slot: self.slot_of(level) }
    }

    /// The index of the first edge with time `> t0` (`START_TIME` at load), or
    /// `None` when the clock has no such edge (or its index is past
    /// `MAX_INDEX`). The edge times decide, not the arithmetic that estimates
    /// where to look.
    pub fn first_edge_after(&self, t0: f64) -> Option<u64> {
        if !(self.duty > 0.0) {
            return None;
        }
        if self.duty >= 1.0 {
            return (self.delay > t0).then_some(0);
        }
        let estimate = if t0 > self.delay { ((t0 - self.delay) / self.period).floor() } else { 0.0 };
        if !(estimate < MAX_INDEX) {
            return None;
        }
        // One cycle early: the estimate can be one cycle off either way, and
        // the edge times are the arbiter. Too high by one is not harmless: a
        // `t0` an ulp under a rise divides up to the rise's own index, and a
        // `duty` within ulps of 1 puts the previous cycle's fall after `t0`
        // too (1 kHz, `duty` 0.9999999999999999, `t0` 0.018: the estimate is
        // 18, the first edge after is the fall of cycle 17, index 35, not the
        // rise of cycle 18). Starting at the estimate's own cycle would skip it.
        let mut k = 2 * (estimate as u64).saturating_sub(1);
        for _ in 0..8 {
            if self.edge(k)?.time > t0 {
                return Some(k);
            }
            k += 1;
        }
        None
    }

    /// The level at load and the index of the first edge still to come
    /// (Sources points 2 and 4): the level after the last edge whose time, the
    /// same expression as the schedule's ([`Clock::edge`]), is `<= t0`, and
    /// `vLow` when there is none. So every edge is either in the load level
    /// or scheduled, never both and never neither.
    ///
    /// This is `clockVoltage(t0)` except when `t0` is itself an edge instant,
    /// where fmod can read the pre-edge level: a literal `clockVoltage` there
    /// would drop the rising edge, and the clock would stay low for its whole
    /// first high half-period.
    /// - `duty <= 0`: no edge, `vLow`;
    /// - `duty >= 1`: the one rising edge at `delay`, in the load level when
    ///   `delay <= t0` (`vHigh`, nothing to come) and scheduled when `delay >
    ///   t0` (`vLow`, edge 0 next);
    /// - otherwise edges `[0, k)` are applied for `k` the first edge after
    ///   `t0`, so the level is edge `k - 1`'s own, `vLow` for `k = 0`.
    ///
    /// When the cycle index is past `MAX_INDEX` (a `t0` of 2^52 periods or
    /// more, where edge times no longer resolve) no edge is found and none
    /// can be scheduled. The level is then classic's `clockVoltage(t0)`, the
    /// only level that can be named there, and the clock is held, as
    /// [`Pulse::load_state`] holds a pulse past its search range.
    pub fn load_state(&self, t0: f64) -> (SourceLevel, Option<u64>) {
        let at = |value: f64| SourceLevel { value, slot: self.slot_of(value), ramp: None };
        if !(self.duty > 0.0) {
            return (at(self.v_low), None);
        }
        if self.duty >= 1.0 {
            return if self.delay > t0 { (at(self.v_low), Some(0)) } else { (at(self.v_high), None) };
        }
        match self.first_edge_after(t0) {
            Some(0) => (at(self.v_low), Some(0)),
            Some(k) => {
                let last = self.edge(k - 1).expect("a clock with 0 < duty < 1 has every edge");
                (at(last.level), Some(k))
            }
            None => (at(self.level_at(t0)), None),
        }
    }
}

// ---- pulse edges and ramps (Sources point 3) -----------------------------------

/// How one ramp is modelled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RampMode {
    /// A step of the level at the ramp's midpoint.
    Step,
    /// Resolved: corner events at its start and end, one at its midpoint (the
    /// slot), and `pulseVoltage`'s linear value between the corners.
    Resolved,
}

/// The shortest flat segment of a pulse's cycle, `min(pw, per - tr - pw - tf)`
/// when periodic and `pw` when one-shot. The margin is clamped at 0 against
/// a `per` equal to `tr + pw + tf` that rounds a hair under it.
pub fn event_timescale(tr: f64, tf: f64, pw: f64, per: f64) -> f64 {
    if per.is_finite() && per > 0.0 {
        pw.min((per - tr - pw - tf).max(0.0))
    } else {
        pw
    }
}

/// A ramp of length `len` is a midpoint step when `len <= RAMP_STEP_RATIO *
/// t_ev`, and resolved otherwise. A ramp of length 0 is a step at its instant
/// whatever `t_ev`.
pub fn ramp_mode(len: f64, t_ev: f64) -> RampMode {
    if len == 0.0 || len <= RAMP_STEP_RATIO * t_ev {
        RampMode::Step
    } else {
        RampMode::Resolved
    }
}

/// One resolved ramp: a linear run from `v_from` to `v_to` over `len` seconds
/// from `t0`. `value` is `pulseVoltage`'s own expression in ramp-local time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RampSeg {
    pub t0: f64,
    pub len: f64,
    pub v_from: f64,
    pub v_to: f64,
}

impl RampSeg {
    /// The value at `t`: `v_from` up to the start, `v_to` from the end, linear
    /// between. `v_from + ((v_to - v_from) * e) / len` is `pulseVoltage`'s rise
    /// and, negated through the subtraction, its fall, to the bit.
    pub fn value(&self, t: f64) -> f64 {
        let e = t - self.t0;
        if e <= 0.0 {
            self.v_from
        } else if e >= self.len {
            self.v_to
        } else {
            self.v_from + ((self.v_to - self.v_from) * e) / self.len
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PulseEventKind {
    /// A midpoint step: the level changes at once.
    Step,
    /// The corner where a resolved ramp begins.
    RampStart,
    /// A resolved ramp's midpoint, where the digital slot flips.
    RampMid,
    /// The corner where a resolved ramp ends.
    RampEnd,
}

/// One event of a pulse source's schedule.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PulseEvent {
    pub time: f64,
    pub kind: PulseEventKind,
    /// The ramp it belongs to: the rise (`v1` to `v2`) or the fall.
    pub rising: bool,
    /// The boundary value right after the event (a mid event's is the linear
    /// value at its instant).
    pub level: f64,
    /// The digital slot after the event.
    pub slot: f64,
    /// The resolved ramp the event lies on; `None` for a step.
    pub ramp: Option<RampSeg>,
}

/// The source's state after some prefix of its events.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SourceLevel {
    pub value: f64,
    pub slot: f64,
    /// `Some` while a resolved ramp runs: the value follows it until the
    /// ramp's end event.
    pub ramp: Option<RampSeg>,
}

/// A pulse record (codes 24, 25; `[v1, v2, td, tr, tf, pw, per]`), its ramps
/// decided once, at load.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pulse {
    v1: f64,
    v2: f64,
    td: f64,
    tr: f64,
    tf: f64,
    pw: f64,
    /// 0 for a one-shot; the compiler emits a `per` that is not finite and
    /// positive as 0.
    per: f64,
    rise: RampMode,
    fall: RampMode,
}

impl Pulse {
    /// Reads a validated record.
    pub fn from_params(params: &[f64]) -> Self {
        let (v1, v2, td, tr, tf, pw, per) = (params[0], params[1], params[2], params[3], params[4], params[5], params[6]);
        let t_ev = event_timescale(tr, tf, pw, per);
        Pulse { v1, v2, td, tr, tf, pw, per, rise: ramp_mode(tr, t_ev), fall: ramp_mode(tf, t_ev) }
    }

    pub fn rise_mode(&self) -> RampMode {
        self.rise
    }

    pub fn fall_mode(&self) -> RampMode {
        self.fall
    }

    /// True when any ramp is resolved: the partitions reading the node are
    /// stepped, or driven by it if linear closed-form (Sources point 3).
    pub fn has_resolved_ramp(&self) -> bool {
        self.rise == RampMode::Resolved || self.fall == RampMode::Resolved
    }

    pub fn is_one_shot(&self) -> bool {
        !(self.per > 0.0)
    }

    /// `pulseVoltage(t)`.
    pub fn voltage_at(&self, t: f64) -> f64 {
        pulse_voltage(t, self.v1, self.v2, self.td, self.tr, self.tf, self.pw, self.per)
    }

    /// The digital slot of `level`: 1 when it exceeds `(v1 + v2) / 2`.
    pub fn slot_of(&self, level: f64) -> f64 {
        slot_above(level, self.v1, self.v2)
    }

    /// Events per cycle: a step ramp has one, a resolved ramp three (start,
    /// midpoint, end). None when the level never changes: `v1 = v2`, or `tr =
    /// pw = tf = 0`, where classic never leaves `v1`.
    pub fn events_per_cycle(&self) -> u64 {
        if self.v1 == self.v2 || (self.tr == 0.0 && self.pw == 0.0 && self.tf == 0.0) {
            return 0;
        }
        let count = |mode: RampMode| if mode == RampMode::Step { 1 } else { 3 };
        count(self.rise) + count(self.fall)
    }

    /// Event `k` of the schedule, in time order: cycle `c = k / m` (`m` events
    /// per cycle, `c = 0` only for a one-shot), then the rise's events, then
    /// the fall's. Each time is `td + c*per` plus an offset that does not
    /// depend on `c`. Events of one instant (a step at `td`, then a fall's
    /// start corner at `td` when `pw = 0`) keep this order.
    pub fn event(&self, k: u64) -> Option<PulseEvent> {
        let m = self.events_per_cycle();
        if m == 0 {
            return None;
        }
        let (c, j) = (k / m, (k % m) as usize);
        if self.is_one_shot() && c > 0 {
            return None;
        }
        let base = self.td + c as f64 * self.per;
        let mut events = [None; 6];
        let mut n = 0;
        let mut push = |e: PulseEvent| {
            events[n] = Some(e);
            n += 1;
        };
        self.push_ramp(&mut push, base, base + self.tr / 2.0, base + self.tr, true);
        let fall_start = base + (self.tr + self.pw);
        self.push_ramp(&mut push, fall_start, base + (self.tr + self.pw + self.tf / 2.0), base + (self.tr + self.pw + self.tf), false);
        events[j]
    }

    /// One ramp's events, in order. `start`, `mid` and `end` are the ramp's
    /// corner and midpoint times; a step ramp emits its one event at `mid`.
    fn push_ramp(&self, push: &mut impl FnMut(PulseEvent), start: f64, mid: f64, end: f64, rising: bool) {
        let (mode, len, v_from, v_to) = if rising { (self.rise, self.tr, self.v1, self.v2) } else { (self.fall, self.tf, self.v2, self.v1) };
        let slot_to = self.slot_of(v_to);
        match mode {
            RampMode::Step => push(PulseEvent { time: mid, kind: PulseEventKind::Step, rising, level: v_to, slot: slot_to, ramp: None }),
            RampMode::Resolved => {
                let seg = RampSeg { t0: start, len, v_from, v_to };
                push(PulseEvent { time: start, kind: PulseEventKind::RampStart, rising, level: v_from, slot: self.slot_of(v_from), ramp: Some(seg) });
                push(PulseEvent { time: mid, kind: PulseEventKind::RampMid, rising, level: seg.value(mid), slot: slot_to, ramp: Some(seg) });
                push(PulseEvent { time: end, kind: PulseEventKind::RampEnd, rising, level: v_to, slot: slot_to, ramp: Some(seg) });
            }
        }
    }

    /// The index of the first event with time `> t0`, or `None` when there is
    /// none (no events at all, or a one-shot past its last).
    pub fn first_event_after(&self, t0: f64) -> Option<u64> {
        let m = self.events_per_cycle();
        if m == 0 {
            return None;
        }
        let first_cycle = if self.is_one_shot() {
            0
        } else {
            let estimate = if t0 >= self.td { ((t0 - self.td) / self.per).floor() } else { 0.0 };
            if !(estimate < MAX_INDEX) {
                return None;
            }
            (estimate as u64).saturating_sub(2)
        };
        // The first event above `t0` is in the estimated cycle or the next;
        // six cycles from two early is margin, and the event times decide.
        let from = first_cycle * m;
        let to = if self.is_one_shot() { m } else { from + 6 * m };
        (from..to).find(|&k| self.event(k).is_some_and(|e| e.time > t0))
    }

    /// The source's state after events `[0, k)` have happened: the initial
    /// level before any, else that of event `k - 1`.
    pub fn level_before(&self, k: u64) -> SourceLevel {
        match k.checked_sub(1).and_then(|last| self.event(last)) {
            None => SourceLevel { value: self.v1, slot: self.slot_of(self.v1), ramp: None },
            Some(e) => SourceLevel { value: e.level, slot: e.slot, ramp: if matches!(e.kind, PulseEventKind::RampStart | PulseEventKind::RampMid) { e.ramp } else { None } },
        }
    }

    /// The level at load and the index of the first event still to come
    /// (Sources point 4): the level after the last event at or before `t0`,
    /// and inside a resolved ramp `pulseVoltage(t0)`.
    pub fn load_state(&self, t0: f64) -> (SourceLevel, Option<u64>) {
        let next = self.first_event_after(t0);
        let applied = match next {
            Some(k) => k,
            None if self.events_per_cycle() == 0 => 0,
            None if self.is_one_shot() => self.events_per_cycle(),
            // A periodic source past the searched range: no events, and no
            // way to say which one was last.
            None => {
                let value = self.voltage_at(t0);
                return (SourceLevel { value, slot: self.slot_of(value), ramp: None }, None);
            }
        };
        let mut level = self.level_before(applied);
        if level.ramp.is_some() {
            level.value = self.voltage_at(t0);
        }
        (level, next)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ulp_key(v: f64) -> i64 {
        let bits = v.to_bits() as i64;
        if bits < 0 {
            i64::MIN.wrapping_sub(bits)
        } else {
            bits
        }
    }

    fn ulps(a: f64, b: f64) -> u64 {
        (ulp_key(a) as i128 - ulp_key(b) as i128).unsigned_abs() as u64
    }

    fn assert_ulps(got: f64, want: f64, max: u64, label: &str) {
        let d = ulps(got, want);
        assert!(d <= max, "{label}: got {got:e}, want {want:e}, {d} ulp apart (max {max})");
    }

    // ---- regimes ---------------------------------------------------------------

    // Every case is `{V 5, iLimit 0.1}` unless the case says otherwise. A CV
    // case passes the CC candidate as `u_cc` and the CV rail, `V`, as
    // `u_rail`. A CC case passes the present rail as `u_rail` and, as `u_cc`,
    // a value that would flip the answer if the exit read it.
    const V: f64 = 5.0;
    const LIMIT: f64 = 0.1;

    #[test]
    fn cv_enters_cc_only_strictly_above_the_limit() {
        // The CC candidate sits at 1 V, far from the setpoint.
        assert_eq!(regime_step(Regime::Cv, 0.1000001, 1.0, V, V, LIMIT), Regime::Cc);
        assert_eq!(regime_step(Regime::Cv, 0.1, 1.0, V, V, LIMIT), Regime::Cv, "exactly the limit is not over it");
        assert_eq!(regime_step(Regime::Cv, 0.0999999, 1.0, V, V, LIMIT), Regime::Cv);
        assert_eq!(regime_step(Regime::Cv, f64::from_bits(LIMIT.to_bits() + 1), 1.0, V, V, LIMIT), Regime::Cc, "one ulp over enters");
    }

    #[test]
    fn a_negative_setpoint_enters_on_the_magnitude_and_follows_its_own_direction() {
        // {-5 V, 0.1 A} into 10 ohm: the draw is -0.5 A, and the CC rail -1 V.
        assert_eq!(regime_step(Regime::Cv, -0.5, -1.0, -5.0, -5.0, LIMIT), Regime::Cc);
        assert_eq!(regime_step(Regime::Cv, -0.1, -1.0, -5.0, -5.0, LIMIT), Regime::Cv);
        assert_eq!(direction(-5.0), -1.0);
        assert_eq!(direction(5.0), 1.0);
        assert_eq!(direction(0.0), 1.0, "a zero setpoint is positive");
    }

    #[test]
    fn the_compliance_clamp_keeps_cv_only_when_the_cc_candidate_reaches_the_setpoint() {
        // Fixed classic (README Rails point 5): no band below |V|.
        assert!(compliance_keeps_cv(5.0, 5.0), "on the setpoint");
        assert!(compliance_keeps_cv(1e8, 5.0), "a removed load sends the CC candidate far above V");
        assert!(!compliance_keeps_cv(5.0 - 1e-10, 5.0), "the old band would have kept CV here");
        assert!(!compliance_keeps_cv(f64::from_bits(5.0f64.to_bits() - 1), 5.0), "an ulp under is a current limit");
        assert_eq!(regime_step(Regime::Cv, 0.2, 5.0 - 1e-10, V, V, LIMIT), Regime::Cc, "over the limit and below the setpoint enters");
        assert_eq!(regime_step(Regime::Cv, 0.2, 5.0, V, V, LIMIT), Regime::Cv, "a candidate at the setpoint keeps CV");
        // Negative direction: dir * u compares.
        assert!(compliance_keeps_cv(-5.0, -5.0));
        assert!(!compliance_keeps_cv(-5.0 + 1e-10, -5.0));
        assert!(!compliance_keeps_cv(-1.0, -5.0));
        assert!(!compliance_keeps_cv(f64::NAN, 5.0));
        assert!(!compliance_keeps_cv(5.0, f64::NAN));
    }

    #[test]
    fn cc_stays_while_the_cv_current_is_over_the_limit_and_exits_once_it_is_not() {
        // Probe P7's window: the rail is at 4.9 V, past 98% of 5 V, and the
        // CV draw is still over the limit. Classic flickered there; fixed
        // classic and the core stay.
        assert_eq!(regime_step(Regime::Cc, 0.11, V, 4.9, V, LIMIT), Regime::Cc);
        assert_eq!(regime_step(Regime::Cc, 0.1, 1.0, 4.9, V, LIMIT), Regime::Cv, "at the limit it exits");
        assert_eq!(regime_step(Regime::Cc, 0.09, 1.0, 4.9, V, LIMIT), Regime::Cv);
        assert_eq!(regime_step(Regime::Cc, f64::from_bits(LIMIT.to_bits() + 1), 1.0, 4.9, V, LIMIT), Regime::Cc, "an ulp over stays");
    }

    #[test]
    fn cc_exit_reads_the_draw_alone_however_far_the_rail_is_from_the_setpoint() {
        // A slight overload (5 V, 0.1 A, 49.9 ohm) settles at 4.99 V, inside
        // classic's old 98% window: it must hold CC there (README, Rails
        // point 5; the core's draw at the setpoint is 5/49.9 > 0.1).
        let i_at_v = 5.0 / 49.9;
        assert_eq!(regime_step(Regime::Cc, i_at_v, 4.99, 4.99, V, LIMIT), Regime::Cc, "no flicker in the old window");
        // Under the limit it exits whatever the rail reads.
        for u_rail in [0.0, 1.0, 4.9, V, f64::NAN] {
            assert_eq!(regime_step(Regime::Cc, 0.05, V, u_rail, V, LIMIT), Regime::Cv, "u_rail {u_rail}");
        }
    }

    #[test]
    fn cc_exits_once_the_cc_solution_itself_reaches_the_setpoint() {
        // FXH (W1gV4's bug A): the draw at the setpoint over the limit while
        // the CC rail, solved where an IC is unpowered, sits past the
        // setpoint. No band, as at the entry.
        assert_eq!(regime_step(Regime::Cc, 1.13, 1.0, 2.1e11, 11.2858, 0.2095), Regime::Cv, "a rail past the setpoint is no current limit");
        assert_eq!(regime_step(Regime::Cc, 0.2, 1.0, V, V, LIMIT), Regime::Cv, "at the setpoint");
        assert_eq!(regime_step(Regime::Cc, 0.2, 1.0, f64::from_bits(V.to_bits() - 1), V, LIMIT), Regime::Cc, "an ulp short holds CC");
        assert_eq!(regime_step(Regime::Cc, -0.2, -1.0, -5.0, -5.0, LIMIT), Regime::Cv, "negative setpoint, at -5 V");
        assert_eq!(regime_step(Regime::Cc, -0.2, -1.0, -4.9, -5.0, LIMIT), Regime::Cc);
        assert_eq!(regime_step(Regime::Cc, 0.2, 1.0, f64::NAN, V, LIMIT), Regime::Cc, "a non-finite rail never clamps");
    }

    #[test]
    fn cc_exit_with_a_negative_setpoint_compares_in_its_direction() {
        // {-5 V, 0.1 A}: the draw at the setpoint is -0.1 A.
        assert_eq!(regime_step(Regime::Cc, -0.1, -5.0, -4.9, -5.0, LIMIT), Regime::Cv);
        assert_eq!(regime_step(Regime::Cc, -0.11, -5.0, -4.9, -5.0, LIMIT), Regime::Cc, "delivering over the limit, in its own direction");
        assert_eq!(regime_step(Regime::Cc, -0.05, -5.0, -1.0, -5.0, LIMIT), Regime::Cv);
        // A draw of the wrong sign is under the limit whatever its size.
        assert_eq!(regime_step(Regime::Cc, 3.0, -1.0, -4.9, -5.0, LIMIT), Regime::Cv);
    }

    #[test]
    fn a_rail_capacitor_exits_cc_only_at_the_setpoint() {
        // Probe P9: a PSU limiting into a capacitor stays CC up to 5.0 V.
        assert_eq!(regime_step_rail_cap(Regime::Cc, 0.0, V, 4.9, V, LIMIT), Regime::Cc, "98% is not enough, and the draw test is off");
        assert_eq!(regime_step_rail_cap(Regime::Cc, 0.0, V, 5.0 - 5e-6, V, LIMIT), Regime::Cc, "no band below the setpoint");
        assert_eq!(regime_step_rail_cap(Regime::Cc, 0.0, V, f64::from_bits(5.0f64.to_bits() - 1), V, LIMIT), Regime::Cc);
        assert_eq!(regime_step_rail_cap(Regime::Cc, 0.0, 1.0, 5.0, V, LIMIT), Regime::Cv, "exit at u = 5");
        // The draw at the setpoint is read only past the limit (ruling R68
        // after G): CC entered at the setpoint, where the draws pull the rail
        // down at once, does not leave there; a rail that climbed to it had
        // its draw within the limit.
        assert_eq!(regime_step_rail_cap(Regime::Cc, 0.05, 1.0, 5.0, V, LIMIT), Regime::Cv, "a draw within the limit at the setpoint leaves");
        assert_eq!(regime_step_rail_cap(Regime::Cc, 5.0, 1.0, 5.0, V, LIMIT), Regime::Cc, "a draw past the limit at the setpoint stays in CC");
        assert_eq!(regime_step_rail_cap(Regime::Cc, 0.0, 1.0, -5.0, -5.0, LIMIT), Regime::Cv, "negative setpoint, at -5 V");
        assert_eq!(regime_step_rail_cap(Regime::Cc, 0.0, 1.0, -4.9, -5.0, LIMIT), Regime::Cc);
        assert_eq!(regime_step_rail_cap(Regime::Cc, 0.0, 1.0, f64::NAN, V, LIMIT), Regime::Cc);
        assert_eq!(regime_step_rail_cap(Regime::Cv, 0.2, 1.0, V, V, LIMIT), Regime::Cc, "entry is unchanged");
        assert_eq!(regime_step_rail_cap(Regime::Cv, 0.1, 1.0, V, V, LIMIT), Regime::Cv);
    }

    #[test]
    fn each_regime_rule_reads_only_its_own_voltage() {
        // CV -> CC tests the CC solution's rail, u_cc. The CC solution is at
        // 1 V; the CV rail is V. Handing the CV rail in as u_cc is the
        // mistake the contract warns of: the clamp reads a rail already at
        // the setpoint and keeps CV forever, however far over the limit.
        assert_eq!(regime_step(Regime::Cv, 0.2, 1.0, V, V, LIMIT), Regime::Cc, "u_cc = the CC solution enters");
        assert_eq!(regime_step(Regime::Cv, 0.2, V, V, V, LIMIT), Regime::Cv, "u_cc = the CV rail never enters");
        assert_eq!(regime_step_rail_cap(Regime::Cv, 0.2, 1.0, V, V, LIMIT), Regime::Cc);
        assert_eq!(regime_step_rail_cap(Regime::Cv, 0.2, V, V, V, LIMIT), Regime::Cv);
        // The entry never reads u_rail.
        for u_rail in [0.0, 1.0, 4.9, V, f64::NAN] {
            assert_eq!(regime_step(Regime::Cv, 0.2, 1.0, u_rail, V, LIMIT), Regime::Cc, "u_rail {u_rail}");
            assert_eq!(regime_step(Regime::Cv, 0.2, V, u_rail, V, LIMIT), Regime::Cv, "u_rail {u_rail}");
        }

        // CC -> CV never reads u_cc: without a rail capacitor it reads the
        // draw and the present rail (the CC solution there), with one the
        // present rail.
        for u_cc in [-1.0, 0.0, 4.9, V, f64::NAN] {
            assert_eq!(regime_step(Regime::Cc, 0.05, u_cc, 1.0, V, LIMIT), Regime::Cv, "u_cc {u_cc}");
            assert_eq!(regime_step(Regime::Cc, 0.2, u_cc, 4.99, V, LIMIT), Regime::Cc, "u_cc {u_cc}");
            assert_eq!(regime_step_rail_cap(Regime::Cc, 0.0, u_cc, 5.0, V, LIMIT), Regime::Cv, "u_cc {u_cc}");
            assert_eq!(regime_step_rail_cap(Regime::Cc, 0.0, u_cc, 4.9, V, LIMIT), Regime::Cc, "u_cc {u_cc}");
        }
    }

    #[test]
    fn an_ideal_source_has_no_regime() {
        for limit in [0.0, -1.0, f64::NAN] {
            assert_eq!(regime_step(Regime::Cv, 100.0, 0.0, V, V, limit), Regime::Cv, "limit {limit}");
            assert_eq!(regime_step(Regime::Cc, 100.0, 0.0, V, V, limit), Regime::Cv, "limit {limit}");
            assert_eq!(regime_step_rail_cap(Regime::Cc, 100.0, 0.0, V, V, limit), Regime::Cv, "limit {limit}");
        }
    }

    #[test]
    fn regime_codes_are_classics_reg_values() {
        assert_eq!(Regime::Cv.code(), 0.0);
        assert_eq!(Regime::Cc.code(), 2.0);
        assert_eq!(Regime::from_code(0.0), Some(Regime::Cv));
        assert_eq!(Regime::from_code(2.0), Some(Regime::Cc));
        assert_eq!(Regime::from_code(1.0), None);
    }

    // ---- classic's waveform functions ----------------------------------------------

    #[test]
    fn the_waveform_functions_are_classics() {
        // clockVoltage (WF:12-23): vLow before the delay, then vHigh while
        // the phase is strictly below the duty.
        assert_eq!(clock_voltage(0.25e-3, 1e-3, 0.5, 5.0, 0.0, 0.0), 5.0);
        assert_eq!(clock_voltage(0.5e-3, 1e-3, 0.5, 5.0, 0.0, 0.0), 0.0, "phase 0.5 is not < 0.5 (probe P8)");
        assert_eq!(clock_voltage(1e-3, 1e-3, 0.5, 5.0, 0.0, 0.0), 5.0);
        assert_eq!(clock_voltage(0.5e-3, 1e-3, 0.5, 5.0, 1.0, 1e-3), 1.0, "before the delay");
        // pulseVoltage (WF:37-57), probe P8's samples: 0 at 0.2 us is 1 V,
        // 0.6 us is 3 V, 501.6 us is 2 V.
        assert_ulps(pulse_voltage(0.2e-6, 0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3), 1.0, 4, "0.2 us");
        assert_ulps(pulse_voltage(0.6e-6, 0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3), 3.0, 4, "0.6 us");
        // 501.6 us less the 501 us fall corner cancels to 6 digits, so this one
        // is an absolute bound.
        assert!((pulse_voltage(501.6e-6, 0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3) - 2.0).abs() < 1e-9, "501.6 us");
        assert_eq!(pulse_voltage(2e-4, 0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3), 5.0);
        assert_eq!(pulse_voltage(9e-4, 0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3), 0.0);
        // Before td, and a one-shot (per 0) after its pulse.
        assert_eq!(pulse_voltage(0.5e-3, 1.0, 5.0, 1e-3, 1e-6, 1e-6, 5e-4, 1e-3), 1.0);
        assert_eq!(pulse_voltage(9e-3, 0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 0.0), 0.0);
        assert_eq!(pulse_voltage(2e-4, 0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 0.0), 5.0);
    }

    // ---- clock edges -----------------------------------------------------------------

    fn clock(freq: f64, duty: f64, v_high: f64, v_low: f64, delay: f64) -> Clock {
        Clock::from_params(&[freq, duty, v_high, v_low, delay])
    }

    #[test]
    fn a_1_khz_clock_edges_are_n_ms_and_n_ms_plus_half() {
        let c = clock(1000.0, 0.5, 5.0, 0.0, 0.0);
        assert_eq!(c.period(), 1e-3);
        for n in 0..50u64 {
            let rising = c.edge(2 * n).unwrap();
            let falling = c.edge(2 * n + 1).unwrap();
            assert!(rising.rising && !falling.rising);
            assert_ulps(rising.time, n as f64 / 1000.0, 4, &format!("rising {n}"));
            assert_ulps(falling.time, (2 * n + 1) as f64 / 2000.0, 4, &format!("falling {n}"));
            assert_eq!(rising.level, 5.0);
            assert_eq!(falling.level, 0.0);
            assert_eq!((rising.slot, falling.slot), (1.0, 0.0));
        }
        // From START_TIME 0 the rising edge at 0 is already applied at load:
        // the first edge to come is the fall at 0.5 ms.
        assert_eq!(c.first_edge_after(0.0), Some(1));
        assert_eq!(c.level_at(0.0), 5.0, "and the level at load is HIGH");
        let (load, next) = c.load_state(0.0);
        assert_eq!((load.value, load.slot, load.ramp, next), (5.0, 1.0, None, Some(1)));
    }

    #[test]
    fn a_negative_delay_is_a_phase_shift() {
        // delay -0.25 ms: rising at 0.75 ms + n ms, falling at 0.25 ms + n ms.
        let c = clock(1000.0, 0.5, 5.0, 0.0, -2.5e-4);
        assert_eq!(c.level_at(0.0), 5.0, "phase 0.25 at load is HIGH");
        let mut k = c.first_edge_after(0.0).unwrap();
        let mut seen = Vec::new();
        for _ in 0..4 {
            let e = c.edge(k).unwrap();
            seen.push((e.time, e.rising));
            k += 1;
        }
        let want = [(2.5e-4, false), (7.5e-4, true), (12.5e-4, false), (17.5e-4, true)];
        for ((t, rising), (want_t, want_rising)) in seen.into_iter().zip(want) {
            assert_eq!(rising, want_rising);
            assert_ulps(t, want_t, 4, "edge time");
        }
        // The rising edges are 7.5e-4 + n * 1e-3 for every later n too.
        for n in 1..40u64 {
            let e = c.edge(2 * (n + 1)).unwrap();
            assert!(e.rising);
            assert_ulps(e.time, 7.5e-4 + n as f64 * 1e-3, 4, "rising");
        }
    }

    #[test]
    fn a_delay_after_start_time_holds_the_low_level_until_the_first_rising_edge() {
        let c = clock(1000.0, 0.5, 5.0, 1.0, 3e-3);
        assert_eq!(c.level_at(0.0), 1.0, "vLow before the delay");
        assert_eq!(c.first_edge_after(0.0), Some(0));
        let first = c.edge(0).unwrap();
        assert_eq!(first.time, 3e-3, "the delay itself");
        assert!(first.rising);
        assert_eq!(first.level, 5.0);
        assert_ulps(c.edge(1).unwrap().time, 3.5e-3, 4, "then the fall");
        // A load past the delay picks the edges up in phase.
        let k = c.first_edge_after(3.2e-3).unwrap();
        assert_eq!(k, 1, "the fall at 3.5 ms is next");
    }

    #[test]
    fn duty_one_is_one_edge_at_the_delay_and_duty_zero_is_none() {
        let high = clock(1000.0, 1.0, 5.0, 0.0, 1e-3);
        assert_eq!(high.first_edge_after(0.0), Some(0));
        assert_eq!(high.edge(0).unwrap().time, 1e-3);
        assert!(high.edge(1).is_none() && high.edge(2).is_none(), "one edge only");
        assert_eq!(high.first_edge_after(1e-3), None, "a delay at START_TIME is applied at load, not scheduled");
        assert_eq!(high.first_edge_after(5.0), None);
        assert_eq!(high.level_at(0.0), 0.0);
        assert_eq!(high.level_at(1e-3), 5.0);
        let above = clock(1000.0, 1.7, 5.0, 0.0, 0.0);
        assert_eq!(above.first_edge_after(0.0), None, "duty above 1 is duty 1 with the delay at 0");

        let low = clock(1000.0, 0.0, 5.0, 0.0, 0.0);
        assert!(low.edge(0).is_none() && low.edge(1).is_none());
        assert_eq!(low.first_edge_after(0.0), None);
        assert_eq!(low.level_at(0.0), 0.0);
        assert_eq!(clock(1000.0, -0.5, 5.0, 0.0, 0.0).first_edge_after(0.0), None);
    }

    #[test]
    fn a_low_duty_puts_the_fall_just_after_the_rise() {
        let c = clock(1000.0, 0.1, 5.0, 0.0, 0.0);
        assert_ulps(c.edge(1).unwrap().time, 1e-4, 4, "fall at duty * P");
        assert_ulps(c.edge(3).unwrap().time, 1.1e-3, 4, "and the next cycle's");
    }

    #[test]
    fn an_inverted_clock_slot_follows_the_level_not_the_phase() {
        // vHigh below vLow: the "HIGH" phase reads slot 0.
        let c = clock(1000.0, 0.5, 1.0, 4.0, 0.0);
        let rising = c.edge(2).unwrap();
        assert_eq!((rising.level, rising.slot), (1.0, 0.0));
        let falling = c.edge(3).unwrap();
        assert_eq!((falling.level, falling.slot), (4.0, 1.0));
        assert_eq!(c.slot_of(2.5), 0.0, "exactly the midpoint is not above it");
    }

    #[test]
    fn a_1_mhz_clock_keeps_its_times_over_ten_thousand_edges_from_a_late_start() {
        let c = clock(1e6, 0.5, 5.0, 0.0, 0.0);
        let start = 12.345;
        let k0 = c.first_edge_after(start).expect("a running clock has an edge after START_TIME");
        assert!(c.edge(k0).unwrap().time > start);
        assert!(k0 == 0 || c.edge(k0 - 1).unwrap().time <= start, "and it is the first");

        let mut previous: Option<ClockEdge> = None;
        for i in 0..10_000u64 {
            let k = k0 + i;
            let edge = c.edge(k).unwrap();
            // Independent closed form: rising at n us, falling at (2n + 1)/2 us,
            // each a single correctly rounded division of exact integers.
            let exact = if edge.rising { (k / 2) as f64 / 1e6 } else { (k as f64) / 2e6 };
            assert_ulps(edge.time, exact, 6, &format!("edge {k}"));
            if let Some(p) = previous {
                assert!(edge.time > p.time, "edge {k} is later than the one before");
                assert_ne!(edge.rising, p.rising, "and alternates");
                // Between two edges the level is the earlier edge's, which is
                // what classic's clockVoltage reads mid-interval.
                let mid = p.time + (edge.time - p.time) / 2.0;
                assert_eq!(c.level_at(mid), p.level, "level between edges {} and {k}", k - 1);
                assert_eq!(c.slot_of(c.level_at(mid)), p.slot);
            }
            previous = Some(edge);
        }
    }

    #[test]
    fn the_edge_level_is_never_read_back_from_the_waveform_function() {
        // Probe P8: at t = 11 ms of a 1 kHz clock, fmod leaves (t % P) / P =
        // 0.9999999999999991, so clockVoltage reads LOW on the very instant of
        // the rising edge. The schedule's edge says HIGH.
        let c = clock(1000.0, 0.5, 5.0, 0.0, 0.0);
        assert_eq!(c.level_at(0.011), 0.0, "clockVoltage is on the wrong side at the edge instant");
        let edge = c.edge(22).unwrap();
        assert!(edge.rising);
        assert_ulps(edge.time, 0.011, 4, "edge 11");
        assert_eq!(edge.level, 5.0);
        assert_eq!(edge.slot, 1.0);
    }

    // ---- clock load state (Sources points 2 and 4) --------------------------------

    /// Replays a clock from `t0` against classic's `clockVoltage`: the load
    /// level, then the first six scheduled edges. Every interval wider than
    /// 1e-7 s must read, at its midpoint, the level the replay holds there. A
    /// midpoint is half an interval from any edge instant, so fmod cannot put
    /// it on the wrong side. Returns the load level and the first scheduled
    /// edge.
    fn replay_against_classic(params: [f64; 5], t0: f64) -> (SourceLevel, u64) {
        let c = Clock::from_params(&params);
        let [freq, duty, v_high, v_low, delay] = params;
        let (load, next) = c.load_state(t0);
        let first = next.unwrap_or_else(|| panic!("t0 {t0:e}: a running clock has an edge after START_TIME"));
        assert!(c.edge(first).unwrap().time > t0, "t0 {t0:e}: the first scheduled edge is after START_TIME");
        assert!(first == 0 || c.edge(first - 1).unwrap().time <= t0, "t0 {t0:e}: the one before it is at or before");
        assert_eq!(load.slot, c.slot_of(load.value));
        let (mut level, mut from) = (load.value, t0);
        for k in first..first + 6 {
            let edge = c.edge(k).unwrap();
            if edge.time - from > 1e-7 {
                let mid = from + (edge.time - from) / 2.0;
                let classic = clock_voltage(mid, 1.0 / freq, duty, v_high, v_low, delay);
                assert_eq!(classic, level, "t0 {t0:e}: the level at {mid:e}, before edge {k}");
            }
            (level, from) = (edge.level, edge.time);
        }
        (load, first)
    }

    #[test]
    fn a_clock_started_on_a_rising_edge_instant_loads_high() {
        // START_TIME 5.0 at 1 kHz, duty 0.5: edge 10000 is rising at 5000 *
        // 1e-3 = 5.0. Classic's fmod reads (5.0 % 1e-3) / 1e-3 just under 1
        // there, so clockVoltage is LOW at the very instant the edge is due,
        // and a literal clockVoltage load would lose the whole first high
        // half-period.
        let c = clock(1000.0, 0.5, 5.0, 0.0, 0.0);
        assert_eq!(clock_voltage(5.0, 1.0 / 1000.0, 0.5, 5.0, 0.0, 0.0), 0.0, "classic reads LOW on the edge instant");
        assert_eq!(c.level_at(5.0), 0.0);
        let rising = c.edge(10_000).unwrap();
        assert!(rising.rising);
        assert_eq!(rising.time, 5.0, "the edge instant is START_TIME itself");
        let (load, next) = c.load_state(5.0);
        assert_eq!((load.value, load.slot, load.ramp), (5.0, 1.0, None), "the edge is applied at load");
        assert_eq!(next, Some(10_001), "and the next edge is the fall");
        let falling = c.edge(10_001).unwrap();
        assert!(!falling.rising);
        assert_ulps(falling.time, 5.0005, 4, "falling at 5.0005");
        // One ulp before the instant the rise is still to come.
        let (load, next) = c.load_state(f64::from_bits(5.0f64.to_bits() - 1));
        assert_eq!((load.value, next), (0.0, Some(10_000)));
    }

    #[test]
    fn a_clock_started_on_a_falling_edge_instant_loads_low() {
        // 0.5 ms is the falling edge (0.5 * 1e-3, exact). The level after it
        // is LOW, as classic's phase 0.5 is not below the duty.
        let c = clock(1000.0, 0.5, 5.0, 1.0, 0.0);
        let (load, next) = c.load_state(5e-4);
        assert_eq!((load.value, load.slot, next), (1.0, 0.0, Some(2)));
        assert_eq!(c.level_at(5e-4), 1.0, "classic agrees on this instant");
        assert_ulps(c.edge(2).unwrap().time, 1e-3, 4, "the next rise");
    }

    #[test]
    fn a_decimal_millisecond_sweep_loses_no_edge_at_load() {
        // 2,999 start times at 1 kHz, m = 1..=2999 ms. Each is a rising edge
        // instant, and classic's fmod reads LOW at many of them: 1,467 of the
        // `m * 1e-3` starts (the verifier's count) and 1,866 of the
        // correctly rounded `m / 1000` ones (both counts from node, running
        // classic's clockVoltage). The expected level is the closed form:
        // the rise at m ms is applied (HIGH, the fall at m + 0.5 ms next).
        // The one exception is an edge whose computed time, 0 + m * P, sits
        // an ulp above the decimal `m / 1000` (399 of them, counted in node
        // as `m * 0.001 > m / 1000`): that edge is then scheduled at the
        // instant itself, never lost.
        let params = [1000.0, 0.5, 5.0, 0.0, 0.0];
        let c = Clock::from_params(&params);
        for (name, start, want_classic_low, want_scheduled) in [
            ("m * 1e-3", (|m: u32| m as f64 * 1e-3) as fn(u32) -> f64, 1467, 0),
            ("m / 1000", (|m: u32| m as f64 / 1000.0) as fn(u32) -> f64, 1866, 399),
        ] {
            let (mut classic_low, mut applied, mut scheduled) = (0, 0, 0);
            for m in 1..=2999u32 {
                let t0 = start(m);
                if clock_voltage(t0, 1.0 / 1000.0, 0.5, 5.0, 0.0, 0.0) == 0.0 {
                    classic_low += 1;
                }
                let (load, first) = replay_against_classic(params, t0);
                let next = c.edge(first).unwrap();
                if load.value == 5.0 {
                    applied += 1;
                    assert!(!next.rising, "{name} m {m}: the edge after a HIGH load is the fall");
                    assert_ulps(next.time, (2 * m + 1) as f64 / 2000.0, 8, &format!("{name} m {m}: the fall"));
                } else {
                    scheduled += 1;
                    assert_eq!(load.value, 0.0);
                    assert!(next.rising, "{name} m {m}: the edge after a LOW load at a rising instant is the rise");
                    assert!((next.time - t0).abs() < 1e-12, "{name} m {m}: scheduled at the instant itself");
                }
            }
            assert_eq!(classic_low, want_classic_low, "{name}: starts a literal clockVoltage load would have begun LOW");
            assert_eq!(applied + scheduled, 2999, "{name}");
            assert_eq!(scheduled, want_scheduled, "{name}: rises an ulp past the decimal start");
        }
    }

    #[test]
    fn a_sweep_over_other_clocks_and_instants_matches_classic_between_edges() {
        // 0.05 ms steps over 6 ms: rising, falling and mid-phase instants, a
        // duty that is not a half, a delay inside the range, a delay before
        // START_TIME, a negative delay, and an inverted pair of levels.
        for params in [
            [1000.0, 0.3, 5.0, 0.0, 0.0],
            [1000.0, 0.25, 3.3, 0.5, 2.5e-3],
            [1000.0, 0.5, 5.0, 0.0, -1.5e-4],
            [2000.0, 0.75, 1.0, 4.0, -7e-4],
            [250.0, 0.1, 5.0, 0.0, 1.2e-3],
        ] {
            for m in 0..=120u32 {
                replay_against_classic(params, m as f64 * 5e-5);
            }
        }
    }

    #[test]
    fn a_negative_delay_loads_the_phase_shifted_level() {
        // delay -0.25 ms: rising at -0.25 ms + n ms, falling at +0.25 ms +
        // n ms. At 0 the clock is 0.25 of a cycle in: HIGH, the fall next.
        let c = clock(1000.0, 0.5, 5.0, 0.0, -2.5e-4);
        let (load, next) = c.load_state(0.0);
        assert_eq!((load.value, next), (5.0, Some(1)));
        assert_ulps(c.edge(1).unwrap().time, 2.5e-4, 4, "the fall");
        // On the next rising edge instant (0.75 ms): applied, the fall at
        // 1.25 ms next. Classic's fmod may read either side of it.
        let (load, next) = c.load_state(7.5e-4);
        assert_eq!((load.value, next), (5.0, Some(3)));
        // A negative delay with a duty of 1 is HIGH from before START_TIME.
        let high = clock(1000.0, 1.0, 5.0, 0.0, -1e-3);
        assert_eq!(high.load_state(0.0).0.value, 5.0);
        assert_eq!(high.load_state(0.0).1, None);
        assert_eq!(high.level_at(0.0), 5.0);
        // And just before the delay (START_TIME below it), LOW with the edge to come.
        let (load, next) = high.load_state(-2e-3);
        assert_eq!((load.value, next), (0.0, Some(0)));
    }

    #[test]
    fn duty_one_or_more_loads_on_the_delay_against_start_time() {
        for duty in [1.0, 1.7] {
            // vLow 1.0, so a LOW load is not mistaken for 0.
            let at = clock(1000.0, duty, 5.0, 1.0, 1e-3);
            // delay == START_TIME: the edge is at or before it, so it is
            // applied. classicVoltage agrees (phase 0 is below any duty).
            let (load, next) = at.load_state(1e-3);
            assert_eq!((load.value, load.slot, next), (5.0, 1.0, None), "duty {duty}: delay == START_TIME");
            assert_eq!(at.level_at(1e-3), 5.0);
            // delay > START_TIME: LOW now, the one edge to come at the delay.
            let (load, next) = at.load_state(0.0);
            assert_eq!((load.value, load.slot, next), (1.0, 0.0, Some(0)), "duty {duty}: delay > START_TIME");
            assert_eq!(at.level_at(0.0), 1.0);
            let edge = at.edge(0).unwrap();
            assert_eq!((edge.time, edge.rising, edge.level), (1e-3, true, 5.0));
            // delay < START_TIME: HIGH and nothing to come.
            let (load, next) = at.load_state(2e-3);
            assert_eq!((load.value, next), (5.0, None), "duty {duty}: delay < START_TIME");
            assert_eq!(at.level_at(2e-3), 5.0);
        }
    }

    #[test]
    fn duty_zero_or_less_loads_low_with_no_edge() {
        for duty in [0.0, -0.5] {
            for delay in [0.0, 3e-3, -1e-3] {
                let c = clock(1000.0, duty, 5.0, 1.5, delay);
                let (load, next) = c.load_state(0.0);
                assert_eq!((load.value, load.slot, next), (1.5, 0.0, None), "duty {duty}, delay {delay:e}");
                assert_eq!(c.level_at(0.0), 1.5);
            }
        }
        // The slot follows the level against the midpoint, not the phase: an
        // inverted pair puts vLow above it.
        assert_eq!(clock(1000.0, 0.0, 1.0, 4.0, 0.0).load_state(1.0).0.slot, 1.0);
    }

    #[test]
    fn a_delay_after_start_time_loads_low_with_the_first_rise_to_come() {
        let c = clock(1000.0, 0.5, 5.0, 1.0, 3e-3);
        let (load, next) = c.load_state(0.0);
        assert_eq!((load.value, next), (1.0, Some(0)));
        // START_TIME on the delay itself: the rise is applied.
        let (load, next) = c.load_state(3e-3);
        assert_eq!((load.value, next), (5.0, Some(1)));
    }

    #[test]
    fn a_start_time_past_the_edge_index_range_holds_classics_level() {
        // 1e13 s at 1 kHz is 1e16 cycles, beyond the 2^52 the schedule
        // indexes: no edge can be named, and the load level is clockVoltage.
        // Classic's fmod reads LOW at 1e13 s and HIGH at 3e13 s (node), so
        // neither constant level passes for both.
        let c = clock(1000.0, 0.5, 5.0, 0.0, 0.0);
        for (t0, classic) in [(1e13, 0.0), (3e13, 5.0)] {
            let (load, next) = c.load_state(t0);
            assert_eq!(next, None, "t0 {t0:e}");
            assert_eq!(load.value, classic, "t0 {t0:e}");
            assert_eq!(load.value, clock_voltage(t0, 1.0 / 1000.0, 0.5, 5.0, 0.0, 0.0));
        }
        // Just inside the range the schedule still resolves.
        let t0 = 4e12;
        let k = c.load_state(t0).1.expect("inside the range");
        assert!(c.edge(k).unwrap().time > t0 && c.edge(k - 1).unwrap().time <= t0);
    }

    // ---- clock edges at a duty within ulps of 1 (Sources point 2) ------------------------

    /// The largest `f64` under 1: `1 - 2^-53`, and so `1 - j * 2^-53` for the
    /// next few `duty` values below it.
    const ULP_UNDER_ONE: f64 = 1.0 / 9_007_199_254_740_992.0;

    #[test]
    fn a_fall_never_lands_after_the_next_rise() {
        // README Sources point 2: a falling edge is `min(delay + n P + duty P,
        // delay + (n + 1) P)`. At 1 kHz and duty 0.9999999999999999 the first
        // term of edge 37 computes to 0.019000000000000003, after edge 38's
        // rise at 0.019. Expected times are the README rule evaluated in node
        // (an independent transcription, not this code), printed as shortest
        // round-trip decimals.
        let duty = 1.0 - ULP_UNDER_ONE;
        assert_eq!(duty, 0.9999999999999999);
        let c = clock(1000.0, duty, 5.0, 0.0, 0.0);
        let want = [
            (34, 0.017, true),
            (35, 0.018000000000000002, false),
            (36, 0.018000000000000002, true),
            (37, 0.019, false),
            (38, 0.019, true),
            (39, 0.02, false),
            (40, 0.02, true),
            (41, 0.021, false),
        ];
        for (k, time, rising) in want {
            let edge = c.edge(k).unwrap();
            assert_eq!((edge.time, edge.rising), (time, rising), "edge {k}");
        }
        for k in 0..400 {
            assert!(c.edge(k).unwrap().time <= c.edge(k + 1).unwrap().time, "edge {k} is after edge {}", k + 1);
        }

        // The same at t = 1e6 s (cycle 1e9), where the rounding bites too:
        // edge 1999999965's raw time is 999999.9830000001, an ulp after the
        // rise that follows it at 999999.983.
        let c0: u64 = 1_000_000_000;
        let (k, clamped) = (1_999_999_965u64, 999999.983);
        assert_eq!(c.edge(k).unwrap().time, clamped, "the clamped fall");
        assert_eq!(c.edge(k + 1).unwrap().time, clamped, "shares the next rise's instant");
        assert!(!c.edge(k).unwrap().rising && c.edge(k + 1).unwrap().rising);
        for k in 2 * c0 - 40..2 * c0 + 40 {
            assert!(c.edge(k).unwrap().time <= c.edge(k + 1).unwrap().time, "edge {k} is after edge {}", k + 1);
        }
    }

    #[test]
    fn the_load_state_agrees_with_the_clamped_schedule() {
        let c = clock(1000.0, 1.0 - ULP_UNDER_ONE, 5.0, 0.0, 0.0);
        // START_TIME 0.019: the clamped fall (edge 37) and the rise (38) are
        // both at 0.019, so both are applied at load, in that order, and the
        // clock loads HIGH with edge 39 (the fall at 0.02) next. Classic's
        // clockVoltage reads HIGH there too (node). Before the clamp the fall
        // sat at 0.019000000000000003, loaded as (HIGH, Some(37)), and edge 38
        // would have been scheduled in the past.
        let (load, next) = c.load_state(0.019);
        assert_eq!((load.value, load.slot, next), (5.0, 1.0, Some(39)));
        assert_eq!(c.level_at(0.019), 5.0);
        // An ulp earlier neither is applied: HIGH from the rise at 0.018, with
        // the fall (37) next, then the rise at the very same instant.
        let before = f64::from_bits(0.019f64.to_bits() - 1);
        let (load, next) = c.load_state(before);
        assert_eq!((load.value, next), (5.0, Some(37)));
        let (fall, rise) = (c.edge(37).unwrap(), c.edge(38).unwrap());
        assert_eq!((fall.time, fall.level), (rise.time, 0.0), "the fall is applied first, to LOW");
        assert_eq!(rise.level, 5.0, "and the rise after it leaves the clock HIGH");
        // For every start in the range, the edges before the first scheduled
        // one are at or before it and it is after it.
        for m in 0..=40u32 {
            for t0 in [m as f64 / 1000.0, m as f64 * 1e-3, f64::from_bits((m as f64 / 1000.0).to_bits() + 1)] {
                let (load, next) = c.load_state(t0);
                let k = next.expect("a running clock has an edge to come");
                assert!(c.edge(k).unwrap().time > t0, "t0 {t0:e}: edge {k} is after it");
                assert!(k == 0 || c.edge(k - 1).unwrap().time <= t0, "t0 {t0:e}: edge {} is at or before it", k.wrapping_sub(1));
                let level = if k == 0 { 0.0 } else { c.edge(k - 1).unwrap().level };
                assert_eq!(load.value, level, "t0 {t0:e}");
            }
        }
    }

    #[test]
    fn edge_times_never_fall_with_the_index_for_duties_within_ulps_of_one() {
        // Every adjacent pair across a window of 80 edges at each time, for
        // duty = 1 - j * 2^-53 (j 1..=4), five frequencies, three delays (one a
        // negative phase shift) and six times from 0.019 s to 3e8 s.
        let mut checked = 0u32;
        let mut unclamped_falls_past_the_next_rise = 0u32;
        for freq in [1.0, 50.0, 1000.0, 44_100.0, 1e6] {
            for j in 1..=4u32 {
                let duty = 1.0 - j as f64 * ULP_UNDER_ONE;
                for delay in [0.0, 0.25, -1.5] {
                    let c = clock(freq, duty, 5.0, 0.0, delay);
                    let period = 1.0 / freq;
                    for t in [0.019, 1.0, 12.345, 1e3, 1e6, 3e8] {
                        let cycle = (((t - delay) / period).floor()).max(0.0) as u64;
                        for k in (2 * cycle).saturating_sub(40)..2 * cycle + 40 {
                            let (a, b) = (c.edge(k).unwrap(), c.edge(k + 1).unwrap());
                            assert!(
                                a.time <= b.time,
                                "freq {freq}, duty {duty}, delay {delay}, t {t}: edge {k} ({:e}) is after edge {} ({:e})",
                                a.time,
                                k + 1,
                                b.time
                            );
                            checked += 1;
                            // The raw first term of a fall, to prove the sweep
                            // reaches the clamp at all.
                            if !a.rising && delay + (k / 2) as f64 * period + duty * period > b.time {
                                unclamped_falls_past_the_next_rise += 1;
                            }
                        }
                    }
                }
            }
        }
        assert_eq!(checked, 26_880);
        // Counted in node over the same grid, for the unclamped expression:
        // without the `min` these 1,117 pairs would be out of order.
        assert_eq!(unclamped_falls_past_the_next_rise, 1_117);
    }

    #[test]
    fn first_edge_after_starts_a_cycle_below_its_estimate() {
        // `floor((t0 - delay) / P)` is the cycle `t0` sits in, give or take a
        // cycle. With `t0` an ulp under a rise it divides up to the rise's own
        // cycle, and with a duty within ulps of 1 the previous cycle's fall is
        // also after `t0`: the first edge after is that fall, index `2 *
        // estimate - 1`, one below the estimate's own first edge. Starting the
        // scan at `2 * estimate` skips it and loads the clock LOW against a
        // clockVoltage that reads HIGH. Each row is (freq, duty, delay, t0,
        // estimate, first edge after, that edge's time): the README rule
        // evaluated in node, with classic's clockVoltage reading HIGH at `t0`
        // for every row.
        let near_one = 1.0 - ULP_UNDER_ONE;
        let rows: [(f64, f64, f64, f64, f64, u64, f64); 6] = [
            (1000.0, near_one, 0.0, 0.018, 18.0, 35, 0.018000000000000002),
            (1000.0, 0.999999999999999, 0.0, 0.036, 36.0, 71, 0.036000000000000004),
            (50.0, near_one, 0.5, 1.2, 35.0, 69, 1.2000000000000002),
            (440.0, near_one, 0.0, 0.022727272727272724, 10.0, 19, 0.022727272727272728),
            (1.0, near_one, -1.5, 1.4999999999999998, 3.0, 5, 1.5),
            (1e6, near_one, 0.0, 1.1999999999999999e-5, 12.0, 23, 1.2e-5),
        ];
        for (freq, duty, delay, t0, estimate, first, time) in rows {
            let c = clock(freq, duty, 5.0, 0.0, delay);
            let label = format!("freq {freq}, duty {duty}, delay {delay}, t0 {t0:e}");
            assert_eq!(((t0 - delay) / c.period()).floor(), estimate, "{label}: the estimate");
            assert!(2 * estimate as u64 > first, "{label}: so the edge is below the estimate's own first edge");
            assert_eq!(c.first_edge_after(t0), Some(first), "{label}");
            assert_eq!(c.edge(first).unwrap().time, time, "{label}: its time");
            assert!(time > t0 && c.edge(first - 1).unwrap().time <= t0, "{label}");
            assert!(!c.edge(first).unwrap().rising, "{label}: it is a fall");
            let (load, next) = c.load_state(t0);
            assert_eq!((load.value, next), (5.0, Some(first)), "{label}: HIGH, the fall to come");
            assert_eq!(c.level_at(t0), 5.0, "{label}: classic reads HIGH");
        }
    }

    // ---- ramp threshold ----------------------------------------------------------------

    /// `{v1, v2, td, tr, tf, pw, per}` as a record.
    fn pulse(v1: f64, v2: f64, td: f64, tr: f64, tf: f64, pw: f64, per: f64) -> Pulse {
        Pulse::from_params(&[v1, v2, td, tr, tf, pw, per])
    }

    #[test]
    fn the_catalog_defaults_are_midpoint_steps() {
        // 1 us ramps, pw 500 us, per 1 ms: T_ev = 498 us, 0.002 of it.
        let p = pulse(0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3);
        assert_ulps(event_timescale(1e-6, 1e-6, 5e-4, 1e-3), 4.98e-4, 16, "T_ev");
        assert_eq!((p.rise_mode(), p.fall_mode()), (RampMode::Step, RampMode::Step));
        assert!(!p.has_resolved_ramp());
        assert_eq!(p.events_per_cycle(), 2);
    }

    #[test]
    fn a_1_ms_rise_with_4_ms_pulse_width_is_resolved() {
        // 1 ms ramps, pw 4 ms, per 10 ms: T_ev = 4 ms, 0.25 of it.
        let p = pulse(0.0, 5.0, 0.0, 1e-3, 1e-3, 4e-3, 1e-2);
        assert_ulps(event_timescale(1e-3, 1e-3, 4e-3, 1e-2), 4e-3, 16, "T_ev");
        assert_eq!((p.rise_mode(), p.fall_mode()), (RampMode::Resolved, RampMode::Resolved));
        assert!(p.has_resolved_ramp());
        assert_eq!(p.events_per_cycle(), 6);
    }

    #[test]
    fn the_threshold_is_one_percent_of_the_event_timescale_inclusive() {
        // One-shot: T_ev = pw = 1 ms, so the threshold is RAMP_STEP_RATIO * 1e-3.
        let limit = RAMP_STEP_RATIO * 1e-3;
        assert_eq!(pulse(0.0, 5.0, 0.0, limit, limit, 1e-3, 0.0).rise_mode(), RampMode::Step, "on the threshold is a step");
        let just_over = f64::from_bits(limit.to_bits() + 1);
        assert_eq!(pulse(0.0, 5.0, 0.0, just_over, limit, 1e-3, 0.0).rise_mode(), RampMode::Resolved, "an ulp over is resolved");
        assert_eq!(pulse(0.0, 5.0, 0.0, just_over, limit, 1e-3, 0.0).fall_mode(), RampMode::Step, "each ramp on its own");
        assert_eq!(ramp_mode(1e-5, 1e-3), RampMode::Step);
        assert_eq!(ramp_mode(2e-5, 1e-3), RampMode::Resolved);
    }

    #[test]
    fn a_zero_length_ramp_is_a_step_whatever_the_timescale() {
        assert_eq!(ramp_mode(0.0, 0.0), RampMode::Step);
        assert_eq!(ramp_mode(0.0, 1e-3), RampMode::Step);
        // pw = 0 makes T_ev 0, so any ramp with a length is resolved.
        assert_eq!(ramp_mode(1e-9, 0.0), RampMode::Resolved);
        let p = pulse(0.0, 5.0, 0.0, 0.0, 1e-3, 0.0, 1e-2);
        assert_eq!((p.rise_mode(), p.fall_mode()), (RampMode::Step, RampMode::Resolved));
    }

    #[test]
    fn the_timescale_uses_the_shorter_flat_segment() {
        // Long pulse, short gap: per - tr - pw - tf = 1 ms is the shorter.
        assert_ulps(event_timescale(1e-6, 1e-6, 9e-3, 1e-2 + 2e-6), 1e-3, 64, "gap");
        assert_eq!(event_timescale(1e-6, 1e-6, 4e-3, 0.0), 4e-3, "one-shot");
        assert_eq!(event_timescale(1e-6, 1e-6, 4e-3, f64::INFINITY), 4e-3, "a non-finite period is one-shot");
        assert!(event_timescale(1e-3, 1e-3, 1e-3, 3e-3) < 1e-18, "per = tr + pw + tf leaves no gap");
    }

    // ---- pulse edges ---------------------------------------------------------------------

    fn times(p: &Pulse, from: u64, count: u64) -> Vec<(f64, bool, f64)> {
        (from..from + count).map(|k| p.event(k).unwrap()).map(|e| (e.time, e.rising, e.level)).collect()
    }

    #[test]
    fn a_catalog_pulse_steps_at_the_ramp_midpoints() {
        // {0, 5, 0, 1 us, 1 us, 500 us, 1 ms}: up at 0.5 us, down at 501.5 us,
        // up at 1.0005 ms.
        let p = pulse(0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3);
        let e = times(&p, 0, 3);
        assert_ulps(e[0].0, 5e-7, 8, "first up");
        assert_ulps(e[1].0, 5.015e-4, 8, "down");
        assert_ulps(e[2].0, 1.0005e-3, 8, "second up");
        assert_eq!((e[0].1, e[1].1, e[2].1), (true, false, true));
        assert_eq!((e[0].2, e[1].2, e[2].2), (5.0, 0.0, 5.0));
        let up = p.event(0).unwrap();
        assert_eq!((up.kind, up.slot, up.ramp), (PulseEventKind::Step, 1.0, None));
        assert_eq!(p.event(1).unwrap().slot, 0.0);
        // The flip instants are the ramp midpoints classic's readout flips at
        // (probe P8: 0 at 0.2 us, 1 at 0.6 us of a 1 us rise).
        assert_eq!(p.slot_of(p.voltage_at(0.2e-6)), 0.0);
        assert_eq!(p.slot_of(p.voltage_at(0.6e-6)), 1.0);
    }

    #[test]
    fn a_one_shot_has_two_edges_only() {
        let p = pulse(0.0, 5.0, 1e-3, 1e-6, 1e-6, 5e-4, 0.0);
        assert!(p.is_one_shot());
        let e = times(&p, 0, 2);
        assert_ulps(e[0].0, 1.0005e-3, 8, "up");
        assert_ulps(e[1].0, 1.5015e-3, 8, "down");
        assert!(p.event(2).is_none() && p.event(3).is_none() && p.event(100).is_none());
        assert_eq!(p.first_event_after(0.0), Some(0));
        assert_eq!(p.first_event_after(1.2e-3), Some(1));
        assert_eq!(p.first_event_after(2e-3), None, "past the last edge");
        // The fall-only state at load after the pulse, and a load inside it.
        let (level, next) = p.load_state(2e-3);
        assert_eq!((level.value, level.slot, next), (0.0, 0.0, None));
        let (level, next) = p.load_state(1.2e-3);
        assert_eq!((level.value, level.slot, next), (5.0, 1.0, Some(1)));
        let (level, next) = p.load_state(0.0);
        assert_eq!((level.value, level.slot, next), (0.0, 0.0, Some(0)));
    }

    #[test]
    fn zero_ramps_step_at_td_and_at_td_plus_pw() {
        let p = pulse(0.0, 5.0, 2e-3, 0.0, 0.0, 1e-3, 4e-3);
        assert_eq!(p.events_per_cycle(), 2);
        assert_eq!(p.event(0).unwrap().time, 2e-3, "up at td");
        assert_eq!(p.event(1).unwrap().time, 2e-3 + 1e-3, "down at td + pw");
        assert_eq!(p.event(2).unwrap().time, 2e-3 + 4e-3, "the next cycle's up");
        assert_eq!(p.event(3).unwrap().time, 2e-3 + 4e-3 + 1e-3);
    }

    #[test]
    fn a_pulse_that_never_leaves_v1_has_no_edges() {
        // tr = tf = pw = 0: classic never leaves v1 (WF:44-56).
        let flat = pulse(0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1e-3);
        assert_eq!(flat.events_per_cycle(), 0);
        assert!(flat.event(0).is_none());
        assert_eq!(flat.first_event_after(0.0), None);
        assert_eq!(flat.voltage_at(1e-4), 0.0, "and classic agrees");
        // v1 = v2: nothing to change.
        let same = pulse(3.0, 3.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3);
        assert_eq!(same.events_per_cycle(), 0);
        assert!(same.event(0).is_none());
        let (level, next) = same.load_state(0.5);
        assert_eq!((level.value, level.slot, next), (3.0, 0.0, None));
    }

    #[test]
    fn pulse_edge_times_do_not_drift_over_ten_thousand_cycles_from_a_late_start() {
        let p = pulse(0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3);
        let start = 12.345;
        let k0 = p.first_event_after(start).unwrap();
        assert!(p.event(k0).unwrap().time > start && p.event(k0 - 1).unwrap().time <= start);
        let mut previous: Option<PulseEvent> = None;
        for i in 0..20_000u64 {
            let k = k0 + i;
            let e = p.event(k).unwrap();
            // Independent closed form: up at (1000 c + 0.5) us, down at
            // (1000 c + 501.5) us; each numerator is exact, so one division.
            let c = (k / 2) as f64;
            let exact = if e.rising { (c * 1000.0 + 0.5) / 1e6 } else { (c * 1000.0 + 501.5) / 1e6 };
            assert_ulps(e.time, exact, 6, &format!("event {k}"));
            if let Some(prev) = previous {
                assert!(e.time > prev.time, "event {k} is later than the one before");
                // The flat level between two events is pulseVoltage's there.
                let mid = prev.time + (e.time - prev.time) / 2.0;
                assert_eq!(p.voltage_at(mid), prev.level, "between events {} and {k}", k - 1);
            }
            previous = Some(e);
        }
    }

    #[test]
    fn a_resolved_rise_and_fall_have_corners_midpoints_and_a_linear_value() {
        // {0, 5, 0, 1 ms, 1 ms, 4 ms, 10 ms}: rise 0..1 ms, flat to 5 ms, fall
        // 5..6 ms.
        let p = pulse(0.0, 5.0, 0.0, 1e-3, 1e-3, 4e-3, 1e-2);
        let want = [
            (0.0, PulseEventKind::RampStart, true, 0.0, 0.0),
            (5e-4, PulseEventKind::RampMid, true, 2.5, 1.0),
            (1e-3, PulseEventKind::RampEnd, true, 5.0, 1.0),
            (5e-3, PulseEventKind::RampStart, false, 5.0, 1.0),
            (5.5e-3, PulseEventKind::RampMid, false, 2.5, 0.0),
            (6e-3, PulseEventKind::RampEnd, false, 0.0, 0.0),
        ];
        for (k, (time, kind, rising, level, slot)) in want.into_iter().enumerate() {
            let e = p.event(k as u64).unwrap();
            assert_eq!(e.kind, kind, "event {k}");
            assert_eq!(e.rising, rising, "event {k}");
            assert_ulps(e.time, time, 8, &format!("time of event {k}"));
            // A mid level is a difference of two nearby instants over the ramp.
            assert!((e.level - level).abs() < 1e-9, "level of event {k}: {} vs {level}", e.level);
            assert_eq!(e.slot, slot, "slot of event {k}");
            assert!(e.ramp.is_some());
        }
        // The next cycle starts at 10 ms.
        assert_ulps(p.event(6).unwrap().time, 1e-2, 8, "cycle 1 start");
        assert_eq!(p.event(6).unwrap().kind, PulseEventKind::RampStart);
    }

    #[test]
    fn the_resolved_value_between_corners_is_pulse_voltages() {
        let p = pulse(0.0, 5.0, 0.0, 1e-3, 2e-3, 4e-3, 1e-2);
        let rise = p.event(0).unwrap().ramp.unwrap();
        let fall = p.event(3).unwrap().ramp.unwrap();
        assert_eq!((rise.t0, rise.len, rise.v_from, rise.v_to), (0.0, 1e-3, 0.0, 5.0));
        assert_ulps(fall.t0, 5e-3, 8, "fall starts at tr + pw");
        assert_eq!((fall.len, fall.v_from, fall.v_to), (2e-3, 5.0, 0.0));
        for i in 1..100 {
            let t = i as f64 * 1e-5;
            assert_ulps(rise.value(t), p.voltage_at(t), 16, &format!("rise at t = {t}"));
        }
        for i in 100..500 {
            let t = i as f64 * 1e-5;
            assert_eq!(p.voltage_at(t), 5.0, "the plateau at t = {t}");
            assert_eq!(rise.value(t), 5.0, "past its end the segment holds v_to");
        }
        for i in 501..700 {
            let t = i as f64 * 1e-5;
            assert_ulps(fall.value(t), p.voltage_at(t), 16, &format!("fall at t = {t}"));
        }
        assert_eq!(rise.value(-1.0), 0.0, "before the start it holds v_from");
        assert_eq!(fall.value(1.0), 0.0);
    }

    #[test]
    fn a_step_rise_with_a_resolved_fall_orders_one_event_then_three() {
        // tr = 1 us is a step, tf = 1 ms is resolved (T_ev 4 ms).
        let p = pulse(0.0, 5.0, 0.0, 1e-6, 1e-3, 4e-3, 1e-2);
        assert_eq!((p.rise_mode(), p.fall_mode()), (RampMode::Step, RampMode::Resolved));
        assert_eq!(p.events_per_cycle(), 4);
        let kinds: Vec<_> = (0..4).map(|k| p.event(k).unwrap().kind).collect();
        assert_eq!(kinds, [PulseEventKind::Step, PulseEventKind::RampStart, PulseEventKind::RampMid, PulseEventKind::RampEnd]);
        assert_ulps(p.event(0).unwrap().time, 5e-7, 8, "the step at the rise's midpoint");
        assert_ulps(p.event(1).unwrap().time, 4.001e-3, 8, "the fall starts at tr + pw");
    }

    #[test]
    fn a_triangle_pulse_shares_one_instant_between_the_rise_end_and_the_fall_start() {
        // pw = 0 with two resolved ramps: the rise's end and the fall's start
        // are one instant, in that order.
        let p = pulse(0.0, 5.0, 0.0, 1e-3, 1e-3, 0.0, 1e-2);
        let end = p.event(2).unwrap();
        let start = p.event(3).unwrap();
        assert_eq!((end.kind, end.rising), (PulseEventKind::RampEnd, true));
        assert_eq!((start.kind, start.rising), (PulseEventKind::RampStart, false));
        assert_eq!(end.time, start.time);
        assert_eq!((end.level, start.level), (5.0, 5.0));
    }

    #[test]
    fn a_pulse_with_a_negative_td_has_its_first_events_in_the_past() {
        // td = -0.5 ms: cycle 0 began before START_TIME. Its up step (at
        // -499.5 us) is applied at load; its down step, at -500 us + 501.5 us =
        // 1.5 us, is still to come. The next up is cycle 1's, at 500.5 us.
        let p = pulse(0.0, 5.0, -5e-4, 1e-6, 1e-6, 5e-4, 1e-3);
        let (level, next) = p.load_state(0.0);
        assert_eq!((level.value, level.slot), (5.0, 1.0), "still inside the first pulse");
        assert_eq!(p.voltage_at(0.0), 5.0, "as classic reads it");
        assert_eq!(next, Some(1));
        assert!((p.event(1).unwrap().time - 1.5e-6).abs() < 1e-15, "down at 1.5 us");
        assert!(!p.event(1).unwrap().rising);
        assert!((p.event(2).unwrap().time - 5.005e-4).abs() < 1e-15, "then up at td + per + tr/2");
        // Later loads agree with classic's level: high inside cycle 1's pulse
        // (500.5 us to 1.0015 ms), low after it.
        let (level, next) = p.load_state(9e-4);
        assert_eq!((level.value, level.slot, next), (5.0, 1.0, Some(3)));
        assert_eq!(p.voltage_at(9e-4), 5.0);
        let (level, next) = p.load_state(1.2e-3);
        assert_eq!((level.value, level.slot, next), (0.0, 0.0, Some(4)));
        assert_eq!(p.voltage_at(1.2e-3), 0.0);
    }

    #[test]
    fn the_load_level_is_the_level_after_the_last_event_at_or_before_start_time() {
        let p = pulse(0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3);
        // Inside the high plateau.
        let (level, next) = p.load_state(2e-4);
        assert_eq!((level.value, level.slot), (5.0, 1.0));
        assert_eq!(next, Some(1));
        // In the low gap.
        let (level, next) = p.load_state(7e-4);
        assert_eq!((level.value, level.slot), (0.0, 0.0));
        assert_eq!(next, Some(2));
        // Exactly on an event's own time: applied.
        let t_up = p.event(0).unwrap().time;
        let (level, next) = p.load_state(t_up);
        assert_eq!((level.value, level.slot, next), (5.0, 1.0, Some(1)));
    }

    #[test]
    fn a_load_inside_a_resolved_ramp_takes_pulse_voltage_and_keeps_the_ramp_going() {
        let p = pulse(0.0, 5.0, 0.0, 1e-3, 1e-3, 4e-3, 1e-2);
        // 2.5e-4: on the rise before its midpoint.
        let (level, next) = p.load_state(2.5e-4);
        assert_eq!(level.value, p.voltage_at(2.5e-4));
        assert_ulps(level.value, 1.25, 8, "a quarter up the rise");
        assert_eq!(level.slot, 0.0, "the slot flips at the midpoint, not yet");
        assert!(level.ramp.is_some());
        assert_eq!(next, Some(1), "the midpoint is next");
        // 7.5e-4: past the midpoint, before the end corner.
        let (level, next) = p.load_state(7.5e-4);
        assert_ulps(level.value, 3.75, 8, "three quarters up");
        assert_eq!(level.slot, 1.0);
        assert_eq!(next, Some(2));
        // 2e-3: on the plateau, no ramp.
        let (level, _) = p.load_state(2e-3);
        assert_eq!((level.value, level.slot), (5.0, 1.0));
        assert!(level.ramp.is_none());
    }

    #[test]
    fn the_first_event_after_is_found_across_a_thousand_cycles() {
        // The estimate divides by the period; the event times decide.
        let p = pulse(0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3);
        for &t in &[0.0, 1e-3, 0.999e-3, 1.0005e-3, 3.2e-3, 0.5, 12.345, 100.0001] {
            let k = p.first_event_after(t).unwrap();
            assert!(p.event(k).unwrap().time > t, "t {t}");
            assert!(k == 0 || p.event(k - 1).unwrap().time <= t, "t {t}");
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Regime {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Regime::Cv => 0,
            Regime::Cc => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for ClockEdge {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.time, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rising, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.level, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slot, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Clock {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.period, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.duty, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_high, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_low, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.delay, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for RampMode {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            RampMode::Step => 0,
            RampMode::Resolved => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for RampSeg {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.len, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_from, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_to, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PulseEventKind {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            PulseEventKind::Step => 0,
            PulseEventKind::RampStart => 0,
            PulseEventKind::RampMid => 0,
            PulseEventKind::RampEnd => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for PulseEvent {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.time, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rising, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.level, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slot, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramp, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SourceLevel {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slot, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramp, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Pulse {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v2, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.td, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.tr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.tf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pw, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.per, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rise, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.fall, w); }
        n
    }
}
