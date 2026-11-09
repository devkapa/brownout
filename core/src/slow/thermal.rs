//! Package thermal one-pole and thermal boundaries (G1): core/README.md, W1g kernel semantics, points 3-4.
//!
//! Pure functions over times, temperatures and segment trajectories, plus the
//! per-device thermal entry that holds them lazily as `(T0, t0, drive)`. Nothing
//! here reads a solver: the caller hands in the power drive it computed from
//! the published voltages.

// A NaN time step or interval means no time has passed, so the guards below are
// written `!(x > 0.0)` on purpose; `x <= 0.0` would let a NaN through.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use super::ExpVoltage;

/// The held junction temperature's band (W1g Q4, point 4). `T_hold` moves only
/// when `T(t)` has drifted this far from it, which bounds the stale law's
/// current error at about 6e-4 relative.
pub const THERMAL_BAND_C: f64 = 0.01;

/// Classic's shutdown hysteresis thresholds (`profile.shutdown`, TP:526-538).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shutdown {
    pub trip_c: f64,
    pub restart_c: f64,
}

/// Everything about one identified device that never changes after load: the
/// thermal record (code 91) plus the ambient the program was loaded with.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermalConfig {
    pub ambient_c: f64,
    pub r_theta: f64,
    pub tau_s: f64,
    /// `None` for every W1 profile.
    pub shutdown: Option<Shutdown>,
    /// A plain `led` in a static or fixed-pair partition: its law follows
    /// `T_hold`, and a boundary is a slow event (point 4). A resistor, or an
    /// LED in a stepped partition (whose `T_hold` is a step-commit lag, no
    /// event), leaves this false.
    pub holds_junction: bool,
}

impl ThermalConfig {
    /// `T_inf = Ta + P * Rtheta`, in TP's operation order (`targetTemperatureC`).
    pub fn target_c(&self, power_w: f64) -> f64 {
        self.ambient_c + power_w * self.r_theta
    }
}

/// The one-pole close, classic's formula (TP:520-524) evaluated once per
/// interval: exact for a constant target, so splitting an interval changes the
/// result only by rounding. A zero or negative `dt` returns `temp0` untouched,
/// so closing at the same instant twice is a bit-exact no-op.
pub fn one_pole(temp0: f64, target_c: f64, dt: f64, tau_s: f64) -> f64 {
    if !(dt > 0.0) {
        return temp0;
    }
    relax(temp0, target_c, (-dt / tau_s).exp())
}

/// `target + (temp0 - target) * decay`, in the association of classic's
/// `thermal-physics.ts` (TP:520-524), `decay` being `exp(-dt/tau)`. Kept apart
/// from the `exp` so a test can pin its rounding against classic's own
/// expression without depending on a libm.
pub fn relax(temp0: f64, target_c: f64, decay: f64) -> f64 {
    target_c + (temp0 - target_c) * decay
}

/// `phi(x) = expm1(x) / x` with `phi(0) = 1`. `expm1` keeps it accurate for
/// tiny `x`, so there is no switch-over threshold anywhere in the forced
/// response.
fn phi(x: f64) -> f64 {
    if x == 0.0 {
        1.0
    } else {
        x.exp_m1() / x
    }
}

/// `psi(x) = (1 - exp(-x)) / x` with `psi(0) = 1`: `phi(-x)`, kept apart because
/// it is the overflow-safe factoring for `x >= 0` (see `mode_term`).
fn psi(x: f64) -> f64 {
    if x == 0.0 {
        1.0
    } else {
        -(-x).exp_m1() / x
    }
}

/// One exponential mode of the forced one-pole response,
/// `(u/tauS) * exp(-u/tauS) * phi(-delta_m * u)` with `delta_m = m/tauC - 1/tauS`
/// (README point 3), without the `Rtheta/R * c_m` factor.
///
/// `-delta_m * u` is `x = (tauC - m*tauS)/(tauC*tauS) * u`. The numerator is the
/// one place a near-resonant drive cancels, and for `tauC` within a factor of
/// two of `m*tauS` the subtraction is exact (Sterbenz), so `x` keeps full
/// relative precision however close to resonance it gets; the direct
/// `1/(1 - m*tauS/tauC)` form would lose it. `phi(x)` is then accurate to a few
/// ulp for every `x`, the resonant `x = 0` included.
///
/// `exp(-u/tauS) * phi(x)` overflows when `x` is large and positive (a drive
/// that decays far slower than the package, over a long segment: `expm1`
/// returns infinity before the decay can scale it back). For `x >= 0` it is
/// factored by the slower exponential instead, `exp(-m*u/tauC) * psi(x)`: the
/// same function, bounded. That is a branch on the sign of `x`, not a
/// tolerance on how close to resonance the drive is.
fn mode_term(m: f64, tau_s: f64, tau_c: f64, u: f64) -> f64 {
    let x = (tau_c - m * tau_s) / (tau_c * tau_s) * u;
    let lead = u / tau_s;
    if x >= 0.0 {
        lead * (-m * u / tau_c).exp() * psi(x)
    } else {
        lead * (-u / tau_s).exp() * phi(x)
    }
}

/// The forced response of the one-pole to `P(t) = gain * V(t)^2` with
/// `V(t) = a + b*exp(-(t - t_seg)/tau)` (point 3, closed-form partitions): the
/// temperature at `t` given `temp0` at `v.t_seg`. `gain` is `1/R`.
///
/// With `u = t - t_seg` and `c_1 = 2ab`, `c_2 = b^2`:
/// `T = Ta + (Rtheta*gain)*a^2*(1 - e^{-u/tauS}) + (T0 - Ta)*e^{-u/tauS}
///      + (Rtheta*gain) * sum_m c_m * mode_term(m)`.
/// Exact for the whole segment, and independent of how it is split.
pub fn forced_temperature(cfg: &ThermalConfig, temp0: f64, gain: f64, v: &ExpVoltage, t: f64) -> f64 {
    let u = t - v.t_seg;
    if !(u > 0.0) {
        return temp0;
    }
    // A voltage that does not relax is a constant power: the modes below divide
    // by `tauC` and would give NaN for `b == 0` with an infinite `tauC`.
    if let Some(volts) = v.constant_value() {
        return one_pole(temp0, cfg.target_c(gain * volts * volts), u, cfg.tau_s);
    }
    let rg = cfg.r_theta * gain;
    let x = u / cfg.tau_s;
    if v.ramp && u / v.tau <= RAMP_SERIES_MAX {
        return ramp_temperature(cfg, temp0, rg, v, u);
    }
    let decay = (-x).exp();
    let steady = rg * v.a * v.a * -(-x).exp_m1();
    let cross = rg * (2.0 * v.a * v.b) * mode_term(1.0, cfg.tau_s, v.tau, u);
    let square = rg * (v.b * v.b) * mode_term(2.0, cfg.tau_s, v.tau, u);
    cfg.ambient_c + steady + (temp0 - cfg.ambient_c) * decay + cross + square
}

/// A ramp's forced response ([`forced_temperature`]) is summed as a series in
/// `y = u/tau` up to this: past it the ramp has moved by at least `0.39·|b|`,
/// so `a` and `b` are the size of `V` and the exponential forms do not
/// cancel.
const RAMP_SERIES_MAX: f64 = 0.5;

/// [`forced_temperature`] for a ramp (`ExpVoltage`'s doc) with `u/tau` at
/// most [`RAMP_SERIES_MAX`]: with `V(s) = v0 + b m(s)`, `m = expm1(-s/tau)`,
/// `T = Ta + (T0 - Ta) e^(-x) + Rtheta gain (v0^2 K0 + 2 v0 b K1 + b^2 K2)`,
/// `K_j` the one-pole's response to `m^j` (`x = u/tauS`). Expanding `m` and
/// `m^2` in `(-y)^n / n!` (`m^2`'s coefficients `2^n - 2`) leaves the moments
/// `M_n = x ∫_0^1 e^(-x(1-t)) t^n dt` ([`one_pole_moments`]), each in `[0,
/// 1]`, so every term is the size of the rise it adds, however large `b`.
fn ramp_temperature(cfg: &ThermalConfig, temp0: f64, rg: f64, v: &ExpVoltage, u: f64) -> f64 {
    const TERMS: usize = 26;
    let x = u / cfg.tau_s;
    let y = u / v.tau;
    let m = one_pole_moments(x, TERMS);
    let (mut k1, mut k2, mut c, mut p2) = (0.0f64, 0.0f64, 1.0f64, 1.0f64);
    for (n, &mn) in m.iter().enumerate().skip(1) {
        c *= -y / n as f64;
        p2 *= 2.0;
        k1 += c * mn;
        if n >= 2 {
            k2 += (p2 - 2.0) * c * mn;
        }
    }
    cfg.ambient_c + (temp0 - cfg.ambient_c) * (-x).exp() + rg * (v.v0 * v.v0 * m[0] + 2.0 * v.v0 * v.b * k1 + v.b * v.b * k2)
}

/// `M_n = x I_n` for `n` in `0..count`, `I_n = ∫_0^1 e^(-x w) (1 - w)^n dw`:
/// the one-pole's response at `u` (`x = u/tauS`) to `(s/u)^n`. `I_n = (1 - n
/// I_(n-1))/x` is stable forward while `n < x` and backward, `I_(n-1) = (1 -
/// x I_n)/n`, otherwise; the backward run starts far enough up (`I_N` taken
/// as `1/(N + 1)`) that its error has shrunk below an ulp by `count`.
fn one_pole_moments(x: f64, count: usize) -> Vec<f64> {
    let mut i = vec![0.0f64; count];
    if !(x > 0.0) {
        return i;
    }
    if x >= count as f64 {
        i[0] = -(-x).exp_m1() / x;
        for n in 1..count {
            i[n] = (1.0 - n as f64 * i[n - 1]) / x;
        }
    } else {
        let top = 2 * count + 64;
        let mut next = 1.0 / (top as f64 + 1.0);
        for n in (1..=top).rev() {
            let prev = (1.0 - x * next) / n as f64;
            if n <= count {
                i[n - 1] = prev;
            }
            next = prev;
        }
        i[0] = -(-x).exp_m1() / x;
    }
    i.iter().map(|v| x * v).collect()
}

/// First time at or after `t0` that a one-pole response from `temp0` toward
/// `t_inf` reaches `level`, going up (`rising`) or down. `None` when `t_inf`
/// does not lie strictly beyond `level`: the response only approaches it.
fn crossing_time(t0: f64, temp0: f64, t_inf: f64, tau_s: f64, level: f64, rising: bool) -> Option<f64> {
    if rising {
        if temp0 >= level {
            return Some(t0);
        }
        if t_inf <= level {
            return None;
        }
    } else {
        if temp0 <= level {
            return Some(t0);
        }
        if t_inf >= level {
            return None;
        }
    }
    Some(t0 + tau_s * ((temp0 - t_inf) / (level - t_inf)).ln())
}

/// The thermal boundary (point 4): the instant `T` reaches `T_hold +/- band` on
/// the side `t_inf` lies, `t0 + tauS * ln((T0 - T_inf) / (T_hold +/- band -
/// T_inf))`. `None` while `|T_inf - T_hold|` does not exceed the band (the
/// response only approaches the edge, which the `ln` would turn into an
/// infinite time). A `T0` already outside the band is due at `t0`: it cannot
/// happen while every boundary is honoured, and latching at once is the safe
/// reading if rounding ever leaves it there.
pub fn boundary_time(t0: f64, temp0: f64, t_inf: f64, tau_s: f64, t_hold: f64) -> Option<f64> {
    if t_inf == t_hold {
        return None;
    }
    if (temp0 - t_hold).abs() > THERMAL_BAND_C {
        return Some(t0);
    }
    let rising = t_inf > t_hold;
    let level = if rising { t_hold + THERMAL_BAND_C } else { t_hold - THERMAL_BAND_C };
    crossing_time(t0, temp0, t_inf, tau_s, level, rising)
}

/// The time a package with a shutdown profile next changes state (TP:526-538):
/// `T` reaching `tripC` while running, or `restartC` while shut down. `None`
/// when it never does. No W1 profile has one.
pub fn shutdown_event_time(t0: f64, temp0: f64, t_inf: f64, tau_s: f64, shutdown_now: bool, sd: Shutdown) -> Option<f64> {
    if shutdown_now {
        crossing_time(t0, temp0, t_inf, tau_s, sd.restart_c, false)
    } else {
        crossing_time(t0, temp0, t_inf, tau_s, sd.trip_c, true)
    }
}

/// Classic's hysteresis on an evaluated temperature (TP:526-538), for a
/// stepped partition that evaluates once per step.
pub fn shutdown_after(shutdown_now: bool, temp_c: f64, sd: Shutdown) -> bool {
    if !shutdown_now && temp_c >= sd.trip_c {
        true
    } else if shutdown_now && temp_c <= sd.restart_c {
        false
    } else {
        shutdown_now
    }
}

/// What dissipates in the package over the interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PowerDrive {
    /// Static, fixed-pair, and one stepped step: `P` held.
    Constant(f64),
    /// A closed-form segment (resistors only): `P(t) = gain * V(t)^2`, `gain =
    /// 1/R`.
    Squared { gain: f64, v: ExpVoltage },
}

/// A device's lazy thermal integrator: `T` at `t0` and what drives it from
/// there (point 2), the held junction temperature and the shutdown flag.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermalEntry {
    pub cfg: ThermalConfig,
    pub temp0: f64,
    pub t0: f64,
    pub drive: PowerDrive,
    pub t_hold: f64,
    pub shutdown: bool,
}

impl ThermalEntry {
    /// A load starts every identified device at `T = T_hold = Ta`, dissipating
    /// nothing (point 15).
    pub fn new(cfg: ThermalConfig, t0: f64) -> Self {
        ThermalEntry { cfg, temp0: cfg.ambient_c, t0, drive: PowerDrive::Constant(0.0), t_hold: cfg.ambient_c, shutdown: false }
    }

    /// `T(t)` under the current drive, committing nothing: the readout (point
    /// 16) and the value `close` stores.
    pub fn temperature_at(&self, t: f64) -> f64 {
        match self.drive {
            PowerDrive::Constant(p) => one_pole(self.temp0, self.cfg.target_c(p), t - self.t0, self.cfg.tau_s),
            PowerDrive::Squared { gain, v } => forced_temperature(&self.cfg, self.temp0, gain, &v.rebased(self.t0), t),
        }
    }

    /// `P` of the current interval at `t`.
    pub fn power_at(&self, t: f64) -> f64 {
        match self.drive {
            PowerDrive::Constant(p) => p,
            PowerDrive::Squared { gain, v } => {
                let volts = v.at(t);
                gain * volts * volts
            }
        }
    }

    /// Brings the integrator exactly to `t` with what drove it. The drive is
    /// left in place; the caller replaces it with the one the new voltages give.
    pub fn close(&mut self, t: f64) {
        if t > self.t0 {
            self.temp0 = self.temperature_at(t);
            self.t0 = t;
        }
    }

    /// A thermal boundary (point 4, step 2): close to `t`, then hold the
    /// temperature reached.
    pub fn rearm(&mut self, t: f64) {
        self.close(t);
        self.t_hold = self.temp0;
    }

    /// The next thermal boundary, for a junction that follows `T_hold` under a
    /// constant power. A relaxing closed-form drive has none: only resistors
    /// are closed-form, and they have no junction law.
    pub fn boundary_time(&self) -> Option<f64> {
        if !self.cfg.holds_junction {
            return None;
        }
        let p = self.constant_power()?;
        boundary_time(self.t0, self.temp0, self.cfg.target_c(p), self.cfg.tau_s, self.t_hold)
    }

    /// The power when it does not vary: a constant drive, or a closed-form one
    /// over a voltage that does not relax.
    fn constant_power(&self) -> Option<f64> {
        match self.drive {
            PowerDrive::Constant(p) => Some(p),
            PowerDrive::Squared { gain, v } => v.constant_value().map(|volts| gain * volts * volts),
        }
    }

    /// The next shutdown or restart time under a constant power. A relaxing
    /// closed-form drive returns `None`: no W1 kind has both a shutdown and a closed-form
    /// partition, and `T` is not monotone there, so a crossing would need a
    /// root find the contract does not ask for.
    pub fn shutdown_time(&self) -> Option<f64> {
        let sd = self.cfg.shutdown?;
        let p = self.constant_power()?;
        shutdown_event_time(self.t0, self.temp0, self.cfg.target_c(p), self.cfg.tau_s, self.shutdown, sd)
    }

    /// The analytic shutdown event: close to `t` and flip the flag. The event
    /// is the transition, so `T` is not re-tested against the threshold (the
    /// crossing rule's twin: rounding would leave it one ulp short).
    pub fn fire_shutdown(&mut self, t: f64) {
        self.close(t);
        self.shutdown = !self.shutdown;
    }

    /// Classic's hysteresis on the closed `T`, for a stepped partition that
    /// evaluates once per step. Returns the new state on a transition.
    pub fn evaluate_shutdown(&mut self) -> Option<bool> {
        let sd = self.cfg.shutdown?;
        let next = shutdown_after(self.shutdown, self.temp0, sd);
        if next == self.shutdown {
            None
        } else {
            self.shutdown = next;
            Some(next)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TA: f64 = 25.0;

    fn resistor_cfg() -> ThermalConfig {
        // The resistor profile (TP:285-324): Rtheta 340, tauS 8.
        ThermalConfig { ambient_c: TA, r_theta: 340.0, tau_s: 8.0, shutdown: None, holds_junction: false }
    }

    fn led_cfg() -> ThermalConfig {
        // The LED profile: Rtheta 560, tauS 2.
        ThermalConfig { ambient_c: TA, r_theta: 560.0, tau_s: 2.0, shutdown: None, holds_junction: true }
    }

    /// Neumaier-compensated RK4 of the one-pole ODE with `P = gain * V^2`, to
    /// `n * h`: `dT/dt = (Ta + Rtheta*P(t) - T)/tauS`. Compensated so the
    /// reference's own rounding stays far below the 1e-9 the closed form is
    /// held to over a few million steps.
    fn rk4_forced(cfg: &ThermalConfig, temp0: f64, gain: f64, v: &ExpVoltage, n: u64, h: f64) -> f64 {
        let f = |t: f64, temp: f64| {
            let volts = v.at(t);
            (cfg.ambient_c + cfg.r_theta * gain * volts * volts - temp) / cfg.tau_s
        };
        let mut sum = temp0;
        let mut comp = 0.0;
        for i in 0..n {
            let t = v.t_seg + i as f64 * h;
            let temp = sum + comp;
            let k1 = f(t, temp);
            let k2 = f(t + 0.5 * h, temp + 0.5 * h * k1);
            let k3 = f(t + 0.5 * h, temp + 0.5 * h * k2);
            let k4 = f(t + h, temp + h * k3);
            let inc = h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
            let s = sum + inc;
            comp += if sum.abs() >= inc.abs() { (sum - s) + inc } else { (inc - s) + sum };
            sum = s;
        }
        sum + comp
    }

    /// The direct (cancelling) form of the forced response, valid and
    /// well-conditioned wherever `tauC` is far from `m*tauS`, and free of the
    /// overflow a long segment gives `phi`: an independent evaluation for the
    /// cases RK4 is too slow to reach.
    fn direct_forced(cfg: &ThermalConfig, temp0: f64, gain: f64, v: &ExpVoltage, t: f64) -> f64 {
        let u = t - v.t_seg;
        let rg = cfg.r_theta * gain;
        let ts = cfg.tau_s;
        let mut temp = cfg.ambient_c + rg * v.a * v.a * (1.0 - (-u / ts).exp()) + (temp0 - cfg.ambient_c) * (-u / ts).exp();
        for (m, c) in [(1.0, 2.0 * v.a * v.b), (2.0, v.b * v.b)] {
            let k = 1.0 / (1.0 - m * ts / v.tau);
            temp += rg * c * k * ((-m * u / v.tau).exp() - (-u / ts).exp());
        }
        temp
    }

    #[test]
    fn resistor_one_pole_matches_the_closed_form() {
        // 0.25 W through Rtheta 340 and tauS 8: T(4 s) = 110 - 85*exp(-0.5),
        // probe F `resHeat`'s analytic value.
        let cfg = resistor_cfg();
        let expected = 58.44489392442616;
        let target = cfg.target_c(0.25);
        assert_eq!(target, 110.0);
        let t = one_pole(TA, target, 4.0, cfg.tau_s);
        assert!((t - expected).abs() < 1e-9, "T(4 s) = {t}, expected {expected}");
    }

    #[test]
    fn splitting_an_interval_changes_t_only_by_rounding() {
        let cfg = resistor_cfg();
        let expected = 58.44489392442616;
        for pieces in [1u32, 4, 1000] {
            let mut entry = ThermalEntry::new(cfg, 0.0);
            entry.drive = PowerDrive::Constant(0.25);
            for i in 1..=pieces {
                entry.close(4.0 * f64::from(i) / f64::from(pieces));
            }
            let rel = (entry.temp0 - expected).abs() / expected;
            assert!(rel < 1e-12, "{pieces} intervals: T = {}, relative error {rel}", entry.temp0);
            assert_eq!(entry.t0, 4.0);
        }
    }

    #[test]
    fn closing_twice_at_one_instant_is_a_bit_exact_no_op() {
        let mut entry = ThermalEntry::new(resistor_cfg(), 0.0);
        entry.drive = PowerDrive::Constant(0.25);
        entry.close(1.5);
        let before = entry;
        entry.close(1.5);
        entry.close(1.0);
        assert_eq!(entry.temp0.to_bits(), before.temp0.to_bits());
        assert_eq!(entry.t0.to_bits(), before.t0.to_bits());
    }

    #[test]
    fn readout_projects_without_committing() {
        let mut entry = ThermalEntry::new(led_cfg(), 0.0);
        entry.drive = PowerDrive::Constant(0.038);
        let before = entry;
        let t = entry.temperature_at(3.0);
        assert!(t > TA);
        assert_eq!(entry, before, "a projection never writes");
        assert_eq!(entry.power_at(3.0), 0.038);
    }

    /// One forced-response case against the 1e-6 s RK4 reference. `tau_c` and
    /// `tau_s` are the real doubles the closed form sees.
    fn assert_forced_matches_rk4(tau_s: f64, tau_c: f64, a: f64, b: f64, temp0: f64, steps: u64) {
        let cfg = ThermalConfig { ambient_c: TA, r_theta: 340.0, tau_s, shutdown: None, holds_junction: false };
        let gain = 1.0 / 50.0;
        let v = ExpVoltage::new(a, b, tau_c, 0.0);
        let h = 1e-6;
        let t_end = steps as f64 * h;
        let reference = rk4_forced(&cfg, temp0, gain, &v, steps, h);
        let closed = forced_temperature(&cfg, temp0, gain, &v, t_end);
        assert!((reference - temp0).abs() > 1.0, "the case must move the temperature for the comparison to mean anything");
        assert!((closed - reference).abs() < 1e-9, "tauS {tau_s} tauC {tau_c}: closed {closed}, RK4 {reference}, diff {}", (closed - reference).abs());
    }

    #[test]
    fn forced_response_matches_rk4_off_resonance() {
        // A resistor settling from 5 V toward 3 V, then one crossing zero
        // (4 V toward -2 V): tauC faster and slower than the package.
        assert_forced_matches_rk4(0.5, 0.2, 3.0, 2.0, TA, 1_000_000);
        assert_forced_matches_rk4(0.5, 1.7, 3.0, 2.0, 60.0, 1_000_000);
        assert_forced_matches_rk4(0.5, 0.31, -2.0, 6.0, TA, 1_000_000);
    }

    #[test]
    fn forced_response_matches_rk4_at_the_resonant_cases_exactly() {
        // tauC = tauS makes delta_1 = 0, and tauC = 2*tauS makes delta_2 = 0.
        let tau_s = 0.5;
        assert_forced_matches_rk4(tau_s, tau_s, 3.0, 2.0, TA, 1_000_000);
        assert_forced_matches_rk4(tau_s, 2.0 * tau_s, 3.0, 2.0, TA, 1_000_000);
        assert_forced_matches_rk4(tau_s, tau_s, -2.0, 6.0, 70.0, 1_000_000);
        assert_forced_matches_rk4(tau_s, 2.0 * tau_s, -2.0, 6.0, 70.0, 1_000_000);
    }

    #[test]
    fn forced_response_matches_rk4_near_resonance() {
        // A direct 1/(1 - m*tauS/tauC) form loses 1e-16/offset of its term to
        // cancellation: 1e-4 relative at 1e-12. Each offset is tried against
        // both modes, on both sides.
        let tau_s = 0.5;
        for offset in [1e-3, 1e-6, 1e-9, 1e-12] {
            for sign in [1.0, -1.0] {
                let rel = 1.0 + sign * offset;
                assert_forced_matches_rk4(tau_s, tau_s * rel, 3.0, 2.0, TA, 400_000);
                assert_forced_matches_rk4(tau_s, 2.0 * tau_s * rel, 3.0, 2.0, TA, 400_000);
            }
        }
    }

    #[test]
    fn the_direct_form_loses_to_cancellation_where_the_closed_form_does_not() {
        // The README's warning, measured: at a relative offset of 1e-12 from
        // resonance, `1/(1 - m*tauS/tauC)` carries 1e-16/1e-12 = 1e-4 of error
        // into its term, far outside the 1e-9 the RK4 reference holds the
        // closed form to.
        let tau_s = 0.5;
        let tau_c = 2.0 * tau_s * (1.0 + 1e-12);
        let cfg = ThermalConfig { ambient_c: TA, r_theta: 340.0, tau_s, shutdown: None, holds_junction: false };
        let gain = 1.0 / 50.0;
        let v = ExpVoltage::new(3.0, 2.0, tau_c, 0.0);
        let steps = 400_000;
        let reference = rk4_forced(&cfg, TA, gain, &v, steps, 1e-6);
        let t_end = steps as f64 * 1e-6;
        let ours = forced_temperature(&cfg, TA, gain, &v, t_end);
        let direct = direct_forced(&cfg, TA, gain, &v, t_end);
        assert!((ours - reference).abs() < 1e-9, "closed form: {}", (ours - reference).abs());
        assert!((direct - reference).abs() > 1e-6, "direct form: {}", (direct - reference).abs());
    }

    #[test]
    fn near_resonant_response_is_continuous_through_the_resonant_form() {
        // The closed form at tauC = 2*tauS*(1 + e) must approach the resonant
        // value smoothly as e shrinks: differences scale with e, with no jump at
        // any offset.
        let cfg = ThermalConfig { ambient_c: TA, r_theta: 340.0, tau_s: 8.0, shutdown: None, holds_junction: false };
        let gain = 1.0 / 50.0;
        let at = |tau_c: f64| forced_temperature(&cfg, TA, gain, &ExpVoltage::new(3.0, 7.0, tau_c, 0.0), 5.0);
        let resonant = at(16.0);
        let mut previous = f64::INFINITY;
        for offset in [1e-3, 1e-5, 1e-7, 1e-9, 1e-11, 1e-13] {
            let diff = (at(16.0 * (1.0 + offset)) - resonant).abs();
            assert!(diff <= previous, "offset {offset}: {diff} after {previous}");
            assert!(diff < 1e3 * offset, "offset {offset}: {diff} is not O(offset)");
            previous = diff;
        }
    }

    #[test]
    fn forced_response_is_stable_and_correct_over_long_segments() {
        // Hours-long segments overflow expm1 in a naive phi: tauC far above
        // tauS puts x in the hundreds. Held against the independent direct
        // form, which is well-conditioned this far from resonance.
        let cfg = ThermalConfig { ambient_c: TA, r_theta: 340.0, tau_s: 2.0, shutdown: None, holds_junction: false };
        let gain = 1.0 / 50.0;
        for (tau_c, t_end) in [(1.0e4, 3600.0), (500.0, 3600.0), (7.0, 3600.0), (1.0e6, 1.0e5)] {
            let v = ExpVoltage::new(5.0, 5.0, tau_c, 0.0);
            let closed = forced_temperature(&cfg, TA, gain, &v, t_end);
            let direct = direct_forced(&cfg, TA, gain, &v, t_end);
            assert!(closed.is_finite(), "tauC {tau_c}: {closed}");
            assert!((closed - direct).abs() < 1e-9 * direct.abs().max(1.0), "tauC {tau_c}: closed {closed}, direct {direct}");
        }
    }

    #[test]
    fn forced_response_is_independent_of_where_the_segment_is_split() {
        // Closing through the entry in pieces equals one close: rebasing the
        // drive at each stop must not drift.
        let cfg = ThermalConfig { ambient_c: TA, r_theta: 340.0, tau_s: 8.0, shutdown: None, holds_junction: false };
        let drive = PowerDrive::Squared { gain: 1.0 / 50.0, v: ExpVoltage::new(2.0, 8.0, 1.3, 0.0) };
        let mut one = ThermalEntry::new(cfg, 0.0);
        one.drive = drive;
        one.close(6.0);
        let mut many = ThermalEntry::new(cfg, 0.0);
        many.drive = drive;
        for i in 1..=60 {
            many.close(0.1 * f64::from(i));
        }
        assert!((one.temp0 - many.temp0).abs() < 1e-10, "{} vs {}", one.temp0, many.temp0);
    }

    #[test]
    fn constant_and_squared_drives_agree_when_the_voltage_is_constant() {
        // b = 0 makes P constant: the forced response must reduce to the
        // one-pole exactly in form.
        let cfg = resistor_cfg();
        let gain = 1.0 / 50.0;
        let v = ExpVoltage::new(5.0, 0.0, 3.0, 0.0);
        let forced = forced_temperature(&cfg, TA, gain, &v, 4.0);
        let plain = one_pole(TA, cfg.target_c(gain * 25.0), 4.0, cfg.tau_s);
        assert!((forced - plain).abs() < 1e-12, "{forced} vs {plain}");
    }

    #[test]
    fn boundary_time_matches_the_closed_form() {
        // T0 = 25, T_inf = 46.299333650880065, tauS = 2, band 0.01: probe F's
        // `ledHeat` first boundary, t_b = 2*ln(21.2993.../21.2893...).
        let t_b = boundary_time(0.0, 25.0, 46.299333650880065, 2.0, 25.0).expect("a boundary");
        assert!((t_b - 9.3921700934e-4).abs() < 1e-14, "t_b = {t_b}");
        // Shifted by t0.
        let shifted = boundary_time(3.0, 25.0, 46.299333650880065, 2.0, 25.0).expect("a boundary");
        assert!((shifted - (3.0 + 9.3921700934e-4)).abs() < 1e-13);
    }

    #[test]
    fn boundary_time_lands_on_the_band_edge() {
        let (t0, temp0, t_inf, tau, hold) = (1.0, 30.0, 52.0, 2.0, 30.0);
        let t_b = boundary_time(t0, temp0, t_inf, tau, hold).expect("a boundary");
        let at = one_pole(temp0, t_inf, t_b - t0, tau);
        assert!((at - (hold + THERMAL_BAND_C)).abs() < 1e-12, "T(t_b) = {at}");
        let t_b = boundary_time(t0, temp0, 8.0, tau, hold).expect("the falling side");
        let at = one_pole(temp0, 8.0, t_b - t0, tau);
        assert!((at - (hold - THERMAL_BAND_C)).abs() < 1e-12, "T(t_b) = {at}");
    }

    #[test]
    fn boundary_time_is_none_inside_the_band_and_takes_the_falling_side() {
        // Inside the band, at its edge and exactly at the held value: the
        // response never leaves it.
        assert_eq!(boundary_time(0.0, 25.0, 25.005, 2.0, 25.0), None);
        assert_eq!(boundary_time(0.0, 25.0, 24.995, 2.0, 25.0), None);
        assert_eq!(boundary_time(0.0, 25.0, 25.0, 2.0, 25.0), None);
        assert_eq!(boundary_time(0.0, 25.0, 25.01, 2.0, 25.0), None, "the asymptote at the edge is never reached");
        // Cooling toward ambient from a hot held value: the lower edge.
        let t_b = boundary_time(0.0, 46.0, 25.0, 2.0, 46.0).expect("falling");
        let expected = 2.0 * ((46.0 - 25.0) / (46.0 - 0.01 - 25.0f64)).ln();
        assert!((t_b - expected).abs() < 1e-12, "{t_b} vs {expected}");
        // A T0 already outside the band (rounding) is due at once, on either
        // side: below the band while heading up it would otherwise be scheduled
        // for the far edge.
        assert_eq!(boundary_time(2.0, 25.5, 40.0, 2.0, 25.0), Some(2.0));
        assert_eq!(boundary_time(2.0, 24.0, 40.0, 2.0, 25.0), Some(2.0));
        assert_eq!(boundary_time(2.0, 26.0, 10.0, 2.0, 25.0), Some(2.0));
    }

    #[test]
    fn entry_boundary_follows_the_drive_and_the_hold() {
        let mut led = ThermalEntry::new(led_cfg(), 0.0);
        assert_eq!(led.boundary_time(), None, "nothing dissipates at load");
        led.drive = PowerDrive::Constant(0.038);
        let t_b = led.boundary_time().expect("rising");
        led.rearm(t_b);
        assert!((led.t_hold - (TA + THERMAL_BAND_C)).abs() < 1e-9, "T_hold = {}", led.t_hold);
        let next = led.boundary_time().expect("the next band");
        assert!(next > t_b);
        // A resistor never schedules a boundary.
        let mut resistor = ThermalEntry::new(resistor_cfg(), 0.0);
        resistor.drive = PowerDrive::Constant(0.25);
        assert_eq!(resistor.boundary_time(), None);
        // Nor does a closed-form drive.
        let mut forced = ThermalEntry::new(led_cfg(), 0.0);
        forced.drive = PowerDrive::Squared { gain: 0.1, v: ExpVoltage::new(3.0, 2.0, 1.0, 0.0) };
        assert_eq!(forced.boundary_time(), None);
    }

    #[test]
    fn shutdown_times_and_hysteresis() {
        // A regulator-like profile, as W3 will carry: trip 150, restart 130.
        let sd = Shutdown { trip_c: 150.0, restart_c: 130.0 };
        let cfg = ThermalConfig { ambient_c: TA, r_theta: 100.0, tau_s: 5.0, shutdown: Some(sd), holds_junction: false };
        let mut entry = ThermalEntry::new(cfg, 0.0);
        // 2 W settles at 225 C: it reaches 150 C at tauS*ln((25-225)/(150-225)).
        entry.drive = PowerDrive::Constant(2.0);
        let trip = entry.shutdown_time().expect("trips");
        let expected = 5.0 * (200.0f64 / 75.0).ln();
        assert!((trip - expected).abs() < 1e-12, "{trip} vs {expected}");
        // 1 W settles at 125 C, below tripC: never.
        entry.drive = PowerDrive::Constant(1.0);
        assert_eq!(entry.shutdown_time(), None);
        // The event is the transition, with no re-test.
        entry.drive = PowerDrive::Constant(2.0);
        entry.fire_shutdown(trip);
        assert!(entry.shutdown);
        assert!((entry.temp0 - 150.0).abs() < 1e-9);
        // Shut down and cooling toward ambient: restart at 130 C.
        entry.drive = PowerDrive::Constant(0.0);
        let restart = entry.shutdown_time().expect("restarts");
        let expected = trip + 5.0 * ((150.0 - 25.0) / (130.0 - 25.0f64)).ln();
        assert!((restart - expected).abs() < 1e-9, "{restart} vs {expected}");
        // Hysteresis between the two thresholds keeps the state.
        assert!(!shutdown_after(false, 149.9, sd));
        assert!(shutdown_after(false, 150.0, sd));
        assert!(shutdown_after(true, 140.0, sd));
        assert!(!shutdown_after(true, 130.0, sd));
        // A temperature already past the threshold is due at once.
        let hot = ThermalEntry { temp0: 155.0, t0: 7.0, shutdown: false, ..ThermalEntry::new(cfg, 7.0) };
        assert_eq!(hot.shutdown_time(), Some(7.0));
        // No profile, no event.
        assert_eq!(ThermalEntry::new(resistor_cfg(), 0.0).shutdown_time(), None);
    }

    #[test]
    fn evaluate_shutdown_applies_classics_hysteresis_per_step() {
        let sd = Shutdown { trip_c: 150.0, restart_c: 130.0 };
        let cfg = ThermalConfig { ambient_c: TA, r_theta: 100.0, tau_s: 5.0, shutdown: Some(sd), holds_junction: false };
        let mut entry = ThermalEntry::new(cfg, 0.0);
        entry.temp0 = 151.0;
        assert_eq!(entry.evaluate_shutdown(), Some(true));
        entry.temp0 = 140.0;
        assert_eq!(entry.evaluate_shutdown(), None);
        entry.temp0 = 129.0;
        assert_eq!(entry.evaluate_shutdown(), Some(false));
        assert_eq!(ThermalEntry::new(resistor_cfg(), 0.0).evaluate_shutdown(), None);
    }

    #[test]
    fn the_one_pole_keeps_classics_association() {
        // Classic's `target + (previous - target) * decay` (TP:520-524) run in
        // node with `decay = Math.exp(-dt/tau)`, whose value is passed in so
        // the pin does not depend on a libm. The blend and increment rewrites
        // each round differently on at least one of these.
        let cases: [(f64, f64, f64, f64); 4] = [
            (25.0, 110.0, 0.6065306597126334, 58.44489392442616),
            (25.0, 46.299333650880065, 0.9995001249791693, 25.010647004852412),
            (46.1619, 25.0, 0.19204990862075413, 29.064140961241538),
            (30.5, 52.8, 0.8311042838521256, 34.2663744700976),
        ];
        for (temp0, target, decay, expected) in cases {
            assert_eq!(relax(temp0, target, decay).to_bits(), expected.to_bits(), "{temp0} -> {target} at {decay}");
        }
        let blend = |t0: f64, tg: f64, d: f64| t0 * d + tg * (1.0 - d);
        let increment = |t0: f64, tg: f64, d: f64| t0 + (tg - t0) * (1.0 - d);
        for (name, variant) in [("blend", &blend as &dyn Fn(f64, f64, f64) -> f64), ("increment", &increment)] {
            assert!(cases.iter().any(|&(t0, tg, d, e)| variant(t0, tg, d).to_bits() != e.to_bits()), "{name}: no case discriminates it");
        }
        // `one_pole` is that formula over its own `exp`.
        assert_eq!(one_pole(25.0, 110.0, 4.0, 8.0).to_bits(), relax(25.0, 110.0, (-4.0f64 / 8.0).exp()).to_bits());
    }

    #[test]
    fn a_voltage_that_does_not_relax_is_a_constant_power() {
        // `b == 0` with `tau = INF` is INF/INF in the modes, and `tau = 0`
        // divides by zero: both returned NaN. `tau = INF` holds `a + b`, and
        // `tau = 0` holds `a`.
        let cfg = resistor_cfg();
        let gain = 1.0 / 50.0;
        let inf = f64::INFINITY;
        for (a, b, tau) in [(5.0, 0.0, inf), (5.0, 0.0, 0.0), (5.0, 0.0, 3.0), (3.0, 2.0, inf), (5.0, 2.0, 0.0)] {
            let v = ExpVoltage::new(a, b, tau, 0.0);
            let forced = forced_temperature(&cfg, TA, gain, &v, 4.0);
            let plain = one_pole(TA, cfg.target_c(gain * 25.0), 4.0, cfg.tau_s);
            assert!(forced.is_finite(), "a {a} b {b} tau {tau}: {forced}");
            assert_eq!(forced.to_bits(), plain.to_bits(), "a {a} b {b} tau {tau}");
            assert_eq!(v.at(1.5), 5.0);
        }
        // Through the entry, from a later t0 (the drive is rebased there), the
        // readout and the boundary agree with the equivalent constant drive.
        let led = ThermalConfig { holds_junction: true, ..led_cfg() };
        let power = gain * 25.0;
        for (a, b, tau) in [(5.0, 0.0, inf), (5.0, 0.0, 0.0), (3.0, 2.0, inf)] {
            let mut squared = ThermalEntry::new(led, 0.0);
            squared.drive = PowerDrive::Squared { gain, v: ExpVoltage::new(a, b, tau, 0.5) };
            let mut constant = ThermalEntry::new(led, 0.0);
            constant.drive = PowerDrive::Constant(power);
            squared.close(1.0);
            constant.close(1.0);
            let t = squared.temperature_at(3.0);
            assert!(t.is_finite());
            assert_eq!(t.to_bits(), constant.temperature_at(3.0).to_bits());
            assert_eq!(squared.power_at(2.0), power);
            assert_eq!(squared.boundary_time().map(f64::to_bits), constant.boundary_time().map(f64::to_bits));
            assert!(squared.boundary_time().is_some());
        }
        // A relaxing closed-form drive still has no boundary.
        let mut relaxing = ThermalEntry::new(led, 0.0);
        relaxing.drive = PowerDrive::Squared { gain, v: ExpVoltage::new(3.0, 2.0, 1.0, 0.0) };
        assert_eq!(relaxing.boundary_time(), None);
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Shutdown {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trip_c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.restart_c, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for ThermalConfig {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ambient_c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.r_theta, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.tau_s, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shutdown, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds_junction, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PowerDrive {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            PowerDrive::Constant(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            PowerDrive::Squared { gain, v } => crate::snapshot::HeapBytes::heap_bytes(gain, w) + crate::snapshot::HeapBytes::heap_bytes(v, w),
        }
    }
}

impl crate::snapshot::HeapBytes for ThermalEntry {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cfg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.temp0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drive, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t_hold, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shutdown, w); }
        n
    }
}
