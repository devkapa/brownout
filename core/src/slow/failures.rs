//! Per-kind failure drives and records (G2a): core/README.md, W1g kernel
//! semantics, points 6-8 and 12; "C ABI", `bo_engine_records`.
//!
//! Pure functions, no solver types. G1's stress integrators hold the dwell
//! state and take `(rate, recovery, threshold)` from a [`Drive`]; the solver
//! feeds each drive the voltages it already holds (point 1: slow physics
//! computes its own currents and never reads `elementI`).
//!
//! Every rate is written in classic's operation order, as the contract asks
//! ("Rust computes each rate in classic's operation order", point 5), so the
//! rate at a given set of voltages is the one classic's `recordAccumulatedStress`
//! integrates: `commitResistiveOverloadStress` (PA:95-159), `fuseModel.
//! updateFailures` (PA:704-725) and `commitLedOvercurrentStress` (SC:472-511),
//! each as fixed by brownout 0.6.1 (`fe9b38a`, W1g Q3).

/// Dwell thresholds (SE:120-122; "Kernel semantics: thermal, failures and
/// warnings (W1g)", constants table).
pub const RESISTOR_DAMAGE_THRESHOLD: f64 = 1.0;
pub const FUSE_I2T_THRESHOLD: f64 = 0.3;
pub const LED_DAMAGE_THRESHOLD: f64 = 24.0;

/// Bleed rates, per second, while a device is under its rating (point 5).
pub const RESISTOR_RECOVERY: f64 = 1.0;
pub const FUSE_RECOVERY: f64 = 0.3;
pub const LED_RECOVERY: f64 = 2.0;

/// The failure codes of `bo_engine_records` channels 0 and 3. `OutputSag` is
/// a failure kind, not a warning (Q-O44); it has no stress integrator, only
/// the window of `digital::sag`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailureKind {
    ResistorOverload = 1,
    FuseTripped = 2,
    LedFailed = 3,
    OutputSag = 4,
}

impl FailureKind {
    pub fn code(self) -> f64 {
        self as u32 as f64
    }

    pub fn from_code(code: f64) -> Option<FailureKind> {
        [FailureKind::ResistorOverload, FailureKind::FuseTripped, FailureKind::LedFailed, FailureKind::OutputSag].into_iter().find(|k| k.code() == code)
    }
}

/// One `bo_engine_records` record is 7 f64: `device, code, pin, since, value,
/// limit, aux`.
pub const RECORD_LEN: usize = 7;

/// A failure record before it is written to the ABI buffer. `pin` is the
/// pin's index in the device's pin records (for an IC, `vcc` and `gnd` are 0
/// and 1), or `None` for classic's absent `pinId`, which the buffer encodes as
/// `-1`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FailureRecord {
    pub device: usize,
    pub kind: FailureKind,
    pub pin: Option<usize>,
    pub since: f64,
    pub value: f64,
    pub limit: f64,
    /// The `output_sag` direction (`1` HIGH, `0` LOW), `0` for every other
    /// kind.
    pub aux: f64,
}

impl FailureRecord {
    pub fn to_f64s(&self) -> [f64; RECORD_LEN] {
        let pin = self.pin.map_or(-1.0, |p| p as f64);
        [self.device as f64, self.kind.code(), pin, self.since, self.value, self.limit, self.aux]
    }

    pub fn push_to(&self, out: &mut Vec<f64>) {
        out.extend_from_slice(&self.to_f64s());
    }
}

/// What one integrated kind's stress integrator runs on at one set of
/// voltages (point 5), and what its record would carry if it latched there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drive {
    pub kind: FailureKind,
    /// `S'` while `rate > 0`, per second; at 0 the integrator bleeds at
    /// `recovery` instead.
    pub rate: f64,
    pub recovery: f64,
    pub threshold: f64,
    /// The record's `value`: the power, the current or `peakI` at the latch
    /// instant (points 6-8).
    pub value: f64,
    /// The record's `limit`: `pMax`, `iRating` or `ifMax`.
    pub limit: f64,
}

impl Drive {
    /// The record this drive writes if it latches at `since`: `pin = -1` and
    /// `aux = 0`, as every integrated kind does (points 6-8).
    pub fn record(&self, device: usize, since: f64) -> FailureRecord {
        FailureRecord { device, kind: self.kind, pin: None, since, value: self.value, limit: self.limit, aux: 0.0 }
    }
}

/// `max(0, P/pMax - 1)`, with `P/pMax` read as 0 unless `pMax > 0`
/// (PA:136-148, `powerRatio`).
pub fn resistor_rate(power: f64, p_max: f64) -> f64 {
    let ratio = if p_max > 0.0 { power / p_max } else { 0.0 };
    (ratio - 1.0).max(0.0)
}

/// `max(0, (x/limit)^2 - 1)` with the ratio read as 0 unless `limit > 0`: the
/// fuse's rate (PA:715-720) and the LED's (SC:492-496) share this shape.
fn squared_ratio_rate(x: f64, limit: f64) -> f64 {
    let ratio = if limit > 0.0 { x / limit } else { 0.0 };
    (ratio * ratio - 1.0).max(0.0)
}

/// A fuse's rate for the current `current` through it: `0` when `iRating <=
/// 0`, which classic reads as no limit.
pub fn fuse_rate(current: f64, i_rating: f64) -> f64 {
    squared_ratio_rate(current, i_rating)
}

/// An LED's rate for `peakI`: `0` when `ifMax <= 0`.
pub fn led_rate(peak_i: f64, if_max: f64) -> f64 {
    squared_ratio_rate(peak_i, if_max)
}

/// A resistor's dissipation as classic computes it for the failure hook:
/// `I = |dv / R|` with `R` the stamped resistance, then `I * I * rPower`, and
/// `0` when `rPower <= 0` (PA:106-109). `rPower` is the Rating record's
/// `Number(params.resistance ?? 1000)`, which is not floored the way `R` is.
pub fn resistor_power(dv: f64, r: f64, r_power: f64) -> f64 {
    let current = (dv / r).abs();
    if r_power > 0.0 {
        current * current * r_power
    } else {
        0.0
    }
}

/// A fuse's current as classic's failure hook reads it: `|(va - vc) / rNormal|`
/// (PA:701-710, 716).
pub fn fuse_current(dv: f64, r_normal: f64) -> f64 {
    (dv / r_normal).abs()
}

/// Ruling R90 (a): the ratio `q` a resistor's rate is `q - 1` of, `P/pMax`
/// (-1 where it has no rating). The step mean and the stepped stress probe
/// both read it here.
pub fn resistor_q(dv: f64, r: f64, r_power: f64, p_max: f64) -> f64 {
    if p_max > 0.0 { resistor_power(dv, r, r_power) / p_max } else { -1.0 }
}

/// A fuse's ratio `(I/iRating)^2` (-1 with no rating).
pub fn fuse_q(dv: f64, r_normal: f64, i_rating: f64) -> f64 {
    if i_rating > 0.0 { (fuse_current(dv, r_normal) / i_rating).powi(2) } else { -1.0 }
}

/// An LED's ratio `(peakI/ifMax)^2` (-1 with no rating).
pub fn led_q(peak_i: f64, if_max: f64) -> f64 {
    if if_max > 0.0 { (peak_i / if_max).powi(2) } else { -1.0 }
}

/// `resistor_overload` for a `resistor` across `dv = V(pins[0]) - V(pins[1])`
/// (point 6). `r` is the resistor record's stamped `R`, `r_power` and `p_max`
/// the Rating record's values. Record: `value = P`, `limit = pMax`.
pub fn resistor_overload(dv: f64, r: f64, r_power: f64, p_max: f64) -> Drive {
    let power = resistor_power(dv, r, r_power);
    Drive {
        kind: FailureKind::ResistorOverload,
        rate: resistor_rate(power, p_max),
        recovery: RESISTOR_RECOVERY,
        threshold: RESISTOR_DAMAGE_THRESHOLD,
        value: power,
        limit: p_max,
    }
}

/// `fuse_tripped` for an intact fuse across `dv` (point 8). Record: `value =
/// I`, `limit = iRating`.
pub fn fuse_tripped(dv: f64, r_normal: f64, i_rating: f64) -> Drive {
    let current = fuse_current(dv, r_normal);
    Drive {
        kind: FailureKind::FuseTripped,
        rate: fuse_rate(current, i_rating),
        recovery: FUSE_RECOVERY,
        threshold: FUSE_I2T_THRESHOLD,
        value: current,
        limit: i_rating,
    }
}

/// `led_failed` for `peak_i` from `devices::led::peak_current` (point 7).
/// Record: `value = peakI`, `limit = ifMax`.
pub fn led_failed(peak_i: f64, if_max: f64) -> Drive {
    Drive {
        kind: FailureKind::LedFailed,
        rate: led_rate(peak_i, if_max),
        recovery: LED_RECOVERY,
        threshold: LED_DAMAGE_THRESHOLD,
        value: peak_i,
        limit: if_max,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- resistor_overload ---------------------------------------------------

    #[test]
    fn a_resistor_at_two_times_its_rating_has_rate_one() {
        // 5 V across 50 ohm is 100 mA and 0.5 W: twice a 0.25 W rating, so
        // P/pMax - 1 = 1 (W1g point 6; probe F `resistor2x`).
        let d = resistor_overload(5.0, 50.0, 50.0, 0.25);
        assert_eq!(d.kind, FailureKind::ResistorOverload);
        assert!((d.rate - 1.0).abs() < 1e-12, "rate {}", d.rate);
        assert!((d.value - 0.5).abs() < 1e-12, "P {}", d.value);
        assert_eq!(d.limit, 0.25);
        assert_eq!((d.recovery, d.threshold), (1.0, 1.0));
        // Latches at S = threshold / rate = 1 s: probe F `resistor2x`'s analytic time.
        assert!((d.threshold / d.rate - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_resistor_with_no_resistance_param_overloads_on_its_one_kilohm_default() {
        // Q-O27 (`fe9b38a`): no `resistance` param means R = 1000 and rPower =
        // 1000. 50 V across it is 50 mA and 2.5 W; at p_max 0.25 that is ten
        // times the rating, rate 9 (probe F `resistorNoParam`: 2.5 W).
        let d = resistor_overload(50.0, 1000.0, 1000.0, 0.25);
        assert!((d.value - 2.5).abs() < 1e-12, "P {}", d.value);
        assert!((d.rate - 9.0).abs() < 1e-12, "rate {}", d.rate);
    }

    #[test]
    fn rpower_below_the_stamped_floor_is_used_as_is() {
        // `resistance` 0.0005: the stamp floors R at 1e-3 but the Rating
        // record's rPower is the raw 5e-4 (PA:107). `I` comes from the stamped
        // R, and `I^2 * rPower` uses the raw one: 1 V across 1 mohm is 1000 A,
        // and (1000 A)^2 * 0.5 mohm is 500 W, where a floored rPower gives 1000 W.
        let d = resistor_overload(1.0, 1e-3, 5e-4, 1.0);
        assert!((d.value - 500.0).abs() < 1e-9, "P {}", d.value);
        assert!((d.rate - 499.0).abs() < 1e-9, "rate {}", d.rate);
    }

    #[test]
    fn a_non_positive_rpower_or_pmax_gives_rate_zero() {
        assert_eq!(resistor_overload(5.0, 50.0, 0.0, 0.25).rate, 0.0);
        assert_eq!(resistor_overload(5.0, 50.0, 0.0, 0.25).value, 0.0);
        assert_eq!(resistor_overload(5.0, 50.0, -3.0, 0.25).rate, 0.0);
        assert_eq!(resistor_overload(5.0, 50.0, 50.0, 0.0).rate, 0.0);
        assert_eq!(resistor_overload(5.0, 50.0, 50.0, -1.0).rate, 0.0);
    }

    #[test]
    fn a_resistor_under_its_rating_has_rate_zero_not_negative() {
        let d = resistor_overload(1.0, 100.0, 100.0, 0.25);
        assert_eq!(d.rate, 0.0);
    }

    #[test]
    fn the_resistor_current_is_signed_away() {
        assert_eq!(resistor_overload(-5.0, 50.0, 50.0, 0.25), resistor_overload(5.0, 50.0, 50.0, 0.25));
    }

    // ---- fuse_tripped --------------------------------------------------------

    #[test]
    fn a_fuse_at_two_times_its_rating_has_rate_three() {
        // 1 A through a 0.5 A fuse: (1/0.5)^2 - 1 = 3, so the 0.3 threshold is
        // reached at 0.1 s (point 8; probe F `fuse2x`). 0.1 ohm: 0.1 V across.
        let d = fuse_tripped(0.1, 0.1, 0.5);
        assert!((d.value - 1.0).abs() < 1e-12, "I {}", d.value);
        assert_eq!(d.limit, 0.5);
        assert!((d.rate - 3.0).abs() < 1e-12, "rate {}", d.rate);
        assert_eq!((d.recovery, d.threshold), (0.3, 0.3));
        assert!((d.threshold / d.rate - 0.1).abs() < 1e-12);
    }

    #[test]
    fn a_fuse_carrying_current_backwards_records_the_magnitude() {
        // The rate squares the ratio, so only the record's `value` shows the
        // sign: classic reads `|elementCurrent|` (PA:716).
        let d = fuse_tripped(-0.1, 0.1, 0.5);
        assert!((d.value - 1.0).abs() < 1e-12, "value {}", d.value);
        assert!((d.rate - 3.0).abs() < 1e-12, "rate {}", d.rate);
        assert_eq!(d.record(0, 0.1).value, d.value);
        assert!(fuse_current(-0.1, 0.1) > 0.0);
    }

    #[test]
    fn a_fuse_with_a_non_positive_rating_has_rate_zero() {
        assert_eq!(fuse_tripped(0.1, 0.1, 0.0).rate, 0.0);
        assert_eq!(fuse_tripped(0.1, 0.1, -1.0).rate, 0.0);
        // The record still carries the rating it was given.
        assert_eq!(fuse_tripped(0.1, 0.1, 0.0).limit, 0.0);
    }

    #[test]
    fn a_fuse_at_exactly_its_rating_is_at_rate_zero() {
        assert_eq!(fuse_tripped(0.05, 0.1, 0.5).rate, 0.0);
    }

    #[test]
    fn the_fuse_record_has_value_one_and_limit_half() {
        let r = fuse_tripped(0.1, 0.1, 0.5).record(3, 0.1);
        assert_eq!(r.to_f64s(), [3.0, 2.0, -1.0, 0.1, r.value, 0.5, 0.0]);
        assert!((r.value - 1.0).abs() < 1e-12);
    }

    // ---- led_failed ----------------------------------------------------------

    #[test]
    fn the_led_overdrive_of_probe_f_has_rate_8_867040() {
        // Probe F `ledOver`: a red LED, if_max 0.02, on 50 ohm from 5 V carries
        // 62.82369 mA (the model's own closed form: 1.8588155 V across the
        // junction). r = 3.1411845, rate = r^2 - 1 = 8.867040.
        let d = led_failed(0.06282369, 0.02);
        assert!((d.rate - 8.867040).abs() < 1e-6, "rate {}", d.rate);
        assert_eq!((d.recovery, d.threshold), (2.0, 24.0));
        assert_eq!((d.value, d.limit), (0.06282369, 0.02));
    }

    #[test]
    fn a_non_positive_ifmax_gives_led_rate_zero() {
        assert_eq!(led_failed(0.1, 0.0).rate, 0.0);
        assert_eq!(led_failed(0.1, -0.02).rate, 0.0);
    }

    #[test]
    fn an_led_at_or_under_its_rating_has_rate_zero() {
        assert_eq!(led_failed(0.02, 0.02).rate, 0.0);
        assert_eq!(led_failed(0.0, 0.02).rate, 0.0);
    }

    // ---- records -------------------------------------------------------------

    #[test]
    fn the_failure_codes_are_one_to_four() {
        assert_eq!(FailureKind::ResistorOverload.code(), 1.0);
        assert_eq!(FailureKind::FuseTripped.code(), 2.0);
        assert_eq!(FailureKind::LedFailed.code(), 3.0);
        assert_eq!(FailureKind::OutputSag.code(), 4.0);
        for kind in [FailureKind::ResistorOverload, FailureKind::FuseTripped, FailureKind::LedFailed, FailureKind::OutputSag] {
            assert_eq!(FailureKind::from_code(kind.code()), Some(kind));
        }
        assert_eq!(FailureKind::from_code(0.0), None);
        assert_eq!(FailureKind::from_code(5.0), None);
    }

    #[test]
    fn an_integrated_record_has_pin_minus_one_and_aux_zero() {
        let d = led_failed(0.06282369, 0.02);
        let r = d.record(7, 2.7066528);
        // device, code, pin, since, value, limit, aux (bo_engine_records).
        assert_eq!(r.to_f64s(), [7.0, 3.0, -1.0, 2.7066528, 0.06282369, 0.02, 0.0]);
        let r = resistor_overload(5.0, 50.0, 50.0, 0.25).record(1, 1.0);
        assert_eq!(r.to_f64s()[..3], [1.0, 1.0, -1.0]);
        assert_eq!(r.to_f64s()[5], 0.25);
    }

    #[test]
    fn a_record_with_a_pin_writes_its_index() {
        let r = FailureRecord { device: 2, kind: FailureKind::OutputSag, pin: Some(3), since: 1e-8, value: 1.9512195, limit: 2.0, aux: 1.0 };
        assert_eq!(r.to_f64s(), [2.0, 4.0, 3.0, 1e-8, 1.9512195, 2.0, 1.0]);
        let mut buf = vec![9.0];
        r.push_to(&mut buf);
        assert_eq!(buf.len(), 1 + RECORD_LEN);
        assert_eq!(buf[1..], r.to_f64s());
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for FailureKind {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            FailureKind::ResistorOverload => 0,
            FailureKind::FuseTripped => 0,
            FailureKind::LedFailed => 0,
            FailureKind::OutputSag => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for FailureRecord {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pin, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.since, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.limit, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aux, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Drive {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rate, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.recovery, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.threshold, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.limit, w); }
        n
    }
}
