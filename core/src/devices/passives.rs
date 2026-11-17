//! The resistor, capacitor, fuse and PTC fuse (core/README.md's kind-code
//! table; spec §7.3, §7.4; W1g's Fuse and PTC records). The resistor's
//! `stamp`/`current` were moved from `engine.rs`'s
//! inline `ResistorStamp` handling with no behaviour change: they produce
//! exactly the `g = 1/R` conductance and `(Va-Vb)/R` current that used to be
//! written directly into `StaticPartitionSolver::build`/`recompute_currents`.
//!
//! The capacitor (W1b step 2, plan row B2/B3) is different in kind from
//! every other device this module or `devices::mod`'s generic `stamp`/
//! `current` dispatch handles: its BE/trap companion (spec §7.4) is a
//! function of DYNAMIC history (the step size `h`, the last COMMITTED
//! terminal voltage `vc`, and — for trap — the last ACCEPTED series current)
//! that the generic `stamp(kind, pin_count, params)`/`current(kind,
//! pin_count, params, pin_voltages)` signatures have no way to carry (they
//! only ever see a device's fixed, load-time `params`). So the capacitor is
//! deliberately NOT added to that dispatch for its companion: `solve::stepped`
//! calls [`capacitor_be_companion`]/[`capacitor_trap_companion`] directly,
//! with the history it alone owns, and `solve::closed_form` does not use a
//! companion at all (it solves the exact continuous relationship instead,
//! spec §7.2.3/§7.4's closed form). Only the capacitor's LEAKAGE — a plain,
//! params-only shunt conductance, exactly like a resistor — is exposed
//! through the ordinary dispatch ([`leakage_stamp`], wired into
//! `devices::stamp` for kind `CAPACITOR`), so both dynamic solvers' shared
//! linear-base stamping (`solve::stamp_linear_base`) can fold it in the same
//! way they fold a resistor.

use crate::devices::Contribution;

/// `g = 1/R` between `pins[0]` and `pins[1]` (positional; core/README.md's
/// record-layout table). `kinds::resistor_check` already guarantees
/// `params[0] > 0` for anything that reaches here.
pub fn stamp(params: &[f64]) -> Vec<Contribution> {
    vec![Contribution::conductance(0, 1, 1.0 / params[0])]
}

/// `(Va-Vb)/R`, spec §7.3.
pub fn current(params: &[f64], pin_voltages: &[f64]) -> f64 {
    (pin_voltages[0] - pin_voltages[1]) / params[0]
}

/// A resistor's stamp with the W1g failed flag: a latched `resistor_overload`
/// is an open, not a large resistance (W1g point 12;
/// `failure-states.test.ts:69-96`), so it stamps nothing.
pub fn resistor_stamp(params: &[f64], failed: bool) -> Vec<Contribution> {
    if failed {
        Vec::new()
    } else {
        stamp(params)
    }
}

/// A resistor's `elementI` with the failed flag: `0` once latched (PA:191-194).
pub fn resistor_current(params: &[f64], pin_voltages: &[f64], failed: bool) -> f64 {
    if failed {
        0.0
    } else {
        current(params, pin_voltages)
    }
}

/// `fuse` params, Fuse record: `[rNormal, iRating]`. `rNormal >= 0.001` is
/// the reader's check, so the stamp never divides by zero.
pub mod fuse_param {
    pub const R_NORMAL: usize = 0;
    pub const I_RATING: usize = 1;
}

/// `ptc_fuse` params, PTC record: `[rNormal, iHold]`.
pub mod ptc_param {
    pub const R_NORMAL: usize = 0;
    pub const I_HOLD: usize = 1;
}

/// A tripped PTC is `rNormal * 200`: high enough to limit a fault, but a PTC
/// goes high-R rather than open (PA:757-759).
pub const PTC_TRIPPED_FACTOR: f64 = 200.0;

/// An intact fuse is `1/rNormal` between `pins[0]` and `pins[1]`; a tripped
/// one stamps nothing (PA:697-699), an open like a failed resistor's.
pub fn fuse_stamp(params: &[f64], failed: bool) -> Vec<Contribution> {
    if failed {
        Vec::new()
    } else {
        vec![Contribution::conductance(0, 1, 1.0 / params[fuse_param::R_NORMAL])]
    }
}

/// `(V(pins[0]) - V(pins[1])) / rNormal`, and `0` once `fuse_tripped` has
/// latched (PA:701-710).
pub fn fuse_current(params: &[f64], pin_voltages: &[f64], failed: bool) -> f64 {
    if failed {
        0.0
    } else {
        (pin_voltages[0] - pin_voltages[1]) / params[fuse_param::R_NORMAL]
    }
}

/// A PTC's resistance: `rNormal`, or `rNormal * 200` while tripped, the
/// product taken first as classic stamps it (PA:757-759), so `1/R` rounds the
/// way classic's does.
pub fn ptc_resistance(r_normal: f64, tripped: bool) -> f64 {
    if tripped {
        r_normal * PTC_TRIPPED_FACTOR
    } else {
        r_normal
    }
}

/// A PTC's stamp, `1/R` between its pins (PA:757-767). It always stamps: a
/// trip is a resistance change, never an open.
pub fn ptc_stamp(params: &[f64], tripped: bool) -> Vec<Contribution> {
    vec![Contribution::conductance(0, 1, 1.0 / ptc_resistance(params[ptc_param::R_NORMAL], tripped))]
}

/// `(V(pins[0]) - V(pins[1])) / R` with `R` as [`ptc_resistance`] (PA:764-772).
pub fn ptc_current(params: &[f64], pin_voltages: &[f64], tripped: bool) -> f64 {
    (pin_voltages[0] - pin_voltages[1]) / ptc_resistance(params[ptc_param::R_NORMAL], tripped)
}

/// Capacitor params, core/README.md's record-layout table: `[C, esr,
/// leakage]`. `kinds::capacitor_check` already guarantees `C > 0` and
/// `esr, leakage >= 0`.
fn capacitor_params(params: &[f64]) -> (f64, f64, f64) {
    (params[0], params[1], params[2])
}

/// The capacitor's leakage shunt (spec §7.4: "leakage across the
/// terminals"), as a plain conductance between `pins[0]` and `pins[1]` —
/// `leakage == 0` is the explicit code for "none" (classic's infinite
/// parallel resistance, core/README.md's record-layout table), so it stamps
/// nothing at all rather than a zero-conductance no-op term.
pub fn leakage_stamp(params: &[f64]) -> Vec<Contribution> {
    let (_, _, leakage) = capacitor_params(params);
    if leakage > 0.0 {
        vec![Contribution::conductance(0, 1, 1.0 / leakage)]
    } else {
        Vec::new()
    }
}

/// The backward-Euler companion (spec §7.4): `G = 1/(esr + h/C)`, `Ihist =
/// G*vcPrev`. Returned as a [`Contribution`] under the convention
/// `solve::stamp_two_terminal`/`solve::branch_current` share with every other
/// two-terminal stamp in this crate — "the branch current flowing `pin_a ->
/// pin_b` is `conductance*(va-vb) + injection`" — so `injection` here is
/// `-Ihist`, matching the spec's own series-current formula `i = G*v -
/// Ihist` exactly (`conductance*(va-vb) + injection == G*v - Ihist`).
pub fn capacitor_be_companion(params: &[f64], h: f64, vc_prev: f64) -> Contribution {
    let (c, esr, _) = capacitor_params(params);
    let g = 1.0 / (esr + h / c);
    Contribution { pin_a: 0, pin_b: 1, conductance: g, injection: -(g * vc_prev) }
}

/// The trapezoidal companion (spec §7.4): `G = 1/(esr + h/2C)`, `Ihist =
/// G*(vcPrev + (h/2C)*iPrev)`, where `iPrev` MUST be the last ACCEPTED series
/// current (spec: "`iPrev` must be the last *accepted* series current"), not
/// a trial value from a rejected step-doubling half-step. Same sign
/// convention as [`capacitor_be_companion`].
pub fn capacitor_trap_companion(params: &[f64], h: f64, vc_prev: f64, i_prev: f64) -> Contribution {
    let (c, esr, _) = capacitor_params(params);
    let half_h_over_c = h / (2.0 * c);
    let g = 1.0 / (esr + half_h_over_c);
    let ihist = g * (vc_prev + half_h_over_c * i_prev);
    Contribution { pin_a: 0, pin_b: 1, conductance: g, injection: -ihist }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamps_the_reciprocal_conductance() {
        let s = stamp(&[2000.0]);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].pin_a, 0);
        assert_eq!(s[0].pin_b, 1);
        assert!((s[0].conductance - 0.0005).abs() < 1e-15);
        assert_eq!(s[0].injection, 0.0);
    }

    #[test]
    fn current_is_ohms_law() {
        let i = current(&[1000.0], &[5.0, 1.0]);
        assert!((i - 0.004).abs() < 1e-15);
    }

    // ---- W1g: failed resistor, fuse, PTC fuse ---------------------------------

    #[test]
    fn a_failed_resistor_stamps_nothing_and_carries_no_current() {
        assert_eq!(resistor_stamp(&[2000.0], false), stamp(&[2000.0]));
        assert!(resistor_stamp(&[2000.0], true).is_empty(), "an open, not a large resistance");
        assert_eq!(resistor_current(&[1000.0], &[5.0, 1.0], false), current(&[1000.0], &[5.0, 1.0]));
        assert_eq!(resistor_current(&[1000.0], &[5.0, 1.0], true), 0.0);
    }

    #[test]
    fn an_intact_fuse_is_its_normal_resistance_and_a_tripped_one_is_open() {
        // rNormal 0.1 ohm, 0.1 V across: 1 A (probe F `fuse2x`'s fuse).
        let params = [0.1, 0.5];
        let s = fuse_stamp(&params, false);
        assert_eq!(s.len(), 1);
        assert_eq!((s[0].pin_a, s[0].pin_b, s[0].injection), (0, 1, 0.0));
        assert_eq!(s[0].conductance, 1.0 / 0.1);
        assert!((fuse_current(&params, &[0.1, 0.0], false) - 1.0).abs() < 1e-12);
        assert!((fuse_current(&params, &[0.0, 0.1], false) + 1.0).abs() < 1e-12, "signed, pins[0] to pins[1]");

        assert!(fuse_stamp(&params, true).is_empty());
        assert_eq!(fuse_current(&params, &[0.1, 0.0], true), 0.0);
    }

    #[test]
    fn a_ptc_tripped_at_half_an_ohm_is_one_hundred_ohms() {
        // rNormal 0.5 ohm (the catalog default) times 200 (PA:759).
        let params = [0.5, 0.5];
        assert_eq!(ptc_resistance(0.5, false), 0.5);
        assert_eq!(ptc_resistance(0.5, true), 100.0);
        assert_eq!(ptc_stamp(&params, false)[0].conductance, 2.0);
        assert_eq!(ptc_stamp(&params, true)[0].conductance, 0.01);
        // 0.5 V across the tripped PTC is 5 mA (probe F `ptc`: "at 0.5 V the
        // tripped current is 5 mA").
        assert!((ptc_current(&params, &[0.5, 0.0], true) - 0.005).abs() < 1e-15);
        assert_eq!(ptc_current(&params, &[0.5, 0.0], false), 1.0);
    }

    #[test]
    fn a_ptc_never_stamps_an_open() {
        for tripped in [false, true] {
            let s = ptc_stamp(&[0.5, 0.5], tripped);
            assert_eq!(s.len(), 1);
            assert_eq!((s[0].pin_a, s[0].pin_b, s[0].injection), (0, 1, 0.0));
        }
    }

    #[test]
    fn the_tripped_resistance_is_multiplied_before_it_is_inverted() {
        // classic: `stampResistor(.., rNormal * 200)` then `g = 1 / R`. Inverting
        // first and dividing by 200 rounds differently for these `rNormal`
        // (values computed with node's `1 / (r * 200)`).
        for (r, g) in [
            (0.11099999999999999, 0.04504504504504506),
            (0.185, 0.02702702702702703),
            (0.37, 0.013513513513513514),
            (0.481, 0.010395010395010394),
        ] {
            assert_eq!(ptc_stamp(&[r, 0.5], true)[0].conductance, g, "rNormal {r}");
            assert_ne!((1.0 / r) / PTC_TRIPPED_FACTOR, g, "the other order would round differently at {r}");
        }
    }

    #[test]
    fn leakage_stamp_is_empty_when_leakage_is_zero() {
        assert!(leakage_stamp(&[1e-6, 0.0, 0.0]).is_empty());
    }

    #[test]
    fn leakage_stamp_is_the_reciprocal_conductance_when_present() {
        let s = leakage_stamp(&[1e-6, 0.0, 1e7]);
        assert_eq!(s.len(), 1);
        assert_eq!((s[0].pin_a, s[0].pin_b), (0, 1));
        assert!((s[0].conductance - 1e-7).abs() < 1e-20);
        assert_eq!(s[0].injection, 0.0);
    }

    #[test]
    fn be_companion_matches_the_spec_formula() {
        // C = 1uF, esr = 0, h = 1e-3, vcPrev = 2.0.
        let c = capacitor_be_companion(&[1e-6, 0.0, 0.0], 1e-3, 2.0);
        let expected_g = 1.0 / (0.0 + 1e-3 / 1e-6);
        assert!((c.conductance - expected_g).abs() < 1e-15);
        assert!((c.injection - (-(expected_g * 2.0))).abs() < 1e-12);
        // Series current i = G*v - Ihist == conductance*(va-vb) + injection.
        let i = c.conductance * (5.0 - 1.0) + c.injection;
        assert!((i - (expected_g * 4.0 - expected_g * 2.0)).abs() < 1e-9);
    }

    #[test]
    fn trap_companion_matches_the_spec_formula() {
        // C = 1uF, esr = 0.05, h = 1e-3, vcPrev = 1.0, iPrev = 0.5e-3.
        let (c_f, esr, h, vc_prev, i_prev) = (1e-6, 0.05, 1e-3, 1.0, 0.5e-3);
        let contrib = capacitor_trap_companion(&[c_f, esr, 0.0], h, vc_prev, i_prev);
        let half_h_over_c = h / (2.0 * c_f);
        let expected_g = 1.0 / (esr + half_h_over_c);
        let expected_ihist = expected_g * (vc_prev + half_h_over_c * i_prev);
        assert!((contrib.conductance - expected_g).abs() < 1e-9);
        assert!((contrib.injection - (-expected_ihist)).abs() < 1e-9);
    }
}
