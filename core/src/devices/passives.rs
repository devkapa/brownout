//! The resistor and capacitor (core/README.md's kind-code table; spec §7.3,
//! §7.4). The resistor's `stamp`/`current` were moved from `engine.rs`'s
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
