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

// ---- W1h magnetics (H4a): the inductor and coupled-inductor companions ----------
//
// The magnetics and the thermistor are stepped-partition devices (R56 5 and 6),
// so they have no `devices::stamp` arm: like the capacitor, their companions
// depend on history the stamp signature cannot carry, and `solve::stepped` calls
// the functions below directly with the state it alone owns (Phase B, H4b).
// Layouts: `kinds::{inductor_param, coupled_param, thermistor_param}`.
//
// Every expression is classic's (`elements.ts` inductorCompanion,
// inductorCompanionTrap, coupledInductorCompanion, coupledInductorCompanionTrap,
// spec 7.5 and 7.16), written in the same operand order so a result matches the
// classic engine bit for bit, floors included (`max(1e-30, L)` and `max(1e-30, h)`
// for the single coil, `max(1e-12, L_w)` for the pair). The `Contribution`
// convention is the capacitor's: the branch current `a -> b` is `conductance *
// (va - vb) + injection`; for a winding that IS classic's `i = G v + Ihist`, so
// `injection` is `+Ihist` (the capacitor's is `-Ihist`).

/// The winding's series resistance, classic's `safeDcr`: finite and `>= 0`.
fn winding_dcr(dcr: f64) -> f64 {
    if dcr.is_finite() { dcr.max(0.0) } else { 0.0 }
}

/// The backward-Euler companion of `L` in series with `dcr` (spec 7.5):
/// `G = h/(L + dcr*h)`, `Ihist = L/(L + dcr*h)*iPrev`. `iPrev` is the last
/// ACCEPTED winding current (not a trial value of a rejected half step).
pub fn inductor_be_companion(params: &[f64], h: f64, i_prev: f64) -> Contribution {
    use crate::kinds::inductor_param::*;
    let (l, h) = (params[L].max(1e-30), h.max(1e-30));
    let denominator = l + winding_dcr(params[DCR]) * h;
    Contribution { pin_a: 0, pin_b: 1, conductance: h / denominator, injection: (l / denominator) * i_prev }
}

/// The trapezoidal companion (spec 7.5): `G = 0.5/(L/h + dcr/2)`, `Ihist =
/// ((L/h - dcr/2)*iPrev + vPrev/2)/(L/h + dcr/2)`. `vPrev` is the terminal
/// voltage `V(a) - V(b)` of the last ACCEPTED step (the winding branch's, since
/// the core-loss shunt is a separate terminal resistor and shares it).
pub fn inductor_trap_companion(params: &[f64], h: f64, i_prev: f64, v_prev: f64) -> Contribution {
    use crate::kinds::inductor_param::*;
    let (l, h) = (params[L].max(1e-30), h.max(1e-30));
    let dcr = winding_dcr(params[DCR]);
    let l_over_h = l / h;
    let denominator = l_over_h + dcr / 2.0;
    Contribution {
        pin_a: 0,
        pin_b: 1,
        conductance: 0.5 / denominator,
        injection: ((l_over_h - dcr / 2.0) * i_prev + v_prev / 2.0) / denominator,
    }
}

/// The core-loss shunt across the terminals (spec 7.5): a plain conductance
/// like the capacitor's leakage, and `coreLossResistance == 0` (none) stamps
/// nothing. It carries no state, so both the stepped stamp and any static
/// path can take it through the ordinary dispatch.
pub fn inductor_core_loss_stamp(params: &[f64]) -> Vec<Contribution> {
    let r = params[crate::kinds::inductor_param::CORE_LOSS];
    if r > 0.0 { vec![Contribution::conductance(0, 1, 1.0 / r)] } else { Vec::new() }
}

/// `elementI` of the inductor (spec 7.5): the winding current plus the
/// core-loss shunt's `V/coreLoss`, `V = V(a) - V(b)` from the solved vector.
pub fn inductor_terminal_current(params: &[f64], winding_current: f64, v_terminal: f64) -> f64 {
    let r = params[crate::kinds::inductor_param::CORE_LOSS];
    winding_current + if r > 0.0 { v_terminal / r } else { 0.0 }
}

/// `M = k*sqrt(L1*L2)` of the coupled inductor (spec 7.16), from the record's
/// `l1`, `l2` and the already clamped `k`.
pub fn coupled_mutual(params: &[f64]) -> f64 {
    use crate::kinds::coupled_param::*;
    params[K] * (params[L1] * params[L2]).sqrt()
}

/// The 2x2 Norton companion of two coupled windings (spec 7.16): `[i1, i2] =
/// G [v1, v2] + [ih1, ih2]`, `i_w` positive `a_w -> b_w`, `v_w = V(a_w) -
/// V(b_w)`. The stamp is `g11` across pair 1, `g22` across pair 2, the eight
/// `+-g12` cross entries and `-ih_w` at `a_w`, `+ih_w` at `b_w`; it does not
/// fit [`Contribution`], so the stepped solver writes it (H4b).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoupledCompanion {
    pub g11: f64,
    pub g12: f64,
    pub g22: f64,
    pub ih1: f64,
    pub ih2: f64,
}

impl CoupledCompanion {
    /// The new-step winding currents at the solved terminal voltages (classic's
    /// `coupledInductorWindingCurrents`), the commit-side twin of the stamp.
    pub fn winding_currents(&self, v1: f64, v2: f64) -> (f64, f64) {
        (self.g11 * v1 + self.g12 * v2 + self.ih1, self.g12 * v1 + self.g22 * v2 + self.ih2)
    }

    /// Inverts the symmetric `A = [[a11, a12], [a12, a22]]` and applies it to the history vector `c`.
    fn from_matrix(a11: f64, a12: f64, a22: f64, c1: f64, c2: f64) -> Self {
        let det = a11 * a22 - a12 * a12;
        let (g11, g22, g12) = (a22 / det, a11 / det, -a12 / det);
        CoupledCompanion { g11, g12, g22, ih1: g11 * c1 + g12 * c2, ih2: g12 * c1 + g22 * c2 }
    }
}

/// Backward Euler: `A = [[L1/h + DCR1, M/h], [M/h, L2/h + DCR2]]`, `c = [(L1 i1
/// + M i2)/h, (M i1 + L2 i2)/h]` from the last accepted currents. `det > 0`
/// for every record the reader accepts (`k <= 0.9999`).
pub fn coupled_inductor_be_companion(params: &[f64], h: f64, i1_prev: f64, i2_prev: f64) -> CoupledCompanion {
    use crate::kinds::coupled_param::*;
    let m = coupled_mutual(params);
    let (l1, l2, h) = (params[L1].max(1e-12), params[L2].max(1e-12), h.max(1e-30));
    let a11 = l1 / h + winding_dcr(params[DCR1]);
    let a22 = l2 / h + winding_dcr(params[DCR2]);
    let a12 = m / h;
    let c1 = (l1 * i1_prev + m * i2_prev) / h;
    let c2 = (m * i1_prev + l2 * i2_prev) / h;
    CoupledCompanion::from_matrix(a11, a12, a22, c1, c2)
}

/// Trapezoid: `A = [[2L1/h + DCR1, 2M/h], [2M/h, 2L2/h + DCR2]]`, `c = [(2L1/h
/// - DCR1) i1 + (2M/h) i2 + v1Prev, (2M/h) i1 + (2L2/h - DCR2) i2 + v2Prev]`,
/// `vPrev` the last accepted winding terminal voltages.
pub fn coupled_inductor_trap_companion(params: &[f64], h: f64, i1_prev: f64, i2_prev: f64, v1_prev: f64, v2_prev: f64) -> CoupledCompanion {
    use crate::kinds::coupled_param::*;
    let m = coupled_mutual(params);
    let (l1, l2, h) = (params[L1].max(1e-12), params[L2].max(1e-12), h.max(1e-30));
    let (dcr1, dcr2) = (winding_dcr(params[DCR1]), winding_dcr(params[DCR2]));
    let two_over_h = 2.0 / h;
    let a11 = l1 * two_over_h + dcr1;
    let a22 = l2 * two_over_h + dcr2;
    let a12 = m * two_over_h;
    let c1 = (l1 * two_over_h - dcr1) * i1_prev + a12 * i2_prev + v1_prev;
    let c2 = a12 * i1_prev + (l2 * two_over_h - dcr2) * i2_prev + v2_prev;
    CoupledCompanion::from_matrix(a11, a12, a22, c1, c2)
}

// ---- W1h reader checks (H0 seam, tightened by H4a: finite everywhere) ------------

/// A finite number `>= 0`; `NaN` and the infinities fail, which `x < 0.0` alone let through.
fn finite_non_negative(x: f64) -> bool {
    x.is_finite() && x >= 0.0
}

/// `inductor` `[L, dcr, coreLossResistance]`: classic floors `L` at 1e-12 in the
/// compiler, but the check keeps only `L > 0` and finite, because the crystal's
/// `Ls` (derived, never floored by classic's stamp) may sit below it. `dcr` and
/// `coreLossResistance` are 0 for none.
pub(crate) fn inductor_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    use crate::kinds::inductor_param::*;
    if !(params[L] > 0.0) || !params[L].is_finite() {
        return Err(format!("inductor L must be finite and > 0, got {}", params[L]));
    }
    if !finite_non_negative(params[DCR]) || !finite_non_negative(params[CORE_LOSS]) {
        return Err(format!("inductor dcr and coreLossResistance must be finite and >= 0, got {} and {}", params[DCR], params[CORE_LOSS]));
    }
    Ok(())
}

/// `coupled_inductor` `[l1, l2, k, dcr1, dcr2]`: `k` is already clamped to
/// `[0, COUPLING_K_MAX]`, so the 2x2 block is always invertible.
pub(crate) fn coupled_inductor_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    use crate::kinds::coupled_param::*;
    if !(params[L1] > 0.0) || !(params[L2] > 0.0) || !params[L1].is_finite() || !params[L2].is_finite() {
        return Err(format!("coupled_inductor l1 and l2 must be finite and > 0, got {} and {}", params[L1], params[L2]));
    }
    if !(0.0..=crate::kinds::COUPLING_K_MAX).contains(&params[K]) {
        return Err(format!("coupled_inductor k must be in [0, {}], got {}", crate::kinds::COUPLING_K_MAX, params[K]));
    }
    if !finite_non_negative(params[DCR1]) || !finite_non_negative(params[DCR2]) {
        return Err(format!("coupled_inductor dcr1 and dcr2 must be finite and >= 0, got {} and {}", params[DCR1], params[DCR2]));
    }
    Ok(())
}

/// `thermistor` `[rNominal, beta, dissipationFactor, thermalTau, ambientC]`:
/// classic floors the last two at 1e-6; `beta` and the ambient are any finite
/// number.
pub(crate) fn thermistor_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    use crate::kinds::thermistor_param::*;
    if !(params[R_NOMINAL] > 0.0) {
        return Err(format!("thermistor rNominal must be > 0, got {}", params[R_NOMINAL]));
    }
    if !(params[DISSIPATION] > 0.0) || !(params[TAU] > 0.0) {
        return Err(format!("thermistor dissipationFactor and thermalTau must be > 0, got {} and {}", params[DISSIPATION], params[TAU]));
    }
    Ok(())
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

    // ---- W1h H4a: the inductor and coupled-inductor companions (spec 7.5, 7.16) ----
    //
    // Expected numbers are classic's own (`elements.ts` inductorCompanion*,
    // coupledInductorCompanion*), printed by a TS probe from the same inputs and
    // pasted as the f64 they round-trip to, so a `assert_eq!` here is a bit-for-bit
    // match, not a tolerance.

    const L_H: f64 = 1e-3;
    const H: f64 = 1e-6;

    fn ind(l: f64, dcr: f64) -> [f64; 3] {
        [l, dcr, 0.0]
    }

    #[test]
    fn inductor_be_companion_matches_the_spec_formula() {
        // G = h/(L + dcr*h), Ihist = L/(L + dcr*h)*iPrev, i = G*v + Ihist.
        let c = inductor_be_companion(&ind(L_H, 5.0), H, 0.2);
        assert_eq!((c.pin_a, c.pin_b), (0, 1));
        assert_eq!(c.conductance, 0.0009950248756218905);
        assert_eq!(c.injection, 0.19900497512437812);
        // The winding current at v = 2 V is the branch current convention's value.
        assert_eq!(c.conductance * 2.0 + c.injection, 0.2009950248756219);
    }

    #[test]
    fn an_ideal_inductor_be_companion_is_h_over_l_and_holds_the_current() {
        let c = inductor_be_companion(&ind(L_H, 0.0), H, 0.2);
        assert_eq!(c.conductance, 0.001);
        assert_eq!(c.injection, 0.2);
    }

    #[test]
    fn inductor_trap_companion_matches_the_spec_formula() {
        // G = 0.5/(L/h + dcr/2), Ihist = ((L/h - dcr/2)*iPrev + vPrev/2)/(L/h + dcr/2).
        let c = inductor_trap_companion(&ind(L_H, 5.0), H, 0.2, 3.0);
        assert_eq!(c.conductance, 0.0004987531172069825);
        assert_eq!(c.injection, 0.200498753117207);
        assert_eq!(c.conductance * 2.0 + c.injection, 0.20149625935162097);
    }

    #[test]
    fn an_ideal_inductor_trap_companion_is_h_over_2l_with_the_averaged_voltage() {
        // dcr = 0: G = h/2L and Ihist = iPrev + vPrev*h/2L (0.2 + 3*1e-6/2e-3).
        let c = inductor_trap_companion(&ind(L_H, 0.0), H, 0.2, 3.0);
        assert_eq!(c.conductance, 0.0004999999999999999);
        assert_eq!(c.injection, 0.2015);
    }

    #[test]
    fn the_core_loss_shunt_is_a_plain_conductance_and_none_stamps_nothing() {
        assert!(inductor_core_loss_stamp(&[L_H, 5.0, 0.0]).is_empty());
        let s = inductor_core_loss_stamp(&[L_H, 5.0, 1e5]);
        assert_eq!(s.len(), 1);
        assert_eq!((s[0].pin_a, s[0].pin_b, s[0].injection), (0, 1, 0.0));
        assert_eq!(s[0].conductance, 1e-5);
    }

    #[test]
    fn the_terminal_current_adds_the_core_loss_shunt_to_the_winding_current() {
        // elementI = i + V/coreLoss (spec 7.5); no shunt adds nothing.
        assert_eq!(inductor_terminal_current(&[L_H, 5.0, 1e5], 0.25, 10.0), 0.25 + 10.0 / 1e5);
        assert_eq!(inductor_terminal_current(&[L_H, 5.0, 0.0], 0.25, 10.0), 0.25);
    }

    #[test]
    fn companion_floors_match_classic() {
        // safeL = max(1e-30, L) and safeH = max(1e-30, h): a zero step is the 1e-30 step, not a division by zero.
        let c = inductor_be_companion(&ind(L_H, 0.0), 0.0, 0.5);
        assert!(c.conductance.is_finite() && c.conductance > 0.0);
        assert_eq!(c.conductance, 1e-30 / L_H);
        let t = inductor_trap_companion(&ind(L_H, 0.0), 0.0, 0.5, 0.0);
        assert!(t.conductance.is_finite() && t.injection.is_finite());
    }

    /// i(n+1) = G*v + Ihist(i(n)) for a constant V across the winding.
    fn be_rl_current(l: f64, dcr: f64, v: f64, h: f64, steps: usize) -> f64 {
        let params = ind(l, dcr);
        let mut i = 0.0;
        for _ in 0..steps {
            let c = inductor_be_companion(&params, h, i);
            i = c.conductance * v + c.injection;
        }
        i
    }

    #[test]
    fn be_rl_step_is_the_geometric_series_and_settles_at_v_over_dcr() {
        // i_n = (V/R)(1 - a^n) with a = L/(L + R*h): the BE recurrence in closed form.
        let (l, r, v, h): (f64, f64, f64, f64) = (1e-3, 5.0, 5.0, 1e-6);
        let a = l / (l + r * h);
        let n = 400;
        let exact = (v / r) * (1.0 - a.powi(n));
        let got = be_rl_current(l, r, v, h, n as usize);
        assert!((got - exact).abs() < 1e-12, "{got} vs {exact}");
        // Many time constants (L/R = 200 us): the DC value.
        assert!((be_rl_current(l, r, v, h, 20_000) - v / r).abs() < 1e-9);
    }

    #[test]
    fn trap_rl_step_tracks_the_exponential_to_second_order() {
        // The trapezoid is second order: at t = tau the error with h = tau/200 is far below BE's.
        let (l, r, v, h) = (1e-3, 5.0, 5.0, 1e-6);
        let params = ind(l, r);
        // The source steps at t = 0, so the history voltage is the stepped value (a trap
        // step across the jump would average 0 and V; the engine re-anchors with BE there).
        let (mut i, mut vp): (f64, f64) = (0.0, v);
        let steps = 200; // t = 200 us = L/R = tau
        for _ in 0..steps {
            let c = inductor_trap_companion(&params, h, i, vp);
            i = c.conductance * v + c.injection;
            vp = v;
        }
        let exact = (v / r) * (1.0 - (-1.0f64).exp());
        let be = be_rl_current(l, r, v, h, steps);
        assert!((i - exact).abs() < 1e-5, "trap {i} vs {exact}");
        assert!((be - exact).abs() > 10.0 * (i - exact).abs(), "trap must beat BE here");
    }

    fn xfmr() -> [f64; 5] {
        // l1 1 mH, l2 4 mH, k 0.98, dcr1 2, dcr2 3.
        [1e-3, 4e-3, 0.98, 2.0, 3.0]
    }

    #[test]
    fn the_mutual_inductance_is_k_times_the_geometric_mean() {
        assert_eq!(coupled_mutual(&xfmr()), 0.00196);
    }

    #[test]
    fn coupled_be_companion_matches_the_spec_numbers() {
        let c = coupled_inductor_be_companion(&xfmr(), H, 0.1, -0.05);
        assert_eq!(c.g11, 0.02362962350802202);
        assert_eq!(c.g12, -0.011569838140325545);
        assert_eq!(c.g22, 0.005914784600309284);
        assert_eq!(c.ih1, 0.09353859957734659);
        assert_eq!(c.ih2, -0.046798814681888415);
        let (i1, i2) = c.winding_currents(1.2, -0.4);
        assert_eq!(i1, 0.12652208304310322);
        assert_eq!(i2, -0.06304853429040277);
    }

    #[test]
    fn coupled_trap_companion_matches_the_spec_numbers() {
        let c = coupled_inductor_trap_companion(&xfmr(), H, 0.1, -0.05, 1.5, -0.7);
        assert_eq!(c.g11, 0.01220702678132903);
        assert_eq!(c.g12, -0.005979200922505285);
        assert_eq!(c.g22, 0.003053663328279485);
        assert_eq!(c.ih1, 0.11581940982846431);
        assert_eq!(c.ih2, -0.05779858634606774);
    }

    #[test]
    fn the_conductance_block_is_the_inverse_of_the_inductance_matrix() {
        // G*A = I with A = [[L1/h + dcr1, M/h], [M/h, L2/h + dcr2]] (BE).
        let p = xfmr();
        let c = coupled_inductor_be_companion(&p, H, 0.0, 0.0);
        let m = coupled_mutual(&p);
        let (a11, a12, a22) = (p[0] / H + p[3], m / H, p[1] / H + p[4]);
        assert!((c.g11 * a11 + c.g12 * a12 - 1.0).abs() < 1e-12);
        assert!((c.g11 * a12 + c.g12 * a22).abs() < 1e-12);
        assert!((c.g12 * a11 + c.g22 * a12).abs() < 1e-12);
        assert!((c.g12 * a12 + c.g22 * a22 - 1.0).abs() < 1e-12);
    }

    #[test]
    fn with_no_coupling_each_winding_is_a_single_inductor() {
        // k = 0: classic's own note, "two independent inductorCompanion() results up to rounding".
        let c = coupled_inductor_be_companion(&[1e-3, 1e-3, 0.0, 5.0, 5.0], H, 0.1, 0.2);
        let a = inductor_be_companion(&ind(1e-3, 5.0), H, 0.1);
        let b = inductor_be_companion(&ind(1e-3, 5.0), H, 0.2);
        assert_eq!(c.g12, 0.0);
        assert!((c.g11 - a.conductance).abs() < 1e-18 && (c.g22 - b.conductance).abs() < 1e-18);
        assert!((c.ih1 - a.injection).abs() < 1e-15 && (c.ih2 - b.injection).abs() < 1e-15);
        assert_eq!(c.g11, 0.0009950248756218905);
        assert_eq!(c.ih1, 0.09950248756218906);
        assert_eq!(c.ih2, 0.19900497512437812);
    }

    #[test]
    fn at_the_clamped_maximum_coupling_the_block_stays_finite() {
        // k = 0.9999, dcr 0: det = (L/h)^2 (1 - k^2) ~ 2e-4 (L/h)^2, large but finite, and no history.
        let c = coupled_inductor_be_companion(&[1e-3, 1e-3, crate::kinds::COUPLING_K_MAX, 0.0, 0.0], H, 0.0, 0.0);
        assert!(c.g11.is_finite() && c.g11 > 0.0);
        assert_eq!(c.g11, 5.000250012495037);
        assert_eq!(c.g12, -4.999749987493787);
        assert_eq!((c.ih1, c.ih2), (0.0, 0.0));
    }

    #[test]
    fn a_dc_short_pair_pins_each_winding_to_v_over_dcr() {
        // Both windings under a constant 1 V and 2 V: BE converges to v_w/dcr_w whatever the coupling.
        let p = [1e-3, 1e-3, 0.5, 2.0, 4.0];
        let (mut i1, mut i2): (f64, f64) = (0.0, 0.0);
        for _ in 0..40_000 {
            let c = coupled_inductor_be_companion(&p, H, i1, i2);
            (i1, i2) = c.winding_currents(1.0, 2.0);
        }
        assert!((i1 - 0.5).abs() < 1e-9 && (i2 - 0.5).abs() < 1e-9, "{i1} {i2}");
    }

    // ---- the reader checks, tightened ----

    fn rejects(check: fn(usize, &[f64], &[f64]) -> Result<(), String>, params: &[f64]) -> bool {
        check(0, params, &[]).is_err()
    }

    #[test]
    fn the_inductor_check_rejects_nan_and_infinity_everywhere() {
        assert!(inductor_check(0, &[1e-3, 5.0, 1e5], &[]).is_ok());
        assert!(inductor_check(0, &[1e-13, 0.0, 0.0], &[]).is_ok(), "a crystal's Ls may sit under classic's 1e-12 floor");
        for bad in [f64::NAN, f64::INFINITY, -1.0, 0.0] {
            assert!(rejects(inductor_check, &[bad, 5.0, 1e5]), "L = {bad}");
        }
        for bad in [f64::NAN, f64::INFINITY, -1.0] {
            assert!(rejects(inductor_check, &[1e-3, bad, 1e5]), "dcr = {bad}");
            assert!(rejects(inductor_check, &[1e-3, 5.0, bad]), "coreLoss = {bad}");
        }
    }

    #[test]
    fn the_coupled_check_rejects_nan_infinity_and_an_unclamped_k() {
        assert!(coupled_inductor_check(0, &[1e-3, 1e-3, 0.98, 5.0, 5.0], &[]).is_ok());
        assert!(coupled_inductor_check(0, &[1e-3, 1e-3, 0.0, 0.0, 0.0], &[]).is_ok());
        assert!(coupled_inductor_check(0, &[1e-3, 1e-3, 0.9999, 0.0, 0.0], &[]).is_ok());
        for bad in [f64::NAN, f64::INFINITY, -1.0, 0.0] {
            assert!(rejects(coupled_inductor_check, &[bad, 1e-3, 0.5, 1.0, 1.0]), "l1 = {bad}");
            assert!(rejects(coupled_inductor_check, &[1e-3, bad, 0.5, 1.0, 1.0]), "l2 = {bad}");
        }
        for bad in [f64::NAN, f64::INFINITY, -0.1, 1.0, 0.99991] {
            assert!(rejects(coupled_inductor_check, &[1e-3, 1e-3, bad, 1.0, 1.0]), "k = {bad}");
        }
        for bad in [f64::NAN, f64::INFINITY, -1.0] {
            assert!(rejects(coupled_inductor_check, &[1e-3, 1e-3, 0.5, bad, 1.0]), "dcr1 = {bad}");
            assert!(rejects(coupled_inductor_check, &[1e-3, 1e-3, 0.5, 1.0, bad]), "dcr2 = {bad}");
        }
    }
}
