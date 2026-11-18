//! Variable resistive devices and the constant current source (W1h item H1a).
//! Reached only through `devices::stamp` and `devices::current` (the H0 seam;
//! `handles` owns the kind list). None of the kinds is in
//! `CORE_IMPLEMENTED_KINDS` yet: the solver arms that route them (Phase B, item
//! H1b) are not in, so no compiled program reaches this file today.
//!
//! The stamps: `potentiometer` and `trimmer` two conductances `cw-wiper` and
//! `wiper-ccw`; `resistor_array` four isolated `a_i-b_i` resistors; `ldr` one
//! resistor; `push_dpdt` two poles of 1 mohm contacts; `current_source` a
//! `+I`/`-I` injection plus an optional shunt, the one kind whose
//! [`Contribution::injection`] is non-zero without a capacitor. The param
//! layouts are fixed in `kinds.rs` (`pot_param`, `array_param`, `ldr_param`,
//! `dpdt_param`, `current_source_param`); a check may be tightened, a layout may
//! not move. The pot's and the array's overload latches (slow state) are item
//! H1c's: nothing here reads a failed flag.

use super::Contribution;
use crate::kinds::{self, array_param, current_source_param, dpdt_param, ldr_param, pot_param};

/// A closed contact's resistance, classic's literal (`passives.ts:537-572`, spec
/// 7.6): the `push_dpdt` stamps and reports through it, as the switch family does.
const CONTACT_OHM: f64 = 0.001;

/// The two (common, throw) pin pairs of `push_dpdt`'s pole `pole` (0 or 1) on
/// the record's `l1 c1 r1 l2 c2 r2` order: `(c, left)` then `(c, right)`.
fn dpdt_pole(pole: usize) -> [(usize, usize); 2] {
    let base = 3 * pole;
    [(base + 1, base), (base + 1, base + 2)]
}

/// The stamp of one of the H1a kinds, one [`Contribution`] per branch, in this
/// order (the solver indexes them by position):
/// - `potentiometer`, `trimmer`: `cw-wiper` at `1/rCw`, then `wiper-ccw` at `1/rCcw`.
/// - `resistor_array`: `a_i-b_i` at `1/R` for i = 1..4. Never a contribution
///   across channels, so the solver must not union them.
/// - `ldr`: `a-b` at `1/R`.
/// - `push_dpdt`: for each pole, `c-left` then `c-right`; the throw not selected
///   is kept as a `g = 0` branch (the switch family's rule), because the
///   partition topology is the union of both throws whatever the position.
/// - `current_source`: one `pos-neg` branch, `conductance = 1/rParallel` (0 for
///   none) and `injection = -I`: the part's own `elementI` is `-I` (spec 8.13), so
///   the shared `g (va - vb) + injection` convention injects `+I` at `pos`.
pub fn stamp(kind: u32, pin_count: usize, params: &[f64]) -> Vec<Contribution> {
    let _ = pin_count;
    match kind {
        kinds::POTENTIOMETER | kinds::TRIMMER => vec![
            Contribution::conductance(0, 1, 1.0 / params[pot_param::R_CW]),
            Contribution::conductance(1, 2, 1.0 / params[pot_param::R_CCW]),
        ],
        kinds::RESISTOR_ARRAY => (0..array_param::CHANNELS).map(|ch| Contribution::conductance(2 * ch, 2 * ch + 1, 1.0 / params[array_param::R])).collect(),
        kinds::LDR => vec![Contribution::conductance(0, 1, 1.0 / params[ldr_param::R])],
        kinds::PUSH_DPDT => {
            let right = params[dpdt_param::POSITION] != 0.0;
            (0..2)
                .flat_map(dpdt_pole)
                .enumerate()
                .map(|(i, (c, throw))| {
                    let active = (i % 2 == 1) == right;
                    Contribution::conductance(c, throw, if active { 1.0 / CONTACT_OHM } else { 0.0 })
                })
                .collect()
        }
        kinds::CURRENT_SOURCE => {
            let r_parallel = params[current_source_param::R_PARALLEL];
            let g = if r_parallel > 0.0 { 1.0 / r_parallel } else { 0.0 };
            vec![Contribution { pin_a: 0, pin_b: 1, conductance: g, injection: -params[current_source_param::CURRENT] }]
        }
        _ => Vec::new(),
    }
}

/// The four signed `a_i -> b_i` channel currents of a `resistor_array`, each
/// `(V(a_i) - V(b_i)) / R` (`passives.ts:962-974`, spec 10.10). The channel
/// slots publish these; [`current`] is their sum.
pub fn channel_currents(params: &[f64], pin_voltages: &[f64]) -> [f64; array_param::CHANNELS] {
    let mut out = [0.0; array_param::CHANNELS];
    for (ch, slot) in out.iter_mut().enumerate() {
        *slot = (pin_voltages[2 * ch] - pin_voltages[2 * ch + 1]) / params[array_param::R];
    }
    out
}

/// The `elementI` of one of the H1a kinds, computed the way classic does (a
/// division by the resistance, not a product with the conductance) so the two
/// agree to the last bit where the solved voltages do:
/// - `potentiometer`, `trimmer`: `(V(cw) - V(wiper)) / rCw`, never the track current.
/// - `resistor_array`: the sum of [`channel_currents`], in channel order.
/// - `ldr`: `(V(a) - V(b)) / R`.
/// - `push_dpdt`: pole 1's `(V(c1) - V(active throw)) / 0.001`.
/// - `current_source`: `-I`, plus `V(pos - neg) / rParallel` when the shunt exists.
pub fn current(kind: u32, pin_count: usize, params: &[f64], pin_voltages: &[f64]) -> f64 {
    let _ = pin_count;
    match kind {
        kinds::POTENTIOMETER | kinds::TRIMMER => (pin_voltages[0] - pin_voltages[1]) / params[pot_param::R_CW],
        kinds::RESISTOR_ARRAY => channel_currents(params, pin_voltages).iter().fold(0.0, |sum, i| sum + i),
        kinds::LDR => (pin_voltages[0] - pin_voltages[1]) / params[ldr_param::R],
        kinds::PUSH_DPDT => {
            let throw = if params[dpdt_param::POSITION] != 0.0 { 2 } else { 0 };
            (pin_voltages[1] - pin_voltages[throw]) / CONTACT_OHM
        }
        kinds::CURRENT_SOURCE => {
            let i = params[current_source_param::CURRENT];
            let r_parallel = params[current_source_param::R_PARALLEL];
            if r_parallel > 0.0 {
                -i + (pin_voltages[0] - pin_voltages[1]) / r_parallel
            } else {
                -i
            }
        }
        _ => 0.0,
    }
}

/// Whether `kind` is one of the kinds this module owns. The dispatch in
/// `devices/mod.rs` calls it so the list lives here, not there.
pub fn handles(kind: u32) -> bool {
    matches!(kind, kinds::POTENTIOMETER | kinds::TRIMMER | kinds::RESISTOR_ARRAY | kinds::LDR | kinds::PUSH_DPDT | kinds::CURRENT_SOURCE)
}

/// `v` finite and `> 0`. Finite matters beyond the reader's own check: a live
/// `bo_engine_set_params` value reaches these functions too, and an infinite
/// resistance is an open the compiler declines, never a stamp (`1/inf = 0` would
/// silently become one).
fn positive(what: &str, v: f64) -> Result<(), String> {
    if v.is_finite() && v > 0.0 {
        Ok(())
    } else {
        Err(format!("{what} must be finite and > 0, got {v}"))
    }
}

fn non_negative(what: &str, v: f64) -> Result<(), String> {
    if v.is_finite() && v >= 0.0 {
        Ok(())
    } else {
        Err(format!("{what} must be finite and >= 0, got {v}"))
    }
}

/// `potentiometer` and `trimmer` `[rCw, rCcw, pMax]`: classic floors each
/// segment at 1 ohm, so the compiler never writes a non-positive one; `pMax`
/// is 0 for no rating.
pub(crate) fn pot_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    positive("potentiometer rCw", params[pot_param::R_CW])?;
    positive("potentiometer rCcw", params[pot_param::R_CCW])?;
    non_negative("potentiometer pMax", params[pot_param::P_MAX])
}

/// `resistor_array` `[R, pMax]`, per channel.
pub(crate) fn array_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    positive("resistor_array R", params[array_param::R])?;
    non_negative("resistor_array pMax", params[array_param::P_MAX])
}

/// `ldr` `[R]`.
pub(crate) fn ldr_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    positive("ldr R", params[ldr_param::R])
}

/// `current_source` `[current, rParallel]`: any finite `current`; `rParallel`
/// is 0 for no shunt, so a negative one has no encoding.
pub(crate) fn current_source_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    if !params[current_source_param::CURRENT].is_finite() {
        return Err(format!("current_source current must be finite, got {}", params[current_source_param::CURRENT]));
    }
    non_negative("current_source rParallel", params[current_source_param::R_PARALLEL])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn g(c: &Contribution) -> (usize, usize, f64, f64) {
        (c.pin_a, c.pin_b, c.conductance, c.injection)
    }

    // ---- potentiometer / trimmer (spec 7.7) ------------------------------

    #[test]
    fn pot_stamps_the_two_track_segments_cw_to_wiper_and_wiper_to_ccw() {
        for kind in [kinds::POTENTIOMETER, kinds::TRIMMER] {
            let s = stamp(kind, 3, &[3000.0, 7000.0, 0.2]);
            assert_eq!(s.iter().map(g).collect::<Vec<_>>(), vec![(0, 1, 1.0 / 3000.0, 0.0), (1, 2, 1.0 / 7000.0, 0.0)]);
        }
    }

    #[test]
    fn pot_at_a_stop_keeps_the_one_ohm_segment_so_the_stamp_stays_finite() {
        // Classic floors each segment at 1 ohm (spec 7.7); the compiler writes it, the stamp trusts it.
        let s = stamp(kinds::POTENTIOMETER, 3, &[1.0, 9999.0, 0.0]);
        assert_eq!(s[0].conductance, 1.0);
        assert_eq!(s[1].conductance, 1.0 / 9999.0);
    }

    #[test]
    fn pot_element_i_is_the_cw_to_wiper_current_not_the_track_current() {
        // 10 V at cw, 4 V at the wiper, 0 V at ccw: (10 - 4) / 3000, whatever the ccw side carries.
        let i = current(kinds::POTENTIOMETER, 3, &[3000.0, 7000.0, 0.2], &[10.0, 4.0, 0.0]);
        assert_eq!(i, (10.0 - 4.0) / 3000.0);
        // Reversed bias reports negative, as every two-terminal current does.
        assert!(current(kinds::TRIMMER, 3, &[3000.0, 7000.0, 0.0], &[0.0, 4.0, 10.0]) < 0.0);
        // The ccw pin does not enter it at all.
        assert_eq!(current(kinds::TRIMMER, 3, &[3000.0, 7000.0, 0.0], &[10.0, 4.0, 123.0]), (10.0 - 4.0) / 3000.0);
    }

    #[test]
    fn pot_check_rejects_a_segment_or_rating_that_cannot_be_stamped_or_rated() {
        assert!(pot_check(3, &[1.0, 1.0, 0.0], &[]).is_ok());
        assert!(pot_check(3, &[0.0, 1.0, 0.0], &[]).is_err(), "rCw 0 divides by zero");
        assert!(pot_check(3, &[1.0, -5.0, 0.0], &[]).is_err());
        assert!(pot_check(3, &[1.0, 1.0, -0.1], &[]).is_err(), "a negative rating has no meaning");
        assert!(pot_check(3, &[f64::INFINITY, 1.0, 0.0], &[]).is_err(), "an infinite segment is an open, which the compiler declines");
        assert!(pot_check(3, &[1.0, f64::NAN, 0.0], &[]).is_err());
        assert!(pot_check(3, &[1.0, 1.0, f64::INFINITY], &[]).is_err(), "pMax must be finite");
    }

    // ---- resistor_array (passives.ts:924-1010; spec 10.10) ---------------

    #[test]
    fn array_stamps_four_isolated_channels_and_never_unions_across_them() {
        let s = stamp(kinds::RESISTOR_ARRAY, 8, &[4700.0, 0.0625]);
        assert_eq!(
            s.iter().map(g).collect::<Vec<_>>(),
            vec![(0, 1, 1.0 / 4700.0, 0.0), (2, 3, 1.0 / 4700.0, 0.0), (4, 5, 1.0 / 4700.0, 0.0), (6, 7, 1.0 / 4700.0, 0.0)]
        );
        // No contribution reaches across a channel: every one joins pins 2i and 2i+1.
        assert!(s.iter().all(|c| c.pin_a / 2 == c.pin_b / 2));
    }

    #[test]
    fn array_channel_currents_are_signed_a_to_b_and_element_i_is_their_sum() {
        let params = [1000.0, 0.0625];
        // a_i, b_i voltages per channel: +5 mA, -2 mA, 0, +1 mA.
        let v = [5.0, 0.0, 0.0, 2.0, 3.0, 3.0, 1.0, 0.0];
        assert_eq!(channel_currents(&params, &v), [5.0 / 1000.0, -2.0 / 1000.0, 0.0, 1.0 / 1000.0]);
        // The scalar is the classic sum (passives.ts:962-974): signed, so a reversed channel subtracts.
        let want = (((0.0 + 5.0 / 1000.0) + -2.0 / 1000.0) + 0.0) + 1.0 / 1000.0;
        assert_eq!(current(kinds::RESISTOR_ARRAY, 8, &params, &v), want);
    }

    #[test]
    fn array_check_needs_a_positive_finite_r_and_a_finite_non_negative_rating() {
        assert!(array_check(8, &[1.0, 0.0], &[0.0]).is_ok());
        assert!(array_check(8, &[0.0, 0.0], &[0.0]).is_err());
        assert!(array_check(8, &[10.0, -1.0], &[0.0]).is_err());
        assert!(array_check(8, &[f64::INFINITY, 0.0], &[0.0]).is_err());
        assert!(array_check(8, &[10.0, f64::INFINITY], &[0.0]).is_err());
    }

    // ---- ldr (spec 7.9) --------------------------------------------------

    #[test]
    fn ldr_is_one_resistor_at_the_resolved_r() {
        let s = stamp(kinds::LDR, 2, &[15_000.0]);
        assert_eq!(s.iter().map(g).collect::<Vec<_>>(), vec![(0, 1, 1.0 / 15_000.0, 0.0)]);
        assert_eq!(current(kinds::LDR, 2, &[15_000.0], &[3.0, 1.5]), (3.0 - 1.5) / 15_000.0);
    }

    #[test]
    fn ldr_check_needs_a_positive_finite_r() {
        assert!(ldr_check(2, &[500.0], &[]).is_ok());
        assert!(ldr_check(2, &[0.0], &[]).is_err());
        assert!(ldr_check(2, &[f64::INFINITY], &[]).is_err());
        assert!(ldr_check(2, &[f64::NAN], &[]).is_err());
    }

    // ---- push_dpdt (spec 7.6) --------------------------------------------

    #[test]
    fn dpdt_closes_only_the_active_throw_of_each_pole_with_1_mohm_contacts() {
        // Pins l1 c1 r1 l2 c2 r2 = 0..5. Position 0: left throws.
        let left = stamp(kinds::PUSH_DPDT, 6, &[0.0]);
        assert_eq!(
            left.iter().map(g).collect::<Vec<_>>(),
            vec![(1, 0, 1000.0, 0.0), (1, 2, 0.0, 0.0), (4, 3, 1000.0, 0.0), (4, 5, 0.0, 0.0)]
        );
        // Position 1: right throws; break before make, so a throw is never bridged to the other.
        let right = stamp(kinds::PUSH_DPDT, 6, &[1.0]);
        assert_eq!(
            right.iter().map(g).collect::<Vec<_>>(),
            vec![(1, 0, 0.0, 0.0), (1, 2, 1000.0, 0.0), (4, 3, 0.0, 0.0), (4, 5, 1000.0, 0.0)]
        );
    }

    #[test]
    fn dpdt_element_i_is_pole_one_common_to_its_active_throw() {
        // c1 at 2 V, l1 at 1.999 V, r1 at 0 V, pole 2 carrying something else entirely.
        let v = [1.999, 2.0, 0.0, 7.0, 9.0, 0.0];
        assert_eq!(current(kinds::PUSH_DPDT, 6, &[0.0], &v), (2.0 - 1.999) / 0.001);
        // At position 1 the open left contact reads nothing and the right one is c1 - r1.
        assert_eq!(current(kinds::PUSH_DPDT, 6, &[1.0], &v), (2.0 - 0.0) / 0.001);
    }

    // ---- current_source (spec 8.13) --------------------------------------

    #[test]
    fn current_source_injects_plus_i_at_pos_and_minus_i_at_neg_with_no_matrix_entry() {
        let s = stamp(kinds::CURRENT_SOURCE, 2, &[1e-3, 0.0]);
        assert_eq!(s.len(), 1);
        // The shared convention: the pin_a -> pin_b branch current is `g (va - vb) + injection`, and the part's
        // elementI is `-I` (the programmed current leaves pos through the circuit), so `injection = -I`.
        assert_eq!(g(&s[0]), (0, 1, 0.0, -1e-3));
        // KCL at pos: the branch current pos -> neg is -I, i.e. +I flows INTO pos from the part.
        assert_eq!(crate::solve::branch_current(s[0].conductance, s[0].injection, 3.0, 1.0), -1e-3);
    }

    #[test]
    fn current_source_with_a_shunt_adds_the_conductance_across_its_pins() {
        let s = stamp(kinds::CURRENT_SOURCE, 2, &[1e-3, 1000.0]);
        assert_eq!(g(&s[0]), (0, 1, 1.0 / 1000.0, -1e-3));
    }

    #[test]
    fn current_source_element_i_is_minus_i_plus_the_shunt_current() {
        // Spec 8.13 probes: 1 mA alone reads -1e-3; with a 1 kohm shunt and 0.5 V across it, -0.5e-3.
        assert_eq!(current(kinds::CURRENT_SOURCE, 2, &[1e-3, 0.0], &[0.0, 5.0]), -1e-3);
        assert_eq!(current(kinds::CURRENT_SOURCE, 2, &[1e-3, 1000.0], &[0.5, 0.0]), -1e-3 + 0.5 / 1000.0);
        assert_eq!(current(kinds::CURRENT_SOURCE, 2, &[-2e-3, 0.0], &[0.0, 0.0]), 2e-3, "a negative current flips the sign");
    }

    #[test]
    fn current_source_check_takes_any_finite_current_and_a_finite_non_negative_shunt() {
        assert!(current_source_check(2, &[-5.0, 0.0], &[]).is_ok());
        assert!(current_source_check(2, &[0.0, 1e-3], &[]).is_ok());
        assert!(current_source_check(2, &[f64::NAN, 0.0], &[]).is_err());
        assert!(current_source_check(2, &[f64::INFINITY, 0.0], &[]).is_err());
        assert!(current_source_check(2, &[1e-3, -1.0], &[]).is_err());
        assert!(current_source_check(2, &[1e-3, f64::INFINITY], &[]).is_err());
    }

    // ---- dispatch --------------------------------------------------------

    #[test]
    fn this_module_owns_exactly_the_h1a_kinds_and_the_dispatch_reaches_it() {
        for kind in [kinds::POTENTIOMETER, kinds::TRIMMER, kinds::RESISTOR_ARRAY, kinds::LDR, kinds::PUSH_DPDT, kinds::CURRENT_SOURCE] {
            assert!(handles(kind));
        }
        for kind in [kinds::RESISTOR, kinds::THERMISTOR, kinds::INDUCTOR, kinds::SIGNAL_GEN, kinds::SPDT_SWITCH] {
            assert!(!handles(kind));
        }
        // devices::stamp / devices::current route here (the H0 seam), not to the empty default.
        let s = crate::devices::stamp(kinds::POTENTIOMETER, 3, &[2.0, 4.0, 0.0]);
        assert_eq!(s.len(), 2);
        assert_eq!(crate::devices::current(kinds::POTENTIOMETER, 3, &[2.0, 4.0, 0.0], &[6.0, 2.0, 0.0]), 2.0);
    }

    #[test]
    fn the_reader_runs_these_checks_for_the_kinds() {
        assert!((kinds::shape(kinds::POTENTIOMETER).unwrap().check)(3, &[0.0, 1.0, 0.0], &[]).is_err());
        assert!((kinds::shape(kinds::RESISTOR_ARRAY).unwrap().check)(8, &[0.0, 0.0], &[0.0]).is_err());
        assert!((kinds::shape(kinds::LDR).unwrap().check)(2, &[0.0], &[]).is_err());
        assert!((kinds::shape(kinds::CURRENT_SOURCE).unwrap().check)(2, &[1e-3, -1.0], &[]).is_err());
    }
}
