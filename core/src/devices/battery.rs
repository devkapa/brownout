//! Battery chemistry and state of charge (core/README.md, "Kernel semantics:
//! rails and sources (W1f)", Rails point 6), as pure functions of a
//! `battery_pack` record. No solver code lives here: the rail (`rails.rs`)
//! owns the SoC state's timeline and calls these at its solves and events.
//!
//! Everything replays classic's `battery-physics.ts` in its own operation
//! order, because the golden table (`battery_golden.rs`, generated from that
//! file's exported functions) is compared bit for bit:
//! - [`BatteryModel::ocv`] is `batteryOpenCircuitVoltageV`;
//! - [`BatteryModel::resistance`] is `batteryInternalResistanceOhm` with the
//!   record's `tempFactor` (the compiler resolves it, so `exp` never runs in
//!   Rust);
//! - [`BatteryModel::advance`] is `advancePrimaryBatteryState` with the
//!   charge `Q` the rail integrated in place of `I * dt`.

use crate::kinds::battery_param;

const SECONDS_PER_HOUR: f64 = 3600.0;

/// A knot within this of a SoC counts as already reached (Rails point 4: "a
/// distance below `1e-12 * max(1, |value|)` counts as reached"; a SoC is at
/// most 1).
const KNOT_REACHED: f64 = 1e-12;

/// Classic's `clampSoc`: non-finite is 0, otherwise clamped to `[0, 1]`. Written
/// as comparisons rather than `min`/`max` so `-0.0` maps to `+0.0` as
/// `Math.max(0, Math.min(1, -0))` does.
pub fn clamp_soc(value: f64) -> f64 {
    if !value.is_finite() || value <= 0.0 {
        0.0
    } else if value >= 1.0 {
        1.0
    } else {
        value
    }
}

/// A `battery_pack` record, resolved. `params` and `blob` are those of
/// code 20 (core/README.md, "Battery record"), already checked by
/// `kinds::battery_check`.
#[derive(Debug, Clone, PartialEq)]
pub struct BatteryModel {
    pub ocv_scale: f64,
    pub r_base: f64,
    pub temp_factor: f64,
    pub dep_a: f64,
    pub dep_p: f64,
    pub dep_k: f64,
    pub capacity_ah: f64,
    pub soc0: f64,
    /// The OCV table, `(soc, volts)`, volts before `ocv_scale`.
    table: Vec<(f64, f64)>,
}

impl BatteryModel {
    /// Reads a validated record. The table pairs are those of `blob[1..]`;
    /// a trailing odd value (never in a checked record) is ignored.
    pub fn from_record(params: &[f64], blob: &[f64]) -> Self {
        use battery_param::*;
        BatteryModel {
            ocv_scale: params[OCV_SCALE],
            r_base: params[R_BASE],
            temp_factor: params[TEMP_FACTOR],
            dep_a: params[DEP_A],
            dep_p: params[DEP_P],
            dep_k: params[DEP_K],
            capacity_ah: params[CAPACITY_AH],
            soc0: params[SOC0],
            table: blob.get(1..).unwrap_or(&[]).chunks_exact(2).map(|pair| (pair[0], pair[1])).collect(),
        }
    }

    /// The table's OCV at `soc`, volts before `ocv_scale`, interpolated as
    /// `batteryOpenCircuitVoltageV`: `soc <= soc_0` gives `v_0`, the first
    /// point with `soc <= soc_i` bounds the segment, a zero-width segment
    /// gives `v_i`, and past the end gives the last value.
    pub fn ocv(&self, soc_input: f64) -> f64 {
        let soc = clamp_soc(soc_input);
        let Some(&(first_soc, first_v)) = self.table.first() else {
            return 0.0;
        };
        if soc <= first_soc {
            return first_v;
        }
        for i in 1..self.table.len() {
            let (right_soc, right_v) = self.table[i];
            if soc > right_soc {
                continue;
            }
            let (left_soc, left_v) = self.table[i - 1];
            let width = right_soc - left_soc;
            if width <= 0.0 {
                return right_v;
            }
            let t = (soc - left_soc) / width;
            return left_v + (right_v - left_v) * t;
        }
        self.table[self.table.len() - 1].1
    }

    /// The source voltage: `OCV(soc) * ocvScale` (the scale is the configured
    /// voltage over the profile's nominal, 1 for an unscaled profile).
    pub fn emf(&self, soc: f64) -> f64 {
        self.ocv(soc) * self.ocv_scale
    }

    /// The least and greatest [`Self::emf`] over every SoC: the table's
    /// extreme knots times the scale, since the interpolation never leaves
    /// its knots (0 for an empty table, which `ocv` reads as 0). `None` when
    /// a knot or the scale is not finite.
    pub fn emf_range(&self) -> Option<(f64, f64)> {
        let mut volts = self.table.iter().map(|&(_, v)| v * self.ocv_scale);
        let first = if self.table.is_empty() { 0.0 } else { volts.next()? };
        let (lo, hi) = volts.fold((first, first), |(lo, hi), v| (if v < lo { v } else { lo }, if v > hi { v } else { hi }));
        let finite = self.table.iter().all(|&(_, v)| (v * self.ocv_scale).is_finite()) && self.ocv_scale.is_finite();
        finite.then_some((lo, hi))
    }

    /// `R(soc) = (rBase * dep) * tempFactor`, `dep = 1 + depA * (1 - soc)^depP
    /// / (soc + depK)`, in classic's order (`A * pow` before the division,
    /// left to right). `depP = 2` is `x * x`, which is what V8's
    /// `Math.pow(x, 2)` returns; any other exponent goes through `powf`, whose
    /// last bit may differ from V8's (no profile has one).
    pub fn resistance(&self, soc_input: f64) -> f64 {
        let soc = clamp_soc(soc_input);
        let x = 1.0 - soc;
        let power = if self.dep_p == 2.0 { x * x } else { x.powf(self.dep_p) };
        let dep = 1.0 + self.dep_a * power / (soc + self.dep_k);
        self.r_base * dep * self.temp_factor
    }

    /// `capacityAh * 3600`, the whole charge in coulombs.
    pub fn capacity_coulombs(&self) -> f64 {
        self.capacity_ah * SECONDS_PER_HOUR
    }

    /// The largest OCV-table knot strictly below `soc` (by more than the
    /// reached tolerance), or 0 when the lowest knot is not lower: the SoC at
    /// which the rail's next `Rail` event is due while the cell drains
    /// (Rails point 4, "SoC reaches the next OCV-table knot below it, or 0").
    /// `None` at `soc = 0`, where there is nothing below.
    pub fn next_knot_below(&self, soc_input: f64) -> Option<f64> {
        let soc = clamp_soc(soc_input);
        if soc <= KNOT_REACHED {
            return None;
        }
        let below = self.table.iter().rev().map(|&(knot, _)| knot).find(|&knot| knot < soc - KNOT_REACHED);
        Some(below.unwrap_or(0.0))
    }

    /// Advances `prev` by the charge `q` delivered out of `pos` over an
    /// interval: `advancePrimaryBatteryState` with `Q` in place of `I * dt`
    /// (BP:447-482). Forward charge is `requested`; reverse charge is counted
    /// as `rejected` and never raises SoC; what a cell accepts is at most the
    /// charge it still holds. A non-finite `q` counts as 0, as classic treats a
    /// non-finite current or step.
    pub fn advance(&self, prev: &BatteryState, q_input: f64) -> SocStep {
        let soc = clamp_soc(prev.soc);
        let q = if q_input.is_finite() { q_input } else { 0.0 };
        let requested = q.max(0.0);
        let rejected = (-q).max(0.0);
        let capacity = self.capacity_coulombs();
        let remaining = capacity * soc;
        let accepted = requested.min(remaining);
        let next_soc = if capacity > 0.0 { clamp_soc(soc - accepted / capacity) } else { 0.0 };
        SocStep {
            state: BatteryState { soc: next_soc, discharged: prev.discharged + accepted, rejected: prev.rejected + rejected },
            requested,
            accepted,
            rejected,
            depleted: next_soc <= 0.0,
        }
    }
}

/// The battery's dynamic state (BP `BatteryDynamicState`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryState {
    pub soc: f64,
    /// Accepted discharge since load, coulombs.
    pub discharged: f64,
    /// Reverse-current charge a primary cell refuses, coulombs.
    pub rejected: f64,
}

impl BatteryState {
    /// The state at load: no time passes at load (owner decision 8), so SoC is
    /// `soc0` and nothing has been discharged.
    pub fn initial(model: &BatteryModel) -> Self {
        BatteryState { soc: clamp_soc(model.soc0), discharged: 0.0, rejected: 0.0 }
    }
}

/// The result of one [`BatteryModel::advance`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SocStep {
    pub state: BatteryState,
    pub requested: f64,
    pub accepted: f64,
    /// The reverse charge of this step alone.
    pub rejected: f64,
    pub depleted: bool,
}

/// The charge of a contribution held constant at `current` amps over `dt`
/// seconds: static and direct terms between events, a stepped term from its
/// last commit.
pub fn held_charge(current: f64, dt: f64) -> f64 {
    current * dt
}

/// The exact charge of a closed-form hub contribution `I(t) = a + b * vc(t)`
/// over `dt` seconds, with `vc(t) = inf + (vc0 - inf) * e^(rate * t)`:
/// `(a + b*inf)*dt + b*(vc0 - inf)*(e^(rate*dt) - 1)/rate`, and `(a + b*vc0)*dt`
/// when `rate = 0` (Rails point 6). `exp_m1` keeps `(e^x - 1)` exact for a
/// small `rate * dt`, where the difference of two nearly equal numbers would
/// cost every digit.
pub fn exp_term_charge(a: f64, b: f64, inf: f64, vc0: f64, rate: f64, dt: f64) -> f64 {
    if rate == 0.0 {
        return (a + b * vc0) * dt;
    }
    let x = rate * dt;
    // `rate` so small that `rate * dt` underflows: the exponential is 1 + x
    // with x below the smallest double, so the term is its own limit.
    let growth = if x == 0.0 { dt } else { x.exp_m1() / rate };
    (a + b * inf) * dt + b * (vc0 - inf) * growth
}

#[cfg(test)]
#[path = "battery_golden.rs"]
mod battery_golden;

#[cfg(test)]
mod tests {
    use super::*;
    use battery_golden::{GoldenProfile, GOLDEN_ROWS, PREV_DISCHARGED, PREV_REJECTED, PROFILES};

    // The generic profile, AA pack, PP3 and CR2032 as `battery_golden` holds
    // them, in the record's `profileCode` order.
    const GENERIC: usize = 0;
    const AA_4S: usize = 1;
    const CR2032: usize = 3;

    /// The record a compiler would emit for a golden profile, with the
    /// temperature factor of the row under test (classic's own value, never
    /// derived here) and no overrides.
    fn model_of(profile: &GoldenProfile, temp_factor: f64, ocv_scale: f64, soc0: f64) -> BatteryModel {
        let mut params = vec![0.0; battery_param::COUNT];
        params[battery_param::OCV_SCALE] = ocv_scale;
        params[battery_param::R_BASE] = profile.r_base;
        params[battery_param::TEMP_FACTOR] = temp_factor;
        params[battery_param::DEP_A] = profile.dep_a;
        params[battery_param::DEP_P] = profile.dep_p;
        params[battery_param::DEP_K] = profile.dep_k;
        params[battery_param::CAPACITY_AH] = profile.capacity_ah;
        params[battery_param::SOC0] = soc0;
        BatteryModel::from_record(&params, profile.ocv_blob)
    }

    fn generic_model(r_base: f64, temp_factor: f64) -> BatteryModel {
        let mut m = model_of(&PROFILES[GENERIC], temp_factor, 1.0, 1.0);
        m.r_base = r_base;
        m
    }

    fn golden_temp_factor(profile: usize, t_c: f64) -> f64 {
        GOLDEN_ROWS.iter().find(|r| r.profile == profile && r.t_c == t_c).expect("the grid holds this temperature").temp_factor
    }

    /// Relative-or-absolute closeness for the closed-form and quadrature
    /// checks.
    fn assert_rel(got: f64, want: f64, rel: f64, label: &str) {
        let bound = rel * want.abs().max(1e-300);
        assert!((got - want).abs() <= bound, "{label}: got {got}, want {want}, off by {} (bound {bound})", (got - want).abs());
    }

    // ---- the golden table, bit for bit ----------------------------------

    #[test]
    fn golden_matches_classics_battery_physics_bit_for_bit() {
        assert_eq!(GOLDEN_ROWS.len(), 4 * 10 * 8, "four profiles, ten SoC points, eight temperatures");
        assert_eq!(PROFILES.len(), 4);
        for (i, row) in GOLDEN_ROWS.iter().enumerate() {
            let profile = &PROFILES[row.profile];
            let model = model_of(profile, row.temp_factor, 1.0, 1.0);
            let label = format!("row {i} ({} soc {} at {} C)", profile.name, row.soc, row.t_c);

            assert_eq!(model.ocv(row.soc).to_bits(), row.ocv.to_bits(), "{label}: OCV {} vs {}", model.ocv(row.soc), row.ocv);
            assert_eq!(model.resistance(row.soc).to_bits(), row.r.to_bits(), "{label}: R {} vs {}", model.resistance(row.soc), row.r);

            let prev = BatteryState { soc: row.soc, discharged: PREV_DISCHARGED, rejected: PREV_REJECTED };
            let step = model.advance(&prev, row.i_a * row.dt_s);
            assert_eq!(step.requested.to_bits(), row.requested.to_bits(), "{label}: requested");
            assert_eq!(step.accepted.to_bits(), row.accepted.to_bits(), "{label}: accepted");
            assert_eq!(step.rejected.to_bits(), row.rejected_step.to_bits(), "{label}: rejected this step");
            assert_eq!(step.state.soc.to_bits(), row.next_soc.to_bits(), "{label}: next soc");
            assert_eq!(step.state.discharged.to_bits(), row.next_discharged.to_bits(), "{label}: discharged");
            assert_eq!(step.state.rejected.to_bits(), row.next_rejected.to_bits(), "{label}: rejected");
            assert_eq!(step.depleted, row.depleted, "{label}: depleted");
        }
    }

    #[test]
    fn golden_reaches_every_branch_of_the_step() {
        let capped = GOLDEN_ROWS.iter().filter(|r| r.accepted < r.requested).count();
        let reverse = GOLDEN_ROWS.iter().filter(|r| r.rejected_step > 0.0).count();
        let untouched = GOLDEN_ROWS.iter().filter(|r| r.accepted == 0.0 && r.requested == 0.0 && r.rejected_step == 0.0).count();
        assert!(capped > 0 && reverse > 0, "capped {capped}, reverse {reverse}");
        assert_eq!(untouched, 0, "every row moves some charge one way or the other");
    }

    // ---- probe values ----------------------------------------------------

    #[test]
    fn supply_5v_full_charge_resistance_is_the_probe_p3_values() {
        // Probe P3 (draft section 8): 0.8040943233289921 at 25 C, 0.9 at 21 C,
        // the AA pack's reference temperature.
        let aa = &PROFILES[AA_4S];
        let at_25 = model_of(aa, golden_temp_factor(AA_4S, 25.0), 5.0 / 6.0, 1.0);
        assert_eq!(at_25.resistance(1.0), 0.8040943233289921);
        let at_21 = model_of(aa, golden_temp_factor(AA_4S, 21.0), 5.0 / 6.0, 1.0);
        assert_eq!(at_21.resistance(1.0), 0.9);
    }

    #[test]
    fn supply_5v_open_circuit_voltage_is_scaled_to_five_thirds_of_a_cell() {
        // Probe P3: fresh OCV 5.3333 V (6.4 V table top scaled by 5/6).
        let model = model_of(&PROFILES[AA_4S], 1.0, 5.0 / 6.0, 1.0);
        assert_rel(model.emf(1.0), 5.0 + 1.0 / 3.0, 1e-15, "scaled OCV at full charge");
        assert_eq!(model.ocv(1.0), 6.4, "the table itself stays unscaled");
    }

    #[test]
    fn cr2032_at_its_cold_bound_is_fifty_ohms() {
        // Gate 1 (battery-engine-physics.test.ts:116-135) expects 50 to 8
        // digits; the golden pins the exact double.
        let model = model_of(&PROFILES[CR2032], golden_temp_factor(CR2032, -30.0), 1.0, 1.0);
        assert!((model.resistance(1.0) - 50.0).abs() < 5e-9, "R(1) at -30 C is {}", model.resistance(1.0));
        let warm = model_of(&PROFILES[CR2032], golden_temp_factor(CR2032, 21.0), 1.0, 1.0);
        assert!((warm.resistance(1.0) - 10.0).abs() < 5e-9, "and 10 ohm at 21 C, got {}", warm.resistance(1.0));
    }

    #[test]
    fn resistance_of_an_empty_generic_cell_is_eleven_times_fresh() {
        // README, Rails point 6, "Empty": 1 + depA/depK = 1 + 0.5/0.05 = 11.
        let model = generic_model(0.9, 1.0);
        assert_rel(model.resistance(0.0), 0.9 * 11.0, 1e-12, "R(0)");
        assert_eq!(model.ocv(0.0), 0.0, "and the OCV is 0 V");
    }

    #[test]
    fn a_zero_base_resistance_is_zero_at_every_soc() {
        let model = generic_model(0.0, 2.5);
        for soc in [0.0, 1e-9, 0.02, 0.5, 0.99, 1.0] {
            assert_eq!(model.resistance(soc), 0.0, "soc {soc}");
        }
        assert_eq!(model.emf(0.5), 2.5, "an ideal rail still has its OCV: 5 V * soc");
    }

    // ---- OCV table interpolation ------------------------------------------

    #[test]
    fn ocv_interpolates_the_table_and_clamps_outside_it() {
        // The AA cell table x4 (BP:147-158), unscaled.
        let aa = model_of(&PROFILES[AA_4S], 1.0, 1.0, 1.0);
        assert_eq!(aa.ocv(0.0), 0.0);
        assert_eq!(aa.ocv(0.02), 3.2, "on a knot");
        assert_rel(aa.ocv(0.2), (4.08 + 4.72) / 2.0, 1e-15, "halfway between the 0.1 and 0.3 knots");
        assert_rel(aa.ocv(0.01), 1.6, 1e-15, "halfway up the collapse to 0 V");
        assert_eq!(aa.ocv(1.0), 6.4);
        assert_eq!(aa.ocv(7.0), 6.4, "SoC clamps to 1 first");
        assert_eq!(aa.ocv(-3.0), 0.0, "and to 0");
        assert_eq!(aa.ocv(f64::NAN), 0.0, "and a non-finite SoC is 0");
    }

    #[test]
    fn ocv_below_the_first_knot_is_that_knot_and_a_repeated_knot_is_a_step() {
        // Table [0.2, 1.0], [0.5, 2.0], [0.5, 3.0], [1.0, 4.0]: nothing below
        // 0.2; a step at 0.5.
        let mut params = vec![0.0; battery_param::COUNT];
        params[battery_param::TEMP_FACTOR] = 1.0;
        params[battery_param::DEP_K] = 0.05;
        params[battery_param::CAPACITY_AH] = 1.0;
        let blob = [4.0, 0.2, 1.0, 0.5, 2.0, 0.5, 3.0, 1.0, 4.0];
        let model = BatteryModel::from_record(&params, &blob);
        assert_eq!(model.ocv(0.0), 1.0);
        assert_eq!(model.ocv(0.2), 1.0);
        assert_rel(model.ocv(0.35), 1.5, 1e-15, "between 0.2 and 0.5");
        assert_eq!(model.ocv(0.5), 2.0, "the first point with soc <= soc_i bounds the segment");
        assert!(model.ocv(0.5000001) > 3.0, "past the step it runs up from the upper value");
        assert_eq!(model.ocv(1.0), 4.0);
    }

    #[test]
    fn a_single_point_table_is_a_constant() {
        let mut params = vec![0.0; battery_param::COUNT];
        params[battery_param::TEMP_FACTOR] = 1.0;
        params[battery_param::DEP_K] = 0.05;
        params[battery_param::CAPACITY_AH] = 1.0;
        let model = BatteryModel::from_record(&params, &[1.0, 0.5, 3.3]);
        for soc in [0.0, 0.5, 0.9, 1.0] {
            assert_eq!(model.ocv(soc), 3.3, "soc {soc}");
        }
    }

    // ---- SoC integral -------------------------------------------------------

    #[test]
    fn half_an_amp_for_a_second_on_one_amp_hour_drains_the_exact_fraction() {
        let model = generic_model(0.9, 1.0);
        let step = model.advance(&BatteryState::initial(&model), held_charge(0.5, 1.0));
        assert_eq!(step.accepted, 0.5);
        assert_eq!(step.state.soc, 1.0 - 0.5 / 3600.0);
        assert_eq!(step.state.discharged, 0.5);
        assert_eq!(step.state.rejected, 0.0);
        assert!(!step.depleted);
    }

    #[test]
    fn soc_is_independent_of_how_an_interval_is_split() {
        // Coulomb counting is linear in Q, so events at any spacing leave the
        // same SoC (the host's choice of t_end changes nothing).
        let model = generic_model(0.9, 1.0);
        let whole = model.advance(&BatteryState::initial(&model), 0.5 * 100.0);
        let mut state = BatteryState::initial(&model);
        for _ in 0..1000 {
            state = model.advance(&state, 0.5 * 100.0 / 1000.0).state;
        }
        assert_rel(state.soc, whole.state.soc, 1e-13, "soc");
        assert_rel(state.discharged, whole.state.discharged, 1e-13, "discharged");
    }

    #[test]
    fn reverse_charge_goes_to_rejected_only() {
        let model = generic_model(0.9, 1.0);
        let start = BatteryState { soc: 0.6, discharged: 2.0, rejected: 1.0 };
        let step = model.advance(&start, -0.75);
        assert_eq!(step.requested, 0.0);
        assert_eq!(step.accepted, 0.0);
        assert_eq!(step.rejected, 0.75);
        assert_eq!(step.state, BatteryState { soc: 0.6, discharged: 2.0, rejected: 1.75 });
    }

    #[test]
    fn a_cell_accepts_no_more_than_it_holds() {
        // 1 Ah = 3600 C; at soc 0.25 it holds 900 C. Ask for 1000.
        let model = generic_model(0.9, 1.0);
        let step = model.advance(&BatteryState { soc: 0.25, discharged: 0.0, rejected: 0.0 }, 1000.0);
        assert_eq!(step.requested, 1000.0);
        assert_eq!(step.accepted, 900.0);
        assert_eq!(step.state.soc, 0.0);
        assert_eq!(step.state.discharged, 900.0);
        assert!(step.depleted);
        let again = model.advance(&step.state, 5.0);
        assert_eq!(again.accepted, 0.0, "an empty cell accepts nothing further");
        assert_eq!(again.state.discharged, 900.0);
    }

    #[test]
    fn a_non_finite_charge_counts_as_zero() {
        let model = generic_model(0.9, 1.0);
        let start = BatteryState::initial(&model);
        for q in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(model.advance(&start, q).state, start, "Q = {q}");
        }
    }

    #[test]
    fn the_initial_state_is_soc0_with_nothing_counted() {
        let mut model = generic_model(0.9, 1.0);
        model.soc0 = 0.3;
        assert_eq!(BatteryState::initial(&model), BatteryState { soc: 0.3, discharged: 0.0, rejected: 0.0 });
    }

    /// Composite 5-point Gauss-Legendre, which sees only the integrand: the
    /// independent reference for the closed form. On the smooth exponentials
    /// used here its truncation error is far below 1e-12.
    fn quadrature(f: impl Fn(f64) -> f64, t_end: f64) -> f64 {
        const NODES: [f64; 5] = [0.0, 0.5384693101056831, -0.5384693101056831, 0.906179845938664, -0.906179845938664];
        const WEIGHTS: [f64; 5] = [0.5688888888888889, 0.47862867049936647, 0.47862867049936647, 0.23692688505618908, 0.23692688505618908];
        const PANELS: usize = 400;
        let h = t_end / PANELS as f64;
        let mut sum = 0.0;
        for p in 0..PANELS {
            let mid = (p as f64 + 0.5) * h;
            for (x, w) in NODES.iter().zip(WEIGHTS) {
                sum += w * f(mid + 0.5 * h * x);
            }
        }
        sum * 0.5 * h
    }

    #[test]
    fn a_closed_form_term_matches_numerical_quadrature_to_1e_12() {
        // I(t) = a + b vc(t), vc(t) = inf + (vc0 - inf) e^(rate t): a decaying
        // capacitor into a load (rate < 0), a growing one, and a slow one.
        let cases = [
            (0.01, 0.002, 3.0, 5.0, -1000.0, 2e-3),
            (-0.004, 0.0007, 4.5, 1.0, -37.5, 0.08),
            (0.002, 0.001, 2.0, 0.5, 50.0, 0.01),
            (0.0, 1e-3, 5.0, 0.0, -1e-3, 100.0),
            (0.03, -0.002, 1.5, 4.0, -1e5, 1e-4),
        ];
        for (a, b, inf, vc0, rate, dt) in cases {
            let want = quadrature(|t| a + b * (inf + (vc0 - inf) * (rate * t).exp()), dt);
            let got = exp_term_charge(a, b, inf, vc0, rate, dt);
            assert_rel(got, want, 1e-12, &format!("a {a} b {b} inf {inf} vc0 {vc0} rate {rate} dt {dt}"));
        }
    }

    #[test]
    fn a_closed_form_term_with_no_rate_is_the_held_current() {
        // rate = 0: vc stays at vc0, so the term is (a + b vc0) dt exactly.
        assert_eq!(exp_term_charge(0.01, 0.002, 3.0, 5.0, 0.0, 2.0), (0.01 + 0.002 * 5.0) * 2.0);
        // A rate so small that e^(rate dt) - 1 would lose every digit as a
        // difference is still the limit.
        let tiny = exp_term_charge(0.01, 0.002, 3.0, 5.0, 1e-14, 1.0);
        assert_rel(tiny, (0.01 + 0.002 * 5.0) * 1.0, 1e-12, "1e-14 rate");
        // rate * dt underflows to 0: the term is (a + b vc0) dt.
        let under = exp_term_charge(0.01, 0.002, 3.0, 5.0, 1e-300, 1e-300);
        assert_rel(under, (0.01 + 0.002 * 5.0) * 1e-300, 1e-12, "underflowing rate * dt");
    }

    #[test]
    fn the_integral_feeds_the_step_as_one_charge() {
        // A 1 mAh generic cell: a closed-form term draining at 0.5 mA on
        // average for 100 s takes 0.05 C of 3.6 C.
        let mut model = generic_model(0.9, 1.0);
        model.capacity_ah = 1e-3;
        let q = exp_term_charge(0.0, 1.0, 0.0, 0.5e-3, 0.0, 100.0);
        assert_rel(q, 0.05, 1e-15, "Q");
        let step = model.advance(&BatteryState::initial(&model), q);
        assert_rel(step.state.soc, 1.0 - 0.05 / 3.6, 1e-15, "soc");
    }

    // ---- knots ---------------------------------------------------------------

    #[test]
    fn the_next_knot_below_is_the_largest_strictly_lower_one_or_zero() {
        // AA table knots: 0, 0.02, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0.
        let aa = model_of(&PROFILES[AA_4S], 1.0, 1.0, 1.0);
        assert_eq!(aa.next_knot_below(1.0), Some(0.9));
        assert_eq!(aa.next_knot_below(0.95), Some(0.9));
        assert_eq!(aa.next_knot_below(0.9), Some(0.7), "on a knot: the one below");
        assert_eq!(aa.next_knot_below(0.9 + 1e-13), Some(0.7), "within the reached tolerance counts as on it");
        assert_eq!(aa.next_knot_below(0.05), Some(0.02));
        assert_eq!(aa.next_knot_below(0.02), Some(0.0));
        assert_eq!(aa.next_knot_below(0.001), Some(0.0));
        assert_eq!(aa.next_knot_below(0.0), None, "nothing below empty");
        assert_eq!(aa.next_knot_below(f64::NAN), None, "a non-finite SoC is empty");
    }

    #[test]
    fn a_table_that_starts_above_zero_still_ends_at_zero() {
        let mut params = vec![0.0; battery_param::COUNT];
        params[battery_param::TEMP_FACTOR] = 1.0;
        params[battery_param::DEP_K] = 0.05;
        params[battery_param::CAPACITY_AH] = 1.0;
        let model = BatteryModel::from_record(&params, &[2.0, 0.2, 1.0, 1.0, 4.0]);
        assert_eq!(model.next_knot_below(0.9), Some(0.2));
        assert_eq!(model.next_knot_below(0.2), Some(0.0));
        assert_eq!(model.next_knot_below(0.1), Some(0.0));
    }

    #[test]
    fn a_full_scale_generic_cell_is_a_two_point_line() {
        let model = generic_model(0.9, 1.0);
        assert_eq!(model.next_knot_below(1.0), Some(0.0));
        assert_rel(model.emf(0.5), 2.5, 1e-15, "5 V * soc");
    }

    #[test]
    fn clamp_soc_matches_classics_clamp() {
        assert_eq!(clamp_soc(0.4), 0.4);
        assert_eq!(clamp_soc(-0.0).to_bits(), 0.0f64.to_bits(), "-0 becomes +0, as Math.max(0, ...) does");
        assert_eq!(clamp_soc(1.5), 1.0);
        assert_eq!(clamp_soc(f64::INFINITY), 0.0, "non-finite is 0, not 1");
        assert_eq!(clamp_soc(f64::NAN), 0.0);
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for BatteryModel {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ocv_scale, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.r_base, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.temp_factor, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dep_a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dep_p, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dep_k, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.capacity_ah, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.soc0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.table, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for BatteryState {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.soc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.discharged, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rejected, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SocStep {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.state, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.requested, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.accepted, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rejected, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.depleted, w); }
        n
    }
}
