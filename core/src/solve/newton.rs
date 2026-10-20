//! Newton's method for a partition whose stamp depends on the current
//! solution guess — pec3.mjs's `_solve` (`:407-426`), classic's `newton.ts`
//! (NW:73-130). Three kinds of partition are nonlinear in that sense, and all
//! run through [`solve`] with `linear = false`:
//! - a partition holding a free IC supply node (W1c point 3), whose driver
//!   and pull conductances follow the iterate through `R_out`;
//! - a partition holding a stamped LED junction (W1e, LEDs points 5 and 6),
//!   whose companion is re-linearised about the iterate;
//! - a partition holding both (LEDs point 9), one loop restamping both.
//!
//! A `static_part` partition of those kinds is iterated in place, from its
//! last published voltages, and (when it holds a junction) polished once
//! after it passes; a `stepped` partition iterates each BE/trap step from the
//! step's own start and never polishes. Every other partition is linear
//! (`linear = true`) and takes exactly one iteration.
//!
//! [`solve`] does not turn "hit the cap unconverged" into an `Err`: it
//! returns the last iterate with `converged = false`, because what that
//! means differs by caller (a static partition is `-4`, a stepped step is
//! rejected and retried smaller). It does turn a singular or non-finite
//! pivot, and a non-finite entry in any iterate (NW:109-111), into an `Err`:
//! those are `-4` at once for everyone.

use crate::lu;

/// Newton tolerance and iteration cap (W1 plan row B3; pec3.mjs
/// `NEWTON_ABS`/`NEWTON_REL`/`NEWTON_MAX`, `:28`).
pub const NEWTON_ABS: f64 = 1e-6;
pub const NEWTON_REL: f64 = 1e-3;
pub const NEWTON_MAX: u32 = 50;

/// The result of a Newton solve.
#[derive(Debug, Clone)]
pub struct NewtonResult {
    pub x: Vec<f64>,
    /// Every iteration run, the polish included (contract stat 11).
    pub iterations: u32,
    /// Whether every one of the first `n` unknowns satisfied the tolerance
    /// on the iteration that produced `x` — always `true` for a `linear`
    /// solve (the one-shot linear answer is exact, not an iterate that
    /// happened to move less than the tolerance), so a caller that only
    /// cares "did this settle" never has to special-case the linear path.
    pub converged: bool,
}

/// One restamp and one dense solve about `x`, with the factors it solved
/// through (`lu::solve` is exactly this factor and apply).
fn iterate<F>(size: usize, x: &[f64], stamp: &mut F) -> Result<(Vec<f64>, lu::LuFactorization), String>
where
    F: FnMut(&[f64], &mut [f64], &mut [f64]),
{
    let mut a = vec![0.0; size * size];
    let mut b = vec![0.0; size];
    stamp(x, &mut a, &mut b);
    let fail = |_| "singular or non-finite pivot in a Newton iteration".to_string();
    let factors = lu::lu_factor(a, size).map_err(fail)?;
    factors.solve(&mut b).map_err(fail)?;
    let solved = b;
    // A finite pivot at every step does not rule out an overflowing entry,
    // and a NaN in the iterate would compare "converged" against itself
    // forever below (NaN > tol is false).
    if solved.iter().any(|v| !v.is_finite()) {
        return Err("non-finite entry in a Newton iterate".to_string());
    }
    Ok((solved, factors))
}

/// Solves `A(x) x = b(x)` starting from `x_init`, re-stamping via `stamp` on
/// every iteration: `stamp(x, a, b)` fills `size x size` row-major `a` and
/// length-`size` `b` from scratch for the CURRENT guess `x` (both arrive
/// zeroed; the closure need not clear them itself).
///
/// `n <= size` is the number of NODE-VOLTAGE unknowns; any further row
/// (`n..size`, a branch-current unknown, as `solve::closed_form`'s segment
/// system or a future nonlinear kind's own extra unknowns might add) settles
/// exactly given the node voltages and is never itself checked for
/// convergence — pec3.mjs's own convergence loop only ever runs `for (let k
/// = 0; k < n; k++)` (`:417`).
///
/// An iterate passes when every node has `|x' - x| <= NEWTON_ABS + NEWTON_REL
/// * max(|x'|, |x|)`, classic's own form (NW:112-114). At most
/// [`NEWTON_MAX`] iterations.
///
/// `linear` short-circuits to exactly one iteration regardless of what the
/// convergence check computes — pec3.mjs's `!part.nonlinear` branch
/// (`:423`): a linear partition's stamp cannot move on a second pass.
/// Passing `false` for an actually-linear stamp would only cost one wasted
/// re-stamp confirming the same answer, never change it.
///
/// `polish` (ignored when `linear`): once an iterate passes, run exactly one
/// more iteration from it, untested, and return its result (LEDs point 6.4).
/// Classic re-solves a quiet partition on every step, so its steady state is
/// the fixed point, not the first passing iterate; a static partition is
/// solved once per change and takes that iteration at once.
pub fn solve<F>(size: usize, n: usize, x_init: &[f64], linear: bool, polish: bool, stamp: F) -> Result<NewtonResult, String>
where
    F: FnMut(&[f64], &mut [f64], &mut [f64]),
{
    solve_keeping_factors(size, n, x_init, linear, polish, stamp).map(|(result, _)| result)
}

/// [`solve`], also returning the factors of the last iteration it ran (the
/// polish included): a hub partition's rail sensitivity solves through the
/// last iteration's Jacobian (README, Rails point 3).
pub fn solve_keeping_factors<F>(size: usize, n: usize, x_init: &[f64], linear: bool, polish: bool, mut stamp: F) -> Result<(NewtonResult, lu::LuFactorization), String>
where
    F: FnMut(&[f64], &mut [f64], &mut [f64]),
{
    debug_assert!(n <= size);
    debug_assert_eq!(x_init.len(), size);
    let (forced_unconverged, polish) = test_overrides(linear, polish);
    #[cfg(test)]
    if !linear {
        hooks::record_initial_guess(x_init);
    }
    let max_iterations = if forced_unconverged { FORCED_UNCONVERGED_ITERATIONS } else { NEWTON_MAX };
    let mut x = x_init.to_vec();
    let mut iterations = 0u32;

    loop {
        iterations += 1;
        let (solved, factors) = iterate(size, &x, &mut stamp)?;

        let mut converged = !forced_unconverged;
        for k in 0..n {
            let (new, old) = (solved[k], x[k]);
            if (new - old).abs() > NEWTON_ABS + NEWTON_REL * new.abs().max(old.abs()) {
                converged = false;
            }
        }
        x = solved;

        if linear {
            return Ok((NewtonResult { x, iterations, converged: true }, factors));
        }
        if converged {
            if polish {
                iterations += 1;
                let (polished, polish_factors) = iterate(size, &x, &mut stamp)?;
                return Ok((NewtonResult { x: polished, iterations, converged: true }, polish_factors));
            }
            return Ok((NewtonResult { x, iterations, converged: true }, factors));
        }
        if iterations >= max_iterations {
            return Ok((NewtonResult { x, iterations, converged: false }, factors));
        }
    }
}

/// How many iterations a solve forced unconverged by the test hook runs
/// before giving up: enough to be a real iteration, few enough that a test
/// that forces every solve to fail stays fast.
#[cfg(test)]
const FORCED_UNCONVERGED_ITERATIONS: u32 = 3;
#[cfg(not(test))]
const FORCED_UNCONVERGED_ITERATIONS: u32 = NEWTON_MAX;

#[cfg(not(test))]
fn test_overrides(_linear: bool, polish: bool) -> (bool, bool) {
    (false, polish)
}

/// Test-only fault injection and observation, per thread so parallel tests
/// never see each other's. A hook is armed by a guard and disarmed when the
/// guard drops. None of this exists in a non-test build.
#[cfg(test)]
pub(crate) mod hooks {
    use std::cell::{Cell, RefCell};

    thread_local! {
        /// `(solves to let through, solves to force unconverged)`.
        static FORCE_UNCONVERGED: Cell<(u32, u32)> = const { Cell::new((0, 0)) };
        static SKIP_POLISH: Cell<bool> = const { Cell::new(false) };
        static RECORDED_GUESSES: RefCell<Option<Vec<Vec<f64>>>> = const { RefCell::new(None) };
    }

    /// Disarms every hook on drop.
    pub struct Guard;

    impl Drop for Guard {
        fn drop(&mut self) {
            FORCE_UNCONVERGED.with(|c| c.set((0, 0)));
            SKIP_POLISH.with(|c| c.set(false));
            RECORDED_GUESSES.with(|c| *c.borrow_mut() = None);
        }
    }

    /// The next `count` nonlinear solves report `converged = false` (after a
    /// few real iterations): a partition that cannot settle. `u32::MAX` never
    /// runs out.
    pub fn force_unconverged(count: u32) -> Guard {
        force_unconverged_after(0, count)
    }

    /// As [`force_unconverged`], after letting the next `skip` nonlinear
    /// solves through untouched: aims the fault at one solve in a sequence.
    pub fn force_unconverged_after(skip: u32, count: u32) -> Guard {
        FORCE_UNCONVERGED.with(|c| c.set((skip, count)));
        Guard
    }

    /// Every polish is skipped: the seeded fault the polish test must catch.
    pub fn skip_polish() -> Guard {
        SKIP_POLISH.with(|c| c.set(true));
        Guard
    }

    /// Records the initial guess of every nonlinear solve from now on, so a
    /// test can see which state a solve was warm-started from.
    pub fn record_initial_guesses() -> Guard {
        RECORDED_GUESSES.with(|c| *c.borrow_mut() = Some(Vec::new()));
        Guard
    }

    /// The guesses recorded since [`record_initial_guesses`], in order.
    pub fn recorded_initial_guesses() -> Vec<Vec<f64>> {
        RECORDED_GUESSES.with(|c| c.borrow().clone().unwrap_or_default())
    }

    pub(super) fn record_initial_guess(x_init: &[f64]) {
        RECORDED_GUESSES.with(|c| {
            if let Some(v) = c.borrow_mut().as_mut() {
                v.push(x_init.to_vec());
            }
        });
    }

    pub(super) fn take_forced() -> bool {
        FORCE_UNCONVERGED.with(|c| {
            let (skip, count) = c.get();
            if skip > 0 {
                c.set((skip - 1, count));
                false
            } else if count == 0 {
                false
            } else if count == u32::MAX {
                true
            } else {
                c.set((0, count - 1));
                true
            }
        })
    }

    pub(super) fn polish_skipped() -> bool {
        SKIP_POLISH.with(|c| c.get())
    }
}

#[cfg(test)]
fn test_overrides(linear: bool, polish: bool) -> (bool, bool) {
    if linear {
        return (false, polish);
    }
    (hooks::take_forced(), polish && !hooks::polish_skipped())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A one-node circuit: a 1 kΩ resistor from a 10 V source to the node,
    /// and a synthetic diode-like nonlinear conductance from the node to
    /// ground (pec3.mjs's own diode companion, `:394-403`, used here purely
    /// as a `#[cfg(test)]` nonlinearity — no diode kind exists in W1b). The
    /// stamp genuinely depends on the current guess `v` (through `gd`/`id`'s
    /// exponential), so this exercises real re-linearizing iteration, unlike
    /// every real W1b kind.
    fn diode_clamped_divider(v_source: f64, r: f64) -> impl FnMut(&[f64], &mut [f64], &mut [f64]) {
        move |x: &[f64], a: &mut [f64], b: &mut [f64]| {
            let v = x[0];
            let g_r = 1.0 / r;
            a[0] += g_r;
            b[0] += g_r * v_source;

            let (is, vt_n, v_sat) = (1e-12, 0.02585, 0.7);
            let vd = v; // diode's cathode is grounded, so vd == the node voltage.
            let clamped = vd.min(v_sat);
            let e = (clamped / vt_n).exp();
            let gd = is * e / vt_n;
            let id = is * (e - 1.0) + gd * (if vd > v_sat { vd - v_sat } else { 0.0 });
            let ieq = id - gd * vd;
            // stamp_two_terminal's (Some(la), None) case with the fixed pin
            // at 0 V: a[la,la] += g; b[la] += g*0 - ieq.
            a[0] += gd;
            b[0] -= ieq;
        }
    }

    #[test]
    fn a_linear_stamp_converges_in_exactly_one_iteration() {
        let stamp = |_x: &[f64], a: &mut [f64], b: &mut [f64]| {
            a[0] = 1.0 / 1000.0;
            b[0] = 10.0 / 1000.0;
        };
        let result = solve(1, 1, &[0.0], true, false, stamp).unwrap();
        assert_eq!(result.iterations, 1);
        assert!(result.converged);
        assert!((result.x[0] - 10.0).abs() < 1e-12);
    }

    #[test]
    fn a_nonlinear_stamp_needs_more_than_one_iteration_and_converges() {
        let result = solve(1, 1, &[0.0], false, false, diode_clamped_divider(10.0, 1000.0)).unwrap();
        assert!(result.converged);
        assert!(result.iterations > 1, "expected genuine Newton iteration, got {} pass(es)", result.iterations);
        // A silicon-diode-like clamp on a 10V/1k source settles in the
        // 0.5-0.7V range, nowhere near the unclamped linear divider's 10V.
        assert!(result.x[0] > 0.3 && result.x[0] < 0.8, "got {}", result.x[0]);

        // The converged point must actually satisfy KCL for THIS x, not just
        // have stopped moving: re-stamping at the answer and re-solving must
        // reproduce it to within Newton's own tolerance (`NEWTON_ABS +
        // NEWTON_REL*|x|` — the same bound `solve`'s own stopping check uses,
        // not an arbitrarily tighter one: the LAST accepted iterate is only
        // ever guaranteed to be within that of a further re-solve).
        let mut stamp = diode_clamped_divider(10.0, 1000.0);
        let mut a = vec![0.0; 1];
        let mut b = vec![0.0; 1];
        stamp(&result.x, &mut a, &mut b);
        let check = lu::solve(a, 1, b).unwrap();
        let tol = NEWTON_ABS + NEWTON_REL * result.x[0].abs();
        assert!((check[0] - result.x[0]).abs() <= tol, "check={} x={} tol={}", check[0], result.x[0], tol);
    }

    #[test]
    fn forcing_linear_on_a_nonlinear_stamp_still_takes_only_one_pass() {
        // `linear = true` on an actually-nonlinear stamp is a caller error in
        // general (W1b never does this — every real kind is linear), but the
        // function must not loop forever or panic; it takes exactly one
        // iteration and reports whatever that single linearization gave.
        let result = solve(1, 1, &[0.0], true, false, diode_clamped_divider(10.0, 1000.0)).unwrap();
        assert_eq!(result.iterations, 1);
        assert!(result.converged);
    }

    #[test]
    fn exhausting_newton_max_still_returns_the_last_iterate_not_an_error() {
        // A stamp that NEVER settles (each iteration reports the opposite of
        // whatever it was handed, an oscillation the tolerance never
        // satisfies) must still terminate at NEWTON_MAX, unconverged, rather
        // than loop forever.
        let stamp = |x: &[f64], a: &mut [f64], b: &mut [f64]| {
            a[0] = 1.0;
            b[0] = if x[0] > 0.5 { 0.0 } else { 1.0 };
        };
        let result = solve(1, 1, &[0.0], false, false, stamp).unwrap();
        assert_eq!(result.iterations, NEWTON_MAX);
        assert!(!result.converged);
    }
    /// The diode divider's fixed point by bisection on its own KCL, so the
    /// reference shares nothing with the iteration under test.
    fn diode_divider_fixed_point() -> f64 {
        let f = |v: f64| (10.0 - v) / 1000.0 - 1e-12 * ((v / 0.02585).exp() - 1.0);
        let (mut lo, mut hi) = (0.0f64, 0.7f64);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if f(mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    #[test]
    fn polish_runs_exactly_one_more_iteration_and_lands_on_the_fixed_point() {
        let reference = diode_divider_fixed_point();
        let plain = solve(1, 1, &[0.0], false, false, diode_clamped_divider(10.0, 1000.0)).unwrap();
        let polished = solve(1, 1, &[0.0], false, true, diode_clamped_divider(10.0, 1000.0)).unwrap();
        assert!(polished.converged);
        assert_eq!(polished.iterations, plain.iterations + 1, "the polish is one extra, counted iteration");
        let (err_plain, err_polished) = ((plain.x[0] - reference).abs(), (polished.x[0] - reference).abs());
        assert!(err_polished <= err_plain + 1e-15, "polish moved away from the fixed point: {err_polished} vs {err_plain}");
        assert!(err_polished < 1e-9, "polished result {} vs fixed point {reference}", polished.x[0]);
    }

    #[test]
    fn polish_is_ignored_for_a_linear_solve() {
        let stamp = |_x: &[f64], a: &mut [f64], b: &mut [f64]| {
            a[0] = 1.0 / 1000.0;
            b[0] = 10.0 / 1000.0;
        };
        let result = solve(1, 1, &[0.0], true, true, stamp).unwrap();
        assert_eq!(result.iterations, 1);
    }

    #[test]
    fn a_non_finite_iterate_is_an_error_not_a_converged_solve() {
        // A finite, nonzero pivot whose solution overflows: 1e-300 * x = 1e10.
        let stamp = |_x: &[f64], a: &mut [f64], b: &mut [f64]| {
            a[0] = 1e-300;
            b[0] = 1e10;
        };
        let err = solve(1, 1, &[0.0], false, false, stamp).unwrap_err();
        assert!(err.contains("non-finite"), "got {err}");
    }

    #[test]
    fn the_test_hooks_force_unconverged_and_skip_polish_then_disarm() {
        {
            let _guard = hooks::force_unconverged(1);
            let forced = solve(1, 1, &[0.0], false, false, diode_clamped_divider(10.0, 1000.0)).unwrap();
            assert!(!forced.converged);
            assert_eq!(forced.iterations, 3);
            let after = solve(1, 1, &[0.0], false, false, diode_clamped_divider(10.0, 1000.0)).unwrap();
            assert!(after.converged, "the hook was armed for one solve only");
        }
        let _guard = hooks::skip_polish();
        let plain = solve(1, 1, &[0.0], false, false, diode_clamped_divider(10.0, 1000.0)).unwrap();
        let skipped = solve(1, 1, &[0.0], false, true, diode_clamped_divider(10.0, 1000.0)).unwrap();
        assert_eq!(skipped.iterations, plain.iterations);
    }
}
