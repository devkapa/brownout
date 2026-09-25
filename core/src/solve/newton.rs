//! Newton's method for a partition whose stamp may depend on the current
//! solution guess — a nonlinear element re-linearizing around it each
//! iteration, pec3.mjs's `_solve` (`:407-426`). W1 plan row B3: "Newton
//! exists for the nonlinear kinds that arrive in W1c-W1e. In W1b every
//! partition is linear, so Newton converges in one iteration; test it anyway
//! with a synthetic nonlinear element."
//!
//! Every kind through W1b (`resistor`, `capacitor`, the switch family) is
//! linear, so no real caller's stamp closure ever actually depends on `x` —
//! [`solve`] with `linear = true` always takes exactly one iteration, since a
//! linear stamp cannot move on a second pass anyway. The `#[cfg(test)]`
//! module below exercises the genuinely-iterative path with a synthetic
//! diode-like nonlinearity (pec3.mjs's own diode companion, `:394-403`) that
//! a real nonlinear kind (W1c-W1e) can be ported onto unchanged.

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
    pub iterations: u32,
    /// Whether every one of the first `n` unknowns satisfied the tolerance
    /// on the iteration that produced `x` — always `true` for a `linear`
    /// solve (the one-shot linear answer is exact, not an iterate that
    /// happened to move less than the tolerance), so a caller that only
    /// cares "did this settle" never has to special-case the linear path.
    pub converged: bool,
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
/// `linear` short-circuits to exactly one iteration regardless of what the
/// convergence check computes — pec3.mjs's `!part.nonlinear` branch
/// (`:423`): every kind through W1b is linear, so `stepped`/`closed_form`
/// always pass `true` today; passing `false` for an actually-linear stamp
/// would only cost one wasted re-stamp confirming the same answer, never
/// change it.
///
/// `Err` on a singular or non-finite pivot at any iteration — the caller
/// turns this into whatever `ERR_SOLVE_FAILED` handling its own step/segment
/// solve already has.
pub fn solve<F>(size: usize, n: usize, x_init: &[f64], linear: bool, mut stamp: F) -> Result<NewtonResult, String>
where
    F: FnMut(&[f64], &mut [f64], &mut [f64]),
{
    debug_assert!(n <= size);
    debug_assert_eq!(x_init.len(), size);
    let mut x = x_init.to_vec();
    let mut iterations = 0u32;

    loop {
        iterations += 1;
        let mut a = vec![0.0; size * size];
        let mut b = vec![0.0; size];
        stamp(&x, &mut a, &mut b);
        let solved = lu::solve(a, size, b).map_err(|_| "singular or non-finite pivot in a Newton iteration".to_string())?;

        let mut converged = true;
        for k in 0..n {
            let (new, old) = (solved[k], x[k]);
            if (new - old).abs() > NEWTON_ABS + NEWTON_REL * new.abs().max(old.abs()) {
                converged = false;
            }
        }
        x = solved;

        if linear || converged || iterations >= NEWTON_MAX {
            return Ok(NewtonResult { x, iterations, converged: linear || converged });
        }
    }
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
        let result = solve(1, 1, &[0.0], true, stamp).unwrap();
        assert_eq!(result.iterations, 1);
        assert!(result.converged);
        assert!((result.x[0] - 10.0).abs() < 1e-12);
    }

    #[test]
    fn a_nonlinear_stamp_needs_more_than_one_iteration_and_converges() {
        let result = solve(1, 1, &[0.0], false, diode_clamped_divider(10.0, 1000.0)).unwrap();
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
        let result = solve(1, 1, &[0.0], true, diode_clamped_divider(10.0, 1000.0)).unwrap();
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
        let result = solve(1, 1, &[0.0], false, stamp).unwrap();
        assert_eq!(result.iterations, NEWTON_MAX);
        assert!(!result.converged);
    }
}
