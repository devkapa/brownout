/**
 * Newton-Raphson iteration for non-linear MNA.
 *
 * Non-linear elements (diode, LED, BJT, MOSFET) contribute companion
 * stamps linearised about the previous iteration's voltage guess. The
 * Newton loop rebuilds the MNA from scratch each iteration, solves it,
 * and repeats until the voltage change between iterations falls below
 * a tolerance (or the iter cap is hit).
 *
 * We don't depend on the MNA implementation directly — the caller
 * supplies closures for `stamp`, `solve`, and `clear`. That lets us
 * unit-test this in isolation with a fake matrix, and lets `SimEngine`
 * decide how to partition linear vs non-linear stamps.
 *
 * Convergence criterion (SPICE-style mixed reltol/abstol):
 *     |x_new[k] - x_old[k]| < vAbsTol + vRelTol * max(|x_new[k]|, |x_old[k]|)
 * for every k. Default vAbsTol=1e-6, vRelTol=1e-3.
 *
 * We return the last iterate even when we fail to converge — callers
 * should surface a warning, but blowing up the simulation on a single
 * non-convergent step is worse than rendering a slightly-off frame.
 */

export interface NewtonOpts {
  /** Matrix/vector dimension (MNA size). Zero → trivially converged. */
  size: number;
  /** Initial voltage guess. Typically the last-step solution. */
  xInit: Float64Array;
  /** Clear the MNA and re-add always-on stamps (GMIN, etc.). */
  clear: () => void;
  /**
   * Stamp all elements into the (already-cleared) MNA using `xGuess`
   * as the linearisation point for non-linear elements. Linear stamps
   * (R, V, C, L) ignore the guess; non-linear stamps use it to compute
   * their companion conductance + current source.
   */
  stamp: (xGuess: Float64Array) => void;
  /** Solve the currently-stamped MNA and return a fresh solution vector. */
  solve: () => Float64Array;
  /** Max iterations before declaring non-convergence. Default 25. */
  maxIter?: number;
  /** Absolute voltage tolerance (V). Default 1e-6. */
  vAbsTol?: number;
  /** Relative voltage tolerance. Default 1e-3. */
  vRelTol?: number;
  /**
   * Returns true when the stamp pass just performed applied device limiting
   * (pnjlim clamped a junction's linearisation point away from the raw
   * guess). A limited iteration stamped a DIFFERENT system than the one the
   * delta-x test measures: the iterate can sit bit-still (e.g. a base parked
   * volts into forward bias while the limiter walks the evaluation voltage
   * up by ~n·Vt per iteration) even though the true device equations are
   * violated by many orders of magnitude. SPICE refuses to accept such an
   * iteration for the same reason; convergence requires a limiting-free
   * final iteration. Near a genuine solution the inter-iteration deltas are
   * below pnjlim's 2·n·Vt trigger, so this guard is inert on every honestly
   * converging solve.
   */
  limitedThisIteration?: () => boolean;
}

export interface NewtonResult {
  /** Last iterate — usable even when `converged === false`. */
  x: Float64Array;
  /** Iterations consumed (1..maxIter). */
  iters: number;
  /** Whether the last delta passed the tolerance check. */
  converged: boolean;
  /** Max |Δx| of the last iteration, for diagnostics. */
  lastDelta: number;
}

export function solveNonlinear(opts: NewtonOpts): NewtonResult {
  const {
    size,
    xInit,
    clear,
    stamp,
    solve,
    maxIter = 25,
    vAbsTol = 1e-6,
    vRelTol = 1e-3,
    limitedThisIteration,
  } = opts;

  if (size === 0) {
    return { x: new Float64Array(0), iters: 0, converged: true, lastDelta: 0 };
  }

  // x holds the current best estimate — start with caller's guess.
  // Explicit Float64Array type (not the generic narrowed variant that
  // TS5 infers from `new Float64Array(xInit)`) so it stays assignable
  // to whatever `solve()` returns.
  let x: Float64Array = new Float64Array(xInit);
  let lastDelta = Infinity;
  let iter = 0;

  for (iter = 1; iter <= maxIter; iter++) {
    clear();
    stamp(x);
    const xNew: Float64Array = solve();

    // Mixed abs/rel tolerance check across every variable.
    let maxDelta = 0;
    let converged = true;
    for (let k = 0; k < size; k++) {
      const a = xNew[k];
      const b = x[k];
      if (!Number.isFinite(a) || !Number.isFinite(b)) {
        return { x, iters: iter, converged: false, lastDelta: Number.POSITIVE_INFINITY };
      }
      const diff = Math.abs(a - b);
      const thresh = vAbsTol + vRelTol * Math.max(Math.abs(a), Math.abs(b));
      if (diff > thresh) converged = false;
      if (diff > maxDelta) maxDelta = diff;
    }

    lastDelta = maxDelta;
    x = xNew;

    // A small delta only proves convergence when the stamps it was measured
    // against were evaluated at the raw iterate. If a limiter clamped any
    // device this iteration, keep walking (see limitedThisIteration docs).
    if (converged && !limitedThisIteration?.()) {
      return { x, iters: iter, converged: true, lastDelta };
    }
  }

  return { x, iters: iter - 1, converged: false, lastDelta };
}

/**
 * "pnjlim" — SPICE's PN-junction voltage limiter.
 *
 * Clamps the inter-iteration voltage swing across a PN junction to
 * keep `exp(v/Vt)` from overflowing and to damp oscillations when the
 * Newton step would drive the solver into a far-from-linear region.
 *
 * `vNew`     proposed voltage (this iteration)
 * `vOld`     previous iteration's voltage
 * `VtN`      thermal voltage × emission coefficient (n · Vt)
 * `vCrit`    critical voltage — above this we log-compress the step.
 *            Typically `VtN · ln(VtN / (Is · sqrt(2)))`.
 *
 * Returns a limited voltage to use as the stamp linearisation point.
 */
export function pnjlim(
  vNew: number,
  vOld: number,
  VtN: number,
  vCrit: number,
): number {
  if (vNew > vCrit && Math.abs(vNew - vOld) > 2 * VtN) {
    if (vOld > 0) {
      const arg = 1 + (vNew - vOld) / VtN;
      if (arg > 0) return vOld + VtN * Math.log(arg);
      return vCrit;
    }
    return VtN * Math.log(vNew / VtN);
  }
  return vNew;
}
