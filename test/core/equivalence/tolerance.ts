/**
 * Equivalence tolerances: how close a core readout must be to the classic
 * engine's to count as "the same answer" (core/README.md's whole point —
 * this harness is what lets a kind graduate from CORE_IMPLEMENTED_KINDS into
 * CORE_SUPPORTED_KINDS). Per the worker brief that specified this harness:
 *
 *   volts: |a-b| <= 100e-6 + 1e-3 * max(|a|,|b|)
 *   amps:  |a-b| <= 1e-9  + 1e-3 * max(|a|,|b|)
 *
 * The volts constants are exactly adaptive-step.ts's own ABS_VOLTAGE/REL_TOL
 * (the classic step controller's accept/reject threshold). The amps ABSOLUTE
 * term is NOT that file's ABS_CURRENT (10e-9): this harness answers a
 * different question — "did the core reproduce classic's number" — than the
 * step controller's "is this trial step accurate enough to keep", and the
 * brief specified 1e-9 (tighter) for it. Kept as its own named constant
 * rather than imported, so the two concerns can't silently drift into each
 * other later.
 */

export const VOLT_ABS_TOL = 100e-6;
export const VOLT_REL_TOL = 1e-3;
export const AMP_ABS_TOL = 1e-9;
export const AMP_REL_TOL = 1e-3;

export function withinTolerance(a: number, b: number, absTol: number, relTol: number): boolean {
  if (!Number.isFinite(a) || !Number.isFinite(b)) {
    // Neither engine is expected to ever produce NaN/Infinity in W0 (the
    // compiler refuses non-finite parameters before a program is built —
    // core/README.md). Treat exact equality (both NaN, both the same
    // infinity) as agreement and anything else as a hard mismatch, the same
    // way adaptive-step.ts's normalizedError does.
    return Object.is(a, b);
  }
  return Math.abs(a - b) <= absTol + relTol * Math.max(Math.abs(a), Math.abs(b));
}

export function withinVoltTolerance(a: number, b: number): boolean {
  return withinTolerance(a, b, VOLT_ABS_TOL, VOLT_REL_TOL);
}

export function withinAmpTolerance(a: number, b: number): boolean {
  return withinTolerance(a, b, AMP_ABS_TOL, AMP_REL_TOL);
}
