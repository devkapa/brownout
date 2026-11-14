/**
 * Quiet-instant selection for analog comparison (gate 2b). A sample is only
 * trustworthy evidence of agreement between the two engines when neither one
 * has a digital event or threshold crossing near it — right at an edge, a
 * microsecond of clock skew between the two engines turns into a comparison
 * of "just before" against "just after" the same transition, which is a
 * guaranteed (and meaningless) mismatch. "Quiet" instants exclude a GUARD
 * WINDOW around every such event on EITHER side.
 */

import type { DigitalTraceEvent } from "./trace.js";

export interface GuardWindowOptions {
  /** The largest datasheet/inertial delay among the fixture's kinds (seconds). */
  readonly maxDelayS: number;
  /** A representative classic step size (seconds) — e.g. the run's own `finalH`, or H_MIN for a conservative bound. */
  readonly classicStepS: number;
}

/** Gate 2b's guard window: 10x the largest delay in play, or at least 2 classic steps, whichever is larger. */
export function guardWindowS(options: GuardWindowOptions): number {
  if (!(options.maxDelayS >= 0) || !(options.classicStepS >= 0)) {
    throw new Error(
      `guardWindowS: maxDelayS and classicStepS must both be finite and >= 0, got ` +
        `maxDelayS=${String(options.maxDelayS)}, classicStepS=${String(options.classicStepS)}.`,
    );
  }
  return Math.max(10 * options.maxDelayS, 2 * options.classicStepS);
}

/** Every distinct instant a digital event occurred in a trace.ts trace, ascending. */
export function eventInstantsFromTrace(trace: readonly DigitalTraceEvent[]): number[] {
  return [...new Set(trace.map((e) => e.t))].sort((a, b) => a - b);
}

/**
 * `candidateTimes` filtered down to instants at least `guard` seconds from
 * EVERY instant in `eventTimes` — from either engine's trace, since either
 * engine's own edge is a real discontinuity the other engine's smooth
 * trajectory is also settling around, at a slightly different instant.
 */
export function selectQuietInstants(
  candidateTimes: readonly number[],
  eventTimes: readonly number[],
  guard: number,
): number[] {
  if (!Number.isFinite(guard) || guard < 0) {
    throw new Error(`selectQuietInstants: guard must be a finite number >= 0, got ${String(guard)}.`);
  }
  const events = [...eventTimes].sort((a, b) => a - b);
  if (events.length === 0) return [...candidateTimes];
  return candidateTimes.filter((t) => events.every((e) => Math.abs(t - e) >= guard));
}

export class TooFewQuietInstantsError extends Error {
  constructor(found: number, required: number) {
    super(
      `Only ${found} quiet instant(s) survived the guard-window filter; this fixture declares a minimum ` +
        `of ${required} (gate 2b: "fewer is an error, not a pass").`,
    );
    this.name = "TooFewQuietInstantsError";
  }
}

/** Gate 2b: "Each fixture declares a minimum number of quiet samples; fewer is an error, not a pass." */
export function requireMinQuietInstants(instants: readonly number[], minRequired: number): void {
  if (instants.length < minRequired) throw new TooFewQuietInstantsError(instants.length, minRequired);
}

/**
 * Proves a candidate list is ACTUALLY guard-safe, independent of whichever
 * picker produced it — the harness's own "prove a detector fires before
 * trusting a clean sweep" rule (build log risk 3), applied to quiet-instant
 * selection specifically. `selectQuietInstants` above cannot by construction
 * produce an unsafe instant, so this exists to catch a DIFFERENT or
 * hand-rolled picker (a future caller, a test's deliberately-broken stand-in)
 * that skips the guard — see quiet.test.ts's "a quiet-instant picker that
 * would select inside a guard window" case.
 */
export function assertInstantsAreQuiet(instants: readonly number[], eventTimes: readonly number[], guard: number): void {
  const events = [...eventTimes].sort((a, b) => a - b);
  for (const t of instants) {
    const offender = events.find((e) => Math.abs(t - e) < guard);
    if (offender !== undefined) {
      throw new Error(
        `assertInstantsAreQuiet: candidate instant t=${t} is inside the guard window of the event at ` +
          `t=${offender} (|Δ|=${Math.abs(t - offender).toExponential(3)} < guard ${guard.toExponential(3)}) — not quiet.`,
      );
    }
  }
}
