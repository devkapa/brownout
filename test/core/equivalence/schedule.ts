/**
 * Sample schedules: WHEN to sample the two engines for analog comparison.
 * W0's fixed 4-point grid (`sample.ts`'s `sampleTimesFor`: 0, D/4, D/2, D) is
 * too sparse to ever have a `quiet.ts` guard-window filter meaningfully
 * matter — with 4 points spread across a whole run, an entry either has
 * every point quiet or, if unlucky, loses most of its already-tiny sample
 * budget to one nearby event. Gate 2b's "dense schedule" replaces it with a
 * uniform, much finer grid, so excluding a handful of guard windows still
 * leaves plenty of quiet instants for `compareSamples` to check.
 *
 * This module is pure time-axis arithmetic — no circuit, no engine — so it
 * composes cleanly with `quiet.ts` (which knows nothing about how the
 * candidate times were generated) and with `sample.ts`'s existing collectors
 * (which already accept an arbitrary `times` array).
 */

const NANOSECOND_GRID = 1e9;

/** Rounds to a whole number of nanoseconds so two schedules built the same way are bit-identical — compare.ts's simTime check is exact to `1e-12 * max(1, |t|)`, tight enough that float drift from repeated addition would trip it for no circuit reason. */
function roundToNsGrid(t: number): number {
  return Math.round(t * NANOSECOND_GRID) / NANOSECOND_GRID;
}

export interface DenseScheduleOptions {
  /** Candidate instants per second of simulated time. Default 200 (a 5 ms grid): dense enough that a guard window on the order of a datasheet delay (microseconds to hundreds of nanoseconds, x10) excludes only a handful of candidates near each event, not the whole grid. */
  readonly instantsPerSecond?: number;
  /** Always include t=0 and t=durationS, so the new dense grid is a strict superset of W0's endpoints. Default true. */
  readonly includeEndpoints?: boolean;
}

/** Gate 2b's dense analog schedule: a uniform grid over `[0, durationS]`, ascending, deduplicated. */
export function denseAnalogSchedule(durationS: number, options: DenseScheduleOptions = {}): number[] {
  if (!Number.isFinite(durationS) || durationS <= 0) {
    throw new Error(`denseAnalogSchedule: durationS must be a positive finite number, got ${String(durationS)}.`);
  }
  const instantsPerSecond = options.instantsPerSecond ?? 200;
  if (!(instantsPerSecond > 0)) {
    throw new Error(`denseAnalogSchedule: instantsPerSecond must be > 0, got ${String(instantsPerSecond)}.`);
  }
  const includeEndpoints = options.includeEndpoints ?? true;
  const stepS = 1 / instantsPerSecond;

  const times = new Set<number>();
  if (includeEndpoints) times.add(0);
  for (let t = stepS; t < durationS; t += stepS) times.add(roundToNsGrid(t));
  if (includeEndpoints) times.add(roundToNsGrid(durationS));

  return [...times].sort((a, b) => a - b);
}

/** Merges any number of time arrays into one ascending, deduplicated (to the nanosecond grid) schedule — used to fold event instants, declared anchors, or another schedule into one candidate list before quiet-filtering. */
export function mergeScheduleTimes(...schedules: ReadonlyArray<readonly number[]>): number[] {
  const set = new Set<number>();
  for (const schedule of schedules) {
    for (const t of schedule) set.add(roundToNsGrid(t));
  }
  return [...set].sort((a, b) => a - b);
}
