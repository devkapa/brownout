/**
 * Per-clock-cycle digital alignment (decision 6, gate 2a). A digital trace
 * (trace.ts) is reduced to one projected state per clock cycle — the settled
 * state just before each rising edge of the fixture's declared clock key —
 * then the two engines' cycle sequences are lined up on a declared anchor
 * state before being compared index by index.
 *
 * WHY ALIGNMENT AT ALL. Classic's `load()` seed can produce a spurious extra
 * clock edge at t=0 (decision 8's phantom edge) that the core's own load
 * settling does not reproduce identically, so cycle N of one engine's trace
 * is not always cycle N of the other's — the W1-0 spike measured this at
 * exactly 6 cycles on every SAP-1 board. Comparing by absolute time or by
 * raw cycle index would misreport that skew as thousands of mismatches.
 * Comparing by a matched ANCHOR STATE instead finds the true offset once,
 * up front, and every cycle after it is compared on equal footing.
 *
 * WHY A BOUND. An anchor is only trustworthy evidence of alignment if it is
 * rare enough to be found by ACCIDENT within a few cycles of startup but not
 * likely to recur later by coincidence in a periodic circuit (a counter
 * revisits the same nibble every 16 cycles, for instance) — bounding the
 * search to the fixture's declared `offsetBoundCycles` (default 6, per the
 * spike) keeps the match meaningful and turns "we searched forever and found
 * it eventually" into an explicit failure instead of a silent, possibly
 * wrong, alignment.
 */

import type { DigitalTraceEvent } from "./trace.js";

export type ProjectedState = Readonly<Record<string, number>>;

/** One clock cycle's settled, (optionally) projected digital state. */
export interface CycleSample {
  readonly cycleIndex: number;
  /** The clock's rising-edge instant that OPENS the next cycle; `null` for cycle 0, which is sampled at the trace's very first instant, before any edge. */
  readonly edgeTime: number | null;
  readonly state: ProjectedState;
}

function project(state: Readonly<Record<string, number>>, projection?: readonly string[]): ProjectedState {
  if (!projection) return { ...state };
  const out: Record<string, number> = {};
  for (const key of projection) {
    if (Object.prototype.hasOwnProperty.call(state, key)) out[key] = state[key]!;
  }
  return out;
}

/**
 * Rising edges of `clockKey`: every event recording a NEW value of exactly
 * `1` where the previously known value was `0` or absent. Absent-then-1
 * counts deliberately — a clock the very first sample already shows HIGH
 * (decision 8's load-time phantom edge) must be visible as an edge, not
 * silently skipped for lack of an observed prior LOW.
 */
export function findRisingEdges(trace: readonly DigitalTraceEvent[], clockKey: string): number[] {
  const edges: number[] = [];
  let last: number | undefined;
  for (const e of trace) {
    if (e.key !== clockKey) continue;
    const value = Number.isNaN(e.value) ? undefined : e.value;
    if (value === 1 && (last === undefined || last === 0)) edges.push(e.t);
    last = value;
  }
  return edges;
}

/**
 * Samples the projected state once per clock cycle: cycle 0 is the state at
 * the trace's first instant (before any edge — classic's post-load,
 * pre-first-digital-pass snapshot); cycle N (N>=1) is the settled state
 * STRICTLY BEFORE the Nth rising edge of `clockKey`, per gate 2a. Anything
 * that changes at exactly the edge instant (the clock's own transition, or a
 * zero-delay combinational fallout of it) belongs to the NEXT cycle's
 * sample, not this one — this is only observable when `projection` itself
 * includes such a same-instant key; the ordinary case (the clock key is not
 * part of the declared projection) is unaffected either way.
 */
export function sampleCycles(
  trace: readonly DigitalTraceEvent[],
  clockKey: string,
  projection?: readonly string[],
): CycleSample[] {
  const edges = findRisingEdges(trace, clockKey);
  const samples: CycleSample[] = [];
  const state: Record<string, number> = {};
  let eventIdx = 0;

  function applyThroughInclusive(limit: number): void {
    while (eventIdx < trace.length && trace[eventIdx]!.t <= limit) {
      const e = trace[eventIdx]!;
      if (Number.isNaN(e.value)) delete state[e.key];
      else state[e.key] = e.value;
      eventIdx++;
    }
  }

  function applyThroughExclusive(limit: number): void {
    while (eventIdx < trace.length && trace[eventIdx]!.t < limit) {
      const e = trace[eventIdx]!;
      if (Number.isNaN(e.value)) delete state[e.key];
      else state[e.key] = e.value;
      eventIdx++;
    }
  }

  const t0 = trace.length > 0 ? trace[0]!.t : 0;
  applyThroughInclusive(t0);
  samples.push({ cycleIndex: 0, edgeTime: null, state: project(state, projection) });

  edges.forEach((edgeTime, i) => {
    applyThroughExclusive(edgeTime);
    samples.push({ cycleIndex: i + 1, edgeTime, state: project(state, projection) });
  });

  return samples;
}

function statesEqualOnAnchor(state: ProjectedState, anchor: Readonly<Record<string, number>>): boolean {
  return Object.entries(anchor).every(([key, value]) => Object.is(state[key], value));
}

function findAnchorCycleIndexes(
  cycles: readonly CycleSample[],
  anchor: Readonly<Record<string, number>>,
  boundCycles: number,
): number[] {
  const limit = Math.min(cycles.length, boundCycles + 1);
  const hits: number[] = [];
  for (let i = 0; i < limit; i++) {
    if (statesEqualOnAnchor(cycles[i]!.state, anchor)) hits.push(cycles[i]!.cycleIndex);
  }
  return hits;
}

export interface AlignmentSuccess {
  readonly ok: true;
  readonly classicAnchorCycle: number;
  readonly coreAnchorCycle: number;
  /** classicAnchorCycle - coreAnchorCycle. Positive: classic is that many cycles "ahead" (has extra leading cycles the core does not, e.g. the phantom edge). */
  readonly offsetCycles: number;
}

export interface AlignmentFailure {
  readonly ok: false;
  readonly reason: string;
}

export type AlignmentResult = AlignmentSuccess | AlignmentFailure;

/**
 * Finds the alignment offset by locating `anchor` in each engine's cycle
 * sequence, searching only the first `offsetBoundCycles + 1` cycles of each
 * (gate 2a: "the anchor must match exactly once within the fixture's
 * declared offset bound"). Fails closed: not found, found more than once, or
 * an offset whose magnitude exceeds the bound are all reported as failures
 * with a distinguishing reason, never silently picked-one-of-many.
 */
export function alignCycles(
  classicCycles: readonly CycleSample[],
  coreCycles: readonly CycleSample[],
  anchor: Readonly<Record<string, number>>,
  offsetBoundCycles: number,
): AlignmentResult {
  const classicHits = findAnchorCycleIndexes(classicCycles, anchor, offsetBoundCycles);
  const coreHits = findAnchorCycleIndexes(coreCycles, anchor, offsetBoundCycles);

  if (classicHits.length === 0) {
    return {
      ok: false,
      reason: `anchor state not found in the classic trace within its first ${offsetBoundCycles + 1} cycle(s)`,
    };
  }
  if (classicHits.length > 1) {
    return {
      ok: false,
      reason: `anchor state matched ${classicHits.length} times in the classic trace within the bound (cycles ${classicHits.join(", ")}) — must match exactly once`,
    };
  }
  if (coreHits.length === 0) {
    return {
      ok: false,
      reason: `anchor state not found in the core trace within its first ${offsetBoundCycles + 1} cycle(s)`,
    };
  }
  if (coreHits.length > 1) {
    return {
      ok: false,
      reason: `anchor state matched ${coreHits.length} times in the core trace within the bound (cycles ${coreHits.join(", ")}) — must match exactly once`,
    };
  }

  // Both hits were found inside [0, offsetBoundCycles] by construction (the
  // search itself is bounded to that window on each side, deliberately —
  // searching one side without a bound risks a spurious match on a later,
  // coincidentally-identical cycle in a periodic circuit), so their
  // difference can never itself exceed the bound. In practice, THIS is how
  // "an offset beyond the bound" actually manifests: the true offset is not
  // found at all within the window, one of the `.length === 0` cases above.
  // The check below is kept as a defensive invariant assertion, not a path
  // expected to be live today, in case a future change searches the two
  // sides asymmetrically.
  const offsetCycles = classicHits[0]! - coreHits[0]!;
  if (Math.abs(offsetCycles) > offsetBoundCycles) {
    return {
      ok: false,
      reason: `alignment offset of ${offsetCycles} cycle(s) (classic cycle ${classicHits[0]}, core cycle ${coreHits[0]}) exceeds the fixture's bound of ${offsetBoundCycles}`,
    };
  }
  return { ok: true, classicAnchorCycle: classicHits[0]!, coreAnchorCycle: coreHits[0]!, offsetCycles };
}

export interface CycleMismatch {
  readonly classicCycle: number;
  readonly coreCycle: number;
  readonly key: string;
  readonly classicValue: number | undefined;
  readonly coreValue: number | undefined;
}

export interface AlignedComparison {
  readonly offsetCycles: number;
  /** Cycles actually compared after alignment — the span gate 2a's declared minimum is checked against. */
  readonly comparedCycles: number;
  readonly mismatches: readonly CycleMismatch[];
}

/** Zips the two cycle sequences from their matched anchors onward and compares every declared (or whole) projected key, exactly (digital values are integers — no volt/amp tolerance applies here). */
export function compareAlignedCycles(
  classicCycles: readonly CycleSample[],
  coreCycles: readonly CycleSample[],
  alignment: AlignmentSuccess,
): AlignedComparison {
  const classicStart = classicCycles.findIndex((c) => c.cycleIndex === alignment.classicAnchorCycle);
  const coreStart = coreCycles.findIndex((c) => c.cycleIndex === alignment.coreAnchorCycle);
  const span = Math.min(classicCycles.length - classicStart, coreCycles.length - coreStart);

  const mismatches: CycleMismatch[] = [];
  for (let i = 0; i < span; i++) {
    const classicCycle = classicCycles[classicStart + i]!;
    const coreCycle = coreCycles[coreStart + i]!;
    const keys = new Set([...Object.keys(classicCycle.state), ...Object.keys(coreCycle.state)]);
    for (const key of keys) {
      const a = classicCycle.state[key];
      const b = coreCycle.state[key];
      if (!Object.is(a, b)) {
        mismatches.push({ classicCycle: classicCycle.cycleIndex, coreCycle: coreCycle.cycleIndex, key, classicValue: a, coreValue: b });
      }
    }
  }

  return { offsetCycles: alignment.offsetCycles, comparedCycles: span, mismatches };
}

export class TooFewComparedCyclesError extends Error {
  constructor(comparedCycles: number, minRequired: number) {
    super(
      `Only ${comparedCycles} cycle(s) were compared after alignment, short of the fixture's declared ` +
        `minimum of ${minRequired} (gate 2a: "the compared span must reach the fixture's declared minimum ` +
        `... a shorter span fails").`,
    );
    this.name = "TooFewComparedCyclesError";
  }
}

export function requireMinComparedCycles(comparison: AlignedComparison, minRequired: number): void {
  if (comparison.comparedCycles < minRequired) {
    throw new TooFewComparedCyclesError(comparison.comparedCycles, minRequired);
  }
}
