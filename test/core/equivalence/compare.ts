/**
 * compareSamples() is the harness's INTERNAL SEAM: it takes two plain arrays
 * of SampleReadout (no engine, no CoreEngine handle, no circuit) and reports
 * where they disagree. run-entry.ts is the only production caller, feeding
 * it two readouts of the SAME circuit from the two engines — but because the
 * function itself only sees data, equivalence.test.ts's detector self-tests
 * can call it directly with readouts that were never produced by a matched
 * pair of engine runs (a genuinely different circuit, or a hand-edited
 * readout) to prove the comparison logic actually fires before trusting a
 * clean sweep on real circuits. That is deliberate, not a loophole: it is
 * the only way to test a detector without editing src to break it.
 */

import { withinTolerance, VOLT_ABS_TOL, VOLT_REL_TOL, AMP_ABS_TOL, AMP_REL_TOL } from "./tolerance.js";
import type { CompareResult, Mismatch, SampleReadout } from "./types.js";

interface ErrorAccumulator {
  maxAbs: number;
  maxRel: number;
}

function compareNumericRecord(
  category: "netV" | "elementI",
  sampleTime: number,
  classic: Record<string, number>,
  core: Record<string, number>,
  absTol: number,
  relTol: number,
  mismatches: Mismatch[],
  errAcc: ErrorAccumulator,
): void {
  const keys = new Set([...Object.keys(classic), ...Object.keys(core)]);
  for (const key of keys) {
    const inClassic = Object.prototype.hasOwnProperty.call(classic, key);
    const inCore = Object.prototype.hasOwnProperty.call(core, key);
    if (!inClassic || !inCore) {
      mismatches.push({
        sampleTime,
        category,
        id: key,
        classicValue: inClassic ? classic[key] : undefined,
        coreValue: inCore ? core[key] : undefined,
        detail: `present only on the ${inClassic ? "classic" : "core"} side`,
      });
      continue;
    }

    const a = classic[key]!;
    const b = core[key]!;
    const absErr = Math.abs(a - b);
    const denom = Math.max(Math.abs(a), Math.abs(b));
    const relErr = denom > 0 ? absErr / denom : 0;
    // Recorded for every compared pair, not just violations — the table
    // wants "how close agreement actually is" (~1e-12 expected in W0), not
    // just a pass/fail bit.
    errAcc.maxAbs = Math.max(errAcc.maxAbs, absErr);
    errAcc.maxRel = Math.max(errAcc.maxRel, relErr);

    if (!withinTolerance(a, b, absTol, relTol)) {
      mismatches.push({
        sampleTime,
        category,
        id: key,
        classicValue: a,
        coreValue: b,
        detail: `|${a} - ${b}| = ${absErr.toExponential(3)} exceeds tolerance ${(absTol + relTol * denom).toExponential(3)}`,
      });
    }
  }
}

function compareDigitalState(
  sampleTime: number,
  classic: Record<string, number>,
  core: Record<string, number>,
  mismatches: Mismatch[],
): void {
  // This is a RAW per-sample comparator: it assumes `classic`/`core` were
  // already sampled at the same instant (compareSamples's own simTime check
  // is the belt-and-suspenders half of that assumption) and says nothing
  // about clock-cycle skew. For a real digital kind, classic's load() seed
  // can produce a spurious extra clock edge at t=0 (decision 8) that the
  // core's exact-crossing settle does not, so a naive instant-for-instant
  // diff would flag that skew as a mismatch. run-entry.ts's runCorpusEntry
  // handles that SEPARATELY and BEFORE this function ever sees a digital
  // key: for an entry that declares CorpusEntry.digital (types.ts), it
  // builds full-resolution traces (trace.ts), aligns them per clock cycle
  // (align.ts), and folds any resulting mismatch into the SAME list this
  // function's caller already assembles — known-divergences.ts's registry is
  // then applied to the combined list, so an owner-approved divergence
  // (e.g. the SAP-1 display's startup phantom edge) is reported, not failed,
  // regardless of which of the two paths produced it. This function itself
  // stays a plain per-sample compare, correct as written for: (a) W0/W1a,
  // where no kind publishes digitalState at all (both sides are always
  // `{}`); and (b) any future entry that does NOT declare `digital` — e.g. a
  // digital signal sampled well after it has settled, where cycle skew does
  // not matter.
  const keys = new Set([...Object.keys(classic), ...Object.keys(core)]);
  for (const key of keys) {
    const a = classic[key];
    const b = core[key];
    if (!Object.is(a, b)) {
      mismatches.push({
        sampleTime,
        category: "digitalState",
        id: key,
        classicValue: a,
        coreValue: b,
        detail: a === undefined ? "present only on the core side" : b === undefined ? "present only on the classic side" : "digital state differs",
      });
    }
  }
}

/** Presence only — a SimFailure's own field values are not compared (the harness's job is topology/value equivalence, not failure-message wording). */
function compareFailureKeys(
  sampleTime: number,
  classic: Record<string, unknown>,
  core: Record<string, unknown>,
  mismatches: Mismatch[],
): void {
  const keys = new Set([...Object.keys(classic), ...Object.keys(core)]);
  for (const key of keys) {
    const inClassic = key in classic;
    const inCore = key in core;
    if (inClassic !== inCore) {
      mismatches.push({
        sampleTime,
        category: "failures",
        id: key,
        classicValue: inClassic ? classic[key] : undefined,
        coreValue: inCore ? core[key] : undefined,
        detail: "failure presence differs",
      });
    }
  }
}

/**
 * Direct check that the two sides were actually sampled at the same
 * instant, independent of whatever collectClassicSamples/collectCoreSamples
 * did to get there. `1e-12 * max(1, |sampleTime|)` is a tight, near-exact
 * tolerance — generous enough for float accumulation across successive
 * classic `run()` calls, tight enough that a sample taken at a genuinely
 * different instant (e.g. a classic run that quietly stopped short) still
 * fires this. This is the belt to collectClassicSamples's suspenders (its
 * own header): even a divergence that collector fails to catch up front
 * still surfaces here as a labelled mismatch instead of a silent pass.
 */
function compareSimTime(sampleTime: number, classicSimTime: number, coreSimTime: number, mismatches: Mismatch[]): void {
  const tol = 1e-12 * Math.max(1, Math.abs(sampleTime));
  const absErr = Math.abs(classicSimTime - coreSimTime);
  if (absErr > tol) {
    mismatches.push({
      sampleTime,
      category: "simTime",
      id: "simTime",
      classicValue: classicSimTime,
      coreValue: coreSimTime,
      detail:
        `|${classicSimTime} - ${coreSimTime}| = ${absErr.toExponential(3)} exceeds tolerance ${tol.toExponential(3)} — ` +
        "the two sides were sampled at different instants.",
    });
  }
}

/** Codes only, as a sorted multiset — a SimWarning's `since`/`value` are readout-specific, not part of "did the same conditions latch". */
function compareWarningCodes(
  sampleTime: number,
  classic: Record<string, { code: string }>,
  core: Record<string, { code: string }>,
  mismatches: Mismatch[],
): void {
  const classicCodes = Object.values(classic).map((w) => w.code).sort();
  const coreCodes = Object.values(core).map((w) => w.code).sort();
  const same =
    classicCodes.length === coreCodes.length && classicCodes.every((code, i) => code === coreCodes[i]);
  if (!same) {
    mismatches.push({
      sampleTime,
      category: "warnings",
      id: "*",
      classicValue: classicCodes,
      coreValue: coreCodes,
      detail: "warning code sets differ",
    });
  }
}

/**
 * Compares two equal-length, time-aligned SampleReadout sequences. Throws
 * (a harness bug, not a circuit fact) if the sequences have different
 * lengths — that means the two sides were sampled on different schedules,
 * which no caller in this package should ever produce.
 */
export function compareSamples(classic: readonly SampleReadout[], core: readonly SampleReadout[]): CompareResult {
  if (classic.length !== core.length) {
    throw new Error(
      `compareSamples: sample count mismatch (classic ${classic.length}, core ${core.length}) — ` +
        "the two runs used different sample schedules; this is a harness bug, not a circuit result.",
    );
  }

  const mismatches: Mismatch[] = [];
  const voltErr: ErrorAccumulator = { maxAbs: 0, maxRel: 0 };
  const ampErr: ErrorAccumulator = { maxAbs: 0, maxRel: 0 };

  for (let i = 0; i < classic.length; i++) {
    const c = classic[i]!;
    const r = core[i]!;
    // classic's own simTime labels the sample — after successive run() calls
    // its accumulated float error (if any) is the more informative value to
    // report a mismatch against, and it is always close enough to r.simTime
    // (both target the same t_i) that using one or the other never changes
    // which sample a reported mismatch points at.
    const sampleTime = c.simTime;
    compareSimTime(sampleTime, c.simTime, r.simTime, mismatches);
    compareNumericRecord("netV", sampleTime, c.netV, r.netV, VOLT_ABS_TOL, VOLT_REL_TOL, mismatches, voltErr);
    compareNumericRecord("elementI", sampleTime, c.elementI, r.elementI, AMP_ABS_TOL, AMP_REL_TOL, mismatches, ampErr);
    compareDigitalState(sampleTime, c.digitalState, r.digitalState, mismatches);
    compareFailureKeys(sampleTime, c.failures, r.failures, mismatches);
    compareWarningCodes(sampleTime, c.warnings, r.warnings, mismatches);
  }

  return {
    mismatches,
    maxAbsErrorVolts: voltErr.maxAbs,
    maxRelErrorVolts: voltErr.maxRel,
    maxAbsErrorAmps: ampErr.maxAbs,
    maxRelErrorAmps: ampErr.maxRel,
  };
}
