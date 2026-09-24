/**
 * Runs one CorpusEntry through the harness's per-entry state machine (the
 * worker brief's "Harness semantics"): not-ported short-circuits before
 * either engine touches the circuit; a CoreUnsupportedError from the core
 * side is an expected fallback (core-declined), not a failure; anything else
 * that throws is an "error" entry; otherwise the two sample sequences are
 * compared and the entry is pass or fail.
 *
 * CORE RUNS FIRST. That matches production's own precedence — HeadlessRunner
 * in `engine: "core"` mode tries the core and falls back to classic, never
 * the other way (headless.ts's `_tryLoadCore`) — and it means a circuit the
 * core declines outright is reported as "core-declined" without classic
 * having to run (or survive its own convergence guard, sample.ts's
 * collectClassicSamples) first. Some topologies the core explicitly rejects
 * (a loop of ideal sources, a floating ideal source — core/README.md's W0
 * kernel semantics #1) are ALSO singular for classic's own MNA; running
 * classic only once the core has already accepted the circuit means that
 * guard only has to hold for entries a real comparison is actually going to
 * happen on.
 */

import { CoreUnsupportedError } from "../../../src/core/errors.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { compareSamples } from "./compare.js";
import { missingCoreKinds } from "./kinds-of.js";
import { collectClassicSamples, collectCoreSamples, DEFAULT_DURATION_S, sampleTimesFor } from "./sample.js";
import type { CorpusEntry, EntryResult } from "./types.js";

/** The first ~10 mismatches are reported per the brief; the rest are still counted (FailResult.totalMismatches). */
const MAX_REPORTED_MISMATCHES = 10;

function errorMessage(err: unknown): string {
  return err instanceof Error ? err.message : String(err);
}

export function runCorpusEntry(entry: CorpusEntry, circuit: SimCircuit): EntryResult {
  const missingKinds = missingCoreKinds(entry.kinds);
  if (missingKinds.length > 0) {
    return { status: "not-ported", kinds: entry.kinds, missingKinds, classicWallMs: null, coreWallMs: null };
  }

  const times = sampleTimesFor(entry.durationS ?? DEFAULT_DURATION_S);

  let coreSamples;
  let coreWallMs: number;
  try {
    const started = performance.now();
    coreSamples = collectCoreSamples(circuit, times);
    coreWallMs = performance.now() - started;
  } catch (err) {
    if (err instanceof CoreUnsupportedError) {
      return {
        status: "core-declined",
        kinds: entry.kinds,
        reasons: [...err.reasons],
        classicWallMs: null,
        coreWallMs: null,
      };
    }
    return {
      status: "error",
      kinds: entry.kinds,
      engine: "core",
      message: errorMessage(err),
      classicWallMs: null,
      coreWallMs: null,
    };
  }

  let classicSamples;
  let classicWallMs: number;
  try {
    const started = performance.now();
    classicSamples = collectClassicSamples(circuit, times);
    classicWallMs = performance.now() - started;
  } catch (err) {
    return {
      status: "error",
      kinds: entry.kinds,
      engine: "classic",
      message: errorMessage(err),
      classicWallMs: null,
      coreWallMs,
    };
  }

  const cmp = compareSamples(classicSamples, coreSamples);
  const errorStats = {
    maxAbsErrorVolts: cmp.maxAbsErrorVolts,
    maxRelErrorVolts: cmp.maxRelErrorVolts,
    maxAbsErrorAmps: cmp.maxAbsErrorAmps,
    maxRelErrorAmps: cmp.maxRelErrorAmps,
  };

  if (cmp.mismatches.length === 0) {
    return { status: "pass", kinds: entry.kinds, classicWallMs, coreWallMs, ...errorStats };
  }
  return {
    status: "fail",
    kinds: entry.kinds,
    classicWallMs,
    coreWallMs,
    mismatches: cmp.mismatches.slice(0, MAX_REPORTED_MISMATCHES),
    totalMismatches: cmp.mismatches.length,
    ...errorStats,
  };
}
