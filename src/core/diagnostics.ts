/**
 * `SolverDiagnostics` for the core (W2 design 2.5, D17 as Dev ruled): the
 * stat counters (`bo_engine_stat`, core/README.md "Stat indices") mapped onto
 * classic's diagnostics fields where, and only where, the meaning matches.
 * Everything else classic reports (the adaptive step, the step cap, matrix
 * health, error control) has no core counterpart and is left out of the
 * record rather than filled with a plausible number.
 *
 * A host reads the counters before and after one batch and passes both here;
 * the counters are cumulative since load, so the batch is the difference.
 * Every counter is published under `core` with its own name: none of them
 * means what a similarly named classic field means (events processed is not
 * `steps`, solution-cache hits are not LU reuse, IC evaluations are not
 * truth-table cache misses, the largest partition is not the MNA size, a
 * failed advance is not a rejected step). The record outside `core` carries
 * only `engine` and the host's batch timing.
 *
 * Pure data shaping, like readout.ts: no ABI calls except `readCoreStats`.
 */

import type { CoreSolverDiagnostics, CoreStatDeltas } from "../sim/messages.js";
import type { CoreEngine } from "./engine.js";

/** Counters this module reads: indices 0-26, the table as of F5 (stats 25 and 26 are F5's). */
export const CORE_STAT_COUNT = 27;

/** A read of the first {@link CORE_STAT_COUNT} counters. */
export type CoreStatSnapshot = readonly number[];

/** Index of each counter this module uses (core/README.md "Stat indices"). */
const STAT = {
  staticSolves: 1,
  largestPartition: 2,
  eventsProcessed: 4,
  deltaPasses: 5,
  maxDeltaDepth: 6,
  solutionCacheHits: 7,
  closedFormSegments: 8,
  crossings: 9,
  steppedSteps: 10,
  newtonIterations: 11,
  icEvaluations: 12,
  driverCommits: 13,
  railMoves: 14,
  railSensitivityComputations: 16,
  railScans: 17,
  railEvents: 18,
  sourceEdges: 19,
  readerFlips: 20,
  slowIntervalCloses: 21,
  slowEventsFired: 22,
  latches: 23,
  junctionRearms: 24,
  powerHoldsTaken: 25,
  powerHoldsReleased: 26,
} as const;

export function readCoreStats(engine: CoreEngine): CoreStatSnapshot {
  return engine.statCounters(CORE_STAT_COUNT);
}

/** What the host knows about the batch that the counters cannot say. */
export interface CoreBatchInfo {
  /** False when the batch's advance failed (`CoreEngine.lastConverged`). */
  readonly converged: boolean;
  readonly batchWallUs?: number;
  readonly batchCompletion?: number;
  readonly realtimeRatio?: number;
}

export function coreSolverDiagnostics(
  before: CoreStatSnapshot,
  after: CoreStatSnapshot,
  info: CoreBatchInfo,
): CoreSolverDiagnostics {
  // A counter that went backwards means a load happened between the reads
  // (counters restart at load): the batch's count is then the new total.
  const delta = (index: number): number => {
    const now = after[index] ?? 0;
    const was = before[index] ?? 0;
    if (!Number.isFinite(now)) return 0;
    return now >= was || !Number.isFinite(was) ? now - (Number.isFinite(was) ? was : 0) : now;
  };
  const level = (index: number): number => {
    const now = after[index] ?? 0;
    return Number.isFinite(now) ? now : 0;
  };

  // Stat 1 (static solves) is a load-time constant, so a batch delta of it is
  // always 0: it is not published. Stat 2 is a level (the largest partition,
  // in nodes), published as `largestPartitionNodes`, not classic's MNA size.
  const core: Record<string, number | boolean> = {};
  for (const key of Object.keys(STAT) as (keyof typeof STAT)[]) {
    if (key === "staticSolves" || key === "largestPartition") continue;
    core[key] = key === "maxDeltaDepth" ? level(STAT[key]) : delta(STAT[key]);
  }
  core.largestPartitionNodes = level(STAT.largestPartition);

  const steppedSteps = delta(STAT.steppedSteps);
  const newtonIterations = delta(STAT.newtonIterations);

  const diagnostics: CoreSolverDiagnostics = {
    engine: "core",
    core: core as unknown as CoreStatDeltas,
  };
  diagnostics.core.advanceFailed = !info.converged;
  // Newton iterations per stepped step: a batch with none has no such mean.
  if (steppedSteps > 0) diagnostics.core.meanNewtonItersPerSteppedStep = newtonIterations / steppedSteps;
  if (info.batchWallUs !== undefined) diagnostics.batchWallUs = info.batchWallUs;
  if (info.batchCompletion !== undefined) diagnostics.batchCompletion = info.batchCompletion;
  if (info.realtimeRatio !== undefined) diagnostics.realtimeRatio = info.realtimeRatio;
  return diagnostics;
}
