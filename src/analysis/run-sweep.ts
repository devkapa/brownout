/**
 * Pure sweep runner — runs a DcSweepSpec or TempSweepSpec headlessly against
 * the real SimEngine without any UI or WebWorker dependency.
 *
 * WHY cold-start per point:
 *   Element state (capacitor voltages, failure stress, latched failures)
 *   carries across load() calls by design.  If we reused one engine instance
 *   across sweep points, hysteresis from capacitor charge at point N would
 *   corrupt the transient at point N+1 — a voltage-divider would look
 *   "correct" only because caps from a previous point happened to pre-charge
 *   to the right level.  coldLoad() zeros all element state so every point
 *   starts from the same known condition.
 *
 * WHY tail averaging:
 *   Averaging the final 20% of the settle window filters AC ripple from sine
 *   sources (the mean over whole cycles is the DC offset) while remaining
 *   computationally trivial.  min/max over the same window expose the ripple
 *   amplitude.  This is teaching-grade rather than SPICE-grade but gives
 *   honest, interpretable numbers for the circuits we target.
 *
 * WHY a fixed analysis step of 5e-5 s:
 *   The live worker uses an adaptive step (1e-8..1e-2 s).  For analysis we
 *   want reproducible, deterministic results across runs, so we use a fixed
 *   step.  5e-5 s (20 kHz) is fast enough to resolve 1 kHz waveforms with
 *   ~20 samples per cycle while keeping the per-point step count bounded:
 *   max settle 0.5 s / 5e-5 s = 10 000 steps per point, × 201 points = 2 M
 *   total steps — acceptable on a modern browser core.
 *
 * WHY two run variants:
 *   runSweep (synchronous) is the testable core loop.  runSweepAsync yields a
 *   macrotask between points; the analysis worker MUST use it, because a
 *   fully synchronous run inside one message handler would never return to
 *   the worker's event loop, so a queued "cancel" message could never be
 *   processed and cooperative cancellation would be a no-op.
 */

import { SimEngine } from "../sim/engine/sim-engine.js";
import type { SimCircuit } from "../sim/engine/sim-engine.js";
import {
  sweepValues,
  type DcSweepSpec,
  type TempSweepSpec,
  type AnalysisJobResult,
} from "./jobs.js";
import { stepAnalysisEngine } from "./checked-step.js";

// run-sweep handles dc-sweep and temp-sweep only.  AcSweepSpec is handled by
// run-ac-sweep.ts.  Using the narrower union keeps TypeScript honest about
// which fields are accessible (settleTimeS, from, to) vs the ac-sweep shape.
type DcOrTempSweepSpec = DcSweepSpec | TempSweepSpec;

/** Fixed simulation step for analysis runs.  See module-level why comment. */
export const ANALYSIS_STEP_S = 5e-5;

export interface SweepHooks {
  onPoint?: (index: number, total: number) => void;
  shouldCancel?: () => boolean;
}

interface PointResult {
  means: number[];
  mins: number[];
  maxs: number[];
  failureCompIds: string[];
}

/**
 * Run a single sweep point: clone the circuit, apply the mutation, cold-load
 * a fresh engine, step to settleTimeS, and tail-average the outputs.
 *
 * Errors inside a point abort the whole run with point-index context. Every
 * step is snapshotted; a thrown or non-converged solve is restored and rejected
 * before any voltage is sampled. The worker shell converts the error to a
 * user-visible message.
 */
function runSweepPoint(
  spec: DcOrTempSweepSpec,
  circuit: SimCircuit,
  xVal: number,
  pointIndex: number,
  outputNetIds: readonly string[],
): PointResult {
  // Clone the circuit so the mutation below does not bleed into future points
  // or into the caller's original circuit object.
  const pointCircuit: SimCircuit = structuredClone(circuit);

  // Apply the per-point mutation.
  if (spec.kind === "dc-sweep") {
    const comp = pointCircuit.components.find((c) => c.id === spec.componentId);
    if (comp) {
      comp.params[spec.param] = xVal;
    }
    // If the component doesn't exist (shouldn't happen after validateJobSpec)
    // we still run the circuit so pointFailures captures any resulting issues.
  } else {
    // temp-sweep: set environment.temperatureC, creating the environment
    // block from defaults if the circuit lacks one.  Default lux=100 matches
    // the historical per-part default (see engine._envLux fallback chain).
    if (!pointCircuit.environment) {
      pointCircuit.environment = { temperatureC: xVal, lux: 100 };
    } else {
      pointCircuit.environment = { ...pointCircuit.environment, temperatureC: xVal };
    }
  }

  // Fresh engine per point — element state must NOT carry across sweep points.
  const engine = new SimEngine();

  try {
    engine.coldLoad(pointCircuit);
  } catch (err) {
    throw new Error(
      `runSweep: coldLoad failed at point ${String(pointIndex)} (x=${String(xVal)}): ${String(err)}`,
    );
  }

  // Threshold: start averaging in the final 20% of the settle window.
  // This discards the initial transient (the first 80%) and captures the
  // steady-state or AC mean/envelope.
  const tailStart = spec.settleTimeS * 0.8;

  const sums: number[] = new Array(outputNetIds.length).fill(0);
  const mins: number[] = new Array(outputNetIds.length).fill(Infinity);
  const maxs: number[] = new Array(outputNetIds.length).fill(-Infinity);
  let tailCount = 0;

  // Step loop: fixed step until simTime reaches settleTimeS.
  let simTime = engine.simTime;
  while (simTime < spec.settleTimeS) {
    const remaining = spec.settleTimeS - simTime;
    const h = Math.min(ANALYSIS_STEP_S, remaining);

    stepAnalysisEngine(
      engine,
      h,
      `runSweep point ${String(pointIndex)} (x=${String(xVal)})`,
    );

    simTime = engine.simTime;

    // Sample during the tail window.
    if (simTime >= tailStart) {
      const netV = engine.getNetV();
      for (let j = 0; j < outputNetIds.length; j++) {
        const v = netV[outputNetIds[j]!] ?? 0;
        sums[j] += v;
        if (v < mins[j]!) mins[j] = v;
        if (v > maxs[j]!) maxs[j] = v;
      }
      tailCount++;
    }
  }

  // Compute means; guard against degenerate zero-length tail window.
  const means: number[] = new Array(outputNetIds.length);
  const outMins: number[] = new Array(outputNetIds.length);
  const outMaxs: number[] = new Array(outputNetIds.length);
  for (let j = 0; j < outputNetIds.length; j++) {
    means[j] = tailCount > 0 ? sums[j]! / tailCount : 0;
    outMins[j] = tailCount > 0 ? mins[j]! : 0;
    outMaxs[j] = tailCount > 0 ? maxs[j]! : 0;
  }

  // Record component ids of any latched failures at this point.  Sort for a
  // stable, reviewable order — Object.values iteration order is otherwise an
  // implementation detail that the determinism test would not catch.
  const failures = engine.getFailures();
  const failureCompIds = Array.from(
    new Set(Object.values(failures).map((f) => f.componentId)),
  ).sort();

  return { means, mins, maxs, failureCompIds };
}

/** Shared accumulation state for the two run variants. */
interface SweepAccumulator {
  xs: number[];
  total: number;
  outputNetIds: string[];
  xResult: number[];
  outputMeans: number[][];
  outputMins: number[][];
  outputMaxs: number[][];
  pointFailures: string[][];
}

function makeAccumulator(spec: DcOrTempSweepSpec): SweepAccumulator {
  const xs = sweepValues(spec.from, spec.to, spec.points);
  const outputNetIds = spec.outputs.map((o) => o.netId);
  return {
    xs,
    total: xs.length,
    outputNetIds,
    xResult: [],
    outputMeans: outputNetIds.map(() => []),
    outputMins: outputNetIds.map(() => []),
    outputMaxs: outputNetIds.map(() => []),
    pointFailures: [],
  };
}

function accumulatePoint(acc: SweepAccumulator, xVal: number, point: PointResult): void {
  acc.xResult.push(xVal);
  for (let j = 0; j < acc.outputNetIds.length; j++) {
    acc.outputMeans[j]!.push(point.means[j]!);
    acc.outputMins[j]!.push(point.mins[j]!);
    acc.outputMaxs[j]!.push(point.maxs[j]!);
  }
  acc.pointFailures.push(point.failureCompIds);
}

function buildResult(
  spec: DcOrTempSweepSpec,
  acc: SweepAccumulator,
  cancelled: boolean,
): AnalysisJobResult {
  return {
    spec,
    x: acc.xResult,
    outputs: acc.outputNetIds.map((netId, j) => ({
      netId,
      mean: acc.outputMeans[j]!,
      min: acc.outputMins[j]!,
      max: acc.outputMaxs[j]!,
    })),
    pointFailures: acc.pointFailures,
    cancelled,
  };
}

/**
 * Run a sweep job synchronously and return the result.
 *
 * Intended for tests and non-worker callers.  Inside the analysis worker use
 * runSweepAsync instead — a synchronous run never returns to the worker's
 * event loop, so cooperative cancellation could not work.
 *
 * @param spec    - Validated sweep specification (call validateJobSpec first).
 * @param circuit - The circuit to sweep.  Mutated via structuredClone per point.
 * @param hooks   - Optional callbacks: onPoint(i, total) for progress,
 *                  shouldCancel() checked after each point.
 */
export function runSweep(
  spec: DcOrTempSweepSpec,
  circuit: SimCircuit,
  hooks?: SweepHooks,
): AnalysisJobResult {
  const acc = makeAccumulator(spec);

  for (let i = 0; i < acc.total; i++) {
    const xVal = acc.xs[i]!;
    accumulatePoint(acc, xVal, runSweepPoint(spec, circuit, xVal, i, acc.outputNetIds));

    // Notify progress (before the cancel check so the caller sees the completed point).
    hooks?.onPoint?.(i, acc.total);

    if (hooks?.shouldCancel?.()) {
      return buildResult(spec, acc, true);
    }
  }

  return buildResult(spec, acc, false);
}

/**
 * Async sweep runner: identical semantics to runSweep, but yields a macrotask
 * between points so a hosting worker's event loop can process queued messages
 * — in particular "cancel", which sets the flag behind shouldCancel().
 * Without this yield, a cancel posted mid-run would only be handled after the
 * final point, making cooperative cancellation a no-op.
 */
export async function runSweepAsync(
  spec: DcOrTempSweepSpec,
  circuit: SimCircuit,
  hooks?: SweepHooks,
): Promise<AnalysisJobResult> {
  const acc = makeAccumulator(spec);

  for (let i = 0; i < acc.total; i++) {
    const xVal = acc.xs[i]!;
    accumulatePoint(acc, xVal, runSweepPoint(spec, circuit, xVal, i, acc.outputNetIds));

    hooks?.onPoint?.(i, acc.total);

    // Macrotask (not microtask): queued worker messages are only dispatched
    // between macrotasks, so a resolved-promise yield would not let "cancel"
    // arrive.
    await new Promise<void>((resolve) => setTimeout(resolve, 0));

    if (hooks?.shouldCancel?.()) {
      return buildResult(spec, acc, true);
    }
  }

  return buildResult(spec, acc, false);
}
