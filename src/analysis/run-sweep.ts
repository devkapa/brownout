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
 * WHY an engine abstraction (engine.ts):
 *   The sweep steps an AnalysisEngine, not a SimEngine, so the same loop runs
 *   on classic's checked step or on the core (advanced to each grid point).
 *   One engine is chosen per RUN after compiling every point, and the result
 *   reports it with the reasons for any fallback; see engine.ts.
 *
 * WHY two run variants:
 *   runSweep (synchronous) is the testable core loop.  runSweepAsync yields a
 *   macrotask between points; the analysis worker MUST use it, because a
 *   fully synchronous run inside one message handler would never return to
 *   the worker's event loop, so a queued "cancel" message could never be
 *   processed and cooperative cancellation would be a no-op.
 */

import type { SimCircuit } from "../sim/engine/sim-engine.js";
import {
  sweepValues,
  type DcSweepSpec,
  type TempSweepSpec,
  type AnalysisJobResult,
} from "./jobs.js";
import {
  coldLoadPoint,
  planAnalysisEngine,
  planOptionsFor,
  runOnPlan,
  runOnPlanAsync,
  stepToGrid,
  type AnalysisEngine,
  type AnalysisEngineOptions,
  type AnalysisRestartHook,
  type AnalysisEngineReport,
} from "./engine.js";

// run-sweep handles dc-sweep and temp-sweep only.  AcSweepSpec is handled by
// run-ac-sweep.ts.  Using the narrower union keeps TypeScript honest about
// which fields are accessible (settleTimeS, from, to) vs the ac-sweep shape.
type DcOrTempSweepSpec = DcSweepSpec | TempSweepSpec;

/** Fixed simulation step for analysis runs.  See module-level why comment. */
export const ANALYSIS_STEP_S = 5e-5;

export interface SweepHooks extends AnalysisEngineOptions, AnalysisRestartHook {
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
 * The circuit for one sweep point: a clone with the point's mutation applied,
 * so the mutation never bleeds into future points or the caller's circuit.
 * Built twice per run on purpose (once for the engine choice, once to run it)
 * rather than holding every point's clone in memory.
 */
function buildSweepPointCircuit(spec: DcOrTempSweepSpec, circuit: SimCircuit, xVal: number): SimCircuit {
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
  return pointCircuit;
}

function* sweepPointCircuits(
  spec: DcOrTempSweepSpec,
  circuit: SimCircuit,
  xs: readonly number[],
): Generator<SimCircuit> {
  for (const xVal of xs) yield buildSweepPointCircuit(spec, circuit, xVal);
}

/**
 * Run a single sweep point: build the point's circuit, cold-load the run's
 * engine, advance to settleTimeS on the analysis grid, and tail-average the
 * outputs.
 *
 * Errors inside a point abort the whole run with point-index context. A
 * non-converged step is rejected (classic restores its snapshot, the core
 * keeps its last trusted instant) before any voltage is sampled. The worker
 * shell converts the error to a user-visible message.
 */
function runSweepPoint(
  engine: AnalysisEngine,
  spec: DcOrTempSweepSpec,
  circuit: SimCircuit,
  xVal: number,
  pointIndex: number,
  outputNetIds: readonly string[],
): PointResult {
  const pointCircuit = buildSweepPointCircuit(spec, circuit, xVal);

  // Cold per point — element state must NOT carry across sweep points (the
  // core engine is reused across points, so its coldLoad is what zeroes it).
  coldLoadPoint(
    engine,
    pointCircuit,
    `runSweep: coldLoad failed at point ${String(pointIndex)} (x=${String(xVal)})`,
  );

  // Threshold: start averaging in the final 20% of the settle window.
  // This discards the initial transient (the first 80%) and captures the
  // steady-state or AC mean/envelope.
  const tailStart = spec.settleTimeS * 0.8;

  const sums: number[] = new Array(outputNetIds.length).fill(0);
  const mins: number[] = new Array(outputNetIds.length).fill(Infinity);
  const maxs: number[] = new Array(outputNetIds.length).fill(-Infinity);
  let tailCount = 0;

  // Fixed grid until simTime reaches settleTimeS; the core is advanced to
  // each grid point, classic steps it (see stepToGrid).
  stepToGrid(
    engine,
    spec.settleTimeS,
    ANALYSIS_STEP_S,
    `runSweep point ${String(pointIndex)} (x=${String(xVal)})`,
    (simTime) => {
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
    },
  );

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
  report: AnalysisEngineReport,
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
    engine: report.engine,
    fallbackReasons: report.fallbackReasons,
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
  const plan = planAnalysisEngine(
    sweepPointCircuits(spec, circuit, makeAccumulator(spec).xs),
    planOptionsFor(spec.engine, hooks),
  );

  return runOnPlan(plan, (engine, report) => {
    const acc = makeAccumulator(spec);

    for (let i = 0; i < acc.total; i++) {
      const xVal = acc.xs[i]!;
      accumulatePoint(acc, xVal, runSweepPoint(engine, spec, circuit, xVal, i, acc.outputNetIds));

      // Notify progress (before the cancel check so the caller sees the completed point).
      hooks?.onPoint?.(i, acc.total);

      if (hooks?.shouldCancel?.()) {
        return buildResult(spec, acc, true, report);
      }
    }

    return buildResult(spec, acc, false, report);
  }, hooks?.onRestart);
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
  const plan = planAnalysisEngine(
    sweepPointCircuits(spec, circuit, makeAccumulator(spec).xs),
    planOptionsFor(spec.engine, hooks),
  );

  return runOnPlanAsync(plan, async (engine, report) => {
    const acc = makeAccumulator(spec);

    for (let i = 0; i < acc.total; i++) {
      const xVal = acc.xs[i]!;
      accumulatePoint(acc, xVal, runSweepPoint(engine, spec, circuit, xVal, i, acc.outputNetIds));

      hooks?.onPoint?.(i, acc.total);

      // Macrotask (not microtask): queued worker messages are only dispatched
      // between macrotasks, so a resolved-promise yield would not let "cancel"
      // arrive.
      await new Promise<void>((resolve) => setTimeout(resolve, 0));

      if (hooks?.shouldCancel?.()) {
        return buildResult(spec, acc, true, report);
      }
    }

    return buildResult(spec, acc, false, report);
  }, hooks?.onRestart);
}
