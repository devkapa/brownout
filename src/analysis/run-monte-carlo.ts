/**
 * Pure Monte Carlo runner — runs a MonteCarloSpec headlessly against the real
 * SimEngine without any UI or WebWorker dependency.
 *
 * WHY cold-start per run:
 *   Element state (capacitor voltages, failure stress, latched failures)
 *   carries across load() calls by design.  If we reused one engine instance
 *   the initial conditions from run N-1 would corrupt run N — a capacitor
 *   pre-charged by the previous run's component values would bias the transient.
 *   coldLoad() zeros all element state so every run starts from the same known
 *   initial condition, matching run-sweep.ts's rationale.
 *
 * WHY tail averaging (same as run-sweep.ts):
 *   The final 20% of the settle window filters AC ripple while remaining trivial
 *   to compute.  Teaching-grade accuracy for the circuits we target.
 *
 * WHY uniform perturbation rather than Gaussian:
 *   Uniform over [-tol, +tol] is an honest worst-case-bounded teaching model.
 *   It requires no distribution assumption and directly corresponds to the
 *   part datasheet tolerance band (e.g. "±5%").  A Gaussian would be more
 *   statistically rigorous but would require choosing a sigma, which is an
 *   invisible extra parameter with no intuitive teaching value.
 *
 * WHY order-independent seeding (FNV-1a over "seed:run:compId:param"):
 *   Each delta is a deterministic function of (spec.seed, runIndex,
 *   componentId, paramName) only.  Changing the order of components in the
 *   circuit.components array therefore does NOT change the per-component
 *   perturbations.  This is critical for reproducibility: a structurally
 *   identical circuit saved with a different component insertion order must
 *   produce bit-identical results, otherwise "same seed" is a weaker contract
 *   than it appears to be.
 *
 * WHY population stddev:
 *   We are describing the distribution of the simulation results for the
 *   given seed and circuit — not estimating a parent population from a sample.
 *   Every drawn value IS the population for this seed, so the N denominator
 *   (population stddev) is correct.  Sample stddev (N-1) is appropriate when
 *   the values are a random subset of a larger unknown population.
 *
 * WHY two run variants:
 *   runMonteCarlo (synchronous) is the testable core loop.  runMonteCarloAsync
 *   yields a macrotask between runs so the analysis worker's event loop can
 *   process a queued "cancel" message.  Without this yield the worker handler
 *   never returns mid-run and cooperative cancellation would be a no-op —
 *   matching the rationale in run-sweep.ts.
 *
 * WHY vth_tol is NOT applied:
 *   The engine reads IC logic input thresholds from the catalog module-level
 *   catalogByKind map at engine init time (not from per-instance params).
 *   There is no params override mechanism for thresholds in v1.  Attempting
 *   to set params.v_il_max or params.v_ih_min would be silently ignored.
 *   vth_tol is therefore metadata-only in v1; the comment here serves as the
 *   reminder for a future runner extension.
 */

import { SimEngine } from "../sim/engine/sim-engine.js";
import type { SimCircuit } from "../sim/engine/sim-engine.js";
import type { ComponentKind } from "../circuit/types.js";
import { resolveCatalogPart } from "../circuit/catalog-resolver.js";
import {
  type MonteCarloSpec,
  type MonteCarloResult,
} from "./jobs.js";
import { ANALYSIS_STEP_S } from "./run-sweep.js";
import { stepAnalysisEngine } from "./checked-step.js";

// ─── Deterministic PRNG ────────────────────────────────────────────────────────

/**
 * FNV-1a 32-bit hash over an arbitrary string.
 *
 * Constants: FNV offset basis 0x811c9dc5, prime 0x01000193.
 * Replicates waveform.ts hash32 exactly — same constants, same loop, kept in
 * sync by the fact that Monte Carlo results must be bit-identical across
 * both modules when given the same seed string.
 */
function fnv1a32(input: string): number {
  let h = 0x811c9dc5;
  for (let i = 0; i < input.length; i++) {
    h ^= input.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h >>> 0;
}

/**
 * Map a 32-bit unsigned integer to a float in [0, 1) via mulberry32 single step.
 *
 * WHY mulberry32:
 *   A single 32-bit hash from FNV-1a does not have ideal bit mixing in the
 *   low bits.  One mulberry32 step avalanches all bits well, giving a
 *   high-quality uniform float with minimal code.  We need only one float per
 *   (seed, run, component, param) tuple — there is no counter state to advance.
 */
function hashToFloat(h: number): number {
  // mulberry32 single step (no counter — h IS the state).
  let t = (h + 0x6d2b79f5) >>> 0;
  t = Math.imul(t ^ (t >>> 15), t | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  t = (t ^ (t >>> 14)) >>> 0;
  // Map to [0, 1).
  return t / 0x100000000;
}

/**
 * Draw a delta uniform in [-tol, +tol] that is a pure function of the
 * string key "seed:runIndex:componentId:paramName".
 *
 * WHY this key:
 *   Including all four pieces means the same run/component/param triple always
 *   gets the same delta regardless of which other components are in the circuit
 *   or what order they appear in the components array (order independence).
 */
function perturbDelta(seed: number, runIndex: number, compId: string, paramName: string, tol: number): number {
  const key = `${String(seed)}:${String(runIndex)}:${compId}:${paramName}`;
  const u = hashToFloat(fnv1a32(key)); // u in [0, 1)
  // Map [0,1) to [-tol, +tol].
  return tol * (2 * u - 1);
}

// ─── Hooks ─────────────────────────────────────────────────────────────────────

export interface MonteCarloHooks {
  /** Called after each run completes (before the cancel check). */
  onPoint?: (runIndex: number, total: number) => void;
  /** Return true to request cooperative cancellation. */
  shouldCancel?: () => boolean;
}

// ─── Per-run runner ─────────────────────────────────────────────────────────────

interface RunResult {
  /** Tail-averaged voltage per output net. */
  means: number[];
  /** Component ids of any latched failures at this run. */
  failureCompIds: string[];
}

/**
 * Run a single Monte Carlo run: clone the circuit, perturb components, cold-load
 * a fresh engine, step to settleTimeS, and tail-average the outputs. Each step
 * is snapshotted and a non-converged iterate is restored and rejected, so it
 * can never enter the reported distribution.
 */
function runMcPoint(
  spec: MonteCarloSpec,
  circuit: SimCircuit,
  runIndex: number,
  outputNetIds: readonly string[],
): RunResult {
  // Clone the circuit so mutations do not bleed into future runs or the caller's
  // original circuit object.
  const runCircuit: SimCircuit = structuredClone(circuit);

  // ── Apply per-component perturbations ────────────────────────────────────────

  for (const comp of runCircuit.components) {
    const specs = resolveCatalogPart({
      kind: comp.kind as ComponentKind,
      params: comp.params,
      catalogUid: comp.catalogUid,
    })?.electrical_specs;
    if (!specs) continue;

    // Resistance variation: multiply params.resistance by (1 + delta).
    if (specs.value_tol != null && comp.params.resistance != null) {
      const delta = perturbDelta(spec.seed, runIndex, comp.id, "resistance", specs.value_tol);
      comp.params.resistance = Number(comp.params.resistance) * (1 + delta);
    }

    // Capacitance variation: multiply params.capacitance by (1 + delta).
    if (specs.value_tol != null && comp.params.capacitance != null) {
      const delta = perturbDelta(spec.seed, runIndex, comp.id, "capacitance", specs.value_tol);
      comp.params.capacitance = Number(comp.params.capacitance) * (1 + delta);
    }

    // Vf variation: for each of vf, vf1, vf2 (bicolor), vf_r, vf_g, vf_b (RGB).
    // The base for plain vf: params.vf if present, else catalog electrical_specs.vf.
    // For vf1/vf2/vf_r/vf_g/vf_b: only if already in params (the engine reads those
    // only from params — there is no catalog-level vf1/vf2 fallback for bicolor).
    // WHY: the engine reads `params.vf ?? default` — setting params.vf to a
    // perturbed value overrides the catalog default, which is orchestrator-verified.
    if (specs.vf_tol != null) {
      // plain vf: base is params.vf if present, else catalog specs.vf (standard diodes/LEDs).
      const vfBase = comp.params.vf != null ? Number(comp.params.vf) : specs.vf;
      if (vfBase != null) {
        const delta = perturbDelta(spec.seed, runIndex, comp.id, "vf", specs.vf_tol);
        comp.params.vf = vfBase * (1 + delta);
      }

      // bicolor vf1/vf2: only vary if already in params (engine reads params.vf1 ?? 1.8).
      for (const p of ["vf1", "vf2"]) {
        if ((comp.params as Record<string, number | string | undefined>)[p] != null) {
          const delta = perturbDelta(spec.seed, runIndex, comp.id, p, specs.vf_tol);
          comp.params[p] = Number(comp.params[p]) * (1 + delta);
        }
      }

      // RGB vf_r/vf_g/vf_b: only vary if already in params.
      for (const p of ["vf_r", "vf_g", "vf_b"]) {
        if ((comp.params as Record<string, number | string | undefined>)[p] != null) {
          const delta = perturbDelta(spec.seed, runIndex, comp.id, p, specs.vf_tol);
          comp.params[p] = Number(comp.params[p]) * (1 + delta);
        }
      }
    }

    // vth_tol: DO NOT vary — the engine reads thresholds from the catalog
    // module-level catalogByKind, not from per-instance params.  There is no
    // override mechanism in v1.  vth_tol is metadata-only.
  }

  // ── Engine run ────────────────────────────────────────────────────────────────

  const engine = new SimEngine();
  try {
    engine.coldLoad(runCircuit);
  } catch (err) {
    throw new Error(
      `runMonteCarlo: coldLoad failed at run ${String(runIndex)}: ${String(err)}`,
    );
  }

  // Tail window: average the final 20% of the settle window (mirrors run-sweep.ts).
  const tailStart = spec.settleTimeS * 0.8;

  const sums: number[] = new Array(outputNetIds.length).fill(0);
  let tailCount = 0;

  let simTime = engine.simTime;
  while (simTime < spec.settleTimeS) {
    const remaining = spec.settleTimeS - simTime;
    const h = Math.min(ANALYSIS_STEP_S, remaining);

    stepAnalysisEngine(
      engine,
      h,
      `runMonteCarlo run ${String(runIndex)}`,
    );

    simTime = engine.simTime;

    if (simTime >= tailStart) {
      const netV = engine.getNetV();
      for (let j = 0; j < outputNetIds.length; j++) {
        sums[j] += netV[outputNetIds[j]!] ?? 0;
      }
      tailCount++;
    }
  }

  const means: number[] = new Array(outputNetIds.length);
  for (let j = 0; j < outputNetIds.length; j++) {
    means[j] = tailCount > 0 ? sums[j]! / tailCount : 0;
  }

  const failures = engine.getFailures();
  const failureCompIds = Array.from(
    new Set(Object.values(failures).map((f) => f.componentId)),
  );

  return { means, failureCompIds };
}

// ─── Result builder ───────────────────────────────────────────────────────────

/**
 * Compute mean, min, max, and population stddev from a values array.
 * Returns zeros for an empty array (guard for a cancelled run with 0 runs).
 */
function computeStats(values: number[]): { mean: number; min: number; max: number; stddev: number } {
  if (values.length === 0) {
    return { mean: 0, min: 0, max: 0, stddev: 0 };
  }

  let sum = 0;
  let mn = values[0]!;
  let mx = values[0]!;

  for (const v of values) {
    sum += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }

  const mean = sum / values.length;

  // Population standard deviation: sum of squared deviations / N.
  let varSum = 0;
  for (const v of values) {
    const d = v - mean;
    varSum += d * d;
  }
  const stddev = Math.sqrt(varSum / values.length);

  return { mean, min: mn, max: mx, stddev };
}

function buildMonteCarloResult(
  spec: MonteCarloSpec,
  outputNetIds: string[],
  runValues: number[][], // runValues[runIdx][outputIdx]
  pointFailures: string[][],
  cancelled: boolean,
): MonteCarloResult {
  const outputs = outputNetIds.map((netId, j) => {
    const values = runValues.map((rv) => rv[j]!);
    const stats = computeStats(values);
    return { netId, values, ...stats };
  });

  return {
    spec,
    runs: runValues.length,
    outputs,
    pointFailures,
    cancelled,
  };
}

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * Run a Monte Carlo job synchronously and return the result.
 *
 * Intended for tests and non-worker callers.  Inside the analysis worker use
 * runMonteCarloAsync instead — a synchronous run never returns to the worker's
 * event loop, so cooperative cancellation could not work.
 *
 * @param spec    - Validated Monte Carlo specification (call validateJobSpec first).
 * @param circuit - The circuit to simulate.  Mutated via structuredClone per run.
 * @param hooks   - Optional: onPoint(r, total) for progress, shouldCancel() checked after each run.
 */
export function runMonteCarlo(
  spec: MonteCarloSpec,
  circuit: SimCircuit,
  hooks?: MonteCarloHooks,
): MonteCarloResult {
  const outputNetIds = spec.outputs.map((o) => o.netId);
  const runValues: number[][] = [];
  const pointFailures: string[][] = [];

  for (let r = 0; r < spec.runs; r++) {
    const { means, failureCompIds } = runMcPoint(spec, circuit, r, outputNetIds);
    runValues.push(means);
    pointFailures.push(failureCompIds);

    // Notify progress before the cancel check so the caller sees the completed run.
    hooks?.onPoint?.(r, spec.runs);

    if (hooks?.shouldCancel?.()) {
      return buildMonteCarloResult(spec, outputNetIds, runValues, pointFailures, true);
    }
  }

  return buildMonteCarloResult(spec, outputNetIds, runValues, pointFailures, false);
}

/**
 * Async Monte Carlo runner: identical semantics to runMonteCarlo, but yields
 * a macrotask between runs so the hosting worker's event loop can process
 * queued messages — in particular "cancel", which sets the flag behind
 * shouldCancel().  Without this yield, a cancel posted mid-run would only be
 * handled after the final run, making cooperative cancellation a no-op.
 */
export async function runMonteCarloAsync(
  spec: MonteCarloSpec,
  circuit: SimCircuit,
  hooks?: MonteCarloHooks,
): Promise<MonteCarloResult> {
  const outputNetIds = spec.outputs.map((o) => o.netId);
  const runValues: number[][] = [];
  const pointFailures: string[][] = [];

  for (let r = 0; r < spec.runs; r++) {
    const { means, failureCompIds } = runMcPoint(spec, circuit, r, outputNetIds);
    runValues.push(means);
    pointFailures.push(failureCompIds);

    hooks?.onPoint?.(r, spec.runs);

    // Macrotask yield — queued worker messages only dispatch between macrotasks.
    // A microtask (resolved Promise) would not let "cancel" arrive mid-run.
    await new Promise<void>((resolve) => setTimeout(resolve, 0));

    if (hooks?.shouldCancel?.()) {
      return buildMonteCarloResult(spec, outputNetIds, runValues, pointFailures, true);
    }
  }

  return buildMonteCarloResult(spec, outputNetIds, runValues, pointFailures, false);
}
