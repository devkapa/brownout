/**
 * The core-free half of the analysis engine (W2e): the AnalysisEngine
 * interface, plan and report types, classic's engine, the run-on-plan helpers
 * and the grid stepper. engine.ts adds the core engine and the planner that
 * can choose it, and re-exports everything here.
 *
 * WHY a separate file: the runners that cannot use the core yet (SPICE
 * `.tran`, forced classic) import only this file, so the `./spice` entry does
 * not pull the core compiler, the wasm loader and CoreEngine into its module
 * graph. The planner stays synchronous (the runners are), which rules out a
 * lazy dynamic import of CoreEngine; a static split is the way to keep the
 * graph small. See engine.ts for the design (D14, D15).
 */

import { SimEngine } from "../sim/engine/sim-engine.js";
import type { SimCircuit, SimFailure } from "../sim/engine/sim-engine.js";
import type { Net } from "../sim/engine/graph.js";
import { AnalysisConvergenceError, stepAnalysisEngine } from "./checked-step.js";

// ─── Public types ────────────────────────────────────────────────────────────

/** What a spec or a caller asks for. `auto` is the default: the core when the whole run supports it, else classic. */
export type AnalysisEngineChoice = "auto" | "classic" | "core";

/** The engine a run actually used. */
export type AnalysisEngineKind = "classic" | "core";

/** `choice` plus the knob the dual-engine tests turn. */
export interface AnalysisEngineOptions {
  /** Overrides `spec.engine`. Unset: the spec's, else `auto`. */
  engine?: AnalysisEngineChoice;
  /**
   * Which kinds `auto` and `core` may route to the core. Defaults to
   * CORE_SUPPORTED_KINDS (empty until W5). Passing CORE_IMPLEMENTED_KINDS
   * routes kinds that have not cleared the accuracy gates; for tests only.
   */
  coreSupportedKinds?: ReadonlySet<string>;
}

/**
 * The restart hook the runners' hooks types share. A run that was on the core
 * and was declined by the Rust load part-way through is replayed whole on
 * classic; `onPoint` would then count the same points twice, so a progress UI
 * is told first.
 */
export interface AnalysisRestartHook {
  /**
   * Called exactly once, before the replay on classic starts and before any
   * `onPoint` of the replay, with the reasons the core declined. After it, the
   * point counter starts again from 0. Not called when the run never restarts
   * (the common core rejection is caught by the planner's first-point load,
   * before any point runs).
   */
  onRestart?: (reasons: readonly string[]) => void;
}

/** What every analysis result carries about its engine. */
export interface AnalysisEngineReport {
  engine: AnalysisEngineKind;
  /**
   * Why the run is on classic when it was not asked to be: the vocabulary of
   * HeadlessRunner.engineFallbackReasons (`unsupported-kind:<kind>`,
   * `malformed:<id>:<what>`, `topology:<message>`, `core-not-initialized`,
   * `core-poisoned`) plus the analysis-specific ones a runner forces
   * (`ac-sweep-needs-signal-gen-on-core`, `tran-needs-carry-seed`). Sorted
   * and deduplicated; empty when the run is on the core or classic was asked for.
   */
  fallbackReasons: string[];
}

export interface AdvanceOptions {
  /**
   * Classic's step for this advance. Classic takes exactly one step of this
   * size (so the float sequence of the runners' old loops is reproduced bit
   * for bit); the core ignores it and advances to `t`. Unset: `t - simTime`.
   */
  stepS?: number;
  /** Names the phase in a rejected step's message ("runSweep point 3 (x=5)"). */
  context?: string;
}

/** The slice of an engine the transient analyses read. */
export interface AnalysisEngine {
  readonly kind: AnalysisEngineKind;
  /** Simulation time (s); 0 after coldLoad. */
  readonly simTime: number;
  /** The loaded topology's nets (deterministic ids, identical on both engines). */
  readonly nets: Net[];
  /** Load `circuit` with every element state zeroed and the clock at 0. */
  coldLoad(circuit: SimCircuit): void;
  /**
   * Advance to `t`. Throws AnalysisConvergenceError if the solve did not
   * converge; the readouts are then the last trusted instant, never a failed
   * iterate. Throws a plain Error if the engine itself threw.
   */
  advanceTo(t: number, options?: AdvanceOptions): void;
  /** Net id -> volts. A live record on both engines: copy it to keep it. */
  getNetV(): Record<string, number>;
  getFailures(): Record<string, SimFailure>;
  /** Release engine resources (the core's wasm handle). Safe to call twice. */
  dispose(): void;
}

/**
 * The core declined a circuit in its Rust load (a topology it does not take,
 * a load-time livelock or solve failure) after every point had compiled.
 * `runOnPlan` turns it into a whole-run rerun on classic under `auto`.
 */
export class AnalysisCoreRejectedError extends Error {
  readonly reasons: readonly string[];
  constructor(reasons: readonly string[]) {
    super(`the core declined the circuit: ${reasons.join(", ")}`);
    this.name = "AnalysisCoreRejectedError";
    this.reasons = reasons;
  }
}

// ─── Classic ─────────────────────────────────────────────────────────────────

class ClassicAnalysisEngine implements AnalysisEngine {
  readonly kind = "classic" as const;
  private _sim: SimEngine;

  constructor(
    sim: SimEngine | null,
    /** False for `.tran`, which has always stepped unchecked: no snapshot per step, and a failed step ends the run anyway. */
    private readonly _checked: boolean,
  ) {
    this._sim = sim ?? new SimEngine();
  }

  get simTime(): number {
    return this._sim.simTime;
  }

  get nets(): Net[] {
    return this._sim.nets;
  }

  coldLoad(circuit: SimCircuit): void {
    // A fresh SimEngine per load, as every runner always did: coldLoad zeroes
    // element state, but a fresh engine also drops every cache and adaptive
    // flag, which keeps each point independent of the one before it.
    this._sim = new SimEngine();
    this._sim.coldLoad(circuit);
  }

  advanceTo(t: number, options?: AdvanceOptions): void {
    const h = options?.stepS ?? t - this._sim.simTime;
    const context = options?.context ?? "analysis step";
    if (this._checked) {
      stepAnalysisEngine(this._sim, h, context);
      return;
    }
    const startTime = this._sim.simTime;
    this._sim.step(h);
    if (!this._sim.lastConverged) {
      throw new AnalysisConvergenceError(
        `Analysis stopped because ${context} did not converge at ${String(startTime)} s.`,
      );
    }
  }

  getNetV(): Record<string, number> {
    return this._sim.getNetV();
  }

  getFailures(): Record<string, SimFailure> {
    return this._sim.getFailures();
  }

  dispose(): void {
    // Nothing to release.
  }
}

/**
 * A classic engine over a SimEngine the caller has already prepared (SPICE
 * `.tran` seeds its operating point and element state before stepping, so it
 * cannot coldLoad). Steps are unchecked, as `.tran` always was.
 */
export function adoptClassicEngine(sim: SimEngine): AnalysisEngine {
  return new ClassicAnalysisEngine(sim, false);
}

// ─── Planning ────────────────────────────────────────────────────────────────

/** A decided engine for one run. */
export interface AnalysisEnginePlan extends AnalysisEngineReport {
  /** Construct the run's engine: one for the whole run, coldLoaded per point. */
  open(): AnalysisEngine;
  /**
   * The same run on classic after the core declined it in its Rust load, or
   * null when the caller asked for the core explicitly (no silent fallback).
   */
  downgrade(reasons: readonly string[]): AnalysisEnginePlan | null;
}

export interface PlanOptions extends AnalysisEngineOptions {
  /**
   * Reasons a runner knows the core cannot take its analysis yet, so it is
   * classic regardless of the circuit. Deleting a runner's reason is the whole
   * switch-over. Ignored for an explicit `classic` (nothing fell back).
   */
  forcedClassicReasons?: readonly string[];
}

export function sortedUnique(reasons: Iterable<string>): string[] {
  return [...new Set(reasons)].sort();
}

export function classicPlan(fallbackReasons: string[]): AnalysisEnginePlan {
  return {
    engine: "classic",
    fallbackReasons,
    open: () => new ClassicAnalysisEngine(null, true),
    downgrade: () => null,
  };
}

/**
 * The plan for a runner whose analysis cannot use the core yet: classic with
 * the runner's reasons, decided without touching the core at all. Same rules
 * as planAnalysisEngine on a forced run: `classic` records nothing, `auto`
 * records the reasons, `core` is an Error naming them.
 */
export function planForcedClassic(
  choice: AnalysisEngineChoice | undefined,
  reasons: readonly string[],
): AnalysisEnginePlan {
  if ((choice ?? "auto") === "classic") return classicPlan([]);
  const sorted = sortedUnique(reasons);
  if (choice === "core") {
    throw new Error(`analysis engine "core" was requested but this run cannot use it: ${sorted.join(", ")}`);
  }
  return classicPlan(sorted);
}

export function planOptionsFor(
  specEngine: AnalysisEngineChoice | undefined,
  hooks: AnalysisEngineOptions | undefined,
  forcedClassicReasons?: readonly string[],
): PlanOptions {
  const engine = hooks?.engine ?? specEngine;
  return {
    ...(engine !== undefined ? { engine } : {}),
    ...(hooks?.coreSupportedKinds !== undefined ? { coreSupportedKinds: hooks.coreSupportedKinds } : {}),
    ...(forcedClassicReasons !== undefined ? { forcedClassicReasons } : {}),
  };
}

// ─── Running on a plan ───────────────────────────────────────────────────────

function explicitCoreRejection(reasons: readonly string[]): Error {
  return new Error(
    `analysis engine "core" was requested but the core declined the circuit: ${reasons.join(", ")}`,
  );
}

/**
 * Run `body` on the plan's engine, then dispose it. A core rejection from the
 * Rust load (the one thing compiling cannot see) discards the partial run and
 * reruns the WHOLE body on classic, so a result never mixes engines; `body`
 * receives the report current at each attempt and must put it on its result.
 * Under an explicit `core` request the rejection is an Error instead.
 * `onRestart(reasons)` is called once before the replay (see AnalysisRestartHook).
 */
export function runOnPlan<R>(
  plan: AnalysisEnginePlan,
  body: (engine: AnalysisEngine, report: AnalysisEngineReport) => R,
  onRestart?: (reasons: readonly string[]) => void,
): R {
  const engine = plan.open();
  try {
    return body(engine, { engine: plan.engine, fallbackReasons: [...plan.fallbackReasons] });
  } catch (err) {
    if (err instanceof AnalysisCoreRejectedError) {
      const next = plan.downgrade(err.reasons);
      if (next === null) throw explicitCoreRejection(err.reasons);
      engine.dispose();
      onRestart?.(err.reasons);
      return runOnPlan(next, body);
    }
    throw err;
  } finally {
    engine.dispose();
  }
}

/** `runOnPlan` for the async runners (they yield a macrotask between points). */
export async function runOnPlanAsync<R>(
  plan: AnalysisEnginePlan,
  body: (engine: AnalysisEngine, report: AnalysisEngineReport) => Promise<R>,
  onRestart?: (reasons: readonly string[]) => void,
): Promise<R> {
  const engine = plan.open();
  try {
    return await body(engine, { engine: plan.engine, fallbackReasons: [...plan.fallbackReasons] });
  } catch (err) {
    if (err instanceof AnalysisCoreRejectedError) {
      const next = plan.downgrade(err.reasons);
      if (next === null) throw explicitCoreRejection(err.reasons);
      engine.dispose();
      onRestart?.(err.reasons);
      return await runOnPlanAsync(next, body);
    }
    throw err;
  } finally {
    engine.dispose();
  }
}

/**
 * `engine.coldLoad(circuit)` with a failure wrapped in `what` (the runners'
 * "runSweep: coldLoad failed at point 3 (x=5)" prefix), except the core's
 * rejection, which must reach `runOnPlan` untouched to trigger the rerun.
 */
export function coldLoadPoint(engine: AnalysisEngine, circuit: SimCircuit, what: string): void {
  try {
    engine.coldLoad(circuit);
  } catch (err) {
    if (err instanceof AnalysisCoreRejectedError) throw err;
    throw new Error(`${what}: ${String(err)}`);
  }
}

// ─── Grid stepping ───────────────────────────────────────────────────────────

/**
 * Step `engine` to `endTime` on a fixed grid of `h`, the last step clamped to
 * land on `endTime`; `onStep(t)` runs after each step with the engine's time.
 * This is the loop every runner used to write by hand, once: the float
 * sequence (`h = min(step, remaining)`, time read back from the engine) is
 * unchanged, so classic reproduces the old runs bit for bit, and the core is
 * advanced to `t + h`, never coarser than classic's step (D15).
 */
export function stepToGrid(
  engine: AnalysisEngine,
  endTime: number,
  h: number,
  context: string,
  onStep?: (t: number) => void,
): void {
  let t = engine.simTime;
  while (t < endTime) {
    const stepH = Math.min(h, endTime - t);
    engine.advanceTo(t + stepH, { stepS: stepH, context });
    t = engine.simTime;
    onStep?.(t);
  }
}
