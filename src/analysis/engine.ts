/**
 * The engine behind the transient analyses (W2e): one small interface the
 * sweep, Monte Carlo, AC sweep and SPICE `.tran` runners step through, over
 * either classic's checked step (SimEngine) or one reused CoreEngine.
 *
 * WHY one engine choice per RUN (D14), made after compiling every point:
 *   A sweep or Monte Carlo result is one curve or one distribution. If point
 *   37 ran on the core and point 38 on classic, any step in the curve could be
 *   an engine difference rather than a circuit fact, and the result could not
 *   say which engine produced it. So the choice is made once, after every
 *   point's circuit has been compiled (a point that only fails to compile at
 *   the end of the run would otherwise discard the work before it), and the
 *   result reports `engine` plus the `fallbackReasons` that made it classic.
 *   The one case compiling cannot see is the Rust load declining a topology;
 *   the planner loads the first point once to catch the common case, and
 *   `runOnPlan` handles a later point's decline by rerunning the WHOLE run on
 *   classic, telling the caller first (`onRestart`).
 *
 * WHY the core is advanced to each grid point (D15):
 *   The analyses sample on a fixed grid (ANALYSIS_STEP_S). The core's own
 *   stepping is free to be coarser between events, but a tail average over a
 *   sine or a switching waveform needs the value AT each grid point, so the
 *   core is advanced to every one: solved values, never interpolated, and the
 *   grid is never coarser than classic's step. Each advance is a clamp on the
 *   core's trajectory (the same D6 clamp the headless fixed grid takes).
 *
 * WHY classic's checked step is kept as the classic implementation:
 *   `stepAnalysisEngine` snapshots, steps and rolls back a failed iterate. It
 *   is the analyses' safety contract, and the classic path through this
 *   abstraction must stay bit-identical to what the runners did before it
 *   existed, so it is wrapped, not re-derived. The core reports a failed
 *   advance as `lastConverged = false` with its readouts at the last trusted
 *   instant, which is the same state a rollback restores, so it maps to the
 *   same AnalysisConvergenceError.
 *
 * WHY the shipped gate is empty today:
 *   `auto` routes to the core only for kinds in CORE_SUPPORTED_KINDS, which is
 *   empty until W5 decides the default. `coreSupportedKinds` widens it for the
 *   dual-engine tests (the HeadlessRunner option of the same name); a normal
 *   host never passes it. So today every `auto` run is classic and says why.
 */

import type { SimCircuit, SimFailure } from "../sim/engine/sim-engine.js";
import type { Net } from "../sim/engine/graph.js";
import { CoreEngine } from "../core/engine.js";
import { CoreUnsupportedError } from "../core/errors.js";
import { compileCircuit } from "../core/compile/index.js";
import { CORE_SUPPORTED_KINDS } from "../core/kinds.js";
import { unsupportedCoreKindReasons } from "../core/select.js";
import { coreReady, coreStatus } from "../core/wasm.js";
import { AnalysisConvergenceError } from "./checked-step.js";
import {
  AnalysisCoreRejectedError,
  classicPlan,
  sortedUnique,
  type AdvanceOptions,
  type AnalysisEngine,
  type AnalysisEngineChoice,
  type AnalysisEnginePlan,
  type PlanOptions,
} from "./engine-base.js";

// The core-free half (types, classic, run-on-plan, grid stepping) lives in
// engine-base.ts so SPICE `.tran` can use it without the core's module graph.
export * from "./engine-base.js";

// ─── Core ────────────────────────────────────────────────────────────────────

class CoreAnalysisEngine implements AnalysisEngine {
  readonly kind = "core" as const;
  private readonly _core = new CoreEngine();
  private _disposed = false;

  get simTime(): number {
    return this._core.simTime;
  }

  get nets(): Net[] {
    return this._core.nets;
  }

  coldLoad(circuit: SimCircuit): void {
    try {
      // `simTime: 0` is what makes a reused engine cold: a bare load() keeps
      // the engine's own clock moving forward, as classic's load() does. The
      // core carries nothing else across a load without a CARRY block.
      this._core.load(circuit, undefined, { simTime: 0 });
    } catch (err) {
      if (err instanceof CoreUnsupportedError) throw new AnalysisCoreRejectedError(err.reasons);
      throw err;
    }
  }

  advanceTo(t: number, options?: AdvanceOptions): void {
    const startTime = this._core.simTime;
    this._core.advance(t);
    if (this._core.lastConverged) return;
    const context = options?.context ?? "analysis step";
    throw new AnalysisConvergenceError(
      `Analysis stopped because ${context} did not converge at ${String(startTime)} s. ` +
      `The core kept its last completed instant (${String(this._core.simTime)} s), this ` +
      `analysis point was rejected, and no invalid values were used. Check for conflicting ` +
      `ideal sources, floating nodes, or extreme component values. Technical detail: ` +
      `advance to ${String(t)} s failed.`,
    );
  }

  getNetV(): Record<string, number> {
    return this._core.getNetV();
  }

  getFailures(): Record<string, SimFailure> {
    return this._core.getFailures();
  }

  dispose(): void {
    if (this._disposed) return;
    this._disposed = true;
    try {
      this._core.dispose();
    } catch {
      // Best-effort, as CoreEngine.dispose documents: a poisoned instance has nothing left to release.
    }
  }
}

// ─── Planning ────────────────────────────────────────────────────────────────

function corePlan(choice: AnalysisEngineChoice): AnalysisEnginePlan {
  return {
    engine: "core",
    fallbackReasons: [],
    open: () => new CoreAnalysisEngine(),
    downgrade: (reasons) => (choice === "core" ? null : classicPlan(sortedUnique(reasons))),
  };
}

/** The reasons the Rust load declines `circuit` with, or none. Uses a throwaway CoreEngine. */
function probeCoreLoad(circuit: SimCircuit): readonly string[] {
  const probe = new CoreEngine();
  try {
    probe.load(circuit, undefined, { simTime: 0 });
    return [];
  } catch (err) {
    if (err instanceof CoreUnsupportedError) return err.reasons;
    throw err;
  } finally {
    try {
      probe.dispose();
    } catch {
      // Best-effort, as CoreEngine.dispose documents.
    }
  }
}

/**
 * Decide the engine for a whole run. `circuits` is every circuit the run will
 * load (one per sweep point, per Monte Carlo run, per frequency); each is
 * compiled here, so a malformed point anywhere sends the whole run to classic
 * before any work is done.
 *
 * `core` never falls back: a circuit the core cannot take is an Error naming
 * the reasons. `auto` falls back and records them. `classic` records nothing.
 */
export function planAnalysisEngine(
  circuits: Iterable<SimCircuit>,
  options: PlanOptions = {},
): AnalysisEnginePlan {
  const choice = options.engine ?? "auto";
  if (choice === "classic") return classicPlan([]);

  const reasons = new Set<string>(options.forcedClassicReasons ?? []);
  if (reasons.size === 0) {
    if (!coreReady()) {
      reasons.add(coreStatus() === "poisoned" ? "core-poisoned" : "core-not-initialized");
    } else {
      const supported = options.coreSupportedKinds ?? CORE_SUPPORTED_KINDS;
      let first: SimCircuit | undefined;
      for (const circuit of circuits) {
        first ??= circuit;
        const unsupported = unsupportedCoreKindReasons(circuit, supported);
        for (const reason of unsupported) reasons.add(reason);
        if (unsupported.length > 0) continue;
        const compiled = compileCircuit(circuit);
        if (!compiled.ok) for (const reason of compiled.reasons) reasons.add(reason);
      }
      // Compiling cannot see the Rust load declining a topology, and the common
      // decline (parallel ideal sources, an IC without supply pins) is a
      // property of the circuit's wiring, which every point shares. One real
      // load of the first point catches it before any point runs or any
      // progress is reported; a decline that only a later point triggers is
      // still handled by runOnPlan's whole-run replay.
      if (reasons.size === 0 && first !== undefined) {
        for (const reason of probeCoreLoad(first)) reasons.add(reason);
      }
    }
  }

  if (reasons.size === 0) return corePlan(choice);
  const sorted = sortedUnique(reasons);
  if (choice === "core") {
    throw new Error(
      `analysis engine "core" was requested but this run cannot use it: ${sorted.join(", ")}`,
    );
  }
  return classicPlan(sorted);
}

/**
 * The planner options a runner takes from its spec and its caller's hooks:
 * the caller's `engine` wins over the spec's, so a host can pin an engine
 * without rewriting a saved spec.
 */
