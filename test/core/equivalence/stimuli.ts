/**
 * Replays a devolt verify-script's phases: `phases: [{ set: [{ id, params
 * }], durationS }]` — the corpus format a later dispatch (W1a-A6, after this
 * one) adds to the external-corpus manifest for the 6 Learn verify-script
 * entries (gate 2d).
 *
 * CLASSIC replays a phase exactly as devolt's own `simulateScript` does
 * (`apps/learn/lib/seed/simulate.ts`, read for this brief — devolt is a
 * private, off-limits repo for this worker, so this is a parallel
 * implementation against brownout's own `HeadlessRunner`, not an import):
 * apply this phase's param overrides to a working copy of the circuit, then
 * RELOAD — `HeadlessRunner.load()` delegates to `SimEngine.load()`, which
 * carries forward capacitor/inductor state for component ids that did not
 * change (devolt's invariant #4), so a toggled switch does not reset a
 * charging capacitor mid-phase — then run the phase's `durationS` at a FIXED
 * step (never adaptive: devolt's own comment on this is that a scripted
 * phase feeds an edge-finding predicate that compares consecutive samples on
 * a uniform grid, and this harness inherits the same reason to avoid
 * adaptive's non-uniform spacing here).
 *
 * THE CORE applies a toggle through a parameter-update event instead of a
 * reload (owner decision 1, `bo_engine_set_params` — core/README.md's ABI v2,
 * W1b): `replayStimuliOnCore` loads the circuit once, then for each phase
 * calls `CoreEngine.setParams` on that phase's `set` overrides BEFORE
 * advancing — never a reload — so capacitor charge, register contents and
 * pending delays all carry across a phase boundary the same way classic's own
 * carry-forward reload preserves them (core/README.md's
 * `bo_engine_set_params` doc: "All other state is kept").
 *
 * A circuit `replayStimuliOnCore` cannot even LOAD (an unimplemented kind, a
 * malformed parameter, or a topology the kernel rejects —
 * `CoreUnsupportedError`) still throws `StimuliNotYetSupportedOnCoreError`,
 * same as before this landed: none of `switch`/`push_button`/`spdt_switch`/
 * `dip_switch` are in `CORE_IMPLEMENTED_KINDS` yet (kinds.ts is the
 * orchestrator's table — this worker does not own it), so every real
 * verify-script corpus entry still reports "core-stimuli-pending" until that
 * table is updated. A caller that already routes a wider `implementedKinds`
 * set through the core (this file's own tests; eventually the harness once
 * the orchestrator's table catches up) gets the REAL replay instead.
 *
 * A phase's `set` naming a param that exists but is not live
 * (`CoreEngine.setParams`'s own check, per `CORE_RECORD_SHAPES.liveParams`)
 * is NEVER folded into "not yet supported" — it propagates as-is (a plain
 * `RangeError` naming exactly what was wrong), because silently skipping it
 * would hide a stimulus the caller asked for and thought was applied.
 */

import { CoreEngine, type CoreParamChange } from "../../../src/core/engine.js";
import { CoreUnsupportedError } from "../../../src/core/errors.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { HeadlessRunner, type HeadlessRunOptions } from "../../../src/host/headless.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";

export interface StimulusPhaseSet {
  readonly id: string;
  readonly params: Readonly<Record<string, number | string>>;
}

export interface StimulusPhase {
  /** Param overrides merged into components (by id) at the START of this phase. Omitted/empty means "no change, just run". */
  readonly set?: readonly StimulusPhaseSet[];
  readonly durationS: number;
}

/** devolt's `simulateScript` default step (`apps/learn/lib/seed/simulate.ts`'s `opts.dt ?? 1e-3`). */
export const STIMULUS_FIXED_STEP_S = 1e-3;

export interface StimulusStepSample {
  readonly phaseIndex: number;
  readonly simTime: number;
  readonly netV: Readonly<Record<string, number>>;
  readonly elementI: Readonly<Record<string, number>>;
  readonly digitalState: Readonly<Record<string, number>>;
}

function applyPhaseOverrides(circuit: SimCircuit, phase: StimulusPhase): SimCircuit {
  if (!phase.set || phase.set.length === 0) return circuit;
  const next: SimCircuit = { ...circuit, components: circuit.components.map((c) => ({ ...c })) };
  for (const override of phase.set) {
    const component = next.components.find((c) => c.id === override.id);
    if (component) component.params = { ...component.params, ...override.params };
  }
  return next;
}

/**
 * Classic replay of every phase, in order, on one `HeadlessRunner` (so
 * `simTime` and carried-forward element state continue across phases exactly
 * as `simulateScript` relies on). Returns one sample per accepted step across
 * every phase. Throws if any phase does not complete cleanly — a truncated
 * phase must not be silently compared as if it ran to `durationS`.
 */
export function replayStimuliOnClassic(circuit: SimCircuit, phases: readonly StimulusPhase[]): StimulusStepSample[] {
  let working = structuredClone(circuit);
  const runner = new HeadlessRunner();
  const samples: StimulusStepSample[] = [];

  phases.forEach((phase, phaseIndex) => {
    working = applyPhaseOverrides(working, phase);
    runner.load(working);

    const runOptions: HeadlessRunOptions = {
      durationS: phase.durationS,
      adaptive: false,
      fixedStepS: STIMULUS_FIXED_STEP_S,
      onSample: (sample) => {
        samples.push({
          phaseIndex,
          simTime: sample.simTime,
          netV: sample.netV,
          elementI: sample.elementI,
          digitalState: { ...runner.engine.digitalState },
        });
      },
    };
    const result = runner.run(runOptions);
    if (result.hitMinStep || result.hitStepCap) {
      const why = result.hitMinStep ? "hit the H_MIN floor" : "hit its step cap";
      throw new Error(
        `replayStimuliOnClassic: phase ${phaseIndex} (durationS=${phase.durationS}) ${why}, reaching only ` +
          `t=${result.simTime}s — this phase did not complete cleanly.`,
      );
    }
  });

  return samples;
}

/**
 * Thrown when `replayStimuliOnCore` cannot even LOAD the circuit onto the
 * core (`CoreUnsupportedError` from `CoreEngine.load` — an unimplemented
 * kind, a malformed parameter, or a topology the kernel rejects). Owner
 * decision 1's parameter-update event itself (`bo_engine_set_params`) has
 * landed (W1b) — this class now means "this PARTICULAR circuit's kinds
 * aren't routed through the core yet", not "the mechanism doesn't exist at
 * all". `reasons` is `CoreUnsupportedError.reasons` verbatim, and the
 * original is kept as `cause`.
 */
export class StimuliNotYetSupportedOnCoreError extends Error {
  readonly reasons: readonly string[];
  constructor(reasons: readonly string[], options?: ErrorOptions) {
    super(
      `replayStimuliOnCore: the core cannot load this circuit (${reasons.join(", ")}), so its phases were never ` +
        "replayed. The parameter-update mechanism itself (owner decision 1; core/README.md's " +
        "bo_engine_set_params, ABI v2 \"W1b\") exists — see CoreEngine.setParams — but this circuit's kind set " +
        "is not yet in CORE_IMPLEMENTED_KINDS (kinds.ts), or the core otherwise declined it. Every verify-script " +
        "corpus entry using such a kind stays \"unstimulated\" on the core side until that table catches up " +
        "(CorpusEntry.stimulus's doc, types.ts) — this is expected, not a bug to work around.",
      options,
    );
    this.name = "StimuliNotYetSupportedOnCoreError";
    this.reasons = reasons;
  }
}

/**
 * Core replay of every phase, in order, on ONE `CoreEngine` (so `simTime` and
 * every device's committed state — capacitor charge, register contents,
 * pending delays — continue across phases exactly as `bo_engine_set_params`'s
 * "all other state is kept" promises). Loads `circuit` once, unmodified, then
 * for each phase applies `phase.set` through `CoreEngine.setParams` (never a
 * reload) before advancing `phase.durationS` at the SAME fixed step
 * `replayStimuliOnClassic` uses, one sample per step, so the two sample
 * sequences line up instant for instant.
 *
 * `implementedKinds` defaults to `CORE_IMPLEMENTED_KINDS` (kinds.ts's
 * orchestrator-owned table) — the same "a caller (chiefly tests) can widen
 * this" escape hatch `compileCircuit`/`CoreEngine.load` already document, for
 * a widened set. This is what lets this file's own tests exercise the switch
 * family before that table is updated, without weakening the DEFAULT
 * behaviour a real corpus entry gets.
 *
 * Throws `StimuliNotYetSupportedOnCoreError` when the core cannot load the
 * circuit at all (see that class's doc comment). Throws a plain `Error`
 * (never that class) when a phase does not complete cleanly (non-convergence
 * — mirrors `replayStimuliOnClassic`'s own "did not complete cleanly" check)
 * or when `CoreEngine.setParams` itself rejects a change (an unknown
 * component/param, or a param that exists but is not live) — a stimulus the
 * caller asked for must fail loudly, never be silently skipped.
 */
export function replayStimuliOnCore(
  circuit: SimCircuit,
  phases: readonly StimulusPhase[],
  implementedKinds: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS,
): StimulusStepSample[] {
  const engine = new CoreEngine();
  try {
    try {
      engine.load(circuit, implementedKinds);
    } catch (err) {
      if (err instanceof CoreUnsupportedError) {
        throw new StimuliNotYetSupportedOnCoreError(err.reasons, { cause: err });
      }
      throw err;
    }

    const samples: StimulusStepSample[] = [];

    phases.forEach((phase, phaseIndex) => {
      if (phase.set && phase.set.length > 0) {
        const changes: CoreParamChange[] = phase.set.map((override) => ({
          componentId: override.id,
          params: override.params,
        }));
        engine.setParams(changes);
      }

      const phaseStartTime = engine.simTime;
      const stepCount = Math.round(phase.durationS / STIMULUS_FIXED_STEP_S);
      for (let step = 1; step <= stepCount; step++) {
        const tEnd = phaseStartTime + step * STIMULUS_FIXED_STEP_S;
        engine.advance(tEnd);
        if (!engine.lastConverged) {
          throw new Error(
            `replayStimuliOnCore: phase ${phaseIndex} (durationS=${phase.durationS}) did not converge, reaching ` +
              `only t=${engine.simTime}s — this phase did not complete cleanly.`,
          );
        }
        samples.push({
          phaseIndex,
          simTime: engine.simTime,
          netV: { ...engine.getNetV() },
          elementI: { ...engine.getElementI() },
          digitalState: { ...engine.digitalState },
        });
      }
    });

    return samples;
  } finally {
    engine.dispose();
  }
}
