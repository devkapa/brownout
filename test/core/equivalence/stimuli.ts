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
 * landing in W1b). That does not exist yet, so `replayStimuliOnCore` always
 * throws `StimuliNotYetSupportedOnCoreError`: per this worker's brief, "For
 * now implement the classic side, and give the core side an interface that
 * throws 'not yet supported' clearly." Every verify-script corpus entry
 * therefore stays `CorpusEntry.stimulus === "verify-script"` / "unstimulated"
 * on the core side until W1b lands the real mechanism.
 */

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

/** Thrown by every `replayStimuliOnCore` call until owner decision 1's parameter-update event lands (W1b). */
export class StimuliNotYetSupportedOnCoreError extends Error {
  constructor() {
    super(
      "replayStimuliOnCore: the core applies a stimulus phase through a parameter-update event " +
        "(owner decision 1; core/README.md's bo_engine_set_params, ABI v2 \"W1b\"), which does not exist " +
        "yet in this worktree. Every verify-script corpus entry stays \"unstimulated\" on the core side " +
        "until then (CorpusEntry.stimulus's doc, types.ts) — this is expected, not a bug to work around.",
    );
    this.name = "StimuliNotYetSupportedOnCoreError";
  }
}

/**
 * Always throws `StimuliNotYetSupportedOnCoreError`. The signature mirrors
 * `replayStimuliOnClassic`'s so a caller can be written against the eventual
 * real implementation now; `_circuit`/`_phases` are accepted (not `never`)
 * for exactly that reason, even though this build never reads them.
 */
export function replayStimuliOnCore(_circuit: SimCircuit, _phases: readonly StimulusPhase[]): StimulusStepSample[] {
  throw new StimuliNotYetSupportedOnCoreError();
}
