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
 * THE CORE applies a LATER phase's toggle through a parameter-update event
 * instead of a reload (owner decision 1, `bo_engine_set_params` —
 * core/README.md's ABI v2, W1b): for phase 1 onward, `replayStimuliOnCore`
 * calls `CoreEngine.setParams` on that phase's `set` overrides BEFORE
 * advancing — never a reload — so capacitor charge, register contents and
 * pending delays all carry across a phase boundary the same way classic's own
 * carry-forward reload preserves them (core/README.md's
 * `bo_engine_set_params` doc: "All other state is kept"). PHASE 0 is the one
 * exception (the investigation behind finding 12): its `set` (if any) is
 * folded into the circuit BEFORE the initial `load()`, exactly mirroring
 * classic's own phase-0 reload, instead of being applied through
 * `setParams` at t=0 — see `replayStimuliOnCore`'s own doc comment for why
 * (a set_params-at-t=0 event schedules a live transition that a genuine t=0
 * LOAD's zero-delay settle would instead have absorbed into the seed,
 * manufacturing a sample classic can never produce).
 *
 * A circuit `replayStimuliOnCore` cannot even LOAD (an unimplemented kind, a
 * malformed parameter, or a topology the kernel rejects —
 * `CoreUnsupportedError`) still throws `StimuliNotYetSupportedOnCoreError`.
 * The switch family (`switch`/`push_button`/`spdt_switch`/`dip_switch`) and
 * `capacitor` joined `CORE_IMPLEMENTED_KINDS` in W1b (kinds.ts, the
 * orchestrator's table), so a real verify-script entry built only from those
 * kinds now gets a genuine classic-vs-core stimuli replay by default; this
 * error is still reachable for any kind that has not been ported yet (e.g.
 * `led`, `ne555`). A caller that widens `implementedKinds` further still
 * (this file's own tests, ahead of the orchestrator's table) can exercise a
 * kind before it lands there.
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
 *
 * `fixedStepS` defaults to `STIMULUS_FIXED_STEP_S` (the real corpus replay
 * step) but is overridable so run-entry.ts's finding-11 Richardson-
 * extrapolation reference can replay the SAME phases a second time at half
 * that step, on a second independent `HeadlessRunner`, without duplicating
 * this function.
 */
export function replayStimuliOnClassic(
  circuit: SimCircuit,
  phases: readonly StimulusPhase[],
  fixedStepS: number = STIMULUS_FIXED_STEP_S,
): StimulusStepSample[] {
  let working = structuredClone(circuit);
  const runner = new HeadlessRunner();
  const samples: StimulusStepSample[] = [];

  phases.forEach((phase, phaseIndex) => {
    working = applyPhaseOverrides(working, phase);
    runner.load(working);

    const runOptions: HeadlessRunOptions = {
      durationS: phase.durationS,
      adaptive: false,
      fixedStepS,
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
 * One instant from CLASSIC's own already-recorded schedule — trimmed to just
 * the two fields `replayStimuliOnCore` needs to advance to. In practice a
 * caller passes `replayStimuliOnClassic`'s own `StimulusStepSample[]`
 * straight through (structurally compatible: the extra fields are simply
 * ignored), never a hand-built schedule.
 */
export interface StimulusScheduleSample {
  readonly phaseIndex: number;
  readonly simTime: number;
}

/**
 * Core replay of every phase, in order, on ONE `CoreEngine` (so `simTime` and
 * every device's committed state — capacitor charge, register contents,
 * pending delays — continue across phases exactly as `bo_engine_set_params`'s
 * "all other state is kept" promises). Loads `circuit` once, unmodified, then
 * for each phase applies `phase.set` through `CoreEngine.setParams` (never a
 * reload) before advancing.
 *
 * SAMPLE SCHEDULE: `schedule` is CLASSIC's OWN already-recorded sample
 * instants (`replayStimuliOnClassic`'s output, or anything shaped like
 * `StimulusScheduleSample[]`) — this function no longer derives a step
 * schedule of its own at all. For every scheduled instant that falls in a
 * phase, in order, it calls `engine.advance(simTime)` and records the sample
 * there; `advance()` itself already processes every event due at or before
 * that instant (core/README.md's C ABI v2 — "Process every event with
 * `t <= t_end`, then set time to `t_end`"), so the physics is correct
 * whether or not `simTime` happens to land on one of the core's own internal
 * due times.
 *
 * WHY NOT RE-DERIVE THE SCHEDULE (supersedes findings 10 and 13; W1d
 * orchestrator decision, 2026-09-29). An earlier version of this function
 * derived its own step schedule from `engine.nextDigitalDueTime()`, snapping
 * a step onto the core's own next pending driver commit to try to land on
 * the same instants classic's `nextScheduledEventTime()`-based clamp would.
 * Finding 10 picked the right, narrow ABI call for that (`nextDigitalDueTime`
 * over the wider `nextEventTime`); finding 13 reproduced classic's own H_MIN
 * overshoot when two due times fall within H_MIN of each other; a further
 * `+1e-12` nudge compensated for classic's reload seed solve. That approach
 * only ever worked as long as classic and the core agreed on WHICH instants
 * are "due" — decision 3 (datasheet delays) breaks the assumption for every
 * kind whose classic model predates that retrofit (74LS161/173/74HC74/
 * 74LS189/28C16/28C256: classic stamps their outputs immediately, writing no
 * `icState` `due:` entry at all, so `nextScheduledEventTime()` never reports
 * one for them, while the core's own retrofit gives every one of them a real
 * due time — W1d worker D4b's fixtures 25/27-29 reproduced this as an outright
 * sample-count mismatch). Re-deriving the core's schedule from the core's OWN
 * idea of "due" can never mirror a classic that has no matching concept for
 * a given kind; that is not a bug to fix by widening the clamp further, it is
 * the wrong strategy — and the same mismatch would resurface for any future
 * kind whose classic and core timing models diverge (W1e's 555 exact-crossing
 * split is the next one already flagged). Taking classic's own recorded
 * instants as the schedule instead sidesteps the mirroring problem entirely:
 * the core is simply told when classic sampled, for every kind, unconditionally.
 *
 * `implementedKinds` defaults to `CORE_IMPLEMENTED_KINDS` (kinds.ts's
 * orchestrator-owned table) — the same "a caller (chiefly tests) can widen
 * this" escape hatch `compileCircuit`/`CoreEngine.load` already document, for
 * a widened set. This is what lets this file's own tests exercise a kind
 * ahead of that table, without weakening the DEFAULT behaviour a real corpus
 * entry gets.
 *
 * Throws `StimuliNotYetSupportedOnCoreError` when the core cannot load the
 * circuit at all (see that class's doc comment). Throws a plain `Error`
 * (never that class) when a phase does not complete cleanly (non-convergence
 * — mirrors `replayStimuliOnClassic`'s own "did not complete cleanly" check),
 * when a phase's slice of `schedule` is empty or its last instant is not
 * within the usual end-of-phase tolerance of `phaseStart + durationS` (the
 * schedule passed in is truncated or was not actually built from this same
 * `phases` array — never silently compared as if the phase ran to
 * completion), or when `CoreEngine.setParams` itself rejects a change (an
 * unknown component/param, or a param that exists but is not live) — a
 * stimulus the caller asked for must fail loudly, never be silently skipped.
 */
export function replayStimuliOnCore(
  circuit: SimCircuit,
  phases: readonly StimulusPhase[],
  schedule: readonly StimulusScheduleSample[],
  implementedKinds: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS,
): StimulusStepSample[] {
  const engine = new CoreEngine();
  try {
    try {
      // INVESTIGATION point 2 (finding 12's write-up): fold phase 0's `set`
      // into the circuit BEFORE the initial load, exactly as
      // `replayStimuliOnClassic` folds every phase's `set` into a reload
      // (`applyPhaseOverrides` then `runner.load(working)`, above) — never
      // through `setParams` at t=0. Applying phase 0 via `setParams` instead
      // schedules a live event AT t=0 (e.g. an IC's pending delay-FSM
      // transition), which a genuine t=0 LOAD's own zero-delay settle pass
      // (decision 8) would have absorbed into the seed instead. Classic's own
      // phase-0 "reload" never has this problem — the override is already
      // baked into the netlist its load() settles — so replaying it through
      // setParams manufactures an extra sample classic can never produce
      // (the investigation's own reproduction: 11 samples on classic, 12 on
      // the core, for a fixture whose phase 0 carries a `set`).
      engine.load(applyPhaseOverrides(circuit, phases[0] ?? { durationS: 0 }), implementedKinds);
    } catch (err) {
      if (err instanceof CoreUnsupportedError) {
        throw new StimuliNotYetSupportedOnCoreError(err.reasons, { cause: err });
      }
      throw err;
    }

    const samples: StimulusStepSample[] = [];

    phases.forEach((phase, phaseIndex) => {
      // Phase 0's `set` (if any) is already folded into the circuit this
      // engine was loaded with, above — applying it again here through
      // setParams would double-apply it as a live event on top of an
      // already-settled initial state. Every LATER phase still uses
      // setParams, exactly as before: only phase 0 gets the "fold into the
      // load" treatment, matching classic's own reload only ever needing one
      // "cold start" instant.
      if (phaseIndex > 0 && phase.set && phase.set.length > 0) {
        const changes: CoreParamChange[] = phase.set.map((override) => ({
          componentId: override.id,
          params: override.params,
        }));
        engine.setParams(changes);
      }

      const phaseStartTime = engine.simTime;
      const phaseEndTime = phaseStartTime + phase.durationS;
      // Same slack headless.ts's run() uses, sized off THIS phase's own
      // durationS — see this function's doc comment.
      const tolerance = Math.max(1e-15, phase.durationS * 1e-12);

      // Advance to exactly classic's own recorded instants for this phase,
      // in order — see this function's own doc comment ("SAMPLE SCHEDULE")
      // for why this replaces the old due-time-derived clamp entirely.
      const scheduleForPhase = schedule.filter((s) => s.phaseIndex === phaseIndex);
      for (const scheduled of scheduleForPhase) {
        engine.advance(scheduled.simTime);
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

      // Integrity check on the SCHEDULE itself (never on the core's own
      // behaviour, which by construction just followed it): classic's last
      // recorded instant for this phase must be where this phase's own
      // durationS says it should be, or `schedule` was not actually built
      // from replaying THIS `phases` array — a silent truncation here would
      // make a later phase's setParams apply at the wrong "current time".
      const lastScheduled = scheduleForPhase[scheduleForPhase.length - 1];
      if (!lastScheduled || Math.abs(lastScheduled.simTime - phaseEndTime) > tolerance) {
        throw new Error(
          `replayStimuliOnCore: phase ${phaseIndex}'s classic schedule ` +
            (lastScheduled
              ? `ends at t=${lastScheduled.simTime}s, not within tolerance ${tolerance}s of phaseStart+durationS=${phaseEndTime}s`
              : "has no scheduled instants at all") +
            " — the schedule passed in is truncated, or was not built by replaying this same `phases` array.",
        );
      }
    });

    return samples;
  } finally {
    engine.dispose();
  }
}
