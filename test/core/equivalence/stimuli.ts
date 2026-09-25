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

/**
 * Finding 13: classic's own fixed-step loop (`src/host/headless.ts`'s `run()`,
 * `if (untilEvent <= h) { h = Math.max(H_MIN, untilEvent); ... }`) floors
 * EVERY event-aligned step at `H_MIN = 1e-8`, even when landing exactly on a
 * due event — so two due times under `H_MIN` apart make classic OVERSHOOT the
 * second one rather than landing on it exactly. `replayStimuliOnCore`'s own
 * clamp (below) must reproduce that overshoot bit-for-bit, or the two engines
 * land on different instants whenever two ICs with different propagation
 * delays are driven by the same edge. Duplicated here (not imported) because
 * `src/host/headless.ts` is outside this worker's ownership (core/src +
 * test/core only) — this constant must be kept equal to headless.ts's own
 * `H_MIN`, which is a private, unexported module constant there.
 */
const STIMULI_CLAMP_H_MIN = 1e-8;

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
 * Core replay of every phase, in order, on ONE `CoreEngine` (so `simTime` and
 * every device's committed state — capacitor charge, register contents,
 * pending delays — continue across phases exactly as `bo_engine_set_params`'s
 * "all other state is kept" promises). Loads `circuit` once, unmodified, then
 * for each phase applies `phase.set` through `CoreEngine.setParams` (never a
 * reload) before advancing `phase.durationS`.
 *
 * STEP SCHEDULE (finding 10). This must take EXACTLY the same steps
 * `HeadlessRunner.run()` takes for `replayStimuliOnClassic` (headless.ts's
 * `run()`, the fixed-step branch): full `STIMULUS_FIXED_STEP_S` steps, then
 * one FINAL step clamped to whatever time is left in the phase, using the
 * identical end-of-phase slack (`Math.max(1e-15, phase.durationS * 1e-12)`,
 * headless.ts's own comment: "so a duration that is an exact multiple of the
 * final step does not spin one extra ~1e-16 s step"). The previous
 * `Math.round(phase.durationS / STIMULUS_FIXED_STEP_S)` whole-step count
 * rounded a non-multiple phase duration (e.g. 2.5ms) up or down to a whole
 * number of 1ms steps, silently drifting the core onto different simTime
 * instants than classic — or, for a phase shorter than one step (e.g.
 * 0.4ms), producing ZERO samples where classic produces one. Mirroring the
 * clamp instead means the two engines always land on the identical instant
 * sequence, whether or not `phase.durationS` is a whole multiple of
 * `STIMULUS_FIXED_STEP_S`.
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
      let t = phaseStartTime;
      while (phaseEndTime - t > tolerance) {
        // W1c EVENT-ALIGNED CLAMP, mirroring headless.ts's own classic-side
        // clamp (`nextScheduledEventTime()`, whose W1c addition also consults
        // `icState`'s DIGITAL_DUE_PREFIX due times — a combinational IC's
        // pending delay-FSM transition, sim-engine.ts). Discovered empirically
        // wiring the 74HC138/74LS283-DIP-switch fixture (test/fixtures/core/
        // 20-*): classic's run() ALWAYS snaps a step boundary onto a due
        // digital transition when one falls inside it, even in fixed-step
        // mode (that clamp is unconditional, not gated on adaptive/error
        // control) — landing an extra, short first step right after a phase's
        // reload whenever the reload leaves a pending delayed transition. This
        // loop had no equivalent, so it always landed on the plain
        // STIMULUS_FIXED_STEP_S grid regardless — same eventual PHYSICS
        // (bo_engine_advance processes every due event up to whatever t_end
        // it is given, whether or not t_end lands exactly on one), but a
        // DIFFERENT sample count/schedule, which compareStimuliSamples
        // rejects outright as a harness bug (it requires two equal-length,
        // time-aligned sequences).
        //
        // FINDING 10: clamping to `engine.nextEventTime()` (bo_engine_next_
        // event_time) is TOO WIDE — that ABI call reports the earliest of ANY
        // pending kernel event (a stepped partition's own internal Step
        // bookkeeping, a closed-form Crossing watch, a floating-input Bucket
        // boundary), none of which classic's `nextScheduledEventTime()` (the
        // function this clamp is modeled on) ever considers — it only ever
        // looks at a genuinely due digital commit. Clamping to the wider call
        // made a W1b stepped-partition stimuli entry (no combinational IC at
        // all) explode into millions of 1e-8-spaced samples the instant a
        // `set` dirtied its stepped partition. `engine.nextDigitalDueTime()`
        // (bo_engine_next_digital_due_time, core/README.md's C ABI v2, added
        // for exactly this) reports ONLY a live driver-commit token, the same
        // narrower thing classic's own function reports — `advance()` already
        // processes every intervening Step/Crossing/Bucket event on its own
        // to reach whatever `t_end` this loop picks, so there was never a
        // reason for this clamp to single-step through the solver's own
        // internal bookkeeping in the first place.
        const candidateStep = Math.min(STIMULUS_FIXED_STEP_S, phaseEndTime - t);
        const untilEvent = engine.nextDigitalDueTime() - t;
        // FINDING 13: classic's own clamp floors EVERY event-aligned step at
        // H_MIN (headless.ts: `h = Math.max(H_MIN, untilEvent)`), even when
        // landing exactly on a due event — so two due times under H_MIN apart
        // make classic OVERSHOOT the second one (it lands `H_MIN` past the
        // first, not exactly on the second) rather than snapping to it. This
        // clamp must reproduce that overshoot, or a fixture with two
        // different-family ICs (74hc: 15ns, 74ls: 20ns delay) commit-chained
        // off the same edge lands the two engines on different instants ~5ns
        // apart, in a way that's `<= 0` (no reproduction of classic's actual
        // OVERSHOOT of the true due instant), and both simTime and the
        // resulting stale netV/elementI mismatch.
        const clampedUntilEvent = Math.max(STIMULI_CLAMP_H_MIN, untilEvent);
        // The `+ 1e-12` nudge: classic applies each phase by RELOADING, and
        // load() runs its 1 ps seed solve (spec §2.2), so a delay-FSM
        // transition scheduled by that seed falls due at `phaseStart + 1e-12
        // + delay`. The core applies a LATER phase's overrides through
        // set_params at `phaseStart` exactly (phase 0's own override is now
        // folded into the initial load instead — see this function's load()
        // call, above — so it never reaches this nudge at all), so the same
        // transition falls due 1 ps earlier. Stepping the core to
        // `clampedUntilEvent + 1e-12` lands its sample on classic's instant,
        // just after the commit on both sides, instead of loosening
        // compare.ts's 1e-12 simTime check.
        t += untilEvent > 0 && untilEvent <= candidateStep ? clampedUntilEvent + 1e-12 : candidateStep;
        engine.advance(t);
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
