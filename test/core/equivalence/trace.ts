/**
 * Digital traces: an ordered (t, key, value) diff stream describing every
 * digitalState change either engine produces, the raw material align.ts
 * turns into per-cycle samples (decision 6, gate 2a).
 *
 * CLASSIC. `digitalState` is committed once per accepted step
 * (sim-engine.ts's `_updateDigitalState`, called from inside `SimEngine.step`)
 * and rebuilt from scratch every pass — a key that stops being published (an
 * IC losing power, a tri-stated bus) simply disappears from the object,
 * rather than the object keeping a stale value. `collectClassicDigitalTrace`
 * below drives a `HeadlessRunner` with `onSample` (fired once per ACCEPTED
 * step, never a rejected trial) and reads `runner.engine.digitalState`
 * inside that callback — the same instant the sample describes, per
 * `simulate.ts`'s own comment in devolt (read for this brief; devolt is a
 * private, off-limits repo for this worker, not imported) doing exactly this
 * for the same reason. The load-time state itself (before any accepted step)
 * is recorded too, as a diff from an EMPTY state — this is deliberate: it is
 * the only way decision 8's load-time phantom edge (`lastClk` seeded 0 while
 * a clock is already HIGH) becomes visible to align.ts's cycle detector
 * instead of being silently swallowed into an assumed-empty starting point.
 *
 * CORE. `CoreEngine` (src/core/engine.ts) now implements the trace ABI
 * (`enableTrace`/`readTrace`/`clearTrace`, over `bo_engine_trace_enable`/
 * `_ptr`/`_len`/`_clear`, contract v2 "W1a") — `readTrace()` is already
 * decoded to `{t, key, value}` via the compiler's own slot table
 * (core/README.md's "Readouts": "Keys are built in TS; Rust publishes
 * numbers"), so `collectCoreDigitalTrace` below does no decoding of its own.
 * `CoreTraceCapable` is the STRUCTURAL shape it needs;
 * `collectCoreDigitalTrace` still feature-detects it at runtime with
 * `hasCoreTraceSupport` rather than importing `CoreEngine`'s concrete type
 * for the check, so this file keeps working unchanged against a synthetic
 * stand-in (trace.test.ts's own coverage: "code against an interface and
 * test with synthetic traces") and degrades cleanly
 * (`CoreTraceUnsupportedError`) if this worktree's wasm build is ever older
 * than the TS side expects it to be.
 */

import { CoreEngine } from "../../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { HeadlessRunner, type HeadlessRunOptions } from "../../../src/host/headless.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";

/**
 * One digitalState change. `value` is `NaN` when the key stopped being
 * published — the same "absent" convention core/README.md's v2 ABI uses for
 * a digital slot ("NaN means the key is absent this pass"), chosen
 * deliberately so classic's and the core's trace events share one
 * vocabulary instead of classic needing `undefined`/omission and the core
 * needing a numeric sentinel.
 */
export interface DigitalTraceEvent {
  readonly t: number;
  readonly key: string;
  readonly value: number;
}

/**
 * Every key whose value differs between `before` and `after`, as trace
 * events at time `t`. A key present in one and absent in the other counts as
 * a change (to/from `NaN`, per this module's absent convention). Exported
 * because align.ts's synthetic-trace tests and this module's own tests both
 * need to build a trace from a sequence of hand-written digitalState
 * snapshots without going through a real engine.
 */
export function diffDigitalStates(
  t: number,
  before: Readonly<Record<string, number>>,
  after: Readonly<Record<string, number>>,
): DigitalTraceEvent[] {
  const events: DigitalTraceEvent[] = [];
  const keys = new Set([...Object.keys(before), ...Object.keys(after)]);
  for (const key of [...keys].sort()) {
    const a = Object.prototype.hasOwnProperty.call(before, key) ? before[key] : NaN;
    const b = Object.prototype.hasOwnProperty.call(after, key) ? after[key] : NaN;
    if (!Object.is(a, b)) events.push({ t, key, value: b! });
  }
  return events;
}

/**
 * Classic's digital trace over `[0, durationS]`: the load-time state (as a
 * diff from `{}`, at t=0) followed by one diff per accepted step. `options`
 * mirrors `HeadlessRunOptions`'s stepping controls (a "refined fixed step"
 * run, per this worker's required test, passes `fixedStepS`).
 *
 * Throws if the run does not cleanly cover the full `durationS` — a trace
 * that quietly stops short would have align.ts read a truncated cycle count
 * as "the circuit only ran this many cycles" instead of "the run failed",
 * exactly the false-pass sample.ts's own collectors already guard against.
 */
export function collectClassicDigitalTrace(
  circuit: SimCircuit,
  durationS: number,
  options?: { readonly fixedStepS?: number; readonly maxSteps?: number },
): DigitalTraceEvent[] {
  const runner = new HeadlessRunner();
  runner.load(circuit);

  const events: DigitalTraceEvent[] = [];
  let prev: Record<string, number> = { ...runner.engine.digitalState };
  events.push(...diffDigitalStates(runner.engine.simTime, {}, prev));

  const runOptions: HeadlessRunOptions = {
    durationS,
    onSample: () => {
      const next = { ...runner.engine.digitalState };
      events.push(...diffDigitalStates(runner.engine.simTime, prev, next));
      prev = next;
    },
  };
  if (options?.fixedStepS !== undefined) {
    runOptions.adaptive = false;
    runOptions.fixedStepS = options.fixedStepS;
  }
  if (options?.maxSteps !== undefined) runOptions.maxSteps = options.maxSteps;

  const result = runner.run(runOptions);
  if (result.hitMinStep || result.hitStepCap) {
    const why = result.hitMinStep ? "hit the H_MIN floor" : "hit its step cap";
    throw new Error(
      `collectClassicDigitalTrace: the run toward t=${durationS}s ${why}, reaching only t=${result.simTime}s — ` +
        "a digital trace built from this run would silently under-report the number of cycles the circuit " +
        "actually completed.",
    );
  }
  return events;
}

/**
 * The shape a `CoreEngine` gains once W1a's ABI v2 trace exports land
 * (`bo_engine_trace_enable`/`_ptr`/`_len`/`_clear`). `advance` is already on
 * every CoreEngine (W0); it is repeated here because `collectCoreDigitalTrace`
 * needs to drive it through this SAME structural handle, not a second
 * reference to the real class.
 */
export interface CoreTraceCapable {
  advance(tEnd: number): void;
  enableTrace(enabled: boolean): void;
  /** Already decoded to (t, key, value) via the compiler's slot table — see core/README.md's "Readouts". */
  readTrace(): readonly DigitalTraceEvent[];
  clearTrace(): void;
}

export function hasCoreTraceSupport(engine: unknown): engine is CoreTraceCapable {
  if (typeof engine !== "object" || engine === null) return false;
  const e = engine as Partial<Record<keyof CoreTraceCapable, unknown>>;
  return (
    typeof e.advance === "function" &&
    typeof e.enableTrace === "function" &&
    typeof e.readTrace === "function" &&
    typeof e.clearTrace === "function"
  );
}

/**
 * Thrown by a real (non-synthetic) core trace call if the live `CoreEngine`
 * ever does NOT satisfy `CoreTraceCapable` — normally unreachable now that
 * `CoreEngine.enableTrace`/`readTrace`/`clearTrace` exist, but kept as a
 * defensive downgrade (not a thrown circuit failure) for a worktree whose
 * built wasm is stale relative to its own TS, or any other build skew.
 * Callers that must tolerate that catch this specifically.
 */
export class CoreTraceUnsupportedError extends Error {
  constructor() {
    super(
      "collectCoreDigitalTrace: this CoreEngine build does not implement the trace ABI yet " +
        "(core/README.md's bo_engine_trace_enable/_ptr/_len/_clear, contract v2 \"W1a\" — landing " +
        "concurrently in this dispatch's sibling workers, A1-A3). Re-run once those land in this " +
        "worktree; until then, treat a digital-trace comparison as unavailable, not as a failure.",
    );
    this.name = "CoreTraceUnsupportedError";
  }
}

/**
 * The core's digital trace over `[0, durationS]` — mirrors
 * `collectClassicDigitalTrace`'s signature so run-entry.ts can call either
 * collector the same way. Builds and disposes its own `CoreEngine`, exactly
 * as sample.ts's `collectCoreSamples` does, so no caller holds a handle
 * across the call.
 */
export function collectCoreDigitalTrace(
  circuit: SimCircuit,
  durationS: number,
  implementedKinds: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS,
): DigitalTraceEvent[] {
  const engine = new CoreEngine();
  try {
    engine.load(circuit, implementedKinds);
    if (!hasCoreTraceSupport(engine)) {
      throw new CoreTraceUnsupportedError();
    }
    // The load-time baseline, as a diff from `{}` at the load's own instant —
    // mirroring `collectClassicDigitalTrace`'s identical `diffDigitalStates(t,
    // {}, prev)` baseline above. `bo_engine_trace_enable` only records CHANGES
    // from whenever it is turned on, so without this the core's trace would
    // start from whatever `load()` already settled to (silently), while
    // classic's trace explicitly carries that same settle as its own cycle-0
    // sample (align.ts's `sampleCycles`, decision 8's own "the load-time
    // state IS the first thing align.ts's cycle 0 reads"). Omitting this made
    // the two traces structurally asymmetric: cycle 0 came out empty on the
    // core (nothing was ever recorded before the first REAL transition) but
    // non-empty on classic, so no anchor state already true at load could
    // ever be found on the core side.
    const baseline = engine.digitalState;
    const baselineEvents: DigitalTraceEvent[] = Object.entries(baseline).map(([key, value]) => ({
      t: engine.simTime,
      key,
      value,
    }));

    engine.enableTrace(true);
    engine.advance(durationS);
    const events = [...engine.readTrace()];
    engine.clearTrace();
    return [...baselineEvents, ...events];
  } finally {
    engine.dispose();
  }
}

/**
 * Folds a diff stream back into a full snapshot as of the last event with
 * `t <= at` (or `{}` if the trace is empty / starts after `at`). Exported for
 * tests and for any caller that wants "the state at this instant" without
 * reimplementing the fold — align.ts's `sampleCycles` does this same fold
 * internally (incrementally, for efficiency over many cycles) but this
 * standalone version is the readable reference implementation the tests
 * check it against.
 */
export function stateAt(trace: readonly DigitalTraceEvent[], at: number): Record<string, number> {
  const state: Record<string, number> = {};
  for (const e of trace) {
    if (e.t > at) break;
    if (Number.isNaN(e.value)) delete state[e.key];
    else state[e.key] = e.value;
  }
  return state;
}
