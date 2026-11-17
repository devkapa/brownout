/**
 * Drives one circuit through one engine across a given sample schedule
 * (core/README.md-adjacent contract, spelled out in the worker brief's
 * "Harness semantics"). `sampleTimesFor` below is W0's fixed 4-point grid
 * (t=0, D/4, D/2, D); gate 2b's dense analog schedule (schedule.ts's
 * `denseAnalogSchedule`, filtered through quiet.ts's guard window) is a drop-
 * in replacement for it — both are just `number[]`, and `collectClassicSamples`/
 * `collectCoreSamples` neither know nor care which produced the array they
 * were given. Classic reaches each later time via successive
 * `run({ durationS })` deltas; the core jumps straight there via
 * `advance(t)` (its `advance` is absolute, per core/README.md's C ABI). Both
 * collectors return the exact same shape (SampleReadout[]) so compare.ts
 * never has to know which engine produced which side.
 */

import { HeadlessRunner } from "../../../src/host/headless.js";
import { CoreEngine } from "../../../src/core/engine.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import type { SampleReadout } from "./types.js";

/** Used when a CorpusEntry's own durationS is null (every local fixture — brownout's own and this harness's synthetic ones — leaves it null; W0 has no time-varying behavior for them to exercise anyway). */
export const DEFAULT_DURATION_S = 10e-3;

export function sampleTimesFor(durationS: number): number[] {
  return [0, durationS / 4, durationS / 2, durationS];
}

/**
 * Guards both collectors below against a caller-assembled schedule (gate
 * 2b's dense-and-quiet grid is no longer the hardcoded 4-point array W0
 * always produced) that is empty or not strictly ascending — either would
 * make `collectClassicSamples`'s successive `run({ durationS: t - prev })`
 * deltas silently misbehave (a zero or negative delta, or comparing samples
 * that were never actually taken in time order) instead of failing loudly at
 * the one place that can see the whole schedule at once.
 */
function assertAscendingTimes(times: readonly number[]): void {
  if (times.length === 0) {
    throw new Error("collectSamples: the sample schedule is empty — there is nothing to sample.");
  }
  for (let i = 1; i < times.length; i++) {
    if (!(times[i]! > times[i - 1]!)) {
      throw new Error(
        `collectSamples: the sample schedule must be strictly ascending, got ...${times[i - 1]}, ${times[i]}... ` +
          `at index ${i}.`,
      );
    }
  }
}

/**
 * classic: `new HeadlessRunner()` (default adaptive stepping), then
 * `load(circuit)`. load() itself performs classic's seed solve, which is
 * exactly the t=0 sample the brief asks for — no run() call needed to reach
 * it.
 *
 * A run that stops short (HeadlessRunResult.hitMinStep / hitStepCap) or a
 * snapshot that reports `converged: false` is refused with a harness error
 * rather than folded into the sample sequence: a `run({ durationS })` that
 * hits the H_MIN floor or the step cap never reaches the requested time, so
 * the sample it produces would be labelled `t` but actually sit at whatever
 * instant classic gave up at — silently comparable-looking, but not
 * actually sampled at the same instant the core's `advance(t)` reaches.
 * Throwing here turns that into run-entry.ts's "error" status (with a clear
 * message) instead of a false pass. See compare.ts's simTime check for the
 * belt-and-suspenders half of this: even a divergence this function fails
 * to catch still shows up there.
 */
export function collectClassicSamples(circuit: SimCircuit, times: readonly number[]): SampleReadout[] {
  assertAscendingTimes(times);
  const runner = new HeadlessRunner();
  runner.load(circuit);

  const samples: SampleReadout[] = [];
  let prev = 0;
  for (const t of times) {
    if (t > 0) {
      const result = runner.run({ durationS: t - prev });
      if (result.hitMinStep || result.hitStepCap) {
        const why = result.hitMinStep ? "hit the H_MIN floor" : "hit its step cap";
        throw new Error(
          `collectClassicSamples: classic run toward t=${t}s ${why}, reaching only t=${result.simTime}s — ` +
            "a sample taken here would be compared against the core at a different instant than it was " +
            "actually collected at.",
        );
      }
      prev = t;
    }
    const snap = runner.snapshot();
    if (!snap.converged) {
      throw new Error(
        `collectClassicSamples: classic snapshot at t=${snap.simTime}s did not converge — its state is not a ` +
          "trusted point of comparison against the core.",
      );
    }
    samples.push({
      simTime: snap.simTime,
      netV: snap.netV,
      elementI: snap.elementI,
      digitalState: snap.digitalState,
      failures: snap.failures,
      warnings: snap.warnings,
    });
  }
  return samples;
}

/**
 * core: `new CoreEngine()` then `load(circuit)` with the DEFAULT implemented-
 * kinds set (CORE_IMPLEMENTED_KINDS) — this harness's whole job is proving
 * out kinds that have not yet cleared CORE_SUPPORTED_KINDS, so it must be
 * free to load anything the compiler understands, not just what a normal
 * host is allowed to route here. Disposes its engine handle in `finally`:
 * this function's caller (run-entry.ts) constructs a fresh CoreEngine per
 * corpus entry rather than reusing one the way HeadlessRunner's own
 * `engine: "core"` option does, so nothing else will dispose it.
 */
export function collectCoreSamples(circuit: SimCircuit, times: readonly number[]): SampleReadout[] {
  assertAscendingTimes(times);
  const engine = new CoreEngine();
  try {
    engine.load(circuit);
    const samples: SampleReadout[] = [];
    for (const t of times) {
      if (t > 0) engine.advance(t);
      samples.push({
        simTime: engine.simTime,
        netV: { ...engine.getNetV() },
        elementI: { ...engine.getElementI() },
        digitalState: { ...engine.digitalState },
        failures: engine.getFailures(),
        warnings: engine.getWarnings(),
      });
    }
    return samples;
  } finally {
    engine.dispose();
  }
}
