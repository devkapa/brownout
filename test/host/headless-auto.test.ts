/**
 * HeadlessRunner's `engine: "auto"` (ruling R116): the core where every kind
 * of the circuit is supported, classic with the reasons otherwise, and a core
 * failure in the middle of a run replayed on classic.
 *
 * The replay is "from load": no state is carried between the two engines
 * (W2 design D9b (a)), so the runner loads on classic every circuit it gave
 * the core since classic last ran, and runs again every run() the core
 * completed. What is pinned here is that the result is the one a classic
 * runner gives for the same calls, bit for bit, that each fallback says why,
 * and that an explicit "core" request still reports the failure itself.
 *
 * The failures are injected by patching CoreEngine (a real one needs a
 * circuit the core gives up on, or a wasm trap).
 */
import { afterEach, beforeAll, describe, expect, it, vi } from "vitest";
import { HeadlessRunner, type HeadlessSample } from "../../src/host/headless.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CoreTrapError } from "../../src/core/errors.js";
import { initCore } from "../../src/core/wasm.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type SimComponent = SimCircuit["components"][number];
const part = (id: string, kind: string, pins: string[], params: Record<string, number> = {}): SimComponent => ({
  id,
  kind,
  pins: pins.map((pin) => ({ id: pin })),
  params,
});
const wire = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

/** 9 V into R and 1 uF: a state that moves, so a replay that lost it would show. */
function rc(resistance = 10_000): SimCircuit {
  return {
    components: [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 9 }),
      part("r1", "resistor", ["a", "b"], { resistance }),
      part("c1", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
    ],
    wires: [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "c1", "a"), wire("c1", "b", "vsrc", "neg")],
  };
}
function rcWithDiode(): SimCircuit {
  const base = rc();
  return {
    components: [...base.components, part("d1", "diode", ["a", "k"])],
    wires: [...base.wires, wire("r1", "b", "d1", "a"), wire("d1", "k", "vsrc", "neg")],
  };
}

const AUTO = { engine: "auto", coreSupportedKinds: CORE_IMPLEMENTED_KINDS } as const;

/** Reports the next `advance` whose convergence the runner reads as failed (-4/-7 without a real one). */
function failNextAdvance(): void {
  let armed = true;
  const original = Object.getOwnPropertyDescriptor(CoreEngine.prototype, "lastConverged")!.get!;
  vi.spyOn(CoreEngine.prototype, "lastConverged", "get").mockImplementation(function (this: CoreEngine) {
    if (armed) {
      armed = false;
      return false;
    }
    return original.call(this);
  });
}

/** Throws `error` from the n-th `advance` from now (1-based); the others are real. */
function throwOnAdvance(n: number, error: Error): void {
  const original = CoreEngine.prototype.advance;
  let calls = 0;
  vi.spyOn(CoreEngine.prototype, "advance").mockImplementation(function (this: CoreEngine, tEnd: number) {
    calls += 1;
    if (calls === n) throw error;
    return original.call(this, tEnd);
  });
}

const capV = (runner: HeadlessRunner): number => runner.snapshot().netV[runner.netIdFor("c1", "a")!]!;

beforeAll(async () => {
  await initCore();
});
afterEach(() => {
  vi.restoreAllMocks();
});

describe('engine: "auto" — the choice at load', () => {
  it("runs a circuit on the core when every kind is supported", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(rc());
    expect(runner.activeEngine).toBe("core");
    expect(runner.engineFallbackReasons).toEqual([]);
  });

  it("runs it on classic, with the reason, when one kind is not", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(rcWithDiode());
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual(["unsupported-kind:diode"]);
    expect(runner.run({ durationS: 1e-3 }).hitMinStep).toBe(false);
  });

  it("is classic on the package's own kind list while that list is empty, each kind a reason", () => {
    const runner = new HeadlessRunner({ engine: "auto" });
    runner.load(rc());
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual([
      "unsupported-kind:capacitor",
      "unsupported-kind:resistor",
      "unsupported-kind:voltage_source",
    ]);
  });

  it("the default is still classic, with no fallback recorded", () => {
    const runner = new HeadlessRunner();
    runner.load(rc());
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual([]);
  });
});

describe('engine: "auto" — a core failure in the middle of a run replays on classic', () => {
  /** What a classic runner gives for the same calls. */
  function classicReference(steps: (runner: HeadlessRunner) => void): HeadlessRunner {
    const runner = new HeadlessRunner();
    steps(runner);
    return runner;
  }

  it("a failed advance: the run finishes on classic with the state classic has for the same calls, and says why", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(rc());
    expect(runner.run({ durationS: 1e-3 }).hitMinStep).toBe(false);
    expect(runner.activeEngine).toBe("core");

    failNextAdvance();
    const result = runner.run({ durationS: 1e-3 });

    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual(["mid-run:advance-failed", "restarted-from-load"]);
    expect(result.hitMinStep).toBe(false);
    expect(result.simulatedS).toBeCloseTo(1e-3, 12);

    const classic = classicReference((r) => {
      r.load(rc());
      r.run({ durationS: 1e-3 });
      r.run({ durationS: 1e-3 });
    });
    expect(runner.simTime).toBe(classic.simTime);
    expect(capV(runner)).toBe(capV(classic));
    // Not the fresh-state answer: a classic engine loaded at 1 ms with an empty capacitor reads far lower.
    expect(capV(runner)).toBeGreaterThan(9 * (1 - Math.exp(-0.2)) * 0.9);
  });

  it("the failure's own code and message are in the reason when the core gave one", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(rc());
    failNextAdvance();
    vi.spyOn(CoreEngine.prototype, "lastAdvanceFailure", "get").mockReturnValue({ code: -4, message: "stepped: Newton did not converge at the minimum step" });
    runner.run({ durationS: 1e-3 });
    expect(runner.engineFallbackReasons).toEqual([
      "mid-run:solve-failed:stepped: Newton did not converge at the minimum step",
      "restarted-from-load",
    ]);
  });

  it("a trap: the same replay, with the core's failure as the reason", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(rc());
    runner.run({ durationS: 1e-3 });
    throwOnAdvance(1, new CoreTrapError("wasm trap"));
    const result = runner.run({ durationS: 1e-3 });

    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual(["core-trap", "restarted-from-load"]);
    expect(result.hitMinStep).toBe(false);
    const classic = classicReference((r) => {
      r.load(rc());
      r.run({ durationS: 1e-3 });
      r.run({ durationS: 1e-3 });
    });
    expect(capV(runner)).toBe(capV(classic));
  });

  it("every circuit loaded since classic last ran is replayed, so a reload's carried charge is classic's", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(rc(10_000));
    runner.run({ durationS: 2e-3 });
    runner.load(rc(47_000)); // an edit: the core carries the capacitor's charge across it
    expect(runner.activeEngine).toBe("core");
    failNextAdvance();
    runner.run({ durationS: 1e-3 });

    const classic = classicReference((r) => {
      r.load(rc(10_000));
      r.run({ durationS: 2e-3 });
      r.load(rc(47_000));
      r.run({ durationS: 1e-3 });
    });
    expect(runner.activeEngine).toBe("classic");
    expect(runner.simTime).toBe(classic.simTime);
    expect(capV(runner)).toBe(capV(classic));
  });

  it("the circuit replayed is the one load() took, not the caller's object as edited since", () => {
    const circuit = rc(10_000);
    const runner = new HeadlessRunner(AUTO);
    runner.load(circuit);
    runner.run({ durationS: 1e-3 });
    circuit.components[1]!.params.resistance = 1; // edited in place, never loaded
    failNextAdvance();
    runner.run({ durationS: 1e-3 });
    const classic = classicReference((r) => {
      r.load(rc(10_000));
      r.run({ durationS: 1e-3 });
      r.run({ durationS: 1e-3 });
    });
    expect(capV(runner)).toBe(capV(classic));
  });

  it("the failed run's samples all come from classic: none from the core's part of it, none twice", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(rc());
    runner.run({ durationS: 1e-3 });
    // Fixed grid: the core samples each point; fail the third point of this run.
    let reads = 0;
    const original = Object.getOwnPropertyDescriptor(CoreEngine.prototype, "lastConverged")!.get!;
    vi.spyOn(CoreEngine.prototype, "lastConverged", "get").mockImplementation(function (this: CoreEngine) {
      reads += 1;
      return reads === 3 ? false : original.call(this);
    });
    const samples: HeadlessSample[] = [];
    runner.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-4, onSample: (s) => samples.push(s) });

    const expected: HeadlessSample[] = [];
    classicReference((r) => {
      r.load(rc());
      r.run({ durationS: 1e-3 });
      r.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-4, onSample: (s) => expected.push(s) });
    });
    expect(samples).toEqual(expected);
    for (let i = 1; i < samples.length; i++) expect(samples[i]!.simTime).toBeGreaterThan(samples[i - 1]!.simTime);
  });

  it("a run the core completes delivers its samples as before", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(rc());
    const samples: HeadlessSample[] = [];
    runner.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-4, onSample: (s) => samples.push(s) });
    expect(runner.activeEngine).toBe("core");
    expect(samples).toHaveLength(10);
    expect(samples[9]!.simTime).toBeCloseTo(1e-3, 15);
  });

  it("the next load() chooses again, and the core takes the circuit back", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(rc());
    failNextAdvance();
    runner.run({ durationS: 1e-3 });
    expect(runner.activeEngine).toBe("classic");
    runner.load(rc());
    expect(runner.activeEngine).toBe("core");
    expect(runner.engineFallbackReasons).toEqual([]);
    expect(runner.simTime).toBeCloseTo(1e-3, 12);
  });
});

describe('engine: "core" — an explicit request still reports the failure itself', () => {
  it("a failed advance is hitMinStep on the result, and the runner stays on the core", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(rc());
    failNextAdvance();
    const result = runner.run({ durationS: 1e-3 });
    expect(result.hitMinStep).toBe(true);
    expect(runner.activeEngine).toBe("core");
    expect(runner.engineFallbackReasons).toEqual([]);
  });

  it("a trap is thrown to the caller", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(rc());
    throwOnAdvance(1, new CoreTrapError("wasm trap"));
    expect(() => runner.run({ durationS: 1e-3 })).toThrow(CoreTrapError);
    expect(runner.activeEngine).toBe("core");
  });
});
