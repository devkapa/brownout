/**
 * HeadlessRunner fixed-grid sampling on the core (W2 design 2.7, D15 (a)).
 *
 * Learn samples a circuit on a fixed `dt` grid (`run({ adaptive: false,
 * fixedStepS })` with an `onSample` per step). On the core that means one
 * `advance` to each grid point, with a sample after each, instead of the W0/W1
 * single jump with one sample at the end. The grid is never coarser than
 * classic's step: classic takes `min(fixedStepS, sourceStepLimit)` away from
 * events, so the core grid spacing is that same minimum.
 *
 * The classic path is untouched; the tests here pin it as stepping `SimEngine`
 * by hand, so a later edit to the shared run() cannot move it unnoticed.
 */

import { beforeAll, describe, expect, it } from "vitest";
import { HeadlessRunner, type HeadlessSample } from "../../src/host/headless.js";
import { CoreEngine } from "../../src/core/engine.js";
import { initCore } from "../../src/core/wasm.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { SimEngine, type SimCircuit } from "../../src/sim/engine/sim-engine.js";

type SimComponent = SimCircuit["components"][number];
const part = (id: string, kind: string, pins: string[], params: Record<string, number> = {}): SimComponent => ({
  id,
  kind,
  pins: pins.map((pin) => ({ id: pin })),
  params,
});
const wire = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

/** 5 V through 1k into 1 uF: tau = 1 ms. */
const RC: SimCircuit = {
  components: [
    part("v", "voltage_source", ["pos", "neg"], { voltage: 5 }),
    part("r", "resistor", ["a", "b"], { resistance: 1000 }),
    part("c", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
  ],
  wires: [wire("v", "pos", "r", "a"), wire("r", "b", "c", "a"), wire("c", "b", "v", "neg")],
};

/** The same RC behind a 1 kHz clock, so the source step ceiling is 1e-4. */
const CLOCKED_RC: SimCircuit = {
  components: [
    part("v", "clock", ["out", "gnd"], { frequency: 1000 }),
    part("r", "resistor", ["a", "b"], { resistance: 1000 }),
    part("c", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
  ],
  wires: [wire("v", "out", "r", "a"), wire("r", "b", "c", "a"), wire("c", "b", "v", "gnd")],
};

function coreRunner(circuit: SimCircuit): HeadlessRunner {
  const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
  runner.load(circuit);
  expect(runner.activeEngine).toBe("core");
  return runner;
}

beforeAll(async () => {
  await initCore();
});

describe("core fixed-grid sampling", () => {
  it("adaptive:false advances to each grid point and samples after each", () => {
    const runner = coreRunner(RC);
    const samples: HeadlessSample[] = [];
    const result = runner.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-4, onSample: (s) => samples.push(s) });

    expect(samples).toHaveLength(10);
    samples.forEach((s, i) => {
      expect(s.simTime).toBeCloseTo((i + 1) * 1e-4, 15);
      expect(s.h).toBeCloseTo(1e-4, 15);
    });
    expect(result.acceptedSteps).toBe(10);
    expect(result.simulatedS).toBe(1e-3);
    expect(result.simTime).toBeCloseTo(1e-3, 15);
    expect(result.finalH).toBe(1e-4);
    expect(result.hitMinStep).toBe(false);
    expect(result.hitStepCap).toBe(false);
    expect(result.errorControlled).toBe(false);
  });

  it("each sample is the solved value at its grid point (the analytic RC charge), not a copy of the last one", () => {
    const runner = coreRunner(RC);
    const samples: HeadlessSample[] = [];
    runner.run({ durationS: 2e-3, adaptive: false, fixedStepS: 2e-4, onSample: (s) => samples.push(s) });
    const netOfCap = runner.netIdFor("c", "a")!;
    expect(samples).toHaveLength(10);
    samples.forEach((s) => {
      const expected = 5 * (1 - Math.exp(-s.simTime / 1e-3));
      expect(Math.abs(s.netV[netOfCap]! - expected), `t=${s.simTime}`).toBeLessThan(1e-6 + 1e-6 * Math.abs(expected));
    });
    // Per-sample copies: an earlier sample is not mutated by a later advance.
    expect(samples[0]!.netV[netOfCap]).not.toBe(samples[9]!.netV[netOfCap]);
  });

  it("the trajectory on the grid agrees with a single advance to the same time (D6: the clamps do not move the answer)", () => {
    const runner = coreRunner(CLOCKED_RC);
    const samples: HeadlessSample[] = [];
    runner.run({ durationS: 3e-3, adaptive: false, fixedStepS: 5e-5, onSample: (s) => samples.push(s) });
    const net = runner.netIdFor("c", "a")!;

    const reference = new CoreEngine();
    reference.load(CLOCKED_RC);
    for (const s of samples.filter((_, i) => i % 7 === 0)) {
      const fresh = new CoreEngine();
      fresh.load(CLOCKED_RC);
      fresh.advance(s.simTime);
      const want = fresh.getNetV()[net]!;
      expect(Math.abs(s.netV[net]! - want), `t=${s.simTime}`).toBeLessThanOrEqual(1e-4 + 1e-3 * Math.abs(want));
    }
  });

  it("is never coarser than classic: a 1 ms request on a 1 kHz clock samples at the 1e-4 source ceiling", () => {
    const runner = coreRunner(CLOCKED_RC);
    const samples: HeadlessSample[] = [];
    const result = runner.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-3, onSample: (s) => samples.push(s) });
    expect(samples).toHaveLength(10);
    expect(samples.every((s) => Math.abs(s.h - 1e-4) < 1e-15)).toBe(true);
    expect(result.finalH).toBe(1e-4);
  });

  it("clamps a too-fine request to classic's H_MIN floor the way classic does (and caps the run at maxSteps)", () => {
    const runner = coreRunner(RC);
    const samples: HeadlessSample[] = [];
    const result = runner.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-12, maxSteps: 7, onSample: (s) => samples.push(s) });
    expect(samples).toHaveLength(7);
    expect(samples[0]!.h).toBe(1e-8);
    expect(result.hitStepCap).toBe(true);
    expect(result.acceptedSteps).toBe(7);
    expect(result.simulatedS).toBeCloseTo(7e-8, 15);
  });

  it("a duration that is not a multiple of the step ends on the duration with a shorter last sample", () => {
    const runner = coreRunner(RC);
    const samples: HeadlessSample[] = [];
    const result = runner.run({ durationS: 2.5e-4, adaptive: false, fixedStepS: 1e-4, onSample: (s) => samples.push(s) });
    expect(samples.map((s) => s.h)).toHaveLength(3);
    expect(samples[2]!.h).toBeCloseTo(0.5e-4, 15);
    expect(samples[2]!.simTime).toBeCloseTo(2.5e-4, 15);
    expect(result.simTime).toBeCloseTo(2.5e-4, 15);
    expect(result.simulatedS).toBe(2.5e-4);
  });

  it("successive runs continue from the current time, each with its own grid", () => {
    const runner = coreRunner(RC);
    runner.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-4 });
    const samples: HeadlessSample[] = [];
    runner.run({ durationS: 3e-4, adaptive: false, fixedStepS: 1e-4, onSample: (s) => samples.push(s) });
    expect(samples).toHaveLength(3);
    expect(samples[0]!.simTime).toBeCloseTo(1.1e-3, 14);
    expect(samples[2]!.simTime).toBeCloseTo(1.3e-3, 14);
  });

  it("without onSample the grid still runs (and lands on the same state as one advance)", () => {
    const runner = coreRunner(RC);
    runner.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-4 });
    const net = runner.netIdFor("c", "a")!;
    expect(runner.snapshot().netV[net]).toBeCloseTo(5 * (1 - Math.exp(-1)), 6);
  });

  it("the default (adaptive) core run is unchanged: one jump, one sample, h = durationS", () => {
    const runner = coreRunner(RC);
    const samples: HeadlessSample[] = [];
    const result = runner.run({ durationS: 1e-3, onSample: (s) => samples.push(s) });
    expect(samples).toHaveLength(1);
    expect(samples[0]!.h).toBe(1e-3);
    expect(result.acceptedSteps).toBe(0);
  });

  it("still rejects adaptive:false with no fixedStepS", () => {
    const runner = coreRunner(RC);
    expect(() => runner.run({ durationS: 1e-3, adaptive: false })).toThrow(/fixedStepS/);
  });
});

describe("classic fixed-step output is unchanged", () => {
  it("adaptive:false on classic is the plain step(h) loop, sample for sample, bit for bit", () => {
    const runner = new HeadlessRunner();
    runner.load(RC);
    const samples: HeadlessSample[] = [];
    runner.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-4, onSample: (s) => samples.push(s) });

    const by = new SimEngine();
    by.load(RC);
    expect(samples).toHaveLength(10);
    for (const s of samples) {
      // The last step is `remaining`, which float subtraction leaves an ulp
      // short of 1e-4; replaying each sample's own h is the exact claim.
      expect(Math.abs(s.h - 1e-4)).toBeLessThan(1e-15);
      by.step(s.h);
      expect(s.simTime).toBe(by.simTime);
      expect(s.netV).toEqual(by.getNetV());
      expect(s.elementI).toEqual(by.getElementI());
    }
  });

  it("a clocked circuit on classic still lands steps on source edges (the grid is the core's, not classic's)", () => {
    const runner = new HeadlessRunner();
    runner.load(CLOCKED_RC);
    const hs: number[] = [];
    runner.run({ durationS: 1e-3, adaptive: false, fixedStepS: 1e-4, onSample: (s) => hs.push(s.h) });
    expect(hs.length).toBeGreaterThan(10);
    expect(hs.some((h) => h <= 1e-8 * 1.01)).toBe(true);
  });
});
