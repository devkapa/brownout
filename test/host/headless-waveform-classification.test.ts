/**
 * HeadlessRunner reads a signal_gen's waveform the way the engine plays it.
 *
 * The engine parses `waveform` (parseSignalGenParams) and plays anything it
 * does not recognise as a sine: "Sine", "SQUARE", a typo, a number. The step
 * ceiling tested the raw string instead, so those sines got none: "Sine" at
 * 1 kHz ran 22 steps over 20 ms with h up to 8.2 ms, and read above
 * mid-scale for 1.1 ms against 10 ms.
 */
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../src/host/headless.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { setLinearSystemBackendForTests } from "../../src/sim/engine/linear-system.js";

type SimComponent = SimCircuit["components"][number];
type Wire = SimCircuit["wires"][number];
const part = (id: string, kind: string, pins: string[], params: Record<string, number | string> = {}): SimComponent => ({
  id,
  kind,
  pins: pins.map((pin) => ({ id: pin })),
  params,
});
const wire = (a: string, ap: string, b: string, bp: string): Wire => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

interface Trace {
  accepted: number;
  rejected: number;
  /** [end time, h, load volts] per accepted step. */
  samples: Array<[number, number, number]>;
}

/** A 20 ms run of a 2.5 V +- 2.5 V, 1 kHz generator into 1 kohm. */
function trace(params: Record<string, number | string>): Trace {
  const runner = new HeadlessRunner();
  runner.load({
    components: [
      part("gen", "signal_gen", ["pos", "neg"], { frequency: 1000, amplitude: 2.5, offset: 2.5, rSource: 0, ...params }),
      part("load", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    wires: [wire("gen", "pos", "load", "a"), wire("load", "b", "gen", "neg")],
  });
  const samples: Trace["samples"] = [];
  const result = runner.run({
    durationS: 0.02,
    onSample: (sample) => samples.push([sample.simTime, sample.h, (sample.elementI.load ?? 0) * 1000]),
  });
  return { accepted: result.acceptedSteps, rejected: result.rejectedSteps, samples };
}

describe("HeadlessRunner with a waveform the engine does not recognise", () => {
  it("gives it a sine's 40 steps a period", () => {
    const run = trace({ waveform: "Sine" });
    expect(Math.max(...run.samples.map(([, h]) => h))).toBeLessThanOrEqual(1 / (1000 * 40));
    // Above mid-scale for half of every period: 10 ms of the 20.
    const high = run.samples.reduce((sum, [, h, v]) => (v > 2.5 ? sum + h : sum), 0);
    expect(high).toBeCloseTo(0.01, 4);
  });

  // A circuit loaded from JSON can carry any value here, whatever the params
  // type admits. The array and the boxed string stringify to "square", so they
  // tell a String()-based check apart from the engine's typeof test, which
  // plays them as a sine.
  const unrecognised: Array<[string, unknown]> = [
    ['"Sine"', "Sine"],
    ['"SQUARE"', "SQUARE"],
    ['"pulse " (trailing space)', "pulse "],
    ["7", 7],
    ["null", null],
    ["{}", {}],
    ['["square"]', ["square"]],
    ['new String("square")', new String("square")],
  ];
  for (const [label, waveform] of unrecognised) {
    it(`steps waveform ${label} exactly as the sine the engine plays`, () => {
      const sine = trace({ waveform: "sine" });
      const run = trace({ waveform: waveform as string });
      expect(run.accepted).toBe(sine.accepted);
      expect(run.rejected).toBe(sine.rejected);
      expect(run.samples).toEqual(sine.samples);
    });
  }
});

interface Fingerprint {
  accepted: number;
  rejected: number;
  failed: number;
  /** Sum of squared accepted steps: moves if any step boundary moves. */
  sumH2: number;
  /** Sum of v * h at the capacitor: moves if the trajectory does. */
  integral: number;
  final: number;
  maxH: number;
}

/**
 * A 1 kHz generator charging 100 nF through 1 kohm, delayed past the 1 ms
 * run so it holds its starting level: the waveform reaches the stepping only
 * through its step ceiling, which is what the classification decides.
 */
function fingerprint(params: Record<string, number | string>): Fingerprint {
  const runner = new HeadlessRunner();
  runner.load({
    components: [
      part("gen", "signal_gen", ["pos", "neg"], { frequency: 1000, delay: 2e-3, ...params }),
      part("r", "resistor", ["a", "b"], { resistance: 1000 }),
      part("c", "capacitor", ["a", "b"], { capacitance: 1e-7 }),
    ],
    wires: [wire("gen", "pos", "r", "a"), wire("r", "b", "c", "a"), wire("c", "b", "gen", "neg")],
  });
  const net = runner.netIdFor("c", "a")!;
  let sumH2 = 0;
  let integral = 0;
  let final = 0;
  let maxH = 0;
  const result = runner.run({
    durationS: 1e-3,
    onSample: (sample) => {
      sumH2 += sample.h * sample.h;
      integral += (sample.netV[net] ?? 0) * sample.h;
      final = sample.netV[net] ?? 0;
      maxH = Math.max(maxH, sample.h);
    },
  });
  return { accepted: result.acceptedSteps, rejected: result.rejectedSteps, failed: result.failedSteps, sumH2, integral, final, maxH };
}

/** Captured on the dense backend at brownout 0.6.1, when the raw string was tested. */
const RECOGNISED: Record<string, Fingerprint> = {
  dc: { accepted: 57, rejected: 0, failed: 0, sumH2: 8.250617940976358e-8, integral: 0.0022432872494047647, final: 2.499316883460329, maxH: 0.00021010818494216206 },
  sine: { accepted: 76, rejected: 0, failed: 0, sumH2: 2.0873090110472178e-8, integral: 0.0022425734660040685, final: 2.499715973519615, maxH: 0.000025 },
  square: { accepted: 60, rejected: 0, failed: 0, sumH2: 5.6717890633370246e-8, integral: 0.004486215467041074, final: 4.998971661070022, maxH: 0.0001 },
  pulse: { accepted: 24, rejected: 0, failed: 0, sumH2: 9.157040496249999e-8, integral: 0, final: 0, maxH: 0.0001 },
  triangle: { accepted: 52, rejected: 0, failed: 0, sumH2: 2.4473135762500022e-8, integral: 0, final: 0, maxH: 0.000025 },
  ramp: { accepted: 52, rejected: 0, failed: 0, sumH2: 2.4473135762500022e-8, integral: 0, final: 0, maxH: 0.000025 },
  pwl: { accepted: 57, rejected: 0, failed: 0, sumH2: 8.250617940976358e-8, integral: 0.0022432872494047647, final: 2.499316883460329, maxH: 0.00021010818494216206 },
  noise: { accepted: 61, rejected: 0, failed: 0, sumH2: 3.601308421273504e-8, integral: 0.0011203344644646696, final: 1.248418379615942, maxH: 0.00005 },
};

/** The forced-sparse lane eliminates in another order, so its floats match the dense capture to roundoff only. */
const FORCED_SPARSE = process.env.SIMCORE_LINEAR_BACKEND === "sparse";

function expectAsBefore(now: Fingerprint, was: Fingerprint): void {
  if (!FORCED_SPARSE) {
    expect(now).toEqual(was);
    return;
  }
  expect([now.accepted, now.rejected, now.failed]).toEqual([was.accepted, was.rejected, was.failed]);
  for (const key of ["sumH2", "integral", "final", "maxH"] as const) {
    expect(Math.abs(now[key] - was[key]), key).toBeLessThanOrEqual(1e-9 + 1e-6 * Math.abs(was[key]));
  }
}

describe("HeadlessRunner with a waveform the engine recognises", () => {
  beforeAll(() => {
    if (!FORCED_SPARSE) setLinearSystemBackendForTests("dense");
  });
  afterAll(() => {
    setLinearSystemBackendForTests(null);
  });

  for (const [waveform, was] of Object.entries(RECOGNISED)) {
    it(`steps "${waveform}" exactly as before`, () => {
      expectAsBefore(fingerprint({ waveform }), was);
    });
  }

  it("steps a generator with no waveform param as the sine it plays, exactly as before", () => {
    expectAsBefore(fingerprint({}), RECOGNISED.sine!);
  });
});
