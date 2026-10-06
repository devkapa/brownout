/**
 * HeadlessRunner lands a step on every source edge, corner and PWL knot.
 *
 * The engine samples a timed source at the END of each step, so a step that
 * straddles an edge applies the new level across the whole step, and a
 * feature narrower than the step is never sampled at all. The step ceiling
 * cannot prevent that on its own: a PWL source has none, and a pulse's is
 * P/10 whatever its width. Before the runner took the browser worker's
 * source breakpoints, a 0.2 ms PWL pulse at 3 ms was never seen (22 steps
 * over 20 ms, h up to 8.2 ms), and a 20 us pulse every 1 ms, from a
 * signal_gen or a pulse_gen, was seen only in its first period, while h was
 * still growing from 10 ns: 11.2 us HIGH against 420 us.
 */
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { HeadlessRunner, type HeadlessRunOptions } from "../../src/host/headless.js";
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
const pinsOf = (component: SimComponent): [string, string] => [component.pins[0]!.id, component.pins[1]!.id];

/** The runner's floor, and so the length of the guard step that crosses a source event. */
const H_MIN = 1e-8;

type RunOptions = Omit<HeadlessRunOptions, "durationS" | "onSample">;

/** A two-pin source into 1 kohm, so the load's current in mA is the source's voltage. */
function sourceIntoLoad(source: SimComponent): SimCircuit {
  const [pos, neg] = pinsOf(source);
  return {
    components: [source, part("load", "resistor", ["a", "b"], { resistance: 1000 })],
    wires: [wire(source.id, pos, "load", "a"), wire("load", "b", source.id, neg)],
  };
}

interface Step {
  start: number;
  end: number;
  /** The step as the runner reported it. */
  h: number;
  /** The load's voltage at the step's end, the instant the engine sampled the source. */
  v: number;
}

function steps(circuit: SimCircuit, durationS: number, options: RunOptions = {}): Step[] {
  const runner = new HeadlessRunner();
  runner.load(circuit);
  const out: Step[] = [];
  let start = 0;
  runner.run({
    durationS,
    ...options,
    onSample: (sample) => {
      out.push({ start, end: sample.simTime, h: sample.h, v: (sample.elementI.load ?? 0) * 1000 });
      start = sample.simTime;
    },
  });
  return out;
}

/** Indices of the 10 ns guard steps, skipping the runner's 10 ns first step. */
function guardSteps(run: readonly Step[]): number[] {
  return run.flatMap((step, k) => (step.start > 0 && step.h <= H_MIN * (1 + 1e-6) ? [k] : []));
}

/** The run's guard steps must end on exactly these events, in order, and nowhere else. */
function expectGuardStepsAt(run: readonly Step[], events: readonly number[]): void {
  const ends = guardSteps(run).map((k) => run[k]!.end);
  expect(ends.length, `guard steps end at ${ends.join(", ")}`).toBe(events.length);
  events.forEach((t, i) => {
    expect(Math.abs(ends[i]! - t), `guard step ${String(i)}`).toBeLessThanOrEqual(Math.max(1e-15, t * 1e-12));
  });
}

/** Seconds the load reads above 2.5 V, each step counted at its end value. */
function highTime(run: readonly Step[]): number {
  return run.reduce((sum, step) => (step.v > 2.5 ? sum + step.end - step.start : sum), 0);
}

/** How many of the run's periods show the load above 2.5 V at least once. */
function periodsSeenHigh(run: readonly Step[], period: number): number {
  return new Set(run.filter((step) => step.v > 2.5).map((step) => Math.floor(step.end / period))).size;
}

/**
 * The source event at `t` must fall inside a step no longer than the 10 ns
 * guard step. The slack is the runner's own: a boundary rounded within it of
 * an event counts as landing on it.
 */
function expectCrossedInGuardStep(run: readonly Step[], t: number): void {
  const slack = Math.max(1e-15, t * 1e-12);
  const crossing = run.find((step) =>
    step.start <= t + slack && t <= step.end + slack && step.end - step.start <= H_MIN * (1 + 1e-6)
  );
  expect(crossing, `no step of 10 ns or less crosses the source event at ${String(t)} s`).toBeDefined();
}

describe("HeadlessRunner source events", () => {
  it("sees a 0.2 ms PWL pulse and lands on each of its knots, adaptive or fixed-step", () => {
    const pwl = "0:0,3e-3:0,3.000001e-3:5,3.2e-3:5,3.200001e-3:0";
    const circuit = sourceIntoLoad(part("gen", "signal_gen", ["pos", "neg"], { waveform: "pwl", pwl, rSource: 0 }));
    // The fixed 1 ms grid passes 3 ms and 4 ms, both outside the pulse.
    for (const options of [{}, { adaptive: false, fixedStepS: 1e-3 }]) {
      const run = steps(circuit, 0.02, options);
      expect(Math.max(...run.map((step) => step.v))).toBeCloseTo(5, 9);
      expect(Math.abs(highTime(run) - 2e-4)).toBeLessThan(1e-7);
      for (const knot of [3e-3, 3.000001e-3, 3.2e-3, 3.200001e-3]) expectCrossedInGuardStep(run, knot);
    }
  });

  it("sees every 20 us pulse of a 1 kHz signal_gen and lands on each corner", () => {
    const circuit = sourceIntoLoad(part("gen", "signal_gen", ["pos", "neg"], {
      waveform: "pulse", frequency: 1000, amplitude: 2.5, offset: 2.5, pw: 2e-5, tr: 1e-6, tf: 1e-6, rSource: 0,
    }));
    const run = steps(circuit, 0.02);
    expect(periodsSeenHigh(run, 1e-3)).toBe(20);
    // Above 2.5 V from mid-rise to mid-fall: tr/2 + pw + tf/2 = 21 us a period.
    expect(Math.abs(highTime(run) - 20 * 21e-6)).toBeLessThan(20 * 1e-6);
    for (let k = 0; k < 20; k++) {
      for (const corner of [0, 1e-6, 21e-6, 22e-6]) {
        if (k > 0 || corner > 0) expectCrossedInGuardStep(run, k * 1e-3 + corner);
      }
    }
  });

  it("sees every 20 us pulse of a pulse_gen", () => {
    const circuit = sourceIntoLoad(part("gen", "pulse_gen", ["pos", "neg"], {
      v1: 0, v2: 5, td: 0, tr: 1e-6, tf: 1e-6, pw: 2e-5, per: 1e-3,
    }));
    const run = steps(circuit, 0.02);
    expect(periodsSeenHigh(run, 1e-3)).toBe(20);
    expect(Math.abs(highTime(run) - 20 * 21e-6)).toBeLessThan(20 * 1e-6);
  });

  it("lands on a clock's edges, on the ceiling's grid or between its steps", () => {
    // At 50% duty every edge sits on a multiple of the P/10 ceiling, where a
    // step can end a rounding error past it; at 25% each falling edge falls
    // halfway between two ceiling steps.
    for (const duty of [0.5, 0.25]) {
      const run = steps(sourceIntoLoad(part("clk", "clock_gen", ["out", "gnd"], { frequency: 1000, duty })), 0.02);
      expect(Math.abs(highTime(run) - 20 * duty * 1e-3)).toBeLessThan(20 * 2 * H_MIN);
      for (let k = 0; k < 20; k++) {
        if (k > 0) expectCrossedInGuardStep(run, k * 1e-3);
        expectCrossedInGuardStep(run, (k + duty) * 1e-3);
      }
    }
  });

  it("resumes the step it had once it has crossed an event", () => {
    // A resistive load gives the error estimate nothing to see, so the
    // controller sits at a 1 kHz clock's 100 us ceiling. Restarting from the
    // guard step would cost about 13 doubling steps after every edge.
    const run = steps(sourceIntoLoad(part("clk", "clock_gen", ["out", "gnd"], { frequency: 1000, duty: 0.5 })), 0.02);
    const guards = guardSteps(run).filter((k) => k + 1 < run.length);
    expect(guards.length).toBe(39);
    for (const k of guards) expect(run[k + 1]!.h).toBe(1e-4);
  });

  it("takes each event's guard step, and the step after it, by backward Euler under trap", () => {
    // Trapezoidal integration carries the derivative from before a jump
    // across it, and rings. The runner marks the step that lands before an
    // event, and the guard step across it, as discontinuities, so the engine
    // takes the step after each by backward Euler.
    const runner = new HeadlessRunner({ integrationMethod: "trap" });
    runner.load(rc(part("gen", "signal_gen", ["pos", "neg"], {
      waveform: "square", frequency: 1000, amplitude: 2.5, offset: 2.5, rSource: 0,
    })));
    const run: Step[] = [];
    const marked: boolean[] = [];
    let start = 0;
    runner.run({
      durationS: 5e-3,
      onSample: (sample) => {
        run.push({ start, end: sample.simTime, h: sample.h, v: 0 });
        start = sample.simTime;
        marked.push(runner.engine.saveState().integrationBeNextStep === true);
      },
    });
    const guards = guardSteps(run);
    expect(guards.length).toBe(10);
    for (const k of guards) expect([marked[k - 1], marked[k]]).toEqual([true, true]);
    // Away from the events the runner is back on trapezoidal steps.
    expect(marked.filter((m) => !m).length).toBeGreaterThan(run.length / 2);
  });

  it("lands on a phase-shifted square's edges", () => {
    // 90 degrees moves the edges to 0.25 ms and 0.75 ms into each period.
    const run = steps(sourceIntoLoad(part("gen", "signal_gen", ["pos", "neg"], {
      waveform: "square", frequency: 1000, amplitude: 2.5, offset: 2.5, phaseDeg: 90, rSource: 0,
    })), 0.01);
    for (let k = 0; k < 10; k++) {
      expectCrossedInGuardStep(run, (k + 0.25) * 1e-3);
      expectCrossedInGuardStep(run, (k + 0.75) * 1e-3);
    }
  });

  it("lands on a delayed source's start and on nothing before it", () => {
    // Until its delay a source holds its first value, so its start is its
    // only event there. This square, 90 degrees in, then falls at 1.25 ms.
    const square = steps(sourceIntoLoad(part("gen", "signal_gen", ["pos", "neg"], {
      waveform: "square", frequency: 1000, amplitude: 2.5, offset: 2.5, phaseDeg: 90, delay: 1e-3, rSource: 0,
    })), 3e-3);
    expectGuardStepsAt(square, [1e-3, 1.25e-3, 1.75e-3, 2.25e-3, 2.75e-3]);
    const sine = steps(sourceIntoLoad(part("gen", "signal_gen", ["pos", "neg"], {
      waveform: "sine", frequency: 1000, amplitude: 2.5, offset: 2.5, delay: 1e-3, rSource: 0,
    })), 3e-3);
    expectGuardStepsAt(sine, [1e-3]);
  });

  it("lands on every 0.1 ms noise sample, counted from the delay", () => {
    const run = steps(sourceIntoLoad(part("gen", "signal_gen", ["pos", "neg"], {
      waveform: "noise", amplitude: 1, offset: 2.5, delay: 5e-5, rSource: 0,
    })), 2e-3);
    expectGuardStepsAt(run, Array.from({ length: 20 }, (_, k) => 5e-5 + k * 1e-4));
  });

  it("lands on every drop of a ramp", () => {
    // A rising sawtooth falls back at each period's start, which 90 degrees
    // moves to 0.75 ms into the period.
    const run = steps(sourceIntoLoad(part("gen", "signal_gen", ["pos", "neg"], {
      waveform: "ramp", frequency: 1000, amplitude: 2.5, offset: 2.5, phaseDeg: 90, rSource: 0,
    })), 5e-3);
    expectGuardStepsAt(run, [0.75e-3, 1.75e-3, 2.75e-3, 3.75e-3, 4.75e-3]);
  });

  it("sees a one-shot pulse_source's pulse and lands on each corner", () => {
    // per 0 plays a single pulse, so there is no period to set a step ceiling.
    // SPICE PULSE cards load as pulse_source.
    const run = steps(sourceIntoLoad(part("gen", "pulse_source", ["pos", "neg"], {
      v1: 0, v2: 5, td: 1e-3, tr: 1e-6, tf: 1e-6, pw: 2e-5, per: 0,
    })), 5e-3);
    expect(Math.abs(highTime(run) - 21e-6)).toBeLessThan(1e-6);
    expectGuardStepsAt(run, [1e-3, 1.001e-3, 1.021e-3, 1.022e-3]);
  });
});

/** A source charging 100 nF through 1 kohm; the capacitor's top net is the probe. */
function rc(source: SimComponent, extra: SimComponent[] = [], extraWires: Wire[] = []): SimCircuit {
  const [pos, neg] = pinsOf(source);
  return {
    components: [
      source,
      part("r", "resistor", ["a", "b"], { resistance: 1000 }),
      part("c", "capacitor", ["a", "b"], { capacitance: 1e-7 }),
      ...extra,
    ],
    wires: [wire(source.id, pos, "r", "a"), wire("r", "b", "c", "a"), wire("c", "b", source.id, neg), ...extraWires],
  };
}

/** NE555 astable, RA 1 kohm, RB 10 kohm, 100 nF: about 690 Hz. */
function astable(): SimCircuit {
  return {
    components: [
      part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("ra", "resistor", ["a", "b"], { resistance: 1_000 }),
      part("rb", "resistor", ["a", "b"], { resistance: 10_000 }),
      part("c", "capacitor", ["a", "b"], { capacitance: 1e-7 }),
      part("load", "resistor", ["a", "b"], { resistance: 1_000 }),
      part("timer", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]),
    ],
    wires: [
      wire("vdd", "pos", "timer", "8"), wire("vdd", "pos", "timer", "4"), wire("vdd", "neg", "timer", "1"),
      wire("vdd", "pos", "ra", "a"), wire("ra", "b", "timer", "7"), wire("timer", "7", "rb", "a"),
      wire("rb", "b", "timer", "6"), wire("timer", "6", "timer", "2"), wire("timer", "6", "c", "a"),
      wire("c", "b", "vdd", "neg"), wire("timer", "3", "load", "a"), wire("load", "b", "vdd", "neg"),
    ],
  };
}

interface Fingerprint {
  accepted: number;
  rejected: number;
  failed: number;
  /** Sum of squared accepted steps: moves if any step boundary moves. */
  sumH2: number;
  /** Sum of v * h at the probe: moves if the trajectory does. */
  integral: number;
  final: number;
}

/**
 * Steps with the source-event landing switched off: the path from before
 * source events existed, on this platform, in this process.
 */
class PreSourceEventRunner extends HeadlessRunner {
  protected override _sourceEvents(): never[] {
    return [];
  }
}

function fingerprint(
  circuit: SimCircuit,
  durationS: number,
  options: RunOptions = {},
  method?: "be" | "trap",
  withoutEvents = false,
): Fingerprint {
  const Runner = withoutEvents ? PreSourceEventRunner : HeadlessRunner;
  const runner = new Runner(method ? { integrationMethod: method } : undefined);
  runner.load(circuit);
  const net = runner.netIdFor("c", "a")!;
  let sumH2 = 0;
  let integral = 0;
  let final = 0;
  const result = runner.run({
    durationS,
    ...options,
    onSample: (sample) => {
      sumH2 += sample.h * sample.h;
      integral += (sample.netV[net] ?? 0) * sample.h;
      final = sample.netV[net] ?? 0;
    },
  });
  return { accepted: result.acceptedSteps, rejected: result.rejectedSteps, failed: result.failedSteps, sumH2, integral, final };
}

const battery = part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 });
const generator = (waveform: string): SimComponent => part("gen", "signal_gen", ["pos", "neg"], { waveform });
const disabledPulse = part("off", "signal_gen", ["pos", "neg"], { waveform: "pulse", pw: 2e-5, enabled: 0 });

/**
 * Runs with nothing to land on: no timed source, a disabled one, or a
 * continuous waveform whose only event is its start at t = 0. The `was`
 * capture is the dense backend at brownout 0.6.1 on macOS; platforms differ
 * from it in the last ulps (V8's transcendentals differ, and the trapezoidal
 * battery run reaches these numbers only to 1e-12), so identity against the
 * capture is asserted on integers and to 1e-6 on floats, and the exact
 * identity claim — landing changes nothing when there is nothing to land on
 * — is made against the same run's own pre-source-event reference.
 */
const NO_SOURCE_EVENTS: Record<string, { run: (withoutEvents?: boolean) => Fingerprint; was: Fingerprint }> = {
  "battery RC": {
    run: (withoutEvents) => fingerprint(rc(battery), 5e-3, {}, undefined, withoutEvents),
    was: { accepted: 62, rejected: 0, failed: 0, sumH2: 0.000005614181152085763, integral: 0.02451080960719315, final: 4.999999993157764 },
  },
  "battery RC, trapezoidal": {
    run: (withoutEvents) => fingerprint(rc(battery), 5e-3, {}, "trap", withoutEvents),
    was: { accepted: 55, rejected: 0, failed: 0, sumH2: 0.000003853193324752912, integral: 0.024543893870414314, final: 4.999999994947919 },
  },
  "battery RC, fixed 10 us": {
    run: (withoutEvents) => fingerprint(rc(battery), 2e-3, { adaptive: false, fixedStepS: 1e-5 }, undefined, withoutEvents),
    was: { accepted: 200, rejected: 0, failed: 0, sumH2: 1.9999999999999908e-8, integral: 0.009499999993632893, final: 4.999999968671096 },
  },
  "555 astable": {
    run: (withoutEvents) => fingerprint(astable(), 20e-3, {}, undefined, withoutEvents),
    was: { accepted: 357, rejected: 39, failed: 0, sumH2: 0.0000012260511645207803, integral: 0.04931132477725497, final: 2.059396382727353 },
  },
  "sine signal_gen into RC": {
    run: (withoutEvents) => fingerprint(rc(generator("sine")), 5e-3, {}, undefined, withoutEvents),
    was: { accepted: 480, rejected: 26, failed: 0, sumH2: 6.896038561936105e-8, integral: 0.012321543054380414, final: 1.3613673830839441 },
  },
  "triangle signal_gen into RC": {
    run: (withoutEvents) => fingerprint(rc(generator("triangle")), 5e-3, {}, undefined, withoutEvents),
    was: { accepted: 387, rejected: 22, failed: 0, sumH2: 9.170639142993196e-8, integral: 0.012361299662034893, final: 1.0279127429154946 },
  },
  "dc signal_gen into RC": {
    run: (withoutEvents) => fingerprint(rc(generator("dc")), 5e-3, {}, undefined, withoutEvents),
    was: { accepted: 60, rejected: 0, failed: 0, sumH2: 0.0000050382470645252705, integral: 0.012243271233703936, final: 2.4999999957050196 },
  },
  "disabled pulse signal_gen beside a battery RC": {
    run: (withoutEvents) => fingerprint(rc(battery, [disabledPulse], [wire("off", "pos", "c", "a"), wire("off", "neg", "c", "b")]), 5e-3, {}, undefined, withoutEvents),
    was: { accepted: 62, rejected: 0, failed: 0, sumH2: 0.000005614181152085763, integral: 0.02451080960719315, final: 4.999999993157764 },
  },
};

/** The forced-sparse lane eliminates in another order, so its floats match the dense capture to roundoff only. */
const FORCED_SPARSE = process.env.SIMCORE_LINEAR_BACKEND === "sparse";

describe("HeadlessRunner with no source events", () => {
  beforeAll(() => {
    if (!FORCED_SPARSE) setLinearSystemBackendForTests("dense");
  });
  afterAll(() => {
    setLinearSystemBackendForTests(null);
  });

  for (const [name, { run, was }] of Object.entries(NO_SOURCE_EVENTS)) {
    it(`steps the ${name} exactly as before`, () => {
      const before = run(true);
      const now = run(false);
      // Exact identity, on every platform and backend: with no event to
      // land on, the landing changes nothing.
      expect(now).toEqual(before);
      // The 0.6.1 capture: integers exactly, floats to 1e-6 (see the map's
      // comment for why not bit-for-bit).
      expect([now.accepted, now.rejected, now.failed]).toEqual([was.accepted, was.rejected, was.failed]);
      for (const key of ["sumH2", "integral", "final"] as const) {
        expect(Math.abs(now[key] - was[key]), key).toBeLessThanOrEqual(1e-9 + 1e-6 * Math.abs(was[key]));
      }
    });
  }
});
