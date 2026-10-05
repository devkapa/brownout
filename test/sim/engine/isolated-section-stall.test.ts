/**
 * A breadboard section wired to a rail that reaches nothing froze the
 * simulation. In de:volt, a 555 blinker wired grounds-first (pin 1, the
 * timing capacitor and the LED's cathode to a bottom rail not yet bridged to
 * the battery) showed SETTLING with fail=1/1 and held the clock until the
 * rail was bridged. The 555's supply current had no way back to the battery
 * except through the 555 itself. Powered, its output stage pulled the
 * floating ground rail up to its supply, which left it unpowered; unpowered,
 * the rail dropped back to the node shunts' 0 V, which powered it again.
 * Newton alternated between the two on every iterate, so every step failed,
 * including at the hosts' 10 ns floor. Every release back to 0.2.0 did this.
 */
import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { estimateStepError, nextStepFactor } from "../../../src/sim/adaptive-step.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

const part = (
  id: string,
  kind: string,
  catalogUid: string,
  pins: string[],
  params: Record<string, number | string> = {},
): Component => ({ id, kind, catalogUid, pins: pins.map((pinId) => ({ id: pinId })), params });

const wire = (fromComponent: string, fromPin: string, toComponent: string, toPin: string) => ({
  from_component: fromComponent,
  from_pin: fromPin,
  to_component: toComponent,
  to_pin: toPin,
});

const TIMER_PINS = ["1", "2", "3", "4", "5", "6", "7", "8"];
const COUNTER_PINS = [
  "q5", "q1", "q0", "q2", "q6", "q7", "q3", "gnd",
  "q8", "q4", "q9", "co", "clkinh", "clk", "reset", "vcc",
];

/**
 * A 555 blinker caught half wired, as it was when the clock froze: pin 8 and
 * R1 are on the battery's rail, R2's far leg has not reached pin 6 yet, pin
 * 1, C1's negative leg and the LED's cathode share the bottom ground rail,
 * and pin 4 is alone on the bottom supply rail. `bridged` wires the bottom
 * rails to the battery; without it they reach nothing.
 */
function halfWiredBlinker(bridged: boolean): SimCircuit {
  const circuit: SimCircuit = {
    components: [
      part("battery", "battery_pack", "supply-5v", ["pos", "neg"], {
        voltage: 6, rInternal: 0.9, capacityAh: 2.5, charge: 1,
      }),
      part("timer", "ne555", "ne555", TIMER_PINS),
      part("r1", "resistor", "resistor", ["a", "b"], { resistance: 1000 }),
      part("r2", "resistor", "resistor", ["a", "b"], { resistance: 47000 }),
      part("c1", "capacitor", "cap-electrolytic", ["a", "b"], {
        capacitance: 10e-6, esr: 0.2, leakageResistance: 1e6,
      }),
      part("r3", "resistor", "resistor", ["a", "b"], { resistance: 330 }),
      part("led", "led", "led-red", ["a", "k"], { color: "red", vf: 1.8 }),
    ],
    wires: [
      wire("timer", "8", "battery", "pos"),
      wire("r1", "a", "battery", "pos"),
      wire("r1", "b", "timer", "7"),
      wire("r2", "a", "timer", "7"),
      wire("c1", "a", "timer", "2"),
      wire("c1", "b", "timer", "1"),
      wire("r3", "a", "timer", "3"),
      wire("r3", "b", "led", "a"),
      wire("led", "k", "timer", "1"),
    ],
  };
  if (bridged) {
    circuit.wires.push(wire("timer", "1", "battery", "neg"), wire("timer", "4", "battery", "pos"));
  }
  return circuit;
}

/** A 9 V battery lighting its own LED, with no connection to anything else. */
function withSecondBattery(circuit: SimCircuit): SimCircuit {
  return {
    components: [
      ...circuit.components,
      part("battery2", "battery_pack", "battery-9v", ["pos", "neg"], {
        voltage: 9, rInternal: 1.5, capacityAh: 0.55, charge: 1,
      }),
      part("r4", "resistor", "resistor", ["a", "b"], { resistance: 1000 }),
      part("led2", "led", "led-green", ["a", "k"], { color: "green", vf: 2.1 }),
    ],
    wires: [
      ...circuit.wires,
      wire("battery2", "pos", "r4", "a"),
      wire("r4", "b", "led2", "a"),
      wire("led2", "k", "battery2", "neg"),
    ],
  };
}

/** A CD4017 clocked by the 555, its supply on the battery and its ground on the 555's ground rail. */
function withCounterOnTheGroundRail(circuit: SimCircuit): SimCircuit {
  return {
    components: [...circuit.components, part("counter", "cd4017", "ic-cd4017", COUNTER_PINS, { vcc: 5 })],
    wires: [
      ...circuit.wires,
      wire("counter", "vcc", "battery", "pos"),
      wire("counter", "gnd", "timer", "1"),
      wire("counter", "clk", "timer", "3"),
    ],
  };
}

// de:volt's browser host (packages/simcore/src/sim/sim.worker.ts) steps the
// engine in 1/60 s batches. WorkerStepLoop is its step loop for these
// circuits, which have no clock or pulse sources, MCUs or sensors, with the
// wall-clock budget left out so the test is deterministic. Every load
// restarts the controller at the 10 ns floor. A step that does not converge
// is retried at a quarter of its size, and one that fails at the floor ends
// the batch with nothing accepted, so a circuit that never converges holds
// the clock still batch after batch.
const H_MIN = 1e-8;
const H_MAX = 1e-2;
const ACTIVE_H_MAX = 2e-3;
const QUIET_FRACTION = 0.1;
// The worker's error-check gap for circuits whose edges are solver events (a 555's).
const EVENT_ALIGNED_ERROR_CHECK_GAP = 15;
const BATCH_S = 1 / 60;

interface Attempt {
  accepted: boolean;
  converged: boolean;
  ratio: number;
  factor: number;
}

/** A host that steps the engine: how far the clock got, and how many attempts did not converge. */
interface Host {
  readonly engine: SimEngine;
  load(circuit: SimCircuit): void;
  advance(durationS: number): { advancedS: number; failedSteps: number };
}

class WorkerStepLoop implements Host {
  readonly engine = new SimEngine();
  private h = H_MIN;
  private changing = true;
  private checkCountdown = 0;
  private validatedFactor = 1;

  constructor() {
    this.engine.captureStepBreakpoints = true;
  }

  load(circuit: SimCircuit): void {
    this.engine.load(circuit);
    this.h = H_MIN;
    this.changing = true;
    this.checkCountdown = 0;
    this.validatedFactor = 1;
  }

  advance(durationS: number): { advancedS: number; failedSteps: number } {
    const start = this.engine.simTime;
    let failedSteps = 0;
    for (let i = 0; i < Math.round(durationS / BATCH_S); i++) failedSteps += this.runBatch();
    return { advancedS: this.engine.simTime - start, failedSteps };
  }

  private runBatch(): number {
    let simmed = 0;
    let failedSteps = 0;
    while (simmed < BATCH_S - BATCH_S * 1e-12) {
      const maxH = this.changing ? ACTIVE_H_MAX : H_MAX;
      const h = Math.min(this.h, maxH, BATCH_S - simmed);
      const check = this.checkCountdown <= 0;
      const attempt = this.attempt(h, check);
      if (!attempt.accepted) {
        if (check) {
          this.checkCountdown = 0;
          this.validatedFactor = 1;
        }
        this.h = Math.max(H_MIN, h * attempt.factor);
        this.changing = true;
        if (!attempt.converged) failedSteps += 1;
        if (h <= H_MIN * 1.01) break;
        continue;
      }
      this.h = Math.max(H_MIN, Math.min(maxH, h * (check ? attempt.factor : this.validatedFactor)));
      if (check) {
        this.validatedFactor = attempt.factor;
        if (h > H_MIN * 2.01) this.changing = attempt.ratio * (H_MAX / h) ** 2 > QUIET_FRACTION;
        this.checkCountdown = attempt.ratio > 0.5 ? 0 : attempt.ratio > 0.1 ? 1 : EVENT_ALIGNED_ERROR_CHECK_GAP;
      } else {
        this.validatedFactor = 1 + (this.validatedFactor - 1) * 0.5;
        this.checkCountdown = Math.max(0, this.checkCountdown - 1);
      }
      simmed += h;
      this.engine.takeStepBreakpoints();
    }
    return failedSteps;
  }

  /** A checked attempt keeps two h/2 steps after comparing them with one h step. */
  private attempt(h: number, check: boolean): Attempt {
    const engine = this.engine;
    if (!check || h <= H_MIN * 2.01) {
      engine.step(h);
      if (!engine.lastConverged) return { accepted: false, converged: false, ratio: Infinity, factor: 0.25 };
      const factor = engine.lastIters > 15 ? 0.7 : engine.lastIters <= 3 ? 1.5 : 1;
      return { accepted: true, converged: true, ratio: 0, factor };
    }
    const before = engine.saveState();
    const failed = (): Attempt => {
      engine.restoreState(before);
      return { accepted: false, converged: false, ratio: Infinity, factor: 0.25 };
    };
    engine.step(h);
    if (!engine.lastConverged) return failed();
    const coarse = engine.captureErrorState();
    engine.restoreState(before);
    engine.step(h / 2);
    if (!engine.lastConverged) return failed();
    engine.step(h / 2);
    if (!engine.lastConverged) return failed();
    const { ratio } = estimateStepError(coarse, engine.captureErrorState());
    const factor = nextStepFactor(ratio);
    if (ratio > 1) {
      engine.restoreState(before);
      return { accepted: false, converged: true, ratio, factor };
    }
    return { accepted: true, converged: true, ratio, factor };
  }
}

function headlessHost(): Host {
  const runner = new HeadlessRunner();
  return {
    engine: runner.engine,
    load: (circuit) => runner.load(circuit),
    advance: (durationS) => {
      const run = runner.run({ durationS });
      return { advancedS: run.simulatedS, failedSteps: run.failedSteps };
    },
  };
}

const HOSTS: Array<[string, () => Host]> = [
  ["the HeadlessRunner", headlessHost],
  ["the worker's batch loop", () => new WorkerStepLoop()],
];

describe.each(HOSTS)("a 555 whose ground rail reaches nothing, under %s", (_name, makeHost) => {
  it("runs, with the clock advancing", () => {
    const host = makeHost();
    host.load(halfWiredBlinker(false));
    const run = host.advance(0.5);
    expect(run.advancedS).toBeCloseTo(0.5, 9);
    expect(run.failedSteps).toBe(0);
    // With no return for its supply the 555 cannot run, so the LED stays dark.
    expect(host.engine.getElementI().led).toBeCloseTo(0, 9);
  });

  it("keeps running when a reload cuts the rail off, and recovers when it is bridged again", () => {
    const host = makeHost();
    // Bridged, the 555's output is high (its trigger sits on the uncharged
    // capacitor) and the LED is lit.
    host.load(halfWiredBlinker(true));
    expect(host.advance(0.5).failedSteps).toBe(0);
    expect(host.engine.getElementI().led).toBeGreaterThan(10e-3);

    host.load(halfWiredBlinker(false));
    const cut = host.advance(0.5);
    expect(cut.advancedS).toBeCloseTo(0.5, 9);
    expect(cut.failedSteps).toBe(0);
    expect(host.engine.getElementI().led).toBeCloseTo(0, 9);

    host.load(halfWiredBlinker(true));
    const restored = host.advance(0.5);
    expect(restored.advancedS).toBeCloseTo(0.5, 9);
    expect(restored.failedSteps).toBe(0);
    expect(host.engine.getElementI().led).toBeGreaterThan(10e-3);
  });
});

describe("a section wired to a rail that reaches nothing", () => {
  it("leaves a 555 and a counter that share that rail unpowered instead of alternating", () => {
    // Each chip's pins reach the battery through the other, but neither
    // conducts until it is powered, so neither is the other's return.
    const runner = new HeadlessRunner();
    runner.load(withCounterOnTheGroundRail(halfWiredBlinker(false)));
    const run = runner.run({ durationS: 0.2 });
    expect(run.hitMinStep).toBe(false);
    expect(run.failedSteps).toBe(0);
    expect(run.simulatedS).toBeCloseTo(0.2, 9);

    runner.load(withCounterOnTheGroundRail(halfWiredBlinker(true)));
    expect(runner.run({ durationS: 0.2 }).failedSteps).toBe(0);
    expect(runner.snapshot().elementI.led).toBeGreaterThan(10e-3);
  });

  it("still reads an isolated section, and the floating rail, at 0 V", () => {
    const runner = new HeadlessRunner();
    runner.load(withSecondBattery(halfWiredBlinker(false)));
    const run = runner.run({ durationS: 0.2 });
    expect(run.hitMinStep).toBe(false);
    expect(run.failedSteps).toBe(0);

    const snap = runner.snapshot();
    expect(Math.abs(snap.netV[runner.netIdFor("timer", "1")!]!)).toBeLessThan(1e-9);
    // The second battery's loop keeps its own reference at its negative
    // terminal. The anchor is 1 S, so its node's voltage in volts is its
    // current in amps: only the node shunts' current flows through it.
    const negative = runner.netIdFor("battery2", "neg")!;
    expect(runner.engine.isolatedSectionAnchorRows()).toContain(runner.engine.netRow(negative));
    for (const row of runner.engine.isolatedSectionAnchorRows()) {
      const netId = Object.keys(snap.netV).find((id) => runner.engine.netRow(id) === row)!;
      expect(Math.abs(snap.netV[netId]!)).toBeLessThan(1e-9);
    }
    expect(snap.elementI.led2).toBeGreaterThan(6e-3);
  });

  it("powers a chain of chips listed consumer-first (the repeat to a fixed point, not one pass)", () => {
    // Each 74HC14 takes its supply from the previous one's output, and the
    // circuit lists them last-powered first, so one pass over the parts would
    // leave b and c waiting and read them unpowered.
    const pins = ["1a", "1y", "2a", "2y", "3a", "3y", "gnd", "4y", "4a", "5y", "5a", "6y", "6a", "vcc"];
    const hc14 = (id: string) => part(id, "74hc14", "ic-74hc14", pins, { vcc: 5 });
    const runner = new HeadlessRunner();
    runner.load({
      components: [
        part("battery", "battery_pack", "supply-5v", ["pos", "neg"], { voltage: 5, rInternal: 0.5, capacityAh: 2.5, charge: 1 }),
        hc14("c"),
        hc14("b"),
        hc14("a"),
      ],
      wires: [
        wire("a", "vcc", "battery", "pos"),
        wire("a", "gnd", "battery", "neg"),
        wire("a", "1a", "battery", "neg"),
        wire("b", "vcc", "a", "1y"),
        wire("b", "gnd", "battery", "neg"),
        wire("b", "1a", "battery", "neg"),
        wire("c", "vcc", "b", "1y"),
        wire("c", "gnd", "battery", "neg"),
      ],
    });
    runner.run({ durationS: 0.05 });
    expect(Object.keys(runner.snapshot().digitalState).some((key) => key.startsWith("c/"))).toBe(true);
  });
});
