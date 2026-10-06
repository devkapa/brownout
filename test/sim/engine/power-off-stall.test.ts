/**
 * Switching a 555 blinker's supply off froze the simulation. A 9 V blinker
 * switched off in either battery lead, or with its ground wire pulled, while
 * its timing capacitor held more than about 4 V left the chip fed only by
 * that charged capacitor, through R1 and R2. Read powered, its own draw
 * pulled its supply below its power-on threshold (90% of the 555's 4.5 V
 * minimum); read unpowered, the supply came back above it. Newton alternated
 * between the two on every iterate at every step size, so the hosts retried
 * down to their 10 ns floor and stopped the clock, at 18 of 24 moments across
 * the cycle. A 555 fed only by a decoupling capacitor did the same as the
 * capacitor drained through that threshold: the HeadlessRunner stopped 17 ms
 * after the rail's bridge to the battery was removed.
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

type Cut = "switched off in the negative lead" | "switched off in the positive lead" | "with its ground wire pulled";
const CUTS: Cut[] = [
  "switched off in the negative lead",
  "switched off in the positive lead",
  "with its ground wire pulled",
];

/**
 * A 9 V 555 blinker (R1 1 kohm, R2 47 kohm, C1 10 uF, about 1.5 Hz), whole
 * or with its supply cut the way `cut` says. A switched board carries its
 * switch either way, so on and off share one topology.
 */
function blinker(cut: Cut, whole: boolean): SimCircuit {
  const switchLead = cut === "switched off in the negative lead" ? "neg"
    : cut === "switched off in the positive lead" ? "pos" : null;
  const supply: [string, string] = switchLead === "pos" ? ["switch", "b"] : ["battery", "pos"];
  const ground: [string, string] = switchLead === "neg" ? ["switch", "b"] : ["battery", "neg"];
  const circuit: SimCircuit = {
    components: [
      part("battery", "battery_pack", "battery-9v", ["pos", "neg"], {
        voltage: 9, rInternal: 1.5, capacityAh: 0.55, charge: 1,
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
      wire("timer", "8", ...supply),
      wire("timer", "4", ...supply),
      wire("r1", "a", ...supply),
      wire("r1", "b", "timer", "7"),
      wire("r2", "a", "timer", "7"),
      wire("r2", "b", "timer", "6"),
      wire("timer", "6", "timer", "2"),
      wire("c1", "a", "timer", "2"),
      wire("c1", "b", "timer", "1"),
      wire("r3", "a", "timer", "3"),
      wire("r3", "b", "led", "a"),
      wire("led", "k", "timer", "1"),
    ],
  };
  if (switchLead) {
    circuit.components.push(part("switch", "switch", "switch-spst", ["a", "b"], { closed: whole ? 1 : 0, momentary: 0 }));
    circuit.wires.push(wire("switch", "a", "battery", switchLead));
  }
  if (whole || switchLead) circuit.wires.push(wire("timer", "1", ...ground));
  return circuit;
}

/**
 * A 555 caught half wired on a 6 V battery (R2's far leg loose, pins 6 and 2
 * apart) with a 100 uF decoupling capacitor across the bottom rails: pin 4
 * and the capacitor's + on the red rail, pin 1, C1's negative leg, the LED's
 * cathode and the capacitor's - on the blue rail. The red rail stays bridged
 * to the battery's +; `blueBridged` bridges the blue rail to its -. With the
 * blue bridge gone the 555 runs on the capacitor alone, and the capacitor
 * drains through the chip's threshold.
 */
function capacitorFedTimer(blueBridged: boolean, prefix = ""): SimCircuit {
  const id = (name: string) => prefix + name;
  const circuit: SimCircuit = {
    components: [
      part(id("battery"), "battery_pack", "supply-5v", ["pos", "neg"], {
        voltage: 6, rInternal: 0.9, capacityAh: 2.5, charge: 1,
      }),
      part(id("timer"), "ne555", "ne555", TIMER_PINS),
      part(id("r1"), "resistor", "resistor", ["a", "b"], { resistance: 1000 }),
      part(id("r2"), "resistor", "resistor", ["a", "b"], { resistance: 47000 }),
      part(id("c1"), "capacitor", "cap-electrolytic", ["a", "b"], {
        capacitance: 10e-6, esr: 0.2, leakageResistance: 1e6,
      }),
      part(id("r3"), "resistor", "resistor", ["a", "b"], { resistance: 330 }),
      part(id("led"), "led", "led-red", ["a", "k"], { color: "red", vf: 1.8 }),
      part(id("decoupling"), "capacitor", "cap-electrolytic", ["a", "b"], {
        capacitance: 100e-6, esr: 0.1, leakageResistance: 1e6,
      }),
    ],
    wires: [
      wire(id("timer"), "8", id("battery"), "pos"),
      wire(id("r1"), "a", id("battery"), "pos"),
      wire(id("r1"), "b", id("timer"), "7"),
      wire(id("r2"), "a", id("timer"), "7"),
      wire(id("c1"), "a", id("timer"), "2"),
      wire(id("c1"), "b", id("timer"), "1"),
      wire(id("r3"), "a", id("timer"), "3"),
      wire(id("r3"), "b", id("led"), "a"),
      wire(id("led"), "k", id("timer"), "1"),
      wire(id("decoupling"), "a", id("timer"), "4"),
      wire(id("decoupling"), "b", id("timer"), "1"),
      wire(id("timer"), "4", id("battery"), "pos"),
    ],
  };
  if (blueBridged) circuit.wires.push(wire(id("timer"), "1", id("battery"), "neg"));
  return circuit;
}

/** Two boards side by side, with nothing between them. */
function sideBySide(a: SimCircuit, b: SimCircuit): SimCircuit {
  return { components: [...a.components, ...b.components], wires: [...a.wires, ...b.wires] };
}

/**
 * A 555 fed from a 9 V battery through 1 kohm, its trigger and threshold
 * grounded so its output lights an LED through 330 ohm. Powered, its draw
 * pulls its supply far below the power-on threshold; unpowered, the full
 * 9 V returns. No capacitor is involved: it starves itself in DC as at
 * every step.
 */
function starvedTimer(): SimCircuit {
  return {
    components: [
      part("battery", "battery_pack", "battery-9v", ["pos", "neg"], {
        voltage: 9, rInternal: 1.5, capacityAh: 0.55, charge: 1,
      }),
      part("feed", "resistor", "resistor", ["a", "b"], { resistance: 1000 }),
      part("timer", "ne555", "ne555", TIMER_PINS),
      part("r3", "resistor", "resistor", ["a", "b"], { resistance: 330 }),
      part("led", "led", "led-red", ["a", "k"], { color: "red", vf: 1.8 }),
    ],
    wires: [
      wire("feed", "a", "battery", "pos"),
      wire("timer", "8", "feed", "b"),
      wire("timer", "4", "feed", "b"),
      wire("timer", "1", "battery", "neg"),
      wire("timer", "2", "battery", "neg"),
      wire("timer", "6", "battery", "neg"),
      wire("timer", "3", "r3", "a"),
      wire("r3", "b", "led", "a"),
      wire("led", "k", "battery", "neg"),
    ],
  };
}

const INVERTER_PINS = ["1a", "1y", "2a", "2y", "3a", "3y", "gnd", "4y", "4a", "5y", "5a", "6y", "6a", "vcc"];
const INVERTER_INPUTS = ["1a", "2a", "3a", "4a", "5a", "6a"];
// A 74LS04 powers on at 90% of its 4.75 V minimum supply.
const INVERTER_POWER_ON_V = 0.9 * 4.75;

/**
 * A pulse source ramping from 0 to 9 V over 4 ms, through 22 ohm, into a
 * 74LS04 with its inputs grounded, one output lighting an LED through
 * 220 ohm. The chip is powered only while its supply sits between 90% of its
 * 4.75 V minimum and 110% of its 5.25 V maximum, which the ramp crosses in
 * about 0.7 ms.
 */
function rampedInverter(): SimCircuit {
  return {
    components: [
      part("source", "pulse_gen", "pulse-source", ["pos", "neg"], {
        v1: 0, v2: 9, td: 1e-3, tr: 4e-3, tf: 4e-3, pw: 0.02, per: 0.05,
      }),
      part("feed", "resistor", "resistor", ["a", "b"], { resistance: 22 }),
      part("inverter", "74ls04", "ic-74ls04", INVERTER_PINS, { vcc: 5 }),
      part("r1", "resistor", "resistor", ["a", "b"], { resistance: 220 }),
      part("led", "led", "led-red", ["a", "k"], { color: "red", vf: 1.9 }),
    ],
    wires: [
      wire("feed", "a", "source", "pos"),
      wire("inverter", "vcc", "feed", "b"),
      wire("inverter", "gnd", "source", "neg"),
      ...INVERTER_INPUTS.map((pin) => wire("inverter", pin, "source", "neg")),
      wire("inverter", "1y", "r1", "a"),
      wire("r1", "b", "led", "a"),
      wire("led", "k", "source", "neg"),
    ],
  };
}

/**
 * A 74LS04 powering up from a 9 V battery through 100 ohm onto a rail held
 * by a 100 uF electrolytic, its inputs grounded and three outputs lighting
 * LEDs through 220 ohm. Switched on, the chip draws its current through the
 * capacitor's 0.2 ohm ESR at once, dropping the rail by a few millivolts, so
 * a step that carries the rail up through the chip's threshold cannot end
 * with the chip on unless it is long enough to climb that drop as well.
 */
function coldPowerUp(): SimCircuit {
  const circuit: SimCircuit = {
    components: [
      part("battery", "battery_pack", "battery-9v", ["pos", "neg"], {
        voltage: 9, rInternal: 1.5, capacityAh: 0.55, charge: 1,
      }),
      part("feed", "resistor", "resistor", ["a", "b"], { resistance: 100 }),
      part("hold", "capacitor", "cap-electrolytic", ["a", "b"], {
        capacitance: 100e-6, esr: 0.2, leakageResistance: 1e6,
      }),
      part("inverter", "74ls04", "ic-74ls04", INVERTER_PINS, { vcc: 5 }),
    ],
    wires: [
      wire("feed", "a", "battery", "pos"),
      wire("hold", "a", "feed", "b"),
      wire("hold", "b", "battery", "neg"),
      wire("inverter", "vcc", "feed", "b"),
      wire("inverter", "gnd", "battery", "neg"),
      ...INVERTER_INPUTS.map((pin) => wire("inverter", pin, "battery", "neg")),
    ],
  };
  ["1y", "2y", "3y"].forEach((output, index) => {
    circuit.components.push(
      part(`r${index}`, "resistor", "resistor", ["a", "b"], { resistance: 220 }),
      part(`led${index}`, "led", "led-red", ["a", "k"], { color: "red", vf: 1.9 }),
    );
    circuit.wires.push(
      wire("inverter", output, `r${index}`, "a"),
      wire(`r${index}`, "b", `led${index}`, "a"),
      wire(`led${index}`, "k", "battery", "neg"),
    );
  });
  return circuit;
}

/**
 * Two 555 blinkers powering up together from a 9 V battery through 100 ohm
 * onto a rail held by a 100 uF electrolytic. Each holds its reset low with a
 * 10 uF capacitor charged through 10 kohm, so its output latch flips as the
 * chip powers up, and the engine splits the step at the flip. A 220 ohm R1
 * makes the discharge transistor that the flip switches on a heavy load.
 */
function twinTimerPowerUp(): SimCircuit {
  const circuit: SimCircuit = {
    components: [
      part("battery", "battery_pack", "battery-9v", ["pos", "neg"], {
        voltage: 9, rInternal: 1.5, capacityAh: 0.55, charge: 1,
      }),
      part("feed", "resistor", "resistor", ["a", "b"], { resistance: 100 }),
      part("hold", "capacitor", "cap-electrolytic", ["a", "b"], {
        capacitance: 100e-6, esr: 0.2, leakageResistance: 1e6,
      }),
    ],
    wires: [
      wire("feed", "a", "battery", "pos"),
      wire("hold", "a", "feed", "b"),
      wire("hold", "b", "battery", "neg"),
    ],
  };
  for (const side of ["a", "b"]) {
    const id = (name: string) => `${name}_${side}`;
    circuit.components.push(
      part(id("timer"), "ne555", "ne555", TIMER_PINS),
      part(id("r1"), "resistor", "resistor", ["a", "b"], { resistance: 220 }),
      part(id("r2"), "resistor", "resistor", ["a", "b"], { resistance: 47000 }),
      part(id("c1"), "capacitor", "cap-electrolytic", ["a", "b"], {
        capacitance: 10e-6, esr: 0.2, leakageResistance: 1e6,
      }),
      part(id("r3"), "resistor", "resistor", ["a", "b"], { resistance: 330 }),
      part(id("led"), "led", "led-red", ["a", "k"], { color: "red", vf: 1.8 }),
      part(id("reset_r"), "resistor", "resistor", ["a", "b"], { resistance: 10000 }),
      part(id("reset_c"), "capacitor", "cap-ceramic", ["a", "b"], {
        capacitance: 10e-6, esr: 0.05, leakageResistance: 1e9,
      }),
    );
    circuit.wires.push(
      wire(id("reset_r"), "a", "feed", "b"),
      wire(id("reset_r"), "b", id("timer"), "4"),
      wire(id("reset_c"), "a", id("timer"), "4"),
      wire(id("reset_c"), "b", "battery", "neg"),
      wire(id("timer"), "8", "feed", "b"),
      wire(id("timer"), "1", "battery", "neg"),
      wire(id("r1"), "a", "feed", "b"),
      wire(id("r1"), "b", id("timer"), "7"),
      wire(id("r2"), "a", id("timer"), "7"),
      wire(id("r2"), "b", id("timer"), "6"),
      wire(id("timer"), "6", id("timer"), "2"),
      wire(id("c1"), "a", id("timer"), "2"),
      wire(id("c1"), "b", "battery", "neg"),
      wire(id("r3"), "a", id("timer"), "3"),
      wire(id("r3"), "b", id("led"), "a"),
      wire(id("led"), "k", "battery", "neg"),
    );
  }
  return circuit;
}

/** The inverter's supply, vcc to gnd, in a set of net voltages. */
function inverterSupply(engine: SimEngine, netV: Record<string, number>): number {
  const vcc = engine.getNetIdForPin("inverter", "vcc");
  const gnd = engine.getNetIdForPin("inverter", "gnd");
  return (vcc === undefined ? Number.NaN : netV[vcc] ?? 0) - (gnd === undefined ? 0 : netV[gnd] ?? 0);
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
// The worker's error-check gaps: longer for circuits whose edges are solver
// events (a 555's), the default for the rest.
const EVENT_ALIGNED_ERROR_CHECK_GAP = 15;
const DEFAULT_ERROR_CHECK_GAP = 3;
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
  /** Called after every accepted step, with the engine at its end. */
  onAccepted: (() => void) | null = null;
  private h = H_MIN;
  private changing = true;
  private checkCountdown = 0;
  private validatedFactor = 1;
  private checkGap = EVENT_ALIGNED_ERROR_CHECK_GAP;

  constructor() {
    this.engine.captureStepBreakpoints = true;
  }

  load(circuit: SimCircuit): void {
    this.engine.load(circuit);
    this.h = H_MIN;
    this.changing = true;
    this.checkCountdown = 0;
    this.validatedFactor = 1;
    this.checkGap = circuit.components.some((component) => component.kind === "ne555")
      ? EVENT_ALIGNED_ERROR_CHECK_GAP
      : DEFAULT_ERROR_CHECK_GAP;
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
        this.checkCountdown = attempt.ratio > 0.5 ? 0 : attempt.ratio > 0.1 ? 1 : this.checkGap;
      } else {
        this.validatedFactor = 1 + (this.validatedFactor - 1) * 0.5;
        this.checkCountdown = Math.max(0, this.checkCountdown - 1);
      }
      simmed += h;
      this.engine.takeStepBreakpoints();
      this.onAccepted?.();
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

/** How each fallback the engine tried ended (see SimEngine._solveHoldingUnholdableOff). */
interface HoldOffCounts {
  accepted: number;
  probeFailed: number;
  partsHold: number;
  supplyLow: number;
  heldOffFailed: number;
  heldAtFloor: number;
}
const holdOffCounts = (engine: SimEngine): HoldOffCounts =>
  ({ ...(engine as unknown as { _holdOffCounts: HoldOffCounts })._holdOffCounts });

/** Rejects every step the fallback would rescue, as the engine did before it. */
function disableFallback(engine: SimEngine): void {
  (engine as unknown as { _solveHoldingUnholdableOff: () => null })._solveHoldingUnholdableOff = () => null;
}

// One cycle of the 9 V blinker is about 0.69 s.
const CYCLE_S = 0.69;
const MOMENTS = 24;

describe.each(HOSTS)("a 9 V 555 blinker whose supply is cut, under %s", (_name, makeHost) => {
  it.each(CUTS)("keeps its clock running at every moment of its cycle when %s", (cut) => {
    const host = makeHost();
    host.load(blinker(cut, true));
    host.advance(0.6);
    const stopped: string[] = [];
    for (let moment = 0; moment < MOMENTS; moment++) {
      const running = host.engine.saveState();
      host.load(blinker(cut, false));
      const off = host.advance(0.1);
      // With nothing returning its supply current the chip is off, and so is the LED.
      const led = host.engine.getElementI().led ?? Number.NaN;
      if (Math.abs(off.advancedS - 0.1) > 1e-9 || off.failedSteps > 0 || Math.abs(led) > 1e-6) {
        stopped.push(`moment ${moment}: ${off.advancedS.toFixed(4)} s, ${off.failedSteps} failed, LED ${led.toExponential(1)} A`);
      }
      host.load(blinker(cut, true));
      host.engine.restoreState(running);
      host.advance(CYCLE_S / MOMENTS);
    }
    expect(stopped).toEqual([]);
    expect(holdOffCounts(host.engine).accepted).toBeGreaterThan(0);

    // Switched back on, the blinker blinks again: two cycles show the LED lit and dark.
    let lit = 0;
    let dark = 0;
    for (let slice = 0; slice < 28; slice++) {
      const restored = host.advance(0.05);
      expect(restored.advancedS).toBeCloseTo(0.05, 9);
      expect(restored.failedSteps).toBe(0);
      const led = host.engine.getElementI().led!;
      if (led > 5e-3) lit += 1;
      if (led < 1e-4) dark += 1;
    }
    expect(lit).toBeGreaterThan(0);
    expect(dark).toBeGreaterThan(0);
  });
});

describe("a 555 fed only by a decoupling capacitor", () => {
  it("keeps the clock running as the capacitor drains through the chip's threshold", () => {
    // The HeadlessRunner checks every step's error, and its steps walked the
    // capacitor into the few millivolts where the chip can hold its supply
    // at no step size; the worker's longer unchecked runs stepped past them.
    for (const [, makeHost] of HOSTS) {
      const host = makeHost();
      host.load(capacitorFedTimer(true));
      expect(host.advance(1).failedSteps).toBe(0);
      expect(host.engine.getElementI().led).toBeGreaterThan(10e-3);

      host.load(capacitorFedTimer(false));
      const drained = host.advance(0.5);
      expect(drained.advancedS).toBeCloseTo(0.5, 9);
      // The capacitor ran the chip until it could no longer hold its supply.
      expect(host.engine.getElementI().led).toBeCloseTo(0, 6);
    }
  });
});

describe("when neither held state is consistent", () => {
  it("rejects the step exactly as before, and the step controller finds one that is", () => {
    // The switched-off blinker cannot hold its supply at any step size. The
    // capacitor-fed timer beside it holds its own at 1 ps but alternates at
    // a long step as its capacitor reaches the threshold, so on those steps
    // the re-solve with the blinker held off still fails, and the step must
    // be rejected without a trace before the controller shrinks it.
    const board = (powered: boolean) => sideBySide(
      blinker("switched off in the negative lead", powered),
      capacitorFedTimer(powered, "fed_"),
    );
    for (const [, makeHost] of HOSTS) {
      const plain = makeHost();
      plain.load(board(true));
      plain.advance(1);
      plain.load(board(false));
      expect(plain.advance(0.5).advancedS).toBeCloseTo(0.5, 9);

      const host = makeHost();
      const engine = host.engine;
      host.load(board(true));
      host.advance(1);

      const internal = engine as unknown as {
        mna: { factorizationCount: number; factorizationReuseCount: number; patternCheckpoint?(): number };
        _solveHoldingUnholdableOff: (...args: unknown[]) => Float64Array | null;
      };
      // Everything a plain rejected solve leaves behind, the engine's own
      // matrix included: its pattern and factorization history steer every
      // later solve's rounding.
      const fingerprint = () => ({
        state: engine.saveState(),
        netV: { ...engine.getNetV() },
        lastIters: engine.lastIters,
        lastConverged: engine.lastConverged,
        lastMatrixSingular: engine.lastMatrixSingular,
        lastMatrixIllConditioned: engine.lastMatrixIllConditioned,
        lastRelativeResidual: engine.lastRelativeResidual,
        factorizations: internal.mna.factorizationCount,
        factorizationReuses: internal.mna.factorizationReuseCount,
        pattern: internal.mna.patternCheckpoint?.(),
      });
      const fallback = internal._solveHoldingUnholdableOff.bind(engine);
      let rejected = 0;
      internal._solveHoldingUnholdableOff = (...args: unknown[]) => {
        const matrix = internal.mna;
        const before = fingerprint();
        const held = fallback(...args);
        if (!held) {
          rejected += 1;
          expect(internal.mna).toBe(matrix);
          expect(fingerprint()).toEqual(before);
        }
        return held;
      };

      host.load(board(false));
      const cut = host.advance(0.5);
      expect(cut.advancedS).toBeCloseTo(0.5, 9);
      const counts = holdOffCounts(engine);
      expect(counts.heldOffFailed).toBeGreaterThan(0);
      expect(counts.accepted).toBeGreaterThan(0);
      expect(rejected).toBe(counts.probeFailed + counts.partsHold + counts.supplyLow + counts.heldOffFailed);
    }
  });
});

describe("a supply rising through a chip's power-on threshold", () => {
  it("is left to the host to land with a shorter step, exactly as before", () => {
    // A long step from well below the threshold ends above it with the chip
    // off and below it with the chip on, its draw dropping the supply across
    // the 22 ohm feed, so the step fails. The chip is not starving itself:
    // its supply is still low at the step's start. The host shortens the
    // step and lands the crossing, and the LED lights while the supply is in
    // the chip's window. Held off instead, the chip would miss the window and
    // the LED would never light.
    const run = (withFallback: boolean) => {
      const runner = new HeadlessRunner();
      if (!withFallback) disableFallback(runner.engine);
      runner.load(rampedInverter());
      const samples: number[][] = [];
      let brightest = 0;
      const result = runner.run({
        durationS: 8e-3,
        onSample: (sample) => {
          samples.push([sample.simTime, ...Object.values(sample.netV), ...Object.values(sample.elementI)]);
          brightest = Math.max(brightest, sample.elementI.led ?? 0);
        },
      });
      return { result, samples, brightest, counts: holdOffCounts(runner.engine) };
    };
    const asBefore = run(false);
    expect(asBefore.result.failedSteps).toBeGreaterThan(0);
    expect(asBefore.result.hitMinStep).toBe(false);
    expect(asBefore.brightest).toBeGreaterThan(1e-3);

    const now = run(true);
    expect(now.samples).toEqual(asBefore.samples);
    expect(now.counts.supplyLow).toBeGreaterThan(0);
    expect(now.counts.accepted).toBe(0);
  });
});

describe("a chip powering up on a capacitor-held rail", () => {
  it("turns on where the HeadlessRunner would have stopped at its floor", () => {
    // Run in 0.5 ms pieces, the HeadlessRunner walks the rail up to the
    // chip's threshold from below, and every step that would carry it
    // through fails: switched on, the chip drops the rail by more than one
    // doubling of the step can climb. Those steps used to shrink to the
    // 10 ns floor and stop the clock 5.98 ms in. A failing step at the floor
    // now holds the chip off for its 10 ns, and the rail goes through the
    // threshold. The chip then stays off until the rail has climbed its own
    // drop and can carry it, about 12 us here.
    const runner = new HeadlessRunner();
    const engine = runner.engine;
    const floorHolds: Array<{ h: number; before: number; after: number }> = [];
    const step = engine.step.bind(engine);
    engine.step = (h: number) => {
      const held = holdOffCounts(engine).heldAtFloor;
      const before = inverterSupply(engine, engine.getNetV());
      step(h);
      if (holdOffCounts(engine).heldAtFloor > held) {
        floorHolds.push({ h, before, after: inverterSupply(engine, engine.getNetV()) });
      }
    };
    runner.load(coldPowerUp());
    let crossedAt = Number.NaN;
    let litAt = Number.NaN;
    for (let piece = 0; piece < 24; piece++) {
      const run = runner.run({
        durationS: 5e-4,
        onSample: (sample) => {
          if (Number.isNaN(crossedAt) && inverterSupply(engine, sample.netV) >= INVERTER_POWER_ON_V) crossedAt = sample.simTime;
          if (Number.isNaN(litAt) && (sample.elementI.led0 ?? 0) > 1e-3) litAt = sample.simTime;
        },
      });
      expect(run.hitMinStep).toBe(false);
      expect(run.simulatedS).toBeCloseTo(5e-4, 12);
    }
    // Only floor steps hold a chip whose supply is still low, and only the
    // step that carries its supply through the threshold.
    expect(floorHolds.length).toBeGreaterThan(0);
    for (const hold of floorHolds) {
      expect(hold.h).toBeLessThanOrEqual(1e-8);
      expect(hold.before).toBeLessThan(INVERTER_POWER_ON_V);
      expect(hold.after).toBeGreaterThanOrEqual(INVERTER_POWER_ON_V);
    }
    // Lit within 20 us of the rail first reaching the threshold.
    expect(litAt - crossedAt).toBeGreaterThanOrEqual(0);
    expect(litAt - crossedAt).toBeLessThan(20e-6);
  });

  it("is landed by a shorter step as before when the worker's loop steps over it", () => {
    // The worker's loop rejects the step that would carry the rail through
    // the threshold once, then lands it with the chip on in the very step
    // that crosses. The floor never comes into it: every sample is the same
    // with the fallback switched off.
    const run = (withFallback: boolean) => {
      const host = new WorkerStepLoop();
      const engine = host.engine;
      if (!withFallback) disableFallback(engine);
      const samples: number[][] = [];
      let crossed = -1;
      let lit = -1;
      host.onAccepted = () => {
        const netV = engine.getNetV();
        const elementI = engine.getElementI();
        if (crossed < 0 && inverterSupply(engine, netV) >= INVERTER_POWER_ON_V) crossed = samples.length;
        if (lit < 0 && (elementI.led0 ?? 0) > 1e-3) lit = samples.length;
        samples.push([engine.simTime, ...Object.values(netV), ...Object.values(elementI)]);
      };
      host.load(coldPowerUp());
      const batch = host.advance(BATCH_S);
      return { batch, samples, crossed, lit, counts: holdOffCounts(engine) };
    };
    const asBefore = run(false);
    const now = run(true);
    expect(now.samples).toEqual(asBefore.samples);
    expect(now.batch.advancedS).toBeCloseTo(BATCH_S, 12);
    expect(now.batch.failedSteps).toBeGreaterThan(0);
    expect(now.counts.supplyLow).toBeGreaterThan(0);
    expect(now.counts.heldAtFloor).toBe(0);
    expect(now.crossed).toBeGreaterThan(0);
    expect(now.lit).toBe(now.crossed);
  });

  it("is landed as before when a 555's split starts a sub-step just past the threshold", () => {
    // The worker's step that carries the twin timers' rail through their
    // threshold converges, and their latches flip, so the engine re-solves
    // it split at the flip. The split's first part ends just above the
    // threshold, and the next sub-step, with the discharge transistors now
    // on, alternates. At that sub-step's own start the supply looks fine
    // with the chips off, but it was still low when the host's step began:
    // the crossing is the host's to land with a shorter step, and it does.
    const run = (withFallback: boolean) => {
      const host = new WorkerStepLoop();
      const engine = host.engine;
      if (!withFallback) disableFallback(engine);
      const internal = engine as unknown as { _solveHoldingUnholdableOff: (...args: unknown[]) => Float64Array | null };
      let stepStart = Number.NaN;
      const step = engine.step.bind(engine);
      engine.step = (h: number) => {
        stepStart = engine.simTime;
        step(h);
      };
      // Fallback calls from a solve that starts inside the host's step.
      let insideSplit = 0;
      const fallback = internal._solveHoldingUnholdableOff.bind(engine);
      internal._solveHoldingUnholdableOff = (...args: unknown[]) => {
        if (engine.simTime > stepStart) insideSplit += 1;
        return fallback(...args);
      };
      const samples: number[][] = [];
      host.onAccepted = () => {
        samples.push([engine.simTime, ...Object.values(engine.getNetV()), ...Object.values(engine.getElementI())]);
      };
      host.load(twinTimerPowerUp());
      const batch = host.advance(BATCH_S);
      return { batch, samples, insideSplit, counts: holdOffCounts(engine) };
    };
    const asBefore = run(false);
    const now = run(true);
    expect(now.samples).toEqual(asBefore.samples);
    expect(now.batch.advancedS).toBeCloseTo(BATCH_S, 12);
    expect(now.insideSplit).toBeGreaterThan(0);
    expect(now.counts.supplyLow).toBeGreaterThan(0);
    expect(now.counts.accepted).toBe(0);
  });
});

describe("the held-off fallback", () => {
  type Internal = {
    _runNewton: (h: number, xInit: Float64Array) => { converged: boolean; x: Float64Array; iters: number };
    _icPowerInfo: (comp: unknown, x: Float64Array) => { powered: boolean };
    _componentById: Map<string, unknown>;
    _solveHoldingUnholdableOff: (...args: unknown[]) => Float64Array | null;
    _heldPower: unknown;
    _powerReadings: unknown;
  };
  const internals = (engine: SimEngine) => engine as unknown as Internal;
  /** The engine's state without its solver diagnostics, which a failed solve updates by design. */
  const stateOf = (engine: SimEngine) => ({ ...engine.saveState(), solverDiagnostics: undefined });

  it("leaves a failed solve alone unless a power reading changed at least twice", () => {
    // A chip starving itself reads powered and unpowered by turns. A failed
    // solve in which a reading changed once (a chip switching for good) or
    // not at all failed for some other reason, and is rejected exactly as
    // before for the host to shorten. Newton is made to fail here after a
    // running blinker's solve, with that many changes added to its timer's
    // reading: no natural circuit fails that way on demand.
    for (const changes of [0, 1]) {
      const runner = new HeadlessRunner();
      runner.load(blinker("switched off in the negative lead", true));
      runner.run({ durationS: 0.3 });
      const engine = runner.engine;
      const internal = internals(engine);
      const runNewton = internal._runNewton.bind(engine);
      let tampered = false;
      internal._runNewton = (h, xInit) => {
        const result = runNewton(h, xInit);
        if (tampered) return result;
        tampered = true;
        const timer = internal._componentById.get("timer");
        // An all-zero solution leaves the timer with no supply: one change.
        for (let change = 0; change < changes; change++) internal._icPowerInfo(timer, new Float64Array(xInit.length));
        return { ...result, converged: false };
      };
      const fallback = internal._solveHoldingUnholdableOff.bind(engine);
      let consulted = 0;
      internal._solveHoldingUnholdableOff = (...args: unknown[]) => {
        consulted += 1;
        return fallback(...args);
      };
      const counts = holdOffCounts(engine);
      const before = stateOf(engine);
      engine.step(1e-4);
      expect(engine.lastConverged).toBe(false);
      expect(consulted).toBe(0);
      expect(holdOffCounts(engine)).toEqual(counts);
      expect(stateOf(engine)).toEqual(before);
    }
  });

  it("releases its hold with the step that used it", () => {
    const runner = new HeadlessRunner();
    const engine = runner.engine;
    const internal = internals(engine);
    runner.load(blinker("switched off in the negative lead", true));
    runner.run({ durationS: 0.6 });
    runner.load(blinker("switched off in the negative lead", false));
    expect(internal._heldPower).toBeNull();
    expect(internal._powerReadings).toBeNull();
    const accepted = holdOffCounts(engine).accepted;
    for (let step = 0; step < 20; step++) {
      engine.step(1e-4);
      expect(internal._heldPower).toBeNull();
      expect(internal._powerReadings).toBeNull();
    }
    expect(holdOffCounts(engine).accepted).toBeGreaterThan(accepted);

    // Switched back on, the timer's own supply decides how it reads in the
    // very next step: the hold belonged to the step that used it.
    runner.load(blinker("switched off in the negative lead", true));
    const icPowerInfo = internal._icPowerInfo.bind(engine);
    const timer = internal._componentById.get("timer");
    const readings: boolean[] = [];
    internal._icPowerInfo = (comp, x) => {
      const info = icPowerInfo(comp, x);
      if (comp === timer) readings.push(info.powered);
      return info;
    };
    engine.step(1e-6);
    expect(engine.lastConverged).toBe(true);
    expect(readings.at(-1)).toBe(true);
  });

  it("is never armed in an operating-point solve", () => {
    // An operating-point analysis reports a point it cannot find instead of
    // holding a part off to reach one, exactly as before. Its steps hold the
    // starved timer off, and the clock runs.
    const runner = new HeadlessRunner();
    runner.load(starvedTimer());
    const counts = holdOffCounts(runner.engine);
    expect(runner.engine.dcOperatingPoint().converged).toBe(false);
    expect(holdOffCounts(runner.engine)).toEqual(counts);

    const run = runner.run({ durationS: 0.01 });
    expect(run.simulatedS).toBeCloseTo(0.01, 9);
    expect(holdOffCounts(runner.engine).accepted).toBeGreaterThan(counts.accepted);
  });
});
