/**
 * Step breakpoints: both sides of an NE555 switch inside a step() interval.
 *
 * The engine splits a step at the 555's threshold crossing, but a consumer
 * that only sees accepted states (before and after step()) interpolates
 * straight across the whole step. On a scope at 10 kSa/s that draws the 555
 * output edge as a ramp centred on the step, up to half a step from where
 * the switch happened: on a 5 V astable with 2-10 ms steps, periods that
 * looked ±3% off cycle to cycle while the simulation's own switching held
 * steady. With `captureStepBreakpoints` on, the engine records the state at
 * the switch and, after a guard sub-step, the state just after it.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type SimComponent = SimCircuit["components"][number];

const part = (id: string, kind: string, pins: string[], params: Record<string, number> = {}): SimComponent => ({
  id,
  kind,
  pins: pins.map((pin) => ({ id: pin })),
  params,
});
const wire = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

// Textbook astable: RA 4.7 kohm, RB 10 kohm, C 10 uF, 5 V. Period ≈ 171 ms.
const ASTABLE: SimCircuit = {
  components: [
    part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 }),
    part("ra", "resistor", ["a", "b"], { resistance: 4_700 }),
    part("rb", "resistor", ["a", "b"], { resistance: 10_000 }),
    part("ct", "capacitor", ["a", "b"], { capacitance: 10e-6 }),
    part("load", "resistor", ["a", "b"], { resistance: 1_000 }),
    part("timer", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]),
  ],
  wires: [
    wire("vdd", "pos", "timer", "8"),
    wire("vdd", "pos", "timer", "4"),
    wire("vdd", "neg", "timer", "1"),
    wire("vdd", "pos", "ra", "a"),
    wire("ra", "b", "timer", "7"),
    wire("timer", "7", "rb", "a"),
    wire("rb", "b", "timer", "6"),
    wire("timer", "6", "timer", "2"),
    wire("timer", "6", "ct", "a"),
    wire("ct", "b", "vdd", "neg"),
    wire("timer", "3", "load", "a"),
    wire("load", "b", "vdd", "neg"),
  ],
};

const H = 2e-3;

function engine(capture: boolean): SimEngine {
  const e = new SimEngine();
  e.captureStepBreakpoints = capture;
  e.load(ASTABLE);
  return e;
}

function net(e: SimEngine, component: string, pin: string): string {
  const id = e.getNetIdForPin(component, pin);
  if (!id) throw new Error(`no net for ${component}.${pin}`);
  return id;
}

describe("SimEngine step breakpoints", () => {
  it("records the state on both sides of each 555 switch, inside the step that switched", () => {
    const e = engine(true);
    const out = net(e, "timer", "3");
    const cap = net(e, "timer", "6");
    let pairs = 0;
    for (let i = 0; i < 300; i++) {
      const t0 = e.simTime;
      e.step(H);
      expect(e.lastConverged).toBe(true);
      const taken = e.takeStepBreakpoints();
      expect(taken.length % 2).toBe(0);
      for (let k = 0; k < taken.length; k += 2) {
        const [at, after] = [taken[k]!, taken[k + 1]!];
        // A crossing in the last 2% of the step switches one guard short of its end.
        expect(at.time).toBeGreaterThan(t0);
        expect(after.time).toBeLessThanOrEqual(e.simTime);
        expect(after.time - at.time).toBeCloseTo(1e-8, 15);
        // The output jumps across the guard; the timing cap does not move.
        expect(Math.abs(after.netV[out]! - at.netV[out]!)).toBeGreaterThan(3);
        expect(Math.abs(after.netV[cap]! - at.netV[cap]!)).toBeLessThan(1e-3);
        // The switch sits on a comparator threshold, 1/3 or 2/3 of 5 V.
        const vCap = at.netV[cap]!;
        expect(Math.min(Math.abs(vCap - 5 / 3), Math.abs(vCap - 10 / 3))).toBeLessThan(5e-3);
        pairs += 1;
      }
    }
    // 0.6 s of a ~171 ms astable: at least six switches.
    expect(pairs).toBeGreaterThanOrEqual(6);
  });

  it("records nothing unless asked", () => {
    const e = engine(false);
    for (let i = 0; i < 300; i++) e.step(H);
    expect(e.takeStepBreakpoints()).toEqual([]);
  });

  it("drops the breakpoints of a rolled-back trial", () => {
    // Find a step that switches, then replay up to it and roll that step back.
    const probe = engine(true);
    let switchingStep = -1;
    for (let i = 0; i < 300 && switchingStep < 0; i++) {
      probe.step(H);
      if (probe.takeStepBreakpoints().length > 0) switchingStep = i;
    }
    expect(switchingStep).toBeGreaterThan(0);

    const e = engine(true);
    for (let i = 0; i < switchingStep; i++) e.step(H);
    e.takeStepBreakpoints();
    const before = e.saveState();
    e.step(H);
    e.restoreState(before);
    expect(e.takeStepBreakpoints()).toEqual([]);
    e.step(H);
    expect(e.takeStepBreakpoints()).toHaveLength(2);
  });

  it("changes the trajectory only by the guard sub-step", () => {
    const withCapture = engine(true);
    const without = engine(false);
    for (let i = 0; i < 300; i++) {
      withCapture.step(H);
      withCapture.takeStepBreakpoints();
      without.step(H);
    }
    const cap = net(without, "timer", "6");
    expect(Math.abs(withCapture.getNetV()[cap]! - without.getNetV()[cap]!)).toBeLessThan(1e-3);
    expect(withCapture.digitalState.timer).toBe(without.digitalState.timer);
  });
});
