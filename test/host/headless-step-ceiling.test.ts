/**
 * HeadlessRunner's step ceiling: 2 ms while the circuit is changing, 10 ms
 * while it sits still.
 *
 * Backward Euler lags an RC curve by about half a step, and the local error
 * tolerance alone let slow RC timing run at 10 ms steps, so a 555 astable ran
 * 1.5% slow and its period wandered with the step history. A flat 2 ms
 * ceiling fixed that but cost static logic boards 3-4x the solves, so the
 * ceiling applies only while the last error estimate says a 10 ms step would
 * spend a real share of the tolerance.
 */
import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../src/host/headless.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

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

// A 1 ms RC that has fully settled long before the window measured below.
const SETTLING_RC: SimCircuit = {
  components: [
    part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 }),
    part("r", "resistor", ["a", "b"], { resistance: 1_000 }),
    part("c", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
  ],
  wires: [
    wire("vdd", "pos", "r", "a"),
    wire("r", "b", "c", "a"),
    wire("c", "b", "vdd", "neg"),
  ],
};

/** Accepted step sizes of a run, from `from` seconds on. */
function steps(circuit: SimCircuit, durationS: number, from: number, options: { adaptive?: boolean; fixedStepS?: number } = {}): number[] {
  const runner = new HeadlessRunner();
  runner.load(circuit);
  const times: number[] = [0];
  runner.run({ durationS, ...options, onSample: (s) => times.push(s.simTime) });
  return times.slice(1).map((t, i) => [t, t - times[i]!] as const).filter(([t]) => t > from).map(([, h]) => h);
}

describe("HeadlessRunner step ceiling", () => {
  it("steps at 10 ms once the circuit sits still", () => {
    const h = steps(SETTLING_RC, 1, 0.5);
    expect(Math.max(...h)).toBeCloseTo(1e-2, 9);
  });

  it("never steps over 2 ms while a 555 astable runs", () => {
    const h = steps(ASTABLE, 1, 0.05);
    expect(h.length).toBeGreaterThan(400);
    expect(Math.max(...h)).toBeLessThanOrEqual(2e-3 * (1 + 1e-9));
  });

  it("honours a fixed step above the changing-circuit ceiling", () => {
    const h = steps(ASTABLE, 0.2, 0, { adaptive: false, fixedStepS: 5e-3 });
    expect(Math.max(...h)).toBeCloseTo(5e-3, 12);
  });
});
