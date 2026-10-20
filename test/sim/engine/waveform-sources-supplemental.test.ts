/**
 * Supplemental classic cases for the W1f waveform sources (clock_gen,
 * pulse_source, pulse_gen).
 *
 * WHY THESE EXIST. No classic suite builds a circuit with a clock_gen, a
 * pulse_source or a pulse_gen from implemented kinds alone: digital-pulse.test.ts
 * and waveform.test.ts test classic's waveform functions with no engine, and
 * the suites that do build these sources pair them with a buzzer, a 74HC595 or
 * a transistor. Gate 1 needs one portable case per implemented kind, so these
 * read each source's level through a resistor divider, node voltages and
 * source currents only (`nets`/`getNetV`/`getElementI` and `step()`), which
 * runs on classic and on the core alike.
 *
 * WHERE THE NUMBERS COME FROM. The level is the source's definition, classic's
 * `clockVoltage`/`pulseVoltage` (src/sim/engine/digital.ts; core/README.md,
 * Sources points 2 and 3), evaluated by the test at an instant away from any
 * edge, then halved by a 10 k / 10 k divider. Every instant sits inside a flat
 * segment or, for the 1 ms ramp (resolved, Sources point 3), in the middle of
 * the ramp, where the level is linear in time. The divider is static, so the
 * answer at an instant depends on the level there and nothing else.
 */
import { describe, expect, it } from "vitest";
import { clockVoltage, pulseVoltage } from "../../../src/sim/engine/digital.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

/** 1 pV and 1 fA: both engines solve the divider directly, so this leaves room only for rounding. */
const VOLTS_TOLERANCE = 1e-12;
const AMPS_TOLERANCE = 1e-15;

function divider(source: SimCircuit["components"][number], outPin: string, gndPin: string): SimCircuit {
  return {
    components: [
      source,
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10_000 } },
      { id: "r2", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10_000 } },
    ],
    wires: [
      { from_component: source.id, from_pin: outPin, to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "r2", to_pin: "a" },
      { from_component: "r2", from_pin: "b", to_component: source.id, to_pin: gndPin },
    ],
  };
}

function netOf(engine: SimEngine, componentId: string, pinId: string): string {
  const net = engine.nets.find((n) => n.pins.some(([id, pin]) => id === componentId && pin === pinId));
  if (!net) throw new Error(`no net for ${componentId}.${pinId}`);
  return net.id;
}

/** Steps to each instant in 10 us steps and reads the divider's midpoint and the source current there. */
function readAt(circuit: SimCircuit, sourceId: string, gndPin: string, instants: readonly number[]): { v: number; i: number }[] {
  const engine = new SimEngine();
  engine.load(circuit);
  const mid = netOf(engine, "r1", "b");
  const gnd = netOf(engine, sourceId, gndPin);
  const readings: { v: number; i: number }[] = [];
  for (const t of instants) {
    while (t - engine.simTime > 1e-12) engine.step(Math.min(1e-5, t - engine.simTime));
    const netV = engine.getNetV();
    readings.push({ v: (netV[mid] ?? Number.NaN) - (netV[gnd] ?? Number.NaN), i: engine.getElementI()[sourceId] ?? Number.NaN });
  }
  return readings;
}

/**
 * Every free node carries the 1e-12 S shunt to the datum and every node's shunt
 * enters its source's current (core/README.md, W0 point 2), so the midpoint is
 * `level (1/R) / (2/R + g)` and the source carries `-((level - v)/R + g level)`.
 */
const SHUNT_S = 1e-12;
const R = 10_000;

function expectDivided(readings: readonly { v: number; i: number }[], levels: readonly number[]): void {
  readings.forEach((r, k) => {
    const level = levels[k]!;
    const v = (level / R) / (2 / R + SHUNT_S);
    expect(Math.abs(r.v - v)).toBeLessThan(VOLTS_TOLERANCE);
    expect(Math.abs(r.i + ((level - v) / R + SHUNT_S * level))).toBeLessThan(AMPS_TOLERANCE);
  });
}

describe("gate 1 supplemental — clock_gen", () => {
  it("a 1 kHz, 50 % clock reads vHigh in each first half-period and vLow in each second", () => {
    const clock = { id: "clk", kind: "clock_gen", pins: [{ id: "out" }, { id: "gnd" }], params: { frequency: 1000, duty: 0.5 } };
    const instants = [0.25e-3, 0.75e-3, 1.25e-3, 1.75e-3, 3.4e-3];
    const levels = instants.map((t) => clockVoltage(t, 1e-3, 0.5, 5, 0, 0));
    expect(levels).toEqual([5, 0, 5, 0, 5]);
    expectDivided(readAt(divider(clock, "out", "gnd"), "clk", "gnd", instants), levels);
  });
});

describe("gate 1 supplemental — pulse_gen", () => {
  it("the catalog pulse (1 us ramps, 0.5 ms wide, 1 ms period) reads v2 on its top and v1 between pulses", () => {
    const params = { v1: 0, v2: 5, td: 0, tr: 1e-6, tf: 1e-6, pw: 5e-4, per: 1e-3 };
    const pulse = { id: "pg", kind: "pulse_gen", catalogUid: "pulse-source", pins: [{ id: "pos" }, { id: "neg" }], params };
    const instants = [0.25e-3, 0.75e-3, 1.25e-3, 2.8e-3];
    const levels = instants.map((t) => pulseVoltage(t, params.v1, params.v2, params.td, params.tr, params.tf, params.pw, params.per));
    expect(levels).toEqual([5, 0, 5, 0]);
    expectDivided(readAt(divider(pulse, "pos", "neg"), "pg", "neg", instants), levels);
  });
});

describe("gate 1 supplemental — pulse_source", () => {
  it("a 1 ms ramp on a 4 ms top is followed point by point through the rise and the fall (Sources point 3, resolved)", () => {
    const params = { v1: 0, v2: 5, td: 0, tr: 1e-3, tf: 1e-3, pw: 4e-3, per: 1e-2 };
    const pulse = { id: "ps", kind: "pulse_source", pins: [{ id: "pos" }, { id: "neg" }], params };
    const instants = [0.2e-3, 0.5e-3, 0.73e-3, 3e-3, 5.4e-3, 5.9e-3, 8e-3];
    const levels = instants.map((t) => pulseVoltage(t, params.v1, params.v2, params.td, params.tr, params.tf, params.pw, params.per));
    expectDivided(readAt(divider(pulse, "pos", "neg"), "ps", "neg", instants), levels);
  });

  it("a one-shot (per = 0) returns to v1 after its fall and stays there", () => {
    const params = { v1: 1, v2: 4, td: 1e-3, tr: 10e-6, tf: 20e-6, pw: 100e-6, per: 0 };
    const pulse = { id: "ps", kind: "pulse_source", pins: [{ id: "pos" }, { id: "neg" }], params };
    const instants = [0.5e-3, 1.06e-3, 2e-3, 5e-3];
    const levels = instants.map((t) => pulseVoltage(t, params.v1, params.v2, params.td, params.tr, params.tf, params.pw, params.per));
    expect(levels).toEqual([1, 4, 1, 1]);
    expectDivided(readAt(divider(pulse, "pos", "neg"), "ps", "neg", instants), levels);
  });
});
