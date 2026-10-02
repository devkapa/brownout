/**
 * bench_psu slightly overloaded with a capacitor across its rail: the case
 * that needs both the rail-capacitor CC entry and the setpoint CC exit.
 *
 * Settled in CV at 5 V on 100 uF, the load steps to 49.9 ohm, which draws
 * 5 / 49.9 = 0.1002 A at the setpoint, 0.2% over the 0.1 A limit. Its CC rail
 * settles at 0.1 * 49.9 = 4.99 V, above 0.98 |V|. Without the rail-cap fix
 * the capacitor held the CC candidate inside the compliance band at fine
 * steps, so the supply stayed at 5 V delivering 0.1002 A. Without the exit
 * fix the 98% exit flipped reg on every step once the supply was in CC, and
 * the adaptive runner rejected every other step. With both it delivers the
 * limit from the first step and reads CC throughout.
 */

import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const I_LIMIT = 0.1;
const C_RAIL = 100e-6;
const R_SLIGHT = 49.9;

function railCap(voltage: number, resistance: number): SimCircuit {
  return {
    components: [
      { id: "psu", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage, iLimit: I_LIMIT } },
      { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: C_RAIL } },
      { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
    ],
    wires: [
      { from_component: "psu", from_pin: "pos", to_component: "c", to_pin: "a" },
      { from_component: "c", from_pin: "b", to_component: "psu", to_pin: "neg" },
      { from_component: "psu", from_pin: "pos", to_component: "rl", to_pin: "a" },
      { from_component: "rl", from_pin: "b", to_component: "psu", to_pin: "neg" },
    ],
  };
}

function nodeVolts(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.getNetIdForPin(componentId, pinId);
  if (net === undefined) throw new Error(`no net for ${componentId}.${pinId}`);
  return engine.getNetV()[net] ?? 0;
}

const rail = (engine: SimEngine) => nodeVolts(engine, "psu", "pos") - nodeVolts(engine, "psu", "neg");
const reg = (engine: SimEngine) => engine.getIcState("psu")?.reg;
const psuCurrent = (engine: SimEngine) => engine.getElementI().psu ?? Number.NaN;

/** Exact CC rail decay from the setpoint toward iLimit * R with tau = R C. */
function ccRail(voltage: number, t: number): number {
  const settled = I_LIMIT * R_SLIGHT;
  return Math.sign(voltage) * (settled + (Math.abs(voltage) - settled) * Math.exp(-t / (R_SLIGHT * C_RAIL)));
}

describe.each([
  { voltage: 5, dir: 1 },
  { voltage: -5, dir: -1 },
])("bench_psu 0.2% over its limit with a rail capacitor ($voltage V)", ({ voltage, dir }) => {
  it.each([1e-8, 1e-6, 1e-5])("delivers the limit and reads CC on every %s s fixed step", (h) => {
    const engine = new SimEngine();
    engine.load(railCap(voltage, 100));
    for (let k = 0; k < 2000; k++) engine.step(1e-4);
    expect(reg(engine)).toBe(0);

    engine.load(railCap(voltage, R_SLIGHT));
    const steps = 2000;
    const off: Array<{ k: number; reg: number | undefined; i: number }> = [];
    for (let k = 0; k < steps; k++) {
      engine.step(h);
      const i = psuCurrent(engine);
      if (reg(engine) !== 2 || Math.abs(i + dir * I_LIMIT) > 1e-9) off.push({ k, reg: reg(engine), i });
    }

    expect(off.length).toBe(0);
    // tau = 4.99 ms; at h / tau <= 2e-3 backward Euler tracks the exact decay to 1e-5 V.
    expect(rail(engine)).toBeCloseTo(ccRail(voltage, steps * h), 5);
  });

  it("delivers the limit and reads CC on every headless step, without rejected steps", () => {
    const runner = new HeadlessRunner();
    runner.load(railCap(voltage, 100));
    runner.run({ durationS: 0.2 });
    expect(reg(runner.engine)).toBe(0);

    runner.load(railCap(voltage, R_SLIGHT));
    const off: Array<{ t: number; reg: number | undefined; i: number }> = [];
    const result = runner.run({
      durationS: 0.05,
      maxSteps: 10_000,
      onSample: (s) => {
        const i = psuCurrent(runner.engine);
        if (reg(runner.engine) !== 2 || Math.abs(i + dir * I_LIMIT) > 1e-9) off.push({ t: s.simTime, reg: reg(runner.engine), i });
      },
    });

    expect(off.length).toBe(0);
    expect(result.rejectedSteps).toBe(0);
    // A linear RC decay from 10 ns to the 10 ms ceiling: about 20 growth
    // steps, a few to follow tau = 5 ms, then 10 ms steps.
    expect(result.acceptedSteps).toBeLessThanOrEqual(100);
    // 50 ms is 10 tau: within 5e-5 * 0.01 V of the settled 4.99 V.
    expect(rail(runner.engine)).toBeCloseTo(dir * I_LIMIT * R_SLIGHT, 5);
  });
});
