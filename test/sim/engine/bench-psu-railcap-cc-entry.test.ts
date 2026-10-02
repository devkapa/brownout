/**
 * bench_psu with a capacitor across its rail enters CC when its load steps
 * over the limit.
 *
 * Inside a solve, a CC candidate that comes back at the setpoint hands the
 * supply to CV: that is how a committed CC supply whose load is removed holds
 * its rail instead of pushing the limit current into nothing for a step. The
 * test accepted any candidate within 1e-9 + 1e-6 |V| of the setpoint, and a
 * rail capacitor keeps an overloaded supply's candidate inside that band: at
 * a step h it falls only (excess current) * h / C below the setpoint. Settled
 * in CV on 100 uF and stepped to a 20% overload, the supply sat at exactly
 * 5 V delivering 0.12 A through its 0.1 A limit under the adaptive runner,
 * with reg flipping and the step pinned at 10-15 ns, forever; at 10 us fixed
 * steps the same circuit entered CC. CV now takes over only once the
 * candidate reaches the setpoint itself. For a load whose draw rises with
 * voltage, a candidate under the setpoint means the draw at the setpoint is
 * over the limit, so CC is the right branch.
 *
 * Supply 5 V, 0.1 A, 100 uF on the rail. 41.67 ohm draws 5 / 41.67 = 0.12 A
 * at 5 V; its CC rail decays from 5 V toward 0.1 * 41.67 = 4.1667 V with
 * tau = R C = 4.17 ms. The overload cases run at +5 V and mirrored at -5 V.
 */

import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const V = 5;
const I_LIMIT = 0.1;
const C_RAIL = 100e-6;
const R_OVERLOAD = V / 0.12;

function psu(voltage: number) {
  return { id: "psu", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage, iLimit: I_LIMIT } };
}

/** Capacitor straight across the supply; a load resistor across the rail when resistance > 0. */
function railCap(voltage: number, resistance: number): SimCircuit {
  return {
    components: [
      psu(voltage),
      { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: C_RAIL } },
      ...(resistance > 0
        ? [{ id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } }]
        : []),
    ],
    wires: [
      { from_component: "psu", from_pin: "pos", to_component: "c", to_pin: "a" },
      { from_component: "c", from_pin: "b", to_component: "psu", to_pin: "neg" },
      ...(resistance > 0
        ? [
          { from_component: "psu", from_pin: "pos", to_component: "rl", to_pin: "a" },
          { from_component: "rl", from_pin: "b", to_component: "psu", to_pin: "neg" },
        ]
        : []),
    ],
  };
}

/** Supply -> series resistor -> (capacitor || load): the series resistor bounds the inrush. */
function seriesFed(seriesResistance: number, load: number): SimCircuit {
  return {
    components: [
      psu(V),
      { id: "rs", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: seriesResistance } },
      { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: C_RAIL } },
      { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: load } },
    ],
    wires: [
      { from_component: "psu", from_pin: "pos", to_component: "rs", to_pin: "a" },
      { from_component: "rs", from_pin: "b", to_component: "c", to_pin: "a" },
      { from_component: "c", from_pin: "b", to_component: "psu", to_pin: "neg" },
      { from_component: "rs", from_pin: "b", to_component: "rl", to_pin: "a" },
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
function ccRail(t: number): number {
  const settled = I_LIMIT * R_OVERLOAD;
  return settled + (V - settled) * Math.exp(-t / (R_OVERLOAD * C_RAIL));
}

/** Settle the supply in CV on 100 ohm (0.05 A) with the rail capacitor charged. */
function settledInCv(voltage: number): SimEngine {
  const engine = new SimEngine();
  engine.load(railCap(voltage, 100));
  for (let k = 0; k < 2000; k++) engine.step(1e-4);
  expect(reg(engine)).toBe(0);
  expect(rail(engine)).toBeCloseTo(voltage, 9);
  return engine;
}

describe.each([
  { voltage: 5, dir: 1 },
  { voltage: -5, dir: -1 },
])("bench_psu with a rail capacitor enters CC on an overload ($voltage V)", ({ voltage, dir }) => {
  it.each([1e-8, 1e-5])("delivers the limit from the first step after a 20%% overload at %s s fixed steps", (h) => {
    const engine = settledInCv(voltage);
    engine.load(railCap(voltage, R_OVERLOAD));
    const steps = 2000;
    let overLimit = 0;
    let notCc = 0;
    for (let k = 0; k < steps; k++) {
      engine.step(h);
      if (Math.abs(psuCurrent(engine) + dir * I_LIMIT) > 1e-9) overLimit += 1;
      if (reg(engine) !== 2) notCc += 1;
    }

    // At 10 ns the excess 0.02 A moves the rail 2 uV per step, inside the
    // old band: before the fix the rail stayed at exactly 5 V and the
    // supply delivered 0.12 A on every step.
    expect(overLimit).toBe(0);
    expect(notCc).toBe(0);
    // Backward Euler at h / tau <= 2.4e-3 tracks the exact decay to 1e-3 V.
    expect(rail(engine)).toBeCloseTo(dir * ccRail(steps * h), 3);
  });

  it("delivers the limit under the headless runner after a 20% overload", () => {
    const runner = new HeadlessRunner();
    runner.load(railCap(voltage, 100));
    runner.run({ durationS: 0.2 });
    expect(reg(runner.engine)).toBe(0);

    runner.load(railCap(voltage, R_OVERLOAD));
    let overLimit = 0;
    let notCc = 0;
    // The cap bounds the cost this test accepts: before the fix every
    // sample read 0.12 A at exactly 5 V with the step pinned at 10-15 ns.
    const result = runner.run({
      durationS: 0.05,
      maxSteps: 10_000,
      onSample: () => {
        if (Math.abs(psuCurrent(runner.engine) + dir * I_LIMIT) > 1e-9) overLimit += 1;
        if (reg(runner.engine) !== 2) notCc += 1;
      },
    });

    expect(overLimit).toBe(0);
    expect(notCc).toBe(0);
    expect(result.rejectedSteps).toBe(0);
    // An RC decay from 10 ns to the 10 ms ceiling: about 20 growth steps,
    // a few to follow tau = 4.17 ms, then 10 ms steps.
    expect(result.acceptedSteps).toBeLessThanOrEqual(100);
    expect(result.hitStepCap).toBe(false);
    expect(reg(runner.engine)).toBe(2);
    // 50 ms is 12 tau: the rail has settled at iLimit * R.
    expect(rail(runner.engine)).toBeCloseTo(dir * I_LIMIT * R_OVERLOAD, 4);
  });
});

describe("bench_psu with a rail capacitor stays in CV within its limit", () => {
  it.each([
    { label: "0.05 A to 0.09 A", resistance: V / 0.09 },
    { label: "0.05 A to 0.0999 A", resistance: V / 0.0999 },
  ])("never enters CC when a settled load steps $label (fixed steps)", ({ resistance }) => {
    for (const h of [1e-8, 1e-6, 1e-5]) {
      const engine = settledInCv(V);
      engine.load(railCap(V, resistance));
      const regs = new Set<number | undefined>();
      for (let k = 0; k < 2000; k++) {
        engine.step(h);
        regs.add(reg(engine));
      }
      expect([...regs]).toEqual([0]);
      expect(rail(engine)).toBeCloseTo(V, 9);
      expect(psuCurrent(engine)).toBeCloseTo(-V / resistance, 9);
    }
  });

  it.each([
    { label: "0.05 A to 0.09 A", resistance: V / 0.09 },
    { label: "0.05 A to 0.0999 A", resistance: V / 0.0999 },
  ])("never enters CC when a settled load steps $label (headless runner)", ({ resistance }) => {
    const runner = new HeadlessRunner();
    runner.load(railCap(V, 100));
    runner.run({ durationS: 0.2 });
    runner.load(railCap(V, resistance));
    const regs = new Set<number | undefined>();
    runner.run({ durationS: 0.05, onSample: () => regs.add(reg(runner.engine)) });

    expect([...regs]).toEqual([0]);
    expect(rail(runner.engine)).toBeCloseTo(V, 9);
    expect(psuCurrent(runner.engine)).toBeCloseTo(-V / resistance, 9);
  });

  it("never enters CC on a cold start whose inrush stays under the limit", () => {
    // 100 ohm in series with 100 uF || 1 kohm: the inrush peaks at 5 / 100 = 0.05 A.
    for (const h of [1e-8, 1e-5]) {
      const engine = new SimEngine();
      engine.load(seriesFed(100, 1000));
      const regs = new Set<number | undefined>();
      let peak = 0;
      for (let k = 0; k < 5000; k++) {
        engine.step(h);
        regs.add(reg(engine));
        peak = Math.max(peak, Math.abs(psuCurrent(engine)));
      }
      expect([...regs]).toEqual([0]);
      expect(peak).toBeLessThanOrEqual(0.05 + 1e-9);
    }

    const runner = new HeadlessRunner();
    runner.load(seriesFed(100, 1000));
    const regs = new Set<number | undefined>();
    runner.run({ durationS: 0.1, onSample: () => regs.add(reg(runner.engine)) });
    expect([...regs]).toEqual([0]);
    // 1 kohm across the capacitor: the supply settles at 5 / 1100 A (the
    // runner's 10 ms steps still lag the last 2 uA of the charge at 0.1 s).
    expect(psuCurrent(runner.engine)).toBeCloseTo(-V / 1100, 4);
  });

  it("enters CC on a cold start whose inrush exceeds the limit and returns to CV once charged", () => {
    // 100 uF straight across the supply with 100 ohm: CC charges the rail as
    // u = 10 (1 - exp(-t / 10 ms)), reaching 5 V at 10 ms * ln 2 = 6.93 ms;
    // then CV holds 5 V at 0.05 A.
    const check = (samples: Array<{ reg: number | undefined; i: number; u: number }>, engine: SimEngine) => {
      expect(samples[0].reg).toBe(2);
      expect(samples.filter((s) => Math.abs(s.i) > I_LIMIT + 1e-9).length).toBe(0);
      expect(samples.filter((s) => s.u > V + 1e-9).length).toBe(0);
      expect(reg(engine)).toBe(0);
      expect(rail(engine)).toBeCloseTo(V, 9);
      expect(psuCurrent(engine)).toBeCloseTo(-0.05, 9);
    };

    const engine = new SimEngine();
    engine.load(railCap(V, 100));
    const fixed: Array<{ reg: number | undefined; i: number; u: number }> = [];
    for (let k = 0; k < 2000; k++) {
      engine.step(1e-5);
      fixed.push({ reg: reg(engine), i: psuCurrent(engine), u: rail(engine) });
    }
    check(fixed, engine);
    // CC holds until the rail reaches the setpoint (6.93 ms); at 6.90 ms it
    // is 4.984 V.
    expect(fixed.slice(0, 690).filter((s) => s.reg !== 2).length).toBe(0);

    const runner = new HeadlessRunner();
    runner.load(railCap(V, 100));
    const adaptive: Array<{ reg: number | undefined; i: number; u: number }> = [];
    const result = runner.run({
      durationS: 0.05,
      onSample: () => adaptive.push({ reg: reg(runner.engine), i: psuCurrent(runner.engine), u: rail(runner.engine) }),
    });
    expect(result.hitStepCap).toBe(false);
    check(adaptive, runner.engine);
  });
});

describe("bench_psu still holds its rail when a CC load is removed", () => {
  function plainLoad(resistance: number): SimCircuit {
    return {
      components: [
        psu(V),
        { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
      ],
      wires: [
        { from_component: "psu", from_pin: "pos", to_component: "rl", to_pin: "a" },
        { from_component: "rl", from_pin: "b", to_component: "psu", to_pin: "neg" },
      ],
    };
  }

  it("sits at the setpoint on the first accepted headless step after the load opens", () => {
    const runner = new HeadlessRunner();
    runner.load(plainLoad(2));
    runner.run({ durationS: 1e-3 });
    expect(reg(runner.engine)).toBe(2);
    expect(rail(runner.engine)).toBeCloseTo(0.2, 9);

    runner.load(plainLoad(1e9));
    const first: Array<{ u: number; i: number; reg: number | undefined }> = [];
    runner.run({
      durationS: 1e-3,
      onSample: () => first.push({ u: rail(runner.engine), i: psuCurrent(runner.engine), reg: reg(runner.engine) }),
    });

    // 0.1 A into 1 Gohm would be 1e8 V for one step without the compliance branch.
    expect(first[0].u).toBeCloseTo(V, 9);
    expect(Math.abs(first[0].i)).toBeLessThan(1e-8 + 1e-12);
    expect(first.filter((s) => s.reg !== 0 || Math.abs(s.u - V) > 1e-9).length).toBe(0);
  });

  it("charges a rail capacitor at the limit to the setpoint, then holds it in CV", () => {
    // CC into 10 ohm holds the rail at 1 V. With the load gone the limit
    // charges 100 uF at 1000 V/s, so the rail reaches 5 V 4 ms later.
    for (const h of [1e-6, 1e-5]) {
      const engine = new SimEngine();
      engine.load(railCap(V, 10));
      for (let k = 0; k < 200; k++) engine.step(1e-4);
      expect(reg(engine)).toBe(2);
      expect(rail(engine)).toBeCloseTo(1, 6);

      engine.load(railCap(V, 0));
      const steps = Math.round(0.01 / h);
      let maxRail = -Infinity;
      for (let k = 0; k < steps; k++) {
        engine.step(h);
        maxRail = Math.max(maxRail, rail(engine));
        if (k * h < 3.5e-3) expect(psuCurrent(engine)).toBeCloseTo(-I_LIMIT, 9);
      }
      expect(maxRail).toBeLessThanOrEqual(V + 1e-9);
      expect(reg(engine)).toBe(0);
      expect(rail(engine)).toBeCloseTo(V, 9);
      expect(Math.abs(psuCurrent(engine))).toBeLessThan(1e-9);
    }
  });
});
