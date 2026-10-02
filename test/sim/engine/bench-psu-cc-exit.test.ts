/**
 * bench_psu leaves constant-current mode at the setpoint itself.
 *
 * A current-limited supply in CC exits when its current-limited output
 * reaches the setpoint: inside the solve, the CC candidate lands at or above
 * |V| and the compliance branch takes over, and the commit reads that same
 * decision. For a load whose draw rises with voltage, the CC rail reaches |V|
 * exactly when the CV draw at |V| is within the limit.
 *
 * The exit used to fire at 0.98 |V|, so a supply only slightly overloaded,
 * whose CC rail iLimit * R sits between 0.98 |V| and |V|, exited on every
 * accepted step and re-entered CC inside the next solve. `reg` read 2, 0, 2,
 * 0, ... while the current stayed at the limit, and the adaptive runner,
 * which rejects a step whose coarse and half-step regimes differ, rejected
 * every other step with the step size locked at 10-15 ns.
 *
 * Supply 5 V, 0.1 A. 49.9 ohm would draw 5 / 49.9 = 0.1002 A at 5 V, so it
 * holds 0.1 A at 0.1 * 49.9 = 4.99 V = 0.998 |V|. 50.1 ohm draws 0.0998 A at
 * 5 V: CV. Every case runs at +5 V and mirrored at -5 V, where the rail and
 * the delivered current change sign together.
 */

import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const I_LIMIT = 0.1;

function resistiveLoad(voltage: number, resistance: number): SimCircuit {
  return {
    components: [
      { id: "psu", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage, iLimit: I_LIMIT } },
      { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
    ],
    wires: [
      { from_component: "psu", from_pin: "pos", to_component: "rl", to_pin: "a" },
      { from_component: "rl", from_pin: "b", to_component: "psu", to_pin: "neg" },
    ],
  };
}

/** 10 ohm in series with 100 uF across the supply: CC charges the capacitor. */
function seriesRcLoad(voltage: number): SimCircuit {
  return {
    components: [
      { id: "psu", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage, iLimit: I_LIMIT } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
      { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 100e-6 } },
    ],
    wires: [
      { from_component: "psu", from_pin: "pos", to_component: "r", to_pin: "a" },
      { from_component: "r", from_pin: "b", to_component: "c", to_pin: "a" },
      { from_component: "c", from_pin: "b", to_component: "psu", to_pin: "neg" },
    ],
  };
}

function nodeVolts(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.getNetIdForPin(componentId, pinId);
  if (net === undefined) throw new Error(`no net for ${componentId}.${pinId}`);
  return engine.getNetV()[net] ?? 0;
}

const rail = (engine: SimEngine) => nodeVolts(engine, "psu", "pos") - nodeVolts(engine, "psu", "neg");
const capVolts = (engine: SimEngine) => nodeVolts(engine, "c", "a") - nodeVolts(engine, "c", "b");
const reg = (engine: SimEngine) => engine.getIcState("psu")?.reg;
const psuCurrent = (engine: SimEngine) => engine.getElementI().psu ?? Number.NaN;

/** How many readings differ from `expected`. Counted, so a regression reports a number, not thousands of entries. */
const countNot = (readings: Array<number | undefined>, expected: number) => readings.filter((r) => r !== expected).length;
/** How many times consecutive readings differ. */
const changes = (readings: Array<number | undefined>) => readings.filter((r, k) => k > 0 && r !== readings[k - 1]).length;

describe.each([
  { voltage: 5, dir: 1 },
  { voltage: -5, dir: -1 },
])("bench_psu CC exit at the setpoint ($voltage V)", ({ voltage, dir }) => {
  it("runs 50 ms of a 49.9 ohm overload under the headless runner without rejected steps", () => {
    const runner = new HeadlessRunner();
    runner.load(resistiveLoad(voltage, 49.9));
    // The cap bounds the cost this test accepts. Before the fix 4,000,000
    // step attempts (2,670,000 accepted) covered only 33 ms.
    const result = runner.run({ durationS: 0.05, maxSteps: 10_000 });

    // A static load held in CC is a linear DC circuit: the controller grows
    // the step from 10 ns by up to 2x per accepted step to its 10 ms ceiling
    // (about 20 steps), then covers the rest of 50 ms at 10 ms. A 10 ohm load,
    // deep in CC, takes 25. 50 leaves room for controller tuning.
    expect(result.acceptedSteps).toBeLessThanOrEqual(50);
    expect(result.rejectedSteps).toBe(0);
    expect(result.hitStepCap).toBe(false);
    expect(result.simulatedS).toBeCloseTo(0.05, 12);
    expect(rail(runner.engine)).toBeCloseTo(dir * 4.99, 6);
    expect(psuCurrent(runner.engine)).toBeCloseTo(-dir * I_LIMIT, 9);
    expect(reg(runner.engine)).toBe(2);
  });

  it("holds reg 2 on every fixed step while 49.9 ohm overloads it", () => {
    const engine = new SimEngine();
    engine.load(resistiveLoad(voltage, 49.9));
    const regs: Array<number | undefined> = [];
    for (let k = 0; k < 200; k++) {
      engine.step(1e-4);
      regs.push(reg(engine));
    }

    expect(countNot(regs, 2)).toBe(0);
    expect(rail(engine)).toBeCloseTo(dir * 4.99, 6);
    expect(psuCurrent(engine)).toBeCloseTo(-dir * I_LIMIT, 9);
  });

  it("holds reg 2 on every accepted headless step while 49.9 ohm overloads it", () => {
    const runner = new HeadlessRunner();
    runner.load(resistiveLoad(voltage, 49.9));
    const regs: Array<number | undefined> = [];
    runner.run({ durationS: 0.05, maxSteps: 10_000, onSample: () => regs.push(reg(runner.engine)) });

    expect(regs.length).toBeGreaterThan(0);
    expect(countNot(regs, 2)).toBe(0);
  });

  it("holds CC when its CC rail sits 10 ppm under the setpoint", () => {
    // 50 / (1 + 1e-5) ohm draws 0.1 * (1 + 1e-5) A at the setpoint, 10 ppm
    // over the limit, so its CC rail is 0.1 * R = 0.99999 |V|. An exit placed
    // anywhere under that, 0.9999 |V| say, would leave CC on every step; the
    // 49.9 ohm cases above (0.998 |V|) cannot tell those exits apart.
    const resistance = 50 / (1 + 1e-5);
    const engine = new SimEngine();
    engine.load(resistiveLoad(voltage, resistance));
    const fixedRegs: Array<number | undefined> = [];
    for (let k = 0; k < 200; k++) {
      engine.step(1e-4);
      fixedRegs.push(reg(engine));
    }
    expect(countNot(fixedRegs, 2)).toBe(0);
    expect(rail(engine)).toBeCloseTo(dir * I_LIMIT * resistance, 9);
    expect(psuCurrent(engine)).toBeCloseTo(-dir * I_LIMIT, 12);

    const runner = new HeadlessRunner();
    runner.load(resistiveLoad(voltage, resistance));
    const regs: Array<number | undefined> = [];
    const result = runner.run({ durationS: 0.05, maxSteps: 10_000, onSample: () => regs.push(reg(runner.engine)) });
    expect(countNot(regs, 2)).toBe(0);
    expect(result.rejectedSteps).toBe(0);
    expect(result.acceptedSteps).toBeLessThanOrEqual(50);
  });

  it("holds one regime at exactly the limit (50 ohm) under the headless runner, without rejected steps", () => {
    // 5 / 50 = 0.1 A at the setpoint: the limit itself. The rail node's own
    // shunt adds 5e-12 A, so CC and CV give the same rail and current to
    // 1e-10; either regime is right, flipping between them is not. Before
    // the fix the commit entered CC on that 5e-12 A and the next solve's
    // compliance check, which accepted a candidate 5 uV under the setpoint,
    // sent it back: reg flipped every step and the runner crawled.
    const runner = new HeadlessRunner();
    runner.load(resistiveLoad(voltage, 50));
    const regs: Array<number | undefined> = [];
    const result = runner.run({ durationS: 0.05, maxSteps: 10_000, onSample: () => regs.push(reg(runner.engine)) });

    expect(changes(regs)).toBe(0);
    expect(result.rejectedSteps).toBe(0);
    expect(result.acceptedSteps).toBeLessThanOrEqual(50);
    expect(result.hitStepCap).toBe(false);
    expect(rail(runner.engine)).toBeCloseTo(voltage, 9);
    expect(psuCurrent(runner.engine)).toBeCloseTo(-dir * I_LIMIT, 9);
  });

  it("returns to CV once the load lightens below the limit (fixed steps)", () => {
    const engine = new SimEngine();
    engine.load(resistiveLoad(voltage, 49.9));
    const overloaded: Array<number | undefined> = [];
    for (let k = 0; k < 50; k++) {
      engine.step(1e-4);
      overloaded.push(reg(engine));
    }
    expect(countNot(overloaded, 2)).toBe(0);

    engine.load(resistiveLoad(voltage, 50.1));
    const recovered: Array<number | undefined> = [];
    for (let k = 0; k < 50; k++) {
      engine.step(1e-4);
      recovered.push(reg(engine));
    }

    // 5 / 50.1 = 0.0998004 A at the setpoint, inside the limit.
    expect(countNot(recovered, 0)).toBe(0);
    expect(rail(engine)).toBeCloseTo(voltage, 9);
    expect(psuCurrent(engine)).toBeCloseTo(-voltage / 50.1, 9);
  });

  it("returns to CV once the load lightens below the limit (headless runner)", () => {
    const runner = new HeadlessRunner();
    runner.load(resistiveLoad(voltage, 49.9));
    const overloaded: Array<number | undefined> = [];
    const first = runner.run({ durationS: 0.02, maxSteps: 10_000, onSample: () => overloaded.push(reg(runner.engine)) });
    expect(first.hitStepCap).toBe(false);
    expect(countNot(overloaded, 2)).toBe(0);

    runner.load(resistiveLoad(voltage, 50.1));
    const recovered: Array<number | undefined> = [];
    const second = runner.run({ durationS: 0.02, maxSteps: 10_000, onSample: () => recovered.push(reg(runner.engine)) });

    expect(second.rejectedSteps).toBe(0);
    expect(countNot(recovered, 0)).toBe(0);
    expect(rail(runner.engine)).toBeCloseTo(voltage, 9);
    expect(psuCurrent(runner.engine)).toBeCloseTo(-voltage / 50.1, 9);
  });

  it("charges a series RC in CC and switches to CV once the CV current falls to the limit (fixed steps)", () => {
    // CC pushes 0.1 A into 100 uF: the capacitor ramps at 1000 V/s and the
    // rail sits 0.1 A * 10 ohm = 1 V above it, so the rail reaches |V| when the
    // capacitor reaches 4 V, at 4.00 ms (step 400 of 10 us). From there CV
    // charges it with tau = 1 ms. The old exit fired at 0.98 |V| (capacitor
    // 3.9 V) and re-entered CC on the next solve, so reg flickered for 0.1 ms
    // on an unchanged trajectory.
    const engine = new SimEngine();
    engine.load(seriesRcLoad(voltage));
    const regs: Array<number | undefined> = [];
    const at = new Map<number, { vc: number; i: number }>();
    for (let k = 1; k <= 600; k++) {
      engine.step(1e-5);
      regs.push(reg(engine));
      if (k < 400) expect(psuCurrent(engine)).toBeCloseTo(-dir * I_LIMIT, 12);
      at.set(k, { vc: capVolts(engine), i: psuCurrent(engine) });
    }

    // Step 400 lands the CC rail on the setpoint itself, to rounding, where
    // CC and CV are the same solution; either readout is right there.
    expect(countNot(regs.slice(0, 399), 2)).toBe(0);
    expect(countNot(regs.slice(400), 0)).toBe(0);
    expect(at.get(100)!.vc).toBeCloseTo(dir * 1, 9);
    expect(at.get(400)!.vc).toBeCloseTo(dir * 4, 9);
    // Backward Euler at h = 10 us lags the exact 5 - exp(-0.05) = 4.0487706 V
    // by under 0.3 mV; these values are unchanged from before the fix.
    expect(at.get(405)!.vc).toBeCloseTo(dir * 4.048534, 6);
    expect(at.get(405)!.i).toBeCloseTo(-dir * 0.0951466, 6);
  });

  it("charges a series RC in CC under the headless runner: no CV before the rail reaches the setpoint", () => {
    const runner = new HeadlessRunner();
    runner.load(seriesRcLoad(voltage));
    const engine = runner.engine;
    let firstCv: number | null = null;
    let regChanges = 0;
    let lastReg = reg(engine);
    let worstOverLimit = -Infinity;
    const result = runner.run({ durationS: 0.01, maxSteps: 10_000, onSample: (s) => {
      const r = reg(engine);
      if (r !== lastReg) {
        regChanges += 1;
        lastReg = r;
      }
      if (r === 0 && firstCv === null) firstCv = s.simTime;
      worstOverLimit = Math.max(worstOverLimit, Math.abs(psuCurrent(engine)) - I_LIMIT);
    } });

    expect(result.hitStepCap).toBe(false);
    expect(regChanges).toBe(1);
    // The rail reaches the setpoint at 4.00 ms; the old exit left CC at the
    // 0.98 |V| crossing, 3.9 ms.
    expect(firstCv).not.toBeNull();
    expect(firstCv!).toBeGreaterThanOrEqual(4e-3);
    expect(worstOverLimit).toBeLessThanOrEqual(1e-9);
    expect(reg(engine)).toBe(0);
    expect(rail(engine)).toBeCloseTo(voltage, 9);
  });
});
