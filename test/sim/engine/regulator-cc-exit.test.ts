/**
 * The regulators leave current limit (CC) exactly when the solve does.
 *
 * dcdc_converter, linear_reg and lm317 select their voltage branch inside the
 * solve once the CC candidate reaches the voltage the part can hold (the
 * setpoint, or the dropout ceiling under low headroom): the compliance clamp.
 * The commit then decided CC exit by a margin of its own: dcdc_converter at
 * 98% of that voltage; linear_reg 50 mV under its setpoint, and lm317 50 mV
 * under its 1.25 V reference on Vout - Vadj (about 200 mV of output with a
 * 240/720 ohm divider); and both linear regulators left for DROPOUT on any
 * low headroom. A part still over its limit whose CC output sat inside that
 * margin, or whose headroom was short (a linear_reg in dropout at any
 * overload; an LM317, whose headroom is measured from ADJ and so grows as its
 * output sags, only while slightly overloaded), left CC at commit and
 * re-entered it inside the next solve. The readings were right, but `reg`
 * flipped on every step, and the adaptive runner, which rejects a step whose
 * coarse and half-step regimes differ, rejected one attempt in three to five
 * with the step size locked near 12 ns: 50 ms ran into a 20,000-step cap
 * after 0.2 ms at most. The commit now reads the solve's own compliance
 * decision, and the regime it stamped is the one committed.
 *
 * Outside the in-solve band (the last part of this file) the electrical
 * trajectory is unchanged: each solve already re-entered CC, so only `reg`
 * differs, and the trajectories below are the 0.6.1 values.
 *
 * Expected values are derived in comments and written as literals; the
 * LM317 ones that depend on its 100 kohm ADJ bias are worked out here:
 *   R2 || 100k = 720 * 1e5 / 100720 = 714.853 ohm, divider 954.853 ohm.
 *   Setpoint: Vout - Vadj = 1.25 with Vadj = Vout * 714.853 / 954.853, so
 *   Vout = 1.25 * 954.853 / 240 = 4.973193 V, and the divider draws
 *   4.973193 / 954.853 = 5.2083 mA there.
 */

import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Kind = "dcdc" | "buck" | "linear" | "lm317";
type Terminal = [component: string, pin: string];
type Load = (out: Terminal, ret: Terminal) => Pick<SimCircuit, "components" | "wires">;

const wire = (a: Terminal, b: Terminal) => ({ from_component: a[0], from_pin: a[1], to_component: b[0], to_pin: b[1] });
const resistor = (id: string, resistance: number) => ({ id, kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } });
const capacitor = (id: string, capacitance: number) => ({ id, kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance } });

const OUT: Record<Kind, Terminal> = { dcdc: ["u", "out_pos"], buck: ["u", "out_pos"], linear: ["u", "out"], lm317: ["u", "out"] };
const RET: Record<Kind, Terminal> = { dcdc: ["u", "out_neg"], buck: ["u", "out_neg"], linear: ["vs", "neg"], lm317: ["vs", "neg"] };

/** The regulator `u` fed from an ideal `vin` source, with `load` across its output. */
function regulator(kind: Kind, vin: number, params: Record<string, number>, load: Load): SimCircuit {
  const components: SimCircuit["components"] = [
    { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: vin } },
  ];
  const wires: SimCircuit["wires"] = [];
  if (kind === "dcdc" || kind === "buck") {
    components.push({
      id: "u", kind: "dcdc_converter", ...(kind === "buck" ? { catalogUid: "dcdc-buck-5v" } : {}),
      pins: [{ id: "in_pos" }, { id: "in_neg" }, { id: "out_pos" }, { id: "out_neg" }],
      params: { eta: 0.85, ...params },
    });
    wires.push(wire(["vs", "pos"], ["u", "in_pos"]), wire(["vs", "neg"], ["u", "in_neg"]));
  } else if (kind === "linear") {
    components.push({ id: "u", kind: "linear_reg", pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }], params });
    wires.push(wire(["vs", "pos"], ["u", "in"]), wire(["vs", "neg"], ["u", "gnd"]));
  } else {
    components.push(
      { id: "u", kind: "lm317", pins: [{ id: "in" }, { id: "adj" }, { id: "out" }], params },
      resistor("r1", 240),
      resistor("r2", 720),
    );
    wires.push(
      wire(["vs", "pos"], ["u", "in"]),
      wire(["u", "out"], ["r1", "a"]),
      wire(["r1", "b"], ["u", "adj"]),
      wire(["u", "adj"], ["r2", "a"]),
      wire(["r2", "b"], ["vs", "neg"]),
    );
  }
  const l = load(OUT[kind], RET[kind]);
  return { components: [...components, ...l.components], wires: [...wires, ...l.wires] };
}

const resistive = (r: number): Load => (out, ret) => ({
  components: [resistor("rl", r)],
  wires: [wire(out, ["rl", "a"]), wire(["rl", "b"], ret)],
});
const resistorParallelCap = (r: number, c: number): Load => (out, ret) => ({
  components: [resistor("rl", r), capacitor("c", c)],
  wires: [wire(out, ["rl", "a"]), wire(["rl", "b"], ret), wire(out, ["c", "a"]), wire(["c", "b"], ret)],
});
const resistorSeriesCap = (r: number, c: number): Load => (out, ret) => ({
  components: [resistor("rs", r), capacitor("c", c)],
  wires: [wire(out, ["rs", "a"]), wire(["rs", "b"], ["c", "a"]), wire(["c", "b"], ret)],
});
const ledWithResistor = (r: number): Load => (out, ret) => ({
  components: [resistor("rl", r), { id: "d", kind: "led", pins: [{ id: "anode" }, { id: "cathode" }], params: { vf: 2.0 } }],
  wires: [wire(out, ["rl", "a"]), wire(["rl", "b"], ["d", "anode"]), wire(["d", "cathode"], ret)],
});

/** The same circuit with its input falling linearly from v1 at t = 0 to v2 at tr, then held. */
function withFallingInput(circuit: SimCircuit, v1: number, v2: number, tr: number): SimCircuit {
  return {
    ...circuit,
    components: circuit.components.map((c) => c.id === "vs"
      ? { id: "vs", kind: "pulse_source", pins: [{ id: "pos" }, { id: "neg" }], params: { v1, v2, td: 0, tr, tf: 1e-6, pw: 1, per: 0 } }
      : c),
  };
}

function volts(engine: SimEngine, [component, pin]: Terminal): number {
  const net = engine.getNetIdForPin(component, pin);
  if (net === undefined) throw new Error(`no net for ${component}.${pin}`);
  return engine.getNetV()[net] ?? 0;
}
const rail = (engine: SimEngine, kind: Kind) => volts(engine, OUT[kind]) - volts(engine, RET[kind]);
const outputAmps = (engine: SimEngine) => Math.abs(engine.getElementI().u ?? Number.NaN);
const reg = (engine: SimEngine) => engine.getIcState("u")?.reg;

function stepFixed(engine: SimEngine, steps: number, h: number): Array<number | undefined> {
  const regs: Array<number | undefined> = [];
  for (let k = 0; k < steps; k++) {
    engine.step(h);
    regs.push(reg(engine));
  }
  return regs;
}

function runHeadless(runner: HeadlessRunner, durationS: number) {
  const regs: Array<number | undefined> = [];
  let peakAmps = 0;
  const result = runner.run({ durationS, maxSteps: 20_000, onSample: () => {
    regs.push(reg(runner.engine));
    peakAmps = Math.max(peakAmps, outputAmps(runner.engine));
  } });
  return { result, regs, peakAmps };
}

interface Row {
  label: string;
  kind: Kind;
  params: Record<string, number>;
}

interface OverloadCase extends Row {
  vin: number;
  limit: number;
  /** A load just over the limit, and the CC output it holds. */
  over: { resistance: number; volts: number };
  /** A load just under the limit, and the regime, output and current it settles at. */
  under: { resistance: number; reg: 0 | 1; regime: string; volts: number; amps: number };
}

const overloadCases: OverloadCase[] = [
  {
    // 5 V into 4.95 ohm draws 1.0101 A, so CC holds 1 A * 4.95 ohm = 4.95 V,
    // 99% of the setpoint, inside the old 98% exit. 5.01 ohm draws
    // 5 / 5.01 = 0.998004 A at 5 V: within the limit.
    label: "dcdc_converter, 12 V -> 5 V, 1 A",
    kind: "dcdc",
    vin: 12,
    params: { vout: 5, iLimit: 1 },
    limit: 1,
    over: { resistance: 4.95, volts: 4.95 },
    under: { resistance: 5.01, reg: 0, regime: "REG", volts: 5, amps: 0.998004 },
  },
  {
    // 4.97 ohm holds 4.97 V in CC, 30 mV under the setpoint, inside the old
    // 50 mV exit.
    label: "linear_reg, 12 V -> 5 V, 1 A",
    kind: "linear",
    vin: 12,
    params: { vout: 5, vdropout: 2, iLimit: 1 },
    limit: 1,
    over: { resistance: 4.97, volts: 4.97 },
    under: { resistance: 5.01, reg: 0, regime: "REG", volts: 5, amps: 0.998004 },
  },
  {
    // 52 ohm in parallel with the 954.853 ohm divider is 49.3144 ohm, so CC
    // holds 0.1 A * 49.3144 ohm = 4.931440 V, and Vout - Vadj =
    // 4.931440 * 240 / 954.853 = 1.2395 V, inside the old 50 mV exit. 54 ohm
    // draws 4.973193 / 54 + 0.0052083 = 0.0973045 A at the setpoint.
    label: "lm317, 12 V in, 240/720 ohm divider, 0.1 A",
    kind: "lm317",
    vin: 12,
    params: { vref: 1.25, vdropout: 2, iLimit: 0.1 },
    limit: 0.1,
    over: { resistance: 52, volts: 4.931440 },
    under: { resistance: 54, reg: 0, regime: "REG", volts: 4.973193, amps: 0.0973045 },
  },
  {
    // 6 V in leaves 6 - 2 = 4 V of output (dropout). 3.9 ohm draws 1.0256 A
    // at 4 V, so CC holds 3.9 V. The old exit switched any CC regulator with
    // headroom under vout + vdropout = 7 V to DROPOUT, whatever its load.
    // 4.01 ohm draws 4 / 4.01 = 0.997506 A at 4 V.
    label: "linear_reg in dropout, 6 V in (4 V ceiling), 1 A",
    kind: "linear",
    vin: 6,
    params: { vout: 5, vdropout: 2, iLimit: 1 },
    limit: 1,
    over: { resistance: 3.9, volts: 3.9 },
    under: { resistance: 4.01, reg: 1, regime: "DROPOUT", volts: 4, amps: 0.997506 },
  },
  {
    // The identified buck from 9 V can hold 9 - 1.5 = 7.5 V of its 8 V
    // setpoint. 14.8 ohm draws 0.5068 A at 7.5 V, so CC holds
    // 0.5 A * 14.8 ohm = 7.4 V, 98.7% of the ceiling, inside the old 98%
    // exit. 15.02 ohm draws 7.5 / 15.02 = 0.499334 A at 7.5 V.
    label: "dcdc-buck-5v in dropout, 9 V in (7.5 V ceiling), 0.5 A",
    kind: "buck",
    vin: 9,
    params: { vout: 8, iLimit: 0.5, vdropout: 1.5 },
    limit: 0.5,
    over: { resistance: 14.8, volts: 7.4 },
    under: { resistance: 15.02, reg: 1, regime: "DROPOUT", volts: 7.5, amps: 0.499334 },
  },
];

describe.each(overloadCases)("$label: slight overload", ({ kind, vin, params, limit, over, under }) => {
  const circuit = (resistance: number) => regulator(kind, vin, params, resistive(resistance));

  it("holds reg 2 on every fixed step", () => {
    const engine = new SimEngine();
    engine.load(circuit(over.resistance));
    const regs = stepFixed(engine, 200, 1e-4);

    expect(regs.filter((r) => r !== 2)).toEqual([]);
    expect(rail(engine, kind)).toBeCloseTo(over.volts, 6);
    expect(outputAmps(engine)).toBeCloseTo(limit, 9);
    expect(engine.getFailures()).toEqual({});
  });

  it("runs 50 ms under the headless runner without rejected steps, reg 2 on every step", () => {
    const runner = new HeadlessRunner();
    runner.load(circuit(over.resistance));
    const { result, regs } = runHeadless(runner, 0.05);

    // A static load held in CC is a linear DC circuit: the controller grows
    // the step from 10 ns by up to 2x per accepted step to its 10 ms ceiling
    // (about 20 steps), then covers the rest at 10 ms; these take 25. 50
    // leaves room for controller tuning and is still hundreds of times under
    // the old crawl, which hit the 20,000-step cap after 0.2 ms at most.
    expect(result.acceptedSteps).toBeLessThanOrEqual(50);
    expect(result.rejectedSteps).toBe(0);
    expect(result.hitStepCap).toBe(false);
    expect(result.simulatedS).toBeCloseTo(0.05, 12);
    expect(regs.filter((r) => r !== 2)).toEqual([]);
    expect(rail(runner.engine, kind)).toBeCloseTo(over.volts, 6);
    expect(outputAmps(runner.engine)).toBeCloseTo(limit, 9);
    expect(runner.engine.getFailures()).toEqual({});
  });

  it(`returns to ${under.regime} once the load lightens under the limit (fixed steps)`, () => {
    const engine = new SimEngine();
    engine.load(circuit(over.resistance));
    expect(stepFixed(engine, 50, 1e-4).filter((r) => r !== 2)).toEqual([]);

    engine.load(circuit(under.resistance));
    const regs = stepFixed(engine, 50, 1e-4);

    expect(regs.filter((r) => r !== under.reg)).toEqual([]);
    expect(rail(engine, kind)).toBeCloseTo(under.volts, 6);
    expect(outputAmps(engine)).toBeCloseTo(under.amps, 6);
  });

  it(`returns to ${under.regime} once the load lightens under the limit (headless runner)`, () => {
    const runner = new HeadlessRunner();
    runner.load(circuit(over.resistance));
    const overloaded = runHeadless(runner, 0.02);
    expect(overloaded.result.hitStepCap).toBe(false);
    expect(overloaded.regs.filter((r) => r !== 2)).toEqual([]);

    runner.load(circuit(under.resistance));
    const recovered = runHeadless(runner, 0.02);

    expect(recovered.result.rejectedSteps).toBe(0);
    expect(recovered.result.hitStepCap).toBe(false);
    expect(recovered.regs.filter((r) => r !== under.reg)).toEqual([]);
    expect(rail(runner.engine, kind)).toBeCloseTo(under.volts, 6);
    expect(outputAmps(runner.engine)).toBeCloseTo(under.amps, 6);
  });
});

interface FallingInputCase extends Row {
  /** Input from v1 at t = 0 down to v2 at 30 ms, then held. */
  v1: number;
  v2: number;
  load: number;
  limit: number;
  /** CC output, held through lastCcStep of 0.1 ms. */
  ccVolts: number;
  lastCcStep: number;
  /** DROPOUT output and current on the next step, and at 30 ms. */
  firstDropout: { volts: number; amps: number };
  end: { volts: number; amps: number };
  /** When the falling dropout ceiling reaches the CC output. */
  crossingS: number;
}

// Every other recovery above goes through engine.load(), whose seed solve
// already commits the exit, and the next step would correct a wrong regime
// before anything reads it. A falling input leaves CC on an ordinary step.
const fallingInputCases: FallingInputCase[] = [
  {
    // At 0.1 ms steps the input is 7.4 - 0.8 k / 300 V. 4.97 ohm draws more
    // than 1 A at any voltage over 4.97 V, so the output holds 4.97 V in CC
    // while the ceiling vin - 2 stays above it: through step 161 (6.970667 V
    // in). From step 151 the input is under vout + vdropout = 7 V, where the
    // old exit already switched to DROPOUT. Step 162 (6.968 V in) leaves CC
    // into DROPOUT at 4.968 V, 4.968 / 4.97 = 0.999598 A; at 30 ms, 6.6 V in,
    // 4.6 V and 0.925553 A. The ceiling meets 4.97 V at 16.125 ms.
    label: "linear_reg, 1 A into 4.97 ohm, input 7.4 V -> 6.6 V",
    kind: "linear",
    params: { vout: 5, vdropout: 2, iLimit: 1 },
    v1: 7.4,
    v2: 6.6,
    load: 4.97,
    limit: 1,
    ccVolts: 4.97,
    lastCcStep: 161,
    firstDropout: { volts: 4.968, amps: 0.999598 },
    end: { volts: 4.6, amps: 0.925553 },
    crossingS: 16.125e-3,
  },
  {
    // The input is 10 - 1.4 k / 300 V. 14.8 ohm holds 0.5 A * 14.8 ohm =
    // 7.4 V in CC while the ceiling min(8, vin - 1.5) stays above it: through
    // step 235 (8.903333 V in). From step 204 (9.048 V in) the CC output is
    // inside 98% of the ceiling, where the old exit fired. Step 236
    // (8.898667 V in) leaves CC into DROPOUT at 7.398667 V, 0.499910 A; at
    // 30 ms, 8.6 V in, 7.1 V and 0.479730 A. The ceiling meets 7.4 V at
    // 23.571 ms.
    label: "dcdc-buck-5v, 0.5 A into 14.8 ohm, input 10 V -> 8.6 V",
    kind: "buck",
    params: { vout: 8, iLimit: 0.5, vdropout: 1.5 },
    v1: 10,
    v2: 8.6,
    load: 14.8,
    limit: 0.5,
    ccVolts: 7.4,
    lastCcStep: 235,
    firstDropout: { volts: 7.398667, amps: 0.499910 },
    end: { volts: 7.1, amps: 0.479730 },
    crossingS: 23.571e-3,
  },
];

describe.each(fallingInputCases)("$label: CC to DROPOUT without a reload", ({ kind, params, v1, v2, load, limit, ccVolts, lastCcStep, firstDropout, end, crossingS }) => {
  const circuit = () => withFallingInput(regulator(kind, v1, params, resistive(load)), v1, v2, 0.03);

  it("goes straight from CC to DROPOUT on the step the ceiling falls under the CC output (fixed 0.1 ms steps)", () => {
    const engine = new SimEngine();
    engine.load(circuit());
    const regs: Array<number | undefined> = [];
    const at = new Map<number, { v: number; i: number }>();
    for (let k = 1; k <= 300; k++) {
      engine.step(1e-4);
      regs.push(reg(engine));
      at.set(k, { v: rail(engine, kind), i: outputAmps(engine) });
    }

    expect(regs.slice(0, lastCcStep).filter((r) => r !== 2)).toEqual([]);
    // The exit commits the branch the solve stamped: DROPOUT on its first
    // step, never one step of REG that the next solve would correct.
    expect(regs.slice(lastCcStep).filter((r) => r !== 1)).toEqual([]);
    expect(at.get(lastCcStep)!.v).toBeCloseTo(ccVolts, 9);
    expect(at.get(lastCcStep)!.i).toBeCloseTo(limit, 12);
    expect(at.get(lastCcStep + 1)!.v).toBeCloseTo(firstDropout.volts, 6);
    expect(at.get(lastCcStep + 1)!.i).toBeCloseTo(firstDropout.amps, 6);
    expect(at.get(300)!.v).toBeCloseTo(end.volts, 6);
    expect(at.get(300)!.i).toBeCloseTo(end.amps, 6);
    expect(engine.getFailures()).toEqual({});
  });

  it("goes straight from CC to DROPOUT, once the ceiling reaches the CC output, under the headless runner", () => {
    const runner = new HeadlessRunner();
    runner.load(circuit());
    const sequence: Array<number | undefined> = [];
    let firstDropoutS: number | null = null;
    let peakAmps = 0;
    const result = runner.run({ durationS: 0.03, maxSteps: 20_000, onSample: (s) => {
      const r = reg(runner.engine);
      if (sequence[sequence.length - 1] !== r) sequence.push(r);
      if (r !== 2 && firstDropoutS === null) firstDropoutS = s.simTime;
      peakAmps = Math.max(peakAmps, outputAmps(runner.engine));
    } });

    // A changing input holds the controller at its 2 ms active ceiling: 15
    // steps for 30 ms plus the climb from 10 ns, 42 and 49 accepted here.
    // 100 leaves room for controller tuning.
    expect(result.acceptedSteps).toBeLessThanOrEqual(100);
    expect(result.hitStepCap).toBe(false);
    expect(sequence).toEqual([2, 1]);
    expect(firstDropoutS).not.toBeNull();
    expect(firstDropoutS!).toBeGreaterThanOrEqual(crossingS);
    expect(peakAmps).toBeLessThanOrEqual(limit + 1e-12);
    expect(rail(runner.engine, kind)).toBeCloseTo(end.volts, 6);
  });
});

describe.each<Row & { limit: number; setpoint: number }>([
  // The LED draws 20.00 mA at 5 V through 150 ohm (19.82 mA at the LM317's
  // 4.973 V, plus 5.21 mA in its divider). Each limit sits 0.5% under that
  // draw, so the CC output sits about 15 mV under the setpoint, inside both
  // old exits. Through 160 ohm the LED draws about 18.8 mA (23.8 mA with the
  // divider), under the limit.
  { label: "dcdc_converter", kind: "dcdc", params: { vout: 5, iLimit: 0.0199 }, limit: 0.0199, setpoint: 5 },
  { label: "linear_reg", kind: "linear", params: { vout: 5, vdropout: 2, iLimit: 0.0199 }, limit: 0.0199, setpoint: 5 },
  { label: "lm317", kind: "lm317", params: { vref: 1.25, vdropout: 2, iLimit: 0.0249 }, limit: 0.0249, setpoint: 4.973193 },
])("$label: LED load slightly over the limit", ({ kind, params, limit, setpoint }) => {
  const circuit = (r: number) => regulator(kind, 12, params, ledWithResistor(r));

  it("holds reg 2 on every fixed step, then returns to REG when the LED's resistor grows", () => {
    const engine = new SimEngine();
    engine.load(circuit(150));
    const overloaded = stepFixed(engine, 200, 1e-4);
    expect(overloaded.filter((r) => r !== 2)).toEqual([]);
    expect(outputAmps(engine)).toBeCloseTo(limit, 9);
    expect(rail(engine, kind)).toBeLessThan(setpoint - 0.005);
    expect(rail(engine, kind)).toBeGreaterThan(setpoint - 0.05);

    engine.load(circuit(160));
    const recovered = stepFixed(engine, 50, 1e-4);
    expect(recovered.filter((r) => r !== 0)).toEqual([]);
    expect(rail(engine, kind)).toBeCloseTo(setpoint, 6);
    expect(outputAmps(engine)).toBeLessThan(limit);
  });

  it("runs 50 ms under the headless runner without rejected steps, then returns to REG", () => {
    const runner = new HeadlessRunner();
    runner.load(circuit(150));
    const overloaded = runHeadless(runner, 0.05);
    // Same bound as the resistive loads: the LED converges inside each step.
    expect(overloaded.result.acceptedSteps).toBeLessThanOrEqual(50);
    expect(overloaded.result.rejectedSteps).toBe(0);
    expect(overloaded.result.hitStepCap).toBe(false);
    expect(overloaded.regs.filter((r) => r !== 2)).toEqual([]);
    expect(outputAmps(runner.engine)).toBeCloseTo(limit, 9);

    runner.load(circuit(160));
    const recovered = runHeadless(runner, 0.02);
    expect(recovered.result.rejectedSteps).toBe(0);
    expect(recovered.regs.filter((r) => r !== 0)).toEqual([]);
    expect(rail(runner.engine, kind)).toBeCloseTo(setpoint, 6);
  });
});

describe("cold start into 10 ohm + 100 uF at a 0.1 A limit: CC until the rail reaches the setpoint, then regulation", () => {
  // CC pushes 0.1 A into 100 uF, so after k steps of 10 us the capacitor holds
  // 0.01 k V and the output sits 0.1 A * 10 ohm = 1 V above it: 2 V at step
  // 100, 4 V at step 300, and the setpoint at step 400 (capacitor at 4 V). From
  // there backward Euler gives i_k = 0.1 * (10 / 10.1)^(k - 400) A. The old
  // dcdc_converter exit fired at 4.9 V (step 390) and linear_reg's at 4.95 V
  // (step 395), and each re-entered CC in the next solve until step 400.
  it.each<Row>([
    { label: "dcdc_converter", kind: "dcdc", params: { vout: 5, iLimit: 0.1 } },
    { label: "linear_reg", kind: "linear", params: { vout: 5, vdropout: 2, iLimit: 0.1 } },
  ])("$label keeps the 0.6.1 trajectory with one regime change (fixed 10 us steps)", ({ kind, params }) => {
    const engine = new SimEngine();
    engine.load(regulator(kind, 12, params, resistorSeriesCap(10, 1e-4)));
    const regs: Array<number | undefined> = [];
    const at = new Map<number, { v: number; i: number }>();
    for (let k = 1; k <= 600; k++) {
      engine.step(1e-5);
      regs.push(reg(engine));
      if (k < 400) expect(outputAmps(engine)).toBeCloseTo(0.1, 12);
      if (k >= 400) expect(rail(engine, kind)).toBeCloseTo(5, 9);
      at.set(k, { v: rail(engine, kind), i: outputAmps(engine) });
    }

    // Step 400 lands the CC output on the setpoint itself, to rounding, where
    // CC and REG are the same solution; either readout is right there.
    expect(regs.slice(0, 399).filter((r) => r !== 2)).toEqual([]);
    expect(regs.slice(400).filter((r) => r !== 0)).toEqual([]);
    expect(at.get(100)!.v).toBeCloseTo(2, 9);
    expect(at.get(300)!.v).toBeCloseTo(4, 9);
    expect(at.get(401)!.i).toBeCloseTo(0.0990099, 7);
    expect(at.get(405)!.i).toBeCloseTo(0.0951466, 7);
    expect(at.get(500)!.i).toBeCloseTo(0.0369711, 7);
  });

  it("lm317 keeps the 0.6.1 trajectory with one regime change (fixed 10 us steps)", () => {
    // The divider takes up to 5.2 mA of the 0.1 A, so the capacitor charges
    // more slowly and the output reaches the 4.973193 V setpoint at step 416
    // (continuous time 4.156 ms). The values are the 0.6.1 trajectory, where
    // the old exit, 50 mV under the 1.25 V reference on Vout - Vadj (199 mV
    // of output here), flickered from step 395 to step 416.
    const engine = new SimEngine();
    engine.load(regulator("lm317", 12, { vref: 1.25, vdropout: 2, iLimit: 0.1 }, resistorSeriesCap(10, 1e-4)));
    const regs: Array<number | undefined> = [];
    const at = new Map<number, { v: number; i: number }>();
    for (let k = 1; k <= 600; k++) {
      engine.step(1e-5);
      regs.push(reg(engine));
      if (k < 416) expect(outputAmps(engine)).toBeCloseTo(0.1, 12);
      at.set(k, { v: rail(engine, "lm317"), i: outputAmps(engine) });
    }

    expect(regs.slice(0, 415).filter((r) => r !== 2)).toEqual([]);
    expect(regs.slice(415).filter((r) => r !== 0)).toEqual([]);
    expect(at.get(100)!.v).toBeCloseTo(1.963906588, 9);
    expect(at.get(300)!.v).toBeCloseTo(3.882417044, 9);
    expect(at.get(405)!.v).toBeCloseTo(4.873828538, 9);
    expect(at.get(416)!.v).toBeCloseTo(4.973193008, 9);
    expect(at.get(416)!.i).toBeCloseTo(0.099612304, 9);
    expect(at.get(500)!.i).toBeCloseTo(0.046133915, 9);
    expect(at.get(600)!.i).toBeCloseTo(0.020338980, 9);
  });

  it.each<Row & { setpoint: number; crossing: number }>([
    { label: "dcdc_converter", kind: "dcdc", params: { vout: 5, iLimit: 0.1 }, setpoint: 5, crossing: 4.0e-3 },
    { label: "lm317", kind: "lm317", params: { vref: 1.25, vdropout: 2, iLimit: 0.1 }, setpoint: 4.973193, crossing: 4.1e-3 },
  ])("$label changes regime once under the headless runner, at the setpoint", ({ kind, params, setpoint, crossing }) => {
    const runner = new HeadlessRunner();
    runner.load(regulator(kind, 12, params, resistorSeriesCap(10, 1e-4)));
    const engine = runner.engine;
    let firstReg: number | null = null;
    let regChanges = 0;
    let lastReg = reg(engine);
    let worstOverLimit = -Infinity;
    const result = runner.run({ durationS: 0.01, maxSteps: 20_000, onSample: (s) => {
      const r = reg(engine);
      if (r !== lastReg) {
        regChanges += 1;
        lastReg = r;
      }
      if (r === 0 && firstReg === null) firstReg = s.simTime;
      worstOverLimit = Math.max(worstOverLimit, outputAmps(engine) - 0.1);
    } });

    expect(result.hitStepCap).toBe(false);
    expect(regChanges).toBe(1);
    // The output reaches the setpoint at 4.00 ms (dcdc_converter) and
    // 4.16 ms (lm317); the old exits left CC up to 0.1 ms and 0.2 ms early.
    expect(firstReg).not.toBeNull();
    expect(firstReg!).toBeGreaterThanOrEqual(crossing);
    expect(worstOverLimit).toBeLessThanOrEqual(1e-9);
    expect(reg(engine)).toBe(0);
    expect(rail(engine, kind)).toBeCloseTo(setpoint, 6);
  });
});

describe.each<Row>([
  { label: "dcdc_converter", kind: "dcdc", params: { vout: 5, iLimit: 1 } },
  { label: "linear_reg", kind: "linear", params: { vout: 5, vdropout: 2, iLimit: 1 } },
])("$label: 100 uF output capacitor discharging through the old exit window", ({ kind, params }) => {
  // Settled at 5 V with 100 ohm || 100 uF, the load steps to 5/1.2 ohm, 20%
  // over a 1 A limit. CC discharges the capacitor toward 1 A * 4.16667 ohm;
  // at h = 1 us backward Euler gives v_k = 25/6 + (5/6) * (100 / 100.24)^k.
  // The old exits fired on every step until the output fell through 4.9 V
  // (dcdc_converter) or 4.95 V (linear_reg).
  const circuit = (r: number) => regulator(kind, 12, params, resistorParallelCap(r, 1e-4));

  it("holds reg 2 on every fixed step from the load step on, on the 0.6.1 trajectory", () => {
    const engine = new SimEngine();
    engine.load(circuit(100));
    stepFixed(engine, 200, 1e-4);
    expect(reg(engine)).toBe(0);
    expect(rail(engine, kind)).toBeCloseTo(5, 9);

    engine.load(circuit(5 / 1.2));
    const regs: Array<number | undefined> = [];
    const at = new Map<number, number>();
    for (let k = 1; k <= 2000; k++) {
      engine.step(1e-6);
      regs.push(reg(engine));
      expect(outputAmps(engine)).toBeCloseTo(1, 12);
      at.set(k, rail(engine, kind));
    }

    expect(regs.filter((r) => r !== 2)).toEqual([]);
    expect(at.get(1)!).toBeCloseTo(4.998004789, 9);
    expect(at.get(10)!).toBeCloseTo(4.980261485, 9);
    expect(at.get(2000)!).toBeCloseTo(4.173564343, 9);
  });

  it("runs 50 ms of the discharge under the headless runner without flicker", () => {
    const runner = new HeadlessRunner();
    runner.load(circuit(100));
    runner.run({ durationS: 0.05 });
    expect(reg(runner.engine)).toBe(0);

    runner.load(circuit(5 / 1.2));
    const { result, regs } = runHeadless(runner, 0.05);

    // The 417 us RC transient takes 45 accepted steps; the old exits took
    // 4,301 accepted and 1,115 rejected (dcdc_converter) or 2,102 and 1,030
    // (linear_reg). 100 leaves room for controller tuning.
    expect(result.acceptedSteps).toBeLessThanOrEqual(100);
    expect(result.rejectedSteps).toBeLessThanOrEqual(5);
    expect(result.hitStepCap).toBe(false);
    expect(regs.filter((r) => r !== 2)).toEqual([]);
    expect(rail(runner.engine, kind)).toBeCloseTo(25 / 6, 6);
    expect(outputAmps(runner.engine)).toBeCloseTo(1, 9);
  });
});

// ─── The in-solve compliance band ───────────────────────────────────────────
//
// Inside a solve, a CC candidate that comes back at the voltage the part can
// hold hands it to REG/DROPOUT: that is how a current-limited regulator whose
// load is removed holds its output instead of pushing the limit current into
// nothing for a step. The test accepted any candidate within
// 1e-9 + 1e-6 * max(1, V) under that voltage, about 5 uV at 5 V. A capacitor
// on the output keeps an overloaded regulator's candidate inside that band:
// at a step h it falls only (excess current) * h / C below it, 200 nV for a
// 0.2% overload of 1 A on 100 uF at 10 ns. The regulator then sat at its
// setpoint delivering past its limit (1.002 A through 1 A; an LM317, whose
// Vout - Vadj moves by only a quarter of its output's fall, 0.121 A through
// 0.1 A), or at 20 ppm over held CC by 2 nV on one step and handed back to
// REG on the next. Either way reg flipped every step and the adaptive runner
// hit a 20,000-step cap within 0.2 ms. REG/DROPOUT now takes over only once
// the candidate reaches the voltage itself.
//
// Each case settles in REG on 100 uF and a light load, then steps the load
// over the limit. CC discharges the capacitor toward v_inf = I_limit * R_eff,
// where R_eff is the load (in parallel with the LM317's 954.853 ohm divider),
// and backward Euler at h = 10 ns gives the drop from the settled v_0 as
// (v_inf - v_0) * (1 - rho^k) with rho = (C / h) / (C / h + 1 / R_eff).

interface CapacitorCase extends Row {
  limit: number;
  setpoint: number;
  /** Settling load, under the limit. */
  light: number;
  /** The overload resistance. */
  over: number;
  /** Output drop from the settled voltage after steps 1, 10 and 2000 of 10 ns. */
  drops: [number, number, number];
  /** v_inf = I_limit * R_eff, where the headless run settles. */
  settled: number;
}

const capacitorCases: CapacitorCase[] = [
  {
    // 5 / 1.002 = 4.990020 ohm draws 1.002 A at 5 V. rho = 1e4 / (1e4 + 1.002 / 5).
    label: "dcdc_converter 0.2% over",
    kind: "dcdc",
    params: { vout: 5, iLimit: 1 },
    limit: 1,
    setpoint: 5,
    light: 100,
    over: 5 / 1.002,
    drops: [-1.999960e-7, -1.999780e-6, -3.920862e-4],
    settled: 4.990020,
  },
  {
    label: "linear_reg 0.2% over",
    kind: "linear",
    params: { vout: 5, vdropout: 2, iLimit: 1 },
    limit: 1,
    setpoint: 5,
    light: 100,
    over: 5 / 1.002,
    drops: [-1.999960e-7, -1.999780e-6, -3.920862e-4],
    settled: 4.990020,
  },
  {
    // 43 ohm draws 4.973193 / 43 + 0.0052083 = 0.120864 A at the setpoint,
    // 21% over. R_eff = 43 || 954.853 = 41.147022 ohm.
    label: "lm317 21% over",
    kind: "lm317",
    params: { vref: 1.25, vdropout: 2, iLimit: 0.1 },
    limit: 0.1,
    setpoint: 4.973193,
    light: 1000,
    over: 43,
    drops: [-2.086393e-6, -2.086371e-5, -4.162667e-3],
    settled: 4.114702,
  },
  {
    // 5 / 1.00002 = 4.999900 ohm draws 1.00002 A at 5 V: 20 ppm over.
    label: "linear_reg 20 ppm over",
    kind: "linear",
    params: { vout: 5, vdropout: 2, iLimit: 1 },
    limit: 1,
    setpoint: 5,
    light: 100,
    over: 5 / 1.00002,
    drops: [-1.999960e-9, -1.999780e-8, -3.921016e-6],
    settled: 4.999900,
  },
  {
    // 52.463347 ohm draws 4.973193 / 52.463347 + 0.0052083 = 0.100002 A at
    // the setpoint: 20 ppm over. R_eff = 52.463347 || 954.853 = 49.730935 ohm.
    label: "lm317 20 ppm over",
    kind: "lm317",
    params: { vref: 1.25, vdropout: 2, iLimit: 0.1 },
    limit: 0.1,
    setpoint: 4.973193,
    light: 1000,
    over: 52.463347,
    drops: [-1.999611e-10, -1.999593e-9, -3.991196e-7],
    settled: 4.973094,
  },
];

describe.each(capacitorCases)("$label with 100 uF on the output", ({ kind, params, limit, setpoint, light, over, drops, settled }) => {
  const circuit = (r: number) => regulator(kind, 12, params, resistorParallelCap(r, 1e-4));

  it("enters CC on the first 10 ns step and holds it, delivering exactly the limit", () => {
    const engine = new SimEngine();
    engine.load(circuit(light));
    stepFixed(engine, 200, 1e-4);
    expect(reg(engine)).toBe(0);
    const settledRail = rail(engine, kind);
    expect(settledRail).toBeCloseTo(setpoint, 6);

    engine.load(circuit(over));
    const regs: Array<number | undefined> = [];
    const rails = new Map<number, number>();
    let peakAmps = 0;
    for (let k = 1; k <= 2000; k++) {
      engine.step(1e-8);
      regs.push(reg(engine));
      peakAmps = Math.max(peakAmps, outputAmps(engine));
      rails.set(k, rail(engine, kind));
    }

    expect(regs.filter((r) => r !== 2)).toEqual([]);
    expect(peakAmps).toBeLessThanOrEqual(limit + 1e-12);
    expect(outputAmps(engine)).toBeCloseTo(limit, 12);
    expect((rails.get(1)! - settledRail) / drops[0]).toBeCloseTo(1, 4);
    expect((rails.get(10)! - settledRail) / drops[1]).toBeCloseTo(1, 4);
    expect((rails.get(2000)! - settledRail) / drops[2]).toBeCloseTo(1, 4);
  });

  it("runs 50 ms under the headless runner in CC from the load step, at exactly the limit", () => {
    const runner = new HeadlessRunner();
    runner.load(circuit(light));
    runner.run({ durationS: 0.05 });
    expect(reg(runner.engine)).toBe(0);

    runner.load(circuit(over));
    const { result, regs, peakAmps } = runHeadless(runner, 0.05);

    // The RC discharge (tau 0.5 ms, or 4.1 ms for the LM317 at 21% over)
    // takes 25 to 59 accepted steps from the runner's 10 ns start, none
    // rejected. The band pinned the step near 12 ns and hit the 20,000-step
    // cap within 0.2 ms. 100 leaves room for controller tuning.
    expect(result.acceptedSteps).toBeLessThanOrEqual(100);
    expect(result.rejectedSteps).toBeLessThanOrEqual(5);
    expect(result.hitStepCap).toBe(false);
    expect(regs.filter((r) => r !== 2)).toEqual([]);
    expect(peakAmps).toBeLessThanOrEqual(limit + 1e-12);
    expect(outputAmps(runner.engine)).toBeCloseTo(limit, 12);
    expect(rail(runner.engine, kind)).toBeCloseTo(settled, 3);
  });
});

describe("linear_reg a millionth over its limit, no capacitor", () => {
  // The in-solve entry fires once the REG candidate exceeds the limit by
  // 1e-12 + 1e-6 * iLimit. Just past that, the CC output
  // 5 / (1 + delta) = 5 - 5.0000e-6 V still sits inside the 5.001e-6 V band,
  // so from REG the solve converged on the CC candidate before testing it,
  // and from CC it tested it and handed back to REG: reg flipped every step.
  const circuit = (delta: number) => regulator("linear", 12, { vout: 5, vdropout: 2, iLimit: 1 }, resistive(5 / (1 + delta)));

  it.each([1.00001e-6, 1.0001e-6])("holds CC at exactly 1 A when the load is %s over (fixed and headless)", (delta) => {
    const engine = new SimEngine();
    engine.load(circuit(delta));
    const regs = stepFixed(engine, 200, 1e-4);
    expect(regs.filter((r) => r !== 2)).toEqual([]);
    expect(outputAmps(engine)).toBeCloseTo(1, 12);
    expect(rail(engine, "linear")).toBeCloseTo(4.999995, 6);

    const runner = new HeadlessRunner();
    runner.load(circuit(delta));
    const { result, regs: headlessRegs, peakAmps } = runHeadless(runner, 0.05);
    // A static load: 25 steps, as for the slight overloads above.
    expect(result.acceptedSteps).toBeLessThanOrEqual(50);
    expect(result.rejectedSteps).toBe(0);
    expect(result.hitStepCap).toBe(false);
    expect(headlessRegs.filter((r) => r !== 2)).toEqual([]);
    expect(peakAmps).toBeLessThanOrEqual(1 + 1e-12);
  });
});
