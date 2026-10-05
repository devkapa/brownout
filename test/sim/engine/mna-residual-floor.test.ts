/**
 * The linear-solve residual gate on a circuit coming to rest.
 *
 * Switch an LM317 driving an LED to a 0 V supply and its off-state solution
 * decays toward exact zero, about 5e-15 per step: the LED's linearised
 * companion keeps a sliver of its last current. Some 20 steps in the values
 * are subnormal (under 2.2e-308), where doubles round to a fixed 5e-324 grid
 * instead of to a relative error. The solve's backward error was measured as
 * the largest |G x - b| over |G|max |x|max + |b|max, so one grid step against a
 * 9e-319 scale read 5.4e-6, past the engine's 1e-8 gate, and the step was
 * rejected. A rejected step commits nothing, so every later step solved the
 * same state and was rejected too: on the dense backend, 16 of 46 load
 * resistors from 50 to 500 ohm stopped converging for good. The sparse
 * backend computes the same ratio but sums the residual in another order, and
 * happened to round all 46 to exactly 0. Both now measure against a scale of
 * at least RESIDUAL_SCALE_FLOOR (1e-300).
 */

import { afterEach, describe, expect, it } from "vitest";
import { setLinearSystemBackendForTests } from "../../../src/sim/engine/linear-system.js";
import { MNA } from "../../../src/sim/engine/mna.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { SparseMNA } from "../../../src/sim/engine/sparse-mna.js";

afterEach(() => {
  setLinearSystemBackendForTests(null);
});

type Wire = SimCircuit["wires"][number];
const wire = (a: string, ap: string, b: string, bp: string): Wire => (
  { from_component: a, from_pin: ap, to_component: b, to_pin: bp }
);

/** An LM317 set to 5 V by 240/720 ohm, driving a red LED through `resistance`. */
function lm317WithLed(supplyVolts: number, resistance: number): SimCircuit {
  return {
    components: [
      { id: "s", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: supplyVolts } },
      { id: "u", kind: "lm317", pins: [{ id: "in" }, { id: "adj" }, { id: "out" }], params: { vref: 1.25, vdropout: 2, iLimit: 1.5 } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 240 } },
      { id: "r2", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 720 } },
      { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
      { id: "d", kind: "led", pins: [{ id: "anode" }, { id: "cathode" }], params: { vf: 2.0 } },
    ],
    wires: [
      wire("s", "pos", "u", "in"),
      wire("u", "out", "r1", "a"),
      wire("r1", "b", "u", "adj"),
      wire("u", "adj", "r2", "a"),
      wire("r2", "b", "s", "neg"),
      wire("u", "out", "rl", "a"),
      wire("rl", "b", "d", "anode"),
      wire("d", "cathode", "s", "neg"),
    ],
  };
}

const LOADS = Array.from({ length: 46 }, (_, k) => 50 + 10 * k);

/**
 * Run each load at 9 V, drop the supply to 0 V and step on. Returns the loads
 * with any rejected step after the drop, and the largest |V| left on any net.
 */
function brownoutSweep(): { failing: number[]; worstResidualVolts: number } {
  const failing: number[] = [];
  let worstResidualVolts = 0;
  for (const resistance of LOADS) {
    const engine = new SimEngine();
    engine.load(lm317WithLed(9, resistance));
    for (let k = 0; k < 10; k++) engine.step(1e-5);
    engine.load(lm317WithLed(0, resistance));
    let rejected = engine.lastConverged ? 0 : 1;
    for (let k = 0; k < 100; k++) {
      engine.step(1e-5);
      if (!engine.lastConverged) rejected += 1;
    }
    if (rejected > 0) failing.push(resistance);
    for (const v of Object.values(engine.getNetV())) worstResidualVolts = Math.max(worstResidualVolts, Math.abs(v));
  }
  return { failing, worstResidualVolts };
}

describe("an LM317 driving an LED keeps converging after its supply drops to 0 V", () => {
  it("on the dense backend, for every load from 50 to 500 ohm", () => {
    // Before the fix: 70, 90, 120, 170, 180, 190, 200, 220, 240, 310, 330,
    // 340, 350, 370, 420 and 470 ohm were rejected on every step from about
    // the 20th on.
    setLinearSystemBackendForTests("dense");
    const { failing, worstResidualVolts } = brownoutSweep();
    expect(failing).toEqual([]);
    // The off state finishes its decay to exactly 0 V.
    expect(worstResidualVolts).toBe(0);
  });

  it("on the sparse backend, for every load from 50 to 500 ohm", () => {
    setLinearSystemBackendForTests("sparse");
    const { failing, worstResidualVolts } = brownoutSweep();
    expect(failing).toEqual([]);
    expect(worstResidualVolts).toBe(0);
  });
});

/**
 * The 70 ohm circuit's system at its first rejected step: nets in, adj, out
 * and anode, then the source and regulator branch rows. The source is at
 * 0 V and the regulator off; the only drive left is the LED companion's
 * 8.9e-322 A at the anode. The two backends sum the residual in different
 * orders, so which drive leaves a rounding step varies: the sweep's own
 * value fails the dense gate, and a slightly larger one the sparse gate.
 */
const STALLED_G: number[][] = [
  [1e-12, 0, 0, 0, 1, -1],
  [0, 0.01845238095338095, -0.004166666666666667, -0.014285714285714285, 0, 1],
  [0, -0.004166666666666667, 0.005565555556555556, 0, 0, 0],
  [0, -0.014285714285714285, 0, 0.014285714287714291, 0, 0],
  [1, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 1],
];

describe("the stalled 70 ohm system, solved directly", () => {
  // Before the fix: dense 5.39e-6 at 8.9e-322 A and 1.50e-7 at 3.2e-320 A;
  // sparse 9.75e-7 at 4.94e-321 A. Every other pair read exactly 0.
  const backends = [
    ["dense", () => new MNA(6)],
    ["sparse", () => new SparseMNA(6)],
  ] as const;
  const anodeDrives = [8.9e-322, 4.94e-321, 3.2e-320];
  it.each(backends.flatMap(([label, make]) => anodeDrives.map((drive) => [label, drive, make] as const)))(
    "passes the 1e-8 residual gate on the %s backend with %s A at the anode",
    (_label, drive, make) => {
      const mna = make();
      STALLED_G.forEach((row, i) => row.forEach((value, j) => {
        if (value !== 0) mna.add(i, j, value);
      }));
      mna.addB(3, drive);
      const x = mna.solve();

      expect(mna.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(1e-8);
      expect(mna.lastSolveInfo.singular).toBe(false);
      expect(mna.lastSolveInfo.nonFinite).toBe(false);
      // The solution is the subnormal decay itself, not a zeroed variable.
      expect(Math.max(...[...x].map(Math.abs))).toBeGreaterThan(0);
      expect(Math.max(...[...x].map(Math.abs))).toBeLessThan(1e-300);
    },
  );
});

describe("guards: the floor hides no bad solve", () => {
  it.each([
    [5, 3],
    [1e-20, 2e-20],
  ])("still rejects ideal sources of %s V and %s V in parallel as singular", (a, b) => {
    const circuit: SimCircuit = {
      components: [
        { id: "a", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: a } },
        { id: "b", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: b } },
      ],
      wires: [wire("a", "pos", "b", "pos"), wire("a", "neg", "b", "neg")],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(1e-4);
    expect(engine.lastConverged).toBe(false);
    expect(engine.lastMatrixSingular).toBe(true);
  });

  it.each([1, 1e-300, 1e-305])("still fails the gate on an element-growth solve scaled by %s (dense)", (scale) => {
    // Wilkinson's matrix: 1 on the diagonal, -1 below it, 1 in the last
    // column. Partial pivoting doubles the last column at every step, so at
    // n = 60 the growth is 2^59 and the solution is rounding noise. Scaled
    // down to 1e-300 its scale is still at the floor, so it is judged as
    // before. At 1e-305 its scale is under the floor and its error is
    // measured against the floor itself, which still rejects it (2.0e-5):
    // a floor set much higher would let it through.
    const n = 60;
    const mna = new MNA(n);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < i; j++) mna.add(i, j, -scale);
      mna.add(i, i, scale);
      if (i < n - 1) mna.add(i, n - 1, scale);
      mna.addB(i, scale * ((i % 3) - 1));
    }
    mna.solve();
    expect(mna.lastSolveInfo.singular).toBe(false);
    expect(mna.lastSolveInfo.relativeResidual).toBeGreaterThan(1e-8);
  });

  it.each([
    ["dense", () => new MNA(2)],
    ["sparse", () => new SparseMNA(2)],
  ] as const)("still reports a nearly singular matrix as ill-conditioned (%s)", (_label, make) => {
    const mna = make();
    mna.add(0, 0, 1);
    mna.add(0, 1, 1);
    mna.add(1, 0, 1);
    mna.add(1, 1, 1 + 1e-13);
    mna.addB(0, 1);
    mna.addB(1, 2);
    mna.solve();
    expect(mna.lastSolveInfo.singular).toBe(false);
    expect(mna.lastSolveInfo.illConditioned).toBe(true);
  });

  it.each([1, 1e-200])("measures a solve above the floor exactly as before (dense, scale %s)", (scale) => {
    // The historical ratio, recomputed in the backend's own summation order.
    const n = 5;
    const G = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (
      i === j ? 3 + i : ((i * 7 + j * 3) % 5) / 10 - 0.2
    ) * scale));
    const b = Array.from({ length: n }, (_, i) => (i - 2) * scale);
    const mna = new MNA(n);
    G.forEach((row, i) => row.forEach((value, j) => mna.add(i, j, value)));
    b.forEach((value, i) => mna.addB(i, value));
    const x = mna.solve();

    let maxMatrix = 0;
    let maxX = 0;
    let maxRhs = 0;
    let maxResidual = 0;
    for (let col = 0; col < n; col++) maxX = Math.max(maxX, Math.abs(x[col]!));
    for (let row = 0; row < n; row++) {
      let ax = 0;
      for (let col = 0; col < n; col++) {
        const a = mna.G[row * n + col]!;
        maxMatrix = Math.max(maxMatrix, Math.abs(a));
        ax += a * x[col]!;
      }
      maxRhs = Math.max(maxRhs, Math.abs(b[row]!));
      maxResidual = Math.max(maxResidual, Math.abs(ax - b[row]!));
    }
    const scaleNow = maxMatrix * maxX + maxRhs;
    // Above RESIDUAL_SCALE_FLOOR (1e-300), so the floor plays no part.
    expect(scaleNow).toBeGreaterThan(1e-300);
    expect(mna.lastSolveInfo.relativeResidual).toBe(maxResidual / scaleNow);
  });
});
