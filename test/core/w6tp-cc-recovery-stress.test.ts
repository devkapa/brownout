/**
 * Core-side counterpart for wave5-regulators.test.ts, "bench_psu - same-step CC
 * compliance recovery", the 1 Gohm load (gate 1, W6-T, case 3). Classic reads
 * `saveState().failureStress` before and after the first recovery step and asserts
 * the resistor's overload stress did not grow. The core publishes no stress, so
 * the claim is stated through what the stress determines on public readings: the
 * failure map and the latch time. The readout the original claim itself needs is
 * listed in the case's row.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Comp = SimCircuit["components"][number];

const wire = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });
const part = (id: string, kind: string, pins: string[], params: Record<string, number | string> = {}, catalogUid?: string): Comp =>
  ({ id, kind, pins: pins.map((p) => ({ id: p })), params, ...(catalogUid ? { catalogUid } : {}) }) as Comp;

const volts = (core: CoreEngine, comp: string, pin: string): number => {
  const net = core.getNetIdForPin(comp, pin);
  if (net === undefined) throw new Error(`no net for ${comp}.${pin}`);
  return core.getNetV()[net] ?? Number.NaN;
};

/** The failure key of a resistor overload, `failureKey("resistor_overload", id)` (src/core/failures.ts). */
const overloadKey = (id: string): string => `resistor_overload\u0000${id}\u0000`;

beforeAll(async () => {
  await initCore();
});

// ─── Case 3: a recovery into a near-open load must not charge the resistor's stress ─

describe("bench_psu CC recovery into 1 Gohm, from the failure map and the latch time", () => {
  // 10 V, 0.5 A limit, resistor rated 0.25 W (the catalog default). In CC into
  // 2 ohm: 0.5 A, V = 1 V, P = 0.5 W, so the overload stress grows at
  // P / 0.25 - 1 = 1 per second and latches at 1.0 s. Into 4 ohm the supply is
  // still in CC at 0.5 A (V = 2 V), P = 1.0 W, the rate is 3 and it latches at
  // 1/3 s. Into 1 Gohm P is about 1e-7 W: the rate is negative and the stress
  // bleeds at 1 per second down to 0. A one-step spike (a bare ideal CC stamp
  // would put 5e8 V across 1 Gohm for a step) would latch at once.
  const board = (resistance: number): SimCircuit => ({
    components: [
      part("psu", "bench_psu", ["pos", "neg"], { voltage: 10, iLimit: 0.5 }),
      part("rl", "resistor", ["a", "b"], { resistance }, "resistor"),
    ],
    wires: [wire("psu", "pos", "rl", "a"), wire("rl", "b", "psu", "neg")],
  });
  const expectLatchTime = (actual: number, analytic: number): void => {
    expect(Math.abs(actual - analytic)).toBeLessThanOrEqual(1e-9 + 1e-9 * analytic);
  };

  it("latches a steady 2 ohm CC at 1.0 s (the control: the stress machinery is live in this run)", () => {
    const core = new CoreEngine();
    core.load(board(2));
    core.advance(0.9);
    expect(core.getFailures()).toEqual({});
    core.advance(1.1);
    expect(Object.keys(core.getFailures())).toEqual([overloadKey("rl")]);
    expectLatchTime(core.getFailures()[overloadKey("rl")]!.since, 1.0);
    core.dispose();
  });

  it("latches a 4 ohm CC at 1/3 s", () => {
    const core = new CoreEngine();
    core.load(board(4));
    core.advance(0.5);
    expectLatchTime(core.getFailures()[overloadKey("rl")]!.since, 1 / 3);
    core.dispose();
  });

  it("bounds the first recovery step at 10 V and latches nothing afterwards", () => {
    const core = new CoreEngine();
    core.load(board(2));
    core.advance(0.4);
    expect(volts(core, "psu", "pos") - volts(core, "psu", "neg")).toBeCloseTo(1, 6);
    core.load(board(1e9));
    core.advance(0.4 + 1e-4);
    const vOut = volts(core, "psu", "pos") - volts(core, "psu", "neg");
    expect(vOut).toBeCloseTo(10, 6);
    expect(vOut).toBeLessThanOrEqual(10.000_01);
    expect(Math.abs(core.getElementI().psu ?? 0)).toBeLessThan(1e-7);
    expect(core.getFailures()).toEqual({});
    for (let t = 0.5; t <= 5.4 + 1e-9; t += 0.1) {
      core.advance(t);
      expect(core.getFailures()).toEqual({});
    }
    expect(core.takeLatchedFailures()).toEqual([]);
    core.dispose();
  });

  it("recovers the stress at 1 per second: 0.4 s of CC, 0.3 s of 1 Gohm, then CC latches 0.9 s later", () => {
    // S = 0.4 after the first CC leg, 0.1 after the easing leg, and it needs
    // 0.9 s more at rate 1 to reach the threshold 1: 0.4 + 0.3 + 0.9 = 1.6 s.
    const core = new CoreEngine();
    core.load(board(2));
    core.advance(0.4);
    core.load(board(1e9));
    core.advance(0.7);
    expect(core.getFailures()).toEqual({});
    core.load(board(2));
    core.advance(1.5);
    expect(core.getFailures()).toEqual({});
    core.advance(1.7);
    expectLatchTime(core.getFailures()[overloadKey("rl")]!.since, 1.6);
    core.dispose();
  });

  it("latches sooner when the load is made heavier after the first CC leg: 0.4 s at 2 ohm, then 4 ohm, at 0.6 s", () => {
    // S = 0.4 at the change, then rate 3: 0.4 + 3 x 0.2 = 1 at 0.6 s.
    const core = new CoreEngine();
    core.load(board(2));
    core.advance(0.4);
    core.load(board(4));
    core.advance(0.55);
    expect(core.getFailures()).toEqual({});
    core.advance(0.7);
    expectLatchTime(core.getFailures()[overloadKey("rl")]!.since, 0.6);
    core.dispose();
  });
});
