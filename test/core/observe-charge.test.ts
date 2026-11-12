/**
 * W2b: the observer and the charge through the facade (core/README.md,
 * "Acquisition", "Charge"): `acquire`/`takeSamples` and
 * `armCharge`/`takeCharge` keyed by net and component id; a load disarms
 * both; a snapshot carries them (they are engine state) and a restore brings
 * back what was not taken.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { initCore } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

function part(id: string, kind: string, pins: string[], params: Record<string, number | string> = {}): Component {
  return { id, kind, pins: pins.map((p) => ({ id: p })), params } as Component;
}

function wire(a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

/** 5 V through 1k into 1 uF: a closed form. */
function rc(): SimCircuit {
  return {
    components: [part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 1000 }), part("c1", "capacitor", ["a", "b"], { capacitance: 1e-6 })],
    wires: [wire("vs", "pos", "r1", "a"), wire("r1", "b", "c1", "a"), wire("c1", "b", "vs", "neg")],
  } as SimCircuit;
}

beforeAll(async () => {
  await initCore();
});

// The node shunt (`NODE_RSHUNT_G`, 1e-12 S) on the capacitor's node.
const RG = 1000 * 1e-12;
const V_INF = 5 / (1 + RG);
const TAU = 1e-3 / (1 + RG);
const vc = (t: number) => V_INF * (1 - Math.exp(-t / TAU));

function loaded(): { engine: CoreEngine; cap: string; top: string } {
  const engine = new CoreEngine();
  engine.load(rc());
  return { engine, cap: engine.getNetIdForPin("c1", "a")!, top: engine.getNetIdForPin("vs", "pos")! };
}

describe("CoreEngine.acquire / takeSamples", () => {
  it("samples nets on the grid, keyed by net id, and empties the queue on a take", () => {
    const { engine, cap, top } = loaded();
    expect(engine.takeSamples()).toBeNull();
    engine.acquire([cap, top], 0, 1e-4);
    engine.advance(1.05e-3);
    const s = engine.takeSamples()!;
    expect(Array.from(s.t)).toEqual(Array.from({ length: 11 }, (_, k) => k * 1e-4));
    s.t.forEach((t, k) => {
      expect(Math.abs(s.values[cap]![k]! - vc(t))).toBeLessThanOrEqual(1e-12 * 5);
      expect(s.values[top]![k]).toBe(5);
    });
    expect([s.overflowed, s.dropped, s.fallbacks[cap], s.maxSpan]).toEqual([false, 0, 0, 0]);
    expect(engine.takeSamples()!.t.length).toBe(0);
    engine.acquire([], 0, 0);
    expect(engine.takeSamples()).toBeNull();
  });

  it("refuses an unknown or repeated net and a bad grid", () => {
    const { engine, cap } = loaded();
    expect(() => engine.acquire(["nope"], 0, 1e-4)).toThrow(RangeError);
    expect(() => engine.acquire([cap, cap], 0, 1e-4)).toThrow(/twice/);
    expect(() => engine.acquire([cap], 0, -1)).toThrow(RangeError);
    expect(() => engine.acquire([cap], Number.NaN, 1)).toThrow(RangeError);
  });
});

describe("CoreEngine.armCharge / takeCharge", () => {
  it("integrates each component's current over the span, keyed by component id", () => {
    const { engine } = loaded();
    expect(engine.takeCharge()).toBeNull();
    engine.armCharge(["c1", "r1", "vs"]);
    engine.advance(2e-3);
    const c = engine.takeCharge()!;
    expect([c.t0, c.t1]).toEqual([0, 2e-3]);
    expect(Math.abs(c.charge.c1! - 1e-6 * vc(2e-3))).toBeLessThanOrEqual(1e-12 * 1e-6 * vc(2e-3));
    const hand = (5 * 2e-3 - V_INF * (2e-3 - TAU * (1 - Math.exp(-2e-3 / TAU)))) / 1000;
    expect(Math.abs(c.charge.r1! - hand)).toBeLessThanOrEqual(1e-12 * hand);
    // The source by KCL: the resistor's and the shunt's, with its sign.
    expect(Math.abs(Math.abs(c.charge.vs!) - (hand + 1e-12 * 5 * 2e-3))).toBeLessThanOrEqual(1e-12 * hand);
    expect(c.finalUpTo).toEqual({ c1: 2e-3, r1: 2e-3, vs: 2e-3 });
    expect(c.channelCharge).toEqual({});
    const next = engine.takeCharge()!;
    expect([next.t0, next.t1, next.charge.c1]).toEqual([2e-3, 2e-3, 0]);
    engine.armCharge([]);
    expect(engine.takeCharge()).toBeNull();
  });

  it("refuses an unknown or repeated component", () => {
    const { engine } = loaded();
    expect(() => engine.armCharge(["nope"])).toThrow(RangeError);
    expect(() => engine.armCharge(["r1", "r1"])).toThrow(/twice/);
  });
});

describe("the observer and the charge across loads and snapshots", () => {
  it("a load disarms both", () => {
    const { engine, cap } = loaded();
    engine.acquire([cap], 0, 1e-4);
    engine.armCharge(["c1"]);
    engine.advance(1e-3);
    engine.load(rc());
    expect(engine.takeSamples()).toBeNull();
    expect(engine.takeCharge()).toBeNull();
    engine.armCharge(["c1"]);
    engine.advance(1.5e-3);
    expect(engine.takeCharge()!.t0).toBe(1e-3);
  });

  it("a snapshot carries both, and a restore retakes what followed it", () => {
    const { engine, cap } = loaded();
    engine.acquire([cap], 0, 1e-4);
    engine.armCharge(["c1", "r1"]);
    engine.advance(0.6e-3);
    const snap = engine.saveState();
    engine.advance(1.3e-3);
    const first = [engine.takeSamples(), engine.takeCharge()];
    // Disarmed after the snapshot: the restore arms them again.
    engine.acquire([], 0, 0);
    engine.armCharge([]);
    engine.restoreState(snap);
    engine.advance(1.3e-3);
    expect([engine.takeSamples(), engine.takeCharge()]).toEqual(first);
    snap.dispose();
  });
});
