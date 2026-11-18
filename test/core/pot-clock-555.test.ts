/**
 * P0-SG (coordinator's addition to the pot work): the Ben Eater clock module, a 555 astable whose timing resistor is
 * a 1 Mohm potentiometer (1 kohm + the pot as a rheostat + 1 uF, ideal 5 V), at 0 %, 50 %, 99 % and 100 % with the
 * position moved live from 50 %. At 100 % the cw segment sits at classic's end-stop clamp (`potentiometerSegments`:
 * 1 ohm), which the compile shares, so both engines stamp the same resistance; the low phase is then 7.7 us.
 *
 * Each period, the mean over the run and the worst single, is held against the hand formula (charge through
 * R1 + R2 by ln 2, discharge through R2 + R1 || 10 ohm toward DIS's Thevenin), within 0.01 tau_w0 (W0's band at each
 * threshold over the timing node's slope there). Measured: within 3e-5 tau_w0 at every position. Classic, stepped
 * at a fixed host step, splits a step once at a crossing, so a low phase shorter than its step holds for the rest of
 * that step: at 100 % its period is 1.068 ms at 100 us steps (+52 %) and 0.739 ms at 10 us (+5.3 %), the slow clock
 * the owner saw; at 99 % +0.48 % and +0.048 % (ship-logs/p0-sg/clock555.log).
 */
import { beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";
import { buildNets } from "../../src/sim/engine/graph.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];
const part = (id: string, kind: string, pins: string[], params: Record<string, number | string> = {}, catalogUid?: string): Component => ({
  id,
  kind,
  pins: pins.map((p) => ({ id: p })),
  params,
  ...(catalogUid ? { catalogUid } : {}),
});
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

function clock(position: number): SimCircuit {
  return {
    components: [
      part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("u1", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
      part("pot", "potentiometer", ["cw", "wiper", "ccw"], { rTotal: 1e6, position }, "potentiometer"),
      part("ct", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
      part("cv", "capacitor", ["a", "b"], { capacitance: 1e-8 }),
      part("rl", "resistor", ["a", "b"], { resistance: 10_000 }),
    ],
    wires: [
      wire("u1", "1", "gnd", "gnd"),
      wire("u1", "1", "vcc", "neg"),
      wire("u1", "8", "vcc", "pos"),
      wire("u1", "4", "u1", "8"),
      wire("r1", "a", "vcc", "pos"),
      wire("r1", "b", "u1", "7"),
      wire("pot", "cw", "u1", "7"),
      wire("pot", "wiper", "ct", "a"),
      wire("pot", "ccw", "pot", "wiper"),
      wire("u1", "2", "ct", "a"),
      wire("u1", "6", "ct", "a"),
      wire("ct", "b", "u1", "1"),
      wire("u1", "5", "cv", "a"),
      wire("cv", "b", "u1", "1"),
      wire("u1", "3", "rl", "a"),
      wire("rl", "b", "u1", "1"),
    ],
  };
}

/** The hand period and tau_w0 summed over its two crossings. */
function hand(position: number): { period: number; tauW0: number } {
  const r2 = Math.max(1, 1e6 * (1 - position));
  const c = 1e-6;
  const vd = (5 * 10) / 1010;
  const rth = r2 + (1000 * 10) / 1010;
  const high = (1000 + r2) * c * Math.LN2;
  const low = rth * c * Math.log((10 / 3 - vd) / (5 / 3 - vd));
  const w0 = (v: number): number => 1e-4 + 1e-3 * v;
  const tauW0 = w0(10 / 3) / ((5 / 3) / ((1000 + r2) * c)) + w0(5 / 3) / ((5 / 3 - vd) / (rth * c));
  return { period: high + low, tauW0 };
}

const OUT = buildNets(clock(0.5) as never).find((n) => n.pins.some((k) => k[0] === "u1" && k[1] === "3"))!.id;

/** OUT's rising edges, each at the event that flips it, after two settling periods. */
function rises(engine: CoreEngine, count: number): number[] {
  const out: number[] = [];
  let high = (engine.getNetV()[OUT] ?? 0) > 2.5;
  while (out.length < count + 2) {
    const t = engine.nextEventTime();
    engine.advance(Number.isFinite(t) ? Math.max(t, engine.simTime + 1e-12) : engine.simTime + 1e-3);
    const now = (engine.getNetV()[OUT] ?? 0) > 2.5;
    if (now && !high) out.push(engine.simTime);
    high = now;
  }
  return out.slice(2);
}

describe("P0-SG: a 555 clock timed by a potentiometer, end stop included", () => {
  beforeAll(async () => {
    await initCore();
  });

  for (const position of [0, 0.5, 0.99, 1]) {
    it(`keeps the hand period within 0.01 tau_w0 at ${position * 100} %, loaded there or moved there live`, () => {
      const { period, tauW0 } = hand(position);
      const n = position >= 0.99 ? 100 : 4;
      const loaded = new CoreEngine();
      loaded.load(clock(position), CORE_IMPLEMENTED_KINDS);
      const moved = new CoreEngine();
      moved.load(clock(0.5), CORE_IMPLEMENTED_KINDS);
      moved.setParams([{ componentId: "pot", params: { position } }]);
      for (const engine of [loaded, moved]) {
        const r = rises(engine, n);
        const periods = r.slice(1).map((t, i) => t - r[i]!);
        const mean = (r[r.length - 1]! - r[0]!) / (r.length - 1);
        expect(Math.abs(mean - period) / tauW0, `mean ${mean} vs ${period}`).toBeLessThan(0.01);
        for (const p of periods) expect(Math.abs(p - period) / tauW0, `single ${p}`).toBeLessThan(0.01);
      }
      expect(loaded.getFailures()).toEqual(moved.getFailures());
    });
  }
});
