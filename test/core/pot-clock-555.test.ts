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
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";
import { buildNets } from "../../src/sim/engine/graph.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

const HAND_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../fixtures/core-hand-judged");
/** The repository's own circuit for each position (test/fixtures/core-hand-judged), so the fixtures and the hand check are one circuit. */
function clock(position: number): SimCircuit {
  const label = Math.round(position * 100);
  return JSON.parse(fs.readFileSync(path.join(HAND_DIR, `555-pot-timing-${label}pct.sim.json`), "utf8")) as SimCircuit;
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
