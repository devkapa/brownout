/**
 * Ruling R94 (a) on fixture 26, the 74HC14 RC oscillator clocking a 74HC74,
 * its timing capacitor cut from 100 nF to oscillate fast enough to stress the
 * event cap. With a host clock (`bo_engine_advance_for`) only the progress
 * floor governs; without one (`bo_engine_advance`) the cap is 20M events x
 * max(1, the host step's span in simulated seconds), 60M for advance(3).
 * - 100 pF: 24.75M events in 3 s, under the cap: both complete.
 * - 10 pF: about ten times that, past the cap: with a clock it completes; without
 *   one it declines -7 at the same event count on every run.
 * Slow (minutes): `pnpm run test:slow`, in the gate. The fast form, 2 ms against a
 * cap scaled to 100,000 events, is cargo's
 * `with_a_clock_only_the_floor_governs_and_without_one_the_scaled_cap_declines_deterministically`.
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { coreCall, initCore, readCoreErrorMessage } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

const FIXTURE = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../fixtures/core/26-74hc74-divide-by-two.sim.json");

type AdvanceExports = {
  bo_engine_advance(h: number, t: number): number;
  bo_engine_advance_for(h: number, t: number, ms: number): number;
  bo_engine_stat(h: number, i: number): number;
  bo_engine_time(h: number): number;
};

function board(capacitanceF: number): { engine: CoreEngine; h: number } {
  const circuit = JSON.parse(fs.readFileSync(FIXTURE, "utf8")) as SimCircuit;
  const c1 = circuit.components.find((c) => c.id === "c1")!;
  c1.params = { ...c1.params, capacitance: capacitanceF };
  const engine = new CoreEngine();
  engine.load(circuit);
  return { engine, h: (engine as unknown as { _handle: number })._handle };
}

/** Plain advance (no clock): the code, the events, and the time reached. */
function plain(capacitanceF: number): { code: number; events: number; t: number; message: string } {
  const { engine, h } = board(capacitanceF);
  try {
    return coreCall((x) => {
      const ex = x as unknown as AdvanceExports;
      const code = ex.bo_engine_advance(h, 3);
      return { code, events: ex.bo_engine_stat(h, 4), t: ex.bo_engine_time(h), message: code === 0 ? "" : readCoreErrorMessage(x) };
    });
  } finally {
    engine.dispose();
  }
}

/** Budgeted advance on the loader's clock (performance.now), 16 ms a call. */
function budgeted(capacitanceF: number): { code: number; calls: number } {
  const { engine, h } = board(capacitanceF);
  try {
    let calls = 0;
    let code = 1;
    while (code === 1) {
      code = coreCall((x) => (x as unknown as AdvanceExports).bo_engine_advance_for(h, 3, 16));
      calls += 1;
    }
    return { code, calls };
  } finally {
    engine.dispose();
  }
}

beforeAll(async () => {
  await initCore();
});

describe("R94 (a): the event cap without a clock, the floor with one (fixture 26)", () => {
  it("at 100 pF advance(3) completes both with a clock and without one", () => {
    const p = plain(100e-12);
    expect(p.code, p.message).toBe(0);
    expect(p.events).toBeGreaterThan(20e6);
    expect(p.events).toBeLessThan(60e6);
    const b = budgeted(100e-12);
    expect(b.code).toBe(0);
    expect(b.calls).toBeGreaterThan(1);
  }, 300_000);

  it("at 10 pF it completes with a clock, and without one declines -7 at the same event count every run", () => {
    expect(budgeted(10e-12).code).toBe(0);
    const first = plain(10e-12);
    const second = plain(10e-12);
    expect(first.code, first.message).toBe(-7);
    expect(first.message).toMatch(/took 6000000\d events/);
    expect(second).toEqual(first);
    expect(first.t).toBeLessThan(3);
  }, 600_000);
});
