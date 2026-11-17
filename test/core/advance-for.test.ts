/**
 * The facade's yielding advance (ruling R86; W2h): `CoreEngine.advanceFor`
 * over `bo_engine_advance_for`, on the loader's clock import. A yield is not
 * an error: the readouts stand at the last completed instant and the same
 * target called again goes on, so a run split by yields is the unsplit run
 * bit for bit. A failed advance (-4, -7) is what `advance` makes of it,
 * never a yield. A snapshot or a reload between yields behaves as W2a and
 * W2c say. A core without the export falls back to `advance`, warning once.
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { afterEach, beforeAll, describe, expect, it, vi } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { __installCoreForTest, coreCall, initCore, type CoreExports } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

const FIXTURES = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../fixtures/core");
const fixture = (name: string): SimCircuit => JSON.parse(fs.readFileSync(path.join(FIXTURES, name), "utf8")) as SimCircuit;
const ASTABLE = "31-ne555-astable-led.sim.json";

let realExports: CoreExports;

beforeAll(async () => {
  await initCore();
  realExports = coreCall((x) => x);
});

afterEach(() => {
  // A test that installed patched exports puts the real module back (a new
  // generation: engines from the patched one are stale, as they should be).
  if (coreCall((x) => x) !== realExports) __installCoreForTest(realExports);
  vi.restoreAllMocks();
});

/** Calls `advanceFor(target, budget)` until it reaches, counting the yields. */
function runSplit(engine: CoreEngine, target: number, budgetMs: number): number {
  let yields = 0;
  for (;;) {
    const outcome = engine.advanceFor(target, budgetMs);
    if (outcome === "reached") return yields;
    expect(outcome).toBe("yielded");
    yields += 1;
    if (yields > 1_000_000) throw new Error("advanceFor made no progress");
  }
}

function readouts(engine: CoreEngine): unknown {
  return {
    simTime: engine.simTime,
    netV: engine.getNetV(),
    elementI: engine.getElementI(),
    digital: engine.digitalState,
    channels: engine.getElementChannelI(),
  };
}

describe("CoreEngine.advanceFor", () => {
  it("reaches the target within a generous budget, as advance does", () => {
    const a = new CoreEngine();
    const b = new CoreEngine();
    a.load(fixture(ASTABLE));
    b.load(fixture(ASTABLE));
    expect(a.advanceFor(0.01, 10_000)).toBe("reached");
    b.advance(0.01);
    expect(a.simTime).toBe(0.01);
    expect(a.lastConverged).toBe(true);
    expect(readouts(a)).toEqual(readouts(b));
  });

  it("yields short of the target on a tiny budget, readouts at the instant reached, and the same target goes on bit for bit", () => {
    const split = new CoreEngine();
    const whole = new CoreEngine();
    split.load(fixture(ASTABLE));
    whole.load(fixture(ASTABLE));
    const cap = split.getNetIdForPin("c1", "a") ?? Object.keys(split.getNetV()).find((id) => id !== "gnd")!;
    split.acquire([cap], 0, 1e-4);
    whole.acquire([cap], 0, 1e-4);

    const first = split.advanceFor(0.2, 1e-6);
    expect(first).toBe("yielded");
    expect(split.lastConverged).toBe(true);
    expect(split.simTime).toBeGreaterThanOrEqual(0);
    expect(split.simTime).toBeLessThan(0.2);
    const yields = 1 + runSplit(split, 0.2, 1e-6);
    expect(yields).toBeGreaterThan(1);
    whole.advance(0.2);
    expect(readouts(split)).toEqual(readouts(whole));
    const s = split.takeSamples()!;
    const w = whole.takeSamples()!;
    expect(Array.from(s.t)).toEqual(Array.from(w.t));
    expect(Array.from(s.values[cap]!)).toEqual(Array.from(w.values[cap]!));
  });

  it("a yield publishes the readouts of the instant it stopped at", () => {
    const engine = new CoreEngine();
    engine.load(fixture(ASTABLE));
    const before = engine.simTime;
    let outcome = engine.advanceFor(0.2, 1e-6);
    while (outcome === "yielded" && engine.simTime === before) outcome = engine.advanceFor(0.2, 1e-6);
    expect(engine.simTime).toBeGreaterThan(before);
    const reached = coreCall((x) => x.bo_engine_time((engine as unknown as { _handle: number })._handle));
    expect(engine.simTime).toBe(reached);
    const probe = new CoreEngine();
    probe.load(fixture(ASTABLE));
    probe.advance(engine.simTime);
    expect(engine.getNetV()).toEqual(probe.getNetV());
  });

  it("refuses a non-finite target or budget, and a target in the past, as advance does", () => {
    const engine = new CoreEngine();
    engine.load(fixture(ASTABLE));
    engine.advance(0.01);
    expect(() => engine.advanceFor(Number.NaN, 16)).toThrow(RangeError);
    expect(() => engine.advanceFor(0.02, Number.NaN)).toThrow(RangeError);
    expect(() => engine.advanceFor(0.001, 16)).toThrow(RangeError);
  });

  it("a snapshot taken between yields restores to a run that goes on bit for bit", () => {
    const engine = new CoreEngine();
    engine.load(fixture(ASTABLE));
    expect(engine.advanceFor(0.1, 1e-6)).toBe("yielded");
    const snap = engine.saveState();
    const at = engine.simTime;
    runSplit(engine, 0.1, 1e-6);
    const first = readouts(engine);
    engine.restoreState(snap);
    expect(engine.simTime).toBe(at);
    runSplit(engine, 0.1, 50);
    expect(readouts(engine)).toEqual(first);
    snap.dispose();
  });

  it("a carrying reload between yields keeps the clock and starts a new host step; a cold load starts at 0", () => {
    const engine = new CoreEngine();
    engine.load(fixture(ASTABLE));
    const cap = engine.getNetIdForPin("c1", "a") ?? Object.keys(engine.getNetV()).find((id) => id !== "gnd")!;
    engine.acquire([cap], 0, 1e-4);
    let outcome = engine.advanceFor(0.1, 1e-6);
    while (outcome === "yielded" && engine.simTime === 0) outcome = engine.advanceFor(0.1, 1e-6);
    expect(outcome).toBe("yielded");
    const at = engine.simTime;
    engine.load(fixture(ASTABLE));
    expect(engine.simTime).toBe(at);
    // W2c: a load disarms the observer; the host re-arms.
    expect(engine.takeSamples()).toBeNull();
    runSplit(engine, 0.1, 1e-6);
    expect(engine.simTime).toBe(0.1);
    expect(engine.lastConverged).toBe(true);

    engine.coldLoad(fixture(ASTABLE));
    expect(engine.simTime).toBe(0);
    runSplit(engine, 0.05, 1e-6);
    expect(engine.simTime).toBe(0.05);
  });

  it.each([
    [-4, "solve"],
    [-7, "no progress"],
  ])("a %i is a failed advance as advance makes it, never a yield", (code) => {
    let fail = false;
    const patched = { ...realExports } as CoreExports & { bo_engine_advance_for(h: number, t: number, ms: number): number };
    const real = (realExports as unknown as { bo_engine_advance_for(h: number, t: number, ms: number): number }).bo_engine_advance_for;
    patched.bo_engine_advance_for = (h: number, t: number, ms: number) => {
      if (!fail) return real(h, t, ms);
      // Stop the real engine at an instant short of t, then report the failure.
      real(h, t, 1e-6);
      return code;
    };
    __installCoreForTest(patched);
    const engine = new CoreEngine();
    engine.load(fixture(ASTABLE));
    expect(engine.advanceFor(0.001, 10_000)).toBe("reached");
    const good = engine.getNetV();
    fail = true;
    const outcome = engine.advanceFor(0.2, 10_000);
    expect(outcome).toBe("failed");
    expect(engine.lastConverged).toBe(false);
    expect(engine.simTime).toBe(realExports.bo_engine_time((engine as unknown as { _handle: number })._handle));
    expect(engine.simTime).toBeLessThan(0.2);
    // The readouts stay those of the last successful call, as advance's.
    expect(engine.getNetV()).toEqual(good);
    expect(engine.lastAdvanceFailure?.code).toBe(code);
    fail = false;
    expect(engine.advanceFor(0.2, 10_000)).toBe("reached");
    expect(engine.lastConverged).toBe(true);
    expect(engine.lastAdvanceFailure).toBeNull();
  });

  it("a core without the export falls back to advance and warns once", () => {
    const patched = { ...realExports } as Record<string, unknown>;
    delete patched.bo_engine_advance_for;
    __installCoreForTest(patched as unknown as CoreExports);
    const warn = vi.spyOn(console, "warn").mockImplementation(() => {});
    const engine = new CoreEngine();
    engine.load(fixture(ASTABLE));
    expect(engine.advanceFor(0.01, 1e-6)).toBe("reached");
    expect(engine.simTime).toBe(0.01);
    expect(engine.advanceFor(0.02, 1e-6)).toBe("reached");
    expect(warn).toHaveBeenCalledTimes(1);
    expect(String(warn.mock.calls[0]![0])).toMatch(/advance_for/);
  });
});
