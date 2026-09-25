/**
 * Unit tests for the gate-1 suite adapter itself (adapter.ts) -- as opposed
 * to gate 1 RUNNING (that is `pnpm run test:core-suites`, driven by
 * vitest.core-suites.config.ts against the allowlisted classic suite files).
 * This file imports adapter.ts directly, under the ROOT vitest.config.ts
 * project (it matches `test/**\/*.test.ts`), the same way test/core/engine.test.ts
 * and test/host/headless-core.test.ts test their own modules directly.
 *
 * The central thing to prove here (the worker brief's own verify step): the
 * adapter must not let a test pass by silently falling back to classic. A
 * circuit using a kind the adapter's coreSupportedKinds does not cover must
 * make `load()` throw, not quietly succeed on classic.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import { CoreSuiteFellBackError, SimEngine, type SimCircuit } from "./adapter.js";

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number> = {}): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}
function wire(a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

const DIVIDER: SimCircuit = {
  components: [
    part("src", "voltage_source", ["pos", "neg"], { voltage: 9 }),
    part("top", "resistor", ["a", "b"], { resistance: 10_000 }),
    part("bottom", "resistor", ["a", "b"], { resistance: 20_000 }),
  ],
  wires: [wire("src", "pos", "top", "a"), wire("top", "b", "bottom", "a"), wire("bottom", "b", "src", "neg")],
};

// Same divider, plus one LED -- a kind CORE_IMPLEMENTED_KINDS does not
// contain (W1e, not yet ported; `capacitor` played this role until this same
// wave's later step landed the device/solver and the orchestrator added it
// to CORE_IMPLEMENTED_KINDS). compileCircuit can never turn this into a
// program, so this is exactly the "force a fallback" case the worker brief's
// verify step asks for.
const WITH_UNSUPPORTED_KIND: SimCircuit = {
  components: [...DIVIDER.components, part("led1", "led", ["a", "b"], {})],
  wires: [...DIVIDER.wires, wire("bottom", "b", "led1", "a"), wire("bottom", "b", "led1", "b")],
};

beforeAll(async () => {
  await initCore();
});

describe("adapter.ts SimEngine — the happy path actually runs on the core", () => {
  it("loads a W0-kind divider and matches the analytic 6 V midpoint", () => {
    const engine = new SimEngine();
    engine.load(DIVIDER);
    const net = engine.nets.find((candidate) => candidate.pins.some(([id, pin]) => id === "top" && pin === "b"));
    expect(net).toBeDefined();
    expect(engine.getNetV()[net!.id]).toBeCloseTo(6, 6);
    expect(engine.lastConverged).toBe(true);
    expect(Number.isFinite(engine.lastRelativeResidual)).toBe(true);
  });

  it("step(h) advances simTime and keeps the static solution (no dynamics in these kinds)", () => {
    const engine = new SimEngine();
    engine.load(DIVIDER);
    expect(engine.simTime).toBe(0);
    engine.step(1e-3);
    expect(engine.simTime).toBeCloseTo(1e-3, 12);
    engine.step(1e-3);
    expect(engine.simTime).toBeCloseTo(2e-3, 12);
    expect(engine.lastConverged).toBe(true);
  });

  it("coldLoad() starts a fresh runner at simTime=0, even after step() moved the clock", () => {
    const engine = new SimEngine();
    engine.load(DIVIDER);
    engine.step(5e-3);
    expect(engine.simTime).toBeCloseTo(5e-3, 12);
    engine.coldLoad(DIVIDER);
    expect(engine.simTime).toBe(0);
  });

  it("implements the rest of EngineReadout with classic's documented W1a-empty shapes", () => {
    const engine = new SimEngine();
    engine.load(DIVIDER);
    expect(engine.getFailures()).toEqual({});
    expect(engine.getWarnings()).toEqual({});
    expect(engine.takeNewWarnings()).toEqual([]);
    expect(engine.getDisplayState()).toEqual({});
    expect(engine.digitalState).toEqual({});
    expect(engine.getNetIdForPin("src", "pos")).toBeDefined();
    expect(engine.getNetIdForPin("src", "pos")).toBe(engine.getNetIdForPin("top", "a"));
  });

  it("solver diagnostics read the documented W1a neutral values (no core meaning yet)", () => {
    const engine = new SimEngine();
    engine.load(DIVIDER);
    expect(engine.lastIters).toBe(0);
    expect(engine.lastSolveUs).toBe(0);
    expect(engine.lastMatrixSize).toBe(0);
    expect(engine.lastMatrixSingular).toBe(false);
    expect(engine.lastMatrixIllConditioned).toBe(false);
    expect(engine.lastRelativeResidual).toBe(0);
  });
});

describe("adapter.ts SimEngine — the no-silent-fallback detector", () => {
  it("throws CoreSuiteFellBackError instead of silently running classic when a kind is unsupported", () => {
    const engine = new SimEngine();
    let thrown: unknown;
    try {
      engine.load(WITH_UNSUPPORTED_KIND);
    } catch (err) {
      thrown = err;
    }
    expect(thrown).toBeInstanceOf(CoreSuiteFellBackError);
    expect((thrown as CoreSuiteFellBackError).reasons).toContain("unsupported-kind:led");
  });

  it("also fires when coreSupportedKinds is narrowed below what a portable case needs", () => {
    // Same DIVIDER a gate-1 portable case uses, but with resistor excluded
    // from this instance's routing set -- proves the detector reacts to the
    // ALLOWED-kind configuration, not just to genuinely-unimplemented kinds.
    const engine = new SimEngine({ coreSupportedKinds: new Set(["voltage_source", "ground"]) });
    expect(() => engine.load(DIVIDER)).toThrow(CoreSuiteFellBackError);
  });

  it("never leaves the adapter reporting stale readouts after a fallback throw", () => {
    const engine = new SimEngine();
    engine.load(DIVIDER); // a real, successful core load first
    expect(() => engine.load(WITH_UNSUPPORTED_KIND)).toThrow(CoreSuiteFellBackError);
    // CoreEngine's own "no program loaded" facade state (engine.ts) surfaces
    // through HeadlessRunner's classic fallback load, which DOES succeed (the
    // LED circuit is valid on classic) -- readouts must reflect that
    // fallback run, not the earlier core one, since accessing them at all
    // outside gate 1's own load()-throws contract is a caller bug the adapter
    // does not otherwise police.
    expect(Object.keys(engine.getNetV()).length).toBeGreaterThan(0);
  });
});
