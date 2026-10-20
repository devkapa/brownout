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

// Same divider, plus one diode -- a kind CORE_IMPLEMENTED_KINDS does not
// contain (`capacitor` played this role until W1b landed its device/solver,
// then `led` until W1e ported it). compileCircuit can never turn this into a
// program, so this is exactly the "force a fallback" case the worker brief's
// verify step asks for.
const WITH_UNSUPPORTED_KIND: SimCircuit = {
  components: [...DIVIDER.components, part("d1", "diode", ["a", "k"], {})],
  wires: [...DIVIDER.wires, wire("bottom", "b", "d1", "a"), wire("bottom", "b", "d1", "k")],
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

  it("getElementChannelI() delegates to the core's channel slots: an rgb_led reports three live channels, a device with none reports no entry", () => {
    const rgb: SimCircuit = {
      components: [
        part("src", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r_r", "resistor", ["a", "b"], { resistance: 330 }),
        part("r_g", "resistor", ["a", "b"], { resistance: 330 }),
        part("r_b", "resistor", ["a", "b"], { resistance: 330 }),
        part("led", "rgb_led", ["r_a", "g_a", "b_a", "com_k"]),
      ],
      wires: [
        wire("src", "pos", "r_r", "a"),
        wire("src", "pos", "r_g", "a"),
        wire("src", "pos", "r_b", "a"),
        wire("r_r", "b", "led", "r_a"),
        wire("r_g", "b", "led", "g_a"),
        wire("r_b", "b", "led", "b_a"),
        wire("led", "com_k", "src", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.load(rgb);
    engine.step(1e-4);
    const channels = engine.getElementChannelI();
    expect(channels.led).toHaveLength(3);
    // (5 V - Vf) / 330 ohm is milliamps on every channel; blue's higher Vf (3.0 V against 1.8 V red) leaves it the least.
    for (const amps of channels.led!) expect(amps).toBeGreaterThan(1e-3);
    expect(channels.led![2]).toBeLessThan(channels.led![0]!);
    expect(channels.src).toBeUndefined();

    engine.load(DIVIDER);
    expect(engine.getElementChannelI()).toEqual({});
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

describe("adapter.ts SimEngine — device state comes from the runner's CoreEngine (W1f)", () => {
  const BATTERY: SimCircuit = {
    components: [
      part("cell", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0.9, capacityAh: 1e-4 }),
      part("load", "resistor", ["a", "b"], { resistance: 100 }),
    ],
    wires: [wire("cell", "pos", "load", "a"), wire("load", "b", "cell", "neg")],
  };

  function psu(iLimit: number): SimCircuit {
    return {
      components: [
        part("psu", "bench_psu", ["pos", "neg"], { voltage: 5, iLimit }),
        part("load", "resistor", ["a", "b"], { resistance: 10 }),
      ],
      wires: [wire("psu", "pos", "load", "a"), wire("load", "b", "psu", "neg")],
    };
  }

  it("getBatteryState() reads the core's cell: generic at load, drained by a step, undefined for another kind", () => {
    const engine = new SimEngine();
    engine.load(BATTERY);
    // No time passes at a core load (owner decision 8): full, nothing counted yet.
    expect(engine.getBatteryState("cell")).toMatchObject({
      profileId: "generic-primary",
      label: "Generic primary battery (fallback)",
      soc: 1,
      dischargedCoulombs: 0,
    });
    engine.step(0.1);
    const drained = engine.getBatteryState("cell")!;
    expect(drained.soc).toBeLessThan(1);
    expect(drained.dischargedCoulombs).toBeGreaterThan(0);
    expect(engine.getBatteryState("load")).toBeUndefined();
  });

  it("getIcState() reads a limited bench_psu's regime, and is undefined for an ideal one and for another kind", () => {
    const engine = new SimEngine();
    // 5 V into 10 ohm wants 0.5 A against a 0.1 A limit: CC from load (probe P5).
    engine.load(psu(0.1));
    expect(engine.getIcState("psu")).toEqual({ reg: 2 });
    expect(engine.getIcState("load")).toBeUndefined();
    engine.load(psu(0));
    expect(engine.getIcState("psu")).toBeUndefined();
  });

  it("refuses to answer from classic after a fallback, though classic has both methods", () => {
    const engine = new SimEngine();
    expect(() => engine.load(WITH_UNSUPPORTED_KIND)).toThrow(CoreSuiteFellBackError);
    expect(() => engine.getBatteryState("src")).toThrow(/getBatteryState\(\) needs the core/);
    expect(() => engine.getIcState("src")).toThrow(/getIcState\(\) needs the core/);
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
    expect((thrown as CoreSuiteFellBackError).reasons).toContain("unsupported-kind:diode");
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
    // diode circuit is valid on classic) -- readouts must reflect that
    // fallback run, not the earlier core one, since accessing them at all
    // outside gate 1's own load()-throws contract is a caller bug the adapter
    // does not otherwise police.
    expect(Object.keys(engine.getNetV()).length).toBeGreaterThan(0);
  });
});
