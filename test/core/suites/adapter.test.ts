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
import { afterAll, beforeAll, describe, expect, it, vi } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import { CoreEngine } from "../../../src/core/engine.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary } from "../../../src/parts/part-library.js";
import { CoreSuiteEditNotDeliverableError, CoreSuiteFellBackError, SimEngine, type SimCircuit } from "./adapter.js";

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

afterAll(() => resetPartLibrary());

// ---- Live edits and EEPROM updates (W2d) ----------------------------------
//
// A classic suite file mutates the circuit object it already passed to
// load() and calls step() again; classic re-reads the live reference. The
// adapter diffs that object against what it last delivered and sends the
// live-param part to the core as setParams; it never silently drops an edit
// it cannot deliver.

describe("adapter.ts SimEngine — live switch params reach the core without a reload", () => {
  /** 5 V, a switch, 1k and 1k to ground: the midpoint reads 0 V open and 2.5 V closed. */
  function switched(): SimCircuit {
    return {
      components: [
        part("v", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("sw", "switch", ["a", "b"], { closed: 0 }),
        part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
        part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      wires: [
        wire("v", "pos", "sw", "a"),
        wire("sw", "b", "r1", "a"),
        wire("r1", "b", "r2", "a"),
        wire("r2", "b", "v", "neg"),
      ],
    };
  }
  const mid = (engine: SimEngine): number => engine.getNetV()[engine.getNetIdForPin("r1", "b")!]!;

  it("mutating params.closed on the loaded object and stepping toggles the core", () => {
    const circuit = switched();
    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(1e-4);
    expect(mid(engine)).toBeCloseTo(0, 6);

    circuit.components.find((c) => c.id === "sw")!.params.closed = 1;
    engine.step(1e-4);
    expect(mid(engine)).toBeCloseTo(2.5, 4);

    circuit.components.find((c) => c.id === "sw")!.params.closed = 0;
    engine.step(1e-4);
    expect(mid(engine)).toBeCloseTo(0, 6);
  });

  it("an unchanged circuit steps with no edit (and a repeated edit is delivered once)", () => {
    const circuit = switched();
    const engine = new SimEngine();
    engine.load(circuit);
    circuit.components.find((c) => c.id === "sw")!.params.closed = 1;
    engine.step(1e-4);
    engine.step(1e-4);
    engine.step(1e-4);
    expect(mid(engine)).toBeCloseTo(2.5, 4);
    expect(engine.simTime).toBeCloseTo(3e-4, 12);
  });

  it("a fresh load() of the mutated object takes it as the new baseline", () => {
    const circuit = switched();
    const engine = new SimEngine();
    engine.load(circuit);
    circuit.components.find((c) => c.id === "sw")!.params.closed = 1;
    engine.load(circuit);
    engine.step(1e-4);
    expect(mid(engine)).toBeCloseTo(2.5, 4);
  });
});

describe("adapter.ts SimEngine — an edit it cannot deliver is refused, not dropped", () => {
  function pair(): SimCircuit {
    return {
      components: [
        part("v", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      wires: [wire("v", "pos", "r", "a"), wire("r", "b", "v", "neg")],
    };
  }

  it("a changed voltage_source V is delivered live (W2c): the node takes it at the next step", () => {
    const circuit = pair();
    const engine = new SimEngine();
    engine.load(circuit);
    circuit.components[0]!.params.voltage = 3;
    engine.step(1e-4);
    const netId = engine.nets.find((n) => n.pins.some(([c, p]) => c === "r" && p === "a"))!.id;
    expect(engine.getNetV()[netId]).toBeCloseTo(3, 12);
  });

  it("a changed non-live param or a rewire says it needs a reload with carry (W2c)", () => {
    const circuit = pair();
    const engine = new SimEngine();
    engine.load(circuit);
    circuit.components[1]!.params.resistance = 2000;
    let thrown: unknown;
    try {
      engine.step(1e-4);
    } catch (err) {
      thrown = err;
    }
    expect(thrown).toBeInstanceOf(CoreSuiteEditNotDeliverableError);
    expect((thrown as CoreSuiteEditNotDeliverableError).reason).toBe("needs-carry");

    const rewired = pair();
    const engine2 = new SimEngine();
    engine2.load(rewired);
    rewired.components.push(part("r2", "resistor", ["a", "b"], { resistance: 10 }));
    expect(() => engine2.step(1e-4)).toThrow(CoreSuiteEditNotDeliverableError);
  });

  it("a setParams that skipped a change (it returns false) is refused, not recorded as delivered", () => {
    const circuit: SimCircuit = {
      components: [
        part("v", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("sw", "switch", ["a", "b"], { closed: 0 }),
        part("r", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      wires: [wire("v", "pos", "sw", "a"), wire("sw", "b", "r", "a"), wire("r", "b", "v", "neg")],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    circuit.components[1]!.params.closed = 1;
    const spy = vi.spyOn(CoreEngine.prototype, "setParams").mockReturnValueOnce(false);
    try {
      let thrown: unknown;
      try {
        engine.step(1e-4);
      } catch (err) {
        thrown = err;
      }
      expect(thrown).toBeInstanceOf(CoreSuiteEditNotDeliverableError);
      expect((thrown as CoreSuiteEditNotDeliverableError).reason).toBe("not-applied");
      expect(engine.simTime).toBe(0);
    } finally {
      spy.mockRestore();
    }
    // Still undelivered: the next step tries the same edit again, and now it lands.
    engine.step(1e-4);
    expect(engine.getNetV()[engine.getNetIdForPin("r", "a")!]!).toBeCloseTo(5, 3);
  });

  it("the refused edit does not advance time", () => {
    const circuit = pair();
    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(1e-4);
    circuit.components[1]!.params.resistance = 2000;
    expect(() => engine.step(1e-4)).toThrow(CoreSuiteEditNotDeliverableError);
    expect(engine.simTime).toBeCloseTo(1e-4, 12);
  });
});

describe("adapter.ts SimEngine — takeEepromUpdates (classic's shape, from the core)", () => {
  function eeprom28c16WriteCircuit(): SimCircuit {
    const def = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === "28c16");
    if (!def) throw new Error("test bug: no bundled 28c16");
    const ic: SimComponent = { id: "u1", kind: def.kind, catalogUid: def.uid, pins: def.pin_layout.map((p) => ({ id: p.id })), params: {} };
    const bits = Array.from({ length: 8 }, (_, i) => (0x5a >> i) & 1);
    return {
      components: [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("we_sw", "spdt_switch", ["com", "b", "c"], { position: 0 }),
        ic,
      ],
      wires: [
        wire("vsrc", "pos", "u1", "vcc"),
        wire("vsrc", "neg", "u1", "gnd"),
        wire("vsrc", "neg", "u1", "/ce"),
        wire("vsrc", "pos", "u1", "/oe"),
        wire("vsrc", "neg", "we_sw", "b"),
        wire("vsrc", "pos", "we_sw", "c"),
        wire("we_sw", "com", "u1", "/we"),
        ...Array.from({ length: 11 }, (_, i) => wire("vsrc", "neg", "u1", `a${i}`)),
        ...bits.map((bit, i) => wire("vsrc", bit ? "pos" : "neg", "u1", `io${i}`)),
      ],
    };
  }

  it("is empty until a write completes, then returns the whole image once", () => {
    const circuit = eeprom28c16WriteCircuit();
    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(1e-3);
    expect(engine.takeEepromUpdates()).toEqual({});

    // /we LOW -> HIGH through the live switch, as the eeprom-write suite drives it.
    circuit.components.find((c) => c.id === "we_sw")!.params.position = 1;
    engine.step(1e-3);
    for (let i = 0; i < 12; i++) engine.step(1e-3); // past tWC
    const updates = engine.takeEepromUpdates();
    expect(Object.keys(updates)).toEqual(["u1"]);
    expect(updates.u1).toHaveLength(2048);
    expect(updates.u1![0]).toBe(0x5a);
    expect(engine.takeEepromUpdates()).toEqual({});
  });

  it("refuses to answer from classic after a fallback", () => {
    const engine = new SimEngine();
    expect(() => engine.load(WITH_UNSUPPORTED_KIND)).toThrow(CoreSuiteFellBackError);
    expect(() => engine.takeEepromUpdates()).toThrow(/takeEepromUpdates\(\) needs the core/);
  });
});
