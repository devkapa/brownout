import { afterAll, afterEach, beforeAll, describe, expect, it, vi } from "vitest";
import fs from "node:fs";
import path from "node:path";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import {
  coreCall,
  getWasmApi,
  initCore,
  readCoreErrorMessage,
  __installCoreForTest,
  __resetCoreForTest,
  type CoreExports,
} from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import {
  CoreNotInitializedError,
  CoreNotLoadedError,
  CoreUnsupportedError,
} from "../../src/core/errors.js";
import type { EngineReadout } from "../../src/core/surface.js";
import { buildCoreNodes, classicElementIKeys, compileCircuit, type CompiledCircuit } from "../../src/core/compile/index.js";
import { CORE_OPTION_KEYS, decodeProgram } from "../../src/core/format.js";
import { CORE_IC_PARAM, CORE_IMPLEMENTED_KINDS, CORE_KIND_CODES } from "../../src/core/kinds.js";
import { HeadlessRunner } from "../../src/host/headless.js";
import { buildDigitalState, buildElementChannelI, buildElementI, buildNetV } from "../../src/core/readout.js";

// ---- compile-time check: SimEngine must satisfy EngineReadout -------------
// A pure type-level assertion — zero runtime footprint (a type alias emits no
// JS at all), unlike `declare const x: T; const y: U = x;`, which still
// emits a real (and here undefined-at-runtime) value reference. If SimEngine
// ever drifts from EngineReadout's shape, this line fails to typecheck
// (`pnpm run typecheck` / this file's own compilation), which is the whole
// reason surface.ts is not designed fresh.
type AssertExtends<T extends U, U> = true;
// eslint-disable-next-line @typescript-eslint/no-unused-vars -- type-only, checked by tsc, not read at runtime.
type _SimEngineSatisfiesEngineReadout = AssertExtends<SimEngine, EngineReadout>;

// ---- small circuit-building helpers, local to this file (matches
// test/core/compile.test.ts's convention — this worker owns no shared
// test-helper files) --------------------------------------------------------

type SimComponent = SimCircuit["components"][number];

function part(
  id: string,
  kind: string,
  pinIds: string[],
  params: Record<string, number | string> = {},
): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function circuit(components: SimComponent[], wires: SimCircuit["wires"] = []): SimCircuit {
  return { components, wires };
}

/** 1e-9 relative + 1e-12 absolute — this file's tolerance for core-vs-classic parity, per the task brief. */
function expectClose(actual: number, expected: number, what: string): void {
  const tolerance = 1e-12 + 1e-9 * Math.abs(expected);
  const diff = Math.abs(actual - expected);
  if (diff > tolerance) {
    throw new Error(`${what}: expected ${expected}, got ${actual} (diff ${diff} > tolerance ${tolerance})`);
  }
}

function expectRecordsClose(actual: Record<string, number>, expected: Record<string, number>, what: string): void {
  expect(new Set(Object.keys(actual)), `${what}: key set`).toEqual(new Set(Object.keys(expected)));
  for (const key of Object.keys(expected)) {
    expectClose(actual[key]!, expected[key]!, `${what}.${key}`);
  }
}

/** Loads `c` on a fresh classic SimEngine and returns it, for parity comparison. */
function loadClassic(c: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(c);
  return engine;
}

function loadCore(c: SimCircuit): CoreEngine {
  const engine = new CoreEngine();
  engine.load(c);
  return engine;
}

function expectParity(c: SimCircuit, what: string): { classic: SimEngine; core: CoreEngine } {
  const classic = loadClassic(c);
  const core = loadCore(c);
  expectRecordsClose(core.getNetV(), classic.getNetV(), `${what} netV`);
  expectRecordsClose(core.getElementI(), classic.getElementI(), `${what} elementI`);
  expect(core.nets, `${what} nets`).toEqual(classic.nets);
  return { classic, core };
}

beforeAll(async () => {
  await initCore();
});

// ---- classic parity, one real CoreEngine + one real classic SimEngine per
// scenario named in the task brief -------------------------------------------

describe("CoreEngine vs classic SimEngine — same circuits, same results", () => {
  it("10 V divider", () => {
    const c = circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 10 }),
        part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
        part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      [
        wire("vsrc", "pos", "r1", "a"),
        wire("r1", "b", "r2", "a"),
        wire("r2", "b", "vsrc", "neg"),
      ],
    );
    const { core } = expectParity(c, "10V divider");
    expectClose(core.getNetV().n0!, 10, "n0");
    expectClose(core.getNetV().n1!, 5, "n1");
  });

  it("source with + on the datum (node at -V)", () => {
    // vs1 defines "gnd" via its own neg pin (buildNets' SOURCE_KIND
    // convention); vs2's "+" (pins[0]) is wired directly onto that SAME gnd
    // net, so vs2's "-" terminal is fixed at -voltage, not +voltage.
    const c = circuit(
      [
        part("vs1", "voltage_source", ["pos", "neg"], { voltage: 9 }),
        part("r1", "resistor", ["a", "b"], { resistance: 100 }),
        part("vs2", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r2", "resistor", ["a", "b"], { resistance: 50 }),
      ],
      [
        wire("vs1", "pos", "r1", "a"),
        wire("r1", "b", "vs1", "neg"),
        wire("vs2", "pos", "vs1", "neg"),
        wire("vs2", "neg", "r2", "a"),
        wire("r2", "b", "vs1", "neg"),
      ],
    );
    const { core } = expectParity(c, "+ on datum");
    const negNodeVoltage = Object.entries(core.getNetV()).find(([id]) => id !== "gnd" && core.getNetIdForPin("vs2", "neg") === id)?.[1];
    expect(negNodeVoltage).toBeLessThan(0);
    expectClose(negNodeVoltage!, -5, "vs2.neg node");
  });

  it("chained sources with loads", () => {
    const c = circuit(
      [
        part("vs1", "voltage_source", ["pos", "neg"], { voltage: 12 }),
        part("r1", "resistor", ["a", "b"], { resistance: 220 }),
        part("vs2", "voltage_source", ["pos", "neg"], { voltage: 6 }),
        part("r2", "resistor", ["a", "b"], { resistance: 330 }),
      ],
      [
        wire("vs1", "pos", "r1", "a"),
        wire("r1", "b", "vs1", "neg"),
        wire("vs2", "pos", "vs1", "neg"),
        wire("vs2", "neg", "r2", "a"),
        wire("r2", "b", "vs1", "neg"),
      ],
    );
    expectParity(c, "chained sources");
  });

  it("isolated/floating nets (an unwired resistor's two pins are each their own single-pin net)", () => {
    const c = circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r1", "resistor", ["a", "b"], { resistance: 100 }),
        part("rFloat", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
    const { core } = expectParity(c, "floating nets");
    expect(Object.keys(core.getNetV())).toHaveLength(4); // gnd, n0 (vsrc.pos/r1.a), plus rFloat's two singletons
  });

  it("two independent partitions (sharing one gnd, no resistor path between the two ladders)", () => {
    const c = circuit(
      [
        part("vs1", "voltage_source", ["pos", "neg"], { voltage: 12 }),
        part("r1", "resistor", ["a", "b"], { resistance: 100 }),
        part("r2", "resistor", ["a", "b"], { resistance: 100 }),
        part("vs2", "voltage_source", ["pos", "neg"], { voltage: 6 }),
        part("r3", "resistor", ["a", "b"], { resistance: 50 }),
        part("r4", "resistor", ["a", "b"], { resistance: 50 }),
      ],
      [
        wire("vs1", "pos", "r1", "a"),
        wire("r1", "b", "r2", "a"),
        wire("r2", "b", "vs1", "neg"),
        wire("vs2", "pos", "r3", "a"),
        wire("r3", "b", "r4", "a"),
        wire("r4", "b", "vs1", "neg"),
        wire("vs2", "neg", "vs1", "neg"),
      ],
    );
    const { core } = expectParity(c, "two partitions");
    expect(core.stats().partitions).toBe(2);
  });

  it("no-source circuit (synthetic datum)", () => {
    const c = circuit(
      [part("r1", "resistor", ["a", "b"], { resistance: 10 }), part("r2", "resistor", ["a", "b"], { resistance: 20 })],
      [wire("r1", "b", "r2", "a")],
    );
    const { core, classic } = expectParity(c, "no-source");
    expect(classic.nets.some((n) => n.id === "gnd")).toBe(false);
    expect(core.getNetV().gnd).toBe(0); // the phantom "gnd": 0 key even with no gnd net (see engine.ts's _publishReadouts)
  });
});

describe("getNetIdForPin", () => {
  it("matches classic for every pin, including a floating one", () => {
    const c = circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r1", "resistor", ["a", "b"], { resistance: 100 }),
        part("rFloat", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
    const classic = loadClassic(c);
    const core = loadCore(c);
    for (const [componentId, pinId] of [
      ["vsrc", "pos"],
      ["vsrc", "neg"],
      ["r1", "a"],
      ["r1", "b"],
      ["rFloat", "a"],
      ["rFloat", "b"],
    ]) {
      expect(core.getNetIdForPin(componentId!, pinId!)).toBe(classic.getNetIdForPin(componentId!, pinId!));
    }
  });

  it("returns undefined for an unknown pin", () => {
    const c = circuit([part("r1", "resistor", ["a", "b"], { resistance: 100 })]);
    const core = loadCore(c);
    expect(core.getNetIdForPin("nope", "a")).toBeUndefined();
    expect(core.getNetIdForPin("r1", "nope")).toBeUndefined();
  });
});

describe("advance()", () => {
  function divider(): SimCircuit {
    return circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
  }

  it("advances forward, W0 only moves time (no resolve)", () => {
    const core = loadCore(divider());
    expect(core.simTime).toBe(0);
    core.advance(3);
    expect(core.simTime).toBe(3);
    expect(core.lastConverged).toBe(true);
    core.advance(3); // advancing to the SAME time is not "backward"
    expect(core.simTime).toBe(3);
  });

  it("rejects moving backward with a RangeError, leaving state intact", () => {
    const core = loadCore(divider());
    core.advance(5);
    const netVBefore = { ...core.getNetV() };
    expect(() => core.advance(2)).toThrow(RangeError);
    expect(core.simTime).toBe(5); // unaffected by the rejected call
    expect(core.getNetV()).toEqual(netVBefore);
  });

  it("throws CoreNotLoadedError if called before any load()", () => {
    const core = new CoreEngine();
    expect(() => core.advance(1)).toThrow(CoreNotLoadedError);
  });
});

describe("simTime across reloads — mirrors classic exactly", () => {
  it("classic SimEngine.load() does not reset simTime (only coldLoad() does); CoreEngine matches", () => {
    const c1 = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );

    const classic = new SimEngine();
    classic.load(c1);
    classic.step(1e-3);
    expect(classic.simTime).toBeCloseTo(1e-3, 12);
    classic.load(c1); // reload the same circuit
    expect(classic.simTime).toBeCloseTo(1e-3, 12); // classic KEEPS time across a reload

    const core = new CoreEngine();
    core.load(c1);
    core.advance(1e-3);
    expect(core.simTime).toBeCloseTo(1e-3, 12);
    core.load(c1); // the Rust engine restarts at t=0 internally...
    expect(core.simTime).toBeCloseTo(1e-3, 12); // ...but the facade carries the old simTime forward, matching classic
  });
});

describe("unsupported topology -> CoreUnsupportedError with a topology: reason", () => {
  it("two parallel ideal sources (a loop of ideal sources)", () => {
    const c = circuit(
      [part("vs1", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("vs2", "voltage_source", ["pos", "neg"], { voltage: 9 })],
      [wire("vs1", "pos", "vs2", "pos"), wire("vs1", "neg", "vs2", "neg")],
    );
    const core = new CoreEngine();
    let caught: CoreUnsupportedError | undefined;
    try {
      core.load(c);
    } catch (err) {
      caught = err as CoreUnsupportedError;
    }
    expect(caught).toBeInstanceOf(CoreUnsupportedError);
    expect(caught!.reasons).toHaveLength(1);
    expect(caught!.reasons[0]).toMatch(/^topology:/);
  });

  it("a floating ideal source (neither terminal reachable from the datum)", () => {
    const c = circuit(
      [
        part("vs1", "voltage_source", ["pos", "neg"], { voltage: 9 }),
        part("r1", "resistor", ["a", "b"], { resistance: 100 }),
        part("vsFloat", "voltage_source", ["pos", "neg"], { voltage: 3 }),
        part("rFloat", "resistor", ["a", "b"], { resistance: 10 }),
      ],
      [
        wire("vs1", "pos", "r1", "a"),
        wire("r1", "b", "vs1", "neg"),
        wire("vsFloat", "pos", "rFloat", "a"),
        wire("vsFloat", "neg", "rFloat", "b"),
      ],
    );
    const core = new CoreEngine();
    expect(() => core.load(c)).toThrow(CoreUnsupportedError);
    try {
      core.load(c);
    } catch (err) {
      expect((err as CoreUnsupportedError).reasons[0]).toMatch(/^topology:/);
    }
  });
});

describe("unsupported kind -> CoreUnsupportedError with an unsupported-kind: reason", () => {
  it("reports the kind name", () => {
    const c = circuit([part("led1", "led", ["a", "k"])]);
    const core = new CoreEngine();
    let caught: CoreUnsupportedError | undefined;
    try {
      core.load(c);
    } catch (err) {
      caught = err as CoreUnsupportedError;
    }
    expect(caught).toBeInstanceOf(CoreUnsupportedError);
    expect(caught!.reasons).toEqual(["unsupported-kind:led"]);
  });
});

describe("load-time delta-cycle livelock (-7) -> CoreUnsupportedError with a livelock: reason, and HeadlessRunner falls back", () => {
  afterEach(async () => {
    // Same rationale as the "pointer sign coercion" block above: this
    // describe installs a synthetic CoreExports and must not leak it into
    // whatever test runs next in this file.
    __resetCoreForTest();
    await initCore();
  });

  /**
   * `mapLoadFailure` (engine.ts) must map a -7 `bo_engine_load` return code
   * the SAME way it maps -3 (topology): CoreUnsupportedError with a reason
   * HeadlessRunner._tryLoadCore's `catch` recognises, not the generic
   * CoreLoadError -3's sibling codes get. This is a synthetic bo_engine_load
   * (no real circuit can produce -7 at load — see the sibling describe block
   * below, "a real zero-delay combinational ring...", for why: none of the 11
   * W1c combinational-IC kinds can reach a catalog-resolved delay of 0
   * through propagationDelayForComponent, so this file's dedicated proof of
   * mapLoadFailure's MAPPING itself has to inject the code directly, exactly
   * as the "pointer sign coercion" describe block above injects addresses it
   * cannot otherwise reach).
   */
  function installSyntheticLoadFailure(code: number, message: string): void {
    const buffer = new ArrayBuffer(0x10000);
    const messageBytes = new TextEncoder().encode(message);
    new Uint8Array(buffer, 0, messageBytes.length).set(messageBytes);
    const exports: CoreExports = {
      memory: { buffer },
      bo_abi_version: () => 2,
      // 8-byte aligned (CoreEngine.load() views this address as a
      // Float64Array), and well past the message bytes at offset 0.
      bo_alloc: () => 0x1000,
      bo_free: () => {},
      bo_engine_create: () => 1,
      bo_engine_destroy: () => {},
      bo_engine_load: () => code,
      bo_engine_advance: () => 0,
      bo_engine_time: () => 0,
      bo_engine_node_count: () => 0,
      bo_engine_node_voltages: () => 0,
      bo_engine_device_count: () => 0,
      bo_engine_device_currents: () => 0,
      bo_engine_stat: () => 0,
      bo_error_ptr: () => 0,
      bo_error_len: () => messageBytes.length,
      bo_engine_set_option: () => 0,
      bo_engine_next_event_time: () => Infinity,
      bo_engine_next_digital_due_time: () => Infinity,      bo_engine_digital_slot_count: () => 0,
      bo_engine_digital_slots: () => 0,
      bo_engine_channel_slot_count: () => 0,
      bo_engine_channel_slots: () => 0,
      bo_engine_trace_enable: () => 0,
      bo_engine_trace_ptr: () => 0,
      bo_engine_trace_len: () => 0,
      bo_engine_trace_clear: () => {},
      bo_engine_seed_memory: () => 0,
      bo_engine_take_eeprom_updates: () => 0,
      bo_engine_eeprom_updates_ptr: () => 0,
      bo_engine_set_params: () => 0,
    };
    __installCoreForTest(exports);
  }

  it("CoreEngine.load() throws CoreUnsupportedError with a livelock: reason (not the generic CoreLoadError -4/-5/... get)", () => {
    installSyntheticLoadFailure(-7, "delta-cycle livelock: pass cap exceeded at t=0");
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
    const core = new CoreEngine();
    let caught: CoreUnsupportedError | undefined;
    try {
      core.load(c);
    } catch (err) {
      caught = err as CoreUnsupportedError;
    }
    expect(caught).toBeInstanceOf(CoreUnsupportedError);
    expect(caught!.reasons).toEqual(["livelock:delta-cycle livelock: pass cap exceeded at t=0"]);
  });

  it("HeadlessRunner falls back to classic for a load-time livelock, exactly as it does for an unsupported topology", () => {
    installSyntheticLoadFailure(-7, "delta-cycle livelock: pass cap exceeded at t=0");
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
    // coreSupportedKinds is widened to CORE_IMPLEMENTED_KINDS the same way
    // test/core/suites/adapter.ts does: CORE_SUPPORTED_KINDS is empty for all
    // of W1 (decision 15), so a default-constructed HeadlessRunner would
    // reject resistor/voltage_source before ever attempting a core load —
    // this test is about the LOAD-TIME livelock fallback, not the separate
    // "kind not supported yet" rejection.
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(c);
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual(["livelock:delta-cycle livelock: pass cap exceeded at t=0"]);
    // The circuit still simulates correctly on the classic fallback — a
    // livelocked load is not a dead end for the caller.
    expect(runner.readout.getNetV()["n0"]).toBeCloseTo(5, 9);
  });
});

describe("a real zero-delay combinational ring genuinely livelocks the Rust kernel at load (-7)", () => {
  /**
   * None of the 11 W1c combinational-IC kinds can reach a catalog-resolved
   * delay of 0 through the normal compiler path: propagationDelayForComponent
   * (sim-engine.ts) only honours a catalog prop_delay_ns when it is `> 0`
   * (explicitly excluding 0), and every one of the 11 kinds' prefixes is
   * "74ls" or "74hc", both of which have a non-zero prefix fallback (20 ns /
   * 15 ns) — so the function's own final `return 0` is unreachable for any of
   * them. Checked directly against the bundled catalog (src/parts/
   * default-parts.json): none of the 11 declares prop_delay_ns: 0 either, and
   * even if one did, the `> 0` guard would still skip it. A real 0-delay
   * combinational ring is therefore impossible to compile through
   * CoreEngine.load()'s public API today (compileCircuit -> propagationDelay-
   * ForComponent), so this test builds one honestly: compile a REAL, valid
   * ring circuit (three 74LS04 inverters, odd — so it has no stable digital
   * fixed point — output-to-input in a cycle, sharing one power rail), then
   * patch ONLY the compiled program's own delay params to 0 before handing it
   * to the REAL wasm module's bo_engine_load directly (bypassing
   * CoreEngine.load()/compileCircuit's own delay resolution, but changing
   * nothing else about the program a real compile would have produced — same
   * pins, same hashes, same topology). This proves the RUST KERNEL itself
   * (not just this file's TS-level mapping test above) returns -7 for a
   * genuine zero-delay livelock, exactly as core/README.md's delta-cycle
   * section documents ("More than 1000 deltas at one instant is -7, never a
   * silent exit").
   */
  function ringCircuit(): SimCircuit {
    // Each 74ls04 package has 6 independent inverter gates; only gate 1 of
    // each package sits in the ring. The other 5 gates' inputs are tied to
    // the powered vcc rail (a FIXED node) rather than left open, so this
    // fixture never has to reason about the floating-input model at all —
    // every pin here is driven or fixed, never OPEN.
    const inverter = (id: string): SimComponent =>
      part(id, "74ls04", ["vcc", "gnd", "1a", "1y", "2a", "2y", "3a", "3y", "4a", "4y", "5a", "5y", "6a", "6y"]);
    const tieUnusedGates = (id: string): SimCircuit["wires"] => [
      wire("vsrc", "pos", id, "2a"),
      wire("vsrc", "pos", id, "3a"),
      wire("vsrc", "pos", id, "4a"),
      wire("vsrc", "pos", id, "5a"),
      wire("vsrc", "pos", id, "6a"),
    ];
    return circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), inverter("u1"), inverter("u2"), inverter("u3")],
      [
        wire("vsrc", "pos", "u1", "vcc"), wire("vsrc", "neg", "u1", "gnd"),
        wire("vsrc", "pos", "u2", "vcc"), wire("vsrc", "neg", "u2", "gnd"),
        wire("vsrc", "pos", "u3", "vcc"), wire("vsrc", "neg", "u3", "gnd"),
        // The ring: an odd number (3) of inversions has no stable fixed
        // point — u1.1y -> u2.1a -> u2.1y -> u3.1a -> u3.1y -> u1.1a.
        wire("u1", "1y", "u2", "1a"),
        wire("u2", "1y", "u3", "1a"),
        wire("u3", "1y", "u1", "1a"),
        ...tieUnusedGates("u1"),
        ...tieUnusedGates("u2"),
        ...tieUnusedGates("u3"),
      ],
    );
  }

  /**
   * Walks a v2 program's own documented layout (core/README.md, "Program
   * format, version 2": header through the option block and slot counts,
   * then D device records of `kindCode, componentIndex, pinCount, pin x4,
   * paramCount, param x paramCount, blobCount, blob x blobCount`) far enough
   * to find each `kindCode` device's own param-block start offset — this
   * test only ever needs to overwrite one param (delay, CORE_IC_PARAM.delay)
   * per matching device, not decode the whole thing (decodeProgram already
   * does that, more thoroughly, for format.test.ts's own purposes).
   */
  function findParamBlockOffsets(program: Float64Array, kindCode: number): number[] {
    const deviceCount = program[3]!;
    const optionCount = program[4]!;
    let i = 5 + optionCount * 2 + 2; // past MAGIC/VERSION/nodeCount/deviceCount/optionCount, the option pairs, and the two slot counts
    const offsets: number[] = [];
    for (let d = 0; d < deviceCount; d++) {
      const devKind = program[i]!;
      i += 2; // kindCode, componentIndex
      const pinCount = program[i]!;
      i += 1 + pinCount * 4; // pinCount field, then the pin records themselves
      const paramCount = program[i]!;
      i += 1; // paramCount field
      if (devKind === kindCode) offsets.push(i);
      i += paramCount;
      const blobCount = program[i]!;
      i += 1 + blobCount;
    }
    return offsets;
  }

  it("bo_engine_load returns -7 for a genuine zero-delay ring (three 74LS04 inverters, real compiled program, delay patched to 0)", () => {
    const result = compileCircuit(ringCircuit(), CORE_IMPLEMENTED_KINDS);
    if (!result.ok) throw new Error(`ringCircuit() failed to compile: ${result.reasons.join(", ")}`);
    const program = result.compiled.program;

    const delayOffsets = findParamBlockOffsets(program, CORE_KIND_CODES["74ls04"]);
    expect(delayOffsets, "expected all three 74ls04 device records in the compiled program").toHaveLength(3);
    for (const paramStart of delayOffsets) {
      // Sanity: this really is the delay slot on a real compiled program,
      // and it really is non-zero before patching (propagationDelayForComponent's
      // 20 ns 74ls prefix fallback) — if either check ever failed, the offset
      // arithmetic above would be wrong, not the kernel under test.
      expect(program[paramStart + CORE_IC_PARAM.delay]).toBeGreaterThan(0);
      program[paramStart + CORE_IC_PARAM.delay] = 0;
    }

    const { code, message } = coreCall((exports) => {
      const ptr = exports.bo_alloc(program.length) >>> 0;
      const handle = exports.bo_engine_create();
      try {
        new Float64Array(exports.memory.buffer, ptr, program.length).set(program);
        const loadCode = exports.bo_engine_load(handle, ptr, program.length);
        return { code: loadCode, message: loadCode !== 0 ? readCoreErrorMessage(exports) : "" };
      } finally {
        exports.bo_engine_destroy(handle);
        exports.bo_free(ptr, program.length);
      }
    });

    expect(code, `expected -7 (delta-cycle livelock); bo_engine_load's own message: ${message}`).toBe(-7);
  });
});

describe("a real IC-supply fixed point that never converges at load (-4) falls back like -7 (finding 11)", () => {
  /**
   * Review finding 11's own reproduction: a 74LS04 fed vcc through a 100 ohm
   * resistor from a 5V source, 1a tied to gnd (so 1y is HIGH), and 1y driving
   * 100 ohm to gnd. When powered, 1y's own output load (in series with the
   * 100 ohm supply feed) sags vcc below 0.9*vccMin, which unpowers the IC;
   * unpowered, vcc recovers to 5V and the IC re-powers — the IC-supply fixed
   * point (README's digital-core point 3) never settles within its 50-
   * iteration cap, so `bo_engine_load` returns a genuine -4. Unlike the -7
   * ring above, this is reachable through the ordinary compiled path — no
   * program patching needed.
   */
  function saggingSupplyCircuit(): SimCircuit {
    return circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r_vcc", "resistor", ["a", "b"], { resistance: 100 }),
        part("u1", "74ls04", ["vcc", "gnd", "1a", "1y", "2a", "2y", "3a", "3y", "4a", "4y", "5a", "5y", "6a", "6y"]),
        part("r_out", "resistor", ["a", "b"], { resistance: 100 }),
      ],
      [
        wire("vsrc", "pos", "r_vcc", "a"),
        wire("r_vcc", "b", "u1", "vcc"),
        wire("vsrc", "neg", "u1", "gnd"),
        wire("vsrc", "neg", "u1", "1a"),
        wire("u1", "1y", "r_out", "a"),
        wire("r_out", "b", "vsrc", "neg"),
        wire("vsrc", "neg", "u1", "2a"),
        wire("vsrc", "neg", "u1", "3a"),
        wire("vsrc", "neg", "u1", "4a"),
        wire("vsrc", "neg", "u1", "5a"),
        wire("vsrc", "neg", "u1", "6a"),
      ],
    );
  }

  it("CoreEngine.load() throws CoreUnsupportedError with a solve-failed: reason (bo_engine_load's real -4, not the generic CoreLoadError)", () => {
    const core = new CoreEngine();
    let caught: CoreUnsupportedError | undefined;
    try {
      core.load(saggingSupplyCircuit(), CORE_IMPLEMENTED_KINDS);
    } catch (err) {
      caught = err as CoreUnsupportedError;
    }
    expect(caught).toBeInstanceOf(CoreUnsupportedError);
    expect(caught!.reasons).toHaveLength(1);
    expect(caught!.reasons[0]).toMatch(/^solve-failed:/);
    expect(caught!.reasons[0]).toMatch(/converge/i);
  });

  it("HeadlessRunner falls back to classic instead of throwing an uncaught error (finding 11's exact reported bug)", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    // Before this fix, this call threw an uncaught CoreLoadError straight out
    // of HeadlessRunner.load(): -4 fell through mapLoadFailure's generic
    // branch, which _tryLoadCore does not catch (only CoreUnsupportedError
    // triggers the classic fallback).
    expect(() => runner.load(saggingSupplyCircuit())).not.toThrow();
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toHaveLength(1);
    expect(runner.engineFallbackReasons[0]).toMatch(/^solve-failed:/);
    // Classic itself loads this circuit without throwing (decision 8: no
    // rescue ladder, lastConverged can end up false, but load() itself must
    // not throw) — the fallback is not a dead end for the caller.
    expect(() => runner.readout.getNetV()).not.toThrow();
  });
});

describe("a failed load() clears the previously loaded circuit's readouts", () => {
  it("every readout throws CoreNotLoadedError after a failed reload, and a later good load() recovers", () => {
    const good = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
    const bad = circuit([part("led1", "led", ["a", "k"])]);

    const core = new CoreEngine();
    core.load(good);
    expect(core.getNetV()).toMatchObject({ n0: 5 });

    expect(() => core.load(bad)).toThrow(CoreUnsupportedError);

    // The Rust side still has the OLD ("good") program loaded (core/README.md's
    // contract), but this facade must not keep serving it.
    expect(() => core.getNetV()).toThrow(CoreNotLoadedError);
    expect(() => core.getElementI()).toThrow(CoreNotLoadedError);
    expect(() => core.simTime).toThrow(CoreNotLoadedError);
    expect(() => core.nets).toThrow(CoreNotLoadedError);
    expect(() => core.lastConverged).toThrow(CoreNotLoadedError);
    expect(() => core.stats()).toThrow(CoreNotLoadedError);
    expect(() => core.getNetIdForPin("vsrc", "pos")).toThrow(CoreNotLoadedError);
    expect(() => core.advance(1)).toThrow(CoreNotLoadedError);

    core.load(good); // recovers
    expect(core.getNetV()).toMatchObject({ n0: 5 });
  });

  it("also clears state after an ABI-level (topology) load failure, not just a compile-time one", () => {
    const good = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
    const topologyBad = circuit(
      [part("vs1", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("vs2", "voltage_source", ["pos", "neg"], { voltage: 9 })],
      [wire("vs1", "pos", "vs2", "pos"), wire("vs1", "neg", "vs2", "neg")],
    );

    const core = new CoreEngine();
    core.load(good);
    expect(() => core.load(topologyBad)).toThrow(CoreUnsupportedError);
    expect(() => core.getNetV()).toThrow(CoreNotLoadedError);
    core.load(good);
    expect(core.getNetV()).toMatchObject({ n0: 5 });
  });

  it("a failed load() does not reset the clock — a fail-then-succeed sequence resumes at the pre-failure simTime, not 0", () => {
    const good = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
    const bad = circuit([part("led1", "led", ["a", "k"])]);

    const core = new CoreEngine();
    core.load(good);
    core.advance(7);
    expect(core.simTime).toBe(7);

    // The buggy version of this computed "previous simTime" as 0 whenever
    // `_compiled` was null — which a failed load() itself makes true — so a
    // fail-then-succeed sequence silently restarted the clock at 0.
    expect(() => core.load(bad)).toThrow(CoreUnsupportedError);
    expect(() => core.simTime).toThrow(CoreNotLoadedError); // no program loaded right now

    core.load(good); // recovers
    expect(core.simTime).toBe(7); // NOT reset to 0
  });
});

describe("dispose()", () => {
  it("is idempotent and every readout throws afterward", () => {
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
    const core = loadCore(c);
    core.dispose();
    expect(() => core.dispose()).not.toThrow(); // idempotent
    expect(() => core.getNetV()).toThrow();
    expect(() => core.advance(1)).toThrow();
  });
});

describe("constructing a CoreEngine before initCore() — a genuinely fresh module", () => {
  it("throws CoreNotInitializedError", async () => {
    // A fresh module instance (not the one this file's beforeAll already
    // initialized) via vi.resetModules() + dynamic import, per the brief:
    // this file's shared, already-initialized module state can't exercise
    // the "nobody has called initCore() yet" path otherwise.
    vi.resetModules();
    const { CoreEngine: FreshCoreEngine } = await import("../../src/core/engine.js");
    const { CoreNotInitializedError: FreshCoreNotInitializedError } = await import("../../src/core/errors.js");
    expect(() => new FreshCoreEngine()).toThrow(FreshCoreNotInitializedError);
  });
});

describe("stats()", () => {
  it("reports fixed nodes and partitions for a simple divider", () => {
    const c = circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r1", "resistor", ["a", "b"], { resistance: 100 }),
        part("r2", "resistor", ["a", "b"], { resistance: 100 }),
      ],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "r2", "a"), wire("r2", "b", "vsrc", "neg")],
    );
    const core = loadCore(c);
    const stats = core.stats();
    expect(stats.fixedNodes).toBe(2); // gnd + vsrc.pos
    expect(stats.partitions).toBe(1); // the single free midpoint node, joined by... actually 1 free node, 0 both-free resistors -> 1 trivial partition
    expect(stats.staticSolves).toBeGreaterThanOrEqual(1);
  });
});

describe("isCurrent()", () => {
  function divider(): SimCircuit {
    return circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
  }

  it("true right after construction, and false once disposed — never throws", () => {
    const core = new CoreEngine();
    expect(core.isCurrent()).toBe(true);
    core.dispose();
    expect(core.isCurrent()).toBe(false);
  });

  it("false while the live instance is poisoned, and false (not throwing) once re-init moves the generation on; a freshly constructed engine is current again", async () => {
    const core = new CoreEngine();
    core.load(divider());
    expect(core.isCurrent()).toBe(true);

    expect(() =>
      coreCall(() => {
        throw new (getWasmApi().RuntimeError)("synthetic trap for isCurrent() test");
      }),
    ).toThrow();
    // Poisoned: coreStatus() is "poisoned", not "ready" — isCurrent() must
    // say so without throwing, so a host (HeadlessRunner._tryLoadCore) can
    // decide whether to rebuild without first provoking CoreStaleInstanceError.
    expect(core.isCurrent()).toBe(false);

    await initCore(); // the documented recovery: a fresh instance, a new generation
    // Now stale (wrong generation), a different reason than "poisoned" but
    // the same false answer — this engine's handle no longer means anything.
    expect(core.isCurrent()).toBe(false);

    const fresh = new CoreEngine();
    expect(fresh.isCurrent()).toBe(true);
    fresh.load(divider());
    expect(fresh.getNetV()).toMatchObject({ n0: 5 });
  });
});

describe("CoreEngine.load's options.simTime — seeding the facade clock on an engine switch", () => {
  function divider(): SimCircuit {
    return circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
  }

  it("starts the just-loaded program's clock at the given time, overriding whatever this engine's own clock already read", () => {
    const core = new CoreEngine();
    core.load(divider(), undefined, { simTime: 2.5 });
    expect(core.simTime).toBe(2.5);
    core.advance(3);
    expect(core.simTime).toBe(3);
  });

  it("rejects a negative simTime with RangeError before touching the currently loaded program", () => {
    const core = new CoreEngine();
    core.load(divider());
    core.advance(1);
    const netVBefore = { ...core.getNetV() };
    expect(() => core.load(divider(), undefined, { simTime: -1 })).toThrow(RangeError);
    // A caller-bug RangeError, unlike a rejected circuit, must not have
    // cleared what was already loaded (engine.ts's own doc comment on this).
    expect(core.simTime).toBe(1);
    expect(core.getNetV()).toEqual(netVBefore);
  });

  it("rejects a non-finite simTime (NaN/Infinity) with RangeError", () => {
    const core = new CoreEngine();
    expect(() => core.load(divider(), undefined, { simTime: NaN })).toThrow(RangeError);
    expect(() => core.load(divider(), undefined, { simTime: Infinity })).toThrow(RangeError);
    expect(() => core.load(divider(), undefined, { simTime: -Infinity })).toThrow(RangeError);
  });
});

describe("advance() rejects a non-finite tEnd before any ABI call (closing the NaN/Infinity time-backwards backdoor)", () => {
  function divider(): SimCircuit {
    return circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
  }

  it("advance(NaN) throws RangeError and leaves the clock and the backwards-time guard intact", () => {
    const core = loadCore(divider());
    core.advance(5);
    expect(() => core.advance(NaN)).toThrow(RangeError);
    // The Rust guard (`t_end < self.time - TIME_EPS`) is false for NaN, so an
    // un-rejected NaN would have reached bo_engine_advance, set the clock to
    // NaN, and made every LATER comparison against it false too — silently
    // disabling core/README.md error -6 for good. None of that may happen.
    expect(core.simTime).toBe(5);
    expect(() => core.advance(2)).toThrow(RangeError); // still enforced
    expect(core.simTime).toBe(5);
  });

  it("advance(Infinity) throws RangeError and leaves the clock finite and still advanceable", () => {
    const core = loadCore(divider());
    core.advance(5);
    expect(() => core.advance(Infinity)).toThrow(RangeError);
    expect(core.simTime).toBe(5);
    core.advance(10); // a normal finite advance still works afterwards
    expect(core.simTime).toBe(10);
  });
});

describe("pointer sign coercion (>>> 0) at engine.ts's own ABI call sites", () => {
  afterEach(async () => {
    // Restore the real, built module: this block installs a synthetic
    // CoreExports via __installCoreForTest, which would otherwise leak into
    // any test that happens to run after it in this file.
    __resetCoreForTest();
    await initCore();
  });

  it("load()'s bo_alloc pointer and _publishReadouts' node-voltages/device-currents pointers all survive an address at/above 2^31", () => {
    // wasm32 pointers cross the ABI as i32: an address at/above 2^31 comes
    // back to JS as a NEGATIVE number. This is the exact shape bo_alloc,
    // bo_engine_node_voltages and bo_engine_device_currents take once the
    // Rust heap has grown past 2 GiB (the finding this test proves fixed).
    const toSignedI32 = (addr: number): number => (addr >= 2 ** 31 ? addr - 2 ** 32 : addr);

    const PROGRAM_ADDR = 2 ** 31;
    const VOLTAGES_ADDR = 2 ** 31 + 0x10000;
    const CURRENTS_ADDR = 2 ** 31 + 0x20000;
    const VOLTAGE_SENTINEL = 9.75;
    const CURRENT_SENTINEL = 3.25;

    // A single large ArrayBuffer, not real growing wasm memory: virtual/
    // lazily-paged, so this stays cheap despite the >2 GiB address space.
    const buffer = new ArrayBuffer(2 ** 31 + 0x30000);
    new Float64Array(buffer, VOLTAGES_ADDR, 8192).fill(VOLTAGE_SENTINEL);
    new Float64Array(buffer, CURRENTS_ADDR, 8192).fill(CURRENT_SENTINEL);

    const exports: CoreExports = {
      memory: { buffer },
      bo_abi_version: () => 2,
      bo_alloc: () => toSignedI32(PROGRAM_ADDR),
      bo_free: () => {},
      bo_engine_create: () => 7,
      bo_engine_destroy: () => {},
      bo_engine_load: (_h: number, ptr: number) => {
        // Proves load()'s own `>>> 0`: had engine.ts passed the RAW signed
        // value through, `new Float64Array(buffer, ptr, ...).set(program)`
        // would already have thrown (a negative byteOffset) before this was
        // ever reached — so simply getting here, with the COERCED address,
        // is already most of the proof; the explicit check below is the rest.
        expect(ptr).toBe(PROGRAM_ADDR);
        return 0;
      },
      bo_engine_advance: () => 0,
      bo_engine_time: () => 0,
      bo_engine_node_count: () => 0,
      bo_engine_node_voltages: () => toSignedI32(VOLTAGES_ADDR),
      bo_engine_device_count: () => 0,
      bo_engine_device_currents: () => toSignedI32(CURRENTS_ADDR),
      bo_engine_stat: () => 0,
      bo_error_ptr: () => 0,
      bo_error_len: () => 0,
      bo_engine_set_option: () => 0,
      bo_engine_next_event_time: () => Infinity,
      bo_engine_next_digital_due_time: () => Infinity,      bo_engine_digital_slot_count: () => 0,
      bo_engine_digital_slots: () => 0,
      bo_engine_channel_slot_count: () => 0,
      bo_engine_channel_slots: () => 0,
      bo_engine_trace_enable: () => 0,
      bo_engine_trace_ptr: () => 0,
      bo_engine_trace_len: () => 0,
      bo_engine_trace_clear: () => {},
      bo_engine_seed_memory: () => 0,
      bo_engine_take_eeprom_updates: () => 0,
      bo_engine_eeprom_updates_ptr: () => 0,
      bo_engine_set_params: () => 0,
    };
    __installCoreForTest(exports);

    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
    const core = new CoreEngine();
    expect(() => core.load(c)).not.toThrow();

    const netV = core.getNetV();
    const nonGndVoltages = Object.entries(netV).filter(([id]) => id !== "gnd");
    expect(nonGndVoltages.length).toBeGreaterThan(0);
    for (const [, v] of nonGndVoltages) expect(v).toBe(VOLTAGE_SENTINEL);

    const elementI = core.getElementI();
    expect(Object.keys(elementI).length).toBeGreaterThan(0);
    for (const v of Object.values(elementI)) expect(v).toBe(CURRENT_SENTINEL);
  });
});

// ---- v2: readout.ts's pure builders, exercised directly -------------------
// (independent of any ABI call — the digital/channel-slot READING inside
// engine.ts's _publishReadouts is a thin `copyF64` + one of these calls, and
// W1a's own kinds never populate a non-empty slot table, so the meaningful
// coverage of "does the builder logic itself work" lives here rather than
// forcing a real compiled circuit to have slots it cannot have yet.)

function withSlots(
  compiled: CompiledCircuit,
  overrides: Partial<Pick<CompiledCircuit, "digitalSlots" | "channelSlots">>,
): CompiledCircuit {
  return { ...compiled, ...overrides };
}

function compileOkForReadoutTests(): CompiledCircuit {
  const c = circuit(
    [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
    [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
  );
  const result = compileCircuit(c);
  if (!result.ok) throw new Error("expected ok:true");
  return result.compiled;
}

describe("readout.ts", () => {
  it("buildNetV seeds gnd:0 unconditionally, then overlays every real net", () => {
    const compiled = compileOkForReadoutTests();
    const netV = buildNetV(compiled, [0, 5]); // node 0 = gnd = 0V, node 1 = 5V
    expect(netV.gnd).toBe(0);
    expect(Object.keys(netV)).toHaveLength(2);
  });

  it("buildNetV keeps the phantom gnd:0 key even when node 0 is a synthetic (no-net) datum", () => {
    const c = circuit(
      [part("r1", "resistor", ["a", "b"], { resistance: 10 }), part("r2", "resistor", ["a", "b"], { resistance: 20 })],
      [wire("r1", "b", "r2", "a")],
    );
    const result = compileCircuit(c);
    if (!result.ok) throw new Error("expected ok:true");
    const netV = buildNetV(result.compiled, [0, 1, 2, 3]);
    expect(netV.gnd).toBe(0);
    expect(result.compiled.netIdOfNode[0]).toBeNull(); // no net actually named "gnd" here
  });

  it("buildElementI zips compiled.devices with the currents array, in order", () => {
    const compiled = compileOkForReadoutTests();
    const currents = compiled.devices.map((_, i) => (i + 1) * 1.5);
    const elementI = buildElementI(compiled, currents);
    compiled.devices.forEach((device, i) => {
      expect(elementI[device.componentId]).toBe(currents[i]);
    });
  });

  it("buildDigitalState treats NaN as an absent key (classic rebuilds digitalState from scratch every pass, spec §10.9)", () => {
    const compiled = withSlots(compileOkForReadoutTests(), { digitalSlots: ["u1/qa", "u1/qb", "u2"] });
    const digitalState = buildDigitalState(compiled, [1, Number.NaN, 0]);
    expect(digitalState).toEqual({ "u1/qa": 1, u2: 0 });
  });

  it("buildElementChannelI groups channel slots by componentId, indexed by channel", () => {
    const compiled = withSlots(compileOkForReadoutTests(), {
      channelSlots: [
        ["u1", 0],
        ["u1", 1],
        ["u2", 0],
      ],
    });
    const elementChannelI = buildElementChannelI(compiled, [0.1, 0.2, 0.3]);
    expect(elementChannelI).toEqual({ u1: [0.1, 0.2], u2: [0.3] });
  });

  // core/README.md's "Readouts": "a component whose slots are NaN is
  // omitted, as classic omits a burnt or failed rgb_led/bicolor_led" —
  // W1e's LEDs point 13. buildElementChannelI must skip the whole component,
  // not publish an array of NaNs, exactly as classic writes no key at all.
  it("buildElementChannelI omits a component whose channel slots are NaN (a burnt rgb_led/bicolor_led, W1e)", () => {
    const compiled = withSlots(compileOkForReadoutTests(), {
      channelSlots: [
        ["u1", 0],
        ["u1", 1],
        ["u1", 2],
        ["u2", 0],
      ],
    });
    const elementChannelI = buildElementChannelI(compiled, [Number.NaN, Number.NaN, Number.NaN, 0.3]);
    expect(elementChannelI).toEqual({ u2: [0.3] });
    expect(elementChannelI).not.toHaveProperty("u1");
  });
});

// ---- elementI key set matches classic's _updateElementI (spec §10.5) ------

describe("readout.ts's buildElementI key set — matches classic's _updateElementI on real fixtures", () => {
  const FIXTURES_DIR = path.resolve(__dirname, "../fixtures/circuits");
  const fixtureNames = fs.readdirSync(FIXTURES_DIR).filter((name) => name.endsWith(".sim.json"));

  it("found at least one fixture to check", () => {
    expect(fixtureNames.length).toBeGreaterThan(0);
  });

  for (const name of fixtureNames) {
    it(`key set for ${name}: every component with >=2 pins, nothing else`, () => {
      const raw = JSON.parse(fs.readFileSync(path.join(FIXTURES_DIR, name), "utf8")) as SimCircuit;
      const classic = new SimEngine();
      classic.load(raw);
      const classicKeys = new Set(Object.keys(classic.getElementI()));

      const ourKeys = new Set(classicElementIKeys(raw, buildCoreNodes(raw).nodeCount));
      expect(ourKeys).toEqual(classicKeys);
    });
  }

  it("a <2-pin component (ground) gets no elementI entry at all, on either side", () => {
    const c = circuit([
      part("gref", "ground", ["gnd"]),
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", ["a", "b"], { resistance: 100 }),
    ]);
    const classic = new SimEngine();
    classic.load(c);
    expect(Object.keys(classic.getElementI())).not.toContain("gref");

    const result = compileCircuit(c);
    if (!result.ok) throw new Error("expected ok:true");
    const ourKeys = Object.keys(buildElementI(result.compiled, result.compiled.devices.map(() => 0)));
    expect(ourKeys).not.toContain("gref");
    expect(new Set(ourKeys)).toEqual(new Set(["vsrc", "r1"]));
  });

  it("a circuit whose only node is the datum publishes no elementI at all, like classic", () => {
    // Classic names the net holding a source's return pin "gnd"; with no
    // source, a board ground is the fallback reference. A resistor with both
    // ends on that one net leaves the MNA with zero rows.
    const c = circuit(
      [part("gref", "ground", ["gnd"]), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("r1", "a", "gref", "gnd"), wire("r1", "b", "gref", "gnd")],
    );
    const classic = new SimEngine();
    classic.load(c);
    const nodeCount = buildCoreNodes(c).nodeCount;
    expect(classicElementIKeys(c, nodeCount)).toEqual(Object.keys(classic.getElementI()));
  });
});

// ---- v2: CoreEngine.load's options passthrough -----------------------------
// Exercised via a synthetic bo_engine_load that captures the uploaded program
// bytes instead of a real Rust decode (the real ABI is still v1 — see the
// worker brief's "Integration timing"), then decodes them in TS to check the
// option block compileCircuit actually wrote.

describe("CoreEngine.load — options passthrough to compileCircuit (integrationMethod, ambient)", () => {
  afterEach(async () => {
    __resetCoreForTest();
    await initCore();
  });

  function installCapturingCore(): { capturedProgram: () => Float64Array | undefined } {
    let captured: Float64Array | undefined;
    const buffer = new ArrayBuffer(1 << 20);
    const exports: CoreExports = {
      memory: { buffer },
      bo_abi_version: () => 2,
      bo_alloc: (lenF64: number) => 0,
      bo_free: () => {},
      bo_engine_create: () => 1,
      bo_engine_destroy: () => {},
      bo_engine_load: (_h: number, ptr: number, lenF64: number) => {
        captured = Float64Array.from(new Float64Array(buffer, ptr, lenF64));
        return 0;
      },
      bo_engine_advance: () => 0,
      bo_engine_time: () => 0,
      bo_engine_node_count: () => 0,
      bo_engine_node_voltages: () => 0,
      bo_engine_device_count: () => 0,
      bo_engine_device_currents: () => 0,
      bo_engine_stat: () => 0,
      bo_error_ptr: () => 0,
      bo_error_len: () => 0,
      bo_engine_set_option: () => 0,
      bo_engine_next_event_time: () => Infinity,
      bo_engine_next_digital_due_time: () => Infinity,      bo_engine_digital_slot_count: () => 0,
      bo_engine_digital_slots: () => 0,
      bo_engine_channel_slot_count: () => 0,
      bo_engine_channel_slots: () => 0,
      bo_engine_trace_enable: () => 0,
      bo_engine_trace_ptr: () => 0,
      bo_engine_trace_len: () => 0,
      bo_engine_trace_clear: () => {},
      bo_engine_seed_memory: () => 0,
      bo_engine_take_eeprom_updates: () => 0,
      bo_engine_eeprom_updates_ptr: () => 0,
      bo_engine_set_params: () => 0,
    };
    __installCoreForTest(exports);
    return { capturedProgram: () => captured };
  }

  function divider(): SimCircuit {
    return circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
  }

  it("defaults to backward Euler and ambient 25 when no options are given", () => {
    const { capturedProgram } = installCapturingCore();
    const core = new CoreEngine();
    core.load(divider());
    const decoded = decodeProgram(capturedProgram()!);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.INTEGRATION, 0]);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.AMBIENT_C, 25]);
  });

  it("passes integrationMethod: 'trap' through to the compiled program's INTEGRATION option", () => {
    const { capturedProgram } = installCapturingCore();
    const core = new CoreEngine();
    core.load(divider(), undefined, { integrationMethod: "trap" });
    const decoded = decodeProgram(capturedProgram()!);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.INTEGRATION, 1]);
  });

  it("derives AMBIENT_C from circuit.environment.temperatureC, not from a load option", () => {
    const { capturedProgram } = installCapturingCore();
    const c = divider();
    c.environment = { temperatureC: 45 };
    const core = new CoreEngine();
    core.load(c);
    const decoded = decodeProgram(capturedProgram()!);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.AMBIENT_C, 45]);
  });

  it("simTime does not disturb the OTHER compile options: options.simTime itself never reaches compileCircuit as one", () => {
    const { capturedProgram } = installCapturingCore();
    const core = new CoreEngine();
    core.load(divider(), undefined, { simTime: 12, integrationMethod: "trap" });
    expect(core.simTime).toBe(12);
    const decoded = decodeProgram(capturedProgram()!);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.INTEGRATION, 1]);
  });

  // Finding 3 (w1a-fixes/ts.json #3): before the fix, CORE_OPTION_KEYS had no
  // START_TIME entry at all and load() never wrote one — a live-edit reload
  // restarted Rust's own clock at 0 while the facade's simTime carried
  // forward, with no ABI field to tell Rust the difference. The fix: every
  // load() writes START_TIME = the carried-over simTime (0 on the first
  // load), so Rust's own clock is seeded absolute too.
  it("writes START_TIME = the carried-over simTime on every load, including the very first (0)", () => {
    const { capturedProgram } = installCapturingCore();
    const core = new CoreEngine();
    core.load(divider());
    expect(decodeProgram(capturedProgram()!).options).toContainEqual([CORE_OPTION_KEYS.START_TIME, 0]);
  });

  it("writes START_TIME = options.simTime when a reload seeds the clock explicitly", () => {
    const { capturedProgram } = installCapturingCore();
    const core = new CoreEngine();
    core.load(divider(), undefined, { simTime: 7.5 });
    expect(decodeProgram(capturedProgram()!).options).toContainEqual([CORE_OPTION_KEYS.START_TIME, 7.5]);
  });

  it("writes START_TIME = the facade's own clock on a bare reload (no options.simTime given)", () => {
    const { capturedProgram } = installCapturingCore();
    const core = new CoreEngine();
    core.load(divider(), undefined, { simTime: 3 });
    core.advance(4);
    core.load(divider()); // reload with no simTime override: carries 4 forward
    expect(decodeProgram(capturedProgram()!).options).toContainEqual([CORE_OPTION_KEYS.START_TIME, 4]);
  });
});

// ---- v2: advance() and the removal of the _loadedAt translation ------------
// (w1a-fixes/ts.json findings 3 and 4)

describe("advance() sends tEnd straight through to Rust — no more _loadedAt translation", () => {
  afterEach(async () => {
    __resetCoreForTest();
    await initCore();
  });

  function divider(): SimCircuit {
    return circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
  }

  function installFakeCore(
    advanceImpl: (tEnd: number) => number,
    timeImpl: () => number = () => 0,
  ): { calls: number[]; voltagesView: Float64Array } {
    const buffer = new ArrayBuffer(1 << 16);
    const voltagesView = new Float64Array(buffer, 0, 8);
    voltagesView.fill(1.0);
    const calls: number[] = [];
    const exports: CoreExports = {
      memory: { buffer },
      bo_abi_version: () => 2,
      bo_alloc: () => 0,
      bo_free: () => {},
      bo_engine_create: () => 1,
      bo_engine_destroy: () => {},
      bo_engine_load: () => 0,
      bo_engine_advance: (_h: number, tEnd: number) => {
        calls.push(tEnd);
        return advanceImpl(tEnd);
      },
      bo_engine_time: () => timeImpl(),
      bo_engine_node_count: () => 0,
      bo_engine_node_voltages: () => 0,
      bo_engine_device_count: () => 0,
      bo_engine_device_currents: () => 0,
      bo_engine_stat: () => 0,
      bo_error_ptr: () => 0,
      bo_error_len: () => 0,
      bo_engine_set_option: () => 0,
      bo_engine_next_event_time: () => Infinity,
      bo_engine_next_digital_due_time: () => Infinity,      bo_engine_digital_slot_count: () => 0,
      bo_engine_digital_slots: () => 0,
      bo_engine_channel_slot_count: () => 0,
      bo_engine_channel_slots: () => 0,
      bo_engine_trace_enable: () => 0,
      bo_engine_trace_ptr: () => 0,
      bo_engine_trace_len: () => 0,
      bo_engine_trace_clear: () => {},
      bo_engine_seed_memory: () => 0,
      bo_engine_take_eeprom_updates: () => 0,
      bo_engine_eeprom_updates_ptr: () => 0,
      bo_engine_set_params: () => 0,
    };
    __installCoreForTest(exports);
    return { calls, voltagesView };
  }

  it("advance(tEnd) reaches bo_engine_advance untranslated, even after a nonzero-time reload", () => {
    // Before the fix, this would have called bo_engine_advance with
    // `tEnd - _loadedAt` = 103 - 100 = 3. Rust's own clock is now seeded
    // absolute via START_TIME at load time, so tEnd reaches it directly.
    const { calls } = installFakeCore(() => 0);
    const core = new CoreEngine();
    core.load(divider(), undefined, { simTime: 100 });
    core.advance(103);
    expect(calls).toEqual([103]);
  });

  // Finding 4 (w1a-fixes/ts.json #4): -4/-7 from advance() used to throw a
  // generic CoreLoadError and leave `lastConverged` at its previous (true)
  // value. core/README.md's "A failed advance" says this is non-convergence,
  // not a load error, exactly like classic's `lastConverged = false`.
  for (const code of [-4, -7]) {
    it(`code ${code} does not throw: lastConverged becomes false and simTime reads back the last completed instant`, () => {
      installFakeCore(() => code, () => 12.25);
      const core = new CoreEngine();
      core.load(divider());
      expect(() => core.advance(20)).not.toThrow();
      expect(core.lastConverged).toBe(false);
      expect(core.simTime).toBe(12.25); // read back via bo_engine_time(), not the requested tEnd
    });
  }

  it("a non-convergent advance() leaves the published readouts exactly as the last successful call left them", () => {
    let calls = 0;
    const { voltagesView } = installFakeCore(() => (calls++ === 0 ? 0 : -7));
    const core = new CoreEngine();
    core.load(divider());
    core.advance(10); // succeeds: publishes voltagesView's current contents
    const netVBefore = { ...core.getNetV() };
    // Mutate the underlying "Rust" memory, as if the instance's own state had
    // moved on: if the non-convergent call below wrongly re-published
    // readouts, getNetV() would pick this up. It must not.
    voltagesView.fill(999);
    core.advance(20); // -7: must not throw, must not republish
    expect(core.lastConverged).toBe(false);
    expect(core.getNetV()).toEqual(netVBefore);
  });
});

// ---- v2: nextEventTime() / getElementChannelI() / digitalState via synthetic exports

describe("CoreEngine — v2 additions (nextEventTime, getElementChannelI, digitalState), via synthetic exports", () => {
  afterEach(async () => {
    __resetCoreForTest();
    await initCore();
  });

  function divider(): SimCircuit {
    return circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );
  }

  function installFakeCore(overrides: Partial<CoreExports> = {}): void {
    const buffer = new ArrayBuffer(1 << 16);
    const base: CoreExports = {
      memory: { buffer },
      bo_abi_version: () => 2,
      bo_alloc: () => 0,
      bo_free: () => {},
      bo_engine_create: () => 1,
      bo_engine_destroy: () => {},
      bo_engine_load: () => 0,
      bo_engine_advance: () => 0,
      bo_engine_time: () => 0,
      bo_engine_node_count: () => 0,
      bo_engine_node_voltages: () => 0,
      bo_engine_device_count: () => 0,
      bo_engine_device_currents: () => 0,
      bo_engine_stat: () => 0,
      bo_error_ptr: () => 0,
      bo_error_len: () => 0,
      bo_engine_set_option: () => 0,
      bo_engine_next_event_time: () => Infinity,
      bo_engine_next_digital_due_time: () => Infinity,      bo_engine_digital_slot_count: () => 0,
      bo_engine_digital_slots: () => 0,
      bo_engine_channel_slot_count: () => 0,
      bo_engine_channel_slots: () => 0,
      bo_engine_trace_enable: () => 0,
      bo_engine_trace_ptr: () => 0,
      bo_engine_trace_len: () => 0,
      bo_engine_trace_clear: () => {},
      bo_engine_seed_memory: () => 0,
      bo_engine_take_eeprom_updates: () => 0,
      bo_engine_eeprom_updates_ptr: () => 0,
      bo_engine_set_params: () => 0,
    };
    __installCoreForTest({ ...base, ...overrides });
  }

  it("nextEventTime() is +Infinity for W1a's static kinds (nothing scheduled)", () => {
    installFakeCore();
    const core = new CoreEngine();
    core.load(divider());
    expect(core.nextEventTime()).toBe(Infinity);
  });

  it("nextEventTime() returns Rust's own reported time UNTRANSLATED — Rust's clock is absolute now (START_TIME), so there is no facade offset left to add", () => {
    // Finding 3 (w1a-fixes/ts.json #3): before the fix, Rust always restarted
    // its internal clock at 0 on load, so this method had to translate its
    // answer through `_loadedAt`. Now load() seeds START_TIME with the
    // carried-over simTime, so Rust's own clock IS the facade's clock and
    // `bo_engine_next_event_time`'s return value needs no adjustment. A fake
    // core can't simulate START_TIME actually taking effect inside Rust, but
    // it CAN prove the facade itself no longer adds anything on top — if it
    // did, this would read 10.002, not 0.002.
    installFakeCore({ bo_engine_next_event_time: () => 0.002 });
    const core = new CoreEngine();
    core.load(divider(), undefined, { simTime: 10 });
    expect(core.nextEventTime()).toBeCloseTo(0.002, 12);
  });

  it("getElementChannelI() is {} for W1a's kinds (no channel slot ever claimed)", () => {
    installFakeCore();
    const core = new CoreEngine();
    core.load(divider());
    expect(core.getElementChannelI()).toEqual({});
  });

  it("digitalState is {} for W1a's kinds (no digital slot ever claimed), not a hardcoded stub — it is read through the same slot machinery", () => {
    installFakeCore();
    const core = new CoreEngine();
    core.load(divider());
    expect(core.digitalState).toEqual({});
  });

  it("getElementChannelI()/digitalState/nextEventTime() all require a loaded program, like every other readout", () => {
    installFakeCore();
    const core = new CoreEngine();
    expect(() => core.getElementChannelI()).toThrow(CoreNotLoadedError);
    expect(() => core.digitalState).toThrow(CoreNotLoadedError);
    expect(() => core.nextEventTime()).toThrow(CoreNotLoadedError);
  });
});

// ---- v2 (W1b): CoreEngine.setParams — mapping, live-param gating, and errors
//
// `switch`/`spdt_switch`/`dip_switch` are not yet in CORE_IMPLEMENTED_KINDS
// (kinds.ts, the orchestrator's table), so every load() below passes a
// widened set — the same documented escape hatch compileCircuit's own doc
// comment describes. `bo_engine_set_params` is faked (it captures the raw
// triples instead of a real Rust decode), matching this file's existing
// "installCapturingCore" convention for exercising the TS-side contract
// before/independent of a specific Rust build.

describe("CoreEngine.setParams", () => {
  afterEach(async () => {
    __resetCoreForTest();
    await initCore();
  });

  const WITH_PASSIVES = new Set([...CORE_IMPLEMENTED_KINDS, "switch", "spdt_switch", "dip_switch"]);

  /** vsrc, sw (switch), spdt (spdt_switch), dip (dip_switch, k=2), r1 — deliberately unwired: this suite never solves the circuit, only exercises setParams' own mapping/validation. Component order fixes device order: vsrc=0, sw=1, spdt=2, dip=3, r1=4. */
  function passivesCircuit(): SimCircuit {
    return circuit([
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("sw", "switch", ["a", "b"], { closed: 0 }),
      part("spdt", "spdt_switch", ["com", "b", "c"], { position: 0 }),
      part("dip", "dip_switch", ["a1", "b1", "a2", "b2"], { positions: 2, sw1: 0, sw2: 0 }),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    ]);
  }

  function installFakeCore(overrides: Partial<CoreExports> = {}): {
    capturedTriples: () => number[];
    setParamsCalls: () => number;
    voltagesReadCalls: () => number;
  } {
    const buffer = new ArrayBuffer(1 << 16);
    let captured: number[] = [];
    let setParamsCalls = 0;
    let voltagesReadCalls = 0;
    const base: CoreExports = {
      memory: { buffer },
      bo_abi_version: () => 2,
      bo_alloc: () => 1024, // fixed scratch offset, well past the voltages/currents views below
      bo_free: () => {},
      bo_engine_create: () => 1,
      bo_engine_destroy: () => {},
      bo_engine_load: () => 0,
      bo_engine_advance: () => 0,
      bo_engine_time: () => 0,
      bo_engine_node_count: () => 0,
      bo_engine_node_voltages: () => {
        voltagesReadCalls++;
        return 0; // byte offset 0
      },
      bo_engine_device_count: () => 0,
      bo_engine_device_currents: () => 512, // byte offset, past any plausible node count here
      bo_engine_stat: () => 0,
      bo_error_ptr: () => 0,
      bo_error_len: () => 0,
      bo_engine_set_option: () => 0,
      bo_engine_next_event_time: () => Infinity,
      bo_engine_next_digital_due_time: () => Infinity,      bo_engine_digital_slot_count: () => 0,
      bo_engine_digital_slots: () => 0,
      bo_engine_channel_slot_count: () => 0,
      bo_engine_channel_slots: () => 0,
      bo_engine_trace_enable: () => 0,
      bo_engine_trace_ptr: () => 0,
      bo_engine_trace_len: () => 0,
      bo_engine_trace_clear: () => {},
      bo_engine_seed_memory: () => 0,
      bo_engine_take_eeprom_updates: () => 0,
      bo_engine_eeprom_updates_ptr: () => 0,
      bo_engine_set_params: (_h: number, ptr: number, lenF64: number) => {
        setParamsCalls++;
        captured = Array.from(new Float64Array(buffer, ptr, lenF64));
        return 0;
      },
    };
    __installCoreForTest({ ...base, ...overrides });
    return {
      capturedTriples: () => captured,
      setParamsCalls: () => setParamsCalls,
      voltagesReadCalls: () => voltagesReadCalls,
    };
  }

  it("maps (componentId, paramName) to (deviceIndex, paramIndex), resolves values the same way the compiler does, and sends every triple in ONE call", () => {
    const { capturedTriples, setParamsCalls } = installFakeCore();
    const core = new CoreEngine();
    core.load(passivesCircuit(), WITH_PASSIVES);

    core.setParams([
      { componentId: "sw", params: { closed: 1 } }, // device 1, param 0 (closed) -> Boolean(1) = 1
      { componentId: "spdt", params: { position: 1 } }, // device 2, param 0 (sel) -> position!==0 selects c = 1
      { componentId: "dip", params: { sw2: 1 } }, // device 3, param 2 (sw2) -> Number(1)!==0 = 1
    ]);

    expect(setParamsCalls()).toBe(1);
    expect(capturedTriples()).toEqual([
      1, 0, 1,
      2, 0, 1,
      3, 2, 1,
    ]);
  });

  it("the string \"0\" resolves to CLOSED (1), matching the compiler's own Boolean(closed) rule", () => {
    const { capturedTriples } = installFakeCore();
    const core = new CoreEngine();
    core.load(passivesCircuit(), WITH_PASSIVES);
    core.setParams([{ componentId: "sw", params: { closed: "0" } }]);
    expect(capturedTriples()).toEqual([1, 0, 1]);
  });

  it("republishes readouts (re-reads node voltages) after a successful call", () => {
    const { voltagesReadCalls } = installFakeCore();
    const core = new CoreEngine();
    core.load(passivesCircuit(), WITH_PASSIVES);
    const afterLoad = voltagesReadCalls();
    core.setParams([{ componentId: "sw", params: { closed: 1 } }]);
    expect(voltagesReadCalls()).toBe(afterLoad + 1);
  });

  it("an empty changes array makes no ABI call and does not republish", () => {
    const { setParamsCalls, voltagesReadCalls } = installFakeCore();
    const core = new CoreEngine();
    core.load(passivesCircuit(), WITH_PASSIVES);
    const afterLoad = voltagesReadCalls();
    expect(() => core.setParams([])).not.toThrow();
    expect(setParamsCalls()).toBe(0);
    expect(voltagesReadCalls()).toBe(afterLoad);
  });

  it("throws for an unknown componentId, without making any ABI call", () => {
    const { setParamsCalls } = installFakeCore();
    const core = new CoreEngine();
    core.load(passivesCircuit(), WITH_PASSIVES);
    expect(() => core.setParams([{ componentId: "no-such-component", params: { closed: 1 } }])).toThrow(RangeError);
    expect(setParamsCalls()).toBe(0);
  });

  it("throws for an unknown param name on a real device's kind, without making any ABI call", () => {
    const { setParamsCalls } = installFakeCore();
    const core = new CoreEngine();
    core.load(passivesCircuit(), WITH_PASSIVES);
    expect(() => core.setParams([{ componentId: "sw", params: { not_a_real_param: 1 } }])).toThrow(RangeError);
    expect(setParamsCalls()).toBe(0);
  });

  it("throws for a param that exists on the kind but is NOT live — a component with no live params at all (resistor), and a non-live param on a kind that DOES have live params (dip_switch's own \"positions\")", () => {
    const { setParamsCalls } = installFakeCore();
    const core = new CoreEngine();
    core.load(passivesCircuit(), WITH_PASSIVES);
    expect(() => core.setParams([{ componentId: "r1", params: { resistance: 5000 } }])).toThrow(RangeError);
    expect(() => core.setParams([{ componentId: "dip", params: { positions: 4 } }])).toThrow(RangeError);
    expect(setParamsCalls()).toBe(0);
  });

  it("validates every change before applying ANY of them — one bad change in a batch makes no ABI call at all", () => {
    const { setParamsCalls } = installFakeCore();
    const core = new CoreEngine();
    core.load(passivesCircuit(), WITH_PASSIVES);
    expect(() =>
      core.setParams([
        { componentId: "sw", params: { closed: 1 } }, // valid
        { componentId: "r1", params: { resistance: 5000 } }, // invalid: not live
      ]),
    ).toThrow(RangeError);
    expect(setParamsCalls()).toBe(0);
  });

  it("a non-zero bo_engine_set_params return throws CoreLoadError, carrying the ABI's own error message", () => {
    const encoder = new TextEncoder();
    const message = "invalid argument";
    const messageBytes = encoder.encode(message);
    installFakeCore({
      bo_engine_set_params: () => -8,
      bo_error_ptr: () => 2048,
      bo_error_len: () => messageBytes.length,
    });
    // Write the error message into the same backing buffer bo_error_ptr points at.
    const core = new CoreEngine();
    core.load(passivesCircuit(), WITH_PASSIVES);
    // Reach into the live instance's memory to seed the error bytes at 2048 —
    // coreCall is the only sanctioned way in from outside engine.ts/wasm.ts.
    coreCall((exports) => new Uint8Array(exports.memory.buffer, 2048, messageBytes.length).set(messageBytes));
    expect(() => core.setParams([{ componentId: "sw", params: { closed: 1 } }])).toThrow(/invalid argument/);
  });

  it("setParams requires a loaded program, like every other mutating call", () => {
    installFakeCore();
    const core = new CoreEngine();
    expect(() => core.setParams([{ componentId: "sw", params: { closed: 1 } }])).toThrow(CoreNotLoadedError);
  });
});
