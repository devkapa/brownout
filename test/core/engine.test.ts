import { afterAll, afterEach, beforeAll, describe, expect, it, vi } from "vitest";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import {
  coreCall,
  getWasmApi,
  initCore,
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
      bo_abi_version: () => 1,
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
