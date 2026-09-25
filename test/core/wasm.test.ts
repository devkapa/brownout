import { afterEach, describe, expect, it, vi } from "vitest";
import fs from "node:fs";
import path from "node:path";
import { pathToFileURL } from "node:url";
import {
  CORE_ABI_VERSION,
  coreCall,
  coreGeneration,
  coreReady,
  coreStatus,
  getWasmApi,
  initCore,
  readCoreErrorMessage,
  verifyCoreExports,
  type CoreExports,
  type WasmMemoryLike,
  __installCoreForTest,
  __resetCoreForTest,
} from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import {
  CoreInitError,
  CoreNotInitializedError,
  CorePoisonedError,
  CoreStaleInstanceError,
  CoreTrapError,
  CoreUnsupportedError,
} from "../../src/core/errors.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

// vitest's globalSetup (test/setup/build-core.ts) rebuilds this before any
// test file runs, so it always reflects the current Rust source.
const WASM_PATH = path.resolve(__dirname, "../../src/core/brownout-core.wasm");

function readWasmBytes(): Buffer {
  return fs.readFileSync(WASM_PATH);
}

function tinyCircuit(): SimCircuit {
  return {
    components: [
      { id: "vsrc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
    ],
    wires: [
      { from_component: "vsrc", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "vsrc", to_pin: "neg" },
    ],
  };
}

describe("initCore — accepted sources (against the real built artifact)", () => {
  afterEach(() => {
    __resetCoreForTest();
  });

  it("default source: resolves the file: URL and instantiates in Node", async () => {
    expect(coreReady()).toBe(false);
    await initCore();
    expect(coreReady()).toBe(true);
    expect(coreCall((exports) => exports.bo_abi_version())).toBe(CORE_ABI_VERSION);
  });

  it("accepts raw bytes (ArrayBuffer)", async () => {
    const bytes = readWasmBytes();
    const arrayBuffer = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
    await initCore(arrayBuffer);
    expect(coreReady()).toBe(true);
  });

  it("accepts an ArrayBufferView (Buffer/Uint8Array)", async () => {
    await initCore(readWasmBytes());
    expect(coreReady()).toBe(true);
  });

  it("accepts a pre-compiled wasm module", async () => {
    const module = await getWasmApi().compile(readWasmBytes());
    await initCore(module);
    expect(coreReady()).toBe(true);
  });

  it("accepts a Response (e.g. from fetch())", async () => {
    const bytes = readWasmBytes();
    const response = new Response(bytes, { headers: { "content-type": "application/wasm" } });
    await initCore(response);
    expect(coreReady()).toBe(true);
  });

  it("accepts a PromiseLike<Response>", async () => {
    const bytes = readWasmBytes();
    const responsePromise = Promise.resolve(new Response(bytes));
    await initCore(responsePromise);
    expect(coreReady()).toBe(true);
  });

  // CoreWasmSource's `PromiseLike` member wraps every direct-source member,
  // not just Response (see the type's own doc comment) — a caller writing
  // `fetch(url).then(r => r.arrayBuffer())`, or handing over a cached
  // `Promise<ArrayBuffer>`, must get the same treatment as passing that value
  // directly, not an assumption that the resolved value is a Response.
  it("accepts a PromiseLike<ArrayBuffer> (not just PromiseLike<Response>)", async () => {
    const bytes = readWasmBytes();
    const arrayBuffer = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
    await initCore(Promise.resolve(arrayBuffer));
    expect(coreReady()).toBe(true);
  });
});

describe("the built module itself", () => {
  it("has zero imports (core/README.md, C ABI v2)", () => {
    const WA = getWasmApi();
    const module = new WA.Module(readWasmBytes());
    expect(WA.Module.imports(module)).toEqual([]);
  });

  it("exports memory and every C ABI v2 (W1a+W1b) function, and NOT seed_memory yet", () => {
    const WA = getWasmApi();
    const module = new WA.Module(readWasmBytes());
    const names = new Set(WA.Module.exports(module).map((e: { name: string }) => e.name));
    for (const name of [
      "memory",
      "bo_abi_version",
      "bo_alloc",
      "bo_free",
      "bo_engine_create",
      "bo_engine_destroy",
      "bo_engine_load",
      "bo_engine_advance",
      "bo_engine_time",
      "bo_engine_node_count",
      "bo_engine_node_voltages",
      "bo_engine_device_count",
      "bo_engine_device_currents",
      "bo_engine_stat",
      "bo_error_ptr",
      "bo_error_len",
      "bo_engine_set_option",
      "bo_engine_next_event_time",
      "bo_engine_digital_slot_count",
      "bo_engine_digital_slots",
      "bo_engine_channel_slot_count",
      "bo_engine_channel_slots",
      "bo_engine_trace_enable",
      "bo_engine_trace_ptr",
      "bo_engine_trace_len",
      "bo_engine_trace_clear",
      "bo_engine_set_params",
    ]) {
      expect(names.has(name)).toBe(true);
    }
    // W1d's row of the same ABI table — must not exist yet.
    expect(names.has("bo_engine_seed_memory")).toBe(false);
  });
});

/** A structurally complete, real-typed fake CoreExports for verifyCoreExports/poison tests — a real WebAssembly.Memory (works standalone, no wasm module needed) plus stub functions. */
function fakeExports(overrides: Partial<CoreExports> = {}): CoreExports {
  const base: CoreExports = {
    memory: new (getWasmApi().Memory)({ initial: 1 }),
    bo_abi_version: () => CORE_ABI_VERSION,
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
    bo_engine_digital_slot_count: () => 0,
    bo_engine_digital_slots: () => 0,
    bo_engine_channel_slot_count: () => 0,
    bo_engine_channel_slots: () => 0,
    bo_engine_trace_enable: () => 0,
    bo_engine_trace_ptr: () => 0,
    bo_engine_trace_len: () => 0,
    bo_engine_trace_clear: () => {},
    bo_engine_set_params: () => 0,
  };
  return { ...base, ...overrides };
}

/** verifyCoreExports takes the pre-verification `Record<string, unknown>` shape (what a real `instance.exports` is), not the post-verification `CoreExports` — this is the cast a real caller's `instance.exports` needs too. */
function asRaw(exports: CoreExports): Record<string, unknown> {
  return exports as unknown as Record<string, unknown>;
}

describe("verifyCoreExports — the internal ABI-verification seam", () => {
  it("rejects a mismatched ABI version, naming both versions", () => {
    const raw = asRaw(fakeExports({ bo_abi_version: () => 999 }));
    expect(() => verifyCoreExports(raw)).toThrow(CoreInitError);
    expect(() => verifyCoreExports(raw)).toThrow(/999/);
    expect(() => verifyCoreExports(raw)).toThrow(new RegExp(String(CORE_ABI_VERSION)));
  });

  it("rejects a missing contract export, naming it", () => {
    const raw = asRaw(fakeExports());
    delete raw.bo_engine_advance;
    expect(() => verifyCoreExports(raw)).toThrow(CoreInitError);
    expect(() => verifyCoreExports(raw)).toThrow(/bo_engine_advance/);
  });

  it("rejects a missing/wrong-typed memory export", () => {
    const raw = asRaw(fakeExports({ memory: {} as unknown as WasmMemoryLike }));
    expect(() => verifyCoreExports(raw)).toThrow(/memory/);
  });

  it("accepts a fully conforming exports object", () => {
    expect(() => verifyCoreExports(asRaw(fakeExports()))).not.toThrow();
  });
});

describe("poison flow via the seam (synthetic trap — the public ABI has no supported way to provoke a real one)", () => {
  afterEach(() => {
    __resetCoreForTest();
  });

  it("trap -> CoreTrapError -> CorePoisonedError on later calls -> re-init recovers -> old CoreEngines refuse to run", async () => {
    await initCore();
    const generationBeforePoison = coreGeneration();

    const engine = new CoreEngine();
    engine.load(tinyCircuit());
    expect(engine.getNetV()).toMatchObject({ n0: 5 });

    // A synthetic trap: the real ABI has no documented way to provoke a
    // genuine Rust panic (every documented failure returns a negative code),
    // so this is the seam the contract's "Traps" section exists to make
    // testable at all.
    expect(() =>
      coreCall(() => {
        throw new (getWasmApi().RuntimeError)("synthetic trap for test");
      }),
    ).toThrow(CoreTrapError);

    expect(coreReady()).toBe(false);
    expect(() => engine.advance(1)).toThrow(CorePoisonedError);
    expect(() => engine.getNetV()).toThrow(CorePoisonedError);

    // Re-init produces a fresh instance with a new generation.
    await initCore();
    expect(coreReady()).toBe(true);
    expect(coreGeneration()).toBe(generationBeforePoison + 1);

    // The OLD engine refuses to run against the new instance — never silent
    // misbehaviour against a handle number that means something else now.
    expect(() => engine.advance(1)).toThrow(CoreStaleInstanceError);
    expect(() => engine.getNetV()).toThrow(CoreStaleInstanceError);

    // A brand new engine works fine against the fresh instance.
    const engine2 = new CoreEngine();
    engine2.load(tinyCircuit());
    expect(engine2.getNetV()).toMatchObject({ n0: 5 });
  });

  it("__installCoreForTest bumps the generation and clears a poisoned state without a real instantiation", () => {
    __resetCoreForTest();
    expect(coreReady()).toBe(false);
    __installCoreForTest(fakeExports());
    expect(coreReady()).toBe(true);
    const gen1 = coreGeneration();

    expect(() =>
      coreCall(() => {
        throw new (getWasmApi().RuntimeError)("boom");
      }),
    ).toThrow(CoreTrapError);
    expect(coreReady()).toBe(false);
    expect(() => coreCall((exports) => exports.bo_abi_version())).toThrow(CorePoisonedError);

    __installCoreForTest(fakeExports());
    expect(coreReady()).toBe(true);
    expect(coreGeneration()).toBe(gen1 + 1);
  });

  it("coreCall refuses uninitialized state with CoreNotInitializedError", () => {
    __resetCoreForTest();
    expect(() => coreCall((exports) => exports.bo_abi_version())).toThrow(CoreNotInitializedError);
  });

  // POISON BACKSTOP (coreCall's own doc comment): the trap path must not be
  // special-cased to WebAssembly.RuntimeError. A wasm stack overflow surfaces
  // to JS as a plain RangeError, not a RuntimeError, and the Rust state after
  // an aborted call is exactly as untrustworthy either way.
  it("poisons the instance on ANY thrown value that is not a Core* error class, e.g. a RangeError (a wasm stack overflow), not just RuntimeError", async () => {
    await initCore();
    const original = new RangeError("Maximum call stack size exceeded");
    let caught: unknown;
    try {
      coreCall(() => {
        throw original;
      });
    } catch (err) {
      caught = err;
    }
    expect(caught).toBeInstanceOf(CoreTrapError);
    expect((caught as Error).cause).toBe(original);
    expect(coreReady()).toBe(false);
    expect(coreStatus()).toBe("poisoned");
  });

  // The other half of the same backstop: a Core* error is a definite,
  // fully-handled outcome one of engine.ts's own closures threw on purpose
  // (e.g. mapLoadFailure's CoreUnsupportedError) — it describes the Rust call
  // returning normally, not the instance breaking, so it must pass through
  // coreCall completely unchanged and must NOT poison anything.
  it("lets a Core* error thrown by the callback pass straight through, unwrapped, without poisoning the instance", async () => {
    await initCore();
    const original = new CoreUnsupportedError(["topology:test"]);
    let caught: unknown;
    try {
      coreCall(() => {
        throw original;
      });
    } catch (err) {
      caught = err;
    }
    expect(caught).toBe(original); // the exact same instance, not re-wrapped
    expect(coreReady()).toBe(true);
    expect(coreStatus()).toBe("ready");
  });
});

describe("pointer sign coercion (>>> 0) — every ABI pointer export is an unsigned wasm32 address", () => {
  afterEach(() => {
    __resetCoreForTest();
  });

  // bo_error_ptr() (like every other pointer export) returns an i32: an
  // address at or above 2^31 crosses the ABI as a NEGATIVE JS number, which a
  // typed-array view built on it directly would reject outright. This
  // reproduces that exact address, without needing to actually grow a real
  // wasm instance's memory past 2 GiB (a single large ArrayBuffer allocation
  // is virtual/lazily-paged, so this is cheap despite the address size).
  it("readCoreErrorMessage resolves a message at an address at/above 2^31 (not a negative offset that throws)", () => {
    const message = "boom at a high address";
    const bytes = new TextEncoder().encode(message);
    const ptrAddress = 2 ** 31; // 0x80000000 — the smallest address that wraps
    const buffer = new ArrayBuffer(ptrAddress + bytes.length + 8);
    new Uint8Array(buffer, ptrAddress, bytes.length).set(bytes);

    const exports = fakeExports({
      memory: { buffer },
      // The i32 bit-pattern a real wasm32 export would hand back to JS for
      // address 0x80000000 — exactly what bo_error_ptr() returns once the
      // Rust heap has grown past 2 GiB (the finding this proves fixed).
      bo_error_ptr: () => -ptrAddress,
      bo_error_len: () => bytes.length,
    });

    expect(readCoreErrorMessage(exports)).toBe(message);
  });
});

describe("initCore concurrency / idempotency (fresh module instances via vi.resetModules)", () => {
  it("a second call after success resolves immediately (same generation)", async () => {
    vi.resetModules();
    const mod = await import("../../src/core/wasm.js");
    await mod.initCore();
    const generationAfterFirst = mod.coreGeneration();
    await mod.initCore();
    expect(mod.coreGeneration()).toBe(generationAfterFirst);
  });

  it("concurrent calls share one in-flight instantiation (only one generation bump for two overlapping calls)", async () => {
    vi.resetModules();
    const mod = await import("../../src/core/wasm.js");
    const [a, b] = [mod.initCore(), mod.initCore()];
    await Promise.all([a, b]);
    expect(mod.coreReady()).toBe(true);
    expect(mod.coreGeneration()).toBe(1);
  });

  it("a failed init clears state so a later call can retry", async () => {
    vi.resetModules();
    const mod = await import("../../src/core/wasm.js");
    await expect(mod.initCore(new ArrayBuffer(8))).rejects.toThrow();
    expect(mod.coreReady()).toBe(false);
    // A fresh, valid attempt succeeds — the failed attempt did not wedge the
    // module into a permanently-rejected or stuck "initializing" state.
    await mod.initCore();
    expect(mod.coreReady()).toBe(true);
  });
});

describe("coreStatus() — tells uninitialized/ready/poisoned apart, and is re-exported publicly", () => {
  afterEach(() => {
    __resetCoreForTest();
  });

  it("uninitialized -> ready -> poisoned (by a trap) -> ready again (after re-init)", async () => {
    __resetCoreForTest();
    expect(coreStatus()).toBe("uninitialized");
    expect(coreReady()).toBe(false);

    await initCore();
    expect(coreStatus()).toBe("ready");

    expect(() =>
      coreCall(() => {
        throw new (getWasmApi().RuntimeError)("trap for coreStatus test");
      }),
    ).toThrow(CoreTrapError);
    // coreReady() alone cannot tell "never initialised" apart from "was
    // initialised and then poisoned" — coreStatus() is what a host adapter
    // (host/headless.ts) needs to pick the right fallback reason.
    expect(coreReady()).toBe(false);
    expect(coreStatus()).toBe("poisoned");

    await initCore();
    expect(coreStatus()).toBe("ready");
  });

  it("is re-exported from src/core.ts, the public brownout/core entry point, and reflects the same lifecycle", async () => {
    // A fresh module registry, entirely separate from this file's shared
    // top-level import — a prior describe block's own vi.resetModules() calls
    // (the "initCore concurrency" tests below) would otherwise leave a STALE
    // cached "./core/wasm.js" instance for src/core.ts's own import to pick
    // up, desyncing it from whatever this test compares it against.
    vi.resetModules();
    const publicEntry: { coreStatus: () => "uninitialized" | "ready" | "poisoned"; initCore: () => Promise<void> } =
      await import("../../src/core.js");
    expect(typeof publicEntry.coreStatus).toBe("function");
    expect(publicEntry.coreStatus()).toBe("uninitialized");
    await publicEntry.initCore();
    expect(publicEntry.coreStatus()).toBe("ready");
  });
});

describe("initCore — bare string source resolution", () => {
  afterEach(() => {
    __resetCoreForTest();
    vi.unstubAllGlobals();
  });

  it("resolves a relative string against globalThis.location.href when a location global exists (the normal browser self-hosted-asset case)", async () => {
    // `location.href` pointing at a fake "page" inside src/core/ — a relative
    // "./brownout-core.wasm" must resolve against THAT, exactly like fetch()
    // would for wasm-pack's generated init(), which this type's doc comment
    // claims parity with.
    const pageUrl = pathToFileURL(path.join(path.dirname(WASM_PATH), "index.html")).href;
    vi.stubGlobal("location", { href: pageUrl });
    await initCore("./brownout-core.wasm");
    expect(coreReady()).toBe(true);
  });

  it("throws CoreInitError (not the WHATWG URL constructor's bare TypeError) for a relative string with no location global in this runtime", async () => {
    expect((globalThis as { location?: unknown }).location).toBeUndefined();
    let caught: unknown;
    try {
      await initCore("./brownout-core.wasm");
    } catch (err) {
      caught = err;
    }
    expect(caught).toBeInstanceOf(CoreInitError);
    expect((caught as Error).message).toMatch(/relative/);
    // A failed init must still leave the module retryable.
    expect(coreReady()).toBe(false);
    await initCore();
    expect(coreReady()).toBe(true);
  });
});

describe("initCore — every failure to locate/fetch/instantiate the bytes is wrapped in CoreInitError", () => {
  afterEach(() => {
    __resetCoreForTest();
  });

  it("a non-ok HTTP Response (e.g. a CDN 404 HTML page) rejects as CoreInitError naming the status, not a raw WebAssembly.CompileError", async () => {
    const response = new Response("<html>not found</html>", { status: 404, statusText: "Not Found" });
    let caught: unknown;
    try {
      await initCore(response);
    } catch (err) {
      caught = err;
    }
    expect(caught).toBeInstanceOf(CoreInitError);
    expect((caught as Error).message).toMatch(/404/);
  });

  it("garbage bytes reject as CoreInitError wrapping the WebAssembly CompileError", async () => {
    const garbage = new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8]);
    let caught: unknown;
    try {
      await initCore(garbage);
    } catch (err) {
      caught = err;
    }
    expect(caught).toBeInstanceOf(CoreInitError);
    expect((caught as Error).cause).toBeInstanceOf(Error);
  });

  it("a missing file: URL rejects as CoreInitError wrapping Node's ENOENT, not a raw fs Error", async () => {
    const missing = new URL("file:///no/such/brownout-core-missing.wasm");
    let caught: unknown;
    try {
      await initCore(missing);
    } catch (err) {
      caught = err;
    }
    expect(caught).toBeInstanceOf(CoreInitError);
    expect((caught as Error).message).toMatch(/could not instantiate/);
    expect((caught as Error).cause).toBeInstanceOf(Error);
  });
});
