/**
 * Loads the brownout-core wasm module and holds the one live instance this
 * process talks to. Everything here is plumbing for the C ABI v2 contract
 * (core/README.md, "C ABI, version 2") — no physics, no program-format
 * knowledge (that's format.ts/compile.ts) and no circuit knowledge (that's
 * engine.ts). engine.ts is the only intended caller of the `@internal`
 * exports below; a host only ever calls initCore()/coreReady().
 *
 * NO DOM LIB, AND NO `declare global` FOR WebAssembly EITHER
 * This package typechecks with zero browser globals (tsconfig.json: `"lib":
 * ["ES2022"]`, no "dom"), so the real `WebAssembly` namespace type (normally
 * declared by lib.dom.d.ts) is not ambiently available here the way it would
 * be in a browser project. An earlier version of this file patched that with
 * `declare global { namespace WebAssembly { ... } }` — which typechecked
 * fine for THIS package, but ships straight into this package's own
 * dist/core/wasm.d.ts. A consumer whose OWN tsconfig has "dom" in `lib` (a
 * browser app, or anything importing brownout/host — src/host.ts pulls the
 * core in transitively) would merge that augmentation against the REAL
 * lib.dom `WebAssembly`, and since both sides declare `class Module` /
 * `Instance` / `Memory` / `RuntimeError`, and classes (unlike interfaces)
 * cannot be declaration-merged, that is `TS2300: Duplicate identifier` in
 * every such consumer — reproduced against a `skipLibCheck: false` consumer
 * in test/types/dom-consumer (this repo's own tsconfig.json sets
 * `skipLibCheck: true`, which is exactly why this never showed up here).
 *
 * The fix: no global augmentation, anywhere, ever. `WebAssembly` is a REAL
 * runtime global (Node has shipped one since Node 8) — we just don't lean on
 * TypeScript knowing its ambient type. Below, `getWasmApi()` reads it off
 * `globalThis` through a LOCAL, non-global interface (`WasmApi`) that only
 * describes the handful of members this file actually calls. Every public
 * type that would otherwise have to say `WebAssembly.Module` /
 * `WebAssembly.Memory` uses an equivalent local structural type instead
 * (`CompiledWasmModule`, `WasmMemoryLike`) — see their doc comments.
 * `ArrayBufferView`/`Response`/`fetch`/`URL`/`TextDecoder` are unaffected:
 * `ArrayBufferView` is declared in lib.es5.d.ts (interfaces merge fine, and
 * every typed array already satisfies it structurally regardless), and the
 * rest come from @types/node (Node has shipped real ambient globals for
 * these since v18).
 */

import {
  CoreInitError,
  CoreLoadError,
  CoreNotInitializedError,
  CoreNotLoadedError,
  CorePoisonedError,
  CoreStaleInstanceError,
  CoreTrapError,
  CoreUnsupportedError,
} from "./errors.js";

/**
 * Every Core* error class this package defines. `coreCall` uses this to tell
 * "a definite, fully-handled outcome one of engine.ts's closures threw on
 * purpose" (CoreUnsupportedError, CoreLoadError, ...) apart from "something
 * escaped that means the instance's state is no longer trustworthy" — see
 * `coreCall`'s own doc comment.
 */
const CORE_ERROR_CLASSES = [
  CoreInitError,
  CoreLoadError,
  CoreNotInitializedError,
  CoreNotLoadedError,
  CorePoisonedError,
  CoreStaleInstanceError,
  CoreTrapError,
  CoreUnsupportedError,
] as const;

function isCoreError(err: unknown): boolean {
  return CORE_ERROR_CLASSES.some((cls) => err instanceof cls);
}

/** `err.message` for a real Error, else a best-effort string — for building a wrapper error's own message. */
function errMessage(err: unknown): string {
  return err instanceof Error ? err.message : String(err);
}

/** The only ABI version this build's TS and Rust halves understand (core/README.md, "C ABI, version 2"). */
export const CORE_ABI_VERSION = 2;

/**
 * A pre-compiled wasm module — what `new WebAssembly.Module(bytes)` or
 * `WebAssembly.compile(bytes)` produce. Deliberately typed as a documented
 * alias for `object` rather than naming `WebAssembly.Module` directly: doing
 * so would require every consumer of this type (including this package's own
 * public API) to have an ambient `WebAssembly` type in scope, which a plain
 * Node/NodeNext project without "dom" does not have. `object` is an honest
 * description of what this type actually promises here — even lib.dom's own
 * `WebAssembly.Module` interface has no members of its own; its identity is
 * purely nominal, established by having come from the real constructor.
 * `initCore()` recognises one at runtime via `instanceof` against the REAL
 * global `WebAssembly.Module` (see `getWasmApi`), never by shape.
 */
export type CompiledWasmModule = object;

/**
 * {@link CoreWasmSource} minus the thenable member — the shape a promise
 * given to `initCore()` must resolve to.
 */
type CoreWasmDirectSource = ArrayBuffer | ArrayBufferView | CompiledWasmModule | URL | string | Response;

/**
 * Anything initCore() accepts as the wasm module's source. `Response` exists
 * so a caller can hand over an in-flight `fetch(...)` directly (the
 * streaming-compile idiom every wasm loader supports), and
 * `CompiledWasmModule` for a caller that already compiled (or cached) one. A
 * bare `string` is treated as a URL, matching every other wasm-loader
 * convention (wasm-pack's generated `init()`, for one) — it is not a
 * filesystem path.
 *
 * `PromiseLike` wraps every one of those members, not just `Response`: a
 * caller writing `fetch(url).then(r => r.arrayBuffer())` (or handing over a
 * cached `Promise<ArrayBuffer>`, or a `Promise<CompiledWasmModule>`) gets the
 * same treatment as `fetch(url)` itself — the resolved value is dispatched
 * through this same union again (see `instantiateFromSource`'s `isThenable`
 * branch), not assumed to be a `Response`.
 */
export type CoreWasmSource = CoreWasmDirectSource | PromiseLike<CoreWasmDirectSource>;

/**
 * @internal Structural stand-in for `WebAssembly.Memory` (just the one
 * member this package reads) — see this file's header for why it isn't
 * named `WebAssembly.Memory` directly. `verifyCoreExports` still checks the
 * REAL constructor identity at runtime (`instanceof` against
 * `getWasmApi().Memory`); this type only describes what engine.ts/wasm.ts
 * read off it afterwards.
 */
export interface WasmMemoryLike {
  readonly buffer: ArrayBuffer;
}

/**
 * @internal Typed mirror of the C ABI v2 export table (core/README.md, "C
 * ABI, version 2"). Pointers are plain `number`s: a wasm32 `*const f64` (or
 * `*mut f64`) crosses the ABI as an i32 byte offset into `memory.buffer`,
 * never a real pointer object.
 *
 * v2 (W1a) adds the rows below `bo_error_len`; `bo_engine_set_params` (W1b,
 * owner decision 1) is now below too. `bo_engine_seed_memory`,
 * `bo_engine_take_eeprom_updates` and `bo_engine_eeprom_updates_ptr` (W1d,
 * owner decision 2) follow, then `bo_engine_device_state` and
 * `bo_engine_device_state_ptr` (W1f Q11), then `bo_engine_reset_failures`,
 * `bo_engine_records` and `bo_engine_records_ptr` (W1g).
 */
export interface CoreExports {
  readonly memory: WasmMemoryLike;
  bo_abi_version(): number;
  bo_alloc(lenF64: number): number;
  bo_free(ptr: number, lenF64: number): void;
  bo_engine_create(): number;
  bo_engine_destroy(h: number): void;
  bo_engine_load(h: number, ptr: number, lenF64: number): number;
  bo_engine_advance(h: number, tEnd: number): number;
  bo_engine_time(h: number): number;
  bo_engine_node_count(h: number): number;
  bo_engine_node_voltages(h: number): number;
  bo_engine_device_count(h: number): number;
  bo_engine_device_currents(h: number): number;
  bo_engine_stat(h: number, i: number): number;
  bo_error_ptr(): number;
  bo_error_len(): number;
  bo_engine_set_option(h: number, key: number, value: number): number;
  bo_engine_next_event_time(h: number): number;
  /**
   * Time of the earliest pending DRIVER COMMIT only (`+Infinity` when none) —
   * finding 10's narrower counterpart to `bo_engine_next_event_time`, which
   * also reports a stepped partition's own internal Step bookkeeping, a
   * closed-form Crossing watch, and a floating-input Bucket boundary. Mirrors
   * classic's `nextScheduledEventTime()` (sim-engine.ts), which only ever
   * considers a digital due time. core/README.md's C ABI v2 table, "W1c".
   */
  bo_engine_next_digital_due_time(h: number): number;
  bo_engine_digital_slot_count(h: number): number;
  bo_engine_digital_slots(h: number): number;
  bo_engine_channel_slot_count(h: number): number;
  bo_engine_channel_slots(h: number): number;
  bo_engine_trace_enable(h: number, on: number): number;
  bo_engine_trace_ptr(h: number): number;
  bo_engine_trace_len(h: number): number;
  bo_engine_trace_clear(h: number): void;
  bo_engine_set_params(h: number, ptr: number, lenF64: number): number;
  /**
   * Replace a 74ls189's 16 stored nibbles (core/README.md's C ABI v2 table,
   * W1d, owner decision 2). `device` is a device record index; the buffer
   * holds 16 values, each the nibble stored at that address in TRUE sense
   * (classic's `mem{i}`), an integer in `[0, 15]`.
   */
  bo_engine_seed_memory(h: number, device: number, ptr: number, lenF64: number): number;
  /**
   * Collect and clear every dirty EEPROM image, returning the update
   * buffer's length in f64 (`>= 0`), or `-5` with no program loaded
   * (core/README.md's C ABI v2 table, W1d). Read the buffer itself with
   * {@link CoreExports.bo_engine_eeprom_updates_ptr}.
   */
  bo_engine_take_eeprom_updates(h: number): number;
  /**
   * The buffer from the last {@link CoreExports.bo_engine_take_eeprom_updates}
   * call, valid until the next call on this engine: per EEPROM,
   * `deviceIndex, W, word x W` (the whole image packed as the ROM blob is).
   */
  bo_engine_eeprom_updates_ptr(h: number): number;
  /**
   * Collect one device's runtime state into the engine's state buffer,
   * projected to the current time without committing anything, and return
   * its length in f64: the kind's layout length (`>= 0`; a `bench_psu` with
   * `iLimit = 0` is `0`, a success). `-8` for a device index out of range or
   * a kind with no device state, `-5` with no program loaded (core/README.md's
   * C ABI v2 table and `bo_engine_device_state` layouts, W1f Q11). Read the
   * values with {@link CoreExports.bo_engine_device_state_ptr}.
   */
  bo_engine_device_state(h: number, device: number): number;
  /**
   * The buffer from the last {@link CoreExports.bo_engine_device_state}
   * collect, valid until the next call on this engine.
   */
  bo_engine_device_state_ptr(h: number): number;
  /**
   * Classic's `resetFailures()`, applied as one event at the current time:
   * `0`; `-4` or `-7` when the re-solve at that instant fails, and then the
   * reset rolls back; `-5` with no program loaded (core/README.md's C ABI v2
   * table, W1g).
   */
  bo_engine_reset_failures(h: number): number;
  /**
   * Collect one record channel (`0` failures, `1` warnings, `2` tripped PTCs,
   * `3` the latch log, which the collect empties) into the engine's record
   * buffer and return its length in f64, a multiple of 7; `-8` for another
   * channel, `-5` with no program loaded. Read it with {@link copyRecords}.
   */
  bo_engine_records(h: number, channel: number): number;
  /** The buffer from the last {@link CoreExports.bo_engine_records} collect, valid until the next call on this engine. */
  bo_engine_records_ptr(h: number): number;
  /**
   * W2a (core/README.md, "Snapshots"): copy the whole engine into the
   * handle's snapshot table and return the snapshot's id, or `0` for an
   * unusable handle. Optional, as a module built before W2a has none of the
   * four; `CoreEngine.saveState` refuses on such a module.
   */
  bo_engine_snapshot?(h: number): number;
  /** Put snapshot `id` back: `0`; `-8` for an id this handle does not hold (dropped, another handle's, or taken before the last load); `-5` for an unusable handle. */
  bo_engine_restore?(h: number, id: number): number;
  /** Free snapshot `id`: `0`; `-8` for an id this handle does not hold; `-5` for an unusable handle. */
  bo_engine_snapshot_drop?(h: number, id: number): number;
  /** The handle's snapshot table in bytes, each shared memory image counted once; `NaN` for an unusable handle. */
  bo_engine_snapshot_bytes?(h: number): number;
  /**
   * W2c (core/README.md, "Carry"): collect every device's carry records and
   * return the buffer's length in f64; `-5` for an unusable handle, `-4` or
   * `-7` when bringing the partitions to the engine's time fails. Optional,
   * detected by its presence (ruling R100 (a)).
   */
  bo_engine_carry_take?(h: number): number;
  /** The buffer from the last {@link CoreExports.bo_engine_carry_take}, valid until the next call on this engine. */
  bo_engine_carry_ptr?(h: number): number;
  /**
   * W2b (core/README.md, "Acquisition"): sample the `n` nodes at `nodesPtr`
   * (f64 indices) at `origin + k * interval` from the engine's time on, at
   * most `limit` held between takes (`0`: 2^20); `interval` 0 disarms. `0`;
   * `-5` for an unusable handle; `-8` for a bad node or a bad origin or
   * interval. Optional, detected by its presence (ruling R100 (a)).
   */
  bo_engine_acquire?(h: number, nodesPtr: number, n: number, origin: number, interval: number, limit: number): number;
  /** Take the final samples, `[count, maxSpan, overflowed, dropped, fallbacks x nodes, t x count, node-major values]`; the length in f64, or `-5`. */
  bo_engine_acquire_take?(h: number): number;
  /** The buffer from the last {@link CoreExports.bo_engine_acquire_take}, valid until the next call on this engine. */
  bo_engine_acquire_ptr?(h: number): number;
}

/** The C ABI v2 export names (core/README.md's table) every instance must have, beyond `memory`. */
const REQUIRED_FUNCTION_EXPORTS = [
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
  "bo_engine_next_digital_due_time",
  "bo_engine_digital_slot_count",
  "bo_engine_digital_slots",
  "bo_engine_channel_slot_count",
  "bo_engine_channel_slots",
  "bo_engine_trace_enable",
  "bo_engine_trace_ptr",
  "bo_engine_trace_len",
  "bo_engine_trace_clear",
  "bo_engine_set_params",
  "bo_engine_seed_memory",
  "bo_engine_take_eeprom_updates",
  "bo_engine_eeprom_updates_ptr",
  "bo_engine_device_state",
  "bo_engine_device_state_ptr",
  "bo_engine_reset_failures",
  "bo_engine_records",
  "bo_engine_records_ptr",
] as const;

// ─── The runtime WebAssembly API, resolved off globalThis — no ambient type ─
//
// See this file's header. `WasmApi` describes only what this file calls; it
// is a LOCAL interface, never a `declare global` augmentation, so it cannot
// collide with a consumer's own "dom" lib. Every member here is backed by a
// REAL runtime global (WebAssembly has been a global in Node since v8), so
// `instanceof`/construction against it behaves exactly like the real thing —
// because it IS the real thing, just read through a narrower type.

interface WasmModuleCtor {
  new (bytes: ArrayBufferView | ArrayBuffer): CompiledWasmModule;
  imports(moduleObject: CompiledWasmModule): { module: string; name: string; kind: string }[];
  exports(moduleObject: CompiledWasmModule): { name: string; kind: string }[];
}

interface WasmMemoryCtor {
  new (descriptor: { initial: number; maximum?: number }): WasmMemoryLike;
}

interface WasmInstanceLike {
  readonly exports: Record<string, unknown>;
}

interface WasmInstantiatedSourceLike {
  module: CompiledWasmModule;
  instance: WasmInstanceLike;
}

interface WasmApi {
  Module: WasmModuleCtor;
  Memory: WasmMemoryCtor;
  RuntimeError: new (message?: string) => Error;
  compile(bytes: ArrayBufferView | ArrayBuffer): Promise<CompiledWasmModule>;
  instantiate(
    bufferSource: ArrayBufferView | ArrayBuffer,
    importObject?: Record<string, unknown>,
  ): Promise<WasmInstantiatedSourceLike>;
  instantiate(
    moduleObject: CompiledWasmModule,
    importObject?: Record<string, unknown>,
  ): Promise<WasmInstanceLike>;
  // Not every runtime implements streaming compilation (Node's support has
  // been inconsistent across versions), so this is the one optional member.
  instantiateStreaming?(
    source: Response | PromiseLike<Response>,
    importObject?: Record<string, unknown>,
  ): Promise<WasmInstantiatedSourceLike>;
}

/**
 * @internal Resolves the real global `WebAssembly` API, typed only as far as
 * `WasmApi` above needs. Exported so tests can drive `instanceof`/construct
 * checks against the SAME real constructors this file uses (e.g. a synthetic
 * `new (getWasmApi().RuntimeError)("...")` trap, or `new
 * (getWasmApi().Memory)(...)` for a fake exports object) without any of them
 * needing an ambient `WebAssembly` type either.
 */
export function getWasmApi(): WasmApi {
  const api = (globalThis as unknown as { WebAssembly?: WasmApi }).WebAssembly;
  if (!api) {
    throw new CoreInitError("brownout/core: no global WebAssembly API is available in this runtime.");
  }
  return api;
}

/**
 * @internal Validates a raw `instance.exports` object against the C ABI v2
 * contract and returns it typed as `CoreExports`, or throws a `CoreInitError`
 * naming exactly what's wrong. Exported (not just used internally by
 * `initCore`) so wasm.test.ts can drive the "missing export" and "ABI
 * version mismatch" checks with a hand-built fake exports object, without
 * needing a real (mis-)built wasm binary to provoke them.
 */
export function verifyCoreExports(raw: Record<string, unknown>): CoreExports {
  const missing: string[] = REQUIRED_FUNCTION_EXPORTS.filter((name) => typeof raw[name] !== "function");
  if (!(raw.memory instanceof getWasmApi().Memory)) missing.push("memory");
  if (missing.length > 0) {
    throw new CoreInitError(
      `brownout/core: the wasm module is missing (or has the wrong type for) the following ` +
        `C ABI v2 export(s): ${missing.join(", ")}. This build does not match the contract in ` +
        "core/README.md — rebuild both sides together (scripts/build-core.mjs).",
    );
  }

  const exports = raw as unknown as CoreExports;
  const reportedVersion = exports.bo_abi_version();
  if (reportedVersion !== CORE_ABI_VERSION) {
    throw new CoreInitError(
      `brownout/core: the wasm module reports ABI version ${reportedVersion}, but this ` +
        `package's TS front end was built for ABI version ${CORE_ABI_VERSION}. The TS and ` +
        "Rust halves ship in one npm package and must be rebuilt together — there is no " +
        "cross-version compatibility (core/README.md, \"Program format, version 2\").",
    );
  }
  return exports;
}

// ─── Module state ───────────────────────────────────────────────────────────
//
// One process-wide instance. "ready" and "poisoned" both carry a real
// `exports` plus the generation it was instantiated at; CoreEngine records
// that generation at construction time and refuses to operate once it no
// longer matches (see engine.ts's `_checkGeneration`), which is what makes
// re-init after a poison safe even though old CoreEngine objects are still
// reachable.

type ModuleState =
  | { status: "uninitialized" }
  | { status: "ready"; exports: CoreExports; generation: number }
  | { status: "poisoned"; exports: CoreExports; generation: number };

let moduleState: ModuleState = { status: "uninitialized" };
let generationCounter = 0;
/** The one in-flight initCore() promise, shared by every concurrent caller until it settles. */
let initPromise: Promise<void> | null = null;

/** True once a live, unpoisoned instance exists — the only state in which coreCall() will actually run something. */
export function coreReady(): boolean {
  return moduleState.status === "ready";
}

/**
 * The module's current lifecycle state: `"uninitialized"` (initCore() has
 * never succeeded, or its most recent attempt failed), `"ready"` (coreReady()
 * would return true), or `"poisoned"` (a trap; every call throws until
 * initCore() succeeds again). A caller that only needs "can I use it right
 * now" wants coreReady(); this is for a caller that needs to tell "nobody
 * has initialised yet" apart from "it was initialised and then poisoned" —
 * e.g. a host adapter picking a fallback reason (host/headless.ts).
 */
export function coreStatus(): "uninitialized" | "ready" | "poisoned" {
  return moduleState.status;
}

/** @internal The live instance's generation counter. Throws if initCore() has never been (successfully) awaited. */
export function coreGeneration(): number {
  if (moduleState.status === "uninitialized") {
    throw new CoreNotInitializedError(
      "coreGeneration(): initCore() must be awaited before the core has an instance generation.",
    );
  }
  return moduleState.generation;
}

/**
 * @internal Runs one ABI call against the live instance. Poisoned/
 * uninitialized states are refused before `fn` ever runs.
 *
 * POISON BACKSTOP: any value `fn` throws that is not one of this package's
 * own Core* error classes (see `isCoreError`) poisons the instance and is
 * re-thrown wrapped in a `CoreTrapError`, with the original as `cause`. A
 * `WebAssembly.RuntimeError` (a real Rust panic, panic=abort — core/README.md
 * "Traps") is the expected case, but it is deliberately not the ONLY one
 * treated this way: a wasm stack overflow surfaces as a JS `RangeError`, not
 * a `RuntimeError`, and the Rust instance's state after an aborted call is
 * exactly as untrustworthy either way — there being no documented name for
 * every way the wasm runtime can unwind out of a call is not a reason to
 * treat the instance as still trustworthy. A Core* error thrown by one of
 * engine.ts's own closures (e.g. `mapLoadFailure`'s `CoreUnsupportedError`/
 * `CoreLoadError`) passes through unchanged: it describes a definite,
 * fully-handled outcome the Rust call returned normally from, not instance
 * corruption.
 */
export function coreCall<T>(fn: (exports: CoreExports) => T): T {
  if (moduleState.status === "uninitialized") {
    throw new CoreNotInitializedError("initCore() must be awaited before calling into the brownout core.");
  }
  if (moduleState.status === "poisoned") {
    throw new CorePoisonedError(
      "the brownout core instance is poisoned by an earlier trap; call initCore() again for a fresh instance.",
    );
  }
  const { exports, generation } = moduleState;
  try {
    return fn(exports);
  } catch (err) {
    if (isCoreError(err)) throw err;
    moduleState = { status: "poisoned", exports, generation };
    throw new CoreTrapError(
      `brownout core call threw an unexpected error (${errMessage(err)}); the instance is now poisoned — ` +
        "call initCore() again to recover.",
      { cause: err },
    );
  }
}

const textDecoder = new TextDecoder();

/**
 * @internal Reads the current `bo_error_ptr()`/`bo_error_len()` message.
 * Call this immediately after the failing ABI call, before any other call on
 * the same instance: the pointer is only valid "until the next call on that
 * engine" (core/README.md, "Memory").
 */
export function readCoreErrorMessage(exports: CoreExports): string {
  // `>>> 0`: a wasm32 pointer crosses the ABI as an i32, so JS sees a
  // negative number once the address reaches 2^31 — coerce to the unsigned
  // byte offset `Uint8Array` actually wants (core/README.md's exports are
  // typed `*const u8` / `*mut f64`, unsigned addresses on wasm32).
  const ptr = exports.bo_error_ptr() >>> 0;
  const len = exports.bo_error_len();
  return textDecoder.decode(new Uint8Array(exports.memory.buffer, ptr, len));
}

/**
 * @internal Copies `len` f64s starting at byte offset `ptr` out of the
 * core's memory into a fresh, independent `Float64Array` — never a live
 * view. The contract: any call may grow `memory` and detach every prior
 * view of `memory.buffer`, so a view must be re-derived from the CURRENT
 * `memory.buffer` and copied out before anything else touches the instance.
 */
export function copyF64(exports: CoreExports, ptr: number, len: number): Float64Array {
  return Float64Array.from(new Float64Array(exports.memory.buffer, ptr, len));
}

/**
 * @internal Collects record channel `channel` of engine `h` and copies the
 * record buffer out, or returns `bo_engine_records`'s negative code untouched
 * so the caller can read the error message next. The view is taken from
 * `memory.buffer` as it stands AFTER the collect, which may have grown memory
 * and detached every earlier view.
 */
export function copyRecords(exports: CoreExports, h: number, channel: number): Float64Array | number {
  const len = exports.bo_engine_records(h, channel);
  if (len < 0) return len;
  if (len === 0) return new Float64Array(0);
  return copyF64(exports, exports.bo_engine_records_ptr(h) >>> 0, len);
}

// ─── Source resolution ──────────────────────────────────────────────────────

function isThenable(x: unknown): x is PromiseLike<unknown> {
  return typeof x === "object" && x !== null && typeof (x as { then?: unknown }).then === "function";
}

function looksLikeResponse(x: unknown): x is Response {
  return (
    typeof x === "object" &&
    x !== null &&
    typeof (x as Response).arrayBuffer === "function" &&
    "headers" in (x as object)
  );
}

function isArrayBufferView(x: unknown): x is ArrayBufferView {
  return (
    typeof x === "object" &&
    x !== null &&
    typeof (x as ArrayBufferView).byteLength === "number" &&
    typeof (x as ArrayBufferView).byteOffset === "number" &&
    (x as ArrayBufferView).buffer !== undefined
  );
}

/** A typed array's (or Buffer's) live window turned into a fresh ArrayBuffer — slicing, not just reading `.buffer`, because a Node `Buffer` is a view into a SHARED, pooled backing buffer. */
/**
 * The module's one import (ruling R86): the host's clock in milliseconds,
 * which `bo_engine_advance_for` reads at its start and end and once every
 * 64 instants to keep within its wall budget. `performance.now()` where the
 * host has it; a host without it reads NaN, which turns the budget off (the
 * event cap still holds). An older module that does not import it ignores
 * the extra entry.
 */
export function coreImports(): Record<string, unknown> {
  const perf = (globalThis as { performance?: { now?: () => number } }).performance;
  const now = typeof perf?.now === "function" ? () => perf.now!() : () => Number.NaN;
  return { env: { bo_host_now_ms: now } };
}

function toArrayBuffer(view: ArrayBufferView): ArrayBuffer {
  const { buffer, byteOffset, byteLength } = view;
  if (byteOffset === 0 && byteLength === (buffer as ArrayBuffer).byteLength) return buffer as ArrayBuffer;
  return (buffer as ArrayBuffer).slice(byteOffset, byteOffset + byteLength);
}

async function instantiateFromResponse(response: Response): Promise<WasmInstanceLike> {
  if (!response.ok) {
    throw new CoreInitError(
      `brownout/core: fetching the wasm module failed with HTTP ${response.status} ` +
        `${response.statusText} (${response.url || "<no url>"}).`,
    );
  }
  const WA = getWasmApi();
  const contentType = response.headers?.get?.("content-type") ?? "";
  if (contentType.includes("application/wasm") && typeof WA.instantiateStreaming === "function") {
    const { instance } = await WA.instantiateStreaming(response, coreImports());
    return instance;
  }
  const bytes = await response.arrayBuffer();
  const { instance } = await WA.instantiate(bytes, coreImports());
  return instance;
}

/**
 * Resolves a bare string source into a URL. An absolute string (`https://…`,
 * `file://…`) works with no base, exactly as `new URL(source)` always did.
 * A relative string (`/assets/brownout-core.wasm`, `./brownout-core.wasm`) —
 * the normal way a browser host points at a self-hosted asset, and the form
 * wasm-pack's generated `init()` accepts via `fetch()` — is resolved against
 * `globalThis.location.href` when a `location` global exists (a browser or a
 * worker). In a runtime with no `location` (plain Node), there is no base to
 * resolve it against, so this throws a `CoreInitError` naming the problem and
 * the alternatives, rather than the WHATWG `URL` constructor's bare
 * `TypeError: Invalid URL`.
 */
function resolveWasmUrlString(source: string): URL {
  try {
    return new URL(source);
  } catch (err) {
    const base = (globalThis as unknown as { location?: { href?: string } }).location?.href;
    if (base === undefined) {
      throw new CoreInitError(
        `initCore: "${source}" is a relative URL, and there is no global "location" in this ` +
          "runtime to resolve it against. Pass an absolute URL (string or URL object), the wasm " +
          "bytes directly (ArrayBuffer/typed array), or a Response instead.",
        { cause: err },
      );
    }
    try {
      return new URL(source, base);
    } catch (err2) {
      throw new CoreInitError(
        `initCore: could not resolve "${source}" against globalThis.location.href (${base}): ` +
          `${errMessage(err2)}.`,
        { cause: err2 },
      );
    }
  }
}

async function instantiateFromUrl(url: URL): Promise<WasmInstanceLike> {
  if (url.protocol === "file:") {
    // Node-only path. A bundler never sees this branch execute for the
    // DEFAULT source: webpack/vite statically rewrite `new URL("./brownout-
    // core.wasm", import.meta.url)` into an asset URL before this code ever
    // runs in a browser build. The dynamic import (with both ignore
    // comments) keeps webpack/vite from trying to resolve "node:fs/promises"
    // into a browser bundle that will never execute this branch either.
    const { readFile } = await import(/* webpackIgnore: true */ /* @vite-ignore */ "node:fs/promises");
    const bytes = await readFile(url);
    const { instance } = await getWasmApi().instantiate(toArrayBuffer(bytes), coreImports());
    return instance;
  }
  return instantiateFromResponse(await fetch(url));
}

async function instantiateFromSource(source: CoreWasmSource): Promise<WasmInstanceLike> {
  const WA = getWasmApi();
  if (source instanceof WA.Module) {
    // Cast, not a narrowed reference: `CompiledWasmModule` is `object` (see
    // its doc comment), so `instanceof` narrowing here can't usefully narrow
    // `source`'s declared union on its own — the runtime check just passed,
    // which is what actually matters.
    return WA.instantiate(source as unknown as CompiledWasmModule, coreImports());
  }
  if (source instanceof URL) {
    return instantiateFromUrl(source);
  }
  if (typeof source === "string") {
    return instantiateFromUrl(resolveWasmUrlString(source));
  }
  if (source instanceof ArrayBuffer) {
    return (await WA.instantiate(source, coreImports())).instance;
  }
  // Explicit casts from here down, rather than leaning on control-flow
  // narrowing: `CoreWasmSource` includes `object` (`CompiledWasmModule`,
  // above), and narrowing a union against a type as broad as `object` does
  // not reliably narrow the REMAINING members the way it would for a union
  // of more specific types — the runtime predicate already proved the shape,
  // which is what actually matters here.
  if (isThenable(source)) {
    // Re-dispatch the resolved value through this SAME function, rather than
    // assuming it is a Response: CoreWasmSource's PromiseLike member wraps
    // every direct-source member (see its doc comment), so a caller handing
    // over a `Promise<ArrayBuffer>` or `Promise<CompiledWasmModule>` gets the
    // exact same treatment as passing that value directly, not a
    // `response.arrayBuffer is not a function` TypeError from assuming the
    // wrong shape.
    const resolved = await source;
    return instantiateFromSource(resolved as CoreWasmSource);
  }
  if (looksLikeResponse(source)) {
    return instantiateFromResponse(source as Response);
  }
  if (isArrayBufferView(source)) {
    return (await WA.instantiate(source as ArrayBufferView, coreImports())).instance;
  }
  throw new CoreInitError(`initCore: unrecognised wasm source (${Object.prototype.toString.call(source)}).`);
}

/**
 * The default wasm asset, resolved with the EXACT expression shape webpack
 * and vite statically detect (`new URL("<literal>", import.meta.url)`) so
 * they emit the asset and rewrite this call at bundle time — do not factor
 * the literal out into a variable or build the string dynamically, or both
 * bundlers silently stop finding it.
 *
 * Wrapped in try/catch: a Next.js SERVER bundle rewrites this expression to
 * `new URL(__webpack_require__.p + "static/media/...wasm")`, which has no
 * base to resolve against in Node and throws "Failed to parse URL" (measured
 * in a Next.js spike — client/worker bundles work as-is; only the server
 * bundle does this).
 */
function defaultWasmSource(): URL {
  try {
    return new URL("./brownout-core.wasm", import.meta.url);
  } catch (err) {
    throw new CoreInitError(
      "brownout/core: could not resolve the default wasm asset URL " +
        `(${(err as Error).message}). This happens when a bundler rewrites the ` +
        '`new URL("./brownout-core.wasm", import.meta.url)` expression to a path with no ' +
        "base to resolve against in the current runtime — a known case is a Next.js SERVER " +
        'bundle. Fix: mark "brownout" as a server external (Next.js `serverExternalPackages: ' +
        '["brownout"]`), or call initCore() with the wasm bytes yourself (see the ' +
        '"brownout/core.wasm" export for hosts that want to fetch/bundle the bytes directly).',
      { cause: err },
    );
  }
}

async function doInit(source: CoreWasmSource | undefined): Promise<void> {
  let instance: WasmInstanceLike;
  try {
    const resolved = source ?? defaultWasmSource();
    instance = await instantiateFromSource(resolved);
  } catch (err) {
    // Every failure to locate/fetch/instantiate the bytes is a CoreInitError
    // (errors.ts's own doc comment for it) — a fetch() network rejection, a
    // non-ok HTTP response (instantiateFromResponse already raises THAT one
    // as a CoreInitError, so it passes through unchanged below), a Node
    // readFile ENOENT, or WebAssembly's own CompileError/LinkError from
    // malformed or incompatible bytes. Without this, a caller that follows
    // errors.ts's documented contract and catches CoreInitError to decide on
    // fallback or error reporting would miss every one of these.
    if (err instanceof CoreInitError) throw err;
    throw new CoreInitError(`brownout/core: could not instantiate the wasm module: ${errMessage(err)}`, {
      cause: err,
    });
  }
  const verified = verifyCoreExports(instance.exports);
  generationCounter += 1;
  moduleState = { status: "ready", exports: verified, generation: generationCounter };
}

/**
 * Instantiates the brownout-core wasm module, or returns immediately if a
 * live, unpoisoned instance already exists. Concurrent calls share one
 * in-flight instantiation. Must be awaited once before constructing a
 * `CoreEngine`; call it again (after `await`ing the previous call so it has
 * actually settled) to recover from a poisoned instance — that produces a
 * fresh instance with its own generation, which existing `CoreEngine`
 * objects will refuse to use (see engine.ts).
 *
 * `source` defaults to this package's own bundled wasm asset (see
 * `defaultWasmSource`'s doc comment for the bundler notes). Pass a
 * `CoreWasmSource` explicitly to supply the bytes yourself — a pre-compiled
 * module, an `ArrayBuffer`/typed array, an absolute `URL`/URL string (a
 * relative one resolves against `globalThis.location.href` where that
 * exists), a `Response`, or a promise of any of those (`fetch()` itself, or
 * something built on top of it like `fetch(url).then(r => r.arrayBuffer())`).
 */
export async function initCore(source?: CoreWasmSource): Promise<void> {
  if (moduleState.status === "ready") return;
  if (initPromise) return initPromise;

  const promise = doInit(source)
    .catch((err: unknown) => {
      // A failed init must not leave stale "ready"/"poisoned" state behind
      // (there is no usable instance), and must not be permanently retryable
      // with the SAME rejected promise — a later initCore() call (with the
      // same or different bytes) gets a genuinely fresh attempt.
      moduleState = { status: "uninitialized" };
      throw err;
    })
    .finally(() => {
      initPromise = null;
    });
  initPromise = promise;
  return promise;
}

// ─── Test seams ─────────────────────────────────────────────────────────────
// Not for host use. wasm.test.ts uses these to drive the trap/poison state
// machine with a SYNTHETIC `WebAssembly.RuntimeError` instead of provoking a
// genuine Rust panic (which the public ABI has no supported way to trigger —
// every documented failure mode returns a negative code, never traps).

/**
 * @internal Test-only: installs `exports` as if `initCore()` had just
 * succeeded against it, without a real wasm instantiation. Bumps the
 * generation counter exactly like a real re-init would.
 */
export function __installCoreForTest(exports: CoreExports): void {
  generationCounter += 1;
  moduleState = { status: "ready", exports, generation: generationCounter };
  initPromise = null;
}

/** @internal Test-only: resets module state to as-if-never-initialized. */
export function __resetCoreForTest(): void {
  moduleState = { status: "uninitialized" };
  initPromise = null;
}
