/**
 * `CoreEngine` — the facade over one Rust engine handle. Implements
 * `EngineReadout` (surface.ts) so a host can read it exactly like a classic
 * `SimEngine`; see that file's header for why loading/advancing is NOT part
 * of the shared interface.
 *
 * One handle per instance, for its whole lifetime. `bo_engine_create()` is
 * called EAGERLY in the constructor rather than lazily on first `load()`:
 * handles are cheap (a Rust `Vec` push), and eager creation means there is no
 * "constructed but not yet holding a handle" state to special-case in every
 * method below — a `CoreEngine` either has a handle or it has thrown out of
 * its constructor, never something in between.
 *
 * GENERATIONS. `initCore()` can be called again after the instance it
 * produced is poisoned by a trap (wasm.ts, "Traps"); that produces a NEW
 * instance with its own handle table, where THIS engine's handle number
 * could coincidentally name a real, unrelated engine. Every method here
 * therefore checks its recorded generation against the live one
 * (`coreGeneration()`) and refuses to run at all once they disagree —
 * `CoreStaleInstanceError`, not silent misbehaviour against the wrong
 * instance.
 *
 * "NO PROGRAM LOADED" IS A FACADE-LEVEL STATE, NOT JUST A RUST ONE. A failed
 * `bo_engine_load` leaves the RUST side exactly as it was — the previous
 * program, still loaded, still answering queries (core/README.md's
 * contract; abi.rs's own `a_failed_reload_leaves_the_previous_program_intact`
 * test asserts this). This facade deliberately does NOT expose that: any
 * `load()` call that throws — whether compileCircuit rejected the circuit or
 * `bo_engine_load` returned an error code — clears this engine's own
 * "loaded" state first, so every readout throws `CoreNotLoadedError` until
 * the NEXT successful `load()`. Silently continuing to serve the OLD
 * circuit's readouts after a `load()` call the caller just saw fail would be
 * a far worse surprise than an explicit throw.
 */

import { compileCircuit, type CompiledCircuit } from "./compile.js";
import { CORE_IMPLEMENTED_KINDS } from "./kinds.js";
import {
  coreCall,
  coreGeneration,
  coreReady,
  coreStatus,
  copyF64,
  readCoreErrorMessage,
  type CoreExports,
} from "./wasm.js";
import {
  CoreLoadError,
  CoreNotInitializedError,
  CoreNotLoadedError,
  CoreStaleInstanceError,
  CoreUnsupportedError,
} from "./errors.js";
import type { EngineReadout } from "./surface.js";
import type { SimCircuit, SimFailure, SimWarning } from "../sim/engine/sim-engine.js";
import type { Net } from "../sim/engine/graph.js";
import type { DisplayInfo } from "../sim/messages.js";

/** `bo_engine_stat`'s four counters (core/README.md's index table), named. */
export interface CoreEngineStats {
  partitions: number;
  staticSolves: number;
  largestPartition: number;
  fixedNodes: number;
}

/**
 * Maps a non-zero `bo_engine_load` return code to the TS error the contract
 * assigns it (core/README.md's error-code table). Only -3 (unsupported
 * topology) is "route this circuit elsewhere" — -2 (unknown kind code) is
 * NOT a fallback case despite superficially reading like one: compileCircuit
 * only ever emits kind codes from format.ts's CORE_KIND_CODE, which is
 * exactly the set CORE_IMPLEMENTED_KINDS lets through, so the Rust reader
 * rejecting one it doesn't recognise means the TS and Rust halves are out of
 * step with each other, not a fact about the user's circuit. Folding it into
 * `unsupported-kind:<message>` would also corrupt that reason vocabulary:
 * the message here is prose like "device[3]: unknown kind code 7", not a
 * kind name.
 */
function mapLoadFailure(code: number, message: string): CoreUnsupportedError | CoreLoadError {
  if (code === -3) {
    return new CoreUnsupportedError([`topology:${message}`]);
  }
  // -1 (malformed), -2 (unknown kind code — a TS/Rust drift bug, see above),
  // -4 (solve failed), -5 (bad handle), and anything this build doesn't
  // otherwise define: none of these mean "route this circuit elsewhere".
  return new CoreLoadError(`bo_engine_load failed (code ${code}): ${message}`);
}

export class CoreEngine implements EngineReadout {
  private readonly _handle: number;
  private readonly _generation: number;
  private _disposed = false;

  private _compiled: CompiledCircuit | null = null;
  private _netV: Record<string, number> = {};
  private _elementI: Record<string, number> = {};
  private _lastConverged = false;

  // Facade-side clock. The Rust engine restarts every loaded program at t=0
  // (core/README.md, "W0 kernel semantics" #6), but classic SimEngine.load()
  // never resets `this.simTime` — only coldLoad() does (confirmed by reading
  // sim-engine.ts: `load()` has no `this.simTime = 0` anywhere in it). A
  // live-edit reload must keep the facade's simTime moving forward exactly
  // like classic does.
  //
  // `_simTime` is the SOURCE OF TRUTH for the public `simTime` getter — it
  // is set to exactly `tEnd` on a successful advance() and carried forward
  // (unchanged) across load(), never derived by adding `bo_engine_time()`'s
  // readback back on top of a base: `_timeBase + bo_engine_time(...)` is not
  // guaranteed to reproduce the exact `tEnd` that was requested in floating
  // point, and a caller comparing `engine.simTime` against the `tEnd` it
  // just passed to advance() deserves EXACT equality, not "close enough".
  //
  // `_loadedAt` is a separate, narrower thing: only the facade-time offset
  // used to translate advance()'s absolute `tEnd` into the Rust-relative
  // call (`rustTEnd = tEnd - _loadedAt`). It is set once per successful
  // load(), from whatever `_simTime` already was — INCLUDING right after a
  // FAILED load(): `_simTime` itself is never reset by a failed load (see
  // load()'s own comment), so a fail-then-succeed sequence still resumes at
  // the pre-failure clock instead of restarting at 0.
  private _simTime = 0;
  private _loadedAt = 0;

  constructor() {
    if (!coreReady()) {
      throw new CoreNotInitializedError(
        "new CoreEngine(): initCore() must be awaited (and must have succeeded) before " +
          "constructing a CoreEngine.",
      );
    }
    this._generation = coreGeneration();
    this._handle = coreCall((exports) => exports.bo_engine_create());
    coreEngineFinalizer.register(this, { handle: this._handle, generation: this._generation }, this);
  }

  /**
   * Every public method's first line: disposed, stale-generation, and
   * poisoned are all refused before anything else runs — INCLUDING a pure
   * cached-value getter like getNetV(). A poisoned instance could in
   * principle leave a cached readout technically still accurate (it was
   * copied out before the trap), but "some methods still work while
   * poisoned and some don't" is a worse contract than "poisoned means
   * refuses everything, uniformly, until initCore() runs again."
   */
  private _checkGeneration(): void {
    if (this._disposed) {
      throw new Error("CoreEngine: this engine has been disposed; construct a new one.");
    }
    if (coreGeneration() !== this._generation) {
      throw new CoreStaleInstanceError(
        "this CoreEngine was created against an earlier brownout core instance. initCore() " +
          "has been called again since (e.g. to recover from a poisoned instance), which " +
          "instantiates a fresh module with its own handle table — this engine's handle no " +
          "longer refers to anything there. Construct a new CoreEngine after re-initialising.",
      );
    }
    // A no-op ABI call, purely to route through coreCall's own poisoned/
    // uninitialized checks (CorePoisonedError / CoreNotInitializedError).
    coreCall((exports) => exports);
  }

  /**
   * True when this engine is still safe to use RIGHT NOW: not disposed, its
   * recorded generation matches the live one, and the live instance is
   * `"ready"` (not poisoned, not uninitialized). Never throws — unlike
   * `_checkGeneration()`, this is a query a caller uses to decide WHETHER to
   * call a method, not a guard inside one. A host adapter holding a
   * long-lived `CoreEngine` (HeadlessRunner) uses this to notice when its
   * cached engine has been superseded by a re-init (e.g. trap recovery) and
   * needs to be dropped and replaced, rather than finding out via a thrown
   * `CoreStaleInstanceError` on the next call.
   */
  isCurrent(): boolean {
    return !this._disposed && coreStatus() === "ready" && coreGeneration() === this._generation;
  }

  /** Every readout's guard: generation-fresh AND a program is currently (successfully) loaded. */
  private _requireLoaded(): CompiledCircuit {
    this._checkGeneration();
    if (!this._compiled) {
      throw new CoreNotLoadedError(
        "CoreEngine: no program is loaded — load() has not been called, or the most recent " +
          "call failed. See load()'s doc comment: a failed load() always clears the previously " +
          "loaded program's readouts too.",
      );
    }
    return this._compiled;
  }

  /**
   * Compile `circuit` and load it onto the core. `implementedKinds` is NOT
   * the "safe to route here" accuracy gate (that's `CORE_SUPPORTED_KINDS` /
   * select.ts, a host-adapter concern) — it is only "which kinds can this
   * compiler even produce a program for," and almost never needs to be
   * narrowed by a caller of `CoreEngine` directly.
   *
   * `options.simTime`, when given, is the facade time the JUST-LOADED
   * program starts at — overriding whatever this engine's own clock already
   * read, rather than carrying it forward as a bare `load(circuit)` does
   * (see the class doc comment on `_simTime`/`_loadedAt`). This is for a host
   * adapter switching a live-edit reload from ANOTHER engine onto this one
   * (HeadlessRunner.load(): "classic to core" passes the classic engine's
   * current simTime here) — a caller loading onto an engine that was already
   * running should not pass it. Must be finite and `>= 0`, or this throws a
   * `RangeError` before anything else (before even a topology/parameter
   * check) — a caller bug, not a circuit problem, so unlike a rejected
   * circuit it does NOT clear this engine's currently loaded program.
   *
   * Throws `CoreUnsupportedError` (with `reasons`) for anything compileCircuit
   * rejects, or for a Rust-side topology rejection (`bo_engine_load` code
   * -3). Throws `CoreLoadError` for -1/-2/-4/-5 (see `mapLoadFailure`'s doc
   * comment for why -2 is grouped there, not with -3). In every OTHER
   * throwing case this engine ends up in the "no program loaded" state (see
   * the class doc comment) regardless of what was loaded before.
   */
  load(
    circuit: SimCircuit,
    implementedKinds: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS,
    options?: { simTime?: number },
  ): void {
    this._checkGeneration();

    const requestedSimTime = options?.simTime;
    if (requestedSimTime !== undefined && (!Number.isFinite(requestedSimTime) || requestedSimTime < 0)) {
      throw new RangeError(
        `CoreEngine.load: options.simTime must be a finite number >= 0, got ${String(requestedSimTime)}.`,
      );
    }

    // The facade CLOCK (`_simTime`) is independent of `_compiled` on
    // purpose: clearing the loaded PROGRAM's state below must not reset the
    // clock, even when `_compiled` is ALREADY null going in (a previous
    // load() call failed). Reading `this._simTime` here (unless the caller
    // supplied an explicit `options.simTime` to seed it from) — rather than
    // branching on `_compiled` — is what makes a fail-then-succeed sequence
    // resume at the pre-failure time instead of restarting at 0; see
    // engine.test.ts's "does not reset the clock" test for exactly this.
    const previousSimTime = requestedSimTime ?? this._simTime;

    // Clear the PROGRAM state now. Every throw from this point on leaves the
    // engine correctly reporting "no program loaded" instead of re-serving
    // the OLD circuit's readouts (see the class doc comment for why this
    // diverges from what the Rust side itself does on a failed reload).
    this._compiled = null;
    this._netV = {};
    this._elementI = {};
    this._lastConverged = false;

    const result = compileCircuit(circuit, implementedKinds);
    if (!result.ok) {
      throw new CoreUnsupportedError(result.reasons);
    }
    const { compiled } = result;
    const program = compiled.program;

    coreCall((exports) => {
      // `>>> 0`: bo_alloc returns a wasm32 pointer, which crosses the ABI as
      // a signed i32 — JS sees a negative number once the address reaches
      // 2^31, which a typed-array view built on it would reject outright.
      const ptr = exports.bo_alloc(program.length) >>> 0;
      try {
        // A fresh view: `bo_alloc` may have grown memory, detaching any view
        // taken before this call (wasm.ts / core/README.md, "Memory").
        new Float64Array(exports.memory.buffer, ptr, program.length).set(program);
        const code = exports.bo_engine_load(this._handle, ptr, program.length);
        if (code !== 0) {
          throw mapLoadFailure(code, readCoreErrorMessage(exports));
        }
      } finally {
        exports.bo_free(ptr, program.length);
      }
    });

    // Success: adopt the new program. `_simTime` itself does not change here
    // (it already equals `previousSimTime`; nothing touched it above), but
    // `_loadedAt` records where the JUST-LOADED program's Rust-internal t=0
    // sits in facade time, for advance() to translate through.
    this._compiled = compiled;
    this._loadedAt = previousSimTime;
    this._simTime = previousSimTime;
    this._lastConverged = true;
    this._publishReadouts();
  }

  /**
   * Advance to `tEnd` (absolute seconds, in the FACADE's clock — see the
   * class doc comment). `tEnd` earlier than the current simTime by more than
   * float slack throws a `RangeError` (core/README.md error -6): a caller
   * bug, not a circuit problem, so it is never wrapped in `CoreUnsupportedError`/
   * `CoreLoadError`. A non-finite `tEnd` (`NaN`/`Infinity`/`-Infinity`) is
   * rejected the same way, and BEFORE any ABI call: the Rust side's own
   * backwards-time guard (`t_end < self.time - TIME_EPS`) is false for NaN,
   * so a NaN or Infinite `tEnd` would otherwise reach `bo_engine_advance`,
   * succeed, and permanently disable that guard (every later comparison
   * against a NaN/Infinite clock is also false) — see engine.rs's `advance`.
   */
  advance(tEnd: number): void {
    this._requireLoaded();
    if (!Number.isFinite(tEnd)) {
      throw new RangeError(`CoreEngine.advance: tEnd must be a finite number, got ${String(tEnd)}.`);
    }
    const rustTEnd = tEnd - this._loadedAt;
    const code = coreCall((exports) => exports.bo_engine_advance(this._handle, rustTEnd));
    if (code !== 0) {
      if (code === -6) {
        throw new RangeError(
          `CoreEngine.advance(${tEnd}): time cannot move backwards (current simTime is ${this._simTime}).`,
        );
      }
      const message = coreCall((exports) => readCoreErrorMessage(exports));
      throw new CoreLoadError(`bo_engine_advance failed (code ${code}): ${message}`);
    }
    // Set to exactly `tEnd`, not re-derived from `bo_engine_time()`'s
    // readback (`_loadedAt + bo_engine_time(...)` is not guaranteed to equal
    // `tEnd` bit-for-bit after the subtract/re-add round trip) — the
    // contract guarantees bo_engine_advance either reaches `t_end` exactly
    // or fails outright (core/README.md: "Advance to t_end"), so the value
    // the caller asked for IS the value now current.
    this._simTime = tEnd;
    this._lastConverged = true;
    this._publishReadouts();
  }

  /**
   * Re-reads node voltages / device currents into `_netV`/`_elementI` after
   * a successful load()/advance(). Called only while `_compiled` is set.
   */
  private _publishReadouts(): void {
    const compiled = this._compiled;
    if (!compiled) return;
    const nodeCount = compiled.netIdOfNode.length;
    const deviceCount = compiled.devices.length;

    // Seeded with "gnd": 0 UNCONDITIONALLY, matching classic exactly —
    // sim-engine.ts's _solve() always seeds `const netV = { gnd: 0 }` before
    // overlaying real nets, so a circuit with no net actually named "gnd"
    // (a synthetic datum) still publishes a phantom `netV.gnd === 0` key.
    // See engine.test.ts for the parity check against a real classic load.
    const netV: Record<string, number> = { gnd: 0 };
    const elementI: Record<string, number> = {};

    coreCall((exports) => {
      // `>>> 0` on both pointers: see load()'s bo_alloc comment — same i32
      // sign issue for every pointer this ABI returns.
      const voltagesPtr = exports.bo_engine_node_voltages(this._handle) >>> 0;
      const voltages = copyF64(exports, voltagesPtr, nodeCount);
      for (let node = 0; node < nodeCount; node++) {
        const netId = compiled.netIdOfNode[node];
        if (netId !== null && netId !== undefined) netV[netId] = voltages[node] ?? 0;
      }

      if (deviceCount > 0) {
        const currentsPtr = exports.bo_engine_device_currents(this._handle) >>> 0;
        const currents = copyF64(exports, currentsPtr, deviceCount);
        compiled.devices.forEach((device, i) => {
          elementI[device.componentId] = currents[i] ?? 0;
        });
      }
    });

    this._netV = netV;
    this._elementI = elementI;
  }

  get simTime(): number {
    this._requireLoaded();
    return this._simTime;
  }

  get nets(): Net[] {
    return this._requireLoaded().nets;
  }

  get lastConverged(): boolean {
    this._requireLoaded();
    return this._lastConverged;
  }

  /** Net id -> volts. LIVE record — see surface.ts's EngineReadout doc comment. */
  getNetV(): Record<string, number> {
    this._requireLoaded();
    return this._netV;
  }

  /** Component id -> amps. LIVE record, same caveat as getNetV(). */
  getElementI(): Record<string, number> {
    this._requireLoaded();
    return this._elementI;
  }

  /**
   * W0 implements only ground/resistor/voltage_source (core/README.md) — no
   * kind produces digital output yet. Always empty — a FRESH object per
   * call, like classic's own per-engine field: a shared constant here would
   * let one caller's mutation of the "empty" record leak into every other
   * engine (and every other call).
   */
  get digitalState(): Record<string, number> {
    this._requireLoaded();
    return {};
  }

  /** No W0 kind has a failure model. Always empty (a fresh object per call — see digitalState's doc comment). */
  getFailures(): Record<string, SimFailure> {
    this._requireLoaded();
    return {};
  }

  /** No W0 kind raises a topology/measurement warning. Always empty (a fresh object per call — see digitalState's doc comment). */
  getWarnings(): Record<string, SimWarning> {
    this._requireLoaded();
    return {};
  }

  takeNewWarnings(): SimWarning[] {
    this._requireLoaded();
    return [];
  }

  /** No W0 kind drives a display (MAX7219/HD44780/micro:bit decoders arrive with the kinds that need them). Always empty (a fresh object per call — see digitalState's doc comment). */
  getDisplayState(): Record<string, DisplayInfo> {
    this._requireLoaded();
    return {};
  }

  getNetIdForPin(componentId: string, pinId: string): string | undefined {
    const compiled = this._requireLoaded();
    const node = compiled.pinNode.get(`${componentId}\u0000${pinId}`);
    if (node === undefined) return undefined;
    return compiled.netIdOfNode[node] ?? undefined;
  }

  stats(): CoreEngineStats {
    this._requireLoaded();
    return coreCall((exports) => ({
      partitions: exports.bo_engine_stat(this._handle, 0),
      staticSolves: exports.bo_engine_stat(this._handle, 1),
      largestPartition: exports.bo_engine_stat(this._handle, 2),
      fixedNodes: exports.bo_engine_stat(this._handle, 3),
    }));
  }

  /** Idempotent. Frees the wasm-side handle — but only if it still belongs to the live instance (see the class doc comment on generations). */
  dispose(): void {
    if (this._disposed) return;
    this._disposed = true;
    coreEngineFinalizer.unregister(this);
    try {
      if (coreGeneration() === this._generation) {
        coreCall((exports) => exports.bo_engine_destroy(this._handle));
      }
    } catch {
      // Best-effort: a poisoned instance can't run bo_engine_destroy either,
      // and there is nothing to free in an instance that's already been
      // replaced by a later initCore() call. dispose() must never throw.
    }
  }
}

interface FinalizationToken {
  handle: number;
  generation: number;
}

/**
 * GC safety net for a `CoreEngine` the host forgot to `dispose()`. Frees the
 * wasm-side handle once JS reclaims the facade object — but ONLY if the
 * generation it belonged to is still the live one; after a re-init the old
 * instance is gone entirely, and the handle NUMBER could otherwise
 * coincidentally name a real, unrelated engine in the new instance.
 */
const coreEngineFinalizer = new FinalizationRegistry<FinalizationToken>((token) => {
  try {
    if (coreGeneration() !== token.generation) return;
    coreCall((exports) => exports.bo_engine_destroy(token.handle));
  } catch {
    // Never throw from a finalizer callback.
  }
});
