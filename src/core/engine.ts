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

import { compileCircuit, LIVE_PARAM_SPECS, type CompiledCircuit, type CompileOptions } from "./compile/index.js";
import { CORE_IMPLEMENTED_KINDS, CORE_KIND_CODES, CORE_RECORD_SHAPES, type CoreKindCodeName } from "./kinds.js";
import { buildDigitalState, buildElementChannelI, buildElementI, buildNetV } from "./readout.js";
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

/** `CoreEngine.load`'s third argument: `CompileOptions` (compile/index.js's options passthrough) plus the facade-clock seed. */
export interface CoreEngineLoadOptions extends CompileOptions {
  simTime?: number;
}

/** One `CoreEngine.setParams` change: every named param in `params` is applied to the device compiled for `componentId`. */
export interface CoreParamChange {
  readonly componentId: string;
  readonly params: Readonly<Record<string, number | string>>;
}

/** `bo_engine_stat`'s four counters (core/README.md's index table), named. */
export interface CoreEngineStats {
  partitions: number;
  staticSolves: number;
  largestPartition: number;
  fixedNodes: number;
}

/**
 * Maps a non-zero `bo_engine_load` return code to the TS error the contract
 * assigns it (core/README.md's error-code table). -3 (unsupported topology)
 * and -7 (delta-cycle livelock) are both "route this circuit elsewhere" —
 * -2 (unknown kind code) is NOT a fallback case despite superficially
 * reading like one: compileCircuit only ever emits kind codes from
 * format.ts's CORE_KIND_CODE, which is exactly the set CORE_IMPLEMENTED_KINDS
 * lets through, so the Rust reader rejecting one it doesn't recognise means
 * the TS and Rust halves are out of step with each other, not a fact about
 * the user's circuit. Folding it into `unsupported-kind:<message>` would
 * also corrupt that reason vocabulary: the message here is prose like
 * "device[3]: unknown kind code 7", not a kind name.
 *
 * WHY -7 FALLS BACK HERE TOO (W1c, decision 8). `bo_engine_load` runs its own
 * zero-delay settle pass at load time (decision 8, option C: "settle
 * zero-delay first observations to a fixed point at t=0 ... no rescue ladder
 * (a failed t=0 solve returns an error and the circuit falls back to
 * classic)"). A combinational ring whose delta cycle never reaches a fixed
 * point exceeds the per-instant pass cap and gets -7 from THAT settle pass,
 * before any `advance()` call ever runs — core/README.md's own error-code
 * table names -7 "delta-cycle livelock", generically, not only an
 * advance()-time condition. Before this fix, -7 at load fell through to the
 * generic `CoreLoadError` branch below, which `HeadlessRunner._tryLoadCore`
 * does not catch (only `CoreUnsupportedError` triggers its classic
 * fallback — see headless.ts) — so a livelocked load would propagate out of
 * `HeadlessRunner.load()` as an uncaught throw instead of the documented
 * "raises -7, then the circuit falls back to classic" outcome. This is
 * distinct from a -7 returned by `advance()` (a MID-RUN livelock after a
 * program already loaded successfully): that case is unaffected by this
 * function — see `advance()`'s own -4/-7 handling below, which keeps
 * classic's non-convergence semantics (`lastConverged = false`), not a
 * fallback, because there is no "elsewhere" to route an already-running
 * program to.
 */
function mapLoadFailure(code: number, message: string): CoreUnsupportedError | CoreLoadError {
  if (code === -3) {
    return new CoreUnsupportedError([`topology:${message}`]);
  }
  if (code === -7) {
    return new CoreUnsupportedError([`livelock:${message}`]);
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
  private _digitalState: Record<string, number> = {};
  private _elementChannelI: Record<string, number[]> = {};
  private _lastConverged = false;

  // Facade-side clock. Classic SimEngine.load() never resets `this.simTime`
  // — only coldLoad() does (confirmed by reading sim-engine.ts: `load()` has
  // no `this.simTime = 0` anywhere in it) — so a live-edit reload must keep
  // the facade's simTime moving forward exactly like classic does.
  //
  // Unlike W0, this is no longer just a facade-side fiction layered on top of
  // a Rust clock that restarts at 0: load() always passes the carried-over
  // simTime as `START_TIME` (option key 7, core/README.md's Options table),
  // so RUST'S OWN internal clock is seeded at the same absolute instant. That
  // is what lets `advance(tEnd)` hand `tEnd` to `bo_engine_advance` directly
  // — no TS-side offset arithmetic translates it first — and it's the fix for
  // the reload bug this replaced: before START_TIME existed, Rust always
  // restarted at 0 regardless of the facade's clock, so a time-dependent read
  // (a floating-input bucket, a waveform phase) taken right after a non-zero-
  // time reload would key on the wrong instant.
  //
  // `_simTime` is the SOURCE OF TRUTH for the public `simTime` getter. On a
  // successful advance() it is set to exactly `tEnd` — not re-derived from
  // `bo_engine_time()`'s readback — because the contract guarantees
  // `bo_engine_advance` either reaches `t_end` exactly (Rust stores the f64
  // it is given, with no arithmetic done on it in TS first now) or fails
  // outright, so the value the caller asked for IS the value now current. On
  // a NON-CONVERGENT advance() (-4/-7, core/README.md's "A failed advance"),
  // `_simTime` IS read back from `bo_engine_time()` instead: Rust kept the
  // state of the last instant it actually completed, and that instant — not
  // the `tEnd` that was requested — is what the caller must see.
  private _simTime = 0;

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
   * (see the class doc comment on `_simTime`). This is for a host
   * adapter switching a live-edit reload from ANOTHER engine onto this one
   * (HeadlessRunner.load(): "classic to core" passes the classic engine's
   * current simTime here) — a caller loading onto an engine that was already
   * running should not pass it. Must be finite and `>= 0`, or this throws a
   * `RangeError` before anything else (before even a topology/parameter
   * check) — a caller bug, not a circuit problem, so unlike a rejected
   * circuit it does NOT clear this engine's currently loaded program.
   *
   * `options`' other fields (`integrationMethod`, and the exactness switches)
   * pass straight through to `compileCircuit` (compile/index.js's
   * `CompileOptions`) — the ambient temperature option is NOT here, since
   * `compileCircuit` always derives it from `circuit.environment` itself.
   * `startTimeS` is the one exception: this method ALWAYS overrides it with
   * `previousSimTime` (below) before calling compileCircuit, regardless of
   * whether the caller's `options` happened to set it — "the facade passes
   * START_TIME on every load" means this engine's own carried-over clock is
   * the only source for it, never a caller-supplied compile option.
   *
   * Throws `CoreUnsupportedError` (with `reasons`) for anything compileCircuit
   * rejects, for a Rust-side topology rejection (`bo_engine_load` code -3,
   * reason prefixed `topology:`), or for a load-time delta-cycle livelock
   * (code -7, reason prefixed `livelock:` — decision 8's load-settle pass
   * exceeding the per-instant delta-cycle cap; see `mapLoadFailure`'s doc
   * comment). Throws `CoreLoadError` for -1/-2/-4/-5/-8 (see
   * `mapLoadFailure`'s doc comment for why -2 is grouped there, not with -3
   * or -7). In every OTHER throwing case this engine ends up in the "no
   * program loaded" state (see the class doc comment) regardless of what was
   * loaded before.
   */
  load(
    circuit: SimCircuit,
    implementedKinds: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS,
    options?: CoreEngineLoadOptions,
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
    this._digitalState = {};
    this._elementChannelI = {};
    this._lastConverged = false;

    // `options`' other fields pass straight through: compileCircuit's
    // `CompileOptions` only reads integrationMethod/noSolutionCache/
    // noRailShortcut/forceStepped/startTimeS, so the extra `simTime` field
    // along for the ride (this method's own, above) is simply never read
    // there. `startTimeS` is forced to `previousSimTime` here — not passed
    // through from the caller — because THIS is "the facade passes
    // START_TIME on every load" (core/README.md, Options key 7): every load,
    // including one with no caller options at all, must seed Rust's clock
    // with wherever this engine's own clock already was.
    const result = compileCircuit(circuit, implementedKinds, { ...(options ?? {}), startTimeS: previousSimTime });
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
    // (it already equals `previousSimTime`; nothing touched it above) — the
    // program was just loaded with START_TIME = previousSimTime (above), so
    // Rust's own internal clock is already at exactly this same instant, and
    // there is no separate facade-side offset left to record (see the class
    // doc comment).
    this._compiled = compiled;
    this._simTime = previousSimTime;
    this._lastConverged = true;
    this._publishReadouts();
  }

  /**
   * Advance to `tEnd` (absolute seconds — Rust's own clock IS the facade's
   * clock now that load() always seeds START_TIME with it; see the class doc
   * comment). `tEnd` earlier than the current simTime by more than float
   * slack throws a `RangeError` (core/README.md error -6): a caller bug, not
   * a circuit problem, so it is never wrapped in `CoreUnsupportedError`/
   * `CoreLoadError`. A non-finite `tEnd` (`NaN`/`Infinity`/`-Infinity`) is
   * rejected the same way, and BEFORE any ABI call: the Rust side's own
   * backwards-time guard (`t_end < self.time - TIME_EPS`) is false for NaN,
   * so a NaN or Infinite `tEnd` would otherwise reach `bo_engine_advance`,
   * succeed, and permanently disable that guard (every later comparison
   * against a NaN/Infinite clock is also false) — see engine.rs's `advance`.
   *
   * A NON-CONVERGENT advance (-4 solve failed, -7 delta-cycle livelock) does
   * NOT throw (core/README.md, "A failed advance": "the host treats this as
   * non-convergence, as classic's `lastConverged = false`... It is not a load
   * error"). Rust keeps the state of the last instant it actually completed,
   * so this sets `lastConverged = false`, reads `simTime` back from
   * `bo_engine_time()` (the instant Rust actually reached, not the `tEnd`
   * that was requested), and leaves the published readouts exactly as the
   * last successful call left them — that IS "the last good instant" the
   * contract promises, so there is nothing to re-publish. Every other
   * non-zero code is still a genuine error and throws `CoreLoadError`, as
   * before.
   */
  advance(tEnd: number): void {
    this._requireLoaded();
    if (!Number.isFinite(tEnd)) {
      throw new RangeError(`CoreEngine.advance: tEnd must be a finite number, got ${String(tEnd)}.`);
    }
    const code = coreCall((exports) => exports.bo_engine_advance(this._handle, tEnd));
    if (code === 0) {
      // Set to exactly `tEnd`, not re-derived from `bo_engine_time()`'s
      // readback: the contract guarantees bo_engine_advance either reaches
      // `t_end` exactly or fails outright, and `tEnd` now reaches Rust with
      // no TS-side arithmetic done on it first, so the value the caller asked
      // for IS the value now current.
      this._simTime = tEnd;
      this._lastConverged = true;
      this._publishReadouts();
      return;
    }
    if (code === -6) {
      throw new RangeError(
        `CoreEngine.advance(${tEnd}): time cannot move backwards (current simTime is ${this._simTime}).`,
      );
    }
    if (code === -4 || code === -7) {
      this._simTime = coreCall((exports) => exports.bo_engine_time(this._handle));
      this._lastConverged = false;
      return;
    }
    const message = coreCall((exports) => readCoreErrorMessage(exports));
    throw new CoreLoadError(`bo_engine_advance failed (code ${code}): ${message}`);
  }

  /**
   * Applies a live parameter update to already-loaded devices through
   * `bo_engine_set_params` (core/README.md's ABI v2, owner decision 1) rather
   * than a reload — the mechanism `test/core/equivalence/stimuli.ts`'s
   * `replayStimuliOnCore` uses to replay a verify-script phase's toggles
   * without discarding capacitor charge, register contents or pending delays
   * the way a reload would.
   *
   * Each `change.params` entry is resolved through the SAME per-kind value
   * resolver the compiler itself used at load time (`compile/index.js`'s
   * `LIVE_PARAM_SPECS`, built from each kind-family module's own resolver
   * functions — e.g. compile/passives.ts's `resolveSwitchClosed`), so a live
   * toggle and the initial compiled value agree bit for bit ("converting
   * values with the same resolution as the compiler", this worker's brief).
   *
   * Every `(deviceIndex, paramIndex, value)` triple is validated on THIS side
   * before any ABI call — belt and braces, mirroring `ProgramWriter
   * .addDevice`'s own "validate, but the reader is still the real source of
   * truth" stance:
   * - an unknown `componentId` (no device compiled for it), an unknown param
   *   name for that device's kind, or a param that exists but is NOT live
   *   (`CORE_RECORD_SHAPES.get(kindCode).liveParams`) each throw a plain
   *   `RangeError` naming exactly what was wrong — a caller/corpus-data bug,
   *   not a circuit fact, so (like `load()`'s `options.simTime` check) this
   *   is refused before anything else runs and never wrapped in
   *   `CoreUnsupportedError`/`CoreLoadError`. This is what makes a stimulus
   *   phase that sets a non-live param fail clearly instead of being
   *   silently skipped.
   * - every change's every triple is collected FIRST; only once all of them
   *   validate are they sent to `bo_engine_set_params` in ONE call ("applied
   *   as ONE event at the current time", the contract's own wording), and
   *   readouts are republished exactly as after `advance()`. An empty
   *   `changes` (or every change's own `params` empty) makes no ABI call at
   *   all and republishes nothing — there is nothing to apply.
   * - a non-zero `bo_engine_set_params` return (code `-8`, invalid argument —
   *   should not happen given the checks above, since the contract's own
   *   validation is the same rules already enforced here, but the ABI
   *   remains the authority) throws `CoreLoadError`, exactly like a
   *   `bo_engine_load`/`bo_engine_advance` failure.
   */
  setParams(changes: readonly CoreParamChange[]): void {
    const compiled = this._requireLoaded();

    const deviceIndexByComponentId = new Map<string, number>();
    compiled.devices.forEach((device, index) => deviceIndexByComponentId.set(device.componentId, index));

    const triples: number[] = [];
    for (const change of changes) {
      const deviceIndex = deviceIndexByComponentId.get(change.componentId);
      if (deviceIndex === undefined) {
        throw new RangeError(
          `CoreEngine.setParams: no device for component "${change.componentId}" in the loaded program.`,
        );
      }
      const kind = compiled.devices[deviceIndex]!.kind;
      const spec = LIVE_PARAM_SPECS.get(kind);
      const kindCode = CORE_KIND_CODES[kind as CoreKindCodeName];
      const shape = CORE_RECORD_SHAPES.get(kindCode);

      for (const [paramName, rawValue] of Object.entries(change.params)) {
        if (!spec) {
          throw new RangeError(
            `CoreEngine.setParams: component "${change.componentId}" (kind "${kind}") has no live params at all.`,
          );
        }
        const paramIndex = spec.paramNames.indexOf(paramName);
        if (paramIndex === -1) {
          throw new RangeError(
            `CoreEngine.setParams: kind "${kind}" has no param named "${paramName}" (component "${change.componentId}").`,
          );
        }
        if (!shape || !shape.liveParams.includes(paramIndex)) {
          throw new RangeError(
            `CoreEngine.setParams: kind "${kind}"'s param "${paramName}" (index ${paramIndex}) is not live — ` +
              `it can only be set by loading a new program (component "${change.componentId}").`,
          );
        }
        const value = spec.resolveLiveValue(paramName, rawValue);
        triples.push(deviceIndex, paramIndex, value);
      }
    }

    // Nothing to apply — no ABI call, no republish. (`changes` empty, or
    // every change's own `params` empty.)
    if (triples.length === 0) return;

    coreCall((exports) => {
      // `>>> 0`: see load()'s bo_alloc comment — same i32 sign issue for
      // every pointer this ABI returns.
      const ptr = exports.bo_alloc(triples.length) >>> 0;
      try {
        new Float64Array(exports.memory.buffer, ptr, triples.length).set(triples);
        const code = exports.bo_engine_set_params(this._handle, ptr, triples.length);
        if (code !== 0) {
          throw new CoreLoadError(`bo_engine_set_params failed (code ${code}): ${readCoreErrorMessage(exports)}`);
        }
      } finally {
        exports.bo_free(ptr, triples.length);
      }
    });

    this._publishReadouts();
  }

  /**
   * Re-reads node voltages / device currents / digital slots / channel slots
   * into `_netV`/`_elementI`/`_digitalState`/`_elementChannelI` after a
   * successful load()/advance(), via readout.ts's pure builders. Called only
   * while `_compiled` is set.
   *
   * Slot counts come from `compiled.digitalSlots.length`/
   * `compiled.channelSlots.length` — the exact counts this circuit's program
   * was written with (compile/index.js's `ProgramWriter.finish` call) —
   * rather than an extra `bo_engine_digital_slot_count`/
   * `bo_engine_channel_slot_count` round trip, the same way `nodeCount`/
   * `deviceCount` above already trust the TS-side compiled table instead of
   * calling `bo_engine_node_count`/`bo_engine_device_count`.
   */
  private _publishReadouts(): void {
    const compiled = this._compiled;
    if (!compiled) return;
    const nodeCount = compiled.netIdOfNode.length;
    const deviceCount = compiled.devices.length;
    const digitalSlotCount = compiled.digitalSlots.length;
    const channelSlotCount = compiled.channelSlots.length;

    coreCall((exports) => {
      // `>>> 0` on every pointer: see load()'s bo_alloc comment — same i32
      // sign issue for every pointer this ABI returns.
      const voltagesPtr = exports.bo_engine_node_voltages(this._handle) >>> 0;
      const voltages = copyF64(exports, voltagesPtr, nodeCount);
      this._netV = buildNetV(compiled, voltages);

      if (deviceCount > 0) {
        const currentsPtr = exports.bo_engine_device_currents(this._handle) >>> 0;
        const currents = copyF64(exports, currentsPtr, deviceCount);
        this._elementI = buildElementI(compiled, currents);
      } else {
        this._elementI = {};
      }

      if (digitalSlotCount > 0) {
        const digitalPtr = exports.bo_engine_digital_slots(this._handle) >>> 0;
        const digital = copyF64(exports, digitalPtr, digitalSlotCount);
        this._digitalState = buildDigitalState(compiled, digital);
      } else {
        this._digitalState = {};
      }

      if (channelSlotCount > 0) {
        const channelPtr = exports.bo_engine_channel_slots(this._handle) >>> 0;
        const channel = copyF64(exports, channelPtr, channelSlotCount);
        this._elementChannelI = buildElementChannelI(compiled, channel);
      } else {
        this._elementChannelI = {};
      }
    });
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
   * Rebuilt from the digital slots after every load()/advance()
   * (readout.ts's `buildDigitalState`). W1c's 11 combinational ICs
   * (core/README.md) are the first kind whose compiled program claims a
   * digital slot (compile/ics.ts) — a circuit with none of them still
   * publishes `{}` here, via the same slot-backed machinery, not a
   * hardcoded empty return. The Rust kernel itself does not publish IC
   * slot VALUES until W1c step 2 lands, so every slot currently reads
   * `NaN` (an absent key, per readout.ts's `buildDigitalState`) until then.
   */
  get digitalState(): Record<string, number> {
    this._requireLoaded();
    return this._digitalState;
  }

  /** Component id -> per-channel amps, from the channel slots. LIVE record, same caveat as getNetV(). Matches classic's `getElementChannelI()` shape (spec §10.10). */
  getElementChannelI(): Record<string, number[]> {
    this._requireLoaded();
    return this._elementChannelI;
  }

  /**
   * The facade-time (absolute, like `simTime`/`advance()`) of the earliest
   * pending event, or `+Infinity` when none is scheduled
   * (`bo_engine_next_event_time`, core/README.md's C ABI v2 table). Returned
   * exactly as Rust reports it, with no offset added: Rust's own clock IS the
   * facade's clock now that load() always seeds START_TIME with it (see the
   * class doc comment), so there is nothing left to translate. W1a's kinds
   * are static, so this is always `+Infinity` today — the event kernel's own
   * W1a machinery (drivers, deltas) has nothing to schedule until a later
   * sub-wave's kinds exist.
   */
  nextEventTime(): number {
    this._requireLoaded();
    return coreCall((exports) => exports.bo_engine_next_event_time(this._handle));
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
