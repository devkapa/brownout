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
import { decodeCarry, eepromCarryImages, encodeCarry, mapCarry, type CarryRecord } from "./carry.js";
import { withCarryBlock } from "./format.js";
import { CORE_IMPLEMENTED_KINDS, CORE_KIND_CODES, CORE_RECORD_SHAPES, type CoreKindCodeName } from "./kinds.js";
import { buildDigitalState, buildElementChannelI, buildElementI, buildNetV } from "./readout.js";
import {
  coreCall,
  coreGeneration,
  coreReady,
  coreStatus,
  copyF64,
  copyRecords,
  readCoreErrorMessage,
  type CoreExports,
} from "./wasm.js";
import {
  CORE_RECORD_CHANNEL,
  decodeRecords,
  failureListFromRecords,
  failuresFromRecords,
  ptcTrippedFromRecords,
  thermalReadout,
  WarningLedger,
  type CoreSlowRecord,
} from "./failures.js";
import {
  CoreInitError,
  CoreLoadError,
  CoreNotInitializedError,
  CoreNotLoadedError,
  CoreStaleInstanceError,
  CoreUnsupportedError,
} from "./errors.js";
import type { EngineReadout } from "./surface.js";
import type {
  BatteryRuntimeReadout,
  SimCircuit,
  SimFailure,
  SimWarning,
  ThermalRuntimeReadout,
} from "../sim/engine/sim-engine.js";
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
 * WHY -7 (AND -4, FINDING 11) FALL BACK HERE TOO (W1c, decision 8).
 * `bo_engine_load` runs its own zero-delay settle pass at load time (decision
 * 8, option C: "settle zero-delay first observations to a fixed point at t=0
 * ... no rescue ladder (a failed t=0 solve returns an error and the circuit
 * falls back to classic)"). A combinational ring whose delta cycle never
 * reaches a fixed point exceeds the per-instant pass cap and gets -7 from
 * THAT settle pass, before any `advance()` call ever runs — core/README.md's
 * own error-code table names -7 "delta-cycle livelock", generically, not only
 * an advance()-time condition. Before this fix, -7 at load fell through to
 * the generic `CoreLoadError` branch below, which `HeadlessRunner._tryLoadCore`
 * does not catch (only `CoreUnsupportedError` triggers its classic
 * fallback — see headless.ts) — so a livelocked load would propagate out of
 * `HeadlessRunner.load()` as an uncaught throw instead of the documented
 * "raises -7, then the circuit falls back to classic" outcome.
 *
 * -4 (solve failed) at LOAD time is exactly the same class of failure: W1c's
 * IC-supply fixed point (README's digital-core point 3) is ALSO part of
 * decision 8's t=0 settle, and re-stamps from the latest iterate "until
 * classic's Newton test passes ... at most 50 iterations; failure is -4" —
 * an ordinary, IC-supply-loading-reachable failure (review finding 11), not a
 * rare pathology. Every call into `mapLoadFailure` is from the LOAD path
 * only, so a -4 it sees is BY CONSTRUCTION a failed t=0 solve, covered by the
 * exact same decision-8 sentence that already routes -7 here. This is
 * distinct from a -4/-7 returned by `advance()` (a MID-RUN non-convergence
 * after a program already loaded successfully): that case is unaffected by
 * this function — see `advance()`'s own -4/-7 handling below, which keeps
 * classic's non-convergence semantics (`lastConverged = false`), not a
 * fallback, because there is no "elsewhere" to route an already-running
 * program to.
 */
/**
 * W2c: a carry take's failure (`bo_engine_carry_take`), as a load's: its
 * clamp's `-4`/`-7` route the reload as a load-time solve failure or
 * livelock does (to classic, whose own load carries), reason prefixed
 * `carry:`.
 */
function carryFailure(code: number, message: string): CoreUnsupportedError | CoreLoadError {
  const mapped = mapLoadFailure(code, message);
  return mapped instanceof CoreUnsupportedError ? new CoreUnsupportedError(mapped.reasons.map((r) => `carry:${r}`)) : new CoreLoadError(`bo_engine_carry_take failed (code ${code}): ${message}`);
}

function mapLoadFailure(code: number, message: string): CoreUnsupportedError | CoreLoadError {
  if (code === -3) {
    return new CoreUnsupportedError([`topology:${message}`]);
  }
  if (code === -7) {
    return new CoreUnsupportedError([`livelock:${message}`]);
  }
  if (code === -4) {
    return new CoreUnsupportedError([`solve-failed:${message}`]);
  }
  // -1 (malformed), -2 (unknown kind code — a TS/Rust drift bug, see above),
  // -5 (bad handle), and anything this build doesn't otherwise define: none
  // of these mean "route this circuit elsewhere".
  return new CoreLoadError(`bo_engine_load failed (code ${code}): ${message}`);
}

/**
 * `bo_engine_device_state`'s layouts (core/README.md, C ABI): each value's
 * name, in index order, so a reader names values and never indexes offsets.
 * The length is the layout's version. A later wave only appends to a kind's
 * layout, so a returned length other than this one means the wasm and this
 * facade were built from different contracts.
 */
const BATTERY_STATE_LAYOUT = [
  "soc",
  "openCircuitVoltageV",
  "internalResistanceOhm",
  "remainingCapacityCoulombs",
  "dischargedCoulombs",
  "rejectedRechargeCoulombs",
] as const;
/** A `bench_psu` with `iLimit > 0`: its regime, 0 (CV) or 2 (CC). */
const BENCH_PSU_STATE_LAYOUT = ["reg"] as const;
/** A `bench_psu` with `iLimit = 0`, an ideal source: nothing to read, though the collect succeeds. */
const EMPTY_STATE_LAYOUT = [] as const;
/** A `resistor` or `led` with a code 91 record (W1g point 3); `thermalShutdown` is 0 or 1. */
const THERMAL_STATE_LAYOUT = ["temperatureC", "dissipatedPowerW", "thermalShutdown"] as const;

/**
 * Whether this is a development build, for diagnostics a production host must
 * never pay for or print. Bundlers (webpack, vite, Next) replace the literal
 * `process.env.NODE_ENV` with a string at build time, so this reads correctly
 * inside a browser worker too; where nothing defines `process` at all (an
 * unbundled browser), the ReferenceError lands in the catch and counts as
 * production, the side that stays silent.
 */
function isDevBuild(): boolean {
  try {
    return process.env.NODE_ENV !== "production";
  } catch {
    return false;
  }
}

type DeviceStateLayout = readonly string[];
/** One collect, each value under its layout name. */
type DeviceState<L extends DeviceStateLayout> = { readonly [K in L[number]]: number };

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
  /** The loaded circuit, which `resetFailures` re-derives the static warnings from, as classic's `this.circuit`. */
  private _circuit: SimCircuit | null = null;
  /**
   * W2c: the loaded circuit as it was at the load, which the next reload's
   * carry maps from. A copy: a host may edit the circuit object it loaded in
   * place and reload that same object (classic keeps what it needs at load,
   * its EEPROM's contents hash and a battery's authored charge).
   */
  private _loadedAs: SimCircuit | null = null;
  /** The static warnings and the announce queue (W1g point 11). A reload carries a same-kind component's entries, as classic's load (W2c); coldLoad starts afresh. */
  private _warnings = new WarningLedger();
  /**
   * W2c: each EEPROM's image as this facade last knew it (the ROM blob it
   * loaded, then every image `takeEepromUpdates` or a reload drained), by
   * component id: what a reload onto the same authored contents loads as
   * the new ROM blob, so a written byte survives it as in classic.
   */
  private _eepromImages = new Map<string, Uint8Array>();
  /** Component ids `setParams` has already reported as unknown, so a host retrying the same stale id every frame logs it once (ruling R41). Per engine, kept across loads. */
  private _reportedUnknownParamIds = new Set<string>();
  /** Bumped at every load() call, so a snapshot knows whether the program it was taken on is still the one loaded (W2a). */
  private _loadSerial = 0;
  /** W2b: the nets the observer samples, in take order (`null`: disarmed); a load disarms. */
  private _acquired: readonly string[] | null = null;
  /** W2b piece 4: the components whose charge is armed, in take order, with each one's channel count (`null`: disarmed); a load disarms. */
  private _charged: { readonly ids: readonly string[]; readonly channels: readonly number[] } | null = null;

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
   * **Carry** (W2c, classic's `load()`, core/README.md "Carry"). A load
   * onto a loaded program, with no `options.simTime`, carries its state:
   * capacitor charges by id or net-pair fingerprint, IC state and outputs
   * in flight, a battery's charge, the slow physics and latched failures,
   * each kind-gated as classic's, an EEPROM's written image onto the same
   * authored contents, and the warnings of components whose id and kind
   * survive, none announced again. The program goes to the core as v4 with
   * its `CARRY` block. A load given a time (another engine's, or a cold
   * start), a core without the carry exports, and {@link coldLoad} carry
   * nothing. A carry the core fails to take (bringing the old program to its
   * time, `-4`/`-7`) throws `CoreUnsupportedError` with `carry:`-prefixed
   * reasons, the engine left with no program as after any failed load: the
   * host routes it as a load-time failure, or calls {@link coldLoad}.
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
   * reason prefixed `topology:`), for a load-time delta-cycle livelock
   * (code -7, reason prefixed `livelock:`), or for a load-time solve failure
   * (code -4, reason prefixed `solve-failed:` — finding 11: W1c's IC-supply
   * fixed point not converging is an ordinary way to reach this) — decision
   * 8's load-settle pass covers all three; see `mapLoadFailure`'s doc
   * comment. Throws `CoreLoadError` for -1/-2/-5/-8 (see `mapLoadFailure`'s
   * doc comment for why -2 is grouped there, not with -3/-4/-7). In every
   * OTHER throwing case this engine ends up in the "no program loaded" state
   * (see the class doc comment) regardless of what was loaded before.
   */
  load(
    circuit: SimCircuit,
    implementedKinds: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS,
    options?: CoreEngineLoadOptions,
  ): void {
    this._load(circuit, implementedKinds, options, true);
  }

  /**
   * Classic's `coldLoad` (W2c): load `circuit` at time 0 with nothing carried,
   * every capacitor, register and integrator at its power-on value, as a
   * fresh engine would. Throws as {@link load} does.
   */
  coldLoad(
    circuit: SimCircuit,
    implementedKinds: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS,
    options?: Omit<CoreEngineLoadOptions, "simTime">,
  ): void {
    this._load(circuit, implementedKinds, { ...(options ?? {}), simTime: 0 }, false);
  }

  /**
   * W2c's carry, taken off the loaded program before a reload replaces it:
   * the core's records (`bo_engine_carry_take`), what they were taken from,
   * and every EEPROM's current image (the dirty ones drained here: the
   * records carry their dirty flags, so the new program still reports them
   * to the host). `null` when nothing is loaded, or the core has no carry
   * exports (R100 (a): an older core loads cold). A take that fails (its
   * clamp to the engine's time: `-4`, `-7`) is the error `_load` throws once
   * it has cleared the program, as a load-time failure: a reload that
   * silently went cold would drop the charges, registers and latches the
   * user edited around.
   */
  private _takeCarry(): { records: CarryRecord[]; from: { circuit: SimCircuit; compiled: CompiledCircuit } } | Error | null {
    const compiled = this._compiled;
    const circuit = this._loadedAs;
    if (!compiled || !circuit) return null;
    const raw = coreCall((exports) => {
      if (typeof exports.bo_engine_carry_take !== "function" || typeof exports.bo_engine_carry_ptr !== "function") return null;
      const len = exports.bo_engine_carry_take(this._handle);
      if (len < 0) return carryFailure(len, readCoreErrorMessage(exports));
      return copyF64(exports, exports.bo_engine_carry_ptr(this._handle) >>> 0, len);
    });
    if (raw === null || raw instanceof Error) return raw;
    this._drainEepromImages(compiled);
    return { records: decodeCarry(raw), from: { circuit, compiled } };
  }

  private _load(
    circuit: SimCircuit,
    implementedKinds: ReadonlySet<string>,
    options: CoreEngineLoadOptions | undefined,
    carrying: boolean,
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

    // W2c: a reload carries the loaded program's state, as classic's load()
    // does. Not when the caller seeds the clock (`options.simTime`: another
    // engine's time, or a cold start), since the carry is this engine's own,
    // at its own time.
    const taken = carrying && requestedSimTime === undefined ? this._takeCarry() : null;
    const previousWarnings = this._warnings;

    // Clear the PROGRAM state now. Every throw from this point on leaves the
    // engine correctly reporting "no program loaded" instead of re-serving
    // the OLD circuit's readouts (see the class doc comment for why this
    // diverges from what the Rust side itself does on a failed reload).
    // Every load, failed ones included, retires the snapshots taken before
    // it: a successful one replaces Rust's table, and after a failed one no
    // program is loaded here to restore onto.
    this._loadSerial++;
    this._compiled = null;
    // A load disarms the observer and the charge (core/README.md,
    // "Acquisition", "Charge"): the host arms them again for the new program.
    this._acquired = null;
    this._charged = null;
    this._netV = {};
    this._elementI = {};
    this._digitalState = {};
    this._elementChannelI = {};
    this._lastConverged = false;
    this._circuit = null;
    this._loadedAs = null;
    this._warnings = new WarningLedger();
    if (taken instanceof Error) {
      throw taken;
    }
    const carried = taken;

    // `options`' other fields pass straight through: compileCircuit's
    // `CompileOptions` only reads integrationMethod/noSolutionCache/
    // noRailShortcut/forceStepped/startTimeS, so the extra `simTime` field
    // along for the ride (this method's own, above) is simply never read
    // there. `startTimeS` is forced to `previousSimTime` here — not passed
    // through from the caller — because THIS is "the facade passes
    // START_TIME on every load" (core/README.md, Options key 7): every load,
    // including one with no caller options at all, must seed Rust's clock
    // with wherever this engine's own clock already was.
    const eepromImages = carried ? eepromCarryImages(carried.from.circuit, circuit, this._eepromImages) : undefined;
    const result = compileCircuit(circuit, implementedKinds, { ...(options ?? {}), startTimeS: previousSimTime, ...(eepromImages ? { eepromImages } : {}) });
    if (!result.ok) {
      throw new CoreUnsupportedError(result.reasons);
    }
    const { compiled } = result;
    const records = carried ? mapCarry(carried.records, carried.from, { circuit, compiled }) : [];
    const program = records.length > 0 ? withCarryBlock(compiled.program, encodeCarry(records)) : compiled.program;

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
    if (carried) {
      // Classic carries a component's warnings onto the same id and kind.
      const kindBefore = new Map(carried.from.circuit.components.map((c) => [c.id, c.kind]));
      const kindAfter = new Map(circuit.components.map((c) => [c.id, c.kind]));
      this._warnings = previousWarnings.carryOnto((id) => kindBefore.get(id) !== undefined && kindBefore.get(id) === kindAfter.get(id));
    }
    // The images a load put in the ROM blobs that are not the authored
    // contents (carried ones); an EEPROM absent here holds its authoring.
    this._eepromImages = new Map(eepromImages ?? []);
    this._circuit = circuit;
    this._loadedAs = carrySnapshot(circuit);
    // Since START_TIME, as classic's load evaluates them at its simTime.
    this._warnings.evaluateStatic(circuit, compiled.nets, previousSimTime);
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
   * - an unknown `componentId` (no device compiled for it) is skipped, and
   *   the call returns `false` (ruling R41, 2026-10-21). A host edits the
   *   circuit while param changes are in flight, so an id the loaded program
   *   no longer holds is routine; classic has no runtime param call at all
   *   and its reload path ignores an override for an absent component the
   *   same way. Outside production the id is logged once per engine
   *   (`isDevBuild`); the rest of the batch is still validated and applied.
   * - an unknown param name for a known device's kind, or a param that
   *   exists but is NOT live (`CORE_RECORD_SHAPES.get(kindCode).liveParams`)
   *   each throw a plain `RangeError` naming exactly what was wrong — a
   *   caller/corpus-data bug, not a circuit fact, so (like `load()`'s
   *   `options.simTime` check) this is refused before anything else runs and
   *   never wrapped in `CoreUnsupportedError`/`CoreLoadError`. This is what
   *   makes a stimulus phase that sets a non-live param fail clearly instead
   *   of being silently skipped.
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
   *
   * Returns `true` when every change named a component the loaded program
   * holds (an empty `changes` included), `false` when at least one was
   * skipped as unknown.
   */
  setParams(changes: readonly CoreParamChange[]): boolean {
    const compiled = this._requireLoaded();

    const deviceIndexByComponentId = new Map<string, number>();
    compiled.devices.forEach((device, index) => deviceIndexByComponentId.set(device.componentId, index));

    const triples: number[] = [];
    let allKnown = true;
    for (const change of changes) {
      const deviceIndex = deviceIndexByComponentId.get(change.componentId);
      if (deviceIndex === undefined) {
        allKnown = false;
        if (isDevBuild() && !this._reportedUnknownParamIds.has(change.componentId)) {
          this._reportedUnknownParamIds.add(change.componentId);
          console.warn(
            `CoreEngine.setParams: no device for component "${change.componentId}" in the loaded program; ` +
              "the change was skipped.",
          );
        }
        continue;
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

    // Nothing to apply — no ABI call, no republish. (`changes` empty, every
    // change's own `params` empty, or every change skipped as unknown.)
    if (triples.length === 0) return allKnown;

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
    return allKnown;
  }

  /**
   * Seeds a 74ls189's 16 stored nibbles through `bo_engine_seed_memory`
   * (core/README.md's ABI v2, W1d, owner decision 2) — the runtime
   * counterpart to `params.contents` for the EEPROM kinds: a 189 has no
   * compiled contents at all (its RAM always powers on at 0xF, README's
   * memories kernel semantics point 2), so this is the only way to give it a
   * starting word.
   *
   * `nibbles[i]` is the nibble stored at address `i`, in TRUE sense
   * (classic's `mem{i}`; the `/o` pins show its complement) — an integer in
   * `[0, 15]`.
   *
   * Every check below runs BEFORE any ABI call, exactly `setParams`'s own
   * "validate everything on this side first, but the ABI remains the
   * authority" stance:
   * - an unknown `componentId` (no device compiled for it), or one whose
   *   compiled kind is not `74ls189`, throws a plain `RangeError` — a
   *   caller/corpus-data bug, not a circuit fact, so (like `setParams`'s own
   *   unknown-param checks; ruling R41 made only `setParams`'s unknown
   *   component a skip) this is refused before
   *   anything else runs and never wrapped in `CoreUnsupportedError`/
   *   `CoreLoadError`;
   * - `nibbles.length !== 16`, or any value that is not an integer in
   *   `[0, 15]`, throws `RangeError` naming the offending index (README:
   *   "Any other kind, an index out of range, `len_f64 != 16`, or a bad
   *   value is `-8`, validated in full before anything changes" — the checks
   *   here are the TS-side mirror of that same rule, which should make a
   *   non-zero `bo_engine_seed_memory` return unreachable in practice; it
   *   still throws `CoreLoadError` rather than being assumed impossible).
   *
   * Applies as ONE event at the current time (README): the device is
   * re-evaluated in a delta cycle there, so readouts are republished exactly
   * as after `advance()`/`setParams()`.
   */
  setMemoryContents(componentId: string, nibbles: readonly number[]): void {
    const compiled = this._requireLoaded();

    const deviceIndex = compiled.devices.findIndex((device) => device.componentId === componentId);
    if (deviceIndex === -1) {
      throw new RangeError(
        `CoreEngine.setMemoryContents: no device for component "${componentId}" in the loaded program.`,
      );
    }
    const kind = compiled.devices[deviceIndex]!.kind;
    if (kind !== "74ls189") {
      throw new RangeError(
        `CoreEngine.setMemoryContents: component "${componentId}" (kind "${kind}") does not accept a memory ` +
          "seed — only a 74ls189 does.",
      );
    }
    if (nibbles.length !== 16) {
      throw new RangeError(
        `CoreEngine.setMemoryContents: component "${componentId}" needs exactly 16 nibble values, got ` +
          `${nibbles.length}.`,
      );
    }
    nibbles.forEach((value, i) => {
      if (!Number.isInteger(value) || value < 0 || value > 15) {
        throw new RangeError(
          `CoreEngine.setMemoryContents: component "${componentId}"'s nibble[${i}] must be an integer in ` +
            `[0, 15], got ${value}.`,
        );
      }
    });

    coreCall((exports) => {
      // `>>> 0`: see load()'s bo_alloc comment — same i32 sign issue for
      // every pointer this ABI returns.
      const ptr = exports.bo_alloc(16) >>> 0;
      try {
        new Float64Array(exports.memory.buffer, ptr, 16).set(nibbles);
        const code = exports.bo_engine_seed_memory(this._handle, deviceIndex, ptr, 16);
        if (code !== 0) {
          throw new CoreLoadError(`bo_engine_seed_memory failed (code ${code}): ${readCoreErrorMessage(exports)}`);
        }
      } finally {
        exports.bo_free(ptr, 16);
      }
    });

    this._publishReadouts();
  }

  /**
   * Collects every EEPROM image that changed since the last call
   * (`bo_engine_take_eeprom_updates`/`bo_engine_eeprom_updates_ptr`,
   * core/README.md's ABI v2, W1d) and clears their dirty flags — the runtime
   * counterpart to classic's `SimEngine.takeEepromUpdates()`
   * (sim-engine.ts), whose `Record<string, Uint8Array>` shape (component id
   * to a FRESH `Uint8Array(size)` copy) this method reproduces exactly. `{}`
   * is the common no-op path: nothing has written to any EEPROM since the
   * last call (also what the W1d Rust stubs always return today — see this
   * worker's report).
   *
   * The wire buffer is `deviceIndex, W, word x W` per dirty EEPROM (README).
   * Each word is unpacked back into 4 little-endian bytes with plain shifts
   * — never a `Uint32Array`/host-endian view — the exact inverse of
   * compile/memories.ts's own `romWords` packing.
   */
  takeEepromUpdates(): Record<string, Uint8Array> {
    const compiled = this._requireLoaded();
    const out = this._takeEepromUpdatesRaw(compiled);
    for (const [id, bytes] of Object.entries(out)) this._eepromImages.set(id, bytes.slice());
    return out;
  }

  /** A reload's drain: every dirty image into `_eepromImages`, none handed out (the carried dirty flags hand them out of the new program). */
  private _drainEepromImages(compiled: CompiledCircuit): void {
    for (const [id, bytes] of Object.entries(this._takeEepromUpdatesRaw(compiled))) this._eepromImages.set(id, bytes);
  }

  private _takeEepromUpdatesRaw(compiled: CompiledCircuit): Record<string, Uint8Array> {
    return coreCall((exports) => {
      const len = exports.bo_engine_take_eeprom_updates(this._handle);
      if (len < 0) {
        throw new CoreLoadError(
          `bo_engine_take_eeprom_updates failed (code ${len}): ${readCoreErrorMessage(exports)}`,
        );
      }
      if (len === 0) return {};

      // `>>> 0`: see load()'s bo_alloc comment.
      const ptr = exports.bo_engine_eeprom_updates_ptr(this._handle) >>> 0;
      const raw = copyF64(exports, ptr, len);

      const out: Record<string, Uint8Array> = {};
      let offset = 0;
      while (offset < raw.length) {
        const deviceIndex = raw[offset++]!;
        const wordCount = raw[offset++]!;
        const device = compiled.devices[deviceIndex];
        const bytes = new Uint8Array(wordCount * 4);
        for (let k = 0; k < wordCount; k++) {
          const word = raw[offset + k]! >>> 0;
          bytes[4 * k] = word & 0xff;
          bytes[4 * k + 1] = (word >>> 8) & 0xff;
          bytes[4 * k + 2] = (word >>> 16) & 0xff;
          bytes[4 * k + 3] = (word >>> 24) & 0xff;
        }
        offset += wordCount;
        // A device index outside this program's own table would be a
        // Rust/TS drift bug, not a fact about the circuit — skipped
        // defensively rather than thrown, the same stance readTrace() takes
        // for a stale slot index (see its own doc comment).
        if (device) out[device.componentId] = bytes;
      }
      return out;
    });
  }

  /**
   * Classic's `SimEngine.getBatteryState` (core/README.md, Readouts, W1f):
   * `undefined` unless `componentId` is a `battery_pack` in the loaded
   * program. The six numbers are Rust's device state, projected to the
   * current time and committed nowhere (Rails point 6), so polling this never
   * changes the run; the profile and the two temperature fields are the
   * compiler's, fixed per load. A fresh object each call, in classic's field
   * order. Not on `EngineReadout`: no host reads it; gate 1 does, through
   * test/core/suites/adapter.ts.
   */
  getBatteryState(componentId: string): BatteryRuntimeReadout | undefined {
    const compiled = this._requireLoaded();
    const info = compiled.batteries.get(componentId);
    if (info === undefined) return undefined;
    const state = this._readDeviceState(compiled, componentId, BATTERY_STATE_LAYOUT);
    return {
      profileId: info.profileId,
      label: info.label,
      soc: state.soc,
      openCircuitVoltageV: state.openCircuitVoltageV,
      internalResistanceOhm: state.internalResistanceOhm,
      remainingCapacityCoulombs: state.remainingCapacityCoulombs,
      dischargedCoulombs: state.dischargedCoulombs,
      rejectedRechargeCoulombs: state.rejectedRechargeCoulombs,
      modelTemperatureC: info.modelTemperatureC,
      temperatureWasClamped: info.temperatureWasClamped,
    };
  }

  /**
   * Classic's `SimEngine.getIcState` for the one kind W1f mirrors
   * (core/README.md, Readouts): a `bench_psu` with `iLimit > 0` reads
   * `{ reg }`, 0 in CV and 2 in CC. The object is fresh each call, where
   * classic hands out its live `icState` entry; mutating either was never
   * supported. A `bench_psu` with `iLimit = 0` has no regime, so `undefined`,
   * as classic creates no entry. Every other kind is `undefined` too: classic's
   * IC `icState` objects are not mirrored (W1f Q11). Not on `EngineReadout`,
   * for the same reason as `getBatteryState`.
   */
  getIcState(componentId: string): Record<string, number> | undefined {
    const compiled = this._requireLoaded();
    const iLimit = compiled.benchPsuLimits.get(componentId);
    if (iLimit === undefined) return undefined;
    if (!(iLimit > 0)) {
      // Collected anyway: the empty layout is the contract's for an ideal
      // supply, and a value there would mean the two halves disagree.
      this._readDeviceState(compiled, componentId, EMPTY_STATE_LAYOUT);
      return undefined;
    }
    const { reg } = this._readDeviceState(compiled, componentId, BENCH_PSU_STATE_LAYOUT);
    return { reg };
  }

  /**
   * One device's state through `bo_engine_device_state`, under the names of
   * `layout`: the README's layout for the device's kind, picked by the caller
   * because a `bench_psu`'s depends on its compiled `iLimit`.
   *
   * A length other than the layout's means the two halves were built from
   * different contracts, so it throws `CoreInitError`, as an ABI version
   * mismatch does, rather than hand out values under the wrong names. A
   * negative code cannot come from a loaded program and a kind that has a
   * layout either; it throws `CoreLoadError` with Rust's message, as
   * `takeEepromUpdates` does.
   */
  private _readDeviceState<L extends DeviceStateLayout>(
    compiled: CompiledCircuit,
    componentId: string,
    layout: L,
  ): DeviceState<L> {
    const deviceIndex = compiled.devices.findIndex((device) => device.componentId === componentId);
    const kind = compiled.devices[deviceIndex]?.kind;
    return coreCall((exports) => {
      const len = exports.bo_engine_device_state(this._handle, deviceIndex);
      if (len < 0) {
        throw new CoreLoadError(
          `bo_engine_device_state failed for "${componentId}" (code ${len}): ${readCoreErrorMessage(exports)}`,
        );
      }
      if (len !== layout.length) {
        throw new CoreInitError(
          `brownout/core: bo_engine_device_state returned ${len} value(s) for ${kind} "${componentId}", ` +
            `but this package's TS front end reads that layout as ${layout.length} (core/README.md, ` +
            '"bo_engine_device_state"). The TS and Rust halves ship in one npm package and must be ' +
            "rebuilt together.",
        );
      }
      const state: Record<string, number> = {};
      if (len > 0) {
        // `>>> 0`: see load()'s bo_alloc comment.
        const values = copyF64(exports, exports.bo_engine_device_state_ptr(this._handle) >>> 0, len);
        layout.forEach((name, i) => {
          state[name] = values[i]!;
        });
      }
      return state as unknown as DeviceState<L>;
    });
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

  /**
   * The facade-time of the earliest pending DIGITAL DUE TIME only —
   * `bo_engine_next_digital_due_time`, finding 10's narrower counterpart to
   * `nextEventTime()` above. Unlike `nextEventTime()` (which also reports a
   * stepped partition's own internal Step bookkeeping, a closed-form
   * partition's Crossing watch, and a floating-input Bucket boundary), this
   * ignores all three and only ever reports a live driver-commit token —
   * mirroring classic's `nextScheduledEventTime()` (sim-engine.ts), which
   * only ever considers `icState`'s DIGITAL_DUE_PREFIX due times. `+Infinity`
   * when nothing is due. `bo_engine_advance` already processes every
   * intervening Step/Crossing/Bucket event on its own to reach an arbitrary
   * `tEnd`, so a caller (test/core/equivalence/stimuli.ts's
   * `replayStimuliOnCore`) that only wants to clamp a step onto a genuinely
   * due digital transition — never onto the solver's own internal analog
   * bookkeeping — uses this instead of `nextEventTime()`.
   */
  nextDigitalDueTime(): number {
    this._requireLoaded();
    return coreCall((exports) => exports.bo_engine_next_digital_due_time(this._handle));
  }

  /**
   * Classic's `getFailures()` (core/README.md, Readouts, W1g): channel 0 of
   * `bo_engine_records`, projected to the current time, as a fresh object in
   * latch order keyed by classic's `failureKey`, each failure in classic's
   * key order with its message built from classic's template (failures.ts).
   */
  getFailures(): Record<string, SimFailure> {
    const compiled = this._requireLoaded();
    return failuresFromRecords(this._collectRecords(CORE_RECORD_CHANNEL.failures), compiled);
  }

  /**
   * Classic's `getWarnings()`: the facade's static warnings (W1g point 11), a
   * fresh object in latch order. Rust's channel 1 has no producer in W1
   * (`transistor_overcurrent` is W3), so a record there means the halves
   * were built from different contracts (`_checkRustWarnings`).
   */
  getWarnings(): Record<string, SimWarning> {
    this._requireLoaded();
    this._checkRustWarnings();
    return this._warnings.all();
  }

  /**
   * Classic's `takeNewWarnings()`: the warnings latched since the previous
   * call, in latch order, then an empty queue. A warning still true is not
   * announced again; one gone before it was drained never is.
   */
  takeNewWarnings(): SimWarning[] {
    this._requireLoaded();
    this._checkRustWarnings();
    return this._warnings.takeNew();
  }

  /**
   * Classic's `getPtcTripped()` (W1g Q5: on `CoreEngine` only): a fresh set
   * of the tripped `ptc_fuse` ids, channel 2. A trip is never a failure or a
   * warning (Q-O45).
   */
  getPtcTripped(): ReadonlySet<string> {
    const compiled = this._requireLoaded();
    return ptcTrippedFromRecords(this._collectRecords(CORE_RECORD_CHANNEL.ptcTripped), compiled);
  }

  /**
   * Classic's `getThermalState(id)` (W1g Q5: on `CoreEngine` only):
   * `undefined` unless the component has a code 91 record. Otherwise Rust's
   * `[T, P, shutdown]`, projected to now, composed through classic's own
   * `stepThermalDevice` at `dt = 0` (failures.ts `thermalReadout`), so the
   * target, the derating, the warnings and `withinContinuousLimits` are
   * classic's by construction. A fresh object each call.
   */
  getThermalState(componentId: string): ThermalRuntimeReadout | undefined {
    const compiled = this._requireLoaded();
    const thermal = compiled.thermalDevices.get(componentId);
    if (thermal === undefined) return undefined;
    const state = this._readDeviceState(compiled, componentId, THERMAL_STATE_LAYOUT);
    return thermalReadout(
      thermal.profile,
      thermal.catalogUid,
      {
        temperatureC: state.temperatureC,
        dissipatedPowerW: state.dissipatedPowerW,
        thermalShutdown: state.thermalShutdown === 1,
      },
      compiled.ambientC,
    );
  }

  /**
   * Classic's `getPartTemperature(id)` (W1g Q5: on `CoreEngine` only): the
   * package temperature of a part with a code 91 record, else `undefined`.
   * A thermistor's body temperature is W1h's.
   */
  getPartTemperature(componentId: string): number | undefined {
    const compiled = this._requireLoaded();
    if (!compiled.thermalDevices.has(componentId)) return undefined;
    return this._readDeviceState(compiled, componentId, THERMAL_STATE_LAYOUT).temperatureC;
  }

  /**
   * Classic's `resetFailures()` (W1g point 13; Q5: on `CoreEngine` only),
   * through `bo_engine_reset_failures`: one event at the current time, which
   * clears every latch and stress integrator and resets every PTC, then
   * re-solves at that instant without advancing time.
   *
   * On success the warnings and the announce queue clear and the static
   * warnings are re-derived with `since` = the current time, so one still
   * true is announced again (SE:2407-2448). The re-solve converged, so
   * `lastConverged` is true, as classic's seed solve sets it.
   *
   * On `-4` or `-7` Rust has rolled the whole reset back, so this sets
   * `lastConverged = false` and changes nothing else: the failures, the
   * warnings, the queue and the published readouts stay exactly as they were.
   * Classic instead keeps its maps cleared over a non-converged seed; no
   * circuit is known to fail that re-solve on both engines (W1g Q6; ruling
   * D7 (b)).
   */
  resetFailures(): void {
    const compiled = this._requireLoaded();
    const code = coreCall((exports) => exports.bo_engine_reset_failures(this._handle));
    if (code === 0) {
      this._warnings.clear();
      this._warnings.evaluateStatic(this._circuit!, compiled.nets, this._simTime);
      this._lastConverged = true;
      this._publishReadouts();
      return;
    }
    if (code === -4 || code === -7) {
      this._lastConverged = false;
      return;
    }
    const message = coreCall((exports) => readCoreErrorMessage(exports));
    throw new CoreLoadError(`bo_engine_reset_failures failed (code ${code}): ${message}`);
  }

  /**
   * @internal Harness instrumentation, not public API (manager ruling): kept
   * out of the 1.0 API surface and of every changeset. Channel 3, the latch
   * log: every failure that latched since the previous call (or the load),
   * once each, in first-latch order, each with that latch's `since`.
   * Collecting empties it. Classic has no counterpart; the equivalence
   * harness reads it at every advance end so it sees every latch over a run
   * rather than a poll of `getFailures()` (Equivalence harness rules, W1g).
   */
  takeLatchedFailures(): SimFailure[] {
    const compiled = this._requireLoaded();
    return failureListFromRecords(this._collectRecords(CORE_RECORD_CHANNEL.latchLog), compiled);
  }

  /**
   * One record channel, collected and decoded. The decode runs outside
   * `coreCall`: a malformed buffer is a contract drift to report, not a trap
   * that should poison the instance.
   */
  private _collectRecords(channel: number): CoreSlowRecord[] {
    const raw = coreCall((exports) => {
      const out = copyRecords(exports, this._handle, channel);
      if (typeof out === "number") {
        throw new CoreLoadError(
          `bo_engine_records failed for channel ${channel} (code ${out}): ${readCoreErrorMessage(exports)}`,
        );
      }
      return out;
    });
    return decodeRecords(raw);
  }

  /** Channel 1 has no producer in W1 (W1g point 11): a record there means the two halves disagree. */
  private _checkRustWarnings(): void {
    const records = this._collectRecords(CORE_RECORD_CHANNEL.warnings);
    if (records.length > 0) {
      throw new CoreInitError(
        `brownout/core: bo_engine_records returned ${records.length} warning record(s), but this package's TS ` +
          "front end has no Rust warning producer in W1 (core/README.md, W1g point 11). The TS and Rust halves " +
          "ship in one npm package and must be rebuilt together.",
      );
    }
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

  /**
   * `bo_engine_stat` for indices `0 .. count - 1`, copied in one call: the raw
   * counters `stats()` names four of (core/README.md, "Stat indices"). An
   * index the Rust side does not know reads `NaN`, as the ABI says. Read-only;
   * `src/core/diagnostics.ts` turns two reads into a batch's diagnostics.
   */
  statCounters(count: number): number[] {
    this._requireLoaded();
    return coreCall((exports) => {
      const out: number[] = [];
      for (let i = 0; i < count; i++) out.push(exports.bo_engine_stat(this._handle, i));
      return out;
    });
  }

  /**
   * Turns the digital trace (`bo_engine_trace_enable`, core/README.md's ABI
   * v2 "W1a") on or off. Harness instrumentation only (the doc comment on the
   * ABI row itself: "W2's host traces are separate") — a normal host adapter
   * never calls this. `code !== 0` should not happen given `_requireLoaded()`
   * already guarantees a live handle, but the ABI remains the authority, same
   * as every other method here.
   */
  enableTrace(enabled: boolean): void {
    this._requireLoaded();
    const code = coreCall((exports) => exports.bo_engine_trace_enable(this._handle, enabled ? 1 : 0));
    if (code !== 0) {
      const message = coreCall((exports) => readCoreErrorMessage(exports));
      throw new CoreLoadError(`bo_engine_trace_enable failed (code ${code}): ${message}`);
    }
  }

  /**
   * Decodes the raw `(t, slot, value)` f64 triples `bo_engine_trace_ptr`/
   * `_len` publish into `{t, key, value}` events, using THIS program's own
   * `digitalSlots` table (slot index -> `digitalState` key, readout.ts's own
   * convention) — "Keys are built in TS; Rust publishes numbers"
   * (core/README.md, "Readouts"). Copies eagerly (like every other readout
   * here): the trace buffer is only valid until the next ABI call on this
   * handle. A slot index outside this program's own table is skipped rather
   * than throwing — defensive only; a well-formed trace from THIS program's
   * own handle should never produce one.
   */
  readTrace(): { readonly t: number; readonly key: string; readonly value: number }[] {
    const compiled = this._requireLoaded();
    return coreCall((exports) => {
      const len = exports.bo_engine_trace_len(this._handle);
      if (len === 0) return [];
      const ptr = exports.bo_engine_trace_ptr(this._handle) >>> 0;
      const raw = copyF64(exports, ptr, len);
      const events: { t: number; key: string; value: number }[] = [];
      for (let i = 0; i + 2 < raw.length; i += 3) {
        const slot = raw[i + 1]!;
        const key = compiled.digitalSlots[slot];
        if (key === undefined) continue;
        events.push({ t: raw[i]!, key, value: raw[i + 2]! });
      }
      return events;
    });
  }

  /** Resets the trace buffer (`bo_engine_trace_clear`) — does not touch whether tracing is enabled. */
  clearTrace(): void {
    this._requireLoaded();
    coreCall((exports) => exports.bo_engine_trace_clear(this._handle));
  }

  /**
   * W2b (core/README.md, "Acquisition"): samples the nets `netIds` at
   * `origin + k * interval` seconds (`k` whole) from the engine's time on,
   * each sample the model's value there, read without moving a step. At most
   * `limit` samples are held between takes (`0`: 2^20); past that the newest
   * are dropped and counted. Re-arming drops what was not taken; `interval`
   * 0 disarms. A `load()` disarms. Throws `RangeError` for an unknown or
   * repeated net or a bad grid, `CoreLoadError` on a core built before W2b.
   */
  acquire(netIds: readonly string[], origin: number, interval: number, limit = 0): void {
    const compiled = this._requireLoaded();
    if (!Number.isFinite(origin) || !Number.isFinite(interval) || interval < 0) {
      throw new RangeError(`CoreEngine.acquire: origin ${origin} and interval ${interval} must be finite, the interval not negative.`);
    }
    if (!Number.isInteger(limit) || limit < 0 || limit > 0xffffffff) {
      throw new RangeError(`CoreEngine.acquire: limit must be a whole number of samples, got ${limit}.`);
    }
    const nodes = indicesOf(netIds, (id) => compiled.nodeOfNet.get(id), "acquire", "net");
    coreCall((exports) => {
      if (typeof exports.bo_engine_acquire !== "function") {
        throw new CoreLoadError("CoreEngine.acquire: this brownout core module has no acquisition exports (built before W2b).");
      }
      const code = withF64Buffer(exports, nodes, (ptr) => exports.bo_engine_acquire!(this._handle, ptr, nodes.length, origin, interval, limit));
      if (code !== 0) {
        throw new CoreLoadError(`bo_engine_acquire failed (code ${code}): ${readCoreErrorMessage(exports)}`);
      }
    });
    this._acquired = interval > 0 ? [...netIds] : null;
  }

  /**
   * W2b: every sample final since the last take, in time order, and the
   * queue emptied; `null` while disarmed. `maxSpan` is the widest step a
   * stepped net's samples were interpolated across, `fallbacks` per net the
   * samples read linear where R104's solve failed.
   */
  takeSamples(): CoreSamples | null {
    this._requireLoaded();
    const nets = this._acquired;
    if (!nets) return null;
    const raw = coreCall((exports) => {
      const len = exports.bo_engine_acquire_take!(this._handle);
      if (len < 0) throw new CoreLoadError(`bo_engine_acquire_take failed (code ${len}): ${readCoreErrorMessage(exports)}`);
      return copyF64(exports, exports.bo_engine_acquire_ptr!(this._handle) >>> 0, len);
    });
    const count = raw[0]!;
    const head = 4 + nets.length;
    const values: Record<string, Float64Array> = {};
    const fallbacks: Record<string, number> = {};
    nets.forEach((id, j) => {
      fallbacks[id] = raw[4 + j]!;
      values[id] = raw.slice(head + count * (1 + j), head + count * (2 + j));
    });
    return { t: raw.slice(head, head + count), values, maxSpan: raw[1]!, overflowed: raw[2] !== 0, dropped: raw[3]!, fallbacks };
  }

  /**
   * W2b piece 4 (core/README.md, "Charge"): integrates each component's
   * published current, and each of its channels' (an LED family part's),
   * from the engine's time on; a light mean is a charge over its span.
   * Re-arming drops what was not taken; an empty list disarms. A `load()`
   * disarms. Throws `RangeError` for an unknown or repeated component,
   * `CoreLoadError` on a core built before W2b piece 4.
   */
  armCharge(componentIds: readonly string[]): void {
    const compiled = this._requireLoaded();
    const deviceOf = new Map<string, number>();
    compiled.devices.forEach((device, i) => deviceOf.set(device.componentId, i));
    const devices = indicesOf(componentIds, (id) => deviceOf.get(id), "armCharge", "component");
    coreCall((exports) => {
      if (typeof exports.bo_engine_charge_arm !== "function") {
        throw new CoreLoadError("CoreEngine.armCharge: this brownout core module has no charge exports (built before W2b piece 4).");
      }
      const code = withF64Buffer(exports, devices, (ptr) => exports.bo_engine_charge_arm!(this._handle, ptr, devices.length));
      if (code !== 0) {
        throw new CoreLoadError(`bo_engine_charge_arm failed (code ${code}): ${readCoreErrorMessage(exports)}`);
      }
    });
    const channelCount = new Map<string, number>();
    for (const [id] of compiled.channelSlots) channelCount.set(id, (channelCount.get(id) ?? 0) + 1);
    this._charged = devices.length > 0 ? { ids: [...componentIds], channels: componentIds.map((id) => channelCount.get(id) ?? 0) } : null;
  }

  /**
   * W2b piece 4: each armed component's charge (coulombs) and its channels'
   * over `[t0, t1]` since the last take (or the arming), the sums cleared;
   * `null` while disarmed. `finalUpTo` is how far each one's charge is
   * final: `t1` after `advance()`, and after a yield of `advanceFor` a part a
   * stepped partition publishes is final only to that partition's last
   * step, the rest coming with a later take.
   */
  takeCharge(): CoreCharge | null {
    this._requireLoaded();
    const armed = this._charged;
    if (!armed) return null;
    const raw = coreCall((exports) => {
      const len = exports.bo_engine_take_charge!(this._handle);
      if (len < 0) throw new CoreLoadError(`bo_engine_take_charge failed (code ${len}): ${readCoreErrorMessage(exports)}`);
      return copyF64(exports, exports.bo_engine_charge_ptr!(this._handle) >>> 0, len);
    });
    const charge: Record<string, number> = {};
    const channelCharge: Record<string, number[]> = {};
    const finalUpTo: Record<string, number> = {};
    let slot = 2;
    const slots = raw.length - 2 - armed.ids.length;
    armed.ids.forEach((id, k) => {
      const n = armed.channels[k]!;
      charge[id] = raw[slot]!;
      if (n > 0) channelCharge[id] = Array.from(raw.subarray(slot + 1, slot + 1 + n));
      slot += 1 + n;
      finalUpTo[id] = raw[2 + slots + k]!;
    });
    return { t0: raw[0]!, t1: raw[1]!, charge, channelCharge, finalUpTo };
  }

  /**
   * A whole-engine snapshot (W2a; core/README.md, "Snapshots"): the Rust
   * engine's state, copied into its handle's table, plus this facade's own
   * clock, convergence flag and warnings ledger with its announce queue.
   * {@link restoreState} puts it back any number of times; a later `load()`
   * retires it. The stats are not part of it (telemetry, monotonic).
   *
   * The core sets no cap on snapshots, and WASM memory never shrinks, so
   * free one with {@link CoreSnapshot.dispose} when done; one that is
   * collected without a `dispose()` is freed by a `FinalizationRegistry`.
   * {@link snapshotBytes} reports what the table holds.
   */
  saveState(): CoreSnapshot {
    this._requireLoaded();
    const id = coreCall((exports) => {
      if (typeof exports.bo_engine_snapshot !== "function") {
        throw new CoreLoadError("CoreEngine.saveState: this brownout core module has no snapshot exports (built before W2a).");
      }
      const out = exports.bo_engine_snapshot(this._handle);
      if (out === 0) {
        throw new CoreLoadError(`bo_engine_snapshot failed: ${readCoreErrorMessage(exports)}`);
      }
      return out >>> 0;
    });
    const snapshot = new CoreSnapshot();
    snapshotState.set(snapshot, {
      engine: this,
      handle: this._handle,
      generation: this._generation,
      loadSerial: this._loadSerial,
      id,
      simTime: this._simTime,
      lastConverged: this._lastConverged,
      warnings: this._warnings.copy(),
      acquired: this._acquired,
      charged: this._charged,
      disposed: false,
    });
    coreSnapshotFinalizer.register(snapshot, { handle: this._handle, generation: this._generation, id }, snapshot);
    return snapshot;
  }

  /**
   * Puts `snapshot` back: every readout, the clock, the convergence flag and
   * the warnings are the snapshot's again, and the run from here is bit for
   * bit the one that followed {@link saveState}. Throws for a snapshot of
   * another engine, a disposed one, or one taken before the last `load()`.
   */
  restoreState(snapshot: CoreSnapshot): void {
    this._requireLoaded();
    const state = snapshotState.get(snapshot);
    if (!state || state.engine !== this) {
      throw new Error("CoreEngine.restoreState: this snapshot was taken on another engine.");
    }
    if (state.disposed) {
      throw new Error("CoreEngine.restoreState: this snapshot has been disposed.");
    }
    if (state.loadSerial !== this._loadSerial) {
      throw new Error("CoreEngine.restoreState: this snapshot was taken before the last load(), which retired it.");
    }
    coreCall((exports) => {
      const code = exports.bo_engine_restore!(this._handle, state.id);
      if (code !== 0) {
        throw new CoreLoadError(`bo_engine_restore failed (code ${code}): ${readCoreErrorMessage(exports)}`);
      }
    });
    this._simTime = state.simTime;
    this._lastConverged = state.lastConverged;
    this._warnings = state.warnings.copy();
    // The observer and the charge are engine state: the core's came back
    // with the snapshot, so their arming does too.
    this._acquired = state.acquired;
    this._charged = state.charged;
    this._publishReadouts();
  }

  /** The bytes this engine's snapshots hold in the core (`bo_engine_snapshot_bytes`): each copy, plus each shared memory image once. */
  snapshotBytes(): number {
    this._checkGeneration();
    return coreCall((exports) => (typeof exports.bo_engine_snapshot_bytes === "function" ? exports.bo_engine_snapshot_bytes(this._handle) : 0));
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

/**
 * An opaque whole-engine snapshot from {@link CoreEngine.saveState} (W2a).
 * Hand it back to the same engine's {@link CoreEngine.restoreState}, and
 * `dispose()` it when done: the core holds its copy until then.
 */
export class CoreSnapshot {
  /** Whether {@link dispose} has run. */
  get disposed(): boolean {
    return snapshotState.get(this)?.disposed ?? true;
  }

  /**
   * Frees the core's copy. A second call does nothing, and so does one after
   * the engine was disposed or reloaded (which already freed it). Never
   * throws.
   */
  dispose(): void {
    const state = snapshotState.get(this);
    if (!state || state.disposed) return;
    state.disposed = true;
    coreSnapshotFinalizer.unregister(this);
    dropSnapshot(state);
  }
}

interface SnapshotState {
  readonly engine: CoreEngine;
  readonly handle: number;
  readonly generation: number;
  readonly loadSerial: number;
  readonly id: number;
  readonly simTime: number;
  readonly lastConverged: boolean;
  readonly warnings: WarningLedger;
  readonly acquired: readonly string[] | null;
  readonly charged: { readonly ids: readonly string[]; readonly channels: readonly number[] } | null;
  disposed: boolean;
}

/** Each snapshot's state, kept off the object so a snapshot is opaque to its holder. */
const snapshotState = new WeakMap<CoreSnapshot, SnapshotState>();

interface SnapshotToken {
  handle: number;
  generation: number;
  id: number;
}

/**
 * Drops the core's copy, best effort: after a re-init the old instance (and
 * its table) is gone, and after a load or the engine's dispose the id is
 * already gone, which the core reports as `-8` or `-5` and this ignores.
 */
function dropSnapshot(token: SnapshotToken): void {
  try {
    if (coreGeneration() !== token.generation) return;
    coreCall((exports) => exports.bo_engine_snapshot_drop?.(token.handle, token.id));
  } catch {
    // Never throw from dispose() or a finalizer callback.
  }
}

/** Frees a snapshot collected without a `dispose()`: the core sets no cap, and WASM memory never shrinks. */
const coreSnapshotFinalizer = new FinalizationRegistry<SnapshotToken>(dropSnapshot);

/**
 * What a reload's carry reads of the circuit it replaces (`src/core/carry.ts`:
 * each component's id, kind, catalog part, params and pins), copied at the
 * load so a host's in-place edit before the reload does not show through.
 * Not `structuredClone`: the circuit's wires and the strings in its params
 * (a ROM image runs to 44 KB of base64) need no copy, and a whole clone cost
 * 1 ms of an 11 ms SAP-1 load, cold loads included.
 */
function carrySnapshot(circuit: SimCircuit): SimCircuit {
  return {
    ...circuit,
    components: circuit.components.map((c) => ({ ...c, params: { ...c.params }, pins: c.pins.map((p) => ({ ...p })) })),
    wires: [],
  };
}

/** W2b: one {@link CoreEngine.takeSamples}. */
export interface CoreSamples {
  /** The sample times, ascending. */
  readonly t: Float64Array;
  /** Net id -> its value at each of `t`. */
  readonly values: Readonly<Record<string, Float64Array>>;
  /** The widest step a stepped net's samples were interpolated across since the last take (0 when none). */
  readonly maxSpan: number;
  /** Whether the queue filled, and how many samples were dropped (the newest; the oldest are kept). */
  readonly overflowed: boolean;
  readonly dropped: number;
  /** Net id -> its samples read linear where R104's solve failed. */
  readonly fallbacks: Readonly<Record<string, number>>;
}

/** W2b piece 4: one {@link CoreEngine.takeCharge}. */
export interface CoreCharge {
  readonly t0: number;
  readonly t1: number;
  /** Component id -> coulombs of its published current over the span. */
  readonly charge: Readonly<Record<string, number>>;
  /** Component id -> each channel's charge, in channel order (LED family parts). */
  readonly channelCharge: Readonly<Record<string, number[]>>;
  /** Component id -> how far its charge is final (`t1` unless a yield left its stepped partition's step pending). */
  readonly finalUpTo: Readonly<Record<string, number>>;
}

/** The index of each id, refusing an unknown or repeated one. */
function indicesOf(ids: readonly string[], index: (id: string) => number | undefined, method: string, what: string): number[] {
  const seen = new Set<string>();
  return ids.map((id) => {
    const i = index(id);
    if (i === undefined) throw new RangeError(`CoreEngine.${method}: no ${what} "${id}" in the loaded circuit.`);
    if (seen.has(id)) throw new RangeError(`CoreEngine.${method}: ${what} "${id}" is listed twice.`);
    seen.add(id);
    return i;
  });
}

/** Runs `call` with `values` copied into a core buffer (0 for none), freed after. */
function withF64Buffer(exports: CoreExports, values: readonly number[], call: (ptr: number) => number): number {
  if (values.length === 0) return call(0);
  const ptr = exports.bo_alloc(values.length) >>> 0;
  try {
    new Float64Array(exports.memory.buffer, ptr, values.length).set(values);
    return call(ptr);
  } finally {
    exports.bo_free(ptr, values.length);
  }
}
