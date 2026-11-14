/**
 * Error classes the core front end throws. One file so wasm.ts, engine.ts,
 * select.ts and host/headless.ts all import the same names instead of each
 * hand-rolling their own — a caller catching `CoreUnsupportedError` doesn't
 * need to know which of those modules constructed it.
 *
 * Every class sets `.name` explicitly: minified/bundled builds can rename
 * the class itself, but `.name` is a plain string a caller can log or
 * pattern-match without relying on `instanceof` surviving a bundler.
 */

/** initCore() has not been awaited (or has not yet succeeded) — there is no live core instance to use. */
export class CoreNotInitializedError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "CoreNotInitializedError";
  }
}

/**
 * initCore() itself could not produce a usable instance: the wasm bytes
 * could not be located/fetched/instantiated, the instance's reported ABI
 * version does not match this package's CORE_ABI_VERSION, or the instance
 * is missing a contract export. Distinct from CoreNotInitializedError
 * ("nobody has awaited initCore() yet") and from CoreLoadError ("the
 * instance is fine, but THIS PROGRAM is not").
 *
 * Also thrown by a device-state read (`CoreEngine.getBatteryState`,
 * `getIcState`) whose returned length is not the layout this package reads
 * (core/README.md, `bo_engine_device_state`): the same TS/Rust mismatch as a
 * wrong ABI version, found at the first read instead of at init.
 */
export class CoreInitError extends Error {
  constructor(message: string, options?: ErrorOptions) {
    super(message, options);
    this.name = "CoreInitError";
  }
}

/**
 * One ABI call threw something that was not one of this package's own
 * Core* error classes — a real `WebAssembly.RuntimeError` trap
 * (panic=abort, core/README.md "Traps"), but also anything else the wasm
 * runtime can throw out from under a call in progress (for example a JS
 * `RangeError` from a wasm stack overflow: it is not a `RuntimeError`, but
 * the Rust state after an aborted call is exactly as untrustworthy). The
 * instance that produced it is now POISONED (see CorePoisonedError): every
 * later call throws until initCore() succeeds again and replaces it. The
 * original error is preserved as `cause`.
 */
export class CoreTrapError extends Error {
  constructor(message: string, options?: ErrorOptions) {
    super(message, options);
    this.name = "CoreTrapError";
  }
}

/** Thrown by every ABI call made after a CoreTrapError poisoned the instance, until initCore() re-instantiates. */
export class CorePoisonedError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "CorePoisonedError";
  }
}

/**
 * A CoreEngine's handle no longer refers to anything: initCore() was called
 * again (a fresh instance, a fresh handle table, a new instance generation)
 * since this CoreEngine was constructed. Its handle NUMBER could now
 * coincidentally name a real, unrelated engine in the new instance, so this
 * is refused outright rather than risked. Construct a new CoreEngine.
 */
export class CoreStaleInstanceError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "CoreStaleInstanceError";
  }
}

/**
 * A read was attempted on a CoreEngine with no confirmed current program:
 * either load() has never been called, or the most recent load() call
 * failed. A failed load() always clears any previously loaded program's
 * readouts — even though the Rust side keeps ITS OWN old program loaded on
 * a failed bo_engine_load (core/README.md's contract, and abi.rs's own
 * `a_failed_reload_leaves_the_previous_program_intact` test) — because
 * silently continuing to serve stale readouts after a load() the caller saw
 * fail would be far more surprising than an explicit throw here.
 */
export class CoreNotLoadedError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "CoreNotLoadedError";
  }
}

/**
 * A circuit could not be routed to the core: an unimplemented/unsupported
 * kind, a malformed parameter (compile.ts), or a topology the W0 kernel
 * cannot solve (bo_engine_load code -3 — see CoreLoadError for why -2 is
 * NOT grouped here). `reasons` is sorted and deduplicated, in the same
 * string vocabulary compile.ts and select.ts use (`unsupported-kind:<kind>`,
 * `malformed:<id>:<what>`, `topology:<message>`) so a caller can
 * pattern-match on the prefix without parsing prose.
 */
export class CoreUnsupportedError extends Error {
  readonly reasons: readonly string[];
  constructor(reasons: readonly string[]) {
    super(`circuit is not supported by the core: ${reasons.join(", ")}`);
    this.name = "CoreUnsupportedError";
    this.reasons = reasons;
  }
}

/**
 * `bo_engine_load`, `bo_engine_advance` or `bo_engine_set_params` failed for
 * a reason that is not "route this circuit elsewhere" — a malformed program,
 * an unknown kind code (-2: a TS/Rust kind-table drift bug, not a fact about
 * the user's circuit — see engine.ts's `mapLoadFailure`), a solve failure, a
 * bad handle, or (for `bo_engine_set_params`) an invalid argument the ABI
 * itself rejected (core/README.md error codes -1/-2/-4/-5/-8). Carries the
 * Rust side's own message verbatim, prefixed with what was being attempted.
 */
export class CoreLoadError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "CoreLoadError";
  }
}
