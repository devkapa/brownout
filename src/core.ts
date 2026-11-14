/**
 * `brownout/core`: the Rust/WASM partitioned event-driven transient engine
 * (core/README.md). Opt-in: it runs the kinds in `CORE_IMPLEMENTED_KINDS`
 * (passives and switches, fuses, the sources, LEDs and displays, the NE555,
 * the 74-series logic and memory parts listed there), and a circuit with any
 * other kind is declined with `CoreUnsupportedError` so a host can run it on
 * the classic engine instead. `CORE_SUPPORTED_KINDS` is the subset brownout's
 * own hosts route to the core without being told which kinds to trust
 * (`HeadlessRunner` on "auto" or "core", the analyses on "auto"): the kinds
 * that have cleared all three accuracy gates (see kinds.ts for the
 * implemented-vs-supported distinction). Nothing in this entry is re-exported from the main
 * `"brownout"` barrel (src/index.ts) — reach it explicitly:
 *
 *   import { initCore, CoreEngine } from "brownout/core";
 *
 * `initCore()` must be awaited — and must succeed — before constructing a
 * `CoreEngine`; every export below documents what happens if you don't.
 *
 * BUNDLER NOTES (wasm.ts has the full detail)
 * `initCore()`'s default source resolves the wasm asset with `new
 * URL("./brownout-core.wasm", import.meta.url)`, which webpack and vite both
 * detect statically in a client or worker bundle — those work with no setup.
 * A Next.js SERVER bundle rewrites that expression to something with no base
 * to resolve against in Node and throws a clear error naming the fix: mark
 * `"brownout"` as a server external (`serverExternalPackages: ["brownout"]`),
 * or call `initCore()` with the wasm bytes yourself. `"brownout/core.wasm"`
 * (this package's `exports` map) is the raw `.wasm` asset, for a host that
 * wants to fetch or bundle the bytes on its own terms instead.
 */

export { initCore, coreReady, coreStatus, CORE_ABI_VERSION } from "./core/wasm.js";
export type { CoreWasmSource } from "./core/wasm.js";

export { CoreEngine } from "./core/engine.js";
export type { CoreAdvanceFailure, CoreAdvanceOutcome, CoreCharge, CoreEngineStats, CoreParamChange, CoreSamples, CoreSnapshot } from "./core/engine.js";

export {
  CoreInitError,
  CoreLoadError,
  CoreNotInitializedError,
  CoreNotLoadedError,
  CorePoisonedError,
  CoreStaleInstanceError,
  CoreTrapError,
  CoreUnsupportedError,
} from "./core/errors.js";

export { CORE_IMPLEMENTED_KINDS, CORE_SUPPORTED_KINDS } from "./core/kinds.js";
export { coreAdvanceFailureReason, coreFailureReasons, unsupportedCoreKinds, unsupportedCoreKindReasons } from "./core/select.js";
export type { EngineChoice } from "./core/select.js";

export type { EngineReadout, TelemetryEngine } from "./core/surface.js";

export { liveParamDiff, VOLTAGE_SOURCE_V_LIVE } from "./core/edit.js";
export type { LiveParamDiffOptions } from "./core/edit.js";
export { coreSolverDiagnostics, readCoreStats, CORE_STAT_COUNT } from "./core/diagnostics.js";
export type { CoreBatchInfo, CoreStatSnapshot } from "./core/diagnostics.js";

// Useful to tooling (the equivalence harness, diagnostics) that wants to
// compile a circuit without driving a full CoreEngine.
export { compileCircuit } from "./core/compile.js";
export type { CompileResult, CompiledCircuit, CoreNodeBuild } from "./core/compile.js";
export { buildCoreNodes } from "./core/compile.js";
