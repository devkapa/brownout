/**
 * Engine-selection glue: turning "which kinds does this circuit use" into
 * either "the core can take it" or a list of human-readable fallback
 * reasons a host can log or surface. kinds.ts is deliberately ignorant of
 * SimCircuit (see its own header comment) — this file is where that
 * kind-support knowledge meets the classic domain type, for the one
 * consumer that actually needs to make a routing decision: a host adapter
 * like HeadlessRunner (src/host/headless.ts).
 */

import type { SimCircuit } from "../sim/engine/sim-engine.js";
import type { CoreAdvanceFailure } from "./engine.js";
import { CoreLoadError, CorePoisonedError, CoreStaleInstanceError, CoreTrapError, CoreUnsupportedError } from "./errors.js";
import { CORE_SUPPORTED_KINDS } from "./kinds.js";

/** Which engine is driving a host adapter's load()/run(). */
export type EngineChoice = "classic" | "core";

/**
 * Every kind `circuit` uses that is NOT in `supported`, sorted and
 * deduplicated. Empty means the whole circuit clears the kind-support gate —
 * compileCircuit's own malformed-parameter check and bo_engine_load's own
 * topology check can still reject it after this, which is why this function
 * alone is not "is this circuit safe to route to the core."
 */
export function unsupportedCoreKinds(
  circuit: SimCircuit,
  supported: ReadonlySet<string> = CORE_SUPPORTED_KINDS,
): string[] {
  const unsupported = new Set<string>();
  for (const component of circuit.components) {
    if (!supported.has(component.kind)) unsupported.add(component.kind);
  }
  return [...unsupported].sort();
}

/**
 * `unsupportedCoreKinds`'s result turned into the `unsupported-kind:<kind>`
 * reason strings a host's `engineFallbackReasons` publishes — the same
 * vocabulary CoreUnsupportedError and compileCircuit's own reasons use
 * (`unsupported-kind:`, `malformed:`, `topology:`), so a caller reading
 * fallback reasons sees one consistent format regardless of which stage
 * (kind check, compile, or the Rust load) produced the fallback.
 */
export function unsupportedCoreKindReasons(
  circuit: SimCircuit,
  supported: ReadonlySet<string> = CORE_SUPPORTED_KINDS,
): string[] {
  return unsupportedCoreKinds(circuit, supported).map((kind) => `unsupported-kind:${kind}`);
}

/**
 * The fallback reasons a thrown core error maps to, or null for an error that
 * is not the core's to answer for (a caller bug such as a RangeError, which
 * must surface).
 *
 * A decline (`CoreUnsupportedError`) is the circuit, and carries its own
 * reasons. A trap, a poisoned or stale instance and a load code for a
 * malformed program (-1/-2/-5/-8) are the CORE failing: none of them says
 * anything about the circuit, so a host on `auto` runs it on classic instead.
 * One vocabulary for every host: the analyses' planner and `HeadlessRunner`
 * both read it from here.
 */
export function coreFailureReasons(err: unknown): readonly string[] | null {
  if (err instanceof CoreUnsupportedError) return err.reasons;
  if (err instanceof CoreTrapError) return ["core-trap"];
  if (err instanceof CorePoisonedError) return ["core-poisoned"];
  if (err instanceof CoreStaleInstanceError) return ["core-stale-instance"];
  if (err instanceof CoreLoadError) return [`core-load-error:${err.message}`];
  return null;
}

/**
 * The fallback reason for an advance that did not converge
 * (`CoreEngine.lastConverged === false`, with `lastAdvanceFailure`): the
 * load-time words for the same two codes (`solve-failed` for -4, `livelock`
 * for -7), prefixed `mid-run:` so a reader can tell a circuit the core gave up
 * on part-way from one it never took. With no failure record (a core built
 * before the record existed) it is `mid-run:advance-failed`.
 */
export function coreAdvanceFailureReason(failure: CoreAdvanceFailure | null): string {
  if (failure === null) return "mid-run:advance-failed";
  const what = failure.code === -4 ? "solve-failed" : failure.code === -7 ? "livelock" : `code ${failure.code}`;
  return `mid-run:${what}:${failure.message}`;
}
