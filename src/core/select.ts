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
