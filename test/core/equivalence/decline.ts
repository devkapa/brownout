/**
 * Why the core would not take a circuit: the reasons `CoreEngine.load` carries
 * on a `CoreUnsupportedError` (`unsupported-kind:<kind>`, `topology:...`,
 * `malformed:...`, `solve-failed:...`, `livelock:...`), asked directly instead
 * of through a HeadlessRunner. Shared by the skeleton fixtures' report line
 * ("declined: <reason>") and the decline census (scripts/decline-census.ts).
 * `initCore()` must have resolved before the first call.
 */
import { CoreEngine } from "../../../src/core/engine.js";
import { CoreUnsupportedError } from "../../../src/core/errors.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";

export interface DeclineResult {
  /** True when the core loaded the circuit. */
  readonly accepted: boolean;
  /** Empty when accepted. Sorted, deduplicated. */
  readonly reasons: readonly string[];
}

/** Strips a trailing component or net id so equal causes group together: `topology:ic-pins:u1` -> `topology:ic-pins`. Only the prefixes known to carry an id are touched. */
export function reasonClass(reason: string): string {
  const parts = reason.split(":");
  if (parts[0] === "unsupported-kind") return reason;
  if (parts[0] === "topology" && parts[1] === "ic-pins") return "topology:ic-pins";
  if (parts[0] === "malformed") return `malformed:${parts.slice(2).join(":")}`;
  // "topology:voltage_source (component 1): both terminals ..." and similar carry an index in prose.
  return reason.replace(/\(component \d+\)/g, "(component N)").replace(/\bn\d+\b/g, "nN");
}

export function declineReasons(circuit: SimCircuit, implementedKinds?: ReadonlySet<string>): DeclineResult {
  const engine = new CoreEngine();
  try {
    if (implementedKinds) engine.load(circuit, implementedKinds);
    else engine.load(circuit);
    return { accepted: true, reasons: [] };
  } catch (error) {
    if (error instanceof CoreUnsupportedError) return { accepted: false, reasons: [...new Set(error.reasons)].sort() };
    return { accepted: false, reasons: [`error:${error instanceof Error ? error.message : String(error)}`] };
  } finally {
    engine.dispose();
  }
}
