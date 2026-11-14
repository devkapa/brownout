/**
 * The "which kinds does this circuit use" half of the not-ported gate
 * (kinds.ts's own header: CORE_IMPLEMENTED_KINDS answers "can the compiler
 * even produce a program"). Kept tiny and dependency-free so both the local
 * fixture loaders (which only have a raw circuit) and run-entry.ts (which
 * only has a CorpusEntry's already-computed `kinds` list) share one
 * definition of "missing".
 */

import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";

export function kindsOf(circuit: { components: ReadonlyArray<{ kind: string }> }): string[] {
  return [...new Set(circuit.components.map((c) => c.kind))].sort();
}

export function missingCoreKinds(
  kinds: readonly string[],
  implemented: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS,
): string[] {
  return kinds.filter((kind) => !implemented.has(kind)).sort();
}
