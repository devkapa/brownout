/**
 * Injectable part library — the seam that keeps proprietary catalogs OUT of
 * the engine package.
 *
 * The de:volt application ships a closed catalog with product metadata (BOM,
 * palette copy, teaching text). The engine only needs model identity: uid,
 * kind, default_params, pin ids/functions, electrical_specs, spice_model.
 * Hosts therefore INJECT their catalog via setPartLibrary(); the bundled
 * default-parts.json is an open, stripped subset sufficient for the engine
 * corpus (family-correct logic thresholds, regulator/LED/battery identities,
 * thermal package profiles).
 *
 * Module state is deliberate: catalog identity resolution happens deep inside
 * stamp paths (sim-engine partFor, thermal profile lookup) where threading a
 * catalog handle through every call would churn the entire engine API for a
 * value that is constant per host process. The version counter lets those
 * paths keep cheap memoization while remaining injection-reactive: any cache
 * derived from the library must be keyed on getPartLibraryVersion() so a
 * late setPartLibrary() call still takes effect (hosts are free to inject
 * after modules were imported and even after engines were constructed —
 * already-committed engine state keeps its old resolution until the next
 * load()).
 */

// The attribute is required by Node's ESM loader for JSON modules; bundlers
// (Vite/esbuild/webpack 5) parse and honor it as well.
import rawDefaultParts from "./default-parts.json" with { type: "json" };
import type { PartDefinition } from "../circuit/types.js";

/**
 * The bundled open subset. Exposed so hosts can extend rather than replace:
 * setPartLibrary([...BUNDLED_DEFAULT_PARTS, ...customParts]).
 */
export const BUNDLED_DEFAULT_PARTS: readonly PartDefinition[] =
  (rawDefaultParts as unknown as { parts: PartDefinition[] }).parts;

let activeParts: readonly PartDefinition[] = BUNDLED_DEFAULT_PARTS;
// Starts at 1 so a consumer-side "0 = never derived" sentinel can never
// collide with a real library generation.
let libraryVersion = 1;

/**
 * Replace the whole active part library. Replacement (not merge) keeps the
 * semantics predictable: the injected array is the complete identity universe,
 * so a host can also REMOVE bundled parts it does not want resolvable.
 */
export function setPartLibrary(parts: PartDefinition[]): void {
  // Defensive copy: later mutation of the caller's array must not silently
  // change resolution results between two identical lookups.
  activeParts = Object.freeze([...parts]);
  libraryVersion += 1;
}

export function getPartLibrary(): readonly PartDefinition[] {
  return activeParts;
}

/**
 * Monotonic generation counter for the active library. Caches derived from
 * the library (catalog resolvers, thermal LED profiles, engine identity
 * caches) key on this instead of on array identity so they stay correct
 * across injections without holding the array alive.
 */
export function getPartLibraryVersion(): number {
  return libraryVersion;
}

/** Restore the bundled defaults; primarily test hygiene between suites. */
export function resetPartLibrary(): void {
  activeParts = BUNDLED_DEFAULT_PARTS;
  libraryVersion += 1;
}
