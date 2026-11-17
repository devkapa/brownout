/**
 * Catalog-identity scaffolding for the v2 compiler. Where a kind's compiled
 * param depends on exact catalog identity (a part's `default_params`, the
 * way classic's private `modelParam` helper resolves one — sim-engine.ts's
 * `partFor`/`modelParam`), this module is where that lookup happens — and it
 * NEVER re-derives catalog matching/inference itself. It delegates entirely
 * to classic's own public resolver, `resolveCatalogPart`
 * (src/circuit/catalog-resolver.ts), the same function `partFor` calls.
 *
 * W1a only implements `resistor` and `voltage_source`, and neither reads a
 * catalog default (both resolve straight from `component.params`, mirroring
 * devices/passives.ts's `resistorModel` and devices/sources.ts's
 * `voltageSourceModel` exactly — see compile/index.ts). So today this file
 * is scaffolding: the machinery a later kind will call, proven correct by
 * compile.test.ts's delegation tests, with nothing in compile/index.ts
 * calling it yet.
 */

import { resolveCatalogPart } from "../../circuit/catalog-resolver.js";
import type { ComponentKind, ComponentParams, PartDefinition } from "../../circuit/types.js";

/** The minimal shape catalog resolution needs from a component — matches `SimCircuit["components"][number]`'s catalog-relevant fields. */
export interface CatalogComponentLike {
  kind: string;
  catalogUid?: string;
  params: ComponentParams;
}

/**
 * A component's exact catalog part, or `null` when identity does not
 * resolve (unknown kind, stale/kind-mismatched uid, or no catalog candidate
 * at all) — the same three-way outcome `partFor` gets from
 * `resolveCatalogPart`, since this function IS that call, with no cache and
 * no re-derived matching logic layered on top.
 */
export function resolveComponentPart(component: CatalogComponentLike): PartDefinition | null {
  return resolveCatalogPart({
    kind: component.kind as ComponentKind,
    params: component.params,
    catalogUid: component.catalogUid,
  });
}

/**
 * Mirrors classic's private `modelParam` (sim-engine.ts:1184-1194) exactly:
 * an authored param always wins; otherwise, when the component carries an
 * EXACT catalog identity (resolution succeeds AND the resolved part's own
 * uid matches the component's authored `catalogUid` — a stale or kind-
 * mismatched uid must not silently adopt a different part's defaults), that
 * part's `default_params[key]` is used; otherwise `fallback`.
 *
 * Identity resolution itself is `resolveComponentPart` above, never
 * re-implemented here.
 */
export function catalogParam(
  component: CatalogComponentLike,
  key: string,
  fallback: number | string,
): number | string {
  if (component.params[key] !== undefined) return component.params[key]!;
  if (!component.catalogUid) return fallback;
  const part = resolveComponentPart(component);
  if (!part || part.uid !== component.catalogUid) return fallback;
  return part.default_params?.[key] ?? fallback;
}
