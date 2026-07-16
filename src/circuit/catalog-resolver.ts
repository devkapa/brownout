import { getPartLibrary, getPartLibraryVersion } from "../parts/part-library.js";
import type {
  CircuitComponent,
  ComponentKind,
  ComponentParams,
  PartCatalog,
  PartDefinition,
} from "./types.js";

/**
 * Stable fallbacks for legacy instances whose params do not uniquely identify
 * one of several catalog variants. These values are deliberately explicit —
 * catalog ordering must never silently change persisted model identity.
 */
export const CANONICAL_CATALOG_UID_BY_KIND: Partial<Record<ComponentKind, string>> = {
  breadboard: "breadboard-830",
  buzzer: "buzzer-active-5v",
  capacitor: "capacitor",
  led: "led-red",
  linear_reg: "reg-7805",
};

export type CatalogResolutionSource =
  | "explicit"
  | "only-candidate"
  | "legacy-params"
  | "canonical-fallback"
  | "unresolved";

export interface CatalogIdentityInput {
  kind: ComponentKind;
  params?: ComponentParams;
  catalogUid?: string;
}

export interface CatalogPartResolution {
  part: PartDefinition | null;
  catalogUid: string | null;
  source: CatalogResolutionSource;
}

export interface CatalogResolver {
  resolve(input: CatalogIdentityInput): CatalogPartResolution;
  byUid(uid: string): PartDefinition | null;
  candidates(kind: ComponentKind): readonly PartDefinition[];
}

function sameParamValue(a: number | string | undefined, b: number | string | undefined): boolean {
  return typeof a === typeof b && a === b;
}

/** Keys whose defaults actually distinguish candidates of the same kind. */
function discriminatingKeys(candidates: readonly PartDefinition[]): string[] {
  const keys = new Set<string>();
  for (const candidate of candidates) {
    for (const key of Object.keys(candidate.default_params ?? {})) keys.add(key);
  }
  return [...keys].filter((key) => {
    const values = new Set(
      candidates.map((candidate) => {
        const value = candidate.default_params?.[key];
        return `${typeof value}:${String(value)}`;
      }),
    );
    return values.size > 1;
  });
}

function uniquelyInferredPart(
  candidates: readonly PartDefinition[],
  params: ComponentParams,
): PartDefinition | null {
  const keys = discriminatingKeys(candidates).filter((key) => key in params);
  if (keys.length === 0) return null;

  // Identity inference is safety-critical: one contradictory discriminator
  // must veto a candidate instead of being out-voted by two coincidental
  // matches. Persisting a best-effort guess can silently select the wrong
  // chemistry, package thermal model, or BOM item on every later load.
  const exactMatches = candidates.filter((part) => keys.every((key) =>
    sameParamValue(params[key], part.default_params?.[key]),
  ));
  return exactMatches.length === 1 ? exactMatches[0] : null;
}

/**
 * The original 4×AA catalog entry authored only `{ voltage: 5 }`. The exact
 * chemistry profile now authors a 6 V nominal pack plus resistance/capacity,
 * so generic current-default matching cannot recognise that historical shape.
 * Keep the alias deliberately narrow: only the old voltage and optional SoC
 * fields are accepted. Explicit UIDs are resolved before this policy, and any
 * package-identifying resistance/capacity fields bypass it.
 */
function legacyBatteryPackUid(params: ComponentParams): string | null {
  if (params.voltage !== 5) return null;
  const keys = Object.keys(params);
  return keys.every((key) => key === "voltage" || key === "charge")
    ? "supply-5v"
    : null;
}

/**
 * Old breadboards often persisted `points` without the newer `rails` flag (or
 * vice versa). Preserve the dimension that is known and choose the railed,
 * 830-point form only for the missing dimension. This is an explicit legacy
 * policy, not catalog-order inference.
 */
function legacyBreadboardFallbackUid(params: ComponentParams): string | null {
  const hasPoints = params.points === 400 || params.points === 830;
  const hasRails = params.rails === 0 || params.rails === 1;
  if (!hasPoints && !hasRails) return null;

  const points = hasPoints ? Number(params.points) : 830;
  const rails = hasRails ? Number(params.rails) : 1;
  return `breadboard-${points}${rails === 0 ? "-railless" : ""}`;
}

export function createCatalogResolver(catalog: PartCatalog): CatalogResolver {
  const byUid = new Map<string, PartDefinition>();
  const byKind = new Map<ComponentKind, PartDefinition[]>();
  for (const part of catalog.parts) {
    byUid.set(part.uid, part);
    const existing = byKind.get(part.kind);
    if (existing) existing.push(part);
    else byKind.set(part.kind, [part]);
  }

  return {
    byUid: (uid) => byUid.get(uid) ?? null,
    candidates: (kind) => byKind.get(kind) ?? [],
    resolve: (input) => {
      if (input.catalogUid) {
        const explicit = byUid.get(input.catalogUid) ?? null;
        // Explicit identity is authoritative. Preserve unresolved/stale UIDs
        // instead of silently changing a saved physical model.
        if (!explicit || explicit.kind !== input.kind) {
          return { part: null, catalogUid: input.catalogUid, source: "unresolved" };
        }
        return { part: explicit, catalogUid: explicit.uid, source: "explicit" };
      }

      const candidates = byKind.get(input.kind) ?? [];
      if (candidates.length === 0) {
        return { part: null, catalogUid: null, source: "unresolved" };
      }
      if (candidates.length === 1) {
        return { part: candidates[0], catalogUid: candidates[0].uid, source: "only-candidate" };
      }

      if (input.kind === "battery_pack") {
        const legacyUid = legacyBatteryPackUid(input.params ?? {});
        const legacy = legacyUid ? byUid.get(legacyUid) ?? null : null;
        if (legacy?.kind === input.kind) {
          return { part: legacy, catalogUid: legacy.uid, source: "legacy-params" };
        }
      }

      const inferred = uniquelyInferredPart(candidates, input.params ?? {});
      if (inferred) {
        return { part: inferred, catalogUid: inferred.uid, source: "legacy-params" };
      }

      if (input.kind === "breadboard") {
        const legacyUid = legacyBreadboardFallbackUid(input.params ?? {});
        const legacy = legacyUid ? byUid.get(legacyUid) ?? null : null;
        if (legacy?.kind === input.kind) {
          return { part: legacy, catalogUid: legacy.uid, source: "canonical-fallback" };
        }
      }

      const canonicalUid = CANONICAL_CATALOG_UID_BY_KIND[input.kind];
      const canonical = canonicalUid ? byUid.get(canonicalUid) ?? null : null;
      if (canonical?.kind === input.kind) {
        return { part: canonical, catalogUid: canonical.uid, source: "canonical-fallback" };
      }
      return { part: null, catalogUid: null, source: "unresolved" };
    },
  };
}

// The resolver indexes (byUid/byKind maps) are rebuilt only when the injected
// part library actually changes; the version key makes a late
// setPartLibrary() call visible without forcing hosts to re-import anything.
let cachedResolver: { resolver: CatalogResolver; version: number } | null = null;

export function activeCatalogResolver(): CatalogResolver {
  const version = getPartLibraryVersion();
  if (!cachedResolver || cachedResolver.version !== version) {
    cachedResolver = {
      resolver: createCatalogResolver({ version, parts: [...getPartLibrary()] }),
      version,
    };
  }
  return cachedResolver.resolver;
}

/**
 * Injection-reactive facade with the historical constant-object API. Each
 * call delegates to the resolver for the CURRENT library, so holding this
 * object across a setPartLibrary() boundary stays correct.
 */
export const catalogResolver: CatalogResolver = {
  resolve: (input) => activeCatalogResolver().resolve(input),
  byUid: (uid) => activeCatalogResolver().byUid(uid),
  candidates: (kind) => activeCatalogResolver().candidates(kind),
};

export function resolveCatalogPart(input: CatalogIdentityInput): PartDefinition | null {
  return catalogResolver.resolve(input).part;
}

/** Populate identity on a legacy component without changing explicit identity. */
export function withResolvedCatalogIdentity(component: CircuitComponent): CircuitComponent {
  if (component.catalogUid) return component;
  const resolution = catalogResolver.resolve(component);
  return resolution.catalogUid
    ? { ...component, catalogUid: resolution.catalogUid }
    : component;
}
