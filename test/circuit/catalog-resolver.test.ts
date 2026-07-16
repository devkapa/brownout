import { afterAll, beforeAll, describe, expect, it } from "vitest";
import rawCatalog from "../helpers/catalog.js";
import type { CircuitComponent, ComponentKind, ComponentParams, PartCatalog } from "../../src/circuit/types.js";
import {
  CANONICAL_CATALOG_UID_BY_KIND,
  catalogResolver,
  withResolvedCatalogIdentity,
} from "../../src/circuit/catalog-resolver.js";
import { resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";

// The corpus was authored against the full de:volt catalog, which the resolver
// read ambiently. Brownout resolves against the INJECTED library instead, and
// the bundled default subset deliberately omits parts these cases depend on
// (the breadboards, which drive the legacy points/rails fallback policy). Inject
// the full fixture so the suite exercises the same identity universe it was
// written for, and restore the bundled defaults afterwards so library state does
// not leak into any suite sharing this worker.
beforeAll(() => {
  setPartLibrary((rawCatalog as unknown as PartCatalog).parts);
});

afterAll(() => {
  resetPartLibrary();
});

function resolve(kind: ComponentKind, params: ComponentParams = {}, catalogUid?: string) {
  return catalogResolver.resolve({ kind, params, catalogUid });
}

function component(
  kind: ComponentKind,
  params: ComponentParams = {},
  catalogUid?: string,
): CircuitComponent {
  return {
    id: `${kind}-1`,
    kind,
    ...(catalogUid ? { catalogUid } : {}),
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [],
    params,
  };
}

describe("catalogResolver", () => {
  it("keeps every documented canonical fallback valid and kind-correct", () => {
    const catalog = rawCatalog as unknown as PartCatalog;
    const byUid = new Map(catalog.parts.map((part) => [part.uid, part]));

    for (const [kind, uid] of Object.entries(CANONICAL_CATALOG_UID_BY_KIND)) {
      expect(byUid.get(uid)?.kind, `${kind} -> ${uid}`).toBe(kind);
    }
  });

  it("treats a valid explicit UID as authoritative even after editable params change", () => {
    expect(resolve("linear_reg", { vout: 4.8, vdropout: 0.9 }, "reg-ams1117-50")).toMatchObject({
      catalogUid: "reg-ams1117-50",
      source: "explicit",
      part: { uid: "reg-ams1117-50", kind: "linear_reg" },
    });
  });

  it("never silently reinterprets an unknown or kind-mismatched explicit UID", () => {
    expect(resolve("linear_reg", {}, "retired-regulator")).toEqual({
      part: null,
      catalogUid: "retired-regulator",
      source: "unresolved",
    });
    expect(resolve("linear_reg", {}, "led-red")).toEqual({
      part: null,
      catalogUid: "led-red",
      source: "unresolved",
    });
    expect(resolve("battery_pack", { voltage: 5 }, "retired-aa-pack")).toEqual({
      part: null,
      catalogUid: "retired-aa-pack",
      source: "unresolved",
    });
  });

  it("recovers the historical 5 V AA-pack signature without overriding explicit identity", () => {
    expect(resolve("battery_pack", { voltage: 5, charge: 0.4 })).toMatchObject({
      catalogUid: "supply-5v",
      source: "legacy-params",
    });
    expect(resolve("battery_pack", { voltage: 5 }, "battery-9v")).toMatchObject({
      catalogUid: "battery-9v",
      source: "explicit",
    });
  });

  it("keeps an unidentified custom battery generic instead of guessing AA chemistry", () => {
    expect(resolve("battery_pack", {
      voltage: 12,
      rInternal: 0.08,
      capacityAh: 7,
      charge: 0.75,
    })).toEqual({
      part: null,
      catalogUid: null,
      source: "unresolved",
    });
    const custom = component("battery_pack", {
      voltage: 12,
      rInternal: 0.08,
      capacityAh: 7,
      charge: 0.75,
    });
    expect(withResolvedCatalogIdentity(custom)).toBe(custom);
  });

  it("rejects contradictory discriminators instead of persisting a best-score guess", () => {
    expect(resolve("battery_pack", {
      voltage: 9,
      rInternal: 10,
      capacityAh: 0.235,
      charge: 1,
    })).toEqual({
      part: null,
      catalogUid: null,
      source: "unresolved",
    });
  });

  it("resolves kinds with one catalog candidate without guessing from params", () => {
    expect(resolve("resistor", { resistance: 47_000 })).toMatchObject({
      catalogUid: "resistor",
      source: "only-candidate",
    });
  });

  it.each([
    ["5 V supply", "battery_pack", { voltage: 5 }, "supply-5v"],
    ["9 V battery", "battery_pack", { voltage: 9, rInternal: 1.5, charge: 1 }, "battery-9v"],
    ["CR2032", "battery_pack", { voltage: 3, rInternal: 10, charge: 1 }, "battery-coin-cr2032"],
    ["generic capacitor", "capacitor", { capacitance: 1e-6, style: "electrolytic" }, "capacitor"],
    ["ceramic capacitor", "capacitor", { capacitance: 1e-7, style: "ceramic" }, "cap-ceramic"],
    ["electrolytic capacitor", "capacitor", { capacitance: 1e-4, style: "electrolytic" }, "cap-electrolytic"],
    ["film capacitor", "capacitor", { capacitance: 1e-6, style: "film" }, "cap-film"],
    ["7805", "linear_reg", { vout: 5, vdropout: 2, iLimit: 1 }, "reg-7805"],
    ["AMS1117-3.3", "linear_reg", { vout: 3.3, vdropout: 1.1, iLimit: 0.8 }, "reg-ams1117-33"],
    ["AMS1117-5.0", "linear_reg", { vout: 5, vdropout: 1.1, iLimit: 0.8 }, "reg-ams1117-50"],
    ["blue LED", "led", { color: "blue", vf: 3 }, "led-blue"],
    ["passive buzzer", "buzzer", { type: "passive", resistance: 32 }, "buzzer-passive-5v"],
  ] as const)("infers a legacy %s from discriminating params", (_name, kind, params, uid) => {
    expect(resolve(kind, params)).toMatchObject({ catalogUid: uid, source: "legacy-params" });
  });

  it("uses a documented canonical fallback when legacy params are ambiguous", () => {
    expect(resolve("linear_reg", { vout: 5 })).toMatchObject({
      catalogUid: "reg-7805",
      source: "canonical-fallback",
    });
    expect(resolve("led", {})).toMatchObject({
      catalogUid: "led-red",
      source: "canonical-fallback",
    });
    expect(resolve("breadboard", { points: 400 })).toMatchObject({
      catalogUid: "breadboard-400",
      source: "canonical-fallback",
    });
    expect(resolve("breadboard", { rails: 0 })).toMatchObject({
      catalogUid: "breadboard-830-railless",
      source: "canonical-fallback",
    });
  });

  it("enriches missing identity while leaving explicit identity untouched", () => {
    const legacy = component("capacitor", { capacitance: 1e-6, style: "film" });
    expect(withResolvedCatalogIdentity(legacy)).toEqual({ ...legacy, catalogUid: "cap-film" });

    const explicit = component("linear_reg", { vout: 3.3 }, "reg-ams1117-50");
    expect(withResolvedCatalogIdentity(explicit)).toBe(explicit);
  });
});
