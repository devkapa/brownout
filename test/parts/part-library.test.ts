/**
 * Contract tests for the injectable part-library seam.
 *
 * WHY this suite exists at all: part-library.ts has no devolt ancestor — it is
 * the module the extraction INVENTED to keep proprietary catalogs out of the
 * engine, so no inherited corpus covers it. It is also load-bearing well beyond
 * its own surface: catalog-resolver, thermal-physics, and sim-engine's partFor
 * each memoize identity resolution and key those caches on
 * getPartLibraryVersion(). A version counter that fails to advance would not
 * throw anywhere — it would silently resolve a late-injecting host against the
 * bundled defaults forever. These tests pin the counter and every cache that
 * depends on it.
 */

import { afterEach, describe, expect, it } from "vitest";
import {
  BUNDLED_DEFAULT_PARTS,
  getPartLibrary,
  getPartLibraryVersion,
  resetPartLibrary,
  setPartLibrary,
} from "../../src/parts/part-library.js";
import { catalogResolver, resolveCatalogPart } from "../../src/circuit/catalog-resolver.js";
import { thermalProfilesByCatalogUid } from "../../src/sim/thermal-physics.js";
import { SimEngine, type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import type { PartDefinition } from "../../src/circuit/types.js";

// Module state is process-global by design; every case restores the bundled
// defaults so ordering between cases (and suites sharing this worker) cannot
// change results.
afterEach(() => {
  resetPartLibrary();
});

/** Minimal engine-identity-shaped part: exactly what the seam promises hosts. */
function customResistor(uid: string, resistance: number): PartDefinition {
  return {
    uid,
    kind: "resistor",
    default_params: { resistance },
    pin_layout: [
      { id: "a", offset: { x: 0, y: 0 }, function: "passive" },
      { id: "b", offset: { x: 0, y: 0 }, function: "passive" },
    ],
  } as unknown as PartDefinition;
}

/**
 * A capacitor identity whose only distinguishing default is leakage. Chosen for
 * the engine-cache case because leakageResistance reaches the stamp through
 * modelParam() — the one path that reads catalog default_params — and at DC the
 * leakage shunt is the only conducting element, so the resolved identity is
 * directly observable as a current.
 */
function customCapacitor(uid: string, leakageResistance: number): PartDefinition {
  return {
    uid,
    kind: "capacitor",
    default_params: { capacitance: 1e-6, leakageResistance },
    pin_layout: [
      { id: "a", offset: { x: 0, y: 0 }, function: "passive" },
      { id: "b", offset: { x: 0, y: 0 }, function: "passive" },
    ],
  } as unknown as PartDefinition;
}

describe("part library — bundled defaults", () => {
  it("starts out serving the bundled default subset", () => {
    expect(getPartLibrary()).toBe(BUNDLED_DEFAULT_PARTS);
    expect(BUNDLED_DEFAULT_PARTS.length).toBeGreaterThan(0);
  });

  it("bundled entries carry the model identity the engine actually reads", () => {
    for (const part of BUNDLED_DEFAULT_PARTS) {
      expect(typeof part.uid).toBe("string");
      expect(typeof part.kind).toBe("string");
      expect(part.default_params).toBeTruthy();
      expect(Array.isArray(part.pin_layout)).toBe(true);
    }
  });

  it("bundled uids are unique — duplicate identity would make resolution order-dependent", () => {
    const uids = BUNDLED_DEFAULT_PARTS.map((part) => part.uid);
    expect(new Set(uids).size).toBe(uids.length);
  });
});

describe("part library — injection", () => {
  it("replaces rather than merges, so a host can remove bundled identities", () => {
    setPartLibrary([customResistor("only-part", 123)]);

    expect(getPartLibrary().map((part) => part.uid)).toEqual(["only-part"]);
    // A bundled uid must stop resolving once the host owns the universe.
    expect(catalogResolver.byUid("led-red")).toBeNull();
  });

  it("supports the documented extend idiom", () => {
    setPartLibrary([...BUNDLED_DEFAULT_PARTS, customResistor("extra-part", 999)]);

    expect(catalogResolver.byUid("extra-part")?.uid).toBe("extra-part");
    expect(catalogResolver.byUid("led-red")?.uid).toBe("led-red");
  });

  it("defensively copies the injected array", () => {
    const parts = [customResistor("kept", 1)];
    setPartLibrary(parts);

    parts.push(customResistor("smuggled-in-after-injection", 2));

    expect(getPartLibrary().map((part) => part.uid)).toEqual(["kept"]);
    expect(catalogResolver.byUid("smuggled-in-after-injection")).toBeNull();
  });

  it("restores the bundled defaults on reset", () => {
    setPartLibrary([customResistor("temporary", 1)]);
    resetPartLibrary();

    expect(getPartLibrary()).toBe(BUNDLED_DEFAULT_PARTS);
    expect(catalogResolver.byUid("temporary")).toBeNull();
  });
});

describe("part library — version counter", () => {
  it("never reports the 0 sentinel a consumer may use for never-derived", () => {
    expect(getPartLibraryVersion()).toBeGreaterThan(0);
  });

  it("advances on every injection", () => {
    const before = getPartLibraryVersion();
    setPartLibrary([customResistor("v1", 1)]);
    const afterFirst = getPartLibraryVersion();
    setPartLibrary([customResistor("v2", 2)]);

    expect(afterFirst).toBeGreaterThan(before);
    expect(getPartLibraryVersion()).toBeGreaterThan(afterFirst);
  });

  it("advances on reset — caches derived from an injected library must drop too", () => {
    setPartLibrary([customResistor("v1", 1)]);
    const injected = getPartLibraryVersion();
    resetPartLibrary();

    expect(getPartLibraryVersion()).toBeGreaterThan(injected);
  });
});

describe("part library — dependent caches are injection-reactive", () => {
  it("invalidates the catalog resolver", () => {
    // Prime the resolver cache against the bundled defaults first: an injection
    // that only works before first use would pass a naive test.
    expect(resolveCatalogPart({ kind: "led", catalogUid: "led-red" })?.uid).toBe("led-red");

    setPartLibrary([customResistor("post-injection", 470)]);

    expect(resolveCatalogPart({ kind: "led", catalogUid: "led-red" })).toBeNull();
    expect(resolveCatalogPart({ kind: "resistor", catalogUid: "post-injection" })?.uid)
      .toBe("post-injection");
  });

  it("invalidates the thermal exact-profile map", () => {
    // LED profiles are derived per catalog uid from the ACTIVE library.
    expect(thermalProfilesByCatalogUid()["led-red"]).toBeDefined();

    setPartLibrary([
      { ...(catalogResolver.byUid("led-red") as PartDefinition), uid: "led-custom" },
    ]);

    const profiles = thermalProfilesByCatalogUid();
    expect(profiles["led-custom"]).toBeDefined();
    expect(profiles["led-red"]).toBeUndefined();
  });

  it("invalidates sim-engine's explicit-uid identity cache across a load()", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "c1",
          kind: "capacitor",
          catalogUid: "swappable",
          pins: [{ id: "a" }, { id: "b" }],
          // Deliberately no params: the leakage default must come from the
          // resolved catalog identity, not from the instance.
          params: {},
        },
        {
          id: "v1",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 10 },
        },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "c1", to_pin: "a" },
        { from_component: "c1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };

    // 10 V across a 100 ohm leakage shunt -> 100 mA.
    setPartLibrary([customCapacitor("swappable", 100)]);
    const first = new SimEngine();
    first.load(circuit);
    first.dcOperatingPoint();
    const firstCurrent = Math.abs(first.getElementI()["v1"] ?? 0);

    // Same uid, different default. partFor() memoizes by kind+uid, so only the
    // version-keyed clear makes this second engine see the new leakage.
    setPartLibrary([customCapacitor("swappable", 1000)]);
    const second = new SimEngine();
    second.load(circuit);
    second.dcOperatingPoint();
    const secondCurrent = Math.abs(second.getElementI()["v1"] ?? 0);

    expect(firstCurrent).toBeCloseTo(0.1, 6);
    expect(secondCurrent).toBeCloseTo(0.01, 6);
  });
});
