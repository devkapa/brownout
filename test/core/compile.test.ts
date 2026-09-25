import { afterAll, describe, expect, it } from "vitest";
import fs from "node:fs";
import path from "node:path";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import { buildCoreNodes, compileCircuit, SlotAllocator } from "../../src/core/compile.js";
import * as compileIndex from "../../src/core/compile/index.js";
import {
  buildPinRecord,
  computeOpenPinKeys,
  corePinKey,
  deadBandHashPrefix,
  finishFloatingHash,
  fnv1a32,
  isOpenHighPin,
  isOpenPin,
  openPinHashPrefix,
} from "../../src/core/compile/topology.js";
import { catalogParam, resolveComponentPart } from "../../src/core/compile/catalog.js";
import { resolveCatalogPart } from "../../src/circuit/catalog-resolver.js";
import { getPartLibrary, resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";
import { CORE_KIND_CODE, CORE_OPTION_KEYS, decodeProgram } from "../../src/core/format.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import type { PartDefinition } from "../../src/circuit/types.js";

// ---- small circuit-building helpers, local to this file (this worker owns
// no shared test-helper files) --------------------------------------------

type SimComponent = SimCircuit["components"][number];

function part(
  id: string,
  kind: string,
  pinIds: string[],
  params: Record<string, number | string> = {},
): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function circuit(components: SimComponent[], wires: SimCircuit["wires"] = [], environment?: SimCircuit["environment"]): SimCircuit {
  return { components, wires, ...(environment ? { environment } : {}) };
}

/** `compiled.nets`/`buildCoreNodes(...).nets` must be produced by calling buildNets — verify via a fresh classic-engine load. */
function classicNets(c: SimCircuit): unknown {
  const engine = new SimEngine();
  engine.load(c);
  return engine.nets;
}

/** Reaches classic's private `_isOpenPin` through a test-only cast (core/README.md's own testing convention — never copied or re-derived, just called). */
function classicIsOpenPin(engine: SimEngine, componentId: string, pinId: string): boolean {
  return (engine as unknown as { _isOpenPin(compId: string, pinId: string): boolean })._isOpenPin(componentId, pinId);
}

// ---- node numbering -------------------------------------------------------

describe("buildCoreNodes — node numbering", () => {
  it("assigns node 0 to the gnd net when buildNets produces one", () => {
    // vsrc.pos -> r1.a, r1.b -> vsrc.neg. vsrc is a SOURCE_KIND, so its "neg"
    // pin's net becomes buildNets' "gnd" net.
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 9 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );

    const built = buildCoreNodes(c);

    expect(built.nodeOfNet.get("gnd")).toBe(0);
    expect(built.netIdOfNode[0]).toBe("gnd");
    // Exactly one other net ("n0" — the vsrc.pos/r1.a net) gets node 1.
    expect(built.nodeCount).toBe(2);
    expect(built.netIdOfNode).toEqual(["gnd", "n0"]);
    expect(built.nodeOfNet.get("n0")).toBe(1);

    expect(built.pinNode.get("vsrc\u0000pos")).toBe(1);
    expect(built.pinNode.get("r1\u0000a")).toBe(1);
    expect(built.pinNode.get("r1\u0000b")).toBe(0);
    expect(built.pinNode.get("vsrc\u0000neg")).toBe(0);

    expect(built.nets).toEqual(classicNets(c));
  });

  it("uses a synthetic, pinless datum at node 0 when there is no gnd net, and gives every floating pin its own node", () => {
    // No source, no board ground reference anywhere -> buildNets names no
    // net "gnd" at all. r1.a and r2.b are left unwired (floating), so each
    // becomes its own single-pin net.
    const c = circuit(
      [part("r1", "resistor", ["a", "b"], { resistance: 10 }), part("r2", "resistor", ["a", "b"], { resistance: 20 })],
      [wire("r1", "b", "r2", "a")],
    );

    const built = buildCoreNodes(c);

    expect(built.netIdOfNode[0]).toBeNull();
    expect(built.nodeOfNet.has("gnd")).toBe(false);
    // 3 nets total (the wired r1.b/r2.a net, plus two floating singletons) +
    // 1 synthetic datum = nodeCount 4.
    expect(built.nodeCount).toBe(4);
    expect(built.netIdOfNode).toHaveLength(4);
    expect(new Set(built.netIdOfNode.slice(1))).toEqual(new Set(["n0", "n1", "n2"]));

    // Every pin resolves, including the two floating ones, and a floating
    // pin's node maps back to a genuine single-pin net.
    const r1a = built.pinNode.get("r1\u0000a")!;
    const r1b = built.pinNode.get("r1\u0000b")!;
    const r2a = built.pinNode.get("r2\u0000a")!;
    const r2b = built.pinNode.get("r2\u0000b")!;
    expect([r1a, r1b, r2a, r2b].every((n) => typeof n === "number")).toBe(true);
    expect(r1b).toBe(r2a); // wired together
    expect(r1a).not.toBe(r1b); // r1.a is floating, on its own net
    expect(r2b).not.toBe(r1b); // r2.b is floating, on its own net
    expect(r1a).not.toBe(r2b);

    const netOf = (node: number): unknown[] | undefined =>
      built.nets.find((n) => built.nodeOfNet.get(n.id) === node)?.pins;
    expect(netOf(r1a)).toEqual([["r1", "a"]]);
    expect(netOf(r2b)).toEqual([["r2", "b"]]);

    expect(built.nets).toEqual(classicNets(c));
  });
});

// ---- nets vs. the classic engine, for every fixture and synthetic circuit -

const FIXTURES_DIR = path.resolve(__dirname, "../fixtures/circuits");

function loadFixture(name: string): SimCircuit {
  return JSON.parse(fs.readFileSync(path.join(FIXTURES_DIR, name), "utf8")) as SimCircuit;
}

const fixtureNames = fs.readdirSync(FIXTURES_DIR).filter((name) => name.endsWith(".sim.json"));

describe("net building matches the classic engine exactly", () => {
  it("found at least one fixture to check", () => {
    // A guard against this suite silently checking nothing if the fixtures
    // directory ever moves or empties out.
    expect(fixtureNames.length).toBeGreaterThan(0);
  });

  for (const name of fixtureNames) {
    it(`buildCoreNodes(...).nets === classic nets for ${name} (kinds need not be implemented)`, () => {
      const c = loadFixture(name);
      // Every real fixture mixes in kinds the core does not implement yet
      // (74-series logic, sensors, ...), so compileCircuit would return
      // `ok: false` here — buildCoreNodes is exactly the seam that lets net
      // building be checked independently of kind support.
      expect(buildCoreNodes(c).nets).toEqual(classicNets(c));
    });
  }

  it("compiled.nets === classic nets for a fully-implemented synthetic circuit (with a gnd net)", () => {
    const c = circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 12 }),
        part("gref", "ground", ["gnd"]),
        part("r1", "resistor", ["a", "b"], { resistance: 470 }),
      ],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg"), wire("vsrc", "neg", "gref", "gnd")],
    );
    const result = compileCircuit(c);
    expect(result.ok).toBe(true);
    if (result.ok) expect(result.compiled.nets).toEqual(classicNets(c));
  });

  it("compiled.nets === classic nets for a fully-implemented synthetic circuit (no gnd net)", () => {
    const c = circuit(
      [part("r1", "resistor", ["a", "b"], { resistance: 1 }), part("r2", "resistor", ["a", "b"], { resistance: 2 })],
      [wire("r1", "b", "r2", "a")],
    );
    const result = compileCircuit(c);
    expect(result.ok).toBe(true);
    if (result.ok) expect(result.compiled.nets).toEqual(classicNets(c));
  });
});

// ---- open-pin flags vs. classic's _isOpenPin (spec §1.6, G8) ---------------

/**
 * Checks every declared pin of every component in `c` against classic's real
 * `_isOpenPin`. Throws (fails the test) immediately on the first mismatch,
 * naming it. Returns the number of pins checked — the caller decides whether
 * a single circuit's own count must be positive (every fixture/synthetic
 * test below) or whether it is enough for a whole CORPUS's total to be
 * (some corpus entries are legitimately empty, e.g. a Learn "starter"
 * template with nothing but a removed breadboard).
 */
function assertOpenFlagsMatchClassic(c: SimCircuit, label: string): number {
  const engine = new SimEngine();
  engine.load(c);
  const built = buildCoreNodes(c);
  const openKeys = computeOpenPinKeys(c, built.nets);

  let checked = 0;
  for (const component of c.components) {
    for (const p of component.pins) {
      const ours = isOpenPin(openKeys, built.pinNode, component.id, p.id);
      const classic = classicIsOpenPin(engine, component.id, p.id);
      if (ours !== classic) {
        throw new Error(
          `${label}: open-flag mismatch for ${component.id}.${p.id} (kind ${component.kind}): ours=${ours}, classic=${classic}`,
        );
      }
      checked++;
    }
  }
  return checked;
}

describe("computeOpenPinKeys / isOpenPin — matches classic's _isOpenPin exactly", () => {
  it("on a synthetic circuit with a floating pin", () => {
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("rFloat", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "rFloat", "a")],
    );
    expect(assertOpenFlagsMatchClassic(c, "synthetic floating-pin circuit")).toBeGreaterThan(0);
  });

  it("on a synthetic circuit with a wire between two pins of the SAME component (not open, G8)", () => {
    const c = circuit(
      [part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("r1", "a", "r1", "b")],
    );
    const built = buildCoreNodes(c);
    const openKeys = computeOpenPinKeys(c, built.nets);
    expect(isOpenPin(openKeys, built.pinNode, "r1", "a")).toBe(false);
    expect(isOpenPin(openKeys, built.pinNode, "r1", "b")).toBe(false);
    expect(assertOpenFlagsMatchClassic(c, "self-wired component")).toBeGreaterThan(0);
  });

  for (const name of fixtureNames) {
    it(`matches classic on every pin of ${name} (brownout fixture)`, () => {
      expect(assertOpenFlagsMatchClassic(loadFixture(name), name)).toBeGreaterThan(0);
    });
  }

  // devolt's core-corpus (87 circuits) is proprietary and generated locally,
  // never committed to brownout — this test reads it at test time only (its
  // circuit names/content are never copied into this repo) and is a no-op
  // when the directory is absent, per the task brief's "when present".
  const DEVOLT_CORPUS_DIR = "/Users/dev/Documents/DEVOLTProject/devolt/tools/core-corpus/out";
  const corpusPresent = fs.existsSync(path.join(DEVOLT_CORPUS_DIR, "manifest.json"));

  (corpusPresent ? describe : describe.skip)("devolt core-corpus (when present)", () => {
    interface CorpusManifestEntry {
      file: string;
    }
    interface CorpusManifest {
      partLibrary: string;
      entries: CorpusManifestEntry[];
    }

    const manifest = corpusPresent
      ? (JSON.parse(fs.readFileSync(path.join(DEVOLT_CORPUS_DIR, "manifest.json"), "utf8")) as CorpusManifest)
      : null;

    afterAll(() => {
      if (corpusPresent) resetPartLibrary();
    });

    it("found at least one corpus entry", () => {
      expect(manifest!.entries.length).toBeGreaterThan(0);
    });

    it("matches classic's _isOpenPin on every pin of every corpus circuit", () => {
      const parts = JSON.parse(
        fs.readFileSync(path.join(DEVOLT_CORPUS_DIR, manifest!.partLibrary), "utf8"),
      ) as PartDefinition[];
      setPartLibrary(parts);
      try {
        let totalChecked = 0;
        for (const entry of manifest!.entries) {
          const c = JSON.parse(fs.readFileSync(path.join(DEVOLT_CORPUS_DIR, entry.file), "utf8")) as SimCircuit;
          // Deliberately no per-file try/catch: a load or comparison failure
          // here is exactly the "prove a detector fires" case the standing
          // rule (devolt-w8-tooling-traps.md) warns about — it must fail the
          // suite loudly, with the offending file's path, not be swallowed.
          // A single entry may legitimately have zero pins (e.g. a Learn
          // "starter" template with only a breadboard, which compiles to
          // nothing); the corpus-wide TOTAL below is what must be non-zero.
          totalChecked += assertOpenFlagsMatchClassic(c, entry.file);
        }
        expect(totalChecked).toBeGreaterThan(0);
      } finally {
        resetPartLibrary();
      }
    });
  });
});

describe("isOpenHighPin — classic's IC_OPEN_HIGH_PINS membership, imported directly", () => {
  it("matches the known open-high pins", () => {
    expect(isOpenHighPin("74ls47", "lt_n")).toBe(true);
    expect(isOpenHighPin("74ls47", "rbi_n")).toBe(true);
    expect(isOpenHighPin("cd4511", "bl_n")).toBe(true);
    expect(isOpenHighPin("74hc74", "pre1_n")).toBe(true);
    expect(isOpenHighPin("74hc74", "clr2_n")).toBe(true);
  });

  it("is false for every other kind/pin, including W1a's own kinds", () => {
    expect(isOpenHighPin("resistor", "a")).toBe(false);
    expect(isOpenHighPin("voltage_source", "pos")).toBe(false);
    expect(isOpenHighPin("74ls47", "a")).toBe(false); // a real 74ls47 pin, just not an open-high one
    expect(isOpenHighPin("unknown_kind", "lt_n")).toBe(false);
  });
});

// ---- FNV-1a-32 golden vectors (spec §1.5) ----------------------------------

describe("FNV-1a-32 — spec §1.5's golden vectors, finished in TS", () => {
  it("hash32('') is the basis", () => {
    expect(fnv1a32("")).toBe(0x811c9dc5);
  });

  it("openPinHashPrefix + finishFloatingHash reproduces hash32('u1:1b:<bucket>') for bucket 0 and 1", () => {
    const prefix = openPinHashPrefix("u1", "1b");
    expect(finishFloatingHash(prefix, 0)).toBe(0x1d7735be);
    expect(finishFloatingHash(prefix, 1)).toBe(0x1e773751);
  });

  it("deadBandHashPrefix + finishFloatingHash reproduces hash32('u1:1b:undefined:<bucket>') for bucket 0", () => {
    const prefix = deadBandHashPrefix("u1", "1b");
    expect(finishFloatingHash(prefix, 0)).toBe(0xb06a5224);
  });

  it("the golden vectors are also directly reproducible by hashing the full key string in one pass", () => {
    // Cross-check: hashing "u1:1b:0" in a single fnv1a32 call (no prefix/
    // finish split) must give the exact same golden value — this is the
    // same algorithm classic's own hash32 runs over the whole string.
    expect(fnv1a32("u1:1b:0")).toBe(0x1d7735be);
    expect(fnv1a32("u1:1b:1")).toBe(0x1e773751);
    expect(fnv1a32("u1:1b:undefined:0")).toBe(0xb06a5224);
  });
});

// ---- buildPinRecord ---------------------------------------------------------

describe("buildPinRecord", () => {
  it("assembles node/open/openHigh/openHash/deadHash for a real pin", () => {
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("rFloat", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "rFloat", "a")],
    );
    const built = buildCoreNodes(c);
    const openKeys = computeOpenPinKeys(c, built.nets);

    const floatingPin = buildPinRecord(built, openKeys, "resistor", "rFloat", "b");
    expect(floatingPin.open).toBe(true);
    expect(floatingPin.openHigh).toBe(false);
    expect(floatingPin.openHash).toBe(openPinHashPrefix("rFloat", "b"));
    expect(floatingPin.deadHash).toBe(deadBandHashPrefix("rFloat", "b"));
    expect(floatingPin.node).toBe(built.pinNode.get(corePinKey("rFloat", "b")));

    const wiredPin = buildPinRecord(built, openKeys, "resistor", "rFloat", "a");
    expect(wiredPin.open).toBe(false);
  });

  it("throws for a pin buildCoreNodes has no node for (a compiler bug, not a circuit fact)", () => {
    const c = circuit([part("r1", "resistor", ["a", "b"], {})]);
    const built = buildCoreNodes(c);
    const openKeys = computeOpenPinKeys(c, built.nets);
    expect(() => buildPinRecord(built, openKeys, "resistor", "r1", "nope")).toThrow(/has no node/);
  });
});

// ---- catalog.ts — delegates to classic's own resolver, never re-derives ----

describe("compile/catalog.ts — delegates to src/circuit/catalog-resolver.ts", () => {
  afterAll(() => resetPartLibrary());

  it("resolveComponentPart is exactly resolveCatalogPart's .part, for an explicit uid", () => {
    const input = { kind: "led" as const, catalogUid: "led-green", params: {} };
    expect(resolveComponentPart(input)).toBe(resolveCatalogPart(input));
    expect(resolveComponentPart(input)?.uid).toBe("led-green");
  });

  it("agrees with resolveCatalogPart for a stale/kind-mismatched uid (unresolved)", () => {
    const input = { kind: "led" as const, catalogUid: "resistor", params: {} }; // uid belongs to a different kind
    expect(resolveComponentPart(input)).toBe(resolveCatalogPart(input));
    expect(resolveComponentPart(input)).toBeNull();
  });

  it("agrees with resolveCatalogPart for a single-candidate kind with no uid ('only-candidate')", () => {
    const input = { kind: "resistor" as const, params: {} };
    const resolved = resolveComponentPart(input);
    expect(resolved).toBe(resolveCatalogPart(input));
    expect(resolved?.uid).toBe("resistor");
  });

  it("agrees with resolveCatalogPart for a multi-candidate kind with no discriminating params (canonical fallback)", () => {
    const input = { kind: "led" as const, params: {} };
    expect(resolveComponentPart(input)).toBe(resolveCatalogPart(input));
  });

  it("reacts to a later setPartLibrary() the same way resolveCatalogPart itself does (no private cache of its own)", () => {
    const custom: PartDefinition[] = [
      ...getPartLibrary(),
      {
        uid: "resistor-custom-test-only",
        kind: "resistor" as PartDefinition["kind"],
        default_params: { resistance: 4700 },
        pin_layout: [],
      },
    ];
    setPartLibrary(custom);
    try {
      const input = { kind: "resistor" as const, catalogUid: "resistor-custom-test-only", params: {} };
      expect(resolveComponentPart(input)?.default_params.resistance).toBe(4700);
    } finally {
      resetPartLibrary();
    }
  });

  describe("catalogParam — mirrors classic's private modelParam", () => {
    it("an authored param always wins, even with a valid catalogUid", () => {
      expect(catalogParam({ kind: "resistor", catalogUid: "resistor", params: { resistance: 999 } }, "resistance", 1000)).toBe(999);
    });

    it("falls back when there is no catalogUid at all", () => {
      expect(catalogParam({ kind: "resistor", params: {} }, "resistance", 1000)).toBe(1000);
    });

    it("reads the catalog default when the uid resolves to an EXACT match", () => {
      expect(catalogParam({ kind: "resistor", catalogUid: "resistor", params: {} }, "resistance", 1000)).toBe(220);
    });

    it("falls back for a stale/kind-mismatched uid rather than adopting a different part's defaults", () => {
      expect(catalogParam({ kind: "led", catalogUid: "resistor", params: {} }, "vf", 2)).toBe(2);
    });

    it("falls back when the resolved part has no default for that key", () => {
      expect(catalogParam({ kind: "resistor", catalogUid: "resistor", params: {} }, "not_a_real_key", "fallback")).toBe(
        "fallback",
      );
    });
  });
});

// ---- SlotAllocator — the readout-slot machinery, tested with synthetic keys

describe("SlotAllocator — synthetic slot-table machinery (no real kind claims a slot in W1a)", () => {
  it("assigns sequential indices in allocation order", () => {
    const digital = new SlotAllocator<string>();
    expect(digital.allocate("u1/qa")).toBe(0);
    expect(digital.allocate("u1/qb")).toBe(1);
    expect(digital.allocate("u2")).toBe(2);
    expect(digital.count).toBe(3);
    expect(digital.table).toEqual(["u1/qa", "u1/qb", "u2"]);
  });

  it("works for the channel-slot key shape too (a tuple, not a string)", () => {
    const channel = new SlotAllocator<readonly [string, number]>();
    expect(channel.allocate(["u3", 0])).toBe(0);
    expect(channel.allocate(["u3", 1])).toBe(1);
    expect(channel.count).toBe(2);
    expect(channel.table).toEqual([
      ["u3", 0],
      ["u3", 1],
    ]);
  });

  it("starts empty", () => {
    const allocator = new SlotAllocator<string>();
    expect(allocator.count).toBe(0);
    expect(allocator.table).toEqual([]);
  });
});

// ---- unsupported-kind reasons ----------------------------------------------

describe("compileCircuit — unsupported-kind reasons", () => {
  it("reports one deduplicated, sorted reason per distinct unimplemented kind, for every fixture", () => {
    for (const name of fixtureNames) {
      const c = loadFixture(name);
      const unimplementedKinds = new Set(
        c.components.map((component) => component.kind).filter((kind) => !CORE_IMPLEMENTED_KINDS.has(kind)),
      );
      const expected = [...unimplementedKinds].map((kind) => `unsupported-kind:${kind}`).sort();
      expect(expected.length).toBeGreaterThan(0); // every fixture here uses at least one un-implemented IC/sensor kind

      const result = compileCircuit(c);
      expect(result.ok).toBe(false);
      if (!result.ok) expect(result.reasons).toEqual(expected);
    }
  });

  it("dedupes repeated instances of the same unsupported kind on a synthetic circuit", () => {
    const c = circuit([
      part("r1", "resistor", ["a", "b"], { resistance: 100 }),
      part("led1", "led", ["a", "k"]),
      part("led2", "led", ["a", "k"]),
      part("ic1", "totally_fake_ic_kind", ["a", "b"]),
    ]);

    const result = compileCircuit(c);
    expect(result.ok).toBe(false);
    if (!result.ok) {
      expect(result.reasons).toEqual(["unsupported-kind:led", "unsupported-kind:totally_fake_ic_kind"].sort());
    }
  });

  it("honours a caller-supplied implementedKinds set narrower than the default", () => {
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
    );

    // The default set implements both kinds here, so this circuit compiles.
    expect(compileCircuit(c).ok).toBe(true);

    // A caller that only claims "resistor" support must see voltage_source
    // flagged, proving the parameter is actually consulted rather than the
    // module-level default being hardcoded somewhere internally.
    const result = compileCircuit(c, new Set(["resistor"]));
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["unsupported-kind:voltage_source"]);
  });
});

// ---- malformed components ---------------------------------------------------

describe("compileCircuit — malformed components", () => {
  it("collects one sorted, deduplicated reason per malformed component, without throwing", () => {
    const c = circuit([
      part("rBadR", "resistor", ["a", "b"], { resistance: "not-a-number" }),
      part("rInfR", "resistor", ["a", "b"], { resistance: Number.POSITIVE_INFINITY }),
      part("rOnePin", "resistor", ["a"]),
      part("vsBadV", "voltage_source", ["pos", "neg"], { voltage: "nope" }),
      part("vsOnePin", "voltage_source", ["pos"]),
      part("rGood", "resistor", ["a", "b"], { resistance: 1000 }),
    ]);

    let result: ReturnType<typeof compileCircuit> | undefined;
    expect(() => {
      result = compileCircuit(c);
    }).not.toThrow();
    expect(result!.ok).toBe(false);
    if (!result!.ok) {
      expect(result!.reasons).toEqual(
        [
          "malformed:rBadR:non-finite-resistance",
          "malformed:rInfR:non-finite-resistance",
          "malformed:rOnePin:too-few-pins",
          "malformed:vsBadV:non-finite-voltage",
          "malformed:vsOnePin:too-few-pins",
        ].sort(),
      );
    }
  });

  it("reports both an unsupported-kind and a malformed reason together when both apply", () => {
    const c = circuit([
      part("r1", "resistor", ["a"]), // malformed: too few pins
      part("mystery", "unknown_kind", ["x"]), // unsupported kind
    ]);
    const result = compileCircuit(c);
    expect(result.ok).toBe(false);
    if (!result.ok) {
      expect(result.reasons).toEqual(["malformed:r1:too-few-pins", "unsupported-kind:unknown_kind"].sort());
    }
  });

  it("rejects a non-finite circuit.environment.temperatureC (a malformed AMBIENT_C option)", () => {
    const c = circuit(
      [part("r1", "resistor", ["a", "b"], { resistance: 100 })],
      [],
      { temperatureC: Number.NaN },
    );
    const result = compileCircuit(c);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["malformed:circuit:non-finite-ambient"]);
  });
});

// ---- parameter resolution, options, pin records and slot tables (v2) ------

describe("compileCircuit — parameter resolution", () => {
  function compileOk(c: SimCircuit) {
    const result = compileCircuit(c);
    if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
    return result.compiled;
  }

  it("defaults resistance to 1000 and voltage to 5 when the param is absent", () => {
    const c = circuit([
      part("r1", "resistor", ["a", "b"]),
      part("vs1", "voltage_source", ["pos", "neg"]),
    ]);
    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);

    const r1 = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.resistor)!;
    const vs1 = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.voltage_source)!;
    expect(r1.params).toEqual([1000]);
    expect(vs1.params).toEqual([5]);
  });

  it("clamps resistance to the 1e-3 floor for zero and negative values", () => {
    const c = circuit([
      part("rZero", "resistor", ["a", "b"], { resistance: 0 }),
      part("rNeg", "resistor", ["a", "b"], { resistance: -5 }),
    ]);
    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    for (const device of decoded.devices) {
      expect(device.params).toEqual([1e-3]);
    }
  });

  it("passes through an explicit finite resistance/voltage unchanged", () => {
    const c = circuit([
      part("r1", "resistor", ["a", "b"], { resistance: 4700 }),
      part("vs1", "voltage_source", ["pos", "neg"], { voltage: 3.3 }),
    ]);
    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const r1 = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.resistor)!;
    const vs1 = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.voltage_source)!;
    expect(r1.params).toEqual([4700]);
    expect(vs1.params).toEqual([3.3]);
  });

  it("records device entries in component order, skipping ground, and componentIndex matches circuit.components", () => {
    const c = circuit([
      part("gref", "ground", ["gnd"]),
      part("r1", "resistor", ["a", "b"], { resistance: 10 }),
      part("vs1", "voltage_source", ["pos", "neg"], { voltage: 5 }),
    ]);
    const compiled = compileOk(c);
    expect(compiled.devices).toEqual([
      { componentId: "r1", kind: "resistor" },
      { componentId: "vs1", kind: "voltage_source" },
    ]);
    const decoded = decodeProgram(compiled.program);
    expect(decoded.devices.map((d) => d.componentIndex)).toEqual([1, 2]); // ground is components[0]
  });
});

describe("compileCircuit — v2 options block", () => {
  function compileOk(c: SimCircuit, options?: Parameters<typeof compileCircuit>[2]) {
    const result = compileCircuit(c, undefined, options);
    if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
    return result.compiled;
  }

  it("defaults INTEGRATION to backward Euler (0) and AMBIENT_C to 25 when unspecified", () => {
    const c = circuit([part("r1", "resistor", ["a", "b"], {})]);
    const decoded = decodeProgram(compileOk(c).program);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.INTEGRATION, 0]);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.AMBIENT_C, 25]);
  });

  it("writes INTEGRATION=1 for trapezoidal", () => {
    const c = circuit([part("r1", "resistor", ["a", "b"], {})]);
    const decoded = decodeProgram(compileOk(c, { integrationMethod: "trap" }).program);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.INTEGRATION, 1]);
  });

  it("derives AMBIENT_C from circuit.environment.temperatureC", () => {
    const c = circuit([part("r1", "resistor", ["a", "b"], {})], [], { temperatureC: 40 });
    const decoded = decodeProgram(compileOk(c).program);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.AMBIENT_C, 40]);
  });

  it("only writes the exactness-switch options when explicitly requested", () => {
    const c = circuit([part("r1", "resistor", ["a", "b"], {})]);
    const bare = decodeProgram(compileOk(c).program);
    expect(bare.options.some(([key]) => key === CORE_OPTION_KEYS.NO_SOLUTION_CACHE)).toBe(false);
    expect(bare.options.some(([key]) => key === CORE_OPTION_KEYS.NO_RAIL_SHORTCUT)).toBe(false);
    expect(bare.options.some(([key]) => key === CORE_OPTION_KEYS.FORCE_STEPPED)).toBe(false);

    const withSwitches = decodeProgram(
      compileOk(c, { noSolutionCache: true, noRailShortcut: true, forceStepped: true }).program,
    );
    expect(withSwitches.options).toContainEqual([CORE_OPTION_KEYS.NO_SOLUTION_CACHE, 1]);
    expect(withSwitches.options).toContainEqual([CORE_OPTION_KEYS.NO_RAIL_SHORTCUT, 1]);
    expect(withSwitches.options).toContainEqual([CORE_OPTION_KEYS.FORCE_STEPPED, 1]);
  });

  // Finding 3 (w1a-fixes/ts.json #3): a live-edit reload restarted Rust's own
  // clock at 0 while classic kept an absolute simTime, so a time-dependent
  // read taken right after a non-zero-time reload would key on the wrong
  // instant with no way to fix it from the TS side. The fix is START_TIME
  // (option key 7): unlike the exactness switches above, it must reach the
  // wire on EVERY program, including the very first at time 0 — omitting it
  // when absent (matching the exactness-switch pattern) would have been the
  // bug, not the fix, since key 7 needs an explicit value in the header to
  // seed Rust's clock at all.
  it("always writes START_TIME (default 0), unlike the opt-in exactness switches", () => {
    const c = circuit([part("r1", "resistor", ["a", "b"], {})]);
    const decoded = decodeProgram(compileOk(c).program);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.START_TIME, 0]);
  });

  it("writes the given startTimeS as START_TIME", () => {
    const c = circuit([part("r1", "resistor", ["a", "b"], {})]);
    const decoded = decodeProgram(compileOk(c, { startTimeS: 12.5 }).program);
    expect(decoded.options).toContainEqual([CORE_OPTION_KEYS.START_TIME, 12.5]);
  });
});

describe("compileCircuit — v2 pin records carry the same open flags computeOpenPinKeys would", () => {
  it("a floating resistor pin's program record has flags.open = true", () => {
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("rFloat", "resistor", ["a", "b"], { resistance: 100 })],
      [wire("vsrc", "pos", "rFloat", "a")],
    );
    const result = compileCircuit(c);
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    const decoded = decodeProgram(result.compiled.program);
    const rFloat = decoded.devices.find((d, i) => result.compiled.devices[i]!.componentId === "rFloat")!;
    // pins[0] = "a" (wired), pins[1] = "b" (floating).
    expect(rFloat.pins[0]!.open).toBe(false);
    expect(rFloat.pins[1]!.open).toBe(true);
    expect(rFloat.pins[1]!.openHash).toBe(openPinHashPrefix("rFloat", "b"));
    expect(rFloat.pins[1]!.deadHash).toBe(deadBandHashPrefix("rFloat", "b"));
  });

  it("neither resistor nor voltage_source ever sets openHigh (IC_OPEN_HIGH_PINS has no entry for them)", () => {
    const c = circuit([
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", ["a", "b"], {}),
    ]);
    const result = compileCircuit(c);
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    const decoded = decodeProgram(result.compiled.program);
    for (const device of decoded.devices) {
      for (const pin of device.pins) expect(pin.openHigh).toBe(false);
    }
  });
});

describe("compileCircuit — v2 readout slot tables (empty for W1a's kinds, machinery still runs)", () => {
  it("digitalSlots and channelSlots are empty, and the program's S/C header fields agree", () => {
    const c = circuit([
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", ["a", "b"], {}),
    ]);
    const result = compileCircuit(c);
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    expect(result.compiled.digitalSlots).toEqual([]);
    expect(result.compiled.channelSlots).toEqual([]);
    const decoded = decodeProgram(result.compiled.program);
    expect(decoded.digitalSlotCount).toBe(0);
    expect(decoded.channelSlotCount).toBe(0);
  });
});

// ---- compile.ts barrel stays in sync with compile/index.ts -----------------

describe("compile.ts — one-line re-export barrel", () => {
  it("re-exports the exact same bindings as compile/index.ts", () => {
    expect(compileCircuit).toBe(compileIndex.compileCircuit);
    expect(buildCoreNodes).toBe(compileIndex.buildCoreNodes);
    expect(SlotAllocator).toBe(compileIndex.SlotAllocator);
  });
});
