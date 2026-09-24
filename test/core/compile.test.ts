import { describe, expect, it } from "vitest";
import fs from "node:fs";
import path from "node:path";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import { buildCoreNodes, compileCircuit } from "../../src/core/compile.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";

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

function circuit(components: SimComponent[], wires: SimCircuit["wires"] = []): SimCircuit {
  return { components, wires };
}

/** `compiled.nets`/`buildCoreNodes(...).nets` must be produced by calling buildNets — verify via a fresh classic-engine load. */
function classicNets(c: SimCircuit): unknown {
  const engine = new SimEngine();
  engine.load(c);
  return engine.nets;
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
      // Every real fixture mixes in kinds the W0 core does not implement
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
});

// ---- parameter defaults and the resistor floor -----------------------------

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
