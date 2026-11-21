/**
 * compile/magnetics.ts (W1h item H4a): `inductor`, `coupled_inductor` and `crystal` compiled to v2 records, the
 * crystal's internal node (compile/topology.ts), `galvanicGroups` in the section anchors (compile/sources.ts) and
 * the readout that sums a component's records (readout.ts).
 *
 * No kind is in `CORE_IMPLEMENTED_KINDS`: nothing steps an inductor until Phase B item H4b, so the default gate still
 * declines them and every `compileCircuit` call here widens it, the documented escape hatch. Expected values are
 * classic's own (spec 7.5, 7.15, 7.16): the exact-uid `modelParam` rule, the 1e-12 floors, `COUPLING_K_MAX`, the
 * crystal's `crystalParams`, and live `SimEngine` runs for the internal-node count and the isolated sections.
 */

import { afterAll, afterEach, beforeAll, describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { MAGNETICS_KINDS } from "../../src/core/compile/magnetics.js";
import { buildCoreNodes, corePinKey } from "../../src/core/compile/topology.js";
import { findSectionAnchors } from "../../src/core/compile/sources.js";
import { buildElementI, buildNetV } from "../../src/core/readout.js";
import { decodeProgram } from "../../src/core/format.js";
import {
  CORE_COUPLING_K_MAX,
  CORE_IMPLEMENTED_KINDS,
  CORE_KIND_CODES,
} from "../../src/core/kinds.js";
import { crystalParams } from "../../src/sim/engine/devices/model-families.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";
import type { PartDefinition } from "../../src/circuit/types.js";
import { __resetCoreForTest, coreCall, initCore } from "../../src/core/wasm.js";

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}, catalogUid?: string): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params, ...(catalogUid ? { catalogUid } : {}) };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

const KINDS = ["inductor", "coupled_inductor", "crystal"] as const;
const GATE: ReadonlySet<string> = new Set([...CORE_IMPLEMENTED_KINDS, ...KINDS]);

afterEach(() => resetPartLibrary());

/** The program's component records, in record order (the section anchors, code 26, left out). */
function recordsOf(program: Float64Array) {
  return decodeProgram(program).devices.filter((device) => device.kindCode !== CORE_KIND_CODES.section_anchor);
}

function compileOk(circuit: SimCircuit) {
  const result = compileCircuit(circuit, GATE);
  if (!result.ok) throw new Error(`expected the circuit to compile, got ${result.reasons.join(", ")}`);
  return result.compiled;
}

function reasonsOf(circuit: SimCircuit): string[] {
  const result = compileCircuit(circuit, GATE);
  if (result.ok) throw new Error("expected the circuit to decline");
  return result.reasons;
}

/** One part on a source loop, so every net is named and the part is not an isolated section. */
function loop(component: SimComponent, extraWires: SimCircuit["wires"] = []): SimCircuit {
  const [p0, p1] = component.pins.map((pin) => pin.id);
  return {
    components: [part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }), component],
    wires: [wire("vs", "pos", component.id, p0!), wire(component.id, p1!, "vs", "neg"), ...extraWires],
  };
}

function loadClassic(circuit: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  return engine;
}

describe("the default gate still declines the magnetics (nothing steps an inductor until H4b)", () => {
  it("answers unsupported-kind:<kind> and keeps all three kinds out of CORE_IMPLEMENTED_KINDS", () => {
    for (const kind of KINDS) {
      const result = compileCircuit({ components: [part("x1", kind, ["a", "b"])], wires: [] });
      expect(result, kind).toEqual({ ok: false, reasons: [`unsupported-kind:${kind}`] });
      expect(CORE_IMPLEMENTED_KINDS.has(kind), kind).toBe(false);
    }
    expect([...MAGNETICS_KINDS].sort()).toEqual([...KINDS].sort());
  });
});

describe("inductor (code 15; spec 7.5)", () => {
  it("emits [L, dcr, coreLossResistance] and positional pins", () => {
    const c = loop(part("l1", "inductor", ["a", "b"], { inductance: 4.7e-3, dcr: 0.8, coreLossResistance: 2e5 }));
    const compiled = compileOk(c);
    const [, record] = recordsOf(compiled.program);
    expect(record).toMatchObject({ kindCode: CORE_KIND_CODES.inductor, componentIndex: 1, params: [4.7e-3, 0.8, 2e5] });
    expect(record!.pins.map((pin) => pin.node)).toEqual(["a", "b"].map((id) => compiled.pinNode.get(corePinKey("l1", id))!));
  });

  it("is ideal by default: L 1e-3, dcr 0 and no core loss (the 0 sentinel)", () => {
    const [, record] = recordsOf(compileOk(loop(part("l1", "inductor", ["a", "b"]))).program);
    expect(record!.params).toEqual([1e-3, 0, 0]);
  });

  it("floors L at classic's 1e-12", () => {
    for (const inductance of [0, -3, 1e-20]) {
      const [, record] = recordsOf(compileOk(loop(part("l1", "inductor", ["a", "b"], { inductance }))).program);
      expect(record!.params[0], String(inductance)).toBe(1e-12);
    }
  });

  it("takes dcr and coreLossResistance from the exact catalog uid (the bundled generic inductor: 5 ohm, 1e5)", () => {
    const generic = part("l1", "inductor", ["a", "b"], { inductance: 1e-3 }, "inductor");
    expect(recordsOf(compileOk(loop(generic)).program)[1]!.params).toEqual([1e-3, 5, 1e5]);
    // An authored value always wins over the catalog default, a zero among them (0 means none).
    const authored = part("l2", "inductor", ["a", "b"], { inductance: 1e-3, dcr: 0, coreLossResistance: 7 }, "inductor");
    expect(recordsOf(compileOk(loop(authored)).program)[1]!.params).toEqual([1e-3, 0, 7]);
    // A stale uid adopts nothing.
    const stale = part("l3", "inductor", ["a", "b"], { inductance: 1e-3 }, "not-a-real-uid");
    expect(recordsOf(compileOk(loop(stale)).program)[1]!.params).toEqual([1e-3, 0, 0]);
  });

  it("agrees with classic's own dcr and core loss: one BE step of 5 V is h/(L + dcr h) V + V/coreLoss (spec 7.5)", () => {
    const c = loop(part("l1", "inductor", ["a", "b"], { inductance: 1e-3 }, "inductor"));
    const engine = loadClassic(c);
    const h = 0.05;
    engine.step(h);
    const [, record] = recordsOf(compileOk(c).program);
    const [l, dcr, coreLoss] = record!.params;
    expect(engine.getElementI()["l1"]).toBeCloseTo((h / (l! + dcr! * h)) * 5 + 5 / coreLoss!, 12);
  });

  it("maps a negative, zero or non-finite loss to none, and a loss that is a string to its number", () => {
    const record = (params: Record<string, number | string>) =>
      recordsOf(compileOk(loop(part("l1", "inductor", ["a", "b"], { inductance: 1e-3, ...params }))).program)[1]!.params;
    expect(record({ dcr: -2, coreLossResistance: -1 })).toEqual([1e-3, 0, 0]);
    expect(record({ dcr: "abc", coreLossResistance: Number.POSITIVE_INFINITY })).toEqual([1e-3, 0, 0]);
    expect(record({ dcr: "2.5", coreLossResistance: "300" })).toEqual([1e-3, 2.5, 300]);
  });

  it("declines fewer than two pins and a non-finite inductance", () => {
    expect(reasonsOf({ components: [part("l1", "inductor", ["a"])], wires: [] })).toEqual(["malformed:l1:too-few-pins"]);
    expect(reasonsOf(loop(part("l1", "inductor", ["a", "b"], { inductance: "abc" })))).toEqual(["malformed:l1:non-finite-inductance"]);
    expect(reasonsOf(loop(part("l1", "inductor", ["a", "b"], { inductance: Number.POSITIVE_INFINITY })))).toEqual(["malformed:l1:non-finite-inductance"]);
  });

  it("has no live params and carries no internal node", () => {
    const c = compileOk(loop(part("l1", "inductor", ["a", "b"])));
    expect(c.devices.map((device) => device.componentId)).toEqual(["vs", "l1"]);
    const built = buildCoreNodes(loop(part("l1", "inductor", ["a", "b"])));
    expect(built.programNodeCount).toBe(built.nodeCount);
    expect(built.internalNodes.size).toBe(0);
  });
});

describe("coupled_inductor (code 17; spec 7.16)", () => {
  const winding = { l1: 2e-3, l2: 8e-3, k: 0.9, dcr1: 1.5, dcr2: 3 };

  function coupledCircuit(params: Record<string, number | string>, uid?: string, pins = ["a1", "b1", "a2", "b2"]): SimCircuit {
    return {
      components: [
        part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("rl", "resistor", ["a", "b"], { resistance: 100 }),
        part("t1", "coupled_inductor", pins, params, uid),
      ],
      wires: [
        wire("vs", "pos", "t1", "a1"),
        wire("t1", "b1", "vs", "neg"),
        wire("t1", "a2", "rl", "a"),
        wire("rl", "b", "t1", "b2"),
        wire("rl", "b", "vs", "neg"),
      ],
    };
  }

  it("emits [l1, l2, k, dcr1, dcr2] with pins a1 b1 a2 b2 by id", () => {
    const compiled = compileOk(coupledCircuit(winding));
    const record = recordsOf(compiled.program).find((device) => device.kindCode === CORE_KIND_CODES.coupled_inductor)!;
    expect(record.params).toEqual([2e-3, 8e-3, 0.9, 1.5, 3]);
    expect(record.pins.map((pin) => pin.node)).toEqual(
      ["a1", "b1", "a2", "b2"].map((id) => compiled.pinNode.get(corePinKey("t1", id))!),
    );
    expect(compiled.devices.find((device) => device.componentId === "t1")?.pinIds).toEqual(["a1", "b1", "a2", "b2"]);
  });

  it("defaults to 1 mH windings, k 0.98 and no resistance", () => {
    const record = recordsOf(compileOk(coupledCircuit({})).program).find((d) => d.kindCode === CORE_KIND_CODES.coupled_inductor)!;
    expect(record.params).toEqual([1e-3, 1e-3, 0.98, 0, 0]);
  });

  it("floors the inductances at 1e-12 and clamps k to [0, COUPLING_K_MAX]", () => {
    const params = (extra: Record<string, number | string>) =>
      recordsOf(compileOk(coupledCircuit(extra)).program).find((d) => d.kindCode === CORE_KIND_CODES.coupled_inductor)!.params;
    expect(params({ l1: 0, l2: -1 }).slice(0, 2)).toEqual([1e-12, 1e-12]);
    expect(params({ k: 1 })[2]).toBe(CORE_COUPLING_K_MAX);
    expect(params({ k: 7 })[2]).toBe(CORE_COUPLING_K_MAX);
    expect(params({ k: -0.4 })[2]).toBe(0);
    expect(params({ k: 0 })[2]).toBe(0);
  });

  it("takes dcr1 and dcr2 by the exact-uid modelParam rule (an injected coupled-inductor part, as devolt's catalog has)", () => {
    const catalogPart: PartDefinition = {
      uid: "coupled-inductor",
      kind: "coupled_inductor",
      default_params: { l1: 1e-3, l2: 1e-3, k: 0.98, dcr1: 5, dcr2: 6 },
      pin_layout: ["a1", "b1", "a2", "b2"].map((id, i) => ({ id, offset: { x: i * 10, y: 0 }, function: "passive" })),
    } as unknown as PartDefinition;
    setPartLibrary([...BUNDLED_DEFAULT_PARTS, catalogPart]);
    const dcrs = (extra: Record<string, number | string>, uid?: string) =>
      recordsOf(compileOk(coupledCircuit(extra, uid)).program).find((d) => d.kindCode === CORE_KIND_CODES.coupled_inductor)!.params.slice(3);
    expect(dcrs({}, "coupled-inductor")).toEqual([5, 6]);
    expect(dcrs({ dcr1: 0.5 }, "coupled-inductor")).toEqual([0.5, 6]);
    expect(dcrs({ dcr2: 0 }, "coupled-inductor")).toEqual([5, 0]);
    expect(dcrs({}, "stale-uid")).toEqual([0, 0]);
    expect(dcrs({})).toEqual([0, 0]);
  });

  it("declines fewer than four pins and a non-finite l1, l2 or k", () => {
    expect(reasonsOf(coupledCircuit({}, undefined, ["a1", "b1", "a2"]))).toEqual(["malformed:t1:too-few-pins"]);
    expect(reasonsOf(coupledCircuit({ l1: "abc" }))).toEqual(["malformed:t1:non-finite-inductance"]);
    expect(reasonsOf(coupledCircuit({ l2: Number.POSITIVE_INFINITY }))).toEqual(["malformed:t1:non-finite-inductance"]);
    expect(reasonsOf(coupledCircuit({ k: "abc" }))).toEqual(["malformed:t1:non-finite-coupling"]);
  });

  it("reads a pin id the part lacks as the reference node, open (README 'Pins by id')", () => {
    // b2 is declared nowhere and wired nowhere: the part has a pin "bx" instead.
    const c = coupledCircuit({}, undefined, ["a1", "b1", "a2", "bx"]);
    c.wires = c.wires.filter((w) => w.from_pin !== "b2" && w.to_pin !== "b2");
    const compiled = compileOk(c);
    const record = recordsOf(compiled.program).find((d) => d.kindCode === CORE_KIND_CODES.coupled_inductor)!;
    expect(record.pins[3]!.node).toBe(0);
    expect(record.pins[3]!.open).toBe(true);
  });
});

describe("coupled_inductor galvanicGroups in the section anchors (spec 7.16; README 'Section anchor record')", () => {
  /** A primary on a source loop and a secondary into its own load, joined by nothing but the core. */
  function transformer(secondaryReturnsToGround: boolean): SimCircuit {
    return {
      components: [
        part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("t1", "coupled_inductor", ["a1", "b1", "a2", "b2"], { l1: 1e-3, l2: 1e-3, k: 0.95 }),
        part("rl", "resistor", ["a", "b"], { resistance: 100 }),
      ],
      wires: [
        wire("vs", "pos", "t1", "a1"),
        wire("t1", "b1", "vs", "neg"),
        wire("t1", "a2", "rl", "a"),
        wire("rl", "b", "t1", "b2"),
        ...(secondaryReturnsToGround ? [wire("t1", "b2", "vs", "neg")] : []),
      ],
    };
  }

  it("a floating secondary gets its own anchor, on the lowest node, as classic anchors it (probe P3d)", () => {
    const c = transformer(false);
    const anchors = findSectionAnchors(c, buildCoreNodes(c));
    const classicRows = loadClassic(c).isolatedSectionAnchorRows();
    expect(classicRows.length).toBe(1);
    expect(anchors.map((anchor) => anchor.node)).toEqual(classicRows.map((row) => row + 1));
    // The program carries the same list: one anchor record, on a node of the secondary section.
    const compiled = compileOk(c);
    const records = decodeProgram(compiled.program).devices.filter((d) => d.kindCode === CORE_KIND_CODES.section_anchor);
    expect(records.map((record) => record.pins[0]!.node)).toEqual(anchors.map((anchor) => anchor.node));
    const secondaryNodes = [compiled.pinNode.get(corePinKey("t1", "a2"))!, compiled.pinNode.get(corePinKey("t1", "b2"))!];
    expect(secondaryNodes).toContain(anchors[0]!.node);
  });

  it("a secondary returned to the primary ground needs no anchor", () => {
    const c = transformer(true);
    expect(findSectionAnchors(c, buildCoreNodes(c))).toEqual([]);
    expect(loadClassic(c).isolatedSectionAnchorRows()).toEqual([]);
  });

  it("an inductor, which declares no groups, still joins all its pins: no anchor for a coil in the source loop", () => {
    const c = loop(part("l1", "inductor", ["a", "b"]));
    expect(findSectionAnchors(c, buildCoreNodes(c))).toEqual([]);
  });
});

describe("crystal decomposed into Cs+ESR, Ls and C0 records with an internal node (R56 3; spec 7.15; D35, D36)", () => {
  const crystal = (params: Record<string, number | string> = {}) => part("x1", "crystal", ["a", "b"], params);
  const xtalLoop = (params: Record<string, number | string> = {}) => loop(crystal(params));

  it("allocates one internal node after the net rows, never in netV or a section", () => {
    const c = xtalLoop();
    const built = buildCoreNodes(c);
    expect(built.internalNodes.get("x1")).toBe(built.nodeCount);
    expect(built.programNodeCount).toBe(built.nodeCount + 1);
    expect(built.netIdOfNode.length).toBe(built.nodeCount);
    // The count classic itself builds: its rows exclude the datum, the core's node 0 is the datum.
    expect(loadClassic(c).matrixDimensions().nodeCount + 1).toBe(built.programNodeCount);
    // The program carries the larger node count, and the readout publishes nets only.
    const compiled = compileOk(c);
    expect(decodeProgram(compiled.program).nodeCount).toBe(built.programNodeCount);
    const voltages = Array.from({ length: built.programNodeCount }, (_, node) => node * 1.5);
    expect(Object.keys(buildNetV(compiled, voltages)).sort()).toEqual(["gnd", ...built.netIdOfNode.filter((id) => id !== null && id !== "gnd")].sort());
    expect(Object.values(buildNetV(compiled, voltages))).not.toContain(built.programNodeCount - 1 + 0.5);
  });

  it("allocates in circuit order, one per crystal with two pins, and none for any other circuit (identity)", () => {
    const two: SimCircuit = {
      components: [part("x1", "crystal", ["a", "b"]), part("r1", "resistor", ["a", "b"]), part("x2", "crystal", ["a", "b"]), part("x3", "crystal", ["a"])],
      wires: [],
    };
    const built = buildCoreNodes(two);
    expect([...built.internalNodes.entries()]).toEqual([["x1", built.nodeCount], ["x2", built.nodeCount + 1]]);
    expect(built.programNodeCount).toBe(built.nodeCount + 2);
    const plain = buildCoreNodes(loop(part("r1", "resistor", ["a", "b"])));
    expect(plain.internalNodes.size).toBe(0);
    expect(plain.programNodeCount).toBe(plain.nodeCount);
  });

  it("emits Cs+ESR (a to m), Ls (m to b, DCR 0, no core loss) and C0 (a to b), in that record order", () => {
    const c = xtalLoop();
    const compiled = compileOk(c);
    const built = buildCoreNodes(c);
    const m = built.internalNodes.get("x1")!;
    const [a, b] = ["a", "b"].map((id) => compiled.pinNode.get(corePinKey("x1", id))!);
    const p = crystalParams(c.components[1]!);
    const records = recordsOf(compiled.program).slice(1);
    expect(records.map((record) => record.kindCode)).toEqual([CORE_KIND_CODES.capacitor, CORE_KIND_CODES.inductor, CORE_KIND_CODES.capacitor]);
    expect(records.map((record) => record.componentIndex)).toEqual([1, 1, 1]);
    expect(records.map((record) => record.pins.map((pin) => pin.node))).toEqual([[a, m], [m, b], [a, b]]);
    expect(records[0]!.params).toEqual([p.cs, p.rs, 0]);
    expect(records[1]!.params).toEqual([p.ls, 0, 0]);
    expect(records[2]!.params).toEqual([p.c0, 0, 0]);
    expect(compiled.devices.slice(1).map((device) => [device.componentId, device.kind])).toEqual(new Array(3).fill(["x1", "crystal"]));
    // The internal node's pin is never open and is never a declared pin of the part.
    expect(records[0]!.pins[1]!.open).toBe(false);
    expect(records[1]!.pins[0]!.open).toBe(false);
  });

  it("derives the engine defaults: Ls 0.3978874 H, Cs 6.366198e-14 F, C0 5 pF (probe P3c)", () => {
    const records = recordsOf(compileOk(xtalLoop()).program).slice(1);
    expect(records[1]!.params[0]).toBeCloseTo(0.3978874, 7);
    expect(records[0]!.params[0]).toBeCloseTo(6.366198e-14, 20);
    expect(records[0]!.params[1]).toBe(50);
    expect(records[2]!.params[0]).toBeCloseTo(5e-12, 24);
  });

  it("follows authored fSeries, q, rs and c0 through classic's crystalParams (floors included)", () => {
    const params = { fSeries: 16e6, q: 25000, rs: 20, c0: 7 };
    const c = xtalLoop(params);
    const p = crystalParams(c.components[1]!);
    const records = recordsOf(compileOk(c).program).slice(1);
    expect(records[0]!.params).toEqual([p.cs, 20, 0]);
    expect(records[1]!.params).toEqual([p.ls, 0, 0]);
    expect(records[2]!.params).toEqual([7e-12, 0, 0]);
  });

  it("drops the C0 record when c0 is 0 (two records, two states)", () => {
    const c = xtalLoop({ c0: 0 });
    const compiled = compileOk(c);
    expect(recordsOf(compiled.program).slice(1).map((record) => record.kindCode)).toEqual([CORE_KIND_CODES.capacitor, CORE_KIND_CODES.inductor]);
    expect(compiled.devices.filter((device) => device.componentId === "x1")).toHaveLength(2);
    expect(buildCoreNodes(c).programNodeCount).toBe(buildCoreNodes(c).nodeCount + 1);
  });

  it("declines fewer than two pins and non-finite or underflowing parameters", () => {
    expect(reasonsOf({ components: [part("x1", "crystal", ["a"])], wires: [] })).toEqual(["malformed:x1:too-few-pins"]);
    const garbage: Array<Record<string, string>> = [{ fSeries: "abc" }, { q: "abc" }, { rs: "abc" }, { c0: "abc" }];
    for (const bad of garbage) {
      expect(reasonsOf(xtalLoop(bad)), JSON.stringify(bad)).toEqual(["malformed:x1:non-finite-crystal-params"]);
    }
    // 1e300 Hz squares to infinity, so Cs underflows to 0: the Rust reader would reject it, the compiler declines.
    expect(reasonsOf(xtalLoop({ fSeries: 1e300 }))).toEqual(["malformed:x1:non-positive-capacitance"]);
  });

  it("anchors no section on the internal node: a crystal across a floating pair is one section, as in classic", () => {
    const c: SimCircuit = {
      components: [
        part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r1", "resistor", ["a", "b"]),
        crystal(),
        part("r2", "resistor", ["a", "b"]),
      ],
      wires: [wire("vs", "pos", "r1", "a"), wire("r1", "b", "vs", "neg"), wire("x1", "a", "r2", "a"), wire("r2", "b", "x1", "b")],
    };
    const built = buildCoreNodes(c);
    const anchors = findSectionAnchors(c, built);
    expect(anchors.map((anchor) => anchor.node)).toEqual(loadClassic(c).isolatedSectionAnchorRows().map((row) => row + 1));
    for (const anchor of anchors) expect(anchor.node).toBeLessThan(built.nodeCount);
    // And the program is the same list, with the internal node still out of every anchor.
    const records = decodeProgram(compileOk(c).program).devices.filter((d) => d.kindCode === CORE_KIND_CODES.section_anchor);
    expect(records.map((record) => record.pins[0]!.node)).toEqual(anchors.map((anchor) => anchor.node));
  });

  it("has no live params", () => {
    const compiled = compileOk(xtalLoop());
    expect(compiled.devices).toHaveLength(4);
  });
});

describe("readout sums a component's records (spec 7.15: the crystal's elementI = Cs + C0)", () => {
  it("adds the Cs and C0 currents and leaves the Ls record out", () => {
    const compiled = compileOk(loop(part("x1", "crystal", ["a", "b"])));
    // Record order: vs, then x1's Cs, Ls, C0. The Ls current equals Cs's up to the node shunt and is not in the sum.
    const elementI = buildElementI(compiled, [0.5, 0.3, 0.29, 0.1]);
    expect(elementI["x1"]).toBe(0.3 + 0.1);
    expect(elementI["vs"]).toBe(0.5);
  });

  it("is the Cs current alone when c0 is 0", () => {
    const compiled = compileOk(loop(part("x1", "crystal", ["a", "b"], { c0: 0 })));
    expect(buildElementI(compiled, [0.5, 0.3, 0.29])["x1"]).toBe(0.3);
  });

  it("changes nothing for a one-record part, -0 included", () => {
    const compiled = compileOk(loop(part("l1", "inductor", ["a", "b"])));
    const elementI = buildElementI(compiled, [-0, -0]);
    expect(Object.is(elementI["l1"], -0)).toBe(true);
    expect(Object.is(elementI["vs"], -0)).toBe(true);
  });

  it("keeps the classic key set: one key per component with two pins, the crystal's three records counted once", () => {
    const compiled = compileOk(loop(part("x1", "crystal", ["a", "b"])));
    expect(Object.keys(buildElementI(compiled, [0, 0, 0, 0])).sort()).toEqual(["vs", "x1"]);
  });
});

describe("the emitted programs against the real Rust reader", () => {
  beforeAll(async () => {
    await initCore();
  });
  afterAll(() => {
    __resetCoreForTest();
  });

  /** `bo_engine_load`'s return code for `program` on a fresh engine: 0, or the reader's negative code. */
  function load(program: Float64Array): number {
    return coreCall((exports) => {
      const handle = exports.bo_engine_create();
      const ptr = exports.bo_alloc(program.length) >>> 0;
      try {
        new Float64Array(exports.memory.buffer, ptr, program.length).set(program);
        return exports.bo_engine_load(handle, ptr, program.length);
      } finally {
        exports.bo_free(ptr, program.length);
        exports.bo_engine_destroy(handle);
      }
    });
  }

  it("accepts an inductor, a coupled inductor and a crystal (with its internal node) as compiled, c0 present or absent", () => {
    const circuits: Array<[string, SimCircuit]> = [
      ["inductor", loop(part("l1", "inductor", ["a", "b"], { inductance: 2e-3 }, "inductor"))],
      ["coupled", {
        components: [
          part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
          part("t1", "coupled_inductor", ["a1", "b1", "a2", "b2"], { l1: 1e-3, l2: 4e-3, k: 0.95, dcr1: 1, dcr2: 2 }),
        ],
        wires: [wire("vs", "pos", "t1", "a1"), wire("t1", "b1", "vs", "neg")],
      }],
      ["crystal", loop(part("x1", "crystal", ["a", "b"]))],
      ["crystal without c0", loop(part("x1", "crystal", ["a", "b"], { c0: 0 }))],
    ];
    for (const [name, circuit] of circuits) {
      const compiled = compileOk(circuit);
      expect(load(compiled.program), name).toBe(0);
    }
  });
});
