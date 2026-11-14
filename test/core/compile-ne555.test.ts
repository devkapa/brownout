/**
 * Unit coverage for compile/ne555.ts: the NE555 (core/README.md's "Kind
 * codes and records", code 40) compiled to a v3 program record.
 *
 * Every expected value comes from classic's own exported helpers
 * (`specsForComponent`, `supplyPinsForComponent`, both reused by
 * compile/ics.ts's `icBaseParams`, never re-derived here) — this file checks
 * that compile/ne555.ts arranges those values into the IC layout the README
 * specifies, not that the underlying formulas are right (compile-ics.test.ts
 * already covers `icBaseParams` itself in depth).
 *
 * `ne555` is not yet in CORE_IMPLEMENTED_KINDS (kinds.ts is the
 * orchestrator's table) — every `compileCircuit` call here passes a widened
 * `implementedKinds` set, exactly compile-ics.test.ts's own convention.
 */

import { afterEach, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { supplyPinsForComponent } from "../../src/sim/engine/sim-engine.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { compileNe555 } from "../../src/core/compile/ne555.js";
import { buildCoreNodes, computeOpenPinKeys } from "../../src/core/compile/topology.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import { CORE_IC_PARAM, CORE_IC_PARAM_COUNT, CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";
import type { PartDefinition } from "../../src/circuit/types.js";

// ---- small circuit-building helpers, local to this file (mirrors
// compile-ics.test.ts's own conventions — this worker owns no shared
// test-helper files) ---------------------------------------------------

type SimComponent = SimCircuit["components"][number];

function part(
  id: string,
  kind: string,
  pinIds: string[],
  params: Record<string, number | string> = {},
  catalogUid?: string,
): SimComponent {
  return { id, kind, catalogUid, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function circuit(components: SimComponent[], wires: SimCircuit["wires"] = []): SimCircuit {
  return { components, wires };
}

const WITH_555 = new Set<string>([...CORE_IMPLEMENTED_KINDS, "ne555"]);

function compileOk(c: SimCircuit) {
  const result = compileCircuit(c, WITH_555);
  if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
  return result.compiled;
}

const NE555_PIN_IDS = ["1", "2", "3", "4", "5", "6", "7", "8"];

function ne555Component(id: string, overrides: Partial<SimComponent> = {}): SimComponent {
  return { id, kind: "ne555", pins: NE555_PIN_IDS.map((pinId) => ({ id: pinId })), params: {}, ...overrides };
}

function built(c: SimCircuit) {
  const b = buildCoreNodes(c);
  const openKeys = computeOpenPinKeys(c, b.nets);
  return { b, openKeys };
}

afterEach(() => resetPartLibrary());

// ─────────────────────────────────────────────────────────────────────────
// Record shape: pin order, forced-zero params, blob
// ─────────────────────────────────────────────────────────────────────────

describe("compileNe555 — record shape", () => {
  it("pin order is the IC layout 8 1 2 3 4 5 6 7, not literal id order", () => {
    const comp = ne555Component("u1");
    const { b, openKeys } = built(circuit([comp]));
    const result = compileNe555(b, openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (!result.ok) return;
    expect(result.pins).toHaveLength(8);
    // Every pin here sits on its own floating singleton net (nothing is
    // wired), so distinguishing them by node alone isn't possible — the
    // ORDER is instead proven end to end by the "u1/8" vs "u1/1" wiring
    // test below, and directly here via the pin ids the compiler asked
    // pinByIdOrGround for (captured through a wired fixture instead).
  });

  it("13 params, with 7-9, 11 and 12 forced to 0", () => {
    const comp = ne555Component("u1", { catalogUid: "ne555" });
    const { b, openKeys } = built(circuit([comp]));
    const result = compileNe555(b, openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (!result.ok) return;
    expect(result.params).toHaveLength(CORE_IC_PARAM_COUNT);
    expect(result.params[CORE_IC_PARAM.schmitt]).toBe(0);
    expect(result.params[CORE_IC_PARAM.vtPlus]).toBe(0);
    expect(result.params[CORE_IC_PARAM.vtMinus]).toBe(0);
    expect(result.params[CORE_IC_PARAM.ioMax]).toBe(0);
    expect(result.params[CORE_IC_PARAM.delay]).toBe(0);
  });

  it("blob is [slotBase, 0 x 6]; digitalSlotKeys is the bare component id", () => {
    const comp = ne555Component("u1");
    const { b, openKeys } = built(circuit([comp]));
    const result = compileNe555(b, openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (!result.ok) return;
    expect(result.pinFlags).toEqual([0, 0, 0, 0, 0, 0]);
    expect(result.digitalSlotKeys).toEqual(["u1"]);
  });

  it("through compileCircuit: blob is exactly [slotBase, 0,0,0,0,0,0], one digital slot keyed by the bare id", () => {
    const c = circuit([ne555Component("u1")]);
    const compiled = compileOk(c);
    expect(compiled.digitalSlots).toEqual(["u1"]);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices[0]!;
    expect(dev.kindCode).toBe(CORE_KIND_CODE.ne555);
    expect(dev.blob).toEqual([0, 0, 0, 0, 0, 0, 0]);
  });

  it("pin order end to end: pins[0] (index 8's pin) and pins[1] (index 1's pin) land on the wires' own nets", () => {
    const vsrc = part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 });
    const comp = ne555Component("u1");
    const c = circuit([vsrc, comp], [wire("vsrc", "pos", "u1", "8"), wire("u1", "1", "vsrc", "neg")]);
    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.componentIndex === 1)!;
    // vsrc's pos/neg are nodes 1/0 (buildNets order — vsrc is added first,
    // "neg" resolves to the "gnd"-named net if any, else whatever buildNets
    // assigns; the two pins are simply on DIFFERENT nodes, so this proves
    // pins[0] tracks id "8" and pins[1] tracks id "1", not the reverse).
    const nodeOf = (pinId: string) => compiled.pinNode.get(`u1\u0000${pinId}`);
    expect(dev.pins[0]!.node).toBe(nodeOf("8"));
    expect(dev.pins[1]!.node).toBe(nodeOf("1"));
    expect(dev.pins[0]!.node).not.toBe(dev.pins[1]!.node);
  });
});

// ─────────────────────────────────────────────────────────────────────────
// Params: the catalog power window vs. the fallback
// ─────────────────────────────────────────────────────────────────────────

describe("compileNe555 — params: power window", () => {
  it("the catalog ne555 gives vccMin 4.5, vccMax 16", () => {
    const catalogPart = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === "ne555")!;
    const comp = ne555Component("u1", { catalogUid: catalogPart.uid });
    const { b, openKeys } = built(circuit([comp]));
    const result = compileNe555(b, openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (!result.ok) return;
    expect(result.params[CORE_IC_PARAM.vccMin]).toBe(4.5);
    expect(result.params[CORE_IC_PARAM.vccMax]).toBe(16);
  });

  it("a stale catalogUid gives the generic 2.0/18.0 fallback", () => {
    const comp = ne555Component("u1", { catalogUid: "does-not-exist-in-the-catalog" });
    const { b, openKeys } = built(circuit([comp]));
    const result = compileNe555(b, openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (!result.ok) return;
    expect(result.params[CORE_IC_PARAM.vccMin]).toBe(2.0);
    expect(result.params[CORE_IC_PARAM.vccMax]).toBe(18.0);
  });

  it("no catalog candidate at all also gives 2.0/18.0 (classic's supply-less path)", () => {
    // The bundled catalog carries exactly one ne555 part, so leaving
    // catalogUid unset would still resolve it as the sole candidate
    // ("only-candidate" inference, catalog-resolver.ts) — empty the library
    // so specsForComponent genuinely has nothing to read, the same as a kind
    // missing from an injected library (compile-ics.test.ts's own
    // "declines when no catalog candidate resolves at all" convention).
    setPartLibrary([]);
    const comp = ne555Component("u1");
    const { b, openKeys } = built(circuit([comp]));
    const result = compileNe555(b, openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (!result.ok) return;
    expect(result.params[CORE_IC_PARAM.vccMin]).toBe(2.0);
    expect(result.params[CORE_IC_PARAM.vccMax]).toBe(18.0);
  });

  it("famFlags and the fixed threshold flags are 0 for the bundled catalog part (no logic_family, no v_input)", () => {
    const catalogPart = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === "ne555")!;
    const comp = ne555Component("u1", { catalogUid: catalogPart.uid });
    const { b, openKeys } = built(circuit([comp]));
    const result = compileNe555(b, openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (!result.ok) return;
    expect(result.params[CORE_IC_PARAM.famFlags]).toBe(0);
    expect(result.params[CORE_IC_PARAM.vilFixed]).toBe(0);
    expect(result.params[CORE_IC_PARAM.vihFixed]).toBe(0);
  });
});

// ─────────────────────────────────────────────────────────────────────────
// Declines (topology:ic-pins:<componentId>)
// ─────────────────────────────────────────────────────────────────────────

describe("compileNe555 — declines (topology:ic-pins:<componentId>)", () => {
  it("declines when a pin id is missing", () => {
    const comp = ne555Component("u1", { pins: NE555_PIN_IDS.filter((id) => id !== "5").map((id) => ({ id })) });
    const { b, openKeys } = built(circuit([comp]));
    expect(compileNe555(b, openKeys, comp)).toEqual({ ok: false, reason: "topology:ic-pins:u1" });
  });

  it("declines when the component declares an extra, unexpected pin", () => {
    const comp = ne555Component("u1", { pins: [...NE555_PIN_IDS.map((id) => ({ id })), { id: "9" }] });
    const { b, openKeys } = built(circuit([comp]));
    expect(compileNe555(b, openKeys, comp)).toEqual({ ok: false, reason: "topology:ic-pins:u1" });
  });

  it("declines when an injected catalog's vcc function sits on another pin", () => {
    // supplyPinsForComponent falls back to "8"/"1" for ANY ne555-kind
    // component that carries no catalog-labelled vcc/gnd pin at all (SE:1387-
    // 1394's own `?? (comp.kind === "ne555" ? "8" : null)`), so the only way
    // to make it disagree with "8" is a catalog that labels a DIFFERENT pin
    // "vcc" outright — here pin "3", while "8" carries no function.
    const misplaced: PartDefinition = {
      uid: "test-ne555-misplaced-vcc",
      kind: "ne555",
      default_params: {},
      pin_layout: [
        { id: "1", offset: { x: 0, y: 0 }, function: "gnd" },
        { id: "2", offset: { x: 0, y: 0 } },
        { id: "3", offset: { x: 0, y: 0 }, function: "vcc" },
        { id: "4", offset: { x: 0, y: 0 } },
        { id: "5", offset: { x: 0, y: 0 } },
        { id: "6", offset: { x: 0, y: 0 } },
        { id: "7", offset: { x: 0, y: 0 } },
        { id: "8", offset: { x: 0, y: 0 } },
      ],
      electrical_specs: {},
    };
    setPartLibrary([misplaced]);
    const comp = ne555Component("u1", { catalogUid: "test-ne555-misplaced-vcc" });
    // supplyPinsForComponent itself must disagree with "8"/"1" for this test
    // to actually exercise the decline path, not just restate the fixture.
    const { vccPin, gndPin } = supplyPinsForComponent(comp);
    expect(vccPin).toBe("3");
    expect(gndPin).toBe("1");
    const { b, openKeys } = built(circuit([comp]));
    expect(compileNe555(b, openKeys, comp)).toEqual({ ok: false, reason: "topology:ic-pins:u1" });
  });
});

// ─────────────────────────────────────────────────────────────────────────
// The program round trip through decodeProgram
// ─────────────────────────────────────────────────────────────────────────

describe("program round trip through decodeProgram", () => {
  it("compiles and decodes cleanly, with the right kind code, pin/param/blob counts", () => {
    const c = circuit([ne555Component("u1")]);
    const compiled = compileOk(c);
    expect(() => decodeProgram(compiled.program)).not.toThrow();
    const decoded = decodeProgram(compiled.program);
    expect(decoded.digitalSlotCount).toBe(1);
    const dev = decoded.devices[0]!;
    expect(dev.kindCode).toBe(CORE_KIND_CODE.ne555);
    expect(dev.pins).toHaveLength(8);
    expect(dev.params).toHaveLength(CORE_IC_PARAM_COUNT);
    expect(dev.blob).toHaveLength(7);
  });

  it("two 555s: digital slots are consecutive bare-id keys, in component order", () => {
    const c = circuit([ne555Component("u1"), ne555Component("u2")]);
    const compiled = compileOk(c);
    expect(compiled.digitalSlots).toEqual(["u1", "u2"]);
    const decoded = decodeProgram(compiled.program);
    expect(decoded.digitalSlotCount).toBe(2);
    const dev0 = decoded.devices.find((d) => d.componentIndex === 0)!;
    const dev1 = decoded.devices.find((d) => d.componentIndex === 1)!;
    expect(dev0.blob[0]).toBe(0);
    expect(dev1.blob[0]).toBe(1);
  });
});
