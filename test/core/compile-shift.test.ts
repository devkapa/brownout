/**
 * Unit coverage for compile/shift.ts: the 74hc595 (code 73) and 74hc165 (code
 * 74) compiled to v2 program records (W1h item H2a). Mirrors compile-registers.test.ts:
 * expected values come from classic's own helpers and the catalog, not from the
 * compiler under test.
 */

import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { propagationDelayForComponent, SimEngine, specsForComponent, supplyPinsForComponent } from "../../src/sim/engine/sim-engine.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { resolveDelayTail } from "../../src/core/compile/delays.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import { CORE_IC_PARAM, CORE_IC_PIN_FLOATING_INPUT, CORE_IC_PIN_OPEN_COLLECTOR, CORE_IMPLEMENTED_KINDS, CORE_W1H_IC_PINS } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS } from "../../src/parts/part-library.js";

type SimComponent = SimCircuit["components"][number];

const KINDS = ["74hc595", "74hc165"] as const;

function icFor(kind: (typeof KINDS)[number]): SimComponent {
  const def = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === kind)!;
  return { id: "u1", kind, catalogUid: def.uid, pins: def.pin_layout.map((p) => ({ id: p.id })), params: {} };
}

function powered(comp: SimComponent): SimCircuit {
  const { vccPin, gndPin } = supplyPinsForComponent(comp);
  return {
    components: [{ id: "vsrc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } }, comp],
    wires: [
      { from_component: "vsrc", from_pin: "pos", to_component: comp.id, to_pin: vccPin! },
      { from_component: comp.id, from_pin: gndPin!, to_component: "vsrc", to_pin: "neg" },
    ],
  };
}

describe("the shift registers are implemented kinds", () => {
  it("compile under the default gate", () => {
    for (const kind of KINDS) {
      expect(CORE_IMPLEMENTED_KINDS.has(kind), kind).toBe(true);
      expect(compileCircuit(powered(icFor(kind))).ok, kind).toBe(true);
    }
  });
});

describe.each(KINDS)("%s record", (kind) => {
  const comp = icFor(kind);
  const result = compileCircuit(powered(comp));
  if (!result.ok) throw new Error(result.reasons.join(", "));
  const compiled = result.compiled;
  const dev = decodeProgram(compiled.program).devices.find((d) => d.kindCode === CORE_KIND_CODE[kind])!;
  const logic = CORE_W1H_IC_PINS[kind]!;

  it("has vcc, gnd and the logic pins, and 13 IC params plus the delay tail", () => {
    expect(dev.pins).toHaveLength(2 + logic.length);
    expect(dev.params).toHaveLength(kind === "74hc595" ? 13 + 2 * 5 : 13 + 2 * 3);
  });

  it("reads through plain thresholds: schmitt, vtPlus and vtMinus are 0", () => {
    expect([dev.params[CORE_IC_PARAM.schmitt], dev.params[CORE_IC_PARAM.vtPlus], dev.params[CORE_IC_PARAM.vtMinus]]).toEqual([0, 0, 0]);
  });

  it("carries the catalog's datasheet delays, falling back to the package delay (R56 10)", () => {
    const specs = specsForComponent(comp);
    expect(dev.params.slice(13)).toEqual(resolveDelayTail(kind, specs, propagationDelayForComponent(comp)));
    const ns = dev.params.slice(13).map((s) => Math.round(s * 1e9));
    expect(ns).toEqual(kind === "74hc595" ? [17, 17, 17, 17, 18, 18, 15, 15, 23, 23] : [15, 15, 20, 20, 15, 15]);
  });

  it("marks the inputs FLOATING_INPUT and never OPEN_COLLECTOR, by classic's own open-pin test", () => {
    const engine = new SimEngine();
    engine.load(powered(comp));
    logic.forEach((pinId, i) => {
      const flags = dev.blob[1 + i]!;
      const classic = (engine as unknown as { _shouldTreatOpenPinAsDigitalInput(c: SimComponent, p: string): boolean })._shouldTreatOpenPinAsDigitalInput(comp, pinId);
      expect((flags & CORE_IC_PIN_FLOATING_INPUT) !== 0, pinId).toBe(classic);
      expect(flags & CORE_IC_PIN_OPEN_COLLECTOR, pinId).toBe(0);
    });
  });

  it("allocates the digital slots in classic's key order, QH' and the unlisted pins excluded", () => {
    const keys = compiled.digitalSlots.filter((k) => k.startsWith("u1/"));
    expect(keys).toEqual(kind === "74hc595" ? ["u1/qa", "u1/qb", "u1/qc", "u1/qd", "u1/qe", "u1/qf", "u1/qg", "u1/qh"] : ["u1/q7", "u1//q7"]);
    expect(dev.blob[0]).toBe(compiled.digitalSlots.indexOf(keys[0]!));
  });
});

describe("a part with the wrong pins", () => {
  it.each(KINDS)("%s declines as topology:ic-pins when a logic pin is missing", (kind) => {
    const comp = icFor(kind);
    const missing = { ...comp, pins: comp.pins.filter((p) => p.id !== CORE_W1H_IC_PINS[kind]![1]) };
    const result = compileCircuit(powered(missing));
    expect(result).toEqual({ ok: false, reasons: ["topology:ic-pins:u1"] });
  });
});

describe("the compiled record keeps srclk and rclk apart", () => {
  it("puts the net wired to srclk on logic pin 1 and the net wired to rclk on logic pin 2 (a swap would reorder the clocks)", () => {
    const comp = icFor("74hc595");
    const c = powered(comp);
    const res = (id: string): SimComponent => ({ id, kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } });
    c.components.push(res("rs"), res("rr"));
    c.wires.push(
      { from_component: "rs", from_pin: "a", to_component: "u1", to_pin: "srclk" },
      { from_component: "rr", from_pin: "a", to_component: "u1", to_pin: "rclk" },
      { from_component: "rs", from_pin: "b", to_component: "vsrc", to_pin: "neg" },
      { from_component: "rr", from_pin: "b", to_component: "vsrc", to_pin: "neg" },
    );
    const result = compileCircuit(c);
    if (!result.ok) throw new Error(result.reasons.join(", "));
    const devices = decodeProgram(result.compiled.program).devices;
    const ic = devices.find((d) => d.kindCode === CORE_KIND_CODE["74hc595"])!;
    const resistors = devices.filter((d) => d.kindCode === CORE_KIND_CODE.resistor);
    const nodeOf = (componentId: string) => resistors[c.components.filter((x) => x.kind === "resistor").findIndex((x) => x.id === componentId)]!.pins[0]!.node;
    const logic = CORE_W1H_IC_PINS["74hc595"]!;
    expect(ic.pins[2 + logic.indexOf("srclk")]!.node).toBe(nodeOf("rs"));
    expect(ic.pins[2 + logic.indexOf("rclk")]!.node).toBe(nodeOf("rr"));
    expect(nodeOf("rs")).not.toBe(nodeOf("rr"));
    expect(logic.indexOf("srclk")).toBe(1);
    expect(logic.indexOf("rclk")).toBe(2);
  });
});
