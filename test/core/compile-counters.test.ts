/**
 * Unit coverage for compile/counters.ts: the cd4017 (code 75) and cd4060 (code 76) compiled to v2 program
 * records (W1h item H2b). Mirrors compile-shift.test.ts: expected values come from classic's own helpers and the
 * catalog, not from the compiler under test.
 */

import { afterEach, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { propagationDelayForComponent, SimEngine, specsForComponent, supplyPinsForComponent } from "../../src/sim/engine/sim-engine.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { resolveDelayTail } from "../../src/core/compile/delays.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import { CORE_IC_PARAM, CORE_IC_PIN_FLOATING_INPUT, CORE_IC_PIN_OPEN_COLLECTOR, CORE_IMPLEMENTED_KINDS, CORE_W1H_IC_OUTPUTS, CORE_W1H_IC_PINS } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";

type SimComponent = SimCircuit["components"][number];

const KINDS = ["cd4017", "cd4060"] as const;

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

describe("the counters are implemented kinds", () => {
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

  it("has vcc, gnd and the logic pins, and 13 IC params plus the three-pair delay tail", () => {
    expect(dev.pins).toHaveLength(2 + logic.length);
    expect(dev.params).toHaveLength(13 + 2 * 3);
  });

  it("carries the catalog's datasheet delays, falling back to the package delay (R56 10)", () => {
    const specs = specsForComponent(comp);
    expect(dev.params.slice(13)).toEqual(resolveDelayTail(kind, specs, propagationDelayForComponent(comp)));
    const ns = dev.params.slice(13).map((s) => Math.round(s * 1e9));
    // clkQ, clrQ, clkCo (4017); clkQ, stageQ, clrQ (4060).
    expect(ns).toEqual(kind === "cd4017" ? [325, 325, 265, 265, 300, 300] : [370, 370, 100, 100, 180, 180]);
  });

  it("marks the inputs FLOATING_INPUT by classic's own open-pin test and never OPEN_COLLECTOR", () => {
    const engine = new SimEngine();
    engine.load(powered(comp));
    logic.forEach((pinId, i) => {
      const flags = dev.blob[1 + i]!;
      const classic = (engine as unknown as { _shouldTreatOpenPinAsDigitalInput(c: SimComponent, p: string): boolean })._shouldTreatOpenPinAsDigitalInput(comp, pinId);
      expect((flags & CORE_IC_PIN_FLOATING_INPUT) !== 0, pinId).toBe(classic);
      expect(flags & CORE_IC_PIN_OPEN_COLLECTOR, pinId).toBe(0);
    });
  });

  it("allocates the count's bare key, then the driven pins in record order", () => {
    const keys = compiled.digitalSlots.filter((k) => k === "u1" || k.startsWith("u1/"));
    const driven = CORE_W1H_IC_OUTPUTS[kind]!.map((i) => `u1/${logic[i]}`);
    expect(keys).toEqual(["u1", ...driven]);
    expect(keys).toEqual(
      kind === "cd4017"
        ? ["u1", "u1/q0", "u1/q1", "u1/q2", "u1/q3", "u1/q4", "u1/q5", "u1/q6", "u1/q7", "u1/q8", "u1/q9", "u1/co"]
        : ["u1", "u1/q4", "u1/q5", "u1/q6", "u1/q7", "u1/q8", "u1/q9", "u1/q10", "u1/q12", "u1/q13", "u1/q14", "u1/rtc", "u1/ctc"],
    );
    expect(dev.blob[0]).toBe(compiled.digitalSlots.indexOf("u1"));
  });
});

/** The compiled record of the one counter on a powered board. */
function deviceOf(kind: (typeof KINDS)[number]) {
  const result = compileCircuit(powered(icFor(kind)));
  if (!result.ok) throw new Error(result.reasons.join(", "));
  return decodeProgram(result.compiled.program).devices.find((d) => d.kindCode === CORE_KIND_CODE[kind])!;
}

describe("the Schmitt params", () => {
  it("are 0 on the cd4017, which reads every pin plainly", () => {
    const dev = deviceOf("cd4017");
    expect([dev.params[CORE_IC_PARAM.schmitt], dev.params[CORE_IC_PARAM.vtPlus], dev.params[CORE_IC_PARAM.vtMinus]]).toEqual([0, 0, 0]);
  });

  it("are the catalog's v_t_plus and v_t_minus on the cd4060 (unscaled: the core scales by Vs / nominal)", () => {
    const comp = icFor("cd4060");
    const specs = specsForComponent(comp)!;
    expect(specs.v_input!.v_t_plus).toBe(2.75);
    const dev = deviceOf("cd4060");
    expect([dev.params[CORE_IC_PARAM.schmitt], dev.params[CORE_IC_PARAM.vtPlus], dev.params[CORE_IC_PARAM.vtMinus]]).toEqual([1, 2.75, 2.25]);
    expect(dev.params[CORE_IC_PARAM.nominal]).toBe(specs.vcc_range!.nominal);
  });

  afterEach(() => resetPartLibrary());

  it("fall back to classic's 2.7 and 1.6 for a cd4060 whose catalog entry has no pair", () => {
    const base = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === "cd4060")!;
    const stripped = { ...base, electrical_specs: { ...base.electrical_specs, v_input: { v_il_max: 1.5, v_ih_min: 3.5 } } };
    setPartLibrary([stripped]);
    const result = compileCircuit(powered(icFor("cd4060")));
    if (!result.ok) throw new Error(result.reasons.join(", "));
    const dev = decodeProgram(result.compiled.program).devices.find((d) => d.kindCode === CORE_KIND_CODE.cd4060)!;
    expect([dev.params[CORE_IC_PARAM.schmitt], dev.params[CORE_IC_PARAM.vtPlus], dev.params[CORE_IC_PARAM.vtMinus]]).toEqual([1, 2.7, 1.6]);
  });
});

describe("the cd4060 pin ids follow the catalog, not icPinMap", () => {
  it("lists the unpinned-stage pins as q9 q8 q10 in the record order, with no q1 q2 q3", () => {
    expect(CORE_W1H_IC_PINS.cd4060).toEqual(["clk_in", "reset", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q12", "q13", "q14", "rtc", "ctc"]);
    const declared = icFor("cd4060").pins.map((p) => p.id).sort();
    expect(declared).toEqual([...CORE_W1H_IC_PINS.cd4060!, "gnd", "vcc"].sort());
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

describe("the compiled record keeps the controls apart", () => {
  it("puts the net wired to clk, clkinh and reset on logic pins 0, 1 and 2 of a cd4017", () => {
    const comp = icFor("cd4017");
    const c = powered(comp);
    const res = (id: string): SimComponent => ({ id, kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } });
    for (const [rid, pin] of [["rc", "clk"], ["ri", "clkinh"], ["rr", "reset"]] as const) {
      c.components.push(res(rid));
      c.wires.push({ from_component: rid, from_pin: "a", to_component: "u1", to_pin: pin }, { from_component: rid, from_pin: "b", to_component: "vsrc", to_pin: "neg" });
    }
    const result = compileCircuit(c);
    if (!result.ok) throw new Error(result.reasons.join(", "));
    const devices = decodeProgram(result.compiled.program).devices;
    const ic = devices.find((d) => d.kindCode === CORE_KIND_CODE.cd4017)!;
    const resistors = devices.filter((d) => d.kindCode === CORE_KIND_CODE.resistor);
    const nodeOf = (id: string) => resistors[c.components.filter((x) => x.kind === "resistor").findIndex((x) => x.id === id)]!.pins[0]!.node;
    expect([ic.pins[2]!.node, ic.pins[3]!.node, ic.pins[4]!.node]).toEqual([nodeOf("rc"), nodeOf("ri"), nodeOf("rr")]);
    expect(new Set([nodeOf("rc"), nodeOf("ri"), nodeOf("rr")]).size).toBe(3);
  });
});
