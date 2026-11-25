/**
 * Unit coverage for compile/decoder.ts: the cd4511 (code 60) compiled to a v2 program record (W1h item H2c).
 * Conventions follow compile-registers.test.ts: expected values come from classic's own helpers, and the pin
 * order is hardcoded here independently of the compiler's copy so a drift between the two fails.
 */

import { afterAll, afterEach, beforeAll, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { propagationDelayForComponent, SimEngine, specsForComponent, supplyPinsForComponent } from "../../src/sim/engine/sim-engine.js";
import { IC_OPEN_HIGH_PINS } from "../../src/sim/engine/digital.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { resolveDelayTail } from "../../src/core/compile/delays.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import { CORE_IC_DELAY_PAIRS, CORE_IC_PARAM, CORE_IC_PIN_FLOATING_INPUT, CORE_IC_PIN_OPEN_COLLECTOR, CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";
import type { PartDefinition } from "../../src/circuit/types.js";

type SimComponent = SimCircuit["components"][number];

/** The H0 record order (core/src/digital/decoder.rs `PINS_CD4511`): controls, BCD data bit 0 first, then the segments. */
const LOGIC_PINS = ["lt_n", "bl_n", "le", "a", "b", "c", "d", "a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"] as const;

const part = (id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}): SimComponent => ({
  id,
  kind,
  pins: pinIds.map((pinId) => ({ id: pinId })),
  params,
});
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

const WITH_DECODER = new Set<string>([...CORE_IMPLEMENTED_KINDS, "cd4511"]);

function compileOk(c: SimCircuit) {
  const result = compileCircuit(c, WITH_DECODER);
  if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
  return result.compiled;
}

const cd4511Parts = BUNDLED_DEFAULT_PARTS.filter((p) => p.kind === "cd4511");

function icComponent(id: string, partDef: PartDefinition): SimComponent {
  return { id, kind: partDef.kind, catalogUid: partDef.uid, pins: partDef.pin_layout.map((p) => ({ id: p.id })), params: {} };
}

function powered(comp: SimComponent, volts = 5): SimCircuit {
  const { vccPin, gndPin } = supplyPinsForComponent(comp);
  return {
    components: [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: volts }), comp],
    wires: [wire("vsrc", "pos", comp.id, vccPin!), wire(comp.id, gndPin!, "vsrc", "neg")],
  };
}

afterAll(() => resetPartLibrary());

describe("cd4511 catalog", () => {
  it("has a part to test", () => {
    expect(cd4511Parts.length).toBeGreaterThan(0);
  });
});

describe.each(cd4511Parts.map((p) => [p.uid, p] as const))("cd4511 %s", (_uid, partDef) => {
  const comp = icComponent("u1", partDef);
  const c = powered(comp);
  const compiled = compileOk(c);
  const dev = decodeProgram(compiled.program).devices.find((d) => d.kindCode === CORE_KIND_CODE.cd4511)!;
  const specs = specsForComponent(comp);
  let engine: SimEngine;
  beforeAll(() => {
    engine = new SimEngine();
    engine.load(c);
  });

  it("is one record: 2 supply pins and 14 logic pins, 13 IC params and three delay pairs, blob [slotBase, flags x 14]", () => {
    expect(dev).toBeDefined();
    expect(dev.pins).toHaveLength(16);
    expect(dev.params).toHaveLength(13 + 2 * 3);
    expect(dev.blob).toHaveLength(15);
    expect(dev.blob[0]).toBe(0);
  });

  it("params 7 to 9 are zero (classic reads the cd4511 through logicHigh, never the Schmitt branch)", () => {
    expect(dev.params[CORE_IC_PARAM.schmitt]).toBe(0);
    expect(dev.params[CORE_IC_PARAM.vtPlus]).toBe(0);
    expect(dev.params[CORE_IC_PARAM.vtMinus]).toBe(0);
  });

  it("the base block and the package delay (param 12, what `le` stays on) are classic's own reads", () => {
    expect(dev.params[CORE_IC_PARAM.vccMin]).toBe(specs?.vcc_range?.min ?? 2.0);
    expect(dev.params[CORE_IC_PARAM.vccMax]).toBe(specs?.vcc_range?.max ?? 18.0);
    expect(dev.params[CORE_IC_PARAM.famFlags]).toBe(2); // CMOS-4000: the supply-ratioed thresholds, not TTL
    expect(dev.params[CORE_IC_PARAM.delay]).toBe(propagationDelayForComponent(comp));
  });

  it("the delay tail is the Data, BL and LT rows of the datasheet (660/520, 400/350, 150/250 ns), in the pair order", () => {
    expect(CORE_IC_DELAY_PAIRS.cd4511).toEqual(["dataSeg", "blSeg", "ltSeg"]);
    expect(dev.params.slice(13)).toEqual(resolveDelayTail("cd4511", specs, dev.params[CORE_IC_PARAM.delay]!));
    expect(dev.params.slice(13).map((s) => Math.round(s * 1e12) / 1e3)).toEqual([660, 520, 400, 350, 150, 250]);
  });

  it("pins are by id in the fixed order, openHigh exactly for lt_n and bl_n (IC_OPEN_HIGH_PINS), open as classic says", () => {
    const { vccPin, gndPin } = supplyPinsForComponent(comp);
    const ids = [vccPin!, gndPin!, ...LOGIC_PINS];
    const isOpenPin = (id: string) => (engine as unknown as { _isOpenPin(c: string, p: string): boolean })._isOpenPin("u1", id);
    dev.pins.forEach((pin, i) => {
      expect(pin.openHigh, ids[i]).toBe(IC_OPEN_HIGH_PINS.cd4511?.has(ids[i]!) ?? false);
      expect(pin.open, ids[i]).toBe(isOpenPin(ids[i]!));
    });
    expect(IC_OPEN_HIGH_PINS.cd4511).toEqual(new Set(["lt_n", "bl_n"]));
  });

  it("FLOATING_INPUT matches classic's open-pin rule on every logic pin, and OPEN_COLLECTOR is never set (push-pull)", () => {
    LOGIC_PINS.forEach((pinId, i) => {
      const flags = dev.blob[1 + i]!;
      const expected = (engine as unknown as { _shouldTreatOpenPinAsDigitalInput(c: SimComponent, p: string): boolean })._shouldTreatOpenPinAsDigitalInput(comp, pinId);
      expect((flags & CORE_IC_PIN_FLOATING_INPUT) !== 0, pinId).toBe(expected);
      expect(flags & CORE_IC_PIN_OPEN_COLLECTOR, pinId).toBe(0);
    });
  });

  it("claims seven digital slots keyed <id>/<segment pin>, a_out to g_out", () => {
    expect(compiled.digitalSlots).toEqual(["u1/a_out", "u1/b_out", "u1/c_out", "u1/d_out", "u1/e_out", "u1/f_out", "u1/g_out"]);
  });
});

describe("cd4511 compile edges", () => {
  afterEach(() => resetPartLibrary());

  it("slots run consecutively across two decoders in component order", () => {
    const partDef = cd4511Parts[0]!;
    const u1 = icComponent("u1", partDef);
    const u2 = icComponent("u2", partDef);
    const c: SimCircuit = {
      components: [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), u1, u2],
      wires: [wire("vsrc", "pos", "u1", "vcc"), wire("u1", "gnd", "vsrc", "neg"), wire("vsrc", "pos", "u2", "vcc"), wire("u2", "gnd", "vsrc", "neg")],
    };
    const compiled = compileOk(c);
    expect(compiled.digitalSlots).toHaveLength(14);
    expect(compiled.digitalSlots[7]).toBe("u2/a_out");
    const devs = decodeProgram(compiled.program).devices.filter((d) => d.kindCode === CORE_KIND_CODE.cd4511);
    expect(devs.map((d) => d.blob[0])).toEqual([0, 7]);
  });

  it("a catalog with no switching_ns falls back to the package delay for every pair", () => {
    const original = cd4511Parts[0]!;
    const stripped: PartDefinition = { ...original, electrical_specs: { ...original.electrical_specs, switching_ns: undefined } };
    setPartLibrary([stripped]);
    const compiled = compileOk(powered(icComponent("u1", stripped)));
    const dev = decodeProgram(compiled.program).devices.find((d) => d.kindCode === CORE_KIND_CODE.cd4511)!;
    expect(dev.params.slice(13)).toEqual(new Array(6).fill(dev.params[CORE_IC_PARAM.delay]));
  });

  it("a component whose pins are not exactly vcc, gnd and the 14 logic pins declines as topology:ic-pins", () => {
    const comp = icComponent("u1", cd4511Parts[0]!);
    comp.pins = comp.pins.filter((p) => p.id !== "le");
    const result = compileCircuit(powered(comp), WITH_DECODER);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toContain("topology:ic-pins:u1");
  });

  it("is not compiled without the kind in the implemented set (the default gate)", () => {
    const result = compileCircuit(powered(icComponent("u1", cd4511Parts[0]!)), new Set(["voltage_source", "ground", "resistor"]));
    expect(result.ok).toBe(false);
  });
});
