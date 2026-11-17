/**
 * Unit coverage for compile/registers.ts: the 3 W1d sequential ICs (core/
 * README.md's "Kind codes and records", codes 70-72: 74ls161, 74ls173,
 * 74hc74) compiled to v2 program records.
 *
 * Follows compile-ics.test.ts's own conventions exactly (same helpers, same
 * "read expected values from classic's own exported/private helpers, never
 * re-derive" stance) — see that file's header. These 3 kinds are not yet in
 * CORE_IMPLEMENTED_KINDS (kinds.ts is the orchestrator's table, W1d worker
 * D3's brief: "CORE_IMPLEMENTED_KINDS stays unchanged"), so every
 * `compileCircuit` call here passes a widened `implementedKinds` set.
 */

import { afterAll, afterEach, beforeAll, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import {
  FLOATING_INPUT_FUNCTIONS,
  propagationDelayForComponent,
  SimEngine,
  specsForComponent,
  supplyPinsForComponent,
} from "../../src/sim/engine/sim-engine.js";
import { IC_OPEN_HIGH_PINS, icPinMap } from "../../src/sim/engine/digital.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { resolveDelayTail } from "../../src/core/compile/delays.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import {
  CORE_IC_DELAY_PAIRS,
  CORE_IC_PARAM,
  CORE_IC_PIN_FLOATING_INPUT,
  CORE_IC_PIN_OPEN_COLLECTOR,
  CORE_IMPLEMENTED_KINDS,
} from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";
import type { PartDefinition } from "../../src/circuit/types.js";

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

const REGISTER_KINDS = ["74ls161", "74ls173", "74hc74"] as const;

/** The README's "Sequential and memory IC logic pins" table, hardcoded independently of compile/registers.ts's own copy — a drift between the two fails here. */
const REGISTER_LOGIC_PINS: Readonly<Record<(typeof REGISTER_KINDS)[number], readonly string[]>> = {
  "74ls161": ["/clr", "clk", "/load", "enp", "ent", "a", "b", "c", "d", "qa", "qb", "qc", "qd", "rco"],
  "74ls173": ["/clr", "clk", "m", "n", "g1", "g2", "d1", "d2", "d3", "d4", "q1", "q2", "q3", "q4"],
  "74hc74": icPinMap("74hc74").map((p) => p.id),
};

const WITH_REGISTERS = new Set<string>([...CORE_IMPLEMENTED_KINDS, ...REGISTER_KINDS]);

function compileOk(c: SimCircuit) {
  const result = compileCircuit(c, WITH_REGISTERS);
  if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
  return result.compiled;
}

function partsFor(kind: string): PartDefinition[] {
  return BUNDLED_DEFAULT_PARTS.filter((p) => p.kind === kind);
}

function icComponent(id: string, partDef: PartDefinition): SimComponent {
  return { id, kind: partDef.kind, catalogUid: partDef.uid, pins: partDef.pin_layout.map((p) => ({ id: p.id })), params: {} };
}

function poweredIcCircuit(comp: SimComponent, voltage: number): SimCircuit {
  const { vccPin, gndPin } = supplyPinsForComponent(comp);
  if (!vccPin || !gndPin) throw new Error(`test bug: no supply pins resolved for ${comp.kind}`);
  return circuit(
    [part("vsrc", "voltage_source", ["pos", "neg"], { voltage }), comp],
    [wire("vsrc", "pos", comp.id, vccPin), wire(comp.id, gndPin, "vsrc", "neg")],
  );
}

function classicFloatingInput(engine: SimEngine, comp: SimComponent, pinId: string): boolean {
  return (
    engine as unknown as { _shouldTreatOpenPinAsDigitalInput(c: SimComponent, pinId: string): boolean }
  )._shouldTreatOpenPinAsDigitalInput(comp, pinId);
}

function classicIsOpenPin(engine: SimEngine, componentId: string, pinId: string): boolean {
  return (engine as unknown as { _isOpenPin(compId: string, pinId: string): boolean })._isOpenPin(componentId, pinId);
}

afterAll(() => resetPartLibrary());

// ---- per-kind: params, delay tail, pin order/flags, no OPEN_COLLECTOR -----

describe.each(REGISTER_KINDS)("%s — every default-parts.json part", (kind) => {
  const logicPins = REGISTER_LOGIC_PINS[kind];

  for (const partDef of partsFor(kind)) {
    describe(partDef.uid, () => {
      const comp = icComponent("u1", partDef);
      const c = poweredIcCircuit(comp, 5);
      const compiled = compileOk(c);
      const decoded = decodeProgram(compiled.program);
      const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE[kind])!;
      const specs = specsForComponent(comp);

      let engine: SimEngine;
      beforeAll(() => {
        engine = new SimEngine();
        engine.load(c);
      });

      it("compiled — one device, pin count = 2 supply + L logic pins, param count = 13 + 2*pairs", () => {
        expect(dev).toBeDefined();
        expect(dev.pins).toHaveLength(2 + logicPins.length);
        expect(dev.params).toHaveLength(13 + 2 * CORE_IC_DELAY_PAIRS[kind]!.length);
      });

      it("params 7 (schmitt)/8 (vtPlus)/9 (vtMinus) are 0 whatever the catalog says", () => {
        expect(dev.params[CORE_IC_PARAM.schmitt]).toBe(0);
        expect(dev.params[CORE_IC_PARAM.vtPlus]).toBe(0);
        expect(dev.params[CORE_IC_PARAM.vtMinus]).toBe(0);
      });

      it("vccMin/vccMax/famFlags/delay match classic's own field reads (icBaseParams, reused unchanged from compile/ics.ts)", () => {
        expect(dev.params[CORE_IC_PARAM.vccMin]).toBe(specs?.vcc_range?.min ?? 2.0);
        expect(dev.params[CORE_IC_PARAM.vccMax]).toBe(specs?.vcc_range?.max ?? 18.0);
        const family = specs?.logic_family ?? "";
        let expectedFamFlags = 0;
        if (family.includes("TTL")) expectedFamFlags |= 1; // CORE_IC_FAM_TTL
        if (family.includes("HC-CMOS") || family.includes("CMOS-4000")) expectedFamFlags |= 2; // CORE_IC_FAM_CMOS_RATIO
        expect(dev.params[CORE_IC_PARAM.famFlags]).toBe(expectedFamFlags);
        expect(dev.params[CORE_IC_PARAM.delay]).toBe(propagationDelayForComponent(comp));
      });

      it("the delay tail (params[13..]) is exactly resolveDelayTail(kind, specs, params[12])", () => {
        const expectedTail = resolveDelayTail(kind, specs, dev.params[CORE_IC_PARAM.delay]!);
        expect(dev.params.slice(13)).toEqual(expectedTail);
      });

      it("pin order matches the README's fixed table, by id, after vcc/gnd", () => {
        const { vccPin, gndPin } = supplyPinsForComponent(comp);
        const allPinIds = [vccPin!, gndPin!, ...logicPins];
        dev.pins.forEach((pinRecord, i) => {
          const pinId = allPinIds[i]!;
          expect(pinRecord.open).toBe(classicIsOpenPin(engine, "u1", pinId));
          expect(pinRecord.openHigh).toBe(IC_OPEN_HIGH_PINS[kind]?.has(pinId) ?? false);
        });
      });

      it("every logic pin's FLOATING_INPUT blob flag matches classic's _shouldTreatOpenPinAsDigitalInput", () => {
        logicPins.forEach((pinId, i) => {
          const flags = dev.blob[1 + i]!;
          const expected = classicFloatingInput(engine, comp, pinId);
          expect((flags & CORE_IC_PIN_FLOATING_INPUT) !== 0).toBe(expected);
        });
      });

      it("OPEN_COLLECTOR is never set (README: the reader rejects it for codes 70-82)", () => {
        logicPins.forEach((_pinId, i) => {
          const flags = dev.blob[1 + i]!;
          expect(flags & CORE_IC_PIN_OPEN_COLLECTOR).toBe(0);
        });
      });
    });
  }
});

// ---- 74hc74's pre/clr pins specifically: OPEN_HIGH through buildPinRecord -

describe("74hc74 — pre1_n/clr1_n/pre2_n/clr2_n carry OPEN_HIGH", () => {
  it("IC_OPEN_HIGH_PINS lists exactly these four for 74hc74 (digital.ts, reused not re-derived)", () => {
    expect(IC_OPEN_HIGH_PINS["74hc74"]).toEqual(new Set(["clr1_n", "pre1_n", "clr2_n", "pre2_n"]));
  });

  it("every one of the four pin records carries openHigh: true, regardless of open/closed", () => {
    const partDef = partsFor("74hc74")[0]!;
    const comp = icComponent("u1", partDef);
    const compiled = compileOk(poweredIcCircuit(comp, 5));
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE["74hc74"])!;
    const logicPins = REGISTER_LOGIC_PINS["74hc74"];
    for (const pinId of ["clr1_n", "pre1_n", "pre2_n", "clr2_n"]) {
      const i = logicPins.indexOf(pinId);
      expect(dev.pins[2 + i]!.openHigh).toBe(true);
    }
  });
});

// ---- fallback: a catalog with no switching_ns uses param 12 for every pair

describe("delay tail fallback: no switching_ns field at all", () => {
  afterEach(() => resetPartLibrary());

  it("74ls161: every pair falls back to param 12 (the package delay) when the catalog carries no switching_ns", () => {
    const original = partsFor("74ls161")[0]!;
    const stripped: PartDefinition = {
      ...original,
      electrical_specs: { ...original.electrical_specs, switching_ns: undefined },
    };
    setPartLibrary([stripped]);
    const comp = icComponent("u1", stripped);
    const compiled = compileOk(poweredIcCircuit(comp, 5));
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE["74ls161"])!;
    const delayParam = dev.params[CORE_IC_PARAM.delay]!;
    expect(dev.params.slice(13)).toEqual(new Array(8).fill(delayParam));
  });
});

// ---- digital slot keys: the README's own fixed list per kind, NOT derived
// from an output-capable-pin scan (see compile/registers.ts's header) -------

describe("digital slot keys (README's fixed key list per kind)", () => {
  it("74ls161: bare <id> (the count) then /qa../qd — no /rco", () => {
    const partDef = partsFor("74ls161")[0]!;
    const comp = icComponent("u1", partDef);
    const compiled = compileOk(poweredIcCircuit(comp, 5));
    expect(compiled.digitalSlots).toEqual(["u1", "u1/qa", "u1/qb", "u1/qc", "u1/qd"]);
  });

  it("74ls173: q1..q4", () => {
    const partDef = partsFor("74ls173")[0]!;
    const comp = icComponent("u1", partDef);
    const compiled = compileOk(poweredIcCircuit(comp, 5));
    expect(compiled.digitalSlots).toEqual(["u1/q1", "u1/q2", "u1/q3", "u1/q4"]);
  });

  it("74hc74: q1 q1_n q2 q2_n — classic's key order, NOT the pin order (q1 q1_n q2_n q2)", () => {
    const partDef = partsFor("74hc74")[0]!;
    const comp = icComponent("u1", partDef);
    const compiled = compileOk(poweredIcCircuit(comp, 5));
    expect(compiled.digitalSlots).toEqual(["u1/q1", "u1/q1_n", "u1/q2", "u1/q2_n"]);
    // The pin order itself really is q1, q1_n, q2_n, q2 — confirming the two
    // orders genuinely differ, not just asserting the slot order in isolation.
    expect(REGISTER_LOGIC_PINS["74hc74"].slice(4, 8)).toEqual(["q1", "q1_n", "q2_n", "q2"]);
  });

  it("slots allocate consecutively across components, in component order", () => {
    const p161 = partsFor("74ls161")[0]!;
    const u1 = icComponent("u1", p161);
    const u2 = icComponent("u2", p161);
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), u1, u2],
      [
        wire("vsrc", "pos", "u1", "vcc"), wire("u1", "gnd", "vsrc", "neg"),
        wire("vsrc", "pos", "u2", "vcc"), wire("u2", "gnd", "vsrc", "neg"),
      ],
    );
    const compiled = compileOk(c);
    expect(compiled.digitalSlots).toEqual(["u1", "u1/qa", "u1/qb", "u1/qc", "u1/qd", "u2", "u2/qa", "u2/qb", "u2/qc", "u2/qd"]);
    const decoded = decodeProgram(compiled.program);
    const devU1 = decoded.devices.find((d) => d.componentIndex === 1)!;
    const devU2 = decoded.devices.find((d) => d.componentIndex === 2)!;
    expect(devU1.blob[0]).toBe(0);
    expect(devU2.blob[0]).toBe(5);
  });
});

// ---- round trip through format.ts's decoder --------------------------------

describe("round trip through format.ts's decoder", () => {
  it("a compiled register program decodes cleanly, with blob[0] + digitalSlots within S", () => {
    const partDef = partsFor("74ls173")[0]!;
    const c = poweredIcCircuit(icComponent("u1", partDef), 5);
    const compiled = compileOk(c);
    expect(() => decodeProgram(compiled.program)).not.toThrow();
    const decoded = decodeProgram(compiled.program);
    expect(decoded.digitalSlotCount).toBe(4);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE["74ls173"])!;
    expect(dev.blob[0]! + 4).toBeLessThanOrEqual(decoded.digitalSlotCount);
  });
});

// ---- FLOATING_INPUT_FUNCTIONS sanity ---------------------------------------

describe("FLOATING_INPUT_FUNCTIONS", () => {
  it("is the exact set classic's own _shouldTreatOpenPinAsDigitalInput reads", () => {
    expect(FLOATING_INPUT_FUNCTIONS).toEqual(
      new Set(["input", "clock", "reset", "enable", "load", "io", "tri_state"]),
    );
  });
});

// ---- declines ---------------------------------------------------------------

describe.each(REGISTER_KINDS)("%s — declines (topology:ic-pins:<componentId>)", (kind) => {
  it("declines when the component's declared pins are missing one of the kind's logic pins", () => {
    const partDef = partsFor(kind)[0]!;
    const comp = icComponent("u1", partDef);
    const lastLogicPin = REGISTER_LOGIC_PINS[kind][REGISTER_LOGIC_PINS[kind].length - 1]!;
    comp.pins = comp.pins.filter((p) => p.id !== lastLogicPin);
    const c = poweredIcCircuit(comp, 5);
    const result = compileCircuit(c, WITH_REGISTERS);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
  });

  it("declines when the component declares an extra, unexpected pin", () => {
    const partDef = partsFor(kind)[0]!;
    const comp = icComponent("u1", partDef);
    comp.pins = [...comp.pins, { id: "bogus" }];
    const c = poweredIcCircuit(comp, 5);
    const result = compileCircuit(c, WITH_REGISTERS);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
  });

  it("declines when the component's pins are exactly the logic pins but neither vcc nor gnd", () => {
    const partDef = partsFor(kind)[0]!;
    const comp = icComponent("u1", partDef);
    comp.pins = comp.pins.filter((p) => p.id !== "vcc" && p.id !== "gnd");
    const c = circuit([comp]);
    const result = compileCircuit(c, WITH_REGISTERS);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
  });

  it("declines when no catalog candidate resolves at all (classic's supply-less path)", () => {
    setPartLibrary([]);
    try {
      const comp: SimComponent = { id: "u1", kind, pins: [{ id: "vcc" }, { id: "gnd" }], params: {} };
      const result = compileCircuit(circuit([comp]), WITH_REGISTERS);
      expect(result.ok).toBe(false);
      if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
    } finally {
      resetPartLibrary();
    }
  });
});
