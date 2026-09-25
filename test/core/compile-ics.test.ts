/**
 * Unit coverage for compile/ics.ts: the 11 W1c combinational ICs (core/
 * README.md's "Kind codes and records", codes 50-59/61) compiled to v2
 * program records.
 *
 * Every part checked here comes straight from src/parts/default-parts.json
 * (BUNDLED_DEFAULT_PARTS) — not a hand-built fixture — and every expected
 * value comes from classic's OWN exported helpers (`specsForComponent`,
 * `supplyPinsForComponent`, `propagationDelayForComponent`, `thresholdsFor`,
 * `FLOATING_INPUT_FUNCTIONS`) or classic's private methods reached through a
 * test-only cast (`_icPowerInfo`, `_shouldTreatOpenPinAsDigitalInput`,
 * `_isOpenPin` — compile.test.ts's own established convention: never copied
 * or re-derived, just called). A drift between this module and classic's
 * real runtime behaviour fails here, not just against this file's own
 * restatement of the README.
 *
 * These 11 kinds are not yet in CORE_IMPLEMENTED_KINDS (kinds.ts is the
 * orchestrator's table) — every `compileCircuit` call here passes a widened
 * `implementedKinds` set, exactly compile-passives.test.ts's own convention.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import {
  FLOATING_INPUT_FUNCTIONS,
  GATE_R_OUT,
  propagationDelayForComponent,
  SimEngine,
  specsForComponent,
  supplyPinsForComponent,
  thresholdsFor,
} from "../../src/sim/engine/sim-engine.js";
import { IC_OPEN_HIGH_PINS, icPinMap } from "../../src/sim/engine/digital.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import {
  CORE_IC_FAM_CMOS_RATIO,
  CORE_IC_FAM_TTL,
  CORE_IC_PARAM,
  CORE_IC_PIN_FLOATING_INPUT,
  CORE_IC_PIN_OPEN_COLLECTOR,
  CORE_IMPLEMENTED_KINDS,
} from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";
import type { PartDefinition } from "../../src/circuit/types.js";

// ---- small circuit-building helpers, local to this file (mirrors
// compile.test.ts / compile-passives.test.ts's own conventions — this worker
// owns no shared test-helper files) -----------------------------------------

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

const IC_KINDS = [
  "74ls00", "74ls04", "74ls08", "74ls32", "74ls86",
  "74ls157", "74ls283", "74hc138", "74hc14", "74ls47", "74ls245",
] as const;

const WITH_ICS = new Set<string>([...CORE_IMPLEMENTED_KINDS, ...IC_KINDS]);

function compileOk(c: SimCircuit) {
  const result = compileCircuit(c, WITH_ICS);
  if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
  return result.compiled;
}

function partsFor(kind: string): PartDefinition[] {
  return BUNDLED_DEFAULT_PARTS.filter((p) => p.kind === kind);
}

/** A component built straight from a catalog part's own pin_layout — the exact set compileCombinationalIc requires (vcc/gnd + the kind's logic pins, no more, no fewer). */
function icComponent(id: string, partDef: PartDefinition): SimComponent {
  return { id, kind: partDef.kind, catalogUid: partDef.uid, pins: partDef.pin_layout.map((p) => ({ id: p.id })), params: {} };
}

/** `comp` wired to an ideal `voltage` source across its vcc/gnd pins; every other logic pin left floating (open) — plenty to exercise power, thresholds and R_out, and it also gives every logic pin a real OPEN reading to check against classic's `_isOpenPin`. */
function poweredIcCircuit(comp: SimComponent, voltage: number): SimCircuit {
  const { vccPin, gndPin } = supplyPinsForComponent(comp);
  if (!vccPin || !gndPin) throw new Error(`test bug: no supply pins resolved for ${comp.kind}`);
  return circuit(
    [part("vsrc", "voltage_source", ["pos", "neg"], { voltage }), comp],
    [wire("vsrc", "pos", comp.id, vccPin), wire(comp.id, gndPin, "vsrc", "neg")],
  );
}

// ---- classic private-method access (compile.test.ts's own convention: a
// test-only cast, never a copy or a re-derivation) --------------------------

interface ClassicIcPowerInfo {
  powered: boolean;
  vSupply: number;
  outputResistance: number;
  thresholds: { vil: number; vih: number };
  propagationDelay: number;
}

function classicIcPowerInfo(engine: SimEngine, comp: SimComponent): ClassicIcPowerInfo {
  const e = engine as unknown as {
    _lastX: Float64Array | null;
    _icPowerInfo(c: SimComponent, x: Float64Array): ClassicIcPowerInfo;
  };
  if (!e._lastX) throw new Error("test bug: engine has no solved state yet");
  return e._icPowerInfo(comp, e._lastX);
}

function classicFloatingInput(engine: SimEngine, comp: SimComponent, pinId: string): boolean {
  return (
    engine as unknown as { _shouldTreatOpenPinAsDigitalInput(c: SimComponent, pinId: string): boolean }
  )._shouldTreatOpenPinAsDigitalInput(comp, pinId);
}

function classicIsOpenPin(engine: SimEngine, componentId: string, pinId: string): boolean {
  return (engine as unknown as { _isOpenPin(compId: string, pinId: string): boolean })._isOpenPin(componentId, pinId);
}

// ---- the contract formulas (core/README.md's kernel semantics point 1),
// applied to the COMPILED params — proves the params carry enough
// information to reproduce classic's runtime behaviour at an ARBITRARY vS,
// not just that this module read the same catalog fields classic did. ------

function contractThresholds(params: readonly number[], vS: number): { vil: number; vih: number } {
  const famFlags = params[CORE_IC_PARAM.famFlags]!;
  if ((famFlags & CORE_IC_FAM_CMOS_RATIO) !== 0) {
    return { vil: Math.max(0.2, 0.3 * vS), vih: Math.max(0.4, 0.7 * vS) };
  }
  const vilFixed = params[CORE_IC_PARAM.vilFixed] === 1;
  const vihFixed = params[CORE_IC_PARAM.vihFixed] === 1;
  return {
    vil: vilFixed ? params[CORE_IC_PARAM.vil]! : Math.max(0.8, 0.3 * vS),
    vih: vihFixed ? params[CORE_IC_PARAM.vih]! : Math.max(2.0, 0.6 * vS),
  };
}

function contractOutputResistance(params: readonly number[], vSupply: number): number {
  const ioMax = params[CORE_IC_PARAM.ioMax]!;
  if (ioMax > 0 && vSupply > 0) {
    return Math.max(GATE_R_OUT, Math.min(1000, vSupply / (4 * ioMax)));
  }
  return GATE_R_OUT;
}

afterAll(() => resetPartLibrary());

// ---- per-kind: params, thresholds, R_out, pin flags ------------------------

describe.each(IC_KINDS)("%s — every default-parts.json part", (kind) => {
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

      it("compiled — one device of the right kind code, with a 13-value param block", () => {
        expect(dev).toBeDefined();
        expect(dev.params).toHaveLength(13);
      });

      it("vccMin/vccMax match specsForComponent's vcc_range", () => {
        expect(dev.params[CORE_IC_PARAM.vccMin]).toBe(specs?.vcc_range?.min ?? 2.0);
        expect(dev.params[CORE_IC_PARAM.vccMax]).toBe(specs?.vcc_range?.max ?? 18.0);
      });

      it("famFlags matches specsForComponent's logic_family substrings", () => {
        const family = specs?.logic_family ?? "";
        let expected = 0;
        if (family.includes("TTL")) expected |= CORE_IC_FAM_TTL;
        if (family.includes("HC-CMOS") || family.includes("CMOS-4000")) expected |= CORE_IC_FAM_CMOS_RATIO;
        expect(dev.params[CORE_IC_PARAM.famFlags]).toBe(expected);
      });

      it("nominal/ioMax/delay match classic's own field reads and propagationDelayForComponent", () => {
        expect(dev.params[CORE_IC_PARAM.nominal]).toBe(specs?.vcc_range?.nominal ?? 5);
        const ioMaxRaw = specs?.io_max;
        expect(dev.params[CORE_IC_PARAM.ioMax]).toBe(ioMaxRaw != null && ioMaxRaw > 0 ? ioMaxRaw : 0);
        expect(dev.params[CORE_IC_PARAM.delay]).toBe(propagationDelayForComponent(comp));
      });

      it("thresholds (vil/vih, vS=5 and vS=3.3): the contract formula applied to the compiled params matches thresholdsFor exactly", () => {
        for (const vS of [5, 3.3]) {
          expect(contractThresholds(dev.params, vS)).toEqual(thresholdsFor(specs, vS));
        }
      });

      it("schmitt/vtPlus/vtMinus match specsForComponent's v_input.v_t_plus presence", () => {
        const vtPlusRaw = specs?.v_input?.v_t_plus;
        if (vtPlusRaw != null) {
          expect(dev.params[CORE_IC_PARAM.schmitt]).toBe(1);
          expect(dev.params[CORE_IC_PARAM.vtPlus]).toBe(vtPlusRaw);
          expect(dev.params[CORE_IC_PARAM.vtMinus]).toBe(specs?.v_input?.v_t_minus ?? 1.6);
        } else {
          expect(dev.params).toMatchObject({
            [CORE_IC_PARAM.schmitt]: 0,
            [CORE_IC_PARAM.vtPlus]: 0,
            [CORE_IC_PARAM.vtMinus]: 0,
          });
        }
      });

      it("R_out: the contract formula applied to the compiled params matches classic's _icPowerInfo at the loaded vSupply", () => {
        const power = classicIcPowerInfo(engine, comp);
        expect(power.vSupply).toBe(5); // an ideal source across vcc/gnd with nothing else drawing current
        expect(contractOutputResistance(dev.params, power.vSupply)).toBe(power.outputResistance);
      });

      it("every logic pin's FLOATING_INPUT blob flag matches classic's _shouldTreatOpenPinAsDigitalInput", () => {
        icPinMap(kind).forEach((p, i) => {
          const flags = dev.blob[1 + i]!; // blob[0] is the slot base
          const expected = classicFloatingInput(engine, comp, p.id);
          expect((flags & CORE_IC_PIN_FLOATING_INPUT) !== 0).toBe(expected);
        });
      });

      it("every logic pin's OPEN_COLLECTOR blob flag matches the catalog's own pin function", () => {
        icPinMap(kind).forEach((p, i) => {
          const flags = dev.blob[1 + i]!;
          const catalogFn = partDef.pin_layout.find((pl) => pl.id === p.id)?.function;
          expect((flags & CORE_IC_PIN_OPEN_COLLECTOR) !== 0).toBe(catalogFn === "open_collector");
        });
      });

      it("every pin record's OPEN/OPEN_HIGH flags match classic's _isOpenPin / IC_OPEN_HIGH_PINS", () => {
        const { vccPin, gndPin } = supplyPinsForComponent(comp);
        const allPinIds = [vccPin!, gndPin!, ...icPinMap(kind).map((p) => p.id)];
        dev.pins.forEach((pinRecord, i) => {
          const pinId = allPinIds[i]!;
          expect(pinRecord.open).toBe(classicIsOpenPin(engine, "u1", pinId));
          expect(pinRecord.openHigh).toBe(IC_OPEN_HIGH_PINS[kind]?.has(pinId) ?? false);
        });
      });
    });
  }
});

// ---- FLOATING_INPUT_FUNCTIONS sanity: the set this module reuses is
// classic's real one, not a stale re-export ---------------------------------

describe("FLOATING_INPUT_FUNCTIONS", () => {
  it("is the exact set classic's own _shouldTreatOpenPinAsDigitalInput reads (sim-engine.ts)", () => {
    expect(FLOATING_INPUT_FUNCTIONS).toEqual(
      new Set(["input", "clock", "reset", "enable", "load", "io", "tri_state"]),
    );
  });
});

// ---- digital slot table ----------------------------------------------------

describe("digital slot table", () => {
  it("assigns consecutive slots in component order, keyed componentId/pinId, in icPinMap output-pin order", () => {
    const p00 = partsFor("74ls00")[0]!;
    const u1 = icComponent("u1", p00);
    const u2 = icComponent("u2", p00);
    const c = circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), u1, u2],
      [
        wire("vsrc", "pos", "u1", "vcc"), wire("u1", "gnd", "vsrc", "neg"),
        wire("vsrc", "pos", "u2", "vcc"), wire("u2", "gnd", "vsrc", "neg"),
      ],
    );
    const compiled = compileOk(c);
    expect(compiled.digitalSlots).toEqual([
      "u1/1y", "u1/2y", "u1/3y", "u1/4y",
      "u2/1y", "u2/2y", "u2/3y", "u2/4y",
    ]);

    const decoded = decodeProgram(compiled.program);
    expect(decoded.digitalSlotCount).toBe(8);
    const devU1 = decoded.devices.find((d) => d.componentIndex === 1)!;
    const devU2 = decoded.devices.find((d) => d.componentIndex === 2)!;
    expect(devU1.blob[0]).toBe(0);
    expect(devU2.blob[0]).toBe(4);
  });

  it("74ls245: every a and b pin publishes a slot (16), not just the b (output-role) pins", () => {
    const p245 = partsFor("74ls245")[0]!;
    const u1 = icComponent("u1", p245);
    const c = poweredIcCircuit(u1, 5);
    const compiled = compileOk(c);
    expect(compiled.digitalSlots).toEqual([
      "u1/a1", "u1/b1", "u1/a2", "u1/b2", "u1/a3", "u1/b3", "u1/a4", "u1/b4",
      "u1/a5", "u1/b5", "u1/a6", "u1/b6", "u1/a7", "u1/b7", "u1/a8", "u1/b8",
    ]);
  });
});

// ---- round trip through format.ts's decoder, with slot-range rejection ----

describe("round trip through format.ts's decoder", () => {
  it("a compiled IC program decodes cleanly, with blob[0] + digitalSlots within S", () => {
    const p00 = partsFor("74ls00")[0]!;
    const c = poweredIcCircuit(icComponent("u1", p00), 5);
    const compiled = compileOk(c);
    expect(() => decodeProgram(compiled.program)).not.toThrow();
    const decoded = decodeProgram(compiled.program);
    expect(decoded.digitalSlotCount).toBe(4);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE["74ls00"])!;
    expect(dev.blob[0]! + 4).toBeLessThanOrEqual(decoded.digitalSlotCount);
  });

  it("decodeProgram rejects a real compiled program whose header digitalSlotCount was shrunk below a device's own slot range", () => {
    const p00 = partsFor("74ls00")[0]!;
    const c = poweredIcCircuit(icComponent("u1", p00), 5);
    const compiled = compileOk(c);

    // The header's digitalSlotCount lives right after the option pairs
    // (format.ts's ProgramWriter layout: MAGIC, VERSION, nodeCount,
    // deviceCount, optionCount, (key,value) x optionCount,
    // digitalSlotCount, channelSlotCount, ...) — computed from the program
    // itself, not a hardcoded offset, so this test survives an unrelated
    // change to the option block.
    const optionCount = compiled.program[4]!;
    const digitalSlotCountIndex = 5 + 2 * optionCount;
    expect(compiled.program[digitalSlotCountIndex]).toBe(4); // one 74ls00, 4 output pins

    const corrupted = Float64Array.from(compiled.program);
    corrupted[digitalSlotCountIndex] = 3; // shrink S to below the device's own [0, 4) slot range
    expect(() => decodeProgram(corrupted)).toThrow(/digital slots \[0, 4\) exceed digitalSlotCount 3/);
  });
});

// ---- declines ---------------------------------------------------------------

describe("declines (topology:ic-pins:<componentId>)", () => {
  it("declines when the component's declared pins are missing one of the kind's logic pins", () => {
    const p00 = partsFor("74ls00")[0]!;
    const comp = icComponent("u1", p00);
    comp.pins = comp.pins.filter((p) => p.id !== "4b");
    const c = poweredIcCircuit(comp, 5);
    const result = compileCircuit(c, WITH_ICS);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
  });

  it("declines when the component declares an extra, unexpected pin", () => {
    const p00 = partsFor("74ls00")[0]!;
    const comp = icComponent("u1", p00);
    comp.pins = [...comp.pins, { id: "bogus" }];
    const c = poweredIcCircuit(comp, 5);
    const result = compileCircuit(c, WITH_ICS);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
  });

  it("declines when the component's pins are exactly the logic pins but neither vcc nor gnd", () => {
    const p00 = partsFor("74ls00")[0]!;
    const comp = icComponent("u1", p00);
    comp.pins = comp.pins.filter((p) => p.id !== "vcc" && p.id !== "gnd");
    const c = circuit([comp]);
    const result = compileCircuit(c, WITH_ICS);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
  });

  it("declines when no catalog candidate resolves at all (classic's supply-less path)", () => {
    setPartLibrary([]);
    try {
      const comp: SimComponent = { id: "u1", kind: "74ls00", pins: [{ id: "vcc" }, { id: "gnd" }], params: {} };
      const result = compileCircuit(circuit([comp]), WITH_ICS);
      expect(result.ok).toBe(false);
      if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
    } finally {
      resetPartLibrary();
    }
  });
});
