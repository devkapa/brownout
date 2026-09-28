/**
 * Unit coverage for compile/memories.ts: the 3 W1d memory ICs (core/
 * README.md's "Kind codes and records", codes 80-82: 74ls189, 28c16,
 * 28c256) compiled to v2 program records.
 *
 * Follows compile-ics.test.ts/compile-registers.test.ts's own conventions
 * (own local helpers, expected values read from classic's own exported/
 * private helpers, never re-derived). These 3 kinds are not yet in
 * CORE_IMPLEMENTED_KINDS, so every `compileCircuit` call here passes a
 * widened `implementedKinds` set.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import {
  eepromImageBytes,
  propagationDelayForComponent,
  SimEngine,
  specsForComponent,
  supplyPinsForComponent,
} from "../../src/sim/engine/sim-engine.js";
import { IC_OPEN_HIGH_PINS } from "../../src/sim/engine/digital.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { resolveDelayTail } from "../../src/core/compile/delays.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import {
  CORE_EEPROM_PARAM,
  CORE_EEPROM_SIZE,
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

const MEMORY_KINDS = ["74ls189", "28c16", "28c256"] as const;

function addressPinIds(count: number): string[] {
  return Array.from({ length: count }, (_, i) => `a${i}`);
}
function ioPinIds(): string[] {
  return Array.from({ length: 8 }, (_, i) => `io${i}`);
}

/** The README's fixed pin order, hardcoded independently of compile/memories.ts's own copy. */
const MEMORY_LOGIC_PINS: Readonly<Record<(typeof MEMORY_KINDS)[number], readonly string[]>> = {
  "74ls189": ["a0", "a1", "a2", "a3", "d1", "d2", "d3", "d4", "/cs", "/we", "/o1", "/o2", "/o3", "/o4"],
  "28c16": [...addressPinIds(11), "/oe", "/ce", "/we", ...ioPinIds()],
  "28c256": [...addressPinIds(15), "/oe", "/ce", "/we", ...ioPinIds()],
};

const WITH_MEMORIES = new Set<string>([...CORE_IMPLEMENTED_KINDS, ...MEMORY_KINDS]);

function compileOk(c: SimCircuit) {
  const result = compileCircuit(c, WITH_MEMORIES);
  if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
  return result.compiled;
}

function partsFor(kind: string): PartDefinition[] {
  return BUNDLED_DEFAULT_PARTS.filter((p) => p.kind === kind);
}

function icComponent(id: string, partDef: PartDefinition, params: Record<string, number | string> = {}): SimComponent {
  return { id, kind: partDef.kind, catalogUid: partDef.uid, pins: partDef.pin_layout.map((p) => ({ id: p.id })), params };
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

/** Independent little-endian word decode (DataView, not the same arithmetic memories.ts's own romWords uses) — for cross-checking the compiled blob's packing. */
function wordsToBytesLE(words: readonly number[]): Uint8Array {
  const bytes = new Uint8Array(words.length * 4);
  const view = new DataView(bytes.buffer);
  words.forEach((w, k) => view.setUint32(k * 4, w, true));
  return bytes;
}

afterAll(() => resetPartLibrary());

// ---- per-kind: params, delay tail, pin order/flags, no OPEN_COLLECTOR -----

describe.each(MEMORY_KINDS)("%s — every default-parts.json part", (kind) => {
  const logicPins = MEMORY_LOGIC_PINS[kind];
  const isEeprom = kind !== "74ls189";

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

      it("compiled — one device, pin count = 2 supply + L logic pins", () => {
        expect(dev).toBeDefined();
        expect(dev.pins).toHaveLength(2 + logicPins.length);
      });

      it("param count = 13 + 2*pairs, plus 2 (tWC, sdp) for the EEPROMs only", () => {
        const expected = 13 + 2 * CORE_IC_DELAY_PAIRS[kind]!.length + (isEeprom ? 2 : 0);
        expect(dev.params).toHaveLength(expected);
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

      it("params 7 (schmitt)/8 (vtPlus)/9 (vtMinus) are 0 whatever the catalog says", () => {
        expect(dev.params[CORE_IC_PARAM.schmitt]).toBe(0);
        expect(dev.params[CORE_IC_PARAM.vtPlus]).toBe(0);
        expect(dev.params[CORE_IC_PARAM.vtMinus]).toBe(0);
      });

      it("the delay tail (params[13..13+2*pairs)) is exactly resolveDelayTail(kind, specs, params[12])", () => {
        const pairs = CORE_IC_DELAY_PAIRS[kind]!.length;
        const expectedTail = resolveDelayTail(kind, specs, dev.params[CORE_IC_PARAM.delay]!);
        expect(dev.params.slice(13, 13 + 2 * pairs)).toEqual(expectedTail);
      });

      if (isEeprom) {
        it("tWC is classic's T_WC (10e-3) always", () => {
          expect(dev.params[CORE_EEPROM_PARAM.tWC]).toBe(10e-3);
        });
      }

      it("pin order matches the README's fixed table, by id, after vcc/gnd; openHigh is always false (no IC_OPEN_HIGH_PINS entry for this kind)", () => {
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

// ---- sdp: 28c256 reads sdpBypass, 28c16 never does ------------------------

describe("sdp param", () => {
  it("28c256: sdp = 1 when sdpBypass is absent or 0, 0 when sdpBypass is any other truthy-ish number/string", () => {
    const partDef = partsFor("28c256")[0]!;
    for (const [sdpBypass, expected] of [
      [undefined, 1],
      [0, 1],
      ["0", 1],
      [1, 0],
      ["1", 0],
      [2, 0],
    ] as const) {
      const params: Record<string, number | string> = {};
      if (sdpBypass !== undefined) params.sdpBypass = sdpBypass;
      const comp = icComponent("u1", partDef, params);
      const compiled = compileOk(poweredIcCircuit(comp, 5));
      const decoded = decodeProgram(compiled.program);
      const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE["28c256"])!;
      expect(dev.params[CORE_EEPROM_PARAM.sdp], `sdpBypass=${String(sdpBypass)}`).toBe(expected);
    }
  });

  it("28c16: sdp is always 0, whatever sdpBypass says (the flag is 28c256-only)", () => {
    const partDef = partsFor("28c16")[0]!;
    for (const sdpBypass of [undefined, 0, 1, "0", "1"] as const) {
      const params: Record<string, number | string> = {};
      if (sdpBypass !== undefined) params.sdpBypass = sdpBypass;
      const comp = icComponent("u1", partDef, params);
      const compiled = compileOk(poweredIcCircuit(comp, 5));
      const decoded = decodeProgram(compiled.program);
      const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE["28c16"])!;
      expect(dev.params[CORE_EEPROM_PARAM.sdp], `sdpBypass=${String(sdpBypass)}`).toBe(0);
    }
  });
});

// ---- ROM image --------------------------------------------------------------

describe.each(["28c16", "28c256"] as const)("%s — ROM image", (kind) => {
  const size = CORE_EEPROM_SIZE[kind]!;
  const wordCount = size / 4;
  const logicPins = MEMORY_LOGIC_PINS[kind];

  function romWordsOf(comp: SimComponent): number[] {
    const compiled = compileOk(poweredIcCircuit(comp, 5));
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE[kind])!;
    return dev.blob.slice(1 + logicPins.length);
  }

  it("empty contents: every word is 0xFFFFFFFF", () => {
    const partDef = partsFor(kind)[0]!;
    const comp = icComponent("u1", partDef, {});
    const words = romWordsOf(comp);
    expect(words).toHaveLength(wordCount);
    expect(words.every((w) => w === 0xffffffff)).toBe(true);
  });

  it("a short image: the given bytes lead, the rest of the image is 0xFF", () => {
    const partDef = partsFor(kind)[0]!;
    const rawBytes = Uint8Array.from([0x01, 0x02, 0x03, 0x04, 0x05]);
    const b64 = Buffer.from(rawBytes).toString("base64");
    const comp = icComponent("u1", partDef, { contents: b64 });
    const words = romWordsOf(comp);
    const bytesBack = wordsToBytesLE(words);
    expect(bytesBack.subarray(0, 5)).toEqual(rawBytes);
    expect(bytesBack.subarray(5).every((b) => b === 0xff)).toBe(true);
  });

  it("invalid base64: atob throws, every word is 0xFFFFFFFF (classic's own catch-and-keep-0xff behaviour)", () => {
    const partDef = partsFor(kind)[0]!;
    const comp = icComponent("u1", partDef, { contents: "not-valid-base64!!!" });
    const words = romWordsOf(comp);
    expect(words.every((w) => w === 0xffffffff)).toBe(true);
  });

  it("a lenient base64 string (whitespace) decodes to the same bytes classic's own eepromImageBytes loads", () => {
    const partDef = partsFor(kind)[0]!;
    const rawBytes = Uint8Array.from({ length: 16 }, (_, i) => i * 7 + 1);
    const cleanB64 = Buffer.from(rawBytes).toString("base64");
    const lenientB64 = `${cleanB64.slice(0, 4)}\n${cleanB64.slice(4)}`; // atob tolerates embedded whitespace
    const comp = icComponent("u1", partDef, { contents: lenientB64 });
    const words = romWordsOf(comp);
    const bytesBack = wordsToBytesLE(words);
    expect(bytesBack).toEqual(eepromImageBytes(size, lenientB64));
  });

  it("word packing is little-endian, checked against eepromImageBytes directly (DataView cross-check, independent of memories.ts's own arithmetic)", () => {
    const partDef = partsFor(kind)[0]!;
    const rawBytes = Uint8Array.from({ length: 32 }, (_, i) => (i * 31 + 5) & 0xff);
    const b64 = Buffer.from(rawBytes).toString("base64");
    const comp = icComponent("u1", partDef, { contents: b64 });
    const words = romWordsOf(comp);
    const expectedBytes = eepromImageBytes(size, b64);
    expect(wordsToBytesLE(words)).toEqual(expectedBytes);
    // And every word value itself is < 2^32 and an exact integer, as the
    // wire format requires (decodeProgram already checked this implicitly by
    // not throwing, but assert it explicitly too).
    words.forEach((w) => {
      expect(Number.isInteger(w)).toBe(true);
      expect(w).toBeGreaterThanOrEqual(0);
      expect(w).toBeLessThan(2 ** 32);
    });
  });
});

it("74ls189 compiles no ROM words at all — its blob is exactly [slotBase, pinFlags x 14]", () => {
  const partDef = partsFor("74ls189")[0]!;
  const comp = icComponent("u1", partDef);
  const compiled = compileOk(poweredIcCircuit(comp, 5));
  const decoded = decodeProgram(compiled.program);
  const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE["74ls189"])!;
  expect(dev.blob).toHaveLength(1 + 14);
});

// ---- digital slot keys ------------------------------------------------------

describe("digital slot keys", () => {
  it("74ls189: <id>//o1 .. <id>//o4 (the double slash is the pin id's own leading slash)", () => {
    const partDef = partsFor("74ls189")[0]!;
    const comp = icComponent("u1", partDef);
    const compiled = compileOk(poweredIcCircuit(comp, 5));
    expect(compiled.digitalSlots).toEqual(["u1//o1", "u1//o2", "u1//o3", "u1//o4"]);
  });

  it("28c16/28c256: <id>/io0 .. <id>/io7", () => {
    for (const kind of ["28c16", "28c256"] as const) {
      const partDef = partsFor(kind)[0]!;
      const comp = icComponent("u1", partDef);
      const compiled = compileOk(poweredIcCircuit(comp, 5));
      expect(compiled.digitalSlots).toEqual(Array.from({ length: 8 }, (_, i) => `u1/io${i}`));
    }
  });
});

// ---- round trip through format.ts's decoder --------------------------------

describe("round trip through format.ts's decoder", () => {
  it("a compiled 28c16 program decodes cleanly, with blob[0] + digitalSlots within S and the exact blob length (1 + 22 + 512)", () => {
    const partDef = partsFor("28c16")[0]!;
    const c = poweredIcCircuit(icComponent("u1", partDef), 5);
    const compiled = compileOk(c);
    expect(() => decodeProgram(compiled.program)).not.toThrow();
    const decoded = decodeProgram(compiled.program);
    expect(decoded.digitalSlotCount).toBe(8);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE["28c16"])!;
    expect(dev.blob).toHaveLength(1 + 22 + 512);
    expect(dev.blob[0]! + 8).toBeLessThanOrEqual(decoded.digitalSlotCount);
  });
});

// ---- declines ---------------------------------------------------------------

describe.each(MEMORY_KINDS)("%s — declines (topology:ic-pins:<componentId>)", (kind) => {
  it("declines when the component's declared pins are missing one of the kind's logic pins", () => {
    const partDef = partsFor(kind)[0]!;
    const comp = icComponent("u1", partDef);
    const lastLogicPin = MEMORY_LOGIC_PINS[kind][MEMORY_LOGIC_PINS[kind].length - 1]!;
    comp.pins = comp.pins.filter((p) => p.id !== lastLogicPin);
    const c = poweredIcCircuit(comp, 5);
    const result = compileCircuit(c, WITH_MEMORIES);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
  });

  it("declines when the component declares an extra, unexpected pin", () => {
    const partDef = partsFor(kind)[0]!;
    const comp = icComponent("u1", partDef);
    comp.pins = [...comp.pins, { id: "bogus" }];
    const c = poweredIcCircuit(comp, 5);
    const result = compileCircuit(c, WITH_MEMORIES);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
  });

  it("declines when the component's pins are exactly the logic pins but neither vcc nor gnd", () => {
    const partDef = partsFor(kind)[0]!;
    const comp = icComponent("u1", partDef);
    comp.pins = comp.pins.filter((p) => p.id !== "vcc" && p.id !== "gnd");
    const c = circuit([comp]);
    const result = compileCircuit(c, WITH_MEMORIES);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
  });

  it("declines when no catalog candidate resolves at all (classic's supply-less path)", () => {
    setPartLibrary([]);
    try {
      const comp: SimComponent = { id: "u1", kind, pins: [{ id: "vcc" }, { id: "gnd" }], params: {} };
      const result = compileCircuit(circuit([comp]), WITH_MEMORIES);
      expect(result.ok).toBe(false);
      if (!result.ok) expect(result.reasons).toEqual(["topology:ic-pins:u1"]);
    } finally {
      resetPartLibrary();
    }
  });
});
