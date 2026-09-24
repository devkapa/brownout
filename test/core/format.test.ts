import { describe, expect, it } from "vitest";
import {
  CORE_FORMAT_MAGIC,
  CORE_FORMAT_VERSION,
  CORE_KIND_CODE,
  CORE_KIND_NAME_BY_CODE,
  decodeProgram,
  ProgramWriter,
} from "../../src/core/format.js";

// A minimal, valid one-resistor program (nodeCount 2, one resistor from node
// 0 to node 1, R = 500). Used as a baseline that "each decode validation
// error" tests mutate one field of at a time. Its layout is fixed by
// ProgramWriter's push order — the offsets below (0..12) are documented as
// each test uses them, so a change to that order is visible here.
function buildBaselineProgram(): Float64Array {
  const writer = new ProgramWriter(2);
  writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 0, pins: [0, 1], params: [500] });
  return writer.finish();
}

function mutated(base: Float64Array, index: number, value: number): Float64Array {
  const copy = Float64Array.from(base);
  copy[index] = value;
  return copy;
}

describe("CORE_FORMAT_MAGIC / CORE_FORMAT_VERSION", () => {
  it("match the contract in core/README.md", () => {
    // core/README.md: `MAGIC` = `0x424F4331` ("BOC1"), `VERSION` = `1`.
    expect(CORE_FORMAT_MAGIC).toBe(0x424f4331);
    expect(CORE_FORMAT_VERSION).toBe(1);
  });
});

describe("CORE_KIND_CODE / CORE_KIND_NAME_BY_CODE", () => {
  it("assigns the contract's codes", () => {
    expect(CORE_KIND_CODE.resistor).toBe(1);
    expect(CORE_KIND_CODE.voltage_source).toBe(2);
  });

  it("reverse-looks-up every declared code, and nothing else", () => {
    expect(CORE_KIND_NAME_BY_CODE.get(1)).toBe("resistor");
    expect(CORE_KIND_NAME_BY_CODE.get(2)).toBe("voltage_source");
    expect(CORE_KIND_NAME_BY_CODE.get(0)).toBeUndefined();
    expect(CORE_KIND_NAME_BY_CODE.get(99)).toBeUndefined();
  });
});

describe("ProgramWriter -> decodeProgram round trip", () => {
  it("round-trips a multi-device program, including a non-empty blob", () => {
    const writer = new ProgramWriter(4);
    writer.addDevice({ kindCode: CORE_KIND_CODE.voltage_source, componentIndex: 0, pins: [1, 0], params: [9] });
    writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 1, pins: [1, 2], params: [220] });
    writer.addDevice({
      kindCode: CORE_KIND_CODE.resistor,
      componentIndex: 2,
      pins: [2, 3],
      params: [330],
      blob: [42, 43],
    });

    const decoded = decodeProgram(writer.finish());

    expect(decoded).toEqual({
      version: 1,
      nodeCount: 4,
      devices: [
        { kindCode: 2, componentIndex: 0, pins: [1, 0], params: [9], blob: [] },
        { kindCode: 1, componentIndex: 1, pins: [1, 2], params: [220], blob: [] },
        { kindCode: 1, componentIndex: 2, pins: [2, 3], params: [330], blob: [42, 43] },
      ],
    });
  });

  it("round-trips a program with zero devices", () => {
    const writer = new ProgramWriter(1);
    const decoded = decodeProgram(writer.finish());
    expect(decoded).toEqual({ version: 1, nodeCount: 1, devices: [] });
  });
});

describe("ProgramWriter's exact byte layout — golden array", () => {
  // A 2-resistor divider fed by a 10 V source: node 0 = the negative rail
  // (gnd/datum), node 1 = the source's positive terminal / R1's top, node 2
  // = the midpoint between R1 and R2. Components in order [source, R1, R2].
  //
  // This array is hand-written, NOT computed from CORE_FORMAT_MAGIC or
  // CORE_KIND_CODE, so a change to those constants that silently breaks the
  // contract's actual layout would still be caught here.
  it("matches a hand-computed Float64Array", () => {
    const writer = new ProgramWriter(3);
    writer.addDevice({ kindCode: CORE_KIND_CODE.voltage_source, componentIndex: 0, pins: [1, 0], params: [10] });
    writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 1, pins: [1, 2], params: [1000] });
    writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 2, pins: [2, 0], params: [1000] });

    const program = writer.finish();

    // prettier-ignore
    const expected = Float64Array.from([
      0x424f4331, 1, 3, 3,                 // header: MAGIC, VERSION, nodeCount=3, deviceCount=3
      2, 0, 2, 1, 0, 1, 10, 0,             // device 0: voltage_source, comp 0, 2 pins (1,0), 1 param (10 V), 0 blob
      1, 1, 2, 1, 2, 1, 1000, 0,           // device 1: resistor, comp 1, 2 pins (1,2), 1 param (1000 ohm), 0 blob
      1, 2, 2, 2, 0, 1, 1000, 0,           // device 2: resistor, comp 2, 2 pins (2,0), 1 param (1000 ohm), 0 blob
      0x424f4331,                          // trailer: MAGIC
    ]);

    expect(program).toEqual(expected);
  });
});

describe("ProgramWriter validation", () => {
  it("rejects a nodeCount below 1", () => {
    expect(() => new ProgramWriter(0)).toThrow(/nodeCount/);
    expect(() => new ProgramWriter(-1)).toThrow(/nodeCount/);
  });

  it("rejects a non-integer nodeCount", () => {
    expect(() => new ProgramWriter(1.5)).toThrow(/integer/);
  });

  it("rejects an unknown kind code", () => {
    const writer = new ProgramWriter(2);
    expect(() => writer.addDevice({ kindCode: 99, componentIndex: 0, pins: [0, 1], params: [1] })).toThrow(
      /unknown kind code/,
    );
  });

  it("rejects the wrong pin count for a kind", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 0, pins: [0, 1, 1], params: [1] }),
    ).toThrow(/pin\(s\)/);
  });

  it("rejects the wrong param count for a kind", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 0, pins: [0, 1], params: [1, 2] }),
    ).toThrow(/param\(s\)/);
  });

  it("rejects a pin node outside [0, nodeCount)", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 0, pins: [0, 2], params: [1] }),
    ).toThrow(/out of range/);
  });

  it("rejects a negative componentIndex", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: -1, pins: [0, 1], params: [1] }),
    ).toThrow(/componentIndex/);
  });

  it("rejects addDevice after finish()", () => {
    const writer = new ProgramWriter(2);
    writer.finish();
    expect(() =>
      writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 0, pins: [0, 1], params: [1] }),
    ).toThrow(/after finish/);
  });

  it("rejects a second finish()", () => {
    const writer = new ProgramWriter(2);
    writer.finish();
    expect(() => writer.finish()).toThrow(/already finished/);
  });
});

describe("decodeProgram validation — each check the contract requires", () => {
  it("rejects an empty program (truncated header)", () => {
    expect(() => decodeProgram(new Float64Array([]))).toThrow(/truncated/);
  });

  it("rejects a bad header magic", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 0, 0))).toThrow(/header magic/);
  });

  it("rejects an unsupported version", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 1, 2))).toThrow(/version/);
  });

  it("rejects nodeCount below 1", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 2, 0))).toThrow(/nodeCount/);
  });

  it("rejects a negative nodeCount", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 2, -1))).toThrow(/non-negative integer/);
  });

  it("rejects a non-integer nodeCount", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 2, 1.5))).toThrow(/non-negative integer/);
  });

  it("rejects a negative deviceCount", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 3, -1))).toThrow(/non-negative integer/);
  });

  it("rejects an unknown kind code", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 4, 99))).toThrow(/unknown kind code/);
  });

  it("rejects a negative componentIndex", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 5, -1))).toThrow(/non-negative integer/);
  });

  it("rejects the wrong pinCount for a kind", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 6, 3))).toThrow(/pinCount 3, expected 2/);
  });

  it("rejects a negative pin node index", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 7, -1))).toThrow(/non-negative integer/);
  });

  it("rejects a pin node index >= nodeCount", () => {
    // Baseline nodeCount is 2; index 7 is the resistor's first pin.
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 7, 5))).toThrow(/out of range for nodeCount 2/);
  });

  it("rejects the wrong paramCount for a kind", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 9, 2))).toThrow(/paramCount 2, expected 1/);
  });

  it("rejects a non-finite param", () => {
    // Offset 10 is the baseline resistor's R.
    for (const bad of [Number.NaN, Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY]) {
      expect(() => decodeProgram(mutated(buildBaselineProgram(), 10, bad))).toThrow(/params\[0\] must be finite/);
    }
  });

  it("rejects a bad trailer magic", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 12, 0))).toThrow(/trailer magic/);
  });

  it("rejects trailing data after the trailer", () => {
    const base = buildBaselineProgram();
    const withTrailingValue = Float64Array.from([...base, 0]);
    expect(() => decodeProgram(withTrailingValue)).toThrow(/trailing/);
  });

  it("rejects a program truncated mid-record", () => {
    const base = buildBaselineProgram();
    expect(() => decodeProgram(base.slice(0, base.length - 1))).toThrow(/truncated/);
  });
});
