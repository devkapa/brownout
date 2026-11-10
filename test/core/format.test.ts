import { describe, expect, it } from "vitest";
import {
  CORE_FORMAT_MAGIC,
  CORE_FORMAT_VERSION,
  CORE_KIND_CODE,
  CORE_KIND_NAME_BY_CODE,
  CORE_OPTION_KEYS,
  decodeProgram,
  ProgramWriter,
  type ProgramPinInput,
} from "../../src/core/format.js";
import { CORE_KIND_CODES } from "../../src/core/kinds.js";

/** A closed pin record with arbitrary (but in-range) hash placeholders — format.ts doesn't care where a hash comes from, only that it's a valid uint32; the real FNV-1a values are compile/topology.ts's job and are golden-vector-tested in compile.test.ts. */
function pin(node: number, overrides: Partial<ProgramPinInput> = {}): ProgramPinInput {
  return { node, open: false, openHigh: false, openHash: 0x1111, deadHash: 0x2222, ...overrides };
}

// A minimal, valid one-resistor program: nodeCount 2, one AMBIENT_C option,
// one resistor from node 0 (closed) to node 1 (open), R = 500, no readout
// slots. Used as a baseline that "each decode validation error" tests mutate
// one field of at a time. Its layout is fixed by ProgramWriter's push order —
// the offsets below (0..23) are documented as each test uses them, so a
// change to that order is visible here.
//
//  0 MAGIC        5 option0.key=AMBIENT_C   12 pin0.node=0   17 pin1.flags=OPEN
//  1 VERSION      6 option0.value=25        13 pin0.flags=0  18 pin1.openHash
//  2 nodeCount=2  7 digitalSlotCount=0      14 pin0.openHash 19 pin1.deadHash
//  3 deviceCount=1 8 channelSlotCount=0     15 pin0.deadHash 20 paramCount=1
//  4 optionCount=1 9 kindCode=resistor(1)   16 pin1.node=1   21 param0=500
//                 10 componentIndex=0                        22 blobCount=0
//                 11 pinCount=2                               23 trailer MAGIC
function buildBaselineProgram(): Float64Array {
  const writer = new ProgramWriter(2, [[CORE_OPTION_KEYS.AMBIENT_C, 25]]);
  writer.addDevice({
    kindCode: CORE_KIND_CODE.resistor,
    componentIndex: 0,
    pins: [pin(0, { openHash: 0x1111, deadHash: 0x2222 }), pin(1, { open: true, openHash: 0x3333, deadHash: 0x4444 })],
    params: [500],
  });
  return writer.finish(0, 0);
}

function mutated(base: Float64Array, index: number, value: number): Float64Array {
  const copy = Float64Array.from(base);
  copy[index] = value;
  return copy;
}

describe("CORE_FORMAT_MAGIC / CORE_FORMAT_VERSION", () => {
  it("match the contract in core/README.md", () => {
    // core/README.md: `MAGIC` = `0x424F4331` ("BOC1"), `VERSION` = `3`.
    expect(CORE_FORMAT_MAGIC).toBe(0x424f4331);
    expect(CORE_FORMAT_VERSION).toBe(3);
  });
});

describe("CORE_OPTION_KEYS", () => {
  it("matches the contract's Options table", () => {
    expect(CORE_OPTION_KEYS).toEqual({
      INTEGRATION: 1,
      NO_SOLUTION_CACHE: 2,
      NO_RAIL_SHORTCUT: 3,
      FORCE_STEPPED: 4,
      AMBIENT_C: 5,
      FLOATING_MODEL: 6,
      START_TIME: 7,
    });
  });
});

describe("CORE_KIND_CODE / CORE_KIND_NAME_BY_CODE — v1 aliases for kinds.ts's table", () => {
  it("CORE_KIND_CODE is exactly kinds.ts's CORE_KIND_CODES (not a stale copy)", () => {
    expect(CORE_KIND_CODE).toBe(CORE_KIND_CODES);
    expect(CORE_KIND_CODE.resistor).toBe(1);
    expect(CORE_KIND_CODE.voltage_source).toBe(2);
  });

  it("reverse-looks-up every declared code, and nothing else", () => {
    expect(CORE_KIND_NAME_BY_CODE.get(1)).toBe("resistor");
    expect(CORE_KIND_NAME_BY_CODE.get(2)).toBe("voltage_source");
    expect(CORE_KIND_NAME_BY_CODE.get(0)).toBeUndefined();
    expect(CORE_KIND_NAME_BY_CODE.get(12345)).toBeUndefined();
  });
});

describe("ProgramWriter -> decodeProgram round trip", () => {
  it("round-trips a multi-device program, including options, flags, hashes and a non-empty blob", () => {
    const writer = new ProgramWriter(4, [
      [CORE_OPTION_KEYS.INTEGRATION, 1],
      [CORE_OPTION_KEYS.AMBIENT_C, 30],
    ]);
    writer.addDevice({
      kindCode: CORE_KIND_CODE.voltage_source,
      componentIndex: 0,
      pins: [pin(1, { openHash: 10, deadHash: 20 }), pin(0, { open: true, openHigh: true, openHash: 30, deadHash: 40 })],
      params: [9],
    });
    writer.addDevice({
      kindCode: CORE_KIND_CODE.resistor,
      componentIndex: 1,
      pins: [pin(1, { openHash: 50, deadHash: 60 }), pin(2, { openHash: 70, deadHash: 80 })],
      params: [220],
    });
    writer.addDevice({
      kindCode: CORE_KIND_CODE.resistor,
      componentIndex: 2,
      pins: [pin(2, { openHash: 90, deadHash: 100 }), pin(3, { openHash: 110, deadHash: 120 })],
      params: [330],
    });

    const decoded = decodeProgram(writer.finish(0, 0));

    expect(decoded).toEqual({
      version: 3,
      nodeCount: 4,
      options: [
        [CORE_OPTION_KEYS.INTEGRATION, 1],
        [CORE_OPTION_KEYS.AMBIENT_C, 30],
      ],
      digitalSlotCount: 0,
      channelSlotCount: 0,
      devices: [
        {
          kindCode: 2,
          componentIndex: 0,
          pins: [
            { node: 1, open: false, openHigh: false, openHash: 10, deadHash: 20 },
            { node: 0, open: true, openHigh: true, openHash: 30, deadHash: 40 },
          ],
          params: [9],
          blob: [],
        },
        {
          kindCode: 1,
          componentIndex: 1,
          pins: [
            { node: 1, open: false, openHigh: false, openHash: 50, deadHash: 60 },
            { node: 2, open: false, openHigh: false, openHash: 70, deadHash: 80 },
          ],
          params: [220],
          blob: [],
        },
        {
          kindCode: 1,
          componentIndex: 2,
          pins: [
            { node: 2, open: false, openHigh: false, openHash: 90, deadHash: 100 },
            { node: 3, open: false, openHigh: false, openHash: 110, deadHash: 120 },
          ],
          params: [330],
          blob: [],
        },
      ],
      carry: [],
    });
  });

  it("round-trips a program with zero devices, zero options and non-zero readout slot counts", () => {
    const writer = new ProgramWriter(1);
    const decoded = decodeProgram(writer.finish(3, 2));
    expect(decoded).toEqual({
      version: 3,
      nodeCount: 1,
      options: [],
      digitalSlotCount: 3,
      channelSlotCount: 2,
      devices: [],
      carry: [],
    });
  });
});

describe("ProgramWriter's exact byte layout — golden array", () => {
  // A 2-resistor divider fed by a 10 V source: node 0 = the negative rail
  // (gnd/datum), node 1 = the source's positive terminal / R1's top, node 2
  // = the midpoint between R1 and R2. Components in order [source, R1, R2].
  // One option (AMBIENT_C=22) and non-zero readout slot counts, to pin the
  // full v3 header shape, not just the v1-era device records.
  //
  // This array is hand-written, NOT computed from CORE_FORMAT_MAGIC or
  // CORE_KIND_CODE, so a change to those constants that silently breaks the
  // contract's actual layout would still be caught here.
  it("matches a hand-computed Float64Array", () => {
    const writer = new ProgramWriter(3, [[CORE_OPTION_KEYS.AMBIENT_C, 22]]);
    writer.addDevice({
      kindCode: CORE_KIND_CODE.voltage_source,
      componentIndex: 0,
      pins: [pin(1, { openHash: 1, deadHash: 2 }), pin(0, { openHash: 3, deadHash: 4 })],
      params: [10],
    });
    writer.addDevice({
      kindCode: CORE_KIND_CODE.resistor,
      componentIndex: 1,
      pins: [pin(1, { openHash: 5, deadHash: 6 }), pin(2, { open: true, openHash: 7, deadHash: 8 })],
      params: [1000],
    });
    writer.addDevice({
      kindCode: CORE_KIND_CODE.resistor,
      componentIndex: 2,
      pins: [pin(2, { openHash: 9, deadHash: 10 }), pin(0, { openHigh: true, openHash: 11, deadHash: 12 })],
      params: [1000],
    });

    const program = writer.finish(0, 5);

    // prettier-ignore
    const expected = Float64Array.from([
      0x424f4331, 3, 3, 3,                        // header: MAGIC, VERSION, nodeCount=3, deviceCount=3
      1, 5, 22,                                   // optionCount=1, (AMBIENT_C, 22)
      0, 5,                                       // digitalSlotCount=0, channelSlotCount=5
      2, 0, 2,  1, 0, 1, 2,  0, 0, 3, 4,  1, 10, 0,       // device 0: voltage_source, comp 0, pins (1 closed / 0 closed), 1 param (10 V), 0 blob
      1, 1, 2,  1, 0, 5, 6,  2, 1, 7, 8,  1, 1000, 0,     // device 1: resistor, comp 1, pins (1 closed / 2 OPEN), 1 param (1000 ohm), 0 blob
      1, 2, 2,  2, 0, 9, 10,  0, 2, 11, 12,  1, 1000, 0,  // device 2: resistor, comp 2, pins (2 closed / 0 OPEN_HIGH), 1 param (1000 ohm), 0 blob
      0x424f4331,                                 // trailer: MAGIC
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

  it("rejects an unknown option key", () => {
    expect(() => new ProgramWriter(1, [[99, 0]])).toThrow(/unknown option key/);
  });

  it("rejects a repeated option key", () => {
    expect(
      () =>
        new ProgramWriter(1, [
          [CORE_OPTION_KEYS.AMBIENT_C, 20],
          [CORE_OPTION_KEYS.AMBIENT_C, 25],
        ]),
    ).toThrow(/repeated/);
  });

  it("rejects an out-of-range option value for every 0/1 key", () => {
    for (const key of [
      CORE_OPTION_KEYS.INTEGRATION,
      CORE_OPTION_KEYS.NO_SOLUTION_CACHE,
      CORE_OPTION_KEYS.NO_RAIL_SHORTCUT,
      CORE_OPTION_KEYS.FORCE_STEPPED,
    ]) {
      expect(() => new ProgramWriter(1, [[key, 2]])).toThrow(/must be 0 or 1/);
    }
  });

  it("rejects a non-finite AMBIENT_C", () => {
    expect(() => new ProgramWriter(1, [[CORE_OPTION_KEYS.AMBIENT_C, NaN]])).toThrow(/must be a finite number/);
  });

  it("rejects a non-zero FLOATING_MODEL (the only value in W1)", () => {
    expect(() => new ProgramWriter(1, [[CORE_OPTION_KEYS.FLOATING_MODEL, 1]])).toThrow(/must be 0/);
  });

  it("rejects a negative START_TIME", () => {
    expect(() => new ProgramWriter(1, [[CORE_OPTION_KEYS.START_TIME, -1]])).toThrow(/must be a finite number >= 0/);
  });

  it("rejects a non-finite START_TIME", () => {
    expect(() => new ProgramWriter(1, [[CORE_OPTION_KEYS.START_TIME, NaN]])).toThrow(/must be a finite number >= 0/);
    expect(() => new ProgramWriter(1, [[CORE_OPTION_KEYS.START_TIME, Infinity]])).toThrow(
      /must be a finite number >= 0/,
    );
  });

  it("accepts START_TIME 0 and a positive value", () => {
    expect(() => new ProgramWriter(1, [[CORE_OPTION_KEYS.START_TIME, 0]])).not.toThrow();
    expect(() => new ProgramWriter(1, [[CORE_OPTION_KEYS.START_TIME, 12.5]])).not.toThrow();
  });

  it("rejects an unknown kind code", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({ kindCode: 99, componentIndex: 0, pins: [pin(0), pin(1)], params: [1] }),
    ).toThrow(/unknown or unshaped kind code/);
  });

  it("rejects the wrong pin count for a kind", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({
        kindCode: CORE_KIND_CODE.resistor,
        componentIndex: 0,
        pins: [pin(0), pin(1), pin(1)],
        params: [1],
      }),
    ).toThrow(/pin\(s\)/);
  });

  it("rejects the wrong param count for a kind", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 0, pins: [pin(0), pin(1)], params: [1, 2] }),
    ).toThrow(/param\(s\)/);
  });

  it("rejects a non-empty blob for a kind whose shape is blob-exact-0", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({
        kindCode: CORE_KIND_CODE.resistor,
        componentIndex: 0,
        pins: [pin(0), pin(1)],
        params: [1],
        blob: [1],
      }),
    ).toThrow(/blob of exactly 0/);
  });

  it("rejects a pin node outside [0, nodeCount)", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 0, pins: [pin(0), pin(2)], params: [1] }),
    ).toThrow(/out of range/);
  });

  it("rejects an out-of-range pin hash (negative, non-integer, or >= 2^32)", () => {
    const writer = new ProgramWriter(2);
    for (const bad of [-1, 1.5, 2 ** 32]) {
      expect(() =>
        writer.addDevice({
          kindCode: CORE_KIND_CODE.resistor,
          componentIndex: 0,
          pins: [pin(0, { openHash: bad }), pin(1)],
          params: [1],
        }),
      ).toThrow(/openHash .* must be an integer in \[0, 2\^32\)/);
    }
  });

  it("rejects a negative componentIndex", () => {
    const writer = new ProgramWriter(2);
    expect(() =>
      writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: -1, pins: [pin(0), pin(1)], params: [1] }),
    ).toThrow(/componentIndex/);
  });

  it("rejects addDevice after finish()", () => {
    const writer = new ProgramWriter(2);
    writer.finish(0, 0);
    expect(() =>
      writer.addDevice({ kindCode: CORE_KIND_CODE.resistor, componentIndex: 0, pins: [pin(0), pin(1)], params: [1] }),
    ).toThrow(/after finish/);
  });

  it("rejects a second finish()", () => {
    const writer = new ProgramWriter(2);
    writer.finish(0, 0);
    expect(() => writer.finish(0, 0)).toThrow(/already finished/);
  });

  it("rejects a negative or non-integer readout slot count", () => {
    const writer = new ProgramWriter(1);
    expect(() => writer.finish(-1, 0)).toThrow(/digitalSlotCount/);
    expect(() => writer.finish(0, 1.5)).toThrow(/channelSlotCount/);
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
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 1, 1))).toThrow(/version/);
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

  it("rejects an unknown option key", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 5, 99))).toThrow(/unknown key/);
  });

  it("rejects an out-of-range option value", () => {
    // Offset 6 is the baseline's single AMBIENT_C value.
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 6, NaN))).toThrow(/must be a finite number/);
  });

  it("rejects a repeated option key (hand-built: ProgramWriter itself already refuses to write one)", () => {
    // MAGIC, VERSION, nodeCount=1, deviceCount=0, optionCount=2,
    // (AMBIENT_C,20), (AMBIENT_C,25), digitalSlotCount=0, channelSlotCount=0, trailer
    const program = Float64Array.from([
      CORE_FORMAT_MAGIC, CORE_FORMAT_VERSION, 1, 0, 2, CORE_OPTION_KEYS.AMBIENT_C, 20, CORE_OPTION_KEYS.AMBIENT_C, 25, 0, 0,
      CORE_FORMAT_MAGIC,
    ]);
    expect(() => decodeProgram(program)).toThrow(/repeated/);
  });

  it("rejects a negative optionCount", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 4, -1))).toThrow(/non-negative integer/);
  });

  it("rejects a negative digitalSlotCount / channelSlotCount", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 7, -1))).toThrow(/non-negative integer/);
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 8, -1))).toThrow(/non-negative integer/);
  });

  it("rejects an unknown kind code", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 9, 99))).toThrow(/unknown kind code/);
  });

  it("rejects a negative componentIndex", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 10, -1))).toThrow(/non-negative integer/);
  });

  it("rejects the wrong pinCount for a kind", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 11, 3))).toThrow(/pinCount 3, expected 2/);
  });

  it("rejects a negative pin node index", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 12, -1))).toThrow(/non-negative integer/);
  });

  it("rejects a pin node index >= nodeCount", () => {
    // Baseline nodeCount is 2; offset 12 is the resistor's first pin's node.
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 12, 5))).toThrow(/out of range for nodeCount 2/);
  });

  it("rejects a pin flags word with an unknown bit set", () => {
    // Offset 13 is pin0's flags (baseline: 0, closed).
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 13, 4))).toThrow(/unknown bit set/);
  });

  // Finding 5 (w1a-fixes/ts.json #5): the old check was a bitwise
  // `(flags & ~KNOWN_BITS) !== 0`, which coerces through ToInt32 (mod 2^32)
  // — so a flags word AT OR ABOVE 2^32 wrapped around and silently passed
  // (2^32 -> 0 "no flags"; 2^32+1 -> 1, a spurious OPEN) instead of being
  // rejected the way the Rust reader rejects it (program.rs's bounded_index
  // compares the raw f64 with no modulo). `flags > PIN_FLAG_KNOWN_BITS`
  // (a plain numeric comparison, safe because readCount already guarantees a
  // non-negative integer) has no such wraparound at any magnitude.
  it("rejects a pin flags word at/above 2^32 (bitwise truncation would otherwise wrap it to a value that passes)", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 13, 2 ** 32))).toThrow(/unknown bit set/);
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 13, 2 ** 32 + 1))).toThrow(/unknown bit set/);
  });

  it("decodes OPEN and OPEN_HIGH bits correctly, including both set at once", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 13, 3))).not.toThrow();
    const decoded = decodeProgram(mutated(buildBaselineProgram(), 13, 3));
    expect(decoded.devices[0]!.pins[0]).toMatchObject({ open: true, openHigh: true });
  });

  it("rejects an out-of-range pin hash (negative, non-integer, or >= 2^32)", () => {
    // Offset 14 is pin0's openHash.
    for (const bad of [-1, 1.5, 2 ** 32]) {
      expect(() => decodeProgram(mutated(buildBaselineProgram(), 14, bad))).toThrow(
        /openHash = .* must be an integer in \[0, 2\^32\)/,
      );
    }
  });

  it("rejects the wrong paramCount for a kind", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 20, 2))).toThrow(/paramCount 2, expected 1/);
  });

  it("rejects a non-finite param", () => {
    // Offset 21 is the baseline resistor's R.
    for (const bad of [Number.NaN, Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY]) {
      expect(() => decodeProgram(mutated(buildBaselineProgram(), 21, bad))).toThrow(/params\[0\] must be finite/);
    }
  });

  it("rejects the wrong blobCount for a kind", () => {
    // Offset 22 is the baseline resistor's blobCount (shape: exact 0).
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 22, 1))).toThrow(/blobCount 1, expected exactly 0/);
  });

  it("rejects a bad trailer magic", () => {
    expect(() => decodeProgram(mutated(buildBaselineProgram(), 23, 0))).toThrow(/trailer magic/);
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

// ---- W1c: a digital-slot-owning kind's blob[0] + digitalSlots <= S --------
// (core/README.md, "Slot indices" — mirrors core/src/program.rs's reader
// check; core/compile/ics.test.ts covers the same rule end to end through a
// real compiled IC program, so these are format.ts's own unit-level checks
// of ProgramWriter/decodeProgram directly.)

describe("blob[0] + digitalSlots <= S — a digital-slot-owning kind (74ls00: 12 logic pins, 4 output slots)", () => {
  /** A minimal, structurally-valid 74ls00 record: 14 pins (all node 0, closed), 13 zeroed params, a blob of [slotBase, 12 zeroed pin flags]. Only `slotBase` varies between tests — the exact param/flag VALUES are format.ts's business only insofar as they're finite, which is checked elsewhere (kind-specific value rules like famFlags' range live in core/src/kinds.rs, not here). */
  function ls00Device(slotBase: number) {
    return {
      kindCode: CORE_KIND_CODE["74ls00"],
      componentIndex: 0,
      pins: Array.from({ length: 14 }, () => pin(0)),
      params: Array.from({ length: 13 }, () => 0),
      blob: [slotBase, ...Array.from({ length: 12 }, () => 0)],
    };
  }

  describe("ProgramWriter", () => {
    it("accepts a device whose digital slot range fits exactly within the final digitalSlotCount", () => {
      const writer = new ProgramWriter(1);
      writer.addDevice(ls00Device(0));
      expect(() => writer.finish(4, 0)).not.toThrow();
    });

    it("rejects (at addDevice) a negative or non-integer blob[0] slot base", () => {
      for (const bad of [-1, 0.5]) {
        const writer = new ProgramWriter(1);
        expect(() => writer.addDevice(ls00Device(bad))).toThrow(
          /blob\[0\] \(digital slot base\) must be a non-negative integer/,
        );
      }
    });

    it("rejects (at finish, once S is final) a device whose digital slots run past digitalSlotCount", () => {
      const writer = new ProgramWriter(1);
      writer.addDevice(ls00Device(1)); // slots [1, 5) — one past a digitalSlotCount of 4
      expect(() => writer.finish(4, 0)).toThrow(/digital slots \[1, 5\) exceed digitalSlotCount 4/);
    });

    it("rejects two devices whose slot ranges, added in order, together exceed digitalSlotCount", () => {
      const writer = new ProgramWriter(1);
      writer.addDevice(ls00Device(0)); // slots [0, 4)
      writer.addDevice(ls00Device(4)); // slots [4, 8) — fits only if S >= 8
      expect(() => writer.finish(4, 0)).toThrow(/digital slots \[4, 8\) exceed digitalSlotCount 4/);
      // The same two devices DO fit a large enough S.
      const writer2 = new ProgramWriter(1);
      writer2.addDevice(ls00Device(0));
      writer2.addDevice(ls00Device(4));
      expect(() => writer2.finish(8, 0)).not.toThrow();
    });
  });

  describe("decodeProgram", () => {
    function programWithSlotBase(slotBase: number, digitalSlotCount: number): Float64Array {
      const writer = new ProgramWriter(1);
      writer.addDevice(ls00Device(slotBase));
      return writer.finish(digitalSlotCount, 0);
    }

    it("decodes cleanly when the slot range fits", () => {
      const decoded = decodeProgram(programWithSlotBase(0, 4));
      expect(decoded.devices[0]!.blob[0]).toBe(0);
    });

    it("rejects a hand-corrupted digitalSlotCount that no longer fits a device's own slot range", () => {
      // ProgramWriter itself already refuses to produce this program (the
      // ProgramWriter describe block above) — this proves decodeProgram, the
      // actual authority per this file's own header comment, independently
      // rejects a hand-built (or hand-corrupted) Float64Array making the same
      // mistake, not just a legitimately-constructed one.
      const program = programWithSlotBase(0, 4);
      // With zero options, the header is exactly [MAGIC, VERSION, nodeCount=1,
      // deviceCount=1, optionCount=0, digitalSlotCount, channelSlotCount, ...] —
      // digitalSlotCount is index 5.
      const corrupted = mutated(program, 5, 3);
      expect(() => decodeProgram(corrupted)).toThrow(/digital slots \[0, 4\) exceed digitalSlotCount 3/);
    });

    it("rejects a hand-corrupted negative or fractional blob[0]", () => {
      const program = programWithSlotBase(0, 4);
      // 7-value header (MAGIC..channelSlotCount, indices 0-6) + kindCode +
      // componentIndex + pinCount (3) + 14 pins x 4 values (56) + paramCount
      // (1) + 13 params + blobCount (1) = 81: blob[0] (slotBase) itself.
      const blobStart = 7 + 3 + 14 * 4 + 1 + 13 + 1;
      expect(program[blobStart]).toBe(0); // sanity: this really is slotBase
      expect(() => decodeProgram(mutated(program, blobStart, -1))).toThrow(
        /blob\[0\] \(digital slot base\) -1, expected a non-negative integer/,
      );
      expect(() => decodeProgram(mutated(program, blobStart, 0.5))).toThrow(
        /blob\[0\] \(digital slot base\) 0\.5, expected a non-negative integer/,
      );
    });
  });
});

// ---- W1e: a channel-slot-owning kind's blob[0] + channelSlots <= C --------
// (core/README.md, "Slot indices" — the channel-slot twin of the digital
// check above, mirroring the Rust reader's new rule; core/compile-leds.test.ts
// covers the same rule end to end through a real compiled `rgb_led` program,
// so these are format.ts's own unit-level checks of ProgramWriter/
// decodeProgram directly.)

describe("blob[0] + channelSlots <= C — a channel-slot-owning kind (rgb_led: 3 channel slots, no digital slots)", () => {
  /** A minimal, structurally-valid rgb_led record: 4 pins (all node 0, closed), 6 zeroed params, a blob of exactly [channelBase] — rgb_led has no digital slots, so the channel base sits at blob[0]. */
  function rgbLedDevice(channelBase: number) {
    return {
      kindCode: CORE_KIND_CODE.rgb_led,
      componentIndex: 0,
      pins: Array.from({ length: 4 }, () => pin(0)),
      params: Array.from({ length: 6 }, () => 0),
      blob: [channelBase],
    };
  }

  describe("ProgramWriter", () => {
    it("accepts a device whose channel slot range fits exactly within the final channelSlotCount", () => {
      const writer = new ProgramWriter(1);
      writer.addDevice(rgbLedDevice(0));
      expect(() => writer.finish(0, 3)).not.toThrow();
    });

    it("rejects (at addDevice) a negative or non-integer blob[0] channel slot base", () => {
      for (const bad of [-1, 0.5]) {
        const writer = new ProgramWriter(1);
        expect(() => writer.addDevice(rgbLedDevice(bad))).toThrow(
          /blob\[0\] \(channel slot base\) must be a non-negative integer/,
        );
      }
    });

    it("rejects (at finish, once C is final) a device whose channel slots run past channelSlotCount", () => {
      const writer = new ProgramWriter(1);
      writer.addDevice(rgbLedDevice(1)); // slots [1, 4) — one past a channelSlotCount of 3
      expect(() => writer.finish(0, 3)).toThrow(/channel slots \[1, 4\) exceed channelSlotCount 3/);
    });

    it("rejects two devices whose channel slot ranges, added in order, together exceed channelSlotCount", () => {
      const writer = new ProgramWriter(1);
      writer.addDevice(rgbLedDevice(0)); // slots [0, 3)
      writer.addDevice(rgbLedDevice(3)); // slots [3, 6) — fits only if C >= 6
      expect(() => writer.finish(0, 3)).toThrow(/channel slots \[3, 6\) exceed channelSlotCount 3/);
      // The same two devices DO fit a large enough C.
      const writer2 = new ProgramWriter(1);
      writer2.addDevice(rgbLedDevice(0));
      writer2.addDevice(rgbLedDevice(3));
      expect(() => writer2.finish(0, 6)).not.toThrow();
    });
  });

  describe("decodeProgram", () => {
    function programWithChannelBase(channelBase: number, channelSlotCount: number): Float64Array {
      const writer = new ProgramWriter(1);
      writer.addDevice(rgbLedDevice(channelBase));
      return writer.finish(0, channelSlotCount);
    }

    it("decodes cleanly when the slot range fits", () => {
      const decoded = decodeProgram(programWithChannelBase(0, 3));
      expect(decoded.devices[0]!.blob[0]).toBe(0);
    });

    it("rejects a hand-corrupted channelSlotCount that no longer fits a device's own slot range", () => {
      const program = programWithChannelBase(0, 3);
      // With zero options, the header is exactly [MAGIC, VERSION, nodeCount=1,
      // deviceCount=1, optionCount=0, digitalSlotCount, channelSlotCount,
      // ...] — channelSlotCount is index 6.
      const corrupted = mutated(program, 6, 2);
      expect(() => decodeProgram(corrupted)).toThrow(/channel slots \[0, 3\) exceed channelSlotCount 2/);
    });

    it("rejects a hand-corrupted negative or fractional blob[0]", () => {
      const program = programWithChannelBase(0, 3);
      // 7-value header (MAGIC..channelSlotCount, indices 0-6) + kindCode +
      // componentIndex + pinCount (3) + 4 pins x 4 values (16) + paramCount
      // (1) + 6 params + blobCount (1) = 34: blob[0] (channelBase) itself.
      const blobStart = 7 + 3 + 4 * 4 + 1 + 6 + 1;
      expect(program[blobStart]).toBe(0); // sanity: this really is channelBase
      expect(() => decodeProgram(mutated(program, blobStart, -1))).toThrow(
        /blob\[0\] \(channel slot base\) -1, expected a non-negative integer/,
      );
      expect(() => decodeProgram(mutated(program, blobStart, 0.5))).toThrow(
        /blob\[0\] \(channel slot base\) 0\.5, expected a non-negative integer/,
      );
    });
  });
});
