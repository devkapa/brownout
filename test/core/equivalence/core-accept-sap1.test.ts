/**
 * The pure pieces of scripts/core-accept-sap1.ts (W1z Z2's first version, written by S1a for checkpoint S1): the
 * display digit decoder, the program-cycle counter, the phase-aligned instant arithmetic and the quiet-instant test.
 * Each has a detector self-test: a deliberately wrong input the check must refuse, so a clean sweep on the real board
 * means something.
 */

import { describe, expect, it } from "vitest";
import {
  atPhase,
  buildDigitTable,
  clockTimeline,
  decodeDigit,
  expectedDigit,
  findProgramCycle,
  highSegment,
  isQuietAt,
  litMask,
  lowSegment,
  noteCoreLatches,
  outValue,
  phaseFraction,
  pickTargetEdges,
  romSegmentMask,
  SEGMENTS,
  type CoreLatchSource,
  type FailureSets,
} from "../../../scripts/core-accept-sap1.js";
import { CoreEngine } from "../../../src/core/engine.js";
import { initCore } from "../../../src/core/wasm.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { findRisingEdges, sampleCycles, type CycleSample } from "./align.js";
import { levelChanges } from "./run-entry.js";
import type { DigitalTraceEvent } from "./trace.js";

/** The SAP-1's own encoding (gen-sap1.mjs): ROM bit 6 drives segment a, 5 b, 4 c, 3 d, 2 e, 1 f, 0 g, 7 dp. */
const SEGMENT_ROM_BIT: Record<string, number> = { a: 6, b: 5, c: 4, d: 3, e: 2, f: 1, g: 0, dp: 7 };
const DIGITS = [0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b];

function displayRom(): Uint8Array {
  const rom = new Uint8Array(2048);
  for (let v = 0; v < 256; v++) {
    rom[v] = DIGITS[v % 10]!;
    rom[v + 256] = DIGITS[Math.floor(v / 10) % 10]!;
    rom[v + 512] = DIGITS[Math.floor(v / 100) % 10]!;
  }
  return rom;
}

/** Channel currents (a, b, c, d, e, f, g, dp) of a display showing `digit`: 3.7 mA on a lit segment, 0 on the rest. */
function channelsShowing(digit: number): number[] {
  return SEGMENTS.map((segment) => (((DIGITS[digit]! >> SEGMENT_ROM_BIT[segment]!) & 1) === 1 ? 3.7e-3 : 0));
}

describe("the display digit decoder", () => {
  const table = buildDigitTable(displayRom(), SEGMENT_ROM_BIT);

  it("reads a segment as lit above the threshold and unlit below it", () => {
    expect(litMask([0, 0, 0, 0, 0, 0, 0, 0])).toBe(0);
    expect(litMask([3.7e-3, 0, 0, 0, 0, 0, 0, 0])).toBe(0b1);
    expect(litMask([0, 0, 0, 0, 0, 0, 0, 5e-3])).toBe(0b10000000);
    // Leakage (nanoamps) and a current just under the threshold are both dark; just over is lit.
    expect(litMask([1e-9, 0.9e-3, 1.1e-3, 0, 0, 0, 0, 0])).toBe(0b100);
    expect(litMask([2e-3, 0], 3e-3)).toBe(0);
  });

  it("maps every digit's lit segments back to that digit through the ROM's own encoding", () => {
    for (let digit = 0; digit < 10; digit++) {
      expect(decodeDigit(litMask(channelsShowing(digit)), table)).toBe(digit);
    }
  });

  it("calls no lit segment blank, and a pattern no digit uses unknown", () => {
    expect(decodeDigit(0, table)).toBe("blank");
    expect(decodeDigit(0b10000000, table)).toBe("unknown"); // the decimal point alone
  });

  it("detector: a decoder built with two segments swapped mis-reads the digits it should not", () => {
    const swapped = { ...SEGMENT_ROM_BIT, a: SEGMENT_ROM_BIT.b, b: SEGMENT_ROM_BIT.a };
    const wrongTable = buildDigitTable(displayRom(), swapped);
    // 0, 2, 3, 7, 8, 9 light a and b together (or neither): symmetric, still read right. 1 (b only), 5 and 6 (a only) are not.
    for (const symmetric of [0, 2, 3, 7, 8, 9]) expect(decodeDigit(litMask(channelsShowing(symmetric)), wrongTable)).toBe(symmetric);
    for (const asymmetric of [1, 5, 6]) expect(decodeDigit(litMask(channelsShowing(asymmetric)), wrongTable)).not.toBe(asymmetric);
  });

  it("refuses a ROM whose ten digits are not distinct", () => {
    const rom = displayRom();
    rom[3] = rom[2]!;
    expect(() => buildDigitTable(rom, SEGMENT_ROM_BIT)).toThrow(/distinct/);
  });

  it("turns a ROM byte into the lit-segment mask the display shows", () => {
    expect(romSegmentMask(DIGITS[8]!, SEGMENT_ROM_BIT)).toBe(0b01111111);
    expect(romSegmentMask(0x80, SEGMENT_ROM_BIT)).toBe(0b10000000);
  });

  it("gives the digit each display position shows for an OUT value: ones, tens, hundreds, then blank", () => {
    expect(expectedDigit(0, 237)).toBe(7);
    expect(expectedDigit(1, 237)).toBe(3);
    expect(expectedDigit(2, 237)).toBe(2);
    expect(expectedDigit(3, 237)).toBe("blank");
    expect(expectedDigit(2, 42)).toBe(0); // the board shows leading zeros
  });

  it("agrees with the ROM itself at every (position, OUT) address", () => {
    const rom = displayRom();
    for (let out = 0; out < 256; out++) {
      for (let d = 0; d < 3; d++) {
        const want = expectedDigit(d, out) as number;
        expect(decodeDigit(romSegmentMask(rom[(d << 8) | out]!, SEGMENT_ROM_BIT), table)).toBe(want);
      }
    }
  });
});

// ---- the cycle counter -------------------------------------------------------------------------------------

const OUT_KEYS = ["o0", "o1", "o2", "o3", "o4", "o5", "o6", "o7"];

function outState(value: number | undefined): Record<string, number> {
  if (value === undefined) return {};
  return Object.fromEntries(OUT_KEYS.map((k, i) => [k, (value >> i) & 1]));
}

function cyclesOf(values: Array<number | undefined>): CycleSample[] {
  return values.map((v, i) => ({ cycleIndex: i, edgeTime: i === 0 ? null : i * 1e-3, state: outState(v) }));
}

describe("the program-cycle counter", () => {
  it("reads OUT as an integer, and as nothing when a bit is missing", () => {
    expect(outValue(outState(0xa5), OUT_KEYS)).toBe(0xa5);
    expect(outValue({ ...outState(0xa5), o3: undefined as unknown as number }, OUT_KEYS)).toBeUndefined();
    expect(outValue({}, OUT_KEYS)).toBeUndefined();
  });

  it("counts the cycles from OUT = 0 to the first 255 to the next 0", () => {
    // Three cycles of 0, three of each value up to 255, two of each value coming down, then more 0.
    const values: number[] = [0, 0, 0];
    for (let v = 1; v <= 255; v++) values.push(v, v, v);
    for (let v = 254; v >= 0; v--) values.push(v, v);
    values.push(0, 0);
    const found = findProgramCycle(cyclesOf(values), OUT_KEYS)!;
    expect(found).toEqual({ startCycle: 0, peakCycle: 3 + 254 * 3, backToZeroCycle: 3 + 255 * 3 + 254 * 2, length: 3 + 255 * 3 + 254 * 2 });
  });

  it("does not count a run that never reaches 255, or never comes back", () => {
    expect(findProgramCycle(cyclesOf([0, 1, 2, 3, 100, 200, 254, 0]), OUT_KEYS)).toBeUndefined();
    expect(findProgramCycle(cyclesOf([0, 1, 255, 254, 3]), OUT_KEYS)).toBeUndefined();
    expect(findProgramCycle(cyclesOf([]), OUT_KEYS)).toBeUndefined();
  });

  it("skips cycles where OUT is not published yet, and starts at the first real 0", () => {
    const found = findProgramCycle(cyclesOf([undefined, undefined, 0, 7, 255, 9, 0]), OUT_KEYS)!;
    expect(found).toEqual({ startCycle: 2, peakCycle: 4, backToZeroCycle: 6, length: 4 });
  });
});

describe("the target cycles", () => {
  // OUT changes at the edge that loads it: the sample before edge i shows the new value, so edge i - 1 loaded it.
  const values = Array.from({ length: 400 }, (_, i) => Math.floor(i / 20) % 256);
  const cycles = cyclesOf(values);

  it("picks a spread of cycles and cycles where OUT changed, sorted and distinct", () => {
    const picked = pickTargetEdges(cycles, OUT_KEYS, 60, 380, 6, 6);
    expect(picked).toEqual([...new Set(picked)].sort((a, b) => a - b));
    expect(picked[0]).toBeGreaterThanOrEqual(60);
    expect(picked[picked.length - 1]).toBeLessThanOrEqual(380);
    // Every change pick is an edge whose next sample shows a new OUT.
    const changeEdges = new Set(cycles.slice(1).filter((c, i) => outValue(c.state, OUT_KEYS) !== outValue(cycles[i]!.state, OUT_KEYS)).map((c) => c.cycleIndex - 1));
    expect(picked.filter((e) => changeEdges.has(e)).length).toBeGreaterThanOrEqual(6);
  });

  it("still returns the even spread when OUT never changes", () => {
    const flat = cyclesOf(Array.from({ length: 200 }, () => 5));
    expect(pickTargetEdges(flat, OUT_KEYS, 10, 190, 4, 4)).toEqual([10, 70, 130, 190]);
  });
});

// ---- the clock, its phases, and the quiet test --------------------------------------------------------------

describe("clock timeline and phase-aligned instants", () => {
  /** A clock HIGH from t = 0 (the harness counts that as rising edge 1), then LOW 1 ms, HIGH 1 ms, and so on. */
  const trace: DigitalTraceEvent[] = [
    { t: 0, key: "clk", value: 1 },
    { t: 1e-3, key: "clk", value: 0 },
    { t: 2e-3, key: "clk", value: 1 },
    { t: 3.2e-3, key: "clk", value: 0 },
    { t: 4e-3, key: "clk", value: 1 },
  ];
  const timeline = clockTimeline(levelChanges(trace, "clk"));

  it("counts a clock that starts HIGH the way align.ts's findRisingEdges does", () => {
    expect(timeline.rise).toEqual([0, 2e-3, 4e-3]);
    expect(timeline.rise).toEqual(findRisingEdges(trace, "clk"));
    expect(timeline.fall).toEqual([1e-3, 3.2e-3]);
  });

  it("agrees with sampleCycles on which edge is which: cycle i's sample is taken before rising edge i", () => {
    const cycles = sampleCycles(trace, "clk");
    expect(cycles.map((c) => c.edgeTime)).toEqual([null, ...timeline.rise]);
  });

  it("finds each complete HIGH and LOW, and none while one is still open", () => {
    expect(highSegment(timeline, 1)).toEqual([0, 1e-3]);
    expect(lowSegment(timeline, 1)).toEqual([1e-3, 2e-3]);
    expect(highSegment(timeline, 2)).toEqual([2e-3, 3.2e-3]);
    expect(lowSegment(timeline, 2)).toEqual([3.2e-3, 4e-3]);
    expect(highSegment(timeline, 3)).toBeUndefined();
    expect(lowSegment(timeline, 3)).toBeUndefined();
  });

  it("maps a fraction of one engine's segment onto the same fraction of another's", () => {
    const classic: [number, number] = [0.1, 0.1021];
    const core: [number, number] = [0.09, 0.0911];
    const phase = phaseFraction(classic, 0.10105);
    expect(phase).toBeCloseTo(0.5, 12);
    expect(atPhase(core, phase)).toBeCloseTo(0.09055, 12);
    expect(atPhase(core, 0)).toBe(0.09);
    expect(atPhase(core, 1)).toBeCloseTo(0.0911, 12);
  });

  it("refuses a clock that starts LOW: its first edge would not be counted the way the harness counts", () => {
    expect(() => clockTimeline([{ t: 0, high: false }])).toThrow(/starts LOW/);
  });
});

describe("the quiet-instant test", () => {
  const events = [0.001, 0.0021, 0.005];

  it("is quiet only when no event is within the guard on either side", () => {
    expect(isQuietAt(events, 0.003, 1e-4)).toBe(true);
    expect(isQuietAt(events, 0.0021 + 5e-5, 1e-4)).toBe(false); // just after an event
    expect(isQuietAt(events, 0.005 - 5e-5, 1e-4)).toBe(false); // just before one
    expect(isQuietAt(events, 0.0021 + 1.5e-4, 1e-4)).toBe(true); // clear of the guard on the far side of an event
    expect(isQuietAt([], 1, 1)).toBe(true);
    expect(isQuietAt(events, 0.0005, 1e-4)).toBe(true); // before the first event
    expect(isQuietAt(events, 0.01, 1e-4)).toBe(true); // after the last
  });
});

describe("criterion 5's core side", () => {
  // A 74LS00 whose inputs a pulse source holds HIGH except for 2 ms: inside the pulse 1Y is commanded HIGH into
  // 100 ohm to ground and sags (W1g point 9), outside it 1Y is LOW and nothing sags. One advance from 0 to 5 ms
  // covers the whole latch, so at the advance end getFailures() is empty and only the latch log has it.
  function pulsedSag(): SimCircuit {
    const nand = ["vcc", "gnd", "1a", "1b", "1y", "2a", "2b", "2y", "3a", "3b", "3y", "4a", "4b", "4y"].map((id) => ({ id }));
    return {
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "in", kind: "pulse_source", pins: [{ id: "pos" }, { id: "neg" }], params: { v1: 5, v2: 0, td: 1e-3, tr: 1e-6, tf: 1e-6, pw: 2e-3, per: 1 } },
        { id: "u1", kind: "74ls00", pins: nand, params: {} },
        { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "u1", to_pin: "vcc" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "gnd" },
        { from_component: "in", from_pin: "neg", to_component: "vcc", to_pin: "neg" },
        { from_component: "in", from_pin: "pos", to_component: "u1", to_pin: "1a" },
        { from_component: "in", from_pin: "pos", to_component: "u1", to_pin: "1b" },
        { from_component: "u1", from_pin: "1y", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
    } as SimCircuit;
  }
  const SAG_KEY = "output_sag\x00u1\x001y";

  function seenOver(stripLog: boolean): { seen: FailureSets; endFailures: number } {
    const engine = new CoreEngine();
    try {
      engine.load(pulsedSag());
      const source: CoreLatchSource = stripLog
        ? {
            getFailures: () => engine.getFailures(),
            getWarnings: () => engine.getWarnings(),
            takeLatchedFailures: () => (engine.takeLatchedFailures(), []),
            takeNewWarnings: () => engine.takeNewWarnings(),
          }
        : engine;
      const seen: FailureSets = { failures: {}, warnings: {} };
      noteCoreLatches(seen, source);
      engine.advance(5e-3);
      expect(engine.lastConverged).toBe(true);
      const endFailures = Object.keys(engine.getFailures()).length;
      noteCoreLatches(seen, source);
      return { seen, endFailures };
    } finally {
      engine.dispose();
    }
  }

  it("sees a latch that clears inside one advance, from the latch log", async () => {
    await initCore();
    const { seen, endFailures } = seenOver(false);
    // The premise: the sag is gone by the advance end, so a getFailures() poll there cannot see it.
    expect(endFailures).toBe(0);
    expect(Object.keys(seen.failures)).toEqual([SAG_KEY]);
    expect(seen.failures[SAG_KEY]).toBe("output_sag");
  });

  it("misses it once the latch is removed from the log, so the criterion would fail", async () => {
    await initCore();
    const { seen, endFailures } = seenOver(true);
    expect(endFailures).toBe(0);
    expect(seen.failures).toEqual({});
  });
});
