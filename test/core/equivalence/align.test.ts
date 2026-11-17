/**
 * Unit coverage for align.ts, entirely against synthetic traces (this
 * worker's brief: "code against an interface and test with synthetic
 * traces" — no real engine is involved anywhere in this file). Each seeded
 * fault from the worker brief's "Tests" list gets its own case: a bit flip,
 * a dropped edge, a shifted state, and an alignment offset beyond the bound.
 */

import { describe, expect, it } from "vitest";
import {
  alignCycles,
  compareAlignedCycles,
  findRisingEdges,
  requireMinComparedCycles,
  sampleCycles,
  TooFewComparedCyclesError,
  type AlignmentSuccess,
} from "./align.js";
import type { DigitalTraceEvent } from "./trace.js";

/**
 * A synthetic clocked trace whose `counter` value is exactly `values[i]` at
 * cycle `i` (i = 0..values.length-1): `counter` is updated to `values[k]`
 * strictly before the k-th rising edge of `clk` (so sampleCycles's "settled
 * state just before the edge" fold picks it up starting at cycle `k`), and
 * `clk` cleanly toggles 0/1/0 between each pair of edges so every edge is a
 * real 0->1 transition.
 */
function syntheticCounterTrace(values: readonly number[]): DigitalTraceEvent[] {
  const events: DigitalTraceEvent[] = [{ t: 0, key: "clk", value: 0 }, { t: 0, key: "counter", value: values[0]! }];
  for (let k = 1; k < values.length; k++) {
    const edgeT = k * 10;
    events.push({ t: edgeT - 1, key: "counter", value: values[k]! });
    events.push({ t: edgeT, key: "clk", value: 1 });
    events.push({ t: edgeT + 5, key: "clk", value: 0 });
  }
  return events;
}

describe("findRisingEdges", () => {
  it("finds every 0/absent -> 1 transition of the named key, ignoring other keys", () => {
    const trace: DigitalTraceEvent[] = [
      { t: 0, key: "clk", value: 0 },
      { t: 0, key: "other", value: 1 },
      { t: 1, key: "clk", value: 1 },
      { t: 1.5, key: "clk", value: 0 },
      { t: 2, key: "clk", value: 1 },
    ];
    expect(findRisingEdges(trace, "clk")).toEqual([1, 2]);
  });

  it("counts a clock that is already HIGH at the very first sample as an edge — decision 8's load-time phantom edge must not be silently skipped", () => {
    const trace: DigitalTraceEvent[] = [{ t: 0, key: "clk", value: 1 }];
    expect(findRisingEdges(trace, "clk")).toEqual([0]);
  });

  it("does not double-count a value that stays HIGH across consecutive events", () => {
    const trace: DigitalTraceEvent[] = [
      { t: 0, key: "clk", value: 1 },
      { t: 1, key: "clk", value: 1 },
    ];
    expect(findRisingEdges(trace, "clk")).toEqual([0]);
  });
});

describe("sampleCycles", () => {
  it("samples one cycle per edge (plus the leading cycle 0), with the correct counter value at each", () => {
    const values = [10, 20, 30, 40, 50];
    const cycles = sampleCycles(syntheticCounterTrace(values), "clk", ["counter"]);
    expect(cycles.map((c) => c.state.counter)).toEqual(values);
    expect(cycles.map((c) => c.cycleIndex)).toEqual([0, 1, 2, 3, 4]);
  });

  it("restricts each sample to the declared projection", () => {
    const trace: DigitalTraceEvent[] = [
      { t: 0, key: "clk", value: 0 },
      { t: 0, key: "counter", value: 1 },
      { t: 0, key: "irrelevant", value: 99 },
    ];
    const cycles = sampleCycles(trace, "clk", ["counter"]);
    expect(cycles[0]!.state).toEqual({ counter: 1 });
  });
});

/** Builds classic/core cycle sequences with a shared anchor and hands back the alignment for reuse across assertions. */
function align(classicValues: readonly number[], coreValues: readonly number[], anchorValue: number, boundCycles = 6) {
  const classicCycles = sampleCycles(syntheticCounterTrace(classicValues), "clk", ["counter"]);
  const coreCycles = sampleCycles(syntheticCounterTrace(coreValues), "clk", ["counter"]);
  const alignment = alignCycles(classicCycles, coreCycles, { counter: anchorValue }, boundCycles);
  return { classicCycles, coreCycles, alignment };
}

describe("alignCycles — the happy path", () => {
  it("finds offset 0 when both sides carry the identical sequence", () => {
    const values = [10, 20, 30, 40, 50];
    const { alignment } = align(values, values, 20);
    expect(alignment.ok).toBe(true);
    expect((alignment as AlignmentSuccess).offsetCycles).toBe(0);
  });

  it("finds the true offset when classic has extra leading cycles the core does not (decision 8's phantom edge shape)", () => {
    const classicValues = [1, 1, 1, 1, 1, 1, 10, 20, 30]; // 6 "phantom" cycles, then the real sequence starts
    const coreValues = [10, 20, 30];
    const { alignment } = align(classicValues, coreValues, 10, 6);
    expect(alignment.ok).toBe(true);
    expect((alignment as AlignmentSuccess).offsetCycles).toBe(6);
  });
});

describe("seeded fault: a bit flip", () => {
  it("passes alignment (the anchor is untouched) but compareAlignedCycles reports the flipped cycle", () => {
    const classicValues = [10, 20, 30, 40, 50];
    const coreValues = [10, 20, 99, 40, 50]; // cycle 2 flipped
    const { classicCycles, coreCycles, alignment } = align(classicValues, coreValues, 20);
    expect(alignment.ok).toBe(true);
    const comparison = compareAlignedCycles(classicCycles, coreCycles, alignment as AlignmentSuccess);
    expect(comparison.mismatches).toHaveLength(1);
    expect(comparison.mismatches[0]).toMatchObject({ classicCycle: 2, coreCycle: 2, key: "counter", classicValue: 30, coreValue: 99 });
  });
});

describe("seeded fault: a dropped edge", () => {
  it("shortens the core's cycle count, which reduces the compared span and produces trailing mismatches", () => {
    const classicValues = [10, 20, 30, 40, 50];
    const coreValues = [10, 20, 40, 50]; // the edge that would have produced "30" never happened
    const { classicCycles, coreCycles, alignment } = align(classicValues, coreValues, 20);
    expect(alignment.ok).toBe(true);
    const comparison = compareAlignedCycles(classicCycles, coreCycles, alignment as AlignmentSuccess);
    expect(comparison.comparedCycles).toBeLessThan(classicValues.length - 1);
    expect(comparison.mismatches.length).toBeGreaterThan(0);
    expect(() => requireMinComparedCycles(comparison, classicValues.length - 1)).toThrow(TooFewComparedCyclesError);
  });
});

describe("seeded fault: a shifted state", () => {
  it("aligns correctly on the anchor but the values downstream no longer line up, one position off", () => {
    const classicValues = [10, 20, 30, 40, 50];
    const coreValues = [10, 20, 40, 50, 50]; // same length as classic, but shifted by one from cycle 2 onward
    const { classicCycles, coreCycles, alignment } = align(classicValues, coreValues, 20);
    expect(alignment.ok).toBe(true);
    expect((alignment as AlignmentSuccess).offsetCycles).toBe(0);
    const comparison = compareAlignedCycles(classicCycles, coreCycles, alignment as AlignmentSuccess);
    expect(comparison.mismatches.map((m) => m.classicCycle)).toEqual([2, 3]);
  });
});

describe("seeded fault: an alignment offset beyond the bound", () => {
  it("fails when the true offset is larger than the fixture's declared bound — the bounded search on the SKEWED side finds no match at all", () => {
    const boundCycles = 6;
    // 8 leading "phantom" cycles on the classic side — one more than the bound allows to be searched.
    const classicValues = [1, 1, 1, 1, 1, 1, 1, 1, 10, 20, 30];
    const coreValues = [10, 20, 30];
    const { alignment } = align(classicValues, coreValues, 10, boundCycles);
    expect(alignment.ok).toBe(false);
    if (!alignment.ok) {
      expect(alignment.reason).toMatch(/not found in the classic trace within its first 7 cycle/);
    }
  });

  it("fails when the anchor matches more than once inside the bound — an ambiguous alignment is refused, not guessed", () => {
    const classicValues = [10, 20, 20, 30]; // anchor value 20 recurs within the bound
    const coreValues = [10, 20, 30];
    const { alignment } = align(classicValues, coreValues, 20, 6);
    expect(alignment.ok).toBe(false);
    if (!alignment.ok) {
      expect(alignment.reason).toMatch(/matched 2 times in the classic trace/);
    }
  });
});

describe("requireMinComparedCycles", () => {
  it("does not throw when the compared span meets the declared minimum", () => {
    const values = [10, 20, 30, 40, 50];
    const { classicCycles, coreCycles, alignment } = align(values, values, 20);
    const comparison = compareAlignedCycles(classicCycles, coreCycles, alignment as AlignmentSuccess);
    expect(() => requireMinComparedCycles(comparison, comparison.comparedCycles)).not.toThrow();
  });

  it("throws TooFewComparedCyclesError when the span falls short", () => {
    const values = [10, 20, 30];
    const { classicCycles, coreCycles, alignment } = align(values, values, 20);
    const comparison = compareAlignedCycles(classicCycles, coreCycles, alignment as AlignmentSuccess);
    expect(() => requireMinComparedCycles(comparison, comparison.comparedCycles + 1)).toThrow(TooFewComparedCyclesError);
  });
});
