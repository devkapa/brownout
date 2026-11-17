/** Unit coverage for schedule.ts's dense analog schedule and time-merging helpers. */

import { describe, expect, it } from "vitest";
import { denseAnalogSchedule, mergeScheduleTimes } from "./schedule.js";

describe("denseAnalogSchedule", () => {
  it("includes both endpoints by default", () => {
    const schedule = denseAnalogSchedule(1);
    expect(schedule[0]).toBe(0);
    expect(schedule[schedule.length - 1]).toBe(1);
  });

  it("is a strict superset (in density) of W0's 4-point grid — many more instants than 4 for a 1s run", () => {
    const schedule = denseAnalogSchedule(1);
    expect(schedule.length).toBeGreaterThan(4);
  });

  it("produces roughly instantsPerSecond * durationS instants", () => {
    const schedule = denseAnalogSchedule(2, { instantsPerSecond: 100, includeEndpoints: false });
    // 2s at 100/s -> ~200 instants (the loop's `<` bound may shave off one).
    expect(schedule.length).toBeGreaterThanOrEqual(198);
    expect(schedule.length).toBeLessThanOrEqual(200);
  });

  it("is strictly ascending and deduplicated", () => {
    const schedule = denseAnalogSchedule(0.5, { instantsPerSecond: 50 });
    for (let i = 1; i < schedule.length; i++) {
      expect(schedule[i]!).toBeGreaterThan(schedule[i - 1]!);
    }
  });

  it("can omit the endpoints", () => {
    const schedule = denseAnalogSchedule(1, { includeEndpoints: false, instantsPerSecond: 10 });
    expect(schedule[0]).not.toBe(0);
  });

  it("rejects a non-positive duration", () => {
    expect(() => denseAnalogSchedule(0)).toThrow();
    expect(() => denseAnalogSchedule(-1)).toThrow();
  });

  it("rejects a non-positive instantsPerSecond", () => {
    expect(() => denseAnalogSchedule(1, { instantsPerSecond: 0 })).toThrow();
  });
});

describe("mergeScheduleTimes", () => {
  it("merges, dedupes, and sorts any number of schedules", () => {
    expect(mergeScheduleTimes([3, 1], [2, 1], [])).toEqual([1, 2, 3]);
  });

  it("dedupes to the nanosecond grid, not exact float equality", () => {
    // Two values that differ only by float noise well under a nanosecond collapse to one.
    expect(mergeScheduleTimes([0.1 + 0.2], [0.3])).toHaveLength(1);
  });

  it("is empty when given no schedules (or only empty ones)", () => {
    expect(mergeScheduleTimes()).toEqual([]);
    expect(mergeScheduleTimes([], [])).toEqual([]);
  });
});
