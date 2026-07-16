import { describe, expect, it } from "vitest";
import { decimateAlignedTrace } from "../../src/sim/trace-decimator.js";

describe("scope plot decimation", () => {
  it("preserves narrow extrema and strictly increasing timestamps", () => {
    const t = Array.from({ length: 1_000 }, (_, index) => index / 1_000);
    const pulse = new Array<number>(1_000).fill(0);
    pulse[503] = 12;
    pulse[504] = -7;

    const result = decimateAlignedTrace(t, [pulse], 100);
    expect(result.values[0]).toContain(12);
    expect(result.values[0]).toContain(-7);
    expect(result.t.every((value, index) => index === 0 || value > result.t[index - 1]!)).toBe(true);
    expect(result.t.length).toBeLessThanOrEqual(102);
  });

  it("retains gap boundaries without retaining an entire missing run", () => {
    const t = Array.from({ length: 400 }, (_, index) => index / 100);
    const signal = t.map((value) => Math.sin(value));
    signal.fill(Number.NaN, 120, 280);

    const result = decimateAlignedTrace(t, [signal], 80);
    const gapTimes = result.t.filter((_, index) => !Number.isFinite(result.values[0]![index]));
    expect(gapTimes).toContain(t[120]);
    expect(gapTimes).toContain(t[279]);
    expect(Number.isFinite(result.values[0]![result.t.indexOf(t[119]!)]!)).toBe(true);
    expect(Number.isFinite(result.values[0]![result.t.indexOf(t[280]!)]!)).toBe(true);
  });

  it("keeps full-resolution data unchanged when already within budget", () => {
    const t = [0, 1, 2];
    const values = [[1, Number.NaN, 3], [4, 5, 6]];
    expect(decimateAlignedTrace(t, values, 10)).toEqual({ t, values });
  });
});
