/**
 * scripts/perf/frames.ts's statistics. The measurement itself needs the SAP-1
 * boards (a generator outside this repo), so it is exercised by running the
 * script; what is pinned here is the arithmetic the report is built from.
 */
import { spawnSync } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";

import { frameStats, percentile } from "../../../scripts/perf/frames.js";

describe("percentile (nearest rank)", () => {
  const hundred = Array.from({ length: 100 }, (_, i) => i + 1);
  it("is the value at rank ceil(q n)", () => {
    expect(percentile(hundred, 0.5)).toBe(50);
    expect(percentile(hundred, 0.95)).toBe(95);
    expect(percentile(hundred, 0.99)).toBe(99);
    expect(percentile(hundred, 1)).toBe(100);
    expect(percentile(hundred, 0)).toBe(1);
  });
  it("gives the max for p99 of fewer than 100 frames, never an out-of-range read", () => {
    expect(percentile([1, 2, 3], 0.99)).toBe(3);
    expect(percentile([7], 0.99)).toBe(7);
    expect(percentile([], 0.99)).toBeNaN();
  });
});

describe("frameStats", () => {
  it("sorts its input, leaves it alone, and reports the spread", () => {
    const values = [5, 1, 9, 3, 7];
    const stats = frameStats(values);
    expect(values).toEqual([5, 1, 9, 3, 7]);
    expect(stats).toMatchObject({ frames: 5, min: 1, median: 5, mean: 5, max: 9 });
    expect(stats.p95).toBe(9);
    expect(stats.p99).toBe(9);
  });
  it("one slow frame in a hundred does not move p99 (nearest rank 99), two do", () => {
    const values = Array.from({ length: 100 }, () => 4);
    values[17] = 40;
    const stats = frameStats(values);
    expect(stats.max).toBe(40);
    expect(stats.p99).toBe(4);
    values[18] = 40;
    expect(frameStats(values).p99).toBe(40);
  });
});

describe("frames.ts's command line", () => {
  const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "..");
  const cli = (...args: string[]): { status: number | null; out: string } => {
    const r = spawnSync(path.join(repoRoot, "node_modules", ".bin", "tsx"), [path.join(repoRoot, "scripts", "perf", "frames.ts"), ...args], { cwd: repoRoot, encoding: "utf8" });
    return { status: r.status, out: `${r.stdout}${r.stderr}` };
  };

  it("refuses a --seconds shorter than one frame instead of reporting NaN statistics", () => {
    const r = cli("--seconds", "0.005");
    expect(r.status).toBe(2);
    expect(r.out).toContain("shorter than one frame");
  });

  it("refuses a negative warm-up and an unknown option", () => {
    expect(cli("--warmup", "-1").status).toBe(2);
    expect(cli("--nope").status).toBe(2);
  });
});
