/**
 * The perf-regression check (scripts/perf/baseline.ts and its use in
 * scripts/bench-core-impl.ts): plan 3.5's "warn when the Node number regresses
 * more than 25% against a recorded baseline". These tests pin the rule itself
 * (where the 25% edge is, that the rep spread decides what is noise, that a
 * speed-up is reported so the baseline does not go stale, that a small board
 * is tracked like any other), the file's validation, the refusal to record on
 * a busy machine, and the command line's exit codes.
 */
import { spawnSync } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { afterAll, beforeAll, describe, expect, it } from "vitest";

import {
  BASELINE_SCHEMA,
  BUSY_LOAD_PER_CPU,
  BaselineError,
  DEFAULT_TOLERANCE,
  boardKey,
  checkAgainstBaseline,
  loadBaseline,
  updateRefusal,
  withEntry,
  writeBaseline,
  type Baseline,
  type BaselineEntry,
  type Measured,
} from "../../../scripts/perf/baseline.js";

const entry = (median: number, min = median, max = median): BaselineEntry => ({
  coreCpuMsPerSimSecond: median,
  minCpuMsPerSimSecond: min,
  maxCpuMsPerSimSecond: max,
  reps: 5,
  measuredS: 3,
  recordedAt: "2026-09-30T00:00:00.000Z",
  commit: "abc1234",
  loadAverage1min: 1,
});
const baselineOf = (boards: Record<string, BaselineEntry>): Baseline => ({
  schema: BASELINE_SCHEMA,
  toleranceFraction: DEFAULT_TOLERANCE,
  machine: { cpu: "test", cpus: 1, node: "v0", platform: "test" },
  boards,
});
/** A run whose reps sit within `spread` of the median on each side. */
const run = (median: number, spread = 0): Measured => ({ median, min: median * (1 - spread), max: median * (1 + spread) });

const baseline = baselineOf({ steady: entry(100, 98, 103), tiny: entry(4, 3.8, 4.4), wobbly: entry(0.3, 0.14, 0.79) });

describe("checkAgainstBaseline", () => {
  it("passes inside the tolerance, in both directions", () => {
    expect(checkAgainstBaseline("steady", run(100), baseline).status).toBe("ok");
    expect(checkAgainstBaseline("steady", run(124.9), baseline).status).toBe("ok");
    expect(checkAgainstBaseline("steady", run(75.1), baseline).status).toBe("ok");
  });

  it("warns above +25% when the runs' spreads do not overlap, and names the numbers", () => {
    const check = checkAgainstBaseline("steady", run(130, 0.02), baseline);
    expect(check.status).toBe("regression");
    expect(check.change).toBeCloseTo(0.3, 10);
    expect(check.message).toContain("WARNING");
    expect(check.message).toContain("130.000");
    expect(check.message).toContain("100.000");
    expect(check.message).toContain("+30.0%");
  });

  it("tracks a small board like any other: 4 ms/s becoming 12 warns", () => {
    const check = checkAgainstBaseline("tiny", run(12, 0.05), baseline);
    expect(check.status).toBe("regression");
    expect(check.change).toBeCloseTo(2, 10);
  });

  it("does not call a wobbly board's move a regression while the reps' ranges overlap", () => {
    // The median moved from 0.3 to 0.5 (+67%) but the fastest rep, 0.2, is inside the baseline's 0.14 to 0.79.
    const check = checkAgainstBaseline("wobbly", { median: 0.5, min: 0.2, max: 0.9 }, baseline);
    expect(check.status).toBe("noisy");
    expect(check.message).toContain("inside the noise");
    // A move that clears the baseline's slowest rep does warn.
    expect(checkAgainstBaseline("wobbly", { median: 1.5, min: 1.2, max: 1.9 }, baseline).status).toBe("regression");
  });

  it("notes a run more than 25% faster (spreads apart), so a stale baseline gets refreshed", () => {
    const check = checkAgainstBaseline("steady", run(60, 0.02), baseline);
    expect(check.status).toBe("faster");
    expect(check.message).toContain("--update-baseline");
    expect(checkAgainstBaseline("steady", { median: 60, min: 55, max: 99 }, baseline).status).toBe("noisy");
  });

  it("treats an entry recorded without a spread as having none: the plain +25% rule", () => {
    const legacy = baselineOf({ old: { ...entry(100), minCpuMsPerSimSecond: undefined, maxCpuMsPerSimSecond: undefined } });
    expect(checkAgainstBaseline("old", run(126, 0.01), legacy).status).toBe("regression");
    expect(checkAgainstBaseline("old", run(120, 0.01), legacy).status).toBe("ok");
  });

  it("says a busy machine makes a regression suspect", () => {
    const check = checkAgainstBaseline("steady", run(200, 0.02), baseline, { loadAverage1min: 9, cpuCount: 10 });
    expect(check.status).toBe("regression");
    expect(check.message).toContain("machine is busy");
    expect(checkAgainstBaseline("steady", run(200, 0.02), baseline, { loadAverage1min: 1, cpuCount: 10 }).message).not.toContain("busy");
  });

  it("reports a board with no entry, and a missing file, as no-baseline", () => {
    expect(checkAgainstBaseline("other", run(100), baseline).status).toBe("no-baseline");
    expect(checkAgainstBaseline("steady", run(100), undefined).status).toBe("no-baseline");
  });
});

describe("updateRefusal", () => {
  it("refuses above the busy line, allows below it, and --force overrides", () => {
    expect(updateRefusal(10 * BUSY_LOAD_PER_CPU + 0.1, 10, false)).toContain("above 0.5 per CPU");
    expect(updateRefusal(10 * BUSY_LOAD_PER_CPU - 0.1, 10, false)).toBeUndefined();
    expect(updateRefusal(50, 10, true)).toBeUndefined();
  });
});

describe("baseline file", () => {
  let dir: string;
  beforeAll(() => {
    dir = fs.mkdtempSync(path.join(os.tmpdir(), "baseline-test-"));
  });
  afterAll(() => {
    fs.rmSync(dir, { recursive: true, force: true });
  });
  const write = (name: string, text: string): string => {
    const file = path.join(dir, name);
    fs.writeFileSync(file, text);
    return file;
  };

  it("names a board by its file's basename without .sim.json", () => {
    expect(boardKey("/x/y/02-resistor-bridge.sim.json")).toBe("02-resistor-bridge");
    expect(boardKey("C:\\x\\sap1-max-ideal-frozen.sim.json")).toBe("sap1-max-ideal-frozen");
  });

  it("round-trips through writeBaseline and withEntry, keeping the other boards, the spread and sorted keys", () => {
    const file = path.join(dir, "roundtrip.json");
    writeBaseline(file, withEntry(baseline, "a-board", entry(50, 49, 52)));
    const back = loadBaseline(file)!;
    expect(Object.keys(back.boards)).toEqual(["a-board", "steady", "tiny", "wobbly"]);
    expect(back.boards["a-board"]!.maxCpuMsPerSimSecond).toBe(52);
    expect(back.boards.steady!.coreCpuMsPerSimSecond).toBe(100);
    expect(back.toleranceFraction).toBe(DEFAULT_TOLERANCE);
    expect(loadBaseline(path.join(dir, "missing.json"))).toBeUndefined();
  });

  it("says clearly, with the file's name, what is wrong with an unusable file", () => {
    const corrupt = write("corrupt.json", "{ this is not json");
    expect(() => loadBaseline(corrupt)).toThrow(BaselineError);
    expect(() => loadBaseline(corrupt)).toThrow(/corrupt\.json is not valid JSON/);
    expect(() => loadBaseline(write("schema.json", JSON.stringify({ schema: 99, toleranceFraction: 0.25, boards: {} })))).toThrow(/schema 99/);
    expect(() => loadBaseline(write("noboards.json", JSON.stringify({ schema: 1, toleranceFraction: 0.25 })))).toThrow(/"boards"/);
    expect(() => loadBaseline(write("badentry.json", JSON.stringify({ schema: 1, toleranceFraction: 0.25, boards: { b: { coreCpuMsPerSimSecond: "fast" } } })))).toThrow(/board "b"/);
    expect(() => loadBaseline(write("badmin.json", JSON.stringify({ schema: 1, toleranceFraction: 0.25, boards: { b: { coreCpuMsPerSimSecond: 5, minCpuMsPerSimSecond: -1 } } })))).toThrow(/minCpuMsPerSimSecond/);
    expect(() => loadBaseline(write("array.json", "[]"))).toThrow(BaselineError);
  });

  it("the checked-in scripts/perf/core-baseline.json is well formed and covers both SAP-1 boards", () => {
    const file = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "..", "scripts", "perf", "core-baseline.json");
    const checked = loadBaseline(file);
    expect(checked).toBeDefined();
    expect(checked!.toleranceFraction).toBe(0.25);
    for (const key of ["sap1-max-ideal-frozen", "sap1-max-ideal-running"]) {
      const e = checked!.boards[key];
      expect(e, key).toBeDefined();
      expect(e!.coreCpuMsPerSimSecond).toBeGreaterThan(0);
      expect(e!.commit).toMatch(/^[0-9a-f]{7,}$/);
    }
  });
});

describe("bench-core's baseline handling (command line, on the small default board)", () => {
  const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "..");
  const impl = path.join(repoRoot, "scripts", "bench-core-impl.ts");
  let dir: string;
  beforeAll(() => {
    dir = fs.mkdtempSync(path.join(os.tmpdir(), "bench-baseline-test-"));
  });
  afterAll(() => {
    fs.rmSync(dir, { recursive: true, force: true });
  });

  /** Runs the implementation under tsx with `env` (PATH may be emptied to hide git). */
  const bench = (args: string[], env: Record<string, string> = {}): { status: number | null; out: string } => {
    const r = spawnSync(process.execPath, ["--import", "tsx", impl, ...args], { cwd: repoRoot, encoding: "utf8", env: { ...process.env, ...env } });
    return { status: r.status, out: `${r.stdout}${r.stderr}` };
  };

  it("a corrupt baseline prints a clear error AFTER the measurement, still prints the JSON, and exits 1", () => {
    const file = path.join(dir, "corrupt.json");
    fs.writeFileSync(file, "not json at all");
    const r = bench([`--baseline=${file}`]);
    const measured = r.out.indexOf("core: median");
    const error = r.out.indexOf("ERROR: the baseline file is unusable");
    expect(measured).toBeGreaterThan(-1);
    expect(error).toBeGreaterThan(measured);
    expect(r.out).toContain("corrupt.json is not valid JSON");
    expect(r.out).toContain("--- JSON ---");
    expect(r.status).toBe(1);
  });

  it("--strict-baseline fails on a board with no baseline entry; without it the same run exits 0", () => {
    const file = path.join(dir, "empty.json");
    writeBaseline(file, baselineOf({ "some-other-board": entry(50) }));
    const lax = bench([`--baseline=${file}`]);
    expect(lax.out).toContain('no baseline recorded for "02-resistor-bridge"');
    expect(lax.status).toBe(0);
    expect(bench([`--baseline=${file}`, "--strict-baseline"]).status).toBe(1);
  });

  it("--update-baseline --force records the board with its spread, even with git unavailable (commit unknown)", () => {
    const file = path.join(dir, "recorded.json");
    const emptyPath = fs.mkdtempSync(path.join(dir, "nopath-"));
    const r = bench([`--baseline=${file}`, "--update-baseline", "--force"], { PATH: emptyPath });
    expect(r.status).toBe(0);
    const recorded = loadBaseline(file)!.boards["02-resistor-bridge"]!;
    expect(recorded.commit).toBe("unknown");
    expect(recorded.minCpuMsPerSimSecond).toBeLessThanOrEqual(recorded.coreCpuMsPerSimSecond);
    expect(recorded.maxCpuMsPerSimSecond).toBeGreaterThanOrEqual(recorded.coreCpuMsPerSimSecond);
    // A small board is recorded (there is no noise floor), and the next run is checked against it.
    expect(recorded.coreCpuMsPerSimSecond).toBeGreaterThan(0);
    const again = bench([`--baseline=${file}`]);
    expect(again.out).toMatch(/"02-resistor-bridge": .* against a baseline of/);
  });

  it("--update-baseline without --force records on a quiet machine and refuses (exit 1) on a busy one", () => {
    const file = path.join(dir, "maybe.json");
    const r = bench([`--baseline=${file}`, "--update-baseline"]);
    // The load moves during the run, so accept whichever branch the run itself reports.
    if (r.out.includes("refusing --update-baseline")) {
      expect(r.status).toBe(1);
      expect(fs.existsSync(file)).toBe(false);
    } else {
      expect(r.status).toBe(0);
      expect(fs.existsSync(file)).toBe(true);
    }
  });
});
