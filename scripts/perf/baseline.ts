/**
 * The perf-regression baseline (plan section 3.5's tracking rule): bench-core
 * compares the Node ms of CPU per simulated second of each board it runs
 * against numbers recorded in a checked-in JSON, and warns when it is more
 * than 25% slower.
 *
 * WHY A WARNING AND NOT A TEST. The number is machine-dependent and the
 * development machine is shared, so a hard gate would fail on load rather than
 * on code. bench-core prints the warning (and its own load-average warning
 * beside it); `--strict-baseline` turns it into an exit code for a quiet
 * machine or a CI runner whose baseline was recorded on itself.
 *
 * NOISE IS MEASURED, NOT ASSUMED. Every board is tracked, however small: an
 * entry keeps the median AND the fastest and slowest rep of the run that
 * recorded it. A run regresses only when its median is more than the
 * tolerance above the baseline's median AND its own FASTEST rep is above the
 * baseline's SLOWEST one, that is, when the two runs' spreads do not overlap.
 * A board that wobbles between 0.14 and 0.8 ms/s (the resistor bridge) needs a
 * large move to warn; one that holds 298 to 310 (the SAP-1) warns on a 25%
 * change; and a 4 ms/s board that becomes 12 warns. An entry recorded without
 * a spread (an older file) is treated as having none, which is the plain
 * +25% rule.
 *
 * WHY IT ALSO SPEAKS UP ABOUT BEING FASTER. A baseline recorded before a
 * speed-up would let a later 5x slowdown pass unnoticed as long as it stays
 * under the old number. A run more than the tolerance FASTER than the baseline
 * (spreads apart, again) says so, and points at `--update-baseline`.
 *
 * WHY RECORDING IS REFUSED ON A BUSY MACHINE. A baseline recorded under load
 * is inflated, and every later honest run then looks like an improvement.
 * `--update-baseline` refuses above BUSY_LOAD_PER_CPU 1-minute load per CPU
 * unless `--force`.
 *
 * The file: scripts/perf/core-baseline.json, keyed by board. A board's key is
 * its file's basename without `.sim.json` (`02-resistor-bridge`), or for the
 * generated SAP-1 `sap1-<hz>-<power>-<running|frozen>`.
 */
import fs from "node:fs";
import os from "node:os";

export const BASELINE_SCHEMA = 1;
/** Plan 3.5: warn when the Node number is more than 25% above the baseline. */
export const DEFAULT_TOLERANCE = 0.25;
/** 1-minute load average per CPU above which the machine is too busy to record a baseline on, or to trust a regression from. */
export const BUSY_LOAD_PER_CPU = 0.5;

export interface BaselineEntry {
  /** Median over the reps of the core's CPU ms per simulated second. */
  readonly coreCpuMsPerSimSecond: number;
  /** The fastest and slowest rep of the recording run: the noise the next run is judged against. Absent in an older file. */
  readonly minCpuMsPerSimSecond?: number;
  readonly maxCpuMsPerSimSecond?: number;
  readonly reps: number;
  readonly measuredS: number;
  /** When and on what it was recorded: a baseline is only comparable with runs on similar hardware. */
  readonly recordedAt: string;
  readonly commit: string;
  readonly loadAverage1min: number;
}

export interface Baseline {
  readonly schema: number;
  readonly toleranceFraction: number;
  readonly machine: { readonly cpu: string; readonly cpus: number; readonly node: string; readonly platform: string };
  readonly boards: Readonly<Record<string, BaselineEntry>>;
}

/** One run's numbers for a board: median, fastest and slowest rep, all in CPU ms per simulated second. */
export interface Measured {
  readonly median: number;
  readonly min: number;
  readonly max: number;
}

export type BaselineStatus = "ok" | "noisy" | "regression" | "faster" | "no-baseline";

export interface BaselineCheck {
  readonly key: string;
  readonly status: BaselineStatus;
  readonly measured: Measured;
  readonly baseline?: number;
  /** measured median / baseline median - 1, e.g. 0.3 for 30% slower. */
  readonly change?: number;
  readonly message: string;
}

/** A baseline file that cannot be used: unreadable JSON, wrong schema, malformed entries. The message names the file and the problem. */
export class BaselineError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "BaselineError";
  }
}

export function isBusy(loadAverage1min: number, cpuCount: number): boolean {
  return cpuCount > 0 && loadAverage1min / cpuCount > BUSY_LOAD_PER_CPU;
}

/** Why --update-baseline must not record right now (a sentence), or undefined when it may. */
export function updateRefusal(loadAverage1min: number, cpuCount: number, force: boolean): string | undefined {
  if (force || !isBusy(loadAverage1min, cpuCount)) return undefined;
  return `1-minute load average ${loadAverage1min.toFixed(2)} across ${String(cpuCount)} CPUs is above ${String(BUSY_LOAD_PER_CPU)} per CPU`;
}

export function machineDescription(): Baseline["machine"] {
  const cpus = os.cpus();
  return { cpu: cpus[0]?.model ?? "unknown", cpus: cpus.length, node: process.version, platform: `${process.platform}-${process.arch}` };
}

/** The board's key in the baseline file. */
export function boardKey(file: string): string {
  const base = file.split(/[\\/]/).pop() ?? file;
  return base.replace(/\.sim\.json$/, "").replace(/\.json$/, "");
}

const finitePositive = (x: unknown): x is number => typeof x === "number" && Number.isFinite(x) && x > 0;

/** Read and validate a baseline. `undefined` when the file does not exist; a BaselineError when it exists and is unusable. */
export function loadBaseline(file: string): Baseline | undefined {
  if (!fs.existsSync(file)) return undefined;
  let parsed: unknown;
  try {
    parsed = JSON.parse(fs.readFileSync(file, "utf8"));
  } catch (err) {
    throw new BaselineError(`${file} is not valid JSON (${err instanceof Error ? err.message : String(err)}); fix it, or delete it and run bench-core --update-baseline`);
  }
  const b = parsed as Partial<Baseline> | null;
  if (b === null || typeof b !== "object") throw new BaselineError(`${file} does not hold a JSON object`);
  if (b.schema !== BASELINE_SCHEMA) throw new BaselineError(`${file}: baseline schema ${String(b.schema)} is not ${String(BASELINE_SCHEMA)}`);
  if (!finitePositive(b.toleranceFraction)) throw new BaselineError(`${file}: "toleranceFraction" must be a positive number`);
  if (b.boards === null || typeof b.boards !== "object") throw new BaselineError(`${file}: "boards" must be an object keyed by board`);
  for (const [key, entry] of Object.entries(b.boards)) {
    const e = entry as Partial<BaselineEntry> | null;
    if (!e || !finitePositive(e.coreCpuMsPerSimSecond)) throw new BaselineError(`${file}: board "${key}" has no positive coreCpuMsPerSimSecond`);
    for (const optional of ["minCpuMsPerSimSecond", "maxCpuMsPerSimSecond"] as const) {
      if (e[optional] !== undefined && !finitePositive(e[optional])) throw new BaselineError(`${file}: board "${key}": ${optional} must be a positive number when present`);
    }
  }
  return b as Baseline;
}

/**
 * Compare one measured run against the baseline. `loadAverage1min` and
 * `cpuCount` only shape the message: a busy machine makes a "regression"
 * suspect, and the message says so instead of the check silently dropping it.
 */
export function checkAgainstBaseline(
  key: string,
  measured: Measured,
  baseline: Baseline | undefined,
  context: { loadAverage1min?: number; cpuCount?: number } = {},
): BaselineCheck {
  const entry = baseline?.boards[key];
  const show = (m: Measured): string => `${m.median.toFixed(3)} ms/simulated-s (reps ${m.min.toFixed(3)} to ${m.max.toFixed(3)})`;
  if (!baseline || !entry) {
    return { key, status: "no-baseline", measured, message: `no baseline recorded for "${key}" (${show(measured)}); bench-core --update-baseline records one` };
  }
  const tolerance = baseline.toleranceFraction;
  const base = entry.coreCpuMsPerSimSecond;
  const baseMin = entry.minCpuMsPerSimSecond ?? base;
  const baseMax = entry.maxCpuMsPerSimSecond ?? base;
  const change = measured.median / base - 1;
  const pct = (x: number): string => `${x >= 0 ? "+" : ""}${(x * 100).toFixed(1)}%`;
  const busy = context.loadAverage1min !== undefined && context.cpuCount !== undefined && isBusy(context.loadAverage1min, context.cpuCount);
  const suspect = busy ? " The machine is busy (load average high); re-run on a quiet machine before believing it." : "";
  const head = `"${key}": ${show(measured)} against a baseline of ${base.toFixed(3)} (${entry.minCpuMsPerSimSecond === undefined ? "no spread recorded" : `reps ${baseMin.toFixed(3)} to ${baseMax.toFixed(3)}`}; ${pct(change)}, recorded ${entry.recordedAt.slice(0, 10)} at ${entry.commit})`;
  const tol = `${(tolerance * 100).toFixed(0)}%`;
  if (change > tolerance) {
    if (measured.min > baseMax) {
      return { key, status: "regression", measured, baseline: base, change, message: `WARNING: core perf regression on ${head}; the limit is +${tol} and the fastest rep is above the baseline's slowest.${suspect}` };
    }
    return { key, status: "noisy", measured, baseline: base, change, message: `over +${tol} but inside the noise on ${head}: the runs' rep ranges overlap, so it is not called a regression.${suspect}` };
  }
  if (change < -tolerance) {
    if (measured.max < baseMin) {
      return { key, status: "faster", measured, baseline: base, change, message: `note: core is faster than its baseline on ${head}; run bench-core --update-baseline (on a quiet machine) so a later slowdown is caught against the new number.` };
    }
    return { key, status: "noisy", measured, baseline: base, change, message: `under -${tol} but inside the noise on ${head}: the runs' rep ranges overlap.` };
  }
  return { key, status: "ok", measured, baseline: base, change, message: `within tolerance on ${head}` };
}

/** A new baseline with `key` set (the rest kept), for --update-baseline. */
export function withEntry(baseline: Baseline | undefined, key: string, entry: BaselineEntry): Baseline {
  return {
    schema: BASELINE_SCHEMA,
    toleranceFraction: baseline?.toleranceFraction ?? DEFAULT_TOLERANCE,
    machine: machineDescription(),
    boards: { ...baseline?.boards, [key]: entry },
  };
}

export function writeBaseline(file: string, baseline: Baseline): void {
  const sorted = Object.fromEntries(Object.entries(baseline.boards).sort(([a], [b]) => a.localeCompare(b)));
  fs.writeFileSync(file, `${JSON.stringify({ ...baseline, boards: sorted }, null, 2)}\n`);
}
