/**
 * Per-frame CPU time of the Rust/WASM core at 1/60 s on both SAP-1 boards, in
 * Node. This is the number behind plan section 3.5's "p99 per-frame worker
 * time" budget: bench-core reports a mean per simulated second, but a worker
 * that misses one frame in a hundred drops a visible frame, and clock-edge
 * bursts (many events in one frame) are what the plan flags as the likely
 * failure. The browser proxy (Chrome, CDP CPU throttling at R=10, the real
 * worker) is W1z's; this is the Node-only view of the same quantity.
 *
 * Usage (from a worktree, after `pnpm run build:core` there):
 *   pnpm exec tsx scripts/perf/frames.ts [--boards <dir>] [--wasm <file>] [--seconds 10]
 *       [--warmup 1] [--reps 3] [--frame <s>] [--only frozen|running] [--worst 3] [--json <file>]
 *
 * WHAT A FRAME IS. One `engine.advance(simTime + 1/60)` followed by what a host
 * reads each frame (node voltages, element currents, digital state, channel
 * currents, failures): the same body as bench-core's core loop, so the mean of
 * the frame times here matches its ms per simulated second divided by 60.
 * Frame CPU time is process.cpuUsage() around that body (user + system, so it
 * includes any GC or JIT thread the frame woke); frame wall time is hrtime
 * around it. Both are reported, because on a busy machine the wall figure
 * carries the scheduler's noise and the CPU figure does not.
 *
 * PERCENTILES. p50, p95 and p99 (plan section 3.5 budgets p99), the mean and
 * the max, by nearest rank. A p99 needs at least 100 frames to mean anything
 * different from the max: --seconds must cover at least one frame, and the
 * default 10 s gives 600.
 *
 * REPS. The board is loaded fresh for every rep and warmed up for --warmup
 * seconds of simulated time (default 1: the load's settling transient, the
 * caches, and the engine's first program steps must not be in the numbers),
 * then run for --seconds. With --reps > 1 each
 * statistic printed is the median over the reps, and the "worst" column is
 * the largest single frame of any rep.
 */
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { pathToFileURL } from "node:url";

import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { __resetCoreForTest, coreCall, initCore } from "../../src/core/wasm.js";
import { DEFAULT_GENERATOR, defaultBoardsDir, loadSap1, programNibbles, sap1BoardFiles, type Sap1Variant } from "./boards.js";

/** bo_engine_stat's index for the events counter (core/README.md). */
const STAT_EVENTS = 4;
/** The plan's browser proxy divides the machine by this (CDP Emulation.setCPUThrottlingRate) and budgets 8 ms per frame at that rate. */
const THROTTLE_RATE = 10;
const BUDGET_P99_THROTTLED_MS = 8;

interface Args {
  boardsDir: string;
  generator: string;
  wasm?: string;
  seconds: number;
  warmupS: number;
  reps: number;
  frameS: number;
  only?: Sap1Variant;
  worst: number;
  json?: string;
}

function usage(message: string): never {
  process.stderr.write(`frames: ${message}\nusage: pnpm exec tsx scripts/perf/frames.ts [--boards dir] [--wasm file] [--seconds n] [--warmup n] [--reps n] [--frame s] [--only frozen|running] [--worst n] [--json file]\n`);
  process.exit(2);
}

function parseArgs(argv: string[]): Args {
  const args: Args = { boardsDir: defaultBoardsDir(), generator: DEFAULT_GENERATOR, seconds: 10, warmupS: 1, reps: 3, frameS: 1 / 60, worst: 3 };
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i]!;
    const value = (): string => {
      const v = argv[(i += 1)];
      if (v === undefined) usage(`${a} needs a value`);
      return v;
    };
    const num = (): number => {
      const n = Number(value());
      if (!Number.isFinite(n) || n <= 0) usage(`${a} needs a positive number`);
      return n;
    };
    if (a === "--boards") args.boardsDir = path.resolve(value());
    else if (a === "--generator") args.generator = value();
    else if (a === "--wasm") args.wasm = path.resolve(value());
    else if (a === "--seconds") args.seconds = num();
    else if (a === "--warmup") {
      const n = Number(value());
      if (!Number.isFinite(n) || n < 0) usage("--warmup needs a number of seconds >= 0");
      args.warmupS = n;
    }
    else if (a === "--reps") args.reps = Math.max(1, Math.round(num()));
    else if (a === "--frame") args.frameS = num();
    else if (a === "--worst") args.worst = Math.max(0, Math.round(num()));
    else if (a === "--json") args.json = path.resolve(value());
    else if (a === "--only") {
      const v = value();
      if (v !== "frozen" && v !== "running") usage("--only takes frozen or running");
      args.only = v;
    } else usage(`unknown option ${a}`);
  }
  // Fewer than one frame measures nothing, and Math.round would silently turn it into zero frames and NaN statistics.
  if (args.seconds < args.frameS) usage(`--seconds ${String(args.seconds)} is shorter than one frame (${String(args.frameS)} s): nothing would be measured`);
  return args;
}

export interface FrameSample {
  readonly cpuMs: number;
  readonly wallMs: number;
  readonly events: number;
  /** The advance target, in simulated seconds. */
  readonly t: number;
}

export interface FrameStats {
  readonly frames: number;
  readonly min: number;
  readonly median: number;
  readonly mean: number;
  readonly p95: number;
  readonly p99: number;
  readonly max: number;
}

/** Nearest-rank percentile of an ascending-sorted array. */
export function percentile(sorted: readonly number[], q: number): number {
  if (sorted.length === 0) return NaN;
  const rank = Math.min(sorted.length, Math.max(1, Math.ceil(q * sorted.length)));
  return sorted[rank - 1]!;
}

export function frameStats(values: readonly number[]): FrameStats {
  const sorted = [...values].sort((a, b) => a - b);
  const sum = sorted.reduce((x, y) => x + y, 0);
  return {
    frames: sorted.length,
    min: sorted[0] ?? NaN,
    median: percentile(sorted, 0.5),
    mean: sorted.length > 0 ? sum / sorted.length : NaN,
    p95: percentile(sorted, 0.95),
    p99: percentile(sorted, 0.99),
    max: sorted[sorted.length - 1] ?? NaN,
  };
}

function medianOf(values: readonly number[]): number {
  return percentile([...values].sort((a, b) => a - b), 0.5);
}

/** One rep: a fresh engine on the live wasm, warmed up, then `seconds` of frames. */
export function measureFrames(circuit: ReturnType<typeof loadSap1>, frameS: number, seconds: number, warmupS = 1): { samples: FrameSample[]; simulatedS: number; converged: boolean } {
  const engine = new CoreEngine();
  try {
    engine.load(circuit, CORE_IMPLEMENTED_KINDS);
    engine.setMemoryContents(circuit.bench.ramLo, programNibbles(circuit.bench.program, 0));
    engine.setMemoryContents(circuit.bench.ramHi, programNibbles(circuit.bench.program, 4));
    const handle = (engine as unknown as { _handle: number })._handle;
    const events = (): number => coreCall((ex) => ex.bo_engine_stat(handle, STAT_EVENTS));

    // Warm-up: the load's settling transient, the caches and the JIT, all outside the measurement.
    const warmupFrames = Math.round(warmupS / frameS);
    for (let f = 0; f < warmupFrames; f += 1) engine.advance(engine.simTime + frameS);

    const frames = Math.round(seconds / frameS);
    const samples: FrameSample[] = [];
    const start = engine.simTime;
    let converged = true;
    for (let f = 0; f < frames; f += 1) {
      const tEnd = engine.simTime + frameS;
      const e0 = events();
      const cpu0 = process.cpuUsage();
      const w0 = process.hrtime.bigint();
      engine.advance(tEnd);
      void engine.getNetV();
      void engine.getElementI();
      void engine.digitalState;
      void engine.getElementChannelI();
      void engine.getFailures();
      const wallMs = Number(process.hrtime.bigint() - w0) / 1e6;
      const cpu = process.cpuUsage(cpu0);
      if (!engine.lastConverged) converged = false;
      samples.push({ cpuMs: (cpu.user + cpu.system) / 1000, wallMs, events: events() - e0, t: tEnd });
    }
    return { samples, simulatedS: engine.simTime - start, converged };
  } finally {
    engine.dispose();
  }
}

const ms = (x: number): string => x.toFixed(3).padStart(8);

async function main(): Promise<void> {
  const args = parseArgs(process.argv.slice(2));
  let boards: ReturnType<typeof sap1BoardFiles>;
  try {
    boards = sap1BoardFiles({ boardsDir: args.boardsDir, generator: args.generator });
  } catch (err) {
    return usage(err instanceof Error ? err.message : String(err));
  }
  const wasmSource = args.wasm ? fs.readFileSync(args.wasm) : undefined;
  const load = os.loadavg();
  process.stdout.write(`frames: ${args.wasm ?? "src/core/brownout-core.wasm (default)"}\n`);
  process.stdout.write(`        ${String(args.reps)} rep(s) x ${String(args.seconds)} s after a ${String(args.warmupS)} s warm-up, at ${(args.frameS * 1000).toFixed(3)} ms frames, load average ${load.map((x) => x.toFixed(2)).join(" ")} on ${String(os.cpus().length)} CPUs\n`);

  const report: Record<string, unknown> = { wasm: args.wasm ?? "default", frameS: args.frameS, seconds: args.seconds, warmupS: args.warmupS, reps: args.reps, loadAverage: load, boards: {} as Record<string, unknown> };
  for (const board of boards) {
    if (args.only && board.variant !== args.only) continue;
    const circuit = loadSap1(board.file);
    const perRep: { cpu: FrameStats; wall: FrameStats; msPerSimS: number; samples: FrameSample[] }[] = [];
    for (let rep = 0; rep < args.reps; rep += 1) {
      __resetCoreForTest();
      await initCore(wasmSource);
      const run = measureFrames(circuit, args.frameS, args.seconds, args.warmupS);
      if (!run.converged) process.stdout.write(`  warning: ${board.name} rep ${String(rep)} hit a non-convergent advance; its frames are not representative\n`);
      const cpuTotal = run.samples.reduce((x, s) => x + s.cpuMs, 0);
      perRep.push({ cpu: frameStats(run.samples.map((s) => s.cpuMs)), wall: frameStats(run.samples.map((s) => s.wallMs)), msPerSimS: cpuTotal / run.simulatedS, samples: run.samples });
    }
    const med = (pick: (r: (typeof perRep)[number]) => number): number => medianOf(perRep.map(pick));
    const all = perRep.flatMap((r) => r.samples);
    const worstCpu = Math.max(...all.map((s) => s.cpuMs));
    const cpuRow = (label: string, pick: (s: FrameStats) => number): string => `${label}${ms(med((r) => pick(r.cpu)))} ms cpu   ${ms(med((r) => pick(r.wall)))} ms wall`;

    process.stdout.write(`\n${board.name}  (${String(perRep[0]!.cpu.frames)} frames per rep; ${med((r) => r.msPerSimS).toFixed(1)} ms CPU per simulated second, median of reps)\n`);
    process.stdout.write(`  ${cpuRow("p50    ", (s) => s.median)}\n  ${cpuRow("mean   ", (s) => s.mean)}\n  ${cpuRow("p95    ", (s) => s.p95)}\n  ${cpuRow("p99    ", (s) => s.p99)}\n  ${cpuRow("max    ", (s) => s.max)}\n`);
    process.stdout.write(`  worst single frame of any rep: ${worstCpu.toFixed(3)} ms cpu\n`);
    const p99 = med((r) => r.cpu.p99);
    process.stdout.write(
      `  plan 3.5 proxy: p99 ${p99.toFixed(3)} ms x ${String(THROTTLE_RATE)} (CPU throttle) = ${(p99 * THROTTLE_RATE).toFixed(1)} ms against the ${String(BUDGET_P99_THROTTLED_MS)} ms budget (${p99 * THROTTLE_RATE <= BUDGET_P99_THROTTLED_MS ? "inside" : "OVER"}); the browser measurement is W1z's\n`,
    );
    const medianEvents = medianOf(all.map((s) => s.events));
    const worst = [...all].sort((a, b) => b.cpuMs - a.cpuMs).slice(0, args.worst);
    for (const s of worst) {
      process.stdout.write(`  slow frame: ${s.cpuMs.toFixed(3)} ms cpu at t=${s.t.toFixed(4)} s, ${String(s.events)} events (a median frame handles ${String(medianEvents)})\n`);
    }
    (report.boards as Record<string, unknown>)[board.name] = {
      msPerSimSecond: med((r) => r.msPerSimS),
      cpuMs: { p50: med((r) => r.cpu.median), mean: med((r) => r.cpu.mean), p95: med((r) => r.cpu.p95), p99, max: med((r) => r.cpu.max), worst: worstCpu },
      wallMs: { p50: med((r) => r.wall.median), mean: med((r) => r.wall.mean), p95: med((r) => r.wall.p95), p99: med((r) => r.wall.p99), max: med((r) => r.wall.max) },
    };
  }
  if (args.json) fs.writeFileSync(args.json, `${JSON.stringify(report, null, 2)}\n`);
}

const isMain = process.argv[1] !== undefined && import.meta.url === pathToFileURL(process.argv[1]).href;
if (isMain) {
  main().catch((err: unknown) => {
    process.stderr.write(`frames: ${err instanceof Error ? (err.stack ?? err.message) : String(err)}\n`);
    process.exit(1);
  });
}
