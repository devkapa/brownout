/**
 * scripts/bench-core.mjs's real implementation (spawned via tsx -- see that
 * file's header for why). devolt memory/decisions/2026-09-25-brownout-core-
 * w1-plan.md, section 3.5, "Performance benchmark protocol":
 *
 *   - classic: HeadlessRunner's default engine (default adaptive stepping),
 *     0.1 s warm-up, 1 s measured, 3 repetitions.
 *   - core: CoreEngine directly -- load, (RAM preload, once a memory kind
 *     exists -- none in W1a), 0.1 s warm-up, then advance in 1/60 s frames
 *     for 3 s of simulated time, reading every frame what a host would read
 *     (node voltages, element currents, digital slots, channel currents,
 *     failures), 5 repetitions.
 *   - both measured with process.cpuUsage() (user+system) and hrtime;
 *     median/min/max reported, plus the core's own kernel stats.
 *   - the 1-minute load average is logged (a warning, not a silent discard --
 *     a script cannot itself decide a run is untrustworthy and hide it).
 *
 * DECISION 12 (SAP-1 generator location): kept outside both repos, read by
 * path. `--sap1` shells out to it exactly as a person would from the CLI
 * (`node gen-sap1.mjs <hz> <power> <outFile>`) and reads back the .sim.json
 * it writes -- there is no importable API to call instead (see this file's
 * own `runSap1Generator` for what was actually found there).
 *
 * WHY THE SAP-1 PATH SKIPS THE CLASSIC NUMBER BY DEFAULT
 * Section 3.4.2's gate 2g measured classic at 50-120 s of CPU per simulated
 * second on the full board; the CLI's own generator can also be pointed at a
 * low clock rate, which does not fix this (Newton/MNA cost per step tracks
 * circuit SIZE, not the clock's own frequency). Since decision 12's own text
 * only asks W1a to report "not portable" with the missing kinds for the SAP-1
 * -- not to also pay for a classic run at every dispatch -- `--sap1` skips
 * the classic measurement unless `--sap1-classic` is passed explicitly. Every
 * OTHER board (a plain file argument, including a sub-board like devolt's
 * clock module) always gets the classic number: those are small enough that
 * skipping it would just be throwing away useful, cheap data.
 *
 * THE WEAK-DEVICE PROXY THIS NUMBER WILL EVENTUALLY BE CHECKED AGAINST
 * (decision 16, wired starting this wave, enforced from W1z): Chrome on the
 * reference machine with CDP `Emulation.setCPUThrottlingRate` at R=10 (~10x
 * slower than an M5), sim/wall >= 0.99 over 60 s, p99 per-FRAME worker time
 * <= 8 ms, on every acceptance board, counted only when a calibration kernel
 * proves the throttle reached the worker. None of that is measured here --
 * this script has no browser, no worker thread and no CDP driver (that is
 * Z1's `bench/browser/`, W1z). It is Node-only on purpose (the worker
 * brief). Reading this script's own numbers against that budget: the plan's
 * worked example turns a 2.7 ms/simulated-second ideal-supply WASM figure
 * into ~27 ms/s at R=10 -- comfortably inside the per-FRAME budget at a
 * 60 Hz frame rate, but frame time and per-simulated-second time are
 * different units, and the plan itself flags clock-edge event BURSTS, not
 * the steady-state average this script reports, as the thing likely to blow
 * the per-frame budget once digital kinds exist.
 */
import { spawnSync } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

import { CoreEngine, unsupportedCoreKinds, CORE_IMPLEMENTED_KINDS } from "../src/core.js";
import { HeadlessRunner } from "../src/host.js";
import type { HeadlessRunResult } from "../src/host.js";
import { initCore } from "../src/core/wasm.js";
import type { SimCircuit } from "../src/sim/engine/sim-engine.js";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(scriptDir, "..");

const FRAME_S = 1 / 60;
const CORE_WARMUP_S = 0.1;
const CORE_MEASURED_S = 3;
const CORE_REPETITIONS = 5;
const CLASSIC_WARMUP_S = 0.1;
const CLASSIC_MEASURED_S = 1;
const CLASSIC_REPETITIONS = 3;

/** Decision 12: the generator lives outside both repos, read by path. This is the documented, fixed location; --sap1-generator overrides it. */
const DEFAULT_SAP1_GENERATOR = "/Users/dev/Documents/DEVOLTProject/brownout-core-prototype/bench/gen-sap1.mjs";
const DEFAULT_BOARD = path.join(repoRoot, "test", "fixtures", "core", "02-resistor-bridge.sim.json");

// ---- CLI --------------------------------------------------------------------

interface ParsedArgs {
  boardPaths: string[];
  sap1: boolean;
  sap1Hz: string;
  sap1Power: string;
  sap1Generator: string;
  sap1RunClassicToo: boolean;
}

function parseArgs(argv: string[]): ParsedArgs {
  const boardPaths: string[] = [];
  let sap1 = false;
  let sap1Hz = "max";
  let sap1Power = "ideal";
  let sap1Generator = DEFAULT_SAP1_GENERATOR;
  let sap1RunClassicToo = false;

  for (const arg of argv) {
    if (arg === "--sap1") {
      sap1 = true;
    } else if (arg.startsWith("--sap1=")) {
      sap1 = true;
      const [hz, power] = arg.slice("--sap1=".length).split(",");
      if (hz) sap1Hz = hz;
      if (power) sap1Power = power;
    } else if (arg.startsWith("--sap1-generator=")) {
      sap1Generator = arg.slice("--sap1-generator=".length);
    } else if (arg === "--sap1-classic") {
      sap1RunClassicToo = true;
    } else if (arg.startsWith("--")) {
      throw new Error(`bench-core: unknown flag ${arg}`);
    } else {
      boardPaths.push(arg);
    }
  }

  if (!sap1 && boardPaths.length === 0) boardPaths.push(DEFAULT_BOARD);
  return { boardPaths, sap1, sap1Hz, sap1Power, sap1Generator, sap1RunClassicToo };
}

// ---- SAP-1 generator (decision 12) -------------------------------------------

/**
 * Shells out to the external generator exactly as its own header comment
 * documents it (`node gen-sap1.mjs <clockHz|max> <ideal|battery> [out.sim.json]`
 * -- it is a CLI script that writes a file, not an importable module; see
 * this repo's copy of the same convention in scripts/build-core.mjs for why
 * shelling out rather than guessing at an internal API is the honest choice
 * here too).
 */
function runSap1Generator(generatorPath: string, hz: string, power: string): SimCircuit {
  if (!fs.existsSync(generatorPath)) {
    throw new Error(
      `bench-core: --sap1 generator not found at ${generatorPath}\n` +
        "  Decision 12 keeps it outside both repos; pass --sap1-generator=<path> if it lives " +
        "somewhere other than the documented default.",
    );
  }
  const outFile = path.join(os.tmpdir(), `bench-core-sap1-${String(process.pid)}.sim.json`);
  const result = spawnSync(process.execPath, [generatorPath, hz, power, outFile], {
    cwd: path.dirname(generatorPath),
    encoding: "utf8",
  });
  if (result.error || result.status !== 0) {
    throw new Error(
      `bench-core: gen-sap1 failed (${result.error?.message ?? `exit ${String(result.status)}`}):\n${result.stderr ?? ""}`,
    );
  }
  try {
    const text = fs.readFileSync(outFile, "utf8");
    return JSON.parse(text) as SimCircuit;
  } finally {
    fs.rmSync(outFile, { force: true });
  }
}

// ---- measurement --------------------------------------------------------------

/**
 * Why a rep can stop short of the requested duration, named so the report
 * can say which one happened instead of silently understating the ratio
 * (the plan's own protocol note in this file's header only budgets for the
 * happy path -- it says nothing about what to do when the classic engine's
 * step cap or H_MIN floor, or the core's own non-convergence, cuts a
 * measured rep short; core/README.md's "A failed advance" is the core's
 * side of the same idea).
 */
type EarlyStop = "hitStepCap" | "hitMinStep" | "coreNonConvergence";

interface Rep {
  cpuMs: number;
  wallMs: number;
  /** Simulated seconds this rep's MEASURED window actually covered -- result.simulatedS for classic, the core clock's own delta for the core. Never the requested duration; that is requestedS on Summary. */
  simulatedS: number;
  /** Set when this rep did not cover the requested duration, and why. */
  earlyStop?: EarlyStop;
}

interface Summary {
  repetitions: number;
  /** The duration each rep was ASKED to cover -- for display and for the "requested vs. covered" comparison in an early-stop report. Never the divisor for cpuMsPerSimSecond (see Rep.simulatedS). */
  requestedS: number;
  cpuMsPerRun: { median: number; min: number; max: number };
  cpuMsPerSimSecond: { median: number; min: number; max: number };
  wallMsPerRun: { median: number; min: number; max: number };
  /** Non-empty when any rep stopped short of requestedS -- printSummaryLine reports this instead of a ms/simulated-s number, since that number would describe a smaller, silently-different measurement window than the protocol calls for. */
  earlyStops: { rep: number; reason: EarlyStop; simulatedS: number }[];
  raw: Rep[];
}

function median(values: number[]): number {
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.floor(sorted.length / 2)]!;
}

function summarize(reps: Rep[], requestedS: number): Summary {
  const byCpu = [...reps].sort((a, b) => a.cpuMs - b.cpuMs);
  const byWall = [...reps].sort((a, b) => a.wallMs - b.wallMs);
  const mid = Math.floor(reps.length / 2);
  // Each rep is divided by the simulated time IT covered, not by a shared
  // requested duration: a rep that stopped early would otherwise understate
  // its true per-simulated-second cost by exactly the fraction it missed.
  const ratios = reps.map((r) => r.cpuMs / r.simulatedS);
  const sortedRatios = [...ratios].sort((a, b) => a - b);
  const earlyStops = reps.flatMap((r, i) =>
    r.earlyStop ? [{ rep: i, reason: r.earlyStop, simulatedS: r.simulatedS }] : [],
  );
  return {
    repetitions: reps.length,
    requestedS,
    cpuMsPerRun: { median: byCpu[mid]!.cpuMs, min: byCpu[0]!.cpuMs, max: byCpu[byCpu.length - 1]!.cpuMs },
    cpuMsPerSimSecond: {
      median: median(ratios),
      min: sortedRatios[0]!,
      max: sortedRatios[sortedRatios.length - 1]!,
    },
    wallMsPerRun: { median: byWall[mid]!.wallMs, min: byWall[0]!.wallMs, max: byWall[byWall.length - 1]!.wallMs },
    earlyStops,
    raw: reps,
  };
}

function timeIt<T>(fn: () => T): { result: T; cpuMs: number; wallMs: number } {
  const cpuStart = process.cpuUsage();
  const hrStart = process.hrtime.bigint();
  const result = fn();
  const wallMs = Number(process.hrtime.bigint() - hrStart) / 1e6;
  const cpu = process.cpuUsage(cpuStart);
  return { result, cpuMs: (cpu.user + cpu.system) / 1000, wallMs };
}

/** "reading each frame what the host will read" (section 3.5) -- every readout a real host adapter surfaces, discarded. */
function readEverything(engine: CoreEngine): void {
  void engine.getNetV();
  void engine.getElementI();
  void engine.digitalState;
  void engine.getElementChannelI();
  void engine.getFailures();
}

function benchCore(circuit: SimCircuit): { summary: Summary; stats: ReturnType<CoreEngine["stats"]> } {
  const reps: Rep[] = [];
  let lastStats: ReturnType<CoreEngine["stats"]> | undefined;
  const warmupFrames = Math.round(CORE_WARMUP_S / FRAME_S);
  const measuredFrames = Math.round(CORE_MEASURED_S / FRAME_S);

  for (let rep = 0; rep < CORE_REPETITIONS; rep++) {
    const engine = new CoreEngine();
    try {
      engine.load(circuit);
      // (RAM preload per decision 2 would run here once a memory kind exists.)
      for (let f = 0; f < warmupFrames; f++) engine.advance(engine.simTime + FRAME_S);

      // The core clock is the "real" time this rep covers: on a
      // non-convergent advance() (core/README.md's "A failed advance", -4 or
      // -7) `simTime` stops at the last converged instant instead of
      // reaching the requested tEnd, exactly like the classic engine's
      // hitMinStep/hitStepCap. Measuring from the clock itself (rather than
      // assuming CORE_MEASURED_S was fully covered) catches that the same
      // way the classic path below now does.
      const startSimTime = engine.simTime;
      let nonConverged = false;
      const { cpuMs, wallMs } = timeIt(() => {
        for (let f = 0; f < measuredFrames; f++) {
          engine.advance(engine.simTime + FRAME_S);
          if (!engine.lastConverged) nonConverged = true;
          readEverything(engine);
        }
      });
      reps.push({
        cpuMs,
        wallMs,
        simulatedS: engine.simTime - startSimTime,
        earlyStop: nonConverged ? "coreNonConvergence" : undefined,
      });
      lastStats = engine.stats();
    } finally {
      engine.dispose();
    }
  }

  if (!lastStats) throw new Error("bench-core: no core repetitions ran");
  return { summary: summarize(reps, CORE_MEASURED_S), stats: lastStats };
}

function benchClassic(circuit: SimCircuit): Summary {
  const reps: Rep[] = [];
  for (let rep = 0; rep < CLASSIC_REPETITIONS; rep++) {
    const runner = new HeadlessRunner(); // engine: "classic" (default) -- byte-for-byte today's baseline path.
    runner.load(circuit);
    runner.run({ durationS: CLASSIC_WARMUP_S });
    let measured: HeadlessRunResult | undefined;
    const { cpuMs, wallMs } = timeIt(() => {
      measured = runner.run({ durationS: CLASSIC_MEASURED_S });
    });
    // timeIt's fn() always runs before returning, so `measured` is set here;
    // the `undefined` in its type is only for the pre-call declaration.
    if (!measured) throw new Error("bench-core: classic measured run produced no result");
    const result: HeadlessRunResult = measured;
    reps.push({
      cpuMs,
      wallMs,
      simulatedS: result.simulatedS,
      earlyStop: result.hitStepCap ? "hitStepCap" : result.hitMinStep ? "hitMinStep" : undefined,
    });
  }
  return summarize(reps, CLASSIC_MEASURED_S);
}

// ---- reporting ----------------------------------------------------------------

function fmt(n: number): string {
  return n.toFixed(3);
}

function printSummaryLine(label: string, s: Summary): void {
  if (s.earlyStops.length > 0) {
    // Finding 14: printing a ms/simulated-s number here -- even one
    // correctly divided by the shorter covered time -- would still describe
    // a measurement window smaller than the documented protocol ("3 reps of
    // 1 s simulated") without saying so, which is exactly the silent
    // understatement this branch exists to stop. Name the stop and the
    // actual coverage instead of a number a reader would otherwise take at
    // face value.
    const reasons = [...new Set(s.earlyStops.map((e) => e.reason))].join(", ");
    const coverage = s.earlyStops.map((e) => `rep ${String(e.rep)}: ${fmt(e.simulatedS)}s simulated`).join(", ");
    console.log(
      `    ${label}: EARLY STOP (${reasons}) -- ${String(s.earlyStops.length)} of ${String(s.repetitions)} reps ` +
        `did not cover the requested ${fmt(s.requestedS)} s simulated (${coverage}); ms/simulated-s withheld ` +
        "because the measurement window was cut short (core/README.md's \"A failed advance\"; " +
        "HeadlessRunResult's hitStepCap/hitMinStep).",
    );
    return;
  }
  console.log(
    `    ${label}: median ${fmt(s.cpuMsPerSimSecond.median)} ms/simulated-s ` +
      `(min ${fmt(s.cpuMsPerSimSecond.min)}, max ${fmt(s.cpuMsPerSimSecond.max)}; ` +
      `${String(s.repetitions)} reps of ${fmt(s.requestedS)} s simulated)`,
  );
}

interface BoardReport {
  board: string;
  kinds: string[];
  classic?: Summary;
  core?: { summary: Summary; stats: ReturnType<CoreEngine["stats"]> };
  corePortable: boolean;
  missingKinds: string[];
  skippedClassic?: string;
}

function benchBoard(label: string, circuit: SimCircuit, opts: { runClassic: boolean }): BoardReport {
  const kinds = [...new Set(circuit.components.map((c) => c.kind))].sort();
  const missingKinds = unsupportedCoreKinds(circuit, CORE_IMPLEMENTED_KINDS);
  const corePortable = missingKinds.length === 0;

  console.log(`\n${label}`);
  console.log(`  kinds: ${kinds.join(", ") || "(none)"}`);

  const report: BoardReport = { board: label, kinds, corePortable, missingKinds };

  if (!corePortable) {
    console.log(`  core: not portable -- missing kinds: ${missingKinds.join(", ")}`);
  } else {
    console.log("  core: running...");
    report.core = benchCore(circuit);
    printSummaryLine("core", report.core.summary);
    console.log(
      `    core kernel stats (last rep): partitions=${String(report.core.stats.partitions)}, ` +
        `staticSolves=${String(report.core.stats.staticSolves)}, ` +
        `largestPartition=${String(report.core.stats.largestPartition)}, ` +
        `fixedNodes=${String(report.core.stats.fixedNodes)}`,
    );
  }

  if (opts.runClassic) {
    console.log("  classic: running...");
    report.classic = benchClassic(circuit);
    printSummaryLine("classic", report.classic);
  } else {
    report.skippedClassic = "skipped (--sap1 without --sap1-classic; see this file's header comment)";
    console.log(`  classic: ${report.skippedClassic}`);
  }

  return report;
}

async function main(): Promise<void> {
  const args = parseArgs(process.argv.slice(2));
  await initCore();

  const loadAvg1min = os.loadavg()[0]!;
  const cpuCount = os.cpus().length;
  console.log(`bench-core: 1-minute load average ${fmt(loadAvg1min)} across ${String(cpuCount)} CPUs.`);
  if (loadAvg1min / cpuCount > 1) {
    console.log(
      "  WARNING: this machine looks busy (load average exceeds CPU count) -- per the W1 plan's " +
        "own protocol note, treat this run's numbers as suspect and re-run on a quiet machine " +
        "before comparing against a tracked baseline.",
    );
  }

  const reports: BoardReport[] = [];

  for (const boardPath of args.boardPaths) {
    const resolved = path.resolve(boardPath);
    const circuit = JSON.parse(fs.readFileSync(resolved, "utf8")) as SimCircuit;
    reports.push(benchBoard(path.relative(repoRoot, resolved) || resolved, circuit, { runClassic: true }));
  }

  if (args.sap1) {
    const circuit = runSap1Generator(args.sap1Generator, args.sap1Hz, args.sap1Power);
    reports.push(
      benchBoard(`SAP-1 (gen-sap1 ${args.sap1Hz} Hz, ${args.sap1Power}, via ${args.sap1Generator})`, circuit, {
        runClassic: args.sap1RunClassicToo,
      }),
    );
  }

  console.log("\n--- JSON ---");
  console.log(JSON.stringify({ loadAvg1min, cpuCount, boards: reports }, null, 2));
}

// Guarded so a test can `import` this module for its pure pieces (summarize,
// printSummaryLine, the Rep/Summary/EarlyStop shapes) without also kicking
// off a real CLI run (argv parsing, initCore(), reading a board file from
// disk) as an import side effect. `tsx implFile ...` (bench-core.mjs's own
// launch) sets process.argv[1] to this file's path, so the guard is a no-op
// for every real invocation.
const isMain = process.argv[1] !== undefined && import.meta.url === pathToFileURL(process.argv[1]).href;
if (isMain) {
  main().catch((err: unknown) => {
    console.error(err instanceof Error ? err.stack ?? err.message : err);
    process.exitCode = 1;
  });
}

export type { Rep, Summary, EarlyStop };
export { summarize, printSummaryLine };
