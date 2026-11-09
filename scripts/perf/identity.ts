/**
 * Bit-identity check between two builds of the Rust/WASM core (perf wave P).
 *
 * Usage (from a worktree, after `pnpm run build:core` there):
 *   pnpm exec tsx scripts/perf/identity.ts <base.wasm> <candidate.wasm> [options]
 *   pnpm exec tsx scripts/perf/identity.ts <base.wasm> src/core/brownout-core.wasm --boards <dir>
 *   pnpm exec tsx scripts/perf/identity.ts <base.wasm> src/core/brownout-core.wasm --boards <dir> --full   # the final gate
 *
 * It loads each wasm on the same list of circuits, advances both in the same
 * frames, takes the same readings, and prints the FIRST bit difference per
 * circuit: which reading, which array, which key, both values and both bit
 * patterns. Exit code 0 means every circuit was identical; 1 means at least
 * one was not, or one was not really checked (see "A PASS HAS TO MEAN
 * SOMETHING"); 2 means a usage error.
 *
 * WHAT IS READ AT EACH READING (scripts/perf/identity-lib.ts has the detail):
 * node voltages, device currents, digital slots, channel currents, the digital
 * trace, the EEPROM update buffer (drained once per reading), the clock and
 * the two next-due times; and for every advance its return code and bo_error
 * text. A reading is taken right after load, right after every setParams and
 * after every advance (every Nth with --read-every). Unless --no-double-read,
 * each reading is followed by a second one in the reverse order, and the two
 * must agree: a compute-on-read cache has to survive being asked twice and in
 * either order.
 *
 * CIRCUITS
 *   - the two SAP-1 boards ("max" clock, ideal supply): frozen display and
 *     running display, from `--boards <dir>` (files named
 *     sap1-max-ideal-frozen.sim.json and sap1-max-ideal-running.sim.json;
 *     `--frozen <file>` / `--running <file>` name them one by one), or
 *     generated with the prototype's generator into `--boards <dir>` when
 *     they are missing (`--generator <path>` overrides its location, as in
 *     scripts/core-accept-sap1.ts);
 *   - every fixture in test/fixtures/core/, with the phases its
 *     .expected.json declares replayed through setParams, for
 *     max(--fixture-seconds, the sidecar's durationS, the sum of its phases).
 *     Fixtures run at 1/60 s and at 1 ms frames.
 *
 * LOAD OPTIONS (identity-lib.ts's OPTION_SETS, `--options a,b,c|all`): every
 * fixture runs under default, noSolutionCache, noRailShortcut, forceStepped,
 * integrationTrap and exact (noSolutionCache + noRailShortcut); the SAP-1
 * boards under default and exact. The summary reports each option.
 *
 * A PASS HAS TO MEAN SOMETHING. A run fails when:
 *   - a circuit's outputs differ, or either build traps or errors;
 *   - a SAP-1 board, or any fixture not declined by design, declines to load
 *     (identical declines on both sides would otherwise pass without
 *     comparing a single number); the fixtures declined by design are the ones
 *     whose sidecar says "core-declined", and their decline message must match;
 *   - a fixture declined by design loads;
 *   - a build answers two reads of the same state differently;
 *   - no circuit with readings was compared at all.
 * The last line reads "N compared, K declined, F failed".
 *
 * OPTIONS
 *   --full                 the final gate: SAP-1 at 24 s (a whole program
 *                          cycle, OUT 0 -> 255 -> 0) plus a sparse-read pass
 *                          (--read-every 7, SAP-1 at 3 s)
 *   --sap1-seconds <n>     SAP-1 run length per board (default 3; --full 24)
 *   --fixture-seconds <n>  minimum fixture run length (default 0.1)
 *   --frame <s>            SAP-1 frame length (default 1/60)
 *   --read-every <n>       read after every nth advance (default 1)
 *   --no-double-read       skip the second, reverse-order reading
 *   --options <a,b|all>    which load options (default all)
 *   --only <substr>        keep circuits whose name contains the substring
 *   --no-sap1 | --no-fixtures
 *   --candidate-advance-for <ms>  the candidate advances through
 *                          bo_engine_advance_for with this budget on a fake
 *                          clock (1 ms a read), yielding every few checks:
 *                          with the same wasm on both sides it is ruling
 *                          R86's split-run identity check
 *   --stats                print every changed counter for every circuit
 *                          (counters never fail the check by themselves)
 *   --strict-stats         a changed counter fails the check
 *   --fail-fast            stop at the first failing circuit
 *   --verbose              one line per passing circuit (default: SAP-1 and
 *                          declined ones, plus the per-option summary)
 *   --detector-check       run the candidate on a circuit whose first resistor
 *                          is nudged by ONE ULP and require the check to fail
 *                          on it (exit 0 when it does on the SAP-1 boards and
 *                          on some others); the proof the tool can fire
 *
 * A CLEAN RUN PROVES NOTHING UNTIL THE DETECTOR HAS FIRED. Run it once with
 * the same wasm on both sides (must be clean: the core is deterministic), and
 * once more with --detector-check (must find a one-ULP change on every SAP-1
 * board). test/core/perf/identity.test.ts does both, and also feeds it a
 * candidate that traps.
 */
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";

import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { DEFAULT_GENERATOR, defaultBoardsDir, loadSap1, sap1BoardFiles } from "./boards.js";
import {
  OPTION_SETS,
  compareRecordings,
  compileWasm,
  describeDifference,
  installWasm,
  recordRun,
  setAdvanceFor,
  advanceForYieldCount,
  type PhaseSpec,
  type ReadPlan,
  type Recording,
  type RunSpec,
} from "./identity-lib.js";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(scriptDir, "..", "..");
const FIXTURE_DIR = path.join(repoRoot, "test", "fixtures", "core");
/** The SAP-1 boards run under these load options (the fixtures run under all of OPTION_SETS). */
const SAP1_OPTIONS = new Set(["default", "exact"]);
/** --full: one whole program cycle, as scripts/core-accept-sap1.ts's --core-seconds 24. */
const FULL_SAP1_SECONDS = 24;
/** --full's second pass reads only every this many advances (a prime, so it does not line up with a phase boundary). */
const SPARSE_READ_EVERY = 7;

interface Args {
  base: string;
  candidate: string;
  boardsDir: string;
  frozen?: string;
  running?: string;
  generator: string;
  sap1Seconds: number;
  sap1SecondsGiven: boolean;
  fixtureSeconds: number;
  frameS: number;
  readEvery: number;
  readEveryGiven: boolean;
  doubleRead: boolean;
  options: string[];
  only?: string;
  sap1: boolean;
  fixtures: boolean;
  stats: boolean;
  candidateAdvanceFor?: number;
  strictStats: boolean;
  failFast: boolean;
  detectorCheck: boolean;
  full: boolean;
  verbose: boolean;
}

function usage(message?: string): never {
  if (message) process.stderr.write(`identity: ${message}\n`);
  process.stderr.write("usage: pnpm exec tsx scripts/perf/identity.ts <base.wasm> <candidate.wasm> [--boards <dir>] [--full] [--options a,b|all] [--sap1-seconds n] [--read-every n] [--only substr] [--stats] (see the file header)\n");
  process.exit(2);
}

function parseArgs(argv: string[]): Args {
  const args: Args = {
    base: "",
    candidate: "",
    boardsDir: defaultBoardsDir(),
    generator: DEFAULT_GENERATOR,
    sap1Seconds: 3,
    sap1SecondsGiven: false,
    fixtureSeconds: 0.1,
    frameS: 1 / 60,
    readEvery: 1,
    readEveryGiven: false,
    doubleRead: true,
    options: OPTION_SETS.map((o) => o.name),
    sap1: true,
    fixtures: true,
    stats: false,
    strictStats: false,
    failFast: false,
    detectorCheck: false,
    full: false,
    verbose: false,
  };
  const positional: string[] = [];
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
    else if (a === "--frozen") args.frozen = path.resolve(value());
    else if (a === "--running") args.running = path.resolve(value());
    else if (a === "--generator") args.generator = value();
    else if (a === "--sap1-seconds") {
      args.sap1Seconds = num();
      args.sap1SecondsGiven = true;
    } else if (a === "--fixture-seconds") args.fixtureSeconds = num();
    else if (a === "--frame") args.frameS = num();
    else if (a === "--read-every") {
      args.readEvery = Math.max(1, Math.round(num()));
      args.readEveryGiven = true;
    } else if (a === "--no-double-read") args.doubleRead = false;
    else if (a === "--double-read") args.doubleRead = true;
    else if (a === "--options") {
      const v = value();
      const names = v === "all" ? OPTION_SETS.map((o) => o.name) : v.split(",").filter((n) => n !== "");
      const unknown = names.filter((n) => !OPTION_SETS.some((o) => o.name === n));
      if (unknown.length > 0) usage(`unknown option set ${unknown.join(", ")} (known: ${OPTION_SETS.map((o) => o.name).join(", ")})`);
      args.options = names;
    } else if (a === "--only") args.only = value();
    else if (a === "--no-sap1") args.sap1 = false;
    else if (a === "--no-fixtures") args.fixtures = false;
    else if (a === "--stats") args.stats = true;
    else if (a === "--candidate-advance-for") args.candidateAdvanceFor = Number(argv[(i += 1)]);
    else if (a === "--strict-stats") args.strictStats = true;
    else if (a === "--fail-fast") args.failFast = true;
    else if (a === "--detector-check") args.detectorCheck = true;
    else if (a === "--full") args.full = true;
    else if (a === "--verbose") args.verbose = true;
    else if (a.startsWith("--")) usage(`unknown option ${a}`);
    else positional.push(a);
  }
  if (positional.length !== 2) usage("expected exactly two wasm paths");
  args.base = path.resolve(positional[0]!);
  args.candidate = path.resolve(positional[1]!);
  return args;
}

/** One circuit run, with what the sweep needs to judge it. */
interface Job {
  readonly spec: RunSpec;
  readonly kind: "sap1" | "fixture";
  readonly option: string;
  /** True only for a fixture whose sidecar says the core declines it. */
  readonly declineExpected: boolean;
}

interface Pass {
  readonly suffix: string;
  readonly reads: ReadPlan;
  readonly sap1Seconds: number;
}

function passesFor(args: Args): Pass[] {
  const passes: Pass[] = [
    { suffix: "", reads: { readEvery: args.readEvery, doubleRead: args.doubleRead }, sap1Seconds: args.sap1SecondsGiven || !args.full ? args.sap1Seconds : FULL_SAP1_SECONDS },
  ];
  if (args.full && !args.readEveryGiven) {
    passes.push({ suffix: `#read${String(SPARSE_READ_EVERY)}`, reads: { readEvery: SPARSE_READ_EVERY, doubleRead: false }, sap1Seconds: args.sap1Seconds });
  }
  return passes;
}

interface Sidecar {
  status?: string;
  durationS?: number;
  phases?: PhaseSpec[];
}

function jobsFor(args: Args): Job[] {
  const jobs: Job[] = [];
  const keep = (name: string): boolean => args.only === undefined || name.includes(args.only);
  const options = OPTION_SETS.filter((o) => args.options.includes(o.name));
  let boards: ReturnType<typeof sap1BoardFiles> = [];
  if (args.sap1) {
    try {
      boards = sap1BoardFiles({
        boardsDir: args.boardsDir,
        generator: args.generator,
        files: { ...(args.frozen ? { frozen: args.frozen } : {}), ...(args.running ? { running: args.running } : {}) },
      });
    } catch (err) {
      return usage(err instanceof Error ? err.message : String(err));
    }
  }
  const fixtureFiles = args.fixtures ? fs.readdirSync(FIXTURE_DIR).filter((f) => f.endsWith(".sim.json")).sort() : [];

  for (const pass of passesFor(args)) {
    for (const { name, file } of boards) {
      const circuit = loadSap1(file);
      for (const option of options) {
        if (!SAP1_OPTIONS.has(option.name)) continue;
        const full = `${name}${option.name === "default" ? "" : `+${option.name}`}${pass.suffix}`;
        if (!keep(full)) continue;
        jobs.push({
          spec: { name: full, circuit, frameS: args.frameS, seconds: pass.sap1Seconds, bench: circuit.bench, options: option.options, reads: pass.reads },
          kind: "sap1",
          option: option.name,
          declineExpected: false,
        });
      }
    }
    for (const file of fixtureFiles) {
      const base = file.replace(/\.sim\.json$/, "");
      const circuit = JSON.parse(fs.readFileSync(path.join(FIXTURE_DIR, file), "utf8")) as SimCircuit;
      const sidecarPath = path.join(FIXTURE_DIR, `${base}.expected.json`);
      const sidecar: Sidecar = fs.existsSync(sidecarPath) ? (JSON.parse(fs.readFileSync(sidecarPath, "utf8")) as Sidecar) : {};
      for (const [frameSuffix, frameS] of [["@60Hz", 1 / 60], ["@1ms", 1e-3]] as const) {
        for (const option of options) {
          const name = `${base}${frameSuffix}${option.name === "default" ? "" : `+${option.name}`}${pass.suffix}`;
          if (!keep(name)) continue;
          jobs.push({
            spec: {
              name,
              circuit,
              frameS,
              // The phases extend this in frameTimes when they sum to more.
              seconds: Math.max(args.fixtureSeconds, sidecar.durationS ?? 0),
              ...(sidecar.phases ? { phases: sidecar.phases } : {}),
              options: option.options,
              reads: pass.reads,
            },
            kind: "fixture",
            option: option.name,
            declineExpected: sidecar.status === "core-declined",
          });
        }
      }
    }
  }
  return jobs;
}

/** The fixtures declined by design, for the header. */
function declinedByDesign(): string[] {
  return fs
    .readdirSync(FIXTURE_DIR)
    .filter((f) => f.endsWith(".expected.json"))
    .filter((f) => (JSON.parse(fs.readFileSync(path.join(FIXTURE_DIR, f), "utf8")) as Sidecar).status === "core-declined")
    .map((f) => f.replace(/\.expected\.json$/, ""))
    .sort();
}

/** The next double above x. */
function nextUp(x: number): number {
  const view = new DataView(new ArrayBuffer(8));
  view.setFloat64(0, x);
  view.setBigUint64(0, view.getBigUint64(0) + 1n);
  return view.getFloat64(0);
}

/** The spec with its first resistor's value one ULP higher, or undefined when it has none. */
function nudged(spec: RunSpec): RunSpec | undefined {
  const index = spec.circuit.components.findIndex((c) => c.kind === "resistor" && typeof c.params.resistance === "number");
  if (index < 0) return undefined;
  const circuit = structuredClone(spec.circuit);
  const resistor = circuit.components[index]!;
  resistor.params = { ...resistor.params, resistance: nextUp(resistor.params.resistance as number) };
  return { ...spec, circuit };
}

/** Record `spec` on `module`; never throws (a failed instantiation is reported as the run's error). */
async function record(module: object, spec: RunSpec): Promise<Recording> {
  try {
    await installWasm(module);
    return recordRun(spec);
  } catch (err) {
    const message = err instanceof Error ? `${err.name}: ${err.message}` : String(err);
    return { name: spec.name, error: { reading: 0, label: "instantiate", message, isTrap: false }, keys: { nodeV: [], deviceI: [], digital: [], channel: [] }, frames: [], stats: [], cpuMs: 0 };
  }
}

interface Tally {
  compared: number;
  declined: number;
  failed: number;
}

async function main(): Promise<void> {
  const args = parseArgs(process.argv.slice(2));
  for (const p of [args.base, args.candidate]) if (!fs.existsSync(p)) usage(`${p} does not exist`);
  let jobs = jobsFor(args);
  if (args.detectorCheck) jobs = jobs.filter((j) => j.option === "default" && !j.spec.name.includes("#"));
  if (jobs.length === 0) usage("no circuit selected");
  const baseBytes = fs.readFileSync(args.base);
  const candidateBytes = fs.readFileSync(args.candidate);
  let baseModule: object;
  let candidateModule: object;
  try {
    [baseModule, candidateModule] = await Promise.all([compileWasm(baseBytes), compileWasm(candidateBytes)]);
  } catch (err) {
    process.stdout.write(`FAIL: a wasm file does not compile: ${err instanceof Error ? err.message : String(err)}\n`);
    process.exit(1);
  }
  const sameFile = baseBytes.equals(candidateBytes);
  const passes = passesFor(args);

  process.stdout.write(`identity: base ${args.base}\n          cand ${args.candidate}${sameFile ? "  (byte-identical files: a determinism control)" : ""}\n`);
  process.stdout.write(`          ${String(jobs.length)} circuit run(s); options: ${args.options.join(", ")}; ${passes.map((p) => `reads every ${String(p.reads.readEvery)}${p.reads.doubleRead ? " + reverse re-read" : ""}, SAP-1 ${String(p.sap1Seconds)} s`).join(" | ")}\n`);
  process.stdout.write(`          declined by design (the sidecars say core-declined): ${declinedByDesign().join(", ") || "none"}\n`);
  process.stdout.write(`          load average ${os.loadavg().map((x) => x.toFixed(2)).join(" ")}\n\n`);

  const byOption = new Map<string, Tally>();
  const tally = (option: string): Tally => {
    let t = byOption.get(option);
    if (!t) byOption.set(option, (t = { compared: 0, declined: 0, failed: 0 }));
    return t;
  };
  let nanPayload = 0;
  let statChanged = 0;
  let frames = 0;
  let baseCpu = 0;
  let candidateCpu = 0;
  let detected = 0;
  let sap1Detected = 0;
  let sap1Runs = 0;
  const missed: string[] = [];
  const declinedSeen = new Set<string>();

  for (const job of jobs) {
    const { spec } = job;
    const t = tally(job.option);
    const base = await record(baseModule, spec);

    if (args.detectorCheck) {
      const changed = nudged(spec);
      if (!changed) {
        process.stdout.write(`skip ${spec.name}: no resistor to nudge\n`);
        continue;
      }
      setAdvanceFor(args.candidateAdvanceFor);
    const candidate = await record(candidateModule, changed);
    setAdvanceFor(undefined);
      const cmp = compareRecordings(base, candidate);
      if (job.kind === "sap1") sap1Runs += 1;
      // Here a difference is the GOOD outcome. A circuit the nudge cannot reach (a resistor on a branch nothing
      // reads, or one the rounding absorbs) is listed, not failed: the check is about the tool, not the circuit.
      if (cmp.difference && cmp.difference.field !== "error") {
        detected += 1;
        if (job.kind === "sap1") sap1Detected += 1;
        process.stdout.write(`ok   ${spec.name}: one-ULP change found at ${cmp.difference.reading === 0 ? "load" : `reading ${String(cmp.difference.reading)}`} in ${cmp.difference.field} ${cmp.difference.key}\n`);
      } else {
        missed.push(spec.name);
        process.stdout.write(`MISS ${spec.name}: ${cmp.difference ? "the nudged run errored" : "the nudged circuit gave bit-identical output"}\n`);
      }
      continue;
    }

    setAdvanceFor(args.candidateAdvanceFor);
    const candidate = await record(candidateModule, spec);
    setAdvanceFor(undefined);
    const cmp = compareRecordings(base, candidate);
    frames += base.frames.length;
    baseCpu += base.cpuMs;
    candidateCpu += candidate.cpuMs;
    nanPayload += cmp.nanPayloadOnly;
    if (cmp.statDeltas.length > 0 && !cmp.difference) statChanged += 1;
    const statNote = cmp.statDeltas.length > 0 ? ` (${String(cmp.statDeltas.length)} counter(s) changed)` : "";
    const fail = (text: string): void => {
      t.failed += 1;
      process.stdout.write(`FAIL ${text}\n`);
    };

    if (cmp.difference) {
      fail(describeDifference(cmp.difference));
    } else if (base.declined !== undefined) {
      // Identical on both sides, and so far vacuous: only a decline the sidecar promised is acceptable.
      if (job.declineExpected) {
        t.declined += 1;
        declinedSeen.add(spec.name);
        if (args.verbose) process.stdout.write(`ok   ${spec.name}: declined by design, identically (${base.declined.slice(0, 90)})\n`);
      } else {
        fail(`${spec.name}: the core DECLINED to load this ${job.kind === "sap1" ? "SAP-1 board" : "fixture"} on both sides, so nothing was compared: ${base.declined.slice(0, 200)}`);
      }
    } else if (job.declineExpected) {
      fail(`${spec.name}: the sidecar says the core declines this fixture, but it loaded on both sides`);
    } else if (base.frames.length === 0) {
      fail(`${spec.name}: no readings were taken`);
    } else {
      t.compared += 1;
      if (job.kind === "sap1" || args.verbose) {
        const cpu = job.kind === "sap1" ? `  cpu ${base.cpuMs.toFixed(0)} -> ${candidate.cpuMs.toFixed(0)} ms` : "";
        process.stdout.write(`ok   ${spec.name}: ${String(base.frames.length)} readings identical${cpu}${statNote}\n`);
      }
    }
    if (args.strictStats && cmp.statDeltas.length > 0 && !cmp.difference) fail(`${spec.name}: ${String(cmp.statDeltas.length)} counter(s) changed (--strict-stats)`);
    if ((args.stats || args.strictStats) && cmp.statDeltas.length > 0) {
      for (const s of cmp.statDeltas) process.stdout.write(`       counter ${s.name}: ${String(s.base)} -> ${String(s.candidate)}\n`);
    }
    if (args.failFast && t.failed > 0) break;
  }

  if (args.detectorCheck) {
    // The SAP-1 boards are the ones that matter most: a detector that cannot see a change there is not fit for the wave.
    process.stdout.write(`\ndetector check: ${String(detected)} circuit run(s) detected the one-ULP change, ${String(missed.length)} did not (${missed.join(", ") || "none"})\n`);
    const sap1Ok = sap1Detected === sap1Runs;
    if (!sap1Ok) process.stdout.write(`detector check FAILED: it must find the change on every SAP-1 run (${String(sap1Detected)} of ${String(sap1Runs)})\n`);
    process.exit(sap1Ok && detected > 0 ? 0 : 1);
  }

  const total = [...byOption.values()].reduce((a, x) => ({ compared: a.compared + x.compared, declined: a.declined + x.declined, failed: a.failed + x.failed }), { compared: 0, declined: 0, failed: 0 });
  process.stdout.write("\nper option:\n");
  for (const [option, x] of byOption) {
    process.stdout.write(`  ${option.padEnd(16)} ${String(x.compared).padStart(4)} compared, ${String(x.declined).padStart(3)} declined, ${String(x.failed).padStart(3)} failed\n`);
  }
  let failures = total.failed;
  if (total.compared === 0) {
    failures += 1;
    process.stdout.write("FAIL: no circuit with readings was compared, so this run checked nothing\n");
  }
  process.stdout.write(
    `\n${failures === 0 ? "IDENTICAL" : "DIFFERENT"}: ${String(total.compared)} compared, ${String(total.declined)} declined${declinedSeen.size > 0 ? ` (by design: ${[...declinedSeen].map((n) => n.replace(/@.*$/, "")).filter((n, i, all) => all.indexOf(n) === i).join(", ")})` : ""}, ${String(total.failed)} failed; ${String(frames)} readings compared (${baseCpu.toFixed(0)} ms base, ${candidateCpu.toFixed(0)} ms candidate CPU)\n`,
  );
  if (nanPayload > 0) process.stdout.write(`warning: ${String(nanPayload)} value(s) differ only in a NaN's payload (Object.is equal; not counted as a difference)\n`);
  if (args.candidateAdvanceFor !== undefined) process.stdout.write(`candidate advances yielded ${String(advanceForYieldCount())} times (bo_engine_advance_for, ${String(args.candidateAdvanceFor)} ms on a fake clock)\n`);
  if (statChanged > 0 && !args.stats) process.stdout.write(`note: ${String(statChanged)} run(s) changed a stat counter (--stats lists them; counters do not fail the check)\n`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((err: unknown) => {
  process.stderr.write(`identity: ${err instanceof Error ? (err.stack ?? err.message) : String(err)}\n`);
  process.exit(1);
});
