/**
 * The rated SAP-1 cost run (core/README.md, "W1g cost"; rulings
 * R38 (a) and R43 (b)): CPU ms per simulated second of the Rust/WASM core on
 * the SAP-1, ideal and battery, display frozen and running, for several
 * builds side by side.
 *
 * Usage (from a worktree, after `pnpm run build:core` in every tree named):
 *   pnpm run bench:sap1-cost -- [options] <label>=<tree> [<label>=<tree> ...]
 *   pnpm run bench:sap1-cost -- --gate 3ca4004=/tmp/tree-3ca4004 cand=.
 *
 * Each <tree> is a checkout (or an extracted `git archive`) holding src/ and
 * its built src/core/brownout-core.wasm; it is imported from there, its own
 * TypeScript compiler included. The first label is the baseline: every other
 * is reported as a change against it. The ceiling the README states (+11 %
 * over 3ca4004, ideal and battery, frozen and running) is read off that.
 *
 * THROUGH THE COMPILER, RATINGS INCLUDED. A board is loaded from its
 * .sim.json with the tree's CoreEngine.load, so the program measured is what
 * a host runs: since W1g every resistor carries a `rating` record (115 on the
 * SAP-1), and the slow physics they switch on is part of the cost. W1g's
 * earlier runs timed .prog files dumped before those records existed, and
 * missed +21 % (frozen) and +29 % (running) on the ideal board (W1gV3). So
 * each run also compiles the board with the tree's compileCircuit and prints
 * how many rating records it carries; a tree whose compiler knows the
 * `rating` kind but emits none is refused (the measurement would not be the
 * workload).
 *
 * PROTOCOL. Per rep, per supply, per tree, a fresh process (no JIT or heap
 * state carried between builds): load, seed the two RAM chips with the
 * board's program, --warmup s of 1/60 s frames (default 5: past the power-on
 * transient and the first program loop), then --measure s (default 1) of
 * frames, timed with process.cpuUsage; frozen, then running. Reps interleave
 * the trees, so a load change on the machine hits them alike. The report is
 * the median over reps (and the minimum), and each change the median of the
 * per-rep ratios to the baseline (runs back to back, so a load that comes
 * and goes cancels), with the 1-minute load average at every run: on a busy
 * machine treat the numbers as suspect and re-run.
 *
 * THE GATE (ruling R43 (b)). R22's ceiling is judged at steady state, `--gate`
 * (`--warmup=30`): 30 s of warm-up on every side, 9 interleaved reps, the
 * paired median. A 5 s warm-up still sits in the wasm tier-up: F5a's build
 * read 1.5-2 % over its head there and 0.7 % at 30 s, with CPU profiles
 * 0.2 % apart. Start-up is what a user feels opening a circuit, so it is
 * never dropped: whenever the warm-up runs past --startup s (default 5) and
 * its measured window, the same run also measures --measure s right after
 * --startup s, the 5 s figure the earlier protocol reported, and prints it
 * beside the steady one. Both come from one process, so they cost one load
 * and see the machine alike.
 *
 * OPTIONS
 *   --reps=<n>          repetitions (default 9)
 *   --power=<list>      ideal,battery (the default) or one of them
 *   --boards=<dir>      where sap1-max-<power>-<variant>.sim.json live; missing
 *                       ones are generated (scripts/perf/boards.ts)
 *   --generator=<path>  the SAP-1 generator (decision 12)
 *   --warmup=<s>        simulated warm-up seconds (default 5)
 *   --gate              the R22 gate: --warmup=30 (with the start-up figure)
 *   --startup=<s>       the start-up figure's warm-up (default 5), measured
 *                       too whenever it is shorter than --warmup
 *   --measure=<s>       simulated measured seconds (default 1)
 */
import { spawnSync } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

import { DEFAULT_GENERATOR, defaultBoardsDir, programNibbles, sap1BoardFiles, type Sap1Bench, type Sap1Power } from "./boards.js";

const FRAME_S = 1 / 60;
/** Ruling R43 (b): the R22 gate's warm-up, on every side. */
const GATE_WARMUP_S = 30;
const thisFile = fileURLToPath(import.meta.url);
const repoRoot = path.resolve(path.dirname(thisFile), "..", "..");

interface ChildResult {
  ratings: number;
  ratingKind: boolean;
  /** Per warm-up, in the order asked: CPU ms per simulated second. */
  frozen: number[];
  running: number[];
}

/**
 * One tree on one supply's two boards, in this (child) process. Each board
 * runs once, measured for `measureS` after each of `warmupsS` (ascending,
 * counted from load): the frames between two of them are warm-up for the
 * later one, the earlier measured second included.
 */
async function child(tree: string, frozenFile: string, runningFile: string, warmupsS: number[], measureS: number): Promise<ChildResult> {
  const src = (rel: string) => pathToFileURL(path.join(tree, "src", rel)).href;
  const core = (await import(src("core.js"))) as { CoreEngine: new () => ChildEngine };
  const wasm = (await import(src("core/wasm.js"))) as { initCore: () => Promise<void> };
  const compile = (await import(src("core/compile/index.js"))) as {
    compileCircuit: (c: unknown, kinds: unknown, o: { startTimeS: number }) => { ok: boolean; reasons?: unknown; compiled?: { devices: { kind: string }[] } };
  };
  const kinds = (await import(src("core/kinds.js"))) as { CORE_IMPLEMENTED_KINDS: unknown; CORE_KIND_CODES: Record<string, number> };
  // SAP1_ADVANCE_FOR_MS (ruling R86, addition (c)): every frame through
  // bo_engine_advance_for with this budget (a non-positive one leaves the
  // clock unread), then a plain advance to the same time, which only
  // publishes; the host clock's reads are counted.
  const forMs = process.env.SAP1_ADVANCE_FOR_MS === undefined ? undefined : Number(process.env.SAP1_ADVANCE_FOR_MS);
  let clockReads = 0;
  if (forMs !== undefined) {
    const now = performance.now.bind(performance);
    performance.now = () => {
      clockReads += 1;
      return now();
    };
  }
  await wasm.initCore();
  const raw = forMs === undefined ? undefined : ((await import(src("core/wasm.js"))) as { coreCall: <T>(f: (x: { bo_engine_advance_for(h: number, t: number, ms: number): number; bo_engine_stat(h: number, i: number): number }) => T) => T }).coreCall;
  let calls = 0;
  const cpu = () => {
    const u = process.cpuUsage();
    return (u.user + u.system) / 1000;
  };
  const out: ChildResult = { ratings: 0, ratingKind: "rating" in kinds.CORE_KIND_CODES, frozen: [], running: [] };
  for (const [variant, file] of [["frozen", frozenFile], ["running", runningFile]] as const) {
    const circuit = JSON.parse(fs.readFileSync(file, "utf8")) as { bench: Sap1Bench };
    const compiled = compile.compileCircuit(circuit, kinds.CORE_IMPLEMENTED_KINDS, { startTimeS: 0 });
    if (!compiled.ok || !compiled.compiled) throw new Error(`${tree}: the compiler declines ${file}: ${JSON.stringify(compiled.reasons)}`);
    out.ratings = compiled.compiled.devices.filter((d) => d.kind === "rating").length;
    const e = new core.CoreEngine();
    if (raw !== undefined) {
      const plain = e.advance.bind(e);
      const h = (e as unknown as { _handle: number })._handle;
      e.advance = (t: number) => {
        let code = 1;
        while (code === 1) {
          code = raw((x) => x.bo_engine_advance_for(h, t, forMs!));
          calls += 1;
        }
        if (code !== 0) throw new Error(`bo_engine_advance_for: ${code}`);
        plain(t);
      };
    }
    try {
      e.load(circuit);
      e.setMemoryContents(circuit.bench.ramLo, programNibbles(circuit.bench.program, 0));
      e.setMemoryContents(circuit.bench.ramHi, programNibbles(circuit.bench.program, 4));
      // Frames are counted, not compared against simTime, so a run reaches
      // each warm-up on the same frame the single-warm-up protocol did.
      let frames = 0;
      for (const warmupS of warmupsS) {
        for (; frames < Math.round(warmupS / FRAME_S); frames++) e.advance(e.simTime + FRAME_S);
        const s0 = e.simTime;
        const c0 = cpu();
        const h = (e as unknown as { _handle: number })._handle;
        const events = () => (raw === undefined ? 0 : raw((x) => x.bo_engine_stat(h, 4)));
        const [r0, k0, ev0] = [clockReads, calls, events()];
        for (let f = 0; f < Math.round(measureS / FRAME_S); f++, frames++) e.advance(e.simTime + FRAME_S);
        if (!e.lastConverged) throw new Error(`${tree}: ${variant} did not converge`);
        out[variant].push((cpu() - c0) / (e.simTime - s0));
        if (forMs !== undefined) {
          // The stride the run settles at, as events per in-call check: the
          // reads that are not a call's start or end read are the checks.
          const [reads, n, ev] = [clockReads - r0, calls - k0, events() - ev0];
          const checks = reads - 2 * n;
          console.error(
            `R86 ${variant}: ${(reads / (e.simTime - s0)).toFixed(0)} clock reads per simulated s, ${n} calls, ` +
              `${checks} in-call checks, ${checks > 0 ? (ev / checks).toFixed(1) : "-"} events a check`,
          );
        }
      }
    } finally {
      e.dispose();
    }
  }
  return out;
}

interface ChildEngine {
  load(c: unknown): void;
  setMemoryContents(id: string, nibbles: number[]): void;
  advance(t: number): void;
  readonly simTime: number;
  readonly lastConverged: boolean;
  dispose(): void;
}

function median(xs: number[]): number {
  const s = [...xs].sort((a, b) => a - b);
  const m = Math.floor(s.length / 2);
  return s.length % 2 ? s[m]! : (s[m - 1]! + s[m]!) / 2;
}

function pct(x: number, base: number): string {
  const p = (x / base - 1) * 100;
  return `${p >= 0 ? "+" : ""}${p.toFixed(1)} %`;
}

async function main(): Promise<void> {
  const argv = process.argv.slice(2);
  if (argv[0] === "--child") {
    const [, tree, frozen, running, warmups, measure] = argv;
    process.stdout.write(JSON.stringify(await child(tree!, frozen!, running!, warmups!.split(",").map(Number), Number(measure))) + "\n");
    return;
  }
  let reps = 9;
  let powers: Sap1Power[] = ["ideal", "battery"];
  let boardsDir = defaultBoardsDir();
  let generator = DEFAULT_GENERATOR;
  let warmupS = 5;
  let startupS = 5;
  let measureS = 1;
  const trees: { label: string; tree: string }[] = [];
  for (const arg of argv) {
    // `pnpm run bench:sap1-cost -- ...` can pass the separator through.
    if (arg === "--") continue;
    const [flag, value] = arg.split("=", 2) as [string, string | undefined];
    if (flag === "--reps") reps = Number(value);
    else if (flag === "--power") powers = (value ?? "").split(",") as Sap1Power[];
    else if (flag === "--boards") boardsDir = path.resolve(value ?? "");
    else if (flag === "--generator") generator = path.resolve(value ?? "");
    else if (flag === "--warmup") warmupS = Number(value);
    else if (flag === "--gate" && value === undefined) warmupS = GATE_WARMUP_S;
    else if (flag === "--startup") startupS = Number(value);
    else if (flag === "--measure") measureS = Number(value);
    else if (flag.startsWith("--")) throw new Error(`sap1-cost: unknown flag ${arg}`);
    else if (value !== undefined) trees.push({ label: flag, tree: path.resolve(value) });
    else throw new Error(`sap1-cost: expected <label>=<tree>, got ${arg}`);
  }
  if (trees.length === 0) throw new Error("sap1-cost: name at least one <label>=<tree>");
  for (const p of powers) if (p !== "ideal" && p !== "battery") throw new Error(`sap1-cost: unknown supply ${p}`);
  for (const [name, s] of [["--warmup", warmupS], ["--startup", startupS], ["--measure", measureS]] as const) {
    if (!(s > 0)) throw new Error(`sap1-cost: ${name} must be a positive number of seconds`);
  }
  // The measured windows, ascending: the start-up one first when the warm-up
  // runs past it (it must end before the steady warm-up does).
  const windows: { name: string; warmupS: number }[] =
    startupS + measureS <= warmupS ? [{ name: "start-up", warmupS: startupS }, { name: "steady", warmupS }] : [{ name: "steady", warmupS }];
  for (const { tree } of trees) {
    if (!fs.existsSync(path.join(tree, "src", "core", "brownout-core.wasm"))) throw new Error(`sap1-cost: ${tree} has no built src/core/brownout-core.wasm (pnpm run build:core there)`);
  }
  const boards = new Map(powers.map((power) => [power, sap1BoardFiles({ boardsDir, generator, power })]));
  const tsx = path.join(repoRoot, "node_modules", ".bin", process.platform === "win32" ? "tsx.cmd" : "tsx");
  // Keyed by window, supply and label.
  const results = new Map<string, { frozen: number[]; running: number[]; ratings: number }>();
  for (let rep = 1; rep <= reps; rep++) {
    for (const power of powers) {
      const files = boards.get(power)!;
      const file = (v: string) => files.find((b) => b.variant === v)!.file;
      for (const { label, tree } of trees) {
        const r = spawnSync(tsx, [thisFile, "--child", tree, file("frozen"), file("running"), windows.map((w) => String(w.warmupS)).join(","), String(measureS)], {
          cwd: repoRoot,
          encoding: "utf8",
        });
        if (r.status !== 0) throw new Error(`sap1-cost: ${label} (${power}) failed:\n${r.stderr}`);
        const res = JSON.parse(r.stdout.trim().split("\n").pop()!) as ChildResult;
        if (res.ratingKind && res.ratings === 0) throw new Error(`sap1-cost: ${label}'s compiler knows the rating record but its SAP-1 compile carries none: not the workload`);
        const parts: string[] = [];
        windows.forEach((w, i) => {
          const key = `${w.name} ${power} ${label}`;
          const acc = results.get(key) ?? { frozen: [], running: [], ratings: res.ratings };
          acc.frozen.push(res.frozen[i]!);
          acc.running.push(res.running[i]!);
          results.set(key, acc);
          parts.push(`${w.name} (${String(w.warmupS)} s) frozen ${res.frozen[i]!.toFixed(2)} running ${res.running[i]!.toFixed(2)}`);
        });
        console.log(`rep ${String(rep)} ${power} ${label}: ${parts.join(", ")} ms/sim-s (ratings ${String(res.ratings)}) | load ${os.loadavg()[0]!.toFixed(2)}`);
      }
    }
  }
  // The change is the median over reps of each rep's ratio to the baseline's
  // run in the same rep: those ran back to back, so a load that comes and
  // goes over the session cancels from it, where it would not from a ratio
  // of the two medians (printed too).
  // Steady state first: it is what R22's ceiling is read off (R43 (b)).
  for (const w of [...windows].reverse()) {
    const role = w.name === "steady" && windows.length > 1 ? " (the gate)" : w.name === "start-up" ? " (reported beside the gate)" : "";
    console.log(`\n${w.name}${role}: median (min) CPU ms per simulated second over ${String(reps)} reps, ${String(w.warmupS)} s warm-up, ${String(measureS)} s measured;`);
    console.log(`change against ${trees[0]!.label}: the median of the paired per-rep ratios [the ratio of the medians]`);
    report(w.name);
  }

  function report(window: string): void {
    for (const power of powers) {
      const base = results.get(`${window} ${power} ${trees[0]!.label}`)!;
      const [bf, br] = [median(base.frozen), median(base.running)];
      for (const { label } of trees) {
        const r = results.get(`${window} ${power} ${label}`)!;
        const [f, g] = [median(r.frozen), median(r.running)];
        const paired = (xs: number[], ys: number[]) => median(xs.map((x, i) => x / ys[i]!));
        console.log(
          `  ${power.padEnd(8)}${label.padEnd(12)} frozen ${f.toFixed(2)} (${Math.min(...r.frozen).toFixed(2)}) ${pct(paired(r.frozen, base.frozen), 1).padStart(9)} [${pct(f, bf)}]` +
            `   running ${g.toFixed(2)} (${Math.min(...r.running).toFixed(2)}) ${pct(paired(r.running, base.running), 1).padStart(9)} [${pct(g, br)}]   ratings ${String(r.ratings)}`,
        );
      }
    }
  }
}

main().catch((err: unknown) => {
  console.error(err instanceof Error ? err.message : err);
  process.exitCode = 1;
});
