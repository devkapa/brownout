/**
 * The creep guard: the catalog SAP-1's INSTRUCTIONS per simulated second, per build, in one node process per run.
 *
 * Why instructions and why this shape. Wall and CPU time move with the machine's load; the instructions a process
 * retires do not, so two builds can be compared to a few tenths of a percent on a busy laptop. The cost per
 * simulated second is the difference of two run lengths (default 66 s minus 6 s, over 60), which removes load, JIT
 * and the power-on transient. `/usr/bin/time -l` is wrapped directly around `node` (never around `pnpm exec`,
 * which would count the package runner), once per run, and every run has a hard wall limit. macOS only: the
 * counter is the `instructions retired` line of BSD `time -l`.
 *
 * What it found, and why it exists. A "layout" cost of about 2 % that came and went with unrelated Rust changes was
 * not in the wasm at all (every hot function's machine code was identical): it was the TypeScript host, whose
 * per-advance readout loop (`buildElementI`) gained a Map operation per device record. A timing check of the
 * wasm alone cannot see that; this one times the whole engine as a host runs it.
 *
 * Usage (from a checkout whose core is built, `pnpm run build:core`):
 *   node --import tsx scripts/perf/sap1-instr.ts --base <tree> [--tree <tree>] [--reps 5] [--limit 0.5]
 *   node --import tsx scripts/perf/sap1-instr.ts --tree <tree>          # just print the figure
 * <tree> is a checkout, or an extracted `git archive`, holding src/ and its built src/core/brownout-core.wasm.
 * Exit 1 when --base is given and the catalog board costs more than --limit percent over it (default 0.5), 0 else.
 * Time: about 25 s per rep per tree on an M-series laptop; 5 reps of two trees is about 4 minutes.
 */
import { spawnSync } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

import { defaultBoardsDir, programNibbles, sap1BoardFiles } from "./boards.js";

const FRAME_S = 1 / 60;
const here = fileURLToPath(import.meta.url);
const repoRoot = path.resolve(path.dirname(here), "..", "..");

async function runOne(tree: string, board: string, seconds: number): Promise<void> {
  const src = (rel: string) => pathToFileURL(path.join(tree, "src", rel)).href;
  const core = (await import(src("core.js"))) as { CoreEngine: new () => any };
  const wasm = (await import(src("core/wasm.js"))) as { initCore: () => Promise<void> };
  await wasm.initCore();
  const circuit = JSON.parse(fs.readFileSync(board, "utf8"));
  const e = new core.CoreEngine();
  e.load(circuit);
  e.setMemoryContents(circuit.bench.ramLo, programNibbles(circuit.bench.program, 0));
  e.setMemoryContents(circuit.bench.ramHi, programNibbles(circuit.bench.program, 4));
  for (let i = 0, n = Math.round(seconds / FRAME_S); i < n; i++) e.advance(e.simTime + FRAME_S);
  if (!e.lastConverged) throw new Error("sap1-instr: did not converge");
  e.dispose();
}

function instructionsOf(tree: string, board: string, seconds: number): number {
  const r = spawnSync("/usr/bin/time", ["-l", process.execPath, "--import", "tsx", here, "--one", tree, board, String(seconds)], {
    cwd: repoRoot,
    encoding: "utf8",
    timeout: 900_000,
    killSignal: "SIGKILL",
  });
  if (r.status !== 0) throw new Error(`sap1-instr: a run failed (status ${String(r.status)}, signal ${String(r.signal)})\n${r.stderr}`);
  const m = /(\d+)\s+instructions retired/.exec(r.stderr);
  if (!m) throw new Error("sap1-instr: no `instructions retired` line (this needs BSD /usr/bin/time -l, macOS)");
  return Number(m[1]);
}

const median = (xs: number[]): number => [...xs].sort((a, b) => a - b)[Math.floor(xs.length / 2)]!;

function perSecond(tree: string, board: string, reps: number, shortS: number, longS: number): number[] {
  instructionsOf(tree, board, 2); // warm the transform cache: a cold first run would sit in the short run only
  const out: number[] = [];
  for (let r = 0; r < reps; r++) out.push((instructionsOf(tree, board, longS) - instructionsOf(tree, board, shortS)) / (longS - shortS) / 1e6);
  return out;
}

async function main(): Promise<void> {
  const args = process.argv.slice(2);
  if (args[0] === "--one") return runOne(path.resolve(args[1]!), args[2]!, Number(args[3]));
  let base: string | undefined, tree = repoRoot, reps = 5, limit = 0.5;
  for (let i = 0; i < args.length; i++) {
    const a = args[i]!;
    if (a === "--base") base = path.resolve(args[++i]!);
    else if (a === "--tree") tree = path.resolve(args[++i]!);
    else if (a === "--reps") reps = Number(args[++i]);
    else if (a === "--limit") limit = Number(args[++i]);
    else throw new Error(`sap1-instr: unknown argument ${a}`);
  }
  const board = sap1BoardFiles({ boardsDir: defaultBoardsDir(), catalog: true }).find((b) => b.variant === "running")!.file;
  const figure = (t: string) => {
    const v = perSecond(t, board, reps, 6, 66);
    console.log(`  ${t}: ${median(v).toFixed(1)} M instructions per simulated second (median of ${String(reps)}, ${Math.min(...v).toFixed(1)} .. ${Math.max(...v).toFixed(1)})`);
    return median(v);
  };
  console.log("catalog ideal SAP-1, program running, 6 s -> 66 s");
  const cand = figure(tree);
  if (base === undefined) return;
  const ref = figure(base);
  const pct = ((cand - ref) / ref) * 100;
  console.log(`  candidate vs base: ${pct >= 0 ? "+" : ""}${pct.toFixed(2)} % (limit +${String(limit)} %)`);
  if (pct > limit) {
    console.log("FAIL: the SAP-1 got dearer. Look at the TS host's per-advance paths (src/core/readout.ts, engine.ts) as well as the wasm: PERF2 found the last creep there.");
    process.exitCode = 1;
  }
}

main().catch((err: unknown) => {
  console.error(err instanceof Error ? err.message : err);
  process.exitCode = 2;
});
