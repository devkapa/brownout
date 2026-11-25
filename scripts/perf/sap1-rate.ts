/**
 * The ideal SAP-1's cost per simulated second, plain and catalog, with its
 * program running, in ONE node process that times itself.
 *
 * Why this exists. A speed check of the first part batch wrapped
 * `/usr/bin/time -l` around `pnpm exec tsx ...`, which counts the package
 * runner (about 1.83 G instructions whatever the run), not the simulation, and
 * so measured nothing while a 3 % cost came in. Here nothing is wrapped: the
 * script reads `process.cpuUsage()` of its own process around the frames it
 * advances, and prints its pid so a log shows which process it was.
 *
 * Usage (from the checkout; `pnpm run build:core` first in the tree measured):
 *   node --import tsx scripts/perf/sap1-rate.ts [--tree=<dir>] [--boards=<dir>] [--reps=7]
 *       [--short=6] [--long=66] [--variant=running|frozen]
 *
 * Do not start it as `pnpm exec`, `pnpm run` or `npx`: that works (the
 * numbers are still its own), but run it directly so a `time` wrapped around
 * it, if someone adds one, at least wraps the right process.
 *
 * What it does. For the plain board and the catalog board (every component
 * with the catalogUid the app sends: a thermal record for each resistor and
 * LED), each rep loads a fresh engine, seeds the two RAM chips with the
 * program, advances 1/60 s frames to --short seconds, reads the CPU time, goes
 * on to --long seconds and reads it again. The cost per simulated second is the
 * difference over the span: load, JIT and the power-on transient all sit before
 * the first reading and cancel. Rep 0 of each board is a warm-up of the JIT
 * and is printed but left out of the statistics.
 *
 * Comparing two builds. Build both (`pnpm run build:core` in each tree), then
 * run it once per tree, `--tree=<dir>`, one after the other with nothing else
 * heavy on the machine, and compare the medians of the same board; repeat in
 * the other order and compare again, the spread printed is what is noise. The
 * catalog board is the app's workload; the plain board does not move when a
 * change only touches catalog-identified parts, which is how the first batch's
 * 3 % was missed. This is a measurement, not a test: it never fails on a
 * number (wall-clock gates are noise on a loaded machine).
 */
import os from "node:os";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

import { DEFAULT_GENERATOR, defaultBoardsDir, programNibbles, sap1BoardFiles, type Sap1Bench } from "./boards.js";

const FRAME_S = 1 / 60;
const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..");

interface Engine {
  load(c: unknown): void;
  setMemoryContents(id: string, nibbles: number[]): void;
  advance(t: number): void;
  readonly simTime: number;
  readonly lastConverged: boolean;
  dispose(): void;
}

const cpuMs = (): number => {
  const u = process.cpuUsage();
  return (u.user + u.system) / 1000;
};

function median(xs: number[]): number {
  const s = [...xs].sort((a, b) => a - b);
  const m = Math.floor(s.length / 2);
  return s.length % 2 ? s[m]! : (s[m - 1]! + s[m]!) / 2;
}

async function main(): Promise<void> {
  let tree = repoRoot;
  let boardsDir = defaultBoardsDir();
  let generator = DEFAULT_GENERATOR;
  let reps = 7;
  let shortS = 6;
  let longS = 66;
  let variant: "running" | "frozen" = "running";
  for (const arg of process.argv.slice(2)) {
    if (arg === "--") continue;
    const [flag, value = ""] = arg.split("=", 2) as [string, string | undefined];
    if (flag === "--tree") tree = path.resolve(value);
    else if (flag === "--boards") boardsDir = path.resolve(value);
    else if (flag === "--generator") generator = path.resolve(value);
    else if (flag === "--reps") reps = Number(value);
    else if (flag === "--short") shortS = Number(value);
    else if (flag === "--long") longS = Number(value);
    else if (flag === "--variant" && (value === "running" || value === "frozen")) variant = value;
    else throw new Error(`sap1-rate: unknown or malformed argument ${arg}`);
  }
  if (!(reps >= 1) || !(shortS > 0) || !(longS > shortS)) throw new Error("sap1-rate: need --reps >= 1 and 0 < --short < --long");
  const src = (rel: string) => pathToFileURL(path.join(tree, "src", rel)).href;
  const core = (await import(src("core.js"))) as { CoreEngine: new () => Engine };
  const wasm = (await import(src("core/wasm.js"))) as { initCore: () => Promise<void> };
  await wasm.initCore();

  console.log(`sap1-rate: pid ${String(process.pid)} (this process times itself with process.cpuUsage), tree ${tree}`);
  console.log(`  node ${process.version}, ${os.cpus()[0]?.model ?? "?"}, load ${os.loadavg()[0]!.toFixed(2)}, variant ${variant}, ${String(reps)} reps after 1 warm-up rep, ${String(shortS)} s -> ${String(longS)} s`);

  const summary: { board: string; med: number; min: number; max: number }[] = [];
  for (const catalog of [false, true]) {
    const file = sap1BoardFiles({ boardsDir, generator, power: "ideal", catalog }).find((b) => b.variant === variant)!.file;
    const { default: fs } = await import("node:fs");
    const circuit = JSON.parse(fs.readFileSync(file, "utf8")) as { bench: Sap1Bench };
    const label = catalog ? "ideal catalog" : "ideal plain";
    const rates: number[] = [];
    for (let rep = 0; rep <= reps; rep++) {
      const e = new core.CoreEngine();
      try {
        e.load(circuit);
        e.setMemoryContents(circuit.bench.ramLo, programNibbles(circuit.bench.program, 0));
        e.setMemoryContents(circuit.bench.ramHi, programNibbles(circuit.bench.program, 4));
        let frames = 0;
        const advanceTo = (s: number) => {
          for (; frames < Math.round(s / FRAME_S); frames++) e.advance(e.simTime + FRAME_S);
        };
        advanceTo(shortS);
        const [c0, t0] = [cpuMs(), e.simTime];
        advanceTo(longS);
        const rate = (cpuMs() - c0) / (e.simTime - t0);
        if (!e.lastConverged) throw new Error(`sap1-rate: ${label} did not converge`);
        console.log(`  ${label.padEnd(14)} rep ${String(rep)}${rep === 0 ? " (warm-up, not counted)" : ""}: ${rate.toFixed(2)} CPU ms per simulated second`);
        if (rep > 0) rates.push(rate);
      } finally {
        e.dispose();
      }
    }
    summary.push({ board: label, med: median(rates), min: Math.min(...rates), max: Math.max(...rates) });
  }
  console.log("\nCPU ms per simulated second (median, min .. max, spread of the reps):");
  for (const s of summary) console.log(`  ${s.board.padEnd(14)} ${s.med.toFixed(2)}  (${s.min.toFixed(2)} .. ${s.max.toFixed(2)}, ${(((s.max - s.min) / s.med) * 100).toFixed(1)} %)`);
  if (summary.length === 2) console.log(`  catalog / plain ${(summary[1]!.med / summary[0]!.med).toFixed(2)}x`);
}

main().catch((err: unknown) => {
  console.error(err instanceof Error ? err.message : err);
  process.exitCode = 1;
});
