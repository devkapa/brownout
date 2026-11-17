/**
 * The W2b cost run (core/README.md, "Charge", ruling R107 (a)): CPU ms per
 * simulated second of the rated SAP-1 (ideal supply, display running)
 * unarmed, with 2 probes at 10 kS/s, with the means (every LED and
 * seven-segment display charged), and with 4 probes at 10 kS/s plus the
 * means, the budget case. Interleaved reps in one process per tree, a take
 * of both after each 1/60 s frame, as a host does.
 *
 * Usage (from a worktree, after `pnpm run build:core`; on a quiet machine):
 *   pnpm run bench:probe-cost -- [--reps=5] [--seconds=20] [--boards=<dir>] [--gate] <tree>
 *
 * --gate fails (exit 1) when the budget case's median is more than +15 %
 * over the unarmed median (R107 (a)'s ceiling). The unarmed cost against the
 * base stays the sap1-cost gate's (within noise).
 */
import path from "node:path";
import { pathToFileURL } from "node:url";
import { defaultBoardsDir, loadSap1, programNibbles, sap1BoardFiles } from "./boards.js";

/** R107 (a): the budget case over the unarmed run, at most. */
export const PROBE_COST_CEILING = 0.15;
const FRAME = 1 / 60;

interface Case {
  name: string;
  probes: [string, string][];
  means: boolean;
}

const CASES: Case[] = [
  { name: "unarmed", probes: [], means: false },
  { name: "2 probes 10kS/s", probes: [["U1", "6"], ["U1", "3"]], means: false },
  { name: "means", probes: [], means: true },
  { name: "4 probes 10kS/s + means", probes: [["U1", "6"], ["U1", "3"], ["LED1", "a"], ["LED2", "a"]], means: true },
];

function median(xs: number[]): number {
  const s = [...xs].sort((a, b) => a - b);
  const m = s.length >> 1;
  return s.length % 2 ? s[m]! : (s[m - 1]! + s[m]!) / 2;
}

async function main(): Promise<void> {
  let reps = 5;
  let seconds = 20;
  let boardsDir = defaultBoardsDir();
  let gate = false;
  let tree = ".";
  for (const arg of process.argv.slice(2)) {
    const [flag, value] = arg.split("=", 2) as [string, string | undefined];
    if (flag === "--reps") reps = Number(value);
    else if (flag === "--seconds") seconds = Number(value);
    else if (flag === "--boards") boardsDir = path.resolve(value ?? "");
    else if (flag === "--gate") gate = true;
    else if (!flag.startsWith("--")) tree = path.resolve(flag);
  }
  const src = (rel: string) => pathToFileURL(path.join(tree, "src", rel)).href;
  const { CoreEngine } = (await import(src("core/engine.js"))) as { CoreEngine: new () => any };
  const wasm = (await import(src("core/wasm.js"))) as any;
  await wasm.initCore();
  const board = sap1BoardFiles({ boardsDir }).find((b) => b.variant === "running")!;
  const sap = loadSap1(board.file) as any;
  const cpu = () => {
    const u = process.cpuUsage();
    return (u.user + u.system) / 1000;
  };
  const run = (c: Case): number => {
    const e = new CoreEngine();
    e.load(structuredClone(sap), undefined, sap.options);
    e.setMemoryContents(sap.bench.ramLo, programNibbles(sap.bench.program, 0));
    e.setMemoryContents(sap.bench.ramHi, programNibbles(sap.bench.program, 4));
    if (c.probes.length > 0) e.acquire(c.probes.map(([id, pin]) => e.getNetIdForPin(id, pin)), 0, 1e-4);
    if (c.means) e.armCharge(sap.components.filter((x: any) => x.kind === "led" || x.kind.startsWith("seg7")).map((x: any) => x.id));
    const take = () => {
      if (c.probes.length > 0) e.takeSamples();
      if (c.means) e.takeCharge();
    };
    for (let t = FRAME; t <= 2 + 1e-12; t += FRAME) {
      e.advance(t);
      take();
    }
    const s0 = e.simTime;
    const c0 = cpu();
    const frames = Math.round(seconds / FRAME);
    for (let f = 0; f < frames; f++) {
      e.advance(e.simTime + FRAME);
      take();
    }
    const ms = (cpu() - c0) / (e.simTime - s0);
    e.dispose();
    return ms;
  };
  const results = new Map<string, number[]>(CASES.map((c) => [c.name, []]));
  for (let r = 0; r < reps; r++) {
    for (const c of r % 2 === 0 ? CASES : [...CASES].reverse()) results.get(c.name)!.push(run(c));
  }
  const base = median(results.get("unarmed")!);
  let budget = 0;
  for (const c of CASES) {
    const m = median(results.get(c.name)!);
    const change = m / base - 1;
    if (c.name === "4 probes 10kS/s + means") budget = change;
    console.log(`${c.name.padEnd(26)} ${m.toFixed(2)} ms/sim-s  ${(change * 100).toFixed(1).padStart(6)} %  [${results.get(c.name)!.map((x) => x.toFixed(2)).join(", ")}]`);
  }
  if (gate) {
    const ok = budget <= PROBE_COST_CEILING;
    console.log(`gate: the budget case ${(budget * 100).toFixed(1)} % against the ceiling ${(PROBE_COST_CEILING * 100).toFixed(0)} %: ${ok ? "PASS" : "FAIL"}`);
    if (!ok) process.exitCode = 1;
  }
}

await main();
