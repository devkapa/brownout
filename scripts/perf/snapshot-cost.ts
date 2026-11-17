/**
 * W2a's snapshot cost on the SAP-1 (core/README.md, "Snapshots"): the time a
 * `saveState()` and a `restoreState()` take, the bytes a snapshot holds
 * (`snapshotBytes()`), and the time of the first 28C16 write after a
 * snapshot, which pays the copy-on-write copy of its image.
 *
 * Usage (from a worktree, after `pnpm run build:core`):
 *   pnpm run bench:snapshot-cost -- [--boards=<dir>] [--warmup=<s>] [--count=<n>]
 *
 * PROTOCOL. Per board (frozen, then running): load, seed the two RAM chips,
 * run --warmup s (default 5) of 1/60 s frames, then --count (default 200)
 * times: save, advance one frame, restore, each call timed with
 * `performance.now()`; then the medians and the 90th percentiles. The bytes
 * are the table's total with --count snapshots held, divided by --count. All
 * snapshots are disposed before the next board.
 *
 * The copy-on-write line runs on fixture 29 (test/fixtures/core): its 28C16
 * latches a byte in phase 1 and completes the write 0.5 ms later. The step
 * that completes it is timed --count times with a snapshot held (the image
 * is shared, so the write copies it) and --count times without (the image is
 * the engine's own), each from a restore of the same state.
 */
import fs from "node:fs";
import path from "node:path";

import { CoreEngine } from "../../src/core/engine.js";
import { initCore } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import type { StimulusPhase } from "../../test/core/equivalence/stimuli.js";
import { loadSyntheticCorpus } from "../../test/core/equivalence/synthetic-fixtures.js";
import { defaultBoardsDir, programNibbles, sap1BoardFiles, type Sap1Bench } from "./boards.js";

const FRAME_S = 1 / 60;

function quantile(xs: number[], q: number): number {
  const s = [...xs].sort((a, b) => a - b);
  return s[Math.min(s.length - 1, Math.floor(q * s.length))]!;
}

function us(ms: number): string {
  return `${(ms * 1000).toFixed(1)} us`;
}

function sap1(file: string, warmupS: number, count: number): void {
  const circuit = JSON.parse(fs.readFileSync(file, "utf8")) as SimCircuit & { bench: Sap1Bench };
  const e = new CoreEngine();
  try {
    e.load(circuit);
    e.setMemoryContents(circuit.bench.ramLo, programNibbles(circuit.bench.program, 0));
    e.setMemoryContents(circuit.bench.ramHi, programNibbles(circuit.bench.program, 4));
    for (let f = 0; f < Math.round(warmupS / FRAME_S); f++) e.advance(e.simTime + FRAME_S);
    const [save, restore] = [[] as number[], [] as number[]];
    const held = [];
    for (let i = 0; i < count; i++) {
      let t0 = performance.now();
      const s = e.saveState();
      save.push(performance.now() - t0);
      held.push(s);
      const at = e.simTime;
      e.advance(at + FRAME_S);
      t0 = performance.now();
      e.restoreState(s);
      restore.push(performance.now() - t0);
      // Go on past it, so the next snapshot is of a later state.
      e.advance(at + FRAME_S);
    }
    const bytes = e.snapshotBytes() / count;
    for (const s of held) s.dispose();
    console.log(
      `${path.basename(file)}: save ${us(quantile(save, 0.5))} (p90 ${us(quantile(save, 0.9))}), ` +
        `restore ${us(quantile(restore, 0.5))} (p90 ${us(quantile(restore, 0.9))}), ` +
        `${(bytes / 1024).toFixed(1)} KiB a snapshot (${count} held: ${((bytes * count) / 1048576).toFixed(1)} MiB); ` +
        `after dispose ${e.snapshotBytes()} B`,
    );
  } finally {
    e.dispose();
  }
}

/**
 * Fixture 29's stops at 10 us, its phases' sets applied at their starts, and
 * the index of the stop whose advance completes the 28C16's write (the first
 * that reports an EEPROM update).
 */
function fixture29(): { circuit: SimCircuit; stops: { t: number; set?: StimulusPhase["set"] }[]; completes: number } {
  const loaded = loadSyntheticCorpus().find((x) => x.entry.name.startsWith("29-"))!;
  const phases = loaded.entry.phases!;
  const first = phases[0]?.set ?? [];
  const circuit: SimCircuit = {
    ...loaded.circuit,
    components: loaded.circuit.components.map((c) => {
      const o = first.find((x) => x.id === c.id);
      return o ? { ...c, params: { ...c.params, ...o.params } } : c;
    }),
  };
  const stops: { t: number; set?: StimulusPhase["set"] }[] = [];
  let t = 0;
  phases.forEach((phase, i) => {
    const n = Math.max(1, Math.round(phase.durationS / 10e-6));
    for (let k = 1; k <= n; k++) stops.push({ t: t + (phase.durationS * k) / n, ...(k === 1 && i > 0 && phase.set ? { set: phase.set } : {}) });
    t += phase.durationS;
  });
  const e = new CoreEngine();
  try {
    e.load(circuit);
    for (let k = 0; k < stops.length; k++) {
      step(e, stops[k]!);
      if (Object.keys(e.takeEepromUpdates()).length > 0) return { circuit, stops, completes: k };
    }
  } finally {
    e.dispose();
  }
  throw new Error("snapshot-cost: fixture 29's write never completed");
}

function step(e: CoreEngine, stop: { t: number; set?: StimulusPhase["set"] }): void {
  if (stop.set) e.setParams(stop.set.map((o) => ({ componentId: o.id, params: o.params })));
  e.advance(stop.t);
}

/**
 * The step that completes the write, timed with the image shared with a
 * snapshot (taken at the stop before; the write copies it) and on a fresh
 * engine that never took one (the image is its own; no copy).
 */
function eepromWrite(count: number): void {
  const { circuit, stops, completes } = fixture29();
  const shared: number[] = [];
  const own: number[] = [];
  const e = new CoreEngine();
  try {
    e.load(circuit);
    for (let k = 0; k < completes; k++) step(e, stops[k]!);
    const before = e.saveState();
    for (let i = 0; i < count; i++) {
      e.restoreState(before);
      const t0 = performance.now();
      step(e, stops[completes]!);
      shared.push(performance.now() - t0);
    }
    before.dispose();
  } finally {
    e.dispose();
  }
  for (let i = 0; i < count; i++) {
    const f = new CoreEngine();
    try {
      f.load(circuit);
      for (let k = 0; k < completes; k++) step(f, stops[k]!);
      const t0 = performance.now();
      step(f, stops[completes]!);
      own.push(performance.now() - t0);
    } finally {
      f.dispose();
    }
  }
  console.log(
    `28C16 write completion (fixture 29, 2 KiB image): with the image shared with a snapshot ${us(quantile(shared, 0.5))} ` +
      `(p90 ${us(quantile(shared, 0.9))}), with its own image ${us(quantile(own, 0.5))} (p90 ${us(quantile(own, 0.9))}); ` +
      `the copy is the difference, ${us(quantile(shared, 0.5) - quantile(own, 0.5))}`,
  );
}

async function main(): Promise<void> {
  let boardsDir = defaultBoardsDir();
  let warmupS = 5;
  let count = 200;
  for (const arg of process.argv.slice(2)) {
    if (arg === "--") continue;
    const [flag, value] = arg.split("=", 2) as [string, string | undefined];
    if (flag === "--boards") boardsDir = path.resolve(value ?? "");
    else if (flag === "--warmup") warmupS = Number(value);
    else if (flag === "--count") count = Number(value);
    else throw new Error(`snapshot-cost: unknown flag ${arg}`);
  }
  await initCore();
  for (const { file } of sap1BoardFiles({ boardsDir })) sap1(file, warmupS, count);
  eepromWrite(count);
}

main().catch((err: unknown) => {
  console.error(err instanceof Error ? err.message : err);
  process.exitCode = 1;
});
