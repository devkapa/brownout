/**
 * The two SAP-1 boards the perf tools run, shared by identity.ts and frames.ts.
 *
 * Decision 12 keeps the generator outside both repos, read by path. The boards
 * are the "max" clock, ideal supply variants: "frozen" (FREEZE_DISPLAY=1, the
 * display 555 held in reset) and "running" (the display multiplexes), named
 * exactly as scripts/core-accept-sap1.ts's generateBoard names them, so a work
 * directory that script filled can be passed straight to --boards.
 */
import { spawnSync } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";

import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

export const DEFAULT_GENERATOR = "/Users/dev/Documents/DEVOLTProject/brownout-core-prototype/bench/gen-sap1.mjs";

export type Sap1Variant = "frozen" | "running";

export interface Sap1Bench {
  readonly program: Readonly<Record<string, number>>;
  readonly ramLo: string;
  readonly ramHi: string;
}

export type Sap1Circuit = SimCircuit & { bench: Sap1Bench };

export interface BoardOptions {
  /** Directory holding (or receiving) sap1-max-ideal-<variant>.sim.json; default is a per-user temp directory. */
  boardsDir?: string;
  /** Explicit board files, overriding boardsDir for that variant. */
  files?: Partial<Record<Sap1Variant, string>>;
  generator?: string;
}

export function defaultBoardsDir(): string {
  return path.join(os.tmpdir(), "brownout-perf-boards");
}

/** The path of each variant's board, generating any missing one. Throws an Error naming the fix when it cannot. */
export function sap1BoardFiles(options: BoardOptions = {}): { name: string; variant: Sap1Variant; file: string }[] {
  const dir = options.boardsDir ?? defaultBoardsDir();
  const generator = options.generator ?? DEFAULT_GENERATOR;
  const out: { name: string; variant: Sap1Variant; file: string }[] = [];
  for (const variant of ["frozen", "running"] as const) {
    const explicit = options.files?.[variant];
    const file = explicit ?? path.join(dir, `sap1-max-ideal-${variant}.sim.json`);
    if (!fs.existsSync(file)) {
      if (explicit) throw new Error(`${file} does not exist`);
      if (!fs.existsSync(generator)) {
        throw new Error(`no SAP-1 board at ${file} and no generator at ${generator} (pass --boards, --frozen/--running or --generator)`);
      }
      fs.mkdirSync(dir, { recursive: true });
      const env = { ...process.env };
      delete env.FREEZE_DISPLAY;
      if (variant === "frozen") env.FREEZE_DISPLAY = "1";
      const result = spawnSync(process.execPath, [generator, "max", "ideal", file], { env, encoding: "utf8" });
      if (result.error || result.status !== 0) {
        throw new Error(`gen-sap1 failed: ${result.error?.message ?? `exit ${String(result.status)}`}\n${result.stderr}`);
      }
    }
    out.push({ name: `sap1-${variant}`, variant, file });
  }
  return out;
}

export function loadSap1(file: string): Sap1Circuit {
  return JSON.parse(fs.readFileSync(file, "utf8")) as Sap1Circuit;
}

/** RAM contents of the 74ls189 pair: the program byte for address a, split into its low and high nibble. */
export function programNibbles(program: Readonly<Record<string, number>>, shift: 0 | 4): number[] {
  return Array.from({ length: 16 }, (_, a) => ((program[String(a)] ?? 0xff) >> shift) & 0xf);
}

/**
 * The classic engine's counterpart of CoreEngine.setMemoryContents for the two
 * SAP-1 RAM chips: classic has no facade for it, so the engine's icState is
 * written directly, as scripts/core-accept-sap1.ts's pokeClassicRam and the
 * prototype's bench/sap1.mjs do (test-only; `lastWE: 1` is the idle write-enable
 * the 74ls189 model expects).
 */
export function pokeClassicRam(engine: unknown, bench: Sap1Bench): void {
  const icState = (engine as { state: { icState: Map<string, Record<string, number>> } }).state.icState;
  for (const [id, shift] of [[bench.ramLo, 0], [bench.ramHi, 4]] as const) {
    const contents: Record<string, number> = { lastWE: 1 };
    programNibbles(bench.program, shift).forEach((nibble, a) => {
      contents[`mem${String(a)}`] = nibble;
    });
    icState.set(id, contents);
  }
}
