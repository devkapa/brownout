/**
 * The two SAP-1 boards the perf tools run, shared by identity.ts, frames.ts and
 * sap1-cost.ts.
 *
 * Decision 12 keeps the generator outside both repos, read by path. The boards
 * are the "max" clock variants: "frozen" (FREEZE_DISPLAY=1, the display 555
 * held in reset) and "running" (the display multiplexes), on the ideal supply
 * unless a caller asks for the battery (sap1-cost.ts), named exactly as
 * scripts/core-accept-sap1.ts's generateBoard names them, so a work directory
 * that script filled can be passed straight to --boards.
 */
import { spawnSync } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";

import { encodeRamContents } from "../../src/circuit/types.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

export const DEFAULT_GENERATOR = "/Users/dev/Documents/DEVOLTProject/brownout-core-prototype/bench/gen-sap1.mjs";

export type Sap1Variant = "frozen" | "running";

/** The generator's supply: an ideal 5 V source, or a battery pack on the rail. */
export type Sap1Power = "ideal" | "battery";

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
  /** The supply (default ideal), in the file name as `sap1-max-<power>-<variant>.sim.json`. */
  power?: Sap1Power;
  /**
   * Give every component the `catalogUid` devolt's catalog assigns it, as the
   * app sends the board ([`withAppCatalogUids`]); the file is then
   * `sap1-max-<power>-<variant>-catalog.sim.json`. Off by default, so the
   * figures taken without it stay reproducible.
   */
  catalog?: boolean;
}

/**
 * The `catalogUid` devolt's catalog assigns each SAP-1 component, as the app
 * sends the board (THERM, read off the app's own circuit,
 * ship-logs/rc-devolt/tier/app-circuit.json): by kind, with the board's blue
 * LEDs and its ceramic capacitor by id. The battery and ground carry none.
 * A uid gives every resistor and LED a thermal record (code 91), which keeps
 * them hot: the generator's boards without it never measured that workload.
 */
export const SAP1_CATALOG_UIDS: { readonly byKind: Readonly<Record<string, string>>; readonly byId: Readonly<Record<string, string>> } = {
  byKind: {
    resistor: "resistor",
    led: "led-red",
    capacitor: "capacitor",
    ne555: "ne555",
    push_button: "switch-push",
    spdt_switch: "switch-spdt",
    switch: "switch-spst",
    dip_switch: "dip-switch",
    seg7_cc: "seg7-cc",
    "28c16": "ic-28c16",
    "74hc138": "ic-74hc138",
    "74hc74": "ic-74hc74",
    "74ls00": "ic-74ls00",
    "74ls04": "ic-74ls04",
    "74ls08": "ic-74ls08",
    "74ls157": "ic-74ls157",
    "74ls161": "ic-74ls161",
    "74ls173": "ic-74ls173",
    "74ls189": "ic-74ls189",
    "74ls245": "ic-74ls245",
    "74ls283": "ic-74ls283",
    "74ls32": "ic-74ls32",
    "74ls86": "ic-74ls86",
  },
  byId: {
    C7: "cap-ceramic",
    ...Object.fromEntries(["LED4", ...Array.from({ length: 16 }, (_, k) => `LED${String(68 + k)}`)].map((id) => [id, "led-blue"])),
  },
};

/** A copy of `circuit` with the app's `catalogUid`s ([`SAP1_CATALOG_UIDS`]). */
export function withAppCatalogUids<C extends { components: { id: string; kind: string; catalogUid?: string }[] }>(circuit: C): C {
  const board = structuredClone(circuit);
  for (const c of board.components) {
    const uid = SAP1_CATALOG_UIDS.byId[c.id] ?? SAP1_CATALOG_UIDS.byKind[c.kind];
    if (uid !== undefined) c.catalogUid = uid;
  }
  return board;
}

export function defaultBoardsDir(): string {
  return path.join(os.tmpdir(), "brownout-perf-boards");
}

/** The path of each variant's board, generating any missing one. Throws an Error naming the fix when it cannot. */
export function sap1BoardFiles(options: BoardOptions = {}): { name: string; variant: Sap1Variant; file: string }[] {
  const dir = options.boardsDir ?? defaultBoardsDir();
  const generator = options.generator ?? DEFAULT_GENERATOR;
  const power = options.power ?? "ideal";
  const out: { name: string; variant: Sap1Variant; file: string }[] = [];
  for (const variant of ["frozen", "running"] as const) {
    const explicit = options.files?.[variant];
    const plain = path.join(dir, `sap1-max-${power}-${variant}.sim.json`);
    const file = explicit ?? (options.catalog ? path.join(dir, `sap1-max-${power}-${variant}-catalog.sim.json`) : plain);
    if (options.catalog && !explicit && !fs.existsSync(file)) {
      // Built from the plain board (generated first if missing), so the two
      // differ in their uids alone.
      const [base] = sap1BoardFiles({ ...options, catalog: false }).filter((b) => b.variant === variant);
      fs.writeFileSync(file, JSON.stringify(withAppCatalogUids(loadSap1(base!.file))));
    }
    if (!fs.existsSync(file)) {
      if (explicit) throw new Error(`${file} does not exist`);
      if (!fs.existsSync(generator)) {
        throw new Error(`no SAP-1 board at ${file} and no generator at ${generator} (pass --boards, --frozen/--running or --generator)`);
      }
      fs.mkdirSync(dir, { recursive: true });
      const env = { ...process.env };
      delete env.FREEZE_DISPLAY;
      if (variant === "frozen") env.FREEZE_DISPLAY = "1";
      const result = spawnSync(process.execPath, [generator, "max", power, file], { env, encoding: "utf8" });
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
 * `circuit` with its program saved in the two RAM chips' own
 * `params.contents` (a power-on preload, src/circuit/types.ts RAM_KINDS): the
 * board then runs the program from load on either engine, with no
 * setMemoryContents and no pokeClassicRam. A copy; `circuit` is untouched.
 */
export function withProgramInRam(circuit: Sap1Circuit): Sap1Circuit {
  const board = structuredClone(circuit);
  for (const [id, shift] of [[board.bench.ramLo, 0], [board.bench.ramHi, 4]] as const) {
    const ram = board.components.find((c) => c.id === id);
    if (!ram || ram.kind !== "74ls189") throw new Error(`withProgramInRam: ${id} is not a 74ls189 on this board`);
    ram.params.contents = encodeRamContents("74ls189", programNibbles(board.bench.program, shift));
  }
  return board;
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
