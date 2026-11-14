/**
 * The RAM preload's showcase (coordinator ruling 2026-11-13): the SAP-1 saved
 * with its program in the two 74LS189s' `params.contents` runs that program
 * from load, with no switch presses, no `setMemoryContents` and no poke of
 * classic's icState, on both engines; and the two agree on the output
 * register.
 *
 * THE BOARD is the perf tools' SAP-1 (scripts/perf/boards.ts; Decision 12
 * keeps its generator outside both repos). `BROWNOUT_SAP1_BOARDS` names a
 * directory holding `sap1-max-ideal-frozen.sim.json`; without it the default
 * perf directory is used, and a missing board is generated when the
 * generator is on this machine. With neither, the suite is skipped and says
 * so: it cannot run without the board.
 *
 * THE PROGRAM is the generator's own (Ben Eater's up/down counter):
 *   0: OUT   1: ADD 15   2: JC 4   3: JMP 0
 *   4: SUB 15   5: OUT   6: JZ 0   7: JMP 4   15: the constant 1
 * A starts at 0 (the reset pulse), so the output register shows 0, 1, 2, ...
 * 255, then the carry sends it down: 254, ... 1, 0, and JZ sends it up again.
 * Each address of a blank RAM reads 0xFF, which is HLT: a board with no
 * preload and no seed never leaves 0 (the detector case below).
 *
 * WHAT CLASSIC COVERS. Classic costs about 160 CPU-seconds per simulated
 * second on this board, and one program cycle is about 21.5 simulated
 * seconds (512 output values, 42 ms each). The default run is 0.3 s, which
 * shows the outputs 0..6; `BROWNOUT_SAP1_CLASSIC_SECONDS=22` runs the whole
 * cycle (about an hour of CPU). The core runs the whole cycle every time.
 */
import fs from "node:fs";
import path from "node:path";
import { beforeAll, describe, expect, it } from "vitest";
import { CORE_IMPLEMENTED_KINDS, CoreEngine } from "../../src/core.js";
import { initCore } from "../../src/core/wasm.js";
import { HeadlessRunner } from "../../src/host/headless.js";
import { ramPreloadWords } from "../../src/circuit/types.js";
import { DEFAULT_GENERATOR, defaultBoardsDir, loadSap1, programNibbles, sap1BoardFiles, withProgramInRam, type Sap1Circuit, type Sap1Power } from "../../scripts/perf/boards.js";
import { digitalOutputPins } from "./equivalence/known-divergences.js";

const BOARDS_DIR = process.env.BROWNOUT_SAP1_BOARDS ?? defaultBoardsDir();
const CLASSIC_SECONDS = Number(process.env.BROWNOUT_SAP1_CLASSIC_SECONDS ?? 0.3);

/** The frozen-display board on `power` (the display section plays no part in the output register), or null when it is not on this machine. */
function findBoard(power: Sap1Power): Sap1Circuit | null {
  const file = path.join(BOARDS_DIR, `sap1-max-${power}-frozen.sim.json`);
  if (!fs.existsSync(file) && !fs.existsSync(DEFAULT_GENERATOR)) return null;
  const frozen = sap1BoardFiles({ boardsDir: BOARDS_DIR, power }).find((b) => b.variant === "frozen");
  return frozen ? loadSap1(frozen.file) : null;
}

const base = findBoard("ideal");
const batteryBase = findBoard("battery");
if (!base) {
  console.warn(`sap1-ram-preload: SKIPPED. No SAP-1 board in ${BOARDS_DIR} and no generator at ${DEFAULT_GENERATOR}; set BROWNOUT_SAP1_BOARDS.`);
}

/** The eight output-register keys, bit 0 first: the one digital output pin on each of the generator's nets OUT0..OUT7. */
function outKeysOf(circuit: Sap1Circuit): string[] {
  const netNames = (circuit.bench as Sap1Circuit["bench"] & { netNames: Record<string, string> }).netNames;
  const kindOf = new Map(circuit.components.map((c) => [c.id, c.kind]));
  return Array.from({ length: 8 }, (_, bit) => {
    const drivers = Object.entries(netNames)
      .filter(([, net]) => net === `OUT${bit}`)
      .map(([ref]) => [ref.slice(0, ref.lastIndexOf(".")), ref.slice(ref.lastIndexOf(".") + 1)] as const)
      .filter(([id, pin]) => digitalOutputPins(kindOf.get(id) ?? "").has(pin));
    if (drivers.length !== 1) throw new Error(`OUT${bit} has ${drivers.length} register outputs on it, expected 1`);
    return `${drivers[0]![0]}/${drivers[0]![1]}`;
  });
}

function outValue(state: Readonly<Record<string, number>>, outKeys: readonly string[]): number {
  return outKeys.reduce((value, key, bit) => (state[key] === 1 ? value | (1 << bit) : value), 0);
}

/** One output value and the first sample time it was seen at. */
type Shown = readonly [value: number, at: number];

/** Appends `value` when it differs from the last one shown. */
function note(shown: Shown[], value: number, at: number): void {
  if (shown.length === 0 || shown[shown.length - 1]![0] !== value) shown.push([value, at]);
}

/** The core's output register from load, sampled every `dt`, until `done` says the sequence is complete or `maxSeconds` pass. */
function coreOutputs(circuit: Sap1Circuit, dt: number, maxSeconds: number, done: (shown: readonly Shown[]) => boolean): Shown[] {
  const outKeys = outKeysOf(circuit);
  const engine = new CoreEngine();
  try {
    engine.load(circuit, CORE_IMPLEMENTED_KINDS);
    const shown: Shown[] = [];
    note(shown, outValue(engine.digitalState, outKeys), engine.simTime);
    for (let k = 1; k * dt <= maxSeconds && !done(shown); k++) {
      engine.advance(k * dt);
      if (!engine.lastConverged) throw new Error(`the core did not converge at ${engine.simTime} s`);
      note(shown, outValue(engine.digitalState, outKeys), engine.simTime);
    }
    return shown;
  } finally {
    engine.dispose();
  }
}

/** Classic's output register from load over `seconds`, sampled every `dt`. */
function classicOutputs(circuit: Sap1Circuit, dt: number, seconds: number): Shown[] {
  const outKeys = outKeysOf(circuit);
  const runner = new HeadlessRunner();
  runner.load(circuit);
  const shown: Shown[] = [];
  note(shown, outValue(runner.engine.digitalState, outKeys), runner.engine.simTime);
  for (let k = 1; k * dt <= seconds + 1e-12; k++) {
    runner.run({ durationS: dt, maxSteps: 1e8 });
    note(shown, outValue(runner.engine.digitalState, outKeys), runner.engine.simTime);
  }
  return shown;
}

/** One program cycle of the up/down counter as the output register shows it: 0 up to 255, down to 0, and the 1 that starts the next. */
const ONE_CYCLE = [...Array.from({ length: 256 }, (_, i) => i), ...Array.from({ length: 255 }, (_, i) => 254 - i), 1];

/** Mean seconds between successive output values, over `shown[from..]`. */
function meanInterval(shown: readonly Shown[], from: number): number {
  return (shown[shown.length - 1]![1] - shown[from]![1]) / (shown.length - 1 - from);
}

describe.skipIf(!base)("SAP-1 with its program in the RAMs' contents", () => {
  const board = base ? withProgramInRam(base) : (undefined as never);

  beforeAll(async () => {
    await initCore();
  });

  it("the saved board carries the program in both RAM chips, low nibble and high", () => {
    const bench = board.bench;
    for (const [id, shift] of [[bench.ramLo, 0], [bench.ramHi, 4]] as const) {
      const ram = board.components.find((c) => c.id === id)!;
      // What both engines will load is the program's own nibble at every address (unprogrammed addresses 0xF).
      expect(ramPreloadWords("74ls189", ram.params.contents)).toEqual(programNibbles(bench.program, shift));
    }
    // The generator's board has no contents on them: the preload is this suite's only change to it.
    for (const id of [bench.ramLo, bench.ramHi]) expect(base!.components.find((c) => c.id === id)!.params.contents).toBeUndefined();
  });

  it("detector: the same board with no contents and no seed halts at 0, so the sequences below come from the preload", () => {
    // Blank RAM reads 0xFF at address 0: HLT. One simulated second is 23 output values on the preloaded board.
    const shown = coreOutputs(base!, 1e-3, 1, () => false);
    expect(shown.map(([value]) => value)).toEqual([0]);
  });

  it("core: runs the whole program from load, 0 up to 255 and back to 0, with nothing seeded", () => {
    // 512 values at about 42 ms each is 21.5 s; 30 s is the stop for a board that never gets there.
    const shown = coreOutputs(board, 1e-3, 30, (s) => s.length >= ONE_CYCLE.length);
    expect(shown.map(([value]) => value)).toEqual(ONE_CYCLE);
  });

  it.skipIf(!batteryBase)("core, on the battery pack: the whole program from load too", () => {
    // The same board behind the generator's battery, whose rail sags under the load. The clock runs slower on it
    // (about 48 ms an output, 24.6 s a cycle); 36 s is the stop.
    const shown = coreOutputs(withProgramInRam(batteryBase!), 1e-3, 36, (s) => s.length >= ONE_CYCLE.length);
    expect(shown.map(([value]) => value)).toEqual(ONE_CYCLE);
  });

  it("classic: runs the program from load with nothing poked, and the core shows the same outputs at the same pace", () => {
    const classic = classicOutputs(board, 2e-3, CLASSIC_SECONDS);
    const values = classic.map(([value]) => value);
    // Classic shows a prefix of the program's cycle, and gets past the first few outputs
    // (0.3 s reaches 6; a board stuck at 0 or showing garbage fails here).
    expect(values).toEqual(ONE_CYCLE.slice(0, values.length));
    expect(values.length).toBeGreaterThanOrEqual(Math.min(6, ONE_CYCLE.length));

    // The core over the same number of outputs: the same values in the same order.
    const core = coreOutputs(board, 1e-3, 30, (s) => s.length >= values.length);
    expect(core.map(([value]) => value)).toEqual(values);

    // And at the same pace. Each output follows the last by the same number of clock cycles, so the mean time per
    // output is the clock period's ratio. Classic's clock astable runs about 1.6 % slow against the core's
    // (core/README.md, NE555 point 13.2: a probe measured classic +1.58 % per period), and sampling every 2 ms over
    // the default run's five intervals adds up to 1 %: within 5 %. Measured from the second output on: the first
    // follows the reset pulse, where the engines start a few clock cycles apart, which is an offset, not a pace.
    const pace = meanInterval(classic, 1) / meanInterval(core, 1);
    expect(pace).toBeGreaterThan(0.95);
    expect(pace).toBeLessThan(1.05);
  });
});
