/**
 * CLK: the battery SAP-1's clock is the same at every host cadence. Its 555
 * astable (U1: R_A = R1 = 1 kohm, R_B = R2 = 1 kohm, C2 = 1 uF) rides the
 * battery pack's coupled path, where the threshold crossing (C2 charging to
 * 2/3 of the rail) was searched only to the rail's next solve and given up
 * when the sum turned past it: the flip waited for the next host stop. At
 * f18bfe5 the clock ran 0.68 % slow at 10 us stops, 6.2 % at 100 us and 48 %
 * at a 60 Hz frame (48 triggers in 0.1 s at 1 us stops, 33 at 1/60 s).
 *
 * The board is the perf tools' battery SAP-1 (scripts/perf/boards.ts), frozen
 * display, its program in the RAMs' contents; skipped when neither the board
 * nor its generator is on this machine.
 *
 * Each cadence loads the board and runs the same 60 Hz frames to T0, so all
 * start from one state; from T0 the host stops every `host` for 0.1 s (about
 * 47 periods of 2.1 ms) and the 555's output flips are read off the engine's
 * own trace, each at its recorded instant.
 *
 * Bound, per flip, index-matched against the 1 us run: tau_w0, the time C2
 * takes to cross W0 of its trigger level. At the trigger C2 discharges from
 * u / 3 (the pack at 4.83 V: 1.61 V) through R_B and DIS's 10 ohm: 1.61 /
 * 1010 = 1.594 mA into 1 uF, 1594 V/s; W0_V(1.61) = 1e-4 + 1.61e-3 = 1.71 mV,
 * so tau_w0 = 1.07 us. (The threshold, charging through R_A + R_B at
 * (u / 3) / 2000 ohm = 0.805 mA, 805 V/s, has W0_V(3.22) / 805 = 4.1 us: the
 * trigger's is the tighter, held for both.)
 */
import fs from "node:fs";
import path from "node:path";
import { beforeAll, describe, expect, it } from "vitest";
import { CORE_IMPLEMENTED_KINDS, CoreEngine } from "../../src/core.js";
import { initCore } from "../../src/core/wasm.js";
import { DEFAULT_GENERATOR, defaultBoardsDir, loadSap1, sap1BoardFiles, withProgramInRam, type Sap1Circuit } from "../../scripts/perf/boards.js";

const BOARDS_DIR = process.env.BROWNOUT_SAP1_BOARDS ?? defaultBoardsDir();

function findBattery(): Sap1Circuit | null {
  const file = path.join(BOARDS_DIR, "sap1-max-battery-frozen.sim.json");
  if (!fs.existsSync(file) && !fs.existsSync(DEFAULT_GENERATOR)) return null;
  const frozen = sap1BoardFiles({ boardsDir: BOARDS_DIR, power: "battery" }).find((b) => b.variant === "frozen");
  return frozen ? loadSap1(frozen.file) : null;
}

const battery = findBattery();
if (!battery) console.warn(`sap1-clock-cadence: SKIPPED. No battery SAP-1 board in ${BOARDS_DIR} and no generator at ${DEFAULT_GENERATOR}.`);

const T0 = 0.5;
const SPAN = 0.1;
const TAU_W0 = (1e-4 + 1e-3 * 1.61) / (1.61 / 1010 / 1e-6);

/** The clock 555's (U1's) rises and falls from T0 to T0 + SPAN with a host stopping every `host` seconds. */
function clockFlips(board: Sap1Circuit, host: number): { up: number[]; down: number[] } {
  const engine = new CoreEngine();
  try {
    engine.load(board, CORE_IMPLEMENTED_KINDS);
    for (let f = 1; f * (1 / 60) <= T0 + 1e-12; f++) engine.advance(f / 60);
    const start = engine.simTime;
    engine.enableTrace(true);
    engine.clearTrace();
    const up: number[] = [];
    const down: number[] = [];
    for (let k = 1; ; k++) {
      const t = Math.min(start + k * host, start + SPAN);
      engine.advance(t);
      for (const ev of engine.readTrace()) {
        // A 555's one digital slot is keyed by its bare component id (compile/ne555.ts).
        if (ev.key !== "U1") continue;
        if (ev.value === 1) up.push(ev.t);
        else if (ev.value === 0) down.push(ev.t);
      }
      engine.clearTrace();
      if (t >= start + SPAN) break;
    }
    return { up, down };
  } finally {
    engine.dispose();
  }
}

describe.skipIf(!battery)("the battery SAP-1's clock at every host cadence (CLK)", () => {
  const board = battery ? withProgramInRam(battery) : (undefined as never);
  let fine: { up: number[]; down: number[] };

  beforeAll(async () => {
    await initCore();
    fine = clockFlips(board, 1e-6);
  });

  it("the 1 us run sees the clock: about 47 periods of 2.1 ms in 0.1 s", () => {
    expect(fine.up.length).toBeGreaterThanOrEqual(40);
    expect(fine.down.length).toBeGreaterThanOrEqual(40);
  });

  for (const host of [1e-5, 1e-4, 1e-3, 1 / 60, 0.1]) {
    it(`host stops every ${host.toExponential(3)} s: every flip within tau_w0 of the 1 us run's`, () => {
      const got = clockFlips(board, host);
      for (const [name, g, f] of [["trigger", got.up, fine.up], ["threshold", got.down, fine.down]] as const) {
        expect(g.length, `${name}s`).toBe(f.length);
        const worst = Math.max(...g.map((t, i) => Math.abs(t - f[i]!)));
        console.log(`battery SAP-1 at host ${host.toExponential(3)}: worst ${name} ${worst.toExponential(3)} s = ${(worst / TAU_W0).toFixed(3)} tau_w0`);
        expect(worst, `worst ${name} against tau_w0 ${TAU_W0.toExponential(3)} s`).toBeLessThanOrEqual(TAU_W0);
      }
    });
  }
});
