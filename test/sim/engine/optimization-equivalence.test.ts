import { describe, expect, it } from "vitest";
import fs from "node:fs";
import path from "node:path";
import type { Circuit } from "../../../src/circuit/types.js";
import { chaser555CounterCircuit } from "../../helpers/embedded-fixtures.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";

// B2 migration: the corpus fixtures were pre-converted with devolt's own
// breadboardToSimCircuit (one-shot, offline), so loading the *.sim.json is
// bit-identical to the runtime conversion the capture literals below were
// recorded against — and the app-domain converter drops out of this suite.
function loadFixture(name: string): Circuit {
  const root = path.resolve(__dirname, "../../fixtures/circuits");
  const file = name.endsWith(".sim.json") ? name : name.replace(/\.json$/, ".sim.json");
  return JSON.parse(fs.readFileSync(path.join(root, file), "utf8")) as Circuit;
}

function engineFor(circuit: Circuit): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  return engine;
}

function netFor(engine: SimEngine, componentId: string, pinId: string): string {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
  );
  if (!net) throw new Error(`No net for ${componentId}.${pinId}`);
  return net.id;
}

function rounded(value: number | undefined): number | null {
  if (value === undefined || !Number.isFinite(value)) return null;
  return Number(value.toPrecision(10));
}

function atCheckpoints<T>(
  engine: SimEngine,
  dt: number,
  checkpoints: readonly number[],
  read: () => T,
): T[] {
  const captures: T[] = [];
  let step = 0;
  for (const checkpoint of checkpoints) {
    while (step < checkpoint) {
      engine.step(dt);
      step += 1;
    }
    captures.push(read());
  }
  return captures;
}

/**
 * The capture literals below lock the DENSE baseline bit-for-bit: these tests
 * exist to catch any numeric drift from engine refactors. The forced-sparse
 * corpus lane (SIMCORE_LINEAR_BACKEND=sparse) legitimately rounds differently
 * because its elimination order differs, so under that lane the same
 * trajectories are asserted within solver roundoff instead of bitwise. Real
 * regressions move values by far more than the tolerance floor.
 */
const FORCED_SPARSE = process.env.SIMCORE_LINEAR_BACKEND === "sparse";

function expectCaptures<T extends Record<string, unknown>>(
  captures: T[],
  expected: T[],
): void {
  if (!FORCED_SPARSE) {
    expect(captures).toEqual(expected);
    return;
  }
  expect(captures.length).toBe(expected.length);
  captures.forEach((capture, index) => {
    const want = expected[index];
    for (const key of Object.keys(want)) {
      const actual = capture[key];
      const target = want[key];
      if (typeof target === "number" && typeof actual === "number") {
        expect(Math.abs(actual - target)).toBeLessThanOrEqual(
          1e-9 + 1e-6 * Math.abs(target),
        );
      } else {
        expect(actual).toEqual(target);
      }
    }
  });
}

describe("optimization equivalence baselines", () => {
  it("compiles the same deterministic pin-to-net mapping as the topology", () => {
    for (const circuit of [
      loadFixture("01-passive-sensor-transistor-lab.json"),
      chaser555CounterCircuit,
    ]) {
      const engine = new SimEngine();
      engine.load(circuit);
      for (const component of circuit.components) {
        for (const pin of component.pins) {
          const legacyNet = engine.nets.find((net) =>
            net.pins.some(([componentId, pinId]) =>
              componentId === component.id && pinId === pin.id,
            ),
          );
          expect(engine.getNetIdForPin(component.id, pin.id)).toBe(legacyNet?.id);
        }
      }
      expect(engine.getNetIdForPin("missing", "missing")).toBeUndefined();
    }
  });

  it("preserves the passive analog trajectory and stored reactive state", () => {
    const engine = engineFor(loadFixture("01-passive-sensor-transistor-lab.json"));
    const dynamicNet = netFor(engine, "ind_choke", "b");
    const captures = atCheckpoints(engine, 1e-4, [1, 10, 50, 200, 700], () => ({
      time: rounded(engine.simTime),
      dynamicV: rounded(engine.getNetV()[dynamicNet]),
      supplyI: rounded(engine.getElementI().vcc_5v),
      potLedI: rounded(engine.getElementI().led_pot_dim),
      capV: rounded(engine.saveState().caps.get("cap_filter")),
      inductorI: rounded(engine.saveState().inds.get("ind_choke")),
      converged: engine.lastConverged,
    }));

    expectCaptures(captures, [
      { time: 0.0001, dynamicV: 4.583333333, supplyI: -0.06399430247, potLedI: 0.00003019795352, capV: 4.166666667, inductorI: 0.04166666667, converged: true },
      { time: 0.001, dynamicV: 4.99999221, supplyI: -0.02232700984, potLedI: 0.00003019795352, capV: 4.999998469, inductorI: -6.259614138e-7, converged: true },
      { time: 0.005, dynamicV: 5, supplyI: -0.02232763583, potLedI: 0.00003019795352, capV: 5, inductorI: 9.999911752e-12, converged: true },
      { time: 0.02, dynamicV: 5, supplyI: -0.02232763591, potLedI: 0.00003019795352, capV: 5, inductorI: 9.999822935e-12, converged: true },
      { time: 0.07, dynamicV: 5, supplyI: -0.02232763618, potLedI: 0.00003019795352, capV: 5, inductorI: 9.999911752e-12, converged: true },
    ]);
  });

  it("preserves mixed-signal oscillator voltages and exact counter states", () => {
    const engine = engineFor(chaser555CounterCircuit);
    const clockNet = netFor(engine, "u2_74ls161", "clk");
    const timingNet = netFor(engine, "u1_ne555", "2");
    const captures = atCheckpoints(
      engine,
      1e-3,
      [1, 250, 500, 750, 1000, 1500, 2000, 2500, 3000],
      () => ({
        time: rounded(engine.simTime),
        clockV: rounded(engine.getNetV()[clockNet]),
        timingV: rounded(engine.getNetV()[timingNet]),
        count: engine.digitalState.u2_74ls161 ?? null,
        qa: engine.digitalState["u2_74ls161/qa"] ?? null,
        qb: engine.digitalState["u2_74ls161/qb"] ?? null,
        qc: engine.digitalState["u2_74ls161/qc"] ?? null,
        qd: engine.digitalState["u2_74ls161/qd"] ?? null,
        converged: engine.lastConverged,
      }),
    );

    expectCaptures(captures, [
      { time: 0.001, clockV: 4.9, timingV: 0.03378378367, count: 1, qa: 1, qb: 0, qc: 0, qd: 0, converged: true },
      { time: 0.25, clockV: 4.9, timingV: 2.047659918, count: 2, qa: 0, qb: 1, qc: 0, qd: 0, converged: true },
      { time: 0.5, clockV: 4.9, timingV: 3.256720352, count: 3, qa: 1, qb: 1, qc: 0, qd: 0, converged: true },
      { time: 0.75, clockV: 4.9, timingV: 1.689777433, count: 5, qa: 1, qb: 0, qc: 1, qd: 0, converged: true },
      { time: 1, clockV: 4.9, timingV: 3.045405645, count: 6, qa: 0, qb: 1, qc: 1, qd: 0, converged: true },
      { time: 1.5, clockV: 4.9, timingV: 2.808468191, count: 9, qa: 1, qb: 0, qc: 0, qd: 1, converged: true },
      { time: 2, clockV: 4.9, timingV: 2.542787775, count: 12, qa: 0, qb: 0, qc: 1, qd: 1, converged: true },
      { time: 2.5, clockV: 4.9, timingV: 2.244913279, count: 15, qa: 1, qb: 1, qc: 1, qd: 1, converged: true },
      { time: 3, clockV: 4.9, timingV: 1.910923261, count: 2, qa: 0, qb: 1, qc: 0, qd: 0, converged: true },
    ]);
  });
});
