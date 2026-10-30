/**
 * `SolverDiagnostics` on the core (W2 design 2.5, D17 as Dev ruled): only the
 * counters whose meaning matches classic's are published, the rest are absent.
 * `coreSolverDiagnostics` turns two stat snapshots (before and after one host
 * batch) into that record. The mapping is checked on synthetic snapshots (each
 * stat index lands on exactly the field the design names), and on a real
 * `CoreEngine` for the kinds the core runs.
 */

import { beforeAll, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { CoreEngine } from "../../src/core/engine.js";
import { initCore } from "../../src/core/wasm.js";
import {
  CORE_STAT_COUNT,
  coreSolverDiagnostics,
  readCoreStats,
  type CoreStatSnapshot,
} from "../../src/core/diagnostics.js";
import type { ClassicSolverDiagnostics, CoreSolverDiagnostics, SolverDiagnostics } from "../../src/sim/messages.js";

function snapshot(values: Record<number, number> = {}): CoreStatSnapshot {
  const out = new Array<number>(CORE_STAT_COUNT).fill(0);
  for (const [index, value] of Object.entries(values)) out[Number(index)] = value;
  return out;
}

describe("the SolverDiagnostics union", () => {
  it("a classic record keeps its required fields and may omit `engine` (hosts before W2)", () => {
    const classic: ClassicSolverDiagnostics = {
      steps: 10,
      failedSteps: 0,
      hitStepCap: false,
      hitMinStep: false,
      adaptiveH: 1e-5,
      lastIters: 3,
      lastSolveUs: 12,
      batchSolveUs: 120,
      stepsPerSecond: 1e5,
      matrixSize: 4,
      cappedBatches5s: 0,
      matrixSingular: false,
      matrixIllConditioned: false,
      relativeResidual: 1e-14,
    };
    const isCore = (d: SolverDiagnostics): boolean => d.engine === "core";
    expect(isCore(classic)).toBe(false);
    expect(isCore({ ...classic, engine: "classic" })).toBe(false);
  });

  it("narrowing on `engine` reaches the classic-only fields and the core record separately", () => {
    function describeIt(d: SolverDiagnostics): string {
      if (d.engine === "core") return `core ${d.core.eventsProcessed}`;
      return `classic ${d.adaptiveH}`;
    }
    const core = coreSolverDiagnostics(snapshot(), snapshot({ 4: 7 }), { converged: true });
    expect(describeIt(core)).toBe("core 7");
  });
});

describe("coreSolverDiagnostics: the stat mapping", () => {
  it("maps each design-2.5 counter to its classic field, as a delta over the batch", () => {
    const before = snapshot({ 1: 5, 4: 100, 7: 10, 10: 20, 11: 40, 12: 3 });
    const after = snapshot({ 1: 8, 2: 6, 4: 130, 7: 25, 10: 50, 11: 100, 12: 9 });
    const d = coreSolverDiagnostics(before, after, { converged: true });
    expect(d.engine).toBe("core");
    expect(d.core.eventsProcessed).toBe(30); // stat 4, under its own name
    expect(d.electricalSolves).toBe(3 + 30); // stat 1 + stat 10
    expect(d.matrixFactorizationReuses).toBe(15); // stat 7
    expect(d.digitalEvaluations).toBe(6); // stat 12
    expect(d.matrixSize).toBe(6); // stat 2, not a delta: the largest partition now
    expect(d.core.meanNewtonItersPerSteppedStep).toBe(2); // stat 11 over stat 10: 60 / 30
    expect(d.core.advanceFailed).toBe(false);
  });

  it("publishes no classic field the core has no matching meaning for", () => {
    const d = coreSolverDiagnostics(snapshot(), snapshot({ 4: 1 }), { converged: true }) as unknown as Record<string, unknown>;
    for (const absent of [
      "adaptiveH",
      "hitStepCap",
      "lastSolveUs",
      "batchSolveUs",
      "cappedBatches5s",
      "matrixSingular",
      "matrixIllConditioned",
      "relativeResidual",
      "steps",
      "lastIters",
      "failedSteps",
      "hitMinStep",
      "rejectedSteps",
      "maxLocalErrorRatio",
      "errorControlled",
      "matrixFactorizations",
      "digitalEvaluationReuses",
      "hitWallBudget",
    ]) {
      expect(absent in d, absent).toBe(false);
    }
  });

  it("the mean Newton count is absent, not zero, when no stepped step ran in the batch", () => {
    const d = coreSolverDiagnostics(snapshot({ 11: 4 }), snapshot({ 11: 9 }), { converged: true });
    expect("meanNewtonItersPerSteppedStep" in d.core).toBe(false);
  });

  it("a failed advance is `core.advanceFailed`, not a classic failed or minimum step", () => {
    const d = coreSolverDiagnostics(snapshot(), snapshot({ 4: 2 }), { converged: false });
    expect(d.core.advanceFailed).toBe(true);
    expect(d.core.eventsProcessed).toBe(2);
  });

  it("host-measured fields ride along when given", () => {
    const d = coreSolverDiagnostics(snapshot(), snapshot(), {
      converged: true,
      batchWallUs: 1500,
      batchCompletion: 0.5,
      realtimeRatio: 2,
    });
    expect(d.batchWallUs).toBe(1500);
    expect(d.batchCompletion).toBe(0.5);
    expect(d.realtimeRatio).toBe(2);
    const bare = coreSolverDiagnostics(snapshot(), snapshot(), { converged: true });
    expect("batchWallUs" in bare).toBe(false);
  });

  it("a counter that went backwards (a reload between the snapshots) reads as the new count", () => {
    const d = coreSolverDiagnostics(snapshot({ 4: 500, 12: 50 }), snapshot({ 4: 20, 12: 5 }), { converged: true });
    expect(d.core.eventsProcessed).toBe(20);
    expect(d.digitalEvaluations).toBe(5);
  });

  it("carries the core-only counters in `core`, with the max delta depth as a level", () => {
    const before = snapshot({ 5: 10, 6: 2, 8: 4, 9: 1, 13: 7, 14: 3 });
    const after = snapshot({ 5: 16, 6: 5, 8: 9, 9: 4, 13: 17, 14: 8 });
    const d: CoreSolverDiagnostics = coreSolverDiagnostics(before, after, { converged: true });
    expect(d.core).toMatchObject({
      deltaPasses: 6,
      maxDeltaDepth: 5,
      closedFormSegments: 5,
      crossings: 3,
      driverCommits: 10,
      railMoves: 5,
    });
  });
});

describe("readCoreStats on a real CoreEngine", () => {
  beforeAll(async () => {
    await initCore();
  });

  /** A 1 kHz clock into an RC: the clock's edges are events, so the batch does work. */
  const circuit: SimCircuit = {
    components: [
      { id: "v", kind: "clock", pins: [{ id: "out" }, { id: "gnd" }], params: { frequency: 1000 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
    ],
    wires: [
      { from_component: "v", from_pin: "out", to_component: "r", to_pin: "a" },
      { from_component: "r", from_pin: "b", to_component: "c", to_pin: "a" },
      { from_component: "c", from_pin: "b", to_component: "v", to_pin: "gnd" },
    ],
  };

  it("a quiet DC circuit does no counted work, and says so", () => {
    const dc: SimCircuit = {
      components: [
        { id: "v", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      ],
      wires: [
        { from_component: "v", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "v", to_pin: "neg" },
      ],
    };
    const core = new CoreEngine();
    core.load(dc);
    const before = readCoreStats(core);
    core.advance(1e-3);
    const d = coreSolverDiagnostics(before, readCoreStats(core), { converged: true });
    expect(d.core.eventsProcessed).toBe(0);
    expect(d.electricalSolves).toBe(0);
    expect(d.digitalEvaluations).toBe(0);
  });

  it("reads every stat index once and a batch of work shows up in the mapped fields", () => {
    const core = new CoreEngine();
    core.load(circuit);
    const before = readCoreStats(core);
    expect(before).toHaveLength(CORE_STAT_COUNT);
    expect(before.every((value) => Number.isFinite(value))).toBe(true);
    core.advance(5e-3);
    const after = readCoreStats(core);
    const d = coreSolverDiagnostics(before, after, { converged: core.lastConverged });
    expect(d.engine).toBe("core");
    expect(d.core.eventsProcessed).toBe(10); // the clock's ten edges in 5 ms
    expect(d.core.sourceEdges).toBe(10);
    expect(d.core.closedFormSegments).toBe(10);
    expect(d.matrixSize).toBeGreaterThan(0);
    expect(d.core.advanceFailed).toBe(false);
  });

  it("two reads with no work between them differ by nothing", () => {
    const core = new CoreEngine();
    core.load(circuit);
    core.advance(1e-3);
    const a = readCoreStats(core);
    const b = readCoreStats(core);
    expect(b).toEqual(a);
    const d = coreSolverDiagnostics(a, b, { converged: true });
    expect(d.core.eventsProcessed).toBe(0);
    expect(d.electricalSolves).toBe(0);
  });
});
