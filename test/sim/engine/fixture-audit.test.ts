import { describe, expect, it } from "vitest";
import fs from "node:fs";
import path from "node:path";
import type { Circuit } from "../../../src/circuit/types.js";
import { chaser555CounterCircuit } from "../../helpers/embedded-fixtures.js";
import { arduinoUnoUsbBlinkCircuit } from "../../helpers/embedded-fixtures.js";
import { arduinoUnoPwmRgbLedCircuit } from "../../helpers/embedded-fixtures.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";

// B2 migration: fixtures were pre-converted offline with devolt's own
// breadboardToSimCircuit, so this suite loads the electrical *.sim.json
// directly. Param-mutation tests below are unaffected: the converter passed
// component objects through untouched, so editing params on the loaded
// circuit is exactly what editing them pre-conversion used to be.
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

function loadEngine(name: string): SimEngine {
  return engineFor(loadFixture(name));
}

function netFor(engine: SimEngine, componentId: string, pinId: string): string {
  const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === componentId && p === pinId));
  if (!net) throw new Error(`No net for ${componentId}.${pinId}`);
  return net.id;
}

function sample(engine: SimEngine, dt: number, steps: number, read: () => unknown): unknown[] {
  const values: unknown[] = [];
  for (let i = 0; i < steps; i++) {
    engine.step(dt);
    values.push(read());
  }
  return values;
}

function runWorkerBatch(engine: SimEngine, adaptiveH: number): { adaptiveH: number; hitStepCap: boolean; failedSteps: number; steps: number } {
  const maxStepsPerBatch = 160;
  let simmed = 0;
  let failedSteps = 0;
  let steps = 0;

  while (simmed < 0.016 - 1e-15) {
    if (++steps > maxStepsPerBatch) {
      return { adaptiveH, hitStepCap: true, failedSteps, steps };
    }

    const dt = Math.min(adaptiveH, 0.016 - simmed);
    const snap = engine.saveState();
    engine.step(dt);

    if (!engine.lastConverged) {
      engine.restoreState(snap);
      failedSteps += 1;
      adaptiveH = Math.max(1e-8, dt * 0.25);
      if (dt <= 1.01e-8) simmed += dt;
      continue;
    }

    if (engine.lastIters > 15) adaptiveH = Math.max(1e-8, adaptiveH * 0.7);
    else if (engine.lastIters <= 3) adaptiveH = Math.min(1e-2, adaptiveH * 1.5);
    simmed += dt;
  }

  return { adaptiveH, hitStepCap: false, failedSteps, steps };
}

function percentile(samples: number[], p: number): number {
  const sorted = [...samples].sort((a, b) => a - b);
  return sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * p))] ?? 0;
}

// Per-step interactive budget, in microseconds. Enforced against CPU time
// (see solveStats) so it stays meaningful without flaking under contention.
const PERF_BUDGET_US = 5000;

// Number of measurement passes for the best-of-N statistic below.
const PERF_PASSES = 3;

// Wall-clock per-step time only reflects real interactive latency on an
// otherwise-idle machine. Inside the parallel vitest pool (~100 files,
// CPU oversubscribed) it measures scheduling contention, not solver cost,
// which is what made this assertion flaky. Enforce wall-clock only in a
// controlled, single-threaded perf lane via this opt-in flag; CI enforces
// the contention-immune CPU-time budget below.
const ASSERT_WALL_CLOCK_PERF =
  process.env.DEVOLT_ASSERT_WALL_PERF === "1";

function measurePass(
  engine: SimEngine,
  dt: number,
  steps: number,
): { p95SolveUs: number; p95StepCpuUs: number } {
  const stepCpuSamples: number[] = [];
  const solveSamples: number[] = [];
  for (let i = 0; i < steps; i++) {
    const cpuStart = process.cpuUsage();
    engine.step(dt);
    const cpu = process.cpuUsage(cpuStart);
    stepCpuSamples.push(cpu.user + cpu.system);
    solveSamples.push(engine.lastSolveUs);
  }
  return {
    p95SolveUs: percentile(solveSamples, 0.95),
    p95StepCpuUs: percentile(stepCpuSamples, 0.95),
  };
}

function solveStats(engine: SimEngine, dt = 1e-4, steps = 50, passes = PERF_PASSES): {
  p95SolveUs: number;
  p95StepCpuUs: number;
  matrixSize: number;
} {
  // Warm up JIT + caches so the first noisy pass does not pollute the budget.
  for (let i = 0; i < 16; i++) engine.step(dt);

  // Best-of-N (min across passes). Scheduling jitter, GC pauses, and
  // CPU-frequency dips under load only ever *inflate* a measurement, so the
  // minimum recovers the true cost. A genuine solver regression raises the
  // floor and fails every pass, so best-of-N stays strict against regressions
  // while shedding the transient contention that caused the flake.
  let p95SolveUs = Infinity;
  let p95StepCpuUs = Infinity;
  for (let pass = 0; pass < passes; pass++) {
    const r = measurePass(engine, dt, steps);
    p95SolveUs = Math.min(p95SolveUs, r.p95SolveUs);
    p95StepCpuUs = Math.min(p95StepCpuUs, r.p95StepCpuUs);
  }
  return { p95SolveUs, p95StepCpuUs, matrixSize: engine.lastMatrixSize };
}

describe("breadboard fixture audit", () => {
  it("keeps the passive fixture finite while the inductor/capacitor node settles", () => {
    const engine = loadEngine("01-passive-sensor-transistor-lab.json");
    const indCapNet = netFor(engine, "ind_choke", "b");
    const values = sample(engine, 1e-4, 200, () => engine.getNetV()[indCapNet]) as number[];

    expect(values.every(Number.isFinite)).toBe(true);
    expect(Math.max(...values.map((v) => Math.abs(v)))).toBeLessThan(6);

    sample(engine, 1e-4, 500, () => null);
    expect(Math.abs(engine.getElementI().vcc_5v)).toBeLessThan(0.2);
  });

  it("keeps the passive fixture bounded without persistent step-cap pressure", () => {
    const engine = loadEngine("01-passive-sensor-transistor-lab.json");
    let hitStepCapBatches = 0;
    const cappedBatches: number[] = [];
    let maxSupplyCurrent = 0;

    for (let batch = 0; batch < 20; batch++) {
      let steps = 0;
      let failedSteps = 0;
      let simmed = 0;
      let h = 1e-4;
      while (simmed < 0.016 - 1e-15) {
        if (++steps > 500) {
          hitStepCapBatches += 1;
          cappedBatches.push(batch);
          engine.step(0.016 - simmed);
          break;
        }
        const dt = Math.min(h, 0.016 - simmed);
        const snap = engine.saveState();
        engine.step(dt);
        if (!engine.lastConverged) {
          engine.restoreState(snap);
          failedSteps += 1;
          h = Math.max(1e-8, dt * 0.25);
          if (dt <= 1.01e-8) simmed += dt;
          continue;
        }
        if (engine.lastIters > 15) h = Math.max(1e-8, h * 0.7);
        else if (engine.lastIters <= 3) h = Math.min(1e-2, h * 1.5);
        simmed += dt;
      }

      for (const value of Object.values(engine.getNetV())) {
        expect(Number.isFinite(value)).toBe(true);
      }
      maxSupplyCurrent = Math.max(maxSupplyCurrent, Math.abs(engine.getElementI().vcc_5v ?? 0));
    }

    expect(hitStepCapBatches).toBeLessThanOrEqual(5);
    expect(cappedBatches.every((batch) => batch < 10)).toBe(true);
    expect(maxSupplyCurrent).toBeLessThan(0.2);
  });

  it("does not let fixture 01's open-base PNP hold the live worker in red status", () => {
    const engine = loadEngine("01-passive-sensor-transistor-lab.json");
    let adaptiveH = 1e-4;
    const cappedBatches: number[] = [];
    let failedSteps = 0;

    for (let batch = 0; batch < 20; batch++) {
      const result = runWorkerBatch(engine, adaptiveH);
      adaptiveH = result.adaptiveH;
      failedSteps += result.failedSteps;
      if (result.hitStepCap) cappedBatches.push(batch);
    }

    expect(cappedBatches).toEqual([]);
    expect(failedSteps).toBe(0);
    expect(engine.simTime).toBeGreaterThan(0.3);
    expect(Math.abs(engine.getElementI().q_pnp ?? 0)).toBeLessThan(1e-9);
  });

  it("routes the passive fixture SPDT through one selected LED branch at a time", () => {
    const fixture = loadFixture("01-passive-sensor-transistor-lab.json");
    const sw = fixture.components.find((c) => c.id === "sw_select");
    if (!sw) throw new Error("missing sw_select");

    sw.params = { ...sw.params, position: 0 };
    const pos0 = engineFor(fixture);
    sample(pos0, 1e-4, 40, () => null);
    expect(pos0.getElementI().led_green_sel).toBeGreaterThan(0.001);
    expect(Math.abs(pos0.getElementI().led_blue_sel ?? 0)).toBeLessThan(1e-6);

    sw.params = { ...sw.params, position: 1 };
    const pos1 = engineFor(fixture);
    sample(pos1, 1e-4, 40, () => null);
    expect(pos1.getElementI().led_blue_sel).toBeGreaterThan(0.001);
    expect(Math.abs(pos1.getElementI().led_green_sel ?? 0)).toBeLessThan(1e-6);
  });

  it("dims the passive fixture potentiometer LED by changing current", () => {
    const currents = [0.15, 0.45, 0.75].map((position) => {
      const fixture = loadFixture("01-passive-sensor-transistor-lab.json");
      const pot = fixture.components.find((c) => c.id === "pot_divider");
      if (!pot) throw new Error("missing pot_divider");
      pot.params = { ...pot.params, position };
      const engine = engineFor(fixture);
      sample(engine, 1e-4, 80, () => null);
      return engine.getElementI().led_pot_dim;
    });

    expect(currents.every(Number.isFinite)).toBe(true);
    expect(currents[0]).toBeGreaterThan(currents[1]);
    expect(currents[1]).toBeGreaterThan(currents[2]);
    expect(currents[0]).toBeLessThan(0.03);
    expect(currents[2]).toBeLessThan(0.001);
  });

  it("advances the NE555/counter fixture", () => {
    const engine = loadEngine("03-ne555-74ls161-counter-7seg.json");
    const outNet = netFor(engine, "u1_ne555", "3");
    const states = sample(engine, 1e-3, 2500, () => ({
      out: engine.getNetV()[outNet],
      cap: engine.getNetV()[netFor(engine, "c_555_timing", "a")],
      dis: engine.getNetV()[netFor(engine, "u1_ne555", "7")],
      count: engine.digitalState.u2_74ls161,
    })) as Array<{ out: number; count: number | undefined }>;

    expect(new Set(states.map((s) => Math.round(s.out))).size).toBeGreaterThan(1);
    expect(new Set(states.map((s) => s.count)).size).toBeGreaterThan(1);
  });

  it("advances the embedded NE555/counter hero fixture", () => {
    const engine = engineFor(chaser555CounterCircuit);
    const clkNet = netFor(engine, "u2_74ls161", "clk");
    const states = sample(engine, 1e-3, 3000, () => ({
      clk: engine.getNetV()[clkNet],
      qa: engine.digitalState["u2_74ls161/qa"],
      qb: engine.digitalState["u2_74ls161/qb"],
      count: engine.digitalState.u2_74ls161,
    })) as Array<{ clk: number; qa: number | undefined; qb: number | undefined; count: number | undefined }>;

    expect(new Set(states.map((s) => Math.round(s.clk))).size).toBeGreaterThan(1);
    expect(new Set(states.map((s) => s.count)).size).toBeGreaterThan(1);
    expect(
      new Set(states.map((s) => `${s.qa ?? 0}${s.qb ?? 0}`)).size,
    ).toBeGreaterThan(1);
  });

  it("keeps advancing the shift-register fixture latch state", () => {
    const engine = loadEngine("04-74hc595-shift-7seg.json");
    const states = sample(engine, 1e-3, 2500, () =>
      ["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh"]
        .map((pin) => engine.digitalState[`u1_74hc595/${pin}`] ?? 0)
        .join(""),
    ) as string[];

    expect(new Set(states).size).toBeGreaterThan(3);
    expect(new Set(states.slice(0, 1000)).size).toBeGreaterThan(1);
    expect(new Set(states.slice(1000)).size).toBeGreaterThan(1);
  });

  it("loads observable datapath and memory outputs", () => {
    const adder = loadEngine("05-register-bus-adder.json");
    sample(adder, 1e-3, 1000, () => null);
    expect(adder.digitalState["u4_74ls283/s1"]).toBe(0);
    expect(adder.digitalState["u4_74ls283/s2"]).toBe(1);
    expect(adder.digitalState["u4_74ls283/s3"]).toBe(1);
    expect(adder.digitalState["u4_74ls283/s4"]).toBe(1);

    const memory = loadEngine("06-memory-rom-lab.json");
    sample(memory, 1e-3, 50, () => null);
    expect(memory.digitalState["u1_74ls189//o1"]).toBeDefined();
    expect(memory.digitalState["u2_28c16/io0"]).toBeDefined();
    expect(memory.digitalState["u3_28c256/io0"]).toBeDefined();
  });

  it("keeps dense MNA solve time within the S11 interactive budget", () => {
    const fixtures = [
      ["chaser-555-counter", engineFor(chaser555CounterCircuit), 1e-4],
      ["03-ne555-74ls161-counter-7seg", loadEngine("03-ne555-74ls161-counter-7seg.json"), 1e-4],
      ["04-74hc595-shift-7seg", loadEngine("04-74hc595-shift-7seg.json"), 1e-4],
      ["05-register-bus-adder", loadEngine("05-register-bus-adder.json"), 1e-4],
      ["06-memory-rom-lab", loadEngine("06-memory-rom-lab.json"), 1e-4],
      ["07-74hc165-parallel-serial-io", loadEngine("07-74hc165-parallel-serial-io.json"), 1e-4],
      ["arduino-uno-usb-blink", engineFor(arduinoUnoUsbBlinkCircuit), 1e-4],
      ["arduino-uno-pwm-rgb-led", engineFor(arduinoUnoPwmRgbLedCircuit), 1e-4],
    ] as const;

    const results = fixtures.map(([name, engine, dt]) => ({ name, ...solveStats(engine, dt, 50) }));
    for (const result of results) {
      expect(result.matrixSize, result.name).toBeGreaterThan(0);
      // CPU time is per-process inside vitest's forked pool, so it counts only
      // this test's solver work regardless of how many sibling files contend
      // for cores. This is the CI-enforced, contention-immune budget.
      expect(result.p95StepCpuUs, result.name).toBeLessThan(PERF_BUDGET_US);
      if (ASSERT_WALL_CLOCK_PERF) {
        expect(result.p95SolveUs, result.name).toBeLessThan(PERF_BUDGET_US);
      }
    }
  });
});
