/**
 * AnalysisEngine (W2e): the one abstraction the transient analyses step
 * through, over classic's checked step or one reused CoreEngine.
 *
 * What is pinned here and why:
 *  - the engine CHOICE (D14): made once per run, after every point's circuit
 *    has been compiled, with the reasons a fallback carries;
 *  - classic through the abstraction is bit-identical to stepping SimEngine
 *    by hand (the analyses' classic path must not move);
 *  - the core is advanced to each grid point (D15), is cold per coldLoad even
 *    though one CoreEngine is reused, and matches the analytic RC answer.
 */

import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { initCore } from "../../src/core/wasm.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import {
  AnalysisCoreRejectedError,
  planAnalysisEngine,
  runOnPlan,
  runOnPlanAsync,
  stepToGrid,
} from "../../src/analysis/engine.js";
import { AnalysisConvergenceError } from "../../src/analysis/checked-step.js";
import { ANALYSIS_STEP_S } from "../../src/analysis/run-sweep.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import { ne555AstablePeriod } from "../core/equivalence/analytic.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

await initCore();

const SUPPORTED = { coreSupportedKinds: CORE_IMPLEMENTED_KINDS } as const;

function rcCircuit(R = 1000, C = 1e-6, V = 5): SimCircuit {
  return {
    components: [
      { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: V } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R } },
      { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: C } },
    ],
    wires: [
      { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "a" },
      { from_component: "c1", from_pin: "b", to_component: "v1", to_pin: "neg" },
    ],
  };
}

function conflictingSourcesCircuit(): SimCircuit {
  return {
    components: [
      { id: "v5", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "v3", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
      { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
    ],
    wires: [
      { from_component: "v5", from_pin: "pos", to_component: "v3", to_pin: "pos" },
      { from_component: "v5", from_pin: "neg", to_component: "v3", to_pin: "neg" },
      { from_component: "v5", from_pin: "pos", to_component: "load", to_pin: "a" },
      { from_component: "load", from_pin: "b", to_component: "v5", to_pin: "neg" },
    ],
  };
}

/** A signal_gen is not a core kind: the unsupported-kind probe. */
function signalGenCircuit(): SimCircuit {
  return {
    components: [
      { id: "sg", kind: "signal_gen", pins: [{ id: "pos" }, { id: "neg" }], params: { waveform: "dc", offset: 1 } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
    ],
    wires: [
      { from_component: "sg", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "sg", to_pin: "neg" },
    ],
  };
}

function capNet(engine: { nets: { id: string; pins: [string, string][] }[] }): string {
  const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === "c1" && p === "a"));
  if (!net) throw new Error("no c1.a net");
  return net.id;
}

describe("planAnalysisEngine: one choice per run, made after compiling every point (D14)", () => {
  it("auto on the shipped list (CORE_SUPPORTED_KINDS) is the core for a circuit of promoted kinds", () => {
    const plan = planAnalysisEngine([rcCircuit()], { engine: "auto" });
    expect(plan.engine).toBe("core");
    expect(plan.fallbackReasons).toEqual([]);
  });

  it("auto is classic for an implemented kind the list in force leaves out, naming it", () => {
    // What a kind implemented by a later wave gets until it is promoted: it compiles and runs on the core
    // (CORE_IMPLEMENTED_KINDS) but is not on the list `auto` reads. Shown here by leaving spdt_switch out.
    const circuit = rcCircuit();
    circuit.components.push({ id: "sw", kind: "spdt_switch", pins: [{ id: "com" }, { id: "b" }, { id: "c" }], params: { position: 0 } });
    const withoutSpdt = new Set([...CORE_IMPLEMENTED_KINDS].filter((kind) => kind !== "spdt_switch"));
    const plan = planAnalysisEngine([circuit], { engine: "auto", coreSupportedKinds: withoutSpdt });
    expect(plan.engine).toBe("classic");
    expect(plan.fallbackReasons).toEqual(["unsupported-kind:spdt_switch"]);
  });

  it("auto on an empty list is classic, with each kind as a reason", () => {
    const plan = planAnalysisEngine([rcCircuit()], { engine: "auto", coreSupportedKinds: new Set() });
    expect(plan.engine).toBe("classic");
    expect(plan.fallbackReasons).toEqual([
      "unsupported-kind:capacitor",
      "unsupported-kind:resistor",
      "unsupported-kind:voltage_source",
    ]);
  });

  it("auto with the gate widened, on a supported circuit, is the core with no reasons", () => {
    const plan = planAnalysisEngine([rcCircuit()], { engine: "auto", ...SUPPORTED });
    expect(plan.engine).toBe("core");
    expect(plan.fallbackReasons).toEqual([]);
  });

  it("an unset engine is auto", () => {
    expect(planAnalysisEngine([rcCircuit()], { ...SUPPORTED }).engine).toBe("core");
  });

  it("auto on a circuit with a kind the core lacks falls back and names it", () => {
    const plan = planAnalysisEngine([signalGenCircuit()], { engine: "auto", ...SUPPORTED });
    expect(plan.engine).toBe("classic");
    expect(plan.fallbackReasons).toEqual(["unsupported-kind:signal_gen"]);
  });

  it("classic is classic with no reasons: nothing fell back", () => {
    const plan = planAnalysisEngine([rcCircuit()], { engine: "classic", ...SUPPORTED });
    expect(plan.engine).toBe("classic");
    expect(plan.fallbackReasons).toEqual([]);
  });

  it("core on an unsupported circuit throws, naming the reasons", () => {
    expect(() => planAnalysisEngine([signalGenCircuit()], { engine: "core", ...SUPPORTED })).toThrow(
      /unsupported-kind:signal_gen/,
    );
  });

  it("core on a supported circuit is the core", () => {
    expect(planAnalysisEngine([rcCircuit()], { engine: "core", ...SUPPORTED }).engine).toBe("core");
  });

  it("ONE bad point sends the WHOLE run to classic: every point is compiled before the choice", () => {
    const good = rcCircuit();
    const bad = rcCircuit();
    bad.components[1]!.params.resistance = Number.NaN;
    const plan = planAnalysisEngine([good, good, bad, good], { engine: "auto", ...SUPPORTED });
    expect(plan.engine).toBe("classic");
    expect(plan.fallbackReasons).toEqual(["malformed:r1:non-finite-resistance"]);
  });

  it("forced-classic reasons (a switch-over later is deleting them) win on auto and are recorded", () => {
    const plan = planAnalysisEngine([rcCircuit()], {
      engine: "auto",
      ...SUPPORTED,
      forcedClassicReasons: ["tran-needs-carry-seed"],
    });
    expect(plan.engine).toBe("classic");
    expect(plan.fallbackReasons).toEqual(["tran-needs-carry-seed"]);
  });

  it("forced-classic reasons make an explicit core request an error, and an explicit classic request silent", () => {
    const forced = { ...SUPPORTED, forcedClassicReasons: ["tran-needs-carry-seed"] };
    expect(() => planAnalysisEngine([rcCircuit()], { engine: "core", ...forced })).toThrow(/tran-needs-carry-seed/);
    expect(planAnalysisEngine([rcCircuit()], { engine: "classic", ...forced }).fallbackReasons).toEqual([]);
  });
});

describe("classic through the abstraction is bit-identical to stepping SimEngine", () => {
  it("a 5 ms RC charge on the old hand-written loop's grid: every node voltage is Object.is-equal", () => {
    const circuit = rcCircuit();
    const raw = new SimEngine();
    raw.coldLoad(structuredClone(circuit));
    // The loop run-sweep.ts used to carry, verbatim.
    let simTime = raw.simTime;
    while (simTime < 5e-3) {
      raw.step(Math.min(ANALYSIS_STEP_S, 5e-3 - simTime));
      simTime = raw.simTime;
    }

    const plan = planAnalysisEngine([circuit], { engine: "classic" });
    const engine = plan.open();
    engine.coldLoad(structuredClone(circuit));
    stepToGrid(engine, 5e-3, ANALYSIS_STEP_S, "test");
    const a = raw.getNetV();
    const b = engine.getNetV();
    expect(Object.keys(b).sort()).toEqual(Object.keys(a).sort());
    for (const key of Object.keys(a)) expect(Object.is(b[key], a[key])).toBe(true);
    expect(Object.is(engine.simTime, raw.simTime)).toBe(true);
    engine.dispose();
  });
});

describe("the core engine: coldLoad, advanceTo, readouts", () => {
  it("the RC charge on the grid matches 5(1 - exp(-t/RC)) at every grid point", () => {
    const circuit = rcCircuit(1000, 1e-6, 5);
    const plan = planAnalysisEngine([circuit], { engine: "core", ...SUPPORTED });
    const engine = plan.open();
    engine.coldLoad(structuredClone(circuit));
    const net = capNet(engine);
    let worst = 0;
    stepToGrid(engine, 5e-3, ANALYSIS_STEP_S, "test", (t) => {
      const analytic = 5 * (1 - Math.exp(-t / 1e-3));
      worst = Math.max(worst, Math.abs(engine.getNetV()[net]! - analytic));
    });
    // W0 band at 5 V is 100 uV + 1e-3 rel; the closed-form RC is far inside it.
    expect(worst).toBeLessThan(1e-4);
    expect(engine.simTime).toBeCloseTo(5e-3, 12);
    engine.dispose();
  });

  it("a reused engine is cold after coldLoad: simTime 0 and an empty capacitor", () => {
    const circuit = rcCircuit();
    const engine = planAnalysisEngine([circuit], { engine: "core", ...SUPPORTED }).open();
    engine.coldLoad(structuredClone(circuit));
    stepToGrid(engine, 2e-3, ANALYSIS_STEP_S, "test");
    expect(engine.getNetV()[capNet(engine)]).toBeGreaterThan(3);

    engine.coldLoad(structuredClone(circuit));
    expect(engine.simTime).toBe(0);
    // Charge carried across the reload would read as volts here.
    expect(Math.abs(engine.getNetV()[capNet(engine)]!)).toBeLessThan(1e-3);
    stepToGrid(engine, 2e-3, ANALYSIS_STEP_S, "test");
    const second = engine.getNetV()[capNet(engine)]!;
    engine.coldLoad(structuredClone(circuit));
    stepToGrid(engine, 2e-3, ANALYSIS_STEP_S, "test");
    expect(engine.getNetV()[capNet(engine)]).toBe(second);
    engine.dispose();
  });

  it("advanceTo lands on the target exactly", () => {
    const engine = planAnalysisEngine([rcCircuit()], { engine: "core", ...SUPPORTED }).open();
    engine.coldLoad(rcCircuit());
    engine.advanceTo(1.234e-3);
    expect(engine.simTime).toBe(1.234e-3);
    engine.dispose();
  });

  it("getFailures reports the loaded circuit's failures with classic's shape", () => {
    const engine = planAnalysisEngine([rcCircuit()], { engine: "core", ...SUPPORTED }).open();
    engine.coldLoad(rcCircuit());
    engine.advanceTo(1e-3);
    expect(engine.getFailures()).toEqual({});
    engine.dispose();
  });

  it("a core rejection at coldLoad (a topology the core declines) is an AnalysisCoreRejectedError with the reasons", () => {
    // Planned on a loadable first point (a plan on the declined circuit itself is an Error now).
    const engine = planAnalysisEngine([rcCircuit()], { engine: "core", ...SUPPORTED }).open();
    let caught: unknown;
    try {
      engine.coldLoad(conflictingSourcesCircuit());
    } catch (err) {
      caught = err;
    }
    expect(caught).toBeInstanceOf(AnalysisCoreRejectedError);
    expect((caught as AnalysisCoreRejectedError).reasons.some((r) => r.startsWith("topology:"))).toBe(true);
    engine.dispose();
  });
});

describe("classic advanceTo rejects a non-converged step as classic always did", () => {
  it("throws AnalysisConvergenceError and leaves the engine at the last good instant", () => {
    const engine = planAnalysisEngine([conflictingSourcesCircuit()], { engine: "classic" }).open();
    engine.coldLoad(conflictingSourcesCircuit());
    expect(() => engine.advanceTo(ANALYSIS_STEP_S, { stepS: ANALYSIS_STEP_S, context: "test" })).toThrow(
      AnalysisConvergenceError,
    );
    expect(engine.simTime).toBe(0);
    engine.dispose();
  });
});

describe("planAnalysisEngine loads the first point on the core, so the common rejection is caught before any point runs", () => {
  it("auto: a topology the Rust load declines is classic with the topology reason", () => {
    const plan = planAnalysisEngine([conflictingSourcesCircuit()], { engine: "auto", ...SUPPORTED });
    expect(plan.engine).toBe("classic");
    expect(plan.fallbackReasons.some((r) => r.startsWith("topology:"))).toBe(true);
  });

  it("core: the same decline is an Error naming it, before any engine exists", () => {
    expect(() => planAnalysisEngine([conflictingSourcesCircuit()], { engine: "core", ...SUPPORTED })).toThrow(/topology:/);
  });
});

describe("runOnPlan: a later point's core rejection reruns the WHOLE run on classic (D14: one engine per run)", () => {
  // Point 0 loads on the core; point 1 is declined by the Rust load, which only a
  // real load can see: this is what the planner's first-point load cannot catch.
  const points = (): SimCircuit[] => [rcCircuit(), conflictingSourcesCircuit()];
  const loadAll = (engine: { coldLoad(c: SimCircuit): void }): void => {
    for (const circuit of points()) engine.coldLoad(structuredClone(circuit));
  };

  it("auto: onRestart fires exactly once, with the reasons, before the replay; the body sees the final plan", () => {
    const plan = planAnalysisEngine(points(), { engine: "auto", ...SUPPORTED });
    expect(plan.engine).toBe("core");
    const events: string[] = [];
    let restartReasons: readonly string[] = [];
    const result = runOnPlan(
      plan,
      (engine, report) => {
        events.push(`attempt:${report.engine}:${engine.kind}`);
        // Stands in for a runner's onPoint: a replay must not look like a continuation.
        engine.coldLoad(structuredClone(rcCircuit()));
        events.push("point");
        loadAll(engine);
        return report;
      },
      (reasons) => {
        restartReasons = reasons;
        events.push("restart");
      },
    );
    expect(events.filter((e) => e === "restart")).toHaveLength(1);
    expect(events.indexOf("restart")).toBeGreaterThan(events.indexOf("attempt:core:core"));
    expect(events.indexOf("restart")).toBeLessThan(events.indexOf("attempt:classic:classic"));
    // No point of the replay is announced before the restart.
    expect(events.slice(0, events.indexOf("restart")).filter((e) => e === "attempt:classic:classic")).toHaveLength(0);
    expect(restartReasons.some((r) => r.startsWith("topology:"))).toBe(true);
    expect(result.engine).toBe("classic");
    expect(result.fallbackReasons).toEqual([...restartReasons].sort());
  });

  it("async: the same, once", async () => {
    const plan = planAnalysisEngine(points(), { engine: "auto", ...SUPPORTED });
    const restarts: (readonly string[])[] = [];
    const result = await runOnPlanAsync(
      plan,
      async (engine, report) => {
        loadAll(engine);
        return report;
      },
      (reasons) => restarts.push(reasons),
    );
    expect(restarts).toHaveLength(1);
    expect(result.engine).toBe("classic");
  });

  it("onRestart is not called when the run never restarts", () => {
    const plan = planAnalysisEngine([rcCircuit()], { engine: "auto", ...SUPPORTED });
    let called = 0;
    runOnPlan(plan, (engine) => engine.coldLoad(rcCircuit()), () => called++);
    expect(called).toBe(0);
  });

  it("core: the same rejection is an error, not a silent fallback, and onRestart is not called", () => {
    const plan = planAnalysisEngine(points(), { engine: "core", ...SUPPORTED });
    let called = 0;
    expect(() => runOnPlan(plan, (engine) => loadAll(engine), () => called++)).toThrow(/topology:/);
    expect(called).toBe(0);
  });
});

describe("the 555 astable period on the analysis grid, against the closed form", () => {
  // The W1e astable fixture: R1 = 1 kohm, R2 = 1 kohm + 1 mohm, C = 1 uF, 5 V.
  const fixture = JSON.parse(
    readFileSync(new URL("../fixtures/core/31-ne555-astable-led.sim.json", import.meta.url), "utf8"),
  ) as SimCircuit;
  const analytic = ne555AstablePeriod({
    r1Ohms: 1000,
    r2Ohms: 1000.001,
    capacitanceF: 1e-6,
    supplyVoltsAtGnd: 5,
  }).periodS;

  /** Mean spacing of the capacitor's upward 2.5 V crossings (linear interpolation inside a grid step) over 50 ms. */
  function measuredPeriod(choice: "classic" | "core"): number {
    const plan = planAnalysisEngine([fixture], { engine: choice, ...SUPPORTED });
    const engine = plan.open();
    engine.coldLoad(structuredClone(fixture));
    const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === "ct" && p === "a"))!.id;
    const ups: number[] = [];
    let prevV = engine.getNetV()[net]!;
    let prevT = engine.simTime;
    stepToGrid(engine, 0.05, ANALYSIS_STEP_S, "555 period", (t) => {
      const v = engine.getNetV()[net]!;
      // 2.5 V sits between the 1/3 and 2/3 thresholds, so the sawtooth passes it
      // once per cycle and the sample grid cannot step over it the way it can
      // over the peak.
      if (prevV < 2.5 && v >= 2.5) ups.push(prevT + ((t - prevT) * (2.5 - prevV)) / (v - prevV));
      prevV = v;
      prevT = t;
    });
    engine.dispose();
    const gaps = ups.slice(1).map((u, i) => u - ups[i]!);
    expect(gaps.length).toBeGreaterThan(20);
    return gaps.reduce((a, b) => a + b, 0) / gaps.length;
  }

  it("the core agrees with the analytic period to 1e-5 relative (measured 6e-7: the interpolation inside a 50 us step)", () => {
    expect(Math.abs(measuredPeriod("core") - analytic) / analytic).toBeLessThan(1e-5);
  });

  it("classic is pinned at its backward-Euler baseline, 1.7 % slow of the analytic period", () => {
    // Reviewed re-baseline: classic's 2.13703 ms against the analytic
    // 2.10165 ms (the "ran 1.5% slow" of headless-step-ceiling.test.ts, on
    // the 50 us analysis grid). Pinned to 1e-7 s so it moves only on purpose.
    const classic = measuredPeriod("classic");
    expect(classic).toBeCloseTo(2.13703e-3, 7);
    expect((classic - analytic) / analytic).toBeGreaterThan(0.015);
    expect((classic - analytic) / analytic).toBeLessThan(0.019);
  });
});
