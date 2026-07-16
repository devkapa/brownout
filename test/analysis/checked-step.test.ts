import { describe, expect, it } from "vitest";
import {
  AnalysisConvergenceError,
  stepAnalysisEngine,
} from "../../src/analysis/index.js";
import { SimEngine, type SimCircuit } from "../../src/sim/engine/sim-engine.js";

function contradictoryIdealSources(): SimCircuit {
  return {
    components: [
      {
        id: "v5",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "v3",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 3 },
      },
      {
        id: "load",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
    ],
    wires: [
      { from_component: "v5", from_pin: "pos", to_component: "v3", to_pin: "pos" },
      { from_component: "v5", from_pin: "neg", to_component: "v3", to_pin: "neg" },
      { from_component: "v5", from_pin: "pos", to_component: "load", to_pin: "a" },
      { from_component: "load", from_pin: "b", to_component: "v5", to_pin: "neg" },
    ],
  };
}

describe("stepAnalysisEngine", () => {
  it("restores the pre-step state and rejects a non-converged iterate", () => {
    const engine = new SimEngine();
    engine.coldLoad(contradictoryIdealSources());
    const beforeTime = engine.simTime;
    const beforeNetV = { ...engine.getNetV() };

    expect(() => stepAnalysisEngine(engine, 50e-6, "test point"))
      .toThrowError(AnalysisConvergenceError);

    expect(engine.lastConverged).toBe(false);
    expect(engine.simTime).toBe(beforeTime);
    expect(engine.getNetV()).toEqual(beforeNetV);
    expect(engine.getFailures()).toEqual({});
  });
});
