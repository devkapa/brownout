import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

function openDivider(resistance: number): SimCircuit {
  return {
    components: [
      { id: "v", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
    ],
    wires: [
      { from_component: "v", from_pin: "pos", to_component: "r", to_pin: "a" },
    ],
  };
}

function openNodeVoltage(engine: SimEngine): number {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([componentId, pinId]) => componentId === "r" && pinId === "b"),
  );
  if (!net) throw new Error("open resistor node not found");
  return engine.getNetV()[net.id] ?? Number.NaN;
}

describe("high-impedance regularisation", () => {
  it("does not materially load a 100 MΩ source", () => {
    const engine = new SimEngine();
    engine.load(openDivider(100_000_000));
    engine.step(1e-4);

    // 1 TΩ rshunt gives 5 V × 1 TΩ / (1 TΩ + 100 MΩ) = 4.9995 V.
    // The former 1 GΩ shunt produced 4.545 V and visibly changed the circuit.
    expect(openNodeVoltage(engine)).toBeGreaterThan(4.999);
    expect(engine.lastConverged).toBe(true);
  });

  it("does not report conflicting ideal voltage constraints as converged", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v5", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "v3", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
      ],
      wires: [
        { from_component: "v5", from_pin: "pos", to_component: "v3", to_pin: "pos" },
        { from_component: "v5", from_pin: "neg", to_component: "v3", to_pin: "neg" },
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(1e-4);

    expect(engine.lastConverged).toBe(false);
    expect(engine.lastMatrixSingular).toBe(true);
  });

  it("keeps the last trusted state and time when a direct step cannot converge", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v5", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "v3", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
        {
          id: "memory",
          kind: "capacitor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { capacitance: 1e-6, esr: 10 },
        },
      ],
      wires: [
        { from_component: "v5", from_pin: "pos", to_component: "v3", to_pin: "pos" },
        { from_component: "v5", from_pin: "neg", to_component: "v3", to_pin: "neg" },
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    expect(engine.lastConverged).toBe(false);

    const before = engine.saveState();
    engine.step(1e-4);
    const after = engine.saveState();

    expect(engine.lastConverged).toBe(false);
    expect(engine.lastMatrixSingular).toBe(true);
    const { solverDiagnostics: _beforeDiagnostics, ...beforeCommitted } = before;
    const { solverDiagnostics: _afterDiagnostics, ...afterCommitted } = after;
    expect(afterCommitted).toEqual(beforeCommitted);
  });

  it("clears stale solver health when the rebuilt circuit has an empty matrix", () => {
    const conflicting: SimCircuit = {
      components: [
        { id: "v5", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "v3", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
      ],
      wires: [
        { from_component: "v5", from_pin: "pos", to_component: "v3", to_pin: "pos" },
        { from_component: "v5", from_pin: "neg", to_component: "v3", to_pin: "neg" },
      ],
    };
    const engine = new SimEngine();
    engine.load(conflicting);
    engine.step(1e-4);
    expect(engine.lastConverged).toBe(false);
    expect(engine.lastMatrixSingular).toBe(true);

    engine.load({ components: [], wires: [] });

    expect(engine.lastConverged).toBe(true);
    expect(engine.lastIters).toBe(0);
    expect(engine.lastSolveUs).toBe(0);
    expect(engine.lastMatrixSize).toBe(0);
    expect(engine.lastMatrixSingular).toBe(false);
    expect(engine.lastMatrixIllConditioned).toBe(false);
    expect(engine.lastRelativeResidual).toBe(0);
  });
});
