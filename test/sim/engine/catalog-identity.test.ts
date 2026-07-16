import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

function ledCircuit(catalogUid: "led-red" | "led-blue"): SimCircuit {
  return {
    components: [
      {
        id: "source",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "limit",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 470 },
      },
      {
        id: "indicator",
        kind: "led",
        catalogUid,
        pins: [{ id: "a" }, { id: "k" }],
        // Intentionally omit vf: exact catalog identity supplies the model
        // default and proves the engine is not using first-per-kind lookup.
        params: {},
      },
    ],
    wires: [
      { from_component: "source", from_pin: "pos", to_component: "limit", to_pin: "a" },
      { from_component: "limit", from_pin: "b", to_component: "indicator", to_pin: "a" },
      { from_component: "indicator", from_pin: "k", to_component: "source", to_pin: "neg" },
    ],
  };
}

function solve(circuit: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.coldLoad(circuit);
  for (let index = 0; index < 12; index++) engine.step(1e-5);
  expect(engine.lastConverged).toBe(true);
  return engine;
}

function voltageAcross(engine: SimEngine, componentId: string): number {
  const componentNets = engine.nets.filter((net) =>
    net.pins.some(([id]) => id === componentId),
  );
  const anode = componentNets.find((net) =>
    net.pins.some(([id, pin]) => id === componentId && pin === "a"),
  )!;
  const cathode = componentNets.find((net) =>
    net.pins.some(([id, pin]) => id === componentId && pin === "k"),
  )!;
  return (engine.getNetV()[anode.id] ?? Number.NaN)
    - (engine.getNetV()[cathode.id] ?? Number.NaN);
}

function regulatorIdleCurrent(catalogUid: string): number {
  const circuit: SimCircuit = {
    components: [
      {
        id: "source",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 9 },
      },
      {
        id: "reg",
        kind: "linear_reg",
        catalogUid,
        pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
        params: { vout: 5, vdropout: 2, iLimit: 1 },
      },
    ],
    wires: [
      { from_component: "source", from_pin: "pos", to_component: "reg", to_pin: "in" },
      { from_component: "source", from_pin: "neg", to_component: "reg", to_pin: "gnd" },
    ],
  };
  const engine = solve(circuit);
  return Math.abs(engine.getElementI().source ?? Number.NaN);
}

describe("engine catalog model identity", () => {
  it("keeps same-kind red and blue LED forward-voltage models distinct", () => {
    const red = solve(ledCircuit("led-red"));
    const blue = solve(ledCircuit("led-blue"));
    const redVf = voltageAcross(red, "indicator");
    const blueVf = voltageAcross(blue, "indicator");

    expect(redVf).toBeGreaterThan(1.6);
    expect(redVf).toBeLessThan(2.1);
    expect(blueVf).toBeGreaterThan(2.7);
    expect(blueVf).toBeLessThan(3.3);
    expect(blueVf - redVf).toBeGreaterThan(0.8);
  });

  it("does not borrow package quiescent current from a kind-mismatched UID", () => {
    expect(regulatorIdleCurrent("reg-7805")).toBeCloseTo(0.0065, 6);
    expect(regulatorIdleCurrent("lm317")).toBeLessThan(1e-9);
  });
});
