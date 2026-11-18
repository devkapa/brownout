/**
 * current_source polarity through `load` and `step`, the transient path every engine shares.
 *
 * current-source-polarity.test.ts locks the same facts through `dcOperatingPoint` and the small-signal analysis,
 * neither of which the Rust core has (W6), so its cases cannot run there. These are their twins on the transient
 * path (W1h H1b), so gate 1 exercises the kind on the core: the programmed current leaves `pos`, flows through the
 * external circuit and returns into `neg`; the part's own element current is `-I` in the pin0 -> pin1 frame, plus
 * its shunt's `V(pos - neg) / rParallel`. Only voltage differences are asserted, as in the polarity file: where a
 * floating loop's absolute potentials sit is the solver's own reference choice, not part of the contract.
 */

import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

function isrcIntoResistor(current: number, resistance: number, rParallel = 0): SimCircuit {
  return {
    components: [
      { id: "i1", kind: "current_source", pins: [{ id: "pos" }, { id: "neg" }], params: { current, rParallel } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
    ],
    wires: [
      { from_component: "i1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "i1", to_pin: "neg" },
    ],
  };
}

function run(circuit: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < 20; i++) engine.step(1e-4);
  return engine;
}

function pinV(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.getNetIdForPin(componentId, pinId);
  if (net === undefined) throw new Error(`no net for ${componentId}.${pinId}`);
  return engine.getNetV()[net] ?? Number.NaN;
}

describe("current_source drives its current out of pos (transient)", () => {
  it("lifts the resistor end on pos above the end on neg by I*R", () => {
    const engine = run(isrcIntoResistor(10e-3, 470));
    expect(pinV(engine, "r1", "a") - pinV(engine, "r1", "b")).toBeCloseTo(4.7, 6);
  });

  it("carries the resistor current from a to b, the same way round", () => {
    expect(run(isrcIntoResistor(1e-3, 1000)).getElementI().r1).toBeCloseTo(1e-3, 9);
  });

  it("publishes its own current in the pin0 -> pin1 frame, which runs against it", () => {
    expect(run(isrcIntoResistor(1e-3, 1000)).getElementI().i1).toBeCloseTo(-1e-3, 9);
    // 1 mA splits evenly between the 1k load and the 1k shunt: 0.5 V across both, 0.5 mA back through the shunt.
    const shunted = run(isrcIntoResistor(1e-3, 1000, 1000));
    expect(pinV(shunted, "r1", "a") - pinV(shunted, "r1", "b")).toBeCloseTo(0.5, 6);
    expect(shunted.getElementI().i1).toBeCloseTo(-1e-3 + 0.5e-3, 9);
  });

  it("reverses with a negative current", () => {
    const engine = run(isrcIntoResistor(-2e-3, 1000));
    expect(pinV(engine, "r1", "a") - pinV(engine, "r1", "b")).toBeCloseTo(-2, 6);
    expect(engine.getElementI().i1).toBeCloseTo(2e-3, 9);
  });
});
