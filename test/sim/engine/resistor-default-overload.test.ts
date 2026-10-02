/**
 * Parts with no rating-relevant param use the same default the stamp uses.
 *
 * A resistor with no `resistance` stamped 1 kohm but its overload check read
 * the missing param as 0 ohm, so it dissipated 0 W and could never fail. A
 * fuse with no `iRating` read as 0 A and never tripped, although the catalog's
 * default rating is 1 A.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit, type SimFailureKind } from "../../../src/sim/engine/sim-engine.js";

function sourceAcross(component: SimCircuit["components"][number], voltage: number): SimCircuit {
  return {
    components: [
      { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage } },
      component,
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: component.id, to_pin: "a" },
      { from_component: component.id, from_pin: "b", to_component: "vcc", to_pin: "neg" },
    ],
  } as SimCircuit;
}

function failureKindsAfter(circuit: SimCircuit, seconds: number): SimFailureKind[] {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < seconds * 1000; i++) engine.step(1e-3);
  return Object.values(engine.getFailures()).map((failure) => failure.kind).sort();
}

const pins = [{ id: "a" }, { id: "b" }];

describe("default resistance and fuse rating reach the failure path", () => {
  it("latches resistor_overload on a resistor with no resistance param at 2.5 W", () => {
    // 50 V across the engine's 1 kohm fallback is 2.5 W against the 0.25 W default rating.
    const circuit = sourceAcross({ id: "r1", kind: "resistor", pins, params: {} }, 50);
    expect(failureKindsAfter(circuit, 2)).toEqual(["resistor_overload"]);
  });

  it("leaves a resistor with no resistance param alone inside its rating", () => {
    // 5 V across 1 kohm is 25 mW.
    const circuit = sourceAcross({ id: "r1", kind: "resistor", pins, params: {} }, 5);
    expect(failureKindsAfter(circuit, 2)).toEqual([]);
  });

  it("trips a fuse with no iRating param at the catalog's 1 A default", () => {
    // 5 V across the 0.1 ohm default is 50 A through a 1 A fuse.
    const circuit = sourceAcross({ id: "f1", kind: "fuse", pins, params: {} }, 5);
    expect(failureKindsAfter(circuit, 2)).toEqual(["fuse_tripped"]);
  });

  it("leaves a fuse with no iRating param alone below 1 A", () => {
    // 0.05 V across the 0.1 ohm default is 0.5 A.
    const circuit = sourceAcross({ id: "f1", kind: "fuse", pins, params: {} }, 0.05);
    expect(failureKindsAfter(circuit, 2)).toEqual([]);
  });
});
