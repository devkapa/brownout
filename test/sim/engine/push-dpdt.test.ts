import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

// Each pole's common is driven from +5V through the DPDT to one of two 1k
// resistors (left throw / right throw) returning to ground. Whichever throw is
// connected carries ~5 mA; the open one carries ~0. Confirms both poles switch
// together and that left/right are never simultaneously connected.
function dpdtRig(position: number): SimCircuit {
  const R = 1000;
  return {
    components: [
      { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      {
        id: "sw", kind: "push_dpdt",
        pins: [{ id: "l1" }, { id: "c1" }, { id: "r1" }, { id: "l2" }, { id: "c2" }, { id: "r2" }],
        params: { position },
      },
      { id: "rL1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R } },
      { id: "rR1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R } },
      { id: "rL2", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R } },
      { id: "rR2", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R } },
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: "sw", to_pin: "c1" },
      { from_component: "vcc", from_pin: "pos", to_component: "sw", to_pin: "c2" },
      { from_component: "sw", from_pin: "l1", to_component: "rL1", to_pin: "a" },
      { from_component: "sw", from_pin: "r1", to_component: "rR1", to_pin: "a" },
      { from_component: "sw", from_pin: "l2", to_component: "rL2", to_pin: "a" },
      { from_component: "sw", from_pin: "r2", to_component: "rR2", to_pin: "a" },
      { from_component: "rL1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      { from_component: "rR1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      { from_component: "rL2", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      { from_component: "rR2", from_pin: "b", to_component: "vcc", to_pin: "neg" },
    ],
  };
}

function run(position: number) {
  const e = new SimEngine();
  e.load(dpdtRig(position));
  for (let i = 0; i < 40; i++) e.step(1e-4);
  return e.getElementI();
}

describe("push_dpdt DPDT switching", () => {
  it("position 0 latches both commons to the LEFT throws", () => {
    const i = run(0);
    expect(Math.abs(i["rL1"])).toBeGreaterThan(4e-3); // ~5 mA
    expect(Math.abs(i["rL2"])).toBeGreaterThan(4e-3);
    expect(Math.abs(i["rR1"])).toBeLessThan(1e-4);     // open
    expect(Math.abs(i["rR2"])).toBeLessThan(1e-4);
  });

  it("position 1 latches both commons to the RIGHT throws", () => {
    const i = run(1);
    expect(Math.abs(i["rR1"])).toBeGreaterThan(4e-3);
    expect(Math.abs(i["rR2"])).toBeGreaterThan(4e-3);
    expect(Math.abs(i["rL1"])).toBeLessThan(1e-4);
    expect(Math.abs(i["rL2"])).toBeLessThan(1e-4);
  });
});
