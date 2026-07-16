import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { buildNets } from "../../../src/sim/engine/graph.js";

const pb4 = (closed: number): SimCircuit["components"][number] => ({
  id: "sw", kind: "push_button",
  pins: [{ id: "a" }, { id: "b" }, { id: "a2" }, { id: "b2" }],
  params: { closed },
});

function run(c: SimCircuit) {
  const e = new SimEngine();
  e.load(c);
  for (let i = 0; i < 40; i++) e.step(1e-4);
  return e.getElementI();
}

describe("push_button 4-pin internal shorts", () => {
  it("a≡a2 and b≡b2 are the SAME net (one conductor, not a resistor bridge)", () => {
    const nets = buildNets({ components: [pb4(0)], wires: [] });
    const netOf = (pin: string) => nets.find((n) => n.pins.some(([c, p]) => c === "sw" && p === pin))?.id;
    expect(netOf("a")).toBe(netOf("a2"));
    expect(netOf("b")).toBe(netOf("b2"));
    expect(netOf("a")).not.toBe(netOf("b")); // top side is distinct from bottom until pressed
  });

  it("current entering a2 leaves via b2 when pressed (a2→a→b→b2 through the contact)", () => {
    // 5V on a2, load on b2 to ground. Closed: a2≡a, contact a→b, b≡b2 → load conducts.
    const i = run({
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        pb4(1),
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "sw", to_pin: "a2" },
        { from_component: "sw", from_pin: "b2", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
    });
    expect(Math.abs(i["r"])).toBeGreaterThan(4e-3); // ~5 mA
  });

  it("open button breaks the a↔b path (a2/b2 still tied to their partners)", () => {
    const i = run({
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        pb4(0),
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "sw", to_pin: "a2" },
        { from_component: "sw", from_pin: "b2", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
    });
    expect(Math.abs(i["r"])).toBeLessThan(1e-4); // open
  });
});
