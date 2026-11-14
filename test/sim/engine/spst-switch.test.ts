/**
 * The plain SPST `switch` (devices/passives.ts `switchModel`): a closed
 * contact is a literal 1 mOhm, an open one stamps nothing, and `closed` is
 * read with `Boolean(...)`, so the string "0" counts as closed. The reported
 * current is the contact current pins[0] -> pins[1], 0 while open.
 *
 * Gate 1 (the core-suites project) also runs this file against the core,
 * which is why every assertion reads through `nets`/`getNetV`/`getElementI`
 * and `step()` only.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const CONTACT_OHMS = 0.001;

/** vcc.pos -> sw.a, sw.b -> node "load" -> r1 -> vcc.neg, plus an optional capacitor from "load" to vcc.neg. */
function switchRig(closed: number | string, withCapacitor = false): SimCircuit {
  const circuit: SimCircuit = {
    components: [
      { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "sw", kind: "switch", pins: [{ id: "a" }, { id: "b" }], params: { closed } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: "sw", to_pin: "a" },
      { from_component: "sw", from_pin: "b", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
    ],
  };
  if (withCapacitor) {
    circuit.components.push({ id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } });
    circuit.wires.push(
      { from_component: "sw", from_pin: "b", to_component: "c1", to_pin: "a" },
      { from_component: "c1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
    );
  }
  return circuit;
}

function loadNodeVoltage(engine: SimEngine): number {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([componentId, pinId]) => componentId === "r1" && pinId === "a"),
  );
  if (!net) throw new Error("r1.a has no net");
  return engine.getNetV()[net.id] ?? 0;
}

function settle(circuit: SimCircuit, steps = 20, h = 1e-4): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(h);
  return engine;
}

describe("spst switch — closed and open contacts", () => {
  it("a closed contact is 1 mOhm: the load sees the supply less the contact drop, and the switch reports the series current", () => {
    const engine = settle(switchRig(1));
    const expectedCurrent = 5 / (1000 + CONTACT_OHMS);
    expect(loadNodeVoltage(engine)).toBeCloseTo(5 - expectedCurrent * CONTACT_OHMS, 6);
    expect(engine.getElementI().sw).toBeCloseTo(expectedCurrent, 9);
    expect(engine.getElementI().r1).toBeCloseTo(expectedCurrent, 9);
  });

  it("an open contact stamps nothing: the load node falls to 0 V and the switch reports 0 A", () => {
    const engine = settle(switchRig(0));
    expect(Math.abs(loadNodeVoltage(engine))).toBeLessThan(1e-6);
    expect(engine.getElementI().sw).toBe(0);
    expect(Math.abs(engine.getElementI().r1 ?? 0)).toBeLessThan(1e-9);
  });

  it('the string "0" counts as closed, because classic reads the param with Boolean()', () => {
    const engine = settle(switchRig("0"));
    expect(loadNodeVoltage(engine)).toBeCloseTo(5, 4);
    expect(engine.getElementI().sw).toBeCloseTo(5 / (1000 + CONTACT_OHMS), 9);
  });
});

describe("spst switch — toggling and dynamics", () => {
  it("reloading with the contact opened after it was closed drops the load to 0 V", () => {
    const engine = new SimEngine();
    engine.load(switchRig(1));
    for (let i = 0; i < 10; i++) engine.step(1e-4);
    expect(loadNodeVoltage(engine)).toBeCloseTo(5, 4);

    engine.load(switchRig(0));
    for (let i = 0; i < 10; i++) engine.step(1e-4);
    expect(Math.abs(loadNodeVoltage(engine))).toBeLessThan(1e-6);
    expect(engine.getElementI().sw).toBe(0);
  });

  it("a closed contact charges a capacitor behind it toward the supply, and an open one never does", () => {
    // Closed, the source drives the node through 1 mOhm (tau = 1 mOhm *
    // 1 uF = 1 ns), so the capacitor sits at the supply; r1 loading it
    // costs only the contact drop. Open, nothing ever charges it.
    const closed = settle(switchRig(1, true), 50, 1e-4);
    expect(loadNodeVoltage(closed)).toBeCloseTo(5, 3);
    const open = settle(switchRig(0, true), 50, 1e-4);
    expect(Math.abs(loadNodeVoltage(open))).toBeLessThan(1e-6);
    expect(open.getElementI().c1 ?? 0).toBeCloseTo(0, 9);
  });
});
