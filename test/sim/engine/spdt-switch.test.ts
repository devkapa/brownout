/**
 * Q-O26 fix: `spdt_switch.updateCurrent` must report the current through the
 * contact IN USE — com to the selected throw (b at position 0, c otherwise)
 * — not always com->b regardless of position (devices/passives.ts,
 * core/README.md's "Currents" section, and the W1 plan's section 2.2).
 *
 * Rig: vcc -> sw.com; sw.b is left completely unwired (a floating throw, only
 * the 1e-12 S node shunt holds it); sw.c -> r1 -> vcc.neg. At position 1 the
 * switch stamps com->c (through r1, a real load), leaving com->b unstamped
 * and b floating. Before the fix, `updateCurrent` read com->b unconditionally:
 * with com pinned near 5 V through the closed com->c contact and b floating,
 * that bogus reading was on the order of 5000 A (the exact probe value the
 * plan's Q-O26 entry cites) — nothing close to the real ~5 mA load current
 * flowing through com->c.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

function spdtRig(position: number): SimCircuit {
  return {
    components: [
      { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "sw", kind: "spdt_switch", pins: [{ id: "com" }, { id: "b" }, { id: "c" }], params: { position } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: "sw", to_pin: "com" },
      // sw.b is intentionally left unwired — a floating, unloaded throw.
      { from_component: "sw", from_pin: "c", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
    ],
  };
}

function run(position: number): SimEngine {
  const engine = new SimEngine();
  engine.load(spdtRig(position));
  for (let i = 0; i < 20; i++) engine.step(1e-4);
  return engine;
}

describe("spdt_switch.updateCurrent — Q-O26 (com to the contact in use)", () => {
  it("position 0 (com->b active, b floating with no load) reports ~0 A: no complete circuit", () => {
    const engine = run(0);
    const i = Math.abs(engine.getElementI().sw ?? NaN);
    // com->b is stamped, but b is an isolated net (only the 1e-12 S node
    // shunt) — there is no return path, so the branch current must be tiny.
    expect(i).toBeLessThan(1e-4);
  });

  it("position 1 (com->c active, c loaded through r1) reports the real ~5 mA load current — THIS is the case that read 5000 A before the fix", () => {
    const engine = run(1);
    const i = engine.getElementI().sw ?? NaN;
    // 5 V / (1000 ohm + 1 mOhm contact) ~= 4.995 mA through com->c.
    expect(Math.abs(i)).toBeGreaterThan(4e-3);
    expect(Math.abs(i)).toBeLessThan(6e-3);
    // The pre-fix bug reported com->b unconditionally: with com pinned near
    // 5 V and b floating, that read on the order of 5000 A. Pin the fixed
    // regression down hard, well below any plausible bogus reading.
    expect(Math.abs(i)).toBeLessThan(1);
  });

  it("fewer than 3 declared pins: stamp() never stamps, updateCurrent() reports 0 (not a phantom current through an unstamped contact)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        // Only com/b declared -- c is entirely missing, so pins.length is 2.
        { id: "sw", kind: "spdt_switch", pins: [{ id: "com" }, { id: "b" }], params: { position: 1 } },
      ],
      wires: [{ from_component: "vcc", from_pin: "pos", to_component: "sw", to_pin: "com" }],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    for (let i = 0; i < 20; i++) engine.step(1e-4);
    // Before the fix, updateCurrent read com (pinned near 5V) against a
    // never-stamped, floating "c" node, reporting ~5000 A.
    expect(engine.getElementI().sw).toBe(0);
  });

  it("keeps reporting com->b at position 0 even when a load is later added to c (no cross-talk between throws)", () => {
    const circuit = spdtRig(0);
    circuit.components.push({ id: "r2", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 500 } });
    circuit.wires.push(
      { from_component: "sw", from_pin: "c", to_component: "r2", to_pin: "a" },
      { from_component: "r2", from_pin: "b", to_component: "vcc", to_pin: "neg" },
    );
    const engine = new SimEngine();
    engine.load(circuit);
    for (let i = 0; i < 20; i++) engine.step(1e-4);
    // b is still unloaded at position 0 (only c has a resistor now), so the
    // reported com->b current is still ~0, not the com->c branch's current.
    expect(Math.abs(engine.getElementI().sw ?? NaN)).toBeLessThan(1e-4);
  });
});
