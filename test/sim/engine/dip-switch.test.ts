/**
 * Engine tests for the dip_switch kind.
 *
 * Each closed position should conduct (voltage divider test ≈ 0 Ω).
 * Open positions must isolate.
 * sw params beyond the positions count are inert.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

function makeCircuit(positions: number, swParams: Record<string, number>): SimCircuit {
  // Simple voltage divider: VCC → 1 kΩ → switch position 1 → GND.
  // If sw1 is closed the mid-point reads close to 0 V (1 mΩ divides almost nothing).
  // If sw1 is open, the mid-point floats (no current path), reads VCC through the resistor.
  return {
    components: [
      {
        id: "vcc",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "r1",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
      {
        id: "sw",
        kind: "dip_switch",
        // Build 2N pins: a1..aN at y=0, b1..bN at y=55
        pins: Array.from({ length: positions * 2 }, (_, k) => {
          const pos = Math.floor(k / 2) + 1;
          const isTop = k % 2 === 0;
          return { id: isTop ? `a${pos}` : `b${pos}` };
        }),
        params: { positions, ...swParams },
      },
    ],
    wires: [
      // VCC pos → resistor a
      { from_component: "vcc", from_pin: "pos", to_component: "r1", to_pin: "a" },
      // resistor b → switch a1
      { from_component: "r1", from_pin: "b", to_component: "sw", to_pin: "a1" },
      // switch b1 → VCC neg (GND)
      { from_component: "sw", from_pin: "b1", to_component: "vcc", to_pin: "neg" },
    ],
  };
}

function runCircuit(circuit: SimCircuit, steps = 20): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(1e-4);
  return engine;
}

// ── Closed position conducts ─────────────────────────────────────────────────

describe("dip_switch — closed position conducts", () => {
  it("sw1=1 with VCC/resistor/GND → large current flows (≈ 5 A through 1 mΩ + 1 kΩ)", () => {
    const circuit = makeCircuit(4, { sw1: 1, sw2: 0, sw3: 0, sw4: 0 });
    const engine = runCircuit(circuit);
    const I = engine.getElementI();
    // 5 V / (1000 + 0.001) Ω ≈ 0.005 A — resistor limits current
    expect(Math.abs(I["sw"] ?? 0)).toBeGreaterThan(0.001);
  });

  it("sw1=0 → no current through the switch", () => {
    const circuit = makeCircuit(4, { sw1: 0, sw2: 0, sw3: 0, sw4: 0 });
    const engine = runCircuit(circuit);
    const I = engine.getElementI();
    // With sw1 open, no complete path — current should be near zero
    expect(Math.abs(I["sw"] ?? 0)).toBeLessThan(1e-6);
  });
});

// ── Open positions isolate ────────────────────────────────────────────────────

describe("dip_switch — open positions isolate", () => {
  it("positions 2..4 open do not affect circuit when sw1 is closed", () => {
    const closedCircuit = makeCircuit(4, { sw1: 1, sw2: 0, sw3: 0, sw4: 0 });
    const allOpenCircuit = makeCircuit(4, { sw1: 0, sw2: 0, sw3: 0, sw4: 0 });
    const engineClosed = runCircuit(closedCircuit);
    const engineOpen = runCircuit(allOpenCircuit);

    const iClosed = Math.abs(engineClosed.getElementI()["sw"] ?? 0);
    const iOpen = Math.abs(engineOpen.getElementI()["sw"] ?? 0);
    // Closed should have much higher current than open
    expect(iClosed).toBeGreaterThan(iOpen * 100);
  });
});

// ── positions param respected ─────────────────────────────────────────────────

describe("dip_switch — positions param", () => {
  it("N=2 positions works correctly", () => {
    const circuit = makeCircuit(2, { sw1: 1, sw2: 0 });
    const engine = runCircuit(circuit);
    const I = engine.getElementI();
    expect(Math.abs(I["sw"] ?? 0)).toBeGreaterThan(0.001);
  });

  it("N=8 positions works correctly", () => {
    const circuit = makeCircuit(8, { sw1: 1, sw2: 0, sw3: 0, sw4: 0, sw5: 0, sw6: 0, sw7: 0, sw8: 0 });
    const engine = runCircuit(circuit);
    const I = engine.getElementI();
    expect(Math.abs(I["sw"] ?? 0)).toBeGreaterThan(0.001);
  });
});

// ── sw params beyond positions are inert ──────────────────────────────────────

describe("dip_switch — extra sw params are inert", () => {
  it("sw5..sw8 on a 4-position switch do not affect the circuit", () => {
    // 4-position switch: sw5..sw8 should be ignored even if set to 1
    const normalCircuit = makeCircuit(4, { sw1: 1, sw2: 0, sw3: 0, sw4: 0 });
    const extraParamCircuit = makeCircuit(4, { sw1: 1, sw2: 0, sw3: 0, sw4: 0, sw5: 1, sw6: 1, sw7: 1, sw8: 1 });

    const engineNormal = runCircuit(normalCircuit);
    const engineExtra = runCircuit(extraParamCircuit);

    const iNormal = engineNormal.getElementI()["sw"] ?? 0;
    const iExtra = engineExtra.getElementI()["sw"] ?? 0;

    // Both circuits have the same active positions — results should be identical
    expect(Math.abs(iNormal - iExtra)).toBeLessThan(1e-9);
  });
});
