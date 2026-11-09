import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

// The micro:bit had the same bug as the Uno (2026-11-08): its 3V pad was an
// ideal 3.3 V source while USB-powered, so a bench PSU on the pad was a second
// ideal source in parallel and no step converged. The V1 board reaches the pad
// through D1, a BAT60A Schottky, from the interface chip's 3.3 V regulator.

function microbitWithPsu(power: 0 | 1, voltage: number | null): SimCircuit {
  const circuit: SimCircuit = {
    components: [
      {
        id: "mb",
        kind: "microbit",
        pins: [{ id: "p0" }, { id: "p1" }, { id: "p2" }, { id: "3v" }, { id: "gnd" }],
        params: { script: "", power },
      },
    ],
    wires: [],
  };
  if (voltage !== null) {
    circuit.components.push({
      id: "p",
      kind: "bench_psu",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { voltage, iLimit: 1 },
    });
    circuit.wires.push(
      { from_component: "p", from_pin: "pos", to_component: "mb", to_pin: "3v" },
      { from_component: "p", from_pin: "neg", to_component: "mb", to_pin: "gnd" },
    );
  }
  return circuit;
}

function run(circuit: SimCircuit, seconds = 0.05): { engine: SimEngine; allConverged: boolean } {
  const engine = new SimEngine();
  engine.load(circuit);
  let allConverged = engine.lastConverged;
  for (let i = 0; i < Math.round(seconds / 1e-3); i++) {
    engine.step(1e-3);
    allConverged &&= engine.lastConverged;
  }
  return { engine, allConverged };
}

function padV(engine: SimEngine): number {
  const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === "mb" && p === "3v"))!;
  return engine.getNetV()[net.id] ?? Number.NaN;
}

function overvoltage(engine: SimEngine) {
  return Object.values(engine.getFailures()).filter((f) => f.kind === "supply_overvoltage");
}

describe("micro:bit 3V pad with a bench PSU on it", () => {
  for (const voltage of [3.3, 5]) {
    it(`converges with USB on and ${voltage} V on the pad, and the PSU holds it`, () => {
      const { engine, allConverged } = run(microbitWithPsu(1, voltage));
      expect(allConverged).toBe(true);
      expect(padV(engine)).toBeCloseTo(voltage, 3);
    });

    it(`converges with USB off and ${voltage} V on the pad (control)`, () => {
      const { engine, allConverged } = run(microbitWithPsu(0, voltage));
      expect(allConverged).toBe(true);
      expect(padV(engine)).toBeCloseTo(voltage, 3);
    });
  }

  it("USB alone: 3.3 V regulator through D1 into the 15 mA idle load (hand value)", () => {
    // D1 BAT60A, Shockley fit to its datasheet typical VF (0.12 V at 10 mA,
    // 0.2 V at 100 mA): n = 1.352, Is = 316 uA, plus 0.02 ohm of bulk.
    // Idle load 3.3 / 0.015 = 220 ohm. Solve 3.3 = I*0.02 + n*Vt*ln(I/Is + 1)
    // + I*220 (Vt at 25 C): I = 14.39 mA, pad = 3.16631 V.
    const { engine, allConverged } = run(microbitWithPsu(1, null));
    expect(allConverged).toBe(true);
    expect(padV(engine)).toBeCloseTo(3.16631, 4);
  });
});

describe("micro:bit supply overvoltage", () => {
  it("latches at 5 V on the 3V pad, naming the nRF51822's 3.9 V absolute maximum", () => {
    const { engine } = run(microbitWithPsu(1, 5));
    const failures = overvoltage(engine);
    expect(failures).toHaveLength(1);
    expect(failures[0]!.componentId).toBe("mb");
    expect(failures[0]!.pinId).toBe("3v");
    expect(failures[0]!.message).toContain("nRF51822");
    expect(failures[0]!.message).toContain("3.90 V");
  });

  it("is quiet at 3.3 V on the pad and on USB alone", () => {
    expect(overvoltage(run(microbitWithPsu(1, 3.3)).engine)).toEqual([]);
    expect(overvoltage(run(microbitWithPsu(1, null)).engine)).toEqual([]);
  });
});
