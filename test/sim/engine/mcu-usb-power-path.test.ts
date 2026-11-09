import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { ARDUINO_NANO_PINS, cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

// Regression for the 2026-11-08 report: a board left on USB power with a bench
// PSU wired to its 5V (or the Pico's 3V3) pin never converged, because the
// board's USB rail was an ideal voltage source in parallel with the PSU's.
// The real boards reach that pin through a protected path (Uno: polyfuse +
// P-MOSFET; Nano: Schottky; Pico: a regulator output), and the engine now
// models that path, so the external supply shares or back-feeds instead of
// making the matrix singular.

type Board = "arduino_uno" | "arduino_nano" | "raspberry_pi_pico";

const PICO_PINS = ["gp15", "3v3", "3v3_en", "vsys", "vbus", "gnd"];

function boardPins(kind: Board): { id: string }[] {
  if (kind === "arduino_uno") return cloneArduinoUnoPins().map((pin) => ({ id: pin.id }));
  if (kind === "arduino_nano") return ARDUINO_NANO_PINS.map((pin) => ({ id: pin.id }));
  return PICO_PINS.map((id) => ({ id }));
}

function boardWithPsu(
  kind: Board,
  usbPower: 0 | 1,
  psu: { voltage: number; iLimit?: number; pin: string } | null,
): SimCircuit {
  const circuit: SimCircuit = {
    components: [
      {
        id: "u",
        kind,
        pins: boardPins(kind),
        params: { vcc: kind === "raspberry_pi_pico" ? 3.3 : 5, usb_power: usbPower },
      },
    ],
    wires: [],
  };
  if (psu) {
    circuit.components.push({
      id: "p",
      kind: "bench_psu",
      pins: [{ id: "pos" }, { id: "neg" }],
      // devolt fills catalogue defaults, which give bench_psu iLimit 1 A.
      params: psu.iLimit === undefined ? { voltage: psu.voltage } : { voltage: psu.voltage, iLimit: psu.iLimit },
    });
    circuit.wires.push(
      { from_component: "p", from_pin: "pos", to_component: "u", to_pin: psu.pin },
      { from_component: "p", from_pin: "neg", to_component: "u", to_pin: "gnd" },
    );
  }
  return circuit;
}

function pinV(engine: SimEngine, compId: string, pinId: string): number {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([c, p]) => c === compId && p === pinId),
  );
  if (!net) throw new Error(`missing net for ${compId}.${pinId}`);
  return engine.getNetV()[net.id] ?? Number.NaN;
}

/** Load, then step for `seconds`; returns whether every solve converged. */
function run(circuit: SimCircuit, seconds: number, h = 1e-3): { engine: SimEngine; allConverged: boolean } {
  const engine = new SimEngine();
  engine.load(circuit);
  let allConverged = engine.lastConverged;
  const steps = Math.round(seconds / h);
  for (let i = 0; i < steps; i++) {
    engine.step(h);
    allConverged &&= engine.lastConverged;
  }
  return { engine, allConverged };
}

function ptcTripped(engine: SimEngine, key: string): boolean | undefined {
  return (engine as unknown as { state: { ptcs: Map<string, { tripped: boolean }> } })
    .state.ptcs.get(key)?.tripped;
}

describe.each(["arduino_uno", "arduino_nano"] as const)("%s on USB with a PSU on the 5V pin", (kind) => {
  for (const iLimit of [undefined, 1]) {
    const label = iLimit === undefined ? "ideal PSU" : `PSU limited to ${iLimit} A`;

    it(`converges at 5 V (${label}) and the PSU holds the rail`, () => {
      const { engine, allConverged } = run(boardWithPsu(kind, 1, { voltage: 5, iLimit, pin: "5v" }), 0.05);
      expect(allConverged).toBe(true);
      expect(pinV(engine, "u", "5v")).toBeCloseTo(5, 3);
    });

    it(`converges at 9 V (${label}) and the rail follows the PSU`, () => {
      const { engine, allConverged } = run(boardWithPsu(kind, 1, { voltage: 9, iLimit, pin: "5v" }), 0.5);
      expect(allConverged).toBe(true);
      // The board's own path cannot hold 9 V down: the Uno's polyfuse trips on
      // the back-feed, the Nano's Schottky blocks it, so the rail sits at the PSU.
      expect(pinV(engine, "u", "5v")).toBeGreaterThan(8.5);
    });
  }

  it("usb_power=0 with 5 V and 9 V on the 5V pin still converges (control)", () => {
    for (const voltage of [5, 9]) {
      const { engine, allConverged } = run(boardWithPsu(kind, 0, { voltage, iLimit: 1, pin: "5v" }), 0.05);
      expect(allConverged).toBe(true);
      expect(pinV(engine, "u", "5v")).toBeCloseTo(voltage, 3);
    }
  });

  it("9 V into VIN converges with USB on and off (control)", () => {
    for (const usb of [0, 1] as const) {
      const { allConverged } = run(boardWithPsu(kind, usb, { voltage: 9, iLimit: 1, pin: "vin" }), 0.05);
      expect(allConverged).toBe(true);
    }
  });
});

describe("arduino_uno USB polyfuse", () => {
  it("trips on a 9 V back-feed into the 5V pin", () => {
    const { engine, allConverged } = run(boardWithPsu("arduino_uno", 1, { voltage: 9, iLimit: 1, pin: "5v" }), 0.5);
    expect(allConverged).toBe(true);
    expect(ptcTripped(engine, "u")).toBe(true);
  });

  it("stays closed when a 5 V PSU shares the rail", () => {
    const { engine } = run(boardWithPsu("arduino_uno", 1, { voltage: 5, iLimit: 1, pin: "5v" }), 0.5);
    expect(ptcTripped(engine, "u") ?? false).toBe(false);
  });
});

describe("raspberry_pi_pico on USB with a PSU on the 3V3 pin", () => {
  for (const iLimit of [undefined, 1]) {
    const label = iLimit === undefined ? "ideal PSU" : `PSU limited to ${iLimit} A`;

    it(`converges at 3.3 V (${label})`, () => {
      const { engine, allConverged } = run(boardWithPsu("raspberry_pi_pico", 1, { voltage: 3.3, iLimit, pin: "3v3" }), 0.05);
      expect(allConverged).toBe(true);
      expect(pinV(engine, "u", "3v3")).toBeCloseTo(3.3, 3);
    });

    it(`converges at 5 V (${label}): the regulator cannot sink the back-feed`, () => {
      const { engine, allConverged } = run(boardWithPsu("raspberry_pi_pico", 1, { voltage: 5, iLimit, pin: "3v3" }), 0.05);
      expect(allConverged).toBe(true);
      expect(pinV(engine, "u", "3v3")).toBeCloseTo(5, 2);
    });
  }

  it("usb_power=0 with 3.3 V on the 3V3 pin converges (control)", () => {
    const { engine, allConverged } = run(boardWithPsu("raspberry_pi_pico", 0, { voltage: 3.3, iLimit: 1, pin: "3v3" }), 0.05);
    expect(allConverged).toBe(true);
    expect(pinV(engine, "u", "3v3")).toBeCloseTo(3.3, 3);
  });
});

describe("USB-only rails through the modelled supply path (hand values)", () => {
  it("Uno: 5 V host behind 0.2 + 0.15 + 0.07 ohm into the 45 mA idle load", () => {
    // Idle load R = 5 / 0.045 = 111.11 ohm; 5 * 111.11 / (111.11 + 0.42).
    const { engine, allConverged } = run(boardWithPsu("arduino_uno", 1, null), 0.01);
    expect(allConverged).toBe(true);
    expect(pinV(engine, "u", "5v")).toBeCloseTo(4.98117, 4);
  });

  it("Nano: 5 V host behind 0.27 ohm and the MBR0520 Schottky into the 20 mA idle load", () => {
    // Solve 5 = I*0.27 + Vt*ln(I/2.4e-6 + 1) + I*250 (Vt at 25 C):
    // I = 19.057 mA, D1 drops 0.2307 V, rail = 4.76414 V.
    const { engine, allConverged } = run(boardWithPsu("arduino_nano", 1, null), 0.01);
    expect(allConverged).toBe(true);
    expect(pinV(engine, "u", "5v")).toBeCloseTo(4.76414, 4);
  });

  it("Pico: 3.3 V regulator behind 10 mohm into the 25 mA idle load", () => {
    // Idle load R = 3.3 / 0.025 = 132 ohm; 3.3 * 132 / 132.01 = 3.29975 V.
    const { engine, allConverged } = run(boardWithPsu("raspberry_pi_pico", 1, null), 0.01);
    expect(allConverged).toBe(true);
    expect(pinV(engine, "u", "3v3")).toBeCloseTo(3.29975, 5);
  });
});

describe("raspberry_pi_pico regulator regime", () => {
  it("turns back on once the back-feeding supply is removed", () => {
    const { engine } = run(boardWithPsu("raspberry_pi_pico", 1, { voltage: 5, iLimit: 1, pin: "3v3" }), 0.02);
    expect(pinV(engine, "u", "3v3")).toBeCloseTo(5, 2);
    engine.load(boardWithPsu("raspberry_pi_pico", 1, null));
    let allConverged = true;
    for (let i = 0; i < 20; i++) {
      engine.step(1e-3);
      allConverged &&= engine.lastConverged;
    }
    expect(allConverged).toBe(true);
    expect(pinV(engine, "u", "3v3")).toBeCloseTo(3.29975, 5);
  });
});
