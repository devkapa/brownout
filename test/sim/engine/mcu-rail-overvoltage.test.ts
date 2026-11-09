import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { ARDUINO_NANO_PINS, cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

// A supply that back-feeds a USB-powered board's rail past its MCU's absolute
// maximum now converges (the board's USB path is modelled), so the engine
// must say the chip is at risk instead of quietly running it at 9 V.

type Board = "arduino_uno" | "arduino_nano" | "raspberry_pi_pico";

function boardPins(kind: Board): { id: string }[] {
  if (kind === "arduino_uno") return cloneArduinoUnoPins().map((pin) => ({ id: pin.id }));
  if (kind === "arduino_nano") return ARDUINO_NANO_PINS.map((pin) => ({ id: pin.id }));
  return ["gp15", "3v3", "3v3_en", "vsys", "vbus", "gnd"].map((id) => ({ id }));
}

function boardWithPsu(kind: Board, voltage: number | null, pin: string, usbPower: 0 | 1 = 1): SimCircuit {
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
  if (voltage !== null) {
    circuit.components.push({
      id: "p",
      kind: "bench_psu",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { voltage, iLimit: 1 },
    });
    circuit.wires.push(
      { from_component: "p", from_pin: "pos", to_component: "u", to_pin: pin },
      { from_component: "p", from_pin: "neg", to_component: "u", to_pin: "gnd" },
    );
  }
  return circuit;
}

function run(circuit: SimCircuit, seconds = 0.05): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < Math.round(seconds / 1e-3); i++) engine.step(1e-3);
  return engine;
}

function overvoltage(engine: SimEngine) {
  return Object.values(engine.getFailures()).filter((f) => f.kind === "supply_overvoltage");
}

const BOARDS = [
  { kind: "arduino_uno", pin: "5v", over: 9, ok: 5, chip: "ATmega328P", limit: "6.0 V" },
  { kind: "arduino_nano", pin: "5v", over: 9, ok: 5, chip: "ATmega328P", limit: "6.0 V" },
  { kind: "raspberry_pi_pico", pin: "3v3", over: 5, ok: 3.3, chip: "RP2040", limit: "3.63 V" },
] as const;

describe.each(BOARDS)("$kind supply overvoltage", (board) => {
  it(`latches when ${board.over} V back-feeds the ${board.pin} pin, naming the ${board.chip}'s ${board.limit} absolute maximum`, () => {
    // 0.5 s: on the Uno a 1 A-limited PSU first holds the rail near 5.4 V
    // until the 1 A back-feed trips the USB polyfuse (~0.1 s), then the rail
    // follows the PSU.
    const engine = run(boardWithPsu(board.kind, board.over, board.pin), 0.5);
    const failures = overvoltage(engine);
    expect(failures).toHaveLength(1);
    const failure = failures[0]!;
    expect(failure.componentId).toBe("u");
    expect(failure.pinId).toBe(board.pin);
    expect(failure.value).toBeCloseTo(board.over, 1);
    expect(failure.message).toContain(board.chip);
    expect(failure.message).toContain(board.limit);
  });

  it(`is quiet at ${board.ok} V on the ${board.pin} pin and on USB alone`, () => {
    expect(overvoltage(run(boardWithPsu(board.kind, board.ok, board.pin)))).toEqual([]);
    expect(overvoltage(run(boardWithPsu(board.kind, null, board.pin)))).toEqual([]);
  });

  it("also latches with USB power off", () => {
    expect(overvoltage(run(boardWithPsu(board.kind, board.over, board.pin, 0)))).toHaveLength(1);
  });

  it("stays latched after the supply comes down, until failures are cleared", () => {
    const engine = run(boardWithPsu(board.kind, board.over, board.pin), 0.5);
    engine.load(boardWithPsu(board.kind, board.ok, board.pin));
    for (let i = 0; i < 20; i++) engine.step(1e-3);
    expect(overvoltage(engine)).toHaveLength(1);
    engine.resetFailures();
    for (let i = 0; i < 20; i++) engine.step(1e-3);
    expect(overvoltage(engine)).toEqual([]);
  });
});
