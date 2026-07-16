import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { ArduinoMcu, parseIntelHex, ARDUINO_IO_PINS, ARDUINO_ANALOG_PINS } from "../../../src/sim/engine/arduino.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { buildNets } from "../../../src/sim/engine/graph.js";
import { ARDUINO_NANO_PINS, cloneArduinoUnoPins, isArduinoBoardKind } from "../../../src/circuit/arduino.js";
import { isDipIcKind } from "../../helpers/circuit/breadboard.js";

const blinkHex = readFileSync(
  join(__dirname, "__fixtures__/blink.hex"),
  "utf-8",
);

// ─── parseIntelHex ────────────────────────────────────────────────────────

describe("parseIntelHex", () => {
  it("returns a Uint8Array", () => {
    const flash = parseIntelHex(blinkHex);
    expect(flash).toBeInstanceOf(Uint8Array);
  });

  it("places the first instruction at offset 0", () => {
    const flash = parseIntelHex(blinkHex);
    // First record contains non-zero data.
    const nonZero = Array.from(flash.slice(0, 32)).some((b) => b !== 0);
    expect(nonZero).toBe(true);
  });

  it("handles empty hex gracefully", () => {
    const flash = parseIntelHex(":00000001FF\n");
    expect(flash.every((b) => b === 0)).toBe(true);
  });
});

// ─── ArduinoMcu ───────────────────────────────────────────────────────────

describe("ArduinoMcu — blink sketch", () => {
  it("constructs without throwing", () => {
    expect(() => new ArduinoMcu(blinkHex)).not.toThrow();
  });

  it("d13 toggles at least once after 1 second of simulation", () => {
    const mcu = new ArduinoMcu(blinkHex);

    // Sample d13 every 10 ms over 1 second.
    const states: string[] = [];
    for (let i = 0; i < 100; i++) {
      mcu.step(0.01);
      states.push(mcu.pinDriveState("d13"));
    }

    const sawHigh = states.includes("out-high");
    const sawLow  = states.includes("out-low");
    expect(sawHigh && sawLow).toBe(true);
  });

  it("d13 starts as output (not input) after DDRB is set", () => {
    const mcu = new ArduinoMcu(blinkHex);
    // Run enough cycles for the first DDRB write to execute (~4 instructions).
    mcu.step(1e-4);
    const state = mcu.pinDriveState("d13");
    expect(state === "out-high" || state === "out-low").toBe(true);
  });

  it("setInputBit does not throw for input pins", () => {
    const mcu = new ArduinoMcu(blinkHex);
    // Before sketch runs, all pins are inputs.
    expect(() => mcu.setInputBit("d2", 1)).not.toThrow();
    expect(() => mcu.setInputBit("a0", 0)).not.toThrow();
  });

  it("preserves the AVR internal pull-up mode", () => {
    const mcu = new ArduinoMcu(blinkHex);
    const cpu = (mcu as unknown as {
      cpu: { writeData(address: number, value: number): void };
    }).cpu;
    // ATmega328P D2 = PORTD bit 2. DDRD low + PORTD high enables INPUT_PULLUP.
    cpu.writeData(0x2a, 0x00);
    cpu.writeData(0x2b, 1 << 2);
    expect(mcu.pinDriveState("d2")).toBe("input-pullup");
  });

  it("reset() restarts the MCU without throwing", () => {
    const mcu = new ArduinoMcu(blinkHex);
    mcu.step(0.5);
    expect(() => mcu.reset()).not.toThrow();
    // After reset, another step should work.
    expect(() => mcu.step(0.01)).not.toThrow();
  });
});

// ─── ARDUINO_IO_PINS ──────────────────────────────────────────────────────

describe("ARDUINO_IO_PINS", () => {
  it("contains d0–d13 and a0–a5", () => {
    for (let i = 0; i <= 13; i++) expect(ARDUINO_IO_PINS).toContain(`d${i}`);
    for (let i = 0; i <= 5; i++)  expect(ARDUINO_IO_PINS).toContain(`a${i}`);
  });

  it("does not contain structural pins or analog-only pins", () => {
    expect(ARDUINO_IO_PINS).not.toContain("vcc");
    expect(ARDUINO_IO_PINS).not.toContain("gnd");
    expect(ARDUINO_IO_PINS).not.toContain("aref");
    expect(ARDUINO_IO_PINS).not.toContain("reset");
    // A6/A7 are Nano-only ADC-only pins with no DDR/PORT path.
    expect(ARDUINO_IO_PINS).not.toContain("a6");
    expect(ARDUINO_IO_PINS).not.toContain("a7");
  });
});

describe("ARDUINO_ANALOG_PINS", () => {
  it("contains every A-pin including Nano-only A6/A7", () => {
    for (let i = 0; i <= 7; i++) expect(ARDUINO_ANALOG_PINS).toContain(`a${i}`);
  });

  it("does not contain digital pins", () => {
    for (let i = 0; i <= 13; i++) expect(ARDUINO_ANALOG_PINS).not.toContain(`d${i}`);
  });
});

// ─── Arduino Uno board integration ───────────────────────────────────────

function arduinoCircuit(params: Record<string, number | string> = {}): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex: blinkHex, usb_power: 1, ...params },
      },
    ],
    wires: [],
  };
}

function pinNet(engine: SimEngine, compId: string, pinId: string): string {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([c, p]) => c === compId && p === pinId),
  );
  if (!net) throw new Error(`missing net for ${compId}.${pinId}`);
  return net.id;
}

function engineArduinoMcu(engine: SimEngine, componentId = "uno"): ArduinoMcu {
  const mcu = (engine as unknown as {
    state: { arduinos: Map<string, ArduinoMcu> };
  }).state.arduinos.get(componentId);
  if (!mcu) throw new Error(`missing MCU core for ${componentId}`);
  return mcu;
}

function avrCycles(mcu: ArduinoMcu): number {
  return (mcu as unknown as { cpu: { cycles: number } }).cpu.cycles;
}

describe("SimEngine MCU commit ordering", () => {
  it("keeps a failed worker-style retry firmware-neutral", () => {
    const circuit = arduinoCircuit();
    circuit.components.push(
      {
        id: "conflict-5v",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "conflict-3v",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 3 },
      },
    );
    circuit.wires.push(
      { from_component: "conflict-5v", from_pin: "pos", to_component: "conflict-3v", to_pin: "pos" },
      { from_component: "conflict-5v", from_pin: "neg", to_component: "conflict-3v", to_pin: "neg" },
    );

    const engine = new SimEngine();
    engine.load(circuit);
    const mcu = engineArduinoMcu(engine);
    const cyclesBefore = avrCycles(mcu);
    const driveBefore = mcu.pinDriveState("d13");
    const eventsBefore = [...mcu.getStepPinEvents()];
    const trusted = engine.saveState();

    engine.step(1e-4);
    expect(engine.lastConverged).toBe(false);
    engine.restoreState(trusted); // mirrors sim.worker retry rollback

    expect(avrCycles(mcu)).toBe(cyclesBefore);
    expect(mcu.pinDriveState("d13")).toBe(driveBefore);
    expect(mcu.getStepPinEvents()).toEqual(eventsBefore);

    // Remove the deliberately singular branch and retry the same physical h.
    circuit.components = circuit.components.filter((component) => component.id === "uno");
    circuit.wires = [];
    engine.load(circuit);
    engine.step(1e-4);
    expect(engine.lastConverged).toBe(true);

    const control = new SimEngine();
    control.load(arduinoCircuit());
    control.step(1e-4);
    const controlMcu = engineArduinoMcu(control);

    expect(avrCycles(mcu)).toBe(avrCycles(controlMcu));
    expect(mcu.pinDriveState("d13")).toBe(controlMcu.pinDriveState("d13"));
    expect(mcu.getStepPinEvents()).toEqual(controlMcu.getStepPinEvents());
  });

  it("samples once and advances firmware once after a converged NE555 split", () => {
    const circuit = arduinoCircuit();
    circuit.components.push({
      id: "timer",
      kind: "ne555",
      pins: Array.from({ length: 8 }, (_, index) => ({ id: String(index + 1) })),
      params: {},
    });

    const engine = new SimEngine();
    engine.load(circuit);
    const mcu = engineArduinoMcu(engine);
    const sequence: string[] = [];
    const internal = engine as unknown as {
      _solve(h: number): void;
      _find555CrossingFrac(...args: unknown[]): number | null;
      _sampleArduinoInputs(): void;
    };
    const solve = internal._solve.bind(engine);
    const sample = internal._sampleArduinoInputs.bind(engine);
    const stepMcu = mcu.step.bind(mcu);

    internal._solve = (h) => {
      sequence.push("solve");
      solve(h);
    };
    internal._find555CrossingFrac = () => 0.5;
    internal._sampleArduinoInputs = () => {
      sequence.push("sample");
      sample();
    };
    mcu.step = (h) => {
      sequence.push("mcu");
      stepMcu(h);
    };

    engine.step(1e-4);

    expect(engine.lastConverged).toBe(true);
    expect(sequence).toEqual(["solve", "solve", "solve", "sample", "mcu"]);
  });
});

describe("Arduino Uno board integration", () => {
  it("is not classified as a breadboard DIP IC", () => {
    expect(isDipIcKind("arduino_uno")).toBe(false);
  });

  it("aliases Arduino ground headers", () => {
    const nets = buildNets(arduinoCircuit());
    const gnd = nets.find((net) => net.pins.some(([c, p]) => c === "uno" && p === "gnd"));
    expect(gnd?.id).toBe("gnd");
    expect(gnd?.pins).toContainEqual(["uno", "gnd2"]);
  });

  it("powers the 5V header from USB when USB power is on", () => {
    const engine = new SimEngine();
    engine.load(arduinoCircuit({ usb_power: 1 }));
    expect(engine.getNetV()[pinNet(engine, "uno", "5v")]).toBeCloseTo(5, 4);
    expect(engine.getNetV()[pinNet(engine, "uno", "gnd")]).toBeCloseTo(0, 6);
  });

  it("leaves the 5V header unpowered when USB power is off", () => {
    const engine = new SimEngine();
    engine.load(arduinoCircuit({ usb_power: 0 }));
    expect(engine.getNetV()[pinNet(engine, "uno", "5v")]).toBeCloseTo(0, 6);
  });

  it("does not drive Arduino output pins while unpowered", () => {
    const engine = new SimEngine();
    engine.load(arduinoCircuit({ usb_power: 0 }));
    for (let i = 0; i < 120; i++) engine.step(0.01);
    expect(engine.getNetV()[pinNet(engine, "uno", "d13")]).toBeCloseTo(0, 6);
  });

  it("runs from an external 5V source when USB power is off", () => {
    const circuit = arduinoCircuit({ usb_power: 0 });
    circuit.components.push({
      id: "bench",
      kind: "voltage_source",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { voltage: 5 },
    });
    circuit.wires.push(
      { from_component: "bench", from_pin: "pos", to_component: "uno", to_pin: "5v" },
      { from_component: "bench", from_pin: "neg", to_component: "uno", to_pin: "gnd" },
    );

    const engine = new SimEngine();
    engine.load(circuit);
    const d13Net = pinNet(engine, "uno", "d13");
    const samples: number[] = [];
    for (let i = 0; i < 120; i++) {
      engine.step(0.01);
      samples.push(engine.getNetV()[d13Net] ?? 0);
    }
    expect(samples.some((v) => v > 4)).toBe(true);
    expect(samples.some((v) => v < 1)).toBe(true);
  });

  it("stamps INPUT_PULLUP as a weak connection to the real 5 V rail", () => {
    const engine = new SimEngine();
    engine.load(arduinoCircuit());
    const mcu = (engine as unknown as {
      state: { arduinos: Map<string, ArduinoMcu> };
    }).state.arduinos.get("uno");
    expect(mcu).toBeDefined();
    const cpu = (mcu as unknown as {
      cpu: { writeData(address: number, value: number): void };
    }).cpu;
    cpu.writeData(0x2a, 0x00);
    cpu.writeData(0x2b, 1 << 2);

    engine.step(1e-4);
    expect(engine.getNetV()[pinNet(engine, "uno", "d2")]).toBeGreaterThan(4.99);
  });

  it("forms the expected divider between INPUT_PULLUP and an external resistor", () => {
    const circuit = arduinoCircuit();
    circuit.components.push({
      id: "pulldown",
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 10_000 },
    });
    circuit.wires.push(
      { from_component: "uno", from_pin: "d2", to_component: "pulldown", to_pin: "a" },
      { from_component: "pulldown", from_pin: "b", to_component: "uno", to_pin: "gnd" },
    );

    const engine = new SimEngine();
    engine.load(circuit);
    const mcu = (engine as unknown as {
      state: { arduinos: Map<string, ArduinoMcu> };
    }).state.arduinos.get("uno");
    const cpu = (mcu as unknown as {
      cpu: { writeData(address: number, value: number): void };
    }).cpu;
    cpu.writeData(0x2a, 0x00);
    cpu.writeData(0x2b, 1 << 2);
    engine.step(1e-4);

    // 5 V × 10 kΩ / (35 kΩ + 10 kΩ) = 1.111... V.
    expect(engine.getNetV()[pinNet(engine, "uno", "d2")]).toBeCloseTo(5 / 4.5, 3);
  });
});

// ─── Arduino Nano board integration ──────────────────────────────────────

const NANO_PINS = ARDUINO_NANO_PINS.map((pin) => pin.id);

function nanoCircuit(params: Record<string, number | string> = {}): SimCircuit {
  return {
    components: [
      {
        id: "nano",
        kind: "arduino_nano",
        pins: NANO_PINS.map((id) => ({ id })),
        params: { vcc: 5, hex: blinkHex, usb_power: 1, ...params },
      },
    ],
    wires: [],
  };
}

describe("Arduino Nano board integration", () => {
  it("is classified as a 30-pin DIP", () => {
    expect(isDipIcKind("arduino_nano")).toBe(true);
  });

  it("is recognised as an Arduino board by the shared helper", () => {
    expect(isArduinoBoardKind("arduino_nano")).toBe(true);
    expect(isArduinoBoardKind("arduino_uno")).toBe(true);
    expect(isArduinoBoardKind("ne555")).toBe(false);
  });

  it("unions both on-board GND pins (and uses that union as the source-less reference)", () => {
    const nets = buildNets(nanoCircuit());
    const gnd = nets.find((net) => net.pins.some(([c, p]) => c === "nano" && p === "gnd"));
    expect(gnd?.id).toBe("gnd");
    expect(gnd?.pins).toContainEqual(["nano", "gnd2"]);
  });

  it("aliases both RESET pins (reset / reset2) onto the same net", () => {
    const nets = buildNets(nanoCircuit());
    const resetNet = nets.find((net) => net.pins.some(([c, p]) => c === "nano" && p === "reset"));
    expect(resetNet?.pins).toContainEqual(["nano", "reset2"]);
  });

  it("powers the 5V header from USB when USB power is on", () => {
    const engine = new SimEngine();
    engine.load(nanoCircuit({ usb_power: 1 }));
    expect(engine.getNetV()[pinNet(engine, "nano", "5v")]).toBeCloseTo(5, 4);
    expect(engine.getNetV()[pinNet(engine, "nano", "gnd")]).toBeCloseTo(0, 6);
  });

  it("blinks D13 with the same sketch the Uno uses (shared ATmega328P core)", () => {
    const engine = new SimEngine();
    engine.load(nanoCircuit({ usb_power: 1 }));
    const d13Net = pinNet(engine, "nano", "d13");
    const samples: number[] = [];
    for (let i = 0; i < 120; i++) {
      engine.step(0.01);
      samples.push(engine.getNetV()[d13Net] ?? 0);
    }
    expect(samples.some((v) => v > 4)).toBe(true);
    expect(samples.some((v) => v < 1)).toBe(true);
  });

  it("accepts setAnalogVolts on the Nano-only A6/A7 channels without throwing", () => {
    const mcu = new ArduinoMcu(blinkHex);
    expect(() => mcu.setAnalogVolts("a6", 1.0)).not.toThrow();
    expect(() => mcu.setAnalogVolts("a7", 4.2)).not.toThrow();
    // pinDriveState on analog-only pins is always "input" — they have no DDR path.
    expect(mcu.pinDriveState("a6")).toBe("input");
    expect(mcu.pinDriveState("a7")).toBe("input");
  });

  it("setAnalogVolts on a digital-only pin is a silent no-op", () => {
    const mcu = new ArduinoMcu(blinkHex);
    expect(() => mcu.setAnalogVolts("d5", 2.5)).not.toThrow();
    expect(() => mcu.setAnalogVolts("reset", 2.5)).not.toThrow();
  });
});
