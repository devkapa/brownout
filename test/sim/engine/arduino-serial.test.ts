/**
 * The ATmega328P's USART, TWI, SPI and EEPROM. Arduino's Serial, Wire, SPI and EEPROM libraries busy-wait on these
 * peripherals' flags, so a sketch using any of them stalls on a board that lacks them.
 *
 * Fixtures: the .ino beside each .hex, built with avr-gcc 7.3.0 (Atmel 3.7.0) against ArduinoCore-avr 1.8.6 with
 * the IDE's Uno flags (-Os, -mmcu=atmega328p, F_CPU=16000000L, no LTO).
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { ArduinoMcu } from "../../../src/sim/engine/arduino.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

const hex = (name: string) => readFileSync(join(__dirname, "__fixtures__", `${name}.hex`), "utf-8");
const text = (bytes: Uint8Array) => new TextDecoder("latin1").decode(bytes);

function run(mcu: ArduinoMcu, seconds: number, step = 0.001): string {
  let out = "";
  for (let t = 0; t < seconds; t += step) {
    mcu.step(step);
    out += text(mcu.drainSerialOutput());
  }
  return out;
}

describe("ArduinoMcu USART", () => {
  it("transmits what Serial prints, at the baud rate, without stalling the sketch", () => {
    const mcu = new ArduinoMcu(hex("serial_counter"));
    let toggles = 0;
    let last = mcu.pinDriveState("d13");
    let out = "";
    for (let t = 0; t < 1; t += 0.001) {
      mcu.step(0.001);
      out += text(mcu.drainSerialOutput());
      const now = mcu.pinDriveState("d13");
      if (now !== last) toggles++;
      last = now;
    }
    expect(out.startsWith("count 0\r\ncount 1\r\ncount 2\r\n")).toBe(true);
    // 9600 baud is 960 characters a second; "count NN\r\n" is 10 of them.
    expect(out.length).toBeGreaterThan(900);
    expect(out.length).toBeLessThanOrEqual(961);
    expect(toggles).toBeGreaterThan(80);
  });

  it("delivers typed bytes to Serial.read()", () => {
    const mcu = new ArduinoMcu(hex("serial_echo"));
    run(mcu, 0.05);
    mcu.writeSerialInput(new TextEncoder().encode("hello, uno"));
    expect(run(mcu, 0.05)).toBe("HELLO, UNO");
  });

  it("starts the transmit log empty again after a reset", () => {
    const mcu = new ArduinoMcu(hex("serial_counter"));
    run(mcu, 0.05);
    mcu.reset();
    expect(run(mcu, 0.05).startsWith("count 0\r\n")).toBe(true);
  });
});

describe("ArduinoMcu EEPROM, TWI and SPI", () => {
  it("lets EEPROM, Wire and SPI calls complete", () => {
    const mcu = new ArduinoMcu(hex("peripherals_ok"));
    const out = run(mcu, 0.2);
    // EEPROM keeps both bytes; nothing answers on I2C (endTransmission 2 = address NACK); SPI with nothing on MISO
    // reads back 0, avr8js's default onTransfer answer.
    expect(out).toBe("eeprom 42,43 i2c 2 spi 0\r\n");
  });

  it("keeps EEPROM contents across a reset, as the real chip does", () => {
    const mcu = new ArduinoMcu(hex("peripherals_ok"));
    run(mcu, 0.2);
    mcu.reset();
    expect(mcu.readEeprom().slice(0, 2)).toEqual(new Uint8Array([42, 43]));
  });
});

describe("SimEngine serial", () => {
  const board = (name: string): SimCircuit => ({
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex: hex(name), usb_power: 1 },
      },
    ],
    wires: [],
  });

  it("reports each board's serial output once, and nothing when it is quiet", () => {
    const engine = new SimEngine();
    engine.load(board("serial_counter"));
    let out = "";
    for (let i = 0; i < 20; i++) {
      engine.step(0.005);
      out += engine.drainSerialOutput().uno ?? "";
    }
    expect(out.startsWith("count 0\r\ncount 1\r\n")).toBe(true);
    expect(engine.drainSerialOutput()).toEqual({});
  });

  it("types into a board's serial port", () => {
    const engine = new SimEngine();
    engine.load(board("serial_echo"));
    for (let i = 0; i < 10; i++) engine.step(0.005);
    engine.drainSerialOutput();
    engine.writeSerialInput("uno", "abc\n");
    let out = "";
    for (let i = 0; i < 10; i++) {
      engine.step(0.005);
      out += engine.drainSerialOutput().uno ?? "";
    }
    expect(out).toBe("ABC\n");
  });
});
