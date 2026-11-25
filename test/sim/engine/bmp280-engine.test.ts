/**
 * A BMP280 module wired to a board's I2C pins (A4 SDA, A5 SCL on the Uno) answers the board's TWI, and a real
 * Adafruit_BMP280 sketch (adafruit_bmp280.ino beside its hex, built by the in-browser avr-gcc) reads the ambient
 * temperature and pressure its parameters set, at the resolution the library's x16 oversampling gives.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

const hex = readFileSync(join(__dirname, "__fixtures__", "adafruit_bmp280.hex"), "utf-8");

function circuit({ sda = "a4", vcc = "5v" as string | null, params = {} as Record<string, number> } = {}): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex, usb_power: 1 },
      },
      { id: "bmp", kind: "bmp280", pins: [{ id: "vcc" }, { id: "gnd" }, { id: "scl" }, { id: "sda" }], params },
    ],
    wires: [
      { from_component: "uno", from_pin: "gnd", to_component: "bmp", to_pin: "gnd" },
      ...(vcc ? [{ from_component: "uno", from_pin: vcc, to_component: "bmp", to_pin: "vcc" }] : []),
      { from_component: "uno", from_pin: "a5", to_component: "bmp", to_pin: "scl" },
      { from_component: "uno", from_pin: sda, to_component: "bmp", to_pin: "sda" },
    ],
  };
}

function serial(engine: SimEngine, seconds: number): string {
  let out = "";
  for (let t = 0; t < seconds; t += 0.001) {
    engine.step(0.001);
    out += engine.drainSerialOutput().uno ?? "";
  }
  return out;
}

function lines(out: string): { t: number; p: number }[] {
  return [...out.matchAll(/T=(-?[\d.]+) P=([\d.]+)/g)].map((m) => ({ t: Number(m[1]), p: Number(m[2]) }));
}

describe("BMP280 on a board's I2C bus", () => {
  it("reads the ambient temperature and pressure through Adafruit_BMP280", () => {
    const engine = new SimEngine();
    engine.load(circuit({ params: { temperature: 23.45, pressure: 987.65 } }));
    const got = lines(serial(engine, 0.5));
    expect(got.length).toBeGreaterThanOrEqual(3);
    // The first read can land before the first x16 conversion ends (about 66 ms), as on the bench.
    for (const { t, p } of got.slice(1)) {
      expect(t).toBe(23.45);
      expect(Math.abs(p - 98765)).toBeLessThanOrEqual(0.5);
    }
  });

  it("follows a parameter changed while the sketch runs, at the next conversion", () => {
    const engine = new SimEngine();
    const c = circuit({ params: { temperature: 20, pressure: 1013.25 } });
    engine.load(c);
    serial(engine, 0.3);
    c.components[1]!.params = { temperature: -5.5, pressure: 900 };
    engine.load(c);
    serial(engine, 0.15);
    const after = lines(serial(engine, 0.3));
    expect(after.length).toBeGreaterThanOrEqual(2);
    for (const { t, p } of after) {
      expect(t).toBe(-5.5);
      expect(Math.abs(p - 90000)).toBeLessThanOrEqual(0.5);
    }
  });

  it("is not found when its SDA goes elsewhere, or without power", () => {
    for (const c of [circuit({ sda: "d2" }), circuit({ vcc: null })]) {
      const engine = new SimEngine();
      engine.load(c);
      expect(serial(engine, 0.1)).toContain("no BMP280");
    }
  });
});
