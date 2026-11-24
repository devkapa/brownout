/**
 * A DHT22 or DHT11 module whose DATA shares a net with one of a board's pins (D2 here) answers the board on that pin,
 * and a real Adafruit DHT sketch (adafruit_dht22.ino / adafruit_dht11.ino beside their hex, built by the in-browser
 * avr-gcc) reads the humidity and temperature its parameters set. The library times each bit by counting loop
 * iterations with interrupts off, so this holds only if the bits reach the pin at the cycles the part drives them.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

const hex = (name: string) => readFileSync(join(__dirname, "__fixtures__", `${name}.hex`), "utf-8");

function circuit({ variant = 22, data = "d2", vcc = "5v" as string | null, params = {} as Record<string, number> } = {}): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex: hex(`adafruit_dht${variant}`), usb_power: 1 },
      },
      { id: "dht", kind: "dht", pins: [{ id: "vcc" }, { id: "data" }, { id: "gnd" }], params: { variant, ...params } },
    ],
    wires: [
      { from_component: "uno", from_pin: "gnd", to_component: "dht", to_pin: "gnd" },
      ...(vcc ? [{ from_component: "uno", from_pin: vcc, to_component: "dht", to_pin: "vcc" }] : []),
      { from_component: "uno", from_pin: data, to_component: "dht", to_pin: "data" },
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

const readings = (out: string) => [...out.matchAll(/H=(-?[\d.]+) T=(-?[\d.]+)/g)].map((m) => ({ h: Number(m[1]), t: Number(m[2]) }));

describe("DHT on a board pin", () => {
  it("reads a DHT22's humidity and temperature through Adafruit's DHT library, negative temperatures included", () => {
    const engine = new SimEngine();
    engine.load(circuit({ params: { humidity: 63.4, temperature: -7.8 } }));
    const got = readings(serial(engine, 4.5));
    expect(got).toEqual([{ h: 63.4, t: -7.8 }, { h: 63.4, t: -7.8 }]);
  });

  it("reads a DHT11 in whole percent and tenths of a degree", () => {
    const engine = new SimEngine();
    engine.load(circuit({ variant: 11, params: { humidity: 41.6, temperature: 23.4 } }));
    const got = readings(serial(engine, 2.3));
    expect(got).toEqual([{ h: 42, t: 23.4 }]);
  });

  it("reports the measurement it made after the previous reading, so a change shows one reading late", () => {
    const engine = new SimEngine();
    const c = circuit({ params: { humidity: 40, temperature: 20 } });
    engine.load(c);
    // First read at 2.1 s; the part measures 40 %RH again just after it.
    serial(engine, 2.3);
    c.components[1]!.params = { variant: 22, humidity: 80, temperature: 30 };
    engine.load(c);
    const got = readings(serial(engine, 4.3));
    expect(got).toEqual([{ h: 40, t: 20 }, { h: 80, t: 30 }]);
  });

  it("does not answer on another pin, or without power", () => {
    for (const c of [circuit({ data: "d3" }), circuit({ vcc: null })]) {
      const engine = new SimEngine();
      engine.load(c);
      const out = serial(engine, 2.3);
      expect(out).toContain("read failed");
      expect(readings(out)).toEqual([]);
    }
  });
});
