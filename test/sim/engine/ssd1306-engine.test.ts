/**
 * An SSD1306 OLED module wired to a board's I2C pins (A4 SDA, A5 SCL on the Uno) answers the board's TWI, and the
 * engine reports what its panel shows. The firmware is bare-metal C beside its hex, driving the panel as
 * Adafruit_SSD1306 does.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

const hex = (name: string) => readFileSync(join(__dirname, "__fixtures__", `${name}.hex`), "utf-8");

function circuit({ sda = "a4", scl = "a5", vcc = "5v" as string | null, firmware = "ssd1306_hello" } = {}): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex: hex(firmware), usb_power: 1 },
      },
      { id: "oled", kind: "ssd1306", pins: [{ id: "gnd" }, { id: "vcc" }, { id: "scl" }, { id: "sda" }], params: {} },
    ],
    wires: [
      { from_component: "uno", from_pin: "gnd", to_component: "oled", to_pin: "gnd" },
      ...(vcc ? [{ from_component: "uno", from_pin: vcc, to_component: "oled", to_pin: "vcc" }] : []),
      { from_component: "uno", from_pin: scl, to_component: "oled", to_pin: "scl" },
      { from_component: "uno", from_pin: sda, to_component: "oled", to_pin: "sda" },
    ],
  };
}

function run(c: SimCircuit, seconds = 0.06) {
  const engine = new SimEngine();
  engine.load(c);
  for (let t = 0; t < seconds; t += 0.001) engine.step(0.001);
  return engine;
}

const lit = (pixels: Uint8Array, x: number, y: number) => (pixels[y * 16 + (x >> 3)]! >> (7 - (x & 7))) & 1;

describe("SSD1306 on a board's I2C bus", () => {
  it("acknowledges the board and shows the frame it was sent", () => {
    const engine = run(circuit());
    const display = engine.getDisplayState().oled;
    expect(display).toMatchObject({ kind: "ssd1306", on: true, width: 128, height: 64 });
    if (display?.kind !== "ssd1306") return;
    expect(lit(display.pixels, 0, 0)).toBe(1);
    expect(lit(display.pixels, 5, 7)).toBe(1);
    expect(lit(display.pixels, 127, 9)).toBe(1);
    expect(display.pixels.reduce((n, b) => n + popcount(b), 0)).toBe(3);
    const i2c = engine.drainMcuBusEvents().uno!.filter((e) => e.bus === "i2c" && e.kind === "address");
    expect(i2c.length).toBe(3);
    expect(i2c.every((e) => e.bus === "i2c" && e.kind === "address" && e.ack)).toBe(true);
  });

  it("is not on the bus when its SDA and SCL go elsewhere", () => {
    const engine = run(circuit({ sda: "d2", scl: "d3" }));
    const display = engine.getDisplayState().oled;
    expect(display).toMatchObject({ kind: "ssd1306", on: false });
    const addresses = engine.drainMcuBusEvents().uno!.filter((e) => e.bus === "i2c" && e.kind === "address");
    expect(addresses.every((e) => e.bus === "i2c" && e.kind === "address" && !e.ack)).toBe(true);
  });

  it("shows what an Adafruit_SSD1306 sketch draws, built by the in-browser avr-gcc", () => {
    // adafruit_ssd1306.ino beside the hex: Adafruit_SSD1306 2.5.13 and Adafruit_GFX 1.12.1 at their real init,
    // a border, a filled block, one pixel and a line of text, then D13 high once display() has returned.
    const engine = run(circuit({ firmware: "adafruit_ssd1306" }), 0.4);
    const display = engine.getDisplayState().oled;
    expect(display).toMatchObject({ kind: "ssd1306", on: true, width: 128, height: 64 });
    if (display?.kind !== "ssd1306") return;
    for (const [x, y] of [[0, 0], [127, 0], [0, 63], [127, 63], [64, 0], [0, 32], [127, 32], [64, 63]] as const) {
      expect(lit(display.pixels, x, y), `border ${x},${y}`).toBe(1);
    }
    for (let y = 40; y < 56; y++) for (let x = 100; x < 120; x++) expect(lit(display.pixels, x, y), `block ${x},${y}`).toBe(1);
    expect(lit(display.pixels, 64, 32)).toBe(1);
    expect(lit(display.pixels, 63, 32) + lit(display.pixels, 65, 32) + lit(display.pixels, 64, 31) + lit(display.pixels, 64, 33)).toBe(0);
    let text = 0;
    for (let y = 4; y < 12; y++) for (let x = 4; x < 4 + 7 * 6; x++) text += lit(display.pixels, x, y);
    expect(text).toBeGreaterThan(40);
    const border = 2 * 128 + 2 * 62;
    const total = display.pixels.reduce((n, b) => n + popcount(b), 0);
    expect(total).toBe(border + 20 * 16 + 1 + text);
  });

  it("stays dark and off the bus without power", () => {
    const engine = run(circuit({ vcc: null }));
    expect(engine.getDisplayState().oled).toMatchObject({ kind: "ssd1306", on: false });
  });

  it("takes the panel's supply from its parameter, building the module again only when that changes", () => {
    const engine = new SimEngine();
    const parts = (engine as unknown as { _boardParts: Map<string, { externalVcc: boolean }> })._boardParts;
    const withSupply = (supply?: string) => {
      const c = circuit();
      c.components[1]!.params = supply ? { supply } : {};
      return c;
    };
    engine.load(withSupply());
    const pumped = parts.get("oled")!;
    expect(pumped.externalVcc).toBe(false);
    engine.load(withSupply());
    expect(parts.get("oled")).toBe(pumped);
    engine.load(withSupply("external"));
    expect(parts.get("oled")!.externalVcc).toBe(true);
  });
});

function popcount(b: number): number {
  let n = 0;
  for (; b; b &= b - 1) n++;
  return n;
}
