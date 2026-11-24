import { describe, expect, it } from "vitest";
import {
  ARDUINO_RESET_BUTTON_BOUNDS,
  ARDUINO_UNO_BOUNDS,
  ARDUINO_UNO_HEIGHT,
  ARDUINO_UNO_PINS,
  ARDUINO_UNO_WIDTH,
  ARDUINO_USB_TOGGLE_BOUNDS,
  arduinoUnoAabb,
} from "../../src/circuit/arduino.js";

// The host draws parts at real size: 19 px between breadboard holes, 0.1".
const PX_PER_MM = 19 / 2.54;
// Arduino Uno R3 reference design: 68.58 x 53.34 mm (2.7 x 2.1 in).
const UNO_W_MM = 68.58;
const UNO_H_MM = 53.34;

describe("Arduino Uno geometry is real size", () => {
  it("the PCB is 68.58 x 53.34 mm", () => {
    expect(ARDUINO_UNO_WIDTH / PX_PER_MM).toBeCloseTo(UNO_W_MM, 0);
    expect(ARDUINO_UNO_HEIGHT / PX_PER_MM).toBeCloseTo(UNO_H_MM, 0);
  });

  it("the header pitch is about 2.54 mm", () => {
    const d13 = ARDUINO_UNO_PINS.find((p) => p.id === "d13")!;
    const d8 = ARDUINO_UNO_PINS.find((p) => p.id === "d8")!;
    const pitchMm = (d8.offset.x - d13.offset.x) / 5 / PX_PER_MM;
    expect(pitchMm).toBeGreaterThan(2.54 * 0.95);
    expect(pitchMm).toBeLessThan(2.54 * 1.05);
  });

  it("every pin, the USB plug and the reset button lie inside the bounds", () => {
    const b = ARDUINO_UNO_BOUNDS;
    const inside = (x: number, y: number) => x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h;
    for (const pin of ARDUINO_UNO_PINS) expect(inside(pin.offset.x, pin.offset.y), pin.id).toBe(true);
    for (const r of [ARDUINO_USB_TOGGLE_BOUNDS, ARDUINO_RESET_BUTTON_BOUNDS]) {
      expect(inside(r.x, r.y)).toBe(true);
      expect(inside(r.x + r.w, r.y + r.h)).toBe(true);
    }
    expect(b.w).toBeCloseTo(ARDUINO_UNO_WIDTH - b.x, 6);
    expect(arduinoUnoAabb({ x: 10, y: 20 })).toEqual({ x: 10 + b.x, y: 20 + b.y, w: b.w, h: b.h });
  });

  it("pins keep the layout of the board art: power header left of analog, digital above both", () => {
    const y = (id: string) => ARDUINO_UNO_PINS.find((p) => p.id === id)!.offset.y;
    const x = (id: string) => ARDUINO_UNO_PINS.find((p) => p.id === id)!.offset.x;
    expect(y("d0")).toBeLessThan(y("a0"));
    expect(x("vin")).toBeLessThan(x("a0"));
    // Top header sits about 2.5 mm in from the board edge, bottom about the same.
    expect(y("d0") / PX_PER_MM).toBeLessThan(4);
    expect((ARDUINO_UNO_HEIGHT - y("a0")) / PX_PER_MM).toBeLessThan(4);
  });
});
