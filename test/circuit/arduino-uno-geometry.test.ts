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

  // Uno R3 drawing: every header is on a 0.1" grid except the 160 mil step
  // between D7 and D8; the power header's VIN sits under D7 and A0 two pitches
  // right of VIN.
  const at = (id: string) => ARDUINO_UNO_PINS.find((p) => p.id === id)!.offset;
  const run = (ids: string[]) => {
    for (let i = 1; i < ids.length; i++) {
      expect(at(ids[i]).x - at(ids[i - 1]).x, `${ids[i - 1]} to ${ids[i]}`).toBeCloseTo(19, 6);
      expect(at(ids[i]).y).toBe(at(ids[0]).y);
    }
  };

  it("adjacent header pins are exactly 19 px (2.54 mm) apart", () => {
    run(["d13", "d12", "d11", "d10", "d9", "d8"]);
    run(["d7", "d6", "d5", "d4", "d3", "d2", "d1", "d0"]);
    run(["io_ref", "reset", "3v3", "5v", "gnd", "gnd2", "vin"]);
    run(["a0", "a1", "a2", "a3", "a4", "a5"]);
    // AREF is two holes left of D13 (the top GND sits between them).
    expect(at("d13").x - at("aref").x).toBeCloseTo(38, 6);
  });

  it("D7 to D8 is the R3 header's 160 mil step", () => {
    expect(at("d7").x - at("d8").x).toBeCloseTo(1.6 * 19, 6);
  });

  it("the analog header starts two pitches right of VIN", () => {
    expect(at("a0").x - at("vin").x).toBeCloseTo(38, 6);
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
