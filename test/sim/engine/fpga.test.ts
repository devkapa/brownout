/**
 * The iCE40 UP5K fabric running netlists from the in-browser Yosys (`synth_ice40 -top top -nobram; write_json`,
 * sources beside each .json): its LUTs, carry chain, flip-flop variants, I/O cells and oscillators, mapped to the
 * UPduino v3.1's pins by name or PCF.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { FpgaError, Ice40, parsePcf, type YosysJson } from "../../../src/sim/engine/fpga.js";

const fixture = (name: string) => join(__dirname, "__fixtures__", "fpga", name);
const netlist = (name: string): YosysJson => JSON.parse(readFileSync(fixture(`${name}.json`), "utf-8"));

/** A netlist of one module from cells and ports, for the error cases. */
function design(cells: YosysJson["modules"][string]["cells"], ports: YosysJson["modules"][string]["ports"] = {}): YosysJson {
  return { modules: { top: { ports, cells } } };
}

describe("Ice40", () => {
  it("counts the 12 MHz the HFOSC makes from 48 MHz divided by 4, its top bits sinking the RGB LED pads", () => {
    const f = new Ice40(netlist("blinky"));
    expect(f.pins).toEqual([2, 3, 39, 40, 41]);
    // Power-up: count 0, so every RGB pad is open and gpio_3 is count[3] ^ gpio_2.
    expect([f.pinDrive(39), f.pinDrive(40), f.pinDrive(41)]).toEqual([null, null, null]);
    f.setPin(2, 0);
    expect(f.pinDrive(3)).toBe(0);
    // 0.75 s: 9,000,000 rising edges, 0x895440: bit 23 (RGB0, green) set, 22 (blue) clear, 21 (red) clear.
    f.advance(0.75);
    const count = 9_000_000;
    expect(f.pinDrive(39)).toBe(0);
    expect(f.pinDrive(40)).toBe(null);
    expect(f.pinDrive(41)).toBe(null);
    expect(f.pinDrive(3)).toBe((count >> 3) & 1);
    f.setPin(2, 1);
    expect(f.pinDrive(3)).toBe(((count >> 3) & 1) ^ 1);
  });

  it("reads a pad through an SB_IO's pull-up and drives one only while its output enable is high", () => {
    const f = new Ice40(netlist("button"));
    expect(f.pullup(2)).toBe(true);
    // Released: the pull-up reads high, the LED is off. Pressed to ground: it lights.
    f.setPin(2, null);
    expect(f.pinDrive(4)).toBe(0);
    f.setPin(2, 0);
    expect(f.pinDrive(4)).toBe(1);
    f.setPin(9, 1);
    expect(f.pinDrive(6)).toBe(1);
    f.setPin(9, 0);
    expect(f.pinDrive(6)).toBe(null);
  });

  it("ripples a divider through flip-flops clocked by each other, held at zero by an asynchronous reset", () => {
    const f = new Ice40(netlist("divider"), { boardClocks: [{ pin: 20, hz: 12e6 }] });
    f.setPin(2, 0);
    const period = 1 / 12e6;
    // The clock rises at a half period and falls at a whole one; sample a quarter period after each fall. Each stage toggles on its predecessor's rising edge, so the chain counts
    // down: after k rising edges it holds -k mod 16.
    f.advance(period * 1.25);
    for (let k = 1; k <= 40; k++) {
      expect(f.pinDrive(4), `t0 after ${k}`).toBe(k & 1);
      expect(f.pinDrive(3), `t3 after ${k}`).toBe((-k & 15) >> 3);
      // The falling-edge register took t[0] at the fall just before this sample.
      expect(f.pinDrive(6), `n after ${k}`).toBe(k & 1);
      f.advance(period);
    }
    f.setPin(2, 1);
    expect([f.pinDrive(3), f.pinDrive(4)]).toEqual([0, 0]);
    f.advance(period * 10);
    expect([f.pinDrive(3), f.pinDrive(4)]).toEqual([0, 0]);
  });

  it("places ports by a PCF and runs a counter only while its clock enable is high, from its initial value", () => {
    const pcf = readFileSync(fixture("pcf_led.pcf"), "utf-8");
    expect(parsePcf(pcf).get("leds[2]")).toBe(44);
    const f = new Ice40(netlist("pcf_led"), { pcf, boardClocks: [{ pin: 20, hz: 1e6 }] });
    const leds = () => [3, 4, 44, 6].reduce((v, pin, i) => v | ((f.pinDrive(pin) ?? 0) << i), 0);
    expect(leds()).toBe(5);
    f.setPin(2, 0);
    f.advance(10.2e-6);
    expect(leds()).toBe(5);
    f.setPin(2, 1);
    // Rising edges at 10.5 us to 22.5 us: thirteen counts.
    f.advance(13e-6);
    expect(leds()).toBe((5 + 13) & 15);
  });

  it("names what it cannot run: an unknown cell, a port with no pin, a pin the board lacks, a combinational loop", () => {
    expect(() => new Ice40(design({ m: { type: "SB_MAC16", connections: {} } }))).toThrow(/not simulated yet: SB_MAC16/);
    expect(() => new Ice40(design({}, { blink: { direction: "output", bits: [2] } }))).toThrow(/port blink has no pin/);
    expect(() => new Ice40(design({}, { gpio_5: { direction: "output", bits: [2] } }))).toThrow(/pin 5, which the UPduino does not bring out/);
    const lut = (o: number, i: number) => ({ type: "SB_LUT4", parameters: { LUT_INIT: "0101010101010101" }, connections: { O: [o], I0: [i], I1: ["0"], I2: ["0"], I3: ["0"] } });
    expect(() => new Ice40(design({ a: lut(2, 3), b: lut(3, 2) }))).toThrow(FpgaError);
    expect(() => new Ice40(design({ a: lut(2, 3), b: lut(3, 2) }))).toThrow(/combinational loop/);
  });
});
