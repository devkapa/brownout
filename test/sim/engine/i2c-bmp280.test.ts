/**
 * The BMP280 as a sketch sees it over I2C: its id and trimming parameters, conversions that take the datasheet's
 * measurement time in forced and normal mode, and raw readings that the datasheet's own compensation (written out
 * again here from section 8.2, not imported) turns back into the ambient values, at the oversampling's resolution.
 */
import { describe, expect, it } from "vitest";
import { Bmp280 } from "../../../src/sim/engine/bmp280.js";

// The datasheet's worked example (section 8.2): these trimming parameters, then adc_T 519888 and adc_P 415148
// compensate to 25.08 degC and 100653.27 Pa.
const TRIM = { T1: 27504, T2: 26435, T3: -1000, P1: 36477, P2: -10685, P3: 3024, P4: 2855, P5: 140, P6: -7, P7: 15500, P8: -14600, P9: 6000 };

function compensate(adcT: number, adcP: number): { celsius: number; pascals: number } {
  const var1 = (((adcT >> 3) - (TRIM.T1 << 1)) * TRIM.T2) >> 11;
  const var2 = (((((adcT >> 4) - TRIM.T1) * ((adcT >> 4) - TRIM.T1)) >> 12) * TRIM.T3) >> 14;
  const tFine = var1 + var2;
  let v1 = BigInt(tFine) - 128000n;
  let v2 = v1 * v1 * BigInt(TRIM.P6) + ((v1 * BigInt(TRIM.P5)) << 17n) + (BigInt(TRIM.P4) << 35n);
  v1 = ((v1 * v1 * BigInt(TRIM.P3)) >> 8n) + ((v1 * BigInt(TRIM.P2)) << 12n);
  v1 = (((1n << 47n) + v1) * BigInt(TRIM.P1)) >> 33n;
  let p = 1048576n - BigInt(adcP);
  p = (((p << 31n) - v2) * 3125n) / v1;
  const v3 = (BigInt(TRIM.P9) * (p >> 13n) * (p >> 13n)) >> 25n;
  const v4 = (BigInt(TRIM.P8) * p) >> 19n;
  p = ((p + v3 + v4) >> 8n) + (BigInt(TRIM.P7) << 4n);
  return { celsius: ((tFine * 5 + 128) >> 8) / 100, pascals: Number(p) / 256 };
}

/** A sensor on a bus whose clock the test sets. */
function sensor(address = 0x77) {
  const d = new Bmp280({ address });
  d.powered = true;
  let t = 0;
  d.clock = () => t;
  return { d, at: (seconds: number) => (t = seconds) };
}

function writeRegisters(d: Bmp280, ...pairs: number[]): void {
  d.start();
  for (const b of pairs) expect(d.write(b)).toBe(true);
  d.stop();
}

function readRegisters(d: Bmp280, register: number, count: number): number[] {
  d.start();
  d.write(register);
  d.stop();
  d.start();
  const out = Array.from({ length: count }, () => d.read());
  d.stop();
  return out;
}

function readings(d: Bmp280): { adcT: number; adcP: number } {
  const b = readRegisters(d, 0xf7, 6);
  return { adcP: (b[0]! << 12) | (b[1]! << 4) | (b[2]! >> 4), adcT: (b[3]! << 12) | (b[4]! << 4) | (b[5]! >> 4) };
}

describe("Bmp280", () => {
  it("compensates the datasheet's worked example as the datasheet does", () => {
    const { celsius, pascals } = compensate(519888, 415148);
    expect(celsius).toBe(25.08);
    expect(pascals).toBeCloseTo(100653.27, 0);
  });

  it("answers chip id 0x58 and carries the trimming parameters little-endian from 88h", () => {
    const { d } = sensor();
    expect(readRegisters(d, 0xd0, 1)).toEqual([0x58]);
    const bytes = readRegisters(d, 0x88, 24);
    const word = (i: number, signed: boolean) => {
      const v = bytes[i * 2]! | (bytes[i * 2 + 1]! << 8);
      return signed && v >= 0x8000 ? v - 0x10000 : v;
    };
    expect([word(0, false), word(1, true), word(2, true), word(3, false)]).toEqual([TRIM.T1, TRIM.T2, TRIM.T3, TRIM.P1]);
    expect(word(11, true)).toBe(TRIM.P9);
  });

  it("reads 80000h for both values out of reset, as no conversion has run", () => {
    const { d } = sensor();
    expect(readings(d)).toEqual({ adcT: 0x80000, adcP: 0x80000 });
  });

  it("makes one forced conversion in the typical measurement time, then sleeps", () => {
    const { d, at } = sensor();
    d.setAmbient(21.5, 98765.4);
    at(1);
    // osrs_t x1, osrs_p x1, forced: 1 + 2 + 2.5 = 5.5 ms.
    writeRegisters(d, 0xf4, 0b001_001_01);
    expect(readRegisters(d, 0xf3, 1)[0]! & 0x08).toBe(0x08);
    at(1.0054);
    expect(readings(d)).toEqual({ adcT: 0x80000, adcP: 0x80000 });
    at(1.0056);
    expect(readRegisters(d, 0xf3, 1)[0]! & 0x08).toBe(0);
    expect(readRegisters(d, 0xf4, 1)[0]! & 3).toBe(0);
    const { adcT, adcP } = readings(d);
    // 16-bit results at x1: the low four bits of the 20 are zero.
    expect(adcT & 0xf).toBe(0);
    expect(adcP & 0xf).toBe(0);
    const out = compensate(adcT, adcP);
    expect(Math.abs(out.celsius - 21.5)).toBeLessThanOrEqual(0.01);
    expect(Math.abs(out.pascals - 98765.4)).toBeLessThan(3);
  });

  it("carries on with the conversion in progress when a sketch asks for forced mode again mid-way", () => {
    const { d, at } = sensor();
    d.setAmbient(21.5, 98765.4);
    at(1);
    writeRegisters(d, 0xf4, 0b001_001_01);
    at(1.003);
    // The part is busy: the second request neither restarts nor extends the 5.5 ms conversion.
    writeRegisters(d, 0xf4, 0b101_101_01);
    expect(readRegisters(d, 0xf4, 1)[0]).toBe(0b001_001_01);
    at(1.0056);
    expect(readRegisters(d, 0xf3, 1)[0]! & 0x08).toBe(0);
    expect(readRegisters(d, 0xf4, 1)[0]! & 3).toBe(0);
    expect(readings(d).adcT).not.toBe(0x80000);
  });

  it("resolves to 0.01 degC and a fraction of a pascal at x16", () => {
    const { d, at } = sensor();
    d.setAmbient(-12.34, 85000.5);
    writeRegisters(d, 0xf4, 0b101_101_01);
    at(0.1);
    const { adcT, adcP } = readings(d);
    const out = compensate(adcT, adcP);
    expect(out.celsius).toBe(-12.34);
    expect(Math.abs(out.pascals - 85000.5)).toBeLessThan(0.2);
  });

  it("skips a measurement whose oversampling is off, reading 80000h for it", () => {
    const { d, at } = sensor();
    writeRegisters(d, 0xf4, 0b001_000_01);
    at(0.01);
    const { adcT, adcP } = readings(d);
    expect(adcP).toBe(0x80000);
    expect(adcT).not.toBe(0x80000);
  });

  it("repeats conversions in normal mode after each standby, so a new ambient value shows at the next one", () => {
    const { d, at } = sensor();
    d.setAmbient(20, 101325);
    // t_sb 62.5 ms, then osrs_t x1, osrs_p x1, normal: a conversion every 5.5 + 62.5 ms.
    writeRegisters(d, 0xf5, 0b001_000_00, 0xf4, 0b001_001_11);
    at(0.006);
    expect(compensate(readings(d).adcT, 0x80000).celsius).toBeCloseTo(20, 1);
    d.setAmbient(30, 101325);
    at(0.07);
    expect(compensate(readings(d).adcT, 0x80000).celsius).toBeCloseTo(20, 1);
    at(0.074);
    expect(compensate(readings(d).adcT, 0x80000).celsius).toBeCloseTo(30, 1);
    expect(readRegisters(d, 0xf4, 1)[0]! & 3).toBe(3);
  });

  it("smooths a step with the IIR filter, each sample moving a quarter of the way at coefficient 4", () => {
    const { d, at } = sensor();
    d.setAmbient(25, 100000);
    writeRegisters(d, 0xf5, 0b000_010_00, 0xf4, 0b001_101_11);
    const period = 0.0005 + (1 + 2 + 32.5) / 1000;
    at(period * 3);
    const settled = compensate(readings(d).adcT, readings(d).adcP).pascals;
    expect(settled).toBeCloseTo(100000, 0);
    d.setAmbient(25, 101000);
    at(period * 4 + 0.0001);
    const first = compensate(readings(d).adcT, readings(d).adcP).pascals;
    expect(first).toBeGreaterThan(100200);
    expect(first).toBeLessThan(100300);
    at(period * 60);
    expect(compensate(readings(d).adcT, readings(d).adcP).pascals).toBeCloseTo(101000, 0);
  });

  it("keeps a burst read to one conversion even when the next one lands mid-read", () => {
    const { d, at } = sensor();
    d.setAmbient(10, 101325);
    writeRegisters(d, 0xf5, 0, 0xf4, 0b001_001_11);
    at(0.0056);
    d.setAmbient(40, 101325);
    d.start();
    d.write(0xf7);
    d.stop();
    d.start();
    const head = [d.read(), d.read(), d.read()];
    at(0.0125);
    const tail = [d.read(), d.read(), d.read()];
    d.stop();
    const adcT = (tail[0]! << 12) | (tail[1]! << 4) | (tail[2]! >> 4);
    expect(head.length).toBe(3);
    expect(compensate(adcT, 0x80000).celsius).toBeCloseTo(10, 1);
  });

  it("returns every register to reset on the B6h soft reset, and ignores other words", () => {
    const { d, at } = sensor();
    writeRegisters(d, 0xf5, 0xa0, 0xf4, 0x27);
    at(0.1);
    writeRegisters(d, 0xe0, 0x12);
    expect(readRegisters(d, 0xf4, 2)).toEqual([0x27, 0xa0]);
    writeRegisters(d, 0xe0, 0xb6);
    expect(readRegisters(d, 0xf4, 2)).toEqual([0, 0]);
    expect(readings(d)).toEqual({ adcT: 0x80000, adcP: 0x80000 });
  });

  it("finishes a conversion a board started before it reset", () => {
    const { d, at } = sensor();
    at(5);
    writeRegisters(d, 0xf4, 0b001_001_01);
    at(0.001);
    expect(readings(d).adcT).not.toBe(0x80000);
  });
});
