/**
 * The SSD1306 OLED controller as an I2C device (src/sim/engine/i2c-devices.ts), driven byte by byte as a sketch's
 * Wire calls would: control bytes, the command set Adafruit_SSD1306 and U8g2 send, and GDDRAM writes in each
 * addressing mode. Expected images follow the SSD1306 datasheet (rev 1.1), sections 8.7 and 10.
 */
import { describe, expect, it } from "vitest";
import { Ssd1306 } from "../../../src/sim/engine/i2c-devices.js";

// One I2C write transaction: address already acknowledged, then these bytes, then STOP.
function send(d: Ssd1306, bytes: number[]): boolean[] {
  d.start();
  const acks = bytes.map((b) => d.write(b));
  d.stop();
  return acks;
}
const commands = (d: Ssd1306, ...c: number[]) => send(d, [0x00, ...c]);
const data = (d: Ssd1306, ...b: number[]) => send(d, [0x40, ...b]);

// Adafruit_SSD1306::begin for a 128x64 panel with the internal charge pump.
const ADAFRUIT_INIT_64 = [
  0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00, 0x40, 0x8d, 0x14, 0x20, 0x00, 0xa1, 0xc8, 0xda, 0x12, 0x81, 0xcf, 0xd9,
  0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0x2e, 0xaf,
];

const pixel = (d: Ssd1306, x: number, y: number) => {
  const img = d.image();
  return (img.pixels[y * (img.width / 8) + (x >> 3)]! >> (7 - (x & 7))) & 1;
};

describe("Ssd1306", () => {
  it("answers at its address and stays dark until a sketch turns the panel and its charge pump on", () => {
    const d = new Ssd1306({ address: 0x3c, height: 64 });
    expect(d.address).toBe(0x3c);
    expect(d.image().on).toBe(false);
    commands(d, 0xaf); // display on, but the charge pump is still off after reset
    expect(d.image().on).toBe(false);
    commands(d, 0x8d, 0x14);
    expect(d.image().on).toBe(true);
  });

  it("draws GDDRAM bytes as vertical 8-pixel columns in horizontal addressing, upright under Adafruit's init", () => {
    const d = new Ssd1306({ address: 0x3c, height: 64 });
    commands(d, ...ADAFRUIT_INIT_64);
    commands(d, 0x21, 0, 127, 0x22, 0, 7);
    // Byte 0 of page 0 is column 0, rows 0-7, LSB at the top; byte 128 starts page 1 (rows 8-15).
    const frame = new Array(1024).fill(0);
    frame[0] = 0b0000_0001; // (0, 0)
    frame[5] = 0b1000_0000; // (5, 7)
    frame[128 + 127] = 0b0000_0010; // (127, 9)
    data(d, ...frame);
    expect(pixel(d, 0, 0)).toBe(1);
    expect(pixel(d, 5, 7)).toBe(1);
    expect(pixel(d, 127, 9)).toBe(1);
    expect(pixel(d, 1, 0)).toBe(0);
    expect(d.image().pixels.reduce((n, b) => n + popcount(b), 0)).toBe(3);
  });

  it("mirrors and flips with the segment remap and COM scan direction a sketch leaves at reset", () => {
    const d = new Ssd1306({ address: 0x3c, height: 64 });
    commands(d, 0x8d, 0x14, 0xaf, 0x20, 0x00); // reset orientation: A0, C0
    data(d, 0b0000_0001); // RAM (0, 0)
    expect(pixel(d, 127, 63)).toBe(1);
    expect(pixel(d, 0, 0)).toBe(0);
  });

  it("follows page addressing: B0-B7 pick the page, 00-1F the column, and the pointer stays on its page", () => {
    const d = new Ssd1306({ address: 0x3c, height: 64 });
    commands(d, ...ADAFRUIT_INIT_64, 0x20, 0x02, 0xb3, 0x05, 0x12); // page 3, column 0x25
    data(d, 0xff, 0x01);
    for (let y = 24; y < 32; y++) expect(pixel(d, 0x25, y)).toBe(1);
    expect(pixel(d, 0x26, 24)).toBe(1);
    expect(pixel(d, 0x26, 25)).toBe(0);
  });

  it("reads a byte with Co set as a single command and the next control byte decides what follows", () => {
    const d = new Ssd1306({ address: 0x3c, height: 64 });
    commands(d, ...ADAFRUIT_INIT_64, 0x21, 0, 127, 0x22, 0, 7);
    // 0x80 = Co 1, D/C 0: one command (A7, invert); then 0x40 starts data.
    send(d, [0x80, 0xa7, 0x40, 0x01]);
    expect(pixel(d, 0, 0)).toBe(0); // inverted: the lit RAM bit shows dark
    expect(pixel(d, 1, 0)).toBe(1);
  });

  it("shows only the multiplexed rows, from the display start line", () => {
    const d = new Ssd1306({ address: 0x3c, height: 64 });
    commands(d, ...ADAFRUIT_INIT_64, 0x21, 0, 127, 0x22, 0, 7);
    data(d, 0b0000_0011); // RAM rows 0 and 1 of column 0
    commands(d, 0x41); // start line 1: RAM row 1 is on the top line
    expect(pixel(d, 0, 0)).toBe(1);
    expect(pixel(d, 0, 1)).toBe(0);
    commands(d, 0xa8, 0x1f); // 32-row multiplex on a 64-row panel: the lower half is dark
    commands(d, 0xa5); // entire display on
    expect(pixel(d, 10, 31)).toBe(1);
    expect(pixel(d, 10, 32)).toBe(0);
  });

  it("parses multi-byte commands so their arguments are never drawn or taken as commands", () => {
    const d = new Ssd1306({ address: 0x3c, height: 64 });
    // Scroll setup carries 6 arguments, one of them 0xAE (display off) were it a command.
    commands(d, ...ADAFRUIT_INIT_64, 0x26, 0x00, 0x00, 0xae, 0x07, 0x00, 0xff);
    expect(d.image().on).toBe(true);
  });

  it("reports its status on a read: bit 6 set while the display is off", () => {
    const d = new Ssd1306({ address: 0x3c, height: 64 });
    expect(d.read() & 0x40).toBe(0x40);
    commands(d, 0xaf);
    expect(d.read() & 0x40).toBe(0);
  });

  it("starts from reset again when power returns, with GDDRAM cleared", () => {
    const d = new Ssd1306({ address: 0x3c, height: 32 });
    commands(d, 0x8d, 0x14, 0xaf, 0xa5);
    expect(d.image()).toMatchObject({ on: true, width: 128, height: 32 });
    d.reset();
    expect(d.image().on).toBe(false);
  });

  it("hands back the same image until a byte changes the panel, so a host can skip an unchanged frame", () => {
    const d = new Ssd1306({ address: 0x3c, height: 64 });
    commands(d, ...ADAFRUIT_INIT_64);
    const first = d.image();
    expect(d.image()).toBe(first);
    data(d, 0x01);
    const drawn = d.image();
    expect(drawn).not.toBe(first);
    expect(pixel(d, 0, 0)).toBe(1);
    commands(d, 0xa7);
    expect(d.image()).not.toBe(drawn);
    const inverted = d.image();
    d.reset();
    expect(d.image()).not.toBe(inverted);
  });
});

function popcount(b: number): number {
  let n = 0;
  for (; b; b &= b - 1) n++;
  return n;
}
