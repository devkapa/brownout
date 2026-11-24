/**
 * The 328P's bus timing where avr8js 0.21.0 has none (src/sim/engine/avr-bus-timing.ts): I2C steps take their time
 * at the SCL rate TWBR sets, and the USART transmitter is double-buffered. Both fixtures are bare-metal C beside
 * their hex, so every cycle between two events is the peripheral's, not a library's.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { ArduinoMcu } from "../../../src/sim/engine/arduino.js";
import { twiSclCycles, twiStepCycles } from "../../../src/sim/engine/avr-bus-timing.js";

const hex = (name: string) => readFileSync(join(__dirname, "__fixtures__", `${name}.hex`), "utf-8");

describe("TWI bus timing", () => {
  it("takes the SCL rate's time for each step: half a period to start, nine per byte, one to stop", () => {
    const mcu = new ArduinoMcu(hex("twi_timing"));
    mcu.step(0.002);
    const i2c = mcu.drainBusEvents().events.filter((e) => e.bus === "i2c");
    expect(i2c.map((e) => e.bus === "i2c" && e.kind)).toEqual(["start", "address", "stop", "start", "address", "stop"]);
    // TWBR 72 is 100 kHz (160 cycles a period), TWBR 12 is 400 kHz (40).
    for (const [n, scl] of [[0, 160], [3, 40]] as const) {
      const [start, address, stop] = i2c.slice(n, n + 3);
      expect(start!.bus === "i2c" && start!.cycles).toBe(scl / 2);
      expect(address!.bus === "i2c" && address!.cycles).toBe(9 * scl);
      expect(stop!.bus === "i2c" && stop!.cycles).toBe(scl);
      // The program starts each step a few instructions after the last one ends; it cannot start sooner.
      const gapAfterStart = address!.cycle - start!.cycle - scl / 2;
      const gapAfterAddress = stop!.cycle - address!.cycle - 9 * scl;
      expect(gapAfterStart).toBeGreaterThanOrEqual(0);
      expect(gapAfterStart).toBeLessThan(40);
      expect(gapAfterAddress).toBeGreaterThanOrEqual(0);
      expect(gapAfterAddress).toBeLessThan(40);
    }
    // The 400 kHz transaction starts only after the 100 kHz STOP has finished.
    expect(i2c[3]!.cycle - i2c[2]!.cycle).toBeGreaterThanOrEqual(160);
  });

  it("raises TWINT only where the datasheet does: not out of reset, not after a STOP, not on a write of zero", () => {
    const mcu = new ArduinoMcu(hex("twi_flags"));
    mcu.step(0.002);
    const portB = mcu.getStepPinEvents().filter((e) => e.pin.startsWith("d") && e.level === 1).map((e) => e.pin);
    // PORTB bits 0-3 are d8-d11.
    expect(portB).toEqual(expect.arrayContaining(["d8", "d9", "d10", "d11"]));
  });

  it("leaves TWINT and UDRE set when their interrupts are taken, as levels the handler must clear", () => {
    const mcu = new ArduinoMcu(hex("flags_isr"));
    mcu.step(0.002);
    const high = new Set(mcu.getStepPinEvents().filter((e) => e.level === 1).map((e) => e.pin));
    // PORTB bits 0-3 are d8-d11: TWINT set in its handler, still set after it, UDRE set in its handler, both ran.
    expect([...high].filter((p) => ["d8", "d9", "d10", "d11"].includes(p)).sort()).toEqual(["d10", "d11", "d8", "d9"]);
  });

  it("holds a START after a STOP for the bus-free time, 4.7 us up to 100 kHz", () => {
    const mcu = new ArduinoMcu(hex("twi_flags"));
    mcu.step(0.002);
    const i2c = mcu.drainBusEvents().events.filter((e) => e.bus === "i2c");
    expect(i2c.map((e) => e.bus === "i2c" && e.kind)).toEqual(["start", "stop", "start", "stop", "start"]);
    const stop = i2c[3]!;
    const free = i2c[4]!.cycle - (stop.cycle + stop.cycles);
    // 4.7 us at 16 MHz is 75.2 cycles; the program asks for the START within a few cycles of the STOP ending.
    expect(free).toBeGreaterThanOrEqual(76);
    expect(free).toBeLessThan(76 + 4);
  });

  it("reads the rate from TWBR and the TWSR prescaler", () => {
    const data = new Uint8Array(0x100);
    data[0xb8] = 72;
    expect(twiSclCycles(data)).toBe(160);
    data[0xb9] = 0b11; // prescaler 64
    data[0xb8] = 255;
    expect(twiSclCycles(data)).toBe(16 + 2 * 255 * 64);
    expect(twiStepCycles("restart", 160)).toBe(240);
  });
});

describe("USART transmit buffer", () => {
  it("queues a second byte behind the first and sends the frames back to back", () => {
    const mcu = new ArduinoMcu(hex("uart_buffer"));
    mcu.step(0.002);
    const tx = mcu.drainBusEvents().events.filter((e) => e.bus === "uart");
    expect(tx.map((e) => (e.bus === "uart" ? String.fromCharCode(e.value) : ""))).toEqual(["A", "B", "C"]);
    const frame = 1360; // 115200 baud with U2X: (16 + 1) x 8 x 10
    expect(tx[1]!.cycle - tx[0]!.cycle).toBe(frame);
    expect(tx[2]!.cycle - tx[1]!.cycle).toBe(frame);
    expect(new TextDecoder().decode(mcu.drainSerialOutput())).toBe("ABC");

    // PORTB counts each accepted write, then 4 once TXC says the last frame left.
    const rises = (pin: string) => mcu.getStepPinEvents().filter((e) => e.pin === pin && e.level === 1).map((e) => e.cycle);
    const [bAccepted] = rises("d9");
    const [, cAccepted] = rises("d8");
    const [allSent] = rises("d10");
    // B waits in UDR while A is on the wire; C has to wait for A's frame to end.
    expect(bAccepted!).toBeLessThan(tx[0]!.cycle + 100);
    expect(cAccepted!).toBeGreaterThanOrEqual(tx[1]!.cycle);
    expect(cAccepted!).toBeLessThan(tx[1]!.cycle + 100);
    expect(allSent!).toBeGreaterThanOrEqual(tx[2]!.cycle + frame);
    expect(allSent!).toBeLessThan(tx[2]!.cycle + frame + 100);
  });

  it("keeps raising the data-register-empty interrupt while UDR is empty, so an ISR-fed stream keeps going", () => {
    // avr8js clears UDRE when it takes the interrupt; the buffer must set it again once UDR has room.
    const mcu = new ArduinoMcu(hex("uart_udrie"));
    mcu.step(0.002);
    expect(new TextDecoder().decode(mcu.drainSerialOutput())).toBe("ABCDE");
    const tx = mcu.drainBusEvents().events.filter((e) => e.bus === "uart");
    for (let i = 1; i < tx.length; i++) expect(tx[i]!.cycle - tx[i - 1]!.cycle).toBe(1360);
  });

  it("ignores a write while UDR is full and leaves TXC set when the next frame starts", () => {
    const mcu = new ArduinoMcu(hex("uart_flags"));
    mcu.step(0.002);
    expect(new TextDecoder().decode(mcu.drainSerialOutput())).toBe("ABD");
    expect(mcu.getStepPinEvents().some((e) => e.pin === "d8" && e.level === 1)).toBe(true);
  });

  it("dates each frame's length from UBRR exactly, at any baud rate", () => {
    const mcu = new ArduinoMcu(hex("uart_1200"));
    mcu.step(0.01);
    const [frame] = mcu.drainBusEvents().events;
    expect(frame).toMatchObject({ bus: "uart", value: 0x78, cycles: (1666 + 1) * 8 * 10 });
  });
});

describe("USART receiver", () => {
  // 115200 with U2X: UBRR 16, 8 samples a bit of 17 cycles each.
  const frame = 1360;
  // RXC rises when the stop bit's middle sample is taken: 9 bits in, then samples 4, 5 and 6 of the stop bit.
  const rxc = (9 * 8 + 5) * 17;

  function receive(text: string) {
    const mcu = new ArduinoMcu(hex("uart_rx"));
    mcu.step(0.0005); // past the receiver's setup
    mcu.getStepPinEvents();
    mcu.writeSerialInput(new TextEncoder().encode(text));
    mcu.step(0.01);
    return mcu;
  }

  it("raises RXC at the stop bit's sample, before the frame's end", () => {
    const mcu = receive("W");
    const [rx] = mcu.drainBusEvents().events.filter((e) => e.bus === "uart" && e.dir === "rx");
    const seen = mcu.getStepPinEvents().find((e) => e.pin === "d8" && e.level === 1)!;
    expect(seen.cycle - rx!.cycle).toBeGreaterThanOrEqual(rxc);
    expect(seen.cycle - rx!.cycle).toBeLessThan(frame);
  });

  it("holds two frames in its FIFO and a third in the shift register, and a fourth overruns the third", () => {
    const mcu = receive("WXYZ");
    const res = mcu.readData(0x300, 7);
    expect(res[0]).toBe(3);
    const got = [0, 1, 2].map((i) => ({ value: String.fromCharCode(res[2 + 2 * i]!), dor: (res[1 + 2 * i]! & 0x08) !== 0 }));
    // Y was waiting in the shift register when Z's start bit came: lost, and Z, the next frame read, carries DOR.
    expect(got).toEqual([
      { value: "W", dor: false },
      { value: "X", dor: false },
      { value: "Z", dor: true },
    ]);
  });
});
