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
});
