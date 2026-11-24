/**
 * The bus activity a board's USART, TWI and SPI carry, as timestamped events: what a host's protocol decoder lanes
 * show. Sampling the pins cannot do this (an MCU-driven net is only seen once per engine step, ~100 us, while a 115200
 * baud bit is 8.7 us), so the events come from the peripherals themselves, which know each byte exactly.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { ArduinoMcu } from "../../../src/sim/engine/arduino.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

const hex = (name: string) => readFileSync(join(__dirname, "__fixtures__", `${name}.hex`), "utf-8");

describe("ArduinoMcu bus events", () => {
  it("records I2C, SPI and UART traffic in order, with each byte's cycle", () => {
    const mcu = new ArduinoMcu(hex("peripherals_ok"));
    mcu.step(0.05);
    const { events, cycles, clockHz, dropped } = mcu.drainBusEvents();
    expect(clockHz).toBe(16_000_000);
    expect(dropped).toBe(0);
    for (let i = 1; i < events.length; i++) expect(events[i]!.cycle).toBeGreaterThanOrEqual(events[i - 1]!.cycle);
    expect(events[events.length - 1]!.cycle).toBeLessThanOrEqual(cycles);

    // Wire.beginTransmission(0x3C); write(0); endTransmission(): an empty bus NACKs the address, so Wire stops there.
    const i2c = events.filter((e) => e.bus === "i2c");
    expect(i2c.map((e) => e.kind)).toEqual(["start", "address", "stop"]);
    expect(i2c[1]).toMatchObject({ address: 0x3c, read: false, ack: false });

    const spi = events.filter((e) => e.bus === "spi");
    expect(spi).toHaveLength(1);
    expect(spi[0]).toMatchObject({ mosi: 0x5a, miso: 0 });
    expect(spi[0]!.cycles).toBeGreaterThan(0);

    const uart = events.filter((e) => e.bus === "uart");
    const text = String.fromCharCode(...uart.map((e) => (e.bus === "uart" ? e.value : 0)));
    expect(text).toBe("eeprom 42,43 i2c 2 spi 0\r\n");
    // 115200 baud, 8N1: (UBRR 16 + 1) x 8 x 10 bits = 1360 cycles a frame (Serial.begin uses U2X).
    expect(uart.every((e) => e.bus === "uart" && e.dir === "tx" && e.cycles === 1360)).toBe(true);
    // I2C happens before SPI before the report.
    expect(i2c[0]!.cycle).toBeLessThan(spi[0]!.cycle);
    expect(spi[0]!.cycle).toBeLessThan(uart[0]!.cycle);

    expect(mcu.drainBusEvents().events).toHaveLength(0);
  });

  it("records received bytes as they start to arrive", () => {
    const mcu = new ArduinoMcu(hex("serial_echo"));
    mcu.step(0.02);
    mcu.drainBusEvents();
    mcu.writeSerialInput(new TextEncoder().encode("hi"));
    mcu.step(0.05);
    const rx = mcu.drainBusEvents().events.filter((e) => e.bus === "uart" && e.dir === "rx");
    expect(rx.map((e) => (e.bus === "uart" ? e.value : 0))).toEqual([0x68, 0x69]);
    expect(rx[1]!.cycle - rx[0]!.cycle).toBeGreaterThanOrEqual(rx[0]!.bus === "uart" ? rx[0]!.cycles : 0);
  });

  it("keeps the newest events when nothing drains them, and counts the rest", () => {
    const mcu = new ArduinoMcu(hex("serial_counter"), { busEventCapacity: 64 });
    mcu.step(0.5);
    const { events, dropped } = mcu.drainBusEvents();
    expect(events).toHaveLength(64);
    expect(dropped).toBeGreaterThan(0);
    for (let i = 1; i < events.length; i++) expect(events[i]!.cycle).toBeGreaterThan(events[i - 1]!.cycle);
  });
});

describe("SimEngine bus events", () => {
  it("reports each board's events on the simulation's clock", () => {
    const engine = new SimEngine();
    engine.load({
      components: [
        {
          id: "uno",
          kind: "arduino_uno",
          pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
          params: { vcc: 5, hex: hex("serial_counter"), usb_power: 1 },
        },
      ],
      wires: [],
    } satisfies SimCircuit);
    for (let i = 0; i < 10; i++) engine.step(0.01);
    const out = engine.drainMcuBusEvents();
    const events = out.uno!;
    expect(events.length).toBeGreaterThan(10);
    // 9600 baud: a frame every ~1.04 ms while the sketch keeps the buffer full; times are seconds of simulation.
    const t = events.map((e) => e.t);
    expect(t[0]).toBeGreaterThanOrEqual(0);
    expect(t[t.length - 1]).toBeLessThanOrEqual(0.1 + 1e-9);
    const gaps = t.slice(1).map((x, i) => x - t[i]!);
    expect(Math.min(...gaps.slice(5))).toBeCloseTo(1 / 960, 5);
    expect(engine.drainMcuBusEvents()).toEqual({});
  });

  function uno(usbPower: number): SimCircuit {
    return {
      components: [
        {
          id: "uno",
          kind: "arduino_uno",
          pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
          params: { vcc: 5, hex: hex("serial_counter"), usb_power: usbPower },
        },
      ],
      wires: [],
    };
  }

  it("dates events from the board's own clock, so time the board did not run after them does not move them", () => {
    const engine = new SimEngine();
    engine.load(uno(1));
    for (let i = 0; i < 5; i++) engine.step(0.01);
    engine.load(uno(0)); // unplugged: the board keeps its state and runs nothing
    for (let i = 0; i < 5; i++) engine.step(0.01);
    const t = engine.drainMcuBusEvents().uno!.map((e) => e.t);
    expect(t.length).toBeGreaterThan(10);
    expect(Math.max(...t)).toBeLessThanOrEqual(0.05);
  });

  it("places events drained step by step exactly as their cycles are apart", () => {
    const engine = new SimEngine();
    engine.load(uno(1));
    const events: Array<{ t: number; cycle: number }> = [];
    for (let i = 0; i < 200; i++) {
      engine.step(0.0005);
      events.push(...(engine.drainMcuBusEvents().uno ?? []));
    }
    expect(events.length).toBeGreaterThan(50);
    for (let i = 1; i < events.length; i++) {
      expect(events[i]!.t - events[i - 1]!.t).toBeCloseTo((events[i]!.cycle - events[i - 1]!.cycle) / 16e6, 12);
    }
  });
});

describe("ArduinoMcu clock", () => {
  it("runs exactly the cycles its steps add up to, carrying each step's overshoot into the next", () => {
    const mcu = new ArduinoMcu(hex("serial_counter"));
    for (let i = 0; i < 1000; i++) mcu.step(1e-4);
    // An instruction can end up to four cycles past a step's end; that must not add up over a thousand steps.
    expect(Math.abs(mcu.drainBusEvents().cycles - 1_600_000)).toBeLessThanOrEqual(4);
  });
});
