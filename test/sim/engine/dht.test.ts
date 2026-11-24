/**
 * The DHT sensor on its data line, driven by a scripted board: the frame it encodes, the response and bit timing it
 * drives (AM2302 section 7), and when it stays silent.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { ArduinoMcu } from "../../../src/sim/engine/arduino.js";
import { DhtSensor, dhtFrame, type PinLine } from "../../../src/sim/engine/dht.js";

const HZ = 16e6;
const us = (n: number) => Math.round((n * HZ) / 1e6);

/** A board line on a clock the test advances, recording the cycles the sensor pulls and releases. */
function board() {
  let cycles = 0;
  const events: { at: number; fn: () => void }[] = [];
  const edges: [number, boolean][] = [];
  const line: PinLine = {
    hz: HZ,
    now: () => cycles,
    after: (n, fn) => events.push({ at: cycles + n, fn }),
    pull: (low) => edges.push([cycles, low]),
  };
  const run = (until: number) => {
    for (;;) {
      events.sort((a, b) => a.at - b.at);
      const next = events[0];
      if (!next || next.at > until) break;
      events.shift();
      cycles = next.at;
      next.fn();
    }
    cycles = until;
  };
  return { line, edges, run, set: (c: number) => (cycles = c), get cycles() { return cycles; } };
}

function sensor(variant: 11 | 22 = 22) {
  const d = new DhtSensor({ variant });
  d.powered = true;
  d.poweredFor = 2;
  return d;
}

/** The host start: low for `lowUs`, then released. */
function start(d: DhtSensor, b: ReturnType<typeof board>, lowUs: number): number {
  d.boardDrive(0, b.line);
  b.run(b.cycles + us(lowUs));
  d.boardDrive(null, b.line);
  return b.cycles;
}

/** The bits in the recorded edges: the length of each high after the 80/80 response. */
function decode(edges: [number, boolean][], from: number) {
  const times = edges.map(([c]) => (c - from) / (HZ / 1e6));
  const bits: number[] = [];
  // edges: low, high (response), then for each bit low, high; the end low and release last.
  for (let i = 0; i < 40; i++) {
    const highStart = times[3 + i * 2]!;
    const nextLow = times[4 + i * 2]!;
    bits.push(nextLow - highStart > 48 ? 1 : 0);
  }
  return bits;
}

describe("DhtSensor", () => {
  it("encodes a DHT22 frame in tenths, the temperature's top bit its sign, with the byte-sum checksum", () => {
    expect([...dhtFrame(22, 65.2, 35.1)]).toEqual([0x02, 0x8c, 0x01, 0x5f, 0xee]);
    expect([...dhtFrame(22, 65.2, -10.1)]).toEqual([0x02, 0x8c, 0x80, 0x65, 0x73]);
  });

  it("encodes a DHT11 frame in whole humidity and whole and tenth degrees, clamped to its range", () => {
    expect([...dhtFrame(11, 55.4, 24.7)]).toEqual([55, 0, 24, 7, 86]);
    expect([...dhtFrame(11, 5, -3)]).toEqual([20, 0, 0, 0, 20]);
  });

  it("answers a 1.1 ms start 30 us after release with 80 us low, 80 us high, then the 40 bits and a 50 us end", () => {
    const d = sensor();
    const b = board();
    d.setAmbient(65.2, 35.1);
    const released = start(d, b, 1100);
    b.run(released + us(6000));
    const times = b.edges.map(([c, low]) => [Math.round((c - released) / 16), low]);
    expect(times.slice(0, 3)).toEqual([[30, true], [110, false], [190, true]]);
    expect(b.edges.length).toBe(2 + 80 + 2);
    const bits = decode(b.edges, released);
    const bytes = [0, 1, 2, 3, 4].map((k) => bits.slice(k * 8, k * 8 + 8).reduce((v, bit) => (v << 1) | bit, 0));
    expect(bytes).toEqual([0x02, 0x8c, 0x01, 0x5f, 0xee]);
    // Each bit is a 50 us low and a 27 or 70 us high.
    const lows = b.edges.slice(2, 82).filter((_, i) => i % 2 === 0).map(([c], i) => (b.edges[2 + i * 2 + 1]![0] - c) / 16);
    expect(new Set(lows)).toEqual(new Set([50]));
    expect(b.edges.at(-1)![1]).toBe(false);
  });

  it("ignores a start shorter than the part's minimum: 0.8 ms on the DHT22, 18 ms on the DHT11", () => {
    for (const [variant, short, long] of [[22, 700, 900], [11, 15000, 18500]] as const) {
      const d = sensor(variant);
      const b = board();
      start(d, b, short);
      b.run(b.cycles + us(6000));
      expect(b.edges, `${variant} short`).toEqual([]);
      start(d, b, long);
      b.run(b.cycles + us(6000));
      expect(b.edges.length, `${variant} long`).toBe(84);
    }
  });

  it("stays silent in its first second of power, and unpowered", () => {
    const d = sensor();
    const b = board();
    d.poweredFor = 0.5;
    start(d, b, 1100);
    b.run(b.cycles + us(6000));
    expect(b.edges).toEqual([]);
    d.poweredFor = 2;
    d.powered = false;
    start(d, b, 1100);
    b.run(b.cycles + us(6000));
    expect(b.edges).toEqual([]);
  });

  it("reports the measurement made after the previous reading", () => {
    const d = sensor();
    const b = board();
    d.setAmbient(40, 20);
    start(d, b, 1100);
    b.run(b.cycles + us(6000));
    d.setAmbient(80, 30);
    b.edges.length = 0;
    const second = start(d, b, 1100);
    b.run(second + us(6000));
    expect(decode(b.edges, second).slice(0, 16).reduce((v, bit) => (v << 1) | bit, 0)).toBe(400);
    b.edges.length = 0;
    const third = start(d, b, 1100);
    b.run(third + us(6000));
    expect(decode(b.edges, third).slice(0, 16).reduce((v, bit) => (v << 1) | bit, 0)).toBe(800);
  });

  it("ignores the board while it answers, and answers again after a board reset cut a reply short", () => {
    const d = sensor();
    const b = board();
    start(d, b, 1100);
    b.run(b.cycles + us(1000));
    const mid = b.edges.length;
    // The board resets: its clock starts over and the reply's events are gone.
    const fresh = board();
    const released = start(d, fresh, 1100);
    fresh.run(released + us(6000));
    expect(mid).toBeGreaterThan(2);
    expect(fresh.edges.length).toBe(84);
  });
});

describe("ArduinoMcu pin targets", () => {
  it("gives a pin back to the circuit's level when the part holding it low goes away", () => {
    const mcu = new ArduinoMcu(readFileSync(join(__dirname, "__fixtures__", "serial_counter.hex"), "utf-8"));
    const sensor = new DhtSensor();
    sensor.powered = true;
    mcu.setPinTargets(new Map([["d2", sensor]]));
    mcu.setInputBit("d2", 1);
    const internals = mcu as unknown as { _lineFor(pin: string, target: unknown): PinLine; portD: { pinValue: number } };
    internals._lineFor("d2", sensor).pull(true);
    expect(internals.portD.pinValue & 0x04).toBe(0);
    mcu.setPinTargets(null);
    expect(internals.portD.pinValue & 0x04).toBe(0x04);
  });

  it("stops a part's answer on a pin it was taken off", () => {
    const mcu = new ArduinoMcu(readFileSync(join(__dirname, "__fixtures__", "serial_counter.hex"), "utf-8"));
    const sensor = new DhtSensor();
    sensor.powered = true;
    sensor.poweredFor = 2;
    sensor.setAmbient(50, 20);
    mcu.setPinTargets(new Map([["d2", sensor]]));
    mcu.setInputBit("d2", 1);
    const internals = mcu as unknown as {
      _lineFor(pin: string, target: unknown): PinLine;
      portD: { pinValue: number };
      cpu: { cycles: number };
    };
    const line = internals._lineFor("d2", sensor);
    // A 2 ms start, then the release that sets the part answering.
    sensor.boardDrive(0, line);
    internals.cpu.cycles += 32_000;
    sensor.boardDrive(null, line);
    mcu.setPinTargets(null);
    let pulledLow = false;
    for (let i = 0; i < 600; i++) {
      mcu.step(0.00001);
      if ((internals.portD.pinValue & 0x04) === 0) pulledLow = true;
    }
    expect(pulledLow).toBe(false);
  });
});
