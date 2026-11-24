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

  it("rounds a DHT11 temperature to tenths before splitting it, so the tenth carries into the whole degrees", () => {
    expect([...dhtFrame(11, 42, 23.96)]).toEqual([42, 0, 24, 0, 66]);
    expect([...dhtFrame(11, 42, 49.97)]).toEqual([42, 0, 50, 0, 92]);
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
    b.run(b.cycles + us(2_100_000));
    d.setAmbient(80, 30);
    b.edges.length = 0;
    const second = start(d, b, 1100);
    b.run(second + us(2_100_000));
    expect(decode(b.edges, second).slice(0, 16).reduce((v, bit) => (v << 1) | bit, 0)).toBe(400);
    b.edges.length = 0;
    const third = start(d, b, 1100);
    b.run(third + us(6000));
    expect(decode(b.edges, third).slice(0, 16).reduce((v, bit) => (v << 1) | bit, 0)).toBe(800);
  });

  it("sends the previous measurement again to a reading sooner than its interval, and does not measure after it", () => {
    const humidity = (b: ReturnType<typeof board>, from: number) =>
      decode(b.edges, from).slice(0, 16).reduce((v, bit) => (v << 1) | bit, 0);
    const read = (d: DhtSensor, b: ReturnType<typeof board>, gapUs: number) => {
      b.edges.length = 0;
      const at = start(d, b, 1100);
      b.run(at + us(gapUs));
      return humidity(b, at);
    };
    // DHT22, 2 s: the reading 0.5 s after a measurement sends it and leaves it; so does the next, 1 s after it.
    const d22 = sensor(22);
    const b22 = board();
    d22.setAmbient(40, 20);
    expect(read(d22, b22, 2_100_000)).toBe(400);
    d22.setAmbient(80, 20);
    expect(read(d22, b22, 500_000)).toBe(400);
    d22.setAmbient(60, 20);
    expect(read(d22, b22, 500_000)).toBe(800);
    expect(read(d22, b22, 2_100_000)).toBe(800);
    // 3 s after the last measurement: it sends that one, then measures.
    expect(read(d22, b22, 10_000)).toBe(800);
    expect(read(d22, b22, 10_000)).toBe(600);
    // DHT11, 1 s: 0.5 s after a measurement does not measure, 1.1 s does.
    const d11 = sensor(11);
    const b11 = board();
    d11.setAmbient(40, 20);
    const read11 = (gapUs: number) => {
      b11.edges.length = 0;
      const at = start(d11, b11, 20_000);
      b11.run(at + us(gapUs));
      return decode(b11.edges, at).slice(0, 8).reduce((v, bit) => (v << 1) | bit, 0);
    };
    // Each reading, then the wait before the next.
    expect(read11(500_000)).toBe(40);
    d11.setAmbient(70, 20);
    expect(read11(1_100_000)).toBe(40);
    expect(read11(10_000)).toBe(40);
    d11.setAmbient(30, 20);
    expect(read11(1_100_000)).toBe(70);
    expect(read11(10_000)).toBe(70);
    expect(read11(10_000)).toBe(30);
  });

  it("times its interval from one reading's start to the next, whatever the bits make each reply's length", () => {
    const humidity = (b: ReturnType<typeof board>, from: number) =>
      decode(b.edges, from).slice(0, 16).reduce((v, bit) => (v << 1) | bit, 0);
    const d = sensor(22);
    const b = board();
    // A long reply (many 1 bits) followed, exactly 2 s after its release, by a short one.
    d.setAmbient(99.9, 79.9);
    const first = start(d, b, 1100);
    d.setAmbient(0, 0);
    b.run(first + us(2_000_000) - us(1100));
    b.edges.length = 0;
    const second = start(d, b, 1100);
    expect(second - first).toBe(us(2_000_000));
    d.setAmbient(50, 20);
    b.run(second + us(10_000));
    expect(humidity(b, second)).toBe(0);
    b.edges.length = 0;
    const third = start(d, b, 1100);
    b.run(third + us(10_000));
    // 2 s from start to start is the interval: it measured after the second reading.
    expect(humidity(b, third)).toBe(500);
  });

  it("measures again after its board resets, the time since its last measurement lost with the board's clock", () => {
    const humidity = (b: ReturnType<typeof board>, from: number) =>
      decode(b.edges, from).slice(0, 16).reduce((v, bit) => (v << 1) | bit, 0);
    const read = (d: DhtSensor, b: ReturnType<typeof board>, at: number) => {
      b.run(at);
      b.edges.length = 0;
      const released = start(d, b, 1100);
      b.run(released + us(10_000));
      return humidity(b, released);
    };
    // The rebuilt board's clock reads 3.5 s (0.5 s after the last measurement on the old one) or 1 s (before it).
    for (const [told, at, measures] of [[true, 3_500_000, true], [false, 3_500_000, false], [false, 1_000_000, true]] as const) {
      const d = sensor(22);
      d.setAmbient(40, 20);
      expect(read(d, board(), us(3_000_000))).toBe(400);
      d.setAmbient(80, 20);
      const fresh = board();
      if (told) d.boardReset();
      expect(read(d, fresh, us(at))).toBe(400);
      expect(read(d, fresh, fresh.cycles + us(10_000)), `told ${told}, ${at} us`).toBe(measures ? 800 : 400);
    }
  });

  it("lets the line go when it loses power or resets in the middle of a reply", () => {
    for (const cut of ["power", "reset"] as const) {
      const d = sensor();
      const b = board();
      const released = start(d, b, 1100);
      // Into the 80 us low that opens the reply.
      b.run(released + us(60));
      expect(b.edges.at(-1)).toEqual([released + us(30), true]);
      if (cut === "power") d.powered = false;
      else d.reset();
      expect(b.edges.at(-1)).toEqual([b.cycles, false]);
      b.run(b.cycles + us(6000));
      expect(b.edges.length).toBe(2);
    }
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

  it("tells a pin target new to it that its clock is not the one the part kept time by", () => {
    const mcu = new ArduinoMcu(readFileSync(join(__dirname, "__fixtures__", "serial_counter.hex"), "utf-8"));
    const sensor = new DhtSensor();
    let told = 0;
    sensor.boardReset = () => void told++;
    mcu.setPinTargets(new Map([["d2", sensor]]));
    expect(told).toBe(1);
    // The same part kept on the same board keeps its time.
    mcu.setPinTargets(new Map([["d2", sensor]]));
    expect(told).toBe(1);
    // A rebuilt board (a firmware re-upload) is a new clock.
    const rebuilt = new ArduinoMcu(readFileSync(join(__dirname, "__fixtures__", "serial_counter.hex"), "utf-8"));
    rebuilt.setPinTargets(new Map([["d2", sensor]]));
    expect(told).toBe(2);
  });

  it("tells its pin targets when the board resets", () => {
    const mcu = new ArduinoMcu(readFileSync(join(__dirname, "__fixtures__", "serial_counter.hex"), "utf-8"));
    const sensor = new DhtSensor();
    let told = 0;
    sensor.boardReset = () => void told++;
    mcu.setPinTargets(new Map([["d2", sensor]]));
    expect(told).toBe(1);
    mcu.reset();
    expect(told).toBe(2);
  });

  it("gives the pin back to its pull-up when the part loses power holding it low", () => {
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
    sensor.boardDrive(0, line);
    internals.cpu.cycles += 32_000;
    sensor.boardDrive(null, line);
    let i = 0;
    while ((internals.portD.pinValue & 0x04) !== 0 && i++ < 100) mcu.step(0.00001);
    expect(internals.portD.pinValue & 0x04).toBe(0);
    sensor.powered = false;
    expect(internals.portD.pinValue & 0x04).toBe(0x04);
  });
});
