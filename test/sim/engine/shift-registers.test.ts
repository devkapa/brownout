/**
 * The shift registers 74HC595 (SIPO with output latch) and 74HC165 (PISO), read
 * only through the surface both engines share: digital state and net voltages,
 * never classic's `icState`. The cases therefore run unchanged on classic and,
 * through gate 1 (test/core/suites), on the Rust core.
 *
 * Every input is a pulse source stepped at 0.25 ms. A clock rises at 0.5, 1.5,
 * 2.5 ... ms, so the step ending 0.25 ms after a rise sees it, and each case
 * samples at whole milliseconds, after the rise and well before the next one.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

const HC595_PINS = [
  "qb", "qc", "qd", "qe", "qf", "qg", "qh", "gnd",
  "qh2", "/srclr", "srclk", "rclk", "/oe", "ser", "qa", "vcc",
];
const HC165_PINS = [
  "/pl", "cp", "d4", "d5", "d6", "d7", "/q7", "gnd",
  "ds", "d0", "d1", "d2", "d3", "q7", "cp_inh", "vcc",
];
const OUTPUTS = ["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh"];

function wire(from: string, fromPin: string, to: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: from, from_pin: fromPin, to_component: to, to_pin: toPin };
}

/** 0 V to 5 V pulse source; params override the 1 us edges. */
function pulse(id: string, params: Record<string, number>): Component {
  return {
    id,
    kind: "pulse_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { v1: 0, v2: 5, tr: 1e-6, tf: 1e-6, ...params },
  };
}

function stepTo(engine: SimEngine, t: number, h = 0.25e-3): void {
  while (engine.simTime < t - h / 2) engine.step(h);
}

/** The byte on QA (bit 0) to QH (bit 7). */
function outputs(engine: SimEngine, id = "u1"): number {
  return OUTPUTS.reduce((byte, pin, bit) => byte | ((engine.digitalState[`${id}/${pin}`] ?? 0) << bit), 0);
}

function pinVoltage(engine: SimEngine, id: string, pin: string): number {
  return engine.getNetV()[engine.getNetIdForPin(id, pin)!] ?? Number.NaN;
}

const supply = (): Component => ({ id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } });

/** Names the source driving a pin; `null` ties it to 5 V (`hi`) or ground. */
type Drive = string | "hi" | "lo";

function drive(source: Drive, chip: string, pin: string): SimCircuit["wires"] {
  if (source === "hi") return [wire("vcc", "pos", chip, pin)];
  if (source === "lo") return [wire("vcc", "neg", chip, pin)];
  return [wire(source, "pos", chip, pin)];
}

function sourceReturns(sources: Component[]): SimCircuit["wires"] {
  return sources.map((source) => wire(source.id, "neg", "vcc", "neg"));
}

/** The clock rises at 0.5, 1.5, 2.5 ... ms. */
const clock = (): Component => pulse("clk", { td: 0.5e-3, pw: 0.5e-3, per: 1e-3 });
/** SER reads 1, 1, 0 at those rises, repeating, and changes only while the clock is low. */
const data = (): Component => pulse("data", { td: 0.2e-3, pw: 1.998e-3, per: 3e-3 });

function board595(opts: { srclk: Drive; rclk: Drive; ser?: Drive; srclr?: Drive; oe?: Drive; sources: Component[]; extra?: Component[]; extraWires?: SimCircuit["wires"] }): SimCircuit {
  return {
    components: [
      supply(),
      { id: "u1", kind: "74hc595", pins: HC595_PINS.map((id) => ({ id })), params: {} },
      ...opts.sources,
      ...(opts.extra ?? []),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      ...drive(opts.oe ?? "lo", "u1", "/oe"),
      ...drive(opts.srclk, "u1", "srclk"),
      ...drive(opts.rclk, "u1", "rclk"),
      ...drive(opts.ser ?? "hi", "u1", "ser"),
      ...drive(opts.srclr ?? "hi", "u1", "/srclr"),
      ...sourceReturns(opts.sources),
      ...(opts.extraWires ?? []),
    ],
  };
}

// The shift register after each clock, by hand: shift left, SER into QA.
//   clock   1   2   3   4   5   6   7   8   9  10
//   SER     1   1   0   1   1   0   1   1   0   1
//   SR     01  03  06  0d  1b  36  6d  db  b6  6d
const SHIFTED = [0x01, 0x03, 0x06, 0x0d, 0x1b, 0x36, 0x6d, 0xdb, 0xb6, 0x6d];

describe("74HC595 shift and store", () => {
  // SRCLK shifts eight times, then RCLK stores the result: the shiftOut()-then-latch pattern.
  const latchAfterEight = (rclkRise: number) => {
    const engine = new SimEngine();
    engine.load(board595({
      srclk: "clk",
      rclk: "latch",
      ser: "data",
      sources: [clock(), data(), pulse("latch", { td: rclkRise, pw: 0.3e-3, per: 8e-3 })],
    }));
    const shown: number[] = [];
    for (const t of [7.25e-3, 8.75e-3, 15.25e-3, 16.75e-3]) {
      stepTo(engine, t);
      shown.push(outputs(engine));
    }
    return shown;
  };

  it("holds the outputs until RCLK, then shows the byte shifted in (QA is the newest bit)", () => {
    // RCLK rises at 8.2 ms, after the eighth SRCLK edge (7.5 ms) has fallen.
    expect(latchAfterEight(8.2e-3)).toEqual([0x00, 0xdb, 0xdb, 0x6d]);
  });

  it("stores the shifted byte when RCLK follows the last shift inside one step", () => {
    // RCLK rises 30 us after the eighth SRCLK edge, so one step sees both rises.
    expect(latchAfterEight(7.53e-3)).toEqual([0x00, 0xdb, 0xdb, 0x6d]);
  });

  it("clears the shift register, not the storage register, on /SRCLR", () => {
    // Four ones are shifted in (clock edges at 0.5 .. 3.5 ms) and stored at 4.2 ms. /SRCLR is low from 4.9 to
    // 5.9 ms, so the edge at 5.5 ms shifts nothing, and the store at 6.2 ms takes the cleared register.
    const engine = new SimEngine();
    engine.load(board595({
      srclk: "clk",
      rclk: "latch",
      srclr: "clr",
      sources: [
        clock(),
        pulse("latch", { td: 4.2e-3, pw: 0.3e-3, per: 2e-3 }),
        pulse("clr", { v1: 5, v2: 0, td: 4.9e-3, pw: 1.0e-3, per: 0 }),
      ],
    }));
    stepTo(engine, 4.1e-3);
    expect(outputs(engine), "nothing is stored yet").toBe(0x00);
    stepTo(engine, 4.75e-3);
    expect(outputs(engine)).toBe(0x0f);
    stepTo(engine, 5.75e-3);
    expect(outputs(engine), "the storage register survives the clear").toBe(0x0f);
    stepTo(engine, 6.75e-3);
    expect(outputs(engine), "the next store takes the cleared register").toBe(0x00);
  });

  it("tri-states the outputs while /OE is high and keeps the stored byte", () => {
    // A 10 kohm pull-up on QA: enabled with QA stored low it pulls to ground, disabled it floats up to 5 V.
    const engine = new SimEngine();
    engine.load(board595({
      srclk: "lo",
      rclk: "lo",
      ser: "lo",
      oe: "oe",
      sources: [pulse("oe", { td: 1.5e-3, pw: 2e-3, per: 0 })],
      extra: [{ id: "rup", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } }],
      extraWires: [wire("vcc", "pos", "rup", "a"), wire("rup", "b", "u1", "qa")],
    }));
    stepTo(engine, 1e-3);
    expect(pinVoltage(engine, "u1", "qa"), "enabled, stored low").toBeLessThan(0.5);
    stepTo(engine, 2.5e-3);
    expect(pinVoltage(engine, "u1", "qa"), "/OE high releases the pin").toBeGreaterThan(4.5);
    expect(outputs(engine), "the register is untouched").toBe(0x00);
    stepTo(engine, 4.5e-3);
    expect(pinVoltage(engine, "u1", "qa"), "/OE low drives it again").toBeLessThan(0.5);
  });
});

describe("74HC595 tied clocks, read through the outputs", () => {
  it("shows the shift register from one clock earlier", () => {
    const engine = new SimEngine();
    engine.load(board595({ srclk: "clk", rclk: "clk", ser: "data", sources: [clock(), data()] }));
    const shown: number[] = [];
    for (let n = 1; n <= 10; n++) {
      stepTo(engine, n * 1e-3);
      shown.push(outputs(engine));
    }
    // The storage register starts empty, then takes the previous clock's register.
    expect(shown).toEqual([0x00, ...SHIFTED.slice(0, 9)]);
  });

  it("keeps the outputs dark at the first clock after load", () => {
    const engine = new SimEngine();
    engine.load(board595({ srclk: "clk", rclk: "clk", sources: [clock()] }));
    stepTo(engine, 0.25e-3);
    expect(outputs(engine)).toBe(0x00);
    stepTo(engine, 1e-3);
    expect(outputs(engine)).toBe(0x00);
    expect(pinVoltage(engine, "u1", "qa")).toBeLessThan(0.5);
    stepTo(engine, 2e-3);
    expect(outputs(engine)).toBe(0x01);
    expect(pinVoltage(engine, "u1", "qa")).toBeGreaterThan(4.5);
  });
});

/** A 165 with the data bits `d` (bit i on `d<i>`) tied to the rails; `/PL`, `CP` and `CP_INH` from named sources. */
function board165(opts: { d: number; pl: Drive; cp: Drive; inh?: Drive; ds?: Drive; sources: Component[] }): SimCircuit {
  const dWires = [0, 1, 2, 3, 4, 5, 6, 7].flatMap((i) => drive(((opts.d >> i) & 1) === 1 ? "hi" : "lo", "u1", `d${i}`));
  return {
    components: [supply(), { id: "u1", kind: "74hc165", pins: HC165_PINS.map((id) => ({ id })), params: {} }, ...opts.sources],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      ...dWires,
      ...drive(opts.pl, "u1", "/pl"),
      ...drive(opts.cp, "u1", "cp"),
      ...drive(opts.inh ?? "lo", "u1", "cp_inh"),
      ...drive(opts.ds ?? "lo", "u1", "ds"),
      ...sourceReturns(opts.sources),
    ],
  };
}

/** q7 and /q7 as digital states. */
function q7(engine: SimEngine): [number, number] {
  return [engine.digitalState["u1/q7"] ?? Number.NaN, engine.digitalState["u1//q7"] ?? Number.NaN];
}

describe("74HC165 load and shift", () => {
  // /PL is low until 0.3 ms, then released.
  const pl = () => pulse("pl", { v1: 0, v2: 5, td: 0.3e-3, pw: 1, per: 0 });

  it("loads D0..D7 asynchronously and shifts them out D7 first", () => {
    const engine = new SimEngine();
    engine.load(board165({ d: 0xa5, pl: "pl", cp: "clk", sources: [pl(), clock()] }));
    stepTo(engine, 0.25e-3);
    expect(q7(engine), "loaded: q7 is D7").toEqual([1, 0]);
    // After n clocks with DS low, q7 is bit 7 of 0xa5 << n.
    const expected = [0, 1, 0, 0, 1, 0, 1, 0];
    const seen: number[] = [];
    for (let n = 1; n <= 8; n++) {
      stepTo(engine, n * 1e-3);
      const [q, qn] = q7(engine);
      expect(qn, `/q7 after clock ${n}`).toBe(1 - q);
      seen.push(q);
    }
    expect(seen).toEqual(expected);
  });

  it("shifts DS in at the far end", () => {
    // D = 0 and DS high: after eight clocks the register is all ones, so q7 is 1 from the eighth clock on.
    const engine = new SimEngine();
    engine.load(board165({ d: 0x00, pl: "pl", cp: "clk", ds: "hi", sources: [pl(), clock()] }));
    const seen: number[] = [];
    for (let n = 1; n <= 9; n++) {
      stepTo(engine, n * 1e-3);
      seen.push(q7(engine)[0]);
    }
    expect(seen).toEqual([0, 0, 0, 0, 0, 0, 0, 1, 1]);
  });

  it("ignores clock edges while CP_INH is high, and releasing it alone does not clock", () => {
    // CP_INH is high from 0 to 3.8 ms. The edges at 0.5, 1.5, 2.5 and 3.5 ms are ignored, and CP is still high
    // when CP_INH falls at 3.8 ms; the edge at 4.5 ms is the first that shifts.
    const engine = new SimEngine();
    engine.load(
      board165({ d: 0x80, pl: "pl", cp: "clk", inh: "inh", sources: [pl(), clock(), pulse("inh", { v1: 5, v2: 0, td: 3.8e-3, pw: 1, per: 0 })] }),
    );
    stepTo(engine, 3e-3);
    expect(q7(engine)[0], "inhibited edges shift nothing").toBe(1);
    stepTo(engine, 4e-3);
    expect(q7(engine)[0], "releasing CP_INH under a high CP does not clock").toBe(1);
    stepTo(engine, 5e-3);
    expect(q7(engine)[0], "the first enabled edge shifts D7 out").toBe(0);
  });

  it("holds /PL low over a clock: the load wins", () => {
    const engine = new SimEngine();
    engine.load(board165({ d: 0x80, pl: "lo", cp: "clk", sources: [clock()] }));
    for (let n = 1; n <= 3; n++) {
      stepTo(engine, n * 1e-3);
      expect(q7(engine)[0]).toBe(1);
    }
  });
});
