/**
 * A 74HC595's QH' (pin 9) carries the shift register's eighth stage.
 *
 * QH' is the cascade output: it taps the eighth shift-register stage
 * directly, before the storage register and outside the /OE gate (TI
 * SN74HC595 data sheet, SCLS041J, section 5 pin functions and the section
 * 8.2 functional block diagram, which shows pin 9 branching off stage 8
 * while QA-QH pass through the storage flops and the 3-state OE gate; the
 * Table 8-1 function table gates only "Outputs QA – QH" with OE). It is
 * what feeds a second 595's SER in a daisy chain. The model never stamped
 * it, so the net sat at 0 V and a chained chip shifted in nothing.
 *
 * Data sent MSB first reaches the far chip: a bit shifted in at clock 1
 * arrives at stage 8 (QH') at clock 8 and steps into the next chip at
 * clock 9, so after 16 clocks the near chip holds the second byte and the
 * far chip the first. With SRCLK and RCLK tied the storage register trails
 * the shift register by one clock (SCLS041J section 8.1); QH' does not —
 * it is upstream of the storage register.
 *
 * The MCU test spaces its clock pulses one MCU step apart: a pin driven by
 * another chip is read at the end of each step (the MCU-edge replay of
 * mcu-clocked-ics covers only the MCU's own pulses), so each SRCLK the
 * program makes must land in its own step for the far chip to see QH' move
 * between clocks.
 */
import { describe, expect, it } from "vitest";
import { runSmallSignalAc } from "../../../src/sim/engine/ac-analysis.js";
import {
  GATE_R_OUT,
  NODE_RSHUNT_G,
  SimEngine,
  type SimCircuit,
} from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";

const catalog = rawCatalog as unknown as PartCatalog;

type Component = SimCircuit["components"][number];

const HC595_PINS = [
  "qb", "qc", "qd", "qe", "qf", "qg", "qh", "gnd",
  "qh2", "/srclr", "srclk", "rclk", "/oe", "ser", "qa", "vcc",
];
const OUTPUTS = ["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh"];

function wire(from: string, fromPin: string, to: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: from, from_pin: fromPin, to_component: to, to_pin: toPin };
}

function part(id: string, kind: string, pins: string[], params: Record<string, number> = {}): Component {
  return { id, kind, pins: pins.map((pin) => ({ id: pin })), params };
}

function chip(id: string): Component {
  return part(id, "74hc595", HC595_PINS);
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

/**
 * One chip's supply rails: VCC/GND from the named source (default the
 * board's "vcc"), /SRCLR high and /OE low unless named.
 */
function supplyWires(
  u: string,
  opts: { oe?: string; vccFrom?: [string, string]; gndFrom?: [string, string] } = {},
): SimCircuit["wires"] {
  const [vccId, vccPin] = opts.vccFrom ?? ["vcc", "pos"];
  const [gndId, gndPin] = opts.gndFrom ?? ["vcc", "neg"];
  return [
    wire(vccId, vccPin, u, "vcc"),
    wire(gndId, gndPin, u, "gnd"),
    wire(vccId, vccPin, u, "/srclr"),
    opts.oe ? wire(opts.oe, "pos", u, "/oe") : wire(gndId, gndPin, u, "/oe"),
  ];
}

/**
 * One 74HC595 fed by named pulse sources. `srclk`/`rclk` name the source on
 * each clock pin (the same name ties them); `ser` and `oe` fall back to a
 * held level.
 */
function board(opts: {
  id?: string;
  srclk: string;
  rclk: string;
  ser?: string;
  oe?: string;
  sources: Component[];
}): SimCircuit {
  const u = opts.id ?? "u1";
  return {
    components: [
      { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      chip(u),
      ...opts.sources,
    ],
    wires: [
      ...supplyWires(u, { oe: opts.oe }),
      wire(opts.srclk, "pos", u, "srclk"),
      wire(opts.rclk, "pos", u, "rclk"),
      opts.ser ? wire(opts.ser, "pos", u, "ser") : wire("vcc", "pos", u, "ser"),
      ...opts.sources.map((source) => wire(source.id, "neg", "vcc", "neg")),
    ],
  };
}

/**
 * Two 595s daisy-chained, U1 QH' -> U2 SER, with shared clock and latch
 * sources (named per pin; one name for srclk and rclk ties the clocks).
 */
function chainBoard(opts: {
  srclk: string;
  rclk: string;
  ser: string;
  sources: Component[];
}): SimCircuit {
  return {
    components: [
      { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      chip("u1"),
      chip("u2"),
      ...opts.sources,
    ],
    wires: [
      ...supplyWires("u1"),
      ...supplyWires("u2"),
      wire("u1", "qh2", "u2", "ser"),
      wire(opts.ser, "pos", "u1", "ser"),
      wire(opts.srclk, "pos", "u1", "srclk"),
      wire(opts.srclk, "pos", "u2", "srclk"),
      wire(opts.rclk, "pos", "u1", "rclk"),
      wire(opts.rclk, "pos", "u2", "rclk"),
      ...opts.sources.map((source) => wire(source.id, "neg", "vcc", "neg")),
    ],
  };
}

function stepTo(engine: SimEngine, t: number, h = 0.25e-3): void {
  while (engine.simTime < t - h / 2) engine.step(h);
}

/** The byte on QA (bit 0) to QH (bit 7). */
function outputs(engine: SimEngine, id = "u1"): number {
  return OUTPUTS.reduce((byte, pin, bit) => byte | ((engine.digitalState[`${id}/${pin}`] ?? 0) << bit), 0);
}

function registers(engine: SimEngine, id = "u1"): { shift: number; latch: number; outputs: number } {
  const st = engine.getIcState(id) ?? {};
  return { shift: st.shift ?? -1, latch: st.latch ?? -1, outputs: outputs(engine, id) };
}

function pinVoltage(engine: SimEngine, id: string, pin: string): number {
  return engine.getNetV()[engine.getNetIdForPin(id, pin)!] ?? Number.NaN;
}

// The clock rises at 0.5, 1.5, 2.5 ... ms; each rise is seen by the step
// that ends 0.25 ms after it, and SER is read there.
const clock = (id = "clk") => pulse(id, { td: 0.5e-3, pw: 0.5e-3, per: 1e-3 });
// SER reads 1, 1, 0 at those rises, repeating; it changes only while the
// clock is low. Sixteen clocks of it carry 0xDB then 0x6D, MSB first.
const data = () => pulse("data", { td: 0.2e-3, pw: 1.998e-3, per: 3e-3 });
// A latch pulse that rises once at `td`, after the last shift has settled.
const latch = (td: number) => pulse("latch", { td, pw: 0.5e-3, per: 0 });

// The shift register after each clock, by hand: shift left, SER into QA.
//   clock  1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16
//   SER    1   1   0   1   1   0   1   1   0   1   1   0   1   1   0   1
//   SR    01  03  06  0d  1b  36  6d  db  b6  6d  db  b6  6d  db  b6  6d
// QH' after clock k is the bit clocked in at k-7: clocks 9 to 14 show
// b2..b7 = 1, 0, 1, 1, 0, 1.
const SHIFTED = [0x01, 0x03, 0x06, 0x0d, 0x1b, 0x36, 0x6d, 0xdb, 0xb6, 0x6d, 0xdb, 0xb6, 0x6d, 0xdb, 0xb6, 0x6d];

describe("74HC595 QH' daisy chain (pulse-driven)", () => {
  it("carries the first byte sent to the far chip", () => {
    const engine = new SimEngine();
    engine.load(chainBoard({
      srclk: "clk",
      rclk: "latch",
      ser: "data",
      sources: [clock(), data(), latch(16.2e-3)],
    }));
    // Sixteen clocks, then the latch rise at 16.2 ms is seen by the step
    // ending at 16.25 ms; the seventeenth clock rise is not until 16.5 ms.
    stepTo(engine, 16.3e-3);
    // The near chip keeps the last byte (0x6D) and the far chip the first
    // (0xDB): a bit sent at clock 1 reaches the far register at clock 9.
    expect(registers(engine, "u1")).toEqual({ shift: 0x6d, latch: 0x6d, outputs: 0x6d });
    expect(registers(engine, "u2")).toEqual({ shift: 0xdb, latch: 0xdb, outputs: 0xdb });
    // One more solve (short of the seventeenth rise) drives the latched
    // levels onto the pins.
    engine.step(0.15e-3);
    expect(pinVoltage(engine, "u2", "qh")).toBeGreaterThan(4.5);
    expect(pinVoltage(engine, "u1", "qh")).toBeLessThan(0.5);
    // QH' ends at the near register's top bit: 0x6D bit 7 = 0.
    expect(pinVoltage(engine, "u1", "qh2")).toBeLessThan(0.5);
  });

  it("shows QH' at the eighth-stage bit as bits shift out of it", () => {
    const engine = new SimEngine();
    engine.load(board({ srclk: "clk", rclk: "later", ser: "data", sources: [clock(), data(), pulse("later", { td: 100e-3 })] }));
    // After each clock past the eighth, QH' is the SER bit from seven
    // clocks earlier — the bits shifted out so far (SHIFTED above).
    const expected = [1, 0, 1, 1, 0, 1];
    for (let n = 9; n <= 14; n++) {
      stepTo(engine, n * 1e-3);
      const shift = engine.getIcState("u1")?.shift ?? -1;
      expect(shift, `shift register after ${n} clocks`).toBe(SHIFTED[n - 1]);
      expect(engine.digitalState["u1/qh2"], `QH' level after ${n} clocks`).toBe(expected[n - 9]);
      const v = pinVoltage(engine, "u1", "qh2");
      if (expected[n - 9]) expect(v, `QH' voltage after ${n} clocks`).toBeGreaterThan(4.5);
      else expect(v, `QH' voltage after ${n} clocks`).toBeLessThan(0.5);
    }
  });

  it("keeps QH' driven while /OE disables the eight outputs", () => {
    // The OE gate sits after the storage register and gates only QA-QH
    // (SCLS041J, Table 8-1); the block diagram branches QH' off before it.
    const engine = new SimEngine();
    engine.load(board({
      srclk: "clk",
      rclk: "later",
      oe: "oe",
      sources: [clock(), pulse("oe", { v1: 5, v2: 0, td: 100e-3 }), pulse("later", { td: 100e-3 })],
    }));
    // SER sits at 5 V (no data source), so nine clocks fill the register
    // with 1s and QH' goes high with the outputs disabled.
    stepTo(engine, 9e-3);
    expect(engine.getIcState("u1")?.shift).toBe(0xff);
    expect(engine.digitalState["u1/qh2"]).toBe(1);
    expect(pinVoltage(engine, "u1", "qh2")).toBeGreaterThan(4.5);
  });

  it("keeps QH' on the shift register when tied clocks lag the storage register", () => {
    const engine = new SimEngine();
    engine.load(board({ srclk: "clk", rclk: "clk", ser: "data", sources: [clock(), data()] }));
    // After the tenth tied clock the outputs show the register from clock 9
    // (0xB6, SCLS041J section 8.1) while the shift register holds 0x6D:
    // storage QH is high, but QH', upstream of the storage register, shows
    // the shift register's own top bit: 0x6D bit 7 = 0. One clock later the
    // two have swapped, so QH' cannot be following the storage register.
    stepTo(engine, 10e-3);
    expect(registers(engine)).toEqual({ shift: 0x6d, latch: 0xb6, outputs: 0xb6 });
    expect(pinVoltage(engine, "u1", "qh")).toBeGreaterThan(4.5);
    expect(pinVoltage(engine, "u1", "qh2")).toBeLessThan(0.5);

    stepTo(engine, 11e-3);
    expect(registers(engine)).toEqual({ shift: 0xdb, latch: 0x6d, outputs: 0x6d });
    expect(pinVoltage(engine, "u1", "qh")).toBeLessThan(0.5);
    expect(pinVoltage(engine, "u1", "qh2")).toBeGreaterThan(4.5);
  });
});

// ── Hand assembly (ATmega328P; D8, D11 and D12 are PB0, PB3 and PB4) ────────
const DDRB = 0x04;
const PORTB = 0x05;
const RCLK_BIT = 0; // D8
const SER_BIT = 3; // D11
const SRCLK_BIT = 4; // D12
const avr = {
  ldi: (d: number, k: number) => 0xe000 | ((k & 0xf0) << 4) | ((d - 16) << 4) | (k & 0x0f),
  out: (a: number, r: number) => 0xb800 | ((a & 0x30) << 5) | (r << 4) | (a & 0x0f),
  sbi: (a: number, bit: number) => 0x9a00 | (a << 3) | bit,
  cbi: (a: number, bit: number) => 0x9800 | (a << 3) | bit,
  sbiw: (d: 24 | 26 | 28 | 30, k: number) => 0x9700 | ((k & 0x30) << 2) | (((d - 24) >> 1) << 4) | (k & 0x0f),
  brne: (k: number) => 0xf401 | ((k & 0x7f) << 3),
  /** An rjmp to itself, so the program halts there. */
  halt: 0xcfff,
};

/** Intel HEX for a list of 16-bit program words, from address 0. */
function intelHex(words: number[]): string {
  const bytes = words.flatMap((word) => [word & 0xff, (word >> 8) & 0xff]);
  const lines: string[] = [];
  for (let at = 0; at < bytes.length; at += 16) {
    const data = bytes.slice(at, at + 16);
    const record = [data.length, (at >> 8) & 0xff, at & 0xff, 0x00, ...data];
    const checksum = -record.reduce((sum, b) => sum + b, 0) & 0xff;
    lines.push(":" + [...record, checksum].map((b) => b.toString(16).toUpperCase().padStart(2, "0")).join(""));
  }
  lines.push(":00000001FF");
  return lines.join("\n") + "\n";
}

const SETUP = [
  avr.ldi(16, (1 << RCLK_BIT) | (1 << SER_BIT) | (1 << SRCLK_BIT)),
  avr.out(DDRB, 16),
  avr.ldi(16, 1 << SER_BIT),
  avr.out(PORTB, 16),
];
// A 1000-pass sbiw loop: 4 cycles a pass, 250 us at 16 MHz.
const DELAY = [
  avr.ldi(24, 1000 & 0xff),
  avr.ldi(25, 1000 >> 8),
  avr.sbiw(24, 1),
  avr.brne(-2),
];

/**
 * shiftOut(dataPin, clockPin, MSBFIRST, byte) for each byte of `bytes`,
 * with a 250 us delay after every clock so each SRCLK pulse the program
 * makes lands in its own MCU step at 100 us stepping.
 */
function pacedShiftOut(bytes: number[]): string {
  const words = [...SETUP];
  for (const byte of bytes) {
    for (let i = 7; i >= 0; i--) {
      words.push(((byte >> i) & 1) ? avr.sbi(PORTB, SER_BIT) : avr.cbi(PORTB, SER_BIT));
      words.push(avr.sbi(PORTB, SRCLK_BIT), avr.cbi(PORTB, SRCLK_BIT));
      words.push(...DELAY);
    }
  }
  words.push(avr.sbi(PORTB, RCLK_BIT), avr.cbi(PORTB, RCLK_BIT), avr.halt);
  return intelHex(words);
}

describe("74HC595 QH' daisy chain (MCU-driven)", () => {
  it("fills both chips when an Arduino shifts 16 bits, one clock per step", () => {
    const engine = new SimEngine();
    engine.load({
      components: [
        {
          id: "uno",
          kind: "arduino_uno",
          pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
          params: { vcc: 5, hex: pacedShiftOut([0xb2, 0x4c]), usb_power: 1 },
        },
        chip("u1"),
        chip("u2"),
      ],
      wires: [
        ...supplyWires("u1", { vccFrom: ["uno", "5v"], gndFrom: ["uno", "gnd"] }),
        ...supplyWires("u2", { vccFrom: ["uno", "5v"], gndFrom: ["uno", "gnd"] }),
        wire("u1", "qh2", "u2", "ser"),
        wire("uno", "d11", "u1", "ser"),
        wire("uno", "d12", "u1", "srclk"),
        wire("uno", "d8", "u1", "rclk"),
        wire("uno", "d12", "u2", "srclk"),
        wire("uno", "d8", "u2", "rclk"),
      ],
    });
    // Byte 1 (0xB2) fills U1 while U2 shifts in the zeros behind U1's
    // empty register; byte 2 (0x4C) pushes byte 1 out of QH' into U2.
    let u2AtByte1 = -1;
    let qh2StateAtByte1 = -1;
    let qh2NetAfterByte1 = Number.NaN;
    for (let i = 0; i < 60; i++) {
      engine.step(100e-6);
      if (qh2StateAtByte1 >= 0 && Number.isNaN(qh2NetAfterByte1)) {
        // One step on: the stamp now carries the register U1 committed.
        qh2NetAfterByte1 = pinVoltage(engine, "u1", "qh2");
      }
      const shift = engine.getIcState("u1")?.shift ?? 0;
      if (shift === 0xb2 && qh2StateAtByte1 < 0) {
        u2AtByte1 = engine.getIcState("u2")?.shift ?? -1;
        // Same pass: the replayed register publishes QH' at once.
        qh2StateAtByte1 = engine.digitalState["u1/qh2"] ?? -1;
      }
    }
    expect(u2AtByte1, "far register while byte 1 sits in the near one").toBe(0x00);
    expect(qh2StateAtByte1, "QH' as byte 1's last bit leaves").toBe(1);
    expect(qh2NetAfterByte1).toBeGreaterThan(4.5);

    expect(registers(engine, "u1")).toEqual({ shift: 0x4c, latch: 0x4c, outputs: 0x4c });
    expect(registers(engine, "u2")).toEqual({ shift: 0xb2, latch: 0xb2, outputs: 0xb2 });
    expect(pinVoltage(engine, "u2", "qh")).toBeGreaterThan(4.5);
    expect(pinVoltage(engine, "u1", "qh")).toBeLessThan(0.5);
    // QH' settles at 0x4C bit 7 = 0.
    expect(pinVoltage(engine, "u1", "qh2")).toBeLessThan(0.5);
  });
});

describe("74HC595 QH' small-signal stage", () => {
  it("stamps the same Thevenin rolloff at QH' as at an enabled output", () => {
    // U1 with SER at 5 V: nine clocks fill the shift register, so QH'
    // rests high through the package output stage. A capacitor from QH' to
    // ground then sees the stage's Thevenin resistance to the VCC rail,
    // exactly as QA-QH do: the documented io_max-derived Rout.
    const ioMax = catalog.parts.find((p) => p.kind === "74hc595")?.electrical_specs?.io_max;
    expect(ioMax).toBeDefined();
    expect(ioMax!).toBeGreaterThan(0);
    const rOut = Math.max(GATE_R_OUT, Math.min(1000, 5 / (ioMax! * 4)));
    const g = 1 / rOut;
    const c = 1e-6;
    const fPole = g / (2 * Math.PI * c);

    const engine = new SimEngine();
    engine.load({
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        chip("u1"),
        clock(),
        part("c_qh2", "capacitor", ["a", "b"], { capacitance: c }),
      ],
      wires: [
        ...supplyWires("u1"),
        wire("vcc", "pos", "u1", "ser"),
        wire("clk", "pos", "u1", "srclk"),
        wire("clk", "neg", "vcc", "neg"),
        wire("u1", "qh2", "c_qh2", "a"),
        wire("c_qh2", "b", "vcc", "neg"),
      ],
    });
    stepTo(engine, 10e-3);
    expect(engine.getIcState("u1")?.shift).toBe(0xff);
    expect(pinVoltage(engine, "u1", "qh2")).toBeGreaterThan(4.5);

    const net = engine.nets.find((n) => n.pins.some(([comp, pin]) => comp === "u1" && pin === "qh2"))!.id;
    const freqs = [0.05, 0.2, 1, 4, 20].map((k) => k * fPole);
    const ss = runSmallSignalAc(engine, { inputId: "vcc", outputNetIds: [net], frequenciesHz: freqs });
    const out = ss.outputs[0]!;
    for (let i = 0; i < freqs.length; i++) {
      const omega = 2 * Math.PI * freqs[i]!;
      // QH'-node KCL: g to the unit-driven VCC rail, C plus the universal
      // shunt to ground -> H = g / (g + G_shunt + jwC).
      const denRe = g + NODE_RSHUNT_G;
      const denIm = omega * c;
      const expectedMag = g / Math.hypot(denRe, denIm);
      const mag = Math.hypot(out.re[i]!, out.im[i]!);
      expect(Math.abs(mag / expectedMag - 1), `QH' rolloff magnitude at ${String(freqs[i])} Hz`).toBeLessThan(1e-9);
    }
  });
});
