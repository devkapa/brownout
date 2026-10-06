/**
 * A program that reads a clocked chip between two of its own writes sees the
 * level those writes produced (H11 readback).
 *
 * The committed digital pass replays an MCU's step only at the END of the
 * step, so during the step the chip's outputs still stand where the last
 * solve left them: every digitalRead() of a 74HC165's Q7 inside one step
 * returned the same pre-step level, and a shiftIn() read garbage (0xFF at
 * 100 us steps with the register holding 0xB2). While a registered clocked
 * part shares nets with the MCU — its clocks driven BY the core, its outputs
 * readable BY the same core — the engine now runs a shadow of the part
 * inside the core's step: each GPIO write the core captures drives the same
 * replay walker the committed pass uses, and the part's output levels are
 * pushed back into the core's input path, where the next read finds them.
 *
 * shiftin_165.hex, shiftin_165_second.hex and shiftin_165_partial.hex are
 * the .ino files beside them, built with arduino-cli 1.4.1 (arduino:avr
 * 1.8.7, `arduino-cli compile --fqbn arduino:avr:uno`). The programs pulse
 * /PL (D8) to load 0xB2 from D0-D7, then read Q7 (D10) MSB first, sampling
 * before each CP (D9) rise — the 74HC165 shifts on the rise, so Q7 holds a
 * bit until its own clock — and drive what they read out to a 74HC595
 * (SER D11, SRCLK D12, RCLK D13) for the engine to observe. Circuits
 * without a part the MCU both writes and reads, including an MCU with no
 * clocked part at all, hash byte-identically to 5b43ffb (digests below).
 */
import { describe, expect, it } from "vitest";
import { createHash } from "node:crypto";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";
import { setMcuFirmware, setRp2040Module } from "../../../src/sim/engine/mcu.js";
import { RP2040Mcu } from "../../../src/sim/engine/rp2040.js";

type Component = SimCircuit["components"][number];

const HEX_FULL = readFileSync(join(__dirname, "__fixtures__/shiftin_165.hex"), "utf-8");
const HEX_PART = readFileSync(join(__dirname, "__fixtures__/shiftin_165_partial.hex"), "utf-8");
const HEX_SECOND = readFileSync(join(__dirname, "__fixtures__/shiftin_165_second.hex"), "utf-8");
const HEX_595 = readFileSync(join(__dirname, "__fixtures__/shiftout_595.hex"), "utf-8");
const HEX_BLINK = readFileSync(join(__dirname, "__fixtures__/blink.hex"), "utf-8");

const HC165_PINS = [
  "/pl", "cp", "d4", "d5", "d6", "d7", "/q7", "gnd",
  "ds", "d0", "d1", "d2", "d3", "q7", "cp_inh", "vcc",
];
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

/**
 * An Uno, a 74HC165 loaded with 0xB2 on D0-D7 and a 74HC595 the program
 * drives its answer to. D8 is /PL, D9 is CP, D10 reads Q7; with `feedback`
 * DS also sits on Q7, so the register rotates instead of shifting in zeros.
 */
function readbackBoard(hex: string, opts: { feedback?: boolean } = {}): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex, usb_power: 1 },
      },
      part("u165", "74hc165", HC165_PINS),
      part("u595", "74hc595", HC595_PINS),
    ],
    wires: [
      wire("uno", "5v", "u165", "vcc"), wire("uno", "gnd", "u165", "gnd"),
      wire("uno", "gnd", "u165", "cp_inh"),
      wire("uno", "gnd", "u165", "d0"), wire("uno", "5v", "u165", "d1"),
      wire("uno", "gnd", "u165", "d2"), wire("uno", "gnd", "u165", "d3"),
      wire("uno", "5v", "u165", "d4"), wire("uno", "5v", "u165", "d5"),
      wire("uno", "gnd", "u165", "d6"), wire("uno", "5v", "u165", "d7"),
      wire("uno", "d8", "u165", "/pl"), wire("uno", "d9", "u165", "cp"),
      wire("u165", "q7", "uno", "d10"),
      wire("uno", "5v", "u595", "vcc"), wire("uno", "gnd", "u595", "gnd"),
      wire("uno", "5v", "u595", "/srclr"), wire("uno", "gnd", "u595", "/oe"),
      wire("uno", "d11", "u595", "ser"), wire("uno", "d12", "u595", "srclk"),
      wire("uno", "d13", "u595", "rclk"),
      ...(opts.feedback
        ? [wire("u165", "q7", "u165", "ds")]
        : [wire("uno", "gnd", "u165", "ds")]),
    ],
  };
}

/** The byte the program read, on the 595's QA (bit 0) to QH (bit 7). */
function outputs(engine: SimEngine, id = "u595"): number {
  return OUTPUTS.reduce((byte, pin, bit) => byte | ((engine.digitalState[`${id}/${pin}`] ?? 0) << bit), 0);
}

function runFixed(circuit: SimCircuit, h: number, durationS: number): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  const steps = Math.round(durationS / h);
  for (let i = 0; i < steps; i++) engine.step(h);
  return engine;
}

/** Readback shadows installed this run (0 before H11 added the counter). */
function shadowInstalls(engine: SimEngine): number {
  return (engine as { readbackShadowInstalls?: number }).readbackShadowInstalls ?? 0;
}

// ── Identity digests (measured at 5b43ffb, the H10 base) ────────────────────
// Every step's digitalState, netV and icState, hashed in order; the readback
// machinery must not move a bit of any circuit it does not shadow.
const f64 = (v: number) => {
  const b = Buffer.alloc(8);
  b.writeDoubleLE(v);
  return b.toString("hex");
};
const sorted = (o: Record<string, unknown>) => Object.keys(o).sort().map((k) => [k, o[k]]);
function stepRecord(engine: SimEngine, circuit: SimCircuit): string {
  const ic = circuit.components
    .map((comp) => [comp.id, engine.getIcState(comp.id)] as const)
    .filter(([, st]) => st !== undefined)
    .sort(([a], [b]) => (a < b ? -1 : 1))
    .map(([id, st]) => [id, sorted(st!).map(([k, v]) => [k, typeof v === "number" ? f64(v) : v])]);
  const nv = sorted(engine.getNetV()).map(([k, v]) => [k, f64(v as number)]);
  const ds = sorted(engine.digitalState as unknown as Record<string, unknown>).map(([k, v]) => [k, f64(v as number)]);
  return JSON.stringify({ ds, nv, ic });
}
function runDigest(circuit: SimCircuit, h: number, durationS: number): string {
  const engine = new SimEngine();
  engine.load(circuit);
  const digest = createHash("sha1");
  const steps = Math.round(durationS / h);
  for (let i = 0; i < steps; i++) {
    engine.step(h);
    digest.update(stepRecord(engine, circuit) + "\n");
  }
  return digest.digest("hex");
}

const MCU_595_DIGEST = "535282133d26a0dbc7f5eb8007a36e88c17c324f";
const MCU_BLINK_DIGEST = "0373b0ba84688f5566f0537005a6cc90bd452ab9";
const NON_MCU_165_DIGEST = "c42a66548f8d2b0a4d184f7919b2ed29824ea4f7";

describe("an Arduino reading a 74HC165 mid-step", () => {
  for (const [h, label] of [[100e-6, "100 us"], [20e-6, "20 us"]] as const) {
    it(`returns the byte the register held, stepping ${label}`, () => {
      // 0xB2 loaded on /PL, two passes read MSB first (DS grounded: the
      // second pass reads zeros), ORed and shifted out: 0xB2.
      const engine = runFixed(readbackBoard(HEX_FULL), h, 5e-3);
      expect(outputs(engine)).toBe(0xb2);
      expect(shadowInstalls(engine)).toBeGreaterThan(0);
      // The committed register is the same one the shadow replayed: sixteen
      // clocks of a grounded DS empty it.
      expect((engine.getIcState("u165") ?? {}).shift).toBe(0x00);
    });

    it(`keeps the byte with DS on Q7, stepping ${label}`, () => {
      // With DS on Q7 each CP rise rotates Q7 back in, so the register
      // still holds 0xB2 after sixteen clocks. shiftin_165_second drives
      // only its SECOND pass out: the rotation reads 0xB2 there (with DS
      // grounded that fixture shows 0x00 — the register has emptied).
      const engine = runFixed(readbackBoard(HEX_SECOND, { feedback: true }), h, 5e-3);
      expect(outputs(engine)).toBe(0xb2);
      expect((engine.getIcState("u165") ?? {}).shift).toBe(0xb2);
    });

    it(`empties the register into a second pass, stepping ${label}`, () => {
      // The same second-pass fixture with DS grounded: sixteen clocks have
      // shifted the byte all the way out.
      const engine = runFixed(readbackBoard(HEX_SECOND), h, 5e-3);
      expect(outputs(engine)).toBe(0x00);
      expect((engine.getIcState("u165") ?? {}).shift).toBe(0x00);
    });

    it(`reads the first half of a stream, stepping ${label}`, () => {
      // Four reads then stop: 0b1011, so the 595 shows 0x0B.
      const engine = runFixed(readbackBoard(HEX_PART), h, 5e-3);
      expect(outputs(engine)).toBe(0x0b);
    });
  }

  it("shows the byte under the host's adaptive stepping", () => {
    const runner = new HeadlessRunner();
    runner.load(readbackBoard(HEX_FULL));
    runner.run({ durationS: 5e-3 });
    expect(outputs(runner.engine)).toBe(0xb2);
  });

  it("does not shadow a 74HC165 the MCU cannot read", () => {
    // The H10 fixture: /PL and CP driven, Q7 and /Q7 unconnected. No output
    // net reaches the MCU, so no shadow installs and the committed replay
    // alone carries the part, exactly as before.
    const program = readbackH10Program();
    const engine = runFixed(program.circuit, 100e-6, 1e-3);
    expect((engine.getIcState("u165") ?? {}).shift).toBe(0x90);
    expect(shadowInstalls(engine)).toBe(0);
  });
});

// Hand assembly (see mcu-clocked-ics.test.ts): D8 (PB0) is /PL, D9 (PB1) is
// CP. /PL pulses low to load 0xB2, then three CP pulses with DS grounded
// shift it to (0xB2 << 3) & 0xff = 0x90.
const DDRB = 0x04;
const PORTB = 0x05;
const avr = {
  ldi: (d: number, k: number) => 0xe000 | ((k & 0xf0) << 4) | ((d - 16) << 4) | (k & 0x0f),
  out: (a: number, r: number) => 0xb800 | ((a & 0x30) << 5) | (r << 4) | (a & 0x0f),
  sbi: (a: number, bit: number) => 0x9a00 | (a << 3) | bit,
  cbi: (a: number, bit: number) => 0x9800 | (a << 3) | bit,
  halt: 0xcfff,
};
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
function readbackH10Program(): { circuit: SimCircuit } {
  const program = intelHex([
    avr.ldi(16, 0x03),
    avr.out(DDRB, 16),
    avr.sbi(PORTB, 0),
    avr.cbi(PORTB, 0),
    avr.sbi(PORTB, 0),
    avr.sbi(PORTB, 1),
    avr.cbi(PORTB, 1),
    avr.sbi(PORTB, 1),
    avr.cbi(PORTB, 1),
    avr.sbi(PORTB, 1),
    avr.cbi(PORTB, 1),
    avr.halt,
  ]);
  return {
    circuit: {
      components: [
        {
          id: "uno",
          kind: "arduino_uno",
          pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
          params: { vcc: 5, hex: program, usb_power: 1 },
        },
        part("u165", "74hc165", HC165_PINS),
      ],
      wires: [
        wire("uno", "5v", "u165", "vcc"), wire("uno", "gnd", "u165", "gnd"),
        wire("uno", "gnd", "u165", "cp_inh"), wire("uno", "gnd", "u165", "ds"),
        wire("uno", "gnd", "u165", "d0"), wire("uno", "5v", "u165", "d1"),
        wire("uno", "gnd", "u165", "d2"), wire("uno", "gnd", "u165", "d3"),
        wire("uno", "5v", "u165", "d4"), wire("uno", "5v", "u165", "d5"),
        wire("uno", "gnd", "u165", "d6"), wire("uno", "5v", "u165", "d7"),
        wire("uno", "d8", "u165", "/pl"), wire("uno", "d9", "u165", "cp"),
      ],
    },
  };
}

describe("circuits the readback must not touch", () => {
  it("runs an MCU with a 74HC595 byte-identically (no 165 present)", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "uno",
          kind: "arduino_uno",
          pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
          params: { vcc: 5, hex: HEX_595, usb_power: 1 },
        },
        part("u1", "74hc595", HC595_PINS),
      ],
      wires: [
        wire("uno", "5v", "u1", "vcc"), wire("uno", "gnd", "u1", "gnd"),
        wire("uno", "5v", "u1", "/srclr"), wire("uno", "gnd", "u1", "/oe"),
        wire("uno", "d11", "u1", "ser"), wire("uno", "d12", "u1", "srclk"),
        wire("uno", "d8", "u1", "rclk"),
      ],
    };
    const engine = runFixed(circuit, 100e-6, 5e-3);
    expect(outputs(engine, "u1")).toBe(0xb2);
    expect(shadowInstalls(engine)).toBe(0);
    expect(runDigest(circuit, 100e-6, 5e-3)).toBe(MCU_595_DIGEST);
  });

  it("runs an MCU in isolation byte-identically", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "uno",
          kind: "arduino_uno",
          pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
          params: { vcc: 5, hex: HEX_BLINK, usb_power: 1 },
        },
        part("r1", "resistor", ["a", "b"], { resistance: 330 }),
        part("led1", "led", ["a", "k"]),
      ],
      wires: [
        wire("uno", "d13", "r1", "a"), wire("r1", "b", "led1", "a"),
        wire("led1", "k", "uno", "gnd"),
      ],
    };
    const engine = runFixed(circuit, 100e-6, 0.2);
    expect(shadowInstalls(engine)).toBe(0);
    expect(runDigest(circuit, 100e-6, 0.2)).toBe(MCU_BLINK_DIGEST);
  });

  it("reads a 74HC165 with no MCU at step end byte-identically", () => {
    // D0-D7 on rails (0xB2), /PL high, DS high, CP from a 1 kHz clock
    // source: the leading half-period plus five periods in 5 ms give six
    // rising edges, so the register reads 0x3F, read once per solve exactly
    // as before.
    const circuit: SimCircuit = {
      components: [
        part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("u165", "74hc165", HC165_PINS),
        part("clk", "clock_gen", ["out", "gnd"], { frequency: 1000, duty: 0.5, v_high: 5, v_low: 0 }),
      ],
      wires: [
        wire("vdd", "pos", "u165", "vcc"), wire("vdd", "neg", "u165", "gnd"),
        wire("vdd", "pos", "u165", "/pl"), wire("vdd", "neg", "u165", "cp_inh"),
        wire("vdd", "neg", "u165", "d0"), wire("vdd", "pos", "u165", "d1"),
        wire("vdd", "neg", "u165", "d2"), wire("vdd", "neg", "u165", "d3"),
        wire("vdd", "pos", "u165", "d4"), wire("vdd", "pos", "u165", "d5"),
        wire("vdd", "neg", "u165", "d6"), wire("vdd", "pos", "u165", "d7"),
        wire("vdd", "pos", "u165", "ds"),
        wire("clk", "out", "u165", "cp"), wire("clk", "gnd", "vdd", "neg"),
      ],
    };
    const engine = runFixed(circuit, 100e-6, 5e-3);
    expect((engine.getIcState("u165") ?? {}).shift).toBe(0x3f);
    expect(shadowInstalls(engine)).toBe(0);
    expect(runDigest(circuit, 100e-6, 5e-3)).toBe(NON_MCU_165_DIGEST);
  });
});

describe("a Raspberry Pi Pico reading a 74HC165 mid-step", () => {
  it("shows the byte a MicroPython read loop read, stepping 100 us", () => {
    setMcuFirmware(
      "raspberry_pi_pico",
      new Uint8Array(readFileSync(fileURLToPath(new URL("../../fixtures/firmware/pico-micropython.uf2", import.meta.url)))),
    );
    setRp2040Module(RP2040Mcu);
    // The same program as shiftin_165.ino in MicroPython: a /PL pulse on
    // GP8 loads 0xB2, eight reads of Q7 (GP10), each taken before the CP
    // (GP9) rise that shifts it on, then the byte is bit-banged out to a
    // 74HC595 on GP11-GP13. Interpreted MicroPython spreads the loop over
    // several steps, so reads land both inside a step with their clock and
    // at the head of the next one.
    const script = [
      "from machine import Pin",
      "pl = Pin(8, Pin.OUT, value=1)",
      "cp = Pin(9, Pin.OUT, value=0)",
      "data = Pin(10, Pin.IN)",
      "ser = Pin(11, Pin.OUT, value=0)",
      "srclk = Pin(12, Pin.OUT, value=0)",
      "rclk = Pin(13, Pin.OUT, value=0)",
      "pl.value(0)",
      "pl.value(1)",
      "value = 0",
      "for i in range(8):",
      "    value = (value << 1) | data.value()",
      "    cp.value(1)",
      "    cp.value(0)",
      "for i in range(8):",
      "    ser.value((value >> (7 - i)) & 1)",
      "    srclk.value(1)",
      "    srclk.value(0)",
      "rclk.value(1)",
      "rclk.value(0)",
      "",
    ].join("\r\n");
    const circuit: SimCircuit = {
      components: [
        {
          id: "pico",
          kind: "raspberry_pi_pico",
          pins: ["gp8", "gp9", "gp10", "gp11", "gp12", "gp13", "3v3", "gnd"].map((pin) => ({ id: pin })),
          params: { vcc: 3.3, usb_power: 1, script },
        },
        part("u165", "74hc165", HC165_PINS),
        part("u595", "74hc595", HC595_PINS),
      ],
      wires: [
        wire("pico", "3v3", "u165", "vcc"), wire("pico", "gnd", "u165", "gnd"),
        wire("pico", "gnd", "u165", "cp_inh"),
        wire("pico", "gnd", "u165", "d0"), wire("pico", "3v3", "u165", "d1"),
        wire("pico", "gnd", "u165", "d2"), wire("pico", "gnd", "u165", "d3"),
        wire("pico", "3v3", "u165", "d4"), wire("pico", "3v3", "u165", "d5"),
        wire("pico", "gnd", "u165", "d6"), wire("pico", "3v3", "u165", "d7"),
        wire("pico", "gnd", "u165", "ds"),
        wire("pico", "gp8", "u165", "/pl"), wire("pico", "gp9", "u165", "cp"),
        wire("u165", "q7", "pico", "gp10"),
        wire("pico", "3v3", "u595", "vcc"), wire("pico", "gnd", "u595", "gnd"),
        wire("pico", "3v3", "u595", "/srclr"), wire("pico", "gnd", "u595", "/oe"),
        wire("pico", "gp11", "u595", "ser"), wire("pico", "gp12", "u595", "srclk"),
        wire("pico", "gp13", "u595", "rclk"),
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    // Booting MicroPython and pasting the script takes simulated time; stop
    // once the outputs show the byte, or give up after 0.6 s.
    for (let i = 0; i < 6000 && outputs(engine) !== 0xb2; i++) engine.step(100e-6);
    for (let i = 0; i < 10; i++) engine.step(100e-6);
    expect(outputs(engine)).toBe(0xb2);
    expect((engine.getIcState("u165") ?? {}).shift).toBe(0x00);
    expect(shadowInstalls(engine)).toBeGreaterThan(0);
  }, 60_000);
});
