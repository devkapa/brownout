/**
 * A clocked IC driven by a microcontroller sees every edge the firmware
 * makes, including pulses that start and end inside one MCU step.
 *
 * The solver sees an MCU's pins only as they stand at the end of each MCU
 * step, and MCU circuits step at up to 100 us. A 74HC595 that read its
 * clocks once per solve therefore missed every SRCLK pulse of an Arduino
 * shiftOut() (each about 4 us wide) and the latch pulse after them, and
 * stayed dark; counters and flip-flops missed every pulse a program made
 * with two digitalWrite() calls. These chips now replay the MCU's
 * cycle-stamped pin edges before they read their step-end levels.
 *
 * shiftout_595.hex is shiftout_595.ino built with arduino-cli 1.4.1
 * (arduino:avr 1.8.7, `arduino-cli compile --fqbn arduino:avr:uno`): setup()
 * shifts 0xB2 MSB first with shiftOut() on D11 (SER) and D12 (SRCLK), then
 * pulses D8 (RCLK) once. The other programs are a few hand-assembled
 * ATmega328P instructions, listed beside each test.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";
import { setMcuFirmware, setRp2040Module } from "../../../src/sim/engine/mcu.js";
import { RP2040Mcu } from "../../../src/sim/engine/rp2040.js";

type Component = SimCircuit["components"][number];

const SHIFTOUT_HEX = readFileSync(join(__dirname, "__fixtures__/shiftout_595.hex"), "utf-8");

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
 * An Uno and a 5 V 74HC595 with /OE low and /SRCLR high. D11 drives SER and
 * D12 SRCLK; RCLK hangs on D8, or on D12 with SRCLK when the clocks are tied.
 */
function unoBoard(hex: string, opts: { tiedClocks?: boolean; extra?: SimCircuit } = {}): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex, usb_power: 1 },
      },
      part("u1", "74hc595", HC595_PINS),
      ...(opts.extra?.components ?? []),
    ],
    wires: [
      wire("uno", "5v", "u1", "vcc"),
      wire("uno", "gnd", "u1", "gnd"),
      wire("uno", "5v", "u1", "/srclr"),
      wire("uno", "gnd", "u1", "/oe"),
      wire("uno", "d11", "u1", "ser"),
      wire("uno", "d12", "u1", "srclk"),
      wire("uno", opts.tiedClocks ? "d12" : "d8", "u1", "rclk"),
      ...(opts.extra?.wires ?? []),
    ],
  };
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

function runFixed(circuit: SimCircuit, h: number, durationS: number): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  const steps = Math.round(durationS / h);
  for (let i = 0; i < steps; i++) engine.step(h);
  return engine;
}

// ── Hand assembly ────────────────────────────────────────────────────────────
// Opcodes from the AVR instruction set manual. The Uno's D8, D11 and D12 are
// PB0, PB3 and PB4, so every program here writes port B.
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

/** Set SER, RCLK and SRCLK as outputs with SER high and both clocks low. */
const SETUP = [
  avr.ldi(16, (1 << RCLK_BIT) | (1 << SER_BIT) | (1 << SRCLK_BIT)),
  avr.out(DDRB, 16),
  avr.ldi(16, 1 << SER_BIT),
  avr.out(PORTB, 16),
];
const CLOCK_PULSE = [avr.sbi(PORTB, SRCLK_BIT), avr.cbi(PORTB, SRCLK_BIT)];
const LATCH_PULSE = [avr.sbi(PORTB, RCLK_BIT), avr.cbi(PORTB, RCLK_BIT)];

describe("74HC595 driven by an Arduino", () => {
  // Hand-derived from the sketch: eight clocks shift 1,0,1,1,0,0,1,0 into QA
  // and on toward QH, so the shift register holds 0b10110010 = 0xB2, and the
  // latch pulse after them copies it to the outputs: QH, QF, QE and QB high.
  const expected = { shift: 0xb2, latch: 0xb2, outputs: 0xb2 };

  for (const [h, label] of [[100e-6, "100 us"], [20e-6, "20 us"]] as const) {
    it(`shows the byte shiftOut() sent, stepping ${label}`, () => {
      const engine = runFixed(unoBoard(SHIFTOUT_HEX), h, 5e-3);
      expect(registers(engine)).toEqual(expected);
      expect(pinVoltage(engine, "u1", "qh")).toBeGreaterThan(4.5);
      expect(pinVoltage(engine, "u1", "qa")).toBeLessThan(0.5);
    });
  }

  it("shows the byte shiftOut() sent under the host's adaptive stepping", () => {
    const runner = new HeadlessRunner();
    runner.load(unoBoard(SHIFTOUT_HEX));
    runner.run({ durationS: 5e-3 });
    expect(registers(runner.engine)).toEqual(expected);
  });

  it("stores the byte from before the shift when one port write raises SRCLK and RCLK", () => {
    // SETUP; two clock pulses shift in 1, 1 (0x03); then ldi r16,0x19 and
    // out PORTB,r16 raise SER, SRCLK and RCLK in one write. On hardware the
    // two clock edges are simultaneous, so the storage register samples the
    // shift register before that edge's shift reaches it: 0x03, while the
    // shift register moves on to 0x07.
    const program = intelHex([
      ...SETUP,
      ...CLOCK_PULSE,
      ...CLOCK_PULSE,
      avr.ldi(16, (1 << RCLK_BIT) | (1 << SER_BIT) | (1 << SRCLK_BIT)),
      avr.out(PORTB, 16),
      avr.ldi(16, 1 << SER_BIT),
      avr.out(PORTB, 16),
      avr.halt,
    ]);
    const engine = runFixed(unoBoard(program), 100e-6, 1e-3);
    expect(registers(engine)).toEqual({ shift: 0x07, latch: 0x03, outputs: 0x03 });
  });

  it("stores the shifted byte when RCLK rises in a later write", () => {
    // As above, but sbi PORTB,4 then sbi PORTB,0 raise SRCLK and, two
    // cycles (125 ns) later, RCLK: the storage register takes 0x07.
    const program = intelHex([
      ...SETUP,
      ...CLOCK_PULSE,
      ...CLOCK_PULSE,
      avr.sbi(PORTB, SRCLK_BIT),
      avr.sbi(PORTB, RCLK_BIT),
      avr.ldi(16, 1 << SER_BIT),
      avr.out(PORTB, 16),
      avr.halt,
    ]);
    const engine = runFixed(unoBoard(program), 100e-6, 1e-3);
    expect(registers(engine)).toEqual({ shift: 0x07, latch: 0x07, outputs: 0x07 });
  });

  it("shifts in the SER level from before a write that changes SER and raises SRCLK", () => {
    // SETUP leaves SER high; ldi r16,0x10 and out PORTB,r16 drop SER and
    // raise SRCLK in one write. The shift register samples SER as it was
    // before the write, so a 1 goes in; a latch pulse then shows 0x01.
    const program = intelHex([
      ...SETUP,
      avr.ldi(16, 1 << SRCLK_BIT),
      avr.out(PORTB, 16),
      avr.ldi(16, 0),
      avr.out(PORTB, 16),
      ...LATCH_PULSE,
      avr.halt,
    ]);
    const engine = runFixed(unoBoard(program), 100e-6, 1e-3);
    expect(registers(engine)).toEqual({ shift: 0x01, latch: 0x01, outputs: 0x01 });
  });

  it("keeps the storage register one clock behind when SRCLK and RCLK are tied", () => {
    // SETUP; three clock pulses on the tied net. Each edge stores the shift
    // register from before its own shift (TI SN74HC595, SCLS041J, 8.1).
    const program = intelHex([...SETUP, ...CLOCK_PULSE, ...CLOCK_PULSE, ...CLOCK_PULSE, avr.halt]);
    const engine = runFixed(unoBoard(program, { tiedClocks: true }), 100e-6, 1e-3);
    expect(registers(engine)).toEqual({ shift: 0x07, latch: 0x03, outputs: 0x03 });
  });

  for (const [h, label] of [[100e-6, "100 us"], [20e-6, "20 us"]] as const) {
    it(`samples SER at the level an earlier step left it, stepping ${label}`, () => {
      // SETUP raises SER in the MCU's first step. A 1000-pass delay loop
      // (4 cycles a pass, 250 us) moves the rest into a later step: a clock
      // pulse with no SER write before it shifts in that old 1, cbi PORTB,3
      // drops SER, a second pulse shifts in 0, and a latch pulse shows 0x02.
      const program = intelHex([
        ...SETUP,
        avr.ldi(24, 1000 & 0xff),
        avr.ldi(25, 1000 >> 8),
        avr.sbiw(24, 1),
        avr.brne(-2),
        ...CLOCK_PULSE,
        avr.cbi(PORTB, SER_BIT),
        ...CLOCK_PULSE,
        ...LATCH_PULSE,
        avr.halt,
      ]);
      const engine = runFixed(unoBoard(program), h, 1e-3);
      expect(registers(engine)).toEqual({ shift: 0x02, latch: 0x02, outputs: 0x02 });
    });
  }

  it("replays each MCU step once when an NE555 splits the step", () => {
    // SETUP; three clock pulses shift in 1, 1, 1; a latch pulse stores 0x07.
    // The 555 astable charges its 10 nF timing capacitor through 1 kohm and
    // 9.1 kohm (tau 101 us) from 0 V, and backward Euler at 100 us steps puts
    // it at 2.49 V after the first step and 3.74 V after the second, so the
    // first 3.33 V threshold crossing falls inside the second step, the step
    // that replays the program's edges. The engine splits that step and
    // solves it three times. A second replay would shift three more 1s in
    // (0x3F); losing the replay to the rolled-back trial would leave 0x00.
    const timer: SimCircuit = {
      components: [
        part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("ra", "resistor", ["a", "b"], { resistance: 1_000 }),
        part("rb", "resistor", ["a", "b"], { resistance: 9_100 }),
        part("ct", "capacitor", ["a", "b"], { capacitance: 10e-9 }),
        part("timer", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]),
      ],
      wires: [
        wire("vdd", "pos", "timer", "8"),
        wire("vdd", "pos", "timer", "4"),
        wire("vdd", "neg", "timer", "1"),
        wire("vdd", "neg", "uno", "gnd"),
        wire("vdd", "pos", "ra", "a"),
        wire("ra", "b", "timer", "7"),
        wire("timer", "7", "rb", "a"),
        wire("rb", "b", "timer", "6"),
        wire("timer", "6", "timer", "2"),
        wire("timer", "6", "ct", "a"),
        wire("ct", "b", "vdd", "neg"),
      ],
    };
    const program = intelHex([...SETUP, ...CLOCK_PULSE, ...CLOCK_PULSE, ...CLOCK_PULSE, ...LATCH_PULSE, avr.halt]);
    const engine = new SimEngine();
    engine.load(unoBoard(program, { extra: timer }));
    engine.step(100e-6);
    const solvesBefore = engine.electricalSolveCount;
    engine.step(100e-6);
    expect(engine.electricalSolveCount - solvesBefore).toBe(3);
    expect(registers(engine)).toEqual({ shift: 0x07, latch: 0x07, outputs: 0x07 });
    for (let i = 0; i < 8; i++) engine.step(100e-6);
    expect(registers(engine)).toEqual({ shift: 0x07, latch: 0x07, outputs: 0x07 });
  });
});

describe("other clocked chips driven by an Arduino", () => {
  // Each program pulses its pins back to back (two cycles high), so every
  // pulse starts and ends inside the MCU's first step. Pins are on port B:
  // D8 = PB0, D9 = PB1, ... D12 = PB4.
  const pulse = (bit: number) => [avr.sbi(PORTB, bit), avr.cbi(PORTB, bit)];
  const pulses = (bit: number, n: number) => Array.from({ length: n }, () => pulse(bit)).flat();
  const outputsOn = (bits: number) => [avr.ldi(16, bits), avr.out(DDRB, 16)];

  /** An Uno running `words` and one chip `kind`, wired by `wires` (Uno pins on the left). */
  function board(words: number[], kind: string, pins: string[], wires: Array<[string, string]>): SimCircuit {
    return {
      components: [
        {
          id: "uno",
          kind: "arduino_uno",
          pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
          params: { vcc: 5, hex: intelHex(words), usb_power: 1 },
        },
        part("u", kind, pins),
      ],
      wires: wires.map(([from, to]) => (from.startsWith("u.") ? wire("u", from.slice(2), "u", to) : wire("uno", from, "u", to))),
    };
  }
  const icState = (circuit: SimCircuit) => runFixed(circuit, 100e-6, 1e-3).getIcState("u") ?? {};

  const CD4017 = ["q5", "q1", "q0", "q2", "q6", "q7", "q3", "gnd", "q8", "q4", "q9", "co", "clkinh", "clk", "reset", "vcc"];
  const cd4017 = (count: number, reset: [string, string]) =>
    board([...outputsOn(1), ...pulses(0, count), avr.halt], "cd4017", CD4017, [
      ["5v", "vcc"], ["gnd", "gnd"], ["gnd", "clkinh"], ["d8", "clk"], reset,
    ]);

  it("counts every clock pulse on a CD4017", () => {
    expect(icState(cd4017(3, ["gnd", "reset"])).count).toBe(3);
  });

  it("wraps a CD4017 at five when RESET is wired to its own Q5", () => {
    // Pulses 1-4 count to 4; the fifth reaches 5, Q5 resets the counter at
    // once, and pulses six and seven leave it at 2.
    expect(icState(cd4017(7, ["u.q5", "reset"])).count).toBe(2);
  });

  const HC74 = ["clr1_n", "d1", "clk1", "pre1_n", "q1", "q1_n", "gnd", "q2_n", "q2", "pre2_n", "clk2", "d2", "clr2_n", "vcc"];

  it("stores D on a 74HC74 clock pulse", () => {
    // D1 (D9) high, one CLK1 (D8) pulse, then D1 low again.
    const st = icState(board(
      [...outputsOn(0x03), avr.sbi(PORTB, 1), ...pulse(0), avr.cbi(PORTB, 1), avr.halt],
      "74hc74",
      HC74,
      [["5v", "vcc"], ["gnd", "gnd"], ["5v", "clr1_n"], ["5v", "pre1_n"], ["5v", "clr2_n"], ["5v", "pre2_n"],
        ["gnd", "d2"], ["gnd", "clk2"], ["d8", "clk1"], ["d9", "d1"]],
    ));
    expect([st.q1, st.q1n]).toEqual([1, 0]);
  });

  it("counts on a two-bit ripple counter inside one 74HC74", () => {
    // D1 = /Q1, CLK2 = /Q1 and D2 = /Q2. Both /CLR sit on D9, low until the
    // program releases them; then CLK1 pulses count up from 0. After one
    // pulse /Q1 has fallen, which does not clock the second flip-flop, even
    // though the solve still shows /Q1 high when the pulse is replayed.
    const ripple = (count: number) => icState(board(
      [...outputsOn(0x03), avr.sbi(PORTB, 1), ...pulses(0, count), avr.halt],
      "74hc74",
      HC74,
      [["5v", "vcc"], ["gnd", "gnd"], ["d9", "clr1_n"], ["5v", "pre1_n"], ["d9", "clr2_n"], ["5v", "pre2_n"],
        ["d8", "clk1"], ["u.q1_n", "d1"], ["u.q1_n", "clk2"], ["u.q2_n", "d2"]],
    ));
    const one = ripple(1);
    const three = ripple(3);
    expect([one.q2, one.q1]).toEqual([0, 1]);
    expect([three.q2, three.q1]).toEqual([1, 1]);
  });

  const LS161 = ["/clr", "clk", "a", "b", "c", "d", "enp", "gnd", "rco", "qd", "qc", "qb", "qa", "/load", "ent", "vcc"];

  it("counts every clock pulse on a 74LS161", () => {
    const st = icState(board([...outputsOn(1), ...pulses(0, 5), avr.halt], "74ls161", LS161, [
      ["5v", "vcc"], ["gnd", "gnd"], ["5v", "/clr"], ["5v", "/load"], ["5v", "enp"], ["5v", "ent"],
      ["gnd", "a"], ["gnd", "b"], ["gnd", "c"], ["gnd", "d"], ["d8", "clk"],
    ]));
    expect(st.count).toBe(5);
  });

  it("loads a 74LS161 through /LOAD and counts on from there", () => {
    // A-D wired to 0, 1, 0, 1 (10). /LOAD (D9) low for one clock loads 10;
    // three more clocks count to 13.
    const st = icState(board(
      [...outputsOn(0x03), avr.sbi(PORTB, 1), avr.cbi(PORTB, 1), ...pulse(0), avr.sbi(PORTB, 1), ...pulses(0, 3), avr.halt],
      "74ls161",
      LS161,
      [["5v", "vcc"], ["gnd", "gnd"], ["5v", "/clr"], ["5v", "enp"], ["5v", "ent"],
        ["gnd", "a"], ["5v", "b"], ["gnd", "c"], ["5v", "d"], ["d8", "clk"], ["d9", "/load"]],
    ));
    expect(st.count).toBe(13);
  });

  it("stores D1-D4 on a 74LS173 clock pulse", () => {
    // D1-D4 on D9-D12 set to 1, 1, 0, 1 in one write, one CLK (D8) pulse,
    // then all four data lines drop.
    const LS173 = ["m", "n", "q1", "q2", "q3", "q4", "clk", "gnd", "/clr", "d1", "d2", "d3", "d4", "g1", "g2", "vcc"];
    const st = icState(board(
      [...outputsOn(0x1f), avr.ldi(16, 0b10110), avr.out(PORTB, 16), ...pulse(0), avr.ldi(16, 0), avr.out(PORTB, 16), avr.halt],
      "74ls173",
      LS173,
      [["5v", "vcc"], ["gnd", "gnd"], ["gnd", "m"], ["gnd", "n"], ["gnd", "/clr"], ["gnd", "g1"], ["gnd", "g2"],
        ["d8", "clk"], ["d9", "d1"], ["d10", "d2"], ["d11", "d3"], ["d12", "d4"]],
    ));
    expect([st.q1, st.q2, st.q3, st.q4]).toEqual([1, 1, 0, 1]);
  });

  it("loads and shifts a 74HC165 on /PL and CP pulses", () => {
    // D0-D7 wired to 0xB2. /PL (D8) pulses low to load it, then three CP
    // (D9) pulses with DS low shift it to 0xB2 << 3 & 0xFF = 0x90.
    const HC165 = ["/pl", "cp", "d4", "d5", "d6", "d7", "/q7", "gnd", "ds", "d0", "d1", "d2", "d3", "q7", "cp_inh", "vcc"];
    const st = icState(board(
      [...outputsOn(0x03), avr.sbi(PORTB, 0), avr.cbi(PORTB, 0), avr.sbi(PORTB, 0), ...pulses(1, 3), avr.halt],
      "74hc165",
      HC165,
      [["5v", "vcc"], ["gnd", "gnd"], ["gnd", "cp_inh"], ["gnd", "ds"],
        ["gnd", "d0"], ["5v", "d1"], ["gnd", "d2"], ["gnd", "d3"], ["5v", "d4"], ["5v", "d5"], ["gnd", "d6"], ["5v", "d7"],
        ["d8", "/pl"], ["d9", "cp"]],
    ));
    expect(st.shift).toBe(0x90);
  });
});

describe("74HC595 driven by a Raspberry Pi Pico", () => {
  it("shows the byte a MicroPython bit-bang sent, stepping 100 us", () => {
    setMcuFirmware(
      "raspberry_pi_pico",
      new Uint8Array(readFileSync(fileURLToPath(new URL("../../fixtures/firmware/pico-micropython.uf2", import.meta.url)))),
    );
    setRp2040Module(RP2040Mcu);
    // Each Pin.value() call takes a few microseconds of interpreted
    // MicroPython, so every pulse here is far shorter than a step.
    const script = [
      "from machine import Pin",
      "ser = Pin(10, Pin.OUT)",
      "clk = Pin(11, Pin.OUT)",
      "lat = Pin(12, Pin.OUT)",
      "b = 0xB2",
      "for i in range(8):",
      "    ser.value((b >> (7 - i)) & 1)",
      "    clk.value(1)",
      "    clk.value(0)",
      "lat.value(1)",
      "lat.value(0)",
      "",
    ].join("\r\n");
    const engine = new SimEngine();
    engine.load({
      components: [
        {
          id: "pico",
          kind: "raspberry_pi_pico",
          pins: ["gp10", "gp11", "gp12", "3v3", "gnd"].map((pin) => ({ id: pin })),
          params: { vcc: 3.3, usb_power: 1, script },
        },
        part("u1", "74hc595", HC595_PINS),
      ],
      wires: [
        wire("pico", "3v3", "u1", "vcc"),
        wire("pico", "gnd", "u1", "gnd"),
        wire("pico", "3v3", "u1", "/srclr"),
        wire("pico", "gnd", "u1", "/oe"),
        wire("pico", "gp10", "u1", "ser"),
        wire("pico", "gp11", "u1", "srclk"),
        wire("pico", "gp12", "u1", "rclk"),
      ],
    });
    // Booting MicroPython and pasting the script takes simulated time; stop
    // once the outputs show the byte, or give up after 0.6 s.
    for (let i = 0; i < 6000 && registers(engine).outputs !== 0xb2; i++) engine.step(100e-6);
    for (let i = 0; i < 10; i++) engine.step(100e-6);
    expect(registers(engine)).toEqual({ shift: 0xb2, latch: 0xb2, outputs: 0xb2 });
  }, 60_000);
});
