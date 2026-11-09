/**
 * Display decoders count each MCU step's pin events exactly once.
 *
 * The max7219 and hd44780 decoders replay an MCU's ordered sub-step pin
 * events through ctx.mergedDisplayEvents. Two defects, both measured here:
 *
 * 1. DOUBLE REPLAY under the NE555 step split: a step with a 555 threshold
 *    crossing solves three times (rolled-back trial + two split halves) and
 *    the merged-events path re-merged the same events on every pass, so a
 *    program sending 16 visible bits produced 32. The clocked-IC path
 *    (_mcuEdgesPending, mcu-clocked-ics.test.ts) already replays once; the
 *    display path now rides the same consumed-once lifecycle.
 *
 * 2. STALE REPLAY after the MCU loses power: an unpowered core never runs
 *    step(), the only place arduino.ts clears its event list, so its last
 *    powered step's events re-replayed on every later step and the bit
 *    count grew without bound (32 -> 48 -> 64 -> 80 ...). The engine now
 *    clears a skipped core's events at the advance that skips it.
 *
 * The Arduino programs are hand-assembled ATmega328P instructions (see
 * mcu-clocked-ics.test.ts for the opcode helpers); the HD44780 cases reuse
 * the compiled hd44780_hello.hex fixture from wave8-hd44780.test.ts.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

type Component = SimCircuit["components"][number];

const HD44780_HEX = readFileSync(join(__dirname, "__fixtures__/hd44780_hello.hex"), "utf-8");

function wire(from: string, fromPin: string, to: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: from, from_pin: fromPin, to_component: to, to_pin: toPin };
}

function part(id: string, kind: string, pins: string[], params: Record<string, number> = {}): Component {
  return { id, kind, pins: pins.map((pin) => ({ id: pin })), params };
}

function uno(id: string, hex: string, usbPower: number): Component {
  return {
    id,
    kind: "arduino_uno",
    pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
    params: { vcc: 5, hex, usb_power: usbPower },
  };
}

/**
 * An NE555 astable hanging off an existing "vdd" 5 V source (timing values
 * from mcu-clocked-ics.test.ts: the 10 nF cap charges through 1 k + 9.1 k,
 * so threshold crossings land inside ordinary steps and split them into a
 * rolled-back trial plus two halves — three solves).
 */
function ne555Astable(): Pick<SimCircuit, "components" | "wires"> {
  return {
    components: [
      part("ra", "resistor", ["a", "b"], { resistance: 1_000 }),
      part("rb", "resistor", ["a", "b"], { resistance: 9_100 }),
      part("ct", "capacitor", ["a", "b"], { capacitance: 10e-9 }),
      part("timer", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]),
    ],
    wires: [
      wire("vdd", "pos", "timer", "8"),
      wire("vdd", "pos", "timer", "4"),
      wire("vdd", "neg", "timer", "1"),
      wire("vdd", "pos", "ra", "a"),
      wire("ra", "b", "timer", "7"),
      wire("timer", "7", "rb", "a"),
      wire("rb", "b", "timer", "6"),
      wire("timer", "6", "timer", "2"),
      wire("timer", "6", "ct", "a"),
      wire("ct", "b", "vdd", "neg"),
    ],
  };
}

/** The display's bits counter (clocks counted since the last CS falling edge). */
function maxBits(engine: SimEngine, id = "mx"): number {
  return engine.getIcState(id)?.bits ?? -1;
}

// ── Hand assembly (ATmega328P; D10 = PB2, D11 = PB3, D13 = PB5) ───────────────
const DDRB = 0x04;
const PORTB = 0x05;
const avr = {
  ldi: (d: number, k: number) => 0xe000 | ((k & 0xf0) << 4) | ((d - 16) << 4) | (k & 0x0f),
  out: (a: number, r: number) => 0xb800 | ((a & 0x30) << 5) | (r << 4) | (a & 0x0f),
  sbi: (a: number, b: number) => 0x9a00 | (a << 3) | b,
  cbi: (a: number, b: number) => 0x9800 | (a << 3) | b,
  sbiw: (d: 24 | 26 | 28 | 30, k: number) => 0x9700 | ((k & 0x30) << 2) | (((d - 24) >> 1) << 4) | (k & 0x0f),
  brne: (k: number) => 0xf401 | ((k & 0x7f) << 3),
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

// The Uno drives the MAX7219 like the probe program: CS (D10) falls in the
// MCU's first step, a 250 us delay loop (1000 passes, 4 cycles each) moves
// the 16 CLK pulses (D13) into a later step with DIN (D11) low, then the
// program halts with CS still low. Each clock rising edge shifts one bit, so
// a decoder that sees each edge once counts bits = 16.
const MAX_FRAME = intelHex([
  avr.ldi(16, 0x2c), // DDRB: D10, D11, D13 outputs
  avr.out(DDRB, 16),
  avr.ldi(16, 0x04), // CS high, DIN and CLK low
  avr.out(PORTB, 16),
  avr.cbi(PORTB, 2), // CS falls: frame opens
  avr.ldi(24, 1000 & 0xff),
  avr.ldi(25, 1000 >> 8),
  avr.sbiw(24, 1),
  avr.brne(-2),
  ...Array.from({ length: 16 }, () => [avr.sbi(PORTB, 5), avr.cbi(PORTB, 5)]).flat(),
  avr.halt,
]);

/** An Uno (usbPower) driving a MAX7219 powered from its own 5 V source. */
function max7219Circuit(usbPower: number, opts: { timer?: boolean; bystander?: boolean } = {}): SimCircuit {
  const timer = opts.timer ? ne555Astable() : { components: [], wires: [] };
  return {
    components: [
      uno("uno", MAX_FRAME, usbPower),
      part("mx", "max7219", ["din", "clk", "cs", "vcc", "gnd"]),
      part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      ...(opts.bystander ? [uno("idle", intelHex([avr.halt]), 1)] : []),
      ...timer.components,
    ],
    wires: [
      wire("vdd", "pos", "mx", "vcc"),
      wire("vdd", "neg", "mx", "gnd"),
      wire("vdd", "neg", "uno", "gnd"),
      wire("uno", "d11", "mx", "din"),
      wire("uno", "d13", "mx", "clk"),
      wire("uno", "d10", "mx", "cs"),
      ...(opts.bystander ? [wire("vdd", "neg", "idle", "gnd")] : []),
      ...timer.wires,
    ],
  };
}

/** An Uno running hd44780_hello.hex against an HD44780 in 4-bit wiring. */
function hd44780Circuit(opts: { timer?: boolean } = {}): SimCircuit {
  const timer = opts.timer ? ne555Astable() : { components: [], wires: [] };
  return {
    components: [
      uno("uno", HD44780_HEX, 1),
      part("lcd", "hd44780", [
        "vss", "vdd", "v0", "rs", "rw", "e",
        "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
        "a", "k",
      ]),
      ...(opts.timer ? [part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 })] : []),
      ...timer.components,
    ],
    wires: [
      wire("uno", "5v", "lcd", "vdd"),
      wire("uno", "gnd", "lcd", "vss"),
      wire("uno", "gnd", "lcd", "rw"),
      wire("uno", "d8", "lcd", "rs"),
      wire("uno", "d9", "lcd", "e"),
      wire("uno", "d4", "lcd", "d4"),
      wire("uno", "d5", "lcd", "d5"),
      wire("uno", "d6", "lcd", "d6"),
      wire("uno", "d7", "lcd", "d7"),
      ...(opts.timer ? [wire("vdd", "neg", "uno", "gnd")] : []),
      ...timer.wires,
    ],
  };
}

describe("MAX7219 driven by an Arduino", () => {
  it("counts each clock edge once when an NE555 splits the step", () => {
    // The 16 clocks are replayed inside a step the 555 splits into three
    // solves; a decoder that re-merged the events on every pass counted 32.
    const engine = new SimEngine();
    engine.load(max7219Circuit(1, { timer: true }));
    let solvesAtFirstBits = 0;
    let bits = 0;
    for (let i = 0; i < 6; i++) {
      const solvesBefore = engine.electricalSolveCount;
      engine.step(100e-6);
      const solves = engine.electricalSolveCount - solvesBefore;
      const nowBits = maxBits(engine);
      if (nowBits > 0 && bits === 0) solvesAtFirstBits = solves;
      bits = nowBits;
    }
    expect(solvesAtFirstBits).toBe(3); // the replay really ran inside a split step
    expect(bits).toBe(16);
  });

  it("counts each clock edge once without a 555", () => {
    const engine = new SimEngine();
    engine.load(max7219Circuit(1));
    for (let i = 0; i < 6; i++) engine.step(100e-6);
    expect(maxBits(engine)).toBe(16);
  });
});

// The same frame, plus five pulses on D9 after it: one Uno clocks a MAX7219
// (the merged display-event consumer) and a 74LS161 (the clocked-IC
// consumer) in one program. Both draw on the same once-per-step event
// handout, so a refactor that let one consumer eat the other's share, or
// re-merged on a later pass, changes one of the two counts.
const MAX_FRAME_PLUS_COUNTER = intelHex([
  avr.ldi(16, 0x2e), // DDRB: D9, D10, D11, D13 outputs
  avr.out(DDRB, 16),
  avr.ldi(16, 0x04), // CS high, DIN, CLK and D9 low
  avr.out(PORTB, 16),
  avr.cbi(PORTB, 2), // CS falls: frame opens
  avr.ldi(24, 1000 & 0xff),
  avr.ldi(25, 1000 >> 8),
  avr.sbiw(24, 1),
  avr.brne(-2),
  ...Array.from({ length: 16 }, () => [avr.sbi(PORTB, 5), avr.cbi(PORTB, 5)]).flat(),
  ...Array.from({ length: 5 }, () => [avr.sbi(PORTB, 1), avr.cbi(PORTB, 1)]).flat(),
  avr.halt,
]);

/** An Uno clocking a MAX7219 on D13 and a 74LS161 on D9, with a 555 present. */
function maxAndCounterCircuit(): SimCircuit {
  const timer = ne555Astable();
  return {
    components: [
      uno("uno", MAX_FRAME_PLUS_COUNTER, 1),
      part("mx", "max7219", ["din", "clk", "cs", "vcc", "gnd"]),
      part(
        "u161",
        "74ls161",
        ["clk", "/clr", "/load", "enp", "ent", "a", "b", "c", "d", "qa", "qb", "qc", "qd", "rco", "vcc", "gnd"],
      ),
      part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      ...timer.components,
    ],
    wires: [
      wire("vdd", "pos", "mx", "vcc"),
      wire("vdd", "neg", "mx", "gnd"),
      wire("vdd", "neg", "uno", "gnd"),
      wire("uno", "d11", "mx", "din"),
      wire("uno", "d13", "mx", "clk"),
      wire("uno", "d10", "mx", "cs"),
      wire("uno", "d9", "u161", "clk"),
      wire("vdd", "pos", "u161", "vcc"),
      wire("vdd", "neg", "u161", "gnd"),
      wire("vdd", "pos", "u161", "/clr"),
      wire("vdd", "pos", "u161", "/load"),
      wire("vdd", "pos", "u161", "enp"),
      wire("vdd", "pos", "u161", "ent"),
      ...(["a", "b", "c", "d"] as const).map((pin) => wire("vdd", "neg", "u161", pin)),
      ...timer.wires,
    ],
  };
}

describe("a display and a clocked IC on one microcontroller", () => {
  it("each consumes the step's events exactly once under the 555 split", () => {
    const engine = new SimEngine();
    engine.load(maxAndCounterCircuit());
    let solvesAtFirstBits = 0;
    let bits = 0;
    for (let i = 0; i < 6; i++) {
      const solvesBefore = engine.electricalSolveCount;
      engine.step(100e-6);
      const solves = engine.electricalSolveCount - solvesBefore;
      const nowBits = maxBits(engine);
      if (nowBits > 0 && bits === 0) solvesAtFirstBits = solves;
      bits = nowBits;
    }
    expect(solvesAtFirstBits).toBe(3); // the split step, as above
    expect(bits).toBe(16); // the display's share, once
    expect(engine.getIcState("u161")?.count ?? -1).toBe(5); // the counter's share, once
  });
});

describe("HD44780 driven by an Arduino", () => {
  it("shows the sketch's message when an NE555 splits the steps", () => {
    // The sketch's init nibbles and message bytes are replayed inside split
    // steps; a second replay latched every E pulse twice and garbled the
    // message. 1 ms steps carry many 555 crossings (the astable runs at
    // ~750 Hz), so most steps split.
    const engine = new SimEngine();
    engine.load(hd44780Circuit({ timer: true }));
    for (let i = 0; i < 150; i++) engine.step(1e-3);
    const lcd = engine.getDisplayState()["lcd"];
    expect(lcd?.kind).toBe("hd44780");
    if (lcd?.kind !== "hd44780") throw new Error("expected hd44780 DisplayInfo");
    expect(lcd.lines[0]).toBe("Hello, World!   ");
    expect(lcd.lines[1]).toBe("                ");
    expect(lcd.on).toBe(true);
  });

  it("shows the sketch's message without a 555", () => {
    const engine = new SimEngine();
    engine.load(hd44780Circuit());
    for (let i = 0; i < 150; i++) engine.step(1e-3);
    const lcd = engine.getDisplayState()["lcd"];
    expect(lcd?.kind).toBe("hd44780");
    if (lcd?.kind !== "hd44780") throw new Error("expected hd44780 DisplayInfo");
    expect(lcd.lines[0]).toBe("Hello, World!   ");
    expect(lcd.lines[1]).toBe("                ");
    expect(lcd.on).toBe(true);
  });
});

describe("display events after the Arduino loses power", () => {
  it("counts the last powered step's clocks once, then stops, after a USB unplug", () => {
    // Three powered steps run the program: the 16 clocks land in the LAST
    // advance, so they are still waiting to be replayed when the USB plug
    // comes out (reloading with usb_power 0 keeps the core but unpowers
    // it, and it never steps again). The waiting clocks are counted exactly
    // once; at the base they re-replayed on every step after the unplug
    // (32 -> 48 -> 64 -> 80).
    const engine = new SimEngine();
    engine.load(max7219Circuit(1));
    for (let i = 0; i < 3; i++) engine.step(100e-6);
    expect(maxBits(engine)).toBe(0); // the clocks ran, but nobody replayed them yet
    engine.load(max7219Circuit(0));
    const seen: number[] = [];
    for (let i = 0; i < 4; i++) {
      engine.step(100e-6);
      seen.push(maxBits(engine));
    }
    expect(seen).toEqual([16, 16, 16, 16]);
  });

  it("stops replaying them even while another powered Arduino keeps stepping", () => {
    // The idle Uno stays powered and keeps advancing, which re-arms the
    // edges-pending flag every step; unless the unpowered core's event list
    // is cleared, its stale clocks re-merge on every one of those steps.
    const engine = new SimEngine();
    engine.load(max7219Circuit(1, { bystander: true }));
    for (let i = 0; i < 3; i++) engine.step(100e-6);
    expect(maxBits(engine)).toBe(0);
    engine.load(max7219Circuit(0, { bystander: true }));
    const seen: number[] = [];
    for (let i = 0; i < 4; i++) {
      engine.step(100e-6);
      seen.push(maxBits(engine));
    }
    expect(seen).toEqual([16, 16, 16, 16]);
  });
});
