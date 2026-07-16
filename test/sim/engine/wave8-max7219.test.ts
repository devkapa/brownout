/**
 * Wave 8.1 — MAX7219 8x8 LED matrix driver engine tests.
 *
 * max7219_frame.hex: an Arduino sketch that initialises a MAX7219 and writes a
 * fixed 8x8 pattern during setup(), then halts.  The fixture ships pre-compiled
 * (built once with arduino-cli, arduino:avr); see the Wave 8 build log in
 * memory/stages/stage-6-component-libraries.md.
 *
 * Sketch register sequence (in order of transmission, LSB-first inside each
 * 16-bit frame, framing is CS↓ … 16 clocks … CS↑):
 *   0x0C = 0x01  shutdown → normal operation
 *   0x09 = 0x00  decode mode → raw matrix bits (not BCD)
 *   0x0A = 0x08  intensity → 8 (mid)
 *   0x0B = 0x07  scan limit → 7 (all 8 rows active)
 *   0x01 = 0x3C  digit 0 / row 0 = 0b00111100
 *   0x02 = 0x42  digit 1 / row 1 = 0b01000010
 *   0x03 = 0x99  digit 2 / row 2 = 0b10011001
 *   0x04 = 0x00  digit 3 / row 3 = 0b00000000
 *   0x05 = 0x00  digit 4 / row 4 = 0b00000000
 *   0x06 = 0x00  digit 5 / row 5 = 0b00000000
 *   0x07 = 0x00  digit 6 / row 6 = 0b00000000
 *   0x08 = 0x81  digit 7 / row 7 = 0b10000001
 *
 * Closed-form expected display state:
 *   rows:      [0x3C, 0x42, 0x99, 0x00, 0x00, 0x00, 0x00, 0x81]
 *   intensity: 8
 *   on:        true   (shutdown = 1 = normal)
 *
 * ─── DERIVATIONS FOR SAMPLED FALLBACK TEST ────────────────────────────────────
 *
 * The sampled fallback path processes one pin edge per step.  To write a single
 * 16-bit frame we need at minimum 16 CLK-rising steps plus the CS framing steps.
 * We use discrete voltage sources to drive the three SPI pins.
 *
 * To write register 0x0C = 0x01 (shutdown) over the sampled path:
 *   Frame = 0x0C01 = 0b0000_1100_0000_0001
 *
 * Steps to drive a single 16-bit frame with sampled fallback (CS starts high):
 *   1. CS = 0  (frame start, shift/bits reset)
 *   2. Bit 15..0 of 0x0C01 each require a CLK-low → CLK-high transition over
 *      two steps (one step with CLK=0 and DIN=bit, next step with CLK=1).
 *      16 bits × 2 steps = 32 steps.
 *   3. CS = 1  (CS rising with bits≥16 → latch)
 *
 * Because the sampled path takes one edge per step, the test drives the full
 * sequence programmatically via separate voltage sources for CS, CLK, DIN,
 * one step per edge transition.
 *
 * After the frame 0x0C = 0x01:
 *   shutdown = 1 → on = true.
 *   All dig registers remain at default 0; intensity = 0; scanLimit = 0.
 *   Row 0 (dig0) is active (r=0 ≤ scanLimit=0), rows 1-7 are blanked.
 *   Expected: { kind:"max7219", rows:[0,0,0,0,0,0,0,0], intensity:0, on:true }
 *
 * NOTE: Because `on` is only visible once shutdown=1, the sampled test asserts
 * on=true and rows all-zero (default; no digit frames sent in this test).
 */

import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

const max7219Hex = readFileSync(
  join(__dirname, "__fixtures__/max7219_frame.hex"),
  "utf-8",
);

// ─── Test circuit builder helpers ────────────────────────────────────────────

/**
 * Builds a SimCircuit with an Arduino Uno running max7219_frame.hex and a
 * MAX7219 module wired to it:
 *   DIN  ← D11 (MOSI)
 *   CLK  ← D13 (SCK)
 *   CS   ← D10 (~SS)
 *   VCC  ← 5V header
 *   GND  ← GND header
 */
function max7219ArduinoCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex: max7219Hex, usb_power: 1 },
      },
      {
        id: "mx",
        kind: "max7219",
        pins: [{ id: "din" }, { id: "clk" }, { id: "cs" }, { id: "vcc" }, { id: "gnd" }],
        params: {},
      },
    ],
    wires: [
      // Power
      { from_component: "uno", from_pin: "5v",  to_component: "mx", to_pin: "vcc" },
      { from_component: "uno", from_pin: "gnd", to_component: "mx", to_pin: "gnd" },
      // SPI lines (hardware SPI pins on the Uno)
      { from_component: "uno", from_pin: "d11", to_component: "mx", to_pin: "din" },
      { from_component: "uno", from_pin: "d13", to_component: "mx", to_pin: "clk" },
      { from_component: "uno", from_pin: "d10", to_component: "mx", to_pin: "cs" },
    ],
  };
}

/**
 * Builds a SimCircuit that drives the MAX7219 directly from three discrete
 * voltage sources (CS, CLK, DIN) plus a 5 V supply.  The voltage values are
 * mutable params so the test can reconfigure them between steps.
 *
 * Returns the circuit and the three voltage-source component IDs.
 */
function max7219DiscreteDriveCircuit(
  csV: number,
  clkV: number,
  dinV: number,
): SimCircuit {
  return {
    components: [
      // 5 V supply
      {
        id: "pwr",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      // CS driver (active-low: 0 V = selected, 5 V = deselected)
      {
        id: "vs_cs",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: csV },
      },
      // CLK driver
      {
        id: "vs_clk",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: clkV },
      },
      // DIN driver
      {
        id: "vs_din",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: dinV },
      },
      // MAX7219 module
      {
        id: "mx",
        kind: "max7219",
        pins: [{ id: "din" }, { id: "clk" }, { id: "cs" }, { id: "vcc" }, { id: "gnd" }],
        params: {},
      },
    ],
    wires: [
      // VCC / GND
      { from_component: "pwr",    from_pin: "pos", to_component: "mx",     to_pin: "vcc" },
      { from_component: "pwr",    from_pin: "neg", to_component: "vs_cs",  to_pin: "neg" },
      { from_component: "vs_cs",  from_pin: "neg", to_component: "vs_clk", to_pin: "neg" },
      { from_component: "vs_clk", from_pin: "neg", to_component: "vs_din", to_pin: "neg" },
      { from_component: "vs_din", from_pin: "neg", to_component: "mx",     to_pin: "gnd" },
      // SPI signals
      { from_component: "vs_cs",  from_pin: "pos", to_component: "mx", to_pin: "cs"  },
      { from_component: "vs_clk", from_pin: "pos", to_component: "mx", to_pin: "clk" },
      { from_component: "vs_din", from_pin: "pos", to_component: "mx", to_pin: "din" },
    ],
  };
}

// ─── Test 1: Event-path integration (Arduino + max7219) ──────────────────────

describe("MAX7219 — event-path integration (Arduino Uno)", () => {
  it("decodes the max7219_frame.hex sketch display state after one 20 ms step", () => {
    const engine = new SimEngine();
    engine.load(max7219ArduinoCircuit());

    // 20 ms contains 320 000 AVR cycles at 16 MHz.  The sketch's setup() burst
    // (SPI init + 12 frames × 16 bits × ~10 instructions each ≈ 2 000 cycles)
    // is well within that window.  After setup() the sketch halts in loop().
    engine.step(0.02);
    // External CPU execution commits only after the accepted electrical
    // interval. One minimum follow-up solve consumes that interval's ordered
    // PinEvents and applies the resulting display protocol state.
    engine.step(1e-8);

    const ds = engine.getDisplayState();
    const mx = ds["mx"] as { kind: string; rows: number[]; intensity: number; on: boolean } | undefined;

    expect(mx).toBeDefined();
    expect(mx?.kind).toBe("max7219");

    // Narrow the union so TypeScript knows we have the max7219 variant.
    if (mx?.kind !== "max7219") throw new Error("expected max7219 DisplayInfo");

    // Closed-form expected values — derived from the sketch source (see file header).
    //   row 0: 0x3C = 0b00111100 → columns 2,3,4,5 lit
    //   row 1: 0x42 = 0b01000010 → columns 1,6 lit
    //   row 2: 0x99 = 0b10011001 → columns 0,3,4,7 lit
    //   rows 3-6: 0x00 = blank
    //   row 7: 0x81 = 0b10000001 → columns 0,7 lit
    expect(mx.rows).toEqual([0x3c, 0x42, 0x99, 0x00, 0x00, 0x00, 0x00, 0x81]);

    // intensity: 0x0A register was written with 8 → mid brightness
    expect(mx.intensity).toBe(8);

    // on: shutdown register 0x0C = 0x01 → normal operation (not shutdown)
    expect(mx.on).toBe(true);
  });
});

// ─── Test 2: Sampled fallback — shutdown frame only ───────────────────────────

describe("MAX7219 — sampled fallback (discrete voltage sources)", () => {
  it("latches shutdown=normal after 16-bit frame 0x0C01 via one-edge-per-step sequence", () => {
    // Frame to write: register 0x0C (shutdown), data 0x01 → exit shutdown.
    // 16-bit word: (0x0C << 8) | 0x01 = 0x0C01 = 0b0000_1100_0000_0001
    const FRAME = 0x0c01;

    // Construct the initial circuit with CS=high (deselected), CLK=low, DIN=low.
    const circuit = max7219DiscreteDriveCircuit(5, 0, 0);
    const engine = new SimEngine();
    engine.load(circuit);

    // One brief step at idle to establish committed levels (CS=high → lastCS=1).
    engine.step(1e-4);

    // Utility: mutate a voltage source's voltage and advance one step.
    // The sampled path reads one pin edge per step via _logicHigh() comparisons
    // against the committed prev state, so each step = one protocol transition.
    function setVoltages(csV: number, clkV: number, dinV: number): void {
      const c = circuit.components;
      c.find((x) => x.id === "vs_cs")!.params.voltage  = csV;
      c.find((x) => x.id === "vs_clk")!.params.voltage = clkV;
      c.find((x) => x.id === "vs_din")!.params.voltage = dinV;
      // Reload to propagate param changes, then step.
      engine.load(circuit);
      engine.step(1e-4);
    }

    // Step 1: CS falls → frame start (shift=0, bits=0 inside the engine).
    setVoltages(0, 0, 0);

    // Steps 2-33: clock in 16 bits of FRAME MSB-first.
    // For each bit: set DIN, then raise CLK (rising edge latches bit), then lower CLK.
    // But the sampled path compares committed lastCLK to current cClk, so each
    // "set CLK low with new DIN" + "set CLK high" pair = 2 steps = 1 bit.
    for (let bit = 15; bit >= 0; bit--) {
      const din = (FRAME >> bit) & 1;
      const dinV = din ? 5 : 0;
      // Step A: CLK=0, DIN=bit — committed prev CLK stays at previous level.
      setVoltages(0, 0, dinV);
      // Step B: CLK=1, DIN=bit — rising edge: shift in the bit.
      setVoltages(0, 5, dinV);
    }

    // Step 34: CS rises → latch (bits=16 triggers applyRegister).
    setVoltages(5, 0, 0);

    const ds = engine.getDisplayState();
    const mx = ds["mx"] as { kind: string; rows: number[]; intensity: number; on: boolean } | undefined;

    expect(mx).toBeDefined();
    expect(mx?.kind).toBe("max7219");

    // Narrow the union so TypeScript knows we have the max7219 variant.
    if (mx?.kind !== "max7219") throw new Error("expected max7219 DisplayInfo");

    // After 0x0C=0x01: shutdown=1 → on=true.
    // No digit registers written → all rows = 0 (default).
    // scanLimit default = 0 → only row 0 (r=0 ≤ 0) is active; dig0=0.
    // intensity default = 0.
    // Closed-form derivation: all dig registers remain at initialised 0.
    expect(mx.on).toBe(true);
    expect(mx.intensity).toBe(0);
    // scanLimit=0: row 0 active (dig0=0), rows 1-7 blanked because r > 0 = scanLimit.
    expect(mx.rows).toEqual([0, 0, 0, 0, 0, 0, 0, 0]);
  });
});

// ─── Test 3: Shutdown / power behaviour ──────────────────────────────────────

describe("MAX7219 — shutdown and power behaviour", () => {
  it("reports on=false when powered but no shutdown frame sent (default shutdown=0)", () => {
    // Circuit: Arduino with max7219 but the sketch is blink.hex, which never
    // drives the SPI pins with a MAX7219 init sequence.  The default icState
    // has shutdown=0 (factory default: chip starts in shutdown mode).
    // We use a simple idle circuit: voltage_source + max7219 with CS held high.
    const circuit: SimCircuit = {
      components: [
        {
          id: "pwr",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        {
          id: "vs_cs",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },  // CS=high: not selected, no frames transmitted
        },
        {
          id: "vs_clk",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 0 },
        },
        {
          id: "vs_din",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 0 },
        },
        {
          id: "mx",
          kind: "max7219",
          pins: [{ id: "din" }, { id: "clk" }, { id: "cs" }, { id: "vcc" }, { id: "gnd" }],
          params: {},
        },
      ],
      wires: [
        { from_component: "pwr",    from_pin: "pos", to_component: "mx",     to_pin: "vcc" },
        { from_component: "pwr",    from_pin: "neg", to_component: "vs_cs",  to_pin: "neg" },
        { from_component: "vs_cs",  from_pin: "neg", to_component: "vs_clk", to_pin: "neg" },
        { from_component: "vs_clk", from_pin: "neg", to_component: "vs_din", to_pin: "neg" },
        { from_component: "vs_din", from_pin: "neg", to_component: "mx",     to_pin: "gnd" },
        { from_component: "vs_cs",  from_pin: "pos", to_component: "mx", to_pin: "cs"  },
        { from_component: "vs_clk", from_pin: "pos", to_component: "mx", to_pin: "clk" },
        { from_component: "vs_din", from_pin: "pos", to_component: "mx", to_pin: "din" },
      ],
    };

    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(0.02);

    const ds = engine.getDisplayState();
    const mx = ds["mx"] as { kind: string; rows: number[]; intensity: number; on: boolean } | undefined;

    expect(mx).toBeDefined();
    expect(mx?.kind).toBe("max7219");
    // Default icState: shutdown=0 (factory reset state) → on=false.
    // No frames sent → all dig registers remain at 0.
    expect(mx?.on).toBe(false);
    // Rows are all 0 (blanked due to shutdown).
    expect(mx?.rows).toEqual([0, 0, 0, 0, 0, 0, 0, 0]);
  });

  it("reports on=false when unpowered (vcc disconnected — open-pin guard)", () => {
    // No VCC wire: the icPowerInfo check fails and the engine takes the early
    // "not powered" branch that emits on:false and skips all decode.
    const circuit: SimCircuit = {
      components: [
        {
          id: "vs_cs",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 0 },
        },
        {
          id: "vs_clk",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 0 },
        },
        {
          id: "vs_din",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 0 },
        },
        {
          id: "mx",
          kind: "max7219",
          pins: [{ id: "din" }, { id: "clk" }, { id: "cs" }, { id: "vcc" }, { id: "gnd" }],
          params: {},
        },
      ],
      wires: [
        // GND connects to a common ground; VCC is intentionally left floating.
        { from_component: "vs_cs",  from_pin: "neg", to_component: "vs_clk", to_pin: "neg" },
        { from_component: "vs_clk", from_pin: "neg", to_component: "vs_din", to_pin: "neg" },
        { from_component: "vs_din", from_pin: "neg", to_component: "mx",     to_pin: "gnd" },
        { from_component: "vs_cs",  from_pin: "pos", to_component: "mx", to_pin: "cs"  },
        { from_component: "vs_clk", from_pin: "pos", to_component: "mx", to_pin: "clk" },
        { from_component: "vs_din", from_pin: "pos", to_component: "mx", to_pin: "din" },
        // mx.vcc is not wired → open pin
      ],
    };

    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(0.02);

    const ds = engine.getDisplayState();
    const mx = ds["mx"] as { kind: string; rows: number[]; intensity: number; on: boolean } | undefined;

    expect(mx).toBeDefined();
    expect(mx?.kind).toBe("max7219");
    // Unpowered (open vcc) → on:false, rows all-zero.
    expect(mx?.kind === "max7219" ? mx.on : undefined).toBe(false);
    expect(mx?.kind === "max7219" ? mx.rows : undefined).toEqual([0, 0, 0, 0, 0, 0, 0, 0]);
  });
});
