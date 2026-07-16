/**
 * Wave 8.2 — HD44780 16x2 character LCD engine tests.
 *
 * hd44780_hello.hex: an Arduino sketch that initialises an HD44780 LCD in 4-bit
 * mode and prints "Hello, World!" to line 1, then halts in loop().
 *
 * ─── SKETCH INITIALISATION SEQUENCE (4-bit mode) ─────────────────────────────
 *
 * The sketch sends the following HD44780 commands (4-bit mode, RS=0 unless noted):
 *
 *   Power-on delay (~50 ms) before first command.
 *
 *   4-bit entry sequence (three 0x30 nibbles + one 0x20 nibble):
 *     E-strobe, D7-D4=0x3  (0x30 high nibble, forces 8-bit then 4-bit)
 *     E-strobe, D7-D4=0x3
 *     E-strobe, D7-D4=0x3
 *     E-strobe, D7-D4=0x2  (switches to 4-bit mode)
 *
 *   After this, all commands/data are sent as 2 nibbles (high then low):
 *     0x28 → function set: 4-bit, 2-line, 5×8 font
 *     0x0C → display on, cursor off, blink off  (displayOn=true)
 *     0x01 → clear display (DDRAM filled with 0x20, addr=0)
 *     0x06 → entry mode: increment, no shift
 *
 *   Data (RS=1): "Hello, World!" = 13 characters written to DDRAM 0x00-0x0C:
 *     H=0x48, e=0x65, l=0x6C, l=0x6C, o=0x6F, ,=0x2C, (space)=0x20,
 *     W=0x57, o=0x6F, r=0x72, l=0x6C, d=0x64, !=0x21
 *
 * ─── CLOSED-FORM EXPECTED DISPLAY STATE ──────────────────────────────────────
 *
 *   lines[0]:  "Hello, World!   "  (13 chars + 3 trailing spaces = 16 chars total)
 *              DDRAM 0x00..0x0C = "Hello, World!"
 *              DDRAM 0x0D..0x0F remain 0x20 (cleared before data write)
 *   lines[1]:  "                "  (16 spaces — clear filled entire DDRAM)
 *   on:        true
 *   cols:      16
 *   rows:      2
 *
 * ─── SAMPLED FALLBACK TEST ─────────────────────────────────────────────────
 *
 * Drive RS, E, D4-D7 from voltage_source components (4-bit mode).
 * Sequence to write one 4-bit byte (e.g. display-on command 0x0C in 4-bit
 * after function-set already switched to 4-bit):
 *
 *   1. Initial step: E=0, RS=0, data=0 → establishes lastE=0
 *   2. High nibble of 0x0C = 0x0: D7=0,D6=0,D5=0,D4=0; E=1 → lastE=1
 *   3. E falls: latches high nibble; E=0 → latchOnEFalling(phase=0→1,hiNib=0)
 *   4. Low nibble of 0x0C = 0xC: D7=1,D6=1,D5=0,D4=0; E=1 → lastE=1
 *   5. E falls: latches low nibble, forms byte 0x0C; E=0 → displayOn=true
 *
 * For the sampled fallback test we send a short 4-bit sequence:
 *   a. Function-set 0x28 (establishes fourBit=true, twoLine=true)
 *   b. Display-on 0x0C (displayOn=true)
 *   c. Set-DDRAM-addr 0x80 (addr=0x00)
 *   d. Data 'A' = 0x41 (RS=1, write 'A' to DDRAM[0])
 *
 * After this sequence: lines[0][0] === 'A', on === true.
 *
 * NOTE: Each 4-bit byte requires exactly 2 E-strobe pulses. One step per edge.
 */

import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

const hd44780Hex = readFileSync(
  join(__dirname, "__fixtures__/hd44780_hello.hex"),
  "utf-8",
);

// ─── Helper: base LCD component (16 pins, 4-bit wiring) ──────────────────────

/**
 * Build an HD44780 component with all 16 declared pins (d0-d3 will be unwired
 * in 4-bit circuits, but they must still be declared so the engine recognises them).
 */
function lcdPins(): Array<{ id: string }> {
  return [
    { id: "vss" }, { id: "vdd" }, { id: "v0" },
    { id: "rs" }, { id: "rw" }, { id: "e" },
    { id: "d0" }, { id: "d1" }, { id: "d2" }, { id: "d3" },
    { id: "d4" }, { id: "d5" }, { id: "d6" }, { id: "d7" },
    { id: "a" }, { id: "k" },
  ];
}

// ─── Test 1: Event-path integration (real MCU, 4-bit) ────────────────────────

describe("HD44780 — event-path integration (Arduino Uno, 4-bit)", () => {
  it('decodes hd44780_hello.hex: lines[0]==="Hello, World!   ", lines[1]===" "*16, on===true', () => {
    // Wiring: rs←d8, e←d9, d4←d4, d5←d5, d6←d6, d7←d7; vdd←5v, vss←gnd, rw←gnd.
    // d0-d3 are left unwired (4-bit mode). v0 and a/k (backlight) also unwired.
    const circuit: SimCircuit = {
      components: [
        {
          id: "uno",
          kind: "arduino_uno",
          pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
          params: { vcc: 5, hex: hd44780Hex, usb_power: 1 },
        },
        {
          id: "lcd",
          kind: "hd44780",
          pins: lcdPins(),
          params: {},
        },
      ],
      wires: [
        // Power
        { from_component: "uno", from_pin: "5v",  to_component: "lcd", to_pin: "vdd" },
        { from_component: "uno", from_pin: "gnd", to_component: "lcd", to_pin: "vss" },
        // RW tied low (write-only)
        { from_component: "uno", from_pin: "gnd", to_component: "lcd", to_pin: "rw"  },
        // Control lines
        { from_component: "uno", from_pin: "d8",  to_component: "lcd", to_pin: "rs"  },
        { from_component: "uno", from_pin: "d9",  to_component: "lcd", to_pin: "e"   },
        // 4-bit data lines (d4-d7 only)
        { from_component: "uno", from_pin: "d4",  to_component: "lcd", to_pin: "d4"  },
        { from_component: "uno", from_pin: "d5",  to_component: "lcd", to_pin: "d5"  },
        { from_component: "uno", from_pin: "d6",  to_component: "lcd", to_pin: "d6"  },
        { from_component: "uno", from_pin: "d7",  to_component: "lcd", to_pin: "d7"  },
      ],
    };

    const engine = new SimEngine();
    engine.load(circuit);

    // The sketch has a ~50 ms power-on delay before sending any commands.
    // Advance 150 ms of sim time (150 × 1 ms steps) so the init and print
    // sequences complete; the LCD decoder accumulates state across steps via state.lcds.
    for (let i = 0; i < 150; i++) {
      engine.step(0.001);
    }

    const ds = engine.getDisplayState();
    const lcd = ds["lcd"];

    expect(lcd).toBeDefined();
    expect(lcd?.kind).toBe("hd44780");

    // Narrow the union type to the hd44780 variant.
    if (lcd?.kind !== "hd44780") throw new Error("expected hd44780 DisplayInfo");

    // Closed-form derivations from the sketch:
    //   "Hello, World!" = 13 chars at DDRAM 0x00-0x0C
    //   0x01 (clear) before the data write sets all DDRAM to 0x20
    //   line slice [0x00-0x0F] = "Hello, World!" + 3 spaces = 16 chars
    expect(lcd.lines[0]).toBe("Hello, World!   ");

    //   line 2 slice [0x40-0x4F] = 0x20×16 (clear command reset everything)
    expect(lcd.lines[1]).toBe("                ");

    // displayOn=true because 0x0C (D=1) was sent before data writes.
    expect(lcd.on).toBe(true);
  });
});

// ─── Test 2: Sampled fallback (discrete voltage sources, 4-bit sequence) ──────

describe("HD44780 — sampled fallback (discrete voltage sources, 4-bit)", () => {
  it("writes 'A' to lines[0][0] and sets on=true via 4-bit strobe sequence", () => {
    // Circuit: voltage sources drive RS, E, D4-D7; 5 V supply for VDD/VSS.
    // All voltage values are mutated between steps to simulate pin transitions.
    const circuit: SimCircuit = {
      components: [
        // 5 V supply
        {
          id: "pwr",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        // RS driver
        { id: "vs_rs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        // E driver
        { id: "vs_e",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        // D4-D7 drivers
        { id: "vs_d4", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "vs_d5", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "vs_d6", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "vs_d7", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        // HD44780 LCD
        { id: "lcd", kind: "hd44780", pins: lcdPins(), params: {} },
      ],
      wires: [
        // Common GND chain
        { from_component: "pwr",   from_pin: "neg", to_component: "vs_rs", to_pin: "neg" },
        { from_component: "vs_rs", from_pin: "neg", to_component: "vs_e",  to_pin: "neg" },
        { from_component: "vs_e",  from_pin: "neg", to_component: "vs_d4", to_pin: "neg" },
        { from_component: "vs_d4", from_pin: "neg", to_component: "vs_d5", to_pin: "neg" },
        { from_component: "vs_d5", from_pin: "neg", to_component: "vs_d6", to_pin: "neg" },
        { from_component: "vs_d6", from_pin: "neg", to_component: "vs_d7", to_pin: "neg" },
        { from_component: "vs_d7", from_pin: "neg", to_component: "lcd",   to_pin: "vss" },
        // VDD / VSS supply
        { from_component: "pwr",   from_pin: "pos", to_component: "lcd",   to_pin: "vdd" },
        // Control signals
        { from_component: "vs_rs", from_pin: "pos", to_component: "lcd", to_pin: "rs" },
        { from_component: "vs_e",  from_pin: "pos", to_component: "lcd", to_pin: "e"  },
        // Data lines D4-D7
        { from_component: "vs_d4", from_pin: "pos", to_component: "lcd", to_pin: "d4" },
        { from_component: "vs_d5", from_pin: "pos", to_component: "lcd", to_pin: "d5" },
        { from_component: "vs_d6", from_pin: "pos", to_component: "lcd", to_pin: "d6" },
        { from_component: "vs_d7", from_pin: "pos", to_component: "lcd", to_pin: "d7" },
      ],
    };

    const engine = new SimEngine();
    engine.load(circuit);

    // Step 0: establish idle state (E=0, RS=0, data=0, powered).
    // This commits lastE=0 so the first E-rising step is detectable.
    engine.step(1e-4);

    /**
     * Set all pin drivers, reload (to propagate param changes), then step once.
     * The sampled fallback compares prev-committed levels to current levels;
     * one step = one potential edge per pin.
     *
     * d7..d4 correspond to D7..D4 on the LCD (the high-nibble/data bus in 4-bit mode).
     */
    function drive(rs: number, e: number, d7: number, d6: number, d5: number, d4: number): void {
      const c = circuit.components;
      c.find((x) => x.id === "vs_rs")!.params.voltage = rs ? 5 : 0;
      c.find((x) => x.id === "vs_e")!.params.voltage  = e  ? 5 : 0;
      c.find((x) => x.id === "vs_d7")!.params.voltage = d7 ? 5 : 0;
      c.find((x) => x.id === "vs_d6")!.params.voltage = d6 ? 5 : 0;
      c.find((x) => x.id === "vs_d5")!.params.voltage = d5 ? 5 : 0;
      c.find((x) => x.id === "vs_d4")!.params.voltage = d4 ? 5 : 0;
      engine.load(circuit);
      engine.step(1e-4);
    }

    /**
     * Send one 4-bit byte (two E pulses: high nibble first, then low nibble).
     * RS selects command (0) or data (1). The sampled path detects E 1→0 edges.
     *
     * For each nibble: raise E with nibble on D7-D4, then lower E to latch.
     */
    function send4bit(rs: number, byte: number): void {
      const hiNib = (byte >> 4) & 0xf;
      const loNib = byte & 0xf;
      // High nibble: E=1
      drive(rs, 1, (hiNib >> 3) & 1, (hiNib >> 2) & 1, (hiNib >> 1) & 1, hiNib & 1);
      // E falls → latch high nibble
      drive(rs, 0, (hiNib >> 3) & 1, (hiNib >> 2) & 1, (hiNib >> 1) & 1, hiNib & 1);
      // Low nibble: E=1
      drive(rs, 1, (loNib >> 3) & 1, (loNib >> 2) & 1, (loNib >> 1) & 1, loNib & 1);
      // E falls → latch low nibble; full byte processed
      drive(rs, 0, (loNib >> 3) & 1, (loNib >> 2) & 1, (loNib >> 1) & 1, loNib & 1);
    }

    // ── 4-bit initialisation: three 0x30 nibbles (8-bit mode pulses) ──────────
    // The HD44780 requires 3 specific 0x3 nibble pulses to reliably enter known state.
    // In our engine, since fourBit starts false (8-bit), the first E-fall with D7-D4=0x3
    // is processed as byte 0x30 (function set, 8-bit), which leaves the controller in
    // 8-bit mode. We drive 3 of these then switch.
    drive(0, 1, 0, 0, 1, 1);  // 0x3 on D7-D4, E=1
    drive(0, 0, 0, 0, 1, 1);  // E falls → byte 0x30 processed (function set, 8-bit no-op)
    drive(0, 1, 0, 0, 1, 1);
    drive(0, 0, 0, 0, 1, 1);  // second 0x30
    drive(0, 1, 0, 0, 1, 1);
    drive(0, 0, 0, 0, 1, 1);  // third 0x30

    // ── Switch to 4-bit: send 0x20 high nibble only (sets fourBit=true) ───────
    // 0x20 = function set with DL=0 (4-bit). As a single nibble E-pulse in 8-bit mode
    // it forms byte 0x20 (DL=0 → fourBit=true).
    drive(0, 1, 0, 0, 1, 0);  // 0x2 on D7-D4
    drive(0, 0, 0, 0, 1, 0);  // E falls → 0x20 processed → fourBit=true

    // ── 4-bit commands ────────────────────────────────────────────────────────
    // 0x28: function set — 4-bit, 2-line, 5×8 font
    send4bit(0, 0x28);

    // 0x0C: display on, cursor off, blink off → displayOn = true
    send4bit(0, 0x0c);

    // 0x01: clear display → DDRAM filled with 0x20, addr=0
    send4bit(0, 0x01);

    // 0x06: entry mode set — increment, no shift
    send4bit(0, 0x06);

    // 0x80: set DDRAM address 0x00 (RS=0, addr=0)
    send4bit(0, 0x80);

    // Write 'A' = 0x41 to DDRAM[0] (RS=1)
    send4bit(1, 0x41);

    const ds = engine.getDisplayState();
    const lcd = ds["lcd"];

    expect(lcd).toBeDefined();
    expect(lcd?.kind).toBe("hd44780");

    if (lcd?.kind !== "hd44780") throw new Error("expected hd44780 DisplayInfo");

    // Closed-form derivations:
    //   After send4bit(1, 0x41): DDRAM[0x00] = 0x41 = 'A'.
    //   0x01 clear was sent before the data write, so DDRAM[0x01-0x4F] = 0x20.
    //   lines[0] = slice [0x00..0x0F] → "A" + 15 spaces.
    //   on = true (0x0C D bit was set).
    expect(lcd.lines[0][0]).toBe("A");
    expect(lcd.on).toBe(true);
  });
});

// ─── Test 3: Display-off / power ─────────────────────────────────────────────

describe("HD44780 — display-off and power behaviour", () => {
  it("reports on=false when powered but display-off command (0x08, D=0) received", () => {
    // Drive a simple powered 4-bit LCD through: function set → display OFF (0x08).
    // displayOn should be false; DDRAM is not checked (no data writes).
    const circuit: SimCircuit = {
      components: [
        { id: "pwr", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vs_rs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "vs_e",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "vs_d4", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "vs_d5", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "vs_d6", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "vs_d7", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "lcd", kind: "hd44780", pins: lcdPins(), params: {} },
      ],
      wires: [
        { from_component: "pwr",   from_pin: "neg", to_component: "vs_rs", to_pin: "neg" },
        { from_component: "vs_rs", from_pin: "neg", to_component: "vs_e",  to_pin: "neg" },
        { from_component: "vs_e",  from_pin: "neg", to_component: "vs_d4", to_pin: "neg" },
        { from_component: "vs_d4", from_pin: "neg", to_component: "vs_d5", to_pin: "neg" },
        { from_component: "vs_d5", from_pin: "neg", to_component: "vs_d6", to_pin: "neg" },
        { from_component: "vs_d6", from_pin: "neg", to_component: "vs_d7", to_pin: "neg" },
        { from_component: "vs_d7", from_pin: "neg", to_component: "lcd",   to_pin: "vss" },
        { from_component: "pwr",   from_pin: "pos", to_component: "lcd",   to_pin: "vdd" },
        { from_component: "vs_rs", from_pin: "pos", to_component: "lcd", to_pin: "rs" },
        { from_component: "vs_e",  from_pin: "pos", to_component: "lcd", to_pin: "e"  },
        { from_component: "vs_d4", from_pin: "pos", to_component: "lcd", to_pin: "d4" },
        { from_component: "vs_d5", from_pin: "pos", to_component: "lcd", to_pin: "d5" },
        { from_component: "vs_d6", from_pin: "pos", to_component: "lcd", to_pin: "d6" },
        { from_component: "vs_d7", from_pin: "pos", to_component: "lcd", to_pin: "d7" },
      ],
    };

    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(1e-4);

    function drive(rs: number, e: number, d7: number, d6: number, d5: number, d4: number): void {
      const c = circuit.components;
      c.find((x) => x.id === "vs_rs")!.params.voltage = rs ? 5 : 0;
      c.find((x) => x.id === "vs_e")!.params.voltage  = e  ? 5 : 0;
      c.find((x) => x.id === "vs_d7")!.params.voltage = d7 ? 5 : 0;
      c.find((x) => x.id === "vs_d6")!.params.voltage = d6 ? 5 : 0;
      c.find((x) => x.id === "vs_d5")!.params.voltage = d5 ? 5 : 0;
      c.find((x) => x.id === "vs_d4")!.params.voltage = d4 ? 5 : 0;
      engine.load(circuit);
      engine.step(1e-4);
    }

    function send4bit(rs: number, byte: number): void {
      const hiNib = (byte >> 4) & 0xf;
      const loNib = byte & 0xf;
      drive(rs, 1, (hiNib >> 3) & 1, (hiNib >> 2) & 1, (hiNib >> 1) & 1, hiNib & 1);
      drive(rs, 0, (hiNib >> 3) & 1, (hiNib >> 2) & 1, (hiNib >> 1) & 1, hiNib & 1);
      drive(rs, 1, (loNib >> 3) & 1, (loNib >> 2) & 1, (loNib >> 1) & 1, loNib & 1);
      drive(rs, 0, (loNib >> 3) & 1, (loNib >> 2) & 1, (loNib >> 1) & 1, loNib & 1);
    }

    // Switch to 4-bit via single 0x20 high-nibble pulse (in 8-bit mode → byte 0x20).
    drive(0, 1, 0, 0, 1, 0);
    drive(0, 0, 0, 0, 1, 0);

    // Display OFF command: 0x08 (D=0, C=0, B=0 → displayOn=false).
    // Note: this is sent BEFORE any display-on, so it confirms default stays false too.
    send4bit(0, 0x08);

    const ds = engine.getDisplayState();
    const lcd = ds["lcd"];

    expect(lcd?.kind).toBe("hd44780");
    if (lcd?.kind !== "hd44780") throw new Error("expected hd44780 DisplayInfo");

    // Closed-form: 0x08 sets D=0 → displayOn=false → on=false.
    expect(lcd.on).toBe(false);
    // Lines are blank spaces (DDRAM default 0x20 = space, not cleared by 0x08).
    expect(lcd.lines[0]).toBe("                ");
  });

  it("reports on=false and blank lines when VDD is open (unpowered)", () => {
    // VDD is intentionally left floating (open pin); icPowerInfo.powered = false.
    const circuit: SimCircuit = {
      components: [
        // GND reference only; VDD left unconnected.
        { id: "pwr_gnd", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        { id: "lcd", kind: "hd44780", pins: lcdPins(), params: {} },
      ],
      wires: [
        // VSS connected to GND reference; VDD pin deliberately unwired.
        { from_component: "pwr_gnd", from_pin: "pos", to_component: "lcd", to_pin: "vss" },
      ],
    };

    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(0.02);

    const ds = engine.getDisplayState();
    const lcd = ds["lcd"];

    expect(lcd).toBeDefined();
    expect(lcd?.kind).toBe("hd44780");

    if (lcd?.kind !== "hd44780") throw new Error("expected hd44780 DisplayInfo");

    // Unpowered (open VDD) → on:false, lines all spaces (16 × " ").
    // Closed-form: icPowerInfo.powered=false → early-exit branch → blank display.
    expect(lcd.on).toBe(false);
    expect(lcd.lines[0]).toBe("                ");
    expect(lcd.lines[1]).toBe("                ");
  });
});
