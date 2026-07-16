/**
 * Wave 8.0 — sub-step edge capture infrastructure tests.
 *
 * shift_byte.hex: bit-bangs the byte 0xB2 (0b10110010) MSB-first on D8=DATA,
 * D9=CLK during setup(), then halts.  Each bit is latched on a CLK rising edge.
 * The fixture ships pre-compiled (built once with arduino-cli, arduino:avr);
 * see the Wave 8 build log in stage-6-component-libraries.md.
 */

import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { ArduinoMcu } from "../../../src/sim/engine/arduino.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const shiftByteHex = readFileSync(
  join(__dirname, "__fixtures__/shift_byte.hex"),
  "utf-8",
);

// ── Part A: ArduinoMcu sub-step edge capture ──────────────────────────────────

describe("ArduinoMcu — sub-step edge capture", () => {
  it("reconstructs 0xB2 from sub-step CLK edges within a single step", () => {
    const mcu = new ArduinoMcu(shiftByteHex);

    // One 10 ms step = 160 000 cycles at 16 MHz.  The setup() burst is well
    // within that budget; the sketch halts in loop() before the step ends.
    mcu.step(0.01);
    const ev = mcu.getStepPinEvents();

    // Walk events in order, tracking current DATA (d8) and CLK (d9) levels.
    // Seed both low — they start low before setup() drives them.
    let d8Level: 0 | 1 = 0;
    let d9Level: 0 | 1 = 0;
    let result = 0;
    let risingEdges = 0;

    for (const e of ev) {
      if (e.pin === "d8" && e.level !== null) {
        d8Level = e.level as 0 | 1;
      }
      if (e.pin === "d9" && e.level !== null) {
        const newClk = e.level as 0 | 1;
        if (d9Level === 0 && newClk === 1) {
          // CLK rising edge: latch DATA into result MSB-first.
          // 0xB2 = 0b10110010; bit 7 arrives first, bit 0 last.
          result = ((result << 1) | d8Level) & 0xff;
          risingEdges++;
        }
        d9Level = newClk;
      }
    }

    // Derivation: 0xB2 = 0b10110010 — the literal byte the sketch transmits.
    expect(result).toBe(0xb2);
    // Exactly 8 CLK rising edges (one per bit of the byte).
    expect(risingEdges).toBe(8);
    // More than 8 total events — proves sub-step capture (per-step sampling
    // of end-of-step pin levels would only see the final resting state, not
    // the individual transitions, yielding far fewer usable edges).
    expect(ev.length).toBeGreaterThan(8);
  });

  it("clears the event log at the start of each new step", () => {
    const mcu = new ArduinoMcu(shiftByteHex);

    // Step 1: setup() runs and emits the shift burst.
    mcu.step(0.01);
    expect(mcu.getStepPinEvents().length).toBeGreaterThan(0);

    // Step 2: loop() is empty (sketch halted after setup()); no new shifts.
    mcu.step(0.01);
    const ev2 = mcu.getStepPinEvents();

    // No CLK rising edges on d9 in the second step — the burst already ran.
    let d9Prev: 0 | 1 | null = null;
    let risingEdges2 = 0;
    for (const e of ev2) {
      if (e.pin === "d9") {
        const lvl = e.level;
        if (d9Prev === 0 && lvl === 1) risingEdges2++;
        d9Prev = lvl as 0 | 1 | null;
      }
    }
    expect(risingEdges2).toBe(0);
  });
});

// ── Part C smoke: displayState plumbing ────────────────────────────────────────

describe("SimEngine — displayState scaffold", () => {
  it("returns an empty object when no display component is present", () => {
    // Minimal circuit: just a resistor and a voltage source — no Arduino,
    // no display.  Confirms the getter exists and the channel stays empty.
    const circuit: SimCircuit = {
      components: [
        {
          id: "vs1",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        {
          id: "r1",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: 1000 },
        },
      ],
      wires: [
        { from_component: "vs1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "vs1", from_pin: "neg", to_component: "r1", to_pin: "b" },
      ],
    };

    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(1e-3);

    const ds = engine.getDisplayState();
    // Must be a plain object, not null/undefined, and contain no keys this wave.
    expect(ds).toBeInstanceOf(Object);
    expect(ds).toEqual({});
  });
});
