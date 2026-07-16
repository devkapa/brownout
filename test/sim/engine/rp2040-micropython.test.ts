import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { RP2040Mcu } from "../../../src/sim/engine/rp2040.js";

// Boots the REAL Raspberry Pi Pico MicroPython firmware in rp2040js and runs a
// student-style program over the REPL, proving that de:volt's Pico core executes
// MicroPython and that program-driven GPIO reaches the MNA-facing pinDriveState.
// Heavier than the pure-coupling test (interprets ~2M+ ARM instructions), so it
// lives in its own file. Budget is tight on purpose: a working boot+feed reaches
// the pin within ~10-30 steps, and rp2040js allocates per instruction, so an
// open-ended run would exhaust the Node heap.

const UF2 = new Uint8Array(
  readFileSync(fileURLToPath(new URL("../../fixtures/firmware/pico-micropython.uf2", import.meta.url))),
);

/** Step the core until `pin` reaches `want`, or fail after a small step budget. */
function runUntil(mcu: RP2040Mcu, pin: string, want: "out-high" | "out-low", maxSteps = 250): boolean {
  for (let i = 0; i < maxSteps; i++) {
    mcu.step(0.002); // 0.002 s ≈ 250k instructions/step
    if (mcu.pinDriveState(pin) === want) return true;
  }
  return false;
}

describe("RP2040Mcu — runs real MicroPython (slow)", () => {
  it("boots MicroPython and drives the on-board LED (GP25) high", () => {
    const mcu = new RP2040Mcu(UF2);
    mcu.runScript("from machine import Pin\r\np = Pin(25, Pin.OUT)\r\np.value(1)\r\n");
    expect(runUntil(mcu, "gp25", "out-high")).toBe(true);
    expect(mcu.scriptStarted).toBe(true);
  }, 30_000);

  it("drives a header GPIO (GP15) as an output-low", () => {
    const mcu = new RP2040Mcu(UF2);
    // RP2040 reset defaults enable the weak pull-down; the program drives low.
    expect(mcu.pinDriveState("gp15")).toBe("input-pulldown");
    mcu.runScript("from machine import Pin\r\np = Pin(15, Pin.OUT)\r\np.value(0)\r\n");
    expect(runUntil(mcu, "gp15", "out-low")).toBe(true);
  }, 30_000);

  it("re-boots and re-feeds the program after reset()", () => {
    const mcu = new RP2040Mcu(UF2);
    mcu.runScript("from machine import Pin\r\np = Pin(16, Pin.OUT)\r\np.value(1)\r\n");
    expect(runUntil(mcu, "gp16", "out-high")).toBe(true);
    expect(mcu.scriptStarted).toBe(true);
    mcu.reset();
    expect(mcu.scriptStarted).toBe(false); // feed state machine re-armed
    // Re-boots MicroPython + re-submits the program to the REPL from scratch.
    expect(runUntil(mcu, "gp16", "out-high")).toBe(true);
    expect(mcu.scriptStarted).toBe(true);
  }, 40_000);
});
