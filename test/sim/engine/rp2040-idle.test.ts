import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { RP2040Mcu } from "../../../src/sim/engine/rp2040.js";

// A Pico sleeping in MicroPython should cost the host almost nothing:
// time.sleep() parks the core in WFE and step() skips the clock to the next
// timer alarm. USBCDC used to answer every read the firmware armed with an
// empty packet, so a USB interrupt woke the core every ~28 us and it executed
// ~113 M of 125 M cycles per simulated second of sleep(0.5); every Pico project
// ran below real time. Counts executed cycles, not wall time, so the gate is
// deterministic.

const UF2 = new Uint8Array(
  readFileSync(fileURLToPath(new URL("../../fixtures/firmware/pico-micropython.uf2", import.meta.url))),
);
const CLOCK_HZ = 125_000_000;
// The PicoEditor "Blink on-board LED (GP25)" example, verbatim.
const BLINK =
  "from machine import Pin\nfrom time import sleep\n\nled = Pin(25, Pin.OUT)\nwhile True:\n    led.toggle()\n    sleep(0.5)\n";

function executedCycles(mcu: RP2040Mcu): number {
  return (mcu as unknown as { rp2040: { core: { cycles: number } } }).rp2040.core.cycles;
}

describe("RP2040Mcu — a sleeping program idles the core (slow)", () => {
  it("runs Blink with the core awake for a sliver of each simulated second", () => {
    const mcu = new RP2040Mcu(UF2);
    mcu.runScript(BLINK);
    for (let i = 0; i < 500 && !mcu.scriptStarted; i++) mcu.step(0.001);
    expect(mcu.scriptStarted).toBe(true);
    for (let i = 0; i < 100; i++) mcu.step(0.001); // let the REPL echo drain

    const edges: number[] = [];
    let level = mcu.pinDriveState("gp25");
    const before = executedCycles(mcu);
    for (let i = 1; i <= 1200; i++) {
      mcu.step(0.001);
      const now = mcu.pinDriveState("gp25");
      if (now !== level) {
        edges.push(i / 1000);
        level = now;
      }
    }
    const busyFraction = (executedCycles(mcu) - before) / CLOCK_HZ / 1.2;

    // Still blinking: a toggle every 0.5 s.
    expect(edges.length).toBeGreaterThanOrEqual(2);
    for (let k = 1; k < edges.length; k++) expect(edges[k] - edges[k - 1]).toBeCloseTo(0.5, 2);
    // Measured under 0.0001 with the fix and 0.905 without it.
    expect(busyFraction).toBeLessThan(0.01);
  }, 30_000);
});
