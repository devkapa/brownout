import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { RP2040Mcu } from "../../../src/sim/engine/rp2040.js";

// A program longer than rp2040js's 512-byte USBCDC tx FIFO must still reach the
// REPL whole. The feed used to push the entire program plus Ctrl-D into that
// FIFO at once; FIFO.push drops bytes when full, so the tail and the Ctrl-D that
// runs it were lost and a student's longer program silently never started.

const UF2 = new Uint8Array(
  readFileSync(fileURLToPath(new URL("../../fixtures/firmware/pico-micropython.uf2", import.meta.url))),
);

/**
 * A program of at least `minBytes` that lights GP25 only if every line
 * arrived: each `n += 1` counts itself, so a dropped or repeated chunk
 * anywhere (not just a lost tail) leaves the LED off or fails to parse.
 */
function countingProgram(minBytes: number): { source: string; lines: number } {
  const head = "from machine import Pin\nn = 0\n";
  const line = "n += 1  # counted line, padding the program past the USB FIFO\n";
  const lines = Math.ceil((minBytes - head.length) / line.length) + 1;
  const tail = `if n == ${lines}:\n    Pin(25, Pin.OUT).value(1)\n`;
  return { source: head + line.repeat(lines) + tail, lines };
}

describe("RP2040Mcu — feeds programs longer than the USB FIFO (slow)", () => {
  it.each([600, 4096])("runs a program of more than %i bytes", (minBytes) => {
    const { source } = countingProgram(minBytes);
    expect(source.length).toBeGreaterThan(minBytes);
    const mcu = new RP2040Mcu(UF2);
    mcu.runScript(source);
    let lit = false;
    for (let i = 0; i < 1500 && !lit; i++) {
      mcu.step(0.002);
      lit = mcu.pinDriveState("gp25") === "out-high";
    }
    expect(mcu.scriptStarted).toBe(true);
    expect(lit).toBe(true);
  }, 30_000);
});
