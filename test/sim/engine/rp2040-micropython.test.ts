import { describe, expect, it, vi } from "vitest";
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

  it("boots and runs a program without writing to the console", () => {
    // rp2040js's default Debug-level logger printed every USB transfer, SEV and
    // unimplemented peripheral access; in the app that was 2,000+ messages in
    // the first seconds and the engine fell behind real time.
    const spies = (["log", "debug", "info", "warn"] as const).map((m) =>
      vi.spyOn(console, m).mockImplementation(() => {}),
    );
    try {
      const mcu = new RP2040Mcu(UF2);
      mcu.runScript("from machine import Pin\r\np = Pin(25, Pin.OUT)\r\np.value(1)\r\n");
      expect(runUntil(mcu, "gp25", "out-high")).toBe(true);
      for (const spy of spies) expect(spy).not.toHaveBeenCalled();
    } finally {
      for (const spy of spies) spy.mockRestore();
    }
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

  // rp2040js reads adc.channelValues as raw 12-bit counts. Volts written there
  // made 3.3 V read as 3 counts, so read_u16() returned 48 and the pot example
  // never lit its LED. MicroPython scales 12 bits to 16 as raw << 4 | raw >> 8.
  it.each([
    [0, 0],
    [1.65, 32776], // 2048 counts: 2048 << 4 | 2048 >> 8
    [3.3, 65535],
  ])("reads %s V on GP26 as read_u16() = %s", (volts, expected) => {
    const mcu = new RP2040Mcu(UF2);
    mcu.setAnalogVolts("gp26", volts, 3.3);
    mcu.runScript("from machine import ADC, Pin\r\nprint('ADC=%d' % ADC(Pin(26)).read_u16())\r\n");
    const serial = () => (mcu as unknown as { _serialOut: string })._serialOut;
    for (let i = 0; i < 250 && !/ADC=\d+/.test(serial()); i++) mcu.step(0.002);
    expect(Number(/ADC=(\d+)/.exec(serial())?.[1])).toBe(expected);
  }, 30_000);

  // ADC4 is the on-die temperature sensor. Nothing fed it, so it read 0 counts,
  // which the datasheet formula below turns into 437 °C.
  it("reads the temperature sensor (ADC4) as about 27 °C", () => {
    const mcu = new RP2040Mcu(UF2);
    mcu.runScript(
      "from machine import ADC\r\nr = ADC(4).read_u16()\r\n" +
        "print('T=%.2f' % (27 - (r * 3.3 / 65535 - 0.706) / 0.001721))\r\n",
    );
    const serial = () => (mcu as unknown as { _serialOut: string })._serialOut;
    const done = /T=(-?[\d.]+)\r?\n/;
    for (let i = 0; i < 250 && !done.test(serial()); i++) mcu.step(0.002);
    const celsius = Number(done.exec(serial())?.[1]);
    expect(Math.abs(celsius - 27)).toBeLessThanOrEqual(1);
  }, 30_000);
});
