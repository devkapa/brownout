import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { setMcuFirmware, setRp2040Module } from "../../../src/sim/engine/mcu.js";
import { RP2040Mcu } from "../../../src/sim/engine/rp2040.js";

// End-to-end: a Raspberry Pi Pico running REAL MicroPython drives a header GPIO,
// and the MNA engine lights a wired breadboard LED. Proves the whole S18 Pico
// path — MicroPython -> rp2040js GPIO -> MicrocontrollerCore -> drive stamp ->
// solved LED current.

const UF2 = new Uint8Array(
  readFileSync(fileURLToPath(new URL("../../fixtures/firmware/pico-micropython.uf2", import.meta.url))),
);
// The worker registers the firmware image AND lazily imports the rp2040 module;
// do both for the test (which builds the Pico through the synchronous mcuFactory).
setMcuFirmware("raspberry_pi_pico", UF2);
setRp2040Module(RP2040Mcu);

// GP15 -> LED anode; LED cathode -> 220Ω -> Pico GND. (Vout - Vf) / (Rout+R) ≈
// (3.3 - 1.8) / (50 + 220) ≈ 5.5 mA when GP15 is driven high.
function picoLedCircuit(script: string): SimCircuit {
  return {
    components: [
      {
        id: "pico",
        kind: "raspberry_pi_pico",
        pins: [{ id: "gp15" }, { id: "3v3" }, { id: "gnd" }],
        params: { vcc: 3.3, usb_power: 1, script },
      },
      { id: "led", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { color: "red", vf: 1.8 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
    ],
    wires: [
      { from_component: "pico", from_pin: "gp15", to_component: "led", to_pin: "a" },
      { from_component: "led", from_pin: "k", to_component: "r", to_pin: "a" },
      { from_component: "r", from_pin: "b", to_component: "pico", to_pin: "gnd" },
    ],
  };
}

describe("Raspberry Pi Pico — engine integration (real MicroPython)", () => {
  it("lights the LED when the program drives GP15 high", () => {
    const engine = new SimEngine();
    engine.load(picoLedCircuit("from machine import Pin\r\np = Pin(15, Pin.OUT)\r\np.value(1)\r\n"));
    let ledI = 0;
    for (let i = 0; i < 200; i++) {
      engine.step(0.001);
      ledI = Math.abs(engine.elementI["led"] ?? 0);
      if (ledI > 1e-4) break;
    }
    expect(ledI).toBeGreaterThan(1e-4); // > 0.1 mA — LED conducting
    expect(ledI).toBeLessThan(0.05); // sane bound (~5.5 mA expected)
  }, 30_000);

  it("leaves the LED dark when the program drives GP15 low", () => {
    const engine = new SimEngine();
    engine.load(picoLedCircuit("from machine import Pin\r\np = Pin(15, Pin.OUT)\r\np.value(0)\r\n"));
    // Enough sim-time to boot + feed the program; the pin is never driven high.
    for (let i = 0; i < 60; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-5);
  }, 30_000);
});
