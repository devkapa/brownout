/**
 * The UPduino v3.1 in a circuit: its configured iCE40 drives a breadboard LED through a header pin, reads a pin a
 * wire grounds through its SB_IO pull-up, runs from its USB supply only, and reconfigures when its design changes.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { checkFpgaProgram } from "../../../src/sim/engine/fpga-mcu.js";

const netlist = (name: string) => readFileSync(join(__dirname, "__fixtures__", "fpga", `${name}.json`), "utf-8");

// The button design: gpio_4 = NOT gpio_2 (gpio_2 pulled up). gpio_4 -> LED anode; cathode -> 220 ohm -> GND.
function circuit({ grounded = false, usb = 1, design = "button" } = {}): SimCircuit {
  return {
    components: [
      {
        id: "fpga",
        kind: "upduino",
        pins: [{ id: "gpio_2" }, { id: "gpio_4" }, { id: "3v3" }, { id: "gnd" }, { id: "gnd2" }],
        params: { vcc: 3.3, usb_power: usb, netlist: netlist(design) },
      },
      { id: "led", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { color: "red", vf: 1.8 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
    ],
    wires: [
      { from_component: "fpga", from_pin: "gpio_4", to_component: "led", to_pin: "a" },
      { from_component: "led", from_pin: "k", to_component: "r", to_pin: "a" },
      { from_component: "r", from_pin: "b", to_component: "fpga", to_pin: "gnd" },
      ...(grounded ? [{ from_component: "fpga", from_pin: "gpio_2", to_component: "fpga", to_pin: "gnd2" }] : []),
    ],
  };
}

function ledCurrent(c: SimCircuit, steps = 5): number {
  const engine = new SimEngine();
  engine.load(c);
  for (let i = 0; i < steps; i++) engine.step(0.001);
  return Math.abs(engine.elementI["led"] ?? 0);
}

describe("UPduino v3.1 in a circuit", () => {
  it("leaves the LED dark while gpio_2 floats on its pull-up, and lights it once a wire grounds gpio_2", () => {
    expect(ledCurrent(circuit())).toBeLessThan(1e-6);
    const lit = ledCurrent(circuit({ grounded: true }));
    // (3.3 V - 1.8 V) / (220 ohm + the pin's output resistance): a few mA.
    expect(lit).toBeGreaterThan(3e-3);
    expect(lit).toBeLessThan(8e-3);
  });

  it("runs only from its USB supply", () => {
    expect(ledCurrent(circuit({ grounded: true, usb: 0 }))).toBeLessThan(1e-6);
  });

  it("reconfigures when its design changes, and reports the on-board RGB LED its design drives", () => {
    const engine = new SimEngine();
    engine.load(circuit({ grounded: true }));
    // The first step reads gpio_2's level for the next: the LED lights from the second.
    for (let i = 0; i < 3; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeGreaterThan(3e-3);
    // The blinky design drives no gpio_4 and its counter starts at zero: the LED goes dark and the RGB LED is off.
    engine.load(circuit({ grounded: true, design: "blinky" }));
    engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-6);
    expect(engine.getArduinoState().fpga?.rgb).toEqual([false, false, false]);
  });

  it("stops a design that starts to oscillate, and says why, instead of failing the step", () => {
    const engine = new SimEngine();
    const c: SimCircuit = {
      components: [{ id: "fpga", kind: "upduino", pins: [{ id: "gpio_2" }, { id: "gpio_3" }, { id: "3v3" }, { id: "gnd" }], params: { vcc: 3.3, usb_power: 1, netlist: netlist("ring") } }],
      wires: [{ from_component: "fpga", from_pin: "gpio_2", to_component: "fpga", to_pin: "3v3" }],
    };
    engine.load(c);
    expect(() => { for (let i = 0; i < 3; i++) engine.step(0.001); }).not.toThrow();
    expect(engine.getArduinoState().fpga?.error).toMatch(/does not settle/);
  });

  it("reconfigures when USB power returns, as an SRAM FPGA does", () => {
    const engine = new SimEngine();
    // The divider's register counts the 12 MHz clock; after power returns it starts again from zero.
    const c = (usb: number): SimCircuit => ({
      components: [{ id: "fpga", kind: "upduino", pins: [{ id: "gpio_2" }, { id: "gpio_20" }, { id: "3v3" }, { id: "gnd" }], params: { vcc: 3.3, usb_power: usb, netlist: netlist("divider") } }],
      wires: [{ from_component: "fpga", from_pin: "gpio_2", to_component: "fpga", to_pin: "gnd" }],
    });
    engine.load(c(1));
    engine.step(0.001);
    const boards = () => (engine as unknown as { state: { arduinos: Map<string, unknown> } }).state.arduinos;
    const before = boards().get("fpga");
    engine.load(c(1));
    expect(boards().get("fpga")).toBe(before);
    engine.load(c(0));
    engine.load(c(1));
    expect(boards().get("fpga")).not.toBe(before);
  });

  it("says why a design cannot run on the board", () => {
    expect(checkFpgaProgram({ netlist: netlist("button") })).toBeNull();
    const bad = JSON.stringify({ modules: { top: { ports: { gpio_5: { direction: "output", bits: [2] } }, cells: {} } } });
    expect(checkFpgaProgram({ netlist: bad })).toMatch(/pin 5/);
    expect(checkFpgaProgram({ netlist: "{" })).toMatch(/JSON/);
    expect(checkFpgaProgram({ netlist: "{}" })).toMatch(/not a Yosys netlist/);
    expect(checkFpgaProgram({ netlist: "null" })).toMatch(/not a Yosys netlist/);
  });
});
