/**
 * The UPduino v3.1 in a circuit: its configured iCE40 drives a breadboard LED through a header pin, reads a pin a
 * wire grounds through its SB_IO pull-up, runs from its USB supply only, and reconfigures when its design changes.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { checkFpgaProgram, UpduinoMcu } from "../../../src/sim/engine/fpga-mcu.js";
import { mcuFactory } from "../../../src/sim/engine/mcu.js";

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
    // Configured with gpio_2 low, then wired high: that rising edge starts the oscillation.
    engine.load({ ...c, wires: [] });
    for (let i = 0; i < 2; i++) engine.step(0.001);
    expect(engine.getArduinoState().fpga?.error).toBeUndefined();
    engine.load(c);
    expect(() => { for (let i = 0; i < 3; i++) engine.step(0.001); }).not.toThrow();
    expect(engine.getArduinoState().fpga?.error).toMatch(/does not settle/);
  });

  it("sees no edge on an input already high when it is configured", () => {
    const engine = new SimEngine();
    // gpio_3 is set by a rising edge on gpio_2; gpio_2 is wired to 3V3 from the start.
    engine.load({
      components: [
        { id: "fpga", kind: "upduino", pins: [{ id: "gpio_2" }, { id: "gpio_3" }, { id: "3v3" }, { id: "gnd" }], params: { vcc: 3.3, usb_power: 1, netlist: netlist("edge") } },
        { id: "led", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { color: "red", vf: 1.8 } },
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
      ],
      wires: [
        { from_component: "fpga", from_pin: "gpio_2", to_component: "fpga", to_pin: "3v3" },
        { from_component: "fpga", from_pin: "gpio_3", to_component: "led", to_pin: "a" },
        { from_component: "led", from_pin: "k", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "fpga", to_pin: "gnd" },
      ],
    });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-6);
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

  it("sinks an external LED on an RGB pad at the driver's set current, or less when the load limits it", () => {
    // 3V3 -> LED -> (resistor) -> LED_RED; the design sinks red at 8 mA while gpio_2 (wired to 3V3) is high.
    const run = (ohms: number) => {
      const engine = new SimEngine();
      engine.load({
        components: [
          { id: "fpga", kind: "upduino", pins: [{ id: "gpio_2" }, { id: "led_red" }, { id: "3v3" }, { id: "gnd" }], params: { vcc: 3.3, usb_power: 1, netlist: netlist("rgb_sink") } },
          { id: "led", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { color: "red", vf: 1.8 } },
          { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: ohms } },
        ],
        wires: [
          { from_component: "fpga", from_pin: "gpio_2", to_component: "fpga", to_pin: "3v3" },
          { from_component: "fpga", from_pin: "3v3", to_component: "led", to_pin: "a" },
          { from_component: "led", from_pin: "k", to_component: "r", to_pin: "a" },
          { from_component: "r", from_pin: "b", to_component: "fpga", to_pin: "led_red" },
        ],
      });
      for (let i = 0; i < 5; i++) engine.step(0.001);
      return Math.abs(engine.elementI["led"] ?? 0);
    };
    // With 10 ohms the pad has headroom: the driver holds its 8 mA.
    expect(run(10)).toBeCloseTo(8e-3, 4);
    // 1 kOhm leaves the pad below the driver's 0.5 V headroom: the load sets the current, under 8 mA.
    const limited = run(1000);
    expect(limited).toBeGreaterThan(1e-3);
    expect(limited).toBeLessThan(1.6e-3);
  });

  it("drives the board's 12 MHz onto gpio_20 while its OSC jumper is fitted, every edge in each step", () => {
    const board = (osc: number) => new UpduinoMcu({ netlist: netlist("button"), oscillator: osc !== 0 });
    const b = board(1);
    b.step(1e-3);
    const events = b.getStepPinEvents().filter((e) => e.pin === "gpio_20");
    // 12 MHz for 1 ms: 24,000 edges, alternating, the first rising.
    expect(events.length).toBe(24000);
    expect(events.slice(0, 4).map((e) => e.level)).toEqual([1, 0, 1, 0]);
    expect(events.every((e, i) => i === 0 || e.cycle > events[i - 1]!.cycle)).toBe(true);
    expect(b.pinDriveState("gpio_20")).toBe(events[events.length - 1]!.level ? "out-high" : "out-low");
    // Without the jumper the pin is the design's alone.
    const n = board(0);
    n.step(1e-3);
    expect(n.getStepPinEvents().filter((e) => e.pin === "gpio_20")).toEqual([]);
    expect(n.pinDriveState("gpio_20")).toBe("input");
    // With no design on the board the oscillator still runs.
    const blank = mcuFactory("upduino", { netlist: "" })!;
    blank.step(1e-6);
    expect(blank.getStepPinEvents().length).toBe(24);
  });

  it("clocks a counter wired to gpio_20 with the board's 12 MHz", () => {
    const engine = new SimEngine();
    const CD4017 = ["q5", "q1", "q0", "q2", "q6", "q7", "q3", "gnd", "q8", "q4", "q9", "co", "clkinh", "clk", "reset", "vcc"];
    engine.load({
      components: [
        { id: "fpga", kind: "upduino", pins: [{ id: "gpio_20" }, { id: "3v3" }, { id: "gnd" }], params: { vcc: 3.3, usb_power: 1, netlist: netlist("button") } },
        { id: "u", kind: "cd4017", pins: CD4017.map((id) => ({ id })), params: {} },
      ],
      wires: [
        { from_component: "fpga", from_pin: "3v3", to_component: "u", to_pin: "vcc" },
        { from_component: "fpga", from_pin: "gnd", to_component: "u", to_pin: "gnd" },
        { from_component: "fpga", from_pin: "gnd", to_component: "u", to_pin: "clkinh" },
        { from_component: "fpga", from_pin: "gnd", to_component: "u", to_pin: "reset" },
        { from_component: "fpga", from_pin: "gpio_20", to_component: "u", to_pin: "clk" },
      ],
    });
    const counts: number[] = [];
    for (let i = 0; i < 4; i++) {
      engine.step(1.004e-4);
      counts.push(engine.getIcState("u")?.count ?? -1);
    }
    // 1.004e-4 s of 12 MHz is 1204.8 rising edges: the decade counter moves by 4 or 5 each step, never standing
    // still as a level sampled once a step would leave it.
    const moved = counts.slice(1).map((c, i) => (c - counts[i]! + 10) % 10);
    expect(moved.every((m) => m === 4 || m === 5)).toBe(true);
  });

  it("pulls a pin up through the strength its PCF selects", () => {
    // gpio_2 (the button design's pulled-up input) to GND through 3.3 kOhm: with the 3P3K pull-up, half of 3V3.
    const run = (pcf: string) => {
      const engine = new SimEngine();
      engine.load({
        components: [
          { id: "fpga", kind: "upduino", pins: [{ id: "gpio_2" }, { id: "3v3" }, { id: "gnd" }], params: { vcc: 3.3, usb_power: 1, netlist: netlist("button"), pcf } },
          { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 3300 } },
        ],
        wires: [
          { from_component: "fpga", from_pin: "gpio_2", to_component: "r", to_pin: "a" },
          { from_component: "r", from_pin: "b", to_component: "fpga", to_pin: "gnd" },
        ],
      });
      for (let i = 0; i < 3; i++) engine.step(0.001);
      return Math.abs(engine.elementI["r"] ?? 0);
    };
    expect(run("set_io -pullup yes -pullup_resistor 3P3K gpio_2 2\n")).toBeCloseTo(3.3 / 6600, 5);
    expect(run("")).toBeCloseTo(3.3 / (3300 + 88_000), 6);
  });

  it("runs a fast design at most a budget of edges per step, and says it fell behind", () => {
    const b = new UpduinoMcu({ netlist: netlist("blinky") });
    b.setInputBit("gpio_2", 0);
    b.step(1e-3);
    expect(b.slowed).toBe(false);
    // 0.5 s of the 12 MHz counter is 12 million edges, beyond one step's budget.
    b.step(0.5);
    expect(b.slowed).toBe(true);
    // The board's state says so while it lasts.
    const engine = new SimEngine();
    engine.load(circuit({ grounded: true, design: "blinky" }));
    engine.step(0.001);
    expect(engine.getArduinoState().fpga?.slowed).toBeUndefined();
    engine.step(0.5);
    expect(engine.getArduinoState().fpga?.slowed).toBe(true);
  });

  it("lets a fault that is not the design's surface instead of leaving the board blank", () => {
    expect(() => mcuFactory("upduino", { netlist: netlist("button"), pcf: 42 as unknown as string })).toThrow(TypeError);
    expect(mcuFactory("upduino", { netlist: "{" })).not.toBeNull();
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
