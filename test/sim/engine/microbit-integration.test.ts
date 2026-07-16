import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

// The micro:bit runs in the client-side embedded MicroPython sim (not the MNA
// worker); its program-driven edge pins arrive via setMicrobitDrive. This tests
// the engine side of the Phase-2 breadboard bridge: a driven P0 stamps a source
// that lights a wired LED, and P0 low leaves it dark.

// P0 -> LED anode; LED cathode -> 220Ω -> micro:bit GND.
function microbitLedCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "mb",
        kind: "microbit",
        pins: [{ id: "p0" }, { id: "p1" }, { id: "p2" }, { id: "3v" }, { id: "gnd" }],
        params: { script: "" },
      },
      { id: "led", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { color: "red", vf: 1.8 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
    ],
    wires: [
      { from_component: "mb", from_pin: "p0", to_component: "led", to_pin: "a" },
      { from_component: "led", from_pin: "k", to_component: "r", to_pin: "a" },
      { from_component: "r", from_pin: "b", to_component: "mb", to_pin: "gnd" },
    ],
  };
}

describe("micro:bit — engine breadboard bridge (P0 drive)", () => {
  it("lights an LED when the program drives P0 high", () => {
    const engine = new SimEngine();
    engine.load(microbitLedCircuit());
    engine.setMicrobitDrive("mb", { p0: { mode: "digital", value: 1 } });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    // (3.3 - 1.8) / (50 + 220) ≈ 5.5 mA
    const i = Math.abs(engine.elementI["led"] ?? 0);
    expect(i).toBeGreaterThan(1e-4);
    expect(i).toBeLessThan(0.05);
  });

  it("leaves the LED dark when P0 is driven low", () => {
    const engine = new SimEngine();
    engine.load(microbitLedCircuit());
    engine.setMicrobitDrive("mb", { p0: { mode: "digital", value: 0 } });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-5);
  });

  it("drives an analog (PWM-duty) level onto P0", () => {
    const engine = new SimEngine();
    engine.load(microbitLedCircuit());
    // ~50% duty -> ~1.65 V; above the 1.8 V LED drop it barely conducts, so use ~90%.
    engine.setMicrobitDrive("mb", { p0: { mode: "analog", value: 920 } });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeGreaterThan(1e-4);
  });

  it("power=0 (USB unplugged) leaves a P0-driven LED dark", () => {
    const engine = new SimEngine();
    const circuit = microbitLedCircuit();
    circuit.components[0].params = { script: "", power: 0 };
    engine.load(circuit);
    engine.setMicrobitDrive("mb", { p0: { mode: "digital", value: 1 } });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-5);
  });
});

// P0-independent: the 3V pad sources 3.3 V for a breadboard rail when powered.
// 3V -> LED anode; LED cathode -> 220Ω -> GND.
function microbit3vRailCircuit(power: number): SimCircuit {
  return {
    components: [
      {
        id: "mb",
        kind: "microbit",
        pins: [{ id: "p0" }, { id: "p1" }, { id: "p2" }, { id: "3v" }, { id: "gnd" }],
        params: { script: "", power },
      },
      { id: "led", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { color: "red", vf: 1.8 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
    ],
    wires: [
      { from_component: "mb", from_pin: "3v", to_component: "led", to_pin: "a" },
      { from_component: "led", from_pin: "k", to_component: "r", to_pin: "a" },
      { from_component: "r", from_pin: "b", to_component: "mb", to_pin: "gnd" },
    ],
  };
}

describe("micro:bit — drive lifecycle across load()", () => {
  it("keeps a surviving board's drive across a topology edit (reload)", () => {
    const engine = new SimEngine();
    engine.load(microbitLedCircuit());
    engine.setMicrobitDrive("mb", { p0: { mode: "digital", value: 1 } });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeGreaterThan(1e-4);
    // An unrelated circuit edit reloads with the SAME board id: the embedded
    // sim is still running, so its program's drive must survive the reload.
    engine.load(microbitLedCircuit());
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeGreaterThan(1e-4);
  });

  it("prunes drives for boards deleted from the circuit", () => {
    const engine = new SimEngine();
    engine.load(microbitLedCircuit());
    engine.setMicrobitDrive("mb", { p0: { mode: "digital", value: 1 } });
    const without = microbitLedCircuit();
    without.components = without.components.filter((c) => c.id !== "mb");
    without.wires = [];
    engine.load(without);
    // Re-adding a board with the same id must NOT resurrect the old drive.
    engine.load(microbitLedCircuit());
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-5);
  });

  it("an empty drives push (host clear on re-flash) releases the pin", () => {
    const engine = new SimEngine();
    engine.load(microbitLedCircuit());
    engine.setMicrobitDrive("mb", { p0: { mode: "digital", value: 1 } });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeGreaterThan(1e-4);
    engine.setMicrobitDrive("mb", {});
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-5);
  });
});

// S18b Feature 3 — GPIO breakout: the same P0 bridge generalises to any pin
// in the component's CURRENT pin set (p8/p12-p16 when the dock is on), and a
// drive for a pin NOT in that set is a silent no-op (_pinNode returns -1).

// P8 -> LED anode; LED cathode -> 220Ω -> micro:bit GND. Pin set includes the
// six breakout-only pads, as it would after toggling params.breakout on.
function microbitBreakoutLedCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "mb",
        kind: "microbit",
        pins: [
          { id: "p0" }, { id: "p1" }, { id: "p2" },
          { id: "p8" }, { id: "p12" }, { id: "p13" }, { id: "p14" }, { id: "p15" }, { id: "p16" },
          { id: "3v" }, { id: "gnd" },
        ],
        params: { script: "", breakout: 1 },
      },
      { id: "led", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { color: "red", vf: 1.8 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
    ],
    wires: [
      { from_component: "mb", from_pin: "p8", to_component: "led", to_pin: "a" },
      { from_component: "led", from_pin: "k", to_component: "r", to_pin: "a" },
      { from_component: "r", from_pin: "b", to_component: "mb", to_pin: "gnd" },
    ],
  };
}

describe("micro:bit — breakout dock GPIO bridge (P8 drive)", () => {
  it("lights an LED when the program drives P8 high", () => {
    const engine = new SimEngine();
    engine.load(microbitBreakoutLedCircuit());
    engine.setMicrobitDrive("mb", { p8: { mode: "digital", value: 1 } });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    const i = Math.abs(engine.elementI["led"] ?? 0);
    expect(i).toBeGreaterThan(1e-4);
    expect(i).toBeLessThan(0.05);
  });

  it("leaves the LED dark when P8 is driven low", () => {
    const engine = new SimEngine();
    engine.load(microbitBreakoutLedCircuit());
    engine.setMicrobitDrive("mb", { p8: { mode: "digital", value: 0 } });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-5);
  });

  it("ignores a drive for a pin not in the component's current pin set", () => {
    // Bare-mode circuit (no p8..p16 pins) whose program nonetheless drives p16 —
    // e.g. stale drivesRef state right after toggling the breakout dock off.
    const engine = new SimEngine();
    const circuit = microbitLedCircuit(); // bare 5-pin component, LED wired to p0
    engine.load(circuit);
    engine.setMicrobitDrive("mb", { p16: { mode: "digital", value: 1 }, p0: { mode: "digital", value: 0 } });
    for (let i = 0; i < 5; i++) engine.step(0.001);
    // p16 has no node on this component (not in its pin set) -> no stamp, no
    // crash; p0 is explicitly low, so the LED stays dark either way.
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-5);
  });
});

describe("micro:bit — 3V pad sources a rail (power switch)", () => {
  it("lights a 3V->LED->R rail when powered on", () => {
    const engine = new SimEngine();
    engine.load(microbit3vRailCircuit(1));
    for (let i = 0; i < 5; i++) engine.step(0.001);
    const i = Math.abs(engine.elementI["led"] ?? 0);
    expect(i).toBeGreaterThan(1e-4); // ≈ (3.3 - 1.8)/(50+220) ≈ 5.5 mA
    expect(i).toBeLessThan(0.05);
  });

  it("kills the 3V rail when powered off", () => {
    const engine = new SimEngine();
    engine.load(microbit3vRailCircuit(0));
    for (let i = 0; i < 5; i++) engine.step(0.001);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-5);
  });
});
