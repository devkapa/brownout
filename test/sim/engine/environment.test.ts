import { describe, expect, it } from "vitest";
import {
  SimEngine,
  junctionForwardVoltage,
  saturationCurrentAtTemperature,
  thermalVoltage,
  type SimCircuit,
} from "../../../src/sim/engine/sim-engine.js";

// Build a voltage divider: VCC → fixed-R → sensor → GND.
// The mid-node voltage (between fixed-R and sensor) rises when sensor R rises.
function ldrDividerCircuit(lux: number, rFixed = 10000): SimCircuit {
  return {
    components: [
      {
        id: "vcc",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "r_fixed",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: rFixed },
      },
      {
        id: "ldr1",
        kind: "ldr",
        pins: [{ id: "a" }, { id: "b" }],
        // rDark / rLight match catalog defaults; no lux param here — supplied via environment
        params: { rDark: 1000000, rLight: 500 },
      },
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: "r_fixed", to_pin: "a" },
      { from_component: "r_fixed", from_pin: "b", to_component: "ldr1", to_pin: "a" },
      { from_component: "ldr1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
    ],
    environment: { lux },
  };
}

function thermistorDividerCircuit(tempC: number, rFixed = 10000): SimCircuit {
  return {
    components: [
      {
        id: "vcc",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "r_fixed",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: rFixed },
      },
      {
        id: "th1",
        kind: "thermistor",
        pins: [{ id: "a" }, { id: "b" }],
        // rNominal / beta match catalog defaults; no tempC param — supplied via environment
        params: { rNominal: 10000, beta: 3950 },
      },
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: "r_fixed", to_pin: "a" },
      { from_component: "r_fixed", from_pin: "b", to_component: "th1", to_pin: "a" },
      { from_component: "th1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
    ],
    environment: { temperatureC: tempC },
  };
}

function runSteady(circuit: SimCircuit, steps = 80): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(1e-4);
  return engine;
}

// Return mid-node voltage between r_fixed and the sensor (pin "a" of sensor = pin "b" of r_fixed)
function midVoltage(engine: SimEngine): number {
  const nets = engine.nets;
  const net = nets.find((n) => n.pins.some(([c, p]) => c === "r_fixed" && p === "b"));
  if (!net) throw new Error("mid-node net not found");
  return engine.getNetV()[net.id] ?? 0;
}

// ─── LDR voltage divider ─────────────────────────────────────────────────────

describe("LDR voltage divider — environment.lux", () => {
  it("low lux (dark) → high LDR resistance → higher mid-node voltage", () => {
    // At lux=0: R_ldr = rDark = 1 MΩ → mid ≈ VCC * R_ldr / (rFixed + R_ldr) ≈ 5 V
    // At lux=10000 the GL5528 power law has reached the 500 Ω bright floor.
    const dark = runSteady(ldrDividerCircuit(0));
    const bright = runSteady(ldrDividerCircuit(10000));
    const vDark = midVoltage(dark);
    const vBright = midVoltage(bright);

    expect(vDark).toBeGreaterThan(vBright);
    // Sanity bounds
    expect(vDark).toBeGreaterThan(4);
    expect(vBright).toBeLessThan(1);
  });

  it("uses the GL5528-style R10/gamma power law", () => {
    const lux = 500;
    const rDark = 1_000_000;
    const rLight = 500;
    const rFixed = 10000;
    const r10 = 15_000;
    const gamma = 0.7;
    const expectedR = Math.max(rLight, Math.min(rDark, r10 * Math.pow(lux / 10, -gamma)));
    const expectedMid = 5 * expectedR / (rFixed + expectedR);

    const engine = runSteady(ldrDividerCircuit(lux, rFixed));
    expect(midVoltage(engine)).toBeCloseTo(expectedMid, 2);
  });
});

describe("ambient junction temperature", () => {
  it("computes kT/q at representative temperatures", () => {
    expect(thermalVoltage(-40)).toBeCloseTo(0.02009, 5);
    expect(thermalVoltage(25)).toBeCloseTo(0.02569, 5);
    expect(thermalVoltage(125)).toBeCloseTo(0.03431, 5);
  });

  it("moves rated-current forward voltage down as a junction heats", () => {
    expect(junctionForwardVoltage(1.8, 125)).toBeCloseTo(1.6, 6);
    expect(junctionForwardVoltage(1.8, -40)).toBeCloseTo(1.93, 6);
  });

  it("raises silicon saturation current with temperature", () => {
    const nominal = saturationCurrentAtTemperature(1e-14, 25);
    const hot = saturationCurrentAtTemperature(1e-14, 125);
    expect(nominal).toBeCloseTo(1e-14, 18);
    expect(hot).toBeGreaterThan(nominal * 10_000);
  });

  it("changes the solved LED forward drop in the physical direction", () => {
    const circuit = (temperatureC: number): SimCircuit => ({
      components: [
        { id: "v", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "led", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { vf: 1.8 } },
      ],
      wires: [
        { from_component: "v", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "led", to_pin: "a" },
        { from_component: "led", from_pin: "k", to_component: "v", to_pin: "neg" },
      ],
      environment: { temperatureC },
    });
    const ledVoltage = (temperatureC: number): number => {
      const engine = runSteady(circuit(temperatureC));
      const net = engine.nets.find((candidate) =>
        candidate.pins.some(([componentId, pinId]) => componentId === "led" && pinId === "a"),
      );
      if (!net) throw new Error("LED anode net missing");
      return engine.getNetV()[net.id] ?? Number.NaN;
    };

    expect(ledVoltage(125)).toBeLessThan(ledVoltage(-40) - 0.2);
  });
});

// ─── Thermistor voltage divider ───────────────────────────────────────────────

describe("thermistor voltage divider — environment.temperatureC", () => {
  it("higher temperature → lower NTC resistance → lower mid-node voltage", () => {
    const cool = runSteady(thermistorDividerCircuit(25));
    const hot = runSteady(thermistorDividerCircuit(60));
    expect(midVoltage(cool)).toBeGreaterThan(midVoltage(hot));
  });

  it("resistance law: R = rNominal * exp(beta * (1/T - 1/T0)), T0=298.15", () => {
    const tempC = 40;
    const rNominal = 10000;
    const beta = 3950;
    const rFixed = 10000;
    const T = tempC + 273.15;
    const T0 = 298.15;
    const expectedR = Math.max(1, rNominal * Math.exp(beta * (1 / T - 1 / T0)));
    const expectedMid = 5 * expectedR / (rFixed + expectedR);

    const engine = runSteady(thermistorDividerCircuit(tempC, rFixed));
    expect(midVoltage(engine)).toBeCloseTo(expectedMid, 2);
  });

  it("models first-order electrical self-heating and snapshot rollback", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "vcc",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        {
          id: "th1",
          kind: "thermistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: {
            rNominal: 10_000,
            beta: 3950,
            dissipationFactor: 0.001,
            thermalTau: 0.05,
          },
        },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "th1", to_pin: "a" },
        { from_component: "th1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
      environment: { temperatureC: 25 },
    };
    const engine = new SimEngine();
    engine.load(circuit);
    const initial = engine.saveState();
    for (let i = 0; i < 200; i++) engine.step(0.005);

    expect(engine.getPartTemperature("th1")).toBeGreaterThan(27);
    engine.restoreState(initial);
    expect(engine.getPartTemperature("th1")).toBeCloseTo(25, 6);
  });

  it("keeps a lightly excited thermistor near ambient", () => {
    const circuit = thermistorDividerCircuit(25, 1_000_000);
    const engine = new SimEngine();
    engine.load(circuit);
    for (let i = 0; i < 200; i++) engine.step(0.01);
    expect(engine.getPartTemperature("th1")).toBeLessThan(25.01);
  });
});

// ─── Legacy fallback: per-part params still work when environment is absent ──

describe("legacy fallback — per-part params.lux / params.tempC", () => {
  it("LDR: same voltage when stimulus comes from params.lux (no environment)", () => {
    const luxValue = 500;
    const rFixed = 10000;

    // Via environment (new path)
    const viaEnv = midVoltage(runSteady(ldrDividerCircuit(luxValue, rFixed)));

    // Via legacy per-part param (no environment on circuit)
    const legacyCircuit: SimCircuit = {
      components: [
        {
          id: "vcc",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        {
          id: "r_fixed",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: rFixed },
        },
        {
          id: "ldr1",
          kind: "ldr",
          pins: [{ id: "a" }, { id: "b" }],
          params: { rDark: 1000000, rLight: 500, lux: luxValue },
        },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "r_fixed", to_pin: "a" },
        { from_component: "r_fixed", from_pin: "b", to_component: "ldr1", to_pin: "a" },
        { from_component: "ldr1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
      // no environment field — exercises the legacy fallback chain
    };
    const viaParam = midVoltage(runSteady(legacyCircuit));
    expect(viaParam).toBeCloseTo(viaEnv, 4);
  });

  it("thermistor: same voltage when stimulus comes from params.tempC (no environment)", () => {
    const tempValue = 40;
    const rFixed = 10000;

    const viaEnv = midVoltage(runSteady(thermistorDividerCircuit(tempValue, rFixed)));

    const legacyCircuit: SimCircuit = {
      components: [
        {
          id: "vcc",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        {
          id: "r_fixed",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: rFixed },
        },
        {
          id: "th1",
          kind: "thermistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { rNominal: 10000, beta: 3950, tempC: tempValue },
        },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "r_fixed", to_pin: "a" },
        { from_component: "r_fixed", from_pin: "b", to_component: "th1", to_pin: "a" },
        { from_component: "th1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
      // no environment field — exercises the legacy fallback chain
    };
    const viaParam = midVoltage(runSteady(legacyCircuit));
    expect(viaParam).toBeCloseTo(viaEnv, 4);
  });
});
