import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const servoPrecisionHex = readFileSync(
  join(__dirname, "__fixtures__/servo_precision.hex"),
  "utf-8",
);

function source(id: string, voltage: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage },
  };
}

function circuit(supplyV: number, signalV: number): SimCircuit {
  return {
    components: [
      source("supply", supplyV),
      source("signal", signalV),
      {
        id: "servo",
        kind: "servo",
        catalogUid: "servo-sg90",
        pins: [{ id: "sig" }, { id: "vplus" }, { id: "gnd" }],
        params: {
          idleR: 330,
          minPulseMs: 1,
          maxPulseMs: 2,
          minAngle: 0,
          maxAngle: 180,
          maxSpeedDegPerSecond: 600,
          nominalVoltage: 4.8,
          minimumOperatingVoltage: 4.5,
          movingCurrent: 0.15,
        },
      },
    ],
    wires: [
      { from_component: "supply", from_pin: "pos", to_component: "servo", to_pin: "vplus" },
      { from_component: "supply", from_pin: "neg", to_component: "servo", to_pin: "gnd" },
      { from_component: "signal", from_pin: "pos", to_component: "servo", to_pin: "sig" },
      { from_component: "signal", from_pin: "neg", to_component: "servo", to_pin: "gnd" },
    ],
  };
}

function command(engine: SimEngine, supplyV: number, pulseSeconds: number): void {
  engine.load(circuit(supplyV, 5));
  engine.step(pulseSeconds);
  engine.load(circuit(supplyV, 0));
  engine.step(1e-6);
}

function arduinoServoCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex: servoPrecisionHex, usb_power: 1 },
      },
      {
        id: "servo",
        kind: "servo",
        catalogUid: "servo-sg90",
        pins: [{ id: "sig" }, { id: "vplus" }, { id: "gnd" }],
        params: {
          idleR: 330,
          minPulseMs: 1,
          maxPulseMs: 2,
          minAngle: 0,
          maxAngle: 180,
          maxSpeedDegPerSecond: 600,
          nominalVoltage: 4.8,
          minimumOperatingVoltage: 4.5,
          movingCurrent: 0.15,
        },
      },
    ],
    wires: [
      { from_component: "uno", from_pin: "d9", to_component: "servo", to_pin: "sig" },
      { from_component: "uno", from_pin: "5v", to_component: "servo", to_pin: "vplus" },
      { from_component: "uno", from_pin: "gnd", to_component: "servo", to_pin: "gnd" },
    ],
  };
}

describe("servo engine electromechanical integration", () => {
  it("decodes an Arduino pulse at cycle precision instead of 100 us / 18 degree steps", () => {
    const engine = new SimEngine();
    engine.load(arduinoServoCircuit());

    // Match the worker's MCU outer-step ceiling. The fixture emits a
    // 24,591-cycle HIGH pulse on D9: 1.5369375 ms at 16 MHz. A sampled decoder
    // can only report 1.5 or 1.6 ms (90 or 108 degrees); PinEvents retain the
    // actual off-grid command near 96.65 degrees.
    for (let step = 0; step < 200 && !Number.isFinite(engine.getServoState("servo")?.pulseMs); step++) {
      engine.step(100e-6);
    }

    const state = engine.getServoState("servo")!;
    expect(state.pulseMs).toBeCloseTo(1.5369375, 7);
    expect(state.targetAngle).toBeCloseTo(96.64875, 5);
    expect(Math.abs(state.targetAngle - Math.round(state.targetAngle / 18) * 18)).toBeGreaterThan(5);

    const decodedPulse = state.pulseMs;
    const decodedTarget = state.targetAngle;
    engine.load(arduinoServoCircuit());
    expect(engine.getServoState("servo")?.pulseMs).toBe(decodedPulse);
    expect(engine.getServoState("servo")?.targetAngle).toBe(decodedTarget);

    const snapshot = engine.saveState();
    engine.step(100e-6);
    engine.restoreState(snapshot);
    expect(engine.getServoState("servo")).toEqual(snapshot.servos.get("servo"));
  });

  it("decodes a target without teleporting and advances at the declared supply-scaled speed", () => {
    const engine = new SimEngine();
    engine.load(circuit(5, 0));
    command(engine, 5, 2e-3);

    const decoded = engine.getServoState("servo")!;
    expect(decoded.targetAngle).toBeCloseTo(180, 8);
    expect(decoded.angle).toBeGreaterThanOrEqual(90);
    expect(decoded.angle).toBeLessThan(91);
    expect(decoded.powered).toBe(true);

    const before = decoded.angle;
    engine.step(10e-3);
    const after = engine.getServoState("servo")!;
    // 600 deg/s at 4.8 V scales to 625 deg/s at the solved 5 V supply.
    expect(after.angle - before).toBeCloseTo(6.25, 6);
    expect(after.velocity).toBeCloseTo(625, 6);
    expect(after.moving).toBe(true);
  });

  it("stops below the exact model's brownout voltage while retaining the command target", () => {
    const engine = new SimEngine();
    engine.load(circuit(4, 0));
    command(engine, 4, 2e-3);

    const state = engine.getServoState("servo")!;
    expect(state.targetAngle).toBeCloseTo(180, 8);
    expect(state.angle).toBe(90);
    expect(state.powered).toBe(false);
    expect(state.velocity).toBe(0);
    expect(state.moving).toBe(false);
  });

  it("draws the declared moving current through the solved supply branch", () => {
    const engine = new SimEngine();
    engine.load(circuit(5, 0));
    command(engine, 5, 2e-3);

    // The command step used the previously committed idle load. The next step
    // stamps the now-committed moving load: R = 4.8 V / 0.15 A = 32 ohm.
    engine.step(1e-3);
    expect(engine.getElementI().servo).toBeCloseTo(5 / 32, 8);
    expect(engine.getServoState("servo")!.loadResistance).toBeCloseTo(32, 10);
  });

  it("replays actual angle, velocity, and load exactly after adaptive rollback", () => {
    const engine = new SimEngine();
    engine.load(circuit(5, 0));
    command(engine, 5, 2e-3);
    engine.step(1e-3);

    const snapshot = engine.saveState();
    engine.step(20e-3);
    const first = { ...engine.getServoState("servo")! };
    const firstCurrent = engine.getElementI().servo;

    engine.restoreState(snapshot);
    engine.step(20e-3);
    expect(engine.getServoState("servo")).toEqual(first);
    expect(engine.getElementI().servo).toBe(firstCurrent);
  });
});
