import { describe, expect, it } from "vitest";
import { buildPhysicsTelemetry } from "../../src/sim/physics-telemetry.js";
import { SimEngine, type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import rawCatalog from "../helpers/catalog.js";

function run(circuit: SimCircuit, steps = 2, h = 1e-4): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let index = 0; index < steps; index++) {
    engine.step(h);
    expect(engine.lastConverged).toBe(true);
  }
  return engine;
}

describe("structured physics telemetry", () => {
  it("invalidates the engine pin-to-net index on load", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "src", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      ],
      wires: [
        { from_component: "src", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "src", to_pin: "neg" },
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    expect(engine.getNetIdForPin("r", "a")).toBeDefined();

    engine.load({ components: [], wires: [] });
    expect(engine.getNetIdForPin("r", "a")).toBeUndefined();
  });

  it("reports signed two-terminal current and power with explicit provenance", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "src", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      ],
      wires: [
        { from_component: "src", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "src", to_pin: "neg" },
      ],
    };
    const engine = run(circuit);
    const telemetry = buildPhysicsTelemetry(engine, circuit, true);

    expect(telemetry.electricalSample).toBe("last-committed-solution");
    expect(telemetry.components.r.current?.valueA).toBeCloseTo(0.005, 8);
    expect(telemetry.components.r.current?.direction).toBe("positive-pin0-to-pin1");
    expect(telemetry.components.r.current?.provenance).toEqual({
      source: "engine",
      quality: "model-derived",
      method: "engine.getElementI",
    });
    expect(telemetry.components.r.power?.valueW).toBeCloseTo(0.025, 8);
    expect(telemetry.components.r.power?.signConvention).toBe("positive-absorbed");
    expect(telemetry.components.r.power?.provenance.source).toBe("worker-derived");

    // The source branch uses positive current into its positive terminal, so a
    // delivering source has negative signed terminal power.
    expect(telemetry.components.src.power?.valueW).toBeCloseTo(-0.025, 8);
  });

  it("reports capacitor terminal current but never exposes stale pre-solve electrical state", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "src", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
      ],
      wires: [
        { from_component: "src", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "c", to_pin: "a" },
        { from_component: "c", from_pin: "b", to_component: "src", to_pin: "neg" },
      ],
    };
    const engine = run(circuit, 1);

    const solved = buildPhysicsTelemetry(engine, circuit, true);
    expect(engine.getElementI().c).toBeGreaterThan(0);
    expect(solved.components.c.current).toMatchObject({
      reference: "pin0-to-pin1",
      direction: "positive-pin0-to-pin1",
    });
    expect(solved.components.c.current?.valueA).toBeCloseTo(engine.getElementI().c, 12);
    expect(solved.components.c.power?.valueW).toBeGreaterThan(0);

    const preSolve = buildPhysicsTelemetry(engine, circuit, false);
    expect(preSolve.electricalSample).toBe("unavailable-before-solve");
    expect(preSolve.components.r).toBeUndefined();
    expect(preSolve.components.src).toBeUndefined();
  });

  it("marks finite-source signal-generator current as positive-delivered and omits ambiguous power", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "sg",
          kind: "signal_gen",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { waveform: "dc", offset: 5, amplitude: 0, rSource: 50, enabled: 1 },
        },
        { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 50 } },
      ],
      wires: [
        { from_component: "sg", from_pin: "pos", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "sg", to_pin: "neg" },
      ],
    };
    const telemetry = buildPhysicsTelemetry(run(circuit), circuit, true);

    expect(telemetry.components.sg.current?.reference).toBe("source-output");
    expect(telemetry.components.sg.current?.direction).toBe("positive-delivered");
    expect(telemetry.components.sg.current?.valueA).toBeCloseTo(0.05, 6);
    expect(telemetry.components.sg.power).toBeUndefined();
  });

  it("normalizes an MCU USB source branch to positive board current draw", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "board",
          kind: "microbit",
          catalogUid: "mcu-microbit-v1",
          pins: [{ id: "p0" }, { id: "3v" }, { id: "gnd" }],
          params: { power: 1, vcc: 3.3, idleCurrent: 0.015 },
        },
      ],
      wires: [],
    };
    const telemetry = buildPhysicsTelemetry(run(circuit, 3), circuit, true);
    expect(telemetry.components.board.catalogUid).toBe("mcu-microbit-v1");
    expect(telemetry.components.board.current?.valueA).toBeCloseTo(0.015, 8);
    expect(telemetry.components.board.current).toMatchObject({
      reference: "supply",
      direction: "positive-draw",
      provenance: {
        source: "worker-derived",
        quality: "model-derived",
        method: "negated-mna-source-branch-current",
      },
    });
  });

  it("exposes only finite thermal and model state fields before the first solve", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "motor", kind: "dc_motor", pins: [{ id: "m1" }, { id: "m2" }], params: {} },
        { id: "servo", kind: "servo", pins: [{ id: "vplus" }, { id: "gnd" }, { id: "sig" }], params: {} },
        { id: "stepper", kind: "stepper", pins: [{ id: "a1" }, { id: "a2" }, { id: "b1" }, { id: "b2" }], params: {} },
        { id: "thermistor", kind: "thermistor", pins: [{ id: "a" }, { id: "b" }], params: {} },
        { id: "range", kind: "hcsr04", pins: [{ id: "vcc" }, { id: "trig" }, { id: "echo" }, { id: "gnd" }], params: {} },
      ],
      wires: [],
      environment: { temperatureC: 31 },
    };
    const engine = new SimEngine();
    engine.load(circuit);
    const telemetry = buildPhysicsTelemetry(engine, circuit, false);

    expect(telemetry.components.thermistor.temperature?.valueC).toBe(31);
    expect(telemetry.components.thermistor.temperature?.provenance.method).toBe("engine.getPartTemperature");
    expect(telemetry.components.motor.modelState).toMatchObject({
      kind: "dc-motor",
      windingCurrentA: 0,
      angularVelocityRadPerS: 0,
    });
    expect(telemetry.components.servo.modelState).toMatchObject({
      kind: "servo",
      angleDeg: 90,
      targetAngleDeg: 90,
      velocityDegPerS: 0,
      moving: false,
      powered: false,
      supplyVoltageV: 0,
    });
    expect(telemetry.components.servo.modelState).not.toHaveProperty("pulseWidthMs");
    expect(telemetry.components.stepper.modelState).not.toHaveProperty("phaseIndex");
    expect(telemetry.components.range.modelState).toMatchObject({ kind: "hcsr04", phase: "idle" });
    expect(telemetry.components.range.modelState).not.toHaveProperty("echoRiseSimTimeS");
  });

  it("reports exact package thermal provenance, load envelope, and state", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "src", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        {
          id: "r",
          kind: "resistor",
          catalogUid: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: 100 },
        },
      ],
      wires: [
        { from_component: "src", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "src", to_pin: "neg" },
      ],
      environment: { temperatureC: 25 },
    };
    const telemetry = buildPhysicsTelemetry(run(circuit, 1, 4), circuit, true);
    const temperature = telemetry.components.r.temperature;
    expect(temperature).toMatchObject({
      profileId: "axial-resistor-quarter-watt",
      profileLabel: "¼ W axial film resistor",
      dissipatedPowerW: 0.25,
      targetTemperatureC: 110,
      allowedPowerW: 0.25,
      withinContinuousLimits: true,
      thermalShutdown: false,
      provenance: { source: "engine", method: "engine.getThermalState" },
    });
    expect(temperature?.valueC).toBeGreaterThan(25);
    expect(temperature?.warnings?.length).toBeGreaterThan(0);
  });

  it("reports a PTC trip as known stress state without inventing progress or temperature", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "src", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.5 } },
        { id: "ptc", kind: "ptc_fuse", pins: [{ id: "a" }, { id: "b" }], params: { iHold: 0.5, rNormal: 0.5 } },
      ],
      wires: [
        { from_component: "src", from_pin: "pos", to_component: "ptc", to_pin: "a" },
        { from_component: "ptc", from_pin: "b", to_component: "src", to_pin: "neg" },
      ],
    };
    const engine = run(circuit, 200, 1e-4);
    const telemetry = buildPhysicsTelemetry(engine, circuit, true);
    const trip = telemetry.components.ptc.stress?.find((state) => state.state === "ptc-tripped");

    expect(trip).toMatchObject({
      state: "ptc-tripped",
      kind: "ptc_fuse_trip",
      provenance: { source: "engine", quality: "state-machine", method: "engine.getPtcTripped" },
    });
    expect(trip).not.toHaveProperty("value");
    expect(trip).not.toHaveProperty("limit");
    expect(telemetry.components.ptc.temperature).toBeUndefined();
  });

  it("preserves the engine's optional value/limit metadata for a latched failure", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "src", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
      ],
      wires: [
        { from_component: "src", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "r", from_pin: "b", to_component: "src", to_pin: "neg" },
      ],
    };
    // 2.5 W through the catalog's 0.25 W resistor rating accumulates the
    // normalized overload threshold in one 200 ms step; the second step solves
    // the now-open failure state before telemetry is sampled.
    const telemetry = buildPhysicsTelemetry(run(circuit, 2, 0.2), circuit, true);
    const failure = telemetry.components.r.stress?.find((state) => state.state === "latched-failure");

    expect(failure).toMatchObject({
      state: "latched-failure",
      kind: "resistor_overload",
      limit: 0.25,
      provenance: { source: "engine", quality: "state-machine", method: "engine.getFailures" },
    });
    // 2.5 W is solver-computed (I^2*R at the solved node voltages), so the
    // two linear backends round it differently at the last ulp; 12 decimals
    // still fails on any real drift while passing both backends.
    expect(failure?.value).toBeCloseTo(2.5, 12);
    expect(failure?.sinceSimTimeS).toBeCloseTo(0.2, 12);
  });

  it("labels loaded digital-output sag as a reversible warning", () => {
    const part = rawCatalog.parts.find((candidate) => candidate.uid === "ic-74ls00");
    if (!part?.pin_layout) throw new Error("74LS00 catalog fixture is missing its pin layout");
    const circuit: SimCircuit = {
      components: [
        { id: "src", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        {
          id: "u1",
          kind: "74ls00",
          catalogUid: part.uid,
          pins: part.pin_layout.map((pin) => ({ id: pin.id })),
          params: {},
        },
        { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      ],
      wires: [
        { from_component: "src", from_pin: "pos", to_component: "u1", to_pin: "vcc" },
        { from_component: "src", from_pin: "neg", to_component: "u1", to_pin: "gnd" },
        { from_component: "src", from_pin: "neg", to_component: "u1", to_pin: "1a" },
        { from_component: "src", from_pin: "neg", to_component: "u1", to_pin: "1b" },
        { from_component: "u1", from_pin: "1y", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "src", to_pin: "neg" },
      ],
    };
    const telemetry = buildPhysicsTelemetry(run(circuit, 2), circuit, true);
    const sag = telemetry.components.u1.stress?.find((state) => state.kind === "output_sag");

    expect(sag).toMatchObject({
      state: "reversible-warning",
      kind: "output_sag",
      pinId: "1y",
      provenance: { source: "engine", quality: "state-machine", method: "engine.getFailures" },
    });
    expect(telemetry.components.u1.stress?.some((state) => state.state === "latched-failure"))
      .toBe(false);
  });
});
