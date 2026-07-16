import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";

function source(id: string, voltage: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage },
  };
}

function wire(
  from_component: string,
  from_pin: string,
  to_component: string,
  to_pin: string,
): SimCircuit["wires"][number] {
  return { from_component, from_pin, to_component, to_pin };
}

function voltageAt(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
  );
  return net ? (engine.getNetV()[net.id] ?? Number.NaN) : Number.NaN;
}

function resistorCircuit(catalogUid: string | null = "resistor"): SimCircuit {
  return {
    components: [
      source("supply", 5),
      {
        id: "load",
        kind: "resistor",
        ...(catalogUid ? { catalogUid } : {}),
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 100 },
      },
    ],
    wires: [
      wire("supply", "pos", "load", "a"),
      wire("supply", "neg", "load", "b"),
    ],
    environment: { temperatureC: 25, lux: 100 },
  };
}

function regulatorCircuit(): SimCircuit {
  return {
    components: [
      source("supply", 12),
      {
        id: "reg",
        kind: "linear_reg",
        catalogUid: "reg-7805",
        pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
        params: { vout: 5, vdropout: 2, iLimit: 1 },
      },
      {
        id: "load",
        kind: "resistor",
        catalogUid: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 5 },
      },
    ],
    wires: [
      wire("supply", "pos", "reg", "in"),
      wire("supply", "neg", "reg", "gnd"),
      wire("reg", "out", "load", "a"),
      wire("reg", "gnd", "load", "b"),
    ],
    environment: { temperatureC: 25, lux: 100 },
  };
}

function externallyHeldRegulatorOutputCircuit(): SimCircuit {
  return {
    components: [
      source("supply", 12),
      {
        id: "reg",
        kind: "linear_reg",
        catalogUid: "reg-7805",
        pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
        params: { vout: 5, vdropout: 2, iLimit: 1 },
      },
      source("output-bias", 3),
      {
        id: "bias-resistor",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1_000 },
      },
    ],
    wires: [
      wire("supply", "pos", "reg", "in"),
      wire("supply", "neg", "reg", "gnd"),
      wire("output-bias", "neg", "reg", "gnd"),
      wire("output-bias", "pos", "bias-resistor", "a"),
      wire("bias-resistor", "b", "reg", "out"),
    ],
    environment: { temperatureC: 25, lux: 100 },
  };
}

function ledCircuit(exactIdentity: boolean): SimCircuit {
  return {
    components: [
      source("supply", 5),
      {
        id: "limit",
        kind: "resistor",
        catalogUid: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 180 },
      },
      {
        id: "led",
        kind: "led",
        ...(exactIdentity ? { catalogUid: "led-red" } : {}),
        pins: [{ id: "a" }, { id: "k" }],
        params: { vf: 1.8, iRated: 0.02, n: 2 },
      },
    ],
    wires: [
      wire("supply", "pos", "limit", "a"),
      wire("limit", "b", "led", "a"),
      wire("led", "k", "supply", "neg"),
    ],
    environment: { temperatureC: 25, lux: 100 },
  };
}

function failedOpenLedCircuit(): SimCircuit {
  return {
    components: [
      source("supply", 2),
      {
        id: "led",
        kind: "led",
        catalogUid: "led-red",
        pins: [{ id: "a" }, { id: "k" }],
        params: { vf: 1.8, iRated: 0.02, n: 2 },
      },
    ],
    wires: [
      wire("supply", "pos", "led", "a"),
      wire("supply", "neg", "led", "k"),
    ],
    environment: { temperatureC: 25, lux: 100 },
  };
}

describe("engine package thermal coupling", () => {
  it.each(["lm358", "mcp6002", "lm386"] as const)(
    "derives %s quiescent heat and output units from catalog metadata",
    (uid) => {
      const catalog = rawCatalog as unknown as PartCatalog;
      const part = catalog.parts.find((candidate) => candidate.uid === uid)!;
      const vccPin = part.pin_layout.find((pin) => pin.function === "vcc")!.id;
      const gndPin = part.pin_layout.find((pin) => pin.function === "gnd")!.id;
      const quiescentCurrent = part.electrical_specs?.quiescent_current_a;
      if (quiescentCurrent === undefined) throw new Error(`${uid} is missing quiescent current metadata`);
      expect(quiescentCurrent).toBeGreaterThan(0);
      expect(part.pin_layout.some((pin) => pin.function === "analog_out")).toBe(true);

      const circuit: SimCircuit = {
        components: [
          source("supply", 5),
          {
            id: "amp",
            kind: part.kind,
            catalogUid: part.uid,
            pins: part.pin_layout.map((pin) => ({ id: pin.id })),
            params: { ...part.default_params },
          },
        ],
        wires: [
          wire("supply", "pos", "amp", vccPin),
          wire("supply", "neg", "amp", gndPin),
        ],
        environment: { temperatureC: 25, lux: 100 },
      };
      const engine = new SimEngine();
      engine.load(circuit);
      engine.step(0.01);
      expect(engine.getThermalState("amp")!.dissipatedPowerW)
        .toBeCloseTo(5 * quiescentCurrent, 9);
    },
  );

  it("advances an identified resistor with the analytical one-pole solution", () => {
    const engine = new SimEngine();
    engine.load(resistorCircuit());
    engine.step(4);

    const state = engine.getThermalState("load")!;
    // P=V²/R=0.25 W; T∞=25+0.25*340=110 °C; tau=8 s.
    const expected = 110 + (25 - 110) * Math.exp(-4 / 8);
    expect(state.dissipatedPowerW).toBeCloseTo(0.25, 9);
    expect(state.targetTemperatureC).toBeCloseTo(110, 8);
    expect(state.temperatureC).toBeCloseTo(expected, 8);
    expect(state.allowedPowerW).toBeCloseTo(0.25, 10);
    expect(state.withinContinuousLimits).toBe(true);
  });

  it("does not silently assign a package profile to an identity-less legacy part", () => {
    const engine = new SimEngine();
    engine.load(resistorCircuit(null));
    engine.step(4);
    expect(engine.getThermalState("load")).toBeUndefined();
    expect(engine.getPartTemperature("load")).toBeUndefined();
  });

  it("rejects a kind-mismatched thermal UID instead of borrowing its package model", () => {
    const engine = new SimEngine();
    engine.load(resistorCircuit("reg-7805"));
    engine.step(1);
    expect(engine.getThermalState("load")).toBeUndefined();
  });

  it("restores thermal energy exactly across a rejected/adaptive trial", () => {
    const engine = new SimEngine();
    engine.load(resistorCircuit());
    engine.step(1);
    const snapshot = engine.saveState();

    engine.step(2.5);
    const first = engine.getThermalState("load")!;
    engine.restoreState(snapshot);
    engine.step(2.5);
    expect(engine.getThermalState("load")).toEqual(first);
  });

  it("shuts an overheated 7805 down, cools with hysteresis, and restarts", () => {
    const engine = new SimEngine();
    engine.load(regulatorCircuit());

    // 5 V into 5 ohm demands 1 A. The pass loss is ~7 W, above the
    // 25 °C package allowance (~4.18 W), so a 25 s step crosses 150 °C.
    engine.step(25);
    let thermal = engine.getThermalState("reg")!;
    expect(thermal.dissipatedPowerW).toBeGreaterThan(7);
    expect(thermal.withinContinuousLimits).toBe(false);
    expect(thermal.thermalShutdown).toBe(true);

    // Preserve the hot package while replacing the load with an independently
    // biased output. Shutdown must disable the pass path: it must neither clamp
    // the output to ground nor backfeed that bias into the input.
    engine.load(externallyHeldRegulatorOutputCircuit());
    engine.step(0.01);
    expect(voltageAt(engine, "reg", "out")).toBeCloseTo(3, 6);
    expect(engine.getElementI().reg).toBeCloseTo(0, 12);
    expect(engine.getThermalState("reg")!.dissipatedPowerW).toBeCloseTo(0, 12);

    // 4 s of first-order cooling crosses the assumed 135 °C restart point.
    engine.step(4);
    thermal = engine.getThermalState("reg")!;
    expect(thermal.thermalShutdown).toBe(false);

    // One committed regime transition later, regulation resumes.
    engine.step(0.01);
    engine.step(0.01);
    expect(voltageAt(engine, "reg", "out")).toBeGreaterThan(4.9);
  });

  it("feeds an identified LED's package self-heating back into its junction law", () => {
    const engine = new SimEngine();
    engine.load(ledCircuit(true));
    engine.step(1e-3);
    const coldCurrent = engine.getElementI().led;

    engine.step(10);
    const hot = engine.getThermalState("led")!;
    expect(hot.temperatureC).toBeGreaterThan(38);
    engine.step(1e-3);
    const hotCurrent = engine.getElementI().led;

    // Negative Vf temperature coefficient raises current in the fixed-R series
    // circuit. This verifies that temperature is coupled, not display-only.
    expect(hotCurrent).toBeGreaterThan(coldCurrent);
  });

  it("cools a failed-open LED instead of heating the intact junction law across its open voltage", () => {
    const engine = new SimEngine();
    engine.load(failedOpenLedCircuit());
    engine.step(0.02);
    expect(Object.values(engine.getFailures()).some((failure) => failure.kind === "led_failed"))
      .toBe(true);
    const temperatureAtFailure = engine.getThermalState("led")!.temperatureC;

    engine.step(1);
    const failed = engine.getThermalState("led")!;
    expect(engine.getElementI().led).toBeCloseTo(0, 8);
    expect(failed.dissipatedPowerW).toBe(0);
    expect(failed.targetTemperatureC).toBe(25);
    expect(failed.temperatureC).toBeLessThan(temperatureAtFailure);

    engine.step(20);
    expect(engine.getThermalState("led")!.temperatureC).toBeCloseTo(25, 3);
  });
});
