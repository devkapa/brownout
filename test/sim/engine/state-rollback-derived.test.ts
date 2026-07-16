import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

function rgbCircuit(voltage: number): SimCircuit {
  return {
    components: [
      {
        id: "source",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage },
      },
      ...["r", "g", "b"].map((channel) => ({
        id: `limit-${channel}`,
        kind: "resistor" as const,
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 220 },
      })),
      {
        id: "rgb",
        kind: "rgb_led",
        pins: [{ id: "r_a" }, { id: "g_a" }, { id: "b_a" }, { id: "com_k" }],
        params: {},
      },
    ],
    wires: [
      ...["r", "g", "b"].flatMap((channel) => [
        {
          from_component: "source",
          from_pin: "pos",
          to_component: `limit-${channel}`,
          to_pin: "a",
        },
        {
          from_component: `limit-${channel}`,
          from_pin: "b",
          to_component: "rgb",
          to_pin: `${channel}_a`,
        },
      ]),
      { from_component: "rgb", from_pin: "com_k", to_component: "source", to_pin: "neg" },
    ],
  };
}

function max7219IdleCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "power",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      ...[
        ["cs", 5],
        ["clk", 0],
        ["din", 0],
      ].map(([pin, voltage]) => ({
        id: `drive-${pin}`,
        kind: "voltage_source" as const,
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: voltage as number },
      })),
      {
        id: "display",
        kind: "max7219",
        pins: [{ id: "din" }, { id: "clk" }, { id: "cs" }, { id: "vcc" }, { id: "gnd" }],
        params: {},
      },
    ],
    wires: [
      { from_component: "power", from_pin: "pos", to_component: "display", to_pin: "vcc" },
      { from_component: "power", from_pin: "neg", to_component: "display", to_pin: "gnd" },
      { from_component: "power", from_pin: "neg", to_component: "drive-cs", to_pin: "neg" },
      { from_component: "power", from_pin: "neg", to_component: "drive-clk", to_pin: "neg" },
      { from_component: "power", from_pin: "neg", to_component: "drive-din", to_pin: "neg" },
      { from_component: "drive-cs", from_pin: "pos", to_component: "display", to_pin: "cs" },
      { from_component: "drive-clk", from_pin: "pos", to_component: "display", to_pin: "clk" },
      { from_component: "drive-din", from_pin: "pos", to_component: "display", to_pin: "din" },
    ],
  };
}

describe("adaptive rollback derived-state integrity", () => {
  it("restores the converged vector used by channel currents and solver diagnostics", () => {
    const circuit = rgbCircuit(1);
    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(1e-4);
    const trustedChannels = engine.getElementChannelI();

    const trustedDiagnostics = {
      lastIters: 17,
      lastConverged: false,
      lastSolveUs: 123,
      lastMatrixSize: 42,
      lastMatrixSingular: true,
      lastMatrixIllConditioned: true,
      lastRelativeResidual: 0.25,
    };
    Object.assign(engine, trustedDiagnostics);
    const trusted = engine.saveState();

    circuit.components.find((component) => component.id === "source")!.params.voltage = 5;
    engine.step(1e-4);
    const trialChannels = engine.getElementChannelI();
    expect(trialChannels.rgb[0]).toBeGreaterThan(trustedChannels.rgb[0] + 1e-4);

    circuit.components.find((component) => component.id === "source")!.params.voltage = 1;
    engine.restoreState(trusted);

    expect(engine.getElementChannelI()).toEqual(trustedChannels);
    expect({
      lastIters: engine.lastIters,
      lastConverged: engine.lastConverged,
      lastSolveUs: engine.lastSolveUs,
      lastMatrixSize: engine.lastMatrixSize,
      lastMatrixSingular: engine.lastMatrixSingular,
      lastMatrixIllConditioned: engine.lastMatrixIllConditioned,
      lastRelativeResidual: engine.lastRelativeResidual,
    }).toEqual(trustedDiagnostics);
  });

  it("restores display output with the protocol state that produced it", () => {
    const engine = new SimEngine();
    engine.load(max7219IdleCircuit());
    engine.step(1e-4);

    const trusted = engine.saveState();
    const trustedDisplay = engine.getDisplayState();
    expect(trustedDisplay.display).toMatchObject({ kind: "max7219", on: false });

    const trialSeed = engine.saveState();
    trialSeed.icState.get("display")!.shutdown = 1;
    engine.restoreState(trialSeed);
    engine.step(1e-4);
    expect(engine.getDisplayState().display).toMatchObject({ kind: "max7219", on: true });

    engine.restoreState(trusted);
    expect(engine.getDisplayState()).toEqual(trustedDisplay);
  });
});
