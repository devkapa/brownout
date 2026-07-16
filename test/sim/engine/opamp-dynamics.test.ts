/**
 * Analytical and integration contracts for the op-amp transient macro-model.
 * Branch-current sign follows MNA: negative means the output sources current;
 * positive means it sinks current.
 */

import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";
import {
  opAmpCurrentLimitMode,
  opAmpDominantPoleCompanion,
  opAmpSlewLimitedTarget,
  opAmpTransientTarget,
} from "../../../src/sim/engine/elements.js";
import { runAcSweep } from "../../../src/analysis/run-ac-sweep.js";
import type { AcSweepSpec } from "../../../src/analysis/jobs.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const catalog = rawCatalog as unknown as PartCatalog;

function voltageSource(id: string, voltage: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage },
  };
}

function dualOpAmp(
  id: string,
  kind: "lm358" | "mcp6002" = "lm358",
  params: Record<string, number> = {},
): SimCircuit["components"][number] {
  return {
    id,
    kind,
    pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((pin) => ({ id: pin })),
    params,
  };
}

function wire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimCircuit["wires"][number] {
  return {
    from_component: fromComponent,
    from_pin: fromPin,
    to_component: toComponent,
    to_pin: toPin,
  };
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
  );
  if (!net) throw new Error(`No net for ${componentId}.${pinId}`);
  return engine.getNetV()[net.id] ?? Number.NaN;
}

function netId(circuit: SimCircuit, componentId: string, pinId: string): string {
  const engine = new SimEngine();
  engine.coldLoad(circuit);
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
  );
  if (!net) throw new Error(`No net for ${componentId}.${pinId}`);
  return net.id;
}

function followerCircuit(
  inputVoltage: number,
  params: Record<string, number> = {},
  loadResistance?: number,
): SimCircuit {
  const components: SimCircuit["components"] = [
    voltageSource("vcc", 5),
    voltageSource("vin", inputVoltage),
    dualOpAmp("oa", "lm358", params),
  ];
  const wires: SimCircuit["wires"] = [
    wire("vcc", "pos", "oa", "8"),
    wire("vcc", "neg", "oa", "4"),
    wire("vin", "pos", "oa", "3"),
    wire("vin", "neg", "vcc", "neg"),
    wire("oa", "1", "oa", "2"),
  ];
  if (loadResistance !== undefined) {
    components.push({
      id: "load",
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: loadResistance },
    });
    wires.push(
      wire("oa", "1", "load", "a"),
      wire("load", "b", "vcc", "neg"),
    );
  }
  return { components, wires };
}

function runSteps(engine: SimEngine, count: number, h: number): void {
  for (let step = 0; step < count; step++) {
    engine.step(h);
    expect(engine.lastConverged).toBe(true);
  }
}

describe("dominant-pole backward-Euler companion", () => {
  it("satisfies the declared first-order differential equation exactly", () => {
    const gain = 100_000;
    const gbw = 1_000_000;
    const h = 2e-6;
    const previous = 1.2;
    const differential = 8e-6;
    const offset = 0.1;
    const pole = opAmpDominantPoleCompanion(gain, gbw, h);
    const target = opAmpTransientTarget(previous, differential, gain, offset, gbw, h);

    expect(pole.poleHz).toBeCloseTo(10, 12);
    const lhs = (target - previous) / h;
    const rhs = 2 * Math.PI * pole.poleHz
      * (gain * differential + offset - target);
    expect(lhs).toBeCloseTo(rhs, 7);
    expect(pole.historyGain + pole.offsetGain).toBeCloseTo(1, 14);
  });

  it("limits a large-signal step to slewRate*h and commits current-limit hysteresis", () => {
    expect(opAmpSlewLimitedTarget(1, 5, 300_000, 1e-6)).toBeCloseTo(1.3, 12);
    expect(opAmpSlewLimitedTarget(1, -5, 300_000, 1e-6)).toBeCloseTo(0.7, 12);
    expect(opAmpCurrentLimitMode(0, -0.041, 0.04, 0.02)).toBe(-1);
    expect(opAmpCurrentLimitMode(0, -0.04, 0.04, 0.02)).toBe(-1);
    expect(opAmpCurrentLimitMode(-1, -0.0395, 0.04, 0.02)).toBe(-1);
    expect(opAmpCurrentLimitMode(-1, -0.038, 0.04, 0.02)).toBe(0);
    expect(opAmpCurrentLimitMode(0, 0.021, 0.04, 0.02)).toBe(1);
    expect(opAmpCurrentLimitMode(-1, 0.021, 0.04, 0.02)).toBe(1);
  });
});

describe("real-sine closed-loop bandwidth", () => {
  it("an LM358 follower is flat below GBW and about -3 dB at unity-gain bandwidth", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("vcc", 5),
        {
          id: "sg",
          kind: "signal_gen",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: {
            waveform: "sine",
            frequency: 100_000,
            amplitude: 0.01,
            offset: 2,
            rSource: 0,
            enabled: 1,
          },
        },
        dualOpAmp("oa"),
      ],
      wires: [
        wire("vcc", "pos", "oa", "8"),
        wire("vcc", "neg", "oa", "4"),
        wire("sg", "pos", "oa", "3"),
        wire("sg", "neg", "vcc", "neg"),
        wire("oa", "1", "oa", "2"),
      ],
    };
    const outputNet = netId(circuit, "oa", "1");
    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg",
      fromHz: 100_000,
      toHz: 1_000_000,
      points: 2,
      outputs: [{ netId: outputNet }],
    };

    const result = runAcSweep(spec, circuit);
    const magnitude = result.outputs[0]!.magnitudeDb;
    expect(result.pointFailures.every((failures) => failures.length === 0)).toBe(true);
    expect(magnitude[0]).toBeGreaterThan(-0.5);
    expect(magnitude[1]).toBeGreaterThan(-4.5);
    expect(
      magnitude[1],
      JSON.stringify({ magnitude, phase: result.outputs[0]!.phaseDeg, notSettled: result.notSettled }),
    ).toBeLessThan(-2);
  });
});

describe("large-signal slew and rollback", () => {
  it("moves no faster than the configured rate and repeats identically after restore", () => {
    const circuit = followerCircuit(0.5, {
      slewRate: 1_000,
      sourceCurrentLimit: 0,
      sinkCurrentLimit: 0,
    });
    const engine = new SimEngine();
    engine.load(circuit);
    runSteps(engine, 20, 1e-4);

    const input = circuit.components.find((component) => component.id === "vin");
    if (!input) throw new Error("input source missing");
    input.params.voltage = 3;
    engine.load(circuit);
    const before = netVoltage(engine, "oa", "1");
    const snapshot = engine.saveState();

    engine.step(1e-4);
    const first = netVoltage(engine, "oa", "1");
    expect(first - before).toBeGreaterThan(0.095);
    expect(first - before).toBeLessThanOrEqual(0.100_001);

    engine.restoreState(snapshot);
    engine.step(1e-4);
    expect(netVoltage(engine, "oa", "1")).toBeCloseTo(first, 10);
  });
});

describe("source/sink current limiting", () => {
  it("enters current limit in the same accepted solve when a hard load is switched in", () => {
    const params = {
      sourceCurrentLimit: 0.01,
      sinkCurrentLimit: 0.008,
      outputResistance: 50,
    };
    const circuit = followerCircuit(2, params);
    circuit.components.push(
      {
        id: "guard",
        kind: "ptc_fuse",
        pins: [{ id: "a" }, { id: "b" }],
        params: { iHold: 0.015, rNormal: 0.5 },
      },
      {
        id: "load",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1_000 },
      },
    );
    circuit.wires.push(
      wire("oa", "1", "guard", "a"),
      wire("guard", "b", "load", "a"),
      wire("load", "b", "vcc", "neg"),
    );

    const engine = new SimEngine();
    engine.load(circuit);
    runSteps(engine, 8, 1e-4);
    expect(Math.abs(engine.getElementI().oa ?? Number.NaN)).toBeLessThan(0.01);

    // Switch the already-running follower from a light load to a hard load.
    // The first accepted frame must be current-limited; accepting the natural
    // ~40 mA voltage-branch solution would also add false PTC I²t stress.
    const load = circuit.components.find((component) => component.id === "load");
    if (!load) throw new Error("load resistor missing");
    load.params.resistance = 1;
    engine.load(circuit);
    engine.step(1e-4);

    expect(engine.lastConverged).toBe(true);
    expect(engine.getElementI().oa).toBeCloseTo(-0.01, 8);
    expect(Math.abs(engine.getElementI().guard ?? Number.NaN)).toBeCloseTo(0.01, 8);
    expect(-(engine.getElementI().vcc ?? Number.NaN)).toBeCloseTo(0.0107, 5);
    const accepted = engine.saveState();
    expect(accepted.icState.get("oa")?.limit1).toBe(-1);
    expect(accepted.ptcs.get("guard")?.tripStress).toBe(0);
    expect(accepted.failureStress.size).toBe(0);
  });

  it("caps sourced short/load current with the documented negative branch sign", () => {
    const circuit = followerCircuit(2, {
      sourceCurrentLimit: 0.01,
      sinkCurrentLimit: 0.008,
      outputResistance: 50,
    }, 1);
    const engine = new SimEngine();
    engine.load(circuit);
    runSteps(engine, 8, 1e-4);

    expect(engine.getElementI().oa).toBeCloseTo(-0.01, 8);
    expect(netVoltage(engine, "oa", "1")).toBeCloseTo(0.01, 5);
    // The positive rail supplies the constrained 10 mA output plus 0.7 mA
    // package quiescent current; the clamp does not create hidden energy.
    expect(-(engine.getElementI().vcc ?? Number.NaN)).toBeCloseTo(0.0107, 5);
  });

  it("caps sink current when a low target is pulled toward VCC", () => {
    const circuit = followerCircuit(1, {
      sourceCurrentLimit: 0.01,
      sinkCurrentLimit: 0.008,
      outputResistance: 50,
    });
    circuit.components.push({
      id: "pullup",
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 1 },
    });
    circuit.wires.push(
      wire("vcc", "pos", "pullup", "a"),
      wire("pullup", "b", "oa", "1"),
    );
    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(1e-4);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getElementI().oa).toBeCloseTo(0.008, 8);
    runSteps(engine, 7, 1e-4);

    expect(engine.getElementI().oa).toBeCloseTo(0.008, 8);
    expect(netVoltage(engine, "oa", "1")).toBeCloseTo(4.992, 5);
  });

  it("recovers to bounded voltage operation in the solve where a limiting load is removed", () => {
    const params = {
      sourceCurrentLimit: 0.01,
      sinkCurrentLimit: 0.008,
      outputResistance: 50,
    };
    const loadedCircuit = followerCircuit(2, params, 1);
    const engine = new SimEngine();
    engine.load(loadedCircuit);
    runSteps(engine, 8, 1e-4);

    expect(engine.getElementI().oa).toBeCloseTo(-0.01, 8);
    expect(netVoltage(engine, "oa", "1")).toBeCloseTo(0.01, 5);

    // Preserve the op-amp state by ID while removing only the limiting load.
    // A fixed-current-only branch would now drive the 1 TΩ numerical shunt to
    // gigavolts (or fail Newton) before accepted-step hysteresis could recover.
    engine.load(followerCircuit(2, params));
    engine.step(1e-4);

    expect(engine.lastConverged).toBe(true);
    expect(netVoltage(engine, "oa", "1")).toBeGreaterThan(1.99);
    expect(netVoltage(engine, "oa", "1")).toBeLessThan(2.01);
    expect(Math.abs(engine.getElementI().oa ?? Number.NaN)).toBeLessThan(1e-6);
  });
});

describe("output energy is returned to the package rails", () => {
  it("the VCC source supplies output load current plus quiescent current", () => {
    const circuit = followerCircuit(2, {}, 1_000);
    const engine = new SimEngine();
    engine.load(circuit);
    runSteps(engine, 20, 1e-4);

    const outputBranchCurrent = engine.getElementI().oa ?? Number.NaN;
    const supplyBranchCurrent = engine.getElementI().vcc ?? Number.NaN;
    expect(outputBranchCurrent).toBeLessThan(0);
    expect(-supplyBranchCurrent).toBeCloseTo(-outputBranchCurrent + 0.0007, 5);
  });

  it("removes package and output-stage supply draw in the solve where V+ is disconnected", () => {
    const circuit = followerCircuit(2, {}, 1_000);
    const engine = new SimEngine();
    engine.load(circuit);
    runSteps(engine, 8, 1e-4);

    expect(netVoltage(engine, "oa", "8")).toBeCloseTo(5, 8);
    expect(-(engine.getElementI().vcc ?? Number.NaN)).toBeGreaterThan(0.0026);

    // Preserve the accepted op-amp state while removing only its V+ lead. A
    // stale quiescent-current stamp would inject 0.7 mA into this isolated net
    // and turn the 1 TΩ numerical shunt into a -700 MV false voltage.
    circuit.wires = circuit.wires.filter((candidate) => !(
      candidate.from_component === "vcc"
      && candidate.from_pin === "pos"
      && candidate.to_component === "oa"
      && candidate.to_pin === "8"
    ));
    engine.load(circuit);

    expect(engine.lastConverged).toBe(true);
    expect(netVoltage(engine, "oa", "8")).toBeCloseTo(0, 10);
    expect(netVoltage(engine, "oa", "1")).toBeCloseTo(0, 8);
    expect(Math.abs(engine.getElementI().vcc ?? Number.NaN)).toBeLessThan(1e-9);
    for (const voltage of Object.values(engine.getNetV())) {
      expect(Number.isFinite(voltage)).toBe(true);
      expect(Math.abs(voltage)).toBeLessThan(10);
    }
  });
});

describe("op-amp catalog model disclosure", () => {
  it("declares editable typical dynamics, current limits, units, and validity limits", () => {
    for (const uid of ["lm358", "mcp6002", "lm386"] as const) {
      const part = catalog.parts.find((candidate) => candidate.uid === uid);
      expect(Number(part?.default_params.gbw)).toBeGreaterThan(0);
      expect(Number(part?.default_params.slewRate)).toBeGreaterThan(0);
      expect(Number(part?.default_params.sourceCurrentLimit)).toBeGreaterThan(0);
      expect(Number(part?.default_params.sinkCurrentLimit)).toBeGreaterThan(0);
      expect(Number(part?.default_params.outputResistance)).toBeGreaterThan(0);
      expect(part?.paramUnits?.gbw).toBe("Hz");
      expect(part?.paramUnits?.slewRate).toBe("V/s");
      expect(part?.paramUnits?.sourceCurrentLimit).toBe("A");
      expect(part?.paramHelp?.gbw).toMatch(/dominant pole|bandwidth/i);
      expect(part?.paramAdvanced).toEqual(expect.arrayContaining([
        "gbw",
        "slewRate",
        "sourceCurrentLimit",
        "sinkCurrentLimit",
        "outputResistance",
      ]));
      expect(part?.electrical_specs?.notes).toMatch(/not modelled/i);
    }
  });
});
