/**
 * Analytical contracts for the lumped non-ideal capacitor and inductor models.
 *
 * Current is positive from a component's first pin to its second pin. Capacitor
 * state is the voltage across the internal ideal C (not the terminal voltage);
 * inductor state is the current through the series DCR + ideal L winding branch.
 */

import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";
import {
  capacitorCompanion,
  capacitorInternalVoltage,
  capacitorSeriesCurrent,
  inductorCompanion,
  inductorWindingCurrent,
  parallelLossCurrent,
} from "../../../src/sim/engine/elements.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const catalog = rawCatalog as unknown as PartCatalog;

function source(id: string, voltage: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage },
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

function terminalVoltage(engine: SimEngine, componentId: string): number {
  const component = engine.nets.flatMap((net) =>
    net.pins
      .filter(([id]) => id === componentId)
      .map(([, pin]) => [pin, engine.getNetV()[net.id] ?? 0] as const),
  );
  const byPin = new Map(component);
  return (byPin.get("a") ?? 0) - (byPin.get("b") ?? 0);
}

describe("capacitor ESR + leakage backward-Euler companion", () => {
  it("satisfies both the capacitor update and ESR voltage law", () => {
    const C = 1;
    const h = 0.5;
    const esr = 2;
    const vcPrev = 3;
    const terminalV = 5;

    const companion = capacitorCompanion(C, h, vcPrev, esr);
    const current = capacitorSeriesCurrent(terminalV, C, h, vcPrev, esr);
    const vc = capacitorInternalVoltage(vcPrev, current, C, h);

    expect(companion.conductance).toBeCloseTo(0.4, 12);
    expect(companion.historyCurrent).toBeCloseTo(1.2, 12);
    expect(current).toBeCloseTo(0.8, 12);
    expect(current).toBeCloseTo((C / h) * (vc - vcPrev), 12);
    expect(terminalV).toBeCloseTo(vc + esr * current, 12);
    expect(parallelLossCurrent(terminalV, 10)).toBeCloseTo(0.5, 12);
  });

  it("preserves the historical ideal companion when loss params are absent", () => {
    const companion = capacitorCompanion(2, 0.5, 3);
    expect(companion.conductance).toBe(4);
    expect(companion.historyCurrent).toBe(12);
    expect(parallelLossCurrent(5, Number.POSITIVE_INFINITY)).toBe(0);
  });

  it("reports total terminal current and snapshots the internal-C state", () => {
    const circuit: SimCircuit = {
      components: [
        source("src", 5),
        {
          id: "c",
          kind: "capacitor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { capacitance: 1, esr: 2, leakageResistance: 10 },
        },
      ],
      wires: [
        wire("src", "pos", "c", "a"),
        wire("c", "b", "src", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    engine.step(0.5);

    // New capacitors start at exactly 0 V. At h=0.5 s:
    // iSeries = 5/(2 + 0.5/1) = 2 A
    // iLeak = 5/10 = 0.5 A, vc = 5 - 2*iSeries = 1 V.
    expect(engine.getElementI().c).toBeCloseTo(2.5, 8);
    const snapshot = engine.saveState();
    expect(snapshot.caps.get("c")).toBeCloseTo(1, 8);
    expect(snapshot.capCurrents?.get("c")).toBeCloseTo(2.5, 8);

    engine.step(0.5);
    engine.restoreState(snapshot);
    expect(engine.getElementI().c).toBeCloseTo(2.5, 8);
    engine.step(0.5);
    expect(engine.getElementI().c).toBeCloseTo(2.1, 8);
  });

  it("stamps leakage as an actual terminal shunt", () => {
    const circuit: SimCircuit = {
      components: [
        source("src", 10),
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1_000 } },
        {
          id: "c",
          kind: "capacitor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { capacitance: 1e-9, esr: 0, leakageResistance: 1_000 },
        },
      ],
      wires: [
        wire("src", "pos", "r", "a"),
        wire("r", "b", "c", "a"),
        wire("c", "b", "src", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    for (let step = 0; step < 4; step++) engine.step(0.1);

    // At DC the ideal C is open, leaving a 1 kohm / 1 kohm divider.
    expect(terminalVoltage(engine, "c")).toBeCloseTo(5, 5);
    expect(engine.getElementI().c).toBeCloseTo(0.005, 6);
  });

  it("applies exact catalog ESR/leakage defaults to an older identified save", () => {
    const circuit: SimCircuit = {
      components: [
        source("src", 5),
        {
          id: "c",
          kind: "capacitor",
          catalogUid: "capacitor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { capacitance: 1 },
        },
      ],
      wires: [
        wire("src", "pos", "c", "a"),
        wire("c", "b", "src", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.coldLoad(circuit);
    engine.step(0.5);

    // Generic identified capacitor defaults: ESR=1 ohm, leakage=10 Mohm.
    const expectedSeries = 5 / (1 + 0.5 / 1);
    expect(engine.getElementI().c).toBeCloseTo(expectedSeries + 5 / 10_000_000, 7);
  });
});

describe("cold reset passive initial conditions", () => {
  it("leaves isolated capacitor voltage and inductor current at exact zero", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "c",
          kind: "capacitor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { capacitance: 1e-6 },
        },
        {
          id: "l",
          kind: "inductor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { inductance: 1e-3 },
        },
      ],
      wires: [],
    };
    const engine = new SimEngine();

    engine.coldLoad(circuit);

    const snapshot = engine.saveState();
    expect(snapshot.caps.get("c")).toBe(0);
    expect(snapshot.inds.get("l")).toBe(0);
  });

  it("does not expose hidden passive energy through high-impedance branches", () => {
    const circuit: SimCircuit = {
      components: [
        source("zero", 0),
        { id: "rc", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1e9 } },
        { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1e9 } },
        { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
        { id: "l", kind: "inductor", pins: [{ id: "a" }, { id: "b" }], params: { inductance: 1e-3 } },
      ],
      wires: [
        wire("zero", "pos", "rc", "a"),
        wire("rc", "b", "c", "a"),
        wire("c", "b", "zero", "neg"),
        wire("zero", "pos", "rl", "a"),
        wire("rl", "b", "l", "a"),
        wire("l", "b", "zero", "neg"),
      ],
    };
    const engine = new SimEngine();

    engine.coldLoad(circuit);

    expect(engine.lastConverged).toBe(true);
    expect(engine.saveState().caps.get("c")).toBe(0);
    expect(engine.saveState().inds.get("l")).toBe(0);
    expect(engine.getElementI().c).toBe(0);
    expect(engine.getElementI().l).toBe(0);
    expect(Object.values(engine.getNetV()).every((voltage) => voltage === 0)).toBe(true);
  });

  it("does not advance zero stored state during a powered t=0 seed solve", () => {
    const circuit: SimCircuit = {
      components: [
        source("src", 5),
        { id: "rc", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1e9 } },
        { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1e9 } },
        { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
        { id: "l", kind: "inductor", pins: [{ id: "a" }, { id: "b" }], params: { inductance: 1e-3 } },
      ],
      wires: [
        wire("src", "pos", "rc", "a"),
        wire("rc", "b", "c", "a"),
        wire("c", "b", "src", "neg"),
        wire("src", "pos", "rl", "a"),
        wire("rl", "b", "l", "a"),
        wire("l", "b", "src", "neg"),
      ],
    };
    const engine = new SimEngine();

    engine.coldLoad(circuit);

    expect(engine.lastConverged).toBe(true);
    expect(engine.simTime).toBe(0);
    expect(engine.saveState().caps.get("c")).toBe(0);
    expect(engine.saveState().inds.get("l")).toBe(0);
    // The source is already present in the algebraic t=0 solution; the test is
    // about stored energy, not pretending a powered network has zero current.
    expect(Object.values(engine.getElementI()).every(Number.isFinite)).toBe(true);
  });
});

describe("inductor DCR + core-loss backward-Euler companion", () => {
  it("satisfies the winding DCR + inductance voltage law", () => {
    const L = 2;
    const h = 0.5;
    const dcr = 3;
    const iPrev = 4;
    const terminalV = 7;

    const companion = inductorCompanion(L, h, iPrev, dcr);
    const current = inductorWindingCurrent(terminalV, L, h, iPrev, dcr);

    expect(companion.conductance).toBeCloseTo(1 / 7, 12);
    expect(companion.historyCurrent).toBeCloseTo(16 / 7, 12);
    expect(current).toBeCloseTo(23 / 7, 12);
    expect(terminalV).toBeCloseTo((L / h) * (current - iPrev) + dcr * current, 12);
    expect(parallelLossCurrent(terminalV, 14)).toBeCloseTo(0.5, 12);
  });

  it("preserves the historical ideal companion when loss params are absent", () => {
    const companion = inductorCompanion(2, 0.5, 4);
    expect(companion.conductance).toBe(0.25);
    expect(companion.historyCurrent).toBe(4);
  });

  it("stamps DCR and core loss and reports their combined terminal current", () => {
    const circuit: SimCircuit = {
      components: [
        source("src", 10),
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
        {
          id: "l",
          kind: "inductor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { inductance: 1e-6, dcr: 10, coreLossResistance: 20 },
        },
      ],
      wires: [
        wire("src", "pos", "r", "a"),
        wire("r", "b", "l", "a"),
        wire("l", "b", "src", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    for (let step = 0; step < 4; step++) engine.step(0.01);

    // DC branch: 10 ohm winding in parallel with 20 ohm core-loss shunt,
    // fed through 10 ohm. Vterminal=4 V, iWinding=.4 A, iCore=.2 A.
    expect(terminalVoltage(engine, "l")).toBeCloseTo(4, 5);
    expect(engine.saveState().inds.get("l")).toBeCloseTo(0.4, 5);
    expect(engine.getElementI().l).toBeCloseTo(0.6, 5);
  });

  it("applies exact catalog DCR/core-loss defaults to an older identified save", () => {
    const circuit: SimCircuit = {
      components: [
        source("src", 10),
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
        {
          id: "l",
          kind: "inductor",
          catalogUid: "inductor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { inductance: 1e-6 },
        },
      ],
      wires: [
        wire("src", "pos", "r", "a"),
        wire("r", "b", "l", "a"),
        wire("l", "b", "src", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.coldLoad(circuit);
    for (let step = 0; step < 4; step++) engine.step(0.01);

    // Exact inductor defaults: 5 ohm winding DCR in parallel with 100 kohm.
    const parallelResistance = 1 / (1 / 5 + 1 / 100_000);
    const expectedTerminal = 10 * parallelResistance / (10 + parallelResistance);
    expect(terminalVoltage(engine, "l")).toBeCloseTo(expectedTerminal, 6);
    expect(engine.getElementI().l).toBeCloseTo(expectedTerminal / 5 + expectedTerminal / 100_000, 6);
  });
});

describe("passive parasitic catalog controls", () => {
  it("exposes values, units, labels, help, and explicit model limits", () => {
    const capacitors = catalog.parts.filter((part) => part.kind === "capacitor");
    expect(capacitors).toHaveLength(4);
    for (const part of capacitors) {
      expect(Number(part.default_params.esr)).toBeGreaterThan(0);
      expect(Number(part.default_params.leakageResistance)).toBeGreaterThan(0);
      expect(part.paramUnits?.esr).toBe("Ω");
      expect(part.paramUnits?.leakageResistance).toBe("Ω");
      expect(part.paramLabels?.esr).toMatch(/ESR/);
      expect(part.paramHelp?.leakageResistance).toMatch(/disable leakage/);
      expect(part.paramAdvanced).toEqual(expect.arrayContaining(["esr", "leakageResistance"]));
      expect(part.electrical_specs?.notes).toMatch(/not modelled/i);
    }

    const inductor = catalog.parts.find((part) => part.uid === "inductor");
    expect(Number(inductor?.default_params.dcr)).toBeGreaterThan(0);
    expect(Number(inductor?.default_params.coreLossResistance)).toBeGreaterThan(0);
    expect(inductor?.paramUnits?.dcr).toBe("Ω");
    expect(inductor?.paramLabels?.coreLossResistance).toMatch(/Core-loss/);
    expect(inductor?.paramHelp?.dcr).toMatch(/ideal winding/);
    expect(inductor?.paramAdvanced).toEqual(expect.arrayContaining(["dcr", "coreLossResistance"]));
    expect(inductor?.electrical_specs?.notes).toMatch(/Saturation.*not modelled/i);
  });
});
