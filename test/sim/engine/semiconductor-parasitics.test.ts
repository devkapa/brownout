/**
 * Physics contracts for the optional BJT Early effect and lumped MOS gate
 * capacitances. Expected values are derived independently from the declared
 * equations; transient tests exercise real two-terminal charge paths.
 */

import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import {
  bjtCurrents,
  stampBJT,
  stampMOSFET,
} from "../../../src/sim/engine/elements.js";
import { MNA } from "../../../src/sim/engine/mna.js";
import { SimEngine, thermalVoltage, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const catalog = rawCatalog;

function voltageSource(id: string, voltage: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage },
  };
}

function resistor(id: string, resistance: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "resistor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { resistance },
  };
}

function mosfet(
  id: string,
  cgs: number,
  cgd: number,
  vto = 100,
): SimCircuit["components"][number] {
  return {
    id,
    kind: "nmos",
    pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
    params: { vto, k: 0.01, lambda: 0, cgs, cgd },
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

function bjtBiasCircuit(earlyVoltage: number): SimCircuit {
  return {
    components: [
      voltageSource("collector", 5),
      voltageSource("base", 0.65),
      {
        id: "q",
        kind: "bjt_npn",
        pins: [{ id: "c" }, { id: "b" }, { id: "e" }],
        params: {
          Is: 1e-15,
          betaF: 100,
          betaR: 2,
          nF: 1,
          nR: 1,
          earlyVoltage,
        },
      },
    ],
    wires: [
      wire("collector", "pos", "q", "c"),
      wire("base", "pos", "q", "b"),
      wire("collector", "neg", "q", "e"),
      wire("base", "neg", "collector", "neg"),
    ],
    environment: { temperatureC: 25 },
  };
}

describe("BJT forward Early effect", () => {
  it("adds the declared VCE/VA transport-current slope and preserves PNP symmetry", () => {
    const Is = 1e-15;
    const betaF = 100;
    const betaR = 2;
    const vt = 0.02585;
    const va = 80;
    const low = bjtCurrents(0.65, 1, 0, 1, Is, betaF, betaR, 1, 1, vt, va);
    const high = bjtCurrents(0.65, 6, 0, 1, Is, betaF, betaR, 1, 1, vt, va);

    const iForward = Is * (Math.exp(0.65 / vt) - 1);
    const reverseLow = Is * (Math.exp((0.65 - 1) / vt) - 1);
    const reverseHigh = Is * (Math.exp((0.65 - 6) / vt) - 1);
    const expectedLow = iForward * (1 + 1 / va) - reverseLow * (1 + 1 / betaR);
    const expectedHigh = iForward * (1 + 6 / va) - reverseHigh * (1 + 1 / betaR);

    expect(low.ic).toBeCloseTo(expectedLow, 14);
    expect(high.ic).toBeCloseTo(expectedHigh, 14);
    expect(high.ic).toBeGreaterThan(low.ic);
    expect(low.ib + low.ic + low.ie).toBe(0);
    expect(high.ib + high.ic + high.ie).toBe(0);

    const mirrored = bjtCurrents(-0.65, -6, 0, -1, Is, betaF, betaR, 1, 1, vt, va);
    expect(mirrored.ib).toBeCloseTo(-high.ib, 14);
    expect(mirrored.ic).toBeCloseTo(-high.ic, 14);
    expect(mirrored.ie).toBeCloseTo(-high.ie, 14);
  });

  it("keeps the historical Ebers-Moll result exactly when VA is absent or zero", () => {
    const absent = bjtCurrents(0.65, 5, 0, 1, 1e-15, 100, 2, 1, 1, 0.02585);
    const disabled = bjtCurrents(0.65, 5, 0, 1, 1e-15, 100, 2, 1, 1, 0.02585, 0);
    const invalid = bjtCurrents(0.65, 5, 0, 1, 1e-15, 100, 2, 1, 1, 0.02585, Number.NaN);
    expect(disabled).toEqual(absent);
    expect(invalid).toEqual(absent);
  });

  it("stamps a collector Jacobian that matches finite differences of reported current", () => {
    const b = 0;
    const c = 1;
    const e = 2;
    const point = [0.65, 5, 0] as const;
    const args = [1 as const, 1e-15, 100, 2, 1, 1, 0.02585, 80] as const;
    const mna = new MNA(3);
    const stamped = stampBJT(mna, b, c, e, ...point, ...args);
    const reported = bjtCurrents(...point, ...args);

    expect(stamped.ib).toBeCloseTo(reported.ib, 14);
    expect(stamped.ic).toBeCloseTo(reported.ic, 14);
    expect(stamped.ie).toBeCloseTo(reported.ie, 14);

    const epsilon = 1e-6;
    for (let column = 0; column < 3; column++) {
      const plus = [...point];
      const minus = [...point];
      plus[column] += epsilon;
      minus[column] -= epsilon;
      const derivative = (
        bjtCurrents(plus[0]!, plus[1]!, plus[2]!, ...args).ic
        - bjtCurrents(minus[0]!, minus[1]!, minus[2]!, ...args).ic
      ) / (2 * epsilon);
      expect(mna.G[c * mna.size + column]).toBeCloseTo(derivative, 8);
      const columnKcl = mna.G[b * mna.size + column]
        + mna.G[c * mna.size + column]
        + mna.G[e * mna.size + column];
      expect(Math.abs(columnKcl)).toBeLessThan(1e-15);
    }
  });

  it("passes Early voltage through both the nonlinear stamp and engine current readout", () => {
    const va = 80;
    const engine = new SimEngine();
    engine.load(bjtBiasCircuit(va));
    engine.step(1e-5);
    expect(engine.lastConverged).toBe(true);

    const expected = bjtCurrents(
      0.65,
      5,
      0,
      1,
      1e-15,
      100,
      2,
      1,
      1,
      thermalVoltage(25),
      va,
    ).ic;
    expect(engine.getElementI().q).toBeCloseTo(expected, 12);
  });

  it("applies an exact catalog Early-voltage default to an older identified save", () => {
    const circuit = bjtBiasCircuit(0);
    const transistor = circuit.components.find((component) => component.id === "q")!;
    transistor.catalogUid = "bjt-npn";
    delete transistor.params.earlyVoltage;
    const engine = new SimEngine();
    engine.coldLoad(circuit);
    engine.step(1e-5);

    const expected = bjtCurrents(
      0.65,
      5,
      0,
      1,
      1e-15,
      100,
      2,
      1,
      1,
      thermalVoltage(25),
      75,
    ).ic;
    expect(engine.getElementI().q).toBeCloseTo(expected, 12);
  });
});

describe("MOSFET lumped gate capacitance", () => {
  it("stamps Cgs and Cgd as conservative two-terminal BE companions", () => {
    const mna = new MNA(3);
    const d = 0;
    const g = 1;
    const s = 2;
    const h = 1e-3;
    const cgs = 2e-6;
    const cgd = 3e-6;
    const vgsPrev = 0.4;
    const vgdPrev = -0.2;
    stampMOSFET(
      mna,
      d,
      g,
      s,
      0,
      0,
      0,
      1,
      100,
      0.01,
      0,
      0.7,
      0.02585,
      { h, cgs, cgd, vgsPrev, vgdPrev },
    );

    expect(mna.G[g * mna.size + g]).toBeCloseTo((cgs + cgd) / h, 14);
    expect(mna.G[g * mna.size + s]).toBeCloseTo(-cgs / h, 14);
    expect(mna.G[g * mna.size + d]).toBeCloseTo(-cgd / h, 14);
    expect(mna.b[g]).toBeCloseTo((cgs * vgsPrev + cgd * vgdPrev) / h, 14);
    expect(mna.b[g] + mna.b[s] + mna.b[d]).toBeCloseTo(0, 14);
  });

  it("charges a resistively driven gate with the closed-form BE time step and rolls back exactly", () => {
    const h = 1e-3;
    const resistance = 1_000;
    const cgs = 1e-6;
    const cgd = 2e-6;
    const circuit: SimCircuit = {
      components: [
        voltageSource("drive", 0),
        resistor("rg", resistance),
        mosfet("m", cgs, cgd),
      ],
      wires: [
        wire("drive", "pos", "rg", "a"),
        wire("rg", "b", "m", "g"),
        wire("drive", "neg", "m", "s"),
        wire("drive", "neg", "m", "d"),
      ],
    };
    const engine = new SimEngine();
    engine.coldLoad(circuit);
    const snapshot = engine.saveState();
    expect(snapshot.mosfetGates?.get("m")).toEqual({ vgs: 0, vgd: 0 });

    circuit.components[0]!.params.voltage = 5;
    engine.step(h);
    expect(engine.lastConverged).toBe(true);
    const expectedGate = 5 * h / (resistance * (cgs + cgd) + h);
    const firstGate = netVoltage(engine, "m", "g");
    const firstState = engine.saveState().mosfetGates?.get("m");
    expect(firstGate).toBeCloseTo(expectedGate, 8);
    expect(firstState?.vgs).toBeCloseTo(expectedGate, 8);
    expect(firstState?.vgd).toBeCloseTo(expectedGate, 8);

    engine.restoreState(snapshot);
    expect(engine.saveState().mosfetGates?.get("m")).toEqual({ vgs: 0, vgd: 0 });
    engine.step(h);
    expect(netVoltage(engine, "m", "g")).toBeCloseTo(firstGate, 12);
    expect(engine.saveState().mosfetGates?.get("m")).toEqual(firstState);
  });

  it("applies exact catalog Cgs/Cgd defaults to an older identified save", () => {
    const h = 1e-8;
    const resistance = 1_000;
    const circuit: SimCircuit = {
      components: [
        voltageSource("drive", 0),
        resistor("rg", resistance),
        {
          id: "m",
          kind: "nmos",
          catalogUid: "nmos",
          pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
          params: { vto: 100, k: 0.01, lambda: 0 },
        },
      ],
      wires: [
        wire("drive", "pos", "rg", "a"),
        wire("rg", "b", "m", "g"),
        wire("drive", "neg", "m", "s"),
        wire("drive", "neg", "m", "d"),
      ],
    };
    const engine = new SimEngine();
    engine.coldLoad(circuit);
    circuit.components[0]!.params.voltage = 5;
    engine.step(h);
    const totalGateCapacitance = 5.5e-11 + 5e-12;
    const expected = 5 * h / (resistance * totalGateCapacitance + h);
    expect(netVoltage(engine, "m", "g")).toBeCloseTo(expected, 8);
  });

  it("couples a drain step into the gate through Cgd without a hidden reference", () => {
    const h = 1e-3;
    const resistance = 1_000;
    const cgs = 1e-6;
    const cgd = 2e-6;
    const circuit: SimCircuit = {
      components: [
        voltageSource("drain-drive", 0),
        resistor("rg", resistance),
        mosfet("m", cgs, cgd),
      ],
      wires: [
        wire("drain-drive", "pos", "m", "d"),
        wire("drain-drive", "neg", "m", "s"),
        wire("rg", "a", "m", "g"),
        wire("rg", "b", "drain-drive", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.coldLoad(circuit);
    circuit.components[0]!.params.voltage = 3;
    engine.step(h);
    expect(engine.lastConverged).toBe(true);

    // Gate-node KCL for this first BE step:
    // Vg/R + Cgs*Vg/h + Cgd*(Vg - Vd)/h = 0.
    const expectedGate = (cgd * 3 / h) / (1 / resistance + (cgs + cgd) / h);
    expect(netVoltage(engine, "m", "g")).toBeCloseTo(expectedGate, 8);
    expect(netVoltage(engine, "m", "d")).toBeCloseTo(3, 10);
    expect(engine.getElementI().m).toBeCloseTo(cgd * (3 - expectedGate) / h, 8);
  });

  it("reports intrinsic body-diode current at the drain terminal", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("reverse", -0.7),
        mosfet("m", 0, 0),
      ],
      wires: [
        wire("reverse", "pos", "m", "d"),
        wire("reverse", "neg", "m", "s"),
        wire("reverse", "neg", "m", "g"),
      ],
    };
    const engine = new SimEngine();
    engine.coldLoad(circuit);
    engine.step(1e-5);
    expect(engine.lastConverged).toBe(true);
    // NMOS body diode conducts source -> drain, so current entering the drain
    // terminal is negative. The 0.7 V / 1 A model anchor gives about -1 A.
    expect(engine.getElementI().m).toBeCloseTo(-1, 3);
  });

  for (const temperatureC of [-40, 125]) {
    it(`keeps ${String(temperatureC)} °C body-diode readout consistent with solved KCL`, () => {
      const circuit: SimCircuit = {
        components: [
          voltageSource("reverse", -0.7),
          mosfet("m", 0, 0),
        ],
        wires: [
          wire("reverse", "pos", "m", "d"),
          wire("reverse", "neg", "m", "s"),
          wire("reverse", "neg", "m", "g"),
        ],
        environment: { temperatureC },
      };
      const engine = new SimEngine();
      engine.coldLoad(circuit);
      engine.step(1e-5);
      expect(engine.lastConverged).toBe(true);

      const mosfetCurrent = engine.getElementI().m ?? Number.NaN;
      const sourceCurrent = engine.getElementI().reverse ?? Number.NaN;
      expect(Math.abs(mosfetCurrent + sourceCurrent))
        .toBeLessThanOrEqual(Math.max(1e-9, Math.abs(mosfetCurrent) * 1e-9));
    });
  }
});

describe("semiconductor catalog model disclosure", () => {
  it("declares exact editable Early-voltage and gate-capacitance defaults with limits", () => {
    const npn = catalog.parts.find((part) => part.uid === "bjt-npn");
    const pnp = catalog.parts.find((part) => part.uid === "bjt-pnp");
    const nmos = catalog.parts.find((part) => part.uid === "nmos");
    const pmos = catalog.parts.find((part) => part.uid === "pmos");

    expect(npn?.default_params.earlyVoltage).toBe(75);
    expect(pnp?.default_params.earlyVoltage).toBe(100);
    for (const part of [npn, pnp]) {
      expect(part?.paramUnits?.earlyVoltage).toBe("V");
      expect(part?.paramAdvanced).toContain("earlyVoltage");
      expect(part?.electrical_specs?.notes).toMatch(/representative fixed forward-active/i);
      expect(part?.electrical_specs?.notes).toMatch(/not modelled/i);
    }

    expect(nmos?.default_params.cgs).toBe(5.5e-11);
    expect(nmos?.default_params.cgd).toBe(5e-12);
    expect(pmos?.default_params.cgs).toBe(4e-11);
    expect(pmos?.default_params.cgd).toBe(2e-11);
    for (const part of [nmos, pmos]) {
      expect(part?.paramUnits?.cgs).toBe("F");
      expect(part?.paramUnits?.cgd).toBe("F");
      expect(part?.paramAdvanced).toEqual(expect.arrayContaining(["cgs", "cgd"]));
      expect(part?.electrical_specs?.notes).toMatch(/displacement current.*Miller coupling/i);
      expect(part?.electrical_specs?.notes).toMatch(/voltage-dependent capacitance.*not modelled/i);
    }
  });
});
