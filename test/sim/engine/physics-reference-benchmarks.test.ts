/**
 * Physics reference benchmarks
 * ============================
 *
 * These are analytical golden tests, not implementation snapshots. Every
 * benchmark states the physical/model envelope in which its reference is
 * valid and an explicit numerical error budget. A model change that leaves
 * this envelope must add a new reference derivation instead of merely widening
 * a tolerance until the test passes.
 *
 * Full contracts and deferred conservation checks are documented in
 * docs/physics-reference-benchmarks.md.
 */

import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const K_B_OVER_Q_V_PER_K = 8.617_333_262_145e-5;
const RSHUNT_G_SIEMENS = 1e-12;

type SimComponent = SimCircuit["components"][number];
type SimWire = SimCircuit["wires"][number];

function voltageSource(id: string, voltage: number): SimComponent {
  return {
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage },
  };
}

function resistor(id: string, resistance: number): SimComponent {
  return {
    id,
    kind: "resistor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { resistance },
  };
}

function wire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimWire {
  return {
    from_component: fromComponent,
    from_pin: fromPin,
    to_component: toComponent,
    to_pin: toPin,
  };
}

function nodeVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
  );
  if (!net) throw new Error(`Benchmark topology error: no net for ${componentId}.${pinId}`);
  const value = engine.getNetV()[net.id];
  if (value === undefined || !Number.isFinite(value)) {
    throw new Error(`Benchmark solve error: ${componentId}.${pinId} has no finite voltage`);
  }
  return value;
}

function componentVoltage(
  engine: SimEngine,
  componentId: string,
  positivePin: string,
  negativePin: string,
): number {
  return nodeVoltage(engine, componentId, positivePin)
    - nodeVoltage(engine, componentId, negativePin);
}

function acceptedStep(engine: SimEngine, h: number, context: string): void {
  engine.step(h);
  expect(
    engine.lastConverged,
    `${context}: solver must converge before the value is benchmarkable; ` +
    `iterations=${String(engine.lastIters)}, singular=${String(engine.lastMatrixSingular)}, ` +
    `illConditioned=${String(engine.lastMatrixIllConditioned)}, residual=${String(engine.lastRelativeResidual)}`,
  ).toBe(true);
  expect(Number.isFinite(engine.lastRelativeResidual), `${context}: linear residual must be finite`).toBe(true);
}

function runSteady(circuit: SimCircuit, steps = 12, h = 1e-4): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let index = 0; index < steps; index++) {
    acceptedStep(engine, h, `steady step ${String(index)}`);
  }
  return engine;
}

function setSourceVoltage(circuit: SimCircuit, sourceId: string, voltage: number): void {
  const source = circuit.components.find((component) => component.id === sourceId);
  if (!source) throw new Error(`Benchmark topology error: source ${sourceId} missing`);
  source.params.voltage = voltage;
}

function expectAbsoluteError(
  actual: number,
  expected: number,
  maxError: number,
  label: string,
): void {
  expect(
    Math.abs(actual - expected),
    `${label}: actual=${String(actual)}, reference=${String(expected)}, budget=${String(maxError)}`,
  ).toBeLessThanOrEqual(maxError);
}

// ─── Linear DC references ────────────────────────────────────────────────────

describe("reference physics — resistor dividers and loading", () => {
  /**
   * Validity envelope: ideal 9 V source, two linear lumped resistors, DC,
   * nominal values, no wire/contact resistance or meter loading.
   * Acceptance: |Vsim - 6 V| <= 0.2 uV. The budget includes the declared
   * 1 Tohm numerical node shunt, which contributes only picamp loading here.
   */
  it("unloaded 10k/20k divider matches 6 V within 0.2 uV", () => {
    const circuit: SimCircuit = {
      components: [voltageSource("src", 9), resistor("top", 10_000), resistor("bottom", 20_000)],
      wires: [
        wire("src", "pos", "top", "a"),
        wire("top", "b", "bottom", "a"),
        wire("bottom", "b", "src", "neg"),
      ],
    };

    const engine = runSteady(circuit, 2);
    expectAbsoluteError(nodeVoltage(engine, "top", "b"), 6, 0.2e-6, "unloaded divider");
  });

  /**
   * Validity envelope: the preceding divider with a second ideal 20 kohm
   * resistor loading the output. The two lower resistors are 10 kohm in
   * parallel, so the analytical result is 9*10k/(10k+10k) = 4.5 V.
   * Acceptance: absolute voltage error <= 0.2 uV.
   */
  it("20k load changes the divider to 4.5 V within 0.2 uV", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("src", 9),
        resistor("top", 10_000),
        resistor("bottom", 20_000),
        resistor("load", 20_000),
      ],
      wires: [
        wire("src", "pos", "top", "a"),
        wire("top", "b", "bottom", "a"),
        wire("top", "b", "load", "a"),
        wire("bottom", "b", "src", "neg"),
        wire("load", "b", "src", "neg"),
      ],
    };

    const engine = runSteady(circuit, 2);
    expectAbsoluteError(nodeVoltage(engine, "top", "b"), 4.5, 0.2e-6, "loaded divider");
  });
});

// ─── First-order transient references ────────────────────────────────────────

interface StepCase {
  stepsPerTau: number;
  maxFullScaleError: number;
}

const FIRST_ORDER_STEP_CASES: readonly StepCase[] = [
  { stepsPerTau: 100, maxFullScaleError: 0.003 },
  { stepsPerTau: 20, maxFullScaleError: 0.012 },
  { stepsPerTau: 10, maxFullScaleError: 0.025 },
];

function rcStepCircuit(resistance: number, capacitance: number): SimCircuit {
  return {
    components: [
      voltageSource("src", 0),
      resistor("r", resistance),
      {
        id: "c",
        kind: "capacitor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { capacitance },
      },
    ],
    wires: [
      wire("src", "pos", "r", "a"),
      wire("r", "b", "c", "a"),
      wire("c", "b", "src", "neg"),
    ],
  };
}

function rlStepCircuit(resistance: number, inductance: number): SimCircuit {
  return {
    components: [
      voltageSource("src", 0),
      resistor("r", resistance),
      {
        id: "l",
        kind: "inductor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { inductance },
      },
    ],
    wires: [
      wire("src", "pos", "r", "a"),
      wire("r", "b", "l", "a"),
      wire("l", "b", "src", "neg"),
    ],
  };
}

describe("reference physics — RC step response at multiple timesteps", () => {
  const resistance = 1_000;
  const capacitance = 10e-6;
  const supply = 5;
  const tau = resistance * capacitance;

  for (const { stepsPerTau, maxFullScaleError } of FIRST_ORDER_STEP_CASES) {
    /**
     * Validity envelope: ideal lumped R and C, zero source impedance, a 0->5 V
     * step, no ESR/leakage/ESL/dielectric absorption. Reference curve is
     * Vc(t)=V+(V0-V)*exp(-t/RC), with V0 measured immediately before the step
     * so the assertion remains valid if the zero-input settling solve changes.
     * Acceptance: full-scale error at t=tau is bounded per timestep below.
     */
    it(`h=tau/${String(stepsPerTau)} stays within ${(maxFullScaleError * 100).toFixed(1)}% full-scale at t=tau`, () => {
      const h = tau / stepsPerTau;
      const circuit = rcStepCircuit(resistance, capacitance);
      const engine = new SimEngine();
      engine.load(circuit);
      acceptedStep(engine, h, "RC zero-input initialization");
      const initialVoltage = componentVoltage(engine, "c", "a", "b");

      setSourceVoltage(circuit, "src", supply);
      engine.load(circuit);
      for (let index = 0; index < stepsPerTau; index++) {
        acceptedStep(engine, h, `RC tau step ${String(index)}`);
      }

      const expected = supply + (initialVoltage - supply) * Math.exp(-1);
      const actual = componentVoltage(engine, "c", "a", "b");
      const fullScale = Math.abs(supply - initialVoltage);
      expectAbsoluteError(
        actual,
        expected,
        maxFullScaleError * fullScale,
        `RC h=tau/${String(stepsPerTau)}`,
      );
    });
  }
});

describe("reference physics — RL step response at multiple timesteps", () => {
  const resistance = 10;
  const inductance = 10e-3;
  const supply = 1;
  const tau = inductance / resistance;
  const steadyCurrent = supply / resistance;

  for (const { stepsPerTau, maxFullScaleError } of FIRST_ORDER_STEP_CASES) {
    /**
     * Validity envelope: ideal lumped R and L, zero source impedance, a 0->1 V
     * step, no winding resistance beyond R, core loss, saturation, hysteresis,
     * or parasitic capacitance. Reference is
     * I(t)=Iinf+(I0-Iinf)*exp(-tR/L), with measured pre-step I0.
     * Acceptance: full-scale current error at t=tau is bounded per timestep.
     */
    it(`h=tau/${String(stepsPerTau)} stays within ${(maxFullScaleError * 100).toFixed(1)}% full-scale at t=tau`, () => {
      const h = tau / stepsPerTau;
      const circuit = rlStepCircuit(resistance, inductance);
      const engine = new SimEngine();
      engine.load(circuit);
      acceptedStep(engine, h, "RL zero-input initialization");
      const initialCurrent = engine.getElementI().l ?? Number.NaN;
      expect(Number.isFinite(initialCurrent)).toBe(true);

      setSourceVoltage(circuit, "src", supply);
      engine.load(circuit);
      for (let index = 0; index < stepsPerTau; index++) {
        acceptedStep(engine, h, `RL tau step ${String(index)}`);
      }

      const expected = steadyCurrent + (initialCurrent - steadyCurrent) * Math.exp(-1);
      const actual = engine.getElementI().l ?? Number.NaN;
      expectAbsoluteError(
        actual,
        expected,
        maxFullScaleError * Math.abs(steadyCurrent - initialCurrent),
        `RL h=tau/${String(stepsPerTau)}`,
      );
    });
  }
});

// ─── Junction-temperature references ─────────────────────────────────────────

function junctionBiasCircuit(
  kind: "diode" | "led",
  temperatureC: number,
  vfAt25C: number,
  resistance: number,
  emission: number,
): SimCircuit {
  return {
    components: [
      voltageSource("src", 5),
      resistor("r", resistance),
      {
        id: "junction",
        kind,
        pins: [{ id: "a" }, { id: "k" }],
        params: { vf: vfAt25C, iRated: 0.01, n: emission },
      },
    ],
    wires: [
      wire("src", "pos", "r", "a"),
      wire("r", "b", "junction", "a"),
      wire("junction", "k", "src", "neg"),
    ],
    environment: { temperatureC },
  };
}

describe("reference physics — diode and LED temperature direction", () => {
  /**
   * Validity envelope: steady forward bias around the declared 10 mA anchor,
   * -40..125 C ambient equals junction temperature, Shockley junction with the
   * declared -2 mV/C rated-current tempco; no self-heating or series resistance.
   * Acceptance: hot forward drop is lower and the 165 C shift is 0.33 V +/-30 mV.
   */
  it("silicon diode forward drop falls by 0.33 V +/- 30 mV from -40 C to 125 C", () => {
    const cold = runSteady(junctionBiasCircuit("diode", -40, 0.7, 430, 1));
    const hot = runSteady(junctionBiasCircuit("diode", 125, 0.7, 430, 1));
    const coldDrop = componentVoltage(cold, "junction", "a", "k");
    const hotDrop = componentVoltage(hot, "junction", "a", "k");
    const observedShift = coldDrop - hotDrop;

    expect(hotDrop).toBeLessThan(coldDrop);
    expectAbsoluteError(observedShift, 0.33, 0.03, "diode temperature shift");
  });

  /**
   * Validity envelope: same rated-current temperature benchmark as the diode,
   * using the LED emission coefficient n=2 and Vf25=1.8 V. Optical output,
   * wavelength drift, self-heating and package thermal impedance are excluded.
   * Acceptance: hot drop is lower and the shift is 0.33 V +/-30 mV.
   */
  it("LED forward drop falls by 0.33 V +/- 30 mV from -40 C to 125 C", () => {
    const cold = runSteady(junctionBiasCircuit("led", -40, 1.8, 320, 2));
    const hot = runSteady(junctionBiasCircuit("led", 125, 1.8, 320, 2));
    const coldDrop = componentVoltage(cold, "junction", "a", "k");
    const hotDrop = componentVoltage(hot, "junction", "a", "k");
    const observedShift = coldDrop - hotDrop;

    expect(hotDrop).toBeLessThan(coldDrop);
    expectAbsoluteError(observedShift, 0.33, 0.03, "LED temperature shift");
  });
});

describe("reference physics — guarded junction overdrive", () => {
  /**
   * Validity envelope: one forced-voltage silicon diode beyond the model's
   * exponential overflow guard. The compact law deliberately continues along
   * the boundary tangent; package resistance, heating, high injection, and
   * destructive failure are outside this mathematical consistency check.
   * Acceptance: reported diode current must equal the MNA source-branch current
   * to one part per billion, apart from the declared 1 Tohm node shunt.
   */
  it("keeps tangent-extended diode telemetry consistent with the solved branch", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("src", 1.2),
        {
          id: "junction",
          kind: "diode",
          pins: [{ id: "a" }, { id: "k" }],
          params: { vf: 0.7, iRated: 0.1, n: 1 },
        },
      ],
      wires: [
        wire("src", "pos", "junction", "a"),
        wire("junction", "k", "src", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    const engine = runSteady(circuit, 2, 1e-6);
    const diodeCurrent = engine.getElementI().junction ?? Number.NaN;
    const sourceCurrent = engine.getElementI().src ?? Number.NaN;

    expect(diodeCurrent).toBeGreaterThan(1);
    expect(Math.abs(diodeCurrent + sourceCurrent) / Math.abs(diodeCurrent))
      .toBeLessThanOrEqual(1e-9);
  });
});

// ─── Static transistor references ────────────────────────────────────────────

function bjtForcedBiasCircuit(baseVoltage: number): SimCircuit {
  return {
    components: [
      voltageSource("collector-bias", 5),
      voltageSource("base-bias", baseVoltage),
      {
        id: "q",
        kind: "bjt_npn",
        pins: [{ id: "c" }, { id: "b" }, { id: "e" }],
        params: { Is: 1e-16, betaF: 100, betaR: 1, nF: 1, nR: 1 },
      },
    ],
    wires: [
      wire("collector-bias", "pos", "q", "c"),
      wire("base-bias", "pos", "q", "b"),
      wire("collector-bias", "neg", "q", "e"),
      wire("base-bias", "neg", "collector-bias", "neg"),
    ],
    environment: { temperatureC: 25 },
  };
}

function mosForcedBiasCircuit(gateVoltage: number): SimCircuit {
  return {
    components: [
      voltageSource("drain-bias", 2),
      voltageSource("gate-bias", gateVoltage),
      {
        id: "m",
        kind: "nmos",
        pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
        params: { vto: 1, k: 0.01, lambda: 0 },
      },
    ],
    wires: [
      wire("drain-bias", "pos", "m", "d"),
      wire("gate-bias", "pos", "m", "g"),
      wire("drain-bias", "neg", "m", "s"),
      wire("gate-bias", "neg", "drain-bias", "neg"),
    ],
  };
}

describe("reference physics — BJT and MOSFET DC sanity", () => {
  /**
   * Validity envelope: NPN Ebers-Moll forward-active region at 25 C with
   * forced VBE=0.7 V, VCE=5 V, Is=1e-16 A, betaF=100, betaR=1. Early effect,
   * high-injection beta roll-off, breakdown and self-heating are excluded.
   * Acceptance: collector current agrees with the closed-form Ebers-Moll
   * equation within 1%.
   */
  it("NPN forced-bias collector current matches forward-active Ebers-Moll within 1%", () => {
    const engine = runSteady(bjtForcedBiasCircuit(0.7), 20, 1e-5);
    const vt = K_B_OVER_Q_V_PER_K * (25 + 273.15);
    const saturationCurrent = 1e-16;
    const forward = saturationCurrent * (Math.exp(0.7 / vt) - 1);
    const reverse = saturationCurrent * (Math.exp((0.7 - 5) / vt) - 1);
    const expectedCollector = forward - reverse * (1 + 1 / 1);
    const actualCollector = engine.getElementI().q ?? Number.NaN;

    expect(Math.abs(actualCollector - expectedCollector) / expectedCollector).toBeLessThanOrEqual(0.01);
  });

  /**
   * Validity envelope: static level-1/Shichman-Hodges NMOS, VTO=1 V,
   * K=0.01 A/V^2, lambda=0, forced VGS=5 V and VDS=2 V (triode region),
   * with the body diode reverse-biased. Gate charge, subthreshold current,
   * capacitances, mobility temperature dependence and self-heating are excluded.
   * Acceptance: triode current within 0.5%; cutoff readout below 1 nA.
   */
  it("NMOS triode and cutoff currents match the level-1 static equations", () => {
    const on = runSteady(mosForcedBiasCircuit(5), 12, 1e-5);
    const off = runSteady(mosForcedBiasCircuit(0.5), 12, 1e-5);
    const overdrive = 5 - 1;
    const vds = 2;
    const expectedOn = 0.01 * (2 * overdrive * vds - vds * vds);
    const actualOn = on.getElementI().m ?? Number.NaN;
    const actualOff = off.getElementI().m ?? Number.NaN;

    expect(Math.abs(actualOn - expectedOn) / expectedOn).toBeLessThanOrEqual(0.005);
    expect(Math.abs(actualOff)).toBeLessThan(1e-9);
  });
});

// ─── Conservation references ─────────────────────────────────────────────────

describe("reference physics — KCL and source power balance", () => {
  /**
   * Validity envelope: DC network containing one ideal independent source and
   * linear resistors only. Source current uses the engine's documented
   * positive-into-source branch convention. No behavioural/digital output is
   * included because several such models do not yet debit their supply rails.
   * Acceptance: midpoint KCL residual <= 0.1 nA and total signed power residual
   * <= 0.5 nW, including the effect of the 1 Tohm numerical shunts.
   */
  it("loaded divider conserves current and signed power within declared residuals", () => {
    const supply = 10;
    const topR = 1_000;
    const bottomR = 2_000;
    const loadR = 2_000;
    const circuit: SimCircuit = {
      components: [
        voltageSource("src", supply),
        resistor("top", topR),
        resistor("bottom", bottomR),
        resistor("load", loadR),
      ],
      wires: [
        wire("src", "pos", "top", "a"),
        wire("top", "b", "bottom", "a"),
        wire("top", "b", "load", "a"),
        wire("bottom", "b", "src", "neg"),
        wire("load", "b", "src", "neg"),
      ],
    };
    const engine = runSteady(circuit, 3);
    const midpoint = nodeVoltage(engine, "top", "b");
    const iTop = (supply - midpoint) / topR;
    const iBottom = midpoint / bottomR;
    const iLoad = midpoint / loadR;
    const kclResidual = iTop - iBottom - iLoad;
    expect(Math.abs(kclResidual)).toBeLessThanOrEqual(0.1e-9);

    const currents = engine.getElementI();
    const sourcePower = supply * (currents.src ?? Number.NaN);
    const resistorPower =
      (currents.top ?? Number.NaN) ** 2 * topR
      + (currents.bottom ?? Number.NaN) ** 2 * bottomR
      + (currents.load ?? Number.NaN) ** 2 * loadR;
    const powerResidual = sourcePower + resistorPower;

    expect(sourcePower).toBeLessThan(0);
    expect(Math.abs(powerResidual)).toBeLessThanOrEqual(0.5e-9);
  });
});

describe("reference physics — transient KCL/KVL and energy residuals", () => {
  /**
   * Validity envelope: ideal series RC step using backward Euler at h=tau/100.
   * The discrete energy identity includes BE's numerical dissipation term
   * 0.5*C*(DeltaV)^2; that term is integration loss, not claimed dielectric loss.
   * Acceptance over 25 consecutive steps: KCL residual <= 10 nA and normalized
   * energy residual <= 1e-6 of source work per step. The one-part-per-million
   * budget is consistent with the companion-law and node-shunt residuals.
   */
  it("RC companion satisfies KCL and its discrete energy identity", () => {
    const resistance = 1_000;
    const capacitance = 10e-6;
    const supply = 5;
    const h = (resistance * capacitance) / 100;
    const circuit = rcStepCircuit(resistance, capacitance);
    const engine = new SimEngine();
    engine.load(circuit);
    acceptedStep(engine, h, "RC conservation initialization");
    let previousVoltage = componentVoltage(engine, "c", "a", "b");
    setSourceVoltage(circuit, "src", supply);
    engine.load(circuit);

    for (let index = 0; index < 25; index++) {
      acceptedStep(engine, h, `RC conservation step ${String(index)}`);
      const voltage = componentVoltage(engine, "c", "a", "b");
      const resistorCurrent = (supply - voltage) / resistance;
      const capacitorCurrent = capacitance * (voltage - previousVoltage) / h;
      // Every non-reference node has the declared 1 Tohm numerical shunt.
      const kclResidual = resistorCurrent - capacitorCurrent - RSHUNT_G_SIEMENS * voltage;
      expect(Math.abs(kclResidual)).toBeLessThanOrEqual(10e-9);

      const sourceWork = supply * resistorCurrent * h;
      const resistorHeat = resistorCurrent * resistorCurrent * resistance * h;
      const storedEnergyChange = 0.5 * capacitance * (voltage * voltage - previousVoltage * previousVoltage);
      const backwardEulerDamping = 0.5 * capacitance * (voltage - previousVoltage) ** 2;
      const shuntHeat = RSHUNT_G_SIEMENS * voltage * voltage * h;
      const energyResidual = sourceWork - resistorHeat - storedEnergyChange
        - backwardEulerDamping - shuntHeat;
      expect(Math.abs(energyResidual) / Math.abs(sourceWork)).toBeLessThanOrEqual(1e-6);
      previousVoltage = voltage;
    }
  });

  /**
   * Validity envelope: ideal series RL step using backward Euler at h=tau/100.
   * The discrete energy identity includes BE numerical dissipation
   * 0.5*L*(DeltaI)^2; winding/core/parasitic losses are outside this model.
   * Acceptance over 25 steps: KVL residual <= 1 uV and normalized energy
   * residual <= 1e-6 of source work per step. Multiplying the KVL budget by
   * step charge gives the same one-part-per-million energy budget.
   */
  it("RL companion satisfies KVL and its discrete energy identity", () => {
    const resistance = 10;
    const inductance = 10e-3;
    const supply = 1;
    const h = (inductance / resistance) / 100;
    const circuit = rlStepCircuit(resistance, inductance);
    const engine = new SimEngine();
    engine.load(circuit);
    acceptedStep(engine, h, "RL conservation initialization");
    let previousCurrent = engine.getElementI().l ?? Number.NaN;
    setSourceVoltage(circuit, "src", supply);
    engine.load(circuit);

    for (let index = 0; index < 25; index++) {
      acceptedStep(engine, h, `RL conservation step ${String(index)}`);
      const current = engine.getElementI().l ?? Number.NaN;
      const kvlResidual = supply - resistance * current
        - inductance * (current - previousCurrent) / h;
      expect(Math.abs(kvlResidual)).toBeLessThanOrEqual(1e-6);

      const sourceWork = supply * current * h;
      const resistorHeat = current * current * resistance * h;
      const storedEnergyChange = 0.5 * inductance * (current * current - previousCurrent * previousCurrent);
      const backwardEulerDamping = 0.5 * inductance * (current - previousCurrent) ** 2;
      const energyResidual = sourceWork - resistorHeat - storedEnergyChange - backwardEulerDamping;
      const normalizedEnergyResidual = Math.abs(energyResidual) / Math.abs(sourceWork);
      expect(
        normalizedEnergyResidual,
        `RL step ${String(index)}: energy=${String(normalizedEnergyResidual)}, ` +
        `KVL=${String(kvlResidual)} V, linear=${String(engine.lastRelativeResidual)}, ` +
        `i0=${String(previousCurrent)}, i=${String(current)}, ` +
        `vsrc=${String(componentVoltage(engine, "src", "pos", "neg"))}, ` +
        `vl=${String(componentVoltage(engine, "l", "a", "b"))}`,
      ).toBeLessThanOrEqual(1e-6);
      previousCurrent = current;
    }
  });
});

describe("reference physics — active-device rail current and power", () => {
  /**
   * Validity envelope: one 74HC14 inverter driving a resistive load HIGH from
   * an ideal 5 V rail. The teaching output stage derives 62.5 ohm from the
   * catalog's 20 mA rating (VCC / 4*Imax, with a 50 ohm floor);
   * package static current is excluded. Acceptance: the rail debit matches the
   * load current within 10 nA and source/load/output-resistance power balances
   * within 50 nW.
   */
  it("digital output load current is debited from the package VCC rail", () => {
    const loadR = 1_000;
    const outputR = 5 / (4 * 0.020);
    const circuit: SimCircuit = {
      components: [
        voltageSource("vdd", 5),
        voltageSource("input", 0),
        resistor("load", loadR),
        {
          id: "gate",
          kind: "74hc14",
          pins: [
            { id: "1a" }, { id: "1y" }, { id: "2a" }, { id: "2y" },
            { id: "3a" }, { id: "3y" }, { id: "gnd" }, { id: "4y" },
            { id: "4a" }, { id: "5y" }, { id: "5a" }, { id: "6y" },
            { id: "6a" }, { id: "vcc" },
          ],
          params: {},
        },
      ],
      wires: [
        wire("vdd", "pos", "gate", "vcc"),
        wire("vdd", "neg", "gate", "gnd"),
        wire("input", "pos", "gate", "1a"),
        wire("input", "neg", "vdd", "neg"),
        wire("gate", "1y", "load", "a"),
        wire("load", "b", "vdd", "neg"),
      ],
    };
    const engine = runSteady(circuit, 12, 1e-6);
    const loadCurrent = engine.getElementI().load ?? Number.NaN;
    const railCurrent = engine.getElementI().vdd ?? Number.NaN;
    expect(Math.abs(-railCurrent - loadCurrent)).toBeLessThanOrEqual(10e-9);

    const powerResidual = 5 * railCurrent
      + loadCurrent * loadCurrent * (loadR + outputR);
    expect(Math.abs(powerResidual)).toBeLessThanOrEqual(50e-9);
  });

  /**
   * Validity envelope: a 5 V NE555 held HIGH by TRIG=0 drives 1 kohm. The
   * compact output is a 20 ohm two-rail Thevenin stage with 0.1 V no-load
   * headroom; its complementary conductances also approximate bipolar idle
   * draw. Acceptance: source current equals both internal rail paths, the
   * public output current equals the external load, and signed power balances.
   */
  it("NE555 output and compact idle current are withdrawn from its package rails", () => {
    const loadR = 1_000;
    const outputR = 20;
    const highFraction = 1 - 0.1 / 5;
    const highPathR = outputR / highFraction;
    const lowPathR = outputR / (1 - highFraction);
    const circuit: SimCircuit = {
      components: [
        voltageSource("vdd", 5),
        resistor("load", loadR),
        {
          id: "timer",
          kind: "ne555",
          pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((id) => ({ id })),
          params: {},
        },
      ],
      wires: [
        wire("vdd", "pos", "timer", "8"),
        wire("vdd", "neg", "timer", "1"),
        wire("timer", "2", "vdd", "neg"),
        wire("timer", "4", "vdd", "pos"),
        wire("timer", "3", "load", "a"),
        wire("load", "b", "vdd", "neg"),
      ],
    };
    const engine = runSteady(circuit, 8, 1e-6);
    const outputVoltage = componentVoltage(engine, "load", "a", "b");
    const loadCurrent = engine.getElementI().load ?? Number.NaN;
    const timerOutputCurrent = engine.getElementI().timer ?? Number.NaN;
    const supplyCurrent = engine.getElementI().vdd ?? Number.NaN;
    const highRailCurrent = (5 - outputVoltage) / highPathR;
    const lowRailCurrent = outputVoltage / lowPathR;

    expect(Math.abs(-supplyCurrent - highRailCurrent)).toBeLessThanOrEqual(20e-9);
    expect(Math.abs(timerOutputCurrent - loadCurrent)).toBeLessThanOrEqual(20e-9);
    expect(Math.abs(highRailCurrent - lowRailCurrent - loadCurrent)).toBeLessThanOrEqual(20e-9);

    const signedPowerResidual = 5 * supplyCurrent
      + loadCurrent * loadCurrent * loadR
      + highRailCurrent * highRailCurrent * highPathR
      + lowRailCurrent * lowRailCurrent * lowPathR;
    expect(Math.abs(signedPowerResidual)).toBeLessThanOrEqual(50e-9);
  });

  /**
   * Validity envelope: a powered micro:bit board drives P0 HIGH into 1 kohm.
   * The board model declares a 15 mA nominal idle load plus its 50 ohm GPIO
   * stage. Firmware-dependent peripheral draw is excluded. Acceptance: USB
   * branch current equals idle plus GPIO current within 10 nA.
   */
  it("MCU USB current includes board idle draw and GPIO load", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "board",
          kind: "microbit",
          pins: [{ id: "p0" }, { id: "3v" }, { id: "gnd" }],
          params: { power: 1, vcc: 3.3 },
        },
        resistor("load", 1_000),
      ],
      wires: [
        wire("board", "p0", "load", "a"),
        wire("load", "b", "board", "gnd"),
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    engine.setMicrobitDrive("board", { p0: { mode: "digital", value: 1 } });
    for (let index = 0; index < 8; index++) acceptedStep(engine, 1e-5, "micro:bit rail power");

    const loadCurrent = engine.getElementI().load ?? Number.NaN;
    const usbCurrent = engine.getElementI().board ?? Number.NaN;
    expect(Math.abs(-usbCurrent - (0.015 + loadCurrent))).toBeLessThanOrEqual(10e-9);
  });

  /**
   * Validity envelope: one LM358 voltage follower sources a 1 kohm load from a
   * 5 V single supply. The macro-model includes 0.7 mA whole-package typical
   * quiescent current and a finite output stage; input bias and frequency effects
   * are excluded. Acceptance: supply debit equals IQ plus delivered load current
   * within 20 nA.
   */
  it("op-amp output and quiescent current are withdrawn from its supply rail", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("vdd", 5),
        voltageSource("vin", 2),
        resistor("load", 1_000),
        {
          id: "amp",
          kind: "lm358",
          pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((id) => ({ id })),
          params: {},
        },
      ],
      wires: [
        wire("vdd", "pos", "amp", "8"),
        wire("vdd", "neg", "amp", "4"),
        wire("vin", "pos", "amp", "3"),
        wire("vin", "neg", "vdd", "neg"),
        wire("amp", "1", "amp", "2"),
        wire("amp", "1", "load", "a"),
        wire("load", "b", "vdd", "neg"),
      ],
    };
    const engine = runSteady(circuit, 20, 1e-5);
    const loadCurrent = engine.getElementI().load ?? Number.NaN;
    const supplyCurrent = engine.getElementI().vdd ?? Number.NaN;
    expect(Math.abs(-supplyCurrent - (0.0007 + loadCurrent))).toBeLessThanOrEqual(20e-9);
  });
});
