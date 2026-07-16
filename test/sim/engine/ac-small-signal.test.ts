/**
 * Analytic Bode fixtures — small-signal AC analysis (Wave A5)
 * ===========================================================
 *
 * Golden-reference tests for runSmallSignalAc (ac-analysis.ts). Every
 * assertion compares the solved complex transfer function against a closed
 * form derived from the SAME lumped model the engine documents (the exact
 * conjugate-expansion admittances the acStamp hooks write), with an explicit
 * error budget per fixture. A model change that leaves a fixture's declared
 * envelope must add a new reference derivation rather than widening a
 * tolerance until the test passes — the same contract as
 * physics-reference-benchmarks.test.ts.
 *
 * Error-budget notes shared by the tight (1e-9 relative) fixtures:
 * - the analysis driver stamps the documented universal 1 Tohm node shunt
 *   (NODE_RSHUNT_G = 1e-12 S) on every node. Source resistances are chosen
 *   at 100 ohm so the shunt perturbs the references by ~1e-10 relative,
 *   an order of magnitude inside the budget instead of consuming it;
 * - LU rounding on these well-conditioned systems is ~1e-15 relative;
 * - reference arithmetic here is independent double-precision complex math,
 *   which agrees with the stamped conjugate expansions to ~1e-15.
 */

import { afterEach, describe, expect, it } from "vitest";
import { runSmallSignalAc } from "../../../src/sim/engine/ac-analysis.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { setLinearSystemBackendForTests } from "../../../src/sim/engine/linear-system.js";

const K_B_OVER_Q_V_PER_K = 8.617_333_262_145e-5;

type SimComponent = SimCircuit["components"][number];
type SimWire = SimCircuit["wires"][number];

// ─── Circuit literal helpers (physics-reference-benchmarks conventions) ─────

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

function capacitor(
  id: string,
  capacitance: number,
  losses?: { esr?: number; leakageResistance?: number },
): SimComponent {
  return {
    id,
    kind: "capacitor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { capacitance, ...losses },
  };
}

function inductor(
  id: string,
  inductance: number,
  losses?: { dcr?: number; coreLossResistance?: number },
): SimComponent {
  return {
    id,
    kind: "inductor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { inductance, ...losses },
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

function loadEngine(circuit: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  return engine;
}

/** Deterministic net id carrying a given component pin (invariant 3). */
function netIdFor(engine: SimEngine, componentId: string, pinId: string): string {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
  );
  if (!net) throw new Error(`Fixture topology error: no net for ${componentId}.${pinId}`);
  return net.id;
}

interface AcPoint {
  re: number;
  im: number;
  mag: number;
  phaseDeg: number;
}

/** One-net sweep through the analysis driver, unpacked per point. */
function sweepAc(
  engine: SimEngine,
  inputId: string,
  outputNetId: string,
  frequenciesHz: readonly number[],
): AcPoint[] {
  const result = runSmallSignalAc(engine, {
    inputId,
    outputNetIds: [outputNetId],
    frequenciesHz,
  });
  const out = result.outputs[0]!;
  return frequenciesHz.map((_, index) => ({
    re: out.re[index]!,
    im: out.im[index]!,
    mag: Math.hypot(out.re[index]!, out.im[index]!),
    phaseDeg: out.phaseDeg[index]!,
  }));
}

function acPointAt(
  engine: SimEngine,
  inputId: string,
  outputNetId: string,
  frequencyHz: number,
): AcPoint {
  return sweepAc(engine, inputId, outputNetId, [frequencyHz])[0]!;
}

function logSpace(startHz: number, stopHz: number, count: number): number[] {
  const a = Math.log10(startHz);
  const b = Math.log10(stopHz);
  const points: number[] = [];
  for (let index = 0; index < count; index++) {
    points.push(Math.pow(10, a + ((b - a) * index) / (count - 1)));
  }
  return points;
}

/**
 * Geometric bisection for a monotone frequency-domain crossing. The
 * predicate must flip exactly once between the endpoints (asserted), so a
 * broken sweep fails loudly instead of converging onto a bracket edge.
 */
function bisectCrossing(
  lo: number,
  hi: number,
  isPastCrossing: (frequencyHz: number) => boolean,
  iterations = 40,
): number {
  expect(isPastCrossing(lo), `bisection bracket: predicate already true at ${String(lo)} Hz`).toBe(false);
  expect(isPastCrossing(hi), `bisection bracket: predicate still false at ${String(hi)} Hz`).toBe(true);
  let below = lo;
  let above = hi;
  for (let index = 0; index < iterations; index++) {
    const mid = Math.sqrt(below * above);
    if (isPastCrossing(mid)) above = mid;
    else below = mid;
  }
  return Math.sqrt(below * above);
}

// ─── Reference complex arithmetic ────────────────────────────────────────────

interface Cx {
  re: number;
  im: number;
}

function cAdd(a: Cx, b: Cx): Cx {
  return { re: a.re + b.re, im: a.im + b.im };
}

function cMul(a: Cx, b: Cx): Cx {
  return { re: a.re * b.re - a.im * b.im, im: a.re * b.im + a.im * b.re };
}

function cInv(z: Cx): Cx {
  const den = z.re * z.re + z.im * z.im;
  return { re: z.re / den, im: -z.im / den };
}

function cDiv(a: Cx, b: Cx): Cx {
  return cMul(a, cInv(b));
}

function cMag(z: Cx): number {
  return Math.hypot(z.re, z.im);
}

function cPhaseDeg(z: Cx): number {
  return (Math.atan2(z.im, z.re) * 180) / Math.PI;
}

function expectRelativeError(
  actual: number,
  expected: number,
  maxRelative: number,
  label: string,
): void {
  expect(Math.abs(expected), `${label}: reference must be nonzero`).toBeGreaterThan(0);
  expect(
    Math.abs(actual - expected) / Math.abs(expected),
    `${label}: actual=${String(actual)}, reference=${String(expected)}, budget=${String(maxRelative)}`,
  ).toBeLessThanOrEqual(maxRelative);
}

function expectComplexRelativeError(
  actual: Cx,
  expected: Cx,
  maxRelative: number,
  label: string,
): void {
  // One complex-distance bound covers magnitude AND phase together, and
  // stays meaningful where the phase passes through zero (a relative bound
  // on a near-zero phase would be vacuous or impossible there).
  expect(
    Math.hypot(actual.re - expected.re, actual.im - expected.im) / cMag(expected),
    `${label}: actual=${String(actual.re)}+j${String(actual.im)}, `
    + `reference=${String(expected.re)}+j${String(expected.im)}, budget=${String(maxRelative)}`,
  ).toBeLessThanOrEqual(maxRelative);
}

// ─── First-order RC/CR references ────────────────────────────────────────────

describe("small-signal AC — RC lowpass and CR highpass against closed forms", () => {
  // 100 ohm / 1.5915 uF puts the corner at exactly 1 kHz while keeping the
  // node-shunt perturbation at ~1e-10 relative (see file header).
  const R = 100;
  const C = 1 / (2 * Math.PI * 1000 * R);
  const frequencies = logSpace(10, 100_000, 20);

  /**
   * Validity envelope: ideal lumped R and C (no ESR/leakage declared), unit
   * drive from an ideal source. Reference H = 1/(1 + j*omega*RC).
   * Acceptance: magnitude and phase within 1e-9 relative at every point.
   */
  it("RC lowpass magnitude and phase match 1/(1 + jwRC) within 1e-9 relative", () => {
    const circuit: SimCircuit = {
      components: [voltageSource("vin", 0), resistor("r", R), capacitor("c", C)],
      wires: [
        wire("vin", "pos", "r", "a"),
        wire("r", "b", "c", "a"),
        wire("c", "b", "vin", "neg"),
      ],
    };
    const engine = loadEngine(circuit);
    const points = sweepAc(engine, "vin", netIdFor(engine, "c", "a"), frequencies);

    for (let index = 0; index < frequencies.length; index++) {
      const omega = 2 * Math.PI * frequencies[index]!;
      const reference = cInv({ re: 1, im: omega * R * C });
      const label = `RC lowpass at ${String(frequencies[index])} Hz`;
      expectRelativeError(points[index]!.mag, cMag(reference), 1e-9, `${label} magnitude`);
      expectRelativeError(points[index]!.phaseDeg, cPhaseDeg(reference), 1e-9, `${label} phase`);
    }
  });

  /**
   * Validity envelope: same parts swapped into the highpass position.
   * Reference H = j*omega*RC / (1 + j*omega*RC).
   * Acceptance: magnitude and phase within 1e-9 relative at every point.
   */
  it("CR highpass magnitude and phase match jwRC/(1 + jwRC) within 1e-9 relative", () => {
    const circuit: SimCircuit = {
      components: [voltageSource("vin", 0), capacitor("c", C), resistor("r", R)],
      wires: [
        wire("vin", "pos", "c", "a"),
        wire("c", "b", "r", "a"),
        wire("r", "b", "vin", "neg"),
      ],
    };
    const engine = loadEngine(circuit);
    const points = sweepAc(engine, "vin", netIdFor(engine, "c", "b"), frequencies);

    for (let index = 0; index < frequencies.length; index++) {
      const omega = 2 * Math.PI * frequencies[index]!;
      const jwrc: Cx = { re: 0, im: omega * R * C };
      const reference = cDiv(jwrc, cAdd({ re: 1, im: 0 }, jwrc));
      const label = `CR highpass at ${String(frequencies[index])} Hz`;
      expectRelativeError(points[index]!.mag, cMag(reference), 1e-9, `${label} magnitude`);
      expectRelativeError(points[index]!.phaseDeg, cPhaseDeg(reference), 1e-9, `${label} phase`);
    }
  });
});

// ─── Series RLC bandpass ─────────────────────────────────────────────────────

describe("small-signal AC — series RLC bandpass resonance", () => {
  /**
   * Validity envelope: ideal series L and C (no declared losses) into a
   * 10 ohm view resistor; H = R/(R + j(wL - 1/(wC))). Analytic anchors:
   * f0 = 1/(2*pi*sqrt(LC)), Q = sqrt(L/C)/R, half-power bandwidth f0/Q
   * (exact for this topology, not the narrowband approximation), phase
   * crossing zero exactly at f0. Acceptance: measured f0 (via the phase
   * zero-crossing) and Q (via the measured -3 dB bandwidth) within 0.1%;
   * |H(f0)| = 1 within 1e-6 (node shunts and LU rounding only).
   */
  it("recovers f0, Q, and the phase zero-crossing within 0.1% of analytic", () => {
    const L = 10e-3;
    const C = 1e-6;
    const R = 10;
    const f0 = 1 / (2 * Math.PI * Math.sqrt(L * C));
    const q = Math.sqrt(L / C) / R;

    const circuit: SimCircuit = {
      components: [voltageSource("vin", 0), inductor("l", L), capacitor("c", C), resistor("r", R)],
      wires: [
        wire("vin", "pos", "l", "a"),
        wire("l", "b", "c", "a"),
        wire("c", "b", "r", "a"),
        wire("r", "b", "vin", "neg"),
      ],
    };
    const engine = loadEngine(circuit);
    const outputNet = netIdFor(engine, "r", "a");
    const magAt = (f: number): number => acPointAt(engine, "vin", outputNet, f).mag;

    // Phase is positive below resonance (capacitive net reactance) and
    // negative above, so "phase < 0" brackets the zero-crossing.
    const f0Measured = bisectCrossing(
      0.9 * f0,
      1.1 * f0,
      (f) => acPointAt(engine, "vin", outputNet, f).phaseDeg < 0,
    );
    expectRelativeError(f0Measured, f0, 1e-3, "RLC phase zero-crossing frequency");

    expectRelativeError(magAt(f0), 1, 1e-6, "RLC |H| at analytic f0");

    const halfPower = Math.SQRT1_2;
    const f1Measured = bisectCrossing(0.5 * f0, f0Measured, (f) => magAt(f) > halfPower);
    const f2Measured = bisectCrossing(f0Measured, 2 * f0, (f) => magAt(f) < halfPower);
    const qMeasured = f0Measured / (f2Measured - f1Measured);
    expectRelativeError(qMeasured, q, 1e-3, "RLC Q from measured -3 dB bandwidth");
  });
});

// ─── Lossy passive branches ──────────────────────────────────────────────────

describe("small-signal AC — lossy capacitor and inductor asymptotes", () => {
  /**
   * Validity envelope: 100 ohm source resistance into one capacitor with
   * declared ESR (series) and leakage (parallel), measured at the shared
   * node. Reference is the exact lossy admittance divider
   * H = Gs / (Gs + 1/(esr + 1/(jwC)) + 1/Rleak).
   * Acceptance: the full sweep within 1e-9 complex-relative; the measured
   * magnitude within 0.1% of the low-frequency asymptote Rleak/(Rs+Rleak)
   * at leak-corner/100 and of the high-frequency asymptote
   * Gs/(Gs + 1/esr + 1/Rleak) at esr-corner*100 (residual reactance at
   * those decades contributes well under the budget).
   */
  it("capacitor ESR + leakage matches the lossy closed form and both asymptotes", () => {
    const rs = 100;
    const esr = 10;
    const rLeak = 100_000;
    const C = 1e-6;
    const gs = 1 / rs;
    const circuit: SimCircuit = {
      components: [
        voltageSource("vin", 0),
        resistor("rsrc", rs),
        capacitor("c", C, { esr, leakageResistance: rLeak }),
      ],
      wires: [
        wire("vin", "pos", "rsrc", "a"),
        wire("rsrc", "b", "c", "a"),
        wire("c", "b", "vin", "neg"),
      ],
    };
    const engine = loadEngine(circuit);
    const outputNet = netIdFor(engine, "c", "a");

    const reference = (f: number): Cx => {
      const omega = 2 * Math.PI * f;
      const ySeries = cInv({ re: esr, im: -1 / (omega * C) });
      const yTotal = cAdd(ySeries, { re: 1 / rLeak, im: 0 });
      return cDiv({ re: gs, im: 0 }, cAdd({ re: gs, im: 0 }, yTotal));
    };

    const frequencies = logSpace(0.1, 10_000_000, 20);
    const points = sweepAc(engine, "vin", outputNet, frequencies);
    for (let index = 0; index < frequencies.length; index++) {
      expectComplexRelativeError(
        points[index]!,
        reference(frequencies[index]!),
        1e-9,
        `lossy capacitor at ${String(frequencies[index])} Hz`,
      );
    }

    const leakCorner = 1 / (2 * Math.PI * C * rLeak);
    const esrCorner = 1 / (2 * Math.PI * C * esr);
    const lowAsymptote = rLeak / (rs + rLeak);
    const highAsymptote = gs / (gs + 1 / esr + 1 / rLeak);
    expectRelativeError(
      acPointAt(engine, "vin", outputNet, leakCorner / 100).mag,
      lowAsymptote,
      1e-3,
      "lossy capacitor low-frequency asymptote (leakage divider)",
    );
    expectRelativeError(
      acPointAt(engine, "vin", outputNet, esrCorner * 100).mag,
      highAsymptote,
      1e-3,
      "lossy capacitor high-frequency asymptote (ESR divider)",
    );
  });

  /**
   * Validity envelope: 100 ohm source resistance into one inductor with
   * declared DCR (series) and core loss (parallel), measured at the shared
   * node. Reference H = Gs / (Gs + 1/(dcr + jwL) + 1/Rcore).
   * Acceptance: full sweep within 1e-9 complex-relative; magnitude within
   * 0.1% of the low-frequency asymptote Gs/(Gs + 1/dcr + 1/Rcore) at
   * dcr-corner/100 and of the high-frequency asymptote Rcore/(Rs+Rcore) at
   * core-corner*100.
   */
  it("inductor DCR + core loss matches the lossy closed form and both asymptotes", () => {
    const rs = 100;
    const dcr = 5;
    const rCore = 50_000;
    const L = 10e-3;
    const gs = 1 / rs;
    const circuit: SimCircuit = {
      components: [
        voltageSource("vin", 0),
        resistor("rsrc", rs),
        inductor("l", L, { dcr, coreLossResistance: rCore }),
      ],
      wires: [
        wire("vin", "pos", "rsrc", "a"),
        wire("rsrc", "b", "l", "a"),
        wire("l", "b", "vin", "neg"),
      ],
    };
    const engine = loadEngine(circuit);
    const outputNet = netIdFor(engine, "l", "a");

    const reference = (f: number): Cx => {
      const omega = 2 * Math.PI * f;
      const yWinding = cInv({ re: dcr, im: omega * L });
      const yTotal = cAdd(yWinding, { re: 1 / rCore, im: 0 });
      return cDiv({ re: gs, im: 0 }, cAdd({ re: gs, im: 0 }, yTotal));
    };

    const frequencies = logSpace(0.1, 10_000_000, 20);
    const points = sweepAc(engine, "vin", outputNet, frequencies);
    for (let index = 0; index < frequencies.length; index++) {
      expectComplexRelativeError(
        points[index]!,
        reference(frequencies[index]!),
        1e-9,
        `lossy inductor at ${String(frequencies[index])} Hz`,
      );
    }

    const dcrCorner = dcr / (2 * Math.PI * L);
    const coreCorner = rCore / (2 * Math.PI * L);
    const lowAsymptote = gs / (gs + 1 / dcr + 1 / rCore);
    const highAsymptote = rCore / (rs + rCore);
    expectRelativeError(
      acPointAt(engine, "vin", outputNet, dcrCorner / 100).mag,
      lowAsymptote,
      1e-3,
      "lossy inductor low-frequency asymptote (DCR divider)",
    );
    expectRelativeError(
      acPointAt(engine, "vin", outputNet, coreCorner * 100).mag,
      highAsymptote,
      1e-3,
      "lossy inductor high-frequency asymptote (core-loss divider)",
    );
  });
});

// ─── Transistor amplifiers linearized at the committed OP ────────────────────

describe("small-signal AC — BJT common-emitter gain from the engine's own OP", () => {
  /**
   * Validity envelope: forced-VBE NPN common-emitter stage at 25 C
   * (Is=1e-16, betaF=100, nF=nR=1, VA=100 V), base driven by an ideal
   * source so r_pi does not load the input. The reference is built FROM THE
   * OPERATING POINT THE ENGINE ITSELF REPORTS after the analysis commits
   * it: Ic from getElementI(), VCE from getNetV(), then gm = Ic/Vt and
   * ro = (VA + VCE)/Ic — the small-signal pair implied by the model's
   * documented Early scaling (Ic = If*(1 + VCE/VA) gives dIc/dVCE =
   * Ic/(VA + VCE) and dIc/dVBE = Ic/Vt at that same committed point).
   * Acceptance: |H| within 2% of gm*(Rc || ro) and phase inverted within
   * 1e-6 degrees (the model carries no junction capacitance).
   */
  it("matches -gm*(Rc || ro) computed from the committed operating point within 2%", () => {
    const rc = 5_000;
    const earlyVoltage = 100;
    const circuit: SimCircuit = {
      components: [
        voltageSource("vcc", 10),
        voltageSource("vbb", 0.77),
        resistor("rc", rc),
        {
          id: "q",
          kind: "bjt_npn",
          pins: [{ id: "c" }, { id: "b" }, { id: "e" }],
          params: { Is: 1e-16, betaF: 100, betaR: 1, nF: 1, nR: 1, earlyVoltage },
        },
      ],
      wires: [
        wire("vcc", "pos", "rc", "a"),
        wire("rc", "b", "q", "c"),
        wire("vbb", "pos", "q", "b"),
        wire("q", "e", "vcc", "neg"),
        wire("vbb", "neg", "vcc", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    const engine = loadEngine(circuit);
    const collectorNet = netIdFor(engine, "q", "c");
    const point = acPointAt(engine, "vbb", collectorNet, 1_000);

    // Committed OP readouts (runSmallSignalAc ran dcOperatingPoint above).
    const ic = engine.getElementI().q ?? Number.NaN;
    const vce = engine.getNetV()[collectorNet] ?? Number.NaN;
    expect(ic).toBeGreaterThan(1e-4);
    expect(vce).toBeGreaterThan(1);

    const vt = K_B_OVER_Q_V_PER_K * (25 + 273.15);
    const gm = ic / vt;
    const ro = (earlyVoltage + vce) / ic;
    const expectedGain = gm * ((rc * ro) / (rc + ro));

    expectRelativeError(point.mag, expectedGain, 0.02, "BJT common-emitter |H|");
    expect(
      Math.abs(Math.abs(point.phaseDeg) - 180),
      `BJT common-emitter phase: ${String(point.phaseDeg)} deg`,
    ).toBeLessThanOrEqual(1e-6);
  });
});

describe("small-signal AC — MOSFET common-source Miller pole", () => {
  /**
   * Validity envelope: level-1 NMOS (VTO=1, K=0.01, lambda=0) biased into
   * saturation, gate fed through Rg=10k, Cgd=1nF declared, Cgs left 0 so
   * the exact network is first-order. With one capacitor the transfer is
   * exactly one pole + one (RHP) zero; the pole is the exact Miller pole
   *   f_p = 1 / (2*pi*Cgd*(Rg*(1 + gm*RL) + RL)),  RL = Rd,
   * and the zero gm/(2*pi*Cgd) sits ~3.3 decades higher, so the -3 dB
   * crossing IS the pole to well under the budget. gm = 2K*(VGS-VTO) is
   * computed from the gate/source voltages the engine reports at the
   * committed OP. Acceptance: measured -3 dB frequency within 5% of f_p,
   * and at least 3x below the non-Miller estimate 1/(2*pi*Cgd*(Rg+RL))
   * (the "pole visibly moved by Miller multiplication" check).
   */
  it("places the -3 dB point within 5% of the analytic Miller pole", () => {
    const rg = 10_000;
    const rd = 500;
    const cgd = 1e-9;
    const vto = 1;
    const k = 0.01;
    const circuit: SimCircuit = {
      components: [
        voltageSource("vdd", 8),
        voltageSource("vin", 2),
        resistor("rg", rg),
        resistor("rd", rd),
        {
          id: "m",
          kind: "nmos",
          pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
          params: { vto, k, lambda: 0, cgd },
        },
      ],
      wires: [
        wire("vin", "pos", "rg", "a"),
        wire("rg", "b", "m", "g"),
        wire("vdd", "pos", "rd", "a"),
        wire("rd", "b", "m", "d"),
        wire("m", "s", "vdd", "neg"),
        wire("vin", "neg", "vdd", "neg"),
      ],
    };
    const engine = loadEngine(circuit);
    const drainNet = netIdFor(engine, "m", "d");
    const gateNet = netIdFor(engine, "m", "g");

    // Reference magnitude far below the pole; also commits the OP the gm
    // readout below depends on.
    const low = acPointAt(engine, "vin", drainNet, 1);
    const vgs = engine.getNetV()[gateNet] ?? Number.NaN;
    const gm = 2 * k * (vgs - vto);
    expect(gm).toBeGreaterThan(0);

    const millerPole = 1 / (2 * Math.PI * cgd * (rg * (1 + gm * rd) + rd));
    const noMillerEstimate = 1 / (2 * Math.PI * cgd * (rg + rd));
    const target = low.mag * Math.SQRT1_2;
    const measuredPole = bisectCrossing(
      millerPole / 30,
      millerPole * 30,
      (f) => acPointAt(engine, "vin", drainNet, f).mag < target,
    );

    expectRelativeError(measuredPole, millerPole, 0.05, "MOSFET Miller pole frequency");
    expect(
      measuredPole,
      "Miller multiplication must move the pole visibly below the Rg+Rd estimate",
    ).toBeLessThan(noMillerEstimate / 3);
  });
});

// ─── Op-amp macro-model regimes ──────────────────────────────────────────────

describe("small-signal AC — op-amp inverting amplifier", () => {
  /**
   * Validity envelope: LM358 unit 1 as an inverting amp (R1=1k, R2=10k) on
   * split 5 V rails, biased at 0 V so the committed regime is linear. The
   * macro-model is a dominant-pole VCVS (A0=1e5, GBW=1 MHz declared in
   * params so the fixture is self-describing), giving the ideal closed loop
   *   H(f) = -(R2/R1) / (1 + j*f/fc),  fc = GBW / (1 + R2/R1).
   * Finite A0 (noise-gain/A0 ~ 1e-4) and the 50 ohm output stage inside
   * the loop (~Rout/R2 = 0.5% near crossover) are the only deviations.
   * Acceptance: |H| within 5% of the ideal single-pole form across
   * 10 Hz..1 MHz, low-frequency gain within 1% of R2/R1, phase inverted at
   * the lowest point.
   */
  it("holds -R2/R1 flat and rolls off at GBW/(noise gain) within 5%", () => {
    const r1 = 1_000;
    const r2 = 10_000;
    const a0 = 100_000;
    const gbwHz = 1_000_000;
    const noiseGain = 1 + r2 / r1;
    const fc = gbwHz / noiseGain;
    const circuit: SimCircuit = {
      components: [
        voltageSource("vcc", 5),
        voltageSource("vee", 5),
        voltageSource("vin", 0),
        resistor("r1", r1),
        resistor("r2", r2),
        {
          id: "amp",
          kind: "lm358",
          pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((id) => ({ id })),
          params: { openLoopGain: a0, gbw: gbwHz },
        },
      ],
      wires: [
        wire("vcc", "pos", "amp", "8"),
        // vee pos joins the gnd reference; its neg is the -5 V rail.
        wire("vee", "pos", "vcc", "neg"),
        wire("vee", "neg", "amp", "4"),
        wire("vin", "pos", "r1", "a"),
        wire("vin", "neg", "vcc", "neg"),
        wire("r1", "b", "amp", "2"),
        wire("r2", "a", "amp", "2"),
        wire("r2", "b", "amp", "1"),
        wire("amp", "3", "vcc", "neg"),
      ],
    };
    const engine = loadEngine(circuit);
    const outputNet = netIdFor(engine, "amp", "1");
    const frequencies = logSpace(10, 1_000_000, 20);
    const points = sweepAc(engine, "vin", outputNet, frequencies);

    for (let index = 0; index < frequencies.length; index++) {
      const idealMag = (r2 / r1) / Math.hypot(1, frequencies[index]! / fc);
      expectRelativeError(
        points[index]!.mag,
        idealMag,
        0.05,
        `inverting amp |H| at ${String(frequencies[index])} Hz`,
      );
    }

    expectRelativeError(points[0]!.mag, r2 / r1, 0.01, "inverting amp low-frequency gain");
    expect(
      Math.abs(points[0]!.phaseDeg),
      `inverting amp low-frequency phase: ${String(points[0]!.phaseDeg)} deg`,
    ).toBeGreaterThan(179);
  });

  /**
   * Validity envelope: the same macro-model driven open-loop into rail
   * saturation (IN+ = 3 V, IN- = 0 V, single 5 V supply, output committed
   * at the vcc-1.5 V ceiling into a 10k load). Documented
   * linearize-at-region semantics (acStampOpAmpUnit): a rail-saturated
   * committed regime keeps the transient constraint's derivative — output
   * pinned to the rail through Rout, inputs uncoupled — so the small-signal
   * gain FROM THE INPUTS is exactly zero, matching SPICE's linearization at
   * the saturated OP (the rail itself couples with derivative 1; see the
   * rail-tracking companion test below). The budget is 1e-12 (solver
   * rounding on a decoupled RHS block), not a loose "small gain" bound.
   */
  function railSaturatedCircuit(): SimCircuit {
    return {
      components: [
        voltageSource("vcc", 5),
        voltageSource("vin", 3),
        resistor("rload", 10_000),
        {
          id: "amp",
          kind: "lm358",
          pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((id) => ({ id })),
          params: {},
        },
      ],
      wires: [
        wire("vcc", "pos", "amp", "8"),
        wire("vcc", "neg", "amp", "4"),
        wire("vin", "pos", "amp", "3"),
        wire("vin", "neg", "vcc", "neg"),
        wire("amp", "2", "vcc", "neg"),
        wire("amp", "1", "rload", "a"),
        wire("rload", "b", "vcc", "neg"),
      ],
    };
  }

  /** OP precondition shared by the two saturated-regime tests: LM358
   *  ceiling is vcc - 1.5 = 3.5 V through the 50 ohm stage into 10k. */
  function expectSaturatedOp(engine: SimEngine, outputNet: string): void {
    const vout = engine.getNetV()[outputNet] ?? Number.NaN;
    expect(vout).toBeGreaterThan(3.4);
    expect(vout).toBeLessThan(3.5);
  }

  it("reports ~zero AC gain when the committed OP is rail-saturated", () => {
    const engine = loadEngine(railSaturatedCircuit());
    const outputNet = netIdFor(engine, "amp", "1");
    const points = sweepAc(engine, "vin", outputNet, [100, 10_000, 1_000_000]);

    expectSaturatedOp(engine, outputNet);

    for (const point of points) {
      expect(
        point.mag,
        `saturated op-amp must have ~zero small-signal gain, got ${String(point.mag)}`,
      ).toBeLessThanOrEqual(1e-12);
    }
  });

  /**
   * Validity envelope: identical circuit and committed OP, but the DESIGNATED
   * AC input is the supply rail. The transient saturated row is
   * Vout - Rout*x[k] = vcc_node - 1.5: the headroom is constant but the rail
   * target is a live node voltage re-read every Newton iterate, so the true
   * derivative passes rail ripple through the pinned stage at
   *   |H| = RL / (RL + Rout) = 10000/10050,
   * not zero (review finding F4: dropping the rail column reported exactly 0).
   * Budget 1e-6: the reference ignores only the 1e-12 S node shunt (~1e-10
   * relative) and LU rounding.
   */
  it("passes rail ripple through the rail-saturated stage at RL/(RL+Rout)", () => {
    const engine = loadEngine(railSaturatedCircuit());
    const outputNet = netIdFor(engine, "amp", "1");
    const points = sweepAc(engine, "vcc", outputNet, [100, 10_000]);

    expectSaturatedOp(engine, outputNet);

    const reference = 10_000 / (10_000 + 50);
    for (const point of points) {
      expectRelativeError(
        point.mag,
        reference,
        1e-6,
        "rail ripple through saturated output stage",
      );
    }
  });
});

// ─── Committed digital output stages load the AC system (review wave) ────────

describe("small-signal AC — committed output stages and open-collector release", () => {
  /**
   * Validity envelope: NE555 powered from an AC-zeroed 5 V source, OUT
   * (pin 3) probed through 1k from the designated input. The committed
   * rail-referenced push-pull stage presents complementary conductances
   * summing to 1/20 ohm toward AC-grounded rails regardless of the committed
   * SR level, so the probe sees the divider
   *   |H| = 20 / (1000 + 20).
   * Review finding F1: without the ne555 acStamp hook the node read back
   * unloaded (|H| = 1, a ~34 dB error). Budget 1e-6 (node shunt ~1e-9
   * relative plus LU rounding).
   */
  it("ne555 committed OUT stage loads a probed net with its 20 ohm Thevenin", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("vcc", 5),
        voltageSource("vin", 1),
        resistor("rprobe", 1_000),
        {
          id: "timer",
          kind: "ne555",
          pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((id) => ({ id })),
          params: {},
        },
      ],
      wires: [
        wire("vcc", "pos", "timer", "8"),
        wire("vcc", "neg", "timer", "1"),
        wire("timer", "4", "vcc", "pos"),
        wire("timer", "2", "vcc", "pos"),
        wire("timer", "6", "vcc", "pos"),
        wire("vin", "pos", "rprobe", "a"),
        wire("rprobe", "b", "timer", "3"),
        wire("vin", "neg", "vcc", "neg"),
      ],
    };
    const engine = loadEngine(circuit);
    const outputNet = netIdFor(engine, "timer", "3");
    const point = acPointAt(engine, "vin", outputNet, 0.1);
    expectRelativeError(point.mag, 20 / 1_020, 1e-6, "ne555 OUT stage divider");
  });

  /**
   * Validity envelope: 74LS47 (catalog open-collector outputs) decoding
   * BCD 0001, so segment a is committed HIGH = released transistor. The
   * transient stamp writes NOTHING for a released OC pin — an external
   * pull determines the net — so probing a_out through 1k must read
   * |H| = 1. Review finding F2: the generic delay-slot default used to
   * stamp a phantom Thevenin pull to VCC for committed-HIGH OC pins
   * (|H| = 0.0495). Budget 1e-6 (node shunt ~1e-9 relative).
   */
  it("a released open-collector output stays hi-Z in the AC system", () => {
    const pins = [
      "b", "c", "lt_n", "rbo_n", "rbi_n", "d", "a", "gnd",
      "e_out", "d_out", "c_out", "b_out", "a_out", "g_out", "f_out", "vcc",
    ];
    const circuit: SimCircuit = {
      components: [
        voltageSource("vcc", 5),
        voltageSource("vin", 1),
        resistor("rprobe", 1_000),
        { id: "dec", kind: "74ls47", pins: pins.map((id) => ({ id })), params: {} },
      ],
      wires: [
        wire("vcc", "pos", "dec", "vcc"),
        wire("vcc", "neg", "dec", "gnd"),
        // BCD = 0001 -> digit 1: segments b,c sink LOW; segment a releases.
        wire("dec", "a", "vcc", "pos"),
        wire("dec", "b", "vcc", "neg"),
        wire("dec", "c", "vcc", "neg"),
        wire("dec", "d", "vcc", "neg"),
        wire("vin", "pos", "rprobe", "a"),
        wire("rprobe", "b", "dec", "a_out"),
        wire("vin", "neg", "vcc", "neg"),
      ],
    };
    const engine = loadEngine(circuit);
    const outputNet = netIdFor(engine, "dec", "a_out");
    const point = acPointAt(engine, "vin", outputNet, 0.1);

    // OP precondition: the released pin really follows the 1 V probe source,
    // i.e. the transient system left it hi-Z too.
    const vout = engine.getNetV()[outputNet] ?? Number.NaN;
    expect(vout).toBeGreaterThan(0.99);
    expect(vout).toBeLessThan(1.01);

    expectRelativeError(point.mag, 1, 1e-6, "released open-collector output");
  });
});

// ─── Independent-source AC-zeroing ───────────────────────────────────────────

describe("small-signal AC — non-designated sources are AC-zeroed", () => {
  const frequencies = logSpace(10, 100_000, 20);

  function summingCircuit(biasVolts: number): SimCircuit {
    return {
      components: [
        voltageSource("vin", 0),
        voltageSource("vbias", biasVolts),
        resistor("r1", 100),
        resistor("r2", 200),
      ],
      wires: [
        wire("vin", "pos", "r1", "a"),
        wire("r1", "b", "r2", "a"),
        wire("r2", "b", "vbias", "pos"),
        wire("vbias", "neg", "vin", "neg"),
      ],
    };
  }

  /**
   * Validity envelope: resistive summing node fed by the designated input
   * through R1 and by a second independent source through R2. The second
   * source must linearize as an AC short (its large-signal value is bias,
   * not signal), so H = G1/(G1 + G2) = 2/3 at every frequency, and changing
   * that source's DC value must not move a single output bit: the AC system
   * never reads it (only the OP does, and no stamp here depends on the OP).
   * Acceptance: H within 1e-9 relative of 2/3, and bitwise-identical
   * responses across the DC change.
   */
  it("keeps the output bit-identical when the second source's DC value changes", () => {
    const engineA = loadEngine(summingCircuit(1));
    const outputNet = netIdFor(engineA, "r1", "b");
    const resultA = runSmallSignalAc(engineA, {
      inputId: "vin",
      outputNetIds: [outputNet],
      frequenciesHz: frequencies,
    });

    const engineB = loadEngine(summingCircuit(3));
    const resultB = runSmallSignalAc(engineB, {
      inputId: "vin",
      outputNetIds: [outputNet],
      frequenciesHz: frequencies,
    });

    const outA = resultA.outputs[0]!;
    const outB = resultB.outputs[0]!;
    for (let index = 0; index < frequencies.length; index++) {
      expectRelativeError(
        Math.hypot(outA.re[index]!, outA.im[index]!),
        2 / 3,
        1e-9,
        `summing node |H| at ${String(frequencies[index])} Hz`,
      );
      expect(
        Math.abs(outA.phaseDeg[index]!),
        `summing node phase at ${String(frequencies[index])} Hz`,
      ).toBeLessThanOrEqual(1e-6);
      expect(
        Object.is(outA.re[index], outB.re[index]) && Object.is(outA.im[index], outB.im[index]),
        `AC-zeroing: point ${String(index)} moved when the bias source's DC value changed `
        + `(${String(outA.re[index])}+j${String(outA.im[index])} vs `
        + `${String(outB.re[index])}+j${String(outB.im[index])})`,
      ).toBe(true);
    }
  });
});

// ─── Backend parity and determinism ──────────────────────────────────────────

describe("small-signal AC — backend parity and determinism", () => {
  afterEach(() => {
    setLinearSystemBackendForTests(null);
  });

  const frequencies = logSpace(10, 100_000, 20);

  function rcLowpassCircuit(): SimCircuit {
    const r = 100;
    const c = 1 / (2 * Math.PI * 1000 * r);
    return {
      components: [voltageSource("vin", 0), resistor("r", r), capacitor("c", c)],
      wires: [
        wire("vin", "pos", "r", "a"),
        wire("r", "b", "c", "a"),
        wire("c", "b", "vin", "neg"),
      ],
    };
  }

  function runRcSweep(): { re: number[]; im: number[]; magnitudeDb: number[]; phaseDeg: number[] } {
    const engine = loadEngine(rcLowpassCircuit());
    return runSmallSignalAc(engine, {
      inputId: "vin",
      outputNetIds: [netIdFor(engine, "c", "a")],
      frequenciesHz: frequencies,
    }).outputs[0]!;
  }

  /**
   * Validity envelope: the RC lowpass fixture forced through the dense and
   * sparse linear backends in turn (the forced mode covers both the
   * engine's own OP solves and the bordered 2n AC system, which is built
   * through the same createLinearSystem). The fixture's stamps do not read
   * the OP, so the only cross-backend difference is LU rounding.
   * Acceptance: per-point complex responses agree within 1e-12.
   */
  it("dense and sparse backends agree within 1e-12 per point", () => {
    setLinearSystemBackendForTests("dense");
    const dense = runRcSweep();
    setLinearSystemBackendForTests("sparse");
    const sparse = runRcSweep();

    for (let index = 0; index < frequencies.length; index++) {
      const distance = Math.hypot(
        dense.re[index]! - sparse.re[index]!,
        dense.im[index]! - sparse.im[index]!,
      );
      expect(
        distance,
        `backend parity at ${String(frequencies[index])} Hz: `
        + `dense=${String(dense.re[index])}+j${String(dense.im[index])}, `
        + `sparse=${String(sparse.re[index])}+j${String(sparse.im[index])}`,
      ).toBeLessThanOrEqual(1e-12);
    }
  });

  /**
   * Validity envelope: two independent engines running the identical
   * analysis under the default backend selection. The whole pipeline (OP
   * ladder, restamp loop, LU) is deterministic by design, so the results
   * must be bitwise equal — any drift here means hidden state leaked into
   * the analysis. Acceptance: Object.is equality on every published array
   * entry (magnitude, phase, re, im).
   */
  it("two runs of the same fixture are bitwise identical", () => {
    const first = runRcSweep();
    const second = runRcSweep();

    for (let index = 0; index < frequencies.length; index++) {
      for (const key of ["re", "im", "magnitudeDb", "phaseDeg"] as const) {
        expect(
          Object.is(first[key][index], second[key][index]),
          `determinism: ${key}[${String(index)}] drifted `
          + `(${String(first[key][index])} vs ${String(second[key][index])})`,
        ).toBe(true);
      }
    }
  });
});
