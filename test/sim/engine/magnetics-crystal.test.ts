/**
 * Magnetics + crystal analytic fixtures — Wave A6
 * ===============================================
 *
 * Golden-reference tests for the coupled_inductor (transformer) and crystal
 * device models (packages/simcore/src/sim/engine/devices/model-families.ts)
 * and their elements.ts companions. The fixture discipline follows
 * trap-integration.test.ts and ac-small-signal.test.ts: every assertion
 * compares the engine against a closed form derived from the SAME lumped
 * model the device documents, each test states its validity envelope and an
 * explicit error budget, and a failing budget means an engine bug — never
 * widen a tolerance to make the engine pass.
 *
 * Coverage contracts locked here:
 *   1. Turns ratio: open-secondary v2/v1 = k*sqrt(l2/l1) is an instantaneous
 *      derivative identity (v1 = L1*di1/dt, v2 = M*di1/dt when i2 = 0), so a
 *      transient RMS ratio must reproduce it without any settling caveat.
 *   2. Impedance reflection: with k -> 1 and omega*L1 >> n^2*RL the primary
 *      sees n^2*RL = (l1/l2)*RL — the ideal-transformer limit the 0.9999
 *      coupling clamp is documented to preserve.
 *   3. The acStamp's exact 2x2 admittance inversion against an independent
 *      complex two-loop solution at k = 0.5.
 *   4. Trapezoidal energy conservation through the mutual term (shorted
 *      secondary, so BOTH windings carry current and the ring frequency
 *      itself proves M: f0 = 1/(2*pi*sqrt(L1*(1-k^2)*C))).
 *   5. Composite-key state (id:1/id:2, id:cs/id:ls/id:c0) round-trips
 *      snapshots bitwise and replays bitwise — the A2 rollback contract.
 *   6. dcOperatingPoint treats each winding as a dc short through its own
 *      resistance with zero mutual contribution.
 *   7. Crystal AC series/parallel resonance placement from the datasheet
 *      triple (fSeries, q, rs) plus c0, and trap-mode ringdown loss bounded
 *      by the physical pi/q per cycle (asserted against 2*pi/q * 1.5).
 *   8. Internal solver nodes (the crystal's "m") never leak into the
 *      published netV surface.
 */

import { describe, expect, it } from "vitest";
import { runSmallSignalAc } from "../../../src/sim/engine/ac-analysis.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type SimComponent = SimCircuit["components"][number];
type SimWire = SimCircuit["wires"][number];

// ─── Circuit literal helpers (trap-integration conventions) ──────────────────

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

function capacitor(id: string, capacitance: number): SimComponent {
  return {
    id,
    kind: "capacitor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { capacitance },
  };
}

function switchComponent(id: string, closed: 0 | 1): SimComponent {
  return {
    id,
    kind: "switch",
    pins: [{ id: "a" }, { id: "b" }],
    params: { closed },
  };
}

/** Ideal (rSource=0) sine generator; graph.ts grounds its "neg" pin. */
function sineGenerator(id: string, amplitude: number, frequency: number): SimComponent {
  return {
    id,
    kind: "signal_gen",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: {
      waveform: "sine",
      amplitude,
      offset: 0,
      frequency,
      duty: 0.5,
      phaseDeg: 0,
      delay: 0,
      rSource: 0,
      enabled: 1,
    },
  };
}

/** Pins a1/b1 = winding 1, a2/b2 = winding 2; a1 and a2 are the dots. */
function coupledInductor(id: string, params: Record<string, number>): SimComponent {
  return {
    id,
    kind: "coupled_inductor",
    pins: [{ id: "a1" }, { id: "b1" }, { id: "a2" }, { id: "b2" }],
    params: { ...params },
  };
}

/** Pins a/b; params fSeries (Hz), q, rs (ohm), c0 (pF — UI-scale unit). */
function crystal(id: string, params: Record<string, number>): SimComponent {
  return {
    id,
    kind: "crystal",
    pins: [{ id: "a" }, { id: "b" }],
    params: { ...params },
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

/** Deterministic net id carrying a given component pin (invariant 3). */
function netIdFor(engine: SimEngine, componentId: string, pinId: string): string {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
  );
  if (!net) throw new Error(`Fixture topology error: no net for ${componentId}.${pinId}`);
  return net.id;
}

function nodeVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const value = engine.getNetV()[netIdFor(engine, componentId, pinId)];
  if (value === undefined || !Number.isFinite(value)) {
    throw new Error(`Fixture solve error: ${componentId}.${pinId} has no finite voltage`);
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

/**
 * Step and assert full solver health. The new device families must never
 * trade convergence or conditioning for accuracy, so every accepted step in
 * this file goes through here — the same blanket contract
 * trap-integration.test.ts applies.
 */
function acceptedStep(engine: SimEngine, h: number, context: string): void {
  engine.step(h);
  expect(
    engine.lastConverged,
    `${context}: solver must converge; iterations=${String(engine.lastIters)}, ` +
    `singular=${String(engine.lastMatrixSingular)}, ` +
    `illConditioned=${String(engine.lastMatrixIllConditioned)}, ` +
    `residual=${String(engine.lastRelativeResidual)}`,
  ).toBe(true);
  expect(engine.lastMatrixSingular, `${context}: matrix must not be singular`).toBe(false);
  expect(
    engine.lastMatrixIllConditioned,
    `${context}: matrix must remain well conditioned`,
  ).toBe(false);
}

function setSwitchClosed(circuit: SimCircuit, switchId: string, closed: 0 | 1): void {
  const sw = circuit.components.find((component) => component.id === switchId);
  if (!sw) throw new Error(`Fixture topology error: switch ${switchId} missing`);
  sw.params.closed = closed;
}

/**
 * Mean-subtracted RMS over [from, to). Mean subtraction removes DC offsets
 * (decaying magnetizing-current tails, source offsets) so amplitude ratios
 * compare only the periodic content.
 */
function windowRms(values: readonly number[], from: number, to: number): number {
  expect(to - from, "RMS window must have samples").toBeGreaterThan(1);
  let mean = 0;
  for (let index = from; index < to; index++) mean += values[index]!;
  mean /= to - from;
  let acc = 0;
  for (let index = from; index < to; index++) {
    const d = values[index]! - mean;
    acc += d * d;
  }
  return Math.sqrt(acc / (to - from));
}

function peakToPeak(values: readonly number[], from: number, to: number): number {
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;
  for (let index = from; index <= to; index++) {
    const v = values[index]!;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  return max - min;
}

interface Sample {
  t: number;
  v: number;
}

/**
 * Frequency from rising zero crossings of (v - center): the zeros of a
 * damped cosine are spaced exactly pi/omega_d (the envelope cancels out of
 * the crossing times), so interpolated crossings measure the ring frequency
 * without any envelope fitting — trap-integration.test.ts's method.
 */
function measuredRingFrequency(samples: readonly Sample[], center: number): number {
  const crossings: number[] = [];
  for (let index = 1; index < samples.length; index++) {
    const before = samples[index - 1]!.v - center;
    const after = samples[index]!.v - center;
    if (before < 0 && after >= 0) {
      const dt = samples[index]!.t - samples[index - 1]!.t;
      crossings.push(samples[index - 1]!.t + dt * (-before) / (after - before));
    }
  }
  expect(
    crossings.length,
    "need several full ring cycles to measure frequency",
  ).toBeGreaterThanOrEqual(4);
  return (crossings.length - 1) / (crossings[crossings.length - 1]! - crossings[0]!);
}

// ─── Reference complex arithmetic (ac-small-signal conventions) ──────────────

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

function linSpace(start: number, stop: number, count: number): number[] {
  const points: number[] = [];
  for (let index = 0; index < count; index++) {
    points.push(start + ((stop - start) * index) / (count - 1));
  }
  return points;
}

// ─── Transformer turns ratio (transient) ─────────────────────────────────────

describe("coupled inductor — transformer turns ratio at 1 kHz (transient)", () => {
  /**
   * Validity envelope: ideal sine (1 V, 1 kHz, rSource=0) directly across
   * winding 1 (l1=1 mH), winding 2 (l2=4 mH, k=0.8) open — only the 1 Tohm
   * node shunt loads it, so i2 ~ 1e-12*v and the winding equations collapse
   * to v2 = (M/L1)*v1 = k*sqrt(l2/l1)*v1 pointwise (both voltages are the
   * same di1/dt scaled). Trap at h = T/200; the discrete companion honors
   * the identity to O((omega*h)^2) ~ 1e-3. Acceptance: RMS ratio over
   * cycles 2..3 within 2% of 1.6.
   */
  it("open secondary follows k*sqrt(l2/l1) within 2%", () => {
    const l1 = 1e-3;
    const l2 = 4e-3;
    const k = 0.8;
    const expectedRatio = k * Math.sqrt(l2 / l1);
    const frequency = 1_000;
    const h = 1 / (frequency * 200);
    const circuit: SimCircuit = {
      components: [
        sineGenerator("gen", 1, frequency),
        coupledInductor("tx", { l1, l2, k }),
      ],
      wires: [
        wire("gen", "pos", "tx", "a1"),
        wire("tx", "b1", "gen", "neg"),
        wire("tx", "b2", "gen", "neg"),
        // a2 is deliberately open: the node shunt defines its voltage.
      ],
    };
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(circuit);

    const primary: number[] = [];
    const secondary: number[] = [];
    for (let index = 0; index < 600; index++) {
      acceptedStep(engine, h, `turns-ratio step ${String(index)}`);
      primary.push(componentVoltage(engine, "tx", "a1", "b1"));
      secondary.push(componentVoltage(engine, "tx", "a2", "b2"));
    }

    const ratio = windowRms(secondary, 200, 600) / windowRms(primary, 200, 600);
    expect(
      Math.abs(ratio / expectedRatio - 1),
      `turns ratio: measured=${String(ratio)}, reference=${String(expectedRatio)}`,
    ).toBeLessThanOrEqual(0.02);
  });

  /**
   * Validity envelope: near-ideal transformer (k=1 authored, clamped to
   * 0.9999 => leakage ~2e-4*L1), l1=1 H, l2=0.25 H, n^2 = l1/l2 = 4,
   * RL=100 ohm, driven at 1 kHz through Rs=400 ohm. The exact input
   * impedance Zin = jwL1 + (wM)^2/(RL + jwL2) evaluates to 398.4 + j26.2
   * (magnetizing shunt wL1 = 6283 ohm and leakage together perturb the
   * ideal 400 ohm by ~0.4%), so treating the measured divider as resistive
   * recovers n^2*RL well inside 5%. Slowest natural mode: tau =
   * L1/(Rs || n^2*RL) = 5 ms = 5 cycles; the measurement window at cycles
   * 14..15 leaves e^-2.8 ~ 6% of a mode whose amplitude is bounded by the
   * magnetizing current scale (~6% of the signal current), contributing
   * well under 1% after the mean subtraction. Acceptance: reflected
   * resistance from the RMS divider within 5% of n^2*RL = 400 ohm.
   */
  it("loaded secondary reflects n^2*RL to the primary within 5%", () => {
    const l1 = 1;
    const l2 = 0.25;
    const rl = 100;
    const rs = 400;
    const reflected = (l1 / l2) * rl;
    const frequency = 1_000;
    const h = 1 / (frequency * 200);
    const circuit: SimCircuit = {
      components: [
        sineGenerator("gen", 1, frequency),
        resistor("rs", rs),
        coupledInductor("tx", { l1, l2, k: 1 }),
        resistor("rl", rl),
      ],
      wires: [
        wire("gen", "pos", "rs", "a"),
        wire("rs", "b", "tx", "a1"),
        wire("tx", "b1", "gen", "neg"),
        wire("tx", "a2", "rl", "a"),
        wire("rl", "b", "gen", "neg"),
        wire("tx", "b2", "gen", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(circuit);

    const source: number[] = [];
    const primary: number[] = [];
    for (let index = 0; index < 3_000; index++) {
      acceptedStep(engine, h, `reflection step ${String(index)}`);
      source.push(nodeVoltage(engine, "gen", "pos"));
      primary.push(componentVoltage(engine, "tx", "a1", "b1"));
    }

    const ratio = windowRms(primary, 2_600, 3_000) / windowRms(source, 2_600, 3_000);
    // Divider inversion assuming a resistive Zin: the reactive residue is
    // inside the declared envelope above.
    const measuredReflected = (rs * ratio) / (1 - ratio);
    expect(
      Math.abs(measuredReflected / reflected - 1),
      `reflected resistance: measured=${String(measuredReflected)}, reference=${String(reflected)}`,
    ).toBeLessThanOrEqual(0.05);
  });
});

// ─── Coupled transfer against the analytic two-loop solution (AC) ────────────

describe("coupled inductor — k=0.5 AC transfer against the two-loop closed form", () => {
  /**
   * Validity envelope: l1 = l2 = 1 mH, k = 0.5 (far from both the k=0
   * decoupled and k->1 clamped extremes), Rs = RL = 100 ohm, one frequency
   * (10 kHz) where wL = 62.8 ohm is comparable to the resistances so every
   * term of the 2x2 admittance matters. Reference: mesh equations with
   * i_w positive into the dotted a_w terminal,
   *   vs = (Rs + jwL1)*i1 + jwM*i2
   *   0  = jwM*i1 + (RL + jwL2)*i2,   v(a2) = -RL*i2
   * giving H = jwM*RL / ((RL + jwL2)*(Rs + jwL1) + (wM)^2). Deviations:
   * 1e-12 S node shunts (~1e-10 relative) and LU rounding. Acceptance:
   * complex distance within the assignment's 1% budget.
   */
  it("matches jwM*RL/((RL+jwL2)(Rs+jwL1)+(wM)^2) at 10 kHz within 1%", () => {
    const l1 = 1e-3;
    const l2 = 1e-3;
    const k = 0.5;
    const m = k * Math.sqrt(l1 * l2);
    const rs = 100;
    const rl = 100;
    const frequency = 10_000;
    const circuit: SimCircuit = {
      components: [
        voltageSource("vin", 0),
        resistor("rs", rs),
        coupledInductor("tx", { l1, l2, k }),
        resistor("rl", rl),
      ],
      wires: [
        wire("vin", "pos", "rs", "a"),
        wire("rs", "b", "tx", "a1"),
        wire("tx", "b1", "vin", "neg"),
        wire("tx", "a2", "rl", "a"),
        wire("rl", "b", "vin", "neg"),
        wire("tx", "b2", "vin", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);

    const outputNet = netIdFor(engine, "tx", "a2");
    const result = runSmallSignalAc(engine, {
      inputId: "vin",
      outputNetIds: [outputNet],
      frequenciesHz: [frequency],
    });
    const actual: Cx = { re: result.outputs[0]!.re[0]!, im: result.outputs[0]!.im[0]! };

    const omega = 2 * Math.PI * frequency;
    const den = cAdd(
      cMul({ re: rl, im: omega * l2 }, { re: rs, im: omega * l1 }),
      { re: omega * m * omega * m, im: 0 },
    );
    const reference = cDiv({ re: 0, im: omega * m * rl }, den);

    expect(
      Math.hypot(actual.re - reference.re, actual.im - reference.im) / cMag(reference),
      `coupled AC transfer: actual=${String(actual.re)}+j${String(actual.im)}, `
      + `reference=${String(reference.re)}+j${String(reference.im)}`,
    ).toBeLessThanOrEqual(0.01);
  });
});

// ─── Lossless coupled pair energy conservation (trap) ────────────────────────

// Shorted-secondary ring: with v2 = 0 the winding equations force
// i2 = -(M/L2)*i1, so the pair presents exactly Leff = L1*(1 - k^2) to the
// capacitor and BOTH windings carry current — the mutual companion terms are
// on the energy path, unlike an open-secondary ring which only exercises L1.
const RING_L1 = 10e-3;
const RING_L2 = 10e-3;
const RING_K = 0.5;
const RING_C = 1e-6;
const RING_LEFF = RING_L1 * (1 - RING_K * RING_K);
const RING_F0 = 1 / (2 * Math.PI * Math.sqrt(RING_LEFF * RING_C));
const RING_H = 1 / (RING_F0 * 100);

describe("coupled inductor — lossless pair energy conservation (trap)", () => {
  /**
   * Validity envelope: 5 V initial capacitor energy released into the
   * primary of a dcr-free coupled pair whose secondary is shorted (both
   * pins on the ground net), h = T/100. Losses are limited to the 1 mOhm
   * closed contact in the primary loop (Q ~ 8.7e4, amplitude loss ~4e-5
   * per cycle) and the 1 Tohm node shunts; trap adds essentially none, so
   * peak-to-peak decay over five cycles must stay under 1%. The measured
   * ring frequency is the mutual-term proof: f0 = 1/(2*pi*sqrt(L1*(1-k^2)*C))
   * sits 15% below the uncoupled 1/(2*pi*sqrt(L1*C)), so a wrong or dropped
   * M fails the 1% frequency budget outright. Acceptance: cycle-1 vs
   * cycle-6 peak-to-peak loss < 1%, frequency within 1% of f0, first
   * peak-to-peak > 9 V (the tank really rings).
   */
  it("keeps peak-to-peak loss under 1% over 5 cycles and rings at the coupled f0", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("src", 5),
        switchComponent("swChg", 1),
        resistor("rchg", 10),
        capacitor("c", RING_C),
        switchComponent("swTank", 0),
        coupledInductor("tx", { l1: RING_L1, l2: RING_L2, k: RING_K }),
      ],
      wires: [
        wire("src", "pos", "swChg", "a"),
        wire("swChg", "b", "rchg", "a"),
        wire("rchg", "b", "c", "a"),
        wire("c", "b", "src", "neg"),
        wire("c", "a", "swTank", "a"),
        wire("swTank", "b", "tx", "a1"),
        wire("tx", "b1", "c", "b"),
        // Secondary shorted onto the ground net: v2 = 0 by topology.
        wire("tx", "a2", "tx", "b2"),
        wire("tx", "b2", "c", "b"),
      ],
    };
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(circuit);

    // 200 x 5 us = 100 charge time constants (tau = 10 ohm x 1 uF); the
    // primary hangs behind the open swTank so it cannot load the charge.
    for (let index = 0; index < 200; index++) {
      acceptedStep(engine, 5e-6, `ring charge step ${String(index)}`);
    }
    expect(Math.abs(componentVoltage(engine, "c", "a", "b") - 5)).toBeLessThanOrEqual(1e-3);

    // Both switch flips at one load() boundary; load() forces the next step
    // onto backward Euler — the documented trap history re-anchor.
    setSwitchClosed(circuit, "swChg", 0);
    setSwitchClosed(circuit, "swTank", 1);
    engine.load(circuit);

    const voltages: number[] = [];
    const samples: Sample[] = [];
    for (let index = 0; index < 801; index++) {
      acceptedStep(engine, RING_H, `ring step ${String(index)}`);
      const v = componentVoltage(engine, "c", "a", "b");
      voltages.push(v);
      samples.push({ t: engine.simTime, v });
    }

    const first = peakToPeak(voltages, 0, 100);
    const last = peakToPeak(voltages, 500, 600);
    expect(first, "the coupled tank must actually ring at ~±5 V").toBeGreaterThan(9);
    expect(
      1 - last / first,
      `coupled ring pp first=${String(first)} last=${String(last)}`,
    ).toBeLessThan(0.01);

    const frequency = measuredRingFrequency(samples, 0);
    expect(
      Math.abs(frequency / RING_F0 - 1),
      `coupled ring frequency: measured=${String(frequency)} Hz, reference=${String(RING_F0)} Hz`,
    ).toBeLessThanOrEqual(0.01);
  });
});

// ─── Snapshot rollback for composite-key state ───────────────────────────────

describe("coupled inductor + crystal — snapshot rollback of composite-key state", () => {
  /**
   * Validity envelope: deterministic replay, not physics — the adaptive-step
   * worker restores a snapshot and re-runs identical intervals, so the
   * composite-key winding state (tx:1/tx:2 in inds+indsV) and crystal branch
   * state (xt:cs/xt:c0 in caps+capsI, xt:ls in inds+indsV) must round-trip a
   * saveState/restoreState pair bitwise and a restored engine must retrace
   * the identical trajectory. Acceptance: Object.is equality on every
   * composite key after the round trip, and zero bitwise netV/elementI
   * mismatches across a 60-step replay taken mid-transient in trap mode.
   */
  it("round-trips winding and motional state bitwise and replays bitwise", () => {
    const circuit: SimCircuit = {
      components: [
        sineGenerator("gen", 2, 1_000),
        resistor("rs", 100),
        coupledInductor("tx", { l1: 1e-3, l2: 1e-3, k: 0.8 }),
        resistor("rl", 100),
        crystal("xt", { fSeries: 1e6, q: 50_000, rs: 50, c0: 5 }),
      ],
      wires: [
        wire("gen", "pos", "rs", "a"),
        wire("rs", "b", "tx", "a1"),
        wire("tx", "b1", "gen", "neg"),
        wire("tx", "a2", "rl", "a"),
        wire("rl", "b", "gen", "neg"),
        wire("tx", "b2", "gen", "neg"),
        wire("xt", "a", "tx", "a2"),
        wire("xt", "b", "gen", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(circuit);
    const h = 5e-6;
    for (let index = 0; index < 150; index++) {
      acceptedStep(engine, h, `rollback warmup step ${String(index)}`);
    }

    const snapshot = engine.saveState();
    expect(snapshot.capsI).toBeInstanceOf(Map);
    expect(snapshot.indsV).toBeInstanceOf(Map);
    // Mid-sine every branch carries live values, so definedness and
    // finiteness prove the snapshot captured the composite-key state.
    const inductiveKeys = ["tx:1", "tx:2", "xt:ls"] as const;
    const capacitiveKeys = ["xt:cs", "xt:c0"] as const;
    for (const key of inductiveKeys) {
      const current = snapshot.inds.get(key);
      const history = snapshot.indsV!.get(key);
      expect(current !== undefined && Number.isFinite(current), `inds[${key}] captured`).toBe(true);
      expect(history !== undefined && Number.isFinite(history), `indsV[${key}] captured`).toBe(true);
    }
    for (const key of capacitiveKeys) {
      const charge = snapshot.caps.get(key);
      const history = snapshot.capsI!.get(key);
      expect(charge !== undefined && Number.isFinite(charge), `caps[${key}] captured`).toBe(true);
      expect(history !== undefined && Number.isFinite(history), `capsI[${key}] captured`).toBe(true);
    }
    // The primary winding must be doing real work when the snapshot is cut,
    // or the bitwise claims below would be vacuously about zeros.
    expect(Math.abs(snapshot.inds.get("tx:1")!)).toBeGreaterThan(1e-6);

    const replaySteps = 60;
    const recordedNetV: Array<Record<string, number>> = [];
    const recordedElementI: Array<Record<string, number>> = [];
    for (let index = 0; index < replaySteps; index++) {
      acceptedStep(engine, h, `rollback record step ${String(index)}`);
      recordedNetV.push({ ...engine.getNetV() });
      recordedElementI.push({ ...engine.getElementI() });
    }

    engine.restoreState(snapshot);
    expect(engine.simTime).toBe(snapshot.simTime);

    const roundTrip = engine.saveState();
    for (const key of inductiveKeys) {
      expect(Object.is(roundTrip.inds.get(key), snapshot.inds.get(key)), `inds[${key}] round-trip`).toBe(true);
      expect(Object.is(roundTrip.indsV!.get(key), snapshot.indsV!.get(key)), `indsV[${key}] round-trip`).toBe(true);
    }
    for (const key of capacitiveKeys) {
      expect(Object.is(roundTrip.caps.get(key), snapshot.caps.get(key)), `caps[${key}] round-trip`).toBe(true);
      expect(Object.is(roundTrip.capsI!.get(key), snapshot.capsI!.get(key)), `capsI[${key}] round-trip`).toBe(true);
    }

    const mismatches: string[] = [];
    for (let index = 0; index < replaySteps; index++) {
      acceptedStep(engine, h, `rollback replay step ${String(index)}`);
      const netV = engine.getNetV();
      for (const key of Object.keys(recordedNetV[index]!)) {
        if (!Object.is(netV[key], recordedNetV[index]![key])) {
          mismatches.push(
            `replay step ${String(index)} net ${key}: `
            + `${String(netV[key])} vs recorded ${String(recordedNetV[index]![key])}`,
          );
        }
      }
      const elementI = engine.getElementI();
      for (const key of Object.keys(recordedElementI[index]!)) {
        if (!Object.is(elementI[key], recordedElementI[index]![key])) {
          mismatches.push(
            `replay step ${String(index)} elementI ${key}: `
            + `${String(elementI[key])} vs recorded ${String(recordedElementI[index]![key])}`,
          );
        }
      }
      if (mismatches.length > 8) break; // enough evidence; keep the failure readable
    }
    expect(mismatches).toEqual([]);
  });
});

// ─── DC operating point ──────────────────────────────────────────────────────

describe("coupled inductor — dcOperatingPoint winding semantics", () => {
  /**
   * Validity envelope: both windings driven from the same 5 V source through
   * 1 kohm each, default dcr (0, stamped as the documented 1e-6 ohm floor),
   * k = 0.9. At a true operating point each winding is a dc short through
   * its own resistance and the mutual term contributes nothing, so both
   * winding nodes sit at ~5e-9 V (the 1e-6/1000 divider) and each winding
   * carries the full 5 mA. Acceptance: |v| <= 1e-6 V on both winding nodes,
   * element current (winding 1 by convention) and the committed winding-2
   * state both within 1e-6 A of 5 mA, ladder converged.
   */
  it("shorts both windings and commits the divider currents", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("v", 5),
        resistor("r1", 1_000),
        resistor("r2", 1_000),
        coupledInductor("tx", { l1: 1e-3, l2: 4e-3, k: 0.9 }),
      ],
      wires: [
        wire("v", "pos", "r1", "a"),
        wire("r1", "b", "tx", "a1"),
        wire("tx", "b1", "v", "neg"),
        wire("v", "pos", "r2", "a"),
        wire("r2", "b", "tx", "a2"),
        wire("tx", "b2", "v", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);

    const result = engine.dcOperatingPoint();
    expect(result.converged, `OP ladder must converge (method=${result.method})`).toBe(true);

    const vA1 = result.netV[netIdFor(engine, "tx", "a1")] ?? Number.NaN;
    const vA2 = result.netV[netIdFor(engine, "tx", "a2")] ?? Number.NaN;
    expect(Math.abs(vA1), `winding 1 must be a dc short, v(a1)=${String(vA1)}`).toBeLessThanOrEqual(1e-6);
    expect(Math.abs(vA2), `winding 2 must be a dc short, v(a2)=${String(vA2)}`).toBeLessThanOrEqual(1e-6);

    const expectedCurrent = 5 / 1_000;
    const i1 = engine.getElementI().tx ?? Number.NaN;
    expect(
      Math.abs(i1 - expectedCurrent),
      `winding 1 dc current: ${String(i1)} vs ${String(expectedCurrent)}`,
    ).toBeLessThanOrEqual(1e-6);
    // updateCurrent only publishes winding 1; winding 2's normalisation is
    // observable through its committed composite-key state.
    const i2 = engine.saveState().inds.get("tx:2") ?? Number.NaN;
    expect(
      Math.abs(i2 - expectedCurrent),
      `winding 2 committed dc current: ${String(i2)} vs ${String(expectedCurrent)}`,
    ).toBeLessThanOrEqual(1e-6);
  });
});

// ─── Crystal resonances (AC) ─────────────────────────────────────────────────

// Datasheet triple + shunt used by every crystal fixture below. The derived
// motional values mirror crystalParams() exactly: ls = q*rs/ws,
// cs = 1/(ws^2*ls); c0 is authored in picofarads.
const XT_FS = 1e6;
const XT_Q = 50_000;
const XT_RS = 50;
const XT_C0_PF = 5;
const XT_WS = 2 * Math.PI * XT_FS;
const XT_LS = (XT_Q * XT_RS) / XT_WS;
const XT_CS = 1 / (XT_WS * XT_WS * XT_LS);
const XT_C0F = XT_C0_PF * 1e-12;

function crystalDividerEngine(): SimEngine {
  // vin — crystal — 50 ohm sense: H = Rsense/(Rsense + Zxtal), so |H| peaks
  // at the conductance maximum (series resonance) and dips at the parallel
  // antiresonance where Zxtal is largest.
  const circuit: SimCircuit = {
    components: [
      voltageSource("vin", 0),
      crystal("xt", { fSeries: XT_FS, q: XT_Q, rs: XT_RS, c0: XT_C0_PF }),
      resistor("rsense", 50),
    ],
    wires: [
      wire("vin", "pos", "xt", "a"),
      wire("xt", "b", "rsense", "a"),
      wire("rsense", "b", "vin", "neg"),
    ],
  };
  const engine = new SimEngine();
  engine.load(circuit);
  return engine;
}

function crystalDividerMagnitudes(engine: SimEngine, frequencies: readonly number[]): number[] {
  const result = runSmallSignalAc(engine, {
    inputId: "vin",
    outputNetIds: [netIdFor(engine, "xt", "b")],
    frequenciesHz: [...frequencies],
  });
  return frequencies.map((_, index) =>
    Math.hypot(result.outputs[0]!.re[index]!, result.outputs[0]!.im[index]!),
  );
}

describe("crystal — AC series resonance and antiresonance placement", () => {
  /**
   * Validity envelope: the divider fixture above swept linearly across
   * fSeries*(1 +/- 0.002) at 10 Hz spacing. The motional branch is derived
   * so its reactance zero lands exactly at fSeries; c0's 31.8 kohm shunt
   * shifts the |H| maximum by well under 1 Hz (its susceptance is 1.6e-3 of
   * the motional conductance). The response FWHM here is
   * ~fs*(rs+Rsense)/(q*rs) = 40 Hz, so 10 Hz sampling resolves the peak.
   * Acceptance: argmax within 0.1% of fSeries, interior to the sweep, and
   * |H| at the peak within [0.49, 0.51] (the rs/(rs+Rsense) = 0.5 divider).
   */
  it("places the series resonance within 0.1% of fSeries", () => {
    const engine = crystalDividerEngine();
    const frequencies = linSpace(XT_FS * 0.998, XT_FS * 1.002, 401);
    const magnitudes = crystalDividerMagnitudes(engine, frequencies);

    let maxIndex = 0;
    for (let index = 1; index < magnitudes.length; index++) {
      if (magnitudes[index]! > magnitudes[maxIndex]!) maxIndex = index;
    }
    expect(maxIndex, "series peak must be interior to the sweep").toBeGreaterThan(0);
    expect(maxIndex, "series peak must be interior to the sweep").toBeLessThan(frequencies.length - 1);

    const fPeak = frequencies[maxIndex]!;
    expect(
      Math.abs(fPeak / XT_FS - 1),
      `series resonance: measured=${String(fPeak)} Hz, reference=${String(XT_FS)} Hz`,
    ).toBeLessThanOrEqual(1e-3);
    expect(magnitudes[maxIndex]!).toBeGreaterThan(0.49);
    expect(magnitudes[maxIndex]!).toBeLessThan(0.51);
  });

  /**
   * Validity envelope: linear sweep of fSeries*[1.0005, 1.02]. |Zxtal| rises
   * monotonically from the series resonance to the parallel antiresonance
   * fp = fs*sqrt(1 + cs/c0) and falls beyond it, so the |H| argmin lands at
   * the sample nearest fp at any resolution; the impedance-peak width
   * (~1.6 kHz) is far above the 49 Hz spacing. Acceptance (assignment
   * bounds): argmin strictly above fSeries and below
   * fSeries*(1 + cs/(2*c0))*1.01 — the first-order expansion of fp with 1%
   * headroom — interior to the sweep, with |H| < 0.05 at the dip (Zxtal is
   * tens of megohms against the 50 ohm sense there).
   */
  it("places the antiresonance above fSeries and below fSeries*(1+cs/(2*c0))*1.01", () => {
    const engine = crystalDividerEngine();
    const frequencies = linSpace(XT_FS * 1.0005, XT_FS * 1.02, 401);
    const magnitudes = crystalDividerMagnitudes(engine, frequencies);

    let minIndex = 0;
    for (let index = 1; index < magnitudes.length; index++) {
      if (magnitudes[index]! < magnitudes[minIndex]!) minIndex = index;
    }
    expect(minIndex, "antiresonance must be interior to the sweep").toBeGreaterThan(0);
    expect(minIndex, "antiresonance must be interior to the sweep").toBeLessThan(frequencies.length - 1);

    const fMin = frequencies[minIndex]!;
    const upperBound = XT_FS * (1 + XT_CS / (2 * XT_C0F)) * 1.01;
    expect(fMin, `antiresonance ${String(fMin)} Hz must sit above fSeries`).toBeGreaterThan(XT_FS);
    expect(
      fMin,
      `antiresonance ${String(fMin)} Hz must sit below ${String(upperBound)} Hz`,
    ).toBeLessThan(upperBound);
    expect(magnitudes[minIndex]!, "the antiresonance dip must be deep").toBeLessThan(0.05);
  });
});

// ─── Crystal trap-mode ringdown ──────────────────────────────────────────────

describe("crystal — trap-mode ringdown loss", () => {
  /**
   * Validity envelope: the motional capacitor is charged to 5 V at dc-like
   * step sizes (backward Euler's stiff damping settles the 1 MHz branch in a
   * handful of 5 us steps; at dc the crystal commits cs = pin-to-pin voltage
   * because the series branch carries no current), then the pins are shorted
   * through a 1 mOhm contact and the motional loop rings at fSeries in trap
   * mode at h = T/100. Physical amplitude loss per cycle for the series
   * RLC loop is pi/Qeff with Qeff = ws*ls/(rs + 1 mOhm) ~ q, i.e. ~6.3e-5;
   * trap adds no envelope damping of its own (the reason the mode exists),
   * so the measured loss must stay under the assignment bound
   * 2*pi/q * 1.5 ~ 1.9e-4. The switch flip also dumps c0's 5 V through the
   * contact — a tau ~ 5e-15 s mode that h = 1e-8 can never resolve, and
   * whose companion-history kick trap would carry as a barely-decaying
   * Nyquist-rate ring — so the fixture applies the engine's documented
   * BE-at-events rule: a few backward-Euler steps park the discharge (BE is
   * L-stable, one step settles it) before the trap window opens on calm
   * histories. The measurement uses 5-cycle RMS windows of the committed
   * element current (the only observable of a shorted crystal) 25 cycles
   * apart: RMS over integer-period windows cancels sampling-phase error,
   * and the trap period warp ((w*h)^2/12 ~ 3e-4) plus the one-off BE anchor
   * step contribute well under 1e-5 per cycle. Acceptance: loss per cycle
   * in (pi/(4*q), 2*pi/q*1.5) — the lower bound proves the rs loss is
   * actually resolved rather than a dead ring being measured — and ring
   * frequency within 0.5% of fSeries.
   */
  it("rings at fSeries and loses less than 2*pi/q*1.5 amplitude per cycle", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("src", 5),
        switchComponent("swChg", 1),
        resistor("rchg", 10),
        crystal("xt", { fSeries: XT_FS, q: XT_Q, rs: XT_RS, c0: XT_C0_PF }),
        switchComponent("swRing", 0),
      ],
      wires: [
        wire("src", "pos", "swChg", "a"),
        wire("swChg", "b", "rchg", "a"),
        wire("rchg", "b", "xt", "a"),
        wire("xt", "b", "src", "neg"),
        wire("swRing", "a", "xt", "a"),
        wire("swRing", "b", "xt", "b"),
      ],
    };
    // Charge under backward Euler: trap would carry the stiff 1 MHz mode
    // undamped through the 5 us charge steps instead of settling it.
    const engine = new SimEngine();
    engine.load(circuit);
    for (let index = 0; index < 200; index++) {
      acceptedStep(engine, 5e-6, `crystal charge step ${String(index)}`);
    }
    const csCharged = engine.saveState().caps.get("xt:cs") ?? Number.NaN;
    expect(
      Math.abs(csCharged - 5),
      `motional capacitor must charge to the rail, cs=${String(csCharged)}`,
    ).toBeLessThanOrEqual(5e-3);

    // Flip to the shorted ring, still under backward Euler: the first steps
    // absorb the stiff c0-through-contact discharge the trap companion
    // could only ring on (see the envelope above). Each BE step costs the
    // motional ring ~(w*h)^2/2 = 0.2% of amplitude — a one-time loss before
    // the measurement window, not a per-cycle one.
    setSwitchClosed(circuit, "swChg", 0);
    setSwitchClosed(circuit, "swRing", 1);
    engine.load(circuit);
    const h = 1 / (XT_FS * 100);
    for (let index = 0; index < 3; index++) {
      acceptedStep(engine, h, `c0 discharge settle step ${String(index)}`);
    }

    // Now open the trap window; the method switch arms the documented
    // one-step BE anchor, taken here so the sampled run is pure trap and
    // the anchored histories are committed from a calm loop.
    engine.setIntegrationMethod("trap");
    acceptedStep(engine, h, "trap anchor step");

    const steps = 3_000; // 30 cycles
    const currents: number[] = [];
    const samples: Sample[] = [];
    for (let index = 0; index < steps; index++) {
      acceptedStep(engine, h, `crystal ring step ${String(index)}`);
      const i = engine.getElementI().xt ?? Number.NaN;
      expect(Number.isFinite(i), `crystal current finite at step ${String(index)}`).toBe(true);
      currents.push(i);
      samples.push({ t: engine.simTime, v: i });
    }

    // Amplitude sanity: 5 V across cs rings at I = V*sqrt(cs/ls) ~ 2 uA.
    const expectedAmplitude = 5 * Math.sqrt(XT_CS / XT_LS);
    const earlyRms = windowRms(currents, 0, 500);
    expect(earlyRms).toBeGreaterThan(expectedAmplitude / 2);

    const frequency = measuredRingFrequency(samples, 0);
    expect(
      Math.abs(frequency / XT_FS - 1),
      `crystal ring frequency: measured=${String(frequency)} Hz, reference=${String(XT_FS)} Hz`,
    ).toBeLessThanOrEqual(0.005);

    const lateRms = windowRms(currents, 2_500, 3_000);
    const cyclesApart = 25;
    const lossPerCycle = Math.log(earlyRms / lateRms) / cyclesApart;
    expect(
      lossPerCycle,
      `ringdown loss per cycle ${String(lossPerCycle)} must stay under 2*pi/q*1.5`,
    ).toBeLessThan(((2 * Math.PI) / XT_Q) * 1.5);
    expect(
      lossPerCycle,
      `ringdown loss per cycle ${String(lossPerCycle)} must resolve the physical pi/q decay`,
    ).toBeGreaterThan(Math.PI / (4 * XT_Q));
  });
});

// ─── Internal node hygiene ───────────────────────────────────────────────────

describe("crystal internal node — netV publishes only net ids", () => {
  /**
   * Validity envelope: a circuit containing the two Wave A6 kinds under test
   * — the crystal allocates exactly one internal solver node ("m"), the
   * coupled inductor allocates none. The engine documents that internal
   * rows live in nodeCount but never in nodeIdx, so getNetV() (transient)
   * and dcOperatingPoint().netV must carry exactly the deterministic net
   * ids plus gnd, all finite, with no composite id:name key. Acceptance:
   * bidirectional set consistency between engine.nets and the netV keys,
   * zero ":"-shaped keys, nodeCount = non-gnd nets + 1, and the internal
   * key resolving to no net row.
   */
  it("keeps engine.nets and getNetV() keys consistent through transient and OP", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("v", 5),
        resistor("r", 1_000),
        crystal("xt", { fSeries: XT_FS, q: XT_Q, rs: XT_RS, c0: XT_C0_PF }),
        coupledInductor("tx", { l1: 1e-3, l2: 1e-3, k: 0.8 }),
        resistor("rl", 100),
      ],
      wires: [
        wire("v", "pos", "r", "a"),
        wire("r", "b", "xt", "a"),
        wire("xt", "b", "v", "neg"),
        wire("r", "b", "tx", "a1"),
        wire("tx", "b1", "v", "neg"),
        wire("tx", "a2", "rl", "a"),
        wire("rl", "b", "v", "neg"),
        wire("tx", "b2", "v", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    for (let index = 0; index < 3; index++) {
      acceptedStep(engine, 1e-6, `hygiene step ${String(index)}`);
    }

    const netIds = new Set(engine.nets.map((net) => net.id));
    expect(netIds.size).toBeGreaterThan(0);

    const checkNetVSurface = (netV: Record<string, number>, label: string): void => {
      for (const key of Object.keys(netV)) {
        expect(
          netIds.has(key) || key === "gnd",
          `${label}: key "${key}" is not a net id`,
        ).toBe(true);
        expect(
          key.includes(":"),
          `${label}: key "${key}" looks like a leaked internal composite key`,
        ).toBe(false);
        expect(
          Number.isFinite(netV[key]),
          `${label}: value for "${key}" must be finite`,
        ).toBe(true);
      }
      for (const id of netIds) {
        expect(netV[id] !== undefined, `${label}: net "${id}" missing from netV`).toBe(true);
      }
    };

    checkNetVSurface(engine.getNetV(), "transient netV");

    // The hygiene claim must not be vacuous: the crystal's "m" row exists in
    // the matrix (nodeCount counts it) yet resolves to no published net.
    const nonGndNets = engine.nets.filter((net) => net.id !== "gnd").length;
    expect(engine.matrixDimensions().nodeCount).toBe(nonGndNets + 1);
    expect(engine.netRow("xt:m")).toBeUndefined();

    const result = engine.dcOperatingPoint();
    expect(result.converged, `OP ladder must converge (method=${result.method})`).toBe(true);
    checkNetVSurface(result.netV, "dcOperatingPoint netV");
    checkNetVSurface(engine.getNetV(), "post-OP netV");
  });
});
