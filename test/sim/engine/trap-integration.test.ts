/**
 * Trapezoidal integration accuracy and behavior tests
 * ===================================================
 *
 * Wave A2 added an opt-in trapezoidal companion model for capacitors and
 * inductors (SimEngine.setIntegrationMethod). These tests lock the three
 * contracts that mode carries:
 *
 *   1. Second-order accuracy: trap preserves ringing frequency and amplitude
 *      envelope where backward Euler visibly over-damps (the reason the wave
 *      exists), and it still meets the first-order reference budgets.
 *   2. Discontinuity safety: the documented BE-at-events protocol
 *      (markDiscontinuity + the worker's land-before/guard-step clamping)
 *      keeps square-wave transients artifact-free.
 *   3. Default isolation: an engine never switched to trap is bitwise
 *      identical to an engine explicitly set to "be", and trap-mode
 *      snapshots round-trip the new capsI/indsV histories exactly.
 *
 * Analytic references follow physics-reference-benchmarks.test.ts: every
 * fixture states its validity envelope and an explicit error budget. Never
 * widen a budget to make a failing engine pass — report the engine bug.
 */

import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

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

function capacitor(id: string, capacitance: number): SimComponent {
  return {
    id,
    kind: "capacitor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { capacitance },
  };
}

function inductor(id: string, inductance: number): SimComponent {
  return {
    id,
    kind: "inductor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { inductance },
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

/** Ideal (rSource=0) square generator; graph.ts grounds its "neg" pin. */
function squareGenerator(id: string, low: number, high: number, frequency: number): SimComponent {
  return {
    id,
    kind: "signal_gen",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: {
      waveform: "square",
      amplitude: (high - low) / 2,
      offset: (high + low) / 2,
      frequency,
      duty: 0.5,
      phaseDeg: 0,
      delay: 0,
      rSource: 0,
      enabled: 1,
    },
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
  if (!net) throw new Error(`Fixture topology error: no net for ${componentId}.${pinId}`);
  const value = engine.getNetV()[net.id];
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
 * Step and assert full solver health. Trap mode must never trade convergence
 * or conditioning for accuracy, so EVERY accepted step in this file goes
 * through here — that blanket coverage is the "solver health clean" contract.
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
  expect(
    Number.isFinite(engine.lastRelativeResidual),
    `${context}: linear residual must be finite`,
  ).toBe(true);
}

function setSourceVoltage(circuit: SimCircuit, sourceId: string, voltage: number): void {
  const source = circuit.components.find((component) => component.id === sourceId);
  if (!source) throw new Error(`Fixture topology error: source ${sourceId} missing`);
  source.params.voltage = voltage;
}

function setSwitchClosed(circuit: SimCircuit, switchId: string, closed: 0 | 1): void {
  const sw = circuit.components.find((component) => component.id === switchId);
  if (!sw) throw new Error(`Fixture topology error: switch ${switchId} missing`);
  sw.params.closed = closed;
}

interface Sample {
  t: number;
  v: number;
}

interface Extremum {
  t: number;
  magnitude: number;
}

/**
 * Local extrema of (v - center), deduplicated to one detection per
 * quarter-period so a flat pair of samples at a peak cannot double-count.
 * The magnitude floor keeps the fully decayed tail out of envelope fits.
 */
function detectExtrema(
  samples: readonly Sample[],
  center: number,
  minMagnitude: number,
  minGapSeconds: number,
): Extremum[] {
  const out: Extremum[] = [];
  for (let index = 1; index < samples.length - 1; index++) {
    const before = samples[index].v - samples[index - 1].v;
    const after = samples[index + 1].v - samples[index].v;
    if (before === 0) continue;
    if ((before > 0 && after <= 0) || (before < 0 && after >= 0)) {
      const magnitude = Math.abs(samples[index].v - center);
      if (magnitude < minMagnitude) continue;
      const t = samples[index].t;
      if (out.length > 0 && t - out[out.length - 1].t < minGapSeconds) continue;
      out.push({ t, magnitude });
    }
  }
  return out;
}

/**
 * Ringing frequency from rising zero crossings of (v - center). The zeros of
 * an exponentially damped cosine are spaced exactly pi/omega_d (the envelope
 * cancels out of the crossing times), so linear interpolation between
 * samples measures f_d directly without any envelope fitting.
 */
function measuredRingingFrequency(samples: readonly Sample[], center: number): number {
  const crossings: number[] = [];
  for (let index = 1; index < samples.length; index++) {
    const before = samples[index - 1].v - center;
    const after = samples[index].v - center;
    if (before < 0 && after >= 0) {
      const dt = samples[index].t - samples[index - 1].t;
      crossings.push(samples[index - 1].t + dt * (-before) / (after - before));
    }
  }
  expect(crossings.length, "need several full ringing cycles to measure frequency").toBeGreaterThanOrEqual(5);
  return (crossings.length - 1) / (crossings[crossings.length - 1] - crossings[0]);
}

/** ln(A_first/A_last) per second — the observed envelope decay rate. */
function envelopeDecayRate(extrema: readonly Extremum[]): number {
  expect(extrema.length, "need several extrema to measure the envelope").toBeGreaterThanOrEqual(4);
  const first = extrema[0];
  const last = extrema[extrema.length - 1];
  return Math.log(first.magnitude / last.magnitude) / (last.t - first.t);
}

function peakToPeak(samples: readonly Sample[], from: number, to: number): number {
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;
  for (let index = from; index <= to; index++) {
    const v = samples[index].v;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  return max - min;
}

// ─── Series RLC ringing ──────────────────────────────────────────────────────

// R deliberately small (Q ≈ 100) so physical damping is weak and backward
// Euler's numerical damping — (omega·h)^2/2 per step — dominates its envelope.
// That separation is what makes the BE/trap damping contrast assertable.
const RLC_R = 1;
const RLC_L = 10e-3;
const RLC_C = 1e-6;
const RLC_SUPPLY = 5;
const RLC_ALPHA = RLC_R / (2 * RLC_L);
const RLC_OMEGA_D = Math.sqrt(1 / (RLC_L * RLC_C) - RLC_ALPHA * RLC_ALPHA);
const RLC_FD = RLC_OMEGA_D / (2 * Math.PI);
const RLC_PERIOD = 1 / RLC_FD;
const RLC_H = RLC_PERIOD / 100;
const RLC_STEPS = 1000; // ten ringing periods

function seriesRlcCircuit(): SimCircuit {
  return {
    components: [
      voltageSource("src", RLC_SUPPLY),
      resistor("r", RLC_R),
      inductor("l", RLC_L),
      capacitor("c", RLC_C),
    ],
    wires: [
      wire("src", "pos", "r", "a"),
      wire("r", "b", "l", "a"),
      wire("l", "b", "c", "a"),
      wire("c", "b", "src", "neg"),
    ],
  };
}

function runSeriesRlc(method: "be" | "trap"): Sample[] {
  const engine = new SimEngine();
  engine.setIntegrationMethod(method);
  engine.load(seriesRlcCircuit());
  const samples: Sample[] = [];
  for (let index = 0; index < RLC_STEPS; index++) {
    acceptedStep(engine, RLC_H, `RLC ${method} step ${String(index)}`);
    samples.push({ t: engine.simTime, v: componentVoltage(engine, "c", "a", "b") });
  }
  return samples;
}

describe("trapezoidal integration — series RLC ringing accuracy", () => {
  /**
   * Validity envelope: ideal series RLC (R=1, L=10 mH, C=1 uF), 5 V step from
   * exactly zero stored energy, h = T_d/100. Reference: capacitor extrema sit
   * at omega_d·t = k·pi with |v - V| = V·exp(-alpha·t) exactly, and zero
   * crossings of (v - V) are spaced pi/omega_d independent of the envelope.
   * Acceptance: measured f_d within 0.5% (trap's theoretical period warp at
   * this step is only (omega·h)^2/12 ≈ 0.03%) and every extremum's decay
   * ratio within 5% of exp(-R·t/(2L)) (discrete peak flattening ≈ 0.05%).
   */
  it("trap mode holds ringing frequency within 0.5% and the exp(-Rt/2L) envelope within 5%", () => {
    const samples = runSeriesRlc("trap");

    const frequency = measuredRingingFrequency(samples, RLC_SUPPLY);
    expect(
      Math.abs(frequency - RLC_FD) / RLC_FD,
      `f_measured=${String(frequency)} Hz vs f_d=${String(RLC_FD)} Hz`,
    ).toBeLessThanOrEqual(0.005);

    const extrema = detectExtrema(samples, RLC_SUPPLY, 0.1, RLC_PERIOD / 4);
    expect(extrema.length).toBeGreaterThanOrEqual(12);
    const base = extrema[0];
    for (let index = 1; index < extrema.length; index++) {
      const peak = extrema[index];
      const observedRatio = peak.magnitude / base.magnitude;
      const referenceRatio = Math.exp(-RLC_ALPHA * (peak.t - base.t));
      expect(
        Math.abs(observedRatio / referenceRatio - 1),
        `extremum ${String(index)} at t=${String(peak.t)}: observed=${String(observedRatio)}, ` +
        `reference=${String(referenceRatio)}`,
      ).toBeLessThanOrEqual(0.05);
    }
  });

  /**
   * Validity envelope: identical circuit and step size in both modes; decay
   * rates measured the same way from detected extrema. Physical decay is
   * alpha = 50 /s; BE adds ~(omega·h)^2/(2h) ≈ 310 /s of numerical damping at
   * h = T_d/100 while trap adds essentially none. Acceptance: BE's observed
   * envelope decay rate exceeds trap's by more than 3x. This contrast is the
   * wave's reason to exist — do not weaken it.
   */
  it("backward Euler over-damps the same ringing by more than 3x", () => {
    const trapExtrema = detectExtrema(runSeriesRlc("trap"), RLC_SUPPLY, 0.1, RLC_PERIOD / 4);
    const beExtrema = detectExtrema(runSeriesRlc("be"), RLC_SUPPLY, 0.1, RLC_PERIOD / 4);
    const trapDecayRate = envelopeDecayRate(trapExtrema);
    const beDecayRate = envelopeDecayRate(beExtrema);

    expect(trapDecayRate).toBeGreaterThan(0);
    expect(
      beDecayRate / trapDecayRate,
      `BE decay=${String(beDecayRate)} /s vs trap decay=${String(trapDecayRate)} /s`,
    ).toBeGreaterThan(3);
  });
});

// ─── LC tank amplitude conservation ──────────────────────────────────────────

const TANK_L = 10e-3;
const TANK_C = 1e-6;
const TANK_PERIOD = 2 * Math.PI * Math.sqrt(TANK_L * TANK_C);
const TANK_H = TANK_PERIOD / 100;
const TANK_RING_STEPS = 1001; // ten periods, inclusive endpoints for both windows

/**
 * Charge topology: src -> swChg -> rchg -> C, with L hanging off the cap
 * through the open swTank so the inductor cannot DC-short the capacitor
 * while it charges. Flipping both switches then leaves a C || (L + 1 mOhm
 * contact) loop whose only other losses are the declared 1 Tohm node shunts;
 * physical decay across ten periods is under 0.05%.
 */
function tankCircuit(): SimCircuit {
  return {
    components: [
      voltageSource("src", 5),
      switchComponent("swChg", 1),
      resistor("rchg", 10),
      capacitor("c", TANK_C),
      inductor("l", TANK_L),
      switchComponent("swTank", 0),
    ],
    wires: [
      wire("src", "pos", "swChg", "a"),
      wire("swChg", "b", "rchg", "a"),
      wire("rchg", "b", "c", "a"),
      wire("c", "b", "src", "neg"),
      wire("c", "a", "l", "a"),
      wire("l", "b", "swTank", "a"),
      wire("swTank", "b", "c", "b"),
    ],
  };
}

function runTankRing(method: "be" | "trap"): Sample[] {
  const engine = new SimEngine();
  engine.setIntegrationMethod(method);
  const circuit = tankCircuit();
  engine.load(circuit);

  // 200 x 5 us = 100 charge time constants (tau = 10 ohm x 1 uF): the cap
  // reaches the rail to machine precision before the tank is released.
  for (let index = 0; index < 200; index++) {
    acceptedStep(engine, 5e-6, `tank ${method} charge step ${String(index)}`);
  }
  expect(Math.abs(componentVoltage(engine, "c", "a", "b") - 5)).toBeLessThanOrEqual(1e-3);

  // Both switch flips happen at one load() boundary; load() forces the next
  // step onto backward Euler, which is exactly the documented discontinuity
  // re-anchor for the trap history.
  setSwitchClosed(circuit, "swChg", 0);
  setSwitchClosed(circuit, "swTank", 1);
  engine.load(circuit);

  const samples: Sample[] = [];
  for (let index = 0; index < TANK_RING_STEPS; index++) {
    acceptedStep(engine, TANK_H, `tank ${method} ring step ${String(index)}`);
    samples.push({ t: engine.simTime, v: componentVoltage(engine, "c", "a", "b") });
  }
  return samples;
}

describe("trapezoidal integration — LC tank amplitude conservation", () => {
  /**
   * Validity envelope: 5 V initial capacitor energy released into a 10 mH ||
   * 1 uF tank (f0 ≈ 1.59 kHz) at h = T/100, losses limited to the 1 mOhm
   * closed contact (0.03% over the window) and 1 Tohm shunts. Peak-to-peak
   * amplitude compares cycle 1 against cycle 10, so discrete peak flattening
   * cancels. Acceptance: trap loses < 1%; BE — whose numerical loss is
   * exp(-(omega·h)^2/2) per step, about 80% over the same window — loses > 50%.
   */
  it("trap keeps peak-to-peak loss under 1% over 10 cycles where BE loses over half", () => {
    const trapSamples = runTankRing("trap");
    const trapFirst = peakToPeak(trapSamples, 0, 100);
    const trapLast = peakToPeak(trapSamples, 900, 1000);
    expect(trapFirst).toBeGreaterThan(9); // sanity: the tank actually rings at ~±5 V
    const trapLoss = 1 - trapLast / trapFirst;
    expect(trapLoss, `trap pp first=${String(trapFirst)} last=${String(trapLast)}`).toBeLessThan(0.01);

    const beSamples = runTankRing("be");
    const beFirst = peakToPeak(beSamples, 0, 100);
    const beLast = peakToPeak(beSamples, 900, 1000);
    const beLoss = 1 - beLast / beFirst;
    expect(beLoss, `BE pp first=${String(beFirst)} last=${String(beLast)}`).toBeGreaterThan(0.5);
  });
});

// ─── First-order reference envelopes ─────────────────────────────────────────

describe("trapezoidal integration — first-order reference envelopes", () => {
  /**
   * Validity envelope: identical to the physics-reference-benchmark RC step
   * (ideal lumped R and C, 0 -> 5 V step, reference Vc(t)=V+(V0-V)exp(-t/RC))
   * with the SAME h = tau/100 budget of 0.3% full-scale. Trap must remain
   * inside the envelope the default integrator already meets — the budget is
   * reused, not retuned.
   */
  it("RC step at h=tau/100 stays within the 0.3% full-scale benchmark budget", () => {
    const resistance = 1_000;
    const capacitance = 10e-6;
    const supply = 5;
    const tau = resistance * capacitance;
    const h = tau / 100;
    const circuit: SimCircuit = {
      components: [voltageSource("src", 0), resistor("r", resistance), capacitor("c", capacitance)],
      wires: [
        wire("src", "pos", "r", "a"),
        wire("r", "b", "c", "a"),
        wire("c", "b", "src", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(circuit);
    acceptedStep(engine, h, "trap RC zero-input initialization");
    const initialVoltage = componentVoltage(engine, "c", "a", "b");

    setSourceVoltage(circuit, "src", supply);
    engine.load(circuit);
    for (let index = 0; index < 100; index++) {
      acceptedStep(engine, h, `trap RC tau step ${String(index)}`);
    }

    const expected = supply + (initialVoltage - supply) * Math.exp(-1);
    const actual = componentVoltage(engine, "c", "a", "b");
    const fullScale = Math.abs(supply - initialVoltage);
    expect(
      Math.abs(actual - expected),
      `trap RC: actual=${String(actual)}, reference=${String(expected)}`,
    ).toBeLessThanOrEqual(0.003 * fullScale);
  });

  /**
   * Validity envelope: identical to the physics-reference-benchmark RL step
   * (ideal lumped R and L, 0 -> 1 V step, reference
   * I(t)=Iinf+(I0-Iinf)exp(-tR/L)) with the SAME h = tau/100 budget of 0.3%
   * full-scale current.
   */
  it("RL step at h=tau/100 stays within the 0.3% full-scale benchmark budget", () => {
    const resistance = 10;
    const inductance = 10e-3;
    const supply = 1;
    const tau = inductance / resistance;
    const h = tau / 100;
    const steadyCurrent = supply / resistance;
    const circuit: SimCircuit = {
      components: [voltageSource("src", 0), resistor("r", resistance), inductor("l", inductance)],
      wires: [
        wire("src", "pos", "r", "a"),
        wire("r", "b", "l", "a"),
        wire("l", "b", "src", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(circuit);
    acceptedStep(engine, h, "trap RL zero-input initialization");
    const initialCurrent = engine.getElementI().l ?? Number.NaN;
    expect(Number.isFinite(initialCurrent)).toBe(true);

    setSourceVoltage(circuit, "src", supply);
    engine.load(circuit);
    for (let index = 0; index < 100; index++) {
      acceptedStep(engine, h, `trap RL tau step ${String(index)}`);
    }

    const expected = steadyCurrent + (initialCurrent - steadyCurrent) * Math.exp(-1);
    const actual = engine.getElementI().l ?? Number.NaN;
    expect(
      Math.abs(actual - expected),
      `trap RL: actual=${String(actual)}, reference=${String(expected)}`,
    ).toBeLessThanOrEqual(0.003 * Math.abs(steadyCurrent - initialCurrent));
  });
});

// ─── Square-wave discontinuity handling ──────────────────────────────────────

const SQ_R = 1_000;
const SQ_C = 100e-9; // tau = 100 us
const SQ_FREQUENCY = 1_000;
const SQ_HALF_PERIOD = 1 / (2 * SQ_FREQUENCY); // 500 us = 5 tau per level
const SQ_LOW = 0;
const SQ_HIGH = 5;
// Deliberately does not divide the half period, so the event clamp below is
// exercised on every edge instead of steps silently landing on boundaries.
const SQ_BASE_H = 3e-5;
// Mirrors the worker's H_MIN guard: land one guard step before the edge, then
// cross the jump inside a dedicated 10 ns interval.
const SQ_GUARD = 1e-8;
const SQ_EDGES = 8; // four full periods

function squareRcCircuit(): SimCircuit {
  return {
    components: [
      squareGenerator("gen", SQ_LOW, SQ_HIGH, SQ_FREQUENCY),
      resistor("r", SQ_R),
      capacitor("c", SQ_C),
    ],
    wires: [
      wire("gen", "pos", "r", "a"),
      wire("r", "b", "c", "a"),
      wire("c", "b", "gen", "neg"),
    ],
  };
}

describe("trapezoidal integration — square-wave discontinuity handling", () => {
  /**
   * Validity envelope: ideal square generator (0/5 V, 1 kHz, 50% duty) into
   * R=1k, C=100 nF, stepped with the worker's event protocol: clamp to land
   * one 10 ns guard before each edge, cross the edge in a 10 ns step, and
   * markDiscontinuity() after every clamped step so both the jump-crossing
   * step and the first post-boundary step integrate with backward Euler.
   * Reference behavior: a first-order RC driven by a level change relaxes
   * monotonically — any post-edge sample moving AWAY from the asymptote or
   * crossing it is trapezoidal ringing leaking through the BE-at-events rule.
   * Acceptance: within every half-period segment, |v - target| is
   * non-increasing, v never changes sides (1 nV float allowance), and every
   * sample tracks the analytic exponential from the segment's first sample
   * within 4% of the segment amplitude. The 4% budget is dominated by the
   * protocol's single first-order BE anchor step at h = 0.3·tau (2.9%
   * measured); a corrupted trap history at an edge would fling the first
   * post-edge samples far outside it.
   */
  it("trap mode with the BE-at-events protocol relaxes monotonically after every edge", () => {
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(squareRcCircuit());

    const endTime = SQ_EDGES * SQ_HALF_PERIOD;
    const samples: Array<{ segment: number; t: number; v: number }> = [];
    let edgeIndex = 1;
    let stepCount = 0;
    while (engine.simTime < endTime - SQ_GUARD && stepCount < 10_000) {
      stepCount += 1;
      const nextEdge = edgeIndex * SQ_HALF_PERIOD;
      const untilEvent = nextEdge - engine.simTime;
      let h = SQ_BASE_H;
      let clamped = false;
      if (untilEvent < h) {
        h = untilEvent > SQ_GUARD * 1.5 ? untilEvent - SQ_GUARD : Math.max(SQ_GUARD, untilEvent);
        clamped = true;
      }
      acceptedStep(engine, h, `square-RC step ${String(stepCount)} (t=${String(engine.simTime)})`);
      if (clamped) engine.markDiscontinuity();
      if (nextEdge - engine.simTime <= SQ_GUARD / 2) edgeIndex += 1;
      samples.push({
        // Segment k spans [k*halfPeriod, (k+1)*halfPeriod); the sample that
        // lands exactly ON an edge starts the new segment because the source
        // already reports the new level at the boundary instant.
        segment: edgeIndex - 1,
        t: engine.simTime,
        v: componentVoltage(engine, "c", "a", "b"),
      });
    }

    const segments = new Map<number, Array<{ t: number; v: number }>>();
    for (const sample of samples) {
      const list = segments.get(sample.segment) ?? [];
      list.push({ t: sample.t, v: sample.v });
      segments.set(sample.segment, list);
    }

    for (let segment = 0; segment < SQ_EDGES; segment++) {
      const list = segments.get(segment);
      expect(list, `segment ${String(segment)} must have samples`).toBeDefined();
      expect(list!.length).toBeGreaterThanOrEqual(10);
      const target = segment % 2 === 0 ? SQ_HIGH : SQ_LOW;
      const tau = SQ_R * SQ_C;
      const start = list![0];
      const startSide = Math.sign(start.v - target);
      const amplitude = Math.abs(start.v - target);
      let previousDistance = amplitude;
      for (let index = 1; index < list!.length; index++) {
        const point = list![index];
        const distance = Math.abs(point.v - target);
        const analytic = target + (start.v - target) * Math.exp(-(point.t - start.t) / tau);
        expect(
          Math.abs(point.v - analytic) <= 0.04 * amplitude,
          `segment ${String(segment)} t=${String(point.t)}: v=${String(point.v)} deviates from ` +
          `analytic ${String(analytic)} by more than 4% of the ${String(amplitude)} V segment amplitude`,
        ).toBe(true);
        expect(
          distance <= previousDistance + 1e-9,
          `segment ${String(segment)} t=${String(point.t)}: |v-target| grew from ` +
          `${String(previousDistance)} to ${String(distance)} — post-edge oscillation artifact`,
        ).toBe(true);
        expect(
          Math.sign(point.v - target) === startSide || distance <= 1e-9,
          `segment ${String(segment)} t=${String(point.t)}: v=${String(point.v)} crossed the ` +
          `${String(target)} V asymptote — post-edge oscillation artifact`,
        ).toBe(true);
        previousDistance = distance;
      }
    }
  });
});

// ─── Stiff square-wave discontinuity handling ────────────────────────────────

// Same 1 kHz square protocol as above but with tau = R*C = 1 us against
// h = 30 us (h/tau = 30). This is the regime the gentle fixture above cannot
// reach: for h < 2*tau the trapezoidal amplification factor is positive and
// CANNOT oscillate no matter how corrupt the history is, so only a stiff
// ratio can falsify the BE-at-events machinery. Here an unprotected trap step
// across an edge rings the capacitor to ~4.8 V outside the rails and the
// residue decays by only (h/2tau - 1)/(h/2tau + 1) = 0.875 per step, while
// the protocol caps the post-edge residue at 5 V/(1 + h/tau) ~ 0.16 V.
const STIFF_R = 1_000;
const STIFF_C = 1e-9; // tau = 1 us
const STIFF_FREQUENCY = 1_000;
const STIFF_HALF_PERIOD = 1 / (2 * STIFF_FREQUENCY);
const STIFF_LOW = 0;
const STIFF_HIGH = 5;
const STIFF_BASE_H = 3e-5;
const STIFF_GUARD = 1e-8;
const STIFF_EDGES = 8;

function stiffSquareRcCircuit(): SimCircuit {
  return {
    components: [
      squareGenerator("gen", STIFF_LOW, STIFF_HIGH, STIFF_FREQUENCY),
      resistor("r", STIFF_R),
      capacitor("c", STIFF_C),
    ],
    wires: [
      wire("gen", "pos", "r", "a"),
      wire("r", "b", "c", "a"),
      wire("c", "b", "gen", "neg"),
    ],
  };
}

describe("trapezoidal integration — stiff square-wave discontinuity handling", () => {
  /**
   * Validity envelope: ideal square generator (0/5 V, 1 kHz) into R=1k,
   * C=1 nF, worker event protocol (land-before clamp, 10 ns guard step,
   * markDiscontinuity after every clamped step), h = 30·tau. At this
   * stiffness the step cannot resolve the exponential, so monotone tracking
   * is out of scope; the protocol's contract is bounded artifacts. Reference
   * bounds (measured, with analytic backing): the single BE re-anchor at
   * h = 30·tau leaves at most 5/(1+30) ~ 0.16 V of residue, ringing decays
   * 0.875x per step, so every post-edge sample stays within 0.5 V of the
   * target and the residue is below 0.05 V by each half-period's end
   * (protected: 0.159 V / 0.018 V measured). Removing markDiscontinuity
   * flings samples 4.8 V outside the rails and leaves 0.53 V at segment end,
   * so BOTH bounds independently catch a dead BE-at-events path — this is the
   * discriminating fixture the gentle tau = 100 us test cannot provide.
   */
  it("trap mode keeps post-edge artifacts bounded at h = 30x tau", () => {
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(stiffSquareRcCircuit());

    const endTime = STIFF_EDGES * STIFF_HALF_PERIOD;
    const segments = new Map<number, Array<{ t: number; v: number }>>();
    let edgeIndex = 1;
    let stepCount = 0;
    while (engine.simTime < endTime - STIFF_GUARD && stepCount < 10_000) {
      stepCount += 1;
      const nextEdge = edgeIndex * STIFF_HALF_PERIOD;
      const untilEvent = nextEdge - engine.simTime;
      let h = STIFF_BASE_H;
      let clamped = false;
      if (untilEvent < h) {
        h = untilEvent > STIFF_GUARD * 1.5 ? untilEvent - STIFF_GUARD : Math.max(STIFF_GUARD, untilEvent);
        clamped = true;
      }
      acceptedStep(engine, h, `stiff square-RC step ${String(stepCount)} (t=${String(engine.simTime)})`);
      if (clamped) engine.markDiscontinuity();
      if (nextEdge - engine.simTime <= STIFF_GUARD / 2) edgeIndex += 1;
      const segment = edgeIndex - 1;
      const list = segments.get(segment) ?? [];
      list.push({ t: engine.simTime, v: componentVoltage(engine, "c", "a", "b") });
      segments.set(segment, list);
    }

    for (let segment = 0; segment < STIFF_EDGES; segment++) {
      const list = segments.get(segment);
      expect(list, `stiff segment ${String(segment)} must have samples`).toBeDefined();
      expect(list!.length).toBeGreaterThanOrEqual(10);
      const target = segment % 2 === 0 ? STIFF_HIGH : STIFF_LOW;
      // Sample 0 lands ON the edge, where the full 5 V distance to the new
      // target is physical, not an artifact; every later sample is post-anchor.
      for (let index = 1; index < list!.length; index++) {
        const point = list![index];
        expect(
          Math.abs(point.v - target) <= 0.5,
          `stiff segment ${String(segment)} t=${String(point.t)}: v=${String(point.v)} is more than ` +
          `0.5 V from the ${String(target)} V target — trap history rang across the edge`,
        ).toBe(true);
      }
      const last = list![list!.length - 1];
      expect(
        Math.abs(last.v - target) <= 0.05,
        `stiff segment ${String(segment)} end t=${String(last.t)}: v=${String(last.v)} has not ` +
        `settled to ${String(target)} V — post-edge ringing is not decaying`,
      ).toBe(true);
    }
  });
});

// ─── Default-mode isolation ──────────────────────────────────────────────────

/**
 * Steps both engines in lockstep and requires bitwise-identical netV at every
 * step, not just the last: any single-step drift would prove the default path
 * is no longer byte-identical to explicit backward Euler.
 */
function expectBitwiseLockstep(
  makeCircuit: () => SimCircuit,
  steps: number,
  h: number,
  label: string,
): void {
  const defaultEngine = new SimEngine();
  const explicitEngine = new SimEngine();
  // Toggling through trap and back is the strongest form of "explicitly set
  // to be": any state a mode switch leaked would break bit-identity here.
  explicitEngine.setIntegrationMethod("trap");
  explicitEngine.setIntegrationMethod("be");
  defaultEngine.load(makeCircuit());
  explicitEngine.load(makeCircuit());

  const mismatches: string[] = [];
  for (let index = 0; index < steps; index++) {
    defaultEngine.step(h);
    explicitEngine.step(h);
    expect(defaultEngine.lastConverged, `${label} default step ${String(index)}`).toBe(true);
    expect(explicitEngine.lastConverged, `${label} explicit-be step ${String(index)}`).toBe(true);
    const defaultNetV = defaultEngine.getNetV();
    const explicitNetV = explicitEngine.getNetV();
    const keys = new Set([...Object.keys(defaultNetV), ...Object.keys(explicitNetV)]);
    for (const key of keys) {
      if (!Object.is(defaultNetV[key], explicitNetV[key])) {
        mismatches.push(
          `${label} step ${String(index)} net ${key}: `
          + `default=${String(defaultNetV[key])} explicit-be=${String(explicitNetV[key])}`,
        );
      }
    }
    if (mismatches.length > 8) break; // enough evidence; keep the failure readable
  }
  expect(mismatches).toEqual([]);
}

describe("trapezoidal integration — default-mode isolation", () => {
  /**
   * Validity envelope: bit-identity, not physics — the default engine (no
   * setIntegrationMethod call) must produce EXACTLY the numbers of an engine
   * explicitly set to "be", for every net at every step. Acceptance: zero
   * bitwise mismatches (Object.is) over the full run of each circuit.
   */
  it("series RLC under default load is bitwise identical to explicit be", () => {
    expectBitwiseLockstep(seriesRlcCircuit, 300, RLC_H, "RLC");
  });

  it("square-driven RC under default load is bitwise identical to explicit be", () => {
    expectBitwiseLockstep(squareRcCircuit, 250, 2e-5, "square-RC");
  });
});

// ─── Snapshot rollback ───────────────────────────────────────────────────────

describe("trapezoidal integration — snapshot rollback", () => {
  /**
   * Validity envelope: deterministic replay — the worker's adaptive-step
   * rollback restores a snapshot and re-runs the same intervals, so a trap
   * snapshot must round-trip capsI/indsV exactly and a restored engine must
   * retrace the identical bitwise trajectory. Acceptance: Object.is equality
   * on the round-tripped history maps, and zero bitwise netV/elementI
   * mismatches across a 120-step replay taken mid-ring.
   */
  it("saveState/restoreState round-trips capsI/indsV and replays bitwise", () => {
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(seriesRlcCircuit());
    for (let index = 0; index < 150; index++) {
      acceptedStep(engine, RLC_H, `rollback warmup step ${String(index)}`);
    }

    const snapshot = engine.saveState();
    expect(snapshot.capsI).toBeInstanceOf(Map);
    expect(snapshot.indsV).toBeInstanceOf(Map);
    const capHistory = snapshot.capsI!.get("c");
    const indHistory = snapshot.indsV!.get("l");
    // Mid-ring both histories carry live oscillation values, so definedness
    // and finiteness prove the snapshot actually captured trap state.
    expect(capHistory !== undefined && Number.isFinite(capHistory)).toBe(true);
    expect(indHistory !== undefined && Number.isFinite(indHistory)).toBe(true);

    const replaySteps = 120;
    const recordedNetV: Array<Record<string, number>> = [];
    const recordedElementI: Array<Record<string, number>> = [];
    for (let index = 0; index < replaySteps; index++) {
      acceptedStep(engine, RLC_H, `rollback record step ${String(index)}`);
      recordedNetV.push({ ...engine.getNetV() });
      recordedElementI.push({ ...engine.getElementI() });
    }

    engine.restoreState(snapshot);
    expect(engine.simTime).toBe(snapshot.simTime);

    // Immediate re-save must reproduce the identical trap histories.
    const roundTrip = engine.saveState();
    expect(roundTrip.capsI!.size).toBe(snapshot.capsI!.size);
    expect(roundTrip.indsV!.size).toBe(snapshot.indsV!.size);
    for (const [id, value] of snapshot.capsI!) {
      expect(Object.is(roundTrip.capsI!.get(id), value), `capsI[${id}] round-trip`).toBe(true);
    }
    for (const [id, value] of snapshot.indsV!) {
      expect(Object.is(roundTrip.indsV!.get(id), value), `indsV[${id}] round-trip`).toBe(true);
    }

    const mismatches: string[] = [];
    for (let index = 0; index < replaySteps; index++) {
      acceptedStep(engine, RLC_H, `rollback replay step ${String(index)}`);
      const netV = engine.getNetV();
      for (const key of Object.keys(recordedNetV[index])) {
        if (!Object.is(netV[key], recordedNetV[index][key])) {
          mismatches.push(
            `replay step ${String(index)} net ${key}: `
            + `${String(netV[key])} vs recorded ${String(recordedNetV[index][key])}`,
          );
        }
      }
      const elementI = engine.getElementI();
      for (const key of Object.keys(recordedElementI[index])) {
        if (!Object.is(elementI[key], recordedElementI[index][key])) {
          mismatches.push(
            `replay step ${String(index)} elementI ${key}: `
            + `${String(elementI[key])} vs recorded ${String(recordedElementI[index][key])}`,
          );
        }
      }
      if (mismatches.length > 8) break;
    }
    expect(mismatches).toEqual([]);
  });
});

// ─── Solver convergence health ───────────────────────────────────────────────

describe("trapezoidal integration — solver convergence health", () => {
  /**
   * Validity envelope: every trap-mode fixture above already routes through
   * acceptedStep(), which fails on the first non-converged, singular, or
   * ill-conditioned step. This test makes the aggregate claim explicit: a
   * long trap run over the stiffest fixture (the near-lossless ring) must
   * accept every single step with clean diagnostics and at least one Newton
   * iteration of real work. Acceptance: 1000/1000 converged steps.
   */
  it("every trap-mode step converges with clean solver diagnostics", () => {
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(seriesRlcCircuit());

    let convergedSteps = 0;
    for (let index = 0; index < 1000; index++) {
      acceptedStep(engine, RLC_H, `health step ${String(index)}`);
      expect(engine.lastIters, `health step ${String(index)} iterations`).toBeGreaterThanOrEqual(1);
      if (engine.lastConverged) convergedSteps += 1;
    }
    expect(convergedSteps).toBe(1000);
  });
});
