/**
 * Integration tests for runAcSweep() against the real SimEngine.
 *
 * WHY integration (not unit):
 *   The AC sweep runner's correctness guarantee is "the magnitude and phase at
 *   each frequency match the known analytical response of the test circuit".
 *   Mocking the engine would test that we call it, not that the DFT math gives
 *   the right answer for a real RC filter.  These tests run against the real
 *   MNA engine — they are slower but catch real DFT errors, wrong reference
 *   net resolution, and step-size bugs.
 *
 * Tolerances are deliberately loose:
 *   +/- 1.5 dB for magnitude, +/- 8 deg for phase at the corner frequency.
 *   The sine-sweep approach is teaching-grade, not SPICE-grade — the DFT uses
 *   only 5 measure cycles, which introduces some spectral leakage.  Looser
 *   tolerances survive that without hiding factor-of-10 errors.
 *
 * Circuit fixture conventions follow run-sweep.test.ts:
 *   - SimCircuit (not breadboard Circuit) built directly.
 *   - Capacitor kind "capacitor", pins "a"/"b", param "capacitance".
 *   - Signal gen kind "signal_gen", pins "pos"/"neg".
 *   - No position/rotation — the engine does not need them.
 */

import { describe, expect, it, vi } from "vitest";
import { runAcSweep, runAcSweepAsync } from "../../src/analysis/run-ac-sweep.js";
import { validateJobSpec } from "../../src/analysis/jobs.js";
import type { AcSweepSpec } from "../../src/analysis/jobs.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";

// ─── Helper: find net id after cold-loading ───────────────────────────────────

function findNetId(circuit: SimCircuit, compId: string, pinId: string): string {
  const engine = new SimEngine();
  engine.coldLoad(circuit);
  const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === compId && p === pinId));
  if (!net) throw new Error(`No net found for ${compId}.${pinId}`);
  return net.id;
}

// ─── Circuit builders ─────────────────────────────────────────────────────────

/**
 * RC low-pass filter:
 *   signal_gen (rSource=0 for clean drive) → resistor R → capacitor C to gnd.
 *   Probe: capacitor node (voltage across C).
 *   Analytical: H(f) = 1 / sqrt(1 + (f/fc)^2), fc = 1/(2π·R·C).
 *   With R=1k, C=1µF: fc = 1/(2π·1000·1e-6) ≈ 159.15 Hz.
 */
function rcLowPassCircuit(R = 1000, C = 1e-6): SimCircuit {
  return {
    components: [
      {
        id: "sg1",
        kind: "signal_gen",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 0, enabled: 1 },
      },
      {
        id: "r1",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: R },
      },
      {
        id: "c1",
        kind: "capacitor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { capacitance: C },
      },
    ],
    wires: [
      // sg1.pos → r1.a
      { from_component: "sg1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      // r1.b → c1.a  (probe node)
      { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "a" },
      // c1.b → sg1.neg (gnd)
      { from_component: "c1", from_pin: "b", to_component: "sg1", to_pin: "neg" },
    ],
  };
}

/**
 * RC high-pass filter:
 *   signal_gen → capacitor C → resistor R to gnd.
 *   Probe: resistor node (voltage across R).
 *   Analytical: H(f) = (f/fc) / sqrt(1 + (f/fc)^2), same fc.
 *   With R=1k, C=1µF: fc ≈ 159.15 Hz.
 */
function rcHighPassCircuit(R = 1000, C = 1e-6): SimCircuit {
  return {
    components: [
      {
        id: "sg1",
        kind: "signal_gen",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 0, enabled: 1 },
      },
      {
        id: "c1",
        kind: "capacitor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { capacitance: C },
      },
      {
        id: "r1",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: R },
      },
    ],
    wires: [
      // sg1.pos → c1.a
      { from_component: "sg1", from_pin: "pos", to_component: "c1", to_pin: "a" },
      // c1.b → r1.a  (probe node)
      { from_component: "c1", from_pin: "b", to_component: "r1", to_pin: "a" },
      // r1.b → sg1.neg (gnd)
      { from_component: "r1", from_pin: "b", to_component: "sg1", to_pin: "neg" },
    ],
  };
}

/**
 * Resistive divider: signal_gen → R_top → R_bot to gnd.
 *   Probe: mid-node. Analytical: H(f) = R_bot/(R_top+R_bot) = 0.5, flat, 0 deg phase.
 *   rSource=0 so we don't need to correct for source impedance.
 */
function resistiveDividerCircuit(R = 1000): SimCircuit {
  return {
    components: [
      {
        id: "sg1",
        kind: "signal_gen",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 0, enabled: 1 },
      },
      {
        id: "r_top",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: R },
      },
      {
        id: "r_bot",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: R },
      },
    ],
    wires: [
      { from_component: "sg1",   from_pin: "pos", to_component: "r_top", to_pin: "a" },
      { from_component: "r_top", from_pin: "b",   to_component: "r_bot", to_pin: "a" },
      { from_component: "r_bot", from_pin: "b",   to_component: "sg1",   to_pin: "neg" },
    ],
  };
}

// ─── Corner frequency for RC filter ──────────────────────────────────────────

const R = 1000;
const C = 1e-6;
const FC = 1 / (2 * Math.PI * R * C); // ≈ 159.15 Hz

// ─── Helper: find the point index nearest to a target frequency ───────────────

function nearestPointIndex(fHz: number[], target: number): number {
  let best = 0;
  let bestDist = Math.abs(fHz[0]! - target);
  for (let i = 1; i < fHz.length; i++) {
    const d = Math.abs(fHz[i]! - target);
    if (d < bestDist) { best = i; bestDist = d; }
  }
  return best;
}

// ─── Test a: RC low-pass magnitude and phase ──────────────────────────────────

describe("rc low-pass: magnitude and phase at corner frequency", () => {
  it("magnitude near fc is within +-1.5 dB of -3 dB", () => {
    const circuit = rcLowPassCircuit();
    // Probe at the capacitor node (after the resistor).
    const probeNet = findNetId(circuit, "c1", "a");

    // Use 21 log-spaced points from 10 Hz to 10*fc so we have good coverage.
    const toHz = Math.round(FC * 10);
    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 10,
      toHz,
      points: 21,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    expect(result.cancelled).toBe(false);

    const fcIdx = nearestPointIndex(result.fHz, FC);
    const magDb = result.outputs[0]!.magnitudeDb[fcIdx]!;

    // Ideal -3 dB at fc; allow +-1.5 dB for DFT approximation error.
    expect(magDb).toBeGreaterThan(-3 - 1.5);
    expect(magDb).toBeLessThan(-3 + 1.5);
  });

  it("magnitude at ~10*fc is <= -15 dB (well into stopband)", () => {
    const circuit = rcLowPassCircuit();
    const probeNet = findNetId(circuit, "c1", "a");

    const tenFc = FC * 10;
    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: Math.max(10, Math.round(FC * 2)),
      toHz: Math.min(100000, Math.round(tenFc * 1.5)),
      points: 11,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    const tenFcIdx = nearestPointIndex(result.fHz, tenFc);
    expect(result.outputs[0]!.magnitudeDb[tenFcIdx]!).toBeLessThan(-15);
  });

  it("phase at fc is within +-8 deg of -45 deg", () => {
    const circuit = rcLowPassCircuit();
    const probeNet = findNetId(circuit, "c1", "a");

    const toHz = Math.round(FC * 10);
    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 10,
      toHz,
      points: 21,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    const fcIdx = nearestPointIndex(result.fHz, FC);
    const phaseDeg = result.outputs[0]!.phaseDeg[fcIdx]!;

    // Ideal phase at fc = -45 deg; allow +-8 deg.
    expect(phaseDeg).toBeGreaterThan(-45 - 8);
    expect(phaseDeg).toBeLessThan(-45 + 8);
  });
});

// ─── Test b: RC high-pass magnitude ──────────────────────────────────────────

describe("rc high-pass: magnitude at corner and below", () => {
  it("magnitude at fc is within +-1.5 dB of -3 dB", () => {
    const circuit = rcHighPassCircuit();
    // Probe at r1.a = the node between the capacitor and the resistor.
    const probeNet = findNetId(circuit, "r1", "a");

    const toHz = Math.round(FC * 10);
    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 10,
      toHz,
      points: 21,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    const fcIdx = nearestPointIndex(result.fHz, FC);
    const magDb = result.outputs[0]!.magnitudeDb[fcIdx]!;

    expect(magDb).toBeGreaterThan(-3 - 1.5);
    expect(magDb).toBeLessThan(-3 + 1.5);
  });

  it("magnitude at fc/10 is <= -15 dB (well into stopband)", () => {
    const circuit = rcHighPassCircuit();
    const probeNet = findNetId(circuit, "r1", "a");

    const tenthFc = FC / 10; // ~15.9 Hz
    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 10,
      toHz: Math.round(FC * 2),
      points: 21,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    const tenthFcIdx = nearestPointIndex(result.fHz, tenthFc);
    expect(result.outputs[0]!.magnitudeDb[tenthFcIdx]!).toBeLessThan(-15);
  });
});

// ─── Test c: resistive divider (flat response) ────────────────────────────────

describe("resistive divider: flat magnitude, near-zero phase", () => {
  it("magnitude is 20*log10(0.5) ± 0.5 dB across the sweep", () => {
    const circuit = resistiveDividerCircuit();
    const probeNet = findNetId(circuit, "r_bot", "a");

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 100,
      toHz: 10000,
      points: 11,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    expect(result.cancelled).toBe(false);

    const expectedDb = 20 * Math.log10(0.5); // ≈ -6.02 dB
    for (let i = 0; i < result.fHz.length; i++) {
      expect(result.outputs[0]!.magnitudeDb[i]!).toBeGreaterThan(expectedDb - 0.5);
      expect(result.outputs[0]!.magnitudeDb[i]!).toBeLessThan(expectedDb + 0.5);
    }
  });

  it("phase is within +-3 deg of 0 across the sweep", () => {
    const circuit = resistiveDividerCircuit();
    const probeNet = findNetId(circuit, "r_bot", "a");

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 100,
      toHz: 10000,
      points: 11,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    for (let i = 0; i < result.fHz.length; i++) {
      expect(Math.abs(result.outputs[0]!.phaseDeg[i]!)).toBeLessThan(3);
    }
  });
});

// ─── Test c2: high-Q resonance settle detection ───────────────────────────────

/**
 * Series RLC band-pass: signal_gen → L → C → R to gnd, probe across R.
 *   At resonance f0 = 1/(2π√(LC)) the L and C impedances cancel, so |H| → 1
 *   (V across R ≈ V source).  Q = (1/R)·√(L/C).
 *   With L≈25.3 mH, C=1µF, R=5Ω: f0 ≈ 1000 Hz, Q ≈ 32 — high enough that the
 *   resonance builds up over many cycles, so the fixed 5-cycle settle under-
 *   reports the peak.  The runner must flag those points notSettled.
 */
function seriesRlcBandpass(L = 0.02533, C = 1e-6, Rohm = 5): SimCircuit {
  return {
    components: [
      {
        id: "sg1",
        kind: "signal_gen",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 0, enabled: 1 },
      },
      { id: "l1", kind: "inductor", pins: [{ id: "a" }, { id: "b" }], params: { inductance: L } },
      { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: C } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: Rohm } },
    ],
    wires: [
      { from_component: "sg1", from_pin: "pos", to_component: "l1", to_pin: "a" },
      { from_component: "l1", from_pin: "b", to_component: "c1", to_pin: "a" },
      { from_component: "c1", from_pin: "b", to_component: "r1", to_pin: "a" }, // probe node
      { from_component: "r1", from_pin: "b", to_component: "sg1", to_pin: "neg" },
    ],
  };
}

describe("high-Q resonance: settle detection", () => {
  it("flags not-settled points near the resonant peak", () => {
    const circuit = seriesRlcBandpass();
    const probeNet = findNetId(circuit, "r1", "a");

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 300,
      toHz: 3000,
      points: 21,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    expect(result.cancelled).toBe(false);
    expect(result.notSettled).toHaveLength(result.fHz.length);

    // The resonance builds over more cycles than the fixed settle, so at least
    // one point (near f0 ≈ 1 kHz) is flagged as not fully settled.
    expect(result.notSettled.some(Boolean)).toBe(true);

    // The under-report manifests as the band-pass peak reading below the ideal
    // 0 dB it would reach if fully settled.
    const peakDb = Math.max(...result.outputs[0]!.magnitudeDb);
    expect(peakDb).toBeLessThan(0);
  });

  it("does not flag a settled first-order RC filter (no false positives)", () => {
    const circuit = rcLowPassCircuit();
    const probeNet = findNetId(circuit, "c1", "a");

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 10,
      toHz: Math.round(FC * 10),
      points: 21,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    expect(result.notSettled.every((s) => s === false)).toBe(true);
  });

  it("does not flag a resistive divider (no false positives)", () => {
    const circuit = resistiveDividerCircuit();
    const probeNet = findNetId(circuit, "r_bot", "a");

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 100,
      toHz: 10000,
      points: 11,
      outputs: [{ netId: probeNet }],
    };

    const result = runAcSweep(spec, circuit);
    expect(result.notSettled.every((s) => s === false)).toBe(true);
  });
});

// ─── Test d: determinism ──────────────────────────────────────────────────────

describe("determinism", () => {
  it("two identical runs produce identical output arrays", () => {
    const circuit = rcLowPassCircuit();
    const probeNet = findNetId(circuit, "c1", "a");

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 50,
      toHz: 2000,
      points: 7,
      outputs: [{ netId: probeNet }],
    };

    const r1 = runAcSweep(spec, circuit);
    const r2 = runAcSweep(spec, circuit);

    expect(r1.fHz).toEqual(r2.fHz);
    expect(r1.outputs[0]!.magnitudeDb).toEqual(r2.outputs[0]!.magnitudeDb);
    expect(r1.outputs[0]!.phaseDeg).toEqual(r2.outputs[0]!.phaseDeg);
  });
});

// ─── Test e: cancellation ─────────────────────────────────────────────────────

describe("cancellation", () => {
  it("shouldCancel after k points truncates the result to k points", () => {
    const circuit = resistiveDividerCircuit();
    const probeNet = findNetId(circuit, "r_bot", "a");

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 100,
      toHz: 10000,
      points: 9,
      outputs: [{ netId: probeNet }],
    };

    let pointCount = 0;
    const result = runAcSweep(spec, circuit, {
      onPoint() { pointCount++; },
      shouldCancel() { return pointCount >= 3; },
    });

    expect(result.cancelled).toBe(true);
    expect(result.fHz).toHaveLength(3);
    expect(result.outputs[0]!.magnitudeDb).toHaveLength(3);
    expect(result.outputs[0]!.phaseDeg).toHaveLength(3);
    expect(result.pointFailures).toHaveLength(3);
  });

  it("runAcSweepAsync: an external timer can flip the cancel flag mid-run", async () => {
    const circuit = resistiveDividerCircuit();
    const probeNet = findNetId(circuit, "r_bot", "a");

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 100,
      toHz: 10000,
      points: 31,
      outputs: [{ netId: probeNet }],
    };

    let cancel = false;
    const promise = runAcSweepAsync(spec, circuit, {
      shouldCancel: () => cancel,
    });
    setTimeout(() => { cancel = true; }, 30);

    const result = await promise;
    expect(result.cancelled).toBe(true);
    expect(result.fHz.length).toBeGreaterThan(0);
    expect(result.fHz.length).toBeLessThan(31);
  });
});

// ─── Test f: dead source (validation path) ────────────────────────────────────

describe("dead source", () => {
  it("validateJobSpec rejects a signal_gen with enabled=0", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "sg1",
          kind: "signal_gen",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 50, enabled: 0 },
        },
        {
          id: "r1",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: 1000 },
        },
      ],
      wires: [
        { from_component: "sg1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "sg1", to_pin: "neg" },
      ],
    };
    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 100,
      toHz: 1000,
      points: 5,
      outputs: [{ netId: "n0" }],
    };
    const err = validateJobSpec(spec, circuit);
    expect(err).toBeTruthy();
    expect(err).toMatch(/disabled/i);
  });
});

describe("AC sweep convergence safety", () => {
  it("rejects a frequency point instead of transforming a failed iterate", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "sg1",
          kind: "signal_gen",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 0, enabled: 1 },
        },
        { id: "fixed", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 2 } },
        { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      ],
      wires: [
        { from_component: "sg1", from_pin: "pos", to_component: "fixed", to_pin: "pos" },
        { from_component: "sg1", from_pin: "neg", to_component: "fixed", to_pin: "neg" },
        { from_component: "sg1", from_pin: "pos", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "sg1", to_pin: "neg" },
      ],
    };
    const outputNet = findNetId(circuit, "load", "a");
    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg1",
      fromHz: 1000,
      toHz: 1000,
      points: 2,
      outputs: [{ netId: outputNet }],
    };

    expect(() => runAcSweep(spec, circuit)).toThrow(/did not converge.*rolled back.*rejected/i);
  });
});
