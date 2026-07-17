/**
 * Tests for the worker-hostable small-signal AC sweep driver.
 *
 * The load-bearing claim is EQUIVALENCE: chunking the frequency range and
 * re-solving the operating point per chunk must produce the SAME numbers as a
 * single runSmallSignalAc() call over the whole range. If that ever drifts, the
 * driver is silently returning a different answer than the function it advertises
 * itself as a wrapper for. The other tests cover the two things the wrapper adds
 * over the bare function — progress reporting and cooperative cancellation.
 *
 * RC low-pass fixture mirrors run-ac-sweep.test.ts: fc = 1/(2π·R·C) ≈ 159 Hz for
 * R = 1k, C = 1µF. The analytic magnitude is included as a sanity floor, but the
 * primary assertion is bit-equivalence to runSmallSignalAc, which is itself
 * cross-validated against ngspice elsewhere.
 */

import { describe, expect, it, vi } from "vitest";
import {
  runSmallSignalAcSweep,
  runSmallSignalAcSweepAsync,
} from "../../src/analysis/run-small-signal-ac-sweep.js";
import { runSmallSignalAc } from "../../src/sim/engine/ac-analysis.js";
import { logSweepValues } from "../../src/analysis/jobs.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";

function rcLowPass(R = 1000, C = 1e-6): SimCircuit {
  return {
    components: [
      {
        id: "sg1",
        kind: "signal_gen",
        // AC 1 designates the small-signal input. rSource 0 for a clean drive.
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 0, enabled: 1 },
      },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R } },
      { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: C } },
    ],
    wires: [
      { from_component: "sg1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "a" },
      { from_component: "c1", from_pin: "b", to_component: "sg1", to_pin: "neg" },
    ],
  };
}

function probeNet(circuit: SimCircuit, compId: string, pinId: string): string {
  const engine = new SimEngine();
  engine.coldLoad(circuit);
  const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === compId && p === pinId));
  if (!net) throw new Error(`no net for ${compId}.${pinId}`);
  return net.id;
}

const SPEC = (out: string) =>
  ({ kind: "small-signal-ac", inputId: "sg1", fromHz: 10, toHz: 100_000, points: 40, outputNetIds: [out] }) as const;

describe("runSmallSignalAcSweep", () => {
  it("produces bit-identical results to a single runSmallSignalAc call", () => {
    const circuit = rcLowPass();
    const out = probeNet(circuit, "c1", "a");

    // The reference: one atomic call over the full log-spaced frequency set.
    const refEngine = new SimEngine();
    refEngine.load(circuit);
    const reference = runSmallSignalAc(refEngine, {
      inputId: "sg1",
      outputNetIds: [out],
      frequenciesHz: logSweepValues(10, 100_000, 40),
    });

    // The driver, which chunks and re-solves the OP between chunks.
    const swept = runSmallSignalAcSweep(SPEC(out), circuit);

    expect(swept.frequenciesHz).toEqual(reference.frequenciesHz);
    // Object.is equality per sample — the OP is deterministic, so chunking must
    // not perturb a single bit.
    for (let i = 0; i < reference.frequenciesHz.length; i++) {
      expect(swept.outputs[0]!.re[i]).toBe(reference.outputs[0]!.re[i]);
      expect(swept.outputs[0]!.im[i]).toBe(reference.outputs[0]!.im[i]);
      expect(swept.outputs[0]!.magnitudeDb[i]).toBe(reference.outputs[0]!.magnitudeDb[i]);
    }
    expect(swept.cancelled).toBe(false);
  });

  it("matches the analytic RC low-pass at the corner and the decade above", () => {
    const circuit = rcLowPass();
    const out = probeNet(circuit, "c1", "a");
    const result = runSmallSignalAcSweep(SPEC(out), circuit);

    const fc = 1 / (2 * Math.PI * 1000 * 1e-6); // ~159.15 Hz
    const nearest = (f: number) =>
      result.frequenciesHz.reduce((best, _, i) =>
        Math.abs(result.frequenciesHz[i]! - f) < Math.abs(result.frequenciesHz[best]! - f) ? i : best, 0);

    // At fc, |H| = 1/sqrt(2) = -3.01 dB. A decade above, -20 dB/decade rolloff.
    expect(result.outputs[0]!.magnitudeDb[nearest(fc)]).toBeCloseTo(-3.01, 0);
    expect(result.outputs[0]!.magnitudeDb[nearest(fc * 10)]).toBeLessThan(-18);
  });

  it("reports one progress callback per frequency, in order", () => {
    const circuit = rcLowPass();
    const out = probeNet(circuit, "c1", "a");
    const onPoint = vi.fn();
    runSmallSignalAcSweep(SPEC(out), circuit, { onPoint });

    expect(onPoint).toHaveBeenCalledTimes(40);
    // total is constant; index ascends 0..39.
    expect(onPoint.mock.calls[0]).toEqual([0, 40]);
    expect(onPoint.mock.calls[39]).toEqual([39, 40]);
  });
});

describe("runSmallSignalAcSweepAsync", () => {
  it("stops early when shouldCancel returns true, and flags it", async () => {
    const circuit = rcLowPass();
    const out = probeNet(circuit, "c1", "a");

    // Cancel after the first chunk. The result carries only the points solved
    // before the cancel, and cancelled is true.
    let seen = 0;
    const result = await runSmallSignalAcSweepAsync(SPEC(out), circuit, {
      onPoint: () => { seen++; },
      shouldCancel: () => true, // checked after the first chunk
    });

    expect(result.cancelled).toBe(true);
    expect(result.frequenciesHz.length).toBeGreaterThan(0);
    expect(result.frequenciesHz.length).toBeLessThan(40); // did not finish
    expect(seen).toBe(result.frequenciesHz.length);
  });

  it("returns the same numbers as the sync form when not cancelled", async () => {
    const circuit = rcLowPass();
    const out = probeNet(circuit, "c1", "a");
    const sync = runSmallSignalAcSweep(SPEC(out), circuit);
    const async = await runSmallSignalAcSweepAsync(SPEC(out), circuit);
    expect(async.frequenciesHz).toEqual(sync.frequenciesHz);
    expect(async.outputs[0]!.magnitudeDb).toEqual(sync.outputs[0]!.magnitudeDb);
    expect(async.cancelled).toBe(false);
  });
});
