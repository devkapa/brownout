/**
 * Integration tests for runSweep() against the real SimEngine.
 *
 * WHY integration (not unit):
 *   The sweep runner's correctness guarantee is "the engine settles to the
 *   right DC operating point per spec value".  Mocking the engine would
 *   test that we call it, not that we call it correctly.  These tests run
 *   against the real MNA engine — they are slower (~1-2 s) but catch
 *   real bugs like wrong param mutation, missing environment creation, or
 *   a tail window that averages the wrong portion of the transient.
 *
 * Circuit fixtures follow the shape in environment.test.ts (which is the
 * canonical example of how to build a small SimCircuit for engine tests):
 *   - Minimal component set (voltage source + resistor(s) + device under test)
 *   - No position/rotation — the engine does not need them
 *   - Pin ids from catalog conventions ("pos"/"neg" for sources, "a"/"b" for passives)
 *
 * Tolerances:
 *   DC divider tests use toBeCloseTo(expected, 1) — 1 decimal place = ±0.05 V.
 *   This is loose enough to survive floating-point variance in the tail
 *   average while being tight enough to catch a factor-of-2 error.
 *
 *   Monotonicity tests use inequality only — no need for absolute tolerance.
 *
 *   Signal_gen AC offset tests: mean should track offset within ±0.05 V
 *   (the ripple is filtered by tail averaging over whole cycles).
 */

import { describe, expect, it, vi } from "vitest";
import { runSweep, runSweepAsync } from "../../src/analysis/run-sweep.js";
import type { DcSweepSpec, TempSweepSpec } from "../../src/analysis/jobs.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

// ─── Circuit builder helpers ──────────────────────────────────────────────────

/** bench_psu driving a symmetric voltage divider: two equal resistors. */
function voltageDividerCircuit(psuVoltage = 5): SimCircuit {
  return {
    components: [
      {
        id: "psu1",
        kind: "bench_psu",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: psuVoltage, iLimit: 1 },
      },
      {
        id: "r_top",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
      {
        id: "r_bot",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
    ],
    wires: [
      { from_component: "psu1", from_pin: "pos", to_component: "r_top", to_pin: "a" },
      { from_component: "r_top", from_pin: "b", to_component: "r_bot", to_pin: "a" },
      { from_component: "r_bot", from_pin: "b", to_component: "psu1", to_pin: "neg" },
    ],
  };
}

/**
 * signal_gen with configurable waveform, driving a 1 MΩ probe to gnd.
 *
 * The 1 MΩ probe ensures the output net is always in the matrix even at DC.
 * rSource=50 so the open-circuit voltage tracks signalGenVoltage within <0.01%.
 */
function signalGenCircuit(params: Record<string, number | string>): SimCircuit {
  return {
    components: [
      {
        id: "sg1",
        kind: "signal_gen",
        pins: [{ id: "pos" }, { id: "neg" }],
        params,
      },
      {
        id: "probe",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1_000_000 },
      },
    ],
    wires: [
      { from_component: "sg1", from_pin: "pos", to_component: "probe", to_pin: "a" },
      { from_component: "probe", from_pin: "b", to_component: "sg1", to_pin: "neg" },
    ],
  };
}

/** NTC thermistor voltage divider against a fixed resistor, driven by 5 V. */
function thermistorDividerCircuit(tempC?: number): SimCircuit {
  const circuit: SimCircuit = {
    components: [
      {
        id: "vcc",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "r_fixed",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 10000 },
      },
      {
        id: "th1",
        kind: "thermistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { rNominal: 10000, beta: 3950 },
      },
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: "r_fixed", to_pin: "a" },
      { from_component: "r_fixed", from_pin: "b", to_component: "th1", to_pin: "a" },
      { from_component: "th1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
    ],
  };
  if (tempC !== undefined) {
    circuit.environment = { temperatureC: tempC, lux: 100 };
  }
  return circuit;
}

/**
 * Find the net id for the mid-node between r_fixed and the sensor
 * (r_fixed.b = th1.a = r_top.b = r_bot.a, depending on circuit).
 *
 * We reconstruct it from first principles to avoid hard-coding "n1", which
 * depends on buildNets() ordering — the tests should not rely on net id
 * stability across engine changes.
 */
function midNetId(circuit: SimCircuit, comp1Id: string, pin1: string): string {
  // Build a minimal net map just for the purpose of finding a shared net.
  // This mirrors how environment.test.ts discovers the mid-node via engine.nets.
  // Here we do it from the wire list so we don't need a loaded engine.
  //
  // For the test circuits every wire endpoint is unique enough that we can
  // find the net by looking for the wire that touches (comp1Id, pin1).
  for (const w of circuit.wires) {
    if (w.from_component === comp1Id && w.from_pin === pin1) {
      // The mid net is identified by the wire destination.
      return `${w.to_component}/${w.to_pin}`;
    }
    if (w.to_component === comp1Id && w.to_pin === pin1) {
      return `${w.from_component}/${w.from_pin}`;
    }
  }
  throw new Error(`mid net not found for ${comp1Id}.${pin1}`);
}

/**
 * After a sweep, find the actual net id from the engine by matching the
 * circuit topology.  We use a single-point sweep for this so we have a
 * loaded engine to inspect nets from.
 *
 * For test purposes we use the simcore SimEngine directly.
 */
import { SimEngine } from "../../src/sim/engine/sim-engine.js";

function findMidNetId(circuit: SimCircuit, compId: string, pinId: string): string {
  const engine = new SimEngine();
  engine.coldLoad(circuit);
  const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === compId && p === pinId));
  if (!net) throw new Error(`no net found for ${compId}.${pinId}`);
  return net.id;
}

// ─── Test a: voltage divider DC sweep ────────────────────────────────────────

describe("dc-sweep: voltage divider mid-net tracks V/2", () => {
  it("mean at mid-net ≈ voltage/2 at each sweep point", () => {
    const circuit = voltageDividerCircuit();
    // Find the mid-node net id from a loaded engine.
    const midNet = findMidNetId(circuit, "r_top", "b");

    const spec: DcSweepSpec = {
      kind: "dc-sweep",
      componentId: "psu1",
      param: "voltage",
      from: 2,
      to: 8,
      points: 7,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };

    const result = runSweep(spec, circuit);

    expect(result.cancelled).toBe(false);
    expect(result.x).toHaveLength(7);
    expect(result.outputs).toHaveLength(1);
    expect(result.outputs[0]!.mean).toHaveLength(7);

    for (let i = 0; i < 7; i++) {
      const xVal = result.x[i]!;
      const meanV = result.outputs[0]!.mean[i]!;
      // DC divider with equal R: V_mid = V_supply / 2
      // Allow ±0.1 V tolerance (generous for a pure resistive divider).
      expect(meanV).toBeCloseTo(xVal / 2, 1);
    }
  });

  it("min and max are tight for a DC source (|max-min| < 0.01 V)", () => {
    const circuit = voltageDividerCircuit();
    const midNet = findMidNetId(circuit, "r_top", "b");

    const spec: DcSweepSpec = {
      kind: "dc-sweep",
      componentId: "psu1",
      param: "voltage",
      from: 5,
      to: 5,
      points: 2,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };

    const result = runSweep(spec, circuit);
    const out = result.outputs[0]!;
    // DC source → no ripple → min ≈ max ≈ mean
    expect(out.max[0]! - out.min[0]!).toBeLessThan(0.01);
  });
});

// ─── Test b: signal_gen offset sweep ─────────────────────────────────────────

describe("dc-sweep: signal_gen offset tracked by tail mean", () => {
  it("mean tracks offset; max-min reflects amplitude", () => {
    const freq = 1000;
    const amplitude = 1.0;
    const circuit = signalGenCircuit({
      waveform: "sine",
      frequency: freq,
      amplitude,
      offset: 2.5, // will be swept — engine uses the cloned value per point
      rSource: 50,
      enabled: 1,
    });

    // Find the pos net of the signal gen.
    const posNet = findMidNetId(circuit, "sg1", "pos");

    // Sweep the offset param from 1 to 4 V in 4 points.
    const spec: DcSweepSpec = {
      kind: "dc-sweep",
      componentId: "sg1",
      param: "offset",
      from: 1.0,
      to: 4.0,
      points: 4,
      // Settle for at least 5 full cycles of the 1 kHz sine at 5e-5 step so
      // the tail average (final 20%) spans >1 full cycle and DC mean converges.
      settleTimeS: 0.05,
      outputs: [{ netId: posNet }],
    };

    const result = runSweep(spec, circuit);

    expect(result.cancelled).toBe(false);
    for (let i = 0; i < 4; i++) {
      const offset = result.x[i]!;
      const mean = result.outputs[0]!.mean[i]!;
      // Tail-averaging a full sine cycle gives the DC offset.
      // Allow ±0.1 V (10% of full range) for the approximation.
      expect(mean).toBeCloseTo(offset, 1);
    }

    // max-min should be roughly 2*amplitude (the peak-to-peak).
    // Allow 50% tolerance because the tail window may not span exactly one cycle.
    const pp0 = result.outputs[0]!.max[0]! - result.outputs[0]!.min[0]!;
    expect(pp0).toBeGreaterThan(amplitude * 0.5);
    expect(pp0).toBeLessThan(amplitude * 3.5);
  });
});

// ─── Test c: temperature sweep ────────────────────────────────────────────────

describe("temp-sweep: thermistor divider monotone in temperature", () => {
  it("output voltage decreases monotonically as temperature rises (NTC)", () => {
    const circuit = thermistorDividerCircuit();
    // Find the mid-node (r_fixed.b = th1.a)
    const midNet = findMidNetId(circuit, "r_fixed", "b");

    // Sweep 0 → 100 °C in 5 points.
    const spec: TempSweepSpec = {
      kind: "temp-sweep",
      from: 0,
      to: 100,
      points: 5,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };

    const result = runSweep(spec, circuit);

    expect(result.cancelled).toBe(false);
    expect(result.x).toHaveLength(5);

    const means = result.outputs[0]!.mean;
    // NTC thermistor: resistance decreases with temperature → mid-node voltage decreases.
    for (let i = 1; i < means.length; i++) {
      expect(means[i]!).toBeLessThan(means[i - 1]!);
    }
  });

  it("creates environment from defaults when circuit has no environment block", () => {
    // Circuit with no environment field — runner must inject one.
    const circuit = thermistorDividerCircuit(); // no environment
    expect(circuit.environment).toBeUndefined();

    const midNet = findMidNetId(circuit, "r_fixed", "b");
    const spec: TempSweepSpec = {
      kind: "temp-sweep",
      from: 25,
      to: 75,
      points: 3,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };

    // Should not throw.
    const result = runSweep(spec, circuit);
    expect(result.cancelled).toBe(false);
    // The original circuit was not mutated — environment still absent.
    expect(circuit.environment).toBeUndefined();
  });
});

// ─── Test d: cancellation ──────────────────────────────────────────────────────

describe("cancellation", () => {
  it("shouldCancel after point 2 → cancelled=true, arrays truncated to 3", () => {
    const circuit = voltageDividerCircuit();
    const midNet = findMidNetId(circuit, "r_top", "b");

    const spec: DcSweepSpec = {
      kind: "dc-sweep",
      componentId: "psu1",
      param: "voltage",
      from: 1,
      to: 9,
      points: 9,
      settleTimeS: 0.01,
      outputs: [{ netId: midNet }],
    };

    const onPoint = vi.fn();
    let pointCount = 0;

    const result = runSweep(spec, circuit, {
      onPoint(i, total) {
        pointCount++;
        onPoint(i, total);
      },
      shouldCancel() {
        // Cancel after the 3rd point (index 2) has been completed.
        return pointCount >= 3;
      },
    });

    expect(result.cancelled).toBe(true);
    // Arrays must be truncated to the number of completed points (3).
    expect(result.x).toHaveLength(3);
    expect(result.outputs[0]!.mean).toHaveLength(3);
    expect(result.outputs[0]!.min).toHaveLength(3);
    expect(result.outputs[0]!.max).toHaveLength(3);
    expect(result.pointFailures).toHaveLength(3);
    // onPoint must have been called exactly 3 times.
    expect(onPoint).toHaveBeenCalledTimes(3);
  });

  it("runSweepAsync: an EXTERNAL timer can flip the cancel flag mid-run", async () => {
    // This is the property the analysis worker depends on: the async runner
    // yields a macrotask between points, so an event scheduled elsewhere on
    // the event loop (here a timer; in the worker the queued "cancel"
    // message) gets dispatched while the sweep is in flight.  The
    // synchronous runSweep cannot pass this test by construction.
    const circuit = voltageDividerCircuit();
    const midNet = findMidNetId(circuit, "r_top", "b");

    const spec: DcSweepSpec = {
      kind: "dc-sweep",
      componentId: "psu1",
      param: "voltage",
      from: 1,
      to: 9,
      points: 60,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };

    let cancel = false;
    const promise = runSweepAsync(spec, circuit, {
      shouldCancel: () => cancel,
    });
    setTimeout(() => { cancel = true; }, 25);

    const result = await promise;
    expect(result.cancelled).toBe(true);
    expect(result.x.length).toBeGreaterThan(0);
    expect(result.x.length).toBeLessThan(60);
    expect(result.outputs[0]!.mean).toHaveLength(result.x.length);
  });
});

// ─── Test e: determinism ───────────────────────────────────────────────────────

describe("determinism", () => {
  it("two identical runs produce identical output arrays", () => {
    const circuit = voltageDividerCircuit();
    const midNet = findMidNetId(circuit, "r_top", "b");

    const spec: DcSweepSpec = {
      kind: "dc-sweep",
      componentId: "psu1",
      param: "voltage",
      from: 2,
      to: 6,
      points: 5,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };

    const r1 = runSweep(spec, circuit);
    const r2 = runSweep(spec, circuit);

    expect(r1.x).toEqual(r2.x);
    expect(r1.outputs[0]!.mean).toEqual(r2.outputs[0]!.mean);
    expect(r1.outputs[0]!.min).toEqual(r2.outputs[0]!.min);
    expect(r1.outputs[0]!.max).toEqual(r2.outputs[0]!.max);
  });
});

// ─── Test f: unknown output net id → 0-filled, no throw ───────────────────────

describe("unknown output net id", () => {
  it("yields 0-filled outputs and does not throw", () => {
    const circuit = voltageDividerCircuit();

    const spec: DcSweepSpec = {
      kind: "dc-sweep",
      componentId: "psu1",
      param: "voltage",
      from: 2,
      to: 4,
      points: 3,
      settleTimeS: 0.01,
      outputs: [{ netId: "net_does_not_exist" }],
    };

    let result;
    expect(() => {
      result = runSweep(spec, circuit);
    }).not.toThrow();

    // The runner returns 0 for unknown net ids via `netV[netId] ?? 0`.
    expect(result!.outputs[0]!.mean.every((v: number) => v === 0)).toBe(true);
    expect(result!.outputs[0]!.min.every((v: number) => v === 0)).toBe(true);
    expect(result!.outputs[0]!.max.every((v: number) => v === 0)).toBe(true);
  });
});

describe("analysis convergence safety", () => {
  function contradictorySources(): SimCircuit {
    return {
      components: [
        { id: "v5", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "v3", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
        { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      ],
      wires: [
        { from_component: "v5", from_pin: "pos", to_component: "v3", to_pin: "pos" },
        { from_component: "v5", from_pin: "neg", to_component: "v3", to_pin: "neg" },
        { from_component: "v5", from_pin: "pos", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "v5", to_pin: "neg" },
      ],
    };
  }

  it("rejects a DC sweep point instead of sampling a failed iterate", () => {
    const circuit = contradictorySources();
    const outputNet = findMidNetId(circuit, "load", "a");
    const spec: DcSweepSpec = {
      kind: "dc-sweep",
      componentId: "v5",
      param: "voltage",
      from: 5,
      to: 5,
      points: 2,
      settleTimeS: 50e-6,
      outputs: [{ netId: outputNet }],
    };

    expect(() => runSweep(spec, circuit)).toThrow(/did not converge.*rolled back.*rejected/i);
  });

  it("rejects a temperature sweep point instead of sampling a failed iterate", () => {
    const circuit = contradictorySources();
    const outputNet = findMidNetId(circuit, "load", "a");
    const spec: TempSweepSpec = {
      kind: "temp-sweep",
      from: 20,
      to: 30,
      points: 2,
      settleTimeS: 50e-6,
      outputs: [{ netId: outputNet }],
    };

    expect(() => runSweep(spec, circuit)).toThrow(/did not converge.*rolled back.*rejected/i);
  });
});
