/**
 * Unit tests for sweep job types and pure helpers (jobs.ts).
 *
 * These tests run in Node via vitest — no engine, no DOM required.
 */

import { describe, expect, it } from "vitest";
import {
  sweepValues,
  logSweepValues,
  validateJobSpec,
  SWEEP_MAX_POINTS,
  SWEEP_MIN_POINTS,
  SWEEP_MAX_OUTPUTS,
  SWEEP_SETTLE_MIN_S,
  SWEEP_SETTLE_MAX_S,
  AC_MIN_HZ,
  AC_MAX_HZ,
  AC_MAX_POINTS,
  MC_MIN_RUNS,
  MC_MAX_RUNS,
} from "../../src/analysis/jobs.js";
import type { DcSweepSpec, TempSweepSpec, AcSweepSpec, MonteCarloSpec } from "../../src/analysis/jobs.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

// ─── sweepValues ─────────────────────────────────────────────────────────────

describe("sweepValues", () => {
  it("returns exactly `points` values", () => {
    expect(sweepValues(0, 10, 5)).toHaveLength(5);
    expect(sweepValues(0, 10, 11)).toHaveLength(11);
  });

  it("first value is exactly `from`", () => {
    expect(sweepValues(2, 8, 7)[0]).toBe(2);
    expect(sweepValues(-5, 5, 3)[0]).toBe(-5);
  });

  it("last value is exactly `to`", () => {
    const vals = sweepValues(2, 8, 7);
    expect(vals[vals.length - 1]).toBe(8);
    const rev = sweepValues(10, 0, 5);
    expect(rev[rev.length - 1]).toBe(0);
  });

  it("values are evenly spaced", () => {
    const vals = sweepValues(0, 1, 5);
    // 0, 0.25, 0.5, 0.75, 1
    expect(vals[1]).toBeCloseTo(0.25, 10);
    expect(vals[2]).toBeCloseTo(0.5, 10);
    expect(vals[3]).toBeCloseTo(0.75, 10);
  });

  it("handles reversed range (from > to)", () => {
    const vals = sweepValues(10, 0, 3);
    expect(vals[0]).toBe(10);
    expect(vals[2]).toBe(0);
    expect(vals[1]).toBeCloseTo(5, 10);
  });

  it("single-point degenerate: returns [from]", () => {
    expect(sweepValues(5, 10, 1)).toEqual([5]);
  });

  it("two-point: exactly from and to", () => {
    const vals = sweepValues(1, 9, 2);
    expect(vals).toHaveLength(2);
    expect(vals[0]).toBe(1);
    expect(vals[1]).toBe(9);
  });
});

// ─── Minimal circuit fixture ─────────────────────────────────────────────────

function minimalCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "psu1",
        kind: "bench_psu",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5, iLimit: 1 },
      },
      {
        id: "r1",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
    ],
    wires: [
      { from_component: "psu1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "psu1", to_pin: "neg" },
    ],
  };
}

function validDcSpec(overrides: Partial<DcSweepSpec> = {}): DcSweepSpec {
  return {
    kind: "dc-sweep",
    componentId: "psu1",
    param: "voltage",
    from: 1,
    to: 10,
    points: 5,
    settleTimeS: 0.02,
    outputs: [{ netId: "n0" }],
    ...overrides,
  };
}

function validTempSpec(overrides: Partial<TempSweepSpec> = {}): TempSweepSpec {
  return {
    kind: "temp-sweep",
    from: 20,
    to: 80,
    points: 5,
    settleTimeS: 0.02,
    outputs: [{ netId: "n0" }],
    ...overrides,
  };
}

// ─── validateJobSpec — dc-sweep ───────────────────────────────────────────────

describe("validateJobSpec — dc-sweep", () => {
  it("returns null for a valid spec", () => {
    expect(validateJobSpec(validDcSpec(), minimalCircuit())).toBeNull();
  });

  it("rejects points below SWEEP_MIN_POINTS", () => {
    const err = validateJobSpec(validDcSpec({ points: 1 }), minimalCircuit());
    expect(err).toMatch(/points/i);
  });

  it("rejects points above SWEEP_MAX_POINTS", () => {
    const err = validateJobSpec(validDcSpec({ points: SWEEP_MAX_POINTS + 1 }), minimalCircuit());
    expect(err).toMatch(/points/i);
  });

  it("accepts exactly SWEEP_MIN_POINTS", () => {
    expect(validateJobSpec(validDcSpec({ points: SWEEP_MIN_POINTS }), minimalCircuit())).toBeNull();
  });

  it("accepts exactly SWEEP_MAX_POINTS", () => {
    expect(validateJobSpec(validDcSpec({ points: SWEEP_MAX_POINTS }), minimalCircuit())).toBeNull();
  });

  it("rejects non-integer points", () => {
    const err = validateJobSpec(validDcSpec({ points: 3.5 }), minimalCircuit());
    expect(err).toMatch(/points/i);
  });

  it("rejects settleTimeS below minimum", () => {
    const err = validateJobSpec(
      validDcSpec({ settleTimeS: SWEEP_SETTLE_MIN_S - 0.001 }),
      minimalCircuit(),
    );
    expect(err).toMatch(/settleTimeS/i);
  });

  it("rejects settleTimeS above maximum", () => {
    const err = validateJobSpec(
      validDcSpec({ settleTimeS: SWEEP_SETTLE_MAX_S + 0.001 }),
      minimalCircuit(),
    );
    expect(err).toMatch(/settleTimeS/i);
  });

  it("accepts exactly SWEEP_SETTLE_MIN_S", () => {
    expect(validateJobSpec(validDcSpec({ settleTimeS: SWEEP_SETTLE_MIN_S }), minimalCircuit())).toBeNull();
  });

  it("accepts exactly SWEEP_SETTLE_MAX_S", () => {
    expect(validateJobSpec(validDcSpec({ settleTimeS: SWEEP_SETTLE_MAX_S }), minimalCircuit())).toBeNull();
  });

  it("rejects 0 outputs", () => {
    const err = validateJobSpec(validDcSpec({ outputs: [] }), minimalCircuit());
    expect(err).toMatch(/outputs/i);
  });

  it("rejects too many outputs", () => {
    const tooMany = Array.from({ length: SWEEP_MAX_OUTPUTS + 1 }, () => ({ netId: "n0" }));
    const err = validateJobSpec(validDcSpec({ outputs: tooMany }), minimalCircuit());
    expect(err).toMatch(/outputs/i);
  });

  it("accepts exactly SWEEP_MAX_OUTPUTS outputs", () => {
    const maxOutputs = Array.from({ length: SWEEP_MAX_OUTPUTS }, (_, i) => ({ netId: `n${String(i)}` }));
    expect(validateJobSpec(validDcSpec({ outputs: maxOutputs }), minimalCircuit())).toBeNull();
  });

  it("rejects unknown componentId", () => {
    const err = validateJobSpec(
      validDcSpec({ componentId: "does_not_exist" }),
      minimalCircuit(),
    );
    expect(err).toMatch(/does_not_exist/);
  });

  it("rejects unknown param name on known component", () => {
    const err = validateJobSpec(
      validDcSpec({ param: "nonexistent_param" }),
      minimalCircuit(),
    );
    expect(err).toMatch(/nonexistent_param/);
  });

  it("rejects non-finite from", () => {
    const err = validateJobSpec(validDcSpec({ from: Infinity }), minimalCircuit());
    expect(err).toMatch(/finite/i);
  });

  it("rejects non-finite to", () => {
    const err = validateJobSpec(validDcSpec({ to: NaN }), minimalCircuit());
    expect(err).toMatch(/finite/i);
  });

  it("rejects a disabled signal_gen (would otherwise sweep to a flat-zero curve)", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "sg1",
          kind: "signal_gen",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 0, enabled: 0 },
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
    const err = validateJobSpec(
      validDcSpec({ componentId: "sg1", param: "offset" }),
      circuit,
    );
    expect(err).toMatch(/disabled/i);
  });

  it("allows an enabled signal_gen target", () => {
    const circuit: SimCircuit = {
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
          params: { resistance: 1000 },
        },
      ],
      wires: [
        { from_component: "sg1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "sg1", to_pin: "neg" },
      ],
    };
    expect(validateJobSpec(validDcSpec({ componentId: "sg1", param: "offset" }), circuit)).toBeNull();
  });
});

// ─── validateJobSpec — temp-sweep ─────────────────────────────────────────────

describe("validateJobSpec — temp-sweep", () => {
  it("returns null for a valid spec", () => {
    expect(validateJobSpec(validTempSpec(), minimalCircuit())).toBeNull();
  });

  it("rejects temperature below -40 C", () => {
    const err = validateJobSpec(validTempSpec({ from: -41, to: 25 }), minimalCircuit());
    expect(err).toMatch(/-40/);
  });

  it("rejects temperature above 125 C", () => {
    const err = validateJobSpec(validTempSpec({ from: 25, to: 126 }), minimalCircuit());
    expect(err).toMatch(/125/);
  });

  it("accepts -40 exactly", () => {
    expect(validateJobSpec(validTempSpec({ from: -40, to: 25 }), minimalCircuit())).toBeNull();
  });

  it("accepts 125 exactly", () => {
    expect(validateJobSpec(validTempSpec({ from: 25, to: 125 }), minimalCircuit())).toBeNull();
  });

  it("accepts reversed range within bounds", () => {
    // Sweeping from 80 down to 20 is valid; lo=20 >= -40, hi=80 <= 125.
    expect(validateJobSpec(validTempSpec({ from: 80, to: 20 }), minimalCircuit())).toBeNull();
  });

  it("rejects non-finite temperature", () => {
    const err = validateJobSpec(validTempSpec({ from: NaN }), minimalCircuit());
    expect(err).toMatch(/finite/i);
  });

  it("common checks also apply: points out of range", () => {
    const err = validateJobSpec(validTempSpec({ points: 1 }), minimalCircuit());
    expect(err).toMatch(/points/i);
  });
});

// ─── logSweepValues ───────────────────────────────────────────────────────────

describe("logSweepValues", () => {
  it("returns exactly `points` values", () => {
    expect(logSweepValues(10, 10000, 5)).toHaveLength(5);
    expect(logSweepValues(10, 100000, 31)).toHaveLength(31);
  });

  it("first value is exactly fromHz", () => {
    const vals = logSweepValues(10, 10000, 11);
    expect(vals[0]).toBe(10);
  });

  it("last value is exactly toHz", () => {
    const vals = logSweepValues(10, 10000, 11);
    expect(vals[vals.length - 1]).toBe(10000);
  });

  it("values are log-spaced: equal ratios between adjacent points", () => {
    const vals = logSweepValues(10, 10000, 5); // 2 decades, 5 points
    // Ratios between consecutive points should be equal.
    const r01 = vals[1]! / vals[0]!;
    const r12 = vals[2]! / vals[1]!;
    const r23 = vals[3]! / vals[2]!;
    expect(r01).toBeCloseTo(r12, 5);
    expect(r12).toBeCloseTo(r23, 5);
  });

  it("midpoint of 3 points covers 1 decade each side", () => {
    const vals = logSweepValues(10, 10000, 3); // 10 → 100 → 10000
    expect(vals[1]).toBeCloseTo(316.23, 0); // 10^2 = 100 would be 1-decade; actual is sqrt(10×10000)=316
  });

  it("single-point degenerate: returns [fromHz]", () => {
    expect(logSweepValues(100, 10000, 1)).toEqual([100]);
  });

  it("two-point: exactly fromHz and toHz", () => {
    const vals = logSweepValues(100, 10000, 2);
    expect(vals[0]).toBe(100);
    expect(vals[1]).toBe(10000);
  });
});

// ─── AC sweep circuit fixture ─────────────────────────────────────────────────

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
}

function validAcSpec(overrides: Partial<AcSweepSpec> = {}): AcSweepSpec {
  return {
    kind: "ac-sweep",
    componentId: "sg1",
    fromHz: 100,
    toHz: 10000,
    points: 11,
    outputs: [{ netId: "n0" }],
    ...overrides,
  };
}

function acCircuit(): SimCircuit {
  return signalGenCircuit({
    waveform: "sine",
    frequency: 1000,
    amplitude: 1,
    offset: 0,
    rSource: 50,
    enabled: 1,
  });
}

// ─── validateJobSpec — ac-sweep ───────────────────────────────────────────────

describe("validateJobSpec — ac-sweep", () => {
  it("returns null for a valid spec", () => {
    expect(validateJobSpec(validAcSpec(), acCircuit())).toBeNull();
  });

  it("rejects componentId not in circuit", () => {
    const err = validateJobSpec(validAcSpec({ componentId: "ghost" }), acCircuit());
    expect(err).toMatch(/ghost/);
  });

  it("rejects component that is not signal_gen (wrong kind)", () => {
    // r1 is a resistor — not a signal_gen.
    const err = validateJobSpec(validAcSpec({ componentId: "r1" }), acCircuit());
    expect(err).toMatch(/signal_gen/i);
  });

  it("rejects when signal_gen has enabled=0", () => {
    const circuit = signalGenCircuit({
      waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 50, enabled: 0,
    });
    const err = validateJobSpec(validAcSpec(), circuit);
    expect(err).toMatch(/disabled/i);
  });

  it("rejects when amplitude is 0", () => {
    const circuit = signalGenCircuit({
      waveform: "sine", frequency: 1000, amplitude: 0, offset: 0, rSource: 50, enabled: 1,
    });
    const err = validateJobSpec(validAcSpec(), circuit);
    expect(err).toMatch(/amplitude/i);
  });

  it("rejects when amplitude is negative", () => {
    const circuit = signalGenCircuit({
      waveform: "sine", frequency: 1000, amplitude: -1, offset: 0, rSource: 50, enabled: 1,
    });
    const err = validateJobSpec(validAcSpec(), circuit);
    expect(err).toMatch(/amplitude/i);
  });

  it("rejects fromHz below AC_MIN_HZ", () => {
    const err = validateJobSpec(validAcSpec({ fromHz: AC_MIN_HZ - 1 }), acCircuit());
    expect(err).toMatch(/fromHz/i);
  });

  it("rejects toHz above AC_MAX_HZ", () => {
    const err = validateJobSpec(validAcSpec({ toHz: AC_MAX_HZ + 1 }), acCircuit());
    expect(err).toMatch(/toHz/i);
  });

  it("rejects fromHz >= toHz", () => {
    const err = validateJobSpec(validAcSpec({ fromHz: 1000, toHz: 1000 }), acCircuit());
    expect(err).toMatch(/less than/i);
  });

  it("rejects fromHz > toHz", () => {
    const err = validateJobSpec(validAcSpec({ fromHz: 5000, toHz: 1000 }), acCircuit());
    expect(err).toMatch(/less than/i);
  });

  it("rejects points below 2", () => {
    const err = validateJobSpec(validAcSpec({ points: 1 }), acCircuit());
    expect(err).toMatch(/points/i);
  });

  it("rejects points above AC_MAX_POINTS", () => {
    const err = validateJobSpec(validAcSpec({ points: AC_MAX_POINTS + 1 }), acCircuit());
    expect(err).toMatch(/points/i);
  });

  it("accepts exactly AC_MAX_POINTS", () => {
    expect(validateJobSpec(validAcSpec({ points: AC_MAX_POINTS }), acCircuit())).toBeNull();
  });

  it("accepts exactly AC_MIN_HZ for fromHz", () => {
    expect(validateJobSpec(validAcSpec({ fromHz: AC_MIN_HZ, toHz: 1000 }), acCircuit())).toBeNull();
  });

  it("accepts exactly AC_MAX_HZ for toHz", () => {
    expect(validateJobSpec(validAcSpec({ fromHz: 100, toHz: AC_MAX_HZ }), acCircuit())).toBeNull();
  });

  it("rejects 0 outputs", () => {
    const err = validateJobSpec(validAcSpec({ outputs: [] }), acCircuit());
    expect(err).toMatch(/outputs/i);
  });

  it("rejects too many outputs", () => {
    const tooMany = Array.from({ length: SWEEP_MAX_OUTPUTS + 1 }, () => ({ netId: "n0" }));
    const err = validateJobSpec(validAcSpec({ outputs: tooMany }), acCircuit());
    expect(err).toMatch(/outputs/i);
  });

  it("rejects non-finite fromHz", () => {
    const err = validateJobSpec(validAcSpec({ fromHz: NaN }), acCircuit());
    expect(err).toMatch(/finite/i);
  });
});

// ─── Monte Carlo fixtures ─────────────────────────────────────────────────────

/** Circuit with a toleranced resistor — validates successfully. */
function mcCircuitWithResistor(): SimCircuit {
  return {
    components: [
      {
        id: "psu1",
        kind: "bench_psu",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5, iLimit: 1 },
      },
      {
        id: "r1",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
    ],
    wires: [
      { from_component: "psu1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "psu1", to_pin: "neg" },
    ],
  };
}

/**
 * Circuit with only a signal_gen and a wire — no toleranced components.
 * Used to test the "no components with tolerance data" rejection path.
 */
function mcCircuitNoTolerance(): SimCircuit {
  return {
    components: [
      {
        id: "sg1",
        kind: "signal_gen",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 50, enabled: 1 },
      },
    ],
    // A single component with no other components to connect to — the engine
    // would fail to build a circuit, but validateJobSpec does not run the engine.
    wires: [],
  };
}

function validMcSpec(overrides: Partial<MonteCarloSpec> = {}): MonteCarloSpec {
  return {
    kind: "monte-carlo",
    runs: 30,
    seed: 42,
    settleTimeS: 0.02,
    outputs: [{ netId: "n0" }],
    ...overrides,
  };
}

// ─── validateJobSpec — monte-carlo ────────────────────────────────────────────

describe("validateJobSpec — monte-carlo", () => {
  it("returns null for a valid spec with a toleranced resistor", () => {
    expect(validateJobSpec(validMcSpec(), mcCircuitWithResistor())).toBeNull();
  });

  it("rejects runs below MC_MIN_RUNS", () => {
    const err = validateJobSpec(validMcSpec({ runs: MC_MIN_RUNS - 1 }), mcCircuitWithResistor());
    expect(err).toMatch(/runs/i);
  });

  it("rejects runs above MC_MAX_RUNS", () => {
    const err = validateJobSpec(validMcSpec({ runs: MC_MAX_RUNS + 1 }), mcCircuitWithResistor());
    expect(err).toMatch(/runs/i);
  });

  it("accepts exactly MC_MIN_RUNS", () => {
    expect(validateJobSpec(validMcSpec({ runs: MC_MIN_RUNS }), mcCircuitWithResistor())).toBeNull();
  });

  it("accepts exactly MC_MAX_RUNS", () => {
    expect(validateJobSpec(validMcSpec({ runs: MC_MAX_RUNS }), mcCircuitWithResistor())).toBeNull();
  });

  it("rejects non-integer runs", () => {
    const err = validateJobSpec(validMcSpec({ runs: 20.5 }), mcCircuitWithResistor());
    expect(err).toMatch(/runs/i);
  });

  it("rejects non-integer seed", () => {
    const err = validateJobSpec(validMcSpec({ seed: 1.5 }), mcCircuitWithResistor());
    expect(err).toMatch(/seed/i);
  });

  it("accepts seed 0", () => {
    expect(validateJobSpec(validMcSpec({ seed: 0 }), mcCircuitWithResistor())).toBeNull();
  });

  it("accepts negative integer seed", () => {
    // Negative seeds are valid integers; the PRNG handles the bit pattern.
    expect(validateJobSpec(validMcSpec({ seed: -7 }), mcCircuitWithResistor())).toBeNull();
  });

  it("rejects settleTimeS below minimum", () => {
    const err = validateJobSpec(
      validMcSpec({ settleTimeS: SWEEP_SETTLE_MIN_S - 0.001 }),
      mcCircuitWithResistor(),
    );
    expect(err).toMatch(/settleTimeS/i);
  });

  it("rejects settleTimeS above maximum", () => {
    const err = validateJobSpec(
      validMcSpec({ settleTimeS: SWEEP_SETTLE_MAX_S + 0.001 }),
      mcCircuitWithResistor(),
    );
    expect(err).toMatch(/settleTimeS/i);
  });

  it("accepts exactly SWEEP_SETTLE_MIN_S", () => {
    expect(validateJobSpec(validMcSpec({ settleTimeS: SWEEP_SETTLE_MIN_S }), mcCircuitWithResistor())).toBeNull();
  });

  it("accepts exactly SWEEP_SETTLE_MAX_S", () => {
    expect(validateJobSpec(validMcSpec({ settleTimeS: SWEEP_SETTLE_MAX_S }), mcCircuitWithResistor())).toBeNull();
  });

  it("rejects non-finite settleTimeS", () => {
    const err = validateJobSpec(validMcSpec({ settleTimeS: Infinity }), mcCircuitWithResistor());
    expect(err).toMatch(/settleTimeS/i);
  });

  it("rejects 0 outputs", () => {
    const err = validateJobSpec(validMcSpec({ outputs: [] }), mcCircuitWithResistor());
    expect(err).toMatch(/outputs/i);
  });

  it("rejects too many outputs", () => {
    const tooMany = Array.from({ length: SWEEP_MAX_OUTPUTS + 1 }, () => ({ netId: "n0" }));
    const err = validateJobSpec(validMcSpec({ outputs: tooMany }), mcCircuitWithResistor());
    expect(err).toMatch(/outputs/i);
  });

  it("accepts exactly SWEEP_MAX_OUTPUTS outputs", () => {
    const maxOutputs = Array.from({ length: SWEEP_MAX_OUTPUTS }, (_, i) => ({ netId: `n${String(i)}` }));
    expect(validateJobSpec(validMcSpec({ outputs: maxOutputs }), mcCircuitWithResistor())).toBeNull();
  });

  it("rejects circuit with no toleranced components (signal_gen + wires only)", () => {
    // The validator must check that at least one component has catalog-provided
    // value_tol or vf_tol; a circuit of only signal_gen parts has neither.
    const err = validateJobSpec(validMcSpec(), mcCircuitNoTolerance());
    expect(err).toMatch(/tolerance/i);
  });

  it("no-tolerance error message mentions resistors, capacitors, diodes, or LEDs", () => {
    const err = validateJobSpec(validMcSpec(), mcCircuitNoTolerance());
    // The message should direct the user to add the right component types.
    expect(err).toMatch(/resistor|capacitor|diode|led/i);
  });

  it("rejects a tvs_diode-only circuit (vf_tol present but no readable Vf)", () => {
    // tvs_diode declares vf_tol but exposes no forward voltage (it has vbr, not
    // vf), and the runner never perturbs it — so it must NOT qualify a circuit
    // for Monte Carlo, otherwise the UI advertises a run that perturbs nothing.
    const circuit: SimCircuit = {
      components: [
        {
          id: "tvs1",
          kind: "tvs_diode",
          pins: [{ id: "a" }, { id: "b" }],
          params: { vbr: 6.8 },
        },
      ],
      wires: [],
    };
    const err = validateJobSpec(validMcSpec(), circuit);
    expect(err).toMatch(/tolerance/i);
  });

  it("rejects a logic-IC-only circuit (vth_tol is metadata-only, not modeled)", () => {
    // 74ls00 carries vth_tol but the engine has no per-instance threshold
    // override, so the runner does not vary it.  A vth-only circuit must be
    // reported as having no toleranced components — this locks the honest
    // behavior against future drift that adds vth_tol to the qualifying set.
    const circuit: SimCircuit = {
      components: [
        {
          id: "u1",
          kind: "74ls00",
          pins: [{ id: "1A" }, { id: "1B" }, { id: "1Y" }, { id: "VCC" }, { id: "GND" }],
          params: {},
        },
      ],
      wires: [],
    };
    const err = validateJobSpec(validMcSpec(), circuit);
    expect(err).toMatch(/tolerance/i);
  });

  it("accepts an rgb_led circuit (catalog Vf is readable)", () => {
    // rgb_led declares vf + vf_tol, so it qualifies; the runner perturbs its
    // per-emitter vf_r/vf_g/vf_b (seeded into default_params).
    const circuit: SimCircuit = {
      components: [
        {
          id: "led1",
          kind: "rgb_led",
          pins: [{ id: "r_a" }, { id: "g_a" }, { id: "b_a" }, { id: "com_k" }],
          params: { vf_r: 1.8, vf_g: 2.0, vf_b: 3.0 },
        },
      ],
      wires: [],
    };
    expect(validateJobSpec(validMcSpec(), circuit)).toBeNull();
  });
});
