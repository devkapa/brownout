/**
 * Integration tests for runMonteCarlo() against the real SimEngine.
 *
 * WHY integration (not unit):
 *   The runner's correctness guarantee is "each run settles to a slightly
 *   different DC operating point because component values were perturbed, and
 *   the aggregate statistics (mean, stddev, min, max) correctly describe the
 *   distribution of those operating points".  Mocking the engine would test
 *   only the perturbation bookkeeping — these tests verify that the
 *   perturbations actually change the simulated voltages and that the results
 *   are analytically bounded.
 *
 * Circuit fixtures follow run-sweep.test.ts conventions:
 *   - Minimal component set (source + passive(s) + device under test)
 *   - No position/rotation metadata
 *   - Pin ids from catalog conventions ("pos"/"neg" for sources, "a"/"b" for passives)
 *
 * Tolerances:
 *   The two 5%-tol resistors in the divider tests bound the mid-node voltage to
 *   the worst-case range [5×(0.95/(0.95+1.05)) , 5×(1.05/(0.95+1.05))].
 *   Values must lie strictly within this range.  Mean must track the nominal
 *   (2.5 V for equal resistors) within ±5% (0.125 V) — a very loose bound to
 *   survive small run counts.
 */

import { describe, expect, it, vi } from "vitest";
import { runMonteCarlo, runMonteCarloAsync } from "../../src/analysis/run-monte-carlo.js";
import { validateJobSpec } from "../../src/analysis/jobs.js";
import type { MonteCarloSpec } from "../../src/analysis/jobs.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";

// ─── Circuit builder helpers ──────────────────────────────────────────────────

/**
 * bench_psu driving a symmetric voltage divider: two 1 kΩ (5% tolerance) resistors.
 *
 * Both r_top and r_bot are catalog kind "resistor" — the catalog has value_tol 0.05
 * for resistors, so the Monte Carlo runner will perturb both.
 *
 * Nominal mid-node voltage: 5 × 1k/(1k+1k) = 2.5 V.
 * Worst-case range: [5×(0.95/2), 5×(1.05/2)] = [2.375, 2.625] when one is at -5%
 * and the other at +5%.  Actual bounds are tighter:
 *   V_mid_min = 5 × (R_top_min / (R_top_min + R_bot_max))
 *             = 5 × 0.95 / (0.95 + 1.05) = 5 × 0.475 = 2.375
 *   V_mid_max = 5 × (R_top_max / (R_top_max + R_bot_min))
 *             = 5 × 1.05 / (1.05 + 0.95) = 5 × 0.525 = 2.625
 */
function voltageDividerCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "psu1",
        kind: "bench_psu",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5, iLimit: 1 },
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
 * Same topology as voltageDividerCircuit but with components in REVERSED order.
 * Used to verify that perturbations are order-independent — the per-component
 * deltas are keyed by component id, not array index.
 */
function voltageDividerReversedCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "r_bot",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
      {
        id: "r_top",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
      {
        id: "psu1",
        kind: "bench_psu",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5, iLimit: 1 },
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
 * bench_psu → resistor → LED (kind "led", vf 2.0 V, vf_tol 0.05) → gnd.
 *
 * The LED conducts, so the anode settles near vf (~2 V) and perturbing vf moves
 * it run to run.  NOTE: the kind must be a real catalog kind ("led") and params
 * must carry a vf — a non-existent kind (e.g. "led-red") is not stamped as a
 * diode, the LED never conducts, vf is never perturbed, and the only spread is
 * resistor float noise (the false-positive this fixture used to hide).
 */
function ledResistorCircuit(): SimCircuit {
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
        params: { resistance: 470 },
      },
      {
        id: "led1",
        kind: "led",
        pins: [{ id: "anode" }, { id: "cathode" }],
        params: { vf: 2.0 },
      },
    ],
    wires: [
      { from_component: "psu1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "led1", to_pin: "anode" },
      { from_component: "led1", from_pin: "cathode", to_component: "psu1", to_pin: "neg" },
    ],
  };
}

function catalogLedCircuit(catalogUid: "led-red" | "led-blue"): SimCircuit {
  const circuit = ledResistorCircuit();
  const led = circuit.components.find((component) => component.id === "led1")!;
  led.catalogUid = catalogUid;
  // Force the runner to obtain the nominal Vf from the exact catalog entry.
  delete led.params.vf;
  return circuit;
}

// ─── Net-id helper ────────────────────────────────────────────────────────────

/**
 * Resolve the engine net id for a given component/pin by cold-loading the circuit.
 * Mirrors run-sweep.test.ts's findMidNetId helper.
 */
function findNetId(circuit: SimCircuit, compId: string, pinId: string): string {
  const engine = new SimEngine();
  engine.coldLoad(circuit);
  const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === compId && p === pinId));
  if (!net) throw new Error(`no net found for ${compId}.${pinId}`);
  return net.id;
}

// ─── Test a: divider values within analytical worst-case bounds ───────────────

describe("monte-carlo: resistor divider values within worst-case bounds", () => {
  it("all run values are within [2.375, 2.625] V and stddev > 0", () => {
    const circuit = voltageDividerCircuit();
    const midNet = findNetId(circuit, "r_top", "b");

    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 30,
      seed: 42,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };

    const result = runMonteCarlo(spec, circuit);

    expect(result.cancelled).toBe(false);
    expect(result.runs).toBe(30);
    expect(result.outputs).toHaveLength(1);
    expect(result.outputs[0]!.values).toHaveLength(30);

    // Every individual run must be within the worst-case tolerance band.
    const V_MIN = 2.375;
    const V_MAX = 2.625;
    for (const v of result.outputs[0]!.values) {
      expect(v).toBeGreaterThan(V_MIN - 0.01);
      expect(v).toBeLessThan(V_MAX + 0.01);
    }

    // With 30 runs and two independently perturbed resistors, stddev must be
    // non-zero — perturbations are producing genuinely different voltages.
    expect(result.outputs[0]!.stddev).toBeGreaterThan(0);
  });

  it("mean is within ±5% of 2.5 V (nominal divider output)", () => {
    const circuit = voltageDividerCircuit();
    const midNet = findNetId(circuit, "r_top", "b");

    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 30,
      seed: 99,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };

    const result = runMonteCarlo(spec, circuit);
    // 5% of 2.5 V = 0.125 V — a very loose bound that holds unless the PRNG
    // is badly biased or the perturbation sign convention is wrong.
    expect(result.outputs[0]!.mean).toBeCloseTo(2.5, 0);
  });
});

// ─── Test b: determinism — same seed → identical values, diff seed → different ─

describe("monte-carlo: determinism", () => {
  it("same seed twice produces identical values arrays", () => {
    const circuit = voltageDividerCircuit();
    const midNet = findNetId(circuit, "r_top", "b");

    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 20,
      seed: 7,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };

    const r1 = runMonteCarlo(spec, circuit);
    const r2 = runMonteCarlo(spec, circuit);

    expect(r1.outputs[0]!.values).toEqual(r2.outputs[0]!.values);
  });

  it("different seed produces at least one differing value", () => {
    const circuit = voltageDividerCircuit();
    const midNet = findNetId(circuit, "r_top", "b");

    const specA: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 20,
      seed: 1,
      settleTimeS: 0.02,
      outputs: [{ netId: midNet }],
    };
    const specB: MonteCarloSpec = { ...specA, seed: 2 };

    const rA = runMonteCarlo(specA, circuit);
    const rB = runMonteCarlo(specB, circuit);

    const allSame = rA.outputs[0]!.values.every((v, i) => v === rB.outputs[0]!.values[i]);
    expect(allSame).toBe(false);
  });
});

// ─── Test c: order-independence ───────────────────────────────────────────────

describe("monte-carlo: order-independence", () => {
  it("reversing the components array produces the same per-run values", () => {
    // The perturbation key includes the component id, not its array index.
    // A structurally identical circuit with components re-ordered must produce
    // the same output — otherwise "same seed" would be a fragile contract.
    const circuitNormal = voltageDividerCircuit();
    const circuitReversed = voltageDividerReversedCircuit();

    // Find the mid-net id from both circuits to confirm they resolve identically.
    const midNetNormal = findNetId(circuitNormal, "r_top", "b");
    const midNetReversed = findNetId(circuitReversed, "r_top", "b");

    const specNormal: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 20,
      seed: 42,
      settleTimeS: 0.02,
      outputs: [{ netId: midNetNormal }],
    };
    const specReversed: MonteCarloSpec = { ...specNormal, outputs: [{ netId: midNetReversed }] };

    const rNormal = runMonteCarlo(specNormal, circuitNormal);
    const rReversed = runMonteCarlo(specReversed, circuitReversed);

    // Per-run values must be bit-identical.
    expect(rNormal.outputs[0]!.values).toEqual(rReversed.outputs[0]!.values);
  });
});

// ─── Test d: vf variation — LED node voltage spreads across runs ──────────────

describe("monte-carlo: vf variation for LED", () => {
  it("LED-anode node values spread across runs (stddev > 0)", () => {
    const circuit = ledResistorCircuit();
    const anodeNet = findNetId(circuit, "led1", "anode");

    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 30,
      seed: 13,
      settleTimeS: 0.05,
      outputs: [{ netId: anodeNet }],
    };

    const result = runMonteCarlo(spec, circuit);

    expect(result.cancelled).toBe(false);

    // The LED must actually be conducting: a non-conducting LED pins the anode
    // to ~5 V (the rail).  Conducting clamps it near vf, well under 4 V.  This
    // guards against the kind/vf fixture regressing to a non-stamped element.
    expect(result.outputs[0]!.mean).toBeLessThan(4);

    // Perturbing vf changes the forward bias and the anode voltage.  Floor the
    // spread well above float noise (a resistor-only perturbation produced
    // ~6.6e-8 V) so the test cannot pass without the vf draw actually moving it.
    expect(result.outputs[0]!.stddev).toBeGreaterThan(0.01);

    // All values must be physically plausible: between 0 V and 5 V.
    for (const v of result.outputs[0]!.values) {
      expect(v).toBeGreaterThanOrEqual(0);
      expect(v).toBeLessThanOrEqual(5);
    }
  });

  it("uses exact same-kind catalog identity for the nominal forward voltage", () => {
    const redCircuit = catalogLedCircuit("led-red");
    const blueCircuit = catalogLedCircuit("led-blue");
    const redNet = findNetId(redCircuit, "led1", "anode");
    const blueNet = findNetId(blueCircuit, "led1", "anode");
    const baseSpec = {
      kind: "monte-carlo" as const,
      runs: 12,
      seed: 21,
      settleTimeS: 0.03,
    };

    const red = runMonteCarlo({ ...baseSpec, outputs: [{ netId: redNet }] }, redCircuit);
    const blue = runMonteCarlo({ ...baseSpec, outputs: [{ netId: blueNet }] }, blueCircuit);

    expect(red.outputs[0]!.mean).toBeGreaterThan(1.5);
    expect(blue.outputs[0]!.mean).toBeGreaterThan(red.outputs[0]!.mean + 0.7);
  });
});

// ─── Test d2: rgb_led per-emitter vf variation ────────────────────────────────

/**
 * bench_psu → resistor → rgb_led red anode (r_a); com_k → gnd.
 *
 * The red channel conducts, so the r_a node settles near vf_r.  The per-emitter
 * vf_r/vf_g/vf_b are seeded into the catalog default_params, so a placed rgb_led
 * carries them in params and the runner perturbs them — without that seeding the
 * engine reads its hardcoded vf_r default and tolerance has no effect.
 */
function rgbLedCircuit(): SimCircuit {
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
        params: { resistance: 470 },
      },
      {
        id: "led1",
        kind: "rgb_led",
        pins: [{ id: "r_a" }, { id: "g_a" }, { id: "b_a" }, { id: "com_k" }],
        // Per-emitter Vf carried in params (mirrors what placement seeds from
        // default_params) so the runner's vf_r/vf_g/vf_b loop perturbs them.
        params: { vf_r: 1.8, vf_g: 2.0, vf_b: 3.0 },
      },
    ],
    wires: [
      { from_component: "psu1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "led1", to_pin: "r_a" },
      { from_component: "led1", from_pin: "com_k", to_component: "psu1", to_pin: "neg" },
    ],
  };
}

describe("monte-carlo: rgb_led per-emitter vf variation", () => {
  it("red-anode node values spread across runs once vf_r is perturbed", () => {
    const circuit = rgbLedCircuit();
    const anodeNet = findNetId(circuit, "led1", "r_a");

    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 30,
      seed: 5,
      settleTimeS: 0.05,
      outputs: [{ netId: anodeNet }],
    };

    const result = runMonteCarlo(spec, circuit);
    expect(result.cancelled).toBe(false);
    // Conducting channel clamps the anode near vf_r (~1.8 V), well under the rail.
    expect(result.outputs[0]!.mean).toBeLessThan(4);
    // Real spread from the vf_r draw, not float noise.
    expect(result.outputs[0]!.stddev).toBeGreaterThan(0.005);
  });
});

// ─── Test e: cancellation ─────────────────────────────────────────────────────

describe("monte-carlo: cancellation", () => {
  it("shouldCancel after 5 runs → cancelled=true, values truncated to 5", () => {
    const circuit = voltageDividerCircuit();
    const midNet = findNetId(circuit, "r_top", "b");

    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 30,
      seed: 1,
      settleTimeS: 0.01,
      outputs: [{ netId: midNet }],
    };

    const onPoint = vi.fn();
    let completedRuns = 0;

    const result = runMonteCarlo(spec, circuit, {
      onPoint(r, total) {
        completedRuns++;
        onPoint(r, total);
      },
      shouldCancel() {
        return completedRuns >= 5;
      },
    });

    expect(result.cancelled).toBe(true);
    expect(result.runs).toBe(5);
    expect(result.outputs[0]!.values).toHaveLength(5);
    expect(result.pointFailures).toHaveLength(5);
    expect(onPoint).toHaveBeenCalledTimes(5);
  });

  it("pointFailures length matches values length after cancellation", () => {
    const circuit = voltageDividerCircuit();
    const midNet = findNetId(circuit, "r_top", "b");

    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 50,
      seed: 2,
      settleTimeS: 0.01,
      outputs: [{ netId: midNet }],
    };

    let n = 0;
    const result = runMonteCarlo(spec, circuit, {
      onPoint() { n++; },
      shouldCancel() { return n >= 8; },
    });

    expect(result.pointFailures).toHaveLength(result.runs);
    expect(result.outputs[0]!.values).toHaveLength(result.runs);
  });
});

// ─── Test f: zero-tolerance circuit rejected by validateJobSpec ───────────────

describe("monte-carlo: zero-tolerance circuit validation", () => {
  it("validateJobSpec rejects a circuit with only signal_gen (no toleranced components)", () => {
    // The runner itself is not tested here — the spec-level guard is the correct
    // place to enforce this, so the runner never needs to handle the edge case.
    const noTolCircuit: SimCircuit = {
      components: [
        {
          id: "sg1",
          kind: "signal_gen",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { waveform: "sine", frequency: 1000, amplitude: 1, offset: 0, rSource: 50, enabled: 1 },
        },
      ],
      wires: [],
    };

    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 20,
      seed: 1,
      settleTimeS: 0.02,
      outputs: [{ netId: "n0" }],
    };

    const err = validateJobSpec(spec, noTolCircuit);
    expect(err).not.toBeNull();
    expect(err).toMatch(/tolerance/i);
  });

  it("validateJobSpec accepts a circuit that has a resistor (tolerance available)", () => {
    const tolCircuit: SimCircuit = {
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

    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 20,
      seed: 1,
      settleTimeS: 0.02,
      outputs: [{ netId: "n0" }],
    };

    expect(validateJobSpec(spec, tolCircuit)).toBeNull();
  });
});

describe("monte-carlo convergence safety", () => {
  it("rejects a run instead of including a failed iterate in its statistics", () => {
    const circuit: SimCircuit = {
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
    const outputNet = findNetId(circuit, "load", "a");
    const spec: MonteCarloSpec = {
      kind: "monte-carlo",
      runs: 2,
      seed: 7,
      settleTimeS: 50e-6,
      outputs: [{ netId: outputNet }],
    };

    expect(() => runMonteCarlo(spec, circuit)).toThrow(/did not converge.*rolled back.*rejected/i);
  });
});
