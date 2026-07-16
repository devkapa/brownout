/**
 * Wave 3 protection-device tests: reverse-breakdown stamp, zener retrofit,
 * TVS diode, and PTC resettable fuse.
 *
 * All numeric expected values are computed from first-principles physics or
 * measured from the engine during authoring — never derived from the
 * implementation constants themselves.
 *
 * Sources for literal values:
 *   - Shockley diode equation: i = Is * (exp(v/(n*Vt)) - 1)
 *     Vt = kT/q = 0.02585 V at 25°C (Neamen, "Semiconductor Physics and
 *     Devices", 4th ed., §8.1, Table 8-1)
 *   - Is_f = iRated * exp(-Vf/(n*Vt)) — solving the Shockley equation for Is
 *     (same source §8.2)
 *   - The reverse exponential uses knee overdrive, so its prefactor is directly
 *     izKnee: the exponent is zero at vAK = -vz (elements.ts model definition).
 *   - PTC trip threshold: i^2 * t = iHold^2 * 0.1 A²s → trip in 100ms at 2x
 *     (implementation definition, sim-engine.ts PTC state machine comment)
 *   - Exponential guard: reverse overdrive is evaluated exponentially through
 *     5*nZ*Vt past the knee, then continued along the boundary tangent. This
 *     prevents f64 overflow while preserving continuous current and slope.
 */

import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";
import { buildNets } from "../../../src/sim/engine/graph.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { analyzeLive } from "../../helpers/diagnostics/live.js";
import type { DiagnosticsInput, LiveReadings } from "../../helpers/diagnostics/index.js";

const catalog = rawCatalog as unknown as PartCatalog;

// ─── Helpers ──────────────────────────────────────────────────────────────────

function runEngine(circuit: SimCircuit, steps = 50, dt = 1e-4): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) =>
    n.pins.some(([cid, pid]) => cid === componentId && pid === pinId),
  );
  if (!net) return 0;
  return engine.getNetV()[net.id] ?? 0;
}

function liveFromEngine(engine: SimEngine, ptcTripped?: ReadonlySet<string>): LiveReadings {
  return {
    netV: engine.getNetV(),
    elementI: engine.getElementI(),
    elementChannelI: engine.getElementChannelI(),
    digitalState: engine.digitalState,
    simTime: engine.simTime,
    converged: engine.lastConverged,
    failures: engine.getFailures(),
    ptcTripped,
  };
}

function makeInput(circuit: SimCircuit, live: LiveReadings): DiagnosticsInput {
  return {
    circuit: circuit as unknown as DiagnosticsInput["circuit"],
    nets: buildNets(circuit),
    catalog,
    live,
  };
}

// ─── Task 1 + 2: Zener diode (stampBreakdownDiode retrofit) ──────────────────

describe("zener_diode — forward conduction (regression)", () => {
  /**
   * Circuit: 5 V source → 1 kΩ resistor → zener_diode (forward-biased) → GND.
   *
   * Expected current derived from Newton-Raphson convergence on:
   *   i(v) = Is_f * (exp(v/Vt) - 1)  where Is_f = 0.1 * exp(-0.7/0.02585) = 1.736e-13 A
   *   i = (5 - v) / 1000
   * Solved iteratively (see /tmp/compute_literals.mjs):
   *   V_junction ≈ 0.619 V, I ≈ 4.381 mA
   *
   * Pre-change (forward-only Shockley) gave the same I because Is_z for vz=5.1
   * at nZ=1 is 1.04e-88 A — negligibly small in the forward region.
   * Source: prototype engine run during test authoring, verified consistent with
   * Newton-Raphson solution to 0.01% tolerance.
   */
  it("carries 4.0–5.0 mA through a 5V/1kΩ circuit", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "z1", kind: "zener_diode", pins: [{ id: "a" }, { id: "k" }], params: { vf: 0.7, vz: 5.1, iRated: 0.1, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "z1", to_pin: "a" },
        { from_component: "z1", from_pin: "k", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const i_A = eng.getElementI()["z1"] ?? 0;
    // 4.0–5.0 mA: matches the Newton solution (4.381 mA) with generous tolerance
    // for numerical convergence variance.
    expect(i_A * 1000).toBeGreaterThan(4.0);
    expect(i_A * 1000).toBeLessThan(5.0);
  });
});

describe("zener_diode — reverse blocking well below vz", () => {
  /**
   * Circuit: 3 V source in reverse-bias polarity across 5.1 V zener through 470 Ω.
   * Cathode connected to source +, anode to GND.
   *
   * At vAK = -3 V (2.1 V below the knee): overdrive = -(v + vz) = -2.1 V →
   *   iRev = izKnee * exp(-2.1/0.02585) = 0.005 * e^-81 ≈ 5e-38 A — nothing.
   * (At 0.1 V below the knee a sharp nZ=1 zener legitimately leaks ~100 uA —
   * "blocking" tests must sit WELL below vz, not graze the knee.)
   * Source: closed-form breakdownDiodeCurrent at v=-3 V.
   */
  it("passes less than 1 nA at 3V reverse-bias (2.1V below vz)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 470 } },
        // Cathode (k) toward +, anode (a) toward GND — reverse-biased (standard clamp orientation)
        { id: "z1", kind: "zener_diode", pins: [{ id: "a" }, { id: "k" }], params: { vf: 0.7, vz: 5.1, iRated: 0.1, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "z1", to_pin: "k" },
        { from_component: "z1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    const i_A = eng.getElementI()["z1"] ?? 0;
    expect(Math.abs(i_A)).toBeLessThan(1e-9); // < 1 nA
  });

  it("node voltage stays near supply (no regulation well below vz)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 470 } },
        { id: "z1", kind: "zener_diode", pins: [{ id: "a" }, { id: "k" }], params: { vf: 0.7, vz: 5.1, iRated: 0.1, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "z1", to_pin: "k" },
        { from_component: "z1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    // No significant current → voltage drop across R is negligible → cathode ≈ 3V
    const vk = netVoltage(eng, "z1", "k");
    expect(vk).toBeGreaterThan(2.9);
  });
});

describe("zener_diode — regulation at vz (the point of the part)", () => {
  /**
   * Circuit: 9 V source → 470 Ω → 5.1 V zener reverse-biased → GND.
   *
   * Closed form: I = (9 - V_z)/470 with V_z = vz + nZ*Vt*ln(I/izKnee).
   * Iterating: I ≈ 8.2 mA → V_z = 5.1 + 0.02585*ln(8.2/5) ≈ 5.113 V.
   * P_zener ≈ 42 mW, P_resistor ≈ 32 mW — both comfortably inside ratings,
   * so no failure-framework interference (a prior over-stressed test design
   * cooked its own series resistor and asserted on the artifact).
   */
  it("regulates the cathode at ~5.1V with ~8mA through the loop", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 9 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 470 } },
        { id: "z1", kind: "zener_diode", pins: [{ id: "a" }, { id: "k" }], params: { vf: 0.7, vz: 5.1, iRated: 0.1, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "z1", to_pin: "k" },
        { from_component: "z1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vk = netVoltage(eng, "z1", "k");
    const i_mA = Math.abs(eng.getElementI()["z1"] ?? 0) * 1000;
    expect(vk).toBeGreaterThan(5.05);
    expect(vk).toBeLessThan(5.2);
    expect(i_mA).toBeGreaterThan(7.5);
    expect(i_mA).toBeLessThan(8.7);
  });
});

// ─── Task 4: TVS diode ────────────────────────────────────────────────────────

describe("tvs_diode — unidirectional blocking below vbr", () => {
  /**
   * Circuit: 5 V source → 100 Ω → TVS cathode (vbr=6.8 V, unidirectional) → GND.
   *
   * At vAK = -5 V (1.8 V below the knee): overdrive = -(v + vbr) = -1.8 V →
   *   iRev = izKnee * exp(-1.8/0.02585) = 0.005 * e^-69.6 ≈ 3e-33 A — nothing.
   * Source: closed-form breakdownDiodeCurrent at v=-5 V.
   */
  it("blocks at 5V (below vbr=6.8V)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
        { id: "d1", kind: "tvs_diode", pins: [{ id: "a" }, { id: "k" }], params: { vbr: 6.8, bidirectional: 0, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "d1", to_pin: "k" },
        { from_component: "d1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    const i_A = Math.abs(eng.getElementI()["d1"] ?? 0);
    expect(i_A).toBeLessThan(1e-9); // < 1 nA — blocking
  });
});

describe("tvs_diode — unidirectional clamping at vbr", () => {
  /**
   * Circuit: 12 V source → 220 Ω → TVS cathode (vbr=6.8 V) → GND.
   *
   * Closed form: I = (12 - V)/220 with V = vbr + nZ*Vt*ln(I/izKnee).
   * Iterating: I ≈ 23.5 mA → V ≈ 6.84 V.
   * P_tvs ≈ 0.16 W (< 0.4 W rating), P_resistor ≈ 0.12 W (< 0.25 W) — the
   * circuit survives indefinitely; no failure-framework interference.
   */
  it("clamps at 12V: cathode held near 6.8V, ~23mA flows", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 12 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
        { id: "d1", kind: "tvs_diode", pins: [{ id: "a" }, { id: "k" }], params: { vbr: 6.8, bidirectional: 0, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "d1", to_pin: "k" },
        { from_component: "d1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vCathode = netVoltage(eng, "d1", "k");
    const i_A = Math.abs(eng.getElementI()["d1"] ?? 0);
    expect(vCathode).toBeGreaterThan(6.8);
    expect(vCathode).toBeLessThan(6.95);
    expect(i_A).toBeGreaterThan(0.020);
    expect(i_A).toBeLessThan(0.026);
  });
});

describe("tvs_diode — bidirectional symmetric clamping", () => {
  /**
   * Circuit: 15 V source → 330 Ω → bidirectional TVS (vbr=6.8) → GND.
   *
   * Closed form per polarity: I = (15 - V)/330 with V = vbr + nZ*Vt*ln(I/izKnee).
   * Iterating: I ≈ 24.7 mA → V ≈ 6.84 V. The bidirectional i(v) is odd-symmetric
   * (i(-v) = -i(v)), so flipping the part yields the same |V| and |I|.
   * P_tvs ≈ 0.17 W, P_resistor ≈ 0.20 W — inside ratings, no failure interference.
   */
  it("clamps in forward polarity: anode held near 6.8V, ~25mA", () => {
    const circuitFwd: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 15 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 330 } },
        { id: "d1", kind: "tvs_diode", pins: [{ id: "a" }, { id: "k" }], params: { vbr: 6.8, bidirectional: 1, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "d1", to_pin: "a" },
        { from_component: "d1", from_pin: "k", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuitFwd);
    expect(eng.lastConverged).toBe(true);
    const vAnode = netVoltage(eng, "d1", "a");
    const i_A = Math.abs(eng.getElementI()["d1"] ?? 0);
    expect(vAnode).toBeGreaterThan(6.8);
    expect(vAnode).toBeLessThan(6.95);
    expect(i_A).toBeGreaterThan(0.022);
    expect(i_A).toBeLessThan(0.027);
  });

  it("clamps symmetrically in reversed polarity: same |V| and |I|", () => {
    const circuitRev: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 15 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 330 } },
        // Cathode toward source — reverse polarity
        { id: "d1", kind: "tvs_diode", pins: [{ id: "a" }, { id: "k" }], params: { vbr: 6.8, bidirectional: 1, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "d1", to_pin: "k" },
        { from_component: "d1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuitRev);
    expect(eng.lastConverged).toBe(true);
    const vCathode = netVoltage(eng, "d1", "k");
    const i_A = Math.abs(eng.getElementI()["d1"] ?? 0);
    expect(vCathode).toBeGreaterThan(6.8);
    expect(vCathode).toBeLessThan(6.95);
    expect(i_A).toBeGreaterThan(0.022);
    expect(i_A).toBeLessThan(0.027);
  });
});

// ─── Task 5: PTC resettable fuse ─────────────────────────────────────────────

describe("ptc_fuse — normal conduction below hold current", () => {
  /**
   * Circuit: 0.2 V source directly across PTC (rNormal=0.5 Ω, iHold=0.5 A).
   * I = Vs / rNormal = 0.2 / 0.5 = 0.4 A.
   * 0.4 A < iHold = 0.5 A → trip condition: current > 2*iHold = 1.0 A is false.
   * No stress accumulates. PTC stays in low-R state after 200 ms.
   * Source: Ohm's law; prototype engine run confirmed tripped=false, I≈0.4A.
   */
  it("carries 0.4A (< iHold=0.5A) without tripping after 200ms", () => {
    const circuit: SimCircuit = {
      components: [
        // 0.2V directly across PTC avoids series-resistor overload failure
        // (catalog resistors trip at P > 0.25W × 2 for RESISTOR_OVERLOAD_SECONDS)
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0.2 } },
        { id: "p1", kind: "ptc_fuse", pins: [{ id: "a" }, { id: "b" }], params: { iHold: 0.5, rNormal: 0.5 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "p1", to_pin: "a" },
        { from_component: "p1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    // 200 steps × 1 ms = 200 ms of simulated time
    const eng = runEngine(circuit, 200, 1e-3);
    const i_A = Math.abs(eng.getElementI()["p1"] ?? 0);
    // I = 0.2V / 0.5Ω = 0.4A (Ohm's law)
    expect(i_A).toBeGreaterThan(0.35);
    expect(i_A).toBeLessThan(0.45);
    expect(eng.getPtcTripped().has("p1")).toBe(false);
  });
});

describe("ptc_fuse — trip under 3A overcurrent", () => {
  /**
   * Circuit: 1.5 V source directly across PTC (rNormal=0.5 Ω, iHold=0.5 A).
   * I = 1.5 / 0.5 = 3.0 A = 6 × iHold, which exceeds 2 × iHold = 1.0 A.
   * Trip threshold: iHold² × 0.1 = 0.25 × 0.1 = 0.025 A²s.
   * Trip time: 0.025 / (3.0² ) = 0.025 / 9 ≈ 2.78 ms.
   * After trip: R = 200 × rNormal = 100 Ω, I = 1.5 / 100 = 0.015 A.
   * Source: engine trip state machine (sim-engine.ts _updateFailureStates);
   * trip threshold formula i²·h cited in code comment; prototype confirmed.
   */
  it("trips within 50ms sim-time and raises resistance 200x", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.5 } },
        { id: "p1", kind: "ptc_fuse", pins: [{ id: "a" }, { id: "b" }], params: { iHold: 0.5, rNormal: 0.5 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "p1", to_pin: "a" },
        { from_component: "p1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    // 50ms at 1ms steps — well past trip time of ~2.78ms
    const eng = runEngine(circuit, 50, 1e-3);
    expect(eng.getPtcTripped().has("p1")).toBe(true);
    // After trip: I = Vs / (200 × rNormal) = 1.5 / 100 = 0.015 A (Ohm's law)
    const i_tripped_A = Math.abs(eng.getElementI()["p1"] ?? 0);
    expect(i_tripped_A).toBeGreaterThan(0.01);
    expect(i_tripped_A).toBeLessThan(0.02);
  });

  it("resistance ratio is approximately 200x after trip", () => {
    // Measure R_tripped / R_normal using V/I at 1.5V
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.5 } },
        { id: "p1", kind: "ptc_fuse", pins: [{ id: "a" }, { id: "b" }], params: { iHold: 0.5, rNormal: 0.5 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "p1", to_pin: "a" },
        { from_component: "p1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    const engTripped = runEngine(circuit, 50, 1e-3);
    const i_tripped = Math.abs(engTripped.getElementI()["p1"] ?? 1);
    // R_eff = V / I = 1.5 / 0.015 = 100 Ω = 200 × rNormal (0.5 Ω)
    const r_eff = 1.5 / i_tripped;
    expect(r_eff).toBeGreaterThan(80);   // at least 160x rNormal
    expect(r_eff).toBeLessThan(150);     // at most 300x rNormal (Ohm's law ratio)
  });
});

describe("ptc_fuse — recovery after 2s below iHold/2", () => {
  /**
   * Procedure:
   *   1. Trip PTC: run 1.5V circuit for 50ms (trips in ~2.78ms).
   *   2. Load recovery circuit: 0.01V → I = 0.02A << iHold/2 = 0.25A.
   *   3. Run 1 s of sim — recovery timer accumulates but 2s not reached.
   *   4. Run 1.5 s more (total > 2s) — should now be untripped.
   * Recovery condition: current < iHold/2 for 2 continuous seconds.
   * Source: engine recovery logic (sim-engine.ts _updateFailureStates comment).
   */
  it("stays tripped after 1s at low current, recovers after 2.5s", () => {
    const tripCircuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.5 } },
        { id: "p1", kind: "ptc_fuse", pins: [{ id: "a" }, { id: "b" }], params: { iHold: 0.5, rNormal: 0.5 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "p1", to_pin: "a" },
        { from_component: "p1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = new SimEngine();
    eng.load(tripCircuit);
    for (let i = 0; i < 50; i++) eng.step(1e-3); // 50ms — PTC tripped
    expect(eng.getPtcTripped().has("p1")).toBe(true);

    // Switch to low-current circuit. load() carries PTC state forward by
    // component ID (same "p1" id), so the tripped flag persists.
    const recCircuit: SimCircuit = {
      components: [
        // 10 mV → I = 0.01 / 0.5 = 0.02A (when tripped: 0.01/100 = 0.0001A) — both << iHold/2=0.25A
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0.01 } },
        { id: "p1", kind: "ptc_fuse", pins: [{ id: "a" }, { id: "b" }], params: { iHold: 0.5, rNormal: 0.5 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "p1", to_pin: "a" },
        { from_component: "p1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    eng.load(recCircuit);
    // Still tripped at load time (state carried forward)
    expect(eng.getPtcTripped().has("p1")).toBe(true);

    // Run 1s — recoveryTime < 2s → still tripped
    for (let i = 0; i < 1000; i++) eng.step(1e-3);
    expect(eng.getPtcTripped().has("p1")).toBe(true);

    // Run 1.5s more (total > 2s) → recoveryTime >= 2.0s → untripped
    for (let i = 0; i < 1500; i++) eng.step(1e-3);
    expect(eng.getPtcTripped().has("p1")).toBe(false);
  });
});

describe("ptc_fuse — resetFailures clears PTC trip state", () => {
  /**
   * resetFailures() is documented to clear PTC state in addition to latched
   * failures — verified by reading resetFailures() in sim-engine.ts which
   * iterates ptcs and resets tripped/tripStress/recoveryTime.
   */
  it("clears tripped flag and stress accumulator", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.5 } },
        { id: "p1", kind: "ptc_fuse", pins: [{ id: "a" }, { id: "b" }], params: { iHold: 0.5, rNormal: 0.5 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "p1", to_pin: "a" },
        { from_component: "p1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = new SimEngine();
    eng.load(circuit);
    for (let i = 0; i < 50; i++) eng.step(1e-3);
    expect(eng.getPtcTripped().has("p1")).toBe(true);

    eng.resetFailures();
    expect(eng.getPtcTripped().has("p1")).toBe(false);
    // Verify the PTC returns to normal conduction immediately after reset
    // (current rises back to 1.5/0.5 = 3A before tripping again)
    eng.step(1e-5); // single short step
    const i_after_reset = Math.abs(eng.getElementI()["p1"] ?? 0);
    // Expect normal rNormal conduction: V/rNormal = 1.5/0.5 = 3.0A
    expect(i_after_reset).toBeGreaterThan(1.0);
  });
});

// ─── Diagnostics: zener-over-dissipation and ptc-tripped findings ─────────────

describe("diagnostics: zener-over-dissipation", () => {
  /**
   * The zener-over-dissipation finding fires when |vAK × current| > p_max × 1.1.
   * We use tvs_diode which has catalog p_max = 0.4 W.
   *
   * Circuit sizing matters: the TVS must over-dissipate while the SERIES
   * RESISTOR survives, or the Wave E failure framework opens the resistor and
   * kills the circuit before the finding can fire (a prior version of this
   * test put 5.3 W into a 0.25 W resistor and asserted on the dead circuit).
   * 9 V → 22 Ω → 6.8 V TVS: I = (9 - 6.88)/22 ≈ 96 mA →
   *   P_tvs ≈ 6.88 × 0.096 ≈ 0.66 W  > 0.44 W threshold (fires)
   *   P_resistor ≈ 0.096^2 × 22 ≈ 0.20 W < 0.25 W rating (survives)
   */
  it("fires zener-over-dissipation when TVS dissipates above p_max", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 9 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 22 } },
        { id: "d1", kind: "tvs_diode", pins: [{ id: "a" }, { id: "k" }], params: { vbr: 6.8, bidirectional: 0, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "d1", to_pin: "k" },
        { from_component: "d1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 1200, 1e-4); // 120ms > 50ms simTime guard
    const findings = analyzeLive(makeInput(circuit, liveFromEngine(eng)));
    const diss = findings.find((f) => f.id === "zener-over-dissipation");
    expect(diss).toBeDefined();
    expect(diss?.severity).toBe("warning");
    expect(diss?.componentIds).toContain("d1");
  });

  it("does NOT fire reverse-overvoltage for zener or TVS (intended breakdown)", () => {
    // Verify the finding 6 exclusion: reverse-overvoltage only fires for 'diode'
    // and 'schottky_diode', not for zener_diode or tvs_diode.
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 30 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
        { id: "d1", kind: "tvs_diode", pins: [{ id: "a" }, { id: "k" }], params: { vbr: 6.8, bidirectional: 0, iz_knee: 0.005 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "d1", to_pin: "k" },
        { from_component: "d1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 1200, 1e-4);
    const findings = analyzeLive(makeInput(circuit, liveFromEngine(eng)));
    const revOv = findings.find((f) => f.id === "reverse-overvoltage");
    // tvs_diode is not in the reverse-overvoltage check (finding 6) — should not fire
    expect(revOv).toBeUndefined();
  });
});

describe("diagnostics: ptc-tripped finding", () => {
  /**
   * The ptc-tripped finding (info severity) fires when ptcTripped contains
   * the component id. It uses the live.ptcTripped set passed through the
   * diagnostics pipeline.
   */
  it("fires when ptcTripped set contains the PTC component id", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.5 } },
        { id: "p1", kind: "ptc_fuse", pins: [{ id: "a" }, { id: "b" }], params: { iHold: 0.5, rNormal: 0.5 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "p1", to_pin: "a" },
        { from_component: "p1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    // Run until tripped
    const eng = runEngine(circuit, 50, 1e-3);
    const ptcSet = eng.getPtcTripped();
    expect(ptcSet.has("p1")).toBe(true);

    // Use engine simTime > 50ms to pass the simTime guard in analyzeLive
    const live = liveFromEngine(eng, ptcSet);
    const findings = analyzeLive(makeInput(circuit, live));
    const ptcFinding = findings.find((f) => f.id === "ptc-tripped");
    expect(ptcFinding).toBeDefined();
    expect(ptcFinding?.severity).toBe("info");
    expect(ptcFinding?.componentIds).toContain("p1");
  });

  it("does NOT fire when PTC is not tripped", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0.2 } },
        { id: "p1", kind: "ptc_fuse", pins: [{ id: "a" }, { id: "b" }], params: { iHold: 0.5, rNormal: 0.5 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "p1", to_pin: "a" },
        { from_component: "p1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 1200, 1e-4); // 120ms — past simTime guard, not tripped
    const ptcSet = eng.getPtcTripped();
    expect(ptcSet.has("p1")).toBe(false);

    const live = liveFromEngine(eng, ptcSet);
    const findings = analyzeLive(makeInput(circuit, live));
    const ptcFinding = findings.find((f) => f.id === "ptc-tripped");
    expect(ptcFinding).toBeUndefined();
  });
});
