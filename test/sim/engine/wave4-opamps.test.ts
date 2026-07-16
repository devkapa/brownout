/**
 * Wave 4.1 op-amp engine tests: LM358, MCP6002, LM386.
 *
 * All numeric expectations are derived from first-principles circuit analysis.
 * No value is read back from the implementation and then asserted — that would
 * allow a self-confirming broken model to pass (the cardinal sin from prior waves).
 *
 * Derivation comments precede each expect() call.
 */

import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const catalog = rawCatalog as unknown as PartCatalog;

// ─── Helpers ──────────────────────────────────────────────────────────────────

function runEngine(circuit: SimCircuit, steps = 100, dt = 1e-4): SimEngine {
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

// ─── Catalog validation ───────────────────────────────────────────────────────

describe("catalog entries exist", () => {
  it("lm358 part is defined and correct", () => {
    const part = catalog.parts.find((p) => p.uid === "lm358");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("lm358");
    expect(part?.electrical_specs?.vcc_range?.min).toBe(3);
    expect(part?.electrical_specs?.vcc_range?.max).toBe(32);
    // Output headroom note must mention 1.5 V
    expect(part?.electrical_specs?.notes).toContain("1.5");
    expect(part?.bom?.package).toBe("DIP-8");
  });

  it("mcp6002 part is defined and correct", () => {
    const part = catalog.parts.find((p) => p.uid === "mcp6002");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("mcp6002");
    expect(part?.electrical_specs?.vcc_range?.min).toBe(1.8);
    expect(part?.electrical_specs?.vcc_range?.max).toBe(6);
    expect(part?.bom?.package).toBe("DIP-8");
  });

  it("lm386 part is defined with gain param and enumeration", () => {
    const part = catalog.parts.find((p) => p.uid === "lm386");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("lm386");
    expect(part?.default_params?.gain).toBe(20);
    expect(part?.paramTypes?.gain).toBe("enum");
    expect(part?.paramEnums?.gain).toEqual(["20", "50", "200"]);
    expect(part?.electrical_specs?.vcc_range?.min).toBe(4);
    expect(part?.electrical_specs?.vcc_range?.max).toBe(12);
    expect(part?.bom?.package).toBe("DIP-8");
  });
});

// ─── LM358 — voltage follower ─────────────────────────────────────────────────

describe("LM358 unit 1 — voltage follower", () => {
  /**
   * Circuit: 5 V supply (v1) powers LM358 (vcc=pin8, gnd=pin4).
   * Vin = 2.0 V source drives in1+ (pin 3).
   * Feedback: out1 (pin 1) tied to in1- (pin 2) via a wire.
   *
   * Closed-loop analysis (A = 1000, the engine's Newton-stable gain):
   *   V_out = A * (V_in+ - V_in-)
   *   V_in- = V_out  (feedback wire)
   *   V_out = A * (2.0 - V_out)
   *   V_out * (1 + A) = A * 2.0
   *   V_out = A * 2.0 / (1 + A) = 1000 * 2.0 / 1001 ≈ 1.9980 V
   *   Error from ideal: 2.0 * 1/1001 ≈ 2.0 mV (< 0.1 %)
   *
   * Why A=1000 (not 1e5): with A=1e5, supply rails read 0 V from xGuess on
   * Newton iteration 1, causing saturation-regime flips between iterations that
   * prevent convergence within the 25-iter budget.  A=1000 avoids the flip
   * while keeping closed-loop error well below any measurable tolerance.
   *
   * Saturation check: vHigh = 5 - 1.5 = 3.5 V, vLow = 0 + 0.02 = 0.02 V.
   * 1.998 V is within range — linear regime applies.
   */
  it("output ≈ 2.0 V (within ±10 mV)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 2.0 } },
        { id: "oa1", kind: "lm358", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        // Power rails
        { from_component: "v1",  from_pin: "pos", to_component: "oa1",  to_pin: "8" },  // VCC
        { from_component: "v1",  from_pin: "neg", to_component: "oa1",  to_pin: "4" },  // GND
        // Input
        { from_component: "vin", from_pin: "pos", to_component: "oa1",  to_pin: "3" },  // in1+
        { from_component: "vin", from_pin: "neg", to_component: "v1",   to_pin: "neg" },
        // Feedback: out1 → in1-
        { from_component: "oa1", from_pin: "1",   to_component: "oa1",  to_pin: "2" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "oa1", "1");
    // V_out = 1000 * 2.0 / 1001 ≈ 1.9980 V; accept ±10 mV (well within tolerance)
    expect(vout).toBeGreaterThan(1.990);
    expect(vout).toBeLessThan(2.010);
  });
});

// ─── LM358 — non-inverting amplifier (gain = 2) ───────────────────────────────

describe("LM358 unit 1 — non-inverting amplifier gain 2", () => {
  /**
   * Circuit: 5 V supply. Vin = 0.5 V on in1+ (pin 3).
   * Rf = 10 kΩ from out1 (pin 1) to in1- (pin 2).
   * Rg = 10 kΩ from in1- (pin 2) to GND.
   *
   * Ideal non-inverting gain: Vout = Vin * (1 + Rf/Rg) = 0.5 * 2 = 1.0 V
   *
   * Precise closed-loop with A = 1000 (engine's Newton-stable gain):
   *   V_minus = Vout * Rg / (Rf + Rg) = Vout / 2
   *   Vout = A * (Vin - Vout/2)
   *   Vout * (1 + A/2) = A * Vin
   *   Vout = A * 0.5 / (1 + A/2) = 1000 * 0.5 / 501 ≈ 0.99800 V
   *   Error from ideal (1.0 V): 0.2 % (well below ±20 mV tolerance)
   *
   * Saturation check: vHigh = 3.5 V, vLow = 0.02 V; ~1.0 V in range.
   */
  it("output ≈ 1.0 V for Vin=0.5 V with Rf=Rg=10 kΩ (within ±20 mV)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0.5 } },
        { id: "rf",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "rg",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "oa1", kind: "lm358", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        // Power
        { from_component: "v1",  from_pin: "pos", to_component: "oa1",  to_pin: "8" },
        { from_component: "v1",  from_pin: "neg", to_component: "oa1",  to_pin: "4" },
        // Non-inverting input
        { from_component: "vin", from_pin: "pos", to_component: "oa1",  to_pin: "3" },
        { from_component: "vin", from_pin: "neg", to_component: "v1",   to_pin: "neg" },
        // Feedback network: out1 → Rf → in1- → Rg → GND
        { from_component: "oa1", from_pin: "1",   to_component: "rf",   to_pin: "a" },
        { from_component: "rf",  from_pin: "b",   to_component: "oa1",  to_pin: "2" },
        { from_component: "oa1", from_pin: "2",   to_component: "rg",   to_pin: "a" },
        { from_component: "rg",  from_pin: "b",   to_component: "v1",   to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "oa1", "1");
    // Ideal: 0.5*(1+1) = 1.0 V; precise (A=1000): 1000*0.5/501 ≈ 0.99800 V
    expect(vout).toBeGreaterThan(0.980);
    expect(vout).toBeLessThan(1.020);
  });
});

// ─── LM358 — inverting amplifier ─────────────────────────────────────────────

describe("LM358 unit 1 — inverting amplifier with mid-rail reference", () => {
  /**
   * Single-supply inverting amp using V+ = 2.5 V reference on in1+.
   * Vin = 1.0 V, Rin = 10 kΩ, Rf = 5 kΩ.
   *
   * Ideal analysis (virtual ground at V+, A → ∞):
   *   V_minus ≈ V_plus = 2.5 V
   *   I_in = (Vin - V_minus) / Rin = (1.0 - 2.5) / 10000 = -1.5e-4 A
   *   Vout = V_minus - I_in * Rf = 2.5 - (-1.5e-4 * 5000) = 2.5 + 0.75 = 3.25 V
   *
   * Finite-gain correction (A = 1000): the virtual-ground assumption holds well
   * at A=1000 since the inverting gain Rf/Rin = 0.5 means the loop gain is ~2000.
   * The error on Vout is < 0.1 %, i.e. well within the ±50 mV window.
   *
   * Saturation check: vHigh = 5 - 1.5 = 3.5 V; 3.25 V < 3.5 V — linear regime.
   */
  it("output ≈ 3.25 V for Vin=1.0 V, Rf/Rin=0.5 with 2.5 V reference (within ±50 mV)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1",   kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vin",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.0 } },
        // Mid-rail reference divider: 10k+10k → 2.5 V at node
        { id: "rd1",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "rd2",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        // Inverting topology
        { id: "rin",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "rf",   kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 5000 } },
        { id: "oa1",  kind: "lm358", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        // Power
        { from_component: "v1",  from_pin: "pos", to_component: "oa1",  to_pin: "8" },
        { from_component: "v1",  from_pin: "neg", to_component: "oa1",  to_pin: "4" },
        // Reference divider: 5V → rd1 → mid → rd2 → GND; mid drives in1+
        { from_component: "v1",  from_pin: "pos", to_component: "rd1",  to_pin: "a" },
        { from_component: "rd1", from_pin: "b",   to_component: "rd2",  to_pin: "a" },
        { from_component: "rd2", from_pin: "b",   to_component: "v1",   to_pin: "neg" },
        { from_component: "rd1", from_pin: "b",   to_component: "oa1",  to_pin: "3" },
        // Input: Vin → Rin → in1-
        { from_component: "vin", from_pin: "pos", to_component: "rin",  to_pin: "a" },
        { from_component: "vin", from_pin: "neg", to_component: "v1",   to_pin: "neg" },
        { from_component: "rin", from_pin: "b",   to_component: "oa1",  to_pin: "2" },
        // Feedback: out1 → Rf → in1-
        { from_component: "oa1", from_pin: "1",   to_component: "rf",   to_pin: "a" },
        { from_component: "rf",  from_pin: "b",   to_component: "oa1",  to_pin: "2" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "oa1", "1");
    // Ideal: 3.25 V; accept ±50 mV to accommodate finite loop-gain error
    expect(vout).toBeGreaterThan(3.200);
    expect(vout).toBeLessThan(3.300);
  });
});

// ─── LM358 — high-side saturation clip ───────────────────────────────────────

describe("LM358 — saturation at vHigh = vcc − 1.5 V", () => {
  /**
   * Non-inverting amplifier gain ×3 (Rf=20k, Rg=10k) with Vin = 2.0 V.
   * 5 V supply.
   *
   * Ideal gain: Vout = 2.0 * (1 + 20k/10k) = 2.0 * 3 = 6.0 V.
   * But vHigh (LM358) = 5 - 1.5 = 3.5 V.
   * So output clamps to 3.5 V.
   *
   * Derivation: the ideal output exceeds vHigh, so the saturated regime
   * applies: stampOpAmp writes b[k] = vHigh = 3.5 V.
   */
  it("LM358 clips at vcc−1.5 = 3.5 V when ideal gain would produce 6.0 V", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 2.0 } },
        { id: "rf",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 20000 } },
        { id: "rg",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "oa1", kind: "lm358", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        { from_component: "v1",  from_pin: "pos", to_component: "oa1",  to_pin: "8" },
        { from_component: "v1",  from_pin: "neg", to_component: "oa1",  to_pin: "4" },
        { from_component: "vin", from_pin: "pos", to_component: "oa1",  to_pin: "3" },
        { from_component: "vin", from_pin: "neg", to_component: "v1",   to_pin: "neg" },
        { from_component: "oa1", from_pin: "1",   to_component: "rf",   to_pin: "a" },
        { from_component: "rf",  from_pin: "b",   to_component: "oa1",  to_pin: "2" },
        { from_component: "oa1", from_pin: "2",   to_component: "rg",   to_pin: "a" },
        { from_component: "rg",  from_pin: "b",   to_component: "v1",   to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "oa1", "1");
    // LM358 vHigh = 5 - 1.5 = 3.5 V; accept ±50 mV
    expect(vout).toBeGreaterThan(3.450);
    expect(vout).toBeLessThan(3.550);
  });

  /**
   * MCP6002 same circuit — same topology, but rail-to-rail: vHigh = 5 − 0.02 = 4.98 V.
   * Ideal gain = 6.0 V → clamps at 4.98 V (not 3.5 V).
   * This confirms the headroom difference between LM358 and MCP6002 is real.
   */
  it("MCP6002 clips at vcc−0.02 = 4.98 V (rail-to-rail, different headroom from LM358)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 2.0 } },
        { id: "rf",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 20000 } },
        { id: "rg",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "oa1", kind: "mcp6002", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        { from_component: "v1",  from_pin: "pos", to_component: "oa1",  to_pin: "8" },
        { from_component: "v1",  from_pin: "neg", to_component: "oa1",  to_pin: "4" },
        { from_component: "vin", from_pin: "pos", to_component: "oa1",  to_pin: "3" },
        { from_component: "vin", from_pin: "neg", to_component: "v1",   to_pin: "neg" },
        { from_component: "oa1", from_pin: "1",   to_component: "rf",   to_pin: "a" },
        { from_component: "rf",  from_pin: "b",   to_component: "oa1",  to_pin: "2" },
        { from_component: "oa1", from_pin: "2",   to_component: "rg",   to_pin: "a" },
        { from_component: "rg",  from_pin: "b",   to_component: "v1",   to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "oa1", "1");
    // MCP6002 vHigh = 5 - 0.02 = 4.98 V; accept ±50 mV
    expect(vout).toBeGreaterThan(4.930);
    expect(vout).toBeLessThan(5.030);
  });
});

// ─── MCP6002 — follower at 1.8 V supply ──────────────────────────────────────

describe("MCP6002 — voltage follower at minimum 1.8 V supply", () => {
  /**
   * 1.8 V supply, Vin = 0.9 V (half-supply) on in1+, feedback out1→in1-.
   *
   * Closed-loop analysis (A = 1000, same as LM358 follower derivation):
   *   Vout = A * Vin / (1 + A) = 1000 * 0.9 / 1001 ≈ 0.89910 V
   *   Error from ideal: 0.9 * 1/1001 ≈ 0.9 mV (< 0.1 %)
   *
   * Saturation check: vHigh = 1.8 - 0.02 = 1.78 V, vLow = 0 + 0.02 = 0.02 V.
   * ~0.9 V is in range.
   *
   * Also verifies the MCP6002 works at its rated minimum supply.
   */
  it("output ≈ 0.9 V at 1.8 V supply (within ±20 mV)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.8 } },
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0.9 } },
        { id: "oa1", kind: "mcp6002", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        { from_component: "v1",  from_pin: "pos", to_component: "oa1",  to_pin: "8" },
        { from_component: "v1",  from_pin: "neg", to_component: "oa1",  to_pin: "4" },
        { from_component: "vin", from_pin: "pos", to_component: "oa1",  to_pin: "3" },
        { from_component: "vin", from_pin: "neg", to_component: "v1",   to_pin: "neg" },
        { from_component: "oa1", from_pin: "1",   to_component: "oa1",  to_pin: "2" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "oa1", "1");
    // Ideal: 0.9 V; A=1000 gives 0.89910 V; accept ±20 mV
    expect(vout).toBeGreaterThan(0.880);
    expect(vout).toBeLessThan(0.920);
  });
});

// ─── LM386 — fixed-gain amplifier ────────────────────────────────────────────

describe("LM386 — audio amplifier DC bias model", () => {
  /**
   * 5 V supply. +IN (pin 3) = 2.5 V + delta. -IN (pin 2) = 0 V (GND).
   * gain param = 20 (default).
   *
   * Model: Vout = clamp(Vmid + gain*(v+ - v-), vLow, vHigh)
   *   Vmid = (vcc + gnd) / 2 = (5 + 0) / 2 = 2.5 V
   *   vHigh = 5 - 0.5 = 4.5 V
   *   vLow  = 0 + 0.5 = 0.5 V
   *
   * With delta = 0.05 V (i.e. v+ = 2.55 V, v- = 0):
   *   vIdeal = 2.5 + 20 * (2.55 - 0) = 2.5 + 51.0 = 53.5 V  → saturates to vHigh = 4.5 V
   *
   * Wait — the model is Vout = Vmid + gain*(v+ - v-) where v+/v- are the INPUT
   * differential, not referenced to ground. In the stamp, gain acts as the open-loop
   * A coefficient and vOffset injects Vmid. The linear input is pin3 - pin2 = v+ - v-.
   *
   * For a valid unsaturated test we need a small differential signal from a floating
   * source whose common-mode is 0.  Let delta = 0.05 V total differential
   * (pin3 = 0.05 V, pin2 = 0 V):
   *   vIdeal = 2.5 + 20 * (0.05 - 0) = 2.5 + 1.0 = 3.5 V
   *   3.5 V < 4.5 V and > 0.5 V → linear regime.
   *   Closed-loop: the model has no negative feedback, so the result equals the
   *   stamp equation directly (no feedback resistor → open-loop single-use stamp).
   *   Vout ≈ 3.5 V
   *
   * Note: the LM386 in this DC model does NOT have a feedback resistor in the
   * typical test circuit — that is the intended datasheet use (no external feedback).
   * The output directly implements Vmid + gain*(v+ - v-).
   */
  it("output ≈ 3.5 V for gain=20, +IN=0.05 V above GND, -IN=GND (within ±100 mV)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        // Drive +IN (pin3) with 0.05 V; -IN (pin2) is undriven (floats to GND)
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0.05 } },
        // Load resistor on output (typical 8 Ω speaker represented here as 1 kΩ to avoid huge currents in DC model)
        { id: "rl",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "lm1", kind: "lm386", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: { gain: 20 } },
      ],
      wires: [
        // Power: VS (pin 6) and GND (pin 4)
        { from_component: "vs",  from_pin: "pos", to_component: "lm1",  to_pin: "6" },
        { from_component: "vs",  from_pin: "neg", to_component: "lm1",  to_pin: "4" },
        // +IN = 0.05 V
        { from_component: "vin", from_pin: "pos", to_component: "lm1",  to_pin: "3" },
        { from_component: "vin", from_pin: "neg", to_component: "vs",   to_pin: "neg" },
        // Load: VOUT (pin5) → rl → GND
        { from_component: "lm1", from_pin: "5",   to_component: "rl",   to_pin: "a" },
        { from_component: "rl",  from_pin: "b",   to_component: "vs",   to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "lm1", "5");
    // Ideal: Vmid + gain*(0.05-0) = 2.5 + 20*0.05 = 2.5 + 1.0 = 3.5 V; accept ±100 mV
    expect(vout).toBeGreaterThan(3.400);
    expect(vout).toBeLessThan(3.600);
  });

  /**
   * LM386 saturation test: +IN = 0.5 V differential → ideal = 2.5 + 20*0.5 = 12.5 V
   * → clamps to vHigh = 5 - 0.5 = 4.5 V.
   *
   * vHigh = VS - 0.5 = 5 - 0.5 = 4.5 V
   */
  it("output saturates to vHigh = VS-0.5 = 4.5 V when ideal exceeds rail", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0.5 } },
        { id: "rl",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "lm1", kind: "lm386", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: { gain: 20 } },
      ],
      wires: [
        { from_component: "vs",  from_pin: "pos", to_component: "lm1",  to_pin: "6" },
        { from_component: "vs",  from_pin: "neg", to_component: "lm1",  to_pin: "4" },
        { from_component: "vin", from_pin: "pos", to_component: "lm1",  to_pin: "3" },
        { from_component: "vin", from_pin: "neg", to_component: "vs",   to_pin: "neg" },
        { from_component: "lm1", from_pin: "5",   to_component: "rl",   to_pin: "a" },
        { from_component: "rl",  from_pin: "b",   to_component: "vs",   to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "lm1", "5");
    // vHigh = 5 - 0.5 = 4.5 V; accept ±50 mV
    expect(vout).toBeGreaterThan(4.450);
    expect(vout).toBeLessThan(4.550);
  });
});

// ─── Unpowered op-amp — no NaN, matrix solves ────────────────────────────────

describe("unpowered op-amp — matrix remains non-singular, no crash, no NaN", () => {
  /**
   * LM358 with NO supply wires (pins 4 and 8 open).
   * Input: 1.0 V on in1+ (pin 3), feedback out1→in1-.
   *
   * When supply pins are open, _icPowerInfo returns powered=false with
   * vcc = 0 (xGuess gives 0 for open/gnd nodes), gnd = 0.
   * stampOpAmp is ALWAYS called (no `if (!powered) break` for VCVS parts).
   *
   * vHigh = 0 - 1.5 = -1.5 V, vLow = 0 + 0.02 = 0.02 V.
   * vIdeal = 0 (inputs effectively at 0 V through a follower).
   * 0 < 0.02 → saturates to vLow = 0.02 V.
   *
   * Key assertions: no NaN, converged = true, Vout is a finite number.
   */
  it("LM358 without supply wires: converges, Vout is finite (not NaN)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.0 } },
        { id: "oa1", kind: "lm358", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        // Input but NO power pins wired
        { from_component: "vin", from_pin: "pos", to_component: "oa1",  to_pin: "3" },
        { from_component: "oa1", from_pin: "1",   to_component: "oa1",  to_pin: "2" },
      ],
    };
    const eng = runEngine(circuit, 10);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "oa1", "1");
    expect(Number.isNaN(vout)).toBe(false);
    expect(Number.isFinite(vout)).toBe(true);
  });
});

// ─── Per-unit independence ────────────────────────────────────────────────────

describe("LM358 — per-unit independence (unit 1 and unit 2 simultaneously)", () => {
  /**
   * Both units operate as voltage followers simultaneously.
   * Unit 1: in1+ = 1.0 V → out1 ≈ 1.0 V
   * Unit 2: in2+ = 3.0 V → out2 ≈ 3.0 V
   *
   * Both within LM358 vHigh = 3.5 V on 5 V supply.
   * Each unit contributes one independent VCVS branch row (op0, op1).
   *
   * Closed-loop derivation (A=1000, same as follower test):
   *   Unit 1: Vout1 = 1000 * 1.0 / 1001 ≈ 0.999 V ≈ 1.0 V (< 0.1 % error)
   *   Unit 2: Vout2 = 1000 * 3.0 / 1001 ≈ 2.997 V ≈ 3.0 V (< 0.1 % error)
   */
  it("unit 1 ≈ 1.0 V and unit 2 ≈ 3.0 V in the same part simultaneously", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",   kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vin1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.0 } },
        { id: "vin2", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3.0 } },
        { id: "oa1",  kind: "lm358", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        // Power
        { from_component: "vs",   from_pin: "pos", to_component: "oa1",  to_pin: "8" },
        { from_component: "vs",   from_pin: "neg", to_component: "oa1",  to_pin: "4" },
        // Unit 1 follower
        { from_component: "vin1", from_pin: "pos", to_component: "oa1",  to_pin: "3" },
        { from_component: "vin1", from_pin: "neg", to_component: "vs",   to_pin: "neg" },
        { from_component: "oa1",  from_pin: "1",   to_component: "oa1",  to_pin: "2" },
        // Unit 2 follower
        { from_component: "vin2", from_pin: "pos", to_component: "oa1",  to_pin: "5" },
        { from_component: "vin2", from_pin: "neg", to_component: "vs",   to_pin: "neg" },
        { from_component: "oa1",  from_pin: "7",   to_component: "oa1",  to_pin: "6" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout1 = netVoltage(eng, "oa1", "1");
    const vout2 = netVoltage(eng, "oa1", "7");
    // Unit 1: Vout1 ≈ 1.0 V
    expect(vout1).toBeGreaterThan(0.990);
    expect(vout1).toBeLessThan(1.010);
    // Unit 2: Vout2 ≈ 3.0 V
    expect(vout2).toBeGreaterThan(2.990);
    expect(vout2).toBeLessThan(3.010);
  });
});

// ─── High-gain accuracy + saturation recovery (orchestrator review) ───────────

describe("LM358 — high closed-loop gain is accurate (A=1e5)", () => {
  /**
   * Non-inverting ×100 amp: Rf=99k (out→in-), Rg=1k (in-→gnd), Vin=0.03 V at in+,
   * 9 V supply. Ideal Vout = 0.03 * (1 + 99k/1k) = 0.03 * 100 = 3.0 V.
   *
   * Closed-loop gain with finite open-loop A: Acl = A/(1 + A*β), β = 1k/100k = 0.01.
   * At A=1e5: Acl = 1e5/(1+1000) = 99.9, Vout = 2.997 V (0.1 % error).
   * At A=1000 the same circuit would read 2.727 V (9 % error) — this test
   * pins the open-loop gain high enough that high-gain configs stay honest.
   */
  it("×100 amp reads ~3.0 V, not the ~2.73 V a low open-loop gain would give", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 9 } },
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0.03 } },
        { id: "rf",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 99000 } },
        { id: "rg",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "oa",  kind: "lm358", pins: ["1","2","3","4","5","6","7","8"].map((id) => ({ id })), params: {} },
      ],
      wires: [
        { from_component: "v1",  from_pin: "pos", to_component: "oa", to_pin: "8" },
        { from_component: "v1",  from_pin: "neg", to_component: "oa", to_pin: "4" },
        { from_component: "vin", from_pin: "pos", to_component: "oa", to_pin: "3" },
        { from_component: "vin", from_pin: "neg", to_component: "v1", to_pin: "neg" },
        { from_component: "oa",  from_pin: "1",   to_component: "rf", to_pin: "a" },
        { from_component: "rf",  from_pin: "b",   to_component: "oa", to_pin: "2" },
        { from_component: "oa",  from_pin: "2",   to_component: "rg", to_pin: "a" },
        { from_component: "rg",  from_pin: "b",   to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "oa", "1");
    expect(vout).toBeGreaterThan(2.94);   // would be 2.73 at A=1000
    expect(vout).toBeLessThan(3.02);
  });
});

describe("LM358 — recovers from output saturation (no latch)", () => {
  /**
   * Drives a ×3 non-inverting amp into the high rail, then lowers Vin so the
   * ideal output returns in-range, WITHOUT reloading the circuit. A committed-
   * output regime that can only stay saturated would latch at the rail forever;
   * the relax-to-linear state machine must recover.
   *
   * ×3 amp on 5 V supply (LM358 vHigh = 3.5 V):
   *   Vin=2.0 V → ideal 6.0 V → clamps to 3.5 V (saturated).
   *   Vin=1.0 V → ideal 3.0 V → in range → must settle to ~3.0 V.
   */
  it("clamps at 3.5 V then returns to ~3.0 V after the input is lowered", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 2.0 } },
        { id: "rf",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 20000 } },
        { id: "rg",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "oa",  kind: "lm358", pins: ["1","2","3","4","5","6","7","8"].map((id) => ({ id })), params: {} },
      ],
      wires: [
        { from_component: "v1",  from_pin: "pos", to_component: "oa", to_pin: "8" },
        { from_component: "v1",  from_pin: "neg", to_component: "oa", to_pin: "4" },
        { from_component: "vin", from_pin: "pos", to_component: "oa", to_pin: "3" },
        { from_component: "vin", from_pin: "neg", to_component: "v1", to_pin: "neg" },
        { from_component: "oa",  from_pin: "1",   to_component: "rf", to_pin: "a" },
        { from_component: "rf",  from_pin: "b",   to_component: "oa", to_pin: "2" },
        { from_component: "oa",  from_pin: "2",   to_component: "rg", to_pin: "a" },
        { from_component: "rg",  from_pin: "b",   to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = new SimEngine();
    eng.load(circuit);
    for (let i = 0; i < 50; i++) eng.step(1e-4);
    // Saturated at the high rail.
    expect(netVoltage(eng, "oa", "1")).toBeGreaterThan(3.45);
    expect(netVoltage(eng, "oa", "1")).toBeLessThan(3.55);

    // Lower the drive in place (no reload) and keep stepping the SAME engine.
    circuit.components.find((c) => c.id === "vin")!.params.voltage = 1.0;
    eng.load(circuit); // load() preserves element state by id (does NOT reset regime)
    for (let i = 0; i < 50; i++) eng.step(1e-4);
    const vout = netVoltage(eng, "oa", "1");
    expect(vout).toBeGreaterThan(2.95);   // recovered to linear ~3.0 V, not latched at 3.5
    expect(vout).toBeLessThan(3.05);
  });
});
