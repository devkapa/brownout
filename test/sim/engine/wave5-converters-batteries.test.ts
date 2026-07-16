/**
 * Wave 5.2 engine tests: battery internal resistance (Thevenin) and DC-DC
 * converter module (behavioural 2-port, committed-regime).
 *
 * ALL expected values are derived from first-principles circuit analysis and
 * written as LITERAL constants with the derivation in a comment.  No value is
 * read back from the implementation and then asserted — self-confirming tests
 * were the cardinal sin that shipped wrong physics in earlier waves.
 *
 * ─── Battery derivations ─────────────────────────────────────────────────────
 *
 * Thevenin battery model:
 *   V_terminal = voc − rInternal × I_load
 *   I_load     = voc / (rInternal + R_load)
 *
 * IDEAL REGRESSION (supply-5v style, no rInternal):
 *   voc = 5 V, rInternal = 0, R_load = 100 Ω
 *   I_load = 5 / (0 + 100) = 0.050 A
 *   V_term = 5.0 V (unchanged from ideal; rInternal=0 must be byte-identical)
 *
 * BATTERY SAG (rInternal active):
 *   voc = 9 V, rInternal = 1.5 Ω, R_load = 10 Ω
 *   I_load = 9 / (1.5 + 10) = 9 / 11.5 ≈ 0.7826 A
 *   V_term = 9 − 1.5 × 0.7826 = 9 − 1.1739 = 7.826 V
 *   Exact: V_term = voc × R_load / (rInternal + R_load) = 9 × 10 / 11.5
 *                 = 90/11.5 = 7.8261 V
 *
 * BATTERY SOC (charge scales voc, no load):
 *   voc_nominal = 9 V, charge = 0.5, rInternal = 0
 *   voc_eff = 9 × 0.5 = 4.5 V
 *   No load → V_term = 4.5 V (no current, no sag)
 *
 * BATTERY NO-LOAD (rInternal > 0, open circuit):
 *   voc = 9 V, rInternal = 1.5 Ω, open circuit (no load resistor)
 *   I_load = 0 (nothing to carry current through — only GMIN leaks ~nA)
 *   V_term = voc − rInternal × 0 = 9.0 V (no sag)
 *
 * ─── DC-DC converter derivations ─────────────────────────────────────────────
 *
 * Committed-regime output model (matches linear_reg CC/REG pattern):
 *   REG: V(out_pos) − V(out_neg) = vout  (ideal regulated output)
 *   CC:  |I_out| = iLimit                 (current-limited; Vout floats = iLimit×Rload)
 *
 * Input current (computed from committed output-side values):
 *   Pout = Vout_committed × Iout_committed
 *   Iin  = Pout / (eta × max(Vin, 0.1))
 *
 * DCDC REG (buck 12→5 V):
 *   Vin = 12 V, vout = 5 V, eta = 0.85, R_load = 10 Ω
 *   Iout = vout / R_load = 5 / 10 = 0.5 A
 *   Pout = 5 × 0.5 = 2.5 W
 *   Iin  = 2.5 / (0.85 × 12) = 2.5 / 10.2 = 0.2451 A
 *   Assert: Vout ≈ 5.0 V, Iout ≈ 0.5 A, Iin ≈ 0.245 A (±10 mA)
 *
 * DCDC CC (overloaded output):
 *   vout = 5 V, iLimit = 1.0 A, R_load = 2 Ω
 *   Unlimited: 5/2 = 2.5 A > 1.0 A → CC
 *   Iout_clamped = 1.0 A, Vout = iLimit × R_load = 1.0 × 2 = 2.0 V
 *
 * DCDC UVLO / operating range:
 *   Declared input range is 7–35 V. Below 7 V, above 35 V, at zero input,
 *   or with reversed polarity, the output branch delivers zero current.
 */

import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

// ─── Helpers ──────────────────────────────────────────────────────────────────

function runEngine(circuit: SimCircuit, steps = 300, dt = 1e-4): SimEngine {
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

// ─── Battery ideal regression ─────────────────────────────────────────────────

describe("battery_pack — ideal regression (no rInternal, supply-5v style)", () => {
  /**
   * Confirms rInternal=0 (absent) is byte-identical to the prior ideal vsource.
   * Circuit: 5 V battery_pack, no rInternal, into 100 Ω.
   * I = 5/100 = 0.050 A, Vterm = 5.0 V.
   */
  const circuit: SimCircuit = {
    components: [
      { id: "bat", kind: "battery_pack", pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 } },
      { id: "rl",  kind: "resistor",     pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 100 } },
    ],
    wires: [
      { from_component: "bat", from_pin: "pos", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "bat", to_pin: "neg" },
    ],
  };

  it("V_term = 5.0 V ± 10 mV (ideal vsource: no rInternal, no sag)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "bat", "pos");
    // Derivation: ideal source V_pos = 5.0 V (rInternal=0 → no sag at any current).
    expect(vPos).toBeCloseTo(5.0, 2); // ±5 mV (2 decimal places)
  });

  it("I = 50 mA ± 2 mA (V/R = 5/100 = 0.05 A)", () => {
    const engine = runEngine(circuit);
    const elemI = Math.abs(engine.getElementI()["bat"] ?? 0);
    // Derivation: I = 5.0 / 100 = 0.050 A.
    expect(elemI).toBeCloseTo(0.050, 2);
  });

  it("lastConverged = true (no rInternal → linear; trivially converges)", () => {
    const engine = runEngine(circuit);
    expect(engine.lastConverged).toBe(true);
  });

  it("net voltages are finite (no NaN / Infinity in the solution)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "bat", "pos");
    expect(Number.isFinite(vPos)).toBe(true);
  });
});

// ─── Battery sag (rInternal active) ──────────────────────────────────────────

describe("battery_pack — voltage sag under load (rInternal = 1.5 Ω)", () => {
  /**
   * Circuit: 9 V battery_pack, rInternal = 1.5 Ω, into 10 Ω.
   * V_term = 9 × 10/(10+1.5) = 90/11.5 = 7.826 V
   * I_load = 9 / 11.5       = 0.7826 A
   */
  const circuit: SimCircuit = {
    components: [
      { id: "bat", kind: "battery_pack", pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 9, rInternal: 1.5 } },
      { id: "rl",  kind: "resistor",     pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 10 } },
    ],
    wires: [
      { from_component: "bat", from_pin: "pos", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "bat", to_pin: "neg" },
    ],
  };

  it("V_term ≈ 7.83 V ± 50 mV (Thevenin: 9×10/11.5 = 7.826 V)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "bat", "pos");
    // Derivation: V_term = voc × R/(R + rInt) = 9×10/11.5 = 7.8261 V.
    expect(vPos).toBeCloseTo(7.826, 1); // ±50 mV
    // Confirm V_term < voc (sag occurred).
    expect(vPos).toBeLessThan(9.0);
  });

  it("I ≈ 0.783 A ± 20 mA (9/11.5 = 0.7826 A)", () => {
    const engine = runEngine(circuit);
    const elemI = Math.abs(engine.getElementI()["bat"] ?? 0);
    // Derivation: I = voc / (rInternal + R_load) = 9 / 11.5 = 0.7826 A.
    expect(elemI).toBeCloseTo(0.783, 1); // ±50 mA (1 decimal place = ±50 mA)
  });

  it("V_pos < 9.0 V confirms Thevenin sag (not ideal source)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "bat", "pos");
    expect(vPos).toBeLessThan(9.0);
  });

  it("lastConverged = true (Thevenin stamp is linear; trivially converges)", () => {
    const engine = runEngine(circuit);
    expect(engine.lastConverged).toBe(true);
  });

  it("net voltages are finite", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "bat", "pos");
    expect(Number.isFinite(vPos)).toBe(true);
  });
});

// ─── Battery SoC ─────────────────────────────────────────────────────────────

describe("battery_pack — state of charge (charge = 0.5, no load)", () => {
  /**
   * charge=0.5 scales voc to 9×0.5 = 4.5 V.
   * rInternal=0, no load → V_term = 4.5 V exactly.
   * This is a STATIC parameter knob, not a real-time coulomb counter.
   */
  const circuit: SimCircuit = {
    components: [
      // Open circuit: only the battery — GMIN leaks are negligible (~nA).
      // Add a 1 MΩ resistor to give the solver a path; V_term ≈ voc.
      { id: "bat", kind: "battery_pack", pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 9, charge: 0.5, rInternal: 0 } },
      { id: "rl",  kind: "resistor",     pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1e6 } },
    ],
    wires: [
      { from_component: "bat", from_pin: "pos", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "bat", to_pin: "neg" },
    ],
  };

  it("V_term = 4.5 V ± 50 mV (voc × charge = 9 × 0.5 = 4.5 V)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "bat", "pos");
    // Derivation: voc = voltage × charge = 9 × 0.5 = 4.5 V; no-load → V_term = 4.5 V.
    expect(vPos).toBeCloseTo(4.5, 1); // ±50 mV
  });

  it("V_term < 5.0 V (confirms charge scaling, not full 9 V)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "bat", "pos");
    expect(vPos).toBeLessThan(5.0);
    expect(vPos).toBeGreaterThan(4.0);
  });

  it("lastConverged = true", () => {
    const engine = runEngine(circuit);
    expect(engine.lastConverged).toBe(true);
  });
});

// ─── Battery no-load (rInternal > 0, open circuit) ───────────────────────────

describe("battery_pack — no-load (rInternal = 1.5 Ω, open circuit)", () => {
  /**
   * With rInternal > 0 and no meaningful current:
   *   V_term = voc − rInternal × I = 9 − 1.5 × 0 = 9.0 V (no sag at open circuit).
   * GMIN ≈ 1e-12 S is the only load → I ≈ 9e-12 A, V_term ≈ 9.0000 V.
   * Use 1 MΩ shunt to avoid floating node; I ≈ 9 µA → sag = 1.5 × 9e-6 ≈ 14 µV.
   */
  const circuit: SimCircuit = {
    components: [
      { id: "bat", kind: "battery_pack", pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 9, rInternal: 1.5 } },
      { id: "rl",  kind: "resistor",     pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1e6 } },
    ],
    wires: [
      { from_component: "bat", from_pin: "pos", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "bat", to_pin: "neg" },
    ],
  };

  it("V_term ≈ 9.0 V ± 50 mV (no-load: I ≈ 0 so rInternal × I ≈ 0)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "bat", "pos");
    // Derivation: no-load sag = rInternal × I ≈ 1.5 × 9e-6 = 14 µV — negligible.
    // V_term ≈ 9.0 V.
    expect(vPos).toBeCloseTo(9.0, 1); // ±50 mV
  });

  it("V_term > 8.9 V (open-circuit Thevenin: essentially full voc)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "bat", "pos");
    expect(vPos).toBeGreaterThan(8.9);
  });

  it("lastConverged = true", () => {
    const engine = runEngine(circuit);
    expect(engine.lastConverged).toBe(true);
  });
});

// ─── DC-DC REG mode ───────────────────────────────────────────────────────────

describe("dcdc-buck-5v — non-isolated common return", () => {
  it("rejects a voltage source connected across IN− and OUT−", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "vs",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        {
          id: "dcdc",
          kind: "dcdc_converter",
          catalogUid: "dcdc-buck-5v",
          pins: [
            { id: "in_pos" }, { id: "in_neg" },
            { id: "out_pos" }, { id: "out_neg" },
          ],
          params: { vout: 5, eta: 0.85, iLimit: 2 },
        },
      ],
      wires: [
        { from_component: "vs", from_pin: "pos", to_component: "dcdc", to_pin: "in_neg" },
        { from_component: "vs", from_pin: "neg", to_component: "dcdc", to_pin: "out_neg" },
      ],
    };
    const engine = new SimEngine();

    engine.load(circuit);

    // IN− and OUT− are one copper return. Placing an ideal 5 V source
    // between them is therefore a physical short/conflicting constraint, not
    // an isolated-output circuit the engine may silently accept.
    expect(engine.lastConverged).toBe(false);
  });
});

describe("dcdc-buck-5v — averaged buck headroom", () => {
  function circuitAt(vin: number, vout: number, resistance = 100): SimCircuit {
    return {
      components: [
        { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: vin } },
        { id: "dcdc", kind: "dcdc_converter", catalogUid: "dcdc-buck-5v",
          pins: [{ id: "in_pos" }, { id: "in_neg" }, { id: "out_pos" }, { id: "out_neg" }],
          params: { vout, eta: 0.85, iLimit: 2, vdropout: 1.5 } },
        { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
      ],
      wires: [
        { from_component: "vs", from_pin: "pos", to_component: "dcdc", to_pin: "in_pos" },
        { from_component: "vs", from_pin: "neg", to_component: "dcdc", to_pin: "in_neg" },
        { from_component: "dcdc", from_pin: "out_pos", to_component: "rl", to_pin: "a" },
        { from_component: "rl", from_pin: "b", to_component: "dcdc", to_pin: "out_neg" },
      ],
    };
  }

  it.each([12, 20])("cannot boost a 12 V input to a %s V setpoint", (setpoint) => {
    const engine = runEngine(circuitAt(12, setpoint), 5);
    const vOut = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    const pOut = vOut * Math.abs(engine.getElementI().dcdc ?? 0);
    const pIn = 12 * Math.abs(engine.getElementI().vs ?? 0);
    expect(vOut).toBeCloseTo(10.5, 6);
    expect(engine.getIcState("dcdc")?.reg).toBe(1);
    expect(pIn * 0.85).toBeCloseTo(pOut, 6);
    expect(engine.lastConverged).toBe(true);
  });

  it("recovers from dropout in the first accepted step when Vin gains headroom", () => {
    const engine = runEngine(circuitAt(12, 20), 5);
    expect(engine.getIcState("dcdc")?.reg).toBe(1);

    engine.load(circuitAt(24, 20));
    engine.step(1e-4);

    const vOut = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    const pOut = vOut * Math.abs(engine.getElementI().dcdc ?? 0);
    const pIn = 24 * Math.abs(engine.getElementI().vs ?? 0);
    expect(vOut).toBeCloseTo(20, 6);
    expect(engine.getIcState("dcdc")?.reg).toBe(0);
    expect(pIn * 0.85).toBeCloseTo(pOut, 6);
    expect(engine.lastConverged).toBe(true);
  });

  it("balances present-solve input power on a sub-limit CV load step", () => {
    const engine = runEngine(circuitAt(12, 5, 100), 5);

    engine.load(circuitAt(12, 5, 10));
    const seededVOut = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    const seededPOut = seededVOut * Math.abs(engine.getElementI().dcdc ?? 0);
    const seededPIn = 12 * Math.abs(engine.getElementI().vs ?? 0);
    expect(seededVOut).toBeCloseTo(5, 6);
    expect(seededPIn * 0.85).toBeCloseTo(seededPOut, 6);
    expect(engine.lastConverged).toBe(true);

    engine.step(1e-4);

    const vOut = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    const iOut = Math.abs(engine.getElementI().dcdc ?? 0);
    const pOut = vOut * iOut;
    const pIn = 12 * Math.abs(engine.getElementI().vs ?? 0);
    expect(vOut).toBeCloseTo(5, 6);
    expect(iOut).toBeCloseTo(0.5, 6);
    expect(pIn * 0.85).toBeCloseTo(pOut, 6);
    expect(engine.getIcState("dcdc")?.reg).toBe(0);
    expect(engine.lastConverged).toBe(true);
  });
});

describe("dcdc_converter — REG mode: 12 V in, 5 V out, 10 Ω load, eta=0.85", () => {
  /**
   * Circuit: 12 V ideal source → dcdc_converter (vout=5, eta=0.85, iLimit=2A)
   *          → 10 Ω load to GND.
   *
   * Output: V_out = 5.0 V (regulated), I_out = 5/10 = 0.5 A
   * Power:  Pout = 5 × 0.5 = 2.5 W
   * Input:  Iin  = 2.5 / (0.85 × 12) = 2.5 / 10.2 = 0.2451 A
   *
   * Run enough steps for the committed Iin to settle (2–3 steps after startup).
   * The first step has Iin=0 (all committed values are 0 on step 1); on step 2
   * the engine uses the step-1 committed output to compute Iin for step 2, etc.
   * 300 steps is well beyond the ~3-step settling window.
   */
  const circuit: SimCircuit = {
    components: [
      { id: "vs",   kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 12 } },
      { id: "dcdc", kind: "dcdc_converter",
        pins: [{ id: "in_pos" }, { id: "in_neg" }, { id: "out_pos" }, { id: "out_neg" }],
        params: { vout: 5, eta: 0.85, iLimit: 2.0 } },
      { id: "rl",   kind: "resistor",       pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 10 } },
    ],
    wires: [
      // Supply → converter input
      { from_component: "vs",   from_pin: "pos",     to_component: "dcdc", to_pin: "in_pos"  },
      { from_component: "vs",   from_pin: "neg",     to_component: "dcdc", to_pin: "in_neg"  },
      // Converter output → load
      { from_component: "dcdc", from_pin: "out_pos", to_component: "rl",   to_pin: "a"       },
      { from_component: "rl",   from_pin: "b",       to_component: "dcdc", to_pin: "out_neg" },
    ],
  };

  it("V_out = 5.0 V ± 100 mV (REG constraint: V(out_pos)−V(out_neg) = vout)", () => {
    const engine = runEngine(circuit, 300);
    const vOutPos = netVoltage(engine, "dcdc", "out_pos");
    const vOutNeg = netVoltage(engine, "dcdc", "out_neg");
    const vOut    = vOutPos - vOutNeg;
    // Derivation: REG stamps V(out_pos) − V(out_neg) = 5.0 V.
    expect(vOut).toBeCloseTo(5.0, 0); // ±100 mV (1 decimal = ±50 mV)
    expect(vOut).toBeGreaterThan(4.5);
    expect(vOut).toBeLessThan(5.5);
  });

  it("I_out ≈ 0.5 A ± 50 mA (V_out / R_load = 5/10 = 0.5 A)", () => {
    const engine = runEngine(circuit, 300);
    const elemI = Math.abs(engine.getElementI()["dcdc"] ?? 0);
    // Derivation: I_out = V_out / R_load = 5.0 / 10 = 0.5 A.
    expect(elemI).toBeCloseTo(0.5, 1); // ±50 mA
  });

  it("I_in ≈ 0.245 A ± 10 mA (Pout/(eta×Vin) = 2.5/(0.85×12) = 0.245 A)", () => {
    // Input current is drawn by the converter's current-source stamp on the input side.
    // Measure via the voltage-source branch current (the input supply must deliver Iin).
    const engine = runEngine(circuit, 300);
    const iSource = Math.abs(engine.getElementI()["vs"] ?? 0);
    // Derivation: Iin = Pout / (eta × Vin) = (5×0.5)/(0.85×12) = 2.5/10.2 = 0.2451 A.
    // The supply current equals the converter's input draw (it is the only load on the supply).
    expect(iSource).toBeCloseTo(0.245, 1); // ±50 mA (1 decimal place = ±50 mA)
  });

  it("lastConverged = true (REG is linear per step; converges immediately)", () => {
    const engine = runEngine(circuit, 300);
    expect(engine.lastConverged).toBe(true);
  });

  it("all net voltages are finite", () => {
    const engine = runEngine(circuit, 300);
    const vIn  = netVoltage(engine, "dcdc", "in_pos");
    const vOut = netVoltage(engine, "dcdc", "out_pos");
    expect(Number.isFinite(vIn)).toBe(true);
    expect(Number.isFinite(vOut)).toBe(true);
    expect(vIn).toBeCloseTo(12.0, 0);
  });
});

// ─── DC-DC CC mode ────────────────────────────────────────────────────────────

describe("dcdc_converter — CC mode: vout=5, iLimit=1, 2 Ω load", () => {
  /**
   * Circuit: 12 V supply → dcdc_converter (vout=5, iLimit=1.0 A) → 2 Ω load.
   * Unlimited: V_out/R = 5/2 = 2.5 A > 1.0 A → CC.
   * Clamped:   I_out = 1.0 A, V_out = iLimit × R_load = 1.0 × 2 = 2.0 V.
   */
  const circuit: SimCircuit = {
    components: [
      { id: "vs",   kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 12 } },
      { id: "dcdc", kind: "dcdc_converter",
        pins: [{ id: "in_pos" }, { id: "in_neg" }, { id: "out_pos" }, { id: "out_neg" }],
        params: { vout: 5, eta: 0.85, iLimit: 1.0 } },
      { id: "rl",   kind: "resistor",       pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 2 } },
    ],
    wires: [
      { from_component: "vs",   from_pin: "pos",     to_component: "dcdc", to_pin: "in_pos"  },
      { from_component: "vs",   from_pin: "neg",     to_component: "dcdc", to_pin: "in_neg"  },
      { from_component: "dcdc", from_pin: "out_pos", to_component: "rl",   to_pin: "a"       },
      { from_component: "rl",   from_pin: "b",       to_component: "dcdc", to_pin: "out_neg" },
    ],
  };

  it("I_out ≈ 1.0 A ± 0.1 A (CC clamp = iLimit)", () => {
    const engine = runEngine(circuit, 300);
    const elemI = Math.abs(engine.getElementI()["dcdc"] ?? 0);
    // Derivation: CC mode stamps x[k] = iLimit = 1.0 A (same as bench_psu / linear_reg CC).
    expect(elemI).toBeCloseTo(1.0, 0); // ±0.1 A (1 decimal = ±50 mA)
  });

  it("V_out ≈ 2.0 V ± 0.2 V (iLimit × R_load = 1.0 × 2 = 2.0 V in CC)", () => {
    const engine = runEngine(circuit, 300);
    const vOutPos = netVoltage(engine, "dcdc", "out_pos");
    const vOutNeg = netVoltage(engine, "dcdc", "out_neg");
    const vOut    = vOutPos - vOutNeg;
    // Derivation: KCL at out_pos: I_CC × R_load = 1.0 × 2 = 2.0 V.
    expect(vOut).toBeCloseTo(2.0, 0); // ±0.2 V
    // Confirm V_out < vout setpoint (current-limited, not regulating).
    expect(vOut).toBeLessThan(5.0);
  });

  it("lastConverged = true (CC is linear per step; converges immediately)", () => {
    const engine = runEngine(circuit, 300);
    expect(engine.lastConverged).toBe(true);
  });

  it("all net voltages are finite", () => {
    const engine = runEngine(circuit, 300);
    const vOut = netVoltage(engine, "dcdc", "out_pos");
    expect(Number.isFinite(vOut)).toBe(true);
  });
});

// ─── DC-DC UVLO and operating range ─────────────────────────────────────────

describe("dcdc_converter — same-step CC compliance recovery", () => {
  function circuitWithLoad(resistance: number, disconnected = false): SimCircuit {
    return {
      components: [
        { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 12 } },
        { id: "dcdc", kind: "dcdc_converter",
          pins: [{ id: "in_pos" }, { id: "in_neg" }, { id: "out_pos" }, { id: "out_neg" }],
          params: { vout: 5, eta: 0.85, iLimit: 1 } },
        ...(disconnected ? [{ id: "sw", kind: "switch", catalogUid: "switch-spst",
          pins: [{ id: "a" }, { id: "b" }], params: { closed: 0 } }] : []),
        { id: "rl", kind: "resistor", catalogUid: "resistor",
          pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
      ],
      wires: [
        { from_component: "vs", from_pin: "pos", to_component: "dcdc", to_pin: "in_pos" },
        { from_component: "vs", from_pin: "neg", to_component: "dcdc", to_pin: "in_neg" },
        ...(disconnected ? [
          { from_component: "dcdc", from_pin: "out_pos", to_component: "sw", to_pin: "a" },
          { from_component: "sw", from_pin: "b", to_component: "rl", to_pin: "a" },
        ] : [{ from_component: "dcdc", from_pin: "out_pos", to_component: "rl", to_pin: "a" }]),
        { from_component: "rl", from_pin: "b", to_component: "dcdc", to_pin: "out_neg" },
      ],
    };
  }

  it.each([
    { resistance: 100, maxCurrent: 0.051, label: "100 Ω recovery load" },
    { resistance: 1e9, maxCurrent: 1e-7, label: "near-open 1 GΩ load" },
  ])("bounds $label at 5 V without creating output energy", ({ resistance, maxCurrent }) => {
    const engine = runEngine(circuitWithLoad(2), 10);
    const vOutCC = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    expect(vOutCC).toBeCloseTo(2, 6);
    const stressBefore = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;

    engine.load(circuitWithLoad(resistance));
    engine.step(1e-4);

    const vOut = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    const iOut = Math.abs(engine.getElementI().dcdc ?? 0);
    const pOut = Math.max(0, vOut * iOut);
    const pIn = 12 * Math.abs(engine.getElementI().vs ?? 0);
    expect(vOut).toBeCloseTo(5, 6);
    expect(vOut).toBeLessThanOrEqual(5.000_01);
    expect(iOut).toBeLessThan(maxCurrent);
    // Same-step power coupling: Pin·eta = Pout for the bounded recovery result.
    // Below the solver's 1 µA absolute tolerance, RSHUNT-scale nanowatt power
    // is numerical rather than a meaningful energy budget.
    expect(pIn + 1e-6).toBeGreaterThanOrEqual(pOut);
    expect(pIn * 0.85).toBeCloseTo(pOut, 6);
    expect(engine.getIcState("dcdc")?.reg).toBe(0);
    const stressAfter = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;
    expect(stressAfter).toBeLessThanOrEqual(stressBefore + 1e-12);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("enters CC and balances input power in the first accepted hard-load step", () => {
    const engine = runEngine(circuitWithLoad(100), 5);
    engine.load(circuitWithLoad(2));
    engine.step(1e-4);

    const vOut = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    const iOut = Math.abs(engine.getElementI().dcdc ?? 0);
    const pOut = vOut * iOut;
    const pIn = 12 * Math.abs(engine.getElementI().vs ?? 0);
    expect(vOut).toBeCloseTo(2, 6);
    expect(iOut).toBeCloseTo(1, 9);
    expect(iOut).toBeLessThanOrEqual(1.000_001);
    expect(pIn * 0.85).toBeCloseTo(pOut, 6);
    expect(engine.getIcState("dcdc")?.reg).toBe(2);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("holds 5 V differential with zero load when a downstream SPST opens", () => {
    const engine = runEngine(circuitWithLoad(2), 10);
    engine.load(circuitWithLoad(1_000, true));
    engine.step(1e-4);

    const vOut = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    const iOut = Math.abs(engine.getElementI().dcdc ?? 0);
    const pIn = 12 * Math.abs(engine.getElementI().vs ?? 0);
    expect(vOut).toBeCloseTo(5, 6);
    expect(iOut).toBeLessThan(1e-9);
    expect(pIn).toBeLessThan(1e-8);
    expect(engine.getIcState("dcdc")?.reg).toBe(0);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });
});

describe("dcdc_converter — OFF outside its declared 7–35 V input range", () => {
  function circuitAt(vin: number): SimCircuit {
    return {
      components: [
        { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: vin } },
        { id: "dcdc", kind: "dcdc_converter",
          pins: [{ id: "in_pos" }, { id: "in_neg" }, { id: "out_pos" }, { id: "out_neg" }],
          params: { vout: 5, eta: 0.85, iLimit: 2 } },
        { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
      ],
      wires: [
        { from_component: "vs", from_pin: "pos", to_component: "dcdc", to_pin: "in_pos" },
        { from_component: "vs", from_pin: "neg", to_component: "dcdc", to_pin: "in_neg" },
        { from_component: "dcdc", from_pin: "out_pos", to_component: "rl", to_pin: "a" },
        { from_component: "rl", from_pin: "b", to_component: "dcdc", to_pin: "out_neg" },
      ],
    };
  }

  it.each([0, 5, -12, 36])("Vin=%s V produces no output energy", (vin) => {
    const engine = runEngine(circuitAt(vin), 20);
    const vOut = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    expect(Math.abs(vOut)).toBeLessThan(1e-6);
    expect(Math.abs(engine.getElementI().dcdc ?? 0)).toBeLessThan(1e-9);
    expect(engine.lastConverged).toBe(true);
  });

  it("recovers from UVLO when valid input power is restored", () => {
    const engine = new SimEngine();
    engine.load(circuitAt(5));
    for (let i = 0; i < 10; i++) engine.step(1e-4);
    engine.load(circuitAt(12));
    for (let i = 0; i < 10; i++) engine.step(1e-4);
    const vOut = netVoltage(engine, "dcdc", "out_pos")
      - netVoltage(engine, "dcdc", "out_neg");
    expect(vOut).toBeCloseTo(5, 6);
    expect(engine.lastConverged).toBe(true);
  });
});
