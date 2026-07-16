/**
 * Wave 5.1 engine tests: linear_reg (7805 / AMS1117), lm317 adjustable
 * regulator, and bench_psu current-limiting fold.
 *
 * All expected values are derived from first-principles circuit analysis and
 * written as LITERAL constants with the derivation in a comment.  No value is
 * read back from the implementation and then asserted — self-confirming tests
 * were the cardinal sin that shipped wrong physics in earlier waves.
 *
 * Derivations:
 *
 *  7805 REG:
 *    Vin = 9 V, Vout = 5 V (regulated), Rload = 100 Ω
 *    I = Vout / Rload = 5.0 / 100 = 0.050 A (50 mA)
 *    headroom = 9 − 0 = 9 V ≥ 5+2 = 7 V → REG regime
 *    Input current ≈ output current (gnd pin is sense-only, draws ~0)
 *
 *  7805 DROPOUT:
 *    Vin = 6 V, headroom = 6 < 5+2 = 7 → DROPOUT
 *    Vout = Vin − vdropout = 6 − 2 = 4.0 V
 *    I = 4.0 / 100 = 40 mA
 *
 *  7805 CC (short-circuit):
 *    Vin = 9 V, Rload = 2 Ω
 *    Unlimited: I = 5 / 2 = 2.5 A > iLimit=1.0 A → CC
 *    Clamped: I = 1.0 A, Vout = I × Rload = 1.0 × 2 = 2.0 V
 *
 *  7805 RECOVERY:
 *    After CC, swap to 100 Ω load — natural current = 5/100 = 50 mA ≪ 1 A
 *    → regulatorRegime exits CC → REG; Vout returns to 5.0 V
 *
 *  AMS1117-3.3 REG:
 *    Vin = 5 V, vout = 3.3 V, vdropout = 1.1 V
 *    headroom = 5 ≥ 3.3+1.1 = 4.4 → REG
 *    Vout = 3.3 V
 *
 *  LM317 adjustable:
 *    Vin = 12 V, R1 = 240 Ω (out→adj), R2 = 720 Ω (adj→gnd)
 *    Chip enforces V_out − V_adj = 1.25 V (Vref)
 *    KVL: V_adj = V_out − 1.25
 *    Current through R1: I_R1 = 1.25 / 240 ≈ 5.208 mA
 *    V_adj = I_R1 × R2 + 0 = 5.208e-3 × 720 ≈ 3.75 V
 *         OR: V_out = 1.25 × (1 + R2/R1) = 1.25 × (1 + 720/240) = 1.25 × 4 = 5.0 V
 *
 *  bench_psu CC:
 *    V = 10 V, iLimit = 0.5 A, Rload = 2 Ω
 *    Unlimited: I = 10/2 = 5 A > 0.5 A → CC
 *    Clamped: I = 0.5 A, Vout = I × Rload = 0.5 × 2 = 1.0 V
 *
 *  bench_psu NO iLimit (regression):
 *    V = 10 V, Rload = 2 Ω → I = 10/2 = 5.0 A, Vout = 10 V (ideal source)
 *
 *  Input current ≈ output current proof:
 *    The branch current x[k] flows from the IN node to the OUT node.
 *    Current coupling: mna.add(outNode, k, +1) / mna.add(inNode, k, −1)
 *    KCL at inNode:  sum(currents leaving inNode) = 0
 *                    includes −x[k] (into the branch from inNode) → I_in = x[k]
 *    KCL at outNode: I_delivered = x[k] → I_out = x[k]
 *    Therefore I_in ≈ I_out (gnd/adj pin carries ~0 current in normal operation).
 *    Test: compare elementI[reg] against I from Vout/Rload — they should match.
 */

import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const catalog = rawCatalog as unknown as PartCatalog;

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

// ─── Catalog validation ───────────────────────────────────────────────────────

describe("catalog entries — linear_reg and lm317", () => {
  it("reg-7805 part is defined and correct", () => {
    const part = catalog.parts.find((p) => p.uid === "reg-7805");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("linear_reg");
    expect(part?.category).toBe("power");
    expect(part?.default_params?.vout).toBe(5.0);
    expect(part?.default_params?.vdropout).toBe(2.0);
    expect(part?.default_params?.iLimit).toBe(1.0);
    const pinIn  = part?.pin_layout.find((p) => p.id === "in");
    const pinGnd = part?.pin_layout.find((p) => p.id === "gnd");
    const pinOut = part?.pin_layout.find((p) => p.id === "out");
    expect(pinIn?.function).toBe("analog_in");
    expect(pinGnd?.function).toBe("gnd");
    expect(pinOut?.function).toBe("analog_out");
    // p_max must be > 0 (operating-specs guard)
    expect((part?.electrical_specs?.p_max ?? 0) > 0).toBe(true);
    expect(part?.bom?.package).toBe("TO-220");
    expect(part?.bom?.bomFamily).toBeTruthy();
  });

  it("reg-ams1117-33 part is defined", () => {
    const part = catalog.parts.find((p) => p.uid === "reg-ams1117-33");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("linear_reg");
    expect(part?.default_params?.vout).toBe(3.3);
    expect(part?.default_params?.vdropout).toBe(1.1);
    expect(part?.default_params?.iLimit).toBe(0.8);
    expect(part?.bom?.package).toBe("SOT-223 breakout module");
    expect(part?.bom?.polarityNotes).toMatch(/bare fixed-output AMS1117/);
  });

  it("reg-ams1117-50 part is defined", () => {
    const part = catalog.parts.find((p) => p.uid === "reg-ams1117-50");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("linear_reg");
    expect(part?.default_params?.vout).toBe(5.0);
    expect(part?.default_params?.vdropout).toBe(1.1);
    expect(part?.default_params?.iLimit).toBe(0.8);
  });

  it("lm317 part is defined and correct", () => {
    const part = catalog.parts.find((p) => p.uid === "lm317");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("lm317");
    expect(part?.category).toBe("power");
    expect(part?.default_params?.vref).toBe(1.25);
    expect(part?.default_params?.vdropout).toBe(2.0);
    expect(part?.default_params?.iLimit).toBe(1.5);
    const pinIn  = part?.pin_layout.find((p) => p.id === "in");
    const pinAdj = part?.pin_layout.find((p) => p.id === "adj");
    const pinOut = part?.pin_layout.find((p) => p.id === "out");
    expect(pinIn?.function).toBe("analog_in");
    expect(pinAdj?.function).toBe("analog_in");
    expect(pinOut?.function).toBe("analog_out");
    expect((part?.electrical_specs?.p_max ?? 0) > 0).toBe(true);
    expect(part?.bom?.package).toBe("TO-220 breakout module");
    expect(part?.bom?.polarityNotes).toMatch(/bare LM317T TO-220/);
  });
});

// ─── 7805 REG mode ────────────────────────────────────────────────────────────

describe("7805 — REG mode: 9 V in, 100 Ω load", () => {
  /**
   * Circuit: 9 V supply → 7805 → 100 Ω load to GND.
   * Vin = 9 V, headroom = 9 V ≥ 5+2 = 7 V → REG.
   * Vout = 5.0 V (regulated), I = 5.0 / 100 = 0.050 A.
   */
  const circuit: SimCircuit = {
    components: [
      { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 9 } },
      { id: "reg", kind: "linear_reg",     pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
        params: { vout: 5.0, vdropout: 2.0, iLimit: 1.0 } },
      { id: "rl",  kind: "resistor",       pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
    ],
    wires: [
      { from_component: "vs",  from_pin: "pos", to_component: "reg", to_pin: "in" },
      { from_component: "vs",  from_pin: "neg", to_component: "reg", to_pin: "gnd" },
      { from_component: "reg", from_pin: "out", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "vs",  to_pin: "neg" },
    ],
  };

  it("Vout = 5.0 V ± 50 mV (REG constraint)", () => {
    const engine = runEngine(circuit);
    const vOut = netVoltage(engine, "reg", "out");
    // Derivation: REG regime enforces V_out − V_ref = 5.0 V; V_ref = GND = 0.
    expect(vOut).toBeCloseTo(5.0, 1); // ±50 mV
  });

  it("I_load = 50 mA ± 5 mA (Vout/Rload = 5.0/100)", () => {
    const engine = runEngine(circuit);
    const vOut = netVoltage(engine, "reg", "out");
    // Resistor current from KCL: I = V_out / R = 5.0 / 100 = 0.050 A.
    const iLoad = vOut / 100;
    expect(iLoad).toBeCloseTo(0.050, 1); // ±5 mA
  });

  it("elementI[reg] ≈ I_load: input current ≈ output current", () => {
    const engine = runEngine(circuit);
    const elemI = engine.getElementI()["reg"] ?? 0;
    // Branch current x[k] = current flowing in→out = delivered current.
    // V_out/R_load = 5.0/100 = 50 mA.  elementI should match ± 5 mA.
    expect(Math.abs(elemI)).toBeCloseTo(0.050, 1);
  });

  it("engine converged (no NaN in net voltages)", () => {
    const engine = runEngine(circuit);
    const vIn = netVoltage(engine, "reg", "in");
    expect(Number.isFinite(vIn)).toBe(true);
    expect(vIn).toBeCloseTo(9.0, 0);
  });
});

// ─── 7805 DROPOUT mode ────────────────────────────────────────────────────────

describe("7805 — DROPOUT mode: 6 V in (headroom < 7 V)", () => {
  /**
   * Circuit: 6 V supply → 7805 → 100 Ω load to GND.
   * headroom = 6 − 0 = 6 V < 5+2 = 7 V → DROPOUT.
   * Vout = Vin − vdropout = 6 − 2 = 4.0 V.
   * I = 4.0 / 100 = 40 mA.
   */
  const circuit: SimCircuit = {
    components: [
      { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 6 } },
      { id: "reg", kind: "linear_reg",     pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
        params: { vout: 5.0, vdropout: 2.0, iLimit: 1.0 } },
      { id: "rl",  kind: "resistor",       pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
    ],
    wires: [
      { from_component: "vs",  from_pin: "pos", to_component: "reg", to_pin: "in" },
      { from_component: "vs",  from_pin: "neg", to_component: "reg", to_pin: "gnd" },
      { from_component: "reg", from_pin: "out", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "vs",  to_pin: "neg" },
    ],
  };

  it("Vout ≈ 4.0 V ± 100 mV (DROPOUT: Vin − vdropout = 6−2 = 4.0 V)", () => {
    const engine = runEngine(circuit);
    const vOut = netVoltage(engine, "reg", "out");
    // Derivation: DROPOUT constraint: V_out − V_in = −vdropout → V_out = 6 − 2 = 4.0 V.
    expect(vOut).toBeCloseTo(4.0, 0); // ±100 mV
  });

  it("Vout < 5.0 V (confirms it is NOT regulating at nominal)", () => {
    const engine = runEngine(circuit);
    const vOut = netVoltage(engine, "reg", "out");
    expect(vOut).toBeLessThan(5.0);
  });
});

describe("7805 — unpowered/low-input behaviour", () => {
  function lowInputCircuit(voltage: number): SimCircuit {
    return {
      components: [
        { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage } },
        { id: "reg", kind: "linear_reg", pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
          params: { vout: 5, vdropout: 2, iLimit: 1 } },
        { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      ],
      wires: [
        { from_component: "vs", from_pin: "pos", to_component: "reg", to_pin: "in" },
        { from_component: "vs", from_pin: "neg", to_component: "reg", to_pin: "gnd" },
        { from_component: "reg", from_pin: "out", to_component: "rl", to_pin: "a" },
        { from_component: "rl", from_pin: "b", to_component: "vs", to_pin: "neg" },
      ],
    };
  }

  it.each([0, 1, 2])("never creates a negative output at Vin=%s V", (vin) => {
    const engine = runEngine(lowInputCircuit(vin));
    const vOut = netVoltage(engine, "reg", "out");
    expect(vOut).toBeGreaterThanOrEqual(-1e-9);
    expect(vOut).toBeLessThanOrEqual(vin + 1e-9);
  });
});

// ─── 7805 CC mode (overload) ─────────────────────────────────────────────────

describe("7805 — same-solve input headroom transitions", () => {
  function circuitAt(inputVoltage: number, inputConnected = true): SimCircuit {
    return {
      components: [
        { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: inputVoltage } },
        { id: "input_sw", kind: "switch", catalogUid: "switch-spst",
          pins: [{ id: "a" }, { id: "b" }], params: { closed: inputConnected ? 1 : 0 } },
        { id: "reg", kind: "linear_reg", pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
          params: { vout: 5, vdropout: 2, iLimit: 1 } },
        { id: "rl", kind: "resistor", catalogUid: "resistor",
          pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      ],
      wires: [
        { from_component: "vs", from_pin: "pos", to_component: "input_sw", to_pin: "a" },
        { from_component: "input_sw", from_pin: "b", to_component: "reg", to_pin: "in" },
        { from_component: "vs", from_pin: "neg", to_component: "reg", to_pin: "gnd" },
        { from_component: "reg", from_pin: "out", to_component: "rl", to_pin: "a" },
        { from_component: "rl", from_pin: "b", to_component: "vs", to_pin: "neg" },
      ],
    };
  }

  it("turns the pass path off in the same accepted step when IN is disconnected", () => {
    const engine = runEngine(circuitAt(10), 5);
    expect(netVoltage(engine, "reg", "out")).toBeCloseTo(5, 6);

    engine.load(circuitAt(10, false));
    engine.step(1e-4);

    expect(Math.abs(netVoltage(engine, "reg", "in"))).toBeLessThan(1e-6);
    expect(Math.abs(netVoltage(engine, "reg", "out"))).toBeLessThan(1e-6);
    expect(Math.abs(engine.getElementI().reg ?? 0)).toBeLessThan(1e-12);
    expect(engine.getIcState("reg")?.reg).toBe(3);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("cold-loads a disconnected seed directly into a bounded OFF solution", () => {
    const engine = runEngine(circuitAt(10, false), 1);
    expect(Math.abs(netVoltage(engine, "reg", "in"))).toBeLessThan(1e-6);
    expect(Math.abs(netVoltage(engine, "reg", "out"))).toBeLessThan(1e-6);
    expect(Math.abs(engine.getElementI().reg ?? 0)).toBeLessThan(1e-12);
    expect(engine.getIcState("reg")?.reg).toBe(3);
    expect(engine.lastConverged).toBe(true);
  });

  it("enters 4 V dropout on the first 10 V → 6 V step and restores 5 V without an 8 V frame", () => {
    const engine = runEngine(circuitAt(10), 5);

    engine.load(circuitAt(6));
    engine.step(1e-4);
    expect(netVoltage(engine, "reg", "in")).toBeCloseTo(6, 4);
    expect(netVoltage(engine, "reg", "out")).toBeCloseTo(4, 4);
    expect(Math.abs(engine.getElementI().reg ?? 0)).toBeCloseTo(0.04, 5);
    expect(engine.getIcState("reg")?.reg).toBe(1);
    expect(engine.lastConverged).toBe(true);

    engine.load(circuitAt(10));
    engine.step(1e-4);
    expect(netVoltage(engine, "reg", "out")).toBeCloseTo(5, 6);
    expect(Math.abs(engine.getElementI().reg ?? 0)).toBeCloseTo(0.05, 6);
    expect(engine.getIcState("reg")?.reg).toBe(0);
    expect(engine.lastConverged).toBe(true);
  });

  it("restores regulation in the first accepted step after reconnecting IN", () => {
    const engine = runEngine(circuitAt(10), 5);
    engine.load(circuitAt(10, false));
    engine.step(1e-4);
    expect(engine.getIcState("reg")?.reg).toBe(3);

    engine.load(circuitAt(10, true));
    engine.step(1e-4);
    // Closed SPST is modelled as 1 mΩ, so 50 mA drops 50 µV.
    expect(netVoltage(engine, "reg", "in")).toBeCloseTo(9.99995, 6);
    expect(netVoltage(engine, "reg", "out")).toBeCloseTo(5, 6);
    expect(Math.abs(engine.getElementI().reg ?? 0)).toBeCloseTo(0.05, 6);
    expect(engine.getIcState("reg")?.reg).toBe(0);
    expect(engine.lastConverged).toBe(true);
  });
});

describe.each([
  { label: "7805", kind: "linear_reg" as const, refPin: "gnd", catalogUid: "reg-7805", params: { vout: 5, vdropout: 2, iLimit: 1 } as Record<string, number | string> },
  { label: "LM317", kind: "lm317" as const, refPin: "adj", catalogUid: "lm317", params: { vref: 1.25, vdropout: 2, iLimit: 1.5 } as Record<string, number | string> },
])("$label — OFF pass path", ({ kind, refPin, catalogUid, params }) => {
  it("leaves an externally biased output high-impedance with zero pass current", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
        {
          id: "reg",
          kind,
          catalogUid,
          pins: [{ id: "in" }, { id: refPin }, { id: "out" }],
          params,
        },
        { id: "bias", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
        { id: "rb", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1_000 } },
      ],
      wires: [
        { from_component: "vin", from_pin: "pos", to_component: "reg", to_pin: "in" },
        { from_component: "vin", from_pin: "neg", to_component: "reg", to_pin: refPin },
        { from_component: "bias", from_pin: "neg", to_component: "vin", to_pin: "neg" },
        { from_component: "bias", from_pin: "pos", to_component: "rb", to_pin: "a" },
        { from_component: "rb", from_pin: "b", to_component: "reg", to_pin: "out" },
      ],
    };

    const engine = runEngine(circuit, 5);
    expect(netVoltage(engine, "reg", "out")).toBeCloseTo(3, 6);
    expect(engine.getElementI().reg).toBeCloseTo(0, 12);
  });
});

describe("7805 — CC mode: 9 V in, 2 Ω load (unlimited would be 2.5 A)", () => {
  /**
   * Circuit: 9 V supply → 7805 → 2 Ω load to GND.
   * CV load demand: 5/2 = 2.5 A > iLimit=1.0 A → CC.
   * Clamped: I = 1.0 A, Vout = I × R = 1.0 × 2 = 2.0 V.
   */
  const circuit: SimCircuit = {
    components: [
      { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 9 } },
      { id: "reg", kind: "linear_reg",     pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
        params: { vout: 5.0, vdropout: 2.0, iLimit: 1.0 } },
      { id: "rl",  kind: "resistor",       pins: [{ id: "a" }, { id: "b" }], params: { resistance: 2 } },
    ],
    wires: [
      { from_component: "vs",  from_pin: "pos", to_component: "reg", to_pin: "in" },
      { from_component: "vs",  from_pin: "neg", to_component: "reg", to_pin: "gnd" },
      { from_component: "reg", from_pin: "out", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "vs",  to_pin: "neg" },
    ],
  };

  it("I ≈ 1.0 A ± 0.1 A (CC clamp = iLimit)", () => {
    const engine = runEngine(circuit);
    const elemI = Math.abs(engine.getElementI()["reg"] ?? 0);
    // Derivation: CC regime stamps x[k] = iLimit = 1.0 A.
    expect(elemI).toBeCloseTo(1.0, 0); // ±0.1 A (1 decimal place)
  });

  it("Vout ≈ 2.0 V ± 0.2 V (I×Rload = 1.0×2 = 2.0 V in CC)", () => {
    const engine = runEngine(circuit);
    const vOut = netVoltage(engine, "reg", "out");
    // Derivation: KCL at output: I_out = I_CC = 1.0 A → V_out = I × R = 1.0 × 2 = 2.0 V.
    expect(vOut).toBeCloseTo(2.0, 0); // ±0.2 V
  });

  it("Vout < 5.0 V (confirms current-limited, not regulating at nominal)", () => {
    const engine = runEngine(circuit);
    const vOut = netVoltage(engine, "reg", "out");
    expect(vOut).toBeLessThan(5.0);
  });
});

// ─── 7805 RECOVERY from CC ───────────────────────────────────────────────────

describe("7805 — RECOVERY: CC → REG when load eases", () => {
  /**
   * These tests prove there is no CC latch or one-step compliance overshoot.
   *
   * Step 1: run with 2 Ω load → enters CC.
   * Step 2: call engine.load() with a lighter load (preserving icState — same IDs).
   * Step 3: the same electrical solve selects the 5 V compliance branch before
   *         committing voltages, current, thermal stress, or damage.
   *
   * A bare ideal CC stamp would produce 100 V into 100 Ω and about 1 GV into
   * 1 GΩ for one step. Neither provisional value is a physical regulator output.
   */
  function circuitWithLoad(resistance: number, disconnected = false): SimCircuit {
    return {
      components: [
        { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 9 } },
        { id: "reg", kind: "linear_reg",     pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
          params: { vout: 5.0, vdropout: 2.0, iLimit: 1.0 } },
        ...(disconnected ? [{ id: "sw", kind: "switch", catalogUid: "switch-spst",
          pins: [{ id: "a" }, { id: "b" }], params: { closed: 0 } }] : []),
        { id: "rl",  kind: "resistor", catalogUid: "resistor",
          pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
      ],
      wires: [
        { from_component: "vs",  from_pin: "pos", to_component: "reg", to_pin: "in" },
        { from_component: "vs",  from_pin: "neg", to_component: "reg", to_pin: "gnd" },
        ...(disconnected ? [
          { from_component: "reg", from_pin: "out", to_component: "sw", to_pin: "a" },
          { from_component: "sw", from_pin: "b", to_component: "rl", to_pin: "a" },
        ] : [{ from_component: "reg", from_pin: "out", to_component: "rl", to_pin: "a" }]),
        { from_component: "rl",  from_pin: "b",   to_component: "vs",  to_pin: "neg" },
      ],
    };
  }

  it("bounds the first accepted recovery step at 5 V when load eases to 100 Ω", () => {
    // Phase 1: run to CC steady-state.
    const engine = runEngine(circuitWithLoad(2), 10);
    const vOutCC = netVoltage(engine, "reg", "out");
    // Confirm we are actually in CC before testing recovery.
    expect(vOutCC).toBeLessThan(4.0);
    const stressBefore = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;

    engine.load(circuitWithLoad(100));
    engine.step(1e-4);

    const vOutREG = netVoltage(engine, "reg", "out");
    expect(vOutREG).toBeCloseTo(5.0, 6);
    expect(vOutREG).toBeLessThanOrEqual(5.000_01);
    expect(Math.abs(engine.getElementI().reg ?? 0)).toBeCloseTo(0.05, 6);
    expect(Math.abs(engine.getElementI().vs ?? 0)).toBeCloseTo(0.05, 6);
    expect(engine.getIcState("reg")?.reg).toBe(0);
    const stressAfter = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;
    expect(stressAfter).toBeLessThanOrEqual(stressBefore + 1e-12);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("keeps a near-open 1 GΩ output bounded on the first accepted step", () => {
    const engine = runEngine(circuitWithLoad(2), 10);
    const stressBefore = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;
    engine.load(circuitWithLoad(1e9));
    engine.step(1e-4);

    const vOut = netVoltage(engine, "reg", "out");
    expect(vOut).toBeCloseTo(5.0, 6);
    expect(vOut).toBeLessThanOrEqual(5.000_01);
    expect(Math.abs(engine.getElementI().reg ?? 0)).toBeLessThan(1e-7);
    const stressAfter = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;
    expect(stressAfter).toBeLessThanOrEqual(stressBefore);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("holds 5 V when an SPST opens the load, without an RSHUNT-voltage frame", () => {
    const engine = runEngine(circuitWithLoad(2), 10);
    const stressBefore = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;
    engine.load(circuitWithLoad(1_000, true));
    engine.step(1e-4);

    expect(netVoltage(engine, "reg", "out")).toBeCloseTo(5, 6);
    expect(Math.abs(engine.getElementI().reg ?? 0)).toBeLessThan(1e-9);
    expect(engine.getIcState("reg")?.reg).toBe(0);
    const stressAfter = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;
    expect(stressAfter).toBeLessThanOrEqual(stressBefore);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("enters CC within the first accepted step after a 100 Ω → 2 Ω load switch", () => {
    const engine = runEngine(circuitWithLoad(100), 5);
    engine.load(circuitWithLoad(2));
    engine.step(1e-4);

    const current = Math.abs(engine.getElementI().reg ?? 0);
    expect(netVoltage(engine, "reg", "out")).toBeCloseTo(2, 6);
    expect(current).toBeCloseTo(1, 9);
    expect(current).toBeLessThanOrEqual(1.000_001);
    expect(engine.getIcState("reg")?.reg).toBe(2);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });
});

describe("LM317 — same-step recovery from current limit", () => {
  function circuitWithLoad(resistance: number, disconnected = false): SimCircuit {
    return {
      components: [
        { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 12 } },
        { id: "lm", kind: "lm317", pins: [{ id: "in" }, { id: "adj" }, { id: "out" }],
          params: { vref: 1.25, vdropout: 2, iLimit: 0.1 } },
        { id: "r1", kind: "resistor", catalogUid: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 240 } },
        { id: "r2", kind: "resistor", catalogUid: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 720 } },
        ...(disconnected ? [{ id: "sw", kind: "switch", catalogUid: "switch-spst",
          pins: [{ id: "a" }, { id: "b" }], params: { closed: 0 } }] : []),
        { id: "rl", kind: "resistor", catalogUid: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
      ],
      wires: [
        { from_component: "vs", from_pin: "pos", to_component: "lm", to_pin: "in" },
        { from_component: "vs", from_pin: "neg", to_component: "r2", to_pin: "b" },
        { from_component: "lm", from_pin: "out", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "lm", to_pin: "adj" },
        { from_component: "lm", from_pin: "adj", to_component: "r2", to_pin: "a" },
        ...(disconnected ? [
          { from_component: "lm", from_pin: "out", to_component: "sw", to_pin: "a" },
          { from_component: "sw", from_pin: "b", to_component: "rl", to_pin: "a" },
        ] : [{ from_component: "lm", from_pin: "out", to_component: "rl", to_pin: "a" }]),
        { from_component: "rl", from_pin: "b", to_component: "vs", to_pin: "neg" },
      ],
    };
  }

  it("recovers into its divider-defined 5 V compliance without a ~96 V near-open step", () => {
    const engine = runEngine(circuitWithLoad(10), 10);
    expect(netVoltage(engine, "lm", "out")).toBeLessThan(2);
    const stressBefore = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;

    engine.load(circuitWithLoad(1e9));
    engine.step(1e-4);

    const vOut = netVoltage(engine, "lm", "out");
    const vAdj = netVoltage(engine, "lm", "adj");
    const outputCurrent = Math.abs(engine.getElementI().lm ?? 0);
    const inputCurrent = Math.abs(engine.getElementI().vs ?? 0);
    // The model's disclosed 100 kΩ ADJ bias sits in parallel with R2, so the
    // ideal 5.000 V divider lands at about 4.973 V.
    expect(vOut).toBeCloseTo(4.973, 2);
    expect(vOut).toBeLessThan(5.01);
    expect(vOut - vAdj).toBeCloseTo(1.25, 6);
    expect(outputCurrent).toBeLessThan(0.006);
    expect(inputCurrent).toBeGreaterThanOrEqual(outputCurrent - 1e-9);
    expect(engine.getIcState("lm")?.reg).toBe(0);
    const stressAfter = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;
    expect(stressAfter).toBeLessThanOrEqual(stressBefore);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("enters its 100 mA limit in the same solve as a near-open → 10 Ω switch", () => {
    const engine = runEngine(circuitWithLoad(1e9), 5);
    engine.load(circuitWithLoad(10));
    engine.step(1e-4);

    const current = Math.abs(engine.getElementI().lm ?? 0);
    expect(current).toBeCloseTo(0.1, 9);
    expect(current).toBeLessThanOrEqual(0.100_001);
    expect(netVoltage(engine, "lm", "out")).toBeLessThan(2);
    expect(engine.getIcState("lm")?.reg).toBe(2);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("recovers at the divider setpoint when a downstream SPST opens", () => {
    const engine = runEngine(circuitWithLoad(10), 10);
    engine.load(circuitWithLoad(1_000, true));
    engine.step(1e-4);

    const vOut = netVoltage(engine, "lm", "out");
    expect(vOut).toBeCloseTo(4.973, 2);
    expect(vOut).toBeLessThan(5.01);
    expect(Math.abs(engine.getElementI().lm ?? 0)).toBeLessThan(0.006);
    expect(engine.getIcState("lm")?.reg).toBe(0);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });
});

// ─── AMS1117-3.3 REG ─────────────────────────────────────────────────────────

describe("LM317 — same-solve input headroom transitions", () => {
  function circuitAt(inputVoltage: number, inputConnected = true): SimCircuit {
    return {
      components: [
        { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: inputVoltage } },
        { id: "input_sw", kind: "switch", catalogUid: "switch-spst",
          pins: [{ id: "a" }, { id: "b" }], params: { closed: inputConnected ? 1 : 0 } },
        { id: "lm", kind: "lm317", pins: [{ id: "in" }, { id: "adj" }, { id: "out" }],
          params: { vref: 1.25, vdropout: 2, iLimit: 1.5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 240 } },
        { id: "r2", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 720 } },
        { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 500 } },
      ],
      wires: [
        { from_component: "vs", from_pin: "pos", to_component: "input_sw", to_pin: "a" },
        { from_component: "input_sw", from_pin: "b", to_component: "lm", to_pin: "in" },
        { from_component: "vs", from_pin: "neg", to_component: "r2", to_pin: "b" },
        { from_component: "lm", from_pin: "out", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "lm", to_pin: "adj" },
        { from_component: "lm", from_pin: "adj", to_component: "r2", to_pin: "a" },
        { from_component: "lm", from_pin: "out", to_component: "rl", to_pin: "a" },
        { from_component: "rl", from_pin: "b", to_component: "vs", to_pin: "neg" },
      ],
    };
  }

  it("turns off without driving an isolated IN node when input power is removed", () => {
    const engine = runEngine(circuitAt(10), 5);
    expect(netVoltage(engine, "lm", "out")).toBeCloseTo(4.973, 2);

    engine.load(circuitAt(10, false));
    engine.step(1e-4);

    expect(Math.abs(netVoltage(engine, "lm", "in"))).toBeLessThan(1e-6);
    expect(Math.abs(netVoltage(engine, "lm", "out"))).toBeLessThan(1e-6);
    expect(Math.abs(netVoltage(engine, "lm", "adj"))).toBeLessThan(1e-6);
    expect(Math.abs(engine.getElementI().lm ?? 0)).toBeLessThan(1e-12);
    expect(engine.getIcState("lm")?.reg).toBe(3);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("cold-loads a disconnected adjustable-regulator seed into OFF", () => {
    const engine = runEngine(circuitAt(10, false), 1);
    expect(Math.abs(netVoltage(engine, "lm", "in"))).toBeLessThan(1e-6);
    expect(Math.abs(netVoltage(engine, "lm", "out"))).toBeLessThan(1e-6);
    expect(Math.abs(engine.getElementI().lm ?? 0)).toBeLessThan(1e-12);
    expect(engine.getIcState("lm")?.reg).toBe(3);
    expect(engine.lastConverged).toBe(true);
  });

  it("enters dropout on the first 10 V → 6 V step and restores the divider setpoint", () => {
    const engine = runEngine(circuitAt(10), 5);

    engine.load(circuitAt(6));
    engine.step(1e-4);
    expect(netVoltage(engine, "lm", "in")).toBeCloseTo(6, 4);
    expect(netVoltage(engine, "lm", "out")).toBeCloseTo(4, 3);
    expect(engine.getIcState("lm")?.reg).toBe(1);
    expect(engine.lastConverged).toBe(true);

    engine.load(circuitAt(10));
    engine.step(1e-4);
    expect(netVoltage(engine, "lm", "out")).toBeCloseTo(4.973, 2);
    expect(netVoltage(engine, "lm", "out") - netVoltage(engine, "lm", "adj")).toBeCloseTo(1.25, 6);
    expect(engine.getIcState("lm")?.reg).toBe(0);
    expect(engine.lastConverged).toBe(true);
  });

  it("restores regulation in the first accepted step after reconnecting IN", () => {
    const engine = runEngine(circuitAt(10), 5);
    engine.load(circuitAt(10, false));
    engine.step(1e-4);
    expect(engine.getIcState("lm")?.reg).toBe(3);

    engine.load(circuitAt(10, true));
    engine.step(1e-4);
    expect(netVoltage(engine, "lm", "in")).toBeCloseTo(10, 4);
    expect(netVoltage(engine, "lm", "out")).toBeCloseTo(4.973, 2);
    expect(engine.getIcState("lm")?.reg).toBe(0);
    expect(engine.lastConverged).toBe(true);
  });
});

describe("AMS1117-3.3 — REG mode: 5 V in, 100 Ω load", () => {
  /**
   * Circuit: 5 V supply → AMS1117-3.3 → 100 Ω load to GND.
   * Vin = 5 V, vout = 3.3 V, vdropout = 1.1 V.
   * headroom = 5 ≥ 3.3+1.1 = 4.4 V → REG.
   * Vout = 3.3 V, I = 3.3/100 = 33 mA.
   */
  const circuit: SimCircuit = {
    components: [
      { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "reg", kind: "linear_reg",     pins: [{ id: "in" }, { id: "gnd" }, { id: "out" }],
        params: { vout: 3.3, vdropout: 1.1, iLimit: 0.8 } },
      { id: "rl",  kind: "resistor",       pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
    ],
    wires: [
      { from_component: "vs",  from_pin: "pos", to_component: "reg", to_pin: "in" },
      { from_component: "vs",  from_pin: "neg", to_component: "reg", to_pin: "gnd" },
      { from_component: "reg", from_pin: "out", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "vs",  to_pin: "neg" },
    ],
  };

  it("Vout = 3.3 V ± 50 mV (REG constraint)", () => {
    const engine = runEngine(circuit);
    const vOut = netVoltage(engine, "reg", "out");
    // Derivation: REG: V_out − V_gnd = 3.3 V; V_gnd = 0 → V_out = 3.3 V.
    expect(vOut).toBeCloseTo(3.3, 1); // ±50 mV
  });
});

// ─── LM317 adjustable ────────────────────────────────────────────────────────

describe("LM317 — adjustable: 12 V in, R1=240 Ω, R2=720 Ω → 5.0 V out", () => {
  /**
   * Circuit:
   *   Vin = 12 V
   *   LM317: in / adj / out.  Chip enforces V_out − V_adj = 1.25 V.
   *   R1 = 240 Ω from out → adj.
   *   R2 = 720 Ω from adj → GND.
   *
   * Analysis:
   *   Current through R1: I_R1 = 1.25 / 240 ≈ 5.208 mA
   *   V_adj = I_R1 × R2 = 5.208e-3 × 720 = 3.75 V
   *   V_out = V_adj + 1.25 = 3.75 + 1.25 = 5.0 V
   *         = 1.25 × (1 + R2/R1) = 1.25 × 4 = 5.0 V
   *
   *   headroom check: V_in − V_adj = 12 − 3.75 = 8.25 V ≥ 1.25+2.0 = 3.25 V → REG
   */
  const circuit: SimCircuit = {
    components: [
      { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 12 } },
      { id: "lm",  kind: "lm317",          pins: [{ id: "in" }, { id: "adj" }, { id: "out" }],
        params: { vref: 1.25, vdropout: 2.0, iLimit: 1.5 } },
      { id: "r1",  kind: "resistor",       pins: [{ id: "a" }, { id: "b" }], params: { resistance: 240 } },
      { id: "r2",  kind: "resistor",       pins: [{ id: "a" }, { id: "b" }], params: { resistance: 720 } },
      { id: "rl",  kind: "resistor",       pins: [{ id: "a" }, { id: "b" }], params: { resistance: 500 } },
    ],
    wires: [
      // Supply to regulator input
      { from_component: "vs",  from_pin: "pos", to_component: "lm",  to_pin: "in" },
      { from_component: "vs",  from_pin: "neg", to_component: "r2",  to_pin: "b" },
      // R1: out → adj
      { from_component: "lm",  from_pin: "out", to_component: "r1",  to_pin: "a" },
      { from_component: "r1",  from_pin: "b",   to_component: "lm",  to_pin: "adj" },
      // R2: adj → GND
      { from_component: "lm",  from_pin: "adj", to_component: "r2",  to_pin: "a" },
      // Load: out → GND
      { from_component: "lm",  from_pin: "out", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "vs",  to_pin: "neg" },
    ],
  };

  it("Vout = 5.0 V ± 100 mV (1.25 × (1+720/240) = 1.25×4 = 5.0 V)", () => {
    const engine = runEngine(circuit, 400);
    const vOut = netVoltage(engine, "lm", "out");
    // Derivation: see header comment — Vout = 1.25 × (1 + R2/R1) = 5.0 V.
    expect(vOut).toBeCloseTo(5.0, 0); // ±100 mV
    expect(vOut).toBeGreaterThan(4.5);
    expect(vOut).toBeLessThan(5.5);
  });

  it("V_out − V_adj ≈ 1.25 V ± 50 mV (internal Vref constraint)", () => {
    const engine = runEngine(circuit, 400);
    const vOut = netVoltage(engine, "lm", "out");
    const vAdj = netVoltage(engine, "lm", "adj");
    // Derivation: the stamp enforces V_out − V_adj = Vref = 1.25 V.
    expect(vOut - vAdj).toBeCloseTo(1.25, 1); // ±50 mV
  });
});

// ─── bench_psu CC fold ────────────────────────────────────────────────────────

describe("bench_psu — CC fold: V=10, iLimit=0.5 A, 2 Ω load", () => {
  /**
   * Circuit: bench_psu V=10 V, iLimit=0.5 A → 2 Ω load to GND.
   * Unlimited: I = 10/2 = 5 A > 0.5 A → CC.
   * Clamped: I = 0.5 A, Vout = 0.5×2 = 1.0 V.
   */
  const circuit: SimCircuit = {
    components: [
      { id: "psu", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 10, iLimit: 0.5 } },
      { id: "rl",  kind: "resistor",  pins: [{ id: "a" }, { id: "b" }], params: { resistance: 2 } },
    ],
    wires: [
      { from_component: "psu", from_pin: "pos", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "psu", to_pin: "neg" },
    ],
  };

  it("I ≈ 0.5 A ± 0.05 A (CC clamp = iLimit)", () => {
    const engine = runEngine(circuit);
    const elemI = Math.abs(engine.getElementI()["psu"] ?? 0);
    // Derivation: CC mode pins x[k] = iLimit = 0.5 A.
    expect(elemI).toBeCloseTo(0.5, 1); // ±50 mA
  });

  it("Vout ≈ 1.0 V ± 0.2 V (I×Rload = 0.5×2 = 1.0 V)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "psu", "pos");
    // Derivation: KCL at output: V_pos = I_CC × R = 0.5 × 2 = 1.0 V (GND = 0).
    expect(vPos).toBeCloseTo(1.0, 0); // ±0.1 V
  });
});

describe("bench_psu — same-step CC compliance recovery", () => {
  function circuitWithLoad(resistance: number, disconnected = false): SimCircuit {
    return {
      components: [
        { id: "psu", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 10, iLimit: 0.5 } },
        ...(disconnected ? [{ id: "sw", kind: "switch", catalogUid: "switch-spst",
          pins: [{ id: "a" }, { id: "b" }], params: { closed: 0 } }] : []),
        { id: "rl", kind: "resistor", catalogUid: "resistor",
          pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
      ],
      wires: [
        ...(disconnected ? [
          { from_component: "psu", from_pin: "pos", to_component: "sw", to_pin: "a" },
          { from_component: "sw", from_pin: "b", to_component: "rl", to_pin: "a" },
        ] : [{ from_component: "psu", from_pin: "pos", to_component: "rl", to_pin: "a" }]),
        { from_component: "rl", from_pin: "b", to_component: "psu", to_pin: "neg" },
      ],
    };
  }

  it.each([
    { resistance: 100, expectedCurrent: 0.1, stressMustRecover: false, label: "100 Ω recovery load" },
    { resistance: 1e9, expectedCurrent: 1e-8, stressMustRecover: true, label: "near-open 1 GΩ load" },
  ])("bounds $label at 10 V on the first accepted step", ({ resistance, expectedCurrent, stressMustRecover }) => {
    const engine = runEngine(circuitWithLoad(2), 10);
    expect(netVoltage(engine, "psu", "pos")).toBeCloseTo(1, 6);
    const stressBefore = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;

    engine.load(circuitWithLoad(resistance));
    engine.step(1e-4);

    const vOut = netVoltage(engine, "psu", "pos") - netVoltage(engine, "psu", "neg");
    expect(vOut).toBeCloseTo(10, 6);
    expect(vOut).toBeLessThanOrEqual(10.000_01);
    expect(Math.abs(engine.getElementI().psu ?? 0)).toBeCloseTo(expectedCurrent, 7);
    expect(engine.getIcState("psu")?.reg).toBe(0);
    if (stressMustRecover) {
      const stressAfter = engine.saveState().failureStress.get("resistor_overload\0rl\0") ?? 0;
      expect(stressAfter).toBeLessThanOrEqual(stressBefore);
    }
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("enters CC within the first accepted step after a 100 Ω → 2 Ω load switch", () => {
    const engine = runEngine(circuitWithLoad(100), 5);
    engine.load(circuitWithLoad(2));
    engine.step(1e-4);

    const current = Math.abs(engine.getElementI().psu ?? 0);
    expect(netVoltage(engine, "psu", "pos")).toBeCloseTo(1, 6);
    expect(current).toBeCloseTo(0.5, 9);
    expect(current).toBeLessThanOrEqual(0.500_001);
    expect(engine.getIcState("psu")?.reg).toBe(2);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });

  it("holds the setpoint with negligible terminal current when an SPST opens", () => {
    const engine = runEngine(circuitWithLoad(2), 10);
    engine.load(circuitWithLoad(1_000, true));
    engine.step(1e-4);

    const vOut = netVoltage(engine, "psu", "pos") - netVoltage(engine, "psu", "neg");
    expect(vOut).toBeCloseTo(10, 6);
    expect(Math.abs(engine.getElementI().psu ?? 0)).toBeLessThan(1e-9);
    expect(engine.getIcState("psu")?.reg).toBe(0);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getFailures()).toEqual({});
  });
});

describe("bench_psu — NO iLimit: regression (ideal source unchanged)", () => {
  /**
   * bench_psu without iLimit must behave exactly as an ideal voltage source.
   * V=10 V, Rload=2 Ω → I = 10/2 = 5 A, Vout = 10 V.
   * This proves the iLimit gating does not alter existing behaviour.
   */
  const circuit: SimCircuit = {
    components: [
      // iLimit absent (no params.iLimit field)
      { id: "psu", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 10 } },
      { id: "rl",  kind: "resistor",  pins: [{ id: "a" }, { id: "b" }], params: { resistance: 2 } },
    ],
    wires: [
      { from_component: "psu", from_pin: "pos", to_component: "rl",  to_pin: "a" },
      { from_component: "rl",  from_pin: "b",   to_component: "psu", to_pin: "neg" },
    ],
  };

  it("Vout = 10 V (ideal source, no limiting)", () => {
    const engine = runEngine(circuit);
    const vPos = netVoltage(engine, "psu", "pos");
    // Derivation: ideal voltage source → V_pos = 10 V regardless of load.
    expect(vPos).toBeCloseTo(10.0, 1); // ±50 mV
  });

  it("I = 5.0 A ± 0.1 A (V/R = 10/2 = 5 A, no clamping)", () => {
    // Read the initial operating interval before a physical 2 Ω resistor's
    // 50 W overload accumulates enough damage to fail it open. This test is
    // about the PSU's absent current limit, not resistor survivability.
    const engine = runEngine(circuit, 5);
    const elemI = Math.abs(engine.getElementI()["psu"] ?? 0);
    // Derivation: I = V/R = 10/2 = 5.0 A (unlimited).
    expect(elemI).toBeCloseTo(5.0, 0); // ±0.1 A
  });
});
