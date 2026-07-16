/**
 * Wave 4.2 engine tests: LM393 open-collector comparator and TL431 shunt reference.
 *
 * All expected values are derived from first principles; no value is read back
 * from the implementation and then asserted.  Self-confirming tests were the
 * cardinal sin that shipped wrong physics in a prior wave.
 *
 * Derivation comments precede each expect() call.
 */

import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const catalog = rawCatalog;

// ─── Helpers ──────────────────────────────────────────────────────────────────

function runEngine(circuit: SimCircuit, steps = 200, dt = 1e-4): SimEngine {
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

describe("catalog entries — lm393 and tl431", () => {
  it("lm393 part is defined and correct", () => {
    const part = catalog.parts.find((p) => p.uid === "lm393");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("lm393");
    // Pinout must have open_collector outputs on pins 1 and 7
    const pin1 = part?.pin_layout.find((p) => p.id === "1");
    const pin7 = part?.pin_layout.find((p) => p.id === "7");
    expect(pin1?.function).toBe("open_collector");
    expect(pin7?.function).toBe("open_collector");
    expect(part?.electrical_specs?.vcc_range?.min).toBe(2);
    expect(part?.electrical_specs?.vcc_range?.max).toBe(36);
    expect(part?.electrical_specs?.io_max).toBe(0.016);
    expect(part?.bom?.package).toBe("DIP-8");
  });

  it("tl431 part is defined and correct", () => {
    const part = catalog.parts.find((p) => p.uid === "tl431");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("tl431");
    // Must have three pins: ref, anode, cathode
    const pinRef  = part?.pin_layout.find((p) => p.id === "ref");
    const pinAno  = part?.pin_layout.find((p) => p.id === "anode");
    const pinKath = part?.pin_layout.find((p) => p.id === "cathode");
    expect(pinRef).toBeDefined();
    expect(pinAno).toBeDefined();
    expect(pinKath).toBeDefined();
    expect(part?.electrical_specs?.p_max).toBe(0.5);
    // Notes must mention 2.495 V reference
    expect(part?.electrical_specs?.notes).toContain("2.495");
  });
});

// ─── LM393 — output released when IN+ > IN- ────────────────────────────────────

describe("LM393 — comparator unit 1 released (IN+ > IN-)", () => {
  /**
   * Circuit: 5 V supply powers LM393.
   * IN1+ (pin 3) = 3.0 V (from voltage source).
   * IN1- (pin 2) = 1.0 V (from voltage source).
   * 10 kΩ pull-up from OUT1 (pin 1) to VCC (5 V).
   *
   * Decision: vP - vM = 3.0 - 1.0 = 2.0 V > +1 mV hysteresis → released (hi-Z).
   * Stamped: nothing on OUT1 node — no sink resistor.
   * Result: the 10k pull-up holds OUT1 at 5 V with no load.
   * Expected: V_out ≈ 5 V.  Sink current ≈ 0.
   */
  it("OUT1 ≈ 5 V when IN1+ > IN1- (pull-up holds high, no sink)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vp",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3.0 } },
        { id: "vm",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.0 } },
        { id: "rpu", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "cmp", kind: "lm393", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        // Power
        { from_component: "vs",  from_pin: "pos", to_component: "cmp", to_pin: "8" },
        { from_component: "vs",  from_pin: "neg", to_component: "cmp", to_pin: "4" },
        // IN1+ = 3.0 V
        { from_component: "vp",  from_pin: "pos", to_component: "cmp", to_pin: "3" },
        { from_component: "vp",  from_pin: "neg", to_component: "vs",  to_pin: "neg" },
        // IN1- = 1.0 V
        { from_component: "vm",  from_pin: "pos", to_component: "cmp", to_pin: "2" },
        { from_component: "vm",  from_pin: "neg", to_component: "vs",  to_pin: "neg" },
        // 10k pull-up: VCC → rpu → OUT1
        { from_component: "vs",  from_pin: "pos", to_component: "rpu", to_pin: "a" },
        { from_component: "rpu", from_pin: "b",   to_component: "cmp", to_pin: "1" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "cmp", "1");
    // Released: pull-up holds OUT1 at 5 V; accept ±100 mV (numerical RSHUNT current is negligible).
    expect(vout).toBeGreaterThan(4.900);
    expect(vout).toBeLessThan(5.100);
  });
});

// ─── LM393 — output sinking when IN+ < IN- ────────────────────────────────────

describe("LM393 — comparator unit 1 sinking (IN+ < IN-)", () => {
  /**
   * Circuit: 5 V supply, same pull-up topology.
   * IN1+ (pin 3) = 1.0 V, IN1- (pin 2) = 3.0 V.
   *
   * Decision: vP - vM = 1.0 - 3.0 = -2.0 V < -1 mV → sinking.
   *
   * R_sink computation (from _icPowerInfo for LM393 on 5 V with io_max=0.016):
   *   R_sink = max(50, min(1000, vSupply / (io_max * 4)))
   *          = max(50, min(1000, 5 / (0.016 * 4)))
   *          = max(50, min(1000, 78.125))
   *          = 78.125 Ω
   *
   * Voltage divider: OUT1 to GND via R_sink; VCC (5V) to OUT1 via R_pullup (10k).
   *   V_out = 5 * R_sink / (R_pullup + R_sink)
   *         = 5 * 78.125 / (10000 + 78.125)
   *         = 390.625 / 10078.125
   *         ≈ 0.03876 V  (~38.8 mV)
   *
   * Sink current I = V_out / R_sink ≈ 0.03876 / 78.125 ≈ 0.000496 A ≈ 0.496 mA
   * OR equivalently I ≈ 5 / (10000 + 78.125) ≈ 4.961e-4 A ≈ 0.496 mA
   */
  it("OUT1 ≈ 39 mV when sinking (voltage divider: R_sink=78.125Ω / 10kΩ pull-up)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vp",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.0 } },
        { id: "vm",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3.0 } },
        { id: "rpu", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "cmp", kind: "lm393", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        { from_component: "vs",  from_pin: "pos", to_component: "cmp", to_pin: "8" },
        { from_component: "vs",  from_pin: "neg", to_component: "cmp", to_pin: "4" },
        { from_component: "vp",  from_pin: "pos", to_component: "cmp", to_pin: "3" },
        { from_component: "vp",  from_pin: "neg", to_component: "vs",  to_pin: "neg" },
        { from_component: "vm",  from_pin: "pos", to_component: "cmp", to_pin: "2" },
        { from_component: "vm",  from_pin: "neg", to_component: "vs",  to_pin: "neg" },
        { from_component: "vs",  from_pin: "pos", to_component: "rpu", to_pin: "a" },
        { from_component: "rpu", from_pin: "b",   to_component: "cmp", to_pin: "1" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "cmp", "1");
    // V_out = 5 * 78.125 / 10078.125 ≈ 0.0388 V; accept ±20 mV
    expect(vout).toBeGreaterThan(0.000);
    expect(vout).toBeLessThan(0.060);
  });
});

// ─── LM393 — threshold crossing flips across two runs ─────────────────────────

describe("LM393 — threshold crossing flips output state", () => {
  /**
   * Run 1: IN1+ = 2.0 V, IN1- = 1.0 V → diff = +1.0 V → released → OUT ~= 5V
   * Run 2: IN1+ = 1.0 V, IN1- = 2.0 V → diff = -1.0 V → sinking  → OUT ~= 0V
   * These are two independent SimEngine instances, not a reload.
   */
  it("output is released when IN+>IN- and sinking when IN+<IN- (two separate runs)", () => {
    function makeCircuit(vp: number, vm: number): SimCircuit {
      return {
        components: [
          { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
          { id: "vp",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: vp } },
          { id: "vm",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: vm } },
          { id: "rpu", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
          { id: "cmp", kind: "lm393", pins: [
            { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
            { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
          ], params: {} },
        ],
        wires: [
          { from_component: "vs",  from_pin: "pos", to_component: "cmp", to_pin: "8" },
          { from_component: "vs",  from_pin: "neg", to_component: "cmp", to_pin: "4" },
          { from_component: "vp",  from_pin: "pos", to_component: "cmp", to_pin: "3" },
          { from_component: "vp",  from_pin: "neg", to_component: "vs",  to_pin: "neg" },
          { from_component: "vm",  from_pin: "pos", to_component: "cmp", to_pin: "2" },
          { from_component: "vm",  from_pin: "neg", to_component: "vs",  to_pin: "neg" },
          { from_component: "vs",  from_pin: "pos", to_component: "rpu", to_pin: "a" },
          { from_component: "rpu", from_pin: "b",   to_component: "cmp", to_pin: "1" },
        ],
      };
    }

    // Run 1: IN+ > IN- → released, OUT ≈ 5 V
    const eng1 = runEngine(makeCircuit(2.0, 1.0));
    expect(eng1.lastConverged).toBe(true);
    const vout1 = netVoltage(eng1, "cmp", "1");
    // Released: pull-up holds near 5 V
    expect(vout1).toBeGreaterThan(4.8);

    // Run 2: IN+ < IN- → sinking, OUT ≈ 0 V
    const eng2 = runEngine(makeCircuit(1.0, 2.0));
    expect(eng2.lastConverged).toBe(true);
    const vout2 = netVoltage(eng2, "cmp", "1");
    // Sinking: OUT pulled low
    expect(vout2).toBeLessThan(0.2);

    // The two outputs are on opposite sides of the midpoint
    expect(vout1).toBeGreaterThan(vout2);
  });
});

// ─── LM393 — hysteresis: dither near threshold does not chatter ───────────────

describe("LM393 — hysteresis band: dither within ±1 mV does not flip committed state", () => {
  /**
   * Start with IN+ = 2.0 V, IN- = 1.0 V → committed as released.
   * Then reload the same engine with IN+ = IN- + 0.5 mV (within the 1 mV band).
   * Committed output must STAY released (not flip to sinking).
   *
   * The hysteresis band is ±1 mV = ±0.001 V.  0.5 mV < 1 mV → keep-previous rule.
   */
  it("committed output stays released when dithered within ±1 mV of threshold", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vp",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 2.0 } },
        { id: "vm",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.0 } },
        { id: "rpu", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "cmp", kind: "lm393", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        { from_component: "vs",  from_pin: "pos", to_component: "cmp", to_pin: "8" },
        { from_component: "vs",  from_pin: "neg", to_component: "cmp", to_pin: "4" },
        { from_component: "vp",  from_pin: "pos", to_component: "cmp", to_pin: "3" },
        { from_component: "vp",  from_pin: "neg", to_component: "vs",  to_pin: "neg" },
        { from_component: "vm",  from_pin: "pos", to_component: "cmp", to_pin: "2" },
        { from_component: "vm",  from_pin: "neg", to_component: "vs",  to_pin: "neg" },
        { from_component: "vs",  from_pin: "pos", to_component: "rpu", to_pin: "a" },
        { from_component: "rpu", from_pin: "b",   to_component: "cmp", to_pin: "1" },
      ],
    };

    const eng = new SimEngine();
    eng.load(circuit);
    // Phase 1: run with IN+ = 2.0 V >> IN- = 1.0 V → committed as released
    for (let i = 0; i < 100; i++) eng.step(1e-4);
    const vAfterPhase1 = netVoltage(eng, "cmp", "1");
    // Released: OUT ≈ 5 V
    expect(vAfterPhase1).toBeGreaterThan(4.8);

    // Phase 2: dither IN+ to IN- + 0.5 mV (diff = 0.0005 V < 1 mV band → keep released)
    circuit.components.find((c) => c.id === "vm")!.params.voltage = 2.0 - 0.0005;
    eng.load(circuit);  // load() preserves icState by component id
    for (let i = 0; i < 100; i++) eng.step(1e-4);
    const vAfterDither = netVoltage(eng, "cmp", "1");
    // Must remain released (OUT still near 5 V), not chatter to sinking
    expect(vAfterDither).toBeGreaterThan(4.5);
  });
});

// ─── LM393 — unit 2 independent of unit 1 ────────────────────────────────────

describe("LM393 — dual units operate independently", () => {
  /**
   * Unit 1: IN1+ = 3 V, IN1- = 1 V → released (OUT1 ≈ 5 V)
   * Unit 2: IN2+ = 1 V, IN2- = 3 V → sinking  (OUT2 ≈ 0 V)
   * Pull-ups on both outputs.
   */
  it("unit 1 released and unit 2 sinking simultaneously in the same chip", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",   kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vp1",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3.0 } },
        { id: "vm1",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.0 } },
        { id: "vp2",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.0 } },
        { id: "vm2",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3.0 } },
        { id: "rpu1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "rpu2", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "cmp",  kind: "lm393", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        // Power
        { from_component: "vs",   from_pin: "pos", to_component: "cmp",  to_pin: "8" },
        { from_component: "vs",   from_pin: "neg", to_component: "cmp",  to_pin: "4" },
        // Unit 1
        { from_component: "vp1",  from_pin: "pos", to_component: "cmp",  to_pin: "3" },
        { from_component: "vp1",  from_pin: "neg", to_component: "vs",   to_pin: "neg" },
        { from_component: "vm1",  from_pin: "pos", to_component: "cmp",  to_pin: "2" },
        { from_component: "vm1",  from_pin: "neg", to_component: "vs",   to_pin: "neg" },
        { from_component: "vs",   from_pin: "pos", to_component: "rpu1", to_pin: "a" },
        { from_component: "rpu1", from_pin: "b",   to_component: "cmp",  to_pin: "1" },
        // Unit 2
        { from_component: "vp2",  from_pin: "pos", to_component: "cmp",  to_pin: "5" },
        { from_component: "vp2",  from_pin: "neg", to_component: "vs",   to_pin: "neg" },
        { from_component: "vm2",  from_pin: "pos", to_component: "cmp",  to_pin: "6" },
        { from_component: "vm2",  from_pin: "neg", to_component: "vs",   to_pin: "neg" },
        { from_component: "vs",   from_pin: "pos", to_component: "rpu2", to_pin: "a" },
        { from_component: "rpu2", from_pin: "b",   to_component: "cmp",  to_pin: "7" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout1 = netVoltage(eng, "cmp", "1");
    const vout2 = netVoltage(eng, "cmp", "7");
    // Unit 1 released: OUT1 ≈ 5 V
    expect(vout1).toBeGreaterThan(4.8);
    // Unit 2 sinking: OUT2 ≈ 0 V
    expect(vout2).toBeLessThan(0.2);
  });
});

// ─── LM393 — unpowered → released (hi-Z) ─────────────────────────────────────

describe("LM393 — unpowered: output released (hi-Z), no sink", () => {
  /**
   * No supply wires connected.  Pull-up from OUT1 to a 5V source (external).
   * Unpowered → _icPowerInfo powered=false → commit sets out1=1 (released).
   * Output floats; pull-up holds OUT1 at 5 V.
   */
  it("unpowered LM393 has released output — pull-up holds OUT1 at ~5 V", () => {
    const circuit: SimCircuit = {
      components: [
        // External 5V for pull-up only — NOT connected to LM393 supply pins
        { id: "vext", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "rpu",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "vp",   kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3.0 } },
        { id: "vm",   kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 1.0 } },
        { id: "cmp",  kind: "lm393", pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ], params: {} },
      ],
      wires: [
        // NO wires to cmp pin 8 (VCC) or pin 4 (GND) — unpowered
        // Inputs still driven (inputs are analog_in; unpowered path releases output)
        { from_component: "vp",   from_pin: "pos", to_component: "cmp",  to_pin: "3" },
        { from_component: "vp",   from_pin: "neg", to_component: "vext", to_pin: "neg" },
        { from_component: "vm",   from_pin: "pos", to_component: "cmp",  to_pin: "2" },
        { from_component: "vm",   from_pin: "neg", to_component: "vext", to_pin: "neg" },
        // Pull-up from external 5V to OUT1
        { from_component: "vext", from_pin: "pos", to_component: "rpu",  to_pin: "a" },
        { from_component: "rpu",  from_pin: "b",   to_component: "cmp",  to_pin: "1" },
      ],
    };
    const eng = runEngine(circuit);
    expect(eng.lastConverged).toBe(true);
    const vout = netVoltage(eng, "cmp", "1");
    // Released: pull-up holds near 5 V
    expect(vout).toBeGreaterThan(4.8);
  });
});

// ─── TL431 — canonical adjustable shunt (R1=R2=10k) ──────────────────────────

describe("TL431 — adjustable shunt regulator Vout = Vref*(1+R1/R2)", () => {
  /**
   * Circuit:
   *   12 V → Rs (1 kΩ) → cathode
   *   cathode → R1 (10 kΩ) → REF
   *   REF → R2 (10 kΩ) → anode → GND
   *
   * At regulation:
   *   V(REF) − V(ANODE) = Vref = 2.495 V  (ideal)
   *   V(ANODE) = 0 V (tied to GND)
   *   V(REF) = Vref = 2.495 V  (ideal)
   *   V(cathode) = 2 * Vref = 4.990 V  (ideal, R1 = R2)
   *
   * Model offset: the exponential knee (nZ=0.5, Vt=25.85 mV) must be driven above
   * Vref to sink the required shunt current.  For I_ka = 7 mA the required overdrive
   * at REF is δ = nZ * Vt * ln(I/Ik) = 0.5*0.02585*ln(7) ≈ 25 mV, so
   *   V(REF)_actual ≈ 2.520 V  → V(cath) ≈ 5.04 V
   * Additionally, the 1 MΩ bias resistor from REF to anode shifts the effective
   * divider ratio, adding another ~10 mV.  Combined model offset ≈ +50–80 mV.
   *
   * Accept range [4.940 V, 5.100 V] (centred on the ideal 4.990 V ± 110 mV to cover
   * the exponential knee offset at full shunt current).
   *
   * Shunt current: I = (12 − vCath) / Rs.  Accept 5.9–8.1 mA (±15 % of 7.01 mA).
   */
  it("cathode ≈ 4.99 V and shunt current ≈ 7 mA with R1=R2=10k on 12 V supply", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 12 } },
        { id: "rs",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "r1",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "r2",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "u1",  kind: "tl431", pins: [
          { id: "ref" }, { id: "anode" }, { id: "cathode" },
        ], params: {} },
      ],
      wires: [
        // 12V → Rs → cathode
        { from_component: "vs",  from_pin: "pos", to_component: "rs",  to_pin: "a" },
        { from_component: "rs",  from_pin: "b",   to_component: "u1",  to_pin: "cathode" },
        // cathode → R1 → REF
        { from_component: "u1",  from_pin: "cathode", to_component: "r1",  to_pin: "a" },
        { from_component: "r1",  from_pin: "b",        to_component: "u1",  to_pin: "ref" },
        // REF → R2 → anode
        { from_component: "u1",  from_pin: "ref",   to_component: "r2",  to_pin: "a" },
        { from_component: "r2",  from_pin: "b",     to_component: "u1",  to_pin: "anode" },
        // anode → GND
        { from_component: "u1",  from_pin: "anode", to_component: "vs",  to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 400);
    expect(eng.lastConverged).toBe(true);
    const vCath = netVoltage(eng, "u1", "cathode");
    // Ideal: 4.990 V.  Model adds ~50–80 mV from exponential knee offset and REF bias.
    // Derivation: δ_ref = nZ*Vt*ln(7/Ik) = 0.5*0.02585*ln(7) ≈ 0.025 V overdrive;
    //   V_cath ≈ 4.99 + 2*0.025 = 5.04 V (divider doubles the shift);
    //   plus ~10 mV from 1 MΩ bias distorting the divider ratio.
    expect(vCath).toBeGreaterThan(4.940);
    expect(vCath).toBeLessThan(5.100);

    // Shunt current I = (12 − vCath) / Rs.
    // At vCath=5.06 V: I = 6.94 / 1000 = 6.94 mA.  Accept 5.9–8.1 mA (±15 %).
    const vVsPos = netVoltage(eng, "vs", "pos");
    const iShunt = (vVsPos - vCath) / 1000;
    expect(iShunt).toBeGreaterThan(0.0059);
    expect(iShunt).toBeLessThan(0.0081);
  });
});

// ─── TL431 — REF tied to cathode → Vout = Vref ────────────────────────────────

describe("TL431 — REF tied directly to cathode → cathode regulates at Vref", () => {
  /**
   * When REF = cathode (R1 = 0), the device holds V(cathode)−V(anode) = Vref = 2.495 V.
   *
   * Circuit: 12 V → Rs (1 kΩ) → cathode; REF shorted to cathode; anode → GND.
   *
   * At regulation: V(cathode) = Vref = 2.495 V
   * Current: I = (12 − 2.495) / 1000 = 9.505 mA
   *
   * Accept ±30 mV on cathode (better tolerance since no divider resistors involved).
   */
  it("cathode ≈ 2.495 V when REF is shorted to cathode", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 12 } },
        { id: "rs",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "u1",  kind: "tl431", pins: [
          { id: "ref" }, { id: "anode" }, { id: "cathode" },
        ], params: {} },
      ],
      wires: [
        // 12V → Rs → cathode
        { from_component: "vs",  from_pin: "pos",     to_component: "rs",  to_pin: "a" },
        { from_component: "rs",  from_pin: "b",       to_component: "u1",  to_pin: "cathode" },
        // REF shorted to cathode
        { from_component: "u1",  from_pin: "cathode", to_component: "u1",  to_pin: "ref" },
        // anode → GND
        { from_component: "u1",  from_pin: "anode",   to_component: "vs",  to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 400);
    expect(eng.lastConverged).toBe(true);
    const vCath = netVoltage(eng, "u1", "cathode");
    // Vout = Vref = 2.495 V; accept ±30 mV
    expect(vCath).toBeGreaterThan(2.465);
    expect(vCath).toBeLessThan(2.525);
  });
});

// ─── TL431 — REF below Vref → device OFF, cathode near supply ────────────────

describe("TL431 — REF below Vref: device off, cathode floats near supply", () => {
  /**
   * REF tied to GND (anode). V(ref)−V(anode) = 0 V < Vref = 2.495 V.
   *
   * Drive: driveC = 0 − 0 − 2.495 = −2.495 V → exp(-2.495/(0.5*0.02585)) ≈ exp(-193) ≈ 0
   * I_ka ≈ 0 → virtually no shunt current → cathode ≈ supply voltage.
   *
   * Circuit: 12V → Rs(1k) → cathode; REF tied to anode; anode to GND.
   * With I≈0: V(cathode) ≈ 12 V (minus only the tiny current through the 1M REF bias resistor).
   *
   * The 1M bias resistor drains some current when cathode is high:
   *   I_bias ≈ V(cathode) / 1M ≈ 12/1e6 = 12 µA (negligible)
   *   V(cathode) ≈ 12 - 12e-6 * 1000 ≈ 12 - 0.012 = 11.988 V ≈ 12 V
   *
   * Sink current < 1 µA (by construction at exp(-193)).
   */
  it("cathode ≈ 12 V when REF is tied to GND (device off)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vs",  kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 12 } },
        { id: "rs",  kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "u1",  kind: "tl431", pins: [
          { id: "ref" }, { id: "anode" }, { id: "cathode" },
        ], params: {} },
      ],
      wires: [
        // 12V → Rs → cathode
        { from_component: "vs",  from_pin: "pos",   to_component: "rs",  to_pin: "a" },
        { from_component: "rs",  from_pin: "b",     to_component: "u1",  to_pin: "cathode" },
        // REF tied to anode (GND side)
        { from_component: "u1",  from_pin: "ref",   to_component: "u1",  to_pin: "anode" },
        // anode → GND
        { from_component: "u1",  from_pin: "anode", to_component: "vs",  to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 400);
    expect(eng.lastConverged).toBe(true);
    const vCath = netVoltage(eng, "u1", "cathode");
    // Device OFF: cathode near supply, only minute current flows.
    // V(cathode) ≈ 12 * (1M / (1k + 1M)) ≈ 11.988 V; accept > 11 V
    expect(vCath).toBeGreaterThan(11.0);
  });
});
