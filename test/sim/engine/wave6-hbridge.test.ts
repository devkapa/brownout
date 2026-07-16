/**
 * Wave 6.3 engine tests: L293D and TB6612FNG dual H-bridge motor drivers,
 * and H-bridge flyback-clamp diagnostic extension (Finding 8 Part C).
 *
 * All expected values are derived from closed-form analysis with derivation
 * comments. NO value is read back from the implementation and then asserted —
 * the cardinal sin of self-confirming tests.
 *
 * ─── DERIVATIONS ────────────────────────────────────────────────────────────
 *
 * H-BRIDGE OUTPUT — push-pull resistive model:
 *   R_on = 2 Ω per output side (locked teaching value; see sim-engine.ts comments).
 *   VM = 12 V (motor supply), GND = 0 V.
 *   VCC1 (L293D) / VCC (TB6612) = 5 V (logic supply).
 *
 * L293D FORWARD (EN12 HIGH, IN1 HIGH → OUT1 HIGH):
 *   Topology: VM = 12 V → stampResistor(out1, vm, 2 Ω) → out1 → R_load → GND.
 *   Equivalent: 12 V source, 2 Ω (R_on) + R_load in series, out1 between them.
 *   V_out1 = 12 × R_load / (R_on + R_load)
 *   With R_load = 100 Ω: V_out1 = 12 × 100 / (100 + 2) = 1200 / 102 ≈ 11.7647 V
 *   (Derivation: Ohm's law on the resistive divider VM → R_on → out1 → R_load → GND.)
 *
 * L293D REVERSE (EN12 HIGH, IN1 LOW → OUT1 LOW):
 *   Topology: stampResistor(out1, gnd, 2 Ω), out1 → R_load → VM (12 V).
 *   V_out1 = VM × R_on / (R_on + R_load) = 12 × 2 / 102 ≈ 0.2353 V
 *   (Voltage divider: R_load + R_on in series; out1 is the tap point at R_on from GND.)
 *
 * L293D DISABLED (EN12 LOW → OUT1 HIZ):
 *   No stamp → OUT1 floats. With R_pull = 10 kΩ from OUT1 to +5 V and no other
 *   path, V_out1 ≈ 5 V (pull-up forces it; numerical RSHUNT current << pull-up current).
 *
 * L293D MOTOR DIRECTION (OUT1/OUT2 across a load resistor R_motor):
 *   Forward: OUT1 HIGH (toward VM), OUT2 LOW (toward GND).
 *     V_out1 = 12 × 100 / 102 ≈ 11.7647 V (same as above, R_motor=100 Ω)
 *     V_out2 = 12 × 2  / 102 ≈  0.2353 V
 *     V_motor = V_out1 − V_out2 = 11.7647 − 0.2353 = 11.5294 V (positive = forward)
 *     Exact: 12 × (100−2)/102 = 12 × 98/102 = 1176/102 ≈ 11.5294 V
 *   Reverse: IN1 LOW, IN2 HIGH → OUT1 LOW, OUT2 HIGH.
 *     V_motor = V_out1 − V_out2 = 0.2353 − 11.7647 = −11.5294 V (negative = reverse)
 *   The signed assertion proves direction reversal; magnitude ≈ 11.5294 V in both cases.
 *
 * L293D VM UNCONNECTED → all outputs HIZ:
 *   With VM open, _isOpenPin("vcc2") = true → vmValidL = false in _updateState →
 *   all committed states = −1 (HIZ) → no stamps → outputs float.
 *   A 10 kΩ pull-up to 5 V on OUT1 → V_out1 ≈ 5 V (same as disabled case).
 *
 * TB6612 FORWARD (STBY HIGH, PWMA HIGH, AIN1=1, AIN2=0 → AO1 HIGH, AO2 LOW):
 *   AO1: V_ao1 = 12 × R_load / (R_on + R_load) = 12 × 100 / 102 ≈ 11.7647 V
 *   AO2: V_ao2 = 12 × R_on  / (R_on + R_load) = 12 × 2  / 102 ≈  0.2353 V
 *   V_motor = V_ao1 − V_ao2 ≈ 11.5294 V (forward, positive)
 *
 * TB6612 REVERSE (AIN1=0, AIN2=1 → AO1 LOW, AO2 HIGH):
 *   V_motor = V_ao1 − V_ao2 ≈ −11.5294 V (reverse, negative)
 *
 * TB6612 BRAKE (AIN1=1, AIN2=1 → AO1 LOW, AO2 LOW):
 *   Both outputs LOW (stamp to GND). With no external path:
 *   V_ao1 ≈ 0 V, V_ao2 ≈ 0 V. Load resistor has GND on both ends → no current.
 *   V_motor = V_ao1 − V_ao2 = 0 V.
 *
 * TB6612 COAST via (0,0) (AIN1=0, AIN2=0 → both HIZ):
 *   With pull-ups to VM on both output nodes: V_ao1 ≈ VM, V_ao2 ≈ VM.
 *   Both pulled to 12 V by 10 kΩ pull-ups; no stamp → GMIN only.
 *   V_motor = V_ao1 − V_ao2 ≈ 0 V (both at VM, no differential).
 *
 * TB6612 STBY LOW → all HIZ:
 *   Standby: stby input LOW → committed state sets all outputs to HIZ.
 *   Same pull-up topology as coast: V_ao1 ≈ V_ao2 ≈ VM = 12 V.
 */

import { describe, expect, it } from "vitest";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import type { Circuit } from "../../../src/circuit/types.js";
import { breadboardToSimCircuit } from "../../helpers/circuit/breadboard.js";
import { buildNets } from "../../../src/sim/engine/graph.js";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";
import { runDiagnostics } from "../../helpers/diagnostics/index.js";

// ─── Helpers ─────────────────────────────────────────────────────────────────

function makeSimCircuit(
  components: SimCircuit["components"],
  wires: SimCircuit["wires"],
): SimCircuit {
  return { components, wires };
}

/** Voltage source with two pins: pos / neg. */
function vs(id: string, v: number): SimCircuit["components"][number] {
  return { id, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: v } };
}

/** Fixed resistor. */
function resistor(id: string, r: number): SimCircuit["components"][number] {
  return { id, kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: r } };
}

function inductor(id: string, inductance: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "inductor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { inductance },
  };
}

/** Wire helper for SimCircuit (no id or resistance field needed). */
function wire(fc: string, fp: string, tc: string, tp: string): SimCircuit["wires"][number] {
  return { from_component: fc, from_pin: fp, to_component: tc, to_pin: tp };
}

/**
 * L293D component factory for engine tests.
 * All 16 pins declared; unused pins are open (not wired).
 */
function l293d(id: string): SimCircuit["components"][number] {
  return {
    id,
    kind: "l293d",
    pins: [
      { id: "en12" }, { id: "in1" }, { id: "out1" }, { id: "gnd1" }, { id: "gnd2" },
      { id: "out2" }, { id: "in2" }, { id: "vcc2" },
      { id: "en34" }, { id: "in3" }, { id: "out3" }, { id: "gnd3" }, { id: "gnd4" },
      { id: "out4" }, { id: "in4" }, { id: "vcc1" },
    ],
    params: {},
  };
}

/**
 * TB6612 component factory for engine tests.
 * All 16 pins declared.
 */
function tb6612(id: string): SimCircuit["components"][number] {
  return {
    id,
    kind: "tb6612",
    pins: [
      { id: "bin1" }, { id: "bin2" }, { id: "pwmb" }, { id: "bo1" }, { id: "bo2" },
      { id: "gnd" },  { id: "vm" },  { id: "vcc" },
      { id: "ao1" },  { id: "ao2" }, { id: "ain1" }, { id: "ain2" },
      { id: "pwma" }, { id: "stby" }, { id: "gnd2" }, { id: "gnd3" },
    ],
    params: {},
  };
}

function netVoltage(engine: SimEngine, compId: string, pinId: string): number {
  const net = engine.nets.find((n) =>
    n.pins.some(([cid, pid]) => cid === compId && pid === pinId),
  );
  return net ? (engine.getNetV()[net.id] ?? 0) : 0;
}

function runSteps(circuit: SimCircuit, steps: number, dt = 50e-6): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

// ─── Diagnostic test helpers ─────────────────────────────────────────────────

const catalog = rawCatalog as unknown as PartCatalog;

function baseCircuit(): Circuit {
  return { id: "test", name: "test", schema_version: 2, components: [], wires: [], nets: [] };
}

function diagWire(
  fc: string, fp: string, tc: string, tp: string,
): Circuit["wires"][number] {
  return {
    id: `w-${fc}-${fp}-${tc}-${tp}`,
    from_component: fc, from_pin: fp,
    to_component: tc, to_pin: tp,
    resistance: 0,
  };
}

function diagBattery(id: string): Circuit["components"][number] {
  return {
    id, kind: "battery_pack",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [{ id: "pos", offset: { x: 0, y: 0 } }, { id: "neg", offset: { x: 0, y: 0 } }],
    params: {},
  };
}

function diagRelay(id: string): Circuit["components"][number] {
  return {
    id, kind: "relay",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [
      { id: "coil_a", offset: { x: 0, y: 0 } },
      { id: "coil_b", offset: { x: 0, y: 0 } },
      { id: "com",    offset: { x: 0, y: 0 } },
      { id: "no",     offset: { x: 0, y: 0 } },
      { id: "nc",     offset: { x: 0, y: 0 } },
    ],
    params: { coilR: 70, coilL: 0.05, vPull: 3.5, vDrop: 1.5 },
  };
}

function diagL293d(id: string): Circuit["components"][number] {
  return {
    id, kind: "l293d",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [
      { id: "en12",  offset: { x: 0, y: 0 } },
      { id: "in1",   offset: { x: 0, y: 0 } },
      { id: "out1",  offset: { x: 0, y: 0 } },
      { id: "gnd1",  offset: { x: 0, y: 0 } },
      { id: "gnd2",  offset: { x: 0, y: 0 } },
      { id: "out2",  offset: { x: 0, y: 0 } },
      { id: "in2",   offset: { x: 0, y: 0 } },
      { id: "vcc2",  offset: { x: 0, y: 0 } },
      { id: "en34",  offset: { x: 0, y: 0 } },
      { id: "in3",   offset: { x: 0, y: 0 } },
      { id: "out3",  offset: { x: 0, y: 0 } },
      { id: "gnd3",  offset: { x: 0, y: 0 } },
      { id: "gnd4",  offset: { x: 0, y: 0 } },
      { id: "out4",  offset: { x: 0, y: 0 } },
      { id: "in4",   offset: { x: 0, y: 0 } },
      { id: "vcc1",  offset: { x: 0, y: 0 } },
    ],
    params: {},
  };
}

function diagTb6612(id: string): Circuit["components"][number] {
  return {
    id, kind: "tb6612",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [
      { id: "bin1",  offset: { x: 0, y: 0 } },
      { id: "bin2",  offset: { x: 0, y: 0 } },
      { id: "pwmb",  offset: { x: 0, y: 0 } },
      { id: "bo1",   offset: { x: 0, y: 0 } },
      { id: "bo2",   offset: { x: 0, y: 0 } },
      { id: "gnd",   offset: { x: 0, y: 0 } },
      { id: "vm",    offset: { x: 0, y: 0 } },
      { id: "vcc",   offset: { x: 0, y: 0 } },
      { id: "ao1",   offset: { x: 0, y: 0 } },
      { id: "ao2",   offset: { x: 0, y: 0 } },
      { id: "ain1",  offset: { x: 0, y: 0 } },
      { id: "ain2",  offset: { x: 0, y: 0 } },
      { id: "pwma",  offset: { x: 0, y: 0 } },
      { id: "stby",  offset: { x: 0, y: 0 } },
      { id: "gnd2",  offset: { x: 0, y: 0 } },
      { id: "gnd3",  offset: { x: 0, y: 0 } },
    ],
    params: {},
  };
}

function makeDiagInput(c: Circuit) {
  const sim = breadboardToSimCircuit(c);
  const nets = buildNets(sim);
  return { circuit: sim, nets, catalog };
}

// ═══════════════════════════════════════════════════════════════════════════════
// PART A — L293D
// ═══════════════════════════════════════════════════════════════════════════════

describe("L293D — forward drive: EN12 HIGH, IN1 HIGH → OUT1 HIGH", () => {
  it("V_out1 = 12 × 100 / (100 + 2) = 11.7647 V with 100 Ω load to GND", () => {
    //
    // Derivation:
    //   OUT1 HIGH: stamp resistor(out1, vcc2, 2 Ω).
    //   vcc2 = 12 V (VM supply source).
    //   R_load = 100 Ω from out1 to GND.
    //   V_out1 = 12 × 100 / (100 + 2) = 1200 / 102 = 11.764705... V ≈ 11.7647 V
    //
    // Wires:
    //   vsVM (12 V): pos → l293d.vcc2, neg → GND
    //   vsVcc1 (5 V): pos → l293d.vcc1, neg → GND
    //   vsEn12 (5 V): pos → l293d.en12, neg → GND
    //   vsIn1  (5 V): pos → l293d.in1,  neg → GND
    //   rLoad (100 Ω): a → l293d.out1, b → GND
    //   l293d.gnd1 → GND
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc1 = vs("vsVcc1",  5);
    const vsEn   = vs("vsEn",    5);
    const vsIn1  = vs("vsIn1",   5);
    const rLoad  = resistor("rLoad", 100);
    const ic     = l293d("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc1, vsEn, vsIn1, rLoad, ic],
      [
        // VM rail (motor supply)
        wire("vsVM",   "pos", "ic1",   "vcc2"),
        wire("vsVM",   "neg", "ic1",   "gnd1"),
        // Logic supply
        wire("vsVcc1", "pos", "ic1",   "vcc1"),
        wire("vsVcc1", "neg", "vsVM",  "neg"),  // common GND rail
        // EN12 HIGH (enables channels 1+2)
        wire("vsEn",   "pos", "ic1",   "en12"),
        wire("vsEn",   "neg", "vsVM",  "neg"),
        // IN1 HIGH (drives OUT1 toward VM)
        wire("vsIn1",  "pos", "ic1",   "in1"),
        wire("vsIn1",  "neg", "vsVM",  "neg"),
        // Load: out1 → 100 Ω → GND
        wire("ic1",    "out1", "rLoad", "a"),
        wire("rLoad",  "b",   "vsVM",  "neg"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vOut1 = netVoltage(engine, "ic1", "out1");

    // Expected: 12 × 100 / 102 = 11.764705... V
    // Allow ±2% tolerance for Newton convergence.
    //   Lower: 11.7647 × 0.98 = 11.5294 V
    //   Upper: 11.7647 × 1.02 = 12.0000 V (capped at VM)
    expect(vOut1).toBeGreaterThan(11.52);
    expect(vOut1).toBeLessThan(12.01);
    expect(engine.lastConverged).toBe(true);
  });
});

describe("L293D — reverse drive: EN12 HIGH, IN1 LOW → OUT1 LOW", () => {
  it("V_out1 ≈ 0.2353 V with 100 Ω load to VM (12 V)", () => {
    //
    // Derivation:
    //   OUT1 LOW: stamp resistor(out1, gnd, 2 Ω).
    //   R_load = 100 Ω from VM (12 V) to out1.
    //   Circuit: VM → R_load → out1 → R_on → GND.
    //   V_out1 = VM × R_on / (R_load + R_on) = 12 × 2 / (100 + 2) = 24 / 102 = 0.23529... V
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc1 = vs("vsVcc1",  5);
    const vsEn   = vs("vsEn",    5);
    const vsIn1  = vs("vsIn1",   0);   // IN1 LOW → OUT1 LOW
    const rLoad  = resistor("rLoad", 100);
    const ic     = l293d("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc1, vsEn, vsIn1, rLoad, ic],
      [
        wire("vsVM",   "pos", "ic1",    "vcc2"),
        wire("vsVM",   "neg", "ic1",    "gnd1"),
        wire("vsVcc1", "pos", "ic1",    "vcc1"),
        wire("vsVcc1", "neg", "vsVM",   "neg"),
        wire("vsEn",   "pos", "ic1",    "en12"),
        wire("vsEn",   "neg", "vsVM",   "neg"),
        wire("vsIn1",  "pos", "ic1",    "in1"),
        wire("vsIn1",  "neg", "vsVM",   "neg"),
        // Load: VM → 100 Ω → out1 (low side)
        wire("vsVM",   "pos", "rLoad",  "a"),
        wire("rLoad",  "b",   "ic1",    "out1"),
        // gnd1 is GND (already tied to vsVM.neg above)
      ],
    );

    const engine = runSteps(circuit, 5);
    const vOut1 = netVoltage(engine, "ic1", "out1");

    // Expected: 12 × 2 / 102 = 0.23529 V
    // Allow ±5%: lower = 0.2235, upper = 0.2470
    expect(vOut1).toBeGreaterThan(0.22);
    expect(vOut1).toBeLessThan(0.25);
    expect(engine.lastConverged).toBe(true);
  });
});

describe("L293D — disabled: EN12 LOW → OUT1 HIZ", () => {
  it("OUT1 floats to pull-up voltage ≈ 5 V when EN12 = 0 V", () => {
    //
    // Derivation:
    //   EN12 LOW → all outputs HIZ (no stamp).
    //   External 10 kΩ pull-up from out1 to 5 V (logic rail).
    //   With no sink/source stamp on out1, only the numerical node reference remains.
    //   Current through GMIN = V × GMIN = 5 × 1e-9 = 5 nA → negligible voltage drop.
    //   V_out1 ≈ 5 V (pull-up dominates).
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc1 = vs("vsVcc1",  5);
    const vsEn   = vs("vsEn",    0);  // EN12 LOW → disabled
    const vsIn1  = vs("vsIn1",   5);  // IN1 value irrelevant when disabled
    const rPull  = resistor("rPull", 10000);  // 10 kΩ pull-up to 5 V
    const ic     = l293d("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc1, vsEn, vsIn1, rPull, ic],
      [
        wire("vsVM",   "pos", "ic1",    "vcc2"),
        wire("vsVM",   "neg", "ic1",    "gnd1"),
        wire("vsVcc1", "pos", "ic1",    "vcc1"),
        wire("vsVcc1", "neg", "vsVM",   "neg"),
        wire("vsEn",   "pos", "ic1",    "en12"),
        wire("vsEn",   "neg", "vsVM",   "neg"),
        wire("vsIn1",  "pos", "ic1",    "in1"),
        wire("vsIn1",  "neg", "vsVM",   "neg"),
        // Pull-up: 5 V → 10 kΩ → out1
        wire("vsVcc1", "pos", "rPull",  "a"),
        wire("rPull",  "b",   "ic1",    "out1"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vOut1 = netVoltage(engine, "ic1", "out1");

    // Expected: out1 floats to ~5 V (pull-up dominates; HIZ = no stamp).
    // Allow ±0.5 V: lower = 4.5 V, upper = 5.5 V.
    expect(vOut1).toBeGreaterThan(4.5);
    expect(vOut1).toBeLessThan(5.5);
    expect(engine.lastConverged).toBe(true);
  });
});

describe("L293D — motor direction reversal (signed assertion)", () => {
  it("forward (IN1=H, IN2=L): V_motor = V_out1 − V_out2 ≈ +11.5294 V", () => {
    //
    // Derivation:
    //   R_motor = 100 Ω across OUT1 (HIGH) and OUT2 (LOW).
    //   OUT1 HIGH: V_out1 = 12 × 100 / 102 ≈ 11.7647 V
    //   OUT2 LOW:  V_out2 = 12 × 2   / 102 ≈  0.2353 V
    //   V_motor = V_out1 − V_out2 = 11.7647 − 0.2353 = 11.5294 V (positive = forward)
    //   Exact: 12 × (100 − 2) / 102 = 12 × 98 / 102 = 1176 / 102 = 11.52941... V
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc1 = vs("vsVcc1",  5);
    const vsEn   = vs("vsEn",    5);
    const vsIn1  = vs("vsIn1",   5);   // IN1 HIGH
    const vsIn2  = vs("vsIn2",   0);   // IN2 LOW
    const rMot   = resistor("rMot", 100);
    const ic     = l293d("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc1, vsEn, vsIn1, vsIn2, rMot, ic],
      [
        wire("vsVM",   "pos", "ic1",   "vcc2"),
        wire("vsVM",   "neg", "ic1",   "gnd1"),
        wire("vsVcc1", "pos", "ic1",   "vcc1"),
        wire("vsVcc1", "neg", "vsVM",  "neg"),
        wire("vsEn",   "pos", "ic1",   "en12"),
        wire("vsEn",   "neg", "vsVM",  "neg"),
        wire("vsIn1",  "pos", "ic1",   "in1"),
        wire("vsIn1",  "neg", "vsVM",  "neg"),
        wire("vsIn2",  "pos", "ic1",   "in2"),
        wire("vsIn2",  "neg", "vsVM",  "neg"),
        // Motor load: out1 → 100 Ω → out2
        wire("ic1",    "out1", "rMot", "a"),
        wire("rMot",   "b",    "ic1",  "out2"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vOut1 = netVoltage(engine, "ic1", "out1");
    const vOut2 = netVoltage(engine, "ic1", "out2");
    const vMotor = vOut1 - vOut2;

    // Expected: 12 × 98 / 102 = 11.5294 V (positive, forward direction)
    expect(vMotor).toBeGreaterThan(11.0);
    expect(vMotor).toBeLessThan(12.0);
    // Must be strictly positive (forward direction)
    expect(vMotor).toBeGreaterThan(0);
    expect(engine.lastConverged).toBe(true);
  });

  it("reverse (IN1=L, IN2=H): V_motor = V_out1 − V_out2 ≈ −11.5294 V", () => {
    //
    // Derivation:
    //   Symmetry: swap IN1/IN2 → OUT1 LOW, OUT2 HIGH.
    //   V_out1 = 12 × 2   / 102 ≈  0.2353 V  (LOW)
    //   V_out2 = 12 × 100 / 102 ≈ 11.7647 V  (HIGH)
    //   V_motor = V_out1 − V_out2 = 0.2353 − 11.7647 = −11.5294 V (negative = reverse)
    //   Exact: 12 × (2 − 100) / 102 = 12 × (−98) / 102 = −11.5294 V
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc1 = vs("vsVcc1",  5);
    const vsEn   = vs("vsEn",    5);
    const vsIn1  = vs("vsIn1",   0);   // IN1 LOW
    const vsIn2  = vs("vsIn2",   5);   // IN2 HIGH
    const rMot   = resistor("rMot", 100);
    const ic     = l293d("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc1, vsEn, vsIn1, vsIn2, rMot, ic],
      [
        wire("vsVM",   "pos", "ic1",   "vcc2"),
        wire("vsVM",   "neg", "ic1",   "gnd1"),
        wire("vsVcc1", "pos", "ic1",   "vcc1"),
        wire("vsVcc1", "neg", "vsVM",  "neg"),
        wire("vsEn",   "pos", "ic1",   "en12"),
        wire("vsEn",   "neg", "vsVM",  "neg"),
        wire("vsIn1",  "pos", "ic1",   "in1"),
        wire("vsIn1",  "neg", "vsVM",  "neg"),
        wire("vsIn2",  "pos", "ic1",   "in2"),
        wire("vsIn2",  "neg", "vsVM",  "neg"),
        wire("ic1",    "out1", "rMot", "a"),
        wire("rMot",   "b",    "ic1",  "out2"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vOut1 = netVoltage(engine, "ic1", "out1");
    const vOut2 = netVoltage(engine, "ic1", "out2");
    const vMotor = vOut1 - vOut2;

    // Expected: −11.5294 V (negative, reverse direction)
    expect(vMotor).toBeLessThan(-11.0);
    expect(vMotor).toBeGreaterThan(-12.0);
    // Must be strictly negative (reverse direction) — proves direction reversal
    expect(vMotor).toBeLessThan(0);
    expect(engine.lastConverged).toBe(true);
  });
});

describe("L293D — VM unconnected → all outputs HIZ", () => {
  it("OUT1 floats to pull-up ≈ 5 V when VCC2 (VM) is disconnected", () => {
    //
    // Derivation:
    //   VM pin open → vmOpen = true → vmValid = false → all outputs HIZ (no stamp).
    //   Same result as EN12 LOW: pull-up dominates.
    //   V_out1 ≈ 5 V (10 kΩ pull-up to logic supply).
    //
    const vsVcc1 = vs("vsVcc1",  5);
    const vsEn   = vs("vsEn",    5);
    const vsIn1  = vs("vsIn1",   5);
    // No vsVM — VCC2 pin is unwired → VM floating → all outputs HIZ.
    const rPull  = resistor("rPull", 10000);
    const ic     = l293d("ic1");

    const circuit = makeSimCircuit(
      [vsVcc1, vsEn, vsIn1, rPull, ic],
      [
        // VCC2 NOT wired — motor rail absent.
        wire("vsVcc1", "pos", "ic1",    "vcc1"),
        wire("vsVcc1", "neg", "ic1",    "gnd1"),
        wire("vsEn",   "pos", "ic1",    "en12"),
        wire("vsEn",   "neg", "ic1",    "gnd1"),
        wire("vsIn1",  "pos", "ic1",    "in1"),
        wire("vsIn1",  "neg", "ic1",    "gnd1"),
        // Pull-up to 5 V on out1
        wire("vsVcc1", "pos", "rPull",  "a"),
        wire("rPull",  "b",   "ic1",    "out1"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vOut1 = netVoltage(engine, "ic1", "out1");

    // OUT1 HIZ with pull-up to 5 V → V_out1 ≈ 5 V.
    expect(vOut1).toBeGreaterThan(4.5);
    expect(vOut1).toBeLessThan(5.5);
    expect(engine.lastConverged).toBe(true);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// PART B — TB6612FNG truth table
// ═══════════════════════════════════════════════════════════════════════════════

describe("TB6612 — forward (STBY=H, PWMA=H, AIN1=H, AIN2=L)", () => {
  it("V_motor = V_ao1 − V_ao2 ≈ +11.5294 V (positive = forward)", () => {
    //
    // Derivation:
    //   AO1 HIGH: V_ao1 = 12 × 100 / (100 + 2) = 1200 / 102 ≈ 11.7647 V
    //   AO2 LOW:  V_ao2 = 12 ×   2 / (100 + 2) =   24 / 102 ≈  0.2353 V
    //   V_motor = V_ao1 − V_ao2 = 12 × 98 / 102 ≈ 11.5294 V
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc  = vs("vsVcc",   5);
    const vsStby = vs("vsStby",  5);
    const vsPwma = vs("vsPwma",  5);
    const vsAin1 = vs("vsAin1",  5);
    const vsAin2 = vs("vsAin2",  0);
    const rMot   = resistor("rMot", 100);
    const ic     = tb6612("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc, vsStby, vsPwma, vsAin1, vsAin2, rMot, ic],
      [
        wire("vsVM",   "pos", "ic1",    "vm"),
        wire("vsVM",   "neg", "ic1",    "gnd"),
        wire("vsVcc",  "pos", "ic1",    "vcc"),
        wire("vsVcc",  "neg", "vsVM",   "neg"),
        wire("vsStby", "pos", "ic1",    "stby"),
        wire("vsStby", "neg", "vsVM",   "neg"),
        wire("vsPwma", "pos", "ic1",    "pwma"),
        wire("vsPwma", "neg", "vsVM",   "neg"),
        wire("vsAin1", "pos", "ic1",    "ain1"),
        wire("vsAin1", "neg", "vsVM",   "neg"),
        wire("vsAin2", "pos", "ic1",    "ain2"),
        wire("vsAin2", "neg", "vsVM",   "neg"),
        // Motor load across AO1 and AO2
        wire("ic1",    "ao1",  "rMot",  "a"),
        wire("rMot",   "b",    "ic1",   "ao2"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vAo1   = netVoltage(engine, "ic1", "ao1");
    const vAo2   = netVoltage(engine, "ic1", "ao2");
    const vMotor = vAo1 - vAo2;

    // Expected: 12 × 98 / 102 ≈ 11.5294 V (positive forward)
    expect(vMotor).toBeGreaterThan(11.0);
    expect(vMotor).toBeLessThan(12.0);
    expect(vMotor).toBeGreaterThan(0);   // strictly positive = forward
    expect(engine.lastConverged).toBe(true);
  });
});

describe("TB6612 — reverse (STBY=H, PWMA=H, AIN1=L, AIN2=H)", () => {
  it("V_motor = V_ao1 − V_ao2 ≈ −11.5294 V (negative = reverse)", () => {
    //
    // Derivation (symmetry of forward):
    //   AO1 LOW:  V_ao1 = 12 × 2   / 102 ≈  0.2353 V
    //   AO2 HIGH: V_ao2 = 12 × 100 / 102 ≈ 11.7647 V
    //   V_motor = V_ao1 − V_ao2 = −11.5294 V (negative = reverse)
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc  = vs("vsVcc",   5);
    const vsStby = vs("vsStby",  5);
    const vsPwma = vs("vsPwma",  5);
    const vsAin1 = vs("vsAin1",  0);  // AIN1 LOW
    const vsAin2 = vs("vsAin2",  5);  // AIN2 HIGH
    const rMot   = resistor("rMot", 100);
    const ic     = tb6612("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc, vsStby, vsPwma, vsAin1, vsAin2, rMot, ic],
      [
        wire("vsVM",   "pos", "ic1",   "vm"),
        wire("vsVM",   "neg", "ic1",   "gnd"),
        wire("vsVcc",  "pos", "ic1",   "vcc"),
        wire("vsVcc",  "neg", "vsVM",  "neg"),
        wire("vsStby", "pos", "ic1",   "stby"),
        wire("vsStby", "neg", "vsVM",  "neg"),
        wire("vsPwma", "pos", "ic1",   "pwma"),
        wire("vsPwma", "neg", "vsVM",  "neg"),
        wire("vsAin1", "pos", "ic1",   "ain1"),
        wire("vsAin1", "neg", "vsVM",  "neg"),
        wire("vsAin2", "pos", "ic1",   "ain2"),
        wire("vsAin2", "neg", "vsVM",  "neg"),
        wire("ic1",    "ao1", "rMot",  "a"),
        wire("rMot",   "b",   "ic1",   "ao2"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vAo1   = netVoltage(engine, "ic1", "ao1");
    const vAo2   = netVoltage(engine, "ic1", "ao2");
    const vMotor = vAo1 - vAo2;

    // Expected: −11.5294 V (negative reverse)
    expect(vMotor).toBeLessThan(-11.0);
    expect(vMotor).toBeGreaterThan(-12.0);
    expect(vMotor).toBeLessThan(0);   // strictly negative = reverse
    expect(engine.lastConverged).toBe(true);
  });
});

describe("TB6612 — brake (STBY=H, PWMA=H, AIN1=H, AIN2=H)", () => {
  it("V_motor ≈ 0 V: both outputs LOW, no differential across load", () => {
    //
    // Derivation:
    //   AIN1=H, AIN2=H → brake → AO1 LOW, AO2 LOW.
    //   Both outputs stamped to GND through R_on.
    //   V_ao1 ≈ 0 V (R_on to GND), V_ao2 ≈ 0 V (R_on to GND).
    //   R_motor between ao1 and ao2: both ends at ~0 V → V_motor ≈ 0 V.
    //   Small deviation from 0 only due to the numerical node reference and slight R_on imbalance,
    //   but both sides are clamped to GND — within ±0.1 V of zero.
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc  = vs("vsVcc",   5);
    const vsStby = vs("vsStby",  5);
    const vsPwma = vs("vsPwma",  5);
    const vsAin1 = vs("vsAin1",  5);  // AIN1 HIGH
    const vsAin2 = vs("vsAin2",  5);  // AIN2 HIGH → brake
    const rMot   = resistor("rMot", 100);
    const ic     = tb6612("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc, vsStby, vsPwma, vsAin1, vsAin2, rMot, ic],
      [
        wire("vsVM",   "pos", "ic1",   "vm"),
        wire("vsVM",   "neg", "ic1",   "gnd"),
        wire("vsVcc",  "pos", "ic1",   "vcc"),
        wire("vsVcc",  "neg", "vsVM",  "neg"),
        wire("vsStby", "pos", "ic1",   "stby"),
        wire("vsStby", "neg", "vsVM",  "neg"),
        wire("vsPwma", "pos", "ic1",   "pwma"),
        wire("vsPwma", "neg", "vsVM",  "neg"),
        wire("vsAin1", "pos", "ic1",   "ain1"),
        wire("vsAin1", "neg", "vsVM",  "neg"),
        wire("vsAin2", "pos", "ic1",   "ain2"),
        wire("vsAin2", "neg", "vsVM",  "neg"),
        wire("ic1",    "ao1", "rMot",  "a"),
        wire("rMot",   "b",   "ic1",   "ao2"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vAo1   = netVoltage(engine, "ic1", "ao1");
    const vAo2   = netVoltage(engine, "ic1", "ao2");
    const vMotor = vAo1 - vAo2;

    // Both outputs LOW → V_motor ≈ 0 V (both clamped to GND through R_on).
    // Allow ±0.5 V for GMIN-level asymmetry.
    expect(vAo1).toBeGreaterThanOrEqual(-0.5);
    expect(vAo1).toBeLessThanOrEqual(1.0);
    expect(vAo2).toBeGreaterThanOrEqual(-0.5);
    expect(vAo2).toBeLessThanOrEqual(1.0);
    expect(Math.abs(vMotor)).toBeLessThan(0.5);
    expect(engine.lastConverged).toBe(true);
  });
});

describe("TB6612 — coast via (0,0) (STBY=H, PWMA=H, AIN1=L, AIN2=L)", () => {
  it("V_motor ≈ 0 V: both HIZ pulled to VM, no differential", () => {
    //
    // Derivation:
    //   AIN1=L, AIN2=L → coast → AO1 HIZ, AO2 HIZ.
    //   No stamp on either output. Pull-ups (10 kΩ) to VM (12 V) on each.
    //   V_ao1 ≈ VM ≈ 12 V, V_ao2 ≈ VM ≈ 12 V (pull-ups dominate).
    //   V_motor = V_ao1 − V_ao2 ≈ 0 V (no differential; both at VM).
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc  = vs("vsVcc",   5);
    const vsStby = vs("vsStby",  5);
    const vsPwma = vs("vsPwma",  5);
    const vsAin1 = vs("vsAin1",  0);  // AIN1 LOW
    const vsAin2 = vs("vsAin2",  0);  // AIN2 LOW → coast
    const rPull1 = resistor("rPull1", 10000);  // 10 kΩ pull-up on ao1
    const rPull2 = resistor("rPull2", 10000);  // 10 kΩ pull-up on ao2
    const ic     = tb6612("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc, vsStby, vsPwma, vsAin1, vsAin2, rPull1, rPull2, ic],
      [
        wire("vsVM",    "pos",  "ic1",    "vm"),
        wire("vsVM",    "neg",  "ic1",    "gnd"),
        wire("vsVcc",   "pos",  "ic1",    "vcc"),
        wire("vsVcc",   "neg",  "vsVM",   "neg"),
        wire("vsStby",  "pos",  "ic1",    "stby"),
        wire("vsStby",  "neg",  "vsVM",   "neg"),
        wire("vsPwma",  "pos",  "ic1",    "pwma"),
        wire("vsPwma",  "neg",  "vsVM",   "neg"),
        wire("vsAin1",  "pos",  "ic1",    "ain1"),
        wire("vsAin1",  "neg",  "vsVM",   "neg"),
        wire("vsAin2",  "pos",  "ic1",    "ain2"),
        wire("vsAin2",  "neg",  "vsVM",   "neg"),
        // Pull-ups to VM on ao1 and ao2
        wire("vsVM",    "pos",  "rPull1", "a"),
        wire("rPull1",  "b",    "ic1",    "ao1"),
        wire("vsVM",    "pos",  "rPull2", "a"),
        wire("rPull2",  "b",    "ic1",    "ao2"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vAo1   = netVoltage(engine, "ic1", "ao1");
    const vAo2   = netVoltage(engine, "ic1", "ao2");
    const vMotor = vAo1 - vAo2;

    // Both pulled to VM ≈ 12 V; no differential → V_motor ≈ 0.
    expect(vAo1).toBeGreaterThan(10.0);
    expect(vAo2).toBeGreaterThan(10.0);
    expect(Math.abs(vMotor)).toBeLessThan(1.0);
    expect(engine.lastConverged).toBe(true);
  });
});

describe("TB6612 — STBY LOW → all outputs HIZ", () => {
  it("V_motor ≈ 0 V when STBY = 0 V (both channels in standby)", () => {
    //
    // Derivation:
    //   STBY LOW → committed state sets ao1=HIZ, ao2=HIZ, bo1=HIZ, bo2=HIZ.
    //   No stamp on any output. Same pull-up topology as coast.
    //   V_ao1 ≈ V_ao2 ≈ VM (12 V). V_motor = V_ao1 − V_ao2 ≈ 0 V.
    //
    const vsVM   = vs("vsVM",   12);
    const vsVcc  = vs("vsVcc",   5);
    const vsStby = vs("vsStby",  0);  // STBY LOW → standby
    const vsPwma = vs("vsPwma",  5);
    const vsAin1 = vs("vsAin1",  5);  // input HIGH but ignored in standby
    const vsAin2 = vs("vsAin2",  0);
    const rPull1 = resistor("rPull1", 10000);
    const rPull2 = resistor("rPull2", 10000);
    const ic     = tb6612("ic1");

    const circuit = makeSimCircuit(
      [vsVM, vsVcc, vsStby, vsPwma, vsAin1, vsAin2, rPull1, rPull2, ic],
      [
        wire("vsVM",    "pos",  "ic1",    "vm"),
        wire("vsVM",    "neg",  "ic1",    "gnd"),
        wire("vsVcc",   "pos",  "ic1",    "vcc"),
        wire("vsVcc",   "neg",  "vsVM",   "neg"),
        wire("vsStby",  "pos",  "ic1",    "stby"),
        wire("vsStby",  "neg",  "vsVM",   "neg"),
        wire("vsPwma",  "pos",  "ic1",    "pwma"),
        wire("vsPwma",  "neg",  "vsVM",   "neg"),
        wire("vsAin1",  "pos",  "ic1",    "ain1"),
        wire("vsAin1",  "neg",  "vsVM",   "neg"),
        wire("vsAin2",  "pos",  "ic1",    "ain2"),
        wire("vsAin2",  "neg",  "vsVM",   "neg"),
        wire("vsVM",    "pos",  "rPull1", "a"),
        wire("rPull1",  "b",    "ic1",    "ao1"),
        wire("vsVM",    "pos",  "rPull2", "a"),
        wire("rPull2",  "b",    "ic1",    "ao2"),
      ],
    );

    const engine = runSteps(circuit, 5);
    const vAo1   = netVoltage(engine, "ic1", "ao1");
    const vAo2   = netVoltage(engine, "ic1", "ao2");
    const vMotor = vAo1 - vAo2;

    // Both HIZ (standby) + pull-ups → ~12 V each, no differential.
    expect(vAo1).toBeGreaterThan(10.0);
    expect(vAo2).toBeGreaterThan(10.0);
    expect(Math.abs(vMotor)).toBeLessThan(1.0);
    expect(engine.lastConverged).toBe(true);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// PART C — Electrical freewheel paths during inductive coast
// ═══════════════════════════════════════════════════════════════════════════════

describe("H-bridge integrated freewheel-diode transients", () => {
  it("L293D clamps both sides of a winding after EN12 is released", () => {
    const makeCircuit = (enableVolts: number): SimCircuit => makeSimCircuit(
      [
        vs("vm", 12), vs("logic", 5), vs("enable", enableVolts),
        inductor("coil", 0.01), l293d("driver"),
      ],
      [
        wire("vm", "pos", "driver", "vcc2"),
        wire("logic", "pos", "driver", "vcc1"),
        // Deliberately wire GND3 while the engine reads GND1: the package's
        // four ground pins are one physical node.
        wire("vm", "neg", "driver", "gnd3"),
        wire("logic", "neg", "vm", "neg"),
        wire("enable", "neg", "vm", "neg"),
        wire("enable", "pos", "driver", "en12"),
        wire("logic", "pos", "driver", "in1"),
        wire("vm", "neg", "driver", "in2"),
        wire("driver", "out1", "coil", "a"),
        wire("coil", "b", "driver", "out2"),
      ],
    );

    const engine = new SimEngine();
    engine.load(makeCircuit(5));
    for (let i = 0; i < 1_000; i++) engine.step(1e-6);
    expect(Math.abs(engine.getElementI().coil ?? 0)).toBeGreaterThan(0.5);

    engine.load(makeCircuit(0));
    engine.step(1e-6);

    const lowSide = netVoltage(engine, "driver", "out1");
    const highSide = netVoltage(engine, "driver", "out2");
    const vmVolts = netVoltage(engine, "driver", "vcc2");
    // Forward current continues OUT1→OUT2, so GND→OUT1 and OUT2→VM conduct.
    expect(lowSide).toBeLessThan(-0.5);
    expect(lowSide).toBeGreaterThan(-1.2);
    expect(highSide - vmVolts).toBeGreaterThan(0.5);
    expect(highSide - vmVolts).toBeLessThan(1.2);
    expect(engine.lastConverged).toBe(true);
  });

  it("TB6612 body diodes clamp both winding terminals during PWM coast", () => {
    const makeCircuit = (pwmVolts: number): SimCircuit => makeSimCircuit(
      [
        vs("vm", 12), vs("logic", 5), vs("pwm", pwmVolts),
        inductor("coil", 0.01), tb6612("driver"),
      ],
      [
        wire("vm", "pos", "driver", "vm"),
        wire("logic", "pos", "driver", "vcc"),
        // Exercise the module's internally common ground header pins.
        wire("vm", "neg", "driver", "gnd3"),
        wire("logic", "neg", "vm", "neg"),
        wire("pwm", "neg", "vm", "neg"),
        wire("logic", "pos", "driver", "stby"),
        wire("pwm", "pos", "driver", "pwma"),
        wire("logic", "pos", "driver", "ain1"),
        wire("vm", "neg", "driver", "ain2"),
        wire("driver", "ao1", "coil", "a"),
        wire("coil", "b", "driver", "ao2"),
      ],
    );

    const engine = new SimEngine();
    engine.load(makeCircuit(5));
    for (let i = 0; i < 1_000; i++) engine.step(1e-6);
    expect(Math.abs(engine.getElementI().coil ?? 0)).toBeGreaterThan(0.5);

    engine.load(makeCircuit(0));
    engine.step(1e-6);

    const lowSide = netVoltage(engine, "driver", "ao1");
    const highSide = netVoltage(engine, "driver", "ao2");
    const vmVolts = netVoltage(engine, "driver", "vm");
    expect(lowSide).toBeLessThan(-0.5);
    expect(lowSide).toBeGreaterThan(-1.2);
    expect(highSide - vmVolts).toBeGreaterThan(0.5);
    expect(highSide - vmVolts).toBeLessThan(1.2);
    expect(engine.lastConverged).toBe(true);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// PART D — H-bridge flyback diagnostic (Finding 8 extension)
// ═══════════════════════════════════════════════════════════════════════════════

describe("H-bridge flyback clamp — L293D", () => {
  it("does NOT fire when relay coil is across two L293D outputs AND VM is powered", () => {
    //
    // Topology:
    //   l293d.out1 → relay.coil_a    (one side of coil)
    //   l293d.out2 → relay.coil_b    (other side; H-bridge drives both)
    //   l293d.vcc2 → bat.pos          (VM powered)
    //   l293d.vcc1 → bat.pos          (logic supply)
    //   l293d.gnd1 → bat.neg
    //   l293d.en12 → bat.pos          (enable)
    //   l293d.in1  → bat.pos, in2 → bat.neg  (forward direction)
    //   bat: simple supply
    //
    // hasHBridgeClamp: both coil ends on l293d output pins, VM wired to supply.
    // Expected: missing-flyback NOT fired for relay r1.
    //
    const c: Circuit = {
      ...baseCircuit(),
      components: [
        diagBattery("bat"),
        diagRelay("r1"),
        diagL293d("ic1"),
      ],
      wires: [
        // L293D power
        diagWire("bat", "pos", "ic1", "vcc2"),   // VM powered from supply
        diagWire("bat", "pos", "ic1", "vcc1"),   // logic supply
        diagWire("bat", "neg", "ic1", "gnd1"),
        // Enable and inputs (direction irrelevant for diagnostic check)
        diagWire("bat", "pos", "ic1", "en12"),
        diagWire("bat", "pos", "ic1", "in1"),
        diagWire("bat", "neg", "ic1", "in2"),
        // Coil across OUT1 and OUT2 — both on H-bridge output pins
        diagWire("ic1", "out1", "r1", "coil_a"),
        diagWire("ic1", "out2", "r1", "coil_b"),
      ],
    };

    const input = makeDiagInput(c);
    const findings = runDiagnostics(input);
    const flybackFindings = findings.filter((f) => f.id === "missing-flyback");

    // Relay coil clamped by H-bridge internal diodes → no finding.
    expect(flybackFindings.every((f) => !f.componentIds.includes("r1"))).toBe(true);
  });

  it("fires when L293D VM is floating (no internal clamp path)", () => {
    //
    // Topology: same as above but VCC2 (VM) is not wired.
    // VM floating → hasHBridgeClamp returns false → missing-flyback fires.
    //
    const c: Circuit = {
      ...baseCircuit(),
      components: [
        diagBattery("bat"),
        diagRelay("r1"),
        diagL293d("ic1"),
      ],
      wires: [
        // VCC2 NOT wired — VM floating
        diagWire("bat", "pos", "ic1", "vcc1"),   // only logic supply
        diagWire("bat", "neg", "ic1", "gnd1"),
        diagWire("bat", "pos", "ic1", "en12"),
        diagWire("bat", "pos", "ic1", "in1"),
        diagWire("bat", "neg", "ic1", "in2"),
        // Coil still across outputs (but VM floating → clamp inactive)
        diagWire("ic1", "out1", "r1", "coil_a"),
        diagWire("ic1", "out2", "r1", "coil_b"),
      ],
    };

    const input = makeDiagInput(c);
    const findings = runDiagnostics(input);
    const flybackFindings = findings.filter((f) => f.id === "missing-flyback");

    // VM floating → clamp not active → missing-flyback must fire.
    expect(flybackFindings.some((f) => f.componentIds.includes("r1"))).toBe(true);
  });

  it("still fires for a transistor-switched relay (bare BJT, no H-bridge)", () => {
    //
    // Sanity check: the existing bare-transistor path must still fire.
    // Topology: battery → relay coil → BJT NPN collector; emitter → GND.
    // No flyback diode, no ULN clamp, no H-bridge clamp.
    //
    const bjt: Circuit["components"][number] = {
      id: "q1", kind: "bjt_npn",
      position: { x: 0, y: 0 }, rotation: 0,
      pins: [
        { id: "b", offset: { x: 0, y: 0 } },
        { id: "c", offset: { x: 0, y: 0 } },
        { id: "e", offset: { x: 0, y: 0 } },
      ],
      params: { beta: 100 },
    };

    const c: Circuit = {
      ...baseCircuit(),
      components: [diagBattery("bat"), diagRelay("r1"), bjt],
      wires: [
        // Coil: bat+ → coil_a → coil_b → BJT collector
        diagWire("bat", "pos", "r1",  "coil_a"),
        diagWire("r1",  "coil_b", "q1", "c"),
        // BJT: emitter → GND
        diagWire("q1",  "e",   "bat", "neg"),
        // Base: driven (value irrelevant for diagnostic)
        diagWire("bat", "pos", "q1",  "b"),
      ],
    };

    const input = makeDiagInput(c);
    const findings = runDiagnostics(input);
    const flybackFindings = findings.filter((f) => f.id === "missing-flyback");

    // No clamp of any kind → missing-flyback must fire.
    expect(flybackFindings.some((f) => f.componentIds.includes("r1"))).toBe(true);
  });
});
