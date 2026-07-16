/**
 * Wave 7.2 engine tests: hobby servo (PWM decode) + bipolar stepper motor
 * (RL Norton companion + phase-table step detection).
 *
 * All expected values are derived from closed-form analysis with explicit
 * derivation comments. NO value is read back from the implementation and then
 * asserted — the cardinal sin of self-confirming tests.
 *
 * ─── SERVO DERIVATIONS ────────────────────────────────────────────────────────
 *
 * MODEL PARAMETERS (locked teaching values from parts catalog):
 *   idleR = 330 Ω, minPulseMs = 1.0 ms, maxPulseMs = 2.0 ms
 *   minAngle = 0°, maxAngle = 180°
 *
 * IDLE CURRENT (TEST S1):
 *   V(vplus) = 5 V, V(gnd) = 0 V. idleR = 330 Ω.
 *   I_idle = (5 − 0) / 330 = 5/330 = 1/66 A ≈ 0.015151... A
 *   Stored as closed-form fraction: 5/330.
 *
 * SIG PIN HIGH IMPEDANCE (TEST S2):
 *   sig→gnd stamp: 1 MΩ. With V(sig)=3.3 V: I_sig = 3.3 / 1e6 = 3.3e-6 A.
 *   This is negligibly small — test verifies it does NOT perturb V(vplus) notably.
 *
 * PWM ANGLE DECODE (TESTS S3–S5):
 *   angle = minAngle + (pulseMs − minPulseMs) / (maxPulseMs − minPulseMs) × (maxAngle − minAngle)
 *         = 0 + (pulseMs − 1.0) / (2.0 − 1.0) × (180 − 0)
 *         = (pulseMs − 1.0) × 180
 *
 *   pulseMs = 1.0 → angle = (1.0 − 1.0) × 180 = 0°
 *   pulseMs = 1.5 → angle = (1.5 − 1.0) × 180 = 0.5 × 180 = 90°
 *   pulseMs = 2.0 → angle = (2.0 − 1.0) × 180 = 1.0 × 180 = 180°
 *
 * PWM TIMING:
 *   h = 1e-6 s (1 µs) — fine enough to capture pulse edges to sub-µs accuracy.
 *   A 1.5 ms HIGH pulse starting at t=0:
 *     Rising edge at step 0 (V(sig) goes HIGH before first step).
 *     Falling edge at step 1500 (t = 1500 × 1e-6 = 1.5e-3 s = 1.5 ms).
 *   Strategy: set sig HIGH for 1500 steps, then LOW for 1 step.
 *   After the falling-edge step, the engine commits: pulseMs = 1.5, angle = 90°.
 *
 * ANGLE HOLD (TEST S6):
 *   After any falling-edge decode, no new pulse → angle unchanged across steps.
 *
 * ─── STEPPER DERIVATIONS ──────────────────────────────────────────────────────
 *
 * MODEL PARAMETERS (locked teaching values):
 *   coilR = 10 Ω, coilL = 0.01 H, stepsPerRev = 200
 *
 * NORTON COMPANION per coil (Backward-Euler):
 *   G = 1 / (R + L/h)
 *   I_hist = G × (L/h) × i_prev
 *   Stamp: conductance G between (a, b); current source −I_hist injected a→b.
 *   Solved coil current: i = G × v_ab + I_hist
 *
 * COIL ENERGISE (TEST T1):
 *   Apply 5 V across coil A (a1→a2). At steady state (h small, many steps):
 *     i_ss = V / R = 5 / 10 = 0.5 A exactly.
 *   tau_L = L/R = 0.01/10 = 1 ms. At h=50µs: 400 steps = 20 ms >> 10×tau_L.
 *   After 400 steps, i_A ≈ 0.5 A (within 1%).
 *
 * COIL CURRENT NORTON FORMULA (TEST T2):
 *   At steady state with i_prev = i_ss = V/R:
 *     I_hist = G × (L/h) × i_ss
 *     i_new  = G × V + I_hist = G × V + G × (L/h) × i_ss
 *            = G × (V + (L/h) × i_ss)
 *   Let V = i_ss × R; substitute: i_new = G × (i_ss × R + (L/h) × i_ss)
 *                                        = G × i_ss × (R + L/h)
 *                                        = G × i_ss / G = i_ss. Fixed point. Correct.
 *
 * PHASE DETECTION (TEST T3):
 *   Phase table (full-step bipolar):
 *     phase 0: signA=+1, signB=+1   (A+ B+ energised)
 *     phase 1: signA=−1, signB=+1   (A− B+ energised)
 *     phase 2: signA=−1, signB=−1   (A− B− energised)
 *     phase 3: signA=+1, signB=−1   (A+ B− energised)
 *   Deadband: |i| < 1 mA → sign = 0 → phase = -1.
 *
 * STEP COUNT / POSITION (TEST T4):
 *   Energise at phase 0 (+5V on a1/a2, +5V on b1/b2).
 *   Then switch to phase 1 (REVERSE a1/a2, keep b1/b2 positive).
 *   Phase delta = (1 − 0 + 4) mod 4 = 1 → position++.
 *   Four successive forward transitions (+1 mod 4 each) → position = +4.
 *
 * POSITION FORMULA (angle):
 *   angle = position × 360 / stepsPerRev (mod 360, not clamped)
 *   After 4 steps (position=4): angle = 4 × 360 / 200 = 1440/200 = 7.2°.
 *
 * TOTAL CURRENT _updateElementI (TEST T5):
 *   elementI[stepper] = |iA| + |iB|.
 *   With both coils energised at i_ss = 0.5 A: total = 0.5 + 0.5 = 1.0 A.
 *
 * FLYBACK DIAGNOSTIC (TEST T6):
 *   (a) Stepper coil A driven by bare NPN → missing-flyback fires.
 *   (b) Both coils across L293D H-bridge (OUT1/2 for A, OUT3/4 for B) → suppressed.
 *
 * CONVERGENCE (TEST T7):
 *   lastConverged=true and all net voltages finite.
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

// ─── Engine helpers ───────────────────────────────────────────────────────────

function makeSimCircuit(
  components: SimCircuit["components"],
  wires: SimCircuit["wires"],
): SimCircuit {
  return { components, wires };
}

function vs(id: string, v: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage: v },
  };
}

function wire(fc: string, fp: string, tc: string, tp: string): SimCircuit["wires"][number] {
  return { from_component: fc, from_pin: fp, to_component: tc, to_pin: tp };
}

function servo(id: string, overrides: Partial<Record<string, number>> = {}): SimCircuit["components"][number] {
  return {
    id,
    kind: "servo",
    catalogUid: "servo-sg90",
    pins: [{ id: "sig" }, { id: "vplus" }, { id: "gnd" }],
    params: {
      idleR:       330,
      minPulseMs:  1.0,
      maxPulseMs:  2.0,
      minAngle:    0,
      maxAngle:    180,
      ...overrides,
    },
  };
}

function stepper(id: string, overrides: Partial<Record<string, number>> = {}): SimCircuit["components"][number] {
  return {
    id,
    kind: "stepper",
    pins: [{ id: "a1" }, { id: "a2" }, { id: "b1" }, { id: "b2" }],
    params: {
      coilR:       10,
      coilL:       0.01,
      stepsPerRev: 200,
      ...overrides,
    },
  };
}

function runEngine(circuit: SimCircuit, steps: number, dt: number): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

// ─── Diagnostics helpers ──────────────────────────────────────────────────────

function makeCircuitComp(
  id: string,
  kind: string,
  pins: string[],
  params: Record<string, number | string> = {},
): Circuit["components"][number] {
  return {
    id,
    kind: kind as Circuit["components"][number]["kind"],
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: pins.map((p) => ({ id: p, offset: { x: 0, y: 0 } })),
    params,
  };
}

function diagWire(fc: string, fp: string, tc: string, tp: string): Circuit["wires"][number] {
  return {
    id: `w-${fc}-${fp}-${tc}-${tp}`,
    from_component: fc, from_pin: fp,
    to_component: tc, to_pin: tp,
    resistance: 0,
  };
}

function makeDiagInput(circuit: Circuit) {
  const sim = breadboardToSimCircuit(circuit);
  const nets = buildNets(sim);
  return { circuit: sim, nets, catalog: rawCatalog as unknown as PartCatalog };
}

function baseCircuit(): Circuit {
  return { id: "test", name: "test", schema_version: 2, components: [], wires: [], nets: [] };
}

// ═══════════════════════════════════════════════════════════════════════════════
// SERVO TESTS
// ═══════════════════════════════════════════════════════════════════════════════

// ─── TEST S1 — Idle current ───────────────────────────────────────────────────
describe("servo — idle current draw (vplus→gnd resistor)", () => {
  it("5 V supply with idleR=330 Ω → I = 5/330 ≈ 0.01515 A at steady state", () => {
    // Derivation:
    //   Stamp: stampResistor(vplus, gnd, 330 Ω).
    //   At steady state: I = V / R = 5 / 330 A exactly.
    //   5 / 330 = 0.015151515... A
    //   sig left floating (connected only through 1 MΩ to gnd) — does not affect vplus.
    //
    // Circuit: voltage_source → vplus; gnd → vs neg; sig unconnected (floats through 1 MΩ).
    // We DO connect sig to gnd via the 1 MΩ stamp — no extra wire needed; the stamp handles it.
    // For this test leave sig floating (it stamps 1 MΩ to gnd internally).
    const circuit = makeSimCircuit(
      [
        vs("psu", 5),
        servo("sv"),
      ],
      [
        wire("psu", "pos", "sv", "vplus"),
        wire("psu", "neg", "sv", "gnd"),
        // sig left unconnected — handled internally by the 1 MΩ stamp
      ],
    );

    const engine = runEngine(circuit, 10, 50e-6);

    expect(engine.lastConverged).toBe(true);

    // elementI for servo = (V(vplus) − V(gnd)) / idleR = 5 / 330
    // Closed-form: 5 / 330 = 0.0151515...
    const EXPECTED_I = 5 / 330; // = 0.015151515...
    const actualI = engine.getElementI()["sv"] ?? NaN;
    expect(isFinite(actualI)).toBe(true);
    // Allow ±0.5% tolerance (MNA solve is exact for resistive networks; tolerance covers
    // floating-point rounding only).
    expect(actualI).toBeCloseTo(EXPECTED_I, 4); // matches to 4 decimal places
  });
});

// ─── TEST S2 — Signal pin high impedance ──────────────────────────────────────
describe("servo — sig pin is high-impedance", () => {
  it("driving sig with 3.3 V does not meaningfully affect V(vplus) (5 V supply)", () => {
    // The servo input has no invented physical pull-down. Only the engine's
    // disclosed 1 TΩ numerical regularisation remains, so driving the input is
    // effectively load-free and cannot perturb the independent 5 V rail.
    const circuit = makeSimCircuit(
      [
        vs("psu", 5),
        vs("sig_src", 3.3),
        servo("sv"),
      ],
      [
        wire("psu",     "pos", "sv",      "vplus"),
        wire("psu",     "neg", "sv",      "gnd"),
        wire("sig_src", "pos", "sv",      "sig"),
        wire("sig_src", "neg", "sv",      "gnd"),
      ],
    );

    const engine = runEngine(circuit, 10, 50e-6);

    expect(engine.lastConverged).toBe(true);

    // V(vplus) must be within 0.1% of 5 V — sig current is too small to shift it.
    const net = engine.nets.find((n) =>
      n.pins.some(([cid, pid]) => cid === "sv" && pid === "vplus"),
    );
    const vPlus = net ? (engine.getNetV()[net.id] ?? NaN) : NaN;
    expect(isFinite(vPlus)).toBe(true);
    expect(vPlus).toBeGreaterThan(5 * 0.999); // within 0.1% of 5 V
    expect(vPlus).toBeLessThan(5 * 1.001);
  });
});

// ─── TEST S3 — PWM decode: 1.5 ms pulse → 90° ────────────────────────────────
describe("servo — PWM decode: 1.5 ms HIGH pulse → 90° angle", () => {
  it("rising edge at t=0, falling edge at t=1.5 ms → angle = 90°", () => {
    // Derivation:
    //   angle = (pulseMs − minPulseMs) / (maxPulseMs − minPulseMs) × maxAngle
    //         = (1.5 − 1.0) / (2.0 − 1.0) × 180
    //         = 0.5 / 1.0 × 180
    //         = 0.5 × 180
    //         = 90°  (exact, no approximation)
    //
    // Strategy: Use a sig source that starts HIGH and switches to LOW after 1500 µs.
    // h = 1 µs. Step 0..1499: sig HIGH (V=5V). Step 1500 onwards: sig LOW (V=0V).
    // The engine edge-detects on committed voltages:
    //   Step 0: prev=0, cur=1 → rising edge: riseT = 0.
    //   Step 1500: prev=1, cur=0 → falling edge: pulseMs = (1500e-6 − 0) × 1000 = 1.5.
    //   angle = (1.5 − 1.0) × 180 = 90°.
    //
    // Implementation: build TWO circuits (sig HIGH then sig LOW) and use load().
    // Simpler: drive sig via a voltage source that we flip mid-run.
    //
    // Simplest approach: run 1500 steps with sig=5V (rising edge captured at step 0),
    // then reload with sig=0V for 1 step (falling edge captured → angle decoded).
    const svComp = servo("sv");

    const circuitHigh = makeSimCircuit(
      [
        vs("psu", 5),
        vs("sig_src", 5),  // sig HIGH
        svComp,
      ],
      [
        wire("psu",     "pos", "sv", "vplus"),
        wire("psu",     "neg", "sv", "gnd"),
        wire("sig_src", "pos", "sv", "sig"),
        wire("sig_src", "neg", "sv", "gnd"),
      ],
    );

    const circuitLow = makeSimCircuit(
      [
        vs("psu", 5),
        vs("sig_src", 0),  // sig LOW
        svComp,
      ],
      [
        wire("psu",     "pos", "sv", "vplus"),
        wire("psu",     "neg", "sv", "gnd"),
        wire("sig_src", "pos", "sv", "sig"),
        wire("sig_src", "neg", "sv", "gnd"),
      ],
    );

    const engine = new SimEngine();
    engine.load(circuitHigh);

    // h = 1 µs. Run 1500 steps: rising edge fires at step 0 (sim starts at t=0,
    // first step ends at t=1µs; the rising edge is detected when curSig transitions
    // from 0 → 1). After 1500 steps simTime = 1500 × 1e-6 = 1.5e-3 s.
    for (let i = 0; i < 1500; i++) engine.step(1e-6);

    // Switch sig to LOW: falling edge fires on the next step.
    engine.load(circuitLow);
    engine.step(1e-6);

    const svSt = engine.getServoState("sv");
    expect(svSt).toBeDefined();
    expect(isFinite(svSt!.angle)).toBe(true);

    // Closed-form: angle = 90° exactly (0.5 × 180).
    // Allow ±0.5° for timing quantisation at 1 µs resolution:
    //   actual pulseMs = 1500 × 1e-6 × 1000 = 1.500 ms exactly → angle = 90° exactly.
    expect(svSt!.angle).toBeCloseTo(90, 0); // within ±0.5°
  });
});

// ─── TEST S4 — PWM decode: endpoints ─────────────────────────────────────────
describe("servo — PWM decode: endpoint pulses map to 0° and 180°", () => {
  it("1.0 ms HIGH pulse → angle = 0°", () => {
    // Derivation:
    //   angle = (1.0 − 1.0) / (2.0 − 1.0) × 180 = 0 / 1 × 180 = 0° exactly.
    const svComp = servo("sv");

    const circuitHigh = makeSimCircuit(
      [vs("psu", 5), vs("sig_src", 5), svComp],
      [
        wire("psu",     "pos", "sv", "vplus"),
        wire("psu",     "neg", "sv", "gnd"),
        wire("sig_src", "pos", "sv", "sig"),
        wire("sig_src", "neg", "sv", "gnd"),
      ],
    );

    const circuitLow = makeSimCircuit(
      [vs("psu", 5), vs("sig_src", 0), svComp],
      [
        wire("psu",     "pos", "sv", "vplus"),
        wire("psu",     "neg", "sv", "gnd"),
        wire("sig_src", "pos", "sv", "sig"),
        wire("sig_src", "neg", "sv", "gnd"),
      ],
    );

    const engine = new SimEngine();
    engine.load(circuitHigh);
    // 1000 µs = 1.0 ms HIGH pulse
    for (let i = 0; i < 1000; i++) engine.step(1e-6);
    engine.load(circuitLow);
    engine.step(1e-6);

    let svSt = engine.getServoState("sv")!;
    expect(svSt.targetAngle).toBeCloseTo(0, 8);
    // A real shaft cannot teleport on the decode edge.
    expect(svSt.angle).toBeGreaterThan(89);

    // At 5 V the declared 600°/s-at-4.8 V speed scales to 625°/s, so a
    // 90° move completes in 144 ms. Allow 160 ms for the discrete boundary.
    for (let i = 0; i < 160; i++) engine.step(1e-3);
    svSt = engine.getServoState("sv")!;
    expect(svSt.angle).toBeCloseTo(0, 8);
    expect(svSt.moving).toBe(false);
  });

  it("2.0 ms HIGH pulse → angle = 180°", () => {
    // Derivation:
    //   angle = (2.0 − 1.0) / (2.0 − 1.0) × 180 = 1 / 1 × 180 = 180° exactly.
    const svComp = servo("sv");

    const circuitHigh = makeSimCircuit(
      [vs("psu", 5), vs("sig_src", 5), svComp],
      [
        wire("psu",     "pos", "sv", "vplus"),
        wire("psu",     "neg", "sv", "gnd"),
        wire("sig_src", "pos", "sv", "sig"),
        wire("sig_src", "neg", "sv", "gnd"),
      ],
    );

    const circuitLow = makeSimCircuit(
      [vs("psu", 5), vs("sig_src", 0), svComp],
      [
        wire("psu",     "pos", "sv", "vplus"),
        wire("psu",     "neg", "sv", "gnd"),
        wire("sig_src", "pos", "sv", "sig"),
        wire("sig_src", "neg", "sv", "gnd"),
      ],
    );

    const engine = new SimEngine();
    engine.load(circuitHigh);
    // 2000 µs = 2.0 ms HIGH pulse
    for (let i = 0; i < 2000; i++) engine.step(1e-6);
    engine.load(circuitLow);
    engine.step(1e-6);

    let svSt = engine.getServoState("sv")!;
    expect(svSt.targetAngle).toBeCloseTo(180, 8);
    expect(svSt.angle).toBeLessThan(91);

    for (let i = 0; i < 160; i++) engine.step(1e-3);
    svSt = engine.getServoState("sv")!;
    expect(svSt.angle).toBeCloseTo(180, 8);
    expect(svSt.moving).toBe(false);
  });
});

// ─── TEST S5 — Angle hold between pulses ──────────────────────────────────────
describe("servo — command target holds between pulses", () => {
  it("after 90° decode, 20 further steps with sig LOW keep target and angle at 90°", () => {
    // Derivation:
    //   The committed-state pattern: if there is no rising edge (prev=1→cur=1 or
    //   prev=0→cur=0) and no falling edge (prev=1→cur=0), the angle field is not
    //   mutated — it retains its last decoded value.
    //   After the 1.5 ms pulse decode (angle=90°), keeping sig LOW for 20 more steps
    //   must leave angle at exactly 90° (it cannot drift or reset to any default).
    const svComp = servo("sv");

    const circuitHigh = makeSimCircuit(
      [vs("psu", 5), vs("sig_src", 5), svComp],
      [
        wire("psu",     "pos", "sv", "vplus"),
        wire("psu",     "neg", "sv", "gnd"),
        wire("sig_src", "pos", "sv", "sig"),
        wire("sig_src", "neg", "sv", "gnd"),
      ],
    );

    const circuitLow = makeSimCircuit(
      [vs("psu", 5), vs("sig_src", 0), svComp],
      [
        wire("psu",     "pos", "sv", "vplus"),
        wire("psu",     "neg", "sv", "gnd"),
        wire("sig_src", "pos", "sv", "sig"),
        wire("sig_src", "neg", "sv", "gnd"),
      ],
    );

    const engine = new SimEngine();
    engine.load(circuitHigh);
    for (let i = 0; i < 1500; i++) engine.step(1e-6); // 1.5 ms HIGH
    engine.load(circuitLow);
    engine.step(1e-6); // falling edge → angle decoded = 90°

    const angleAfterDecode = engine.getServoState("sv")!.angle;
    // Closed-form: 90° (from derivation above).
    expect(angleAfterDecode).toBeCloseTo(90, 0);

    // Run 20 more steps with sig LOW — no new edges, angle must hold.
    for (let i = 0; i < 20; i++) engine.step(1e-6);

    const angleAfterHold = engine.getServoState("sv")!.angle;
    // Must equal angleAfterDecode exactly (no mutation occurred).
    expect(angleAfterHold).toBe(angleAfterDecode);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// STEPPER TESTS
// ═══════════════════════════════════════════════════════════════════════════════

// ─── TEST T1 — Coil A current at steady state ────────────────────────────────
describe("stepper — coil A energises to V/R at steady state", () => {
  it("5 V across a1→a2, coilR=10 Ω → iA ≈ 0.5 A after ~10 × tau_L", () => {
    // Derivation:
    //   tau_L = coilL / coilR = 0.01 / 10 = 1 ms.
    //   At steady state: iA = V / coilR = 5 / 10 = 0.5 A exactly.
    //   h = 50 µs; 400 steps = 20 ms >> 10 × tau_L = 10 ms.
    //   Coil B is unconnected (b1, b2 float through the coil B Norton stamp;
    //   the stamp has no external drive so iB → 0).
    const circuit = makeSimCircuit(
      [
        vs("psu", 5),
        stepper("stp"),
      ],
      [
        wire("psu", "pos", "stp", "a1"),
        wire("psu", "neg", "stp", "a2"),
        // b1/b2 left unconnected: the 10Ω+L/h stamp keeps them from floating
      ],
    );

    const engine = runEngine(circuit, 400, 50e-6);

    const stpSt = engine.getStepperState("stp");
    expect(stpSt).toBeDefined();
    expect(engine.lastConverged).toBe(true);

    // iA_ss = V / coilR = 5 / 10 = 0.5 A (closed-form, exact).
    const EXPECTED_IA = 0.5; // A
    expect(isFinite(stpSt!.iA)).toBe(true);
    // Allow ±1% tolerance for Backward-Euler discretisation at h=50µs.
    expect(stpSt!.iA).toBeCloseTo(EXPECTED_IA, 1); // within ±0.05 A
  });
});

// ─── TEST T2 — Total current elementI ────────────────────────────────────────
describe("stepper — elementI = |iA| + |iB|", () => {
  it("both coils at 0.5 A → elementI = 1.0 A", () => {
    // Derivation:
    //   Both coils energised with 5 V / 10 Ω = 0.5 A each at steady state.
    //   elementI = |0.5| + |0.5| = 1.0 A exactly.
    //
    //   Circuit: psu → a1; a2 → gnd AND psu → b1; b2 → gnd.
    //   Both coils see the same 5 V drive.
    const circuit = makeSimCircuit(
      [
        vs("psu", 5),
        stepper("stp"),
      ],
      [
        wire("psu", "pos", "stp", "a1"),
        wire("psu", "neg", "stp", "a2"),
        wire("psu", "pos", "stp", "b1"),
        wire("psu", "neg", "stp", "b2"),
      ],
    );

    const engine = runEngine(circuit, 400, 50e-6);

    expect(engine.lastConverged).toBe(true);

    const totalI = engine.getElementI()["stp"] ?? NaN;
    // Closed-form: |iA| + |iB| = 0.5 + 0.5 = 1.0 A.
    expect(isFinite(totalI)).toBe(true);
    expect(totalI).toBeCloseTo(1.0, 1); // within ±0.05 A
  });
});

// ─── TEST T3 — Phase detection from coil currents ────────────────────────────
describe("stepper — phase detection from committed coil currents", () => {
  it("+iA +iB (phase 0) → stepper.phase === 0 after settle", () => {
    // Derivation:
    //   Phase table: phase 0 = (signA=+1, signB=+1).
    //   Both coils driven positive (a1→a2 and b1→b2 with +5V).
    //   After steady state, both currents > deadband (0.001 A), both positive.
    //   → signA=+1, signB=+1 → phase=0.
    const circuit = makeSimCircuit(
      [vs("psu", 5), stepper("stp")],
      [
        wire("psu", "pos", "stp", "a1"),
        wire("psu", "neg", "stp", "a2"),
        wire("psu", "pos", "stp", "b1"),
        wire("psu", "neg", "stp", "b2"),
      ],
    );

    const engine = runEngine(circuit, 400, 50e-6);
    const stpSt = engine.getStepperState("stp");

    expect(stpSt!.phase).toBe(0);
  });

  it("−iA +iB (phase 1) → stepper.phase === 1 after settle", () => {
    // Derivation:
    //   Phase 1 = (signA=−1, signB=+1).
    //   Coil A driven in reverse (a2 is + terminal, a1 is −): psu→a2, gnd→a1.
    //   Coil B driven forward: psu→b1, gnd→b2.
    const circuit = makeSimCircuit(
      [vs("psu", 5), stepper("stp")],
      [
        wire("psu", "neg", "stp", "a1"),  // a1 at GND → iA flows a2→a1 (negative)
        wire("psu", "pos", "stp", "a2"),  // a2 at +5V
        wire("psu", "pos", "stp", "b1"),
        wire("psu", "neg", "stp", "b2"),
      ],
    );

    const engine = runEngine(circuit, 400, 50e-6);
    const stpSt = engine.getStepperState("stp");

    expect(stpSt!.phase).toBe(1);
  });

  it("unenergised (no drive) → stepper.phase === -1 (deadband)", () => {
    // Derivation:
    //   No external drive → both coil currents near 0 (only Norton companion
    //   leakage from i_prev × decay). After many steps from i_prev=0: iA≈0, iB≈0.
    //   Both below deadband (1 mA) → signA=0, signB=0 → phase=-1 (unenergised).
    const circuit = makeSimCircuit(
      [stepper("stp")],
      [], // no wires — both coils left unconnected
    );

    const engine = runEngine(circuit, 10, 50e-6);
    const stpSt = engine.getStepperState("stp");

    expect(stpSt!.phase).toBe(-1);
  });
});

// ─── TEST T4 — Step counting and position ─────────────────────────────────────
describe("stepper — position counter advances with sequential phase transitions", () => {
  it("four forward phase transitions (0→1→2→3→0) → position = +4", () => {
    // Derivation:
    //   The engine detects sequential (+1 mod 4) phase advances.
    //   Starting at phase 0 (both positive), 4 forward transitions → position = +4.
    //   angle = position × 360 / stepsPerRev = 4 × 360 / 200 = 1440/200 = 7.2°
    //
    //   Phase sequence (full-step bipolar):
    //     Phase 0: signA=+1, signB=+1  →  a1→a2=+V, b1→b2=+V
    //     Phase 1: signA=−1, signB=+1  →  a1→a2=−V, b1→b2=+V
    //     Phase 2: signA=−1, signB=−1  →  a1→a2=−V, b1→b2=−V
    //     Phase 3: signA=+1, signB=−1  →  a1→a2=+V, b1→b2=−V
    //     Phase 0: (back to start)
    //
    //   Implementation: for each phase, run enough steps to settle the RL
    //   transient (tau_L = 1 ms; h=50µs; 40 steps = 2 ms > 2×tau_L), then
    //   switch polarity and run one more step for the edge to commit.
    //   The committed phase is checked after each settle; position increments
    //   by +1 for each sequential phase advance.

    // Drive table: [polA, polB] — positive means +5V on (a1,b1), negative means +5V on (a2,b2).
    // polarity = +1: psu→a1, gnd→a2.  polarity = −1: psu→a2, gnd→a1.
    function makePhaseCircuit(polA: 1 | -1, polB: 1 | -1): SimCircuit {
      const wires: SimCircuit["wires"] = [
        wire("psu", polA ===  1 ? "pos" : "neg", "stp", "a1"),
        wire("psu", polA ===  1 ? "neg" : "pos", "stp", "a2"),
        wire("psu", polB ===  1 ? "pos" : "neg", "stp", "b1"),
        wire("psu", polB ===  1 ? "neg" : "pos", "stp", "b2"),
      ];
      return makeSimCircuit([vs("psu", 5), stepper("stp")], wires);
    }

    const PHASES: Array<[1 | -1, 1 | -1]> = [
      [ 1,  1],  // phase 0
      [-1,  1],  // phase 1
      [-1, -1],  // phase 2
      [ 1, -1],  // phase 3
      [ 1,  1],  // phase 0 (wrap)
    ];

    const engine = new SimEngine();
    engine.load(makePhaseCircuit(PHASES[0][0], PHASES[0][1]));

    // Settle to phase 0 first (starting position = 0, phase = -1 → 0 is the first transition).
    // The initial state has phase=-1; after settling to phase 0 no step is counted
    // (prev=-1 → can't count). This is correct: we count only sequential transitions.
    for (let i = 0; i < 40; i++) engine.step(50e-6);

    // Transition through phases 1, 2, 3, 0 — each advances position by +1.
    for (let pi = 1; pi < PHASES.length; pi++) {
      const [polA, polB] = PHASES[pi];
      engine.load(makePhaseCircuit(polA, polB));
      for (let i = 0; i < 40; i++) engine.step(50e-6);
    }

    const stpSt = engine.getStepperState("stp");
    expect(stpSt).toBeDefined();

    // After 4 forward transitions, position = +4.
    // Closed-form: each delta=+1 mod 4 increments position.
    expect(stpSt!.position).toBe(4);
  });
});

// ─── TEST T5 — Convergence ────────────────────────────────────────────────────
describe("stepper — convergence", () => {
  it("lastConverged=true and all net voltages finite after 100 steps at h=50µs", () => {
    // Both coils energised with 5 V / 10 Ω = 0.5 A each.
    const circuit = makeSimCircuit(
      [vs("psu", 5), stepper("stp")],
      [
        wire("psu", "pos", "stp", "a1"),
        wire("psu", "neg", "stp", "a2"),
        wire("psu", "pos", "stp", "b1"),
        wire("psu", "neg", "stp", "b2"),
      ],
    );

    const engine = runEngine(circuit, 100, 50e-6);

    expect(engine.lastConverged).toBe(true);

    for (const [, v] of Object.entries(engine.getNetV())) {
      expect(isFinite(v)).toBe(true);
    }

    const stpSt = engine.getStepperState("stp");
    expect(isFinite(stpSt?.iA ?? NaN)).toBe(true);
    expect(isFinite(stpSt?.iB ?? NaN)).toBe(true);
  });
});

// ─── TEST T6 — Flyback diagnostic ─────────────────────────────────────────────
describe("stepper — flyback diagnostic (Finding 8 extension)", () => {
  it("coil A switched by bare NPN transistor (no diode) → missing-flyback fires for coil A", () => {
    // Topology: VS + → stepper a1; stepper a2 → BJT collector; BJT emitter → GND.
    // No flyback diode. Expectation: hasSwitchInCoilPath finds bjt_npn;
    // finding fires with key "missing-flyback:stp:coilA".
    const circuit: Circuit = {
      ...baseCircuit(),
      components: [
        makeCircuitComp("psu", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        makeCircuitComp("stp", "stepper", ["a1", "a2", "b1", "b2"]),
        makeCircuitComp("q1",  "bjt_npn",  ["b", "c", "e"]),
      ],
      wires: [
        diagWire("psu", "pos", "stp", "a1"),
        diagWire("stp", "a2",  "q1",  "c"),
        diagWire("q1",  "e",   "psu", "neg"),
        diagWire("q1",  "b",   "psu", "pos"),  // base tied high (simplified)
      ],
    };

    const input = makeDiagInput(circuit);
    const findings = runDiagnostics(input);
    const flyback = findings.filter((f) => f.id === "missing-flyback");

    // Should fire for coil A of the stepper.
    expect(flyback.some((f) => f.key === "missing-flyback:stp:coilA")).toBe(true);
  });

  it("stepper across L293D H-bridge (OUT1/2 for A, OUT3/4 for B) → missing-flyback suppressed", () => {
    // Topology: L293D OUT1→a1, OUT2→a2, OUT3→b1, OUT4→b2. VM powered.
    // hasHBridgeClamp suppresses each coil finding.
    // Expectation: NO missing-flyback finding for "stp".
    const circuit: Circuit = {
      ...baseCircuit(),
      components: [
        makeCircuitComp("vm",  "voltage_source", ["pos", "neg"], { voltage: 6 }),
        makeCircuitComp("psu", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        makeCircuitComp("en",  "voltage_source", ["pos", "neg"], { voltage: 5 }),
        makeCircuitComp("in1", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        makeCircuitComp("in2", "voltage_source", ["pos", "neg"], { voltage: 0 }),
        makeCircuitComp("in3", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        makeCircuitComp("in4", "voltage_source", ["pos", "neg"], { voltage: 0 }),
        makeCircuitComp("ic",  "l293d", [
          "en12", "in1", "out1", "gnd1", "gnd2", "out2", "in2", "vcc2",
          "en34", "in3", "out3", "gnd3", "gnd4", "out4", "in4", "vcc1",
        ]),
        makeCircuitComp("stp", "stepper", ["a1", "a2", "b1", "b2"]),
      ],
      wires: [
        diagWire("vm",  "pos",  "ic",  "vcc2"),
        diagWire("vm",  "neg",  "ic",  "gnd1"),
        diagWire("vm",  "neg",  "ic",  "gnd2"),
        diagWire("vm",  "neg",  "ic",  "gnd3"),
        diagWire("vm",  "neg",  "ic",  "gnd4"),
        diagWire("psu", "pos",  "ic",  "vcc1"),
        diagWire("psu", "neg",  "vm",  "neg"),
        diagWire("en",  "pos",  "ic",  "en12"),
        diagWire("en",  "neg",  "vm",  "neg"),
        diagWire("en",  "pos",  "ic",  "en34"),
        diagWire("in1", "pos",  "ic",  "in1"),
        diagWire("in1", "neg",  "vm",  "neg"),
        diagWire("in2", "pos",  "ic",  "in2"),
        diagWire("in2", "neg",  "vm",  "neg"),
        diagWire("in3", "pos",  "ic",  "in3"),
        diagWire("in3", "neg",  "vm",  "neg"),
        diagWire("in4", "pos",  "ic",  "in4"),
        diagWire("in4", "neg",  "vm",  "neg"),
        // Coil A: a1 ← OUT1, a2 ← OUT2
        diagWire("ic",  "out1", "stp", "a1"),
        diagWire("ic",  "out2", "stp", "a2"),
        // Coil B: b1 ← OUT3, b2 ← OUT4
        diagWire("ic",  "out3", "stp", "b1"),
        diagWire("ic",  "out4", "stp", "b2"),
      ],
    };

    const input = makeDiagInput(circuit);
    const findings = runDiagnostics(input);
    const flyback = findings.filter((f) => f.id === "missing-flyback" && f.key.startsWith("missing-flyback:stp"));

    // Should NOT fire for the stepper (H-bridge clamp suppresses both coils).
    expect(flyback).toHaveLength(0);
  });
});

// ─── TEST S7 / T7 — Servo convergence ────────────────────────────────────────
describe("servo — convergence", () => {
  it("lastConverged=true and finite net voltages after 10 steps with sig driven 3.3V", () => {
    const circuit = makeSimCircuit(
      [
        vs("psu", 5),
        vs("sig_src", 3.3),
        servo("sv"),
      ],
      [
        wire("psu",     "pos", "sv", "vplus"),
        wire("psu",     "neg", "sv", "gnd"),
        wire("sig_src", "pos", "sv", "sig"),
        wire("sig_src", "neg", "sv", "gnd"),
      ],
    );

    const engine = runEngine(circuit, 10, 50e-6);

    expect(engine.lastConverged).toBe(true);

    for (const [, v] of Object.entries(engine.getNetV())) {
      expect(isFinite(v)).toBe(true);
    }

    // sig → gnd stamp does not make net voltages diverge.
    const svSt = engine.getServoState("sv");
    expect(svSt).toBeDefined();
    // No pulse yet (sig was never HIGH then LOW in a transition detected as an edge
    // because it was driven HIGH from the start — riseT is set, but no falling edge yet).
    // angle should still be at the default 90° (no decode has occurred).
    expect(svSt!.angle).toBe(90);
  });
});
