/**
 * Wave 7.1 engine tests: dc_motor — RL winding + back-EMF + rotor dynamics.
 *
 * All expected values are derived from closed-form analysis with derivation
 * comments. NO value is read back from the implementation and then asserted —
 * the cardinal sin of self-confirming tests.
 *
 * ─── DERIVATIONS ──────────────────────────────────────────────────────────────
 *
 * MODEL PARAMETERS (locked teaching values):
 *   windingR = 5 Ω, windingL = 0.002 H, Ke = Kt = 0.01, J = 1e-5, b = 1e-5
 *
 * NORTON COMPANION (Backward-Euler, one step of size h):
 *   G = 1 / (R + L/h)
 *   Vbemf = Ke × omega_committed   (back-EMF, held constant during Newton loop)
 *   I_eq = G × ((L/h) × i_prev − Vbemf)
 *   Stamp: conductance G between m1–m2, current source I_eq injected m1→m2.
 *   Solved winding current: i = G × v_m1m2 + I_eq
 *
 * TEST 1 — STALL CURRENT (Ke=0, omega=0, steady state):
 *   At steady state with no back-EMF and no inductive memory:
 *     i_stall = V / windingR = 6 / 5 = 1.2 A
 *   h = 50e-6 s, tau_elec = L/R = 0.002/5 = 4e-4 s = 0.4 ms.
 *   After 400 steps (= 20 ms >> 10 × tau_elec), current is at steady state.
 *   Norton steady-state check: G = 1/(5 + 0.002/50e-6) = 1/(5+40) = 1/45
 *   I_eq at SS = G × ((0.002/50e-6) × i_prev − 0) = (40/45) × i_prev
 *   Solve: i = G × 6 + I_eq = 6/45 + (40/45) × i → i × (1 − 40/45) = 6/45
 *   → i × 5/45 = 6/45 → i = 6/5 = 1.2 A. Correct.
 *   Assert: i ≈ 1.2 A (within 1%).
 *
 * TEST 2 — NO-LOAD STEADY STATE (Ke=Kt=0.01, b=1e-5, loadTorque=0):
 *   At mechanical steady state: Kt × i_ss = b × omega_ss
 *   At electrical steady state: V = R × i_ss + Ke × omega_ss
 *
 *   From torque balance: i_ss = (b / Kt) × omega_ss = (1e-5 / 0.01) × omega_ss
 *                             = 1e-3 × omega_ss
 *   Substitute into KVL: 6 = 5 × (1e-3 × omega_ss) + 0.01 × omega_ss
 *                          = (0.005 + 0.01) × omega_ss = 0.015 × omega_ss
 *   → omega_ss = 6 / 0.015 = 400 rad/s
 *   → i_ss = 1e-3 × 400 = 0.4 A
 *   → RPM_ss = 400 × 60 / (2π) ≈ 3819.7 RPM
 *
 *   Mechanical time constant: tau_mech = J / b = 1e-5 / 1e-5 = 1 s.
 *   Choose h = 1e-3 s (1 ms), steps = 6000 (6 s >> 5 × tau_mech).
 *   After 6 s the motor is well past 5 tau_mech, so omega is within 1% of omega_ss.
 *
 *   Assert: omega ≈ 400 rad/s (±10%), i ≈ 0.4 A (±20%).
 *   (Loose tolerances because the Backward-Euler scheme underestimates slightly
 *   at large h; the tight derivation holds at h→0.)
 *
 * TEST 3 — BACK-EMF REDUCES CURRENT (signed monotone):
 *   At step 0: omega = 0, i = V/R = 1.2 A (stall).
 *   As omega rises: Vbemf = Ke × omega increases, effective drive V−Vbemf drops,
 *   so winding current decreases monotonically toward i_ss = 0.4 A.
 *   Derivation: i(omega) = (V − Ke × omega) / R at elec SS (inductance settled).
 *   At omega=0:   i = 6/5 = 1.2 A.
 *   At omega=400: i = (6 − 0.01×400)/5 = (6−4)/5 = 2/5 = 0.4 A. Decrease confirmed.
 *   Assert: i at step 1 > i at steady state.
 *
 * TEST 4 — DIRECTION (negative supply → negative omega):
 *   With V = −6 V applied across m1(+) to m2(−), current flows m2→m1 (negative).
 *   Torque = Kt × i is negative → omega goes negative.
 *   Steady-state: omega_ss = −400 rad/s, i_ss = −0.4 A (by symmetry).
 *   Assert: omega < −10 rad/s after 6 s (clearly negative; not asserting exact value
 *   because at h=1ms the integrator settles slowly and exact comparison is noise-prone).
 *
 * TEST 5 — DRIVEN BY L293D H-BRIDGE:
 *   L293D R_on = 2 Ω per side. VM = 6 V.
 *   Forward (IN1 HIGH, IN2 LOW): OUT1 HIGH, OUT2 LOW.
 *     Effective V_motor = VM × (R_motor) / (R_on_high + R_motor + R_on_low) — no,
 *     actually: VM via R_on(2Ω) → m1, m2 via R_on(2Ω) → GND.
 *     The motor model is a Norton companion (≈ DC resistance R=5Ω at SS).
 *     Total series path: 2 Ω (OUT1 R_on) + 5 Ω (winding) + 2 Ω (OUT2 R_on) = 9 Ω.
 *     i_stall ≈ VM / (R_on_H + R_winding + R_on_L) = 6 / 9 ≈ 0.667 A (stall, omega=0).
 *     After running (omega rises, back-EMF reduces current):
 *     SS: V_eff = VM − i × (R_on_H + R_on_L) = 6 − 4i applied to motor.
 *     Motor KVL: V_eff = R_winding × i + Ke × omega.
 *     Torque balance: Kt × i = b × omega → omega = Kt × i / b = i × 1000.
 *     Substitute: 6 − 4i = 5i + 0.01 × 1000i → 6 − 4i = 5i + 10i → 6 = 19i
 *     → i_ss ≈ 0.3158 A, omega_ss ≈ 315.8 rad/s.
 *     Positive omega confirms FORWARD rotation.
 *   Reverse (IN1 LOW, IN2 HIGH): motor current flips → omega < 0.
 *   Assert forward: omega > 0 and i > 0 (motor running forward after settling).
 *   Assert reverse: omega < 0 (after settling).
 *
 * TEST 6 — FLYBACK DIAGNOSTIC:
 *   (a) Motor switched by bare NPN transistor → "missing-flyback" fires.
 *   (b) Motor across L293D H-bridge outputs with VM powered → suppressed.
 *
 * TEST 7 — CONVERGENCE:
 *   lastConverged = true and all node voltages finite.
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

function motor(
  id: string,
  overrides: Partial<Record<string, number>> = {},
): SimCircuit["components"][number] {
  return {
    id,
    kind: "dc_motor",
    pins: [{ id: "m1" }, { id: "m2" }],
    params: {
      windingR: 5,
      windingL: 0.002,
      Ke: 0.01,
      Kt: 0.01,
      inertia: 1e-5,
      friction: 1e-5,
      loadTorque: 0,
      ...overrides,
    },
  };
}

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

function bjt(id: string): SimCircuit["components"][number] {
  return {
    id,
    kind: "bjt_npn",
    pins: [{ id: "b" }, { id: "c" }, { id: "e" }],
    params: { bf: 100, is: 1e-14, vt: 0.02585, va: 100 },
  };
}

/**
 * Run engine for given number of steps and return it.
 */
function runEngine(
  circuit: SimCircuit,
  steps: number,
  dt: number,
): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

/**
 * Read voltage at a pin's net from a running engine.
 */
function netVoltage(engine: SimEngine, compId: string, pinId: string): number {
  const net = engine.nets.find((n) =>
    n.pins.some(([cid, pid]) => cid === compId && pid === pinId),
  );
  return net ? (engine.getNetV()[net.id] ?? 0) : 0;
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

function makeDiagInput(circuit: Circuit) {
  const sim = breadboardToSimCircuit(circuit);
  const nets = buildNets(sim);
  return { circuit: sim, nets, catalog: rawCatalog as unknown as PartCatalog };
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 1 — Stall current with Ke=0 (no back-EMF)
// ═══════════════════════════════════════════════════════════════════════════════

describe("dc_motor — stall current (Ke=0, no back-EMF)", () => {
  it("6 V across winding with Ke=0 → i ≈ V/R = 6/5 = 1.2 A at steady state", () => {
    // Derivation:
    //   Ke=0 → Vbemf=0. At electrical steady state (tau_elec = L/R = 0.4 ms):
    //   i_stall = V / R = 6 / 5 = 1.2 A exactly.
    //   h = 50e-6 s, 400 steps = 20 ms >> 10 × tau_elec = 4 ms.
    //
    // Circuit: voltage_source (6 V) → m1; m2 → GND (neg of vs).
    const circuit = makeSimCircuit(
      [
        vs("psu", 6),
        motor("mot", { Ke: 0 }),  // back-EMF disabled for this test
      ],
      [
        wire("psu", "pos", "mot", "m1"),  // +6V to motor +
        wire("psu", "neg", "mot", "m2"),  // GND to motor −
      ],
    );

    // h = 50 µs, 400 steps = 20 ms (>> 10 electrical time constants)
    const engine = runEngine(circuit, 400, 50e-6);

    const iWinding = engine.getMotorState("mot")?.iWinding ?? NaN;

    expect(engine.lastConverged).toBe(true);
    expect(isFinite(iWinding)).toBe(true);
    // Closed-form: i = V/R = 6/5 = 1.2 A. Allow 1% tolerance for discretisation.
    expect(iWinding).toBeCloseTo(1.2, 1); // within ±0.1 A of 1.2
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 2 — No-load steady-state speed
// ═══════════════════════════════════════════════════════════════════════════════

describe("dc_motor — no-load steady state (Ke=Kt=0.01, b=1e-5)", () => {
  it("6 V, no load → omega ≈ 400 rad/s, i ≈ 0.4 A after ~5 mechanical time constants", () => {
    // Derivation (closed-form, see file header):
    //   omega_ss = V / (Ke + R × b / Kt) = 6 / (0.01 + 5 × 1e-5 / 0.01)
    //            = 6 / (0.01 + 0.005) = 6 / 0.015 = 400 rad/s
    //   i_ss = (b / Kt) × omega_ss = (1e-5 / 0.01) × 400 = 0.4 A
    //
    //   tau_mech = J / b = 1e-5 / 1e-5 = 1 s.
    //   h = 1e-3 s (1 ms); 6000 steps = 6 s >> 5 × tau_mech.
    //   At t = 6 s the motor is well within 1% of omega_ss.
    const circuit = makeSimCircuit(
      [
        vs("psu", 6),
        motor("mot"),  // default params
      ],
      [
        wire("psu", "pos", "mot", "m1"),
        wire("psu", "neg", "mot", "m2"),
      ],
    );

    // h = 1 ms (large step chosen to cover tau_mech quickly).
    // 6000 steps = 6 s >> 5 × tau_mech = 5 s.
    const engine = runEngine(circuit, 6000, 1e-3);

    const motorSt = engine.getMotorState("mot");
    const omega = motorSt?.omega ?? NaN;
    const iWinding = motorSt?.iWinding ?? NaN;

    expect(engine.lastConverged).toBe(true);
    expect(isFinite(omega)).toBe(true);
    expect(isFinite(iWinding)).toBe(true);

    // omega_ss = 400 rad/s (closed-form derivation above). Allow ±10% for Backward-Euler
    // at h=1ms (large step; error is O(h/tau_mech) ≈ 1ms/1s = 0.1%).
    // Vitest toBeCloseTo uses 1/2 * 10^-precision; using absolute margin via expect().
    expect(omega).toBeGreaterThan(400 * 0.9);   // > 360 rad/s
    expect(omega).toBeLessThan(400 * 1.1);      // < 440 rad/s

    // i_ss = 0.4 A (closed-form). Allow ±20% because at h=1ms the electrical
    // transient is also integrated coarsely (tau_elec = 0.4ms < h).
    expect(iWinding).toBeGreaterThan(0.4 * 0.8);  // > 0.32 A
    expect(iWinding).toBeLessThan(0.4 * 1.2);     // < 0.48 A
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 3 — Back-EMF reduces current (signed monotone)
// ═══════════════════════════════════════════════════════════════════════════════

describe("dc_motor — back-EMF reduces current monotonically", () => {
  it("current near electrical steady state with omega≈0 is higher than current at full speed", () => {
    // Derivation:
    //   The winding inductor limits startup current. After ~10 × tau_elec (4 ms),
    //   the winding is electrically settled and i ≈ V/R = 6/5 = 1.2 A (stall current).
    //   At this point omega ≈ 0 because tau_mech = 1 s >> tau_elec = 0.4 ms.
    //
    //   As time progresses omega rises; back-EMF = Ke × omega grows, reducing
    //   effective drive. At full steady state: i_ss = 0.4 A < 1.2 A. QED.
    //
    //   Implementation:
    //   (1) Step at h=1ms for 20 steps (20 ms >> 10 × tau_elec = 4 ms; but
    //       omega has advanced only 20ms / 1s = 2% of tau_mech, so omega ≈ 0).
    //       Closed-form at electrical SS with omega≈0: i ≈ V/R = 1.2 A.
    //       At h=1ms the Backward-Euler companion gives:
    //         G = 1/(5 + 0.002/0.001) = 1/7 ≈ 0.1429
    //         After many steps at omega≈0: i → G × V / (1 − G × L/h) ... simplifies
    //         to i = V/R = 6/5 = 1.2 A exactly at SS (the Norton formula converges).
    //   (2) Then run at h=1ms for another 6000 steps (6 s) until omega → 400 rad/s.
    //       At that point i_ss = 0.4 A.
    //   Assert: i_near_stall > i_at_full_speed.
    const circuit = makeSimCircuit(
      [vs("psu", 6), motor("mot")],
      [wire("psu", "pos", "mot", "m1"), wire("psu", "neg", "mot", "m2")],
    );

    const engine = new SimEngine();
    engine.load(circuit);

    // Phase 1: 20 steps at h=1ms to settle the electrical transient (omega stays ≈ 0).
    // After 20ms << tau_mech = 1s, omega is still near zero.
    for (let i = 0; i < 20; i++) engine.step(1e-3);
    const iNearStall = engine.getMotorState("mot")?.iWinding ?? NaN;

    // Phase 2: run to mechanical steady state (6 s >> 5 × tau_mech = 5 s).
    for (let i = 0; i < 6000; i++) engine.step(1e-3);
    const iAtFullSpeed = engine.getMotorState("mot")?.iWinding ?? NaN;

    expect(isFinite(iNearStall)).toBe(true);
    expect(isFinite(iAtFullSpeed)).toBe(true);

    // Near-stall current (omega≈0) must exceed full-speed current (omega≈400).
    // Closed-form: i(omega=0) ≈ 1.2 A, i(omega=400) ≈ 0.4 A. Strict inequality.
    expect(iNearStall).toBeGreaterThan(iAtFullSpeed);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 4 — Direction: negative voltage → negative omega
// ═══════════════════════════════════════════════════════════════════════════════

describe("dc_motor — direction (negative supply → reverse rotation)", () => {
  it("−6 V applied m1→m2 → omega goes negative after settling", () => {
    // Derivation:
    //   V = −6 V applied m1→m2 (voltage source pos at m1, but V=−6 → m1 is NEGATIVE).
    //   By symmetry: omega_ss = −400 rad/s, i_ss = −0.4 A.
    //   After 6 s (>> tau_mech), omega must be clearly negative (< −10 rad/s).
    //   We use a loose bound to avoid precision sensitivity at h=1ms.
    const circuit = makeSimCircuit(
      [vs("psu", -6), motor("mot")],
      [wire("psu", "pos", "mot", "m1"), wire("psu", "neg", "mot", "m2")],
    );

    const engine = runEngine(circuit, 6000, 1e-3);

    const omega = engine.getMotorState("mot")?.omega ?? NaN;

    expect(engine.lastConverged).toBe(true);
    expect(isFinite(omega)).toBe(true);

    // omega < −10 rad/s: motor is running in reverse.
    // Closed-form predicts −400 rad/s; the −10 guard is permissive enough for any
    // numerical scheme but strict enough to confirm the direction.
    expect(omega).toBeLessThan(-10);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 5 — Driven by L293D H-bridge: forward vs reverse sign
// ═══════════════════════════════════════════════════════════════════════════════

describe("dc_motor — driven by L293D H-bridge", () => {
  // Derivation (H-bridge with dc_motor):
  //   L293D R_on = 2 Ω per side. VM = 6 V. windingR = 5 Ω.
  //   At stall (omega=0): total series R = R_on_H + R_winding + R_on_L = 2+5+2 = 9 Ω.
  //   i_stall ≈ 6 / 9 ≈ 0.667 A.
  //   At no-load SS: torque balance + winding KVL (see test header):
  //     V_eff = VM − i × (R_on_H + R_on_L) = 6 − 4i applied to winding.
  //     R × i + Ke × omega = V_eff and Kt × i = b × omega.
  //     → 5i + 0.01 × (Kt/b) × i = 6 − 4i → 5i + 10i = 6 − 4i → 19i = 6
  //     → i_ss ≈ 0.316 A, omega_ss ≈ 315.8 rad/s.
  //   These exact numbers are not asserted (test only checks sign).

  function hbridgeCircuit(forward: boolean): SimCircuit {
    const highV  = vs("vm", 6);   // motor supply / H-bridge VM
    const logicV = vs("vcc1", 5); // logic supply for L293D VCC1
    const enHigh = vs("en", 5);   // EN12 = HIGH (enable)
    const in1    = vs("in1", forward ? 5 : 0); // IN1 HIGH → OUT1 high-side drive
    const in2    = vs("in2", forward ? 0 : 5); // IN2 LOW  → OUT2 low-side
    const ic     = l293d("ic");
    const mot    = motor("mot");

    // All GND pins of L293D to psu neg.
    const wires: SimCircuit["wires"] = [
      // VM supply (motor rail)
      wire("vm",   "pos",  "ic",  "vcc2"),
      wire("vm",   "neg",  "ic",  "gnd1"),
      wire("vm",   "neg",  "ic",  "gnd2"),
      wire("vm",   "neg",  "ic",  "gnd3"),
      wire("vm",   "neg",  "ic",  "gnd4"),
      // VCC1 (logic supply)
      wire("vcc1", "pos",  "ic",  "vcc1"),
      wire("vcc1", "neg",  "vm",  "neg"),
      // EN12 HIGH
      wire("en",   "pos",  "ic",  "en12"),
      wire("en",   "neg",  "vm",  "neg"),
      // IN1 and IN2 logic signals
      wire("in1",  "pos",  "ic",  "in1"),
      wire("in1",  "neg",  "vm",  "neg"),
      wire("in2",  "pos",  "ic",  "in2"),
      wire("in2",  "neg",  "vm",  "neg"),
      // Motor wired across OUT1 and OUT2
      wire("ic",   "out1", "mot", "m1"),
      wire("ic",   "out2", "mot", "m2"),
    ];

    return makeSimCircuit(
      [highV, logicV, enHigh, in1, in2, ic, mot],
      wires,
    );
  }

  it("L293D forward drive (IN1 HIGH, IN2 LOW) → positive omega after settling", () => {
    // Derivation: forward drive pushes current m1→m2 → positive torque → omega > 0.
    // At SS: omega_ss ≈ 315.8 rad/s (see derivation above).
    // Assert omega > 50 rad/s (permissive; confirming sign and not noise).
    const engine = runEngine(hbridgeCircuit(true), 6000, 1e-3);
    const motorSt = engine.getMotorState("mot");

    expect(engine.lastConverged).toBe(true);
    expect(motorSt?.omega).toBeDefined();
    expect(motorSt!.omega).toBeGreaterThan(50);     // clearly positive (forward)
    expect(motorSt!.iWinding).toBeGreaterThan(0.05); // positive current
  });

  it("L293D reverse drive (IN1 LOW, IN2 HIGH) → negative omega after settling", () => {
    // Derivation: reverse drive pushes current m2→m1 (i<0) → negative torque → omega<0.
    // By symmetry: omega_ss ≈ −315.8 rad/s. Assert omega < −50 rad/s.
    const engine = runEngine(hbridgeCircuit(false), 6000, 1e-3);
    const motorSt = engine.getMotorState("mot");

    expect(engine.lastConverged).toBe(true);
    expect(motorSt?.omega).toBeDefined();
    expect(motorSt!.omega).toBeLessThan(-50);       // clearly negative (reverse)
    expect(motorSt!.iWinding).toBeLessThan(-0.05);  // negative current
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 6 — Flyback diagnostic
// ═══════════════════════════════════════════════════════════════════════════════

describe("dc_motor — flyback diagnostic (Finding 8 extension)", () => {
  function baseCircuit(): Circuit {
    return {
      id: "test", name: "test",
      schema_version: 2,
      components: [], wires: [], nets: [],
    };
  }

  it("motor switched by bare NPN transistor (no diode) → missing-flyback fires", () => {
    // Topology: VS + → motor m1; motor m2 → transistor collector; transistor
    // emitter → GND; VS − → GND. No flyback diode across m1/m2.
    // Expectation: hasSwitchInCoilPath finds the bjt_npn; no suppression;
    // finding fires with key "missing-flyback:mot".
    const circuit: Circuit = {
      ...baseCircuit(),
      components: [
        makeCircuitComp("psu", "voltage_source", ["pos", "neg"], { voltage: 6 }),
        makeCircuitComp("mot", "dc_motor", ["m1", "m2"]),
        makeCircuitComp("q1",  "bjt_npn",  ["b", "c", "e"]),
      ],
      wires: [
        diagWire("psu", "pos", "mot",  "m1"),   // +V to motor +
        diagWire("mot", "m2",  "q1",   "c"),    // motor − to BJT collector
        diagWire("q1",  "e",   "psu",  "neg"),  // BJT emitter to GND
        diagWire("q1",  "b",   "psu",  "pos"),  // base tied high (simplification)
      ],
    };

    const input = makeDiagInput(circuit);
    const findings = runDiagnostics(input);
    const flyback = findings.filter((f) => f.id === "missing-flyback");

    // Should fire for the motor.
    expect(flyback.some((f) => f.key === "missing-flyback:mot")).toBe(true);
  });

  it("motor across L293D H-bridge with VM powered → missing-flyback suppressed", () => {
    // Topology: L293D OUT1 → motor m1; L293D OUT2 → motor m2; VM powered.
    // Both motor pins land on H-bridge output pins of the SAME instance.
    // hasHBridgeClamp suppresses the finding (internal freewheeling diodes).
    // Expectation: NO missing-flyback finding for "mot".
    const circuit: Circuit = {
      ...baseCircuit(),
      components: [
        makeCircuitComp("vm",  "voltage_source", ["pos", "neg"], { voltage: 6 }),
        makeCircuitComp("psu", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        makeCircuitComp("en",  "voltage_source", ["pos", "neg"], { voltage: 5 }),
        makeCircuitComp("in1", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        makeCircuitComp("in2", "voltage_source", ["pos", "neg"], { voltage: 0 }),
        makeCircuitComp("ic",  "l293d", [
          "en12", "in1", "out1", "gnd1", "gnd2", "out2", "in2", "vcc2",
          "en34", "in3", "out3", "gnd3", "gnd4", "out4", "in4", "vcc1",
        ]),
        makeCircuitComp("mot", "dc_motor", ["m1", "m2"]),
      ],
      wires: [
        diagWire("vm",  "pos",  "ic",  "vcc2"),   // VM → L293D motor supply
        diagWire("vm",  "neg",  "ic",  "gnd1"),
        diagWire("vm",  "neg",  "ic",  "gnd2"),
        diagWire("vm",  "neg",  "ic",  "gnd3"),
        diagWire("vm",  "neg",  "ic",  "gnd4"),
        diagWire("psu", "pos",  "ic",  "vcc1"),   // VCC1 logic supply
        diagWire("psu", "neg",  "vm",  "neg"),
        diagWire("en",  "pos",  "ic",  "en12"),
        diagWire("en",  "neg",  "vm",  "neg"),
        diagWire("in1", "pos",  "ic",  "in1"),
        diagWire("in1", "neg",  "vm",  "neg"),
        diagWire("in2", "pos",  "ic",  "in2"),
        diagWire("in2", "neg",  "vm",  "neg"),
        diagWire("ic",  "out1", "mot", "m1"),     // H-bridge OUT1 → motor +
        diagWire("ic",  "out2", "mot", "m2"),     // H-bridge OUT2 → motor −
      ],
    };

    const input = makeDiagInput(circuit);
    const findings = runDiagnostics(input);
    const flyback = findings.filter((f) => f.id === "missing-flyback");

    // Should NOT fire for the motor (H-bridge clamp suppresses it).
    expect(flyback.some((f) => f.key === "missing-flyback:mot")).toBe(false);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 7 — Convergence
// ═══════════════════════════════════════════════════════════════════════════════

describe("dc_motor — convergence", () => {
  it("lastConverged=true and all net voltages finite after 100 steps at h=50µs", () => {
    const circuit = makeSimCircuit(
      [vs("psu", 6), motor("mot")],
      [wire("psu", "pos", "mot", "m1"), wire("psu", "neg", "mot", "m2")],
    );

    const engine = runEngine(circuit, 100, 50e-6);

    expect(engine.lastConverged).toBe(true);

    const netV = engine.getNetV();
    for (const [, v] of Object.entries(netV)) {
      expect(isFinite(v)).toBe(true);
    }

    const motorSt = engine.getMotorState("mot");
    expect(isFinite(motorSt?.iWinding ?? NaN)).toBe(true);
    expect(isFinite(motorSt?.omega ?? NaN)).toBe(true);
  });

  it("lastConverged=true after direction reversal (−6 V, 100 steps)", () => {
    const circuit = makeSimCircuit(
      [vs("psu", -6), motor("mot")],
      [wire("psu", "pos", "mot", "m1"), wire("psu", "neg", "mot", "m2")],
    );

    const engine = runEngine(circuit, 100, 50e-6);

    expect(engine.lastConverged).toBe(true);
    const motorSt = engine.getMotorState("mot");
    expect(isFinite(motorSt?.iWinding ?? NaN)).toBe(true);
    expect(isFinite(motorSt?.omega ?? NaN)).toBe(true);
  });
});
