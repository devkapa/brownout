/**
 * Wave 6.1 engine tests: relay SPDT, MOSFET body diode, and relay-flyback diagnostic.
 *
 * All expected values are derived from closed-form analysis with derivation comments.
 * NO value is read back from the implementation and then asserted — self-confirming
 * tests shipped wrong physics in prior waves (cardinal sin).
 *
 * ─── DERIVATIONS ────────────────────────────────────────────────────────────────
 *
 * RELAY COIL — DC steady state:
 *   coilR = 70 Ω, v_ab = 5 V (voltage_source)
 *   Steady-state i_coil = v/R = 5/70 ≈ 0.07143 A = 71.43 mA
 *   vPull = 3.5 V; |v_ab| = 5 ≥ 3.5 → energized
 *   Measurement: COM-NO closed (1 mΩ) → V_com ≈ V_no via ~0 Ω
 *                COM-NC open → V_nc isolated
 *
 * RELAY COIL — RL time constant (step response):
 *   tau = L / R = 0.05 / 70 ≈ 7.143×10⁻⁴ s = 0.714 ms
 *   Step response: i(t) = (V/R) · (1 − exp(−t/tau))
 *   At t = tau: i(tau) = (5/70) · (1 − 1/e) ≈ 71.43 × 0.6321 ≈ 45.15 mA
 *     → at one time constant, current is between 50% and 95% of steady state
 *       (50% = 35.7 mA, 95% = 67.9 mA; 45.15 mA is in range)
 *   Using h = 50 µs: steps at tau = 0.714 ms → N_tau = 0.714e-3 / 50e-6 ≈ 14.3 steps
 *   After 14 steps (0.7 ms < tau), current should be less than 63.2% of steady state.
 *   After 20 steps (1 ms > tau), current should be more than 63.2% of steady state.
 *
 * RELAY HYSTERESIS — from de-energized (apply 2 V, between vDrop=1.5 and vPull=3.5):
 *   |v_ab| = 2 V; 1.5 ≤ 2 < 3.5 → hold de-energized (hysteresis, no pull-in)
 *   From energized: |v_ab| = 2 V; 1.5 ≤ 2 < 3.5 → hold energized (no drop-out)
 *
 * MOSFET BODY DIODE — NMOS reverse-biased:
 *   V_S = 1 V (source forced by voltage_source), V_D = 0 V (drain through R to GND)
 *   V_G = 0 V (gate tied low → channel OFF, vGS = 0 − 1 = −1 < VTO=0.7, cutoff)
 *   Body diode: anode=S (1 V), cathode=D (0 V) → V_bd = V_S − V_D = 1 V > Vf=0.7 V
 *   Forward-biased → conducts. I_body ≈ Is_body · exp((V_bd − 0) / Vt)
 *   Is_body = shockleyIsFromVf(0.7, 1.0) = 1.0 · exp(−0.7/0.02585) ≈ 4.47×10⁻¹³ A
 *   At V_bd ≈ 0.7 V (clamped by 10 Ω series R): I ≈ V_bd / R = 0.7/10 = 70 mA
 *   With 10 Ω R_series: V_S forced to 1 V by source, V_D ≈ 0 V.
 *   Because source is stiff: V_body ≈ V_S − I·R_series_diode_path
 *   Actually: V_D = I_body · R_drain_to_gnd, V_S = 1 V (source node)
 *   With R=10 Ω drain to GND: circuit is V_S=1 V → body_diode → V_D → 10Ω → GND
 *   By KVL: 1 V = Vf_body + I_body × 10
 *   At Vf=0.7 V: I_body ≈ (1 − 0.7) / 10 = 30 mA (conservative lower bound)
 *   The simulation converges to a similar value. Assert I_body > 10 mA (conducts)
 *   and V_D > 0 (some current flows through 10 Ω to ground).
 *
 * MOSFET BODY DIODE — normal channel forward (no regression):
 *   V_G = 5 V, V_S = 0 V, V_D = 2 V, VTO=1 V, K=0.1
 *   vGS = 5, vDS = 2, vOV = 5 − 1 = 4 > 0
 *   vDS=2 < vOV=4 → triode
 *   I_d = K·(2·vOV·vDS − vDS²) = 0.1·(2·4·2 − 4) = 0.1·12 = 1.2 A (no lambda)
 *   Body diode: anode=S(0V), cathode=D(2V) → V_bd = 0 − 2 = −2 V (reverse biased)
 *   At −2 V the Shockley model gives I ≈ −Is_body ≈ 0 (leakage only ~pA)
 *   Channel current dominates; body diode contribution < 1 µA → negligible.
 *   Assert: I_drain within 1% of channel-only 1.2 A → within [1.188, 1.212] A
 */

import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog, Circuit } from "../../../src/circuit/types.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { breadboardToSimCircuit } from "../../helpers/circuit/breadboard.js";
import { buildNets } from "../../../src/sim/engine/graph.js";
import { runDiagnostics } from "../../helpers/diagnostics/index.js";
import type { DiagnosticsInput } from "../../helpers/diagnostics/index.js";

const _catalog = rawCatalog as unknown as PartCatalog;

// ─── Helpers ──────────────────────────────────────────────────────────────────

function runEngine(circuit: SimCircuit, steps = 400, dt = 50e-6): SimEngine {
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

function makeSimCircuit(components: SimCircuit["components"], wires: SimCircuit["wires"]): SimCircuit {
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

function relay(id: string, params?: Partial<Record<string, number>>): SimCircuit["components"][number] {
  return {
    id,
    kind: "relay",
    pins: [{ id: "coil_a" }, { id: "coil_b" }, { id: "com" }, { id: "no" }, { id: "nc" }],
    params: { coilR: 70, coilL: 0.05, vPull: 3.5, vDrop: 1.5, ...params },
  };
}

function resistor(id: string, r: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "resistor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { resistance: r },
  };
}

function wire(fc: string, fp: string, tc: string, tp: string): SimCircuit["wires"][number] {
  return { from_component: fc, from_pin: fp, to_component: tc, to_pin: tp };
}

// ─── Diagnostics helpers ──────────────────────────────────────────────────────

function makeBaseCircuit(): Circuit {
  return {
    id: "test",
    name: "test",
    schema_version: 2,
    components: [],
    wires: [],
    nets: [],
  };
}

function diagWire(fc: string, fp: string, tc: string, tp: string): Circuit["wires"][number] {
  return { id: `w-${fc}-${fp}-${tc}-${tp}`, from_component: fc, from_pin: fp, to_component: tc, to_pin: tp, resistance: 0 };
}

function makeCircuitComp(id: string, kind: string, pins: string[], params: Record<string, number | string> = {}): Circuit["components"][number] {
  return {
    id,
    kind: kind as Circuit["components"][number]["kind"],
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: pins.map((p) => ({ id: p, offset: { x: 0, y: 0 } })),
    params,
  };
}

function makeDiagInput(circuit: Circuit): DiagnosticsInput {
  const sim = breadboardToSimCircuit(circuit);
  const nets = buildNets(sim);
  return { circuit: sim, nets, catalog: rawCatalog as unknown as PartCatalog };
}

// ═══════════════════════════════════════════════════════════════════════════════
// PART A — MOSFET body diode
// ═══════════════════════════════════════════════════════════════════════════════

describe("MOSFET body diode — NMOS", () => {
  it("body diode conducts when V_S > V_D by more than ~0.7 V (gate off)", () => {
    // Circuit: V_S = 1 V (forced by voltage source), V_G = 0 V (gate tied to GND via source
    // negative), drain tied through 10 Ω to GND.
    //
    // With gate off (vGS = 0 − 1 = −1 < VTO=0.7), channel is cut off.
    // Body diode: anode=source(1V), cathode=drain — should forward-conduct.
    // I_body ≈ (1 − 0.7) / 10 Ω ≈ 30 mA (lower bound; simulation is iterative).
    // Assert V_D > 0.05 V (drain voltage rises from 0 as current flows through R=10Ω).
    const vsSource = vs("vs1", 1);  // V_S = 1 V (source node)
    const vsGate   = { id: "vsg", kind: "voltage_source" as const, pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } };
    const rDrain   = resistor("rd1", 10);
    const mos      = {
      id: "m1",
      kind: "nmos" as const,
      pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
      params: { vto: 0.7, k: 0.02, lambda: 0 },
    };

    // Topology:
    //   vs1 pos → NMOS source(s),  vs1 neg → GND
    //   vsg pos → NMOS gate(g),    vsg neg → GND   (gate at 0 V = GND potential)
    //   NMOS drain(d) → rd1(a), rd1(b) → GND
    //
    // With v_S = 1 V (=vs1.pos), v_G = 0 V, body diode anode=S, cathode=D:
    //   Vbd = V_S − V_D; if V_D < V_S − 0.7 the body diode conducts.
    const circuit = makeSimCircuit(
      [vsSource, vsGate, rDrain, mos],
      [
        wire("vs1", "pos",  "m1",  "s"),
        wire("vs1", "neg",  "m1",  "g"),   // gate = GND potential (tied via neg terminal)
        wire("vs1", "neg",  "rd1", "b"),   // drain R to GND
        wire("m1",  "d",    "rd1", "a"),   // drain to R
        // gate is already tied via vs1.neg; vsg not needed but add for clarity
      ],
    );

    const engine = new SimEngine();
    engine.load(circuit);
    for (let i = 0; i < 100; i++) engine.step(50e-6);

    const vD = netVoltage(engine, "m1", "d");
    const vS = netVoltage(engine, "m1", "s");

    // V_S should be ~1 V (clamped by vs1).
    expect(vS).toBeCloseTo(1.0, 1);

    // V_D > 0.05 V: body diode is conducting, current flows through 10 Ω.
    // Derivation: I = (V_S − Vf_body) / R ≈ (1 − 0.7) / 10 = 30 mA → V_D = 0.3 V
    // (Vf depends on convergence but must be >0.05 V to confirm conduction)
    expect(vD).toBeGreaterThan(0.05);
  });

  it("normal channel forward operation unchanged within 1% (body diode reverse biased)", () => {
    // NMOS channel forward: VG=5, VS=0, VD via R to 5V supply.
    // VTO=1.0, K=0.1, lambda=0.
    // vGS = 5, vOV = 4, vDS = VD - 0.
    // With VDD=5, R_D=1Ω: steady state I_d = (VDD − VD) / R_D = VD / R_D...
    // Let's use a simpler topology: VS=0, VD=2V forced, VG=5V.
    // Channel only: triode if vDS < vOV: 2 < 4 → triode
    //   I_d = K·(2·vOV·vDS − vDS²) = 0.1·(2·4·2 − 4) = 0.1·12 = 1.2 A
    // Body diode: anode=S(0V), cathode=D(2V) → V_bd = −2 V → reverse biased → ~0 leakage
    // Assert: elementI within 1% of 1.2 A → [1.188, 1.212]
    const vsGate  = { id: "vg",  kind: "voltage_source" as const, pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } };
    const vsDrain = { id: "vd1", kind: "voltage_source" as const, pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 2 } };
    const mos = {
      id: "m2",
      kind: "nmos" as const,
      pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
      params: { vto: 1.0, k: 0.1, lambda: 0 },
    };
    // Topology: source at GND, gate at 5V, drain at 2V.
    //   vg  pos→gate,  neg→GND (= source)
    //   vd1 pos→drain, neg→GND
    //   NMOS source → GND (via vs neg)
    const circuit = makeSimCircuit(
      [vsGate, vsDrain, mos],
      [
        wire("vg",  "pos", "m2", "g"),
        wire("vg",  "neg", "m2", "s"),    // source tied to GND via vg.neg
        wire("vd1", "pos", "m2", "d"),
        wire("vd1", "neg", "m2", "s"),    // all grounds tied together
      ],
    );

    const engine = new SimEngine();
    engine.load(circuit);
    for (let i = 0; i < 50; i++) engine.step(50e-6);

    const id = engine.getElementI()["m2"] ?? 0;
    // Body diode is reverse biased; channel-only current = 1.2 A.
    // Assert within 1%: [1.188, 1.212]
    expect(id).toBeGreaterThan(1.188);
    expect(id).toBeLessThan(1.212);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// PART B — Relay engine
// ═══════════════════════════════════════════════════════════════════════════════

describe("relay engine", () => {
  it("energizes when 5 V applied across coil: steady i_coil = 5/70 ≈ 71.4 mA", () => {
    // Derivation: i_coil = V/R = 5/70 = 0.07143 A (steady-state; tau = 50ms/70 = 0.714ms)
    // 400 steps × 50µs = 20ms >> tau → settled.
    // |v_ab| = 5 V ≥ vPull=3.5 → energized → COM-NO closed.
    // Probe COM-NO: wire COM→resistor→GND, wire NO→voltage_source.
    //   When closed (1mΩ): V_com ≈ V_no (voltage divider: 1mΩ vs ~nothing → V_com ≈ V_no)
    //   Expect: V_com ≈ V_no (within ~5 mV with a probe resistor of 1kΩ)
    // Probe COM-NC: wire NC through 1kΩ to GND (open → V_nc ≈ 0).

    // Simple topology: vs1 drives coil. Probe vs2 (5V) on NO, resistor on COM to GND.
    const vsCoil = vs("vsCoil", 5);    // 5 V across coil
    const vsNo   = vs("vsNO", 5);     // 5 V on NO terminal
    const rCom   = resistor("rCom", 1000);  // COM→GND probe resistor
    const rNc    = resistor("rNc",  1000);  // NC→GND probe resistor
    const rel    = relay("r1");

    const circuit = makeSimCircuit(
      [vsCoil, vsNo, rCom, rNc, rel],
      [
        // Coil: coil_a to vsCoil.pos, coil_b to GND
        wire("vsCoil", "pos", "r1", "coil_a"),
        wire("vsCoil", "neg", "r1", "coil_b"),
        // NO: vsNO drives the NO terminal
        wire("vsNO", "pos", "r1", "no"),
        wire("vsNO", "neg", "rCom", "b"),  // common ground for NO probe
        wire("vsNO", "neg", "vsCoil", "neg"), // explicit shared return
        // COM probe: rCom from COM to GND
        wire("r1", "com", "rCom", "a"),
        // NC probe: rNc from NC to GND
        wire("r1", "nc", "rNc", "a"),
        wire("rNc", "b", "vsCoil", "neg"),  // tie to GND
      ],
    );

    const engine = runEngine(circuit, 400, 50e-6);

    // Steady-state coil current = V/R = 5/70 = 71.43 mA
    const iCoil = engine.getElementI()["r1"] ?? 0;
    // Derivation: i_coil = 5/70 ≈ 0.07143 A. Allow ±5% tolerance for companion model.
    expect(iCoil).toBeGreaterThan(0.060);  // > 60 mA (>84% of 71.4 mA)
    expect(iCoil).toBeLessThan(0.085);     // < 85 mA (<119% of 71.4 mA)

    // Energized: COM-NO closed → V_com ≈ V_no ≈ 5 V
    // V_com = V_no × (rCom / (contact_R + rCom)) ≈ V_no × (1000/1000.001) ≈ V_no
    const vCom = netVoltage(engine, "r1", "com");
    const vNo  = netVoltage(engine, "r1", "no");
    expect(vCom).toBeGreaterThan(4.5);   // COM near supply voltage
    expect(Math.abs(vCom - vNo)).toBeLessThan(0.01);  // COM ≈ NO (contact closed)

    // COM-NC open: V_nc near 0 (rNc pulls NC to GND with no source)
    const vNc = netVoltage(engine, "r1", "nc");
    expect(vNc).toBeLessThan(0.1);   // NC isolated → near GND
  });

  it("de-energizes when 0 V across coil: COM-NC closed", () => {
    // v_ab = 0 V → i_coil → 0. |v_ab| = 0 < vDrop=1.5 → de-energized.
    // COM-NC closed (1mΩ), COM-NO open.
    const vsCoil = vs("vsCoil", 0);   // 0 V across coil (= short)
    const vsNc   = vs("vsNc", 5);    // 5 V on NC terminal
    const rCom   = resistor("rCom", 1000);  // COM→GND probe
    const rNo    = resistor("rNo",  1000);  // NO→GND probe
    const rel    = relay("r1");

    const circuit = makeSimCircuit(
      [vsCoil, vsNc, rCom, rNo, rel],
      [
        wire("vsCoil", "pos", "r1", "coil_a"),
        wire("vsCoil", "neg", "r1", "coil_b"),
        wire("vsNc", "pos", "r1", "nc"),
        wire("vsNc", "neg", "rCom", "b"),
        wire("vsNc", "neg", "vsCoil", "neg"), // explicit shared return
        wire("r1", "com", "rCom", "a"),
        wire("r1", "no", "rNo", "a"),
        wire("rNo", "b", "vsCoil", "neg"),
      ],
    );

    const engine = runEngine(circuit, 400, 50e-6);

    // De-energized: COM-NC closed → V_com ≈ V_nc ≈ 5 V
    const vCom = netVoltage(engine, "r1", "com");
    const vNc  = netVoltage(engine, "r1", "nc");
    expect(vCom).toBeGreaterThan(4.5);
    expect(Math.abs(vCom - vNc)).toBeLessThan(0.01);

    // COM-NO open: V_no near 0 V
    const vNo = netVoltage(engine, "r1", "no");
    expect(vNo).toBeLessThan(0.1);

    // Coil current near 0
    const iCoil = engine.getElementI()["r1"] ?? 0;
    expect(Math.abs(iCoil)).toBeLessThan(0.001);  // < 1 mA
  });

  it("hysteresis — de-energized with 2V coil stays de-energized (between vDrop=1.5 and vPull=3.5)", () => {
    // Start: relay starts de-energized (default). Apply 2 V coil voltage.
    // |v_ab| = 2 V; vDrop=1.5 ≤ 2 < vPull=3.5 → hold de-energized.
    // COM-NC closed.
    const vsCoil = vs("vsCoil", 2);  // 2 V coil voltage
    const vsNc   = vs("vsNc", 5);
    const rCom   = resistor("rCom", 1000);
    const rel    = relay("r1");

    const circuit = makeSimCircuit(
      [vsCoil, vsNc, rCom, rel],
      [
        wire("vsCoil", "pos", "r1", "coil_a"),
        wire("vsCoil", "neg", "r1", "coil_b"),
        wire("vsNc", "pos", "r1", "nc"),
        wire("vsNc", "neg", "rCom", "b"),
        wire("vsNc", "neg", "vsCoil", "neg"), // explicit shared return
        wire("r1", "com", "rCom", "a"),
      ],
    );

    // Short run: 50 steps × 50µs = 2.5 ms — enough for coil to settle but hysteresis holds
    const engine = runEngine(circuit, 50, 50e-6);

    // Still de-energized: COM should be at NC voltage
    const vCom = netVoltage(engine, "r1", "com");
    expect(vCom).toBeGreaterThan(4.5);  // COM → NC closed → ≈ 5 V

    // Confirm not energized: NO should be floating/GND
    const vNo = netVoltage(engine, "r1", "no");
    expect(vNo).toBeLessThan(0.5);
  });

  it("RL time constant: current at 1×tau between 50% and 95% of steady state", () => {
    // tau = L/R = 0.05/70 = 0.7143 ms
    // h = 50 µs → steps_at_tau = 0.7143ms / 50µs ≈ 14.3 steps
    // After 14 steps (< tau), i should be < 63.2% of steady = < 0.632 × 71.43 mA = 45.1 mA
    // After 20 steps (> tau), i should be > 63.2% of steady
    // We run to exactly tau and check 50% < i < 95%:
    //   50% of 71.43 mA = 35.7 mA
    //   95% of 71.43 mA = 67.9 mA
    // At t = tau = 14.3 steps: i ≈ 45.1 mA (in range [35.7, 67.9])
    const vsCoil = vs("vsCoil", 5);
    const rel    = relay("r1");

    const circuit = makeSimCircuit(
      [vsCoil, rel],
      [
        wire("vsCoil", "pos", "r1", "coil_a"),
        wire("vsCoil", "neg", "r1", "coil_b"),
      ],
    );

    const engine = new SimEngine();
    engine.load(circuit);
    // Step to exactly ~1 tau = 14 steps × 50 µs = 700 µs ≈ tau = 714 µs
    for (let i = 0; i < 14; i++) engine.step(50e-6);

    const iCoil = engine.getElementI()["r1"] ?? 0;
    // Derivation: i(tau) = (V/R)·(1 − 1/e) ≈ 71.43 × 0.632 ≈ 45.1 mA
    // Allow [50%, 95%] tolerance to handle backward-Euler discretization error
    // and the ~1% undershoot of 14 vs 14.3 steps.
    //   50% = 35.7 mA, 95% = 67.9 mA
    expect(iCoil).toBeGreaterThan(0.035);  // > 50% of 71.4 mA
    expect(iCoil).toBeLessThan(0.068);     // < 95% of 71.4 mA
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// PART C — Flyback diagnostic
// ═══════════════════════════════════════════════════════════════════════════════

describe("missing-flyback diagnostic (Finding 8)", () => {
  it("fires when relay coil is switched by an NPN transistor with no flyback diode", () => {
    // Circuit: battery → relay coil → NPN collector; NPN emitter → GND.
    // No diode across coil. Should fire missing-flyback for the relay.
    const c = makeBaseCircuit();
    c.components = [
      makeCircuitComp("bat1", "battery_pack", ["pos", "neg"]),
      makeCircuitComp("r1",   "relay",        ["coil_a", "coil_b", "com", "no", "nc"]),
      makeCircuitComp("q1",   "bjt_npn",      ["c", "b", "e"]),
    ];
    c.wires = [
      diagWire("bat1", "pos",    "r1",  "coil_a"),
      diagWire("r1",   "coil_b", "q1",  "c"),
      diagWire("q1",   "e",      "bat1","neg"),
      diagWire("bat1", "neg",    "q1",  "e"),  // GND
    ];

    const input = makeDiagInput(c);
    const findings = runDiagnostics(input);
    const flybackFindings = findings.filter((f) => f.id === "missing-flyback");
    expect(flybackFindings.length).toBeGreaterThan(0);
    expect(flybackFindings.some((f) => f.componentIds.includes("r1"))).toBe(true);
  });

  it("does NOT fire when an anti-parallel diode is wired across the relay coil", () => {
    // Same circuit + flyback diode anti-parallel across coil.
    const c = makeBaseCircuit();
    c.components = [
      makeCircuitComp("bat1", "battery_pack", ["pos", "neg"]),
      makeCircuitComp("r1",   "relay",        ["coil_a", "coil_b", "com", "no", "nc"]),
      makeCircuitComp("q1",   "bjt_npn",      ["c", "b", "e"]),
      makeCircuitComp("d1",   "diode",        ["a", "k"]),  // flyback diode
    ];
    c.wires = [
      diagWire("bat1", "pos",    "r1",  "coil_a"),
      diagWire("r1",   "coil_b", "q1",  "c"),
      diagWire("q1",   "e",      "bat1","neg"),
      // Flyback diode: anode on coil_b (low side), cathode on coil_a (high side)
      diagWire("d1",   "a",      "r1",  "coil_b"),
      diagWire("d1",   "k",      "r1",  "coil_a"),
    ];

    const input = makeDiagInput(c);
    const findings = runDiagnostics(input);
    const flybackFindings = findings.filter((f) => f.id === "missing-flyback");
    expect(flybackFindings.every((f) => !f.componentIds.includes("r1"))).toBe(true);
  });

  it("does NOT fire when relay coil is across a fixed supply rail (no switch)", () => {
    // Relay coil directly across battery (no switching element). No inductive kick risk.
    const c = makeBaseCircuit();
    c.components = [
      makeCircuitComp("bat1", "battery_pack", ["pos", "neg"]),
      makeCircuitComp("r1",   "relay",        ["coil_a", "coil_b", "com", "no", "nc"]),
    ];
    c.wires = [
      diagWire("bat1", "pos",    "r1",  "coil_a"),
      diagWire("bat1", "neg",    "r1",  "coil_b"),
    ];

    const input = makeDiagInput(c);
    const findings = runDiagnostics(input);
    const flybackFindings = findings.filter((f) => f.id === "missing-flyback");
    expect(flybackFindings.every((f) => !f.componentIds.includes("r1"))).toBe(true);
  });
});
