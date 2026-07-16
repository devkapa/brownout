/**
 * Wave A3 — pathological cold-start and rescue-ladder contracts
 * =============================================================
 *
 * These tests aim dcOperatingPoint() and the load()-seed rescue at circuits
 * chosen to be hostile: bistable latches (multiple DC solutions), stiff
 * near-ideal-source exponential stacks, open-loop gain in the millions, a
 * breakdown knee, and one structurally impossible network. Every assertion
 * states a physical contract, not an implementation snapshot; a solver
 * change that breaks one of these has changed what the user sees, not just
 * how the answer is computed.
 *
 * Status: the three defects these tests were authored red against
 * (2026-07-16) are fixed the same day — solveNonlinear now refuses to
 * accept an iteration in which pnjlim clamped a junction, and the OP
 * ladder screens every accepted point with the determinant pencil
 * stability test and escapes saddles along their growing eigenmode (see
 * newton.ts limitedThisIteration and sim-engine.ts
 * DC_OP_STABILITY_MAX_SIZE). The assertions are the correct physics; do
 * not widen them if they ever regress.
 */

import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { setLinearSystemBackendForTests } from "../../../src/sim/engine/linear-system.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type SimComponent = SimCircuit["components"][number];
type SimWire = SimCircuit["wires"][number];

// Backend hygiene: another test file in this suite forces the dense/sparse
// backend through the same module-level hook. Reset on both sides so these
// contracts always exercise the production auto selection.
beforeEach(() => { setLinearSystemBackendForTests(null); });
afterEach(() => { setLinearSystemBackendForTests(null); });

function voltageSource(id: string, voltage: number): SimComponent {
  return {
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage },
  };
}

function resistor(id: string, resistance: number): SimComponent {
  return {
    id,
    kind: "resistor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { resistance },
  };
}

function npn(id: string): SimComponent {
  return {
    id,
    kind: "bjt_npn",
    pins: [{ id: "c" }, { id: "b" }, { id: "e" }],
    params: { Is: 1e-16, betaF: 100, betaR: 1, nF: 1, nR: 1 },
  };
}

// cgs/cgd forced to zero: the pathology under test is the resistive
// bistability. The catalog's default gate capacitances would add 1 ps
// companion conductances of tens of siemens to the load() seed and blur the
// DC contract with integration artifacts.
function nmos(id: string, vto: number): SimComponent {
  return {
    id,
    kind: "nmos",
    pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
    params: { vto, k: 0.01, lambda: 0, cgs: 0, cgd: 0 },
  };
}

function wire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimWire {
  return {
    from_component: fromComponent,
    from_pin: fromPin,
    to_component: toComponent,
    to_pin: toPin,
  };
}

function nodeVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
  );
  if (!net) throw new Error(`Topology error: no net for ${componentId}.${pinId}`);
  const value = engine.getNetV()[net.id];
  if (value === undefined || !Number.isFinite(value)) {
    throw new Error(`Solve error: ${componentId}.${pinId} has no finite voltage`);
  }
  return value;
}

/** Every published node voltage must stay physical — no gigavolt debris. */
function expectFrameElectricallyBounded(engine: SimEngine, boundV: number, label: string): void {
  for (const [netId, value] of Object.entries(engine.getNetV())) {
    expect(
      Math.abs(value),
      `${label}: net ${netId} sits at ${String(value)} V, beyond the ${String(boundV)} V sanity bound`,
    ).toBeLessThan(boundV);
  }
}

/**
 * Classic two-transistor bistable: collector resistors to the 5 V rail,
 * each collector cross-linked to the other base through a base resistor.
 * The 1% collector-resistor mismatch mirrors real part tolerance; it keeps
 * the circuit bistable while making "which state is preferred" a property
 * of the physics rather than of floating-point noise on a perfectly
 * symmetric matrix.
 */
function bjtLatchCircuit(): SimCircuit {
  return {
    components: [
      voltageSource("vcc", 5),
      resistor("rc1", 1_000),
      resistor("rc2", 1_010),
      resistor("rb1", 10_000),
      resistor("rb2", 10_000),
      npn("q1"),
      npn("q2"),
    ],
    wires: [
      wire("vcc", "pos", "rc1", "a"),
      wire("vcc", "pos", "rc2", "a"),
      wire("rc1", "b", "q1", "c"),
      wire("rc2", "b", "q2", "c"),
      wire("rc1", "b", "rb2", "a"),
      wire("rb2", "b", "q2", "b"),
      wire("rc2", "b", "rb1", "a"),
      wire("rb1", "b", "q1", "b"),
      wire("q1", "e", "vcc", "neg"),
      wire("q2", "e", "vcc", "neg"),
    ],
    environment: { temperatureC: 25 },
  };
}

/** Cross-coupled NMOS inverters with the same deliberate 1% drain mismatch rationale. */
function nmosLatchCircuit(): SimCircuit {
  return {
    components: [
      voltageSource("vdd", 5),
      resistor("rd1", 1_000),
      resistor("rd2", 1_010),
      nmos("m1", 1),
      nmos("m2", 1),
    ],
    wires: [
      wire("vdd", "pos", "rd1", "a"),
      wire("vdd", "pos", "rd2", "a"),
      wire("rd1", "b", "m1", "d"),
      wire("rd2", "b", "m2", "d"),
      wire("rd1", "b", "m2", "g"),
      wire("rd2", "b", "m1", "g"),
      wire("m1", "s", "vdd", "neg"),
      wire("m2", "s", "vdd", "neg"),
    ],
  };
}

function solvedOperatingPoint(circuit: SimCircuit): {
  engine: SimEngine;
  result: ReturnType<SimEngine["dcOperatingPoint"]>;
} {
  const engine = new SimEngine();
  engine.load(circuit);
  const result = engine.dcOperatingPoint();
  return { engine, result };
}

// ─── Bistable cross-coupled BJT latch ────────────────────────────────────────

describe("pathological cold start — bistable BJT latch", () => {
  it("dcOperatingPoint converges on the latch at some ladder stage", () => {
    const { engine, result } = solvedOperatingPoint(bjtLatchCircuit());
    expect(
      result.converged,
      `latch OP failed: method=${result.method}, iterations=${String(result.iterations)}`,
    ).toBe(true);
    expect(engine.lastConverged).toBe(true);
    expectFrameElectricallyBounded(engine, 50, "BJT latch OP");
  });

  /**
   * Contract: a bistable latch must settle into one of its two STABLE
   * states — one transistor saturated (collector below ~0.3 V plus margin),
   * the other cut off (collector within a base-current drop of the rail).
   * The metastable saddle (both collectors near 1.2 V, both transistors
   * half-on) is also a root of the DC equations, but it is an unstable
   * equilibrium no physical latch can rest at, and the engine has no noise
   * source or later transient mechanism that would ever leave it.
   *
   * Engine mechanism: the direct Newton stage still walks to the saddle
   * (~1.18 V / ~1.18 V) even with the 1% mismatch — every homotopy stage
   * does, the saddle branch being the one connected to the cold start —
   * but the ladder's stability screen detects the odd growing mode via the
   * determinant pencil test and the escape kick along the unstable
   * eigenmode re-settles onto a stable state within the same stage.
   */
  it("the accepted point is one of the two stable states, not the saddle", () => {
    const { result, engine } = solvedOperatingPoint(bjtLatchCircuit());
    expect(result.converged).toBe(true);
    const vc1 = nodeVoltage(engine, "q1", "c");
    const vc2 = nodeVoltage(engine, "q2", "c");
    const low = Math.min(vc1, vc2);
    const high = Math.max(vc1, vc2);
    expect(
      low,
      `one collector must be saturated low; got Vc1=${String(vc1)}, Vc2=${String(vc2)}`,
    ).toBeLessThan(0.4);
    expect(
      high,
      `the other collector must sit near the rail; got Vc1=${String(vc1)}, Vc2=${String(vc2)}`,
    ).toBeGreaterThan(4.0);
  });

  it("repeated cold runs from fresh engines land on the identical point", () => {
    const baseline = solvedOperatingPoint(bjtLatchCircuit());
    expect(baseline.result.converged).toBe(true);
    const baselineNetV = baseline.engine.getNetV();
    for (let run = 0; run < 3; run++) {
      const repeat = solvedOperatingPoint(bjtLatchCircuit());
      expect(repeat.result.converged).toBe(true);
      // Same-state determinism is asserted as bit-identity: the engine is
      // deterministic code on identical inputs, so any drift here means a
      // hidden ordering or state leak, not legitimate numeric fuzz.
      const netV = repeat.engine.getNetV();
      for (const [netId, value] of Object.entries(baselineNetV)) {
        expect(netV[netId], `run ${String(run)}: net ${netId} diverged`).toBe(value);
      }
    }
  });
});

// ─── Cross-coupled NMOS inverter pair ────────────────────────────────────────

describe("pathological cold start — cross-coupled NMOS inverter pair", () => {
  it("dcOperatingPoint converges on the NMOS latch at some ladder stage", () => {
    const { engine, result } = solvedOperatingPoint(nmosLatchCircuit());
    expect(
      result.converged,
      `NMOS latch OP failed: method=${result.method}, iterations=${String(result.iterations)}`,
    ).toBe(true);
    expect(engine.lastConverged).toBe(true);
    expectFrameElectricallyBounded(engine, 50, "NMOS latch OP");
  });

  /**
   * Contract: in a stable state one drain sits below VTO=1 V (its partner
   * therefore cut off) and the released drain sits near the 5 V rail.
   *
   * Engine mechanism: Newton genuinely converges on the analytically
   * derivable saddle at Vd1=Vd2 near 1.58 V (both devices in saturation,
   * the second root of 10*(v-1)^2 = 5-v) — no junction limiter is involved
   * for MOSFETs, so only the stability screen catches this one; the escape
   * kick then lands the stable state, same as the BJT latch.
   */
  it("the accepted point is one of the two stable states, not the saddle", () => {
    const { result, engine } = solvedOperatingPoint(nmosLatchCircuit());
    expect(result.converged).toBe(true);
    const vd1 = nodeVoltage(engine, "m1", "d");
    const vd2 = nodeVoltage(engine, "m2", "d");
    const low = Math.min(vd1, vd2);
    const high = Math.max(vd1, vd2);
    expect(
      low,
      `one drain must be pulled below VTO; got Vd1=${String(vd1)}, Vd2=${String(vd2)}`,
    ).toBeLessThan(1.0);
    expect(
      high,
      `the other drain must be released to the rail; got Vd1=${String(vd1)}, Vd2=${String(vd2)}`,
    ).toBeGreaterThan(4.0);
  });

  it("repeated cold runs from fresh engines land on the identical point", () => {
    const baseline = solvedOperatingPoint(nmosLatchCircuit());
    expect(baseline.result.converged).toBe(true);
    const baselineNetV = baseline.engine.getNetV();
    for (let run = 0; run < 3; run++) {
      const repeat = solvedOperatingPoint(nmosLatchCircuit());
      expect(repeat.result.converged).toBe(true);
      const netV = repeat.engine.getNetV();
      for (const [netId, value] of Object.entries(baselineNetV)) {
        expect(netV[netId], `run ${String(run)}: net ${netId} diverged`).toBe(value);
      }
    }
  });
});

// ─── Stiff diode chain across a near-ideal source ────────────────────────────

describe("pathological cold start — three-diode chain through 1 milliohm", () => {
  /**
   * Validity envelope: an ideal 5 V source feeding three series junctions
   * through 1 mOhm of wiring — a conditioning torture case (nine decades
   * between the wiring conductance and the node shunts, kiloamp currents).
   * The diodes are anchored at Vf=0.7 V at a 1 kA rating (a bus-bar clamp
   * scale device) so that the honest solution keeps every junction inside
   * the ordinary 0.6-0.8 V band: the analytical fixed point is about
   * 2.82 kA with 0.727 V per junction. With small-signal 10 mA diodes the
   * same topology is still solvable but each drop sits near 1.0 V, which
   * would test the log-overdrive region instead of the wiring-bounded
   * envelope this case is about.
   */
  it("converges with wiring-bounded current and 0.6-0.8 V per junction", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("src", 5),
        resistor("rw", 0.001),
        { id: "d1", kind: "diode", pins: [{ id: "a" }, { id: "k" }], params: { vf: 0.7, iRated: 1_000, n: 1 } },
        { id: "d2", kind: "diode", pins: [{ id: "a" }, { id: "k" }], params: { vf: 0.7, iRated: 1_000, n: 1 } },
        { id: "d3", kind: "diode", pins: [{ id: "a" }, { id: "k" }], params: { vf: 0.7, iRated: 1_000, n: 1 } },
      ],
      wires: [
        wire("src", "pos", "rw", "a"),
        wire("rw", "b", "d1", "a"),
        wire("d1", "k", "d2", "a"),
        wire("d2", "k", "d3", "a"),
        wire("d3", "k", "src", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    const { engine, result } = solvedOperatingPoint(circuit);
    expect(result.converged, `chain OP failed at stage ${result.method}`).toBe(true);

    const current = engine.getElementI().rw ?? Number.NaN;
    // Hard physical ceiling: the source dead-shorted through the wiring.
    expect(current).toBeLessThan(5 / 0.001);
    // Envelope-consistent band: 5 V minus three drops inside [0.6, 0.8] V,
    // divided by the wiring resistance.
    expect(current).toBeGreaterThan((5 - 3 * 0.8) / 0.001);
    expect(current).toBeLessThan((5 - 3 * 0.6) / 0.001);

    const drops = [
      nodeVoltage(engine, "d1", "a") - nodeVoltage(engine, "d2", "a"),
      nodeVoltage(engine, "d2", "a") - nodeVoltage(engine, "d3", "a"),
      nodeVoltage(engine, "d3", "a") - nodeVoltage(engine, "d3", "k"),
    ];
    for (const [index, drop] of drops.entries()) {
      expect(drop, `junction ${String(index + 1)} drop out of band`).toBeGreaterThan(0.6);
      expect(drop, `junction ${String(index + 1)} drop out of band`).toBeLessThan(0.8);
    }

    // Series KCL: the same current must be reported through every element.
    const currents = engine.getElementI();
    for (const id of ["d1", "d2", "d3"]) {
      const branch = currents[id] ?? Number.NaN;
      expect(Math.abs(branch - current) / current).toBeLessThanOrEqual(1e-9);
    }
  });
});

// ─── High-gain open-loop comparator ──────────────────────────────────────────

describe("pathological cold start — LM358 open-loop comparator at microvolt offsets", () => {
  /**
   * openLoopGain is raised to 1e6 (an editable model parameter) so that a
   * genuine few-microvolt differential decides the output: at the default
   * 1e5 gain a 5 uV offset commands only 0.5 V of ideal output and the amp
   * would legitimately rest mid-rail. The LM358 output stage is not
   * rail-to-rail; its high rail is VCC - 1.5 V by model contract.
   */
  function comparatorCircuit(plusMicrovolts: number): SimCircuit {
    return {
      components: [
        voltageSource("vdd", 5),
        voltageSource("vp", 2.5 + plusMicrovolts * 1e-6),
        voltageSource("vm", 2.5),
        resistor("rl", 100_000),
        {
          id: "amp",
          kind: "lm358",
          pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((id) => ({ id })),
          params: { openLoopGain: 1_000_000 },
        },
      ],
      wires: [
        wire("vdd", "pos", "amp", "8"),
        wire("vdd", "neg", "amp", "4"),
        wire("vp", "pos", "amp", "3"),
        wire("vp", "neg", "vdd", "neg"),
        wire("vm", "pos", "amp", "2"),
        wire("vm", "neg", "vdd", "neg"),
        wire("amp", "1", "rl", "a"),
        wire("rl", "b", "vdd", "neg"),
      ],
    };
  }

  it("+5 uV lands at the high rail without regime oscillation", () => {
    const { engine, result } = solvedOperatingPoint(comparatorCircuit(5));
    expect(result.converged).toBe(true);
    // regimeIterations counts committed-regime re-solves; the settle loop
    // caps each pass at 8 outer iterations. Staying strictly below the cap
    // proves the saturation active-set committed once and stopped moving
    // instead of flapping between linear and railed regimes until cut off.
    expect(
      result.regimeIterations,
      `regime picture kept moving: ${String(result.regimeIterations)} settle solves`,
    ).toBeLessThan(8);
    const out = nodeVoltage(engine, "amp", "1");
    // High rail is 3.5 V (VCC - 1.5), minus the output-resistance drop into
    // the 100 kOhm load.
    expect(out).toBeGreaterThan(3.4);
    expect(out).toBeLessThanOrEqual(3.5);
  });

  it("-5 uV lands at the low rail without regime oscillation", () => {
    const { engine, result } = solvedOperatingPoint(comparatorCircuit(-5));
    expect(result.converged).toBe(true);
    expect(result.regimeIterations).toBeLessThan(8);
    const out = nodeVoltage(engine, "amp", "1");
    // Low rail is GND + 20 mV by model contract.
    expect(out).toBeGreaterThanOrEqual(0);
    expect(out).toBeLessThan(0.1);
  });
});

// ─── Zener regulation knee ───────────────────────────────────────────────────

describe("pathological cold start — zener shunt regulator on the breakdown knee", () => {
  /**
   * Validity envelope: 9 V source, 330 Ohm series resistor, 5.1 V zener to
   * ground, operating at roughly 11.8 mA — about a decade past the 5 mA
   * knee anchor, so the expected cathode voltage is vz plus one decade of
   * logarithmic overdrive (n=1: about 22 mV). The band below allows the
   * knee's soft region without admitting either the pre-breakdown regime
   * (cathode drifting toward 9 V) or a hard-clamp artifact (cathode pinned
   * exactly at vz regardless of current).
   */
  it("lands cleanly on the knee with a self-consistent branch current", () => {
    const circuit: SimCircuit = {
      components: [
        voltageSource("src", 9),
        resistor("rs", 330),
        {
          id: "dz",
          kind: "zener_diode",
          pins: [{ id: "a" }, { id: "k" }],
          params: { vf: 0.7, vz: 5.1, iRated: 0.1, iz_knee: 0.005 },
        },
      ],
      wires: [
        wire("src", "pos", "rs", "a"),
        wire("rs", "b", "dz", "k"),
        wire("dz", "a", "src", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    const { engine, result } = solvedOperatingPoint(circuit);
    expect(result.converged).toBe(true);

    const cathode = nodeVoltage(engine, "dz", "k");
    expect(cathode).toBeGreaterThan(5.0);
    expect(cathode).toBeLessThan(5.3);

    const currents = engine.getElementI();
    const seriesCurrent = currents.rs ?? Number.NaN;
    // The solved node must reproduce Ohm's law on the series arm...
    expect(Math.abs(seriesCurrent - (9 - cathode) / 330)).toBeLessThanOrEqual(1e-9);
    // ...and KCL must close through the zener (reverse conduction reads
    // negative in the anode-to-cathode measurement convention).
    const zenerCurrent = currents.dz ?? Number.NaN;
    expect(zenerCurrent).toBeLessThan(-0.005);
    expect(Math.abs(seriesCurrent + zenerCurrent)).toBeLessThanOrEqual(1e-9);
  });
});

// ─── Rescue at load() ────────────────────────────────────────────────────────

describe("Wave A3 rescue at load()", () => {
  /**
   * Contract: load()'s published convergence flag must be truthful, because
   * the entire Wave A3 seed rescue is gated on it. If the seed genuinely
   * converged, the frame must satisfy the device equations it was stamped
   * with — in particular no silicon junction can rest multiple volts into
   * forward bias, since the diode law puts its current beyond any rail's
   * ability to supply. If the seed did not reach such a frame, the flag
   * must be false so the rescue ladder (or the documented quiet give-up)
   * engages.
   *
   * Fixed defect this locks against (root cause was in newton.ts): the
   * Newton loop's convergence test was delta-x only. On this latch,
   * iteration 1 parks both bases at the 5 V rail with the BJTs stamped at
   * VBE=0 (off); iteration 2 re-stamps with pnjlim limiting the evaluation
   * voltage near vCrit, the resulting device current is still negligible,
   * the node voltages do not move, and the loop declared convergence while
   * the limiter was still actively walking — publishing lastConverged=true
   * on a frame with VBE=5.0 V and picoamp base currents. solveNonlinear now
   * refuses to accept an iteration in which any junction limiter fired
   * (SPICE's guard); with limiting refused, the walk lets the softly-driven
   * bases respond and the seed converges honestly (~10 iterations, every
   * junction inside the ordinary band).
   */
  it("load() must not report a converged seed on a frame violating the junction law", () => {
    const engine = new SimEngine();
    engine.load(bjtLatchCircuit());
    expectFrameElectricallyBounded(engine, 50, "latch seed frame");
    const vbe1 = nodeVoltage(engine, "q1", "b") - nodeVoltage(engine, "q1", "e");
    const vbe2 = nodeVoltage(engine, "q2", "b") - nodeVoltage(engine, "q2", "e");
    const truthful = !engine.lastConverged || (vbe1 < 1.0 && vbe2 < 1.0);
    expect(
      truthful,
      "lastConverged=true with a base-emitter junction at "
      + `${String(Math.max(vbe1, vbe2))} V; a truthful converged frame keeps `
      + "every junction under 1 V or reports the seed as failed so the "
      + "rescue ladder can engage",
    ).toBe(true);
  });

  /**
   * A circuit whose plain 1 ps seed fails AND whose rescue succeeds could
   * not be constructed despite genuine attempts, all replayed against the
   * real engine (scratchpad a3-patho-probe*, 2026-07-16):
   *   - the cross-coupled BJT latch at 5 V and 24 V with base resistances
   *     of 10 kOhm, 1 Ohm, and 1 mOhm: under the (since fixed) delta-x-only
   *     convergence check every variant falsely converged at iteration 2;
   *     with the limiter guard in place the 10 kOhm variant converges
   *     honestly within the 50-iteration seed cap instead (the softly
   *     driven bases respond as the junctions open), so it still never
   *     exercises the rescue;
   *   - TL431 feedback loops (ref tied to cathode and 2x dividers, series
   *     resistances from 220 Ohm down to 10 mOhm, 9-36 V): the sharp
   *     nZ=0.5 knee converges honestly in 3-5 iterations from a cold start
   *     because the overdrive clamp keeps the first linearisation finite;
   *   - a 1 F capacitor (1e12 S companion at the 1 ps seed) in series with
   *     a junction: 13 honest iterations, healthy residual;
   *   - 12 V dead across a small-signal diode: the tangent-extended stamp
   *     is linear in the far region, honest convergence in 3 iterations.
   * Every probed candidate either converges honestly at the seed or has no
   * solution at all, so the success leg of the rescue stays unreachable by
   * construction. What remains testable without mocking internals is the
   * failure leg: a structurally impossible network must fail the seed,
   * walk the ladder, give up cleanly, and leave the engine uncorrupted and
   * reusable.
   */
  it("impossible parallel sources fail cleanly through the whole ladder", () => {
    const impossible: SimCircuit = {
      components: [voltageSource("s1", 5), voltageSource("s2", 3)],
      wires: [
        wire("s1", "pos", "s2", "pos"),
        wire("s1", "neg", "s2", "neg"),
      ],
    };
    const engine = new SimEngine();
    engine.load(impossible);
    // Two ideal sources disagreeing across the same net pair have no
    // solution; the seed and its rescue must both refuse rather than
    // publish a compromise voltage.
    expect(engine.lastConverged).toBe(false);
    expectFrameElectricallyBounded(engine, 50, "impossible-source seed frame");

    const before = JSON.stringify(engine.getNetV());
    const result = engine.dcOperatingPoint();
    expect(result.converged).toBe(false);
    // "pseudo-transient" is the terminal ladder stage: reporting it proves
    // gmin stepping, source stepping, and PTA all genuinely ran and were
    // reached, i.e. the rescue path is exercised end to end.
    expect(result.method).toBe("pseudo-transient");
    expect(result.iterations).toBeGreaterThan(0);
    // Total failure is diagnostic-only by contract: the engine must keep
    // the exact pre-call electrical state.
    expect(JSON.stringify(engine.getNetV())).toBe(before);

    // The failed engine must remain fully usable: a subsequent valid load
    // on the same instance solves to reference accuracy.
    engine.load({
      components: [voltageSource("src", 9), resistor("top", 10_000), resistor("bottom", 20_000)],
      wires: [
        wire("src", "pos", "top", "a"),
        wire("top", "b", "bottom", "a"),
        wire("bottom", "b", "src", "neg"),
      ],
    });
    engine.step(1e-4);
    expect(engine.lastConverged).toBe(true);
    expect(Math.abs(nodeVoltage(engine, "top", "b") - 6)).toBeLessThanOrEqual(1e-6);
  });
});
