/**
 * Wave A5 cross-validation and integration tests
 * ==============================================
 *
 * runSmallSignalAc (linearized SPICE-style AC, ac-analysis.ts) and the legacy
 * runAcSweep (large-signal transient + single-bin DFT, run-ac-sweep.ts)
 * answer different questions on purpose. These tests lock the boundary
 * between them:
 *
 *   1. Honesty check: on a LINEAR circuit the two modes measure the same
 *      physical transfer function, so their results must agree within the
 *      DFT's own documented settling tolerance (1 dB magnitude / 5 degrees
 *      phase). Both are additionally anchored to the analytic answer so a
 *      shared bug cannot self-confirm.
 *   2. Divergence check: on a NONLINEAR circuit driven hard (diode clipper)
 *      the two must legitimately DISAGREE — the small-signal result is the
 *      tangent conductance gd at the bias point while the DFT fundamental
 *      reflects clipping. Agreement here would mean one of them is not
 *      measuring what it claims to.
 *   3. OP interplay: a converged A3 operating point (op-amp regime settle)
 *      feeds a converged AC run; a failed OP (impossible parallel sources)
 *      surfaces as a clean throw with no engine state corruption.
 *   4. MCU rejection, the committed-digital Thevenin default, and the
 *      pattern-reuse sweep performance contract.
 *
 * Analytic expectations are derived from first principles (never read back
 * from the implementation under test) using the same exported primitives the
 * device stamps consume. Never loosen a tolerance to make a failing engine
 * pass — report the engine bug.
 */

import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";
import { logSweepValues, type AcSweepSpec } from "../../../src/analysis/jobs.js";
import { runAcSweep } from "../../../src/analysis/run-ac-sweep.js";
import { runSmallSignalAc } from "../../../src/sim/engine/ac-analysis.js";
import { shockleyIsFromVf } from "../../../src/sim/engine/elements.js";
import {
  GATE_R_OUT,
  NODE_RSHUNT_G,
  SimEngine,
  junctionForwardVoltage,
  thermalVoltage,
  type SimCircuit,
} from "../../../src/sim/engine/sim-engine.js";

const catalog = rawCatalog as unknown as PartCatalog;

type SimComponent = SimCircuit["components"][number];
type SimWire = SimCircuit["wires"][number];

/**
 * GMIN floor every junction acStamp adds on top of the tangent conductance
 * (semiconductors.ts AC_GMIN, module-private there). Part of the documented
 * linearized matrix, so analytic expectations must include it.
 */
const AC_GMIN = 1e-12;

/** Engine default ambient (sim-engine NOMINAL_TEMP_C) — fixtures set no
 *  circuit.environment, so every junction linearizes at 25 C. */
const AMBIENT_C = 25;

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

/** Deterministic net id for a component pin, resolved post-buildNets. */
function netIdForPin(engine: SimEngine, componentId: string, pinId: string): string {
  const net = engine.nets.find((n) =>
    n.pins.some(([c, p]) => c === componentId && p === pinId),
  );
  if (!net) throw new Error(`no net contains ${componentId}.${pinId}`);
  return net.id;
}

/** Absolute circular distance between two phases in degrees, in [0, 180]. */
function phaseDistanceDeg(a: number, b: number): number {
  let d = a - b;
  while (d > 180) d -= 360;
  while (d <= -180) d += 360;
  return Math.abs(d);
}

/** [context, netId, first, second] rows for every non-Object.is entry
 *  (same drift-report shape as dc-op-parity.test.ts). */
function collectNetVDrift(
  context: string,
  first: Record<string, number>,
  second: Record<string, number>,
): Array<[string, string, number | undefined, number | undefined]> {
  const drift: Array<[string, string, number | undefined, number | undefined]> = [];
  const keys = new Set([...Object.keys(first), ...Object.keys(second)]);
  for (const key of [...keys].sort()) {
    if (!Object.is(first[key], second[key])) {
      drift.push([context, key, first[key], second[key]]);
    }
  }
  return drift;
}

// ─── Fixtures ────────────────────────────────────────────────────────────────

/**
 * Loaded RC divider: sg → R1 → out, with R2 and C to ground at out.
 * First-order (single real pole), so the DFT's 5-cycle settle derivation
 * holds exactly and its 1 dB tolerance is the honest comparison bound.
 * rSource = 0 keeps the driven node at exactly the configured amplitude,
 * making the DFT reference net and the unit small-signal drive the same
 * quantity. sg.neg is the first source return, so buildNets makes it the
 * "gnd" reference and node voltages are absolute.
 */
function rcDividerCircuit(): SimCircuit {
  const components: SimComponent[] = [
    {
      id: "sg",
      kind: "signal_gen",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { waveform: "sine", amplitude: 1, offset: 0, frequency: 1000, rSource: 0, enabled: 1 },
    },
    {
      id: "r1",
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 1000 },
    },
    {
      id: "r2",
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 2000 },
    },
    {
      id: "c1",
      kind: "capacitor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { capacitance: 1e-7 },
    },
  ];
  const wires: SimWire[] = [
    wire("sg", "pos", "r1", "a"),
    wire("r1", "b", "r2", "a"),
    wire("r1", "b", "c1", "a"),
    wire("r2", "b", "sg", "neg"),
    wire("c1", "b", "sg", "neg"),
  ];
  return { components, wires };
}

/**
 * Diode clipper: sg (1 V offset, 4 V amplitude) → 1 kOhm → diode to ground.
 * The offset biases the diode conducting at the OP, so the small-signal gain
 * is a resistive divider against the tangent conductance gd — strongly
 * attenuating. The 4 V drive clips only the positive half-cycles, so the
 * large-signal fundamental stays near half the drive. The two modes must
 * disagree by a wide margin: that disagreement is the PROOF they measure
 * different things (tangent slope vs distorted steady-state fundamental).
 */
function diodeClipperCircuit(): SimCircuit {
  const components: SimComponent[] = [
    {
      id: "sg",
      kind: "signal_gen",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { waveform: "sine", amplitude: 4, offset: 1, frequency: 1000, rSource: 0, enabled: 1 },
    },
    {
      id: "rs",
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 1000 },
    },
    {
      id: "d1",
      kind: "diode",
      pins: [{ id: "a" }, { id: "k" }],
      params: { vf: 0.7, iRated: 0.1, n: 1 },
    },
  ];
  const wires: SimWire[] = [
    wire("sg", "pos", "rs", "a"),
    wire("rs", "b", "d1", "a"),
    wire("d1", "k", "sg", "neg"),
  ];
  return { components, wires };
}

/**
 * LM358 unity follower (same topology as wave4-opamps.test.ts): the op-amp
 * carries committed regime state, so its OP must run the A3 regime-settle
 * loop before the linearization bias is trustworthy.
 */
function followerCircuit(): SimCircuit {
  return {
    components: [
      { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "vin", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 2.0 } },
      {
        id: "oa1",
        kind: "lm358",
        pins: [
          { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
          { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
        ],
        params: {},
      },
    ],
    wires: [
      wire("v1", "pos", "oa1", "8"),
      wire("v1", "neg", "oa1", "4"),
      wire("vin", "pos", "oa1", "3"),
      wire("vin", "neg", "v1", "neg"),
      wire("oa1", "1", "oa1", "2"),
    ],
  };
}

/**
 * Two ideal voltage sources hard-paralleled at different values: the two
 * branch rows are linearly dependent with inconsistent right-hand sides, so
 * no homotopy rung can produce a solution — the canonical OP failure.
 */
function conflictingSourcesCircuit(): SimCircuit {
  return {
    components: [
      { id: "va", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "vb", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
    ],
    wires: [
      wire("va", "pos", "vb", "pos"),
      wire("va", "neg", "vb", "neg"),
    ],
  };
}

/** Plain converging RC — the post-failure recovery baseline. Component ids
 *  share nothing with conflictingSourcesCircuit so load()'s state
 *  carry-forward cannot alias the two circuits. */
function recoveryRcCircuit(): SimCircuit {
  return {
    components: [
      { id: "src", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
    ],
    wires: [
      wire("src", "pos", "r1", "a"),
      wire("r1", "b", "c1", "a"),
      wire("c1", "b", "src", "neg"),
    ],
  };
}

const blinkHex = readFileSync(join(__dirname, "__fixtures__/blink.hex"), "utf-8");

function arduinoCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex: blinkHex, usb_power: 1 },
      },
      { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
    ],
    wires: [
      wire("vs", "pos", "rl", "a"),
      wire("rl", "b", "vs", "neg"),
    ],
  };
}

/**
 * 74hc14 inverter with its input tied LOW, so the committed output stage
 * drives HIGH — a Thevenin resistance to the VCC rail. A capacitor from the
 * output to ground turns that resistance into a measurable single-pole
 * rolloff. The AC drive is designated on the SUPPLY: the output stage is an
 * admittance to the rail node, so a rail perturbation passes through the
 * documented g = 1/Rout into the RC pole. The 74hc14 has no acStamp hook —
 * this exercises ac-analysis.ts stampCommittedDigitalOutputs exclusively.
 */
function hc14RolloffCircuit(): SimCircuit {
  return {
    components: [
      { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      {
        id: "u1",
        kind: "74hc14",
        pins: [
          { id: "1a" }, { id: "1y" },
          { id: "2a" }, { id: "2y" },
          { id: "3a" }, { id: "3y" },
          { id: "gnd" },
          { id: "4y" }, { id: "4a" },
          { id: "5y" }, { id: "5a" },
          { id: "6y" }, { id: "6a" },
          { id: "vcc" },
        ],
        params: {},
      },
      { id: "cl", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("u1", "1a", "vcc", "neg"),
      wire("u1", "1y", "cl", "a"),
      wire("cl", "b", "vcc", "neg"),
    ],
  };
}

/** N-stage RC low-pass ladder (dc-op-parity shape, deeper): every stage adds
 *  one node, so 30 stages give a > 30-node MNA system. */
function rcLadderCircuit(stages: number): SimCircuit {
  const components: SimComponent[] = [
    {
      id: "src",
      kind: "voltage_source",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { voltage: 5 },
    },
  ];
  const wires: SimWire[] = [wire("src", "pos", "r1", "a")];
  for (let stage = 1; stage <= stages; stage++) {
    components.push({
      id: `r${String(stage)}`,
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 1000 },
    });
    components.push({
      id: `c${String(stage)}`,
      kind: "capacitor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { capacitance: 1e-8 },
    });
    wires.push(wire(`r${String(stage)}`, "b", `c${String(stage)}`, "a"));
    wires.push(wire(`c${String(stage)}`, "b", "src", "neg"));
    if (stage < stages) {
      wires.push(wire(`r${String(stage)}`, "b", `r${String(stage + 1)}`, "a"));
    }
  }
  return { components, wires };
}

// ─── 1. DFT cross-check on a linear fixture ─────────────────────────────────

describe("A5 cross-validation — linear RC divider, DFT vs small-signal", () => {
  it("magnitudes agree within 1 dB and phases within 5 degrees at every shared frequency", () => {
    const circuit = rcDividerCircuit();

    const probe = new SimEngine();
    probe.load(circuit);
    const outNet = netIdForPin(probe, "r1", "b");

    // Same generator the sweep runner uses, so both analyses see the exact
    // same frequency list (bitwise) rather than a re-derived approximation.
    const freqs = logSweepValues(20, 20000, 7);

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg",
      fromHz: 20,
      toHz: 20000,
      points: 7,
      outputs: [{ netId: outNet }],
    };
    const dft = runAcSweep(spec, rcDividerCircuit());
    expect(dft.fHz).toEqual(freqs);
    // First-order fixture: the 5-cycle settle derivation holds, so a flagged
    // point would mean the comparison premise (1 dB DFT accuracy) is void.
    expect(dft.notSettled).toEqual(freqs.map(() => false));
    expect(dft.pointFailures.flat()).toEqual([]);

    const engine = new SimEngine();
    engine.load(rcDividerCircuit());
    const ss = runSmallSignalAc(engine, {
      inputId: "sg",
      outputNetIds: [outNet],
      frequenciesHz: freqs,
    });
    expect(ss.frequenciesHz).toEqual(freqs);

    // Analytic anchor so the two implementations cannot co-sign a shared
    // wrong answer: out node KCL gives H = g1 / (g1 + g2 + G_shunt + jwC).
    const g1 = 1 / 1000;
    const g2 = 1 / 2000;
    const c = 1e-7;

    const dftOut = dft.outputs[0]!;
    const ssOut = ss.outputs[0]!;
    for (let i = 0; i < freqs.length; i++) {
      const f = freqs[i]!;
      const omega = 2 * Math.PI * f;
      const denRe = g1 + g2 + NODE_RSHUNT_G;
      const denIm = omega * c;
      const analyticMagDb = 20 * Math.log10(g1 / Math.hypot(denRe, denIm));
      const analyticPhaseDeg = (-Math.atan2(denIm, denRe) * 180) / Math.PI;

      // Small-signal is the exact linear solve: near machine precision.
      expect(
        Math.abs(ssOut.magnitudeDb[i]! - analyticMagDb),
        `small-signal magnitude vs analytic at ${String(f)} Hz`,
      ).toBeLessThan(1e-9);
      expect(
        phaseDistanceDeg(ssOut.phaseDeg[i]!, analyticPhaseDeg),
        `small-signal phase vs analytic at ${String(f)} Hz`,
      ).toBeLessThan(1e-6);

      // Headline honesty check: the DFT agrees within its own documented
      // settling tolerance. Do not widen these bounds — a violation means
      // one of the two analyses is broken, not that the test is too strict.
      expect(
        Math.abs(dftOut.magnitudeDb[i]! - ssOut.magnitudeDb[i]!),
        `DFT vs small-signal magnitude at ${String(f)} Hz`,
      ).toBeLessThanOrEqual(1.0);
      expect(
        phaseDistanceDeg(dftOut.phaseDeg[i]!, ssOut.phaseDeg[i]!),
        `DFT vs small-signal phase at ${String(f)} Hz`,
      ).toBeLessThanOrEqual(5.0);
    }
  });
});

// ─── 2. Nonlinear fixture: the modes legitimately diverge ───────────────────

describe("A5 divergence — diode clipper at high drive", () => {
  const freqs = logSweepValues(100, 1000, 3);

  it("small-signal gain equals the analytic gd divider at the committed OP", () => {
    const engine = new SimEngine();
    engine.load(diodeClipperCircuit());
    const outNet = netIdForPin(engine, "rs", "b");

    const ss = runSmallSignalAc(engine, {
      inputId: "sg",
      outputNetIds: [outNet],
      frequenciesHz: freqs,
    });

    // runSmallSignalAc committed the OP it linearized about, so the public
    // readout IS the bias point. Cathode sits on the gnd reference net, so
    // the out-node voltage is the junction voltage directly.
    const vd = engine.getNetV()[outNet] ?? 0;
    // Fixture premise: the 1 V offset biases the diode conducting. If this
    // fails the divergence below would be testing the wrong regime.
    expect(vd).toBeGreaterThan(0.5);
    expect(vd).toBeLessThan(0.7);

    // Replicate acStampShockleyJunction's derivation from first principles
    // with the fixture's explicit params (vf=0.7, iRated=0.1, n=1, 25 C):
    // gd = Is * exp(min(vd, vSat)/(n*Vt)) / (n*Vt), Is from the rated point.
    const vt = thermalVoltage(AMBIENT_C);
    const vf = junctionForwardVoltage(0.7, AMBIENT_C);
    const is = shockleyIsFromVf(vf, 0.1, 1, vt);
    const vSat = Math.min(Math.max(40 * vt, vf + 5 * vt), 80 * vt);
    const gd = (is * Math.exp(Math.min(vd, vSat) / vt)) / vt;
    const gR = 1 / 1000;
    const expectedMag = gR / (gR + gd + AC_GMIN + NODE_RSHUNT_G);

    const out = ss.outputs[0]!;
    for (let i = 0; i < freqs.length; i++) {
      const mag = Math.hypot(out.re[i]!, out.im[i]!);
      expect(
        Math.abs(mag / expectedMag - 1),
        `linearized magnitude vs analytic gd divider at ${String(freqs[i])} Hz`,
      ).toBeLessThan(1e-9);
      // Purely resistive linearization: no phase shift at any frequency.
      expect(Math.abs(out.phaseDeg[i]!)).toBeLessThan(1e-6);
    }
    // The conducting diode must dominate the divider — this is what makes
    // the small-signal answer strongly attenuating (about -20 dB or lower).
    expect(gd).toBeGreaterThan(5 * gR);
  });

  it("large-signal DFT reflects clipping and departs from the linearization by > 6 dB", () => {
    const circuit = diodeClipperCircuit();
    const probe = new SimEngine();
    probe.load(circuit);
    const outNet = netIdForPin(probe, "rs", "b");

    const engine = new SimEngine();
    engine.load(diodeClipperCircuit());
    const ss = runSmallSignalAc(engine, {
      inputId: "sg",
      outputNetIds: [outNet],
      frequenciesHz: freqs,
    });

    const spec: AcSweepSpec = {
      kind: "ac-sweep",
      componentId: "sg",
      fromHz: 100,
      toHz: 1000,
      points: 3,
      outputs: [{ netId: outNet }],
    };
    const dft = runAcSweep(spec, circuit);
    expect(dft.fHz).toEqual(freqs);

    for (let i = 0; i < freqs.length; i++) {
      const dftDb = dft.outputs[0]!.magnitudeDb[i]!;
      const ssDb = ss.outputs[0]!.magnitudeDb[i]!;
      // EXPECTED DIVERGENCE — this is the point of the fixture. The DFT
      // fundamental keeps the unclipped negative half-swings (gain near
      // -6 dB); the linearization sees only the conducting-diode tangent
      // (about -20 dB or lower). If these ever agree, one of the two modes
      // stopped measuring what it documents.
      expect(
        dftDb - ssDb,
        `clipper divergence at ${String(freqs[i])} Hz (DFT ${String(dftDb)} dB vs small-signal ${String(ssDb)} dB)`,
      ).toBeGreaterThan(6);
      // The DFT itself must still show clipping-scale output, not the
      // linearized attenuation and not an unclipped unity pass-through.
      expect(dftDb).toBeGreaterThan(-12);
      expect(dftDb).toBeLessThan(-3);
    }
  });
});

// ─── 3. Operating-point interplay ────────────────────────────────────────────

describe("A5 OP interplay", () => {
  it("op-amp follower: regime-settled OP produces a converged unity-gain AC run", () => {
    // Premise: the op-amp carries committed regime state, so its OP must go
    // through the A3 regime-settle loop (at least one settle re-solve).
    const twin = new SimEngine();
    twin.load(followerCircuit());
    const op = twin.dcOperatingPoint();
    expect(op.converged).toBe(true);
    expect(op.regimeIterations).toBeGreaterThanOrEqual(1);

    const engine = new SimEngine();
    engine.load(followerCircuit());
    const outNet = netIdForPin(engine, "oa1", "1");
    const freqs = [100, 1000, 10000];
    const ss = runSmallSignalAc(engine, {
      inputId: "vin",
      outputNetIds: [outNet],
      frequenciesHz: freqs,
    });
    // A converging follower seed must be accepted by the ladder's first stage.
    expect(ss.opMethod).toBe("direct");

    const out = ss.outputs[0]!;
    for (let i = 0; i < freqs.length; i++) {
      expect(Number.isFinite(out.magnitudeDb[i]!)).toBe(true);
      // Closed-loop follower: |A/(1+A)| stays within 0.2 dB of unity for any
      // open-loop gain >= 1000 regardless of where the dominant pole sits,
      // so this bound is model-parameter-agnostic.
      expect(
        Math.abs(out.magnitudeDb[i]!),
        `follower gain at ${String(freqs[i])} Hz`,
      ).toBeLessThan(0.2);
      expect(Math.abs(out.phaseDeg[i]!)).toBeLessThan(5);
    }
  });

  it("impossible parallel sources: clean throw, then the engine is reusable bit-for-bit", () => {
    const engine = new SimEngine();
    engine.load(conflictingSourcesCircuit());
    expect(() =>
      runSmallSignalAc(engine, {
        inputId: "va",
        outputNetIds: [netIdForPin(engine, "va", "pos")],
        frequenciesHz: [1000],
      }),
    ).toThrow(/DC operating point did not converge/);
    // The failed ladder must not have advanced physical time.
    expect(Object.is(engine.simTime, 0)).toBe(true);

    // No state corruption: the survivor engine, handed a good circuit, must
    // step bitwise-identically to a fresh engine. Any leaked homotopy
    // control (dc mode, gmin overlay, source scale, Newton cap) would stamp
    // a different matrix and split the trajectories immediately.
    engine.load(recoveryRcCircuit());
    const fresh = new SimEngine();
    fresh.load(recoveryRcCircuit());
    expect(engine.lastConverged).toBe(true);
    expect(fresh.lastConverged).toBe(true);

    const drift: Array<[string, string, number | undefined, number | undefined]> = [];
    for (let step = 0; step < 20; step++) {
      engine.step(1e-5);
      fresh.step(1e-5);
      expect(Object.is(engine.simTime, fresh.simTime), `step ${String(step)} simTime`).toBe(true);
      expect(engine.lastConverged, `step ${String(step)} converged`).toBe(fresh.lastConverged);
      drift.push(
        ...collectNetVDrift(`post-failure step ${String(step)}`, engine.getNetV(), fresh.getNetV()),
      );
    }
    expect(drift).toEqual([]);
  });
});

// ─── 4. MCU rejection ────────────────────────────────────────────────────────

describe("A5 MCU rejection", () => {
  it("an arduino_uno circuit rejects with the documented error", () => {
    const engine = new SimEngine();
    engine.load(arduinoCircuit());
    expect(() =>
      runSmallSignalAc(engine, {
        inputId: "vs",
        outputNetIds: [netIdForPin(engine, "rl", "b")],
        frequenciesHz: [1000],
      }),
    ).toThrow(/circuit contains MCU board "uno" \(arduino_uno\).*large-signal ac-sweep/s);
  });
});

// ─── 5. Committed digital Thevenin default ──────────────────────────────────

describe("A5 committed digital output default — 74hc14", () => {
  it("output stage shows the documented Thevenin output-resistance rolloff into a capacitor", () => {
    // The engine derives the stage resistance from the catalog io_max:
    // Rout = max(GATE_R_OUT, min(1000, vSupply / (io_max * 4))). Replicate
    // that documented formula rather than reading the engine's value back.
    const ioMax = catalog.parts.find((p) => p.kind === "74hc14")?.electrical_specs?.io_max;
    expect(ioMax).toBeDefined();
    expect(ioMax!).toBeGreaterThan(0);
    const rOut = Math.max(GATE_R_OUT, Math.min(1000, 5 / (ioMax! * 4)));
    const g = 1 / rOut;
    const c = 1e-6;
    const fPole = g / (2 * Math.PI * c);

    const engine = new SimEngine();
    engine.load(hc14RolloffCircuit());
    // Two transient steps let the 15 ns propagation delay elapse so the
    // output stage's committed delay:1y slot exists before the OP holds it.
    engine.step(1e-4);
    engine.step(1e-4);

    const outNet = netIdForPin(engine, "u1", "1y");
    // Probe around the expected pole so the rolloff (not just the passband)
    // is what the assertions constrain.
    const freqs = [0.05, 0.2, 1, 4, 20].map((k) => k * fPole);
    const ss = runSmallSignalAc(engine, {
      inputId: "vcc",
      outputNetIds: [outNet],
      frequenciesHz: freqs,
    });

    // Bias premise: input LOW commits the output HIGH, so the stage ties to
    // the VCC rail and the OP output rests at the (lightly loaded) rail.
    expect(engine.getNetV()[outNet] ?? 0).toBeGreaterThan(4.5);

    const out = ss.outputs[0]!;
    for (let i = 0; i < freqs.length; i++) {
      const omega = 2 * Math.PI * freqs[i]!;
      // Out-node KCL: g to the (unit-driven) rail, C plus universal shunt to
      // ground -> H = g / (g + G_shunt + jwC).
      const denRe = g + NODE_RSHUNT_G;
      const denIm = omega * c;
      const expectedMag = g / Math.hypot(denRe, denIm);
      const expectedPhaseDeg = (-Math.atan2(denIm, denRe) * 180) / Math.PI;
      const mag = Math.hypot(out.re[i]!, out.im[i]!);
      expect(
        Math.abs(mag / expectedMag - 1),
        `Thevenin rolloff magnitude at ${String(freqs[i])} Hz`,
      ).toBeLessThan(1e-9);
      expect(
        phaseDistanceDeg(out.phaseDeg[i]!, expectedPhaseDeg),
        `Thevenin rolloff phase at ${String(freqs[i])} Hz`,
      ).toBeLessThan(1e-6);
    }

    // The pole point itself reads -3.01 dB: the classic single-pole marker.
    const atPole = out.magnitudeDb[2]!;
    expect(Math.abs(atPole - 20 * Math.log10(1 / Math.SQRT2))).toBeLessThan(1e-6);
  });
});

// ─── 6. Frequency-sweep pattern reuse ────────────────────────────────────────

describe("A5 sweep pattern reuse — 100 points over a 30-node ladder", () => {
  it("produces monotone-smooth results in bounded time", () => {
    const engine = new SimEngine();
    engine.load(rcLadderCircuit(30));
    // Fixture premise: this is genuinely a 30+ node system, so the sweep
    // exercises the shared sparse/dense pattern at meaningful size.
    expect(engine.matrixDimensions().nodeCount).toBeGreaterThanOrEqual(30);

    const outNet = netIdForPin(engine, "r30", "b");
    const freqs = logSweepValues(10, 10000, 100);

    const started = performance.now();
    const ss = runSmallSignalAc(engine, {
      inputId: "src",
      outputNetIds: [outNet],
      frequenciesHz: freqs,
    });
    const elapsedMs = performance.now() - started;
    // Pattern reuse contract: one symbolic analysis, then value-only
    // restamps. A per-point pattern rebuild would blow well past this.
    expect(elapsedMs).toBeLessThan(2000);

    const mags = ss.outputs[0]!.magnitudeDb;
    expect(mags.length).toBe(100);
    for (const m of mags) expect(Number.isFinite(m)).toBe(true);

    // A cascaded-RC all-pole low-pass is strictly monotone in |H|; any
    // non-monotone kink would be a per-point restamp/factorization artifact.
    for (let i = 1; i < mags.length; i++) {
      expect(
        mags[i]!,
        `monotone rolloff between ${String(freqs[i - 1])} and ${String(freqs[i])} Hz`,
      ).toBeLessThanOrEqual(mags[i - 1]! + 1e-9);
    }
    // Smoothness on the log grid: the curve's second difference stays small
    // (a smooth transfer function sampled at ~33 points/decade bends far
    // less than 1 dB per step); a single corrupted point would spike it.
    for (let i = 2; i < mags.length; i++) {
      const secondDiff = (mags[i]! - mags[i - 1]!) - (mags[i - 1]! - mags[i - 2]!);
      expect(
        Math.abs(secondDiff),
        `smoothness at ${String(freqs[i])} Hz`,
      ).toBeLessThan(1);
    }
  });
});
