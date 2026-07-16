/**
 * SPICE directive execution — engine-native equivalence (Wave A7)
 * ===============================================================
 *
 * runSpice (packages/simcore/src/sim/engine/spice/run.ts) promises to be a
 * THIN DRIVER: every analysis is a composition of the engine's existing
 * dcOperatingPoint(), step(), and runSmallSignalAc() over the SimCircuit the
 * parser produced, with one added convention — all reported voltages are
 * normalized to V(node) - V("0"). These tests lock that promise from both
 * sides:
 *
 *   1. Physics side: each analysis reproduces hand analysis / closed forms
 *      within the repo's established budgets (linear solves near-exact,
 *      trap-mode transients inside the 0.3% full-scale benchmark budget of
 *      trap-integration.test.ts).
 *   2. Equivalence side: a netlist and the structurally identical hand-built
 *      SimCircuit produce the same answers through the same engine entry
 *      points — bitwise for the transient (same component order, same wire
 *      chains, same call sequence means the same arithmetic), 1e-12 for AC,
 *      1e-9 for the nonlinear OP. The mapping layer must add nothing.
 *
 * The hand-built mirrors below replicate the parser's documented conversion
 * exactly (component order = card order, one wire chain per node in
 * first-appearance order, K merged into the first L's slot) so any
 * disagreement is a runner/parser bug, never an artifact of a different but
 * equivalent topology. Never widen a budget to make a failing runner pass —
 * report the bug.
 */

import { describe, expect, it } from "vitest";
import { runSmallSignalAc } from "../../../src/sim/engine/ac-analysis.js";
import { parseSpiceNetlist } from "../../../src/sim/engine/spice/netlist.js";
import { runSpice } from "../../../src/sim/engine/spice/run.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type SimComponent = SimCircuit["components"][number];
type SimWire = SimCircuit["wires"][number];

// ─── Hand-built circuit helpers (trap-integration conventions) ───────────────

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

function capacitor(id: string, capacitance: number): SimComponent {
  return {
    id,
    kind: "capacitor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { capacitance },
  };
}

/**
 * Engine loaded exactly the way runSpice's freshEngine loads: integration
 * method set BEFORE load so the post-load discontinuity anchor is armed
 * identically — required for the bitwise transient comparison.
 */
function loadNative(circuit: SimCircuit, method: "be" | "trap" = "trap"): SimEngine {
  const engine = new SimEngine();
  engine.setIntegrationMethod(method);
  engine.load(circuit);
  return engine;
}

/** Net id for a pin, throwing so a typo'd mirror fails loudly. */
function netOf(engine: SimEngine, componentId: string, pinId: string): string {
  const netId = engine.getNetIdForPin(componentId, pinId);
  if (netId === undefined) {
    throw new Error(`no net for ${componentId}.${pinId} — mirror circuit is wrong`);
  }
  return netId;
}

/** Worst absolute deviation from a closed form over a recorded trace. */
function maxAbsError(
  timeS: number[],
  values: number[],
  ideal: (t: number) => number,
  fromIndex = 0,
): number {
  let worst = 0;
  for (let i = fromIndex; i < timeS.length; i++) {
    worst = Math.max(worst, Math.abs(values[i] - ideal(timeS[i])));
  }
  return worst;
}

// ─── .op — resistive divider vs hand analysis ────────────────────────────────

describe("runSpice .op — three-node divider vs hand analysis", () => {
  /**
   * 5 V across R1=1k, R2=2k, R3=2k in series: I = 1 mA exactly, so
   * V(1)=5, V(2)=4, V(3)=2. The solve is one linear LU pass; the only
   * modeled deviation is the engine's disclosed 1e-12 S universal node
   * shunt, worth <= ~1e-8 V through kohm Thevenin impedances. Budget 1e-7.
   */
  const NETLIST = [
    "three node divider",
    "v1 1 0 5",
    "r1 1 2 1k",
    "r2 2 3 2k",
    "r3 3 0 2k",
    ".op",
    ".end",
  ].join("\n");

  it("reports the exact divider voltages normalized to node 0", () => {
    const result = runSpice(NETLIST);
    const op = result.op;
    expect(op).toBeDefined();
    if (!op) return;
    expect(Math.abs(op.nodeVoltages["1"] - 5)).toBeLessThanOrEqual(1e-7);
    expect(Math.abs(op.nodeVoltages["2"] - 4)).toBeLessThanOrEqual(1e-7);
    expect(Math.abs(op.nodeVoltages["3"] - 2)).toBeLessThanOrEqual(1e-7);
    expect(op.nodeVoltages["0"]).toBe(0);
  });

  it("reports the series current on every element", () => {
    const op = runSpice(NETLIST).op;
    if (!op) throw new Error(".op result missing");
    // Resistor convention is pin0 -> pin1 through the element: node 1 is
    // r1's "a" pin and sits 1 V above node 2, so the current is +1 mA.
    expect(Math.abs(op.elementCurrents.r1 - 1e-3)).toBeLessThanOrEqual(1e-9);
    // The source current sign convention is the engine's branch convention;
    // magnitude is the invariant hand analysis pins down.
    expect(Math.abs(Math.abs(op.elementCurrents.v1) - 1e-3)).toBeLessThanOrEqual(1e-9);
  });
});

// ─── .tran — RC charge curve and trace shape ─────────────────────────────────

describe("runSpice .tran — RC charge against the closed form", () => {
  // tau = R*C = 1 ms; h = tau/100 matches the trap-integration benchmark
  // fixture, whose established budget is 0.3% of full scale (15 mV on 5 V).
  const R = 1000;
  const C = 1e-6;
  const TAU = R * C;
  const H = TAU / 100;
  const TSTOP = 5 * TAU;
  // .ic v(out)=0 forces the charge transient: without it SPICE (and the
  // runner) start .tran from the DC operating point, where the cap is
  // already full and the trace is flat.
  const NETLIST = [
    "rc charge",
    "v1 in 0 5",
    "r1 in out 1k",
    "c1 out 0 1u",
    ".ic v(out)=0",
    `.tran ${String(H)} ${String(TSTOP)}`,
    ".end",
  ].join("\n");

  it("matches v(t) = 5(1 - exp(-t/tau)) within the 0.3% full-scale budget", () => {
    const tran = runSpice(NETLIST).tran;
    if (!tran) throw new Error(".tran result missing");
    const ideal = (t: number): number => 5 * (1 - Math.exp(-t / TAU));
    expect(maxAbsError(tran.timeS, tran.nodeVoltages.out, ideal)).toBeLessThanOrEqual(0.015);
    // Steady-state spot check: after 5 tau the curve sits at 4.966 V.
    const last = tran.nodeVoltages.out[tran.timeS.length - 1];
    expect(Math.abs(last - ideal(TSTOP))).toBeLessThanOrEqual(0.015);
  });

  it("produces a well-formed trace: monotone time, all nodes present, aligned lengths", () => {
    const tran = runSpice(NETLIST).tran;
    if (!tran) throw new Error(".tran result missing");
    expect(tran.timeS.length).toBe(501);
    expect(tran.timeS[0]).toBe(0);
    for (let i = 1; i < tran.timeS.length; i++) {
      expect(tran.timeS[i]).toBeGreaterThan(tran.timeS[i - 1]);
    }
    for (const node of ["in", "out", "0"]) {
      expect(tran.nodeVoltages).toHaveProperty(node);
      expect(tran.nodeVoltages[node].length).toBe(tran.timeS.length);
    }
    // The driven node holds the source value for the whole run.
    for (const v of tran.nodeVoltages.in) {
      expect(Math.abs(v - 5)).toBeLessThanOrEqual(1e-3);
    }
  });
});

// ─── .tran window semantics — tstop is a hard ceiling ────────────────────────

describe("runSpice .tran window — samples never pass tstop", () => {
  // Static divider: the grid is what is under test, not the physics.
  const deckWith = (tran: string): string =>
    ["tran window", "v1 in 0 5", "r1 in 0 1k", tran, ".end"].join("\n");

  it("truncates a non-multiple tstop to the last on-grid sample and warns", () => {
    // floor(10u / 4u) = 2 steps: samples at 0, 4u, 8u. A round() here would
    // take 3 steps and report a sample at 12u — PAST the requested stop,
    // which ngspice never does.
    const result = runSpice(deckWith(".tran 4u 10u"));
    const tran = result.tran;
    if (!tran) throw new Error(".tran result missing");
    expect(tran.timeS.length).toBe(3);
    expect(Math.abs(tran.timeS[2] - 8e-6)).toBeLessThanOrEqual(1e-17);
    for (const t of tran.timeS) expect(t).toBeLessThanOrEqual(1e-5);
    expect(result.warnings.some((w) => /not a whole multiple of tstep/.test(w))).toBe(true);
  });

  it("never rounds a short window up either", () => {
    // 10u / 3u = 3.33: the window ends at 9u (floor), not 12u (any rounding
    // that carries the fraction up).
    const result = runSpice(deckWith(".tran 3u 10u"));
    const tran = result.tran;
    if (!tran) throw new Error(".tran result missing");
    expect(tran.timeS.length).toBe(4);
    for (const t of tran.timeS) expect(t).toBeLessThanOrEqual(1e-5);
    expect(result.warnings.some((w) => /not a whole multiple of tstep/.test(w))).toBe(true);
  });

  it("keeps an exact-multiple window exact, with no truncation warning", () => {
    const result = runSpice(deckWith(".tran 2u 10u"));
    const tran = result.tran;
    if (!tran) throw new Error(".tran result missing");
    expect(tran.timeS.length).toBe(6);
    // The f64 quotient of decimal literals may sit just under the integer;
    // the runner's relative guard must still read this as exact.
    expect(Math.abs(tran.timeS[5] - 1e-5)).toBeLessThanOrEqual(1e-17);
    expect(result.warnings).toEqual([]);
  });
});

// ─── .dc — diode knee, every point an independent operating point ────────────

describe("runSpice .dc — source sweep across a diode", () => {
  // 100 ohm ballast so the post-knee slope is visibly resistor-limited.
  const dcNetlist = [
    "diode dc sweep",
    "v1 in 0 0",
    "r1 in out 100",
    "d1 out 0 dm",
    ".model dm d is=1e-12 n=1",
    ".dc v1 0 1 0.05",
    ".end",
  ].join("\n");

  const opNetlistAt = (volts: number): string =>
    [
      "diode op point",
      `v1 in 0 ${String(volts)}`,
      "r1 in out 100",
      "d1 out 0 dm",
      ".model dm d is=1e-12 n=1",
      ".op",
      ".end",
    ].join("\n");

  it("shows the forward knee: unity slope below, resistor-clamped above", () => {
    const dc = runSpice(dcNetlist).dc;
    if (!dc) throw new Error(".dc result missing");
    expect(dc.source).toBe("v1");
    expect(dc.sweepValues.length).toBe(21);
    expect(dc.sweepValues[0]).toBe(0);
    const out = dc.nodeVoltages.out;
    // Below the knee the diode conducts ~nA: no drop across R, out tracks in.
    expect(Math.abs(out[4] - dc.sweepValues[4])).toBeLessThanOrEqual(1e-3);
    const lowSlope = (out[4] - out[3]) / (dc.sweepValues[4] - dc.sweepValues[3]);
    expect(lowSlope).toBeGreaterThan(0.99);
    // Above the knee the exponential pins the diode voltage: at 1 V the
    // drop solves near n*Vt*ln(I/Is) ~ 0.57 V and the incremental slope
    // collapses to rd/(R + rd) << 1.
    const top = out[out.length - 1];
    expect(top).toBeGreaterThan(0.4);
    expect(top).toBeLessThan(0.75);
    const highSlope = (out[out.length - 1] - out[out.length - 2])
      / (dc.sweepValues[out.length - 1] - dc.sweepValues[out.length - 2]);
    expect(highSlope).toBeLessThan(0.2);
  });

  it("reproduces an independently-run .op at every sweep value", () => {
    const dc = runSpice(dcNetlist).dc;
    if (!dc) throw new Error(".dc result missing");
    // Budget: the sweep warm-starts from the previous point while the
    // independent .op cold-starts from the load seed, so the two answers
    // are distinct converged Newton iterates. Newton's mixed stopping
    // criterion (newton.ts defaults: 1e-6 V absolute + 1e-3 relative)
    // bounds each solve's accepted delta, and two independently converged
    // answers can differ by up to the sum of both entitlements. Observed
    // agreement is ~2e-6 near the knee — orders below the mV-scale knee
    // structure the sweep exists to resolve.
    for (let i = 0; i < dc.sweepValues.length; i++) {
      const op = runSpice(opNetlistAt(dc.sweepValues[i])).op;
      if (!op) throw new Error(".op result missing");
      const outBudget = 2 * (1e-6 + 1e-3 * Math.abs(op.nodeVoltages.out));
      const inBudget = 2 * (1e-6 + 1e-3 * Math.abs(op.nodeVoltages.in));
      expect(Math.abs(dc.nodeVoltages.out[i] - op.nodeVoltages.out)).toBeLessThanOrEqual(outBudget);
      expect(Math.abs(dc.nodeVoltages.in[i] - op.nodeVoltages.in)).toBeLessThanOrEqual(inBudget);
    }
  });
});

// ─── .ac — RC lowpass vs native runSmallSignalAc ─────────────────────────────

describe("runSpice .ac — RC lowpass equals native runSmallSignalAc", () => {
  const AC_NETLIST = [
    "rc lowpass ac",
    "v1 in 0 0 ac 1",
    "r1 in out 1k",
    "c1 out 0 1u",
    ".ac dec 10 10 100k",
    ".end",
  ].join("\n");

  /**
   * The parser's documented conversion of the netlist above: components in
   * card order; one wire chain per node in first-appearance order
   * (in: v1.pos->r1.a; 0: v1.neg->c1.b; out: r1.b->c1.a). Structural
   * identity is what makes the comparison a test of the RUNNER alone.
   */
  function nativeLowpass(): SimCircuit {
    return {
      components: [voltageSource("v1", 0), resistor("r1", 1000), capacitor("c1", 1e-6)],
      wires: [
        wire("v1", "pos", "r1", "a"),
        wire("v1", "neg", "c1", "b"),
        wire("r1", "b", "c1", "a"),
      ],
    };
  }

  it("matches the native driver bitwise-close (1e-12) at every frequency", () => {
    const ac = runSpice(AC_NETLIST).ac;
    if (!ac) throw new Error(".ac result missing");
    expect(ac.inputId).toBe("v1");
    expect(ac.inputMagnitude).toBe(1);
    expect(ac.frequenciesHz.length).toBe(41);
    expect(ac.frequenciesHz[0]).toBe(10);

    const engine = loadNative(nativeLowpass());
    const native = runSmallSignalAc(engine, {
      inputId: "v1",
      outputNetIds: [netOf(engine, "c1", "a")],
      frequenciesHz: ac.frequenciesHz,
    });
    const trace = ac.nodeResponses.out;
    for (let i = 0; i < ac.frequenciesHz.length; i++) {
      // Unit AC magnitude and a grounded node "0" make the runner's
      // normalization/scaling the identity, so a thin driver must agree to
      // floating-point noise. 1e-12 on O(1) responses.
      expect(Math.abs(trace.re[i] - native.outputs[0].re[i])).toBeLessThanOrEqual(1e-12);
      expect(Math.abs(trace.im[i] - native.outputs[0].im[i])).toBeLessThanOrEqual(1e-12);
      expect(Math.abs(trace.magnitude[i] - Math.hypot(trace.re[i], trace.im[i])))
        .toBeLessThanOrEqual(1e-15);
    }
  });

  it("matches the first-order lowpass closed form", () => {
    const ac = runSpice(AC_NETLIST).ac;
    if (!ac) throw new Error(".ac result missing");
    const fc = 1 / (2 * Math.PI * 1000 * 1e-6);
    for (let i = 0; i < ac.frequenciesHz.length; i++) {
      const x = ac.frequenciesHz[i] / fc;
      const idealMag = 1 / Math.sqrt(1 + x * x);
      const idealPhase = (-Math.atan(x) * 180) / Math.PI;
      // Linear complex solve: the only deviation is the 1e-12 S node shunt
      // (~1e-9 relative through 1 kohm). 1e-6 relative is generous.
      expect(Math.abs(ac.nodeResponses.out.magnitude[i] - idealMag))
        .toBeLessThanOrEqual(1e-6 * idealMag);
      expect(Math.abs(ac.nodeResponses.out.phaseDeg[i] - idealPhase)).toBeLessThanOrEqual(1e-4);
    }
  });
});

// ─── Netlist vs native — CE amplifier operating point ────────────────────────

describe("runSpice netlist-vs-native — CE amplifier OP equivalence", () => {
  const CE_NETLIST = [
    "ce amplifier",
    "vcc vcc 0 5",
    "rc vcc c 2.2k",
    "rb vcc b 470k",
    "q1 c b 0 qnpn",
    ".model qnpn npn is=1e-15 bf=150",
    ".op",
    ".end",
  ].join("\n");

  /**
   * Hand-built mirror of the parser's conversion. BJT params spell out the
   * documented .model NPN mapping (IS->Is, BF->betaF) plus the SPICE
   * defaults the mapper fills in (BR=1, NF=NR=1, VAF=0 sentinel). Wire
   * chains follow node first-appearance order: vcc (vcc.pos, rc.a, rb.a),
   * 0 (vcc.neg, q1.e), c (rc.b, q1.c), b (rb.b, q1.b).
   */
  function nativeCeAmp(): SimCircuit {
    return {
      components: [
        voltageSource("vcc", 5),
        resistor("rc", 2200),
        resistor("rb", 470000),
        {
          id: "q1",
          kind: "bjt_npn",
          pins: [{ id: "c" }, { id: "b" }, { id: "e" }],
          params: { Is: 1e-15, betaF: 150, betaR: 1, nF: 1, nR: 1, earlyVoltage: 0 },
        },
      ],
      wires: [
        wire("vcc", "pos", "rc", "a"),
        wire("rc", "a", "rb", "a"),
        wire("vcc", "neg", "q1", "e"),
        wire("rc", "b", "q1", "c"),
        wire("rb", "b", "q1", "b"),
      ],
    };
  }

  it("produces identical OP node voltages through nodeNets to 1e-9", () => {
    const op = runSpice(CE_NETLIST).op;
    if (!op) throw new Error(".op result missing");

    const engine = loadNative(nativeCeAmp());
    const nativeOp = engine.dcOperatingPoint();
    expect(nativeOp.converged).toBe(true);
    // The engine grounds vcc.neg (node 0's net), so V("0") is already the
    // reference; subtract it anyway to apply the runner's exact convention.
    const vZero = nativeOp.netV[netOf(engine, "vcc", "neg")] ?? 0;
    const nativeAt = (componentId: string, pinId: string): number =>
      (nativeOp.netV[netOf(engine, componentId, pinId)] ?? 0) - vZero;

    // Identical structure means identical Newton arithmetic: 1e-9 leaves
    // room only for genuinely divergent mapping, not for solver noise.
    expect(Math.abs(op.nodeVoltages.vcc - nativeAt("vcc", "pos"))).toBeLessThanOrEqual(1e-9);
    expect(Math.abs(op.nodeVoltages.c - nativeAt("q1", "c"))).toBeLessThanOrEqual(1e-9);
    expect(Math.abs(op.nodeVoltages.b - nativeAt("q1", "b"))).toBeLessThanOrEqual(1e-9);
    expect(op.nodeVoltages["0"]).toBe(0);

    // Sanity anchor so the equivalence is not vacuous: the bias point is in
    // the forward-active region (VBE ~ 0.6-0.75 V, collector pulled down but
    // not saturated).
    expect(op.nodeVoltages.b).toBeGreaterThan(0.5);
    expect(op.nodeVoltages.b).toBeLessThan(0.85);
    expect(op.nodeVoltages.c).toBeGreaterThan(op.nodeVoltages.b);
    expect(op.nodeVoltages.c).toBeLessThan(4.5);
  });
});

// ─── Netlist vs native — K transformer bitwise transient ────────────────────

describe("runSpice K coupling — netlist equals native coupled_inductor bitwise", () => {
  const K_NETLIST = [
    "transformer",
    "v1 in 0 sin(0 1 1k)",
    "r1 in p1 100",
    "l1 p1 0 10m",
    "l2 s1 0 1m",
    "k1 l1 l2 0.8",
    "rl s1 0 1k",
    ".tran 2u 100u",
    ".end",
  ].join("\n");

  /**
   * The parser's documented K rewrite: l1 and l2 merge into ONE
   * coupled_inductor at l1's slot (id k1, a-pins are the dots), l2's slot
   * vanishes, and node attachments are rewritten in place. Node chains in
   * first-appearance order: in (v1.pos, r1.a), 0 (v1.neg, k1.b1, k1.b2,
   * rl.b), p1 (r1.b, k1.a1), s1 (k1.a2, rl.a). The SIN source is the
   * parser's signal_gen with rSource=0 and explicit enabled.
   */
  function nativeTransformer(): SimCircuit {
    return {
      components: [
        {
          id: "v1",
          kind: "signal_gen",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: {
            waveform: "sine",
            offset: 0,
            amplitude: 1,
            frequency: 1000,
            delay: 0,
            enabled: 1,
            rSource: 0,
          },
        },
        resistor("r1", 100),
        {
          id: "k1",
          kind: "coupled_inductor",
          pins: [{ id: "a1" }, { id: "b1" }, { id: "a2" }, { id: "b2" }],
          params: { l1: 0.01, l2: 0.001, k: 0.8 },
        },
        resistor("rl", 1000),
      ],
      wires: [
        wire("v1", "pos", "r1", "a"),
        wire("v1", "neg", "k1", "b1"),
        wire("k1", "b1", "k1", "b2"),
        wire("k1", "b2", "rl", "b"),
        wire("r1", "b", "k1", "a1"),
        wire("k1", "a2", "rl", "a"),
      ],
    };
  }

  it("replays a 50-step transient bitwise (Object.is per sample)", () => {
    const tran = runSpice(K_NETLIST).tran;
    if (!tran) throw new Error(".tran result missing");
    expect(tran.timeS.length).toBe(51);

    // Native replay of exactly what runTranAnalysis does for a no-IC deck:
    // fresh trap engine, DC operating point, record, then fixed 2 us steps.
    const engine = loadNative(nativeTransformer());
    const op = engine.dcOperatingPoint();
    expect(op.converged).toBe(true);
    const zeroNet = netOf(engine, "v1", "neg");
    const nets = new Map<string, string>([
      ["in", netOf(engine, "v1", "pos")],
      ["p1", netOf(engine, "k1", "a1")],
      ["s1", netOf(engine, "k1", "a2")],
      ["0", zeroNet],
    ]);
    const native = new Map<string, number[]>([...nets.keys()].map((n) => [n, []]));
    const record = (): void => {
      const netV = engine.getNetV();
      const vZero = netV[zeroNet] ?? 0;
      for (const [node, netId] of nets) {
        native.get(node)?.push((netV[netId] ?? 0) - vZero);
      }
    };
    record();
    for (let i = 1; i <= 50; i++) {
      engine.step(2e-6);
      expect(engine.lastConverged).toBe(true);
      record();
    }

    for (const [node, samples] of native) {
      const spiceTrace = tran.nodeVoltages[node];
      expect(spiceTrace.length).toBe(samples.length);
      for (let i = 0; i < samples.length; i++) {
        expect(Object.is(spiceTrace[i], samples[i])).toBe(true);
      }
    }
    // Guard against a vacuous all-zeros comparison: the mutual term must
    // have driven the (galvanically isolated) secondary.
    const s1Peak = Math.max(...tran.nodeVoltages.s1.map(Math.abs));
    expect(s1Peak).toBeGreaterThan(1e-3);
  });
});

// ─── SIN negative VA — |VA| + 180-degree phase, not a dead source ────────────

describe("runSpice SIN negative VA — exact inversion, never a nulled source", () => {
  // The engine's signal_gen clamps negative amplitudes to 0, so before the
  // parser mapped VA < 0 onto |VA| with a 180-degree phase this deck ran a
  // FLAT 0 V source with no warning while ngspice ran the inverted sine.
  const deckFor = (va: number): string =>
    [
      "sin polarity",
      `v1 in 0 sin(0 ${String(va)} 1k)`,
      "r1 in out 1k",
      "r2 out 0 1k",
      ".tran 50u 1m",
      ".end",
    ].join("\n");

  it("produces the sample-wise negation of the positive-VA deck", () => {
    const neg = runSpice(deckFor(-1)).tran;
    const pos = runSpice(deckFor(1)).tran;
    if (!neg || !pos) throw new Error(".tran result missing");
    expect(neg.timeS).toEqual(pos.timeS);
    // sin(x + pi) vs -sin(x) differ only by the f64 rounding of the phase
    // addition (~1e-15 absolute); the linear divider scales that, nothing
    // more. 1e-12 leaves room only for a genuinely wrong mapping.
    let peak = 0;
    for (let i = 0; i < neg.timeS.length; i++) {
      expect(Math.abs(neg.nodeVoltages.out[i] + pos.nodeVoltages.out[i])).toBeLessThanOrEqual(1e-12);
      peak = Math.max(peak, Math.abs(neg.nodeVoltages.out[i]));
    }
    // Guard against a vacuously-passing dead source (0 == -0 everywhere):
    // the divider must actually swing to half the 1 V amplitude.
    expect(peak).toBeGreaterThan(0.4);
  });
});

// ─── .temp — environment plumbing and diode tempco direction ─────────────────

describe("runSpice .temp — flows into environment.temperatureC", () => {
  const diodeNetlistAt = (tempCard: string): string =>
    [
      "diode temperature",
      "v1 in 0 5",
      "r1 in d 1k",
      "d1 d 0 dm",
      ".model dm d",
      tempCard,
      ".op",
      ".end",
    ].join("\n");

  it("parses .temp 85 into the circuit environment", () => {
    const parsed = parseSpiceNetlist(diodeNetlistAt(".temp 85"));
    expect(parsed.circuit.environment?.temperatureC).toBe(85);
  });

  it("shifts the diode forward drop down as the junction heats", () => {
    const hot = runSpice(diodeNetlistAt(".temp 85")).op;
    const nominal = runSpice(diodeNetlistAt("* no .temp card")).op;
    if (!hot || !nominal) throw new Error(".op result missing");
    // Documented tempco direction: saturation current rises steeply with
    // temperature, so the solved forward drop FALLS (~ -2 mV/C for silicon;
    // 60 C above nominal is well over 50 mV). Both drops stay in the
    // physical forward window so the comparison is between real solutions.
    expect(nominal.nodeVoltages.d).toBeGreaterThan(0.4);
    expect(nominal.nodeVoltages.d).toBeLessThan(0.8);
    expect(hot.nodeVoltages.d).toBeLessThan(nominal.nodeVoltages.d - 0.05);
  });
});

// ─── .ic / IC= — transients start from the stated state, not zero ────────────

describe("runSpice initial conditions — .tran starts from the IC", () => {
  const R = 1000;
  const C = 1e-6;
  const TAU = R * C;

  it(".ic v(out)=2 clamps the release point and the trajectory follows it", () => {
    const tran = runSpice(
      [
        "ic clamp rc",
        "v1 in 0 5",
        "r1 in out 1k",
        "c1 out 0 1u",
        ".ic v(out)=2",
        ".tran 10u 2m",
        ".end",
      ].join("\n"),
    ).tran;
    if (!tran) throw new Error(".tran result missing");
    // The t=0 sample IS the clamped operating point (ideal source to
    // ground), carried through the release reload by cap state.
    expect(Math.abs(tran.nodeVoltages.out[0] - 2)).toBeLessThanOrEqual(1e-3);
    // Released trajectory: v(t) = 5 - 3*exp(-t/tau). Same 0.3% full-scale
    // budget as the charge-curve fixture (BE-anchored first step included).
    const ideal = (t: number): number => 5 - 3 * Math.exp(-t / TAU);
    expect(maxAbsError(tran.timeS, tran.nodeVoltages.out, ideal)).toBeLessThanOrEqual(0.015);
  });

  it("element IC=2 seeds the cap UIC-style: first step continues from 2 V, not 0 and not the OP", () => {
    const tran = runSpice(
      [
        "uic rc",
        "v1 in 0 5",
        "r1 in out 1k",
        "c1 out 0 1u ic=2",
        ".tran 10u 1m",
        ".end",
      ].join("\n"),
    ).tran;
    if (!tran) throw new Error(".tran result missing");
    // UIC mode skips the operating point: the t=0 sample is the engine's
    // documented load-seed solve near zero — NOT the OP's fully-charged 5 V.
    expect(Math.abs(tran.nodeVoltages.out[0])).toBeLessThanOrEqual(1e-3);
    // From the first real step the seeded 2 V state governs: one h=tau/100
    // step from 2 V lands at 2 + 3*(1 - exp(-h/tau)) ~ 2.03 V. A zero-state
    // start would read ~0.05 V and an OP start ~5 V, so the window below
    // uniquely identifies the seeded trajectory.
    expect(tran.nodeVoltages.out[1]).toBeGreaterThan(1.9);
    expect(tran.nodeVoltages.out[1]).toBeLessThan(2.15);
    // And the whole tail follows the from-2V closed form within budget.
    const ideal = (t: number): number => 5 - 3 * Math.exp(-t / TAU);
    expect(maxAbsError(tran.timeS, tran.nodeVoltages.out, ideal, 1)).toBeLessThanOrEqual(0.015);
  });
});

// ─── Reference independence — node 0 away from the engine's ground pick ──────

describe("runSpice reference normalization — source neg terminal off node 0", () => {
  // The engine grounds v1.neg (node b), NOT node 0: raw engine voltages
  // would read b=0, 0=+2.5, a=+5. The runner's normalization must re-anchor
  // every analysis output to V(0)=0, giving a=+2.5 and b=-2.5.
  const NETLIST = [
    "floating reference",
    "v1 a b 5 ac 1",
    "r1 a 0 1k",
    "r2 0 b 1k",
    ".op",
    ".tran 10u 50u",
    ".dc v1 0 5 2.5",
    ".ac lin 2 100 1k",
    ".end",
  ].join("\n");

  it("reports V(0)=0 and symmetric divider voltages in every analysis output", () => {
    const result = runSpice(NETLIST);
    const { op, tran, dc, ac } = result;
    if (!op || !tran || !dc || !ac) throw new Error("expected all four analyses to run");

    // .op — exact zero by construction, +/-2.5 within the linear budget.
    expect(op.nodeVoltages["0"]).toBe(0);
    expect(Math.abs(op.nodeVoltages.a - 2.5)).toBeLessThanOrEqual(1e-7);
    expect(Math.abs(op.nodeVoltages.b - -2.5)).toBeLessThanOrEqual(1e-7);
    expect(Math.abs(op.nodeVoltages.a - op.nodeVoltages.b - 5)).toBeLessThanOrEqual(1e-7);

    // .tran — a static circuit: node 0 pinned at exactly 0 on every sample.
    for (let i = 0; i < tran.timeS.length; i++) {
      expect(tran.nodeVoltages["0"][i]).toBe(0);
      expect(Math.abs(tran.nodeVoltages.a[i] - 2.5)).toBeLessThanOrEqual(1e-6);
    }

    // .dc — each point re-anchored: at v1 = 2.5 the halves read +/-1.25.
    expect(dc.sweepValues).toEqual([0, 2.5, 5]);
    for (let i = 0; i < dc.sweepValues.length; i++) {
      expect(dc.nodeVoltages["0"][i]).toBe(0);
      expect(Math.abs(dc.nodeVoltages.a[i] - dc.sweepValues[i] / 2)).toBeLessThanOrEqual(1e-6);
      expect(Math.abs(dc.nodeVoltages.b[i] + dc.sweepValues[i] / 2)).toBeLessThanOrEqual(1e-6);
    }

    // .ac — the complex normalization subtracts node 0's own trace from
    // itself, so its response is exactly zero; the halves split the unit
    // drive resistively (+0.5 and -0.5 real, no reactance in the circuit).
    for (let i = 0; i < ac.frequenciesHz.length; i++) {
      expect(ac.nodeResponses["0"].re[i]).toBe(0);
      expect(ac.nodeResponses["0"].im[i]).toBe(0);
      expect(ac.nodeResponses["0"].magnitude[i]).toBe(0);
      expect(Math.abs(ac.nodeResponses.a.re[i] - 0.5)).toBeLessThanOrEqual(1e-6);
      expect(Math.abs(ac.nodeResponses.a.im[i])).toBeLessThanOrEqual(1e-6);
      expect(Math.abs(ac.nodeResponses.b.re[i] + 0.5)).toBeLessThanOrEqual(1e-6);
    }
  });
});
