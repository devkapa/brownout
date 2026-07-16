/**
 * Wave A6 — extended-semiconductor physics contracts
 * ==================================================
 *
 * JFET, SCR, triac, optocoupler, and analog-switch models from the
 * model-families cohort, tested against first-principles physics rather
 * than implementation snapshots (the dc-homotopy-pathological.test.ts
 * rule): every tolerance below encodes a datasheet or textbook identity —
 * Shichman-Hodges transfer, thyristor latch/dropout thresholds, CTR
 * current transfer, transmission-gate resistances — so a regression here
 * means the user-visible device physics changed.
 *
 * Determinism assertions are bit-identity, mirroring
 * dc-homotopy-pathological.test.ts and ac-isolation.test.ts: the engine is
 * deterministic code on identical inputs, so any drift is a hidden
 * ordering or state leak, not legitimate numeric fuzz.
 */

import { describe, expect, it } from "vitest";
import { runSmallSignalAc } from "../../../src/sim/engine/ac-analysis.js";
import { shockleyIsFromVf } from "../../../src/sim/engine/elements.js";
import {
  N_DIODE,
  NODE_RSHUNT_G,
  SimEngine,
  thermalVoltage,
  type SimCircuit,
} from "../../../src/sim/engine/sim-engine.js";

type SimComponent = SimCircuit["components"][number];
type SimWire = SimCircuit["wires"][number];

// ─── Circuit-building helpers (dc-homotopy-pathological.test.ts pattern) ────

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

function pulseSource(
  id: string,
  params: Record<string, number>,
): SimComponent {
  return {
    id,
    kind: "pulse_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params,
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

function netIdFor(engine: SimEngine, compId: string, pinId: string): string {
  const netId = engine.getNetIdForPin(compId, pinId);
  if (!netId) throw new Error(`No net for ${compId}.${pinId}`);
  return netId;
}

function expectRel(actual: number, expected: number, relTol: number, label: string): void {
  expect(
    Math.abs(actual - expected),
    `${label}: got ${String(actual)}, expected ${String(expected)} within `
    + `${String(relTol * 100)}%`,
  ).toBeLessThanOrEqual(Math.abs(expected) * relTol);
}

// ─── Bitwise frame capture (condensed from ac-isolation.test.ts) ────────────

/** Exact string token for a double; -0 tagged so a sign flip cannot hide. */
function numToken(v: number): string {
  return Object.is(v, -0) ? "-0" : String(v);
}

function canon(value: unknown): unknown {
  if (typeof value === "number") return numToken(value);
  if (value === null || value === undefined || typeof value !== "object") {
    return value ?? null;
  }
  if (value instanceof Map) {
    return [...value.entries()]
      .map(([k, v]) => [String(k), canon(v)] as const)
      .sort((a, b) => (a[0] < b[0] ? -1 : a[0] > b[0] ? 1 : 0));
  }
  if (ArrayBuffer.isView(value)) {
    return [...(value as unknown as Iterable<number>)].map(numToken);
  }
  if (Array.isArray(value)) return value.map(canon);
  const record = value as Record<string, unknown>;
  return Object.keys(record)
    .sort()
    .map((key) => [key, canon(record[key])]);
}

/**
 * Full saveState() snapshot with lastSolveUs zeroed — wall-clock timing is
 * the only non-deterministic value in the snapshot.
 */
function serializeFrame(engine: SimEngine): string {
  const snapshot = engine.saveState();
  if (snapshot.solverDiagnostics) {
    snapshot.solverDiagnostics = { ...snapshot.solverDiagnostics, lastSolveUs: 0 };
  }
  return JSON.stringify(canon(snapshot));
}

/** Fail with the first differing region instead of dumping whole frames. */
function expectIdenticalFrames(label: string, expected: string[], actual: string[]): void {
  expect(actual.length, `${label}: frame count`).toBe(expected.length);
  for (let i = 0; i < expected.length; i++) {
    if (actual[i] === expected[i]) continue;
    const a = expected[i]!;
    const b = actual[i]!;
    let at = 0;
    while (at < a.length && at < b.length && a[at] === b[at]) at++;
    const from = Math.max(0, at - 120);
    expect.fail(
      `${label}: frame ${i + 1} of ${expected.length} diverged.\n`
        + `expected ...${a.slice(from, at + 160)}...\n`
        + `actual   ...${b.slice(from, at + 160)}...`,
    );
  }
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

// ─── JFET ────────────────────────────────────────────────────────────────────

// Datasheet-style n-JFET: vto = -2 V, idss = 10 mA (2N5457 magnitudes), so
// beta = idss/vto^2 = 2.5 mA/V^2 by the model's own construction.
const JFET_VTO = -2;
const JFET_IDSS = 0.01;
const JFET_BETA = JFET_IDSS / (JFET_VTO * JFET_VTO);

/** 10 V rail on the drain, source grounded, gate held at vGate by a source. */
function jfetBiasCircuit(vGate: number, lambda: number): SimCircuit {
  return {
    components: [
      voltageSource("vdd", 10),
      voltageSource("vg", vGate),
      {
        id: "j1",
        kind: "njfet",
        pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
        params: { vto: JFET_VTO, idss: JFET_IDSS, lambda },
      },
    ],
    wires: [
      wire("vdd", "pos", "j1", "d"),
      wire("j1", "s", "vdd", "neg"),
      wire("vg", "pos", "j1", "g"),
      wire("vg", "neg", "vdd", "neg"),
    ],
    environment: { temperatureC: 25 },
  };
}

describe("njfet — Shichman-Hodges channel physics", () => {
  it("conducts idss at vGS = 0 in saturation, within 1%", () => {
    // Depletion-mode defining property: full channel current with the gate
    // shorted to the source. vDS = 10 V >= vOV = 2 V, so saturation holds
    // and Id = beta*vto^2 = idss exactly (lambda = 0).
    const { engine, result } = solvedOperatingPoint(jfetBiasCircuit(0, 0));
    expect(result.converged).toBe(true);
    expectRel(engine.getElementI().j1 ?? Number.NaN, JFET_IDSS, 0.01, "Id at vGS=0");
  });

  it("pinches off below vto: only leakage-scale current remains", () => {
    // vGS = -3 V < vto = -2 V: the channel formula is zero; what remains is
    // the GMIN floor (1e-12 S across 10 V) plus reverse gate leakage, both
    // nanoamp-scale or below. Anything larger means the channel failed to
    // pinch off.
    const { engine, result } = solvedOperatingPoint(jfetBiasCircuit(-3, 0));
    expect(result.converged).toBe(true);
    expect(Math.abs(engine.getElementI().j1 ?? Number.NaN)).toBeLessThan(1e-9);
  });

  it("saturation transfer curve matches beta*(vGS-vto)^2*(1+lambda*vDS) within 1%", () => {
    // Three bias points spanning the square law, with channel-length
    // modulation enabled so the (1 + lambda*vDS) factor is actually
    // exercised. The drain is tied to the ideal 10 V rail, so vDS = 10
    // exactly and the reference needs no solved-voltage feedback.
    const lambda = 0.05;
    const vds = 10;
    for (const vgs of [-0.5, -1.0, -1.5]) {
      const { engine, result } = solvedOperatingPoint(jfetBiasCircuit(vgs, lambda));
      expect(result.converged, `OP at vGS=${String(vgs)}`).toBe(true);
      const vOV = vgs - JFET_VTO;
      const expected = JFET_BETA * vOV * vOV * (1 + lambda * vds);
      expectRel(
        engine.getElementI().j1 ?? Number.NaN,
        expected,
        0.01,
        `Id at vGS=${String(vgs)}`,
      );
    }
  });

  it("forward gate drive conducts through the physical gate junctions", () => {
    // The gate is a real pn junction against the channel, not a MOSFET
    // insulator: driven positive through 10 kOhm with drain and source
    // grounded, it must clamp near one diode drop (both gate junctions in
    // parallel) instead of floating at the 5 V drive.
    const circuit: SimCircuit = {
      components: [
        voltageSource("vg", 5),
        resistor("rg", 10_000),
        {
          id: "j1",
          kind: "njfet",
          pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
          params: { vto: JFET_VTO, idss: JFET_IDSS, lambda: 0 },
        },
      ],
      wires: [
        wire("vg", "pos", "rg", "a"),
        wire("rg", "b", "j1", "g"),
        wire("j1", "s", "vg", "neg"),
        wire("j1", "d", "vg", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    const { engine, result } = solvedOperatingPoint(circuit);
    expect(result.converged).toBe(true);
    const vGate = nodeVoltage(engine, "j1", "g");
    expect(vGate, "gate must clamp to a junction drop").toBeGreaterThan(0.4);
    expect(vGate, "gate must clamp to a junction drop").toBeLessThan(0.8);
    // The drive resistor must carry real junction current, not leakage.
    expect(engine.getElementI().rg ?? Number.NaN).toBeGreaterThan(3e-4);
  });

  it("common-source small-signal gain equals gm*RL within 2%", () => {
    // Bias: vGS = -1 V, vOV = 1 V, Id = 2.5 mA through RL = 1 kOhm from a
    // 15 V rail, so vDS = 12.5 V >> vOV keeps the device in saturation.
    // With lambda = 0 the output conductance is the bare GMIN floor and the
    // textbook CS gain |H| = gm*RL = 2*beta*vOV*RL = 5.0 holds to solver
    // precision; the 2% band is the contract's own tolerance.
    const rl = 1_000;
    const circuit: SimCircuit = {
      components: [
        voltageSource("vdd", 15),
        resistor("rl", rl),
        voltageSource("vg", -1),
        {
          id: "j1",
          kind: "njfet",
          pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
          params: { vto: JFET_VTO, idss: JFET_IDSS, lambda: 0 },
        },
      ],
      wires: [
        wire("vdd", "pos", "rl", "a"),
        wire("rl", "b", "j1", "d"),
        wire("j1", "s", "vdd", "neg"),
        wire("vg", "pos", "j1", "g"),
        wire("vg", "neg", "vdd", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    const engine = new SimEngine();
    engine.load(circuit);
    const drainNet = netIdFor(engine, "j1", "d");
    const ac = runSmallSignalAc(engine, {
      inputId: "vg",
      outputNetIds: [drainNet],
      frequenciesHz: [10, 1e3, 1e5],
    });
    // Bias sanity first: the committed OP is the engine's public readout.
    expectRel(engine.getNetV()[drainNet] ?? Number.NaN, 12.5, 0.01, "drain bias");
    const gm = 2 * JFET_BETA * 1; // 2*beta*vOV at vOV = 1 V
    const out = ac.outputs[0]!;
    for (let i = 0; i < ac.frequenciesHz.length; i++) {
      const mag = Math.hypot(out.re[i]!, out.im[i]!);
      expectRel(mag, gm * rl, 0.02, `CS gain at ${String(ac.frequenciesHz[i])} Hz`);
      // Common-source stages invert; a positive real part would mean the
      // transconductance was stamped with the wrong sign.
      expect(out.re[i]!).toBeLessThan(0);
      expect(Math.abs(out.im[i]!)).toBeLessThan(1e-9);
    }
  });
});

// ─── SCR ─────────────────────────────────────────────────────────────────────

// Datasheet trigger set used everywhere below (explicit so the reference
// currents in the assertions are self-describing).
const SCR_VGT = 0.7;
const SCR_IGT = 5e-3;
const SCR_IH = 5e-3;
const SCR_VTM = 1.2;
const SCR_RON = 0.1;
const SCR_SUPPLY = 12;
const SCR_RL = 100;
const SCR_ON_CURRENT = (SCR_SUPPLY - SCR_VTM) / (SCR_RL + SCR_RON);

function scrComponent(): SimComponent {
  return {
    id: "scr1",
    kind: "scr",
    pins: [{ id: "a" }, { id: "k" }, { id: "g" }],
    params: { vgt: SCR_VGT, igt: SCR_IGT, ih: SCR_IH, vtm: SCR_VTM, ron: SCR_RON },
  };
}

/**
 * One-run lifecycle stimulus. Anode supply: 12 V, dipping to 0 V during
 * 8.1-10.1 ms and back (per=0 selects the documented one-shot pulse mode).
 * Gate: a single 5 V pulse into 220 Ohm during 2.1-3.1 ms, delivering
 * ~19 mA >> igt while it lasts. Timeline: block (0-2 ms), fire (~2.2 ms),
 * latch with the gate removed (3.2-8 ms), dropout during the supply dip,
 * re-block when the supply returns without gate drive.
 */
function scrLifecycleCircuit(): SimCircuit {
  return {
    components: [
      pulseSource("vsup", { v1: SCR_SUPPLY, v2: 0, td: 8e-3, tr: 1e-4, tf: 1e-4, pw: 2e-3, per: 0 }),
      resistor("rl", SCR_RL),
      scrComponent(),
      pulseSource("vgate", { v1: 0, v2: 5, td: 2e-3, tr: 1e-4, tf: 1e-4, pw: 1e-3, per: 0 }),
      resistor("rg", 220),
    ],
    wires: [
      wire("vsup", "pos", "rl", "a"),
      wire("rl", "b", "scr1", "a"),
      wire("scr1", "k", "vsup", "neg"),
      wire("vgate", "pos", "rg", "a"),
      wire("rg", "b", "scr1", "g"),
      wire("vgate", "neg", "vsup", "neg"),
    ],
    environment: { temperatureC: 25 },
  };
}

interface ScrRun {
  frames: string[];
  scrI: number[];
  loadI: number[];
  anodeV: number[];
  gateV: number[];
}

function runScrLifecycle(): ScrRun {
  const engine = new SimEngine();
  engine.load(scrLifecycleCircuit());
  const anodeNet = netIdFor(engine, "scr1", "a");
  const gateNet = netIdFor(engine, "scr1", "g");
  const run: ScrRun = { frames: [], scrI: [], loadI: [], anodeV: [], gateV: [] };
  for (let i = 0; i < 120; i++) {
    engine.step(1e-4);
    run.frames.push(serializeFrame(engine));
    run.scrI.push(engine.getElementI().scr1 ?? Number.NaN);
    run.loadI.push(engine.getElementI().rl ?? Number.NaN);
    run.anodeV.push(engine.getNetV()[anodeNet] ?? Number.NaN);
    run.gateV.push(engine.getNetV()[gateNet] ?? Number.NaN);
  }
  return run;
}

describe("scr — latching thyristor lifecycle", () => {
  it("blocks, fires on gate, latches after gate removal, drops out below ih, re-blocks", () => {
    const run = runScrLifecycle();

    // t ~ 1.5 ms: forward-blocking. The 1 nS principal leak across 12 V is
    // ~12 nA; the anode sees essentially the full supply.
    expect(Math.abs(run.scrI[14]!), "blocking-phase current").toBeLessThan(1e-6);
    expect(run.anodeV[14]!, "blocking anode voltage").toBeGreaterThan(11.9);

    // t ~ 2.8 ms: fired by the gate pulse. Load current is the on-state
    // law (V - vtm)/(RL + ron), asserted on both the SCR element and the
    // series resistor (series KCL).
    expectRel(run.scrI[27]!, SCR_ON_CURRENT, 0.02, "on-state SCR current");
    expectRel(run.loadI[27]!, SCR_ON_CURRENT, 0.02, "on-state load current");

    // t ~ 6.1 ms: the gate pulse ended at 3.1 ms — the latch must hold
    // with the gate at rest. That IS the device: while on, the gate is
    // ignored.
    expect(run.gateV[60]!, "gate must be at rest while latched").toBeLessThan(0.1);
    expectRel(run.scrI[60]!, SCR_ON_CURRENT, 0.02, "latched current after gate removal");

    // t ~ 9.5 ms: the supply dip carried the principal current below ih,
    // so the latch dropped; at 0 V supply only noise-scale current remains.
    expect(Math.abs(run.scrI[94]!), "post-dropout current").toBeLessThan(1e-4);

    // t ~ 11.5 ms: supply restored, gate still at rest — the SCR must
    // RE-block, proving dropout cleared the latch rather than merely
    // starving it.
    expect(Math.abs(run.scrI[114]!), "re-blocked current").toBeLessThan(1e-6);
    expect(run.anodeV[114]!, "re-blocked anode voltage").toBeGreaterThan(11.9);
  });

  it("the full lifecycle trajectory is bitwise deterministic across two fresh runs", () => {
    const first = runScrLifecycle();
    const second = runScrLifecycle();
    expectIdenticalFrames("SCR lifecycle replay", first.frames, second.frames);
  });

  it("does not fire at half the rated gate current", () => {
    // Gate resistor sized so the junction (anchored to pass igt exactly at
    // vgt) carries ~0.5*igt: at 2.5 mA the junction sits at
    // vgt + Vt*ln(0.5) ~ 0.682 V, so (5 - 0.682 V)/1727 Ohm ~ 2.5 mA. Both
    // trigger conditions (iGate >= igt AND vGK >= vgt) then fail together,
    // by the model's Is anchoring.
    const circuit: SimCircuit = {
      components: [
        voltageSource("vsup", SCR_SUPPLY),
        resistor("rl", SCR_RL),
        scrComponent(),
        voltageSource("vg", 5),
        resistor("rg", 1_727),
      ],
      wires: [
        wire("vsup", "pos", "rl", "a"),
        wire("rl", "b", "scr1", "a"),
        wire("scr1", "k", "vsup", "neg"),
        wire("vg", "pos", "rg", "a"),
        wire("rg", "b", "scr1", "g"),
        wire("vg", "neg", "vsup", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    const engine = new SimEngine();
    engine.load(circuit);
    const anodeNet = netIdFor(engine, "scr1", "a");
    for (let i = 0; i < 30; i++) {
      engine.step(1e-4);
      expect(
        Math.abs(engine.getElementI().scr1 ?? Number.NaN),
        `step ${String(i)}: SCR fired on a sub-threshold gate`,
      ).toBeLessThan(1e-6);
      expect(engine.getNetV()[anodeNet]!).toBeGreaterThan(11.9);
    }
    // Sanity on the stimulus itself: the gate really is carrying about half
    // the trigger current, so the no-fire result is a threshold statement
    // rather than a dead-gate artifact.
    const iGate = engine.getElementI().rg ?? Number.NaN;
    expectRel(iGate, 0.5 * SCR_IGT, 0.05, "sub-threshold gate current");
  });

  it("fires with an authored vgt of 2.5 V (above the generic 80*n*Vt exponential cap)", () => {
    // Datasheet Vgt(max) runs to ~2.5 V (2N690-class parts). The gate
    // junction's Is is anchored to pass exactly igt AT vgt, so the
    // exponential guard must clear the anchor: a cap below it (the LED
    // stamps' fixed 80*n*Vt is ~2.07 V at n = 1) freezes the tangent
    // extension's slope so far under the anchor that iGate >= igt becomes
    // unreachable at ANY drive — a silently untriggerable part. 12 V into
    // 100 Ohm offers ~95 mA >> igt, so more gate drive must always fire.
    const circuit: SimCircuit = {
      components: [
        voltageSource("vsup", SCR_SUPPLY),
        resistor("rl", SCR_RL),
        {
          ...scrComponent(),
          params: { ...scrComponent().params, vgt: 2.5 },
        },
        voltageSource("vg", 12),
        resistor("rg", 100),
      ],
      wires: [
        wire("vsup", "pos", "rl", "a"),
        wire("rl", "b", "scr1", "a"),
        wire("scr1", "k", "vsup", "neg"),
        wire("vg", "pos", "rg", "a"),
        wire("rg", "b", "scr1", "g"),
        wire("vg", "neg", "vsup", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    const engine = new SimEngine();
    engine.load(circuit);
    for (let i = 0; i < 10; i++) engine.step(1e-4);
    expectRel(
      engine.getElementI().scr1 ?? Number.NaN,
      SCR_ON_CURRENT,
      0.02,
      "high-vgt on-state current",
    );
  });

  it("a maintained gate sustains conduction below the holding current without chatter", () => {
    // Only ~0.38 mA is available through 10 kOhm — under ih = 5 mA. An open
    // gate must not stay latched there (the lifecycle test pins that), but
    // ih is an open-gate datasheet number: while trigger drive is maintained
    // the gate current keeps the regenerative pair conducting. The broken
    // alternative is a blocking/conducting latch flip every commit — a
    // square wave at the step rate through a DC circuit.
    const circuit: SimCircuit = {
      components: [
        voltageSource("vsup", 5),
        resistor("rl", 10_000),
        scrComponent(),
        voltageSource("vg", 5),
        resistor("rg", 220),
      ],
      wires: [
        wire("vsup", "pos", "rl", "a"),
        wire("rl", "b", "scr1", "a"),
        wire("scr1", "k", "vsup", "neg"),
        wire("vg", "pos", "rg", "a"),
        wire("rg", "b", "scr1", "g"),
        wire("vg", "neg", "vsup", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    const engine = new SimEngine();
    engine.load(circuit);
    const expected = (5 - SCR_VTM) / (10_000 + SCR_RON);
    for (let i = 0; i < 40; i++) engine.step(1e-4);
    // Every settled step must carry the steady gate-sustained on current —
    // any latch chatter would alternate it with nanoamp blocking readings.
    for (let i = 0; i < 10; i++) {
      engine.step(1e-4);
      expectRel(
        engine.getElementI().scr1 ?? Number.NaN,
        expected,
        0.02,
        `step ${String(i)}: gate-sustained sub-ih current`,
      );
    }
  });

  it("publishes the stamped-regime current on flip steps (series KCL holds on every step)", () => {
    // The latch flips POST-solve, so on a trigger/dropout step the accepted
    // x still solved the OLD regime's branch. Publishing the just-committed
    // regime's law on that x invents current no series neighbour carries
    // (the conducting law over a blocking-state 12 V reads ~108 A). The
    // element current must track the series resistor on EVERY step of the
    // lifecycle, transitions included, up to node-shunt leakage.
    const run = runScrLifecycle();
    for (let i = 0; i < run.scrI.length; i++) {
      expect(
        Math.abs(run.scrI[i]! - run.loadI[i]!),
        `step ${String(i)}: SCR vs series load KCL`,
      ).toBeLessThan(1e-6);
    }
  });
});

// ─── SCR — DC operating point and small-signal linearization ────────────────

/** DC-driven conducting fixture: 12 V anode loop, 5 V gate through 220 Ohm. */
function scrDcConductingCircuit(): SimCircuit {
  return {
    components: [
      voltageSource("vsup", SCR_SUPPLY),
      resistor("rl", SCR_RL),
      scrComponent(),
      voltageSource("vg", 5),
      resistor("rg", 220),
    ],
    wires: [
      wire("vsup", "pos", "rl", "a"),
      wire("rl", "b", "scr1", "a"),
      wire("scr1", "k", "vsup", "neg"),
      wire("vg", "pos", "rg", "a"),
      wire("rg", "b", "scr1", "g"),
      wire("vg", "neg", "vsup", "neg"),
    ],
    environment: { temperatureC: 25 },
  };
}

describe("scr — dcOperatingPoint and runSmallSignalAc", () => {
  it("dcOperatingPoint settles the latch into conduction, bit-identically across runs", () => {
    const baseline = solvedOperatingPoint(scrDcConductingCircuit());
    expect(baseline.result.converged).toBe(true);
    // The regime settle loop must have committed the fired latch: the OP
    // readout carries the on-state load current.
    expectRel(
      baseline.engine.getElementI().scr1 ?? Number.NaN,
      SCR_ON_CURRENT,
      0.02,
      "OP on-state current",
    );
    const baselineNetV = baseline.engine.getNetV();
    for (let run = 0; run < 2; run++) {
      const repeat = solvedOperatingPoint(scrDcConductingCircuit());
      expect(repeat.result.converged).toBe(true);
      const netV = repeat.engine.getNetV();
      for (const [netId, value] of Object.entries(baselineNetV)) {
        expect(netV[netId], `run ${String(run)}: net ${netId} diverged`).toBe(value);
      }
    }
  });

  it("conducting small-signal model is 1/ron on the principal path plus the gate junction slope", () => {
    const engine = new SimEngine();
    engine.load(scrDcConductingCircuit());
    const anodeNet = netIdFor(engine, "scr1", "a");
    const gateNet = netIdFor(engine, "scr1", "g");
    const frequenciesHz = [10, 1e3, 1e5];

    // Sweep 1 — principal path. Unit drive on the anode supply divides
    // between RL and the SCR's committed on-regime conductance 1/ron (the
    // vtm Norton offset differentiates away), so the anode transfer is the
    // exact resistive divider below. Any blocking-leak stamping would put
    // H near 1 instead of ~1e-3.
    const acAnode = runSmallSignalAc(engine, {
      inputId: "vsup",
      outputNetIds: [anodeNet],
      frequenciesHz,
    });
    const gRl = 1 / SCR_RL;
    const expectedAnodeH = gRl / (gRl + 1 / SCR_RON + NODE_RSHUNT_G);
    for (let i = 0; i < frequenciesHz.length; i++) {
      const out = acAnode.outputs[0]!;
      expectRel(out.re[i]!, expectedAnodeH, 1e-9, "anode divider vs 1/ron");
      expect(Math.abs(out.im[i]!)).toBeLessThan(1e-15);
    }

    // Sweep 2 — gate path. The gate node divides rg against the committed
    // gate-junction slope at the OP: gd = Is*exp(vGK/Vt)/Vt with the same
    // trigger-anchored Is the transient stamp uses. The 1e-12 term is the
    // stamp's AC_GMIN floor (devices/semiconductors.ts), inlined because
    // that module is not on the package export map.
    const acGate = runSmallSignalAc(engine, {
      inputId: "vg",
      outputNetIds: [gateNet],
      frequenciesHz,
    });
    const vt = thermalVoltage(25);
    const vGk = engine.getNetV()[gateNet]!; // cathode is the gnd reference
    // Precondition for the closed form below: the bias sits under the
    // raised exponential guard, so no tangent extension is in play.
    expect(vGk).toBeLessThan(Math.max(40 * N_DIODE * vt, SCR_VGT + 5 * N_DIODE * vt));
    const isGate = shockleyIsFromVf(SCR_VGT, SCR_IGT, N_DIODE, vt);
    const gGate = (isGate * Math.exp(vGk / (N_DIODE * vt))) / (N_DIODE * vt);
    const gRg = 1 / 220;
    const expectedGateH = gRg / (gRg + gGate + 1e-12 + NODE_RSHUNT_G);
    for (let i = 0; i < frequenciesHz.length; i++) {
      const out = acGate.outputs[0]!;
      expectRel(out.re[i]!, expectedGateH, 1e-9, "gate divider vs junction slope");
      expect(Math.abs(out.im[i]!)).toBeLessThan(1e-15);
    }
  });

  it("a same-id kind swap (scr -> triac) starts from the fresh kind's cold latch", () => {
    // load() carries icState by component ID; that carry must be kind-gated
    // (the failures/mosfetGates rule) or the scr's committed on-latch would
    // seed the replacement triac's regime and stamp a phantom conducting
    // interval on the load seed solve before dropout self-corrects.
    const engine = new SimEngine();
    engine.load(scrDcConductingCircuit());
    for (let i = 0; i < 20; i++) engine.step(1e-4);
    expectRel(
      engine.getElementI().scr1 ?? Number.NaN,
      SCR_ON_CURRENT,
      0.02,
      "pre-swap latched scr",
    );

    // Same ID and position, triac kind, gate returned to rest so the
    // replacement part cannot legitimately fire on its own.
    const swapped: SimCircuit = {
      components: [
        voltageSource("vsup", SCR_SUPPLY),
        resistor("rl", SCR_RL),
        {
          id: "scr1",
          kind: "triac",
          pins: [{ id: "mt1" }, { id: "mt2" }, { id: "g" }],
          params: { vgt: SCR_VGT, igt: SCR_IGT, ih: SCR_IH, vtm: SCR_VTM, ron: SCR_RON },
        },
        voltageSource("vg", 0),
        resistor("rg", 220),
      ],
      wires: [
        wire("vsup", "pos", "rl", "a"),
        wire("rl", "b", "scr1", "mt2"),
        wire("scr1", "mt1", "vsup", "neg"),
        wire("vg", "pos", "rg", "a"),
        wire("rg", "b", "scr1", "g"),
        wire("vg", "neg", "vsup", "neg"),
      ],
      environment: { temperatureC: 25 },
    };
    engine.load(swapped);
    // Cold triac latch: the load seed solve must already publish blocking
    // nanoamps with mt2 at the full rail, never a carried conducting stamp.
    expect(
      Math.abs(engine.getElementI().scr1 ?? Number.NaN),
      "post-swap seed current",
    ).toBeLessThan(1e-6);
    expect(
      engine.getNetV()[netIdFor(engine, "scr1", "mt2")]!,
      "post-swap blocked mt2",
    ).toBeGreaterThan(11.9);
    engine.step(1e-4);
    expect(
      Math.abs(engine.getElementI().scr1 ?? Number.NaN),
      "post-swap first-step current",
    ).toBeLessThan(1e-6);
  });
});

// ─── Triac ───────────────────────────────────────────────────────────────────

const TRIAC_AMPLITUDE = 24;
const TRIAC_RL = 100;
const TRIAC_ON_CURRENT = (TRIAC_AMPLITUDE - SCR_VTM) / (TRIAC_RL + SCR_RON);

/**
 * 50 Hz, 24 V-peak sine through 100 Ohm into mt2, mt1 grounded, gate
 * referenced to mt1 through 220 Ohm. The gate component is the parameter:
 * a DC source proves both-polarity firing, a one-shot pulse proves
 * commutation at the zero crossing once the gate is removed.
 */
function triacCircuit(gate: SimComponent): SimCircuit {
  return {
    components: [
      {
        id: "sg",
        kind: "signal_gen",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: {
          waveform: "sine",
          amplitude: TRIAC_AMPLITUDE,
          offset: 0,
          frequency: 50,
          rSource: 0,
          enabled: 1,
        },
      },
      resistor("rl", TRIAC_RL),
      {
        id: "tr1",
        kind: "triac",
        pins: [{ id: "mt1" }, { id: "mt2" }, { id: "g" }],
        params: { vgt: SCR_VGT, igt: SCR_IGT, ih: SCR_IH, vtm: SCR_VTM, ron: SCR_RON },
      },
      gate,
      resistor("rg", 220),
    ],
    wires: [
      wire("sg", "pos", "rl", "a"),
      wire("rl", "b", "tr1", "mt2"),
      wire("tr1", "mt1", "sg", "neg"),
      wire(gate.id, "pos", "rg", "a"),
      wire("rg", "b", "tr1", "g"),
      wire(gate.id, "neg", "sg", "neg"),
    ],
    environment: { temperatureC: 25 },
  };
}

interface TriacRun {
  triacI: number[];
  mt2V: number[];
  gateV: number[];
}

function runTriac(gate: SimComponent, steps: number): TriacRun {
  const engine = new SimEngine();
  engine.load(triacCircuit(gate));
  const mt2Net = netIdFor(engine, "tr1", "mt2");
  const gateNet = netIdFor(engine, "tr1", "g");
  const run: TriacRun = { triacI: [], mt2V: [], gateV: [] };
  for (let i = 0; i < steps; i++) {
    engine.step(1e-4);
    run.triacI.push(engine.getElementI().tr1 ?? Number.NaN);
    run.mt2V.push(engine.getNetV()[mt2Net] ?? Number.NaN);
    run.gateV.push(engine.getNetV()[gateNet] ?? Number.NaN);
  }
  return run;
}

describe("triac — bidirectional latching", () => {
  it("with a held gate it conducts BOTH mains polarities with the correct sign", () => {
    const run = runTriac(voltageSource("vgd", 5), 160);

    // Positive peak (t ~ 5 ms): conduction is mt2 -> mt1, which reads
    // NEGATIVE in the element's mt1 -> mt2 convention, and the on-state
    // drop across the device is +vtm plus the small ron term.
    expectRel(run.triacI[49]!, -TRIAC_ON_CURRENT, 0.02, "positive-half current");
    expect(run.mt2V[49]!, "positive-half device drop").toBeGreaterThan(1.05);
    expect(run.mt2V[49]!, "positive-half device drop").toBeLessThan(1.4);

    // Negative peak (t ~ 15 ms): polarity flipped, current now mt1 -> mt2
    // (positive) and the drop mirrors to -vtm. A unidirectional (SCR-like)
    // defect would leave the negative half blocked at the full -24 V.
    expectRel(run.triacI[149]!, TRIAC_ON_CURRENT, 0.02, "negative-half current");
    expect(run.mt2V[149]!, "negative-half device drop").toBeLessThan(-1.05);
    expect(run.mt2V[149]!, "negative-half device drop").toBeGreaterThan(-1.4);
  });

  it("latches through the half-cycle after the gate pulse, then commutates at the zero crossing", () => {
    // One-shot gate pulse during 1.1-2.1 ms only.
    const run = runTriac(
      pulseSource("vgp", { v1: 0, v2: 5, td: 1e-3, tr: 1e-4, tf: 1e-4, pw: 1e-3, per: 0 }),
      160,
    );

    // t ~ 5 ms: the gate has been at rest for ~3 ms, yet the latch holds
    // through the rest of the positive half-cycle.
    expect(run.gateV[49]!, "gate must be at rest while latched").toBeLessThan(0.1);
    expectRel(run.triacI[49]!, -TRIAC_ON_CURRENT, 0.02, "latched positive-half current");

    // t ~ 12 ms: past the 10 ms zero crossing the conduction current fell
    // below ih and no gate re-fires it — the device must sit blocking in
    // the negative half with only the 1 nS leak.
    expect(Math.abs(run.triacI[119]!), "post-commutation current").toBeLessThan(1e-6);

    // t ~ 15 ms (negative peak): still blocked, so mt2 sees the full
    // negative supply through the unloaded RL.
    expect(Math.abs(run.triacI[149]!), "negative-peak current").toBeLessThan(1e-6);
    expect(run.mt2V[149]!, "blocked mt2 must follow the supply").toBeLessThan(-23);
  });

  it("a driven gate cannot latch an unbiased MT pair into a phantom vtm rail", () => {
    // MT2 tied to MT1 only through 1 MOhm — no source anywhere in the
    // principal loop. A real triac with zero MT bias conducts nothing and
    // drops nothing; firing here would stamp the conducting branch's
    // pol*vtm/ron Norton source and erect a +-1.2 V rail across a passive
    // pair (alternating with dropout commits). The trigger therefore
    // requires |v12| to clear the on-state drop vtm as well as the gate
    // conditions.
    const circuit: SimCircuit = {
      components: [
        voltageSource("vgd", 5),
        resistor("rg", 220),
        {
          id: "tr1",
          kind: "triac",
          pins: [{ id: "mt1" }, { id: "mt2" }, { id: "g" }],
          params: { vgt: SCR_VGT, igt: SCR_IGT, ih: SCR_IH, vtm: SCR_VTM, ron: SCR_RON },
        },
        resistor("rmt", 1e6),
      ],
      wires: [
        wire("vgd", "pos", "rg", "a"),
        wire("rg", "b", "tr1", "g"),
        wire("vgd", "neg", "tr1", "mt1"),
        wire("tr1", "mt2", "rmt", "a"),
        wire("rmt", "b", "tr1", "mt1"),
      ],
      environment: { temperatureC: 25 },
    };
    const engine = new SimEngine();
    engine.load(circuit);
    const mt2Net = netIdFor(engine, "tr1", "mt2");
    for (let i = 0; i < 30; i++) {
      engine.step(1e-4);
      expect(
        Math.abs(engine.getNetV()[mt2Net] ?? Number.NaN),
        `step ${String(i)}: unbiased mt2 voltage`,
      ).toBeLessThan(1e-3);
      expect(
        Math.abs(engine.getElementI().tr1 ?? Number.NaN),
        `step ${String(i)}: unbiased principal current`,
      ).toBeLessThan(1e-6);
    }
  });
});

// ─── Optocoupler ─────────────────────────────────────────────────────────────

const OPTO_CTR = 1.0;

/**
 * Input: 5 V through 383.6 Ohm sizes the LED at ~10 mA (vf = 1.2 V at
 * 20 mA, n = 2: the drop at 10 mA is ~1.164 V). Output: an INDEPENDENT
 * 5 V / 200 Ohm collector loop sharing no net with the input side — the
 * point of the part. ledVoltage lets the dark test reuse the topology.
 */
function optoCircuit(ledVoltage: number, withShiftProbe: boolean): SimCircuit {
  const components: SimComponent[] = [
    voltageSource("vin", ledVoltage),
    resistor("rled", 383.6),
    {
      id: "oc1",
      kind: "opto_npn",
      pins: [{ id: "led_a" }, { id: "led_k" }, { id: "c" }, { id: "e" }],
      params: { ctr: OPTO_CTR, betaF: 100, vf: 1.2, iRated: 0.02 },
    },
    voltageSource("vcc", 5),
    resistor("rc", 200),
  ];
  const wires: SimWire[] = [
    wire("vin", "pos", "rled", "a"),
    wire("rled", "b", "oc1", "led_a"),
    wire("oc1", "led_k", "vin", "neg"),
    wire("vcc", "pos", "rc", "a"),
    wire("rc", "b", "oc1", "c"),
    wire("oc1", "e", "vcc", "neg"),
  ];
  if (withShiftProbe) {
    // 100 V common-mode tie between the sides: any conductive input-output
    // path would drive real current through this source; true galvanic
    // isolation leaves it carrying only the output nodes' 1e-12 S shunts.
    components.push(voltageSource("vshift", 100));
    wires.push(
      wire("vshift", "pos", "oc1", "e"),
      wire("vshift", "neg", "vin", "neg"),
    );
  }
  return { components, wires, environment: { temperatureC: 25 } };
}

describe("opto_npn — current-transfer ratio and isolation", () => {
  it("transfers ctr * iLed to the collector within 5% at 10 mA drive", () => {
    const { engine, result } = solvedOperatingPoint(optoCircuit(5, false));
    expect(result.converged).toBe(true);
    // Element current is the input LED current by convention; pin the
    // stimulus first so the CTR check is anchored at the rated point.
    const iLed = engine.getElementI().oc1 ?? Number.NaN;
    expectRel(iLed, 0.010, 0.02, "LED drive current");
    // Collector current through the load; vce = 5 - 2 = 3 V keeps the
    // phototransistor active, so Ic = ctr*iLed up to base-current-scale
    // corrections (~1/betaF), well inside the 5% band.
    const iCollector = engine.getElementI().rc ?? Number.NaN;
    expectRel(iCollector, OPTO_CTR * iLed, 0.05, "collector current vs ctr*iLed");
  });

  it("is dark with the LED unpowered", () => {
    const { engine, result } = solvedOperatingPoint(optoCircuit(0, false));
    expect(result.converged).toBe(true);
    expect(Math.abs(engine.getElementI().oc1 ?? Number.NaN), "LED current").toBeLessThan(1e-9);
    expect(Math.abs(engine.getElementI().rc ?? Number.NaN), "dark collector current").toBeLessThan(1e-6);
    // No collector load drop: the output loop sees its full supply.
    const vce = nodeVoltage(engine, "oc1", "c") - nodeVoltage(engine, "oc1", "e");
    expect(vce).toBeGreaterThan(4.9);
  });

  it("has no DC input-output path: a 100 V common-mode tie carries only shunt leakage", () => {
    const { engine, result } = solvedOperatingPoint(optoCircuit(5, true));
    expect(result.converged).toBe(true);
    // Three output-side nodes riding at ~100 V leak ~3e-10 A through the
    // disclosed per-node shunts; a real conductive path would be orders of
    // magnitude above this bound.
    expect(Math.abs(engine.getElementI().vshift ?? Number.NaN)).toBeLessThan(1e-8);
    // And the optical transfer is indifferent to the common-mode shift.
    const iLed = engine.getElementI().oc1 ?? Number.NaN;
    expectRel(iLed, 0.010, 0.02, "LED current under common-mode shift");
    expectRel(
      engine.getElementI().rc ?? Number.NaN,
      OPTO_CTR * iLed,
      0.05,
      "collector current under common-mode shift",
    );
  });
});

// ─── Analog switch ───────────────────────────────────────────────────────────

const SWITCH_RON = 100;
// 5 V -> 1 kOhm -> switch -> 1 kOhm -> gnd: the on-state output node.
const SWITCH_DIVIDER_V = (5 * 1_000) / (1_000 + SWITCH_RON + 1_000);

function analogSwitchCircuit(ctrl: SimComponent): SimCircuit {
  return {
    components: [
      voltageSource("vsrc", 5),
      resistor("r1", 1_000),
      {
        id: "sw1",
        kind: "analog_switch",
        pins: [{ id: "a" }, { id: "b" }, { id: "ctrl" }],
        params: { ron: SWITCH_RON, roff: 1e9 },
      },
      resistor("r2", 1_000),
      ctrl,
    ],
    wires: [
      wire("vsrc", "pos", "r1", "a"),
      wire("r1", "b", "sw1", "a"),
      wire("sw1", "b", "r2", "a"),
      wire("r2", "b", "vsrc", "neg"),
      wire(ctrl.id, "pos", "sw1", "ctrl"),
      wire(ctrl.id, "neg", "vsrc", "neg"),
    ],
  };
}

describe("analog_switch — transmission gate", () => {
  it("ctrl high passes the divider through ron within 1%", () => {
    const { engine, result } = solvedOperatingPoint(
      analogSwitchCircuit(voltageSource("vctrl", 5)),
    );
    expect(result.converged).toBe(true);
    expectRel(
      nodeVoltage(engine, "sw1", "b"),
      SWITCH_DIVIDER_V,
      0.01,
      "on-state divider voltage",
    );
  });

  it("ctrl low isolates the channel", () => {
    const { engine, result } = solvedOperatingPoint(
      analogSwitchCircuit(voltageSource("vctrl", 0)),
    );
    expect(result.converged).toBe(true);
    // roff = 1 GOhm against the 1 kOhm pulldown: microvolt-scale residue.
    expect(nodeVoltage(engine, "sw1", "b"), "off-state output").toBeLessThan(1e-3);
    expect(Math.abs(engine.getElementI().sw1 ?? Number.NaN), "off-state channel current")
      .toBeLessThan(1e-8);
  });

  it("an authored roff of 0 fails toward isolation, not toward a conducting off-state", () => {
    // roff = 0 is a plausible "ideal switch" author intent. A bare 1 ohm
    // floor (the ron treatment) would make the OFF regime conduct 100x
    // better than the on-state — silently inverted control sense — so
    // non-positive roff must fall back to the isolating default instead.
    const circuit = analogSwitchCircuit(voltageSource("vctrl", 0));
    circuit.components = circuit.components.map((c) =>
      c.id === "sw1" ? { ...c, params: { ...c.params, roff: 0 } } : c,
    );
    const { engine, result } = solvedOperatingPoint(circuit);
    expect(result.converged).toBe(true);
    expect(nodeVoltage(engine, "sw1", "b"), "off-state output with roff = 0").toBeLessThan(1e-3);
    expect(Math.abs(engine.getElementI().sw1 ?? Number.NaN), "off-state current with roff = 0")
      .toBeLessThan(1e-8);
  });

  it("a mid-run ctrl flip replays bitwise after a snapshot rollback", () => {
    // ctrl steps 0 -> 5 V at t = 2 ms, crossing the VIH threshold mid-run.
    // Snapshot before the flip, step across it, roll back, step again: the
    // committed-regime latch, waveform clock, and every derived readout
    // must replay bit-identically — the adaptive-step rollback contract
    // applied to this model's icState key.
    const engine = new SimEngine();
    engine.load(analogSwitchCircuit(
      pulseSource("vctrl", { v1: 0, v2: 5, td: 2e-3, tr: 1e-6, tf: 1e-6, pw: 1, per: 0 }),
    ));
    const outNet = netIdFor(engine, "sw1", "b");
    for (let i = 0; i < 15; i++) engine.step(1e-4);
    // Still off just before the flip.
    expect(engine.getNetV()[outNet]!).toBeLessThan(1e-3);
    const checkpoint = engine.saveState();

    const firstPass: string[] = [];
    for (let i = 0; i < 15; i++) {
      engine.step(1e-4);
      firstPass.push(serializeFrame(engine));
    }
    // The flip genuinely happened inside the replayed window.
    expectRel(engine.getNetV()[outNet]!, SWITCH_DIVIDER_V, 0.01, "post-flip divider");

    engine.restoreState(checkpoint);
    const secondPass: string[] = [];
    for (let i = 0; i < 15; i++) {
      engine.step(1e-4);
      secondPass.push(serializeFrame(engine));
    }
    expectIdenticalFrames("ctrl-flip rollback replay", firstPass, secondPass);
  });
});
