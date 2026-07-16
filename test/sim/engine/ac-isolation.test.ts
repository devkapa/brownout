/**
 * Wave A5 — transient-isolation proof and acStamp registry-hook coverage.
 *
 * runSmallSignalAc's contract is that the AC analysis is a pure OBSERVER of
 * the transient engine: beyond the dcOperatingPoint() commit it documents
 * (the bias point becomes the engine's public readout), the sweep must leave
 * no residue in transient state, committed regimes, OP caches, or the linear
 * system. This suite proves that two ways:
 *
 * 1. Cold-reload bit-identity: on ONE engine instance, a coldLoad()+200-step
 *    trajectory captured before any AC call must be bitwise identical to the
 *    same trajectory captured after a 20-point sweep. coldLoad (not load) is
 *    the reset between captures because load() deliberately carries element
 *    state forward across edits (repo invariant 4), so only coldLoad gives
 *    both captures the same starting state; what the proof then isolates is
 *    residue in everything a reload does NOT rebuild on that instance.
 *
 * 2. Post-OP continuation bit-identity: a fresh engine that ran the full
 *    sweep must keep stepping bitwise identically to a fresh engine that ran
 *    only dcOperatingPoint(). No reload happens between the AC call and the
 *    continued stepping, so this catches residue in the LIVE MNA system and
 *    solver caches that proof 1's rebuild would silently repair.
 *
 * Bitwise identity is deliberate (mirroring optimization-equivalence.test.ts'
 * dense-lane baselines): the AC system is a separate bordered 2n solve, so
 * even one ULP of drift in a transient frame means AC machinery touched the
 * transient path. Both captures of each comparison run in the same process
 * and lane, so the assertion is backend-agnostic — no forced-sparse split.
 *
 * Part (b) is the acStamp coverage census: every registered kind either has
 * an acStamp hook or appears in the documented high-Z/Thevenin-default list
 * below, so adding a kind forces an explicit small-signal decision. Part (c)
 * proves the seam end-to-end for a third-party kind: a synthetic varistor
 * (the test_varistor pattern from device-registry.test.ts) registers its own
 * acStamp and the sweep reproduces the analytic transfer built from its
 * small-signal conductance gd at the committed operating point.
 */

import { describe, expect, it } from "vitest";
import fs from "node:fs";
import path from "node:path";
import { breadboardToSimCircuit } from "../../helpers/circuit/breadboard.js";
import type { Circuit } from "../../../src/circuit/types.js";
import { chaser555CounterCircuit } from "../../helpers/embedded-fixtures.js";
import { runSmallSignalAc, type SmallSignalAcResult } from "../../../src/sim/engine/ac-analysis.js";
import { stampAcAdmittance } from "../../../src/sim/engine/ac-system.js";
import {
  getDeviceModel,
  listRegisteredKinds,
  registerDeviceModel,
  type DeviceModel,
} from "../../../src/sim/engine/device-registry.js";
// Importing sim-engine is load-bearing for the census: its module evaluation
// imports devices/index, which performs the built-in registration audited
// below (same note as device-registry.test.ts).
import { NODE_RSHUNT_G, SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

// ─── Fixtures ─────────────────────────────────────────────────────────────────

// B2 migration: loads the pre-converted electrical corpus (see
// test/fixtures/circuits). breadboardToSimCircuit below is a structural
// no-op on these board-free circuits, so the call sites stay as authored.
function loadFixture(name: string): Circuit {
  const root = path.resolve(__dirname, "../../fixtures/circuits");
  const file = name.endsWith(".sim.json") ? name : name.replace(/\.json$/, ".sim.json");
  return JSON.parse(fs.readFileSync(path.join(root, file), "utf8")) as Circuit;
}

const passiveLabCircuit = (): SimCircuit =>
  breadboardToSimCircuit(loadFixture("01-passive-sensor-transistor-lab.json"));
const chaserCircuit = (): SimCircuit =>
  breadboardToSimCircuit(chaser555CounterCircuit);

// ─── Bitwise frame capture ────────────────────────────────────────────────────

/**
 * Exact string token for a double: String() is shortest-round-trip, so two
 * finite doubles serialize equally iff they are the same bits; NaN and the
 * infinities keep their names instead of JSON's null, and -0 (which String
 * folds to "0") is tagged so a sign flip cannot hide.
 */
function numToken(v: number): string {
  return Object.is(v, -0) ? "-0" : String(v);
}

/**
 * Canonical JSON-ready form of an engine value: Maps and object keys sorted
 * with plain UTF-16 comparison (localeCompare would be environment-dependent),
 * typed arrays expanded, every number tokenized via numToken.
 */
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
 * One trajectory frame: the full saveState() snapshot (element state, trap
 * histories, netV/elementI/digitalState, the last MNA solution vector, and
 * solver diagnostics). lastSolveUs is the one field zeroed out — it is
 * wall-clock timing, the only non-deterministic value in the snapshot.
 */
function serializeFrame(engine: SimEngine): string {
  const snapshot = engine.saveState();
  if (snapshot.solverDiagnostics) {
    snapshot.solverDiagnostics = { ...snapshot.solverDiagnostics, lastSolveUs: 0 };
  }
  return JSON.stringify(canon(snapshot));
}

function stepFrames(engine: SimEngine, steps: number, dt: number): string[] {
  const frames: string[] = [];
  for (let i = 0; i < steps; i++) {
    engine.step(dt);
    frames.push(serializeFrame(engine));
  }
  return frames;
}

function captureColdTrajectory(
  engine: SimEngine,
  circuit: SimCircuit,
  steps: number,
  dt: number,
): string[] {
  engine.coldLoad(circuit);
  return stepFrames(engine, steps, dt);
}

/**
 * Assert two frame sequences are bitwise identical; on divergence, fail with
 * the step index and the first differing region so an isolation violation
 * names the exact quantity instead of dumping whole-snapshot JSON.
 */
function expectIdenticalFrames(
  label: string,
  expected: string[],
  actual: string[],
): void {
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

function logSpacedHz(points: number, loHz: number, hiHz: number): number[] {
  const lo = Math.log10(loHz);
  const hi = Math.log10(hiHz);
  return Array.from(
    { length: points },
    (_, i) => 10 ** (lo + (i * (hi - lo)) / (points - 1)),
  );
}

const SWEEP_20_POINTS = logSpacedHz(20, 10, 1e6);

function netIdFor(engine: SimEngine, compId: string, pinId: string): string {
  const netId = engine.getNetIdForPin(compId, pinId);
  if (!netId) throw new Error(`No net for ${compId}.${pinId}`);
  return netId;
}

/** The sweep must have genuinely run and produced usable numbers. */
function expectUsableSweep(ac: SmallSignalAcResult, points: number): void {
  expect(ac.frequenciesHz).toHaveLength(points);
  for (const out of ac.outputs) {
    expect(out.re).toHaveLength(points);
    expect(out.re.every(Number.isFinite), `finite re for ${out.netId}`).toBe(true);
    expect(out.im.every(Number.isFinite), `finite im for ${out.netId}`).toBe(true);
  }
  // The first (most interesting) probe must show an actual response, so a
  // sweep that silently stamped nothing cannot pass the isolation proof.
  const primary = ac.outputs[0]!;
  expect(Math.hypot(primary.re[0]!, primary.im[0]!)).toBeGreaterThan(0);
}

// ─── Part (a): transient bit-identity spot proof ─────────────────────────────

interface IsolationFixture {
  name: string;
  circuit: () => SimCircuit;
  steps: number;
  dt: number;
  inputId: string;
  /** [componentId, pinId] probes; the first must respond at 10 Hz. */
  outputPins: ReadonlyArray<readonly [string, string]>;
}

const ISOLATION_FIXTURES: IsolationFixture[] = [
  {
    name: "passive lab",
    circuit: passiveLabCircuit,
    steps: 200,
    dt: 1e-4,
    inputId: "vcc_5v",
    outputPins: [["ind_choke", "b"], ["cap_filter", "a"]],
  },
  {
    // Mixed-signal coverage: the 555 and the 74LS161 exercise the
    // committed-output-stage acStamp hooks (rail-referenced 555 push-pull,
    // sequential-counter Thevenin stages), and the bench_psu input
    // exercises the regulated-source branch designation.
    name: "555 chaser",
    circuit: chaserCircuit,
    steps: 200,
    dt: 1e-3,
    inputId: "bench_psu-283d1eee",
    outputPins: [["u1_ne555", "2"], ["u2_74ls161", "clk"]],
  },
];

describe("small-signal AC — transient isolation (Wave A5)", () => {
  for (const fixture of ISOLATION_FIXTURES) {
    it(`${fixture.name}: cold-reload trajectory is bit-identical across a 20-point sweep`, () => {
      const circuit = fixture.circuit();
      const engine = new SimEngine();

      const before = captureColdTrajectory(engine, circuit, fixture.steps, fixture.dt);
      // Determinism precondition on the SAME instance: if a plain replay
      // already diverges, the failure is engine replay nondeterminism, not
      // an A5 isolation bug — this control keeps the blame assignment exact.
      const control = captureColdTrajectory(engine, circuit, fixture.steps, fixture.dt);
      expectIdenticalFrames(
        `${fixture.name} control replay (no AC involved — precondition)`,
        before,
        control,
      );

      engine.coldLoad(circuit);
      const ac = runSmallSignalAc(engine, {
        inputId: fixture.inputId,
        outputNetIds: fixture.outputPins.map(([c, p]) => netIdFor(engine, c, p)),
        frequenciesHz: SWEEP_20_POINTS,
      });
      expectUsableSweep(ac, SWEEP_20_POINTS.length);

      const after = captureColdTrajectory(engine, circuit, fixture.steps, fixture.dt);
      expectIdenticalFrames(
        `${fixture.name} post-AC replay (isolation violation: AC left residue that survived coldLoad)`,
        before,
        after,
      );
    });

    it(`${fixture.name}: stepping continues bit-identically after the sweep vs after a plain OP`, () => {
      const circuit = fixture.circuit();

      // Reference: the state change runSmallSignalAc documents is exactly a
      // committed dcOperatingPoint(); an engine that ran only that call is
      // therefore the baseline the swept engine must match from here on.
      const opOnly = new SimEngine();
      opOnly.coldLoad(circuit);
      const op = opOnly.dcOperatingPoint();
      expect(op.converged, `${fixture.name}: OP must converge`).toBe(true);
      const opFrames = [serializeFrame(opOnly), ...stepFrames(opOnly, fixture.steps, fixture.dt)];

      const swept = new SimEngine();
      swept.coldLoad(circuit);
      const ac = runSmallSignalAc(swept, {
        inputId: fixture.inputId,
        outputNetIds: fixture.outputPins.map(([c, p]) => netIdFor(swept, c, p)),
        frequenciesHz: SWEEP_20_POINTS,
      });
      expectUsableSweep(ac, SWEEP_20_POINTS.length);
      // No reload between the sweep and these steps: residue in the live MNA
      // pattern, OP caches, or committed regimes shows up here even though a
      // rebuild would have hidden it from the cold-reload proof above.
      const acFrames = [serializeFrame(swept), ...stepFrames(swept, fixture.steps, fixture.dt)];

      expectIdenticalFrames(
        `${fixture.name} post-OP continuation (isolation violation: sweep perturbed the live engine beyond the documented OP commit)`,
        opFrames,
        acFrames,
      );
    });
  }
});

// ─── Part (b): acStamp hook coverage census ──────────────────────────────────

/**
 * Registered kinds that DELIBERATELY have no acStamp hook, in registration
 * order. Per the DeviceModel.acStamp contract these are high-Z in the AC
 * system except for two generic obligations ac-analysis.ts fulfils once for
 * all of them: committed digital output stages get the Thevenin-to-rail
 * default, and allocated branch rows are pinned to keep the system regular.
 *
 * Rationale, verified against the Wave A5 device modules: the combinational
 * 74xx cohort is the ONLY set of kinds whose transient analog face is
 * exactly (a) sampled-logic inputs with no admittance to linearize and
 * (b) output stages recorded as committed `delay:<pinId>` icState slots —
 * the record the generic default keys off (with the same open-collector
 * released-HIGH gate the transient stamp applies). Every other kind whose
 * transient stamp writes conductances — sequential/driver/display ICs
 * stamping from their own state keys, segment LED junctions, winding and
 * supply loads — carries a per-kind acStamp hook: the review of this wave
 * confirmed that leaving them to the default silently unloaded their nets
 * in the AC system.
 *
 * This list is asserted with EXACT equality against the registry, so any new
 * kind (or a hook added to or removed from an existing kind) fails the census
 * until its small-signal story is decided here explicitly.
 */
const AC_HIGH_Z_DEFAULT_KINDS = [
  "74ls00",
  "74ls04",
  "74ls08",
  "74ls32",
  "74ls86",
  "74ls157",
  "74ls283",
  "74ls245",
  "74hc14",
  "74hc138",
  "74ls47",
];

describe("device registry — acStamp coverage census", () => {
  // Synthetic kinds registered by test files (this one registers
  // test_ac_varistor below) are not part of the shipped registration and
  // must not fail the census of built-ins.
  const builtinKinds = (): string[] =>
    listRegisteredKinds().filter((kind) => !kind.startsWith("test_"));

  it("every registered kind has acStamp or is on the documented high-Z default list", () => {
    const withoutAcStamp = builtinKinds().filter(
      (kind) => !getDeviceModel(kind)!.acStamp,
    );
    // Exact equality (values AND registration order) is the enforcement: a
    // kind gaining acStamp must leave the list, a kind losing it (or a new
    // hookless kind) must be added here with a documented rationale.
    expect(withoutAcStamp).toEqual(AC_HIGH_Z_DEFAULT_KINDS);
  });

  it("the default list and the acStamp-bearing kinds exactly partition the registry", () => {
    const defaults = new Set(AC_HIGH_Z_DEFAULT_KINDS);
    expect(defaults.size).toBe(AC_HIGH_Z_DEFAULT_KINDS.length);
    for (const kind of builtinKinds()) {
      const hasHook = Boolean(getDeviceModel(kind)!.acStamp);
      expect(
        hasHook !== defaults.has(kind),
        `${kind}: must have exactly one of acStamp hook / default-list entry`,
      ).toBe(true);
    }
  });

  // Wave A6 census extension: the model-families cohort ships transient AND
  // small-signal models together, so each kind is pinned to carry its own
  // acStamp hook. Every one of these stamps conductances from state the
  // generic defaults cannot see (committed thyristor/switch regimes, winding
  // coupling, junction bias, the crystal's reactive branches), so silently
  // moving any of them onto the high-Z default list would unload its nets in
  // the AC system — this test makes that a loud decision, mirroring the
  // partition rule above.
  const WAVE_A6_KINDS = [
    "coupled_inductor",
    "njfet",
    "pjfet",
    "scr",
    "triac",
    "opto_npn",
    "crystal",
    "analog_switch",
  ];

  it("Wave A6 model families each carry a per-kind acStamp hook", () => {
    for (const kind of WAVE_A6_KINDS) {
      const model = getDeviceModel(kind);
      expect(model, `model for ${kind}`).toBeDefined();
      expect(model!.acStamp, `acStamp for ${kind}`).toBeDefined();
      expect(AC_HIGH_Z_DEFAULT_KINDS).not.toContain(kind);
    }
  });
});

// ─── Part (c): third-party acStamp participates in the sweep ─────────────────
//
// Same synthetic-device pattern as device-registry.test.ts' test_varistor:
// conductance g(v) = g0 * (1 + (v/v0)^2), current i(v) = g0*v*(1 + (v/v0)^2).
// Its small-signal conductance at the OP is gd = di/dv = g0*(1 + 3*(v/v0)^2),
// which differs materially from g0 at the chosen bias, so the sweep result
// can only be right if THIS file's acStamp ran against the committed OP.

const G0_SIEMENS = 1e-3;
const V0_VOLTS = 2;

function varistorCurrent(v: number): number {
  return G0_SIEMENS * v * (1 + (v / V0_VOLTS) ** 2);
}

/** Analytic di/dv — the transient Newton companion AND the AC linearization. */
function varistorConductance(v: number): number {
  return G0_SIEMENS * (1 + 3 * (v / V0_VOLTS) ** 2);
}

const acVaristorModel: DeviceModel = {
  kinds: ["test_ac_varistor"],
  stamp: (ctx, comp, xGuess) => {
    const a = ctx.pinNode(comp.id, comp.pins[0]!.id);
    const b = ctx.pinNode(comp.id, comp.pins[1]!.id);
    const vg = ctx.vAt(xGuess, a) - ctx.vAt(xGuess, b);
    const g = varistorConductance(vg);
    const ieq = varistorCurrent(vg) - g * vg;
    if (a >= 0) ctx.mna.add(a, a, g);
    if (b >= 0) ctx.mna.add(b, b, g);
    if (a >= 0 && b >= 0) {
      ctx.mna.add(a, b, -g);
      ctx.mna.add(b, a, -g);
    }
    if (a >= 0) ctx.mna.addB(a, -ieq);
    if (b >= 0) ctx.mna.addB(b, ieq);
  },
  acStamp: (ctx, comp, ac) => {
    // Derivative of the transient stamp at the committed OP (the acStamp
    // contract): the same gd the last Newton companion used, frequency-flat
    // because the device stores no energy.
    const vOp =
      ctx.opPinVoltage(comp.id, comp.pins[0]!.id)
      - ctx.opPinVoltage(comp.id, comp.pins[1]!.id);
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, comp.pins[0]!.id),
      ctx.pinNode(comp.id, comp.pins[1]!.id),
      varistorConductance(vOp),
      0,
    );
  },
};

// Module-scope registration, mirroring a real third-party module registering
// at import time (and device-registry.test.ts' precedent).
registerDeviceModel(acVaristorModel);

const VS = 5;
const R_SERIES = 1000;

function varistorCircuit(): SimCircuit {
  return {
    components: [
      { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: VS } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R_SERIES } },
      { id: "var1", kind: "test_ac_varistor", pins: [{ id: "a" }, { id: "b" }], params: {} },
    ],
    wires: [
      { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "var1", to_pin: "a" },
      { from_component: "var1", from_pin: "b", to_component: "v1", to_pin: "neg" },
    ],
  };
}

/**
 * Independent OP reference (same first-principles bisection as
 * device-registry.test.ts): KCL at the middle node of 5 V -> 1 kOhm ->
 * varistor -> GND, including the disclosed per-node stabilization shunt.
 */
function solveReferenceMidVoltage(): number {
  const f = (v: number): number =>
    (VS - v) / R_SERIES - varistorCurrent(v) - NODE_RSHUNT_G * v;
  let lo = 0;
  let hi = VS;
  // f(0) > 0, f(VS) < 0 and f is strictly decreasing, so bisection is exact.
  for (let i = 0; i < 200; i++) {
    const mid = (lo + hi) / 2;
    if (f(mid) > 0) lo = mid;
    else hi = mid;
  }
  return (lo + hi) / 2;
}

describe("test_ac_varistor — third-party acStamp participates in the sweep", () => {
  it("sweep output matches the analytic transfer built from gd at the OP", () => {
    const vRef = solveReferenceMidVoltage();
    // Nonlinearity window (as in device-registry.test.ts): a linear g0
    // device would bias the node at exactly 2.5 V, so results inside this
    // window can only come from the cubic model.
    expect(vRef).toBeGreaterThan(1.5);
    expect(vRef).toBeLessThan(2.4);

    const engine = new SimEngine();
    engine.load(varistorCircuit());
    const midNet = netIdFor(engine, "var1", "a");

    const frequenciesHz = [10, 1e3, 1e5];
    const ac = runSmallSignalAc(engine, {
      inputId: "v1",
      outputNetIds: [midNet],
      frequenciesHz,
    });

    // runSmallSignalAc committed the OP it linearized about, so the public
    // net readout now IS the bias point; pin it against the reference first.
    const vOp = engine.getNetV()[midNet] ?? 0;
    expect(Math.abs(vOp - vRef)).toBeLessThan(1e-9);

    // Small-signal division at the middle node: the unit drive is pinned on
    // the source node, so H = gR / (gR + gd + gShunt) with gd taken at the OP.
    const gR = 1 / R_SERIES;
    const gd = varistorConductance(vRef);
    const expectedH = gR / (gR + gd + NODE_RSHUNT_G);
    // The discriminator: linearizing with g0 instead of gd would land far
    // away, so a hook that ignored the OP cannot pass the tolerance below.
    const linearH = gR / (gR + G0_SIEMENS + NODE_RSHUNT_G);
    expect(Math.abs(expectedH - linearH)).toBeGreaterThan(0.1);

    const out = ac.outputs[0]!;
    for (let i = 0; i < frequenciesHz.length; i++) {
      // Frequency-flat purely real response: the device stores no energy and
      // nothing else in the loop is reactive.
      expect(Math.abs(out.re[i]! - expectedH)).toBeLessThan(1e-9);
      expect(Math.abs(out.im[i]!)).toBeLessThan(1e-12);
    }
    expect(Math.abs(out.re[0]! - linearH)).toBeGreaterThan(0.1);
  });

  it("registers with an acStamp hook (census interlock)", () => {
    // The census above filters test_ kinds; this pins that the third-party
    // model in this file is registered and does carry the hook the sweep
    // dispatched to.
    expect(getDeviceModel("test_ac_varistor")).toBe(acVaristorModel);
    expect(acVaristorModel.acStamp).toBeDefined();
  });
});
