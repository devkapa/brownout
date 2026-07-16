/**
 * Wave A6 — integration and isolation proof for the model-families cohort
 * (coupled_inductor, njfet/pjfet, scr, triac, opto_npn, crystal,
 * analog_switch) and the internal-solver-node machinery it rides on.
 *
 * A6 is the first registry-FIRST cohort: nothing in it existed as an engine
 * switch case, so per-kind physics is not the risk (each family documents its
 * own math in devices/model-families.ts). The risk is the SEAMS — module-scope
 * registration and key caches shared across engine instances, the new
 * internalNodes row allocation threaded through _buildMatrix, composite-key
 * element state flowing through snapshot/rollback/load-carry, and the acStamp
 * coverage promise. This suite proves those seams four ways:
 *
 * 1. Corpus isolation: the pre-A6 corpus fixtures (passive lab, 555 chaser)
 *    contain no A6 kind, so fresh-engine trajectories must stay bitwise
 *    deterministic. Any divergence means A6's module evaluation or the
 *    internal-node allocation (which must be EMPTY for these circuits — the
 *    dense baselines in optimization-equivalence.test.ts pin the absolute
 *    values, this suite pins cross-instance reproducibility) leaked into
 *    existing solves.
 * 2. Registry and catalog census: the six new families (eight kinds) are
 *    registered at the tail of the deterministic registration order, each
 *    carries its own acStamp hook, each appears in COMPONENT_KINDS, and the
 *    duplicate-registration guard protects every one of them.
 * 3. Mega-fixture: ONE circuit instantiating every A6 kind alongside
 *    battery/resistor/op-amp built-ins converges for 100 steps under BE and
 *    trap, has a convergent DC operating point, sweeps finite small-signal
 *    responses at 10 frequencies, and agrees across the dense and sparse
 *    linear backends within 1e-9 V.
 * 4. State machinery on that fixture: saveState/restoreState is a bitwise
 *    round trip, a mid-run rewind replays bitwise under both integration
 *    methods (composite keys id:1/id:2 and id:cs/id:ls/id:c0 must flow
 *    through the generic snapshot maps with no special case), and a benign
 *    same-id edit carries transformer winding currents and crystal branch
 *    state across load() (repo invariant 4 extended to composite keys).
 */

import { afterEach, describe, expect, it } from "vitest";
import fs from "node:fs";
import path from "node:path";
import { breadboardToSimCircuit } from "../../helpers/circuit/breadboard.js";
import { COMPONENT_KINDS, type Circuit } from "../../../src/circuit/types.js";
import { chaser555CounterCircuit } from "../../helpers/embedded-fixtures.js";
import { runSmallSignalAc } from "../../../src/sim/engine/ac-analysis.js";
import {
  getDeviceModel,
  listRegisteredKinds,
  registerDeviceModel,
} from "../../../src/sim/engine/device-registry.js";
import { setLinearSystemBackendForTests } from "../../../src/sim/engine/linear-system.js";
// Importing sim-engine is load-bearing: its module evaluation imports
// devices/index, which performs the deterministic built-in registration —
// including the A6 tail — that the census below audits (same note as
// device-registry.test.ts and ac-isolation.test.ts).
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

// ─── Fixtures ─────────────────────────────────────────────────────────────────

// B2 migration: loads the pre-converted electrical corpus (see
// test/fixtures/circuits). breadboardToSimCircuit below is a structural
// no-op on these board-free circuits, so the call sites stay as authored.
function loadFixture(name: string): Circuit {
  const root = path.resolve(__dirname, "../../fixtures/circuits");
  const file = name.endsWith(".sim.json") ? name : name.replace(/\.json$/, ".sim.json");
  return JSON.parse(fs.readFileSync(path.join(root, file), "utf8")) as Circuit;
}

// ─── Bitwise frame capture ────────────────────────────────────────────────────
// Duplicated from ac-isolation.test.ts (test modules do not export): the
// canonical serialization makes "same trajectory" mean same DOUBLE BITS in
// every committed quantity, not merely close values.

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
 * One trajectory frame: the full saveState() snapshot. lastSolveUs is the one
 * field zeroed out — wall-clock timing is the only non-deterministic value in
 * the snapshot.
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

/**
 * Assert two frame sequences are bitwise identical; on divergence, fail with
 * the step index and the first differing region so a violation names the
 * exact quantity instead of dumping whole-snapshot JSON.
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

// ─── Shared helpers ───────────────────────────────────────────────────────────

function netIdFor(engine: SimEngine, compId: string, pinId: string): string {
  const netId = engine.getNetIdForPin(compId, pinId);
  if (!netId) throw new Error(`No net for ${compId}.${pinId}`);
  return netId;
}

function netV(engine: SimEngine, compId: string, pinId: string): number {
  return engine.getNetV()[netIdFor(engine, compId, pinId)] ?? 0;
}

/** Step and require convergence every step — a mega-fixture that "runs" by
 *  limping through non-converged frames would prove nothing. */
function runConverged(engine: SimEngine, steps: number, dt: number, label: string): void {
  for (let i = 0; i < steps; i++) {
    engine.step(dt);
    expect(engine.lastConverged, `${label}: step ${i + 1} of ${steps} must converge`).toBe(true);
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

// ─── Part 1: corpus isolation ─────────────────────────────────────────────────

interface CorpusFixture {
  name: string;
  circuit: () => SimCircuit;
  steps: number;
  dt: number;
}

const CORPUS_FIXTURES: CorpusFixture[] = [
  {
    name: "passive lab",
    circuit: () =>
      breadboardToSimCircuit(loadFixture("01-passive-sensor-transistor-lab.json")),
    steps: 200,
    dt: 1e-4,
  },
  {
    name: "555 chaser",
    circuit: () => breadboardToSimCircuit(chaser555CounterCircuit),
    steps: 200,
    dt: 1e-3,
  },
];

describe("corpus isolation — pre-A6 fixtures stay bitwise deterministic (Wave A6)", () => {
  for (const fixture of CORPUS_FIXTURES) {
    it(`${fixture.name}: two fresh engines replay 200 steps bit-identically`, () => {
      // Fresh engines, not same-instance reloads: what A6 could have broken
      // for these circuits is cross-instance state — module-scope caches in
      // the new device modules and the internalNodeIdx rebuild — so the proof
      // must span two engine INSTANCES sharing one process/module graph.
      const engineA = new SimEngine();
      engineA.load(fixture.circuit());
      const framesA = stepFrames(engineA, fixture.steps, fixture.dt);
      expect(engineA.lastConverged, `${fixture.name}: run A must end converged`).toBe(true);

      const engineB = new SimEngine();
      engineB.load(fixture.circuit());
      const framesB = stepFrames(engineB, fixture.steps, fixture.dt);

      expect(framesA).toHaveLength(fixture.steps);
      expectIdenticalFrames(
        `${fixture.name} fresh-engine determinism (A6 registration leaked cross-instance state)`,
        framesA,
        framesB,
      );
    });
  }
});

// ─── Part 2: registry and COMPONENT_KINDS census ─────────────────────────────

/**
 * The six A6 model families as eight registered kinds, in devices/index.ts
 * registration order (which is also their COMPONENT_KINDS declaration order):
 * coupled inductors, JFETs (both channels share one model), the two latching
 * thyristors, the optocoupler, the crystal, and the analog switch.
 */
const WAVE_A6_KINDS = [
  "coupled_inductor",
  "njfet",
  "pjfet",
  "scr",
  "triac",
  "opto_npn",
  "crystal",
  "analog_switch",
] as const;

/**
 * Wave A7 (SPICE interop) appended current_source after the A6 block in both
 * the registration order and COMPONENT_KINDS, for the same order-preservation
 * rationale. The tail censuses below cover both blocks so the loud-on-change
 * interlock keeps working: the next appended kind must be recorded here.
 */
const WAVE_A7_KINDS = ["current_source"] as const;

const REGISTRY_TAIL_KINDS = [...WAVE_A6_KINDS, ...WAVE_A7_KINDS] as const;

describe("device registry — Wave A6 census", () => {
  // Synthetic kinds registered by other test files sharing this worker are
  // not part of the shipped registration and must not perturb the census
  // (the ac-isolation.test.ts precedent).
  const builtinKinds = (): string[] =>
    listRegisteredKinds().filter((kind) => !kind.startsWith("test_"));

  it("all A6 kinds are registered, appended at the registration-order tail", () => {
    // devices/index.ts appends the cohort at the end on purpose so every
    // pre-A6 registration order is untouched; exact tail equality enforces
    // both the presence and that placement decision (with the A7 append
    // following the same rule — see REGISTRY_TAIL_KINDS).
    expect(builtinKinds().slice(-REGISTRY_TAIL_KINDS.length)).toEqual([...REGISTRY_TAIL_KINDS]);
  });

  it("every A6 kind carries its own acStamp hook", () => {
    // The cohort ships transient and small-signal models together; each kind
    // stamps conductances from state the generic AC defaults cannot see
    // (committed regimes, winding coupling, junction bias, reactive
    // branches), so a missing hook silently unloads its nets in the sweep.
    for (const kind of WAVE_A6_KINDS) {
      const model = getDeviceModel(kind);
      expect(model, `model for ${kind}`).toBeDefined();
      expect(model!.acStamp, `acStamp for ${kind}`).toBeDefined();
    }
  });

  it("rejects duplicate registration of every A6 kind and keeps the first model", () => {
    for (const kind of WAVE_A6_KINDS) {
      const original = getDeviceModel(kind);
      expect(original, `pre-existing model for ${kind}`).toBeDefined();
      expect(() =>
        registerDeviceModel({ kinds: [kind], stamp: () => {} }),
      ).toThrow(/already registered/);
      // The failed attempt must not replace the shipped physics model, and
      // must not double the kind in the deterministic registration order.
      expect(getDeviceModel(kind)).toBe(original);
      expect(listRegisteredKinds().filter((k) => k === kind)).toHaveLength(1);
    }
  });

  it("COMPONENT_KINDS declares every A6 kind (catalog census)", () => {
    // Declared as the trailing block of the const list; slice equality keeps
    // the census loud if a kind is dropped, renamed, or reordered without a
    // deliberate decision here (the A7 current_source append is that
    // decision for its own block).
    expect(COMPONENT_KINDS.slice(-REGISTRY_TAIL_KINDS.length)).toEqual([...REGISTRY_TAIL_KINDS]);
  });
});

// ─── Part 3: the mega-fixture ────────────────────────────────────────────────
//
// One battery-fed circuit with every A6 kind in a live, non-degenerate
// operating region plus battery/resistor/op-amp built-ins sharing the rails.
// Blocks hang off the same supply net, so the solve is one coupled system
// while each family's bias point stays hand-computable for the vacuous-pass
// guards below:
//
//   transformer:  5 V -> 100R -> winding 1 -> gnd; winding 2 loads into 100R
//                 (DC: i1 = 5/101 ~ 49.5 mA, i2 -> 0 — mutual coupling is
//                 pure d/dt)
//   njfet:        self-biased common source (rd 1k, rs 330, rg 100k to gnd);
//                 Id ~ 2.85 mA from Id = beta*(2 - 330*Id)^2, so vD ~ 2.15 V
//   pjfet:        the exact mirror off the positive rail; vD ~ 2.85 V
//   scr:          anode via 100R, gate via 470R (~9 mA >= igt 5 mA) — latches
//                 on load and conducts (5 - 1.2)/100.1 ~ 38 mA >= ih
//   triac:        mt2 via 100R, gate via 470R referenced to grounded mt1;
//                 fires with pol = -1 (current mt2 -> mt1), ~ -38 mA
//   opto_npn:     LED at (5 - vf)/330 ~ 11.5 mA; CTR 1 wants ~11.5 mA of
//                 collector current through 1k — impossible from 5 V, so the
//                 output transistor sits SATURATED (vC < 1 V), exercising the
//                 internal-node Ebers-Moll well away from cutoff
//   crystal:      1 MHz series part behind 1k from the rail; DC-open, so its
//                 top pin reads the rail and all three branch memories live
//   analog_switch: ctrl wired to the 5 V rail (>= VIH 2.0 -> on); 1k/ron/1k
//                 ladder puts its b pin at 5*1000/2100 ~ 2.38 V
//   lm358:        unity follower on a 10k/10k divider -> out ~ 2.5 V
//
// A "benign edit" for the carry test changes only the opto collector
// resistor, far from the transformer and crystal whose state must survive.

const MEGA_SUPPLY = 5;
const MEGA_DT = 1e-4;
const MEGA_STEPS = 100;

function megaFixture(overrides?: { rOptoCollector?: number }): SimCircuit {
  const rOptoCollector = overrides?.rOptoCollector ?? 1000;
  const resistor = (
    id: string,
    resistance: number,
  ): SimCircuit["components"][number] => ({
    id,
    kind: "resistor",
    pins: [{ id: "a" }, { id: "b" }],
    params: { resistance },
  });
  return {
    components: [
      { id: "bat", kind: "battery_pack", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: MEGA_SUPPLY } },
      // Transformer branch
      resistor("r_t1", 100),
      { id: "xfmr", kind: "coupled_inductor", pins: [{ id: "a1" }, { id: "b1" }, { id: "a2" }, { id: "b2" }], params: { l1: 10e-3, l2: 10e-3, k: 0.9, dcr1: 1, dcr2: 1 } },
      resistor("r_t2", 100),
      // n-JFET common source
      { id: "q_nj", kind: "njfet", pins: [{ id: "d" }, { id: "g" }, { id: "s" }], params: { vto: -2, idss: 0.01 } },
      resistor("r_njd", 1000),
      resistor("r_njg", 100_000),
      resistor("r_njs", 330),
      // p-JFET mirror
      { id: "q_pj", kind: "pjfet", pins: [{ id: "d" }, { id: "g" }, { id: "s" }], params: { vto: -2, idss: 0.01 } },
      resistor("r_pjd", 1000),
      resistor("r_pjg", 100_000),
      resistor("r_pjs", 330),
      // SCR (default vgt 0.7 / igt 5 mA / ih 5 mA / vtm 1.2 / ron 0.1)
      { id: "scr1", kind: "scr", pins: [{ id: "a" }, { id: "k" }, { id: "g" }], params: {} },
      resistor("r_sa", 100),
      resistor("r_sg", 470),
      // Triac (same defaults)
      { id: "tr1", kind: "triac", pins: [{ id: "mt1" }, { id: "mt2" }, { id: "g" }], params: {} },
      resistor("r_ta", 100),
      resistor("r_tg", 470),
      // Optocoupler (defaults ctr 1 / vf 1.2 / iRated 20 mA / betaF 100)
      { id: "u_opto", kind: "opto_npn", pins: [{ id: "led_a" }, { id: "led_k" }, { id: "c" }, { id: "e" }], params: {} },
      resistor("r_ol", 330),
      resistor("r_oc", rOptoCollector),
      // Crystal
      { id: "xt1", kind: "crystal", pins: [{ id: "a" }, { id: "b" }], params: { fSeries: 1e6, q: 50_000, rs: 50, c0: 5 } },
      resistor("r_xd", 1000),
      // Analog switch
      { id: "sw1", kind: "analog_switch", pins: [{ id: "a" }, { id: "b" }, { id: "ctrl" }], params: { ron: 100, roff: 1e9 } },
      resistor("r_swa", 1000),
      resistor("r_swb", 1000),
      // Op-amp follower on a divider
      resistor("r_d1", 10_000),
      resistor("r_d2", 10_000),
      { id: "oa1", kind: "lm358", pins: [
        { id: "1" }, { id: "2" }, { id: "3" }, { id: "4" },
        { id: "5" }, { id: "6" }, { id: "7" }, { id: "8" },
      ], params: {} },
    ],
    wires: [
      // Transformer
      { from_component: "bat", from_pin: "pos", to_component: "r_t1", to_pin: "a" },
      { from_component: "r_t1", from_pin: "b", to_component: "xfmr", to_pin: "a1" },
      { from_component: "xfmr", from_pin: "b1", to_component: "bat", to_pin: "neg" },
      { from_component: "xfmr", from_pin: "a2", to_component: "r_t2", to_pin: "a" },
      { from_component: "r_t2", from_pin: "b", to_component: "bat", to_pin: "neg" },
      { from_component: "xfmr", from_pin: "b2", to_component: "bat", to_pin: "neg" },
      // n-JFET
      { from_component: "bat", from_pin: "pos", to_component: "r_njd", to_pin: "a" },
      { from_component: "r_njd", from_pin: "b", to_component: "q_nj", to_pin: "d" },
      { from_component: "q_nj", from_pin: "g", to_component: "r_njg", to_pin: "a" },
      { from_component: "r_njg", from_pin: "b", to_component: "bat", to_pin: "neg" },
      { from_component: "q_nj", from_pin: "s", to_component: "r_njs", to_pin: "a" },
      { from_component: "r_njs", from_pin: "b", to_component: "bat", to_pin: "neg" },
      // p-JFET
      { from_component: "bat", from_pin: "pos", to_component: "r_pjs", to_pin: "a" },
      { from_component: "r_pjs", from_pin: "b", to_component: "q_pj", to_pin: "s" },
      { from_component: "q_pj", from_pin: "g", to_component: "r_pjg", to_pin: "a" },
      { from_component: "r_pjg", from_pin: "b", to_component: "bat", to_pin: "pos" },
      { from_component: "q_pj", from_pin: "d", to_component: "r_pjd", to_pin: "a" },
      { from_component: "r_pjd", from_pin: "b", to_component: "bat", to_pin: "neg" },
      // SCR
      { from_component: "bat", from_pin: "pos", to_component: "r_sa", to_pin: "a" },
      { from_component: "r_sa", from_pin: "b", to_component: "scr1", to_pin: "a" },
      { from_component: "scr1", from_pin: "k", to_component: "bat", to_pin: "neg" },
      { from_component: "bat", from_pin: "pos", to_component: "r_sg", to_pin: "a" },
      { from_component: "r_sg", from_pin: "b", to_component: "scr1", to_pin: "g" },
      // Triac
      { from_component: "bat", from_pin: "pos", to_component: "r_ta", to_pin: "a" },
      { from_component: "r_ta", from_pin: "b", to_component: "tr1", to_pin: "mt2" },
      { from_component: "tr1", from_pin: "mt1", to_component: "bat", to_pin: "neg" },
      { from_component: "bat", from_pin: "pos", to_component: "r_tg", to_pin: "a" },
      { from_component: "r_tg", from_pin: "b", to_component: "tr1", to_pin: "g" },
      // Optocoupler
      { from_component: "bat", from_pin: "pos", to_component: "r_ol", to_pin: "a" },
      { from_component: "r_ol", from_pin: "b", to_component: "u_opto", to_pin: "led_a" },
      { from_component: "u_opto", from_pin: "led_k", to_component: "bat", to_pin: "neg" },
      { from_component: "bat", from_pin: "pos", to_component: "r_oc", to_pin: "a" },
      { from_component: "r_oc", from_pin: "b", to_component: "u_opto", to_pin: "c" },
      { from_component: "u_opto", from_pin: "e", to_component: "bat", to_pin: "neg" },
      // Crystal
      { from_component: "bat", from_pin: "pos", to_component: "r_xd", to_pin: "a" },
      { from_component: "r_xd", from_pin: "b", to_component: "xt1", to_pin: "a" },
      { from_component: "xt1", from_pin: "b", to_component: "bat", to_pin: "neg" },
      // Analog switch
      { from_component: "bat", from_pin: "pos", to_component: "r_swa", to_pin: "a" },
      { from_component: "r_swa", from_pin: "b", to_component: "sw1", to_pin: "a" },
      { from_component: "sw1", from_pin: "b", to_component: "r_swb", to_pin: "a" },
      { from_component: "r_swb", from_pin: "b", to_component: "bat", to_pin: "neg" },
      { from_component: "sw1", from_pin: "ctrl", to_component: "bat", to_pin: "pos" },
      // Op-amp follower
      { from_component: "bat", from_pin: "pos", to_component: "r_d1", to_pin: "a" },
      { from_component: "r_d1", from_pin: "b", to_component: "r_d2", to_pin: "a" },
      { from_component: "r_d2", from_pin: "b", to_component: "bat", to_pin: "neg" },
      { from_component: "r_d1", from_pin: "b", to_component: "oa1", to_pin: "3" },
      { from_component: "oa1", from_pin: "1", to_component: "oa1", to_pin: "2" },
      { from_component: "bat", from_pin: "pos", to_component: "oa1", to_pin: "8" },
      { from_component: "bat", from_pin: "neg", to_component: "oa1", to_pin: "4" },
    ],
  };
}

/**
 * Vacuous-pass guard: every family must sit in the live operating region the
 * fixture was designed for. A block that quietly stamped nothing (or a latch
 * that never fired) would still "converge", so convergence alone proves too
 * little — these are the hand-computed bias points from the fixture header.
 */
function expectMegaSteadyState(engine: SimEngine, label: string): void {
  const elementI = engine.getElementI();
  // Transformer primary at DC: 5 V across 100R + dcr1 = 1R.
  expect(elementI["xfmr"], `${label}: xfmr winding 1 current`).toBeGreaterThan(0.04);
  expect(elementI["xfmr"], `${label}: xfmr winding 1 current`).toBeLessThan(0.06);
  // SCR latched and conducting forward; triac latched with pol = -1, so its
  // mt1 -> mt2 element current reads negative.
  expect(elementI["scr1"], `${label}: scr principal current`).toBeGreaterThan(0.03);
  expect(elementI["scr1"], `${label}: scr principal current`).toBeLessThan(0.05);
  expect(elementI["tr1"], `${label}: triac principal current`).toBeGreaterThan(-0.05);
  expect(elementI["tr1"], `${label}: triac principal current`).toBeLessThan(-0.03);
  // Opto LED biased near 11.5 mA and the output transistor saturated.
  expect(elementI["u_opto"], `${label}: opto LED current`).toBeGreaterThan(0.008);
  expect(elementI["u_opto"], `${label}: opto LED current`).toBeLessThan(0.016);
  expect(netV(engine, "u_opto", "c"), `${label}: opto collector saturated`).toBeLessThan(1.0);
  // Both JFETs in saturation at the self-bias point (~2.85 mA).
  expect(netV(engine, "q_nj", "d"), `${label}: njfet drain`).toBeGreaterThan(1.2);
  expect(netV(engine, "q_nj", "d"), `${label}: njfet drain`).toBeLessThan(3.2);
  expect(netV(engine, "q_pj", "d"), `${label}: pjfet drain`).toBeGreaterThan(1.8);
  expect(netV(engine, "q_pj", "d"), `${label}: pjfet drain`).toBeLessThan(3.8);
  // Analog switch ON: b pin at the 1000/2100 ladder tap.
  expect(netV(engine, "sw1", "b"), `${label}: analog switch tap`).toBeGreaterThan(2.2);
  expect(netV(engine, "sw1", "b"), `${label}: analog switch tap`).toBeLessThan(2.6);
  // Crystal DC-open: its top pin reads the rail through the 1k feed.
  expect(netV(engine, "xt1", "a"), `${label}: crystal top pin at rail`).toBeGreaterThan(4.5);
  // Op-amp follower of the 2.5 V divider.
  expect(netV(engine, "oa1", "1"), `${label}: op-amp follower out`).toBeGreaterThan(2.35);
  expect(netV(engine, "oa1", "1"), `${label}: op-amp follower out`).toBeLessThan(2.65);
}

describe("mega-fixture — every A6 kind in one converging circuit", () => {
  it("converges through 100 backward-Euler steps into the designed bias points", () => {
    const engine = new SimEngine();
    engine.load(megaFixture());
    runConverged(engine, MEGA_STEPS, MEGA_DT, "mega BE");
    expectMegaSteadyState(engine, "mega BE");
    // Transformer secondary: coupling is pure d/dt, so its transient must
    // have decayed to the zero-DC answer — a nonzero i2 here means phantom
    // steady-state flux crossed the mutual block.
    const i2 = engine.saveState().inds.get("xfmr:2");
    expect(i2, "mega BE: secondary winding state present").toBeDefined();
    expect(Math.abs(i2!), "mega BE: secondary DC current decays to zero").toBeLessThan(1e-3);
  });

  it("converges through 100 trapezoidal steps into the same bias points", () => {
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(megaFixture());
    runConverged(engine, MEGA_STEPS, MEGA_DT, "mega trap");
    expectMegaSteadyState(engine, "mega trap");
  });

  it("two fresh engines replay the mega fixture bit-identically", () => {
    // The A6 modules keep module-scope composite-key caches shared by every
    // engine in the process; a second instance must reuse them without any
    // numeric consequence.
    const engineA = new SimEngine();
    engineA.load(megaFixture());
    const framesA = stepFrames(engineA, MEGA_STEPS, MEGA_DT);
    const engineB = new SimEngine();
    engineB.load(megaFixture());
    const framesB = stepFrames(engineB, MEGA_STEPS, MEGA_DT);
    expectIdenticalFrames(
      "mega fixture fresh-engine determinism (shared module-scope state diverged)",
      framesA,
      framesB,
    );
  });

  it("dcOperatingPoint converges with every family regime-settled", () => {
    const engine = new SimEngine();
    engine.load(megaFixture());
    const op = engine.dcOperatingPoint();
    expect(op.converged, `OP must converge (method=${op.method})`).toBe(true);
    for (const [netId, value] of Object.entries(op.netV)) {
      expect(Number.isFinite(value), `OP netV[${netId}] finite`).toBe(true);
    }
    // The regime settle loop must have latched the thyristors at the OP too:
    // a blocking SCR/triac would leave the anode/mt2 nodes at the rail.
    expect(op.netV[netIdFor(engine, "scr1", "a")]!).toBeLessThan(3);
    expect(op.netV[netIdFor(engine, "tr1", "mt2")]!).toBeLessThan(3);
    expect(op.netV[netIdFor(engine, "oa1", "1")]!).toBeGreaterThan(2.35);
    expect(op.netV[netIdFor(engine, "oa1", "1")]!).toBeLessThan(2.65);
  });

  it("runSmallSignalAc returns finite responses at 10 frequencies", () => {
    const engine = new SimEngine();
    engine.load(megaFixture());
    const frequenciesHz = logSpacedHz(10, 10, 1e6);
    // First output is the analog-switch ladder tap: purely resistive from the
    // driven rail, so its response (~0.476 flat) is the liveliness guard — a
    // sweep that silently stamped nothing cannot show it.
    const outputPins: ReadonlyArray<readonly [string, string]> = [
      ["sw1", "b"],
      ["xfmr", "a2"],
      ["u_opto", "c"],
      ["xt1", "a"],
      ["q_nj", "d"],
      ["oa1", "1"],
    ];
    const ac = runSmallSignalAc(engine, {
      inputId: "bat",
      outputNetIds: outputPins.map(([c, p]) => netIdFor(engine, c, p)),
      frequenciesHz,
    });
    expect(ac.frequenciesHz).toHaveLength(frequenciesHz.length);
    expect(ac.illConditionedFrequenciesHz).toEqual([]);
    for (const out of ac.outputs) {
      expect(out.re, `points for ${out.netId}`).toHaveLength(frequenciesHz.length);
      expect(out.re.every(Number.isFinite), `finite re for ${out.netId}`).toBe(true);
      expect(out.im.every(Number.isFinite), `finite im for ${out.netId}`).toBe(true);
    }
    const ladder = ac.outputs[0]!;
    for (let i = 0; i < frequenciesHz.length; i++) {
      expect(
        Math.hypot(ladder.re[i]!, ladder.im[i]!),
        `ladder tap responds at ${frequenciesHz[i]!} Hz`,
      ).toBeGreaterThan(0.1);
    }
  });
});

// ─── Part 4: dense/sparse backend equivalence on the mega-fixture ────────────

/** Nets watched across the backend runs — one per A6 family plus the op-amp. */
const MEGA_PROBES: ReadonlyArray<readonly [string, string]> = [
  ["xfmr", "a2"],
  ["q_nj", "d"],
  ["q_pj", "d"],
  ["scr1", "a"],
  ["tr1", "mt2"],
  ["u_opto", "c"],
  ["xt1", "a"],
  ["sw1", "b"],
  ["oa1", "1"],
];

/**
 * Backend agreement bound. Tighter than the 1e-6 V corpus-ladder bound in
 * linear-backend-equivalence.test.ts on purpose: the mega-fixture settles to
 * a stationary bias within a few time constants, so Newton's final iterates
 * differ only by elimination-order roundoff, not by accumulated transient
 * phase — and a committed-regime flip (SCR/triac/switch latch disagreeing
 * between backends) would blow this bound by six orders of magnitude, which
 * is exactly the failure this test exists to catch.
 */
const BACKEND_TOLERANCE_V = 1e-9;

describe("mega-fixture — dense and sparse backends agree within 1e-9 V", () => {
  // The hook is global state; a failed assertion mid-test must never leak a
  // forced backend into other suites running in this worker (the
  // linear-backend-equivalence.test.ts rule).
  afterEach(() => {
    setLinearSystemBackendForTests(null);
  });

  function runBackendTrajectory(backend: "dense" | "sparse"): number[][] {
    setLinearSystemBackendForTests(backend);
    const engine = new SimEngine();
    engine.load(megaFixture());
    const netIds = MEGA_PROBES.map(([c, p]) => netIdFor(engine, c, p));
    const traces: number[][] = MEGA_PROBES.map(() => []);
    for (let step = 0; step < MEGA_STEPS; step++) {
      engine.step(MEGA_DT);
      expect(engine.lastConverged, `${backend} step ${step + 1}: must converge`).toBe(true);
      const v = engine.getNetV();
      for (let probe = 0; probe < netIds.length; probe++) {
        const value = v[netIds[probe]!];
        expect(
          value !== undefined && Number.isFinite(value),
          `${backend} step ${step + 1}: probe ${netIds[probe]!} finite`,
        ).toBe(true);
        traces[probe]!.push(value!);
      }
    }
    return traces;
  }

  it("probe voltages diverge at most 1e-9 V across 100 steps", () => {
    const dense = runBackendTrajectory("dense");
    const sparse = runBackendTrajectory("sparse");
    // Liveliness: the ladder tap probe must be at its designed bias, or a
    // dead engine would trivially satisfy the divergence bound.
    expect(dense[7]![MEGA_STEPS - 1]!).toBeGreaterThan(2.2);
    expect(dense[7]![MEGA_STEPS - 1]!).toBeLessThan(2.6);
    let worst = 0;
    for (let probe = 0; probe < dense.length; probe++) {
      for (let step = 0; step < MEGA_STEPS; step++) {
        worst = Math.max(worst, Math.abs(dense[probe]![step]! - sparse[probe]![step]!));
      }
    }
    expect(worst).toBeLessThanOrEqual(BACKEND_TOLERANCE_V);
  });
});

// ─── Part 5: snapshot round trip and mid-run rewind ──────────────────────────

describe("mega-fixture — saveState/restoreState is bitwise", () => {
  // Both methods on purpose: BE snapshots omit the trap histories, trap
  // snapshots carry the composite-key capsI/indsV entries the crystal and
  // transformer maintain — the rewind must be exact in each regime.
  for (const method of ["be", "trap"] as const) {
    it(`${method}: restore round-trips bitwise and a mid-run rewind replays bitwise`, () => {
      const engine = new SimEngine();
      engine.setIntegrationMethod(method);
      engine.load(megaFixture());
      runConverged(engine, 40, MEGA_DT, `mega rewind ${method} (lead-in)`);

      const preBranch = serializeFrame(engine);
      const snap = engine.saveState();
      const firstRun = stepFrames(engine, 30, MEGA_DT);

      engine.restoreState(snap);
      // Round trip first: restoreState must reconstruct EXACTLY the committed
      // state saveState captured — element maps (composite keys included),
      // netV/elementI/digitalState, the MNA vector, and diagnostics.
      expect(serializeFrame(engine), `${method}: snapshot round trip`).toBe(preBranch);

      const replay = stepFrames(engine, 30, MEGA_DT);
      expectIdenticalFrames(
        `mega rewind ${method} (rollback replay diverged — snapshot missed A6 state)`,
        firstRun,
        replay,
      );
    });
  }
});

// ─── Part 6: same-id load() carry across a benign edit ───────────────────────

describe("mega-fixture — benign edit carries A6 composite-key state (invariant 4)", () => {
  it("same-id transformer keeps winding currents; crystal keeps branch state", () => {
    const engine = new SimEngine();
    engine.load(megaFixture());
    runConverged(engine, 50, MEGA_DT, "carry lead-in");

    const before = engine.saveState();
    const i1 = before.inds.get("xfmr:1");
    const i2 = before.inds.get("xfmr:2");
    const ls = before.inds.get("xt1:ls");
    const cs = before.caps.get("xt1:cs");
    const c0 = before.caps.get("xt1:c0");
    // The primary must be carrying real current or the carry check is vacuous.
    expect(i1, "primary winding state").toBeDefined();
    expect(i1!).toBeGreaterThan(0.04);
    expect(i2, "secondary winding state").toBeDefined();
    for (const [key, value] of [["xt1:ls", ls], ["xt1:cs", cs], ["xt1:c0", c0]] as const) {
      expect(value, `crystal state ${key}`).toBeDefined();
      expect(Number.isFinite(value!), `crystal state ${key} finite`).toBe(true);
    }

    // Benign edit: only the opto collector resistor changes; the transformer
    // and crystal keep their ids, kinds, and pins, so load() must carry their
    // composite-key state forward bit-exactly (Object.is, not toBeCloseTo).
    engine.load(megaFixture({ rOptoCollector: 1200 }));
    const after = engine.saveState();
    expect(Object.is(after.inds.get("xfmr:1"), i1), "xfmr:1 carried bitwise").toBe(true);
    expect(Object.is(after.inds.get("xfmr:2"), i2), "xfmr:2 carried bitwise").toBe(true);
    expect(Object.is(after.inds.get("xt1:ls"), ls), "xt1:ls carried bitwise").toBe(true);
    expect(Object.is(after.caps.get("xt1:cs"), cs), "xt1:cs carried bitwise").toBe(true);
    expect(Object.is(after.caps.get("xt1:c0"), c0), "xt1:c0 carried bitwise").toBe(true);

    // The edited circuit must keep solving from the carried state.
    runConverged(engine, 10, MEGA_DT, "carry post-edit");
    expectMegaSteadyState(engine, "carry post-edit");
  });
});
