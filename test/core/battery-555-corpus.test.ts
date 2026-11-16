/**
 * A 555 on a battery, on both engines, at several host cadences (CORPUS).
 *
 * For four weeks the core ran a 555's clock up to 48 % slow on a battery
 * board, depending on how often the host stopped the engine, and no gate saw
 * it: the identity check compares two builds at one cadence, and the
 * classic-against-core corpus held no 555 on a battery (fixed at 574767b).
 * The three boards (test/fixtures/core, 77 to 79) close that hole:
 *
 *   77  an astable on a battery pack with a decoupling capacitor, driving an LED
 *   78  the same kind of astable clocking a 74LS161, so a late or lost edge is a wrong count
 *   79  a monostable triggered by a push button mid-run
 *
 * Each sidecar derives its edge times by hand and carries them in its
 * `handBound` marker (ruling R122 (a)): the corpus harness judges the core
 * against them at the app's cadence (hand-bound.ts) and records classic's
 * distance. This file runs the same boards on both engines at the other host
 * stops, which the harness does not sample.
 *
 * What is held:
 *   - the core, at host stops of 1 us, 100 us, 1 ms and 1/60 s: the number of
 *     OUT edges and each edge's time against the hand value (0.25 % of the
 *     edge's time + 5 us: the rail's swing between the HIGH and LOW phases),
 *     and, against the 1 us run, each edge within tau_w0 of the capacitor's
 *     slope at its trigger (the bound sap1-clock-cadence.slow.test.ts uses);
 *   - the final 74LS161 count (board 78) equals the hand count at every cadence.
 * Classic is run at the same stops and its numbers are printed, not gated:
 * it is the engine that differs (a step-limited crossing search, and a count
 * that depends on the stop).
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";
import { HeadlessRunner } from "../../src/host/headless.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { handEdges, type HandEdges, type HandModel } from "./equivalence/hand-555.js";
import type { HandBoundSpec } from "./equivalence/types.js";

/** The discharge pin (core/README.md NE555 point 7), for the tau_w0 below. */
const R_DIS = 10;

const DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../fixtures/core");
const circuitOf = (name: string): SimCircuit => JSON.parse(fs.readFileSync(path.join(DIR, `${name}.sim.json`), "utf8")) as SimCircuit;
const sidecarOf = (name: string): { handBound: HandBoundSpec } => JSON.parse(fs.readFileSync(path.join(DIR, `${name}.expected.json`), "utf8"));

interface Edit { readonly t: number; readonly id: string; readonly params: Record<string, number> }
interface Edges { up: number[]; down: number[]; count?: number }

/** Core: advances to each host stop in turn and reads the 555's OUT edges off the engine's trace. */
function coreRun(circuit: SimCircuit, host: number, end: number, edits: readonly Edit[], counter?: string): Edges {
  const engine = new CoreEngine();
  try {
    engine.load(circuit, CORE_IMPLEMENTED_KINDS);
    engine.enableTrace(true);
    engine.clearTrace();
    const up: number[] = [];
    const down: number[] = [];
    const pending = [...edits];
    for (let k = 1; ; k++) {
      const t = Math.min(k * host, end);
      engine.advance(t);
      for (const ev of engine.readTrace()) {
        // A 555's one digital slot is keyed by its bare component id; the load-time level is no edge.
        if (ev.key !== "u1" || ev.t <= 0) continue;
        (ev.value === 1 ? up : down).push(ev.t);
      }
      engine.clearTrace();
      while (pending.length > 0 && pending[0]!.t <= t + 1e-12) {
        const edit = pending.shift()!;
        expect(engine.setParams([{ componentId: edit.id, params: edit.params }])).toBe(true);
      }
      if (t >= end) break;
    }
    return { up, down, count: counter ? engine.digitalState[counter] : undefined };
  } finally {
    engine.dispose();
  }
}

/** Classic: the same stops, its edges read at each accepted step. An edit is a reload, which carries the state. */
function classicRun(circuit: SimCircuit, host: number, end: number, edits: readonly Edit[], counter?: string): Edges {
  const runner = new HeadlessRunner();
  runner.load(circuit);
  const up: number[] = [];
  const down: number[] = [];
  let previous = runner.engine.digitalState["u1"];
  const pending = [...edits];
  let current = circuit;
  for (let k = 1; ; k++) {
    const t = Math.min(k * host, end);
    const span = t - runner.simTime;
    if (span > 1e-15) {
      runner.run({
        durationS: span,
        onSample: () => {
          const v = runner.engine.digitalState["u1"];
          if (v !== undefined && v !== previous) {
            (v === 1 ? up : down).push(runner.engine.simTime);
            previous = v;
          }
        },
      });
    }
    while (pending.length > 0 && pending[0]!.t <= t + 1e-12) {
      const edit = pending.shift()!;
      current = structuredClone(current);
      Object.assign(current.components.find((c) => c.id === edit.id)!.params, edit.params);
      runner.load(current);
    }
    if (t >= end) break;
  }
  // Classic's seed: OUT reads low from load for 76 us, then rises (README NE555 point 9; the harness's minIntervalS). Not an edge of the circuit.
  const seeded = (xs: number[]) => xs.filter((x) => x > 2e-4);
  return { up: seeded(up), down: seeded(down), count: counter ? runner.engine.digitalState[counter] : undefined };
}

/** W0 on a node's voltage, as sap1-clock-cadence.slow.test.ts has it: 1e-4 + 1e-3 |v|. */
const w0 = (v: number): number => 1e-4 + 1e-3 * Math.abs(v);

interface Board {
  readonly name: string;
  readonly end: number;
  readonly edits: readonly Edit[];
  readonly counter?: string;
}

const BOARDS: readonly Board[] = [
  { name: "77-battery-555-astable-led", end: 0.1, edits: [] },
  { name: "78-battery-555-clocks-161", end: 0.1, edits: [], counter: "c1" },
  {
    name: "79-battery-555-monostable-button",
    end: 0.4,
    edits: [
      { t: 0.2, id: "btn", params: { closed: 1 } },
      { t: 0.25, id: "btn", params: { closed: 0 } },
    ],
  },
];

/** tau_w0: the time the timing capacitor takes to cross W0 of its trigger level, the tighter of its trigger and threshold crossings, at the 5 V pack. */
function tauW0(model: HandModel): number {
  if (model.kind === "monostable") {
    // C charges from empty through R toward the rail: the threshold crossing at 2u/3 is the only one that moves a pulse's end.
    return w0((2 / 3) * 5) / ((5 - (2 / 3) * 5) / (model.rOhm * model.cF));
  }
  const { raOhm: ra, rbOhm: rb, cF: c } = model;
  const k = R_DIS / (R_DIS + ra);
  const rth = rb + (R_DIS * ra) / (R_DIS + ra);
  const trigger = w0(5 / 3) / ((5 / 3 - 5 * k) / (rth * c));
  const threshold = w0((2 * 5) / 3) / ((5 - (2 * 5) / 3) / ((ra + rb) * c));
  return Math.min(trigger, threshold);
}

const CADENCES = [
  { label: "100 us", host: 1e-4 },
  { label: "1 ms", host: 1e-3 },
  { label: "1/60 s", host: 1 / 60 },
] as const;

/** The largest |a_i - b_i| over the pairs both lists hold. */
const worst = (a: readonly number[], b: readonly number[]): number => Math.max(0, ...a.slice(0, b.length).map((x, i) => Math.abs(x - b[i]!)));
const fmt = (s: number): string => `${(s * 1e6).toFixed(1)} us`;

beforeAll(async () => {
  await initCore();
});

describe.each(BOARDS)("$name", (board) => {
  const circuit = circuitOf(board.name);
  const spec = sidecarOf(board.name).handBound;
  const hand: HandEdges = handEdges(spec.model, board.end);
  const tau = tauW0(spec.model);
  // A tolerance for each edge: the sidecar's own bound.
  const { relative, absoluteS } = spec.bound;
  const core: Record<string, Edges> = {};
  const classic: Record<string, Edges> = {};

  beforeAll(() => {
    core["1 us"] = coreRun(circuit, 1e-6, board.end, board.edits, board.counter);
    for (const { label, host } of CADENCES) {
      core[label] = coreRun(circuit, host, board.end, board.edits, board.counter);
      classic[label] = classicRun(circuit, host, board.end, board.edits, board.counter);
    }
  });

  it("the hand values are the sidecar's", () => {
    expect(hand.up.length).toBe(spec.hand.rises);
    expect(hand.down.length).toBe(spec.hand.falls);
    if (spec.hand.periodS !== undefined) expect(Math.abs(hand.periodS! - spec.hand.periodS)).toBeLessThan(1e-12);
    if (spec.hand.finalCount !== undefined) expect(hand.finalCount).toBe(spec.hand.finalCount);
  });

  it.each([{ label: "1 us" }, ...CADENCES])("core at host stops of $label: every OUT edge is where the hand value puts it", ({ label }) => {
    const got = core[label]!;
    expect(got.up.length, "rises").toBe(hand.up.length);
    expect(got.down.length, "falls").toBe(hand.down.length);
    for (const [name, g, h] of [["rise", got.up, hand.up], ["fall", got.down, hand.down]] as const) {
      g.forEach((t, i) => expect(Math.abs(t - h[i]!), `${name} ${i} at ${h[i]!.toExponential(4)} s: core ${t.toExponential(6)} s`).toBeLessThanOrEqual(relative * h[i]! + absoluteS));
    }
    if (board.counter) expect(got.count, "the count at the end").toBe(hand.finalCount);
  });

  it.each(CADENCES)("core at host stops of $label: every edge within tau_w0 of the 1 us run's, and the same count", ({ label }) => {
    const fine = core["1 us"]!;
    const got = core[label]!;
    expect(got.up.length, "rises").toBe(fine.up.length);
    expect(got.down.length, "falls").toBe(fine.down.length);
    const spread = Math.max(worst(got.up, fine.up), worst(got.down, fine.down));
    console.log(`${board.name}, core at ${label}: worst edge ${fmt(spread)} from the 1 us run = ${(spread / tau).toFixed(3)} tau_w0 (${fmt(tau)})`);
    expect(spread, `worst edge against tau_w0 ${tau.toExponential(3)} s`).toBeLessThanOrEqual(tau);
    if (board.counter) expect(got.count).toBe(fine.count);
  });

  it("classic at the same stops, recorded (not gated)", () => {
    for (const { label } of CADENCES) {
      const c = classic[label]!;
      const k = core[label]!;
      const d = (a: readonly number[], h: readonly number[]) => (a.length > 0 ? `${fmt(worst(a, h))}` : "none");
      console.log(
        `${board.name}, classic at ${label}: ${c.up.length} rises, ${c.down.length} falls` +
          (board.counter ? `, count ${c.count} (core ${k.count}, hand ${hand.finalCount})` : "") +
          `; worst edge from the hand value: rises ${d(c.up, hand.up)}, falls ${d(c.down, hand.down)}` +
          `; core's: rises ${d(k.up, hand.up)}, falls ${d(k.down, hand.down)}`,
      );
      expect(c.down.length).toBeGreaterThan(0);
    }
  });
});
