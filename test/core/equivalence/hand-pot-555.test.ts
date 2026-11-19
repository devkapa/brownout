/**
 * P0-IC: the hand-judged class "555 timed by a potentiometer" (R122 (a)'s marker, hand-bound.ts), over the repository's
 * four circuits (test/fixtures/core-hand-judged, positions 0, 50, 99 and 100 %). The core's edges, period and duty are
 * held against the hand model (hand-555.ts's `PotAstableHandModel`) within 0.01 tau_w0, W0's band at each threshold over
 * the timing node's slope there (the bound test/core/pot-clock-555.test.ts holds the period to); classic's distance, which
 * is the step split of a low phase shorter than its host step, is recorded as rows and never asserted.
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { handEdges, potRheostatOhm, type PotAstableHandModel } from "./hand-555.js";
import { assertHandBoundApplies, CLASSIC_555_POT_CLOCK_ID, runHandBoundEntry } from "./hand-bound.js";
import type { CorpusEntry, HandBoundSpec } from "./types.js";

const HAND_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../../fixtures/core-hand-judged");
const circuitAt = (position: number): SimCircuit =>
  JSON.parse(fs.readFileSync(path.join(HAND_DIR, `555-pot-timing-${Math.round(position * 100)}pct.sim.json`), "utf8")) as SimCircuit;

const modelAt = (position: number): PotAstableHandModel => ({ kind: "astable-pot", raOhm: 1000, potTotalOhm: 1e6, position, cF: 1e-6, disOhm: 10 });

/** tau_w0 summed over the two crossings (the same derivation as pot-clock-555.test.ts's). */
function tauW0(position: number): number {
  const r2 = potRheostatOhm(modelAt(position));
  const c = 1e-6;
  const vd = (5 * 10) / 1010;
  const rth = r2 + (1000 * 10) / 1010;
  const w0 = (v: number): number => 1e-4 + 1e-3 * v;
  return w0(10 / 3) / (5 / 3 / ((1000 + r2) * c)) + w0(5 / 3) / ((5 / 3 - vd) / (rth * c));
}

/** The run's length: a few periods at every position (the per-period error adds up edge by edge, so the long runs stay short). */
const DURATION_S: Record<number, number> = { 0: 3.5, 0.5: 3.0, 0.99: 0.12, 1: 0.006 };

function entryAt(position: number, over: Partial<HandBoundSpec> = {}): { entry: CorpusEntry; circuit: SimCircuit } {
  const model = modelAt(position);
  const durationS = DURATION_S[position]!;
  const edges = handEdges(model, durationS);
  const handBound: HandBoundSpec = {
    circuitClass: "555 timed by a potentiometer",
    component: "u1",
    model,
    hand: { rises: edges.up.length, falls: edges.down.length, periodS: edges.periodS, duty: edges.duty },
    // 0.01 tau_w0 per period, accumulating edge by edge (an edge's allowance is its index in periods, the duty's the one period's).
    bound: { relative: (0.01 * tauW0(position)) / edges.periodS!, absoluteS: 0.01 * tauW0(position) },
    justification:
      "0.01 tau_w0: W0's band (100 uV + 1e-3 relative) at each threshold over the timing node's slope there, summed over the two crossings. " +
      "Measured on the core: within 3e-5 tau_w0 per period at every position (test/core/pot-clock-555.test.ts).",
    ...over,
  };
  const entry: CorpusEntry = {
    id: `hand-pot-555/${position}`,
    origin: "synthetic",
    name: `555 pot clock at ${position * 100} %`,
    durationS,
    kinds: ["ne555", "potentiometer", "capacitor", "resistor", "voltage_source"],
    stimulus: null,
    file: `555-pot-timing-${Math.round(position * 100)}pct.sim.json`,
    handBound,
  } as CorpusEntry;
  return { entry, circuit: circuitAt(position) };
}

beforeAll(async () => {
  await initCore();
});

describe("the hand-judged class \"555 timed by a potentiometer\"", () => {
  for (const position of [0, 0.5, 0.99, 1]) {
    it(`passes at ${position * 100} % inside 0.01 tau_w0, with classic's distance recorded and never asserted`, () => {
      const { entry, circuit } = entryAt(position);
      const result = runHandBoundEntry(entry, circuit);
      if (result.status !== "pass") throw new Error(JSON.stringify(result.status === "fail" ? result.mismatches : result).slice(0, 1500));
      expect(result.handBound!.coreBoundRatio).toBeLessThanOrEqual(1);
      expect(result.knownDivergences.every((k) => k.divergenceId === CLASSIC_555_POT_CLOCK_ID)).toBe(true);
      // At 100 % classic's low phase (7.7 us) is shorter than its host step: it is far outside the bound, and the rows say by how much.
      if (position === 1) expect(result.knownDivergences.length).toBeGreaterThan(0);
    });
  }

  it("rejects a wrong period: a hand model whose discharge pin is 100 ohm instead of 10 (the 100 % clock's low phase is then wrong by a factor of ten)", () => {
    const base = entryAt(1);
    const wrong = modelAt(1);
    const model = { ...wrong, disOhm: 100 };
    const edges = handEdges(model, DURATION_S[1]!);
    const right = handEdges(wrong, DURATION_S[1]!);
    expect(Math.abs(edges.periodS! - right.periodS!) / right.periodS!, "the wrong model is a clearly different period").toBeGreaterThan(0.05);
    const entry: CorpusEntry = {
      ...base.entry,
      handBound: { ...base.entry.handBound!, model, hand: { rises: edges.up.length, falls: edges.down.length, periodS: edges.periodS, duty: edges.duty } },
    };
    const result = runHandBoundEntry(entry, base.circuit);
    expect(result.status).toBe("fail");
    if (result.status === "fail") expect(result.mismatches.some((m) => m.id.startsWith("555 "))).toBe(true);
  });

  it("rejects a period the core does not keep: a bound a thousand times tighter fails the 50 % clock", () => {
    const { entry, circuit } = entryAt(0.5);
    const tight: CorpusEntry = { ...entry, handBound: { ...entry.handBound!, bound: { relative: (1e-5 * tauW0(0.5)) / handEdges(modelAt(0.5), 3).periodS!, absoluteS: 1e-5 * tauW0(0.5) } } };
    expect(runHandBoundEntry(tight, circuit).status).toBe("fail");
  });

  it("takes a sidecar that is not the model's for a fixture error, not a verdict", () => {
    const { entry, circuit } = entryAt(0.5);
    const stale: CorpusEntry = { ...entry, handBound: { ...entry.handBound!, hand: { ...entry.handBound!.hand, periodS: 0.7 } } };
    expect(() => runHandBoundEntry(stale, circuit)).toThrow(/period/);
  });

  it("is refused on a circuit that is not a pot-timed astable", () => {
    const { entry } = entryAt(0.5);
    const spec = entry.handBound!;
    // The model names another position than the circuit's; a circuit without the potentiometer; the cell-rail class's own fixture.
    expect(() => assertHandBoundApplies(circuitAt(0.99), spec)).toThrow(/potentiometer is .* the model's/);
    const bare = structuredClone(circuitAt(0.5));
    bare.components = bare.components.filter((c) => c.kind !== "potentiometer");
    expect(() => assertHandBoundApplies(bare, spec)).toThrow(/no potentiometer/);
    const wired = structuredClone(circuitAt(0.5));
    wired.wires = wired.wires.filter((w) => !(w.from_component === "pot" && w.from_pin === "ccw"));
    expect(() => assertHandBoundApplies(wired, spec)).toThrow(/not a rheostat/);
    expect(() => assertHandBoundApplies(circuitAt(0.5), { ...spec, model: { kind: "astable", raOhm: 1000, rbOhm: 5e5, cF: 1e-6, disOhm: 10 } })).toThrow(/not an astable-pot model/);
    expect(() => assertHandBoundApplies(circuitAt(0.5), { ...spec, component: "r1" })).toThrow(/not an ne555/);
    expect(() => assertHandBoundApplies(circuitAt(0.5), { ...spec, justification: " " })).toThrow(/justification/);
  });
});
