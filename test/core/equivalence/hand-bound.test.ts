/**
 * Ruling R122 (a)'s marker (hand-bound.ts): a fixture opts in with `handBound`
 * and is then judged against its hand values. These guards hold the marker to
 * what it is for: it fails a core outside its bound, it changes nothing for a
 * fixture without it, it is refused where there is no 555 on a cell's rail,
 * and the per-kind table counts a hand-judged pass in its own column.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import { assertHandBoundApplies, judge, runHandBoundEntry, type Run } from "./hand-bound.js";
import { runCorpusEntry } from "./run-entry.js";
import { buildKindTable } from "./table.js";
import { loadSyntheticCorpus, type LoadedSyntheticEntry } from "./synthetic-fixtures.js";
import type { CorpusEntry, HandBoundSpec } from "./types.js";

let corpus: LoadedSyntheticEntry[];
const named = (prefix: string): LoadedSyntheticEntry => corpus.find((c) => c.entry.name.startsWith(prefix))!;

beforeAll(async () => {
  await initCore();
  corpus = loadSyntheticCorpus();
});

describe("the handBound marker (ruling R122 (a))", () => {
  it("the three battery 555 boards carry it and pass inside their bound, with classic's distance recorded", () => {
    for (const prefix of ["77-", "78-", "79-"]) {
      const { entry, circuit } = named(prefix);
      expect(entry.handBound, prefix).toBeDefined();
      const result = runCorpusEntry(entry, circuit);
      expect(result.status, prefix).toBe("pass");
      if (result.status !== "pass") continue;
      const report = result.handBound!;
      expect(report.coreBoundRatio, `${prefix}: the core's worst quantity against its bound`).toBeLessThanOrEqual(1);
      // Classic's distance is a divergence row with numbers, and it is outside the bound (the bound is not set to admit it).
      expect(result.knownDivergences.length, `${prefix}: classic's rows`).toBeGreaterThan(0);
      expect(result.knownDivergences.every((k) => k.divergenceId === "classic-555-on-cell-rail")).toBe(true);
      // The rail capacitor's reading is listed, not hidden and not judged: the core reads 0, classic does not.
      const reading = report.knownFailingReadings.find((k) => k.id === "cdec")!;
      expect(reading.core).toBe(0);
      expect(Math.abs(reading.classic)).toBeGreaterThan(1e-3);
    }
  });

  it("a core outside its bound fails", () => {
    const { entry, circuit } = named("77-");
    const tight: CorpusEntry = { ...entry, handBound: { ...entry.handBound!, bound: { relative: 1e-5, absoluteS: 0 } } };
    const result = runHandBoundEntry(tight, circuit);
    expect(result.status).toBe("fail");
    if (result.status === "fail") {
      expect(result.mismatches.some((m) => m.category === "sidecar" && m.id.startsWith("555 "))).toBe(true);
    }
  });

  it("a quantity that was not measured is a miss, not a pass", () => {
    // One rise and one fall hold no complete cycle, so the period and the duty are not measured (NaN). Before this
    // guard `|NaN - hand| > bound` was false and both passed.
    const spec = named("77-").entry.handBound!;
    const hand = { up: [5.0809e-3], down: [3.5156e-3], periodS: 3.7834e-3, duty: 0.5863, finalCount: 2 };
    const run: Run = { up: [5.0809e-3], down: [3.5156e-3], capCurrent: [] };
    const verdict = judge(run, { rises: 1, falls: 1, worstEdgeS: 0 }, spec, hand);
    expect(verdict.misses.map((m) => m.id)).toEqual(["period", "duty"]);
    expect(verdict.ratio).toBe(Infinity);
  });

  it("a hand value the model does not derive is a fixture error, not a verdict", () => {
    const { entry, circuit } = named("78-");
    const stale: CorpusEntry = { ...entry, handBound: { ...entry.handBound!, hand: { ...entry.handBound!.hand, finalCount: 8 } } };
    expect(() => runHandBoundEntry(stale, circuit)).toThrow(/final count/);
  });

  it("without the marker the same board still fails the classic-against-core comparison, as before", () => {
    const { entry, circuit } = named("77-");
    const bare: CorpusEntry = { ...entry, handBound: undefined };
    expect(runCorpusEntry(bare, circuit).status).toBe("fail");
  });

  it("the marker is refused on a 555 that is not on a cell's rail, and on anything else", () => {
    const spec = named("77-").entry.handBound!;
    // An ideal-supply 555 (fixture 31) and a circuit with no 555 at all (fixture 66).
    for (const prefix of ["31-", "66-"]) {
      const { circuit } = named(prefix);
      expect(() => assertHandBoundApplies(circuit, spec), prefix).toThrow(/refused/);
    }
    // The battery board with the 555 named wrongly, with the supply moved off the pack, and with no justification.
    const { circuit } = named("77-");
    expect(() => assertHandBoundApplies(circuit, { ...spec, component: "ra" })).toThrow(/not an ne555/);
    const apart = structuredClone(circuit);
    apart.wires = apart.wires.filter((w) => !(w.from_component === "u1" && w.from_pin === "8"));
    apart.components.push({ id: "vx", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } });
    apart.wires.push({ from_component: "u1", from_pin: "8", to_component: "vx", to_pin: "pos" }, { from_component: "vx", from_pin: "neg", to_component: "gnd", to_pin: "gnd" });
    expect(() => assertHandBoundApplies(apart, spec)).toThrow(/not on a battery_pack/);
    expect(() => assertHandBoundApplies(circuit, { ...spec, justification: " " })).toThrow(/justification/);
    const wrongClass = { ...spec, circuitClass: "anything" } as unknown as HandBoundSpec;
    expect(() => assertHandBoundApplies(circuit, wrongClass)).toThrow(/unknown circuit class/);
  });

  it("the per-kind table counts a hand-judged pass in its own column, and only that", () => {
    const outcomes = ["77-", "31-", "66-"].map((prefix) => {
      const { entry, circuit } = named(prefix);
      return { entry, result: runCorpusEntry(entry, circuit) };
    });
    const rows = new Map(buildKindTable(outcomes).map((r) => [r.kind, r]));
    expect(rows.get("ne555")!.handJudged).toBe(1);
    expect(rows.get("ne555")!.pass).toBe(2);
    expect(rows.get("battery_pack")!.handJudged).toBe(1);
    expect(rows.get("74ls47")!.handJudged).toBe(0);
  });
});
