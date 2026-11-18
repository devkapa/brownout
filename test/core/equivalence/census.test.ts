/**
 * G-0 item 4: the decline census gives the known counts on a known pair of
 * boards (test/fixtures/census: one the core takes, one holding a diode and a
 * BJT), sorts its lists, merges, and its CLI prints the same numbers as JSON.
 */
import { spawnSync } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import { loadBrownoutFixtureCorpus } from "./brownout-fixtures.js";
import { loadBoardsDir, mergeCensus, renderCensus, runCensus, runCensusOn, type CensusResult } from "./census.js";
import { SKELETONS } from "./skeleton-fixtures.js";
import { loadSyntheticCorpus } from "./synthetic-fixtures.js";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "..");
const PAIR_DIR = path.join(ROOT, "test", "fixtures", "census");

beforeAll(async () => {
  await initCore();
});

describe("decline census", () => {
  it("counts the known pair of boards: one accepted, one declined for its diode and its BJT", () => {
    const result = runCensusOn({ fixtures: false, boardDirs: [PAIR_DIR] });
    expect(result.circuits).toBe(2);
    expect(result.accepted).toBe(1);
    expect(result.declined).toBe(1);
    expect(result.byReason).toEqual([
      { key: "unsupported-kind:bjt_npn", circuits: 1 },
      { key: "unsupported-kind:diode", circuits: 1 },
    ]);
    expect(result.byMissingKind).toEqual([
      { key: "bjt_npn", circuits: 1 },
      { key: "diode", circuits: 1 },
    ]);
    expect(result.byPackage).toEqual([
      { key: "W3-D1", circuits: 1 },
      { key: "W3-T", circuits: 1 },
    ]);
    expect(result.bySource).toEqual([{ source: `boards:${PAIR_DIR}`, circuits: 2, accepted: 1 }]);
    expect(result.declinedList.map((d) => d.id)).toEqual(["declines-diode-and-bjt"]);
  });

  it("sorts most circuits first, ties by name, and counts a circuit once per reason", () => {
    const boards = loadBoardsDir(PAIR_DIR);
    const bad = boards.find((b) => b.id === "declines-diode-and-bjt")!;
    const result = runCensus([bad, { ...bad, id: "again" }, ...boards.filter((b) => b.id === "accepts-divider")]);
    expect(result.circuits).toBe(3);
    expect(result.byReason.map((r) => r.circuits)).toEqual([2, 2]);
    expect(result.byReason.map((r) => r.key)).toEqual(["unsupported-kind:bjt_npn", "unsupported-kind:diode"]);
  });

  it("merges two results by adding the counts", () => {
    const a = runCensusOn({ fixtures: false, boardDirs: [PAIR_DIR] });
    const merged: CensusResult = mergeCensus([a, a]);
    expect(merged.circuits).toBe(4);
    expect(merged.byReason[0]!.circuits).toBe(2);
    expect(renderCensus(merged)).toContain("Decline census: 4 circuits, 2 on the core, 2 declined");
  });

  it("covers the fixtures by default: brownout's, the synthetic ones and every skeleton, with every skeleton's kind among the missing kinds", () => {
    const result = runCensusOn({});
    expect(result.circuits).toBe(loadBrownoutFixtureCorpus().length + loadSyntheticCorpus().length + SKELETONS.length);
    expect(result.bySource.map((s) => s.source)).toEqual(["brownout-fixture", "skeleton", "synthetic"]);
    const missing = new Map(result.byMissingKind.map((k) => [k.key, k.circuits]));
    for (const skeleton of SKELETONS) expect(missing.get(skeleton.kind) ?? 0, skeleton.kind).toBeGreaterThanOrEqual(1);
    expect(result.byReason.every((r, i, all) => i === 0 || all[i - 1]!.circuits >= r.circuits)).toBe(true);
  });

  it("rejects a directory that is not one", () => {
    expect(() => loadBoardsDir(path.join(PAIR_DIR, "nope"))).toThrow(/not a directory/);
  });

  it("prints the same counts as JSON from the command line", () => {
    const run = spawnSync("pnpm", ["exec", "tsx", "scripts/decline-census.ts", "--no-fixtures", "--json", PAIR_DIR], { cwd: ROOT, encoding: "utf8", timeout: 120_000 });
    expect(run.status, `${run.stdout}\n${run.stderr}`).toBe(0);
    const parsed = JSON.parse(run.stdout) as CensusResult;
    expect(parsed.circuits).toBe(2);
    expect(parsed.byReason).toEqual([
      { key: "unsupported-kind:bjt_npn", circuits: 1 },
      { key: "unsupported-kind:diode", circuits: 1 },
    ]);
  }, 130_000);
});
