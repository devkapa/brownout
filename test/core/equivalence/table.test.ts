/**
 * Unit coverage for buildKindTable's `unstimulated` column (KindRow's own
 * doc, and CorpusEntry.stimulus's doc): an entry devolt additionally checks
 * under scripted stimuli this corpus doesn't carry must be counted
 * separately, without changing whether it counts as ran/pass/fail.
 */

import { describe, expect, it } from "vitest";
import { buildKindTable } from "./table.js";
import type { CorpusEntry, EntryResult } from "./types.js";

function passResult(): EntryResult {
  return {
    status: "pass",
    kinds: ["resistor"],
    classicWallMs: 1,
    coreWallMs: 1,
    maxAbsErrorVolts: 0,
    maxRelErrorVolts: 0,
    maxAbsErrorAmps: 0,
    maxRelErrorAmps: 0,
  };
}

function entry(id: string, stimulus: "verify-script" | null): CorpusEntry {
  return {
    id,
    origin: "test",
    name: id,
    durationS: null,
    kinds: ["resistor"],
    stimulus,
    file: `/tmp/${id}.sim.json`,
  };
}

describe("buildKindTable", () => {
  it("counts entries whose stimulus is verify-script in the unstimulated column, without changing pass/ran/status", () => {
    const outcomes = [
      { entry: entry("a", "verify-script"), result: passResult() },
      { entry: entry("b", null), result: passResult() },
    ];

    const table = buildKindTable(outcomes);
    const row = table.find((r) => r.kind === "resistor");

    expect(row).toBeDefined();
    expect(row!.unstimulated).toBe(1);
    expect(row!.pass).toBe(2);
    expect(row!.ran).toBe(2);
    expect(row!.status).toBe("pass");
  });

  it("reports unstimulated as 0 when no entry for that kind uses scripted stimuli", () => {
    const outcomes = [{ entry: entry("a", null), result: passResult() }];
    const table = buildKindTable(outcomes);
    expect(table.find((r) => r.kind === "resistor")!.unstimulated).toBe(0);
  });
});
