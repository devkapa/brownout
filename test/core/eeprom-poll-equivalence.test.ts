/**
 * Classic and the core agree on a 28C16's DATA-polling bit with NO registered
 * divergence to explain a difference away (W2f). Fixture 29 latches a byte
 * and then reads the part three times inside its 0.5 ms write cycle (each read
 * a falling /OE), then once after the cycle ended. I/O6 must read high, low,
 * high on the three busy reads (one toggle per read cycle) and carry the
 * written bit afterwards, in both engines.
 *
 * "pass" with an empty `knownDivergences` means the comparison found no
 * mismatch at all, so the old `28c16-28c256-io6-write-poll-step-grid`
 * registration is not needed to make this pass.
 */

import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../src/core/wasm.js";
import { runCorpusEntry } from "./equivalence/run-entry.js";
import { replayStimuliOnClassic } from "./equivalence/stimuli.js";
import { loadSyntheticCorpus } from "./equivalence/synthetic-fixtures.js";

beforeAll(async () => {
  await initCore();
});

function fixture29() {
  const loaded = loadSyntheticCorpus().find((e) => e.entry.name.startsWith("29-"));
  if (!loaded) throw new Error("fixture 29 (28c16 write poll) is missing from test/fixtures/core");
  return loaded;
}

describe("28C16 DATA polling: classic and the core agree (fixture 29)", () => {
  it("passes with no mismatch and no known divergence, so nothing is explained away", () => {
    const { entry, circuit } = fixture29();
    const result = runCorpusEntry(entry, circuit);
    expect(result.status).toBe("pass");
    if (result.status === "pass") expect(result.knownDivergences).toEqual([]);
  });

  it("really exercises the toggle: I/O6's pull-down current is on, off, on over the three busy reads, off after the write", () => {
    const { entry, circuit } = fixture29();
    const samples = replayStimuliOnClassic(circuit, entry.phases!);
    // r_io6 pulls I/O6 to ground, so its current is non-zero exactly when the
    // chip drives I/O6 high. Read cycles end phases 3, 5 and 7; phase 9 is
    // the read after the write cycle completed (written bit 6 is 0).
    const driven = (phase: number) => Math.abs(samples.find((s) => s.phaseIndex === phase)!.elementI.r_io6!) > 1e-7;
    expect([3, 5, 7].map(driven)).toEqual([true, false, true]);
    expect(driven(9)).toBe(false);
  });
});
