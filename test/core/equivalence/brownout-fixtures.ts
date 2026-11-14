/**
 * Corpus source 1 (the worker brief's "Corpus sources"): brownout's own
 * `test/fixtures/circuits/*.sim.json` boards. Every one of these predates
 * the core (they exercise BJTs, logic ICs, NE555s, shift registers, ...), so
 * in W0 every single one is expected to land on "not-ported" — the harness
 * still loads and classifies them so the per-kind table's `fixtures`/
 * `not-ported` columns are accurate, and so the day a kind here gets
 * implemented, this corpus starts exercising it for free.
 */

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { kindsOf } from "./kinds-of.js";
import { detectOpenInput } from "./open-input.js";
import type { CorpusEntry } from "./types.js";

const FIXTURES_DIR = path.resolve(
  path.dirname(fileURLToPath(import.meta.url)),
  "..",
  "..",
  "fixtures",
  "circuits",
);

export interface LoadedEntry {
  entry: CorpusEntry;
  circuit: SimCircuit;
}

export function loadBrownoutFixtureCorpus(): LoadedEntry[] {
  const files = fs
    .readdirSync(FIXTURES_DIR)
    .filter((file) => file.endsWith(".sim.json"))
    .sort();

  return files.map((file) => {
    const full = path.join(FIXTURES_DIR, file);
    const circuit = JSON.parse(fs.readFileSync(full, "utf8")) as SimCircuit;
    const name = file.replace(/\.sim\.json$/, "");
    const entry: CorpusEntry = {
      id: `brownout-fixture/${name}`,
      origin: "brownout-fixture",
      name,
      durationS: null,
      kinds: kindsOf(circuit),
      stimulus: null,
      file: full,
      // Finding 8 (review): always derived from the circuit, never a manual
      // flag — see open-input.ts's own doc and CorpusEntry.openInput's doc
      // (types.ts). Every one of these boards predates the core (W0 header
      // comment above), so this only matters the day a kind here is ported.
      openInput: detectOpenInput(circuit),
    };
    return { entry, circuit };
  });
}
