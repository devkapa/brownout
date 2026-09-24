/**
 * Corpus source 2 (the worker brief's "Corpus sources"): this harness's own
 * hand-written W0 fixtures under test/fixtures/core/, one pure-SimCircuit
 * `*.sim.json` per scenario plus a `*.expected.json` sidecar declaring the
 * outcome the gate test asserts against (equivalence.test.ts, assertion
 * (a)). A sidecar (rather than a field embedded in the .sim.json) keeps the
 * circuit files themselves plain SimCircuit JSON — the exact shape
 * HeadlessRunner/CoreEngine.load() accept — with no harness-only field for a
 * future reader of the fixture to wonder about.
 */

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { kindsOf } from "./kinds-of.js";
import type { CorpusEntry } from "./types.js";

const SYNTHETIC_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "fixtures", "core");

/** What equivalence.test.ts's assertion (a) checks a synthetic fixture against. */
export interface SyntheticExpectation {
  readonly status: "pass" | "core-declined";
  /** Only meaningful when status is "core-declined" — e.g. "topology:" for the two topology-rejection fixtures. */
  readonly reasonPrefix?: string;
}

export interface LoadedSyntheticEntry {
  entry: CorpusEntry;
  circuit: SimCircuit;
  expected: SyntheticExpectation;
}

export function loadSyntheticCorpus(): LoadedSyntheticEntry[] {
  const files = fs
    .readdirSync(SYNTHETIC_DIR)
    .filter((file) => file.endsWith(".sim.json"))
    .sort();

  return files.map((file) => {
    const full = path.join(SYNTHETIC_DIR, file);
    const name = file.replace(/\.sim\.json$/, "");
    const circuit = JSON.parse(fs.readFileSync(full, "utf8")) as SimCircuit;

    const expectedPath = path.join(SYNTHETIC_DIR, `${name}.expected.json`);
    if (!fs.existsSync(expectedPath)) {
      throw new Error(
        `loadSyntheticCorpus: ${file} has no matching ${name}.expected.json — every synthetic ` +
          "fixture must declare its expected outcome (equivalence.test.ts assertion (a)).",
      );
    }
    const expected = JSON.parse(fs.readFileSync(expectedPath, "utf8")) as SyntheticExpectation;

    const entry: CorpusEntry = {
      id: `synthetic/${name}`,
      origin: "synthetic",
      name,
      durationS: null,
      kinds: kindsOf(circuit),
      stimulus: null,
      file: full,
    };
    return { entry, circuit, expected };
  });
}
