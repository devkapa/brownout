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
import type { StimulusPhase } from "./stimuli.js";
import type { AnalyticFixtureModel, CorpusEntry } from "./types.js";

const SYNTHETIC_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "fixtures", "core");

/** What equivalence.test.ts's assertion (a) checks a synthetic fixture against. */
export interface SyntheticExpectation {
  readonly status: "pass" | "core-declined";
  /** Only meaningful when status is "core-declined" — e.g. "topology:" for the two topology-rejection fixtures. */
  readonly reasonPrefix?: string;
  /** Gate 2e / decision 3's analytic reference, when this fixture declares one — threaded straight onto the CorpusEntry (types.ts's own doc); see AnalyticFixtureModel and run-entry.ts's use of it. `undefined` for every fixture without one. */
  readonly analytic?: AnalyticFixtureModel;
  /**
   * Finding 12: gate 2d's verify-script phases (stimuli.ts's `StimulusPhase[]`),
   * threaded straight onto the CorpusEntry the same way `analytic` is —
   * `undefined` (every synthetic fixture before this one) means "no
   * stimuli, use the plain fixed/dense sample grid" (`runCorpusEntry`'s own
   * `entry.phases && entry.phases.length > 0` check). A fixture that
   * declares this actually exercises `bo_engine_set_params` mid-run through
   * `runStimuliEntry`, instead of only ever comparing its circuit's static
   * declared param values.
   */
  readonly phases?: readonly StimulusPhase[];
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
      // Finding 12: a fixture with declared phases genuinely gets replayed
      // under scripted stimuli (runCorpusEntry routes to runStimuliEntry),
      // so it is not "unstimulated" the way CorpusEntry.stimulus's doc means
      // for an entry that only ever compares static declared param values —
      // but `stimulus` itself is the EXTERNAL-corpus "verify-script" marker
      // (types.ts: "null for every local fixture"), so this stays null here
      // regardless of whether `phases` is set.
      stimulus: null,
      file: full,
      analytic: expected.analytic,
      phases: expected.phases,
    };
    return { entry, circuit, expected };
  });
}
