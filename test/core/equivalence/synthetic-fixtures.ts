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
import { detectOpenInput } from "./open-input.js";
import type { StimulusPhase } from "./stimuli.js";
import type { AnalyticFixtureModel, CorpusEntry, DigitalFixtureSpec } from "./types.js";

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
  /**
   * Finding 8 (review): decision 9's open-input tag is now AUTO-DETECTED
   * from the circuit itself (`open-input.ts`'s `detectOpenInput`), the same
   * "always derived, never trusted" rule `kinds` already gets — this field
   * is kept only as a diagnostic the loader can cross-check against the
   * detector (a fixture that declares one value while the detector computes
   * the other is a stale fixture or a detector bug, either way worth
   * surfacing), never as the value actually placed on the CorpusEntry.
   */
  readonly openInput?: boolean;
  /**
   * Overrides `sampleTimesFor`'s default 10 ms window (sample.ts's
   * DEFAULT_DURATION_S) for this one fixture. `undefined` (every synthetic
   * fixture before W1c) keeps the old hardcoded-null behaviour. A W1c digital
   * fixture needs this for real: the default grid's last point sits AT
   * t=10ms exactly, which is also decision 9's floating-input bucket
   * boundary (`floor((t+1e-12)/0.01)` ticks from 0 to 1 exactly there) — an
   * open-input fixture that cares which bucket every sample lands in must be
   * able to choose a window that keeps all four default sample points
   * strictly inside one bucket, and a fixture with a real analog transition
   * (the 74HC14 relaxation oscillator) must be able to stay inside one
   * monotonic segment. `sampleTimesFor` itself is untouched; this only picks
   * WHICH duration it is called with.
   */
  readonly durationS?: number;
  /**
   * Gate 2a's per-cycle digital alignment (decision 6), threaded straight
   * onto the CorpusEntry the same way `analytic`/`phases` are — see
   * `CorpusEntry.digital`'s own doc (types.ts) and `runDigitalAlignment`
   * (run-entry.ts). `undefined` (every synthetic fixture before fixture 19)
   * means "compare digitalState per-sample, unaligned" (the plain flow).
   */
  readonly digital?: DigitalFixtureSpec;
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

    // Finding 8: always derived from the circuit (detectOpenInput), never
    // trusted from the fixture's own manual flag — see SyntheticExpectation.
    // openInput's own doc. A fixture whose manual flag disagrees with the
    // detector is loudly wrong rather than silently ignored, exactly like
    // external-corpus.ts's own manifestKindsMismatch treats a stale kinds list.
    const openInput = detectOpenInput(circuit);
    if (expected.openInput !== undefined && expected.openInput !== openInput) {
      throw new Error(
        `loadSyntheticCorpus: ${name}.expected.json declares openInput: ${String(expected.openInput)}, but ` +
          `detectOpenInput(circuit) computes ${String(openInput)} — the manual flag is stale (or the detector is ` +
          "wrong); fix the fixture's declaration or investigate the detector, never silently trust either.",
      );
    }

    const entry: CorpusEntry = {
      id: `synthetic/${name}`,
      origin: "synthetic",
      name,
      // `expected.durationS ?? null`: every W0/W1b fixture predating this
      // field has no time-varying behavior to exercise, so `null` (the
      // harness default, sample.ts's DEFAULT_DURATION_S) is still correct
      // for them; a W1c digital fixture that cares about its own sample
      // window sets this explicitly (see SyntheticExpectation.durationS's doc).
      durationS: expected.durationS ?? null,
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
      digital: expected.digital,
      openInput,
    };
    return { entry, circuit, expected };
  });
}
