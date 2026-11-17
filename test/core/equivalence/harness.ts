/**
 * Top-level orchestration: loads every requested corpus source, runs each
 * entry through run-entry.ts, and folds the results into the per-kind table.
 * equivalence.test.ts and scripts/core-equivalence.ts both call only this
 * module — everything else in test/core/equivalence/** is plumbing for it.
 */

import { getPartLibrary, setPartLibrary } from "../../../src/parts/part-library.js";
import type { PartDefinition } from "../../../src/circuit/types.js";
import { loadBrownoutFixtureCorpus } from "./brownout-fixtures.js";
import { loadExternalCorpus } from "./external-corpus.js";
import { loadSyntheticCorpus, type SyntheticExpectation } from "./synthetic-fixtures.js";
import { runCorpusEntry } from "./run-entry.js";
import { buildKindTable, type HarnessEntryOutcome } from "./table.js";
import type { CorpusEntry, EntryResult, KindRow } from "./types.js";

export interface SyntheticOutcome {
  entry: CorpusEntry;
  expected: SyntheticExpectation;
  result: EntryResult;
}

export interface HarnessRunOptions {
  /** External corpus directories (BROWNOUT_CORE_CORPUS, already split — see external-corpus.ts's parseCorpusDirsEnv). Empty by default: nothing outside this repo runs unless a caller explicitly asks. */
  corpusDirs?: readonly string[];
  /** Default true. Exposed for a caller (e.g. a self-test) that wants an isolated harness run without the whole brownout corpus. */
  includeBrownoutFixtures?: boolean;
  /** Default true, same rationale as includeBrownoutFixtures. */
  includeSyntheticFixtures?: boolean;
}

export interface HarnessRun {
  outcomes: HarnessEntryOutcome[];
  syntheticOutcomes: SyntheticOutcome[];
  kindTable: KindRow[];
}

export function runEquivalenceHarness(options: HarnessRunOptions = {}): HarnessRun {
  const outcomes: HarnessEntryOutcome[] = [];
  const syntheticOutcomes: SyntheticOutcome[] = [];

  if (options.includeBrownoutFixtures ?? true) {
    for (const { entry, circuit } of loadBrownoutFixtureCorpus()) {
      outcomes.push({ entry, result: runCorpusEntry(entry, circuit) });
    }
  }

  if (options.includeSyntheticFixtures ?? true) {
    for (const { entry, circuit, expected } of loadSyntheticCorpus()) {
      const result = runCorpusEntry(entry, circuit);
      outcomes.push({ entry, result });
      syntheticOutcomes.push({ entry, expected, result });
    }
  }

  for (const dir of options.corpusDirs ?? []) {
    const corpus = loadExternalCorpus(dir);
    // Inject this corpus's producer part library for exactly its own runs,
    // then restore whatever was active before — getPartLibrary()/try-finally
    // per the brief, so one corpus's catalog never leaks into the next
    // corpus's (or the local fixtures') resolution.
    const previous = getPartLibrary();
    setPartLibrary([...corpus.parts] as PartDefinition[]);
    try {
      for (const { entry, circuit } of corpus.entries) {
        outcomes.push({ entry, result: runCorpusEntry(entry, circuit) });
      }
    } finally {
      setPartLibrary([...previous] as PartDefinition[]);
    }
  }

  return { outcomes, syntheticOutcomes, kindTable: buildKindTable(outcomes) };
}
