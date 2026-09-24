/**
 * Barrel for the dual-engine equivalence harness library. equivalence.test.ts
 * (the gate) and scripts/core-equivalence.ts (the CLI) are the two intended
 * consumers; everything importable here is safe for both.
 */

export { runEquivalenceHarness } from "./harness.js";
export type { HarnessRun, HarnessRunOptions, SyntheticOutcome } from "./harness.js";

export type { HarnessEntryOutcome } from "./table.js";
export { buildKindTable } from "./table.js";

export type { SupportedKindOffense } from "./supported-kinds-gate.js";
export { checkSupportedKindsGate } from "./supported-kinds-gate.js";

export { renderMarkdownTable, renderEntryDetail, renderMarkdownReport, toJsonReport } from "./report.js";

export { runCorpusEntry } from "./run-entry.js";

export { compareSamples } from "./compare.js";

export {
  collectClassicSamples,
  collectCoreSamples,
  sampleTimesFor,
  DEFAULT_DURATION_S,
} from "./sample.js";

export { kindsOf, missingCoreKinds } from "./kinds-of.js";

export { loadBrownoutFixtureCorpus } from "./brownout-fixtures.js";
export type { LoadedEntry } from "./brownout-fixtures.js";

export { loadSyntheticCorpus } from "./synthetic-fixtures.js";
export type { LoadedSyntheticEntry, SyntheticExpectation } from "./synthetic-fixtures.js";

export { loadExternalCorpus, parseCorpusDirsEnv } from "./external-corpus.js";
export type { ExternalCorpus } from "./external-corpus.js";

export type {
  CorpusEntry,
  CorpusOrigin,
  SampleReadout,
  Mismatch,
  CompareResult,
  EntryStatus,
  EntryResult,
  NotPortedResult,
  CoreDeclinedResult,
  PassResult,
  FailResult,
  ErrorResult,
  KindRow,
} from "./types.js";

export {
  VOLT_ABS_TOL,
  VOLT_REL_TOL,
  AMP_ABS_TOL,
  AMP_REL_TOL,
  withinTolerance,
  withinVoltTolerance,
  withinAmpTolerance,
} from "./tolerance.js";
