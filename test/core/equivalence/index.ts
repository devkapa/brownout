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
  DigitalFixtureSpec,
  AnalogFixtureSpec,
  SampleReadout,
  Mismatch,
  CompareResult,
  EntryStatus,
  EntryResult,
  NotPortedResult,
  CoreDeclinedResult,
  CoreStimuliPendingResult,
  PassResult,
  FailResult,
  ErrorResult,
  KindRow,
  KnownDivergenceHit,
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

export {
  diffDigitalStates,
  collectClassicDigitalTrace,
  collectCoreDigitalTrace,
  hasCoreTraceSupport,
  stateAt,
  CoreTraceUnsupportedError,
} from "./trace.js";
export type { DigitalTraceEvent, CoreTraceCapable } from "./trace.js";

export {
  findRisingEdges,
  sampleCycles,
  alignCycles,
  compareAlignedCycles,
  requireMinComparedCycles,
  TooFewComparedCyclesError,
} from "./align.js";
export type { ProjectedState, CycleSample, AlignmentResult, AlignmentSuccess, AlignmentFailure, CycleMismatch, AlignedComparison } from "./align.js";

export {
  guardWindowS,
  eventInstantsFromTrace,
  selectQuietInstants,
  requireMinQuietInstants,
  assertInstantsAreQuiet,
  TooFewQuietInstantsError,
} from "./quiet.js";
export type { GuardWindowOptions } from "./quiet.js";

export { denseAnalogSchedule, mergeScheduleTimes } from "./schedule.js";
export type { DenseScheduleOptions } from "./schedule.js";

export { rcCrossingTime, ne555AstablePeriod, ne555MonostablePulseWidth, checkAnalyticCrossing } from "./analytic.js";
export type { Ne555AstableParams, Ne555AstableAnalytic, Ne555MonostableParams, AnalyticCheckResult } from "./analytic.js";

export {
  classicIdentityHash,
  classicSourceFiles,
  contentHashOfFiles,
  DEFAULT_COLLECTOR_SOURCE_FILES,
  resetClassicIdentityCacheForTest,
  fixtureHash,
  createMemoryReferenceCache,
  createFileReferenceCache,
  getOrCompute,
} from "./reference-cache.js";
export type { ClassicReferenceCacheKey, ClassicReferenceCache, ClassicIdentityInputs } from "./reference-cache.js";

export {
  STIMULUS_FIXED_STEP_S,
  replayStimuliOnClassic,
  replayStimuliOnCore,
  StimuliNotYetSupportedOnCoreError,
} from "./stimuli.js";
export type { StimulusPhase, StimulusPhaseSet, StimulusStepSample, StimulusScheduleSample } from "./stimuli.js";

export { KNOWN_DIVERGENCES, classifyMismatches, computeDivergenceCone, digitalOutputPins } from "./known-divergences.js";
export type { KnownDivergence, DivergenceSeed, DivergenceCone, DivergenceMatch, ClassifiedMismatches } from "./known-divergences.js";

export { W1G_RESET_FAILED_SEED, W1G_SAG_COMMITTED_TARGET, explainCommittedTarget, explainFailedResetSeed } from "./known-divergences.js";
export type { KnownTimedDivergence, PendingDelayWindow } from "./known-divergences.js";

export { compareFailureSets, classicFailureMessage } from "./w1g-failures.js";
export { runSlowChecks, slowEvents } from "./w1g-sidecars.js";
