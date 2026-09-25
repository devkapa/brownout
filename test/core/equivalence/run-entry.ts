/**
 * Runs one CorpusEntry through the harness's per-entry state machine (the
 * worker brief's "Harness semantics"): not-ported short-circuits before
 * either engine touches the circuit; a CoreUnsupportedError from the core
 * side is an expected fallback (core-declined), not a failure; anything else
 * that throws is an "error" entry; otherwise the two sample sequences are
 * compared and the entry is pass or fail.
 *
 * CORE RUNS FIRST. That matches production's own precedence — HeadlessRunner
 * in `engine: "core"` mode tries the core and falls back to classic, never
 * the other way (headless.ts's `_tryLoadCore`) — and it means a circuit the
 * core declines outright is reported as "core-declined" without classic
 * having to run (or survive its own convergence guard, sample.ts's
 * collectClassicSamples) first. Some topologies the core explicitly rejects
 * (a loop of ideal sources, a floating ideal source — core/README.md's W0
 * kernel semantics #1) are ALSO singular for classic's own MNA; running
 * classic only once the core has already accepted the circuit means that
 * guard only has to hold for entries a real comparison is actually going to
 * happen on.
 *
 * DIGITAL ALIGNMENT AND QUIET INSTANTS (gate 2, W1a). When an entry declares
 * `digital` (types.ts's `DigitalFixtureSpec`), its classic and core digital
 * traces (trace.ts) are additionally aligned per-cycle (align.ts) and any
 * cycle mismatch is folded into the SAME mismatch list the plain per-sample
 * `compareSamples` produces — one pass/fail decision per entry, not two. When
 * an entry declares `analog` (`AnalogFixtureSpec`), its sample times come
 * from `schedule.ts`'s dense grid filtered by `quiet.ts`'s guard window
 * instead of `sampleTimesFor`'s fixed 4 points. Neither field is set by any
 * loader in this worktree yet (no digital kind is `CORE_IMPLEMENTED_KINDS`,
 * so a `digital`-bearing entry would only ever reach "not-ported" today) —
 * this is deliberately forward-compatible plumbing, proven against synthetic
 * data in this package's own tests, for the sub-wave that first sets them.
 *
 * A `CoreTraceUnsupportedError` while collecting the core's digital trace
 * (the trace ABI has not landed in this CoreEngine build yet — this worker's
 * own Verify step: "run the classic-only paths, and re-run the core paths at
 * the end") downgrades to "digital dimension not checked this run", not a
 * circuit failure: the entry's status still reflects whatever the analog
 * comparison found.
 */

import os from "node:os";
import path from "node:path";
import { getPartLibrary } from "../../../src/parts/part-library.js";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { CoreUnsupportedError } from "../../../src/core/errors.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { alignCycles, compareAlignedCycles, sampleCycles } from "./align.js";
import { compareSamples } from "./compare.js";
import { classifyMismatches } from "./known-divergences.js";
import { missingCoreKinds } from "./kinds-of.js";
import { guardWindowS, requireMinQuietInstants, selectQuietInstants } from "./quiet.js";
import { classicIdentityHash, createFileReferenceCache, getOrCompute, fixtureHash, type ClassicReferenceCache } from "./reference-cache.js";
import { collectClassicSamples, collectCoreSamples, DEFAULT_DURATION_S, sampleTimesFor } from "./sample.js";
import { denseAnalogSchedule } from "./schedule.js";
import { replayStimuliOnClassic, replayStimuliOnCore, StimuliNotYetSupportedOnCoreError } from "./stimuli.js";
import { collectClassicDigitalTrace, collectCoreDigitalTrace, CoreTraceUnsupportedError, type DigitalTraceEvent } from "./trace.js";
import type { CorpusEntry, CorpusOrigin, EntryResult, KnownDivergenceHit, Mismatch, SampleReadout } from "./types.js";

/** The first ~10 mismatches are reported per the brief; the rest are still counted (FailResult.totalMismatches). */
const MAX_REPORTED_MISMATCHES = 10;

function errorMessage(err: unknown): string {
  return err instanceof Error ? err.message : String(err);
}

/** `CorpusEntry.origin`'s own doc: "this harness's own two sources use fixed literals" for local fixtures; anything else is an external corpus's own producer string. */
export function isLocalCorpusOrigin(origin: CorpusOrigin): boolean {
  return origin === "brownout-fixture" || origin === "synthetic";
}

/**
 * Finding 12: an EXTERNAL corpus's classic readouts carry that corpus's own
 * component/net ids, currents and failure detail — genuinely derived data,
 * not just a hash — so they must never be written inside this checkout
 * (scripts/core-equivalence.ts's own documented guarantee: "Nothing under
 * --corpus is ever written into this repo"). The OS temp dir is outside any
 * git checkout by construction and still gets the same cross-run reuse a
 * local fixture's cache gets.
 */
export const EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR = path.join(os.tmpdir(), "core-equivalence-classic-refs");

/** Lazily constructed: a fresh process should not touch the filesystem cache until an entry actually needs it (a `--corpus`-less test run over only the synthetic fixtures, for instance). */
let localClassicReferenceCache: ClassicReferenceCache<SampleReadout[]> | null = null;
let externalClassicReferenceCache: ClassicReferenceCache<SampleReadout[]> | null = null;

/** The classic-sample cache for LOCAL fixtures (brownout's own + this harness's synthetic ones) — public, MIT-licensed data, so the in-repo, gitignored default cache directory is fine (finding 12 explicitly carves this out; only an EXTERNAL corpus's derived data must never land inside the checkout). */
function getLocalClassicReferenceCache(): ClassicReferenceCache<SampleReadout[]> {
  if (!localClassicReferenceCache) localClassicReferenceCache = createFileReferenceCache<SampleReadout[]>();
  return localClassicReferenceCache;
}

/** The classic-sample cache for an EXTERNAL corpus (finding 12) — see EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR's own doc. */
function getExternalClassicReferenceCache(): ClassicReferenceCache<SampleReadout[]> {
  if (!externalClassicReferenceCache) {
    externalClassicReferenceCache = createFileReferenceCache<SampleReadout[]>(EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR);
  }
  return externalClassicReferenceCache;
}

/**
 * Classic samples, cached by (classic identity, fixture) — see reference-
 * cache.ts's own doc for what "classic identity" hashes (finding 8) and
 * which of the two caches above an entry's `origin` routes it to (finding
 * 12). Falls back to an uncached run when hashing the identity fails for any
 * reason (e.g. a packaged tarball missing a `src/` directory) — the cache is
 * a performance optimisation (gate 2g); losing it must never turn into a
 * harness failure.
 */
export function collectClassicSamplesCached(entry: CorpusEntry, circuit: SimCircuit, times: readonly number[]): SampleReadout[] {
  let classicIdentity: string;
  try {
    classicIdentity = classicIdentityHash({ partLibraryContent: getPartLibrary() });
  } catch {
    return collectClassicSamples(circuit, times);
  }
  const cache = isLocalCorpusOrigin(entry.origin) ? getLocalClassicReferenceCache() : getExternalClassicReferenceCache();
  return getOrCompute(
    cache,
    { classicIdentity, fixtureHash: fixtureHash({ id: entry.id, circuit, times }) },
    () => collectClassicSamples(circuit, times),
  );
}

/**
 * Gate 2d's stimuli path: replays `entry.phases` on classic, then attempts
 * the core side purely to observe it throw `StimuliNotYetSupportedOnCoreError`
 * (owner decision 1's parameter-update event is W1b, not yet landed).
 * Runs BEFORE the plain not-ported/pass/fail flow and takes priority over it
 * whenever `entry.phases` is set — a verify-script entry's real question is
 * "can this be validated under scripted stimuli", which is unconditionally
 * "not yet, core-side" regardless of whether its kinds happen to already be
 * `CORE_IMPLEMENTED_KINDS`. Reporting "core-stimuli-pending" here instead of
 * silently falling through to "not-ported" (today) or a "pass" that never
 * actually exercised the phases (once every kind involved is implemented) is
 * the whole point: neither of those would tell a reader that classic-side
 * replay genuinely ran.
 */
function runStimuliEntry(entry: CorpusEntry, circuit: SimCircuit): EntryResult {
  const phases = entry.phases!;

  let classicWallMs: number;
  try {
    const started = performance.now();
    replayStimuliOnClassic(circuit, phases);
    classicWallMs = performance.now() - started;
  } catch (err) {
    return {
      status: "error",
      kinds: entry.kinds,
      engine: "classic",
      message: errorMessage(err),
      classicWallMs: null,
      coreWallMs: null,
    };
  }

  try {
    replayStimuliOnCore(circuit, phases);
  } catch (err) {
    if (err instanceof StimuliNotYetSupportedOnCoreError) {
      return { status: "core-stimuli-pending", kinds: entry.kinds, classicWallMs, coreWallMs: null, phaseCount: phases.length };
    }
    // Any other throw is a genuine bug in this harness (or a sign
    // replayStimuliOnCore's contract has changed underneath this function),
    // not the expected "not yet supported" signal — surface it as a real error.
    return { status: "error", kinds: entry.kinds, engine: "core", message: errorMessage(err), classicWallMs, coreWallMs: null };
  }
  // replayStimuliOnCore is documented to always throw; reaching here means
  // owner decision 1's mechanism has landed and this function is stale.
  throw new Error(
    "runStimuliEntry: replayStimuliOnCore returned instead of throwing — the classic-vs-core stimuli " +
      "comparison (W1b) needs to be implemented here; this function's \"always throws\" assumption no longer holds.",
  );
}

/**
 * Finding 10: quiet-instant selection cannot certify any candidate as quiet
 * without real event-time evidence, so an analog entry that would otherwise
 * silently get zero evidence (or evidence that does not cover its whole
 * schedule) refuses instead of admitting every candidate by default.
 */
export class QuietInstantEvidenceError extends Error {
  constructor(entryId: string, reason: string) {
    super(
      `resolveSampleTimes: entry "${entryId}" declares an analog guard window, but ${reason}. ` +
        "Quiet-instant selection cannot certify any sample as quiet without real event-time evidence, " +
        "so it refuses the schedule instead of silently admitting every candidate as \"quiet\" by default.",
    );
    this.name = "QuietInstantEvidenceError";
  }
}

/**
 * A representative classic step size (s) for gate 2b's guard window,
 * measured from an ACTUAL classic run over the full requested duration —
 * quiet.ts's own documented contract ("e.g. the run's own finalH"). Finding
 * 10: the dense schedule's fixed sample spacing (durationS / candidates.length,
 * the old value here) is a property of the SAMPLING GRID, not of classic's
 * adaptive step controller, and has no relationship to how big a step
 * classic actually took near any given instant.
 */
function measureClassicStepS(circuit: SimCircuit, durationS: number): number {
  const runner = new HeadlessRunner();
  runner.load(circuit);
  const result = runner.run({ durationS });
  return result.finalH;
}

/** Resolves an entry's analog sample schedule: its declared dense/quiet grid (gate 2b) when `entry.analog` is set, else the W0 fixed 4-point default. `guardEventTimes` are digital-event instants (from a trace, when one exists) the guard window must also avoid; `tracedDurationS` is how far those event times are known to actually cover (`undefined` when no trace was even attempted). Exported for direct unit coverage of finding 10 (run-entry.test.ts). */
export function resolveSampleTimes(
  entry: CorpusEntry,
  circuit: SimCircuit,
  durationS: number,
  guardEventTimes: readonly number[],
  tracedDurationS: number | undefined,
): number[] {
  if (!entry.analog) return sampleTimesFor(durationS);

  if (tracedDurationS !== undefined && durationS > tracedDurationS) {
    throw new QuietInstantEvidenceError(
      entry.id,
      `its analog schedule runs to t=${durationS}s but the digital trace supplying guard-window event times only covers t=${tracedDurationS}s`,
    );
  }
  if (guardEventTimes.length === 0) {
    throw new QuietInstantEvidenceError(
      entry.id,
      "no digital-event evidence is available to guard against (no `digital` spec on this entry, or its trace came back unsupported)",
    );
  }

  const candidates = denseAnalogSchedule(durationS);
  const classicStepS = measureClassicStepS(circuit, durationS);
  const guard = guardWindowS({ maxDelayS: entry.analog.maxDelayS, classicStepS });
  const quiet = selectQuietInstants(candidates, guardEventTimes, guard);
  requireMinQuietInstants(quiet, entry.analog.minQuietInstants);
  return quiet;
}

interface DigitalAlignmentOutcome {
  readonly mismatches: readonly Mismatch[];
  /** Every distinct instant either trace recorded a digital event — feeds the analog guard window (gate 2b) when the entry also declares `analog`. */
  readonly eventTimes: readonly number[];
}

/** The gate 2a digital-alignment check, as extra `Mismatch`es folded into the entry's normal comparison — `undefined` when the entry declares no `digital` spec, `"unsupported"` when it does but the core's trace ABI is not available in this build yet (downgrade, not a failure). */
function runDigitalAlignment(entry: CorpusEntry, circuit: SimCircuit): DigitalAlignmentOutcome | "unsupported" | undefined {
  const spec = entry.digital;
  if (!spec) return undefined;

  const durationS = spec.traceDurationS ?? entry.durationS ?? DEFAULT_DURATION_S;
  let coreTrace: DigitalTraceEvent[];
  try {
    coreTrace = collectCoreDigitalTrace(circuit, durationS);
  } catch (err) {
    if (err instanceof CoreTraceUnsupportedError) return "unsupported";
    throw err;
  }
  const classicTrace = collectClassicDigitalTrace(circuit, durationS);
  const eventTimes = [...new Set([...classicTrace, ...coreTrace].map((e) => e.t))].sort((a, b) => a - b);

  const classicCycles = sampleCycles(classicTrace, spec.clockKey, spec.projection);
  const coreCycles = sampleCycles(coreTrace, spec.clockKey, spec.projection);
  const alignment = alignCycles(classicCycles, coreCycles, spec.anchor, spec.offsetBoundCycles);

  if (!alignment.ok) {
    return {
      eventTimes,
      mismatches: [
        {
          sampleTime: 0,
          category: "digitalState",
          id: "*",
          classicValue: null,
          coreValue: null,
          detail: `digital trace alignment failed: ${alignment.reason}`,
        },
      ],
    };
  }

  const comparison = compareAlignedCycles(classicCycles, coreCycles, alignment);
  const mismatches: Mismatch[] = comparison.mismatches.map((m) => ({
    sampleTime: classicCycles[classicCycles.findIndex((c) => c.cycleIndex === m.classicCycle)]?.edgeTime ?? 0,
    category: "digitalState",
    id: m.key,
    classicValue: m.classicValue,
    coreValue: m.coreValue,
    detail: `cycle ${m.classicCycle} (classic) vs ${m.coreCycle} (core), aligned at offset ${comparison.offsetCycles}: digital state differs`,
  }));

  if (comparison.comparedCycles < spec.minComparedCycles) {
    mismatches.push({
      sampleTime: 0,
      category: "digitalState",
      id: "*",
      classicValue: comparison.comparedCycles,
      coreValue: spec.minComparedCycles,
      detail:
        `only ${comparison.comparedCycles} cycle(s) were compared after alignment, short of the fixture's ` +
        `declared minimum of ${spec.minComparedCycles}`,
    });
  }

  return { eventTimes, mismatches };
}

function toKnownDivergenceHit(match: { mismatch: Mismatch; divergence: { id: string; decisionRef: string; reason: string } }): KnownDivergenceHit {
  return { mismatch: match.mismatch, divergenceId: match.divergence.id, decisionRef: match.divergence.decisionRef, reason: match.divergence.reason };
}

export function runCorpusEntry(entry: CorpusEntry, circuit: SimCircuit): EntryResult {
  // Gate 2d's stimuli path takes priority over everything below: a
  // verify-script entry with exported phases is answering a different
  // question ("can this be validated under scripted stimuli") than the
  // plain not-ported/pass/fail flow, and that answer is unconditionally
  // "not yet, core-side" regardless of whether the circuit's kinds are
  // otherwise CORE_IMPLEMENTED_KINDS.
  if (entry.phases && entry.phases.length > 0) {
    return runStimuliEntry(entry, circuit);
  }

  const missingKinds = missingCoreKinds(entry.kinds);
  if (missingKinds.length > 0) {
    return { status: "not-ported", kinds: entry.kinds, missingKinds, classicWallMs: null, coreWallMs: null };
  }

  // Digital alignment (gate 2a) runs first: it needs its own full-resolution
  // traces, independent of the analog sample schedule below, and — when the
  // entry ALSO declares `analog` — its event instants feed the guard window
  // quiet.ts computes for the analog comparison.
  let digitalMismatches: readonly Mismatch[] = [];
  let digitalEventTimes: readonly number[] = [];
  // How far `digitalEventTimes` above is known to actually cover — undefined
  // when no trace was even attempted (finding 10: resolveSampleTimes needs
  // this to refuse an analog schedule that outruns the traced span).
  let digitalTracedDurationS: number | undefined;
  if (entry.digital) {
    digitalTracedDurationS = entry.digital.traceDurationS ?? entry.durationS ?? DEFAULT_DURATION_S;
    const outcome = runDigitalAlignment(entry, circuit);
    if (outcome && outcome !== "unsupported") {
      digitalMismatches = outcome.mismatches;
      digitalEventTimes = outcome.eventTimes;
    }
    // "unsupported": the core's trace ABI has not landed in this build yet —
    // downgrade, per this worker's Verify step ("run the classic-only paths,
    // and re-run the core paths at the end"). The digital dimension is
    // simply not checked this run; the entry's status still reflects
    // whatever the analog comparison below finds. digitalEventTimes stays
    // empty, which resolveSampleTimes treats as "no evidence" for an entry
    // that also declares `analog` (finding 10) — it does NOT treat an
    // unsupported trace as "confirmed zero events".
  }

  const times = resolveSampleTimes(
    entry,
    circuit,
    entry.durationS ?? DEFAULT_DURATION_S,
    digitalEventTimes,
    digitalTracedDurationS,
  );

  let coreSamples;
  let coreWallMs: number;
  try {
    const started = performance.now();
    coreSamples = collectCoreSamples(circuit, times);
    coreWallMs = performance.now() - started;
  } catch (err) {
    if (err instanceof CoreUnsupportedError) {
      return {
        status: "core-declined",
        kinds: entry.kinds,
        reasons: [...err.reasons],
        classicWallMs: null,
        coreWallMs: null,
      };
    }
    return {
      status: "error",
      kinds: entry.kinds,
      engine: "core",
      message: errorMessage(err),
      classicWallMs: null,
      coreWallMs: null,
    };
  }

  let classicSamples;
  let classicWallMs: number;
  try {
    const started = performance.now();
    classicSamples = collectClassicSamplesCached(entry, circuit, times);
    classicWallMs = performance.now() - started;
  } catch (err) {
    return {
      status: "error",
      kinds: entry.kinds,
      engine: "classic",
      message: errorMessage(err),
      classicWallMs: null,
      coreWallMs,
    };
  }

  const cmp = compareSamples(classicSamples, coreSamples);
  const errorStats = {
    maxAbsErrorVolts: cmp.maxAbsErrorVolts,
    maxRelErrorVolts: cmp.maxRelErrorVolts,
    maxAbsErrorAmps: cmp.maxAbsErrorAmps,
    maxRelErrorAmps: cmp.maxRelErrorAmps,
  };

  const allMismatches = [...cmp.mismatches, ...digitalMismatches];
  const { known, unexplained } = classifyMismatches(circuit, allMismatches);
  const knownDivergences = known.map(toKnownDivergenceHit);

  if (unexplained.length === 0) {
    return { status: "pass", kinds: entry.kinds, classicWallMs, coreWallMs, knownDivergences, ...errorStats };
  }
  return {
    status: "fail",
    kinds: entry.kinds,
    classicWallMs,
    coreWallMs,
    knownDivergences,
    mismatches: unexplained.slice(0, MAX_REPORTED_MISMATCHES),
    totalMismatches: unexplained.length,
    ...errorStats,
  };
}
