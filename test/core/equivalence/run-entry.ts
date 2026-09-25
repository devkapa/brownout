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
import { buildCoreNodes, corePinKey } from "../../../src/core/compile/topology.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { alignCycles, compareAlignedCycles, sampleCycles } from "./align.js";
import { checkAnalyticCrossing, fixedPairCapacitorCurrentAt, rcLadderVoltagesAt, rcTerminalThevenin, rcVoltageAt } from "./analytic.js";
import { compareSamples, compareStimuliSamples } from "./compare.js";
import { classifyMismatches } from "./known-divergences.js";
import { missingCoreKinds } from "./kinds-of.js";
import { guardWindowS, requireMinQuietInstants, selectQuietInstants } from "./quiet.js";
import { classicIdentityHash, createFileReferenceCache, getOrCompute, fixtureHash, type ClassicReferenceCache } from "./reference-cache.js";
import { collectClassicSamples, collectCoreSamples, DEFAULT_DURATION_S, sampleTimesFor } from "./sample.js";
import { denseAnalogSchedule } from "./schedule.js";
import {
  replayStimuliOnClassic,
  replayStimuliOnCore,
  STIMULUS_FIXED_STEP_S,
  StimuliNotYetSupportedOnCoreError,
  type StimulusPhase,
  type StimulusStepSample,
} from "./stimuli.js";
import { AMP_ABS_TOL, AMP_REL_TOL, VOLT_ABS_TOL, VOLT_REL_TOL, withinTolerance } from "./tolerance.js";
import { collectClassicDigitalTrace, collectCoreDigitalTrace, CoreTraceUnsupportedError, type DigitalTraceEvent } from "./trace.js";
import type {
  AnalyticFixtureModel,
  AnalyticNodeRef,
  CorpusEntry,
  CorpusOrigin,
  EntryResult,
  KnownDivergenceHit,
  Mismatch,
  SampleReadout,
} from "./types.js";

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

// ─── Gate 2d / finding 11: Richardson-extrapolated forgiveness for the stimuli path ──
//
// `runStimuliEntry` has no fixture-level analytic model — `entry.analytic`
// (types.ts) is a plain-flow-only field, and a verify-script phase list has
// no closed-form reference of its own. But decision 3's underlying claim
// still applies here: classic's fixed-step (`STIMULUS_FIXED_STEP_S`)
// backward-Euler replay lags the true trajectory by a known, bounded amount,
// and a core that is genuinely CLOSER to the truth than that lagging
// classic sample must not fail an entry just because it disagrees with
// classic's own discretisation error (section 11 of the W1b review; a probe
// against fixture 13's own RC-switch circuit showed the core's exact
// closed-form answer failing against classic's one-step BE value by nearly
// double the tolerance).
//
// ORCHESTRATOR DECISION (locked): where no closed-form model exists, a
// Richardson extrapolation of CLASSIC AGAINST ITSELF stands in for one.
// Replay classic a SECOND time on the same phases at half the fixed step
// (`STIMULUS_FIXED_STEP_S / 2`), read that fine replay's own sample at each
// coarse sample's instant, and combine the two backward-Euler (order 1)
// estimates into `R = 2*c(h/2) - c(h)` — the usual Richardson cancellation
// of a first-order method's leading error term. (For trapezoidal, order 2,
// the equivalent combination is `R = (4*c(h/2) - c(h)) / 3`; nothing in this
// file drives `replayStimuliOnClassic` at anything but backward Euler today
// — see stimuli.ts's `replayStimuliOnClassic`, which never passes an
// `integrationMethod` to its `HeadlessRunner` — so `richardsonMethod` below
// defaults to "be" and only matters once/if that changes.)
//
// A mismatch is explained — never widened — only when BOTH:
//   1. the core is within the ORDINARY volt/current tolerance of R (the same
//      `withinTolerance` compare.ts itself uses, never a looser one), AND
//   2. the core is at least as close to R as classic's own coarse sample is
//      (`|core - R| <= |classic - R|`) — unlike decision 3's plain-flow rule
//      (`checkAnalyticCrossing`, which only requires the core to be within
//      tolerance of the analytic reference and treats `classicValue` as
//      informational), the ORCHESTRATOR DECISION for this numerical
//      stand-in requires the core to actually be the closer of the two,
//      since there is no independently-derived ground truth backing R the
//      way a fixture's own equations back an analytic model.
// Anything else — including a fine reference replay that itself throws, in
// which case R cannot be trusted at all — stays a failure. Digital-slot
// (and simTime) mismatches are never touched: this is an analog-only
// mechanism, exactly like `explainAnalyticMismatches` below.

/** Exported so a test can assert an explained mismatch's `divergenceId` without hardcoding the string twice. */
export const RICHARDSON_DIVERGENCE_ID = "richardson-classic-self-reference";
const RICHARDSON_DECISION_REF =
  "Finding 11 (W1b review), orchestrator decision: gate 2d's stimuli path has no per-fixture analytic model, " +
  "so a Richardson-extrapolated classic-vs-classic reference (R = 2*c(h/2) - c(h) for backward Euler) stands " +
  "in for one, forgiving a mismatch only when the core is within tolerance of R AND at least as close to R as " +
  "classic's own coarse sample is.";

function richardsonEstimate(coarse: number, fine: number, method: "be" | "trap"): number {
  return method === "trap" ? (4 * fine - coarse) / 3 : 2 * fine - coarse;
}

/** The fine (h/2) replay's own sample for `mismatch`'s exact `(phaseIndex, sampleTime)`, or `undefined` if none matches within a tight time tolerance — should always be found (see this section's own header: every coarse instant is also a fine-replay instant), but a caller must not manufacture an R from the wrong instant if a future change to either replay's schedule ever breaks that. */
function findFineSampleAt(
  fineSamples: readonly StimulusStepSample[],
  phaseIndex: number,
  simTime: number,
): StimulusStepSample | undefined {
  const tol = 1e-9 * Math.max(1, Math.abs(simTime));
  return fineSamples.find((s) => s.phaseIndex === phaseIndex && Math.abs(s.simTime - simTime) <= tol);
}

/**
 * Splits `mismatches` into ones this Richardson reference explains (see this
 * section's header for the exact rule) and everything else, which callers
 * must still judge exactly as before. `digitalState`/`simTime` mismatches,
 * and any `netV`/`elementI` mismatch whose values are not both plain
 * numbers or whose `phaseIndex` is missing (never true of a real
 * `compareStimuliSamples` mismatch — guarded defensively anyway), pass
 * straight through to `remaining` untouched.
 *
 * Replays classic a second time at half the fixed step, over the SAME
 * `phases` and the SAME (pre-override) `circuit` `runStimuliEntry` already
 * replayed at the coarse step — an independent `HeadlessRunner`, so this
 * never disturbs the samples already collected. If that fine reference
 * replay itself throws (non-convergence, most likely), R cannot be trusted
 * for ANY mismatch, so everything is returned unexplained rather than
 * guessed at.
 */
export function explainRichardsonMismatches(
  circuit: SimCircuit,
  phases: readonly StimulusPhase[],
  mismatches: readonly Mismatch[],
  richardsonMethod: "be" | "trap" = "be",
): { readonly explained: readonly KnownDivergenceHit[]; readonly remaining: readonly Mismatch[] } {
  const analogMismatches = mismatches.filter((m) => m.category === "netV" || m.category === "elementI");
  if (analogMismatches.length === 0) return { explained: [], remaining: mismatches };

  let fineSamples: readonly StimulusStepSample[];
  try {
    fineSamples = replayStimuliOnClassic(circuit, phases, STIMULUS_FIXED_STEP_S / 2);
  } catch {
    return { explained: [], remaining: mismatches };
  }

  const explained: KnownDivergenceHit[] = [];
  const remaining: Mismatch[] = [];

  for (const mismatch of mismatches) {
    if (mismatch.category !== "netV" && mismatch.category !== "elementI") {
      remaining.push(mismatch);
      continue;
    }
    if (typeof mismatch.classicValue !== "number" || typeof mismatch.coreValue !== "number" || mismatch.phaseIndex === undefined) {
      remaining.push(mismatch);
      continue;
    }

    const fineSample = findFineSampleAt(fineSamples, mismatch.phaseIndex, mismatch.sampleTime);
    const fineValue = fineSample ? (mismatch.category === "netV" ? fineSample.netV[mismatch.id] : fineSample.elementI[mismatch.id]) : undefined;
    if (typeof fineValue !== "number") {
      remaining.push(mismatch);
      continue;
    }

    const r = richardsonEstimate(mismatch.classicValue, fineValue, richardsonMethod);
    const [absTol, relTol] = mismatch.category === "netV" ? [VOLT_ABS_TOL, VOLT_REL_TOL] : [AMP_ABS_TOL, AMP_REL_TOL];
    const coreAbsErr = Math.abs(mismatch.coreValue - r);
    const classicAbsErr = Math.abs(mismatch.classicValue - r);

    if (withinTolerance(mismatch.coreValue, r, absTol, relTol) && coreAbsErr <= classicAbsErr) {
      explained.push({
        mismatch,
        divergenceId: RICHARDSON_DIVERGENCE_ID,
        decisionRef: RICHARDSON_DECISION_REF,
        reason:
          `core is within tolerance of the Richardson-extrapolated classic reference R=${r} (|core-R|=` +
          `${coreAbsErr.toExponential(3)} <= |classic-R|=${classicAbsErr.toExponential(3)})`,
      });
    } else {
      remaining.push(mismatch);
    }
  }

  return { explained, remaining };
}

/**
 * Gate 2d's stimuli path (W1b): replays `entry.phases` on classic, then on
 * the core, and — once both sides actually produced samples — compares them
 * the same way the plain flow compares its own two sample sequences
 * (compare.ts's shared tolerances/comparators, known-divergences.ts's
 * registry, and — finding 11 — the Richardson-extrapolated forgiveness
 * above). Runs BEFORE the plain not-ported/pass/fail flow and takes
 * priority over it whenever `entry.phases` is set — a verify-script entry's
 * real question is "can this be validated under scripted stimuli", which is
 * answered here directly now that owner decision 1's parameter-update event
 * (`bo_engine_set_params`) has landed: a circuit whose kinds are ALL
 * `CORE_IMPLEMENTED_KINDS` (the switch family, W1b) gets a real classic-vs-
 * core comparison, not just a bookkeeping gesture toward one.
 *
 * Results:
 *   - "core-stimuli-pending" when the core cannot even LOAD the circuit
 *     (`StimuliNotYetSupportedOnCoreError` — some kind isn't
 *     `CORE_IMPLEMENTED_KINDS` yet, or the core otherwise declines the
 *     topology). Distinct from "not-ported": that status never even
 *     attempts classic, whereas this one always has a real classic-side
 *     replay behind it (`classicWallMs` is never null here).
 *   - "error" when either side's replay throws for any OTHER reason —
 *     classic did not complete cleanly, the core did not converge, or
 *     (critically) a phase named a param that exists on the core's record
 *     but is not live (`CoreEngine.setParams`'s own check): that must
 *     surface loudly, never silently fold into "not yet supported" or get
 *     dropped.
 *   - "pass"/"fail" once both replays succeed: `compareStimuliSamples`
 *     (compare.ts) diffs the two `StimulusStepSample` sequences — netV,
 *     elementI, digitalState, and the samples' own simTime — tagging each
 *     resulting mismatch with the phase it came from (`Mismatch.phaseIndex`,
 *     types.ts) so a report can point straight at which phase produced it.
 *     Finding 11's Richardson forgiveness is tried first (analog mismatches
 *     only); `known-divergences.ts`'s registry then classifies whatever it
 *     leaves unexplained, exactly as the plain flow does, against the
 *     entry's ORIGINAL (pre-override) circuit — a registration's cone is a
 *     topological fact, and a phase's `set` only ever changes a component's
 *     params, never the netlist a cone is computed from.
 */
function runStimuliEntry(entry: CorpusEntry, circuit: SimCircuit): EntryResult {
  const phases = entry.phases!;

  let classicSamples: StimulusStepSample[];
  let classicWallMs: number;
  try {
    const started = performance.now();
    classicSamples = replayStimuliOnClassic(circuit, phases);
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

  let coreSamples: StimulusStepSample[];
  let coreWallMs: number;
  try {
    const started = performance.now();
    coreSamples = replayStimuliOnCore(circuit, phases);
    coreWallMs = performance.now() - started;
  } catch (err) {
    if (err instanceof StimuliNotYetSupportedOnCoreError) {
      return { status: "core-stimuli-pending", kinds: entry.kinds, classicWallMs, coreWallMs: null, phaseCount: phases.length };
    }
    // Any other throw — non-convergence, or CoreEngine.setParams rejecting a
    // phase's override (an unknown component, or a param that exists but is
    // not live) — is a genuine failure to surface, never a silent skip and
    // never folded into "not yet supported" (that status is reserved for
    // StimuliNotYetSupportedOnCoreError alone, above).
    return { status: "error", kinds: entry.kinds, engine: "core", message: errorMessage(err), classicWallMs, coreWallMs: null };
  }

  let cmp: ReturnType<typeof compareStimuliSamples>;
  try {
    // `circuit` enables finding 12's stamp-after-commit-lag exemption
    // (compare.ts's own header) — the ORIGINAL (pre-override) circuit is
    // correct here for the same reason known-divergences.ts's registry below
    // uses it: a commit's driven-net topology is fixed by the netlist, never
    // changed by a phase's `set` (which only ever touches component params).
    cmp = compareStimuliSamples(classicSamples, coreSamples, circuit);
  } catch (err) {
    // Finding 10: compareStimuliSamples only ever throws on a sample-count
    // mismatch — the two replays used different phase/step schedules, a
    // harness bug rather than a circuit fact. That must become THIS entry's
    // "error" status, the same as every other failure path above, rather
    // than escape uncaught and abort runEquivalenceHarness (and with it,
    // every other entry's result) for the whole run. Defense in depth: the
    // step-schedule fix in stimuli.ts's replayStimuliOnCore is what actually
    // prevents this from firing on a real corpus entry.
    return { status: "error", kinds: entry.kinds, engine: "core", message: errorMessage(err), classicWallMs, coreWallMs };
  }
  const errorStats = {
    maxAbsErrorVolts: cmp.maxAbsErrorVolts,
    maxRelErrorVolts: cmp.maxRelErrorVolts,
    maxAbsErrorAmps: cmp.maxAbsErrorAmps,
    maxRelErrorAmps: cmp.maxRelErrorAmps,
  };

  // Finding 11 first: forgive an analog mismatch that is explained by a
  // Richardson-extrapolated classic-vs-classic reference (this section's own
  // header, above) before ever asking known-divergences.ts's registry about
  // it — the two are independent explanations for the SAME mismatch list, so
  // both are tried and merged, mirroring the plain flow's
  // explainAnalyticMismatches-then-classifyMismatches order.
  const richardsonSplit = explainRichardsonMismatches(circuit, phases, cmp.mismatches);
  const { known, unexplained } = classifyMismatches(circuit, richardsonSplit.remaining);
  const knownDivergences = [...cmp.knownDivergences, ...richardsonSplit.explained, ...known.map(toKnownDivergenceHit)];

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

// ─── Gate 2e / decision 3: analytic-closer judging for the plain comparison ──
//
// A fixture that declares `CorpusEntry.analytic` (types.ts) names one node
// (or ladder pair) it has a closed-form answer for. Decision 3: classic's own
// backward-Euler discretisation can lag the true curve by more than the flat
// classic-vs-core tolerance even when the core is EXACT — that is classic
// being "wrong" by a known, bounded amount, not the core failing. Rather than
// widen the tolerance everywhere (which would just as easily hide a real core
// bug), `explainAnalyticMismatches` re-judges ONLY the mismatches that name
// the declared node/current, against the analytic answer instead of against
// classic, and only when the core itself is within the SAME tolerance
// compareSamples already uses (tolerance.ts) — never a looser one invented
// for this path. A mismatch outside the declared node/elements, or one where
// the core itself is off, is untouched and still fails the entry exactly as
// before classic became "unfair" to compare against.

/** `AnalyticNodeRef` resolved to this circuit's net id (`buildCoreNodes`, the same resolution `compileCircuit` itself uses) — `null` when the pin does not resolve (a malformed model, never expected of a fixture this harness owns). */
function analyticNetId(built: ReturnType<typeof buildCoreNodes>, ref: AnalyticNodeRef): string | null {
  const node = built.pinNode.get(corePinKey(ref.component, ref.pin));
  if (node === undefined) return null;
  return built.netIdOfNode[node] ?? null;
}

interface AnalyticTargets {
  /** net id -> analytic voltage at time t. */
  readonly netV: ReadonlyMap<string, (t: number) => number>;
  /** component id (elementI key) -> analytic current at time t. */
  readonly elementI: ReadonlyMap<string, (t: number) => number>;
}

/** Resolves one entry's declared `AnalyticFixtureModel` into ready-to-evaluate functions, keyed exactly the way `Mismatch.id` names a `netV`/`elementI` target — computed once per entry, reused across every sample time a mismatch might name. */
function buildAnalyticTargets(circuit: SimCircuit, model: AnalyticFixtureModel): AnalyticTargets {
  const built = buildCoreNodes(circuit);
  const netV = new Map<string, (t: number) => number>();
  const elementI = new Map<string, (t: number) => number>();

  if (model.kind === "rc") {
    const { tau, vAsymptote, v0AtUncharged } = rcTerminalThevenin({
      sourceVolts: model.sourceVolts,
      seriesOhms: model.seriesOhms,
      capacitanceF: model.capacitanceF,
      esrOhms: model.esrOhms,
      leakageOhms: model.leakageOhms,
    });
    const voltageAt = (t: number) => rcVoltageAt(v0AtUncharged, vAsymptote, tau, t);
    const netId = analyticNetId(built, model.node);
    if (netId) netV.set(netId, voltageAt);
    for (const componentId of model.currentComponents ?? []) {
      elementI.set(componentId, (t) => (model.sourceVolts - voltageAt(t)) / model.seriesOhms);
    }
    for (const componentId of model.negatedCurrentComponents ?? []) {
      elementI.set(componentId, (t) => -(model.sourceVolts - voltageAt(t)) / model.seriesOhms);
    }
  } else if (model.kind === "rc-ladder") {
    const ladderParams = { sourceVolts: model.sourceVolts, r1Ohms: model.r1Ohms, c1F: model.c1F, r2Ohms: model.r2Ohms, c2F: model.c2F };
    const v1At = (t: number) => rcLadderVoltagesAt(ladderParams, t).v1;
    const v2At = (t: number) => rcLadderVoltagesAt(ladderParams, t).v2;
    const netId1 = analyticNetId(built, model.node1);
    const netId2 = analyticNetId(built, model.node2);
    if (netId1) netV.set(netId1, v1At);
    if (netId2) netV.set(netId2, v2At);
    if (model.r1Component) elementI.set(model.r1Component, (t) => (model.sourceVolts - v1At(t)) / model.r1Ohms);
    if (model.r2Component) elementI.set(model.r2Component, (t) => (v1At(t) - v2At(t)) / model.r2Ohms);
  } else {
    // "fixed-pair-capacitor" (rule 2a): no free node at all, so only
    // elementI targets exist — see AnalyticFixedPairCapacitorModel's own doc
    // comment (types.ts) for the `sign`/`constantAmps` derivation.
    const iCapAt = (t: number) => fixedPairCapacitorCurrentAt(model.dV, model.capacitanceF, model.esrOhms, model.leakageOhms, t);
    elementI.set(model.capacitorComponent, iCapAt);
    for (const src of model.fixingSources) {
      elementI.set(src.component, (t) => src.sign * iCapAt(t) + src.constantAmps);
    }
  }

  return { netV, elementI };
}

/** `Decision 3 (W1 plan)`, recorded on every mismatch this path explains — kept as one exported constant so a report and a test can both name it exactly, rather than re-typing the string. */
export const ANALYTIC_CLOSER_DIVERGENCE_ID_PREFIX = "analytic";
const ANALYTIC_DECISION_REF =
  "Decision 3 (W1 plan): the core's closed-form/stepped solve may be closer to a fixture's declared analytic " +
  "answer than classic's own backward-Euler discretisation is — judged here against the SAME tolerance " +
  "(tolerance.ts) compareSamples itself uses, never a looser one.";

/**
 * Splits `mismatches` into ones `entry.analytic` explains (the core is within
 * tolerance of the analytic answer at that mismatch's own `sampleTime` — gate
 * 2e's rule, `checkAnalyticCrossing`) and everything else, which callers must
 * still judge exactly as before. Returns `mismatches` itself unchanged (in
 * `remaining`) when the entry declares no analytic model, or when a mismatch
 * is not numeric (`classicValue`/`coreValue` are not both `number` — this
 * never checks a `digitalState`/`failures`/`warnings`/`simTime` mismatch)
 * or does not name the declared node/current.
 *
 * Exported for equivalence.test.ts's detector self-test (compareSamples's
 * own header: "the only way to test a detector without editing src to break
 * it") — proving a core value perturbed away from the analytic answer still
 * ends up in `remaining`, never silently explained away.
 */
export function explainAnalyticMismatches(
  entry: CorpusEntry,
  circuit: SimCircuit,
  mismatches: readonly Mismatch[],
): { readonly explained: readonly KnownDivergenceHit[]; readonly remaining: readonly Mismatch[] } {
  if (!entry.analytic) return { explained: [], remaining: mismatches };

  const targets = buildAnalyticTargets(circuit, entry.analytic);
  const explained: KnownDivergenceHit[] = [];
  const remaining: Mismatch[] = [];

  for (const mismatch of mismatches) {
    const analyticFn =
      mismatch.category === "netV"
        ? targets.netV.get(mismatch.id)
        : mismatch.category === "elementI"
          ? targets.elementI.get(mismatch.id)
          : undefined;

    if (!analyticFn || typeof mismatch.classicValue !== "number" || typeof mismatch.coreValue !== "number") {
      remaining.push(mismatch);
      continue;
    }

    const analyticValue = analyticFn(mismatch.sampleTime);
    const [absTol, relTol] = mismatch.category === "netV" ? [VOLT_ABS_TOL, VOLT_REL_TOL] : [AMP_ABS_TOL, AMP_REL_TOL];
    const result = checkAnalyticCrossing({
      coreValue: mismatch.coreValue,
      analyticValue,
      classicValue: mismatch.classicValue,
      absTol,
      relTol,
    });

    if (result.status === "pass") {
      explained.push({
        mismatch,
        divergenceId: `${ANALYTIC_CLOSER_DIVERGENCE_ID_PREFIX}:${entry.analytic.kind}`,
        decisionRef: ANALYTIC_DECISION_REF,
        reason: result.reason,
      });
    } else {
      remaining.push(mismatch);
    }
  }

  return { explained, remaining };
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
  // Gate 2e / decision 3 first: re-judge a mismatch on the entry's declared
  // analytic node/current against the analytic answer before ever asking
  // known-divergences.ts's registry about it — the two are independent
  // explanations for the SAME mismatch list, so both are tried and merged.
  const analyticSplit = explainAnalyticMismatches(entry, circuit, allMismatches);
  const { known, unexplained } = classifyMismatches(circuit, analyticSplit.remaining);
  const knownDivergences = [...analyticSplit.explained, ...known.map(toKnownDivergenceHit)];

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
