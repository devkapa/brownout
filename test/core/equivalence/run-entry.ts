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
 *
 * W1e ADDS three flows, each documented at its own section below: an entry
 * holding an `ne555` is judged as core/README.md NE555 point 13 says
 * (`runNe555Entry` for the plain flow, `prepareNe555Stimuli` under gate 2d's
 * scripted stimuli): the core's own switch times against a model-exact
 * sidecar, the timing network against the sidecar's exponentials, everything
 * else at phase-aligned quiet instants; an entry declaring an LED `sidecar`
 * also checks the core against the junction law's own answer
 * (`runSidecarChecks`); and every entry holding an LED-family kind is run a
 * second time with the solution cache off (`compareCacheOnOff`, LEDs point 7).
 * None of it touches an entry without those kinds or that field.
 */

import os from "node:os";
import path from "node:path";
import { getPartLibrary } from "../../../src/parts/part-library.js";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { CoreEngine } from "../../../src/core/engine.js";
import { CoreUnsupportedError } from "../../../src/core/errors.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { buildCoreNodes, corePinKey } from "../../../src/core/compile/topology.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { alignCycles, compareAlignedCycles, sampleCycles } from "./align.js";
import {
  checkAnalyticCrossing,
  detectNe555Specs,
  evaluateLedStatic,
  extractNe555Sidecar,
  fixedPairCapacitorCurrentAt,
  ledRcVoltages,
  rcLadderVoltagesAt,
  rcTerminalThevenin,
  rcVoltageAt,
  type Ne555Phase,
  type Ne555Sidecar,
  type Ne555SidecarSpec,
} from "./analytic.js";
import { comparePhaseAlignedSamples, compareSamples, compareStimuliSamples } from "./compare.js";
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
  DigitalFixtureSpec,
  EntryResult,
  KnownDivergenceHit,
  Mismatch,
  Ne555FixtureSpec,
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
//
// SECOND RULE, `richardson-closer-than-classic` (manager ruling 2026-09-29,
// plan accuracy gate 2: "agree within tolerance (or come closer to the
// analytic answer, with the reason recorded)"). Rule 1 needs the core to sit
// within the ordinary tolerance of R. The core steps at a few ms and is
// itself only backward Euler, so a core far nearer the truth than classic
// can still sit more than a tolerance from R (mostly in capacitor currents,
// where a tolerance of 0.1% is tighter than a voltage's), and rule 1 fails
// it. A mismatch that fails rule 1 is therefore also explained when BOTH
// hold:
//   a. `|core - R| < |classic - R|`, strictly, and
//   b. R has itself converged: replay classic a third time at h/4, form
//      `R' = richardsonEstimate(c(h/2), c(h/4))` at the same instant, and
//      `|R' - R|` is within the ordinary tolerance at R's magnitude (the
//      same `withinTolerance` bounds, volts for netV and amps for
//      elementI).
// (b) is what stops "closer than classic" from being a free pass: a core
// that beats classic against a reference that is still moving under
// refinement has shown nothing. The h/4 replay is the expensive one, so it
// runs lazily (only for a mismatch that already satisfies (a)) and at most
// once per entry; a replay that throws, or a missing sample at the instant,
// leaves the mismatch unexplained. Each hit records `|core - R|`,
// `|classic - R|` and the estimate, in its reason and in
// `KnownDivergenceHit.richardson`.

/** Exported so a test can assert an explained mismatch's `divergenceId` without hardcoding the string twice. */
export const RICHARDSON_DIVERGENCE_ID = "richardson-classic-self-reference";
/** The second rule's id (this section's header): kept apart from rule 1's so a report can count how much of a pass rests on the weaker "closer than classic" evidence. */
export const RICHARDSON_CLOSER_DIVERGENCE_ID = "richardson-closer-than-classic";
const RICHARDSON_CLOSER_DECISION_REF =
  "Delta (W1e, manager ruling 2026-09-29), plan accuracy gate 2 (\"agree within tolerance, or come closer to the " +
  "analytic answer, with the reason recorded\"): a mismatch rule 1 does not take is explained when the core is " +
  "strictly closer than classic to the Richardson reference R AND R's own convergence estimate (classic at h/4 " +
  "against h/2, |R' - R|) is within the ordinary tolerance at R's magnitude.";
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

/** The signature of `replayStimuliOnClassic`, so a test can stand in a hand-built classic (the detector's own numbers must not come from an engine). */
export type ClassicStimuliReplay = (
  circuit: SimCircuit,
  phases: readonly StimulusPhase[],
  fixedStepS: number,
) => readonly StimulusStepSample[];

/**
 * Splits `mismatches` into ones this Richardson reference explains (see this
 * section's header for the two rules) and everything else, which callers
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
 * guessed at. The third replay (h/4, rule 2's convergence estimate) is taken
 * only when a mismatch needs it, once.
 */
export function explainRichardsonMismatches(
  circuit: SimCircuit,
  phases: readonly StimulusPhase[],
  mismatches: readonly Mismatch[],
  richardsonMethod: "be" | "trap" = "be",
  replay: ClassicStimuliReplay = replayStimuliOnClassic,
): { readonly explained: readonly KnownDivergenceHit[]; readonly remaining: readonly Mismatch[] } {
  const analogMismatches = mismatches.filter((m) => m.category === "netV" || m.category === "elementI");
  if (analogMismatches.length === 0) return { explained: [], remaining: mismatches };

  let fineSamples: readonly StimulusStepSample[];
  try {
    fineSamples = replay(circuit, phases, STIMULUS_FIXED_STEP_S / 2);
  } catch {
    return { explained: [], remaining: mismatches };
  }

  // `undefined`: not replayed yet; `null`: replayed and threw, which is not retried (once per entry).
  let finestSamples: readonly StimulusStepSample[] | null | undefined;
  const finest = (): readonly StimulusStepSample[] | null => {
    if (finestSamples === undefined) {
      try {
        finestSamples = replay(circuit, phases, STIMULUS_FIXED_STEP_S / 4);
      } catch {
        finestSamples = null;
      }
    }
    return finestSamples;
  };

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

    const valueAt = (samples: readonly StimulusStepSample[]): number | undefined => {
      const sample = findFineSampleAt(samples, mismatch.phaseIndex!, mismatch.sampleTime);
      return sample ? (mismatch.category === "netV" ? sample.netV[mismatch.id] : sample.elementI[mismatch.id]) : undefined;
    };
    const fineValue = valueAt(fineSamples);
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
        richardson: { reference: r, coreError: coreAbsErr, classicError: classicAbsErr, tolerance: absTol + relTol * Math.abs(r) },
      });
      continue;
    }

    // Rule 2. The strict comparison comes first because it is free and the
    // h/4 replay is not: a core no closer than classic is unexplained
    // whatever R's estimate is.
    if (!(coreAbsErr < classicAbsErr)) {
      remaining.push(mismatch);
      continue;
    }
    const finestAll = finest();
    const finestValue = finestAll ? valueAt(finestAll) : undefined;
    if (typeof finestValue !== "number") {
      remaining.push(mismatch);
      continue;
    }
    const rNext = richardsonEstimate(fineValue, finestValue, richardsonMethod);
    const estimate = Math.abs(rNext - r);
    if (!withinTolerance(rNext, r, absTol, relTol)) {
      remaining.push(mismatch);
      continue;
    }
    explained.push({
      mismatch,
      divergenceId: RICHARDSON_CLOSER_DIVERGENCE_ID,
      decisionRef: RICHARDSON_CLOSER_DECISION_REF,
      reason:
        `core is closer to the Richardson reference R=${r} than classic is (|core-R|=${coreAbsErr.toExponential(3)} < ` +
        `|classic-R|=${classicAbsErr.toExponential(3)}), though not within tolerance of it; R has converged: ` +
        `|R'-R|=${estimate.toExponential(3)} (R'=${rNext}, classic at h/4 against h/2) is within the ordinary tolerance`,
      richardson: { reference: r, coreError: coreAbsErr, classicError: classicAbsErr, tolerance: absTol + relTol * Math.abs(r), convergenceEstimate: estimate },
    });
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
    // The core follows CLASSIC's own recorded schedule (stimuli.ts's
    // replayStimuliOnCore doc comment: "WHY NOT RE-DERIVE THE SCHEDULE") —
    // classicSamples IS that schedule, structurally compatible as-is.
    coreSamples = replayStimuliOnCore(circuit, phases, classicSamples);
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

  // W1e: an entry holding a 555 takes its timing network out of the classic-versus-core comparison
  // (the core is held to the sidecar instead), drops the timing branch from the supply current, and
  // lets an instant where classic's 555 lags the core's pass for what its outputs reach.
  let ne555: Ne555StimuliPrep | null;
  try {
    ne555 = prepareNe555Stimuli(entry, circuit, phases, classicSamples, coreSamples);
  } catch (err) {
    return { status: "error", kinds: entry.kinds, engine: "core", message: errorMessage(err), classicWallMs, coreWallMs };
  }

  let cmp: ReturnType<typeof compareStimuliSamples>;
  try {
    // `circuit` enables finding 12's stamp-after-commit-lag exemption
    // (compare.ts's own header) — the ORIGINAL (pre-override) circuit is
    // correct here for the same reason known-divergences.ts's registry below
    // uses it: a commit's driven-net topology is fixed by the netlist, never
    // changed by a phase's `set` (which only ever touches component params).
    cmp = compareStimuliSamples(ne555 ? ne555.classic : classicSamples, ne555 ? ne555.core : coreSamples, circuit);
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
  const lagKnown: KnownDivergenceHit[] = [];
  let afterLag: readonly Mismatch[] = cmp.mismatches;
  if (ne555) {
    const still: Mismatch[] = [];
    for (const m of cmp.mismatches) {
      const onSurface =
        m.category === "digitalState"
          ? ne555.lagSurface.components.has(m.id.split("/")[0]!)
          : m.category === "netV"
            ? ne555.lagSurface.nets.has(m.id)
            : m.category === "elementI"
              ? ne555.lagSurface.components.has(m.id)
              : false;
      if (onSurface && ne555.lagTimes.has(m.sampleTime)) {
        lagKnown.push({
          mismatch: m,
          divergenceId: CLASSIC_555_LAG_DIVERGENCE_ID,
          decisionRef: CLASSIC_555_LAG_REF,
          reason: `classic's 555 lags the core's at t=${m.sampleTime} and agrees again at the next sample; "${m.id}" is one of what its outputs reach`,
        });
      } else {
        still.push(m);
      }
    }
    afterLag = still;
  }
  const richardsonSplit = explainRichardsonMismatches(circuit, phases, afterLag);
  const { known, unexplained: unexplainedRaw } = classifyMismatches(circuit, richardsonSplit.remaining);
  const unexplained = [...unexplainedRaw, ...(ne555?.extra ?? [])];
  const knownDivergences = [...cmp.knownDivergences, ...lagKnown, ...richardsonSplit.explained, ...known.map(toKnownDivergenceHit)];

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

export interface DigitalAlignmentOutcome {
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
  return alignDigitalTraces(spec, classicTrace, coreTrace);
}

/** Gate 2a on two already-collected traces (`runDigitalAlignment` above collects them; an NE555 entry collects them once for its period check and reuses them here). */
export function alignDigitalTraces(
  spec: DigitalFixtureSpec,
  classicTrace: readonly DigitalTraceEvent[],
  coreTrace: readonly DigitalTraceEvent[],
): DigitalAlignmentOutcome {
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

// ─── W1e: NE555 entries (core/README.md "Kernel semantics: NE555 (W1e)" point 13) ──
//
// Classic's 555 output lags its latch by one solve and its crossing search
// is step-limited (+1.58% per period on the SAP-1 clock astable), so a plain
// instant-for-instant comparison of a 555 circuit fails for reasons that are
// classic's, not the core's. An entry holding an `ne555` is therefore judged
// as point 13 says, in place of the four-point comparison:
//   2. the core's own switch times against the model-exact sidecar, 1e-6
//      relative (`checkNe555Periods`); classic's are recorded, not gated;
//   4. the timing network's nodes and currents against the sidecar's
//      exponentials at the core's own instants, and everything else at
//      phase-aligned quiet instants: the midpoint of each engine's own k-th
//      HIGH and LOW (`ne555Midpoints`).
// The digital comparison (point 13.1) is the entry's `digital` spec, per cycle
// after alignment. An entry that declares no `ne555` spec gets one derived
// from its own wiring (`detectNe555Specs`); a 555 the sidecar cannot model
// (a foreign element on its timing network, an RST driven by something else)
// is still compared at phase-aligned instants, with its timing nodes held to
// classic there as they are for any other node.

/** NE555 point 13.2: every phase duration and every period agrees with the sidecar to 1e-6 relative. */
export const NE555_PERIOD_REL_TOL = 1e-6;
const NE555_DEFAULT_INTERVALS = 8;
const NE555_DEFAULT_MIN_INTERVAL_S = 1e-5;
/** Classic's own switch times and the sidecar's, reported and never gated (D11). */
export const CLASSIC_555_PERIOD_DRIFT_ID = "classic-555-period-drift";
const CLASSIC_555_PERIOD_DRIFT_REF =
  "core/README.md NE555 point 13.2 (D11): the core's switch times are exact event times; classic's come from " +
  "its step-limited crossing search and are recorded against the same sidecar, not gated.";

export interface LevelChange {
  readonly t: number;
  readonly high: boolean;
}

export interface LevelSegment {
  readonly high: boolean;
  readonly t0: number;
  readonly t1: number;
}

/** `key`'s level changes in a digital trace, consecutive repeats dropped. A `NaN` (the key absent: an unpowered 555) is skipped, so the level before it carries on. */
export function levelChanges(trace: readonly DigitalTraceEvent[], key: string): LevelChange[] {
  const out: LevelChange[] = [];
  for (const e of trace) {
    if (e.key !== key || Number.isNaN(e.value)) continue;
    const high = e.value === 1;
    if (out.length === 0 || out[out.length - 1]!.high !== high) out.push({ t: e.t, high });
  }
  return out;
}

/** The stretches between consecutive changes: every complete HIGH or LOW. The last level, still open at the end of the trace, is not one. */
export function closedSegments(changes: readonly LevelChange[]): LevelSegment[] {
  const out: LevelSegment[] = [];
  for (let i = 0; i + 1 < changes.length; i++) out.push({ high: changes[i]!.high, t0: changes[i]!.t, t1: changes[i + 1]!.t });
  return out;
}

function phaseLabel(index: number, high: boolean, segments: readonly LevelSegment[]): string {
  const ordinal = segments.slice(0, index + 1).filter((s) => s.high === high).length;
  return index === 0 && high ? "first HIGH" : `${high ? "HIGH" : "LOW"} #${ordinal}`;
}

const fmtS = (t: number): string => `${(t * 1e3).toPrecision(9)} ms`;

/**
 * NE555 point 13.2. `changes` is the core's trace of the 555's key: each
 * closed HIGH and LOW (the first HIGH from START_TIME) and each period
 * `r_(k+1) - r_k` between two real rising edges (the power-on cycle, which
 * starts at START_TIME rather than at an edge, is excluded) must agree with
 * the sidecar's `phases` to `NE555_PERIOD_REL_TOL` relative. Fewer than
 * `minPhases` closed phases is itself a mismatch: a run too short to measure
 * proves nothing.
 *
 * Mismatch fields: `classicValue` is the SIDECAR's value and `coreValue` the
 * measured one (types.ts, `Mismatch.category`).
 */
export function checkNe555Periods(params: {
  readonly key: string;
  readonly changes: readonly LevelChange[];
  readonly phases: readonly Ne555Phase[];
  readonly minPhases: number;
}): Mismatch[] {
  const { key, changes, phases, minPhases } = params;
  const segments = closedSegments(changes);
  const modelled = phases.filter((p) => Number.isFinite(p.t1));
  const mismatches: Mismatch[] = [];
  const push = (id: string, expected: number, measured: number, detail: string): void => {
    mismatches.push({ sampleTime: 0, category: "period", id: `${key}: ${id}`, classicValue: expected, coreValue: measured, detail });
  };

  if (segments.length < minPhases) {
    push("phases", minPhases, segments.length, `the core completed only ${segments.length} phase(s) of the sidecar's, short of the fixture's declared minimum of ${minPhases}`);
  }
  if (changes.length > 0 && phases.length > 0 && changes[0]!.high !== phases[0]!.high) {
    push("initial level", phases[0]!.high ? 1 : 0, changes[0]!.high ? 1 : 0, "the core starts at a different level than the sidecar");
    return mismatches;
  }

  const n = Math.min(segments.length, modelled.length);
  for (let i = 0; i < n; i++) {
    const expected = modelled[i]!.t1 - modelled[i]!.t0;
    const measured = segments[i]!.t1 - segments[i]!.t0;
    const rel = Math.abs(measured - expected) / expected;
    if (!(rel <= NE555_PERIOD_REL_TOL)) {
      push(phaseLabel(i, segments[i]!.high, segments), expected, measured, `${fmtS(measured)} against the sidecar's ${fmtS(expected)}: relative error ${rel.toExponential(3)} exceeds ${NE555_PERIOD_REL_TOL.toExponential(0)}`);
    }
  }
  for (let i = 1; i + 1 < n; i++) {
    if (!segments[i]!.high) continue;
    const expected = modelled[i + 1]!.t1 - modelled[i]!.t0;
    const measured = segments[i + 1]!.t1 - segments[i]!.t0;
    const rel = Math.abs(measured - expected) / expected;
    if (!(rel <= NE555_PERIOD_REL_TOL)) {
      push(`period #${segments.slice(0, i + 1).filter((s) => s.high).length - 1}`, expected, measured, `${fmtS(measured)} against the sidecar's ${fmtS(expected)}: relative error ${rel.toExponential(3)} exceeds ${NE555_PERIOD_REL_TOL.toExponential(0)}`);
    }
  }
  return mismatches;
}

/** Classic's own phase durations against the sidecar, as known-divergence records (never gated): the first few complete phases. */
function recordClassicPeriods(key: string, changes: readonly LevelChange[], phases: readonly Ne555Phase[]): KnownDivergenceHit[] {
  const segments = closedSegments(changes);
  const modelled = phases.filter((p) => Number.isFinite(p.t1));
  const hits: KnownDivergenceHit[] = [];
  const n = Math.min(segments.length, modelled.length, 4);
  for (let i = 0; i < n; i++) {
    const expected = modelled[i]!.t1 - modelled[i]!.t0;
    const measured = segments[i]!.t1 - segments[i]!.t0;
    const rel = (measured - expected) / expected;
    if (Math.abs(rel) <= NE555_PERIOD_REL_TOL) continue;
    const label = phaseLabel(i, segments[i]!.high, segments);
    hits.push({
      mismatch: {
        sampleTime: segments[i]!.t0,
        category: "period",
        id: `${key}: classic ${label}`,
        classicValue: measured,
        coreValue: expected,
        detail: `classic ${fmtS(measured)} against the sidecar's ${fmtS(expected)} (${(rel * 100).toFixed(2)}%)`,
      },
      divergenceId: CLASSIC_555_PERIOD_DRIFT_ID,
      decisionRef: CLASSIC_555_PERIOD_DRIFT_REF,
      reason: `classic's ${label} is ${(rel * 100).toFixed(2)}% off the model-exact duration; recorded, not gated`,
    });
  }
  return hits;
}

/**
 * The instants at which one engine is compared phase-aligned: the midpoint
 * of each of its complete HIGH and LOW stretches, in order. A stretch shorter
 * than `minIntervalS` is no phase (classic's load-seed pulse when the
 * comparators resolve 0 at load lasts one solve) and is skipped in both
 * engines' lists, so the k-th of one is the k-th of the other. A monostable's
 * last LOW, open at the end of the run, contributes its midpoint to the run's
 * end.
 */
export function ne555Midpoints(changes: readonly LevelChange[], horizonS: number, minIntervalS: number, monostable: boolean): number[] {
  const mids = closedSegments(changes)
    .filter((s) => s.t1 - s.t0 >= minIntervalS)
    .map((s) => 0.5 * (s.t0 + s.t1));
  const last = changes[changes.length - 1];
  if (monostable && last && !last.high && horizonS - last.t >= 2 * minIntervalS) mids.push(0.5 * (last.t + horizonS));
  return mids;
}

/**
 * The supply's phase-independent current. A source on the timing network's
 * supply net carries the network's own branch current (the charging current
 * through RA), which follows the capacitor's phase and so differs between the
 * engines at their k-th midpoints; this adds the timing resistors' current
 * leaving the source's + net back on, leaving what the 555 and its load draw.
 */
export function ne555SupplyResidual(
  circuit: SimCircuit,
  resistors: readonly { readonly id: string; readonly aNet: string; readonly bNet: string }[],
): (elementI: Record<string, number>) => Record<string, number> {
  const built = buildCoreNodes(circuit);
  const adjustments = circuit.components
    .filter((c) => c.kind === "voltage_source")
    .map((source) => {
      const node = built.pinNode.get(corePinKey(source.id, "pos"));
      const posNet = node === undefined ? null : built.netIdOfNode[node];
      const terms = resistors
        .filter((r) => r.aNet === posNet || r.bNet === posNet)
        .map((r) => ({ id: r.id, sign: r.aNet === posNet ? 1 : -1 }));
      return { id: source.id, terms };
    })
    .filter((a) => a.terms.length > 0);
  return (elementI) => {
    if (adjustments.length === 0) return elementI;
    const out = { ...elementI };
    for (const { id, terms } of adjustments) {
      if (out[id] === undefined) continue;
      out[id] = out[id]! + terms.reduce((sum, t) => sum + t.sign * (elementI[t.id] ?? 0), 0);
    }
    return out;
  };
}

/** The core's readout against the sidecar's prediction at the core's own instants (point 13.4): `classicValue` is the sidecar's value, `coreValue` the core's. */
export function compareNe555Sidecar(sidecar: Ne555Sidecar, coreSamples: readonly Pick<SampleReadout, "simTime" | "netV" | "elementI">[]): Mismatch[] {
  const mismatches: Mismatch[] = [];
  for (const sample of coreSamples) {
    const predicted = sidecar.at(sample.simTime);
    const check = (category: "netV" | "elementI", id: string, want: number, got: number | undefined, absTol: number, relTol: number): void => {
      if (got === undefined || !withinTolerance(want, got, absTol, relTol)) {
        mismatches.push({
          sampleTime: sample.simTime,
          category: "sidecar",
          id: `${category}.${id}`,
          classicValue: want,
          coreValue: got,
          detail:
            got === undefined
              ? "the core does not publish it"
              : `|${want} - ${got}| = ${Math.abs(want - got).toExponential(3)} exceeds tolerance ${(absTol + relTol * Math.max(Math.abs(want), Math.abs(got))).toExponential(3)} of the sidecar's exponential`,
        });
      }
    };
    for (const [id, want] of Object.entries(predicted.netV)) check("netV", id, want, sample.netV[id], VOLT_ABS_TOL, VOLT_REL_TOL);
    for (const [id, want] of Object.entries(predicted.elementI)) check("elementI", id, want, sample.elementI[id], AMP_ABS_TOL, AMP_REL_TOL);
  }
  return mismatches;
}

/** True when the entry's circuit holds a 555 (from its derived `kinds`, never trusted from a manifest). */
export function hasNe555(entry: CorpusEntry): boolean {
  return entry.kinds.includes("ne555");
}

/** The sidecars an entry's 555s get: its declared spec, else one derived from the wiring of each 555 (`detectNe555Specs`). A 555 the sidecar cannot model is listed in `unmodelled` rather than thrown for; a DECLARED spec that cannot be built is a fixture bug and throws. */
export function ne555Sidecars(
  entry: CorpusEntry,
  circuit: SimCircuit,
  horizonS: number,
  stimuli?: readonly StimulusPhase[],
): { readonly sidecars: readonly Ne555Sidecar[]; readonly unmodelled: readonly string[] } {
  const specs: readonly (Ne555SidecarSpec | Ne555FixtureSpec)[] = entry.ne555 ? [entry.ne555] : detectNe555Specs(circuit);
  const sidecars: Ne555Sidecar[] = [];
  const unmodelled: string[] = circuit.components
    .filter((c) => c.kind === "ne555" && !specs.some((sp) => sp.component === c.id))
    .map((c) => c.id);
  for (const spec of specs) {
    try {
      sidecars.push(extractNe555Sidecar(circuit, spec, horizonS, { stimuli }));
    } catch (err) {
      if (entry.ne555) throw err;
      unmodelled.push(spec.component);
    }
  }
  return { sidecars, unmodelled };
}

// ─── W1e: NE555 entries under scripted stimuli (gate 2d) ──────────────────────
//
// A verify-script entry replays classic on a fixed 1 ms grid and the core on
// classic's own instants (stimuli.ts). The 555 rules of point 13 carry over:
// the timing network is held to the sidecar's exponentials at every instant
// (a scripted button press on TRIG is a trigger the sidecar follows, from
// `v0 = V∞d` after a full discharge), the supply current is compared without
// the timing network's own branch current, and an instant where the engines
// disagree on the 555's level (classic's output lags its latch by one solve,
// decision 7) is not a quiet instant for anything the 555's OUT and DIS reach.

/** `phase.set` folded into a copy of the circuit (phase 0 becomes part of the load, as stimuli.ts does it). */
function withPhaseSet(circuit: SimCircuit, phase: StimulusPhase | undefined): SimCircuit {
  if (!phase?.set || phase.set.length === 0) return circuit;
  const next: SimCircuit = { ...circuit, components: circuit.components.map((c) => ({ ...c })) };
  for (const override of phase.set) {
    const component = next.components.find((c) => c.id === override.id);
    if (component) component.params = { ...component.params, ...override.params };
  }
  return next;
}

/** The core's digital trace over a scripted stimulus: `collectCoreDigitalTrace`'s baseline, then every change while each phase's `set` is applied through `setParams` (phase 0 through the load) and the run advances to the phase's end. */
export function collectCoreDigitalTraceWithPhases(circuit: SimCircuit, phases: readonly StimulusPhase[]): DigitalTraceEvent[] {
  const engine = new CoreEngine();
  try {
    engine.load(withPhaseSet(circuit, phases[0]));
    const baseline = Object.entries(engine.digitalState).map(([key, value]) => ({ t: engine.simTime, key, value }));
    engine.enableTrace(true);
    let end = engine.simTime;
    phases.forEach((phase, index) => {
      if (index > 0 && phase.set && phase.set.length > 0) {
        engine.setParams(phase.set.map((o) => ({ componentId: o.id, params: o.params })));
      }
      end += phase.durationS;
      engine.advance(end);
    });
    const events = [...engine.readTrace()];
    engine.clearTrace();
    return [...baseline, ...events];
  } finally {
    engine.dispose();
  }
}

/** Kinds a 555's OUT and DIS reach without a source or a driver in between (a two-terminal passive, or an LED-family part through its own junction). */
const PASSIVE_KINDS: ReadonlySet<string> = new Set(["resistor", "capacitor", "led", "push_button", "switch"]);

/**
 * The nets and components one instant of classic's one-solve output lag can
 * touch: the nets of each named 555's OUT and DIS, everything reachable from
 * them through passive two-terminals (stopping at fixed nodes, where a source
 * or the datum sits), and every voltage source (it carries whatever the
 * outputs draw). Narrow on purpose: an IC, a switch driver or anything past a
 * fixed node is never inside it.
 */
export function ne555LagSurface(
  circuit: SimCircuit,
  chipIds: readonly string[],
): { readonly nets: ReadonlySet<string>; readonly components: ReadonlySet<string> } {
  const built = buildCoreNodes(circuit);
  const fixedNets = new Set<string>();
  for (const src of circuit.components.filter((c) => c.kind === "voltage_source")) {
    for (const pin of ["pos", "neg"]) {
      const node = built.pinNode.get(corePinKey(src.id, pin));
      const id = node === undefined ? undefined : built.netIdOfNode[node];
      if (id) fixedNets.add(id);
    }
  }
  fixedNets.add("gnd");
  const netOfPin = (componentId: string, pinId: string): string | null => {
    const node = built.pinNode.get(corePinKey(componentId, pinId));
    return node === undefined ? null : (built.netIdOfNode[node] ?? null);
  };
  const nets = new Set<string>();
  const components = new Set<string>(chipIds);
  const queue: string[] = [];
  for (const chip of chipIds) {
    for (const pin of ["3", "7"]) {
      const net = netOfPin(chip, pin);
      if (net && !nets.has(net)) {
        nets.add(net);
        queue.push(net);
      }
    }
  }
  while (queue.length > 0) {
    const net = queue.shift()!;
    for (const [componentId] of built.nets.find((n) => n.id === net)?.pins ?? []) {
      const comp = circuit.components.find((c) => c.id === componentId);
      if (!comp || !PASSIVE_KINDS.has(comp.kind)) continue;
      components.add(componentId);
      for (const pin of comp.pins) {
        const other = netOfPin(componentId, pin.id);
        if (other && !fixedNets.has(other) && !nets.has(other)) {
          nets.add(other);
          queue.push(other);
        }
      }
    }
  }
  for (const src of circuit.components.filter((c) => c.kind === "voltage_source")) components.add(src.id);
  return { nets, components };
}

export const CLASSIC_555_LAG_DIVERGENCE_ID = "classic-555-one-solve-lag";
const CLASSIC_555_LAG_REF =
  "core/README.md NE555 point 12 (decision 7): classic's OUT and DIS stamps run one solve behind its latch, so at an " +
  "instant where the engines disagree on the 555's level nothing its outputs reach is a quiet instant; the core has " +
  "no such lag. Both engines agree on the level again at the very next sample.";

interface Ne555StimuliPrep {
  readonly classic: readonly StimulusStepSample[];
  readonly core: readonly StimulusStepSample[];
  readonly sidecars: readonly Ne555Sidecar[];
  /** Sidecar and period mismatches (core against the model): never explained by a divergence or by Richardson. */
  readonly extra: readonly Mismatch[];
  /** Sample times classic's 555 lags the core's, with the surface each can touch. */
  readonly lagTimes: ReadonlySet<number>;
  readonly lagSurface: { readonly nets: ReadonlySet<string>; readonly components: ReadonlySet<string> };
}

/** Prepares an NE555 entry's stimuli comparison: the timing network is taken out of the classic-versus-core comparison (the core is held to the sidecar instead) and the supply current loses the timing branch. Returns `null` for an entry with no 555. */
function prepareNe555Stimuli(
  entry: CorpusEntry,
  circuit: SimCircuit,
  phases: readonly StimulusPhase[],
  classic: readonly StimulusStepSample[],
  core: readonly StimulusStepSample[],
): Ne555StimuliPrep | null {
  if (!hasNe555(entry)) return null;
  const horizonS = phases.reduce((sum, p) => sum + p.durationS, 0);
  const { sidecars } = ne555Sidecars(entry, circuit, horizonS, phases);

  const skipNetV = new Set<string>();
  const skipElementI = new Set<string>();
  const resistors: { id: string; aNet: string; bNet: string }[] = [];
  const extra: Mismatch[] = [];
  for (const sidecar of sidecars) {
    sidecar.netIds.forEach((id) => skipNetV.add(id));
    sidecar.elementIds.forEach((id) => skipElementI.add(id));
    resistors.push(...sidecar.resistors);
    extra.push(...compareNe555Sidecar(sidecar, core));
    // Point 13.2: the core's own switch times, from its trace over the same script.
    const key = sidecar.spec.component;
    const minPhases = entry.ne555 && entry.ne555.component === key ? entry.ne555.minPhases : 0;
    extra.push(...checkNe555Periods({ key, changes: levelChanges(collectCoreDigitalTraceWithPhases(circuit, phases), key), phases: sidecar.phases, minPhases }));
  }
  const adjust = ne555SupplyResidual(circuit, resistors);
  const prepare = (sample: StimulusStepSample): StimulusStepSample => {
    const netV = Object.fromEntries(Object.entries(sample.netV).filter(([id]) => !skipNetV.has(id)));
    const elementI = Object.fromEntries(Object.entries(adjust({ ...sample.elementI })).filter(([id]) => !skipElementI.has(id)));
    return { ...sample, netV, elementI };
  };

  const chips = circuit.components.filter((c) => c.kind === "ne555").map((c) => c.id);
  return { classic: classic.map(prepare), core: core.map(prepare), sidecars, extra, lagTimes: ne555LagTimes(chips, classic, core), lagSurface: ne555LagSurface(circuit, chips) };
}

/**
 * The sample times (classic's clock) where classic's 555 lags the core's: the
 * engines disagree on a 555's level there and agree again at the very next
 * sample. A disagreement that outlasts one sample is a standing difference, not
 * a lag, and is never excused.
 */
export function ne555LagTimes(
  chipIds: readonly string[],
  classic: readonly Pick<StimulusStepSample, "simTime" | "digitalState">[],
  core: readonly Pick<StimulusStepSample, "simTime" | "digitalState">[],
): Set<number> {
  const lagTimes = new Set<number>();
  const disagrees = (i: number): boolean => chipIds.some((id) => !Object.is(classic[i]!.digitalState[id], core[i]!.digitalState[id]));
  for (let i = 0; i + 1 < classic.length; i++) {
    if (disagrees(i) && !disagrees(i + 1)) lagTimes.add(classic[i]!.simTime);
  }
  return lagTimes;
}

function runNe555Entry(entry: CorpusEntry, circuit: SimCircuit): EntryResult {
  const durationS = entry.digital?.traceDurationS ?? entry.durationS ?? DEFAULT_DURATION_S;
  const kinds = entry.kinds;
  const fail = (engine: "classic" | "core", err: unknown, classicWallMs: number | null, coreWallMs: number | null): EntryResult => ({
    status: "error",
    kinds,
    engine,
    message: errorMessage(err),
    classicWallMs,
    coreWallMs,
  });

  let coreTrace: DigitalTraceEvent[];
  let coreWallMs: number;
  try {
    const started = performance.now();
    coreTrace = collectCoreDigitalTrace(circuit, durationS);
    coreWallMs = performance.now() - started;
  } catch (err) {
    if (err instanceof CoreUnsupportedError) {
      return { status: "core-declined", kinds, reasons: [...err.reasons], classicWallMs: null, coreWallMs: null };
    }
    return fail("core", err, null, null);
  }
  let classicTrace: DigitalTraceEvent[];
  let classicWallMs: number;
  try {
    const started = performance.now();
    classicTrace = collectClassicDigitalTrace(circuit, durationS);
    classicWallMs = performance.now() - started;
  } catch (err) {
    return fail("classic", err, null, coreWallMs);
  }

  const mismatches: Mismatch[] = [];
  const knownDivergences: KnownDivergenceHit[] = [];

  let sidecars: readonly Ne555Sidecar[];
  try {
    sidecars = ne555Sidecars(entry, circuit, durationS).sidecars;
  } catch (err) {
    return fail("core", err, classicWallMs, coreWallMs);
  }
  const firstKey = sidecars[0]?.spec.component ?? circuit.components.find((c) => c.kind === "ne555")!.id;
  const monostable = (sidecars[0]?.spec.mode ?? "astable") === "monostable";

  // Point 13.1: `<id>` is in every declared 555 fixture's projection.
  if (entry.ne555) {
    if (!entry.digital) {
      mismatches.push({
        sampleTime: 0,
        category: "digitalState",
        id: "*",
        classicValue: null,
        coreValue: null,
        detail: "an NE555 fixture must declare a `digital` spec whose projection names the 555's own key (NE555 point 13.1)",
      });
    } else if (entry.digital.projection && !entry.digital.projection.includes(entry.ne555.component)) {
      mismatches.push({
        sampleTime: 0,
        category: "digitalState",
        id: "*",
        classicValue: null,
        coreValue: null,
        detail: `the digital projection must include the 555's key "${entry.ne555.component}" (NE555 point 13.1)`,
      });
    }
  }
  if (entry.digital) mismatches.push(...alignDigitalTraces(entry.digital, classicTrace, coreTrace).mismatches);

  // Point 13.2: the core's switch times against the sidecar; classic's recorded.
  for (const sidecar of sidecars) {
    const key = sidecar.spec.component;
    const minPhases = entry.ne555 && entry.ne555.component === key ? entry.ne555.minPhases : 0;
    mismatches.push(...checkNe555Periods({ key, changes: levelChanges(coreTrace, key), phases: sidecar.phases, minPhases }));
    knownDivergences.push(...recordClassicPeriods(key, levelChanges(classicTrace, key), sidecar.phases));
  }

  // Point 13.4: phase-aligned instants, each engine's own.
  const minIntervalS = entry.ne555?.minIntervalS ?? NE555_DEFAULT_MIN_INTERVAL_S;
  const wanted = entry.ne555?.intervals ?? NE555_DEFAULT_INTERVALS;
  const classicMids = ne555Midpoints(levelChanges(classicTrace, firstKey), durationS, minIntervalS, monostable);
  const coreMids = ne555Midpoints(levelChanges(coreTrace, firstKey), durationS, minIntervalS, monostable);
  const count = Math.min(wanted, classicMids.length, coreMids.length);
  let classicTimes = classicMids.slice(0, count);
  let coreTimes = coreMids.slice(0, count);
  if (count === 0) {
    if (entry.ne555) {
      mismatches.push({
        sampleTime: 0,
        category: "sidecar",
        id: "phase-aligned instants",
        classicValue: wanted,
        coreValue: 0,
        detail: "no complete HIGH or LOW interval on both engines: nothing to compare phase-aligned",
      });
    } else {
      // No phase closed inside the entry's own duration on at least one engine, so no
      // switching has separated them yet: the entry's ordinary sample grid (the same
      // instants on both sides) is the phase-aligned comparison, and the timing network
      // is still held to the sidecar's exponentials there.
      try {
        const eventTimes = [...new Set([...classicTrace, ...coreTrace].map((e) => e.t))].sort((a, b) => a - b);
        const plain = resolveSampleTimes(entry, circuit, durationS, eventTimes, durationS);
        classicTimes = plain;
        coreTimes = plain;
      } catch (err) {
        return fail("core", err, classicWallMs, coreWallMs);
      }
    }
  }

  let errorStats = { maxAbsErrorVolts: 0, maxRelErrorVolts: 0, maxAbsErrorAmps: 0, maxRelErrorAmps: 0 };
  if (classicTimes.length > 0) {
    let coreSamples: SampleReadout[];
    let classicSamples: SampleReadout[];
    try {
      const started = performance.now();
      coreSamples = collectCoreSamples(circuit, coreTimes);
      coreWallMs += performance.now() - started;
    } catch (err) {
      return fail("core", err, classicWallMs, coreWallMs);
    }
    try {
      const started = performance.now();
      classicSamples = collectClassicSamplesCached(entry, circuit, classicTimes);
      classicWallMs += performance.now() - started;
    } catch (err) {
      return fail("classic", err, classicWallMs, coreWallMs);
    }

    if (hasLedFamily(entry)) mismatches.push(...compareCacheOnOff(circuit, coreTimes, coreSamples));

    const skipNetV = new Set<string>();
    const skipElementI = new Set<string>();
    const resistors: { id: string; aNet: string; bNet: string }[] = [];
    for (const sidecar of sidecars) {
      sidecar.netIds.forEach((id) => skipNetV.add(id));
      sidecar.elementIds.forEach((id) => skipElementI.add(id));
      resistors.push(...sidecar.resistors);
      mismatches.push(...compareNe555Sidecar(sidecar, coreSamples));
    }
    const cmp = comparePhaseAlignedSamples(classicSamples, coreSamples, {
      skipNetV,
      skipElementI,
      adjustElementI: ne555SupplyResidual(circuit, resistors),
    });
    mismatches.push(...cmp.mismatches);
    errorStats = {
      maxAbsErrorVolts: cmp.maxAbsErrorVolts,
      maxRelErrorVolts: cmp.maxRelErrorVolts,
      maxAbsErrorAmps: cmp.maxAbsErrorAmps,
      maxRelErrorAmps: cmp.maxRelErrorAmps,
    };
  }

  // The LED sidecar checks, when the entry also declares them.
  const extra = runSidecarChecks(entry, circuit);
  mismatches.push(...extra.mismatches);
  coreWallMs += extra.coreWallMs;

  const { known, unexplained } = classifyMismatches(circuit, mismatches);
  const hits = [...knownDivergences, ...known.map(toKnownDivergenceHit)];
  if (unexplained.length === 0) {
    return { status: "pass", kinds, classicWallMs, coreWallMs, knownDivergences: hits, ...errorStats };
  }
  return {
    status: "fail",
    kinds,
    classicWallMs,
    coreWallMs,
    knownDivergences: hits,
    mismatches: unexplained.slice(0, MAX_REPORTED_MISMATCHES),
    totalMismatches: unexplained.length,
    ...errorStats,
  };
}

// ─── W1e: the solution cache (core/README.md LEDs point 7) ──────────────────────
//
// A nonlinear static partition keeps the solution cache, and with
// NO_SOLUTION_CACHE it is solved by the Newton of point 6 from its last
// published voltages instead, so the two runs agree within 1 uV (the polish
// residual, at most 0.39 uV measured), not bit for bit; a linear partition
// stays bit-identical. Every entry holding an LED-family kind is run both ways
// at the same instants and held to that bound: a cache hit that published
// stale bits, or a miss that missed its epoch, shows up here on a circuit
// whose drivers repeat modes (an IC or a 555 driving an LED).

/** The bound a cached and an uncached nonlinear partition must agree within (LEDs point 7), volts. */
export const CACHE_AGREEMENT_VOLTS = 1e-6;
const LED_FAMILY_KINDS: ReadonlySet<string> = new Set(["led", "rgb_led", "bicolor_led", "seg7_cc", "seg7_ca"]);

/** True when the entry's circuit holds an LED-family kind (LEDs point 7's nonlinear static partitions). */
export function hasLedFamily(entry: CorpusEntry): boolean {
  return entry.kinds.some((kind) => LED_FAMILY_KINDS.has(kind));
}

/** The core with the solution cache on (`cached`, already collected by the caller) against the same run with it off, at the same instants: netV within 1 uV, elementI within the ordinary amp tolerance. `classicValue` is the cached run's, `coreValue` the uncached one's (types.ts, `Mismatch.category`). */
export function compareCacheOnOff(circuit: SimCircuit, times: readonly number[], cached: readonly SampleReadout[]): Mismatch[] {
  const engine = new CoreEngine();
  const mismatches: Mismatch[] = [];
  try {
    engine.load(circuit, CORE_IMPLEMENTED_KINDS, { noSolutionCache: true });
    times.forEach((t, i) => {
      if (t > 0) engine.advance(t);
      const uncached = { netV: engine.getNetV(), elementI: engine.getElementI() };
      const on = cached[i]!;
      for (const [id, v] of Object.entries(on.netV)) {
        const w = uncached.netV[id];
        if (w === undefined || !(Math.abs(v - w) <= CACHE_AGREEMENT_VOLTS)) {
          mismatches.push({ sampleTime: t, category: "sidecar", id: `cache.netV.${id}`, classicValue: v, coreValue: w, detail: `cache on ${v} against cache off ${w}: |diff| exceeds ${CACHE_AGREEMENT_VOLTS.toExponential(0)} V (LEDs point 7)` });
        }
      }
      for (const [id, v] of Object.entries(on.elementI)) {
        const w = uncached.elementI[id];
        if (w === undefined || !withinTolerance(v, w, AMP_ABS_TOL, AMP_REL_TOL)) {
          mismatches.push({ sampleTime: t, category: "sidecar", id: `cache.elementI.${id}`, classicValue: v, coreValue: w, detail: `cache on ${v} against cache off ${w}: exceeds the amp tolerance (LEDs point 7)` });
        }
      }
    });
  } finally {
    engine.dispose();
  }
  return mismatches;
}

// ─── W1e: LED sidecars (core/README.md "Kernel semantics: LEDs and displays (W1e)") ──
//
// A static LED fixture compares the plain classic-versus-core samples AND the
// core against the model's own answer (analytic.ts's `evaluateLedStatic`),
// plus the channel currents of the multi-channel kinds, which the plain
// samples do not carry: core against the sidecar and against classic's
// `getElementChannelI()`, key set included. An RC-fed LED (a stepped,
// nonlinear partition) compares the core under TRAPEZOIDAL integration
// against the ODE at mid-transient instants (orchestrator ruling: backward
// Euler's own mid-transient error is inherent, so BE is judged only where the
// plain sample instants are settled).

const channelKey = (id: string, j: number): string => `${id}[${j}]`;

function sidecarMismatch(sampleTime: number, id: string, want: number | string, got: number | string | undefined, detail: string): Mismatch {
  return { sampleTime, category: "sidecar", id, classicValue: want, coreValue: got, detail };
}

export function runSidecarChecks(entry: CorpusEntry, circuit: SimCircuit): { readonly mismatches: Mismatch[]; readonly coreWallMs: number } {
  const model = entry.sidecar;
  if (!model) return { mismatches: [], coreWallMs: 0 };
  const started = performance.now();
  const mismatches: Mismatch[] = [];
  const durationS = entry.durationS ?? DEFAULT_DURATION_S;

  if (model.kind === "led-static") {
    const want = evaluateLedStatic(circuit, model);
    const engine = new CoreEngine();
    let coreChannels: Record<string, number[]>;
    try {
      engine.load(circuit);
      engine.advance(durationS);
      const netV = engine.getNetV();
      const elementI = engine.getElementI();
      coreChannels = { ...engine.getElementChannelI() };
      const t = engine.simTime;
      for (const [id, v] of Object.entries(want.netV)) {
        if (!withinTolerance(v, netV[id] ?? Number.NaN, VOLT_ABS_TOL, VOLT_REL_TOL)) {
          mismatches.push(sidecarMismatch(t, `netV.${id}`, v, netV[id], `|${v} - ${netV[id]}| exceeds the volt tolerance of the model's own answer`));
        }
      }
      for (const [id, i] of Object.entries(want.elementI)) {
        if (!withinTolerance(i, elementI[id] ?? Number.NaN, AMP_ABS_TOL, AMP_REL_TOL)) {
          mismatches.push(sidecarMismatch(t, `elementI.${id}`, i, elementI[id], `|${i} - ${elementI[id]}| exceeds the amp tolerance of the model's own answer`));
        }
      }
      for (const [id, channels] of Object.entries(want.channels)) {
        const got = coreChannels[id];
        if (channels === null) {
          if (got !== undefined) mismatches.push(sidecarMismatch(t, `channels.${id}`, "absent", `[${got.join(", ")}]`, "a burnt part's channel entry must be absent"));
          continue;
        }
        if (channels.length === 0) continue; // a plain `led` publishes no channels
        if (!got || got.length !== channels.length) {
          mismatches.push(sidecarMismatch(t, `channels.${id}`, channels.length, got?.length, "the core's channel array has the wrong length"));
          continue;
        }
        channels.forEach((amps, j) => {
          if (!withinTolerance(amps, got[j]!, AMP_ABS_TOL, AMP_REL_TOL)) {
            mismatches.push(sidecarMismatch(t, `channels.${channelKey(id, j)}`, amps, got[j], `|${amps} - ${got[j]}| exceeds the amp tolerance of the model's own answer`));
          }
        });
      }
    } catch (err) {
      return {
        mismatches: [sidecarMismatch(0, "core run", "ok", errorMessage(err), "the core could not run the fixture for the sidecar check")],
        coreWallMs: performance.now() - started,
      };
    } finally {
      engine.dispose();
    }

    // Classic's channel currents, key set included (`getElementChannelI`, spec §10.10).
    try {
      const runner = new HeadlessRunner();
      runner.load(circuit);
      if (durationS > 0) runner.run({ durationS });
      const classicChannels = runner.engine.getElementChannelI() as Record<string, number[]>;
      const keys = new Set([...Object.keys(classicChannels), ...Object.keys(coreChannels)]);
      for (const id of keys) {
        const a = classicChannels[id];
        const b = coreChannels[id];
        if (!a || !b || a.length !== b.length) {
          mismatches.push({ sampleTime: durationS, category: "elementI", id: `channels.${id}`, classicValue: a?.length, coreValue: b?.length, detail: "the channel arrays differ in presence or length" });
          continue;
        }
        a.forEach((x, j) => {
          if (!withinTolerance(x, b[j]!, AMP_ABS_TOL, AMP_REL_TOL)) {
            mismatches.push({ sampleTime: durationS, category: "elementI", id: `channels.${channelKey(id, j)}`, classicValue: x, coreValue: b[j], detail: `|${x} - ${b[j]}| exceeds the amp tolerance` });
          }
        });
      }
    } catch (err) {
      mismatches.push(sidecarMismatch(0, "classic run", "ok", errorMessage(err), "classic could not run the fixture for the channel check"));
    }
  } else {
    const built = buildCoreNodes(circuit);
    const device = circuit.components.find((c) => c.id === model.device);
    const anodePin = device?.pins[0]?.id;
    const node = anodePin === undefined ? undefined : built.pinNode.get(corePinKey(model.device, anodePin));
    const netId = node === undefined ? undefined : built.netIdOfNode[node];
    if (!netId) {
      return { mismatches: [sidecarMismatch(0, "led-rc node", "resolved", undefined, `${model.device}'s anode is not on a named net`)], coreWallMs: 0 };
    }
    const want = ledRcVoltages(circuit, model, model.instantsS);
    const engine = new CoreEngine();
    try {
      engine.load(circuit, CORE_IMPLEMENTED_KINDS, { integrationMethod: "trap" });
      model.instantsS.forEach((t, i) => {
        engine.advance(t);
        const got = engine.getNetV()[netId];
        if (!withinTolerance(want[i]!, got ?? Number.NaN, VOLT_ABS_TOL, VOLT_REL_TOL)) {
          mismatches.push(sidecarMismatch(t, `netV.${netId}`, want[i]!, got, `trapezoidal core |${want[i]} - ${got}| exceeds the volt tolerance of the ODE`));
        }
      });
    } catch (err) {
      mismatches.push(sidecarMismatch(0, "core run", "ok", errorMessage(err), "the core (trapezoidal) could not run the fixture for the ODE check"));
    } finally {
      engine.dispose();
    }
  }
  return { mismatches, coreWallMs: performance.now() - started };
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

  // W1e: an entry holding an NE555 is judged as NE555 point 13 says (see the block above).
  if (hasNe555(entry)) return runNe555Entry(entry, circuit);

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

  // W1e (LEDs point 7): an entry holding an LED-family kind also runs the core with the solution cache off.
  const cacheMismatches = hasLedFamily(entry) ? compareCacheOnOff(circuit, times, coreSamples) : [];

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

  const sidecarChecks = runSidecarChecks(entry, circuit);
  const allMismatches = [...cmp.mismatches, ...digitalMismatches, ...sidecarChecks.mismatches, ...cacheMismatches];
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
