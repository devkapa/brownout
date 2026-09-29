/**
 * compareSamples() is the harness's INTERNAL SEAM: it takes two plain arrays
 * of SampleReadout (no engine, no CoreEngine handle, no circuit) and reports
 * where they disagree. run-entry.ts is the only production caller, feeding
 * it two readouts of the SAME circuit from the two engines — but because the
 * function itself only sees data, equivalence.test.ts's detector self-tests
 * can call it directly with readouts that were never produced by a matched
 * pair of engine runs (a genuinely different circuit, or a hand-edited
 * readout) to prove the comparison logic actually fires before trusting a
 * clean sweep on real circuits. That is deliberate, not a loophole: it is
 * the only way to test a detector without editing src to break it.
 */

import { buildCoreNodes, corePinKey, type CoreNodeBuild } from "../../../src/core/compile/topology.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { withinTolerance, VOLT_ABS_TOL, VOLT_REL_TOL, AMP_ABS_TOL, AMP_REL_TOL } from "./tolerance.js";
import { digitalOutputPins } from "./known-divergences.js";
import type { CompareResult, KnownDivergenceHit, Mismatch, SampleReadout } from "./types.js";
import type { StimulusStepSample } from "./stimuli.js";

interface ErrorAccumulator {
  maxAbs: number;
  maxRel: number;
}

function compareNumericRecord(
  category: "netV" | "elementI",
  sampleTime: number,
  classic: Record<string, number>,
  core: Record<string, number>,
  absTol: number,
  relTol: number,
  mismatches: Mismatch[],
  errAcc: ErrorAccumulator,
  phaseIndex?: number,
): void {
  const keys = new Set([...Object.keys(classic), ...Object.keys(core)]);
  for (const key of keys) {
    const inClassic = Object.prototype.hasOwnProperty.call(classic, key);
    const inCore = Object.prototype.hasOwnProperty.call(core, key);
    if (!inClassic || !inCore) {
      mismatches.push({
        sampleTime,
        category,
        id: key,
        classicValue: inClassic ? classic[key] : undefined,
        coreValue: inCore ? core[key] : undefined,
        detail: `present only on the ${inClassic ? "classic" : "core"} side`,
        phaseIndex,
      });
      continue;
    }

    const a = classic[key]!;
    const b = core[key]!;
    const absErr = Math.abs(a - b);
    const denom = Math.max(Math.abs(a), Math.abs(b));
    const relErr = denom > 0 ? absErr / denom : 0;
    // Recorded for every compared pair, not just violations — the table
    // wants "how close agreement actually is" (~1e-12 expected in W0), not
    // just a pass/fail bit.
    errAcc.maxAbs = Math.max(errAcc.maxAbs, absErr);
    errAcc.maxRel = Math.max(errAcc.maxRel, relErr);

    if (!withinTolerance(a, b, absTol, relTol)) {
      mismatches.push({
        sampleTime,
        category,
        id: key,
        classicValue: a,
        coreValue: b,
        detail: `|${a} - ${b}| = ${absErr.toExponential(3)} exceeds tolerance ${(absTol + relTol * denom).toExponential(3)}`,
        phaseIndex,
      });
    }
  }
}

function compareDigitalState(
  sampleTime: number,
  classic: Record<string, number>,
  core: Record<string, number>,
  mismatches: Mismatch[],
  phaseIndex?: number,
): void {
  // This is a RAW per-sample comparator: it assumes `classic`/`core` were
  // already sampled at the same instant (compareSamples's own simTime check
  // is the belt-and-suspenders half of that assumption) and says nothing
  // about clock-cycle skew. For a real digital kind, classic's load() seed
  // can produce a spurious extra clock edge at t=0 (decision 8) that the
  // core's exact-crossing settle does not, so a naive instant-for-instant
  // diff would flag that skew as a mismatch. run-entry.ts's runCorpusEntry
  // handles that SEPARATELY and BEFORE this function ever sees a digital
  // key: for an entry that declares CorpusEntry.digital (types.ts), it
  // builds full-resolution traces (trace.ts), aligns them per clock cycle
  // (align.ts), and folds any resulting mismatch into the SAME list this
  // function's caller already assembles — known-divergences.ts's registry is
  // then applied to the combined list, so an owner-approved divergence
  // (e.g. the SAP-1 display's startup phantom edge) is reported, not failed,
  // regardless of which of the two paths produced it. This function itself
  // stays a plain per-sample compare, correct as written for: (a) W0/W1a,
  // where no kind publishes digitalState at all (both sides are always
  // `{}`); and (b) any future entry that does NOT declare `digital` — e.g. a
  // digital signal sampled well after it has settled, where cycle skew does
  // not matter.
  const keys = new Set([...Object.keys(classic), ...Object.keys(core)]);
  for (const key of keys) {
    const a = classic[key];
    const b = core[key];
    if (!Object.is(a, b)) {
      mismatches.push({
        sampleTime,
        category: "digitalState",
        id: key,
        classicValue: a,
        coreValue: b,
        detail: a === undefined ? "present only on the core side" : b === undefined ? "present only on the classic side" : "digital state differs",
        phaseIndex,
      });
    }
  }
}

/**
 * Presence only — a SimFailure's own field values are not compared (the
 * harness's job is topology/value equivalence, not failure-message wording).
 *
 * W1c EXEMPTION: failure/warning DETECTION (slow/failures.rs, the
 * output-sag target lists in digital/*) is W1g, still un-landed at the time
 * the 11 W1c combinational ICs join the corpus — the core reports `{}` for
 * EVERY circuit's failures right now, unconditionally, regardless of kind.
 * A circuit that genuinely trips a classic failure (bus contention between
 * two totem-pole drivers is a real, expected output_sag trigger — G11's own
 * fixture) would otherwise fail here for a reason that has nothing to do
 * with the actual physics under test (the resistive contention midpoint,
 * which DOES match — this exemption only ever fires on the FAILURE-PRESENCE
 * dimension, never on netV/elementI/digitalState). So: when the core's own
 * side is COMPLETELY EMPTY (it detected nothing at all, the only shape W1g's
 * absence can produce), a key present only on the classic side is not
 * reported as a mismatch. This is narrow by construction — the moment the
 * core reports EVEN ONE failure for a sample (W1g has landed for at least
 * one kind), this exemption stops applying for that sample entirely and a
 * genuine set-mismatch is caught exactly as before; it can never mask the
 * core reporting the WRONG failure set, only "no failure machinery exists
 * yet". Remove this block once W1g lands.
 */
function compareFailureKeys(
  sampleTime: number,
  classic: Record<string, unknown>,
  core: Record<string, unknown>,
  mismatches: Mismatch[],
): void {
  const coreImplementsNoFailuresYet = Object.keys(core).length === 0;
  const keys = new Set([...Object.keys(classic), ...Object.keys(core)]);
  for (const key of keys) {
    const inClassic = key in classic;
    const inCore = key in core;
    if (inClassic !== inCore) {
      if (coreImplementsNoFailuresYet && !inCore) continue;
      mismatches.push({
        sampleTime,
        category: "failures",
        id: key,
        classicValue: inClassic ? classic[key] : undefined,
        coreValue: inCore ? core[key] : undefined,
        detail: "failure presence differs",
      });
    }
  }
}

/**
 * Direct check that the two sides were actually sampled at the same
 * instant, independent of whatever collectClassicSamples/collectCoreSamples
 * did to get there. `1e-12 * max(1, |sampleTime|)` is a tight, near-exact
 * tolerance — generous enough for float accumulation across successive
 * classic `run()` calls, tight enough that a sample taken at a genuinely
 * different instant (e.g. a classic run that quietly stopped short) still
 * fires this. This is the belt to collectClassicSamples's suspenders (its
 * own header): even a divergence that collector fails to catch up front
 * still surfaces here as a labelled mismatch instead of a silent pass.
 */
function compareSimTime(
  sampleTime: number,
  classicSimTime: number,
  coreSimTime: number,
  mismatches: Mismatch[],
  phaseIndex?: number,
): void {
  const tol = 1e-12 * Math.max(1, Math.abs(sampleTime));
  const absErr = Math.abs(classicSimTime - coreSimTime);
  if (absErr > tol) {
    mismatches.push({
      sampleTime,
      category: "simTime",
      id: "simTime",
      classicValue: classicSimTime,
      coreValue: coreSimTime,
      detail:
        `|${classicSimTime} - ${coreSimTime}| = ${absErr.toExponential(3)} exceeds tolerance ${tol.toExponential(3)} — ` +
        "the two sides were sampled at different instants.",
      phaseIndex,
    });
  }
}

/** Codes only, as a sorted multiset — a SimWarning's `since`/`value` are readout-specific, not part of "did the same conditions latch". */
function compareWarningCodes(
  sampleTime: number,
  classic: Record<string, { code: string }>,
  core: Record<string, { code: string }>,
  mismatches: Mismatch[],
): void {
  const classicCodes = Object.values(classic).map((w) => w.code).sort();
  const coreCodes = Object.values(core).map((w) => w.code).sort();
  const same =
    classicCodes.length === coreCodes.length && classicCodes.every((code, i) => code === coreCodes[i]);
  if (!same) {
    mismatches.push({
      sampleTime,
      category: "warnings",
      id: "*",
      classicValue: classicCodes,
      coreValue: coreCodes,
      detail: "warning code sets differ",
    });
  }
}

/**
 * Compares two equal-length, time-aligned SampleReadout sequences. Throws
 * (a harness bug, not a circuit fact) if the sequences have different
 * lengths — that means the two sides were sampled on different schedules,
 * which no caller in this package should ever produce.
 */
export function compareSamples(classic: readonly SampleReadout[], core: readonly SampleReadout[]): CompareResult {
  if (classic.length !== core.length) {
    throw new Error(
      `compareSamples: sample count mismatch (classic ${classic.length}, core ${core.length}) — ` +
        "the two runs used different sample schedules; this is a harness bug, not a circuit result.",
    );
  }

  const mismatches: Mismatch[] = [];
  const voltErr: ErrorAccumulator = { maxAbs: 0, maxRel: 0 };
  const ampErr: ErrorAccumulator = { maxAbs: 0, maxRel: 0 };

  for (let i = 0; i < classic.length; i++) {
    const c = classic[i]!;
    const r = core[i]!;
    // classic's own simTime labels the sample — after successive run() calls
    // its accumulated float error (if any) is the more informative value to
    // report a mismatch against, and it is always close enough to r.simTime
    // (both target the same t_i) that using one or the other never changes
    // which sample a reported mismatch points at.
    const sampleTime = c.simTime;
    compareSimTime(sampleTime, c.simTime, r.simTime, mismatches);
    compareNumericRecord("netV", sampleTime, c.netV, r.netV, VOLT_ABS_TOL, VOLT_REL_TOL, mismatches, voltErr);
    compareNumericRecord("elementI", sampleTime, c.elementI, r.elementI, AMP_ABS_TOL, AMP_REL_TOL, mismatches, ampErr);
    compareDigitalState(sampleTime, c.digitalState, r.digitalState, mismatches);
    compareFailureKeys(sampleTime, c.failures, r.failures, mismatches);
    compareWarningCodes(sampleTime, c.warnings, r.warnings, mismatches);
  }

  return {
    mismatches,
    maxAbsErrorVolts: voltErr.maxAbs,
    maxRelErrorVolts: voltErr.maxRel,
    maxAbsErrorAmps: ampErr.maxAbs,
    maxRelErrorAmps: ampErr.maxRel,
  };
}

/** What `comparePhaseAlignedSamples` leaves out of the classic-versus-core comparison, and how it prepares the rest. */
export interface PhaseAlignedOptions {
  /** Net ids judged against the sidecar instead (NE555 point 13.4: the timing network's nodes). */
  readonly skipNetV: ReadonlySet<string>;
  /** Component ids judged against the sidecar instead (the timing network's resistors and capacitors). */
  readonly skipElementI: ReadonlySet<string>;
  /**
   * Applied to each side's `elementI` before it is compared: the supply
   * current carries the timing network's own branch current (the charging
   * current through RA), which follows the capacitor's phase and so differs
   * between the engines at their k-th midpoints; run-entry.ts's
   * `ne555SupplyResidual` removes it, leaving the phase-independent rest.
   */
  readonly adjustElementI?: (elementI: Record<string, number>) => Record<string, number>;
}

/**
 * NE555 point 13.4: compares `classic[k]` with `core[k]` where each was
 * sampled at ITS OWN instant (the midpoint of its own k-th HIGH or LOW), so
 * the two `simTime`s differ by design and are never compared. Everything else
 * is `compareSamples`'s own comparison, same tolerances: netV, elementI (bar
 * the sidecar's), digitalState, failures, warnings. Each mismatch's detail
 * names both instants.
 */
export function comparePhaseAlignedSamples(
  classic: readonly SampleReadout[],
  core: readonly SampleReadout[],
  options: PhaseAlignedOptions,
): CompareResult {
  if (classic.length !== core.length) {
    throw new Error(
      `comparePhaseAlignedSamples: sample count mismatch (classic ${classic.length}, core ${core.length}) — ` +
        "the caller must pair the engines' k-th intervals; this is a harness bug, not a circuit result.",
    );
  }
  const without = <T>(record: Record<string, T>, skip: ReadonlySet<string>): Record<string, T> =>
    Object.fromEntries(Object.entries(record).filter(([key]) => !skip.has(key)));

  const mismatches: Mismatch[] = [];
  const voltErr: ErrorAccumulator = { maxAbs: 0, maxRel: 0 };
  const ampErr: ErrorAccumulator = { maxAbs: 0, maxRel: 0 };

  for (let i = 0; i < classic.length; i++) {
    const c = classic[i]!;
    const r = core[i]!;
    const local: Mismatch[] = [];
    const adjust = options.adjustElementI ?? ((record: Record<string, number>) => record);
    compareNumericRecord("netV", c.simTime, without(c.netV, options.skipNetV), without(r.netV, options.skipNetV), VOLT_ABS_TOL, VOLT_REL_TOL, local, voltErr);
    compareNumericRecord(
      "elementI",
      c.simTime,
      without(adjust(c.elementI), options.skipElementI),
      without(adjust(r.elementI), options.skipElementI),
      AMP_ABS_TOL,
      AMP_REL_TOL,
      local,
      ampErr,
    );
    compareDigitalState(c.simTime, c.digitalState, r.digitalState, local);
    compareFailureKeys(c.simTime, c.failures, r.failures, local);
    compareWarningCodes(c.simTime, c.warnings, r.warnings, local);
    for (const m of local) {
      mismatches.push({ ...m, detail: `${m.detail} [interval ${i}: classic sampled at t=${c.simTime}, core at t=${r.simTime}]` });
    }
  }

  return {
    mismatches,
    maxAbsErrorVolts: voltErr.maxAbs,
    maxRelErrorVolts: voltErr.maxRel,
    maxAbsErrorAmps: ampErr.maxAbs,
    maxRelErrorAmps: ampErr.maxRel,
  };
}

/**
 * Finding 12 + the follow-up INVESTIGATION: classic stamps a delayed IC
 * output's electrical effect ONE SOLVE AFTER the digital commit that changes
 * it (core/spec/w1-classic-semantics.md §2.4, "The electrical stamp flips on
 * the solve after the commit" — `stampCombinationalIC` reads the OLD, stored
 * `_delayedOutputLevel` for the step that lands exactly on the commit's due
 * time, and `_updateDelayedOutputs` only flips it AFTER that solve). So at a
 * sample where classic's step commits an IC output, classic's own
 * `digitalState` (published post-commit) and its `netV`/`elementI` (solved
 * pre-commit) are mutually INCONSISTENT for one sample — a genuine classic
 * artifact, not a core bug (D8 mandates the core's delta-cycle deliver
 * commit-then-resolve atomically, so the core's own two readouts never have
 * this lag). This is registered as a documented divergence, narrowly: only a
 * netV/elementI mismatch on a net a JUST-CHANGED IC output pin drives, at the
 * exact sample the commit shows up in classic's digitalState, and ONLY when
 * that same (category, id) pair agrees again at the VERY NEXT sample — a
 * seeded fault that does not resolve by the next sample is never excused.
 */
export const STAMP_AFTER_COMMIT_DIVERGENCE_ID = "classic-stamp-after-commit-lag";
const STAMP_AFTER_COMMIT_DECISION_REF =
  "core/spec/w1-classic-semantics.md §2.4 (\"The electrical stamp flips on the solve after the commit\"); " +
  "D8 (delta-cycle exact delivery); review finding 12 + its INVESTIGATION follow-up.";

function stampAfterCommitReason(category: "netV" | "elementI", id: string): string {
  return (
    `classic's electrical stamp flips one solve AFTER its digital commit (spec §2.4), so at this sample ` +
    `classic's ${category} on "${id}" reflects the driver's PRE-commit level while its digitalState already ` +
    `shows the POST-commit one; the core commits and re-solves in the same delta (README W1a point 3 / W1c ` +
    `point 7), so its ${category} is already consistent with its own digitalState. Confirmed transient, not a ` +
    `standing divergence: both sides agree again on this exact quantity at the very next sample.`
  );
}

/** `${componentId}/${pinId}` digitalState keys that changed between two consecutive classic digitalState snapshots AND are one of that component's own digital OUTPUT pins (known-divergences.ts's `digitalOutputPins`) — i.e., a driver commit, not an input changing (an input changing has no "electrical stamp lags the commit" story at all). */
function changedIcOutputPins(
  kindById: ReadonlyMap<string, string>,
  prevDigital: Readonly<Record<string, number>>,
  curDigital: Readonly<Record<string, number>>,
): string[] {
  const changed: string[] = [];
  const keys = new Set([...Object.keys(prevDigital), ...Object.keys(curDigital)]);
  for (const key of keys) {
    if (Object.is(prevDigital[key], curDigital[key])) continue;
    const slash = key.indexOf("/");
    if (slash < 0) continue;
    const componentId = key.slice(0, slash);
    const pin = key.slice(slash + 1);
    const kind = kindById.get(componentId);
    if (kind && digitalOutputPins(kind).has(pin)) changed.push(key);
  }
  return changed;
}

/** The net ids driven by `pinKeys` (`${componentId}/${pinId}`), plus every OTHER component sharing one of those nets — the netV/elementI surface finding 12's lag can plausibly touch. */
function commitExplainedSurface(
  built: CoreNodeBuild,
  pinKeys: readonly string[],
): { readonly nets: ReadonlySet<string>; readonly components: ReadonlySet<string> } {
  const nets = new Set<string>();
  for (const key of pinKeys) {
    const slash = key.indexOf("/");
    const node = built.pinNode.get(corePinKey(key.slice(0, slash), key.slice(slash + 1)));
    if (node === undefined) continue;
    const netId = built.netIdOfNode[node];
    if (netId !== null && netId !== undefined) nets.add(netId);
  }
  const components = new Set<string>();
  for (const net of built.nets) {
    if (!nets.has(net.id)) continue;
    for (const [componentId] of net.pins) components.add(componentId);
  }
  return { nets, components };
}

/**
 * Gate 2d's classic-vs-core stimuli comparison (run-entry.ts's
 * `runStimuliEntry`, W1b): compares two equal-length `StimulusStepSample`
 * sequences produced by `replayStimuliOnClassic`/`replayStimuliOnCore` on the
 * SAME phase list and the SAME fixed step schedule (stimuli.ts's
 * `STIMULUS_FIXED_STEP_S`), so `classic[i]`/`core[i]` are always the same
 * step of the same phase.
 *
 * Reuses `compareSamples`'s own comparators (same tolerances, same
 * netV/elementI/digitalState/simTime checks) rather than duplicating them —
 * the only difference is that every `Mismatch` this produces also carries
 * `phaseIndex` (the worker brief: "with the phase index in each mismatch"),
 * and there is no failures/warnings dimension to compare: `StimulusStepSample`
 * (unlike `SampleReadout`) never carries either, since `replayStimuliOnCore`/
 * `replayStimuliOnClassic` throw instead of returning a sample for a phase
 * that does not complete cleanly (both files' own "did not complete cleanly"
 * checks) — there is nothing for a `failures`/`warnings` record to report by
 * the time a sample reaches here.
 *
 * Throws (a harness bug, not a circuit fact) if the sequences have different
 * lengths, exactly like `compareSamples` — both sides replay the identical
 * phase list at the identical fixed step, so a length mismatch can only mean
 * one side's schedule diverged from the other's.
 *
 * `circuit`, when given, enables finding 12's stamp-after-commit-lag
 * exemption (this file's own header above `STAMP_AFTER_COMMIT_DIVERGENCE_ID`):
 * a netV/elementI mismatch on a net a just-committed IC output pin drives,
 * that resolves again by the very next sample, is moved into the returned
 * `knownDivergences` instead of `mismatches`. Omitting `circuit` (or passing
 * one where no IC ever commits mid-phase) simply finds nothing to exempt —
 * every mismatch stays in `mismatches`, exactly as before this exemption
 * existed.
 */
export function compareStimuliSamples(
  classic: readonly StimulusStepSample[],
  core: readonly StimulusStepSample[],
  circuit?: SimCircuit,
): CompareResult & { readonly knownDivergences: readonly KnownDivergenceHit[] } {
  if (classic.length !== core.length) {
    throw new Error(
      `compareStimuliSamples: sample count mismatch (classic ${classic.length}, core ${core.length}) — ` +
        "the two replays used different phase/step schedules; this is a harness bug, not a circuit result.",
    );
  }

  const voltErr: ErrorAccumulator = { maxAbs: 0, maxRel: 0 };
  const ampErr: ErrorAccumulator = { maxAbs: 0, maxRel: 0 };

  // Per-sample-index mismatch lists (not one flat array) — the exemption
  // pass below needs to look at "does this SAME (category, id) mismatch
  // still appear at i+1", which is only answerable if each sample's own
  // mismatches are kept separately addressable by index first.
  const perSample: Mismatch[][] = [];

  for (let i = 0; i < classic.length; i++) {
    const c = classic[i]!;
    const r = core[i]!;
    const local: Mismatch[] = [];
    // classic's own simTime/phaseIndex label the sample — see compareSamples's
    // own comment on why classic's copy is the one used to label a mismatch.
    const sampleTime = c.simTime;
    const phaseIndex = c.phaseIndex;
    compareSimTime(sampleTime, c.simTime, r.simTime, local, phaseIndex);
    compareNumericRecord("netV", sampleTime, c.netV, r.netV, VOLT_ABS_TOL, VOLT_REL_TOL, local, voltErr, phaseIndex);
    compareNumericRecord("elementI", sampleTime, c.elementI, r.elementI, AMP_ABS_TOL, AMP_REL_TOL, local, ampErr, phaseIndex);
    compareDigitalState(sampleTime, c.digitalState, r.digitalState, local, phaseIndex);
    perSample.push(local);
  }

  const built = circuit ? buildCoreNodes(circuit) : null;
  const kindById = circuit ? new Map(circuit.components.map((comp) => [comp.id, comp.kind])) : null;

  const mismatches: Mismatch[] = [];
  const knownDivergences: KnownDivergenceHit[] = [];

  for (let i = 0; i < classic.length; i++) {
    const local = perSample[i]!;

    // The commit-explained surface for THIS sample: which nets/components a
    // just-changed IC output pin (per classic's OWN digitalState, i>0 only —
    // there is no "previous sample" at i=0) plausibly touches. Built lazily,
    // only when this sample actually has local mismatches worth checking.
    let surface: { readonly nets: ReadonlySet<string>; readonly components: ReadonlySet<string> } | null = null;
    if (built && kindById && i > 0 && local.length > 0) {
      const changedPins = changedIcOutputPins(kindById, classic[i - 1]!.digitalState, classic[i]!.digitalState);
      if (changedPins.length > 0) surface = commitExplainedSurface(built, changedPins);
    }

    const next = i + 1 < perSample.length ? perSample[i + 1]! : null;

    for (const m of local) {
      const onSurface =
        surface !== null &&
        ((m.category === "netV" && surface.nets.has(m.id)) ||
          (m.category === "elementI" && surface.components.has(m.id)));
      // "Agrees at the next sample" means the SAME (category, id) pair is
      // simply absent from that sample's own mismatch list — never true when
      // there IS no next sample (the last sample of the whole replay), so a
      // mismatch there can never be excused this way.
      const explainedAtNext = onSurface && next !== null && !next.some((n) => n.category === m.category && n.id === m.id);

      if (explainedAtNext) {
        knownDivergences.push({
          mismatch: m,
          divergenceId: STAMP_AFTER_COMMIT_DIVERGENCE_ID,
          decisionRef: STAMP_AFTER_COMMIT_DECISION_REF,
          reason: stampAfterCommitReason(m.category as "netV" | "elementI", m.id),
        });
      } else {
        mismatches.push(m);
      }
    }
  }

  return {
    mismatches,
    knownDivergences,
    maxAbsErrorVolts: voltErr.maxAbs,
    maxRelErrorVolts: voltErr.maxRel,
    maxAbsErrorAmps: ampErr.maxAbs,
    maxRelErrorAmps: ampErr.maxRel,
  };
}
