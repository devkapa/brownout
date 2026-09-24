/**
 * Shared types for the dual-engine equivalence harness (core/README.md's
 * contract is what this harness checks; this file just names the shapes the
 * rest of test/core/equivalence/** pass around).
 *
 * A CorpusEntry is deliberately engine-agnostic and circuit-agnostic: it is
 * metadata ABOUT a circuit (where it came from, what it contains, how long
 * to sample it for), never the circuit itself. Callers load the actual
 * SimCircuit JSON separately (brownout-fixtures.ts / synthetic-fixtures.ts /
 * external-corpus.ts each pair a CorpusEntry with its circuit) so this type
 * can be logged, tabulated and JSON-serialized without ever accidentally
 * embedding a proprietary circuit's full topology in a report.
 */

import type { SimFailure, SimWarning } from "../../../src/sim/engine/sim-engine.js";

/** Where a CorpusEntry's circuit came from. External corpora supply their own string (falling back to the manifest's `producer`); this harness's own two sources use fixed literals. */
export type CorpusOrigin = "brownout-fixture" | "synthetic" | (string & {});

export interface CorpusEntry {
  /** Stable id for reporting: "<origin>/<name>[@variant]". Never a filesystem path. */
  readonly id: string;
  readonly origin: CorpusOrigin;
  readonly name: string;
  readonly variant?: string;
  /** Seconds to sample over; `null` means "use the harness default" (see sample.ts's DEFAULT_DURATION_S). */
  readonly durationS: number | null;
  /** Distinct component kinds this circuit contains, sorted. Always derived from the circuit itself (kinds-of.ts's kindsOf) — never trusted from an external manifest, so a stale manifest can't hide a kind from the not-ported check or credit a kind the circuit doesn't have. */
  readonly kinds: readonly string[];
  /**
   * The external manifest's own `kinds` list, sorted, present only when it
   * disagrees with `kinds` above. `kinds` always wins for every harness
   * decision (not-ported, the per-kind table); this field exists only so a
   * stale or hand-edited manifest is surfaced in the report instead of
   * silently trusted. `undefined` for a local fixture (no manifest exists)
   * or when the manifest agrees with the circuit.
   */
  readonly manifestKindsMismatch?: readonly string[];
  /**
   * "verify-script" means the circuit's producer (devolt) additionally
   * checks it under scripted stimuli this corpus snapshot does not carry —
   * a button press, a timed input sequence, and so on. This harness still
   * runs the entry, but a pass on it is NOT parity with that check, so the
   * report marks it "unstimulated" and the per-kind table counts it
   * separately. `null` for every local fixture and for an external entry
   * whose manifest omits the field.
   */
  readonly stimulus: "verify-script" | null;
  /** Absolute path the circuit JSON was loaded from. Diagnostics only — never printed for an external-corpus entry in anything this repo commits. */
  readonly file: string;
}

/** One accepted-step-equivalent readout from either engine, at one sample time — see sample.ts. */
export interface SampleReadout {
  simTime: number;
  netV: Record<string, number>;
  elementI: Record<string, number>;
  digitalState: Record<string, number>;
  failures: Record<string, SimFailure>;
  warnings: Record<string, SimWarning>;
}

/** One comparison failure, sample-and-quantity addressable so a report can point straight at the disagreement. */
export interface Mismatch {
  readonly sampleTime: number;
  readonly category: "netV" | "elementI" | "digitalState" | "failures" | "warnings" | "simTime";
  /** Net id, component id, failure/warning key, "*" for a whole-record comparison (warning code sets), or "simTime" for the simTime category. */
  readonly id: string;
  readonly classicValue: unknown;
  readonly coreValue: unknown;
  readonly detail: string;
}

/** compareSamples()'s full result — see compare.ts. */
export interface CompareResult {
  /** Every mismatch found, uncapped (run-entry.ts caps what it reports). Empty means the two runs agree within tolerance everywhere. */
  readonly mismatches: readonly Mismatch[];
  readonly maxAbsErrorVolts: number;
  readonly maxRelErrorVolts: number;
  readonly maxAbsErrorAmps: number;
  readonly maxRelErrorAmps: number;
}

export type EntryStatus = "not-ported" | "core-declined" | "pass" | "fail" | "error";

interface EntryResultBase {
  readonly kinds: readonly string[];
  /** Wall time (ms) for that engine's part of this entry, or null if it never ran (not-ported skips both; a classic-side error leaves coreWallMs null). */
  readonly classicWallMs: number | null;
  readonly coreWallMs: number | null;
}

export interface NotPortedResult extends EntryResultBase {
  readonly status: "not-ported";
  readonly missingKinds: readonly string[];
}

export interface CoreDeclinedResult extends EntryResultBase {
  readonly status: "core-declined";
  readonly reasons: readonly string[];
}

/** Shared by "pass" and "fail": both ran a real comparison and have real error stats. */
interface ComparedResultBase extends EntryResultBase {
  readonly maxAbsErrorVolts: number;
  readonly maxRelErrorVolts: number;
  readonly maxAbsErrorAmps: number;
  readonly maxRelErrorAmps: number;
}

export interface PassResult extends ComparedResultBase {
  readonly status: "pass";
}

export interface FailResult extends ComparedResultBase {
  readonly status: "fail";
  /** First ~10 mismatches (see run-entry.ts); totalMismatches is the uncapped count. */
  readonly mismatches: readonly Mismatch[];
  readonly totalMismatches: number;
}

export interface ErrorResult extends EntryResultBase {
  readonly status: "error";
  readonly engine: "classic" | "core";
  readonly message: string;
}

export type EntryResult = NotPortedResult | CoreDeclinedResult | PassResult | FailResult | ErrorResult;

/** One kind's row in the per-kind pass table (kinds.ts's CORE_IMPLEMENTED_KINDS/CORE_SUPPORTED_KINDS gate on this). */
export interface KindRow {
  readonly kind: string;
  readonly implemented: boolean;
  readonly supported: boolean;
  /** Entries containing this kind, regardless of outcome. */
  readonly fixtures: number;
  /** Entries containing this kind where both engines actually ran a comparison (pass + fail; an "error" entry also ran but is tracked separately — see table.ts). */
  readonly ran: number;
  readonly pass: number;
  readonly fail: number;
  readonly declined: number;
  readonly notPorted: number;
  /** Entries containing this kind that ended in "error" — not one of the table's required columns, kept here so nothing is silently dropped (table.ts's rendered markdown omits it when zero everywhere). */
  readonly errored: number;
  /** Entries containing this kind whose `stimulus` is "verify-script" (CorpusEntry's own doc) — counted here so a pass count that is mostly unstimulated entries doesn't read as parity with devolt's own scripted check. */
  readonly unstimulated: number;
  readonly status: "pass" | "fail" | "untested" | "not-ported";
}
