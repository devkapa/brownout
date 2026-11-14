/**
 * The per-kind pass table (worker brief's "Per-kind table"): one row per
 * kind seen ANYWHERE in the corpus, counting every entry that contains it —
 * even an entry whose overall status is "not-ported" because of some OTHER
 * kind it also contains. That is intentional: a "resistor" row must reflect
 * every entry resistor appears in, not just the ones where resistor alone
 * decided the outcome, or the `fixtures` column would undercount.
 *
 * A "pass" here already accounts for known-divergences.ts's registry:
 * run-entry.ts classifies every mismatch (including a digital-alignment one,
 * gate 2a) before deciding `EntryResult.status`, so an owner-approved
 * divergence (e.g. the SAP-1 display's startup phantom edge) never turns a
 * "pass" into a "fail" here — it shows up only in that pass result's own
 * `knownDivergences` list (types.ts), not in this table.
 */

import { CORE_IMPLEMENTED_KINDS, CORE_SUPPORTED_KINDS } from "../../../src/core/kinds.js";
import type { CorpusEntry, EntryResult, KindRow } from "./types.js";

export interface HarnessEntryOutcome {
  entry: CorpusEntry;
  result: EntryResult;
}

export function buildKindTable(outcomes: readonly HarnessEntryOutcome[]): KindRow[] {
  const kinds = new Set<string>();
  for (const { entry } of outcomes) {
    for (const kind of entry.kinds) kinds.add(kind);
  }

  const rows: KindRow[] = [];
  for (const kind of [...kinds].sort()) {
    let fixtures = 0;
    let pass = 0;
    let fail = 0;
    let declined = 0;
    let notPorted = 0;
    let errored = 0;
    let phaseMissingId = 0;
    let unstimulated = 0;

    for (const { entry, result } of outcomes) {
      if (!entry.kinds.includes(kind)) continue;
      fixtures += 1;
      // Gate 2d: only an entry whose phases were NEVER replayed (no
      // `entry.phases` at all — the field export failed, or this snapshot
      // predates it) counts as "unstimulated" now. One WITH phases got a
      // real classic-side replay (result.status "core-stimuli-pending"),
      // so counting it here too would understate how much this entry
      // actually exercised, even though the core side is still pending.
      if (entry.stimulus === "verify-script" && !entry.phases) unstimulated += 1;
      switch (result.status) {
        case "not-ported":
          notPorted += 1;
          break;
        case "core-declined":
          declined += 1;
          break;
        case "core-stimuli-pending":
          // The core side declined for a stimuli-specific reason (owner
          // decision 1's mechanism is W1b, not a kind/topology rejection) —
          // folded into the same column as core-declined rather than adding
          // a new one, since from this table's perspective both mean "the
          // core did not attempt a real comparison here".
          declined += 1;
          break;
        case "phase-missing-id":
          // A phase that sets a component the circuit lacks would have run as
          // a no-op on both engines (ruling R41 (3)): a harness finding,
          // counted on its own so it can never be read as a pass or a decline.
          phaseMissingId += 1;
          break;
        case "pass":
          pass += 1;
          break;
        case "fail":
          fail += 1;
          break;
        case "error":
          // Both engines were attempted (this kind was implemented enough to
          // try), but the run never produced a comparable readout. Neither
          // "ran and passed" nor cleanly "declined" — counted separately
          // (see KindRow.errored) rather than folded into fail, so a fail
          // count of 0 with errored > 0 does not silently read as "safe".
          errored += 1;
          break;
      }
    }

    const ran = pass + fail;
    const implemented = CORE_IMPLEMENTED_KINDS.has(kind);
    const supported = CORE_SUPPORTED_KINDS.has(kind);

    let status: KindRow["status"];
    if (!implemented) {
      status = "not-ported";
    } else if (fail > 0 || errored > 0 || phaseMissingId > 0) {
      // An errored or phase-missing-id entry means equivalence was never
      // actually demonstrated for this kind on that circuit — that must not
      // read as "pass" any more than an outright mismatch would.
      status = "fail";
    } else if (ran > 0) {
      status = "pass";
    } else {
      status = "untested";
    }

    rows.push({ kind, implemented, supported, fixtures, ran, pass, fail, declined, notPorted, errored, phaseMissingId, unstimulated, status });
  }
  return rows;
}
