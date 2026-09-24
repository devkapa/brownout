/**
 * The per-kind pass table (worker brief's "Per-kind table"): one row per
 * kind seen ANYWHERE in the corpus, counting every entry that contains it —
 * even an entry whose overall status is "not-ported" because of some OTHER
 * kind it also contains. That is intentional: a "resistor" row must reflect
 * every entry resistor appears in, not just the ones where resistor alone
 * decided the outcome, or the `fixtures` column would undercount.
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
    let unstimulated = 0;

    for (const { entry, result } of outcomes) {
      if (!entry.kinds.includes(kind)) continue;
      fixtures += 1;
      if (entry.stimulus === "verify-script") unstimulated += 1;
      switch (result.status) {
        case "not-ported":
          notPorted += 1;
          break;
        case "core-declined":
          declined += 1;
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
    } else if (fail > 0 || errored > 0) {
      // An errored entry means equivalence was never actually demonstrated
      // for this kind on that circuit — that must not read as "pass" any
      // more than an outright mismatch would.
      status = "fail";
    } else if (ran > 0) {
      status = "pass";
    } else {
      status = "untested";
    }

    rows.push({ kind, implemented, supported, fixtures, ran, pass, fail, declined, notPorted, errored, unstimulated, status });
  }
  return rows;
}
