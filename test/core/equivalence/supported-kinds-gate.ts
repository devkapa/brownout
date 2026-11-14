/**
 * The CI gate behind equivalence.test.ts's "assertion b": every kind in
 * CORE_SUPPORTED_KINDS (kinds.ts) must have proof, in the per-kind table,
 * that it ran on at least one corpus entry and never failed. Filtering the
 * table's own rows (`row.kind` for `kind in CORE_SUPPORTED_KINDS`) cannot
 * catch a supported kind that has NO row at all — no always-on entry
 * contains it — because there is nothing in the table to filter to. This
 * function iterates the supported set itself instead, so a missing row is
 * exactly as much of a failure as a "fail"/"untested" one.
 *
 * `supportedKinds` is a parameter, not a fixed import of CORE_SUPPORTED_KINDS,
 * so a test can inject a set the real corpus's kindTable doesn't cover a row
 * for and prove this check actually fires — without widening production's
 * CORE_SUPPORTED_KINDS or adding a fixture to the always-on corpus just to
 * exercise the failure path.
 */

import type { KindRow } from "./types.js";

export interface SupportedKindOffense {
  readonly kind: string;
  readonly reason: string;
}

export function checkSupportedKindsGate(
  kindTable: readonly KindRow[],
  supportedKinds: ReadonlySet<string>,
): SupportedKindOffense[] {
  const rowsByKind = new Map(kindTable.map((row) => [row.kind, row] as const));
  const offenses: SupportedKindOffense[] = [];

  for (const kind of supportedKinds) {
    const row = rowsByKind.get(kind);
    if (!row) {
      offenses.push({
        kind,
        reason: "no corpus entry exercises this kind — no row for it in the per-kind table",
      });
      continue;
    }
    if (row.ran === 0) {
      offenses.push({
        kind,
        reason: `ran on no entry (ran=0, table status "${row.status}")`,
      });
      continue;
    }
    if (row.status !== "pass") {
      offenses.push({ kind, reason: `table status is "${row.status}", not "pass"` });
    }
  }

  return offenses;
}
