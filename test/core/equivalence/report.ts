/**
 * Renders a HarnessRun as markdown (per-kind table + per-entry detail) or as
 * a plain JSON-serializable object. Both the gate test (equivalence.test.ts,
 * only under CORE_EQUIV_REPORT/CORE_EQUIV_PRINT) and the CLI
 * (scripts/core-equivalence.ts) call this — one renderer, so the two never
 * drift into reporting the same run differently.
 */

import type { HarnessEntryOutcome } from "./table.js";
import type { HarnessRun } from "./harness.js";
import type { EntryResult, KindRow, Mismatch } from "./types.js";

function yesNo(value: boolean): string {
  return value ? "yes" : "no";
}

export function renderMarkdownTable(rows: readonly KindRow[]): string {
  const header =
    "| kind | implemented | supported | fixtures | ran | pass | fail | declined | not-ported | unstimulated | status |";
  const sep = "|---|---|---|---|---|---|---|---|---|---|---|";
  const lines = rows.map(
    (r) =>
      `| ${r.kind} | ${yesNo(r.implemented)} | ${yesNo(r.supported)} | ${r.fixtures} | ${r.ran} | ${r.pass} | ${r.fail} | ${r.declined} | ${r.notPorted} | ${r.unstimulated} | ${r.status} |`,
  );
  return [header, sep, ...lines].join("\n");
}

function fmtNum(value: number): string {
  return Number.isFinite(value) ? value.toExponential(3) : String(value);
}

function fmtMs(value: number | null): string {
  return value === null ? "n/a" : `${value.toFixed(2)} ms`;
}

function describeMismatch(m: Mismatch): string {
  return `[t=${m.sampleTime}] ${m.category}.${m.id}: ${m.detail}`;
}

/** Notes appended to an entry's report line for conditions that are not failures but change how the outcome should be read (CorpusEntry's own docs on `stimulus` / `manifestKindsMismatch`). */
function describeEntryNotes(entry: HarnessEntryOutcome["entry"]): string {
  const notes: string[] = [];
  if (entry.stimulus === "verify-script") {
    notes.push("unstimulated: devolt additionally checks this under scripted stimuli this corpus does not carry — a pass here is not parity with that check");
  }
  if (entry.manifestKindsMismatch) {
    notes.push(
      `manifest/circuit kinds mismatch: manifest says [${entry.manifestKindsMismatch.join(", ") || "none"}], ` +
        `circuit actually has [${entry.kinds.join(", ") || "none"}]`,
    );
  }
  return notes.length > 0 ? ` (${notes.join("; ")})` : "";
}

function describeEntry(entry: HarnessEntryOutcome["entry"], result: EntryResult): string {
  const head = `- **${entry.id}** (${entry.kinds.join(", ") || "no components"}) -> ${result.status}${describeEntryNotes(entry)}`;
  switch (result.status) {
    case "not-ported":
      return `${head} (missing: ${result.missingKinds.join(", ")})`;
    case "core-declined":
      // classicWallMs/coreWallMs are always null here: core runs first
      // (run-entry.ts), and a declined circuit never reaches classic.
      return `${head} (${result.reasons.join("; ")})`;
    case "pass":
      return (
        `${head} (max abs V ${fmtNum(result.maxAbsErrorVolts)}, max rel V ${fmtNum(result.maxRelErrorVolts)}, ` +
        `max abs I ${fmtNum(result.maxAbsErrorAmps)}, max rel I ${fmtNum(result.maxRelErrorAmps)}; ` +
        `classic ${fmtMs(result.classicWallMs)}, core ${fmtMs(result.coreWallMs)})`
      );
    case "fail": {
      const shown = result.mismatches.map((m) => `\n  - ${describeMismatch(m)}`).join("");
      return `${head} (${result.totalMismatches} mismatch(es)):${shown}`;
    }
    case "error":
      return `${head} ([${result.engine}] ${result.message})`;
  }
}

export function renderEntryDetail(outcomes: readonly HarnessEntryOutcome[]): string {
  return outcomes.map(({ entry, result }) => describeEntry(entry, result)).join("\n");
}

export function renderMarkdownReport(run: HarnessRun): string {
  return [
    "# brownout/core dual-engine equivalence report",
    "",
    "## Per-kind table",
    "",
    renderMarkdownTable(run.kindTable),
    "",
    "## Entries",
    "",
    renderEntryDetail(run.outcomes),
    "",
  ].join("\n");
}

export function toJsonReport(run: HarnessRun): unknown {
  return {
    kindTable: run.kindTable,
    entries: run.outcomes.map(({ entry, result }) => ({ entry, result })),
  };
}
