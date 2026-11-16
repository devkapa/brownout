/**
 * Renders a HarnessRun as markdown (per-kind table + per-entry detail) or as
 * a plain JSON-serializable object. Both the gate test (equivalence.test.ts,
 * only under CORE_EQUIV_REPORT/CORE_EQUIV_PRINT) and the CLI
 * (scripts/core-equivalence.ts) call this — one renderer, so the two never
 * drift into reporting the same run differently.
 */

import type { HarnessEntryOutcome } from "./table.js";
import type { HarnessRun } from "./harness.js";
import type { EntryResult, HandBoundReport, KindRow, KnownDivergenceHit, Mismatch } from "./types.js";

function yesNo(value: boolean): string {
  return value ? "yes" : "no";
}

export function renderMarkdownTable(rows: readonly KindRow[]): string {
  const header =
    "| kind | implemented | supported | fixtures | ran | pass | fail | declined | not-ported | phase-missing-id | unstimulated | hand-judged | status |";
  const sep = "|---|---|---|---|---|---|---|---|---|---|---|---|---|";
  const lines = rows.map(
    (r) =>
      `| ${r.kind} | ${yesNo(r.implemented)} | ${yesNo(r.supported)} | ${r.fixtures} | ${r.ran} | ${r.pass} | ${r.fail} | ${r.declined} | ${r.notPorted} | ${r.phaseMissingId} | ${r.unstimulated} | ${r.handJudged} | ${r.status} |`,
  );
  return [header, sep, ...lines].join("\n");
}

function fmtNum(value: number): string {
  return Number.isFinite(value) ? value.toExponential(3) : String(value);
}

function fmtMs(value: number | null): string {
  return value === null ? "n/a" : `${value.toFixed(2)} ms`;
}

/** `phaseIndex` (set only by gate 2d's `compareStimuliSamples`, compare.ts) is prefixed on its own so a stimuli-path mismatch names which `StimulusPhase` it came from — a plain (non-stimuli) mismatch never has one. */
function describeMismatch(m: Mismatch): string {
  const phase = m.phaseIndex !== undefined ? `phase ${m.phaseIndex}, ` : "";
  return `[${phase}t=${m.sampleTime}] ${m.category}.${m.id}: ${m.detail}`;
}

/** Notes appended to an entry's report line for conditions that are not failures but change how the outcome should be read (CorpusEntry's own docs on `stimulus` / `manifestKindsMismatch` / `phasesUnsupported`). */
function describeEntryNotes(entry: HarnessEntryOutcome["entry"]): string {
  const notes: string[] = [];
  if (entry.stimulus === "verify-script" && !entry.phases) {
    notes.push("unstimulated: devolt additionally checks this under scripted stimuli this corpus does not carry — a pass here is not parity with that check");
  }
  if (entry.phasesUnsupported) {
    notes.push(`phases unsupported: ${entry.phasesUnsupported}`);
  }
  if (entry.manifestKindsMismatch) {
    notes.push(
      `manifest/circuit kinds mismatch: manifest says [${entry.manifestKindsMismatch.join(", ") || "none"}], ` +
        `circuit actually has [${entry.kinds.join(", ") || "none"}]`,
    );
  }
  if (entry.openInput) {
    notes.push(
      "open-input (decision 9): reads at least one OPEN floating-input pin — the hashed floating model is bit-exact " +
        "with classic except within one classic step of a 10 ms bucket boundary, so a mismatch here is a bucket-" +
        "boundary-alignment question, not necessarily a floating-model disagreement",
    );
  }
  return notes.length > 0 ? ` (${notes.join("; ")})` : "";
}

/** Gate 2c: "Matching mismatches are reported as 'known' and do not fail" — this is that reporting. */
function describeKnownDivergences(knownDivergences: readonly KnownDivergenceHit[]): string {
  if (knownDivergences.length === 0) return "";
  return knownDivergences
    .map((hit) => `\n  - [known: ${hit.divergenceId}, ${hit.decisionRef}] ${describeMismatch(hit.mismatch)} — ${hit.reason}`)
    .join("");
}

/** R122 (a): what a hand-judged entry measured, the core's bound ratio and classic's distance, and the readings the marker does not excuse. */
function describeHandBound(report: HandBoundReport | undefined): string {
  if (!report) return "";
  const q = (x: HandBoundReport["core"]): string =>
    `${x.rises} rises, ${x.falls} falls, worst edge ${fmtNum(x.worstEdgeS)} s` +
    (x.periodS !== undefined ? `, period ${fmtNum(x.periodS)} s` : "") +
    (x.duty !== undefined ? `, duty ${x.duty.toFixed(5)}` : "") +
    (x.pulsesS ? `, pulses ${x.pulsesS.map(fmtNum).join(" / ")} s` : "") +
    (x.finalCount !== undefined ? `, final count ${x.finalCount}` : "");
  const lines = [
    `\n  - [hand-judged: ${report.circuitClass}, R122 (a); bound ${report.bound.relative} x value + ${report.bound.absoluteS} s] hand: ${q(report.hand)}`,
    `\n  - core (${fmtNum(report.coreBoundRatio)} of the bound): ${q(report.core)}`,
    `\n  - classic (recorded, not gated): ${q(report.classic)}`,
    ...report.knownFailingReadings.map(
      (k) => `\n  - [known failing reading: ${k.id} ${k.quantity}] classic ${fmtNum(k.classic)}, core ${fmtNum(k.core)}; ${k.note}`,
    ),
  ];
  return lines.join("");
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
    case "core-stimuli-pending":
      return (
        `${head} (classic replayed ${result.phaseCount} phase(s) in ${fmtMs(result.classicWallMs)}; ` +
        "core stimuli pending — owner decision 1's parameter-update event is W1b, not yet landed)"
      );
    case "phase-missing-id":
      return (
        `${head} (harness finding, neither engine run: ` +
        result.missing.map((m) => `phase ${m.phaseIndex} sets "${m.id}"`).join(", ") +
        ` of ${result.phaseCount} phase(s), which the circuit does not contain)`
      );
    case "pass":
      return (
        `${head} (max abs V ${fmtNum(result.maxAbsErrorVolts)}, max rel V ${fmtNum(result.maxRelErrorVolts)}, ` +
        `max abs I ${fmtNum(result.maxAbsErrorAmps)}, max rel I ${fmtNum(result.maxRelErrorAmps)}; ` +
        `classic ${fmtMs(result.classicWallMs)}, core ${fmtMs(result.coreWallMs)})` +
        describeKnownDivergences(result.knownDivergences) +
        describeHandBound(result.handBound)
      );
    case "fail": {
      const shown = result.mismatches.map((m) => `\n  - ${describeMismatch(m)}`).join("");
      return `${head} (${result.totalMismatches} mismatch(es)):${shown}${describeKnownDivergences(result.knownDivergences)}${describeHandBound(result.handBound)}`;
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
