// Test-support copy extracted from devolt (B2 corpus migration). App-domain
// module that the engine corpus exercises but the published package does not
// ship; only the import specifiers were rewritten to brownout paths.
/**
 * Diagnostics engine entry point.
 *
 * Orchestrates the static analysis pass (and, later, the live-readings pass)
 * into a single deduplicated, sorted finding list.
 *
 * Wave C architecture:
 *   C1 (this file + static.ts) — static analysis, no live store.
 *   C2 — live-readings analyser wired to the live simulation store.
 *   The seam between them is clearly marked below.
 */

import type { DiagnosticFinding, DiagnosticsInput } from "./types.js";
import { analyzeStatic } from "./static.js";
import { analyzeLive } from "./live.js";

// Re-export all public types so callers need only import from this module.
export * from "./types.js";

const SEVERITY_ORDER: Record<string, number> = { critical: 0, warning: 1, info: 2 };

/**
 * Run the full diagnostics pipeline and return a deduplicated, sorted finding list.
 *
 * Deduplication: when the static pass and the live pass (C2) emit a finding
 * with the same key, the first one wins. Static findings come first in the
 * merge, so a static finding always takes precedence over a redundant live one.
 *
 * Sort order: critical → warning → info, then by id, then by key.
 * This ordering is stable and deterministic so snapshot tests never flicker.
 */
export function runDiagnostics(input: DiagnosticsInput): DiagnosticFinding[] {
  const staticFindings = analyzeStatic(input);

  // ── C2 SEAM ─────────────────────────────────────────────────────────────
  // Live-readings analyser wired in C2. Returns [] when live is absent,
  // not converged, or too early in the run (see live.ts for the guards).
  const liveFindings: DiagnosticFinding[] = analyzeLive(input);
  // ── END C2 SEAM ──────────────────────────────────────────────────────────

  // Merge: static first, then live; deduplicate by key (first wins).
  const seen = new Set<string>();
  const merged: DiagnosticFinding[] = [];
  for (const f of [...staticFindings, ...liveFindings]) {
    if (seen.has(f.key)) continue;
    seen.add(f.key);

    // Sort componentIds and netIds inside each finding for deterministic output.
    merged.push({
      ...f,
      componentIds: [...f.componentIds].sort(),
      netIds: [...f.netIds].sort(),
    });
  }

  // Sort: severity (critical first), then id, then key.
  merged.sort((a, b) => {
    const sev = (SEVERITY_ORDER[a.severity] ?? 99) - (SEVERITY_ORDER[b.severity] ?? 99);
    if (sev !== 0) return sev;
    const id = a.id.localeCompare(b.id);
    if (id !== 0) return id;
    return a.key.localeCompare(b.key);
  });

  return merged;
}
