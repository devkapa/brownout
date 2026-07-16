/**
 * traceToCsv — convert a TraceSnapshot to RFC 4180-compliant CSV.
 *
 * WHY here rather than in apps/sim:
 *   This is a pure data transformation with no DOM/React/Next.js dependencies.
 *   Co-locating it in simcore lets tests run in Node without a browser harness,
 *   and keeps the CSV format close to the data structure that produced it.
 *
 * WHY RFC 4180 quoting:
 *   Channel labels are user-supplied (net IDs or human renames). A user may
 *   name a net "V(in,out)" which contains a comma — RFC 4180 double-quote
 *   wrapping handles that without inventing a custom escaping scheme.
 *
 * WHY String(v) instead of toFixed:
 *   Full-precision round-trip fidelity. The simulation engine works at
 *   IEEE 754 double precision; rounding to e.g. 6 decimal places would
 *   silently destroy information for values like 1.234567890123456e-5.
 *   The consumer (a spreadsheet or script) can always round; we cannot
 *   recover lost precision after the fact.
 */

import type { TraceSnapshot } from "../sim/trace-recorder.js";

/**
 * Quote a single CSV cell value per RFC 4180:
 *   - If the value contains a comma, double-quote, or newline, wrap in "...".
 *   - Any " inside the value is escaped as "".
 *   - Values that need no escaping are returned as-is (no unnecessary quoting).
 */
function quoteCsvCell(value: string): string {
  if (value.includes(",") || value.includes('"') || value.includes("\n") || value.includes("\r")) {
    return `"${value.replace(/"/g, '""')}"`;
  }
  return value;
}

/**
 * Convert a TraceSnapshot to a CSV string.
 *
 * Header row: time_s, then one column per channel (using label if provided,
 *             otherwise the channel key).
 * Data rows:  one per sample, full IEEE 754 numeric precision.
 *
 * An empty snapshot (no samples) returns a header-only string (one line + LF).
 * A snapshot with no channels returns just "time_s\r\n" rows — degenerate but
 * valid so callers do not need to special-case it.
 *
 * @param snapshot  The captured trace data from TraceRecorder.snapshot().
 * @param labels    Optional channel-key → display-label map for the header.
 *                  Keys not found in this map fall back to the channel key.
 */
export function traceToCsv(
  snapshot: TraceSnapshot,
  labels?: Record<string, string>,
): string {
  const { t, channels } = snapshot;

  // Fail loudly on a broken recorder invariant rather than silently writing a
  // fabricated 0 for a missing sample (a real-looking value a user would trust
  // in a spreadsheet).  The TraceRecorder guarantees aligned lengths; this is a
  // cheap O(channels) boundary assertion on an exported pure function.
  for (const ch of channels) {
    if (ch.values.length !== t.length) {
      throw new Error(
        `traceToCsv: channel "${ch.key}" has ${String(ch.values.length)} samples, expected ${String(t.length)}`,
      );
    }
  }

  // Build header
  const headerCells = ["time_s", ...channels.map((ch) => {
    const label = labels?.[ch.key] ?? ch.key;
    return quoteCsvCell(label);
  })];
  const lines: string[] = [headerCells.join(",")];

  // Build one row per sample
  for (let i = 0; i < t.length; i++) {
    // String(n) preserves full IEEE 754 double precision. toFixed would truncate.
    // A sparse/missing sample is exported as an empty cell, never a plausible
    // zero. Deliberate NaN gap markers remain "NaN" so downstream tools can
    // distinguish a recorded acquisition gap from an empty array slot.
    const cells = [String(t[i]!), ...channels.map((ch) => {
      const value = ch.values[i];
      return value === undefined ? "" : String(value);
    })];
    lines.push(cells.join(","));
  }

  // RFC 4180 mandates CRLF between records; trailing newline is conventional.
  return lines.join("\r\n") + "\r\n";
}
