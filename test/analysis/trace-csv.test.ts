/**
 * Tests for traceToCsv — RFC 4180 CSV output from a TraceSnapshot.
 */

import { describe, expect, it } from "vitest";
import { traceToCsv } from "../../src/analysis/trace-csv.js";
import type { TraceSnapshot } from "../../src/sim/trace-recorder.js";

function snap(
  t: number[],
  channels: Array<{ key: string; values: number[] }>,
): TraceSnapshot {
  return { t, channels };
}

describe("traceToCsv", () => {
  // ── Empty snapshot ─────────────────────────────────────────────────────────

  it("returns a header-only CSV for an empty snapshot", () => {
    const csv = traceToCsv(snap([], [{ key: "n0", values: [] }]));
    expect(csv).toBe("time_s,n0\r\n");
  });

  it("uses the channel key as header when no label map is provided", () => {
    const csv = traceToCsv(snap([0], [{ key: "n0", values: [1.0] }]));
    expect(csv.startsWith("time_s,n0\r\n")).toBe(true);
  });

  // ── Labels ────────────────────────────────────────────────────────────────

  it("uses labels from the label map in the header", () => {
    const csv = traceToCsv(
      snap([0], [{ key: "n0", values: [1.0] }]),
      { n0: "VCC" },
    );
    expect(csv.startsWith("time_s,VCC\r\n")).toBe(true);
  });

  it("falls back to the channel key when the label map has no entry for the key", () => {
    const csv = traceToCsv(
      snap([0], [
        { key: "n0", values: [1.0] },
        { key: "n1", values: [2.0] },
      ]),
      { n0: "VCC" },
    );
    // n0 gets label, n1 falls back to key
    const header = csv.split("\r\n")[0];
    expect(header).toBe("time_s,VCC,n1");
  });

  // ── RFC 4180 quoting ──────────────────────────────────────────────────────

  it("quotes labels containing commas", () => {
    const csv = traceToCsv(
      snap([0], [{ key: "n0", values: [1.0] }]),
      { n0: "V(in,out)" },
    );
    const header = csv.split("\r\n")[0];
    expect(header).toBe('time_s,"V(in,out)"');
  });

  it("doubles internal double-quotes in labels per RFC 4180", () => {
    const csv = traceToCsv(
      snap([0], [{ key: "n0", values: [1.0] }]),
      { n0: 'V"in"' },
    );
    const header = csv.split("\r\n")[0];
    expect(header).toBe('time_s,"V""in"""');
  });

  it("does not quote plain labels unnecessarily", () => {
    const csv = traceToCsv(
      snap([0], [{ key: "n0", values: [1.0] }]),
      { n0: "VCC" },
    );
    const header = csv.split("\r\n")[0];
    // No extra quotes around a plain label.
    expect(header).toBe("time_s,VCC");
  });

  // ── Full precision round-trip ─────────────────────────────────────────────

  it("preserves full IEEE 754 double precision (no rounding)", () => {
    const v = 1.2345678901234567e-5;
    const csv = traceToCsv(snap([0.1], [{ key: "ch", values: [v] }]));
    const rows = csv.split("\r\n").filter(Boolean);
    const dataRow = rows[1]!;
    // Row format: time_s,value — value is at index 1
    const parsed = parseFloat(dataRow.split(",")[1]!);
    expect(parsed).toBe(v);
  });

  it("preserves time values at full precision", () => {
    const t = 1.2345678901234567;
    const csv = traceToCsv(snap([t], [{ key: "ch", values: [0] }]));
    const rows = csv.split("\r\n").filter(Boolean);
    const parsedT = parseFloat(rows[1]!.split(",")[0]!);
    expect(parsedT).toBe(t);
  });

  // ── Multiple channels and rows ────────────────────────────────────────────

  it("produces correct columns for multiple channels", () => {
    const csv = traceToCsv(
      snap(
        [0, 0.1],
        [
          { key: "n0", values: [1, 2] },
          { key: "n1", values: [3, 4] },
        ],
      ),
    );
    const rows = csv.split("\r\n").filter(Boolean);
    expect(rows).toHaveLength(3); // header + 2 data rows
    expect(rows[0]).toBe("time_s,n0,n1");
    expect(rows[1]).toBe("0,1,3");
    expect(rows[2]).toBe("0.1,2,4");
  });

  // ── CRLF line endings ─────────────────────────────────────────────────────

  it("uses CRLF line endings throughout", () => {
    const csv = traceToCsv(snap([0, 1], [{ key: "n0", values: [1, 2] }]));
    // Every line (including the last) should end with \r\n.
    expect(csv.endsWith("\r\n")).toBe(true);
    // Check that \r\n appears between lines (not just at end).
    expect(csv.includes("\r\n")).toBe(true);
  });

  // ── Snapshot with no channels ─────────────────────────────────────────────

  it("handles a snapshot with no channels gracefully", () => {
    const csv = traceToCsv(snap([0, 1, 2], []));
    const rows = csv.split("\r\n").filter(Boolean);
    expect(rows[0]).toBe("time_s");
    // Data rows contain only time values.
    expect(rows[1]).toBe("0");
  });

  it("exports a sparse missing sample as empty instead of fabricating zero", () => {
    const values = new Array<number>(1);
    const csv = traceToCsv(snap([0], [{ key: "n0", values }]));
    expect(csv.split("\r\n")[1]).toBe("0,");
  });
});
