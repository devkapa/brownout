/**
 * Unit tests for scripts/bench-core-impl.ts's pure reporting math --
 * `summarize` and `printSummaryLine` -- run directly, without booting the
 * wasm core or driving a real HeadlessRunner/CoreEngine (bench-core-impl.ts
 * guards its `main()` CLI entry point behind an `isMain` check for exactly
 * this: importing it for its exported pieces must not also launch a real
 * benchmark run). This file matches adapter.test.ts's and
 * gate-reporter.test.ts's pattern: a plain unit test under the ROOT
 * vitest.config.ts project (test/**\/*.test.ts), not under
 * vitest.core-suites.config.ts (which only ever runs the allowlisted classic
 * suite files).
 *
 * Finding 14: benchClassic/benchCore must divide each rep's CPU time by the
 * simulated time THAT REP actually covered (not a fixed requested duration),
 * and printSummaryLine must flag and report an early stop (hitStepCap,
 * hitMinStep, or a core non-convergence) instead of printing a
 * ms/simulated-s number for it. The numbers below reproduce the finding's
 * own reported evidence: a 200 kHz-clock circuit that hits the classic
 * engine's step cap at ~0.5 s of the requested 1 s, with the reported
 * cpuMs=6535.193 understating the true per-simulated-second cost by ~2x
 * when divided by the requested 1 s instead of the ~0.5 s actually covered.
 */
import { describe, expect, it, vi } from "vitest";
import { summarize, printSummaryLine, type Rep } from "../../../scripts/bench-core-impl.js";

describe("bench-core-impl summarize -- divides by simulated time actually covered", () => {
  it("divides a rep's cpuMs by ITS OWN simulatedS, not the requested duration, when they differ", () => {
    // The finding's own evidence: requested 1 s, classic's HeadlessRunner
    // hit the step cap after covering only ~0.5 s.
    const reps: Rep[] = [{ cpuMs: 6535.193, wallMs: 6600, simulatedS: 0.4999987, earlyStop: "hitStepCap" }];
    const summary = summarize(reps, /* requestedS */ 1);
    const correct = 6535.193 / 0.4999987; // ~13070.4 ms/simulated-s
    const wrongFixedDivisor = 6535.193 / 1; // ~6535.2 -- what the pre-fix code printed, ~2x too low
    expect(summary.cpuMsPerSimSecond.median).toBeCloseTo(correct, 3);
    expect(summary.cpuMsPerSimSecond.median).not.toBeCloseTo(wrongFixedDivisor, 0);
  });

  it("still divides by the requested duration when a rep covers it exactly (no regression on the happy path)", () => {
    const reps: Rep[] = [
      { cpuMs: 900, wallMs: 950, simulatedS: 1 },
      { cpuMs: 1100, wallMs: 1150, simulatedS: 1 },
      { cpuMs: 1000, wallMs: 1050, simulatedS: 1 },
    ];
    const summary = summarize(reps, 1);
    expect(summary.cpuMsPerSimSecond.median).toBeCloseTo(1000, 6);
    expect(summary.cpuMsPerSimSecond.min).toBeCloseTo(900, 6);
    expect(summary.cpuMsPerSimSecond.max).toBeCloseTo(1100, 6);
    expect(summary.earlyStops).toEqual([]);
  });

  it("records which rep(s) stopped early and why", () => {
    const reps: Rep[] = [
      { cpuMs: 1000, wallMs: 1050, simulatedS: 1 },
      { cpuMs: 500, wallMs: 550, simulatedS: 0.25, earlyStop: "hitMinStep" },
    ];
    const summary = summarize(reps, 1);
    expect(summary.earlyStops).toEqual([{ rep: 1, reason: "hitMinStep", simulatedS: 0.25 }]);
  });
});

describe("bench-core-impl printSummaryLine -- flags an early stop instead of printing a number", () => {
  it.each(["hitStepCap", "hitMinStep", "coreNonConvergence"] as const)(
    "reports %s instead of a ms/simulated-s figure",
    (reason) => {
      const logSpy = vi.spyOn(console, "log").mockImplementation(() => {});
      try {
        const reps: Rep[] = [{ cpuMs: 6535.193, wallMs: 6600, simulatedS: 0.5, earlyStop: reason }];
        printSummaryLine("classic", summarize(reps, 1));
        expect(logSpy).toHaveBeenCalledTimes(1);
        const printed = logSpy.mock.calls[0]!.join(" ");
        expect(printed).toContain("EARLY STOP");
        expect(printed).toContain(reason);
        // The explanatory text may still name the "ms/simulated-s" unit
        // (see the message above) -- what must never appear is the numeric
        // report line's own shape, "median <number> ms/simulated-s".
        expect(printed).not.toMatch(/median\s+[\d.]+\s+ms\/simulated-s/);
      } finally {
        logSpy.mockRestore();
      }
    },
  );

  it("prints the normal ms/simulated-s line when no rep stopped early", () => {
    const logSpy = vi.spyOn(console, "log").mockImplementation(() => {});
    try {
      const reps: Rep[] = [{ cpuMs: 1000, wallMs: 1050, simulatedS: 1 }];
      printSummaryLine("classic", summarize(reps, 1));
      expect(logSpy).toHaveBeenCalledTimes(1);
      const printed = logSpy.mock.calls[0]!.join(" ");
      expect(printed).toMatch(/ms\/simulated-s/);
      expect(printed).not.toContain("EARLY STOP");
    } finally {
      logSpy.mockRestore();
    }
  });
});
