/**
 * Unit coverage for quiet.ts's guard-window arithmetic and instant selection
 * — the two required seeded faults are "too few quiet instants" and "a
 * quiet-instant picker that would select inside a guard window".
 */

import { describe, expect, it } from "vitest";
import {
  assertInstantsAreQuiet,
  eventInstantsFromTrace,
  guardWindowS,
  requireMinQuietInstants,
  selectQuietInstants,
  TooFewQuietInstantsError,
} from "./quiet.js";
import type { DigitalTraceEvent } from "./trace.js";

describe("guardWindowS", () => {
  it("is 10x the max delay when that dominates 2 classic steps", () => {
    expect(guardWindowS({ maxDelayS: 1e-6, classicStepS: 1e-9 })).toBeCloseTo(1e-5, 15);
  });

  it("is 2 classic steps when that dominates 10x the max delay", () => {
    expect(guardWindowS({ maxDelayS: 1e-9, classicStepS: 1e-3 })).toBeCloseTo(2e-3, 15);
  });

  it("rejects a negative input", () => {
    expect(() => guardWindowS({ maxDelayS: -1, classicStepS: 1e-3 })).toThrow();
    expect(() => guardWindowS({ maxDelayS: 1e-3, classicStepS: -1 })).toThrow();
  });
});

describe("eventInstantsFromTrace", () => {
  it("returns every distinct instant, sorted, deduplicated", () => {
    const trace: DigitalTraceEvent[] = [
      { t: 5, key: "a", value: 1 },
      { t: 1, key: "b", value: 0 },
      { t: 5, key: "c", value: 1 }, // same instant as the first
    ];
    expect(eventInstantsFromTrace(trace)).toEqual([1, 5]);
  });

  it("is empty for an empty trace", () => {
    expect(eventInstantsFromTrace([])).toEqual([]);
  });
});

describe("selectQuietInstants", () => {
  it("keeps every candidate at least `guard` seconds from every event", () => {
    const candidates = [0, 1, 2, 3, 4, 5];
    const events = [2];
    // guard = 0.5: excludes [1.5, 2.5], so 2 is excluded but 1 and 3 survive (both exactly 1 away).
    expect(selectQuietInstants(candidates, events, 0.5)).toEqual([0, 1, 3, 4, 5]);
  });

  it("excludes an instant exactly AT the guard boundary (>= guard survives, < guard does not — checked with a boundary-hugging case)", () => {
    // guard=1: distance exactly 1 must survive ("at least guard away").
    expect(selectQuietInstants([1], [0], 1)).toEqual([1]);
    // distance 0.999 must not.
    expect(selectQuietInstants([0.999], [0], 1)).toEqual([]);
  });

  it("keeps every candidate when there are no events to guard against", () => {
    expect(selectQuietInstants([0, 1, 2], [], 0.5)).toEqual([0, 1, 2]);
  });

  it("rejects a negative or non-finite guard", () => {
    expect(() => selectQuietInstants([0], [1], -1)).toThrow();
    expect(() => selectQuietInstants([0], [1], NaN)).toThrow();
  });
});

describe("seeded fault: too few quiet instants", () => {
  it("requireMinQuietInstants throws TooFewQuietInstantsError when the guard window ate too much of the schedule", () => {
    const candidates = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    const events = [5]; // one big event right in the middle
    const quiet = selectQuietInstants(candidates, events, 4); // excludes (1, 9) -> leaves 0, 1, 9, 10
    expect(quiet).toEqual([0, 1, 9, 10]);
    expect(() => requireMinQuietInstants(quiet, 5)).toThrow(TooFewQuietInstantsError);
  });

  it("does not throw when enough quiet instants survive", () => {
    expect(() => requireMinQuietInstants([1, 2, 3], 3)).not.toThrow();
  });
});

describe("seeded fault: a quiet-instant picker that would select inside a guard window", () => {
  it("assertInstantsAreQuiet catches a broken picker that ignores the guard window entirely", () => {
    // A deliberately-broken stand-in: unlike selectQuietInstants, it does
    // not filter anything — every candidate is returned verbatim, including
    // ones sitting right on top of an event.
    function brokenPicker(candidateTimes: readonly number[], _eventTimes: readonly number[]): number[] {
      return [...candidateTimes];
    }

    const events = [5];
    const guard = 1;
    const picked = brokenPicker([3, 4.5, 5.2, 8], events);
    // 4.5 and 5.2 are both within the guard window of the event at t=5.
    expect(() => assertInstantsAreQuiet(picked, events, guard)).toThrow(/inside the guard window/);
  });

  it("never fires on the real selectQuietInstants's own output (proves the detector isn't just always-throwing)", () => {
    const events = [5];
    const guard = 1;
    const picked = selectQuietInstants([3, 4.5, 5.2, 8], events, guard);
    expect(() => assertInstantsAreQuiet(picked, events, guard)).not.toThrow();
  });
});
