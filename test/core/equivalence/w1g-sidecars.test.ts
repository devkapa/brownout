/**
 * Detector self-tests for W1g's gate 2c comparisons (w1g-failures.ts,
 * w1g-sidecars.ts, and the two timed registrations in known-divergences.ts).
 * Each feeds the judgement a deliberately wrong input and requires it to
 * fail, so a clean sweep over the fixtures means something. The fixtures
 * themselves run through equivalence.test.ts's assertion (a).
 */
import { describe, expect, it } from "vitest";
import type { SimFailure } from "../../../src/sim/engine/sim-engine.js";
import {
  explainCommittedTarget,
  explainFailedResetSeed,
  type PendingDelayWindow,
} from "./known-divergences.js";
import type { Mismatch, SlowSidecarModel } from "./types.js";
import { classicFailureMessage, compareFailureSets } from "./w1g-failures.js";
import { judgeSlow, slowTimeTolerance, type SlowExpectation } from "./w1g-sidecars.js";

function failure(overrides: Partial<SimFailure> = {}): SimFailure {
  const base: SimFailure = { componentId: "r1", kind: "resistor_overload", since: 1, value: 0.5, limit: 0.25, message: "" };
  const f = { ...base, ...overrides };
  return { ...f, message: overrides.message ?? classicFailureMessage(f)! };
}

const KEY = "resistor_overload\x00r1\x00";

describe("the failure-set comparison (W1g)", () => {
  it("passes two records that agree within decision 6's tolerance, messages from each side's own numbers", () => {
    const mismatches: Mismatch[] = [];
    compareFailureSets(1, { [KEY]: failure() }, { [KEY]: failure({ value: 0.5002, since: 1.0018 }) }, mismatches);
    expect(mismatches).toEqual([]);
  });

  it("fails a dropped latch", () => {
    const mismatches: Mismatch[] = [];
    compareFailureSets(1, { [KEY]: failure() }, {}, mismatches);
    expect(mismatches.map((m) => m.id)).toEqual([KEY]);
  });

  it("fails a wrong message, and a value past the tolerance", () => {
    const mismatches: Mismatch[] = [];
    compareFailureSets(1, { [KEY]: failure() }, { [KEY]: failure({ message: "r1 dissipated 0.500 W through a 0.250 W rating." }) }, mismatches);
    expect(mismatches.map((m) => m.id)).toEqual([`${KEY}#message`]);
    const more: Mismatch[] = [];
    compareFailureSets(1, { [KEY]: failure() }, { [KEY]: failure({ value: 0.51 }) }, more);
    expect(more.map((m) => m.id)).toEqual([`${KEY}#value`]);
  });

  it("refuses a template that does not rebuild classic's own message", () => {
    const mismatches: Mismatch[] = [];
    compareFailureSets(1, { [KEY]: failure({ message: "something classic never wrote" }) }, { [KEY]: failure() }, mismatches);
    expect(mismatches[0]?.detail).toMatch(/template is wrong/);
  });
});

describe("the gate 2c latch judgement", () => {
  const model: SlowSidecarModel = { kind: "slow", horizonS: 1.4, checks: [{ check: "resistor-overload", resistor: "r1" }], classicS: [1.0018] };
  const exp: SlowExpectation = {
    events: [{ label: "r1 resistor_overload #0", type: "latch", componentId: "r1", kind: "resistor_overload", t: 1, value: 0.5, limit: 0.25 }],
    readings: [],
  };
  const classic = { latches: [{ key: "resistor_overload/r1/", t: 1.0018, failure: failure({ since: 1.0018 }) }], ptc: [], clears: [], readingAt: new Map<string, number>() };
  const core = (latches: SimFailure[]) => ({ latches, ptcAt: new Map(), failuresAt: new Map(), readingAt: new Map<string, number>() });

  it("passes the core at the analytic time", () => {
    expect(judgeSlow(model, exp, core([failure({ since: 1 + 0.5e-9 })]), classic).mismatches).toEqual([]);
  });

  it("fails a dropped latch", () => {
    expect(judgeSlow(model, exp, core([]), classic).mismatches.map((m) => m.id)).toContain("core latches");
  });

  it("fails a wrong since on an integrated kind, just past 1e-9 s + 1e-9 relative", () => {
    const since = 1 + 1.01 * slowTimeTolerance(1);
    expect(judgeSlow(model, exp, core([failure({ since })]), classic).mismatches.map((m) => m.id)).toEqual(["r1 resistor_overload #0 since"]);
  });

  it("fails a wrong message", () => {
    const bad = failure({ message: "r1 failed." });
    expect(judgeSlow(model, exp, core([bad]), classic).mismatches.map((m) => m.id)).toEqual(["r1 resistor_overload #0 message"]);
  });

  it("fails a fixture whose classic never fires (the classic-fires proof)", () => {
    const silent = { ...classic, latches: [] };
    expect(judgeSlow(model, exp, core([failure()]), silent).mismatches.map((m) => m.id)).toContain("classic r1 resistor_overload #0");
  });
});

describe("w1g-sag-committed-target", () => {
  const window: PendingDelayWindow = { componentId: "u1", pinId: "b1", fromS: 0.001, untilS: 0.001 + 4e-8 };
  const hit = (t: number, id = "output_sag\x00u1\x00b1"): Mismatch => ({ sampleTime: t, category: "failures", id, classicValue: {}, coreValue: undefined, detail: "" });

  it("excuses a hit inside the pin's pending delay", () => {
    expect(explainCommittedTarget(hit(0.001 + 1e-8), [window])).toBe(true);
  });

  it("fails a hit outside the delay, on either side of it", () => {
    expect(explainCommittedTarget(hit(0.001 - 1e-9), [window])).toBe(false);
    expect(explainCommittedTarget(hit(0.001 + 4e-8), [window])).toBe(false);
    expect(explainCommittedTarget(hit(0.0015), [window])).toBe(false);
  });

  it("fails a hit on another pin, or of another kind", () => {
    expect(explainCommittedTarget(hit(0.001 + 1e-8, "output_sag\x00u1\x00b2"), [window])).toBe(false);
    expect(explainCommittedTarget(hit(0.001 + 1e-8, "resistor_overload\x00u1\x00"), [window])).toBe(false);
  });
});

describe("w1g-reset-failures-failed-seed", () => {
  const at = (t: number, category: Mismatch["category"], id = "x"): Mismatch => ({ sampleTime: t, category, id, classicValue: 1, coreValue: 0, detail: "" });

  it("excuses the failure and warning sets after a failed reset", () => {
    expect(explainFailedResetSeed(at(1.5, "failures"), [1.2])).toBe(true);
    expect(explainFailedResetSeed(at(1.2, "warnings"), [1.2])).toBe(true);
  });

  it("never excuses lastConverged, nor anything before the reset or after a later successful one", () => {
    expect(explainFailedResetSeed(at(1.5, "sidecar", "lastConverged"), [1.2])).toBe(false);
    expect(explainFailedResetSeed(at(1.1, "failures"), [1.2])).toBe(false);
    expect(explainFailedResetSeed(at(2.5, "failures"), [1.2], [2.0])).toBe(false);
    expect(explainFailedResetSeed(at(1.5, "netV"), [1.2])).toBe(false);
  });
});
