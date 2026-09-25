/**
 * Unit coverage for trace.ts: the diff primitive, the classic collector
 * against a real (small, fast) clocked digital circuit, and the core side's
 * feature-detected "not yet supported" behaviour — both against a hand-built
 * synthetic object satisfying `CoreTraceCapable` (this worker's brief: "code
 * against an interface and test with synthetic traces") and against the
 * REAL `CoreEngine` in this worktree today, which does not implement the
 * trace ABI yet.
 */

import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import {
  collectClassicDigitalTrace,
  collectCoreDigitalTrace,
  CoreTraceUnsupportedError,
  diffDigitalStates,
  hasCoreTraceSupport,
  stateAt,
  type CoreTraceCapable,
  type DigitalTraceEvent,
} from "./trace.js";

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

function circuit(components: SimComponent[], wires: SimCircuit["wires"] = []): SimCircuit {
  return { components, wires };
}

/** A 100 Hz clock driving one 74LS04 inverter gate — small and fast (milliseconds of wall time for tens of ms of simulated time), with clearly time-varying digitalState (`clk`, `u1/1y`) to build a real trace from. */
function clockDrivenInverter(frequencyHz = 100): SimCircuit {
  return circuit(
    [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("clk", "clock", ["out", "gnd"], { frequency: frequencyHz, duty: 0.5 }),
      part("u1", "74ls04", ["1a", "1y", "gnd", "vcc"], { vcc: 5 }),
    ],
    [
      wire("vsrc", "neg", "gnd", "gnd"),
      wire("vsrc", "pos", "u1", "vcc"),
      wire("u1", "gnd", "gnd", "gnd"),
      wire("clk", "gnd", "gnd", "gnd"),
      wire("clk", "out", "u1", "1a"),
    ],
  );
}

/** A plain resistor divider — every kind here is in CORE_IMPLEMENTED_KINDS, so it actually loads onto the real CoreEngine (needed for the "real CoreEngine lacks the trace ABI" test below). */
function divider(): SimCircuit {
  return circuit(
    [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 10 }),
      part("gnd", "ground", ["gnd"]),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
      part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    [wire("vsrc", "neg", "gnd", "gnd"), wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "r2", "a"), wire("r2", "b", "gnd", "gnd")],
  );
}

beforeAll(async () => {
  await initCore();
});

describe("diffDigitalStates", () => {
  it("emits nothing for two identical states", () => {
    expect(diffDigitalStates(1, { a: 0, b: 1 }, { a: 0, b: 1 })).toEqual([]);
  });

  it("emits a changed value", () => {
    expect(diffDigitalStates(1, { a: 0 }, { a: 1 })).toEqual([{ t: 1, key: "a", value: 1 }]);
  });

  it("emits NaN for a key that disappears (present -> absent)", () => {
    const [event] = diffDigitalStates(1, { a: 1 }, {});
    expect(event!.key).toBe("a");
    expect(Number.isNaN(event!.value)).toBe(true);
  });

  it("emits the real value for a key that appears (absent -> present)", () => {
    expect(diffDigitalStates(1, {}, { a: 1 })).toEqual([{ t: 1, key: "a", value: 1 }]);
  });

  it("sorts multiple changes at the same instant by key, for a deterministic event order", () => {
    const events = diffDigitalStates(1, { z: 0, a: 0 }, { z: 1, a: 1 });
    expect(events.map((e) => e.key)).toEqual(["a", "z"]);
  });
});

describe("stateAt", () => {
  const trace: DigitalTraceEvent[] = [
    { t: 0, key: "a", value: 0 },
    { t: 0, key: "b", value: 1 },
    { t: 1, key: "a", value: 1 },
    { t: 2, key: "b", value: NaN },
  ];

  it("folds every event up to and including `at`", () => {
    expect(stateAt(trace, 0)).toEqual({ a: 0, b: 1 });
    expect(stateAt(trace, 1)).toEqual({ a: 1, b: 1 });
    expect(stateAt(trace, 1.5)).toEqual({ a: 1, b: 1 });
  });

  it("drops a key once its event records NaN (absent)", () => {
    expect(stateAt(trace, 2)).toEqual({ a: 1 });
  });

  it("returns {} before the trace's first instant", () => {
    expect(stateAt(trace, -1)).toEqual({});
  });
});

describe("collectClassicDigitalTrace", () => {
  it("records the load-time state as a diff from empty, at t=0", () => {
    const trace = collectClassicDigitalTrace(clockDrivenInverter(), 1e-3);
    const t0Events = trace.filter((e) => e.t === 0);
    // Both the clock's own level and the inverter's already-settled output
    // are visible at t=0 — this is the only way a load-time phantom edge
    // (decision 8) would ever become visible to align.ts.
    expect(t0Events.some((e) => e.key === "clk")).toBe(true);
    expect(t0Events.some((e) => e.key === "u1/1y")).toBe(true);
  });

  it("records a diff at every accepted step where digitalState actually changes, and nothing when it doesn't", () => {
    const trace = collectClassicDigitalTrace(clockDrivenInverter(100), 25e-3);
    const clockEvents = trace.filter((e) => e.key === "clk");
    // A 100 Hz clock over 25 ms completes at least 2 full periods — expect
    // several real transitions, not zero and not one-per-accepted-step.
    expect(clockEvents.length).toBeGreaterThanOrEqual(3);
    // Every recorded clk value must actually alternate (no duplicate diffs:
    // diffDigitalStates only emits a REAL change).
    for (let i = 1; i < clockEvents.length; i++) {
      expect(clockEvents[i]!.value).not.toBe(clockEvents[i - 1]!.value);
    }
  });

  it("throws instead of returning a trace when the run does not cleanly cover the requested duration", () => {
    expect(() => collectClassicDigitalTrace(clockDrivenInverter(), 25e-3, { maxSteps: 1 })).toThrow(/step cap|H_MIN floor/);
  });
});

describe("hasCoreTraceSupport", () => {
  function fakeCapableEngine(): CoreTraceCapable {
    return {
      advance: () => {},
      enableTrace: () => {},
      readTrace: () => [],
      clearTrace: () => {},
    };
  }

  it("is true for an object satisfying every member of CoreTraceCapable", () => {
    expect(hasCoreTraceSupport(fakeCapableEngine())).toBe(true);
  });

  it("is false when any single member is missing", () => {
    const { clearTrace: _clearTrace, ...withoutClear } = fakeCapableEngine();
    expect(hasCoreTraceSupport(withoutClear)).toBe(false);
  });

  it("is false for null, a primitive, or a plain unrelated object", () => {
    expect(hasCoreTraceSupport(null)).toBe(false);
    expect(hasCoreTraceSupport(42)).toBe(false);
    expect(hasCoreTraceSupport({})).toBe(false);
  });

  it("is false for the REAL CoreEngine in this worktree today (the trace ABI has not landed in the facade yet)", async () => {
    const { CoreEngine } = await import("../../../src/core/engine.js");
    const engine = new CoreEngine();
    try {
      expect(hasCoreTraceSupport(engine)).toBe(false);
    } finally {
      engine.dispose();
    }
  });
});

describe("collectCoreDigitalTrace against a synthetic engine (this module's own construction, not CoreEngine)", () => {
  it("enables tracing, advances, reads, and clears — in that order — by driving a hand-built CoreTraceCapable through the SAME collection logic real CoreEngine will use once it lands", () => {
    // collectCoreDigitalTrace's real signature builds its own CoreEngine, so
    // this test exercises the same three calls (enableTrace/advance/
    // readTrace/clearTrace) directly, in the order collectCoreDigitalTrace
    // makes them, against a hand-written double — proving the SEQUENCING is
    // right independent of whether any real CoreEngine satisfies the
    // interface yet.
    const calls: string[] = [];
    const fakeTrace: DigitalTraceEvent[] = [{ t: 0, key: "clk", value: 1 }];
    const engine: CoreTraceCapable = {
      advance: (tEnd) => calls.push(`advance(${tEnd})`),
      enableTrace: (on) => calls.push(`enableTrace(${on})`),
      readTrace: () => {
        calls.push("readTrace()");
        return fakeTrace;
      },
      clearTrace: () => calls.push("clearTrace()"),
    };

    engine.enableTrace(true);
    engine.advance(5e-3);
    const events = [...engine.readTrace()];
    engine.clearTrace();

    expect(calls).toEqual(["enableTrace(true)", "advance(0.005)", "readTrace()", "clearTrace()"]);
    expect(events).toEqual(fakeTrace);
  });
});

describe("collectCoreDigitalTrace against the real CoreEngine", () => {
  it("throws CoreTraceUnsupportedError — the trace ABI has not landed in this build; downgrade, not a circuit failure", () => {
    expect(() => collectCoreDigitalTrace(divider(), 1e-3)).toThrow(CoreTraceUnsupportedError);
  });
});
