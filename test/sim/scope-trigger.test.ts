import { describe, expect, it } from "vitest";
import {
  ScopeTrigger,
  findTriggerCrossings,
  extractTriggerFrame,
  type TriggerConfig,
  type TriggerWindow,
} from "../../src/sim/scope-trigger.js";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Build a single-channel window from sample values at a fixed dt. */
function chan(values: number[], dt = 0.1, t0 = 0): TriggerWindow {
  const t = values.map((_, i) => t0 + i * dt);
  return { t, values: [values] };
}

/**
 * One period of a 0..1 square wave: `low` samples low then `high` samples high,
 * repeated. Returns a single-channel window starting at t0.
 */
function square(periods: number, low: number, high: number, dt = 0.1, t0 = 0): TriggerWindow {
  const v: number[] = [];
  for (let p = 0; p < periods; p++) {
    for (let i = 0; i < low; i++) v.push(0);
    for (let i = 0; i < high; i++) v.push(1);
  }
  return chan(v, dt, t0);
}

const baseCfg: TriggerConfig = {
  sourceIndex: 0,
  level: 0.5,
  edge: "rising",
  position: 0.5,
  windowSeconds: 1,
  holdoff: 0,
  hysteresis: 0,
};

// ---------------------------------------------------------------------------
// findTriggerCrossings
// ---------------------------------------------------------------------------

describe("findTriggerCrossings", () => {
  it("finds rising crossings interpolated at the level", () => {
    // 0 -> 1 across one dt; level 0.5 sits at the midpoint => crossing at t=0.05.
    const win = chan([0, 1, 0, 1], 0.1);
    const cs = findTriggerCrossings(win, baseCfg, Number.NEGATIVE_INFINITY);
    expect(cs.length).toBe(2);
    expect(cs[0]).toBeCloseTo(0.05, 6);
    expect(cs[1]).toBeCloseTo(0.25, 6);
  });

  it("finds falling crossings when edge is 'falling'", () => {
    const win = chan([1, 0, 1, 0], 0.1);
    const cs = findTriggerCrossings(win, { ...baseCfg, edge: "falling" }, Number.NEGATIVE_INFINITY);
    expect(cs.length).toBe(2);
    expect(cs[0]).toBeCloseTo(0.05, 6);
  });

  it("does not fire on a signal that starts above the level (must arm first)", () => {
    // Starts at 1 (above), dips to 0.6 (never below level on a rising scan),
    // back to 1 — with no excursion below 0.5 there is no armed rising edge.
    const win = chan([1, 0.6, 1, 0.6, 1], 0.1);
    const cs = findTriggerCrossings(win, baseCfg, Number.NEGATIVE_INFINITY);
    expect(cs.length).toBe(0);
  });

  it("hysteresis rejects ripple that wiggles across the level", () => {
    // Ripple oscillates 0.45/0.55 around level 0.5 — a bare crossing detector
    // would report many edges; a 0.2 V band requires dropping below 0.3 to
    // re-arm, so none of these wiggles count.
    const win = chan([0.45, 0.55, 0.45, 0.55, 0.45, 0.55], 0.1);
    const noHyst = findTriggerCrossings(win, baseCfg, Number.NEGATIVE_INFINITY);
    expect(noHyst.length).toBeGreaterThan(1);
    const withHyst = findTriggerCrossings(
      win,
      { ...baseCfg, hysteresis: 0.2 },
      Number.NEGATIVE_INFINITY,
    );
    expect(withHyst.length).toBe(0);
  });

  it("honours minTime, returning only later crossings", () => {
    const win = chan([0, 1, 0, 1, 0, 1], 0.1); // rising crossings ~0.05, 0.25, 0.45
    const cs = findTriggerCrossings(win, baseCfg, 0.1);
    expect(cs.length).toBe(2);
    expect(cs[0]).toBeCloseTo(0.25, 6);
  });
});

// ---------------------------------------------------------------------------
// extractTriggerFrame
// ---------------------------------------------------------------------------

describe("extractTriggerFrame", () => {
  it("re-zeros time and reports the trigger offset at the pre-trigger span", () => {
    // 21 samples at dt=0.1 spanning t=0..2. Trigger at t=1.0, window 1 s,
    // position 0.5 => frame spans t in [0.5, 1.5], re-zeroed to [0, 1].
    const values = Array.from({ length: 21 }, (_, i) => (i >= 10 ? 1 : 0));
    const win = chan(values, 0.1);
    const frame = extractTriggerFrame(win, 1.0, 1, 0.5, 7)!;
    expect(frame).not.toBeNull();
    expect(frame.t[0]).toBeCloseTo(0, 6);
    expect(frame.t[frame.t.length - 1]).toBeCloseTo(1, 6);
    expect(frame.triggerOffset).toBeCloseTo(0.5, 6);
    expect(frame.triggerTime).toBeCloseTo(1.0, 6);
    expect(frame.seq).toBe(7);
  });

  it("returns null when the pre-trigger window is not buffered", () => {
    // Buffer starts at t=0; trigger at t=0.2 with 0.5 s pre-trigger needs t=-0.3.
    const win = chan(Array.from({ length: 21 }, () => 0), 0.1);
    expect(extractTriggerFrame(win, 0.2, 1, 0.5, 1)).toBeNull();
  });

  it("returns null when the post-trigger window is not buffered", () => {
    // Buffer ends at t=2; trigger at t=1.8 with 0.5 s post-trigger needs t=2.3.
    const win = chan(Array.from({ length: 21 }, () => 0), 0.1);
    expect(extractTriggerFrame(win, 1.8, 1, 0.5, 1)).toBeNull();
  });

  it("position 0 puts the trigger at the left edge", () => {
    const values = Array.from({ length: 21 }, (_, i) => (i >= 5 ? 1 : 0));
    const win = chan(values, 0.1);
    const frame = extractTriggerFrame(win, 0.5, 1, 0, 1)!;
    expect(frame.triggerOffset).toBeCloseTo(0, 6);
    expect(frame.t[0]).toBeCloseTo(0, 6);
  });
});

// ---------------------------------------------------------------------------
// ScopeTrigger coordinator
// ---------------------------------------------------------------------------

describe("ScopeTrigger.evaluate", () => {
  it("produces one frame for a clean periodic signal, centred on a rising edge", () => {
    // 5 periods of a square wave, 10 samples each (5 low / 5 high), dt=0.1 =>
    // period 1 s. Window 1 s, centred. Buffer spans t=0..4.9.
    const win = square(5, 5, 5, 0.1);
    const trig = new ScopeTrigger();
    const frame = trig.evaluate(win, { ...baseCfg, windowSeconds: 1, position: 0.5 });
    expect(frame).not.toBeNull();
    // The source value at the trigger offset should be right at the rising edge.
    const idx = frame!.t.findIndex((tt) => tt >= frame!.triggerOffset);
    expect(frame!.values[0]![idx]).toBeGreaterThanOrEqual(0.5);
  });

  it("does not emit a second frame until a newer trigger completes", () => {
    const win = square(5, 5, 5, 0.1);
    const cfg = { ...baseCfg, windowSeconds: 1, position: 0.5 };
    const trig = new ScopeTrigger();
    const first = trig.evaluate(win, cfg)!;
    expect(first).not.toBeNull();
    // Same buffer, nothing new arrived => no fresh trigger.
    expect(trig.evaluate(win, cfg)).toBeNull();
    expect(trig.frameCount).toBe(1);
  });

  it("emits a fresh frame once a newer complete cycle is buffered", () => {
    const cfg = { ...baseCfg, windowSeconds: 1, position: 0.5 };
    const trig = new ScopeTrigger();
    const win1 = square(4, 5, 5, 0.1); // spans t=0..3.9
    const first = trig.evaluate(win1, cfg)!;
    const win2 = square(6, 5, 5, 0.1); // more cycles, later edges available
    const second = trig.evaluate(win2, cfg);
    expect(second).not.toBeNull();
    expect(second!.triggerTime).toBeGreaterThan(first.triggerTime);
    expect(trig.frameCount).toBe(2);
  });

  it("picks the latest complete cycle (live), not the oldest", () => {
    const cfg = { ...baseCfg, windowSeconds: 1, position: 0.5 };
    const trig = new ScopeTrigger();
    const win = square(6, 5, 5, 0.1); // edges near t=0.5,1.5,2.5,3.5,4.5,5.5; spans 0..5.9
    const frame = trig.evaluate(win, cfg)!;
    // Latest edge with a full post-window (needs t up to trigger+0.5<=5.9).
    // The newest qualifying rising edge is ~5.45, not the first (~0.45).
    expect(frame.triggerTime).toBeGreaterThan(4);
  });

  it("holdoff suppresses triggers that are too close in time", () => {
    const cfg = { ...baseCfg, windowSeconds: 0.5, position: 0.2, holdoff: 5 };
    const trig = new ScopeTrigger();
    const win1 = square(3, 5, 5, 0.1); // period 1 s
    const first = trig.evaluate(win1, cfg)!;
    expect(first).not.toBeNull();
    // Next cycle is only 1 s later (< 5 s holdoff) => suppressed.
    const win2 = square(4, 5, 5, 0.1);
    expect(trig.evaluate(win2, cfg)).toBeNull();
  });

  it("returns null for an out-of-range source index", () => {
    const win = square(5, 5, 5, 0.1);
    const trig = new ScopeTrigger();
    expect(trig.evaluate(win, { ...baseCfg, sourceIndex: 5 })).toBeNull();
  });

  it("reset() forgets trigger history so the next evaluate can re-fire", () => {
    const cfg = { ...baseCfg, windowSeconds: 1, position: 0.5 };
    const win = square(5, 5, 5, 0.1);
    const trig = new ScopeTrigger();
    expect(trig.evaluate(win, cfg)).not.toBeNull();
    expect(trig.evaluate(win, cfg)).toBeNull();
    trig.reset();
    expect(trig.lastTime).toBeNull();
    expect(trig.evaluate(win, cfg)).not.toBeNull();
  });

  it("flat (DC) signal never triggers", () => {
    const win = chan(Array.from({ length: 50 }, () => 0.7), 0.1);
    const trig = new ScopeTrigger();
    expect(trig.evaluate(win, { ...baseCfg, windowSeconds: 1 })).toBeNull();
  });

  it("fires through evaluate() on a falling edge", () => {
    const win = square(5, 5, 5, 0.1);
    const trig = new ScopeTrigger();
    const frame = trig.evaluate(win, { ...baseCfg, edge: "falling", windowSeconds: 1, position: 0.5 });
    expect(frame).not.toBeNull();
    // At the trigger offset the source should be at/below the level (falling).
    const idx = frame!.t.findIndex((tt) => tt >= frame!.triggerOffset);
    expect(frame!.values[0]![idx]).toBeLessThanOrEqual(0.5);
  });

  it("still triggers through evaluate() with a nonzero hysteresis band", () => {
    const win = square(5, 5, 5, 0.1);
    const trig = new ScopeTrigger();
    const frame = trig.evaluate(win, { ...baseCfg, hysteresis: 0.2, windowSeconds: 1, position: 0.5 });
    expect(frame).not.toBeNull();
  });

  it("triggers through evaluate() at position 0 (trigger at left edge)", () => {
    const win = square(5, 5, 5, 0.1);
    const trig = new ScopeTrigger();
    const frame = trig.evaluate(win, { ...baseCfg, windowSeconds: 1, position: 0 });
    expect(frame).not.toBeNull();
    expect(frame!.triggerOffset).toBeCloseTo(0, 6);
  });

  it("triggers a signal that starts above the level (arms after dipping below)", () => {
    // Starts high, dips low, rises again: a rising trigger should arm on the dip
    // and fire on the subsequent crossing, not on the initial high state.
    const v = [1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0];
    const win = chan(v, 0.1);
    const trig = new ScopeTrigger();
    const frame = trig.evaluate(win, { ...baseCfg, windowSeconds: 0.6, position: 0.5 });
    expect(frame).not.toBeNull();
  });

  it("yields exactly one unique frame per new complete cycle as the buffer grows", () => {
    // Simulate the buffer accumulating cycles one period at a time; each
    // evaluate should surface at most one fresh, non-duplicate frame.
    const cfg = { ...baseCfg, windowSeconds: 1, position: 0.5 };
    const trig = new ScopeTrigger();
    const triggerTimes: number[] = [];
    for (let periods = 2; periods <= 10; periods++) {
      const frame = trig.evaluate(square(periods, 5, 5, 0.1), cfg);
      if (frame) triggerTimes.push(frame.triggerTime);
    }
    expect(triggerTimes.length).toBeGreaterThan(2);
    // Strictly increasing ⇒ no duplicates, no going backwards.
    for (let i = 1; i < triggerTimes.length; i++) {
      expect(triggerTimes[i]!).toBeGreaterThan(triggerTimes[i - 1]!);
    }
    expect(new Set(triggerTimes).size).toBe(triggerTimes.length);
  });
});
