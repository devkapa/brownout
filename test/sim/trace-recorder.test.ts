import { describe, expect, it } from "vitest";
import { TraceRecorder } from "../../src/sim/trace-recorder.js";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Record a sequence of (t, values) pairs, one sample per integer step. */
function fill(rec: TraceRecorder, count: number, startT = 0): void {
  for (let i = 0; i < count; i++) {
    const t = startT + i;
    const values = rec.channelKeys.map((_, ci) => (t * 10 + ci) as number);
    rec.record(t, values);
  }
}

// ---------------------------------------------------------------------------
// Bounded ring buffer
// ---------------------------------------------------------------------------

describe("bounded ring buffer", () => {
  it("recording maxPoints+k samples keeps only the newest maxPoints", () => {
    const max = 10;
    const rec = new TraceRecorder({ maxPoints: max });
    rec.setChannels(["a"]);
    fill(rec, max + 5);
    expect(rec.pointCount).toBe(max);
  });

  it("t and values arrays stay aligned after trimming", () => {
    const max = 5;
    const rec = new TraceRecorder({ maxPoints: max });
    rec.setChannels(["x", "y"]);
    for (let i = 0; i < 8; i++) rec.record(i, [i, i * 2]);

    const snap = rec.snapshot();
    expect(snap.t.length).toBe(max);
    expect(snap.channels[0].values.length).toBe(max);
    expect(snap.channels[1].values.length).toBe(max);

    // Newest sample is the last one recorded (t = 7).
    expect(snap.t[snap.t.length - 1]).toBe(7);
    expect(snap.channels[0].values[snap.t.length - 1]).toBe(7);
    expect(snap.channels[1].values[snap.t.length - 1]).toBe(14);

    // Oldest kept sample is t = 8 - max = 3.
    expect(snap.t[0]).toBe(3);
  });
});

// ---------------------------------------------------------------------------
// Backward time jump
// ---------------------------------------------------------------------------

describe("backward time jump", () => {
  it("clears buffers and records the new sample when t jumps back by more than 1e-9", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.record(2.0, [20]);
    // Jump backward by 1 second — should clear and start fresh.
    rec.record(0.5, [99]);

    const snap = rec.snapshot();
    expect(snap.t).toEqual([0.5]);
    expect(snap.channels[0].values).toEqual([99]);
  });

  it("keeps t monotonically increasing after a backward jump", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(5.0, [1]);
    rec.record(1.0, [2]); // backward jump
    rec.record(1.5, [3]); // forward from new start
    rec.record(2.0, [4]);

    const snap = rec.snapshot();
    expect(snap.t).toEqual([1.0, 1.5, 2.0]);
  });

  it("drops backward jitter within 1e-9 instead of appending non-monotonic time", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.record(2.0, [20]);
    // Sub-nanosecond jitter — not a real backward jump.
    rec.record(2.0 - 5e-10, [30]);

    expect(rec.pointCount).toBe(2);
    expect(rec.snapshot().t).toEqual([1, 2]);
  });

  it("drops duplicate timestamps from unrelated store notifications", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1, [10]);
    rec.record(1, [99]);
    expect(rec.snapshot()).toEqual({
      t: [1],
      channels: [{ key: "v", values: [10] }],
    });
  });
});

describe("acquisition generation", () => {
  it("clears retained channels when a warm reload changes acquisition id", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["n1"]);
    expect(rec.startAcquisition(4)).toBe(true);
    rec.record(1, [10]);
    rec.record(2, [20]);

    expect(rec.startAcquisition(4)).toBe(false);
    expect(rec.pointCount).toBe(2);
    expect(rec.startAcquisition(5)).toBe(true);
    expect(rec.pointCount).toBe(0);

    rec.record(3, [99]);
    expect(rec.snapshot()).toEqual({
      t: [3],
      channels: [{ key: "n1", values: [99] }],
    });
  });

  it("can preserve a warm acquisition and let the caller insert a gap", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["n1"]);
    rec.startAcquisition(4);
    rec.record(1, [10]);
    rec.record(2, [20]);

    expect(rec.startAcquisition(5, { preserveHistory: true })).toBe(true);
    expect(rec.pointCount).toBe(2);
    rec.record(3, [Number.NaN]);
    rec.record(4, [40]);

    const snapshot = rec.snapshot();
    expect(snapshot.t).toEqual([1, 2, 3, 4]);
    expect(snapshot.channels[0]!.values.slice(0, 2)).toEqual([10, 20]);
    expect(Number.isNaN(snapshot.channels[0]!.values[2]!)).toBe(true);
    expect(snapshot.channels[0]!.values[3]).toBe(40);
  });
});

// ---------------------------------------------------------------------------
// Mismatched values length
// ---------------------------------------------------------------------------

describe("mismatched values length", () => {
  it("ignores samples whose values length differs from channel count", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["a", "b"]);
    rec.record(1.0, [1, 2]);       // valid: length 2
    rec.record(2.0, [3]);          // invalid: length 1 — should be ignored
    rec.record(3.0, [5, 6, 7]);    // invalid: length 3 — should be ignored
    rec.record(4.0, [7, 8]);       // valid: length 2

    expect(rec.pointCount).toBe(2);
    const snap = rec.snapshot();
    expect(snap.t).toEqual([1.0, 4.0]);
  });
});

// ---------------------------------------------------------------------------
// setChannels
// ---------------------------------------------------------------------------

describe("setChannels", () => {
  it("identical keys (same order, same values) are a no-op — data is preserved", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["a", "b"]);
    rec.record(1.0, [1, 2]);
    rec.record(2.0, [3, 4]);

    rec.setChannels(["a", "b"]); // identical — should preserve
    expect(rec.pointCount).toBe(2);
  });

  it("removing a channel keeps the time axis and the remaining channel's history", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["a", "b"]);
    rec.record(1.0, [10, 20]);
    rec.record(2.0, [11, 21]);

    rec.setChannels(["a"]); // deselect "b"
    expect([...rec.channelKeys]).toEqual(["a"]);
    expect(rec.pointCount).toBe(2);
    const snap = rec.snapshot();
    expect(snap.t).toEqual([1.0, 2.0]);
    expect(snap.channels).toHaveLength(1);
    expect(snap.channels[0].values).toEqual([10, 11]); // "a" history intact
  });

  it("adding a channel preserves existing history and back-fills the new one with NaN", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["a"]);
    rec.record(1.0, [10]);
    rec.record(2.0, [11]);

    rec.setChannels(["a", "b"]); // select "b" — others keep rolling
    expect([...rec.channelKeys]).toEqual(["a", "b"]);
    expect(rec.pointCount).toBe(2);
    const snap = rec.snapshot();
    expect(snap.channels[0].values).toEqual([10, 11]); // "a" preserved
    expect(snap.channels[1].values).toHaveLength(2);
    expect(snap.channels[1].values.every((v) => Number.isNaN(v))).toBe(true); // "b" blank in the past

    // "b" records normally from now on; its past stays NaN.
    rec.record(3.0, [12, 22]);
    const snap2 = rec.snapshot();
    expect(snap2.channels[0].values).toEqual([10, 11, 12]);
    expect(snap2.channels[1].values[2]).toBe(22);
    expect(Number.isNaN(snap2.channels[1].values[0])).toBe(true);
  });

  it("reordering keys preserves each channel's history, remapped by key", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["a", "b"]);
    rec.record(1.0, [10, 20]);
    rec.record(2.0, [11, 21]);

    rec.setChannels(["b", "a"]); // same set, different order
    expect([...rec.channelKeys]).toEqual(["b", "a"]);
    expect(rec.pointCount).toBe(2);
    const snap = rec.snapshot();
    expect(snap.channels[0]).toEqual({ key: "b", values: [20, 21] });
    expect(snap.channels[1]).toEqual({ key: "a", values: [10, 11] });
  });

  it("exposes the new channel keys immediately after setChannels", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["x", "y", "z"]);
    expect([...rec.channelKeys]).toEqual(["x", "y", "z"]);
  });
});

// ---------------------------------------------------------------------------
// clear()
// ---------------------------------------------------------------------------

describe("clear()", () => {
  it("empties the buffer but keeps the channel configuration", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["a", "b"]);
    rec.record(1.0, [1, 2]);
    rec.record(2.0, [3, 4]);

    rec.clear();

    expect(rec.pointCount).toBe(0);
    expect([...rec.channelKeys]).toEqual(["a", "b"]);
  });

  it("allows recording new samples after clear()", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [5]);
    rec.clear();
    rec.record(10.0, [99]);

    const snap = rec.snapshot();
    expect(snap.t).toEqual([10.0]);
    expect(snap.channels[0].values).toEqual([99]);
  });

  it("a backward jump after clear() does not throw (clears an already-empty buffer)", () => {
    // Backward-jump detection derives from the last buffered sample, so after
    // clear() the next record is always accepted — same observable outcome as
    // the pre-extraction closure, where the guard fired on an empty buffer.
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(5.0, [1]);
    rec.clear();
    // t=1.0 is before the last recorded t=5.0, so the guard fires on an empty buffer.
    expect(() => rec.record(1.0, [2])).not.toThrow();
    expect(rec.pointCount).toBe(1);
    expect(rec.snapshot().t).toEqual([1.0]);
  });
});

// ---------------------------------------------------------------------------
// windowFrom()
// ---------------------------------------------------------------------------

describe("windowFrom()", () => {
  it("returns null when fewer than 2 points are buffered", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    expect(rec.windowFrom(0)).toBeNull();

    rec.record(1.0, [10]);
    expect(rec.windowFrom(0)).toBeNull();
  });

  it("returns full range when tStart is before the first sample", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.record(2.0, [20]);
    rec.record(3.0, [30]);

    const win = rec.windowFrom(0.5); // before first sample
    expect(win).not.toBeNull();
    expect(win!.t).toEqual([1.0, 2.0, 3.0]);
    expect(win!.values[0]).toEqual([10, 20, 30]);
  });

  it("binary-search boundary: returns first index with t >= tStart", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    for (let i = 0; i < 10; i++) rec.record(i, [i * 10]);

    // tStart = 3.5 should start at t=4 (first t >= 3.5).
    const win = rec.windowFrom(3.5);
    expect(win).not.toBeNull();
    expect(win!.t[0]).toBe(4);
  });

  it("tStart exactly on a sample boundary includes that sample", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.record(2.0, [20]);
    rec.record(3.0, [30]);

    const win = rec.windowFrom(2.0); // exact match at index 1
    expect(win).not.toBeNull();
    expect(win!.t[0]).toBe(2.0);
    expect(win!.values[0][0]).toBe(20);
  });

  it("tStart after last sample returns null (nothing visible)", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.record(2.0, [20]);

    // tStart beyond the last sample — nothing to render.
    expect(rec.windowFrom(5.0)).toBeNull();
  });

  it("returned slices are independent of the recorder's internal arrays", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.record(2.0, [20]);

    const win = rec.windowFrom(0);
    win!.t.push(999);
    win!.values[0].push(999);

    // Recorder is unaffected.
    expect(rec.pointCount).toBe(2);
  });
});

// ---------------------------------------------------------------------------
// windowRange()  (horizontal pan: bounded on both ends)
// ---------------------------------------------------------------------------

describe("windowRange()", () => {
  function ramp(n: number): TraceRecorder {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    for (let i = 0; i < n; i++) rec.record(i, [i * 10]); // t=0..n-1
    return rec;
  }

  it("returns null when fewer than 2 points are buffered", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    expect(rec.windowRange(0, 10)).toBeNull();
    rec.record(1.0, [10]);
    expect(rec.windowRange(0, 10)).toBeNull();
  });

  it("returns the inclusive [tStart, tStop] slice", () => {
    const rec = ramp(10); // t=0..9
    const win = rec.windowRange(3, 6);
    expect(win).not.toBeNull();
    expect(win!.t).toEqual([3, 4, 5, 6]);
    expect(win!.values[0]).toEqual([30, 40, 50, 60]);
  });

  it("clamps to the buffered ends when the range overhangs", () => {
    const rec = ramp(5); // t=0..4
    const win = rec.windowRange(-100, 100);
    expect(win!.t).toEqual([0, 1, 2, 3, 4]);
  });

  it("does not include samples beyond tStop (upper bound is inclusive)", () => {
    const rec = ramp(10);
    const win = rec.windowRange(2, 4);
    expect(win!.t).toEqual([2, 3, 4]);
    expect(win!.t.at(-1)).toBe(4);
  });

  it("returns null when the range holds fewer than 2 samples", () => {
    const rec = ramp(10);
    expect(rec.windowRange(3.4, 3.6)).toBeNull(); // no sample inside
    expect(rec.windowRange(3, 3)).toBeNull();     // single sample
  });

  it("returns null when tStop < tStart", () => {
    const rec = ramp(10);
    expect(rec.windowRange(6, 3)).toBeNull();
  });

  it("returned slices are independent of the recorder's internal arrays", () => {
    const rec = ramp(5);
    const win = rec.windowRange(0, 4);
    win!.t.push(999);
    win!.values[0].push(999);
    expect(rec.pointCount).toBe(5);
  });
});

// ---------------------------------------------------------------------------
// firstTime()
// ---------------------------------------------------------------------------

describe("firstTime()", () => {
  it("returns null on an empty buffer", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    expect(rec.firstTime()).toBeNull();
  });

  it("returns the oldest buffered timestamp", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.5, [10]);
    rec.record(2.5, [20]);
    expect(rec.firstTime()).toBe(1.5);
  });

  it("tracks the oldest sample after ring-buffer trimming", () => {
    const rec = new TraceRecorder({ maxPoints: 3 });
    rec.setChannels(["v"]);
    for (let i = 0; i < 6; i++) rec.record(i, [i]); // keeps t=3,4,5
    expect(rec.firstTime()).toBe(3);
    expect(rec.lastTime()).toBe(5);
  });
});

// ---------------------------------------------------------------------------
// snapshot() isolation
// ---------------------------------------------------------------------------

describe("snapshot() isolation", () => {
  it("mutating a snapshot t array does not affect the recorder", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.record(2.0, [20]);

    const snap = rec.snapshot();
    snap.t.push(999);
    snap.channels[0].values.push(999);

    expect(rec.pointCount).toBe(2);
  });

  it("recording after snapshot does not mutate the prior snapshot", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.record(2.0, [20]);

    const snap = rec.snapshot();
    rec.record(3.0, [30]);

    expect(snap.t.length).toBe(2);
    expect(snap.channels[0].values.length).toBe(2);
  });

  it("two snapshots at different times are fully independent", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    const snap1 = rec.snapshot();

    rec.record(2.0, [20]);
    const snap2 = rec.snapshot();

    expect(snap1.t.length).toBe(1);
    expect(snap2.t.length).toBe(2);
    snap2.t[0] = -1;
    expect(snap1.t[0]).toBe(1.0); // not affected
  });
});

// ---------------------------------------------------------------------------
// Zero-channel recorder
// ---------------------------------------------------------------------------

describe("zero-channel recorder", () => {
  it("never throws when no channels are set", () => {
    const rec = new TraceRecorder();
    expect(() => {
      rec.record(1.0, []);
      rec.clear();
      rec.windowFrom(0);
      rec.snapshot();
      rec.lastTime();
    }).not.toThrow();
  });

  it("pointCount is 0 and channelKeys is empty by default", () => {
    const rec = new TraceRecorder();
    expect(rec.pointCount).toBe(0);
    expect([...rec.channelKeys]).toEqual([]);
  });

  it("setChannels to empty list then record empty values works", () => {
    const rec = new TraceRecorder();
    rec.setChannels([]);
    rec.record(1.0, []);
    // Values length === keys length (0 === 0), so the sample is accepted.
    expect(rec.pointCount).toBe(1);
  });
});

// ---------------------------------------------------------------------------
// lastTime()
// ---------------------------------------------------------------------------

describe("lastTime()", () => {
  it("returns null when no samples have been recorded", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    expect(rec.lastTime()).toBeNull();
  });

  it("returns the timestamp of the most recently recorded sample", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.record(2.5, [20]);
    expect(rec.lastTime()).toBe(2.5);
  });

  it("returns null after clear()", () => {
    const rec = new TraceRecorder();
    rec.setChannels(["v"]);
    rec.record(1.0, [10]);
    rec.clear();
    expect(rec.lastTime()).toBeNull();
  });
});
