/**
 * TraceRecorder — per-channel ring buffer for oscilloscope capture.
 *
 * Responsibility split: this module owns capture (time-series storage,
 * monotonic-time enforcement, ring-buffer trimming) so the UI component
 * owns only presentation (uPlot, RAF loop, React state). Stage 5 Wave A.
 *
 * Design constraint: zero React/zustand/DOM imports. This file must remain
 * importable from workers, tests, and server-side code without side effects.
 *
 * No React state is mutated per tick. The subscription closure calls
 * record() directly into these plain arrays; the RAF loop reads them
 * without going through React at all.
 */

export interface TraceSnapshot {
  /** Monotonically increasing time values, one entry per sample. */
  t: number[];
  /**
   * One entry per channel, in the same order as the keys passed to
   * setChannels(). Each inner array is a deep copy independent of the
   * recorder's live buffers.
   */
  channels: Array<{ key: string; values: number[] }>;
}

export class TraceRecorder {
  private readonly maxPoints: number;
  private keys: readonly string[] = [];
  private acquisitionId: number | null = null;
  private _revision = 0;
  /** Parallel arrays: t[i] is the time of sample i, ch[k][i] its value. */
  private t: number[] = [];
  private ch: number[][] = [];

  constructor(options?: { maxPoints?: number }) {
    this.maxPoints = options?.maxPoints ?? 2000;
  }

  /**
   * Replace the active channel set, reconciling columns in place so that
   * selecting or deselecting one channel does NOT throw away the others'
   * history. Shared channels keep their full buffer (and keep rolling), removed
   * channels are dropped, and a newly-added channel is back-filled with NaN for
   * the already-buffered samples so it simply starts drawing from "now" (uPlot
   * renders NaN as a gap). Identical key lists (same order) are a no-op so
   * in-flight data is preserved; callers rely on that contract in tests.
   *
   * Full buffer resets happen at an explicit worker acquisition-generation
   * boundary. record()'s backward-time guard remains as a compatibility
   * fallback for callers that do not have generation metadata.
   */
  setChannels(keys: readonly string[]): void {
    if (keys.length === this.keys.length && keys.every((k, i) => k === this.keys[i])) {
      // Identical channel set and order — nothing to do.
      return;
    }
    const len = this.t.length;
    const prev = new Map<string, number[]>();
    this.keys.forEach((k, i) => prev.set(k, this.ch[i]));
    // Keep each retained channel's existing array (preserving history); give a
    // brand-new channel a NaN column the same length as the time axis.
    this.ch = keys.map((k) => prev.get(k) ?? new Array<number>(len).fill(NaN));
    this.keys = keys;
    this._revision += 1;
  }

  get channelKeys(): readonly string[] {
    return this.keys;
  }

  /**
   * Mark the worker acquisition generation that subsequent samples belong to.
   * A changed generation clears every retained channel by default. A display
   * that can represent discontinuities may preserve history and insert a gap
   * across a warm reload. Returns true when the generation changed so stateful
   * front ends can reset their own history either way.
   */
  startAcquisition(
    acquisitionId: number,
    options?: { preserveHistory?: boolean },
  ): boolean {
    if (!Number.isSafeInteger(acquisitionId) || acquisitionId < 0) {
      throw new RangeError("trace acquisition id must be a non-negative safe integer");
    }
    if (this.acquisitionId === acquisitionId) return false;
    this.acquisitionId = acquisitionId;
    if (options?.preserveHistory !== true) this._resetBuffers();
    return true;
  }

  /**
   * Append one sample. Guards:
   *  - mismatched values length → ignored (mirrors today's buf.vs.length !== nets.length check)
   *  - backward time jump by more than 1e-9 → buffers cleared first, then
   *    the sample is recorded (today's reload guard, previously a local var
   *    in the subscription closure; moving it here keeps it working even
   *    when no React effect re-runs on reload, e.g. during hot reload or
   *    fast circuit swaps)
   *  - ring-buffer cap enforced by a single splice (same as today)
   */
  record(t: number, values: readonly number[]): void {
    if (values.length !== this.keys.length || !Number.isFinite(t)) return;

    // Detect non-monotonic time (circuit reload mid-subscription).
    if (this.t.length > 0) {
      const last = this.t[this.t.length - 1];
      if (t < last - 1e-9) {
        this._resetBuffers();
      } else if (t <= last) {
        // Store notifications unrelated to acquisition can repeat a timestamp.
        // Dropping them preserves the strict ordering required by trigger and
        // binary-search code and avoids biasing sample-weighted measurements.
        return;
      }
    }

    this.t.push(t);
    for (let i = 0; i < this.keys.length; i++) {
      this.ch[i].push(values[i]);
    }

    if (this.t.length > this.maxPoints) {
      const excess = this.t.length - this.maxPoints;
      this.t.splice(0, excess);
      for (const ch of this.ch) ch.splice(0, excess);
    }
    this._revision += 1;
  }

  /**
   * Empty all buffers but keep the channel configuration.
   *
   * Backward-jump detection derives from the last buffered sample, so
   * after clear() the next record() is always accepted. The pre-extraction
   * code kept `lastT` in the subscription closure across clears, but the
   * observable outcome was identical: a backward jump after a clear just
   * cleared an already-empty buffer and recorded the sample.
   */
  clear(): void {
    this.t = [];
    for (let i = 0; i < this.ch.length; i++) this.ch[i] = [];
    this._revision += 1;
  }

  /**
   * The timestamp of the most-recently recorded sample, or null when the
   * buffer is empty. Used by the RAF loop to compute tEnd without exposing
   * the internal array.
   */
  lastTime(): number | null {
    return this.t.length > 0 ? this.t[this.t.length - 1] : null;
  }

  /**
   * The timestamp of the OLDEST buffered sample, or null when empty. With
   * lastTime() this bounds the buffered time span — used by the scope's
   * horizontal-pan control to know how far back acquisition memory reaches.
   */
  firstTime(): number | null {
    return this.t.length > 0 ? this.t[0] : null;
  }

  get pointCount(): number {
    return this.t.length;
  }

  /** Monotonic invalidation token for renderers; not tied to React state. */
  get revision(): number {
    return this._revision;
  }

  /**
   * Return aligned time + value slices starting at the first index whose
   * t >= tStart. Uses binary search — O(log n) vs. O(n).
   *
   * Returns null when fewer than 2 points are buffered (matches the RAF
   * loop's `buf.t.length > 1` guard — with only one point uPlot has no
   * segment to draw). tStart before the first sample returns the full
   * buffer; tStart beyond the last sample returns null (nothing in the
   * window — unreachable from the scope's RAF loop, where tStart is
   * always lastTime() minus a positive time window).
   */
  windowFrom(tStart: number): { t: number[]; values: number[][] } | null {
    if (this.t.length < 2) return null;

    // Binary search for the first index where t[mid] >= tStart.
    let lo = 0;
    let hi = this.t.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (this.t[mid] < tStart) lo = mid + 1;
      else hi = mid;
    }

    // Binary search converges to the leftmost index where t[lo] >= tStart.
    // When tStart is beyond the last sample, lo stops at the last index but
    // t[lo] < tStart — nothing in the buffer is within the window.
    if (lo >= this.t.length || this.t[lo] < tStart) return null;

    return {
      t: this.t.slice(lo),
      values: this.ch.map((ch) => ch.slice(lo)),
    };
  }

  /**
   * Return aligned time + value slices for the closed range [tStart, tStop].
   * Like windowFrom() but bounded on BOTH ends — used for horizontal pan, where
   * the visible window is a fixed slice somewhere inside acquisition memory
   * rather than the trailing window. Returns null when the range contains fewer
   * than 2 samples (uPlot has no segment to draw) or when tStop < tStart.
   */
  windowRange(tStart: number, tStop: number): { t: number[]; values: number[][] } | null {
    if (this.t.length < 2 || tStop < tStart) return null;

    // Leftmost index where t >= tStart (lower bound).
    let lo = 0;
    let hi = this.t.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (this.t[mid] < tStart) lo = mid + 1;
      else hi = mid;
    }

    // First index where t > tStop (upper bound) — exclusive end of the slice.
    let rlo = 0;
    let rhi = this.t.length;
    while (rlo < rhi) {
      const mid = (rlo + rhi) >> 1;
      if (this.t[mid] <= tStop) rlo = mid + 1;
      else rhi = mid;
    }
    const end = rlo;

    if (end - lo < 2) return null;

    return {
      t: this.t.slice(lo, end),
      values: this.ch.map((ch) => ch.slice(lo, end)),
    };
  }

  /**
   * Deep copy of the entire buffer. Mutations to the returned snapshot
   * never affect the recorder, and subsequent record() calls never mutate
   * a previously returned snapshot (each snapshot is fully independent).
   */
  snapshot(): TraceSnapshot {
    return {
      t: this.t.slice(),
      channels: this.keys.map((key, i) => ({
        key,
        values: this.ch[i].slice(),
      })),
    };
  }

  // ---------------------------------------------------------------------------
  // Internal helpers
  // ---------------------------------------------------------------------------

  private _resetBuffers(): void {
    this.t = [];
    this.ch = Array.from({ length: this.keys.length }, () => []);
    this._revision += 1;
  }
}
