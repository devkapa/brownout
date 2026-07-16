/**
 * ScopeTrigger — oscilloscope-style triggered acquisition (pure, no React/DOM).
 *
 * WHY this exists
 *   The live scope samples net voltages at the worker tick rate (~60 Hz of
 *   *simulated* time) into a TraceRecorder ring buffer. In "roll" mode the UI
 *   shows the latest window and the trace scrolls left as new samples arrive.
 *   In "trigger" mode we instead find a repeatable event — an edge crossing a
 *   chosen level on a chosen source channel — and extract a fixed window AROUND
 *   that event, re-zeroed so the trigger always sits at the same horizontal
 *   position. Periodic signals then appear stationary; cycle-to-cycle variation
 *   (noise, jitter, modulation) shows up as the trace subtly changing in place
 *   rather than scrolling.
 *
 * Responsibility split (mirrors trace-recorder.ts / measure.ts)
 *   This module owns ONLY the pure math + state machine: edge detection with
 *   Schmitt hysteresis, holdoff, and frame extraction from an aligned
 *   (t, values[][]) window. Policy that needs wall-clock time (auto-trigger
 *   timeout), the persistence ring, and all rendering live in the React
 *   component. Keeping this DOM/React/zustand-free makes it importable from
 *   tests, workers, and server code, and cheap to unit-test exhaustively.
 *
 * Design notes
 *   - Edge detection reuses the same hardening ideas as measure.ts (a real edge
 *     comes from the far side of the level; ripple/noise around the level is
 *     collapsed with a hysteresis band) so trigger behaviour matches the
 *     frequency the measurements strip reports.
 *   - We pick the LATEST extractable trigger newer than the last one, not the
 *     earliest. After an idle gap this jumps straight to the most recent
 *     complete cycle instead of replaying ancient edges, keeping the display
 *     live; for a periodic signal each new cycle still produces a fresh frame
 *     (one per RAF at most), which feeds the persistence ring.
 *   - When a candidate edge is found but its post-trigger window has not been
 *     buffered yet, evaluate() returns null WITHOUT advancing the last-trigger
 *     marker, so the same edge is re-evaluated next frame once enough samples
 *     have accumulated.
 */

export type TriggerEdge = "rising" | "falling";

/**
 * - "auto":   free-run if no trigger occurs within a timeout (component-owned).
 * - "normal": only ever update the frame on a real trigger.
 * - "single": capture one frame on the next trigger, then freeze until re-armed.
 */
export type TriggerMode = "auto" | "normal" | "single";

/** Top-level acquisition mode toggle. */
export type AcquireMode = "roll" | "trigger";

export interface TriggerConfig {
  /** Index into the window's `values[]` array identifying the source channel. */
  sourceIndex: number;
  /** Trigger threshold in volts. */
  level: number;
  /** Which slope qualifies as a trigger. */
  edge: TriggerEdge;
  /**
   * Pre-trigger fraction of the window shown to the left of the trigger,
   * in [0, 1]. 0.5 centres the trigger; 0 puts it at the left edge.
   */
  position: number;
  /** Total displayed span in seconds (reuses the scope's time-window control). */
  windowSeconds: number;
  /**
   * Minimum sim-time between accepted triggers, in seconds. Suppresses
   * re-triggering inside the same cycle (0 disables — hysteresis is the
   * primary noise guard).
   */
  holdoff: number;
  /**
   * Noise-rejection band around the level in volts. The detector must travel
   * a full band past the level on the far side before it will re-arm, so
   * ripple sitting on the threshold cannot register multiple edges per cycle.
   */
  hysteresis: number;
}

/** A single completed triggered acquisition, ready to render. */
export interface TriggerFrame {
  /** Relative time from the window start (seconds), spanning ~[0, windowSeconds]. */
  t: number[];
  /** Per-channel values aligned to `t`, in the recorder's channel order. */
  values: number[][];
  /** Relative time of the trigger instant within the frame (== position*window). */
  triggerOffset: number;
  /** Absolute sim time of the trigger instant (used for holdoff bookkeeping). */
  triggerTime: number;
  /** Monotonic id so the renderer can tell one acquisition from the next. */
  seq: number;
}

/** Aligned buffer slice — the shape TraceRecorder.windowFrom() returns. */
export interface TriggerWindow {
  t: number[];
  values: number[][];
}

/** Linear-interpolated time at which a segment crosses `level`. */
function crossTime(t0: number, t1: number, v0: number, v1: number, level: number): number {
  if (v1 === v0) return t1;
  const frac = (level - v0) / (v1 - v0);
  return t0 + frac * (t1 - t0);
}

/**
 * Collect interpolated crossing times of `level` on the source channel, in
 * ascending order, keeping only crossings strictly later than `minTime`.
 *
 * Arming: a rising trigger only fires after the signal has dropped a full
 * hysteresis band below the level (and symmetrically for falling). Each
 * detected crossing disarms the detector, so it must travel back across the
 * band before the next crossing counts — collapsing ripple/noise that sits on
 * the threshold to one edge per real cycle.
 */
export function findTriggerCrossings(
  win: TriggerWindow,
  cfg: Pick<TriggerConfig, "sourceIndex" | "level" | "edge" | "hysteresis">,
  minTime: number,
): number[] {
  const src = win.values[cfg.sourceIndex];
  const out: number[] = [];
  if (!src || src.length < 2 || win.t.length !== src.length) return out;

  const { level, edge } = cfg;
  const hysteresis = Math.max(0, cfg.hysteresis);
  const rising = edge === "rising";
  const armBelow = level - hysteresis; // rising: must dip below this to arm
  const armAbove = level + hysteresis; // falling: must rise above this to arm

  let armed = false;
  for (let i = 0; i < src.length; i++) {
    const v = src[i]!;
    if (!armed) {
      if (rising ? v < armBelow : v > armAbove) armed = true;
      continue;
    }
    if (i === 0) continue;
    const prev = src[i - 1]!;
    const crossed = rising ? prev < level && v >= level : prev > level && v <= level;
    if (crossed) {
      const tc = crossTime(win.t[i - 1]!, win.t[i]!, prev, v, level);
      if (tc > minTime) out.push(tc);
      armed = false; // require travelling back across the band before re-firing
    }
  }
  return out;
}

/**
 * Slice an aligned window into a trigger frame centred (per `position`) on
 * `triggerTime`, re-zeroed so the window starts at relative time 0.
 *
 * Returns null when the buffer does not fully cover the requested
 * [triggerTime - pre, triggerTime + post] span (too early in the run, or the
 * post-trigger tail has not been sampled yet).
 */
export function extractTriggerFrame(
  win: TriggerWindow,
  triggerTime: number,
  windowSeconds: number,
  position: number,
  seq: number,
): TriggerFrame | null {
  const t = win.t;
  const n = t.length;
  if (n < 2) return null;

  const pos = Math.min(1, Math.max(0, position));
  const preSpan = pos * windowSeconds;
  const postSpan = windowSeconds - preSpan;
  const tStart = triggerTime - preSpan;
  const tStop = triggerTime + postSpan;

  // Require the buffer to fully bracket the window; otherwise the frame would
  // be clipped and the trace would visibly grow as samples arrive.
  if (t[0]! > tStart || t[n - 1]! < tStop) return null;

  // First index with t >= tStart (binary search; buffer is sorted ascending).
  let lo = 0;
  let hi = n - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (t[mid]! < tStart) lo = mid + 1;
    else hi = mid;
  }
  const startIdx = lo;

  const outT: number[] = [];
  const indices: number[] = [];
  for (let i = startIdx; i < n; i++) {
    const ti = t[i]!;
    if (ti > tStop) break;
    outT.push(ti - tStart);
    indices.push(i);
  }
  if (outT.length < 2) return null;

  const values = win.values.map((ch) => indices.map((i) => ch[i]!));

  return {
    t: outT,
    values,
    triggerOffset: preSpan,
    triggerTime,
    seq,
  };
}

/**
 * Stateful coordinator: remembers the last accepted trigger so each evaluate()
 * yields at most one fresh frame, and applies holdoff. Construct one per scope
 * instance; reset() on circuit reload / channel change / manual clear.
 */
export class ScopeTrigger {
  private lastTriggerTime = Number.NEGATIVE_INFINITY;
  private seq = 0;

  /** Forget all trigger history (sim reload, channel change, manual clear). */
  reset(): void {
    this.lastTriggerTime = Number.NEGATIVE_INFINITY;
    this.seq = 0;
  }

  /** Sim time of the most recently accepted trigger, or null if none yet. */
  get lastTime(): number | null {
    return this.lastTriggerTime === Number.NEGATIVE_INFINITY ? null : this.lastTriggerTime;
  }

  /** Number of frames produced since the last reset(). */
  get frameCount(): number {
    return this.seq;
  }

  /**
   * Look for the newest acceptable trigger in `win` whose full pre/post window
   * is buffered, and return its frame. Returns null when no fresh, complete
   * trigger is available — in which case the caller keeps showing the previous
   * frame (normal/single) or free-runs after a timeout (auto).
   *
   * The last-trigger marker only advances on a returned frame, so a candidate
   * whose post-window is still filling is re-tried on subsequent calls.
   */
  evaluate(win: TriggerWindow, cfg: TriggerConfig): TriggerFrame | null {
    if (cfg.sourceIndex < 0 || cfg.sourceIndex >= win.values.length) return null;

    const minTime =
      this.lastTriggerTime === Number.NEGATIVE_INFINITY
        ? Number.NEGATIVE_INFINITY
        : this.lastTriggerTime + Math.max(0, cfg.holdoff);

    const crossings = findTriggerCrossings(win, cfg, minTime);
    if (crossings.length === 0) return null;

    const pos = Math.min(1, Math.max(0, cfg.position));
    const preSpan = pos * cfg.windowSeconds;
    const postSpan = cfg.windowSeconds - preSpan;
    const tFirst = win.t[0]!;
    const tLast = win.t[win.t.length - 1]!;

    // Newest-first: prefer the most recent complete cycle so the display stays
    // live. Skip edges whose post-tail is still filling; stop once an edge is
    // too old to have its pre-window (older edges only get worse).
    for (let k = crossings.length - 1; k >= 0; k--) {
      const et = crossings[k]!;
      if (et + postSpan > tLast) continue; // post-window not buffered yet
      if (et - preSpan < tFirst) break; // pre-window scrolled off; older edges worse
      const frame = extractTriggerFrame(win, et, cfg.windowSeconds, pos, this.seq + 1);
      if (frame) {
        this.seq += 1;
        this.lastTriggerTime = et;
        return frame;
      }
    }
    return null;
  }
}
