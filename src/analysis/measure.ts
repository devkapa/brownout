/**
 * measureTrace — teaching-grade time-domain estimator.
 *
 * WHY this approach (no FFT/windowing):
 *   This is an interactive oscilloscope for circuit learners, not an
 *   instrument-grade analyser. We want numbers that are immediately
 *   comprehensible and match what a student would compute by hand from the
 *   waveform. Threshold-crossing edge detection gives that, and a DFT/FFT would
 *   add windowing artefacts the user would have to understand — not the right
 *   trade-off here.
 *
 *   The engine does NOT only produce clean sines: capacitor ringing, PWM
 *   ripple, and switching glitches are normal. A bare midpoint crossing is
 *   fragile against these (a lone spike suppresses detection; ripple near the
 *   threshold inflates the edge count and the reported frequency by 2-5x), so
 *   the estimator is hardened three ways below.
 *
 * Edge-detection rules:
 *   - Threshold = midpoint of the 2nd/98th percentiles (not raw min/max), so a
 *     single outlier spike cannot move it outside the real signal.
 *   - Rising crossing with Schmitt hysteresis: after an edge, re-arm only once
 *     the signal falls a band (10% of span) below mid, collapsing ripple/noise
 *     crossings to one edge per cycle.
 *   - Single-sample spikes are rejected (a real edge stays above mid on the
 *     next sample; a glitch falls straight back).
 *   - At least 2 rising edges are required to compute frequency/period; if the
 *     inter-edge intervals are too irregular (no stable period) frequency is
 *     left undefined rather than reporting a confident wrong value.
 *   - Duty cycle: fraction of samples above the midpoint between the first
 *     and last rising-edge timestamps. Using sample fractions (not time-
 *     weighted) is simpler and correct for the uniformly-sampled output the
 *     recorder provides.
 *
 * No React/DOM/Worker imports — importable from any context including tests.
 */

export interface TraceMeasurements {
  /** Minimum sample value. */
  min: number;
  /** Maximum sample value. */
  max: number;
  /** Arithmetic mean. */
  mean: number;
  /** Root mean square. */
  rms: number;
  /** Estimated fundamental frequency in Hz; undefined when not detectable. */
  frequency?: number;
  /** Estimated period in seconds; undefined when not detectable. */
  period?: number;
  /**
   * Fraction of samples above the midpoint (0–1); undefined when not
   * detectable (e.g. dc or fewer than 2 rising edges).
   */
  dutyCycle?: number;
}

/**
 * Maximum coefficient of variation (stddev/mean) of the inter-edge intervals
 * for a period estimate to be trusted.  Clean and even moderately-jittery
 * signals sit well under this; a noise-dominated trace with no real period
 * blows past it, at which point we report no frequency rather than a confident
 * wrong number.
 */
const MAX_PERIOD_COV = 0.5;

/**
 * Linear-interpolated quantile of an ASCENDING-sorted array.
 * Used to derive a spike-resistant threshold band (2nd/98th percentile) so a
 * lone outlier sample cannot push the edge-detection midpoint outside the
 * real signal.
 */
function quantile(sorted: number[], q: number): number {
  const n = sorted.length;
  if (n === 0) return 0;
  if (n === 1) return sorted[0]!;
  const pos = (n - 1) * q;
  const lo = Math.floor(pos);
  const hi = Math.ceil(pos);
  if (lo === hi) return sorted[lo]!;
  return sorted[lo]! + (sorted[hi]! - sorted[lo]!) * (pos - lo);
}

/**
 * Compute time-domain statistics for a single trace window.
 *
 * @param t  Monotonically increasing time values (seconds). Non-finite entries
 *           delimit acquisition gaps.
 * @param v  Corresponding voltage samples, same length as t. Non-finite entries
 *           delimit acquisition gaps rather than being joined across.
 */
export function measureTrace(t: number[], v: number[]): TraceMeasurements {
  const n = v.length;

  // Guard: empty or mismatched arrays should never throw.
  if (n === 0 || t.length !== n) {
    return { min: 0, max: 0, mean: 0, rms: 0 };
  }

  // Scope recorders deliberately insert NaN when acquisition is interrupted.
  // Never remove those markers and join the remaining samples: doing so can
  // turn the time between unrelated edges into a plausible but false period.
  // Amplitude statistics still describe every finite displayed sample, while
  // frequency/period/duty come only from the longest contiguous finite segment
  // (the newest segment wins ties).
  let hasGap = false;
  let segmentStart = -1;
  let longestStart = -1;
  let longestLength = 0;
  const finiteValues: number[] = [];
  for (let i = 0; i <= n; i++) {
    const finite = i < n && Number.isFinite(t[i]) && Number.isFinite(v[i]);
    if (finite) {
      finiteValues.push(v[i]!);
      if (segmentStart < 0) segmentStart = i;
      continue;
    }

    if (i < n) hasGap = true;
    if (segmentStart >= 0) {
      const length = i - segmentStart;
      if (length >= longestLength) {
        longestStart = segmentStart;
        longestLength = length;
      }
      segmentStart = -1;
    }
  }

  if (hasGap) {
    if (finiteValues.length === 0) {
      // An all-gap record contains no amplitude evidence. NaN deliberately
      // carries that absence through arithmetic (including Vpp = max - min)
      // so callers cannot accidentally present a fabricated 0 V reading.
      return {
        min: Number.NaN,
        max: Number.NaN,
        mean: Number.NaN,
        rms: Number.NaN,
      };
    }

    let min = finiteValues[0]!;
    let max = finiteValues[0]!;
    let sum = 0;
    let sumSq = 0;
    for (const sample of finiteValues) {
      if (sample < min) min = sample;
      if (sample > max) max = sample;
      sum += sample;
      sumSq += sample * sample;
    }

    const timing = longestStart >= 0
      ? measureTrace(
          t.slice(longestStart, longestStart + longestLength),
          v.slice(longestStart, longestStart + longestLength),
        )
      : undefined;
    return {
      min,
      max,
      mean: sum / finiteValues.length,
      rms: Math.sqrt(sumSq / finiteValues.length),
      ...(timing?.frequency !== undefined ? { frequency: timing.frequency } : {}),
      ...(timing?.period !== undefined ? { period: timing.period } : {}),
      ...(timing?.dutyCycle !== undefined ? { dutyCycle: timing.dutyCycle } : {}),
    };
  }

  // ── Basic statistics ─────────────────────────────────────────────────────
  let min = v[0]!;
  let max = v[0]!;
  let sum = 0;
  let sumSq = 0;

  for (let i = 0; i < n; i++) {
    const s = v[i]!;
    if (s < min) min = s;
    if (s > max) max = s;
    sum += s;
    sumSq += s * s;
  }

  const mean = sum / n;
  const rms = Math.sqrt(sumSq / n);

  // ── Frequency / period / duty cycle ─────────────────────────────────────
  //
  // Robust thresholding: the engine produces ringing, PWM ripple, and switching
  // glitches, not just clean SPICE sines.  Deriving the threshold from raw
  // min/max is fragile — a single outlier spike pushes the midpoint outside the
  // real signal and suppresses detection entirely.  We bracket the signal with
  // the 2nd/98th percentiles instead, which ignore a few extreme samples while
  // still spanning a narrow-duty pulse (down to ~2% duty).  Reported
  // min/max/mean/rms stay raw.
  const sorted = v.slice().sort((a, b) => a - b);
  const robustMin = quantile(sorted, 0.02);
  const robustMax = quantile(sorted, 0.98);
  const span = robustMax - robustMin;

  // Flat / DC trace (after spike rejection): span too small to identify edges.
  // Threshold tolerates MNA solver float noise while catching a real 1 mV signal.
  if (span < 1e-9) {
    return { min, max, mean, rms };
  }

  const mid = (robustMin + robustMax) / 2;

  // Schmitt-trigger hysteresis: after a rising edge, the detector only re-arms
  // once the signal drops a band below mid.  Without it, ringing/ripple/noise
  // that wiggles across mid registers multiple edges per cycle and the reported
  // frequency scales up with the spurious count (the worst failure for a number
  // a learner trusts).
  const hysteresis = 0.1 * span;

  // Collect rising-edge timestamps with hysteresis, single-sample-spike
  // rejection, and sub-sample interpolation.
  const risingTimes: number[] = [];
  let armed = v[0]! < mid; // do not fire until the signal has been below mid
  for (let i = 1; i < n; i++) {
    const prev = v[i - 1]!;
    const curr = v[i]!;
    if (armed && prev < mid && curr >= mid) {
      // A genuine edge stays above mid on the next sample too; a lone glitch
      // immediately falls back.  Rejecting the unsustained case ignores
      // single-sample spikes without disarming, so the real edge is still
      // caught.
      const sustained = i + 1 >= n || v[i + 1]! >= mid;
      if (sustained) {
        const frac = (mid - prev) / (curr - prev);
        risingTimes.push(t[i - 1]! + frac * (t[i]! - t[i - 1]!));
        armed = false;
      }
    } else if (!armed && curr < mid - hysteresis) {
      armed = true;
    }
  }

  if (risingTimes.length < 2) {
    // Single edge or no edges: cannot estimate period.
    return { min, max, mean, rms };
  }

  // Period = mean interval between consecutive rising edges over all detected
  // edges. Averaging across cycles improves accuracy for short periods.
  const intervals: number[] = [];
  for (let i = 1; i < risingTimes.length; i++) {
    intervals.push(risingTimes[i]! - risingTimes[i - 1]!);
  }
  const period = intervals.reduce((a, b) => a + b, 0) / intervals.length;

  if (!(period > 0)) {
    return { min, max, mean, rms };
  }

  // Stability guard: if the inter-edge intervals vary wildly the trace has no
  // stable period — prefer reporting nothing over a confident wrong number.
  const variance =
    intervals.reduce((acc, x) => acc + (x - period) * (x - period), 0) / intervals.length;
  if (Math.sqrt(variance) / period > MAX_PERIOD_COV) {
    return { min, max, mean, rms };
  }

  const frequency = 1 / period;

  // Duty cycle: fraction of samples above mid, measured between the first and
  // last rising edge so we count only complete cycles.
  const tFirst = risingTimes[0]!;
  const tLast = risingTimes[risingTimes.length - 1]!;
  let aboveMid = 0;
  let counted = 0;
  for (let i = 0; i < n; i++) {
    const ti = t[i]!;
    if (ti >= tFirst && ti <= tLast) {
      counted++;
      if (v[i]! > mid) aboveMid++;
    }
  }
  const dutyCycle = counted > 0 ? aboveMid / counted : undefined;

  return {
    min,
    max,
    mean,
    rms,
    frequency,
    period,
    dutyCycle,
  };
}
