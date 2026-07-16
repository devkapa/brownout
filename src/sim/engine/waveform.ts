/**
 * waveform.ts — pure, dependency-free waveform evaluation for signal_gen.
 *
 * Why dependency-free: this module is imported by the MNA engine step loop,
 * which must remain allocation-light and has no access to higher-level modules.
 * Keeping it self-contained also makes it trivially testable in isolation.
 *
 * Unified model: V(t) = offset + amplitude * unit(t)
 * where unit(t) is normally in [-1, 1] (except dc which fixes unit = 0,
 * Gaussian noise which is intentionally unbounded, and pwl which bypasses the
 * model entirely — see the waveform-specific notes below).
 */

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/** A single point in a PWL waveform, pre-parsed from the "t:v,..." string. */
export interface PwlPoint {
  t: number;
  v: number;
}

/**
 * Validated, clamped, pre-parsed view of signal_gen component params.
 * parseSignalGenParams() produces this ONCE per load; signalGenVoltage() reads it
 * many thousands of times per second — keep it a plain object.
 */
export interface SignalGenParams {
  waveform: "dc" | "sine" | "square" | "pulse" | "triangle" | "ramp" | "pwl" | "noise";
  amplitude: number;  // volts, >= 0; for noise this is the 1-sigma RMS voltage
  offset: number;     // volts
  frequency: number;  // Hz, >= 1e-6
  phaseDeg: number;   // degrees
  duty: number;       // 0.01..0.99, square only
  delay: number;      // seconds, >= 0 — waveform starts from its own t=0 after delay elapses
  tr: number;         // rise time, seconds, >= 1e-9, pulse only
  tf: number;         // fall time, seconds, >= 1e-9, pulse only
  pw: number;         // pulse width, seconds, >= 1e-9, pulse only
  pwlPoints: PwlPoint[];  // pre-parsed, sorted by t; max 64 points
  seed: number;       // integer, noise only
}

// ---------------------------------------------------------------------------
// FNV-1a 32-bit hash. Kept local so this allocation-light waveform module stays
// dependency-free; a finalizer below improves adjacent-key distribution before
// the values enter the Box-Muller transform.
// ---------------------------------------------------------------------------
function hash32(input: string): number {
  let h = 0x811c9dc5;
  for (let i = 0; i < input.length; i++) {
    h ^= input.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h >>> 0;
}

/**
 * Convert a string hash to a well-mixed open-interval uniform variate.
 *
 * FNV-1a is excellent for stable keys but adjacent string keys retain some bit
 * correlation. The integer finalizer avalanches those bits before Box-Muller.
 * Adding 0.5 maps to (0, 1), avoiding log(0) without endpoint branches.
 */
function hashToOpenUnit(input: string): number {
  let h = hash32(input);
  h ^= h >>> 16;
  h = Math.imul(h, 0x7feb352d);
  h ^= h >>> 15;
  h = Math.imul(h, 0x846ca68b);
  h ^= h >>> 16;
  return ((h >>> 0) + 0.5) / 0x100000000;
}

/** Independent Gaussian draws are updated at 10 kSa/s and held between updates. */
export const SIGNAL_GEN_NOISE_UPDATE_RATE_HZ = 10_000;

/** One-sided equivalent noise bandwidth of the 10 kSa/s zero-order hold. */
export const SIGNAL_GEN_NOISE_EQUIVALENT_BANDWIDTH_HZ = SIGNAL_GEN_NOISE_UPDATE_RATE_HZ / 2;

// ---------------------------------------------------------------------------
// Param parsing
// ---------------------------------------------------------------------------

const VALID_WAVEFORMS = new Set(["dc", "sine", "square", "pulse", "triangle", "ramp", "pwl", "noise"]);

function parsePwl(raw: string): PwlPoint[] {
  if (!raw || typeof raw !== "string") return [];
  const pairs = raw.split(",");
  const points: PwlPoint[] = [];
  for (const pair of pairs) {
    const trimmed = pair.trim();
    const colonIdx = trimmed.indexOf(":");
    if (colonIdx < 0) continue;
    const t = parseFloat(trimmed.slice(0, colonIdx));
    const v = parseFloat(trimmed.slice(colonIdx + 1));
    if (!isFinite(t) || !isFinite(v)) continue;
    points.push({ t, v });
  }
  // Sort ascending by t so interpolation is O(n) linear scan.
  points.sort((a, b) => a.t - b.t);
  // Cap at 64 points to bound interpolation cost.
  return points.slice(0, 64);
}

/**
 * Parse and validate raw params from the component catalog into a typed,
 * clamped SignalGenParams.  Designed to be called once per engine load, not
 * per-step — but kept allocation-light for cases where the engine calls it
 * more frequently during adaptive-step retry.
 */
export function parseSignalGenParams(params: Record<string, number | string>): SignalGenParams {
  const waveformRaw = typeof params.waveform === "string" ? params.waveform : "sine";
  const waveform = VALID_WAVEFORMS.has(waveformRaw)
    ? (waveformRaw as SignalGenParams["waveform"])
    : "sine";  // unknown values fall back to sine

  const amplitude = Math.max(0, Number(params.amplitude ?? 2.5));
  const offset = Number(params.offset ?? 2.5);
  const frequency = Math.max(1e-6, Number(params.frequency ?? 1000));
  const phaseDeg = Number(params.phaseDeg ?? 0);
  const duty = Math.min(0.99, Math.max(0.01, Number(params.duty ?? 0.5)));
  const delay = Math.max(0, Number(params.delay ?? 0));
  const tr = Math.max(1e-9, Number(params.tr ?? 1e-6));
  const tf = Math.max(1e-9, Number(params.tf ?? 1e-6));
  const pw = Math.max(1e-9, Number(params.pw ?? 5e-4));
  const pwlPoints = parsePwl(typeof params.pwl === "string" ? params.pwl : "");
  const seed = Math.round(Number(params.seed ?? 1));

  return { waveform, amplitude, offset, frequency, phaseDeg, duty, delay, tr, tf, pw, pwlPoints, seed };
}

/**
 * Returns whether the signal_gen output is enabled (connected).
 * When `enabled` is 0 the output is Hi-Z (disconnected).
 * Default is enabled (any value other than 0).
 */
export function signalGenEnabled(params: Record<string, number | string>): boolean {
  return Number(params.enabled ?? 1) !== 0;
}

// ---------------------------------------------------------------------------
// Waveform evaluation
// ---------------------------------------------------------------------------

/**
 * Compute the output voltage for a signal_gen component at simulation time t.
 *
 * For most waveforms: V = offset + amplitude * unit(t')
 * where t' = t - delay (the waveform's own time after the delay elapses).
 *
 * Special cases:
 *   dc    — unit = 0, so V = offset.  amplitude is intentionally ignored
 *           because a DC source is fully described by its offset; making
 *           amplitude contribute would surprise users who set it for a
 *           waveform and then switch to dc.
 *   pwl   — bypasses the unified model entirely.  PWL points specify
 *           absolute voltages chosen by the user; wrapping them in
 *           offset + amplitude * v would double-transform the signal and
 *           produce wrong values.  Offset and amplitude are ignored.
 *   pulse — phase is NOT applied (SPICE convention: pulse is defined by
 *           absolute time parameters td/tr/pw/tf/period, not a phase shift;
 *           applying phase would change the initial delay semantics that
 *           SPICE users expect).
 */
export function signalGenVoltage(p: SignalGenParams, t: number): number {
  // Before the delay expires the waveform holds its own t'=0 value.
  // For all periodic waveforms we evaluate at t'=0 by passing tEff=0,
  // which gives the correct "hold" output without special-casing every branch.
  const tEff = t < p.delay ? 0 : t - p.delay;

  switch (p.waveform) {
    case "dc":
      // offset only; amplitude is intentionally unused (see JSDoc above)
      return p.offset;

    case "sine":
      return p.offset + p.amplitude * sineUnit(tEff, p.frequency, p.phaseDeg);

    case "square":
      return p.offset + p.amplitude * squareUnit(tEff, p.frequency, p.phaseDeg, p.duty);

    case "triangle":
      return p.offset + p.amplitude * triangleUnit(tEff, p.frequency, p.phaseDeg);

    case "ramp":
      return p.offset + p.amplitude * rampUnit(tEff, p.frequency, p.phaseDeg);

    case "pulse":
      // delay is already factored into the engine-level td; tEff is used
      // directly as the pulse's own time axis (phase not applied — see JSDoc)
      return p.offset + p.amplitude * pulseUnit(tEff, p.frequency, p.tr, p.tf, p.pw);

    case "pwl":
      return pwlVoltage(p, t);  // absolute voltages, see JSDoc

    case "noise":
      return p.offset + p.amplitude * noiseUnit(p.seed, tEff);
  }
}

// ---------------------------------------------------------------------------
// Unit waveform helpers (return values in [-1, 1])
// ---------------------------------------------------------------------------

function sineUnit(t: number, f: number, phaseDeg: number): number {
  const phaseRad = (phaseDeg * Math.PI) / 180;
  return Math.sin(2 * Math.PI * f * t + phaseRad);
}

function squareUnit(t: number, f: number, phaseDeg: number, duty: number): number {
  const period = 1 / f;
  const phaseOffset = ((phaseDeg / 360) * period + period) % period;
  // Shift t by the phase offset so phase=90 advances the waveform by 1/4 period.
  const tShifted = ((t + phaseOffset) % period + period) % period;
  return tShifted < duty * period ? 1 : -1;
}

function triangleUnit(t: number, f: number, phaseDeg: number): number {
  const period = 1 / f;
  const phaseOffset = ((phaseDeg / 360) * period + period) % period;
  // Normalise into [0, 1) within the period.
  const pos = ((t + phaseOffset) % period + period) % period / period;
  // Rising half [0, 0.5): -1 -> +1; falling half [0.5, 1): +1 -> -1.
  // At pos=0 unit=-1, at pos=0.5 unit=+1, at pos=1 (=0 next cycle) unit=-1.
  if (pos < 0.5) {
    return -1 + 4 * pos;
  } else {
    return 3 - 4 * pos;
  }
}

function rampUnit(t: number, f: number, phaseDeg: number): number {
  const period = 1 / f;
  const phaseOffset = ((phaseDeg / 360) * period + period) % period;
  // Rising sawtooth: linearly ramps from -1 to +1 over each period, instant fall.
  const pos = ((t + phaseOffset) % period + period) % period / period;
  return -1 + 2 * pos;
}

/**
 * SPICE-style trapezoid pulse unit waveform.
 * base = -1, peak = +1 per period (= 1/f):
 *   [0, tr)            — rise from -1 to +1
 *   [tr, tr+pw)        — hold at +1
 *   [tr+pw, tr+pw+tf)  — fall from +1 to -1
 *   [tr+pw+tf, period) — hold at -1
 *
 * Phase is NOT applied here; the `delay` parameter is handled at the call
 * site via tEff = t - delay (SPICE convention — see signalGenVoltage JSDoc).
 */
function pulseUnit(t: number, f: number, tr: number, tf: number, pw: number): number {
  const period = 1 / f;
  // Wrap t into the current period.
  const tMod = ((t % period) + period) % period;

  if (tMod < tr) {
    // Rising edge: interpolate from -1 to +1
    return -1 + 2 * (tMod / tr);
  } else if (tMod < tr + pw) {
    return 1;
  } else if (tMod < tr + pw + tf) {
    // Falling edge: interpolate from +1 to -1
    return 1 - 2 * ((tMod - tr - pw) / tf);
  } else {
    return -1;
  }
}

/**
 * PWL: piecewise-linear absolute voltages.
 * Bypasses the unified model (offset/amplitude ignored) — see signalGenVoltage JSDoc.
 * Uses the real simulation time t (not tEff) because the delay behaviour for
 * PWL is implicit in the point timestamps the user supplies.
 */
function pwlVoltage(p: SignalGenParams, t: number): number {
  const pts = p.pwlPoints;
  // Empty or unparseable table falls back to a sane DC value.
  if (pts.length === 0) return p.offset;
  // Hold the first value before the first point.
  if (t <= pts[0].t) return pts[0].v;
  // Hold the last value after the last point.
  if (t >= pts[pts.length - 1].t) return pts[pts.length - 1].v;
  // Linear interpolation between bracketing points.
  for (let i = 1; i < pts.length; i++) {
    if (t <= pts[i].t) {
      const a = pts[i - 1];
      const b = pts[i];
      const frac = (t - a.t) / (b.t - a.t);
      return a.v + frac * (b.v - a.v);
    }
  }
  // Should be unreachable given the guards above.
  return pts[pts.length - 1].v;
}

/**
 * Noise: zero-mean Gaussian with unit standard deviation, updated at 10 kSa/s
 * and held piecewise-constant for each 0.1 ms bucket. `amplitude` therefore
 * means 1-sigma RMS volts. The zero-order hold has a one-sided equivalent
 * noise bandwidth of 5 kHz.
 *
 * Deterministic by design: the adaptive-step solver may evaluate the same
 * time point multiple times during step-size negotiation.  A non-deterministic
 * noise source would produce different values on retry, breaking convergence.
 * We achieve determinism by hashing (seed, bucketIndex, variate) and applying
 * a Box-Muller transform — the same inputs always produce the same output
 * regardless of evaluation order. Gaussian noise is intentionally unbounded;
 * clipping it to offset +/- amplitude would turn it into a different
 * distribution and make `amplitude` cease to be an RMS quantity.
 */
function noiseUnit(seed: number, t: number): number {
  const BUCKET_WIDTH = 1 / SIGNAL_GEN_NOISE_UPDATE_RATE_HZ;
  const bucket = Math.floor((t + 1e-12) / BUCKET_WIDTH);
  const u1 = hashToOpenUnit(`${String(seed)}:${String(bucket)}:radius`);
  const u2 = hashToOpenUnit(`${String(seed)}:${String(bucket)}:angle`);
  return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}
