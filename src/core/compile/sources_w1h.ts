/**
 * The W1h signal generator (code 27, W1h item H3a): classic's `signal_gen` compiled to a program record.
 *
 * THE RECORD is classic's own parse and nothing else (core/spec §8.12.1, README Sources). `parseSignalGenParams`
 * runs here, once, so the clamps (`amplitude >= 0`, `frequency >= 1e-6`, `duty` in [0.01, 0.99], `delay >= 0`, the
 * 1e-9 floors, `seed` rounded), the unknown-name fallback to sine and the PWL table (parseFloat pairs, stable sort,
 * first 64 points) match by construction instead of by a second copy. Params are, in `CORE_SIGNAL_GEN_PARAM` order,
 * `[waveform code, amplitude, offset, frequency, phaseDeg, duty, delay, tr, tf, pw, seed, enabled, rSource]`; the
 * blob is `[n, t0, v0, ..., t(n-1), v(n-1)]`, with `n = 0` for every shape but `pwl`.
 *
 * PINS are `pins[0]` (+) and `pins[1]` (-) by position, as classic's stamp reads them (`sources.ts:516-517`); a
 * generator with fewer than two pins declines `malformed:<id>:too-few-pins` like the W1f sources.
 *
 * DECLINES (the circuit then runs on classic):
 * - `non-finite-<param>` for any record value that is not finite. Classic lets a NaN through every clamp
 *   (`Math.max(0, NaN)` is NaN, probe P1a) and plays it, so the core cannot match it; `rSource` is read raw at each
 *   hook, so it is checked here too (a NaN `rSource` takes classic's Norton path and stamps NaN, §8.12.2).
 * - `truncated-pulse` when a `pulse` has `tr + pw + tf > 1/frequency` (Q-O72), summed `(tr + pw) + tf` left to right
 *   as classic's `pulseUnit` does. Classic plays a cycle cut short; the edge model has no cycle to place its edges
 *   in, exactly as for `pulse_gen`.
 * - `level-out-of-range` for an offset beyond 1e100 V, an `offset + 8 * amplitude` beyond it (every shape but dc and
 *   pwl; 8 covers noise's draws of about 6.8 sigma), or a PWL value or time beyond it: no circuit has such a source,
 *   and below it every level, swing and line the core's schedule forms stays finite (the reader repeats it).
 * - `delay-too-late` for a square, triangle, ramp, pulse or noise whose `ulp(delay) > period * 2^-20` (`period` is
 *   `1/frequency`, noise's 1e-4 s): the edge times `(delay - phaseOffset) + n P` are strictly ordered only while the
 *   float spacing at the delay is a small fraction of the period, so `first_event_after` could skip an edge. A 1 kHz
 *   generator may start up to about 4e6 s in. Classic plays it.
 * - `seed-out-of-range` for `|seed| >= 1e21`: the noise hash is over `String(seed)`, which JS prints in exponent
 *   form from 1e21 on, a spelling the core does not reproduce. Below it the decimal digits agree.
 *
 * `enabled` and `rSource` only choose the form (disabled, ideal, Norton); the record carries both, and the form is
 * decided in Rust (H3b). The kind stays out of `CORE_IMPLEMENTED_KINDS` until that lands, so the default gate still
 * answers `unsupported-kind:signal_gen` and this compiler runs only for a caller that widens the set.
 */

import { parseSignalGenParams, signalGenEnabled } from "../../sim/engine/waveform.js";
import { CORE_KIND_CODES, CORE_SIGNAL_GEN_PARAM, CORE_SIGNAL_GEN_SHAPE } from "../kinds.js";
import type { LiveParamSpec } from "./passives.js";
import type { KindCompileContext, SimComponent } from "./seam.js";

/** Kinds this module compiles (signal_gen). */
export const SIGNAL_GEN_KINDS: ReadonlySet<string> = new Set(["signal_gen"]);

const PARAM_NAMES = Object.keys(CORE_SIGNAL_GEN_PARAM) as (keyof typeof CORE_SIGNAL_GEN_PARAM)[];

/** The first `|seed|` JS prints in exponent form. */
const SEED_DECIMAL_LIMIT = 1e21;
/** Volts (and PWL seconds) a record may carry; core/src/waveform.rs `LEVEL_LIMIT`. */
const LEVEL_LIMIT = 1e100;
/** core/src/waveform.rs `DELAY_ULP_PER_PERIOD`: the schedule's times stay ordered while ulp(delay) <= period * 2^-20. */
const DELAY_ULP_PER_PERIOD = 1 / 1_048_576;

const f64Bits = new DataView(new ArrayBuffer(8));
/** The spacing of doubles at `x >= 0`, exact. */
function ulpOf(x: number): number {
  f64Bits.setFloat64(0, x);
  f64Bits.setBigUint64(0, f64Bits.getBigUint64(0) + 1n);
  return f64Bits.getFloat64(0) - x;
}

export type SignalGenRecord =
  | { ok: true; params: number[]; blob: number[] }
  | { ok: false; reason: string };

/**
 * The params and blob of one generator from its authored `params`, or the decline reason. Pure: no circuit, so a
 * test or a fixture generator can build the same record the compiler does.
 */
export function encodeSignalGen(componentId: string, raw: Record<string, number | string>): SignalGenRecord {
  const parsed = parseSignalGenParams(raw);
  const params: number[] = new Array<number>(PARAM_NAMES.length).fill(0);
  params[CORE_SIGNAL_GEN_PARAM.waveform] = CORE_SIGNAL_GEN_SHAPE[parsed.waveform];
  params[CORE_SIGNAL_GEN_PARAM.amplitude] = parsed.amplitude;
  params[CORE_SIGNAL_GEN_PARAM.offset] = parsed.offset;
  params[CORE_SIGNAL_GEN_PARAM.frequency] = parsed.frequency;
  params[CORE_SIGNAL_GEN_PARAM.phaseDeg] = parsed.phaseDeg;
  params[CORE_SIGNAL_GEN_PARAM.duty] = parsed.duty;
  params[CORE_SIGNAL_GEN_PARAM.delay] = parsed.delay;
  params[CORE_SIGNAL_GEN_PARAM.tr] = parsed.tr;
  params[CORE_SIGNAL_GEN_PARAM.tf] = parsed.tf;
  params[CORE_SIGNAL_GEN_PARAM.pw] = parsed.pw;
  params[CORE_SIGNAL_GEN_PARAM.seed] = parsed.seed;
  params[CORE_SIGNAL_GEN_PARAM.enabled] = signalGenEnabled(raw) ? 1 : 0;
  params[CORE_SIGNAL_GEN_PARAM.rSource] = Number(raw.rSource ?? 50);

  for (const name of PARAM_NAMES) {
    if (!Number.isFinite(params[CORE_SIGNAL_GEN_PARAM[name]]!)) {
      return { ok: false, reason: `malformed:${componentId}:non-finite-${name}` };
    }
  }
  if (Math.abs(parsed.seed) >= SEED_DECIMAL_LIMIT) {
    return { ok: false, reason: `malformed:${componentId}:seed-out-of-range` };
  }
  const wave = parsed.waveform;
  if (
    Math.abs(parsed.offset) > LEVEL_LIMIT ||
    (wave !== "dc" && wave !== "pwl" && Math.abs(parsed.offset) + 8 * parsed.amplitude > LEVEL_LIMIT) ||
    parsed.pwlPoints.some((point) => Math.abs(point.t) > LEVEL_LIMIT || Math.abs(point.v) > LEVEL_LIMIT)
  ) {
    return { ok: false, reason: `malformed:${componentId}:level-out-of-range` };
  }
  const latticePeriod =
    wave === "square" || wave === "triangle" || wave === "ramp" || wave === "pulse"
      ? 1 / parsed.frequency
      : wave === "noise"
        ? 1 / 10_000
        : undefined;
  if (latticePeriod !== undefined && ulpOf(parsed.delay) > latticePeriod * DELAY_ULP_PER_PERIOD) {
    return { ok: false, reason: `malformed:${componentId}:delay-too-late` };
  }
  if (parsed.waveform === "pulse") {
    // `1 / frequency` is the period classic divides by (`pulseUnit`, waveform.ts:253); `(tr + pw) + tf` is its
    // `tr + pw + tf`. Written out so the association is the code's: for tr 0.1, pw 0.2, tf 0.3 the other grouping
    // is one ulp smaller.
    const period = 1 / parsed.frequency;
    const fall = parsed.tr + parsed.pw;
    if (period < fall + parsed.tf) return { ok: false, reason: `malformed:${componentId}:truncated-pulse` };
  }

  const blob: number[] = [0];
  if (parsed.waveform === "pwl") {
    blob[0] = parsed.pwlPoints.length;
    for (const point of parsed.pwlPoints) blob.push(point.t, point.v);
  }
  return { ok: true, params, blob };
}

export function compileSignalGenKind(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  if (component.pins.length < 2) {
    ctx.reasons.add(`malformed:${component.id}:too-few-pins`);
    return;
  }
  const record = encodeSignalGen(component.id, component.params);
  if (!record.ok) {
    ctx.reasons.add(record.reason);
    return;
  }
  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES.signal_gen,
    componentIndex,
    pins: [
      ctx.pin(component.kind, component.id, component.pins[0]!.id),
      ctx.pin(component.kind, component.id, component.pins[1]!.id),
    ],
    params: record.params,
    blob: record.blob,
  });
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). None until H3b decides. */
export const SIGNAL_GEN_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
