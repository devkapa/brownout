/**
 * AC frequency-response sweep runner.
 *
 * IMPORTANT — bespoke sine-sweep, NOT SPICE AC analysis:
 *   This module drives the circuit with a real sine waveform at each frequency
 *   and extracts magnitude/phase via a single-bin DFT.  It does NOT compute a
 *   small-signal transfer matrix, does NOT linearise the circuit, and does NOT
 *   run an eigenvalue decomposition.  Results reflect the actual nonlinear
 *   behaviour of the circuit at the user's configured drive amplitude —
 *   teaching-grade accuracy appropriate for RC filters and resistive dividers,
 *   not SPICE-grade accuracy for BJT amplifiers at small signal.
 *
 * WHY cold-start per point (same rationale as run-sweep.ts):
 *   Capacitor and inductor state would carry from one frequency to the next,
 *   introducing spurious transients at each new drive frequency.  coldLoad()
 *   zeroes all element state so every point starts from a known initial
 *   condition.
 *
 * WHY adaptive step h = min(ANALYSIS_STEP_S, 1/(f × SAMPLES_PER_CYCLE)):
 *   The fixed analysis step (5e-5 s = 20 kHz) would produce fewer than 1
 *   sample/cycle above 20 kHz and ~20 samples/cycle at 1 kHz.  40
 *   samples/cycle keeps the single-bin DFT error well under 1 dB across the
 *   full sweep range; below ~500 Hz the fixed step already provides >40
 *   samples/cycle, so no additional refinement is needed.
 *
 * WHY SETTLE_CYCLES = 5 before measuring:
 *   An RC circuit's transient decays as e^(-t/τ).  After 5 cycles at the
 *   drive frequency, t = 5/f.  For the worst-case RC low-pass (fc = f, so
 *   τ = 1/(2π·f)), e^(-5/(2π·τ · τ^{-1})) = e^(-5) ≈ 0.007 — the
 *   transient is <1% of the drive amplitude.
 *
 *   IMPORTANT — this derivation only holds for a FIRST-ORDER circuit.  A sharp
 *   resonant (high-Q) circuit builds up over an envelope time constant
 *   τ = 2Q/ω₀, so 5 drive cycles at resonance leave the build-up largely
 *   undecayed and the measured peak reads several dB low.  We cannot afford to
 *   settle 2Q cycles for every point (Q can be 50+), so instead we DETECT the
 *   non-settled case via the convergence check below and flag those points so
 *   the UI can mark them low-confidence rather than presenting a confident
 *   wrong number.  See AcSweepResult.notSettled.
 *
 * WHY MEASURE_CYCLES = 6 as two consecutive 3-cycle windows (whole-cycle DFT):
 *   The single-bin DFT accumulates I and Q sums over the measure window.  If
 *   the window does not span an integer number of cycles the tails of the
 *   sine contribute a DC-like artefact that biases the amplitude estimate.
 *   Using whole cycles zeroes this error by orthogonality.  We split the
 *   measure phase into two consecutive 3-cycle windows (W1, W2): the reported
 *   amplitude/phase come from the combined 6-cycle whole-window DFT (leakage
 *   free, well-averaged), while the per-window amplitudes A(W1) vs A(W2) drive
 *   the settle convergence check — in steady state they are equal; while a
 *   high-Q resonance is still building they differ.  Six cycles (vs the old
 *   five) splits evenly into two leakage-free halves at a ~20% per-point cost.
 *
 * WHY the settle convergence check (notSettled):
 *   For each output net we compare the amplitude measured over W1 against W2.
 *   If they differ by more than SETTLE_TOL_DB the amplitude is still changing
 *   across the measure window, which means the fixed settle was too short for
 *   this (resonant) point — the magnitude is under-reported.  We only evaluate
 *   the check when the output is above SETTLE_FLOOR_FRAC of the reference, so
 *   deep-stopband points (amplitude in the numerical noise floor) do not
 *   produce spurious flags.
 *
 * WHY measure relative to the source output net (not the configured amplitude):
 *   A non-zero rSource causes a voltage drop that reduces the actual injected
 *   amplitude below the configured value.  Measuring the reference at the
 *   source's "pos" net (after rSource) means all magnitude ratios are relative
 *   to what the circuit actually sees, not the open-circuit drive level.  This
 *   keeps the gain flat for a unity-gain buffer at any rSource.
 *
 * WHY two run variants:
 *   runAcSweep (synchronous) is the testable core loop.  runAcSweepAsync
 *   yields a macrotask between points so the analysis worker's event loop can
 *   process a queued "cancel" message.  Without the yield the worker handler
 *   never returns mid-run and cooperative cancellation is a no-op.
 */

import { SimEngine } from "../sim/engine/sim-engine.js";
import type { SimCircuit } from "../sim/engine/sim-engine.js";
import {
  logSweepValues,
  type AcSweepSpec,
  type AcSweepResult,
} from "./jobs.js";
import { ANALYSIS_STEP_S } from "./run-sweep.js";
import { stepAnalysisEngine } from "./checked-step.js";

// ─── Constants ────────────────────────────────────────────────────────────────

/**
 * Number of cycles to run before starting DFT accumulation.
 * See module-level WHY comment for the derivation.
 */
const SETTLE_CYCLES = 5;

/**
 * Total whole cycles over which to accumulate DFT sums, split into two equal
 * consecutive windows.  Must be an even integer — see WHY whole-cycle DFT
 * window above.
 */
const MEASURE_CYCLES = 6;

/** Cycles per sub-window (MEASURE_CYCLES / 2).  Each window is leakage-free. */
const MEASURE_HALF_CYCLES = MEASURE_CYCLES / 2;

/**
 * Amplitude-change threshold (dB) between the two measure windows above which a
 * point is flagged notSettled.  0.5 dB is well below the looser sweep
 * tolerances yet large enough that steady-state numerical jitter never trips
 * it.
 */
const SETTLE_TOL_DB = 0.5;

/**
 * An output net is only eligible for the settle check when its amplitude is at
 * least this fraction of the reference amplitude.  Below it (≈ −40 dB) the
 * point is in the stopband noise floor where W1/W2 ratios are meaningless.
 */
const SETTLE_FLOOR_FRAC = 0.01;

/**
 * Target samples per cycle for the adaptive step.
 * 40 keeps the single-bin DFT amplitude error well under 1 dB.
 */
const SAMPLES_PER_CYCLE = 40;

const TWO_PI = 2 * Math.PI;

// ─── Types ────────────────────────────────────────────────────────────────────

export interface AcSweepHooks {
  onPoint?: (index: number, total: number) => void;
  shouldCancel?: () => boolean;
}

interface AcPointResult {
  magnitudes: number[];   // amplitude for each output net (linear V)
  phases: number[];       // phase for each output net (radians)
  refAmplitude: number;   // reference amplitude (linear V, at source pos net)
  refPhase: number;       // reference phase (radians)
  notSettled: boolean;    // amplitude still changing across the measure window
  failureCompIds: string[];
}

/** Single-bin DFT accumulators for one measure window. */
interface WindowDft {
  I_out: Float64Array;   // in-phase sum per output net
  Q_out: Float64Array;   // quadrature sum per output net
  I_ref: number;         // in-phase sum for the reference net
  Q_ref: number;         // quadrature sum for the reference net
  samples: number;       // sample count in this window
}

/**
 * Step the engine to `endTime`, accumulating single-bin DFT sums for the
 * reference net and every output net.  Samples AFTER each step so voltages
 * reflect the post-step simulation time.  Clamps the final step to land exactly
 * on `endTime` so the window spans a whole number of cycles.
 */
function accumulateWindow(
  engine: SimEngine,
  endTime: number,
  h: number,
  f: number,
  refNetId: string,
  outputNetIds: readonly string[],
  pointIndex: number,
): WindowDft {
  const numNets = outputNetIds.length;
  const I_out = new Float64Array(numNets);
  const Q_out = new Float64Array(numNets);
  let I_ref = 0;
  let Q_ref = 0;
  let samples = 0;

  let t = engine.simTime;
  while (t < endTime) {
    const remaining = endTime - t;
    const stepH = Math.min(h, remaining);
    stepAnalysisEngine(
      engine,
      stepH,
      `run-ac-sweep measure at point ${String(pointIndex)} (f=${String(f)} Hz)`,
    );
    t = engine.simTime;

    const netV = engine.getNetV();
    const vRef = netV[refNetId] ?? 0;
    const sinT = Math.sin(TWO_PI * f * t);
    const cosT = Math.cos(TWO_PI * f * t);

    I_ref += vRef * sinT;
    Q_ref += vRef * cosT;

    for (let j = 0; j < numNets; j++) {
      const v = netV[outputNetIds[j]!] ?? 0;
      I_out[j] += v * sinT;
      Q_out[j] += v * cosT;
    }
    samples++;
  }

  return { I_out, Q_out, I_ref, Q_ref, samples };
}

/** Single-bin DFT amplitude from accumulated I/Q sums over `samples` points. */
function windowAmplitude(I: number, Q: number, samples: number): number {
  if (samples === 0) return 0;
  return (2 / samples) * Math.sqrt(I * I + Q * Q);
}

// ─── Reference net resolution ────────────────────────────────────────────────

/**
 * Resolve the net id for the signal_gen's positive output pin.
 *
 * The engine's `nets` array is of type Net[] where each net has
 * `pins: Array<[componentId: string, pinId: string]>`.  We search for
 * the net whose pins include (componentId, "pos") — "pos" is the positive
 * pin id for signal_gen per the catalog convention (same as bench_psu and
 * battery_pack).
 *
 * WHY do this after coldLoad() rather than from the raw circuit:
 *   buildNets() may merge wired pins into combined nets.  The net id is
 *   only stable after the engine has run buildNets() internally.
 */
function resolveRefNetId(engine: SimEngine, componentId: string): string {
  const net = engine.nets.find((n) =>
    n.pins.some(([c, p]) => c === componentId && p === "pos"),
  );
  if (!net) {
    throw new Error(
      `run-ac-sweep: cannot resolve reference net for signal_gen "${componentId}" — ` +
      `no net contains its "pos" pin. Check that the source is wired into the circuit.`,
    );
  }
  return net.id;
}

// ─── Per-point runner ─────────────────────────────────────────────────────────

/**
 * Run one AC sweep point at frequency `f` Hz.
 *
 * Throws on irrecoverable errors (engine failure, non-convergence, dead
 * reference net). Each attempted step is restored before a convergence error
 * escapes, so the DFT never consumes a failed iterate.
 * The worker shell converts thrown errors to error messages.
 */
function runAcPoint(
  spec: AcSweepSpec,
  circuit: SimCircuit,
  f: number,
  pointIndex: number,
  outputNetIds: readonly string[],
): AcPointResult {
  // Clone so the mutation below does not affect the caller's circuit object.
  const pointCircuit: SimCircuit = structuredClone(circuit);

  // Mutate the target signal_gen: set waveform to sine at frequency f.
  // Amplitude, offset, rSource, and enabled are intentionally left untouched —
  // see module-level WHY comment for the rationale.
  const sg = pointCircuit.components.find((c) => c.id === spec.componentId);
  if (sg) {
    sg.params.waveform = "sine";
    sg.params.frequency = f;
    sg.params.delay = 0;
    sg.params.phaseDeg = 0;
  }

  // Fresh engine per point — element state must not carry between frequencies.
  const engine = new SimEngine();
  try {
    engine.coldLoad(pointCircuit);
  } catch (err) {
    throw new Error(
      `run-ac-sweep: coldLoad failed at point ${String(pointIndex)} (f=${String(f)} Hz): ${String(err)}`,
    );
  }

  // Resolve the reference net after coldLoad() so we see the post-buildNets ids.
  const refNetId = resolveRefNetId(engine, spec.componentId);

  // Adaptive step: ANALYSIS_STEP_S is the ceiling; 1/(f×40) ensures ≥40 samples/cycle.
  const h = Math.min(ANALYSIS_STEP_S, 1 / (f * SAMPLES_PER_CYCLE));

  // ── Settle phase: run SETTLE_CYCLES full cycles without accumulation ─────

  const settleEnd = SETTLE_CYCLES / f;
  let simTime = engine.simTime;

  while (simTime < settleEnd) {
    const remaining = settleEnd - simTime;
    const stepH = Math.min(h, remaining);
    stepAnalysisEngine(
      engine,
      stepH,
      `run-ac-sweep settle at point ${String(pointIndex)} (f=${String(f)} Hz)`,
    );
    simTime = engine.simTime;
  }

  // ── Measure phase: two consecutive whole-cycle windows (W1, W2) ──────────
  // The reported amplitude/phase come from the COMBINED 6-cycle window (DFT
  // sums are linear, so summing the two windows' I/Q gives the exact 6-cycle
  // result); the per-window amplitudes drive the settle convergence check.
  const numNets = outputNetIds.length;
  const w1End = simTime + MEASURE_HALF_CYCLES / f;
  const w2End = simTime + MEASURE_CYCLES / f;

  const w1 = accumulateWindow(engine, w1End, h, f, refNetId, outputNetIds, pointIndex);
  const w2 = accumulateWindow(engine, w2End, h, f, refNetId, outputNetIds, pointIndex);

  const N = w1.samples + w2.samples;

  // Guard against degenerate case (no samples — should not happen with ≥40 per cycle).
  if (N === 0) {
    throw new Error(
      `run-ac-sweep: no samples accumulated at point ${String(pointIndex)} (f=${String(f)} Hz)`,
    );
  }

  // ── Compute amplitudes and phases from the combined window ───────────────

  // Single-bin DFT amplitude: A = (2/N) × sqrt(I² + Q²).
  // The factor 2/N converts the half-spectrum DFT sum to a peak amplitude.
  const scale = 2 / N;

  const I_refTotal = w1.I_ref + w2.I_ref;
  const Q_refTotal = w1.Q_ref + w2.Q_ref;
  const refAmplitude = scale * Math.sqrt(I_refTotal * I_refTotal + Q_refTotal * Q_refTotal);
  const refPhase = Math.atan2(Q_refTotal, I_refTotal);

  // A flat or dead reference means all magnitude ratios are undefined.
  // Throw rather than return NaN/±Infinity — a dead source invalidates the
  // entire sweep run, not just this point.
  if (refAmplitude < 1e-9) {
    throw new Error(
      `run-ac-sweep: source net "${refNetId}" shows no signal at ${String(f)} Hz ` +
      `— check that the signal_gen is enabled and wired into the circuit.`,
    );
  }

  const magnitudes: number[] = new Array(numNets);
  const phases: number[] = new Array(numNets);
  const settleFloor = refAmplitude * SETTLE_FLOOR_FRAC;
  let notSettled = false;

  for (let j = 0; j < numNets; j++) {
    const Ij = w1.I_out[j]! + w2.I_out[j]!;
    const Qj = w1.Q_out[j]! + w2.Q_out[j]!;
    const A = scale * Math.sqrt(Ij * Ij + Qj * Qj);
    const phi = Math.atan2(Qj, Ij);
    magnitudes[j] = A;

    // Normalise phase difference into (−180, 180] degrees.
    let phaseDiffDeg = (phi - refPhase) * (180 / Math.PI);
    while (phaseDiffDeg > 180) phaseDiffDeg -= 360;
    while (phaseDiffDeg <= -180) phaseDiffDeg += 360;
    phases[j] = phaseDiffDeg;

    // Settle check: a still-building resonance shows a > SETTLE_TOL_DB change
    // in amplitude from W1 to W2; a settled point shows ~0.  Skip outputs in
    // the stopband noise floor (below SETTLE_FLOOR_FRAC of the reference).
    const a1 = windowAmplitude(w1.I_out[j]!, w1.Q_out[j]!, w1.samples);
    const a2 = windowAmplitude(w2.I_out[j]!, w2.Q_out[j]!, w2.samples);
    if (a1 > 0 && a2 > 0 && Math.max(a1, a2) >= settleFloor) {
      const dbDelta = Math.abs(20 * Math.log10(a2 / a1));
      if (dbDelta > SETTLE_TOL_DB) notSettled = true;
    }
  }

  // Collect latched failure component ids (mirrors run-sweep.ts).
  const failures = engine.getFailures();
  const failureCompIds = Array.from(
    new Set(Object.values(failures).map((f) => f.componentId)),
  );

  return { magnitudes, phases, refAmplitude, refPhase, notSettled, failureCompIds };
}

// ─── Result builders ─────────────────────────────────────────────────────────

interface AcAccumulator {
  freqs: number[];
  total: number;
  outputNetIds: string[];
  fHzResult: number[];
  magnitudesDb: number[][];
  phasesDeg: number[][];
  pointFailures: string[][];
  notSettled: boolean[];
}

function makeAcAccumulator(spec: AcSweepSpec): AcAccumulator {
  const freqs = logSweepValues(spec.fromHz, spec.toHz, spec.points);
  const outputNetIds = spec.outputs.map((o) => o.netId);
  return {
    freqs,
    total: freqs.length,
    outputNetIds,
    fHzResult: [],
    magnitudesDb: outputNetIds.map(() => []),
    phasesDeg: outputNetIds.map(() => []),
    pointFailures: [],
    notSettled: [],
  };
}

function accumulateAcPoint(
  acc: AcAccumulator,
  f: number,
  point: AcPointResult,
): void {
  acc.fHzResult.push(f);
  for (let j = 0; j < acc.outputNetIds.length; j++) {
    // magnitudeDb = 20 × log10(A_out / A_ref).
    // A_ref >= 1e-9 is guaranteed by the throw guard in runAcPoint.
    const db = 20 * Math.log10(point.magnitudes[j]! / point.refAmplitude);
    acc.magnitudesDb[j]!.push(db);
    acc.phasesDeg[j]!.push(point.phases[j]!);
  }
  acc.pointFailures.push(point.failureCompIds);
  acc.notSettled.push(point.notSettled);
}

function buildAcResult(
  spec: AcSweepSpec,
  acc: AcAccumulator,
  cancelled: boolean,
): AcSweepResult {
  return {
    spec,
    fHz: acc.fHzResult,
    outputs: acc.outputNetIds.map((netId, j) => ({
      netId,
      magnitudeDb: acc.magnitudesDb[j]!,
      phaseDeg: acc.phasesDeg[j]!,
    })),
    pointFailures: acc.pointFailures,
    notSettled: acc.notSettled,
    cancelled,
  };
}

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * Run an AC frequency-response sweep synchronously and return the result.
 *
 * Intended for tests and non-worker callers.  Inside the analysis worker use
 * runAcSweepAsync instead — a synchronous run never returns to the worker's
 * event loop, so cooperative cancellation could not work.
 *
 * @param spec    - Validated AC sweep specification (call validateJobSpec first).
 * @param circuit - The circuit to sweep.  Mutated via structuredClone per point.
 * @param hooks   - Optional: onPoint(i, total) for progress, shouldCancel() checked after each point.
 */
export function runAcSweep(
  spec: AcSweepSpec,
  circuit: SimCircuit,
  hooks?: AcSweepHooks,
): AcSweepResult {
  const acc = makeAcAccumulator(spec);

  for (let i = 0; i < acc.total; i++) {
    const f = acc.freqs[i]!;
    accumulateAcPoint(acc, f, runAcPoint(spec, circuit, f, i, acc.outputNetIds));

    // Notify progress before the cancel check so the caller sees the completed point.
    hooks?.onPoint?.(i, acc.total);

    if (hooks?.shouldCancel?.()) {
      return buildAcResult(spec, acc, true);
    }
  }

  return buildAcResult(spec, acc, false);
}

/**
 * Async AC sweep runner: identical semantics to runAcSweep, but yields a
 * macrotask between points so the hosting worker's event loop can process
 * queued messages — in particular "cancel", which sets the flag behind
 * shouldCancel().  Without this yield, a cancel posted mid-run would only be
 * handled after the final point, making cooperative cancellation a no-op.
 */
export async function runAcSweepAsync(
  spec: AcSweepSpec,
  circuit: SimCircuit,
  hooks?: AcSweepHooks,
): Promise<AcSweepResult> {
  const acc = makeAcAccumulator(spec);

  for (let i = 0; i < acc.total; i++) {
    const f = acc.freqs[i]!;
    accumulateAcPoint(acc, f, runAcPoint(spec, circuit, f, i, acc.outputNetIds));

    hooks?.onPoint?.(i, acc.total);

    // Macrotask yield — queued worker messages only dispatch between macrotasks.
    // A microtask (resolved Promise) would not let "cancel" arrive mid-run.
    await new Promise<void>((resolve) => setTimeout(resolve, 0));

    if (hooks?.shouldCancel?.()) {
      return buildAcResult(spec, acc, true);
    }
  }

  return buildAcResult(spec, acc, false);
}
