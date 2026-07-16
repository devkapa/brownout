/**
 * Analysis job types and pure helpers for sweep analysis.
 *
 * WHY shared types in a dedicated module:
 *   Both the worker shell (analysis.worker.ts) and the main-thread client
 *   (analysis-worker-client.ts) need the same spec/result types.  Keeping
 *   them here—separate from the runner—avoids circular imports and lets
 *   vitest import the validators without loading the engine.
 *
 * WHY voltage-only outputs in v1:
 *   Current probes require a specific branch (component + pin pair) rather
 *   than a net; adding that adds surface area without a teaching use-case
 *   today.  Net voltage is sufficient for divider/sweep demonstrations.
 *
 * NOTE on output net id validation:
 *   Net IDs are engine artefacts produced by buildNets() after coldLoad().
 *   They are NOT knowable from the raw circuit JSON, so validateJobSpec()
 *   does NOT validate them here.  The runner returns 0-filled arrays for
 *   unresolvable net ids and does not throw; the UI is responsible for
 *   letting users pick live net ids from a loaded circuit.
 *
 * NOTE on AcSweepSpec — bespoke sine-sweep, NOT SPICE AC:
 *   The frequency response job drives the circuit with a real sine waveform
 *   per frequency point using the live MNA engine.  It does NOT linearise the
 *   circuit or compute a small-signal transfer matrix.  Nonlinear elements
 *   affect the result (clipping, bias-point shifts) — this is intentional:
 *   learners see what the physical circuit actually does, not an idealised
 *   linear approximation.  Accuracy is teaching-grade (single-bin DFT, 40
 *   samples/cycle, 5 settle cycles).
 */

import type { SimCircuit } from "../sim/engine/sim-engine.js";
import type { ComponentKind } from "../circuit/types.js";
import { resolveCatalogPart } from "../circuit/catalog-resolver.js";

// ─── Caps ─────────────────────────────────────────────────────────────────────

/**
 * Maximum sweep points: 201 gives ≤201 distinct independent variable values
 * across [from, to].  Browser CPU budget: worst case 201 × 0.5 s settle at
 * ANALYSIS_STEP_S=5e-5 is ≈2 M iterations — acceptable on a modern core.
 */
export const SWEEP_MAX_POINTS = 201;

/**
 * Minimum two points so there is at least a start and an end; a single-point
 * sweep has no meaning (no independent variable range).
 */
export const SWEEP_MIN_POINTS = 2;

/**
 * Four output nets: enough for a standard scope (ch1–ch4).  More outputs
 * multiply the per-point work with diminishing teaching value.
 */
export const SWEEP_MAX_OUTPUTS = 4;

/**
 * Settle time lower bound: 10 ms gives enough time for most RC time-constants
 * in teaching circuits (τ = RC; typical RC = 1 kΩ × 10 µF = 10 ms).
 */
export const SWEEP_SETTLE_MIN_S = 0.01;

/**
 * Settle time upper bound: 500 ms keeps worst-case total job time under
 * SWEEP_MAX_POINTS × SWEEP_SETTLE_MAX_S = 201 × 0.5 s = ~100 s on a single
 * core.  Beyond this the browser feels frozen even with a cancel button.
 */
export const SWEEP_SETTLE_MAX_S = 0.5;

// ─── Types ────────────────────────────────────────────────────────────────────

/**
 * One net to record per sweep point.
 *
 * v1 records only voltage; future versions may add { kind: "current"; compId; pinId }.
 */
export interface SweepOutputSpec {
  netId: string;
}

/**
 * Sweep the DC voltage (or any numeric param) of a single component.
 *
 * Typical use: sweep bench_psu.voltage or battery_pack.voltage to see how
 * divider / LED-resistor output varies with supply.
 *
 * settleTimeS: how long (simulated seconds) to run the engine at each point
 *   before sampling the tail average.
 */
export interface DcSweepSpec {
  kind: "dc-sweep";
  componentId: string;
  param: string;
  from: number;
  to: number;
  points: number;
  settleTimeS: number;
  outputs: SweepOutputSpec[];
}

/**
 * Sweep ambient temperature across the whole circuit's environment.
 *
 * Affects all temperature-sensitive parts (thermistor) simultaneously.
 * The range -40..125 °C covers the full commercial/industrial/automotive range.
 */
export interface TempSweepSpec {
  kind: "temp-sweep";
  from: number;
  to: number;
  points: number;
  settleTimeS: number;
  outputs: SweepOutputSpec[];
}

// ─── Monte Carlo caps ─────────────────────────────────────────────────────────

/**
 * Minimum Monte Carlo runs.
 *
 * WHY 10: fewer than 10 samples give a highly unstable stddev estimate — the
 * teaching value of the histogram evaporates with tiny sample counts.
 */
export const MC_MIN_RUNS = 10;

/**
 * Maximum Monte Carlo runs.
 *
 * WHY 200: each run is a full transient settle (up to 0.5 s simulated at
 * ANALYSIS_STEP_S = 5e-5).  200 × 0.5 s / 5e-5 = 2 M steps — the accepted
 * worst-case CPU ceiling on a modern browser core.  Beyond 200 runs the
 * incremental distribution improvement is marginal for teaching purposes.
 */
export const MC_MAX_RUNS = 200;

// ─── AC Sweep caps ────────────────────────────────────────────────────────────

/**
 * Minimum frequency for the AC sweep.
 *
 * WHY 10 Hz: below this the settle+measure window (11 cycles × 1/f) exceeds
 * 1 second per point.  With up to 101 points that would be >100 s total —
 * unacceptable for an interactive teaching tool.
 */
export const AC_MIN_HZ = 10;

/**
 * Maximum frequency for the AC sweep.
 *
 * WHY 100 kHz: above this the fixed-step approach (ANALYSIS_STEP_S = 5e-5)
 * would produce only 2 samples/cycle — well below the SAMPLES_PER_CYCLE=40
 * floor that keeps the single-bin DFT error under 1 dB.  The adaptive step
 * h = min(ANALYSIS_STEP_S, 1/(f·40)) compensates, but at high frequencies
 * the step becomes extremely small and per-point time explodes.
 */
export const AC_MAX_HZ = 100_000;

/**
 * Maximum number of log-spaced frequency points.
 *
 * WHY 101: at worst case (100 kHz top, 40 samples/cycle) each point runs
 * ~5×40 = 200 steps for settle + 6×40 = 240 for measure = 440 steps.
 * 101 × 440 = ~44 k steps — fast even on a slow core.  101 gives dense
 * enough Bode coverage (≈2 points/decade for 10 Hz–100 kHz = 4 decades).
 */
export const AC_MAX_POINTS = 101;

/**
 * Frequency response (AC) sweep specification.
 *
 * Drives the named signal_gen with a sine at each log-spaced frequency,
 * measures magnitude and phase at each output net relative to the source's
 * actual output net (after rSource loading).
 *
 * WHY amplitude/offset are used as-is:
 *   Keeping the user's configured drive level means the result reflects
 *   nonlinear behaviour (clipping, saturation) honestly.  A small-signal AC
 *   analysis would linearise at the bias point and miss these effects.
 *   Users who want small-signal results should use a small amplitude.
 *
 * WHY no settleTimeS param:
 *   Settle is fixed at 5 cycles and measure at 6 (two 3-cycle windows; see
 *   run-ac-sweep.ts) so users do not need to reason about RC time constants
 *   relative to the drive frequency.  Resonant circuits whose build-up exceeds
 *   the fixed settle are detected and flagged via AcSweepResult.notSettled
 *   rather than requiring a user-tuned settle.
 */
export interface AcSweepSpec {
  kind: "ac-sweep";
  /** Component id of the signal_gen to drive. */
  componentId: string;
  /** Sweep start frequency in Hz (>= AC_MIN_HZ). */
  fromHz: number;
  /** Sweep end frequency in Hz (<= AC_MAX_HZ, > fromHz). */
  toHz: number;
  /** Number of log-spaced frequency points (2..AC_MAX_POINTS). */
  points: number;
  /** Output nets to probe. */
  outputs: SweepOutputSpec[];
}

/**
 * Monte Carlo tolerance sweep specification.
 *
 * Runs the circuit `runs` times, each with independently perturbed component
 * values (within their catalog-specified tolerances), and records the
 * tail-averaged output voltage distribution.
 *
 * settleTimeS: how long (simulated seconds) to run the engine per run before
 *   sampling the tail mean (same semantics as DcSweepSpec.settleTimeS).
 *
 * seed: integer seed for the deterministic PRNG.  The same seed always
 *   produces the same perturbations regardless of component iteration order
 *   (see run-monte-carlo.ts seeding scheme).
 */
export interface MonteCarloSpec {
  kind: "monte-carlo";
  runs: number;
  seed: number;
  settleTimeS: number;
  outputs: SweepOutputSpec[];
}

export type AnalysisJobSpec = DcSweepSpec | TempSweepSpec | AcSweepSpec | MonteCarloSpec;

/**
 * Result for a completed (or cooperatively-cancelled) sweep job.
 *
 * x[i]             — independent variable value at point i
 * outputs[j].mean[i] — tail-averaged voltage at output j, point i
 * outputs[j].min[i]  — minimum voltage seen during the averaging window
 * outputs[j].max[i]  — maximum voltage seen during the averaging window
 * pointFailures[i]  — failure component ids latched at point i (empty when clean)
 * cancelled        — true when the run was stopped before all points completed;
 *                    arrays are truncated to completed-point length.
 */
export interface AnalysisJobResult {
  spec: AnalysisJobSpec;
  x: number[];
  outputs: Array<{
    netId: string;
    mean: number[];
    min: number[];
    max: number[];
  }>;
  pointFailures: string[][];
  cancelled: boolean;
}

/**
 * Result for a completed (or cooperatively-cancelled) AC sweep job.
 *
 * fHz[i]                      — frequency in Hz at point i
 * outputs[j].netId            — net being probed
 * outputs[j].magnitudeDb[i]   — 20·log10(A_out / A_ref) at frequency i
 * outputs[j].phaseDeg[i]      — phase difference (output − source) in degrees,
 *                               normalised into (−180, 180]
 * pointFailures[i]            — failure component ids at point i (empty = clean)
 * notSettled[i]              — true when the measured amplitude was still
 *                              changing across the measure window at point i, so
 *                              the magnitude is under-reported (see run-ac-sweep.ts
 *                              "settle convergence check").  This fires for sharp
 *                              resonant (high-Q) circuits whose build-up time
 *                              constant exceeds the fixed settle window.  The UI
 *                              surfaces these points as low-confidence.
 * cancelled                   — true when the run was stopped early
 *
 * WHY separate from AnalysisJobResult:
 *   The AC result has fundamentally different columns (dB, phase) vs the DC
 *   result (mean, min, max voltage).  A union keeps the type system honest
 *   and lets the UI branch explicitly on isAcSweepResult().
 */
export interface AcSweepResult {
  spec: AcSweepSpec;
  fHz: number[];
  outputs: Array<{
    netId: string;
    magnitudeDb: number[];
    phaseDeg: number[];
  }>;
  pointFailures: string[][];
  notSettled: boolean[];
  cancelled: boolean;
}

/**
 * Result for a completed (or cooperatively-cancelled) Monte Carlo run.
 *
 * outputs[j].values[r]  — tail-averaged voltage at output j in run r
 * outputs[j].mean       — mean of values across all completed runs
 * outputs[j].min        — minimum value across all completed runs
 * outputs[j].max        — maximum value across all completed runs
 * outputs[j].stddev     — population standard deviation across all completed runs
 *                         (population rather than sample: we are describing the
 *                         distribution of the simulation, not estimating a
 *                         parent population — every drawn sample IS the
 *                         population for the given seed)
 * pointFailures[r]      — failure component ids in run r (empty when clean)
 * cancelled             — true when the run was stopped before all runs completed
 */
export interface MonteCarloResult {
  spec: MonteCarloSpec;
  runs: number;
  outputs: Array<{
    netId: string;
    values: number[];
    mean: number;
    min: number;
    max: number;
    stddev: number;
  }>;
  pointFailures: string[][];
  cancelled: boolean;
}

/**
 * Union of all result types produced by the analysis worker.
 *
 * Use isAcSweepResult() / isMonteCarloResult() to discriminate before accessing
 * kind-specific fields.
 */
export type AnalysisAnyResult = AnalysisJobResult | AcSweepResult | MonteCarloResult;

/**
 * Type guard: true when result is an AcSweepResult rather than an AnalysisJobResult.
 */
export function isAcSweepResult(r: AnalysisAnyResult): r is AcSweepResult {
  return r.spec.kind === "ac-sweep";
}

/**
 * Type guard: true when result is a MonteCarloResult.
 */
export function isMonteCarloResult(r: AnalysisAnyResult): r is MonteCarloResult {
  return r.spec.kind === "monte-carlo";
}

// ─── Pure helpers ─────────────────────────────────────────────────────────────

/**
 * Inclusive linear space from `from` to `to` with exactly `points` values.
 *
 * Handles reversed ranges (from > to) correctly — the result always starts
 * at `from` and ends at `to`.  Points < 2 returns a single-element array
 * containing `from`; callers should validate before calling.
 */
export function sweepValues(from: number, to: number, points: number): number[] {
  if (points <= 1) return [from];
  const result: number[] = new Array(points);
  const step = (to - from) / (points - 1);
  for (let i = 0; i < points; i++) {
    result[i] = from + i * step;
  }
  // Force the last value to be exactly `to` to avoid floating-point drift.
  result[points - 1] = to;
  return result;
}

/**
 * Inclusive log-spaced array from `fromHz` to `toHz` with exactly `points`
 * values.
 *
 * WHY log spacing: frequency response analysis is conventionally plotted on
 * a logarithmic frequency axis.  Equal log steps place the same number of
 * points per decade, giving uniform coverage from low to high frequencies.
 *
 * Points < 2 returns a single-element array containing `fromHz`.
 */
export function logSweepValues(fromHz: number, toHz: number, points: number): number[] {
  if (points <= 1) return [fromHz];
  const logFrom = Math.log10(fromHz);
  const logTo = Math.log10(toHz);
  const result: number[] = new Array(points);
  const step = (logTo - logFrom) / (points - 1);
  for (let i = 0; i < points; i++) {
    result[i] = Math.pow(10, logFrom + i * step);
  }
  // Force endpoints to be exact to avoid floating-point drift at the boundaries.
  result[0] = fromHz;
  result[points - 1] = toHz;
  return result;
}

/**
 * Validate an AnalysisJobSpec against the provided circuit.
 *
 * Returns a human-readable error string or null when the spec is valid.
 *
 * NOTE: output net ids are NOT validated here — they are only meaningful
 * after engine coldLoad() resolves the net topology.  The runner returns
 * 0-filled arrays for unknown net ids rather than throwing.
 */
export function validateJobSpec(
  spec: AnalysisJobSpec,
  circuit: SimCircuit,
): string | null {
  // ── Monte Carlo has its own validation path ───────────────────────────────

  if (spec.kind === "monte-carlo") {
    if (!Number.isInteger(spec.runs) || spec.runs < MC_MIN_RUNS || spec.runs > MC_MAX_RUNS) {
      return `runs must be an integer between ${String(MC_MIN_RUNS)} and ${String(MC_MAX_RUNS)}`;
    }

    if (!Number.isInteger(spec.seed)) {
      return "seed must be an integer";
    }

    if (!isFinite(spec.settleTimeS) || spec.settleTimeS < SWEEP_SETTLE_MIN_S || spec.settleTimeS > SWEEP_SETTLE_MAX_S) {
      return `settleTimeS must be between ${String(SWEEP_SETTLE_MIN_S)} and ${String(SWEEP_SETTLE_MAX_S)}`;
    }

    if (!spec.outputs || spec.outputs.length < 1 || spec.outputs.length > SWEEP_MAX_OUTPUTS) {
      return `outputs must have between 1 and ${String(SWEEP_MAX_OUTPUTS)} entries`;
    }

    // Verify at least one component in the circuit has applicable tolerance
    // metadata (value_tol for resistors/capacitors, vf_tol for diodes/LEDs).
    // vth_tol is metadata-only in v1 and does NOT qualify here because the
    // engine has no per-instance threshold override.
    const hasTolComponent = circuit.components.some((comp) => {
      const specs = resolveCatalogPart({
        kind: comp.kind as ComponentKind,
        params: comp.params,
        catalogUid: comp.catalogUid,
      })?.electrical_specs;
      if (!specs) return false;
      // value_tol applies when the component also has the corresponding param.
      if (specs.value_tol != null && (
        comp.params.resistance != null || comp.params.capacitance != null
      )) return true;
      // vf_tol applies only when a forward voltage is actually readable by the
      // runner — the catalog scalar vf or a per-emitter vf param.  This mirrors
      // run-monte-carlo.ts's perturbation set and excludes parts like tvs_diode
      // that declare vf_tol but expose no Vf, so a tvs-only circuit is correctly
      // reported as having nothing to perturb instead of advertising an empty run.
      if (specs.vf_tol != null && (
        specs.vf != null ||
        comp.params.vf != null ||
        comp.params.vf1 != null || comp.params.vf2 != null ||
        comp.params.vf_r != null || comp.params.vf_g != null || comp.params.vf_b != null
      )) return true;
      return false;
    });

    if (!hasTolComponent) {
      return "no components with tolerance data (resistors, capacitors, diodes, LEDs)";
    }

    return null;
  }

  // ── AC sweep has its own validation path (different caps and no settleTimeS) ──

  if (spec.kind === "ac-sweep") {
    if (!spec.outputs || spec.outputs.length < 1 || spec.outputs.length > SWEEP_MAX_OUTPUTS) {
      return `outputs must have between 1 and ${String(SWEEP_MAX_OUTPUTS)} entries`;
    }

    if (!Number.isInteger(spec.points) || spec.points < 2 || spec.points > AC_MAX_POINTS) {
      return `points must be an integer between 2 and ${String(AC_MAX_POINTS)}`;
    }

    const comp = circuit.components.find((c) => c.id === spec.componentId);
    if (!comp) {
      return `component "${spec.componentId}" not found in circuit`;
    }
    if (comp.kind !== "signal_gen") {
      return `component "${spec.componentId}" must be a signal_gen (bench_psu and battery_pack cannot generate a sine wave)`;
    }
    // A disabled source produces no signal — the reference net will be flat
    // and the DFT reference amplitude will be ~0, making all ratios meaningless.
    if (Number(comp.params.enabled ?? 1) === 0) {
      return `signal_gen "${spec.componentId}" is disabled — enable it before running an AC sweep`;
    }
    const amplitude = Number(comp.params.amplitude ?? 0);
    if (!isFinite(amplitude) || amplitude <= 0) {
      return `signal_gen "${spec.componentId}" must have a positive amplitude for the AC sweep`;
    }

    if (!isFinite(spec.fromHz) || !isFinite(spec.toHz)) {
      return "fromHz and toHz must be finite numbers";
    }
    if (spec.fromHz < AC_MIN_HZ || spec.fromHz > AC_MAX_HZ) {
      return `fromHz must be between ${String(AC_MIN_HZ)} and ${String(AC_MAX_HZ)} Hz`;
    }
    if (spec.toHz < AC_MIN_HZ || spec.toHz > AC_MAX_HZ) {
      return `toHz must be between ${String(AC_MIN_HZ)} and ${String(AC_MAX_HZ)} Hz`;
    }
    if (spec.fromHz >= spec.toHz) {
      return "fromHz must be less than toHz";
    }

    return null;
  }

  // ── Common checks for dc-sweep and temp-sweep ────────────────────────────

  if (!Number.isInteger(spec.points) || spec.points < SWEEP_MIN_POINTS || spec.points > SWEEP_MAX_POINTS) {
    return `points must be an integer between ${String(SWEEP_MIN_POINTS)} and ${String(SWEEP_MAX_POINTS)}`;
  }

  if (!isFinite(spec.settleTimeS) || spec.settleTimeS < SWEEP_SETTLE_MIN_S || spec.settleTimeS > SWEEP_SETTLE_MAX_S) {
    return `settleTimeS must be between ${String(SWEEP_SETTLE_MIN_S)} and ${String(SWEEP_SETTLE_MAX_S)}`;
  }

  if (!spec.outputs || spec.outputs.length < 1 || spec.outputs.length > SWEEP_MAX_OUTPUTS) {
    return `outputs must have between 1 and ${String(SWEEP_MAX_OUTPUTS)} entries`;
  }

  // ── Kind-specific checks ─────────────────────────────────────────────────

  if (spec.kind === "dc-sweep") {
    const comp = circuit.components.find((c) => c.id === spec.componentId);
    if (!comp) {
      return `component "${spec.componentId}" not found in circuit`;
    }
    // A disabled signal_gen is stamped hi-Z by the engine, so sweeping its
    // offset/amplitude produces a flat ~0 curve at every point — an
    // authoritative-looking but meaningless result.  Reject it up front, the
    // same way the ac-sweep branch does.  Only signal_gen carries `enabled`;
    // bench_psu/battery_pack have no such param, so this check is kind-gated.
    if (comp.kind === "signal_gen" && Number(comp.params.enabled ?? 1) === 0) {
      return `signal_gen "${spec.componentId}" is disabled — enable it before running a DC sweep`;
    }
    const paramVal = comp.params[spec.param];
    if (paramVal === undefined) {
      return `component "${spec.componentId}" does not have param "${spec.param}"`;
    }
    if (typeof paramVal !== "number" && typeof Number(paramVal) !== "number") {
      return `param "${spec.param}" on component "${spec.componentId}" is not numeric`;
    }
    if (!isFinite(Number(paramVal))) {
      return `param "${spec.param}" on component "${spec.componentId}" is not a finite number`;
    }
    if (!isFinite(spec.from) || !isFinite(spec.to)) {
      return "from and to must be finite numbers";
    }
  }

  if (spec.kind === "temp-sweep") {
    const TEMP_MIN = -40;
    const TEMP_MAX = 125;
    if (!isFinite(spec.from) || !isFinite(spec.to)) {
      return "from and to must be finite numbers";
    }
    const lo = Math.min(spec.from, spec.to);
    const hi = Math.max(spec.from, spec.to);
    if (lo < TEMP_MIN || hi > TEMP_MAX) {
      return `temperature range must be within ${String(TEMP_MIN)}..${String(TEMP_MAX)} °C`;
    }
  }

  return null;
}
