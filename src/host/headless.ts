/**
 * Headless host adapter: drives SimEngine from plain Node with no worker, no
 * transport, and no DOM.
 *
 * WHAT A HOST DOES (and why this class exists)
 * SimEngine is a solver, not a loop: it exposes load()/step(h) and reports
 * Newton/MNA trouble through `lastConverged` instead of throwing, because
 * only the host knows whether to retry smaller, roll back, or stop. The
 * choose-h / attempt / accept-or-rollback policy therefore lives OUTSIDE the
 * engine, and every host reimplements it. de:volt's browser host is
 * packages/simcore/src/sim/sim.worker.ts; this is the Node equivalent of its
 * step loop with the transport removed.
 *
 * ADAPTIVE STEPPING IS PORTED, NOT REINVENTED
 * The controller below is sim.worker.ts's, using the same exported
 * estimateStepError/nextStepFactor and the same constants. Correspondence to
 * that file, so the two can be diffed:
 *
 *   - attemptStep()               <- sim.worker.ts attemptStep()
 *   - sourceStepLimit()           <- sim.worker.ts sourceStepLimit()
 *   - sourceEventDescriptors(), nextSourceEventTime() and the source-event
 *     clamp in run()              <- sim.worker.ts, same names and clamp
 *   - circuitSupportsStepReplay() <- sim.worker.ts, same name
 *   - the accept/reject arms      <- sim.worker.ts's batch `while` loop
 *   - H_MIN/H_MAX/H_INITIAL/MCU_H_MAX, and the 0.25 reject factor and the
 *     >15 / <=3 Newton-iteration heuristic, are its values verbatim.
 *
 * DELIBERATELY NOT PORTED (a headless run has different pressures than a
 * 60 Hz browser frame — each omission is a behavior difference, not an
 * oversight):
 *
 *   - The 16 ms wall budget and batch cadence. A headless run has no frame to
 *     protect and no consumer waiting on a tick; it runs to completion. The
 *     runaway guard is `maxSteps` instead, mirroring runSpice's maxTranSteps
 *     rationale: a typo'd duration must fail fast, not hang the caller.
 *   - The error-check gap (worker: errorCheckCountdown / validatedStepFactor
 *     reuse, which skips the doubled trial on most steps to buy frame time).
 *     This runner checks EVERY error-controlled step. That is strictly more
 *     conservative — it costs ~3 solves per step and can only tighten h — and
 *     it makes the worker's postDiscontinuityChecks bookkeeping moot, since
 *     that exists solely to force checks the gap would otherwise skip.
 *   - Probe acquisition (uniform sample grid + interpolation). onSample fires
 *     on ACCEPTED STEP BOUNDARIES, which under adaptive control are not
 *     uniformly spaced — see HeadlessSample.
 *
 * MCU circuits are handled exactly as the worker handles them: the external
 * CPU emulators are not snapshot-rewindable, so a circuit containing one gets
 * the conservative controller with no trial replay, capped at MCU_H_MAX.
 *
 * ENGINE SELECTION (brownout/core, W0)
 * `HeadlessRunnerOptions.engine` opts a runner into trying the Rust/WASM
 * core (brownout/core) instead of the classic engine above. This is ADDITIVE:
 * with `engine` absent or `"classic"`, every code path below is exactly what
 * it was before the core existed — same fields, same control flow, same
 * error messages. In `"core"` mode, `load()`/`run()` take a completely
 * different path (see `_tryLoadCore`/`_runCore`): the adaptive step
 * controller above is entirely a classic-engine concern and has no
 * involvement once `activeEngine` is `"core"`. `"auto"` loads as `"core"`
 * does and adds one thing: a core failure in the middle of a run is replayed
 * on classic (see `_runCoreOrReplay`).
 */

import { estimateStepError, nextStepFactor } from "../sim/adaptive-step.js";
import { parseSpiceNetlist } from "../sim/engine/spice/netlist.js";
import { SIGNAL_GEN_NOISE_UPDATE_RATE_HZ, parseSignalGenParams } from "../sim/engine/waveform.js";
import { SimEngine } from "../sim/engine/sim-engine.js";
import type {
  SimCircuit,
  SimFailure,
  SimWarning,
  SolverDiagnosticsSnapshot,
} from "../sim/engine/sim-engine.js";
import { CoreEngine } from "../core/engine.js";
import { CoreUnsupportedError } from "../core/errors.js";
import { CORE_SUPPORTED_KINDS } from "../core/kinds.js";
import { coreAdvanceFailureReason, coreFailureReasons, unsupportedCoreKindReasons } from "../core/select.js";
import { coreReady, coreStatus } from "../core/wasm.js";
import type { EngineReadout } from "../core/surface.js";

// Adaptive step-size bounds (seconds) — sim.worker.ts values verbatim.
const H_MIN = 1e-8;
const H_MAX = 1e-2;
const H_INITIAL = 1e-8;
const MCU_H_MAX = 1e-4;
// While the circuit is changing, error-controlled steps stop at ACTIVE_H_MAX.
// Backward Euler lags an RC curve by about half a step, and the local
// tolerance alone let slow RC timing run at H_MAX: a 555 astable ran 1.5%
// slow (2.2% at 4x) and its period wandered ±0.6 ms cycle to cycle; at 2 ms,
// 0.6% and ±0.04 ms. "Changing" means a full H_MAX step would spend more than
// QUIET_FRACTION of the error tolerance, extrapolated from the last estimate
// (local error scales as h^(order+1)). A circuit sitting still keeps H_MAX:
// a flat 2 ms ceiling cost static logic boards 3-4x the solves for nothing.
const ACTIVE_H_MAX = 2e-3;
const QUIET_FRACTION = 0.1;

/**
 * Runaway guard. Same rationale as runSpice's maxTranSteps ceiling: an
 * adaptive run pinned at the 10 ns floor would need 1e8 steps to cover one
 * second, and a caller who typo'd durationS deserves an error naming the
 * problem rather than a hang.
 */
const DEFAULT_MAX_STEPS = 1_000_000;

/**
 * Circuits the trial-replay error estimate is valid for. sim.worker.ts
 * circuitSupportsStepReplay(), verbatim: the avr8js/rp2040js cores advance
 * real emulator state that saveState()/restoreState() do not capture, so
 * stepping one twice and rewinding would desynchronize the CPU from the
 * circuit. Those circuits grow h through the Newton-iteration heuristic only.
 */
function circuitSupportsStepReplay(circuit: SimCircuit): boolean {
  return !circuit.components.some((component) =>
    component.kind === "arduino_uno" ||
    component.kind === "arduino_nano" ||
    component.kind === "raspberry_pi_pico"
  );
}

/**
 * Source-driven step ceiling. sim.worker.ts sourceStepLimit(), verbatim.
 *
 * WHY THIS IS NOT OPTIONAL, despite the error estimate: a square/pulse source
 * is piecewise CONSTANT, so a coarse h step and two refined h/2 steps both
 * land on the same flat level and AGREE — the estimator reports ratio 0 and
 * grows h, right through the edges. A 1 kHz square then aliases to near-DC
 * (measured before this floor existed: h grew to 4.1 ms and a 5 ms run saw 3
 * transitions instead of 10) while every step reports itself accepted and
 * healthy. Error control cannot see a discontinuity it steps over; only a
 * density floor derived from the source's own parameters can.
 *
 * Periodic continuous waveforms need enough integration points per cycle;
 * discontinuous sources need at least ten points per period.
 */
function sourceStepLimit(circuit: SimCircuit): number {
  let limit = H_MAX;
  for (const component of circuit.components) {
    const p = component.params;
    if (component.kind === "clock" || component.kind === "clock_gen") {
      const frequency = Math.max(1e-6, Number(p.frequency ?? 1000));
      limit = Math.min(limit, 1 / (frequency * 10));
      continue;
    }
    if (component.kind === "pulse_source" || component.kind === "pulse_gen") {
      const period = Number(p.per ?? 1e-3);
      if (Number.isFinite(period) && period > 0) limit = Math.min(limit, period / 10);
      continue;
    }
    if (component.kind !== "signal_gen" || Number(p.enabled ?? 1) === 0) continue;
    const frequency = Math.max(1e-6, Number(p.frequency ?? 1000));
    // Unrecognised strings play as a sine and need its ceiling.
    const waveform = parseSignalGenParams(p).waveform;
    if (waveform === "sine" || waveform === "triangle" || waveform === "ramp") {
      limit = Math.min(limit, 1 / (frequency * 40));
    } else if (waveform === "square" || waveform === "pulse") {
      limit = Math.min(limit, 1 / (frequency * 10));
    } else if (waveform === "noise") {
      // The declared generator model is 10 kSa/s zero-order hold.
      limit = Math.min(limit, 1 / (SIGNAL_GEN_NOISE_UPDATE_RATE_HZ * 2));
    }
  }
  return Math.max(H_MIN, limit);
}

/**
 * A source event this close to a time counts as reached: floating point
 * cannot tell an edge from a step boundary rounded onto it. The clamp in
 * run() uses the same margin, so a step that ends this close to an edge is
 * the step that handles it. sim.worker.ts sourceEdgeEpsilon(), verbatim.
 */
function sourceEdgeEpsilon(t: number): number {
  return Math.max(1e-15, Math.abs(t) * 1e-12);
}

/** sim.worker.ts nextPeriodicEvent(), verbatim. */
function nextPeriodicEvent(
  now: number,
  origin: number,
  period: number,
  offsets: readonly number[],
): number | null {
  if (!Number.isFinite(period) || period <= 0) return null;
  const epsilon = sourceEdgeEpsilon(now);
  let next: number | null = null;
  for (const rawOffset of offsets) {
    const offset = ((rawOffset % period) + period) % period;
    const cycles = Math.floor((now - origin - offset) / period) + 1;
    const candidate = origin + offset + Math.max(0, cycles) * period;
    if (candidate > now + epsilon && (next === null || candidate < next)) next = candidate;
  }
  return next;
}

/** Run-invariant, pre-parsed source breakpoint description. */
type SourceEventDescriptor =
  | {
      kind: "periodic";
      startAt: number;
      origin: number;
      period: number;
      offsets: readonly number[];
    }
  | {
      kind: "fixed";
      startAt: number | null;
      times: readonly number[];
    };

/**
 * Every timed source's edges and corners. sim.worker.ts
 * sourceEventDescriptors(), verbatim.
 *
 * WHY THE CEILING ALONE IS NOT ENOUGH: sourceStepLimit() sets how many steps
 * a period gets, not where they fall, and the engine samples a source at the
 * END of a step. A step that straddles an edge applies the new level across
 * the whole step, and a feature narrower than the step is never sampled at
 * all. PWL has no ceiling and a pulse's ignores its width, so a 0.2 ms PWL
 * pulse was never seen in 20 ms, and of a 20 us pulse every 1 ms only the
 * first was (11.2 us HIGH against 420 us). run() lands a step on each event.
 */
function sourceEventDescriptors(circuit: SimCircuit): SourceEventDescriptor[] {
  const descriptors: SourceEventDescriptor[] = [];
  const addFixed = (times: number[], startAt: number | null = null): void => {
    const finiteTimes = [...new Set(times.filter(Number.isFinite))].sort((a, b) => a - b);
    descriptors.push({ kind: "fixed", startAt, times: finiteTimes });
  };
  const addPeriodic = (
    startAt: number,
    origin: number,
    period: number,
    offsets: readonly number[],
  ): void => {
    descriptors.push({ kind: "periodic", startAt, origin, period, offsets });
  };

  for (const component of circuit.components) {
    const p = component.params;
    if (component.kind === "clock" || component.kind === "clock_gen") {
      const frequency = Math.max(1e-6, Number(p.frequency ?? 1000));
      const period = 1 / frequency;
      const duty = Math.max(0, Math.min(1, Number(p.duty ?? 0.5)));
      const delay = Math.max(0, Number(p.delay ?? 0));
      addPeriodic(delay, delay, period, [0, duty * period]);
      continue;
    }

    if (component.kind === "pulse_source" || component.kind === "pulse_gen") {
      const delay = Math.max(0, Number(p.td ?? 0));
      const rise = Math.max(H_MIN, Number(p.tr ?? 1e-6));
      const width = Math.max(0, Number(p.pw ?? 5e-4));
      const fall = Math.max(H_MIN, Number(p.tf ?? 1e-6));
      const period = Number(p.per ?? 1e-3);
      const offsets = [0, rise, rise + width, rise + width + fall];
      if (Number.isFinite(period) && period > 0) {
        addPeriodic(delay, delay, period, offsets);
      } else {
        addFixed(offsets.map((offset) => delay + offset));
      }
      continue;
    }

    if (component.kind !== "signal_gen" || Number(p.enabled ?? 1) === 0) continue;
    // The engine's own parse: a string it does not recognise ("SQUARE",
    // "Sine", a typo) plays as a sine, and PWL corners are read with its
    // parseFloat, sort and 64-point cap.
    const parsed = parseSignalGenParams(p);
    const waveform = parsed.waveform;
    const frequency = Math.max(1e-6, Number(p.frequency ?? 1000));
    const period = 1 / frequency;
    const delay = Math.max(0, Number(p.delay ?? 0));

    if (waveform === "square") {
      const duty = Math.max(0.01, Math.min(0.99, Number(p.duty ?? 0.5)));
      const phaseOffset = ((Number(p.phaseDeg ?? 0) / 360) * period + period) % period;
      addPeriodic(delay, delay - phaseOffset, period, [0, duty * period]);
    } else if (waveform === "pulse") {
      const rise = Math.max(H_MIN, Number(p.tr ?? 1e-6));
      const width = Math.max(0, Number(p.pw ?? 5e-4));
      const fall = Math.max(H_MIN, Number(p.tf ?? 1e-6));
      addPeriodic(delay, delay, period, [0, rise, rise + width, rise + width + fall]);
    } else if (waveform === "ramp") {
      const phaseOffset = ((Number(p.phaseDeg ?? 0) / 360) * period + period) % period;
      addPeriodic(delay, delay - phaseOffset, period, [0]);
    } else if (waveform === "noise") {
      const bucket = 1 / SIGNAL_GEN_NOISE_UPDATE_RATE_HZ;
      addPeriodic(delay, delay, bucket, [0]);
    } else if (waveform === "pwl") {
      // No delay gate: the engine plays PWL corners at absolute time and
      // ignores `delay`, so a corner before the delay is still an edge.
      addFixed(parsed.pwlPoints.map((point) => point.t));
    } else {
      // Continuous waveforms still have one authored start boundary.
      addFixed([delay]);
    }
  }
  return descriptors;
}

/** Earliest source event after `now`. sim.worker.ts nextSourceEventTime(), verbatim. */
function nextSourceEventTime(
  now: number,
  descriptors: readonly SourceEventDescriptor[],
): number | null {
  let next: number | null = null;
  const consider = (candidate: number | null): void => {
    if (candidate !== null && candidate > now && (next === null || candidate < next)) {
      next = candidate;
    }
  };

  for (const descriptor of descriptors) {
    if (descriptor.startAt !== null && now < descriptor.startAt) {
      consider(descriptor.startAt);
      continue;
    }
    if (descriptor.kind === "periodic") {
      consider(nextPeriodicEvent(now, descriptor.origin, descriptor.period, descriptor.offsets));
      continue;
    }
    for (const eventTime of descriptor.times) {
      if (eventTime > now) {
        consider(eventTime);
        break;
      }
    }
  }
  return next;
}

/** One accepted step's committed readout, delivered to HeadlessRunOptions.onSample. */
export interface HeadlessSample {
  /** Simulation time (s) at the END of the accepted step. */
  simTime: number;
  /**
   * The accepted step interval (s) that produced this sample. It VARIES, so
   * samples are not a uniform grid and must not be treated as one (no fixed
   * sample rate, no FFT without resampling). Under adaptive control it
   * changes step to step. With `adaptive: false` it is `fixedStepS` only
   * away from events: a step lands 10 ns before each source edge, corner or
   * PWL point, a 10 ns step crosses it, and the grid restarts there (a 1 kHz
   * clock at 10 us steps takes 9.99 us and 10 ns at each edge). Callers
   * wanting a uniform grid should use the `.tran` runner in brownout/spice,
   * which owns SPICE's fixed-grid semantics.
   */
  h: number;
  /** Net id -> volts. A per-sample copy: the engine's own map is live and is mutated in place by the next step. */
  netV: Record<string, number>;
  /** Component id -> amps (pin0 -> pin1 through the element). Copied for the same reason as netV. */
  elementI: Record<string, number>;
}

/** Committed engine readout at an instant, via HeadlessRunner.snapshot(). */
export interface HeadlessSnapshot {
  simTime: number;
  /** False when the last solve did not converge; every readout below is then the last TRUSTED state, not the failed iterate. */
  converged: boolean;
  netV: Record<string, number>;
  elementI: Record<string, number>;
  digitalState: Record<string, number>;
  failures: Record<string, SimFailure>;
  /** Latched engine advisories (SimEngine.getWarnings()); inert to the solve. */
  warnings: Record<string, SimWarning>;
  solver: SolverDiagnosticsSnapshot;
}

export interface HeadlessRunnerOptions {
  /**
   * Integration method for the initial load. Must be settled BEFORE
   * engine.load(): the load seed solve and the first step's backward-Euler
   * anchoring read the method already in force (sim.worker.ts load handler).
   * Default "be", matching that handler's default.
   */
  integrationMethod?: "be" | "trap";
  /**
   * Which engine drives load()/run(). Default "classic" — every existing
   * caller is unaffected; the classic path below is byte-for-byte what it
   * was before this option existed.
   *
   * "core" attempts the Rust/WASM core (brownout/core) first and falls back
   * to classic when: `initCore()` hasn't been awaited yet
   * ("core-not-initialized"), the live instance is poisoned by an earlier
   * trap and hasn't been re-initialised yet ("core-poisoned" — a fresh
   * `CoreEngine` is built automatically the next time `initCore()` succeeds
   * again, so this is transient, not permanent), or the circuit uses a kind
   * not in `coreSupportedKinds` (`"unsupported-kind:<kind>"`, one per kind),
   * or the core itself rejects the circuit's topology (`"topology:<message>"`)
   * or a malformed parameter (`compileCircuit`'s own `"malformed:<id>:<what>"`).
   * See `activeEngine` / `engineFallbackReasons` to observe which happened.
   * Once a circuit is running on the core, `"core"` keeps it there: a failed
   * advance is reported on the run's result (`hitMinStep`), and a trap is
   * thrown to the caller.
   *
   * "auto" chooses at each `load()` exactly as "core" does, and also covers
   * the run: when the core fails part-way (an advance that does not converge,
   * a livelock, a trap or a poisoned instance), the runner finishes on
   * classic and records why. No state is carried between the two engines, so
   * the replay starts from the load: classic loads every circuit this runner
   * gave the core since classic last ran, runs again every `run()` the core
   * completed, then the run that failed, whose samples all come from classic.
   * The readings afterwards are the ones a classic runner gives for the same
   * calls. `engineFallbackReasons` is then the failure
   * (`"mid-run:solve-failed:<message>"`, `"mid-run:livelock:<message>"`,
   * `"core-trap"`, ...) followed by `"restarted-from-load"`, and by
   * `"replay-incomplete"` if classic stopped early in one of the earlier
   * runs. The next `load()` chooses again. To make this possible the runner
   * keeps a copy of each circuit loaded on the core and the options of each
   * run, and it holds a run's samples until the core has finished that run,
   * so `onSample` is called at the end of a core run rather than during it.
   * A circuit is replayed as `load()` took it: an edit made in place and
   * never loaded reached neither engine.
   */
  engine?: "classic" | "core" | "auto";
  /**
   * Overrides which kinds `engine: "core"` and `"auto"` are allowed to route to the core.
   * Defaults to `CORE_SUPPORTED_KINDS`, the kinds that have cleared
   * brownout's accuracy gates (kinds.ts). This option exists for
   * the dual-engine equivalence harness and this package's own tests: passing
   * a wider set (e.g. `CORE_IMPLEMENTED_KINDS`) routes kinds that have NOT
   * cleared those gates to the core, which is exactly what proving them out
   * requires but is never something a normal host should do.
   */
  coreSupportedKinds?: ReadonlySet<string>;
}

export interface HeadlessRunOptions {
  /** Simulated seconds to advance. Added to whatever time already elapsed — successive run() calls continue. */
  durationS: number;
  /**
   * Error-controlled adaptive stepping (default true). False steps at
   * `fixedStepS` and disables trial replay entirely, the equivalent of the
   * worker's manualStepH path: no controller overrides the caller's step,
   * but the source step ceiling still caps it, and steps still land on
   * source and device events, so the samples around an event are not
   * evenly spaced (see HeadlessSample.h). On the core this is a uniform grid
   * of `fixedStepS` (never coarser than the source step ceiling) with one
   * sample per grid point; `adaptive: true` there is one advance, one sample.
   */
  adaptive?: boolean;
  /** Step size (s) when `adaptive` is false. Required in that mode; clamped to [H_MIN, H_MAX]. */
  fixedStepS?: number;
  /**
   * Re-select the integration method for this run. Only legal at t=0: the
   * method must precede the load seed, so applying it re-loads the circuit
   * (see HeadlessRunnerOptions.integrationMethod). Throws if the engine has
   * already stepped and the request differs from the method in force.
   */
  integrationMethod?: "be" | "trap";
  /** Called once per ACCEPTED step, in time order. Rejected trials never surface. */
  onSample?: (sample: HeadlessSample) => void;
  /** Runaway ceiling on attempted steps (default 1e6). */
  maxSteps?: number;
}

export interface HeadlessRunResult {
  /** Simulation time (s) after the run. */
  simTime: number;
  /** Simulated seconds actually covered. Short of durationS only when a stop flag below is set. */
  simulatedS: number;
  acceptedSteps: number;
  /** Trials rolled back for exceeding the error tolerance. Healthy runs have a few; they cost time, not accuracy. */
  rejectedSteps: number;
  /** Trials rolled back because Newton/MNA did not converge. */
  failedSteps: number;
  /**
   * Largest local-error ratio among ACCEPTED steps; 1.0 is the accept
   * threshold, so a run reporting well under 1.0 never had to work for its
   * accuracy. Rejected trials are excluded (they show up in rejectedSteps).
   * Always 0 when errorControlled is false — there was no estimate to report.
   */
  maxLocalErrorRatio: number;
  /**
   * Step size (s) the CONTROLLER finished on — the useful read on how far it
   * opened up. Individual steps may have been smaller than this: the source
   * density floor, an event clamp, and the final partial step all shrink h
   * for one step without moving the controller.
   */
  finalH: number;
  /** True when trial replay was in force. False for MCU circuits and fixed-step runs — maxLocalErrorRatio is then meaningless (0). */
  errorControlled: boolean;
  /**
   * The run stopped early at the H_MIN floor: neither a converged nor an
   * in-tolerance state was reachable. The last trusted state is preserved and
   * simulatedS reports how far it got. This is the honest failure signal —
   * check it.
   */
  hitMinStep: boolean;
  /** The run stopped early on the maxSteps ceiling. */
  hitStepCap: boolean;
}

/** What a runner on "auto" keeps while the core runs, so a failed run can be replayed on classic from the load. */
type CoreHistoryEntry =
  | { readonly kind: "load"; readonly circuit: SimCircuit; readonly integrationMethod: "be" | "trap"; readonly simTime: number }
  | { readonly kind: "run"; readonly options: HeadlessRunOptions };

interface StepAttempt {
  accepted: boolean;
  iters: number;
  errorRatio: number;
  nextFactor: number;
  reason: "accepted" | "nonconverged" | "accuracy";
}

/**
 * A minimal Node host for SimEngine.
 *
 * ```ts
 * const runner = new HeadlessRunner();
 * runner.load(circuit);                       // or a SPICE netlist string
 * runner.run({ durationS: 5e-3, onSample: (s) => console.log(s.simTime) });
 * console.log(runner.snapshot().netV);
 * ```
 */
export class HeadlessRunner {
  private readonly _engine = new SimEngine();
  private _circuit: SimCircuit | null = null;
  /** Mirrors the engine's method so the controller can pick the matching error-order exponent, exactly as the worker mirrors it. */
  private _integrationMethod: "be" | "trap";
  private _adaptiveH = H_INITIAL;
  private _errorReplay = false;
  /** Whether the last error estimate said the circuit is changing (see ACTIVE_H_MAX). */
  private _changing = true;

  private readonly _engineChoice: "classic" | "core" | "auto";
  private readonly _coreSupportedKinds: ReadonlySet<string>;
  /**
   * Created lazily on the first successful core load, then reused across
   * every later load() call — see HeadlessRunnerOptions.engine's doc comment.
   * `_tryLoadCore` drops and replaces it (best-effort dispose(), then a fresh
   * `new CoreEngine()`) the moment `isCurrent()` says it no longer belongs to
   * the live core instance — e.g. a trap poisoned the instance and the host
   * re-initialised — so this field can silently change identity across two
   * `load()` calls; nothing outside `_tryLoadCore` may assume otherwise.
   * Never explicitly disposed by this class on its own initiative: HeadlessRunner
   * has no existing lifecycle/close hook to hang a deterministic dispose() off
   * of (it never needed one for the classic engine), so this relies on
   * CoreEngine's own FinalizationRegistry safety net instead. A host that
   * wants deterministic disposal should own a CoreEngine directly rather than
   * go through this convenience path.
   */
  private _coreEngine: CoreEngine | null = null;
  private _activeEngine: "classic" | "core" = "classic";
  private _engineFallbackReasons: string[] = [];
  /**
   * "auto" only: every load and completed run since the core took over from
   * classic, oldest first. Its first entry is always a load. Empty whenever
   * classic is the active engine: classic's own state is then current, and a
   * replay has nothing to redo.
   */
  private _coreHistory: CoreHistoryEntry[] = [];

  constructor(options?: HeadlessRunnerOptions) {
    this._integrationMethod = options?.integrationMethod ?? "be";
    this._engineChoice = options?.engine ?? "classic";
    this._coreSupportedKinds = options?.coreSupportedKinds ?? CORE_SUPPORTED_KINDS;
  }

  /**
   * The hosted classic engine, for readouts this class does not wrap
   * (getServoState, getPtcTripped, dcOperatingPoint, ...). Stepping it
   * directly bypasses the controller and desynchronizes `adaptiveH` from the
   * state it was measured against; use run() to advance time.
   *
   * Throws in core mode: the classic SimEngine held here is never loaded and
   * never runs while `activeEngine === "core"` (see load()), so returning it
   * would silently hand back a stale, unloaded engine instead of the one
   * actually simulating the circuit. Use `readout` for the active engine's
   * read surface, or construct/hold your own CoreEngine for core-specific APIs.
   */
  get engine(): SimEngine {
    if (this._activeEngine === "core") {
      throw new Error(
        'HeadlessRunner.engine: the classic SimEngine is not running this circuit (engine: "core" ' +
          "landed on the core — see activeEngine). Use `readout` for the active engine's read " +
          "surface instead.",
      );
    }
    return this._engine;
  }

  /** Which engine actually loaded the current circuit. "classic" until a "core" load succeeds; reverts to "classic" on any fallback. */
  get activeEngine(): "classic" | "core" {
    return this._activeEngine;
  }

  /**
   * Why load() fell back to classic, in the vocabulary select.ts/compile.ts/
   * CoreUnsupportedError all share (`"core-not-initialized"`,
   * `"core-poisoned"`, `"unsupported-kind:<kind>"`, `"malformed:<id>:<what>"`,
   * `"topology:<message>"`). Empty when `engine: "core"` landed on the core,
   * or when `engine` is `"classic"`/absent (no fallback was ever attempted).
   * Under `"auto"` it is also why a run() left the core part-way: the
   * failure, then `"restarted-from-load"` (see HeadlessRunnerOptions.engine).
   */
  get engineFallbackReasons(): readonly string[] {
    return this._engineFallbackReasons;
  }

  /** The active engine's read-only surface — works in both classic and core mode, unlike `engine` (classic-only). */
  get readout(): EngineReadout {
    return this._activeEngine === "core" ? (this._coreEngine as CoreEngine) : this._engine;
  }

  /** Simulation time (s), from whichever engine is active. */
  get simTime(): number {
    return this.readout.simTime;
  }

  /**
   * Load a circuit, or a SPICE netlist to parse into one. Throws
   * SpiceParseError (line + card attached) for a netlist outside the
   * documented subset.
   *
   * The netlist path uses the parser's converted circuit only; netlist
   * ANALYSES (.tran/.ac/.op) are ignored here, because the whole point of
   * this runner is that the caller drives the loop. Use runSpice() from
   * brownout/spice to execute directives as written.
   */
  load(source: SimCircuit | string): void {
    const circuit = typeof source === "string" ? parseSpiceNetlist(source).circuit : source;

    // Carry the runner's OBSERVABLE clock across an engine switch. Classic
    // SimEngine and the CoreEngine facade each keep their own independent
    // clock (engine.ts's own note: a live-edit reload must keep the facade's
    // simTime moving forward exactly like classic does), and neither engine
    // knows the other exists. Without this, a reload that changes
    // `activeEngine` — a live edit that adds/removes an unsupported kind —
    // would make `this.simTime` jump to whatever the OTHER engine's clock
    // last read (0, on its first use), rather than continuing from where the
    // circuit actually was. There is no prior time to preserve on the very
    // FIRST load (`this._circuit` is still null), and when the active engine
    // does NOT change this reduces to a no-op — a classic-only caller (or a
    // core-mode caller staying on the same engine) sees exactly what it saw
    // before this existed.
    const previousEngine = this._circuit !== null ? this._activeEngine : null;
    let carryTime: number | undefined;
    if (previousEngine !== null) {
      try {
        carryTime = this.readout.simTime;
      } catch {
        // The previously active engine can no longer report its own time —
        // e.g. the core was poisoned by a trap (during an earlier run()) since
        // the last load(). There is nothing trustworthy to carry over; the
        // load attempt below sorts out fallback/recovery on its own account
        // (see _tryLoadCore's "core-poisoned" reason), so this just skips the
        // seed rather than letting an unrelated read throw out of load().
        carryTime = undefined;
      }
    }

    if (this._engineChoice !== "classic") {
      const fallbackReasons = this._tryLoadCore(circuit, previousEngine === "classic" ? carryTime : undefined);
      if (fallbackReasons === null) {
        if (this._engineChoice === "auto") {
          // A copy: a host may edit its circuit object in place and load it
          // again, and the replay must load what the core was given each time.
          const entry: CoreHistoryEntry = {
            kind: "load",
            circuit: structuredClone(circuit),
            integrationMethod: this._integrationMethod,
            simTime: (this._coreEngine as CoreEngine).simTime,
          };
          // From classic (or a first load) the history starts here; a reload
          // on the core, which carries the core's state, continues it.
          if (previousEngine === "core") this._coreHistory.push(entry);
          else this._coreHistory = [entry];
        }
        // Landed on the core. Deliberately do NOT touch the classic engine
        // (or _adaptiveH/_changing/_errorReplay, which are meaningless for
        // the core) — a caller that later switches back to "classic" must
        // not be surprised by leftover state from a run that never actually
        // happened on the classic engine.
        this._circuit = circuit;
        this._activeEngine = "core";
        this._engineFallbackReasons = [];
        return;
      }
      this._engineFallbackReasons = fallbackReasons;
    } else {
      this._engineFallbackReasons = [];
    }
    // Classic runs from here, so its own state is current: nothing to replay.
    this._coreHistory = [];

    // Classic path: byte-for-byte what this method was before engine
    // selection existed, and the fallback path when "core" mode couldn't
    // take the circuit — except for seeding the classic clock when the
    // PREVIOUS load actually ran on the core (see the carryTime comment
    // above). That seed must land BEFORE load(), which is what a caller
    // switching engines by hand would do too.
    this._activeEngine = "classic";
    if (previousEngine === "core" && carryTime !== undefined) {
      this._engine.simTime = carryTime;
    }
    // Method before load: the seed solve reads it (see the option's doc).
    this._engine.setIntegrationMethod(this._integrationMethod);
    this._engine.load(circuit);
    this._circuit = circuit;
    this._adaptiveH = H_INITIAL;
    this._changing = true;
    this._errorReplay = circuitSupportsStepReplay(circuit);
  }

  /**
   * Attempts to load `circuit` onto the core engine, creating one (or
   * replacing a superseded one — see below) as needed. Returns `null` on
   * success, or the fallback reasons to report (and to fall back to classic
   * over) otherwise. Only a genuinely unexpected error (anything but
   * `CoreUnsupportedError`) propagates, per HeadlessRunnerOptions.engine's
   * doc comment.
   *
   * `seedSimTime`, when given (a "classic to core" engine switch — see
   * load()), is passed through to `CoreEngine.load()`'s own `options.simTime`
   * so the newly-core circuit starts at the runner's carried-over time
   * instead of wherever this engine's own clock last stood.
   *
   * Always passes `this._integrationMethod` through too: W1b gave the core
   * real dynamics (capacitors, stepped partitions choosing BE/trapezoidal via
   * the program's INTEGRATION option — load-time only, like classic's own
   * seed), so a caller's requested method must reach the compiled program
   * here, not just the classic engine's `setIntegrationMethod`.
   */
  private _tryLoadCore(circuit: SimCircuit, seedSimTime?: number): string[] | null {
    if (!coreReady()) {
      // Tell "nobody has awaited initCore() yet" apart from "it WAS ready
      // and a trap poisoned it" — the two need different host responses (the
      // latter implies initCore() should be called AGAIN, not for the first
      // time).
      return [coreStatus() === "poisoned" ? "core-poisoned" : "core-not-initialized"];
    }
    const unsupported = unsupportedCoreKindReasons(circuit, this._coreSupportedKinds);
    if (unsupported.length > 0) return unsupported;
    try {
      // A cached engine from an earlier, now-superseded core instance (a trap
      // poisoned it and the host re-initialised — core/README.md, "Traps")
      // can never load again: its handle table belongs to a generation that
      // no longer exists, so every call on it throws CoreStaleInstanceError
      // (see CoreEngine's own generation check). Drop it and build a fresh
      // one instead of letting that become a permanent failure for this
      // runner.
      if (this._coreEngine && !this._coreEngine.isCurrent()) {
        try {
          this._coreEngine.dispose();
        } catch {
          // Best-effort — see CoreEngine.dispose()'s own doc comment.
        }
        this._coreEngine = null;
      }
      this._coreEngine ??= new CoreEngine();
      this._coreEngine.load(circuit, undefined, {
        integrationMethod: this._integrationMethod,
        ...(seedSimTime !== undefined ? { simTime: seedSimTime } : {}),
      });
      return null;
    } catch (err) {
      if (err instanceof CoreUnsupportedError) return [...err.reasons];
      throw err;
    }
  }

  /** Net id carrying a component pin, or undefined if the pin is unwired/unknown — from whichever engine is active. */
  netIdFor(componentId: string, pinId: string): string | undefined {
    return this.readout.getNetIdForPin(componentId, pinId);
  }

  /** Committed readout at the current instant, from whichever engine is active. All maps are copies; the engines' own are live. */
  snapshot(): HeadlessSnapshot {
    if (this._activeEngine === "core") {
      const readout = this.readout;
      return {
        simTime: readout.simTime,
        converged: readout.lastConverged,
        netV: { ...readout.getNetV() },
        elementI: { ...readout.getElementI() },
        digitalState: { ...readout.digitalState },
        failures: readout.getFailures(),
        warnings: readout.getWarnings(),
        // The core has no Newton/MNA loop in W0 — these describe the
        // CLASSIC solver and have no core meaning yet. lastConverged is the
        // one field that still means something (true after a successful
        // load()/advance()); every numeric field reads as its neutral zero
        // rather than a fabricated non-zero number.
        solver: {
          lastIters: 0,
          lastConverged: readout.lastConverged,
          lastSolveUs: 0,
          lastMatrixSize: 0,
          lastMatrixSingular: false,
          lastMatrixIllConditioned: false,
          lastRelativeResidual: 0,
        },
      };
    }
    return {
      simTime: this._engine.simTime,
      converged: this._engine.lastConverged,
      netV: { ...this._engine.getNetV() },
      elementI: { ...this._engine.getElementI() },
      digitalState: { ...this._engine.digitalState },
      failures: this._engine.getFailures(),
      warnings: this._engine.getWarnings(),
      solver: {
        lastIters: this._engine.lastIters,
        lastConverged: this._engine.lastConverged,
        lastSolveUs: this._engine.lastSolveUs,
        lastMatrixSize: this._engine.lastMatrixSize,
        lastMatrixSingular: this._engine.lastMatrixSingular,
        lastMatrixIllConditioned: this._engine.lastMatrixIllConditioned,
        lastRelativeResidual: this._engine.lastRelativeResidual,
      },
    };
  }

  /**
   * Attempt one physical interval of length h.
   *
   * sim.worker.ts attemptStep(), with its solveUs accounting dropped (nothing
   * here reports per-batch throughput). Unchecked intervals take no snapshot
   * at all: SimEngine publishes no state from a failed solve and performs its
   * own rollback for a failed NE555 split, so the expensive deep snapshot is
   * reserved for the replay path that actually rewinds.
   */
  private attemptStep(h: number, useErrorEstimate: boolean): StepAttempt {
    const engine = this._engine;

    if (!useErrorEstimate || h <= H_MIN * 2.01) {
      engine.step(h);
      if (!engine.lastConverged) {
        return {
          accepted: false,
          iters: engine.lastIters,
          errorRatio: Number.POSITIVE_INFINITY,
          nextFactor: 0.25,
          reason: "nonconverged",
        };
      }
      // Newton effort as a proxy for step difficulty: the only growth signal
      // available without a trial to compare against.
      const factor = engine.lastIters > 15 ? 0.7 : engine.lastIters <= 3 ? 1.5 : 1;
      return { accepted: true, iters: engine.lastIters, errorRatio: 0, nextFactor: factor, reason: "accepted" };
    }

    const before = engine.saveState();
    let iters = 0;

    const rollback = (reason: "nonconverged"): StepAttempt => {
      engine.restoreState(before);
      return { accepted: false, iters, errorRatio: Number.POSITIVE_INFINITY, nextFactor: 0.25, reason };
    };

    // Coarse leg: one h step.
    engine.step(h);
    iters = Math.max(iters, engine.lastIters);
    if (!engine.lastConverged) return rollback("nonconverged");
    const coarse = engine.captureErrorState();
    engine.restoreState(before);

    // Refined leg: two h/2 steps. This is the state that survives acceptance.
    engine.step(h / 2);
    iters = Math.max(iters, engine.lastIters);
    if (!engine.lastConverged) return rollback("nonconverged");

    // An accepted step consumes the engine's pending-BE anchor. Without this
    // re-arm the first half-step would eat the flag and the second half would
    // stamp trapezoidal from mid-jump history, so the two legs would be
    // integrated by DIFFERENT methods and the "error" would be method bias.
    if (this._integrationMethod === "trap" && before.integrationBeNextStep) {
      engine.markDiscontinuity();
    }

    engine.step(h / 2);
    iters = Math.max(iters, engine.lastIters);
    if (!engine.lastConverged) return rollback("nonconverged");

    const refined = engine.captureErrorState();
    const estimate = estimateStepError(coarse, refined);
    // Trapezoidal is second order, so it grows on the cube-root exponent; the
    // BE call is the historical order-1 default.
    const factor = nextStepFactor(estimate.ratio, this._integrationMethod === "trap" ? 2 : 1);
    if (estimate.ratio > 1) {
      engine.restoreState(before);
      return { accepted: false, iters, errorRatio: estimate.ratio, nextFactor: factor, reason: "accuracy" };
    }

    return { accepted: true, iters, errorRatio: estimate.ratio, nextFactor: factor, reason: "accepted" };
  }

  /**
   * The source events this run lands its steps on. A subclass returning an
   * empty list steps exactly the path from before source events existed,
   * which is how the regression tests compute their reference on the
   * platform under test instead of against numbers captured elsewhere.
   */
  protected _sourceEvents(circuit: SimCircuit): SourceEventDescriptor[] {
    return sourceEventDescriptors(circuit);
  }

  /**
   * Apply a run's integrationMethod request. See
   * HeadlessRunOptions.integrationMethod for why t=0 is the only legal point.
   * The legality rule is the SAME regardless of which engine is active —
   * `this.simTime` reads whichever one is (see the `simTime` getter). W1b
   * gave the core real dynamics (capacitors; stepped partitions pick BE or
   * trapezoidal from the program's INTEGRATION option), so — unlike W0/W1a,
   * when the core had nothing to seed and this only recorded the request —
   * core mode now needs the SAME re-seed classic gets: load() again, whose
   * `_tryLoadCore` bakes `_integrationMethod` into the program (see its own
   * doc comment). Lossless at t=0: nothing has been stepped yet on either
   * engine.
   */
  private applyIntegrationMethod(method: "be" | "trap", circuit: SimCircuit): void {
    if (method === this._integrationMethod) return;
    if (this.simTime !== 0) {
      throw new Error(
        `HeadlessRunner: cannot switch integrationMethod to "${method}" at ` +
        `t=${String(this.simTime)} s. The method must be in force before the ` +
        "load seed solve and the first step's backward-Euler anchoring, so switching " +
        "mid-run would re-seed the circuit and discard the state already computed. " +
        "Select it in the constructor, or on a run that starts at t=0.",
      );
    }
    this._integrationMethod = method;
    if (this._activeEngine === "core") {
      // Re-run engine selection rather than reloading the core directly, so
      // the program is rebuilt under the new method through the one path
      // that also handles a core that cannot take it (load() then falls
      // back to classic, seeded under the new method, with the reasons
      // recorded). run() branches on activeEngine only after this call.
      this.load(circuit);
      return;
    }
    this._engine.setIntegrationMethod(method);
    // Re-seed under the new method. Lossless: nothing has been stepped yet.
    this._engine.load(circuit);
    this._adaptiveH = H_INITIAL;
    this._changing = true;
  }

  /** Advance the simulation by `durationS` seconds. Throws only on misuse; solver trouble is reported on the result. */
  run(options: HeadlessRunOptions): HeadlessRunResult {
    const circuit = this._circuit;
    if (!circuit) {
      throw new Error("HeadlessRunner: run() before load() — there is no circuit to step.");
    }
    const { durationS, onSample } = options;
    if (!Number.isFinite(durationS) || durationS <= 0) {
      throw new Error(`HeadlessRunner: durationS must be a positive finite number of seconds, got ${String(durationS)}.`);
    }

    // Applied before the engine branch: in core mode it can reload, and a
    // reload that falls back to classic changes which branch runs.
    if (options.integrationMethod) this.applyIntegrationMethod(options.integrationMethod, circuit);

    if (this._activeEngine === "core") {
      // adaptive:true ignores fixedStepS/maxSteps on the core (one advance,
      // one sample); adaptive:false samples a fixed grid (_runCoreGrid). A
      // caller's misuse must still be caught the same way — this is the SAME check the
      // classic path below runs, duplicated rather than factored out, so the
      // classic block stays byte-for-byte what it was before "core" mode
      // existed (diff it against `git show HEAD:src/host/headless.ts`).
      if (!(options.adaptive ?? true)) {
        const requested = options.fixedStepS;
        if (requested === undefined || !Number.isFinite(requested) || requested <= 0) {
          throw new Error(
            "HeadlessRunner: run({ adaptive: false }) requires a positive fixedStepS — with the " +
            "controller off there is nothing to choose a step size.",
          );
        }
      }
      if (this._engineChoice === "auto") return this._runCoreOrReplay(circuit, options);
      if (!(options.adaptive ?? true)) {
        // Fixed-grid sampling (W2, D15 (a)): the grid classic would step on,
        // advanced to point by point on the core.
        return this._runCoreGrid(
          this._coreEngine as CoreEngine,
          durationS,
          Math.min(Math.max(H_MIN, Math.min(H_MAX, options.fixedStepS!)), sourceStepLimit(circuit)),
          options.maxSteps ?? DEFAULT_MAX_STEPS,
          onSample,
        );
      }
      return this._runCore(this._coreEngine as CoreEngine, durationS, onSample);
    }

    const adaptive = options.adaptive ?? true;
    let fixedH: number | null = null;
    if (!adaptive) {
      const requested = options.fixedStepS;
      if (requested === undefined || !Number.isFinite(requested) || requested <= 0) {
        throw new Error(
          "HeadlessRunner: run({ adaptive: false }) requires a positive fixedStepS — with the " +
          "controller off there is nothing to choose a step size.",
        );
      }
      fixedH = Math.max(H_MIN, Math.min(H_MAX, requested));
    }

    const engine = this._engine;
    const maxSteps = options.maxSteps ?? DEFAULT_MAX_STEPS;
    // Error control needs BOTH a rewindable circuit and no caller-pinned step.
    const errorControlEnabled = this._errorReplay && fixedH === null;
    // The source floor and source events bind even in fixed-step mode: a
    // caller-chosen step that aliases the source or steps over its edges is
    // a wrong answer, not a caller preference.
    // The changing-circuit ceiling does not: it tunes the controller's choice.
    const sourceMaxH = sourceStepLimit(circuit);
    const sourceEvents = this._sourceEvents(circuit);
    const order = this._integrationMethod === "trap" ? 2 : 1;

    let simmed = 0;
    let stepCount = 0;
    let acceptedSteps = 0;
    let rejectedSteps = 0;
    let failedSteps = 0;
    let maxLocalErrorRatio = 0;
    let hitMinStep = false;
    let hitStepCap = false;

    // Float slack so a duration that is an exact multiple of the final step
    // does not spin one extra ~1e-16 s step. Worker's batchTolerance.
    const tolerance = Math.max(1e-15, durationS * 1e-12);

    while (simmed < durationS - tolerance) {
      if (stepCount >= maxSteps) {
        hitStepCap = true;
        break;
      }
      stepCount += 1;

      const remaining = durationS - simmed;
      const controllerH = fixedH ?? this._adaptiveH;
      const circuitMaxH = Math.min(
        !this._errorReplay ? MCU_H_MAX : errorControlEnabled && this._changing ? ACTIVE_H_MAX : H_MAX,
        sourceMaxH,
      );
      let h = Math.min(controllerH, circuitMaxH, remaining);
      let eventClamped = false;

      // Land the step boundary exactly ON a scheduled device event (an hcsr04
      // ECHO edge) instead of aliasing past it. The event can be far finer or
      // far coarser than the controller's h, so the clamp lives out here in
      // the loop rather than inside the engine's step.
      const nextEventTime = engine.nextScheduledEventTime();
      if (nextEventTime !== null) {
        const untilEvent = nextEventTime - engine.simTime;
        // <=, not <: a step landing EXACTLY on the event still counts as
        // clamped, or the trap re-anchor below is skipped for the step that
        // departs the discontinuity.
        if (untilEvent <= h) {
          h = Math.max(H_MIN, untilEvent);
          eventClamped = true;
        }
      }
      const nextSourceEvent = nextSourceEventTime(engine.simTime, sourceEvents);
      if (nextSourceEvent !== null) {
        const untilEvent = nextSourceEvent - engine.simTime;
        // The epsilon: a square's edges sit on multiples of its P/10 ceiling
        // at 50% duty, and a step starting P/10 before one measured it a
        // rounding error beyond h, so it ended on the edge and applied the
        // new level across the whole step.
        if (untilEvent <= h + sourceEdgeEpsilon(nextSourceEvent)) {
          // Land one minimum step before the event, then cross it in a 10 ns
          // guard step: the source is sampled at a step's end, so only that
          // guard step carries the new level early.
          h = untilEvent > H_MIN * 1.5 ? untilEvent - H_MIN : Math.max(H_MIN, untilEvent);
          eventClamped = true;
        }
      }
      // A TRIG fall detected mid-step schedules an event with no advance
      // warning, so every step is capped under the fixed rise delay whenever
      // an hcsr04 is present. See SimEngine.hcsr04MaxStepH().
      const hcsr04Cap = engine.hcsr04MaxStepH();
      if (hcsr04Cap !== null) h = Math.min(h, hcsr04Cap);

      const attempt = this.attemptStep(h, errorControlEnabled);

      if (!attempt.accepted) {
        this._adaptiveH = Math.max(H_MIN, h * attempt.nextFactor);
        this._changing = true;
        if (attempt.reason === "nonconverged") failedSteps += 1;
        else rejectedSteps += 1;

        // At the floor there is no smaller step to retry with. Stop and keep
        // the last trusted solution rather than accepting a bad state.
        if (h <= H_MIN * 1.01) {
          hitMinStep = true;
          break;
        }
        continue;
      }

      if (fixedH === null) {
        const proposed = h * attempt.nextFactor;
        // An event clamp shrinks h for THIS step only. Without this floor the
        // controller would mistake the clamp for a difficulty signal and crawl
        // afterwards; the pre-clamp controller value is the honest carry-over.
        const unclampedController = eventClamped ? Math.min(controllerH, circuitMaxH) : 0;
        this._adaptiveH = Math.max(H_MIN, Math.min(circuitMaxH, Math.max(proposed, unclampedController)));
        // Steps at the floor skip the estimate (errorRatio 0), which says
        // nothing about whether the circuit is changing.
        if (errorControlEnabled && h > H_MIN * 2.01) {
          this._changing = attempt.errorRatio * (H_MAX / h) ** (order + 1) > QUIET_FRACTION;
        }
      }

      acceptedSteps += 1;
      simmed += h;
      // Accepted steps only: a rejected trial was rolled back and contributed
      // nothing to the trajectory, so folding its (over-tolerance) ratio in
      // here would describe a state that was never delivered. Rejections are
      // reported by rejectedSteps instead. Diverges from sim.worker.ts, which
      // tracks the max over every finite-ratio ATTEMPT to drive its solver
      // health indicator — a different question than "how good is this run".
      maxLocalErrorRatio = Math.max(maxLocalErrorRatio, attempt.errorRatio);

      // Trap mode: the next step crosses or departs a discontinuity, so force
      // it onto backward Euler. Inert in BE mode, so it is not touched there.
      if (this._integrationMethod === "trap" && eventClamped) {
        engine.markDiscontinuity();
      }

      if (onSample) {
        onSample({
          simTime: engine.simTime,
          h,
          netV: { ...engine.getNetV() },
          elementI: { ...engine.getElementI() },
        });
      }
    }

    return {
      simTime: engine.simTime,
      simulatedS: simmed,
      acceptedSteps,
      rejectedSteps,
      failedSteps,
      maxLocalErrorRatio,
      finalH: fixedH ?? this._adaptiveH,
      errorControlled: errorControlEnabled,
      hitMinStep,
      hitStepCap,
    };
  }

  /**
   * run()'s core path under `engine: "auto"` (ruling R116): the same two core
   * runs as under "core", and a replay on classic when the core fails in
   * one. A failure is an advance that did not converge (`hitMinStep` from
   * either core run) or a thrown core failure (a trap, a poisoned or stale
   * instance, a load code); a caller bug (a RangeError) still surfaces.
   *
   * The run's samples are held until the core has finished it, so a failed
   * run delivers classic's samples only, each once.
   */
  private _runCoreOrReplay(circuit: SimCircuit, options: HeadlessRunOptions): HeadlessRunResult {
    const engine = this._coreEngine as CoreEngine;
    const { durationS, onSample } = options;
    const held: HeadlessSample[] = [];
    const hold = onSample ? (sample: HeadlessSample): void => void held.push(sample) : undefined;
    let reasons: readonly string[];
    try {
      const result = (options.adaptive ?? true)
        ? this._runCore(engine, durationS, hold)
        : this._runCoreGrid(
            engine,
            durationS,
            Math.min(Math.max(H_MIN, Math.min(H_MAX, options.fixedStepS!)), sourceStepLimit(circuit)),
            options.maxSteps ?? DEFAULT_MAX_STEPS,
            hold,
          );
      if (!result.hitMinStep) {
        if (onSample) for (const sample of held) onSample(sample);
        const { onSample: _unused, ...replayable } = options;
        this._coreHistory.push({ kind: "run", options: replayable });
        return result;
      }
      reasons = [coreAdvanceFailureReason(engine.lastAdvanceFailure)];
    } catch (err) {
      const mapped = coreFailureReasons(err);
      if (mapped === null) throw err;
      reasons = mapped;
    }
    this._replayOnClassic(reasons);
    // Classic is the active engine now: this is its own run of the failed span.
    return this.run(options);
  }

  /**
   * Puts classic where it would be had every call in `_coreHistory` gone to
   * it: each load (classic's own load carries its state from one to the
   * next), each completed run, in order. No state crosses from the core (W2
   * design D9b (a)); what the core computed is dropped.
   *
   * The first load is seeded with the time the core was given for it, the
   * same seed load() gives classic on any other switch from the core.
   */
  private _replayOnClassic(reasons: readonly string[]): void {
    const history = this._coreHistory;
    this._coreHistory = [];
    this._activeEngine = "classic";
    let incomplete = false;
    let seeded = false;
    for (const entry of history) {
      if (entry.kind === "load") {
        if (!seeded) {
          this._engine.simTime = entry.simTime;
          seeded = true;
        }
        this._integrationMethod = entry.integrationMethod;
        this._engine.setIntegrationMethod(entry.integrationMethod);
        this._engine.load(entry.circuit);
        this._circuit = entry.circuit;
        this._adaptiveH = H_INITIAL;
        this._changing = true;
        this._errorReplay = circuitSupportsStepReplay(entry.circuit);
      } else {
        const replayed = this.run(entry.options);
        if (replayed.hitMinStep || replayed.hitStepCap) incomplete = true;
      }
    }
    this._engineFallbackReasons = [...reasons, "restarted-from-load", ...(incomplete ? ["replay-incomplete"] : [])];
  }

  /**
   * run()'s core-mode path for `adaptive: false` (W2 design 2.7, D15 (a)):
   * Learn's fixed-`dt` sampling. The grid spacing `gridH` is what classic's
   * fixed-step loop steps at away from events (`min(fixedStepS, H_MAX, the
   * source step ceiling)`, computed by the caller), so the core is never
   * sampled coarser than classic is stepped. The core advances to each grid
   * point and `onSample` fires after each, with the solved values at that
   * point (not interpolated from a later one). Classic's extra steps that land
   * on source edges are a step-size artifact and are not reproduced: the core
   * handles those instants inside its own advance.
   *
   * The grid is anchored at the run's start time and each point is computed as
   * `start + k * gridH`, not by accumulating, so it does not drift; the last
   * point is exactly `start + durationS` and may be a short step. `maxSteps`
   * bounds the number of grid points as it bounds classic's attempted steps.
   *
   * Each advance is a clamp on the core's trajectory (D6); the corpus gate at
   * 1/60 s, 1 ms and 10 ms frames is what keeps that within W0.
   *
   * A failed advance stops the run there: the failed point gets no sample
   * (the readouts are still the last trusted instant), `failedSteps` is 1 and
   * `hitMinStep` is set, the same honest-failure signal `_runCore` uses.
   */
  private _runCoreGrid(
    engine: CoreEngine,
    durationS: number,
    gridH: number,
    maxSteps: number,
    onSample: ((sample: HeadlessSample) => void) | undefined,
  ): HeadlessRunResult {
    const startTime = engine.simTime;
    // Same float slack as the classic loop, so an exact multiple of the step
    // does not grow one extra ~1e-16 s point.
    const tolerance = Math.max(1e-15, durationS * 1e-12);
    const wanted = Math.max(1, Math.ceil((durationS - tolerance) / gridH));
    // A NaN cap never trips classic's `stepCount >= maxSteps`, so it is no cap.
    const cap = Number.isNaN(maxSteps) ? Number.POSITIVE_INFINITY : maxSteps;
    const points = Math.min(wanted, cap);
    const hitStepCap = wanted > cap;

    let accepted = 0;
    let failed = false;
    let previous = startTime;
    for (let k = 1; k <= points; k++) {
      const target = k === wanted ? startTime + durationS : startTime + k * gridH;
      engine.advance(target);
      if (!engine.lastConverged) {
        failed = true;
        break;
      }
      accepted += 1;
      if (onSample) {
        onSample({
          simTime: engine.simTime,
          h: engine.simTime - previous,
          netV: { ...engine.getNetV() },
          elementI: { ...engine.getElementI() },
        });
      }
      previous = engine.simTime;
    }

    const simulatedS = !failed && !hitStepCap ? durationS : engine.simTime - startTime;
    return {
      simTime: engine.simTime,
      simulatedS,
      acceptedSteps: accepted,
      rejectedSteps: 0,
      failedSteps: failed ? 1 : 0,
      maxLocalErrorRatio: 0,
      finalH: gridH,
      errorControlled: false,
      hitMinStep: failed,
      hitStepCap,
    };
  }

  /**
   * run()'s core-mode path. The core has no step loop in W0/W1a —
   * bo_engine_advance jumps straight to tEnd (core/README.md, "W0 kernel
   * semantics" #6: "Everything in W0 is static, so advancing only moves
   * time") — so there is nothing here for adaptive/fixedStepS/maxSteps to
   * control, and integrationMethod has already been validated/recorded (not
   * applied — the core has no dynamics to seed) by run() before this is
   * called; run() validates all of them (misuse is still caught) but this
   * method ignores every one of them. (`adaptive: false` is `_runCoreGrid`'s.)
   *
   * NON-CONVERGENCE. `CoreEngine.advance` does not throw for a -4/-7 result
   * (core/README.md, "A failed advance") — it sets `lastConverged = false`
   * and reads `simTime` back as however far Rust actually got. This method's
   * own doc-inherited contract ("Throws only on misuse; solver trouble is
   * reported on the result", run()'s doc comment) therefore holds here too:
   * a non-converged advance is reported via `hitMinStep`/`simulatedS`, the
   * SAME honest-failure signal the classic step loop above uses when it hits
   * the H_MIN floor, rather than propagating a `CoreLoadError` out of run().
   */
  private _runCore(
    engine: CoreEngine,
    durationS: number,
    onSample: ((sample: HeadlessSample) => void) | undefined,
  ): HeadlessRunResult {
    const startTime = engine.simTime;
    engine.advance(startTime + durationS);
    const converged = engine.lastConverged;
    // On non-convergence `engine.simTime` is however far Rust actually got
    // (the last instant it completed), not `startTime + durationS`, so the
    // gap between them IS how much of the requested duration was actually
    // simulated.
    const simulatedS = converged ? durationS : engine.simTime - startTime;

    if (onSample) {
      // No per-step sample grid exists in the core in W0/W1a (one jump, not a
      // step loop): onSample fires exactly ONCE, at the run's end, with
      // h = simulatedS (== durationS on convergence). W2's acquisition
      // subsystem replaces this with the core's real sample points. The
      // readouts are always the last CONVERGED instant's — CoreEngine.advance
      // never republishes them on a non-convergent call — so this is exactly
      // the "last trusted state is preserved" guarantee hitMinStep's own doc
      // comment describes for the classic path.
      onSample({
        simTime: engine.simTime,
        h: simulatedS,
        netV: { ...engine.getNetV() },
        elementI: { ...engine.getElementI() },
      });
    }

    return {
      simTime: engine.simTime,
      simulatedS,
      acceptedSteps: 0,
      rejectedSteps: 0,
      failedSteps: 0,
      maxLocalErrorRatio: 0,
      finalH: 0,
      errorControlled: false,
      hitMinStep: !converged,
      hitStepCap: false,
    };
  }
}
