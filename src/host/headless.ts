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
 *   - Source-EDGE clamping (worker: sourceEventDescriptors /
 *     nextSourceEventTime), which lands a step boundary immediately before a
 *     waveform discontinuity and crosses it in a 10 ns guard step. Without it
 *     a backward-Euler step that straddles an edge applies the new level
 *     across the whole interval, so edge TIMING carries an error of up to one
 *     step. The source density floor below bounds that (>= 10 steps per
 *     period), which is why the omission costs fidelity rather than
 *     correctness; callers needing exact edge placement want `.tran` via
 *     brownout/spice, which owns SPICE's fixed-grid semantics.
 *     sourceStepLimit() IS ported — see its comment for why it cannot be
 *     treated as an optimization the error estimate subsumes.
 *   - Probe acquisition (uniform sample grid + interpolation). onSample fires
 *     on ACCEPTED STEP BOUNDARIES, which under adaptive control are not
 *     uniformly spaced — see HeadlessSample.
 *
 * MCU circuits are handled exactly as the worker handles them: the external
 * CPU emulators are not snapshot-rewindable, so a circuit containing one gets
 * the conservative controller with no trial replay, capped at MCU_H_MAX.
 */

import { estimateStepError, nextStepFactor } from "../sim/adaptive-step.js";
import { parseSpiceNetlist } from "../sim/engine/spice/netlist.js";
import { SIGNAL_GEN_NOISE_UPDATE_RATE_HZ } from "../sim/engine/waveform.js";
import { SimEngine } from "../sim/engine/sim-engine.js";
import type {
  SimCircuit,
  SimFailure,
  SolverDiagnosticsSnapshot,
} from "../sim/engine/sim-engine.js";

// Adaptive step-size bounds (seconds) — sim.worker.ts values verbatim.
const H_MIN = 1e-8;
const H_MAX = 1e-2;
const H_INITIAL = 1e-8;
const MCU_H_MAX = 1e-4;

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
    const waveform = String(p.waveform ?? "sine");
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

/** One accepted step's committed readout, delivered to HeadlessRunOptions.onSample. */
export interface HeadlessSample {
  /** Simulation time (s) at the END of the accepted step. */
  simTime: number;
  /**
   * The accepted step interval (s) that produced this sample. Under adaptive
   * control this VARIES step to step, so samples are not a uniform grid and
   * must not be treated as one (no fixed sample rate, no FFT without
   * resampling). Callers wanting a uniform grid should either run with
   * `adaptive: false` and a fixed step, or use the `.tran` runner in
   * brownout/spice, which owns SPICE's fixed-grid semantics.
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
}

export interface HeadlessRunOptions {
  /** Simulated seconds to advance. Added to whatever time already elapsed — successive run() calls continue. */
  durationS: number;
  /**
   * Error-controlled adaptive stepping (default true). False pins the step to
   * `fixedStepS` and disables trial replay entirely — the equivalent of the
   * worker's manualStepH path, where a caller-chosen step is honored as
   * given rather than overridden by the controller.
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

  constructor(options?: HeadlessRunnerOptions) {
    this._integrationMethod = options?.integrationMethod ?? "be";
  }

  /**
   * The hosted engine, for readouts this class does not wrap (getServoState,
   * getPtcTripped, dcOperatingPoint, ...). Stepping it directly bypasses the
   * controller and desynchronizes `adaptiveH` from the state it was measured
   * against; use run() to advance time.
   */
  get engine(): SimEngine {
    return this._engine;
  }

  /** Simulation time (s). */
  get simTime(): number {
    return this._engine.simTime;
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
    // Method before load: the seed solve reads it (see the option's doc).
    this._engine.setIntegrationMethod(this._integrationMethod);
    this._engine.load(circuit);
    this._circuit = circuit;
    this._adaptiveH = H_INITIAL;
    this._errorReplay = circuitSupportsStepReplay(circuit);
  }

  /** Net id carrying a component pin, or undefined if the pin is unwired/unknown. */
  netIdFor(componentId: string, pinId: string): string | undefined {
    return this._engine.getNetIdForPin(componentId, pinId);
  }

  /** Committed readout at the current instant. All maps are copies; the engine's own are live. */
  snapshot(): HeadlessSnapshot {
    return {
      simTime: this._engine.simTime,
      converged: this._engine.lastConverged,
      netV: { ...this._engine.getNetV() },
      elementI: { ...this._engine.getElementI() },
      digitalState: { ...this._engine.digitalState },
      failures: this._engine.getFailures(),
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

  /** Apply a run's integrationMethod request. See HeadlessRunOptions.integrationMethod for why t=0 is the only legal point. */
  private applyIntegrationMethod(method: "be" | "trap", circuit: SimCircuit): void {
    if (method === this._integrationMethod) return;
    if (this._engine.simTime !== 0) {
      throw new Error(
        `HeadlessRunner: cannot switch integrationMethod to "${method}" at ` +
        `t=${String(this._engine.simTime)} s. The method must be in force before the ` +
        "load seed solve and the first step's backward-Euler anchoring, so switching " +
        "mid-run would re-seed the circuit and discard the state already computed. " +
        "Select it in the constructor, or on a run that starts at t=0.",
      );
    }
    this._integrationMethod = method;
    this._engine.setIntegrationMethod(method);
    // Re-seed under the new method. Lossless: nothing has been stepped yet.
    this._engine.load(circuit);
    this._adaptiveH = H_INITIAL;
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
    if (options.integrationMethod) this.applyIntegrationMethod(options.integrationMethod, circuit);

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
    // The source floor binds even in fixed-step mode: a caller-chosen step
    // that aliases the source is a wrong answer, not a caller preference.
    const circuitMaxH = Math.min(this._errorReplay ? H_MAX : MCU_H_MAX, sourceStepLimit(circuit));

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
      // A TRIG fall detected mid-step schedules an event with no advance
      // warning, so every step is capped under the fixed rise delay whenever
      // an hcsr04 is present. See SimEngine.hcsr04MaxStepH().
      const hcsr04Cap = engine.hcsr04MaxStepH();
      if (hcsr04Cap !== null) h = Math.min(h, hcsr04Cap);

      const attempt = this.attemptStep(h, errorControlEnabled);

      if (!attempt.accepted) {
        this._adaptiveH = Math.max(H_MIN, h * attempt.nextFactor);
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
}
