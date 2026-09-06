/**
 * Typed message protocol between the sim WebWorker and the main thread.
 *
 * WorkerIn  — messages the main thread sends to the worker.
 * WorkerOut — messages the worker sends back.
 */

import type { Net, SimCircuit, SimFailure } from "./engine/sim-engine.js";

export interface SolverDiagnostics {
  steps: number;
  failedSteps: number;
  /** Converged trial steps rejected because the local transient error was too large. */
  rejectedSteps?: number;
  /** Largest accepted/rejected normalized local error estimate in this batch. */
  maxLocalErrorRatio?: number;
  /** True when the circuit supports rewindable step-doubling error control. */
  errorControlled?: boolean;
  hitStepCap: boolean;
  /**
   * True when the batch yielded because the step loop consumed its wall-clock
   * budget (one frame) before covering the requested sim interval. The final
   * state is still a resolved solution built from accepted steps — this is a
   * throughput signal, not a solver-health failure.
   */
  hitWallBudget?: boolean;
  /** Fraction (0..1] of the requested batch sim-time interval actually simulated. */
  batchCompletion?: number;
  hitMinStep: boolean;
  adaptiveH: number;
  lastIters: number;
  lastSolveUs: number;
  batchSolveUs: number;
  /** End-to-end wall time spent producing this worker batch. */
  batchWallUs?: number;
  /** Fraction of the requested real-time rate achieved while computing this batch. */
  realtimeRatio?: number;
  /** Electrical solve invocations performed while producing this batch. */
  electricalSolves?: number;
  /** Dense MNA factorizations performed while producing this batch. */
  matrixFactorizations?: number;
  /** Exact-matrix LU cache hits after stamping rebuilt the same matrix. */
  matrixFactorizationReuses?: number;
  /** Combinational truth-table evaluations performed in this batch. */
  digitalEvaluations?: number;
  /** Exact digital-input signature cache hits in this batch. */
  digitalEvaluationReuses?: number;
  stepsPerSecond: number;
  matrixSize: number;
  cappedBatches5s: number;
  matrixSingular: boolean;
  matrixIllConditioned: boolean;
  relativeResidual: number;
}

/**
 * Uniform, noise-free simulator-domain samples captured inside one worker batch.
 *
 * `times` and every entry in `values` have identical lengths. Timestamps are
 * strictly increasing within an acquisition and are expressed in simulated
 * seconds. Values between accepted solver states use linear dense output; they
 * are not independent circuit solves. `sampleGridRateHz` is the uniform output
 * grid, while `effectiveSampleRateHz` is conservatively capped by the coarsest
 * accepted solver interval since this acquisition began. Consumers must use
 * the effective rate for Nyquist and measurement-confidence claims. The
 * generation changes whenever the worker resets acquisition phase (circuit
 * load/cold reset, rate change, or probe configuration change), so a consumer
 * can discard history without guessing from a backward time jump.
 */
export interface ProbeSampleBatch {
  acquisitionId: number;
  netIds: string[];
  times: Float64Array;
  values: Record<string, Float64Array>;
  requestedSampleRateHz: number;
  /** Uniform timestamp/output grid after the worker's transport cap. */
  sampleGridRateHz: number;
  /** Conservative numerical rate: min(grid rate, 1 / largest accepted solver step). */
  effectiveSampleRateHz: number;
}

/** Main-thread request aggregated from all active instrument subscribers. */
export interface ProbeConfiguration {
  netIds: string[];
  requestedSampleRateHz: number;
}

/** How a structured physics value was obtained. */
export interface PhysicsTelemetryProvenance {
  /** Runtime that owns the underlying value or derivation. */
  source: "engine" | "worker-derived";
  /** Prevents a behavioural estimate from being mistaken for a solved quantity. */
  quality: "model-derived" | "behavioral-estimate" | "state-machine";
  /** Stable, machine-readable description of the calculation/readout path. */
  method: string;
}

export type PhysicsCurrentReference =
  | "pin0-to-pin1"
  | "source-output"
  | "collector"
  | "drain"
  | "common-terminal"
  | "coil"
  | "winding"
  | "supply"
  | "output"
  | "output-unit-1"
  | "package-total"
  | "maximum-channel";

export interface PhysicsCurrentTelemetry {
  valueA: number;
  reference: PhysicsCurrentReference;
  /** Included only when the sign/aggregation has a well-defined interpretation. */
  direction?: "positive-pin0-to-pin1" | "positive-delivered" | "positive-draw";
  aggregation?: "single-path" | "signed-sum" | "absolute-sum" | "maximum" | "unit-1-only";
  provenance: PhysicsTelemetryProvenance;
}

export interface PhysicsPowerTelemetry {
  valueW: number;
  /** Positive means the two-terminal component absorbs power; negative means it delivers power. */
  signConvention: "positive-absorbed";
  terminals: [string, string];
  provenance: PhysicsTelemetryProvenance;
}

export interface PhysicsTemperatureTelemetry {
  valueC: number;
  /** Present only for an exact identified package/body profile. */
  profileId?: string;
  profileLabel?: string;
  dissipatedPowerW?: number;
  targetTemperatureC?: number;
  allowedPowerW?: number;
  withinContinuousLimits?: boolean;
  thermalShutdown?: boolean;
  warnings?: string[];
  provenance: PhysicsTelemetryProvenance;
}

/**
 * A known latched/tripped state or an explicit reversible warning, not an
 * estimate of progress toward failure.
 * Absence means no public stress state is available; it does not mean zero stress.
 */
export interface PhysicsStressTelemetry {
  state: "latched-failure" | "reversible-warning" | "ptc-tripped" | "thermal-shutdown";
  kind: string;
  pinId?: string;
  sinceSimTimeS?: number;
  value?: number;
  limit?: number;
  message?: string;
  provenance: PhysicsTelemetryProvenance;
}

export type PhysicsModelStateTelemetry =
  | {
      kind: "battery";
      profileId: string;
      stateOfCharge: number;
      openCircuitVoltageV: number;
      internalResistanceOhm: number;
      remainingCapacityAh: number;
      dischargedCoulombs: number;
      rejectedRechargeCoulombs: number;
      modelTemperatureC: number;
      temperatureWasClamped: boolean;
      provenance: PhysicsTelemetryProvenance;
    }
  | {
      kind: "dc-motor";
      windingCurrentA: number;
      angularVelocityRadPerS: number;
      provenance: PhysicsTelemetryProvenance;
    }
  | {
      kind: "servo";
      angleDeg: number;
      targetAngleDeg: number;
      velocityDegPerS: number;
      moving: boolean;
      powered: boolean;
      supplyVoltageV: number;
      signalHigh: boolean;
      /** Omitted until a complete PWM pulse has actually been decoded. */
      pulseWidthMs?: number;
      /** Omitted until a rising edge has actually been observed. */
      lastRiseSimTimeS?: number;
      provenance: PhysicsTelemetryProvenance;
    }
  | {
      kind: "stepper";
      coilCurrentAA: number;
      coilCurrentBA: number;
      positionSteps: number;
      /** Omitted while both coils are unenergised or inside the deadband. */
      phaseIndex?: number;
      provenance: PhysicsTelemetryProvenance;
    }
  | {
      kind: "hcsr04";
      phase: "idle" | "armed" | "pending" | "active";
      triggerHigh: boolean;
      echoHigh: boolean;
      supplyVoltageV: number;
      /** Omitted when no corresponding echo transition is scheduled. */
      echoRiseSimTimeS?: number;
      echoFallSimTimeS?: number;
      provenance: PhysicsTelemetryProvenance;
    };

export interface ComponentPhysicsTelemetry {
  componentKind: string;
  /** Exact persisted catalog/model variant when known. */
  catalogUid?: string;
  current?: PhysicsCurrentTelemetry;
  /** Present only for a two-terminal path with a trustworthy current direction. */
  power?: PhysicsPowerTelemetry;
  temperature?: PhysicsTemperatureTelemetry;
  /** Only known latched/tripped states are emitted; unexposed accumulated stress stays unknown. */
  stress?: PhysicsStressTelemetry[];
  modelState?: PhysicsModelStateTelemetry;
}

/** Additive, versioned physics telemetry. Older workers may omit it entirely. */
export interface PhysicsTelemetrySnapshot {
  schemaVersion: 1;
  sampledAtSimTimeS: number;
  /**
   * Loaded/reset messages intentionally omit electrical values until the next
   * accepted solve, preventing stale pre-reload currents from masquerading as measurements.
   */
  electricalSample: "last-committed-solution" | "unavailable-before-solve";
  components: Record<string, ComponentPhysicsTelemetry>;
}

/**
 * S18b — HC-SR04 -> micro:bit ECHO bridge event. Emitted the instant a hcsr04
 * TRIG falls and its ECHO net is bridged to a micro:bit edge pin, so the
 * payload (delay + width) is already fully known at emit time — no need to
 * wait for the scheduled rise itself. `pinId` is the SIM id form ("pin0" ..
 * "pin16"), matching the micro:bit fork's own EdgePin naming, NOT the devolt
 * catalog pin id ("p0".."p16"). A later agent wires MicrobitHost to turn this
 * into an inbound `{kind:"pulse_input", ...}` iframe message.
 */
export interface Hcsr04EchoEvent {
  boardComponentId: string;
  pinId: string;
  delayUs: number;
  widthUs: number;
}

/** Per-board runtime state exposed to the renderer for onboard LED/button art. */
export interface ArduinoState {
  /** Board is powered (Vcc-GND > 3 V). */
  powered: boolean;
  /** D13 drive state: true = high output. */
  d13High: boolean;
  /** MCU is held in reset. */
  resetActive: boolean;
  /** UART TX activity (true if avr8js exposes it; otherwise false). */
  txActivity: boolean;
  /** UART RX activity (true if avr8js exposes it; otherwise false). */
  rxActivity: boolean;
}

export type WorkerIn =
  // integrationMethod selects the reactive-element companion model: "be"
  // (default, omitted by current callers) keeps backward Euler byte-identical
  // to the historical engine; "trap" opts into trapezoidal integration.
  | { type: "load"; circuit: SimCircuit; integrationMethod?: "be" | "trap" }
  | { type: "pause" }
  | { type: "resume" }
  | { type: "coldReset" }      // cold reset: zero all element state first
  | { type: "resetFailures" }  // clear session-only simulated component failures
  | { type: "resetArduino"; componentId: string }  // pulse MCU RESET for the given board
  // micro:bit edge-pin drives (P0/P1/P2) pushed from the client-side embedded sim,
  // so the engine can stamp them onto the breadboard. Keyed by de:volt pin id (p0/p1/p2).
  | { type: "microbitDrive"; componentId: string; drives: Record<string, { mode: "digital" | "analog"; value: number }> }
  | ({ type: "configureProbes" } & ProbeConfiguration)
  | { type: "setStepSize"; stepSize: number }  // seconds per sim step (overrides adaptive)
  | { type: "setRate"; multiplier: number };   // 0.1 – 10 × real time

/**
 * Decoded display state for a single driven display component.
 * Populated by W8.1 (MAX7219) and W8.2 (HD44780) decoders; empty this wave.
 */
export type DisplayInfo =
  | { kind: "max7219"; rows: number[]; intensity: number; on: boolean }
  | { kind: "hd44780"; lines: string[]; on: boolean; cols: number; rows: number }
  // micro:bit 5x5 matrix — client-sourced from the embedded MicroPython sim (not
  // the MNA engine), 25 row-major brightness values 0-9.
  | { kind: "microbit"; pixels: number[] };

export type WorkerOut =
  | {
      type: "loaded";
      nets: Net[];
      simTime: number;
      netV: Record<string, number>;
      /** Legacy scalar current map; semantics vary by kind. Prefer physicsTelemetry when present. */
      elementI: Record<string, number>;
      /** Digital output levels (0 or 1) for clock/gate/DFF/555 components. */
      digitalState: Record<string, number>;
      /** Whether the worker is running or paused — lets the main thread preserve pause state. */
      running: boolean;
      /** Whether the topology's seed operating-point solve was accepted. */
      converged: boolean;
      /** Current adaptive step size in seconds, matching tick messages. */
      adaptiveH: number;
      /** Diagnostics for the seed/most-recent solve represented by this snapshot. */
      solver: SolverDiagnostics;
      /** EEPROM byte-array updates since the previous tick. compId → bytes. */
      eepromUpdates?: Record<string, Uint8Array>;
      /** Per-board runtime state for onboard LED rendering. compId → state. */
      arduinoState?: Record<string, ArduinoState>;
      /** Per-channel currents for rgb_led + bicolor_led. compId → [ch0, ch1, ...] A. */
      elementChannelI?: Record<string, number[]>;
      /** Decoded display state for driven display components. compId → state. */
      displayState?: Record<string, DisplayInfo>;
      /** Session-only simulated failures, keyed by failure id. */
      failures: Record<string, SimFailure>;
      /** Component IDs of PTC fuses that are currently in the tripped state. */
      ptcTripped?: string[];
      /** S18b — HC-SR04 -> micro:bit ECHO bridge events queued since the last tick. */
      hcsr04EchoEvents?: Hcsr04EchoEvent[];
      /** Versioned structured physics readouts; optional for protocol backward compatibility. */
      physicsTelemetry?: PhysicsTelemetrySnapshot;
    }
  | {
      type: "tick";
      simTime: number;
      netV: Record<string, number>;
      /** Legacy scalar current map; semantics vary by kind. Prefer physicsTelemetry when present. */
      elementI: Record<string, number>;
      /** Digital output levels (0 or 1) for clock/gate/DFF/555 components. */
      digitalState: Record<string, number>;
      /** Non-convergence signal for this step — if true, UI can show a spinner or warn. */
      converged: boolean;
      /** Current adaptive step size in seconds — displayed in the UI status bar. */
      adaptiveH: number;
      solver: SolverDiagnostics;
      /** EEPROM byte-array updates since the previous tick. compId → bytes. */
      eepromUpdates?: Record<string, Uint8Array>;
      /** Per-board runtime state for onboard LED rendering. compId → state. */
      arduinoState?: Record<string, ArduinoState>;
      /** Per-channel currents for rgb_led + bicolor_led. compId → [ch0, ch1, ...] A. */
      elementChannelI?: Record<string, number[]>;
      /** Decoded display state for driven display components. compId → state. */
      displayState?: Record<string, DisplayInfo>;
      /** Session-only simulated failures, keyed by failure id. */
      failures: Record<string, SimFailure>;
      /** Component IDs of PTC fuses that are currently in the tripped state. */
      ptcTripped?: string[];
      /** S18b — HC-SR04 -> micro:bit ECHO bridge events queued since the last tick. */
      hcsr04EchoEvents?: Hcsr04EchoEvent[];
      /** Uniform samples for the currently configured instrument probe union. */
      probeSamples?: ProbeSampleBatch;
      /** Versioned structured physics readouts; optional for protocol backward compatibility. */
      physicsTelemetry?: PhysicsTelemetrySnapshot;
    }
  | {
      type: "warn";
      /** Stable code — an engine SimWarningCode, or a host/transport code. */
      code: string;
      message: string;
      /** Set when the warning is about one component (engine warnings always are). */
      componentId?: string;
    };
