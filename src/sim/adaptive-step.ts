import type { ErrorStateSnapshot } from "./engine/sim-engine.js";

/**
 * Error controller for the first-order backward-Euler transient integrator.
 *
 * A coarse h step is compared with two h/2 steps. For a first-order method,
 * their difference is the local error estimate (Richardson denominator is 1).
 * The refined two-half-step state is retained when the estimate is accepted.
 *
 * Tolerances intentionally describe observable electrical quantities rather
 * than matrix coefficients. They are tight enough for breadboard teaching but
 * loose enough that a noisy nonlinear junction does not pin every circuit to
 * the 10 ns floor.
 */
export interface StepErrorEstimate {
  ratio: number;
  quantity: string | null;
}

const REL_TOL = 1e-3;
const ABS_VOLTAGE = 100e-6;
const ABS_CURRENT = 10e-9;
const ABS_SPEED = 1e-3;
const ABS_ANGLE = 1e-4;
const ABS_TEMPERATURE = 1e-3;
const ABS_TIME = 10e-9;
const ABS_PULSE_WIDTH_MS = 1e-5;
const ABS_RESISTANCE = 1e-6;
const ABS_FREQUENCY = 1e-3;
const ABS_GENERIC = 1e-9;

// Shared read-only stand-in for an absent trapezoidal history map, so the
// one-sided comparison below never allocates on the per-checked-step path.
const EMPTY_HISTORY = new Map<string, number>();

function normalizedError(a: number, b: number, absTol: number): number {
  if (!Number.isFinite(a) || !Number.isFinite(b)) {
    return Object.is(a, b) ? 0 : Number.POSITIVE_INFINITY;
  }
  const scale = absTol + REL_TOL * Math.max(Math.abs(a), Math.abs(b));
  return Math.abs(a - b) / scale;
}

/** Event timestamps are absolute simulation times, so epoch size is not a useful scale. */
function absoluteError(a: number, b: number, absTol: number): number {
  if (!Number.isFinite(a) || !Number.isFinite(b)) {
    return Object.is(a, b) ? 0 : Number.POSITIVE_INFINITY;
  }
  return Math.abs(a - b) / absTol;
}

function compareNumber(
  a: number,
  b: number,
  absTol: number,
  quantity: string,
  current: StepErrorEstimate,
  absoluteOnly = false,
): StepErrorEstimate {
  const ratio = absoluteOnly
    ? absoluteError(a, b, absTol)
    : normalizedError(a, b, absTol);
  return ratio > current.ratio ? { ratio, quantity } : current;
}

/** A different latch/regime is not a small numerical error; the trial must be refined. */
function compareExact(
  a: unknown,
  b: unknown,
  quantity: string,
  current: StepErrorEstimate,
): StepErrorEstimate {
  if (Object.is(a, b) || current.ratio === Number.POSITIVE_INFINITY) return current;
  return { ratio: Number.POSITIVE_INFINITY, quantity };
}

function comparePresence(
  a: unknown,
  b: unknown,
  quantity: string,
  current: StepErrorEstimate,
): StepErrorEstimate | null {
  const aPresent = a !== undefined;
  const bPresent = b !== undefined;
  if (aPresent === bPresent) return null;
  return compareExact(aPresent, bPresent, quantity, current);
}

function compareRecord(
  coarse: Record<string, number>,
  refined: Record<string, number>,
  absTol: number,
  prefix: string,
  current: StepErrorEstimate,
): StepErrorEstimate {
  let worst = current;
  const keys = new Set([...Object.keys(coarse), ...Object.keys(refined)]);
  for (const key of keys) {
    const ratio = normalizedError(coarse[key] ?? Number.NaN, refined[key] ?? Number.NaN, absTol);
    if (ratio > worst.ratio) worst = { ratio, quantity: `${prefix}:${key}` };
  }
  return worst;
}

function compareMap(
  coarse: Map<string, number>,
  refined: Map<string, number>,
  absTol: number,
  prefix: string,
  current: StepErrorEstimate,
): StepErrorEstimate {
  let worst = current;
  const keys = new Set([...coarse.keys(), ...refined.keys()]);
  for (const key of keys) {
    if (!coarse.has(key) || !refined.has(key)) {
      worst = compareExact(coarse.has(key), refined.has(key), `${prefix}:${key}:present`, worst);
      continue;
    }
    const ratio = normalizedError(
      coarse.get(key) as number,
      refined.get(key) as number,
      absTol,
    );
    if (ratio > worst.ratio) worst = { ratio, quantity: `${prefix}:${key}` };
  }
  return worst;
}

function icStateTolerance(key: string): { absTol: number; absoluteOnly?: boolean } | null {
  if (key === "iin" || key === "i1" || key === "i2") return { absTol: ABS_CURRENT };
  if (
    key === "u1" || key === "u2" || key === "d1" || key === "d2" ||
    key === "supplyV" || key === "vm" || key === "vPeak" ||
    key === "vTrough" || key === "peakToPeak"
  ) {
    return { absTol: ABS_VOLTAGE };
  }
  if (key === "lastCrossT" || key === "windowStart" || key.startsWith("due:")) {
    return { absTol: ABS_TIME, absoluteOnly: true };
  }
  if (key === "detectedHz") return { absTol: ABS_FREQUENCY };
  return null;
}

/**
 * Compare every committed state that can change a later stamp or state-machine
 * transition. Continuous memories use physical-unit tolerances; latch/regime
 * disagreements are rejected exactly because averaging two regimes is invalid.
 */
export function estimateStepError(
  coarse: ErrorStateSnapshot,
  refined: ErrorStateSnapshot,
): StepErrorEstimate {
  let worst: StepErrorEstimate = { ratio: 0, quantity: null };
  worst = compareRecord(coarse.netV, refined.netV, ABS_VOLTAGE, "voltage", worst);
  worst = compareMap(coarse.caps, refined.caps, ABS_VOLTAGE, "capacitor", worst);
  worst = compareMap(coarse.inds, refined.inds, ABS_CURRENT, "inductor", worst);
  // Trapezoidal history terms. The engine includes these maps only when they
  // feed the next stamp (trap mode); backward-Euler snapshots omit them so the
  // default controller's decisions stay byte-identical to the pre-trap engine.
  // Skipping the compares outright when BOTH sides omit a map keeps this hot
  // path allocation-free for the default mode (it runs on every checked step);
  // a one-sided map still compares against the shared empty history so a
  // present/absent disagreement forces refinement exactly as before.
  if (coarse.capsI !== undefined || refined.capsI !== undefined) {
    worst = compareMap(coarse.capsI ?? EMPTY_HISTORY, refined.capsI ?? EMPTY_HISTORY, ABS_CURRENT, "capacitor-current", worst);
  }
  if (coarse.indsV !== undefined || refined.indsV !== undefined) {
    worst = compareMap(coarse.indsV ?? EMPTY_HISTORY, refined.indsV ?? EMPTY_HISTORY, ABS_VOLTAGE, "inductor-voltage", worst);
  }
  worst = compareMap(coarse.thermalTemps, refined.thermalTemps, ABS_TEMPERATURE, "temperature", worst);
  worst = compareMap(coarse.failureStress, refined.failureStress, ABS_GENERIC, "stress", worst);

  const coarseMosfets = coarse.mosfetGates ?? new Map();
  const refinedMosfets = refined.mosfetGates ?? new Map();
  const mosfetIds = new Set([...coarseMosfets.keys(), ...refinedMosfets.keys()]);
  for (const id of mosfetIds) {
    const a = coarseMosfets.get(id);
    const b = refinedMosfets.get(id);
    const presence = comparePresence(a, b, `mosfet:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareNumber(a!.vgs, b!.vgs, ABS_VOLTAGE, `mosfet-vgs:${id}`, worst);
    worst = compareNumber(a!.vgd, b!.vgd, ABS_VOLTAGE, `mosfet-vgd:${id}`, worst);
  }

  const coarseBatteries = coarse.batteries ?? new Map();
  const refinedBatteries = refined.batteries ?? new Map();
  const batteryIds = new Set([...coarseBatteries.keys(), ...refinedBatteries.keys()]);
  for (const id of batteryIds) {
    const a = coarseBatteries.get(id);
    const b = refinedBatteries.get(id);
    const presence = comparePresence(a, b, `battery:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareNumber(a!.soc, b!.soc, ABS_GENERIC, `battery-soc:${id}`, worst);
  }

  const coarseThermal = coarse.thermalDevices ?? new Map();
  const refinedThermal = refined.thermalDevices ?? new Map();
  const thermalIds = new Set([...coarseThermal.keys(), ...refinedThermal.keys()]);
  for (const id of thermalIds) {
    const a = coarseThermal.get(id);
    const b = refinedThermal.get(id);
    const presence = comparePresence(a, b, `package-thermal:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareNumber(
      a!.temperatureC,
      b!.temperatureC,
      ABS_TEMPERATURE,
      `package-temperature:${id}`,
      worst,
    );
    worst = compareExact(
      a!.thermalShutdown,
      b!.thermalShutdown,
      `package-thermal-shutdown:${id}`,
      worst,
    );
    worst = compareExact(
      a!.withinContinuousLimits,
      b!.withinContinuousLimits,
      `package-continuous-limit:${id}`,
      worst,
    );
  }

  const ne555Ids = new Set([...coarse.ne555s.keys(), ...refined.ne555s.keys()]);
  for (const id of ne555Ids) {
    const a = coarse.ne555s.get(id);
    const b = refined.ne555s.get(id);
    const presence = comparePresence(a, b, `ne555:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareExact(a!.outHigh, b!.outHigh, `ne555-output:${id}`, worst);
  }

  const relayIds = new Set([...coarse.relays.keys(), ...refined.relays.keys()]);
  for (const id of relayIds) {
    const a = coarse.relays.get(id);
    const b = refined.relays.get(id);
    const presence = comparePresence(a, b, `relay:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareNumber(a!.iCoil, b!.iCoil, ABS_CURRENT, `relay-current:${id}`, worst);
    worst = compareExact(a!.energized, b!.energized, `relay-energized:${id}`, worst);
  }

  const motorIds = new Set([...coarse.motors.keys(), ...refined.motors.keys()]);
  for (const id of motorIds) {
    const a = coarse.motors.get(id);
    const b = refined.motors.get(id);
    const presence = comparePresence(a, b, `motor:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareNumber(a!.iWinding, b!.iWinding, ABS_CURRENT, `motor-current:${id}`, worst);
    worst = compareNumber(a!.omega, b!.omega, ABS_SPEED, `motor-speed:${id}`, worst);
  }

  const servoIds = new Set([...coarse.servos.keys(), ...refined.servos.keys()]);
  for (const id of servoIds) {
    const a = coarse.servos.get(id);
    const b = refined.servos.get(id);
    const presence = comparePresence(a, b, `servo:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareExact(a!.lastSig, b!.lastSig, `servo-signal-state:${id}`, worst);
    worst = compareNumber(a!.riseT, b!.riseT, ABS_TIME, `servo-rise-time:${id}`, worst, true);
    worst = compareNumber(
      a!.pulseMs,
      b!.pulseMs,
      ABS_PULSE_WIDTH_MS,
      `servo-pulse-width:${id}`,
      worst,
    );
    worst = compareNumber(a!.targetAngle, b!.targetAngle, ABS_ANGLE, `servo-target-angle:${id}`, worst);
    worst = compareNumber(a!.angle, b!.angle, ABS_ANGLE, `servo-angle:${id}`, worst);
    worst = compareNumber(a!.velocity, b!.velocity, ABS_SPEED, `servo-speed:${id}`, worst);
    worst = compareExact(a!.moving, b!.moving, `servo-moving:${id}`, worst);
    worst = compareExact(a!.powered, b!.powered, `servo-powered:${id}`, worst);
    worst = compareNumber(
      a!.supplyVoltage,
      b!.supplyVoltage,
      ABS_VOLTAGE,
      `servo-supply-voltage:${id}`,
      worst,
    );
    worst = compareNumber(
      a!.loadResistance,
      b!.loadResistance,
      ABS_RESISTANCE,
      `servo-load-resistance:${id}`,
      worst,
    );
  }

  const stepperIds = new Set([...coarse.steppers.keys(), ...refined.steppers.keys()]);
  for (const id of stepperIds) {
    const a = coarse.steppers.get(id);
    const b = refined.steppers.get(id);
    const presence = comparePresence(a, b, `stepper:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareNumber(a!.iA, b!.iA, ABS_CURRENT, `stepper-current-a:${id}`, worst);
    worst = compareNumber(a!.iB, b!.iB, ABS_CURRENT, `stepper-current-b:${id}`, worst);
    worst = compareExact(a!.phase, b!.phase, `stepper-phase:${id}`, worst);
    worst = compareExact(
      a!.lastKnownPhase,
      b!.lastKnownPhase,
      `stepper-last-phase:${id}`,
      worst,
    );
    worst = compareExact(a!.position, b!.position, `stepper-position:${id}`, worst);
  }

  const hcsr04Ids = new Set([...coarse.hcsr04.keys(), ...refined.hcsr04.keys()]);
  for (const id of hcsr04Ids) {
    const a = coarse.hcsr04.get(id);
    const b = refined.hcsr04.get(id);
    const presence = comparePresence(a, b, `hcsr04:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareExact(a!.trigLevel, b!.trigLevel, `hcsr04-trigger:${id}`, worst);
    worst = compareExact(a!.phase, b!.phase, `hcsr04-phase:${id}`, worst);
    worst = compareExact(a!.echoOut, b!.echoOut, `hcsr04-echo:${id}`, worst);
    worst = compareNumber(
      a!.echoRiseAt,
      b!.echoRiseAt,
      ABS_TIME,
      `hcsr04-echo-rise:${id}`,
      worst,
      true,
    );
    worst = compareNumber(
      a!.echoFallAt,
      b!.echoFallAt,
      ABS_TIME,
      `hcsr04-echo-fall:${id}`,
      worst,
      true,
    );
    worst = compareNumber(a!.vccV, b!.vccV, ABS_VOLTAGE, `hcsr04-supply:${id}`, worst);
    worst = compareExact(
      a!.pendingEchoEvent === null,
      b!.pendingEchoEvent === null,
      `hcsr04-pending-event:${id}`,
      worst,
    );
    if (a!.pendingEchoEvent && b!.pendingEchoEvent) {
      worst = compareExact(
        a!.pendingEchoEvent.boardComponentId,
        b!.pendingEchoEvent.boardComponentId,
        `hcsr04-pending-board:${id}`,
        worst,
      );
      worst = compareExact(
        a!.pendingEchoEvent.pinId,
        b!.pendingEchoEvent.pinId,
        `hcsr04-pending-pin:${id}`,
        worst,
      );
      worst = compareExact(
        a!.pendingEchoEvent.delayUs,
        b!.pendingEchoEvent.delayUs,
        `hcsr04-pending-delay:${id}`,
        worst,
      );
      worst = compareExact(
        a!.pendingEchoEvent.widthUs,
        b!.pendingEchoEvent.widthUs,
        `hcsr04-pending-width:${id}`,
        worst,
      );
    }
  }

  const ptcIds = new Set([...coarse.ptcs.keys(), ...refined.ptcs.keys()]);
  for (const id of ptcIds) {
    const a = coarse.ptcs.get(id);
    const b = refined.ptcs.get(id);
    const presence = comparePresence(a, b, `ptc:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareExact(a!.tripped, b!.tripped, `ptc-tripped:${id}`, worst);
    worst = compareNumber(a!.tripStress, b!.tripStress, ABS_GENERIC, `ptc-trip-stress:${id}`, worst);
    worst = compareNumber(a!.recoveryTime, b!.recoveryTime, ABS_TIME, `ptc-recovery-time:${id}`, worst);
  }

  const failureIds = new Set([...coarse.failures.keys(), ...refined.failures.keys()]);
  for (const id of failureIds) {
    const a = coarse.failures.get(id);
    const b = refined.failures.get(id);
    const presence = comparePresence(a, b, `failure:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    worst = compareExact(a!.kind, b!.kind, `failure:${id}:kind`, worst);
    worst = compareExact(a!.componentId, b!.componentId, `failure:${id}:component`, worst);
    worst = compareExact(a!.pinId, b!.pinId, `failure:${id}:pin`, worst);
    worst = compareNumber(a!.since, b!.since, ABS_TIME, `failure:${id}:time`, worst, true);
  }

  const icIds = new Set([...coarse.icState.keys(), ...refined.icState.keys()]);
  for (const id of icIds) {
    const a = coarse.icState.get(id);
    const b = refined.icState.get(id);
    const presence = comparePresence(a, b, `ic-state:${id}:present`, worst);
    if (presence) {
      worst = presence;
      continue;
    }
    const keys = new Set([...Object.keys(a!), ...Object.keys(b!)]);
    for (const key of keys) {
      if (!(key in a!) || !(key in b!)) {
        worst = compareExact(key in a!, key in b!, `ic-state:${id}:${key}:present`, worst);
        continue;
      }
      const tolerance = icStateTolerance(key);
      worst = tolerance
        ? compareNumber(
            a![key]!,
            b![key]!,
            tolerance.absTol,
            `ic-state:${id}:${key}`,
            worst,
            tolerance.absoluteOnly,
          )
        : compareExact(a![key], b![key], `ic-state:${id}:${key}`, worst);
    }
  }

  return worst;
}

/**
 * Step-size multiplier with conservative bounds. `order` is the integration
 * method's order p: local truncation error scales as h^(p+1), so the optimal
 * growth factor is ratio^(-1/(p+1)) — 1 for backward Euler (the historical
 * sqrt), 2 for trapezoidal.
 */
export function nextStepFactor(errorRatio: number, order = 1): number {
  if (!Number.isFinite(errorRatio)) return 0.2;
  if (errorRatio <= 1e-12) return 2;
  // Keep the historical sqrt expression for order 1: Math.pow(x, -0.5) is not
  // guaranteed bit-identical to 1/Math.sqrt(x), and default-mode step sizes
  // must not move by even one ulp.
  if (order === 1) {
    return Math.max(0.2, Math.min(2, 0.9 / Math.sqrt(errorRatio)));
  }
  return Math.max(0.2, Math.min(2, 0.9 * Math.pow(errorRatio, -1 / (order + 1))));
}
