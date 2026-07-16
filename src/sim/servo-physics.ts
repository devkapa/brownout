/**
 * Small, deterministic hobby-servo mechanical model.
 *
 * A command pulse sets a target angle; the shaft then travels toward it at the
 * declared no-load speed. Speed scales linearly with usable supply voltage and
 * becomes zero below the electronics brown-out threshold. This is intentionally
 * a rate-limited position actuator, not a rigid-body gear-train simulation:
 * acceleration, backlash, load torque, stall heating and control-loop hunting
 * remain outside the model and must be disclosed by the UI.
 */

export interface ServoMotionInput {
  angleDeg: number;
  targetAngleDeg: number;
  dtSeconds: number;
  maxSpeedDegPerSecond: number;
  supplyVoltageV: number;
  nominalSupplyVoltageV?: number;
  minimumOperatingVoltageV?: number;
  minAngleDeg?: number;
  maxAngleDeg?: number;
}
export interface ServoMotionResult {
  angleDeg: number;
  velocityDegPerSecond: number;
  moving: boolean;
  powered: boolean;
  speedScale: number;
}

function finite(value: number, fallback: number): number {
  return Number.isFinite(value) ? value : fallback;
}

export function advanceServoMotion(input: ServoMotionInput): ServoMotionResult {
  const minAngleDeg = finite(input.minAngleDeg ?? 0, 0);
  const maxAngleDeg = Math.max(minAngleDeg, finite(input.maxAngleDeg ?? 180, 180));
  const angleDeg = Math.max(minAngleDeg, Math.min(maxAngleDeg, finite(input.angleDeg, 90)));
  const targetAngleDeg = Math.max(
    minAngleDeg,
    Math.min(maxAngleDeg, finite(input.targetAngleDeg, angleDeg)),
  );
  const dtSeconds = Math.max(0, finite(input.dtSeconds, 0));
  const nominalSupplyVoltageV = Math.max(1e-9, finite(input.nominalSupplyVoltageV ?? 6, 6));
  const minimumOperatingVoltageV = Math.max(
    0,
    finite(input.minimumOperatingVoltageV ?? 3, 3),
  );
  const supplyVoltageV = Math.max(0, finite(input.supplyVoltageV, 0));
  const powered = supplyVoltageV >= minimumOperatingVoltageV;
  const speedScale = powered
    ? Math.max(0, Math.min(1.5, supplyVoltageV / nominalSupplyVoltageV))
    : 0;
  const maxSpeed = Math.max(0, finite(input.maxSpeedDegPerSecond, 0)) * speedScale;
  const error = targetAngleDeg - angleDeg;
  const maxTravel = maxSpeed * dtSeconds;
  const travel = Math.max(-maxTravel, Math.min(maxTravel, error));
  const nextAngle = Math.max(minAngleDeg, Math.min(maxAngleDeg, angleDeg + travel));
  const velocity = dtSeconds > 0 ? travel / dtSeconds : 0;

  return {
    angleDeg: nextAngle,
    velocityDegPerSecond: velocity,
    moving: Math.abs(targetAngleDeg - nextAngle) > 1e-9,
    powered,
    speedScale,
  };
}
