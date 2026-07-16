import { describe, expect, it } from "vitest";
import { advanceServoMotion } from "../../src/sim/servo-physics.js";

describe("hobby-servo rate-limited motion", () => {
  it("moves at the declared no-load speed without snapping to target", () => {
    const next = advanceServoMotion({
      angleDeg: 0,
      targetAngleDeg: 180,
      dtSeconds: 0.1,
      maxSpeedDegPerSecond: 500,
      supplyVoltageV: 6,
    });
    expect(next.angleDeg).toBeCloseTo(50, 12);
    expect(next.velocityDegPerSecond).toBeCloseTo(500, 12);
    expect(next.moving).toBe(true);
  });

  it("lands exactly on the target without overshoot", () => {
    const next = advanceServoMotion({
      angleDeg: 85,
      targetAngleDeg: 90,
      dtSeconds: 1,
      maxSpeedDegPerSecond: 500,
      supplyVoltageV: 6,
    });
    expect(next.angleDeg).toBe(90);
    expect(next.velocityDegPerSecond).toBe(5);
    expect(next.moving).toBe(false);
  });

  it("scales speed with supply and stops below brown-out", () => {
    const half = advanceServoMotion({
      angleDeg: 0,
      targetAngleDeg: 180,
      dtSeconds: 0.1,
      maxSpeedDegPerSecond: 500,
      supplyVoltageV: 3,
      minimumOperatingVoltageV: 2.5,
    });
    const off = advanceServoMotion({
      angleDeg: 0,
      targetAngleDeg: 180,
      dtSeconds: 0.1,
      maxSpeedDegPerSecond: 500,
      supplyVoltageV: 2.4,
      minimumOperatingVoltageV: 2.5,
    });
    expect(half.angleDeg).toBeCloseTo(25, 12);
    expect(half.speedScale).toBeCloseTo(0.5, 12);
    expect(off.angleDeg).toBe(0);
    expect(off.powered).toBe(false);
  });

  it("clamps authored and target angles to mechanical travel", () => {
    const next = advanceServoMotion({
      angleDeg: -40,
      targetAngleDeg: 300,
      dtSeconds: 2,
      maxSpeedDegPerSecond: 500,
      supplyVoltageV: 6,
      minAngleDeg: 10,
      maxAngleDeg: 170,
    });
    expect(next.angleDeg).toBe(170);
  });
});
