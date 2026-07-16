import { describe, expect, it } from "vitest";
import { pulseVoltage } from "../../../src/sim/engine/digital.js";

describe("pulse source one-shot mode", () => {
  it("per=0 emits one finite trapezoid and then holds the low level", () => {
    const v1 = 1;
    const v2 = 4;
    const td = 1e-3;
    const tr = 10e-6;
    const pw = 100e-6;
    const tf = 20e-6;

    expect(pulseVoltage(td - 1e-9, v1, v2, td, tr, tf, pw, 0)).toBe(v1);
    expect(pulseVoltage(td + tr / 2, v1, v2, td, tr, tf, pw, 0)).toBeCloseTo(2.5, 12);
    expect(pulseVoltage(td + tr + pw / 2, v1, v2, td, tr, tf, pw, 0)).toBe(v2);
    expect(pulseVoltage(td + tr + pw + tf / 2, v1, v2, td, tr, tf, pw, 0)).toBeCloseTo(2.5, 12);
    expect(pulseVoltage(td + 1, v1, v2, td, tr, tf, pw, 0)).toBe(v1);
    expect(Number.isFinite(pulseVoltage(td + 1, v1, v2, td, tr, tf, pw, 0))).toBe(true);
  });

  it("keeps positive periods repeating", () => {
    const args = [0, 5, 0, 1e-6, 1e-6, 100e-6, 1e-3] as const;
    expect(pulseVoltage(50e-6, ...args)).toBe(pulseVoltage(1.05e-3, ...args));
  });
});
