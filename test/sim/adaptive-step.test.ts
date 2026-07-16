import { describe, expect, it } from "vitest";
import { estimateStepError, nextStepFactor } from "../../src/sim/adaptive-step.js";
import type { StateSnapshot } from "../../src/sim/engine/sim-engine.js";

function snapshot(voltage: number, cap = voltage): StateSnapshot {
  return {
    caps: new Map([["c1", cap]]),
    inds: new Map(),
    ne555s: new Map(),
    relays: new Map(),
    motors: new Map(),
    servos: new Map(),
    steppers: new Map(),
    lcds: new Map(),
    hcsr04: new Map(),
    failureStress: new Map(),
    failures: new Map(),
    icState: new Map(),
    eeproms: new Map(),
    ptcs: new Map(),
    thermalTemps: new Map(),
    netV: { n0: voltage },
    elementI: {},
    digitalState: {},
    simTime: 1,
  };
}

describe("adaptive transient error control", () => {
  it("accepts matching coarse and refined states and permits bounded growth", () => {
    expect(estimateStepError(snapshot(1), snapshot(1))).toEqual({ ratio: 0, quantity: null });
    expect(nextStepFactor(0)).toBe(2);
  });

  it("detects an electrically meaningful coarse/refined disagreement", () => {
    const estimate = estimateStepError(snapshot(1), snapshot(1.01));
    expect(estimate.ratio).toBeGreaterThan(1);
    expect(estimate.quantity).toMatch(/voltage|capacitor/);
    expect(nextStepFactor(estimate.ratio)).toBeLessThan(1);
  });

  it("includes package temperature, servo motion, and op-amp pole memory", () => {
    const coarse = snapshot(1);
    const refined = snapshot(1);
    const thermalBase = {
      catalogUid: "resistor",
      profileId: "axial-resistor-quarter-watt",
      temperatureC: 25,
      thermalShutdown: false,
      dissipatedPowerW: 0.25,
      targetTemperatureC: 110,
      allowedPowerW: 0.25,
      withinContinuousLimits: true,
      warnings: [],
    };
    coarse.thermalDevices = new Map([["r1", thermalBase]]);
    refined.thermalDevices = new Map([["r1", { ...thermalBase, temperatureC: 26 }]]);
    expect(estimateStepError(coarse, refined).quantity).toBe("package-temperature:r1");

    refined.thermalDevices = new Map(coarse.thermalDevices);
    const servoBase = {
      lastSig: 0,
      riseT: Number.NaN,
      pulseMs: 2,
      targetAngle: 180,
      angle: 90,
      velocity: 600,
      moving: true,
      powered: true,
      supplyVoltage: 4.8,
      loadResistance: 32,
    };
    coarse.servos.set("servo", servoBase);
    refined.servos.set("servo", { ...servoBase, angle: 91 });
    expect(estimateStepError(coarse, refined).quantity).toBe("servo-angle:servo");

    refined.servos = new Map(coarse.servos);
    coarse.icState.set("op", { u1: 1 });
    refined.icState.set("op", { u1: 1.1 });
    expect(estimateStepError(coarse, refined).quantity).toBe("ic-state:op:u1");
  });

  it("covers committed electrical memories that can differ behind fixed node voltages", () => {
    {
      const coarse = snapshot(1);
      const refined = snapshot(1);
      coarse.mosfetGates = new Map([["m", { vgs: 1, vgd: 0.5 }]]);
      refined.mosfetGates = new Map([["m", { vgs: 1.01, vgd: 0.5 }]]);
      expect(estimateStepError(coarse, refined).quantity).toBe("mosfet-vgs:m");
    }

    {
      const coarse = snapshot(1);
      const refined = snapshot(1);
      coarse.relays.set("relay", { iCoil: 0.01, energized: false });
      refined.relays.set("relay", { iCoil: 0.011, energized: false });
      expect(estimateStepError(coarse, refined).quantity).toBe("relay-current:relay");
    }

    {
      const coarse = snapshot(1);
      const refined = snapshot(1);
      const base = { iA: 0.1, iB: -0.1, phase: 0, lastKnownPhase: 0, position: 4 };
      coarse.steppers.set("stepper", base);
      refined.steppers.set("stepper", { ...base, iB: -0.11 });
      expect(estimateStepError(coarse, refined).quantity).toBe("stepper-current-b:stepper");
    }

    {
      const coarse = snapshot(1);
      const refined = snapshot(1);
      coarse.ptcs.set("ptc", { tripped: false, tripStress: 0.1, recoveryTime: 0 });
      refined.ptcs.set("ptc", { tripped: false, tripStress: 0.11, recoveryTime: 0 });
      expect(estimateStepError(coarse, refined).quantity).toBe("ptc-trip-stress:ptc");
    }

    {
      const coarse = snapshot(1);
      const refined = snapshot(1);
      coarse.icState.set("converter", { reg: 0, iin: 0.1 });
      refined.icState.set("converter", { reg: 0, iin: 0.11 });
      expect(estimateStepError(coarse, refined).quantity).toBe("ic-state:converter:iin");
    }
  });

  it("rejects exact latch, failure, and operating-regime disagreements", () => {
    function expectDiscreteMismatch(
      mutate: (coarse: StateSnapshot, refined: StateSnapshot) => void,
      quantity: string,
    ): void {
      const coarse = snapshot(1);
      const refined = snapshot(1);
      mutate(coarse, refined);
      expect(estimateStepError(coarse, refined)).toEqual({
        ratio: Number.POSITIVE_INFINITY,
        quantity,
      });
    }

    expectDiscreteMismatch((coarse, refined) => {
      coarse.relays.set("relay", { iCoil: 0.01, energized: false });
      refined.relays.set("relay", { iCoil: 0.01, energized: true });
    }, "relay-energized:relay");

    expectDiscreteMismatch((coarse, refined) => {
      const base = { iA: 0.1, iB: 0.1, phase: 0, lastKnownPhase: 0, position: 0 };
      coarse.steppers.set("stepper", base);
      refined.steppers.set("stepper", { ...base, phase: 1 });
    }, "stepper-phase:stepper");

    expectDiscreteMismatch((coarse, refined) => {
      coarse.ptcs.set("ptc", { tripped: false, tripStress: 0.1, recoveryTime: 0 });
      refined.ptcs.set("ptc", { tripped: true, tripStress: 0.1, recoveryTime: 0 });
    }, "ptc-tripped:ptc");

    expectDiscreteMismatch((coarse, refined) => {
      const base = {
        catalogUid: "reg-7805",
        profileId: "to-220-regulator",
        temperatureC: 125,
        thermalShutdown: false,
        dissipatedPowerW: 2,
        targetTemperatureC: 130,
        allowedPowerW: 1,
        withinContinuousLimits: false,
        warnings: [],
      };
      coarse.thermalDevices = new Map([["reg", base]]);
      refined.thermalDevices = new Map([["reg", { ...base, thermalShutdown: true }]]);
    }, "package-thermal-shutdown:reg");

    expectDiscreteMismatch((coarse, refined) => {
      coarse.failures.set("resistor_overload:r1", {
        componentId: "r1",
        kind: "resistor_overload",
        since: 1,
        message: "failed open",
      });
      refined.failures.clear();
    }, "failure:resistor_overload:r1:present");

    expectDiscreteMismatch((coarse, refined) => {
      coarse.icState.set("converter", { reg: 0, iin: 0.1 });
      refined.icState.set("converter", { reg: 2, iin: 0.1 });
    }, "ic-state:converter:reg");
  });

  it("never changes the step by an unsafe factor", () => {
    expect(nextStepFactor(Number.POSITIVE_INFINITY)).toBe(0.2);
    expect(nextStepFactor(1e20)).toBe(0.2);
    expect(nextStepFactor(1e-20)).toBe(2);
  });

  it("uses the order-2 exponent for trapezoidal step growth", () => {
    // Order 1 keeps the historical sqrt controller: 0.9 / sqrt(8) ~ 0.318.
    expect(nextStepFactor(8)).toBe(0.9 / Math.sqrt(8));
    expect(nextStepFactor(8, 1)).toBe(0.9 / Math.sqrt(8));
    // Order 2 uses ratio^(-1/(p+1)) = ratio^(-1/3): 0.9 * 8^(-1/3) = 0.45.
    // The expected expression mirrors the implementation exactly so the
    // assertion is bit-stable across platforms.
    expect(nextStepFactor(8, 2)).toBe(0.9 * Math.pow(8, -1 / 3));
    expect(nextStepFactor(8, 2)).toBeCloseTo(0.45, 12);
    // A second-order method shrinks less for the same over-tolerance error,
    // because its local truncation error falls faster with h.
    expect(nextStepFactor(8, 2)).toBeGreaterThan(nextStepFactor(8, 1));
  });

  it("clamps order-2 factors to the same [0.2, 2] safety bounds", () => {
    // 0.9 * (1e-9)^(-1/3) = 900: growth is still capped at doubling.
    expect(nextStepFactor(1e-9, 2)).toBe(2);
    // 0.9 * (1e9)^(-1/3) = 9e-4: shrink is still floored at one fifth.
    expect(nextStepFactor(1e9, 2)).toBe(0.2);
    // The near-zero shortcut applies before the exponent is ever evaluated.
    expect(nextStepFactor(0, 2)).toBe(2);
    expect(nextStepFactor(1e-12, 2)).toBe(2);
  });

  it("collapses non-finite error ratios to the maximum shrink for every order", () => {
    expect(nextStepFactor(Number.NaN)).toBe(0.2);
    expect(nextStepFactor(Number.NaN, 2)).toBe(0.2);
    expect(nextStepFactor(Number.POSITIVE_INFINITY, 2)).toBe(0.2);
    expect(nextStepFactor(Number.NEGATIVE_INFINITY, 2)).toBe(0.2);
  });

  it("keeps the default-order factor identical to the historical controller", () => {
    // Every existing call site omits `order`; default-mode step sizes must not
    // move by even one ulp, so compare against the pre-A2 expression exactly.
    for (const ratio of [1e-13, 1e-6, 0.25, 0.5, 1, 2, 4, 8, 100, 1e6]) {
      const historical = ratio <= 1e-12
        ? 2
        : Math.max(0.2, Math.min(2, 0.9 / Math.sqrt(ratio)));
      expect(nextStepFactor(ratio)).toBe(historical);
      expect(nextStepFactor(ratio, 1)).toBe(historical);
    }
  });

  it("refines on trapezoidal history (capsI/indsV) divergence", () => {
    {
      const coarse = snapshot(1);
      const refined = snapshot(1);
      coarse.capsI = new Map([["c1", 0.01]]);
      refined.capsI = new Map([["c1", 0.011]]);
      const estimate = estimateStepError(coarse, refined);
      expect(estimate.quantity).toBe("capacitor-current:c1");
      expect(estimate.ratio).toBeGreaterThan(1);
      expect(nextStepFactor(estimate.ratio, 2)).toBeLessThan(1);
    }

    {
      const coarse = snapshot(1);
      const refined = snapshot(1);
      coarse.indsV = new Map([["l1", 1]]);
      refined.indsV = new Map([["l1", 1.01]]);
      const estimate = estimateStepError(coarse, refined);
      expect(estimate.quantity).toBe("inductor-voltage:l1");
      expect(estimate.ratio).toBeGreaterThan(1);
    }

    {
      // A history term present on one side only is a regime disagreement, not
      // a small numerical error: it must force refinement outright.
      const coarse = snapshot(1);
      const refined = snapshot(1);
      coarse.capsI = new Map([["c1", 0.01]]);
      refined.capsI = new Map();
      expect(estimateStepError(coarse, refined)).toEqual({
        ratio: Number.POSITIVE_INFINITY,
        quantity: "capacitor-current:c1:present",
      });
    }

    {
      // Matching history terms contribute nothing to the estimate.
      const coarse = snapshot(1);
      const refined = snapshot(1);
      coarse.capsI = new Map([["c1", 0.01]]);
      refined.capsI = new Map([["c1", 0.01]]);
      coarse.indsV = new Map([["l1", 1]]);
      refined.indsV = new Map([["l1", 1]]);
      expect(estimateStepError(coarse, refined)).toEqual({ ratio: 0, quantity: null });
    }

    // Backward-Euler snapshots omit both maps entirely; the estimate must be
    // byte-identical to the pre-trap comparison in that case.
    expect(estimateStepError(snapshot(1), snapshot(1))).toEqual({ ratio: 0, quantity: null });
  });
});
