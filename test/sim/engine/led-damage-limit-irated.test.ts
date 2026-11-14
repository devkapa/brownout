import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

// Q-O24: commitLedOvercurrentStress (semiconductors.ts:472-508, shared by
// led/bicolor_led/rgb_led) used to read specs.if_max before params.iRated,
// the reverse of the I-V stamp's precedence (stampShockleyJunction:374). A
// user-set iRated therefore moved the curve but not the damage limit. "led"
// is the smallest circuit that reaches the shared function; bicolor_led and
// rgb_led call the identical fixed line, so they are not duplicated here.
function ledOnResistor(resistance: number, iRated?: number): SimCircuit {
  return {
    components: [
      {
        id: "vcc",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "r1",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance },
      },
      {
        id: "led1",
        kind: "led",
        // No explicit catalogUid: color "red" infers led-red (vf 1.8,
        // if_max 0.02 A / 20 mA) per spec 6.5, and without an exact uid the
        // part gets no thermal identity, so the drive current stays exactly
        // constant across the run (verified against the fixed code below).
        pins: [{ id: "a" }, { id: "k" }],
        params: iRated === undefined ? { color: "red" } : { color: "red", iRated },
      },
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "led1", to_pin: "a" },
      { from_component: "led1", from_pin: "k", to_component: "vcc", to_pin: "neg" },
    ],
  };
}

// Steps until getFailures() is non-empty, or undefined if it never latches
// within maxSteps. Returns the simulated time of the step that latched.
function stepUntilFailure(engine: SimEngine, dt: number, maxSteps: number): number | undefined {
  for (let step = 1; step <= maxSteps; step++) {
    engine.step(dt);
    if (Object.keys(engine.getFailures()).length > 0) return step * dt;
  }
  return undefined;
}

describe("Q-O24: LED damage limit reads iRated before if_max, like the stamp", () => {
  it("fails open on a 10 mA iRated LED driven at ~15.9 mA, under its 20 mA catalog if_max", () => {
    const engine = new SimEngine();
    engine.load(ledOnResistor(200, 0.01));
    engine.step(1e-3);
    const peakI = engine.getElementI()["led1"] ?? 0;
    // ~15.9 mA: over the 10 mA iRated, under the 20 mA catalog if_max. This
    // is the case Q-O24 names: only the fixed precedence damages the part.
    expect(peakI).toBeGreaterThan(0.014);
    expect(peakI).toBeLessThan(0.017);

    // Dwell derivation (spec 9.5.1 recordAccumulatedStress; 9.5.2/6.7 for the
    // LED-specific constants): with ifMax = iRated = 0.01 A (the fix),
    // r = peakI / ifMax ~= 1.588, stress rate = max(0, r^2 - 1) ~= 1.52 /s,
    // recovery is irrelevant once rate > 0, and the latch threshold is
    // LED_DAMAGE_THRESHOLD = 24. Time to fail ~= 24 / 1.52 ~= 15.8 s.
    const failAt = stepUntilFailure(engine, 1e-3, 18_000);
    expect(failAt).toBeDefined();
    expect(failAt!).toBeGreaterThan(15);
    expect(failAt!).toBeLessThan(16.5);

    const failure = Object.values(engine.getFailures())[0];
    expect(failure?.kind).toBe("led_failed");
    expect(failure?.componentId).toBe("led1");
    expect(failure?.limit).toBeCloseTo(0.01, 6);
    // The gate (semiconductors.ts:358-360, :441) reads the failure state
    // committed by the PREVIOUS step, so the latching step's own elementI is
    // still the pre-open current; one more step is open.
    engine.step(1e-3);
    expect(Math.abs(engine.getElementI()["led1"] ?? 0)).toBeLessThan(1e-6);
  });

  it("control: with no iRated param, the limit stays the catalog's 20 mA if_max", () => {
    const engine = new SimEngine();
    engine.load(ledOnResistor(200));
    // Same ~15.9 mA drive is under the 20 mA if_max (r ~= 0.79 < 1), so
    // stress never accumulates. Run well past the fixed case's ~15.8 s.
    const failAt = stepUntilFailure(engine, 1e-3, 18_000);
    expect(failAt).toBeUndefined();
  });
});
