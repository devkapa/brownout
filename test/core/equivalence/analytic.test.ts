/** Unit coverage for analytic.ts's closed-form references and gate 2e's pass rule. */

import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { checkAnalyticCrossing, ne555AstablePeriod, ne555MonostablePulseWidth, rcCrossingTime } from "./analytic.js";

describe("rcCrossingTime", () => {
  it("matches the textbook single time-constant crossing (63.2% at t=tau)", () => {
    const t = rcCrossingTime(0, 1 - Math.E ** -1, 1, 1);
    expect(t).toBeCloseTo(1, 12);
  });

  it("throws for a target that is not strictly between v0 and the asymptote", () => {
    expect(() => rcCrossingTime(0, 1, 1, 1)).toThrow(); // target equals the asymptote — never reached
    expect(() => rcCrossingTime(0, -1, 1, 1)).toThrow(); // target is on the wrong side of v0
  });

  it("throws for a non-positive tau", () => {
    expect(() => rcCrossingTime(0, 0.5, 1, 0)).toThrow();
    expect(() => rcCrossingTime(0, 0.5, 1, -1)).toThrow();
  });
});

describe("ne555AstablePeriod", () => {
  it("matches the classic 555 astable formula (T = ln(2) * (R1 + 2*R2) * C) under the default 1/3-2/3 thresholds", () => {
    const r1 = 10_000;
    const r2 = 100_000;
    const c = 1e-6;
    const result = ne555AstablePeriod({ r1Ohms: r1, r2Ohms: r2, capacitanceF: c, supplyVoltsAtGnd: 5, disResistanceOhms: 0 });
    const textbook = Math.log(2) * (r1 + 2 * r2) * c;
    expect(result.periodS).toBeCloseTo(textbook, 9);
  });

  it("a nonzero DIS resistance lengthens the discharge time (and thus the period) relative to disResistanceOhms=0", () => {
    const base = { r1Ohms: 10_000, r2Ohms: 100_000, capacitanceF: 1e-6, supplyVoltsAtGnd: 5 };
    const withoutDis = ne555AstablePeriod({ ...base, disResistanceOhms: 0 });
    const withDis = ne555AstablePeriod({ ...base, disResistanceOhms: 10 });
    expect(withDis.dischargeTimeS).toBeGreaterThan(withoutDis.dischargeTimeS);
    expect(withDis.periodS).toBeGreaterThan(withoutDis.periodS);
  });

  it("frequencyHz is the reciprocal of periodS", () => {
    const result = ne555AstablePeriod({ r1Ohms: 1000, r2Ohms: 2000, capacitanceF: 1e-7, supplyVoltsAtGnd: 5 });
    expect(result.frequencyHz).toBeCloseTo(1 / result.periodS, 12);
  });

  /**
   * Finding 6: the discharge half must be classic's OWN linear model (spec
   * §5.2), not the textbook (R2+Rdis)-toward-0 formula — classic's DIS stamp
   * (digital-ics.ts:216-218) adds Rdis to GND but never disconnects R1 from
   * VCC, so the DIS node is a three-way Thevenin junction, not a plain
   * series path. R1=R2=1kΩ, C=1µF, 5V is the ben-eater clock module's own
   * astable and the exact circuit the W1-0 spike measured "core 2.10165 ms,
   * exact" against (devolt/memory/progress/brownout-core-v2.md:361) — this
   * derives that same number independently, via Thevenin's theorem on
   * classic's own stamp, rather than importing the spike's figure verbatim.
   */
  it("derives the discharge half as classic's own Thevenin junction at the DIS node (R1 to VCC, Rdis to GND, R2 into the cap), not (R2+Rdis) toward 0", () => {
    const r1 = 1000;
    const r2 = 1000;
    const c = 1e-6;
    const vs = 5;
    const rdis = 10;

    // Independent re-derivation (Thevenin's theorem on the three-resistor
    // junction classic's stamp creates), not a call into analytic.ts's own
    // implementation — this is what makes it a real regression test rather
    // than the function checking itself.
    const thevRes = (r1 * rdis) / (r1 + rdis);
    const thevVolts = vs * (rdis / (r1 + rdis));
    const vLow = (1 / 3) * vs;
    const vHigh = (2 / 3) * vs;
    const tauCharge = (r1 + r2) * c;
    const tauDischarge = (r2 + thevRes) * c;
    const expectedCharge = -tauCharge * Math.log((vHigh - vs) / (vLow - vs));
    const expectedDischarge = -tauDischarge * Math.log((vLow - thevVolts) / (vHigh - thevVolts));
    const expectedPeriod = expectedCharge + expectedDischarge;

    // Sanity-check the hand derivation against the W1-0 spike's own recorded
    // "exact" figure before trusting it as the test's oracle.
    expect(expectedPeriod).toBeCloseTo(2.101645e-3, 8);

    const result = ne555AstablePeriod({ r1Ohms: r1, r2Ohms: r2, capacitanceF: c, supplyVoltsAtGnd: vs, disResistanceOhms: rdis });
    expect(result.periodS).toBeCloseTo(expectedPeriod, 12);

    // The pre-fix formula — (R2+Rdis) toward 0, dropping R1 entirely — is
    // provably a different, ~2%-off value. Pinning that down here means a
    // regression back to it fails loudly rather than by coincidence.
    const tauDischargePreFix = (r2 + rdis) * c;
    const dischargePreFix = -tauDischargePreFix * Math.log(vLow / vHigh);
    expect(Math.abs(dischargePreFix - expectedDischarge) / expectedDischarge).toBeGreaterThan(1e-3);
  });

  /**
   * Finding 6 (locked decision 1): check the closed form against an actual
   * classic run on a simple astable, to within classic's own (backward-
   * Euler step-grid) error — not just the hand-derived math above. Same
   * R1=R2=1kΩ/C=1µF/5V circuit, wired exactly like
   * test/host/headless-step-ceiling.test.ts's ASTABLE fixture.
   */
  it("matches an actual classic run of a simple R1=R2=1kΩ/C=1µF astable to within classic's own discretisation error", () => {
    const part = (id: string, kind: string, pins: string[], params: Record<string, number> = {}) => ({
      id,
      kind,
      pins: pins.map((pin) => ({ id: pin })),
      params,
    });
    const wire = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

    const astable: SimCircuit = {
      components: [
        part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("ra", "resistor", ["a", "b"], { resistance: 1000 }),
        part("rb", "resistor", ["a", "b"], { resistance: 1000 }),
        part("ct", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
        part("load", "resistor", ["a", "b"], { resistance: 1000 }),
        part("timer", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]),
      ],
      wires: [
        wire("vdd", "pos", "timer", "8"),
        wire("vdd", "pos", "timer", "4"),
        wire("vdd", "neg", "timer", "1"),
        wire("vdd", "pos", "ra", "a"),
        wire("ra", "b", "timer", "7"),
        wire("timer", "7", "rb", "a"),
        wire("rb", "b", "timer", "6"),
        wire("timer", "6", "timer", "2"),
        wire("timer", "6", "ct", "a"),
        wire("ct", "b", "vdd", "neg"),
        wire("timer", "3", "load", "a"),
        wire("load", "b", "vdd", "neg"),
      ],
    };

    const runner = new HeadlessRunner();
    runner.load(astable);
    const outNet = runner.netIdFor("timer", "3")!;
    const risingEdges: number[] = [];
    let prev: number | undefined;
    runner.run({
      durationS: 0.15,
      onSample: (s) => {
        const v = s.netV[outNet]! > 2.5 ? 1 : 0;
        if (v === 1 && prev === 0) risingEdges.push(s.simTime);
        prev = v;
      },
    });

    // Drop the first few edges (startup transient) and measure over the
    // remaining span so a single noisy period doesn't dominate the average.
    const settled = risingEdges.slice(3);
    const classicPeriodS = (settled[settled.length - 1]! - settled[0]!) / (settled.length - 1);

    const analytic = ne555AstablePeriod({ r1Ohms: 1000, r2Ohms: 1000, capacitanceF: 1e-6, supplyVoltsAtGnd: 5 });

    // Classic's own backward-Euler discretisation is documented to run a 555
    // astable slow by roughly this much (test/host/headless-step-ceiling.
    // test.ts's own docstring: "a 555 astable ran 1.5% slow"; measured here
    // at ~1.5%). The pre-fix (R1-less) formula sits on the FAR side of that
    // band from classic's real behaviour (measured ~2.2% here) because it is
    // wrong in the opposite direction from classic's lag, not merely less
    // precise — so this tolerance catches the bug without being so tight
    // that classic's ordinary discretisation slop trips it.
    const relErr = Math.abs(classicPeriodS - analytic.periodS) / analytic.periodS;
    expect(relErr).toBeLessThan(0.018);
  });
});

describe("ne555MonostablePulseWidth", () => {
  it("matches the classic 555 monostable formula (t = ln(3) * R * C) under the default 2/3 threshold", () => {
    const r = 100_000;
    const c = 10e-6;
    const width = ne555MonostablePulseWidth({ rOhms: r, capacitanceF: c, supplyVoltsAtGnd: 5 });
    expect(width).toBeCloseTo(Math.log(3) * r * c, 12);
  });
});

describe("checkAnalyticCrossing (gate 2e)", () => {
  it("passes when the core is within tolerance of the analytic value, even if classic is NOT (the 555 period case)", () => {
    const result = checkAnalyticCrossing({
      coreValue: 2.1016e-3, // matches the analytic answer closely (per the W1-0 spike)
      analyticValue: 2.1016e-3,
      classicValue: 2.14e-3, // classic's own discretised period — expected to be off by design
      absTol: 1e-9,
      relTol: 1e-6,
    });
    expect(result.status).toBe("pass");
    expect(result.reason).toMatch(/within tolerance/);
  });

  it("fails when the core itself is outside tolerance, regardless of what classic does", () => {
    const result = checkAnalyticCrossing({
      coreValue: 2.5e-3,
      analyticValue: 2.1016e-3,
      classicValue: 2.14e-3,
      absTol: 1e-9,
      relTol: 1e-6,
    });
    expect(result.status).toBe("fail");
  });

  it("records whether the core is at least as close to the analytic answer as classic, without that ever flipping the verdict", () => {
    const closerThanClassic = checkAnalyticCrossing({
      coreValue: 1.0000001,
      analyticValue: 1,
      classicValue: 1.02,
      absTol: 0,
      relTol: 1e-6,
    });
    expect(closerThanClassic.status).toBe("pass");
    expect(closerThanClassic.reason).toMatch(/at least as close as classic/);

    const classicHappensCloser = checkAnalyticCrossing({
      coreValue: 1.0000005,
      analyticValue: 1,
      classicValue: 1.0000001,
      absTol: 0,
      relTol: 1e-3, // generous enough that the core still passes on its own
    });
    expect(classicHappensCloser.status).toBe("pass");
    expect(classicHappensCloser.reason).toMatch(/classic happens to be closer/);
  });

  it("works without a classicValue at all (pure core-vs-analytic)", () => {
    const result = checkAnalyticCrossing({ coreValue: 1.0000001, analyticValue: 1, absTol: 0, relTol: 1e-6 });
    expect(result.status).toBe("pass");
    expect(result.classicAbsError).toBeUndefined();
  });

  /**
   * Finding 9: a core measurement that never happened (no crossing found, a
   * period computed from an empty edge list, 0/0) surfaces as NaN, and
   * `NaN > tolerance` is always false in JS — the old `coreAbsError >
   * tolerance` fail-check therefore fell through to "pass". Every case here
   * failed the pre-fix implementation with status "pass".
   */
  describe("seeded fault: NaN or non-finite inputs must fail, never fall through to pass", () => {
    it("fails when coreValue is NaN", () => {
      const result = checkAnalyticCrossing({ coreValue: NaN, analyticValue: 2.10165e-3, absTol: 0, relTol: 1e-6 });
      expect(result.status).toBe("fail");
    });

    it("fails when analyticValue is NaN", () => {
      const result = checkAnalyticCrossing({ coreValue: 2.10165e-3, analyticValue: NaN, absTol: 0, relTol: 1e-6 });
      expect(result.status).toBe("fail");
    });

    it("fails when absTol is NaN (a NaN tolerance can't certify anything as within it)", () => {
      const result = checkAnalyticCrossing({ coreValue: 1, analyticValue: 1, absTol: NaN, relTol: 0 });
      expect(result.status).toBe("fail");
    });

    it("fails when coreValue is +/-Infinity", () => {
      expect(checkAnalyticCrossing({ coreValue: Infinity, analyticValue: 1, absTol: 0, relTol: 1e-6 }).status).toBe("fail");
      expect(checkAnalyticCrossing({ coreValue: -Infinity, analyticValue: 1, absTol: 0, relTol: 1e-6 }).status).toBe("fail");
    });

    it("still passes a genuinely finite, in-tolerance case (the fix does not make everything fail)", () => {
      const result = checkAnalyticCrossing({ coreValue: 2.10165e-3, analyticValue: 2.10165e-3, absTol: 1e-9, relTol: 1e-6 });
      expect(result.status).toBe("pass");
    });

    /**
     * Finding 9 repair round: a classicValue that IS supplied but is NaN (or
     * +/-Infinity) means classic's own run never produced a measurement —
     * that is a broken comparison, not "classic happens to be far off", and
     * must fail rather than pass with a NaN classicAbsError buried in the
     * report. Before this fix, `classicAbsError` fed only the `reason`
     * string and never the fail condition, so a NaN/non-finite classicValue
     * fell straight through to "pass".
     */
    it("fails when classicValue is supplied but NaN, even though the core itself is in tolerance", () => {
      const result = checkAnalyticCrossing({
        coreValue: 2.10165e-3,
        analyticValue: 2.10165e-3,
        classicValue: NaN,
        absTol: 1e-9,
        relTol: 1e-6,
      });
      expect(result.status).toBe("fail");
      expect(result.reason).toMatch(/classicValue/);
    });

    it("fails when classicValue is supplied but +/-Infinity, even though the core itself is in tolerance", () => {
      expect(
        checkAnalyticCrossing({
          coreValue: 2.10165e-3,
          analyticValue: 2.10165e-3,
          classicValue: Infinity,
          absTol: 1e-9,
          relTol: 1e-6,
        }).status,
      ).toBe("fail");
      expect(
        checkAnalyticCrossing({
          coreValue: 2.10165e-3,
          analyticValue: 2.10165e-3,
          classicValue: -Infinity,
          absTol: 1e-9,
          relTol: 1e-6,
        }).status,
      ).toBe("fail");
    });

    it("still passes with no classicValue at all, unaffected by the classicValue finiteness check", () => {
      const result = checkAnalyticCrossing({ coreValue: 2.10165e-3, analyticValue: 2.10165e-3, absTol: 1e-9, relTol: 1e-6 });
      expect(result.status).toBe("pass");
      expect(result.classicAbsError).toBeUndefined();
    });
  });
});
