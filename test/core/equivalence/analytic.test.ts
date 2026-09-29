/** Unit coverage for analytic.ts's closed-form references and gate 2e's pass rule. */

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { initCore } from "../../../src/core/wasm.js";
import { HeadlessRunner } from "../../../src/host/headless.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import {
  checkAnalyticCrossing,
  fixedPairCapacitorCurrentAt,
  ne555AstablePeriod,
  ne555MonostablePulseWidth,
  rcCrossingTime,
  rcLadderVoltagesAt,
  rcTerminalThevenin,
  rcVoltageAt,
  type RcLadderParams,
} from "./analytic.js";
import { collectClassicSamples } from "./sample.js";
import { replayStimuliOnClassic, replayStimuliOnCore, type StimulusPhase } from "./stimuli.js";

// This worker's own new synthetic fixtures (W1b step 2, capacitor): loaded by
// path directly (not through synthetic-fixtures.ts's whole-directory scan)
// since these tests want ONE specific fixture each, cross-checked against an
// analytic reference — not the generic "declared outcome" pass/core-declined
// check equivalence.test.ts already runs over every file in this directory.
const FIXTURES_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "fixtures", "core");
function loadFixtureCircuit(name: string): SimCircuit {
  return JSON.parse(fs.readFileSync(path.join(FIXTURES_DIR, `${name}.sim.json`), "utf8")) as SimCircuit;
}

/** Every kind this worker's new fixtures need beyond CORE_IMPLEMENTED_KINDS — the documented "local override" escape hatch (compileCircuit/CoreEngine.load/replayStimuliOnCore all accept one) until the orchestrator adds "capacitor" to kinds.ts. */
const WITH_CAPACITOR: ReadonlySet<string> = new Set([...CORE_IMPLEMENTED_KINDS, "capacitor"]);

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

// ─── rcVoltageAt ──────────────────────────────────────────────────────────

describe("rcVoltageAt", () => {
  it("is rcCrossingTime's inverse: evaluating at the crossing time reproduces the target", () => {
    const t = rcCrossingTime(0, 3.1, 5, 2e-3);
    expect(rcVoltageAt(0, 5, 2e-3, t)).toBeCloseTo(3.1, 12);
  });

  it("v(0) = v0 and v(infinity-ish) tends to vAsymptote", () => {
    expect(rcVoltageAt(1.5, 5, 1e-3, 0)).toBeCloseTo(1.5, 15);
    expect(rcVoltageAt(1.5, 5, 1e-3, 100e-3)).toBeCloseTo(5, 9); // 100 tau
  });

  it("throws for a non-positive tau", () => {
    expect(() => rcVoltageAt(0, 5, 0, 1e-3)).toThrow();
    expect(() => rcVoltageAt(0, 5, -1, 1e-3)).toThrow();
  });
});

// ─── rcTerminalThevenin ───────────────────────────────────────────────────

/**
 * Independent re-derivation of a single lossy capacitor's TERMINAL voltage
 * by direct forward-Euler integration of the physical state equations (spec
 * §7.4's model: esr in series with C, leakage across the device's own
 * terminals) — never calling rcTerminalThevenin/rcVoltageAt itself. At each
 * instant, the terminal voltage v is pinned algebraically by KCL given the
 * CURRENT capacitor-only voltage vc (v is NOT a state variable — only vc
 * is), then vc is advanced by forward Euler. A tiny fixed step (h =
 * tEnd/steps, steps in the millions) keeps forward Euler's own truncation
 * error far below this test's tolerance without needing anything fancier.
 */
function bruteForceLossyTerminalV(
  sourceVolts: number,
  seriesOhms: number,
  capacitanceF: number,
  esrOhms: number,
  leakageOhms: number,
  tEnd: number,
  steps: number,
): number {
  const h = tEnd / steps;
  const gSum = 1 / seriesOhms + 1 / esrOhms + 1 / leakageOhms;
  let vc = 0;
  let v = (sourceVolts / seriesOhms + vc / esrOhms) / gSum;
  for (let n = 0; n < steps; n++) {
    v = (sourceVolts / seriesOhms + vc / esrOhms) / gSum;
    const i = (v - vc) / esrOhms;
    vc += (h / capacitanceF) * i;
  }
  return v;
}

describe("rcTerminalThevenin", () => {
  it("ideal case (esr=0, no leakage): v0=0, vAsymptote=sourceVolts, tau=seriesOhms*capacitanceF", () => {
    const result = rcTerminalThevenin({ sourceVolts: 5, seriesOhms: 2000, capacitanceF: 1e-6, esrOhms: 0 });
    expect(result.tau).toBeCloseTo(2000 * 1e-6, 15);
    expect(result.vAsymptote).toBeCloseTo(5, 15);
    expect(result.v0AtUncharged).toBeCloseTo(0, 15);
  });

  it("no leakage, esr>0: vAsymptote is still the full source voltage (leakage=Infinity draws no steady current)", () => {
    const result = rcTerminalThevenin({ sourceVolts: 5, seriesOhms: 1000, capacitanceF: 1e-6, esrOhms: 50 });
    expect(result.vAsymptote).toBeCloseTo(5, 15);
    // esr passes an instantaneous current the moment the source is applied,
    // even though the capacitor itself (vc) starts at 0 — v(0) is NOT 0 here.
    expect(result.v0AtUncharged).toBeGreaterThan(0);
  });

  /**
   * The bug this cross-check exists to catch: assuming v(0)=0 for a
   * nonzero-ESR capacitor (conflating the TERMINAL voltage with the
   * INTERNAL vc state) is off by roughly 18% at t ~ 0.2*tau for fixture 12's
   * own numbers — caught by exactly this comparison while drafting that
   * fixture's analytic reference.
   */
  it("matches an independent brute-force ODE integration for fixture 12's exact numbers (esr and leakage both nonzero)", () => {
    const sourceVolts = 5, seriesOhms = 1000, capacitanceF = 1e-6, esrOhms = 50, leakageOhms = 200_000;
    const { tau, vAsymptote, v0AtUncharged } = rcTerminalThevenin({ sourceVolts, seriesOhms, capacitanceF, esrOhms, leakageOhms });

    // Sanity: the naive (wrong) v0=0 assumption must NOT pass this bound —
    // proves this test would have caught the bug described above.
    const naiveWrongAnswer = rcVoltageAt(0, vAsymptote, tau, 0.2e-3);
    const brute0 = bruteForceLossyTerminalV(sourceVolts, seriesOhms, capacitanceF, esrOhms, leakageOhms, 0.2e-3, 2_000_000);
    expect(Math.abs(naiveWrongAnswer - brute0) / brute0).toBeGreaterThan(0.1); // ~18% off for this fixture's numbers

    for (const t of [0.2e-3, 0.5e-3, 1e-3, 2e-3, 5e-3]) {
      const analytic = rcVoltageAt(v0AtUncharged, vAsymptote, tau, t);
      const brute = bruteForceLossyTerminalV(sourceVolts, seriesOhms, capacitanceF, esrOhms, leakageOhms, t, 2_000_000);
      const tolerance = 1e-5 + 1e-5 * Math.abs(brute); // forward Euler's own tiny residual truncation error
      expect(Math.abs(analytic - brute), `t=${t}`).toBeLessThanOrEqual(tolerance);
    }
  });

  it("rejects non-positive seriesOhms/capacitanceF/leakageOhms and a negative esrOhms", () => {
    expect(() => rcTerminalThevenin({ sourceVolts: 5, seriesOhms: 0, capacitanceF: 1e-6, esrOhms: 0 })).toThrow();
    expect(() => rcTerminalThevenin({ sourceVolts: 5, seriesOhms: 1000, capacitanceF: 0, esrOhms: 0 })).toThrow();
    expect(() => rcTerminalThevenin({ sourceVolts: 5, seriesOhms: 1000, capacitanceF: 1e-6, esrOhms: -1 })).toThrow();
    expect(() => rcTerminalThevenin({ sourceVolts: 5, seriesOhms: 1000, capacitanceF: 1e-6, esrOhms: 0, leakageOhms: -1 })).toThrow();
  });
});

// ─── rcLadderVoltagesAt ───────────────────────────────────────────────────

/** Independent RK4 numerical integration of the same 2x2 linear ODE, never calling rcLadderVoltagesAt itself — see that function's own doc comment for the system. */
function rk4Ladder(params: RcLadderParams, tEnd: number, steps: number): [number, number] {
  const { sourceVolts: v, r1Ohms: r1, c1F: c1, r2Ohms: r2, c2F: c2 } = params;
  const a11 = -(1 / r1 + 1 / r2) / c1;
  const a12 = 1 / (r2 * c1);
  const a21 = 1 / (r2 * c2);
  const a22 = -1 / (r2 * c2);
  const b1 = v / (r1 * c1);
  const f = (x: readonly [number, number]): [number, number] => [a11 * x[0] + a12 * x[1] + b1, a21 * x[0] + a22 * x[1]];
  let x: [number, number] = [0, 0];
  const h = tEnd / steps;
  for (let i = 0; i < steps; i++) {
    const k1 = f(x);
    const k2 = f([x[0] + (h / 2) * k1[0], x[1] + (h / 2) * k1[1]]);
    const k3 = f([x[0] + (h / 2) * k2[0], x[1] + (h / 2) * k2[1]]);
    const k4 = f([x[0] + h * k3[0], x[1] + h * k3[1]]);
    x = [
      x[0] + (h / 6) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
      x[1] + (h / 6) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]),
    ];
  }
  return x;
}

describe("rcLadderVoltagesAt", () => {
  const params: RcLadderParams = { sourceVolts: 5, r1Ohms: 1000, c1F: 1e-6, r2Ohms: 1500, c2F: 2e-6 };

  it("matches an independent RK4 numerical integration of the same system, at several times", () => {
    for (const t of [0.5e-3, 1e-3, 2e-3, 4e-3, 8e-3, 20e-3]) {
      const analytic = rcLadderVoltagesAt(params, t);
      const [numericV1, numericV2] = rk4Ladder(params, t, 200_000);
      expect(analytic.v1, `v1 at t=${t}`).toBeCloseTo(numericV1, 6);
      expect(analytic.v2, `v2 at t=${t}`).toBeCloseTo(numericV2, 6);
    }
  });

  it("both nodes settle to the source voltage (no current flows anywhere once both capacitors are fully charged)", () => {
    const settled = rcLadderVoltagesAt(params, 1); // ~1e4 tau
    expect(settled.v1).toBeCloseTo(5, 9);
    expect(settled.v2).toBeCloseTo(5, 9);
  });

  it("starts at (0,0) — both capacitors uncharged", () => {
    const initial = rcLadderVoltagesAt(params, 0);
    expect(initial.v1).toBeCloseTo(0, 9);
    expect(initial.v2).toBeCloseTo(0, 9);
  });

  it("node1 always leads node2 (node2 only charges through node1, via r2)", () => {
    for (const t of [0.2e-3, 1e-3, 3e-3]) {
      const { v1, v2 } = rcLadderVoltagesAt(params, t);
      expect(v1).toBeGreaterThan(v2);
    }
  });

  it("rejects non-positive resistances/capacitances", () => {
    expect(() => rcLadderVoltagesAt({ ...params, r1Ohms: 0 }, 1e-3)).toThrow();
    expect(() => rcLadderVoltagesAt({ ...params, c2F: -1 }, 1e-3)).toThrow();
  });
});

// ─── This worker's new synthetic fixtures (11-15) vs. their analytic reference, on CLASSIC ───
//
// These prove the fixtures' own chosen R/C/esr/leakage values and the
// analytic formulas above are mutually consistent against a REAL classic
// engine run — independent of whether the core (still landing concurrently,
// R2's worktree) supports capacitor yet. The core-side half of gate 2e
// (decision 3's rule: the core may be CLOSER to the analytic answer than
// classic) is the separate describe block below.

/**
 * Classic's own error-controlled adaptive stepper caps at ACTIVE_H_MAX=2ms
 * while a signal is "changing" (the W1 plan's B3 constants) — a genuinely
 * coarse step next to a tau in the microsecond-to-low-millisecond range, so
 * every fixture below uses a tau around 100ms specifically to keep that cap
 * a small fraction of tau (h/tau ~ 2%). The resulting classic-vs-analytic
 * error is a full-scale-referenced quantity (backward Euler's own lag is
 * proportional to how far the step spans, not to the curve's instantaneous
 * value — which is why an early, still-small sample point is NOT held to a
 * tighter absolute budget than a later, larger one): budgeted here at 0.5%
 * of the circuit's own supply voltage, comfortably above the ~0.17% full-
 * scale actually observed while choosing these fixtures' R/C values (well
 * inside physics-reference-benchmarks.test.ts's own established "h=tau/10"
 * bound of 2.5% full-scale for a much coarser h/tau ratio than this one).
 */
function expectCloseToAnalytic(actual: number, analytic: number, fullScaleVolts: number, what: string): void {
  const tolerance = 0.005 * fullScaleVolts;
  expect(Math.abs(actual - analytic), `${what}: actual=${actual}, analytic=${analytic}`).toBeLessThanOrEqual(tolerance);
}

function classicNetIdFor(circuit: SimCircuit, componentId: string, pinId: string): string {
  const runner = new HeadlessRunner();
  runner.load(circuit);
  const netId = runner.readout.getNetIdForPin(componentId, pinId);
  if (!netId) throw new Error(`classicNetIdFor: no net for ${componentId}.${pinId}`);
  return netId;
}

describe("W1b capacitor fixtures (11-15) vs. their analytic reference — classic side", () => {
  it("11-rc-charge: classic tracks the ideal single-exponential RC curve", () => {
    const circuit = loadFixtureCircuit("11-rc-charge");
    const netId = classicNetIdFor(circuit, "c1", "a");
    const tau = 100_000 * 1e-6; // R=100k, C=1uF
    const times = [10e-3, 25e-3, 50e-3, 100e-3, 200e-3];
    const samples = collectClassicSamples(circuit, times);
    times.forEach((t, i) => {
      expectCloseToAnalytic(samples[i]!.netV[netId]!, rcVoltageAt(0, 5, tau, t), 5, `t=${t}`);
    });
  });

  /**
   * The item 2 brief also asks for "the crossing time", not just the exact
   * exponential at a chosen sample — rcCrossingTime's own reference. Checked
   * indirectly (and just as rigorously, for a monotonic curve): classic's
   * OWN voltage sampled at the ANALYTIC crossing time must equal the target
   * crossing voltage within the same full-scale budget as the value checks
   * above; a wrong crossing-time reference would show up here as a voltage
   * mismatch just as surely as a direct root-finding comparison would.
   */
  it("11-rc-charge: classic crosses 4V (80% of the 5V supply) at rcCrossingTime's own analytic instant", () => {
    const circuit = loadFixtureCircuit("11-rc-charge");
    const netId = classicNetIdFor(circuit, "c1", "a");
    const tau = 100_000 * 1e-6;
    const vTarget = 4;
    const crossingTimeS = rcCrossingTime(0, vTarget, 5, tau);
    const samples = collectClassicSamples(circuit, [crossingTimeS]);
    expectCloseToAnalytic(samples[0]!.netV[netId]!, vTarget, 5, `at rcCrossingTime=${crossingTimeS}`);
  });

  it("12-rc-esr-leakage: classic tracks the lossy single-exponential RC curve (Thevenin-reduced esr+leakage)", () => {
    const circuit = loadFixtureCircuit("12-rc-esr-leakage");
    const netId = classicNetIdFor(circuit, "c1", "a");
    const { tau, vAsymptote, v0AtUncharged } = rcTerminalThevenin({
      sourceVolts: 5,
      seriesOhms: 100_000,
      capacitanceF: 1e-6,
      esrOhms: 500,
      leakageOhms: 20_000_000,
    });
    const times = [10e-3, 25e-3, 50e-3, 100e-3, 200e-3];
    const samples = collectClassicSamples(circuit, times);
    times.forEach((t, i) => {
      expectCloseToAnalytic(samples[i]!.netV[netId]!, rcVoltageAt(v0AtUncharged, vAsymptote, tau, t), 5, `t=${t}`);
    });
  });

  // Finding 12: the fixture's OWN declared param now starts the switch OPEN
  // (sim.json), so its toggle sequence (below, "a live switch toggle") can
  // exercise the open-to-closed direction first — a "closed switch..." plain
  // static-load check would just be testing the same closed-contact-is-
  // negligible fact the "a live switch toggle" case's phase-1 charging curve
  // already covers, in more depth (a whole trajectory, not three points).
  // This checks the OTHER static fact the declared param now stands for
  // instead: with no override applied at all, the capacitor never charges.
  it("13-rc-switch-toggle: open switch (the fixture's own declared param) leaves the capacitor at 0V — no path to the source", () => {
    const circuit = loadFixtureCircuit("13-rc-switch-toggle");
    const netId = classicNetIdFor(circuit, "c1", "a");
    const times = [10e-3, 20e-3, 40e-3];
    const samples = collectClassicSamples(circuit, times);
    times.forEach((t, i) => {
      expect(Math.abs(samples[i]!.netV[netId]!), `t=${t}`).toBeLessThan(1e-6);
    });
  });

  it("14-two-capacitor-ladder: classic tracks the two-eigenvalue analytic ladder solution at both nodes", () => {
    const circuit = loadFixtureCircuit("14-two-capacitor-ladder");
    const netId1 = classicNetIdFor(circuit, "c1", "a");
    const netId2 = classicNetIdFor(circuit, "c2", "a");
    const params: RcLadderParams = { sourceVolts: 5, r1Ohms: 100_000, c1F: 1e-6, r2Ohms: 150_000, c2F: 2e-6 };
    const times = [10e-3, 25e-3, 50e-3, 100e-3, 200e-3];
    const samples = collectClassicSamples(circuit, times);
    times.forEach((t, i) => {
      const { v1, v2 } = rcLadderVoltagesAt(params, t);
      expectCloseToAnalytic(samples[i]!.netV[netId1]!, v1, 5, `node1 t=${t}`);
      expectCloseToAnalytic(samples[i]!.netV[netId2]!, v2, 5, `node2 t=${t}`);
    });
  });

  it("15-capacitor-both-ends-fixed: classic settles the capacitor's terminal voltage to the fixed 5V-3V=2V difference (both terminals are ideal sources)", () => {
    const circuit = loadFixtureCircuit("15-capacitor-both-ends-fixed");
    const netIdA = classicNetIdFor(circuit, "c1", "a");
    const netIdB = classicNetIdFor(circuit, "c1", "b");
    const samples = collectClassicSamples(circuit, [1e-3]);
    const terminalV = samples[0]!.netV[netIdA]! - samples[0]!.netV[netIdB]!;
    expectCloseToAnalytic(terminalV, 2, 5, "c1 terminal V (5V - 3V)");
  });
});

// ─── The same fixtures vs. their analytic reference — CORE side (gate 2e proper) ───
//
// Uses the documented "local override" escape hatch (WITH_CAPACITOR) rather
// than CORE_IMPLEMENTED_KINDS itself (kinds.ts is the orchestrator's table —
// off-limits to this worker; it adds "capacitor" once both this worker's
// compiler emission and R2's Rust device/solver land). Decision 3's rule:
// the core's single-capacitor closed form may be CLOSER to the analytic
// answer than classic is — checkAnalyticCrossing's own pass rule already
// encodes this (a classicValue farther from the analytic answer than the
// core's is recorded, never held against the core).

// Both engines stamp NODE_RSHUNT_G = 1e-12 S from every node to the datum
// (core/README.md, static-solve point 2), so the exact answer for these
// fixtures includes it as a leakage in parallel with the capacitor. Leaving
// it out shifts the asymptote by about R * 1e-12, which is 1e-7 relative at
// these fixtures' 100 kohm: far above the 1e-9 bound checked here.
const NODE_SHUNT_OHMS = 1e12;

function parallelOhms(a: number, b: number): number {
  return 1 / (1 / a + 1 / b);
}

describe("W1b capacitor fixtures (11-15) vs. their analytic reference — core side (gate 2e)", () => {
  beforeAll(async () => {
    await initCore();
  });

  function loadCore(circuit: SimCircuit): CoreEngine {
    const engine = new CoreEngine();
    engine.load(circuit, WITH_CAPACITOR);
    return engine;
  }

  it("11-rc-charge: the core's closed form is within 1e-9 relative of the exact exponential (classic recorded for the report, not asserted)", () => {
    const circuit = loadFixtureCircuit("11-rc-charge");
    const { tau, vAsymptote } = rcTerminalThevenin({
      sourceVolts: 5,
      seriesOhms: 100_000,
      capacitanceF: 1e-6,
      esrOhms: 0,
      leakageOhms: NODE_SHUNT_OHMS,
    });
    const t = tau; // one tau
    const analyticValue = rcVoltageAt(0, vAsymptote, tau, t);

    const core = loadCore(circuit);
    core.advance(t);
    const netId = core.getNetIdForPin("c1", "a")!;
    const coreValue = core.getNetV()[netId]!;

    const classicNetId = classicNetIdFor(circuit, "c1", "a");
    const classicValue = collectClassicSamples(circuit, [t])[0]!.netV[classicNetId]!;

    const result = checkAnalyticCrossing({ coreValue, analyticValue, classicValue, absTol: 1e-9, relTol: 1e-9 });
    expect(result.status, result.reason).toBe("pass");
  });

  it("11-rc-charge: the core's closed form reaches 4V (80% of the 5V supply) within 1e-9 relative of rcCrossingTime's own analytic instant", () => {
    const circuit = loadFixtureCircuit("11-rc-charge");
    const { tau, vAsymptote } = rcTerminalThevenin({
      sourceVolts: 5,
      seriesOhms: 100_000,
      capacitanceF: 1e-6,
      esrOhms: 0,
      leakageOhms: NODE_SHUNT_OHMS,
    });
    const vTarget = 4;
    const crossingTimeS = rcCrossingTime(0, vTarget, vAsymptote, tau);

    const core = loadCore(circuit);
    core.advance(crossingTimeS);
    const netId = core.getNetIdForPin("c1", "a")!;
    const coreValue = core.getNetV()[netId]!;

    const classicNetId = classicNetIdFor(circuit, "c1", "a");
    const classicValue = collectClassicSamples(circuit, [crossingTimeS])[0]!.netV[classicNetId]!;

    const result = checkAnalyticCrossing({ coreValue, analyticValue: vTarget, classicValue, absTol: 1e-9, relTol: 1e-9 });
    expect(result.status, result.reason).toBe("pass");
  });

  it("12-rc-esr-leakage: the core's closed form (esr+leakage Thevenin-reduced) is within 1e-9 relative of the analytic answer", () => {
    const circuit = loadFixtureCircuit("12-rc-esr-leakage");
    const { tau, vAsymptote, v0AtUncharged } = rcTerminalThevenin({
      sourceVolts: 5,
      seriesOhms: 100_000,
      capacitanceF: 1e-6,
      esrOhms: 500,
      leakageOhms: parallelOhms(20_000_000, NODE_SHUNT_OHMS),
    });
    const t = tau;
    const analyticValue = rcVoltageAt(v0AtUncharged, vAsymptote, tau, t);

    const core = loadCore(circuit);
    core.advance(t);
    const netId = core.getNetIdForPin("c1", "a")!;
    const coreValue = core.getNetV()[netId]!;

    const result = checkAnalyticCrossing({ coreValue, analyticValue, absTol: 1e-9, relTol: 1e-9 });
    expect(result.status, result.reason).toBe("pass");
  });

  it("13-rc-switch-toggle: a live switch toggle (bo_engine_set_params) charges then freezes the capacitor, matching the ideal RC curve then holding", () => {
    const circuit = loadFixtureCircuit("13-rc-switch-toggle");
    const tau = 100_000 * 1e-6;
    const phases: StimulusPhase[] = [
      { set: [{ id: "sw", params: { closed: 1 } }], durationS: 40e-3 },
      { set: [{ id: "sw", params: { closed: 0 } }], durationS: 40e-3 },
    ];

    const classicSamples = replayStimuliOnClassic(circuit, phases);
    const coreSamples = replayStimuliOnCore(circuit, phases, classicSamples, WITH_CAPACITOR);

    const netId = classicNetIdFor(circuit, "c1", "a");
    const endOfPhase1 = classicSamples.findIndex((s) => s.phaseIndex === 1) - 1;
    expect(endOfPhase1).toBeGreaterThan(0);

    // Phase 1 (closed): both engines track the exact exponential. A FIXED
    // h=1ms=tau/100 backward-Euler step (not the adaptive controller
    // expectCloseToAnalytic above is tuned for) — reusing physics-reference-
    // benchmarks.test.ts's own established bound for that exact h/tau ratio
    // (0.3% full-scale, "h=tau/100 stays within 0.3% full-scale at t=tau"),
    // applied here as a flat absolute budget at every checkpoint rather than
    // scaled to each checkpoint's own (smaller, earlier-in-the-transient)
    // analytic value.
    const fullScaleBudgetV = 0.003 * 5;
    for (const i of [9, 19, 29, 39]) {
      const t = classicSamples[i]!.simTime;
      const analyticValue = rcVoltageAt(0, 5, tau, t);
      expect(Math.abs(classicSamples[i]!.netV[netId]! - analyticValue), `classic phase1 t=${t}`).toBeLessThanOrEqual(fullScaleBudgetV);
      expect(Math.abs(coreSamples[i]!.netV[netId]! - analyticValue), `core phase1 t=${t}`).toBeLessThanOrEqual(fullScaleBudgetV);
    }

    // Phase 2 (open): the capacitor holds whatever charge it had at the
    // toggle instant — no path to the source remains, and the universal
    // node-shunt decay (README's NODE_RSHUNT_G, tau ~ C/1e-12 ~ 1e6 s) is
    // utterly negligible over this phase's 40 ms span.
    // Each engine must hold ITS OWN charge at the toggle. The two engines'
    // phase-1 values differ by classic's backward-Euler lag against the
    // core's exact trajectory (decision 3), so holding is checked per
    // engine, not against the other engine's value.
    const lastIndex = classicSamples.length - 1;
    const classicAtToggle = classicSamples[endOfPhase1]!.netV[netId]!;
    const coreAtToggle = coreSamples[endOfPhase1]!.netV[netId]!;
    expect(Math.abs(classicSamples[lastIndex]!.netV[netId]! - classicAtToggle)).toBeLessThan(1e-3);
    expect(Math.abs(coreSamples[lastIndex]!.netV[netId]! - coreAtToggle)).toBeLessThan(1e-3);
  });

  it("14-two-capacitor-ladder: the core's stepped solver (decision 14: not exact, but within tolerance) agrees with the analytic ladder solution (classic recorded for the report, not asserted)", () => {
    const circuit = loadFixtureCircuit("14-two-capacitor-ladder");
    const params: RcLadderParams = { sourceVolts: 5, r1Ohms: 100_000, c1F: 1e-6, r2Ohms: 150_000, c2F: 2e-6 };
    const t = 50e-3;
    const analytic = rcLadderVoltagesAt(params, t);

    const core = loadCore(circuit);
    core.advance(t);
    const netId1 = core.getNetIdForPin("c1", "a")!;
    const netId2 = core.getNetIdForPin("c2", "a")!;
    const coreV1 = core.getNetV()[netId1]!;
    const coreV2 = core.getNetV()[netId2]!;

    const classicNetId1 = classicNetIdFor(circuit, "c1", "a");
    const classicNetId2 = classicNetIdFor(circuit, "c2", "a");
    const classicSample = collectClassicSamples(circuit, [t])[0]!;

    // Decision 14: the stepped path is not claimed exact, so this uses a
    // looser, full-scale-referenced tolerance than the single-capacitor
    // closed-form cases above (1e-9 relative) — matching this file's own
    // classic-side reasoning (expectCloseToAnalytic's doc comment): backward
    // Euler's lag scales with the step, not with the curve's instantaneous
    // value, so the budget is a flat fraction of the supply voltage rather
    // than of each node's own (here quite different, v1 >> v2) value.
    const fullScaleBudgetV = 0.005 * 5;
    const result1 = checkAnalyticCrossing({
      coreValue: coreV1,
      analyticValue: analytic.v1,
      classicValue: classicSample.netV[classicNetId1]!,
      absTol: fullScaleBudgetV,
      relTol: 0,
    });
    const result2 = checkAnalyticCrossing({
      coreValue: coreV2,
      analyticValue: analytic.v2,
      classicValue: classicSample.netV[classicNetId2]!,
      absTol: fullScaleBudgetV,
      relTol: 0,
    });
    expect(result1.status, result1.reason).toBe("pass");
    expect(result2.status, result2.reason).toBe("pass");
  });

  it("15-capacitor-both-ends-fixed: the core loads the topology (rule 2a — both terminals fixed is no longer a decline) and publishes the exact dV/esr/leakage answer at load, matching the analytic reference exactly (classic's own load-instant artifact recorded for the report, not asserted)", () => {
    const circuit = loadFixtureCircuit("15-capacitor-both-ends-fixed");
    // dV = 5V - 3V = 2V; this fixture's capacitor declares no catalogUid, so
    // esr/leakage both resolve to their "none" sentinel (0) — the series
    // current is identically 0 the way `esrOhms === 0` always gives
    // (fixedPairCapacitorCurrentAt's own doc comment), and so is the
    // leakage term (`leakageOhms === 0`). This is the simplest possible
    // instance of rule 2a: the whole point of this test is that the core
    // now ACCEPTS the topology at all and reports the mathematically exact
    // (here: exactly zero) answer where classic's own load-seed formula
    // (`C*dV/1e-12`, core/README.md's documented known divergence) does not.
    const analyticI = fixedPairCapacitorCurrentAt(2, 1e-6, 0, 0, 0);
    expect(analyticI).toBe(0);

    const core = loadCore(circuit);
    const coreI = core.getElementI().c1!;

    const classicSamples = collectClassicSamples(circuit, [0]);
    const classicI = classicSamples[0]!.elementI.c1!;

    const result = checkAnalyticCrossing({ coreValue: coreI, analyticValue: analyticI, classicValue: classicI, absTol: 1e-9, relTol: 1e-9 });
    expect(result.status, result.reason).toBe("pass");
    // Sanity: classic's own known-divergence artifact is genuinely present
    // in this run (C*dV/1e-12 = 1e-6*2/1e-12 = 2e6A) — otherwise this test
    // would not actually be proving anything about the artifact it names.
    expect(classicI).toBeCloseTo(2_000_000, 0);
  });
});
