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
  detectNe555Specs,
  evaluateLedStatic,
  extractNe555Sidecar,
  fixedPairCapacitorCurrentAt,
  ledCurrent,
  ledLaw,
  ledRcVoltages,
  ledSeriesNode,
  ledSharedCommon,
  ne555AstablePeriod,
  ne555MonostablePulseWidth,
  ne555Phases,
  NE555_DIS_OHMS,
  rcCrossingTime,
  rcLadderVoltagesAt,
  rcTerminalThevenin,
  rcVoltageAt,
  theveninAt,
  type Ne555TimingModel,
  type RcLadderParams,
  type ResistiveNetwork,
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


// ─── W1e: the model-exact NE555 sidecar (core/README.md NE555 point 13.3) ────
//
// Every number below is from the drafting probe `periods2.py`, which solves the
// timing network in exact rational arithmetic with the 1e-12 S shunt on every
// free node; analytic.ts is plain double precision, so they agree to about 1e-12
// relative and are asserted here to 1e-9.

/** The astable of a resistor chain `VCC - RA - D - RB (- RPOT) - X` with C from X to GND, as a `ne555Phases` model built straight from `theveninAt`. */
function astableModel(params: { ra: number; rb: number; rpot?: number; c: number; vs: number }): Ne555TimingModel {
  const nodes = params.rpot === undefined ? ["D", "X"] : ["D", "P", "X"];
  const chain = params.rpot === undefined ? [{ id: "rb", a: "D", b: "X", ohms: params.rb }] : [{ id: "rb", a: "D", b: "P", ohms: params.rb }, { id: "rp", a: "P", b: "X", ohms: params.rpot }];
  const charge: ResistiveNetwork = {
    fixed: { VCC: params.vs, GND: 0 },
    free: nodes,
    edges: [{ id: "ra", a: "VCC", b: "D", ohms: params.ra }, ...chain],
  };
  const discharge: ResistiveNetwork = { ...charge, edges: [...charge.edges, { id: "dis", a: "D", b: "GND", ohms: NE555_DIS_OHMS }] };
  const c = theveninAt(charge, "X");
  const d = theveninAt(discharge, "X");
  return { mode: "astable", vGnd: 0, vS: params.vs, capacitanceF: params.c, charge: { vInf: c.vth, rOhms: c.rth }, discharge: { vInf: d.vth, rOhms: d.rth } };
}

function durations(model: Ne555TimingModel, horizonS = 0.5): number[] {
  return ne555Phases(model, horizonS).map((p) => p.t1 - p.t0);
}

describe("W1e sidecar: the shunt-inclusive Thevenin and the NE555 phases (NE555 point 13.3)", () => {
  it("F-A, the SAP-1 clock astable (RA 1k, RB 1k + RPOT 1 mOhm, C 1 uF): first HIGH, t_high, t_low and T from periods2.py", () => {
    const [first, low, high] = durations(astableModel({ ra: 1000, rb: 1000, rpot: 0.001, c: 1e-6, vs: 5 }));
    expect(first).toBeCloseTo(2.19722569e-3, 10);
    expect(high).toBeCloseTo(1.38629506e-3, 10);
    expect(low).toBeCloseTo(0.71535129e-3, 10);
    expect(high + low).toBeCloseTo(2.10164636e-3, 10);
  });

  it("F-B, the SAP-1 display astable (RA 1k, RB 15k, C 100 nF)", () => {
    const [first, low, high] = durations(astableModel({ ra: 1000, rb: 15000, c: 1e-7, vs: 5 }));
    expect(first).toBeCloseTo(1.75777969e-3, 11);
    expect(high).toBeCloseTo(1.10903551e-3, 11);
    expect(low).toBeCloseTo(1.06320733e-3, 11);
    expect(high + low).toBeCloseTo(2.17224284e-3, 11);
  });

  it("F-C, brownout fixture 03's astable (RA 10k, RB 100k, C 1 uF)", () => {
    const [first, low, high] = durations(astableModel({ ra: 10000, rb: 100000, c: 1e-6, vs: 5 }));
    expect(first).toBeCloseTo(120.847365e-3, 9);
    expect(high).toBeCloseTo(76.2462012e-3, 9);
    expect(low).toBeCloseTo(69.4718384e-3, 9);
    expect(high + low).toBeCloseTo(145.71804e-3, 9);
  });

  it("F-D, the headless-step-ceiling astable (RA 4.7k, RB 10k, C 10 uF): T = 171.596843 ms", () => {
    const [, low, high] = durations(astableModel({ ra: 4700, rb: 10000, c: 1e-5, vs: 5 }));
    expect(high + low).toBeCloseTo(171.596843e-3, 8);
  });

  it("F-E, the SAP-1 manual monostable (R 1 MOhm, C 1 uF): the pulse is 1.09861319 s, and the shunt term is what moves it off ln(3) R C by 8.2e-7 relative", () => {
    const net: ResistiveNetwork = { fixed: { VCC: 5, GND: 0 }, free: ["X"], edges: [{ id: "r", a: "VCC", b: "X", ohms: 1e6 }] };
    const dis: ResistiveNetwork = { ...net, edges: [...net.edges, { id: "dis", a: "X", b: "GND", ohms: NE555_DIS_OHMS }] };
    const c = theveninAt(net, "X");
    const d = theveninAt(dis, "X");
    const pulse = ne555Phases({ mode: "monostable", vGnd: 0, vS: 5, capacitanceF: 1e-6, charge: { vInf: c.vth, rOhms: c.rth }, discharge: { vInf: d.vth, rOhms: d.rth } }, 5)[0]!;
    const measured = pulse.t1 - pulse.t0;
    expect(measured).toBeCloseTo(1.09861319, 8);
    const withoutShunts = ne555MonostablePulseWidth({ rOhms: 1e6, capacitanceF: 1e-6, supplyVoltsAtGnd: 5 });
    expect(withoutShunts).toBeCloseTo(1.09861229, 8);
    expect(Math.abs(measured - withoutShunts) / measured).toBeGreaterThan(5e-7);
  });

  it("F-F, R 100 kOhm C 1 uF re-triggered at 0.5 s: 0.109861238 s from power-on, 0.109851238 s from v0 = V_inf,d = 4.9995e-4 V", () => {
    const net: ResistiveNetwork = { fixed: { VCC: 5, GND: 0 }, free: ["X"], edges: [{ id: "r", a: "VCC", b: "X", ohms: 1e5 }] };
    const dis: ResistiveNetwork = { ...net, edges: [...net.edges, { id: "dis", a: "X", b: "GND", ohms: NE555_DIS_OHMS }] };
    const c = theveninAt(net, "X");
    const d = theveninAt(dis, "X");
    expect(d.vth).toBeCloseTo(4.9995e-4, 8);
    const phases = ne555Phases(
      { mode: "monostable", vGnd: 0, vS: 5, capacitanceF: 1e-6, charge: { vInf: c.vth, rOhms: c.rth }, discharge: { vInf: d.vth, rOhms: d.rth } },
      2,
      [{ t: 0.5, low: true }, { t: 0.501, low: false }],
    );
    const highs = phases.filter((p) => p.high).map((p) => p.t1 - p.t0);
    expect(highs).toHaveLength(2);
    expect(highs[0]).toBeCloseTo(0.109861238, 9);
    expect(highs[1]).toBeCloseTo(0.109851238, 9);
    expect(phases.find((p) => p.high && p.t0 > 0)!.t0).toBe(0.5);
  });

  it("a TRIG held low past the threshold crossing holds OUT HIGH until it is released (SET beats RESET), then resets at once", () => {
    const net: ResistiveNetwork = { fixed: { VCC: 5, GND: 0 }, free: ["X"], edges: [{ id: "r", a: "VCC", b: "X", ohms: 1e5 }] };
    const dis: ResistiveNetwork = { ...net, edges: [...net.edges, { id: "dis", a: "X", b: "GND", ohms: NE555_DIS_OHMS }] };
    const c = theveninAt(net, "X");
    const d = theveninAt(dis, "X");
    const model: Ne555TimingModel = { mode: "monostable", vGnd: 0, vS: 5, capacitanceF: 1e-6, charge: { vInf: c.vth, rOhms: c.rth }, discharge: { vInf: d.vth, rOhms: d.rth } };
    const phases = ne555Phases(model, 2, [{ t: 0.05, low: true }, { t: 0.3, low: false }]);
    const first = phases[0]!;
    // TRIG went low at 0.05 s, during the power-on pulse, and stays low past its natural end (0.1099 s).
    expect(first.high).toBe(true);
    expect(first.t1).toBe(0.3);
    expect(phases[1]!.high).toBe(false);
  });

  it("F-G, RST on 100 kOhm / 1 uF: OUT is LOW until RC ln(vS/(vS-0.5)) = 10.5360516 ms, and the first HIGH then starts from the capacitor's voltage at release", () => {
    const circuit = loadFixtureCircuit("33-ne555-rst-delay-74hc74");
    const sidecar = extractNe555Sidecar(circuit, { component: "u1", mode: "astable", capacitor: "ct", rstCapacitor: "cr" }, 0.03);
    const [hold, firstHigh] = sidecar.phases;
    expect(hold!.high).toBe(false);
    expect(hold!.t1).toBeCloseTo(10.5360516e-3, 10);
    // Independent recomputation of the first HIGH: v0 = V_inf,d (1 - exp(-t_rel / (Rd C))) then the charge side to 2/3 vS.
    const model = astableModel({ ra: 1000, rb: 1000, rpot: 0.001, c: 1e-6, vs: 5 });
    const v0 = model.discharge.vInf * (1 - Math.exp(-hold!.t1 / (model.discharge.rOhms * 1e-6)));
    expect(firstHigh!.v0).toBeCloseTo(v0, 9);
    expect(firstHigh!.t1 - firstHigh!.t0).toBeCloseTo(rcCrossingTime(v0, 10 / 3, model.charge.vInf, model.charge.rOhms * 1e-6), 9);
  });

  it("refuses a release that leaves the capacitor above vL (the latch would hold, which the model does not cover)", () => {
    const m = astableModel({ ra: 1000, rb: 1000, c: 1e-6, vs: 5 });
    expect(() => ne555Phases({ ...m, startVolts: 2, releaseS: 1e-9 }, 1)).toThrow(/would hold/);
  });

  it("extractNe555Sidecar reads fixture 31 (the SAP-1 clock with an LED on OUT) and reproduces F-A", () => {
    const sidecar = extractNe555Sidecar(loadFixtureCircuit("31-ne555-astable-led"), { component: "u1", mode: "astable", capacitor: "ct" }, 0.02);
    expect(sidecar.phases[0]!.t1).toBeCloseTo(2.19722569e-3, 10);
    const at = sidecar.at(3.6057245e-3);
    // Mid-HIGH of the second cycle: exactly 2.6430 V at the capacitor and 3.8215 V at the DIS node.
    expect(Object.values(at.netV).sort((a, b) => a - b)[1]).toBeCloseTo(2.642978, 5);
    expect(Object.values(at.netV).sort((a, b) => a - b)[2]).toBeCloseTo(3.821489, 5);
    // The capacitor's and the resistors' currents, the same 1.1785 mA at mid-HIGH.
    expect(at.elementI.ra).toBeCloseTo(1.178511e-3, 8);
    expect(at.elementI.ct).toBeCloseTo(1.178511e-3, 8);
  });

  it("the detector: dropping RA from the discharge network gives the SAP-1 clock 2.086374 ms, off the true 2.101646 ms by 0.7%", () => {
    const circuit = loadFixtureCircuit("31-ne555-astable-led");
    const spec = { component: "u1", mode: "astable", capacitor: "ct" } as const;
    const wrong = extractNe555Sidecar(circuit, spec, 0.02, { dischargeDropsResistor: "ra" });
    const right = extractNe555Sidecar(circuit, spec, 0.02);
    const period = (phases: readonly { t1: number; t0: number }[]) => phases[1]!.t1 - phases[1]!.t0 + (phases[2]!.t1 - phases[2]!.t0);
    expect(period(wrong.phases)).toBeCloseTo(2.086374e-3, 9);
    expect(Math.abs(period(wrong.phases) - period(right.phases)) / period(right.phases)).toBeGreaterThan(5e-3);
  });

  it("detectNe555Specs finds an astable, a monostable and an RST RC in the W1e fixtures, and nothing in a bistable", () => {
    expect(detectNe555Specs(loadFixtureCircuit("31-ne555-astable-led"))).toEqual([{ component: "u1", mode: "astable", capacitor: "ct" }]);
    expect(detectNe555Specs(loadFixtureCircuit("32-ne555-monostable"))).toEqual([{ component: "u1", mode: "monostable", capacitor: "ct" }]);
    expect(detectNe555Specs(loadFixtureCircuit("33-ne555-rst-delay-74hc74"))).toEqual([{ component: "u1", mode: "astable", capacitor: "ct", rstCapacitor: "cr" }]);
    const bistable = loadFixtureCircuit("32-ne555-monostable");
    // THR (pin 6) on the datum, DIS open: neither TRIG = THR nor THR = DIS.
    const rewired: SimCircuit = { ...bistable, wires: bistable.wires.filter((w) => !(w.from_component === "u1" && w.from_pin === "7" && w.to_pin === "6")).filter((w) => !(w.from_pin === "6" && w.to_component === "ct")).concat([{ from_component: "u1", from_pin: "6", to_component: "vcc", to_pin: "neg" }]) };
    expect(detectNe555Specs(rewired)).toEqual([]);
  });

  it("extractNe555Sidecar refuses what the model does not cover: a foreign element on the timing node, an RST that is neither on the supply nor an RC", () => {
    const base = loadFixtureCircuit("31-ne555-astable-led");
    const spec = { component: "u1", mode: "astable", capacitor: "ct" } as const;
    const withLed: SimCircuit = {
      components: [...base.components, { id: "rx", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1e6 } }, { id: "dx", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: {} }],
      wires: [...base.wires, { from_component: "ct", from_pin: "a", to_component: "rx", to_pin: "a" }, { from_component: "rx", from_pin: "b", to_component: "dx", to_pin: "a" }, { from_component: "dx", from_pin: "k", to_component: "u1", to_pin: "1" }],
    };
    expect(() => extractNe555Sidecar(withLed, spec, 0.01)).toThrow(/does not cover/);
    const rstOpen: SimCircuit = { ...base, wires: base.wires.filter((w) => !(w.from_pin === "4" || w.to_pin === "4")) };
    expect(() => extractNe555Sidecar(rstOpen, spec, 0.01)).toThrow(/RST/);
  });
});

// ─── The lossy timing capacitor (the "555 LED Chaser" corpus FAIL) ───────────
//
// The chaser's timing capacitor carries `catalogUid: "capacitor"` on the
// COMPONENT, which classic and the core both resolve to ESR 1 ohm and 10 Mohm
// leakage. The numbers are FXJ's model2 (ESR in series with C, leakage across
// the terminals, the 1e-12 S shunts on DIS and the capacitor node), its 2x2
// nodal solve redone in exact rationals for more digits; the core reproduces
// them to 1e-9 and classic at a fine fixed trapezoidal step agrees.

/** Fixture 31 rewired as the chaser's timing network: RA 4.7 k, RB 10 k straight to the capacitor (no RPOT), C 10 uF. `capacitor` overrides the timing capacitor's catalogUid and params. */
function chaserTiming(capacitor: { catalogUid?: string; params?: Record<string, number> } = { catalogUid: "capacitor" }): SimCircuit {
  const base = loadFixtureCircuit("31-ne555-astable-led");
  const components = base.components
    .filter((c) => c.id !== "rpot")
    .map((c) => {
      if (c.id === "ra") return { ...c, params: { resistance: 4700 } };
      if (c.id === "rb") return { ...c, params: { resistance: 10000 } };
      if (c.id !== "ct") return c;
      const ct = { ...c, params: { capacitance: 9.999999999999999e-6, ...capacitor.params } };
      return capacitor.catalogUid === undefined ? ct : { ...ct, catalogUid: capacitor.catalogUid };
    });
  const wires = base.wires
    .filter((w) => w.from_component !== "rpot" && w.to_component !== "rpot")
    .concat([{ from_component: "rb", from_pin: "b", to_component: "ct", to_pin: "a" }]);
  return { ...base, components, wires };
}

const CHASER_SPEC = { component: "u1", mode: "astable", capacitor: "ct" } as const;

function firstThree(phases: readonly { t0: number; t1: number }[]): number[] {
  return phases.slice(0, 3).map((p) => p.t1 - p.t0);
}

function expectRelative(actual: readonly number[], want: readonly number[], rel: number): void {
  want.forEach((w, i) => expect(Math.abs(actual[i]! - w) / w, `phase ${i}: ${actual[i]} against ${w}`).toBeLessThan(rel));
}

describe("the NE555 sidecar on a lossy timing capacitor (the chaser's catalogUid \"capacitor\")", () => {
  const LOSSY = [161.6921553387e-3, 69.62773935e-3, 102.0569749489e-3];
  const IDEAL = [161.4960095211e-3, 69.704204502e-3, 101.8926381691e-3];

  it("reads catalogUid from the component, resolves ESR 1 ohm and 10 Mohm leakage, and times the phases as FXJ's model2: 161.6921553 / 69.6277394 / 102.0569749 ms", () => {
    const sidecar = extractNe555Sidecar(chaserTiming(), CHASER_SPEC, 1);
    expect(sidecar.model.esrOhms).toBe(1);
    expectRelative(firstThree(sidecar.phases), LOSSY, 1e-9);
  });

  it("authored esr and leakageResistance params model the same capacitor (an authored param wins, as in classic's modelParam)", () => {
    const sidecar = extractNe555Sidecar(chaserTiming({ params: { esr: 1, leakageResistance: 1e7 } }), CHASER_SPEC, 1);
    expectRelative(firstThree(sidecar.phases), LOSSY, 1e-9);
  });

  it("a capacitor with no catalog identity and no loss params stays ideal: 161.4960095 / 69.7042045 / 101.8926382 ms", () => {
    const sidecar = extractNe555Sidecar(chaserTiming({}), CHASER_SPEC, 1);
    expect(sidecar.model.esrOhms ?? 0).toBe(0);
    expectRelative(firstThree(sidecar.phases), IDEAL, 1e-9);
  });

  it("a uid the catalog does not hold falls back to ideal, as resolveCapacitorParams does", () => {
    const sidecar = extractNe555Sidecar(chaserTiming({ catalogUid: "not-a-real-uid" }), CHASER_SPEC, 1);
    expectRelative(firstThree(sidecar.phases), IDEAL, 1e-9);
  });

  it("the seed: the same sidecar with its leakage term dropped (161.4969953 ms) is off the true first HIGH by 1.2e-3", () => {
    const seeded = extractNe555Sidecar(chaserTiming(), CHASER_SPEC, 1, { dropCapacitorLeakage: true });
    expectRelative(firstThree(seeded.phases), [161.4969953043e-3, 69.6977514425e-3, 101.8822727192e-3], 1e-9);
    expect(Math.abs(seeded.phases[0]!.t1 - LOSSY[0]!) / LOSSY[0]!).toBeGreaterThan(1e-3);
  });

  it("at(t): the capacitor node reads the terminal (vc + ESR i), and the capacitor's current is the series current plus the leakage", () => {
    const sidecar = extractNe555Sidecar(chaserTiming(), CHASER_SPEC, 1);
    const mid = sidecar.phases[2]!;
    const t = (mid.t0 + mid.t1) / 2;
    const vc = rcVoltageAt(mid.v0, mid.vInf, mid.tauS, t - mid.t0);
    const series = (mid.vInf - vc) / (mid.tauS / 9.999999999999999e-6);
    const terminal = vc + 1 * series;
    const at = sidecar.at(t);
    const capNet = Object.entries(at.netV).find(([, v]) => Math.abs(v - terminal) < 1e-12);
    expect(capNet, "a timing net reads the terminal voltage").toBeDefined();
    expect(at.elementI.ct).toBeCloseTo(series + terminal / 1e7, 15);
    expect(at.elementI["ct#leakage"]).toBeUndefined();
    expect(sidecar.resistors.map((r) => r.id).sort()).toEqual(["ra", "rb"]);
  });

  it("a lossy RST capacitor moves the release to where its terminal crosses 0.5 V through ESR and leakage", () => {
    const base = loadFixtureCircuit("33-ne555-rst-delay-74hc74");
    const circuit: SimCircuit = { ...base, components: base.components.map((c) => (c.id === "cr" ? { ...c, catalogUid: "capacitor" } : c)) };
    const sidecar = extractNe555Sidecar(circuit, { component: "u1", mode: "astable", capacitor: "ct", rstCapacitor: "cr" }, 0.03);
    // RR 100 k from 5 V, leakage 10 M across C 1 uF, ESR 1 ohm: the Thevenin (shunt included) seen by the series ESR-C branch.
    const net: ResistiveNetwork = { fixed: { VCC: 5, GND: 0 }, free: ["X"], edges: [{ id: "rr", a: "VCC", b: "X", ohms: 1e5 }, { id: "leak", a: "X", b: "GND", ohms: 1e7 }] };
    const { vth, rth } = theveninAt(net, "X");
    const vcAtRelease = (0.5 * (rth + 1) - 1 * vth) / rth;
    expect(sidecar.phases[0]!.t1).toBeCloseTo(rcCrossingTime(0, vcAtRelease, vth, (rth + 1) * 1e-6), 12);
    expect(Math.abs(sidecar.phases[0]!.t1 - 10.5360516e-3)).toBeGreaterThan(1e-6);
  });
});

describe("the LED sidecars' catalog guards read the component's catalogUid", () => {
  it("evaluateLedStatic refuses a device whose catalogUid sits on the component (where the corpus puts it), not in params", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "gnd", kind: "ground", pins: [{ id: "gnd" }], params: {} },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
        { id: "d1", kind: "led", catalogUid: "led-red", pins: [{ id: "a" }, { id: "k" }], params: { vf: 1.8, iRated: 0.02 } },
      ],
      wires: [
        { from_component: "vcc", from_pin: "neg", to_component: "gnd", to_pin: "gnd" },
        { from_component: "vcc", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "d1", to_pin: "a" },
        { from_component: "d1", from_pin: "k", to_component: "gnd", to_pin: "gnd" },
      ],
    };
    const model = {
      kind: "led-static" as const,
      cells: [{ device: "d1", pin: "a", resistor: "r1", source: "vcc", orientation: "anode-node" as const, vfParam: "vf", vfDefault: 1.8 }],
      devices: [{ device: "d1" }],
    };
    expect(() => evaluateLedStatic(circuit, model)).toThrow(/catalogUid/);
    const plain: SimCircuit = { ...circuit, components: circuit.components.map(({ catalogUid: _drop, ...c }) => c) };
    expect(() => evaluateLedStatic(plain, model)).not.toThrow();
  });

  it("ledRcVoltages refuses a capacitor whose catalog identity makes it lossy, and keeps an ideal one", () => {
    const base = loadFixtureCircuit("35-led-rc-stepped");
    const model = { kind: "led-rc" as const, source: "vcc", resistor: "r1", capacitor: "c1", device: "d1", instantsS: [] };
    const lossy: SimCircuit = { ...base, components: base.components.map((c) => (c.id === "c1" ? { ...c, catalogUid: "capacitor" } : c)) };
    expect(() => ledRcVoltages(lossy, model, [1e-3])).toThrow(/ideal/);
    expect(() => ledRcVoltages(base, model, [1e-3])).not.toThrow();
  });
});

// ─── W1e: the LED junction sidecars (LEDs point 2, 8 and 13) ──────────────────
//
// The numbers are from `drled-sidecars.mjs` (bisection on the node's KCL with
// GMIN_D and the shunt), 25 C unless stated.

describe("W1e sidecar: the LED junction law and its one-node solves", () => {
  const law = (vf25: number, tempC = 25, iRated = 0.02, n = 2) => ledLaw({ vf25, iRated, n, tempC });

  it("F1: a red LED (vf 1.8) through 220 ohm from 5 V sits at 1.783894181 V with 14.61866281 mA", () => {
    const r = ledSeriesNode(law(1.8), 5, 220, "anode-node");
    expect(r.nodeV).toBeCloseTo(1.783894181, 9);
    expect(r.ledAmps).toBeCloseTo(14.61866281e-3, 11);
    expect(r.resistorAmps).toBeCloseTo((5 - r.nodeV) / 220, 15);
  });

  it("F2: red, green, blue and white through 150 ohm (green sits at exactly Vf, where I = iRated - Is)", () => {
    const want: Array<[number, number, number]> = [[1.8, 1.803263884, 21.3115741e-3], [2.0, 2.0, 20.0e-3], [3.0, 2.979684437, 13.4687704e-3], [3.2, 3.174474705, 12.1701686e-3]];
    for (const [vf, v, i] of want) {
      const r = ledSeriesNode(law(vf), 5, 150, "anode-node");
      expect(r.nodeV).toBeCloseTo(v, 8);
      expect(r.ledAmps).toBeCloseTo(i, 9);
    }
  });

  it("F3: the junction voltage falls with ambient temperature (the fixed -2 mV/C tempco): 1.8556, 1.7073, 1.5711, 1.4808 V at -40, 25, 85, 125 C through 1 kohm", () => {
    const want: Array<[number, number]> = [[-40, 1.855657028], [25, 1.707299923], [85, 1.571145893], [125, 1.480773665]];
    for (const [t, v] of want) expect(ledSeriesNode(law(1.8, t), 5, 1000, "anode-node").nodeV).toBeCloseTo(v, 8);
  });

  it("F4: across 5 V through 0.1 ohm the load line meets the tangent past vSat = 2.056925791 V, at 2.447429482 V and 25.5257052 A", () => {
    const l = law(1.8);
    expect(l.vSat).toBeCloseTo(2.056925791, 8);
    const r = ledSeriesNode(l, 5, 0.1, "anode-node");
    expect(r.nodeV).toBeCloseTo(2.447429482, 8);
    expect(r.ledAmps).toBeCloseTo(25.5257052, 5);
    // The tangent line itself: I(v) is linear above vSat with the slope of the exponential at vSat.
    const slope = (l.is * Math.exp(l.vSat / l.vtn)) / l.vtn;
    expect(ledCurrent(l, l.vSat + 1) - ledCurrent(l, l.vSat)).toBeCloseTo(slope, 6);
  });

  it("F7: a seg7_cc showing 7 (a b c through 330 ohm, common to the datum through a shared 10 ohm): common 0.2536861645 V, anode 2.20945219 V, 8.456205481 mA per segment", () => {
    const l = law(2.0);
    const r = ledSharedCommon([l, l, l], 5, [330, 330, 330], 10);
    expect(r.commonV).toBeCloseTo(0.2536861645, 9);
    expect(r.nodeV[0]).toBeCloseTo(2.20945219, 8);
    expect(r.ledAmps[0]).toBeCloseTo(8.456205481e-3, 11);
    expect(r.commonAmps).toBeCloseTo(0.02536861645, 10);
  });

  it("a cathode-node cell (a seg7_ca segment: anode fixed at 5 V, resistor to the datum) is the mirror of the anode-node one", () => {
    const l = law(2.0);
    const a = ledSeriesNode(l, 5, 330, "anode-node");
    const c = ledSeriesNode(l, 5, 330, "cathode-node");
    expect(c.ledAmps).toBeCloseTo(a.ledAmps, 9);
    expect(c.nodeV).toBeCloseTo(5 - a.nodeV, 9);
  });

  it("the RC-fed LED's ODE settles on the one-node answer (5 V, 1 kohm, 85 C), and starts at 0 V", () => {
    const circuit = loadFixtureCircuit("35-led-rc-stepped");
    const [early, late] = ledRcVoltages(circuit, { kind: "led-rc", source: "vcc", resistor: "r1", capacitor: "c1", device: "d1", instantsS: [] }, [1e-3, 0.2]);
    expect(early!).toBeCloseTo(5 * (1 - Math.exp(-1e-3 / (1000 * 1e-5))) , 2);
    expect(late!).toBeCloseTo(ledSeriesNode(law(1.8, 85), 5, 1000, "anode-node").nodeV, 6);
  });
});

describe("W1e sidecar: evaluateLedStatic against the core", () => {
  beforeAll(async () => {
    await initCore();
  });

  it("F4 through the core (no classic: classic latches this LED as failed within nanoseconds at 25 A): the tangent region answers exactly", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "gnd", kind: "ground", pins: [{ id: "gnd" }], params: {} },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 0.1 } },
        { id: "d1", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { vf: 1.8, iRated: 0.02 } },
      ],
      wires: [
        { from_component: "vcc", from_pin: "neg", to_component: "gnd", to_pin: "gnd" },
        { from_component: "vcc", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "d1", to_pin: "a" },
        { from_component: "d1", from_pin: "k", to_component: "gnd", to_pin: "gnd" },
      ],
    };
    const want = evaluateLedStatic(circuit, {
      kind: "led-static",
      cells: [{ device: "d1", pin: "a", resistor: "r1", source: "vcc", orientation: "anode-node", vfParam: "vf", vfDefault: 1.8 }],
      devices: [{ device: "d1" }],
    });
    const engine = new CoreEngine();
    engine.load(circuit);
    const netId = engine.getNetIdForPin("d1", "a")!;
    expect(engine.getNetV()[netId]).toBeCloseTo(want.netV[netId]!, 6);
    expect(engine.getNetV()[netId]).toBeCloseTo(2.447429482, 6);
    expect(engine.getElementI().d1).toBeCloseTo(25.5257052, 4);
    expect(engine.getElementI().d1).toBeCloseTo(want.elementI.d1!, 4);
  });
});
