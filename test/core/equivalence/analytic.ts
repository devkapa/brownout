/**
 * Analytic-answer references (gate 2e). For a few quantities the plan makes
 * the core MORE accurate than classic by design — decision 7's exact 555
 * crossings and decision 3's exact RC timing — so comparing core-against-
 * classic directly on those quantities can fail even when the core is
 * correct: classic's own discretisation is the thing that is "wrong" (by an
 * expected, bounded amount). Gate 2e's rule: compute the closed-form answer
 * from the model's own equations, then pass when the CORE is within
 * tolerance of it, recording the reason (whether or not classic also is).
 *
 * These formulas are the sidecar generator's job in production ("computed
 * from the model's own equations... never by core code") — this module is
 * that generator's reusable arithmetic, kept here so a fixture sidecar and
 * this harness's own tests share one implementation instead of two
 * hand-derived copies drifting apart.
 */

/**
 * Time for a first-order RC exponential `v(t) = vAsymptote + (v0 -
 * vAsymptote) * exp(-t/tau)` to cross `vTarget`, given it starts at `v0` at
 * t=0. Throws if `vTarget` is not actually between `v0` and `vAsymptote`
 * (exclusive) — such a crossing never happens, and returning a NaN or
 * negative time would be a worse failure mode than refusing to compute one.
 */
export function rcCrossingTime(v0: number, vTarget: number, vAsymptote: number, tau: number): number {
  if (!(tau > 0)) {
    throw new Error(`rcCrossingTime: tau must be > 0, got ${String(tau)}.`);
  }
  const num = vTarget - vAsymptote;
  const den = v0 - vAsymptote;
  if (den === 0 || num / den <= 0 || num / den >= 1) {
    throw new Error(
      `rcCrossingTime: vTarget=${vTarget} is not strictly between v0=${v0} and vAsymptote=${vAsymptote} — ` +
        "this exponential never crosses it.",
    );
  }
  return -tau * Math.log(num / den);
}

export interface Ne555AstableParams {
  readonly r1Ohms: number;
  readonly r2Ohms: number;
  readonly capacitanceF: number;
  /** vS measured GND-relative, per decision 7's comparator reference frame. */
  readonly supplyVoltsAtGnd: number;
  /** Series resistance in the discharge (DIS) path — core/README.md: "the 555 period includes DIS 10 Ω". Default 10. */
  readonly disResistanceOhms?: number;
  /** Upper comparator threshold as a fraction of vS. Default 2/3 (decision 7). */
  readonly upperFraction?: number;
  /** Lower comparator threshold as a fraction of vS. Default 1/3 (decision 7). */
  readonly lowerFraction?: number;
}

export interface Ne555AstableAnalytic {
  readonly chargeTimeS: number;
  readonly dischargeTimeS: number;
  readonly periodS: number;
  readonly frequencyHz: number;
}

/**
 * The 555 astable closed form under decision 7's exact-crossing semantics —
 * derived from classic's OWN linear model (spec section 5.2), not the
 * textbook formula:
 *
 * CHARGE (OUT HIGH): classic's DIS stamp adds nothing while HIGH ("open
 * collector released" — digital-ics.ts:216-218, spec §5.2 "DIS"), so the cap
 * charges through the plain series path R1+R2 toward vS. Crossing from vLow
 * to vHigh.
 *
 * DISCHARGE (OUT LOW): classic's DIS stamp adds a 10 ohm resistor from the
 * DIS node to GND (digital-ics.ts:216-218), but it does NOT disconnect R1 —
 * R1 still ties the DIS node to VCC the whole time (spec §5.2 cites this as
 * the literal 10 ohm at digital-ics.ts:217). So the DIS node is a three-way
 * junction (R1 to VCC, Rdis to GND, R2 to the cap): by Thevenin's theorem
 * (R1 and Rdis form a plain voltage divider from VCC to GND, unloaded by the
 * high-impedance R2 branch for the open-circuit voltage), the cap sees, via
 * R2:
 *   - Thevenin resistance  Rth = R1*Rdis/(R1+Rdis)
 *   - Thevenin source      va  = vS * Rdis/(R1+Rdis)
 * and discharges through (R2+Rth) toward va, crossing from vHigh to vLow —
 * NOT through (R2+Rdis) toward 0, which silently drops R1's contribution and
 * is off by about 0.7-1.1% depending on R1/R2/C (confirmed against the W1-0
 * spike's recorded exact value: 2.101645 ms for R1=R2=1kΩ, C=1µF, 5V, vs.
 * the R1-less formula's 2.086373 ms).
 *
 * Classic's own (discretised) period is still expected to differ from this
 * closed form by design (backward-Euler step-grid lag, headless-step-
 * ceiling.test.ts's "ran 1.5% slow") — that is exactly what this function
 * exists to compare the CORE against instead (gate 2e).
 */
export function ne555AstablePeriod(params: Ne555AstableParams): Ne555AstableAnalytic {
  const dis = params.disResistanceOhms ?? 10;
  const upper = params.upperFraction ?? 2 / 3;
  const lower = params.lowerFraction ?? 1 / 3;
  const vs = params.supplyVoltsAtGnd;
  const vLow = lower * vs;
  const vHigh = upper * vs;

  const tauCharge = (params.r1Ohms + params.r2Ohms) * params.capacitanceF;
  const chargeTimeS = rcCrossingTime(vLow, vHigh, vs, tauCharge);

  // Thevenin equivalent of R1 (to VCC) and Rdis (to GND) as seen from the DIS
  // node, with R2 carrying that into the cap — see the derivation above.
  const dischargeThevRes = (params.r1Ohms * dis) / (params.r1Ohms + dis);
  const dischargeThevVolts = vs * (dis / (params.r1Ohms + dis));
  const tauDischarge = (params.r2Ohms + dischargeThevRes) * params.capacitanceF;
  const dischargeTimeS = rcCrossingTime(vHigh, vLow, dischargeThevVolts, tauDischarge);

  const periodS = chargeTimeS + dischargeTimeS;
  return { chargeTimeS, dischargeTimeS, periodS, frequencyHz: 1 / periodS };
}

export interface Ne555MonostableParams {
  readonly rOhms: number;
  readonly capacitanceF: number;
  readonly supplyVoltsAtGnd: number;
  readonly upperFraction?: number;
}

/** The 555 monostable closed form: charging from 0 toward vS, crossing the upper threshold. With the default 2/3 fraction this is the familiar `ln(3) * R * C`. */
export function ne555MonostablePulseWidth(params: Ne555MonostableParams): number {
  const upper = params.upperFraction ?? 2 / 3;
  const vs = params.supplyVoltsAtGnd;
  const tau = params.rOhms * params.capacitanceF;
  return rcCrossingTime(0, upper * vs, vs, tau);
}

export interface AnalyticCheckResult {
  readonly status: "pass" | "fail";
  readonly analyticValue: number;
  readonly coreValue: number;
  readonly coreAbsError: number;
  readonly classicValue?: number;
  readonly classicAbsError?: number;
  readonly reason: string;
}

/**
 * Gate 2e's pass rule: the core must be within `absTol + relTol *
 * |analyticValue|` of the analytic answer. `classicValue`, when given, is
 * recorded for the report and never flips the verdict by MAGNITUDE — classic
 * being closer to or farther from the analytic answer than the core is
 * informational only (per gate 2e: "Where the core is closer to the
 * analytic answer than classic, the entry passes with the reason recorded
 * in the table"); the verdict on that axis depends only on the core's own
 * error. But a `classicValue` that is NaN or otherwise non-finite is not "far
 * off" — it means classic's own measurement never happened (no crossing
 * found, a period from an empty edge list), which is a broken comparison the
 * report cannot make any claim from. That fails the gate outright, the same
 * way a non-finite coreValue/analyticValue/tolerance does, rather than
 * silently reporting a NaN classicAbsError next to a "pass".
 */
export function checkAnalyticCrossing(params: {
  readonly coreValue: number;
  readonly analyticValue: number;
  readonly classicValue?: number;
  readonly absTol: number;
  readonly relTol: number;
}): AnalyticCheckResult {
  const { coreValue, analyticValue, classicValue, absTol, relTol } = params;
  const coreAbsError = Math.abs(coreValue - analyticValue);
  const tolerance = absTol + relTol * Math.abs(analyticValue);
  const classicAbsError = classicValue === undefined ? undefined : Math.abs(classicValue - analyticValue);

  // `!(coreAbsError <= tolerance)` rather than `coreAbsError > tolerance`:
  // a measurement that never happened (no crossing found, a period from an
  // empty edge list, 0/0) surfaces as NaN, and `NaN > tolerance` is always
  // false in JS — the `>` form would fall through to "pass" on exactly the
  // inputs this check exists to catch. `!(x <= y)` is true for any NaN
  // operand (coreValue, analyticValue, or a NaN/non-finite tolerance), so
  // this fails instead.
  if (!(coreAbsError <= tolerance)) {
    return {
      status: "fail",
      analyticValue,
      coreValue,
      coreAbsError,
      classicValue,
      classicAbsError,
      reason:
        `core error ${coreAbsError.toExponential(3)} exceeds tolerance ${tolerance.toExponential(3)} of the ` +
        `analytic answer ${analyticValue}`,
    };
  }

  // classicValue is optional — undefined just means "no classic comparison
  // was made", which is fine. But a classicValue that WAS supplied and is
  // NaN/non-finite (as opposed to merely a large, legitimately-far number)
  // means classic's own run never produced a value, so classicAbsError is
  // meaningless and must not be reported inside a "pass". This can't be
  // folded into the `coreAbsError <= tolerance` check above because
  // classicValue never feeds coreAbsError or tolerance — it's an isolated
  // input that needs its own finiteness check.
  if (classicValue !== undefined && !Number.isFinite(classicValue)) {
    return {
      status: "fail",
      analyticValue,
      coreValue,
      coreAbsError,
      classicValue,
      classicAbsError,
      reason: `classicValue ${String(classicValue)} is not a finite number — classic's own measurement never produced one, so it cannot be compared to the analytic answer`,
    };
  }

  const reason =
    classicAbsError === undefined
      ? `core is within tolerance ${tolerance.toExponential(3)} of the analytic answer (error ${coreAbsError.toExponential(3)})`
      : coreAbsError <= classicAbsError
        ? `core is within tolerance of the analytic answer and at least as close as classic ` +
          `(core error ${coreAbsError.toExponential(3)}, classic error ${classicAbsError.toExponential(3)})`
        : `core is within tolerance of the analytic answer (error ${coreAbsError.toExponential(3)}); classic ` +
          `happens to be closer (${classicAbsError.toExponential(3)}) but gate 2e only requires the core to ` +
          "be within tolerance";

  return { status: "pass", analyticValue, coreValue, coreAbsError, classicValue, classicAbsError, reason };
}
