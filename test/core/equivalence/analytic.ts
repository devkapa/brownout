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

/**
 * `rcCrossingTime`'s counterpart: evaluates the same exponential directly
 * instead of inverting it for a crossing. `v(t) = vAsymptote + (v0 -
 * vAsymptote) * exp(-t/tau)`, `t` measured from the instant `v(t) = v0`.
 * W1b's "the exact exponential" reference (this worker's brief, item 2):
 * a fixture's expected voltage at a chosen sample time, for a single ideal
 * (or lossy — see `rcTerminalThevenin` below) capacitor's closed-form
 * partition.
 */
export function rcVoltageAt(v0: number, vAsymptote: number, tau: number, t: number): number {
  if (!(tau > 0)) {
    throw new Error(`rcVoltageAt: tau must be > 0, got ${String(tau)}.`);
  }
  return vAsymptote + (v0 - vAsymptote) * Math.exp(-t / tau);
}

export interface RcLossyTerminalParams {
  /** Source voltage driving the network (ground-referenced), volts. */
  readonly sourceVolts: number;
  /** External series resistance between the source and the capacitor's own terminal, ohms. */
  readonly seriesOhms: number;
  /** Capacitance, farads. */
  readonly capacitanceF: number;
  /** The capacitor device's own ESR (spec §7.4's series loss), ohms. 0 is ideal (no series loss). */
  readonly esrOhms: number;
  /** The capacitor device's own leakage resistance (spec §7.4's parallel loss, across ITS OWN terminals), ohms. `Infinity` (or omitted) is ideal (no leakage). */
  readonly leakageOhms?: number;
}

/**
 * The capacitor's OWN TERMINAL voltage (`pins[0]` to `pins[1]` — what a
 * `netV`/`elementI` readout sees) for a single capacitor with ESR and
 * leakage, external series resistance to an ideal source, everything else
 * ideal, starting from an UNCHARGED capacitor (the internal `vc` state — the
 * voltage across C alone — is 0 at t=0; true of a fresh load, never a
 * carried-forward one). `tau`/`vAsymptote`/`v0AtUncharged` feed
 * `rcVoltageAt`/`rcCrossingTime` above.
 *
 * Model (spec §7.4): the device is `esr` in series with `C`, and `leakage`
 * as a resistor directly across the device's own two external terminals (in
 * parallel with the esr+C branch) — so from the rest of the circuit's point
 * of view, the leakage resistor is NOT inside the "capacitor branch" that
 * `esr`/`C` sit in; it is a second, parallel path between the same two
 * nodes. Removing the esr+C branch, the remaining network (source, series
 * resistor, leakage resistor) is a plain two-resistor divider — a pure
 * Thevenin source as seen by the esr+C branch:
 *   Rth = seriesOhms*leakageOhms / (seriesOhms+leakageOhms)   (Rth = seriesOhms when leakageOhms is Infinity/absent)
 *   Vth = sourceVolts * leakageOhms / (seriesOhms+leakageOhms) (Vth = sourceVolts when leakageOhms is Infinity/absent)
 * The esr+C branch then sees a plain series RC toward Vth through (Rth+esr),
 * so the INTERNAL state `vc` follows a plain single exponential toward Vth
 * with that time constant.
 *
 * IMPORTANT: the device's TERMINAL voltage `v` is NOT the same trajectory
 * shifted to start at 0, unless `esrOhms` is 0. `v` and `vc` relate by `v =
 * i*esr + vc` with `i = (V*g1 - v*(g1+g2))/... ` — solving the instantaneous
 * KCL at the terminal node for `v` given the current `vc` gives `v = (V*g1 +
 * vc/esr) / (1/esr + g1 + g2)` (`g1=1/seriesOhms`, `g2=1/leakageOhms`, and
 * `1/esr` reads as `Infinity` when `esrOhms` is 0, which correctly forces
 * `v = vc` in that limit — the ideal case). `v` turns out to be an AFFINE
 * image of `vc` (same derivation as above, algebra omitted), so it is STILL
 * a plain single exponential with the identical `tau`/`vAsymptote` — but its
 * OWN starting value `v(0)` is generally nonzero even though `vc(0) = 0`,
 * because `esr` is a plain resistor: it passes an instantaneous current the
 * moment the source is applied, before the capacitor has had any time to
 * charge at all. `v0AtUncharged` is exactly this `v(0)` (with `vc(0) = 0`
 * substituted above) — use it, not 0, as `rcVoltageAt`'s `v0` argument
 * whenever `esrOhms > 0`. (Caught by this worker's own brute-force ODE
 * cross-check in analytic.test.ts: assuming `v0 = 0` for a nonzero-ESR case
 * is off by roughly 18% at t ~ 0.2*tau for this module's own fixture 12
 * numbers — a real bug this cross-check exists to catch, not a hypothetical
 * one.)
 */
export function rcTerminalThevenin(
  params: RcLossyTerminalParams,
): { readonly tau: number; readonly vAsymptote: number; readonly v0AtUncharged: number } {
  const { sourceVolts, seriesOhms, capacitanceF, esrOhms, leakageOhms } = params;
  if (!(seriesOhms > 0)) throw new Error(`rcTerminalThevenin: seriesOhms must be > 0, got ${String(seriesOhms)}.`);
  if (!(capacitanceF > 0)) throw new Error(`rcTerminalThevenin: capacitanceF must be > 0, got ${String(capacitanceF)}.`);
  if (esrOhms < 0) throw new Error(`rcTerminalThevenin: esrOhms must be >= 0, got ${String(esrOhms)}.`);
  const leakage = leakageOhms ?? Number.POSITIVE_INFINITY;
  if (!(leakage > 0)) throw new Error(`rcTerminalThevenin: leakageOhms must be > 0, got ${String(leakage)}.`);

  // g2 is 0 when leakage is Infinity, and ge is Infinity when esrOhms is 0 —
  // both fall out of plain IEEE-754 division (1/Infinity = 0, x/Infinity =
  // 0 for finite x), so neither needs its own branch below.
  const g1 = 1 / seriesOhms;
  const g2 = 1 / leakage;
  const ge = 1 / esrOhms;

  const rTh = 1 / (g1 + g2);
  const vTh = (sourceVolts * g1) / (g1 + g2);
  const tau = capacitanceF * (rTh + esrOhms);
  // v(0) with vc(0)=0 substituted into v = (V*g1 + vc*ge) / (ge+g1+g2).
  const v0AtUncharged = (sourceVolts * g1) / (ge + g1 + g2);
  return { tau, vAsymptote: vTh, v0AtUncharged };
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

export interface RcLadderParams {
  /** Ideal source voltage (ground-referenced), volts. */
  readonly sourceVolts: number;
  /** Series resistance from the source to node 1, ohms. */
  readonly r1Ohms: number;
  /** Capacitance from node 1 to ground, farads. */
  readonly c1F: number;
  /** Series resistance from node 1 to node 2, ohms. */
  readonly r2Ohms: number;
  /** Capacitance from node 2 to ground, farads. */
  readonly c2F: number;
}

export interface RcLadderState {
  readonly v1: number;
  readonly v2: number;
}

/**
 * Exact two-capacitor RC-ladder trajectory: `sourceVolts --r1Ohms-- node1
 * --r2Ohms-- node2`, `c1F` from node1 to ground, `c2F` from node2 to ground
 * (both capacitors ideal — no ESR, no leakage), both starting uncharged
 * (`v1(0) = v2(0) = 0`).
 *
 * This is decision 14's "more than one capacitor in one linear partition"
 * case (the W1 plan's `solve/stepped.rs`, not the single-capacitor closed
 * form) — node1 and node2 are joined into ONE partition by r2Ohms, and it
 * contains two capacitors. The stepped solver is explicitly NOT claimed
 * exact (decision 14: "within classic's tolerance but not exact"), so this
 * reference exists to bound how far a numerically-converged answer may
 * legitimately sit from the true trajectory — not to hold the core to the
 * single-capacitor closed form's 1e-9 relative bar.
 *
 * DERIVATION. KCL at node1 and node2 gives the linear system `dx/dt = A x +
 * b`, `x = (v1, v2)`:
 * ```
 * A = [ -(1/r1+1/r2)/c1,      1/(r2*c1) ]
 *     [       1/(r2*c2), -1/(r2*c2)     ]
 * b = (sourceVolts/(r1*c1), 0)
 * ```
 * whose steady state is `v1 = v2 = sourceVolts`: once both capacitors are
 * fully charged no current flows anywhere in the ladder, so there is no IR
 * drop across either resistor. Solved here by direct eigendecomposition of
 * the real 2x2 matrix `A` — always real, non-positive eigenvalues for a
 * passive RC network — rather than pulled from a linear-algebra dependency
 * brownout does not have; cross-checked in this module's own tests against
 * an independent RK4 numerical integration, so a transcription slip in the
 * eigendecomposition algebra below would be caught separately from this
 * formula itself.
 */
export function rcLadderVoltagesAt(params: RcLadderParams, t: number): RcLadderState {
  const { sourceVolts: v, r1Ohms: r1, c1F: c1, r2Ohms: r2, c2F: c2 } = params;
  if (!(r1 > 0) || !(r2 > 0)) throw new Error("rcLadderVoltagesAt: r1Ohms and r2Ohms must be > 0.");
  if (!(c1 > 0) || !(c2 > 0)) throw new Error("rcLadderVoltagesAt: c1F and c2F must be > 0.");

  const a11 = -(1 / r1 + 1 / r2) / c1;
  const a12 = 1 / (r2 * c1);
  const a21 = 1 / (r2 * c2);
  const a22 = -1 / (r2 * c2);

  const trace = a11 + a22;
  const det = a11 * a22 - a12 * a21;
  const discriminant = trace * trace - 4 * det;
  if (discriminant < 0) {
    // Not expected for any passive RC ladder (A is similar to a symmetric
    // negative-semidefinite matrix), but guarded rather than silently
    // producing a complex (NaN-in-JS) result.
    throw new Error("rcLadderVoltagesAt: complex eigenvalues — not expected for a passive RC ladder.");
  }
  const sqrtDiscriminant = Math.sqrt(discriminant);
  const lambda1 = (trace + sqrtDiscriminant) / 2;
  const lambda2 = (trace - sqrtDiscriminant) / 2;
  if (Math.abs(lambda1 - lambda2) < 1e-9 * Math.max(1, Math.abs(trace))) {
    // Repeated eigenvalues need a generalized-eigenvector (Jordan-block) form
    // this function does not implement — avoid the degenerate divide below
    // producing a near-singular, numerically garbage answer instead.
    throw new Error(
      "rcLadderVoltagesAt: near-repeated eigenvalues — choose r1Ohms/c1F/r2Ohms/c2F that are not the degenerate " +
        "symmetric case this function does not handle.",
    );
  }

  // Eigenvector for eigenvalue lambda: (A - lambda*I) e = 0. Row 1 gives
  // (a11-lambda)*ex + a12*ey = 0; a12 > 0 always here, so ex=a12, ey=lambda-a11
  // is always a valid (nonzero) solution.
  const eigenvector = (lambda: number): readonly [number, number] => [a12, lambda - a11];
  const e1 = eigenvector(lambda1);
  const e2 = eigenvector(lambda2);

  // x(t) = (v,v) + k1*e1*exp(lambda1*t) + k2*e2*exp(lambda2*t); solve
  // (k1,k2) from x(0)=(0,0): k1*e1 + k2*e2 = -(v,v), by Cramer's rule on
  // M = [[e1[0],e2[0]],[e1[1],e2[1]]], rhs = (-v,-v).
  const detM = e1[0] * e2[1] - e2[0] * e1[1];
  const k1 = (-v * e2[1] - e2[0] * -v) / detM; // = v*(e2[0]-e2[1]) / detM
  const k2 = (e1[0] * -v - -v * e1[1]) / detM; // = v*(e1[1]-e1[0]) / detM

  const mode1 = k1 * Math.exp(lambda1 * t);
  const mode2 = k2 * Math.exp(lambda2 * t);
  return {
    v1: v + mode1 * e1[0] + mode2 * e2[0],
    v2: v + mode1 * e1[1] + mode2 * e2[1],
  };
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
