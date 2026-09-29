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

import { buildCoreNodes, corePinKey } from "../../../src/core/compile/topology.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import type { StimulusPhase } from "./stimuli.js";
import type { AnalyticLedRcModel, AnalyticLedStaticModel, LedJunctionCell, Ne555FixtureSpec } from "./types.js";

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

/**
 * Rule 2a (core/README.md, "Kernel semantics: partitions and dynamic
 * solves"): a capacitor across two FIXED nodes, `dV = V(pins[0]) -
 * V(pins[1])` volts apart FOREVER — no fixed-node voltage is ever live in
 * W1b (a `voltage_source` has no live params), so unlike every other
 * closed-form reference in this file there is no re-aiming, no crossing, and
 * no dependence on anything but `t` itself. Mirrors `core/src/solve/mod.rs`'s
 * `FixedPairCapacitor::current_at` exactly (that Rust function's own doc
 * comment explains why the series term is computed directly in this form,
 * rather than through an intermediate `vc`, to avoid catastrophic
 * cancellation once `vc` has relaxed close to `dV`):
 * - `esrOhms > 0`: series current `(dV/esrOhms) * exp(-t / (esrOhms *
 *   capacitanceF))`, from `vc = 0` at `t = 0` (the load instant — `t` here
 *   is already relative to `START_TIME`, matching `rcVoltageAt`'s own `t`
 *   convention).
 * - `esrOhms === 0`: `vc = dV` from the load instant on, so the series
 *   current is exactly `0`.
 * Either way, `leakageOhms`'s current (`dV/leakageOhms`, `0` when
 * `leakageOhms` is `0` — the record-layout table's OWN sentinel for "none",
 * NOT `rcTerminalThevenin`'s different Infinity-when-absent convention,
 * since this quantity is never combined with the universal node shunt: `dV`
 * is exact regardless of it, both terminals being fixed nodes the shunt
 * cannot move) is constant for as long as `dV` is — forever.
 */
export function fixedPairCapacitorCurrentAt(dV: number, capacitanceF: number, esrOhms: number, leakageOhms: number, t: number): number {
  if (!(capacitanceF > 0)) throw new Error(`fixedPairCapacitorCurrentAt: capacitanceF must be > 0, got ${String(capacitanceF)}.`);
  if (esrOhms < 0) throw new Error(`fixedPairCapacitorCurrentAt: esrOhms must be >= 0, got ${String(esrOhms)}.`);
  if (leakageOhms < 0) throw new Error(`fixedPairCapacitorCurrentAt: leakageOhms must be >= 0, got ${String(leakageOhms)}.`);
  const series = esrOhms > 0 ? (dV / esrOhms) * Math.exp(-t / (esrOhms * capacitanceF)) : 0;
  const leak = leakageOhms > 0 ? dV / leakageOhms : 0;
  return series + leak;
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


// ─── W1e: the nodal solve, the model-exact NE555 sidecar, the LED sidecars ───
//
// core/README.md "Kernel semantics: NE555 (W1e)" point 13 and "Kernel
// semantics: LEDs and displays (W1e)": the sidecar is model-exact and never
// computed by core code. Everything below is plain double-precision
// arithmetic on the model's own equations.

/** The universal free-node shunt (core/README.md W0 point 2), siemens: both engines stamp it, so the model-exact answer includes it. */
export const NODE_SHUNT_SIEMENS = 1e-12;
/** A junction's own GMIN (LEDs point 2), siemens. */
export const JUNCTION_GMIN_SIEMENS = 1e-12;
/** The DIS pull-down while the latch is LOW (NE555 point 7), ohms. */
export const NE555_DIS_OHMS = 10;
/** The RST comparator level, absolute volts above pin 1 (NE555 point 3). */
export const NE555_RST_VOLTS = 0.5;

export interface NetworkEdge {
  readonly id: string;
  readonly a: string;
  readonly b: string;
  readonly ohms: number;
}

/** A resistive network: fixed nodes with their volts, free nodes (each carrying the universal shunt to the datum) and the resistors between them. */
export interface ResistiveNetwork {
  readonly fixed: Readonly<Record<string, number>>;
  readonly free: readonly string[];
  readonly edges: readonly NetworkEdge[];
}

/** Gaussian elimination with partial pivoting; throws on a zero pivot. */
function solveDense(matrix: readonly (readonly number[])[], rhs: readonly number[]): number[] {
  const n = rhs.length;
  const a = matrix.map((row, i) => [...row, rhs[i]!]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let row = col + 1; row < n; row++) {
      if (Math.abs(a[row]![col]!) > Math.abs(a[pivot]![col]!)) pivot = row;
    }
    if (!(Math.abs(a[pivot]![col]!) > 0)) throw new Error("solveDense: singular matrix");
    const held = a[col]!;
    a[col] = a[pivot]!;
    a[pivot] = held;
    const pivotRow = a[col]!;
    for (let row = col + 1; row < n; row++) {
      const target = a[row]!;
      const factor = target[col]! / pivotRow[col]!;
      if (factor === 0) continue;
      for (let k = col; k <= n; k++) target[k] = target[k]! - factor * pivotRow[k]!;
    }
  }
  const x = new Array<number>(n).fill(0);
  for (let row = n - 1; row >= 0; row--) {
    let sum = a[row]![n]!;
    for (let k = row + 1; k < n; k++) sum -= a[row]![k]! * x[k]!;
    x[row] = sum / a[row]![row]!;
  }
  return x;
}

/** Every node's volts (fixed ones included). `injection` adds `amps` into `node` (a free one), for a Thevenin resistance. */
export function solveResistiveNetwork(
  network: ResistiveNetwork,
  injection?: { readonly node: string; readonly amps: number },
): Record<string, number> {
  const index = new Map(network.free.map((name, i) => [name, i] as const));
  const n = network.free.length;
  const matrix = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  const rhs = new Array<number>(n).fill(0);
  for (let i = 0; i < n; i++) matrix[i]![i] = NODE_SHUNT_SIEMENS;

  const fixedVolts = (name: string): number => {
    const v = network.fixed[name];
    if (v === undefined) throw new Error(`solveResistiveNetwork: node "${name}" is neither free nor fixed.`);
    return v;
  };
  for (const edge of network.edges) {
    const g = 1 / edge.ohms;
    const ia = index.get(edge.a);
    const ib = index.get(edge.b);
    if (ia !== undefined) {
      matrix[ia]![ia] = matrix[ia]![ia]! + g;
      if (ib !== undefined) matrix[ia]![ib] = matrix[ia]![ib]! - g;
      else rhs[ia] = rhs[ia]! + g * fixedVolts(edge.b);
    }
    if (ib !== undefined) {
      matrix[ib]![ib] = matrix[ib]![ib]! + g;
      if (ia !== undefined) matrix[ib]![ia] = matrix[ib]![ia]! - g;
      else rhs[ib] = rhs[ib]! + g * fixedVolts(edge.a);
    }
  }
  if (injection) {
    const i = index.get(injection.node);
    if (i === undefined) throw new Error(`solveResistiveNetwork: injection node "${injection.node}" is not free.`);
    rhs[i] = rhs[i]! + injection.amps;
  }

  const x = solveDense(matrix, rhs);
  const out: Record<string, number> = { ...network.fixed };
  network.free.forEach((name, i) => {
    out[name] = x[i]!;
  });
  return out;
}

/** The network with one free node held at `volts` (the capacitor's voltage, in a timing network). */
export function withNodeFixed(network: ResistiveNetwork, node: string, volts: number): ResistiveNetwork {
  return { fixed: { ...network.fixed, [node]: volts }, free: network.free.filter((name) => name !== node), edges: network.edges };
}

/** The Thevenin equivalent seen at `node`, shunts included: the open-circuit voltage, and the resistance from a 1 A injection with every source at 0 V. */
export function theveninAt(network: ResistiveNetwork, node: string): { readonly vth: number; readonly rth: number } {
  const vth = solveResistiveNetwork(network)[node]!;
  const zeroed: ResistiveNetwork = {
    fixed: Object.fromEntries(Object.keys(network.fixed).map((name) => [name, 0])),
    free: network.free,
    edges: network.edges,
  };
  const rth = solveResistiveNetwork(zeroed, { node, amps: 1 })[node]!;
  return { vth, rth };
}

/** One side of the capacitor's Thevenin equivalent: the asymptote (absolute volts) and the resistance. */
export interface Ne555Thevenin {
  readonly vInf: number;
  readonly rOhms: number;
}

/** NE555 point 13.3's model: the capacitor sees `charge` while OUT is HIGH (DIS released) and `discharge` while it is LOW (DIS a 10 ohm resistor to pin 1). */
export interface Ne555TimingModel {
  readonly mode: "astable" | "monostable";
  /** V(pin 1): the comparators' reference. */
  readonly vGnd: number;
  /** V(8) - V(1). */
  readonly vS: number;
  readonly capacitanceF: number;
  readonly charge: Ne555Thevenin;
  readonly discharge: Ne555Thevenin;
  /** The capacitor node's voltage at t = 0 (the other terminal's voltage: an uncharged capacitor). Default 0. */
  readonly startVolts?: number;
  /** RST held below 0.5 V until this time: the latch is 0 and DIS active until then, and the first HIGH starts here (point 9). */
  readonly releaseS?: number;
}

/** One HIGH or LOW stretch of the model: the capacitor node follows `v0 -> thevenin.vInf` with `tauS` from `t0` to `t1` (`Infinity` for a monostable's last). */
export interface Ne555Phase {
  readonly high: boolean;
  readonly t0: number;
  readonly t1: number;
  readonly v0: number;
  readonly tauS: number;
  readonly vInf: number;
}

/** A change of the TRIG input a monostable's phases depend on: pulled low (`low`) or released, at time `t` (from a scripted button, `ne555Triggers`). */
export interface Ne555Trigger {
  readonly t: number;
  readonly low: boolean;
}

/**
 * The phases of the model up to (at least) `horizonS`, from `rcCrossingTime`
 * on each side's Thevenin equivalent (point 13.3):
 * `t_high = Rc C ln((Vc - vL) / (Vc - vU))`, `t_low = Rd C ln((vU - Vd) / (vL - Vd))`,
 * the first HIGH from `v0` (`ln(V∞c / (V∞c - vU))` from 0 V), a monostable
 * pulse from `v0` likewise. With `releaseS`, the first phase is a LOW from
 * t = 0 (discharging toward `V∞d`) and the HIGH starts at the release, from
 * the capacitor's voltage then (NE555 point 9; the latch SETs because TRIG is
 * below `vL`, which is checked).
 *
 * `triggers` (a monostable only) are the scripted TRIG changes after power-on:
 * a pull low while the latch is 0 SETs it, and the pulse starts from the
 * capacitor's voltage then (`v0 = V∞d` after a full discharge, point 13.3);
 * a pull low while HIGH changes nothing, except that a TRIG still low when the
 * threshold is crossed holds OUT HIGH (SET beats RESET, point 3) until it is
 * released, when THR is still above `vU` and the latch RESETs at once.
 */
export function ne555Phases(model: Ne555TimingModel, horizonS: number, triggers: readonly Ne555Trigger[] = []): Ne555Phase[] {
  const vL = model.vGnd + model.vS / 3;
  const vU = model.vGnd + (2 * model.vS) / 3;
  const c = model.capacitanceF;
  const tauC = model.charge.rOhms * c;
  const tauD = model.discharge.rOhms * c;
  const phases: Ne555Phase[] = [];
  let t = 0;
  let v = model.startVolts ?? 0;
  if (triggers.length > 0 && model.mode !== "monostable") throw new Error("ne555Phases: scripted triggers are modelled for a monostable only.");

  if (model.releaseS !== undefined && model.releaseS > 0) {
    phases.push({ high: false, t0: 0, t1: model.releaseS, v0: v, tauS: tauD, vInf: model.discharge.vInf });
    v = rcVoltageAt(v, model.discharge.vInf, tauD, model.releaseS);
    t = model.releaseS;
    if (!(v < vL)) {
      throw new Error(`ne555Phases: the capacitor is at ${v} V when RST releases, not below vL=${vL}: the latch would hold, which this model does not cover.`);
    }
  }

  const trigLowAt = (time: number): boolean => triggers.reduce((low, e) => (e.t <= time ? e.low : low), false);
  for (;;) {
    const natural = t + rcCrossingTime(v, vU, model.charge.vInf, tauC);
    let end = natural;
    if (trigLowAt(natural)) {
      const release = triggers.find((e) => !e.low && e.t > natural);
      end = release ? release.t : Number.POSITIVE_INFINITY;
    }
    phases.push({ high: true, t0: t, t1: end, v0: v, tauS: tauC, vInf: model.charge.vInf });
    if (!Number.isFinite(end)) return phases;
    v = rcVoltageAt(v, model.charge.vInf, tauC, end - t);
    t = end;

    if (model.mode === "monostable") {
      const next = triggers.find((e) => e.low && e.t >= t);
      if (!next) {
        phases.push({ high: false, t0: t, t1: Number.POSITIVE_INFINITY, v0: v, tauS: tauD, vInf: model.discharge.vInf });
        return phases;
      }
      phases.push({ high: false, t0: t, t1: next.t, v0: v, tauS: tauD, vInf: model.discharge.vInf });
      v = rcVoltageAt(v, model.discharge.vInf, tauD, next.t - t);
      t = next.t;
      if (t > horizonS) return phases;
      continue;
    }
    const low = rcCrossingTime(vU, vL, model.discharge.vInf, tauD);
    phases.push({ high: false, t0: t, t1: t + low, v0: v, tauS: tauD, vInf: model.discharge.vInf });
    t += low;
    v = vL;
    if (t > horizonS) return phases;
  }
}

/**
 * The phase holding instant `t`, each phase taken as `(t0, t1]`: a sample AT a
 * scripted trigger is the last sample of the phase before it (stimuli.ts
 * applies a phase's `set` after the previous phase's final sample), and t = 0
 * belongs to the first phase.
 */
export function ne555PhaseAt(phases: readonly Ne555Phase[], t: number): Ne555Phase {
  return phases.find((p) => t > p.t0 && t <= p.t1) ?? (t <= 0 ? phases[0]! : phases[phases.length - 1]!);
}

/** The capacitor node's exact voltage at `t`. */
export function ne555CapVoltageAt(phases: readonly Ne555Phase[], t: number): number {
  const p = ne555PhaseAt(phases, t);
  return rcVoltageAt(p.v0, p.vInf, p.tauS, t - p.t0);
}

/** Which components, and which of their pins, the sidecar's extraction may find on a timing node besides the resistors and the capacitor. */
const NE555_TIMING_PINS: Readonly<Record<string, readonly string[]>> = { timing: ["2", "6", "7"], rst: ["4"] };

export interface Ne555SidecarSpec {
  readonly component: string;
  readonly mode: "astable" | "monostable";
  readonly capacitor: string;
  readonly rstCapacitor?: string;
}

/** What the sidecar predicts at one instant, keyed the way `Mismatch.id` names netV and elementI targets. */
export interface Ne555SidecarValues {
  readonly netV: Record<string, number>;
  readonly elementI: Record<string, number>;
}

export interface Ne555Sidecar {
  readonly spec: Ne555SidecarSpec;
  readonly model: Ne555TimingModel;
  readonly phases: readonly Ne555Phase[];
  /** Net ids of every free node the sidecar predicts (the timing network's and the RST network's). */
  readonly netIds: ReadonlySet<string>;
  /** Component ids the sidecar predicts a current for: the networks' resistors and capacitors. */
  readonly elementIds: ReadonlySet<string>;
  /** The networks' resistors with their two net ids, a to b: the supply residual removes their current from a source's. */
  readonly resistors: readonly { readonly id: string; readonly aNet: string; readonly bNet: string }[];
  /** The model's prediction at `t`. */
  readonly at: (t: number) => Ne555SidecarValues;
}

interface ExtractedRcNetwork {
  /** Node names are `n<node number>`. */
  readonly network: ResistiveNetwork;
  readonly capNode: string;
  /** Sign of the capacitor's current into its pin `a` (+1 when `a` is on the free node). */
  readonly capSign: 1 | -1;
  readonly capOtherVolts: number;
  readonly capacitanceF: number;
  readonly edges: readonly NetworkEdge[];
}

function fixedNodeVolts(circuit: SimCircuit, pinNode: ReadonlyMap<string, number>): Map<number, number> {
  const fixed = new Map<number, number>([[0, 0]]);
  const sources = circuit.components.filter((c) => c.kind === "voltage_source");
  for (let changed = true; changed; ) {
    changed = false;
    for (const source of sources) {
      const pos = pinNode.get(corePinKey(source.id, "pos"));
      const neg = pinNode.get(corePinKey(source.id, "neg"));
      if (pos === undefined || neg === undefined) continue;
      const volts = Number(source.params.voltage ?? 5);
      if (fixed.has(neg) && !fixed.has(pos)) {
        fixed.set(pos, fixed.get(neg)! + volts);
        changed = true;
      } else if (fixed.has(pos) && !fixed.has(neg)) {
        fixed.set(neg, fixed.get(pos)! - volts);
        changed = true;
      }
    }
  }
  return fixed;
}

/**
 * The resistive network around one capacitor: every node reachable from the
 * capacitor's free terminal through resistors, stopping at fixed nodes.
 * Anything else sitting on one of those nodes (other than the 555 pins the
 * caller allows) is refused: the sidecar models the timing network, and a
 * foreign element on it would silently change the answer.
 */
function extractRcNetwork(
  circuit: SimCircuit,
  built: ReturnType<typeof buildCoreNodes>,
  fixed: ReadonlyMap<number, number>,
  capacitorId: string,
  allowed555: { readonly component: string; readonly pins: readonly string[] },
): ExtractedRcNetwork {
  const cap = circuit.components.find((c) => c.id === capacitorId && c.kind === "capacitor");
  if (!cap) throw new Error(`ne555 sidecar: capacitor "${capacitorId}" is not in the circuit.`);
  const nodeOf = (componentId: string, pinId: string): number => {
    const node = built.pinNode.get(corePinKey(componentId, pinId));
    if (node === undefined) throw new Error(`ne555 sidecar: ${componentId}.${pinId} is not wired.`);
    return node;
  };
  const capA = nodeOf(cap.id, cap.pins[0]!.id);
  const capB = nodeOf(cap.id, cap.pins[1]!.id);
  const capAFixed = fixed.has(capA);
  const capBFixed = fixed.has(capB);
  if (capAFixed === capBFixed) throw new Error(`ne555 sidecar: capacitor "${capacitorId}" must have exactly one fixed terminal.`);
  const capNode = capAFixed ? capB : capA;
  const capOtherVolts = fixed.get(capAFixed ? capA : capB)!;
  const name = (node: number): string => `n${node}`;

  const resistors = circuit.components.filter((c) => c.kind === "resistor");
  const free = new Set<number>([capNode]);
  const edges: NetworkEdge[] = [];
  const seen = new Set<string>();
  const queue = [capNode];
  while (queue.length > 0) {
    const node = queue.shift()!;
    for (const r of resistors) {
      if (seen.has(r.id)) continue;
      const a = nodeOf(r.id, "a");
      const b = nodeOf(r.id, "b");
      if (a !== node && b !== node) continue;
      seen.add(r.id);
      const ohms = Number(r.params.resistance);
      if (!(ohms > 0)) throw new Error(`ne555 sidecar: resistor "${r.id}" has no positive resistance param.`);
      edges.push({ id: r.id, a: name(a), b: name(b), ohms });
      for (const end of [a, b]) {
        if (!fixed.has(end) && !free.has(end)) {
          free.add(end);
          queue.push(end);
        }
      }
    }
  }

  const edgeIds = new Set(edges.map((e) => e.id));
  for (const node of free) {
    const net = built.nets.find((n) => built.nodeOfNet.get(n.id) === node);
    for (const [componentId, pinId] of net?.pins ?? []) {
      const ok =
        edgeIds.has(componentId) ||
        componentId === capacitorId ||
        (componentId === allowed555.component && allowed555.pins.includes(pinId));
      if (!ok) {
        throw new Error(`ne555 sidecar: net ${net!.id} carries ${componentId}.${pinId}, which the timing model does not cover.`);
      }
    }
  }

  const fixedRecord: Record<string, number> = {};
  for (const [node, volts] of fixed) fixedRecord[name(node)] = volts;
  const capacitanceF = Number(cap.params.capacitance);
  if (!(capacitanceF > 0)) throw new Error(`ne555 sidecar: capacitor "${capacitorId}" has no positive capacitance param.`);
  if (Number(cap.params.esr ?? 0) !== 0 || cap.params.leakage !== undefined || cap.params.catalogUid !== undefined) {
    throw new Error(`ne555 sidecar: capacitor "${capacitorId}" carries ESR, leakage or a catalog part; the model assumes an ideal capacitor (ESR is 0 without a catalogUid).`);
  }
  return {
    network: { fixed: fixedRecord, free: [...free].map(name), edges },
    capNode: name(capNode),
    capSign: capAFixed ? -1 : 1,
    capOtherVolts,
    capacitanceF,
    edges,
  };
}

/**
 * The TRIG changes a scripted stimulus makes (stimuli.ts): every `set` on a
 * `push_button` or `switch` whose one pin is on the 555's TRIG net and the
 * other on pin 1's net pulls TRIG low while `closed` is truthy. A `set` on
 * anything the sidecar models (the timing network's parts, an ideal source) is
 * refused, since its answer would no longer be the model's; every other `set`
 * does not touch the 555 and is ignored. Phase 0's `set` is part of the load
 * (stimuli.ts folds it into the netlist), so a TRIG pulled low there is
 * refused too: the one-pulse model starts with TRIG high.
 */
function ne555Triggers(
  circuit: SimCircuit,
  spec: Ne555SidecarSpec | Ne555FixtureSpec,
  built: ReturnType<typeof buildCoreNodes>,
  phases: readonly StimulusPhase[],
): Ne555Trigger[] {
  const monostable = spec.mode === "monostable";
  const node = (componentId: string, pinId: string): number | undefined => built.pinNode.get(corePinKey(componentId, pinId));
  const trig = node(spec.component, "2");
  const gnd = node(spec.component, "1");
  const buttons = circuit.components.filter((c) => {
    if (c.kind !== "push_button" && c.kind !== "switch") return false;
    const a = node(c.id, c.pins[0]!.id);
    const b = node(c.id, c.pins[1]!.id);
    return (a === trig && b === gnd) || (b === trig && a === gnd);
  });
  const closed = new Map<string, boolean>(buttons.map((b) => [b.id, Boolean(b.params.closed)]));
  const triggers: Ne555Trigger[] = [];
  let low = false;
  let start = 0;
  phases.forEach((phase, index) => {
    const phaseStart = start;
    start += phase.durationS;
    for (const change of phase.set ?? []) {
      const target = circuit.components.find((c) => c.id === change.id);
      if (!target) continue;
      if (closed.has(change.id)) {
        if (change.params.closed !== undefined) closed.set(change.id, Boolean(change.params.closed));
      } else if (target.kind === "voltage_source" || target.kind === "resistor" || target.kind === "capacitor" || change.id === spec.component) {
        throw new Error(`ne555 sidecar: the stimulus changes "${change.id}", which the timing model does not follow.`);
      }
    }
    const now = [...closed.values()].some(Boolean);
    if (!monostable) return; // an astable ignores TRIG (a button on its timing node was refused at extraction)
    if (index === 0) {
      if (now) throw new Error("ne555 sidecar: TRIG is pulled low from load; the one-pulse model starts with TRIG high.");
    } else if (now !== low) {
      triggers.push({ t: phaseStart, low: now });
    }
    low = now;
  });
  return triggers;
}

/**
 * NE555 point 13.3, from the circuit itself: the capacitor's Thevenin
 * equivalents with DIS released and with DIS a 10 ohm resistor from pin 7 to
 * pin 1, both including the 1e-12 S shunt on every free node (solved nodally,
 * `theveninAt`), the phases they imply (`ne555Phases`) and, for the analog
 * comparison (point 13.4), the exact voltage of every node of the timing
 * network and the current of every element on it at any instant.
 *
 * `options.dischargeDropsResistor` exists for the detector self-test only: it
 * removes the named resistor from the DISCHARGE network (a seeded wrong
 * sidecar; dropping RA gives the SAP-1 clock 2.086373 ms).
 */
export function extractNe555Sidecar(
  circuit: SimCircuit,
  spec: Ne555SidecarSpec | Ne555FixtureSpec,
  horizonS: number,
  options: {
    /** Detector self-test only: the discharge network without this resistor. */
    readonly dischargeDropsResistor?: string;
    /** A scripted stimulus (stimuli.ts): its button presses on TRIG are the monostable's triggers. */
    readonly stimuli?: readonly StimulusPhase[];
  } = {},
): Ne555Sidecar {
  const fault = options.dischargeDropsResistor === undefined ? undefined : { dischargeDropsResistor: options.dischargeDropsResistor };
  const built = buildCoreNodes(circuit);
  const fixed = fixedNodeVolts(circuit, built.pinNode);
  const nodeOf = (componentId: string, pinId: string): number => {
    const node = built.pinNode.get(corePinKey(componentId, pinId));
    if (node === undefined) throw new Error(`ne555 sidecar: ${componentId}.${pinId} is not wired.`);
    return node;
  };
  const name = (node: number): string => `n${node}`;
  const netIdOf = (nodeName: string): string => built.netIdOfNode[Number(nodeName.slice(1))] ?? nodeName;

  const vcc = fixed.get(nodeOf(spec.component, "8"));
  const gnd = fixed.get(nodeOf(spec.component, "1"));
  if (vcc === undefined || gnd === undefined) throw new Error("ne555 sidecar: pins 8 and 1 must both sit on fixed nodes (an ideal supply).");
  const vS = vcc - gnd;

  if (spec.mode === "monostable") {
    // TRIG is left to whatever the fixture wires; the model (one power-on pulse, TRIG high and
    // THR low at load) holds only while nothing pulls it low: resistors, an unpressed button
    // and ideal sources may sit on it, anything else is not covered.
    const trigNet = built.nets.find((n) => built.nodeOfNet.get(n.id) === nodeOf(spec.component, "2"));
    for (const [componentId, pinId] of trigNet?.pins ?? []) {
      const comp = circuit.components.find((c) => c.id === componentId);
      const fine =
        comp !== undefined &&
        ((comp.kind === "ne555" && componentId === spec.component && pinId === "2") ||
          comp.kind === "resistor" ||
          comp.kind === "voltage_source" ||
          ((comp.kind === "push_button" || comp.kind === "switch") && !comp.params.closed));
      if (!fine) throw new Error(`ne555 sidecar: TRIG net carries ${componentId}.${pinId}, which the one-pulse model does not cover.`);
    }
  }

  const timing = extractRcNetwork(circuit, built, fixed, spec.capacitor, { component: spec.component, pins: NE555_TIMING_PINS.timing! });
  const disName = name(nodeOf(spec.component, "7"));
  if (!timing.network.free.includes(disName)) {
    throw new Error("ne555 sidecar: pin 7 (DIS) must sit on the timing network.");
  }
  const gndName = name(nodeOf(spec.component, "1"));
  const chargeNet = timing.network;
  const dischargeEdges: NetworkEdge[] = [
    ...timing.edges.filter((e) => e.id !== fault?.dischargeDropsResistor),
    { id: "dis", a: disName, b: gndName, ohms: NE555_DIS_OHMS },
  ];
  const dischargeNet: ResistiveNetwork = { ...timing.network, edges: dischargeEdges };
  const charge = theveninAt(chargeNet, timing.capNode);
  const discharge = theveninAt(dischargeNet, timing.capNode);

  // The model is the free-running one: RST must sit on the supply (never resets) or on
  // its own RC (a power-on reset). Driven by anything else, the fixture is not covered.
  const rstNode = nodeOf(spec.component, "4");
  if (!spec.rstCapacitor && rstNode !== nodeOf(spec.component, "8")) {
    throw new Error("ne555 sidecar: pin 4 (RST) must sit on the supply, or on an RC named by rstCapacitor.");
  }

  let releaseS: number | undefined;
  let rst: ExtractedRcNetwork | undefined;
  let rstThevenin: { vth: number; rth: number } | undefined;
  if (spec.rstCapacitor) {
    rst = extractRcNetwork(circuit, built, fixed, spec.rstCapacitor, { component: spec.component, pins: NE555_TIMING_PINS.rst! });
    if (name(nodeOf(spec.component, "4")) !== rst.capNode) throw new Error("ne555 sidecar: pin 4 (RST) must sit on the RST capacitor's node.");
    rstThevenin = theveninAt(rst.network, rst.capNode);
    const target = gnd + NE555_RST_VOLTS;
    releaseS = rcCrossingTime(rst.capOtherVolts, target, rstThevenin.vth, rstThevenin.rth * rst.capacitanceF);
  }

  const model: Ne555TimingModel = {
    mode: spec.mode,
    vGnd: gnd,
    vS,
    capacitanceF: timing.capacitanceF,
    charge: { vInf: charge.vth, rOhms: charge.rth },
    discharge: { vInf: discharge.vth, rOhms: discharge.rth },
    startVolts: timing.capOtherVolts,
    releaseS,
  };
  const triggers = options.stimuli ? ne555Triggers(circuit, spec, built, options.stimuli) : [];
  const phases = ne555Phases(model, horizonS, triggers);

  const netIds = new Set<string>();
  const elementIds = new Set<string>();
  const resistors: { id: string; aNet: string; bNet: string }[] = [];
  const collect = (extracted: ExtractedRcNetwork, capacitorId: string): void => {
    for (const nodeName of extracted.network.free) netIds.add(netIdOf(nodeName));
    for (const edge of extracted.edges) {
      elementIds.add(edge.id);
      resistors.push({ id: edge.id, aNet: netIdOf(edge.a), bNet: netIdOf(edge.b) });
    }
    elementIds.add(capacitorId);
  };
  collect(timing, spec.capacitor);
  if (rst && spec.rstCapacitor) collect(rst, spec.rstCapacitor);

  const at = (t: number): Ne555SidecarValues => {
    const netV: Record<string, number> = {};
    const elementI: Record<string, number> = {};
    const fill = (extracted: ExtractedRcNetwork, capacitorId: string, net: ResistiveNetwork, x: number, capAmps: number): void => {
      const solved = solveResistiveNetwork(withNodeFixed(net, extracted.capNode, x));
      for (const nodeName of extracted.network.free) netV[netIdOf(nodeName)] = solved[nodeName]!;
      for (const edge of extracted.edges) elementI[edge.id] = (solved[edge.a]! - solved[edge.b]!) / edge.ohms;
      elementI[capacitorId] = extracted.capSign * capAmps;
    };

    const phase = ne555PhaseAt(phases, t);
    const x = rcVoltageAt(phase.v0, phase.vInf, phase.tauS, t - phase.t0);
    fill(timing, spec.capacitor, phase.high ? chargeNet : dischargeNet, x, (phase.vInf - x) / (phase.tauS / timing.capacitanceF));

    if (rst && rstThevenin && spec.rstCapacitor) {
      const rstTau = rstThevenin.rth * rst.capacitanceF;
      const xr = rcVoltageAt(rst.capOtherVolts, rstThevenin.vth, rstTau, t);
      fill(rst, spec.rstCapacitor, rst.network, xr, (rstThevenin.vth - xr) / rstThevenin.rth);
    }
    return { netV, elementI };
  };

  return { spec, model, phases, netIds, elementIds, resistors, at };
}

/**
 * Every 555 in the circuit whose wiring is a recognisable astable (TRIG and THR
 * on one node) or monostable (THR and DIS on one node, TRIG elsewhere) with a
 * timing capacitor from that node to a fixed node, with `capacitor` and, when
 * RST sits on a capacitor's node, `rstCapacitor`. This only reads the topology:
 * whether `extractNe555Sidecar` can then model it (a foreign element on the
 * timing network, an RST that is neither on the supply nor on an RC, a
 * non-ideal supply) is that function's own refusal, and the caller falls back
 * to the unmodelled path when it throws. A bistable or a Schmitt trigger is not
 * listed.
 */
export function detectNe555Specs(circuit: SimCircuit): Ne555SidecarSpec[] {
  const built = buildCoreNodes(circuit);
  const fixed = fixedNodeVolts(circuit, built.pinNode);
  const specs: Ne555SidecarSpec[] = [];
  const nodeOfPin = (componentId: string, pinId: string): number | undefined => built.pinNode.get(corePinKey(componentId, pinId));
  const capacitorOnNode = (node: number): string | undefined =>
    circuit.components.find((c) => {
      if (c.kind !== "capacitor") return false;
      const a = nodeOfPin(c.id, c.pins[0]!.id);
      const b = nodeOfPin(c.id, c.pins[1]!.id);
      if (a === undefined || b === undefined) return false;
      return (a === node && fixed.has(b) && !fixed.has(a)) || (b === node && fixed.has(a) && !fixed.has(b));
    })?.id;

  for (const chip of circuit.components) {
    if (chip.kind !== "ne555") continue;
    const n2 = nodeOfPin(chip.id, "2");
    const n4 = nodeOfPin(chip.id, "4");
    const n6 = nodeOfPin(chip.id, "6");
    const n7 = nodeOfPin(chip.id, "7");
    if (n2 === undefined || n6 === undefined || n7 === undefined || fixed.has(n6)) continue;
    const capacitor = capacitorOnNode(n6);
    if (!capacitor) continue;
    const mode = n2 === n6 ? "astable" : n7 === n6 ? "monostable" : undefined;
    if (!mode) continue;
    const rstCapacitor = n4 !== undefined && !fixed.has(n4) ? capacitorOnNode(n4) : undefined;
    specs.push(rstCapacitor ? { component: chip.id, mode, capacitor, rstCapacitor } : { component: chip.id, mode, capacitor });
  }
  return specs;
}

// ─── LED junction sidecars (LEDs point 2 and 8) ─────────────────────────────

const K_B_OVER_Q = 8.617333262145e-5;

export interface LedLawParams {
  readonly vf25: number;
  readonly iRated: number;
  readonly n: number;
  readonly tempC: number;
}

export interface LedLaw {
  readonly vtn: number;
  readonly is: number;
  readonly vSat: number;
}

/** LEDs point 2's `(vtn, Is, vSat)` at temperature `tempC`, without the `Is` override (the fixtures never set it). */
export function ledLaw(params: LedLawParams): LedLaw {
  const vt = K_B_OVER_Q * Math.max(1, params.tempC + 273.15);
  const vf = Math.max(0.05, params.vf25 + -0.002 * (params.tempC - 25));
  const vtn = params.n * vt;
  const is = params.iRated * Math.exp(-vf / vtn);
  const vSat = Math.min(Math.max(40 * vtn, vf + 5 * vtn), 80 * vtn);
  return { vtn, is, vSat };
}

/** `I(v)`: the exponential below `vSat`, its tangent above (GMIN excluded, as `elementI` excludes it). */
export function ledCurrent(law: LedLaw, v: number): number {
  const c = Math.min(v, law.vSat);
  const e = Math.exp(c / law.vtn);
  return law.is * (e - 1) + ((law.is * e) / law.vtn) * Math.max(0, v - law.vSat);
}

/** The root of a decreasing `f` on `[lo, hi]`, to the last bit. */
function bisectDecreasing(f: (x: number) => number, lo: number, hi: number): number {
  if (!(f(lo) > 0) || !(f(hi) < 0)) throw new Error(`bisectDecreasing: [${lo}, ${hi}] does not bracket the root.`);
  for (let i = 0; i < 400; i++) {
    const mid = 0.5 * (lo + hi);
    if (mid === lo || mid === hi) break;
    if (f(mid) > 0) lo = mid;
    else hi = mid;
  }
  return 0.5 * (lo + hi);
}

const LED_GLOBAL_G = JUNCTION_GMIN_SIEMENS + NODE_SHUNT_SIEMENS;

export interface LedSeriesResult {
  readonly nodeV: number;
  readonly ledAmps: number;
  readonly resistorAmps: number;
}

/**
 * One junction fed through one resistor from an ideal source (`anode-node`:
 * `vs - R - x - LED - datum`; `cathode-node`: `vs - LED - x - R - datum`),
 * solved by bisection on the node's KCL with the junction's GMIN and the
 * node's shunt. `resistorAmps` is the current from the upstream end of the
 * resistor to the downstream end.
 */
export function ledSeriesNode(law: LedLaw, vs: number, rOhms: number, orientation: "anode-node" | "cathode-node"): LedSeriesResult {
  if (orientation === "anode-node") {
    const x = bisectDecreasing((v) => (vs - v) / rOhms - ledCurrent(law, v) - LED_GLOBAL_G * v, -1, vs + 1);
    return { nodeV: x, ledAmps: ledCurrent(law, x), resistorAmps: (vs - x) / rOhms };
  }
  const x = bisectDecreasing(
    (v) => ledCurrent(law, vs - v) + JUNCTION_GMIN_SIEMENS * (vs - v) - v / rOhms - NODE_SHUNT_SIEMENS * v,
    -1,
    vs + 1,
  );
  return { nodeV: x, ledAmps: ledCurrent(law, vs - x), resistorAmps: x / rOhms };
}

export interface LedSharedCommonResult {
  readonly commonV: number;
  readonly nodeV: readonly number[];
  readonly ledAmps: readonly number[];
  readonly commonAmps: number;
}

/**
 * `seg7_cc` segments (one junction each, anode fed from `vs` through its own
 * resistor) sharing a common cathode node that reaches the datum through
 * `commonOhms`: the coupled solve, nested bisection (the common node's KCL
 * outside, each anode's inside).
 */
export function ledSharedCommon(laws: readonly LedLaw[], vs: number, rOhms: readonly number[], commonOhms: number): LedSharedCommonResult {
  const anodeFor = (i: number, u: number): number =>
    bisectDecreasing(
      (x) => (vs - x) / rOhms[i]! - ledCurrent(laws[i]!, x - u) - JUNCTION_GMIN_SIEMENS * (x - u) - NODE_SHUNT_SIEMENS * x,
      u - 1,
      vs + 1,
    );
  const commonResidual = (u: number): number => {
    let into = 0;
    laws.forEach((law, i) => {
      const x = anodeFor(i, u);
      into += ledCurrent(law, x - u) + JUNCTION_GMIN_SIEMENS * (x - u);
    });
    return into - u / commonOhms - NODE_SHUNT_SIEMENS * u;
  };
  const u = bisectDecreasing(commonResidual, 0, vs);
  const nodeV = laws.map((_, i) => anodeFor(i, u));
  const ledAmps = laws.map((law, i) => ledCurrent(law, nodeV[i]! - u));
  return { commonV: u, nodeV, ledAmps, commonAmps: u / commonOhms };
}

/** What `evaluateLedStatic` predicts: netV by net id, elementI by component id, channel currents by component id (`null`: the entry is absent, a burnt `rgb_led` or `bicolor_led`). */
export interface LedStaticValues {
  readonly netV: Record<string, number>;
  readonly elementI: Record<string, number>;
  readonly channels: Record<string, readonly number[] | null>;
}

const LED_CHANNELS_OF_KIND: Readonly<Record<string, number>> = { led: 0, bicolor_led: 2, rgb_led: 3, seg7_cc: 8, seg7_ca: 8 };

function paramNumber(params: Readonly<Record<string, unknown>>, key: string, fallback: number): number {
  const v = params[key];
  return v === undefined ? fallback : Number(v);
}

/**
 * LEDs points 2 and 13, from the model's own equations: every junction cell's
 * node voltage and currents (`ledSeriesNode`, or `ledSharedCommon` for a
 * common group), then each device's aggregate `elementI` and channel array
 * (`led`: the signed current; `bicolor_led`: `max(I1, I2)` unclamped;
 * `rgb_led`: `max(0, Ir, Ig, Ib)`; `seg7`: the sum of `max(0, I_seg)`; a burnt
 * device 0 with its channels absent for `rgb_led` and `bicolor_led`). Every
 * channel value is `max(0, I_j)`, the bicolor's included (LEDs point 13):
 * only its aggregate is unclamped.
 * Params come from the circuit (`vf`, `iRated`, `n`, resistances, source
 * volts) and the temperature from `circuit.environment.temperatureC`
 * (default 25); a fixture with a `catalogUid` is refused, since a catalog part
 * changes the resolved params (and a `led` one is declined in W1e).
 */
export function evaluateLedStatic(circuit: SimCircuit, model: AnalyticLedStaticModel): LedStaticValues {
  const built = buildCoreNodes(circuit);
  const netIdAt = (componentId: string, pinId: string): string => {
    const node = built.pinNode.get(corePinKey(componentId, pinId));
    const id = node === undefined ? undefined : built.netIdOfNode[node];
    if (!id) throw new Error(`led sidecar: ${componentId}.${pinId} is not on a named net.`);
    return id;
  };
  const component = (id: string) => {
    const c = circuit.components.find((x) => x.id === id);
    if (!c) throw new Error(`led sidecar: component "${id}" is not in the circuit.`);
    if (c.params.catalogUid !== undefined) throw new Error(`led sidecar: "${id}" carries a catalogUid, so its params are not the ones the sidecar reads.`);
    return c;
  };
  const tempC = Number((circuit as { environment?: { temperatureC?: number } }).environment?.temperatureC ?? 25);

  const netV: Record<string, number> = {};
  const elementI: Record<string, number> = {};
  const currents = new Map<string, number[]>();
  const lawOf = (cell: LedJunctionCell): LedLaw => {
    const device = component(cell.device);
    return ledLaw({
      vf25: paramNumber(device.params, cell.vfParam, cell.vfDefault),
      iRated: paramNumber(device.params, "iRated", 0.02),
      n: device.kind === "led" ? paramNumber(device.params, "n", 2) : 2,
      tempC,
    });
  };
  const record = (cell: LedJunctionCell, ledAmps: number): void => {
    const device = component(cell.device);
    const slots = currents.get(cell.device) ?? new Array<number>(LED_CHANNELS_OF_KIND[device.kind] ?? 0).fill(0);
    if (cell.channel !== undefined) slots[cell.channel] = ledAmps;
    else slots[0] = ledAmps; // a plain `led`: its single current is kept at index 0 (no channel slots exist)
    currents.set(cell.device, slots);
  };

  const grouped = new Map<string, LedJunctionCell[]>();
  for (const cell of model.cells) {
    if (cell.commonGroup) {
      grouped.set(cell.commonGroup, [...(grouped.get(cell.commonGroup) ?? []), cell]);
      continue;
    }
    const vs = Number(component(cell.source).params.voltage ?? 5);
    const rOhms = Number(component(cell.resistor).params.resistance);
    const solved = ledSeriesNode(lawOf(cell), vs, rOhms, cell.orientation);
    netV[netIdAt(cell.device, cell.pin)] = solved.nodeV;
    elementI[cell.resistor] = solved.resistorAmps;
    record(cell, solved.ledAmps);
  }
  for (const group of model.commonGroups ?? []) {
    const cells = grouped.get(group.id) ?? [];
    if (cells.length === 0) throw new Error(`led sidecar: common group "${group.id}" has no cells.`);
    const vs = Number(component(cells[0]!.source).params.voltage ?? 5);
    const solved = ledSharedCommon(
      cells.map(lawOf),
      vs,
      cells.map((cell) => Number(component(cell.resistor).params.resistance)),
      Number(component(group.resistor).params.resistance),
    );
    netV[netIdAt(group.device, group.pin)] = solved.commonV;
    elementI[group.resistor] = solved.commonAmps;
    cells.forEach((cell, i) => {
      netV[netIdAt(cell.device, cell.pin)] = solved.nodeV[i]!;
      elementI[cell.resistor] = (vs - solved.nodeV[i]!) / Number(component(cell.resistor).params.resistance);
      record(cell, solved.ledAmps[i]!);
    });
  }

  const channels: Record<string, readonly number[] | null> = {};
  for (const { device: id } of model.devices) {
    const device = component(id);
    const count = LED_CHANNELS_OF_KIND[device.kind];
    if (count === undefined) throw new Error(`led sidecar: "${id}" is not an LED-family component.`);
    // JS truthiness, as the compiler reads it: the string "0" is burnt.
    const burnt = device.kind !== "seg7_cc" && device.kind !== "seg7_ca" && Boolean(device.params.burnt);
    const slots = currents.get(id) ?? new Array<number>(Math.max(count, 1)).fill(0);
    if (burnt) {
      elementI[id] = 0;
      channels[id] = device.kind === "led" ? [] : null;
      continue;
    }
    switch (device.kind) {
      case "led":
        elementI[id] = slots[0]!;
        channels[id] = [];
        break;
      case "bicolor_led":
        elementI[id] = Math.max(slots[0]!, slots[1]!);
        channels[id] = slots.map((amps) => Math.max(0, amps));
        break;
      case "rgb_led":
        elementI[id] = Math.max(0, ...slots);
        channels[id] = slots.map((amps) => Math.max(0, amps));
        break;
      default:
        elementI[id] = slots.reduce((sum, amps) => sum + Math.max(0, amps), 0);
        channels[id] = slots.map((amps) => Math.max(0, amps));
    }
  }
  return { netV, elementI, channels };
}

/**
 * LEDs point 8's ODE, from an uncharged capacitor:
 * `C dv/dt = (Vs - v)/R - I(v) - (GMIN + shunt) v`, classic fourth-order
 * Runge-Kutta at a step far below the fastest time constant (the junction's
 * dynamic resistance times C), so the sidecar's own error is negligible
 * beside W0's tolerance. Returns the node voltage at each instant in `timesS`
 * (ascending).
 */
export function ledRcVoltages(circuit: SimCircuit, model: AnalyticLedRcModel, timesS: readonly number[]): number[] {
  const component = (id: string) => {
    const c = circuit.components.find((x) => x.id === id);
    if (!c) throw new Error(`led-rc sidecar: component "${id}" is not in the circuit.`);
    return c;
  };
  const vs = Number(component(model.source).params.voltage ?? 5);
  const rOhms = Number(component(model.resistor).params.resistance);
  const cap = component(model.capacitor);
  const capacitanceF = Number(cap.params.capacitance);
  if (Number(cap.params.esr ?? 0) !== 0 || cap.params.leakage !== undefined) throw new Error("led-rc sidecar: the capacitor must be ideal.");
  const device = component(model.device);
  const tempC = Number((circuit as { environment?: { temperatureC?: number } }).environment?.temperatureC ?? 25);
  const law = ledLaw({
    vf25: paramNumber(device.params, "vf", 1.8),
    iRated: paramNumber(device.params, "iRated", 0.02),
    n: paramNumber(device.params, "n", 2),
    tempC,
  });
  const dvdt = (v: number): number => ((vs - v) / rOhms - ledCurrent(law, v) - LED_GLOBAL_G * v) / capacitanceF;
  const h = 1e-6;
  let t = 0;
  let v = 0;
  const out: number[] = [];
  for (const target of timesS) {
    while (t < target) {
      const step = Math.min(h, target - t);
      const k1 = dvdt(v);
      const k2 = dvdt(v + 0.5 * step * k1);
      const k3 = dvdt(v + 0.5 * step * k2);
      const k4 = dvdt(v + step * k3);
      v += (step / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
      t += step;
    }
    out.push(v);
  }
  return out;
}
