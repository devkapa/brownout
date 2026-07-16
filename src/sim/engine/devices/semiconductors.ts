/**
 * SEMICONDUCTORS device cohort (Wave A4 phase 2): diode, led,
 * schottky_diode, zener_diode, tvs_diode, bicolor_led, rgb_led, bjt_npn,
 * bjt_pnp, nmos, pmos, tl431.
 *
 * Every handler body below is the engine's old switch-case / if-block body
 * MOVED VERBATIM, with identifier access adapted through DeviceContext
 * (this._pinNode -> ctx.pinNode, this._vAt -> ctx.vAt, the _stampAll /
 * _updateElementI pass-level `vt` local -> ctx.junctionVt() — the same
 * pure thermalVoltage(ambient) computation, so the float is identical —
 * this.state.mosfetGates -> ctx.state.mosfetGates, out[comp.id] = v ->
 * ctx.setElementCurrent(comp.id, v), loop break/continue -> handler
 * `return`). Do not simplify, reorder mna.add calls, or rewrite algebra
 * here: float accumulation order in the matrix is semantics, and the
 * bitwise oracle suite pins these exact trajectories.
 *
 * Model grouping vs the old switch labels: bjt_npn/bjt_pnp and nmos/pmos
 * keep their shared case bodies as single models. The engine's shared
 * schottky_diode/led/diode case body is kept as ONE copy in shared helpers
 * but split across TWO models (diodeModel and ledModel) because hook
 * PRESENCE drives compiled bucket membership: only "led" was ever in
 * PASSIVE_FAILURE_UPDATE_KINDS, so an updateFailures hook on a model
 * covering plain diodes would pull them into the failure pass — a bucket
 * (and damage-behavior) change the old kind-gated block never made.
 *
 * BJT pnjlim interplay: the per-load cache (_rebuildJunctionCache), the
 * per-solve _vPrevBJT reset, and the _junctionLimitedThisIteration
 * consumption inside solveNonlinear (Wave A3 acceptance guard) all remain
 * engine-owned; handlers reach the cache, the previous-iterate map, and
 * the limited flag through their context aliases only.
 *
 * Other engine-owned machinery deliberately NOT moved: mosfetGates
 * load()-time carry-forward and snapshot/rollback, the pre-commit gate
 * snapshot for the element-current pass (reached via
 * ctx.previousMosfetGate), getElementChannelI (public telemetry shared
 * with the unmigrated seg7 kinds, reached via ctx.elementChannelCurrents),
 * and the _thermalDissipationW LED branch (profile-keyed thermal pass, not
 * a per-kind switch).
 *
 * Import layering (registry header, decision 7): pure helper FUNCTIONS and
 * exported consts from sim-engine.ts are safe to import despite the module
 * cycle because handlers dereference them only at solve time. Never read
 * sim-engine bindings at module evaluation time from this file.
 */

import type {
  AcDeviceContext,
  DeviceComponent,
  DeviceContext,
  DeviceModel,
} from "../device-registry.js";
import { stampAcAdmittance, type AcStampSurface } from "../ac-system.js";
import {
  bjtCurrents,
  bjtLinearization,
  breakdownDiodeCurrent,
  mosDrainCurrent,
  shockleyDiodeCurrent,
  shockleyIsFromVf,
  stampBJT,
  stampBreakdownDiode,
  stampDiodeShockley,
  stampMOSFET,
  stampResistor,
} from "../elements.js";
import { pnjlim } from "../newton.js";
import {
  IRATED_LED,
  junctionForwardVoltage,
  LED_DAMAGE_THRESHOLD,
  N_DIODE,
  N_LED,
  saturationCurrentAtTemperature,
  thermalVoltage,
  vCritFor,
} from "../sim-engine.js";

// GMIN floor shared by every junction Newton stamp in this cohort (see
// stampDiodeShockley/stampBreakdownDiode/stampMOSFET/tl431). The AC system is
// the Newton Jacobian at the operating point, so the small-signal stamps
// include the identical floor — it is part of the linearized matrix, and it
// keeps a cut-off junction from leaving its nodes floating in AC exactly as
// it does in transient. Exported for the other cohorts' junction acStamps
// (seg7 segment LEDs, ULN/H-bridge clamp diodes) for the same reason.
export const AC_GMIN = 1e-12;

/**
 * Small-signal conductance of the guarded Shockley law at a bias voltage —
 * d(i_d)/d(v) of the SAME piecewise model stampDiodeShockley stamps: the
 * exponential slope below the vSat guard, continued as the constant tangent
 * slope beyond it. This is exactly the `gd` the Newton stamp computes, so
 * the AC matrix reproduces the transient Jacobian entry bit-for-bit.
 * Exported alongside AC_GMIN for the other cohorts' junction acStamps.
 */
export function shockleyConductanceAtOp(
  voltage: number,
  Is: number,
  n: number,
  Vt: number,
  vSatOverride?: number,
): number {
  const VtN = n * Vt;
  const vSat = vSatOverride ?? 40 * VtN;
  const v = voltage > vSat ? vSat : voltage;
  return (Is * Math.exp(v / VtN)) / VtN;
}

/**
 * Small-signal conductance of the breakdown-diode law at a bias voltage —
 * gFwd + gRev of the piecewise model stampBreakdownDiode stamps, including
 * both exponential guards' tangent extensions (a constant slope contributes
 * its own value past each guard). Mirrors that stamp's `g` minus the GMIN
 * floor, which callers add explicitly.
 */
function breakdownConductanceAtOp(
  v: number,
  Is_f: number,
  izKnee: number,
  nF: number,
  nZ: number,
  Vt: number,
  vz: number,
  vSatFOverride?: number,
): number {
  const VtNF = nF * Vt;
  const VtNZ = nZ * Vt;
  const vSatF = vSatFOverride ?? 40 * VtNF;
  const vSatZ = 5 * VtNZ;
  const vF = v > vSatF ? vSatF : v;
  const revArg = -(v + vz);
  const revArgClamped = revArg > vSatZ ? vSatZ : revArg;
  const gFwd = (Is_f * Math.exp(vF / VtNF)) / VtNF;
  const gRev = (izKnee * Math.exp(revArgClamped / VtNZ)) / VtNZ;
  return gFwd + gRev;
}

export const zenerDiodeModel: DeviceModel = {
  kinds: ["zener_diode"],
  stamp: (ctx, comp, xGuess, _h) => {
    const pins = comp.pins;
    const vt = ctx.junctionVt();
    if (pins.length < 2) return;
    const ai = ctx.pinNode(comp.id, pins[0].id);
    const ci = ctx.pinNode(comp.id, pins[1].id);
    if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) {
      return;
    }
    const vd = ctx.vAt(xGuess, ai) - ctx.vAt(xGuess, ci);
    const specsZ = ctx.electricalSpecs(comp);
    const vfZ = ctx.junctionVf(Number(comp.params.vf ?? specsZ?.vf ?? 0.7));
    const vzZ = Number(comp.params.vz ?? 5.1);
    const nFZ = Number(comp.params.n ?? N_DIODE);
    const iRatedZ = Number(comp.params.iRated ?? specsZ?.if_max ?? 0.1);
    const Is_fZ = shockleyIsFromVf(vfZ, iRatedZ, nFZ, vt);
    // iz_knee: the catalog stores 5 mA; fall back to 5 mA if absent on old saves.
    // The overdrive form anchors the knee directly: the prefactor IS izKnee
    // (see the stampBreakdownDiode comment — nothing to back-solve).
    const izKneeZ = Number(comp.params.iz_knee ?? 0.005);
    const nZZ = 1.0;
    stampBreakdownDiode(ctx.mna, ai, ci, vd, Is_fZ, izKneeZ, nFZ, nZZ, vt, vzZ);
  },
  updateCurrent: (ctx, comp, x) => {
    // Engine _updateElementI preamble, copied per-handler (same pure reads).
    const pins = comp.pins;
    const a = ctx.pinNode(comp.id, pins[0].id);
    const b = ctx.pinNode(comp.id, pins[1].id);
    const va = ctx.vAt(x, a);
    const vb = ctx.vAt(x, b);
    const vd = va - vb;
    const vt = ctx.junctionVt();
    if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) { ctx.setElementCurrent(comp.id, 0); return; }
    // Reconstruct breakdown current from the converged voltage using the shared helper.
    const specsZI = ctx.electricalSpecs(comp);
    const vfZI = ctx.junctionVf(Number(comp.params.vf ?? specsZI?.vf ?? 0.7));
    const vzZI = Number(comp.params.vz ?? 5.1);
    const nFZI = Number(comp.params.n ?? N_DIODE);
    const iRatedZI = Number(comp.params.iRated ?? specsZI?.if_max ?? 0.1);
    const Is_fZI = shockleyIsFromVf(vfZI, iRatedZI, nFZI, vt);
    const izKneeZI = Number(comp.params.iz_knee ?? 0.005);
    const nZZI = 1.0;
    ctx.setElementCurrent(comp.id, breakdownDiodeCurrent(vd, Is_fZI, izKneeZI, nFZI, nZZI, vt, vzZI));
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Guarded tangent slope of the breakdown law at the OP junction voltage,
    // with the same parameter derivation as the Newton stamp.
    const pins = comp.pins;
    if (pins.length < 2) return;
    if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) return;
    const vt = ctx.junctionVt();
    const ai = ctx.pinNode(comp.id, pins[0].id);
    const ci = ctx.pinNode(comp.id, pins[1].id);
    const vd = ctx.opVoltage(ai) - ctx.opVoltage(ci);
    const specs = ctx.electricalSpecs(comp);
    const vf = ctx.junctionVf(Number(comp.params.vf ?? specs?.vf ?? 0.7));
    const nF = Number(comp.params.n ?? N_DIODE);
    const iRated = Number(comp.params.iRated ?? specs?.if_max ?? 0.1);
    const Is_f = shockleyIsFromVf(vf, iRated, nF, vt);
    const izKnee = Number(comp.params.iz_knee ?? 0.005);
    const gd = breakdownConductanceAtOp(
      vd,
      Is_f,
      izKnee,
      nF,
      1.0,
      vt,
      Number(comp.params.vz ?? 5.1),
    );
    stampAcAdmittance(ac, ai, ci, gd + AC_GMIN, 0);
  },
};

export const tvsDiodeModel: DeviceModel = {
  kinds: ["tvs_diode"],
  stamp: (ctx, comp, xGuess, _h) => {
    const pins = comp.pins;
    const vt = ctx.junctionVt();
    if (pins.length < 2) return;
    const ai = ctx.pinNode(comp.id, pins[0].id);
    const ci = ctx.pinNode(comp.id, pins[1].id);
    if (comp.params.burnt) {
      return;
    }
    const vd = ctx.vAt(xGuess, ai) - ctx.vAt(xGuess, ci);
    const vbrT = Number(comp.params.vbr ?? 6.8);
    const bidir = Number(comp.params.bidirectional ?? 0) !== 0;
    const iRatedT = Number(comp.params.iRated ?? 0.1);
    const izKneeT = Number(comp.params.iz_knee ?? 0.005);
    const nZT = 1.0;

    if (!bidir) {
      // Unidirectional: forward diode (Vf ≈ 1.1 V like a rectifier) + reverse breakdown at vbr.
      const vfT = ctx.junctionVf(1.1);
      const Is_fT = shockleyIsFromVf(vfT, iRatedT, N_DIODE, vt);
      stampBreakdownDiode(ctx.mna, ai, ci, vd, Is_fT, izKneeT, N_DIODE, nZT, vt, vbrT);
    } else {
      // Bidirectional: symmetric — breakdown in both polarities. Each side
      // is an OVERDRIVE exponential anchored at its knee (exponent zero at
      // |v| = vbr, prefactor = izKnee; no "-1" — pure exponentials vanish
      // at rest and cannot leak knee-scale current at small |v|).
      // Overdrive clamps at 5*nZ*Vt past each knee, as in stampBreakdownDiode.
      const VtNZT = nZT * vt;
      const vSatOver = 5 * VtNZT;

      const argPos = vd - vbrT;          // forward breakdown argument (pos when v > vbr)
      const argNeg = -(vd + vbrT);        // reverse breakdown argument (pos when v < -vbr)
      const argPosClamped = argPos > vSatOver ? vSatOver : argPos;
      const argNegClamped = argNeg > vSatOver ? vSatOver : argNeg;

      const ePos = Math.exp(argPosClamped / VtNZT);
      const eNeg = Math.exp(argNegClamped / VtNZT);
      const gPos = (izKneeT * ePos) / VtNZT;
      const gNeg = (izKneeT * eNeg) / VtNZT;

      // Continue linearly beyond each exponential guard. Anchoring a
      // clamped current at the raw voltage makes Newton crawl by ~nVt per
      // iteration from a cold high-voltage guess; the true tangent
      // extension is continuous and solves that guarded region directly.
      const iPos = izKneeT * ePos + gPos * Math.max(0, argPos - vSatOver);
      const iNeg = izKneeT * eNeg + gNeg * Math.max(0, argNeg - vSatOver);
      const id = iPos - iNeg;
      const GMIN_D = 1e-12;
      const g = gPos + gNeg + GMIN_D;
      const ieq = id - (gPos + gNeg) * vd;

      if (ai >= 0) ctx.mna.add(ai, ai, g);
      if (ci >= 0) ctx.mna.add(ci, ci, g);
      if (ai >= 0 && ci >= 0) { ctx.mna.add(ai, ci, -g); ctx.mna.add(ci, ai, -g); }
      if (ai >= 0) ctx.mna.addB(ai, -ieq);
      if (ci >= 0) ctx.mna.addB(ci, ieq);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    // Engine _updateElementI preamble, copied per-handler (same pure reads).
    const pins = comp.pins;
    const a = ctx.pinNode(comp.id, pins[0].id);
    const b = ctx.pinNode(comp.id, pins[1].id);
    const va = ctx.vAt(x, a);
    const vb = ctx.vAt(x, b);
    const vd = va - vb;
    const vt = ctx.junctionVt();
    if (comp.params.burnt) { ctx.setElementCurrent(comp.id, 0); return; }
    const vbrTI = Number(comp.params.vbr ?? 6.8);
    const bidirTI = Number(comp.params.bidirectional ?? 0) !== 0;
    const iRatedTI = Number(comp.params.iRated ?? 0.1);
    const izKneeTI = Number(comp.params.iz_knee ?? 0.005);
    const nZTI = 1.0;

    if (!bidirTI) {
      const vfTI = ctx.junctionVf(1.1);
      const Is_fTI = shockleyIsFromVf(vfTI, iRatedTI, N_DIODE, vt);
      ctx.setElementCurrent(comp.id, breakdownDiodeCurrent(vd, Is_fTI, izKneeTI, N_DIODE, nZTI, vt, vbrTI));
    } else {
      // Mirror the bidirectional stamp branch exactly: overdrive
      // exponentials anchored at each knee, with a tangent extension
      // past the 5*nZ*Vt exponential guard.
      const VtNZTI = nZTI * vt;
      const vSatOverI = 5 * VtNZTI;
      const argPos = vd - vbrTI;
      const argNeg = -(vd + vbrTI);
      const ePos = Math.exp(Math.min(argPos, vSatOverI) / VtNZTI);
      const eNeg = Math.exp(Math.min(argNeg, vSatOverI) / VtNZTI);
      const gPos = (izKneeTI * ePos) / VtNZTI;
      const gNeg = (izKneeTI * eNeg) / VtNZTI;
      const iPos = izKneeTI * ePos + gPos * Math.max(0, argPos - vSatOverI);
      const iNeg = izKneeTI * eNeg + gNeg * Math.max(0, argNeg - vSatOverI);
      ctx.setElementCurrent(comp.id, iPos - iNeg);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same guarded slopes the transient stamp computes: forward Shockley +
    // reverse overdrive for the unidirectional part, the two symmetric
    // overdrive slopes for the bidirectional part.
    const pins = comp.pins;
    if (pins.length < 2) return;
    if (comp.params.burnt) return;
    const vt = ctx.junctionVt();
    const ai = ctx.pinNode(comp.id, pins[0].id);
    const ci = ctx.pinNode(comp.id, pins[1].id);
    const vd = ctx.opVoltage(ai) - ctx.opVoltage(ci);
    const vbr = Number(comp.params.vbr ?? 6.8);
    const bidir = Number(comp.params.bidirectional ?? 0) !== 0;
    const iRated = Number(comp.params.iRated ?? 0.1);
    const izKnee = Number(comp.params.iz_knee ?? 0.005);
    const nZ = 1.0;
    let gd: number;
    if (!bidir) {
      const vf = ctx.junctionVf(1.1);
      const Is_f = shockleyIsFromVf(vf, iRated, N_DIODE, vt);
      gd = breakdownConductanceAtOp(vd, Is_f, izKnee, N_DIODE, nZ, vt, vbr);
    } else {
      const VtNZ = nZ * vt;
      const vSatOver = 5 * VtNZ;
      const argPos = vd - vbr;
      const argNeg = -(vd + vbr);
      const gPos = (izKnee * Math.exp(Math.min(argPos, vSatOver) / VtNZ)) / VtNZ;
      const gNeg = (izKnee * Math.exp(Math.min(argNeg, vSatOver) / VtNZ)) / VtNZ;
      gd = gPos + gNeg;
    }
    stampAcAdmittance(ac, ai, ci, gd + AC_GMIN, 0);
  },
};

/**
 * Shared Shockley junction stamp for schottky_diode/led/diode — one body in
 * the engine's _stampAll switch, kept as one body here (the internal isLed
 * branch is part of the moved code; see the file header for why the kinds
 * land in two models).
 */
function stampShockleyJunction(
  ctx: DeviceContext,
  comp: DeviceComponent,
  xGuess: Float64Array,
): void {
  const pins = comp.pins;
  if (pins.length < 2) return;
  const ai = ctx.pinNode(comp.id, pins[0].id);
  const ci = ctx.pinNode(comp.id, pins[1].id);
  if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) {
    return;
  }
  const vd = ctx.vAt(xGuess, ai) - ctx.vAt(xGuess, ci);

  const isLed = comp.kind === "led";
  const specs = ctx.electricalSpecs(comp);
  const junctionTempC = isLed
    ? (ctx.state.thermalDevices.get(comp.id)?.temperatureC ?? ctx.ambientTempC())
    : ctx.ambientTempC();
  const vtDevice = thermalVoltage(junctionTempC);
  const Vf = junctionForwardVoltage(
    Number(comp.params.vf ?? specs?.vf ?? (isLed ? 1.8 : 0.7)),
    junctionTempC,
  );
  const n = Number(comp.params.n ?? (isLed ? N_LED : N_DIODE));
  const iRated = Number(comp.params.iRated ?? specs?.if_max ?? (isLed ? IRATED_LED : 0.1));
  const Is =
    comp.params.Is !== undefined
      ? saturationCurrentAtTemperature(Number(comp.params.Is), junctionTempC, n)
      : shockleyIsFromVf(Vf, iRated, n, vtDevice);
  // Raise the exp-clamp for high-Vf diodes (blue/white LEDs have Vf > 40·VtN).
  // Capped at 80·VtN to prevent f64 overflow in the stamp.
  const VtN_led = n * vtDevice;
  const vSat = Math.min(Math.max(40 * VtN_led, Vf + 5 * VtN_led), 80 * VtN_led);
  stampDiodeShockley(ctx.mna, ai, ci, vd, Is, n, vtDevice, vSat);
}

/**
 * Shared small-signal stamp for schottky_diode/led/diode: the guarded
 * Shockley tangent slope at the OP junction voltage, with the identical
 * parameter derivation (temperature-corrected Vf/Is via the committed LED
 * junction temperature, raised vSat for high-Vf parts) the transient stamp
 * uses, plus the same GMIN floor. One body for the same reason
 * stampShockleyJunction is one body.
 */
function acStampShockleyJunction(
  ctx: AcDeviceContext,
  comp: DeviceComponent,
  ac: AcStampSurface,
): void {
  const pins = comp.pins;
  if (pins.length < 2) return;
  if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) return;
  const ai = ctx.pinNode(comp.id, pins[0].id);
  const ci = ctx.pinNode(comp.id, pins[1].id);
  const vd = ctx.opVoltage(ai) - ctx.opVoltage(ci);

  const isLed = comp.kind === "led";
  const specs = ctx.electricalSpecs(comp);
  const junctionTempC = isLed
    ? (ctx.state.thermalDevices.get(comp.id)?.temperatureC ?? ctx.ambientTempC())
    : ctx.ambientTempC();
  const vtDevice = thermalVoltage(junctionTempC);
  const Vf = junctionForwardVoltage(
    Number(comp.params.vf ?? specs?.vf ?? (isLed ? 1.8 : 0.7)),
    junctionTempC,
  );
  const n = Number(comp.params.n ?? (isLed ? N_LED : N_DIODE));
  const iRated = Number(comp.params.iRated ?? specs?.if_max ?? (isLed ? IRATED_LED : 0.1));
  const Is =
    comp.params.Is !== undefined
      ? saturationCurrentAtTemperature(Number(comp.params.Is), junctionTempC, n)
      : shockleyIsFromVf(Vf, iRated, n, vtDevice);
  const VtN = n * vtDevice;
  const vSat = Math.min(Math.max(40 * VtN, Vf + 5 * VtN), 80 * VtN);
  const gd = shockleyConductanceAtOp(vd, Is, n, vtDevice, vSat);
  stampAcAdmittance(ac, ai, ci, gd + AC_GMIN, 0);
}

/** Shared element-current reconstruction for schottky_diode/led/diode. */
function commitShockleyJunctionCurrent(
  ctx: DeviceContext,
  comp: DeviceComponent,
  x: Float64Array,
): void {
  // Engine _updateElementI preamble, copied per-handler (same pure reads).
  const pins = comp.pins;
  const a = ctx.pinNode(comp.id, pins[0].id);
  const b = ctx.pinNode(comp.id, pins[1].id);
  const va = ctx.vAt(x, a);
  const vb = ctx.vAt(x, b);
  const vd = va - vb;
  if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) { ctx.setElementCurrent(comp.id, 0); return; }
  // Reconstruct Shockley current from the converged voltage.
  const isLed = comp.kind === "led";
  const specs = ctx.electricalSpecs(comp);
  const junctionTempC = isLed
    ? (ctx.state.thermalDevices.get(comp.id)?.temperatureC ?? ctx.ambientTempC())
    : ctx.ambientTempC();
  const vtDevice = thermalVoltage(junctionTempC);
  const Vf = junctionForwardVoltage(
    Number(comp.params.vf ?? specs?.vf ?? (isLed ? 1.8 : 0.7)),
    junctionTempC,
  );
  const nCoef = Number(comp.params.n ?? (isLed ? N_LED : N_DIODE));
  const iRated = Number(
    comp.params.iRated ?? specs?.if_max ?? (isLed ? IRATED_LED : 0.1),
  );
  const Is =
    comp.params.Is !== undefined
      ? saturationCurrentAtTemperature(Number(comp.params.Is), junctionTempC, nCoef)
      : shockleyIsFromVf(Vf, iRated, nCoef, vtDevice);
  const VtN = nCoef * vtDevice;
  const vSat = Math.min(Math.max(40 * VtN, Vf + 5 * VtN), 80 * VtN);
  const I = shockleyDiodeCurrent(vd, Is, nCoef, vtDevice, vSat);
  ctx.setElementCurrent(comp.id, I);
}

/**
 * Shared LED overcurrent damage integrator for led/bicolor_led/rgb_led —
 * one body in the engine's _updateFailureStates, kept as one body here
 * (the internal multi-channel branch is part of the moved code).
 */
function commitLedOvercurrentStress(
  ctx: DeviceContext,
  comp: DeviceComponent,
  h: number,
): void {
  const specs = ctx.electricalSpecs(comp);
  const ifMax = specs?.if_max ?? Number(comp.params.iRated ?? IRATED_LED);
  let peakI = Math.abs(ctx.elementCurrent(comp.id) ?? 0);
  if (comp.kind === "bicolor_led" || comp.kind === "rgb_led") {
    // Computed at most once per failure pass and shared across handlers
    // (the context restores the engine's old pass-local cache): the readout
    // is a pure function of the committed solution, and this component's
    // own entry cannot be changed by another component's stress commit, so
    // the shared snapshot holds exactly the floats a fresh read would.
    const channelCurrents = ctx.elementChannelCurrents();
    peakI = Math.max(peakI, ...(channelCurrents[comp.id] ?? []).map(Math.abs));
  }
  const currentRatio = ifMax > 0 ? peakI / ifMax : 0;
  ctx.recordAccumulatedStress(
    "led_failed",
    comp.id,
    "",
    Math.max(0, currentRatio * currentRatio - 1),
    2,
    h,
    LED_DAMAGE_THRESHOLD,
    () => ({
      componentId: comp.id,
      kind: "led_failed",
      since: ctx.simTime() + h,
      value: peakI,
      limit: ifMax,
      message:
        `${comp.id} exceeded its ${(ifMax * 1000).toFixed(1)} mA LED rating with ${(peakI * 1000).toFixed(1)} mA long enough to fail open.`,
    }),
  );
}

export const diodeModel: DeviceModel = {
  kinds: ["schottky_diode", "diode"],
  stamp: (ctx, comp, xGuess, _h) => {
    stampShockleyJunction(ctx, comp, xGuess);
  },
  updateCurrent: (ctx, comp, x) => {
    commitShockleyJunctionCurrent(ctx, comp, x);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    acStampShockleyJunction(ctx, comp, ac);
  },
};

export const ledModel: DeviceModel = {
  kinds: ["led"],
  stamp: (ctx, comp, xGuess, _h) => {
    stampShockleyJunction(ctx, comp, xGuess);
  },
  updateCurrent: (ctx, comp, x) => {
    commitShockleyJunctionCurrent(ctx, comp, x);
  },
  updateFailures: (ctx, comp, _x, h) => {
    commitLedOvercurrentStress(ctx, comp, h);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    acStampShockleyJunction(ctx, comp, ac);
  },
};

export const bicolorLedModel: DeviceModel = {
  kinds: ["bicolor_led"],
  stamp: (ctx, comp, xGuess, _h) => {
    const pins = comp.pins;
    const vt = ctx.junctionVt();
    if (pins.length < 3) return;
    if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) {
      return;
    }
    const kNode = ctx.pinNode(comp.id, "k");
    for (const [aPin, vfParam, vfDefault] of [["a1", "vf1", 1.8], ["a2", "vf2", 2.0]] as [string, string, number][]) {
      const aNode = ctx.pinNode(comp.id, aPin);
      const vd = ctx.vAt(xGuess, aNode) - ctx.vAt(xGuess, kNode);
      const Vf = ctx.junctionVf(Number(comp.params[vfParam] ?? vfDefault));
      const iRated = Number(comp.params.iRated ?? IRATED_LED);
      const Is = shockleyIsFromVf(Vf, iRated, N_LED, vt);
      const VtN_bc = N_LED * vt;
      const vSat_bc = Math.min(Math.max(40 * VtN_bc, Vf + 5 * VtN_bc), 80 * VtN_bc);
      stampDiodeShockley(ctx.mna, aNode, kNode, vd, Is, N_LED, vt, vSat_bc);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    const vt = ctx.junctionVt();
    ctx.setElementCurrent(comp.id, 0);
    if (!comp.params.burnt && !ctx.hasFailure(comp.id, "led_failed")) {
      const kNode = ctx.pinNode(comp.id, "k");
      const iRated = Number(comp.params.iRated ?? IRATED_LED);
      const VtN = N_LED * vt;
      const a1Node = ctx.pinNode(comp.id, "a1");
      const vd1 = ctx.vAt(x, a1Node) - ctx.vAt(x, kNode);
      const Vf1 = ctx.junctionVf(Number(comp.params.vf1 ?? 1.8));
      const Is1 = shockleyIsFromVf(Vf1, iRated, N_LED, vt);
      const vSat1 = Math.min(Math.max(40 * VtN, Vf1 + 5 * VtN), 80 * VtN);
      const I1 = shockleyDiodeCurrent(vd1, Is1, N_LED, vt, vSat1);
      const a2Node = ctx.pinNode(comp.id, "a2");
      const vd2 = ctx.vAt(x, a2Node) - ctx.vAt(x, kNode);
      const Vf2 = ctx.junctionVf(Number(comp.params.vf2 ?? 2.0));
      const Is2 = shockleyIsFromVf(Vf2, iRated, N_LED, vt);
      const vSat2 = Math.min(Math.max(40 * VtN, Vf2 + 5 * VtN), 80 * VtN);
      const I2 = shockleyDiodeCurrent(vd2, Is2, N_LED, vt, vSat2);
      ctx.setElementCurrent(comp.id, Math.max(I1, I2));
    }
  },
  updateFailures: (ctx, comp, _x, h) => {
    commitLedOvercurrentStress(ctx, comp, h);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Per-junction guarded slopes anode -> shared cathode, same parameter
    // derivation (ambient-referenced Vf/Is) as the transient stamp.
    const pins = comp.pins;
    if (pins.length < 3) return;
    if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) return;
    const vt = ctx.junctionVt();
    const kNode = ctx.pinNode(comp.id, "k");
    for (const [aPin, vfParam, vfDefault] of [["a1", "vf1", 1.8], ["a2", "vf2", 2.0]] as [string, string, number][]) {
      const aNode = ctx.pinNode(comp.id, aPin);
      const vd = ctx.opVoltage(aNode) - ctx.opVoltage(kNode);
      const Vf = ctx.junctionVf(Number(comp.params[vfParam] ?? vfDefault));
      const Is = shockleyIsFromVf(Vf, Number(comp.params.iRated ?? IRATED_LED), N_LED, vt);
      const VtN = N_LED * vt;
      const vSat = Math.min(Math.max(40 * VtN, Vf + 5 * VtN), 80 * VtN);
      const gd = shockleyConductanceAtOp(vd, Is, N_LED, vt, vSat);
      stampAcAdmittance(ac, aNode, kNode, gd + AC_GMIN, 0);
    }
  },
};

export const rgbLedModel: DeviceModel = {
  kinds: ["rgb_led"],
  stamp: (ctx, comp, xGuess, _h) => {
    const pins = comp.pins;
    const vt = ctx.junctionVt();
    if (pins.length < 4) return;
    if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) {
      return;
    }
    const comKNode = ctx.pinNode(comp.id, "com_k");
    const channels: [string, string, number][] = [
      ["r_a", "vf_r", 1.8],
      ["g_a", "vf_g", 2.0],
      ["b_a", "vf_b", 3.0],
    ];
    for (const [aPin, vfParam, vfDefault] of channels) {
      const aNode = ctx.pinNode(comp.id, aPin);
      const vd = ctx.vAt(xGuess, aNode) - ctx.vAt(xGuess, comKNode);
      const Vf = ctx.junctionVf(Number(comp.params[vfParam] ?? vfDefault));
      const iRated = Number(comp.params.iRated ?? IRATED_LED);
      const Is = shockleyIsFromVf(Vf, iRated, N_LED, vt);
      const VtN_rgb = N_LED * vt;
      const vSat_rgb = Math.min(Math.max(40 * VtN_rgb, Vf + 5 * VtN_rgb), 80 * VtN_rgb);
      stampDiodeShockley(ctx.mna, aNode, comKNode, vd, Is, N_LED, vt, vSat_rgb);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    const vt = ctx.junctionVt();
    ctx.setElementCurrent(comp.id, 0);
    if (!comp.params.burnt && !ctx.hasFailure(comp.id, "led_failed")) {
      const comKNode = ctx.pinNode(comp.id, "com_k");
      const iRated = Number(comp.params.iRated ?? IRATED_LED);
      const VtN = N_LED * vt;
      const rgbChannels: [string, string, number][] = [
        ["r_a", "vf_r", 1.8],
        ["g_a", "vf_g", 2.0],
        ["b_a", "vf_b", 3.0],
      ];
      let maxI = 0;
      for (const [aPin, vfParam, vfDefault] of rgbChannels) {
        const Vf = ctx.junctionVf(Number(comp.params[vfParam] ?? vfDefault));
        const Is = shockleyIsFromVf(Vf, iRated, N_LED, vt);
        const vSat = Math.min(Math.max(40 * VtN, Vf + 5 * VtN), 80 * VtN);
        const vdCh = ctx.vAt(x, ctx.pinNode(comp.id, aPin)) - ctx.vAt(x, comKNode);
        maxI = Math.max(
          maxI,
          shockleyDiodeCurrent(vdCh, Is, N_LED, vt, vSat),
        );
      }
      ctx.setElementCurrent(comp.id, maxI);
    }
  },
  updateFailures: (ctx, comp, _x, h) => {
    commitLedOvercurrentStress(ctx, comp, h);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Per-channel guarded slopes anode -> common cathode, mirroring the
    // transient stamp's channel table.
    const pins = comp.pins;
    if (pins.length < 4) return;
    if (comp.params.burnt || ctx.hasFailure(comp.id, "led_failed")) return;
    const vt = ctx.junctionVt();
    const comKNode = ctx.pinNode(comp.id, "com_k");
    const channels: [string, string, number][] = [
      ["r_a", "vf_r", 1.8],
      ["g_a", "vf_g", 2.0],
      ["b_a", "vf_b", 3.0],
    ];
    for (const [aPin, vfParam, vfDefault] of channels) {
      const aNode = ctx.pinNode(comp.id, aPin);
      const vd = ctx.opVoltage(aNode) - ctx.opVoltage(comKNode);
      const Vf = ctx.junctionVf(Number(comp.params[vfParam] ?? vfDefault));
      const Is = shockleyIsFromVf(Vf, Number(comp.params.iRated ?? IRATED_LED), N_LED, vt);
      const VtN = N_LED * vt;
      const vSat = Math.min(Math.max(40 * VtN, Vf + 5 * VtN), 80 * VtN);
      const gd = shockleyConductanceAtOp(vd, Is, N_LED, vt, vSat);
      stampAcAdmittance(ac, aNode, comKNode, gd + AC_GMIN, 0);
    }
  },
};

export const bjtModel: DeviceModel = {
  kinds: ["bjt_npn", "bjt_pnp"],
  stamp: (ctx, comp, xGuess, _h) => {
    // Pin order: [C, B, E] — SPICE-native.
    const pins = comp.pins;
    const vt = ctx.junctionVt();
    if (pins.length < 3) return;
    if (ctx.isOpenPin(comp.id, pins[1].id)) return;
    const ci = ctx.pinNode(comp.id, pins[0].id);
    const bi = ctx.pinNode(comp.id, pins[1].id);
    const ei = ctx.pinNode(comp.id, pins[2].id);
    const vCraw = ctx.vAt(xGuess, ci);
    const vBraw = ctx.vAt(xGuess, bi);
    const vE    = ctx.vAt(xGuess, ei);
    const pol = comp.kind === "bjt_npn" ? 1 : -1;
    const Is25 = Number(comp.params.Is ?? 1e-16);
    const betaF = Number(comp.params.betaF ?? 100);
    const betaR = Number(comp.params.betaR ?? 1);
    const nF = Number(comp.params.nF ?? 1);
    const nR = Number(comp.params.nR ?? 1);
    const earlyVoltage = Number(ctx.modelParam(comp, "earlyVoltage", 0));

    // pnjlim each junction independently, then rebuild node voltages
    // with vE held fixed (the conventional choice; both junctions
    // share node B so we let the BE limit carry vB and the BC limit
    // carry vC). On Newton iter 1 the map is empty → `prev` falls
    // back to the raw guess and pnjlim is a no-op.
    const vBEraw = pol * (vBraw - vE);
    const vBCraw = pol * (vBraw - vCraw);
    const cache = ctx.bjtJunctionCache(comp.id) ?? {
      IsT: saturationCurrentAtTemperature(Is25, ctx.ambientTempC(), nF),
      VtNF: nF * vt, VtNR: nR * vt,
      vCritBE: vCritFor(saturationCurrentAtTemperature(Is25, ctx.ambientTempC(), nF), nF * vt),
      vCritBC: vCritFor(saturationCurrentAtTemperature(Is25, ctx.ambientTempC(), nF), nR * vt),
    };
    const prev = ctx.bjtPrevJunction(comp.id) ?? { vBE: vBEraw, vBC: vBCraw };
    const vBElim = pnjlim(vBEraw, prev.vBE, cache.VtNF, cache.vCritBE);
    const vBClim = pnjlim(vBCraw, prev.vBC, cache.VtNR, cache.vCritBC);
    // pnjlim returns vNew unchanged when it does not clamp, so exact
    // inequality is the precise "limiter fired" signal.
    if (vBElim !== vBEraw || vBClim !== vBCraw) {
      ctx.markJunctionLimitedThisIteration();
    }
    ctx.setBjtPrevJunction(comp.id, { vBE: vBElim, vBC: vBClim });
    const vB_eff = vE + pol * vBElim;
    const vC_eff = vB_eff - pol * vBClim;
    stampBJT(
      ctx.mna,
      bi,
      ci,
      ei,
      vB_eff,
      vC_eff,
      vE,
      pol,
      cache.IsT,
      betaF,
      betaR,
      nF,
      nR,
      vt,
      earlyVoltage,
    );
  },
  updateCurrent: (ctx, comp, x) => {
    const pins = comp.pins;
    const vt = ctx.junctionVt();
    if (pins.length < 3) return;
    if (ctx.isOpenPin(comp.id, pins[1].id)) {
      ctx.setElementCurrent(comp.id, 0);
      return;
    }
    const vC = ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id));
    const vB = ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id));
    const vE = ctx.vAt(x, ctx.pinNode(comp.id, pins[2].id));
    const pol = comp.kind === "bjt_npn" ? 1 : -1;
    const Is = saturationCurrentAtTemperature(
      Number(comp.params.Is ?? 1e-16),
      ctx.ambientTempC(),
      Number(comp.params.nF ?? 1),
    );
    const betaF = Number(comp.params.betaF ?? 100);
    const betaR = Number(comp.params.betaR ?? 1);
    const nF = Number(comp.params.nF ?? 1);
    const nR = Number(comp.params.nR ?? 1);
    const earlyVoltage = Number(ctx.modelParam(comp, "earlyVoltage", 0));
    // Report collector current — the useful observable for biasing.
    ctx.setElementCurrent(comp.id, bjtCurrents(
      vB,
      vC,
      vE,
      pol,
      Is,
      betaF,
      betaR,
      nF,
      nR,
      vt,
      earlyVoltage,
    ).ic);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The exact Ebers-Moll Jacobian stampBJT writes, evaluated at the raw OP
    // node voltages (at a converged point pnjlim is inert: the accepted
    // solution IS the linearization point). bjtLinearization is the factored
    // core of stampBJT, so every entry — gpi/gmu/gm structure and the Early
    // output conductance included — is the transient matrix's own float.
    // Temperature-scaled Is mirrors the per-load junction cache's expression.
    const pins = comp.pins;
    if (pins.length < 3) return;
    if (ctx.isOpenPin(comp.id, pins[1].id)) return;
    const vt = ctx.junctionVt();
    const ci = ctx.pinNode(comp.id, pins[0].id);
    const bi = ctx.pinNode(comp.id, pins[1].id);
    const ei = ctx.pinNode(comp.id, pins[2].id);
    const nF = Number(comp.params.nF ?? 1);
    const lin = bjtLinearization(
      ctx.opVoltage(bi),
      ctx.opVoltage(ci),
      ctx.opVoltage(ei),
      comp.kind === "bjt_npn" ? 1 : -1,
      saturationCurrentAtTemperature(
        Number(comp.params.Is ?? 1e-16),
        ctx.ambientTempC(),
        nF,
      ),
      Number(comp.params.betaF ?? 100),
      Number(comp.params.betaR ?? 1),
      nF,
      Number(comp.params.nR ?? 1),
      vt,
      Number(ctx.modelParam(comp, "earlyVoltage", 0)),
    );
    ac.addAc(bi, bi, lin.gbB, 0);
    ac.addAc(bi, ci, lin.gbC, 0);
    ac.addAc(bi, ei, lin.gbE, 0);
    ac.addAc(ci, bi, lin.gcB, 0);
    ac.addAc(ci, ci, lin.gcC, 0);
    ac.addAc(ci, ei, lin.gcE, 0);
    ac.addAc(ei, bi, lin.geB, 0);
    ac.addAc(ei, ci, lin.geC, 0);
    ac.addAc(ei, ei, lin.geE, 0);
  },
};

export const mosfetModel: DeviceModel = {
  kinds: ["nmos", "pmos"],
  stamp: (ctx, comp, xGuess, h) => {
    // Pin order: [D, G, S].
    const pins = comp.pins;
    const vt = ctx.junctionVt();
    if (pins.length < 3) return;
    const di = ctx.pinNode(comp.id, pins[0].id);
    const gi = ctx.pinNode(comp.id, pins[1].id);
    const si = ctx.pinNode(comp.id, pins[2].id);
    const vD = ctx.vAt(xGuess, di);
    const vG = ctx.vAt(xGuess, gi);
    const vS = ctx.vAt(xGuess, si);
    const pol = comp.kind === "nmos" ? 1 : -1;
    const VTO = Number(comp.params.vto ?? 0.7);
    const K = Number(comp.params.k ?? 0.02);
    const lambda_ = Number(comp.params.lambda ?? 0);
    // Gate charge is integration state, not DC topology: the gate is
    // open at the operating point (I_G = 0), so dc-mode solves drop
    // Cgs/Cgd exactly like the capacitor case above skips its companion.
    const cgs = ctx.dcSolveMode() ? 0 : Number(ctx.modelParam(comp, "cgs", 0));
    const cgd = ctx.dcSolveMode() ? 0 : Number(ctx.modelParam(comp, "cgd", 0));
    const gateState = ctx.state.mosfetGates.get(comp.id) ?? { vgs: 0, vgd: 0 };
    // Gate caps stay backward Euler in every integration mode:
    // MosfetGateState tracks only history voltages, and threading branch
    // currents through stampMOSFET is not the trivial change Wave A2
    // allows for partial coverage (op-amp pole and coil companions
    // likewise remain BE this wave).
    stampMOSFET(
      ctx.mna,
      di,
      gi,
      si,
      vD,
      vG,
      vS,
      pol,
      VTO,
      K,
      lambda_,
      ctx.junctionVf(Number(comp.params.bodyDiodeVf ?? 0.7)),
      vt,
      {
        h,
        cgs,
        cgd,
        vgsPrev: gateState.vgs,
        vgdPrev: gateState.vgd,
      },
    );
  },
  commitState: (ctx, comp, x) => {
    if (!(comp.pins.length >= 3)) return;
    const drain = ctx.pinNode(comp.id, comp.pins[0].id);
    const gate = ctx.pinNode(comp.id, comp.pins[1].id);
    const source = ctx.pinNode(comp.id, comp.pins[2].id);
    const vG = ctx.vAt(x, gate);
    ctx.state.mosfetGates.set(comp.id, {
      vgs: vG - ctx.vAt(x, source),
      vgd: vG - ctx.vAt(x, drain),
    });
  },
  updateCurrent: (ctx, comp, x, h) => {
    const pins = comp.pins;
    const vt = ctx.junctionVt();
    if (pins.length < 3) return;
    const vD = ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id));
    const vG = ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id));
    const vS = ctx.vAt(x, ctx.pinNode(comp.id, pins[2].id));
    const pol = comp.kind === "nmos" ? 1 : -1;
    const VTO = Number(comp.params.vto ?? 0.7);
    const K = Number(comp.params.k ?? 0.02);
    const lambda_ = Number(comp.params.lambda ?? 0);
    const channelCurrent = mosDrainCurrent(vD, vG, vS, pol, VTO, K, lambda_);

    // Drain-terminal current also includes the intrinsic body diode and
    // Cgd displacement current. Reporting channel current alone violates
    // KCL during reverse conduction and gate/drain transients.
    const bodyDiodeVf = ctx.junctionVf(Number(comp.params.bodyDiodeVf ?? 0.7));
    const bodyDiodeIs = shockleyIsFromVf(bodyDiodeVf, 1, 1, vt);
    const bodyVoltage = pol === 1 ? vS - vD : vD - vS;
    const bodyAnodeToCathode = shockleyDiodeCurrent(bodyVoltage, bodyDiodeIs, 1, vt);
    const bodyDrainCurrent = -pol * bodyAnodeToCathode;

    const cgd = Math.max(0, Number(ctx.modelParam(comp, "cgd", 0)));
    const previousVgd = ctx.previousMosfetGate(comp.id)?.vgd ?? 0;
    const currentVgd = vG - vD;
    const cgdDrainCurrent = h > 0
      ? -cgd * (currentVgd - previousVgd) / h
      : 0;
    ctx.setElementCurrent(comp.id, channelCurrent + bodyDrainCurrent + cgdDrainCurrent);
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Channel gm/gds at the OP — the same piecewise Shichman-Hodges Jacobian
    // (GMIN floor included) stampMOSFET writes — plus the intrinsic body
    // diode's guarded slope (part of that same transient stamp; ~GMIN at any
    // forward-conduction OP, load-bearing at a freewheeling one) and the
    // lumped gate charge as true j*omega*C admittances between G-S and G-D.
    const pins = comp.pins;
    if (pins.length < 3) return;
    const vt = ctx.junctionVt();
    const di = ctx.pinNode(comp.id, pins[0].id);
    const gi = ctx.pinNode(comp.id, pins[1].id);
    const si = ctx.pinNode(comp.id, pins[2].id);
    const vD = ctx.opVoltage(di);
    const vG = ctx.opVoltage(gi);
    const vS = ctx.opVoltage(si);
    const pol = comp.kind === "nmos" ? 1 : -1;
    const VTO = Number(comp.params.vto ?? 0.7);
    const K = Number(comp.params.k ?? 0.02);
    const lambda_ = Number(comp.params.lambda ?? 0);
    const vGS = pol * (vG - vS);
    const vDS = pol * (vD - vS);
    const vOV = vGS - VTO;
    let gm = 0;
    let gds = 0;
    if (vOV <= 0) {
      gds = AC_GMIN;
    } else if (vDS < vOV) {
      const m = 1 + lambda_ * vDS;
      gm = 2 * K * vDS * m;
      gds = K * (2 * vOV - 2 * vDS) * m + K * (2 * vOV * vDS - vDS * vDS) * lambda_;
      if (gds < AC_GMIN) gds = AC_GMIN;
    } else {
      const m = 1 + lambda_ * vDS;
      gm = 2 * K * vOV * m;
      gds = K * vOV * vOV * lambda_;
      if (gds < AC_GMIN) gds = AC_GMIN;
    }
    ac.addAc(di, gi, gm, 0);
    ac.addAc(di, di, gds, 0);
    ac.addAc(di, si, -gm - gds, 0);
    ac.addAc(si, gi, -gm, 0);
    ac.addAc(si, di, -gds, 0);
    ac.addAc(si, si, gm + gds, 0);

    const [bdAnode, bdCathode, vBd] = pol === 1
      ? [si, di, vS - vD]
      : [di, si, vD - vS];
    const bodyDiodeIs = shockleyIsFromVf(
      ctx.junctionVf(Number(comp.params.bodyDiodeVf ?? 0.7)),
      1.0,
      1.0,
      vt,
    );
    const gBody = shockleyConductanceAtOp(vBd, bodyDiodeIs, 1.0, vt);
    stampAcAdmittance(ac, bdAnode, bdCathode, gBody + AC_GMIN, 0);

    const cgs = Math.max(0, Number(ctx.modelParam(comp, "cgs", 0)));
    const cgd = Math.max(0, Number(ctx.modelParam(comp, "cgd", 0)));
    if (cgs > 0) stampAcAdmittance(ac, gi, si, 0, omega * cgs);
    if (cgd > 0) stampAcAdmittance(ac, gi, di, 0, omega * cgd);
  },
};

export const tl431Model: DeviceModel = {
  kinds: ["tl431"],
  stamp: (ctx, comp, xGuess, _h) => {
    // The TL431 sinks current cathode→anode to hold V(REF)−V(ANODE) at the
    // internal bandgap Vref = 2.495 V.  Modelled as a VCCS (voltage-controlled
    // current source):
    //   I_ka = ikKnee * exp((V(ref) - V(anode) - Vref) / (nZ * Vt))
    // I_ka flows FROM cathode TO anode.  Controlling input is V(ref) - V(anode)
    // only — V(cathode) does not appear in the expression.
    //
    // Newton companion stamp in G*V = b form:
    //   Residual convention: f_k = sum(currents_leaving_k); solved as G*V = b.
    //   A current leaving node k contributes +I to f_k → subtract from b[k].
    //   A current entering node k contributes -I to f_k → add to b[k].
    //
    // I_ka leaves CATHODE and enters ANODE:
    //   f_cathode += I_ka   → df/dV_ref   = +gZ  → G[cath][ref]   += gZ
    //                         df/dV_anode  = -gZ  → G[cath][anode] -= gZ
    //                         b[cath] -= Ieq       (Norton RHS)
    //   f_anode   -= I_ka   → df/dV_ref   = -gZ  → G[anode][ref]   -= gZ
    //                         df/dV_anode  = +gZ  → G[anode][anode] += gZ
    //                         b[anode] += Ieq
    //
    //   where Ieq = I_ka - gZ*(vRefG - vAnodeG)  [linearised at xGuess]
    //
    // Note: G[cath][ref] += gZ means that if refNode == cathNode (REF shorted to
    // cathode), the self-conductance G[cath][cath] gains +gZ (positive), which is
    // exactly what a diode-to-GND stamp would give.  No negative diagonal entries.
    //
    // A large bias resistor REF→anode prevents REF from floating when the
    // feedback divider is absent.  stampResistor is bounds-safe (skips GND rows).
    const vt = ctx.junctionVt();
    const refNode   = ctx.pinNode(comp.id, "ref");
    const anodeNode = ctx.pinNode(comp.id, "anode");
    const cathNode  = ctx.pinNode(comp.id, "cathode");

    // cathode must be in the MNA matrix (anode is often GND = -1; ref must
    // always be wired, but we guard every call to be safe).
    if (cathNode < 0) return;

    // 1 MΩ bias to keep REF from floating.  stampResistor skips GND-side entries.
    stampResistor(ctx.mna, refNode, anodeNode, 1e6);

    const TL431_VREF = 2.495;
    const TL431_IK   = 1e-3;   // 1 mA knee current at threshold
    const TL431_NZ   = 0.5;    // sub-unity emission coefficient → sharper knee
    const VtNZ  = TL431_NZ * vt;
    const vSatZ = 5 * VtNZ; // overdrive clamp (same limit as breakdownDiodeCurrent)

    const vRefG   = ctx.vAt(xGuess, refNode);
    const vAnodeG = ctx.vAt(xGuess, anodeNode);
    const drive   = vRefG - vAnodeG - TL431_VREF;
    const driveC  = drive > vSatZ ? vSatZ : drive; // prevent exp overflow

    const eZ   = Math.exp(driveC / VtNZ);
    const gPhysical = (TL431_IK * eZ) / VtNZ;
    // The guarded high-drive region is a finite-slope tangent extension,
    // not a flat current clamp. This keeps the model continuous and lets
    // a cold Newton solve reach the regulation knee without leaking a
    // rejected iterate into the next timestep.
    const I_ka = TL431_IK * eZ + gPhysical * Math.max(0, drive - vSatZ);
    const gZ   = gPhysical + 1e-12; // +GMIN floor avoids singular G at cutoff

    // Norton equivalent value at the linearisation point.
    // The numerical GMIN transconductance is zero-origin, so it adds no
    // constant offset to the physical companion source.
    const Ieq = I_ka - gPhysical * (vRefG - vAnodeG);

    // VCCS stamp: all mna.add / mna.addB calls individually guarded — GND node
    // (-1) is not in the matrix and mna.add has no bounds check.
    //
    // Cathode row (current leaves cathode):
    if (cathNode >= 0 && refNode   >= 0) ctx.mna.add(cathNode, refNode,    gZ);
    if (cathNode >= 0 && anodeNode >= 0) ctx.mna.add(cathNode, anodeNode, -gZ);
    if (cathNode >= 0) ctx.mna.addB(cathNode, -Ieq);

    // Anode row (current enters anode):
    if (anodeNode >= 0 && refNode   >= 0) ctx.mna.add(anodeNode, refNode,   -gZ);
    if (anodeNode >= 0)                   ctx.mna.add(anodeNode, anodeNode,  gZ);
    if (anodeNode >= 0) ctx.mna.addB(anodeNode, Ieq);
  },
  updateCurrent: (ctx, comp, x) => {
    // Report the cathode-to-anode shunt current from the solved node voltages.
    // I_ka = ikKnee * exp((V(ref)-V(anode)-Vref)/(nZ*Vt)) — same math as the stamp.
    const vt = ctx.junctionVt();
    const TL431_VREF_I = 2.495;
    const TL431_IK_I   = 1e-3;
    const TL431_NZ_I   = 0.5;
    const VtNZ_I = TL431_NZ_I * vt;
    const vSatZ_I = 5 * VtNZ_I;
    const vRefI   = ctx.vAt(x, ctx.pinNode(comp.id, "ref"));
    const vAnodeI = ctx.vAt(x, ctx.pinNode(comp.id, "anode"));
    const driveI  = vRefI - vAnodeI - TL431_VREF_I;
    const driveCI = driveI > vSatZ_I ? vSatZ_I : driveI;
    const baseI = TL431_IK_I * Math.exp(driveCI / VtNZ_I);
    const slopeI = baseI / VtNZ_I;
    ctx.setElementCurrent(comp.id, baseI + slopeI * Math.max(0, driveI - vSatZ_I));
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The VCCS small-signal transconductance at the OP: the identical
    // asymmetric Jacobian pattern the Newton stamp writes (cathode/anode
    // rows, ref/anode columns — V(cathode) does not control the current),
    // plus the same 1 MOhm REF bias resistor.
    const vt = ctx.junctionVt();
    const refNode = ctx.pinNode(comp.id, "ref");
    const anodeNode = ctx.pinNode(comp.id, "anode");
    const cathNode = ctx.pinNode(comp.id, "cathode");
    if (cathNode < 0) return;
    stampAcAdmittance(ac, refNode, anodeNode, 1 / 1e6, 0);
    const TL431_VREF = 2.495;
    const TL431_IK = 1e-3;
    const TL431_NZ = 0.5;
    const VtNZ = TL431_NZ * vt;
    const vSatZ = 5 * VtNZ;
    const drive = ctx.opVoltage(refNode) - ctx.opVoltage(anodeNode) - TL431_VREF;
    const driveC = drive > vSatZ ? vSatZ : drive;
    const gZ = (TL431_IK * Math.exp(driveC / VtNZ)) / VtNZ + AC_GMIN;
    ac.addAc(cathNode, refNode, gZ, 0);
    ac.addAc(cathNode, anodeNode, -gZ, 0);
    ac.addAc(anodeNode, refNode, -gZ, 0);
    ac.addAc(anodeNode, anodeNode, gZ, 0);
  },
};
