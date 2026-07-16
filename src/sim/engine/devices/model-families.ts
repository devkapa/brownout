/**
 * MODEL-FAMILIES device cohort (Wave A6): coupled_inductor, njfet, pjfet,
 * scr, triac, opto_npn, crystal, analog_switch.
 *
 * Unlike the A4 cohorts, nothing here is moved engine code — these are the
 * first families implemented registry-first, so this module doubles as the
 * reference for how a NEW device family lands:
 *
 * - Transient stamps compose the shared companion helpers in elements.ts
 *   (BE and trap variants selected via ctx.useTrapThisSolve(), exactly the
 *   capacitor/inductor gating) or follow the committed-regime pattern
 *   (regimes decided POST-SOLVE from the accepted solution, never from a
 *   Newton iterate — the relay/regulator rule).
 * - Element state lives in the existing engine maps (caps/capsI, inds/indsV,
 *   icState) under this module's composite keys, so snapshots, adaptive-step
 *   rollback, the coarse/refined error walk, and load-by-ID carry all apply
 *   without any engine special case.
 * - DC operating-point normalisation happens INSIDE commitState via
 *   ctx.dcSolveMode() (the dc stamps are R-only shorts/opens, so companion
 *   math would commit garbage there). The A4 kinds get the same treatment
 *   from _commitDcOperatingPointState; owning it here keeps the engine free
 *   of composite-key knowledge.
 * - Every model carries an acStamp that is the exact linearization of its
 *   transient stamp about the held OP: conductances keep their Newton
 *   Jacobian values (GMIN floors included), energy storage becomes exact
 *   complex admittance, committed regimes select the same piecewise-linear
 *   region the transient stamp would use.
 * - crystal and opto_npn use the Wave A6 internal-node hook (see
 *   DeviceModel.internalNodes) for their mid-branch/base rows.
 *
 * Import layering (registry header, decision 7): pure helper FUNCTIONS and
 * exported consts from sim-engine.ts are safe to import despite the module
 * cycle because handlers dereference them only at solve time. Never read
 * sim-engine bindings at module evaluation time from this file.
 */

import type {
  DeviceComponent,
  DeviceContext,
  DeviceModel,
} from "../device-registry.js";
import { stampAcAdmittance } from "../ac-system.js";
import {
  bjtLinearization,
  capacitorInternalVoltage,
  capacitorInternalVoltageTrap,
  capacitorSeriesCurrent,
  capacitorSeriesCurrentTrap,
  coupledInductorCompanion,
  coupledInductorCompanionTrap,
  coupledInductorWindingCurrents,
  inductorWindingCurrent,
  inductorWindingCurrentTrap,
  jfetLinearization,
  shockleyDiodeCurrent,
  shockleyIsFromVf,
  stampBJT,
  stampCapacitor,
  stampCapacitorTrap,
  stampCoupledInductor,
  stampCurrentSource,
  stampDiodeShockley,
  stampInductor,
  stampInductorTrap,
  stampJFET,
  stampResistor,
} from "../elements.js";
import { pnjlim } from "../newton.js";
import { AC_GMIN, shockleyConductanceAtOp } from "./semiconductors.js";
import {
  N_DIODE,
  N_LED,
  saturationCurrentAtTemperature,
  seriesLossResistance,
  vCritFor,
} from "../sim-engine.js";

// ── Shared helpers ──────────────────────────────────────────────────────────

/**
 * Composite state-map keys, cached per component id. Keys are pure functions
 * of the id, so a module-level cache is safe across engines and loads (the
 * explicitCatalogPartCache precedent in sim-engine.ts); caching keeps the
 * per-Newton-iteration stamps free of string building.
 */
const windingKeysById = new Map<string, { w1: string; w2: string }>();
function windingKeys(compId: string): { w1: string; w2: string } {
  let keys = windingKeysById.get(compId);
  if (!keys) {
    keys = { w1: `${compId}:1`, w2: `${compId}:2` };
    windingKeysById.set(compId, keys);
  }
  return keys;
}

const crystalKeysById = new Map<string, { cs: string; ls: string; c0: string }>();
function crystalKeys(compId: string): { cs: string; ls: string; c0: string } {
  let keys = crystalKeysById.get(compId);
  if (!keys) {
    keys = { cs: `${compId}:cs`, ls: `${compId}:ls`, c0: `${compId}:c0` };
    crystalKeysById.set(compId, keys);
  }
  return keys;
}

/**
 * Raised exponential guard for junctions whose anchor voltage can exceed the
 * generic 40*n*Vt clamp (SCR/triac gate thresholds, opto LED Vf). The guard
 * must clear the anchor: Is is anchored so the junction passes the datasheet
 * current AT anchorV, and a saturation knee below that point collapses the
 * tangent extension's slope to the knee's — the anchored current becomes
 * unreachable at any physical drive. For the thyristor gate that silently
 * disables triggering (datasheet Vgt runs to ~2.5 V while the LED stamps'
 * fixed 80*n*Vt cap is ~2.07 V at n = 1). The 700*n*Vt ceiling keeps
 * exp(vSat/(n*Vt)) finite in f64 (Math.exp overflows near 709) even for
 * garbage-scale authored anchors.
 */
function raisedJunctionVSat(anchorV: number, vtN: number): number {
  return Math.min(Math.max(40 * vtN, anchorV + 5 * vtN), 700 * vtN);
}

// ── Coupled inductors (transformer) ─────────────────────────────────────────

/**
 * The [L] matrix is exactly singular at k = 1 (an ideal transformer has no
 * admittance form: L1*L2 - M^2 = 0), so the companion's 2x2 inverse would
 * divide by zero. Clamping to 0.9999 leaves ~2e-4 of leakage inductance —
 * numerically comfortable at f64 while indistinguishable from ideal at
 * circuit tolerances.
 */
const COUPLING_K_MAX = 0.9999;

interface CoupledInductorParams {
  l1: number;
  l2: number;
  m: number;
  dcr1: number;
  dcr2: number;
}

function coupledInductorParams(
  ctx: Pick<DeviceContext, "modelParam">,
  comp: DeviceComponent,
): CoupledInductorParams {
  const l1 = Math.max(1e-12, Number(comp.params.l1 ?? 1e-3));
  const l2 = Math.max(1e-12, Number(comp.params.l2 ?? 1e-3));
  const k = Math.min(COUPLING_K_MAX, Math.max(0, Number(comp.params.k ?? 0.98)));
  return {
    l1,
    l2,
    m: k * Math.sqrt(l1 * l2),
    dcr1: seriesLossResistance(ctx.modelParam(comp, "dcr1", 0)),
    dcr2: seriesLossResistance(ctx.modelParam(comp, "dcr2", 0)),
  };
}

export const coupledInductorModel: DeviceModel = {
  kinds: ["coupled_inductor"],
  stamp: (ctx, comp, _xGuess, h) => {
    // Pins a1/b1 = winding 1, a2/b2 = winding 2; a1 and a2 are the dotted
    // terminals (see coupledInductorCompanion's derivation).
    if (comp.pins.length < 4) return;
    const a1 = ctx.pinNode(comp.id, "a1");
    const b1 = ctx.pinNode(comp.id, "b1");
    const a2 = ctx.pinNode(comp.id, "a2");
    const b2 = ctx.pinNode(comp.id, "b2");
    const { l1, l2, m, dcr1, dcr2 } = coupledInductorParams(ctx, comp);
    if (ctx.dcSolveMode()) {
      // Each winding is a dc short through its own resistance — the exact
      // rule inductorModel applies, per winding. The mutual term is pure
      // d/dt coupling and contributes nothing at DC.
      stampResistor(ctx.mna, a1, b1, Math.max(dcr1, 1e-6));
      stampResistor(ctx.mna, a2, b2, Math.max(dcr2, 1e-6));
      return;
    }
    const keys = windingKeys(comp.id);
    const i1Prev = ctx.state.inds.get(keys.w1) ?? 0;
    const i2Prev = ctx.state.inds.get(keys.w2) ?? 0;
    const companion = ctx.useTrapThisSolve()
      ? coupledInductorCompanionTrap(
          l1,
          l2,
          m,
          h,
          i1Prev,
          i2Prev,
          ctx.state.indsV.get(keys.w1) ?? 0,
          ctx.state.indsV.get(keys.w2) ?? 0,
          dcr1,
          dcr2,
        )
      : coupledInductorCompanion(l1, l2, m, h, i1Prev, i2Prev, dcr1, dcr2);
    stampCoupledInductor(ctx.mna, a1, b1, a2, b2, companion);
  },
  commitState: (ctx, comp, x, h) => {
    if (comp.pins.length < 4) return;
    const v1 = ctx.vAt(x, ctx.pinNode(comp.id, "a1"))
      - ctx.vAt(x, ctx.pinNode(comp.id, "b1"));
    const v2 = ctx.vAt(x, ctx.pinNode(comp.id, "a2"))
      - ctx.vAt(x, ctx.pinNode(comp.id, "b2"));
    const { l1, l2, m, dcr1, dcr2 } = coupledInductorParams(ctx, comp);
    const keys = windingKeys(comp.id);
    if (ctx.dcSolveMode()) {
      // The dc stamp was exactly R = max(dcr, 1e-6) per winding, so the
      // solved terminal voltages map 1:1 onto DC winding currents — the
      // normalisation _commitDcOperatingPointState applies to plain
      // inductors, owned here because the composite keys are this model's.
      // The trap history voltage refreshes either way; the OP commit forces
      // a BE re-anchor before any trap stamp could read it.
      ctx.state.inds.set(keys.w1, v1 / Math.max(dcr1, 1e-6));
      ctx.state.inds.set(keys.w2, v2 / Math.max(dcr2, 1e-6));
      ctx.state.indsV.set(keys.w1, v1);
      ctx.state.indsV.set(keys.w2, v2);
      return;
    }
    // Same trap gating rationale as inductorModel.commitState: the stamp's
    // companion choice is stable across the whole solve, and history upkeep
    // follows the MODE so a forced-BE anchor step still refreshes it.
    const useTrap = ctx.useTrapThisSolve();
    const maintainTrapHistory = ctx.integrationMethod() === "trap";
    const i1Prev = ctx.state.inds.get(keys.w1) ?? 0;
    const i2Prev = ctx.state.inds.get(keys.w2) ?? 0;
    const companion = useTrap
      ? coupledInductorCompanionTrap(
          l1,
          l2,
          m,
          h,
          i1Prev,
          i2Prev,
          ctx.state.indsV.get(keys.w1) ?? 0,
          ctx.state.indsV.get(keys.w2) ?? 0,
          dcr1,
          dcr2,
        )
      : coupledInductorCompanion(l1, l2, m, h, i1Prev, i2Prev, dcr1, dcr2);
    const { i1, i2 } = coupledInductorWindingCurrents(companion, v1, v2);
    ctx.state.inds.set(keys.w1, i1);
    ctx.state.inds.set(keys.w2, i2);
    if (maintainTrapHistory) {
      ctx.state.indsV.set(keys.w1, v1);
      ctx.state.indsV.set(keys.w2, v2);
    }
  },
  updateCurrent: (ctx, comp, _x) => {
    // Element-current convention is pin0 -> pin1: winding 1 (a1 -> b1),
    // committed just above in commitState.
    ctx.setElementCurrent(comp.id, ctx.state.inds.get(windingKeys(comp.id).w1) ?? 0);
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Exact complex 2-port admittance of the coupled windings:
    //
    //   Z = | dcr1 + jwL1   jwM        |    Y = Z^-1 = adj(Z)/det(Z)
    //       | jwM           dcr2 + jwL2|
    //
    //   det = (dcr1 + jwL1)(dcr2 + jwL2) - (jwM)^2
    //       = [dcr1*dcr2 - w^2(L1*L2 - M^2)] + jw(dcr1*L2 + dcr2*L1)
    //
    //   Y11 = (dcr2 + jwL2)/det,  Y22 = (dcr1 + jwL1)/det,
    //   Y12 = Y21 = -jwM/det
    //
    // expanded via (a+jb)/(c+jd) = [(ac+bd) + j(bc-ad)]/(c^2+d^2). This is
    // the omega-domain limit of the transient companion (whose history
    // sources vanish in small signal); with k clamped below 1 the real part
    // of det keeps the leakage term, so the sweep stays regular through the
    // coupling extreme.
    if (comp.pins.length < 4) return;
    const a1 = ctx.pinNode(comp.id, "a1");
    const b1 = ctx.pinNode(comp.id, "b1");
    const a2 = ctx.pinNode(comp.id, "a2");
    const b2 = ctx.pinNode(comp.id, "b2");
    const { l1, l2, m, dcr1, dcr2 } = coupledInductorParams(ctx, comp);
    const detRe = dcr1 * dcr2 - omega * omega * (l1 * l2 - m * m);
    const detIm = omega * (dcr1 * l2 + dcr2 * l1);
    const dd = detRe * detRe + detIm * detIm;
    const y11re = (dcr2 * detRe + omega * l2 * detIm) / dd;
    const y11im = (omega * l2 * detRe - dcr2 * detIm) / dd;
    const y22re = (dcr1 * detRe + omega * l1 * detIm) / dd;
    const y22im = (omega * l1 * detRe - dcr1 * detIm) / dd;
    const y12re = (-omega * m * detIm) / dd;
    const y12im = (-omega * m * detRe) / dd;
    stampAcAdmittance(ac, a1, b1, y11re, y11im);
    stampAcAdmittance(ac, a2, b2, y22re, y22im);
    // Cross block (Y12 = Y21), the AC image of stampCoupledInductor's
    // explicit eight-entry incidence.
    ac.addAc(a1, a2, y12re, y12im);
    ac.addAc(a1, b2, -y12re, -y12im);
    ac.addAc(b1, a2, -y12re, -y12im);
    ac.addAc(b1, b2, y12re, y12im);
    ac.addAc(a2, a1, y12re, y12im);
    ac.addAc(a2, b1, -y12re, -y12im);
    ac.addAc(b2, a1, -y12re, -y12im);
    ac.addAc(b2, b1, y12re, y12im);
  },
};

// ── JFET ────────────────────────────────────────────────────────────────────

interface JfetParams {
  vto: number;
  beta: number;
  lambda: number;
  isGate: number;
}

/**
 * Datasheet convention: vto is NEGATIVE for both channel types (depletion
 * mode — the n-JFET conducts idss at vGS = 0 and pinches off at vGS <= vto;
 * the p-JFET applies the same negative number to polarity-flipped voltages).
 * Authored values are coerced onto that convention via -|vto| so a
 * positive-magnitude entry cannot silently build an enhancement device;
 * beta = idss/vto^2 makes Id(vGS = 0) = idss in saturation by construction.
 */
function jfetParams(comp: DeviceComponent): JfetParams {
  const vto = -Math.max(0.05, Math.abs(Number(comp.params.vto ?? -2)));
  const idss = Math.max(1e-9, Number(comp.params.idss ?? 0.01));
  return {
    vto,
    beta: idss / (vto * vto),
    lambda: Math.max(0, Number(comp.params.lambda ?? 0)),
    isGate: Math.max(1e-30, Number(comp.params.Is ?? 1e-14)),
  };
}

export const jfetModel: DeviceModel = {
  kinds: ["njfet", "pjfet"],
  stamp: (ctx, comp, xGuess, _h) => {
    // Pins d/g/s. Channel: Shichman-Hodges companion (stampJFET — the
    // MOSFET Jacobian pattern without body diode or gate charge). Gate:
    // the physical gate-channel junctions, one Shockley diode to each of
    // source and drain — reverse-biased in normal operation, so they
    // contribute ~GMIN leakage; forward gate drive conducts realistically.
    // No pnjlim, matching the standalone-diode rationale in
    // stampDiodeShockley: both junctions are current-limited by the
    // surrounding circuit through the channel/gate resistors.
    if (comp.pins.length < 3) return;
    const di = ctx.pinNode(comp.id, "d");
    const gi = ctx.pinNode(comp.id, "g");
    const si = ctx.pinNode(comp.id, "s");
    const vD = ctx.vAt(xGuess, di);
    const vG = ctx.vAt(xGuess, gi);
    const vS = ctx.vAt(xGuess, si);
    const pol = comp.kind === "njfet" ? 1 : -1;
    const { vto, beta, lambda, isGate } = jfetParams(comp);
    stampJFET(ctx.mna, di, gi, si, vD, vG, vS, pol, vto, beta, lambda);
    const vt = ctx.junctionVt();
    // n-JFET: p-gate against n-channel — anode g, cathodes s/d. p-JFET is
    // the mirror image (anodes s/d, cathode g).
    const [gsAnode, gsCathode] = pol === 1 ? [gi, si] : [si, gi];
    stampDiodeShockley(
      ctx.mna,
      gsAnode,
      gsCathode,
      ctx.vAt(xGuess, gsAnode) - ctx.vAt(xGuess, gsCathode),
      isGate,
      N_DIODE,
      vt,
    );
    const [gdAnode, gdCathode] = pol === 1 ? [gi, di] : [di, gi];
    stampDiodeShockley(
      ctx.mna,
      gdAnode,
      gdCathode,
      ctx.vAt(xGuess, gdAnode) - ctx.vAt(xGuess, gdCathode),
      isGate,
      N_DIODE,
      vt,
    );
  },
  updateCurrent: (ctx, comp, x) => {
    // Drain-terminal current: channel current plus the gate-drain junction's
    // internal contribution (KCL at the drain node — the junction injects
    // into the drain for the n-JFET, out of it for the p-JFET, hence -pol).
    // The gate-source junction never touches the drain node.
    if (comp.pins.length < 3) return;
    const di = ctx.pinNode(comp.id, "d");
    const gi = ctx.pinNode(comp.id, "g");
    const si = ctx.pinNode(comp.id, "s");
    const vD = ctx.vAt(x, di);
    const vG = ctx.vAt(x, gi);
    const vS = ctx.vAt(x, si);
    const pol = comp.kind === "njfet" ? 1 : -1;
    const { vto, beta, lambda, isGate } = jfetParams(comp);
    const vt = ctx.junctionVt();
    const channel = jfetLinearization(vD, vG, vS, pol, vto, beta, lambda).id;
    const vGd = pol === 1 ? vG - vD : vD - vG;
    const gateDrain = shockleyDiodeCurrent(vGd, isGate, N_DIODE, vt);
    ctx.setElementCurrent(comp.id, channel - pol * gateDrain);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Channel gm/gds at the OP — jfetLinearization is the stamp's own
    // Jacobian (GMIN floor included) — plus the two gate junction slopes,
    // exactly the conductances the transient Newton matrix carries there.
    if (comp.pins.length < 3) return;
    const di = ctx.pinNode(comp.id, "d");
    const gi = ctx.pinNode(comp.id, "g");
    const si = ctx.pinNode(comp.id, "s");
    const vD = ctx.opVoltage(di);
    const vG = ctx.opVoltage(gi);
    const vS = ctx.opVoltage(si);
    const pol = comp.kind === "njfet" ? 1 : -1;
    const { vto, beta, lambda, isGate } = jfetParams(comp);
    const { gm, gds } = jfetLinearization(vD, vG, vS, pol, vto, beta, lambda);
    ac.addAc(di, gi, gm, 0);
    ac.addAc(di, di, gds, 0);
    ac.addAc(di, si, -gm - gds, 0);
    ac.addAc(si, gi, -gm, 0);
    ac.addAc(si, di, -gds, 0);
    ac.addAc(si, si, gm + gds, 0);
    const vt = ctx.junctionVt();
    const vGs = pol === 1 ? vG - vS : vS - vG;
    const vGd = pol === 1 ? vG - vD : vD - vG;
    stampAcAdmittance(
      ac,
      gi,
      si,
      shockleyConductanceAtOp(vGs, isGate, N_DIODE, vt) + AC_GMIN,
      0,
    );
    stampAcAdmittance(
      ac,
      gi,
      di,
      shockleyConductanceAtOp(vGd, isGate, N_DIODE, vt) + AC_GMIN,
      0,
    );
  },
};

// ── SCR and triac (committed-regime latching thyristors) ────────────────────

/**
 * Off-state principal leakage: 1 nS (1 GOhm). Deliberately above the 1e-12
 * node shunt so a blocked series string still divides deterministically,
 * while staying nanoamp-scale at circuit voltages ("GMIN leak only").
 */
const THYRISTOR_BLOCKING_G = 1e-9;

interface ThyristorParams {
  vgt: number;
  igt: number;
  ih: number;
  vtm: number;
  ron: number;
  isGate: number;
  vSatGate: number;
}

/**
 * Gate junction saturation current anchored on the datasheet trigger pair:
 * shockleyIsFromVf(vgt, igt) makes the junction pass exactly igt at vgt, so
 * the two trigger conditions (gate current AND gate voltage) coincide by
 * construction and the commit check can assert both without a tuning knob.
 */
function thyristorParams(comp: DeviceComponent, vt: number): ThyristorParams {
  const vgt = Math.max(0.3, Number(comp.params.vgt ?? 0.7));
  const igt = Math.max(1e-6, Number(comp.params.igt ?? 0.005));
  const vtN = N_DIODE * vt;
  return {
    vgt,
    igt,
    ih: Math.max(1e-6, Number(comp.params.ih ?? 0.005)),
    vtm: Math.max(0.1, Number(comp.params.vtm ?? 1.2)),
    ron: Math.max(1e-3, Number(comp.params.ron ?? 0.1)),
    isGate: shockleyIsFromVf(vgt, igt, N_DIODE, vt),
    vSatGate: raisedJunctionVSat(vgt, vtN),
  };
}

export const scrModel: DeviceModel = {
  kinds: ["scr"],
  stamp: (ctx, comp, xGuess, _h) => {
    // Pins a (anode), k (cathode), g (gate). Committed-regime pattern
    // (relay contacts): the icState `on` latch decided at the LAST commit
    // selects which linear principal branch stamps, so the region can never
    // flip inside a Newton solve. Both regimes are pure resistive/junction
    // stamps with no integration state, so the same stamp serves dc-mode OP
    // solves — the A3 regime settle loop re-solves until the committed
    // latch picture stops moving.
    //
    //   blocking:   THYRISTOR_BLOCKING_G leak between a and k
    //   conducting: i(a->k) = (v_ak - vtm)/ron, stamped as the ron
    //               conductance plus a constant Norton source vtm/ron INTO
    //               the anode (i = g*v_ak - g*vtm; the -g*vtm term leaves
    //               node a's KCL as +g*vtm entering, hence the source sign).
    //
    // The gate-cathode junction stamps in both regimes: it is the physical
    // sense element the post-solve trigger decision reads.
    if (comp.pins.length < 3) return;
    const aNode = ctx.pinNode(comp.id, "a");
    const kNode = ctx.pinNode(comp.id, "k");
    const gNode = ctx.pinNode(comp.id, "g");
    const vt = ctx.junctionVt();
    const p = thyristorParams(comp, vt);
    stampDiodeShockley(
      ctx.mna,
      gNode,
      kNode,
      ctx.vAt(xGuess, gNode) - ctx.vAt(xGuess, kNode),
      p.isGate,
      N_DIODE,
      vt,
      p.vSatGate,
    );
    const on = (ctx.state.icState.get(comp.id)?.on ?? 0) >= 0.5;
    if (on) {
      stampResistor(ctx.mna, aNode, kNode, p.ron);
      stampCurrentSource(ctx.mna, aNode, kNode, p.vtm / p.ron);
    } else {
      stampResistor(ctx.mna, aNode, kNode, 1 / THYRISTOR_BLOCKING_G);
    }
  },
  commitState: (ctx, comp, x, _h) => {
    // Trigger/dropout decided POST-SOLVE from the accepted solution only.
    // Trigger needs the committed gate drive to reach BOTH datasheet
    // numbers (they coincide by the Is anchoring above, but asserting both
    // keeps the intent explicit) plus a principal bias above the on-state
    // drop: the conducting law (v - vtm)/ron carries no forward current
    // below vtm, so firing there would only stamp a Norton source propping
    // the anode at a phantom vtm rail. Dropout: conduction current below
    // the holding current ih (a reverse-biased on-state reads negative and
    // therefore below ih, so reverse recovery is immediate) — UNLESS the
    // gate still holds trigger drive with forward current flowing: ih is
    // an open-gate datasheet number, and a maintained gate keeps a real
    // SCR conducting below it (dropping here would chatter the latch at
    // the step rate, and bounce the OP regime settle with it). Latching
    // with the gate removed is unchanged: only ih governs.
    if (comp.pins.length < 3) return;
    const vA = ctx.vAt(x, ctx.pinNode(comp.id, "a"));
    const vK = ctx.vAt(x, ctx.pinNode(comp.id, "k"));
    const vG = ctx.vAt(x, ctx.pinNode(comp.id, "g"));
    const vt = ctx.junctionVt();
    const p = thyristorParams(comp, vt);
    const on = (ctx.state.icState.get(comp.id)?.on ?? 0) >= 0.5;
    const vAk = vA - vK;
    // Principal current under the regime this solve was STAMPED with (the
    // entry latch), committed for updateCurrent: on a flip step the accepted
    // x solved the OLD branch, and evaluating the new branch's law on it
    // would publish a current no series neighbour carries. Same commit-then-
    // publish pattern as the relay's iCoil; capCurrents is snapshot-carried
    // but excluded from the error walk, exactly right for a derived readout.
    const iPrincipal = on ? (vAk - p.vtm) / p.ron : vAk * THYRISTOR_BLOCKING_G;
    ctx.state.capCurrents.set(comp.id, iPrincipal);
    const iGate = shockleyDiodeCurrent(vG - vK, p.isGate, N_DIODE, vt, p.vSatGate);
    const gateDriven = iGate >= p.igt && vG - vK >= p.vgt;
    let next = on;
    if (!on) {
      if (gateDriven && vAk > p.vtm) next = true;
    } else if (iPrincipal < p.ih) {
      if (!(gateDriven && iPrincipal > 0)) next = false;
    }
    ctx.state.icState.set(comp.id, { on: next ? 1 : 0 });
  },
  updateCurrent: (ctx, comp, _x) => {
    // Principal (anode) current, pin0 -> pin1 = a -> k, committed just above
    // from the stamped regime (commitState runs before this pass).
    ctx.setElementCurrent(comp.id, ctx.state.capCurrents.get(comp.id) ?? 0);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The committed regime selects the same piecewise-linear branch the
    // transient stamp uses: blocking = the leak, conducting = 1/ron (the
    // Norton vtm offset differentiates away). Plus the gate junction's
    // guarded Shockley slope at the OP with the stamp's GMIN floor.
    if (comp.pins.length < 3) return;
    const aNode = ctx.pinNode(comp.id, "a");
    const kNode = ctx.pinNode(comp.id, "k");
    const gNode = ctx.pinNode(comp.id, "g");
    const vt = ctx.junctionVt();
    const p = thyristorParams(comp, vt);
    const on = (ctx.state.icState.get(comp.id)?.on ?? 0) >= 0.5;
    stampAcAdmittance(
      ac,
      aNode,
      kNode,
      on ? 1 / p.ron : THYRISTOR_BLOCKING_G,
      0,
    );
    const gGate = shockleyConductanceAtOp(
      ctx.opVoltage(gNode) - ctx.opVoltage(kNode),
      p.isGate,
      N_DIODE,
      vt,
      p.vSatGate,
    );
    stampAcAdmittance(ac, gNode, kNode, gGate + AC_GMIN, 0);
  },
};

export const triacModel: DeviceModel = {
  kinds: ["triac"],
  stamp: (ctx, comp, xGuess, _h) => {
    // Pins mt1, mt2, g; gate referenced to mt1 (datasheet convention). Same
    // committed-regime pattern as the SCR made bidirectional: icState holds
    // the on latch AND the conduction polarity pol (+1 = current flows
    // mt1 -> mt2), frozen at trigger time. Conducting law in the mt1 -> mt2
    // sense: i = (v12 - pol*vtm)/ron with v12 = v(mt1) - v(mt2), so the vtm
    // junction drop always opposes the latched conduction direction. The
    // gate is a bidirectional junction (anti-parallel Shockley pair to
    // mt1): real triacs trigger on either gate polarity.
    if (comp.pins.length < 3) return;
    const mt1 = ctx.pinNode(comp.id, "mt1");
    const mt2 = ctx.pinNode(comp.id, "mt2");
    const gNode = ctx.pinNode(comp.id, "g");
    const vt = ctx.junctionVt();
    const p = thyristorParams(comp, vt);
    const vGate = ctx.vAt(xGuess, gNode) - ctx.vAt(xGuess, mt1);
    stampDiodeShockley(ctx.mna, gNode, mt1, vGate, p.isGate, N_DIODE, vt, p.vSatGate);
    stampDiodeShockley(ctx.mna, mt1, gNode, -vGate, p.isGate, N_DIODE, vt, p.vSatGate);
    const st = ctx.state.icState.get(comp.id);
    const on = (st?.on ?? 0) >= 0.5;
    if (on) {
      const pol = (st?.pol ?? 1) >= 0 ? 1 : -1;
      stampResistor(ctx.mna, mt1, mt2, p.ron);
      stampCurrentSource(ctx.mna, mt1, mt2, (pol * p.vtm) / p.ron);
    } else {
      stampResistor(ctx.mna, mt1, mt2, 1 / THYRISTOR_BLOCKING_G);
    }
  },
  commitState: (ctx, comp, x, _h) => {
    // Post-solve trigger on committed |gate current| (either polarity —
    // the anti-parallel pair's net current) with the |vgt| threshold AND
    // |v12| above the on-state drop vtm — the SCR's forward-bias rule made
    // bidirectional; without it a driven gate latches an unbiased MT pair
    // and the pol*vtm/ron Norton source erects a phantom +-vtm rail across
    // it. Conduction polarity latches from the principal voltage's sign at
    // the trigger instant. Dropout mirrors the SCR: conduction-direction
    // current below ih drops the latch unless the gate still holds trigger
    // drive with forward (pol-sense) current flowing; a polarity reversal
    // (pol*i <= 0) always commutates, so a still-driven gate re-fires the
    // opposite polarity from the blocking state once |v12| clears vtm.
    if (comp.pins.length < 3) return;
    const vMt1 = ctx.vAt(x, ctx.pinNode(comp.id, "mt1"));
    const vMt2 = ctx.vAt(x, ctx.pinNode(comp.id, "mt2"));
    const vG = ctx.vAt(x, ctx.pinNode(comp.id, "g"));
    const vt = ctx.junctionVt();
    const p = thyristorParams(comp, vt);
    const st = ctx.state.icState.get(comp.id);
    const on = (st?.on ?? 0) >= 0.5;
    let pol = (st?.pol ?? 1) >= 0 ? 1 : -1;
    let next = on;
    const v12 = vMt1 - vMt2;
    // Stamped-regime principal current for updateCurrent — see the SCR
    // commit for the flip-step rationale.
    const iPrincipal = on ? (v12 - pol * p.vtm) / p.ron : v12 * THYRISTOR_BLOCKING_G;
    ctx.state.capCurrents.set(comp.id, iPrincipal);
    const vGate = vG - vMt1;
    const iGate = shockleyDiodeCurrent(vGate, p.isGate, N_DIODE, vt, p.vSatGate)
      - shockleyDiodeCurrent(-vGate, p.isGate, N_DIODE, vt, p.vSatGate);
    const gateDriven = Math.abs(iGate) >= p.igt && Math.abs(vGate) >= p.vgt;
    if (!on) {
      if (gateDriven && Math.abs(v12) > p.vtm) {
        next = true;
        pol = v12 >= 0 ? 1 : -1;
      }
    } else if (pol * iPrincipal < p.ih) {
      if (!(gateDriven && pol * iPrincipal > 0)) next = false;
    }
    ctx.state.icState.set(comp.id, { on: next ? 1 : 0, pol });
  },
  updateCurrent: (ctx, comp, _x) => {
    // Principal current, pin0 -> pin1 = mt1 -> mt2, committed just above
    // from the stamped regime (commitState runs before this pass).
    ctx.setElementCurrent(comp.id, ctx.state.capCurrents.get(comp.id) ?? 0);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Committed regime selects leak vs 1/ron (polarity only shifts the
    // Norton offset, which differentiates away); the bidirectional gate
    // contributes both anti-parallel junction slopes at the OP.
    if (comp.pins.length < 3) return;
    const mt1 = ctx.pinNode(comp.id, "mt1");
    const mt2 = ctx.pinNode(comp.id, "mt2");
    const gNode = ctx.pinNode(comp.id, "g");
    const vt = ctx.junctionVt();
    const p = thyristorParams(comp, vt);
    const on = (ctx.state.icState.get(comp.id)?.on ?? 0) >= 0.5;
    stampAcAdmittance(ac, mt1, mt2, on ? 1 / p.ron : THYRISTOR_BLOCKING_G, 0);
    const vGate = ctx.opVoltage(gNode) - ctx.opVoltage(mt1);
    const gGate = shockleyConductanceAtOp(vGate, p.isGate, N_DIODE, vt, p.vSatGate)
      + shockleyConductanceAtOp(-vGate, p.isGate, N_DIODE, vt, p.vSatGate);
    stampAcAdmittance(ac, gNode, mt1, gGate + AC_GMIN, 0);
  },
};

// ── Optocoupler (LED + phototransistor) ─────────────────────────────────────

const OPTO_INTERNAL_NODES = ["b"] as const;

interface OptoParams {
  ctr: number;
  betaF: number;
  isLed: number;
  vSatLed: number;
}

function optoParams(
  ctx: Pick<DeviceContext, "junctionVf">,
  comp: DeviceComponent,
  vt: number,
): OptoParams {
  const vfLed = ctx.junctionVf(Number(comp.params.vf ?? 1.2));
  const iRated = Number(comp.params.iRated ?? 0.02);
  const vtN = N_LED * vt;
  return {
    ctr: Math.max(0, Number(comp.params.ctr ?? 1.0)),
    betaF: Math.max(1, Number(comp.params.betaF ?? 100)),
    isLed: shockleyIsFromVf(vfLed, iRated, N_LED, vt),
    vSatLed: raisedJunctionVSat(vfLed, vtN),
  };
}

export const optoNpnModel: DeviceModel = {
  kinds: ["opto_npn"],
  // The phototransistor base is a real circuit node inside the package with
  // no external pin — the canonical internal-node consumer.
  internalNodes: () => OPTO_INTERNAL_NODES,
  stamp: (ctx, comp, xGuess, _h) => {
    // Pins led_a/led_k (input LED) and c/e (phototransistor). Input side:
    // the standard Shockley LED stamp. Output side: a full Ebers-Moll NPN
    // on (internal b, c, e) whose base is driven by the photocurrent
    //
    //   Ip = (ctr/betaF) * iLed_committed
    //
    // injected collector -> base, the physical photodiode path (CB junction
    // photocurrent), which keeps output-side KCL conserved. Datasheet CTR
    // is the Ic/If ratio, so dividing by betaF makes the transistor's
    // forward gain reproduce ctr*iLed at the collector in the active
    // region, while saturation emerges naturally when the collector cannot
    // pull that current.
    //
    // ONE-STEP LAG, DOCUMENTED HONESTLY: iLed_committed is the LED current
    // of the PREVIOUS accepted solve (icState.i1). The drive is therefore a
    // constant per solve and the transient matrix carries NO cross-isolation
    // Jacobian entries — the galvanic isolation stays structurally visible.
    // A same-solve coupling was rejected as not "trivially safe": it would
    // add led-side columns to output-side Newton rows and change the sparse
    // pattern class of every opto circuit for a one-step latency nobody can
    // observe below the engine's step sizes. At a DC operating point the A3
    // settle loop iterates icState.i1 until its quantised signature rests,
    // within the bounded settle budget — external output-to-LED feedback
    // shares the documented op-amp accept-last semantics — which is why the
    // AC stamp may linearize the coupling as if instantaneous.
    if (comp.pins.length < 4) return;
    const ledA = ctx.pinNode(comp.id, "led_a");
    const ledK = ctx.pinNode(comp.id, "led_k");
    const cNode = ctx.pinNode(comp.id, "c");
    const eNode = ctx.pinNode(comp.id, "e");
    const vt = ctx.junctionVt();
    const p = optoParams(ctx, comp, vt);
    stampDiodeShockley(
      ctx.mna,
      ledA,
      ledK,
      ctx.vAt(xGuess, ledA) - ctx.vAt(xGuess, ledK),
      p.isLed,
      N_LED,
      vt,
      p.vSatLed,
    );

    const bNode = ctx.internalNode(comp.id, "b");
    if (bNode < 0) return;
    // Ebers-Moll with the bjtModel pnjlim pattern. This kind has no
    // per-load junction cache (the engine builds it for bjt_npn/bjt_pnp
    // only), so the cache expressions are computed inline — the documented
    // uncached fallback path of the BJT handler, one log per junction per
    // stamp.
    const is25 = Number(comp.params.Is ?? 1e-16);
    const isT = saturationCurrentAtTemperature(is25, ctx.ambientTempC(), 1);
    const vtN = N_DIODE * vt;
    const vCrit = vCritFor(isT, vtN);
    const vB = ctx.vAt(xGuess, bNode);
    const vC = ctx.vAt(xGuess, cNode);
    const vE = ctx.vAt(xGuess, eNode);
    const vBEraw = vB - vE;
    const vBCraw = vB - vC;
    const prev = ctx.bjtPrevJunction(comp.id) ?? { vBE: vBEraw, vBC: vBCraw };
    const vBElim = pnjlim(vBEraw, prev.vBE, vtN, vCrit);
    const vBClim = pnjlim(vBCraw, prev.vBC, vtN, vCrit);
    if (vBElim !== vBEraw || vBClim !== vBCraw) {
      ctx.markJunctionLimitedThisIteration();
    }
    ctx.setBjtPrevJunction(comp.id, { vBE: vBElim, vBC: vBClim });
    const vBeff = vE + vBElim;
    const vCeff = vBeff - vBClim;
    stampBJT(ctx.mna, bNode, cNode, eNode, vBeff, vCeff, vE, 1, isT, p.betaF, 1, 1, 1, vt, 0);

    // Photocurrent from the last ACCEPTED solve; clamped at zero because a
    // reverse-biased LED emits nothing.
    const iLedCommitted = Math.max(0, ctx.state.icState.get(comp.id)?.i1 ?? 0);
    const ip = (p.ctr / p.betaF) * iLedCommitted;
    stampCurrentSource(ctx.mna, bNode, cNode, ip);
  },
  commitState: (ctx, comp, x, _h) => {
    // Commit THIS solve's LED current for the next solve's photo drive (and
    // for the element-current readout below). Same guarded Shockley law the
    // stamp used, evaluated at the accepted voltages.
    if (comp.pins.length < 4) return;
    const vt = ctx.junctionVt();
    const p = optoParams(ctx, comp, vt);
    const vLed = ctx.vAt(x, ctx.pinNode(comp.id, "led_a"))
      - ctx.vAt(x, ctx.pinNode(comp.id, "led_k"));
    ctx.state.icState.set(comp.id, {
      i1: shockleyDiodeCurrent(vLed, p.isLed, N_LED, vt, p.vSatLed),
    });
  },
  updateCurrent: (ctx, comp, _x) => {
    // Element-current convention pin0 -> pin1 = led_a -> led_k: the input
    // LED current committed just above (commitState runs before this pass).
    ctx.setElementCurrent(comp.id, ctx.state.icState.get(comp.id)?.i1 ?? 0);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Input LED: guarded Shockley slope at the OP. Output: the exact
    // Ebers-Moll Jacobian on (internal b, c, e). Coupling: the transient
    // drive is Ip = (ctr/betaF)*iLed_prev, and at the held OP the one-step
    // lag sits at its fixed point (iLed_prev == iLed(v_op)), so the
    // physical coupling's derivative is
    //
    //   gp = (ctr/betaF) * gd_led,
    //
    // a transconductance from the LED branch voltage into the c->b photo
    // path. Through the transistor's forward gain that is ctr*gd_led into
    // the collector KCL — the linearization of the model the transient
    // stamps actually solve, not an ambition beyond it. Sign derivation
    // (KCL rows carry current LEAVING the node): Ip enters b, so
    // f_b -= gp*dvLed -> G[b][led_a] -= gp, G[b][led_k] += gp; Ip leaves c,
    // so f_c += gp*dvLed -> G[c][led_a] += gp, G[c][led_k] -= gp.
    if (comp.pins.length < 4) return;
    const ledA = ctx.pinNode(comp.id, "led_a");
    const ledK = ctx.pinNode(comp.id, "led_k");
    const cNode = ctx.pinNode(comp.id, "c");
    const eNode = ctx.pinNode(comp.id, "e");
    const vt = ctx.junctionVt();
    const p = optoParams(ctx, comp, vt);
    const vLed = ctx.opVoltage(ledA) - ctx.opVoltage(ledK);
    const gdLed = shockleyConductanceAtOp(vLed, p.isLed, N_LED, vt, p.vSatLed);
    stampAcAdmittance(ac, ledA, ledK, gdLed + AC_GMIN, 0);

    const bNode = ctx.internalNode(comp.id, "b");
    if (bNode < 0) return;
    const isT = saturationCurrentAtTemperature(
      Number(comp.params.Is ?? 1e-16),
      ctx.ambientTempC(),
      1,
    );
    const lin = bjtLinearization(
      ctx.opVoltage(bNode),
      ctx.opVoltage(cNode),
      ctx.opVoltage(eNode),
      1,
      isT,
      p.betaF,
      1,
      1,
      1,
      vt,
      0,
    );
    ac.addAc(bNode, bNode, lin.gbB, 0);
    ac.addAc(bNode, cNode, lin.gbC, 0);
    ac.addAc(bNode, eNode, lin.gbE, 0);
    ac.addAc(cNode, bNode, lin.gcB, 0);
    ac.addAc(cNode, cNode, lin.gcC, 0);
    ac.addAc(cNode, eNode, lin.gcE, 0);
    ac.addAc(eNode, bNode, lin.geB, 0);
    ac.addAc(eNode, cNode, lin.geC, 0);
    ac.addAc(eNode, eNode, lin.geE, 0);

    const gp = (p.ctr / p.betaF) * gdLed;
    ac.addAc(bNode, ledA, -gp, 0);
    ac.addAc(bNode, ledK, gp, 0);
    ac.addAc(cNode, ledA, gp, 0);
    ac.addAc(cNode, ledK, -gp, 0);
  },
};

// ── Quartz crystal ──────────────────────────────────────────────────────────

const CRYSTAL_INTERNAL_NODES = ["m"] as const;

interface CrystalParams {
  rs: number;
  ls: number;
  cs: number;
  /** Farads (the authored c0 param is in picofarads — UI-scale unit). */
  c0: number;
}

/**
 * Standard crystal equivalent-circuit derivation from the datasheet triple
 * (series-resonant frequency, quality factor, motional/series resistance):
 *
 *   Q  = ws*Ls/Rs  and  ws^2 = 1/(Ls*Cs),  ws = 2*pi*fSeries
 *   =>  Ls = Q*Rs/ws,   Cs = 1/(ws^2*Ls) = 1/(ws*Q*Rs)
 *
 * (1 MHz, Q = 50k, Rs = 50 gives Ls ~ 0.40 H, Cs ~ 64 fF — textbook AT-cut
 * magnitudes.) C0 is the physical shunt (holder + electrode) capacitance
 * across the pins; the parallel antiresonance lands at
 * fp = fs*sqrt(1 + Cs/C0), a fraction of a percent above fs.
 */
function crystalParams(comp: DeviceComponent): CrystalParams {
  const fSeries = Math.max(1e-3, Number(comp.params.fSeries ?? 1e6));
  const q = Math.max(1, Number(comp.params.q ?? 50_000));
  const rs = Math.max(1e-3, Number(comp.params.rs ?? 50));
  const omegaS = 2 * Math.PI * fSeries;
  const ls = (q * rs) / omegaS;
  return {
    rs,
    ls,
    cs: 1 / (omegaS * omegaS * ls),
    c0: Math.max(0, Number(comp.params.c0 ?? 5)) * 1e-12,
  };
}

export const crystalModel: DeviceModel = {
  kinds: ["crystal"],
  // One internal solver node "m" splits the motional branch into the two
  // primitives the engine already integrates: series (Rs + Cs) from pin a
  // to m (Rs rides in the capacitor companion's ESR slot), Ls from m to
  // pin b. C0 shunts the pins directly.
  internalNodes: () => CRYSTAL_INTERNAL_NODES,
  stamp: (ctx, comp, _xGuess, h) => {
    // Transient model composes the EXISTING capacitor/inductor companions
    // (BE or trap following the engine mode, exactly like the standalone
    // parts) — no new integration math, so A2's rollback/anchor rules hold
    // for the crystal by construction.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const aNode = ctx.pinNode(comp.id, pins[0].id);
    const bNode = ctx.pinNode(comp.id, pins[1].id);
    const mNode = ctx.internalNode(comp.id, "m");
    if (mNode < 0) return;
    const p = crystalParams(comp);
    if (ctx.dcSolveMode()) {
      // A crystal is an open at DC: motional Cs and shunt C0 both block
      // (the capacitor dc rule). Only the inductor keeps its dc short so
      // the internal node stays tied to pin b instead of floating on the
      // 1e-12 node shunt alone.
      stampResistor(ctx.mna, mNode, bNode, 1e-6);
      return;
    }
    const keys = crystalKeys(comp.id);
    const vcsPrev = ctx.state.caps.get(keys.cs) ?? 0;
    const ilsPrev = ctx.state.inds.get(keys.ls) ?? 0;
    const vc0Prev = ctx.state.caps.get(keys.c0) ?? 0;
    if (ctx.useTrapThisSolve()) {
      stampCapacitorTrap(
        ctx.mna,
        aNode,
        mNode,
        p.cs,
        h,
        vcsPrev,
        ctx.state.capsI.get(keys.cs) ?? 0,
        p.rs,
      );
      stampInductorTrap(
        ctx.mna,
        mNode,
        bNode,
        p.ls,
        h,
        ilsPrev,
        ctx.state.indsV.get(keys.ls) ?? 0,
        0,
      );
      if (p.c0 > 0) {
        stampCapacitorTrap(
          ctx.mna,
          aNode,
          bNode,
          p.c0,
          h,
          vc0Prev,
          ctx.state.capsI.get(keys.c0) ?? 0,
          0,
        );
      }
      return;
    }
    stampCapacitor(ctx.mna, aNode, mNode, p.cs, h, vcsPrev, p.rs);
    stampInductor(ctx.mna, mNode, bNode, p.ls, h, ilsPrev, 0);
    if (p.c0 > 0) {
      stampCapacitor(ctx.mna, aNode, bNode, p.c0, h, vc0Prev, 0);
    }
  },
  commitState: (ctx, comp, x, h) => {
    // Per-branch commits mirror capacitorModel/inductorModel exactly, each
    // against its own composite key, so snapshots/rollback/error-walk treat
    // the crystal's three memories like three ordinary parts.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const aNode = ctx.pinNode(comp.id, pins[0].id);
    const bNode = ctx.pinNode(comp.id, pins[1].id);
    const mNode = ctx.internalNode(comp.id, "m");
    if (mNode < 0) return;
    const p = crystalParams(comp);
    const keys = crystalKeys(comp.id);
    const va = ctx.vAt(x, aNode);
    const vb = ctx.vAt(x, bNode);
    const vm = ctx.vAt(x, mNode);
    if (ctx.dcSolveMode()) {
      // DC normalisation (see the module header): the open series branch
      // carries no current, so the motional cap holds its solved branch
      // voltage with zero Rs drop; the inductor current maps 1:1 onto the
      // 1e-6 dc short's terminal voltage (both are solver-noise scale, but
      // committing the stamp-consistent value keeps the OP self-describing);
      // C0 holds the pin-to-pin voltage.
      ctx.state.caps.set(keys.cs, va - vm);
      ctx.state.capsI.set(keys.cs, 0);
      ctx.state.capCurrents.set(keys.cs, 0);
      ctx.state.inds.set(keys.ls, (vm - vb) / 1e-6);
      ctx.state.indsV.set(keys.ls, vm - vb);
      ctx.state.caps.set(keys.c0, va - vb);
      ctx.state.capsI.set(keys.c0, 0);
      ctx.state.capCurrents.set(keys.c0, 0);
      return;
    }
    const useTrap = ctx.useTrapThisSolve();
    const maintainTrapHistory = ctx.integrationMethod() === "trap";

    const vAm = va - vm;
    const vcsPrev = ctx.state.caps.get(keys.cs) ?? 0;
    const iCsPrev = useTrap ? ctx.state.capsI.get(keys.cs) ?? 0 : 0;
    const iCs = useTrap
      ? capacitorSeriesCurrentTrap(vAm, p.cs, h, vcsPrev, iCsPrev, p.rs)
      : capacitorSeriesCurrent(vAm, p.cs, h, vcsPrev, p.rs);
    ctx.state.caps.set(
      keys.cs,
      useTrap
        ? capacitorInternalVoltageTrap(vcsPrev, iCsPrev, iCs, p.cs, h)
        : capacitorInternalVoltage(vcsPrev, iCs, p.cs, h),
    );
    if (maintainTrapHistory) ctx.state.capsI.set(keys.cs, iCs);
    ctx.state.capCurrents.set(keys.cs, iCs);

    const vMb = vm - vb;
    const ilsPrev = ctx.state.inds.get(keys.ls) ?? 0;
    const vLsPrev = useTrap ? ctx.state.indsV.get(keys.ls) ?? 0 : 0;
    ctx.state.inds.set(
      keys.ls,
      useTrap
        ? inductorWindingCurrentTrap(vMb, p.ls, h, ilsPrev, vLsPrev, 0)
        : inductorWindingCurrent(vMb, p.ls, h, ilsPrev, 0),
    );
    if (maintainTrapHistory) ctx.state.indsV.set(keys.ls, vMb);

    if (p.c0 > 0) {
      const vAb = va - vb;
      const vc0Prev = ctx.state.caps.get(keys.c0) ?? 0;
      const iC0Prev = useTrap ? ctx.state.capsI.get(keys.c0) ?? 0 : 0;
      const iC0 = useTrap
        ? capacitorSeriesCurrentTrap(vAb, p.c0, h, vc0Prev, iC0Prev, 0)
        : capacitorSeriesCurrent(vAb, p.c0, h, vc0Prev, 0);
      ctx.state.caps.set(
        keys.c0,
        useTrap
          ? capacitorInternalVoltageTrap(vc0Prev, iC0Prev, iC0, p.c0, h)
          : capacitorInternalVoltage(vc0Prev, iC0, p.c0, h),
      );
      if (maintainTrapHistory) ctx.state.capsI.set(keys.c0, iC0);
      ctx.state.capCurrents.set(keys.c0, iC0);
    } else {
      ctx.state.capCurrents.set(keys.c0, 0);
    }
  },
  updateCurrent: (ctx, comp, _x) => {
    // Pin-a terminal current = motional series branch + C0 shunt, both
    // committed just above.
    const keys = crystalKeys(comp.id);
    ctx.setElementCurrent(
      comp.id,
      (ctx.state.capCurrents.get(keys.cs) ?? 0)
        + (ctx.state.capCurrents.get(keys.c0) ?? 0),
    );
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Exact complex composition of the same physical branches:
    //
    //   Y_motional = 1/(Rs + j*(w*Ls - 1/(w*Cs)))
    //              = (Rs - j*x)/(Rs^2 + x^2),  x = w*Ls - 1/(w*Cs)
    //   Y_shunt    = j*w*C0
    //
    // Series elimination of the internal node is exact in the phasor
    // domain, so "m" is deliberately left unstamped here — the AC driver's
    // node shunt keeps its row regular and it reads ~0. Below fs the
    // motional branch is capacitive (x < 0), at fs purely Rs (the
    // conductance peak), above fs inductive until the C0 antiresonance at
    // fs*sqrt(1 + Cs/C0) — the series/parallel split the smoke fixtures
    // verify.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const aNode = ctx.pinNode(comp.id, pins[0].id);
    const bNode = ctx.pinNode(comp.id, pins[1].id);
    const p = crystalParams(comp);
    const x = omega * p.ls - 1 / (omega * p.cs);
    const den = p.rs * p.rs + x * x;
    stampAcAdmittance(ac, aNode, bNode, p.rs / den, -x / den);
    if (p.c0 > 0) {
      stampAcAdmittance(ac, aNode, bNode, 0, omega * p.c0);
    }
  },
};

// ── Analog switch (transmission gate) ───────────────────────────────────────

// TTL-compatible control thresholds with a hysteresis hold band between
// them (the relay vPull/vDrop pattern): a slow control ramp cannot chatter
// the committed state, and a floating ctrl pin — defined at ~0 V by the
// node shunt — reads off. The part models a self-contained gate (no supply
// pins), so the control level is ground-referenced by construction.
const ANALOG_SWITCH_VIH = 2.0;
const ANALOG_SWITCH_VIL = 0.8;

function analogSwitchResistances(comp: DeviceComponent): { ron: number; roff: number } {
  const roff = Number(comp.params.roff ?? 1e9);
  return {
    ron: Math.max(1e-3, Number(comp.params.ron ?? 100)),
    // Zero/negative/NaN roff falls back to the default rather than a floor:
    // flooring like ron would hand the OFF regime a 1 ohm channel — an "off"
    // switch conducting better than the on-state, silently inverting the
    // control sense. Isolation is the only safe failure direction here.
    roff: roff > 0 ? Math.max(1, roff) : 1e9,
  };
}

export const analogSwitchModel: DeviceModel = {
  kinds: ["analog_switch"],
  stamp: (ctx, comp, _xGuess, _h) => {
    // Committed control level selects ron/roff between a and b (the relay
    // contact pattern — never re-decided from a Newton iterate). The ctrl
    // pin is a pure sense input: no stamp, mirroring a CMOS gate input.
    // Purely resistive in both regimes, so the same stamp serves dc-mode
    // OP solves.
    if (comp.pins.length < 3) return;
    const on = (ctx.state.icState.get(comp.id)?.on ?? 0) >= 0.5;
    const { ron, roff } = analogSwitchResistances(comp);
    stampResistor(
      ctx.mna,
      ctx.pinNode(comp.id, "a"),
      ctx.pinNode(comp.id, "b"),
      on ? ron : roff,
    );
  },
  commitState: (ctx, comp, x, _h) => {
    if (comp.pins.length < 3) return;
    const prev = (ctx.state.icState.get(comp.id)?.on ?? 0) >= 0.5;
    // Channel current under the resistance this solve was STAMPED with (the
    // entry latch), committed for updateCurrent: publishing the just-decided
    // regime's law on a flip step would report a current the solved circuit
    // never carried (the SCR commit documents the shared pattern).
    const vAb = ctx.vAt(x, ctx.pinNode(comp.id, "a"))
      - ctx.vAt(x, ctx.pinNode(comp.id, "b"));
    const { ron, roff } = analogSwitchResistances(comp);
    ctx.state.capCurrents.set(comp.id, vAb / (prev ? ron : roff));
    const vCtrl = ctx.vAt(x, ctx.pinNode(comp.id, "ctrl"));
    const next = vCtrl >= ANALOG_SWITCH_VIH
      ? true
      : vCtrl <= ANALOG_SWITCH_VIL
        ? false
        : prev;
    ctx.state.icState.set(comp.id, { on: next ? 1 : 0 });
  },
  updateCurrent: (ctx, comp, _x) => {
    // Channel current, pin0 -> pin1 = a -> b, committed just above from the
    // stamped regime (commitState runs before this pass).
    ctx.setElementCurrent(comp.id, ctx.state.capCurrents.get(comp.id) ?? 0);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The committed regime's conductance, exactly the transient stamp's.
    if (comp.pins.length < 3) return;
    const on = (ctx.state.icState.get(comp.id)?.on ?? 0) >= 0.5;
    const { ron, roff } = analogSwitchResistances(comp);
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, "a"),
      ctx.pinNode(comp.id, "b"),
      1 / (on ? ron : roff),
      0,
    );
  },
};
