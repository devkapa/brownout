/**
 * PASSIVES device cohort (Wave A4 phase 1): resistor, resistor_array,
 * potentiometer, trimmer, capacitor, inductor, ferrite_bead, ldr,
 * thermistor, switch, push_button, spdt_switch, push_dpdt, dip_switch,
 * fuse, ptc_fuse.
 *
 * Every handler body below is the engine's old switch-case / if-block body
 * MOVED VERBATIM, with identifier access adapted through DeviceContext
 * (this._pinNode -> ctx.pinNode, this.state.caps -> ctx.state.caps,
 * out[comp.id] = v -> ctx.setElementCurrent(comp.id, v), and so on). Do not
 * simplify, reorder mna.add calls, or rewrite algebra here: float
 * accumulation order in the matrix is semantics, and the bitwise oracle
 * suite pins these exact trajectories.
 *
 * Kinds grouped into one model correspond exactly to switch cases that
 * shared a body in the engine (potentiometer/trimmer everywhere;
 * switch/push_button, whose separate stamp cases were textually identical
 * code and whose current case was already shared).
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
  capacitorInternalVoltage,
  capacitorInternalVoltageTrap,
  capacitorSeriesCurrent,
  capacitorSeriesCurrentTrap,
  inductorWindingCurrent,
  inductorWindingCurrentTrap,
  parallelLossCurrent,
  stampCapacitor,
  stampCapacitorTrap,
  stampInductor,
  stampInductorTrap,
  stampResistor,
} from "../elements.js";
import {
  FUSE_I2T_THRESHOLD,
  ldrResistance,
  parallelLossResistance,
  RESISTOR_DAMAGE_THRESHOLD,
  seriesLossResistance,
  thermistorResistance,
} from "../sim-engine.js";

/**
 * Shared resistive-overload damage integrator for resistor, potentiometer,
 * and trimmer — one body in the engine's _updateFailureStates, kept as one
 * body here (the internal kind branch is part of the moved code).
 */
function commitResistiveOverloadStress(
  ctx: DeviceContext,
  comp: DeviceComponent,
  x: Float64Array,
  h: number,
): void {
  const specs = ctx.electricalSpecs(comp);
  const pMax = specs?.p_max;
  let power = 0;
  let localLimit = pMax;

  if (comp.kind === "resistor") {
    const resistance = Number(comp.params.resistance ?? 0);
    const current = Math.abs(ctx.elementCurrent(comp.id) ?? 0);
    power = resistance > 0 ? current * current * resistance : 0;
  } else {
    // A loaded wiper carries different current in each track segment.
    // Pot power ratings apply to the full track, so the short segment's
    // safe share scales with its resistance/length. Using terminal-to-
    // terminal voltage here would miss a common rheostat overload by
    // orders of magnitude.
    const rTotal = Math.max(1, Number(comp.params.rTotal ?? comp.params.resistance ?? 10_000));
    const position = Math.max(0, Math.min(1, Number(comp.params.position ?? 0.5)));
    const effectivePosition = String(comp.params.taper ?? "linear") === "log"
      ? Math.pow(10, 2 * (position - 1))
      : position;
    const rCw = Math.max(1, rTotal * effectivePosition);
    const rCcw = Math.max(1, rTotal * (1 - effectivePosition));
    const vCw = ctx.vAt(x, ctx.pinNode(comp.id, "cw"));
    const vWiper = ctx.vAt(x, ctx.pinNode(comp.id, "wiper"));
    const vCcw = ctx.vAt(x, ctx.pinNode(comp.id, "ccw"));
    const pCw = (vCw - vWiper) ** 2 / rCw;
    const pCcw = (vWiper - vCcw) ** 2 / rCcw;
    const trackResistance = rCw + rCcw;
    const cwLimit = pMax != null ? pMax * rCw / trackResistance : undefined;
    const ccwLimit = pMax != null ? pMax * rCcw / trackResistance : undefined;
    const cwRatio = cwLimit != null && cwLimit > 0 ? pCw / cwLimit : 0;
    const ccwRatio = ccwLimit != null && ccwLimit > 0 ? pCcw / ccwLimit : 0;
    if (cwRatio >= ccwRatio) {
      power = pCw;
      localLimit = cwLimit;
    } else {
      power = pCcw;
      localLimit = ccwLimit;
    }
  }

  const powerRatio = localLimit != null && localLimit > 0 ? power / localLimit : 0;
  ctx.recordAccumulatedStress(
    "resistor_overload",
    comp.id,
    "",
    Math.max(0, powerRatio - 1),
    1,
    h,
    RESISTOR_DAMAGE_THRESHOLD,
    () => ({
      componentId: comp.id,
      kind: "resistor_overload",
      since: ctx.simTime() + h,
      value: power,
      limit: localLimit,
      message:
        comp.kind === "resistor"
          ? `${comp.id} dissipated ${power.toFixed(3)} W through a ${localLimit!.toFixed(3)} W rating long enough to fail open.`
          : `${comp.id} dissipated ${power.toFixed(3)} W in one wiper track segment, above that segment's ${localLimit!.toFixed(3)} W share, long enough to fail open.`,
    }),
  );
}

export const resistorModel: DeviceModel = {
  kinds: ["resistor"],
  staticStamp: true,
  staticSignature: (ctx, comp, out) => {
    const p = comp.params;
    out.push(
      String(p.resistance ?? 1000),
      String(ctx.hasFailure(comp.id, "resistor_overload")),
    );
  },
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    if (ctx.hasFailure(comp.id, "resistor_overload")) return;
    const R = Math.max(1e-3, Number(comp.params.resistance ?? 1000));
    stampResistor(
      ctx.mna,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      R,
    );
  },
  updateCurrent: (ctx, comp, x) => {
    // Engine _updateElementI preamble, copied per-handler (same pure reads).
    const pins = comp.pins;
    const a = ctx.pinNode(comp.id, pins[0].id);
    const b = ctx.pinNode(comp.id, pins[1].id);
    const va = ctx.vAt(x, a);
    const vb = ctx.vAt(x, b);
    const vd = va - vb;
    if (ctx.hasFailure(comp.id, "resistor_overload")) {
      ctx.setElementCurrent(comp.id, 0);
      return;
    }
    const R = Math.max(1e-3, Number(comp.params.resistance ?? 1000));
    ctx.setElementCurrent(comp.id, vd / R);
  },
  updateFailures: (ctx, comp, x, h) => {
    commitResistiveOverloadStress(ctx, comp, x, h);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Small-signal resistor = the transient conductance at the OP parameter
    // values; a failed-open part stays open, mirroring the stamp's gate.
    const pins = comp.pins;
    if (pins.length < 2) return;
    if (ctx.hasFailure(comp.id, "resistor_overload")) return;
    const R = Math.max(1e-3, Number(comp.params.resistance ?? 1000));
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      1 / R,
      0,
    );
  },
};

export const capacitorModel: DeviceModel = {
  kinds: ["capacitor"],
  stamp: (ctx, comp, _xGuess, h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    const C = Math.max(1e-15, Number(comp.params.capacitance ?? 1e-6));
    const vcPrev = ctx.state.caps.get(comp.id) ?? 0;
    const esr = seriesLossResistance(ctx.modelParam(comp, "esr", 0));
    const leakageResistance = parallelLossResistance(ctx.modelParam(comp, "leakageResistance", 0));
    if (ctx.dcSolveMode()) {
      // A capacitor is open at DC: skipping the companion entirely lets
      // the resistive topology set the node instead of the held initial
      // condition. Only a declared leakage shunt still conducts — it is
      // a physical resistor across the terminals, not integration state.
      if (Number.isFinite(leakageResistance) && leakageResistance > 0) {
        stampResistor(
          ctx.mna,
          ctx.pinNode(comp.id, pins[0].id),
          ctx.pinNode(comp.id, pins[1].id),
          leakageResistance,
        );
      }
      return;
    }
    if (ctx.useTrapThisSolve()) {
      stampCapacitorTrap(
        ctx.mna,
        ctx.pinNode(comp.id, pins[0].id),
        ctx.pinNode(comp.id, pins[1].id),
        C,
        h,
        vcPrev,
        ctx.state.capsI.get(comp.id) ?? 0,
        esr,
        leakageResistance,
      );
      return;
    }
    stampCapacitor(
      ctx.mna,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      C,
      h,
      vcPrev,
      esr,
      leakageResistance,
    );
  },
  commitState: (ctx, comp, x, h) => {
    if (!(comp.pins.length >= 2)) return;
    // Must match what the stamp used for THIS solve: the discontinuity
    // anchor only mutates outside the solve, so the choice is stable across
    // the Newton iterations and this post-solve advance.
    const useTrap = ctx.useTrapThisSolve();
    // History upkeep is gated on the MODE, not on useTrap: a forced-BE
    // anchor step must still refresh capsI/indsV so the next trap stamp
    // reads post-discontinuity history. BE mode skips the map traffic
    // entirely — the values are never read there, and switching into trap
    // mode re-anchors through a forced BE step before any trap stamp could
    // see the stale entries.
    const maintainTrapHistory = ctx.integrationMethod() === "trap";
    const a = ctx.pinNode(comp.id, comp.pins[0].id);
    const b = ctx.pinNode(comp.id, comp.pins[1].id);
    const terminalVoltage = ctx.vAt(x, a) - ctx.vAt(x, b);
    const C = Math.max(1e-15, Number(comp.params.capacitance ?? 1e-6));
    const vcPrev = ctx.state.caps.get(comp.id) ?? 0;
    const iPrevTrap = useTrap ? ctx.state.capsI.get(comp.id) ?? 0 : 0;
    const esr = seriesLossResistance(ctx.modelParam(comp, "esr", 0));
    const leakageResistance = parallelLossResistance(ctx.modelParam(comp, "leakageResistance", 0));
    const seriesCurrent = useTrap
      ? capacitorSeriesCurrentTrap(terminalVoltage, C, h, vcPrev, iPrevTrap, esr)
      : capacitorSeriesCurrent(terminalVoltage, C, h, vcPrev, esr);
    ctx.state.caps.set(
      comp.id,
      useTrap
        ? capacitorInternalVoltageTrap(vcPrev, iPrevTrap, seriesCurrent, C, h)
        : capacitorInternalVoltage(vcPrev, seriesCurrent, C, h),
    );
    if (maintainTrapHistory) ctx.state.capsI.set(comp.id, seriesCurrent);
    ctx.state.capCurrents.set(
      comp.id,
      seriesCurrent + parallelLossCurrent(terminalVoltage, leakageResistance),
    );
  },
  updateCurrent: (ctx, comp, _x) => {
    ctx.setElementCurrent(comp.id, ctx.state.capCurrents.get(comp.id) ?? 0);
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Full complex admittance of the physical branch the transient companion
    // integrates: series ESR + ideal C, with the declared leakage resistor in
    // parallel across the terminals.
    //
    //   Z_series = esr + 1/(j*omega*C) = esr - j*xc,  xc = 1/(omega*C)
    //   Y_series = 1/Z = (esr + j*xc) / (esr^2 + xc^2)
    //
    // (multiply numerator and denominator by the conjugate). At esr = 0 this
    // reduces to Y = j*omega*C exactly; the leakage shunt adds a pure real
    // conductance, matching its transient role as a plain terminal resistor.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const a = ctx.pinNode(comp.id, pins[0].id);
    const b = ctx.pinNode(comp.id, pins[1].id);
    const C = Math.max(1e-15, Number(comp.params.capacitance ?? 1e-6));
    const esr = seriesLossResistance(ctx.modelParam(comp, "esr", 0));
    const leakageResistance = parallelLossResistance(ctx.modelParam(comp, "leakageResistance", 0));
    const xc = 1 / (omega * C);
    const den = esr * esr + xc * xc;
    stampAcAdmittance(ac, a, b, esr / den, xc / den);
    if (Number.isFinite(leakageResistance) && leakageResistance > 0) {
      stampAcAdmittance(ac, a, b, 1 / leakageResistance, 0);
    }
  },
};

export const inductorModel: DeviceModel = {
  kinds: ["inductor"],
  stamp: (ctx, comp, _xGuess, h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    const L = Math.max(1e-12, Number(comp.params.inductance ?? 1e-3));
    const iPrev = ctx.state.inds.get(comp.id) ?? 0;
    const dcr = seriesLossResistance(ctx.modelParam(comp, "dcr", 0));
    const coreLossResistance = parallelLossResistance(ctx.modelParam(comp, "coreLossResistance", 0));
    if (ctx.dcSolveMode()) {
      // An inductor is a short at DC bounded by its winding resistance;
      // the 1e-6 ohm floor keeps ideal (dcr = 0) coils well-posed while
      // staying invisible at circuit scales. The declared core-loss
      // shunt is omitted: it models AC flux loss and merely parallels
      // this near-short, so it cannot shape the DC point.
      stampResistor(
        ctx.mna,
        ctx.pinNode(comp.id, pins[0].id),
        ctx.pinNode(comp.id, pins[1].id),
        Math.max(dcr, 1e-6),
      );
      return;
    }
    if (ctx.useTrapThisSolve()) {
      stampInductorTrap(
        ctx.mna,
        ctx.pinNode(comp.id, pins[0].id),
        ctx.pinNode(comp.id, pins[1].id),
        L,
        h,
        iPrev,
        ctx.state.indsV.get(comp.id) ?? 0,
        dcr,
        coreLossResistance,
      );
      return;
    }
    stampInductor(
      ctx.mna,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      L,
      h,
      iPrev,
      dcr,
      coreLossResistance,
    );
  },
  commitState: (ctx, comp, x, h) => {
    if (!(comp.pins.length >= 2)) return;
    // Same trap gating rationale as capacitorModel.commitState above.
    const useTrap = ctx.useTrapThisSolve();
    const maintainTrapHistory = ctx.integrationMethod() === "trap";
    const a = ctx.pinNode(comp.id, comp.pins[0].id);
    const b = ctx.pinNode(comp.id, comp.pins[1].id);
    const terminalVoltage = ctx.vAt(x, a) - ctx.vAt(x, b);
    const L = Math.max(1e-12, Number(comp.params.inductance ?? 1e-3));
    const iPrev = ctx.state.inds.get(comp.id) ?? 0;
    const vPrevTrap = useTrap ? ctx.state.indsV.get(comp.id) ?? 0 : 0;
    const dcr = seriesLossResistance(ctx.modelParam(comp, "dcr", 0));
    ctx.state.inds.set(
      comp.id,
      useTrap
        ? inductorWindingCurrentTrap(terminalVoltage, L, h, iPrev, vPrevTrap, dcr)
        : inductorWindingCurrent(terminalVoltage, L, h, iPrev, dcr),
    );
    // Winding terminal voltage doubles as next step's trap history.
    if (maintainTrapHistory) ctx.state.indsV.set(comp.id, terminalVoltage);
  },
  updateCurrent: (ctx, comp, x) => {
    // Engine _updateElementI preamble, copied per-handler (same pure reads).
    const pins = comp.pins;
    const a = ctx.pinNode(comp.id, pins[0].id);
    const b = ctx.pinNode(comp.id, pins[1].id);
    const va = ctx.vAt(x, a);
    const vb = ctx.vAt(x, b);
    const vd = va - vb;
    const coreLossResistance = parallelLossResistance(ctx.modelParam(comp, "coreLossResistance", 0));
    ctx.setElementCurrent(
      comp.id,
      (ctx.state.inds.get(comp.id) ?? 0) + parallelLossCurrent(vd, coreLossResistance),
    );
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Winding branch admittance with the declared core-loss shunt:
    //
    //   Y_winding = 1/(dcr + j*omega*L)
    //             = (dcr - j*omega*L) / (dcr^2 + (omega*L)^2)
    //
    // (conjugate expansion; exact -j/(omega*L) for an ideal dcr = 0 coil).
    // The core-loss resistor stays a plain parallel conductance, exactly as
    // the transient stamp treats it.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const a = ctx.pinNode(comp.id, pins[0].id);
    const b = ctx.pinNode(comp.id, pins[1].id);
    const L = Math.max(1e-12, Number(comp.params.inductance ?? 1e-3));
    const dcr = seriesLossResistance(ctx.modelParam(comp, "dcr", 0));
    const coreLossResistance = parallelLossResistance(ctx.modelParam(comp, "coreLossResistance", 0));
    const xl = omega * L;
    const den = dcr * dcr + xl * xl;
    stampAcAdmittance(ac, a, b, dcr / den, -xl / den);
    if (Number.isFinite(coreLossResistance) && coreLossResistance > 0) {
      stampAcAdmittance(ac, a, b, 1 / coreLossResistance, 0);
    }
  },
};

export const switchModel: DeviceModel = {
  kinds: ["switch", "push_button"],
  staticStamp: true,
  staticSignature: (_ctx, comp, out) => {
    const p = comp.params;
    out.push(String(Boolean(p.closed)));
  },
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    // For the 4-pin tact switch (push_button) the two legs on each side are
    // one conductor, so a≡a2 and b≡b2 are already merged into single nets by
    // buildNets — no stamp needed for those. The momentary contact bridges
    // side A↔B (i.e. the a/a2 net to the b/b2 net) only while pressed.
    // pins[0]/pins[1] are a and b, so this also covers pre-4-pin (a,b only)
    // saves — and is exactly the plain SPST "switch" stamp.
    if (Boolean(comp.params.closed)) {
      stampResistor(
        ctx.mna,
        ctx.pinNode(comp.id, pins[0].id),
        ctx.pinNode(comp.id, pins[1].id),
        0.001, // 1 mΩ closed resistance
      );
    }
    // open: no stamp = open circuit
  },
  updateCurrent: (ctx, comp, x) => {
    // Engine _updateElementI preamble, copied per-handler (same pure reads).
    const pins = comp.pins;
    const a = ctx.pinNode(comp.id, pins[0].id);
    const b = ctx.pinNode(comp.id, pins[1].id);
    const va = ctx.vAt(x, a);
    const vb = ctx.vAt(x, b);
    const vd = va - vb;
    const closed = Boolean(comp.params.closed);
    ctx.setElementCurrent(comp.id, closed ? vd / 0.001 : 0);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Closed contact = the transient 1 mOhm (1e3 S); open contact stamps
    // nothing, exactly like the transient stamp.
    const pins = comp.pins;
    if (pins.length < 2) return;
    if (Boolean(comp.params.closed)) {
      stampAcAdmittance(
        ac,
        ctx.pinNode(comp.id, pins[0].id),
        ctx.pinNode(comp.id, pins[1].id),
        1 / 0.001,
        0,
      );
    }
  },
};

export const spdtSwitchModel: DeviceModel = {
  kinds: ["spdt_switch"],
  staticStamp: true,
  staticSignature: (_ctx, comp, out) => {
    const p = comp.params;
    out.push(String(p.position ?? 0));
  },
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    if (pins.length < 3) return;
    const pos = Number(comp.params.position ?? 0);
    const comNode = ctx.pinNode(comp.id, "com");
    const activePin = pos === 0 ? "b" : "c";
    stampResistor(ctx.mna, comNode, ctx.pinNode(comp.id, activePin), 0.001);
  },
  updateCurrent: (ctx, comp, x) => {
    // Engine _updateElementI preamble, copied per-handler (same pure reads).
    const pins = comp.pins;
    const a = ctx.pinNode(comp.id, pins[0].id);
    const b = ctx.pinNode(comp.id, pins[1].id);
    const va = ctx.vAt(x, a);
    const vb = ctx.vAt(x, b);
    const vd = va - vb;
    // Report current from com pin perspective.
    ctx.setElementCurrent(comp.id, vd / 0.001);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Only the selected throw conducts (break-before-make), same 1 mOhm.
    const pins = comp.pins;
    if (pins.length < 3) return;
    const pos = Number(comp.params.position ?? 0);
    const activePin = pos === 0 ? "b" : "c";
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, "com"),
      ctx.pinNode(comp.id, activePin),
      1 / 0.001,
      0,
    );
  },
};

export const pushDpdtModel: DeviceModel = {
  kinds: ["push_dpdt"],
  staticStamp: true,
  staticSignature: (_ctx, comp, out) => {
    const p = comp.params;
    out.push(String(p.position ?? 0));
  },
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    if (pins.length < 6) return;
    // DPDT latching push switch: two ganged poles. Each common (c1/c2)
    // connects to its left throw (l1/l2) at position 0, its right throw
    // (r1/r2) at position 1. Break-before-make: only the active side of
    // each pole is stamped, so the two throws are never bridged.
    const right = Number(comp.params.position ?? 0) !== 0;
    const dp = (id: string) => ctx.pinNode(comp.id, id);
    stampResistor(ctx.mna, dp("c1"), dp(right ? "r1" : "l1"), 0.001);
    stampResistor(ctx.mna, dp("c2"), dp(right ? "r2" : "l2"), 0.001);
  },
  updateCurrent: (ctx, comp, x) => {
    // Report pole-1 common current through whichever throw is active.
    const right = Number(comp.params.position ?? 0) !== 0;
    const vc1 = ctx.vAt(x, ctx.pinNode(comp.id, "c1"));
    const vthr = ctx.vAt(x, ctx.pinNode(comp.id, right ? "r1" : "l1"));
    ctx.setElementCurrent(comp.id, (vc1 - vthr) / 0.001);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Both ganged poles conduct on their active side only, same 1 mOhm.
    const pins = comp.pins;
    if (pins.length < 6) return;
    const right = Number(comp.params.position ?? 0) !== 0;
    const dp = (id: string) => ctx.pinNode(comp.id, id);
    stampAcAdmittance(ac, dp("c1"), dp(right ? "r1" : "l1"), 1 / 0.001, 0);
    stampAcAdmittance(ac, dp("c2"), dp(right ? "r2" : "l2"), 1 / 0.001, 0);
  },
};

export const dipSwitchModel: DeviceModel = {
  kinds: ["dip_switch"],
  staticStamp: true,
  staticSignature: (_ctx, comp, out) => {
    const p = comp.params;
    const positions = Math.max(2, Math.min(8, Math.round(Number(p.positions ?? 4))));
    out.push(String(positions));
    for (let index = 1; index <= positions; index++) out.push(String(p[`sw${index}`] ?? 0));
  },
  stamp: (ctx, comp, _xGuess, _h) => {
    // Each of the N positions is an independent SPST switch between ai and bi.
    // Closed positions stamp 1 mΩ (mirrors the "switch" kind).  Open positions
    // leave no stamp — open circuit.  sw params beyond params.positions are inert.
    const positions = Math.max(2, Math.min(8, Math.round(Number(comp.params.positions ?? 4))));
    for (let i = 1; i <= positions; i++) {
      const isClosed = Number(comp.params[`sw${i}`] ?? 0) !== 0;
      if (!isClosed) continue;
      const aNode = ctx.pinNode(comp.id, `a${i}`);
      const bNode = ctx.pinNode(comp.id, `b${i}`);
      stampResistor(ctx.mna, aNode, bNode, 0.001);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    // Report the sum of currents through all closed positions.
    const positions = Math.max(2, Math.min(8, Math.round(Number(comp.params.positions ?? 4))));
    let totalI = 0;
    for (let i = 1; i <= positions; i++) {
      const isClosed = Number(comp.params[`sw${i}`] ?? 0) !== 0;
      if (!isClosed) continue;
      const va = ctx.vAt(x, ctx.pinNode(comp.id, `a${i}`));
      const vb = ctx.vAt(x, ctx.pinNode(comp.id, `b${i}`));
      totalI += (va - vb) / 0.001;
    }
    ctx.setElementCurrent(comp.id, totalI);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Each closed position is an independent 1 mOhm contact; open positions
    // stamp nothing, mirroring the transient stamp.
    const positions = Math.max(2, Math.min(8, Math.round(Number(comp.params.positions ?? 4))));
    for (let i = 1; i <= positions; i++) {
      if (Number(comp.params[`sw${i}`] ?? 0) === 0) continue;
      stampAcAdmittance(
        ac,
        ctx.pinNode(comp.id, `a${i}`),
        ctx.pinNode(comp.id, `b${i}`),
        1 / 0.001,
        0,
      );
    }
  },
};

export const potentiometerModel: DeviceModel = {
  kinds: ["potentiometer", "trimmer"],
  staticStamp: true,
  staticSignature: (ctx, comp, out) => {
    const p = comp.params;
    out.push(
      String(p.rTotal ?? 10000),
      String(p.position ?? 0.5),
      String(p.taper ?? "linear"),
      String(ctx.hasFailure(comp.id, "resistor_overload")),
    );
  },
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    if (pins.length < 3) return;
    const cwNode  = ctx.pinNode(comp.id, "cw");
    const wNode   = ctx.pinNode(comp.id, "wiper");
    const ccwNode = ctx.pinNode(comp.id, "ccw");
    const rTotal  = Math.max(1, Number(comp.params.rTotal ?? 10000));
    const pos     = Math.max(0, Math.min(1, Number(comp.params.position ?? 0.5)));
    const taper   = String(comp.params.taper ?? "linear");
    const effPos  = taper === "log"
      ? Math.pow(10, 2 * (pos - 1))
      : pos;
    if (ctx.hasFailure(comp.id, "resistor_overload")) {
      return;
    } else {
      stampResistor(ctx.mna, cwNode,  wNode,   Math.max(1, rTotal * effPos));
      stampResistor(ctx.mna, wNode,   ccwNode, Math.max(1, rTotal * (1 - effPos)));
    }
  },
  updateCurrent: (ctx, comp, x) => {
    const pins = comp.pins;
    if (pins.length < 3) return;
    if (ctx.hasFailure(comp.id, "resistor_overload")) {
      ctx.setElementCurrent(comp.id, 0);
      return;
    }
    const cwV  = ctx.vAt(x, ctx.pinNode(comp.id, "cw"));
    const wiperV = ctx.vAt(x, ctx.pinNode(comp.id, "wiper"));
    const rTotal = Math.max(1, Number(comp.params.rTotal ?? 10000));
    const position = Math.max(0, Math.min(1, Number(comp.params.position ?? 0.5)));
    const effectivePosition = String(comp.params.taper ?? "linear") === "log"
      ? Math.pow(10, 2 * (position - 1))
      : position;
    const rCw = Math.max(1, rTotal * effectivePosition);
    // Per-component current convention is pin 0 → pin 1. For a pot that
    // is CW → wiper, not the unrelated end-to-end track current.
    ctx.setElementCurrent(comp.id, (cwV - wiperV) / rCw);
  },
  updateFailures: (ctx, comp, x, h) => {
    commitResistiveOverloadStress(ctx, comp, x, h);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Two track segments at the OP wiper position; a failed-open track
    // contributes nothing, mirroring the transient stamp's failure gate.
    const pins = comp.pins;
    if (pins.length < 3) return;
    if (ctx.hasFailure(comp.id, "resistor_overload")) return;
    const rTotal = Math.max(1, Number(comp.params.rTotal ?? 10000));
    const pos = Math.max(0, Math.min(1, Number(comp.params.position ?? 0.5)));
    const effPos = String(comp.params.taper ?? "linear") === "log"
      ? Math.pow(10, 2 * (pos - 1))
      : pos;
    const cwNode = ctx.pinNode(comp.id, "cw");
    const wNode = ctx.pinNode(comp.id, "wiper");
    const ccwNode = ctx.pinNode(comp.id, "ccw");
    stampAcAdmittance(ac, cwNode, wNode, 1 / Math.max(1, rTotal * effPos), 0);
    stampAcAdmittance(ac, wNode, ccwNode, 1 / Math.max(1, rTotal * (1 - effPos)), 0);
  },
};

export const fuseModel: DeviceModel = {
  kinds: ["fuse"],
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    const rNormal = Math.max(0.001, Number(comp.params.rNormal ?? 0.1));
    const ai = ctx.pinNode(comp.id, pins[0].id);
    const ci = ctx.pinNode(comp.id, pins[1].id);
    if (!ctx.hasFailure(comp.id, "fuse_tripped")) {
      stampResistor(ctx.mna, ai, ci, rNormal);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    const pins = comp.pins;
    if (ctx.hasFailure(comp.id, "fuse_tripped")) {
      ctx.setElementCurrent(comp.id, 0);
      return;
    }
    const rNormal = Math.max(0.001, Number(comp.params.rNormal ?? 0.1));
    const va = ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id));
    const vc = ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id));
    ctx.setElementCurrent(comp.id, (va - vc) / rNormal);
  },
  updateFailures: (ctx, comp, _x, h) => {
    const iRating = Number(comp.params.iRating ?? 0);
    const current = Math.abs(ctx.elementCurrent(comp.id) ?? 0);
    const currentRatio = iRating > 0 ? current / iRating : 0;
    ctx.recordAccumulatedStress(
      "fuse_tripped",
      comp.id,
      "",
      Math.max(0, currentRatio * currentRatio - 1),
      0.3,
      h,
      FUSE_I2T_THRESHOLD,
      () => ({
        componentId: comp.id,
        kind: "fuse_tripped",
        since: ctx.simTime() + h,
        value: current,
        limit: iRating,
        message:
          `${comp.id} carried ${(current * 1000).toFixed(1)} mA through a ${(iRating * 1000).toFixed(1)} mA fuse rating and tripped open.`,
      }),
    );
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Intact fuse = its normal resistance; a tripped fuse is open, exactly
    // like the transient stamp's committed-failure gate.
    const pins = comp.pins;
    if (ctx.hasFailure(comp.id, "fuse_tripped")) return;
    const rNormal = Math.max(0.001, Number(comp.params.rNormal ?? 0.1));
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      1 / rNormal,
      0,
    );
  },
};

export const ptcFuseModel: DeviceModel = {
  kinds: ["ptc_fuse"],
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    const rNormalPtc = Math.max(0.001, Number(comp.params.rNormal ?? 0.5));
    const ptcSt = ctx.state.ptcs.get(comp.id) ?? { tripped: false, tripStress: 0, recoveryTime: 0 };
    // Tripped resistance = 200x rNormal — high enough to limit fault current,
    // but not truly open (PTCs don't blow, they go high-R).
    const ptcR = ptcSt.tripped ? rNormalPtc * 200 : rNormalPtc;
    const aiPtc = ctx.pinNode(comp.id, pins[0].id);
    const ciPtc = ctx.pinNode(comp.id, pins[1].id);
    stampResistor(ctx.mna, aiPtc, ciPtc, ptcR);
  },
  updateCurrent: (ctx, comp, x) => {
    const pins = comp.pins;
    const rNormalPtcI = Math.max(0.001, Number(comp.params.rNormal ?? 0.5));
    const ptcStI = ctx.state.ptcs.get(comp.id);
    const ptcRI = ptcStI?.tripped ? rNormalPtcI * 200 : rNormalPtcI;
    const vaP = ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id));
    const vcP = ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id));
    ctx.setElementCurrent(comp.id, (vaP - vcP) / ptcRI);
  },
  updateFailures: (ctx, comp, _x, h) => {
    const iHold = Math.max(1e-6, Number(comp.params.iHold ?? 0.5));
    const current = Math.abs(ctx.elementCurrent(comp.id) ?? 0);
    // Initialise state entry if missing (can happen on first step after load()).
    let ptcSt = ctx.state.ptcs.get(comp.id);
    if (!ptcSt) {
      ptcSt = { tripped: false, tripStress: 0, recoveryTime: 0 };
      ctx.state.ptcs.set(comp.id, ptcSt);
    }

    if (!ptcSt.tripped) {
      // Normalized excess I²t: 100 ms at exactly 2× hold current, faster
      // at larger faults. Stress bleeds below hold current instead of being
      // erased by every PWM off interval.
      if (current >= iHold) {
        ptcSt.tripStress += Math.max(0, current * current - iHold * iHold) * h;
        const tripThreshold = 3 * iHold * iHold * 0.1;
        if (ptcSt.tripStress >= tripThreshold) {
          ptcSt.tripped = true;
          ptcSt.tripStress = 0;
          ptcSt.recoveryTime = 0;
        }
      } else {
        // Bleed stress when current is below trip threshold so short spikes
        // don't permanently accumulate.
        ptcSt.tripStress = Math.max(0, ptcSt.tripStress - h * iHold * iHold * 0.5);
      }
    } else {
      // Recovery also requires low hot-state dissipation. A fault that is
      // merely current-limited by the tripped PTC keeps it hot and must not
      // unrealistically reset after a fixed timer.
      const rNormal = Math.max(0.001, Number(comp.params.rNormal ?? 0.5));
      const trippedPower = current * current * rNormal * 200;
      const coolPower = iHold * iHold * rNormal * 0.1;
      if (current < iHold / 2 && trippedPower < coolPower) {
        ptcSt.recoveryTime += h;
        if (ptcSt.recoveryTime >= 2.0) {
          ptcSt.tripped = false;
          ptcSt.tripStress = 0;
          ptcSt.recoveryTime = 0;
        }
      } else {
        // Current rose above recovery threshold — restart the recovery timer.
        ptcSt.recoveryTime = 0;
      }
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // A PTC never opens: the committed trip state selects the same 200x
    // high-resistance value the transient stamp uses, so the small-signal
    // load matches the large-signal one at the OP.
    const pins = comp.pins;
    const rNormal = Math.max(0.001, Number(comp.params.rNormal ?? 0.5));
    const tripped = ctx.state.ptcs.get(comp.id)?.tripped === true;
    const ptcR = tripped ? rNormal * 200 : rNormal;
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      1 / ptcR,
      0,
    );
  },
};

export const ferriteBeadModel: DeviceModel = {
  kinds: ["ferrite_bead"],
  staticStamp: true,
  staticSignature: (_ctx, comp, out) => {
    const p = comp.params;
    out.push(String(p.rDc ?? 0.5));
  },
  stamp: (ctx, comp, _xGuess, _h) => {
    // DC/LF model: stamp as a plain series resistor using the DC resistance
    // parameter. HF suppression needs the bead's frequency-dependent
    // complex impedance and loss curve; adaptive timesteps alone cannot
    // turn this deliberately DC-only part into an RF material model.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const rDc = Math.max(0.001, Number(comp.params.rDc ?? 0.5));
    const aFb = ctx.pinNode(comp.id, pins[0].id);
    const bFb = ctx.pinNode(comp.id, pins[1].id);
    stampResistor(ctx.mna, aFb, bFb, rDc);
  },
  updateCurrent: (ctx, comp, x) => {
    // Reconstruct I from the converged voltages using the DC resistance.
    const pins = comp.pins;
    if (pins.length < 2) { ctx.setElementCurrent(comp.id, 0); return; }
    const rDcI = Math.max(0.001, Number(comp.params.rDc ?? 0.5));
    const vaFb = ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id));
    const vbFb = ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id));
    ctx.setElementCurrent(comp.id, (vaFb - vbFb) / rDcI);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The bead's transient model is deliberately DC-only (rDc); its AC model
    // mirrors that same resistance rather than inventing an RF impedance
    // curve the large-signal engine does not carry.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const rDc = Math.max(0.001, Number(comp.params.rDc ?? 0.5));
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      1 / rDc,
      0,
    );
  },
};

export const resistorArrayModel: DeviceModel = {
  kinds: ["resistor_array"],
  staticStamp: true,
  staticSignature: (ctx, comp, out) => {
    const p = comp.params;
    out.push(String(p.resistance ?? 10_000));
    for (let channel = 1; channel <= 4; channel++) {
      out.push(String(ctx.hasFailure(comp.id, "resistor_overload", `ch${channel}`)));
    }
  },
  stamp: (ctx, comp, _xGuess, _h) => {
    // Four independent resistors sharing the same resistance value.
    // Pins are columnar pairs: a1<->b1, a2<->b2, a3<->b3, a4<->b4.
    // Each channel is fully isolated — no shared bus pin.
    const rArr = Math.max(1, Number(comp.params.resistance ?? 10000));
    for (let i = 1; i <= 4; i++) {
      if (ctx.hasFailure(comp.id, "resistor_overload", `ch${i}`)) continue;
      const aNode = ctx.pinNode(comp.id, `a${i}`);
      const bNode = ctx.pinNode(comp.id, `b${i}`);
      stampResistor(ctx.mna, aNode, bNode, rArr);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    // Report per-channel currents in getElementChannelI; here report the
    // sum of all four channel currents as the scalar elementI (same
    // convention as dip_switch — total through the package).
    const rArrI = Math.max(1, Number(comp.params.resistance ?? 10000));
    let totalArrI = 0;
    for (let i = 1; i <= 4; i++) {
      if (ctx.hasFailure(comp.id, "resistor_overload", `ch${i}`)) continue;
      const vaArr = ctx.vAt(x, ctx.pinNode(comp.id, `a${i}`));
      const vbArr = ctx.vAt(x, ctx.pinNode(comp.id, `b${i}`));
      totalArrI += (vaArr - vbArr) / rArrI;
    }
    ctx.setElementCurrent(comp.id, totalArrI);
  },
  updateFailures: (ctx, comp, x, h) => {
    const specs = ctx.electricalSpecs(comp);
    const pMaxPerChannel = specs?.p_max;
    const resistance = Math.max(1, Number(comp.params.resistance ?? 10_000));
    for (let channel = 1; channel <= 4; channel++) {
      const va = ctx.vAt(x, ctx.pinNode(comp.id, `a${channel}`));
      const vb = ctx.vAt(x, ctx.pinNode(comp.id, `b${channel}`));
      const current = ctx.hasFailure(comp.id, "resistor_overload", `ch${channel}`)
        ? 0
        : (va - vb) / resistance;
      const power = current * current * resistance;
      const powerRatio = pMaxPerChannel != null && pMaxPerChannel > 0
        ? power / pMaxPerChannel
        : 0;
      ctx.recordAccumulatedStress(
        "resistor_overload",
        comp.id,
        `ch${channel}`,
        Math.max(0, powerRatio - 1),
        1,
        h,
        RESISTOR_DAMAGE_THRESHOLD,
        () => ({
          componentId: comp.id,
          pinId: `ch${channel}`,
          kind: "resistor_overload",
          since: ctx.simTime() + h,
          value: power,
          limit: pMaxPerChannel,
          message:
            `${comp.id} channel ${channel} dissipated ${power.toFixed(3)} W through a ${pMaxPerChannel!.toFixed(3)} W per-channel rating long enough to fail open.`,
        }),
      );
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Four isolated channels; failed-open channels contribute nothing.
    const rArr = Math.max(1, Number(comp.params.resistance ?? 10000));
    for (let i = 1; i <= 4; i++) {
      if (ctx.hasFailure(comp.id, "resistor_overload", `ch${i}`)) continue;
      stampAcAdmittance(
        ac,
        ctx.pinNode(comp.id, `a${i}`),
        ctx.pinNode(comp.id, `b${i}`),
        1 / rArr,
        0,
      );
    }
  },
};

export const ldrModel: DeviceModel = {
  kinds: ["ldr"],
  staticStamp: true,
  staticSignature: (ctx, comp, out) => {
    const p = comp.params;
    out.push(
      String(p.rDark ?? 1_000_000),
      String(p.rLight ?? 500),
      String(ctx.envLux(comp)),
      String(p.r10 ?? 15_000),
      String(p.gamma ?? 0.7),
    );
  },
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    const rDark  = Number(comp.params.rDark  ?? 1_000_000);
    const rLight = Number(comp.params.rLight ?? 500);
    const lux    = ctx.envLux(comp);
    const r10 = Number(comp.params.r10 ?? 15_000);
    const gamma = Number(comp.params.gamma ?? 0.7);
    const R = ldrResistance(rDark, rLight, lux, r10, gamma);
    const ai = ctx.pinNode(comp.id, pins[0].id);
    const ci = ctx.pinNode(comp.id, pins[1].id);
    stampResistor(ctx.mna, ai, ci, R);
  },
  updateCurrent: (ctx, comp, x) => {
    const pins = comp.pins;
    const rDark  = Number(comp.params.rDark  ?? 1_000_000);
    const rLight = Number(comp.params.rLight ?? 500);
    const lux    = ctx.envLux(comp);
    const r10 = Number(comp.params.r10 ?? 15_000);
    const gamma = Number(comp.params.gamma ?? 0.7);
    const R = ldrResistance(rDark, rLight, lux, r10, gamma);
    const va = ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id));
    const vc = ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id));
    ctx.setElementCurrent(comp.id, (va - vc) / R);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Conductance at the OP illuminance — the same resistance the transient
    // stamp computes from the held environment.
    const pins = comp.pins;
    const R = ldrResistance(
      Number(comp.params.rDark ?? 1_000_000),
      Number(comp.params.rLight ?? 500),
      ctx.envLux(comp),
      Number(comp.params.r10 ?? 15_000),
      Number(comp.params.gamma ?? 0.7),
    );
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      1 / R,
      0,
    );
  },
};

export const thermistorModel: DeviceModel = {
  kinds: ["thermistor"],
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    const rNominal = Number(comp.params.rNominal ?? 10000);
    const beta     = Number(comp.params.beta    ?? 3950);
    const tempC    = ctx.state.thermalTemps.get(comp.id) ?? ctx.envTempC(comp);
    const R  = thermistorResistance(rNominal, beta, tempC);
    const ai = ctx.pinNode(comp.id, pins[0].id);
    const ci = ctx.pinNode(comp.id, pins[1].id);
    stampResistor(ctx.mna, ai, ci, R);
  },
  commitState: (ctx, comp, x, h) => {
    if (!(comp.pins.length >= 2)) return;
    const ambientC = ctx.envTempC(comp);
    const previousC = ctx.state.thermalTemps.get(comp.id) ?? ambientC;
    const rNominal = Number(comp.params.rNominal ?? 10_000);
    const beta = Number(comp.params.beta ?? 3950);
    const resistance = thermistorResistance(rNominal, beta, previousC);
    const a = ctx.pinNode(comp.id, comp.pins[0].id);
    const b = ctx.pinNode(comp.id, comp.pins[1].id);
    const voltage = ctx.vAt(x, a) - ctx.vAt(x, b);
    const power = voltage * voltage / Math.max(1, resistance);
    // Dissipation factor is W/K. A first-order thermal body approaches
    // Tamb + P/delta with time constant tau, so low-excitation sensors
    // remain at ambient while excessive divider current reads high.
    const dissipationFactor = Math.max(
      1e-6,
      Number(comp.params.dissipationFactor ?? 0.007),
    );
    const thermalTau = Math.max(1e-6, Number(comp.params.thermalTau ?? 15));
    const targetC = ambientC + power / dissipationFactor;
    const nextC = targetC + (previousC - targetC) * Math.exp(-h / thermalTau);
    ctx.state.thermalTemps.set(
      comp.id,
      Number.isFinite(nextC) ? Math.max(-272.15, Math.min(1000, nextC)) : ambientC,
    );
  },
  updateCurrent: (ctx, comp, x) => {
    const pins = comp.pins;
    const rNominal = Number(comp.params.rNominal ?? 10000);
    const beta     = Number(comp.params.beta    ?? 3950);
    const tempC    = ctx.state.thermalTemps.get(comp.id) ?? ctx.envTempC(comp);
    const R  = thermistorResistance(rNominal, beta, tempC);
    const va = ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id));
    const vc = ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id));
    ctx.setElementCurrent(comp.id, (va - vc) / R);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Conductance at the committed body temperature — self-heating is held
    // at its OP value (the thermal integrator is physical-time state).
    const pins = comp.pins;
    const tempC = ctx.state.thermalTemps.get(comp.id) ?? ctx.envTempC(comp);
    const R = thermistorResistance(
      Number(comp.params.rNominal ?? 10000),
      Number(comp.params.beta ?? 3950),
      tempC,
    );
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      1 / R,
      0,
    );
  },
};
