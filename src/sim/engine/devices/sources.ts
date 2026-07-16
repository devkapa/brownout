/**
 * SOURCES device cohort (Wave A4 phase 2): voltage_source, battery_pack,
 * bench_psu, clock, clock_gen, pulse_source, pulse_gen, signal_gen; plus
 * current_source (Wave A7, registry-first — see its own comment).
 *
 * Every handler body below is the engine's old switch-case / if-block body
 * MOVED VERBATIM, with identifier access adapted through DeviceContext
 * (this.vsrcIdx.get -> ctx.vsrcRow, this._pinNode -> ctx.pinNode,
 * this._stampIndependentVoltageSource -> ctx.stampIndependentVoltageSource,
 * out[comp.id] = v -> ctx.setElementCurrent(comp.id, v), loop `continue` ->
 * handler `return` — every legacy _updateState block is kind-exclusive, so
 * the two are equivalent). Do not simplify, reorder mna.add calls, or
 * rewrite algebra here: float accumulation order in the matrix is
 * semantics, and the bitwise oracle suite pins these exact trajectories.
 *
 * Branch-row allocation moved from _buildMatrix into branchRows hooks. The
 * engine consults the hook at the component's exact position in the same
 * component iteration, so returning [comp.id] under the old chain's exact
 * condition preserves every vsrcIdx row index bit-for-bit.
 *
 * Kinds grouped into one model correspond exactly to switch cases that
 * shared a body in the engine (clock/clock_gen and pulse_source/pulse_gen
 * in the stamp switch; all non-signal_gen sources shared one element-
 * current case, kept as one shared helper here).
 *
 * Engine-owned machinery deliberately NOT moved: the static ideal-source
 * incidence (reached through ctx.stampIndependentVoltageSource), the
 * current-limit active sets (shared with the unmigrated regulator kinds),
 * the battery model cache (shared with the public telemetry readout),
 * load()-time state carry-forward, and the _updateDigitalState cases for
 * clock/pulse level readouts (the registry has no digital hook).
 */

import type {
  AcDeviceContext,
  DeviceComponent,
  DeviceContext,
  DeviceModel,
} from "../device-registry.js";
import {
  stampAcAdmittance,
  stampAcVoltageSourceRow,
  type AcStampSurface,
} from "../ac-system.js";
import {
  stampCurrentSource,
  stampResistor,
  stampVSource,
  stampVSourceSeriesR,
} from "../elements.js";
import { clockVoltage, pulseVoltage } from "../digital.js";
import {
  parseSignalGenParams,
  signalGenEnabled,
  signalGenVoltage,
  type SignalGenParams,
} from "../waveform.js";
import { advancePrimaryBatteryState } from "../../battery-physics.js";

// Per-params-object parse cache for signal_gen components. Keyed by the
// params object reference — the store produces a new object on every edit,
// so a new load() naturally invalidates the old entry without any manual
// bookkeeping. Module-scoped (the engine field it replaces was per-instance)
// because the parse is a pure function of the params object, so sharing a
// parsed entry across engine instances cannot change a stamped value.
const signalGenCache = new WeakMap<Record<string, number | string>, SignalGenParams>();

/**
 * Shared element-current publisher for every source stamped as an ideal
 * vsrc branch — one case body in the engine's _updateElementI
 * (battery_pack, bench_psu, pulse_gen, clock_gen, voltage_source,
 * pulse_source, clock), kept as one body here.
 */
function publishSourceBranchCurrent(
  ctx: DeviceContext,
  comp: DeviceComponent,
  x: Float64Array,
): void {
  const k = ctx.vsrcRow(comp.id);
  // Extra-variable row holds the source current directly.
  ctx.setElementCurrent(comp.id, k !== undefined ? (x[k] ?? 0) : 0);
}

/**
 * Shared small-signal stamp for every source that is an ideal voltage branch
 * in the transient system (voltage_source, clock/clock_gen,
 * pulse_source/pulse_gen). Waveform shape is large-signal state, so all of
 * them linearize identically: the branch row keeps the exact stampVSource
 * incidence — an ideal independent source is an AC short — and the drive is
 * AC-zeroed unless this component is the analysis' designated input, which
 * injects the unit excitation V = 1 + j0.
 */
function acStampIdealSourceBranch(
  ctx: AcDeviceContext,
  comp: DeviceComponent,
  ac: AcStampSurface,
): void {
  const pins = comp.pins;
  if (pins.length < 2) return;
  const k = ctx.vsrcRow(comp.id);
  if (k === undefined) return;
  stampAcVoltageSourceRow(
    ac,
    ctx.pinNode(comp.id, pins[0].id),
    ctx.pinNode(comp.id, pins[1].id),
    k,
    ctx.acInputMagnitude(comp.id),
    0,
  );
}

export const voltageSourceModel: DeviceModel = {
  kinds: ["voltage_source"],
  branchRows: (comp, _ctx) => [comp.id],
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    {
      const k = ctx.vsrcRow(comp.id);
      if (k === undefined) return;
      const V = ctx.independentSourceMagnitude(Number(comp.params.voltage ?? 5));
      ctx.stampIndependentVoltageSource(comp, k, V);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    publishSourceBranchCurrent(ctx, comp, x);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    acStampIdealSourceBranch(ctx, comp, ac);
  },
};

export const batteryPackModel: DeviceModel = {
  kinds: ["battery_pack"],
  branchRows: (comp, _ctx) => [comp.id],
  stamp: (ctx, comp, _xGuess, _h) => {
    // Chemistry-aware Thevenin source. Exact catalog identities use a
    // piecewise OCV/SoC curve plus depletion- and temperature-dependent
    // resistance; legacy/custom sources without identity keep an explicit
    // generic-primary fallback. Runtime SoC is committed post-solve.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const kBat = ctx.vsrcRow(comp.id);
    if (kBat === undefined) return;
    const battery = ctx.batteryOperatingPoint(comp);
    stampVSourceSeriesR(
      ctx.mna,
      ctx.pinNode(comp.id, "pos"),
      ctx.pinNode(comp.id, "neg"),
      kBat,
      ctx.independentSourceMagnitude(
        battery.point.openCircuitVoltageV * battery.nominalVoltageScale,
      ),
      battery.point.internalResistanceOhm,
    );
  },
  commitState: (ctx, comp, x, h) => {
    const k = ctx.vsrcRow(comp.id);
    const battery = ctx.batteryOperatingPoint(comp);
    if (k !== undefined) {
      const advanced = advancePrimaryBatteryState(
        battery.profile,
        battery.state,
        {
          // Voltage-source branch current is positive into the source;
          // negate it so positive means discharge delivered to the circuit.
          dischargeCurrentA: -(x[k] ?? 0),
          dtSeconds: h,
        },
      );
      ctx.state.batteries.set(comp.id, {
        ...advanced.state,
        authoredCharge: battery.state.authoredCharge,
        catalogUid: battery.state.catalogUid,
      });
    }
  },
  updateCurrent: (ctx, comp, x) => {
    publishSourceBranchCurrent(ctx, comp, x);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // AC short through the chemistry model's internal resistance at the OP:
    // the same stampVSourceSeriesR shape (incidence plus -r on the branch
    // diagonal), with the OCV drive AC-zeroed unless designated as input.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const k = ctx.vsrcRow(comp.id);
    if (k === undefined) return;
    const battery = ctx.batteryOperatingPoint(comp);
    stampAcVoltageSourceRow(
      ac,
      ctx.pinNode(comp.id, "pos"),
      ctx.pinNode(comp.id, "neg"),
      k,
      ctx.acInputMagnitude(comp.id),
      0,
    );
    const rInternal = battery.point.internalResistanceOhm;
    if (rInternal > 0) ac.addAc(k, k, -rInternal, 0);
  },
};

export const benchPsuModel: DeviceModel = {
  kinds: ["bench_psu"],
  branchRows: (comp, _ctx) => [comp.id],
  stamp: (ctx, comp, xGuess, _h) => {
    // W5.1: bench_psu gains optional constant-current fold.
    // When params.iLimit is present and > 0, the committed-regime pattern
    // is used (CV mode = ideal vsource; CC mode = branch current pinned to iLimit).
    // When iLimit is absent or <= 0, behaviour is byte-identical to the ideal
    // vsource path above — no icState entry is created, no regime logic runs.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const kPsu = ctx.vsrcRow(comp.id);
    if (kPsu === undefined) return;
    // Scaled once here so the CC compliance comparison and the CV stamp
    // see one consistent per-rung setpoint during source stepping.
    const Vpsu = ctx.independentSourceMagnitude(Number(comp.params.voltage ?? 5));
    const iLimitPsu = Number(comp.params.iLimit ?? 0);

    if (iLimitPsu > 0) {
      // iLimit is active — check committed regime (0=CV, 2=CC).
      const stPsu = ctx.state.icState.get(comp.id);
      const regPsu = stPsu ? (stPsu.reg ?? 0) : 0;

      const posNode = ctx.pinNode(comp.id, pins[0].id);
      const negNode = ctx.pinNode(comp.id, pins[1].id);
      const sourceDirection = Vpsu < 0 ? -1 : 1;
      const activeSetKey = `bench_psu:${comp.id}`;
      const entryClamp = regPsu !== 2 && ctx.useCurrentLimitEntryClamp(
        activeSetKey,
        xGuess[kPsu] ?? 0,
        iLimitPsu,
      );
      const currentLimited = regPsu === 2 || entryClamp;
      const outputMagnitudeGuess = sourceDirection * (
        ctx.vAt(xGuess, posNode) - ctx.vAt(xGuess, negNode)
      );
      const complianceClamp = currentLimited && ctx.useCurrentLimitComplianceClamp(
        activeSetKey,
        outputMagnitudeGuess,
        Math.abs(Vpsu),
      );

      if (currentLimited && !complianceClamp) {
        // CC mode: pin the branch current to iLimit.
        // KCL coupling rows match stampVSource convention:
        //   mna.add(pos, k,  1) → x[k] leaves posNode (enters source branch)
        //   mna.add(neg, k, -1) → x[k] enters negNode
        // In this MNA convention, x[k] is NEGATIVE when the source delivers
        // current (V_pos > 0 means the source pushes current into the circuit).
        // stampVSource with V=10, R=2 gives x[k]=-5 (5A delivered).
        // Constraint: x[k] = −iLimit (negative = delivering iLimit to circuit).
        if (posNode >= 0) ctx.mna.add(posNode, kPsu,  1);
        if (negNode >= 0) ctx.mna.add(negNode, kPsu, -1);
        // Constraint row: x[k] = −direction·iLimit. Negative x delivers
        // power from a positive-voltage source; a negative setpoint reverses
        // both voltage and delivery-current direction.
        ctx.mna.add(kPsu, kPsu, 1);
        ctx.mna.addB(kPsu, -sourceDirection * iLimitPsu);
      } else {
        // CV mode, or the same-step compliance branch selected from a
        // provisional CC candidate. The ordinary voltage-source branch
        // current is the actual terminal current, so source power and
        // telemetry never retain the stale full CC current after recovery.
        stampVSource(
          ctx.mna,
          posNode,
          negNode,
          kPsu,
          Vpsu,
        );
      }
    } else {
      // No current limit — ideal voltage source (legacy behaviour).
      stampVSource(
        ctx.mna,
        ctx.pinNode(comp.id, pins[0].id),
        ctx.pinNode(comp.id, pins[1].id),
        kPsu,
        Vpsu,
      );
    }
  },
  commitState: (ctx, comp, x, _h) => {
    // W5.1 — bench_psu iLimit commit: only when iLimit is active.
    const iLimitBpsu = Number(comp.params.iLimit ?? 0);
    if (iLimitBpsu > 0) {
      const kBpsu = ctx.vsrcRow(comp.id);
      if (kBpsu === undefined) return;
      const stBpsu = ctx.state.icState.get(comp.id);
      const prevRegBpsu = stBpsu ? (stBpsu.reg ?? 0) : 0;
      const iBpsu = x[kBpsu] ?? 0;  // negative when delivering current (MNA convention)
      const VBpsu = Number(comp.params.voltage ?? 5);
      const posNodeBpsu = ctx.pinNode(comp.id, comp.pins[0].id);
      const negNodeBpsu = ctx.pinNode(comp.id, comp.pins[1].id);
      const sourceDirectionBpsu = VBpsu < 0 ? -1 : 1;
      const vOutBpsu = sourceDirectionBpsu * (
        ctx.vAt(x, posNodeBpsu) - ctx.vAt(x, negNodeBpsu)
      );
      let newRegBpsu: number;
      const activeSetKeyBpsu = `bench_psu:${comp.id}`;
      if (
        ctx.currentLimitEntryClampActive(activeSetKeyBpsu)
        && !ctx.currentLimitComplianceClampActive(activeSetKeyBpsu)
      ) {
        newRegBpsu = 2;
      } else if (prevRegBpsu === 2) {
        // In CC mode: x[k] is forced to −iLimit; cannot use |i| to detect recovery.
        // Exit CC when the voltage at the pos node has risen to near the source
        // voltage — this means the effective load resistance has increased to the
        // point where the natural (CV) current would be ≤ iLimit.
        // Condition uses the source differential (and its configured
        // polarity), rather than assuming the negative terminal is ground.
        newRegBpsu = vOutBpsu >= Math.abs(VBpsu) * 0.98 ? 0 : 2;
      } else {
        // In CV: enter CC if the supplied current exceeds the limit.
        // x[k] is negative (source delivers current); |iBpsu| is the magnitude.
        newRegBpsu = Math.abs(iBpsu) > iLimitBpsu ? 2 : 0;
      }
      ctx.state.icState.set(comp.id, { reg: newRegBpsu });
    }
  },
  updateCurrent: (ctx, comp, x) => {
    publishSourceBranchCurrent(ctx, comp, x);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Linearize at the committed regime: CV is an ideal AC short (branch row
    // with V = 0, or 1 for the designated input); a committed CC fold pins
    // the branch current, so its small-signal current is exactly zero and no
    // drive can inject through it.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const k = ctx.vsrcRow(comp.id);
    if (k === undefined) return;
    const posNode = ctx.pinNode(comp.id, pins[0].id);
    const negNode = ctx.pinNode(comp.id, pins[1].id);
    const iLimit = Number(comp.params.iLimit ?? 0);
    const st = ctx.state.icState.get(comp.id);
    const reg = iLimit > 0 && st ? (st.reg ?? 0) : 0;
    if (reg === 2) {
      // Committed CC: keep the KCL coupling incidence, constrain dI = 0.
      ac.addAc(posNode, k, 1, 0);
      ac.addAc(negNode, k, -1, 0);
      ac.addAc(k, k, 1, 0);
      return;
    }
    stampAcVoltageSourceRow(ac, posNode, negNode, k, ctx.acInputMagnitude(comp.id), 0);
  },
};

export const clockModel: DeviceModel = {
  kinds: ["clock", "clock_gen"],
  branchRows: (comp, _ctx) => [comp.id],
  stamp: (ctx, comp, _xGuess, h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    const k = ctx.vsrcRow(comp.id);
    if (k === undefined) return;
    const freq = Math.max(1e-6, Number(comp.params.frequency ?? 1000));
    const duty = Number(comp.params.duty ?? 0.5);
    const vHi = Number(comp.params.v_high ?? 5);
    const vLo = Number(comp.params.v_low ?? 0);
    const delay = Number(comp.params.delay ?? 0);
    const V = ctx.independentSourceMagnitude(
      clockVoltage(ctx.waveformSampleTime(h), 1 / freq, duty, vHi, vLo, delay),
    );
    ctx.stampIndependentVoltageSource(comp, k, V);
  },
  updateCurrent: (ctx, comp, x) => {
    publishSourceBranchCurrent(ctx, comp, x);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    acStampIdealSourceBranch(ctx, comp, ac);
  },
};

export const pulseSourceModel: DeviceModel = {
  kinds: ["pulse_source", "pulse_gen"],
  branchRows: (comp, _ctx) => [comp.id],
  stamp: (ctx, comp, _xGuess, h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    const k = ctx.vsrcRow(comp.id);
    if (k === undefined) return;
    const v1  = Number(comp.params.v1  ?? 0);
    const v2  = Number(comp.params.v2  ?? 5);
    const td  = Number(comp.params.td  ?? 0);
    const tr  = Number(comp.params.tr  ?? 1e-6);
    const tf  = Number(comp.params.tf  ?? 1e-6);
    const pw  = Number(comp.params.pw  ?? 5e-4);
    const per = Number(comp.params.per ?? 1e-3);
    const V = ctx.independentSourceMagnitude(
      pulseVoltage(ctx.waveformSampleTime(h), v1, v2, td, tr, tf, pw, per),
    );
    ctx.stampIndependentVoltageSource(comp, k, V);
  },
  updateCurrent: (ctx, comp, x) => {
    publishSourceBranchCurrent(ctx, comp, x);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    acStampIdealSourceBranch(ctx, comp, ac);
  },
};

export const currentSourceModel: DeviceModel = {
  kinds: ["current_source"],
  // Wave A7 (SPICE interop): the first NEW kind in this cohort — nothing
  // here is moved engine code. Branch-free Norton form on purpose: an ideal
  // current source is RHS-only in MNA (stampCurrentSource), so it needs no
  // extra-variable row and no branchRows hook; the universal 1e-12 node
  // shunt keeps a source feeding an otherwise-open node solvable exactly as
  // it does for every high-Z default.
  //
  // Sign convention is the SPICE I-element's: params.current is the current
  // flowing from the pos pin THROUGH the source to the neg pin, i.e. it is
  // drawn out of the pos node and injected into the neg node ("I1 0 out 1m"
  // lifts node out positive). stampCurrentSource(i, j, I) injects INTO i and
  // OUT of j, hence the (neg, pos) argument order below.
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    const posNode = ctx.pinNode(comp.id, pins[0].id);
    const negNode = ctx.pinNode(comp.id, pins[1].id);
    // Scaled like every independent drive so the Wave A3 source-stepping
    // ladder relaxes I sources together with V sources.
    const I = ctx.independentSourceMagnitude(Number(comp.params.current ?? 0));
    stampCurrentSource(ctx.mna, negNode, posNode, I);
    const rParallel = Number(comp.params.rParallel ?? 0);
    if (Number.isFinite(rParallel) && rParallel > 0) {
      stampResistor(ctx.mna, posNode, negNode, rParallel);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    // Element-current convention is pin0 -> pin1 (pos -> neg THROUGH the
    // component): the ideal branch carries exactly the programmed current,
    // plus the declared parallel shunt's terminal current. The homotopy
    // scale is applied so a mid-ladder rung publishes the same drive it
    // stamped (a voltage source's x[k] reflects the scaled stamp the same
    // way); at any accepted point the scale is 1 and this is exact.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const I = ctx.independentSourceMagnitude(Number(comp.params.current ?? 0));
    const rParallel = Number(comp.params.rParallel ?? 0);
    if (Number.isFinite(rParallel) && rParallel > 0) {
      const vd = ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id))
        - ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id));
      ctx.setElementCurrent(comp.id, I + vd / rParallel);
      return;
    }
    ctx.setElementCurrent(comp.id, I);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The SPICE small-signal convention for an I element: AC-zeroed it is an
    // OPEN (a constant-current branch has zero admittance — nothing stamps),
    // and as the designated input it injects the unit 1 A drive with the
    // transient stamp's polarity (out of pos, into neg). The declared
    // rParallel is a physical terminal resistor, so it stays as a real
    // conductance either way, mirroring the transient stamp exactly.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const posNode = ctx.pinNode(comp.id, pins[0].id);
    const negNode = ctx.pinNode(comp.id, pins[1].id);
    const rParallel = Number(comp.params.rParallel ?? 0);
    if (Number.isFinite(rParallel) && rParallel > 0) {
      stampAcAdmittance(ac, posNode, negNode, 1 / rParallel, 0);
    }
    const magnitude = ctx.acInputMagnitude(comp.id);
    if (magnitude !== 0) {
      ac.addBAc(negNode, magnitude, 0);
      ac.addBAc(posNode, -magnitude, 0);
    }
  },
};

export const signalGenModel: DeviceModel = {
  kinds: ["signal_gen"],
  branchRows: (comp, _ctx) => {
    // Register a branch row ONLY when the component will actually be stamped
    // as an ideal voltage source (rSource <= 0 and enabled).  A registered-but-
    // unstamped row leaves an all-zero matrix row which makes the matrix singular.
    // The registration condition must mirror the stamp condition exactly; param
    // edits always go through a fresh load(), keeping the two in sync.
    const rSource = Number(comp.params.rSource ?? 50);
    if (signalGenEnabled(comp.params) && rSource <= 0) {
      return [comp.id];
    }
    return [];
  },
  stamp: (ctx, comp, _xGuess, h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    // Parse once per params object reference; the store produces a new
    // object on every edit, so the WeakMap entry is naturally invalidated.
    let parsed = signalGenCache.get(comp.params);
    if (!parsed) {
      parsed = parseSignalGenParams(comp.params);
      signalGenCache.set(comp.params, parsed);
    }

    // Disabled: hi-Z; stamp nothing. RSHUNT keeps the matrix regular.
    if (!signalGenEnabled(comp.params)) return;

    const posNode = ctx.pinNode(comp.id, pins[0].id);
    const negNode = ctx.pinNode(comp.id, pins[1].id);
    // Scaled before the branch below so the ideal-vsource and Norton
    // paths ramp identically during source stepping.
    const Vt = ctx.independentSourceMagnitude(
      signalGenVoltage(parsed, ctx.waveformSampleTime(h)),
    );
    const rSource = Number(comp.params.rSource ?? 50);

    if (rSource <= 0) {
      // Ideal voltage source: stamp as a proper vsrc branch.
      const k = ctx.vsrcRow(comp.id);
      if (k === undefined) return;
      stampVSource(ctx.mna, posNode, negNode, k, Vt);
    } else {
      // Finite source impedance: Norton equivalent (Thevenin V+R needs an
      // internal node the engine does not allocate; Norton is electrically
      // identical at the terminals and requires only the two existing nodes).
      // Norton current I_N = V_oc / rSource flows INTO pos and OUT of neg.
      stampResistor(ctx.mna, posNode, negNode, rSource);
      stampCurrentSource(ctx.mna, posNode, negNode, Vt / rSource);
    }
  },
  updateCurrent: (ctx, comp, x, h) => {
    if (!signalGenEnabled(comp.params)) {
      ctx.setElementCurrent(comp.id, 0);
      return;
    }
    const rSource = Number(comp.params.rSource ?? 50);
    if (rSource <= 0) {
      // Ideal path: branch current lives in the extra-variable row.
      const k = ctx.vsrcRow(comp.id);
      ctx.setElementCurrent(comp.id, k !== undefined ? (x[k] ?? 0) : 0);
    } else {
      // Norton path: compute how much current the instrument actually sources
      // at the terminals given the converged node voltages.
      let parsed = signalGenCache.get(comp.params);
      if (!parsed) {
        parsed = parseSignalGenParams(comp.params);
        signalGenCache.set(comp.params, parsed);
      }
      // _stampAll solved the source at _waveformSampleTime(h). Use the
      // same helper here so the sourced current is measured against
      // the exact instant that was stamped (simTime advances only
      // after _solve; OP solves pin the instant).
      const Vt = signalGenVoltage(parsed, ctx.waveformSampleTime(h));
      // Engine _updateElementI preamble, copied per-handler (same pure reads).
      const pins = comp.pins;
      const a = ctx.pinNode(comp.id, pins[0].id);
      const b = ctx.pinNode(comp.id, pins[1].id);
      const va = ctx.vAt(x, a);
      const vb = ctx.vAt(x, b);
      const vd = va - vb;
      // vd is already va - vb (pos - neg) from the preamble above.
      ctx.setElementCurrent(comp.id, (Vt - vd) / rSource);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Mirrors the transient stamp's three shapes: disabled = hi-Z (nothing),
    // rSource <= 0 = ideal branch (AC short / unit drive), rSource > 0 =
    // Norton — the source resistance stays as a real conductance and the
    // designated-input drive injects its Norton current 1/rSource.
    const pins = comp.pins;
    if (pins.length < 2) return;
    if (!signalGenEnabled(comp.params)) return;
    const posNode = ctx.pinNode(comp.id, pins[0].id);
    const negNode = ctx.pinNode(comp.id, pins[1].id);
    const rSource = Number(comp.params.rSource ?? 50);
    const magnitude = ctx.acInputMagnitude(comp.id);
    if (rSource <= 0) {
      const k = ctx.vsrcRow(comp.id);
      if (k === undefined) return;
      stampAcVoltageSourceRow(ac, posNode, negNode, k, magnitude, 0);
      return;
    }
    stampAcAdmittance(ac, posNode, negNode, 1 / rSource, 0);
    if (magnitude !== 0) {
      ac.addBAc(posNode, magnitude / rSource, 0);
      ac.addBAc(negNode, -magnitude / rSource, 0);
    }
  },
};
