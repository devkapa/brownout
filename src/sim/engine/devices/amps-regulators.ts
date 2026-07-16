/**
 * AMPS-REGULATORS device cohort (Wave A4 phase 2): dcdc_converter, lm358,
 * mcp6002, lm386, lm393, linear_reg, lm317.
 *
 * Every handler body below is the engine's old switch-case / if-block body
 * MOVED VERBATIM, with identifier access adapted through DeviceContext
 * (this._pinNode -> ctx.pinNode, this._vAt -> ctx.vAt, this.vsrcIdx.get ->
 * ctx.vsrcRow, this._icPowerInfo -> ctx.icPowerInfo, this.state.icState ->
 * ctx.state.icState, this._currentLimitEntryClamps.has ->
 * ctx.currentLimitEntryClampActive, out[comp.id] = v ->
 * ctx.setElementCurrent(comp.id, v), loop break/continue -> handler
 * `return` — every legacy _updateState block is kind-exclusive, so the two
 * are equivalent). Do not simplify, reorder mna.add calls, or rewrite
 * algebra here: float accumulation order in the matrix is semantics, and
 * the bitwise oracle suite pins these exact trajectories.
 *
 * These kinds own branch rows (branchRows hooks reproduce the old
 * _buildMatrix chain conditions exactly — unconditional per kind) AND
 * committed active-set regimes. All regime state lives in engine state
 * reached only through the context: icState (committed regimes, op-amp
 * analogue memory), the per-solve current-limit/headroom active sets
 * (ctx.useCurrentLimitEntryClamp, ctx.useCurrentLimitComplianceClamp, and
 * ctx.regulatorHeadroomRegime — engine solve state cleared once per Newton
 * solve), and thermalDevices (read-only shutdown flag written by the
 * engine's package-thermal pass).
 *
 * Engine-owned machinery deliberately NOT moved: _icPowerInfo and
 * _stampDigitalOutput (shared with the unmigrated digital IC kinds,
 * reached via ctx), defaultIcState and the load()-time icState
 * carry-forward (defaultIcState covers every IC_STATE_KINDS kind, so the
 * whole table stays with the engine and is imported here),
 * regulatorQuiescentCurrent/analogIcQuiescentCurrent (also consumed by the
 * engine's _thermalDissipationW pass), and the _drivenOutputTargets lm393
 * case (part of the generic output-sag scan, not a per-kind pass switch).
 *
 * Import layering (registry header, decision 7): pure helper FUNCTIONS
 * from sim-engine.ts are safe to import despite the module cycle because
 * handlers dereference them only at solve time. Never read sim-engine
 * bindings at module evaluation time from this file.
 */

import type {
  AcDeviceContext,
  DeviceComponent,
  DeviceContext,
  DeviceModel,
} from "../device-registry.js";
import { stampAcAdmittance, type AcStampSurface } from "../ac-system.js";
import {
  opAmpCurrentLimitMode,
  opAmpSlewLimitedTarget,
  opAmpTransientTarget,
  regulatorRegime,
  stampCurrentSource,
  stampLinearRegulator,
  stampOpAmp,
  stampResistor,
} from "../elements.js";
import {
  analogIcQuiescentCurrent,
  defaultIcState,
  regulatorQuiescentCurrent,
  type IcPowerInfo,
} from "../sim-engine.js";

interface OpAmpModelParams {
  openLoopGain: number;
  gbwHz: number;
  slewRate: number;
  sourceCurrentLimit: number;
  sinkCurrentLimit: number;
  outputResistance: number;
}

const OPAMP_MODEL_DEFAULTS: Readonly<Record<
  "lm358" | "mcp6002" | "lm386",
  OpAmpModelParams
>> = {
  // Typical room-temperature teaching values. Exact limits vary with supply,
  // load, package vendor, and temperature; catalog notes disclose that envelope.
  lm358: {
    openLoopGain: 100_000,
    gbwHz: 1_000_000,
    slewRate: 300_000,
    sourceCurrentLimit: 0.04,
    sinkCurrentLimit: 0.02,
    outputResistance: 50,
  },
  mcp6002: {
    openLoopGain: 100_000,
    gbwHz: 1_000_000,
    slewRate: 600_000,
    sourceCurrentLimit: 0.023,
    sinkCurrentLimit: 0.023,
    outputResistance: 50,
  },
  // 6 MHz gain-bandwidth yields the LM386's declared ~300 kHz bandwidth at
  // its default closed-loop gain of 20.
  lm386: {
    openLoopGain: 20,
    gbwHz: 6_000_000,
    slewRate: 300_000,
    sourceCurrentLimit: 0.25,
    sinkCurrentLimit: 0.25,
    outputResistance: 2,
  },
};

function nonNegativeParam(value: unknown, fallback: number): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function opAmpModelParams(comp: DeviceComponent, fixedGain?: number): OpAmpModelParams {
  const defaults = OPAMP_MODEL_DEFAULTS[comp.kind as keyof typeof OPAMP_MODEL_DEFAULTS];
  const openLoopGain = fixedGain ?? Math.max(
    1,
    nonNegativeParam(comp.params.openLoopGain, defaults.openLoopGain),
  );
  return {
    openLoopGain,
    gbwHz: nonNegativeParam(comp.params.gbw, defaults.gbwHz),
    slewRate: nonNegativeParam(comp.params.slewRate, defaults.slewRate),
    sourceCurrentLimit: nonNegativeParam(
      comp.params.sourceCurrentLimit,
      defaults.sourceCurrentLimit,
    ),
    sinkCurrentLimit: nonNegativeParam(
      comp.params.sinkCurrentLimit,
      defaults.sinkCurrentLimit,
    ),
    outputResistance: Math.max(
      0.01,
      nonNegativeParam(comp.params.outputResistance, defaults.outputResistance),
    ),
  };
}

/**
 * Resolve the accepted-step gain-stage target used by the op-amp stamp.
 * Slew limiting is applied before rail limiting, so recovery from one rail
 * cannot jump directly to the opposite rail even under extreme overdrive.
 */
function opAmpStepTarget(
  previousInternalVoltage: number,
  inputDifferential: number,
  model: OpAmpModelParams,
  outputOffset: number,
  h: number,
  vLow: number,
  vHigh: number,
): { unconstrained: number; bounded: number; fixedTarget: number | null } {
  const unconstrained = opAmpTransientTarget(
    previousInternalVoltage,
    inputDifferential,
    model.openLoopGain,
    outputOffset,
    model.gbwHz,
    h,
  );
  const slewLimited = opAmpSlewLimitedTarget(
    previousInternalVoltage,
    unconstrained,
    model.slewRate,
    h,
  );
  const bounded = Math.max(vLow, Math.min(vHigh, slewLimited));
  const scale = Math.max(1, Math.abs(unconstrained), Math.abs(bounded));
  return {
    unconstrained,
    bounded,
    fixedTarget: Math.abs(bounded - unconstrained) > 1e-12 * scale ? bounded : null,
  };
}

/**
 * Activate a rail/slew branch only after Newton has first found either the
 * unconstrained pole solution or the constrained solution itself. This is a
 * piecewise-linear active-set test: it avoids guessing from the unsolved
 * iteration-0 feedback error, while keeping an active limiter stable on later
 * iterations of the same solve.
 */
function opAmpNewtonFixedTarget(
  drive: { unconstrained: number; bounded: number; fixedTarget: number | null },
  internalVoltageGuess: number,
): number | null {
  if (drive.fixedTarget === null) return null;
  const scale = Math.max(
    1,
    Math.abs(drive.unconstrained),
    Math.abs(drive.bounded),
    Math.abs(internalVoltageGuess),
  );
  const tolerance = 1e-8 * scale;
  const onUnconstrainedBranch = Math.abs(
    internalVoltageGuess - drive.unconstrained,
  ) <= tolerance;
  const onLimitedBranch = Math.abs(internalVoltageGuess - drive.bounded) <= tolerance;
  return onUnconstrainedBranch || onLimitedBranch ? drive.bounded : null;
}

/**
 * Signed branch-current constraint for the present Newton stamp. A committed
 * regime controls hysteresis, but the Newton active set may enter or leave a
 * current branch within this solve. A branch is admissible only while both
 * complementarity conditions hold:
 *
 *   - the voltage-source branch still demands (approximately) the limit, and
 *   - the constrained output remains on the overload side of the voltage
 *     source's compliance point, Vtarget + Rout * Ilimit.
 *
 * This prevents both failure directions: a newly applied hard load cannot
 * expose one accepted over-current voltage-source step, and a removed load
 * cannot make fixed current drive the numerical 1 TΩ node reference toward a
 * gigavolt iterate. Entry/recovery state itself is committed only after the
 * nonlinear solve converges.
 */
function opAmpCurrentConstraint(
  committedMode: number,
  currentGuess: number,
  outputVoltageGuess: number,
  voltageTarget: number,
  model: OpAmpModelParams,
  powered: boolean,
): number | null {
  if (!powered) return null;
  const candidateMode = opAmpCurrentLimitMode(
    committedMode,
    currentGuess,
    model.sourceCurrentLimit,
    model.sinkCurrentLimit,
  );
  if (candidateMode === -1) {
    const constraint = -model.sourceCurrentLimit;
    const complianceVoltage = voltageTarget + model.outputResistance * constraint;
    const voltageTolerance = 1e-8 * Math.max(
      1,
      Math.abs(outputVoltageGuess),
      Math.abs(complianceVoltage),
    );
    const constrainedVoltageIsAdmissible =
      outputVoltageGuess <= complianceVoltage + voltageTolerance;
    if (constrainedVoltageIsAdmissible) return constraint;
  }
  if (candidateMode === 1) {
    const constraint = model.sinkCurrentLimit;
    const complianceVoltage = voltageTarget + model.outputResistance * constraint;
    const voltageTolerance = 1e-8 * Math.max(
      1,
      Math.abs(outputVoltageGuess),
      Math.abs(complianceVoltage),
    );
    const constrainedVoltageIsAdmissible =
      outputVoltageGuess >= complianceVoltage - voltageTolerance;
    if (constrainedVoltageIsAdmissible) return constraint;
  }
  return null;
}

/**
 * Return an op-amp output branch to the package rail that supplied it.
 * `stampOpAmp` defines x[k] as current entering the output stage: negative
 * means sourcing (energy comes from V+), positive means sinking (energy is
 * returned to V-). Routing follows an active current constraint immediately;
 * voltage operation uses the last accepted sign until the next accepted step.
 */
function stampOpAmpSupplyReturn(
  ctx: DeviceContext,
  comp: DeviceComponent,
  k: number,
  power: IcPowerInfo,
  committedCurrent: number,
): void {
  // Connectivity and supply validity must come from this Newton iterate.
  // A committed powered state can outlive a topology edit; stamping its
  // output-current return into a newly isolated rail would create charge and
  // drive that rail toward the numerical shunt's teravolt scale.
  if (!power.powered || !power.vccPin || !power.gndPin) return;
  const railPin = committedCurrent > 0 ? power.gndPin : power.vccPin;
  const railNode = ctx.pinNode(comp.id, railPin);
  if (railNode >= 0) ctx.mna.add(railNode, k, -1);
}

/** Stamp a package's typical quiescent current as a physical V+ -> V- load. */
function stampIcQuiescentCurrent(
  ctx: DeviceContext,
  comp: DeviceComponent,
  power: IcPowerInfo,
  currentA: number,
): void {
  // Use present connectivity/power, never the previous accepted supply.
  // This keeps a hot-reloaded, disconnected V+ pin genuinely open.
  if (!power.powered || !power.vccPin || !power.gndPin || currentA <= 0) return;
  const vccNode = ctx.pinNode(comp.id, power.vccPin);
  const gndNode = ctx.pinNode(comp.id, power.gndPin);
  // stampCurrentSource injects into its first node. Injecting into V- and
  // removing from V+ is a load drawing current from the positive supply.
  stampCurrentSource(ctx.mna, gndNode, vccNode, currentA);
}

/** Declared operating window for the behavioural buck module. */
function dcdcInputRange(
  ctx: DeviceContext,
  comp: DeviceComponent,
): { min: number; max: number } {
  const declared = ctx.electricalSpecs(comp)?.vcc_range;
  const rawMin = Number(comp.params.vinMin ?? declared?.min ?? 7);
  const rawMax = Number(comp.params.vinMax ?? declared?.max ?? 35);
  const min = Number.isFinite(rawMin) ? Math.max(0, rawMin) : 7;
  const max = Number.isFinite(rawMax) ? Math.max(min, rawMax) : Math.max(min, 35);
  return { min, max };
}

/**
 * Averaged switch/inductor headroom for the identified non-isolated buck.
 * LM2596-class modules approach 100% duty cycle in dropout, but their switch,
 * diode, and inductor still leave a load-dependent voltage loss. The compact
 * teaching model uses an editable 1.5 V typical ceiling rather than silently
 * turning a step-down part into a boost converter.
 */
function dcdcBuckDropout(ctx: DeviceContext, comp: DeviceComponent): number | null {
  if (ctx.catalogPart(comp)?.uid !== "dcdc-buck-5v") return null;
  const raw = Number(ctx.modelParam(comp, "vdropout", 1.5));
  return Number.isFinite(raw) ? Math.max(0, raw) : 1.5;
}

/**
 * Shared element-current publisher for the two 3-terminal regulators — one
 * case body in the engine's _updateElementI (linear_reg, lm317), kept as
 * one body here.
 */
function publishRegulatorBranchCurrent(
  ctx: DeviceContext,
  comp: DeviceComponent,
  x: Float64Array,
): void {
  // Branch current x[k] is the delivered output current (in → out).
  const kRegI = ctx.vsrcRow(comp.id);
  if (kRegI !== undefined && kRegI < x.length) {
    ctx.setElementCurrent(comp.id, x[kRegI] ?? 0);
  } else {
    ctx.setElementCurrent(comp.id, 0);
  }
}

/**
 * Small-signal stamp for one op-amp unit, linearized at its COMMITTED
 * operating regime exactly like SPICE linearizes at the OP region:
 *
 * - committed current limit: the transient row is x[k] = const, so its
 *   derivative is dI = 0 — the output stage contributes no admittance and no
 *   drive passes through it.
 * - rail saturation / unpowered clamp: the transient row is
 *   Vout - Rout*x[k] = target, where the target is the supplying rail NODE
 *   voltage minus a constant headroom — vHigh/vLow are re-read from the
 *   package rails every Newton iterate, so the committed point tracks that
 *   rail with derivative 1. The AC row therefore carries -1 on the rail
 *   node column: dVout - Rout*dI - dVrail = 0. Inputs stay uncoupled (gain
 *   0), but rail ripple passes straight through the pinned stage.
 * - linear region: the dominant-pole gain evaluated at s = j*omega,
 *
 *     A(j*omega) = A0 / (1 + j*omega/wp),  wp = 2*pi*GBW/A0
 *
 *   which is the continuous-frequency response of the exact first-order ODE
 *   the transient backward-Euler companion integrates (see
 *   opAmpDominantPoleCompanion). The row is
 *   dVout - Rout*dI - A(j*omega)*(dV+ - dV-) = 0.
 *   For the LM386's mid-supply self-bias (midSupplyBias), the transient
 *   offset (Vcc+Vgnd)/2 also enters through the pole's offsetGain — its
 *   continuous limit is A(j*omega)/A0 — so the linear row additionally
 *   carries -(A(j*omega)/A0)/2 on each rail node column.
 *   The large-signal slew limiter has no small-signal contribution: at an
 *   operating point in the linear region the slew constraint is inactive.
 *
 * The branch current's KCL coupling into the output node and its return into
 * the supplying rail (committed current sign, mirroring
 * stampOpAmpSupplyReturn) are stamped in every regime so AC rail currents
 * stay conservative.
 */
function acStampOpAmpUnit(
  ctx: AcDeviceContext,
  comp: DeviceComponent,
  ac: AcStampSurface,
  omega: number,
  model: OpAmpModelParams,
  power: IcPowerInfo,
  rowKey: string,
  inMinusPin: string,
  inPlusPin: string,
  outPin: string,
  committedRegime: number,
  committedLimit: number,
  committedCurrent: number,
  midSupplyBias = false,
): void {
  const k = ctx.vsrcRow(rowKey);
  if (k === undefined) return;
  const outNode = ctx.pinNode(comp.id, outPin);
  const inMinus = ctx.pinNode(comp.id, inMinusPin);
  const inPlus = ctx.pinNode(comp.id, inPlusPin);
  // Rail node columns for the target coupling below. A package without
  // explicit supply pins has CONSTANT fallback rails (fallbackV / 0), which
  // correctly contribute nothing (pinNode of a null pin is never consulted).
  const vccNode = power.vccPin ? ctx.pinNode(comp.id, power.vccPin) : -1;
  const gndNode = power.gndPin ? ctx.pinNode(comp.id, power.gndPin) : -1;
  ac.addAc(outNode, k, 1, 0);
  if (power.powered && power.vccPin && power.gndPin) {
    const railPin = committedCurrent > 0 ? power.gndPin : power.vccPin;
    const railNode = ctx.pinNode(comp.id, railPin);
    if (railNode >= 0) ac.addAc(railNode, k, -1, 0);
  }

  if (power.powered && committedLimit !== 0) {
    ac.addAc(k, k, 1, 0);
    return;
  }

  ac.addAc(k, outNode, 1, 0);
  if (model.outputResistance > 0) ac.addAc(k, k, -model.outputResistance, 0);

  const regime = Number.isFinite(committedRegime) ? committedRegime : 0;
  if (!power.powered) {
    // Unpowered clamp target is power.gnd — the gnd NODE voltage when the
    // package has a wired gnd pin, a constant 0 otherwise.
    ac.addAc(k, gndNode, -1, 0);
    return;
  }
  if (regime === 1) {
    ac.addAc(k, vccNode, -1, 0);
    return;
  }
  if (regime === 2) {
    ac.addAc(k, gndNode, -1, 0);
    return;
  }

  const A0 = model.openLoopGain;
  let aRe = A0;
  let aIm = 0;
  if (model.gbwHz > 0) {
    // omega/wp with wp = 2*pi*GBW/A0; a non-positive GBW keeps the
    // historical instantaneous (flat) VCVS, matching the transient stamp.
    const ratio = omega / ((2 * Math.PI * model.gbwHz) / A0);
    const den = 1 + ratio * ratio;
    aRe = A0 / den;
    aIm = -(A0 * ratio) / den;
  }
  ac.addAc(k, inPlus, -aRe, -aIm);
  ac.addAc(k, inMinus, aRe, aIm);
  if (midSupplyBias) {
    // d(offset) = (dVcc + dVgnd)/2 through the pole's offset transfer
    // A(j*omega)/A0 (offsetGain's continuous limit; exactly 1 for the
    // instantaneous gbw <= 0 VCVS, matching the transient stamp).
    const offRe = aRe / (2 * A0);
    const offIm = aIm / (2 * A0);
    ac.addAc(k, vccNode, -offRe, -offIm);
    ac.addAc(k, gndNode, -offRe, -offIm);
  }
}

export const dcdcConverterModel: DeviceModel = {
  kinds: ["dcdc_converter"],
  // W5.2 — DC-DC converter: one branch row for the output side.
  // The output is a regulated source (REG or CC) identical in structure to the
  // linear_reg stamp.  The input side is a current source (no branch row).
  // The row is never all-zero: REG/CC both write a non-zero entry on row k_out.
  branchRows: (comp, _ctx) => [comp.id],
  stamp: (ctx, comp, xGuess, _h) => {
    // W5.2 — DC-DC converter module, 4-pin: in_pos, in_neg, out_pos, out_neg.
    //
    // OUTPUT SIDE: regulated source out_pos → out_neg at vout while powered.
    //   Four regimes for the identified buck:
    //   REG (0): V(out_pos) − V(out_neg) = vout  (ideal voltage source)
    //   DROP(1): Vout = Vin − vdropout             (averaged max-duty ceiling)
    //   CC  (2): x[k_out] = −iLimit              (floats below compliance)
    //   OFF (3): x[k_out] = 0                    (output is not an energy source)
    //
    //   The accepted regime seeds a monotonic per-solve active set. A
    //   provisional REG overcurrent enters CC in this solve; a provisional
    //   CC overvoltage selects voltage compliance in this solve. Monotonic
    //   selection avoids the historical REG/CC iteration chatter.
    //
    // INPUT SIDE: current source drawing Iin from in_pos → in_neg.
    // The identified buck and every CC transition couple present-solve
    // candidates as Iin=Pout/(eta·Vin), so no accepted load/headroom step
    // can create or lose a frame of energy. Unidentified legacy behavioural
    // converters retain their prior-step input-power compatibility path.
    const pins = comp.pins;
    if (pins.length < 4) return;
    const kDcdc = ctx.vsrcRow(comp.id);
    if (kDcdc === undefined) return;

    const stDcdc = ctx.state.icState.get(comp.id) ?? { reg: 3, iin: 0 };
    const iLimitDcdc = Math.max(0, Number(comp.params.iLimit ?? 2.0));
    const voutDcdc   = Math.max(0, Number(comp.params.vout ?? 5.0));
    const buckDropoutDcdc = dcdcBuckDropout(ctx, comp);

    // Node indices for the 4 pins (catalog order: in_pos, in_neg, out_pos, out_neg).
    const nInPos  = ctx.pinNode(comp.id, "in_pos");
    const nInNeg  = ctx.pinNode(comp.id, "in_neg");
    const nOutPos = ctx.pinNode(comp.id, "out_pos");
    const nOutNeg = ctx.pinNode(comp.id, "out_neg");

    // UVLO / operating-range gate. Use the Newton guess so removing or
    // reversing the input disables the source in the same solve instead
    // of leaking one committed timestep of impossible output energy.
    const vinGuess = ctx.vAt(xGuess, nInPos) - ctx.vAt(xGuess, nInNeg);
    const inputRange = dcdcInputRange(ctx, comp);
    const inputPowered = Number.isFinite(vinGuess)
      && vinGuess >= inputRange.min
      && vinGuess <= inputRange.max;
    const committedReg = stDcdc.reg ?? 3;
    const regDcdcCommitted = inputPowered
      ? (committedReg === 3 ? 0 : committedReg)
      : 3;
    const committedIinDcdc = inputPowered && Number.isFinite(stDcdc.iin)
      ? Math.max(0, stDcdc.iin as number)
      : 0;

    const requiredBuckVoltageRegDcdc: 0 | 1 = buckDropoutDcdc !== null
      && vinGuess < voutDcdc + buckDropoutDcdc
      ? 1
      : 0;
    const voltageRegDcdc = buckDropoutDcdc !== null
      && inputPowered
      && regDcdcCommitted !== 2
      ? ctx.regulatorHeadroomRegime(
        `dcdc_headroom:${comp.id}`,
        regDcdcCommitted,
        vinGuess,
        voutDcdc,
        buckDropoutDcdc,
      )
      : regDcdcCommitted;
    const complianceVoltageDcdc = buckDropoutDcdc !== null
      ? Math.min(voutDcdc, Math.max(0, vinGuess - buckDropoutDcdc))
      : voutDcdc;

    // Output-side KCL coupling (both REG and CC regimes).
    // x[k_out] is the output branch current; +1 at out_pos means current is
    // DELIVERED to out_pos (leaves out_pos into the circuit → positive sense).
    if (nOutPos >= 0) ctx.mna.add(nOutPos, kDcdc,  1);
    if (nOutNeg >= 0) ctx.mna.add(nOutNeg, kDcdc, -1);

    const activeSetKeyDcdc = `dcdc_converter:${comp.id}`;
    const entryClampDcdc = voltageRegDcdc !== 2 && voltageRegDcdc !== 3
      && ctx.useCurrentLimitEntryClamp(
        activeSetKeyDcdc,
        xGuess[kDcdc] ?? 0,
        iLimitDcdc,
      );
    const currentLimitedDcdc = voltageRegDcdc === 2 || entryClampDcdc;
    const outputMagnitudeGuess = ctx.vAt(xGuess, nOutPos) - ctx.vAt(xGuess, nOutNeg);
    const complianceClamp = currentLimitedDcdc && ctx.useCurrentLimitComplianceClamp(
      activeSetKeyDcdc,
      outputMagnitudeGuess,
      complianceVoltageDcdc,
    );
    const regDcdc = complianceClamp
      ? requiredBuckVoltageRegDcdc
      : currentLimitedDcdc
        ? 2
        : voltageRegDcdc;
    const etaDcdc = Math.max(0.01, Math.min(1, Number(comp.params.eta ?? 0.85)));
    // CC/compliance Newton candidates carry their output voltage/current
    // in xGuess. Couple their bounded output power back to the input in
    // this same solve. Entry initially uses the maximum compliance power;
    // later iterations settle to Pout/(eta·Vin), so the accepted result
    // neither creates output energy nor retains stale full-CC input draw.
    const activeSetPowerCoupling = inputPowered
      && (currentLimitedDcdc || buckDropoutDcdc !== null);
    const boundedOutputVoltageGuess = Math.min(
      Math.max(0, outputMagnitudeGuess),
      complianceVoltageDcdc,
    );
    const boundedOutputCurrentGuess = Math.min(
      Math.abs(xGuess[kDcdc] ?? 0),
      iLimitDcdc,
    );
    const iinDcdc = activeSetPowerCoupling
      ? boundedOutputVoltageGuess * boundedOutputCurrentGuess / (etaDcdc * vinGuess)
      : committedIinDcdc;

    if (regDcdc === 3) {
      // OFF: retain a well-posed branch row but deliver exactly zero
      // current. The output terminals are then passive/floating and the
      // connected load determines their voltage.
      ctx.mna.add(kDcdc, kDcdc, 1);
    } else if (regDcdc === 2) {
      // CC mode: pin x[k] = −iLimit (negative = source delivers iLimit into out_pos).
      // Same convention as bench_psu CC and linear_reg CC.
      ctx.mna.add(kDcdc, kDcdc, 1);
      ctx.mna.addB(kDcdc, -iLimitDcdc);
    } else if (regDcdc === 1 && buckDropoutDcdc !== null) {
      // Averaged maximum-duty dropout:
      // (Vout+ − Vout−) − (Vin+ − Vin−) = −Vdropout.
      if (nOutPos >= 0) ctx.mna.add(kDcdc, nOutPos,  1);
      if (nOutNeg >= 0) ctx.mna.add(kDcdc, nOutNeg, -1);
      if (nInPos >= 0) ctx.mna.add(kDcdc, nInPos, -1);
      if (nInNeg >= 0) ctx.mna.add(kDcdc, nInNeg,  1);
      ctx.mna.addB(kDcdc, -buckDropoutDcdc);
    } else {
      // REG mode: V(out_pos) − V(out_neg) = vout.
      if (nOutPos >= 0) ctx.mna.add(kDcdc, nOutPos,  1);
      if (nOutNeg >= 0) ctx.mna.add(kDcdc, nOutNeg, -1);
      ctx.mna.addB(kDcdc, voutDcdc);
    }

    // Input-side current source: draw iinDcdc from in_pos → in_neg.
    // stampCurrentSource injects current INTO in_pos and OUT of in_neg,
    // but we want to DRAW current (load the input supply), so the signs flip:
    // current flows IN_NEG → IN_POS through the source means the external
    // supply must push current from in_pos → in_neg (drawing from the supply).
    // Stamp as −Iin into in_pos (i.e. current leaving in_pos = Iin drawn from supply).
    // Standard: stampCurrentSource(i, j, I) injects I into i, removes from j.
    // We inject iinDcdc into in_neg and remove from in_pos → draws from supply.
    if (inputPowered) {
      stampCurrentSource(ctx.mna, nInNeg, nInPos, iinDcdc);
    }
  },
  commitState: (ctx, comp, x, _h) => {
    // W5.2 — dcdc_converter commit: decide output regime and update committed Iin.
    const kDcdcU = ctx.vsrcRow(comp.id);
    if (kDcdcU === undefined) return;
    const stDcdcU = ctx.state.icState.get(comp.id) ?? { reg: 3, iin: 0 };
    const prevRegDcdcU = stDcdcU.reg ?? 3;

    // Read committed output current (x[k] negative when delivering → use abs).
    const iOutRaw = x[kDcdcU] ?? 0;
    const iOutDcdc = Math.abs(iOutRaw);

    const iLimitDcdcU = Math.max(0, Number(comp.params.iLimit ?? 2.0));
    const voutDcdcU   = Math.max(0, Number(comp.params.vout ?? 5.0));
    const etaDcdcU    = Math.max(0.01, Math.min(1.0, Number(comp.params.eta ?? 0.85)));
    const buckDropoutDcdcU = dcdcBuckDropout(ctx, comp);

    // Committed output voltage (differential across out_pos − out_neg).
    const nOutPosDcdc = ctx.pinNode(comp.id, "out_pos");
    const nOutNegDcdc = ctx.pinNode(comp.id, "out_neg");
    const vOutCommitted = Math.abs(
      ctx.vAt(x, nOutPosDcdc) - ctx.vAt(x, nOutNegDcdc),
    );

    // Committed input voltage (differential across in_pos − in_neg).
    const nInPosDcdc  = ctx.pinNode(comp.id, "in_pos");
    const nInNegDcdc  = ctx.pinNode(comp.id, "in_neg");
    const vinCommitted = ctx.vAt(x, nInPosDcdc) - ctx.vAt(x, nInNegDcdc);
    const inputRangeDcdc = dcdcInputRange(ctx, comp);
    const inputPoweredDcdc = Number.isFinite(vinCommitted)
      && vinCommitted >= inputRangeDcdc.min
      && vinCommitted <= inputRangeDcdc.max;
    const buckVoltageRegDcdcU: 0 | 1 = buckDropoutDcdcU !== null
      && vinCommitted < voutDcdcU + buckDropoutDcdcU
      ? 1
      : 0;
    const complianceVoltageDcdcU = buckDropoutDcdcU !== null
      ? Math.min(voutDcdcU, Math.max(0, vinCommitted - buckDropoutDcdcU))
      : voutDcdcU;

    // Output-regime state machine (no latch — CC relaxes to REG when load eases).
    //   from REG: enter CC if |I_out| > iLimit×1.001.
    //   from CC:  exit to REG when V_out has recovered near the setpoint (hysteresis).
    let newRegDcdcU: number;
    if (!inputPoweredDcdc) {
      newRegDcdcU = 3;
    } else if (
      ctx.currentLimitEntryClampActive(`dcdc_converter:${comp.id}`)
      && !ctx.currentLimitComplianceClampActive(`dcdc_converter:${comp.id}`)
    ) {
      newRegDcdcU = 2;
    } else if (prevRegDcdcU === 2) {
      // In CC: exit when V_out has risen back near the attainable buck
      // compliance voltage (regulated setpoint or max-duty dropout ceiling).
      newRegDcdcU = vOutCommitted >= complianceVoltageDcdcU * 0.98
        ? buckVoltageRegDcdcU
        : 2;
    } else {
      // In REG: enter CC when load demands more than iLimit.
      newRegDcdcU = iOutDcdc > iLimitDcdcU * 1.001 ? 2 : buckVoltageRegDcdcU;
    }

    // Input current from committed output values.
    // Pout = Vout_committed × Iout_committed (always non-negative).
    // Iin  = Pout / (eta × max(Vin, 0.1))
    // This is constant across the Newton inner loop (held from committed values).
    const pOut  = inputPoweredDcdc ? vOutCommitted * iOutDcdc : 0;
    const iin   = inputPoweredDcdc ? pOut / (etaDcdcU * vinCommitted) : 0;

    ctx.state.icState.set(comp.id, { reg: newRegDcdcU, iin });
  },
  updateCurrent: (ctx, comp, x) => {
    // Report the output branch current (delivered to out_pos).
    // x[k_out] is negative when delivering (MNA convention); take abs for
    // reporting, matching the linear_reg convention.
    const kDcdcI = ctx.vsrcRow(comp.id);
    if (kDcdcI !== undefined && kDcdcI < x.length) {
      ctx.setElementCurrent(comp.id, x[kDcdcI] ?? 0);
    } else {
      ctx.setElementCurrent(comp.id, 0);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same regime treatment as linear_reg, on the output differential pair:
    // REG holds d(Vout+ - Vout-) = 0, the identified buck's DROPOUT ceiling
    // holds d(Vout_diff - Vin_diff) = 0, CC/OFF pin dI = 0. HONEST
    // IDEALIZATION: the averaged compact model has no control-loop
    // bandwidth, and the input side is treated as an AC open. That is a
    // deliberate simplification, not a stamp derivative: the legacy
    // behavioural converter does hold Iin at its committed value, but the
    // identified buck (and every CC transition) recouples
    // Iin = Pout/(eta*Vin) to the live iterate in the transient stamp, whose
    // derivative would be the buck's negative incremental input resistance
    // plus input-to-output coupling (audio susceptibility). Both are
    // deliberately outside this compact AC model — the derivative is also
    // ambiguous at the REG fixed point, which sits exactly on the
    // bounded-output clamp corner.
    const pins = comp.pins;
    if (pins.length < 4) return;
    const k = ctx.vsrcRow(comp.id);
    if (k === undefined) return;
    const st = ctx.state.icState.get(comp.id);
    const regRaw = st ? (st.reg ?? 3) : 3;
    const reg = Number.isFinite(regRaw) ? regRaw : 3;
    const nInPos = ctx.pinNode(comp.id, "in_pos");
    const nInNeg = ctx.pinNode(comp.id, "in_neg");
    const nOutPos = ctx.pinNode(comp.id, "out_pos");
    const nOutNeg = ctx.pinNode(comp.id, "out_neg");
    ac.addAc(nOutPos, k, 1, 0);
    ac.addAc(nOutNeg, k, -1, 0);
    if (reg === 0) {
      ac.addAc(k, nOutPos, 1, 0);
      ac.addAc(k, nOutNeg, -1, 0);
    } else if (reg === 1) {
      ac.addAc(k, nOutPos, 1, 0);
      ac.addAc(k, nOutNeg, -1, 0);
      ac.addAc(k, nInPos, -1, 0);
      ac.addAc(k, nInNeg, 1, 0);
    } else {
      ac.addAc(k, k, 1, 0);
    }
  },
};

export const dualOpAmpModel: DeviceModel = {
  kinds: ["lm358", "mcp6002"],
  // Dual op-amp: one VCVS branch row per unit.  Always allocated — the
  // stampOpAmp row is self-consistent (V_out coefficient always present)
  // so the matrix is never singular even when the part is unpowered.
  branchRows: (comp, _ctx) => [`${comp.id}:op0`, `${comp.id}:op1`],
  stamp: (ctx, comp, xGuess, h) => {
    // W4.1 — Dual op-amp.  Units share VCC (pin 8) and GND (pin 4).
    // Pinout (verified against TI LM358 datasheet / Microchip MCP6002):
    //   1=OUT1, 2=IN1-, 3=IN1+, 4=GND(V-), 5=IN2+, 6=IN2-, 7=OUT2, 8=VCC(V+)
    //
    // The active-set test first admits the unconstrained dominant-pole
    // solution, then activates slew/rail bounds only when that solution
    // reaches a bound. This avoids guessing from iteration-0 feedback error.
    // Output-current clamp entry/recovery remains accepted-step state.
    const powerDual = ctx.icPowerInfo(comp, xGuess);
    // LM358: output swings to within ~1.5 V of VCC (not rail-to-rail).
    // MCP6002: rail-to-rail output (within ~20 mV of each rail).
    const rtrDual = comp.kind === "mcp6002";
    const vHighDual = rtrDual
      ? powerDual.vcc - 0.02
      : powerDual.vcc - 1.5;
    const vLowDual = powerDual.gnd + 0.02;
    const modelDual = opAmpModelParams(comp);
    const stDual = ctx.state.icState.get(comp.id) ?? defaultIcState(comp.kind);
    // load() reserves a 1 ps solve for the DC operating point. It must
    // initialise genuinely new analogue state directly, not spend
    // simulated microseconds slewing from zero before an AC analysis.
    // A live reload carries finite u1/u2 and therefore keeps full dynamics.
    const stepModelDual = h <= 1e-12
      && !Number.isFinite(stDual.u1)
      && !Number.isFinite(stDual.u2)
      ? { ...modelDual, gbwHz: 0, slewRate: 0 }
      : modelDual;
    stampIcQuiescentCurrent(
      ctx,
      comp,
      powerDual,
      analogIcQuiescentCurrent(comp),
    );

    // Unit 1: OUT1=pin1, IN1-=pin2, IN1+=pin3
    const k0 = ctx.vsrcRow(`${comp.id}:op0`);
    if (k0 !== undefined) {
      const inMinus = ctx.pinNode(comp.id, "2");
      const inPlus = ctx.pinNode(comp.id, "3");
      const outNode = ctx.pinNode(comp.id, "1");
      const uPrev = Number.isFinite(stDual.u1) ? stDual.u1 : powerDual.gnd;
      const drive = opAmpStepTarget(
        uPrev,
        ctx.vAt(xGuess, inPlus) - ctx.vAt(xGuess, inMinus),
        stepModelDual,
        0,
        h,
        vLowDual,
        vHighDual,
      );
      const internalGuess = ctx.vAt(xGuess, outNode)
        - modelDual.outputResistance * (xGuess[k0] ?? 0);
      const fixedTarget = powerDual.powered
        ? opAmpNewtonFixedTarget(drive, internalGuess)
        : powerDual.gnd;
      const currentConstraint = opAmpCurrentConstraint(
        stDual.limit1 ?? 0,
        xGuess[k0] ?? 0,
        ctx.vAt(xGuess, outNode),
        drive.bounded,
        modelDual,
        powerDual.powered,
      );
      stampOpAmp(
        ctx.mna,
        inMinus,
        inPlus,
        outNode,
        k0,
        modelDual.openLoopGain,
        0,
        fixedTarget,
        modelDual.outputResistance,
        {
          h,
          gbwHz: stepModelDual.gbwHz,
          previousInternalVoltage: uPrev,
          currentConstraint,
        },
      );
      stampOpAmpSupplyReturn(
        ctx,
        comp,
        k0,
        powerDual,
        currentConstraint ?? stDual.i1 ?? 0,
      );
    }

    // Unit 2: OUT2=pin7, IN2-=pin6, IN2+=pin5
    const k1 = ctx.vsrcRow(`${comp.id}:op1`);
    if (k1 !== undefined) {
      const inMinus = ctx.pinNode(comp.id, "6");
      const inPlus = ctx.pinNode(comp.id, "5");
      const outNode = ctx.pinNode(comp.id, "7");
      const uPrev = Number.isFinite(stDual.u2) ? stDual.u2 : powerDual.gnd;
      const drive = opAmpStepTarget(
        uPrev,
        ctx.vAt(xGuess, inPlus) - ctx.vAt(xGuess, inMinus),
        stepModelDual,
        0,
        h,
        vLowDual,
        vHighDual,
      );
      const internalGuess = ctx.vAt(xGuess, outNode)
        - modelDual.outputResistance * (xGuess[k1] ?? 0);
      const fixedTarget = powerDual.powered
        ? opAmpNewtonFixedTarget(drive, internalGuess)
        : powerDual.gnd;
      const currentConstraint = opAmpCurrentConstraint(
        stDual.limit2 ?? 0,
        xGuess[k1] ?? 0,
        ctx.vAt(xGuess, outNode),
        drive.bounded,
        modelDual,
        powerDual.powered,
      );
      stampOpAmp(
        ctx.mna,
        inMinus,
        inPlus,
        outNode,
        k1,
        modelDual.openLoopGain,
        0,
        fixedTarget,
        modelDual.outputResistance,
        {
          h,
          gbwHz: stepModelDual.gbwHz,
          previousInternalVoltage: uPrev,
          currentConstraint,
        },
      );
      stampOpAmpSupplyReturn(
        ctx,
        comp,
        k1,
        powerDual,
        currentConstraint ?? stDual.i2 ?? 0,
      );
    }
  },
  commitState: (ctx, comp, x, h) => {
    // W4.1 op-amps: commit the internal dominant-pole voltage, signed output
    // current-limit regime, and rail state once per accepted solve. icState is
    // snapshot/rollback state, so rejected adaptive steps cannot leak analogue
    // memory into the retry.
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState(comp.kind);
    const power = ctx.icPowerInfo(comp, x);
    const vHigh = (comp.kind === "mcp6002" ? power.vcc - 0.02 : power.vcc - 1.5);
    const vLow = power.gnd + 0.02;
    const model = opAmpModelParams(comp);
    const d1 = ctx.vAt(x, ctx.pinNode(comp.id, "3")) - ctx.vAt(x, ctx.pinNode(comp.id, "2"));
    const d2 = ctx.vAt(x, ctx.pinNode(comp.id, "5")) - ctx.vAt(x, ctx.pinNode(comp.id, "6"));
    const u1Prev = Number.isFinite(st.u1) ? st.u1 : power.gnd;
    const u2Prev = Number.isFinite(st.u2) ? st.u2 : power.gnd;
    const k0 = ctx.vsrcRow(`${comp.id}:op0`);
    const k1 = ctx.vsrcRow(`${comp.id}:op1`);
    const i1 = k0 !== undefined ? (x[k0] ?? 0) : 0;
    const i2 = k1 !== undefined ? (x[k1] ?? 0) : 0;
    const output1 = ctx.vAt(x, ctx.pinNode(comp.id, "1"));
    const output2 = ctx.vAt(x, ctx.pinNode(comp.id, "7"));
    const drive1 = opAmpStepTarget(
      u1Prev,
      d1,
      model,
      0,
      h,
      vLow,
      vHigh,
    );
    const drive2 = opAmpStepTarget(
      u2Prev,
      d2,
      model,
      0,
      h,
      vLow,
      vHigh,
    );
    const currentConstraint1 = opAmpCurrentConstraint(
      st.limit1 ?? 0,
      i1,
      output1,
      drive1.bounded,
      model,
      power.powered,
    );
    const currentConstraint2 = opAmpCurrentConstraint(
      st.limit2 ?? 0,
      i2,
      output2,
      drive2.bounded,
      model,
      power.powered,
    );
    const u1 = !power.powered
      ? power.gnd
      : currentConstraint1 !== null
        ? drive1.bounded
        : output1 - model.outputResistance * i1;
    const u2 = !power.powered
      ? power.gnd
      : currentConstraint2 !== null
        ? drive2.bounded
        : output2 - model.outputResistance * i2;
    const limit1 = currentConstraint1 === null
      ? 0
      : currentConstraint1 < 0 ? -1 : 1;
    const limit2 = currentConstraint2 === null
      ? 0
      : currentConstraint2 < 0 ? -1 : 1;
    ctx.state.icState.set(comp.id, {
      reg1: u1 >= vHigh ? 1 : u1 <= vLow ? 2 : 0,
      d1, u1, limit1, i1,
      reg2: u2 >= vHigh ? 1 : u2 <= vLow ? 2 : 0,
      d2, u2, limit2, i2,
      supplyV: power.vSupply,
    });
  },
  updateCurrent: (ctx, comp, x) => {
    // Report output current for unit 1 from the VCVS branch solution.
    // The branch row k0 gives i_out1; unit 2 (k1) is available but we
    // report unit 1 only as a representative scalar for the elementI map.
    // Per-unit channel currents are not yet exposed via getElementChannelI.
    const k0I = ctx.vsrcRow(`${comp.id}:op0`);
    if (k0I !== undefined && k0I < x.length) {
      ctx.setElementCurrent(comp.id, x[k0I] ?? 0);
    } else {
      ctx.setElementCurrent(comp.id, 0);
    }
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Both units linearized at their committed regimes (see acStampOpAmpUnit).
    // The quiescent-current source is constant, so it has no AC contribution.
    const power = ctx.icPowerInfoAtOp(comp);
    const model = opAmpModelParams(comp);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState(comp.kind);
    acStampOpAmpUnit(
      ctx, comp, ac, omega, model, power,
      `${comp.id}:op0`, "2", "3", "1",
      st.reg1 ?? 0, st.limit1 ?? 0, st.i1 ?? 0,
    );
    acStampOpAmpUnit(
      ctx, comp, ac, omega, model, power,
      `${comp.id}:op1`, "6", "5", "7",
      st.reg2 ?? 0, st.limit2 ?? 0, st.i2 ?? 0,
    );
  },
};

export const lm386Model: DeviceModel = {
  kinds: ["lm386"],
  // Single-unit audio power amp: one VCVS branch row.
  branchRows: (comp, _ctx) => [`${comp.id}:op0`],
  stamp: (ctx, comp, xGuess, h) => {
    // W4.1 — Audio power amplifier, single unit, fixed gain.
    // Pinout (verified against TI LM386 datasheet):
    //   1=GAIN, 2=-IN(INV), 3=+IN(NON-INV), 4=GND, 5=VOUT, 6=VS(VCC),
    //   7=BYPASS, 8=GAIN
    // BEHAVIORAL transient model: a dominant-pole gain stage centred on
    // Vmid, followed by slew, rail, output-resistance, and current clamps.
    // Vmid = (vcc+gnd)/2; gain param selects 20/50/200 (open/ext cap/shorted).
    // It is deliberately lumped; speaker impedance and distortion remain
    // outside this teaching model (documented in the catalog).
    const powerLM386 = ctx.icPowerInfo(comp, xGuess);
    const gain386 = Math.max(1, Number(comp.params.gain ?? 20));
    const modelLM386 = opAmpModelParams(comp, gain386);
    const stLM386 = ctx.state.icState.get(comp.id) ?? defaultIcState("lm386");
    const stepModelLM386 = h <= 1e-12 && !Number.isFinite(stLM386.u1)
      ? { ...modelLM386, gbwHz: 0, slewRate: 0 }
      : modelLM386;
    const vmid386 = (powerLM386.vcc + powerLM386.gnd) / 2;
    // LM386 output headroom: ~0.5 V from each rail (datasheet Vos max ~50 mV,
    // but rail headroom is roughly 0.5 V at rated current load).
    const vHigh386 = powerLM386.vcc - 0.5;
    const vLow386 = powerLM386.gnd + 0.5;
    stampIcQuiescentCurrent(
      ctx,
      comp,
      powerLM386,
      analogIcQuiescentCurrent(comp),
    );

    const k386 = ctx.vsrcRow(`${comp.id}:op0`);
    if (k386 !== undefined) {
      const inMinus = ctx.pinNode(comp.id, "2");
      const inPlus = ctx.pinNode(comp.id, "3");
      const outNode = ctx.pinNode(comp.id, "5");
      const uPrev = Number.isFinite(stLM386.u1) ? stLM386.u1 : vmid386;
      const drive = opAmpStepTarget(
        uPrev,
        ctx.vAt(xGuess, inPlus) - ctx.vAt(xGuess, inMinus),
        stepModelLM386,
        vmid386,
        h,
        vLow386,
        vHigh386,
      );
      const internalGuess = ctx.vAt(xGuess, outNode)
        - modelLM386.outputResistance * (xGuess[k386] ?? 0);
      const fixedTarget = powerLM386.powered
        ? opAmpNewtonFixedTarget(drive, internalGuess)
        : powerLM386.gnd;
      const currentConstraint = opAmpCurrentConstraint(
        stLM386.limit1 ?? 0,
        xGuess[k386] ?? 0,
        ctx.vAt(xGuess, outNode),
        drive.bounded,
        modelLM386,
        powerLM386.powered,
      );
      stampOpAmp(
        ctx.mna,
        inMinus,
        inPlus,
        outNode,
        k386,
        modelLM386.openLoopGain,
        vmid386,
        fixedTarget,
        modelLM386.outputResistance,
        {
          h,
          gbwHz: stepModelLM386.gbwHz,
          previousInternalVoltage: uPrev,
          currentConstraint,
        },
      );
      stampOpAmpSupplyReturn(
        ctx,
        comp,
        k386,
        powerLM386,
        currentConstraint ?? stLM386.i1 ?? 0,
      );
    }
  },
  commitState: (ctx, comp, x, h) => {
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("lm386");
    const power = ctx.icPowerInfo(comp, x);
    const gain = Math.max(1, Number(comp.params.gain ?? 20));
    const model = opAmpModelParams(comp, gain);
    const vmid = (power.vcc + power.gnd) / 2;
    const vHigh = power.vcc - 0.5;
    const vLow = power.gnd + 0.5;
    const d1 = ctx.vAt(x, ctx.pinNode(comp.id, "3")) - ctx.vAt(x, ctx.pinNode(comp.id, "2"));
    const uPrev = Number.isFinite(st.u1) ? st.u1 : vmid;
    const k = ctx.vsrcRow(`${comp.id}:op0`);
    const i1 = k !== undefined ? (x[k] ?? 0) : 0;
    const output = ctx.vAt(x, ctx.pinNode(comp.id, "5"));
    const drive = opAmpStepTarget(
      uPrev,
      d1,
      model,
      vmid,
      h,
      vLow,
      vHigh,
    );
    const currentConstraint = opAmpCurrentConstraint(
      st.limit1 ?? 0,
      i1,
      output,
      drive.bounded,
      model,
      power.powered,
    );
    const u1 = !power.powered
      ? power.gnd
      : currentConstraint !== null
        ? drive.bounded
        : output - model.outputResistance * i1;
    const limit1 = currentConstraint === null
      ? 0
      : currentConstraint < 0 ? -1 : 1;
    ctx.state.icState.set(comp.id, {
      reg1: u1 >= vHigh ? 1 : u1 <= vLow ? 2 : 0,
      d1, u1, limit1, i1,
      supplyV: power.vSupply,
    });
  },
  updateCurrent: (ctx, comp, x) => {
    // Single-unit: branch current directly from the solution row.
    const k386I = ctx.vsrcRow(`${comp.id}:op0`);
    if (k386I !== undefined && k386I < x.length) {
      ctx.setElementCurrent(comp.id, x[k386I] ?? 0);
    } else {
      ctx.setElementCurrent(comp.id, 0);
    }
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Single fixed-gain unit. The mid-supply self-bias is NOT a small-signal
    // constant — it tracks the package rails at (Vcc+Vgnd)/2 — so the unit
    // stamp carries its rail coupling (midSupplyBias).
    const power = ctx.icPowerInfoAtOp(comp);
    const gain = Math.max(1, Number(comp.params.gain ?? 20));
    const model = opAmpModelParams(comp, gain);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("lm386");
    acStampOpAmpUnit(
      ctx, comp, ac, omega, model, power,
      `${comp.id}:op0`, "2", "3", "5",
      st.reg1 ?? 0, st.limit1 ?? 0, st.i1 ?? 0,
      true,
    );
  },
};

export const lm393Model: DeviceModel = {
  kinds: ["lm393"],
  stamp: (ctx, comp, xGuess, _h) => {
    // W4.2 — The LM393 is NOT a VCVS — it has no branch row.  Each unit reads
    // the analog input differential and controls an NPN open-collector output:
    //   - sinking (low):  ctx.stampDigitalOutput(..., low, power)
    //   - released (hi-Z): stamp nothing; external pull-up sets net voltage.
    //
    // stampDigitalOutput is used (not stampResistor) because GND is the
    // reference node (row -1 in MNA).  stampResistor(out, gnd, R) would use
    // gndNode=-1 and skip the off-diagonal cross-terms, giving wrong physics.
    // stampDigitalOutput uses power.gnd (the voltage, 0 V) as the Norton
    // target, which correctly sinks current through outputResistance to the
    // chip's V- rail regardless of whether GND is in the matrix.
    //
    // Stamp from COMMITTED state (prev timestep), not from xGuess, for
    // the same reason as the LM358: xGuess supply rows are unsolved on
    // Newton iteration 1 so input voltages chatter, flipping the output
    // state on every iteration and preventing convergence.
    const powerComp = ctx.icPowerInfo(comp, xGuess);
    if (!powerComp.powered) return;

    const stComp = ctx.state.icState.get(comp.id) ?? defaultIcState("lm393");

    // Unit 1: IN1+= pin3, IN1-= pin2, OUT1= pin1
    if (stComp.out1 === 0) {
      // Committed sinking: stamp Norton equivalent to chip GND through outputResistance.
      ctx.stampDigitalOutput(comp, "1", false, powerComp);
    }
    // Released (out1 === 1 or NaN): stamp nothing — hi-Z, pull-up drives the net.

    // Unit 2: IN2+= pin5, IN2-= pin6, OUT2= pin7
    if (stComp.out2 === 0) {
      ctx.stampDigitalOutput(comp, "7", false, powerComp);
    }
  },
  commitState: (ctx, comp, x, _h) => {
    // W4.2 — LM393 comparator: commit output state after solve (post-solve,
    // rails established).  The ±1 mV hysteresis band prevents chatter when the
    // input differential is near zero — same philosophy as the 555's threshold
    // hysteresis. Decision: released if diff > +hyst, sinking if diff < -hyst,
    // keep previous if inside the band.
    const LM393_HYST = 0.001; // 1 mV input-referred hysteresis
    const stLm393 = ctx.state.icState.get(comp.id) ?? defaultIcState("lm393");
    const powerLm393 = ctx.icPowerInfo(comp, x);
    if (!powerLm393.powered) {
      // Unpowered: output releases (hi-Z). No sink path when supply is absent.
      ctx.state.icState.set(comp.id, { out1: 1, out2: 1 });
    } else {
      // Unit 1: IN1+ = pin3, IN1- = pin2
      const d1 = ctx.vAt(x, ctx.pinNode(comp.id, "3")) - ctx.vAt(x, ctx.pinNode(comp.id, "2"));
      const prevOut1 = Number.isNaN(stLm393.out1) ? 1 : stLm393.out1;
      const newOut1 = d1 > LM393_HYST ? 1 : d1 < -LM393_HYST ? 0 : prevOut1;
      // Unit 2: IN2+ = pin5, IN2- = pin6
      const d2 = ctx.vAt(x, ctx.pinNode(comp.id, "5")) - ctx.vAt(x, ctx.pinNode(comp.id, "6"));
      const prevOut2 = Number.isNaN(stLm393.out2) ? 1 : stLm393.out2;
      const newOut2 = d2 > LM393_HYST ? 1 : d2 < -LM393_HYST ? 0 : prevOut2;
      ctx.state.icState.set(comp.id, { out1: newOut1, out2: newOut2 });
    }
  },
  updateCurrent: (ctx, comp, x) => {
    // Report unit-1 sink current when committed sinking.
    // I = (V_out1 − power.gnd) / outputResistance — mirrors _stampDigitalOutput math.
    const stLm393I = ctx.state.icState.get(comp.id) ?? defaultIcState("lm393");
    const powerLm393I = ctx.icPowerInfo(comp, x);
    if (stLm393I.out1 === 0 && powerLm393I.powered) {
      const vOut1 = ctx.vAt(x, ctx.pinNode(comp.id, "1"));
      ctx.setElementCurrent(comp.id, (vOut1 - powerLm393I.gnd) / powerLm393I.outputResistance);
    } else {
      ctx.setElementCurrent(comp.id, 0);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Committed comparator outputs: a sinking open-collector stage is its
    // Thevenin output resistance to the chip's V- rail (an AC ground through
    // the supply); a released output is hi-Z and stamps nothing. The input
    // pins are comparator sense inputs — no small-signal path exists from
    // them to the output stage (the decision is a committed latch), which is
    // exactly the transient stamp's structure.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("lm393");
    const g = 1 / power.outputResistance;
    const stampSink = (outPin: string): void => {
      const node = ctx.pinNode(comp.id, outPin);
      if (node < 0) return;
      const railPin = power.gndPin;
      if (railPin && !ctx.isOpenPin(comp.id, railPin)) {
        stampAcAdmittance(ac, node, ctx.pinNode(comp.id, railPin), g, 0);
      } else {
        // Local-reference Norton fallback: the fixed target voltage
        // differentiates away, leaving the self conductance only (mirrors
        // _stampDigitalOutput's supply-less branch).
        ac.addAc(node, node, g, 0);
      }
    };
    if (st.out1 === 0) stampSink("1");
    if (st.out2 === 0) stampSink("7");
  },
};

export const linearRegModel: DeviceModel = {
  kinds: ["linear_reg"],
  // W5.1 — 3-terminal linear regulator: one branch row per part.
  // The row is always allocated (the constraint is never all-zero in any regime:
  // REG/DROPOUT have +1 on V_out, CC has +1 on x[k] itself).
  branchRows: (comp, _ctx) => [comp.id],
  stamp: (ctx, comp, xGuess, _h) => {
    // W5.1 — Fixed-output LDO (e.g. 7805, AMS1117).  Pins: in / gnd / out.
    // The ref node is the gnd pin — constraint row enforces V_out − V_gnd = vout.
    // Regime is committed from the previous step; NaN triggers an initial pick.
    const pins = comp.pins;
    const kLR = ctx.vsrcRow(comp.id);
    if (kLR === undefined || pins.length < 3) return;
    const inNodeLR  = ctx.pinNode(comp.id, "in");
    const refNodeLR = ctx.pinNode(comp.id, "gnd");
    const outNodeLR = ctx.pinNode(comp.id, "out");
    const VregLR    = Number(comp.params.vout ?? 5.0);
    const vdropLR   = Number(comp.params.vdropout ?? 2.0);
    const iLimLR    = Number(comp.params.iLimit ?? 1.0);
    const stLR = ctx.state.icState.get(comp.id) ?? defaultIcState("linear_reg");
    // Accepted regime seeds a monotonic present-solve CC/compliance active set.
    const thermalShutdown = ctx.state.thermalDevices.get(comp.id)?.thermalShutdown === true;
    const regLR = thermalShutdown ? 3 : (Number.isFinite(stLR.reg) ? stLR.reg : 0);
    const activeSetKeyLR = `linear_reg:${comp.id}`;
    const vInGuessLR = ctx.vAt(xGuess, inNodeLR);
    const vRefGuessLR = ctx.vAt(xGuess, refNodeLR);
    const vOutGuessLR = ctx.vAt(xGuess, outNodeLR);
    const headroomGuessLR = vInGuessLR - vRefGuessLR;
    const headroomRegLR = !thermalShutdown && regLR !== 2
      ? ctx.regulatorHeadroomRegime(
        activeSetKeyLR,
        regLR,
        headroomGuessLR,
        VregLR,
        vdropLR,
      )
      : regLR;
    const entryClampLR = headroomRegLR !== 2 && headroomRegLR !== 3
      && ctx.useCurrentLimitEntryClamp(
        activeSetKeyLR,
        xGuess[kLR] ?? 0,
        iLimLR,
      );
    const currentRegLR = entryClampLR ? 2 : headroomRegLR;
    const complianceLR = Math.max(0, Math.min(VregLR, headroomGuessLR - vdropLR));
    const complianceClampLR = currentRegLR === 2 && ctx.useCurrentLimitComplianceClamp(
      activeSetKeyLR,
      vOutGuessLR - vRefGuessLR,
      complianceLR,
    );
    const stampRegLR = complianceClampLR
      ? (headroomGuessLR <= vdropLR ? 3 : headroomGuessLR < VregLR + vdropLR ? 1 : 0)
      : currentRegLR;
    stampLinearRegulator(ctx.mna, inNodeLR, refNodeLR, outNodeLR, kLR,
      stampRegLR, VregLR, vdropLR, iLimLR);
    const iqLR = regulatorQuiescentCurrent(comp);
    if (!thermalShutdown && stampRegLR !== 3 && iqLR > 0) {
      stampCurrentSource(ctx.mna, refNodeLR, inNodeLR, iqLR);
    }
  },
  commitState: (ctx, comp, x, _h) => {
    // W5.1 — linear regulators: commit regime once per step (post-solve, stable).
    // regulatorRegime runs here, NOT at stamp time — re-deriving from xGuess would
    // oscillate on Newton iteration 1 when supply node voltages are not yet solved.
    const kLRU = ctx.vsrcRow(comp.id);
    if (kLRU === undefined) return;
    const stLRU = ctx.state.icState.get(comp.id) ?? defaultIcState("linear_reg");
    const iLRU    = x[kLRU] ?? 0;
    const vInLRU  = ctx.vAt(x, ctx.pinNode(comp.id, "in"));
    const vRefLRU = ctx.vAt(x, ctx.pinNode(comp.id, "gnd"));
    const vOutLRU = ctx.vAt(x, ctx.pinNode(comp.id, "out"));
    const VregLRU   = Number(comp.params.vout ?? 5.0);
    const vdropLRU  = Number(comp.params.vdropout ?? 2.0);
    const iLimLRU   = Number(comp.params.iLimit ?? 1.0);
    const thermalShutdown = ctx.state.thermalDevices.get(comp.id)?.thermalShutdown === true;
    const activeSetKeyLRU = `linear_reg:${comp.id}`;
    const activeSetEnteredLRU = ctx.currentLimitEntryClampActive(activeSetKeyLRU)
      && !ctx.currentLimitComplianceClampActive(activeSetKeyLRU);
    const newRegLRU = thermalShutdown
      ? 3
      : activeSetEnteredLRU
        ? 2
      : regulatorRegime(stLRU.reg, iLRU, vInLRU, vRefLRU, vOutLRU,
          VregLRU, vdropLRU, iLimLRU);
    ctx.state.icState.set(comp.id, { reg: newRegLRU });
  },
  updateCurrent: (ctx, comp, x) => {
    publishRegulatorBranchCurrent(ctx, comp, x);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Linearized at the committed regime. HONEST IDEALIZATION: in REG the
    // compact regulator is an ideal servo — the branch-row constraint
    // becomes d(Vout - Vref) = 0, i.e. perfect regulation with no bandwidth,
    // output impedance, or ripple-rejection rolloff model (none exists in
    // the transient stamp either). DROPOUT passes the input through the
    // saturated pass device (fixed drop -> d(Vout - Vin) = 0); CC and OFF
    // pin the branch current, whose derivative is dI = 0 -> the output is
    // open through the regulator. The quiescent-current source is constant
    // and vanishes.
    const pins = comp.pins;
    const k = ctx.vsrcRow(comp.id);
    if (k === undefined || pins.length < 3) return;
    const inNode = ctx.pinNode(comp.id, "in");
    const refNode = ctx.pinNode(comp.id, "gnd");
    const outNode = ctx.pinNode(comp.id, "out");
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("linear_reg");
    const thermalShutdown = ctx.state.thermalDevices.get(comp.id)?.thermalShutdown === true;
    const reg = thermalShutdown ? 3 : (Number.isFinite(st.reg) ? st.reg : 0);
    // Branch-to-node coupling in every regime, mirroring stampLinearRegulator.
    ac.addAc(outNode, k, 1, 0);
    ac.addAc(inNode, k, -1, 0);
    if (reg === 0) {
      ac.addAc(k, outNode, 1, 0);
      ac.addAc(k, refNode, -1, 0);
    } else if (reg === 1) {
      ac.addAc(k, outNode, 1, 0);
      ac.addAc(k, inNode, -1, 0);
    } else {
      ac.addAc(k, k, 1, 0);
    }
  },
};

export const lm317Model: DeviceModel = {
  kinds: ["lm317"],
  // Same always-allocated branch-row rationale as linear_reg above.
  branchRows: (comp, _ctx) => [comp.id],
  stamp: (ctx, comp, xGuess, _h) => {
    // W5.1 — Adjustable LDO.  Pins: in / adj / out.  The chip enforces
    // V_out − V_adj = 1.25 V (Vref); external R1/R2 sets the actual output.
    // A 100 kΩ bias resistor adj → gnd prevents the adj pin from floating
    // if the user forgets R2.  stampResistor is GND-safe (skips the -1 row).
    const pins = comp.pins;
    const kL3 = ctx.vsrcRow(comp.id);
    if (kL3 === undefined || pins.length < 3) return;
    const inNodeL3  = ctx.pinNode(comp.id, "in");
    const adjNodeL3 = ctx.pinNode(comp.id, "adj");
    const outNodeL3 = ctx.pinNode(comp.id, "out");
    const VrefL3    = Number(comp.params.vref ?? 1.25);
    const vdropL3   = Number(comp.params.vdropout ?? 2.0);
    const iLimL3    = Number(comp.params.iLimit ?? 1.5);
    // Bias resistor: keeps adj from floating when R2 is absent.
    // 100 kΩ draws ~12 µA at a typical 1.25 V adj voltage — negligible.
    stampResistor(ctx.mna, adjNodeL3, -1, 1e5);
    const stL3 = ctx.state.icState.get(comp.id) ?? defaultIcState("lm317");
    const thermalShutdown = ctx.state.thermalDevices.get(comp.id)?.thermalShutdown === true;
    const regL3 = thermalShutdown ? 3 : (Number.isFinite(stL3.reg) ? stL3.reg : 0);
    const activeSetKeyL3 = `lm317:${comp.id}`;
    const vInGuessL3 = ctx.vAt(xGuess, inNodeL3);
    const vAdjGuessL3 = ctx.vAt(xGuess, adjNodeL3);
    const vOutGuessL3 = ctx.vAt(xGuess, outNodeL3);
    const headroomGuessL3 = vInGuessL3 - vAdjGuessL3;
    const headroomRegL3 = !thermalShutdown && regL3 !== 2
      ? ctx.regulatorHeadroomRegime(
        activeSetKeyL3,
        regL3,
        headroomGuessL3,
        VrefL3,
        vdropL3,
      )
      : regL3;
    const entryClampL3 = headroomRegL3 !== 2 && headroomRegL3 !== 3
      && ctx.useCurrentLimitEntryClamp(
        activeSetKeyL3,
        xGuess[kL3] ?? 0,
        iLimL3,
      );
    const currentRegL3 = entryClampL3 ? 2 : headroomRegL3;
    const complianceL3 = Math.max(0, Math.min(VrefL3, headroomGuessL3 - vdropL3));
    const complianceClampL3 = currentRegL3 === 2 && ctx.useCurrentLimitComplianceClamp(
      activeSetKeyL3,
      vOutGuessL3 - vAdjGuessL3,
      complianceL3,
    );
    const stampRegL3 = complianceClampL3
      ? (headroomGuessL3 <= vdropL3 ? 3 : headroomGuessL3 < VrefL3 + vdropL3 ? 1 : 0)
      : currentRegL3;
    stampLinearRegulator(ctx.mna, inNodeL3, adjNodeL3, outNodeL3, kL3,
      stampRegL3, VrefL3, vdropL3, iLimL3);
    const iqL3 = regulatorQuiescentCurrent(comp);
    if (!thermalShutdown && stampRegL3 !== 3 && iqL3 > 0) {
      stampCurrentSource(ctx.mna, adjNodeL3, inNodeL3, iqL3);
    }
  },
  commitState: (ctx, comp, x, _h) => {
    const kL3U = ctx.vsrcRow(comp.id);
    if (kL3U === undefined) return;
    const stL3U = ctx.state.icState.get(comp.id) ?? defaultIcState("lm317");
    const iL3U    = x[kL3U] ?? 0;
    const vInL3U  = ctx.vAt(x, ctx.pinNode(comp.id, "in"));
    const vAdjL3U = ctx.vAt(x, ctx.pinNode(comp.id, "adj"));
    const vOutL3U = ctx.vAt(x, ctx.pinNode(comp.id, "out"));
    const VrefL3U  = Number(comp.params.vref ?? 1.25);
    const vdropL3U = Number(comp.params.vdropout ?? 2.0);
    const iLimL3U  = Number(comp.params.iLimit ?? 1.5);
    const thermalShutdown = ctx.state.thermalDevices.get(comp.id)?.thermalShutdown === true;
    const activeSetKeyL3U = `lm317:${comp.id}`;
    const activeSetEnteredL3U = ctx.currentLimitEntryClampActive(activeSetKeyL3U)
      && !ctx.currentLimitComplianceClampActive(activeSetKeyL3U);
    const newRegL3U = thermalShutdown
      ? 3
      : activeSetEnteredL3U
        ? 2
      : regulatorRegime(stL3U.reg, iL3U, vInL3U, vAdjL3U, vOutL3U,
          VrefL3U, vdropL3U, iLimL3U);
    ctx.state.icState.set(comp.id, { reg: newRegL3U });
  },
  updateCurrent: (ctx, comp, x) => {
    publishRegulatorBranchCurrent(ctx, comp, x);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same regime linearization as linear_reg with the adj pin as the
    // reference (REG holds d(Vout - Vadj) = 0 — the external divider then
    // shapes the closed-loop AC response), plus the same always-present
    // 100 kOhm adj bias resistor the transient stamp writes.
    const pins = comp.pins;
    const k = ctx.vsrcRow(comp.id);
    if (k === undefined || pins.length < 3) return;
    const inNode = ctx.pinNode(comp.id, "in");
    const adjNode = ctx.pinNode(comp.id, "adj");
    const outNode = ctx.pinNode(comp.id, "out");
    stampAcAdmittance(ac, adjNode, -1, 1 / 1e5, 0);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("lm317");
    const thermalShutdown = ctx.state.thermalDevices.get(comp.id)?.thermalShutdown === true;
    const reg = thermalShutdown ? 3 : (Number.isFinite(st.reg) ? st.reg : 0);
    ac.addAc(outNode, k, 1, 0);
    ac.addAc(inNode, k, -1, 0);
    if (reg === 0) {
      ac.addAc(k, outNode, 1, 0);
      ac.addAc(k, adjNode, -1, 0);
    } else if (reg === 1) {
      ac.addAc(k, outNode, 1, 0);
      ac.addAc(k, inNode, -1, 0);
    } else {
      ac.addAc(k, k, 1, 0);
    }
  },
};
