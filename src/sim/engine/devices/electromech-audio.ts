/**
 * ELECTROMECH-AUDIO device cohort (Wave A4 phase 2): relay, dc_motor,
 * servo, stepper, hcsr04, uln2003, uln2803, buzzer, speaker, l293d,
 * tb6612.
 *
 * Every handler body below is the engine's old switch-case / if-block body
 * MOVED VERBATIM, with identifier access adapted through DeviceContext
 * (this._pinNode -> ctx.pinNode, this._vAt -> ctx.vAt, this._isOpenPin ->
 * ctx.isOpenPin, this.state.relays -> ctx.state.relays (and motors/servos/
 * steppers/hcsr04/icState/arduinos alike), this.simTime -> ctx.simTime(),
 * this._icPowerInfo -> ctx.icPowerInfo, the _stampAll pass-level `vt`
 * local -> ctx.junctionVt() — the same pure thermalVoltage(ambient)
 * computation, so the float is identical — the paired
 * `mcuEventDrivers ??= this._buildArduinoPinDrivers(true)` +
 * `mcuEventDrivers.get(node)` calls -> ctx.mcuEventDriverForNode(node)
 * (the lazy shared-map build now lives inside the context alias, still
 * built at most once per state pass), this._componentById.get ->
 * ctx.componentById, this._mcuPowered -> ctx.mcuPowered,
 * this._mcuBootSimTime.get -> ctx.mcuBootSimTime,
 * this._hcsr04MicrobitBridge -> ctx.hcsr04MicrobitBridge,
 * specsForComponent -> ctx.electricalSpecs, out[comp.id] = v ->
 * ctx.setElementCurrent(comp.id, v), and loop break -> handler return).
 * Do not simplify, reorder mna.add calls, or rewrite algebra here: float
 * accumulation order in the matrix is semantics, and the bitwise oracle
 * suite pins these exact trajectories.
 *
 * uln2003/uln2803 share one model exactly as their switch cases shared a
 * body; the handlers still branch on comp.kind for the channel count.
 *
 * Moved WITH this cohort (their only consumers are here now):
 * - _mcuClockHz (servo/hcsr04 cycle-to-seconds conversion),
 * - _servoLoadResistance -> the local servoLoadResistance helper,
 * - _stampClampDiode -> the local stampClampDiode helper (ULN catch
 *   diodes, L293D/TB6612 freewheel paths),
 * - the advanceServoMotion physics-sidecar call (import moved here).
 *
 * Engine-owned machinery deliberately NOT moved:
 * - HC-SR04 event scheduling: HCSR04_MAX_STEP_H / hcsr04MaxStepH(),
 *   nextScheduledEventTime()'s event-aligned step clamping, and the
 *   takeHcsr04EchoEvents() drain of pendingEchoEvent — they schedule
 *   ENGINE step sizes and worker events, not device physics.
 * - MICROBIT_EDGE_PIN_TO_SIM + _hcsr04MicrobitBridge (a topology scan
 *   over this.nets), reached via ctx.hcsr04MicrobitBridge.
 * - _buildArduinoPinDrivers, _mcuPowered, _mcuBootSimTime and the MCU
 *   cores themselves — board machinery shared with the unmigrated MCU
 *   board kinds, reached read-only via ctx.
 * - defaultServoState/defaultHcsr04State and the load()-time
 *   relays/motors/servos/steppers/hcsr04/icState carry-forward plus
 *   snapshot/rollback.
 * - The DC operating-point physical-time hold
 *   (DC_OP_PHYSICAL_TIME_HOLD_KINDS), applied BEFORE commitState dispatch.
 * - The _drivenOutputTargets per-kind cases for uln/l293d/tb6612 (generic
 *   output-sag scan, not a per-kind pass switch — same treatment as lm393
 *   and the digital-ics cohort).
 *
 * Import layering (registry header, decision 7): pure helper FUNCTIONS
 * and exported consts from sim-engine.ts are safe to import despite the
 * module cycle because handlers dereference them only at solve time.
 * Never read sim-engine bindings at module evaluation time from this
 * file.
 */

import type {
  AcDeviceContext,
  DeviceComponent,
  DeviceContext,
  DeviceModel,
} from "../device-registry.js";
import { stampAcAdmittance, type AcStampSurface } from "../ac-system.js";
import {
  shockleyIsFromVf,
  stampCurrentSource,
  stampDiodeShockley,
  stampResistor,
} from "../elements.js";
import { AC_GMIN, shockleyConductanceAtOp } from "./semiconductors.js";
import { advanceServoMotion } from "../../servo-physics.js";
import {
  defaultHcsr04State,
  defaultIcState,
  defaultServoState,
  GATE_R_OUT,
  N_DIODE,
  type ServoEngineState,
} from "../sim-engine.js";

// MCU core clock rates, needed to convert a PinEvent.cycle (cumulative
// instruction-clock count since boot, arduino.ts / rp2040.ts) into seconds for
// cycle-accurate servo and HC-SR04 input edges. Neither core exposes its clock
// on the shared MicrocontrollerCore contract, so this is duplicated here by board
// kind — keep in sync if either core's CLOCK_HZ constant changes.
function _mcuClockHz(kind: string): number {
  return kind === "raspberry_pi_pico" ? 125_000_000 : 16_000_000;
}

/**
 * Committed hobby-servo supply load. Idle draw keeps the historical
 * `idleR` model; while the shaft is actually travelling, an explicit
 * no-load motion current replaces it. The state is committed, so the stamp
 * never switches inside a Newton iteration. This is intentionally not a
 * stall/torque model—the component disclosure says so.
 *
 * The parameter type is the structural intersection of DeviceContext and
 * AcDeviceContext: the servo's small-signal stamp must present the exact
 * committed load its transient stamp presented at the OP, so both facades
 * share this one body.
 */
function servoLoadResistance(
  ctx: Pick<DeviceContext, "electricalSpecs"> & {
    readonly state: { readonly servos: ReadonlyMap<string, ServoEngineState> };
  },
  comp: DeviceComponent,
): number {
  const idleR = Math.max(1, Number(comp.params.idleR ?? 330));
  const state = ctx.state.servos.get(comp.id);
  if (!state || !state.moving || Math.abs(state.velocity) <= 1e-9 || !state.powered) {
    return idleR;
  }

  const specs = ctx.electricalSpecs(comp);
  const nominalVoltage = Math.max(
    0.1,
    Number(comp.params.nominalVoltage ?? specs?.vcc_range?.nominal ?? 4.8),
  );
  const movingCurrent = Math.max(0, Number(comp.params.movingCurrent ?? 0.15));
  return movingCurrent > 0 ? Math.max(0.1, nominalVoltage / movingCurrent) : idleR;
}

/**
 * Stamp one integrated silicon catch/body diode. The 0.7 V point is
 * calibrated at 100 mA, representative of the ULN/L293D protection paths;
 * the Shockley slope then lets clamp voltage rise naturally with current.
 */
function stampClampDiode(
  ctx: DeviceContext,
  anode: number,
  cathode: number,
  xGuess: Float64Array,
  vt: number,
): void {
  const vf = ctx.junctionVf(0.7);
  const ratedCurrent = 0.1;
  const saturationCurrent = shockleyIsFromVf(vf, ratedCurrent, N_DIODE, vt);
  const vtN = N_DIODE * vt;
  const vSat = Math.min(Math.max(40 * vtN, vf + 5 * vtN), 80 * vtN);
  const vd = ctx.vAt(xGuess, anode) - ctx.vAt(xGuess, cathode);
  stampDiodeShockley(
    ctx.mna,
    anode,
    cathode,
    vd,
    saturationCurrent,
    N_DIODE,
    vt,
    vSat,
  );
}

/**
 * Small-signal image of stampClampDiode: the guarded Shockley tangent slope
 * at the OP junction voltage with the identical parameter derivation, plus
 * stampDiodeShockley's GMIN floor. Cut-off clamps contribute ~GMIN; a clamp
 * conducting at the OP (freewheeling bias) is load-bearing.
 */
function acStampClampDiode(
  ctx: AcDeviceContext,
  ac: AcStampSurface,
  anode: number,
  cathode: number,
  vt: number,
): void {
  const vf = ctx.junctionVf(0.7);
  const saturationCurrent = shockleyIsFromVf(vf, 0.1, N_DIODE, vt);
  const vtN = N_DIODE * vt;
  const vSat = Math.min(Math.max(40 * vtN, vf + 5 * vtN), 80 * vtN);
  const vd = ctx.opVoltage(anode) - ctx.opVoltage(cathode);
  const gd = shockleyConductanceAtOp(vd, saturationCurrent, N_DIODE, vt, vSat);
  stampAcAdmittance(ac, anode, cathode, gd + AC_GMIN, 0);
}

export const relayModel: DeviceModel = {
  kinds: ["relay"],
  stamp: (ctx, comp, _xGuess, h) => {
    // W6.1 — SPDT relay.
    //
    // COIL (coil_a / coil_b): series R+L modelled as a Norton companion.
    //
    // Backward-Euler on v_coil = R·i + L·(i − i_prev)/h:
    //   i = (1 / (R + L/h)) · v_ab + I_hist     … (steady-state: i = v/R, correct)
    //   G_coil = 1 / (R + L/h)
    //   I_hist  = G_coil · (L/h) · i_prev
    //
    // Stamp as: conductance G_coil between coil_a and coil_b (= Thevenin R+L/h),
    // plus a current source injecting I_hist from coil_a to coil_b.
    //
    // Current source direction derivation: the inductor companion current
    // source injects i_prev from the "current flow" sense. In the standard
    // inductor companion (stampInductor) with current flowing node i → node j:
    //   addB(i, -iPrev)  — removes iPrev from node i (current leaves i)
    //   addB(j, +iPrev)  — injects iPrev into node j (current enters j)
    // Here we adopt convention: I flows coil_a → coil_b (positive i_coil means
    // current entering coil_a and leaving coil_b).
    //   stampCurrentSource(coil_a, coil_b, -I_hist)
    //     → addB(coil_a, -I_hist): removes I_hist from coil_a
    //     → addB(coil_b, +I_hist): injects I_hist into coil_b
    // This correctly continues inductive current when the driving voltage
    // is removed (i_prev > 0 keeps coil_a→coil_b current flowing via the
    // companion, producing the inductive-kick transient).
    //
    // CONTACTS: committed-state pattern — use energized from the previous step
    // (never re-decided from xGuess within Newton to avoid chattering).
    //   Energized: COM-NO closed (1 mΩ), COM-NC open (RSHUNT only)
    //   De-energized: COM-NC closed (1 mΩ), COM-NO open (RSHUNT only)
    //
    // Relay needs NO branch row: the Norton companion uses only node rows.
    const caNode = ctx.pinNode(comp.id, "coil_a");
    const cbNode = ctx.pinNode(comp.id, "coil_b");
    const comNode = ctx.pinNode(comp.id, "com");
    const noNode  = ctx.pinNode(comp.id, "no");
    const ncNode  = ctx.pinNode(comp.id, "nc");

    const relayR = Math.max(1, Number(comp.params.coilR ?? 70));
    const relayL = Math.max(1e-6, Number(comp.params.coilL ?? 0.05));

    const relaySt = ctx.state.relays.get(comp.id) ?? { iCoil: 0, energized: false };
    if (ctx.dcSolveMode()) {
      // A winding is a short through its resistance at DC — the identical
      // treatment inductorModel gives a plain coil (coilR is already
      // floored at 1 ohm, so no extra well-posedness floor is needed). The
      // history source is integration state and must not bias the true
      // operating point; _commitDcOperatingPointState maps the solved
      // terminal voltage back onto the DC coil current.
      stampResistor(ctx.mna, caNode, cbNode, relayR);
    } else {
      // Norton companion for R+L coil.
      const G_coil = 1 / (relayR + relayL / h);
      const I_hist  = G_coil * (relayL / h) * relaySt.iCoil;

      // Stamp coil impedance.
      stampResistor(ctx.mna, caNode, cbNode, relayR + relayL / h);
      // Stamp history current source: I_hist flows coil_a → coil_b.
      // stampCurrentSource(i, j, I) injects I into node i and removes from j.
      // We want current INTO coil_a (node i=caNode) and OUT of coil_b (node j=cbNode).
      stampCurrentSource(ctx.mna, caNode, cbNode, -I_hist);
      // Sign derivation: with v_ab > 0 and i_prev > 0 the coil is carrying
      // current.  After the driver opens (v_ab → 0), the companion injects
      // -I_hist at caNode (removes current from caNode) driving v_caNode below
      // v_cbNode — the inductive kick flows backward through RSHUNT or the
      // flyback diode path.  Steady-state DC: I_hist = G_coil·(L/h)·i_prev,
      // and i_coil = G_coil·0 + I_hist → decays each step to 0. Correct.
    }

    // Contact stamp: committed energized state from previous step.
    const CONTACT_R = 1e-3;  // 1 mΩ closed contact resistance
    if (relaySt.energized) {
      // Energized: COM↔NO closed, COM↔NC open.
      stampResistor(ctx.mna, comNode, noNode, CONTACT_R);
    } else {
      // De-energized: COM↔NC closed, COM↔NO open.
      stampResistor(ctx.mna, comNode, ncNode, CONTACT_R);
    }
  },
  // W6.1 — relay post-solve state update.
  //
  // After Newton convergence, recompute i_coil and update the energized flag
  // with hysteresis on the coil voltage |v_ab|.
  //
  // i_coil = G_coil · v_ab + I_hist
  //        where G_coil = 1/(R + L/h), I_hist = G_coil·(L/h)·i_prev
  //
  // Energised state transitions (hysteresis on |v_coil| — simpler and more
  // robust than a current threshold for a fixed-R coil because the voltage
  // is directly controlled by the driver):
  //   de-energised: energise when |v_ab| >= vPull
  //   energised:    de-energise when |v_ab| < vDrop
  //   otherwise:    hold current state
  commitState: (ctx, comp, x, h) => {
    const caNode = ctx.pinNode(comp.id, "coil_a");
    const cbNode = ctx.pinNode(comp.id, "coil_b");
    const vCa = ctx.vAt(x, caNode);
    const vCb = ctx.vAt(x, cbNode);
    const vAb = vCa - vCb;

    const relayRU  = Math.max(1, Number(comp.params.coilR ?? 70));
    const relayLU  = Math.max(1e-6, Number(comp.params.coilL ?? 0.05));
    const vPullU   = Number(comp.params.vPull ?? 3.5);
    const vDropU   = Number(comp.params.vDrop ?? 1.5);

    const G_coilU  = 1 / (relayRU + relayLU / h);
    const prevStU  = ctx.state.relays.get(comp.id) ?? { iCoil: 0, energized: false };
    const I_histU  = G_coilU * (relayLU / h) * prevStU.iCoil;
    const iCoilNew = G_coilU * vAb + I_histU;

    const absV     = Math.abs(vAb);
    let newEnergized = prevStU.energized;
    if (!prevStU.energized && absV >= vPullU) {
      newEnergized = true;
    } else if (prevStU.energized && absV < vDropU) {
      newEnergized = false;
    }
    // else: hold (hysteresis band between vDrop and vPull)

    ctx.state.relays.set(comp.id, { iCoil: iCoilNew, energized: newEnergized });
  },
  updateCurrent: (ctx, comp, _x) => {
    // W6.1 — relay: report committed coil current from the relay state Map.
    // The coil current is updated post-solve in commitState — reading it here
    // gives the converged value for this timestep without recomputing the companion.
    const relaySti = ctx.state.relays.get(comp.id);
    ctx.setElementCurrent(comp.id, relaySti ? relaySti.iCoil : 0);
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Coil: the true R+L winding admittance 1/(R + j*omega*L) — the complex
    // limit of the transient companion whose history current vanishes in
    // small signal. Contacts: the committed energized state selects the same
    // 1 mOhm closed pair the transient stamp uses; the open pair stamps
    // nothing (armature mechanics have no small-signal model).
    const caNode = ctx.pinNode(comp.id, "coil_a");
    const cbNode = ctx.pinNode(comp.id, "coil_b");
    const relayR = Math.max(1, Number(comp.params.coilR ?? 70));
    const relayL = Math.max(1e-6, Number(comp.params.coilL ?? 0.05));
    const xl = omega * relayL;
    const den = relayR * relayR + xl * xl;
    stampAcAdmittance(ac, caNode, cbNode, relayR / den, -xl / den);
    const energized = ctx.state.relays.get(comp.id)?.energized === true;
    const comNode = ctx.pinNode(comp.id, "com");
    if (energized) {
      stampAcAdmittance(ac, comNode, ctx.pinNode(comp.id, "no"), 1 / 1e-3, 0);
    } else {
      stampAcAdmittance(ac, comNode, ctx.pinNode(comp.id, "nc"), 1 / 1e-3, 0);
    }
  },
};

export const dcMotorModel: DeviceModel = {
  kinds: ["dc_motor"],
  // ── W7.1 DC brushed motor ────────────────────────────────────────────
  stamp: (ctx, comp, _xGuess, h) => {
    // Series R+L winding PLUS back-EMF (Ke·omega), ONE Norton companion.
    //
    // Winding KVL (m1 = + terminal, m2 = − terminal, i flows m1→m2):
    //   v_ab = R·i + L·di/dt + Vbemf,    Vbemf = Ke·omega_committed
    //
    // Backward-Euler on the RL part:
    //   i = (v_ab − Vbemf + (L/h)·i_prev) / (R + L/h)
    //     = G·v_ab + G·((L/h)·i_prev − Vbemf)
    //   G = 1 / (R + L/h)
    //   I_eq = G·((L/h)·i_prev − Vbemf)   (the constant term)
    //
    // Norton stamp:
    //   conductance  G between m1 and m2  (stampResistor uses R+L/h equiv)
    //   current src  I_eq from m1 to m2   (drives history + back-EMF offset)
    //
    // Sign check at R=0, omega=0 (pure inductor):
    //   G = h/L, I_eq = (1/h)·i_prev·h/L · L/h × ... simplifies to i_prev/1
    //   reduces to stampInductor(m1,m2,L,h,i_prev) — correct.
    //
    // Back-EMF sign: Vbemf = Ke·omega OPPOSES applied voltage (motor convention).
    //   Applied +V draws +i, which accelerates omega, which raises Vbemf,
    //   which reduces i at steady state. Correct motoring behaviour.
    //
    // No branch row needed: Norton companion uses only node rows.
    // omega is the COMMITTED speed from the previous step, held constant
    // through all Newton iterations (committed-state pattern).

    const mR = Math.max(0.001, Number(comp.params.windingR ?? 5));
    const mL = Math.max(1e-9,  Number(comp.params.windingL ?? 0.002));
    const Ke  = Number(comp.params.Ke ?? 0.01);

    const motorSt = ctx.state.motors.get(comp.id) ?? { iWinding: 0, omega: 0 };
    const Vbemf = Ke * motorSt.omega;               // back-EMF (V)

    const m1Node = ctx.pinNode(comp.id, "m1");
    const m2Node = ctx.pinNode(comp.id, "m2");

    if (ctx.dcSolveMode()) {
      // DC winding law i = (v_ab - Vbemf) / R: a short through the winding
      // resistance (inductorModel's dc treatment) plus the back-EMF of the
      // physical-time-held rotor speed as the Norton offset. The inductive
      // history is integration state and is dropped; the OP commit maps the
      // solved terminal voltage back onto the DC winding current.
      const I_eqDc = -Vbemf / mR;
      stampResistor(ctx.mna, m1Node, m2Node, mR);
      stampCurrentSource(ctx.mna, m1Node, m2Node, -I_eqDc);
      return;
    }

    const G_m   = 1 / (mR + mL / h);
    // I_eq encodes both the inductive history and the back-EMF offset.
    // Positive I_eq: net current source injected from m1 toward m2.
    const I_eq  = G_m * ((mL / h) * motorSt.iWinding - Vbemf);

    // Stamp equivalent resistance (winding impedance + inductive memory).
    stampResistor(ctx.mna, m1Node, m2Node, mR + mL / h);
    // Stamp Norton current: I_eq injected from m1 toward m2.
    // stampCurrentSource(i, j, I) → addB(i, I), addB(j, -I).
    // With i=m1, j=m2, I=I_eq: injects I_eq into m1 (source) and removes from m2.
    // Negative sign because we want the source to model i_prev carrying forward
    // and Vbemf reducing the effective drive — the standard RL companion direction.
    stampCurrentSource(ctx.mna, m1Node, m2Node, -I_eq);
  },
  // W7.1 — dc_motor post-solve state update.
  //
  // After Newton convergence:
  //   1. Recompute iWinding from the solved node voltages (same formula as stamp).
  //   2. Advance omega using Forward-Euler speed dynamics (committed post-solve).
  //
  // Speed dynamics:
  //   dω/dt = (Kt·i − b·ω − loadTorque) / J
  //   ω_new = ω + h·(Kt·i − b·ω − loadTorque) / J
  //
  // Stability: J/b = 1e-5/1e-5 = 1 s >> h (typical h ≤ 1e-3 s).
  // Forward Euler is unconditionally stable when h << J/b.
  //
  // Negative ω is allowed — reverse polarity drives reverse rotation.
  // No clamping of direction: the model supports bidirectional drive.
  commitState: (ctx, comp, x, h) => {
    const mRU = Math.max(0.001, Number(comp.params.windingR ?? 5));
    const mLU = Math.max(1e-9,  Number(comp.params.windingL ?? 0.002));
    const KeU = Number(comp.params.Ke ?? 0.01);
    const KtU = Number(comp.params.Kt ?? 0.01);
    const J   = Math.max(1e-12, Number(comp.params.inertia ?? 1e-5));
    const b   = Math.max(0,     Number(comp.params.friction ?? 1e-5));
    const TL  = Number(comp.params.loadTorque ?? 0);

    const prevMst  = ctx.state.motors.get(comp.id) ?? { iWinding: 0, omega: 0 };
    const G_mU     = 1 / (mRU + mLU / h);
    const VbemfU   = KeU * prevMst.omega;
    const I_eqU    = G_mU * ((mLU / h) * prevMst.iWinding - VbemfU);

    const m1NodeU  = ctx.pinNode(comp.id, "m1");
    const m2NodeU  = ctx.pinNode(comp.id, "m2");
    const vm1      = ctx.vAt(x, m1NodeU);
    const vm2      = ctx.vAt(x, m2NodeU);
    const v_ab     = vm1 - vm2;

    // Winding current: Norton companion formula (same as stamp).
    const iWindingNew = G_mU * v_ab + I_eqU;

    // Rotor speed: Forward-Euler integration of the torque equation.
    const torqueNet   = KtU * iWindingNew - b * prevMst.omega - TL;
    const omegaNew    = prevMst.omega + (h / J) * torqueNet;

    ctx.state.motors.set(comp.id, { iWinding: iWindingNew, omega: omegaNew });
  },
  updateCurrent: (ctx, comp, _x) => {
    // W7.1 — dc_motor: report committed winding current from the motors Map.
    // iWinding is updated post-solve in commitState — reading it here gives
    // the converged value for this timestep without recomputing the companion.
    const motorSti = ctx.state.motors.get(comp.id);
    ctx.setElementCurrent(comp.id, motorSti ? motorSti.iWinding : 0);
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Electrical winding only: R + j*omega*L admittance. The rotor speed is
    // held at its committed OP value, so the back-EMF term is a constant
    // that vanishes in small signal — electromechanical coupling (the
    // motional impedance) is deliberately outside this compact AC model.
    const mR = Math.max(0.001, Number(comp.params.windingR ?? 5));
    const mL = Math.max(1e-9, Number(comp.params.windingL ?? 0.002));
    const xl = omega * mL;
    const den = mR * mR + xl * xl;
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, "m1"),
      ctx.pinNode(comp.id, "m2"),
      mR / den,
      -xl / den,
    );
  },
};

export const servoModel: DeviceModel = {
  kinds: ["servo"],
  // ── W7.2 Hobby servo ────────────────────────────────────────────────
  stamp: (ctx, comp, _xGuess, _h) => {
    // The power branch is a committed idle/moving load. PWM decode and
    // shaft motion happen post-solve; no state decision is made inside
    // Newton. The signal input itself is genuinely high impedance: the
    // engine-wide 1 TΩ numerical shunt regularises a floating node and is
    // disclosed as numerical, so an invented 1 MΩ physical pull-down is
    // neither needed nor appropriate.
    const supplyR = servoLoadResistance(ctx, comp);
    const vplusNode = ctx.pinNode(comp.id, "vplus");
    const gndNode   = ctx.pinNode(comp.id, "gnd");

    stampResistor(ctx.mna, vplusNode, gndNode, supplyR);
  },
  // W7.2 — servo post-solve state update (PWM decode + finite motion).
  //
  // Strategy: committed-state edge detection.
  //   - A directly connected Arduino/Pico output replays every ordered
  //     cycle-timestamped PinEvent from this MCU step.
  //   - Other drivers retain the solved V(sig)-V(gnd) 2.0 V threshold
  //     fallback, with at most one edge at the outer step boundary.
  //   - Rising edges record their exact time; falling edges measure and
  //     clamp the HIGH width before mapping it to the target angle.
  //   - Every accepted step advances the actual shaft toward that target at
  //     the declared no-load speed, scaled by solved VCC.
  //
  commitState: (ctx, comp, x, h) => {
    const loadResistance = servoLoadResistance(ctx, comp);
    const sigNodeS   = ctx.pinNode(comp.id, "sig");
    const gndNodeS   = ctx.pinNode(comp.id, "gnd");
    const vSig       = ctx.vAt(x, sigNodeS) - ctx.vAt(x, gndNodeS);

    const SIG_THRESH = 2.0; // V — same TTL threshold used throughout the engine
    const sampledSig: 0 | 1 = vSig >= SIG_THRESH ? 1 : 0;

    const prevSvSt = ctx.state.servos.get(comp.id) ?? defaultServoState();

    const minPulse  = Number(comp.params.minPulseMs ?? 1.0);
    const maxPulse  = Number(comp.params.maxPulseMs ?? 2.0);
    const minAngle  = Number(comp.params.minAngle   ?? 0);
    const maxAngle  = Number(comp.params.maxAngle   ?? 180);

    let { riseT, pulseMs, targetAngle } = prevSvSt;
    let lastSig: 0 | 1 = prevSvSt.lastSig === 1 ? 1 : 0;
    const edges: Array<{ level: 0 | 1; time: number }> = [];

    const driver = ctx.mcuEventDriverForNode(sigNodeS);
    const driverComp = driver
      ? ctx.componentById(driver.compId)
      : undefined;
    const driverMcu = driver ? ctx.state.arduinos.get(driver.compId) : undefined;
    const driverPowered = driverComp ? ctx.mcuPowered(driverComp, x) : false;
    const driverPinEvents = driver && driverMcu
      ? driverMcu.getStepPinEvents().filter(
          (event) => event.pin === driver.pin && event.level !== null,
        )
      : [];
    const finalDrive = driver && driverMcu
      ? driverMcu.pinDriveState(driver.pin)
      : "input";
    const hasEventDriver = Boolean(
      driver &&
      driverComp &&
      driverMcu &&
      driverPowered &&
      (finalDrive === "out-high" || finalDrive === "out-low" || driverPinEvents.length > 0),
    );

    if (hasEventDriver && driver && driverComp) {
      const clockHz = _mcuClockHz(driverComp.kind);
      const bootT = ctx.mcuBootSimTime(driver.compId) ?? 0;
      for (const event of driverPinEvents) {
        edges.push({ level: event.level!, time: bootT + event.cycle / clockHz });
      }
      edges.sort((a, b) => a.time - b.time);
    } else if (sampledSig !== lastSig) {
      // Historical sampled-level fallback for non-event sources.
      edges.push({ level: sampledSig, time: ctx.simTime() });
    }

    const applyEdge = (edge: { level: 0 | 1; time: number }): void => {
      if (edge.level === lastSig) return;
      if (lastSig === 0 && edge.level === 1) {
        riseT = edge.time;
      } else if (lastSig === 1 && edge.level === 0 && Number.isFinite(riseT) && edge.time >= riseT) {
        const measuredMs = (edge.time - riseT) * 1000;
        pulseMs = Math.max(minPulse, Math.min(maxPulse, measuredMs));
        const pulseSpan = Math.max(1e-9, maxPulse - minPulse);
        const t = (pulseMs - minPulse) / pulseSpan;
        targetAngle = minAngle + t * (maxAngle - minAngle);
      }
      lastSig = edge.level;
    };

    for (const edge of edges) applyEdge(edge);

    if (hasEventDriver && driver && driverMcu) {
      // Reconcile a stable MCU level after topology load, reset, or an
      // output-to-input transition that produces no non-null edge. Normal
      // PWM edges already agree and never take this sampled fallback.
      const finalLevel: 0 | 1 = finalDrive === "out-high"
        ? 1
        : finalDrive === "out-low"
          ? 0
          : sampledSig;
      if (finalLevel !== lastSig) applyEdge({ level: finalLevel, time: ctx.simTime() + h });
    }
    // No complete pulse: target holds, while the shaft may continue moving.

    const vplusNodeS = ctx.pinNode(comp.id, "vplus");
    const supplyVoltage = Math.max(
      0,
      ctx.vAt(x, vplusNodeS) - ctx.vAt(x, gndNodeS),
    );
    const servoSpecs = ctx.electricalSpecs(comp);
    const motion = advanceServoMotion({
      angleDeg: prevSvSt.angle,
      targetAngleDeg: targetAngle,
      dtSeconds: h,
      maxSpeedDegPerSecond: Math.max(
        0,
        Number(comp.params.maxSpeedDegPerSecond ?? comp.params.speedDegPerSecond ?? 600),
      ),
      supplyVoltageV: supplyVoltage,
      nominalSupplyVoltageV: Math.max(
        0.1,
        Number(comp.params.nominalVoltage ?? servoSpecs?.vcc_range?.nominal ?? 4.8),
      ),
      minimumOperatingVoltageV: Math.max(
        0,
        Number(comp.params.minimumOperatingVoltage ?? servoSpecs?.vcc_range?.min ?? 4.5),
      ),
      minAngleDeg: minAngle,
      maxAngleDeg: maxAngle,
    });

    ctx.state.servos.set(comp.id, {
      lastSig,
      riseT,
      pulseMs,
      targetAngle,
      angle: motion.angleDeg,
      velocity: motion.velocityDegPerSecond,
      moving: motion.moving && Math.abs(motion.velocityDegPerSecond) > 1e-9,
      powered: motion.powered,
      supplyVoltage,
      loadResistance,
    });
  },
  updateCurrent: (ctx, comp, x) => {
    // W7.2 — servo: report the current through the same committed
    // idle/moving supply load stamped for this step.
    const stateSi = ctx.state.servos.get(comp.id);
    const loadRSi = stateSi && Number.isFinite(stateSi.loadResistance)
      ? stateSi.loadResistance
      : servoLoadResistance(ctx, comp);
    const vplusNodeI = ctx.pinNode(comp.id, "vplus");
    const gndNodeI   = ctx.pinNode(comp.id, "gnd");
    const vDiffI     = ctx.vAt(x, vplusNodeI) - ctx.vAt(x, gndNodeI);
    ctx.setElementCurrent(comp.id, vDiffI / loadRSi);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The committed idle/moving supply load is the servo's only analog
    // contribution (the SIG input is genuinely high-Z, disclosed as relying
    // on the engine-wide numerical shunt). PWM decode and shaft motion are
    // physical-time state with no small-signal model.
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, "vplus"),
      ctx.pinNode(comp.id, "gnd"),
      1 / servoLoadResistance(ctx, comp),
      0,
    );
  },
};

export const stepperModel: DeviceModel = {
  kinds: ["stepper"],
  // ── W7.2 Bipolar stepper motor ──────────────────────────────────────
  stamp: (ctx, comp, _xGuess, h) => {
    // Two independent RL Norton companions, one per coil (A and B).
    // This is the same companion used by the relay and dc_motor (minus
    // back-EMF — v1 omits back-EMF for simplicity).
    //
    // Backward-Euler derivation (same as relay/dc_motor):
    //   G = 1 / (R + L/h)
    //   I_hist = G * (L/h) * i_prev
    //
    // Norton stamp per coil:
    //   stampResistor(a, b, R + L/h)       ← conductance branch
    //   stampCurrentSource(a, b, -I_hist)  ← inductive memory injection
    //
    // Negative sign on I_hist: stampCurrentSource(i,j,I) adds I to node i
    // and subtracts from j.  We want the history current to flow a→b (the
    // same direction as i_prev), so we inject −I_hist at node a (the +
    // terminal).  The stampResistor with G then pulls the rest of the
    // current to satisfy KCL.
    //
    // Phase detection and position update happen post-solve in commitState,
    // using committed coil currents — NOT here inside Newton iterations.

    const coilR = Math.max(0.001, Number(comp.params.coilR ?? 10));
    const coilL = Math.max(1e-9,  Number(comp.params.coilL ?? 0.01));

    const stepSt  = ctx.state.steppers.get(comp.id) ?? { iA: 0, iB: 0, phase: -1, lastKnownPhase: -1, position: 0 };

    const a1Node = ctx.pinNode(comp.id, "a1");
    const a2Node = ctx.pinNode(comp.id, "a2");
    const b1Node = ctx.pinNode(comp.id, "b1");
    const b2Node = ctx.pinNode(comp.id, "b2");

    if (ctx.dcSolveMode()) {
      // Both windings short through their resistance at DC (inductorModel's
      // dc treatment; coilR is floored well above the 1e-6 ohm guard). The
      // history sources are integration state and dropped; the OP commit
      // maps the solved terminal voltages back onto DC coil currents.
      stampResistor(ctx.mna, a1Node, a2Node, coilR);
      stampResistor(ctx.mna, b1Node, b2Node, coilR);
      return;
    }

    const G_coil  = 1 / (coilR + coilL / h);
    const I_histA = G_coil * (coilL / h) * stepSt.iA;
    const I_histB = G_coil * (coilL / h) * stepSt.iB;

    // Coil A: a1 → a2
    stampResistor(ctx.mna, a1Node, a2Node, coilR + coilL / h);
    stampCurrentSource(ctx.mna, a1Node, a2Node, -I_histA);
    // Coil B: b1 → b2
    stampResistor(ctx.mna, b1Node, b2Node, coilR + coilL / h);
    stampCurrentSource(ctx.mna, b1Node, b2Node, -I_histB);
  },
  // W7.2 — stepper post-solve state update (phase detection + position count).
  //
  // Strategy: committed-state phase table.
  //   - Read coil currents from committed node voltages via the Norton formula.
  //   - Map (signA, signB) to phase 0–3 using the Gray-code full-step table.
  //     Deadband: |i| < 1 mA → treated as zero (phase = -1, unenergised).
  //   - If phase transitions +1 mod 4 → position++.
  //     If phase transitions −1 mod 4 → position--.
  //     Non-sequential jump or phase=-1 or no change → hold.
  //
  // Phase table (sign(iA), sign(iB)):
  //   phase 0: (+1, +1)    phase 1: (−1, +1)
  //   phase 2: (−1, −1)    phase 3: (+1, −1)
  commitState: (ctx, comp, x, h) => {
    const coilRS = Math.max(0.001, Number(comp.params.coilR ?? 10));
    const coilLS = Math.max(1e-9,  Number(comp.params.coilL ?? 0.01));

    const prevStSt = ctx.state.steppers.get(comp.id) ?? { iA: 0, iB: 0, phase: -1, lastKnownPhase: -1, position: 0 };
    const G_coilS  = 1 / (coilRS + coilLS / h);

    const a1NodeS = ctx.pinNode(comp.id, "a1");
    const a2NodeS = ctx.pinNode(comp.id, "a2");
    const b1NodeS = ctx.pinNode(comp.id, "b1");
    const b2NodeS = ctx.pinNode(comp.id, "b2");

    const vA1 = ctx.vAt(x, a1NodeS);
    const vA2 = ctx.vAt(x, a2NodeS);
    const vB1 = ctx.vAt(x, b1NodeS);
    const vB2 = ctx.vAt(x, b2NodeS);

    // Reconstruct coil currents via Norton companion formula.
    const I_histAS = G_coilS * (coilLS / h) * prevStSt.iA;
    const I_histBS = G_coilS * (coilLS / h) * prevStSt.iB;
    const iANew    = G_coilS * (vA1 - vA2) + I_histAS;
    const iBNew    = G_coilS * (vB1 - vB2) + I_histBS;

    // ±1 mA deadband: distinguish intentional zero from tiny leakage.
    const DEADBAND = 0.001; // A
    const signA = Math.abs(iANew) < DEADBAND ? 0 : (iANew > 0 ? 1 : -1);
    const signB = Math.abs(iBNew) < DEADBAND ? 0 : (iBNew > 0 ? 1 : -1);

    // Determine current phase from sign pair.
    // Phase index maps to (signA, signB): 0=(+,+), 1=(-,+), 2=(-,-), 3=(+,-).
    let newPhase: number;
    if      (signA ===  1 && signB ===  1) newPhase = 0;
    else if (signA === -1 && signB ===  1) newPhase = 1;
    else if (signA === -1 && signB === -1) newPhase = 2;
    else if (signA ===  1 && signB === -1) newPhase = 3;
    else                                   newPhase = -1; // unenergised / deadband

    // lastKnownPhase skips deadband (-1) steps so that polarity reversals
    // that momentarily pass through zero are still counted as a single step.
    // Only update lastKnownPhase when we actually observe a valid phase.
    const newLastKnownPhase = newPhase >= 0 ? newPhase : prevStSt.lastKnownPhase;

    let newPosition = prevStSt.position;
    if (newPhase >= 0 && prevStSt.lastKnownPhase >= 0) {
      // Detect forward or reverse sequential step using the last confirmed phase,
      // bridging across any deadband gap caused by the coil current crossing zero.
      const delta = (newPhase - prevStSt.lastKnownPhase + 4) % 4;
      if (delta === 1) {
        newPosition++; // forward step
      } else if (delta === 3) {
        newPosition--; // reverse step (3 forward ≡ −1 mod 4)
      }
      // delta 0 = hold, delta 2 = non-sequential jump → ignore
    }

    ctx.state.steppers.set(comp.id, {
      iA: iANew, iB: iBNew,
      phase: newPhase, lastKnownPhase: newLastKnownPhase,
      position: newPosition,
    });
  },
  updateCurrent: (ctx, comp, _x) => {
    // W7.2 — stepper: report total coil current draw |iA| + |iB|.
    // Both values are the committed currents from the steppers Map, set
    // post-solve in commitState for this step.
    const stepSti = ctx.state.steppers.get(comp.id);
    ctx.setElementCurrent(comp.id, stepSti ? (Math.abs(stepSti.iA) + Math.abs(stepSti.iB)) : 0);
  },
  acStamp: (ctx, comp, ac, omega) => {
    // Two independent R + j*omega*L winding admittances — the electrically
    // identical treatment the relay coil and dc_motor winding get. Phase and
    // position are step-counting mechanics with no small-signal model.
    const coilR = Math.max(0.001, Number(comp.params.coilR ?? 10));
    const coilL = Math.max(1e-9, Number(comp.params.coilL ?? 0.01));
    const xl = omega * coilL;
    const den = coilR * coilR + xl * xl;
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, "a1"),
      ctx.pinNode(comp.id, "a2"),
      coilR / den,
      -xl / den,
    );
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, "b1"),
      ctx.pinNode(comp.id, "b2"),
      coilR / den,
      -xl / den,
    );
  },
};

export const hcsr04Model: DeviceModel = {
  kinds: ["hcsr04"],
  // ── S18b HC-SR04 ultrasonic distance sensor ─────────────────────────
  stamp: (ctx, comp, _xGuess, _h) => {
    // TRIG: 1 MΩ pull-down, unconditional (never-floating input net —
    // servo SIG_R precedent). Edge/level decisions happen post-solve in
    // commitState using committed x, NEVER here inside Newton iterations.
    const TRIG_R = 1e6;
    const trigNodeH = ctx.pinNode(comp.id, "trig");
    const gndNodeH  = ctx.pinNode(comp.id, "gnd");
    stampResistor(ctx.mna, trigNodeH, gndNodeH, TRIG_R);

    // ECHO: push-pull rail-referenced drive, gated on the COMMITTED (previous
    // step) VCC-GND reading — never the live xGuess, so the drive
    // decision cannot flicker mid-iteration. A real HC-SR04 drives ECHO
    // continuously while powered (0 V when idle, not hi-Z); only an
    // unpowered part (VCC < 3.0 V) goes hi-Z here, matching real
    // dropout behaviour (a dead chip cannot actively drive its output).
    // Wide-supply: the target voltage is the part's OWN committed VCC
    // (3.0-5.5 V), not a fixed constant, so a 3.3 V micro:bit rail and a
    // 5 V Arduino rail both get a correctly-scaled ECHO high level.
    const hcSt = ctx.state.hcsr04.get(comp.id);
    const physicallyPowered = hcSt && hcSt.vccV >= 3.0
      && !ctx.isOpenPin(comp.id, "vcc")
      && !ctx.isOpenPin(comp.id, "gnd");
    if (physicallyPowered) {
      const echoNodeH = ctx.pinNode(comp.id, "echo");
      const vccNodeH = ctx.pinNode(comp.id, "vcc");
      const echoRailH = hcSt.echoOut ? vccNodeH : gndNodeH;
      stampResistor(ctx.mna, echoNodeH, echoRailH, GATE_R_OUT);

      // Typical HC-SR04 operating current. This is deliberately stamped,
      // rather than merely reported in getElementI(), so batteries and
      // finite-impedance supplies sag by the energy the sensor consumes.
      stampCurrentSource(ctx.mna, gndNodeH, vccNodeH, 0.002);
    }
  },
  // S18b — HC-SR04 TRIG state-machine + ECHO schedule, committed
  // post-solve (never inside Newton iterations — mirrors the servo PWM
  // decode above). See Hcsr04EngineState's doc comment for the full
  // design; this is the state-machine half (the stamp above reads
  // `echoOut`/`vccV` committed HERE on the PREVIOUS step).
  commitState: (ctx, comp, x, h) => {
    const hSt = ctx.state.hcsr04.get(comp.id) ?? defaultHcsr04State();

    const vccNodeH = ctx.pinNode(comp.id, "vcc");
    const gndNodeH = ctx.pinNode(comp.id, "gnd");
    const vccVNew = ctx.vAt(x, vccNodeH) - ctx.vAt(x, gndNodeH);
    const vccOkNew = vccVNew >= 3.0;

    let { trigLevel, phase, echoRiseAt, echoFallAt, echoOut, pendingEchoEvent } = hSt;

    if (!vccOkNew) {
      // Unpowered: real dropout behaviour resets the sensor entirely
      // (matches the stamp above, which skips the ECHO drive below 3.0 V).
      trigLevel = 0; phase = "idle"; echoRiseAt = NaN; echoFallAt = NaN; echoOut = 0;
    } else {
      const trigNodeH = ctx.pinNode(comp.id, "trig");

      // Path (a): an MCU pin drives TRIG directly -> cycle-timestamped
      // edges, exact regardless of the outer step h. Path (b) otherwise:
      // committed net-voltage threshold sampling (2.0 V), timed at
      // `ctx.simTime()` (the step-start value — servo precedent above).
      const driver = ctx.mcuEventDriverForNode(trigNodeH);
      const driverComp = driver
        ? ctx.componentById(driver.compId)
        : undefined;
      const driverMcu = driver ? ctx.state.arduinos.get(driver.compId) : undefined;
      const driverPowered = driverComp ? ctx.mcuPowered(driverComp, x) : false;

      const edges: Array<{ level: 0 | 1; time: number }> = [];
      if (driver && driverMcu && driverPowered) {
        const clockHz = _mcuClockHz(driverComp!.kind);
        const bootT = ctx.mcuBootSimTime(driver.compId) ?? 0;
        for (const ev of driverMcu.getStepPinEvents()) {
          if (ev.pin !== driver.pin || ev.level === null) continue;
          edges.push({ level: ev.level, time: bootT + ev.cycle / clockHz });
        }
      } else {
        const vTrigH = ctx.vAt(x, trigNodeH) - ctx.vAt(x, gndNodeH);
        const curLevelH: 0 | 1 = vTrigH >= 2.0 ? 1 : 0;
        if (curLevelH !== trigLevel) edges.push({ level: curLevelH, time: ctx.simTime() });
      }

      const distanceCmH = Math.max(2, Math.min(400, Number(comp.params.distanceCm ?? 50)));
      const noEchoH = Number(comp.params.noEcho ?? 0) !== 0;

      for (const edge of edges) {
        if (phase === "idle" && trigLevel === 0 && edge.level === 1) {
          phase = "armed";
        } else if (phase === "armed" && trigLevel === 1 && edge.level === 0) {
          // Schedule: rise at tFall+250us, fall at rise + width (distance,
          // or the 38 ms out-of-range timeout).
          const riseAt = edge.time + 250e-6;
          const widthS = noEchoH ? 38e-3 : 58.0e-6 * distanceCmH;
          echoRiseAt = riseAt;
          echoFallAt = riseAt + widthS;
          phase = "pending";

          // Payload is fully known now (no need to wait for the rise
          // itself) — see messages.ts Hcsr04EchoEvent.
          const bridge = ctx.hcsr04MicrobitBridge(comp);
          if (bridge) {
            pendingEchoEvent = {
              boardComponentId: bridge.boardComponentId,
              pinId: bridge.pinId,
              delayUs: 250,
              widthUs: widthS * 1e6,
            };
          }
        }
        // phase "pending"/"active": retrigger ignored (real module
        // behaviour) — trigLevel still tracks so a genuine post-echo
        // edge is seen cleanly once phase returns to "idle".
        trigLevel = edge.level;
      }

      // Fire the schedule at the step boundary it lands on. `now` is this
      // step's END time (ctx.simTime() + h), mirroring the IC
      // delayed-output due-time check (_updateDelayedOutputs) —
      // event-aligned stepping (nextScheduledEventTime()) guarantees this
      // boundary lands ON (or a hair past) the scheduled instant instead
      // of an adaptive step aliasing straight over a µs-scale edge.
      const now = ctx.simTime() + h;
      if (phase === "pending" && now >= echoRiseAt) {
        phase = "active";
        echoOut = 1;
        echoRiseAt = NaN;
      }
      if (phase === "active" && now >= echoFallAt) {
        phase = "idle";
        echoOut = 0;
        echoFallAt = NaN;
        trigLevel = 0; // fresh, ready for the next trigger cycle
      }
    }

    ctx.state.hcsr04.set(comp.id, {
      trigLevel, phase, echoRiseAt, echoFallAt, echoOut, vccV: vccVNew, pendingEchoEvent,
    });
  },
  updateCurrent: (ctx, comp, _x) => {
    // S18b — hcsr04: report the same ~2 mA typical quiescent draw that is
    // physically stamped between VCC and GND in the stamp handler.
    const hcsr04Sti = ctx.state.hcsr04.get(comp.id);
    ctx.setElementCurrent(comp.id, hcsr04Sti && hcsr04Sti.vccV >= 3.0 ? 0.002 : 0);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Derivative of the transient stamp at the committed OP: the TRIG
    // pull-down is unconditional; the ECHO push-pull stage keeps its
    // GATE_R_OUT path to the committed rail while the committed state says
    // the part is physically powered; the constant 2 mA quiescent draw
    // differentiates away.
    const trigNode = ctx.pinNode(comp.id, "trig");
    const gndNode = ctx.pinNode(comp.id, "gnd");
    stampAcAdmittance(ac, trigNode, gndNode, 1 / 1e6, 0);

    const hcSt = ctx.state.hcsr04.get(comp.id);
    const physicallyPowered = hcSt && hcSt.vccV >= 3.0
      && !ctx.isOpenPin(comp.id, "vcc")
      && !ctx.isOpenPin(comp.id, "gnd");
    if (physicallyPowered) {
      const echoNode = ctx.pinNode(comp.id, "echo");
      const railNode = hcSt.echoOut
        ? ctx.pinNode(comp.id, "vcc")
        : gndNode;
      stampAcAdmittance(ac, echoNode, railNode, 1 / GATE_R_OUT, 0);
    }
  },
};

export const ulnModel: DeviceModel = {
  kinds: ["uln2003", "uln2803"],
  // ── W6.2 ULN2003 / ULN2803 Darlington sink array ───────────────────
  stamp: (ctx, comp, xGuess, _h) => {
    // Each channel is an open-collector Darlington SINK modelled as a
    // committed-state on-resistance (R_on = 5 Ω) when sinking.
    //
    // R_on = 5 Ω is a deliberate simplification: real Darlington Vce_sat
    // is ~1 V at typical currents (0.1–0.5 A), expressed as a fixed
    // voltage drop. Modelling as a resistor means Vce_sat scales with
    // current (I × R_on), which gives ~0.5 V at 100 mA — close enough
    // to the datasheet's 1 V saturation at higher currents for simulation
    // purposes. A future wave can replace with a Thevenin voltage-source
    // if precision is needed.
    //
    // GND pin must be wired for sinking to work (mirrors _icPowerInfo check
    // for ICs with explicit VCC/GND — if GND is open, the sink path is
    // physically absent and all outputs must be treated as hi-Z).
    //
    // Every OUT also has a physical catch diode to COM (anode at OUT,
    // cathode at COM). It conducts only when an inductive turn-off lifts
    // the output above the connected supply rail.
    const vt = ctx.junctionVt();

    const ULN_RON = 5; // 5 Ω on-resistance models Vce_sat at operating currents
    const ULN_TTL_VIH = 2.5; // TTL-compatible input threshold (ULN2003A datasheet)
    const channels = comp.kind === "uln2003" ? 7 : 8;

    // The ULN has no VCC pin; power validity is determined by GND alone.
    // If GND is open the sink path is absent (no return path to supply).
    const gndNode = ctx.pinNode(comp.id, "gnd");
    const comNode = ctx.pinNode(comp.id, "com");
    const gndOpen = ctx.isOpenPin(comp.id, "gnd");

    const stUln = ctx.state.icState.get(comp.id) ?? defaultIcState(comp.kind);

    for (let ch = 1; ch <= channels; ch++) {
      const inPin  = `in${ch}`;
      const outPin = `out${ch}`;
      const outNode = ctx.pinNode(comp.id, outPin);
      if (outNode < 0) continue;

      stampClampDiode(ctx, outNode, comNode, xGuess, vt);

      // Read committed state (from previous step). NaN means first step → treat
      // as LOW (hi-Z); this is safe: on step 1 no current flows through the output.
      const committedIn = stUln[`in${ch}`] as number;
      const sinking = !gndOpen && (Number.isNaN(committedIn) ? false : committedIn >= 0.5);

      if (sinking) {
        // Sinking: stamp R_on from outPin to GND.
        // stampResistor handles gndNode=-1 correctly via MNA bounds checks —
        // but gndNode may not be -1 here: "gnd" is a local pin, not the global
        // ground net, so we must check its node index.
        stampResistor(ctx.mna, outNode, gndNode, ULN_RON);
      }
      // Released (input LOW): stamp nothing — outPin is hi-Z; external pull-up or
      // load determines the net voltage. Per-node RSHUNT prevents singularity.
    }
  },
  // W6.2 — ULN2003/ULN2803: commit input channel states post-solve.
  //
  // Each input is evaluated as a TTL-compatible logic level referenced to the
  // ULN's GND pin (not the global circuit GND — the ULN may share GND with the
  // circuit but we read the voltage difference to keep correctness when the ULN
  // GND is wired to a local ground net).
  //
  // TTL threshold: Vih ≥ 2.5 V (ULN2003A/2803A datasheet Table 1, VIH min).
  // Vil ≤ 1.5 V. We use a single-threshold commit with 0.5 V deadband:
  //   > 2.0 V → HIGH (sinking next step)
  //   < 1.5 V → LOW  (hi-Z next step)
  //   else   → hold previous (hysteresis)
  //
  // No GND pin → gndV = 0 (global reference). GND open → gndOpen flag forces
  // all outputs hi-Z via the stamp check (gndOpen is checked at stamp time,
  // not here — the committed state is still stored for the UI readout).
  commitState: (ctx, comp, x, _h) => {
    const ULN_VIH_COMMIT = 2.0; // lower bound of HIGH region (with 0.5 V margin)
    const ULN_VIL_COMMIT = 1.5; // upper bound of LOW region
    const channels = comp.kind === "uln2003" ? 7 : 8;
    const stUln = ctx.state.icState.get(comp.id) ?? defaultIcState(comp.kind);
    const gndNodeU = ctx.pinNode(comp.id, "gnd");
    const gndVU = ctx.vAt(x, gndNodeU); // voltage at the ULN GND pin

    const newSt: Record<string, number> = {};
    for (let ch = 1; ch <= channels; ch++) {
      const inPin = `in${ch}`;
      const vIn = ctx.vAt(x, ctx.pinNode(comp.id, inPin)) - gndVU;
      const prevIn = stUln[inPin] as number;
      let newIn: number;
      if (vIn >= ULN_VIH_COMMIT) {
        newIn = 1; // HIGH — output will sink next step
      } else if (vIn < ULN_VIL_COMMIT) {
        newIn = 0; // LOW — output hi-Z next step
      } else {
        // Deadband: hold previous state (hysteresis prevents rapid toggling).
        newIn = Number.isNaN(prevIn) ? 0 : prevIn;
      }
      newSt[inPin] = newIn;
    }
    ctx.state.icState.set(comp.id, newSt);
  },
  updateCurrent: (ctx, comp, x) => {
    // ── W6.2 ULN2003/ULN2803 Darlington sink arrays ─────────────────────
    // Report total sink current across all active channels.
    // When a channel sinks, current = V(outN) / R_on flows from outPin to gnd.
    // V(outN) is read from the solution; R_on = 5 Ω.
    //
    // Only report when GND pin is wired (if GND is open, no sinking occurs).
    const ULN_RON_I = 5; // must match the stamp constant above
    const channels_I = comp.kind === "uln2003" ? 7 : 8;
    const stUlnI = ctx.state.icState.get(comp.id) ?? defaultIcState(comp.kind);
    const gndOpenI = ctx.isOpenPin(comp.id, "gnd");
    const gndNodeI = ctx.pinNode(comp.id, "gnd");
    const gndVI = ctx.vAt(x, gndNodeI);
    let totalUlnI = 0;
    if (!gndOpenI) {
      for (let ch = 1; ch <= channels_I; ch++) {
        const committedInI = stUlnI[`in${ch}`] as number;
        if (!Number.isNaN(committedInI) && committedInI >= 0.5) {
          const vOutCh = ctx.vAt(x, ctx.pinNode(comp.id, `out${ch}`));
          // Current flows from vOut into the GND node through R_on.
          totalUlnI += (vOutCh - gndVI) / ULN_RON_I;
        }
      }
    }
    // Report total (sum of all sinking channels). 0 when all hi-Z or GND open.
    ctx.setElementCurrent(comp.id, totalUlnI);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Per channel, the derivative of the transient stamp at the committed
    // OP: the catch diode's guarded slope to COM always, plus the 5 ohm
    // R_on sink to the package GND pin while the committed input state says
    // the channel is sinking. Released channels are genuinely hi-Z. The
    // inputs are sampled logic — the sink decision is a committed latch, so
    // no small-signal path exists from IN to OUT.
    const vt = ctx.junctionVt();
    const ULN_RON = 5; // must match the transient stamp constant
    const channels = comp.kind === "uln2003" ? 7 : 8;
    const gndNode = ctx.pinNode(comp.id, "gnd");
    const comNode = ctx.pinNode(comp.id, "com");
    const gndOpen = ctx.isOpenPin(comp.id, "gnd");
    const stUln = ctx.state.icState.get(comp.id) ?? defaultIcState(comp.kind);
    for (let ch = 1; ch <= channels; ch++) {
      const outNode = ctx.pinNode(comp.id, `out${ch}`);
      if (outNode < 0) continue;
      acStampClampDiode(ctx, ac, outNode, comNode, vt);
      const committedIn = stUln[`in${ch}`] as number;
      const sinking = !gndOpen && (Number.isNaN(committedIn) ? false : committedIn >= 0.5);
      if (sinking) stampAcAdmittance(ac, outNode, gndNode, 1 / ULN_RON, 0);
    }
  },
};

export const buzzerModel: DeviceModel = {
  kinds: ["buzzer"],
  // ── W6.2 Buzzer (active / passive) ──────────────────────────────────
  stamp: (ctx, comp, _xGuess, _h) => {
    // Both active and passive buzzers present as a resistive load to the circuit.
    // Active buzzer: the internal oscillator is powered by DC; all the engine
    //   sees is the coil resistance (~16–32 Ω).
    // Passive buzzer: the oscillator must be driven externally at audio frequency;
    //   electrically still a resistive load at the drive frequency.
    // One stamp covers both: stampResistor(p1, p2, resistance).
    //
    // The sounding state readout (whether the buzzer is audible) is computed
    // post-solve in commitState and stored in icState for the UI.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const bR = Math.max(1, Number(comp.params.resistance ?? 32));
    stampResistor(
      ctx.mna,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      bR,
    );
  },
  // W6.2 — Buzzer: update sounding state and passive-frequency estimate.
  //
  // Active buzzer: sounding = |V(p1) − V(p2)| >= 1.5 V.
  //   The internal oscillator needs ~1.5 V DC to run. Below that threshold
  //   the oscillator stalls (most active piezo buzzers spec 2–5 V operating).
  //   We use 1.5 V as a conservative threshold that matches common 3.3 V / 5 V
  //   systems where anything above 1.5 V would run the oscillator.
  //
  // Passive buzzer: sounding = detected frequency > 0.
  //   Zero-crossing detection: every time the terminal voltage changes sign
  //   (above/below a small threshold to avoid noise near 0 V), we record
  //   the simTime and compute: Hz = 1 / (2 × half-period).
  //   This gives the fundamental frequency of the drive signal.
  //   The 1 mV threshold avoids noise triggering.
  commitState: (ctx, comp, x, h) => {
    if (!(comp.pins.length >= 2)) return;
    const stBuzz = ctx.state.icState.get(comp.id) ?? defaultIcState("buzzer");
    const vBuzz = ctx.vAt(x, ctx.pinNode(comp.id, comp.pins[0].id))
                - ctx.vAt(x, ctx.pinNode(comp.id, comp.pins[1].id));
    const buzzerType = String(comp.params.type ?? "active");
    let newLastSign = stBuzz.lastSign as number;
    let newLastCrossT = stBuzz.lastCrossT as number;
    let newDetectedHz = stBuzz.detectedHz as number;

    if (buzzerType === "passive") {
      // Passive: zero-crossing frequency detector.
      // Track sign of terminal voltage. A crossing from positive→negative or
      // negative→positive (with 1 mV noise floor) marks a half-period.
      const CROSS_THRESHOLD = 0.001; // 1 mV noise floor
      const curSign = vBuzz > CROSS_THRESHOLD ? 1 : vBuzz < -CROSS_THRESHOLD ? -1 : 0;
      if (curSign !== 0) {
        if (!Number.isNaN(newLastSign) && newLastSign !== 0 && curSign !== newLastSign) {
          // Sign changed: one half-period elapsed.
          const now = ctx.simTime() + h;
          if (!Number.isNaN(newLastCrossT) && newLastCrossT > 0 && now > newLastCrossT) {
            const halfPeriod = now - newLastCrossT;
            // hz = 1 / (2 × halfPeriod). Clamp to audio range 1 Hz – 100 kHz
            // to filter transient glitches from circuit startup noise.
            const hz = 1 / (2 * halfPeriod);
            newDetectedHz = (hz >= 1 && hz <= 100000) ? hz : 0;
          }
          newLastCrossT = ctx.simTime() + h;
        }
        newLastSign = curSign;
      }
    }
    // Store sounding flag in the same icState slot for easy UI access.
    // "sounding" is 1 when the buzzer should be audible, 0 when silent.
    let sounding: number;
    if (buzzerType === "passive") {
      sounding = newDetectedHz > 0 ? 1 : 0;
    } else {
      // Active: DC threshold check.
      sounding = Math.abs(vBuzz) >= 1.5 ? 1 : 0;
    }
    ctx.state.icState.set(comp.id, {
      lastSign: newLastSign,
      lastCrossT: newLastCrossT,
      detectedHz: newDetectedHz,
      sounding,
    });
  },
  updateCurrent: (ctx, comp, x) => {
    // ── W6.2 Buzzer ───────────────────────────────────────────────────────
    // Report current through the resistive load (V/R across the two pins).
    if (comp.pins.length < 2) { ctx.setElementCurrent(comp.id, 0); return; }
    const bRI = Math.max(1, Number(comp.params.resistance ?? 32));
    const vBuzzI = ctx.vAt(x, ctx.pinNode(comp.id, comp.pins[0].id))
                 - ctx.vAt(x, ctx.pinNode(comp.id, comp.pins[1].id));
    ctx.setElementCurrent(comp.id, vBuzzI / bRI);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same resistive load the transient stamp presents (both buzzer types).
    const pins = comp.pins;
    if (pins.length < 2) return;
    const bR = Math.max(1, Number(comp.params.resistance ?? 32));
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      1 / bR,
      0,
    );
  },
};

export const speakerModel: DeviceModel = {
  kinds: ["speaker"],
  // ── W6.2 Speaker ─────────────────────────────────────────────────────
  stamp: (ctx, comp, _xGuess, _h) => {
    // Speaker coil presented as a resistive load.
    // Voice-coil impedance is complex (Re + jωLe) but at audio frequencies
    // Le is small enough that the DC resistance dominates in a breadboard
    // simulator context. The AC reactance is ignored (same simplification as
    // the buzzer). Signal-present detection is done post-solve in commitState.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const spR = Math.max(1, Number(comp.params.resistance ?? 8));
    stampResistor(
      ctx.mna,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      spR,
    );
  },
  // W6.2 — Speaker: update signal-present detection.
  //
  // A speaker produces sound when its terminal voltage is CHANGING
  // (AC signal present). We track a short running window of the terminal
  // voltage peak and trough; if the peak-to-peak amplitude exceeds 50 mV
  // the signal is considered present ("sounding").
  //
  // Window duration: 5 ms (covers audio frequencies down to 200 Hz;
  // each window resets at windowStart + 5 ms). Short windows prevent
  // a single DC transient from latching "sounding" for too long.
  commitState: (ctx, comp, x, h) => {
    if (!(comp.pins.length >= 2)) return;
    const stSpk = ctx.state.icState.get(comp.id) ?? defaultIcState("speaker");
    const vSpk = ctx.vAt(x, ctx.pinNode(comp.id, comp.pins[0].id))
               - ctx.vAt(x, ctx.pinNode(comp.id, comp.pins[1].id));
    const now = ctx.simTime() + h;
    const SPK_WINDOW = 0.005; // 5 ms measurement window
    const SPK_THRESHOLD = 0.05; // 50 mV peak-to-peak to declare signal present

    let vPkSpk = stSpk.vPeak as number;
    let vTrSpk = stSpk.vTrough as number;
    let winStart = stSpk.windowStart as number;
    let p2p = stSpk.peakToPeak as number;

    if (Number.isNaN(winStart)) {
      // Initialise window on first sample.
      winStart = now;
      vPkSpk = vSpk;
      vTrSpk = vSpk;
    } else if (now - winStart >= SPK_WINDOW) {
      // Window expired: commit the accumulated peak-to-peak and start fresh.
      p2p = Number.isNaN(vPkSpk) || Number.isNaN(vTrSpk) ? 0 : (vPkSpk - vTrSpk);
      winStart = now;
      vPkSpk = vSpk;
      vTrSpk = vSpk;
    } else {
      // Update running extrema.
      if (Number.isNaN(vPkSpk) || vSpk > vPkSpk) vPkSpk = vSpk;
      if (Number.isNaN(vTrSpk) || vSpk < vTrSpk) vTrSpk = vSpk;
    }

    const spkSounding = p2p >= SPK_THRESHOLD ? 1 : 0;
    ctx.state.icState.set(comp.id, {
      vPeak: vPkSpk,
      vTrough: vTrSpk,
      windowStart: winStart,
      peakToPeak: p2p,
      sounding: spkSounding,
    });
  },
  updateCurrent: (ctx, comp, x) => {
    // ── W6.2 Speaker ─────────────────────────────────────────────────────
    // Report current through the resistive voice-coil model (V/R).
    if (comp.pins.length < 2) { ctx.setElementCurrent(comp.id, 0); return; }
    const spRI = Math.max(1, Number(comp.params.resistance ?? 8));
    const vSpkI = ctx.vAt(x, ctx.pinNode(comp.id, comp.pins[0].id))
                - ctx.vAt(x, ctx.pinNode(comp.id, comp.pins[1].id));
    ctx.setElementCurrent(comp.id, vSpkI / spRI);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same resistive voice-coil load as the transient stamp (the deliberate
    // Re-only simplification; no motional impedance model exists to mirror).
    const pins = comp.pins;
    if (pins.length < 2) return;
    const spR = Math.max(1, Number(comp.params.resistance ?? 8));
    stampAcAdmittance(
      ac,
      ctx.pinNode(comp.id, pins[0].id),
      ctx.pinNode(comp.id, pins[1].id),
      1 / spR,
      0,
    );
  },
};

// W6.3 — L293D / TB6612: commit output drive states post-solve.
//
// For each output pin the drive state is {HIGH, LOW, HIZ} and is committed
// once per timestep (post-solve, stable x) then held through the next
// Newton loop.  This is the standard committed-state pattern that prevents
// chatter when input/enable nodes are still settling during Newton iteration.
//
// L293D: 4 outputs, two enable groups (EN12 → OUT1/OUT2, EN34 → OUT3/OUT4).
//   Drive state per output n:
//     enable HIGH AND VM valid → follow INn: HIGH if INn high, LOW if INn low.
//     enable LOW              → HIZ (coast).
//     VM not valid            → HIZ (motor rail absent, no drive path).
//     NaN (first step)        → HIZ (safe initial state, no current flows).
//
// TB6612: 4 outputs (AO1/AO2/BO1/BO2), per-channel STBY + PWM controls.
//   Drive state per output pair (xO1/xO2):
//     STBY LOW  → both HIZ (standby).
//     PWMX LOW  → both HIZ (coast, treated as coast in this DC model).
//     PWMX HIGH AND STBY HIGH → follow xIN1/xIN2 truth table:
//       (1,0) forward: xO1 HIGH, xO2 LOW
//       (0,1) reverse: xO1 LOW,  xO2 HIGH
//       (1,1) brake:   xO1 LOW,  xO2 LOW
//       (0,0) coast:   both HIZ
//     VM not valid → all HIZ.
//
// State stored as "outN": 1=HIGH, 0=LOW, -1=HIZ (using -1 so 0/1/NaN
// are unambiguous — a single numeric slot per output, not a string enum).
//
// Logic supply validity: VCC1 (l293d) / VCC (tb6612) check via _icPowerInfo.
// Motor rail validity:   V(VM pin) − V(GND pin) ≥ 1.5 V (local read, dual-rail).

export const l293dModel: DeviceModel = {
  kinds: ["l293d"],
  // ── W6.3 L293D Dual H-Bridge ─────────────────────────────────────────
  stamp: (ctx, comp, xGuess, _h) => {
    // Push-pull output stage with committed drive state (HIGH / LOW / HIZ).
    //
    // R_on = 2 Ω per output side.  Derivation: the L293D datasheet specifies
    // ~0.7 V voltage drop per transistor at 1 A, giving ~0.7 Ω each.  Two
    // transistors in series (source and sink) give ~1.4 Ω total.  We use 2 Ω
    // as the teaching value because it:
    //   (a) gives a clearly visible voltage drop at motor currents, and
    //   (b) matches the locked R_on spec for all H-bridge kinds in this wave.
    //
    // No branch rows are added to _buildMatrix — pure committed-state resistor
    // stamps.  The drive state is decided once per step in commitState and
    // read here as committed state from icState.
    //
    // Dual-rail: motor rail validity is stored in icState.vm by commitState.
    // If icState is absent (first step) all outputs are treated as HIZ.
    const vt = ctx.junctionVt();

    const H_BRIDGE_RON = 2; // 2 Ω per output side (locked teaching value)

    const stL293 = ctx.state.icState.get(comp.id) ?? defaultIcState("l293d");
    const vmNodeL293  = ctx.pinNode(comp.id, "vcc2");
    const gndNodeL293 = ctx.pinNode(comp.id, "gnd1");

    // L293D integrates two freewheel diodes per output: GND→OUT and
    // OUT→VM. They remain present in drive and HIZ states, so coast and
    // supply pumping are solved electrically instead of only diagnosed.
    for (const outPin of ["out1", "out2", "out3", "out4"]) {
      const outNode = ctx.pinNode(comp.id, outPin);
      stampClampDiode(ctx, gndNodeL293, outNode, xGuess, vt);
      stampClampDiode(ctx, outNode, vmNodeL293, xGuess, vt);
    }

    // Stamp each output according to its committed drive state.
    // The four outputs share the same VM and GND nodes.
    const stampL293Output = (outPin: string, driveState: number): void => {
      const outNode = ctx.pinNode(comp.id, outPin);
      if (outNode < 0) return;
      if (driveState === 1) {
        // HIGH: pull toward VM through R_on.
        if (!ctx.isOpenPin(comp.id, "vcc2")) {
          stampResistor(ctx.mna, outNode, vmNodeL293, H_BRIDGE_RON);
        }
      } else if (driveState === 0) {
        // LOW: pull toward GND through R_on.
        stampResistor(ctx.mna, outNode, gndNodeL293, H_BRIDGE_RON);
      }
      // HIZ (-1): stamp nothing; RSHUNT provides a numerical reference.
    };

    stampL293Output("out1", stL293.out1 as number ?? -1);
    stampL293Output("out2", stL293.out2 as number ?? -1);
    stampL293Output("out3", stL293.out3 as number ?? -1);
    stampL293Output("out4", stL293.out4 as number ?? -1);
  },
  commitState: (ctx, comp, x, _h) => {
    const powerL = ctx.icPowerInfo(comp, x);
    // Motor rail: read VM pin (pin id "vcc2") relative to GND pin.
    // We use one of the GND pins (gnd1) as the ground reference; all four
    // GND pins are tied together on the physical IC so any one suffices.
    const vmNodeL  = ctx.pinNode(comp.id, "vcc2");
    const gndNodeL = ctx.pinNode(comp.id, "gnd1");
    const vmOpenL  = ctx.isOpenPin(comp.id, "vcc2");
    const vmVoltL  = vmOpenL ? 0 : (ctx.vAt(x, vmNodeL) - ctx.vAt(x, gndNodeL));
    const vmValidL = powerL.powered && !vmOpenL && vmVoltL >= 1.5;

    // Input threshold: TTL Vih ≥ 2.0 V (L293D datasheet: Vih min = 2.3 V;
    // we use 2.0 V as the committed commit threshold with 0.5 V margin).
    const L293D_VIH = 2.0;
    const L293D_VIL = 1.5;

    const gndVL = ctx.vAt(x, gndNodeL);

    /** Committed logic high for L293D input/enable (relative to GND pin). */
    const l293dHigh = (pinId: string): boolean => {
      if (ctx.isOpenPin(comp.id, pinId)) return false; // floating = LOW
      const vPin = ctx.vAt(x, ctx.pinNode(comp.id, pinId)) - gndVL;
      return vPin >= L293D_VIH;
    };

    const en12High = l293dHigh("en12");
    const en34High = l293dHigh("en34");
    const in1High  = l293dHigh("in1");
    const in2High  = l293dHigh("in2");
    const in3High  = l293dHigh("in3");
    const in4High  = l293dHigh("in4");

    // Drive state: 1=HIGH, 0=LOW, -1=HIZ
    // Determine per-output committed state; first step NaN → HIZ handled
    // by the stamp reading icState (NaN entry absent → -1 fallback).
    const L293D_HIZ  = -1;
    const L293D_HIGH =  1;
    const L293D_LOW  =  0;

    const out1 = (!vmValidL || !en12High) ? L293D_HIZ : (in1High ? L293D_HIGH : L293D_LOW);
    const out2 = (!vmValidL || !en12High) ? L293D_HIZ : (in2High ? L293D_HIGH : L293D_LOW);
    const out3 = (!vmValidL || !en34High) ? L293D_HIZ : (in3High ? L293D_HIGH : L293D_LOW);
    const out4 = (!vmValidL || !en34High) ? L293D_HIZ : (in4High ? L293D_HIGH : L293D_LOW);

    ctx.state.icState.set(comp.id, { out1, out2, out3, out4, vm: vmVoltL });
  },
  updateCurrent: (ctx, comp, x) => {
    // ── W6.3 L293D / TB6612 H-bridge motor drivers ───────────────────────
    // Report total current sourced/sunk across all active output channels.
    // Current is computed from the stamped resistor conductance:
    //   HIGH channel: I = (V_vm − V_out) / R_on  (sourcing from VM)
    //   LOW channel:  I = (V_out − V_gnd) / R_on  (sinking to GND)
    // HIZ channels contribute 0 current.
    const H_BRIDGE_RON_I = 2;
    const stL293I = ctx.state.icState.get(comp.id) ?? defaultIcState("l293d");
    const vmNodeL293I  = ctx.pinNode(comp.id, "vcc2");
    const gndNodeL293I = ctx.pinNode(comp.id, "gnd1");
    const vmVL293I  = ctx.vAt(x, vmNodeL293I);
    const gndVL293I = ctx.vAt(x, gndNodeL293I);
    const l293dOutPins = ["out1", "out2", "out3", "out4"];
    const l293dStates  = [
      stL293I.out1 as number,
      stL293I.out2 as number,
      stL293I.out3 as number,
      stL293I.out4 as number,
    ];
    let totalL293I = 0;
    for (let oi = 0; oi < 4; oi++) {
      const ds = l293dStates[oi];
      if (ds === 1) {
        const vOut = ctx.vAt(x, ctx.pinNode(comp.id, l293dOutPins[oi]));
        totalL293I += (vmVL293I - vOut) / H_BRIDGE_RON_I;
      } else if (ds === 0) {
        const vOut = ctx.vAt(x, ctx.pinNode(comp.id, l293dOutPins[oi]));
        totalL293I += (vOut - gndVL293I) / H_BRIDGE_RON_I;
      }
    }
    ctx.setElementCurrent(comp.id, totalL293I);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Per output, the derivative of the transient stamp at the committed
    // OP: both freewheel clamp diodes (GND->OUT and OUT->VM) always, plus
    // the 2 ohm R_on to the committed rail for a driven output. HIZ outputs
    // are genuinely released. Inputs/enables are sampled logic — the drive
    // decision is a committed latch with no small-signal path to the stage.
    const vt = ctx.junctionVt();
    const H_BRIDGE_RON = 2; // must match the transient stamp constant
    const stL293 = ctx.state.icState.get(comp.id) ?? defaultIcState("l293d");
    const vmNode = ctx.pinNode(comp.id, "vcc2");
    const gndNode = ctx.pinNode(comp.id, "gnd1");
    const vmOpen = ctx.isOpenPin(comp.id, "vcc2");
    for (const outPin of ["out1", "out2", "out3", "out4"]) {
      const outNode = ctx.pinNode(comp.id, outPin);
      acStampClampDiode(ctx, ac, gndNode, outNode, vt);
      acStampClampDiode(ctx, ac, outNode, vmNode, vt);
      if (outNode < 0) continue;
      const driveState = (stL293[outPin] as number | undefined) ?? -1;
      if (driveState === 1 && !vmOpen) {
        stampAcAdmittance(ac, outNode, vmNode, 1 / H_BRIDGE_RON, 0);
      } else if (driveState === 0) {
        stampAcAdmittance(ac, outNode, gndNode, 1 / H_BRIDGE_RON, 0);
      }
    }
  },
};

export const tb6612Model: DeviceModel = {
  kinds: ["tb6612"],
  // ── W6.3 TB6612FNG Dual H-Bridge ─────────────────────────────────────
  stamp: (ctx, comp, xGuess, _h) => {
    // Same push-pull committed-state output stage as L293D.
    // R_on = 2 Ω per side (same locked value — real TB6612 is ~0.5 Ω but
    // we use the shared H-bridge teaching value for pedagogical consistency).
    //
    // The four outputs (ao1/ao2/bo1/bo2) are stamped from the committed
    // drive state in icState.  No branch rows (checklist item 7: no branch
    // rows for H-bridge push-pull output stages).
    const vt = ctx.junctionVt();

    const TB_RON = 2; // 2 Ω per output side (locked teaching value, matches L293D)

    const stTb = ctx.state.icState.get(comp.id) ?? defaultIcState("tb6612");
    const vmNodeTb6  = ctx.pinNode(comp.id, "vm");
    const gndNodeTb6 = ctx.pinNode(comp.id, "gnd");

    // MOSFET body/freewheel paths in both half-bridges. The teaching
    // model uses the same silicon catch curve as L293D so inductive
    // current always has a real MNA path during coast/dead time.
    for (const outPin of ["ao1", "ao2", "bo1", "bo2"]) {
      const outNode = ctx.pinNode(comp.id, outPin);
      stampClampDiode(ctx, gndNodeTb6, outNode, xGuess, vt);
      stampClampDiode(ctx, outNode, vmNodeTb6, xGuess, vt);
    }

    const stampTbOutput = (outPin: string, driveState: number): void => {
      const outNode = ctx.pinNode(comp.id, outPin);
      if (outNode < 0) return;
      if (driveState === 1) {
        if (!ctx.isOpenPin(comp.id, "vm")) {
          stampResistor(ctx.mna, outNode, vmNodeTb6, TB_RON);
        }
      } else if (driveState === 0) {
        stampResistor(ctx.mna, outNode, gndNodeTb6, TB_RON);
      }
      // HIZ: stamp nothing.
    };

    stampTbOutput("ao1", stTb.ao1 as number ?? -1);
    stampTbOutput("ao2", stTb.ao2 as number ?? -1);
    stampTbOutput("bo1", stTb.bo1 as number ?? -1);
    stampTbOutput("bo2", stTb.bo2 as number ?? -1);
  },
  commitState: (ctx, comp, x, _h) => {
    const powerTb = ctx.icPowerInfo(comp, x);
    // Motor rail: read VM pin relative to GND pin (id "gnd").
    const vmNodeTb  = ctx.pinNode(comp.id, "vm");
    const gndNodeTb = ctx.pinNode(comp.id, "gnd");
    const vmOpenTb  = ctx.isOpenPin(comp.id, "vm");
    const vmVoltTb  = vmOpenTb ? 0 : (ctx.vAt(x, vmNodeTb) - ctx.vAt(x, gndNodeTb));
    const vmValidTb = powerTb.powered && !vmOpenTb && vmVoltTb >= 1.5;

    // TB6612 logic supply threshold (CMOS-compatible, 2.7–5.5 V VCC).
    // We use the _icPowerInfo thresholds for input decisions so the same
    // supply-ratioed logic applies as for other CMOS parts.
    const gndVTb = ctx.vAt(x, gndNodeTb);

    const tb6612High = (pinId: string): boolean => {
      if (ctx.isOpenPin(comp.id, pinId)) return false;
      const vPin = ctx.vAt(x, ctx.pinNode(comp.id, pinId)) - gndVTb;
      return vPin >= powerTb.thresholds.vih;
    };

    const stbyHigh = tb6612High("stby");
    const pwmaHigh = tb6612High("pwma");
    const pwmbHigh = tb6612High("pwmb");
    const ain1High = tb6612High("ain1");
    const ain2High = tb6612High("ain2");
    const bin1High = tb6612High("bin1");
    const bin2High = tb6612High("bin2");

    const TB_HIZ  = -1;
    const TB_HIGH =  1;
    const TB_LOW  =  0;

    // Channel A: ao1 / ao2
    let ao1: number;
    let ao2: number;
    if (!vmValidTb || !stbyHigh) {
      ao1 = TB_HIZ; ao2 = TB_HIZ;
    } else if (!pwmaHigh) {
      // PWM LOW → coast (hi-Z).  In DC model PWM is a binary enable.
      ao1 = TB_HIZ; ao2 = TB_HIZ;
    } else {
      // PWM HIGH + STBY HIGH: truth table on AIN1/AIN2.
      if (ain1High && !ain2High) {
        ao1 = TB_HIGH; ao2 = TB_LOW;          // forward
      } else if (!ain1High && ain2High) {
        ao1 = TB_LOW;  ao2 = TB_HIGH;          // reverse
      } else if (ain1High && ain2High) {
        ao1 = TB_LOW;  ao2 = TB_LOW;           // brake
      } else {
        ao1 = TB_HIZ; ao2 = TB_HIZ;            // coast (0,0)
      }
    }

    // Channel B: bo1 / bo2
    let bo1: number;
    let bo2: number;
    if (!vmValidTb || !stbyHigh) {
      bo1 = TB_HIZ; bo2 = TB_HIZ;
    } else if (!pwmbHigh) {
      bo1 = TB_HIZ; bo2 = TB_HIZ;
    } else {
      if (bin1High && !bin2High) {
        bo1 = TB_HIGH; bo2 = TB_LOW;           // forward
      } else if (!bin1High && bin2High) {
        bo1 = TB_LOW;  bo2 = TB_HIGH;          // reverse
      } else if (bin1High && bin2High) {
        bo1 = TB_LOW;  bo2 = TB_LOW;           // brake
      } else {
        bo1 = TB_HIZ; bo2 = TB_HIZ;            // coast
      }
    }

    ctx.state.icState.set(comp.id, { ao1, ao2, bo1, bo2, vm: vmVoltTb });
  },
  updateCurrent: (ctx, comp, x) => {
    // Same current readout pattern as L293D.
    const TB_RON_I = 2;
    const stTbI = ctx.state.icState.get(comp.id) ?? defaultIcState("tb6612");
    const vmNodeTbI  = ctx.pinNode(comp.id, "vm");
    const gndNodeTbI = ctx.pinNode(comp.id, "gnd");
    const vmVTbI  = ctx.vAt(x, vmNodeTbI);
    const gndVTbI = ctx.vAt(x, gndNodeTbI);
    const tbOutPins  = ["ao1", "ao2", "bo1", "bo2"];
    const tbStates   = [
      stTbI.ao1 as number,
      stTbI.ao2 as number,
      stTbI.bo1 as number,
      stTbI.bo2 as number,
    ];
    let totalTbI = 0;
    for (let oi = 0; oi < 4; oi++) {
      const ds = tbStates[oi];
      if (ds === 1) {
        const vOut = ctx.vAt(x, ctx.pinNode(comp.id, tbOutPins[oi]));
        totalTbI += (vmVTbI - vOut) / TB_RON_I;
      } else if (ds === 0) {
        const vOut = ctx.vAt(x, ctx.pinNode(comp.id, tbOutPins[oi]));
        totalTbI += (vOut - gndVTbI) / TB_RON_I;
      }
    }
    ctx.setElementCurrent(comp.id, totalTbI);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same shape as the l293d acStamp: freewheel clamp pair per output plus
    // the committed 2 ohm R_on drive, HIZ outputs released.
    const vt = ctx.junctionVt();
    const TB_RON = 2; // must match the transient stamp constant
    const stTb = ctx.state.icState.get(comp.id) ?? defaultIcState("tb6612");
    const vmNode = ctx.pinNode(comp.id, "vm");
    const gndNode = ctx.pinNode(comp.id, "gnd");
    const vmOpen = ctx.isOpenPin(comp.id, "vm");
    for (const outPin of ["ao1", "ao2", "bo1", "bo2"]) {
      const outNode = ctx.pinNode(comp.id, outPin);
      acStampClampDiode(ctx, ac, gndNode, outNode, vt);
      acStampClampDiode(ctx, ac, outNode, vmNode, vt);
      if (outNode < 0) continue;
      const driveState = (stTb[outPin] as number | undefined) ?? -1;
      if (driveState === 1 && !vmOpen) {
        stampAcAdmittance(ac, outNode, vmNode, 1 / TB_RON, 0);
      } else if (driveState === 0) {
        stampAcAdmittance(ac, outNode, gndNode, 1 / TB_RON, 0);
      }
    }
  },
};
