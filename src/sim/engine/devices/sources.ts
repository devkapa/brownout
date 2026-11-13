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

// A linear bench supply's output stage is a series pass element: it sources
// current and has no path to sink it beyond its own output capacitor and
// bleeder, which classic does not model. Driven above its setpoint by another
// source, it lets go of the output (committed regime 3, branch current pinned
// to zero) and takes the output back once the output falls below the
// setpoint. The regime is frozen for a Newton solve and decided on a
// converged point: the dc operating point re-solves until it settles, the
// load seed re-solves once per switch, and a transient step that crosses
// into the other regime is split at the crossing (SimEngine._solveSegment).
// A supply wired across the same node pair as another ideal source (two
// supplies in parallel) also gets a 1 uohm series resistance in CV: two ideal
// sources on one node pair are a singular system that never converged.
// 1 uohm moves a reading by 1 uV per amp; a supply on its own stays ideal.
const BENCH_PSU_R_OUT = 1e-6;
/** Sink current above which a supply at its setpoint is being driven off. */
const BENCH_PSU_SINK_ENTRY_A = 1e-6;
export const BENCH_PSU_BACKDRIVEN = 3;

interface BenchPsuReading {
  k: number;
  /** Setpoint magnitude (scaled during source stepping). */
  setpoint: number;
  /** Output voltage in the source's own direction. */
  output: number;
  /** Branch current in the source's own direction, positive when sinking. */
  sink: number;
  reg: number;
  /** Output tolerance below the setpoint. */
  tol: number;
}

function readBenchPsu(ctx: DeviceContext, comp: DeviceComponent, x: Float64Array): BenchPsuReading | null {
  const pins = comp.pins;
  if (pins.length < 2) return null;
  const k = ctx.vsrcRow(comp.id);
  if (k === undefined) return null;
  const v = ctx.independentSourceMagnitude(Number(comp.params.voltage ?? 5));
  const dir = v < 0 ? -1 : 1;
  const st = ctx.state.icState.get(comp.id);
  return {
    k,
    setpoint: Math.abs(v),
    output: dir * (ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id)) - ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id))),
    sink: dir * (x[k] ?? 0),
    reg: st ? (st.reg ?? 0) : 0,
    tol: 1e-6 * Math.max(1, Math.abs(v)),
  };
}

/**
 * The let-go / take-back regime a converged solution asks for, or null when
 * the committed one stands. Source-stepping rungs are not physical states
 * (every setpoint is scaled toward 0 V), so no switch is asked there.
 */
export function benchPsuSinkRegimeWanted(
  ctx: DeviceContext,
  comp: DeviceComponent,
  x: Float64Array,
): 0 | 3 | null {
  if (ctx.independentSourceMagnitude(1) !== 1) return null;
  const r = readBenchPsu(ctx, comp, x);
  if (!r) return null;
  if (r.reg === BENCH_PSU_BACKDRIVEN) return r.output < r.setpoint - r.tol ? 0 : null;
  if (r.reg === 2) return null;
  return r.sink > BENCH_PSU_SINK_ENTRY_A && r.output >= r.setpoint - r.tol ? BENCH_PSU_BACKDRIVEN : null;
}

/**
 * Where in a step from x0 to x1 the supply crossed into the wanted regime,
 * as a fraction of the step: where its branch current crossed into sinking
 * (let-go), or where its output fell through the setpoint (take-back). 0 when
 * it was already past the crossing at the start.
 */
export function benchPsuCrossingFraction(
  ctx: DeviceContext,
  comp: DeviceComponent,
  want: 0 | 3,
  x0: Float64Array | null,
  x1: Float64Array,
): number {
  const r1 = readBenchPsu(ctx, comp, x1);
  const r0 = x0 && x0.length === x1.length ? readBenchPsu(ctx, comp, x0) : null;
  if (!r0 || !r1) return 0;
  const [a, b, level] = want === BENCH_PSU_BACKDRIVEN
    ? [r0.sink, r1.sink, BENCH_PSU_SINK_ENTRY_A]
    : [-r0.output, -r1.output, -(r1.setpoint - r1.tol)];
  if (a >= level || b === a) return 0;
  return Math.min(1, Math.max(0, (level - a) / (b - a)));
}

/** Commit a let-go / take-back switch decided by the engine. */
export function setBenchPsuSinkRegime(ctx: DeviceContext, comp: DeviceComponent, reg: 0 | 3): void {
  if (reg === 0 && !(Number(comp.params.iLimit ?? 0) > 0)) ctx.state.icState.delete(comp.id);
  else ctx.state.icState.set(comp.id, { reg });
}

export const benchPsuModel: DeviceModel = {
  kinds: ["bench_psu"],
  branchRows: (comp, _ctx) => [comp.id],
  stamp: (ctx, comp, xGuess, _h) => {
    // W5.1: bench_psu gains optional constant-current fold.
    // When params.iLimit is present and > 0, the committed-regime pattern
    // is used (CV mode = ideal vsource; CC mode = branch current pinned to iLimit).
    const pins = comp.pins;
    if (pins.length < 2) return;
    const kPsu = ctx.vsrcRow(comp.id);
    if (kPsu === undefined) return;
    // Scaled once here so the CC compliance comparison and the CV stamp
    // see one consistent per-rung setpoint during source stepping.
    const Vpsu = ctx.independentSourceMagnitude(Number(comp.params.voltage ?? 5));
    const iLimitPsu = Number(comp.params.iLimit ?? 0);
    const posNode = ctx.pinNode(comp.id, pins[0].id);
    const negNode = ctx.pinNode(comp.id, pins[1].id);
    const sourceDirection = Vpsu < 0 ? -1 : 1;
    const stPsu = ctx.state.icState.get(comp.id);
    const regPsu = stPsu ? (stPsu.reg ?? 0) : 0;

    if (regPsu === BENCH_PSU_BACKDRIVEN) {
      // Let go: keep the KCL incidence, pin the branch current to zero.
      if (posNode >= 0) ctx.mna.add(posNode, kPsu, 1);
      if (negNode >= 0) ctx.mna.add(negNode, kPsu, -1);
      ctx.mna.add(kPsu, kPsu, 1);
      return;
    }

    if (iLimitPsu > 0) {
      // iLimit is active — check committed regime (0=CV, 2=CC).
      const outputMagnitudeGuess = sourceDirection * (
        ctx.vAt(xGuess, posNode) - ctx.vAt(xGuess, negNode)
      );
      const activeSetKey = `bench_psu:${comp.id}`;
      const entryClamp = regPsu !== 2 && ctx.useCurrentLimitEntryClamp(
        activeSetKey,
        // Only delivered current counts against the limit; a sinking supply
        // lets go instead (see BENCH_PSU_BACKDRIVEN).
        Math.max(0, -sourceDirection * (xGuess[kPsu] ?? 0)),
        iLimitPsu,
      );
      const currentLimited = regPsu === 2 || entryClamp;
      // CV takes over only once the CC candidate reaches the setpoint itself,
      // not the shared band just under it. With a capacitor on the rail an
      // overloaded supply's CC candidate falls only (excess current) * h / C
      // per step, so at fine steps the band held it at the setpoint: the
      // supply sat in CV past its limit with reg flipping every step. For a
      // load whose draw rises with voltage, a candidate under the setpoint
      // means the draw at the setpoint exceeds the limit, so CC is right for
      // it. A removed load still sends the candidate far above the setpoint,
      // so CV holds the rail in that same solve.
      const complianceClamp = currentLimited && ctx.useCurrentLimitComplianceClamp(
        activeSetKey,
        outputMagnitudeGuess,
        Math.abs(Vpsu),
        0,
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
        return;
      }
    }
    // CV mode, or the same-step compliance branch selected from a
    // provisional CC candidate. The ordinary voltage-source branch current
    // is the actual terminal current, so source power and telemetry never
    // retain the stale full CC current after recovery.
    if (ctx.sharesIdealSourceNodePair(comp.id)) {
      stampVSourceSeriesR(ctx.mna, posNode, negNode, kPsu, Vpsu, BENCH_PSU_R_OUT);
    } else {
      stampVSource(ctx.mna, posNode, negNode, kPsu, Vpsu);
    }
  },
  commitState: (ctx, comp, x, _h) => {
    const kBpsu = ctx.vsrcRow(comp.id);
    if (kBpsu === undefined) return;
    const Vpsu = ctx.independentSourceMagnitude(Number(comp.params.voltage ?? 5));
    const sourceDirection = Vpsu < 0 ? -1 : 1;
    const stBpsu = ctx.state.icState.get(comp.id);
    const prevRegBpsu = stBpsu ? (stBpsu.reg ?? 0) : 0;
    // The dc operating point decides let-go / take-back on each converged
    // solve and re-solves until the picture settles. Transient and the load
    // seed leave it to the engine (see benchPsuSinkRegimeWanted).
    let newRegBpsu = prevRegBpsu;
    if (ctx.dcSolveMode()) {
      const want = benchPsuSinkRegimeWanted(ctx, comp, x);
      if (want !== null) newRegBpsu = want;
    }
    // W5.1 — bench_psu iLimit commit: only when iLimit is active.
    const iLimitBpsu = Number(comp.params.iLimit ?? 0);
    if (newRegBpsu !== BENCH_PSU_BACKDRIVEN && iLimitBpsu > 0) {
      const iBpsu = x[kBpsu] ?? 0;  // negative when delivering current (MNA convention)
      const activeSetKeyBpsu = `bench_psu:${comp.id}`;
      if (prevRegBpsu === BENCH_PSU_BACKDRIVEN) {
        // Just taken back: start in CV; an overload enters CC on the next solve.
        newRegBpsu = 0;
      } else if (
        ctx.currentLimitEntryClampActive(activeSetKeyBpsu)
        && !ctx.currentLimitComplianceClampActive(activeSetKeyBpsu)
      ) {
        newRegBpsu = 2;
      } else if (prevRegBpsu === 2) {
        // In CC mode: x[k] is forced to −iLimit; cannot use |i| to detect recovery.
        // Exit CC exactly when this solve's CC candidate reached the setpoint
        // and the compliance branch took over: for a load whose draw rises
        // with voltage, that is when the natural (CV) current at V is ≤ iLimit.
        // Reusing the solve's own decision keeps the exit on the stamp's
        // threshold. An exit that fires anywhere below the setpoint lets a
        // slightly overloaded supply, whose CC rail sits just under V, leave
        // CC here and re-enter it in the next solve, flipping reg every step,
        // which the adaptive controller rejects as a regime change on every
        // other step.
        newRegBpsu = ctx.currentLimitComplianceClampActive(activeSetKeyBpsu) ? 0 : 2;
      } else {
        // In CV: enter CC if the delivered current exceeds the limit.
        newRegBpsu = -sourceDirection * iBpsu > iLimitBpsu ? 2 : 0;
      }
    }
    if (newRegBpsu === 0 && iLimitBpsu <= 0) {
      // An unlimited supply keeps no regime entry while in CV.
      ctx.state.icState.delete(comp.id);
    } else {
      ctx.state.icState.set(comp.id, { reg: newRegBpsu });
    }
  },
  updateCurrent: (ctx, comp, x) => {
    publishSourceBranchCurrent(ctx, comp, x);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Linearize at the committed regime: CV is an ideal AC short (branch row
    // with V = 0, or 1 for the designated input); a committed CC fold or a
    // let-go output pins the branch current, so its small-signal current is
    // exactly zero and no drive can inject through it.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const k = ctx.vsrcRow(comp.id);
    if (k === undefined) return;
    const posNode = ctx.pinNode(comp.id, pins[0].id);
    const negNode = ctx.pinNode(comp.id, pins[1].id);
    const st = ctx.state.icState.get(comp.id);
    const reg = st ? (st.reg ?? 0) : 0;
    if (reg === 2 || reg === BENCH_PSU_BACKDRIVEN) {
      // Keep the KCL coupling incidence, constrain dI = 0.
      ac.addAc(posNode, k, 1, 0);
      ac.addAc(negNode, k, -1, 0);
      ac.addAc(k, k, 1, 0);
      return;
    }
    stampAcVoltageSourceRow(ac, posNode, negNode, k, ctx.acInputMagnitude(comp.id), 0);
    // The same 1 uohm the transient CV stamp carries.
    if (ctx.sharesIdealSourceNodePair(comp.id)) ac.addAc(k, k, -BENCH_PSU_R_OUT, 0);
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
  // Sign convention is a battery's, not the SPICE I-element's: params.current
  // leaves the pos pin, flows through the external circuit and returns into
  // the neg pin, so pos is the terminal the source drives positive — what a
  // part labelled "+" promises. SPICE names the terminals the other way round
  // (current flows from n+ THROUGH the source to n-), so the netlist parser
  // attaches an I card's n+ to neg and its n- to pos; "I1 0 out 1m" still
  // lifts node out positive. stampCurrentSource(i, j, I) injects INTO i and
  // OUT of j, hence the (pos, neg) argument order below.
  stamp: (ctx, comp, _xGuess, _h) => {
    const pins = comp.pins;
    if (pins.length < 2) return;
    const posNode = ctx.pinNode(comp.id, pins[0].id);
    const negNode = ctx.pinNode(comp.id, pins[1].id);
    // Scaled like every independent drive so the Wave A3 source-stepping
    // ladder relaxes I sources together with V sources.
    const I = ctx.independentSourceMagnitude(Number(comp.params.current ?? 0));
    stampCurrentSource(ctx.mna, posNode, negNode, I);
    const rParallel = Number(comp.params.rParallel ?? 0);
    if (Number.isFinite(rParallel) && rParallel > 0) {
      stampResistor(ctx.mna, posNode, negNode, rParallel);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    // Element-current convention is pin0 -> pin1 (pos -> neg THROUGH the
    // component), the passive frame every two-terminal part publishes in.
    // The ideal branch carries the programmed current the other way (it
    // flows neg -> pos inside and leaves pos), hence -I, plus the declared
    // parallel shunt's terminal current. The homotopy scale is applied so a
    // mid-ladder rung publishes the same drive it stamped (a voltage source's
    // x[k] reflects the scaled stamp the same way); at any accepted point the
    // scale is 1 and this is exact.
    const pins = comp.pins;
    if (pins.length < 2) return;
    const I = ctx.independentSourceMagnitude(Number(comp.params.current ?? 0));
    const rParallel = Number(comp.params.rParallel ?? 0);
    if (Number.isFinite(rParallel) && rParallel > 0) {
      const vd = ctx.vAt(x, ctx.pinNode(comp.id, pins[0].id))
        - ctx.vAt(x, ctx.pinNode(comp.id, pins[1].id));
      ctx.setElementCurrent(comp.id, -I + vd / rParallel);
      return;
    }
    ctx.setElementCurrent(comp.id, -I);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The SPICE small-signal convention for an I element: AC-zeroed it is an
    // OPEN (a constant-current branch has zero admittance — nothing stamps),
    // and as the designated input it injects the unit 1 A drive with the
    // transient stamp's polarity (into the pos node, out of neg). The declared
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
      ac.addBAc(posNode, magnitude, 0);
      ac.addBAc(negNode, -magnitude, 0);
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
