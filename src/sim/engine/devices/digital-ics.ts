/**
 * DIGITAL-ICS device cohort (Wave A4 phase 2): ne555, 74ls00, 74ls04,
 * 74ls08, 74ls32, 74ls86, 74ls157, 74ls283, 74ls245, 74hc14, 74hc138,
 * 74ls47, 74ls161, 74ls173, 74ls189, 74hc595, 74hc165, 74hc74, cd4017,
 * cd4060, cd4511, seg7_cc, seg7_ca, hd44780, max7219, 28c16, 28c256.
 *
 * Every handler body below is the engine's old switch-case body MOVED
 * VERBATIM, with identifier access adapted through DeviceContext
 * (this._pinNode -> ctx.pinNode, this._icPowerInfo -> ctx.icPowerInfo,
 * this._logicHigh -> ctx.logicHigh, this.state.icState ->
 * ctx.state.icState, this.simTime -> ctx.simTime(), the _stampAll
 * pass-level `vt` local -> ctx.junctionVt() — the same pure
 * thermalVoltage(ambient) computation, so the float is identical —
 * ds[key] = v -> ctx.setDigitalState(key, v), disp[comp.id] = info ->
 * ctx.setDisplayInfo(comp.id, info), out[comp.id] = v ->
 * ctx.setElementCurrent(comp.id, v), the paired
 * `this._dispDrivers ??= this._buildArduinoPinDrivers()` +
 * `this._mergedDisplayEvents(...)` calls -> ctx.mergedDisplayEvents(...)
 * (the lazy shared-map build now lives inside the context alias), and
 * loop break -> handler `return`). Do not simplify, reorder mna.add
 * calls, or rewrite algebra here: float accumulation order in the matrix
 * is semantics, and the bitwise oracle suite pins these exact
 * trajectories.
 *
 * Kinds grouped into one model correspond exactly to switch cases that
 * shared a body in the engine (the 11 combinational-eval kinds; the
 * 28c16/28c256 EEPROMs; seg7_cc/seg7_ca) — handlers still branch on
 * comp.kind internally where the original shared body did.
 *
 * Moved WITH this cohort (their only consumers are here now):
 * NE555_OUTPUT_R_OHM / NE555_OUTPUT_HEADROOM_V, the EEPROM write-cycle
 * time T_WC, and the SDP unlock-sequence machinery (SDP_ENABLE +
 * consumeEepromWrite). _stampCombinationalIC became the local
 * stampCombinationalIC helper.
 *
 * Engine-owned machinery deliberately NOT moved:
 * - The 555 crossing-split step machinery (_find555CrossingFrac, the
 *   pre-step ne555s snapshot, _hasNe555 gating) — it re-solves partial
 *   steps around the state.ne555s map this cohort commits into.
 * - Delayed-output event scheduling (_delayedOutputLevel /
 *   _updateDelayedOutputs and the next-due-time step limiter), reached
 *   via ctx: it schedules ENGINE step sizes, not device physics.
 * - _icPowerInfo, _stampDigitalOutput, the _logicHigh family,
 *   _stampFloatingDigitalInputs, _stampRailReferencedOutput and
 *   _readAddr4 — shared with the unmigrated MCU/driver kinds and the
 *   generic output-sag scan (_drivenOutputTargets), reached via ctx.
 * - The _drivenOutputTargets per-kind cases (generic output-sag scan,
 *   not a per-kind pass switch — same treatment as lm393 last cohort).
 * - defaultIcState, defaultHd44780State and the load()-time
 *   icState/lcds/eeproms/ne555s carry-forward + snapshot/rollback.
 * - getElementChannelI's seg7 per-segment telemetry readout.
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
  shockleyDiodeCurrent,
  shockleyIsFromVf,
  stampDiodeShockley,
  stampResistor,
} from "../elements.js";
import { AC_GMIN, shockleyConductanceAtOp } from "./semiconductors.js";
import { evalCD4511 } from "../digital.js";
import {
  defaultHd44780State,
  defaultIcState,
  N_LED,
  type EepromState,
  type IcPowerInfo,
} from "../sim-engine.js";

// 555 output stage: finite Thevenin drive strength and the compact no-load
// headroom below VCC (moved with the ne555 stamp/current handlers — the
// crossing-split machinery in the engine never reads them).
const NE555_OUTPUT_R_OHM = 20;
const NE555_OUTPUT_HEADROOM_V = 0.1;

// EEPROM write-cycle time (datasheet t_WC, 10 ms class) — a started byte
// write completes this long after the latching edge.
const T_WC = 10e-3;

// 28C256 software-data-protection unlock sequence (Atmel datasheet order).
const SDP_ENABLE: Array<[number, number]> = [
  [0x5555, 0xaa],
  [0x2aaa, 0x55],
  [0x5555, 0xa0],
];

/**
 * Run a candidate (addr, byte) write through the SDP state machine.
 * Returns true if the write should be latched; false if it was consumed
 * as part of an unlock sequence or dropped while locked.
 */
function consumeEepromWrite(st: EepromState, addr: number, data: number): boolean {
  if (!st.sdpEnabled || st.sdpUnlocked) return true;
  const [expectAddr, expectData] = SDP_ENABLE[st.sdpStep];
  if (addr === expectAddr && data === expectData) {
    st.sdpStep = (st.sdpStep + 1) as 0 | 1 | 2 | 3;
    if (st.sdpStep === 3) {
      st.sdpUnlocked = true;
      st.sdpStep = 0;
    }
    return false;
  }
  st.sdpStep = 0;
  return false;
}

/**
 * Small-signal image of ctx.stampDigitalOutput (_stampDigitalOutput): a
 * driven finite-strength stage is its Thevenin output resistance to the
 * package rail node — an AC path that follows rail sag exactly like the
 * transient resistor — or, for supply-less behavioural parts, a Norton to a
 * fixed local target whose constant differentiates away, leaving only the
 * self conductance. The sequential/driver kinds in this cohort stamp their
 * output stages from their OWN committed state keys (never `delay:` slots),
 * so they carry per-kind acStamp hooks built on this helper instead of the
 * driver's generic delay-slot default in ac-analysis.ts.
 */
function acStampDigitalOutput(
  ctx: AcDeviceContext,
  comp: DeviceComponent,
  ac: AcStampSurface,
  pinId: string,
  high: boolean,
  power: IcPowerInfo,
): void {
  const node = ctx.pinNode(comp.id, pinId);
  if (node < 0) return;
  const g = 1 / power.outputResistance;
  const railPin = high ? power.vccPin : power.gndPin;
  if (railPin && !ctx.isOpenPin(comp.id, railPin)) {
    stampAcAdmittance(ac, node, ctx.pinNode(comp.id, railPin), g, 0);
    return;
  }
  ac.addAc(node, node, g, 0);
}

/** Stamp a combinational IC (74LS00/04/08/32/86/157/245/283) using evalCombinationalIC. */
function stampCombinationalIC(
  ctx: DeviceContext,
  comp: DeviceComponent,
  xGuess: Float64Array,
  power: IcPowerInfo,
): void {
  ctx.stampFloatingDigitalInputs(comp, xGuess, power);
  const outputs = ctx.combinationalOutputs(comp, xGuess, power);
  for (const [pinId, level] of Object.entries(outputs)) {
    // level is now in ELECTRICAL convention: true = HIGH, false = LOW.
    const committed = ctx.delayedOutputLevel(comp, pinId, level);
    const isOpenCollector = ctx.catalogPart(comp)
      ?.pin_layout.find((p) => p.id === pinId)?.function === "open_collector";
    if (isOpenCollector) {
      if (!committed) {
        // Transistor sinking: committed LOW → stamp LOW to chip GND through output resistance.
        ctx.stampDigitalOutput(comp, pinId, false, power);
      }
      // committed HIGH = transistor off = output released → stamp nothing (hi-Z).
      // External pull-up determines the net voltage. Mirrors the NE555 DIS-pin pattern.
    } else {
      ctx.stampDigitalOutput(comp, pinId, committed, power);
    }
  }
}

export const ne555Model: DeviceModel = {
  kinds: ["ne555"],
  stamp: (ctx, comp, xGuess, _h) => {
    // Pins (by catalog ID): 1=GND, 2=TRIG, 3=OUT, 4=RST, 6=THR, 7=DIS, 8=VCC.
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    const pGnd = ctx.pinNode(comp.id, "1");
    const pOut  = ctx.pinNode(comp.id, "3");
    const pDis  = ctx.pinNode(comp.id, "7");
    const pVcc  = ctx.pinNode(comp.id, "8");

    // Use only the committed state from the previous time step.
    // Re-evaluating the SR comparators from xGuess would cause outHigh
    // to flip between Newton iterations near a threshold — the DIS
    // resistor would stamp/unstamp, making the matrix non-stationary
    // and preventing convergence. SR state transitions belong in
    // _updateDigitalState (post-solve only).
    const { outHigh } = ctx.state.ne555s.get(comp.id) ?? { outHigh: true };

    // OUT (pin 3): finite Thevenin drive referenced to the package's
    // physical VCC/GND rails. Complementary conductances preserve the
    // compact ~0.1 V no-load headroom while debiting sourced/sunk load
    // current from the rails instead of creating energy at a hidden
    // absolute-voltage Norton source. The small unloaded cross-current
    // is also a reasonable compact stand-in for bipolar-555 idle draw.
    const outputFraction = power.vSupply > 0
      ? outHigh
        ? 1 - Math.min(0.5, NE555_OUTPUT_HEADROOM_V / power.vSupply)
        : Math.min(0.5, NE555_OUTPUT_HEADROOM_V / power.vSupply)
      : 0;
    ctx.stampRailReferencedOutput(
      pOut,
      pGnd,
      pVcc,
      outputFraction,
      NE555_OUTPUT_R_OHM,
    );

    // DIS (pin 7): discharge transistor — low-Z to GND when output is LOW.
    if (!outHigh && pDis >= 0) {
      stampResistor(ctx.mna, pDis, pGnd, 10);
    }
  },
  updateDigital: (ctx, comp, x, _h) => {
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const pRst  = ctx.pinNode(comp.id, "4");
    const pThr  = ctx.pinNode(comp.id, "6");
    const pTrig = ctx.pinNode(comp.id, "2");
    const vRst  = ctx.vAt(x, pRst) - power.gnd;
    const vThr  = ctx.vAt(x, pThr) - power.gnd;
    const vTrig = ctx.vAt(x, pTrig) - power.gnd;
    const vUpper = (2 / 3) * power.vSupply;
    const vLower = (1 / 3) * power.vSupply;

    const prev = ctx.state.ne555s.get(comp.id) ?? { outHigh: true };
    let outHigh = prev.outHigh;
    // RST (pin 4) is the hard master clear and stays first. TRIG-set is
    // evaluated BEFORE THR-reset so that, in the one step where both the
    // set and reset comparators are simultaneously true, SET wins: the
    // physical 555 flip-flop holds OUT HIGH while TRIG is held below
    // 1/3 Vcc instead of glitching LOW when THR transiently crosses
    // 2/3 Vcc. This is what lets a monostable hold its one-shot pulse.
    // Astable is unaffected: TRIG and THR share the timing-cap node and
    // cross their thresholds on opposite half-cycles, so the two
    // comparators are never simultaneously true and this tie-break is
    // never exercised (verified: fixture-audit + full sim suite green).
    if (vRst < 0.5) outHigh = false;
    else if (vTrig < vLower) outHigh = true;
    else if (vThr  > vUpper) outHigh = false;

    ctx.state.ne555s.set(comp.id, { outHigh });
    ctx.setDigitalState(comp.id, outHigh ? 1 : 0);
  },
  updateCurrent: (ctx, comp, x) => {
    const power555 = ctx.icPowerInfo(comp, x);
    if (!power555.powered || !power555.vccPin || !power555.gndPin) {
      ctx.setElementCurrent(comp.id, 0);
      return;
    }
    const outputNode = ctx.pinNode(comp.id, "3");
    const lowNode = ctx.pinNode(comp.id, power555.gndPin);
    const highNode = ctx.pinNode(comp.id, power555.vccPin);
    const outputHigh = (ctx.state.ne555s.get(comp.id) ?? { outHigh: true }).outHigh;
    const fraction = power555.vSupply > 0
      ? outputHigh
        ? 1 - Math.min(0.5, NE555_OUTPUT_HEADROOM_V / power555.vSupply)
        : Math.min(0.5, NE555_OUTPUT_HEADROOM_V / power555.vSupply)
      : 0;
    const lowConductance = (1 - fraction) / NE555_OUTPUT_R_OHM;
    const highConductance = fraction / NE555_OUTPUT_R_OHM;
    const internalCurrentLeavingOutput =
      lowConductance * (ctx.vAt(x, outputNode) - ctx.vAt(x, lowNode))
      + highConductance * (ctx.vAt(x, outputNode) - ctx.vAt(x, highNode));
    // Positive means current delivered out of pin 3 to the external load.
    ctx.setElementCurrent(comp.id, -internalCurrentLeavingOutput);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Derivative of the transient stamp at the committed OP. The
    // rail-referenced push-pull OUT stage keeps its two complementary
    // conductances (the fraction is frozen at its OP value — its weak
    // supply dependence is a compact-model headroom artifact, not a
    // transconductance), and a committed-LOW output keeps the 10 ohm DIS
    // discharge path. The SR flip-flop is a committed latch: no
    // small-signal path exists from TRIG/THR/RST to the stage.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const pGnd = ctx.pinNode(comp.id, "1");
    const pOut = ctx.pinNode(comp.id, "3");
    const pDis = ctx.pinNode(comp.id, "7");
    const pVcc = ctx.pinNode(comp.id, "8");
    const { outHigh } = ctx.state.ne555s.get(comp.id) ?? { outHigh: true };
    const outputFraction = power.vSupply > 0
      ? outHigh
        ? 1 - Math.min(0.5, NE555_OUTPUT_HEADROOM_V / power.vSupply)
        : Math.min(0.5, NE555_OUTPUT_HEADROOM_V / power.vSupply)
      : 0;
    // Same shape and guards as _stampRailReferencedOutput.
    if (pOut >= 0) {
      const f = Math.max(0, Math.min(1, outputFraction));
      if (f > 1e-12) {
        stampAcAdmittance(ac, pOut, pVcc, f / NE555_OUTPUT_R_OHM, 0);
      }
      if (f < 1 - 1e-12) {
        stampAcAdmittance(ac, pOut, pGnd, (1 - f) / NE555_OUTPUT_R_OHM, 0);
      }
    }
    if (!outHigh && pDis >= 0) {
      stampAcAdmittance(ac, pDis, pGnd, 1 / 10, 0);
    }
  },
};

export const combinationalIcModel: DeviceModel = {
  // ── S4 combinational ICs ────────────────────────────────────────
  kinds: [
    "74ls00", "74ls04", "74ls08", "74ls32", "74ls86", "74ls157", "74ls283",
    "74ls245", "74hc14", "74hc138", "74ls47",
  ],
  stamp: (ctx, comp, xGuess, _h) => {
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    stampCombinationalIC(ctx, comp, xGuess, power);
  },
  // ── S4 combinational ICs — record output pin states ───────────
  updateDigital: (ctx, comp, x, h) => {
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    // commit = true: post-solve with converged x — safe to latch Schmitt state.
    const immediate = ctx.combinationalOutputs(comp, x, power, true);
    const visible = ctx.updateDelayedOutputs(comp, immediate, h, power);
    for (const [pinId, level] of Object.entries(visible)) {
      ctx.setDigitalState(`${comp.id}/${pinId}`, level ? 1 : 0);
    }
  },
};

// ── S4 sequential ICs (stamp committed output state) ───────────
export const ls161Model: DeviceModel = {
  kinds: ["74ls161"],
  stamp: (ctx, comp, xGuess, _h) => {
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74ls161");
    const count = st.count ?? 0;
    for (const [bit, pin] of [["qa", 0], ["qb", 1], ["qc", 2], ["qd", 3]] as [string, number][]) {
      ctx.stampDigitalOutput(comp, bit, ((count >> pin) & 1) === 1, power);
    }
    // RCO: high when count==15 and ENT is high
    const entHigh = ctx.logicHigh(comp, "ent", xGuess, power);
    const rco = count === 15 && entHigh;
    ctx.stampDigitalOutput(comp, "rco", rco, power);
  },
  // ── S4 sequential ICs — edge detection and state update ───────
  updateDigital: (ctx, comp, x, _h) => {
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74ls161");
    const clkNow  = ctx.logicHigh(comp, "clk", x, power);
    const clrLow  = !ctx.logicHigh(comp, "/clr", x, power);
    const loadLow = !ctx.logicHigh(comp, "/load", x, power);
    const enpHigh = ctx.logicHigh(comp, "enp", x, power);
    const entHigh = ctx.logicHigh(comp, "ent", x, power);
    const rising  = clkNow && !st.lastClk;
    let count = st.count ?? 0;
    if (clrLow) {
      count = 0;
    } else if (rising) {
      if (loadLow) {
        // Synchronous parallel load
        let loaded = 0;
        for (const [bit, pin] of [["a",0],["b",1],["c",2],["d",3]] as [string,number][]) {
          if (ctx.logicHigh(comp, bit, x, power)) loaded |= (1 << pin);
        }
        count = loaded;
      } else if (enpHigh && entHigh) {
        count = (count + 1) & 0xf;
      }
    }
    ctx.state.icState.set(comp.id, { count, lastClk: clkNow ? 1 : 0 });
    ctx.setDigitalState(comp.id, count);
    for (let bit = 0; bit < 4; bit++) {
      ctx.setDigitalState(`${comp.id}/q${["a","b","c","d"][bit]}`, (count >> bit) & 1);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same committed output stages the transient stamp writes: QA-QD from
    // the committed count, RCO from count==15 gated by the ENT level read
    // over the held OP (the identical logicHigh the stamp used).
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74ls161");
    const count = st.count ?? 0;
    for (const [bit, pin] of [["qa", 0], ["qb", 1], ["qc", 2], ["qd", 3]] as [string, number][]) {
      acStampDigitalOutput(ctx, comp, ac, bit, ((count >> pin) & 1) === 1, power);
    }
    const entHigh = ctx.logicHighAtOp(comp, "ent", power);
    acStampDigitalOutput(ctx, comp, ac, "rco", count === 15 && entHigh, power);
  },
};

export const ls173Model: DeviceModel = {
  kinds: ["74ls173"],
  stamp: (ctx, comp, xGuess, _h) => {
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74ls173");
    const mHigh = ctx.logicHigh(comp, "m", xGuess, power);
    const nHigh = ctx.logicHigh(comp, "n", xGuess, power);
    // Output disable: M=1 OR N=1 → Hi-Z (modelled as no stamp)
    if (!mHigh && !nHigh) {
      for (let i = 1; i <= 4; i++) {
        ctx.stampDigitalOutput(comp, `q${i}`, (st[`q${i}`] ?? 0) === 1, power);
      }
    }
  },
  updateDigital: (ctx, comp, x, _h) => {
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74ls173");
    const clkNow  = ctx.logicHigh(comp, "clk", x, power);
    const clrHigh = ctx.logicHigh(comp, "/clr", x, power);
    const g1Low   = !ctx.logicHigh(comp, "g1", x, power);
    const g2Low   = !ctx.logicHigh(comp, "g2", x, power);
    const rising  = clkNow && !st.lastClk;
    let q1 = st.q1 ?? 0, q2 = st.q2 ?? 0, q3 = st.q3 ?? 0, q4 = st.q4 ?? 0;
    if (clrHigh) {
      q1 = q2 = q3 = q4 = 0;
    } else if (rising && g1Low && g2Low) {
      q1 = ctx.logicHigh(comp, "d1", x, power) ? 1 : 0;
      q2 = ctx.logicHigh(comp, "d2", x, power) ? 1 : 0;
      q3 = ctx.logicHigh(comp, "d3", x, power) ? 1 : 0;
      q4 = ctx.logicHigh(comp, "d4", x, power) ? 1 : 0;
    }
    ctx.state.icState.set(comp.id, { q1, q2, q3, q4, lastClk: clkNow ? 1 : 0 });
    ctx.setDigitalState(`${comp.id}/q1`, q1); ctx.setDigitalState(`${comp.id}/q2`, q2);
    ctx.setDigitalState(`${comp.id}/q3`, q3); ctx.setDigitalState(`${comp.id}/q4`, q4);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Committed Q stages, gated on the same M/N output-disable reads (over
    // the held OP) the transient stamp used: disabled outputs are hi-Z.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74ls173");
    const mHigh = ctx.logicHighAtOp(comp, "m", power);
    const nHigh = ctx.logicHighAtOp(comp, "n", power);
    if (mHigh || nHigh) return;
    for (let i = 1; i <= 4; i++) {
      acStampDigitalOutput(ctx, comp, ac, `q${i}`, (st[`q${i}`] ?? 0) === 1, power);
    }
  },
};

export const ls189Model: DeviceModel = {
  kinds: ["74ls189"],
  stamp: (ctx, comp, xGuess, _h) => {
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    // Outputs are active-low. Only drive when /CS is LOW.
    const csLow = !ctx.logicHigh(comp, "/cs", xGuess, power);
    if (!csLow) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74ls189");
    const weHigh = ctx.logicHigh(comp, "/we", xGuess, power); // /WE=1 → read mode
    if (!weHigh) return; // write mode: outputs not driven
    const addr = ctx.readAddr4(comp, xGuess, power);
    const data = st[`mem${addr}`] ?? 0xf;
    for (let bit = 0; bit < 4; bit++) {
      // Outputs are inverted: /O1 = LOW when bit is 1
      const level = ((data >> bit) & 1) === 0;
      ctx.stampDigitalOutput(comp, `/o${bit + 1}`, level, power);
    }
  },
  updateDigital: (ctx, comp, x, _h) => {
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74ls189");
    const csLow = !ctx.logicHigh(comp, "/cs", x, power);
    const weHigh = ctx.logicHigh(comp, "/we", x, power);
    const weRising = weHigh && (st.lastWE ?? 1) === 0;
    if (csLow && weRising) {
      const addr = ctx.readAddr4(comp, x, power);
      let data = 0;
      for (let b = 0; b < 4; b++) {
        if (ctx.logicHigh(comp, `d${b + 1}`, x, power)) data |= (1 << b);
      }
      st[`mem${addr}`] = data;
    }
    st.lastWE = weHigh ? 1 : 0;
    ctx.state.icState.set(comp.id, { ...st });
    if (csLow) {
      const addr = ctx.readAddr4(comp, x, power);
      const data = st[`mem${addr}`] ?? 0xf;
      for (let bit = 0; bit < 4; bit++) {
        ctx.setDigitalState(`${comp.id}//o${bit + 1}`, ((data >> bit) & 1) === 0 ? 1 : 0);
      }
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same /CS and /WE gating and address decode (over the held OP) the
    // transient stamp used; a deselected or writing part drives nothing.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const csLow = !ctx.logicHighAtOp(comp, "/cs", power);
    if (!csLow) return;
    const weHigh = ctx.logicHighAtOp(comp, "/we", power);
    if (!weHigh) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74ls189");
    const addr = ctx.readAddr4AtOp(comp, power);
    const data = st[`mem${addr}`] ?? 0xf;
    for (let bit = 0; bit < 4; bit++) {
      acStampDigitalOutput(ctx, comp, ac, `/o${bit + 1}`, ((data >> bit) & 1) === 0, power);
    }
  },
};

export const hc595Model: DeviceModel = {
  kinds: ["74hc595"],
  stamp: (ctx, comp, xGuess, _h) => {
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74hc595");
    const latch = st.latch ?? 0;
    const oeHigh = ctx.logicHigh(comp, "/oe", xGuess, power);
    if (oeHigh) return; // outputs disabled
    for (const [pin, bit] of [["qa",0],["qb",1],["qc",2],["qd",3],["qe",4],["qf",5],["qg",6],["qh",7]] as [string, number][]) {
      ctx.stampDigitalOutput(comp, pin, ((latch >> bit) & 1) === 1, power);
    }
  },
  updateDigital: (ctx, comp, x, _h) => {
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74hc595");
    const srclkNow = ctx.logicHigh(comp, "srclk", x, power);
    const rclkNow  = ctx.logicHigh(comp, "rclk", x, power);
    const clrLow   = !ctx.logicHigh(comp, "/srclr", x, power);
    const rising595 = srclkNow && !(st.lastSRCLK ?? 0);
    const latchRise = rclkNow  && !(st.lastRCLK  ?? 0);
    let shift = st.shift ?? 0;
    let latch = st.latch ?? 0;
    if (clrLow) { shift = 0; }
    else if (rising595) {
      const serBit = ctx.logicHigh(comp, "ser", x, power) ? 1 : 0;
      shift = ((shift << 1) | serBit) & 0xff;
    }
    if (latchRise) latch = shift;
    ctx.state.icState.set(comp.id, { shift, latch, lastSRCLK: srclkNow ? 1 : 0, lastRCLK: rclkNow ? 1 : 0 });
    for (let b = 0; b < 8; b++) {
      const pinName = ["qa","qb","qc","qd","qe","qf","qg","qh"][b];
      ctx.setDigitalState(`${comp.id}/${pinName}`, (latch >> b) & 1);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Committed latch stages behind the same /OE tristate gate (over the
    // held OP) the transient stamp used.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    if (ctx.logicHighAtOp(comp, "/oe", power)) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74hc595");
    const latch = st.latch ?? 0;
    for (const [pin, bit] of [["qa",0],["qb",1],["qc",2],["qd",3],["qe",4],["qf",5],["qg",6],["qh",7]] as [string, number][]) {
      acStampDigitalOutput(ctx, comp, ac, pin, ((latch >> bit) & 1) === 1, power);
    }
  },
};

export const hc165Model: DeviceModel = {
  kinds: ["74hc165"],
  stamp: (ctx, comp, xGuess, _h) => {
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74hc165");
    const shift = st.shift ?? 0;
    const q7bit = (shift >> 7) & 1;
    ctx.stampDigitalOutput(comp, "q7", q7bit === 1, power);
    ctx.stampDigitalOutput(comp, "/q7", q7bit === 0, power);
  },
  updateDigital: (ctx, comp, x, _h) => {
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74hc165");
    const plLow    = !ctx.logicHigh(comp, "/pl", x, power);
    const cpNow    = ctx.logicHigh(comp, "cp", x, power);
    const cpInhHigh = ctx.logicHigh(comp, "cp_inh", x, power);
    const clockEnable = !cpInhHigh;
    const rising165 = cpNow && !(st.lastClk ?? 0) && clockEnable;
    let shift = st.shift ?? 0;
    if (plLow) {
      // Asynchronous parallel load
      shift = 0;
      for (let i = 0; i < 8; i++) {
        const pin = i < 4 ? `d${i}` : `d${i}`;
        if (ctx.logicHigh(comp, pin, x, power)) shift |= (1 << i);
      }
    } else if (rising165) {
      const dsBit = ctx.logicHigh(comp, "ds", x, power) ? 1 : 0;
      shift = (((shift << 1) | dsBit) & 0xff);
    }
    ctx.state.icState.set(comp.id, { shift, lastClk: cpNow ? 1 : 0, lastLoad: plLow ? 0 : 1 });
    ctx.setDigitalState(`${comp.id}/q7`,  (shift >> 7) & 1);
    ctx.setDigitalState(`${comp.id}//q7`, ((shift >> 7) & 1) ^ 1);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Complementary committed Q7 stages, exactly the transient pair.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74hc165");
    const q7bit = ((st.shift ?? 0) >> 7) & 1;
    acStampDigitalOutput(ctx, comp, ac, "q7", q7bit === 1, power);
    acStampDigitalOutput(ctx, comp, ac, "/q7", q7bit === 0, power);
  },
};

// ── W2.2 sequential ICs ──────────────────────────────────────────
export const hc74Model: DeviceModel = {
  kinds: ["74hc74"],
  stamp: (ctx, comp, xGuess, _h) => {
    // Dual D flip-flop — stamp COMMITTED Q outputs from icState.
    // Async PRE_n / CLR_n can override from the stamp path when asserted
    // (to avoid lagging by one step). Since PRE_n/CLR_n are asynchronous,
    // we re-evaluate them here from committed input levels.
    //
    // FIX 6: store q1n/q2n explicitly in icState. In the both-asserted case
    // (PRE_n LOW AND CLR_n LOW), the datasheet says Q and Q_n BOTH go HIGH
    // (logically inconsistent, documented as "not allowed" in SN74HC74).
    // We stamp the stored qn value rather than computing !q, so that path
    // is correctly represented.
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74hc74");
    let q1 = (st.q1 ?? 0) >= 0.5;
    let q1n = (st.q1n ?? 1) >= 0.5;
    let q2 = (st.q2 ?? 0) >= 0.5;
    let q2n = (st.q2n ?? 1) >= 0.5;
    // Immediate async override: if /PRE or /CLR currently asserted, reflect now
    const pre1LowNow = !ctx.logicHighH(comp, "pre1_n", xGuess, power);
    const clr1LowNow = !ctx.logicHighH(comp, "clr1_n", xGuess, power);
    if (clr1LowNow && pre1LowNow) { q1 = true; q1n = true; }  // both asserted: Q and Q_n HIGH
    else if (clr1LowNow) { q1 = false; q1n = true; }
    else if (pre1LowNow) { q1 = true;  q1n = false; }
    // else: hold stored values
    const pre2LowNow = !ctx.logicHighH(comp, "pre2_n", xGuess, power);
    const clr2LowNow = !ctx.logicHighH(comp, "clr2_n", xGuess, power);
    if (clr2LowNow && pre2LowNow) { q2 = true; q2n = true; }  // both asserted: Q and Q_n HIGH
    else if (clr2LowNow) { q2 = false; q2n = true; }
    else if (pre2LowNow) { q2 = true;  q2n = false; }
    // else: hold stored values
    ctx.stampDigitalOutput(comp, "q1",   q1,  power);
    ctx.stampDigitalOutput(comp, "q1_n", q1n, power);
    ctx.stampDigitalOutput(comp, "q2",   q2,  power);
    ctx.stampDigitalOutput(comp, "q2_n", q2n, power);
  },
  // ── W2.2 sequential ICs — edge detection and state update ─────────
  updateDigital: (ctx, comp, x, _h) => {
    // Dual D flip-flop with async PRE_n / CLR_n.
    // Source: TI SN74HC74 datasheet, SCLS108 (Texas Instruments).
    // Priority per datasheet (both FF units identical):
    //   1. CLR_n LOW: Q=0, Q_n=1 (async)
    //   2. PRE_n LOW: Q=1, Q_n=0 (async)
    //   3. PRE_n LOW + CLR_n LOW simultaneously: Q=1, Q_n=1 (unstable;
    //      when released one output will dominate — documented as "not allowed")
    //   4. Rising CLK edge: Q captures D
    //   5. Otherwise: Q holds
    //
    // FIX 6: q1n/q2n are stored explicitly in icState rather than computed as !q.
    // In the both-asserted case Q and Q_n are both HIGH per datasheet.
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74hc74");

    let q1  = (st.q1  ?? 0) >= 0.5;
    let q1n = (st.q1n ?? 1) >= 0.5;
    let q2  = (st.q2  ?? 0) >= 0.5;
    let q2n = (st.q2n ?? 1) >= 0.5;

    // --- FF1 ---
    // Use _logicHighH for /PRE and /CLR: IC_OPEN_HIGH_PINS makes open
    // pins deterministically HIGH (inactive) rather than using the seeded model.
    const clr1Low = !ctx.logicHighH(comp, "clr1_n", x, power);
    const pre1Low = !ctx.logicHighH(comp, "pre1_n", x, power);
    const clk1Now = ctx.logicHigh(comp, "clk1", x, power);
    const rising1 = clk1Now && !(st.lastClk1 ?? 0);
    if (clr1Low && pre1Low) {
      // Both asserted simultaneously: Q and Q_n both HIGH (unstable per datasheet)
      q1 = true; q1n = true;
    } else if (clr1Low) {
      q1 = false; q1n = true;
    } else if (pre1Low) {
      q1 = true; q1n = false;
    } else if (rising1) {
      q1 = ctx.logicHigh(comp, "d1", x, power);
      q1n = !q1;
    }
    // else: hold — q1/q1n unchanged

    // --- FF2 ---
    const clr2Low = !ctx.logicHighH(comp, "clr2_n", x, power);
    const pre2Low = !ctx.logicHighH(comp, "pre2_n", x, power);
    const clk2Now = ctx.logicHigh(comp, "clk2", x, power);
    const rising2 = clk2Now && !(st.lastClk2 ?? 0);
    if (clr2Low && pre2Low) {
      q2 = true; q2n = true;
    } else if (clr2Low) {
      q2 = false; q2n = true;
    } else if (pre2Low) {
      q2 = true; q2n = false;
    } else if (rising2) {
      q2 = ctx.logicHigh(comp, "d2", x, power);
      q2n = !q2;
    }
    // else: hold — q2/q2n unchanged

    ctx.state.icState.set(comp.id, {
      q1:  q1  ? 1 : 0,
      q1n: q1n ? 1 : 0,
      lastClk1: clk1Now ? 1 : 0,
      q2:  q2  ? 1 : 0,
      q2n: q2n ? 1 : 0,
      lastClk2: clk2Now ? 1 : 0,
    });
    ctx.setDigitalState(`${comp.id}/q1`,   q1  ? 1 : 0);
    ctx.setDigitalState(`${comp.id}/q1_n`, q1n ? 1 : 0);
    ctx.setDigitalState(`${comp.id}/q2`,   q2  ? 1 : 0);
    ctx.setDigitalState(`${comp.id}/q2_n`, q2n ? 1 : 0);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Committed Q/Q_n stages with the same immediate async PRE/CLR override
    // (over the held OP, IC_OPEN_HIGH_PINS aware) the transient stamp
    // applies before stamping.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("74hc74");
    let q1 = (st.q1 ?? 0) >= 0.5;
    let q1n = (st.q1n ?? 1) >= 0.5;
    let q2 = (st.q2 ?? 0) >= 0.5;
    let q2n = (st.q2n ?? 1) >= 0.5;
    const pre1LowNow = !ctx.logicHighHAtOp(comp, "pre1_n", power);
    const clr1LowNow = !ctx.logicHighHAtOp(comp, "clr1_n", power);
    if (clr1LowNow && pre1LowNow) { q1 = true; q1n = true; }
    else if (clr1LowNow) { q1 = false; q1n = true; }
    else if (pre1LowNow) { q1 = true; q1n = false; }
    const pre2LowNow = !ctx.logicHighHAtOp(comp, "pre2_n", power);
    const clr2LowNow = !ctx.logicHighHAtOp(comp, "clr2_n", power);
    if (clr2LowNow && pre2LowNow) { q2 = true; q2n = true; }
    else if (clr2LowNow) { q2 = false; q2n = true; }
    else if (pre2LowNow) { q2 = true; q2n = false; }
    acStampDigitalOutput(ctx, comp, ac, "q1", q1, power);
    acStampDigitalOutput(ctx, comp, ac, "q1_n", q1n, power);
    acStampDigitalOutput(ctx, comp, ac, "q2", q2, power);
    acStampDigitalOutput(ctx, comp, ac, "q2_n", q2n, power);
  },
};

export const cd4017Model: DeviceModel = {
  kinds: ["cd4017"],
  stamp: (ctx, comp, xGuess, _h) => {
    // Decade counter — stamp COMMITTED one-hot outputs from icState.
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("cd4017");
    const count = st.count ?? 0;
    // One-hot: only the active count output is HIGH
    for (let i = 0; i <= 9; i++) {
      ctx.stampDigitalOutput(comp, `q${i}`, count === i, power);
    }
    // CO: HIGH for counts 0-4, LOW for counts 5-9 (divide-by-10 square wave)
    ctx.stampDigitalOutput(comp, "co", count < 5, power);
  },
  updateDigital: (ctx, comp, x, _h) => {
    // Decade counter.
    // Source: TI CD4017B datasheet, SCHS027 (Texas Instruments).
    // RESET HIGH: asynchronous clear to count 0.
    // CLK rising edge with CLKINH LOW: advance count.
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("cd4017");

    const resetHigh = ctx.logicHigh(comp, "reset", x, power);
    const clkinhHigh = ctx.logicHigh(comp, "clkinh", x, power);
    const clkNow = ctx.logicHigh(comp, "clk", x, power);
    const rising4017 = clkNow && !(st.lastClk ?? 0) && !clkinhHigh;

    let count = st.count ?? 0;
    if (resetHigh) {
      count = 0;
    } else if (rising4017) {
      count = (count + 1) % 10;
    }

    ctx.state.icState.set(comp.id, { count, lastClk: clkNow ? 1 : 0 });
    ctx.setDigitalState(`${comp.id}`, count);
    for (let i = 0; i <= 9; i++) {
      ctx.setDigitalState(`${comp.id}/q${i}`, count === i ? 1 : 0);
    }
    ctx.setDigitalState(`${comp.id}/co`, count < 5 ? 1 : 0);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Committed one-hot decade outputs plus CO, identical to the transient
    // stamp's committed-state table.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("cd4017");
    const count = st.count ?? 0;
    for (let i = 0; i <= 9; i++) {
      acStampDigitalOutput(ctx, comp, ac, `q${i}`, count === i, power);
    }
    acStampDigitalOutput(ctx, comp, ac, "co", count < 5, power);
  },
};

export const cd4511Model: DeviceModel = {
  kinds: ["cd4511"],
  stamp: (ctx, comp, xGuess, _h) => {
    // BCD latch/decoder — stamp COMMITTED segment outputs from latched state.
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("cd4511");
    const latchedBcd = st.latchedBcd ?? 0;
    // Use _logicHighH for /LT and /BL so open pins resolve HIGH (inactive)
    // via IC_OPEN_HIGH_PINS rather than the seeded floating model.
    const ltLow = !ctx.logicHighH(comp, "lt_n", xGuess, power);
    const blLow = !ctx.logicHighH(comp, "bl_n", xGuess, power);
    const segs = evalCD4511(latchedBcd, ltLow, blLow);
    for (const [pinId, high] of Object.entries(segs)) {
      ctx.stampDigitalOutput(comp, pinId, high, power);
    }
  },
  updateDigital: (ctx, comp, x, _h) => {
    // BCD latch/decoder/driver.
    // Source: TI CD4511B datasheet, SCHS021 (Texas Instruments).
    // LE LOW: transparent (outputs follow BCD inputs).
    // LE HIGH: latch — outputs hold value present at LE rising edge.
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("cd4511");

    const leNow = ctx.logicHigh(comp, "le", x, power);
    const leRising = leNow && !(st.lastLE ?? 0);

    // Read BCD inputs from the CURRENT converged voltages
    const bcdNow =
      (ctx.logicHigh(comp, "a", x, power) ? 1 : 0) |
      (ctx.logicHigh(comp, "b", x, power) ? 2 : 0) |
      (ctx.logicHigh(comp, "c", x, power) ? 4 : 0) |
      (ctx.logicHigh(comp, "d", x, power) ? 8 : 0);

    let latchedBcd = st.latchedBcd ?? 0;
    if (!leNow) {
      // LE LOW: transparent — latch tracks BCD continuously
      latchedBcd = bcdNow;
    } else if (leRising) {
      // LE rising edge: capture current BCD into latch
      latchedBcd = bcdNow;
    }
    // LE HIGH steady: hold latchedBcd (no update)

    const ltLow = !ctx.logicHighH(comp, "lt_n", x, power);
    const blLow = !ctx.logicHighH(comp, "bl_n", x, power);
    const segs = evalCD4511(latchedBcd, ltLow, blLow);

    ctx.state.icState.set(comp.id, { latchedBcd, lastLE: leNow ? 1 : 0 });
    for (const [pinId, high] of Object.entries(segs)) {
      ctx.setDigitalState(`${comp.id}/${pinId}`, high ? 1 : 0);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same latched-BCD decode and /LT and /BL reads (over the held OP,
    // IC_OPEN_HIGH_PINS aware) the transient stamp used for its segment
    // output stages.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("cd4511");
    const ltLow = !ctx.logicHighHAtOp(comp, "lt_n", power);
    const blLow = !ctx.logicHighHAtOp(comp, "bl_n", power);
    const segs = evalCD4511(st.latchedBcd ?? 0, ltLow, blLow);
    for (const [pinId, high] of Object.entries(segs)) {
      acStampDigitalOutput(ctx, comp, ac, pinId, high, power);
    }
  },
};

export const cd4060Model: DeviceModel = {
  kinds: ["cd4060"],
  stamp: (ctx, comp, xGuess, _h) => {
    // 14-stage ripple counter + oscillator.
    // Stamp committed counter outputs and inverter stage outputs.
    //
    // Oscillator topology (Nexperia HEF4060B Rev. 11, Section 11 RC oscillator):
    //   Inverter 1: input = CLK/RS (pin 11), output drives RTC (pin 10) = NOT(RS).
    //   Inverter 2: input = inv1Out, output drives CTC (pin 9) = in-phase with RS.
    //   External network: Rt from RTC (pin 10) to CLK/RS (pin 11),
    //                     Ct from CTC (pin 9) to CLK/RS (pin 11).
    //   (R2 series protection resistor not needed in-sim — input clamp diodes
    //    are not modelled.)
    //
    // Pin map (Nexperia Table 2, converted to TI naming):
    //   Pin 13 = Q9, Pin 14 = Q8, Pin 15 = Q10.
    //   Internal stages with no pins: Q1-Q3, Q11.
    //
    // FIX 5: always stamp inv1Out/inv2Out when powered — never float ctc/rtc.
    // The update case forces both LOW during reset, so the pins sit driven-LOW
    // and oscillation restarts cleanly on reset release.
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("cd4060");
    const count = st.count ?? 0;
    const resetHigh = ctx.logicHigh(comp, "reset", xGuess, power);
    const effectiveCount = resetHigh ? 0 : count;
    // Stamp committed Q outputs — pinned stages only (Q1-Q3 and Q11 are INTERNAL only)
    for (const [pinId, stage] of [
      ["q4", 4], ["q5", 5], ["q6", 6], ["q7", 7],
      ["q8", 8], ["q9", 9], ["q10", 10],
      ["q12", 12], ["q13", 13], ["q14", 14],
    ] as [string, number][]) {
      ctx.stampDigitalOutput(comp, pinId, ((effectiveCount >> (stage - 1)) & 1) === 1, power);
    }
    // Oscillator inverter stages: stamp from committed state.
    // inv1Out drives rtc (pin 10) = NOT(RS); inv2Out drives ctc (pin 9) = in-phase with RS.
    // Always stamp — both are LOW during reset (update case ensures this),
    // so ctc/rtc sit driven-LOW and oscillation restarts cleanly on release.
    const inv1Out = (st.inv1Out ?? 0) >= 0.5;
    const inv2Out = (st.inv2Out ?? 0) >= 0.5;
    ctx.stampDigitalOutput(comp, "rtc", inv1Out, power);
    ctx.stampDigitalOutput(comp, "ctc", inv2Out, power);
  },
  updateDigital: (ctx, comp, x, _h) => {
    // 14-stage ripple counter with oscillator.
    // Source: TI CD4060B datasheet, SCHS007 (Texas Instruments).
    // Pin map cross-referenced to Nexperia HEF4060B Rev. 11, Table 2.
    //
    // OSCILLATOR TOPOLOGY (Nexperia HEF4060B Rev. 11, Section 11 RC oscillator):
    //   Inverter 1: input = CLK/RS (pin 11), output drives RTC (pin 10) = NOT(RS).
    //   Inverter 2: input = inv1Out, output drives CTC (pin 9) = in-phase with RS.
    //   External network: Rt from RTC (pin 10) to CLK/RS (pin 11);
    //                     Ct from CTC (pin 9) to CLK/RS (pin 11).
    //   The RESISTOR pin (RTC) carries the ANTIPHASE signal so the junction relaxes
    //   toward the flip threshold; the CAP pin (CTC) carries IN-PHASE so the cap
    //   kicks the junction past the threshold on each flip.
    //   (R2 series protection resistor not modelled — input clamp diodes absent.)
    //
    // COUNTER: advances on FALLING edge of CLK/RS (pin 11) itself.
    //   Nexperia function table: RS rising → no change; RS falling → count.
    //   lastRs tracks the committed RS level; count when lastRs HIGH and rsNow LOW.
    //
    // RESET HIGH: clears counter asynchronously; both inv outputs forced LOW.
    //
    // PIN MAP (internal stages with NO pins: Q1-Q3, Q11):
    //   Pin 13 = Q9, Pin 14 = Q8, Pin 15 = Q10.
    //
    // HYSTERESIS NOTE: The real chip's oscillator works without a Schmitt input
    //   because the cap regeneration snaps the junction through the indeterminate
    //   band. Our solver resolves the 0.3-0.7*VDD band with seeded noise, so the
    //   slowly ramping junction would chatter. We use the W2.1 Schmitt seam
    //   (v_t_plus/v_t_minus in electrical_specs.v_input) as a solver-honesty
    //   substitute for the real chip's regenerative snap.
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("cd4060");

    const resetHigh = ctx.logicHigh(comp, "reset", x, power);

    // Read CLK/RS (pin 11) with Schmitt hysteresis (commit=true: post-solve
    // converged x). _logicPinMap auto-Schmitt does NOT apply to cd4060 (it is
    // not a combinational-eval part), hence the explicit call here.
    const rsNow = ctx.schmittLogicHigh(comp, "clk_in", x, power, true);

    // Inverter stages.
    // inv1Out = NOT(RS) → drives RTC (pin 10).
    // inv2Out = NOT(inv1Out) = in-phase with RS → drives CTC (pin 9).
    let inv1Out: boolean;
    let inv2Out: boolean;

    if (!resetHigh) {
      inv1Out = !rsNow;
      inv2Out = !inv1Out;
    } else {
      // RESET forces both inverter outputs LOW; pins sit driven-LOW so
      // oscillation restarts cleanly on release (FIX 5).
      inv1Out = false;
      inv2Out = false;
    }

    // Counter advance: falling edge of CLK/RS (pin 11) itself.
    // Nexperia function table: RS rising → no change; RS falling → count.
    const lastRs = (st.lastClk ?? 0) >= 0.5; // lastClk stores the RS level
    const fallingRs = lastRs && !rsNow;

    let count = st.count ?? 0;
    if (resetHigh) {
      count = 0;
    } else if (fallingRs) {
      count = (count + 1) & 0x3fff; // 14-bit counter
    }

    // Merge into the LIVE map entry rather than replacing it: the Schmitt
    // read above committed "schmitt_clk_in" into the map (possibly into a
    // different object than `st` on the first step), and a replacement
    // write would clobber it — destroying RS hysteresis every step.
    const live4060 = ctx.state.icState.get(comp.id) ?? {};
    ctx.state.icState.set(comp.id, {
      ...live4060,
      count,
      lastClk: rsNow ? 1 : 0,  // store RS level (not inv1Out)
      inv1Out: inv1Out ? 1 : 0,
      inv2Out: inv2Out ? 1 : 0,
    });

    ctx.setDigitalState(`${comp.id}`, count);
    // Publish pinned stages only (Q1-Q3 and Q11 are internal — no pins)
    for (const [pinId, stage] of [
      ["q4", 4], ["q5", 5], ["q6", 6], ["q7", 7],
      ["q8", 8], ["q9", 9], ["q10", 10],
      ["q12", 12], ["q13", 13], ["q14", 14],
    ] as [string, number][]) {
      ctx.setDigitalState(`${comp.id}/${pinId}`, ((count >> (stage - 1)) & 1));
    }
    ctx.setDigitalState(`${comp.id}/rtc`, inv1Out ? 1 : 0);
    ctx.setDigitalState(`${comp.id}/ctc`, inv2Out ? 1 : 0);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Committed counter stages (reset gate re-read over the held OP, as the
    // transient stamp does) plus the two committed oscillator inverter
    // stages that are always driven while powered.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("cd4060");
    const resetHigh = ctx.logicHighAtOp(comp, "reset", power);
    const effectiveCount = resetHigh ? 0 : (st.count ?? 0);
    for (const [pinId, stage] of [
      ["q4", 4], ["q5", 5], ["q6", 6], ["q7", 7],
      ["q8", 8], ["q9", 9], ["q10", 10],
      ["q12", 12], ["q13", 13], ["q14", 14],
    ] as [string, number][]) {
      acStampDigitalOutput(
        ctx, comp, ac, pinId,
        ((effectiveCount >> (stage - 1)) & 1) === 1,
        power,
      );
    }
    acStampDigitalOutput(ctx, comp, ac, "rtc", (st.inv1Out ?? 0) >= 0.5, power);
    acStampDigitalOutput(ctx, comp, ac, "ctc", (st.inv2Out ?? 0) >= 0.5, power);
  },
};

export const eepromModel: DeviceModel = {
  kinds: ["28c16", "28c256"],
  stamp: (ctx, comp, xGuess, _h) => {
    const power = ctx.icPowerInfo(comp, xGuess);
    if (!power.powered) return;
    ctx.stampFloatingDigitalInputs(comp, xGuess, power);
    const st = ctx.state.eeproms.get(comp.id);
    if (!st) return;
    const oeLow = !ctx.logicHigh(comp, "/oe", xGuess, power);
    const ceLow = !ctx.logicHigh(comp, "/ce", xGuess, power);
    if (!oeLow || !ceLow) return; // outputs tristated

    const addrBits = comp.kind === "28c16" ? 11 : 15;
    let addr = 0;
    for (let i = 0; i < addrBits; i++) {
      if (ctx.logicHigh(comp, `a${i}`, xGuess, power)) addr |= (1 << i);
    }

    let byte: number;
    let pollMask = 0;
    let pollValue = 0;
    const addrMask = (1 << addrBits) - 1;
    const pollingAddr = st.pendingAddr !== null && (addr & addrMask) === st.pendingAddr;
    if (st.writeCompletesAt !== null && ctx.simTime() < st.writeCompletesAt && st.pendingByte !== null && pollingAddr) {
      byte = st.bytes[addr] ?? 0xff;
      const trueBit7 = (st.pendingByte >> 7) & 1;
      const toggleBit = st.pollReads & 1;
      pollMask = (1 << 7) | (1 << 6);
      pollValue = ((trueBit7 ^ 1) << 7) | (toggleBit << 6);
    } else {
      byte = st.bytes[addr] ?? 0xff;
    }

    for (let bit = 0; bit < 8; bit++) {
      const useOverride = (pollMask >> bit) & 1;
      const level = useOverride
        ? ((pollValue >> bit) & 1) === 1
        : ((byte >> bit) & 1) === 1;
      ctx.stampDigitalOutput(comp, `io${bit}`, level, power);
    }
  },
  updateDigital: (ctx, comp, x, _h) => {
    const power = ctx.icPowerInfo(comp, x);
    if (!power.powered) return;
    const st = ctx.state.eeproms.get(comp.id);
    if (!st) return;

    const oeLow = !ctx.logicHigh(comp, "/oe", x, power);
    const ceLow = !ctx.logicHigh(comp, "/ce", x, power);
    const weHigh = ctx.logicHigh(comp, "/we", x, power);
    const ceHigh = !ceLow;
    const addrBits = comp.kind === "28c16" ? 11 : 15;
    const addrMask = (1 << addrBits) - 1;

    if (st.writeCompletesAt !== null && ctx.simTime() >= st.writeCompletesAt) {
      if (st.pendingAddr !== null && st.pendingByte !== null) {
        st.bytes[st.pendingAddr & addrMask] = st.pendingByte & 0xff;
        st.dirty = true;
      }
      st.writeCompletesAt = null;
      st.pendingAddr = null;
      st.pendingByte = null;
      st.pollReads = 0;
    }

    const weRising = weHigh && !st.lastWeHigh;
    const ceRising = ceHigh && !st.lastCeHigh;
    const oeHigh = !oeLow;
    const weLatch = weRising && ceLow && oeHigh;
    const ceLatch = ceRising && !weHigh && oeHigh;

    if ((weLatch || ceLatch) && st.writeCompletesAt === null) {
      let addr = 0;
      for (let i = 0; i < addrBits; i++) {
        if (ctx.logicHigh(comp, `a${i}`, x, power)) addr |= (1 << i);
      }
      let data = 0;
      for (let b = 0; b < 8; b++) {
        if (ctx.logicHigh(comp, `io${b}`, x, power)) data |= (1 << b);
      }
      const accepted = consumeEepromWrite(st, addr, data & 0xff);
      if (accepted) {
        st.pendingAddr = addr & addrMask;
        st.pendingByte = data & 0xff;
        st.writeCompletesAt = ctx.simTime() + T_WC;
        st.pollReads = 0;
      }
    }

    if (st.writeCompletesAt !== null) st.pollReads = (st.pollReads + 1) & 0xffff;

    st.lastWeHigh = weHigh;
    st.lastCeHigh = ceHigh;

    if (!oeLow || !ceLow || st.writeCompletesAt !== null) return;

    let addr = 0;
    for (let i = 0; i < addrBits; i++) {
      if (ctx.logicHigh(comp, `a${i}`, x, power)) addr |= (1 << i);
    }
    const byte = st.bytes[addr] ?? 0xff;
    for (let bit = 0; bit < 8; bit++) {
      ctx.setDigitalState(`${comp.id}/io${bit}`, (byte >> bit) & 1);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Same /OE and /CE tristate gates, address decode, and in-flight
    // write-cycle polling override (bits 7/6) the transient stamp computed
    // from the held OP and frozen simTime.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const st = ctx.state.eeproms.get(comp.id);
    if (!st) return;
    const oeLow = !ctx.logicHighAtOp(comp, "/oe", power);
    const ceLow = !ctx.logicHighAtOp(comp, "/ce", power);
    if (!oeLow || !ceLow) return;

    const addrBits = comp.kind === "28c16" ? 11 : 15;
    let addr = 0;
    for (let i = 0; i < addrBits; i++) {
      if (ctx.logicHighAtOp(comp, `a${i}`, power)) addr |= (1 << i);
    }

    let pollMask = 0;
    let pollValue = 0;
    const addrMask = (1 << addrBits) - 1;
    const pollingAddr = st.pendingAddr !== null && (addr & addrMask) === st.pendingAddr;
    if (st.writeCompletesAt !== null && ctx.simTime() < st.writeCompletesAt && st.pendingByte !== null && pollingAddr) {
      const trueBit7 = (st.pendingByte >> 7) & 1;
      const toggleBit = st.pollReads & 1;
      pollMask = (1 << 7) | (1 << 6);
      pollValue = ((trueBit7 ^ 1) << 7) | (toggleBit << 6);
    }
    const byte = st.bytes[addr] ?? 0xff;

    for (let bit = 0; bit < 8; bit++) {
      const useOverride = (pollMask >> bit) & 1;
      const level = useOverride
        ? ((pollValue >> bit) & 1) === 1
        : ((byte >> bit) & 1) === 1;
      acStampDigitalOutput(ctx, comp, ac, `io${bit}`, level, power);
    }
  },
};

export const seg7Model: DeviceModel = {
  kinds: ["seg7_cc", "seg7_ca"],
  stamp: (ctx, comp, xGuess, _h) => {
    // Each segment is a real LED diode. Honest current draw matters now
    // that 74LS47 (open-collector sink) and CD4511 (push-pull source) drive
    // these displays. Without the stamp the IC output nodes see no load and
    // their voltages are determined only by internal Norton sources.
    //
    // seg7_cc: anode = segment pin, cathode = common (common cathode to GND)
    // seg7_ca: anode = common (common anode to VCC), cathode = segment pin
    const vt = ctx.junctionVt();
    const SEG7_PINS = ["a", "b", "c", "d", "e", "f", "g", "dp"] as const;
    const seg7CommonAnode = comp.kind === "seg7_ca";
    const seg7Specs = ctx.electricalSpecs(comp);
    const seg7Vf = ctx.junctionVf(Number(comp.params.vf ?? seg7Specs?.vf ?? 2.0));
    const seg7iRated = Number(comp.params.iRated ?? seg7Specs?.if_max ?? 0.02);
    const VtN_seg7 = N_LED * vt;
    const seg7Is = shockleyIsFromVf(seg7Vf, seg7iRated, N_LED, vt);
    // Raise the exp-clamp to include Vf + margin, same as the led case.
    const seg7vSat = Math.min(Math.max(40 * VtN_seg7, seg7Vf + 5 * VtN_seg7), 80 * VtN_seg7);
    // buildNets bonds COM and COM2 to one package node. Prefer the canonical
    // COM label; the COM2 fallback supports malformed/legacy pin lists.
    let comN: number;
    if (!ctx.isOpenPin(comp.id, "com")) {
      comN = ctx.pinNode(comp.id, "com");
    } else if (!ctx.isOpenPin(comp.id, "com2")) {
      comN = ctx.pinNode(comp.id, "com2");
    } else {
      return; // no common pin wired — nothing to stamp
    }
    for (const seg of SEG7_PINS) {
      if (ctx.isOpenPin(comp.id, seg)) continue; // unconnected segment — leave open
      const segN = ctx.pinNode(comp.id, seg);
      const [ai, ci] = seg7CommonAnode ? [comN, segN] : [segN, comN];
      const vd = ctx.vAt(xGuess, ai) - ctx.vAt(xGuess, ci);
      stampDiodeShockley(ctx.mna, ai, ci, vd, seg7Is, N_LED, vt, seg7vSat);
    }
  },
  updateCurrent: (ctx, comp, x) => {
    // Report total current through all lit segments combined. The per-segment
    // breakdown is available from getElementChannelI() for the renderer.
    const vt = ctx.junctionVt();
    const SEG7_PINS_I = ["a", "b", "c", "d", "e", "f", "g", "dp"] as const;
    const seg7CommonAnodeI = comp.kind === "seg7_ca";
    const seg7SpecsI = ctx.electricalSpecs(comp);
    const seg7VfI = ctx.junctionVf(Number(comp.params.vf ?? seg7SpecsI?.vf ?? 2.0));
    const seg7iRatedI = Number(comp.params.iRated ?? seg7SpecsI?.if_max ?? 0.02);
    const VtNI = N_LED * vt;
    const seg7IsI = shockleyIsFromVf(seg7VfI, seg7iRatedI, N_LED, vt);
    const seg7vSatI = Math.min(Math.max(40 * VtNI, seg7VfI + 5 * VtNI), 80 * VtNI);
    // buildNets bonds COM and COM2 to one package node. Prefer the canonical
    // COM label; the COM2 fallback supports malformed/legacy pin lists.
    let comNI: number;
    if (!ctx.isOpenPin(comp.id, "com")) {
      comNI = ctx.pinNode(comp.id, "com");
    } else if (!ctx.isOpenPin(comp.id, "com2")) {
      comNI = ctx.pinNode(comp.id, "com2");
    } else {
      comNI = -2; // sentinel: neither COM pin is wired
    }
    let totalI = 0;
    if (comNI !== -2) {
      for (const seg of SEG7_PINS_I) {
        if (ctx.isOpenPin(comp.id, seg)) continue;
        const segNI = ctx.pinNode(comp.id, seg);
        const [aiI, ciI] = seg7CommonAnodeI ? [comNI, segNI] : [segNI, comNI];
        const vdI = ctx.vAt(x, aiI) - ctx.vAt(x, ciI);
        totalI += Math.max(
          0,
          shockleyDiodeCurrent(vdI, seg7IsI, N_LED, vt, seg7vSatI),
        );
      }
    }
    ctx.setElementCurrent(comp.id, totalI);
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // Per-segment guarded Shockley slope at the OP junction voltage with
    // the identical parameter derivation the transient stamp uses. This is
    // the real load a lit segment presents to its driver (a 74LS47 sink or
    // CD4511 source stage) — leaving it out would report the driver node
    // unloaded.
    const vt = ctx.junctionVt();
    const SEG7_PINS = ["a", "b", "c", "d", "e", "f", "g", "dp"] as const;
    const seg7CommonAnode = comp.kind === "seg7_ca";
    const seg7Specs = ctx.electricalSpecs(comp);
    const seg7Vf = ctx.junctionVf(Number(comp.params.vf ?? seg7Specs?.vf ?? 2.0));
    const seg7iRated = Number(comp.params.iRated ?? seg7Specs?.if_max ?? 0.02);
    const VtN_seg7 = N_LED * vt;
    const seg7Is = shockleyIsFromVf(seg7Vf, seg7iRated, N_LED, vt);
    const seg7vSat = Math.min(Math.max(40 * VtN_seg7, seg7Vf + 5 * VtN_seg7), 80 * VtN_seg7);
    let comN: number;
    if (!ctx.isOpenPin(comp.id, "com")) {
      comN = ctx.pinNode(comp.id, "com");
    } else if (!ctx.isOpenPin(comp.id, "com2")) {
      comN = ctx.pinNode(comp.id, "com2");
    } else {
      return; // no common pin wired — nothing to stamp
    }
    for (const seg of SEG7_PINS) {
      if (ctx.isOpenPin(comp.id, seg)) continue;
      const segN = ctx.pinNode(comp.id, seg);
      const [ai, ci] = seg7CommonAnode ? [comN, segN] : [segN, comN];
      const vd = ctx.opVoltage(ai) - ctx.opVoltage(ci);
      const gd = shockleyConductanceAtOp(vd, seg7Is, N_LED, vt, seg7vSat);
      stampAcAdmittance(ac, ai, ci, gd + AC_GMIN, 0);
    }
  },
};

// W8.2 — HD44780: 10 high-impedance logic inputs + supply load. Driven
// inputs are not burdened by invented pull-downs; genuinely open pins
// use the engine's disclosed floating-input decision model.
// Behavioural supply load: ~25 mA at 5 V represents logic + backlight draw.
export const hd44780Model: DeviceModel = {
  kinds: ["hd44780"],
  stamp: (ctx, comp, xGuess, _h) => {
    const powerLcd = ctx.icPowerInfo(comp, xGuess);
    // Behavioural supply load (~25 mA at 5 V for a typical LCD + backlight draw).
    // Actual draw depends on backlight brightness and contrast setting; this is a
    // teaching approximation sufficient for supply current and voltage-drop analysis.
    if (powerLcd.powered) {
      const vddNode = ctx.pinNode(comp.id, "vdd");
      const vssNode = ctx.pinNode(comp.id, "vss");
      if (vddNode >= 0 && !ctx.isOpenPin(comp.id, "vdd") && !ctx.isOpenPin(comp.id, "vss")) {
        stampResistor(ctx.mna, vddNode, vssNode, 200);
      }
    }
  },
  // W8.2 — HD44780 16x2 character LCD decoder.
  // Replays E-falling-edge-triggered latches from ordered sub-step MCU pin events
  // (event path) or falls back to sampled level comparison (one edge per step).
  updateDigital: (ctx, comp, x, _h) => {
    const power = ctx.icPowerInfo(comp, x);
    // Retrieve committed state (or fresh defaults for a new LCD).
    let st = ctx.state.lcds.get(comp.id) ?? defaultHd44780State();

    // 16 spaces for a blank 16-char line.
    const blank16 = "                ";

    if (!power.powered) {
      // Chip off: emit dark display, carry LCD state unchanged.
      ctx.setDisplayInfo(comp.id, { kind: "hd44780", lines: [blank16, blank16], on: false, cols: 16, rows: 2 });
      ctx.state.lcds.set(comp.id, st);
      return;
    }

    // Build (or reuse) the Arduino-pin-driver map for this step.
    const mergedLcd = ctx.mergedDisplayEvents(
      comp,
      ["e", "rs", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7"],
    );

    // Working copies seeded from committed state so partial-step events accumulate correctly.
    let lcdE: 0 | 1 = st.lastE;
    let lcdRs: 0 | 1 = st.lastRS;
    const lcdD: number[] = [...st.lastData];
    let lcdFourBit = st.fourBit;
    let lcdNibblePhase = st.nibblePhase;
    let lcdHiNib = st.hiNib;
    let lcdAddr = st.addr;
    let lcdDisplayOn = st.displayOn;
    let lcdIncrement = st.increment;
    let lcdCgram = st.cgram;
    let lcdTwoLine = st.twoLine;
    const lcdDdram = [...st.ddram];

    // processByte: execute one complete byte transfer (RS=rsBit, data=byte).
    // Mirrors the HD44780 command set; commands are matched from most-specific
    // (higher address bits) to least-specific per the datasheet decode table.
    function processByte(byte: number, rsBit: 0 | 1): void {
      if (rsBit === 1) {
        // Data write: store character code at current DDRAM address (CGRAM writes ignored in v1).
        if (!lcdCgram && lcdAddr < lcdDdram.length) {
          lcdDdram[lcdAddr] = byte;
          lcdAddr += lcdIncrement ? 1 : -1;
        }
        return;
      }
      // Command decode (RS=0) — priority order mirrors HD44780 datasheet instruction table.
      if (byte === 0x01) {
        // Display clear: fill DDRAM with spaces, reset address counter.
        lcdDdram.fill(0x20);
        lcdAddr = 0;
        lcdCgram = false;
      } else if ((byte & 0xfe) === 0x02) {
        // Return home: reset address counter (DDRAM unchanged).
        lcdAddr = 0;
        lcdCgram = false;
      } else if ((byte & 0xfc) === 0x04) {
        // Entry mode set: I/D bit selects increment (1) or decrement (0).
        lcdIncrement = (byte & 0x02) !== 0;
      } else if ((byte & 0xf8) === 0x08) {
        // Display on/off control: D bit controls display visibility.
        lcdDisplayOn = (byte & 0x04) !== 0;
      } else if ((byte & 0xf0) === 0x10) {
        // Cursor/display shift: accepted but not rendered in v1.
        // Shift is a cosmetic cursor/window operation not affecting DDRAM.
      } else if ((byte & 0xe0) === 0x20) {
        // Function set: DL=0 → 4-bit, DL=1 → 8-bit; N=1 → 2-line.
        lcdFourBit = (byte & 0x10) === 0;
        lcdTwoLine = (byte & 0x08) !== 0;
      } else if ((byte & 0xc0) === 0x40) {
        // Set CGRAM address: subsequent data writes go to CGRAM (ignored in v1).
        lcdCgram = true;
      } else if ((byte & 0x80) === 0x80) {
        // Set DDRAM address: bits 6-0 specify the DDRAM address.
        lcdAddr = byte & 0x7f;
        lcdCgram = false;
      }
    }

    // latchOnEFalling: called when E transitions 1→0 (data latched by HD44780 on falling E).
    // In 8-bit mode: form a full byte from all 8 data lines and process immediately.
    // In 4-bit mode: on phase=0 capture D7-D4 as high nibble; on phase=1 assemble and process.
    // NOTE: the full-8-bit byte form (using d0..d7) also works correctly with 4-bit wiring
    // because d0..d3 read 0 when unconnected (pulled low by 1M insurance stamps), so 0x30
    // in 8-bit representation is still 0x30.
    function latchOnEFalling(): void {
      if (!lcdFourBit) {
        // 8-bit mode: form the full byte from D7..D0. With 4-bit wiring d0..d3=0, so
        // 0x30/0x20/0x28 init nibbles still decode correctly via d4..d7 contributions.
        const fullByte =
          (lcdD[7] << 7) | (lcdD[6] << 6) | (lcdD[5] << 5) | (lcdD[4] << 4) |
          (lcdD[3] << 3) | (lcdD[2] << 2) | (lcdD[1] << 1) | lcdD[0];
        processByte(fullByte, lcdRs as 0 | 1);
      } else if (lcdNibblePhase === 0) {
        // 4-bit high nibble: capture D7-D4.
        lcdHiNib = (lcdD[7] << 3) | (lcdD[6] << 2) | (lcdD[5] << 1) | lcdD[4];
        lcdNibblePhase = 1;
      } else {
        // 4-bit low nibble: assemble full byte and process.
        const lowNib = (lcdD[7] << 3) | (lcdD[6] << 2) | (lcdD[5] << 1) | lcdD[4];
        processByte((lcdHiNib << 4) | lowNib, lcdRs as 0 | 1);
        lcdNibblePhase = 0;
      }
    }

    if (mergedLcd.events.length > 0) {
      // Event path: replay every MCU sub-step edge in order.
      // This handles full command sequences that span a single sim timestep.
      for (const { displayPin, level } of mergedLcd.events) {
        const lvl = (level === null ? 0 : level) as 0 | 1;
        if (displayPin === "e") {
          // E falling edge: latch the data bus into the HD44780.
          if (lcdE === 1 && lvl === 0) latchOnEFalling();
          lcdE = lvl;
        } else if (displayPin === "rs") {
          lcdRs = lvl;
        } else if (displayPin.startsWith("d")) {
          const idx = parseInt(displayPin.slice(1), 10);
          if (idx >= 0 && idx <= 7) lcdD[idx] = lvl;
        }
      }
    } else {
      // Sampled fallback: compare committed last levels to current solved levels.
      // Full multi-byte message sequences are only reliable with event-driven (MCU) drive;
      // this path captures at most one E-falling edge per sim step.
      const cE: 0 | 1 = ctx.logicHigh(comp, "e", x, power) ? 1 : 0;
      lcdRs = ctx.logicHigh(comp, "rs", x, power) ? 1 : 0;
      for (let i = 0; i < 8; i++) {
        lcdD[i] = ctx.logicHigh(comp, `d${i}`, x, power) ? 1 : 0;
      }
      // Detect E falling edge relative to committed previous E level.
      if (st.lastE === 1 && cE === 0) latchOnEFalling();
      lcdE = cE;
    }

    // Commit decoded state back into state.lcds for the next step.
    ctx.state.lcds.set(comp.id, {
      ddram: lcdDdram,
      addr: lcdAddr,
      fourBit: lcdFourBit,
      nibblePhase: lcdNibblePhase as 0 | 1,
      hiNib: lcdHiNib,
      displayOn: lcdDisplayOn,
      increment: lcdIncrement,
      cgram: lcdCgram,
      twoLine: lcdTwoLine,
      lastE: lcdE,
      lastRS: lcdRs,
      lastData: [...lcdD],
    });

    // Build the display lines from DDRAM.
    // Only printable ASCII 0x20-0x7E is rendered; all other codes display as a space.
    // CGRAM custom chars (0x00-0x07) and katakana (0xA0-0xFF) are deferred to a future wave.
    // HD44780 ROM quirks (0x5C=yen, 0x7E/0x7F=arrows) are treated as ASCII for v1.
    function lcdCharToDisplay(code: number): string {
      return code >= 0x20 && code <= 0x7e ? String.fromCharCode(code) : " ";
    }
    const line0 = lcdDdram.slice(0x00, 0x10).map(lcdCharToDisplay).join("");
    const line1 = lcdDdram.slice(0x40, 0x50).map(lcdCharToDisplay).join("");
    ctx.setDisplayInfo(comp.id, { kind: "hd44780", lines: [line0, line1], on: lcdDisplayOn, cols: 16, rows: 2 });
  },
  // W8.2 — HD44780: report VDD supply current from the behavioural load resistor.
  // I = (V_vdd − V_vss) / 200 Ω — mirrors the stamp handler's behavioral load stamp.
  updateCurrent: (ctx, comp, x) => {
    const powerLcdI = ctx.icPowerInfo(comp, x);
    if (powerLcdI.powered && !ctx.isOpenPin(comp.id, "vdd") && !ctx.isOpenPin(comp.id, "vss")) {
      const vddNodeI = ctx.pinNode(comp.id, "vdd");
      const vssNodeI = ctx.pinNode(comp.id, "vss");
      ctx.setElementCurrent(comp.id, (ctx.vAt(x, vddNodeI) - ctx.vAt(x, vssNodeI)) / 200);
    } else {
      ctx.setElementCurrent(comp.id, 0);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The behavioural 200 ohm supply load is the module's only analog
    // contribution; the bus pins are genuinely high-Z inputs.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const vddNode = ctx.pinNode(comp.id, "vdd");
    if (vddNode >= 0 && !ctx.isOpenPin(comp.id, "vdd") && !ctx.isOpenPin(comp.id, "vss")) {
      stampAcAdmittance(ac, vddNode, ctx.pinNode(comp.id, "vss"), 1 / 200, 0);
    }
  },
};

// W8.1 — MAX7219: high-impedance DIN/CLK/CS inputs + supply load.
export const max7219Model: DeviceModel = {
  kinds: ["max7219"],
  stamp: (ctx, comp, xGuess, _h) => {
    const powerMx = ctx.icPowerInfo(comp, xGuess);
    // Behavioural supply load (~33 mA at 5 V for a lit matrix).
    // This is a teaching approximation — actual draw depends on how many
    // LEDs are lit; per-LED accuracy is out of scope for v1.
    if (powerMx.powered) {
      const vccNode = ctx.pinNode(comp.id, "vcc");
      const gndNode = ctx.pinNode(comp.id, "gnd");
      if (vccNode >= 0 && !ctx.isOpenPin(comp.id, "vcc") && !ctx.isOpenPin(comp.id, "gnd")) {
        stampResistor(ctx.mna, vccNode, gndNode, 150);
      }
    }
  },
  // W8.1 — MAX7219: decode SPI-like frames from Arduino sub-step events
  // (preferred) or from sampled pin levels (fallback for discrete drive).
  updateDigital: (ctx, comp, x, _h) => {
    const power = ctx.icPowerInfo(comp, x);
    const st = ctx.state.icState.get(comp.id) ?? defaultIcState("max7219");

    if (!power.powered) {
      // Chip is off: emit dark display state, carry registers unchanged.
      ctx.setDisplayInfo(comp.id, {
        kind: "max7219",
        rows: [st.dig0, st.dig1, st.dig2, st.dig3, st.dig4, st.dig5, st.dig6, st.dig7],
        intensity: st.intensity,
        on: false,
      });
      ctx.state.icState.set(comp.id, st);
      return;
    }

    // Build (or reuse) the Arduino-pin-driver map for this step.
    const merged = ctx.mergedDisplayEvents(comp, ["din", "clk", "cs"]);

    // Working copies of register state seeded from last committed values.
    let din   = st.lastDIN;
    let clk   = st.lastCLK;
    let cs    = st.lastCS;
    let shift = st.shift;
    let bits  = st.bits;
    let dig0  = st.dig0, dig1 = st.dig1, dig2 = st.dig2, dig3 = st.dig3;
    let dig4  = st.dig4, dig5 = st.dig5, dig6 = st.dig6, dig7 = st.dig7;
    let decodeMode  = st.decodeMode;
    let intensity   = st.intensity;
    let scanLimit   = st.scanLimit;
    let shutdown    = st.shutdown;
    let displayTest = st.displayTest;

    // Write a decoded 16-bit frame into the appropriate register.
    function applyRegister(addr: number, data: number): void {
      switch (addr) {
        case 0x01: dig0 = data; break;
        case 0x02: dig1 = data; break;
        case 0x03: dig2 = data; break;
        case 0x04: dig3 = data; break;
        case 0x05: dig4 = data; break;
        case 0x06: dig5 = data; break;
        case 0x07: dig6 = data; break;
        case 0x08: dig7 = data; break;
        case 0x09: decodeMode = data;           break;
        case 0x0a: intensity   = data & 0x0f;   break;
        case 0x0b: scanLimit   = data & 0x07;   break;
        case 0x0c: shutdown    = data & 1;       break;
        case 0x0f: displayTest = data & 1;       break;
        // 0x00 = no-op; any other address is undefined — ignore both.
      }
    }

    if (merged.events.length > 0) {
      // Event path: replay every MCU sub-step edge in order.
      // This handles full 16-bit frames that cross a sim timestep boundary.
      for (const { displayPin, level } of merged.events) {
        const lvl = level === null ? 0 : level;
        if (displayPin === "din") {
          din = lvl;
        } else if (displayPin === "clk") {
          if (clk === 0 && lvl === 1) {
            // Rising CLK: shift in DIN (MSB first).
            shift = ((shift << 1) | din) & 0xffff;
            bits++;
          }
          clk = lvl;
        } else if (displayPin === "cs") {
          if (cs === 1 && lvl === 0) {
            // CS falling: start of a new frame — reset the shift register.
            shift = 0;
            bits = 0;
          } else if (cs === 0 && lvl === 1 && bits >= 16) {
            // CS rising with a full frame: latch the register write.
            applyRegister((shift >> 8) & 0xff, shift & 0xff);
          }
          cs = lvl;
        }
      }
    } else {
      // Sampled fallback: compare committed last levels to current sampled levels.
      // Full 16-bit framing over many steps is only reliable with event-driven
      // (Arduino) drive; this path is best-effort for discrete voltage sources.
      const cDin = ctx.logicHigh(comp, "din", x, power) ? 1 : 0;
      const cClk = ctx.logicHigh(comp, "clk", x, power) ? 1 : 0;
      const cCs  = ctx.logicHigh(comp, "cs",  x, power) ? 1 : 0;
      din = cDin;
      // Handle CS transition (evaluated before CLK to respect SPI protocol order).
      if (st.lastCS === 1 && cCs === 0) {
        shift = 0;
        bits = 0;
      } else if (st.lastCS === 0 && cCs === 1 && bits >= 16) {
        applyRegister((shift >> 8) & 0xff, shift & 0xff);
      }
      cs = cCs;
      // Handle CLK transition.
      if (st.lastCLK === 0 && cClk === 1) {
        shift = ((shift << 1) | din) & 0xffff;
        bits++;
      }
      clk = cClk;
    }

    // Commit the decoded state back into icState.
    ctx.state.icState.set(comp.id, {
      shift, bits,
      dig0, dig1, dig2, dig3, dig4, dig5, dig6, dig7,
      decodeMode, intensity, scanLimit, shutdown, displayTest,
      lastCLK: clk, lastDIN: din, lastCS: cs,
    });

    // Build the display state, honouring shutdown / displayTest / scanLimit.
    // shutdown=1 = normal operation; shutdown=0 = blank (chip in shutdown mode).
    // Display-test mode lights every LED and overrides shutdown (datasheet),
    // so the display is "on" whenever either is asserted.
    const on = shutdown === 1 || displayTest === 1;
    const effRows: number[] = [];
    for (let r = 0; r < 8; r++) {
      if (displayTest === 1) {
        // Display test overrides everything: all LEDs on.
        effRows.push(0xff);
      } else if (!on) {
        effRows.push(0);
      } else {
        const digVal = [dig0, dig1, dig2, dig3, dig4, dig5, dig6, dig7][r];
        // scanLimit 0–7: only rows 0..scanLimit are active; rest are blank.
        effRows.push(r <= scanLimit ? digVal : 0);
      }
    }

    ctx.setDisplayInfo(comp.id, { kind: "max7219", rows: effRows, intensity, on });
  },
  // W8.1 — MAX7219: report VCC supply current from the behavioural load resistor.
  // I = (V_vcc − V_gnd) / 150 Ω — mirrors the stamp handler's behavioral load stamp.
  updateCurrent: (ctx, comp, x) => {
    const powerMxI = ctx.icPowerInfo(comp, x);
    if (powerMxI.powered && !ctx.isOpenPin(comp.id, "vcc") && !ctx.isOpenPin(comp.id, "gnd")) {
      const vccNodeMxI = ctx.pinNode(comp.id, "vcc");
      const gndNodeMxI = ctx.pinNode(comp.id, "gnd");
      ctx.setElementCurrent(comp.id, (ctx.vAt(x, vccNodeMxI) - ctx.vAt(x, gndNodeMxI)) / 150);
    } else {
      ctx.setElementCurrent(comp.id, 0);
    }
  },
  acStamp: (ctx, comp, ac, _omega) => {
    // The behavioural 150 ohm supply load is the module's only analog
    // contribution; DIN/CLK/CS are genuinely high-Z inputs.
    const power = ctx.icPowerInfoAtOp(comp);
    if (!power.powered) return;
    const vccNode = ctx.pinNode(comp.id, "vcc");
    if (vccNode >= 0 && !ctx.isOpenPin(comp.id, "vcc") && !ctx.isOpenPin(comp.id, "gnd")) {
      stampAcAdmittance(ac, vccNode, ctx.pinNode(comp.id, "gnd"), 1 / 150, 0);
    }
  },
};
