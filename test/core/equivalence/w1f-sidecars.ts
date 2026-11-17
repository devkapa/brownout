/**
 * W1f sidecars (core/README.md, "Kernel semantics: rails and sources (W1f)"
 * and "Equivalence harness rules", W1f adds). Each is computed from the
 * model's own equations and classic's own model functions, never by core
 * code:
 *
 * - `pulse-rc`: a resistor and capacitor fed by a pulse source. The source is
 *   classic's `pulseVoltage`, piecewise linear, so the first-order node has a
 *   closed form on every linear piece (Sources point 3: `pulse-161` below the
 *   ramp threshold, `pulse-ramp-1ms` above it, coordinator D3 (b)).
 * - `psu-rail`: a limited bench PSU with a rail capacitor and switched
 *   resistive loads, through CC entries and exits (Rails points 4 and 5, P9).
 *   After an exit the capacitor relaxes through its `esr` with `esr·C`; the
 *   samples from an exit to the next entry compare against this exact value,
 *   not classic (coordinator, P9 option (a)).
 * - `battery-drain`: a battery into a resistive load. SoC follows the ODE of
 *   Rails point 6, integrated here to well below the tolerance; SoC,
 *   `dischargedCoulombs` and the OCV compare against it (Equivalence harness
 *   rules, battery drain).
 * - `ramp-crossings`: a Schmitt input on a resolved pulse ramp crosses its
 *   threshold at an instant that is linear in `t`, and the output follows
 *   after the part's propagation delay (Sources point 3, resolved ramps).
 */

import { buildCoreNodes, corePinKey, type CoreNodeBuild } from "../../../src/core/compile/topology.js";
import { resolveCatalogPart } from "../../../src/circuit/catalog-resolver.js";
import { batteryModelForComponent, batteryOperatingPointFor, authoredBatteryCharge, propagationDelayForComponent } from "../../../src/sim/engine/sim-engine.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import type { ComponentKind } from "../../../src/circuit/types.js";
import { NODE_SHUNT_SIEMENS } from "./analytic.js";
import type { StimulusPhase } from "./stimuli.js";
import type { AnalyticPsuRailModel, AnalyticPulseRcModel, BatteryDrainModel, PsuRailGateModel, RampCrossingsModel } from "./types.js";

type SimComponent = SimCircuit["components"][number];

function componentOf(circuit: SimCircuit, id: string, kinds: readonly string[]): SimComponent {
  const comp = circuit.components.find((c) => c.id === id);
  if (!comp) throw new Error(`w1f sidecar: no component "${id}"`);
  if (!kinds.includes(comp.kind)) throw new Error(`w1f sidecar: "${id}" is a ${comp.kind}, expected ${kinds.join(" or ")}`);
  return comp;
}

function num(comp: SimComponent, key: string, fallback: number): number {
  const v = comp.params[key];
  return v === undefined ? fallback : Number(v);
}

function nodeOf(built: CoreNodeBuild, componentId: string, pinId: string): number {
  const node = built.pinNode.get(corePinKey(componentId, pinId));
  if (node === undefined) throw new Error(`w1f sidecar: pin ${componentId}.${pinId} is on no node`);
  return node;
}

function netIdOf(built: CoreNodeBuild, node: number): string {
  const id = built.netIdOfNode[node];
  if (!id) throw new Error(`w1f sidecar: node ${node} has no net id`);
  return id;
}

/** The capacitor's own `esr` and leakage must be absent: the closed forms below hold an ideal capacitor plus the node shunt. */
function idealCapacitance(cap: SimComponent): number {
  const c = num(cap, "capacitance", Number.NaN);
  if (!(c > 0)) throw new Error(`w1f sidecar: ${cap.id} has no positive capacitance`);
  if (num(cap, "esr", 0) !== 0 || num(cap, "leakage", 0) !== 0) {
    throw new Error(`w1f sidecar: ${cap.id} carries esr or leakage, which this closed form does not hold`);
  }
  return c;
}

// ─── pulse sources ───────────────────────────────────────────────────────────

export interface PulseParams {
  readonly v1: number;
  readonly v2: number;
  readonly td: number;
  readonly tr: number;
  readonly tf: number;
  readonly pw: number;
  /** 0 is classic's one-shot. */
  readonly per: number;
}

/** The compiler's reading of a `pulse_source`/`pulse_gen` (compile/sources.ts, classic's defaults), from the circuit itself. */
export function pulseParamsOf(comp: SimComponent): PulseParams {
  const rawPer = num(comp, "per", 1e-3);
  return {
    v1: num(comp, "v1", 0),
    v2: num(comp, "v2", 5),
    td: num(comp, "td", 0),
    tr: num(comp, "tr", 1e-6),
    tf: num(comp, "tf", 1e-6),
    pw: num(comp, "pw", 5e-4),
    per: Number.isFinite(rawPer) && rawPer > 0 ? rawPer : 0,
  };
}

/** One linear piece of the source: `v(t) = v0 + (v1 - v0) (t - t0) / (t1 - t0)` on `[t0, t1)`. A zero-length piece never occurs; a step is two pieces meeting at a jump. */
export interface LinearPiece {
  readonly t0: number;
  readonly t1: number;
  readonly v0: number;
  readonly v1: number;
}

/**
 * `pulseVoltage` as linear pieces from 0 to `horizonS`. The corners are
 * computed as the core's schedule computes them, `(td + c·per) + offset` with
 * the offsets `tr`, `tr + pw` and `(tr + pw) + tf` (Sources point 3), so the
 * pieces meet classic's waveform everywhere but on fmod's own rounding at a
 * corner.
 */
export function pulsePieces(p: PulseParams, horizonS: number): LinearPiece[] {
  if (!(p.td >= 0)) throw new Error("pulsePieces: a negative td is not modelled here");
  const pieces: LinearPiece[] = [];
  const push = (t0: number, t1: number, v0: number, v1: number): void => {
    if (t1 > t0 && t0 < horizonS) pieces.push({ t0, t1, v0, v1 });
  };
  push(0, p.td, p.v1, p.v1);
  for (let c = 0; ; c++) {
    if (p.per === 0 && c > 0) break;
    const base = p.td + c * p.per;
    if (base >= horizonS) break;
    const rise = base + p.tr;
    const fall = base + (p.tr + p.pw);
    const end = base + (p.tr + p.pw + p.tf);
    const next = p.per === 0 ? Number.POSITIVE_INFINITY : p.td + (c + 1) * p.per;
    push(base, rise, p.v1, p.v2);
    push(rise, fall, p.v2, p.v2);
    push(fall, end, p.v2, p.v1);
    push(end, Math.min(next, Number.MAX_VALUE), p.v1, p.v1);
    if (p.per === 0) break;
  }
  return pieces;
}

export function pieceValueAt(piece: LinearPiece, t: number): number {
  return piece.v0 + ((piece.v1 - piece.v0) * (t - piece.t0)) / (piece.t1 - piece.t0);
}

/** Sources point 3's `RAMP_STEP_RATIO`. */
export const RAMP_STEP_RATIO = 0.01;

/**
 * The pulse as the model defines it (Sources point 3): each ramp is a
 * midpoint step when `t_r <= RAMP_STEP_RATIO · T_ev` (or `t_r = 0`), with `T_ev`
 * the shortest flat segment of the cycle, and linear otherwise. A step is a
 * jump between two flat pieces at `+ tr/2` (rise) or `+ (tr + pw) + tf/2`
 * (fall). This is the waveform the core is exact for; below the threshold it
 * differs from `pulseVoltage` only inside a ramp, by at most the ramp's
 * own swing for at most half its length.
 */
export function pulseModelPieces(p: PulseParams, horizonS: number): LinearPiece[] {
  const tEv = p.per > 0 ? Math.min(p.pw, p.per - p.tr - p.pw - p.tf) : p.pw;
  const riseIsStep = p.tr === 0 || p.tr <= RAMP_STEP_RATIO * tEv;
  const fallIsStep = p.tf === 0 || p.tf <= RAMP_STEP_RATIO * tEv;
  if (!riseIsStep && !fallIsStep) return pulsePieces(p, horizonS);
  const out: LinearPiece[] = [];
  for (const piece of pulsePieces(p, horizonS)) {
    const ramp = piece.v0 !== piece.v1;
    const isStep = ramp && (piece.v1 === p.v2 ? riseIsStep : fallIsStep);
    if (!isStep) {
      out.push(piece);
      continue;
    }
    // A rise runs v1 -> v2 (`tr`), a fall v2 -> v1 (`tf`), whichever level is higher.
    const mid = piece.t0 + (piece.v1 === p.v2 ? p.tr : p.tf) / 2;
    if (mid > piece.t0) out.push({ t0: piece.t0, t1: mid, v0: piece.v0, v1: piece.v0 });
    out.push({ t0: mid, t1: piece.t1, v0: piece.v1, v1: piece.v1 });
  }
  // Merge the flat runs a step leaves behind, so a jump is the only discontinuity.
  const merged: LinearPiece[] = [];
  for (const piece of out) {
    const last = merged[merged.length - 1];
    if (last && last.v0 === last.v1 && piece.v0 === piece.v1 && last.v1 === piece.v0 && last.t1 === piece.t0) {
      merged[merged.length - 1] = { ...last, t1: piece.t1 };
    } else {
      merged.push(piece);
    }
  }
  return merged;
}

// ─── pulse-rc ────────────────────────────────────────────────────────────────

export interface PulseRcSidecar {
  /** The capacitor's net id. */
  readonly node: string;
  /** The source's `pos` net id. */
  readonly sourceNode: string;
  readonly resistor: string;
  readonly capacitor: string;
  readonly source: string;
  readonly voltageAt: (t: number) => number;
  readonly sourceAt: (t: number) => number;
  /** The resistor's `elementI` (a to b, source side to node). */
  readonly resistorCurrentAt: (t: number) => number;
  /** The capacitor's `elementI` (into its pin on the node). */
  readonly capacitorCurrentAt: (t: number) => number;
  /** The source's `elementI` when only the resistor and the node shunt draw from its node: `-(I_R + g s)` (W0 point 5's sign). */
  readonly sourceCurrentAt: (t: number) => number;
}

/**
 * The node `x` of `source.pos --R-- x --C-- datum`, with the node shunt to the
 * datum: `C x' = (s(t) - x)/R - g x`. On a piece `s = s0 + m τ` the solution
 * is `x = A + B τ + (x0 - A) e^(-a τ)` with `a = (1/R + g)/C`, `B = m/(R C a)`
 * and `A = (s0/(R C) - B)/a`, exact. The capacitor starts uncharged at the
 * load (a fresh load).
 */
export function extractPulseRc(circuit: SimCircuit, model: AnalyticPulseRcModel, horizonS: number): PulseRcSidecar {
  const built = buildCoreNodes(circuit);
  const src = componentOf(circuit, model.source, ["pulse_source", "pulse_gen"]);
  const res = componentOf(circuit, model.resistor, ["resistor"]);
  const cap = componentOf(circuit, model.capacitor, ["capacitor"]);
  const R = num(res, "resistance", Number.NaN);
  if (!(R > 0)) throw new Error(`pulse-rc: ${res.id} has no positive resistance`);
  const C = idealCapacitance(cap);

  const sPos = nodeOf(built, src.id, src.pins[0]!.id);
  const sNeg = nodeOf(built, src.id, src.pins[1]!.id);
  const ra = nodeOf(built, res.id, res.pins[0]!.id);
  const rb = nodeOf(built, res.id, res.pins[1]!.id);
  const ca = nodeOf(built, cap.id, cap.pins[0]!.id);
  const cb = nodeOf(built, cap.id, cap.pins[1]!.id);
  if (sNeg !== 0 || cb !== 0 || ra !== sPos || rb !== ca || ca === 0) {
    throw new Error("pulse-rc: expected source.pos - R.a, R.b - C.a, and source.neg and C.b on the datum");
  }
  const otherOnNode = circuit.components.some(
    (c) => c.id !== res.id && c.id !== cap.id && c.pins.some((p) => built.pinNode.get(corePinKey(c.id, p.id)) === ca),
  );
  if (otherOnNode) throw new Error("pulse-rc: the capacitor's node carries another device");

  // A sample instant is classic's simTime, which can sit a rounding past the requested horizon.
  const reach = horizonS * 1.5;
  const pieces = pulseModelPieces(pulseParamsOf(src), reach);
  const g = NODE_SHUNT_SIEMENS;
  const a = (1 / R + g) / C;
  const b = 1 / (R * C);
  // The state at each piece's start, by marching the closed form piece to piece.
  const startX: number[] = [];
  let x = 0;
  for (const piece of pieces) {
    startX.push(x);
    const m = (piece.v1 - piece.v0) / (piece.t1 - piece.t0);
    const B = (b * m) / a;
    const A = (b * piece.v0 - B) / a;
    const tau = piece.t1 - piece.t0;
    x = Number.isFinite(tau) ? A + B * tau + (x - A) * Math.exp(-a * tau) : A;
  }
  const pieceIndex = (t: number): number => {
    let lo = 0;
    let hi = pieces.length - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (pieces[mid]!.t0 <= t) lo = mid;
      else hi = mid - 1;
    }
    return lo;
  };
  const evalAt = (t: number): { x: number; s: number } => {
    if (t > reach) throw new Error(`pulse-rc: t=${t} is past the sidecar's reach ${reach}`);
    const k = pieceIndex(t);
    const piece = pieces[k]!;
    const m = (piece.v1 - piece.v0) / (piece.t1 - piece.t0);
    const B = (b * m) / a;
    const A = (b * piece.v0 - B) / a;
    const tau = t - piece.t0;
    return { x: A + B * tau + (startX[k]! - A) * Math.exp(-a * tau), s: piece.v0 + m * tau };
  };
  return {
    node: netIdOf(built, ca),
    sourceNode: netIdOf(built, sPos),
    resistor: res.id,
    capacitor: cap.id,
    source: src.id,
    voltageAt: (t) => evalAt(t).x,
    sourceAt: (t) => evalAt(t).s,
    resistorCurrentAt: (t) => {
      const { x, s } = evalAt(t);
      return (s - x) / R;
    },
    capacitorCurrentAt: (t) => {
      const { x, s } = evalAt(t);
      return (s - x) / R - g * x;
    },
    sourceCurrentAt: (t) => {
      const { x, s } = evalAt(t);
      return -((s - x) / R + g * s);
    },
  };
}

// ─── psu-rail (CC entry and exit, P9) ──────────────────────────────────────

export interface PsuRailRegime {
  /** When this regime starts. */
  readonly t0: number;
  readonly cc: boolean;
}

export interface PsuRailSidecar {
  readonly rail: string;
  readonly psu: string;
  readonly capacitor: string;
  /** Every regime change, in order, the load's CC first (an uncharged rail capacitor is demand). */
  readonly regimes: readonly PsuRailRegime[];
  /** CC exits: the rail reaches the setpoint (Rails point 5, with a rail capacitor). */
  readonly exitsS: readonly number[];
  /**
   * True from a CC exit until the next CC entry: the samples the P9 ruling
   * judges against this sidecar. Every function here takes the sample's
   * phase index when it has one: a sample on a phase boundary belongs to the
   * phase that ends there (stimuli.ts samples before the next `set`), which
   * its time alone cannot say once the phase durations have been summed.
   */
  readonly afterExit: (t: number, phase?: number) => boolean;
  readonly railAt: (t: number, phase?: number) => number;
  /** The rail capacitor's `elementI` (into its rail pin). */
  readonly capacitorCurrentAt: (t: number, phase?: number) => number;
  /** The PSU's `elementI`: `-iLimit` in CC, `-(draw)` in CV (Rails point 5). */
  readonly psuCurrentAt: (t: number, phase?: number) => number;
  /** Each load resistor's `elementI` (and its switch's, when it has one), keyed by component id. */
  readonly loadCurrentAt: ReadonlyMap<string, (t: number, phase?: number) => number>;
}

/** A closed switch contact: `1 mΩ` (classic's passives.ts, the core's switches.rs). */
const SWITCH_CONTACT_SIEMENS = 1 / 0.001;

interface RailLoad {
  readonly resistor: string;
  readonly switch?: string;
  readonly ohms: number;
  /** The resistor's `elementI` sign: +1 when its pin a is on the rail side. */
  readonly sign: 1 | -1;
}

/**
 * A `bench_psu` (`V > 0`, `iLimit > 0`) with one rail capacitor (`C`, `esr`)
 * from its `pos` to the datum, and resistive loads to the datum, each
 * directly on the rail or through one switch the entry's phases toggle. The
 * rail node also has its shunt `g`. With the load conductance `G` (shunt
 * included) constant between phase boundaries, Rails points 4, 5 and 7 give
 * a closed form on every stretch:
 * - CC: `I_cap = (iLimit - G vc)/(1 + esr G)` into the capacitor, so `vc` is
 *   one exponential toward `iLimit/G` at rate `G/(C (1 + esr G))`, and the rail
 *   is `u = (vc + esr iLimit)/(1 + esr G)`, `esr·I_cap` above the charge. It
 *   leaves CC when `u` reaches `V`, at `vc_x = V (1 + esr G) - esr iLimit`.
 * - CV: `u = V`; the capacitor relaxes to `V` through `esr` with `esr·C`
 *   (rule 2a; `vc = V` at once when `esr = 0`), carrying `(V - vc)/esr`. It
 *   enters CC when the draw at the setpoint, `G V` plus that capacitor
 *   current, exceeds `iLimit` strictly, which with constant `G` can only
 *   happen at a phase boundary (or at the load, where the uncharged
 *   capacitor is the demand).
 * The capacitor's charge is continuous; the rail steps at an entry by the
 * `esr` offset.
 *
 * A switched load is the switch's closed contact, `1 mΩ` in classic
 * (passives.ts) and the core (switches.rs) alike, in series with the
 * resistor and the node between them with its shunt `g`. Leaving the contact
 * out moved fixture 45's asymptote by 6.4e-5 V and its second exit by 1.1 us,
 * which F4 read as the core missing the coupled solution (D10); with it the
 * core agrees to 6e-12 V.
 */
export function extractPsuRail(circuit: SimCircuit, model: AnalyticPsuRailModel, phases: readonly StimulusPhase[] = []): PsuRailSidecar {
  const built = buildCoreNodes(circuit);
  const psu = componentOf(circuit, model.psu, ["bench_psu"]);
  const cap = componentOf(circuit, model.capacitor, ["capacitor"]);
  const V = num(psu, "voltage", 5);
  const iLimit = num(psu, "iLimit", 0);
  if (!(V > 0) || !(iLimit > 0)) throw new Error("psu-rail: needs a positive setpoint and a positive iLimit");
  const C = num(cap, "capacitance", Number.NaN);
  const esr = num(cap, "esr", 0);
  if (!(C > 0) || !(esr >= 0) || num(cap, "leakage", 0) !== 0) throw new Error(`psu-rail: ${cap.id} must be a capacitor with esr >= 0 and no leakage`);

  const pos = nodeOf(built, psu.id, psu.pins[0]!.id);
  if (nodeOf(built, psu.id, psu.pins[1]!.id) !== 0 || nodeOf(built, cap.id, cap.pins[0]!.id) !== pos || nodeOf(built, cap.id, cap.pins[1]!.id) !== 0) {
    throw new Error("psu-rail: expected the capacitor from the PSU's pos to the datum, and the PSU's neg on the datum");
  }
  const loads: RailLoad[] = (model.loads ?? []).map((spec) => {
    const res = componentOf(circuit, spec.resistor, ["resistor"]);
    const ohms = num(res, "resistance", Number.NaN);
    if (!(ohms > 0)) throw new Error(`psu-rail: ${res.id} has no positive resistance`);
    const ra = nodeOf(built, res.id, res.pins[0]!.id);
    const rb = nodeOf(built, res.id, res.pins[1]!.id);
    let top = pos;
    if (spec.switch) {
      const sw = componentOf(circuit, spec.switch, ["switch"]);
      const sa = nodeOf(built, sw.id, sw.pins[0]!.id);
      const sb = nodeOf(built, sw.id, sw.pins[1]!.id);
      if (sa !== pos || (sb !== ra && sb !== rb)) throw new Error(`psu-rail: expected ${sw.id} from the rail (its pin a) to ${res.id}`);
      top = sb;
      const onTop = circuit.components.filter((c) => c.id !== sw.id && c.id !== res.id && c.pins.some((p) => built.pinNode.get(corePinKey(c.id, p.id)) === top));
      if (onTop.length > 0) throw new Error(`psu-rail: the node between ${sw.id} and ${res.id} carries ${onTop.map((c) => c.id).join(", ")}`);
    }
    if (!((ra === top && rb === 0) || (rb === top && ra === 0))) throw new Error(`psu-rail: expected ${res.id} from ${spec.switch ? "its switch" : "the rail"} to the datum`);
    return { resistor: res.id, ...(spec.switch ? { switch: spec.switch } : {}), ohms, sign: ra === top ? 1 : -1 };
  });
  const declared = new Set([psu.id, cap.id, ...loads.flatMap((l) => (l.switch ? [l.switch] : [l.resistor]))]);
  const others = circuit.components.filter((c) => !declared.has(c.id) && c.pins.some((p) => built.pinNode.get(corePinKey(c.id, p.id)) === pos));
  if (others.length > 0) throw new Error(`psu-rail: the rail carries ${others.map((c) => c.id).join(", ")}, which this closed form does not hold`);

  // Switch states per phase: phase 0's `set` is folded into the load (stimuli.ts), each later one applies at its start.
  const closedAt = new Map<string, boolean>();
  for (const l of loads) {
    if (!l.switch) continue;
    const sw = circuit.components.find((c) => c.id === l.switch)!;
    closedAt.set(l.switch, Number(sw.params.closed ?? 0) >= 0.5);
  }
  interface Stretch { readonly t0: number; readonly t1: number; readonly closed: ReadonlyMap<string, boolean>; readonly phase: number }
  const stretches: Stretch[] = [];
  const phaseList = phases.length > 0 ? phases : [{ durationS: Number.POSITIVE_INFINITY }];
  let tPhase = 0;
  phaseList.forEach((phase, index) => {
    for (const set of phase.set ?? []) {
      if (closedAt.has(set.id) && set.params.closed !== undefined) closedAt.set(set.id, Number(set.params.closed) >= 0.5);
    }
    stretches.push({ t0: tPhase, t1: tPhase + phase.durationS, closed: new Map(closedAt), phase: index });
    tPhase += phase.durationS;
  });
  const g = NODE_SHUNT_SIEMENS;
  // What a load draws per volt of rail, and what share of the rail its resistor sees: behind a switch the
  // contact and the resistor (with the shunt of the node between them) divide the rail.
  const loadShare = (l: RailLoad): number => (l.switch ? SWITCH_CONTACT_SIEMENS / (SWITCH_CONTACT_SIEMENS + 1 / l.ohms + g) : 1);
  const loadSiemens = (l: RailLoad): number => (l.switch ? SWITCH_CONTACT_SIEMENS * (1 - loadShare(l)) : 1 / l.ohms);
  const conductance = (closed: ReadonlyMap<string, boolean>): number =>
    g + loads.reduce((sum, l) => sum + (!l.switch || closed.get(l.switch) ? loadSiemens(l) : 0), 0);

  // Closed-form pieces: in CC vc follows one exponential, in CV the esr relaxation.
  interface Piece { readonly t0: number; readonly t1: number; readonly cc: boolean; readonly vc0: number; readonly G: number; readonly closed: ReadonlyMap<string, boolean>; readonly phase: number }
  const pieces: Piece[] = [];
  const regimes: PsuRailRegime[] = [];
  const exitsS: number[] = [];
  const vcAt = (p: Piece, t: number): number => {
    const dt = t - p.t0;
    if (p.cc) {
      const k = p.G / (C * (1 + esr * p.G));
      const vcInf = iLimit / p.G;
      // With only the node shunt, vcInf is 1e11 V: expm1 keeps the ramp's digits.
      return p.vc0 - (vcInf - p.vc0) * Math.expm1(-k * dt);
    }
    return esr > 0 ? V - (V - p.vc0) * Math.exp(-dt / (esr * C)) : V;
  };
  const capI = (p: Piece, t: number): number => {
    const vc = vcAt(p, t);
    if (p.cc) return (iLimit - p.G * vc) / (1 + esr * p.G);
    return esr > 0 ? (V - vc) / esr : 0;
  };
  let vc = 0;
  let cc: boolean | undefined;
  for (const st of stretches) {
    const G = conductance(st.closed);
    let t = st.t0;
    // The CV -> CC test at the stretch's start: the draw at the setpoint, the rail capacitor's deficit included (Rails point 5).
    const deficitDraw = vc < V ? (esr > 0 ? (V - vc) / esr : Number.POSITIVE_INFINITY) : 0;
    if (cc !== true && G * V + deficitDraw > iLimit) {
      cc = true;
      regimes.push({ t0: t, cc: true });
    } else if (cc === undefined) {
      cc = false;
      regimes.push({ t0: t, cc: false });
    }
    while (t < st.t1) {
      if (cc) {
        const vcInf = iLimit / G;
        const vcX = V * (1 + esr * G) - esr * iLimit;
        const k = G / (C * (1 + esr * G));
        // The rail reaches V only rising toward an asymptote above it.
        const exit = vc < vcX && vcInf > vcX ? t - Math.log1p((vcX - vc) / (vc - vcInf)) / k : Number.POSITIVE_INFINITY;
        const end = Math.min(exit, st.t1);
        const piece = { t0: t, t1: end, cc: true, vc0: vc, G, closed: st.closed, phase: st.phase };
        pieces.push(piece);
        if (exit <= st.t1) {
          vc = vcX;
          cc = false;
          exitsS.push(exit);
          regimes.push({ t0: exit, cc: false });
        } else {
          vc = vcAt(piece, end);
        }
        t = end;
      } else {
        const piece = { t0: t, t1: st.t1, cc: false, vc0: vc, G, closed: st.closed, phase: st.phase };
        pieces.push(piece);
        vc = Number.isFinite(st.t1) ? vcAt(piece, st.t1) : V;
        t = st.t1;
      }
    }
  }
  // The last piece starting before t, within the sample's own phase when it names one.
  const pieceAt = (t: number, phase?: number): Piece => {
    let found: Piece | undefined;
    for (const p of pieces) {
      if (phase !== undefined && p.phase !== phase) continue;
      if (found === undefined || p.t0 < t) found = p;
    }
    if (!found) throw new Error(`psu-rail: no piece for phase ${String(phase)}`);
    return found;
  };
  const railAt = (t: number, phase?: number): number => {
    const p = pieceAt(t, phase);
    return p.cc ? (vcAt(p, t) + esr * iLimit) / (1 + esr * p.G) : V;
  };
  const loadCurrentAt = new Map<string, (t: number, phase?: number) => number>();
  for (const l of loads) {
    const on = (t: number, phase?: number): boolean => !l.switch || pieceAt(t, phase).closed.get(l.switch) === true;
    // The resistor carries its node's share of the rail; the switch also carries that node's shunt.
    const f = (t: number, phase?: number): number => (on(t, phase) ? (railAt(t, phase) * loadShare(l)) / l.ohms : 0);
    loadCurrentAt.set(l.resistor, (t, phase) => l.sign * f(t, phase));
    if (l.switch) loadCurrentAt.set(l.switch, (t, phase) => (on(t, phase) ? railAt(t, phase) * loadSiemens(l) : 0));
  }
  const afterExit = (t: number, phase?: number): boolean => {
    const p = pieceAt(t, phase);
    if (p.cc) return false;
    // The regime that governs the piece began at or before the piece did.
    let last: PsuRailRegime | undefined;
    for (const r of regimes) if (r.t0 <= p.t0) last = r;
    return last !== undefined && !last.cc && exitsS.includes(last.t0);
  };
  return {
    rail: netIdOf(built, pos),
    psu: psu.id,
    capacitor: cap.id,
    regimes,
    exitsS,
    afterExit,
    railAt,
    capacitorCurrentAt: (t, phase) => capI(pieceAt(t, phase), t),
    psuCurrentAt: (t, phase) => {
      const p = pieceAt(t, phase);
      return p.cc ? -iLimit : -(p.G * V + capI(p, t));
    },
    loadCurrentAt,
  };
}

// ─── battery-drain ───────────────────────────────────────────────────────────

export interface BatteryDrainSidecar {
  readonly rail: string;
  /** The exact state at `t` (integrated, see `extractBatteryDrain`). */
  readonly at: (t: number) => { readonly soc: number; readonly dischargedCoulombs: number; readonly openCircuitVoltageV: number; readonly currentA: number; readonly railV: number };
  /** `|I|` averaged over `[0, t]` (the README's integrated tolerance reads it). */
  readonly meanCurrentA: (t: number) => number;
  readonly capacityCoulombs: number;
}

/**
 * A `battery_pack` into one resistor to the datum. With `E(soc)` and
 * `r(soc)` from classic's own model (`batteryModelForComponent`,
 * `batteryOperatingPointFor`, OCV times the nominal-voltage scale, as the
 * stamp uses), the rail node's conductance `G = 1/R + g` draws `I =
 * E G/(1 + r G)`, and the charge out of `pos` obeys `Q' = I(soc0 - Q/Qcap)`
 * (Rails point 6). This integrates that ODE with classical RK4 at `h`, and
 * again at `h/2`; the generator refuses to return a table whose two
 * integrations disagree by more than 1e-12 relative in `Q`, so the table is
 * a reference far inside the tolerance it is used with. The cell must not
 * empty inside the horizon (a clamped SoC has a kink RK4 does not resolve).
 */
export function extractBatteryDrain(circuit: SimCircuit, model: BatteryDrainModel, horizonS: number): BatteryDrainSidecar {
  const built = buildCoreNodes(circuit);
  const bat = componentOf(circuit, model.battery, ["battery_pack"]);
  const load = componentOf(circuit, model.load, ["resistor"]);
  const R = num(load, "resistance", Number.NaN);
  if (!(R > 0)) throw new Error(`battery-drain: ${load.id} has no positive resistance`);
  const pos = nodeOf(built, bat.id, bat.pins[0]!.id);
  const neg = nodeOf(built, bat.id, bat.pins[1]!.id);
  const la = nodeOf(built, load.id, load.pins[0]!.id);
  const lb = nodeOf(built, load.id, load.pins[1]!.id);
  if (neg !== 0 || !((la === pos && lb === 0) || (lb === pos && la === 0))) {
    throw new Error("battery-drain: expected the load from the battery's pos to the datum, and the battery's neg on the datum");
  }
  const others = circuit.components.filter(
    (c) => c.id !== bat.id && c.id !== load.id && c.pins.some((p) => built.pinNode.get(corePinKey(c.id, p.id)) === pos),
  );
  if (others.length > 0) throw new Error(`battery-drain: the rail carries ${others.map((c) => c.id).join(", ")}`);

  const batteryModel = batteryModelForComponent(bat);
  // Classic's `_ambientTempC`: the circuit's environment, else 25 °C.
  const ambient = Number(circuit.environment?.temperatureC ?? 25);
  const soc0 = Math.max(0, Math.min(1, authoredBatteryCharge(bat)));
  const qCap = batteryModel.profile.referenceCapacityAh * 3600;
  const G = 1 / R + NODE_SHUNT_SIEMENS;
  const point = (soc: number): { E: number; r: number; ocv: number } => {
    const p = batteryOperatingPointFor(batteryModel, soc, ambient);
    const ocv = p.openCircuitVoltageV * batteryModel.nominalVoltageScale;
    return { E: ocv, r: p.internalResistanceOhm, ocv };
  };
  const currentAt = (q: number): number => {
    const soc = soc0 - q / qCap;
    if (!(soc > 0)) throw new Error("battery-drain: the cell empties inside the horizon; this sidecar does not model the clamp");
    const { E, r } = point(soc);
    return (E * G) / (1 + r * G);
  };

  const integrate = (h: number): { t: number[]; q: number[] } => {
    const n = Math.ceil(horizonS / h);
    const step = horizonS / n;
    const t = [0];
    const q = [0];
    let Q = 0;
    for (let i = 0; i < n; i++) {
      const k1 = currentAt(Q);
      const k2 = currentAt(Q + (step / 2) * k1);
      const k3 = currentAt(Q + (step / 2) * k2);
      const k4 = currentAt(Q + step * k3);
      Q += (step / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
      t.push((i + 1) * step);
      q.push(Q);
    }
    return { t, q };
  };
  const h = horizonS / 4000;
  const coarse = integrate(h);
  const fine = integrate(h / 2);
  coarse.q.forEach((q, i) => {
    const qf = fine.q[2 * i]!;
    if (Math.abs(q - qf) > 1e-12 * Math.max(Math.abs(qf), 1e-30) + 1e-18) {
      throw new Error(`battery-drain: RK4 at h and h/2 disagree at t=${coarse.t[i]} (${q} against ${qf})`);
    }
  });

  // Between the fine table's points the charge is carried by one RK4 step from the point before.
  const fineStep = fine.t[1]! - fine.t[0]!;
  const qAt = (t: number): number => {
    if (t < 0 || t > horizonS * (1 + 1e-12)) throw new Error(`battery-drain: t=${t} is outside [0, ${horizonS}]`);
    const i = Math.min(Math.floor(t / fineStep), fine.q.length - 1);
    const dt = t - fine.t[i]!;
    const Q = fine.q[i]!;
    if (dt === 0) return Q;
    const k1 = currentAt(Q);
    const k2 = currentAt(Q + (dt / 2) * k1);
    const k3 = currentAt(Q + (dt / 2) * k2);
    const k4 = currentAt(Q + dt * k3);
    return Q + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
  };

  return {
    rail: netIdOf(built, pos),
    capacityCoulombs: qCap,
    at: (t) => {
      const q = qAt(t);
      const soc = soc0 - q / qCap;
      const { E, r, ocv } = point(soc);
      const current = (E * G) / (1 + r * G);
      return { soc, dischargedCoulombs: q, openCircuitVoltageV: ocv, currentA: current, railV: E / (1 + r * G) };
    },
    // The current only falls as the cell drains, so Q/t is the mean |I|.
    meanCurrentA: (t) => (t > 0 ? qAt(t) / t : currentAt(0)),
  };
}

/**
 * The README's integrated tolerance for `dischargedCoulombs`, `(1 nA + 1e-3
 * |I|avg) T` (Equivalence harness rules, battery drain). SoC is `soc0 -
 * Q/Qcap` exactly, so its bound is the same one divided by `Qcap`.
 */
export function drainCoulombTolerance(meanCurrentA: number, tS: number): number {
  return (1e-9 + 1e-3 * Math.abs(meanCurrentA)) * tS;
}

// ─── ramp-crossings ──────────────────────────────────────────────────────────

export interface RampCrossing {
  /** When the output key takes `value`: the input's crossing plus the propagation delay. */
  readonly t: number;
  readonly value: number;
  /** The input's own crossing instant. */
  readonly crossingS: number;
}

/**
 * A Schmitt inverter input (a part whose catalog `v_input` carries `v_t_plus`
 * and `v_t_minus`, scaled by `vSupply / vcc_range.nominal` as classic scales
 * them) on a pulse source's node, with an ideal supply of `vSupplyV`. On a
 * rising piece the input crosses `V_T+` at `t0 + (V_T+ - v0)/m` and the
 * output goes LOW; on a falling piece it crosses `V_T-` and the output goes
 * HIGH. Each output change lands `propagationDelayForComponent` later.
 */
export function rampCrossings(circuit: SimCircuit, model: RampCrossingsModel, vSupplyV: number, horizonS: number): RampCrossing[] {
  const src = componentOf(circuit, model.source, ["pulse_source", "pulse_gen"]);
  const ic = circuit.components.find((c) => c.id === model.ic);
  if (!ic) throw new Error(`ramp-crossings: no component "${model.ic}"`);
  const part = resolveCatalogPart({ kind: ic.kind as ComponentKind, params: ic.params, catalogUid: ic.catalogUid });
  const vi = part?.electrical_specs?.v_input;
  if (vi?.v_t_plus == null || vi.v_t_minus == null) throw new Error(`ramp-crossings: ${ic.id} has no Schmitt thresholds in its catalog entry`);
  const nominal = part?.electrical_specs?.vcc_range?.nominal ?? 5;
  const scale = nominal > 0 ? vSupplyV / nominal : 1;
  const vtPlus = vi.v_t_plus * scale;
  const vtMinus = vi.v_t_minus * scale;
  const delay = propagationDelayForComponent(ic);
  const out: RampCrossing[] = [];
  let previous: LinearPiece | undefined;
  for (const piece of pulseModelPieces(pulseParamsOf(src), horizonS)) {
    // A midpoint step crosses at its own instant.
    if (previous && previous.v1 !== piece.v0) {
      if (previous.v1 < vtPlus && piece.v0 >= vtPlus) out.push({ t: piece.t0 + delay, value: 0, crossingS: piece.t0 });
      else if (previous.v1 > vtMinus && piece.v0 <= vtMinus) out.push({ t: piece.t0 + delay, value: 1, crossingS: piece.t0 });
    }
    previous = piece;
    const m = (piece.v1 - piece.v0) / (piece.t1 - piece.t0);
    if (m > 0 && piece.v0 < vtPlus && piece.v1 >= vtPlus) {
      const crossingS = piece.t0 + (vtPlus - piece.v0) / m;
      out.push({ t: crossingS + delay, value: 0, crossingS });
    } else if (m < 0 && piece.v0 > vtMinus && piece.v1 <= vtMinus) {
      const crossingS = piece.t0 + (vtMinus - piece.v0) / m;
      out.push({ t: crossingS + delay, value: 1, crossingS });
    }
  }
  return out.filter((c) => c.t <= horizonS);
}

// ─── psu-rail-gate ───────────────────────────────────────────────────────────

export interface RailGateFlip {
  readonly t: number;
  readonly powered: boolean;
}

export interface RailGateSidecar {
  readonly flips: readonly RailGateFlip[];
  readonly railAt: (t: number) => number;
  /** The power window's lower edge, `vccMin * 0.9` (W1c point 1). */
  readonly gateLowV: number;
}

/**
 * A `bench_psu` (`V`, `iLimit`) with a rail capacitor (`esr` 0) and a
 * resistor `R` from the rail to a pulse source's node `s(t)`, feeding an IC
 * whose `vcc` is the rail and whose `gnd` is the datum. The IC's outputs are
 * left open, so its draw on the rail is its outputs' node shunts (picoamps
 * against `iLimit`), which this generator leaves out. The rail then obeys
 * Rails points 4 and 5 exactly:
 * - CC: `C u' = iLimit - (u - s)/R - g u`, linear with piecewise-linear
 *   forcing, so `u = A + B τ + (u0 - A) e^(-a τ)` on each piece of the
 *   model's pulse (`pulseModelPieces`); it leaves CC when `u` reaches `V`;
 * - CV: `u = V`; it enters CC when the draw at the setpoint, `(V - s)/R +
 *   g V`, exceeds `iLimit`, or at load, where the uncharged capacitor is
 *   demand.
 * The IC is powered while `vccMin*0.9 <= u <= vccMax*1.1` (W1c point 1), so
 * each crossing of an edge of that window is a power-gate flip.
 */
export function psuRailGate(circuit: SimCircuit, model: PsuRailGateModel, horizonS: number): RailGateSidecar {
  const built = buildCoreNodes(circuit);
  const psu = componentOf(circuit, model.psu, ["bench_psu"]);
  const cap = componentOf(circuit, model.capacitor, ["capacitor"]);
  const load = componentOf(circuit, model.load, ["resistor"]);
  const src = componentOf(circuit, model.source, ["pulse_source", "pulse_gen"]);
  const ic = circuit.components.find((c) => c.id === model.ic);
  if (!ic) throw new Error(`psu-rail-gate: no component "${model.ic}"`);
  const V = num(psu, "voltage", 5);
  const iLimit = num(psu, "iLimit", 0);
  if (!(V > 0) || !(iLimit > 0)) throw new Error("psu-rail-gate: needs a positive setpoint and a positive iLimit");
  const C = idealCapacitance(cap);
  const R = num(load, "resistance", Number.NaN);
  if (!(R > 0)) throw new Error(`psu-rail-gate: ${load.id} has no positive resistance`);

  const rail = nodeOf(built, psu.id, psu.pins[0]!.id);
  const sNode = nodeOf(built, src.id, src.pins[0]!.id);
  const loadNodes = [nodeOf(built, load.id, load.pins[0]!.id), nodeOf(built, load.id, load.pins[1]!.id)];
  if (
    nodeOf(built, psu.id, psu.pins[1]!.id) !== 0 ||
    nodeOf(built, src.id, src.pins[1]!.id) !== 0 ||
    nodeOf(built, cap.id, cap.pins[0]!.id) !== rail ||
    nodeOf(built, cap.id, cap.pins[1]!.id) !== 0 ||
    !(loadNodes.includes(rail) && loadNodes.includes(sNode)) ||
    nodeOf(built, ic.id, "vcc") !== rail ||
    nodeOf(built, ic.id, "gnd") !== 0
  ) {
    throw new Error("psu-rail-gate: expected the PSU, the capacitor and the IC's supply on the rail, the load from the rail to the source's pos, and every return on the datum");
  }
  const part = resolveCatalogPart({ kind: ic.kind as ComponentKind, params: ic.params, catalogUid: ic.catalogUid });
  const vccMin = part?.electrical_specs?.vcc_range?.min ?? 2.0;
  const vccMax = part?.electrical_specs?.vcc_range?.max ?? 18.0;
  const gateLow = vccMin * 0.9;
  const gateHigh = vccMax * 1.1;
  if (!(V < gateHigh)) throw new Error("psu-rail-gate: the setpoint is above the window; only the lower edge is modelled");

  const g = NODE_SHUNT_SIEMENS;
  const a = (1 / R + g) / C;
  const pieces = pulseModelPieces(pulseParamsOf(src), horizonS);

  // Each CC stretch is a closed form from (t0, u0) on one piece.
  interface Seg { readonly t0: number; readonly t1: number; readonly cc: boolean; readonly u0: number; readonly A: number; readonly B: number }
  const segs: Seg[] = [];
  const flips: RailGateFlip[] = [];
  const ccSeg = (piece: LinearPiece, t0: number, u0: number): Seg => {
    const m = (piece.v1 - piece.v0) / (piece.t1 - piece.t0);
    const s0 = piece.v0 + m * (t0 - piece.t0);
    const B = m / (R * C) / a;
    const A = ((iLimit + s0 / R) / C - B) / a;
    return { t0, t1: piece.t1, cc: true, u0, A, B };
  };
  const uOf = (seg: Seg, t: number): number => (seg.cc ? seg.A + seg.B * (t - seg.t0) + (seg.u0 - seg.A) * Math.exp(-a * (t - seg.t0)) : V);
  // The first t in (t0, t1] where f changes sign, to the last bit by bisection; f is a line plus one exponential, so at most two roots, both bracketed by a 256-point scan.
  const firstRoot = (f: (t: number) => number, t0: number, t1: number): number | undefined => {
    const n = 256;
    let prevT = t0;
    let prevF = f(t0);
    for (let i = 1; i <= n; i++) {
      const t = i === n ? t1 : t0 + ((t1 - t0) * i) / n;
      const ft = f(t);
      if ((prevF < 0 && ft >= 0) || (prevF > 0 && ft <= 0)) {
        let lo = prevT;
        let hi = t;
        for (let k = 0; k < 200 && hi - lo > Number.EPSILON * Math.max(1, Math.abs(hi)); k++) {
          const mid = (lo + hi) / 2;
          const fm = f(mid);
          if ((prevF < 0 && fm >= 0) || (prevF > 0 && fm <= 0)) hi = mid;
          else lo = mid;
        }
        return hi;
      }
      prevT = t;
      prevF = ft;
    }
    return undefined;
  };

  let cc = true; // the uncharged rail capacitor is demand at load (Rails point 5)
  let u = 0;
  let powered = u >= gateLow && u <= gateHigh;
  for (const piece of pieces) {
    let t = piece.t0;
    while (t < piece.t1 && t < horizonS) {
      const end = Math.min(piece.t1, horizonS);
      if (!cc) {
        const drawAt = (x: number): number => (V - pieceValueAt(piece, x)) / R + g * V - iLimit;
        if (drawAt(t) > 0) {
          cc = true;
          continue;
        }
        const entry = firstRoot(drawAt, t, end);
        const stop = entry ?? end;
        segs.push({ t0: t, t1: stop, cc: false, u0: V, A: V, B: 0 });
        if (entry === undefined) break;
        t = entry;
        cc = true;
        continue;
      }
      const seg = ccSeg(piece, t, u);
      const exit = firstRoot((x) => uOf(seg, x) - V, t, end);
      const gate = firstRoot((x) => uOf(seg, x) - gateLow, t, end);
      const next = Math.min(exit ?? Number.POSITIVE_INFINITY, gate ?? Number.POSITIVE_INFINITY, end);
      segs.push({ ...seg, t1: next });
      if (next === exit) {
        cc = false;
        u = V;
      } else {
        // `firstRoot` returns the far side of a root, so restarting the same closed form there cannot find it again.
        u = uOf(seg, next);
        if (next === gate) {
          powered = !powered;
          flips.push({ t: next, powered });
        }
      }
      t = next;
    }
  }
  const railAt = (x: number): number => {
    const seg = segs.find((sg) => x >= sg.t0 && x <= sg.t1);
    if (!seg) throw new Error(`psu-rail-gate: t=${x} is outside the computed rail`);
    return uOf(seg, x);
  };
  return { flips, railAt, gateLowV: gateLow };
}
