/**
 * MNA stamp functions for each element type.
 *
 * Convention: i, j = node indices in the MNA matrix. -1 = ground (excluded
 * from matrix). k = extra-variable row for voltage sources.
 *
 * Capacitor / Inductor use backward-Euler companion models with optional
 * lumped loss terms:
 *   Cap branch: series ESR + ideal C, with terminal-parallel leakage R.
 *   Ind branch: series winding DCR + ideal L, with terminal-parallel core R.
 * Absent loss parameters preserve the historical ideal companions exactly.
 *
 * The *Trap variants are opt-in trapezoidal companions (engine
 * integrationMethod "trap"). They are additive: the backward-Euler functions
 * above them remain the default and are never altered, so default-mode
 * trajectories stay byte-identical.
 */

import type { MnaStampSurface } from "./linear-system.js";

export function stampResistor(mna: MnaStampSurface, i: number, j: number, R: number): void {
  const g = 1 / R;
  if (i >= 0) mna.add(i, i, g);
  if (j >= 0) mna.add(j, j, g);
  if (i >= 0 && j >= 0) {
    mna.add(i, j, -g);
    mna.add(j, i, -g);
  }
}

function stampConductance(mna: MnaStampSurface, i: number, j: number, g: number): void {
  if (!(g > 0) || !Number.isFinite(g)) return;
  if (i >= 0) mna.add(i, i, g);
  if (j >= 0) mna.add(j, j, g);
  if (i >= 0 && j >= 0) {
    mna.add(i, j, -g);
    mna.add(j, i, -g);
  }
}

/**
 * Voltage source: k is the extra-variable row allocated by the caller.
 * Positive current flows i → j through the source.
 */
export function stampVSource(
  mna: MnaStampSurface,
  i: number,
  j: number,
  k: number,
  V: number,
): void {
  if (i >= 0) {
    mna.add(i, k, 1);
    mna.add(k, i, 1);
  }
  if (j >= 0) {
    mna.add(j, k, -1);
    mna.add(k, j, -1);
  }
  mna.addB(k, V);
}

/**
 * Voltage source with series resistance — Thevenin equivalent.
 *
 * When rSeries = 0 this is byte-identical to stampVSource.
 *
 * Physical terminal voltage:
 *   V_term = V_i − V_j = Voc − rSeries × I_delivered
 *
 * MNA sign note: in the standard vsource stamp, mna.add(i, k, +1) means x[k]
 * is the current ENTERING the source branch from node i.  When the source
 * delivers current into the external circuit, x[k] < 0 (x[k] = −I_delivered).
 * Therefore the KVL row that produces the correct sag is:
 *   V_i − V_j − rSeries × x[k] = Voc   →   G[k][k] = −rSeries
 *
 * x[k] is still the branch current variable, so _updateElementI is unchanged.
 *
 * Example: Voc = 9, rSeries = 1.5, R_load = 10 →
 *   I_delivered = 9 / (1.5 + 10) = 0.783 A
 *   V_term = 9 − 1.5 × 0.783 = 7.826 V
 */
export function stampVSourceSeriesR(
  mna: MnaStampSurface,
  i: number,
  j: number,
  k: number,
  V: number,
  rSeries: number,
): void {
  // Standard voltage-source current coupling (all regimes).
  if (i >= 0) {
    mna.add(i, k, 1);
    mna.add(k, i, 1);
  }
  if (j >= 0) {
    mna.add(j, k, -1);
    mna.add(k, j, -1);
  }
  mna.addB(k, V);
  // Thevenin internal resistance: adds G[k][k] = −rSeries to the KVL row.
  //
  // MNA sign analysis for a delivering source (current flows pos → external → neg):
  //   The KCL coupling convention stamps mna.add(pos, k, +1), so x[k] is the
  //   current that would ENTER the source branch from the pos node — equivalently,
  //   x[k] = −I_delivered (negative when the source is delivering current).
  //
  // Physical Thevenin equation:
  //   V_term = V_pos − V_neg = Voc − rSeries × I_delivered
  //          = Voc + rSeries × x[k]   (because x[k] = −I_delivered)
  //
  // KVL row k (before the rSeries term) enforces:
  //   V_pos − V_neg = Voc   →   V_pos − V_neg − Voc = 0
  //
  // To get the Thevenin sag, we want:
  //   V_pos − V_neg − rSeries × x[k] = Voc
  //
  // Subtract rSeries×x[k] from both sides of the row:
  //   G[k][k] = −rSeries  (negative sign here)
  //
  // When rSeries = 0 this adds nothing → byte-identical to stampVSource.
  if (rSeries > 0) {
    mna.add(k, k, -rSeries);
  }
}

/**
 * Independent current source driving current I INTO node i and OUT of node j.
 *
 * Standard MNA: only the RHS vector is touched — the conductance matrix has no
 * off-diagonal entry for an ideal current source.  Sign convention follows the
 * capacitor/inductor companion sources: addB(i, +I) injects current at i,
 * addB(j, -I) removes it at j, consistent with KCL (current leaving a node is
 * positive in the conductance-form equation).
 *
 * Ground row (index -1) is excluded, mirroring stampResistor.
 */
export function stampCurrentSource(mna: MnaStampSurface, i: number, j: number, I: number): void {
  if (i >= 0) mna.addB(i, I);
  if (j >= 0) mna.addB(j, -I);
}

export interface CapacitorCompanion {
  /** Norton conductance of the series ESR + companion capacitor branch. */
  conductance: number;
  /**
   * G * history voltage from the previous accepted step: the internal
   * capacitor voltage for backward Euler, augmented with the (h/2C)·iPrev
   * term for the trapezoidal variant.
   */
  historyCurrent: number;
}

/**
 * Backward-Euler companion for an ideal capacitor C in series with ESR.
 *
 * Let v be the terminal voltage, vcPrev the prior internal capacitor voltage,
 * and i positive from pin i to pin j:
 *   i = C/h * (vc - vcPrev)
 *   v = vc + ESR*i
 * Therefore:
 *   i = G*(v - vcPrev), where G = 1 / (ESR + h/C)
 */
export function capacitorCompanion(
  C: number,
  h: number,
  vcPrev: number,
  esr = 0,
): CapacitorCompanion {
  const safeC = Math.max(1e-30, C);
  const safeH = Math.max(1e-30, h);
  const safeEsr = Number.isFinite(esr) ? Math.max(0, esr) : 0;
  const conductance = 1 / (safeEsr + safeH / safeC);
  return { conductance, historyCurrent: conductance * vcPrev };
}

/** Series-branch current through ESR and C, positive i→j. */
export function capacitorSeriesCurrent(
  terminalVoltage: number,
  C: number,
  h: number,
  vcPrev: number,
  esr = 0,
): number {
  const companion = capacitorCompanion(C, h, vcPrev, esr);
  return companion.conductance * terminalVoltage - companion.historyCurrent;
}

/**
 * Internal capacitor voltage after the accepted backward-Euler step.
 *
 * Advancing vc from i=C·dvc/dt avoids subtracting two nearly equal voltages
 * when ESR is large compared with h/C. It is algebraically identical to
 * vc=terminalVoltage-ESR·i for the solved companion branch.
 */
export function capacitorInternalVoltage(
  vcPrev: number,
  seriesCurrent: number,
  C: number,
  h: number,
): number {
  const safeC = Math.max(1e-30, C);
  const safeH = Math.max(1e-30, h);
  return vcPrev + (safeH / safeC) * seriesCurrent;
}

/** Current through an optional terminal-parallel loss resistance. */
export function parallelLossCurrent(voltage: number, resistance: number): number {
  return Number.isFinite(resistance) && resistance > 0 ? voltage / resistance : 0;
}

/**
 * Capacitor companion (backward Euler).
 * vcPrev is the internal ideal-capacitor voltage from the prior accepted step.
 * leakageResistance is a physical resistor in parallel with the full terminals.
 */
export function stampCapacitor(
  mna: MnaStampSurface,
  i: number,
  j: number,
  C: number,
  h: number,
  vcPrev: number,
  esr = 0,
  leakageResistance = Number.POSITIVE_INFINITY,
): void {
  const companion = capacitorCompanion(C, h, vcPrev, esr);
  stampConductance(mna, i, j, companion.conductance);
  if (i >= 0) mna.addB(i, companion.historyCurrent);
  if (j >= 0) mna.addB(j, -companion.historyCurrent);
  if (Number.isFinite(leakageResistance) && leakageResistance > 0) {
    stampConductance(mna, i, j, 1 / leakageResistance);
  }
}

/**
 * Trapezoidal companion for an ideal capacitor C in series with ESR.
 *
 * Trapezoidal rule on i = C·dvc/dt averages the endpoint currents:
 *   vc_new = vcPrev + (h/(2C))·(i_new + iPrev)
 * Substituting the terminal law v = vc + ESR·i and solving for i_new:
 *   v = vcPrev + (h/(2C))·iPrev + (ESR + h/(2C))·i_new
 *   i_new = G·(v - vcPrev - (h/(2C))·iPrev), G = 1/(ESR + h/(2C))
 * This is exactly the backward-Euler companion with h replaced by h/2 and the
 * history voltage augmented to vcPrev + (h/(2C))·iPrev; the extra term carries
 * the previous accepted branch current that gives second-order accuracy.
 *
 * iPrev must be the series-branch current committed at the last ACCEPTED step
 * (never a Newton iterate), or step-doubling rollback would not be replayable.
 */
export function capacitorCompanionTrap(
  C: number,
  h: number,
  vcPrev: number,
  iPrev: number,
  esr = 0,
): CapacitorCompanion {
  const safeC = Math.max(1e-30, C);
  const safeH = Math.max(1e-30, h);
  const safeEsr = Number.isFinite(esr) ? Math.max(0, esr) : 0;
  const halfStepOverC = safeH / (2 * safeC);
  const conductance = 1 / (safeEsr + halfStepOverC);
  return {
    conductance,
    historyCurrent: conductance * (vcPrev + halfStepOverC * iPrev),
  };
}

/** Series-branch current for the trapezoidal companion, positive i→j. */
export function capacitorSeriesCurrentTrap(
  terminalVoltage: number,
  C: number,
  h: number,
  vcPrev: number,
  iPrev: number,
  esr = 0,
): number {
  const companion = capacitorCompanionTrap(C, h, vcPrev, iPrev, esr);
  return companion.conductance * terminalVoltage - companion.historyCurrent;
}

/**
 * Internal capacitor voltage after an accepted trapezoidal step:
 *   vc_new = vcPrev + (h/(2C))·(iNew + iPrev)
 * Advancing vc from the integrated current mirrors capacitorInternalVoltage's
 * rationale: it avoids subtracting nearly equal terminal/ESR voltages and is
 * algebraically identical to vc = terminalVoltage - ESR·iNew for the solved
 * companion branch.
 */
export function capacitorInternalVoltageTrap(
  vcPrev: number,
  iPrev: number,
  iNew: number,
  C: number,
  h: number,
): number {
  const safeC = Math.max(1e-30, C);
  const safeH = Math.max(1e-30, h);
  return vcPrev + (safeH / (2 * safeC)) * (iNew + iPrev);
}

/**
 * Capacitor companion (trapezoidal). Identical stamp topology to
 * stampCapacitor; only the companion coefficients differ. The leakage
 * resistance stays a plain terminal shunt — it is resistive, so it has no
 * integration-order dependence.
 */
export function stampCapacitorTrap(
  mna: MnaStampSurface,
  i: number,
  j: number,
  C: number,
  h: number,
  vcPrev: number,
  iPrev: number,
  esr = 0,
  leakageResistance = Number.POSITIVE_INFINITY,
): void {
  const companion = capacitorCompanionTrap(C, h, vcPrev, iPrev, esr);
  stampConductance(mna, i, j, companion.conductance);
  if (i >= 0) mna.addB(i, companion.historyCurrent);
  if (j >= 0) mna.addB(j, -companion.historyCurrent);
  if (Number.isFinite(leakageResistance) && leakageResistance > 0) {
    stampConductance(mna, i, j, 1 / leakageResistance);
  }
}

export interface InductorCompanion {
  /** Norton conductance of the series DCR + companion inductor branch. */
  conductance: number;
  /**
   * History current flowing i→j: the decayed previous winding current for
   * backward Euler, plus the averaged previous terminal voltage term for the
   * trapezoidal variant.
   */
  historyCurrent: number;
}

/**
 * Backward-Euler companion for ideal L in series with winding DCR:
 *   v = L/h * (i - iPrev) + DCR*i
 *   i = G*v + I_hist
 *   G = h / (L + DCR*h)
 *   I_hist = L/(L + DCR*h) * iPrev
 */
export function inductorCompanion(
  L: number,
  h: number,
  iPrev: number,
  dcr = 0,
): InductorCompanion {
  const safeL = Math.max(1e-30, L);
  const safeH = Math.max(1e-30, h);
  const safeDcr = Number.isFinite(dcr) ? Math.max(0, dcr) : 0;
  const denominator = safeL + safeDcr * safeH;
  return {
    conductance: safeH / denominator,
    historyCurrent: (safeL / denominator) * iPrev,
  };
}

/** Winding-branch current through DCR and L, positive i→j. */
export function inductorWindingCurrent(
  terminalVoltage: number,
  L: number,
  h: number,
  iPrev: number,
  dcr = 0,
): number {
  const companion = inductorCompanion(L, h, iPrev, dcr);
  return companion.conductance * terminalVoltage + companion.historyCurrent;
}

/**
 * Inductor companion (backward Euler). coreLossResistance is a declared
 * terminal-parallel approximation; saturation and frequency dependence remain
 * outside this lumped model.
 */
export function stampInductor(
  mna: MnaStampSurface,
  i: number,
  j: number,
  L: number,
  h: number,
  iPrev: number,
  dcr = 0,
  coreLossResistance = Number.POSITIVE_INFINITY,
): void {
  const companion = inductorCompanion(L, h, iPrev, dcr);
  stampConductance(mna, i, j, companion.conductance);
  if (i >= 0) mna.addB(i, -companion.historyCurrent);
  if (j >= 0) mna.addB(j, companion.historyCurrent);
  if (Number.isFinite(coreLossResistance) && coreLossResistance > 0) {
    stampConductance(mna, i, j, 1 / coreLossResistance);
  }
}

/**
 * Trapezoidal companion for ideal L in series with winding DCR.
 *
 * Trapezoidal rule on v = L·di/dt + DCR·i averages both endpoint voltages:
 *   (i_new - iPrev)/h = (1/(2L))·((v_new - DCR·i_new) + (vPrev - DCR·iPrev))
 * Collecting i_new terms and dividing by h/L:
 *   i_new = [(L/h - DCR/2)·iPrev + (v_new + vPrev)/2] / (L/h + DCR/2)
 * Stamped as a Norton pair:
 *   G = (1/2)/(L/h + DCR/2) = h/(2L + DCR·h)
 *   I_hist = [(L/h - DCR/2)·iPrev + vPrev/2] / (L/h + DCR/2)
 *
 * vPrev is the terminal voltage of the WINDING branch (series DCR + L) at the
 * last ACCEPTED step. The optional core-loss resistor is a separate terminal
 * shunt, exactly as in the backward-Euler model, so the same terminal voltage
 * serves both and no loss-current subtraction is needed.
 */
export function inductorCompanionTrap(
  L: number,
  h: number,
  iPrev: number,
  vPrev: number,
  dcr = 0,
): InductorCompanion {
  const safeL = Math.max(1e-30, L);
  const safeH = Math.max(1e-30, h);
  const safeDcr = Number.isFinite(dcr) ? Math.max(0, dcr) : 0;
  const lOverH = safeL / safeH;
  const denominator = lOverH + safeDcr / 2;
  return {
    conductance: 0.5 / denominator,
    historyCurrent: ((lOverH - safeDcr / 2) * iPrev + vPrev / 2) / denominator,
  };
}

/** Winding-branch current for the trapezoidal companion, positive i→j. */
export function inductorWindingCurrentTrap(
  terminalVoltage: number,
  L: number,
  h: number,
  iPrev: number,
  vPrev: number,
  dcr = 0,
): number {
  const companion = inductorCompanionTrap(L, h, iPrev, vPrev, dcr);
  return companion.conductance * terminalVoltage + companion.historyCurrent;
}

/**
 * Inductor companion (trapezoidal). Identical stamp topology to stampInductor;
 * only the companion coefficients differ. coreLossResistance remains the same
 * declared terminal-parallel approximation.
 */
export function stampInductorTrap(
  mna: MnaStampSurface,
  i: number,
  j: number,
  L: number,
  h: number,
  iPrev: number,
  vPrev: number,
  dcr = 0,
  coreLossResistance = Number.POSITIVE_INFINITY,
): void {
  const companion = inductorCompanionTrap(L, h, iPrev, vPrev, dcr);
  stampConductance(mna, i, j, companion.conductance);
  if (i >= 0) mna.addB(i, -companion.historyCurrent);
  if (j >= 0) mna.addB(j, companion.historyCurrent);
  if (Number.isFinite(coreLossResistance) && coreLossResistance > 0) {
    stampConductance(mna, i, j, 1 / coreLossResistance);
  }
}

// ── Wave A6: coupled inductors (transformer) ────────────────────────────────

export interface CoupledInductorCompanion {
  /** Self conductance of winding 1 in the inverted 2x2 block. */
  g11: number;
  /** Mutual conductance (symmetric: G12 = G21). */
  g12: number;
  /** Self conductance of winding 2. */
  g22: number;
  /** Norton history current of winding 1, flowing a1 -> b1. */
  ih1: number;
  /** Norton history current of winding 2, flowing a2 -> b2. */
  ih2: number;
}

/**
 * Backward-Euler companion for two magnetically coupled windings with
 * series resistances (transformer / coupled inductor).
 *
 * Continuous equations (dot convention: a1 and a2 are the dotted terminals,
 * so positive i1 into a1 and positive i2 into a2 both magnetize the core in
 * the same sense; v_w = v(a_w) - v(b_w), i_w positive a_w -> b_w):
 *
 *   v1 = L1·di1/dt + M·di2/dt + DCR1·i1
 *   v2 = M·di1/dt + L2·di2/dt + DCR2·i2,   M = k·sqrt(L1·L2)
 *
 * Backward Euler (di/dt -> (i - iPrev)/h) turns both into one linear system
 * A·[i1, i2]^T = [v1, v2]^T + c with
 *
 *   A = | L1/h + DCR1   M/h         |      c = | (L1·i1Prev + M·i2Prev)/h |
 *       | M/h           L2/h + DCR2 |          | (M·i1Prev + L2·i2Prev)/h |
 *
 * Inverting the symmetric 2x2 symbolically (det = a11·a22 - a12²) gives the
 * Norton form the MNA wants:
 *
 *   [i1, i2]^T = G·[v1, v2]^T + Ihist,   G = A⁻¹,  Ihist = A⁻¹·c
 *   G11 = a22/det, G22 = a11/det, G12 = G21 = -a12/det
 *
 * so each winding stamps a self conductance across its own node pair, a
 * cross conductance block against the other pair, and a history current
 * (positive a -> b, exactly the single-inductor companion's convention —
 * at k = 0 this degenerates to two independent inductorCompanion() results
 * up to rounding of the shared division).
 *
 * Singularity: det -> 0 when k -> 1 with zero winding resistance
 * (L1·L2 - M² = L1·L2·(1 - k²) vanishes and an ideal transformer has no
 * admittance representation). Callers must clamp k below 1; the device
 * model uses 0.9999, which keeps the leakage inductance ~2e-4·L — far
 * above f64 noise at every circuit scale while staying visually "ideal".
 */
export function coupledInductorCompanion(
  l1: number,
  l2: number,
  m: number,
  h: number,
  i1Prev: number,
  i2Prev: number,
  dcr1 = 0,
  dcr2 = 0,
): CoupledInductorCompanion {
  const safeL1 = Math.max(1e-12, l1);
  const safeL2 = Math.max(1e-12, l2);
  const safeH = Math.max(1e-30, h);
  const safeDcr1 = Number.isFinite(dcr1) ? Math.max(0, dcr1) : 0;
  const safeDcr2 = Number.isFinite(dcr2) ? Math.max(0, dcr2) : 0;
  const a11 = safeL1 / safeH + safeDcr1;
  const a22 = safeL2 / safeH + safeDcr2;
  const a12 = m / safeH;
  const det = a11 * a22 - a12 * a12;
  const g11 = a22 / det;
  const g22 = a11 / det;
  const g12 = -a12 / det;
  const c1 = (safeL1 * i1Prev + m * i2Prev) / safeH;
  const c2 = (m * i1Prev + safeL2 * i2Prev) / safeH;
  return {
    g11,
    g12,
    g22,
    ih1: g11 * c1 + g12 * c2,
    ih2: g12 * c1 + g22 * c2,
  };
}

/**
 * Trapezoidal companion for the coupled windings. Averaging both endpoint
 * voltages of each continuous equation (the same step inductorCompanionTrap
 * documents for one winding):
 *
 *   (v1 + v1Prev)/2 = L1·(i1 - i1Prev)/h + M·(i2 - i2Prev)/h
 *                     + DCR1·(i1 + i1Prev)/2
 *
 * and likewise for winding 2. Collecting new-step terms:
 *
 *   v1 = (2L1/h + DCR1)·i1 + (2M/h)·i2
 *        - [(2L1/h - DCR1)·i1Prev + (2M/h)·i2Prev + v1Prev]
 *
 * i.e. the SAME 2x2 inversion as backward Euler with
 *
 *   A_trap = | 2L1/h + DCR1   2M/h          |
 *            | 2M/h           2L2/h + DCR2  |
 *
 *   c_trap = | (2L1/h - DCR1)·i1Prev + (2M/h)·i2Prev + v1Prev |
 *            | (2M/h)·i1Prev + (2L2/h - DCR2)·i2Prev + v2Prev |
 *
 * At k = 0 each winding reduces exactly to inductorCompanionTrap's algebra
 * (conductance 1/(2L/h + DCR), history ((2L/h - DCR)·iPrev + vPrev)
 * / (2L/h + DCR)). vPrev terms are the winding terminal voltages of the
 * last ACCEPTED step (never a Newton iterate), the same history contract
 * as the single-inductor trap companion.
 */
export function coupledInductorCompanionTrap(
  l1: number,
  l2: number,
  m: number,
  h: number,
  i1Prev: number,
  i2Prev: number,
  v1Prev: number,
  v2Prev: number,
  dcr1 = 0,
  dcr2 = 0,
): CoupledInductorCompanion {
  const safeL1 = Math.max(1e-12, l1);
  const safeL2 = Math.max(1e-12, l2);
  const safeH = Math.max(1e-30, h);
  const safeDcr1 = Number.isFinite(dcr1) ? Math.max(0, dcr1) : 0;
  const safeDcr2 = Number.isFinite(dcr2) ? Math.max(0, dcr2) : 0;
  const twoOverH = 2 / safeH;
  const a11 = safeL1 * twoOverH + safeDcr1;
  const a22 = safeL2 * twoOverH + safeDcr2;
  const a12 = m * twoOverH;
  const det = a11 * a22 - a12 * a12;
  const g11 = a22 / det;
  const g22 = a11 / det;
  const g12 = -a12 / det;
  const c1 = (safeL1 * twoOverH - safeDcr1) * i1Prev + a12 * i2Prev + v1Prev;
  const c2 = a12 * i1Prev + (safeL2 * twoOverH - safeDcr2) * i2Prev + v2Prev;
  return {
    g11,
    g12,
    g22,
    ih1: g11 * c1 + g12 * c2,
    ih2: g12 * c1 + g22 * c2,
  };
}

/**
 * Stamp the coupled-winding companion between the two node pairs. Self
 * blocks mirror stampConductance's pattern per pair; the cross block writes
 * the full 2x2 incidence (row pair 1 x column pair 2 and its transpose,
 * G12 = G21). History currents flow a -> b in each winding, so the RHS
 * signs match stampInductor's (-Ihist at a, +Ihist at b).
 */
export function stampCoupledInductor(
  mna: MnaStampSurface,
  a1: number,
  b1: number,
  a2: number,
  b2: number,
  companion: CoupledInductorCompanion,
): void {
  const { g11, g12, g22, ih1, ih2 } = companion;
  stampConductance(mna, a1, b1, g11);
  stampConductance(mna, a2, b2, g22);
  // Cross-coupling block: i1 gains +G12·(v_a2 - v_b2) and i2 gains
  // +G12·(v_a1 - v_b1). stampConductance cannot express an off-pair block,
  // so the eight entries are written explicitly with the usual ground skip.
  const add = (row: number, col: number, value: number): void => {
    if (row >= 0 && col >= 0) mna.add(row, col, value);
  };
  add(a1, a2, g12); add(a1, b2, -g12);
  add(b1, a2, -g12); add(b1, b2, g12);
  add(a2, a1, g12); add(a2, b1, -g12);
  add(b2, a1, -g12); add(b2, b1, g12);
  if (a1 >= 0) mna.addB(a1, -ih1);
  if (b1 >= 0) mna.addB(b1, ih1);
  if (a2 >= 0) mna.addB(a2, -ih2);
  if (b2 >= 0) mna.addB(b2, ih2);
}

/**
 * New-step winding currents of the coupled companion at the solved terminal
 * voltages — the commit-side counterpart of stampCoupledInductor, mirroring
 * inductorWindingCurrent's role for the single coil.
 */
export function coupledInductorWindingCurrents(
  companion: CoupledInductorCompanion,
  v1: number,
  v2: number,
): { i1: number; i2: number } {
  return {
    i1: companion.g11 * v1 + companion.g12 * v2 + companion.ih1,
    i2: companion.g12 * v1 + companion.g22 * v2 + companion.ih2,
  };
}

/**
 * Shockley diode / LED — companion model for Newton-Raphson (R2).
 *
 *   i_d = Is · (exp(v_d / (n·Vt)) - 1)
 *   g_d = di/dv = (i_d + Is) / (n·Vt)
 *
 * Linearise about the last-iteration voltage `vPrev`. Using the
 * tangent line at `vPrev`:
 *
 *   i_d(v) ≈ g_d · v + (i_d - g_d · vPrev)
 *          = g_d · v + I_eq
 *
 * Stamp as a Norton equivalent: conductance `g_d` between i and j,
 * plus a current source `I_eq` flowing from i to j (i.e. pulled from
 * node i and pushed into node j in the KCL sense).
 *
 * Returns the scalar current `i_d` at `vPrev` so the caller can record
 * element current for reporting. Callers that don't need it can ignore
 * the return value.
 *
 * Numerical safety: this stamp does a one-shot `vSat` cap on `vPrev` so
 * `Math.exp` cannot overflow. Standalone diodes/LEDs in this engine are
 * current-driven (a series resistor or follow-on conductance always
 * bounds the loop) and are well-conditioned without inter-iteration
 * damping, so we deliberately skip SPICE `pnjlim` here. The BJT stamper
 * does apply `pnjlim` upstream in `SimEngine._stampAll`.
 */
export function stampDiodeShockley(
  mna: MnaStampSurface,
  i: number,
  j: number,
  vPrev: number,
  Is = 1e-14,
  n = 1.0,
  Vt = 0.02585,
  vSatOverride?: number,
): number {
  const VtN = n * Vt;

  // Hard exp-overflow guard. If the voltage is way past any physical
  // regime we'd hit exp ≈ 1e300 and explode the matrix. Cap it.
  // vSatOverride lets callers raise the cap for high-Vf diodes (e.g. blue
  // LED Vf=3V) where 40·VtN ≈ 2.07V would prevent the Newton iteration
  // from converging past the forward-voltage knee.
  const vSat = vSatOverride ?? (40 * VtN);
  const v = vPrev > vSat ? vSat : vPrev;
  const e = Math.exp(v / VtN);
  const gd = (Is * e) / VtN; // = (id + Is) / VtN
  const id = shockleyDiodeCurrent(vPrev, Is, n, Vt, vSat);

  // Minimum conductance across the diode — without this, a deeply
  // reverse-biased diode has g_d ≈ 0 and creates a floating node.
  const GMIN_D = 1e-12;
  const g = gd + GMIN_D;

  // The exponential guard is a finite-slope tangent extension. Anchor the
  // physical companion at the actual guess so its solved branch current and
  // telemetry remain identical beyond vSat. GMIN is a zero-origin numerical
  // shunt and therefore contributes no Norton offset.
  const ieq = id - gd * vPrev;

  // Stamp conductance g between i, j
  if (i >= 0) mna.add(i, i, g);
  if (j >= 0) mna.add(j, j, g);
  if (i >= 0 && j >= 0) {
    mna.add(i, j, -g);
    mna.add(j, i, -g);
  }

  // Norton current source: I_eq flows i → j. In KCL at node i, the
  // diode current leaving i is g·(vi-vj) + I_eq; moving the constant
  // to the RHS gives b[i] -= I_eq, b[j] += I_eq.
  if (i >= 0) mna.addB(i, -ieq);
  if (j >= 0) mna.addB(j, ieq);

  return id;
}

/**
 * Compute Shockley parameters (Is) from a user-facing (Vf, iRated)
 * pair. `Vf` is the forward voltage at which the diode passes `iRated`
 * current; solving i_d(Vf)=iRated for Is gives:
 *
 *   Is = iRated · exp(-Vf / (n·Vt))
 *
 * Used by SimEngine to turn catalog Vf/iRated values into a Shockley
 * saturation current so the Newton stamp stays self-consistent with
 * whatever the user typed in the Inspector.
 */
export function shockleyIsFromVf(
  Vf: number,
  iRated = 0.02,
  n = 1.0,
  Vt = 0.02585,
): number {
  return iRated * Math.exp(-Vf / (n * Vt));
}

/**
 * Evaluate the same guarded Shockley law used by stampDiodeShockley.
 *
 * Above the exponential guard the junction continues along the boundary
 * tangent. Freezing current while retaining the guarded derivative would make
 * the MNA branch, telemetry, thermal stress, and failure diagnostics disagree
 * during exactly the overdrive faults where those values matter most.
 */
export function shockleyDiodeCurrent(
  voltage: number,
  Is = 1e-14,
  n = 1,
  Vt = 0.02585,
  vSatOverride?: number,
): number {
  const thermalScale = n * Vt;
  const vSat = vSatOverride ?? 40 * thermalScale;
  const clampedVoltage = Math.min(voltage, vSat);
  const exponential = Math.exp(clampedVoltage / thermalScale);
  const currentAtGuard = Is * (exponential - 1);
  const slopeAtGuard = (Is * exponential) / thermalScale;
  return currentAtGuard + slopeAtGuard * Math.max(0, voltage - vSat);
}

/**
 * NOTE on the reverse-breakdown prefactor: the breakdown term below is written
 * as an OVERDRIVE exponential, iRev = Is_z * exp(-(v + vz) / (nZ*Vt)), whose
 * exponent is ZERO at the knee (v = -vz). The knee condition |i(-vz)| = izKnee
 * therefore pins the prefactor directly: Is_z = izKnee. There is nothing to
 * solve — unlike the forward Shockley, where the exponent at the anchor point
 * is vf/(n*Vt) and shockleyIsFromVf back-computes Is. A previous revision
 * mirrored that derivation here (Is_z = izKnee * exp(-vz/(nZ*Vt)) ~ 1e-117 for
 * a 6.8 V part), which pushed visible conduction out to v ~ -2*vz — an
 * apparent breakdown at double the rated voltage.
 */

/**
 * Evaluate the breakdown diode i(v) at a given voltage without stamping MNA.
 * Used by both the stamp and _updateElementI so both paths share identical math.
 *
 * Equation (v = vAnode - vCathode):
 *   i(v) = Is_f * (exp(v / (nF * Vt)) - 1)
 *          - Is_z * (exp(-(v + vz) / (nZ * Vt)) - 1)
 *
 * The forward term is the standard Shockley diode (positive current for v > 0).
 * The reverse term activates when v goes below -vz, pulling i negative (cathode
 * to anode), modelling controlled breakdown. Sign convention: positive i = current
 * flowing anode → cathode (same as a forward-biased diode).
 *
 * Clamp logic matches stampBreakdownDiode below — see that comment for rationale.
 */
export function breakdownDiodeCurrent(
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
  // Reverse OVERDRIVE past the knee: 0 at v = -vz. Cap at 5*nZ*Vt — beyond
  // that the junction is deep in avalanche and current is circuit-limited;
  // the tangent extension keeps Newton conditioned.
  const vSatZ = 5 * VtNZ;

  const vF = v > vSatF ? vSatF : v;
  const revArg = -(v + vz);
  const revArgClamped = revArg > vSatZ ? vSatZ : revArg;

  const eF = Math.exp(vF / VtNF);
  const eZ = Math.exp(revArgClamped / VtNZ);
  const gFwd = (Is_f * eF) / VtNF;
  const gRev = (izKnee * eZ) / VtNZ;

  // Past either exponential guard, continue along the tangent at the guard
  // rather than freezing the current.  This is both a better high-current
  // approximation (finite dynamic resistance) and keeps the evaluator exactly
  // consistent with the Newton companion stamped below.
  const iFwd = Is_f * (eF - 1) + gFwd * Math.max(0, v - vSatF);
  // No "-1" here: with the prefactor anchored at the knee (Is_z = izKnee), a
  // (exp - 1) form would inject a constant -izKnee of phantom leakage at all
  // forward biases. The pure exponential vanishes for v >> -vz as it should.
  const iRev = izKnee * eZ + gRev * Math.max(0, revArg - vSatZ);
  return iFwd - iRev;
}

/**
 * Reverse-breakdown diode stamp — zener / TVS model (R2 Newton-Raphson).
 *
 * Full i(v) equation (v = vAnode - vCathode):
 *
 *   i(v) = Is_f · (exp(v/(nF·Vt)) − 1)
 *          − izKnee · exp(−(v+vz)/(nZ·Vt))
 *
 * Forward component: standard Shockley, Is_f from shockleyIsFromVf(vf, iRated, nF, Vt).
 * Reverse component: an OVERDRIVE exponential anchored at the knee — its
 *   exponent is zero at v = -vz, so the prefactor IS the knee current
 *   (|i(-vz)| = izKnee by construction; nothing to back-solve). No "-1" term:
 *   anchored at milliamp scale, a (exp − 1) form would leak a constant
 *   -izKnee at all forward biases. nZ = 1 gives a sharp silicon-zener knee.
 *
 * Companion model (Newton-Raphson linearisation about vGuess):
 *   g = di/dv = (Is_f · exp(vF/VtNF)) / VtNF
 *             + (izKnee · exp(revArgClamped/VtNZ)) / VtNZ
 *   Ieq = i(vGuess) − g · vGuess
 *
 * Clamp rationale:
 *   Forward side: vSatF = vSatFOverride ?? 40·nF·Vt, mirroring
 *   stampDiodeShockley — callers with high forward Vf (the TVS at 1.1 V)
 *   raise it the same way the LED path does.
 *
 *   Reverse side: cap the OVERDRIVE -(v+vz) at 5·nZ·Vt (~0.13 V past the
 *   knee). Beyond that the junction is deep in avalanche and the current is
 *   limited by the series circuit, not the junction; the constant-slope
 *   tangent extension keeps Newton well-conditioned.
 *
 * Returns the scalar current i(vGuess) at the linearisation point.
 */
export function stampBreakdownDiode(
  mna: MnaStampSurface,
  i: number,
  j: number,
  vGuess: number,
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
  // Overdrive cap past the knee — see function comment.
  const vSatZ = 5 * VtNZ;

  const vF = vGuess > vSatF ? vSatF : vGuess;

  const revArg = -(vGuess + vz);
  const revArgClamped = revArg > vSatZ ? vSatZ : revArg;

  const eF = Math.exp(vF / VtNF);
  const eZ = Math.exp(revArgClamped / VtNZ);

  // Conductance: sum of both exponential slopes (both contribute positive g).
  const gFwd = (Is_f * eF) / VtNF;
  const gRev = (izKnee * eZ) / VtNZ;

  // A hard-clamped current combined with a tangent anchored at vGuess makes a
  // far-from-knee Newton solve advance by only ~nVt per iteration.  Treat the
  // guarded region as the documented tangent extension instead: current and
  // derivative remain continuous at each guard, and the companion is an exact
  // linearisation of this piecewise model even for a cold, far-away guess.
  const iFwd = Is_f * (eF - 1) + gFwd * Math.max(0, vGuess - vSatF);
  const iRev = izKnee * eZ + gRev * Math.max(0, revArg - vSatZ);
  const id = iFwd - iRev;

  // GMIN floor prevents a floating node when both junctions are cut off.
  const GMIN_D = 1e-12;
  const g = gFwd + gRev + GMIN_D;

  // GMIN is a zero-origin shunt and therefore contributes no Norton offset.
  const ieq = id - (gFwd + gRev) * vGuess;

  if (i >= 0) mna.add(i, i, g);
  if (j >= 0) mna.add(j, j, g);
  if (i >= 0 && j >= 0) {
    mna.add(i, j, -g);
    mna.add(j, i, -g);
  }

  if (i >= 0) mna.addB(i, -ieq);
  if (j >= 0) mna.addB(j, ieq);

  return id;
}

/** Terminal currents and the full Jacobian of the Ebers-Moll BJT at a bias. */
export interface BjtLinearization {
  ib: number;
  ic: number;
  ie: number;
  /** Jacobian entries: row = terminal KCL, col = node voltage. */
  gbB: number;
  gbC: number;
  gbE: number;
  gcB: number;
  gcC: number;
  gcE: number;
  geB: number;
  geC: number;
  geE: number;
}

/**
 * Pure Ebers-Moll linearization at a node-voltage triple — the terminal
 * currents plus the exact Jacobian stampBJT writes into the matrix. Factored
 * out of stampBJT (Wave A5) so the small-signal AC analysis stamps the
 * identical conductance entries the Newton solve used at the operating
 * point; stampBJT's arithmetic (expression order included) is unchanged, it
 * now just reads these values instead of computing them inline. See
 * stampBJT's comment for the model derivation and sign conventions.
 */
export function bjtLinearization(
  vB: number,
  vC: number,
  vE: number,
  polarity: 1 | -1,
  Is = 1e-16,
  betaF = 100,
  betaR = 1,
  nF = 1.0,
  nR = 1.0,
  Vt = 0.02585,
  earlyVoltage = 0,
): BjtLinearization {
  const pol = polarity;
  const vBE = pol * (vB - vE);
  const vBC = pol * (vB - vC);

  const nfVt = nF * Vt;
  const nrVt = nR * Vt;
  const vSatF = 40 * nfVt;
  const vSatR = 40 * nrVt;
  const vf = vBE > vSatF ? vSatF : vBE;
  const vr = vBC > vSatR ? vSatR : vBC;

  const eF = Math.exp(vf / nfVt);
  const eR = Math.exp(vr / nrVt);
  const iF = Is * (eF - 1);
  const iR = Is * (eR - 1);
  const gF = (Is * eF) / nfVt;
  const gR = (Is * eR) / nrVt;

  const bRR = 1 + 1 / betaR;

  // Forward Early effect. A positive VA gives the forward transport current
  // the usual (1 + VCE/VA) dependence, producing finite output resistance in
  // forward-active operation. Zero/non-finite VA intentionally selects the
  // historical Ebers-Moll result. Clamp the multiplier at zero so the
  // simplified model cannot create negative forward transport far outside its
  // useful reverse-VCE region.
  const va = Number.isFinite(earlyVoltage) && earlyVoltage > 0 ? earlyVoltage : 0;
  const earlyRaw = va > 0 ? 1 + (pol * (vC - vE)) / va : 1;
  const earlyScale = Math.max(0, earlyRaw);
  const earlySlope = va > 0 && earlyRaw > 0 ? 1 / va : 0;

  // NPN-formula terminal currents (pre-polarity).
  const icF = iF * earlyScale - iR * bRR;
  const ibF = iF / betaF + iR / betaR;

  // Apply polarity to get actual currents for the chosen device type.
  const ib = pol * ibF;
  const ic = pol * icF;
  const ie = -(ib + ic);

  // Jacobian. Same sign for NPN and PNP — see stampBJT's derivation below.
  // Row = terminal (KCL contribution), col = node voltage.
  const gbB = gF / betaF + gR / betaR;
  const gbC = -gR / betaR;
  const gbE = -gF / betaF;
  const gcB = gF * earlyScale - gR * bRR;
  const gcC = gR * bRR + iF * earlySlope;
  const gcE = -gF * earlyScale - iF * earlySlope;
  // Derive the emitter row from terminal KCL. This keeps every Jacobian
  // column conservative even when the Early-effect product term is active.
  const geB = -(gbB + gcB);
  const geC = -(gbC + gcC);
  const geE = -(gbE + gcE);

  return { ib, ic, ie, gbB, gbC, gbE, gcB, gcC, gcE, geB, geC, geE };
}

/**
 * BJT — transport-form Ebers-Moll (R2).
 *
 * Terminal currents (NPN; polarity flag flips sign for PNP):
 *
 *   V_BE = V_B - V_E        V_BC = V_B - V_C
 *   I_F  = Is · (exp(V_BE/(n_F·Vt)) - 1)   "forward" BE injection
 *   I_R  = Is · (exp(V_BC/(n_R·Vt)) - 1)   "reverse" BC injection
 *   g_F  = (I_F + Is) / (n_F·Vt)
 *   g_R  = (I_R + Is) / (n_R·Vt)
 *   M_E  = max(0, 1 + V_CE/V_A) when V_A > 0, otherwise 1
 *
 *   I_c  =  I_F·M_E - I_R·(1 + 1/β_R)
 *   I_b  =  I_F/β_F + I_R/β_R
 *   I_e  = -(I_c + I_b)
 *
 * PNP is the same topology with inverted junction biases; we compute
 * with V_BE/V_BC pre-multiplied by polarity and then multiply the
 * resulting terminal currents by polarity.
 *
 * Conductance entries (the Jacobian of I_terminal wrt V_node) are
 * identical for NPN and PNP — the two polarity factors cancel through
 * the chain rule. Only the companion RHS current flips sign.
 *
 * Stamps into `mna` and returns the three terminal currents at the
 * linearisation point so the caller can (a) report element current and
 * (b) compute the companion RHS without recomputing the exponentials.
 */
export function stampBJT(
  mna: MnaStampSurface,
  b: number,
  c: number,
  e: number,
  vBprev: number,
  vCprev: number,
  vEprev: number,
  polarity: 1 | -1,
  Is = 1e-16,
  betaF = 100,
  betaR = 1,
  nF = 1.0,
  nR = 1.0,
  Vt = 0.02585,
  earlyVoltage = 0,
): { ib: number; ic: number; ie: number } {
  // Currents and Jacobian come from the shared linearization (Wave A5
  // factoring): identical expressions in identical order, so this stamp's
  // floats are unchanged. Only the matrix/RHS writes remain here.
  const { ib, ic, ie, gbB, gbC, gbE, gcB, gcC, gcE, geB, geC, geE } =
    bjtLinearization(
      vBprev,
      vCprev,
      vEprev,
      polarity,
      Is,
      betaF,
      betaR,
      nF,
      nR,
      Vt,
      earlyVoltage,
    );

  const add = (row: number, col: number, val: number) => {
    if (row >= 0 && col >= 0) mna.add(row, col, val);
  };
  add(b, b, gbB); add(b, c, gbC); add(b, e, gbE);
  add(c, b, gcB); add(c, c, gcC); add(c, e, gcE);
  add(e, b, geB); add(e, c, geC); add(e, e, geE);

  // Companion RHS: I_eq_k = I_k - Σ G_k,col · V_col_prev.
  // KCL at node row has +I_k leaving → RHS contribution is -I_eq_k.
  const iBeq = ib - gbB * vBprev - gbC * vCprev - gbE * vEprev;
  const iCeq = ic - gcB * vBprev - gcC * vCprev - gcE * vEprev;
  const iEeq = ie - geB * vBprev - geC * vCprev - geE * vEprev;
  if (b >= 0) mna.addB(b, -iBeq);
  if (c >= 0) mna.addB(c, -iCeq);
  if (e >= 0) mna.addB(e, -iEeq);

  return { ib, ic, ie };
}

/** Pure BJT terminal-current computation — used by `_updateElementI` to
 *  report per-component current at the converged solution without
 *  restamping the matrix. Mirrors the math inside `stampBJT`. */
export function bjtCurrents(
  vB: number,
  vC: number,
  vE: number,
  polarity: 1 | -1,
  Is = 1e-16,
  betaF = 100,
  betaR = 1,
  nF = 1.0,
  nR = 1.0,
  Vt = 0.02585,
  earlyVoltage = 0,
): { ib: number; ic: number; ie: number } {
  const pol = polarity;
  const vBE = pol * (vB - vE);
  const vBC = pol * (vB - vC);
  const nfVt = nF * Vt;
  const nrVt = nR * Vt;
  const vSatF = 40 * nfVt;
  const vSatR = 40 * nrVt;
  const vf = vBE > vSatF ? vSatF : vBE;
  const vr = vBC > vSatR ? vSatR : vBC;
  const iF = Is * (Math.exp(vf / nfVt) - 1);
  const iR = Is * (Math.exp(vr / nrVt) - 1);
  const bRR = 1 + 1 / betaR;
  const va = Number.isFinite(earlyVoltage) && earlyVoltage > 0 ? earlyVoltage : 0;
  const earlyScale = va > 0
    ? Math.max(0, 1 + (pol * (vC - vE)) / va)
    : 1;
  const icF = iF * earlyScale - iR * bRR;
  const ibF = iF / betaF + iR / betaR;
  const ib = pol * ibF;
  const ic = pol * icF;
  return { ib, ic, ie: -(ib + ic) };
}

/**
 * MOSFET — Shichman-Hodges level-1 (R2) WITH intrinsic body diode.
 *
 * Three terminals (D, G, S); bulk is assumed tied to source. Polarity
 * flag flips sign for PMOS. VTO is a positive magnitude in both cases —
 * the polarity flag (not a signed threshold) selects the channel type,
 * which matches how users think about it in a schematic.
 *
 *   K = (KP/2)·(W/L)
 *   V_GS = V_G - V_S            V_DS = V_D - V_S
 *   V_ov = V_GS - VTO
 *
 *   cutoff     (V_ov ≤ 0):        I_d = 0
 *   triode     (V_DS < V_ov):     I_d = K·(2·V_ov·V_DS - V_DS²)·(1+λ·V_DS)
 *   saturation (V_DS ≥ V_ov):     I_d = K·V_ov²·(1+λ·V_DS)
 *
 * Conductances:
 *   g_m  = ∂I_d/∂V_GS
 *   g_ds = ∂I_d/∂V_DS
 * The DC gate is ideal. Optional Cgs/Cgd values are stamped as true
 * two-terminal backward-Euler companions, so transient gate drive and Miller
 * coupling conserve charge without a hidden reference node.
 *
 * Body diode (intrinsic to all real MOSFETs — bulk tied to source):
 *   NMOS (pol=+1): anode=source, cathode=drain.
 *     Conducts when V_S − V_D > Vf (≈0.7 V), i.e. the device is reverse-
 *     biased or the load is pulling D below S (inductive kick / H-bridge
 *     freewheel). In MNA terms: Shockley stamp with i=source, j=drain.
 *   PMOS (pol=−1): anode=drain, cathode=source (mirror).
 *     Conducts when V_D − V_S > Vf.
 *   Body-diode Is derived from shockleyIsFromVf(0.7, 1.0) → Is_body.
 *   The channel current and body-diode current both sum at d/s nodes.
 *
 * BEHAVIOUR CHANGE: a reverse-biased MOSFET now freewheels above ~0.7 V
 * instead of blocking. Existing NMOS/PMOS fixtures with forward V_DS are
 * unaffected (body diode reverse-biased → near-zero leakage only).
 *
 * Stamp on node indices (d, g, s) with polarity-flipped V_GS / V_DS
 * (same trick as BJT); the resulting conductance entries are identical
 * for NMOS and PMOS, only the companion RHS current flips sign.
 */

export function stampMOSFET(
  mna: MnaStampSurface,
  d: number,
  g: number,
  s: number,
  vDprev: number,
  vGprev: number,
  vSprev: number,
  polarity: 1 | -1,
  VTO = 0.7,
  K = 0.02,
  lambda_ = 0,
  bodyDiodeVf = 0.7,
  thermalV = 0.02585,
  gateDynamics?: {
    h: number;
    cgs: number;
    cgd: number;
    vgsPrev: number;
    vgdPrev: number;
  },
): { id: number } {
  const pol = polarity;
  const vGS = pol * (vGprev - vSprev);
  const vDS = pol * (vDprev - vSprev);
  const vOV = vGS - VTO;

  let idFormula = 0;
  let gm = 0;
  let gds = 0;

  // A tiny conductance floor prevents a dead-flat MOSFET in cutoff or
  // saturation from producing a singular row when it's the only path
  // between D and S.
  const GMIN_M = 1e-12;

  if (vOV <= 0) {
    gds = GMIN_M;
  } else if (vDS < vOV) {
    const r = 2 * vOV * vDS - vDS * vDS;
    const m = 1 + lambda_ * vDS;
    idFormula = K * r * m;
    gm = 2 * K * vDS * m;
    gds = K * (2 * vOV - 2 * vDS) * m + K * r * lambda_;
    if (gds < GMIN_M) gds = GMIN_M;
  } else {
    const m = 1 + lambda_ * vDS;
    idFormula = K * vOV * vOV * m;
    gm = 2 * K * vOV * m;
    gds = K * vOV * vOV * lambda_;
    if (gds < GMIN_M) gds = GMIN_M;
  }

  // Polarity-flipped actual drain current.
  const idActual = pol * idFormula;

  // Conductance stamps. row D / row S pick up the VCCS linearisation;
  // the gate row has no stamp (I_G = 0 for an ideal MOSFET).
  const add = (row: number, col: number, val: number) => {
    if (row >= 0 && col >= 0) mna.add(row, col, val);
  };
  add(d, g, gm);
  add(d, d, gds);
  add(d, s, -gm - gds);
  add(s, g, -gm);
  add(s, d, -gds);
  add(s, s, gm + gds);

  // Companion RHS. With Jacobian entries above the chain-rule polarity
  // factors cancel, so the usual I_eq = I_actual - Σ G · V_prev form
  // works in NMOS-sense node voltages (vDprev, vGprev, vSprev).
  const iDeq =
    idActual -
    (gm * vGprev + gds * vDprev + (-gm - gds) * vSprev);
  if (d >= 0) mna.addB(d, -iDeq);
  if (s >= 0) mna.addB(s, iDeq);

  // ── Intrinsic body diode ──────────────────────────────────────────────────
  // Real MOSFETs have a body diode from bulk (= source) to drain.
  //
  // NMOS (pol=+1): anode=source(s), cathode=drain(d).
  //   Forward biases when V_S > V_D by ~0.7 V (freewheeling / inductive kick).
  //   In stampDiodeShockley convention: node i=anode, node j=cathode.
  //   → stampDiodeShockley(mna, s, d, vPrev_SD, ...)
  //   where vPrev_SD = vSprev − vDprev (anode − cathode voltage).
  //
  // PMOS (pol=−1): anode=drain(d), cathode=source(s) (mirror of NMOS).
  //   → stampDiodeShockley(mna, d, s, vPrev_DS, ...)
  //   where vPrev_DS = vDprev − vSprev (anode − cathode voltage).
  //
  // The body diode current adds to the channel current at d/s; both are
  // captured by KCL at each node — no separate tracking needed here.
  {
    const [bdAnode, bdCathode, vBdPrev] =
      polarity === 1
        ? [s, d, vSprev - vDprev]   // NMOS: anode=S, cathode=D
        : [d, s, vDprev - vSprev];  // PMOS: anode=D, cathode=S
    const bodyDiodeIs = shockleyIsFromVf(bodyDiodeVf, 1.0, 1.0, thermalV);
    stampDiodeShockley(mna, bdAnode, bdCathode, vBdPrev, bodyDiodeIs, 1.0, thermalV);
  }

  // ── Lumped gate charge ────────────────────────────────────────────────────
  // These are physical two-terminal capacitors: Cgs is between G-S and Cgd is
  // between G-D. Their history voltages come only from the previous accepted
  // time step, never a Newton iterate, so adaptive-step restore is deterministic.
  if (gateDynamics && gateDynamics.h > 0) {
    const cgs = Number.isFinite(gateDynamics.cgs) ? Math.max(0, gateDynamics.cgs) : 0;
    const cgd = Number.isFinite(gateDynamics.cgd) ? Math.max(0, gateDynamics.cgd) : 0;
    if (cgs > 0) {
      stampCapacitor(mna, g, s, cgs, gateDynamics.h, gateDynamics.vgsPrev);
    }
    if (cgd > 0) {
      stampCapacitor(mna, g, d, cgd, gateDynamics.h, gateDynamics.vgdPrev);
    }
  }

  return { id: idActual };
}

export interface OpAmpDominantPoleCompanion {
  /** Open-loop dominant-pole frequency, GBW/A0, in hertz. */
  poleHz: number;
  /** Coefficient multiplying the present input differential. */
  differentialGain: number;
  /** Coefficient multiplying the previous internal gain-stage voltage. */
  historyGain: number;
  /** Coefficient multiplying the DC output offset. */
  offsetGain: number;
}

/**
 * Backward-Euler companion for a one-pole op-amp gain stage.
 *
 *   A(s) = A0 / (1 + s/wp),  wp = 2*pi*GBW/A0
 *   du/dt = wp * (A0*vd + offset - u)
 *
 * Backward Euler gives:
 *   u[n] = historyGain*u[n-1]
 *        + differentialGain*vd[n]
 *        + offsetGain*offset
 *
 * A non-positive GBW intentionally selects the historical instantaneous VCVS.
 */
export function opAmpDominantPoleCompanion(
  openLoopGain: number,
  gbwHz: number,
  h: number,
): OpAmpDominantPoleCompanion {
  const gain = Math.max(1e-12, Math.abs(openLoopGain));
  const safeGbw = Number.isFinite(gbwHz) ? Math.max(0, gbwHz) : 0;
  const safeH = Number.isFinite(h) ? Math.max(0, h) : 0;
  if (!(safeGbw > 0) || !(safeH > 0)) {
    return {
      poleHz: safeGbw / gain,
      differentialGain: openLoopGain,
      historyGain: 0,
      offsetGain: 1,
    };
  }
  const poleHz = safeGbw / gain;
  const lambda = 2 * Math.PI * poleHz * safeH;
  const denominator = 1 + lambda;
  return {
    poleHz,
    differentialGain: (lambda / denominator) * openLoopGain,
    historyGain: 1 / denominator,
    offsetGain: lambda / denominator,
  };
}

/** Unconstrained internal gain-stage voltage for one accepted BE step. */
export function opAmpTransientTarget(
  previousInternalVoltage: number,
  inputDifferential: number,
  openLoopGain: number,
  outputOffset: number,
  gbwHz: number,
  h: number,
): number {
  const companion = opAmpDominantPoleCompanion(openLoopGain, gbwHz, h);
  return companion.historyGain * previousInternalVoltage
    + companion.differentialGain * inputDifferential
    + companion.offsetGain * outputOffset;
}

/** Apply a symmetric large-signal slew ceiling to an internal-voltage step. */
export function opAmpSlewLimitedTarget(
  previousInternalVoltage: number,
  unconstrainedTarget: number,
  slewRateVPerSecond: number,
  h: number,
): number {
  const slewRate = Number.isFinite(slewRateVPerSecond)
    ? Math.max(0, slewRateVPerSecond)
    : 0;
  const safeH = Number.isFinite(h) ? Math.max(0, h) : 0;
  // Zero disables the optional large-signal limiter while retaining the pole.
  if (!(slewRate > 0) || !(safeH > 0)) return unconstrainedTarget;
  const maximumStep = slewRate * safeH;
  return previousInternalVoltage
    + Math.max(-maximumStep, Math.min(maximumStep, unconstrainedTarget - previousInternalVoltage));
}

export type OpAmpCurrentLimitMode = -1 | 0 | 1;

/**
 * Commit a source (-1), voltage (0), or sink (+1) output-current regime.
 * naturalCurrent uses stampOpAmp's branch convention: negative sources load
 * current and positive sinks it. Hysteresis lets a constrained output recover
 * without chattering at the exact limit.
 */
export function opAmpCurrentLimitMode(
  previousMode: number,
  naturalCurrent: number,
  sourceLimitA: number,
  sinkLimitA: number,
): OpAmpCurrentLimitMode {
  const sourceLimit = Math.max(0, Number(sourceLimitA));
  const sinkLimit = Math.max(0, Number(sinkLimitA));
  if (!Number.isFinite(naturalCurrent)) return 0;
  if (
    previousMode === -1
    && sourceLimit > 0
    && naturalCurrent <= -sourceLimit * 0.98
  ) {
    return -1;
  }
  if (
    previousMode === 1
    && sinkLimit > 0
    && naturalCurrent >= sinkLimit * 0.98
  ) {
    return 1;
  }
  // Inclusive entry keeps the exact fixed-current solution in the same active
  // set on the following Newton iteration. Opposite-direction overload can
  // replace a released committed mode without exposing an accepted frame.
  if (sourceLimit > 0 && naturalCurrent <= -sourceLimit) return -1;
  if (sinkLimit > 0 && naturalCurrent >= sinkLimit) return 1;
  return 0;
}

export interface OpAmpTransientStamp {
  h: number;
  gbwHz: number;
  previousInternalVoltage: number;
  /** Signed branch current constraint; null keeps voltage-source operation. */
  currentConstraint?: number | null;
}

/**
 * Op-amp macro model — dominant-pole VCVS with rail, slew, and current clamps.
 *
 * Three-terminal ideal model: in_minus (-), in_plus (+), out.
 * The extra-variable row `k` carries the output source current, exactly
 * like a voltage source. The caller selects the present piecewise-linear
 * region: dominant-pole voltage operation, a fixed rail/slew target, or a
 * signed current constraint.
 *
 * The stamp is ALWAYS emitted (even when unpowered) so the matrix row is never
 * all-zero.  When unpowered, vHigh ≈ vLow ≈ 0, causing the output to be
 * clamped to 0 V — a reasonable unpowered model — without any singular row.
 * The row is non-singular in all regimes: voltage operation writes V_out and
 * current-limit operation writes x[k].
 *
 * Linear regime (vLow < vOffset + A·(V+ − V−) < vHigh):
 *   KVL row k:  V_out − A·(V+ − V−) = vOffset
 *   → G[k][out] += 1, G[k][+] −= A, G[k][−] += A, b[k] = vOffset
 *
 * Saturated regime:
 *   KVL row k:  V_out = vHigh  (or vLow)
 *   → G[k][out] += 1, b[k] = vHigh (or vLow)
 *
 * In both regimes the output-source current i_k flows into `out`:
 *   G[out][k] += 1
 *
 * Parameters:
 *   vHigh   — upper saturation rail (absolute node volts, e.g. vcc - headroom)
 *   vLow    — lower saturation rail (absolute node volts, e.g. gnd + headroom)
 *   vOffset — DC bias added to the VCVS output before clamping; used for the
 *             LM386 mid-supply self-bias.  Pass 0 for standard op-amps.
 *   outputResistance — finite closed-loop output-stage resistance.  The KVL
 *             row becomes Vout - R_out*i_k = Vtarget.  Because i_k is negative
 *             while the amplifier sources a load, this produces the expected
 *             load-dependent droop and prevents an unphysical infinite short-
 *             circuit current.  Pass 0 only for an intentionally ideal model.
 *   transient — optional backward-Euler dominant-pole state and signed current
 *             constraint. Omitting it preserves the historical static VCVS.
 */
export function stampOpAmp(
  mna: MnaStampSurface,
  inMinus: number,
  inPlus: number,
  out: number,
  k: number,
  A: number,
  vOffset: number,
  saturatedRail: number | null,
  outputResistance = 0,
  transient: OpAmpTransientStamp | null = null,
): void {
  // VCVS with an extra branch row k. SimEngine performs the active-set test;
  // this routine stamps exactly one linear region and never inspects xGuess.
  if (out >= 0) mna.add(out, k, 1);

  // In current-limit mode KCL and the external load determine output voltage.
  // The caller's complementarity active set must select this branch only while
  // that voltage remains on the overload side of the rail-bounded voltage
  // target's compliance point. x[k] keeps the voltage-source sign convention,
  // so rail-current routing remains conservative.
  const currentConstraint = transient?.currentConstraint;
  if (currentConstraint !== null && currentConstraint !== undefined) {
    mna.add(k, k, 1);
    mna.addB(k, currentConstraint);
    return;
  }

  if (out >= 0) mna.add(k, out, 1);

  // x[k] is current entering the output source.  A sourcing amplifier has
  // x[k] < 0, so Vout - R*x[k] = Vtarget gives Vout = Vtarget - R*Iload.
  // This is the same sign convention as stampVSourceSeriesR.
  if (outputResistance > 0) mna.add(k, k, -outputResistance);

  if (saturatedRail !== null) {
    // Output pinned to the rail: row k is  V_out = saturatedRail.
    // Passed as an explicit absolute voltage — NOT reconstructed by running a
    // rail voltage back through the gain (vRail/A · A round-trips to
    // vRail·(1−ε) at A=1e5 and silently drops below the rail, defeating the
    // clamp). Inputs are intentionally not coupled in this regime.
    mna.addB(k, saturatedRail);
  } else if (transient) {
    // Linear dominant-pole region. The history source and reduced present-time
    // gain are the exact backward-Euler companion of the first-order ODE above.
    const companion = opAmpDominantPoleCompanion(A, transient.gbwHz, transient.h);
    if (inPlus >= 0) mna.add(k, inPlus, -companion.differentialGain);
    if (inMinus >= 0) mna.add(k, inMinus, companion.differentialGain);
    mna.addB(
      k,
      companion.historyGain * transient.previousInternalVoltage
        + companion.offsetGain * vOffset,
    );
  } else {
    // Historical instantaneous linear VCVS.
    if (inPlus >= 0) mna.add(k, inPlus, -A);
    if (inMinus >= 0) mna.add(k, inMinus, A);
    if (vOffset !== 0) mna.addB(k, vOffset);
  }
}

/**
 * Legacy pure output-saturation state helper, retained for compatibility with
 * callers of the original static VCVS API. The transient engine now uses the
 * bounded dominant-pole active set above instead.
 *
 *   ideal = vOffset + A·(V+ − V−)   — the unclamped output the gain stage wants,
 *                                     reconstructed from the committed input
 *                                     differential (works for both feedback
 *                                     op-amps where A is huge and fixed-gain
 *                                     parts like the LM386 where A is small).
 *
 * Transitions (prevRegime: 0 linear, 1 hi-rail, 2 lo-rail; NaN ⇒ linear):
 *   - from linear: enter a rail only if `ideal` exceeds it.
 *   - from a rail: STAY only while `ideal` still exceeds that same rail;
 *     otherwise relax to LINEAR — never jump straight to the opposite rail.
 *     The relax-to-linear step re-solves the unclamped output and the next
 *     commit re-decides cleanly. Jumping rail→rail directly is what oscillates;
 *     staying latched at a rail is what never recovers — this avoids both.
 */
export function opAmpRegime(
  prevRegime: number,
  ideal: number,
  vHigh: number,
  vLow: number,
): 0 | 1 | 2 {
  if (!Number.isFinite(ideal) || !Number.isFinite(prevRegime)) return 0;
  if (prevRegime === 1) return ideal >= vHigh ? 1 : 0;
  if (prevRegime === 2) return ideal <= vLow ? 2 : 0;
  // linear (or unknown): enter a rail only when the gain stage overdrives it.
  if (ideal >= vHigh) return 1;
  if (ideal <= vLow) return 2;
  return 0;
}

// ── W5.1 Linear regulator stamp ──────────────────────────────────────────────

/**
 * Stamps a 3-terminal linear voltage regulator into the MNA matrix.
 *
 * Terminals: inNode (input), refNode (gnd or adj pin), outNode (output).
 * Branch row k holds the branch current — current flows IN → OUT, so
 * it is DRAWN from the input and DELIVERED to the output.
 *
 * Current coupling: in REG/DROPOUT/CC the branch current i represents physical
 * current flowing from inNode to outNode. The gnd/adj pin (refNode) carries
 * negligible current and is used only as a voltage reference. OFF retains the
 * same matrix topology, but pins i=0 so neither terminal loads the other.
 *   if (outNode >= 0) mna.add(outNode, k, +1)   — delivers current to output
 *   if (inNode  >= 0) mna.add(inNode,  k, -1)   — draws current from input
 *
 * Four regimes (decided at commit, NEVER re-decided from within-iteration xGuess):
 *   REG     (0): V_out − V_ref = Vreg   (regulated; ref=gnd for a fixed regulator,
 *                                        ref=adj for LM317 so V_out−V_adj=1.25 V)
 *   DROPOUT (1): V_out − V_in = −vdropout  (pass transistor saturated; output
 *                                            tracks input minus a fixed drop)
 *   CC      (2): i = iLimit              (current-limited below compliance;
 *                                         KCL sets V_out = iLimit * R_load)
 *   OFF     (3): i = 0                    (pass path disabled; output is high-Z)
 *
 * The row k constraint is NEVER all-zero:
 *   REG     → row k has +1 on V_out column
 *   DROPOUT → row k has +1 on V_out column
 *   CC      → row k has +1 on x[k] itself
 *   OFF     → row k has +1 on x[k] itself
 *
 * `SimEngine` wraps CC with a monotonic per-solve active set. If the KCL
 * solution reaches the available REG/DROPOUT voltage, it re-stamps that
 * compliance branch before accepting the step; this primitive therefore never
 * leaves an open/light load as a committed unconstrained current source.
 */
export function stampLinearRegulator(
  mna: MnaStampSurface,
  inNode: number,
  refNode: number,
  outNode: number,
  k: number,
  regime: number,
  Vreg: number,
  vdropout: number,
  iLimit: number,
): void {
  // Keep the branch-to-node coupling in every regime so the matrix topology is
  // stable. In OFF, the branch row below constrains x[k] to exactly zero, which
  // makes this coupling electrically inert and leaves input/output independent.
  if (outNode >= 0) mna.add(outNode, k,  1);
  if (inNode  >= 0) mna.add(inNode,  k, -1);

  if (regime === 0) {
    // REG: V_out − V_ref = Vreg.
    // For a fixed regulator (linear_reg), refNode is gnd (may be -1).
    // For LM317, refNode is the adj pin, so V_out − V_adj = 1.25 V.
    if (outNode >= 0) mna.add(k, outNode,  1);
    if (refNode >= 0) mna.add(k, refNode, -1);
    mna.addB(k, Vreg);
  } else if (regime === 1) {
    // DROPOUT: V_out − V_in = −vdropout.
    // The pass transistor is saturated; output follows input minus a fixed drop.
    if (outNode >= 0) mna.add(k, outNode,  1);
    if (inNode  >= 0) mna.add(k, inNode,  -1);
    mna.addB(k, -vdropout);
  } else if (regime === 2) {
    // CC: i = iLimit. V_out floats and is set by KCL (load).
    // The constraint pins the branch current to −iLimit.  The sign is negative
    // because the MNA coupling convention (mna.add(outNode, k, 1)) means x[k]
    // is current LEAVING the output node — negative for a source delivering
    // current into the output node.  Setting x[k] = −iLimit gives:
    //   KCL at outNode: V_out/Rload + x[k] = 0
    //   V_out/Rload − iLimit = 0 → V_out = iLimit × Rload   (correct physics).
    mna.add(k, k, 1);
    mna.addB(k, -iLimit);
  } else {
    // OFF: disable the pass path. Pinning the branch current to zero keeps the
    // row well posed without inventing an ideal output-to-reference clamp or a
    // reverse/backfeed path from an externally driven output into the input.
    mna.add(k, k, 1);
  }
}

/**
 * Regime state machine for a linear voltage regulator.  Pure function — call
 * ONLY at commit (post-solve, stable x), never from within the Newton loop.
 *
 * Inputs (all from the committed solution):
 *   prevRegime — NaN on first step; 0=REG, 1=DROPOUT, 2=CC, 3=OFF thereafter.
 *   i          — branch current x[k] (positive = flowing in→out).
 *   vIn        — committed voltage at the input pin.
 *   vRef       — committed voltage at the ref pin (gnd or adj).
 *   vOut       — committed voltage at the output pin.
 *   Vreg       — regulated voltage above vRef (e.g. 5.0 for 7805, 1.25 for LM317).
 *   vdropout   — minimum headroom required above the regulated point (e.g. 2.0 V).
 *   iLimit     — maximum output current in amps.
 *
 * Transition table:
 *   prevRegime = NaN/initial
 *     headroom = vIn − vRef
 *     → headroom <= vdropout         → OFF (3)
 *     → headroom < Vreg + vdropout  → DROPOUT (1)
 *     → else                        → REG (0)
 *
 *   from REG (0)
 *     → i > iLimit*1.001            → CC (2)
 *     → headroom < Vreg + vdropout  → DROPOUT (1)
 *     → else                        → REG (0)
 *
 *   from DROPOUT (1)
 *     → headroom >= Vreg + vdropout → REG (0)
 *     → i > iLimit*1.001            → CC (2)
 *     → else                        → DROPOUT (1)
 *
 *   from CC (2)
 *     → vOut >= Vreg + vRef − 0.05  → REG (0)   [load eased, output recovered]
 *     → headroom < Vreg + vdropout  → DROPOUT (1)
 *     → else                        → CC (2)
 *
 *   from OFF (3)
 *     → headroom >= Vreg + vdropout → REG (0)
 *     → headroom > vdropout         → DROPOUT (1)
 *     → else                        → OFF (3)
 *
 * The 1.001 factor and 0.05 V hysteresis prevent chattering at regime edges.
 */
export function regulatorRegime(
  prevRegime: number,
  i: number,
  vIn: number,
  vRef: number,
  vOut: number,
  Vreg: number,
  vdropout: number,
  iLimit: number,
): 0 | 1 | 2 | 3 {
  const headroom = vIn - vRef; // voltage available above the reference node

  // Use absolute value of branch current for regime decisions.
  // x[k] is negative in MNA convention when the regulator delivers current
  // (the KCL coupling row at outNode has +1, so x[k] = −I_delivered).
  // Comparing |i| to iLimit gives the correct overload detection.
  const absI = Math.abs(i);

  // A positive regulator cannot drive below its own reference rail. Below the
  // pass-device drop it is off, not a negative voltage source.
  if (headroom <= Math.max(0, vdropout)) return 3;

  if (!Number.isFinite(prevRegime)) {
    // First step: pick the most likely starting regime from input conditions.
    return headroom < Vreg + vdropout ? 1 : 0;
  }

  if (prevRegime === 0) {
    // Currently regulating.
    if (absI > iLimit * 1.001) return 2;              // load demanding too much → CC
    if (headroom < Vreg + vdropout) return 1;         // insufficient headroom  → DROPOUT
    return 0;
  }

  if (prevRegime === 1) {
    // Currently in dropout.
    if (headroom >= Vreg + vdropout) return 0;        // headroom restored      → REG
    if (absI > iLimit * 1.001) return 2;              // still overloaded       → CC
    return 1;
  }

  if (prevRegime === 3) {
    return headroom >= Vreg + vdropout ? 0 : 1;
  }

  // prevRegime === 2: currently current-limited.
  if (vOut >= Vreg + vRef - 0.05) return 0;           // output recovered       → REG
  if (headroom < Vreg + vdropout) return 1;            // low headroom           → DROPOUT
  return 2;
}

/** Pure MOSFET drain-current computation — matches `stampMOSFET` math. */
export function mosDrainCurrent(
  vD: number,
  vG: number,
  vS: number,
  polarity: 1 | -1,
  VTO = 0.7,
  K = 0.02,
  lambda_ = 0,
): number {
  const pol = polarity;
  const vGS = pol * (vG - vS);
  const vDS = pol * (vD - vS);
  const vOV = vGS - VTO;
  if (vOV <= 0) return 0;
  const m = 1 + lambda_ * vDS;
  const id =
    vDS < vOV
      ? K * (2 * vOV * vDS - vDS * vDS) * m
      : K * vOV * vOV * m;
  return pol * id;
}

// ── Wave A6: JFET (Shichman-Hodges) ─────────────────────────────────────────

/** Drain current and channel small-signal slopes of the JFET law below. */
export interface JfetLinearization {
  /** Polarity-corrected drain current (positive into the drain terminal). */
  id: number;
  /** dId/dVgs at the bias (polarity-invariant, like the MOSFET Jacobian). */
  gm: number;
  /** dId/dVds at the bias, floored at GMIN so D-S never floats in cutoff. */
  gds: number;
}

/**
 * Shichman-Hodges JFET channel linearization at a node-voltage triple.
 *
 * Same quadratic law as the level-1 MOSFET but in the depletion-mode
 * convention: `vto` is SIGNED AND NEGATIVE for both channel types (n-JFET
 * vto = -2 V conducts at vGS = 0 and pinches off at vGS <= -2 V; the p-JFET
 * uses the same negative number against polarity-flipped voltages, exactly
 * the MOSFET polarity trick). beta comes from the datasheet pair
 * (idss, vto) via beta = idss/vto², so Id(vGS = 0) = idss in saturation by
 * construction.
 *
 *   vGS' = pol·(vG - vS), vDS' = pol·(vD - vS), vOV = vGS' - vto
 *   cutoff     (vOV <= 0):     Id = 0
 *   triode     (vDS' < vOV):   Id = beta·(2·vOV·vDS' - vDS'²)·(1 + λ·vDS')
 *   saturation (vDS' >= vOV):  Id = beta·vOV²·(1 + λ·vDS')
 *
 * Like stampMOSFET, the law is valid for forward operation (vDS' >= 0);
 * gm/gds are the exact partial derivatives of each region and the GMIN
 * floor mirrors the MOSFET stamp's conditioning guard. No body diode here —
 * a JFET's gate-channel junctions are stamped separately by the caller.
 */
export function jfetLinearization(
  vD: number,
  vG: number,
  vS: number,
  polarity: 1 | -1,
  vto: number,
  beta: number,
  lambda_ = 0,
): JfetLinearization {
  const pol = polarity;
  const vGS = pol * (vG - vS);
  const vDS = pol * (vD - vS);
  const vOV = vGS - vto;
  const GMIN_J = 1e-12;

  let idFormula = 0;
  let gm = 0;
  let gds = 0;
  if (vOV <= 0) {
    gds = GMIN_J;
  } else if (vDS < vOV) {
    const r = 2 * vOV * vDS - vDS * vDS;
    const m = 1 + lambda_ * vDS;
    idFormula = beta * r * m;
    gm = 2 * beta * vDS * m;
    gds = beta * (2 * vOV - 2 * vDS) * m + beta * r * lambda_;
    if (gds < GMIN_J) gds = GMIN_J;
  } else {
    const m = 1 + lambda_ * vDS;
    idFormula = beta * vOV * vOV * m;
    gm = 2 * beta * vOV * m;
    gds = beta * vOV * vOV * lambda_;
    if (gds < GMIN_J) gds = GMIN_J;
  }
  return { id: pol * idFormula, gm, gds };
}

/**
 * Stamp the JFET channel as the Newton VCCS companion — the identical
 * matrix/RHS pattern stampMOSFET writes for its channel (the chain-rule
 * polarity factors cancel in the Jacobian, only the companion current
 * flips), without the MOSFET's intrinsic body diode or gate charge. The
 * caller stamps the gate-channel junction diodes on top; the DC gate here
 * carries no current. Returns the drain current at the linearisation point.
 */
export function stampJFET(
  mna: MnaStampSurface,
  d: number,
  g: number,
  s: number,
  vDprev: number,
  vGprev: number,
  vSprev: number,
  polarity: 1 | -1,
  vto: number,
  beta: number,
  lambda_ = 0,
): { id: number } {
  const { id, gm, gds } = jfetLinearization(
    vDprev,
    vGprev,
    vSprev,
    polarity,
    vto,
    beta,
    lambda_,
  );
  const add = (row: number, col: number, val: number) => {
    if (row >= 0 && col >= 0) mna.add(row, col, val);
  };
  add(d, g, gm);
  add(d, d, gds);
  add(d, s, -gm - gds);
  add(s, g, -gm);
  add(s, d, -gds);
  add(s, s, gm + gds);
  const iDeq = id - (gm * vGprev + gds * vDprev + (-gm - gds) * vSprev);
  if (d >= 0) mna.addB(d, -iDeq);
  if (s >= 0) mna.addB(s, iDeq);
  return { id };
}
