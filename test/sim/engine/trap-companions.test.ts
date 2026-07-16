/**
 * Analytical contracts for the opt-in trapezoidal capacitor and inductor
 * companions.
 *
 * Current is positive from a component's first pin to its second pin. All
 * checks here are companion-level (pure functions, no SimEngine): the trap
 * variants are additive, so the guarantees are (a) they satisfy the documented
 * trapezoidal discretisation exactly, (b) their guard semantics match the
 * backward-Euler functions, and (c) the backward-Euler functions themselves
 * are byte-unchanged.
 */

import { describe, expect, it } from "vitest";
import {
  capacitorCompanion,
  capacitorCompanionTrap,
  capacitorInternalVoltageTrap,
  capacitorSeriesCurrentTrap,
  inductorCompanion,
  inductorCompanionTrap,
  inductorWindingCurrentTrap,
} from "../../../src/sim/engine/elements.js";

describe("capacitor trapezoidal companion", () => {
  it("matches the documented derivation with nonzero ESR", () => {
    // Values chosen so every intermediate is exact in binary floating point:
    // h/(2C) = 0.5/4 = 0.125, G = 1/(3 + 0.125) = 0.32.
    const C = 2;
    const h = 0.5;
    const esr = 3;
    const vcPrev = 3;
    const iPrev = 4;

    const companion = capacitorCompanionTrap(C, h, vcPrev, iPrev, esr);
    expect(companion.conductance).toBe(1 / (esr + h / (2 * C)));
    expect(companion.historyCurrent).toBe(
      companion.conductance * (vcPrev + (h / (2 * C)) * iPrev),
    );

    const terminalV = 6;
    const current = capacitorSeriesCurrentTrap(terminalV, C, h, vcPrev, iPrev, esr);
    expect(current).toBeCloseTo(
      companion.conductance * terminalV - companion.historyCurrent,
      15,
    );
  });

  it("is backward Euler at h/2 with the history voltage augmented by (h/2C)*iPrev", () => {
    // Power-of-two step and capacitance keep h/(2C) and (h/2)/C bit-identical,
    // so the equivalence can be asserted exactly rather than approximately.
    const C = 2;
    const h = 0.5;
    const esr = 3;
    const vcPrev = 3;
    const iPrev = 4;

    const trap = capacitorCompanionTrap(C, h, vcPrev, iPrev, esr);
    const augmentedHistory = vcPrev + (h / (2 * C)) * iPrev;
    const be = capacitorCompanion(C, h / 2, augmentedHistory, esr);

    expect(trap.conductance).toBe(be.conductance);
    expect(trap.historyCurrent).toBe(be.historyCurrent);
  });

  it("advances vc by the trapezoid of branch currents and satisfies the ESR terminal law", () => {
    const C = 1;
    const h = 0.5;
    const esr = 2;
    const vcPrev = 3;
    const iPrev = 1;
    const terminalV = 6;

    const iNew = capacitorSeriesCurrentTrap(terminalV, C, h, vcPrev, iPrev, esr);
    const vcNew = capacitorInternalVoltageTrap(vcPrev, iPrev, iNew, C, h);

    // Charge conservation: the state advance is exactly the trapezoidal
    // integral of the two endpoint currents.
    expect(vcNew - vcPrev).toBeCloseTo((h / (2 * C)) * (iNew + iPrev), 12);
    // Hand-derived values: G = 4/9, hist = 13/9, iNew = 11/9, vcNew = 32/9.
    expect(iNew).toBeCloseTo(11 / 9, 12);
    expect(vcNew).toBeCloseTo(32 / 9, 12);
    // The advance-from-current form must remain algebraically identical to
    // vc = terminalVoltage - ESR*iNew for the solved companion branch.
    expect(terminalV).toBeCloseTo(vcNew + esr * iNew, 12);
  });

  it("applies the same C/h floors and esr sanitisation as backward Euler", () => {
    // Zero and negative C/h must resolve to the same 1e-30 floor the BE
    // companion uses, so degenerate parts behave identically in both modes.
    expect(capacitorCompanionTrap(0, 0.5, 1, 2, 3)).toEqual(
      capacitorCompanionTrap(1e-30, 0.5, 1, 2, 3),
    );
    expect(capacitorCompanionTrap(1, 0, 1, 2, 3)).toEqual(
      capacitorCompanionTrap(1, 1e-30, 1, 2, 3),
    );
    expect(capacitorCompanion(0, 0.5, 1, 3)).toEqual(
      capacitorCompanion(1e-30, 0.5, 1, 3),
    );

    // Nonfinite or negative ESR collapses to 0 rather than poisoning the
    // matrix, mirroring the BE guard.
    const clean = capacitorCompanionTrap(1e-6, 1e-5, 2, 0.5, 0);
    expect(capacitorCompanionTrap(1e-6, 1e-5, 2, 0.5, Number.NaN)).toEqual(clean);
    expect(
      capacitorCompanionTrap(1e-6, 1e-5, 2, 0.5, Number.POSITIVE_INFINITY),
    ).toEqual(clean);
    expect(capacitorCompanionTrap(1e-6, 1e-5, 2, 0.5, -1)).toEqual(clean);
    expect(capacitorCompanion(1e-6, 1e-5, 2, Number.NaN)).toEqual(
      capacitorCompanion(1e-6, 1e-5, 2, 0),
    );

    // The state updater shares the same floors, so it stays finite for C = 0.
    expect(capacitorInternalVoltageTrap(1, 2, 3, 0, 0.5)).toBe(
      capacitorInternalVoltageTrap(1, 2, 3, 1e-30, 0.5),
    );
    expect(Number.isFinite(capacitorInternalVoltageTrap(1, 2, 3, 0, 0.5))).toBe(true);
  });
});

describe("inductor trapezoidal companion", () => {
  it("matches the documented derivation with nonzero DCR", () => {
    const L = 2;
    const h = 0.5;
    const dcr = 3;
    const iPrev = 4;
    const vPrev = 7;
    const terminalV = 9;

    const companion = inductorCompanionTrap(L, h, iPrev, vPrev, dcr);
    // lOverH = 4, denominator = 5.5: G = 1/11, hist = 13.5/5.5 = 27/11.
    expect(companion.conductance).toBeCloseTo(1 / 11, 12);
    expect(companion.historyCurrent).toBeCloseTo(27 / 11, 12);

    const iNew = inductorWindingCurrentTrap(terminalV, L, h, iPrev, vPrev, dcr);
    expect(iNew).toBeCloseTo(36 / 11, 12);

    // The solved current must satisfy the trapezoidal winding law
    //   (iNew - iPrev)/h = (1/2L)*((vNew - DCR*iNew) + (vPrev - DCR*iPrev))
    // which is the ground truth the Norton pair was derived from.
    expect((iNew - iPrev) / h).toBeCloseTo(
      ((terminalV - dcr * iNew) + (vPrev - dcr * iPrev)) / (2 * L),
      12,
    );
  });

  it("reduces to G = h/2L with history iPrev + vPrev*h/2L when DCR is zero", () => {
    const L = 2;
    const h = 0.5;
    const iPrev = 4;
    const vPrev = 7;

    const companion = inductorCompanionTrap(L, h, iPrev, vPrev, 0);
    expect(companion.conductance).toBeCloseTo(h / (2 * L), 15);
    expect(companion.historyCurrent).toBeCloseTo(iPrev + (vPrev * h) / (2 * L), 15);
  });

  it("applies the same L/h floors and dcr sanitisation as backward Euler", () => {
    expect(inductorCompanionTrap(0, 0.5, 1, 2, 3)).toEqual(
      inductorCompanionTrap(1e-30, 0.5, 1, 2, 3),
    );
    expect(inductorCompanionTrap(1, 0, 1, 2, 3)).toEqual(
      inductorCompanionTrap(1, 1e-30, 1, 2, 3),
    );
    expect(inductorCompanion(0, 0.5, 1, 3)).toEqual(
      inductorCompanion(1e-30, 0.5, 1, 3),
    );

    const clean = inductorCompanionTrap(1e-3, 1e-5, 0.25, 2, 0);
    expect(inductorCompanionTrap(1e-3, 1e-5, 0.25, 2, Number.NaN)).toEqual(clean);
    expect(
      inductorCompanionTrap(1e-3, 1e-5, 0.25, 2, Number.POSITIVE_INFINITY),
    ).toEqual(clean);
    expect(inductorCompanionTrap(1e-3, 1e-5, 0.25, 2, -1)).toEqual(clean);
    expect(inductorCompanion(1e-3, 1e-5, 0.25, Number.NaN)).toEqual(
      inductorCompanion(1e-3, 1e-5, 0.25, 0),
    );
  });
});

describe("backward-Euler companions stay byte-identical after the trap addition", () => {
  it("capacitorCompanion returns exactly the pre-trap values", () => {
    // Expected values are computed with the exact operation order from the
    // TSDoc formula G = 1/(ESR + h/C), so toBe locks every bit.
    const companion = capacitorCompanion(1e-6, 1e-5, 2, 0.5);
    const expectedConductance = 1 / (0.5 + 1e-5 / 1e-6);
    expect(companion.conductance).toBe(expectedConductance);
    expect(companion.historyCurrent).toBe(expectedConductance * 2);
  });

  it("inductorCompanion returns exactly the pre-trap values", () => {
    // G = h/(L + DCR*h), I_hist = L/(L + DCR*h)*iPrev per the TSDoc.
    const companion = inductorCompanion(1e-3, 1e-5, 0.25, 0.1);
    const denominator = 1e-3 + 0.1 * 1e-5;
    expect(companion.conductance).toBe(1e-5 / denominator);
    expect(companion.historyCurrent).toBe((1e-3 / denominator) * 0.25);
  });
});

describe("analytic single-element behaviour at the companion level", () => {
  it("one trap step of a series RC matches the closed-form trapezoid update", () => {
    const R = 1_000;
    const C = 1e-6;
    const h = 1e-4;
    const Vs = 5;
    const vc0 = 1;
    // iPrev is the accepted branch current at t0; for a constant source and
    // ideal C (no ESR) that is the resistor current at the old state.
    const iPrev = (Vs - vc0) / R;

    const companion = capacitorCompanionTrap(C, h, vc0, iPrev, 0);
    // KCL at the single interior node: (Vs - v)/R = G*v - hist.
    const v = (Vs / R + companion.historyCurrent) / (1 / R + companion.conductance);
    const iNew = capacitorSeriesCurrentTrap(v, C, h, vc0, iPrev, 0);
    const vcNew = capacitorInternalVoltageTrap(vc0, iPrev, iNew, C, h);

    // Closed-form trapezoid on dvc/dt = (Vs - vc)/(RC):
    //   vc1*(1 + a) = vc0*(1 - a) + 2a*Vs, a = h/(2RC)
    const a = h / (2 * R * C);
    const vcClosedForm = (vc0 * (1 - a) + 2 * a * Vs) / (1 + a);

    expect(vcNew).toBeCloseTo(vcClosedForm, 12);
    // Zero ESR makes the terminal voltage equal the new internal voltage.
    expect(v).toBeCloseTo(vcClosedForm, 12);
    // The branch current the state update consumed is the resistor current.
    expect(iNew).toBeCloseTo((Vs - v) / R, 12);
  });

  it("conserves lossless LC energy to machine precision over two trap steps", () => {
    // Trapezoidal integration is a Cayley transform of the LC state matrix,
    // so 0.5*C*vc^2 + 0.5*L*iL^2 is an exact discrete invariant; any drift
    // beyond floating-point noise indicates a companion coefficient bug.
    const C = 1e-6;
    const L = 1e-3;
    const h = 1e-5;

    // Parallel LC loop, node a against ground. Both branch currents are
    // positive a -> ground, so KCL is iCap + iInd = 0.
    let vc = 1;
    let iL = 0;
    let iCap = -iL;
    let vTerm = vc;

    const energy = () => 0.5 * C * vc * vc + 0.5 * L * iL * iL;
    const initialEnergy = energy();

    for (let step = 0; step < 2; step++) {
      const cap = capacitorCompanionTrap(C, h, vc, iCap, 0);
      const ind = inductorCompanionTrap(L, h, iL, vTerm, 0);
      // iCap + iInd = 0 with iCap = Gc*v - histC and iInd = Gl*v + histL.
      const v =
        (cap.historyCurrent - ind.historyCurrent) /
        (cap.conductance + ind.conductance);

      const iCapNew = capacitorSeriesCurrentTrap(v, C, h, vc, iCap, 0);
      const iLNew = inductorWindingCurrentTrap(v, L, h, iL, vTerm, 0);
      const vcNew = capacitorInternalVoltageTrap(vc, iCap, iCapNew, C, h);

      // Per-step discretisation identities guard against sign slips that
      // energy conservation alone might mask.
      expect(iCapNew + iLNew).toBeCloseTo(0, 12);
      expect(vcNew - vc).toBeCloseTo((h / (2 * C)) * (iCapNew + iCap), 12);
      expect(iLNew - iL).toBeCloseTo((h / (2 * L)) * (v + vTerm), 12);

      vc = vcNew;
      iL = iLNew;
      iCap = iCapNew;
      vTerm = v;
    }

    expect(Math.abs(energy() - initialEnergy) / initialEnergy).toBeLessThan(1e-12);
  });
});
