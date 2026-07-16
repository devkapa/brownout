import { describe, expect, it } from "vitest";
import { MNA } from "../../../src/sim/engine/mna.js";

describe("MNA equilibrated solver", () => {
  it("solves mixed-scale rows without discarding a valid tiny conductance", () => {
    const mna = new MNA(2);
    // 1 TΩ node alongside an ideal-source-scale row. The former absolute
    // 1e-15 pivot cutoff treated matrices in raw units and could erase this
    // physically valid row as values moved into the high-impedance range.
    mna.add(0, 0, 1e-12);
    mna.add(1, 1, 1);
    mna.addB(0, 2e-12);
    mna.addB(1, 3);

    const x = mna.solve();
    expect(x[0]).toBeCloseTo(2, 12);
    expect(x[1]).toBeCloseTo(3, 12);
    expect(mna.lastSolveInfo.singular).toBe(false);
  });

  it("reuses the factorisation for a separate probe RHS", () => {
    const mna = new MNA(2);
    mna.add(0, 0, 2);
    mna.add(0, 1, -1);
    mna.add(1, 0, -1);
    mna.add(1, 1, 3);

    const operatingPoint = mna.solve();
    expect([...operatingPoint]).toEqual([0, 0]);

    const response = mna.solveRhs(new Float64Array([1, 0]));
    expect(response[0]).toBeCloseTo(0.6, 12);
    expect(response[1]).toBeCloseTo(0.2, 12);
    expect(mna.factorizationCount).toBe(1);
  });

  it("reuses LU only when a fresh stamp rebuilds the exact same matrix", () => {
    const mna = new MNA(2);
    const stamp = (diagonal = 2): void => {
      mna.clear();
      mna.add(0, 0, diagonal);
      mna.add(0, 1, -1);
      mna.add(1, 0, -1);
      mna.add(1, 1, 3);
      mna.addB(0, 1);
    };

    stamp();
    const first = mna.solve();
    stamp();
    const reused = mna.solve();
    expect([...reused]).toEqual([...first]);
    expect(mna.factorizationCount).toBe(1);
    expect(mna.factorizationReuseCount).toBe(1);

    stamp(2.000000000000001);
    mna.solve();
    expect(mna.factorizationCount).toBe(2);
    expect(mna.factorizationReuseCount).toBe(1);
  });

  it("reports a singular matrix instead of silently presenting it as healthy", () => {
    const mna = new MNA(2);
    mna.add(0, 0, 1);
    mna.add(1, 0, 2);
    mna.solve();

    expect(mna.lastSolveInfo.singular).toBe(true);
    expect(mna.lastSolveInfo.rank).toBe(1);
  });

  it("reports non-finite coefficients and residuals", () => {
    const mna = new MNA(1);
    mna.add(0, 0, Number.NaN);
    mna.addB(0, 1);
    mna.solve();
    expect(mna.lastSolveInfo.nonFinite).toBe(true);
  });
});
