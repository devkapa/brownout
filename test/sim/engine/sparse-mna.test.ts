import { describe, expect, it } from "vitest";
import { MNA } from "../../../src/sim/engine/mna.js";
import { SparseMNA } from "../../../src/sim/engine/sparse-mna.js";

/** Minimal stamping surface shared by both backends for cross-checks. */
interface Stampable {
  add(row: number, col: number, val: number): void;
  addB(row: number, val: number): void;
}

const LADDER_SIZE = 300;

/**
 * Resistor ladder with an ideal-source bordered branch row: 299 node
 * unknowns plus one branch current. Mixed series resistances, sparse
 * megaohm shunts, and a mid-ladder load current give the cross-check
 * a spread of magnitudes without making the system ill-conditioned.
 */
function stampLadder(sys: Stampable): void {
  const nodes = LADDER_SIZE - 1;
  const branch = LADDER_SIZE - 1;
  for (let i = 0; i < nodes - 1; i++) {
    const g = 1 / (100 + (i % 10) * 47);
    sys.add(i, i, g);
    sys.add(i + 1, i + 1, g);
    sys.add(i, i + 1, -g);
    sys.add(i + 1, i, -g);
  }
  for (let i = 0; i < nodes; i += 7) {
    sys.add(i, i, 1e-6);
  }
  sys.add(0, branch, 1);
  sys.add(branch, 0, 1);
  sys.addB(branch, 5);
  sys.addB(150, 2e-3);
}

describe("SparseMNA equilibrated solver contract", () => {
  it("solves mixed-scale rows without discarding a valid tiny conductance", () => {
    const mna = new SparseMNA(2);
    // 1 TΩ node alongside an ideal-source-scale row. An absolute pivot
    // cutoff in raw units would erase this physically valid row; the
    // sparse backend must equilibrate exactly like dense before testing.
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
    const mna = new SparseMNA(2);
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
    const mna = new SparseMNA(2);
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

  it("factorises a voltage-source bordered block with a structural-zero diagonal", () => {
    const mna = new SparseMNA(2);
    // Classic ideal-source border: the branch row (1,1) is never stamped,
    // so the pattern has no diagonal entry there and the pivot search must
    // escape to the off-diagonal instead of assuming the diagonal exists.
    mna.add(0, 0, 0.5);
    mna.add(0, 1, 1);
    mna.add(1, 0, 1);
    mna.addB(1, 3);

    const x = mna.solve();
    expect(x[0]).toBeCloseTo(3, 12);
    expect(x[1]).toBeCloseTo(-1.5, 12);
    expect(mna.lastSolveInfo.singular).toBe(false);
  });

  it("reports a singular matrix instead of silently presenting it as healthy", () => {
    const mna = new SparseMNA(2);
    mna.add(0, 0, 1);
    mna.add(1, 0, 2);
    mna.solve();

    expect(mna.lastSolveInfo.singular).toBe(true);
    expect(mna.lastSolveInfo.rank).toBe(1);
  });

  it("flags a duplicate-constraint system without throwing and keeps output finite", () => {
    const mna = new SparseMNA(3);
    // Two ideal sources pinning the same node to the same value: rows 1 and
    // 2 are identical, so one pivot step must fail. The engine relies on the
    // singular flag to reject the step; the solver itself must stay total.
    mna.add(0, 0, 1);
    mna.add(0, 1, 1);
    mna.add(1, 0, 1);
    mna.addB(1, 5);
    mna.add(0, 2, 1);
    mna.add(2, 0, 1);
    mna.addB(2, 5);

    const x = mna.solve();
    expect(mna.lastSolveInfo.singular).toBe(true);
    expect(mna.lastSolveInfo.rank).toBe(2);
    for (let i = 0; i < 3; i++) {
      expect(Number.isFinite(x[i])).toBe(true);
    }
  });

  it("solves an all-zero matrix as fully singular with a zero vector", () => {
    const mna = new SparseMNA(2);
    const x = mna.solve();
    expect([...x]).toEqual([0, 0]);
    expect(mna.lastSolveInfo.singular).toBe(true);
    expect(mna.lastSolveInfo.rank).toBe(0);
    expect(mna.lastSolveInfo.nonFinite).toBe(false);
  });

  it("handles size 0 without factorising", () => {
    const mna = new SparseMNA(0);
    expect(mna.solve().length).toBe(0);
    expect(mna.solveRhs(new Float64Array(0)).length).toBe(0);
  });

  it("rejects an RHS whose length does not match the matrix size", () => {
    const mna = new SparseMNA(2);
    mna.add(0, 0, 1);
    mna.add(1, 1, 1);
    expect(() => mna.solveRhs(new Float64Array(3))).toThrow(RangeError);
  });

  it("reports non-finite coefficients and residuals", () => {
    const mna = new SparseMNA(1);
    mna.add(0, 0, Number.NaN);
    mna.addB(0, 1);
    // Must surface through lastSolveInfo, not throw: the engine rejects the
    // step based on the flag, and a throw would take down the worker loop.
    const x = mna.solve();
    expect(mna.lastSolveInfo.nonFinite).toBe(true);
    expect(x.length).toBe(1);
  });
});

describe("SparseMNA base/overlay snapshot contract", () => {
  it("erases overlay slots appended after captureBase on resetToBase", () => {
    const mna = new SparseMNA(2);
    mna.add(0, 0, 2);
    mna.add(1, 1, 4);
    mna.addB(0, 2);
    mna.addB(1, 4);
    mna.captureBase();
    const baseX = mna.solve();
    expect(baseX[0]).toBeCloseTo(1, 12);
    expect(baseX[1]).toBeCloseTo(1, 12);

    // Overlay at coordinates the base never touched: these mint new slots,
    // so resetToBase must zero them rather than merely restoring the base
    // region — a stale coupling here would corrupt every later iteration.
    const overlay = (): void => {
      mna.add(0, 1, 1);
      mna.add(1, 0, 1);
      mna.addB(0, 1);
    };
    overlay();
    const overlayX = mna.solve();
    expect(overlayX[0]).toBeCloseTo(8 / 7, 12);
    expect(overlayX[1]).toBeCloseTo(5 / 7, 12);

    mna.resetToBase();
    const restored = mna.solve();
    expect(restored[0]).toBeCloseTo(baseX[0], 12);
    expect(restored[1]).toBeCloseTo(baseX[1], 12);
    expect(mna.lastSolveInfo.singular).toBe(false);

    // A fresh identical overlay must land in the retained slots and
    // reproduce the overlay solution, proving reset did not lose the
    // pattern or leave residue in the overlay values.
    mna.resetToBase();
    overlay();
    const replayed = mna.solve();
    expect(replayed[0]).toBeCloseTo(overlayX[0], 12);
    expect(replayed[1]).toBeCloseTo(overlayX[1], 12);
  });

  it("recaptures a grown pattern so later resets keep the new slots' values", () => {
    const mna = new SparseMNA(2);
    mna.add(0, 0, 2);
    mna.add(1, 1, 2);
    mna.addB(0, 2);
    mna.addB(1, 2);
    mna.captureBase();

    // Grow the pattern past the first snapshot, then promote the grown
    // matrix to the new base. resetToBase must now preserve the coupling
    // values instead of zeroing them as overlay.
    mna.add(0, 1, -1);
    mna.add(1, 0, -1);
    const grownX = mna.solve();
    expect(grownX[0]).toBeCloseTo(2, 12);
    expect(grownX[1]).toBeCloseTo(2, 12);
    mna.captureBase();

    mna.add(0, 0, 1);
    mna.addB(0, 1);
    const perturbed = mna.solve();
    expect(perturbed[0]).toBeCloseTo(1.6, 12);
    expect(perturbed[1]).toBeCloseTo(1.8, 12);

    mna.resetToBase();
    const restored = mna.solve();
    expect(restored[0]).toBeCloseTo(grownX[0], 12);
    expect(restored[1]).toBeCloseTo(grownX[1], 12);
  });
});

describe("SparseMNA vs dense MNA cross-check", () => {
  it("matches the dense backend on a 300-unknown ladder", () => {
    const dense = new MNA(LADDER_SIZE);
    const sparse = new SparseMNA(LADDER_SIZE);
    stampLadder(dense);
    stampLadder(sparse);

    const xd = dense.solve();
    const xs = sparse.solve();
    expect(dense.lastSolveInfo.singular).toBe(false);
    expect(sparse.lastSolveInfo.singular).toBe(false);
    // Both backends validate against the original unscaled matrix, so the
    // residual is the shared oracle; the elementwise band absorbs the
    // roundoff difference from the differing elimination orders.
    expect(dense.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(1e-10);
    expect(sparse.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(1e-10);
    for (let i = 0; i < LADDER_SIZE; i++) {
      const tolerance = 1e-9 + 1e-9 * Math.max(Math.abs(xd[i]), Math.abs(xs[i]));
      expect(Math.abs(xd[i] - xs[i])).toBeLessThanOrEqual(tolerance);
    }
  });
});
