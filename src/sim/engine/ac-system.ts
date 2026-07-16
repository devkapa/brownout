/**
 * Complex linear system for small-signal AC analysis (Wave A5), built as a
 * bordered 2n REAL system on top of the existing LinearSystem backends.
 *
 * For the complex admittance system  (G + jB)(xr + j·xi) = (br + j·bi)  the
 * real and imaginary parts separate into
 *
 *   [ G  -B ] [xr]   [br]
 *   [ B   G ] [xi] = [bi]
 *
 * stamped with the imaginary rows/columns stacked at index + n.
 *
 * WHY a bordered real system instead of a native complex kernel:
 * createLinearSystem() already carries everything a production solve needs —
 * the row-equilibrated dense LU, the sparse LU with its AMD fill-reducing
 * ordering and value-only refactorization fast path, solution validation
 * against the original (unscaled) matrix, and singular/ill-conditioned/
 * non-finite reporting through lastSolveInfo. A native complex kernel would
 * have to duplicate every one of those guards for a single analysis mode and
 * would sit outside the backend-equivalence gates that keep dense and sparse
 * results interchangeable. The bordered form costs a 2n system (8x the dense
 * flops of an n-sized real solve), which is negligible at this engine's
 * matrix scales, and in exchange every future backend improvement applies to
 * AC solves for free. Because the sparse backend keeps its discovered stamp
 * pattern across clear(), re-stamping the frequency-invariant topology per
 * sweep point reuses the pattern and its ordering across the whole sweep —
 * only the values change between points.
 */

import { createLinearSystem, type LinearSystem } from "./linear-system.js";
import type { MnaSolveInfo } from "./mna.js";

export class AcSystem {
  /** Rows of the underlying MNA layout (node rows plus branch rows). */
  readonly size: number;
  private readonly system: LinearSystem;

  constructor(size: number) {
    this.size = size;
    this.system = createLinearSystem(2 * size);
  }

  /** Backend discriminator of the wrapped 2n system (diagnostics only). */
  get backend(): "dense" | "sparse" {
    return this.system.backend ?? "dense";
  }

  /**
   * Zero all matrix values and the RHS. The sparse backend keeps its
   * discovered pattern, so a per-frequency clear/restamp/solve cycle pays
   * the symbolic analysis once per sweep, not once per point.
   */
  clear(): void {
    this.system.clear();
  }

  /**
   * Accumulate the complex matrix entry Y[row][col] += re + j·im into the
   * bordered blocks. Ground rows/columns (index -1) are skipped exactly like
   * the transient stamps skip them — callers pass raw pinNode() results.
   * Exact-zero parts are not written so purely real stamps never enter the
   * imaginary coupling blocks' sparsity pattern; the stamped values are
   * frequency-dependent but their zero/non-zero split is not (reactive parts
   * scale with omega > 0), so the pattern stays sweep-invariant.
   */
  addAc(row: number, col: number, re: number, im: number): void {
    if (row < 0 || col < 0) return;
    const n = this.size;
    if (re !== 0) {
      this.system.add(row, col, re);
      this.system.add(row + n, col + n, re);
    }
    if (im !== 0) {
      this.system.add(row, col + n, -im);
      this.system.add(row + n, col, im);
    }
  }

  /** Accumulate the complex RHS entry b[row] += re + j·im (ground skipped). */
  addBAc(row: number, re: number, im: number): void {
    if (row < 0) return;
    if (re !== 0) this.system.addB(row, re);
    if (im !== 0) this.system.addB(row + this.size, im);
  }

  /**
   * Factor and solve the bordered system, returning the complex solution
   * split into real/imaginary parts of the original n-row layout plus the
   * backend's diagnostics for this factorization (singular systems report
   * through info, never as an exception — mirroring LinearSystem.solve()).
   */
  solveAc(): { re: Float64Array; im: Float64Array; info: MnaSolveInfo } {
    const x = this.system.solve();
    const n = this.size;
    const re = new Float64Array(n);
    const im = new Float64Array(n);
    for (let row = 0; row < n; row++) {
      re[row] = x[row] ?? 0;
      im[row] = x[row + n] ?? 0;
    }
    return { re, im, info: this.system.lastSolveInfo };
  }
}

/**
 * Stamp-only accumulation surface of an AcSystem — the AC counterpart of
 * MnaStampSurface. Device acStamp hooks are typed against this narrowed view
 * so the clear/solve lifecycle stays owned by the analysis driver.
 */
export type AcStampSurface = Pick<AcSystem, "size" | "addAc" | "addBAc">;

/**
 * Two-terminal complex admittance between nodes i and j — the AC analogue of
 * stampResistor's conductance pattern (diagonal +Y, off-diagonal -Y), with
 * the same ground handling via addAc's skip.
 */
export function stampAcAdmittance(
  ac: AcStampSurface,
  i: number,
  j: number,
  re: number,
  im: number,
): void {
  ac.addAc(i, i, re, im);
  ac.addAc(j, j, re, im);
  ac.addAc(i, j, -re, -im);
  ac.addAc(j, i, -re, -im);
}

/**
 * Ideal voltage-source branch for the AC system: the identical incidence
 * pattern stampVSource writes (KCL coupling on the node rows, KVL constraint
 * on branch row k) with a complex drive on the RHS. Independent sources are
 * AC-zeroed (reV = imV = 0) unless they are the analysis' designated input,
 * so the branch row still participates and the incidence matches the
 * transient system row-for-row.
 */
export function stampAcVoltageSourceRow(
  ac: AcStampSurface,
  i: number,
  j: number,
  k: number,
  reV: number,
  imV: number,
): void {
  ac.addAc(i, k, 1, 0);
  ac.addAc(k, i, 1, 0);
  ac.addAc(j, k, -1, 0);
  ac.addAc(k, j, -1, 0);
  ac.addBAc(k, reV, imV);
}
