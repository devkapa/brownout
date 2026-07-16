/**
 * Dense MNA (Modified Nodal Analysis) matrix + partial-pivot LU solver.
 *
 * Rows/columns 0..nodeCount-1 correspond to circuit nodes (ground excluded).
 * Rows/columns nodeCount..size-1 are extra variables for voltage sources.
 *
 * For R1 circuits (< 200 nodes) a dense solver is fast enough at 10 kHz.
 * Sparse LU can replace this later without changing the stamp API.
 */

export class MNA {
  readonly size: number;
  readonly G: Float64Array; // row-major size×size conductance matrix
  readonly b: Float64Array; // RHS current/voltage vector

  /** Diagnostics for the most recently factorised matrix. */
  lastSolveInfo: MnaSolveInfo = {
    singular: false,
    illConditioned: false,
    rank: 0,
    minScaledPivot: 0,
    relativeResidual: 0,
    nonFinite: false,
  };

  private factorization: MnaFactorization | null = null;
  private factorizedMatrix: Float64Array | null = null;
  private baseMatrix: Float64Array | null = null;
  private baseRhs: Float64Array | null = null;
  private matrixDirty = true;
  /** Monotonic diagnostic counter; never affects factorization validity. */
  factorizationCount = 0;
  factorizationReuseCount = 0;

  constructor(size: number) {
    this.size = size;
    this.G = new Float64Array(size * size);
    this.b = new Float64Array(size);
  }

  clear(): void {
    this.G.fill(0);
    this.b.fill(0);
    this.matrixDirty = true;
  }

  add(row: number, col: number, val: number): void {
    this.G[row * this.size + col] += val;
    this.matrixDirty = true;
  }

  addB(row: number, val: number): void {
    this.b[row] += val;
  }

  /** Save the current matrix/RHS as the immutable per-topology stamp base. */
  captureBase(): void {
    this.baseMatrix = new Float64Array(this.G);
    this.baseRhs = new Float64Array(this.b);
  }

  /** Restore the compiled base before adding dynamic/nonlinear overlays. */
  resetToBase(): void {
    if (this.baseMatrix && this.baseRhs) {
      this.G.set(this.baseMatrix);
      this.b.set(this.baseRhs);
    } else {
      this.G.fill(0);
      this.b.fill(0);
    }
    this.matrixDirty = true;
  }

  /**
   * Solve G·x = b using equilibrated LU with partial pivoting.
   *
   * MNA mixes conductance rows with ideal-source constraint rows, so raw
   * coefficients can differ by many orders of magnitude. Row and column
   * equilibration makes the pivot test relative to the matrix instead of an
   * arbitrary absolute unit. The factorisation is retained so small-signal
   * probe solves can reuse it without another O(n³) decomposition.
   */
  solve(): Float64Array {
    return this.solveRhs(this.b);
  }

  /** Solve G·x = rhs, reusing the latest factorisation when G is unchanged. */
  solveRhs(rhs: ArrayLike<number>): Float64Array {
    const n = this.size;
    if (n === 0) return new Float64Array(0);
    if (rhs.length !== n) {
      throw new RangeError(`MNA RHS length ${rhs.length} does not match matrix size ${n}`);
    }

    let factor = this.factorization;
    if (!factor || (this.matrixDirty && !this.matrixMatchesFactorized())) {
      factor = this.factor();
    } else if (this.matrixDirty) {
      // Stamping rebuilt the exact same numeric matrix. Different RHS values
      // are allowed; the residual is still checked against the current G.
      this.matrixDirty = false;
      this.factorizationReuseCount += 1;
    }
    const work = new Float64Array(n);

    // Dr·rhs, followed by the same row permutations used during LU.
    for (let row = 0; row < n; row++) {
      work[row] = Number(rhs[row] ?? 0) * factor.rowScale[row];
    }
    for (let col = 0; col < n; col++) {
      const pivotRow = factor.pivots[col];
      if (pivotRow !== col) {
        const tmp = work[col];
        work[col] = work[pivotRow];
        work[pivotRow] = tmp;
      }
    }

    // Forward substitution through unit-diagonal L.
    for (let row = 0; row < n; row++) {
      let sum = work[row];
      for (let col = 0; col < row; col++) {
        sum -= factor.lu[row * n + col] * work[col];
      }
      work[row] = sum;
    }

    // Back substitution through U. A singular row is deliberately returned as
    // zero, but unlike the former solver it is also surfaced in lastSolveInfo
    // so the engine can reject the step instead of presenting the value as real.
    const x = new Float64Array(n);
    for (let row = n - 1; row >= 0; row--) {
      let sum = work[row];
      for (let col = row + 1; col < n; col++) {
        sum -= factor.lu[row * n + col] * work[col];
      }
      const pivot = factor.lu[row * n + row];
      work[row] = Math.abs(pivot) <= factor.pivotTolerance ? 0 : sum / pivot;
      // x = Dc·y reverses the column equilibration.
      x[row] = factor.colScale[row] * work[row];
    }

    let maxMatrix = 0;
    let maxX = 0;
    let maxRhs = 0;
    let maxResidual = 0;
    let nonFinite = false;
    for (let col = 0; col < n; col++) {
      maxX = Math.max(maxX, Math.abs(x[col]));
      if (!Number.isFinite(x[col])) nonFinite = true;
    }
    for (let row = 0; row < n; row++) {
      let ax = 0;
      for (let col = 0; col < n; col++) {
        const a = this.G[row * n + col];
        maxMatrix = Math.max(maxMatrix, Math.abs(a));
        ax += a * x[col];
      }
      const rhsValue = Number(rhs[row] ?? 0);
      maxRhs = Math.max(maxRhs, Math.abs(rhsValue));
      maxResidual = Math.max(maxResidual, Math.abs(ax - rhsValue));
      if (!Number.isFinite(ax) || !Number.isFinite(rhsValue)) nonFinite = true;
    }
    const residualScale = maxMatrix * maxX + maxRhs;
    const relativeResidual = residualScale > 0 ? maxResidual / residualScale : maxResidual;
    if (!Number.isFinite(relativeResidual)) nonFinite = true;
    this.lastSolveInfo = {
      singular: factor.rank < n,
      illConditioned: factor.minScaledPivot > 0 && factor.minScaledPivot < 1e-12,
      rank: factor.rank,
      minScaledPivot: factor.minScaledPivot,
      relativeResidual,
      nonFinite,
    };

    return x;
  }

  private factor(): MnaFactorization {
    this.factorizationCount += 1;
    const n = this.size;
    const lu = new Float64Array(n * n);
    const rowScale = new Float64Array(n);
    const colScale = new Float64Array(n);

    // Row equilibration: Dr·G.
    for (let row = 0; row < n; row++) {
      let max = 0;
      for (let col = 0; col < n; col++) {
        max = Math.max(max, Math.abs(this.G[row * n + col]));
      }
      const scale = max > 0 && Number.isFinite(max) ? 1 / max : 1;
      rowScale[row] = scale;
      for (let col = 0; col < n; col++) {
        lu[row * n + col] = this.G[row * n + col] * scale;
      }
    }

    // Column equilibration: Dr·G·Dc.
    for (let col = 0; col < n; col++) {
      let max = 0;
      for (let row = 0; row < n; row++) {
        max = Math.max(max, Math.abs(lu[row * n + col]));
      }
      colScale[col] = max > 0 && Number.isFinite(max) ? 1 / max : 1;
      for (let row = 0; row < n; row++) {
        lu[row * n + col] *= colScale[col];
      }
    }

    const pivots = new Int32Array(n);
    // The equilibrated matrix has O(1) coefficients, making this a meaningful
    // relative threshold. It is intentionally well above denormal noise and
    // below the smallest pivot expected from a regularised circuit.
    // Equilibration makes exact rank loss collapse to a zero (or denormal)
    // pivot. Keep valid 1 pS regularisation pivots solvable, and report their
    // conditioning separately instead of treating every high-Z network as
    // algebraically singular.
    const pivotTolerance = Number.EPSILON / 2;
    let rank = 0;
    let minScaledPivot = Number.POSITIVE_INFINITY;

    for (let col = 0; col < n; col++) {
      let pivotRow = col;
      let maxVal = Math.abs(lu[col * n + col]);
      for (let row = col + 1; row < n; row++) {
        const value = Math.abs(lu[row * n + col]);
        if (value > maxVal) {
          maxVal = value;
          pivotRow = row;
        }
      }
      pivots[col] = pivotRow;

      if (pivotRow !== col) {
        for (let k = 0; k < n; k++) {
          const tmp = lu[col * n + k];
          lu[col * n + k] = lu[pivotRow * n + k];
          lu[pivotRow * n + k] = tmp;
        }
      }

      const pivot = lu[col * n + col];
      const absPivot = Math.abs(pivot);
      if (absPivot <= pivotTolerance || !Number.isFinite(absPivot)) continue;

      rank++;
      minScaledPivot = Math.min(minScaledPivot, absPivot);
      for (let row = col + 1; row < n; row++) {
        const multiplier = lu[row * n + col] / pivot;
        lu[row * n + col] = multiplier;
        for (let k = col + 1; k < n; k++) {
          lu[row * n + k] -= multiplier * lu[col * n + k];
        }
      }
    }

    const factorization: MnaFactorization = {
      lu,
      rowScale,
      colScale,
      pivots,
      pivotTolerance,
      rank,
      minScaledPivot: Number.isFinite(minScaledPivot) ? minScaledPivot : 0,
    };
    this.factorization = factorization;
    if (!this.factorizedMatrix || this.factorizedMatrix.length !== this.G.length) {
      this.factorizedMatrix = new Float64Array(this.G.length);
    }
    this.factorizedMatrix.set(this.G);
    this.matrixDirty = false;
    this.lastSolveInfo = {
      singular: rank < n,
      illConditioned: minScaledPivot < 1e-12,
      rank,
      minScaledPivot: Number.isFinite(minScaledPivot) ? minScaledPivot : 0,
      relativeResidual: 0,
      nonFinite: false,
    };
    return factorization;
  }

  /** Exact numeric guard: a stale LU can never be reused on a changed matrix. */
  private matrixMatchesFactorized(): boolean {
    const previous = this.factorizedMatrix;
    if (!previous || previous.length !== this.G.length) return false;
    for (let index = 0; index < this.G.length; index++) {
      const current = this.G[index];
      const cached = previous[index];
      if (current !== cached && !(Number.isNaN(current) && Number.isNaN(cached))) {
        return false;
      }
    }
    return true;
  }
}

export interface MnaSolveInfo {
  singular: boolean;
  illConditioned: boolean;
  rank: number;
  /** Smallest accepted pivot after row/column equilibration. */
  minScaledPivot: number;
  /** Backward error in the original, unscaled matrix units. */
  relativeResidual: number;
  nonFinite: boolean;
}

interface MnaFactorization {
  lu: Float64Array;
  rowScale: Float64Array;
  colScale: Float64Array;
  pivots: Int32Array;
  pivotTolerance: number;
  rank: number;
  minScaledPivot: number;
}
