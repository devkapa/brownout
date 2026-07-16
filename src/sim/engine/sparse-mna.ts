/**
 * Sparse MNA backend: pattern-persistent stamp slots compiled to CSC, a
 * quotient-graph approximate-minimum-degree column preorder (amd-ordering.ts),
 * and left-looking Gilbert-Peierls LU with threshold partial pivoting on the
 * equilibrated matrix.
 *
 * Observable semantics (lastSolveInfo fields, factorization counters,
 * base/overlay behavior, singular handling, RangeError message, residual
 * validation in original units) mirror the dense `MNA` class; the shared
 * contract lives in linear-system.ts. Numeric results may differ from dense
 * by roundoff because the elimination order differs — both backends validate
 * against the original unscaled matrix, so `relativeResidual` is the
 * cross-backend equivalence oracle.
 *
 * Value-only restamps (every Newton iteration) take a numeric-only
 * refactorization fast path: while the pattern is unchanged and the previous
 * factorization was clean (threshold pivoting, full rank, growth under the
 * limit), the stored pivot order and L/U column patterns are replayed with no
 * reachability DFS. Guards inside the replay (pivot tolerance, degradation
 * against the fresh column maximum, running element growth) abandon it and
 * rerun the full pass with fresh pivoting, so the fast path can only trade
 * time, never stability. The AMD preorder replaced an earlier greedy
 * minimum-degree ordering that materialised fill edges explicitly — quadratic
 * around hub vertices and dependent on work budgets; the quotient graph
 * represents cliques implicitly, which handles hub and expander-like patterns
 * (the random long-range chords in benchmark-linear-scale.ts) without budgets.
 */

import { amdOrder } from "./amd-ordering.js";
import type { LinearSystem } from "./linear-system.js";
import type { MnaSolveInfo } from "./mna.js";

/**
 * Same acceptance threshold as the dense backend: equilibration normalises
 * coefficients to O(1), so anything at or below half an ulp of 1 is exact
 * rank loss rather than a small-but-real conductance.
 */
const PIVOT_TOLERANCE = Number.EPSILON / 2;

/**
 * SPICE-style threshold pivoting: keep the structural diagonal whenever it is
 * within 1e-3 of the column's best candidate. MNA matrices are mostly
 * diagonally dominant, and preferring the diagonal preserves sparsity; the
 * escape to the max row is what makes voltage-source branch rows (structural
 * zero diagonal) factorable at all.
 */
const DIAGONAL_PREFERENCE = 1e-3;

/**
 * Element-growth bound for the threshold-pivoted factorization. Threshold
 * pivoting allows per-step multipliers up to 1/DIAGONAL_PREFERENCE, which
 * compounds exponentially on adversarial chains while rank, minScaledPivot
 * and the singular flag all still read healthy — the factors themselves are
 * the only honest signal. Equilibration normalises the input to O(1), so the
 * largest intermediate magnitude IS the classical growth factor; backward
 * error scales with eps*growth and the engine accepts steps at
 * relativeResidual <= 1e-8, so 1e6 leaves two decades of margin while
 * healthy MNA factorizations (growth of order 1 to 100) never trip it.
 */
const GROWTH_LIMIT = 1e6;

/**
 * Refactorization guard: abandon the replay when the stored pivot's
 * magnitude falls below this fraction of the fresh column maximum.
 * Threshold pivoting accepted the pivot at >= 1e-3 of its column max when
 * the order was elected; drifting three more decades means the stale order
 * now keeps a pivot fresh pivoting would reject, and the implied multiplier
 * bound (columnMax / pivot up to 1e6) is about to consume the entire
 * backward-error margin the growth limit protects.
 */
const REFACTOR_PIVOT_DEGRADATION = 1e-6;

/**
 * The engine rejects any step whose validated relativeResidual exceeds this
 * (sim-engine's linear-solve health gate). Reused here as feedback into
 * replay eligibility: the in-replay guards sample only end-of-column
 * magnitudes, so a stale pivot order could in principle pass every guard
 * through cancelling intermediate products yet still yield a solution that
 * fails validation. The guards that admitted that order once would admit it
 * on every later restamp, so a failing residual must retire the stored
 * order — otherwise the warm path has no escape short of a pattern rebuild.
 */
const REFACTOR_RESIDUAL_LIMIT = 1e-8;

/** Grow an Int32Array to at least `needed` entries, doubling to amortise. */
function growInt32(arr: Int32Array, needed: number): Int32Array {
  if (arr.length >= needed) return arr;
  const next = new Int32Array(Math.max(arr.length * 2, needed, 256));
  next.set(arr);
  return next;
}

/** Grow a Float64Array to at least `needed` entries, doubling to amortise. */
function growFloat64(arr: Float64Array, needed: number): Float64Array {
  if (arr.length >= needed) return arr;
  const next = new Float64Array(Math.max(arr.length * 2, needed, 256));
  next.set(arr);
  return next;
}

export class SparseMNA implements LinearSystem {
  readonly size: number;
  readonly backend = "sparse" as const;

  /** Diagnostics for the most recently factorised matrix. */
  lastSolveInfo: MnaSolveInfo = {
    singular: false,
    illConditioned: false,
    rank: 0,
    minScaledPivot: 0,
    relativeResidual: 0,
    nonFinite: false,
  };

  /** Monotonic diagnostic counters; never affect factorization validity. */
  factorizationCount = 0;
  factorizationReuseCount = 0;
  /**
   * Numeric-only refactorizations completed — always a subset of
   * factorizationCount, which keeps counting every numeric pass either way.
   * SparseMNA-only diagnostic, deliberately outside the LinearSystem
   * contract: the dense backend has no equivalent fast path, and backends
   * may expose extras beyond the shared interface (MNA does the same).
   */
  refactorizationCount = 0;

  // --- Stamp storage -------------------------------------------------------
  // Slots are append-only and never reorder, so `captureBase` can identify
  // the base region by a single count and any symbolic artefact keyed by slot
  // index survives value-only restamps. The Map key row*size+col is exact in
  // a double because size stays far below 2^26.
  private readonly slotIndex = new Map<number, number>();
  // Growable fields carry the general typed-array type: the grow helpers
  // return Int32Array<ArrayBufferLike>, which TS 5.7+ will not narrow back
  // to the ArrayBuffer-specific type these initializers would otherwise infer.
  private slotRow: Int32Array = new Int32Array(256);
  private slotCol: Int32Array = new Int32Array(256);
  private values: Float64Array = new Float64Array(256);
  private slotCount = 0;
  private readonly b: Float64Array;

  /** True when a slot was appended since the last symbolic analysis. */
  private patternDirty = true;
  /** True when any value/clear/reset touched the matrix since factorization. */
  private matrixDirty = true;

  // --- Base/overlay snapshot ----------------------------------------------
  private baseSlotCount = -1;
  private baseValues: Float64Array | null = null;
  private baseRhs: Float64Array | null = null;

  // --- Symbolic analysis (rebuilt only when patternDirty) ------------------
  private readonly colPtr: Int32Array;
  private cscRowIdx: Int32Array = new Int32Array(0);
  /** slot -> CSC position, so each factorization scatters values in O(nnz). */
  private slotToCsc: Int32Array = new Int32Array(0);
  private cscVal: Float64Array = new Float64Array(0);
  /** colOrder[k] = original column eliminated at step k (bijective). */
  private readonly colOrder: Int32Array;

  // --- Numeric factorization state ------------------------------------------
  private hasFactorization = false;
  /**
   * True when the stored L/U came from a clean threshold-pivoted pass (full
   * rank, growth under the limit, no fallback) over the current symbolic
   * pattern — the preconditions for replaying its pivot order on new values.
   */
  private refactorEligible = false;
  private readonly rowScale: Float64Array;
  private readonly colScale: Float64Array;
  /** prow[k] = original row pivotal at step k, or -1 for a singular step. */
  private readonly prow: Int32Array;
  /** pinv[row] = pivot step of an original row during factorization, else -1. */
  private readonly pinv: Int32Array;
  private readonly lColPtr: Int32Array;
  private lRows: Int32Array = new Int32Array(256);
  private lVals: Float64Array = new Float64Array(256);
  private lCount = 0;
  private readonly uColPtr: Int32Array;
  private uRows: Int32Array = new Int32Array(256);
  private uVals: Float64Array = new Float64Array(256);
  private uCount = 0;
  private readonly uDiag: Float64Array;
  private fRank = 0;
  private fMinScaledPivot = 0;
  /** Exact numeric guard: a stale LU can never be reused on a changed matrix. */
  private lastFactorizedValues: Float64Array | null = null;

  // --- Reusable scratch (allocated once; no per-solve allocation) ----------
  private readonly mark: Int32Array;
  private markGen = 0;
  private readonly dfsNode: Int32Array;
  private readonly dfsEdge: Int32Array;
  private readonly reach: Int32Array;
  private readonly xWork: Float64Array;
  private readonly solveW: Float64Array;
  private readonly solveY: Float64Array;
  private readonly axScratch: Float64Array;

  constructor(size: number) {
    this.size = size;
    this.b = new Float64Array(size);
    this.colPtr = new Int32Array(size + 1);
    this.colOrder = new Int32Array(size);
    this.rowScale = new Float64Array(size);
    this.colScale = new Float64Array(size);
    this.prow = new Int32Array(size);
    this.pinv = new Int32Array(size);
    this.lColPtr = new Int32Array(size + 1);
    this.uColPtr = new Int32Array(size + 1);
    this.uDiag = new Float64Array(size);
    this.mark = new Int32Array(size);
    this.dfsNode = new Int32Array(size);
    this.dfsEdge = new Int32Array(size);
    this.reach = new Int32Array(size);
    this.xWork = new Float64Array(size);
    this.solveW = new Float64Array(size);
    this.solveY = new Float64Array(size);
    this.axScratch = new Float64Array(size);
  }

  /**
   * Zero values and RHS but KEEP the discovered pattern: dropping it would
   * rebuild the symbolic analysis every Newton iteration even though the
   * restamped pattern is identical.
   */
  clear(): void {
    this.values.fill(0, 0, this.slotCount);
    this.b.fill(0);
    this.matrixDirty = true;
  }

  add(row: number, col: number, val: number): void {
    // Positive-form bounds test: NaN indexes fail every comparison and are
    // dropped, where the negated form would let them through to mint a
    // phantom slot at row 0 that splits one matrix cell across duplicate
    // CSC positions. Out-of-range stamps are dropped rather than mirroring
    // dense, whose flat-array write silently aliases another cell whenever
    // row*size+col happens to land in bounds — a latent bug this backend
    // deliberately does not emulate. Stamps guard the ground index (-1)
    // themselves.
    if (!(row >= 0 && row < this.size && col >= 0 && col < this.size)) return;
    // Truncate before keying so the Map key always agrees with the Int32
    // slot arrays: a fractional index must not mint a second slot for a
    // cell the truncated coordinates already own.
    const r = row | 0;
    const c = col | 0;
    const key = r * this.size + c;
    const slot = this.slotIndex.get(key);
    if (slot !== undefined) {
      this.values[slot] += val;
    } else {
      const s = this.slotCount;
      this.slotRow = growInt32(this.slotRow, s + 1);
      this.slotCol = growInt32(this.slotCol, s + 1);
      this.values = growFloat64(this.values, s + 1);
      this.slotRow[s] = r;
      this.slotCol[s] = c;
      this.values[s] = val;
      this.slotIndex.set(key, s);
      this.slotCount = s + 1;
      this.patternDirty = true;
    }
    this.matrixDirty = true;
  }

  addB(row: number, val: number): void {
    if (row < 0 || row >= this.size) return;
    this.b[row] += val;
  }

  /** Save the current matrix/RHS as the immutable per-topology stamp base. */
  captureBase(): void {
    this.baseSlotCount = this.slotCount;
    this.baseValues = this.values.slice(0, this.slotCount);
    this.baseRhs = new Float64Array(this.b);
  }

  /** Append-only slot watermark; see LinearSystem.patternCheckpoint. */
  patternCheckpoint(): number {
    return this.slotCount;
  }

  /**
   * Drop every slot appended after the checkpoint. Exact because slots are
   * append-only and never reorder: the trailing region is precisely what the
   * exploratory sequence discovered. The symbolic analysis and any stored
   * factorization refer to the enlarged pattern, so both are invalidated;
   * the next solve rebuilds them over the restored pattern and every later
   * factorization is bit-identical to a backend that never saw the dropped
   * positions.
   */
  restorePatternCheckpoint(checkpoint: number): void {
    // Never truncate into the captured base region: resetToBase() rebuilds
    // values from baseSlotCount, so cutting below it would corrupt the base.
    const floor = Math.max(this.baseSlotCount, 0);
    if (checkpoint < floor || checkpoint >= this.slotCount) return;
    for (let slot = checkpoint; slot < this.slotCount; slot++) {
      this.slotIndex.delete(this.slotRow[slot] * this.size + this.slotCol[slot]);
    }
    this.slotCount = checkpoint;
    this.patternDirty = true;
    this.matrixDirty = true;
    this.hasFactorization = false;
    this.refactorEligible = false;
    this.lastFactorizedValues = null;
  }

  /** Drop pivot-replay and exact-values reuse memory; see LinearSystem. */
  invalidateFactorization(): void {
    this.hasFactorization = false;
    this.refactorEligible = false;
    this.lastFactorizedValues = null;
    this.matrixDirty = true;
  }

  /**
   * Restore the compiled base before adding dynamic/nonlinear overlays.
   *
   * Overlay slots appended after `captureBase` stay in the pattern with a
   * zero value: the pattern is append-only, so the base region is exactly
   * the first `baseSlotCount` slots, and keeping the overlay slots means the
   * per-iteration restamp hits existing slots instead of re-growing.
   */
  resetToBase(): void {
    if (this.baseValues && this.baseRhs && this.baseSlotCount >= 0) {
      this.values.set(this.baseValues);
      this.values.fill(0, this.baseSlotCount, this.slotCount);
      this.b.set(this.baseRhs);
    } else {
      this.values.fill(0, 0, this.slotCount);
      this.b.fill(0);
    }
    this.matrixDirty = true;
  }

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

    if (!this.hasFactorization || (this.matrixDirty && !this.matrixMatchesFactorized())) {
      this.factorize();
    } else if (this.matrixDirty) {
      // Stamping rebuilt the exact same numeric matrix. Different RHS values
      // are allowed; the residual is still checked against the current values.
      this.matrixDirty = false;
      this.factorizationReuseCount += 1;
    }

    // Row equilibration of the RHS. The row permutation is folded into the
    // forward pass below, which reads the pivot row of each step directly.
    const w = this.solveW;
    for (let row = 0; row < n; row++) {
      w[row] = Number(rhs[row] ?? 0) * this.rowScale[row];
    }

    // Forward substitution through unit-diagonal L, processed in pivot-step
    // order but indexed by original rows. L multipliers keep their original
    // row indices: remapping rows to step coordinates is unsafe when singular
    // steps leave rows permanently unpivoted, while this formulation replays
    // the exact update order used during factorization.
    const y = this.solveY;
    const lColPtr = this.lColPtr;
    const lRows = this.lRows;
    const lVals = this.lVals;
    for (let k = 0; k < n; k++) {
      const pivotRow = this.prow[k];
      if (pivotRow < 0) {
        // Singular step: no pivot was assigned; back substitution forces the
        // variable to zero, so the intermediate value is irrelevant.
        y[k] = 0;
        continue;
      }
      const value = w[pivotRow];
      y[k] = value;
      for (let p = lColPtr[k]; p < lColPtr[k + 1]; p++) {
        w[lRows[p]] -= lVals[p] * value;
      }
    }

    // Column-oriented back substitution through U in pivot-step coordinates.
    // A singular pivot is deliberately returned as zero, mirroring dense; it
    // is surfaced through lastSolveInfo so the engine can reject the step.
    const x = new Float64Array(n);
    const uColPtr = this.uColPtr;
    const uRows = this.uRows;
    const uVals = this.uVals;
    for (let k = n - 1; k >= 0; k--) {
      const pivot = this.uDiag[k];
      const z = Math.abs(pivot) <= PIVOT_TOLERANCE ? 0 : y[k] / pivot;
      y[k] = z;
      for (let p = uColPtr[k]; p < uColPtr[k + 1]; p++) {
        y[uRows[p]] -= uVals[p] * z;
      }
      // x = Dc·z undoes the column equilibration and the column preorder.
      const originalCol = this.colOrder[k];
      x[originalCol] = this.colScale[originalCol] * z;
    }

    // Residual validation in original, unscaled units over the slot arrays:
    // O(nnz) replacing the dense O(n^2) sweep, same formula and zero guard.
    const ax = this.axScratch;
    ax.fill(0);
    let maxMatrix = 0;
    let maxX = 0;
    let maxRhs = 0;
    let maxResidual = 0;
    let nonFinite = false;
    for (let col = 0; col < n; col++) {
      maxX = Math.max(maxX, Math.abs(x[col]));
      if (!Number.isFinite(x[col])) nonFinite = true;
    }
    for (let s = 0; s < this.slotCount; s++) {
      const a = this.values[s];
      maxMatrix = Math.max(maxMatrix, Math.abs(a));
      ax[this.slotRow[s]] += a * x[this.slotCol[s]];
    }
    for (let row = 0; row < n; row++) {
      const rhsValue = Number(rhs[row] ?? 0);
      maxRhs = Math.max(maxRhs, Math.abs(rhsValue));
      maxResidual = Math.max(maxResidual, Math.abs(ax[row] - rhsValue));
      if (!Number.isFinite(ax[row]) || !Number.isFinite(rhsValue)) nonFinite = true;
    }
    const residualScale = maxMatrix * maxX + maxRhs;
    const relativeResidual = residualScale > 0 ? maxResidual / residualScale : maxResidual;
    if (!Number.isFinite(relativeResidual)) nonFinite = true;
    this.lastSolveInfo = {
      singular: this.fRank < n,
      illConditioned: this.fMinScaledPivot > 0 && this.fMinScaledPivot < 1e-12,
      rank: this.fRank,
      minScaledPivot: this.fMinScaledPivot,
      relativeResidual,
      nonFinite,
    };

    // Residual backstop feeding the fast path (see REFACTOR_RESIDUAL_LIMIT):
    // a factorization whose validated solution fails the engine's acceptance
    // gate must not have its pivot order replayed again. Negated comparison
    // so a NaN residual (nonFinite poisoning) also retires the order.
    if (!(relativeResidual <= REFACTOR_RESIDUAL_LIMIT)) {
      this.refactorEligible = false;
    }

    return x;
  }

  /**
   * Element-wise value comparison against the matrix that was factorized.
   * NaN is treated as matching NaN so a NaN-poisoned matrix does not force an
   * endless refactorization loop, exactly like the dense guard. Slots
   * appended since the factorization are numerically neutral while they hold
   * an exact zero, so the LU stays valid and the reuse decision matches
   * dense (which sees an unchanged numeric matrix); the symbolic rebuild is
   * deferred until one of them actually carries a value.
   */
  private matrixMatchesFactorized(): boolean {
    const previous = this.lastFactorizedValues;
    if (!previous || previous.length > this.slotCount) return false;
    for (let s = previous.length; s < this.slotCount; s++) {
      if (this.values[s] !== 0) return false;
    }
    for (let s = 0; s < previous.length; s++) {
      const current = this.values[s];
      const cached = previous[s];
      if (current !== cached && !(Number.isNaN(current) && Number.isNaN(cached))) {
        return false;
      }
    }
    return true;
  }

  /**
   * Compile the slot pattern to CSC and compute a fill-reducing column
   * preorder. Runs only when a slot was appended since the last analysis;
   * value-only restamps (every Newton iteration) skip it entirely.
   */
  private rebuildSymbolic(): void {
    const n = this.size;
    const nnz = this.slotCount;
    const slotRow = this.slotRow;
    const slotCol = this.slotCol;

    // CSC construction via counting sort. Rows within a column stay in slot
    // order: Gilbert-Peierls scatters through a dense accumulator, so sorted
    // rows buy nothing.
    const colPtr = this.colPtr;
    colPtr.fill(0);
    for (let s = 0; s < nnz; s++) colPtr[slotCol[s] + 1] += 1;
    for (let col = 0; col < n; col++) colPtr[col + 1] += colPtr[col];
    if (this.cscRowIdx.length < nnz) {
      // Doubling headroom: single-slot pattern growth is common right after
      // a topology edit, and exact-size reallocation would churn on each one.
      const capacity = Math.max(nnz, this.cscRowIdx.length * 2, 256);
      this.cscRowIdx = new Int32Array(capacity);
      this.slotToCsc = new Int32Array(capacity);
      this.cscVal = new Float64Array(capacity);
    }
    const cursor = new Int32Array(n);
    for (let col = 0; col < n; col++) cursor[col] = colPtr[col];
    for (let s = 0; s < nnz; s++) {
      const pos = cursor[slotCol[s]]++;
      this.cscRowIdx[pos] = slotRow[s];
      this.slotToCsc[s] = pos;
    }

    this.computeColumnOrder();

    // Debug guard: a non-bijective order would silently scramble the
    // factorization coordinate system; O(n) is negligible next to the rest
    // of the symbolic work, so the check stays on permanently.
    const seen = new Uint8Array(n);
    for (let k = 0; k < n; k++) {
      const col = this.colOrder[k];
      if (col < 0 || col >= n || seen[col] !== 0) {
        throw new Error("SparseMNA: column ordering is not a permutation");
      }
      seen[col] = 1;
    }

    this.patternDirty = false;
    // Stored L/U patterns are keyed to the previous symbolic analysis; a
    // fresh analysis invalidates the refactorization replay.
    this.refactorEligible = false;
  }

  /**
   * Fill-reducing column preorder: build the deduplicated symmetric pattern
   * of A + A^T without self-edges and delegate to the quotient-graph AMD
   * ordering. The symmetrization matters because Gilbert-Peierls pivots by
   * row within the preordered column — an order computed on the unsymmetric
   * pattern would ignore half the fill paths threshold pivoting can take.
   */
  private computeColumnOrder(): void {
    const n = this.size;
    const nnz = this.slotCount;
    if (n === 0) return;

    // Raw symmetric degree count (duplicates included) sizes the CSR once.
    const rawDeg = new Int32Array(n);
    for (let s = 0; s < nnz; s++) {
      const r = this.slotRow[s];
      const c = this.slotCol[s];
      if (r !== c) {
        rawDeg[r] += 1;
        rawDeg[c] += 1;
      }
    }
    const rawPtr = new Int32Array(n + 1);
    for (let v = 0; v < n; v++) rawPtr[v + 1] = rawPtr[v] + rawDeg[v];
    const rawIdx = new Int32Array(rawPtr[n]);
    const cursor = new Int32Array(n);
    for (let v = 0; v < n; v++) cursor[v] = rawPtr[v];
    for (let s = 0; s < nnz; s++) {
      const r = this.slotRow[s];
      const c = this.slotCol[s];
      if (r !== c) {
        rawIdx[cursor[r]++] = c;
        rawIdx[cursor[c]++] = r;
      }
    }

    // Deduplicate per vertex with a stamp; v + 1 is a unique generation per
    // row of this single pass, so the stamp array needs no clearing.
    const adjPtr = new Int32Array(n + 1);
    const adjIdx = new Int32Array(rawPtr[n]);
    const stamp = new Int32Array(n);
    let out = 0;
    for (let v = 0; v < n; v++) {
      adjPtr[v] = out;
      const g = v + 1;
      for (let t = rawPtr[v]; t < rawPtr[v + 1]; t++) {
        const w = rawIdx[t];
        if (stamp[w] !== g) {
          stamp[w] = g;
          adjIdx[out++] = w;
        }
      }
    }
    adjPtr[n] = out;

    this.colOrder.set(amdOrder(n, adjPtr, adjIdx));
  }

  /** Bump the DFS visit generation, recycling marks before Int32 overflow. */
  private nextMarkGen(): number {
    this.markGen += 1;
    if (this.markGen === 0x7fffffff) {
      this.mark.fill(0);
      this.markGen = 1;
    }
    return this.markGen;
  }

  /**
   * Depth-first reachability over the partially built L from one root of
   * A(:,j), appending finished vertices topologically into `reach` from
   * `reachTop` downward. Explicit stacks: recursion would overflow on long
   * dependency chains at n around 50k.
   */
  private dfsReach(root: number, gen: number, reachTop: number): number {
    const mark = this.mark;
    const pinv = this.pinv;
    const lColPtr = this.lColPtr;
    const lRows = this.lRows;
    const nodeStack = this.dfsNode;
    const edgeStack = this.dfsEdge;
    const reach = this.reach;
    let head = 0;
    nodeStack[0] = root;
    mark[root] = gen;
    edgeStack[0] = pinv[root] >= 0 ? lColPtr[pinv[root]] : 0;
    while (head >= 0) {
      const node = nodeStack[head];
      const step = pinv[node];
      const end = step >= 0 ? lColPtr[step + 1] : 0;
      let p = edgeStack[head];
      let descended = false;
      while (p < end) {
        const child = lRows[p];
        p += 1;
        if (mark[child] !== gen) {
          edgeStack[head] = p;
          head += 1;
          nodeStack[head] = child;
          mark[child] = gen;
          edgeStack[head] = pinv[child] >= 0 ? lColPtr[pinv[child]] : 0;
          descended = true;
          break;
        }
      }
      if (descended) continue;
      reachTop -= 1;
      reach[reachTop] = node;
      head -= 1;
    }
    return reachTop;
  }

  /**
   * Numeric factorization: equilibrate exactly like the dense backend, then
   * either replay the stored pivot order on the new values (refactorization
   * fast path) or run the threshold-pivoted elimination, rerunning it with
   * pure partial pivoting when element growth shows the diagonal preference
   * destabilised the factors.
   */
  private factorize(): void {
    this.factorizationCount += 1;
    if (this.patternDirty) this.rebuildSymbolic();
    const n = this.size;
    const nnz = this.slotCount;
    const values = this.values;

    this.equilibrateAndScatter();

    if (this.refactorEligible && this.refactor()) {
      // refactorEligible stays true: the replay reproduced a clean
      // threshold-pivoted full-rank factorization, so the next value-only
      // restamp may replay again.
      this.refactorizationCount += 1;
    } else {
      // The retry is the growth-triggered fallback the header describes: on
      // adversarial patterns the diagonal preference compounds multipliers
      // exponentially while every per-step flag still reads healthy, and only
      // the factors themselves reveal it. Pure partial pivoting restores
      // backward stability at the cost of extra fill.
      const growth = this.eliminate(true);
      if (Number.isFinite(growth) && growth > GROWTH_LIMIT) {
        this.eliminate(false);
        // The stored factors now come from pure partial pivoting; replaying
        // them under the threshold-path guards would mix pivot policies.
        this.refactorEligible = false;
      } else {
        // NaN growth (a poisoned matrix) also disqualifies the replay: the
        // guards inside refactor() cannot see growth that never compares.
        this.refactorEligible = this.fRank === n && Number.isFinite(growth);
      }
    }

    this.hasFactorization = true;
    if (!this.lastFactorizedValues || this.lastFactorizedValues.length !== nnz) {
      this.lastFactorizedValues = new Float64Array(nnz);
    }
    this.lastFactorizedValues.set(values.subarray(0, nnz));
    this.matrixDirty = false;
    this.lastSolveInfo = {
      singular: this.fRank < n,
      illConditioned: this.fMinScaledPivot > 0 && this.fMinScaledPivot < 1e-12,
      rank: this.fRank,
      minScaledPivot: this.fMinScaledPivot,
      relativeResidual: 0,
      nonFinite: false,
    };
  }

  /**
   * Shared prologue of every numeric pass: recompute the equilibration
   * scales for the current values and scatter the equilibrated matrix into
   * the symbolic CSC positions. Split out because the refactorization fast
   * path needs fresh scales too — the values changed, so reusing stale
   * scales would silently change which pivots the guards compare against.
   */
  private equilibrateAndScatter(): void {
    const n = this.size;
    const nnz = this.slotCount;
    const values = this.values;
    const slotRow = this.slotRow;
    const slotCol = this.slotCol;

    // Row equilibration: Dr·G. Zero or non-finite row maxima fall back to a
    // unit scale so a broken row poisons only itself, mirroring dense. A
    // denormal-only maximum overflows 1/max to Infinity, which would poison
    // every downstream product, so the reciprocal is clamped finite; the
    // clamp is unreachable for physical conductances and only differs from
    // dense (which lets the Infinity through) below 1e-308 stamp magnitudes.
    const rowScale = this.rowScale;
    rowScale.fill(0);
    for (let s = 0; s < nnz; s++) {
      const row = slotRow[s];
      rowScale[row] = Math.max(rowScale[row], Math.abs(values[s]));
    }
    for (let row = 0; row < n; row++) {
      const max = rowScale[row];
      const inv = 1 / max;
      rowScale[row] =
        max > 0 && Number.isFinite(max) ? (Number.isFinite(inv) ? inv : Number.MAX_VALUE) : 1;
    }

    // Column equilibration: Dr·G·Dc, computed on the row-scaled values.
    const colScale = this.colScale;
    colScale.fill(0);
    for (let s = 0; s < nnz; s++) {
      const col = slotCol[s];
      colScale[col] = Math.max(colScale[col], Math.abs(values[s] * rowScale[slotRow[s]]));
    }
    for (let col = 0; col < n; col++) {
      const max = colScale[col];
      const inv = 1 / max;
      colScale[col] =
        max > 0 && Number.isFinite(max) ? (Number.isFinite(inv) ? inv : Number.MAX_VALUE) : 1;
    }

    // Scatter equilibrated values into CSC positions computed symbolically.
    const cscVal = this.cscVal;
    for (let s = 0; s < nnz; s++) {
      cscVal[this.slotToCsc[s]] = values[s] * rowScale[slotRow[s]] * colScale[slotCol[s]];
    }
  }

  /**
   * Numeric-only refactorization: replay the stored pivot sequence over the
   * stored L/U column patterns, recomputing values with no reachability DFS
   * and no pivot search. Valid because the update order stored in U per
   * column is exactly the topological order the discovery DFS produced, so
   * replaying it on the same pattern performs the same arithmetic as the
   * full pass would with the same pivot choices — column k only consumes L
   * columns of earlier steps, which this loop has already rewritten.
   *
   * Returns false to abandon (caller reruns the full pass with fresh
   * pivoting; the arrays are fully rewritten there, so a partial replay
   * leaves no residue). Abandon triggers:
   * - pivot at or below PIVOT_TOLERANCE, or non-finite: the step would be
   *   singular under the stored order even though the matrix may not be;
   * - pivot below REFACTOR_PIVOT_DEGRADATION of the fresh column maximum:
   *   value drift demoted the stored pivot relative to what fresh partial
   *   pivoting would now elect;
   * - running element growth above GROWTH_LIMIT: same honest-signal bound
   *   as the full pass, checked online so a destabilised replay stops
   *   before wasting the remaining columns.
   */
  private refactor(): boolean {
    const n = this.size;
    const colPtr = this.colPtr;
    const cscRowIdx = this.cscRowIdx;
    const cscVal = this.cscVal;
    const lColPtr = this.lColPtr;
    const lRows = this.lRows;
    const lVals = this.lVals;
    const uColPtr = this.uColPtr;
    const uRows = this.uRows;
    const uVals = this.uVals;
    const prow = this.prow;
    const xWork = this.xWork;
    let minScaledPivot = Number.POSITIVE_INFINITY;
    let growth = 0;

    for (let k = 0; k < n; k++) {
      const j = this.colOrder[k];
      const uStart = uColPtr[k];
      const uEnd = uColPtr[k + 1];
      const lStart = lColPtr[k];
      const lEnd = lColPtr[k + 1];
      // Eligibility guarantees the stored factorization was full-rank, so
      // every step has a pivot row.
      const pivotRow = prow[k];

      // Zero the stored column pattern, then scatter A(:,j) over it; the
      // pattern is a superset of A(:,j)'s rows by construction, so stale
      // values from the previous column can never leak in.
      for (let q = uStart; q < uEnd; q++) xWork[prow[uRows[q]]] = 0;
      xWork[pivotRow] = 0;
      for (let q = lStart; q < lEnd; q++) xWork[lRows[q]] = 0;
      for (let t = colPtr[j]; t < colPtr[j + 1]; t++) xWork[cscRowIdx[t]] = cscVal[t];

      // Replay the pending-column updates in stored order. U rows are step
      // indices whose original rows finished updating earlier in this same
      // order, so each xWork read is final when harvested.
      for (let q = uStart; q < uEnd; q++) {
        const step = uRows[q];
        const value = xWork[prow[step]];
        uVals[q] = value;
        const magnitude = Math.abs(value);
        if (magnitude > growth) growth = magnitude;
        for (let t = lColPtr[step]; t < lColPtr[step + 1]; t++) {
          xWork[lRows[t]] -= lVals[t] * value;
        }
      }

      const pivotValue = xWork[pivotRow];
      const absPivot = Math.abs(pivotValue);
      // Fresh partial pivoting would choose the largest not-yet-pivotal
      // candidate: exactly the pivot row plus this column's L rows.
      let columnMax = absPivot;
      for (let q = lStart; q < lEnd; q++) {
        const a = Math.abs(xWork[lRows[q]]);
        if (a > columnMax) columnMax = a;
      }
      if (columnMax > growth) growth = columnMax;
      if (!(absPivot > PIVOT_TOLERANCE) || !Number.isFinite(absPivot)) return false;
      if (absPivot < REFACTOR_PIVOT_DEGRADATION * columnMax) return false;
      if (growth > GROWTH_LIMIT) return false;

      if (absPivot < minScaledPivot) minScaledPivot = absPivot;
      this.uDiag[k] = pivotValue;
      for (let q = lStart; q < lEnd; q++) {
        lVals[q] = xWork[lRows[q]] / pivotValue;
      }
    }

    // Rank/minScaledPivot bookkeeping identical to eliminate(): every step
    // above accepted a finite pivot beyond the tolerance.
    this.fRank = n;
    this.fMinScaledPivot = Number.isFinite(minScaledPivot) ? minScaledPivot : 0;
    return true;
  }

  /**
   * One left-looking Gilbert-Peierls pass over the preordered columns of the
   * equilibrated matrix in cscVal. Every rejected pivot marks a singular
   * step whose variable back-substitutes to zero; accepted pivots drive rank
   * and minScaledPivot like dense. Returns the largest intermediate column
   * magnitude — the classical element-growth factor, since equilibration
   * normalises the input to O(1) — so the caller can detect when threshold
   * pivoting destabilised the factors. With `preferDiagonal` false the pivot
   * is always the largest reachable candidate (pure partial pivoting).
   */
  private eliminate(preferDiagonal: boolean): number {
    const n = this.size;
    const cscVal = this.cscVal;
    const colPtr = this.colPtr;
    const cscRowIdx = this.cscRowIdx;
    const pinv = this.pinv;
    const prow = this.prow;
    const reach = this.reach;
    const xWork = this.xWork;
    pinv.fill(-1);
    prow.fill(-1);
    xWork.fill(0);
    this.lCount = 0;
    this.uCount = 0;
    this.lColPtr[0] = 0;
    this.uColPtr[0] = 0;
    let rank = 0;
    let minScaledPivot = Number.POSITIVE_INFINITY;
    let growth = 0;

    for (let k = 0; k < n; k++) {
      const j = this.colOrder[k];
      const colStart = colPtr[j];
      const colEnd = colPtr[j + 1];

      // Nonzero pattern of L \ A(:,j) via reachability over L's graph.
      const gen = this.nextMarkGen();
      let reachTop = n;
      for (let p = colStart; p < colEnd; p++) {
        const row = cscRowIdx[p];
        if (this.mark[row] !== gen) reachTop = this.dfsReach(row, gen, reachTop);
      }

      // Scatter A(:,j), then apply pending column updates in topological
      // order. Updates run unconditionally (no zero-skip) so NaN and
      // Infinity propagate exactly as they would in the dense kernel.
      for (let idx = reachTop; idx < n; idx++) xWork[reach[idx]] = 0;
      for (let p = colStart; p < colEnd; p++) xWork[cscRowIdx[p]] = cscVal[p];
      const lColPtr = this.lColPtr;
      let lRows = this.lRows;
      let lVals = this.lVals;
      for (let idx = reachTop; idx < n; idx++) {
        const node = reach[idx];
        const step = pinv[node];
        if (step < 0) continue;
        const xi = xWork[node];
        for (let p = lColPtr[step]; p < lColPtr[step + 1]; p++) {
          xWork[lRows[p]] -= lVals[p] * xi;
        }
      }

      // Threshold partial pivoting among reachable rows not yet pivotal.
      // NaN candidates lose every comparison, so a NaN-only column falls
      // through to the singular path instead of electing a NaN pivot.
      let maxAbs = 0;
      let maxRow = -1;
      for (let idx = reachTop; idx < n; idx++) {
        const row = reach[idx];
        if (pinv[row] >= 0) continue;
        const a = Math.abs(xWork[row]);
        if (a > maxAbs) {
          maxAbs = a;
          maxRow = row;
        }
      }
      let pivotRow = maxRow;
      if (
        preferDiagonal &&
        maxRow >= 0 &&
        this.mark[j] === gen &&
        pinv[j] < 0 &&
        Math.abs(xWork[j]) >= DIAGONAL_PREFERENCE * maxAbs
      ) {
        pivotRow = j;
      }
      // maxAbs covers the not-yet-pivotal rows; U entries below cover the
      // pivotal ones, together giving the full intermediate magnitude.
      if (maxAbs > growth) growth = maxAbs;

      const patternSize = n - reachTop;
      if (this.uCount + patternSize > this.uRows.length) {
        this.uRows = growInt32(this.uRows, this.uCount + patternSize);
        this.uVals = growFloat64(this.uVals, this.uCount + patternSize);
      }
      if (this.lCount + patternSize > lRows.length) {
        this.lRows = lRows = growInt32(lRows, this.lCount + patternSize);
        this.lVals = lVals = growFloat64(lVals, this.lCount + patternSize);
      }
      const uRows = this.uRows;
      const uVals = this.uVals;

      // U(:,k): entries whose rows are already pivotal, in step coordinates.
      // Stored for singular steps too — their pivot solves to zero, so the
      // entries are inert, but the uniform layout keeps the solver branchless.
      for (let idx = reachTop; idx < n; idx++) {
        const row = reach[idx];
        const step = pinv[row];
        if (step >= 0) {
          const entry = xWork[row];
          uRows[this.uCount] = step;
          uVals[this.uCount] = entry;
          this.uCount += 1;
          const magnitude = Math.abs(entry);
          if (magnitude > growth) growth = magnitude;
        }
      }

      const pivotValue = pivotRow >= 0 ? xWork[pivotRow] : 0;
      const absPivot = Math.abs(pivotValue);
      if (absPivot > PIVOT_TOLERANCE && Number.isFinite(absPivot)) {
        rank += 1;
        minScaledPivot = Math.min(minScaledPivot, absPivot);
        pinv[pivotRow] = k;
        prow[k] = pivotRow;
        this.uDiag[k] = pivotValue;
        // L(:,k): multipliers for rows still below the diagonal, kept in
        // original row indices (see the forward-substitution note).
        for (let idx = reachTop; idx < n; idx++) {
          const row = reach[idx];
          if (pinv[row] >= 0) continue;
          lRows[this.lCount] = row;
          lVals[this.lCount] = xWork[row] / pivotValue;
          this.lCount += 1;
        }
      } else {
        // Singular column: no pivot assigned, no elimination, rank not
        // incremented. The sub-diagonal values are dropped; the residual
        // check reports the damage and the engine rejects the step.
        prow[k] = -1;
        this.uDiag[k] = 0;
      }
      this.lColPtr[k + 1] = this.lCount;
      this.uColPtr[k + 1] = this.uCount;
    }

    this.fRank = rank;
    this.fMinScaledPivot = Number.isFinite(minScaledPivot) ? minScaledPivot : 0;
    return growth;
  }
}
