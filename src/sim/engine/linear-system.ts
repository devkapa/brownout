/**
 * Backend-neutral linear-system contract for the MNA solve path.
 *
 * `SimEngine` and every element stamp talk to this interface only; the
 * concrete backend is chosen per topology size in `createLinearSystem`.
 * The dense `MNA` class satisfies it structurally (it predates this file
 * and stays untouched); `SparseMNA` implements it explicitly.
 *
 * Contract notes that both backends must honor identically:
 * - `clear()` zeroes matrix values and RHS. A sparse backend keeps its
 *   discovered pattern across `clear()` — dropping it would rebuild the
 *   symbolic analysis every Newton iteration.
 * - `captureBase()`/`resetToBase()` snapshot/restore values AND RHS. Stamps
 *   added after `captureBase()` (the per-iteration nonlinear overlay) must
 *   be erased by `resetToBase()`.
 * - `solveRhs()` may reuse the previous factorization only when the matrix
 *   values are exactly those that were factorized; reuse increments
 *   `factorizationReuseCount`, a fresh factorization increments
 *   `factorizationCount`. Solutions are validated against the original
 *   (unscaled) matrix and reported through `lastSolveInfo` — algebraically
 *   invalid results must be surfaced there, never silently returned.
 * - Singular systems return 0 for the deficient variables and set
 *   `lastSolveInfo.singular`; callers reject the step.
 */

import { MNA, type MnaSolveInfo } from "./mna.js";
import { SparseMNA } from "./sparse-mna.js";

export interface LinearSystem {
  readonly size: number;
  /** Diagnostics for the most recent factorize/solve. */
  lastSolveInfo: MnaSolveInfo;
  /** Monotonic diagnostic counters; never affect solve validity. */
  factorizationCount: number;
  factorizationReuseCount: number;
  /** Optional backend discriminator for diagnostics ("dense" when absent). */
  readonly backend?: "dense" | "sparse";
  clear(): void;
  add(row: number, col: number, val: number): void;
  addB(row: number, val: number): void;
  captureBase(): void;
  resetToBase(): void;
  solve(): Float64Array;
  solveRhs(rhs: ArrayLike<number>): Float64Array;
  /**
   * Optional pattern checkpointing. The sparse backend keeps its discovered
   * stamp pattern across `clear()`, so an exploratory solve sequence (the
   * dcOperatingPoint homotopy ladder walking regimes ordinary stepping never
   * revisits) can permanently enlarge the pattern. Pattern membership feeds
   * the fill-reducing ordering, and the ordering fixes the exact rounding of
   * every later factorization — without a restore, post-exploration
   * trajectories drift by ULPs from an engine handed the identical state.
   * Backends whose numerics are pattern-free (dense) omit both hooks.
   * `restorePatternCheckpoint` must only receive a value produced by
   * `patternCheckpoint` on the same instance.
   */
  patternCheckpoint?(): number;
  restorePatternCheckpoint?(checkpoint: number): void;
  /**
   * Optional numeric-history invalidation, the companion of
   * restorePatternCheckpoint for state a slot checkpoint cannot express:
   * the sparse backend replays its last pivot order on value-only restamps
   * and reuses stored factors on exact value matches, both keyed to
   * whatever system was factorized LAST. After an exploratory sequence is
   * rolled back without pattern growth the checkpoint restore no-ops, yet
   * that memory still describes systems the caller never accepted — the
   * next replay would eliminate in a different order than an engine that
   * never explored, drifting by ULPs. Dropping the memory forces the next
   * solve to re-derive pivots from the restored pattern and values alone.
   * History-free backends (dense: pivoting is a pure function of the
   * current values) omit the hook.
   */
  invalidateFactorization?(): void;
}

/**
 * Stamp-only accumulation surface of a LinearSystem. Element stamp helpers
 * and registered device models are typed against this narrowed view (see
 * device-registry.ts, decision 6): the clear/captureBase/solve lifecycle is
 * engine-owned, and handing stamp code the full interface would let one
 * mis-called clear() or captureBase() silently poison the captured static
 * base for every later solve. Type-level only — the runtime object is the
 * engine's LinearSystem instance.
 */
export type MnaStampSurface = Pick<LinearSystem, "size" | "add" | "addB">;

export type LinearBackendMode = "auto" | "dense" | "sparse";

/**
 * Dense LU is O(n^3) per factorization but has lower constants than the
 * sparse path. Measured 2026-07-16 (after the AMD ordering and the
 * numeric-only refactorization fast path landed) cold-vs-cold, a fresh
 * backend per rep — the full symbolic + AMD + numeric cost paid after a
 * topology edit, and the sparse path's worst case since every later
 * value-only pass is a cheaper replay: dense wins at n = 32 (0.019 ms vs
 * 0.027 ms), sparse wins from n = 48 up (0.038 ms vs 0.047 ms, then
 * 0.050 ms vs 0.076 ms at 64 and roughly 4x by 96), and the replay wins at
 * every measured size. The bench's own crossover scan reuses one warm
 * instance per size, so its sparse medians are replay-dominated and its
 * printed recommendation reflects the warm path only. The threshold sits
 * one step above the cold crossover: below it the dense backend keeps
 * existing small-circuit behavior bit-identical, which the deterministic
 * trajectory baselines rely on, and the full default-mode corpus is green
 * at this value, so no baseline fixture crossed backends.
 */
export const SPARSE_BACKEND_THRESHOLD = 64;

let forcedBackend: LinearBackendMode | null = null;

/** Test hook. Pass null to restore automatic selection. */
export function setLinearSystemBackendForTests(mode: LinearBackendMode | null): void {
  forcedBackend = mode;
}

function resolveBackendMode(): LinearBackendMode {
  if (forcedBackend && forcedBackend !== "auto") return forcedBackend;
  // Environment override so the full simulator test corpus can be replayed
  // through either backend without code changes (equivalence gate).
  // Guarded: `process` does not exist inside a browser WebWorker.
  if (typeof process !== "undefined" && process.env) {
    const env = process.env.SIMCORE_LINEAR_BACKEND;
    if (env === "dense" || env === "sparse") return env;
  }
  return "auto";
}

export function createLinearSystem(size: number): LinearSystem {
  const mode = resolveBackendMode();
  if (mode === "dense") return new MNA(size);
  if (mode === "sparse") return new SparseMNA(size);
  return size >= SPARSE_BACKEND_THRESHOLD ? new SparseMNA(size) : new MNA(size);
}
