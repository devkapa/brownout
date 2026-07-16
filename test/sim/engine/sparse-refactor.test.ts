/**
 * Refactorization fast-path equivalence and stability suite
 * =========================================================
 *
 * Value-only restamps (every Newton iteration) take SparseMNA's numeric-only
 * refactorization fast path: the stored pivot order and L/U column patterns
 * are replayed with no reachability DFS and no pivot search (sparse-mna.ts).
 * That path is only allowed to trade time, never stability, so this suite
 * drives Newton-like jitter sequences through one persistent instance and
 * compares every solve against a fresh SparseMNA — a fresh instance has no
 * stored pivot order, so its first solve is a full symbolic+numeric pass by
 * construction and stands in for what fresh pivoting would produce.
 *
 * The guard-trip tests aim one restamp at each replay-abandonment condition
 * (degraded stored pivot, singular pivot, non-finite values) and assert the
 * fallback through every observable: the refactorization counter must not
 * move, singularity must stay in parity with a fresh instance, outputs must
 * stay finite, and the shared backward-error oracle (relativeResidual against
 * the original unscaled matrix, see linear-system.ts) must hold. Counter
 * expectations are exact: the fixture is fully deterministic, so a drifting
 * count means the fast path silently changed shape, which is precisely what
 * this suite exists to catch.
 */

import { describe, expect, it } from "vitest";
import type { MnaSolveInfo } from "../../../src/sim/engine/mna.js";
import { SparseMNA } from "../../../src/sim/engine/sparse-mna.js";

/** Deterministic PRNG so every jitter sequence is reproducible from its seed. */
function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const SYSTEM_SIZE = 36;
/** Node unknowns 0..33; ideal-source branch currents 34..35. */
const NODE_COUNT = 34;
const BRANCH_A = 34;
const BRANCH_B = 35;

/**
 * Degree-1 node hanging off the ladder. AMD eliminates a degree-1 vertex
 * before its higher-degree anchor, so the pendant's column is replayed while
 * the anchor row is still an unpivoted fresh candidate. Shrinking the
 * pendant's diagonal therefore trips the REFACTOR_PIVOT_DEGRADATION guard
 * deterministically; an interior ladder diagonal would not, because by its
 * elimination step the Schur updates dominate the shrunken cell.
 */
const PENDANT = 33;
const PENDANT_ANCHOR = 20;

/** Coordinates deliberately absent from the base pattern (growth/overlay tests). */
const GROWTH_COUPLING: readonly [number, number] = [5, 28];
const OVERLAY_COUPLING: readonly [number, number] = [11, 24];

/**
 * Long-range chords are fixed rather than PRNG-drawn so the reserved
 * coordinates above stay structurally absent and the pendant keeps degree 1;
 * the seeded PRNG drives the value jitters instead.
 */
const CHORDS: ReadonlyArray<readonly [number, number]> = [
  [2, 14],
  [6, 25],
  [9, 30],
  [13, 27],
  [4, 19],
];

/** Backward-error bound every accepted solve must meet, replayed or full. */
const RESIDUAL_TOLERANCE = 1e-10;
/** Mixed absolute+relative band absorbing elimination-order roundoff. */
const SOLUTION_TOLERANCE = 1e-9;
/**
 * Relative half-width of one multiplicative jitter. Fifty compounded steps
 * drift values by at most a few percent, orders of magnitude inside the
 * replay guards (pivot degradation at 1e-6 of the column max, growth at
 * 1e6), so every replay in the Newton-like sequence must succeed.
 */
const JITTER_SCALE = 5e-4;
const NEWTON_ITERATIONS = 50;

interface MatrixStamp {
  row: number;
  col: number;
  val: number;
}

interface RhsStamp {
  row: number;
  val: number;
}

interface StampSystem {
  stamps: MatrixStamp[];
  rhs: RhsStamp[];
}

/**
 * Fixed MNA-shaped pattern: resistor ladder over nodes 0..32, megaohm shunts
 * every fifth node, long-range chords, the guard-test pendant, and two
 * ideal-source border rows with structurally zero branch diagonals — the
 * shape that forces off-diagonal pivoting and mirrors what the engine stamps.
 */
function buildSystem(): StampSystem {
  const stamps: MatrixStamp[] = [];
  const rhs: RhsStamp[] = [];
  const couple = (i: number, j: number, g: number): void => {
    stamps.push({ row: i, col: i, val: g });
    stamps.push({ row: j, col: j, val: g });
    stamps.push({ row: i, col: j, val: -g });
    stamps.push({ row: j, col: i, val: -g });
  };

  // Ladder chains nodes 0..32; the pendant stays out so its degree is 1.
  for (let i = 0; i < NODE_COUNT - 2; i++) {
    couple(i, i + 1, 1 / (100 + (i % 10) * 47));
  }
  for (let i = 0; i < NODE_COUNT - 1; i += 5) {
    stamps.push({ row: i, col: i, val: 1e-6 });
  }
  CHORDS.forEach(([i, j], index) => {
    couple(i, j, 1 / (150 + index * 83));
  });
  couple(PENDANT, PENDANT_ANCHOR, 1 / 220);
  stamps.push({ row: PENDANT, col: PENDANT, val: 1e-6 });

  stamps.push({ row: 0, col: BRANCH_A, val: 1 });
  stamps.push({ row: BRANCH_A, col: 0, val: 1 });
  rhs.push({ row: BRANCH_A, val: 5 });
  stamps.push({ row: 17, col: BRANCH_B, val: -1 });
  stamps.push({ row: BRANCH_B, col: 17, val: -1 });
  rhs.push({ row: BRANCH_B, val: -2.5 });

  rhs.push({ row: 8, val: 2e-3 });
  rhs.push({ row: 26, val: -1.5e-3 });
  return { stamps, rhs };
}

function copySystem(sys: StampSystem): StampSystem {
  return {
    stamps: sys.stamps.map((s) => ({ ...s })),
    rhs: sys.rhs.map((r) => ({ ...r })),
  };
}

function stampInto(mna: SparseMNA, sys: StampSystem): void {
  for (const s of sys.stamps) mna.add(s.row, s.col, s.val);
  for (const r of sys.rhs) mna.addB(r.row, r.val);
}

/** Newton-iteration shape: clear (pattern kept) and restamp every value. */
function restamp(mna: SparseMNA, sys: StampSystem): void {
  mna.clear();
  stampInto(mna, sys);
}

/** Small multiplicative perturbation of every value, matrix and RHS alike. */
function jitterSystem(sys: StampSystem, rng: () => number): void {
  for (const s of sys.stamps) s.val *= 1 + JITTER_SCALE * (rng() * 2 - 1);
  for (const r of sys.rhs) r.val *= 1 + JITTER_SCALE * (rng() * 2 - 1);
}

/** Copy with every stamp accumulating into one cell scaled by `factor`. */
function scaleCell(sys: StampSystem, row: number, col: number, factor: number): StampSystem {
  const out = copySystem(sys);
  for (const s of out.stamps) {
    if (s.row === row && s.col === col) s.val *= factor;
  }
  return out;
}

/**
 * Copy with every stamp touching `node` zeroed: the row and column become
 * exactly zero (structural rank deficiency), while the slots stay in the
 * pattern so the persistent instance sees a value-only change.
 */
function zeroNode(sys: StampSystem, node: number): StampSystem {
  const out = copySystem(sys);
  for (const s of out.stamps) {
    if (s.row === node || s.col === node) s.val = 0;
  }
  return out;
}

/**
 * Solve on a brand-new instance: no stored pivot order exists, so this is a
 * forced full symbolic+numeric factorization. The counter assertions keep
 * the oracle honest — if the fresh path ever replays, the whole suite would
 * be comparing the fast path against itself.
 */
function freshFullFactorSolve(sys: StampSystem): { x: Float64Array; info: MnaSolveInfo } {
  const mna = new SparseMNA(SYSTEM_SIZE);
  stampInto(mna, sys);
  const x = mna.solve();
  expect(mna.factorizationCount).toBe(1);
  expect(mna.refactorizationCount).toBe(0);
  return { x, info: mna.lastSolveInfo };
}

function expectSolutionsClose(actual: Float64Array, reference: Float64Array, label: string): void {
  for (let i = 0; i < actual.length; i++) {
    const bound =
      SOLUTION_TOLERANCE +
      SOLUTION_TOLERANCE * Math.max(Math.abs(actual[i]), Math.abs(reference[i]));
    expect(
      Math.abs(actual[i] - reference[i]),
      `${label}: component ${String(i)} replayed=${String(actual[i])} fresh=${String(reference[i])}`,
    ).toBeLessThanOrEqual(bound);
  }
}

describe("SparseMNA refactorization fast path — Newton-like jitter sequences", () => {
  it("replays the stored pivot order for 50 value-only restamps and matches a full factorization each time", () => {
    const sys = buildSystem();
    const mna = new SparseMNA(SYSTEM_SIZE);
    stampInto(mna, sys);
    mna.solve();
    expect(mna.factorizationCount).toBe(1);
    expect(mna.refactorizationCount).toBe(0);
    expect(mna.lastSolveInfo.singular).toBe(false);

    const rng = mulberry32(0xa11ce);
    for (let iter = 0; iter < NEWTON_ITERATIONS; iter++) {
      jitterSystem(sys, rng);
      restamp(mna, sys);
      const x = mna.solve();
      const label = `iteration ${String(iter)}`;
      expect(mna.lastSolveInfo.singular, `${label}: replay must stay full rank`).toBe(false);
      expect(
        mna.lastSolveInfo.relativeResidual,
        `${label}: replayed-factor backward error`,
      ).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);

      const reference = freshFullFactorSolve(sys);
      expect(
        reference.info.relativeResidual,
        `${label}: fresh-factor backward error`,
      ).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
      expectSolutionsClose(x, reference.x, label);
    }

    // Every jitter is one numeric pass and every pass replayed: the fast
    // path must neither skip a factorization, nor fall back, nor be
    // misclassified as an exact-restamp reuse by the value guard.
    expect(mna.factorizationCount).toBe(1 + NEWTON_ITERATIONS);
    expect(mna.refactorizationCount).toBe(NEWTON_ITERATIONS);
    expect(mna.factorizationReuseCount).toBe(0);
  });
});

describe("SparseMNA refactorization guard trips", () => {
  it("falls back to fresh pivoting when a stored pivot degrades by 1e12, stays correct, then re-arms", () => {
    const sys = buildSystem();
    const mna = new SparseMNA(SYSTEM_SIZE);
    stampInto(mna, sys);
    mna.solve();
    const rng = mulberry32(0xd15c0);
    jitterSystem(sys, rng);
    restamp(mna, sys);
    mna.solve();
    // The fast path must demonstrably be live before the trip, otherwise
    // the unchanged-counter assertion below would pass vacuously.
    expect(mna.refactorizationCount).toBe(1);

    const shrunk = scaleCell(sys, PENDANT, PENDANT, 1e-12);
    restamp(mna, shrunk);
    const x = mna.solve();
    // The numeric pass ran but the replay counter did not move: the
    // degradation guard abandoned the replay and the full pass took over.
    expect(mna.factorizationCount).toBe(3);
    expect(mna.refactorizationCount).toBe(1);
    expect(mna.lastSolveInfo.singular).toBe(false);
    expect(mna.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);

    const reference = freshFullFactorSolve(shrunk);
    expect(reference.info.singular).toBe(false);
    expect(reference.info.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
    expectSolutionsClose(x, reference.x, "degraded-pivot fallback");

    // The clean fallback factorization re-elects pivots for the shrunken
    // matrix, so the very next jitter must replay again.
    jitterSystem(shrunk, rng);
    restamp(mna, shrunk);
    mna.solve();
    expect(mna.refactorizationCount).toBe(2);
    expect(mna.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
  });

  it("keeps singular-flag parity with a fresh instance when values go singular mid-sequence, then recovers", () => {
    const sys = buildSystem();
    const mna = new SparseMNA(SYSTEM_SIZE);
    stampInto(mna, sys);
    mna.solve();
    const rng = mulberry32(0xdead5);
    jitterSystem(sys, rng);
    restamp(mna, sys);
    mna.solve();
    expect(mna.refactorizationCount).toBe(1);

    // Node 25 is unpinned and not the pendant anchor; zeroing its row and
    // column is exact structural rank loss under an unchanged pattern.
    const dead = zeroNode(sys, 25);
    restamp(mna, dead);
    const x = mna.solve();
    expect(mna.refactorizationCount).toBe(1);
    expect(mna.lastSolveInfo.singular).toBe(true);

    const reference = freshFullFactorSolve(dead);
    expect(reference.info.singular).toBe(mna.lastSolveInfo.singular);
    expect(reference.info.rank).toBe(mna.lastSolveInfo.rank);
    for (let i = 0; i < SYSTEM_SIZE; i++) {
      expect(Number.isFinite(x[i]), `persistent x[${String(i)}] must stay finite`).toBe(true);
      expect(Number.isFinite(reference.x[i]), `fresh x[${String(i)}] must stay finite`).toBe(true);
    }

    // Restoring the values must fully recover: correct results and, after
    // one clean full pass, a working fast path again.
    restamp(mna, sys);
    const restored = mna.solve();
    expect(mna.lastSolveInfo.singular).toBe(false);
    expect(mna.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
    const restoredReference = freshFullFactorSolve(sys);
    expectSolutionsClose(restored, restoredReference.x, "post-singular recovery");

    jitterSystem(sys, rng);
    restamp(mna, sys);
    mna.solve();
    expect(mna.refactorizationCount).toBe(2);
  });

  it("reports non-finite values without throwing and recovers on the next clean restamp", () => {
    const sys = buildSystem();
    const mna = new SparseMNA(SYSTEM_SIZE);
    stampInto(mna, sys);
    mna.solve();
    const rng = mulberry32(0xbadf0);
    jitterSystem(sys, rng);
    restamp(mna, sys);
    mna.solve();
    expect(mna.refactorizationCount).toBe(1);

    // Any stamp accumulating into (10,10) poisons that cell; a diagonal the
    // stored order pivots on maximises how far the NaN reaches.
    const poisoned = copySystem(sys);
    const diagIndex = poisoned.stamps.findIndex((s) => s.row === 10 && s.col === 10);
    expect(diagIndex).toBeGreaterThanOrEqual(0);
    poisoned.stamps[diagIndex].val = Number.NaN;
    restamp(mna, poisoned);
    // Must surface through lastSolveInfo, not throw: the engine rejects the
    // step on the flag, and a throw would take down the worker loop.
    const x = mna.solve();
    expect(mna.lastSolveInfo.nonFinite).toBe(true);
    expect(x.length).toBe(SYSTEM_SIZE);

    restamp(mna, sys);
    const restored = mna.solve();
    expect(mna.lastSolveInfo.nonFinite).toBe(false);
    expect(mna.lastSolveInfo.singular).toBe(false);
    expect(mna.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
    const reference = freshFullFactorSolve(sys);
    expectSolutionsClose(restored, reference.x, "post-NaN recovery");
  });
});

describe("SparseMNA pattern growth mid-sequence", () => {
  it("takes the full symbolic path for a new nonzero coupling slot, then replays on the grown pattern", () => {
    const sys = buildSystem();
    const mna = new SparseMNA(SYSTEM_SIZE);
    stampInto(mna, sys);
    mna.solve();
    const rng = mulberry32(0x9f0f1);
    jitterSystem(sys, rng);
    restamp(mna, sys);
    mna.solve();
    expect(mna.refactorizationCount).toBe(1);

    // Stamp a coupling at reserved coordinates on top of the live matrix:
    // the off-diagonal slots are new, so the pattern grows in place.
    const g = 1 / 250;
    const [a, b] = GROWTH_COUPLING;
    const grown = copySystem(sys);
    grown.stamps.push({ row: a, col: a, val: g });
    grown.stamps.push({ row: b, col: b, val: g });
    grown.stamps.push({ row: a, col: b, val: -g });
    grown.stamps.push({ row: b, col: a, val: -g });
    mna.add(a, a, g);
    mna.add(b, b, g);
    mna.add(a, b, -g);
    mna.add(b, a, -g);
    const x = mna.solve();
    // A dirty pattern must force symbolic analysis plus a full numeric
    // pass; a replay here would read L/U patterns from a stale analysis.
    expect(mna.factorizationCount).toBe(3);
    expect(mna.refactorizationCount).toBe(1);
    expect(mna.lastSolveInfo.singular).toBe(false);
    expect(mna.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
    const reference = freshFullFactorSolve(grown);
    expectSolutionsClose(x, reference.x, "post-growth full pass");

    // The full pass on the grown pattern re-establishes replay eligibility.
    jitterSystem(grown, rng);
    restamp(mna, grown);
    const jittered = mna.solve();
    expect(mna.refactorizationCount).toBe(2);
    expect(mna.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
    const jitteredReference = freshFullFactorSolve(grown);
    expectSolutionsClose(jittered, jitteredReference.x, "post-growth replay");
  });
});

describe("SparseMNA base/overlay interplay with refactorization", () => {
  it("matches a fresh base-only instance after every resetToBase across overlay cycles", () => {
    const sys = buildSystem();
    const mna = new SparseMNA(SYSTEM_SIZE);
    stampInto(mna, sys);
    mna.captureBase();
    const baseReference = freshFullFactorSolve(sys);
    const first = mna.solve();
    expectSolutionsClose(first, baseReference.x, "pre-overlay base");

    const g = 1 / 300;
    const [a, b] = OVERLAY_COUPLING;
    const overlaySys = copySystem(sys);
    overlaySys.stamps.push({ row: a, col: a, val: g });
    overlaySys.stamps.push({ row: b, col: b, val: g });
    overlaySys.stamps.push({ row: a, col: b, val: -g });
    overlaySys.stamps.push({ row: b, col: a, val: -g });
    overlaySys.rhs.push({ row: a, val: 1e-3 });
    const overlayReference = freshFullFactorSolve(overlaySys);

    const applyOverlay = (): void => {
      mna.add(a, a, g);
      mna.add(b, b, g);
      mna.add(a, b, -g);
      mna.add(b, a, -g);
      mna.addB(a, 1e-3);
    };

    for (let cycle = 0; cycle < 3; cycle++) {
      const label = `cycle ${String(cycle)}`;
      applyOverlay();
      const withOverlay = mna.solve();
      expect(mna.lastSolveInfo.singular, `${label}: overlay`).toBe(false);
      expect(
        mna.lastSolveInfo.relativeResidual,
        `${label}: overlay backward error`,
      ).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
      expectSolutionsClose(withOverlay, overlayReference.x, `${label}: overlay`);

      // Post-reset state must equal base stamped from scratch even though
      // the persistent pattern still carries the (now zero) overlay slots.
      mna.resetToBase();
      const restored = mna.solve();
      expect(mna.lastSolveInfo.singular, `${label}: post-reset`).toBe(false);
      expect(
        mna.lastSolveInfo.relativeResidual,
        `${label}: post-reset backward error`,
      ).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
      expectSolutionsClose(restored, baseReference.x, `${label}: post-reset`);
    }

    // Deterministic pass ledger: initial full pass, one symbolic+full pass
    // when the first overlay grows the pattern, then every overlay/reset
    // restamp is value-only and must replay — five refactorizations across
    // the seven numeric passes, with no exact-restamp reuse in between.
    expect(mna.factorizationCount).toBe(7);
    expect(mna.refactorizationCount).toBe(5);
    expect(mna.factorizationReuseCount).toBe(0);
  });
});

describe("SparseMNA existing reuse/refactorize contract", () => {
  it("still reuses the LU on an exact restamp and refactorizes on a jitter", () => {
    const sys = buildSystem();
    const mna = new SparseMNA(SYSTEM_SIZE);
    stampInto(mna, sys);
    const firstX = mna.solve();

    // Identical restamp: the value guard must classify this as the same
    // numeric matrix and skip factorization entirely, replay included.
    restamp(mna, sys);
    const reused = mna.solve();
    expect([...reused]).toEqual([...firstX]);
    expect(mna.factorizationCount).toBe(1);
    expect(mna.factorizationReuseCount).toBe(1);
    expect(mna.refactorizationCount).toBe(0);

    // Jittered restamp: same pattern, changed values — one fresh numeric
    // pass through the value-only route, with the reuse counter untouched.
    jitterSystem(sys, mulberry32(0xc0ffe));
    restamp(mna, sys);
    mna.solve();
    expect(mna.factorizationCount).toBe(2);
    expect(mna.factorizationReuseCount).toBe(1);
    expect(mna.refactorizationCount).toBe(1);
    expect(mna.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
  });
});
