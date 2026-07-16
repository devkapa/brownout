/**
 * Dense/sparse linear-backend equivalence suite
 * =============================================
 *
 * The sparse Gilbert-Peierls backend eliminates in a different order than
 * the dense partial-pivot LU, so bitwise-equal solutions are impossible by
 * design; the shared contract (linear-system.ts) instead makes the backward
 * error against the original unscaled matrix (`relativeResidual`) the
 * cross-backend oracle. This suite enforces that contract from two sides:
 *
 * Part 1 stamps identical seeded sequences (ladder base, long-range
 * couplings, voltage-source border rows, occasional decade-spread scaling)
 * into `MNA` and `SparseMNA` and asserts backward stability of both,
 * component-wise agreement away from ill-conditioning, agreement on
 * singularity, and lockstep factorization/reuse counters across restamps.
 *
 * Part 2 replays full `SimEngine` trajectories (an RC ladder and a
 * diode-terminated variant) once per forced backend and bounds the probe
 * voltage divergence, proving equivalence survives Newton iteration,
 * companion-model restamps, and element-state carry across hundreds of
 * steps — not just a single factorization.
 */

import { afterEach, describe, expect, it } from "vitest";
import { MNA } from "../../../src/sim/engine/mna.js";
import { SparseMNA } from "../../../src/sim/engine/sparse-mna.js";
import { createLinearSystem, setLinearSystemBackendForTests } from "../../../src/sim/engine/linear-system.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

// The hook is global state; a failed assertion mid-test must never leak a
// forced backend into unrelated suites running in the same worker.
afterEach(() => {
  setLinearSystemBackendForTests(null);
});

/** Deterministic PRNG so every generated system is reproducible from its seed. */
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

function randInt(rng: () => number, bound: number): number {
  return Math.floor(rng() * bound);
}

function pick<T>(rng: () => number, items: readonly T[]): T {
  return items[randInt(rng, items.length)];
}

// ─── Part 1: matrix-level property tests ─────────────────────────────────────

/**
 * Values are quantized to a small mantissa set times a power of ten so that
 * rank deficiency is always structural (an exactly zero row/column), never a
 * borderline pivot that the two backends' different elimination orders could
 * classify differently.
 */
const MANTISSAS = [1, 2, 3, 5, 7] as const;

/** Backward-error bound both backends must meet on every accepted solve. */
const RESIDUAL_TOLERANCE = 1e-8;
/** Component agreement is only meaningful away from ill-conditioning. */
const WELL_CONDITIONED_PIVOT = 1e-6;
const SOLUTION_TOLERANCE = 1e-6;

interface MatrixStamp {
  row: number;
  col: number;
  val: number;
}

interface RhsStamp {
  row: number;
  val: number;
}

interface StampSequence {
  n: number;
  matrix: MatrixStamp[];
  rhs: RhsStamp[];
  singularByConstruction: boolean;
}

/**
 * Generate one MNA-shaped system: a resistive ladder over the node block,
 * random long-range couplings, and 1-3 voltage-source border rows (+-1
 * coupling, structurally zero diagonal, RHS constraint value) — the exact
 * pattern that forces off-diagonal pivoting in both backends. Roughly a
 * third of the systems spread coefficients across 1e-9..1e6 to exercise the
 * equilibration paths.
 */
function generateSystem(n: number, seed: number, makeSingular: boolean): StampSequence {
  const rng = mulberry32(seed);
  // Cap the source count well under half the unknowns so border rows always
  // pin distinct nodes and a singular variant still has a free node to kill.
  const vsCount = Math.max(1, Math.min(1 + randInt(rng, 3), Math.floor((n - 1) / 2)));
  const nodes = n - vsCount;
  const decadeSpread = rng() < 0.35;

  const matrix: MatrixStamp[] = [];
  const rhs: RhsStamp[] = [];
  const magnitude = (): number =>
    pick(rng, MANTISSAS) * Math.pow(10, decadeSpread ? randInt(rng, 16) - 9 : 0);

  // Conductance-style two-node stamp. The repeated diagonal contributions
  // are the point: they exercise SparseMNA's slot accumulation against the
  // dense backend's plain adds on identical stamp order.
  const couple = (i: number, j: number): void => {
    const g = magnitude();
    matrix.push({ row: i, col: i, val: g });
    matrix.push({ row: j, col: j, val: g });
    matrix.push({ row: i, col: j, val: -g });
    matrix.push({ row: j, col: i, val: -g });
  };

  for (let i = 0; i + 1 < nodes; i++) couple(i, i + 1);
  for (let i = 0; i < nodes; i++) {
    // Ground legs keep nodes far from the pinned ones from floating on
    // conductance scale alone; node 0 always gets one for determinism.
    if (i === 0 || rng() < 0.25) matrix.push({ row: i, col: i, val: magnitude() });
  }
  if (nodes >= 4) {
    const extra = 1 + randInt(rng, Math.max(1, nodes >> 2));
    for (let e = 0; e < extra; e++) {
      const i = randInt(rng, nodes);
      let j = randInt(rng, nodes);
      while (Math.abs(i - j) <= 1) j = randInt(rng, nodes);
      couple(i, j);
    }
  }

  const pinned = new Set<number>();
  for (let br = nodes; br < n; br++) {
    let k = randInt(rng, nodes);
    while (pinned.has(k)) k = (k + 1) % nodes;
    pinned.add(k);
    const sign = rng() < 0.5 ? -1 : 1;
    matrix.push({ row: k, col: br, val: sign });
    matrix.push({ row: br, col: k, val: sign });
    rhs.push({
      row: br,
      val: sign * pick(rng, [1, 2, 5]) * (decadeSpread ? pick(rng, [1e-3, 1, 1e3]) : 1),
    });
  }
  for (let i = 0; i < nodes; i++) {
    if (rng() < 0.4) rhs.push({ row: i, val: (rng() < 0.5 ? -1 : 1) * magnitude() });
  }

  if (makeSingular) {
    // Exact rank deficiency by construction: strip every stamp touching one
    // unpinned node, leaving its row and column identically zero. Both
    // backends must classify this the same way regardless of pivot order.
    let dead = randInt(rng, nodes);
    while (pinned.has(dead)) dead = (dead + 1) % nodes;
    return {
      n,
      matrix: matrix.filter((s) => s.row !== dead && s.col !== dead),
      rhs: rhs.filter((s) => s.row !== dead),
      singularByConstruction: true,
    };
  }
  return { n, matrix, rhs, singularByConstruction: false };
}

function stampBoth(sequence: StampSequence, dense: MNA, sparse: SparseMNA): void {
  for (const s of sequence.matrix) {
    dense.add(s.row, s.col, s.val);
    sparse.add(s.row, s.col, s.val);
  }
  for (const s of sequence.rhs) {
    dense.addB(s.row, s.val);
    sparse.addB(s.row, s.val);
  }
}

function solveAndCompare(sequence: StampSequence, label: string): void {
  const dense = new MNA(sequence.n);
  const sparse = new SparseMNA(sequence.n);
  stampBoth(sequence, dense, sparse);
  const xd = dense.solve();
  const xs = sparse.solve();
  const di = dense.lastSolveInfo;
  const si = sparse.lastSolveInfo;

  expect(
    di.singular,
    `${label}: singular flags must agree (dense=${String(di.singular)}, sparse=${String(si.singular)}, ` +
      `denseRank=${String(di.rank)}, sparseRank=${String(si.rank)})`,
  ).toBe(si.singular);
  // Outputs stay finite in every case: singular variables back-substitute
  // to exact zero in both backends rather than NaN/Infinity.
  for (let i = 0; i < sequence.n; i++) {
    expect(Number.isFinite(xd[i]), `${label}: dense x[${String(i)}]=${String(xd[i])} must be finite`).toBe(true);
    expect(Number.isFinite(xs[i]), `${label}: sparse x[${String(i)}]=${String(xs[i])} must be finite`).toBe(true);
  }

  // The generator's quantization guarantees singularity appears exactly when
  // constructed; a mismatch here is a generator bug, not a kernel bug.
  expect(di.singular, `${label}: singularity must match construction`).toBe(
    sequence.singularByConstruction,
  );
  if (di.singular) return;

  expect(
    di.relativeResidual,
    `${label}: dense backward error (minScaledPivot=${String(di.minScaledPivot)})`,
  ).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
  expect(
    si.relativeResidual,
    `${label}: sparse backward error (minScaledPivot=${String(si.minScaledPivot)})`,
  ).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);

  if (di.minScaledPivot > WELL_CONDITIONED_PIVOT && si.minScaledPivot > WELL_CONDITIONED_PIVOT) {
    for (let i = 0; i < sequence.n; i++) {
      const bound =
        SOLUTION_TOLERANCE + SOLUTION_TOLERANCE * Math.max(Math.abs(xd[i]), Math.abs(xs[i]));
      expect(
        Math.abs(xd[i] - xs[i]),
        `${label}: component ${String(i)} dense=${String(xd[i])} sparse=${String(xs[i])}`,
      ).toBeLessThanOrEqual(bound);
    }
  }
}

const SIZES: number[] = [];
for (let n = 3; n <= 40; n++) SIZES.push(n);
SIZES.push(130, 300);
const SEEDS_PER_SIZE = 5;

describe("matrix property equivalence — identical stamps into MNA and SparseMNA", () => {
  SIZES.forEach((n, sizeIndex) => {
    it(`size ${String(n)}: ${String(SEEDS_PER_SIZE)} seeded systems agree`, () => {
      for (let s = 0; s < SEEDS_PER_SIZE; s++) {
        const globalIndex = sizeIndex * SEEDS_PER_SIZE + s;
        // Interleave a structurally singular system roughly every 7th draw
        // so both agreement branches stay exercised at every size band.
        const makeSingular = globalIndex % 7 === 3;
        const seed = (n * 7919 + s * 104729 + 1) >>> 0;
        const sequence = generateSystem(n, seed, makeSingular);
        solveAndCompare(sequence, `n=${String(n)} seed=${String(seed)} singular=${String(makeSingular)}`);
      }
    });
  });
});

describe("factorization/reuse counters track between backends", () => {
  function expectCountersInLockstep(dense: MNA, sparse: SparseMNA, label: string): void {
    expect(dense.factorizationCount, `${label}: factorizationCount`).toBe(
      sparse.factorizationCount,
    );
    expect(dense.factorizationReuseCount, `${label}: factorizationReuseCount`).toBe(
      sparse.factorizationReuseCount,
    );
  }

  for (const n of [6, 14, 33, 130]) {
    it(`size ${String(n)}: identical restamp reuses, jittered restamp refactorizes, in lockstep`, () => {
      const seed = (n * 6151 + 17) >>> 0;
      const sequence = generateSystem(n, seed, false);
      const dense = new MNA(n);
      const sparse = new SparseMNA(n);

      stampBoth(sequence, dense, sparse);
      dense.solve();
      sparse.solve();
      expectCountersInLockstep(dense, sparse, "first solve");
      expect(dense.factorizationCount).toBe(1);
      expect(dense.factorizationReuseCount).toBe(0);

      // Newton-iteration shape: clear + restamp the numerically identical
      // matrix. Both backends must detect the match and skip factorization.
      dense.clear();
      sparse.clear();
      stampBoth(sequence, dense, sparse);
      dense.solve();
      sparse.solve();
      expectCountersInLockstep(dense, sparse, "identical restamp");
      expect(dense.factorizationCount).toBe(1);
      expect(dense.factorizationReuseCount).toBe(1);

      // One changed value (same pattern) must force a fresh factorization in
      // both backends — the sparse path takes its value-only route here.
      const jittered: StampSequence = {
        ...sequence,
        matrix: sequence.matrix.map((s, index) => (index === 0 ? { ...s, val: s.val * 2 } : s)),
      };
      dense.clear();
      sparse.clear();
      stampBoth(jittered, dense, sparse);
      const xd = dense.solve();
      const xs = sparse.solve();
      expectCountersInLockstep(dense, sparse, "jittered restamp");
      expect(dense.factorizationCount).toBe(2);
      expect(dense.factorizationReuseCount).toBe(1);
      // The refactorized solves must still satisfy the shared oracle.
      expect(dense.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
      expect(sparse.lastSolveInfo.relativeResidual).toBeLessThanOrEqual(RESIDUAL_TOLERANCE);
      for (let i = 0; i < n; i++) {
        expect(Number.isFinite(xd[i])).toBe(true);
        expect(Number.isFinite(xs[i])).toBe(true);
      }
    });
  }
});

// ─── Part 2: engine-level trajectory equivalence ─────────────────────────────

const RC_STAGES = 30;
const TRAJECTORY_STEPS = 400;
const TRAJECTORY_H = 1e-5;
/** Probe fronts: ladder entry, midpoint, and far end (the diode node). */
const LADDER_PROBES: ReadonlyArray<readonly [string, string]> = [
  ["c1", "a"],
  ["c15", "a"],
  ["c30", "a"],
];
const TRAJECTORY_TOLERANCE_V = 1e-6;

/**
 * 30-stage RC ladder: 5 V -> (100 ohm series, 1 uF to return)^30. Large
 * enough to make elimination-order differences visible, linear enough that
 * any divergence beyond roundoff is a kernel bug. The diode variant
 * terminates the far end so every step also runs the Newton restamp path.
 */
function rcLadderCircuit(withDiode: boolean): SimCircuit {
  const components: SimCircuit["components"] = [
    {
      id: "src",
      kind: "voltage_source",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { voltage: 5 },
    },
  ];
  const wires: SimCircuit["wires"] = [
    { from_component: "src", from_pin: "pos", to_component: "r1", to_pin: "a" },
  ];
  for (let i = 1; i <= RC_STAGES; i++) {
    components.push({
      id: `r${String(i)}`,
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 100 },
    });
    components.push({
      id: `c${String(i)}`,
      kind: "capacitor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { capacitance: 1e-6 },
    });
    wires.push({
      from_component: `r${String(i)}`,
      from_pin: "b",
      to_component: `c${String(i)}`,
      to_pin: "a",
    });
    wires.push({
      from_component: `c${String(i)}`,
      from_pin: "b",
      to_component: "src",
      to_pin: "neg",
    });
    if (i < RC_STAGES) {
      wires.push({
        from_component: `r${String(i)}`,
        from_pin: "b",
        to_component: `r${String(i + 1)}`,
        to_pin: "a",
      });
    }
  }
  if (withDiode) {
    components.push({
      id: "d1",
      kind: "diode",
      pins: [{ id: "a" }, { id: "k" }],
      params: { vf: 0.7, iRated: 0.1, n: 1 },
    });
    wires.push({
      from_component: `r${String(RC_STAGES)}`,
      from_pin: "b",
      to_component: "d1",
      to_pin: "a",
    });
    wires.push({ from_component: "d1", from_pin: "k", to_component: "src", to_pin: "neg" });
  }
  return { components, wires, environment: { temperatureC: 25 } };
}

/** Minimal divider: matrix size far below SPARSE_BACKEND_THRESHOLD. */
function tinyDividerCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "src",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "r1",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
      {
        id: "r2",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 1000 },
      },
    ],
    wires: [
      { from_component: "src", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "r2", to_pin: "a" },
      { from_component: "r2", from_pin: "b", to_component: "src", to_pin: "neg" },
    ],
    environment: { temperatureC: 25 },
  };
}

/**
 * Run one cold-loaded trajectory under a forced backend and record probe-net
 * voltages every step. A fresh circuit per run keeps the two runs from
 * sharing mutable params/state through the engine.
 */
function runTrajectory(
  makeCircuit: () => SimCircuit,
  backend: "dense" | "sparse",
  probes: ReadonlyArray<readonly [string, string]>,
  steps: number,
  h: number,
): number[][] {
  setLinearSystemBackendForTests(backend);
  const engine = new SimEngine();
  engine.coldLoad(makeCircuit());
  const netIds = probes.map(([componentId, pinId]) => {
    const net = engine.nets.find((candidate) =>
      candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
    );
    if (!net) {
      throw new Error(`equivalence topology error: no net for ${componentId}.${pinId}`);
    }
    return net.id;
  });
  const traces: number[][] = probes.map(() => []);
  for (let step = 0; step < steps; step++) {
    engine.step(h);
    expect(
      engine.lastConverged,
      `${backend} step ${String(step)}: solver must converge throughout the trajectory`,
    ).toBe(true);
    const netV = engine.getNetV();
    for (let probe = 0; probe < netIds.length; probe++) {
      const value = netV[netIds[probe]];
      if (value === undefined || !Number.isFinite(value)) {
        throw new Error(`${backend} step ${String(step)}: probe ${netIds[probe]} not finite`);
      }
      traces[probe].push(value);
    }
  }
  return traces;
}

function maxDivergence(a: number[][], b: number[][]): number {
  let worst = 0;
  for (let probe = 0; probe < a.length; probe++) {
    for (let step = 0; step < a[probe].length; step++) {
      worst = Math.max(worst, Math.abs(a[probe][step] - b[probe][step]));
    }
  }
  return worst;
}

describe("engine trajectory equivalence — forced dense vs forced sparse", () => {
  it("RC ladder: probe voltages diverge at most 1e-6 V across 400 steps", () => {
    const dense = runTrajectory(
      () => rcLadderCircuit(false),
      "dense",
      LADDER_PROBES,
      TRAJECTORY_STEPS,
      TRAJECTORY_H,
    );
    const sparse = runTrajectory(
      () => rcLadderCircuit(false),
      "sparse",
      LADDER_PROBES,
      TRAJECTORY_STEPS,
      TRAJECTORY_H,
    );
    // Vacuous-pass guard: the ladder front must actually have charged, or a
    // dead engine would trivially satisfy the divergence bound.
    expect(dense[0][TRAJECTORY_STEPS - 1]).toBeGreaterThan(0.5);
    expect(maxDivergence(dense, sparse)).toBeLessThanOrEqual(TRAJECTORY_TOLERANCE_V);
  });

  it("diode-terminated RC ladder: probe voltages diverge at most 1e-6 V across 400 steps", () => {
    const dense = runTrajectory(
      () => rcLadderCircuit(true),
      "dense",
      LADDER_PROBES,
      TRAJECTORY_STEPS,
      TRAJECTORY_H,
    );
    const sparse = runTrajectory(
      () => rcLadderCircuit(true),
      "sparse",
      LADDER_PROBES,
      TRAJECTORY_STEPS,
      TRAJECTORY_H,
    );
    expect(dense[0][TRAJECTORY_STEPS - 1]).toBeGreaterThan(0.5);
    expect(maxDivergence(dense, sparse)).toBeLessThanOrEqual(TRAJECTORY_TOLERANCE_V);
  });

  it("tiny divider under forced sparse matches forced dense (small-n sparse path)", () => {
    // The hook must actually override the auto threshold, otherwise this
    // test would silently compare dense against dense.
    setLinearSystemBackendForTests("sparse");
    expect(createLinearSystem(4).backend).toBe("sparse");
    setLinearSystemBackendForTests("dense");
    expect(createLinearSystem(400).backend ?? "dense").toBe("dense");

    const probes: ReadonlyArray<readonly [string, string]> = [["r1", "b"]];
    const dense = runTrajectory(tinyDividerCircuit, "dense", probes, 50, TRAJECTORY_H);
    const sparse = runTrajectory(tinyDividerCircuit, "sparse", probes, 50, TRAJECTORY_H);
    // Divider midpoint sits at 2.5 V; require the trace to be live.
    expect(dense[0][49]).toBeGreaterThan(2.4);
    expect(dense[0][49]).toBeLessThan(2.6);
    expect(maxDivergence(dense, sparse)).toBeLessThanOrEqual(TRAJECTORY_TOLERANCE_V);
  });
});
