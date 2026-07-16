/**
 * Scaling benchmark for the simcore linear backends (dense MNA vs SparseMNA)
 * and the SimEngine end-to-end step cost on synthetic RC ladders.
 *
 * Five sections, each printed as a markdown table:
 * 1. Kernel: median factor and LU-reuse solve cost per backend across sizes.
 *    A deterministic per-rep jitter perturbs the diagonal so every timed
 *    solve is a true refactorization, never an exact-match reuse. Since the
 *    A1.5 replay fast path landed, reps after the first take that path, so
 *    the sparse column now reflects steady-state Newton cost rather than the
 *    A1-era full elimination; section 4 separates the two explicitly.
 * 2. Engine: ms/step on an RC ladder with the backend forced dense or sparse
 *    via the test hook, so the auto-threshold cannot mask either path.
 * 3. Crossover: smallest n where the sparse factor beats dense, reported as
 *    a recommendation for SPARSE_BACKEND_THRESHOLD in linear-system.ts.
 * 4. Refactor path (A1.5): cold factor (symbolic + AMD + full LU on a fresh
 *    backend) versus the numeric-only replay on value-jittered restamps of a
 *    fixed pattern, with refactorizationCount proving which path ran.
 * 5. Fill metrics (A1.5): L+U nonzeros and fill ratio versus the stamped
 *    pattern for a pure band, a locality-plus-rails circuit shape, and the
 *    expander-like pattern that broke the pre-AMD greedy ordering.
 *
 * Run with: pnpm run bench
 */
import { performance } from "node:perf_hooks";

import type { LinearSystem } from "../src/sim/engine/linear-system.js";
import {
  SPARSE_BACKEND_THRESHOLD,
  setLinearSystemBackendForTests,
} from "../src/sim/engine/linear-system.js";
import { MNA } from "../src/sim/engine/mna.js";
import type { SimCircuit } from "../src/sim/engine/sim-engine.js";
import { SimEngine } from "../src/sim/engine/sim-engine.js";
import { SparseMNA } from "../src/sim/engine/sparse-mna.js";

function median(values: number[]): number {
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.floor(sorted.length / 2)]!;
}

// LCG instead of Math.random so the stamped pattern (and therefore fill-in
// and pivoting work) is identical across runs and across backends.
let seed = 0x2f6e2b1 >>> 0;
function rnd(): number {
  seed = (seed * 1664525 + 1013904223) >>> 0;
  return seed / 0x100000000;
}

// ---------------------------------------------------------------------------
// Section 1: kernel factor / reuse-solve medians
// ---------------------------------------------------------------------------

/**
 * Stamp an MNA-like pattern: a tridiagonal ladder plus one deterministic
 * long-range coupling per row (mimicking supply rails and multi-terminal
 * devices), so the sparse backend sees realistic fill-in rather than a
 * pure band it could factor unrealistically fast.
 *
 * The optional `cells` set records every distinct (row, col) touched so
 * section 5 can compute an exact pattern nnz without re-deriving the stamp
 * logic; passing it does not change what gets stamped, keeping sections 1-3
 * byte-comparable with the A1 run.
 */
function stampLadder(sys: LinearSystem, n: number, jitter: number, cells?: Set<number>): void {
  sys.clear();
  for (let i = 0; i < n; i++) {
    sys.add(i, i, 2.5 + jitter * (i + 1));
    cells?.add(i * n + i);
    if (i + 1 < n) {
      sys.add(i, i + 1, -1);
      sys.add(i + 1, i, -1);
      cells?.add(i * n + i + 1);
      cells?.add((i + 1) * n + i);
    }
    const j = Math.floor(rnd() * n);
    if (j !== i) {
      sys.add(i, j, -0.01);
      sys.add(j, i, -0.01);
      sys.add(i, i, 0.01);
      sys.add(j, j, 0.01);
      cells?.add(i * n + j);
      cells?.add(j * n + i);
      cells?.add(j * n + j);
    }
    sys.addB(i, i % 7 === 0 ? 1 : 0);
  }
}

interface KernelTimings {
  factorMs: number;
  reuseSolveMs: number;
}

function benchKernel(sys: LinearSystem, n: number, reps: number): KernelTimings {
  const factorTimes: number[] = [];
  for (let r = 0; r < reps; r++) {
    // Reset the PRNG so every rep stamps the same pattern; only the jitter
    // magnitude changes, which is enough to defeat the exact-match reuse
    // check and force a genuine refactorization.
    seed = 12345;
    stampLadder(sys, n, 1e-9 * (r + 1));
    const t0 = performance.now();
    sys.solve();
    factorTimes.push(performance.now() - t0);
  }
  // Reuse path: same factorized matrix, new RHS -> substitution only.
  const rhs = new Float64Array(n).fill(0.5);
  const reuseTimes: number[] = [];
  for (let r = 0; r < reps; r++) {
    const t0 = performance.now();
    sys.solveRhs(rhs);
    reuseTimes.push(performance.now() - t0);
  }
  return { factorMs: median(factorTimes), reuseSolveMs: median(reuseTimes) };
}

function kernelReps(n: number): number {
  return n >= 3200 ? 3 : n >= 800 ? 3 : 7;
}

function runKernelSection(): void {
  console.log("## Section 1: kernel factor / reuse-solve (median ms)\n");
  console.log("| n | dense factor | dense reuse | sparse factor | sparse reuse |");
  console.log("| ---: | ---: | ---: | ---: | ---: |");
  const denseSizes = new Set([100, 200, 400, 800, 1600]);
  for (const n of [100, 200, 400, 800, 1600, 3200, 6400]) {
    const reps = kernelReps(n);
    // Dense at n > 1600 is O(n^3) with no sparsity to exploit; skip rather
    // than burn minutes confirming the obvious.
    const dense = denseSizes.has(n) ? benchKernel(new MNA(n), n, reps) : null;
    const sparse = benchKernel(new SparseMNA(n), n, reps);
    const denseFactor = dense ? dense.factorMs.toFixed(3) : "-";
    const denseReuse = dense ? dense.reuseSolveMs.toFixed(4) : "-";
    console.log(
      `| ${n} | ${denseFactor} | ${denseReuse} | ${sparse.factorMs.toFixed(3)} | ${sparse.reuseSolveMs.toFixed(4)} |`,
    );
  }
}

// ---------------------------------------------------------------------------
// Section 2: SimEngine end-to-end on an RC ladder
// ---------------------------------------------------------------------------

function ladderCircuit(stages: number): SimCircuit {
  const components: SimCircuit["components"] = [
    {
      id: "src",
      kind: "voltage_source",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { voltage: 5 },
    },
  ];
  const wires: SimCircuit["wires"] = [];
  for (let i = 0; i < stages; i++) {
    components.push({
      id: `r${i}`,
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 100 },
    });
    components.push({
      id: `c${i}`,
      kind: "capacitor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { capacitance: 1e-6 },
    });
    wires.push(
      i === 0
        ? { from_component: "src", from_pin: "pos", to_component: "r0", to_pin: "a" }
        : { from_component: `r${i - 1}`, from_pin: "b", to_component: `r${i}`, to_pin: "a" },
    );
    wires.push({ from_component: `r${i}`, from_pin: "b", to_component: `c${i}`, to_pin: "a" });
    wires.push({ from_component: `c${i}`, from_pin: "b", to_component: "src", to_pin: "neg" });
  }
  return { components, wires, environment: { temperatureC: 25 } };
}

interface EngineTimings {
  msPerStep: number;
  wallMs: number;
}

function benchEngine(stages: number, backend: "dense" | "sparse", steps: number): EngineTimings {
  setLinearSystemBackendForTests(backend);
  try {
    const engine = new SimEngine();
    engine.load(ladderCircuit(stages));
    const h = 1e-5;
    // Warm-up covers the first factorization and JIT so the timed loop
    // measures the steady-state step cost the worker actually pays.
    for (let i = 0; i < 10; i++) engine.step(h);
    const t0 = performance.now();
    for (let i = 0; i < steps; i++) engine.step(h);
    const wallMs = performance.now() - t0;
    return { msPerStep: wallMs / steps, wallMs };
  } finally {
    setLinearSystemBackendForTests(null);
  }
}

function engineSteps(stages: number): number {
  return stages >= 800 ? 30 : stages >= 400 ? 50 : stages >= 200 ? 100 : 300;
}

function runEngineSection(): void {
  console.log("\n## Section 2: SimEngine end-to-end, RC ladder, h=10us (ms/step)\n");
  console.log("| stages | dense ms/step | sparse ms/step | sparse speedup |");
  console.log("| ---: | ---: | ---: | ---: |");
  // Once a dense run blows the wall-clock budget there is no information in
  // even larger dense sizes; stop the dense column at 800 stages.
  const denseWallBudgetMs = 10_000;
  let denseCapped = false;
  for (const stages of [50, 100, 200, 400, 800, 1600]) {
    const steps = engineSteps(stages);
    const skipDense = denseCapped && stages > 800;
    const dense = skipDense ? null : benchEngine(stages, "dense", steps);
    if (dense && dense.wallMs > denseWallBudgetMs) denseCapped = true;
    const sparse = benchEngine(stages, "sparse", steps);
    const denseCell = dense ? dense.msPerStep.toFixed(3) : "-";
    const speedup = dense ? `${(dense.msPerStep / sparse.msPerStep).toFixed(2)}x` : "-";
    console.log(`| ${stages} | ${denseCell} | ${sparse.msPerStep.toFixed(3)} | ${speedup} |`);
  }
}

// ---------------------------------------------------------------------------
// Section 3: dense/sparse factor crossover
// ---------------------------------------------------------------------------

function runCrossoverSection(): void {
  console.log("\n## Section 3: factor crossover scan (median ms)\n");
  console.log("| n | dense factor | sparse factor | winner |");
  console.log("| ---: | ---: | ---: | :--- |");
  let crossover: number | null = null;
  for (let n = 32; n <= 256; n += 16) {
    // More reps than section 1: at these sizes a single factor is tens of
    // microseconds, so the median needs a larger sample to be stable.
    const reps = 15;
    const dense = benchKernel(new MNA(n), n, reps);
    const sparse = benchKernel(new SparseMNA(n), n, reps);
    const sparseWins = sparse.factorMs < dense.factorMs;
    if (sparseWins && crossover === null) crossover = n;
    console.log(
      `| ${n} | ${dense.factorMs.toFixed(4)} | ${sparse.factorMs.toFixed(4)} | ${sparseWins ? "sparse" : "dense"} |`,
    );
  }
  console.log("");
  if (crossover === null) {
    console.log(
      `Recommendation: sparse never beat dense up to n=256; keep SPARSE_BACKEND_THRESHOLD at ${SPARSE_BACKEND_THRESHOLD} or raise it.`,
    );
  } else {
    console.log(
      `Recommendation: sparse factor first beats dense at n=${crossover}; ` +
        `SPARSE_BACKEND_THRESHOLD is currently ${SPARSE_BACKEND_THRESHOLD}.`,
    );
  }
}

// ---------------------------------------------------------------------------
// Section 4: refactorization fast path (A1.5)
// ---------------------------------------------------------------------------

interface RefactorTimings {
  coldFactorMs: number;
  refactorMs: number;
  replayShare: number;
}

/**
 * Cold path versus replay path on the section-1 pattern at a fixed n.
 *
 * Cold uses a fresh backend per rep so every timed solve pays symbolic
 * analysis + AMD + a fully pivoted elimination — the cost the engine pays
 * after a topology edit, and the honest denominator for the replay speedup
 * (there is no public hook to suppress the replay on a warm instance).
 * Steady mimics a Newton iteration exactly: one warm-up factorization, then
 * value-only jittered restamps of the unchanged pattern, which the backend
 * should serve via the stored-pivot replay with no reachability DFS.
 */
function benchRefactorPath(n: number): RefactorTimings {
  const coldReps = n >= 6400 ? 3 : n >= 1600 ? 5 : 7;
  const coldTimes: number[] = [];
  for (let r = 0; r < coldReps; r++) {
    const sys = new SparseMNA(n);
    seed = 12345;
    stampLadder(sys, n, 1e-9 * (r + 1));
    const t0 = performance.now();
    sys.solve();
    coldTimes.push(performance.now() - t0);
  }

  const steadyReps = n >= 6400 ? 7 : n >= 1600 ? 15 : 30;
  const sys = new SparseMNA(n);
  seed = 12345;
  stampLadder(sys, n, 0);
  sys.solve();
  const replaysBefore = sys.refactorizationCount;
  const steadyTimes: number[] = [];
  for (let r = 0; r < steadyReps; r++) {
    // Distinct jitter per rep defeats the exact-match LU reuse, so each
    // timed solve is a genuine numeric refactorization, never a free ride.
    seed = 12345;
    stampLadder(sys, n, 1e-9 * (r + 1));
    const t0 = performance.now();
    sys.solve();
    steadyTimes.push(performance.now() - t0);
  }
  // The share proves the replay actually ran; a silent fallback to the full
  // pivoting pass would make the ms/solve column mislabelled, not just slow.
  const replayShare = (sys.refactorizationCount - replaysBefore) / steadyReps;
  return {
    coldFactorMs: median(coldTimes),
    refactorMs: median(steadyTimes),
    replayShare,
  };
}

function runRefactorSection(): void {
  console.log("\n## Section 4: refactorization fast path, fixed pattern (median ms)\n");
  console.log("| n | cold factor (symbolic+LU) | refactor ms/solve | speedup | replay share |");
  console.log("| ---: | ---: | ---: | ---: | ---: |");
  for (const n of [400, 1600, 6400]) {
    const t = benchRefactorPath(n);
    console.log(
      `| ${n} | ${t.coldFactorMs.toFixed(3)} | ${t.refactorMs.toFixed(3)} | ` +
        `${(t.coldFactorMs / t.refactorMs).toFixed(2)}x | ${(t.replayShare * 100).toFixed(0)}% |`,
    );
  }
}

// ---------------------------------------------------------------------------
// Section 5: fill metrics per pattern (A1.5)
// ---------------------------------------------------------------------------

/**
 * Pure tridiagonal band: an RC ladder with no cross-coupling. Essentially
 * zero fill under any sane ordering, so it anchors the table's lower bound.
 */
function stampTridiagonal(sys: LinearSystem, n: number, jitter: number, cells: Set<number>): void {
  sys.clear();
  for (let i = 0; i < n; i++) {
    sys.add(i, i, 2.5 + jitter * (i + 1));
    cells.add(i * n + i);
    if (i + 1 < n) {
      sys.add(i, i + 1, -1);
      sys.add(i + 1, i, -1);
      cells.add(i * n + i + 1);
      cells.add((i + 1) * n + i);
    }
    sys.addB(i, i % 7 === 0 ? 1 : 0);
  }
}

/**
 * Circuit-like pattern: local wiring plus shared rails. Each row couples to
 * a neighbour within a short random span (components connect nearby nets)
 * and every 16th row couples to node 0 (a supply hub) — the realistic-span
 * shape the A1 kernel report measured at ~0.5 ms per factor at n=1600.
 */
function stampCircuitLike(sys: LinearSystem, n: number, jitter: number, cells: Set<number>): void {
  sys.clear();
  for (let i = 0; i < n; i++) {
    sys.add(i, i, 2.5 + jitter * (i + 1));
    cells.add(i * n + i);
    if (i + 1 < n) {
      sys.add(i, i + 1, -1);
      sys.add(i + 1, i, -1);
      cells.add(i * n + i + 1);
      cells.add((i + 1) * n + i);
    }
    const j = Math.min(n - 1, i + 1 + Math.floor(rnd() * 64));
    if (j !== i) {
      sys.add(i, j, -0.01);
      sys.add(j, i, -0.01);
      sys.add(j, j, 0.01);
      cells.add(i * n + j);
      cells.add(j * n + i);
      cells.add(j * n + j);
    }
    if (i % 16 === 0 && i !== 0) {
      sys.add(i, 0, -0.02);
      sys.add(0, i, -0.02);
      sys.add(0, 0, 0.02);
      cells.add(i * n);
      cells.add(i);
      cells.add(0);
    }
    sys.addB(i, i % 7 === 0 ? 1 : 0);
  }
}

interface FillMetrics {
  patternNnz: number;
  luNnz: number;
  factorMs: number;
}

/**
 * Fresh backend per rep so the timed solve is always the cold factorization
 * whose fill the L/U counters describe; a warm instance would replay and
 * report a time the fill numbers do not explain.
 */
function measureFill(
  stamp: (sys: LinearSystem, n: number, jitter: number, cells: Set<number>) => void,
  n: number,
): FillMetrics {
  const reps = n >= 6400 ? 3 : 5;
  const times: number[] = [];
  let patternNnz = 0;
  let luNnz = 0;
  for (let r = 0; r < reps; r++) {
    const sys = new SparseMNA(n);
    const cells = new Set<number>();
    seed = 12345;
    stamp(sys, n, 1e-9 * (r + 1), cells);
    const t0 = performance.now();
    sys.solve();
    times.push(performance.now() - t0);
    patternNnz = cells.size;
    // lCount/uCount are private diagnostics; a structural cast keeps them
    // off the LinearSystem contract, which a benchmark must not widen.
    const internals = sys as unknown as { lCount: number; uCount: number };
    // L stores its unit diagonal implicitly and U keeps pivots in a separate
    // array, so the shared diagonal contributes exactly n entries once.
    luNnz = internals.lCount + internals.uCount + n;
  }
  return { patternNnz, luNnz, factorMs: median(times) };
}

function runFillSection(): void {
  console.log("\n## Section 5: fill metrics per pattern (fresh factor, median ms)\n");
  console.log("| pattern | n | pattern nnz | L+U nnz | fill ratio | factor ms |");
  console.log("| :--- | ---: | ---: | ---: | ---: | ---: |");
  const patterns: Array<
    [string, (sys: LinearSystem, n: number, jitter: number, cells: Set<number>) => void]
  > = [
    ["ladder", stampTridiagonal],
    ["circuit-like", stampCircuitLike],
    // The section-1 stamp IS the expander case: unbounded random chords are
    // the pattern that forced the pre-AMD greedy ordering into ~100x fill.
    ["expander", (sys, n, jitter, cells) => stampLadder(sys, n, jitter, cells)],
  ];
  for (const [name, stamp] of patterns) {
    for (const n of [400, 1600, 6400]) {
      const m = measureFill(stamp, n);
      console.log(
        `| ${name} | ${n} | ${m.patternNnz} | ${m.luNnz} | ` +
          `${(m.luNnz / m.patternNnz).toFixed(2)} | ${m.factorMs.toFixed(3)} |`,
      );
    }
  }
}

runKernelSection();
runEngineSection();
runCrossoverSection();
runRefactorSection();
runFillSection();
