import { describe, expect, it } from "vitest";
// Direct relative import: the AMD preorder is internal to the sparse backend,
// so simcore deliberately exports no amd-ordering subpath. Same pattern as
// sim-worker.test.ts reaching into packages/simcore/src.
import { amdOrder } from "../../../src/sim/engine/amd-ordering.js";

type Edge = readonly [number, number];

/**
 * Build the CSR adjacency amdOrder documents as its precondition: symmetric,
 * deduplicated, no self-edges. Mirrors SparseMNA.computeColumnOrder so the
 * tests exercise the module through the same input shape as production.
 */
function buildCsr(n: number, edges: readonly Edge[]): { adjPtr: Int32Array; adjIdx: Int32Array } {
  const nbr: Array<Set<number>> = Array.from({ length: n }, () => new Set<number>());
  for (const [a, b] of edges) {
    if (a === b) continue;
    nbr[a].add(b);
    nbr[b].add(a);
  }
  const adjPtr = new Int32Array(n + 1);
  for (let v = 0; v < n; v++) adjPtr[v + 1] = adjPtr[v] + nbr[v].size;
  const adjIdx = new Int32Array(adjPtr[n]);
  let out = 0;
  for (let v = 0; v < n; v++) {
    for (const w of nbr[v]) adjIdx[out++] = w;
  }
  return { adjPtr, adjIdx };
}

/** Fails with the offending index rather than a bare boolean so a regression names the vertex. */
function expectPermutation(order: Int32Array, n: number): void {
  expect(order.length).toBe(n);
  const seen = new Uint8Array(n);
  for (let k = 0; k < n; k++) {
    const v = order[k];
    expect(v).toBeGreaterThanOrEqual(0);
    expect(v).toBeLessThan(n);
    expect(seen[v]).toBe(0);
    seen[v] = 1;
  }
}

/**
 * Reference symbolic factorization: play the elimination game on the given
 * order and count fill edges. This is the exact quantity AMD approximates
 * (fill of Cholesky on the symmetrized pattern), computed the slow honest
 * way — eliminate each vertex in turn, clique its surviving neighbors, count
 * every edge that did not already exist. Independent of the implementation
 * under test on purpose: it shares no code or data structures with amdOrder,
 * so a bug in the quotient-graph bookkeeping cannot cancel out here.
 */
function eliminationFill(
  n: number,
  adjPtr: Int32Array,
  adjIdx: Int32Array,
  order: Int32Array,
): number {
  const adj: Array<Set<number>> = Array.from({ length: n }, () => new Set<number>());
  for (let v = 0; v < n; v++) {
    for (let t = adjPtr[v]; t < adjPtr[v + 1]; t++) adj[v].add(adjIdx[t]);
  }
  let fill = 0;
  for (let k = 0; k < n; k++) {
    const v = order[k];
    const nbrs = [...adj[v]];
    // Detach v first so the sets only ever hold uneliminated vertices.
    for (const u of nbrs) adj[u].delete(v);
    for (let a = 0; a < nbrs.length; a++) {
      for (let b = a + 1; b < nbrs.length; b++) {
        const x = nbrs[a];
        const y = nbrs[b];
        if (!adj[x].has(y)) {
          adj[x].add(y);
          adj[y].add(x);
          fill += 1;
        }
      }
    }
    adj[v].clear();
  }
  return fill;
}

/** Deterministic PRNG (mulberry32) so every random case is replayable from its seed. */
function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function pathEdges(n: number): Edge[] {
  const edges: Edge[] = [];
  for (let i = 0; i < n - 1; i++) edges.push([i, i + 1]);
  return edges;
}

function cliqueEdges(n: number): Edge[] {
  const edges: Edge[] = [];
  for (let a = 0; a < n; a++) {
    for (let b = a + 1; b < n; b++) edges.push([a, b]);
  }
  return edges;
}

const STAR_LEAVES = 500;
const STAR_HUB = 0;

/** Hub-and-spoke with the hub at index 0 and 500 leaves: the dense-row deferral case. */
function starCsr(): { n: number; adjPtr: Int32Array; adjIdx: Int32Array } {
  const n = STAR_LEAVES + 1;
  const edges: Edge[] = [];
  for (let leaf = 1; leaf < n; leaf++) edges.push([STAR_HUB, leaf]);
  return { n, ...buildCsr(n, edges) };
}

describe("amdOrder permutation validity", () => {
  it("returns an empty order for the empty graph", () => {
    const { adjPtr, adjIdx } = buildCsr(0, []);
    expect(amdOrder(0, adjPtr, adjIdx).length).toBe(0);
  });

  it("orders a singleton", () => {
    const { adjPtr, adjIdx } = buildCsr(1, []);
    expect([...amdOrder(1, adjPtr, adjIdx)]).toEqual([0]);
  });

  it("permutes a path", () => {
    const n = 50;
    const { adjPtr, adjIdx } = buildCsr(n, pathEdges(n));
    expectPermutation(amdOrder(n, adjPtr, adjIdx), n);
  });

  it("permutes a cycle", () => {
    const n = 60;
    const edges = pathEdges(n);
    edges.push([n - 1, 0]);
    const { adjPtr, adjIdx } = buildCsr(n, edges);
    expectPermutation(amdOrder(n, adjPtr, adjIdx), n);
  });

  it("permutes a star hub with 500 leaves", () => {
    const { n, adjPtr, adjIdx } = starCsr();
    expectPermutation(amdOrder(n, adjPtr, adjIdx), n);
  });

  it("permutes the clique K30", () => {
    const n = 30;
    const { adjPtr, adjIdx } = buildCsr(n, cliqueEdges(n));
    expectPermutation(amdOrder(n, adjPtr, adjIdx), n);
  });

  it("permutes two disconnected blocks", () => {
    // A path block and a cycle block that never touch: exercises the outer
    // pivot loop draining one component after the other.
    const n = 20;
    const edges: Edge[] = [];
    for (let i = 0; i < 9; i++) edges.push([i, i + 1]);
    for (let i = 10; i < 19; i++) edges.push([i, i + 1]);
    edges.push([19, 10]);
    const { adjPtr, adjIdx } = buildCsr(n, edges);
    expectPermutation(amdOrder(n, adjPtr, adjIdx), n);
  });

  it("permutes 20 seeded random sparse graphs up to n=400", () => {
    const rand = mulberry32(0xdeadbeef);
    for (let c = 0; c < 20; c++) {
      const n = 20 + Math.floor(rand() * 381);
      const m = Math.floor(n * (1 + rand() * 2));
      const edges: Edge[] = [];
      for (let e = 0; e < m; e++) {
        edges.push([Math.floor(rand() * n), Math.floor(rand() * n)]);
      }
      const { adjPtr, adjIdx } = buildCsr(n, edges);
      expectPermutation(amdOrder(n, adjPtr, adjIdx), n);
    }
  });

  it("permutes a graph with an isolated vertex alongside a dense row", () => {
    // Vertex 0 never appears in any list (degree 0) while vertex 1 crosses
    // the dense-row threshold (250 > 10*sqrt(350)): the order must include
    // both the vertex the quotient graph never touches and the one it
    // deferred to the tail.
    const n = 350;
    const edges: Edge[] = [];
    for (let v = 50; v < 300; v++) edges.push([1, v]);
    for (let v = 10; v < 339; v++) edges.push([v, v + 1]);
    const { adjPtr, adjIdx } = buildCsr(n, edges);
    expectPermutation(amdOrder(n, adjPtr, adjIdx), n);
  });
});

describe("amdOrder determinism", () => {
  it("returns the identical order for the same random input twice", () => {
    const rand = mulberry32(42);
    const n = 300;
    const edges: Edge[] = [];
    for (let e = 0; e < 700; e++) {
      edges.push([Math.floor(rand() * n), Math.floor(rand() * n)]);
    }
    const { adjPtr, adjIdx } = buildCsr(n, edges);
    const first = amdOrder(n, adjPtr, adjIdx);
    const second = amdOrder(n, adjPtr, adjIdx);
    expect([...second]).toEqual([...first]);
  });

  it("returns the identical order for the star hub twice", () => {
    // The dense-deferral path bypasses the degree lists entirely; it must be
    // just as reproducible as the quotient-graph path.
    const { n, adjPtr, adjIdx } = starCsr();
    const first = amdOrder(n, adjPtr, adjIdx);
    const second = amdOrder(n, adjPtr, adjIdx);
    expect([...second]).toEqual([...first]);
  });
});

describe("amdOrder fill quality", () => {
  it("orders a 500-node ladder with zero fill", () => {
    // A resistor-ladder chain is a tree: a true minimum-degree order always
    // has a degree-1 leaf available, so any fill at all means the
    // approximate degrees drifted from the exact ones on the easiest
    // possible input.
    const n = 500;
    const { adjPtr, adjIdx } = buildCsr(n, pathEdges(n));
    const order = amdOrder(n, adjPtr, adjIdx);
    expect(eliminationFill(n, adjPtr, adjIdx, order)).toBe(0);
  });

  it("keeps 20x20 grid fill under the loose AMD-class bound", () => {
    const s = 20;
    const n = s * s;
    const edges: Edge[] = [];
    for (let r = 0; r < s; r++) {
      for (let c = 0; c < s; c++) {
        const v = r * s + c;
        if (c + 1 < s) edges.push([v, v + 1]);
        if (r + 1 < s) edges.push([v, v + s]);
      }
    }
    const { adjPtr, adjIdx } = buildCsr(n, edges);
    const order = amdOrder(n, adjPtr, adjIdx);
    const fill = eliminationFill(n, adjPtr, adjIdx, order);
    // 4n log2(n) is deliberately loose (about 13.8k for n=400; a good AMD
    // run lands near 2.6k): it will not flap on legitimate tie-break
    // changes, but a degenerate order — worst case fill for this grid is
    // O(n^2), about 80k — sails past it.
    expect(fill).toBeGreaterThan(0);
    expect(fill).toBeLessThan(4 * n * Math.log2(n));
  });

  it("defers the star hub to the last position and produces zero fill", () => {
    // Leaves only ever touch the hub, so eliminating them first adds no
    // edges, and the hub's clique is empty by the time it pivots. Fill 0 is
    // only reachable if the hub really is ordered last, so assert both: the
    // position check names the failure, the fill check proves the cost.
    const { n, adjPtr, adjIdx } = starCsr();
    const order = amdOrder(n, adjPtr, adjIdx);
    expect(order[n - 1]).toBe(STAR_HUB);
    expect(eliminationFill(n, adjPtr, adjIdx, order)).toBe(0);
  });

  it("orders the clique K30 with zero fill", () => {
    // Every order of a complete graph is perfect; nonzero fill here would
    // mean the reference factorization itself is broken, so this doubles as
    // a self-check of the elimination game.
    const n = 30;
    const { adjPtr, adjIdx } = buildCsr(n, cliqueEdges(n));
    const order = amdOrder(n, adjPtr, adjIdx);
    expect(eliminationFill(n, adjPtr, adjIdx, order)).toBe(0);
  });
});

describe("amdOrder supervariable behavior", () => {
  it("collapses 10 indistinguishable twins and emits them consecutively", () => {
    // Vertex 0 has the unique minimum degree (10 vs 12 for the twins and 10+
    // for the shared set, ties breaking to index 0), so it pivots first and
    // its element list is exactly the twins. Each twin's compacted adjacency
    // is then {element 0} plus the same 11 shared vertices, so hashing and
    // exact comparison must merge all ten into the lowest-indexed
    // representative, and the merged set leaves the order as one block.
    const n = 22;
    const edges: Edge[] = [];
    for (let t = 1; t <= 10; t++) {
      edges.push([0, t]);
      for (let s = 11; s <= 21; s++) edges.push([t, s]);
    }
    const { adjPtr, adjIdx } = buildCsr(n, edges);
    const order = amdOrder(n, adjPtr, adjIdx);
    expectPermutation(order, n);

    const positions: number[] = [];
    for (let k = 0; k < n; k++) {
      if (order[k] >= 1 && order[k] <= 10) positions.push(k);
    }
    expect(positions.length).toBe(10);
    // Consecutive slots prove the collapse: unmerged twins would be
    // interleaved with the shared-set eliminations that separate them.
    expect(positions[9] - positions[0]).toBe(9);
    // Documented tie-break: the representative is the lowest index and the
    // chain is built ascending, so the block reads 1..10 in order.
    for (let t = 0; t < 10; t++) {
      expect(order[positions[0] + t]).toBe(t + 1);
    }
  });
});

describe("amdOrder adversarial input", () => {
  it("tolerates duplicate edges under the always-on bijectivity postcondition", () => {
    // The documented contract puts deduplication on the caller, so duplicate
    // entries are a precondition violation, not an error the module promises
    // to detect. What it does promise — unconditionally — is the checked
    // postcondition: any order that returns is bijective. Duplicates only
    // inflate the initial degree counts, so the order must still come back
    // valid rather than scrambled.
    const n = 4;
    // Symmetric star around 0 with the 0-1 edge duplicated on both sides,
    // plus a 1-3 and 2-3 chain: 0:[1,1,2], 1:[0,0,3], 2:[0,3], 3:[1,2].
    const adjPtr = new Int32Array([0, 3, 6, 8, 10]);
    const adjIdx = new Int32Array([1, 1, 2, 0, 0, 3, 0, 3, 1, 2]);
    const order = amdOrder(n, adjPtr, adjIdx);
    expectPermutation(order, n);
  });
});
