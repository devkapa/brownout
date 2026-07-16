/**
 * Approximate minimum degree ordering on a quotient graph, after Amestoy,
 * Davis and Duff ("An Approximate Minimum Degree Ordering Algorithm", SIAM
 * J. Matrix Anal. Appl. 17(4), 1996) and the SuiteSparse AMD reference
 * implementation.
 *
 * The quotient graph replaces the explicit fill edges of classical minimum
 * degree with elements: eliminating pivot p turns it into an element whose
 * variable list L_p is the union of p's surviving variable neighbors and the
 * variable lists of the elements adjacent to p, which are absorbed (their
 * lists die) in the same step. Cliques are therefore represented by one list
 * instead of O(|clique|^2) edges, which is what makes hub/supply-rail rows
 * affordable without the work budgets the previous greedy ordering needed.
 *
 * Implemented from the AMD toolbox:
 * - approximate external degrees via the three-way upper bound
 *   d_i = min(nLeft - nv_i, d_prev + |L_p \ i|, |A_i| + |L_p \ i| +
 *   sum_e |L_e \ L_p|), with the |L_e \ L_p| terms computed by the two-pass
 *   set-counter scheme (w initialised to |L_e|, decremented per member seen
 *   in L_p);
 * - element absorption, plus aggressive absorption (an element whose live
 *   variables all land in L_p has |L_e \ L_p| = 0 and is deleted even when
 *   it was not adjacent to the pivot);
 * - supervariable detection: variables in L_p are hashed by the sum of their
 *   adjacency lists mod n, then compared exactly; indistinguishable ones are
 *   merged and emitted consecutively when their representative leaves;
 * - mass elimination: a variable whose compacted adjacency is exactly the
 *   new element is ordered immediately after the pivot (zero extra fill);
 * - dense-row deferral exactly as in the reference implementation (alpha =
 *   10): rows with more than max(16, 10*sqrt(n)) neighbors are taken out of
 *   the quotient graph up front and appended to the order last. This is the
 *   reference algorithm's structural answer to hubs, not a work budget —
 *   without it every hub update rescans the hub's O(n) list and the
 *   ordering degenerates to quadratic time on star patterns.
 *
 * Not implemented: assembly-tree postordering (the Gilbert-Peierls consumer
 * derives its own topological structure per column, so postorder would only
 * change memory locality, not fill), multiple elimination (that is MMD, not
 * AMD), and exact-degree recomputation.
 *
 * Determinism: the order is a pure function of the input pattern — no
 * randomness, no recursion; all workspaces are typed arrays sized once per
 * call. Index tie-breaks are bucket-local, not global: the initial degree
 * lists are linked in descending index order so bucket heads start lowest,
 * L_p is sorted ascending before it is scanned, hash chains are built
 * ascending so the representative of an indistinguishable set is always its
 * lowest index, and each pivot's update relinks in descending index order so
 * the lowest just-updated index surfaces first. Degree buckets are LIFO,
 * though, so a freshly relinked vertex sits ahead of untouched vertices of
 * equal degree regardless of index — the O(1) head insertion the reference
 * implementation also uses. Which of two equal-degree pivots goes first is
 * an ordering-quality heuristic either way; validity never depends on it.
 */

/** Live principal variable: belongs to a degree list, nv > 0. */
const ST_VAR = 0;
/** Live element: an eliminated pivot's clique list. */
const ST_ELEM = 1;
/** Dead: absorbed element, merged variable, mass-eliminated or emitted pivot. */
const ST_DEAD = 2;
/** Dense variable deferred to the tail of the order (never enters the graph). */
const ST_DENSE = 3;

/**
 * Compute an AMD elimination order for the symmetric pattern given as a
 * deduplicated adjacency structure WITHOUT self-edges (CSR over vertices;
 * the caller symmetrises A + A^T and strips the diagonal).
 *
 * Returns `order` with order[k] = vertex eliminated at step k (bijective).
 * The permutation postcondition is checked on every call and violations
 * throw: a non-bijective order would silently scramble the factorization
 * coordinate system downstream, and the O(n) check is negligible next to
 * the ordering work itself (same rationale as SparseMNA's own guard).
 */
export function amdOrder(n: number, adjPtr: Int32Array, adjIdx: Int32Array): Int32Array {
  const order = new Int32Array(n);
  if (n === 0) return order;

  const nnz = adjPtr[n];

  // Single pooled workspace for every adjacency/element list. Live memory
  // never exceeds the initial adjacency: a new element list is no longer
  // than the source lists (pivot block plus absorbed elements) that die in
  // the same step, and variable lists only shrink when rebuilt. nnz + n is
  // therefore always enough after a compaction; the extra quarter just makes
  // compactions rare.
  let iw = new Int32Array(nnz + n + 16 + (nnz >> 2));
  iw.set(adjIdx.subarray(0, nnz));
  let pfree = nnz;

  /** Block start of a vertex's list in iw; -1 once the block is garbage. */
  const pe = new Int32Array(n);
  /** Variable: total list length (elements then variables). Element: |L_e|. */
  const len = new Int32Array(n);
  /** Variable: number of element entries at the head of its block. */
  const elen = new Int32Array(n);
  /**
   * Supervariable weight (number of original variables represented). Sign
   * doubles as the L_p membership flag: negated while a variable sits in the
   * pivot's list, zero once dead. Eliminated pivots keep a negative nv so
   * every `nv > 0` liveness test treats them like dead entries.
   */
  const nv = new Int32Array(n);
  /** Variable: approximate external degree. Element: live weight of L_e. */
  const deg = new Int32Array(n);
  const status = new Uint8Array(n);
  /** Two-pass |L_e \ L_p| counters, valid when wGen matches this step. */
  const wVal = new Int32Array(n);
  const wGen = new Int32Array(n);
  /** Scatter marks for exact supervariable comparison. */
  const mark = new Int32Array(n);
  /** Degree buckets as doubly-linked lists (head per degree value). */
  const head = new Int32Array(n).fill(-1);
  const next = new Int32Array(n).fill(-1);
  const prev = new Int32Array(n).fill(-1);
  /** Hash buckets for supervariable detection, rebuilt per pivot. */
  const hashBucket = new Int32Array(n).fill(-1);
  const hashNext = new Int32Array(n);
  const hashOf = new Int32Array(n).fill(-1);
  /** Merged-member chains so a representative emits its set consecutively. */
  const svNext = new Int32Array(n).fill(-1);
  const svTail = new Int32Array(n);
  /** Mass-eliminated variables of the current step, in ascending order. */
  const massBuf = new Int32Array(n);
  let gen = 0;
  let markGen = 0;

  for (let v = 0; v < n; v++) {
    pe[v] = adjPtr[v];
    len[v] = adjPtr[v + 1] - adjPtr[v];
    deg[v] = len[v];
    nv[v] = 1;
    svTail[v] = v;
  }

  // Dense-row deferral (reference AMD, alpha = 10). Deferred rows leave the
  // quotient graph entirely: nv = 0 makes every list reference to them skip
  // lazily, and they are appended to the order after the last live pivot.
  const denseThreshold = Math.max(16, 10 * Math.sqrt(n));
  let liveVars = 0;
  let nElim = 0;
  for (let v = 0; v < n; v++) {
    if (len[v] > denseThreshold) {
      status[v] = ST_DENSE;
      nv[v] = 0;
      pe[v] = -1;
      len[v] = 0;
      // Counted as gone so the nLeft degree cap tracks the live subgraph.
      nElim += 1;
    } else {
      liveVars += 1;
    }
  }

  const link = (v: number, d: number): void => {
    prev[v] = -1;
    next[v] = head[d];
    if (head[d] >= 0) prev[head[d]] = v;
    head[d] = v;
  };
  const unlink = (v: number, d: number): void => {
    if (prev[v] >= 0) next[prev[v]] = next[v];
    else head[d] = next[v];
    if (next[v] >= 0) prev[next[v]] = prev[v];
  };
  // Descending build order leaves the lowest index at each bucket head.
  for (let v = n - 1; v >= 0; v--) {
    if (status[v] === ST_VAR) link(v, deg[v]);
  }

  /**
   * Mark-and-slide compaction (the reference implementation's sentinel
   * trick): the first word of each live block temporarily moves into pe and
   * a negative sentinel encodes the owner, so one left-to-right sweep can
   * slide every live block down in address order.
   */
  const compactPool = (): void => {
    for (let v = 0; v < n; v++) {
      const st = status[v];
      if ((st === ST_VAR || st === ST_ELEM) && len[v] > 0) {
        const start = pe[v];
        pe[v] = iw[start];
        // List entries are vertex ids >= 0, so <= -2 is unambiguous.
        iw[start] = -v - 2;
      }
    }
    let src = 0;
    let dst = 0;
    while (src < pfree) {
      const x = iw[src++];
      if (x <= -2) {
        const v = -x - 2;
        iw[dst] = pe[v];
        pe[v] = dst;
        dst += 1;
        const rest = len[v] - 1;
        for (let t = 0; t < rest; t++) iw[dst++] = iw[src++];
      }
    }
    pfree = dst;
  };

  const ensureSpace = (needed: number): void => {
    if (pfree + needed <= iw.length) return;
    compactPool();
    if (pfree + needed <= iw.length) return;
    // Unreachable per the live-memory bound above, but a defensive grow
    // beats a wrong order if that argument is ever violated.
    const grown = new Int32Array(Math.max(iw.length * 2, pfree + needed));
    grown.set(iw.subarray(0, pfree));
    iw = grown;
  };

  let outCount = 0;
  const emit = (v: number): void => {
    order[outCount++] = v;
    for (let m = svNext[v]; m >= 0; m = svNext[m]) order[outCount++] = m;
  };

  let minDeg = 0;
  while (liveVars > 0) {
    // ---- Pivot selection: minimum approximate degree, lowest index ------
    while (minDeg < n && head[minDeg] < 0) minDeg += 1;
    if (minDeg >= n) break; // unreachable; the postcondition reports if not
    const p = head[minDeg];
    unlink(p, deg[p]);
    liveVars -= 1;
    const nvPivot = nv[p];
    nElim += nvPivot;

    // ---- Build the pivot element L_p ------------------------------------
    // Space first, while p's block is still live for the compaction sweep;
    // |L_p| is bounded by the number of remaining live variables.
    ensureSpace(liveVars);
    const pme = pfree;
    // Negative nv flags membership so union construction deduplicates and
    // the pivot itself can never re-enter through an element list.
    nv[p] = -nvPivot;
    const pStart = pe[p];
    const pElen = elen[p];
    const pLen = len[p];
    for (let t = 0; t < pElen; t++) {
      const e = iw[pStart + t];
      if (status[e] !== ST_ELEM) continue;
      const eStart = pe[e];
      const eLen = len[e];
      for (let q = 0; q < eLen; q++) {
        const j = iw[eStart + q];
        if (nv[j] > 0) {
          unlink(j, deg[j]);
          nv[j] = -nv[j];
          iw[pfree++] = j;
        }
      }
      // Element absorption: e's variables now live in L_p, its list dies.
      status[e] = ST_DEAD;
      pe[e] = -1;
    }
    for (let t = pElen; t < pLen; t++) {
      const j = iw[pStart + t];
      if (nv[j] > 0) {
        unlink(j, deg[j]);
        nv[j] = -nv[j];
        iw[pfree++] = j;
      }
    }
    const lpLen = pfree - pme;
    status[p] = ST_DEAD;
    pe[p] = -1;
    if (lpLen === 0) {
      // Isolated pivot (or fully covered by dead neighbors): no element.
      emit(p);
      continue;
    }
    status[p] = ST_ELEM;
    pe[p] = pme;
    len[p] = lpLen;
    elen[p] = 0;
    // Ascending L_p fixes the scan order, hash-chain order and emit order,
    // making every tie-break below index-deterministic.
    iw.subarray(pme, pfree).sort();
    const lpEnd = pme + lpLen;

    // ---- Pass 1: set counters w[e] = |L_e \ L_p| in weight units --------
    gen += 1;
    for (let idx = pme; idx < lpEnd; idx++) {
      const i = iw[idx];
      const iStart = pe[i];
      const iElen = elen[i];
      for (let t = 0; t < iElen; t++) {
        const e = iw[iStart + t];
        if (status[e] !== ST_ELEM) continue; // absorbed by p just now
        if (wGen[e] !== gen) {
          wGen[e] = gen;
          wVal[e] = deg[e]; // live weight of L_e
        }
        // nv[i] is negated while i is in L_p; this subtracts i's weight.
        wVal[e] += nv[i];
      }
    }

    // ---- Pass 2: approximate degrees, list compaction, mass elimination -
    let massCount = 0;
    for (let idx = pme; idx < lpEnd; idx++) {
      const i = iw[idx];
      const p1 = pe[i];
      const iElen = elen[i];
      const iLen = len[i];
      let pdst = p1;
      let esum = 0;
      let hash = 0;
      for (let t = 0; t < iElen; t++) {
        const e = iw[p1 + t];
        if (status[e] !== ST_ELEM) continue;
        const we = wVal[e]; // pass 1 visited (i, e), so the counter is valid
        if (we === 0) {
          // Aggressive absorption: every live variable of L_e is in L_p, so
          // e is covered by the new element and disappears.
          status[e] = ST_DEAD;
          pe[e] = -1;
        } else {
          esum += we;
          hash = (hash + e) % n;
          iw[pdst++] = e;
        }
      }
      const keptElems = pdst - p1;
      let asum = 0;
      for (let t = iElen; t < iLen; t++) {
        const j = iw[p1 + t];
        // nv < 0: j is in L_p, pruned (now reached through element p).
        // nv === 0: dead (merged, mass-eliminated, dense or a former pivot).
        if (nv[j] > 0) {
          asum += nv[j];
          hash = (hash + j) % n;
          iw[pdst++] = j;
        }
      }
      const keptVars = pdst - p1 - keptElems;
      // Insert p at the head of the element part in O(1): the first kept
      // variable moves to the end, the first kept element into its slot.
      // At least one source entry died (the direct edge to p or a shared
      // absorbed element), so the rebuilt list fits in the old block.
      if (keptVars > 0) {
        iw[pdst++] = iw[p1 + keptElems];
        iw[p1 + keptElems] = keptElems > 0 ? iw[p1] : p;
        if (keptElems > 0) iw[p1] = p;
      } else {
        iw[pdst++] = keptElems > 0 ? iw[p1] : p;
        if (keptElems > 0) iw[p1] = p;
      }
      elen[i] = keptElems + 1;
      len[i] = keptElems + 1 + keptVars;
      const dExt = esum + asum;
      if (dExt === 0) {
        // Mass elimination: i's entire adjacency collapsed into the new
        // element, so ordering i directly after p adds zero fill and can
        // never help later. Removed from the graph now; emitted below.
        status[i] = ST_DEAD;
        pe[i] = -1;
        nElim += -nv[i];
        nv[i] = 0;
        liveVars -= 1;
        massBuf[massCount++] = i;
      } else {
        // Approximate-degree bound, part 1: min(previous bound, external
        // work). |L_p \ i| joins in the finalize pass, once mass
        // eliminations have settled the final element weight.
        deg[i] = Math.min(deg[i], dExt);
        hashOf[i] = hash;
      }
    }

    // ---- Pass 3: supervariable detection over the survivors -------------
    // Chains are built by walking L_p descending so each chain is ascending
    // and its head — the comparison anchor — is the lowest index, which
    // therefore becomes the representative of anything merged into it.
    for (let idx = lpEnd - 1; idx >= pme; idx--) {
      const i = iw[idx];
      if (nv[i] >= 0) continue; // mass-eliminated this step
      const b = hashOf[i];
      hashNext[i] = hashBucket[b];
      hashBucket[b] = i;
    }
    if (markGen >= 0x7ffffff0) {
      mark.fill(0);
      markGen = 0;
    }
    for (let idx = pme; idx < lpEnd; idx++) {
      const first = iw[idx];
      if (nv[first] >= 0 || hashOf[first] < 0) continue; // dead or processed
      const b = hashOf[first];
      let anchor = hashBucket[b];
      hashBucket[b] = -1;
      while (anchor >= 0) {
        hashOf[anchor] = -1;
        let cand = hashNext[anchor];
        if (cand >= 0) {
          markGen += 1;
          const g = markGen;
          const aStart = pe[anchor];
          const aElen = elen[anchor];
          const aLen = len[anchor];
          // Element ids and variable ids never collide (a vertex is one or
          // the other), so one scatter covers both list segments.
          for (let t = 0; t < aLen; t++) mark[iw[aStart + t]] = g;
          let candPrev = anchor;
          while (cand >= 0) {
            const candNext = hashNext[cand];
            let same = elen[cand] === aElen && len[cand] === aLen;
            if (same) {
              const cStart = pe[cand];
              for (let t = 0; t < aLen; t++) {
                if (mark[iw[cStart + t]] !== g) {
                  same = false;
                  break;
                }
              }
            }
            if (same) {
              // Indistinguishable in the quotient graph: merge cand into the
              // lower-indexed anchor. Weights add (both are negated L_p
              // flags); the anchor's stored external degree is unchanged
              // because cand was reachable only through the shared element.
              nv[anchor] += nv[cand];
              nv[cand] = 0;
              status[cand] = ST_DEAD;
              pe[cand] = -1;
              hashOf[cand] = -1;
              liveVars -= 1;
              svNext[svTail[anchor]] = cand;
              svTail[anchor] = svTail[cand];
              hashNext[candPrev] = candNext;
            } else {
              candPrev = cand;
            }
            cand = candNext;
          }
        }
        anchor = hashNext[anchor];
      }
    }

    // ---- Finalize: element weight, degrees, degree-list relinks ---------
    let degme = 0;
    for (let idx = pme; idx < lpEnd; idx++) {
      const i = iw[idx];
      if (nv[i] < 0) degme += -nv[i];
    }
    deg[p] = degme;
    if (degme === 0) {
      // Everything merged or mass-eliminated: the element is empty and no
      // future column can reach it.
      status[p] = ST_DEAD;
      pe[p] = -1;
    }
    const nLeft = n - nElim;
    // Descending walk so the lowest surviving index is relinked last and
    // surfaces at its bucket head for the next pivot selection.
    for (let idx = lpEnd - 1; idx >= pme; idx--) {
      const i = iw[idx];
      if (nv[i] >= 0) continue;
      const nvi = -nv[i];
      nv[i] = nvi;
      // Approximate-degree bound, parts 2 and 3: add |L_p \ i| with the
      // final element weight, cap by the live variables outside i.
      let d = Math.min(deg[i] + degme - nvi, nLeft - nvi);
      if (d < 0) d = 0;
      deg[i] = d;
      link(i, d);
      if (d < minDeg) minDeg = d;
    }

    emit(p);
    for (let t = 0; t < massCount; t++) emit(massBuf[t]);
  }

  // Dense rows come last, in index order: eliminating a hub late confines
  // its (unavoidable) clique to the trailing submatrix.
  for (let v = 0; v < n; v++) {
    if (status[v] === ST_DENSE) emit(v);
  }

  // Always-on postcondition: a non-bijective order would silently scramble
  // the factorization coordinate system, and O(n) here is negligible.
  if (outCount !== n) {
    throw new Error(`amdOrder: emitted ${outCount} of ${n} vertices`);
  }
  const seen = new Uint8Array(n);
  for (let k = 0; k < n; k++) {
    const v = order[k];
    if (v < 0 || v >= n || seen[v] !== 0) {
      throw new Error("amdOrder: output is not a permutation");
    }
    seen[v] = 1;
  }
  return order;
}
