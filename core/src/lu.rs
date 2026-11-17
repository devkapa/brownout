//! Dense LU with partial pivoting, LINPACK layout (core/README.md, "W0 kernel
//! semantics", point 4). This is not a fused "eliminate the augmented
//! [A|b] system" solver: it factors `A` in place into `L` (unit lower,
//! multipliers stored where the eliminated entries were) and `U` (upper,
//! on and above the diagonal), plus a pivot vector, and a separate solve
//! step applies that factorization to a right-hand side.
//!
//! The layout rule the contract calls out: at elimination step `k`, a row
//! swap moves only columns `k..n` of the two rows. Columns `0..k` already
//! hold multipliers from earlier steps and are left where they are — NOT
//! carried along with the swap. That is deliberate, not an oversight: this
//! is the classic LINPACK DGEFA/DGESL split, where the row swap and the use
//! of a step's multipliers are interleaved during the solve rather than
//! applied as one final permutation. Swapping full rows (including the
//! already-computed columns) during factorization while then applying the
//! interleaved solve below silently produces a wrong answer for any matrix
//! that pivots at more than one step — see `tests::full_row_swap_is_wrong`,
//! which is this module's regression test for exactly that bug (the
//! prototype in brownout-core-prototype hit it).

/// Pivot search treats a non-finite candidate as worse than any finite one
/// (`f64` comparisons with NaN are always false, so a NaN sitting on the
/// diagonal at the start of a step must not survive by default).
fn abs_or_worst(v: f64) -> f64 {
    if v.is_finite() {
        v.abs()
    } else {
        -1.0
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LuFactorization {
    /// n x n, row-major. L's sub-diagonal multipliers and U's on/above
    /// diagonal entries, interleaved in place.
    a: Vec<f64>,
    /// piv[k] is the row (>= k) swapped into position k at step k.
    piv: Vec<usize>,
    n: usize,
    /// Exact mode's block elimination over the rail (ruling R105 part 2):
    /// when set, `a` and `piv` are empty and the system is solved block by
    /// block with the rail as the one shared unknown ([`lu_factor_hub`]).
    hub: Option<Box<HubBlocks>>,
}

/// A system whose unknowns other than `r` fall into blocks that meet only
/// through `r` (ruling R105 part 2): each block's own LU, its solve of the
/// rail's column (`z = A_kk^-1 a_kr`) and the rail row's entries over it
/// (`d = a_rk`), and the Schur complement on the rail, `s = a_rr - sum d.z`.
#[derive(Debug, Clone, PartialEq)]
pub struct HubBlocks {
    r: usize,
    blocks: Vec<HubBlock>,
    s: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct HubBlock {
    idx: Vec<usize>,
    lu: LuFactorization,
    z: Vec<f64>,
    d: Vec<f64>,
}

/// Factors `a` (n x n, row-major, consumed) in place. Returns the
/// factorization, or `Err(())` if a pivot is exactly zero or non-finite
/// (singular or numerically unusable) — the caller turns that into
/// `ERR_SOLVE_FAILED` (-4).
pub fn lu_factor(mut a: Vec<f64>, n: usize) -> Result<LuFactorization, ()> {
    debug_assert_eq!(a.len(), n * n);
    let mut piv = vec![0usize; n];

    for k in 0..n {
        let mut best_row = k;
        let mut best = abs_or_worst(a[k * n + k]);
        for i in (k + 1)..n {
            let v = abs_or_worst(a[i * n + k]);
            if v > best {
                best = v;
                best_row = i;
            }
        }
        piv[k] = best_row;

        if best_row != k {
            // LINPACK layout: swap only columns k..n. Columns 0..k (already
            // holding earlier steps' multipliers) stay put — see module docs.
            for j in k..n {
                a.swap(k * n + j, best_row * n + j);
            }
        }

        let pivot = a[k * n + k];
        if !pivot.is_finite() || pivot == 0.0 {
            return Err(());
        }

        for i in (k + 1)..n {
            let m = a[i * n + k] / pivot;
            a[i * n + k] = m;
            if m != 0.0 {
                for j in (k + 1)..n {
                    a[i * n + j] -= m * a[k * n + j];
                }
            }
        }
    }

    Ok(LuFactorization { a, piv, n, hub: None })
}

/// Factors `a` (n x n, row-major) by block elimination over unknown `hub`
/// (ruling R105 part 2, exact mode's rail): the unknowns other than `hub`
/// are split into the connected components of `a`'s pattern with `hub`
/// taken out, each factored on its own (dense, partial pivoting within it),
/// and the rail is solved from the Schur complement, one scalar. `None`
/// when that does not pay (one block holding nearly everything, or a block
/// structure too fine to matter) or a block or the complement is singular:
/// the caller then factors `a` whole, as before.
pub fn lu_factor_hub(a: &[f64], n: usize, hub: usize) -> Option<LuFactorization> {
    debug_assert_eq!(a.len(), n * n);
    if hub >= n || n < HUB_MIN_N {
        return None;
    }
    #[cfg(test)]
    if hub_seam::OFF.with(|c| c.get()) {
        return None;
    }
    // Components of the pattern without the hub, by union-find.
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    for i in 0..n {
        if i == hub {
            continue;
        }
        let row = &a[i * n..(i + 1) * n];
        for (j, &v) in row.iter().enumerate() {
            if v != 0.0 && j != hub && j != i {
                let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                if ri != rj {
                    parent[ri.max(rj)] = ri.min(rj);
                }
            }
        }
    }
    let mut members: Vec<Vec<usize>> = Vec::new();
    let mut slot = vec![usize::MAX; n];
    for i in 0..n {
        if i == hub {
            continue;
        }
        let root = find(&mut parent, i);
        if slot[root] == usize::MAX {
            slot[root] = members.len();
            members.push(Vec::new());
        }
        members[slot[root]].push(i);
    }
    // Pays when the blocks' dense work is a small part of the whole's.
    let work: f64 = members.iter().map(|m| (m.len() as f64).powi(3)).sum();
    if work > HUB_WORK_FRACTION * (n as f64).powi(3) {
        return None;
    }
    let mut blocks = Vec::with_capacity(members.len());
    let mut s = a[hub * n + hub];
    for idx in members {
        let m = idx.len();
        let mut block = vec![0.0; m * m];
        for (bi, &i) in idx.iter().enumerate() {
            for (bj, &j) in idx.iter().enumerate() {
                block[bi * m + bj] = a[i * n + j];
            }
        }
        let lu = lu_factor(block, m).ok()?;
        let mut z: Vec<f64> = idx.iter().map(|&i| a[i * n + hub]).collect();
        lu.solve(&mut z).ok()?;
        let d: Vec<f64> = idx.iter().map(|&j| a[hub * n + j]).collect();
        s -= d.iter().zip(&z).map(|(d, z)| d * z).sum::<f64>();
        blocks.push(HubBlock { idx, lu, z, d });
    }
    if !s.is_finite() || s == 0.0 {
        return None;
    }
    #[cfg(test)]
    hub_seam::USES.with(|c| c.set(c.get() + 1));
    Some(LuFactorization { a: Vec::new(), piv: Vec::new(), n, hub: Some(Box::new(HubBlocks { r: hub, blocks, s })) })
}

/// Test seam: `OFF` makes [`lu_factor_hub`] decline (every system factored
/// whole, as before ruling R105 part 2), `USES` counts the systems it split.
/// Per thread, so parallel tests never see each other's.
#[cfg(test)]
pub(crate) mod hub_seam {
    thread_local! {
        pub static OFF: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
        pub static USES: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    }
}

/// The smallest system [`lu_factor_hub`] splits: below it the whole's LU
/// costs little.
pub const HUB_MIN_N: usize = 32;
/// [`lu_factor_hub`] splits only when its blocks' summed cubes are at most
/// this fraction of the whole's.
const HUB_WORK_FRACTION: f64 = 0.25;

impl LuFactorization {
    /// Solves `A x = b` for the `A` this factorization came from, in place
    /// on `b`. The row swaps recorded in `piv` are replayed one at a time,
    /// each immediately followed by that step's elimination against `b` —
    /// interleaved, not applied as a single upfront permutation. That
    /// ordering is what makes the "swap only columns k..n" factorization
    /// above correct (see module docs).
    pub fn solve(&self, b: &mut [f64]) -> Result<(), ()> {
        if let Some(h) = self.hub.as_deref() {
            return h.solve(b);
        }
        let n = self.n;
        debug_assert_eq!(b.len(), n);

        // Forward substitution (Ly = Pb), permutation interleaved.
        for k in 0..n {
            let p = self.piv[k];
            if p != k {
                b.swap(k, p);
            }
            let bk = b[k];
            if bk != 0.0 {
                for i in (k + 1)..n {
                    b[i] -= self.a[i * n + k] * bk;
                }
            }
        }

        // Back substitution (Ux = y).
        for i in (0..n).rev() {
            let mut s = b[i];
            for j in (i + 1)..n {
                s -= self.a[i * n + j] * b[j];
            }
            let d = self.a[i * n + i];
            if !d.is_finite() || d == 0.0 {
                return Err(());
            }
            b[i] = s / d;
        }

        Ok(())
    }
}

impl HubBlocks {
    fn solve(&self, b: &mut [f64]) -> Result<(), ()> {
        let mut ys: Vec<Vec<f64>> = Vec::with_capacity(self.blocks.len());
        let mut rhs = b[self.r];
        for blk in &self.blocks {
            let mut y: Vec<f64> = blk.idx.iter().map(|&i| b[i]).collect();
            blk.lu.solve(&mut y)?;
            rhs -= blk.d.iter().zip(&y).map(|(d, y)| d * y).sum::<f64>();
            ys.push(y);
        }
        let xr = rhs / self.s;
        if !xr.is_finite() {
            return Err(());
        }
        b[self.r] = xr;
        for (blk, y) in self.blocks.iter().zip(ys) {
            for ((&i, y), z) in blk.idx.iter().zip(y).zip(&blk.z) {
                b[i] = y - z * xr;
            }
        }
        Ok(())
    }
}

/// Factors and solves `a x = b` in one call, for callers with a single
/// right-hand side (every W0 partition solve).
pub fn solve(a: Vec<f64>, n: usize, mut b: Vec<f64>) -> Result<Vec<f64>, ()> {
    let lu = lu_factor(a, n)?;
    lu.solve(&mut b)?;
    Ok(b)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn matvec(a: &[f64], n: usize, x: &[f64]) -> Vec<f64> {
        (0..n)
            .map(|i| (0..n).map(|j| a[i * n + j] * x[j]).sum())
            .collect()
    }

    fn assert_close(a: &[f64], b: &[f64], tol: f64) {
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(b) {
            assert!((x - y).abs() <= tol, "{x} !~= {y}");
        }
    }

    #[test]
    fn solves_a_trivial_1x1_system() {
        let x = solve(vec![2.0], 1, vec![10.0]).unwrap();
        assert_close(&x, &[5.0], 1e-12);
    }

    #[test]
    fn solves_a_diagonal_system_with_no_pivoting_needed() {
        let a = vec![2.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 5.0];
        let x = solve(a, 3, vec![4.0, 8.0, 15.0]).unwrap();
        assert_close(&x, &[2.0, 2.0, 3.0], 1e-12);
    }

    #[test]
    fn solves_a_system_requiring_one_pivot_swap() {
        // Row 0's leading entry is smaller than row 1's, so partial pivoting
        // must swap once at step 0.
        let a = vec![1.0, 1.0, 2.0, 1.0];
        let x_expected = [1.0, 2.0];
        let b = matvec(&a, 2, &x_expected);
        let x = solve(a, 2, b).unwrap();
        assert_close(&x, &x_expected, 1e-9);
    }

    /// The regression case the contract calls out by name: a matrix that
    /// requires row swaps at more than one elimination step, checked against
    /// a known solution. This is deliberately a hand-built 4x4 whose natural
    /// pivots (the diagonal, in order) are each smaller than an entry below
    /// them, forcing a swap at step 0 AND at step 1.
    #[test]
    fn solves_a_system_requiring_swaps_at_two_elimination_steps() {
        let a = vec![
            1.0, 2.0, 3.0, 4.0, //
            2.0, 5.0, 2.0, 1.0, //
            4.0, 1.0, 9.0, 2.0, //
            3.0, 3.0, 3.0, 11.0,
        ];
        let x_expected = [2.0, -1.0, 0.5, 3.0];
        let b = matvec(&a, 4, &x_expected);
        let x = solve(a, 4, b).unwrap();
        assert_close(&x, &x_expected, 1e-8);
    }

    /// Same system as above, but factored by a deliberately wrong variant
    /// that swaps FULL rows (columns 0..n, including already-stored
    /// multipliers) while still using the interleaved solve above. The
    /// contract is explicit that this combination is wrong; this test proves
    /// this module's actual `lu_factor` does NOT behave this way, by showing
    /// the broken variant disagrees with the true solution on this same
    /// two-swap system (guarding against a future edit reintroducing the
    /// prototype's bug).
    #[test]
    fn full_row_swap_is_wrong() {
        fn broken_factor_and_solve(mut a: Vec<f64>, n: usize, b: &mut [f64]) -> Result<(), ()> {
            let mut piv = vec![0usize; n];
            for k in 0..n {
                let mut best_row = k;
                let mut best = abs_or_worst(a[k * n + k]);
                for i in (k + 1)..n {
                    let v = abs_or_worst(a[i * n + k]);
                    if v > best {
                        best = v;
                        best_row = i;
                    }
                }
                piv[k] = best_row;
                if best_row != k {
                    // BUG: full row swap, columns 0..n instead of k..n.
                    for j in 0..n {
                        a.swap(k * n + j, best_row * n + j);
                    }
                }
                let pivot = a[k * n + k];
                if !pivot.is_finite() || pivot == 0.0 {
                    return Err(());
                }
                for i in (k + 1)..n {
                    let m = a[i * n + k] / pivot;
                    a[i * n + k] = m;
                    if m != 0.0 {
                        for j in (k + 1)..n {
                            a[i * n + j] -= m * a[k * n + j];
                        }
                    }
                }
            }
            // Same interleaved apply as the real solve.
            for k in 0..n {
                let p = piv[k];
                if p != k {
                    b.swap(k, p);
                }
                let bk = b[k];
                if bk != 0.0 {
                    for i in (k + 1)..n {
                        b[i] -= a[i * n + k] * bk;
                    }
                }
            }
            for i in (0..n).rev() {
                let mut s = b[i];
                for j in (i + 1)..n {
                    s -= a[i * n + j] * b[j];
                }
                b[i] = s / a[i * n + i];
            }
            Ok(())
        }

        let a = vec![
            1.0, 2.0, 3.0, 4.0, //
            2.0, 5.0, 2.0, 1.0, //
            4.0, 1.0, 9.0, 2.0, //
            3.0, 3.0, 3.0, 11.0,
        ];
        let x_expected = [2.0_f64, -1.0, 0.5, 3.0];
        let mut b = matvec(&a, 4, &x_expected);
        broken_factor_and_solve(a, 4, &mut b).unwrap();
        // The broken variant must NOT match the true solution: if this
        // assertion ever starts failing, the "bug" is no longer reproduced
        // and this test has stopped proving anything.
        let matches = b
            .iter()
            .zip(x_expected.iter())
            .all(|(got, want)| (got - want).abs() <= 1e-8);
        assert!(!matches, "expected the full-row-swap variant to be wrong, but it matched");
    }

    #[test]
    fn zero_pivot_is_an_error() {
        // Column 0 has no nonzero entry at all, so no pivot choice can save it.
        let singular = vec![0.0, 0.0, 0.0, 0.0];
        assert!(solve(singular, 2, vec![1.0, 1.0]).is_err());
    }

    /// Ruling R105 part 2: blocks that meet only through one unknown, the
    /// rail, solved block by block over its Schur complement, agree with
    /// the whole's LU to rounding, through the rail's row being a source
    /// constraint (`u = V`) as well as a KCL row, and decline a system that
    /// is one block.
    #[test]
    fn a_hub_factorization_solves_blocks_meeting_at_one_unknown_as_the_whole_does() {
        let mut seed = 0x9e3779b97f4a7c15u64;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let (n, hub) = (60, 17);
        for constraint in [false, true] {
            // Blocks of 1-5 unknowns, each a conductance network with ties
            // to the hub, diagonally dominant as nodal analysis is.
            let mut a = vec![0.0; n * n];
            let others: Vec<usize> = (0..n).filter(|&i| i != hub).collect();
            let mut k = 0;
            while k < others.len() {
                let m = (1 + (rnd() * 5.0) as usize).min(others.len() - k);
                let blk = &others[k..k + m];
                for (x, &i) in blk.iter().enumerate() {
                    for &j in &blk[x + 1..] {
                        let g = rnd();
                        a[i * n + j] -= g;
                        a[j * n + i] -= g;
                        a[i * n + i] += g;
                        a[j * n + j] += g;
                    }
                    let g = rnd() + 0.1;
                    a[i * n + hub] -= g;
                    a[hub * n + i] -= g;
                    a[i * n + i] += g + rnd();
                    a[hub * n + hub] += g;
                }
                k += m;
            }
            a[hub * n + hub] += 1.0;
            if constraint {
                for j in 0..n {
                    a[hub * n + j] = if j == hub { 1.0 } else { 0.0 };
                }
            }
            let x_expected: Vec<f64> = (0..n).map(|_| rnd() * 10.0 - 5.0).collect();
            let b = matvec(&a, n, &x_expected);
            let whole = solve(a.clone(), n, b.clone()).unwrap();
            let f = lu_factor_hub(&a, n, hub).expect("the blocks pay");
            let mut x = b.clone();
            f.solve(&mut x).unwrap();
            assert_close(&x, &whole, 1e-11);
            assert_close(&x, &x_expected, 1e-9);
        }
        // One block holding everything: no split.
        let n = 40;
        let mut a = vec![0.0; n * n];
        for i in 0..n {
            a[i * n + i] = 3.0;
            if i + 1 < n {
                a[i * n + i + 1] = -1.0;
                a[(i + 1) * n + i] = -1.0;
            }
        }
        assert!(lu_factor_hub(&a, n, 0).is_none());
    }

    #[test]
    fn nan_pivot_is_an_error() {
        let a = vec![f64::NAN, 0.0, 0.0, 1.0];
        assert!(solve(a, 2, vec![1.0, 1.0]).is_err());
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for LuFactorization {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.piv, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.n, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for HubBlocks {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.r, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.blocks, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for HubBlock {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.idx, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.lu, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.z, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.d, w); }
        n
    }
}
