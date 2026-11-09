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

    Ok(LuFactorization { a, piv, n })
}

impl LuFactorization {
    /// Solves `A x = b` for the `A` this factorization came from, in place
    /// on `b`. The row swaps recorded in `piv` are replayed one at a time,
    /// each immediately followed by that step's elimination against `b` —
    /// interleaved, not applied as a single upfront permutation. That
    /// ordering is what makes the "swap only columns k..n" factorization
    /// above correct (see module docs).
    pub fn solve(&self, b: &mut [f64]) -> Result<(), ()> {
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

    #[test]
    fn nan_pivot_is_an_error() {
        let a = vec![f64::NAN, 0.0, 0.0, 1.0];
        assert!(solve(a, 2, vec![1.0, 1.0]).is_err());
    }
}

// ---- W2a: heap bytes (generated by the size walk generator; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for LuFactorization {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.piv, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.n, w); }
        n
    }
}
