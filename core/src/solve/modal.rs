//! The exact step for linear partitions (design-exact-step.md, D2), slice 1:
//! the numerical kernels. Nothing in the engine calls this module yet.
//!
//! The system is `x' = A x + B u(t)` with `u(t) = u0 + du t` over a step.
//! [`decompose`] factors `A = S D S^-1` once per system, `D` block diagonal
//! with blocks of order 1 (a real rate), 2 (a complex pair in standardised
//! real form) or more (a merged cluster of close eigenvalues); then
//! [`Modal::at`] gives `x(tau)` for any `tau >= 0` at O(n^2 + n m) plus one
//! exponential per block, closed form for order 1 and 2 and Pade 13 for a
//! merged block. The kernels, in the order `decompose` runs them:
//!
//! - [`balance`]: power-of-two diagonal scaling (EISPACK `balanc`, no
//!   permutations), so a circuit's 1e-12 F and 1e-3 H entries do not decide
//!   which eigenvalues keep their digits.
//! - K1 [`hessenberg`]: Householder reduction.
//! - K2 [`schur`]: Francis double-shift QR to real Schur form, with the
//!   Ahues-Tisseur deflation test of LAPACK's `dlahqr`, exceptional shifts
//!   and an iteration cap; 2x2 blocks standardised (`dlanv2`).
//! - K3 [`sylvester`]: `A X - X B = C` for quasi-triangular `A`, `B`.
//! - K4 [`group`]: Bavely-Stewart block diagonalisation: a split whose
//!   Sylvester solution is larger than [`SPLIT_NORM_MAX`] is refused, the
//!   nearest eigenvalue is moved up by orthogonal block swaps and the
//!   cluster grows.
//! - K5 [`real_exp_phi`], [`pair_exp_phi`]: exp, phi_1, phi_2 of an order-1
//!   or order-2 block in closed form.
//! - K6 [`pade_exp`]: Pade 13 with scaling and squaring, the merged blocks'
//!   exponential (augmented with the source so one exponential gives the
//!   whole response), in caller-owned buffers.
//! - K7 [`Modal`]: the assembled object.
//!
//! Certification (R151): [`decompose`] declines a system it cannot certify;
//! the certification is paid once per system and [`Modal::step`] and
//! [`Modal::at`] do none of it (see [`certify`]).
//!
//! Failure: every routine returns [`ModalError`] rather than panicking on
//! any finite input, and every iteration has a stated cap whose hitting is
//! an error. The caller turns an error into "this partition stays on the
//! trapezoid". Accuracy figures quoted below are the measurements of
//! `modal_tests.rs` (seeded; the double-double Pade oracle is the reference).

/// The largest system the exact step takes (design D1/D2: past 32 states a
/// partition stays on the trapezoid).
pub const MAX_N: usize = 32;

/// Francis QR's cap on iterations spent on one deflation, after LAPACK
/// `dlahqr`'s `30 * max(10, nh)` (per active window), read per eigenvalue
/// here: past it [`schur`] returns [`ModalError::QrNoConvergence`]. The
/// random set's worst eigenvalue took 9 iterations (`modal_tests`,
/// `qr_iterations_measured`); the cap is far from ordinary use.
pub const QR_ITER_CAP: usize = 300;

/// Bavely-Stewart's split threshold: a cluster is split off the rest only
/// when the Sylvester solution `X` that decouples it has Frobenius norm at
/// most this. The design proposed 1e4; the threshold sweep in `modal_tests`
/// (`split_threshold_sweep`) measured both ends on the random set and the
/// stiff boards, see the figure there and in the module report.
pub const SPLIT_NORM_MAX: f64 = 1e2;

/// The lowest split threshold the regrouping reaches (see [`REGROUP_MAX`]):
/// at `|X|_F <= 1` a split at most quadruples cond(S).
pub const SPLIT_NORM_MIN: f64 = 1.0;

/// Regroupings after the first grouping, each at a tenth of the previous
/// threshold, while the factorisation's [`Modal::rounding_floor`] exceeds
/// [`REGROUP_FLOOR`] or it fails its budget (residual, cond(S), floor):
/// [`SPLIT_NORM_MAX`] 1e2, then 1e1, then [`SPLIT_NORM_MIN`]; the attempt
/// with the lowest floor within budget is kept. Bounded by construction.
pub const REGROUP_MAX: usize = 2;

/// The largest rounding floor [`decompose`] certifies: past it no step of
/// the system is trusted to keep 1e-12 relative even where the answer is
/// insensitive to its data (a step of length 0 returns `x0` through `S` and
/// `S^-1`), so the decomposition declines. See [`Modal::rounding_floor`].
pub const ROUND_MAX: f64 = 1e-12;

/// `decompose` refuses a factorisation whose measured `cond(S)` (1-norm, in
/// balanced coordinates) is above this after regrouping (EX1's bound; the
/// rounding floor, [`ROUND_MAX`], is the finer test and binds first).
pub const COND_MAX: f64 = 1e6;

/// `decompose` refuses a factorisation whose backward error
/// `|S D S^-1 - A|_1 / |A|_1` (balanced coordinates) exceeds this.
pub const RESIDUAL_MAX: f64 = 1e-13;

/// The relative size of the certification's shadow perturbations
/// ([`certify`]; 2^-26, the square root of
/// eps: far above the step's rounding, small enough that the response is
/// linear for any condition below about 1e7).
pub const SCE_DELTA: f64 = 1.4901161193847656e-8;

/// Shadow decompositions per system (small-sample statistical condition
/// estimation, Kenney and Laub 1994: a few samples bound the condition
/// within a small factor with high probability).
pub const SCE_SAMPLES: usize = 3;

/// The certification's probes stop at this many Pade squarings of a merged
/// block (the random set's steps took at most 29); a longer step's
/// squarings and its sensitivity both grow with `h`, so the ratio the
/// probes certify carries on.
pub const CERT_SQ_MAX: i32 = 30;

/// A merged block's Pade error a probe may show whatever its sensitivity: a
/// tenth of the 1e-12 old bound (R159's K7 floor), the rest left to the
/// rounding through `S` (at most [`ROUND_MAX`]) and the input's own
/// sensitivity.
pub const CERT_TOL: f64 = 1e-13;

/// The Pade error and the rounding floor together may take this when the
/// floor leaves room for more than [`CERT_TOL`]: R159's K7 floor (1e-12,
/// R151's whole allowance below the well-conditioned line, sensitivity
/// 45 eps) less the step's input rounding there, which the random set's
/// uncertified steps hold to 0.2 of the allowance (`measure_r151`: 20 eps
/// times 45, 2e-13). EX1E re-derived it from that budget when R161's
/// directions found true Pade errors of 6.6e-13 (n = 32 Jordan member 377,
/// a forced direction of a slowly decaying merged block at 2^18 times its
/// own scale, a step within R151 at 0.65 of its allowance) over EX1C's
/// share of half the floor. History: EX1B accepted `2 |diff| <= max(2e-13,
/// 20 eps kappa)`; EX1C `|diff| <= max(1e-13, 5e-13 - floor, 20 eps
/// kappa)` with an estimate EX1D measured up to 1.6e6 times under the true
/// error.
pub const CERT_SHARE: f64 = 8e-13;

/// The Pade error a probe may show, in units of `eps kappa`, `kappa` the
/// sensitivity the shadows measure along that probe's own direction (R161:
/// each direction its own). R151 allows 100 eps times the answer's
/// sensitivity. Every shadow's response is a sample of that sensitivity
/// (to second order, `kappa SCE_DELTA`, under 1e-1 wherever a digit is
/// left), so `kappa` is at most the true one; the step's input rounding
/// takes at most 20 (the random set's uncertified steps, 0.2 of the
/// allowance); 50 for the Pade leaves 30 of R151's 100 for the test's
/// own sampling of the sensitivity (`kappa_fd`, four samples): the
/// certificate's limit stays inside the test's allowance while the
/// shadows read at most 1.6 times `kappa_fd`. Measured (EX1E,
/// `certify_estimate_bounds_the_pade_error`): at every judged probe whose
/// estimate took over a quarter of its limit, the probe's own step against
/// the oracle, worst 0.34 of R151's allowance; the shadows' kappa over
/// `kappa_fd` from 0.002 to 2.8 there. EX1D's 40 was set from one
/// direction's kappa on one case (5126, skewed, a measured 1.70e-12); R161's
/// forced singular direction of the n = 32 ortho member 348 needs 45 (a
/// true Pade error of 9.6e-13 at kappa 96, its step at 0.34 of R151).
pub const CERT_KAPPA: f64 = 50.0;

/// The factor on the evaluations' differences in [`Modal::pade_estimate`]:
/// the differences measure the other evaluations' errors as much as the
/// step's, and an error the step shares with one of them only partly, so
/// the sum is scaled. Measured (`certify_estimate_bounds_the_pade_error`,
/// 10,000 cases and the n = 16 and 32 members, four directions, every
/// judged probe): true error / (`max(d1, d9) + d_id` + floor) at worst 2.28
/// (a Jordan member), so 3 leaves it at 0.77; the estimate is never the
/// last word on a decline ([`certify`] measures first).
pub const CERT_EST_FACTOR: f64 = 3.0;

/// The certification does not probe a merged block over a step that grows
/// it by more than `e^CERT_GROWTH_STOP` (its largest real part times the
/// step): the mirror of the decayed stop (`e^-39`, below 1e-17). Every
/// evaluation inside the two stops must succeed or the system declines.
pub const CERT_GROWTH_STOP: f64 = 39.0;

/// R161: per merged block the certification probes the probe direction
/// ([`probe_vectors`]) and this many more, along the block's leading
/// singular directions ([`Modal::block_directions`]): 3 to 4 directions
/// per block (fewer singular ones for a block of 2 states, whose projector
/// has rank 2).
pub const CERT_SINGULAR_DIRS: usize = 3;

/// The finest error [`Modal::pade_estimate`] claims to resolve, relative to
/// the step's reference: eps / 16 (1.4e-17). A step's result is itself
/// rounded to eps of the reference, so nothing under this is an error any
/// reader of the step can see. Below it the estimate's differences can all
/// be exactly zero while the true Pade error is not (a decayed block whose
/// every evaluation squares to the same bits; EX1E measured such errors up
/// to 2.2e-19 on the random set's Jordan member 49), so the estimate adds it
/// and the tests can hold the estimate itself to the true error.
pub const EST_RESOLUTION: f64 = f64::EPSILON / 4.0;

/// A merged block whose largest real part times the step is at most
/// `-SOLVE_DECAY` has its input response computed by two quasi-triangular
/// solves ([`Modal::at`]: `M^-1 [E c0 - h B~ u(h) + M^-1 (E - I) c1]`, `M =
/// D h`, the exact identity of the augmented exponential's last columns)
/// instead of the augmented Pade. Why (EX1F, random case 2360): once the
/// block has decayed, the constant input's and the ramp's responses tend to
/// `-M^-1 c0` and `-M^-1 (c0 + c1) - M^-2 c1` (`c0 = h g0`, `c1 = h^2 g1`),
/// which cancel to `M^-2 c1` as `u(h)` goes to 0 by a factor up to `|lambda
/// h|` (6e5 there); the augmented Pade rounds each response to its own size
/// over its squarings (2.3e-12 of the result at 18 squarings, 2.3 times
/// R151's allowance), and that cancellation is invisible to the data's
/// sensitivity (a perturbed `A` or `B` moves both responses alike), so no
/// allowance excuses it. The solves form the end input `u(h) = u0 + du h`
/// with one rounding (`fma`) and never form the two responses apart. Below
/// the threshold `e^(M)` is not small, the cancellation ratio is at most
/// `~e^SOLVE_DECAY` beyond what the phase sensitivity of `e^(M)` already
/// allows, and `E - I` may cancel: the augmented Pade stays. 1 keeps that
/// unexcused factor at e (the augmented Pade's own error at 18 squarings is
/// 1e-13 of the response, so e times it is within R151's floor), and every
/// eigenvalue of a block past it has `|mu| >= 1`, so `M^-1` amplifies at
/// most by the block's non-normality (which the certification's rounding
/// term carries, [`Modal::eval_rounding`]). The closed forms (a real rate,
/// a pair) take the same identity past the same threshold: their phi
/// functions are each right to eps, but formed apart a constant and a ramp
/// that cancel lose the same factor (EX1F measured 1.7e-11, 6.8 times
/// R151, along case 2360's input-response direction, from its pairs), and
/// no certification probes a block that is not merged.
pub const SOLVE_DECAY: f64 = 1.0;

/// Sweep cap of [`sym_eig`]: cyclic Jacobi converges quadratically, and
/// symmetric matrices of order 32 take under a dozen sweeps.
pub const JACOBI_SWEEPS: usize = 50;

/// [`decompose_core`] stops regrouping once the rounding floor is at most
/// this, a tenth of [`ROUND_MAX`]: the two well-conditioned merged-cluster
/// systems EX1 declined had S conditioned 1.8e3 and 6.6e3 by decoupling
/// close Jordan clusters (floors 3.1e-13, 8.1e-13); regrouped, the clusters
/// merge and the floor falls an order under the gate.
pub const REGROUP_FLOOR: f64 = 1e-13;

/// Below this `|z|` the phi functions are summed as power series (no
/// cancellation in `e^z - 1 - z`); at and above it the closed forms are used.
pub const SERIES_SWITCH: f64 = 1.0;

/// Terms of the series below [`SERIES_SWITCH`]: the first omitted term is at
/// most `1 / 24!` = 1.6e-24 of the sum's leading term.
const SERIES_TERMS: usize = 22;

/// Pade 13's scaling threshold `theta_13` (Higham 2005).
const THETA13: f64 = 5.371920351148152;

/// Pade 13's squarings are bounded by the exponent range: `norm <= f64::MAX`
/// gives `s <= 1024`. A larger computed `s` (a non-finite norm) is an error.
pub const PADE_MAX_SQUARINGS: i32 = 1100;

/// Balancing sweeps (EISPACK `balanc` stops when a sweep changes nothing;
/// ordinary matrices take two to four). The sweep cap only bounds the work:
/// any diagonal scaling is a valid similarity, so stopping there is not an
/// error, the one cap in this module that is not.
const BALANCE_SWEEPS: usize = 32;

/// Why a decomposition or a step could not be produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModalError {
    /// `n` is 0 or above [`MAX_N`], or a slice has the wrong length.
    Shape,
    /// An input or a result is not finite (a growing mode that overflows).
    NonFinite,
    /// Francis QR hit [`QR_ITER_CAP`] on one eigenvalue.
    QrNoConvergence,
    /// A Sylvester system was singular to working precision (a common
    /// eigenvalue across the split): the split is refused.
    SylvesterSingular,
    /// An orthogonal block swap failed its stability test.
    SwapRejected,
    /// Pade's denominator was singular, or the squarings would exceed
    /// [`PADE_MAX_SQUARINGS`].
    PadeFailed,
    /// The system cannot be certified: no grouping keeps the factorisation
    /// within its budget ([`RESIDUAL_MAX`], [`COND_MAX`], [`ROUND_MAX`]), a
    /// shadow decomposition failed, or a certification probe was over its
    /// limit ([`certify`]).
    IllConditioned,
    /// `terms` asked for a block the exponential-sum form cannot express (a
    /// merged block, or a zero rate under a ramp).
    Unsupported,
}

#[inline(always)]
fn ix(n: usize, i: usize, j: usize) -> usize {
    i * n + j
}

fn all_finite(v: &[f64]) -> bool {
    v.iter().all(|x| x.is_finite())
}

fn inf(v: &[f64]) -> f64 {
    v.iter().fold(0.0f64, |m, x| m.max(x.abs()))
}

fn norm1(a: &[f64], n: usize) -> f64 {
    let mut best = 0.0f64;
    for j in 0..n {
        let mut s = 0.0;
        for i in 0..n {
            s += a[ix(n, i, j)].abs();
        }
        best = best.max(s);
    }
    best
}

/// Fortran SIGN(a, b): |a| with the sign of b (b = 0 counts as positive).
#[inline(always)]
fn sign(a: f64, b: f64) -> f64 {
    if b >= 0.0 {
        a.abs()
    } else {
        -a.abs()
    }
}

// ---- balancing --------------------------------------------------------------

/// Balances `a` (n x n, row-major) in place: `a <- D^-1 a D` with `D =
/// diag(d)`, each `d_i` a power of two, so that row and column off-diagonal
/// norms are comparable (EISPACK `balanc`, radix 2, no permutations). Exact:
/// powers of two scale without rounding, so the similarity loses nothing.
/// Never fails; at most [`BALANCE_SWEEPS`] sweeps, scale factors kept within
/// 2^-500 .. 2^500.
pub fn balance(a: &mut [f64], n: usize, d: &mut [f64]) {
    for v in d.iter_mut().take(n) {
        *v = 1.0;
    }
    const FMAX: f64 = 3.273390607896142e150; // 2^500
    const FMIN: f64 = 3.054936363499605e-151; // 2^-500
    for _sweep in 0..BALANCE_SWEEPS {
        let mut changed = false;
        for i in 0..n {
            let (mut c, mut r) = (0.0f64, 0.0f64);
            for j in 0..n {
                if j != i {
                    c += a[ix(n, j, i)].abs();
                    r += a[ix(n, i, j)].abs();
                }
            }
            if c == 0.0 || r == 0.0 || !c.is_finite() || !r.is_finite() {
                continue;
            }
            let s = c + r;
            let mut f = 1.0f64;
            let mut cc = c;
            let g = r / 2.0;
            // Bounded by the exponent range: each pass doubles f.
            for _ in 0..1100 {
                if cc >= g || f * d[i] >= FMAX {
                    break;
                }
                f *= 2.0;
                cc *= 4.0;
            }
            let g = r * 2.0;
            for _ in 0..1100 {
                if cc < g || f * d[i] <= FMIN {
                    break;
                }
                f /= 2.0;
                cc /= 4.0;
            }
            if (cc + r) / f < 0.95 * s {
                d[i] *= f;
                changed = true;
                for j in 0..n {
                    a[ix(n, i, j)] /= f;
                }
                for j in 0..n {
                    a[ix(n, j, i)] *= f;
                }
            }
        }
        if !changed {
            break;
        }
    }
}

// ---- K1: Hessenberg --------------------------------------------------------

/// K1. Reduces `a` (n x n, row-major) in place to upper Hessenberg form `H`
/// by Householder reflections and returns the orthogonal `q` (n x n) with
/// `A = Q H Q^T`. Entries below the subdiagonal are set to exactly 0.
/// Backward stable: measured `|Q H Q^T - A|_1 / |A|_1` at most 2.0e-15 and
/// `|Q^T Q - I|_1` at most 2.2e-15 on the random set (`modal_tests`,
/// `hessenberg_and_schur_backward`). Fails only on a non-finite input.
pub fn hessenberg(a: &mut [f64], q: &mut [f64], n: usize) -> Result<(), ModalError> {
    if a.len() != n * n || q.len() != n * n {
        return Err(ModalError::Shape);
    }
    if !all_finite(a) {
        return Err(ModalError::NonFinite);
    }
    for i in 0..n {
        for j in 0..n {
            q[ix(n, i, j)] = if i == j { 1.0 } else { 0.0 };
        }
    }
    let mut v = [0.0f64; MAX_N];
    if n > MAX_N {
        return Err(ModalError::Shape);
    }
    for k in 0..n.saturating_sub(2) {
        let mut scale = 0.0f64;
        for i in (k + 1)..n {
            scale += a[ix(n, i, k)].abs();
        }
        if scale == 0.0 {
            continue;
        }
        let mut ss = 0.0f64;
        for i in (k + 1)..n {
            v[i] = a[ix(n, i, k)] / scale;
            ss += v[i] * v[i];
        }
        let sigma = sign(ss.sqrt(), v[k + 1]);
        v[k + 1] += sigma;
        let beta = 1.0 / (sigma * v[k + 1]);
        // From the left: rows k+1.., columns k..
        for j in k..n {
            let mut s = 0.0;
            for i in (k + 1)..n {
                s += v[i] * a[ix(n, i, j)];
            }
            s *= beta;
            for i in (k + 1)..n {
                a[ix(n, i, j)] -= s * v[i];
            }
        }
        // From the right: all rows, columns k+1..
        for i in 0..n {
            let mut s = 0.0;
            for j in (k + 1)..n {
                s += a[ix(n, i, j)] * v[j];
            }
            s *= beta;
            for j in (k + 1)..n {
                a[ix(n, i, j)] -= s * v[j];
            }
        }
        for i in 0..n {
            let mut s = 0.0;
            for j in (k + 1)..n {
                s += q[ix(n, i, j)] * v[j];
            }
            s *= beta;
            for j in (k + 1)..n {
                q[ix(n, i, j)] -= s * v[j];
            }
        }
        a[ix(n, k + 1, k)] = -sigma * scale;
        for i in (k + 2)..n {
            a[ix(n, i, k)] = 0.0;
        }
    }
    Ok(())
}

// ---- K2: Francis QR ----------------------------------------------------------

/// LAPACK `dlanv2`: the Schur factorisation of a real 2x2 block,
/// `[a b; c d] = G [aa bb; cc dd] G^T`, `G = [cs -sn; sn cs]`, with either
/// `cc = 0` (real eigenvalues) or `aa = dd` and `bb cc < 0` (a complex pair).
/// Returns (aa, bb, cc, dd, cs, sn).
fn lanv2(mut a: f64, mut b: f64, mut c: f64, mut d: f64) -> (f64, f64, f64, f64, f64, f64) {
    let eps = f64::EPSILON;
    let (mut cs, mut sn);
    if c == 0.0 {
        cs = 1.0;
        sn = 0.0;
    } else if b == 0.0 {
        cs = 0.0;
        sn = 1.0;
        std::mem::swap(&mut a, &mut d);
        b = -c;
        c = 0.0;
    } else if (a - d) == 0.0 && sign(1.0, b) != sign(1.0, c) {
        cs = 1.0;
        sn = 0.0;
    } else {
        let temp = a - d;
        let mut p = 0.5 * temp;
        let bcmax = b.abs().max(c.abs());
        let bcmis = b.abs().min(c.abs()) * sign(1.0, b) * sign(1.0, c);
        let scale = p.abs().max(bcmax);
        let mut z = (p / scale) * p + (bcmax / scale) * bcmis;
        if z >= 4.0 * eps {
            // Real eigenvalues.
            z = p + sign(scale.sqrt() * z.sqrt(), p);
            a = d + z;
            d -= (bcmax / z) * bcmis;
            let tau = c.hypot(z);
            cs = z / tau;
            sn = c / tau;
            b -= c;
            c = 0.0;
        } else {
            // Complex, or real (almost) equal: make the diagonal equal.
            let sigma = b + c;
            let tau = sigma.hypot(temp);
            cs = (0.5 * (1.0 + sigma.abs() / tau)).sqrt();
            sn = -(p / (tau * cs)) * sign(1.0, sigma);
            let aa = a * cs + b * sn;
            let bb = -a * sn + b * cs;
            let cc = c * cs + d * sn;
            let dd = -c * sn + d * cs;
            a = aa * cs + cc * sn;
            b = bb * cs + dd * sn;
            c = -aa * sn + cc * cs;
            d = -bb * sn + dd * cs;
            let temp = 0.5 * (a + d);
            a = temp;
            d = temp;
            if c != 0.0 {
                if b != 0.0 {
                    if sign(1.0, b) == sign(1.0, c) {
                        // Real eigenvalues: reduce to upper triangular.
                        let sab = b.abs().sqrt();
                        let sac = c.abs().sqrt();
                        p = sign(sab * sac, c);
                        let tau = 1.0 / (b + c).abs().sqrt();
                        a = temp + p;
                        d = temp - p;
                        b -= c;
                        c = 0.0;
                        let cs1 = sab * tau;
                        let sn1 = sac * tau;
                        let t = cs * cs1 - sn * sn1;
                        sn = cs * sn1 + sn * cs1;
                        cs = t;
                    }
                } else {
                    b = -c;
                    c = 0.0;
                    let t = cs;
                    cs = -sn;
                    sn = t;
                }
            }
        }
    }
    (a, b, c, d, cs, sn)
}

/// Applies the plane rotation `G = [cs -sn; sn cs]` on indices (k, k+1) as a
/// similarity of the quasi-triangular `t` outside its 2x2 diagonal block
/// (rows k, k+1 right of it; columns k, k+1 above it), and to `s` (columns)
/// and `sinv` (rows, `G^T` from the left) when given.
fn rot_outside(t: &mut [f64], n: usize, k: usize, cs: f64, sn: f64, s: &mut [f64], sinv: Option<&mut [f64]>) {
    for j in (k + 2)..n {
        let x = t[ix(n, k, j)];
        let y = t[ix(n, k + 1, j)];
        t[ix(n, k, j)] = cs * x + sn * y;
        t[ix(n, k + 1, j)] = cs * y - sn * x;
    }
    for i in 0..k {
        let x = t[ix(n, i, k)];
        let y = t[ix(n, i, k + 1)];
        t[ix(n, i, k)] = cs * x + sn * y;
        t[ix(n, i, k + 1)] = cs * y - sn * x;
    }
    for i in 0..n {
        let x = s[ix(n, i, k)];
        let y = s[ix(n, i, k + 1)];
        s[ix(n, i, k)] = cs * x + sn * y;
        s[ix(n, i, k + 1)] = cs * y - sn * x;
    }
    if let Some(si) = sinv {
        for j in 0..n {
            let x = si[ix(n, k, j)];
            let y = si[ix(n, k + 1, j)];
            si[ix(n, k, j)] = cs * x + sn * y;
            si[ix(n, k + 1, j)] = cs * y - sn * x;
        }
    }
}

/// Standardises the 2x2 diagonal block at (k, k+1) with [`lanv2`].
fn standardize(t: &mut [f64], n: usize, k: usize, s: &mut [f64], sinv: Option<&mut [f64]>) {
    let (a, b, c, d, cs, sn) = lanv2(t[ix(n, k, k)], t[ix(n, k, k + 1)], t[ix(n, k + 1, k)], t[ix(n, k + 1, k + 1)]);
    t[ix(n, k, k)] = a;
    t[ix(n, k, k + 1)] = b;
    t[ix(n, k + 1, k)] = c;
    t[ix(n, k + 1, k + 1)] = d;
    rot_outside(t, n, k, cs, sn, s, sinv);
}

/// LAPACK `dlahqr`'s deflation test for the subdiagonal entry `h[k][k-1]`
/// (Ahues and Tisseur): a small subdiagonal is set to zero only when doing so
/// moves the smaller nearby eigenvalue by at most a rounding of itself, so a
/// 1e12 spread keeps the slow eigenvalue's relative digits.
fn negligible(h: &[f64], n: usize, k: usize, lo: usize, hi: usize, ulp: f64, smlnum: f64) -> bool {
    let sub = h[ix(n, k, k - 1)].abs();
    if sub <= smlnum {
        return true;
    }
    let mut tst = h[ix(n, k - 1, k - 1)].abs() + h[ix(n, k, k)].abs();
    if tst == 0.0 {
        if k >= lo + 2 {
            tst += h[ix(n, k - 1, k - 2)].abs();
        }
        if k < hi {
            tst += h[ix(n, k + 1, k)].abs();
        }
    }
    if sub <= ulp * tst {
        let ab = sub.max(h[ix(n, k - 1, k)].abs());
        let ba = sub.min(h[ix(n, k - 1, k)].abs());
        let dd = (h[ix(n, k - 1, k - 1)] - h[ix(n, k, k)]).abs();
        let aa = h[ix(n, k, k)].abs().max(dd);
        let bb = h[ix(n, k, k)].abs().min(dd);
        let s = aa + ab;
        return ba * (ab / s) <= smlnum.max(ulp * (bb * (aa / s)));
    }
    false
}

/// K2. Reduces the upper Hessenberg `h` (n x n) in place to real Schur form
/// `T` (quasi upper triangular: 1x1 blocks and standardised 2x2 blocks
/// `[a b; c a]`, `b c < 0`, each a complex pair; exact zeros elsewhere below
/// the diagonal) by Francis double-shift QR, accumulating the transformations
/// into `q` (on entry the Hessenberg reduction's Q, on exit Q with
/// `A = Q T Q^T`). Shifts: the trailing 2x2's eigenvalues (implicit), with
/// exceptional shifts at the 10th, 20th, ... iteration on one eigenvalue
/// (`dlahqr`'s two ad hoc shifts in turn). Fails with
/// [`ModalError::QrNoConvergence`] after [`QR_ITER_CAP`] iterations on one
/// eigenvalue, [`ModalError::NonFinite`] if the result is not finite.
/// Backward error measured at most 6.3e-15 relative on the random set,
/// orthogonality 6.0e-15 (`hessenberg_and_schur_backward`).
pub fn schur(h: &mut [f64], q: &mut [f64], n: usize) -> Result<(), ModalError> {
    if h.len() != n * n || q.len() != n * n || n > MAX_N {
        return Err(ModalError::Shape);
    }
    if n == 0 {
        return Ok(());
    }
    if !all_finite(h) {
        return Err(ModalError::NonFinite);
    }
    for i in 0..n {
        for j in 0..i.saturating_sub(1) {
            h[ix(n, i, j)] = 0.0;
        }
    }
    let ulp = f64::EPSILON;
    let eps = f64::EPSILON;
    let smlnum = f64::MIN_POSITIVE * (n as f64 / ulp);
    let mut i = n - 1;
    let mut its = 0usize;
    // Each pass deflates one or two eigenvalues or spends one iteration, and
    // the iterations per eigenvalue are capped: the pass count is bounded.
    let max_passes = n * (QR_ITER_CAP + 2) + 2;
    for _pass in 0..max_passes {
        let mut l = 0usize;
        for k in (1..=i).rev() {
            if negligible(h, n, k, 0, i, ulp, smlnum) {
                l = k;
                break;
            }
        }
        if l > 0 {
            h[ix(n, l, l - 1)] = 0.0;
        }
        if l == i {
            if i == 0 {
                return finish_schur(h, q);
            }
            i -= 1;
            its = 0;
            continue;
        }
        if l + 1 == i {
            standardize(h, n, l, q, None);
            if l == 0 {
                return finish_schur(h, q);
            }
            i = l - 1;
            its = 0;
            continue;
        }
        if its >= QR_ITER_CAP {
            return Err(ModalError::QrNoConvergence);
        }
        #[cfg(test)]
        QR_MAX_ITS.with(|c| c.set(c.get().max(its + 1)));
        // Shifts: the trailing 2x2 (x, y on the diagonal, w its off-diagonal
        // product), or an exceptional shift.
        let mut x = h[ix(n, i, i)];
        let mut y = h[ix(n, i - 1, i - 1)];
        let mut w = h[ix(n, i, i - 1)] * h[ix(n, i - 1, i)];
        if its > 0 && its % 20 == 10 {
            let s = h[ix(n, i, i - 1)].abs() + h[ix(n, i - 1, i - 2)].abs();
            x = h[ix(n, i, i)] + 0.75 * s;
            y = x;
            w = -0.4375 * s * s;
        } else if its > 0 && its % 20 == 0 {
            let s = h[ix(n, l + 1, l)].abs() + h[ix(n, l + 2, l + 1)].abs();
            x = h[ix(n, l, l)] + 0.75 * s;
            y = x;
            w = -0.4375 * s * s;
        }
        its += 1;
        // Two consecutive small subdiagonal elements.
        let (mut p, mut qq, mut r) = (0.0, 0.0, 0.0);
        let mut m = i - 2;
        for mm in (l..=i - 2).rev() {
            m = mm;
            let z = h[ix(n, mm, mm)];
            let rr = x - z;
            let ss = y - z;
            p = (rr * ss - w) / h[ix(n, mm + 1, mm)] + h[ix(n, mm, mm + 1)];
            qq = h[ix(n, mm + 1, mm + 1)] - z - rr - ss;
            r = h[ix(n, mm + 2, mm + 1)];
            let mut s = p.abs() + qq.abs() + r.abs();
            if s == 0.0 || !s.is_finite() {
                s = 1.0;
            }
            p /= s;
            qq /= s;
            r /= s;
            if mm == l {
                break;
            }
            if h[ix(n, mm, mm - 1)].abs() * (qq.abs() + r.abs())
                < eps * (p.abs() * (h[ix(n, mm - 1, mm - 1)].abs() + z.abs() + h[ix(n, mm + 1, mm + 1)].abs()))
            {
                break;
            }
        }
        for k in (m + 2)..=i {
            h[ix(n, k, k - 2)] = 0.0;
            if k > m + 2 {
                h[ix(n, k, k - 3)] = 0.0;
            }
        }
        // The double QR step on rows l..=i, columns m.., chasing the bulge.
        for k in m..i {
            let notlast = k != i - 1;
            let mut xx = 1.0;
            if k != m {
                p = h[ix(n, k, k - 1)];
                qq = h[ix(n, k + 1, k - 1)];
                r = if notlast { h[ix(n, k + 2, k - 1)] } else { 0.0 };
                xx = p.abs() + qq.abs() + r.abs();
                if xx == 0.0 {
                    continue;
                }
                p /= xx;
                qq /= xx;
                r /= xx;
            }
            let s = sign((p * p + qq * qq + r * r).sqrt(), p);
            if s != 0.0 {
                if k != m {
                    h[ix(n, k, k - 1)] = -s * xx;
                    // The reflector zeroes the rest of the bulge's column;
                    // the row update below starts at column k, so set them.
                    h[ix(n, k + 1, k - 1)] = 0.0;
                    if notlast {
                        h[ix(n, k + 2, k - 1)] = 0.0;
                    }
                } else if l != m {
                    h[ix(n, k, k - 1)] = -h[ix(n, k, k - 1)];
                }
                p += s;
                let x1 = p / s;
                let y1 = qq / s;
                let z1 = r / s;
                let q1 = qq / p;
                let r1 = r / p;
                for j in k..n {
                    let mut pp = h[ix(n, k, j)] + q1 * h[ix(n, k + 1, j)];
                    if notlast {
                        pp += r1 * h[ix(n, k + 2, j)];
                        h[ix(n, k + 2, j)] -= pp * z1;
                    }
                    h[ix(n, k, j)] -= pp * x1;
                    h[ix(n, k + 1, j)] -= pp * y1;
                }
                let top = i.min(k + 3);
                for ii in 0..=top {
                    let mut pp = x1 * h[ix(n, ii, k)] + y1 * h[ix(n, ii, k + 1)];
                    if notlast {
                        pp += z1 * h[ix(n, ii, k + 2)];
                        h[ix(n, ii, k + 2)] -= pp * r1;
                    }
                    h[ix(n, ii, k)] -= pp;
                    h[ix(n, ii, k + 1)] -= pp * q1;
                }
                for ii in 0..n {
                    let mut pp = x1 * q[ix(n, ii, k)] + y1 * q[ix(n, ii, k + 1)];
                    if notlast {
                        pp += z1 * q[ix(n, ii, k + 2)];
                        q[ix(n, ii, k + 2)] -= pp * r1;
                    }
                    q[ix(n, ii, k)] -= pp;
                    q[ix(n, ii, k + 1)] -= pp * q1;
                }
            }
        }
    }
    Err(ModalError::QrNoConvergence)
}

#[cfg(test)]
thread_local! {
    /// Test seam: `decompose` skips balancing.
    pub(crate) static NO_BALANCE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Test seam: the most iterations [`schur`] spent on one eigenvalue.
    pub(crate) static QR_MAX_ITS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    /// Test seam: [`certify`] prints each probe.
    pub(crate) static CERT_TRACE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Test seam: the last [`Modal::pade_estimate`]'s three differences.
    pub(crate) static LAST_DIFFS: std::cell::Cell<[f64; 3]> = const { std::cell::Cell::new([0.0; 3]) };
    /// Test seam: no block takes the decayed form ([`SOLVE_DECAY`]), so a
    /// test can show the error that form removes.
    pub(crate) static NO_SOLVES: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Test seam: [`certify`]'s evaluation at this rung of every ladder
    /// fails (-1: none).
    pub(crate) static CERT_FAIL_AT: std::cell::Cell<i32> = const { std::cell::Cell::new(-1) };
}

/// Whether a block whose largest real part times the step is `zr` takes
/// the decayed form of its input response ([`SOLVE_DECAY`]).
fn decayed(zr: f64) -> bool {
    #[cfg(test)]
    if NO_SOLVES.with(|c| c.get()) {
        return false;
    }
    zr <= -SOLVE_DECAY
}

fn finish_schur(h: &[f64], q: &[f64]) -> Result<(), ModalError> {
    if all_finite(h) && all_finite(q) {
        Ok(())
    } else {
        Err(ModalError::NonFinite)
    }
}

/// The diagonal block starting at `i` of a quasi-triangular `t`: 2 when
/// `t[i+1][i]` is nonzero, else 1.
fn block_size(t: &[f64], n: usize, i: usize) -> usize {
    if i + 1 < n && t[ix(n, i + 1, i)] != 0.0 {
        2
    } else {
        1
    }
}

/// `sqrt(|b c|)`, the frequency of a standardised pair `[a b; c a]`, to
/// within one ulp: the product is formed exactly (`fma` gives its rounding
/// error) and the square root corrected by it, so the angle `w h` of a long
/// step carries no extra rounding of its own. Falls back to `sqrt|b| sqrt|c|`
/// when the product would overflow or underflow.
pub fn pair_omega(b: f64, c: f64) -> f64 {
    let (b, c) = (b.abs(), c.abs());
    let p = b * c;
    if !(p.is_finite() && p > 1e-290) {
        return b.sqrt() * c.sqrt();
    }
    let e = b.mul_add(c, -p);
    let r = p.sqrt();
    r + (e - r.mul_add(r, -p)) / (2.0 * r)
}

/// The eigenvalue (re, im >= 0) of the 1x1 or standardised 2x2 block at `i`.
fn block_eig(t: &[f64], n: usize, i: usize) -> (f64, f64) {
    if block_size(t, n, i) == 2 {
        let b = t[ix(n, i, i + 1)];
        let c = t[ix(n, i + 1, i)];
        (0.5 * (t[ix(n, i, i)] + t[ix(n, i + 1, i + 1)]), pair_omega(b, c))
    } else {
        (t[ix(n, i, i)], 0.0)
    }
}

// ---- K3: Sylvester -------------------------------------------------------------

/// Solves the order <= 4 system `A Y - Y B = R` for one pair of diagonal
/// blocks (A pa x pa, B pb x pb, R pa x pb, all row-major, pa, pb in 1..=2)
/// by Gaussian elimination with complete pivoting on its Kronecker form.
/// A zero right-hand side gives `Y = 0` without a solve (a split of exactly
/// repeated, uncoupled eigenvalues is valid). A pivot below
/// `eps * max|K|` is singular: [`ModalError::SylvesterSingular`].
fn small_sylv(a: &[f64; 4], pa: usize, b: &[f64; 4], pb: usize, r: &[f64; 4]) -> Result<[f64; 4], ModalError> {
    let k = pa * pb;
    let mut y = [0.0f64; 4];
    if r[..k].iter().all(|&v| v == 0.0) {
        return Ok(y);
    }
    let mut m = [[0.0f64; 4]; 4];
    let mut rhs = [0.0f64; 4];
    for ri in 0..pa {
        for ci in 0..pb {
            let row = ri * pb + ci;
            rhs[row] = r[ri * pb + ci];
            for kk in 0..pa {
                m[row][kk * pb + ci] += a[ri * pa + kk];
            }
            for kk in 0..pb {
                m[row][ri * pb + kk] -= b[kk * pb + ci];
            }
        }
    }
    let mut kmax = 0.0f64;
    for row in m.iter().take(k) {
        for v in row.iter().take(k) {
            kmax = kmax.max(v.abs());
        }
    }
    let smin = (f64::EPSILON * kmax).max(f64::MIN_POSITIVE * 1e3);
    let mut colp = [0usize, 1, 2, 3];
    for step in 0..k {
        let (mut bi, mut bj, mut best) = (step, step, -1.0f64);
        for (i, row) in m.iter().enumerate().take(k).skip(step) {
            for (j, v) in row.iter().enumerate().take(k).skip(step) {
                if v.abs() > best {
                    best = v.abs();
                    bi = i;
                    bj = j;
                }
            }
        }
        if !(best >= smin) {
            return Err(ModalError::SylvesterSingular);
        }
        m.swap(step, bi);
        rhs.swap(step, bi);
        if bj != step {
            for row in m.iter_mut().take(k) {
                row.swap(step, bj);
            }
            colp.swap(step, bj);
        }
        let piv = m[step][step];
        for i in (step + 1)..k {
            let f = m[i][step] / piv;
            if f != 0.0 {
                for j in step..k {
                    m[i][j] -= f * m[step][j];
                }
                rhs[i] -= f * rhs[step];
            }
        }
    }
    let mut z = [0.0f64; 4];
    for i in (0..k).rev() {
        let mut s = rhs[i];
        for j in (i + 1)..k {
            s -= m[i][j] * z[j];
        }
        z[i] = s / m[i][i];
    }
    for i in 0..k {
        y[colp[i]] = z[i];
    }
    if y[..k].iter().all(|v| v.is_finite()) {
        Ok(y)
    } else {
        Err(ModalError::SylvesterSingular)
    }
}

/// K3. Solves `A X - X B = C` where `A = t[i0.., i0..]` (p x p) and
/// `B = t[j0.., j0..]` (q x q) are quasi-upper-triangular diagonal blocks of
/// `t` (n x n; their 2x2 blocks marked by nonzero subdiagonals) and `C`
/// (p x q, row-major in `c`) is overwritten with `X`, by block back
/// substitution (Bartels-Stewart): rows of `X` bottom-up, columns left to
/// right, one order <= 4 Kronecker system per pair of blocks. Returns
/// `|X|_F`. Gives up early, with [`ModalError::SylvesterSingular`], when an
/// entry of `X` exceeds `xmax` (the caller refuses such a split anyway) or a
/// block system is singular. Measured residual `|A X - X B - C|_F` at most
/// 1.1e-15 (|A|_F + |B|_F) |X|_F (`sylvester_residual`).
pub fn sylvester(t: &[f64], n: usize, i0: usize, p: usize, j0: usize, q: usize, c: &mut [f64], xmax: f64) -> Result<f64, ModalError> {
    if t.len() != n * n || c.len() < p * q || i0 + p > n || j0 + q > n {
        return Err(ModalError::Shape);
    }
    // Block starts of A (descending) and B (ascending).
    let mut ra = [0usize; MAX_N];
    let mut na = 0usize;
    let mut r = 0usize;
    while r < p && na < MAX_N {
        ra[na] = r;
        na += 1;
        r += if r + 1 < p && t[ix(n, i0 + r + 1, i0 + r)] != 0.0 { 2 } else { 1 };
    }
    let mut cb = [0usize; MAX_N];
    let mut nb = 0usize;
    let mut cc = 0usize;
    while cc < q && nb < MAX_N {
        cb[nb] = cc;
        nb += 1;
        cc += if cc + 1 < q && t[ix(n, j0 + cc + 1, j0 + cc)] != 0.0 { 2 } else { 1 };
    }
    let bsize = |starts: &[usize], cnt: usize, k: usize, total: usize| if k + 1 < cnt { starts[k + 1] - starts[k] } else { total - starts[k] };
    let mut norm2 = 0.0f64;
    for bi in (0..na).rev() {
        let r0 = ra[bi];
        let pa = bsize(&ra, na, bi, p);
        for bj in 0..nb {
            let c0 = cb[bj];
            let pb = bsize(&cb, nb, bj, q);
            let mut rhs = [0.0f64; 4];
            for ri in 0..pa {
                for ci in 0..pb {
                    let (row, col) = (r0 + ri, c0 + ci);
                    let mut s = c[row * q + col];
                    for k in (r0 + pa)..p {
                        s -= t[ix(n, i0 + row, i0 + k)] * c[k * q + col];
                    }
                    for k in 0..c0 {
                        s += c[row * q + k] * t[ix(n, j0 + k, j0 + col)];
                    }
                    rhs[ri * pb + ci] = s;
                }
            }
            let mut ab = [0.0f64; 4];
            for ri in 0..pa {
                for k in 0..pa {
                    ab[ri * pa + k] = t[ix(n, i0 + r0 + ri, i0 + r0 + k)];
                }
            }
            let mut bb = [0.0f64; 4];
            for k in 0..pb {
                for ci in 0..pb {
                    bb[k * pb + ci] = t[ix(n, j0 + c0 + k, j0 + c0 + ci)];
                }
            }
            let y = small_sylv(&ab, pa, &bb, pb, &rhs)?;
            for ri in 0..pa {
                for ci in 0..pb {
                    let v = y[ri * pb + ci];
                    if !(v.abs() <= xmax) {
                        return Err(ModalError::SylvesterSingular);
                    }
                    c[(r0 + ri) * q + c0 + ci] = v;
                    norm2 += v * v;
                }
            }
        }
    }
    Ok(norm2.sqrt())
}

// ---- K4: block grouping ---------------------------------------------------------

/// Applies the orthogonal `w` (k x k, row-major) on indices `j..j+k` as a
/// similarity of the quasi-triangular `t`: `t <- W^T t W` restricted to rows
/// `j..j+k` (columns `j..`) and columns `j..j+k` (rows `..j+k`), `s <- s W`,
/// `sinv <- W^T sinv`.
fn apply_orth(t: &mut [f64], s: &mut [f64], sinv: &mut [f64], n: usize, j: usize, k: usize, w: &[f64; 16]) {
    let mut tmp = [0.0f64; 4];
    for col in j..n {
        for a in 0..k {
            let mut acc = 0.0;
            for b in 0..k {
                acc += w[b * 4 + a] * t[ix(n, j + b, col)];
            }
            tmp[a] = acc;
        }
        for a in 0..k {
            t[ix(n, j + a, col)] = tmp[a];
        }
        for a in 0..k {
            let mut acc = 0.0;
            for b in 0..k {
                acc += w[b * 4 + a] * sinv[ix(n, j + b, col)];
            }
            tmp[a] = acc;
        }
        for a in 0..k {
            sinv[ix(n, j + a, col)] = tmp[a];
        }
    }
    for col in 0..j {
        for a in 0..k {
            let mut acc = 0.0;
            for b in 0..k {
                acc += w[b * 4 + a] * sinv[ix(n, j + b, col)];
            }
            tmp[a] = acc;
        }
        for a in 0..k {
            sinv[ix(n, j + a, col)] = tmp[a];
        }
    }
    for row in 0..n {
        let in_t = row < j + k;
        for a in 0..k {
            let mut acc = 0.0;
            for b in 0..k {
                acc += s[ix(n, row, j + b)] * w[b * 4 + a];
            }
            tmp[a] = acc;
        }
        for a in 0..k {
            s[ix(n, row, j + a)] = tmp[a];
        }
        if in_t {
            for a in 0..k {
                let mut acc = 0.0;
                for b in 0..k {
                    acc += t[ix(n, row, j + b)] * w[b * 4 + a];
                }
                tmp[a] = acc;
            }
            for a in 0..k {
                t[ix(n, row, j + a)] = tmp[a];
            }
        }
    }
}

/// Swaps the adjacent diagonal blocks of `t` at `j` (order p1) and `j + p1`
/// (order p2) by an orthogonal similarity (LAPACK `dlaexc`): for two 1x1
/// blocks a plane rotation; otherwise `X` from `T11 X - X T22 = T12`, the QR
/// factorisation of `[-X; I]`, and the weak stability test (the new lower
/// left block at most `10 eps |D|` before it is zeroed). Updates `s` and
/// `sinv`, and re-standardises the new 2x2 blocks. Fails with
/// [`ModalError::SwapRejected`] (nothing changed) when the blocks share an
/// eigenvalue or the test fails.
fn swap_blocks(t: &mut [f64], s: &mut [f64], sinv: &mut [f64], n: usize, j: usize, p1: usize, p2: usize) -> Result<(), ModalError> {
    if p1 == 1 && p2 == 1 {
        let t11 = t[ix(n, j, j)];
        let t22 = t[ix(n, j + 1, j + 1)];
        let f = t[ix(n, j, j + 1)];
        let g = t22 - t11;
        let r = f.hypot(g);
        let (cs, sn) = if r == 0.0 { (1.0, 0.0) } else { (f / r, g / r) };
        rot_outside(t, n, j, cs, sn, s, Some(sinv));
        t[ix(n, j, j)] = t22;
        t[ix(n, j + 1, j + 1)] = t11;
        t[ix(n, j + 1, j)] = 0.0;
        return Ok(());
    }
    let k = p1 + p2;
    let mut d = [0.0f64; 16];
    let mut dnorm = 0.0f64;
    for a in 0..k {
        for b in 0..k {
            d[a * 4 + b] = t[ix(n, j + a, j + b)];
            dnorm = dnorm.max(d[a * 4 + b].abs());
        }
    }
    let thresh = (10.0 * f64::EPSILON * dnorm).max(f64::MIN_POSITIVE * 1e3);
    let mut a11 = [0.0f64; 4];
    for a in 0..p1 {
        for b in 0..p1 {
            a11[a * p1 + b] = d[a * 4 + b];
        }
    }
    let mut a22 = [0.0f64; 4];
    for a in 0..p2 {
        for b in 0..p2 {
            a22[a * p2 + b] = d[(p1 + a) * 4 + p1 + b];
        }
    }
    let mut a12 = [0.0f64; 4];
    for a in 0..p1 {
        for b in 0..p2 {
            a12[a * p2 + b] = d[a * 4 + p1 + b];
        }
    }
    let x = small_sylv(&a11, p1, &a22, p2, &a12).map_err(|_| ModalError::SwapRejected)?;
    // M = [-X; I] (k x p2); W = H_1 .. H_p2 from its QR.
    let mut mm = [0.0f64; 8];
    for a in 0..p1 {
        for b in 0..p2 {
            mm[a * 2 + b] = -x[a * p2 + b];
        }
    }
    for a in 0..p2 {
        mm[(p1 + a) * 2 + a] = 1.0;
    }
    let mut w = [0.0f64; 16];
    for a in 0..k {
        w[a * 4 + a] = 1.0;
    }
    for c in 0..p2 {
        let mut v = [0.0f64; 4];
        let mut ss = 0.0;
        let mut scale = 0.0f64;
        for a in c..k {
            scale = scale.max(mm[a * 2 + c].abs());
        }
        if scale == 0.0 {
            return Err(ModalError::SwapRejected);
        }
        for a in c..k {
            v[a] = mm[a * 2 + c] / scale;
            ss += v[a] * v[a];
        }
        let sigma = sign(ss.sqrt(), v[c]);
        v[c] += sigma;
        let beta = 1.0 / (sigma * v[c]);
        for cc in c..p2 {
            let mut acc = 0.0;
            for a in c..k {
                acc += v[a] * mm[a * 2 + cc];
            }
            acc *= beta;
            for a in c..k {
                mm[a * 2 + cc] -= acc * v[a];
            }
        }
        for row in 0..k {
            let mut acc = 0.0;
            for a in c..k {
                acc += w[row * 4 + a] * v[a];
            }
            acc *= beta;
            for a in c..k {
                w[row * 4 + a] -= acc * v[a];
            }
        }
    }
    // Weak stability test on W^T D W's lower-left p1 x p2 block.
    let mut dw = [0.0f64; 16];
    for a in 0..k {
        for b in 0..k {
            let mut acc = 0.0;
            for c in 0..k {
                acc += d[a * 4 + c] * w[c * 4 + b];
            }
            dw[a * 4 + b] = acc;
        }
    }
    for a in p2..k {
        for b in 0..p2 {
            let mut acc = 0.0;
            for c in 0..k {
                acc += w[c * 4 + a] * dw[c * 4 + b];
            }
            if !(acc.abs() <= thresh) {
                return Err(ModalError::SwapRejected);
            }
        }
    }
    apply_orth(t, s, sinv, n, j, k, &w);
    for a in p2..k {
        for b in 0..p2 {
            t[ix(n, j + a, j + b)] = 0.0;
        }
    }
    if p2 == 2 {
        standardize(t, n, j, s, Some(sinv));
    }
    if p1 == 2 {
        standardize(t, n, j + p2, s, Some(sinv));
    }
    Ok(())
}

/// One block of the block-diagonal `D`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Block {
    pub start: usize,
    pub size: usize,
    pub kind: BlockKind,
}

/// What a block of `D` is, which decides how its exponential is evaluated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockKind {
    /// Order 1: a real rate, closed form ([`real_exp_phi`]).
    Real,
    /// Order 2, standardised `[a b; c a]` with `b c < 0`: a complex pair,
    /// closed form ([`pair_exp_phi`]).
    Pair,
    /// A merged cluster (repeated or close eigenvalues): Pade 13.
    Merged,
}

/// K4. Bavely-Stewart block diagonalisation of the real Schur form `t`
/// (n x n, from [`schur`]), with `s` = Q and `sinv` = Q^T on entry. Walks the
/// diagonal: the current cluster starts as one block; it is split off the
/// trailing rest when `T11 X - X T22 = -T12` has `|X|_F <= split_max` (then
/// `T12 <- 0`, `S[:, rest] += S[:, cl] X`, `Sinv[cl, :] -= X Sinv[rest, :]`);
/// otherwise the block of the rest whose eigenvalue is nearest (absolute
/// distance) to the cluster's is moved up next to it by orthogonal swaps and
/// the cluster grows by it (a swap that fails merges every block in
/// between). On exit `t` is block diagonal (`D`), `A = S D S^-1`, and the
/// blocks are returned. Bounded: at most n clusters, each growing at most n
/// times, each growth at most n swaps.
pub fn group(t: &mut [f64], s: &mut [f64], sinv: &mut [f64], n: usize, split_max: f64) -> Result<Vec<Block>, ModalError> {
    if t.len() != n * n || s.len() != n * n || sinv.len() != n * n || n > MAX_N {
        return Err(ModalError::Shape);
    }
    let mut out = Vec::new();
    let mut x = vec![0.0f64; n * n];
    let mut g0 = 0usize;
    for _cluster in 0..=n {
        if g0 >= n {
            break;
        }
        let mut g1 = g0 + block_size(t, n, g0);
        for _grow in 0..=n {
            if g1 >= n {
                g1 = n;
                break;
            }
            let (p, q) = (g1 - g0, n - g1);
            for a in 0..p {
                for b in 0..q {
                    x[a * q + b] = -t[ix(n, g0 + a, g1 + b)];
                }
            }
            if let Ok(norm) = sylvester(t, n, g0, p, g1, q, &mut x, split_max) {
                if norm <= split_max {
                    // S[:, rest] += S[:, cl] X ; Sinv[cl, :] -= X Sinv[rest, :]
                    for row in 0..n {
                        for b in 0..q {
                            let mut acc = 0.0;
                            for a in 0..p {
                                acc += s[ix(n, row, g0 + a)] * x[a * q + b];
                            }
                            s[ix(n, row, g1 + b)] += acc;
                        }
                    }
                    for a in 0..p {
                        for col in 0..n {
                            let mut acc = 0.0;
                            for b in 0..q {
                                acc += x[a * q + b] * sinv[ix(n, g1 + b, col)];
                            }
                            sinv[ix(n, g0 + a, col)] -= acc;
                        }
                        for b in 0..q {
                            t[ix(n, g0 + a, g1 + b)] = 0.0;
                        }
                    }
                    break;
                }
            }
            // Refused: bring the nearest eigenvalue of the rest up to g1.
            let mut eigs = [(0.0f64, 0.0f64); MAX_N];
            let mut ne = 0usize;
            let mut i = g0;
            while i < g1 && ne < MAX_N {
                eigs[ne] = block_eig(t, n, i);
                ne += 1;
                i += block_size(t, n, i);
            }
            let (mut best, mut best_at) = (f64::INFINITY, g1);
            let mut i = g1;
            while i < n {
                let (re, im) = block_eig(t, n, i);
                for &(er, ei) in eigs.iter().take(ne) {
                    let dd = (re - er).hypot(im.abs() - ei.abs());
                    if dd < best {
                        best = dd;
                        best_at = i;
                    }
                }
                i += block_size(t, n, i);
            }
            let mut pos = best_at;
            let mut ok = true;
            for _swap in 0..n {
                if pos <= g1 {
                    break;
                }
                // The block just before `pos`.
                let mut prev = g1;
                let mut i = g1;
                while i < pos {
                    prev = i;
                    i += block_size(t, n, i);
                }
                let (p1, p2) = (pos - prev, block_size(t, n, pos));
                if swap_blocks(t, s, sinv, n, prev, p1, p2).is_err() {
                    ok = false;
                    break;
                }
                pos = prev;
            }
            if ok {
                g1 += block_size(t, n, g1);
            } else {
                g1 = pos + block_size(t, n, pos);
            }
        }
        let size = g1 - g0;
        let kind = match size {
            1 => BlockKind::Real,
            2 if t[ix(n, g0 + 1, g0)] != 0.0 => BlockKind::Pair,
            _ => BlockKind::Merged,
        };
        out.push(Block { start: g0, size, kind });
        g0 = g1;
    }
    if g0 < n {
        return Err(ModalError::Shape);
    }
    Ok(out)
}

// ---- K5: closed-form block exponentials and phi functions ------------------------

/// `sin(x) / x`, series below 1e-2 (error < 1e-20 there).
fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-2 {
        let x2 = x * x;
        1.0 - x2 / 6.0 * (1.0 - x2 / 20.0 * (1.0 - x2 / 42.0))
    } else {
        x.sin() / x
    }
}

/// K5, order 1. `[e^z, phi_1(z), phi_2(z)]` for real `z`, `phi_1(z) =
/// (e^z - 1) / z`, `phi_2(z) = (e^z - 1 - z) / z^2`, so the block `y' = l y +
/// g0 + g1 t` gives `y(h) = e^z y0 + h phi_1 g0 + h^2 phi_2 g1` with `z = l h`.
/// `phi_1` is `expm1(z) / z` everywhere (1 at 0); `phi_2` is its power series
/// for `|z| <` [`SERIES_SWITCH`] and `(phi_1 - 1) / z` above (no overflow:
/// no `z^2` is formed). `e^z` is exactly 0 below z = -745.2. Measured against
/// the double-double oracle for `|z|` from 1e-12 to 700, both signs, and
/// across the switch: at most 2.2e-16 relative (`block_closed_forms`). A
/// growing `z` above 709.8 overflows: the caller sees a non-finite result.
pub fn real_exp_phi(z: f64) -> [f64; 3] {
    let e = z.exp();
    let p1 = if z == 0.0 { 1.0 } else { z.exp_m1() / z };
    let p2 = if z.abs() < SERIES_SWITCH {
        let mut acc = 0.0;
        // sum_{j} z^j / (j + 2)!, Horner from the top.
        for j in (0..SERIES_TERMS).rev() {
            acc = acc * z / ((j + 3) as f64) + 1.0;
        }
        acc / 2.0
    } else {
        (p1 - 1.0) / z
    };
    [e, p1, p2]
}

/// K5, order 2. For `z = sr + i si` (si > 0), `[(Re f(z), Im f(z) / si)]` for
/// `f` = exp, phi_1, phi_2. A standardised block `M = [a b; c a]` with
/// eigenvalues `a +- i w` then has `f(M h) = Re f I + (Im f / si) h [0 b; c
/// 0]` with `z = (a + i w) h`, entry by entry a product of accurate scalars,
/// however non-normal the block. Below `|z|` = [`SERIES_SWITCH`] the series
/// with `Im z^j / si` carried by recurrence (no division by `si`); above it
/// `e^z - 1 = expm1(sr) cos si - 2 sin^2(si / 2) + i e^sr sin si` and the
/// divisions by `z` scaled by `|z|` (no overflow, no cancellation at a small
/// `si`). Measured against the double-double oracle for `|z|` from 1e-12 to
/// 1e3, damping ratios from 1e-5 to 1 and block non-normality to 1e6: at
/// most 4.4e-16 relative to the block's norm (`block_closed_forms`).
pub fn pair_exp_phi(sr: f64, si: f64) -> [(f64, f64); 3] {
    let er = sr.exp();
    let ex = (er * si.cos(), er * sinc(si));
    let m = sr.hypot(si);
    if m < SERIES_SWITCH {
        // z^j = p_j + i si r_j : p_{j+1} = sr p_j - si^2 r_j, r_{j+1} = p_j + sr r_j.
        let mut fact1 = 1.0f64; // (j + 1)!
        let mut fact2 = 2.0f64; // (j + 2)!
        let (mut p, mut r) = (1.0f64, 0.0f64);
        let (mut a1, mut b1, mut a2, mut b2) = (0.0, 0.0, 0.0, 0.0);
        for j in 0..SERIES_TERMS {
            a1 += p / fact1;
            b1 += r / fact1;
            a2 += p / fact2;
            b2 += r / fact2;
            let np = sr * p - si * si * r;
            let nr = p + sr * r;
            p = np;
            r = nr;
            fact1 *= (j + 2) as f64;
            fact2 *= (j + 3) as f64;
        }
        return [ex, (a1, b1), (a2, b2)];
    }
    let (ur, ui) = (sr / m, si / m);
    let u = sr.exp_m1() * si.cos() - 2.0 * (0.5 * si).sin().powi(2);
    let v = ex.1; // Im e^z / si
    // phi_1 = (u + i si v) (ur - i ui) / m
    let re1 = (u * ur + er * si.sin() * ui) / m;
    let r1 = (v * ur - u / m) / m;
    // phi_2 = ((re1 - 1) + i si r1) (ur - i ui) / m
    let pm = re1 - 1.0;
    let re2 = (pm * ur + si * r1 * ui) / m;
    let r2 = (r1 * ur - pm / m) / m;
    [ex, (re1, r1), (re2, r2)]
}

// ---- K6: Pade 13 ----------------------------------------------------------------

/// Caller-owned buffers for [`pade_exp`] on matrices up to order `k`.
#[derive(Debug, Clone)]
pub struct PadeWork {
    k: usize,
    /// Input and output (k x k, row-major, leading `dim` used).
    pub m: Vec<f64>,
    a2: Vec<f64>,
    a4: Vec<f64>,
    a6: Vec<f64>,
    t: Vec<f64>,
    u: Vec<f64>,
    v: Vec<f64>,
    piv: Vec<usize>,
    /// The squarings the last [`pade_exp`] took.
    pub last_s: i32,
}

impl PadeWork {
    pub fn new(k: usize) -> PadeWork {
        let z = vec![0.0; k * k];
        PadeWork { k, m: z.clone(), a2: z.clone(), a4: z.clone(), a6: z.clone(), t: z.clone(), u: z.clone(), v: z, piv: vec![0; k], last_s: 0 }
    }
    pub fn capacity(&self) -> usize {
        self.k
    }
}

fn matmul(c: &mut [f64], a: &[f64], b: &[f64], k: usize) {
    for v in c.iter_mut().take(k * k) {
        *v = 0.0;
    }
    for i in 0..k {
        for l in 0..k {
            let x = a[i * k + l];
            if x != 0.0 {
                for j in 0..k {
                    c[i * k + j] += x * b[l * k + j];
                }
            }
        }
    }
}

const PADE_B: [f64; 14] = [
    64764752532480000.0,
    32382376266240000.0,
    7771770303897600.0,
    1187353796428800.0,
    129060195264000.0,
    10559470521600.0,
    670442572800.0,
    33522128640.0,
    1323241920.0,
    40840800.0,
    960960.0,
    16380.0,
    182.0,
    1.0,
];

/// K6. `w.m <- exp(w.m)` for the leading `dim x dim` (row-major, stride
/// `dim`) by the [13/13] Pade approximant with scaling and squaring (Higham
/// 2005: scale by `2^-s` to 1-norm <= theta_13 = 5.37, then square `s`
/// times), no allocation. Fails with [`ModalError::PadeFailed`] on a singular
/// denominator (LU with partial pivoting, a zero or non-finite pivot) or
/// `s >` [`PADE_MAX_SQUARINGS`], [`ModalError::NonFinite`] on a non-finite
/// input or result. Accuracy, measured against the double-double oracle on
/// the random set (`pade_against_dd`, R151's criterion): within 100 eps
/// times the exponential's own sensitivity to its f64 input, at worst 0.55
/// of that (1.1e-10 relative at 7 squarings, sensitivity 9.3e3 eps); 2.7e-13
/// with no squaring. Each squaring can double the error of a non-normal
/// matrix, which is why the design splits stiff blocks off first.
pub fn pade_exp(w: &mut PadeWork, dim: usize) -> Result<(), ModalError> {
    pade_exp_scaled(w, dim, 0, PadeForm::P13)
}

/// How [`pade_exp_scaled`] evaluates. `P13`: [13/13] at `theta_13`,
/// squaring `E` itself, the step's (K6). `P9`: [9/9] at `theta_9`, squaring
/// `E`. `P13Minus1`: [13/13] at `theta_13` carried as `F = E - I` through the
/// squarings (`F <- 2 F + F^2` from `r - I = (V - U)^-1 2 U`, then `E = I +
/// F`). Squaring `E` rounds a diagonal entry's departure from 1 away at the
/// scaled level (`1 + lambda h / 2^s` is 1 once `|lambda h| / 2^s` is below
/// eps / 2), the same at every scaling and every degree, so a near-zero
/// rate's `e^(lambda h) - 1` is lost in all of them alike, and no
/// difference between them shows it (EX1D: 3.5e-11 relative at 20
/// squarings on an n = 16 Jordan member with a rate of 4e-16, doubling with
/// each squaring, while `P13` at `s`, `s + 1`, `s + 6` and `P9` agreed to
/// 1e-16 of each other; `P13Minus1` was 2.1e-16 from the oracle). Carried
/// as `F`, the loss is the other way about (a decayed entry's `E` near 0
/// lives in `F` near -1), so `P13Minus1` is the certification's
/// differently-wrong evaluation, not a better step: as K6 on the random set
/// it was worse than `P13` (squarings 1 to 4: 0.52 of R151's allowance
/// against 0.14), and as the step it left errors 600 times the scaling
/// differences.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PadeForm {
    P13,
    P9,
    P13Minus1,
}

/// Pade 9's coefficients (Higham 2005), for the certification's
/// evaluations only ([`PadeForm::P9`]).
const PADE9_B: [f64; 10] = [17643225600.0, 8821612800.0, 2075673600.0, 302702400.0, 30270240.0, 2162160.0, 110880.0, 3960.0, 90.0, 1.0];

/// Pade 9's scaling threshold `theta_9` (Higham 2005).
const THETA9: f64 = 2.097847961257068;

/// [`pade_exp`] with `extra` squarings beyond its own choice, in the given
/// [`PadeForm`] (the certification's other evaluations, whose differences
/// from the first estimate its error; see [`Modal::pade_estimate`]).
fn pade_exp_scaled(w: &mut PadeWork, dim: usize, extra: i32, form: PadeForm) -> Result<(), ModalError> {
    let deg9 = form == PadeForm::P9;
    if dim > w.k || dim == 0 {
        return Err(ModalError::Shape);
    }
    let k = dim;
    let kk = k * k;
    if !all_finite(&w.m[..kk]) {
        return Err(ModalError::NonFinite);
    }
    let norm = norm1(&w.m[..kk], k);
    if !norm.is_finite() {
        return Err(ModalError::NonFinite);
    }
    // Squarings from Al-Mohy and Higham (2009): the truncation is bounded
    // by eta = min(max(d6, d8), max(d8, d10)), d_k = |A^k|_1^(1/k), which
    // for a non-normal matrix can be far below |A|_1, so fewer squarings
    // (each can double the error) are taken. The powers are of the
    // unscaled matrix; A^8 and A^10 cost two products more.
    matmul(&mut w.a2, &w.m, &w.m, k);
    matmul(&mut w.a4, &w.a2, &w.a2, k);
    matmul(&mut w.a6, &w.a4, &w.a2, k);
    matmul(&mut w.t, &w.a4, &w.a4, k);
    let d8 = norm1(&w.t[..kk], k).powf(1.0 / 8.0);
    matmul(&mut w.u, &w.a6, &w.a4, k);
    let d10 = norm1(&w.u[..kk], k).powf(1.0 / 10.0);
    let d6 = norm1(&w.a6[..kk], k).powf(1.0 / 6.0);
    // Non-finite powers (a norm past 1e51) fall back to the 1-norm.
    let eta = if d6.is_finite() && d8.is_finite() && d10.is_finite() { d6.max(d8).min(d8.max(d10)).min(norm) } else { norm };
    let theta = if deg9 { THETA9 } else { THETA13 };
    let s: i32 = if eta > theta { (eta / theta).log2().ceil() as i32 } else { 0 } + extra;
    if !(0..=PADE_MAX_SQUARINGS).contains(&s) {
        return Err(ModalError::PadeFailed);
    }
    w.last_s = s;
    // Scale, and form the powers again from the scaled matrix (scaling the
    // unscaled powers could overflow or underflow at large norms).
    let sc = 2.0f64.powi(-s);
    for v in w.m[..kk].iter_mut() {
        *v *= sc;
    }
    if s > 0 {
        matmul(&mut w.a2, &w.m, &w.m, k);
        matmul(&mut w.a4, &w.a2, &w.a2, k);
        matmul(&mut w.a6, &w.a4, &w.a2, k);
    }
    if deg9 {
        // U = A (b9 A8 + b7 A6 + b5 A4 + b3 A2 + b1 I), V = b8 A8 + b6 A6 +
        // b4 A4 + b2 A2 + b0 I; U into t, V into v.
        let b = &PADE9_B;
        matmul(&mut w.t, &w.a4, &w.a4, k); // A8
        for i in 0..kk {
            w.v[i] = b[8] * w.t[i] + b[6] * w.a6[i] + b[4] * w.a4[i] + b[2] * w.a2[i];
            w.u[i] = b[9] * w.t[i] + b[7] * w.a6[i] + b[5] * w.a4[i] + b[3] * w.a2[i];
        }
        for i in 0..k {
            w.v[i * k + i] += b[0];
            w.u[i * k + i] += b[1];
        }
        matmul(&mut w.t, &w.m, &w.u, k);
    } else {
        pade13_uv(w, k);
    }
    pade_finish(w, k, s, form == PadeForm::P13Minus1)
}

/// [13/13]'s `U` into `w.t` and `V` into `w.v`, from the scaled powers.
fn pade13_uv(w: &mut PadeWork, k: usize) {
    let kk = k * k;
    let b = &PADE_B;
    for i in 0..kk {
        w.t[i] = b[13] * w.a6[i] + b[11] * w.a4[i] + b[9] * w.a2[i];
    }
    matmul(&mut w.u, &w.a6, &w.t, k);
    for i in 0..kk {
        w.u[i] += b[7] * w.a6[i] + b[5] * w.a4[i] + b[3] * w.a2[i];
    }
    for i in 0..k {
        w.u[i * k + i] += b[1];
    }
    matmul(&mut w.t, &w.m, &w.u, k); // t = U
    for i in 0..kk {
        w.u[i] = b[12] * w.a6[i] + b[10] * w.a4[i] + b[8] * w.a2[i];
    }
    matmul(&mut w.v, &w.a6, &w.u, k);
    for i in 0..kk {
        w.v[i] += b[6] * w.a6[i] + b[4] * w.a4[i] + b[2] * w.a2[i];
    }
    for i in 0..k {
        w.v[i * k + i] += b[0];
    }
}

/// The rational approximant `(V - U)^-1 (V + U)` from `U` in `w.t` and `V`
/// in `w.v`, squared `s` times, into `w.m`; with `minus1`, carried as
/// `(V - U)^-1 2 U = r - I` through the squarings `F <- 2 F + F^2`.
fn pade_finish(w: &mut PadeWork, k: usize, s: i32, minus1: bool) -> Result<(), ModalError> {
    let kk = k * k;
    // num = V + U (or 2 U) into a2, den = V - U into a4; solve den X = num.
    for i in 0..kk {
        w.a2[i] = if minus1 { 2.0 * w.t[i] } else { w.v[i] + w.t[i] };
        w.a4[i] = w.v[i] - w.t[i];
    }
    lu_in_place(&mut w.a4, &mut w.piv, k)?;
    for col in 0..k {
        // Forward with the interleaved row swaps, then back substitution.
        for r in 0..k {
            let p = w.piv[r];
            if p != r {
                w.a2.swap(r * k + col, p * k + col);
            }
            let br = w.a2[r * k + col];
            if br != 0.0 {
                for i in (r + 1)..k {
                    w.a2[i * k + col] -= w.a4[i * k + r] * br;
                }
            }
        }
        for r in (0..k).rev() {
            let mut acc = w.a2[r * k + col];
            for j in (r + 1)..k {
                acc -= w.a4[r * k + j] * w.a2[j * k + col];
            }
            w.a2[r * k + col] = acc / w.a4[r * k + r];
        }
    }
    for _ in 0..s {
        matmul(&mut w.a4, &w.a2, &w.a2, k);
        if minus1 {
            for i in 0..kk {
                w.a4[i] += 2.0 * w.a2[i];
            }
        }
        std::mem::swap(&mut w.a2, &mut w.a4);
    }
    if minus1 {
        for i in 0..k {
            w.a2[i * k + i] += 1.0;
        }
    }
    w.m[..kk].copy_from_slice(&w.a2[..kk]);
    if all_finite(&w.m[..kk]) {
        Ok(())
    } else {
        Err(ModalError::NonFinite)
    }
}

// ---- the certification's double-double measurement ----------------------------

/// A double-double number (`hi + lo`, about 32 digits), for the
/// certification's measurement of a merged block's Pade error only
/// ([`Modal::pade_measured`]); nothing per step uses it.
#[derive(Debug, Clone, Copy)]
struct Dd {
    hi: f64,
    lo: f64,
}

impl Dd {
    const ZERO: Dd = Dd { hi: 0.0, lo: 0.0 };
    fn f(x: f64) -> Dd {
        Dd { hi: x, lo: 0.0 }
    }
    fn quick(a: f64, b: f64) -> Dd {
        let s = a + b;
        Dd { hi: s, lo: b - (s - a) }
    }
    fn add(self, y: Dd) -> Dd {
        let two = |a: f64, b: f64| {
            let s = a + b;
            let bb = s - a;
            (s, (a - (s - bb)) + (b - bb))
        };
        let (s, e) = two(self.hi, y.hi);
        let (t, f) = two(self.lo, y.lo);
        let r = Dd::quick(s, e + t);
        Dd::quick(r.hi, r.lo + f)
    }
    fn sub(self, y: Dd) -> Dd {
        self.add(Dd { hi: -y.hi, lo: -y.lo })
    }
    fn mul(self, y: Dd) -> Dd {
        let p = self.hi * y.hi;
        let e = self.hi.mul_add(y.hi, -p) + (self.hi * y.lo + self.lo * y.hi);
        Dd::quick(p, e)
    }
    fn div(self, y: Dd) -> Dd {
        let q1 = self.hi / y.hi;
        let r = self.sub(y.mul(Dd::f(q1)));
        let q2 = r.hi / y.hi;
        let r = r.sub(y.mul(Dd::f(q2)));
        Dd::quick(q1, q2).add(Dd::f(r.hi / y.hi))
    }
}

/// `exp(pm)` (`dim x dim`, row-major) in double-double: [13/13] scaled by
/// `2^-s` to a 1-norm at most `theta_13 / 2`, then `s` squarings. A
/// different scaling from the step's (Al-Mohy and Higham's at `theta_13`)
/// and from the tests' oracle (`theta_13 / 4`), so it shares no rounding
/// with either. Errors as [`pade_exp`]'s: [`ModalError::PadeFailed`] on a
/// singular denominator or too many squarings, [`ModalError::NonFinite`].
fn dd_expm(pm: &[Dd], dim: usize) -> Result<Vec<Dd>, ModalError> {
    let k = dim;
    let hi: Vec<f64> = pm[..k * k].iter().map(|v| v.hi).collect();
    let norm = norm1(&hi, k);
    if !norm.is_finite() {
        return Err(ModalError::NonFinite);
    }
    let lim = THETA13 / 2.0;
    let s = if norm > lim { (norm / lim).log2().ceil() as i32 } else { 0 };
    if !(0..=PADE_MAX_SQUARINGS).contains(&s) {
        return Err(ModalError::PadeFailed);
    }
    // A power of two scales both parts exactly.
    let sc = 2f64.powi(-s);
    let a: Vec<Dd> = pm[..k * k].iter().map(|v| Dd { hi: v.hi * sc, lo: v.lo * sc }).collect();
    let mm = |x: &[Dd], y: &[Dd]| -> Vec<Dd> {
        let mut r = vec![Dd::ZERO; k * k];
        for i in 0..k {
            for l in 0..k {
                let v = x[i * k + l];
                if v.hi != 0.0 {
                    for j in 0..k {
                        r[i * k + j] = r[i * k + j].add(v.mul(y[l * k + j]));
                    }
                }
            }
        }
        r
    };
    let lin = |terms: &[(&[Dd], f64)], id: f64| -> Vec<Dd> {
        let mut r = vec![Dd::ZERO; k * k];
        for (x, c) in terms {
            for i in 0..k * k {
                r[i] = r[i].add(x[i].mul(Dd::f(*c)));
            }
        }
        for i in 0..k {
            r[i * k + i] = r[i * k + i].add(Dd::f(id));
        }
        r
    };
    let b = &PADE_B;
    let a2 = mm(&a, &a);
    let a4 = mm(&a2, &a2);
    let a6 = mm(&a4, &a2);
    let t = lin(&[(&a6, b[13]), (&a4, b[11]), (&a2, b[9])], 0.0);
    let u = lin(&[(&mm(&a6, &t), 1.0), (&a6, b[7]), (&a4, b[5]), (&a2, b[3])], b[1]);
    let u = mm(&a, &u);
    let t = lin(&[(&a6, b[12]), (&a4, b[10]), (&a2, b[8])], 0.0);
    let v = lin(&[(&mm(&a6, &t), 1.0), (&a6, b[6]), (&a4, b[4]), (&a2, b[2])], b[0]);
    let mut x: Vec<Dd> = (0..k * k).map(|i| v[i].add(u[i])).collect();
    let mut den: Vec<Dd> = (0..k * k).map(|i| v[i].sub(u[i])).collect();
    for c in 0..k {
        let mut p = c;
        for r in (c + 1)..k {
            if den[r * k + c].hi.abs() > den[p * k + c].hi.abs() {
                p = r;
            }
        }
        if !(den[p * k + c].hi != 0.0) || !den[p * k + c].hi.is_finite() {
            return Err(ModalError::PadeFailed);
        }
        if p != c {
            for j in 0..k {
                den.swap(c * k + j, p * k + j);
                x.swap(c * k + j, p * k + j);
            }
        }
        let piv = den[c * k + c];
        for r in (c + 1)..k {
            let f = den[r * k + c].div(piv);
            if f.hi != 0.0 {
                for j in c..k {
                    den[r * k + j] = den[r * k + j].sub(f.mul(den[c * k + j]));
                }
                for j in 0..k {
                    x[r * k + j] = x[r * k + j].sub(f.mul(x[c * k + j]));
                }
            }
        }
    }
    for c in (0..k).rev() {
        for j in 0..k {
            let mut acc = x[c * k + j];
            for l in (c + 1)..k {
                acc = acc.sub(den[c * k + l].mul(x[l * k + j]));
            }
            x[c * k + j] = acc.div(den[c * k + c]);
        }
    }
    for _ in 0..s {
        x = mm(&x, &x);
    }
    if x.iter().all(|v| v.hi.is_finite() && v.lo.is_finite()) {
        Ok(x)
    } else {
        Err(ModalError::NonFinite)
    }
}

/// LU with partial pivoting in place (LINPACK layout, as `crate::lu`, which
/// this cannot reuse: it consumes a `Vec` and the step must not allocate).
fn lu_in_place(a: &mut [f64], piv: &mut [usize], k: usize) -> Result<(), ModalError> {
    for c in 0..k {
        let mut best = c;
        let mut bv = -1.0f64;
        for r in c..k {
            let v = a[r * k + c].abs();
            if v > bv {
                bv = v;
                best = r;
            }
        }
        piv[c] = best;
        if best != c {
            for j in c..k {
                a.swap(c * k + j, best * k + j);
            }
        }
        let p = a[c * k + c];
        if p == 0.0 || !p.is_finite() {
            return Err(ModalError::PadeFailed);
        }
        for r in (c + 1)..k {
            let f = a[r * k + c] / p;
            a[r * k + c] = f;
            if f != 0.0 {
                for j in (c + 1)..k {
                    a[r * k + j] -= f * a[c * k + j];
                }
            }
        }
    }
    Ok(())
}

// ---- K7: the assembled object ------------------------------------------------------

/// The decomposition of one system `x' = A x + B u`: `A = S D S^-1` with `D`
/// block diagonal, and `S^-1 B`. Built once per system by [`decompose`]
/// (allocates); evaluated per step by [`Modal::at`] (no allocation: buffers
/// from a [`Workspace`]).
#[derive(Debug, Clone)]
pub struct Modal {
    n: usize,
    m: usize,
    s: Vec<f64>,
    sinv: Vec<f64>,
    d: Vec<f64>,
    gb: Vec<f64>,
    blocks: Vec<Block>,
    cond: f64,
    cond_bal: f64,
    residual: f64,
    max_merged: usize,
    /// Per block: (largest real part, imaginary part, non-normality factor,
    /// |D_block|_1), for the certification's probe ladder.
    bdesc: Vec<(f64, f64, f64, f64)>,
    /// [`Modal::rounding_floor`].
    floor: f64,
    /// The split threshold the grouping ended at.
    split: f64,
}

/// Per-step buffers for one [`Modal`] (from [`Modal::workspace`]).
#[derive(Debug, Clone)]
pub struct Workspace {
    y0: Vec<f64>,
    g0: Vec<f64>,
    g1: Vec<f64>,
    y: Vec<f64>,
    /// Per state: the homogeneous and the forcing part of `y`, kept for
    /// the dense-output readers of a later slice.
    yh: Vec<f64>,
    yf: Vec<f64>,
    /// Per block, the squarings its Pade took in the last step (merged).
    bsq: Vec<i32>,
    pade: PadeWork,
}

/// The terms of `f(t) = c0 + q t + sum_j w_j e^(lambda_j t)` for one
/// observable row (complex pairs listed with both conjugates, so `f` is
/// real). Buffers sized by [`Modal::terms_buffer`]; `len` terms valid.
#[derive(Debug, Clone)]
pub struct Terms {
    pub c0: f64,
    pub q: f64,
    pub w: Vec<(f64, f64)>,
    pub lambda: Vec<(f64, f64)>,
    pub len: usize,
}

/// K7. Decomposes `A` (n x n) with input matrix `B` (n x m), both row-major,
/// `1 <= n <=` [`MAX_N`]: balance, Hessenberg, Schur, grouping at
/// [`SPLIT_NORM_MAX`], per-cluster column scaling of `S` (the norms of a
/// cluster's columns of `S` and rows of `S^-1` equalised, which leaves `D`
/// unchanged), then the a-posteriori budget: backward error at most
/// [`RESIDUAL_MAX`], cond(S) at most [`COND_MAX`], rounding floor at most
/// [`ROUND_MAX`] (regrouping at lower thresholds first, [`REGROUP_MAX`]),
/// then the certification ([`certify`]: shadow decompositions and probes,
/// only for a system with a merged block), else
/// [`ModalError::IllConditioned`]. All of it once per system. About 10 n^3
/// flops plus the grouping's Sylvester solves; the cost table is in
/// `modal_tests` (`modal_cost`).
///
/// Accuracy (R151): every step of a certified system is within 100 eps
/// times the answer's own sensitivity to its f64 data, and within 1e-12
/// relative wherever that is smaller (`random_set_against_oracle`, 10,000
/// matrices: worst 0.2 of that allowance; no decline on a matrix whose
/// sensitivity is below 45 eps, where the allowance is 1e-12 itself).
pub fn decompose(a: &[f64], b: &[f64], n: usize, m: usize) -> Result<Modal, ModalError> {
    decompose_with(a, b, n, m, SPLIT_NORM_MAX)
}

/// [`decompose`] with the split threshold given (the threshold sweep's
/// seam): the factorisation, then its certification ([`certify`]), both
/// once per system.
pub fn decompose_with(a: &[f64], b: &[f64], n: usize, m: usize, split_max: f64) -> Result<Modal, ModalError> {
    let base = decompose_core(a, b, n, m, split_max, true)?;
    certify(&base, a, b)?;
    Ok(base)
}

/// The factorisation alone, without the certification's shadow work (the
/// cost table's comparison; a caller must not step an uncertified system).
pub fn decompose_uncertified(a: &[f64], b: &[f64], n: usize, m: usize) -> Result<Modal, ModalError> {
    decompose_core(a, b, n, m, SPLIT_NORM_MAX, true)
}

/// Certification, paid once per decomposition ([`Modal::step`] and
/// [`Modal::at`] do none of it). A step's error has three parts: the
/// answer's own sensitivity to its f64 data (no f64 method avoids it; R151
/// allows 100 eps times it), the rounding through `S` and `S^-1` (bounded
/// for every step by [`Modal::rounding_floor`], held to [`ROUND_MAX`] by
/// [`decompose_core`]), and a merged block's Pade squarings, which are not
/// backward stable and are the one part a closed form does not cover. So a
/// system without a merged block is certified by its floor alone; one with
/// a merged block is probed: for each merged block, the steps at which its
/// Pade takes 0, 1, 2, ... squarings (up to [`CERT_SQ_MAX`], until the
/// block has decayed below 1e-17, or before the system grows past
/// `e^CERT_GROWTH_STOP`), along each of R161's 3 to 4 deterministic
/// directions ([`Modal::block_directions`]: the pseudo-random probe and
/// the block's leading singular directions), the Pade error must stay
/// within `max(CERT_TOL, CERT_SHARE - floor, CERT_KAPPA eps kappa)`, `kappa`
/// measured along that direction, relative to the step's
/// reference ([`Modal::probe_ref`]). The error is first estimated
/// ([`Modal::pade_estimate`]) by evaluations that share no arithmetic with
/// the step's after the block's input: [13/13] at one squaring more, [9/9]
/// (another approximant and scaling), and [13/13] carried as `E - I`; a
/// block that takes solves ([`SOLVE_DECAY`]) has no `E - I` form, and its
/// differences are also carried entrywise in absolute value
/// ([`Modal::solve_carry`], EX1H). EX1C's estimate, the first two's kind alone, cannot see an error common
/// to every scaling and every degree: squaring `E` rounds a near-zero
/// rate's departure from 1 away at the scaled level ([`PadeForm`]); EX1D
/// measured it 1.6e6 times under the true error, 3.6e-8 at 30 squarings on
/// an n = 16 Jordan member where the differences were 2.3e-14. The `E - I`
/// form loses the other thing (a decayed entry near 0), so the differences
/// together see both. An estimate over its limit is not taken as final: the
/// block is then evaluated in double-double ([`Modal::pade_measured`]) and
/// the measured error decides. Measured over the whole random set and the
/// n = 16 and 32 members, along four directions, every judged probe: the
/// estimate (or measurement) plus the floor was never under the oracle's
/// Pade error (`certify_estimate_bounds_the_pade_error`). The bound
/// `4 eps 2^s |S_i| |y_i|` that EX1 used is blind to a decayed block's
/// squarings adding errors rather than doubling them, and declined a
/// well-conditioned Jordan cluster of the random set at 6 squarings. `kappa` is the step's sensitivity
/// measured on the same probe through [`SCE_SAMPLES`] shadow decompositions
/// of `(A, B)` perturbed entrywise by +-[`SCE_DELTA`] relative (small-sample
/// statistical condition estimation, Kenney and Laub 1994). A probe that
/// fails, or whose answer has no digit left (`CERT_KAPPA eps kappa >= 1`),
/// ends that direction's ladder. A shadow that cannot be decomposed, an
/// evaluation that fails (the base's or a shadow's), or a probe over its
/// limit, declines the system ([`ModalError::IllConditioned`]: the
/// partition stays on the trapezoid).
fn certify(base: &Modal, a: &[f64], b: &[f64]) -> Result<(), ModalError> {
    #[cfg(test)]
    let r = certify_seen(base, a, b, None);
    #[cfg(not(test))]
    let r = certify_seen(base, a, b);
    r
}

/// What [`certify`] judged at one probe, for the tests (a test build only):
/// the block, the direction ([`Modal::block_directions`]), the ladder's
/// rung, the step, the estimate, its limit, the reference, the shadows'
/// sensitivity, the probe's inputs and result, and the workspace of the
/// evaluation steps make.
#[cfg(test)]
pub(crate) struct ProbeSeen<'a> {
    pub bi: usize,
    pub dir: usize,
    pub k: i32,
    pub h: f64,
    pub est: f64,
    pub allow: f64,
    pub reff: f64,
    pub kappa: f64,
    /// The estimate was over its limit and `est` is the measurement.
    pub measured: bool,
    pub x0: &'a [f64],
    pub u0: &'a [f64],
    pub du: &'a [f64],
    pub x: &'a [f64],
    pub ws: &'a Workspace,
}

/// One certification direction: `x0`, `u0`, and the ramp `dr` that a probe
/// of length `h` applies as `du = dr / h`.
type Dir = (Vec<f64>, Vec<f64>, Vec<f64>);

/// [`certify`], showing each judged probe to `seen` in a test build (the
/// tests' seam; a non-test build has neither the parameter nor
/// [`ProbeSeen`]).
fn certify_seen(base: &Modal, a: &[f64], b: &[f64], #[cfg(test)] mut seen: Option<&mut dyn FnMut(&ProbeSeen)>) -> Result<(), ModalError> {
    if base.max_merged == 0 {
        return Ok(());
    }
    let (n, m) = (base.n, base.m);
    // Deterministic factors r in [-1, 1) (splitmix64 from a fixed seed), so
    // a rebuilt decomposition is bit-identical: the snapshot never stores
    // one. Independent per entry: equal signs would only rescale A, which
    // moves no eigenvalue relative to itself and hides the sensitivity.
    let mut rnd = splitmix(0x2545_F491_4F6C_DD1D);
    let mut shadows = Vec::with_capacity(SCE_SAMPLES);
    for _ in 0..SCE_SAMPLES {
        let ap: Vec<f64> = a.iter().map(|v| v * (1.0 + SCE_DELTA * rnd())).collect();
        let bp: Vec<f64> = b.iter().map(|v| v * (1.0 + SCE_DELTA * rnd())).collect();
        match decompose_core(&ap, &bp, n, m, base.split, false) {
            Ok(md) => shadows.push(md),
            Err(_) => return Err(ModalError::IllConditioned),
        }
    }
    let mut du = vec![0.0; m];
    let mut ws = base.workspace();
    let mut sws: Vec<Workspace> = shadows.iter().map(|sh| sh.workspace()).collect();
    let mut x = vec![0.0; n];
    let mut x2 = vec![0.0; n];
    let mut x3 = vec![0.0; n];
    let mut xs = vec![0.0; n];
    let eps = f64::EPSILON;
    let tol = CERT_TOL.max(CERT_SHARE - base.floor);
    // The system's largest real part (a pair's is its diagonal).
    let sys_re = base.bdesc.iter().fold(f64::NEG_INFINITY, |w, d| w.max(d.0));
    for (bi, blk) in base.blocks.iter().enumerate() {
        if blk.kind != BlockKind::Merged {
            continue;
        }
        let (re_max, dn) = (base.bdesc[bi].0, base.bdesc[bi].3);
        if dn == 0.0 {
            continue;
        }
        let dirs = base.block_directions(bi)?;
        // EX1F: after R161's directions, the block's input-response columns
        // as one more ([`Modal::cancel_direction`]): along the input that
        // drives the block hardest (the forced singular direction's), the
        // constant and the ramp set to cancel in the block's response at
        // each probe's own step.
        let cancel_v = if m > 0 { dirs.get(1).map(|d| d.1.clone()) } else { None };
        let count = dirs.len() + usize::from(cancel_v.is_some());
        for di in 0..count {
            for k in 0..=CERT_SQ_MAX {
                let h = THETA13 * 2f64.powi(k) / dn;
                // The mirror of the decayed stop below: a step over which
                // the system grows past e^39 is not probed (the probe
                // evaluates every block; a growing mode elsewhere would
                // overflow the probe, not the merged block).
                if k > 0 && sys_re * h > CERT_GROWTH_STOP {
                    break;
                }
                let cd: Dir;
                let (x0, u0, dr) = match (dirs.get(di), cancel_v.as_deref()) {
                    (Some(d), _) => (&d.0, &d.1, &d.2),
                    (None, v) => {
                        // A failed evaluation declines, as the probes' own.
                        cd = base.cancel_direction(bi, v.unwrap_or(&[]), h).map_err(|_| ModalError::IllConditioned)?;
                        (&cd.0, &cd.1, &cd.2)
                    }
                };
                for j in 0..m {
                    du[j] = dr[j] / h;
                }
                let est = base.pade_estimate(x0, u0, &du, h, &mut ws, &mut x, &mut x2, &mut x3);
                #[cfg(test)]
                let est = if CERT_FAIL_AT.with(|c| c.get()) == k { Err(ModalError::NonFinite) } else { est };
                // An evaluation that fails declines (EX1E, the verifier's
                // finding 6): within the ladder's stops every evaluation is
                // of a step the engine may take, so a failure is not an
                // end of the probes but an answer the kernel cannot give.
                let pade = est.map_err(|_| ModalError::IllConditioned)?;
                let reff = base.probe_ref(x0, &x, &ws);
                let mut kappa = 0.0f64;
                for (sh, sw) in shadows.iter().zip(sws.iter_mut()) {
                    sh.at(x0, u0, &du, h, sw, &mut xs).map_err(|_| ModalError::IllConditioned)?;
                    let d = xs.iter().zip(&x).fold(0.0f64, |acc, (p, q)| acc.max((p - q).abs()));
                    kappa = kappa.max(d / (SCE_DELTA * reff));
                }
                if !kappa.is_finite() {
                    return Err(ModalError::IllConditioned);
                }
                #[cfg(test)]
                if CERT_TRACE.with(|c| c.get()) {
                    println!("    blk {bi} dir {di} k {k} h {h:.3e} s {:?} pade {pade:.3e} kappa {kappa:.3e} allow {:.3e} reff {reff:.3e} re_max*h {:.3e}", ws.bsq, tol.max(CERT_KAPPA * eps * kappa), re_max * h);
                }
                // The answer has no digit left to certify: R151 allows a
                // step there an error of the answer's own size.
                if !(CERT_KAPPA * eps * kappa < 1.0) {
                    break;
                }
                let allow = tol.max(CERT_KAPPA * eps * kappa);
                // Over its limit, the estimate (which over-estimates: its other
                // evaluations are less accurate than the step's) gives way to the
                // measurement.
                let (pade, measured) = if pade <= allow {
                    (pade, false)
                } else {
                    match base.pade_measured(h, u0, &du, &ws, reff) {
                        Ok(e) => (e, true),
                        Err(_) => return Err(ModalError::IllConditioned),
                    }
                };
                #[cfg(test)]
                if let Some(f) = seen.as_mut() {
                    f(&ProbeSeen { bi, dir: di, k, h, est: pade, allow, reff, kappa, measured, x0, u0, du: &du, x: &x, ws: &ws });
                }
                // Read only by the test hook above.
                #[cfg(not(test))]
                let _ = (di, measured);
                if !(pade <= allow) {
                    return Err(ModalError::IllConditioned);
                }
                if re_max * h < -39.0 {
                    break;
                }
            }
        }
    }
    Ok(())
}

/// Eigenvalues and eigenvectors of the symmetric `g` (order `k`,
/// row-major) by cyclic Jacobi rotations: `g` ends diagonal (the
/// eigenvalues on it, each within a few eps `|g|_F`), `v` holds the
/// eigenvectors in its columns (orthonormal to rounding). Converges
/// quadratically once the off-diagonal part is small; it stops when that
/// part is at most `k eps |g|_F`. [`ModalError::IllConditioned`] at
/// [`JACOBI_SWEEPS`] sweeps, [`ModalError::NonFinite`] on non-finite input.
/// O(k^3) per sweep; the certification's directions only.
fn sym_eig(g: &mut [f64], v: &mut [f64], k: usize) -> Result<(), ModalError> {
    for (p, e) in v[..k * k].iter_mut().enumerate() {
        *e = if p / k == p % k { 1.0 } else { 0.0 };
    }
    if !all_finite(&g[..k * k]) {
        return Err(ModalError::NonFinite);
    }
    let fro = g[..k * k].iter().map(|x| x * x).sum::<f64>().sqrt();
    for _ in 0..JACOBI_SWEEPS {
        let mut off = 0.0f64;
        for p in 0..k {
            for q in (p + 1)..k {
                off += g[p * k + q] * g[p * k + q];
            }
        }
        if !(off.sqrt() > k as f64 * f64::EPSILON * fro) {
            return Ok(());
        }
        for p in 0..k {
            for q in (p + 1)..k {
                let apq = g[p * k + q];
                if apq == 0.0 {
                    continue;
                }
                let theta = (g[q * k + q] - g[p * k + p]) / (2.0 * apq);
                let t = if theta.abs() > 1e150 { 0.5 / theta } else { theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt()) };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for r in 0..k {
                    let (gp, gq) = (g[r * k + p], g[r * k + q]);
                    g[r * k + p] = c * gp - s * gq;
                    g[r * k + q] = s * gp + c * gq;
                }
                for r in 0..k {
                    let (gp, gq) = (g[p * k + r], g[q * k + r]);
                    g[p * k + r] = c * gp - s * gq;
                    g[q * k + r] = s * gp + c * gq;
                }
                for r in 0..k {
                    let (vp, vq) = (v[r * k + p], v[r * k + q]);
                    v[r * k + p] = c * vp - s * vq;
                    v[r * k + q] = s * vp + c * vq;
                }
            }
        }
    }
    Err(ModalError::IllConditioned)
}

/// The eigenvectors of the symmetric `g` (order `k`) for its `count`
/// largest eigenvalues, largest first (ties by index), each as a vector
/// of `k` ([`sym_eig`]).
fn leading_eigvecs(mut g: Vec<f64>, k: usize, count: usize) -> Result<Vec<Vec<f64>>, ModalError> {
    let mut v = vec![0.0; k * k];
    sym_eig(&mut g, &mut v, k)?;
    let mut order: Vec<usize> = (0..k).collect();
    order.sort_by(|&p, &q| g[q * k + q].partial_cmp(&g[p * k + p]).unwrap_or(core::cmp::Ordering::Equal));
    Ok(order.iter().take(count).map(|&c| (0..k).map(|r| v[r * k + c]).collect()).collect())
}

/// A deterministic stream of factors in `[-1, 1)` (splitmix64 from `seed`),
/// so a rebuilt decomposition is bit-identical: the snapshot never stores
/// one.
fn splitmix(seed: u64) -> impl FnMut() -> f64 {
    let mut st = seed;
    move || {
        st = st.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = st;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        (z >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    }
}

/// The certification's probe direction `dir` (0 is the one [`certify`]
/// uses; the tests draw others): `x0`, `u0`, and the ramp `dr` that each
/// probe of length `h` applies as `du = dr / h`.
fn probe_vectors(n: usize, m: usize, dir: u64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut rnd = splitmix(0x5DEE_CE66_D1CE_4E5B ^ dir.wrapping_mul(0x9E37_79B9_7F4A_7C15));
    let x0 = (0..n).map(|_| rnd()).collect();
    let u0 = (0..m).map(|_| rnd()).collect();
    let dr = (0..m).map(|_| rnd()).collect();
    (x0, u0, dr)
}

/// One decomposition; `checks` applies the a-posteriori refusals (the
/// shadows skip them: they are perturbation probes, not steps).
fn decompose_core(a: &[f64], b: &[f64], n: usize, m: usize, split_max: f64, checks: bool) -> Result<Modal, ModalError> {
    if n == 0 || n > MAX_N || a.len() != n * n || b.len() != n * m {
        return Err(ModalError::Shape);
    }
    if !all_finite(a) || !all_finite(b) {
        return Err(ModalError::NonFinite);
    }
    let mut t = a.to_vec();
    let mut dbal = vec![1.0; n];
    #[cfg(test)]
    let skip = NO_BALANCE.with(|c| c.get());
    #[cfg(not(test))]
    let skip = false;
    if !skip {
        balance(&mut t, n, &mut dbal);
    }
    let abal = t.clone();
    let mut q = vec![0.0; n * n];
    hessenberg(&mut t, &mut q, n)?;
    schur(&mut t, &mut q, n)?;
    // The grouping fix (EX1B): a factorisation whose rounding floor or
    // cond(S) is over budget is grouped again at a tenth of the threshold,
    // which merges the close clusters whose decoupling made S ill
    // conditioned; a stiff separation has a small Sylvester solution and
    // stays split at any of these thresholds.
    let mut split = split_max;
    let mut best: Option<Modal> = None;
    let mut err = ModalError::IllConditioned;
    for _ in 0..=REGROUP_MAX {
        match group_and_finish(&abal, &dbal, &t, &q, b, n, m, split) {
            Ok(md) if !checks => return Ok(md),
            Ok(md) => {
                let done = md.within_budget() && md.floor <= REGROUP_FLOOR;
                if md.within_budget() && best.as_ref().map_or(true, |bm| md.floor < bm.floor) {
                    best = Some(md);
                }
                if done {
                    break;
                }
            }
            Err(e) => err = e,
        }
        if split <= SPLIT_NORM_MIN {
            break;
        }
        split = (split / 10.0).max(SPLIT_NORM_MIN);
    }
    best.ok_or(err)
}

/// Grouping at `split_max` and everything after it, from the Schur form
/// `(t0, q)` of the balanced `abal = Dbal^-1 A Dbal`.
#[allow(clippy::too_many_arguments)]
fn group_and_finish(abal: &[f64], dbal: &[f64], t0: &[f64], q: &[f64], b: &[f64], n: usize, m: usize, split_max: f64) -> Result<Modal, ModalError> {
    let mut t = t0.to_vec();
    let mut s = q.to_vec();
    let mut sinv = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            sinv[ix(n, i, j)] = s[ix(n, j, i)];
        }
    }
    let blocks = group(&mut t, &mut s, &mut sinv, n, split_max)?;
    // Per-cluster scaling: S[:, cl] *= f, Sinv[cl, :] /= f, f = sqrt(|Sinv_cl| / |S_cl|).
    for blk in &blocks {
        let (mut ns, mut ni) = (0.0f64, 0.0f64);
        for c in blk.start..blk.start + blk.size {
            for r in 0..n {
                ns = ns.max(s[ix(n, r, c)].abs());
                ni = ni.max(sinv[ix(n, c, r)].abs());
            }
        }
        if ns > 0.0 && ni > 0.0 {
            let f = (ni / ns).sqrt();
            let f = 2.0f64.powi(f.log2().round() as i32);
            for c in blk.start..blk.start + blk.size {
                for r in 0..n {
                    s[ix(n, r, c)] *= f;
                    sinv[ix(n, c, r)] /= f;
                }
            }
        }
    }
    // A-posteriori checks in balanced coordinates.
    let anorm = norm1(abal, n).max(f64::MIN_POSITIVE);
    let mut sd = vec![0.0; n * n];
    for i in 0..n {
        for blk in &blocks {
            for j in blk.start..blk.start + blk.size {
                let mut acc = 0.0;
                for k in blk.start..blk.start + blk.size {
                    acc += s[ix(n, i, k)] * t[ix(n, k, j)];
                }
                sd[ix(n, i, j)] = acc;
            }
        }
    }
    let mut resid = 0.0f64;
    for j in 0..n {
        let mut col = 0.0;
        for i in 0..n {
            let mut acc = -abal[ix(n, i, j)];
            for k in 0..n {
                acc += sd[ix(n, i, k)] * sinv[ix(n, k, j)];
            }
            col += acc.abs();
        }
        resid = resid.max(col);
    }
    let residual = resid / anorm;
    let cond_bal = norm1(&s, n) * norm1(&sinv, n);
    if !residual.is_finite() || !cond_bal.is_finite() {
        return Err(ModalError::NonFinite);
    }
    // Per block: (largest real part, frequency (NaN for a merged block),
    // |N|_1 the off-diagonal part's norm, |D_blk|_1).
    let bdesc: Vec<(f64, f64, f64, f64)> = blocks
        .iter()
        .map(|bl| {
            let j = bl.start;
            let mut dn = 0.0f64;
            let mut nn = 0.0f64;
            for c in j..j + bl.size {
                dn = dn.max((j..j + bl.size).map(|r| t[ix(n, r, c)].abs()).sum::<f64>());
                nn = nn.max((j..j + bl.size).filter(|&r| r != c).map(|r| t[ix(n, r, c)].abs()).sum::<f64>());
            }
            match bl.kind {
                BlockKind::Real => (t[ix(n, j, j)], 0.0, 0.0, dn),
                BlockKind::Pair => (t[ix(n, j, j)], pair_omega(t[ix(n, j, j + 1)], t[ix(n, j + 1, j)]), nn, dn),
                BlockKind::Merged => ((j..j + bl.size).map(|i| t[ix(n, i, i)]).fold(f64::NEG_INFINITY, f64::max), f64::NAN, nn, dn),
            }
        })
        .collect();
    // The rounding floor in balanced coordinates, where the states are on
    // comparable scales: in the caller's, a worst `x0` of 1 A beside volts
    // that are 1e-3 of it is not a state any circuit reaches, and the
    // three-scales board (a 5 ns pole beside a 48 ms decay) read 5.1e-12 there.
    let floor = rounding_floor(&s, &sinv, n);
    if !floor.is_finite() {
        return Err(ModalError::NonFinite);
    }
    // Back to the caller's coordinates: S <- Dbal S, Sinv <- Sinv Dbal^-1 (exact).
    for i in 0..n {
        for j in 0..n {
            s[ix(n, i, j)] *= dbal[i];
            sinv[ix(n, i, j)] /= dbal[j];
        }
    }
    let cond = norm1(&s, n) * norm1(&sinv, n);
    let mut gb = vec![0.0; n * m];
    for i in 0..n {
        for j in 0..m {
            let mut acc = 0.0;
            for k in 0..n {
                acc += sinv[ix(n, i, k)] * b[k * m + j];
            }
            gb[i * m + j] = acc;
        }
    }
    let max_merged = blocks.iter().filter(|b| b.kind == BlockKind::Merged).map(|b| b.size).max().unwrap_or(0);
    Ok(Modal { n, m, s, sinv, d: t, gb, blocks, cond, cond_bal, residual, max_merged, bdesc, floor, split: split_max })
}

/// `2 eps | |S| |S^-1| |_inf`: the worst relative error, over every `x0`, of
/// the round trip `S (S^-1 x0)` that every step makes (row `r` of `|S|
/// |S^-1|` sums `|S_rk| |S^-1_kj|`), taken in balanced coordinates. O(n^2).
fn rounding_floor(s: &[f64], sinv: &[f64], n: usize) -> f64 {
    let mut rs = [0.0f64; MAX_N];
    for k in 0..n {
        rs[k] = (0..n).map(|j| sinv[ix(n, k, j)].abs()).sum::<f64>();
    }
    let mut w = 0.0f64;
    for r in 0..n {
        w = w.max((0..n).map(|k| s[ix(n, r, k)].abs() * rs[k]).sum::<f64>());
    }
    2.0 * f64::EPSILON * w
}

impl Modal {
    /// The a-posteriori budget [`decompose`] holds a factorisation to.
    fn within_budget(&self) -> bool {
        self.residual <= RESIDUAL_MAX && self.cond_bal <= COND_MAX && self.floor <= ROUND_MAX
    }
    /// The rounding floor `2 eps | |S| |S^-1| |_inf` (see [`ROUND_MAX`]).
    pub fn rounding_floor(&self) -> f64 {
        self.floor
    }
    /// The split threshold the grouping ended at (see [`REGROUP_MAX`]).
    pub fn split_threshold(&self) -> f64 {
        self.split
    }
    pub fn n(&self) -> usize {
        self.n
    }
    pub fn m(&self) -> usize {
        self.m
    }
    pub fn blocks(&self) -> &[Block] {
        &self.blocks
    }
    /// cond(S) = |S|_1 |S^-1|_1 in the caller's coordinates.
    pub fn cond(&self) -> f64 {
        self.cond
    }
    /// cond(S) in balanced coordinates, the one [`COND_MAX`] bounds.
    pub fn cond_balanced(&self) -> f64 {
        self.cond_bal
    }
    /// The measured backward error `|S D S^-1 - A|_1 / |A|_1` (balanced).
    pub fn residual(&self) -> f64 {
        self.residual
    }
    pub fn s(&self) -> &[f64] {
        &self.s
    }
    pub fn s_inv(&self) -> &[f64] {
        &self.sinv
    }
    pub fn d(&self) -> &[f64] {
        &self.d
    }

    /// The per-step buffers for this system (allocates; once per system).
    pub fn workspace(&self) -> Workspace {
        let n = self.n;
        Workspace { y0: vec![0.0; n], g0: vec![0.0; n], g1: vec![0.0; n], y: vec![0.0; n], yh: vec![0.0; n], yf: vec![0.0; n], bsq: vec![0; self.blocks.len()], pade: PadeWork::new(if self.max_merged > 0 { self.max_merged + 2 } else { 0 }) }
    }

    /// Buffers for [`Modal::terms`] (allocates; once per system).
    pub fn terms_buffer(&self) -> Terms {
        Terms { c0: 0.0, q: 0.0, w: vec![(0.0, 0.0); self.n], lambda: vec![(0.0, 0.0); self.n], len: 0 }
    }

    /// One step: `x(h)` from `x(0) = x0` under `u(t) = u0 + du t`. The same
    /// as [`Modal::at`] at `tau = h`.
    pub fn step(&self, x0: &[f64], u0: &[f64], du: &[f64], h: f64, ws: &mut Workspace, out: &mut [f64]) -> Result<(), ModalError> {
        self.at(x0, u0, du, h, ws, out)
    }

    /// `x(tau)` for any `tau >= 0` (dense output from the step's start): per
    /// block `y_j(tau) = e^(D_j tau) y_j0 + tau phi_1 g0_j + tau^2 phi_2 g1_j`
    /// with `y0 = S^-1 x0`, `g0 = S^-1 B u0`, `g1 = S^-1 B du`, then `x = S y`:
    /// 2 n^2 + 2 n m multiply-adds plus one closed form per order 1 or 2
    /// block and one Pade 13 of order `size + 2` per merged block. No
    /// allocation. Fails on a negative or non-finite `tau`, wrong lengths
    /// ([`ModalError::Shape`]), a non-finite result (a growing mode that
    /// overflows), or a Pade failure.
    pub fn at(&self, x0: &[f64], u0: &[f64], du: &[f64], tau: f64, ws: &mut Workspace, out: &mut [f64]) -> Result<(), ModalError> {
        self.at_scaled(x0, u0, du, tau, ws, out, 0, PadeForm::P13)
    }

    /// [`Modal::at`] with `extra` more squarings in every merged block's
    /// Pade, in the given [`PadeForm`] (the certification's other
    /// evaluations, [`Modal::pade_estimate`]).
    #[allow(clippy::too_many_arguments)]
    fn at_scaled(&self, x0: &[f64], u0: &[f64], du: &[f64], tau: f64, ws: &mut Workspace, out: &mut [f64], extra: i32, form: PadeForm) -> Result<(), ModalError> {
        let (n, m) = (self.n, self.m);
        if x0.len() != n || u0.len() != m || du.len() != m || out.len() != n || ws.y0.len() != n {
            return Err(ModalError::Shape);
        }
        if !(tau >= 0.0) || !tau.is_finite() {
            return Err(ModalError::Shape);
        }
        for i in 0..n {
            let mut a = 0.0;
            for k in 0..n {
                a += self.sinv[ix(n, i, k)] * x0[k];
            }
            ws.y0[i] = a;
            let (mut b0, mut b1) = (0.0, 0.0);
            for j in 0..m {
                b0 += self.gb[i * m + j] * u0[j];
                b1 += self.gb[i * m + j] * du[j];
            }
            ws.g0[i] = b0;
            ws.g1[i] = b1;
        }
        let h = tau;
        let h2 = h * h;
        // `S^-1 B u(h)` for row `r`, the end input formed with one rounding
        // (EX1F, the decayed blocks' solves; [`SOLVE_DECAY`]).
        let gend = |r: usize| {
            let mut a = 0.0;
            for q in 0..m {
                a += self.gb[r * m + q] * du[q].mul_add(h, u0[q]);
            }
            a
        };
        for (bi, blk) in self.blocks.iter().enumerate() {
            let j = blk.start;
            match blk.kind {
                BlockKind::Real => {
                    let z = self.d[ix(n, j, j)] * h;
                    let [e, p1, p2] = real_exp_phi(z);
                    ws.yh[j] = e * ws.y0[j];
                    // Decayed (EX1F): `(e c0 - h B~ u(h) + (e - 1) c1 / z) / z`,
                    // the closed form's own identity, so a constant and a ramp
                    // that cancel are never formed apart (as a merged block's
                    // solves, [`SOLVE_DECAY`]).
                    ws.yf[j] = if decayed(z) { (e * (h * ws.g0[j]) - h * gend(j) + (e - 1.0) * (h2 * ws.g1[j]) / z) / z } else { h * p1 * ws.g0[j] + h2 * p2 * ws.g1[j] };
                    ws.y[j] = ws.yh[j] + ws.yf[j];
                }
                BlockKind::Pair => {
                    let a = self.d[ix(n, j, j)];
                    let b = self.d[ix(n, j, j + 1)];
                    let c = self.d[ix(n, j + 1, j)];
                    let w = pair_omega(b, c);
                    let f = pair_exp_phi(a * h, w * h);
                    // f(M h) v = Re f v + (Im f / si) h [b v1; c v0]
                    let apply = |re: f64, ri: f64, v0: f64, v1: f64| (re * v0 + ri * h * b * v1, re * v1 + ri * h * c * v0);
                    let (e0, e1) = apply(f[0].0, f[0].1, ws.y0[j], ws.y0[j + 1]);
                    ws.yh[j] = e0;
                    ws.yh[j + 1] = e1;
                    if decayed(a * h) {
                        // Decayed (EX1F): `M^-1 [E c0 - h B~ u(h) + M^-1 (E - I)
                        // c1]` with `M = D h` and `M^-1 = [a -b; -c a] / ((a^2 -
                        // b c) h)` (`b c < 0`: no cancellation), as the merged
                        // block's solves ([`SOLVE_DECAY`]).
                        let det = (a * a - b * c) * h;
                        let minv = |v0: f64, v1: f64| ((a * v0 - b * v1) / det, (a * v1 - c * v0) / det);
                        let (c10, c11) = (h2 * ws.g1[j], h2 * ws.g1[j + 1]);
                        let (t0, t1) = apply(f[0].0, f[0].1, c10, c11);
                        let (z0, z1) = minv(t0 - c10, t1 - c11);
                        let (s0, s1) = apply(f[0].0, f[0].1, h * ws.g0[j], h * ws.g0[j + 1]);
                        let (f0, f1) = minv(s0 - h * gend(j) + z0, s1 - h * gend(j + 1) + z1);
                        ws.yf[j] = f0;
                        ws.yf[j + 1] = f1;
                    } else {
                        let (p0, p1) = apply(f[1].0, f[1].1, ws.g0[j], ws.g0[j + 1]);
                        let (q0, q1) = apply(f[2].0, f[2].1, ws.g1[j], ws.g1[j + 1]);
                        ws.yf[j] = h * p0 + h2 * q0;
                        ws.yf[j + 1] = h * p1 + h2 * q1;
                    }
                    ws.y[j] = e0 + ws.yf[j];
                    ws.y[j + 1] = e1 + ws.yf[j + 1];
                }
                BlockKind::Merged if self.solves(bi, h) => {
                    // EX1F: the decayed block's input response by two solves
                    // with `M = D h` (see [`SOLVE_DECAY`]); `E - I` as the
                    // `P13Minus1` form carries it gives nothing here (no rate
                    // near zero) and would round a decayed `E` to eps, so that
                    // form evaluates as `P13`.
                    let k = blk.size;
                    if ws.pade.k < k {
                        return Err(ModalError::Shape);
                    }
                    self.block_m(j, k, h, &mut ws.pade.m);
                    pade_exp_scaled(&mut ws.pade, k, extra, if form == PadeForm::P13Minus1 { PadeForm::P13 } else { form })?;
                    if let Some(slot) = ws.bsq.get_mut(bi) {
                        *slot = ws.pade.last_s;
                    }
                    let PadeWork { m: e, t: va, u: vb, .. } = &mut ws.pade;
                    // va = (E - I) c1, then M^-1 of it.
                    for r in 0..k {
                        let mut acc = 0.0;
                        for c in 0..k {
                            acc += e[r * k + c] * (h2 * ws.g1[j + c]);
                        }
                        va[r] = acc - h2 * ws.g1[j + r];
                    }
                    self.block_solve(j, k, h, &mut va[..k])?;
                    // vb = E c0 - h B~ u(h) + va, then M^-1 of it.
                    for r in 0..k {
                        let mut acc = 0.0;
                        for c in 0..k {
                            acc += e[r * k + c] * (h * ws.g0[j + c]);
                        }
                        vb[r] = acc - h * gend(j + r) + va[r];
                    }
                    self.block_solve(j, k, h, &mut vb[..k])?;
                    for r in 0..k {
                        let mut acc = 0.0;
                        for c in 0..k {
                            acc += e[r * k + c] * ws.y0[j + c];
                        }
                        ws.yh[j + r] = acc;
                        ws.yf[j + r] = vb[r];
                        ws.y[j + r] = acc + vb[r];
                    }
                }
                BlockKind::Merged => {
                    let k = blk.size;
                    let dim = k + 2;
                    if ws.pade.k < dim {
                        return Err(ModalError::Shape);
                    }
                    self.merged_input(j, k, h, &ws.g0, &ws.g1, &mut ws.pade.m);
                    pade_exp_scaled(&mut ws.pade, dim, extra, form)?;
                    if let Some(slot) = ws.bsq.get_mut(bi) {
                        *slot = ws.pade.last_s;
                    }
                    let pm = &ws.pade.m;
                    for r in 0..k {
                        let mut acc = 0.0;
                        for c in 0..k {
                            acc += pm[r * dim + c] * ws.y0[j + c];
                        }
                        ws.yh[j + r] = acc;
                        ws.yf[j + r] = pm[r * dim + k];
                        ws.y[j + r] = acc + pm[r * dim + k];
                    }
                }
            }
        }
        for i in 0..n {
            let mut a = 0.0;
            for k in 0..n {
                a += self.s[ix(n, i, k)] * ws.y[k];
            }
            out[i] = a;
        }
        if all_finite(out) {
            Ok(())
        } else {
            Err(ModalError::NonFinite)
        }
    }

    /// Whether merged block `bi` takes its input response by solves over a
    /// step `h` ([`SOLVE_DECAY`]).
    fn solves(&self, bi: usize, h: f64) -> bool {
        decayed(self.bdesc[bi].0 * h)
    }

    /// `M = D h` of the merged block at `j` (order `k`), row-major of
    /// stride `k`, into `pm` (the same `D h` the augmented form uses).
    fn block_m(&self, j: usize, k: usize, h: f64, pm: &mut [f64]) {
        let n = self.n;
        for r in 0..k {
            for c in 0..k {
                pm[r * k + c] = self.d[ix(n, j + r, j + c)] * h;
            }
        }
    }

    /// `r <- M^-1 r` for the merged block at `j` (order `k`), `M = D h`,
    /// by back substitution on the quasi-triangular block (its 2 x 2 bumps
    /// by Gaussian elimination with partial pivoting): backward stable,
    /// each computed `z` the exact solution of `(M + dM) z = r` with `|dM|
    /// <= (k + 2) eps |M|`. No allocation. [`ModalError::PadeFailed`] on a
    /// zero pivot (a singular `M`, which a decayed block cannot have),
    /// [`ModalError::NonFinite`] on a non-finite result.
    fn block_solve(&self, j: usize, k: usize, h: f64, r: &mut [f64]) -> Result<(), ModalError> {
        let n = self.n;
        let mm = |a: usize, b: usize| self.d[ix(n, j + a, j + b)] * h;
        let mut i = k;
        while i > 0 {
            if i >= 2 && self.d[ix(n, j + i - 1, j + i - 2)] != 0.0 {
                let (p, q) = (i - 2, i - 1);
                let (mut r1, mut r2) = (r[p], r[q]);
                for c in i..k {
                    r1 -= mm(p, c) * r[c];
                    r2 -= mm(q, c) * r[c];
                }
                let (mut a, mut b, mut c, mut d) = (mm(p, p), mm(p, q), mm(q, p), mm(q, q));
                if c.abs() > a.abs() {
                    core::mem::swap(&mut a, &mut c);
                    core::mem::swap(&mut b, &mut d);
                    core::mem::swap(&mut r1, &mut r2);
                }
                if a == 0.0 {
                    return Err(ModalError::PadeFailed);
                }
                let l = c / a;
                let d2 = d - l * b;
                if d2 == 0.0 {
                    return Err(ModalError::PadeFailed);
                }
                let z2 = (r2 - l * r1) / d2;
                r[p] = (r1 - b * z2) / a;
                r[q] = z2;
                i -= 2;
            } else {
                let p = i - 1;
                let mut acc = r[p];
                for c in i..k {
                    acc -= mm(p, c) * r[c];
                }
                let a = mm(p, p);
                if a == 0.0 {
                    return Err(ModalError::PadeFailed);
                }
                r[p] = acc / a;
                i -= 1;
            }
        }
        if all_finite(&r[..k]) {
            Ok(())
        } else {
            Err(ModalError::NonFinite)
        }
    }

    /// The augmented matrix whose exponential gives the merged block at
    /// `j` (order `k`) over `h`: `[[D h, h g0, 0], [0, 0, 1], [0, 0, 0]]`
    /// with the ramp's `h^2 g1` in the last column, row-major of order
    /// `k + 2`, into `pm`.
    fn merged_input(&self, j: usize, k: usize, h: f64, g0: &[f64], g1: &[f64], pm: &mut [f64]) {
        let n = self.n;
        let dim = k + 2;
        let h2 = h * h;
        for v in pm[..dim * dim].iter_mut() {
            *v = 0.0;
        }
        for r in 0..k {
            for c in 0..k {
                pm[r * dim + c] = self.d[ix(n, j + r, j + c)] * h;
            }
            pm[r * dim + k] = h * g0[j + r];
            pm[r * dim + k + 1] = h2 * g1[j + r];
        }
        pm[(k + 1) * dim + k] = 1.0;
    }

    /// The certification's estimate of the merged blocks' Pade error in
    /// `x(h)`, with the evaluation every step makes ([13/13] squared `s`
    /// times) left in `x` and in `ws`: [`CERT_EST_FACTOR`] times
    /// `max(d1, d9) + d_id`, where `d1`, `d9` and `d_id` are its differences
    /// from [13/13] at `s + 1`, from [9/9] at `theta_9`, and from [13/13]
    /// at `s` carried as `E - I` ([`PadeForm`]), plus the products'
    /// rounding ([`Modal::eval_rounding`]) and, for a block that takes
    /// solves, the same differences of `E` carried entrywise in absolute
    /// value ([`Modal::solve_carry`]), relative to the step's reference
    /// ([`Modal::probe_ref`]). See [`certify`] for the argument and the
    /// measurement.
    #[allow(clippy::too_many_arguments)]
    fn pade_estimate(&self, x0: &[f64], u0: &[f64], du: &[f64], h: f64, ws: &mut Workspace, x: &mut [f64], x2: &mut [f64], x3: &mut [f64]) -> Result<f64, ModalError> {
        let dist = |p: &[f64], q: &[f64]| p.iter().zip(q.iter()).fold(0.0f64, |acc, (a, b)| acc.max((a - b).abs()));
        self.at_scaled(x0, u0, du, h, ws, x2, 1, PadeForm::P13)?;
        self.at_scaled(x0, u0, du, h, ws, x3, 0, PadeForm::P9)?;
        self.at_scaled(x0, u0, du, h, ws, x, 0, PadeForm::P13)?;
        let d_scale = dist(x2, x).max(dist(x3, x));
        #[cfg(test)]
        let (d1, d9) = (dist(x2, x), dist(x3, x));
        self.at_scaled(x0, u0, du, h, ws, x2, 0, PadeForm::P13Minus1)?;
        let d_id = dist(x2, x);
        // Again, so `ws` holds the step's own evaluation (the tests read it).
        self.at_scaled(x0, u0, du, h, ws, x, 0, PadeForm::P13)?;
        let reff = self.probe_ref(x0, x, ws);
        let rnd = self.eval_rounding(h, u0, du, ws)?;
        let carry = self.solve_carry(h, ws)?;
        #[cfg(test)]
        LAST_DIFFS.with(|c| c.set([d1 / reff, d9 / reff, d_id / reff]));
        Ok((CERT_EST_FACTOR * (d_scale + d_id) + rnd + carry) / reff + EST_RESOLUTION)
    }

    /// R161's certification directions for merged block `bi`: the probe
    /// ([`probe_vectors`], direction 0, the same for every block), then `x0`
    /// along each of the `min(CERT_SINGULAR_DIRS, size)` leading right
    /// singular vectors of the block's spectral projector `P = S[:, blk]
    /// S^-1[blk, :]`: the unit initial states that put the most of
    /// themselves into the block and get the most back out of it, so the
    /// block's Pade error reaches `x` there most amplified. The first of
    /// them is forced along the leading right singular vector of `P B`
    /// (`u0 = dr`), the input that drives the block hardest; the others are
    /// unforced. Deterministic (fixed-order Jacobi on `P^T P`, [`sym_eig`]),
    /// so a rebuilt decomposition certifies bit for bit. O(n^3), once per
    /// block per system.
    fn block_directions(&self, bi: usize) -> Result<Vec<Dir>, ModalError> {
        let (n, m) = (self.n, self.m);
        let (j, k) = (self.blocks[bi].start, self.blocks[bi].size);
        let mut out = vec![probe_vectors(n, m, 0)];
        let mut p = vec![0.0; n * n];
        for i in 0..n {
            for l in 0..n {
                p[ix(n, i, l)] = (0..k).map(|r| self.s[ix(n, i, j + r)] * self.sinv[ix(n, j + r, l)]).sum::<f64>();
            }
        }
        let mut g = vec![0.0; n * n];
        for a in 0..n {
            for b in 0..n {
                g[ix(n, a, b)] = (0..n).map(|i| p[ix(n, i, a)] * p[ix(n, i, b)]).sum::<f64>();
            }
        }
        let xs = leading_eigvecs(g, n, CERT_SINGULAR_DIRS.min(k))?;
        let ub = if m > 0 {
            // P B = S[:, blk] (S^-1 B)[blk, :].
            let pb: Vec<f64> = (0..n * m).map(|q| (0..k).map(|r| self.s[ix(n, q / m, j + r)] * self.gb[(j + r) * m + q % m]).sum::<f64>()).collect();
            let gm: Vec<f64> = (0..m * m).map(|q| (0..n).map(|i| pb[i * m + q / m] * pb[i * m + q % m]).sum::<f64>()).collect();
            leading_eigvecs(gm, m, 1)?.remove(0)
        } else {
            Vec::new()
        };
        for (di, x0) in xs.into_iter().enumerate() {
            let u = if di == 0 { ub.clone() } else { vec![0.0; m] };
            out.push((x0, u.clone(), u));
        }
        Ok(out)
    }

    /// The certification's input-response direction for merged block `bi`
    /// at a probe of length `h` (EX1F): `x0 = 0`, `u0 = v`, `dr = -alpha v`,
    /// where `alpha` makes the block's response to the constant input `v`
    /// (`r0 = S_blk yf` for `u0 = v, du = 0`) and to the ramp `v t / h`
    /// (`r1`, `u0 = 0`) cancel as far as they can: `alpha = <r0, r1> / <r1,
    /// r1>` (least squares; 0 when `r1 = 0`). That is the combination that
    /// set random case 2360's hole (the two responses 36 times their sum),
    /// and its worst along `v`: in a decayed block the two tend to
    /// `-M^-1 c` and `-M^-1 c - M^-2 c / h` for the same `c = h S^-1 B v`,
    /// so `alpha` tends to 1 (an input that ramps back to 0 at the step's
    /// end), where they cancel by up to `|lambda h|`, and since a perturbed
    /// `A` or `B` moves the two alike, no sensitivity excuses an error that
    /// cancellation exposes. Inputs cancelling across different input
    /// vectors are not this kind: a perturbed `B` moves them apart, so the
    /// measured sensitivity carries them. Two evaluations; allocates
    /// (certification only).
    fn cancel_direction(&self, bi: usize, v: &[f64], h: f64) -> Result<Dir, ModalError> {
        let (n, m) = (self.n, self.m);
        let (j, k) = (self.blocks[bi].start, self.blocks[bi].size);
        let mut ws = self.workspace();
        let mut x = vec![0.0; n];
        let z0 = vec![0.0; n];
        let zu = vec![0.0; m];
        let vh: Vec<f64> = v.iter().map(|q| q / h).collect();
        let mut resp = |u0: &[f64], du: &[f64]| -> Result<Vec<f64>, ModalError> {
            self.at(&z0, u0, du, h, &mut ws, &mut x)?;
            Ok((0..n).map(|i| (0..k).map(|r| self.s[ix(n, i, j + r)] * ws.yf[j + r]).sum::<f64>()).collect())
        };
        let r0 = resp(v, &zu)?;
        let r1 = resp(&zu, &vh)?;
        let (p, q) = r0.iter().zip(&r1).fold((0.0f64, 0.0f64), |(p, q), (a, b)| (p + a * b, q + b * b));
        let alpha = if q > 0.0 { p / q } else { 0.0 };
        if !alpha.is_finite() {
            return Err(ModalError::NonFinite);
        }
        Ok((vec![0.0; n], v.to_vec(), v.iter().map(|q| -alpha * q).collect()))
    }

    /// A bound on the rounding of the merged blocks' products from their
    /// exponentials in the evaluation `ws` holds (after [`Modal::at`] over
    /// `h` from `u0`, `du`), carried to `x` through `|S|`. An augmented
    /// block: per row `(k + 2) eps (sum |E_rc y0_c| + |f_r|)`. Every
    /// evaluation [`Modal::pade_estimate`] compares rounds its own products
    /// alike, so their differences do not see this part of a step's error
    /// after the exponential; the estimate adds it. A block that takes
    /// solves ([`SOLVE_DECAY`]): the same for `E y0`, plus the first-order
    /// bound of the solves' path with `|M^-1|` formed here: forming `c0 = h
    /// g0`, `c1 = h^2 g1` and `B~ u(h)` (`(m + 2) eps` of `|S^-1 B| |u|`),
    /// `(E - I) c1` and `E c0`, each solve's backward error `(k + 2) eps
    /// |M|` (`|dz| <= |M^-1| (|dr| + (k + 2) eps |M| |z|)`), so the
    /// estimate holds the solves' rounding too, which no difference between
    /// exponentials can see. Recomputes each block's exponential and
    /// solves (bit for bit the step's).
    fn eval_rounding(&self, h: f64, u0: &[f64], du: &[f64], ws: &mut Workspace) -> Result<f64, ModalError> {
        let (n, m) = (self.n, self.m);
        let eps = f64::EPSILON;
        let mut acc = [0.0f64; MAX_N];
        for (bi, blk) in self.blocks.iter().enumerate() {
            if blk.kind != BlockKind::Merged {
                continue;
            }
            let (j, k) = (blk.start, blk.size);
            let g = (k + 2) as f64 * eps;
            let mut e_row = vec![0.0f64; k];
            if self.solves(bi, h) {
                let h2 = h * h;
                self.block_m(j, k, h, &mut ws.pade.m);
                let ma: Vec<f64> = ws.pade.m[..k * k].iter().map(|v| v.abs()).collect();
                pade_exp_scaled(&mut ws.pade, k, 0, PadeForm::P13)?;
                let ea: Vec<f64> = ws.pade.m[..k * k].iter().map(|v| v.abs()).collect();
                let e: Vec<f64> = ws.pade.m[..k * k].to_vec();
                let ninv = self.block_minv_abs(j, k, h)?;
                let mv = |a: &[f64], v: &[f64]| -> Vec<f64> { (0..k).map(|r| (0..k).map(|c| a[r * k + c] * v[c]).sum::<f64>()).collect() };
                let gm = (m + 2) as f64 * eps;
                let gbu = |r: usize, u: &[f64]| (0..m).map(|q| (self.gb[(j + r) * m + q] * u[q]).abs()).sum::<f64>();
                let uend: Vec<f64> = (0..m).map(|q| du[q].mul_add(h, u0[q])).collect();
                let c0: Vec<f64> = (0..k).map(|r| h * ws.g0[j + r]).collect();
                let c1: Vec<f64> = (0..k).map(|r| h2 * ws.g1[j + r]).collect();
                let dc0: Vec<f64> = (0..k).map(|r| eps * c0[r].abs() + h * gm * gbu(r, u0)).collect();
                let dc1: Vec<f64> = (0..k).map(|r| 2.0 * eps * c1[r].abs() + h2 * gm * gbu(r, du)).collect();
                let c0a: Vec<f64> = c0.iter().map(|v| v.abs()).collect();
                let c1a: Vec<f64> = c1.iter().map(|v| v.abs()).collect();
                // z = M^-1 (E - I) c1, as the step.
                let mut z: Vec<f64> = mv(&e, &c1).iter().zip(&c1).map(|(a, b)| a - b).collect();
                self.block_solve(j, k, h, &mut z)?;
                let ec1 = mv(&ea, &c1a);
                let edc1 = mv(&ea, &dc1);
                let r1: Vec<f64> = (0..k).map(|r| g * (ec1[r] + c1a[r]) + edc1[r] + dc1[r]).collect();
                let za: Vec<f64> = z.iter().map(|v| v.abs()).collect();
                let mz = mv(&ma, &za);
                let dz = mv(&ninv, &(0..k).map(|r| r1[r] + g * mz[r]).collect::<Vec<_>>());
                // f = M^-1 (E c0 - h B~ u(h) + z), as the step.
                let mut f = mv(&e, &c0);
                for r in 0..k {
                    let ge: f64 = (0..m).map(|q| self.gb[(j + r) * m + q] * uend[q]).sum();
                    f[r] = f[r] - h * ge + z[r];
                }
                self.block_solve(j, k, h, &mut f)?;
                let ec0 = mv(&ea, &c0a);
                let edc0 = mv(&ea, &dc0);
                let r2: Vec<f64> = (0..k).map(|r| g * (ec0[r] + h * gbu(r, &uend) + za[r]) + edc0[r] + h * gm * gbu(r, &uend) + dz[r]).collect();
                let fa: Vec<f64> = f.iter().map(|v| v.abs()).collect();
                let mf = mv(&ma, &fa);
                let df = mv(&ninv, &(0..k).map(|r| r2[r] + g * mf[r]).collect::<Vec<_>>());
                let y0a: Vec<f64> = (0..k).map(|r| ws.y0[j + r].abs()).collect();
                let ey = mv(&ea, &y0a);
                for r in 0..k {
                    e_row[r] = df[r] + g * (ey[r] + fa[r]);
                }
            } else {
                let dim = k + 2;
                self.merged_input(j, k, h, &ws.g0, &ws.g1, &mut ws.pade.m);
                pade_exp_scaled(&mut ws.pade, dim, 0, PadeForm::P13)?;
                let pm = &ws.pade.m;
                for r in 0..k {
                    let mut a = pm[r * dim + k].abs();
                    for c in 0..k {
                        a += (pm[r * dim + c] * ws.y0[j + c]).abs();
                    }
                    e_row[r] = g * a;
                }
            }
            for r in 0..k {
                for (i, v) in acc[..n].iter_mut().enumerate() {
                    *v += self.s[ix(n, i, j + r)].abs() * e_row[r];
                }
            }
        }
        Ok(acc[..n].iter().fold(0.0f64, |w, v| w.max(*v)))
    }

    /// `|M^-1|` (entrywise, row-major of stride `k`) of the merged block at
    /// `j` over `h`, `M = D h`, column by column by [`Modal::block_solve`].
    /// The certification's terms only. O(k^3).
    fn block_minv_abs(&self, j: usize, k: usize, h: f64) -> Result<Vec<f64>, ModalError> {
        let mut ninv = vec![0.0f64; k * k];
        let mut col = vec![0.0f64; k];
        for c in 0..k {
            col.iter_mut().enumerate().for_each(|(r, v)| *v = if r == c { 1.0 } else { 0.0 });
            self.block_solve(j, k, h, &mut col)?;
            for r in 0..k {
                ninv[r * k + c] = col[r].abs();
            }
        }
        Ok(ninv)
    }

    /// The Pade error of a block that takes solves ([`SOLVE_DECAY`]),
    /// carried to `x` in absolute value (EX1H, EX1G's failure B). Such a
    /// block's step is linear in its `E`: an error `dE` reaches `y` as
    /// `L(dE) = dE y0 + M^-1 dE c0 + M^-2 dE c1` (`c0 = h g0`, `c1 = h^2
    /// g1`). The estimate's signed differences carry `E`'s other
    /// evaluations along the probe's own inputs, `L(E' - E)`, and along a
    /// direction whose constant and ramp cancel ([`Modal::cancel_direction`])
    /// `c0` and `c1` are each up to 1e5 times the result, so `L` weighs the
    /// entries of `dE` very unevenly and the step's own `dE` can sit where
    /// the differences do not: on random case 4270 (skewed, n 8, rung 23)
    /// `|dE|` was 1.4e-15 against differences of 7.5e-15, yet `L(dE)` was
    /// 5.4 times `L(E' - E)` (true 2.91e-10, estimate 1.62e-10); on the n =
    /// 32 ortho member 260 (rung 16) `|dE|` 3.7e-11 was 2.6 times the
    /// largest difference, the error common to every scaling that `d_id`
    /// sees in an augmented block and that a solving block has no `E - I`
    /// form to see ([`Modal::at_scaled`]), so the signed estimate was 2.2
    /// times under. Entrywise, with `Delta = max(|E1 - E|, |E9 - E|)` (`E1`
    /// at one squaring more, `E9` [9/9]): `|S| [`CERT_EST_FACTOR`] (Delta
    /// |y0| + |M^-1| (Delta |c0| + |M^-1| Delta |c1|))`, a bound on `|L(dE)|`
    /// wherever `|dE| <= CERT_EST_FACTOR Delta` entrywise, whatever the
    /// inputs' alignment. Its size only sends more probes to the
    /// measurement ([`Modal::pade_measured`]), which then decides. Absolute
    /// (not relative to the reference). Recomputes the block's three
    /// exponentials; the certification only.
    fn solve_carry(&self, h: f64, ws: &mut Workspace) -> Result<f64, ModalError> {
        let n = self.n;
        let mut acc = [0.0f64; MAX_N];
        for (bi, blk) in self.blocks.iter().enumerate() {
            if blk.kind != BlockKind::Merged || !self.solves(bi, h) {
                continue;
            }
            let (j, k) = (blk.start, blk.size);
            let eval = |extra: i32, form: PadeForm, ws: &mut Workspace| -> Result<Vec<f64>, ModalError> {
                self.block_m(j, k, h, &mut ws.pade.m);
                pade_exp_scaled(&mut ws.pade, k, extra, form)?;
                Ok(ws.pade.m[..k * k].to_vec())
            };
            let e = eval(0, PadeForm::P13, ws)?;
            let e1 = eval(1, PadeForm::P13, ws)?;
            let e9 = eval(0, PadeForm::P9, ws)?;
            let delta: Vec<f64> = (0..k * k).map(|p| CERT_EST_FACTOR * (e1[p] - e[p]).abs().max((e9[p] - e[p]).abs())).collect();
            let ninv = self.block_minv_abs(j, k, h)?;
            let mv = |a: &[f64], v: &[f64]| -> Vec<f64> { (0..k).map(|r| (0..k).map(|c| a[r * k + c] * v[c]).sum::<f64>()).collect() };
            let h2 = h * h;
            let y0a: Vec<f64> = (0..k).map(|r| ws.y0[j + r].abs()).collect();
            let c0a: Vec<f64> = (0..k).map(|r| (h * ws.g0[j + r]).abs()).collect();
            let c1a: Vec<f64> = (0..k).map(|r| (h2 * ws.g1[j + r]).abs()).collect();
            let inner = mv(&ninv, &mv(&delta, &c1a));
            let dc = mv(&delta, &c0a);
            let outer = mv(&ninv, &(0..k).map(|r| dc[r] + inner[r]).collect::<Vec<_>>());
            let dy = mv(&delta, &y0a);
            for r in 0..k {
                let row = dy[r] + outer[r];
                for (i, v) in acc[..n].iter_mut().enumerate() {
                    *v += self.s[ix(n, i, j + r)].abs() * row;
                }
            }
        }
        Ok(acc[..n].iter().fold(0.0f64, |w, v| w.max(*v)))
    }

    /// The reference a probe's error is relative to, as R151's oracle tests
    /// measure a step's (`modal_tests`' module docs): the largest of `|x0|`,
    /// `|x(h)|`, and the homogeneous and forcing parts `S yh` and `S yf` the
    /// step summed (from `ws` after [`Modal::at`]). O(n^2).
    fn probe_ref(&self, x0: &[f64], x: &[f64], ws: &Workspace) -> f64 {
        let n = self.n;
        let (mut eh, mut ef) = (0.0f64, 0.0f64);
        for i in 0..n {
            let (mut a, mut b) = (0.0, 0.0);
            for k in 0..n {
                a += self.s[ix(n, i, k)] * ws.yh[k];
                b += self.s[ix(n, i, k)] * ws.yf[k];
            }
            eh = eh.max(a.abs());
            ef = ef.max(b.abs());
        }
        inf(x0).max(inf(x)).max(eh).max(ef).max(f64::MIN_POSITIVE)
    }

    /// The merged blocks' Pade error in the evaluation `ws` holds (after
    /// [`Modal::at`] over `h`), measured: each block's f64 result against a
    /// double-double exponential of the same f64 input ([`dd_expm`]),
    /// carried to `x` through `S`, relative to `reff`. The certification's
    /// fallback when the estimate is over its limit: about 20 times the
    /// cost of an f64 evaluation, paid only on those probes.
    fn pade_measured(&self, h: f64, u0: &[f64], du: &[f64], ws: &Workspace, reff: f64) -> Result<f64, ModalError> {
        let n = self.n;
        let mut dx = vec![0.0; n];
        // |S| |dy| (the f64 sum's rounding) and |S| (|E| |y0| + |f|) (the
        // double-double exponential's own error), per row of `x`.
        let mut mag = vec![0.0; n];
        let mut big = vec![0.0; n];
        for (bi, blk) in self.blocks.iter().enumerate() {
            if blk.kind != BlockKind::Merged {
                continue;
            }
            let (j, k) = (blk.start, blk.size);
            let e = self.block_truth(bi, h, u0, du, ws)?;
            let dim = k + 2;
            for r in 0..k {
                let mut acc = e[r * dim + k];
                let mut size = e[r * dim + k].hi.abs();
                for c in 0..k {
                    acc = acc.add(e[r * dim + c].mul(Dd::f(ws.y0[j + c])));
                    size += (e[r * dim + c].hi * ws.y0[j + c]).abs();
                }
                let dy = Dd::f(ws.y[j + r]).sub(acc);
                let dy = dy.hi + dy.lo;
                for i in 0..n {
                    let sv = self.s[ix(n, i, j + r)];
                    dx[i] += sv * dy;
                    mag[i] += (sv * dy).abs();
                    big[i] += sv.abs() * size;
                }
            }
        }
        // The measurement is the error itself, so a margin for its own
        // rounding keeps it from reading a hair under it (EX1E: the tests
        // assert the estimate is never under the true error): the f64 sum
        // through `S`, `2 (n + 2) eps |S| |dy|`, and the double-double
        // exponential's error, under 2^-104 times its at most ~40 squarings'
        // growth, so 2^-60 of `|S| (|E| |y0| + |f|)`.
        let margin = 2.0 * (n + 2) as f64 * f64::EPSILON * inf(&mag) + 2f64.powi(-60) * inf(&big);
        Ok((inf(&dx) + margin) / reff)
    }

    /// The double-double exponential of merged block `bi`'s augmented
    /// matrix (order `size + 2`, row-major) over `h`, from the data that
    /// block's evaluation starts from: an augmented block, its f64 input
    /// ([`Modal::merged_input`] from `ws`'s `g0`, `g1`); a block that takes
    /// solves ([`SOLVE_DECAY`]), `fl(D h)` with `c0 = h S^-1 B u0` and `c1 =
    /// h^2 S^-1 B du` formed in double-double from `S^-1 B`, `u0`, `du`, so
    /// the solves' forming of `u(h)` is measured, not assumed.
    fn block_truth(&self, bi: usize, h: f64, u0: &[f64], du: &[f64], ws: &Workspace) -> Result<Vec<Dd>, ModalError> {
        let (j, k) = (self.blocks[bi].start, self.blocks[bi].size);
        let m = self.m;
        let dim = k + 2;
        let mut pm = vec![0.0; dim * dim];
        self.merged_input(j, k, h, &ws.g0, &ws.g1, &mut pm);
        let mut pd: Vec<Dd> = pm.iter().map(|&v| Dd::f(v)).collect();
        if self.solves(bi, h) {
            let hd = Dd::f(h);
            for r in 0..k {
                let (mut a, mut b) = (Dd::ZERO, Dd::ZERO);
                for q in 0..m {
                    let g = Dd::f(self.gb[(j + r) * m + q]);
                    a = a.add(g.mul(Dd::f(u0[q])));
                    b = b.add(g.mul(Dd::f(du[q])));
                }
                pd[r * dim + k] = a.mul(hd);
                pd[r * dim + k + 1] = b.mul(hd).mul(hd);
            }
        }
        dd_expm(&pd, dim)
    }

    /// The terms of `f(t) = c x(t)` from `x(0) = x0` under `u0 + du t`, as
    /// `c0 + q t + sum w_j e^(lambda_j t)` (the crossing search's input, a
    /// later slice). Per block `y = alpha + beta t + e^(D t) (y0 - alpha)`,
    /// `beta = -D^-1 g1`, `alpha = D^-1 (beta - g0)`. No allocation (buffers
    /// from [`Modal::terms_buffer`]). [`ModalError::Unsupported`] for a merged
    /// block (its response is a polynomial times exponentials) and for a zero
    /// rate under a ramp (a `t^2` term). A rate near zero gives large `alpha`
    /// that cancel against `w`: the form is exact but such terms carry the
    /// rounding of their size, which the search's rounding floor must cover.
    pub fn terms(&self, c: &[f64], x0: &[f64], u0: &[f64], du: &[f64], out: &mut Terms) -> Result<(), ModalError> {
        let (n, m) = (self.n, self.m);
        if c.len() != n || x0.len() != n || u0.len() != m || du.len() != m || out.w.len() < n || out.lambda.len() < n {
            return Err(ModalError::Shape);
        }
        out.c0 = 0.0;
        out.q = 0.0;
        out.len = 0;
        // cs = c S (row), y0, g0, g1 on the fly.
        let cs = |k: usize| (0..n).map(|i| c[i] * self.s[ix(n, i, k)]).sum::<f64>();
        let y0 = |k: usize| (0..n).map(|i| self.sinv[ix(n, k, i)] * x0[i]).sum::<f64>();
        let g = |k: usize, u: &[f64]| (0..m).map(|j| self.gb[k * m + j] * u[j]).sum::<f64>();
        for blk in &self.blocks {
            let j = blk.start;
            match blk.kind {
                BlockKind::Merged => return Err(ModalError::Unsupported),
                BlockKind::Real => {
                    let l = self.d[ix(n, j, j)];
                    let (yy, g0, g1, cj) = (y0(j), g(j, u0), g(j, du), cs(j));
                    if l == 0.0 {
                        if g1 != 0.0 {
                            return Err(ModalError::Unsupported);
                        }
                        out.c0 += cj * yy;
                        out.q += cj * g0;
                        continue;
                    }
                    let beta = -g1 / l;
                    let alpha = (beta - g0) / l;
                    out.c0 += cj * alpha;
                    out.q += cj * beta;
                    out.w[out.len] = (cj * (yy - alpha), 0.0);
                    out.lambda[out.len] = (l, 0.0);
                    out.len += 1;
                }
                BlockKind::Pair => {
                    let a = self.d[ix(n, j, j)];
                    let b = self.d[ix(n, j, j + 1)];
                    let cc = self.d[ix(n, j + 1, j)];
                    let w = pair_omega(b, cc);
                    let det = a * a - b * cc;
                    // D^-1 = [a -b; -c a] / det
                    let inv = |v0: f64, v1: f64| ((a * v0 - b * v1) / det, (-cc * v0 + a * v1) / det);
                    let (g00, g01, g10, g11) = (g(j, u0), g(j + 1, u0), g(j, du), g(j + 1, du));
                    let (bt0, bt1) = inv(-g10, -g11);
                    let (al0, al1) = inv(bt0 - g00, bt1 - g01);
                    let (c0j, c1j) = (cs(j), cs(j + 1));
                    out.c0 += c0j * al0 + c1j * al1;
                    out.q += c0j * bt0 + c1j * bt1;
                    let (v0, v1) = (y0(j) - al0, y0(j + 1) - al1);
                    let cv = c0j * v0 + c1j * v1;
                    let cnv = c0j * b * v1 + c1j * cc * v0;
                    let wr = 0.5 * cv;
                    let wi = -0.5 * cnv / w;
                    out.w[out.len] = (wr, wi);
                    out.lambda[out.len] = (a, w);
                    out.w[out.len + 1] = (wr, -wi);
                    out.lambda[out.len + 1] = (a, -w);
                    out.len += 2;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "modal_tests.rs"]
mod modal_tests;

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for ModalError {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            ModalError::Shape => 0,
            ModalError::NonFinite => 0,
            ModalError::QrNoConvergence => 0,
            ModalError::SylvesterSingular => 0,
            ModalError::SwapRejected => 0,
            ModalError::PadeFailed => 0,
            ModalError::IllConditioned => 0,
            ModalError::Unsupported => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for Block {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.start, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.size, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for BlockKind {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            BlockKind::Real => 0,
            BlockKind::Pair => 0,
            BlockKind::Merged => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for PadeWork {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.k, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.m, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a2, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a4, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a6, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.piv, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_s, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PadeForm {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            PadeForm::P13 => 0,
            PadeForm::P9 => 0,
            PadeForm::P13Minus1 => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for Dd {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hi, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.lo, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Modal {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.n, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.m, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sinv, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.d, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.gb, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.blocks, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cond, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cond_bal, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.residual, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.max_merged, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.bdesc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.floor, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.split, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Workspace {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.y0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.y, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.yh, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.yf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.bsq, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pade, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Terms {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.lambda, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.len, w); }
        n
    }
}
