//! Tests of the exact step's kernels (modal.rs). The reference throughout is
//! a Pade 13 oracle evaluated in double-double arithmetic (~32 digits, scaled
//! to theta_13 / 4 so the approximant's own truncation is below 1e-31): it
//! computes the exact exponential of the given f64 matrix, which the f64
//! Pade (K6) cannot on stiff matrices. Every random draw is seeded.
//!
//! Relative error of a step: `|x - x_true|_inf / ref`, `ref` the largest of
//! `|x_true|`, `|x0|`, `|e^(Ah) x0|` and `|forcing response|` (the terms the
//! step sums), so a fully decayed state is judged against what it decayed
//! from.

use super::*;

// ---- seeded generator ------------------------------------------------------------

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }
    fn unif(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.unif()
    }
    fn logu(&mut self, a: f64, b: f64) -> f64 {
        10f64.powf(self.range(a, b))
    }
    fn normal(&mut self) -> f64 {
        let u1 = self.unif().max(1e-300);
        let u2 = self.unif();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
    fn below(&mut self, k: usize) -> usize {
        (self.next() % k as u64) as usize
    }
}

// ---- double-double -------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
struct Dd {
    hi: f64,
    lo: f64,
}

fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let bb = s - a;
    (s, (a - (s - bb)) + (b - bb))
}
fn quick(a: f64, b: f64) -> Dd {
    let s = a + b;
    Dd { hi: s, lo: b - (s - a) }
}

impl Dd {
    const ZERO: Dd = Dd { hi: 0.0, lo: 0.0 };
    fn f(x: f64) -> Dd {
        Dd { hi: x, lo: 0.0 }
    }
    fn add(self, y: Dd) -> Dd {
        let (s, e) = two_sum(self.hi, y.hi);
        let (t, f) = two_sum(self.lo, y.lo);
        let r = quick(s, e + t);
        quick(r.hi, r.lo + f)
    }
    fn neg(self) -> Dd {
        Dd { hi: -self.hi, lo: -self.lo }
    }
    fn sub(self, y: Dd) -> Dd {
        self.add(y.neg())
    }
    fn mul(self, y: Dd) -> Dd {
        let p = self.hi * y.hi;
        let e = self.hi.mul_add(y.hi, -p) + (self.hi * y.lo + self.lo * y.hi);
        quick(p, e)
    }
    fn mulf(self, y: f64) -> Dd {
        self.mul(Dd::f(y))
    }
    fn div(self, y: Dd) -> Dd {
        let q1 = self.hi / y.hi;
        let r = self.sub(y.mulf(q1));
        let q2 = r.hi / y.hi;
        let r = r.sub(y.mulf(q2));
        let q3 = r.hi / y.hi;
        quick(q1, q2).add(Dd::f(q3))
    }
    fn to_f(self) -> f64 {
        self.hi + self.lo
    }
}

/// exp of a k x k double-double matrix by Pade 13, scaled to theta_13 / 4.
fn dd_expm(m: &[Dd], k: usize) -> Vec<Dd> {
    let mut norm = 0.0f64;
    for j in 0..k {
        norm = norm.max((0..k).map(|i| m[i * k + j].hi.abs()).sum::<f64>());
    }
    let s = if norm > THETA13 / 4.0 { (norm / (THETA13 / 4.0)).log2().ceil() as i32 } else { 0 };
    let sc = 2f64.powi(-s);
    let a: Vec<Dd> = m.iter().map(|v| Dd { hi: v.hi * sc, lo: v.lo * sc }).collect();
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
                r[i] = r[i].add(x[i].mulf(*c));
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
    let u = mm(&a6, &t);
    let u = lin(&[(&u, 1.0), (&a6, b[7]), (&a4, b[5]), (&a2, b[3])], b[1]);
    let u = mm(&a, &u);
    let t = lin(&[(&a6, b[12]), (&a4, b[10]), (&a2, b[8])], 0.0);
    let v = mm(&a6, &t);
    let v = lin(&[(&v, 1.0), (&a6, b[6]), (&a4, b[4]), (&a2, b[2])], b[0]);
    let num: Vec<Dd> = (0..k * k).map(|i| v[i].add(u[i])).collect();
    let mut den: Vec<Dd> = (0..k * k).map(|i| v[i].sub(u[i])).collect();
    // Gaussian elimination with partial pivoting, all right-hand sides.
    let mut x = num;
    for c in 0..k {
        let p = (c..k).max_by(|&i, &j| den[i * k + c].hi.abs().partial_cmp(&den[j * k + c].hi.abs()).unwrap()).unwrap();
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
    x
}

/// The oracle: `x(h)` for `x' = A x + B (u0 + du t)`, with the reference
/// magnitude (see the module docs).
fn oracle(a: &[f64], b: &[f64], n: usize, m: usize, x0: &[f64], u0: &[f64], du: &[f64], h: f64) -> (Vec<f64>, f64) {
    let k = n + 2;
    let hd = Dd::f(h);
    let mut mat = vec![Dd::ZERO; k * k];
    for i in 0..n {
        for j in 0..n {
            mat[i * k + j] = Dd::f(a[i * n + j]).mul(hd);
        }
        let (mut g0, mut g1) = (Dd::ZERO, Dd::ZERO);
        for j in 0..m {
            g0 = g0.add(Dd::f(b[i * m + j]).mulf(u0[j]));
            g1 = g1.add(Dd::f(b[i * m + j]).mulf(du[j]));
        }
        mat[i * k + n] = g0.mul(hd);
        mat[i * k + n + 1] = g1.mul(hd).mul(hd);
    }
    mat[(n + 1) * k + n] = Dd::f(1.0);
    let e = dd_expm(&mat, k);
    let mut x = vec![0.0; n];
    let (mut ex, mut fo, mut xt) = (0.0f64, 0.0f64, 0.0f64);
    for i in 0..n {
        let mut acc = Dd::ZERO;
        for j in 0..n {
            acc = acc.add(e[i * k + j].mulf(x0[j]));
        }
        ex = ex.max(acc.to_f().abs());
        fo = fo.max(e[i * k + n].to_f().abs());
        let v = acc.add(e[i * k + n]).to_f();
        xt = xt.max(v.abs());
        x[i] = v;
    }
    let x0n = x0.iter().fold(0.0f64, |s, v| s.max(v.abs()));
    (x, xt.max(ex).max(fo).max(x0n))
}

/// exp of a f64 matrix by the double-double oracle, rounded to f64.
fn dd_exp_f(a: &[f64], k: usize, h: f64) -> Vec<f64> {
    let m: Vec<Dd> = a.iter().map(|&v| Dd::f(v).mulf(h)).collect();
    dd_expm(&m, k).iter().map(|v| v.to_f()).collect()
}

fn matmul_n(a: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut r = vec![0.0; n * n];
    for i in 0..n {
        for k in 0..n {
            for j in 0..n {
                r[i * n + j] += a[i * n + k] * b[k * n + j];
            }
        }
    }
    r
}
fn transpose(a: &[f64], n: usize) -> Vec<f64> {
    (0..n * n).map(|p| a[(p % n) * n + p / n]).collect()
}
fn eye(n: usize) -> Vec<f64> {
    (0..n * n).map(|p| if p / n == p % n { 1.0 } else { 0.0 }).collect()
}
fn diff_norm1(a: &[f64], b: &[f64], n: usize) -> f64 {
    let d: Vec<f64> = a.iter().zip(b).map(|(x, y)| x - y).collect();
    norm1(&d, n)
}
fn inf_norm(v: &[f64]) -> f64 {
    v.iter().fold(0.0f64, |s, x| s.max(x.abs()))
}

// ---- the random set ----------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Class {
    /// Constructed spectrum, orthogonal V: the exponential is well conditioned.
    Ortho,
    /// Constructed spectrum with Jordan and near-Jordan blocks, orthogonal V.
    Jordan,
    /// Constructed spectrum, V with cond <= 100.
    Skewed,
    /// Dense Gaussian.
    Gauss,
}

struct Case {
    n: usize,
    m: usize,
    a: Vec<f64>,
    b: Vec<f64>,
    x0: Vec<f64>,
    u0: Vec<f64>,
    du: Vec<f64>,
    h: f64,
    class: Class,
}

fn random_orthogonal(rng: &mut Rng, n: usize) -> Vec<f64> {
    let mut q = eye(n);
    for _ in 0..n {
        let v: Vec<f64> = (0..n).map(|_| rng.normal()).collect();
        let vv: f64 = v.iter().map(|x| x * x).sum();
        if vv == 0.0 {
            continue;
        }
        for i in 0..n {
            let s: f64 = (0..n).map(|j| q[i * n + j] * v[j]).sum::<f64>() * 2.0 / vv;
            for j in 0..n {
                q[i * n + j] -= s * v[j];
            }
        }
    }
    q
}

/// A quasi-triangular T with the drawn spectrum; returns (T, rate_min, rate_max).
fn draw_t(rng: &mut Rng, n: usize, jordan: bool) -> (Vec<f64>, f64, f64) {
    let mut t = vec![0.0; n * n];
    let base = rng.logu(-3.0, 3.0);
    let spread = if rng.unif() < 0.5 { rng.logu(0.0, 12.0) } else { rng.logu(0.0, 3.0) };
    let mag = |rng: &mut Rng| base * spread.powf(rng.unif());
    let (mut rmin, mut rmax) = (f64::INFINITY, 0.0f64);
    let mut i = 0;
    let p_jordan = if jordan { 0.45 } else { 0.0 };
    while i < n {
        let r = rng.unif();
        let left = n - i;
        if left >= 2 && r < p_jordan {
            // A repeated eigenvalue (Jordan chain) or a near-defective pair.
            let len = (2 + rng.below(3)).min(left);
            let l = -mag(rng);
            let near = rng.unif() < 0.5;
            for k in 0..len {
                let dl = if near { l * rng.logu(-10.0, -3.0) * k as f64 } else { 0.0 };
                t[(i + k) * n + i + k] = l + dl;
                if k + 1 < len {
                    t[(i + k) * n + i + k + 1] = l.abs() * rng.logu(-2.0, 1.0);
                }
            }
            rmin = rmin.min(l.abs());
            rmax = rmax.max(l.abs());
            i += len;
        } else if left >= 2 && r < p_jordan + 0.5 * (1.0 - p_jordan) {
            let w0 = mag(rng);
            let q = rng.logu(0.5f64.log10(), 5.0);
            let sr = -w0 / (2.0 * q);
            let wi = w0 * (1.0 - 1.0 / (4.0 * q * q)).max(0.0).sqrt();
            let nn = rng.logu(-1.0, 1.0);
            t[i * n + i] = sr;
            t[(i + 1) * n + i + 1] = sr;
            t[i * n + i + 1] = wi * nn;
            t[(i + 1) * n + i] = -wi / nn;
            rmin = rmin.min(w0);
            rmax = rmax.max(w0);
            i += 2;
        } else {
            let z = rng.unif();
            let l = if z < 0.06 { 0.0 } else { -mag(rng) };
            t[i * n + i] = l;
            if l != 0.0 {
                rmin = rmin.min(l.abs());
            }
            rmax = rmax.max(l.abs());
            i += 1;
        }
    }
    // Coupling above the diagonal blocks.
    for r in 0..n {
        for c in (r + 2)..n {
            let s = (t[r * n + r].abs() * t[c * n + c].abs()).sqrt().max(rmin.min(1e300));
            t[r * n + c] += 0.3 * rng.normal() * s;
        }
    }
    if !rmin.is_finite() {
        rmin = 1.0;
    }
    if rmax == 0.0 {
        rmax = 1.0;
    }
    (t, rmin, rmax)
}

fn gen_case(rng: &mut Rng, idx: usize) -> Case {
    let n = 1 + rng.below(12);
    let m = rng.below(3);
    let class = [Class::Ortho, Class::Jordan, Class::Skewed, Class::Gauss][idx % 4];
    let (a, rmin, rmax) = match class {
        Class::Gauss => {
            let s = rng.logu(-3.0, 3.0);
            let a: Vec<f64> = (0..n * n).map(|_| rng.normal() * s / (n as f64).sqrt()).collect();
            (a, s, s)
        }
        _ => {
            let (t, rmin, rmax) = draw_t(rng, n, class == Class::Jordan);
            let a = if class == Class::Skewed {
                let q1 = random_orthogonal(rng, n);
                let q2 = random_orthogonal(rng, n);
                let dg: Vec<f64> = (0..n).map(|_| rng.logu(-1.0, 1.0)).collect();
                let mut v = q1.clone();
                let mut vi = transpose(&q1, n);
                for i in 0..n {
                    for j in 0..n {
                        v[i * n + j] *= dg[j];
                        vi[i * n + j] /= dg[i];
                    }
                }
                let v = matmul_n(&v, &q2, n);
                let vi = matmul_n(&transpose(&q2, n), &vi, n);
                matmul_n(&matmul_n(&v, &t, n), &vi, n)
            } else {
                let q = random_orthogonal(rng, n);
                matmul_n(&matmul_n(&q, &t, n), &transpose(&q, n), n)
            };
            (a, rmin, rmax)
        }
    };
    let lo = (0.1 / rmax).log10();
    let hi = (10.0 / rmin).log10().max(lo);
    let mut h = rng.logu(lo, hi);
    if class == Class::Gauss {
        h = h.min(3.0 / rmax);
    }
    let b: Vec<f64> = (0..n * m).map(|_| rng.normal()).collect();
    let x0: Vec<f64> = (0..n).map(|_| rng.normal()).collect();
    let u0: Vec<f64> = (0..m).map(|_| rng.normal()).collect();
    let du: Vec<f64> = (0..m).map(|_| rng.normal() / h).collect();
    Case { n, m, a, b, x0, u0, du, h, class }
}

fn cases(count: usize, seed: u64) -> Vec<Case> {
    let mut rng = Rng(seed);
    (0..count).map(|i| gen_case(&mut rng, i)).collect()
}

/// The random set's size: 10,000 in release; 200 in debug, where the
/// double-double oracle is about 40 times slower (the release run is the
/// gate's measurement).
const RANDOM_CASES: usize = if cfg!(debug_assertions) { 200 } else { 10_000 };

// ---- R151: conditioning-aware oracle criterion ------------------------------------

/// R151 (coordinator ruling, confirmed by the owner 2026-11-24): an accepted
/// answer must be within this many ulp (units of eps) times the answer's
/// measured sensitivity to its own f64 input.
const R151_ULPS: f64 = 100.0;

/// R151's floor: wherever `R151_ULPS eps kappa` is below this, the answer
/// must still be within it (the bound the tests held before R151).
const R151_FLOOR: f64 = 1e-12;

/// The sensitivity below which a matrix counts as well conditioned:
/// `R151_FLOOR / (R151_ULPS eps)` = 45.04. Below it R151's allowance is the
/// floor 1e-12 itself, the old bound, so nothing about the input excuses
/// an error there, and a decline on such a matrix is a failure.
const WELL_CONDITIONED_KAPPA: f64 = R151_FLOOR / (R151_ULPS * f64::EPSILON);

/// R151's allowance for an answer of measured sensitivity `kappa`.
fn r151_allowed(kappa: f64) -> f64 {
    (R151_ULPS * f64::EPSILON * kappa).max(R151_FLOOR)
}

/// The worst `error / allowance` over a test and the case it came from;
/// `kappa` is measured only when the error could beat the current worst
/// (an error at most `worst` times the floor cannot, as the allowance is
/// never below the floor), so the oracle's extra runs stay few.
#[derive(Default)]
struct R151Worst {
    ratio: f64,
    kappa: f64,
    err: f64,
    at: String,
    kappa_max: f64,
}

impl R151Worst {
    fn add(&mut self, err: f64, at: impl FnOnce() -> String, kappa: impl FnOnce() -> f64) -> f64 {
        if err / R151_FLOOR <= self.ratio.min(1.0) {
            return err / R151_FLOOR;
        }
        let k = kappa();
        self.kappa_max = self.kappa_max.max(k);
        let r = err / r151_allowed(k);
        if r > self.ratio {
            *self = R151Worst { ratio: r, kappa: k, err, at: at(), kappa_max: self.kappa_max };
        }
        r
    }
    fn line(&self) -> String {
        format!("worst error / R151 allowance {:.3} (error {:.2e}, kappa {:.2e}, {})", self.ratio, self.err, self.kappa, self.at)
    }
}

/// The sensitivity of `e^(A h)` to its f64 input: the oracle's response to
/// four random entrywise relative perturbations of `A` of at most eps (one
/// ulp), `|e^(A' h) - e^(A h)|_1 / (eps |e^(A h)|_1)`, the largest of four
/// (the 1-norm the K2 and K6 tests measure their error in).
fn kappa_exp(a: &[f64], n: usize, h: f64, rng: &mut Rng) -> f64 {
    let e = dd_exp_f(a, n, h);
    let en = norm1(&e, n).max(1e-300);
    let mut k = 0.0f64;
    for _ in 0..4 {
        let ap: Vec<f64> = a.iter().map(|v| v * (1.0 + f64::EPSILON * rng.range(-1.0, 1.0))).collect();
        let ep = dd_exp_f(&ap, n, h);
        k = k.max(diff_norm1(&ep, &e, n) / (f64::EPSILON * en));
    }
    k
}

// ---- K1 / K2 ---------------------------------------------------------------------

#[test]
fn hessenberg_and_schur_backward() {
    let set = cases(RANDOM_CASES, 11);
    let (mut hb, mut ho, mut sb, mut so) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let mut fails = 0;
    QR_MAX_ITS.with(|c| c.set(0));
    for c in &set {
        let n = c.n;
        let anorm = norm1(&c.a, n).max(1e-300);
        let mut h = c.a.clone();
        let mut q = vec![0.0; n * n];
        hessenberg(&mut h, &mut q, n).unwrap();
        for i in 0..n {
            for j in 0..i.saturating_sub(1) {
                assert_eq!(h[i * n + j], 0.0, "Hessenberg structure");
            }
        }
        let back = matmul_n(&matmul_n(&q, &h, n), &transpose(&q, n), n);
        hb = hb.max(diff_norm1(&back, &c.a, n) / anorm);
        ho = ho.max(diff_norm1(&matmul_n(&transpose(&q, n), &q, n), &eye(n), n));
        if schur(&mut h, &mut q, n).is_err() {
            fails += 1;
            continue;
        }
        // Quasi-triangular, 2x2 blocks standardised and complex.
        let mut i = 0;
        while i < n {
            for r in (i + 2)..n {
                assert_eq!(h[r * n + i], 0.0);
            }
            if i + 1 < n && h[(i + 1) * n + i] != 0.0 {
                assert_eq!(h[i * n + i], h[(i + 1) * n + i + 1], "standardised diagonal");
                assert!(h[i * n + i + 1] * h[(i + 1) * n + i] < 0.0, "complex pair");
                if i + 2 < n {
                    assert_eq!(h[(i + 2) * n + i + 1], 0.0, "no adjacent 2x2 blocks overlap");
                }
                i += 2;
            } else {
                i += 1;
            }
        }
        let back = matmul_n(&matmul_n(&q, &h, n), &transpose(&q, n), n);
        sb = sb.max(diff_norm1(&back, &c.a, n) / anorm);
        so = so.max(diff_norm1(&matmul_n(&transpose(&q, n), &q, n), &eye(n), n));
    }
    let its = QR_MAX_ITS.with(|c| c.get());
    println!("K1 Hessenberg: backward {hb:.2e}, orthogonality {ho:.2e}; K2 Schur: backward {sb:.2e}, orthogonality {so:.2e}, failures {fails}/{}, most iterations on one eigenvalue {its}", set.len());
    assert_eq!(fails, 0);
    assert!(hb < 1e-13 && ho < 1e-13 && sb < 1e-13 && so < 1e-13);
}

/// K2 against the oracle: `Q e^(T h) Q^T` equals `e^(A h)` (both by the
/// double-double oracle), so the Schur form preserves the exponential.
///
/// Restated by R151 (coordinator ruling, confirmed by the owner 2026-11-24).
/// Old bound: 1e-11 relative (1-norm) on every matrix. Replaced because the
/// exact exponential of the f64 data is itself that sensitive: Schur's
/// backward error is 6e-15, but on stiff non-normal draws `e^(A h)` moves by
/// up to 2e11 eps for a one-ulp change of `A` (`kappa_exp`; the random set's
/// worst, a skewed-spectrum matrix), so the 6.2e-6 EX1 measured is that
/// sensitivity, not the reduction. Criterion: `R151_ULPS eps kappa`, and
/// `R151_FLOOR` wherever that is smaller.
#[test]
fn schur_preserves_the_exponential() {
    let set = cases(RANDOM_CASES / 10, 12);
    let mut w = R151Worst::default();
    let mut raw = 0.0f64;
    let mut rng = Rng(1200);
    for (idx, c) in set.iter().enumerate() {
        let n = c.n;
        let mut h = c.a.clone();
        let mut q = vec![0.0; n * n];
        hessenberg(&mut h, &mut q, n).unwrap();
        schur(&mut h, &mut q, n).unwrap();
        let ea = dd_exp_f(&c.a, n, c.h);
        let et = dd_exp_f(&h, n, c.h);
        let back = matmul_n(&matmul_n(&q, &et, n), &transpose(&q, n), n);
        let e = diff_norm1(&back, &ea, n) / norm1(&ea, n).max(1e-300);
        raw = raw.max(e);
        w.add(e, || format!("case {idx}, {:?}, n {n}", c.class), || kappa_exp(&c.a, n, c.h, &mut rng));
    }
    println!("K2 vs oracle over {}: |Q e^(Th) Q^T - e^(Ah)|_1 / |e^(Ah)|_1 worst {raw:.2e}; {}", set.len(), w.line());
    assert!(w.ratio < 1.0, "{}", w.line());
}

// ---- K3 ---------------------------------------------------------------------------

#[test]
fn sylvester_residual() {
    let mut rng = Rng(21);
    let mut worst = 0.0f64;
    let mut singular = 0;
    let total = RANDOM_CASES / 2;
    for idx in 0..total {
        // A Schur form of order n, split at a block boundary; shift the
        // second part away so the split is solvable.
        let c = gen_case(&mut rng, idx);
        let n = c.n;
        if n < 2 {
            continue;
        }
        let mut t = c.a.clone();
        let mut q = vec![0.0; n * n];
        hessenberg(&mut t, &mut q, n).unwrap();
        schur(&mut t, &mut q, n).unwrap();
        let mut p = 1 + rng.below(n - 1);
        if t[p * n + p - 1] != 0.0 {
            p += 1;
        }
        if p >= n {
            continue;
        }
        let qn = n - p;
        let c0: Vec<f64> = (0..p * qn).map(|_| rng.normal()).collect();
        let mut x = c0.clone();
        match sylvester(&t, n, 0, p, p, qn, &mut x, f64::INFINITY) {
            Ok(_) => {
                let mut r = 0.0f64;
                for i in 0..p {
                    for j in 0..qn {
                        let mut s = -c0[i * qn + j];
                        for k in 0..p {
                            s += t[i * n + k] * x[k * qn + j];
                        }
                        for k in 0..qn {
                            s -= x[i * qn + k] * t[(p + k) * n + p + j];
                        }
                        r += s * s;
                    }
                }
                let fa: f64 = (0..p).flat_map(|i| (0..p).map(move |k| (i, k))).map(|(i, k)| t[i * n + k].powi(2)).sum::<f64>().sqrt();
                let fb: f64 = (0..qn).flat_map(|i| (0..qn).map(move |k| (i, k))).map(|(i, k)| t[(p + i) * n + p + k].powi(2)).sum::<f64>().sqrt();
                let fx: f64 = x.iter().map(|v| v * v).sum::<f64>().sqrt();
                let fc: f64 = c0.iter().map(|v| v * v).sum::<f64>().sqrt();
                worst = worst.max(r.sqrt() / ((fa + fb) * fx + fc));
            }
            Err(_) => singular += 1,
        }
    }
    println!("K3 Sylvester: residual / ((|A| + |B|) |X| + |C|) worst {worst:.2e}; singular (a shared eigenvalue across the split) {singular}/{total}");
    assert!(worst < 1e-14, "{worst:e}");
}

// ---- K5 ---------------------------------------------------------------------------

/// K5 against the oracle. Order 1 keeps its bound (1e-14 relative per
/// function, met at 2.2e-16). Order 2 is restated by R151 (coordinator
/// ruling, confirmed by the owner 2026-11-24). Old bound: 1e-13 relative to
/// the block's largest entry on every block. Replaced because a standardised
/// pair `[a b; c a]` with non-normality `nn = sqrt(|b / c|)` up to 1e3 and
/// `|z|` up to 1e3 has an exponential that moves by about `|z| nn` eps for a
/// one-ulp change of `a`, `b` or `c` (EX1's diagnostic: eps |z| nn up to
/// 2e-10; `kappa_block` measures it). Criterion: `R151_ULPS eps kappa`, and
/// `R151_FLOOR` wherever that is smaller; the near-normal blocks (nn < 2,
/// |z| <= 1e3) are also still held to 1e-12 outright, as before.
#[test]
fn block_closed_forms() {
    // Order 1, |z| from 1e-12 to 700, both signs, dense across the switch.
    let mut worst1 = 0.0f64;
    let mut zs: Vec<f64> = (0..=300).map(|k| 10f64.powf(-12.0 + 15.0 * k as f64 / 300.0)).filter(|z| *z <= 700.0).collect();
    zs.extend((0..200).map(|k| 0.9 + 0.2 * k as f64 / 200.0));
    for &z0 in &zs {
        for z in [z0, -z0] {
            let got = real_exp_phi(z);
            // Oracle: exp of [[z, 1, 0], [0, 0, 1], [0, 0, 0]] gives e^z, phi_1, phi_2.
            let e = dd_exp_f(&[z, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], 3, 1.0);
            for (k, want) in [e[0], e[1], e[2]].iter().enumerate() {
                let rel = (got[k] - want).abs() / want.abs().max(f64::MIN_POSITIVE);
                worst1 = worst1.max(rel);
                assert!(rel < 1e-14, "order 1 z {z:e} f{k}: {} vs {want} ({rel:e})", got[k]);
            }
        }
    }
    // Order 2: [a b; c a] h, |z| from 1e-12 to 1e3, damping 1e-5 .. ~1, non-normality to 1e6.
    let mut rng = Rng(5);
    let (mut worst2, mut worst2u, mut worst2n) = (0.0f64, 0.0f64, 0.0f64);
    let mut r151 = R151Worst::default();
    for k in 0..4000 {
        let mz = 10f64.powf(-12.0 + 15.0 * (k % 400) as f64 / 400.0);
        let zeta = rng.logu(-5.0, -0.0001);
        let sr = -mz * zeta * if rng.unif() < 0.1 { -0.5 } else { 1.0 };
        let si = mz * (1.0 - zeta * zeta).sqrt();
        if sr > 700.0 {
            continue;
        }
        let nn = rng.logu(-3.0, 3.0);
        let (b, c) = (si * nn, -si / nn);
        let blk = [sr, b, c, sr];
        // The kernel reads the frequency from the block, as Modal::at does.
        let si = pair_omega(b, c);
        let f = pair_exp_phi(sr, si);
        // f(M) = Re f I + (Im f / si) [0 b; c 0]
        let mat = |re: f64, ri: f64| [re, ri * b, ri * c, re];
        let got = [mat(f[0].0, f[0].1), mat(f[1].0, f[1].1), mat(f[2].0, f[2].1)];
        // Oracle on [[M, I, 0], [0, 0, I], [0, 0, 0]] (6 x 6).
        let mut aug = vec![0.0; 36];
        for r in 0..2 {
            for cc in 0..2 {
                aug[r * 6 + cc] = blk[r * 2 + cc];
            }
            aug[r * 6 + 2 + r] = 1.0;
            aug[(2 + r) * 6 + 4 + r] = 1.0;
        }
        let e = dd_exp_f(&aug, 6, 1.0);
        let pick = |e: &[f64], fk: usize| [e[fk * 2], e[fk * 2 + 1], e[6 + fk * 2], e[6 + fk * 2 + 1]];
        for (fk, g) in got.iter().enumerate() {
            let want = pick(&e, fk);
            let scale = want.iter().fold(0.0f64, |s, v| s.max(v.abs())).max(f64::MIN_POSITIVE);
            let err = (0..4).map(|i| (g[i] - want[i]).abs()).fold(0.0f64, f64::max) / scale;
            worst2 = worst2.max(err);
            // Diagnostic: the error in units of the block's own condition
            // (a rounding of b or c moves the result by about eps |z| nn).
            worst2u = worst2u.max(err / (f64::EPSILON * (1.0 + mz * nn.max(1.0 / nn))));
            if nn.max(1.0 / nn) < 2.0 && mz <= 1e3 {
                worst2n = worst2n.max(err);
            }
            // kappa_block: the oracle's response to one-ulp relative changes
            // of the block's inputs a, b, c (four draws, the largest).
            let kappa_block = || {
                let mut kk = 0.0f64;
                let mut kr = Rng(500 + 3 * k as u64 + fk as u64);
                for _ in 0..4 {
                    let p = |v: f64, r: &mut Rng| v * (1.0 + f64::EPSILON * r.range(-1.0, 1.0));
                    let (ap, bp, cp) = (p(sr, &mut kr), p(b, &mut kr), p(c, &mut kr));
                    let mut augp = aug.clone();
                    augp[0] = ap;
                    augp[1] = bp;
                    augp[6] = cp;
                    augp[7] = ap;
                    let ep = dd_exp_f(&augp, 6, 1.0);
                    let wp = pick(&ep, fk);
                    let d = (0..4).map(|i| (wp[i] - want[i]).abs()).fold(0.0f64, f64::max) / scale;
                    kk = kk.max(d / f64::EPSILON);
                }
                kk
            };
            r151.add(err, || format!("z {sr:.2e}{:+.2e}i nn {nn:.2e} f{fk}", si), kappa_block);
        }
    }
    // Decayed blocks are exactly 0 past underflow.
    assert_eq!(real_exp_phi(-800.0)[0], 0.0);
    assert_eq!(pair_exp_phi(-800.0, 3.0)[0], (0.0, 0.0));
    println!("K5 closed forms vs oracle: order 1 worst {worst1:.2e} relative; order 2 worst {worst2:.2e} relative to the block norm ({worst2u:.2} units of eps (1 + |z| nn)), near-normal blocks with |z| <= 1e3 worst {worst2n:.2e}; order 2 {}", r151.line());
    assert!(worst2n < 1e-12);
    assert!(r151.ratio < 1.0, "{}", r151.line());
}

// ---- K6 ---------------------------------------------------------------------------

/// K6 alone on the random set (`A h`, the full matrix, so up to 39
/// squarings) against the oracle.
///
/// Restated by R151 (coordinator ruling, confirmed by the owner 2026-11-24).
/// Old bound: 1e-14 relative (1-norm) on every matrix whose Pade took at
/// most 9 squarings. Replaced because `e^(A h)` of the f64 data moves by up
/// to 2e11 eps for a one-ulp change of `A` (`kappa_exp`, a skewed-spectrum
/// matrix of the random set), and the squarings' error tracks that
/// sensitivity (EX1 measured 1.1e-11 at 5 to 9 squarings, 5.9e-6 at 20 to
/// 39). Criterion: `R151_ULPS eps kappa`, and `R151_FLOOR` wherever that is
/// smaller, every bucket.
#[test]
fn pade_against_dd() {
    let set = cases(RANDOM_CASES / 5, 31);
    let mut buckets: Vec<(usize, f64, R151Worst)> = (0..6).map(|_| (0usize, 0.0f64, R151Worst::default())).collect();
    let mut w = PadeWork::new(12);
    let mut rng = Rng(3100);
    let mut failed = 0;
    for (idx, c) in set.iter().enumerate() {
        let n = c.n;
        for i in 0..n * n {
            w.m[i] = c.a[i] * c.h;
        }
        if pade_exp(&mut w, n).is_err() {
            failed += 1;
            continue;
        }
        let s = w.last_s;
        let want = dd_exp_f(&c.a, n, c.h);
        let e = diff_norm1(&w.m[..n * n], &want, n) / norm1(&want, n).max(1e-300);
        let bk = [0, 1, 5, 10, 20, 40].iter().rposition(|&lim| s >= lim).unwrap();
        buckets[bk].0 += 1;
        buckets[bk].1 = buckets[bk].1.max(e);
        buckets[bk].2.add(e, || format!("case {idx}, {:?}, n {n}, {s} squarings", c.class), || kappa_exp(&c.a, n, c.h, &mut rng));
    }
    let mut worst = 0.0f64;
    for (k, lim) in ["0", "1-4", "5-9", "10-19", "20-39", ">=40"].iter().enumerate() {
        println!("K6 Pade (f64) vs oracle, squarings {lim:>5}: {:5} matrices, worst {:.2e} relative (1-norm); {}", buckets[k].0, buckets[k].1, buckets[k].2.line());
        worst = worst.max(buckets[k].2.ratio);
    }
    assert_eq!(failed, 0, "Pade failed on a finite matrix");
    assert!(worst < 1.0, "worst error / allowance {worst}");
}

// ---- K4 / K7: the random set ---------------------------------------------------

struct SetStats {
    total: usize,
    counts: [usize; 4],
    /// Declines (any returned error: the partition stays on the trapezoid),
    /// per family, [below, at or above] `WELL_CONDITIONED_KAPPA`.
    declined: [[usize; 2]; 4],
    /// Accepted steps: the worst raw relative error and the worst ratio to
    /// R151's allowance, per family.
    raw: [f64; 4],
    worst: [R151Worst; 4],
    /// Accepted steps over R151's allowance.
    over: usize,
    worst_rt: f64,
    worst_comp: f64,
    /// The composition's worst ratio to R151's allowance.
    comp: R151Worst,
    notes: Vec<String>,
    /// States in merged blocks, and decompositions that regrouped.
    merged_states: usize,
    regrouped: usize,
}

/// The step's measured condition on its f64 data: the oracle's response to
/// four random entrywise relative perturbations of size eps of A and B, in
/// units of eps (relative to the same reference). An f64 method cannot be
/// expected to beat eps times this.
fn kappa_fd(c: &Case, rng: &mut Rng) -> f64 {
    let (x, rf) = oracle(&c.a, &c.b, c.n, c.m, &c.x0, &c.u0, &c.du, c.h);
    let mut k = 0.0f64;
    for _ in 0..4 {
        let a: Vec<f64> = c.a.iter().map(|v| v * (1.0 + f64::EPSILON * rng.range(-1.0, 1.0))).collect();
        let b: Vec<f64> = c.b.iter().map(|v| v * (1.0 + f64::EPSILON * rng.range(-1.0, 1.0))).collect();
        let (y, _) = oracle(&a, &b, c.n, c.m, &c.x0, &c.u0, &c.du, c.h);
        let d = x.iter().zip(&y).map(|(p, q)| (p - q).abs()).fold(0.0f64, f64::max);
        k = k.max(d / (f64::EPSILON * rf));
    }
    k
}

fn run_set(set: &[Case], split: f64, composition: bool) -> SetStats {
    let mut st = SetStats { total: set.len(), counts: [0; 4], declined: [[0; 2]; 4], raw: [0.0; 4], worst: Default::default(), over: 0, worst_rt: 0.0, worst_comp: 0.0, comp: R151Worst::default(), notes: Vec::new(), merged_states: 0, regrouped: 0 };
    for (idx, c) in set.iter().enumerate() {
        let ci = c.class as usize;
        st.counts[ci] += 1;
        let n = c.n;
        let decline = |st: &mut SetStats, what: String| {
            let kf = kappa_fd(c, &mut Rng(c.n as u64 + 2000));
            let well = kf < WELL_CONDITIONED_KAPPA;
            st.declined[ci][if well { 0 } else { 1 }] += 1;
            if well {
                st.notes.push(format!("DECLINED a well-conditioned case {idx}: {:?} n {n} kappa_fd {kf:.2e}: {what}", c.class));
            }
        };
        let md = match decompose_with(&c.a, &c.b, n, c.m, split) {
            Ok(md) => md,
            Err(e) => {
                decline(&mut st, format!("decompose {e:?}"));
                continue;
            }
        };
        st.merged_states += md.blocks().iter().filter(|b| b.kind == BlockKind::Merged).map(|b| b.size).sum::<usize>();
        if md.split_threshold() < split {
            st.regrouped += 1;
        }
        let mut ws = md.workspace();
        let mut x = vec![0.0; n];
        if let Err(e) = md.at(&c.x0, &c.u0, &c.du, c.h, &mut ws, &mut x) {
            decline(&mut st, format!("step {e:?}"));
            continue;
        }
        let (want, rf) = oracle(&c.a, &c.b, n, c.m, &c.x0, &c.u0, &c.du, c.h);
        let rel = x.iter().zip(&want).map(|(a, b)| (a - b).abs()).fold(0.0f64, f64::max) / rf.max(f64::MIN_POSITIVE);
        st.raw[ci] = st.raw[ci].max(rel);
        let blocks = || md.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>();
        let r = st.worst[ci].add(
            rel,
            || format!("case {idx}, n {n}, floor {:.1e}, split {:.0e}, blocks {:?}", md.rounding_floor(), md.split_threshold(), blocks()),
            || kappa_fd(c, &mut Rng(c.n as u64 + 1000)),
        );
        if r >= 1.0 {
            st.over += 1;
            st.notes.push(format!("OVER case {idx}: {:?} n {n} rel {rel:.2e} ratio {r:.2} blocks {:?}", c.class, blocks()));
        }
        st.worst_rt = st.worst_rt.max(diff_norm1(&matmul_n(md.s(), md.s_inv(), n), &eye(n), n) / md.cond());
        if composition {
            // step(h1) then step(h2) against step(h1 + h2).
            let h1 = c.h * 0.37;
            let h2 = c.h - h1;
            let mut x1 = vec![0.0; n];
            let mut x2 = vec![0.0; n];
            if md.step(&c.x0, &c.u0, &c.du, h1, &mut ws, &mut x1).is_ok() {
                let u1: Vec<f64> = c.u0.iter().zip(&c.du).map(|(u, d)| u + d * h1).collect();
                if md.step(&x1, &u1, &c.du, h2, &mut ws, &mut x2).is_ok() {
                    let e = x2.iter().zip(&x).map(|(a, b)| (a - b).abs()).fold(0.0f64, f64::max) / rf.max(inf_norm(&x1));
                    st.worst_comp = st.worst_comp.max(e);
                    st.comp.add(e, || format!("case {idx}, {:?}, n {n}, blocks {:?}", c.class, blocks()), || kappa_fd(c, &mut Rng(c.n as u64 + 1000)));
                }
            }
        }
    }
    st
}

/// K7 on the random set against the oracle (and K4 through it).
///
/// Restated by R151 (coordinator ruling, confirmed by the owner 2026-11-24).
/// Old bound: 1e-12 relative on every accepted step, an over-bound step
/// having to be declined by the kernel's own per-step check. Replaced
/// because the exact answer of the f64 data is itself that sensitive: on
/// the random set the step moves by up to 2.3e11 eps for a one-ulp change of
/// `A` and `B` (`kappa_fd`, the skewed-spectrum family), which no f64 method
/// can beat. Criterion: every accepted step within `R151_ULPS eps kappa_fd`,
/// and within `R151_FLOOR` wherever that is smaller; a decline (any returned
/// error) on a matrix with `kappa_fd` below `WELL_CONDITIONED_KAPPA` is a
/// failure; declines are counted per family on both sides of that line.
#[test]
fn random_set_against_oracle() {
    let set = cases(RANDOM_CASES, 1);
    let st = run_set(&set, SPLIT_NORM_MAX, true);
    let names = ["ortho", "jordan", "skewed (cond V <= 100)", "gauss"];
    for k in 0..4 {
        let cnt = st.counts[k].max(1) as f64;
        println!(
            "K7 random set, {:24}: {:5} cases, declined {:4} ({:.2} %): {} well conditioned (kappa_fd < {:.1}), {} ({:.2} %) not; accepted: worst relative error {:.2e}, {}",
            names[k],
            st.counts[k],
            st.declined[k][0] + st.declined[k][1],
            100.0 * (st.declined[k][0] + st.declined[k][1]) as f64 / cnt,
            st.declined[k][0],
            WELL_CONDITIONED_KAPPA,
            st.declined[k][1],
            100.0 * st.declined[k][1] as f64 / cnt,
            st.raw[k],
            st.worst[k].line()
        );
    }
    println!(
        "K7: accepted over the R151 allowance: {}; S S^-1 round trip worst |S S^-1 - I|_1 / cond {:.2e}; step(h1) + step(h2) vs step(h1 + h2) worst {:.2e}, {}; states in merged blocks {}; regrouped {}",
        st.over, st.worst_rt, st.worst_comp, st.comp.line(), st.merged_states, st.regrouped
    );
    for note in &st.notes {
        println!("  {note}");
    }
    let well: usize = st.declined.iter().map(|d| d[0]).sum();
    assert_eq!(well, 0, "a decline on a well-conditioned case is a failure");
    assert_eq!(st.over, 0, "an accepted step over R151's allowance");
    // The composition property, on the same terms: 1e-12 on every
    // well-conditioned case (there R151's allowance is 1e-12 itself), R151's
    // allowance on the rest, whose split steps carry the same input rounding
    // the oracle comparison excuses.
    // Restated by R151, extended to this property by R158 (coordinator).
    // Old bound: a flat 1e-12 relative on every case. Replaced because the
    // raw worst is 1.95e-9, on an ill-conditioned merged case: the exact
    // exponential's own sensitivity to its f64 input, which the two split
    // steps carry as the oracle comparison does. A decline never counts as a
    // pass (a declined case is counted above, and fails the test below the
    // well-conditioned line).
    assert!(st.comp.ratio < 1.0, "composition: {}", st.comp.line());
}

#[test]
#[ignore = "measurement: the random set with and without balancing"]
fn balance_ab() {
    let set = cases(RANDOM_CASES, 1);
    for nb in [false, true] {
        NO_BALANCE.with(|c| c.set(nb));
        let st = run_set(&set, SPLIT_NORM_MAX, false);
        let r = st.worst.iter().fold(0.0f64, |m, w| m.max(w.ratio));
        println!("BALANCE skipped={nb}: declined {:?}, raw worst {:?}, worst R151 ratio {r:.3}, over {}", st.declined, st.raw, st.over);
    }
    NO_BALANCE.with(|c| c.set(false));
}

/// The split threshold's two ends (Risks): too eager loses digits through
/// cond(S), too timid merges stiff poles into Pade blocks that square.
#[test]
#[ignore = "measurement: prints the threshold sweep"]
fn split_threshold_sweep() {
    let set = cases(RANDOM_CASES / 5, 1);
    for split in [0.0, 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e8] {
        let st = run_set(&set, split, false);
        let dec: [usize; 2] = [st.declined.iter().map(|d| d[0]).sum(), st.declined.iter().map(|d| d[1]).sum()];
        let w = st.raw.iter().fold(0.0f64, |s, v| s.max(*v));
        let r = st.worst.iter().fold(0.0f64, |m, w| m.max(w.ratio));
        let (xt, xt1) = crystal_ring_error(1e-3, split);
        println!(
            "SWEEP split {split:.0e}: {} cases: declined {} well conditioned, {} not; accepted over R151: {}, worst accepted {w:.2e}, worst R151 ratio {r:.3}; states in merged blocks {}, regrouped {}; crystal ring 1 mohm {xt:.2e} W0 (merged blocks {xt1})",
            st.total, dec[0], dec[1], st.over, st.merged_states, st.regrouped
        );
    }
}

// ---- closed forms --------------------------------------------------------------

fn w0v(x: f64) -> f64 {
    1e-4 + 1e-3 * x.abs()
}
fn w0i(x: f64) -> f64 {
    1e-9 + 1e-3 * x.abs()
}

/// Series RLC, states [vC, iL], `vC' = -iL / C`, `iL' = (vC - R iL) / L`,
/// from vC = 5, iL = 0: the textbook closed form (any damping).
fn rlc(l: f64, c: f64, r: f64) -> (Vec<f64>, impl Fn(f64) -> (f64, f64)) {
    let a = vec![0.0, -1.0 / c, 1.0 / l, -r / l];
    let alpha = r / (2.0 * l);
    let w0sq = 1.0 / (l * c);
    let disc = alpha * alpha - w0sq;
    let f = move |t: f64| {
        // v = 5 e^{-at} (C(t) + a S(t)), S = sin(wd t)/wd (or sinh, or t), v' = -5 e^{-at} w0^2 S
        let (cc, ss) = if disc < 0.0 {
            let wd = (-disc).sqrt();
            ((wd * t).cos(), (wd * t).sin() / wd)
        } else if disc > 0.0 {
            let b = disc.sqrt();
            ((b * t).cosh(), (b * t).sinh() / b)
        } else {
            (1.0, t)
        };
        let e = (-alpha * t).exp();
        let v = 5.0 * e * (cc + alpha * ss);
        let dv = -5.0 * e * w0sq * ss;
        (v, -c * dv)
    };
    (a, f)
}

/// Worst error in W0 of (one step to each sample, 4096 chained steps) over
/// [0, life] against the closed form.
fn ring_check(a: &[f64], x0: &[f64], f: &dyn Fn(f64) -> (f64, f64), life: f64) -> (f64, f64, Vec<Block>) {
    let md = decompose(a, &[], 2, 0).unwrap();
    let mut ws = md.workspace();
    let mut x = vec![0.0; 2];
    let mut one = 0.0f64;
    for k in 1..=997 {
        let t = life * k as f64 / 997.0;
        md.step(x0, &[], &[], t, &mut ws, &mut x).unwrap();
        let (v, i) = f(t);
        one = one.max((x[0] - v).abs() / w0v(v)).max((x[1] - i).abs() / w0i(i));
    }
    let mut chain = 0.0f64;
    let mut xc = x0.to_vec();
    let dt = life / 4096.0;
    for k in 1..=4096 {
        md.step(&xc.clone(), &[], &[], dt, &mut ws, &mut xc).unwrap();
        let (v, i) = f(dt * k as f64);
        chain = chain.max((xc[0] - v).abs() / w0v(v)).max((xc[1] - i).abs() / w0i(i));
    }
    (one, chain, md.blocks().to_vec())
}

#[test]
fn closed_form_series_rlc() {
    let w0 = 2.0 * std::f64::consts::PI * 10e3;
    let l = 1e-3;
    let c = 1.0 / (w0 * w0 * l);
    for q in [3.0, 30.0, 300.0, 3e3, 3e4] {
        let r = w0 * l / q;
        let (a, f) = rlc(l, c, r);
        let life = (5.0f64 / 1e-4).ln() / (r / (2.0 * l)); // envelope down to 100 uV
        let (one, chain, blocks) = ring_check(&a, &[5.0, 0.0], &f, life);
        println!("RLC Q {q:>7}: life {life:.4e} s ({:.0} periods): one step worst {one:.2e} W0, 4096 steps worst {chain:.2e} W0; blocks {:?}", life * 10e3, blocks.iter().map(|b| b.kind).collect::<Vec<_>>());
        assert!(one < 1e-3 && chain < 1e-3);
    }
}

#[test]
fn closed_form_critical_damping() {
    let (l, c) = (1e-3f64, 1e-6f64);
    let rc = 2.0 * (l / c).sqrt();
    for (label, r) in [("critical", rc), ("critical x (1 + 1e-4)", rc * (1.0 + 1e-4)), ("critical x (1 - 1e-4)", rc * (1.0 - 1e-4))] {
        let (a, f) = rlc(l, c, r);
        let life = 25.0 / (r / (2.0 * l));
        let (one, chain, blocks) = ring_check(&a, &[5.0, 0.0], &f, life);
        println!("{label:24}: one step worst {one:.2e} W0, 4096 steps worst {chain:.2e} W0; blocks {:?}", blocks.iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>());
        assert!(one < 1e-3 && chain < 1e-3);
    }
}

#[test]
fn closed_form_two_equal_tanks() {
    let (l, c, r) = (1e-3, 1e-6, 1.0);
    let (a1, f) = rlc(l, c, r);
    let mut a = vec![0.0; 16];
    for i in 0..2 {
        for j in 0..2 {
            a[i * 4 + j] = a1[i * 2 + j];
            a[(i + 2) * 4 + j + 2] = a1[i * 2 + j];
        }
    }
    let md = decompose(&a, &[], 4, 0).unwrap();
    let mut ws = md.workspace();
    let x0 = [5.0, 0.0, 2.0, 0.0];
    let mut x = vec![0.0; 4];
    let mut worst = 0.0f64;
    for k in 1..=2000 {
        let t = k as f64 * 1.37e-5;
        md.step(&x0, &[], &[], t, &mut ws, &mut x).unwrap();
        let (v, i) = f(t);
        worst = worst.max((x[0] - v).abs() / w0v(v)).max((x[1] - i).abs() / w0i(i));
        worst = worst.max((x[2] - 0.4 * v).abs() / w0v(0.4 * v)).max((x[3] - 0.4 * i).abs() / w0i(0.4 * i));
    }
    println!("two equal tanks: worst {worst:.2e} W0; blocks {:?}", md.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>());
    assert!(worst < 1e-3);
}

/// The crystal skeleton (design m_xtal): states [vCs, iL, vC0], 32.768 kHz,
/// rs 50, C0 5 pF behind `r1` from a source `vs`.
fn xtal(q: f64, r1: f64) -> (Vec<f64>, Vec<f64>, f64, f64) {
    let w = 2.0 * std::f64::consts::PI * 32768.0;
    let rs = 50.0;
    let ls = q * rs / w;
    let cs = 1.0 / (w * w * ls);
    let c0 = 5e-12;
    let a = vec![0.0, 1.0 / cs, 0.0, -1.0 / ls, -rs / ls, 1.0 / ls, 0.0, -1.0 / c0, -1.0 / (r1 * c0)];
    let b = vec![0.0, 0.0, 1.0 / (r1 * c0)];
    (a, b, ls, cs)
}

/// The released ring (vCs = 5, behind `r1`, source 0) against the textbook
/// series RLC (R = rs + r1, L = ls, C = cs) over its life: worst W0 of vCs
/// and iL, one step to each sample and 4096 chained steps.
fn crystal_ring_error(r1: f64, split: f64) -> (f64, usize) {
    let (a, b, ls, cs) = xtal(5e4, r1);
    let md = match decompose_with(&a, &b, 3, 1, split) {
        Ok(m) => m,
        Err(_) => return (f64::INFINITY, 0),
    };
    let merged = md.blocks().iter().filter(|b| b.kind == BlockKind::Merged).count();
    let r = 50.0 + r1;
    let alpha = r / (2.0 * ls);
    let w0sq = 1.0 / (ls * cs);
    let wd = (w0sq - alpha * alpha).sqrt();
    let f = |t: f64| {
        let e = (-alpha * t).exp();
        let v = 5.0 * e * ((wd * t).cos() + alpha / wd * (wd * t).sin());
        let i = -5.0 * cs * w0sq / wd * e * (wd * t).sin();
        (v, i)
    };
    let life = 3.6917;
    let mut ws = md.workspace();
    let mut x = vec![0.0; 3];
    let mut worst = 0.0f64;
    let x0 = [5.0, 0.0, 0.0];
    for k in 1..=1500 {
        let t = life * k as f64 / 1500.0;
        if md.step(&x0, &[0.0], &[0.0], t, &mut ws, &mut x).is_err() {
            return (f64::INFINITY, merged);
        }
        let (v, i) = f(t);
        worst = worst.max((x[0] - v).abs() / w0v(v)).max((x[1] - i).abs() / w0i(i));
    }
    let mut xc = x0.to_vec();
    let dt = life / 4096.0;
    for k in 1..=4096 {
        if md.step(&xc.clone(), &[0.0], &[0.0], dt, &mut ws, &mut xc).is_err() {
            return (f64::INFINITY, merged);
        }
        let (v, i) = f(dt * k as f64);
        worst = worst.max((xc[0] - v).abs() / w0v(v)).max((xc[1] - i).abs() / w0i(i));
    }
    (worst, merged)
}

#[test]
fn closed_form_crystal_ring() {
    for r1 in [1.0, 1e-3] {
        let (w, merged) = crystal_ring_error(r1, SPLIT_NORM_MAX);
        let (a, b, _, _) = xtal(5e4, r1);
        let md = decompose(&a, &b, 3, 1).unwrap();
        println!("crystal ring behind {r1:e} ohm: worst {w:.2e} W0 against the textbook series RLC (bound 1e-2); blocks {:?}, merged {merged}, cond(S) {:.2e} (balanced {:.2e})", md.blocks().iter().map(|b| b.kind).collect::<Vec<_>>(), md.cond(), md.cond_balanced());
        assert!(w < 1e-2, "{w:e}");
    }
}

/// 5 ns pole (1 k into 5 pF), 30.5 us period, 48 ms decay (Q 5000) in one
/// matrix, driven by 5 V: against the oracle, one step and 4096 steps.
#[test]
fn closed_form_three_scales() {
    let (a, b, _, _) = xtal(5000.0, 1000.0);
    {
        let mdu = decompose_uncertified(&a, &b, 3, 1);
        let mdc = group_and_finish_probe(&a, &b);
        println!("three scales: uncertified {:?}; floors (balanced, caller) {mdc:?}", mdu.as_ref().map(|m| m.rounding_floor()));
    }
    let md = decompose(&a, &b, 3, 1).unwrap();
    let mut ws = md.workspace();
    let mut x = vec![0.0; 3];
    let x0 = [0.0; 3];
    let mut worst = 0.0f64;
    let tol = |k: usize, v: f64| if k == 1 { w0i(v) } else { w0v(v) };
    for t in [1e-9, 5e-9, 3.05e-5, 1e-3, 0.048, 0.25] {
        md.step(&x0, &[5.0], &[0.0], t, &mut ws, &mut x).unwrap();
        let (want, _) = oracle(&a, &b, 3, 1, &x0, &[5.0], &[0.0], t);
        for k in 0..3 {
            worst = worst.max((x[k] - want[k]).abs() / tol(k, want[k]));
        }
    }
    let t_end = 0.25;
    let mut xc = x0.to_vec();
    for _ in 0..4096 {
        md.step(&xc.clone(), &[5.0], &[0.0], t_end / 4096.0, &mut ws, &mut xc).unwrap();
    }
    let (want, _) = oracle(&a, &b, 3, 1, &x0, &[5.0], &[0.0], t_end);
    let chain = (0..3).map(|k| (xc[k] - want[k]).abs() / tol(k, want[k])).fold(0.0f64, f64::max);
    println!("5 ns + 30.5 us + 48 ms: one step worst {worst:.2e} W0, 4096 steps {chain:.2e} W0 against the oracle; blocks {:?}", md.blocks().iter().map(|b| b.kind).collect::<Vec<_>>());
    assert!(worst < 1e-3 && chain < 1e-3);
}

/// RL and RLC under a ramped source against their closed forms.
#[test]
fn closed_form_ramps() {
    // RL: L di/dt = k t - R i.
    let (l, r, k) = (10e-3, 105.0, 5e6);
    let md = decompose(&[-r / l], &[1.0 / l], 1, 1).unwrap();
    let mut ws = md.workspace();
    let mut x = vec![0.0];
    let tau = l / r;
    let mut worst = 0.0f64;
    for j in 1..=500 {
        let t = 1e-6 * j as f64 / 50.0;
        md.at(&[0.0], &[0.0], &[k], t, &mut ws, &mut x).unwrap();
        let want = k / r * (t + tau * (-t / tau).exp_m1());
        worst = worst.max((x[0] - want).abs() / w0i(want));
    }
    println!("RL under a ramp: worst {worst:.2e} W0");
    assert!(worst < 1e-3);
    // Series RLC driven by vs = k t, states [vC, iL]: vC' = iL / C, iL' = (vs - vC - R iL) / L.
    let (l, c, r, k) = (1e-3f64, 1e-6f64, 10.0f64, 1e4f64);
    let a = [0.0, 1.0 / c, -1.0 / l, -r / l];
    let md = decompose(&a, &[0.0, 1.0 / l], 2, 1).unwrap();
    let mut ws = md.workspace();
    let mut x = vec![0.0; 2];
    let alpha = r / (2.0 * l);
    let wd = (1.0 / (l * c) - alpha * alpha).sqrt();
    let (aa, bb) = (k * r * c, (alpha * k * r * c - k) / wd);
    let mut worst = 0.0f64;
    for j in 1..=500 {
        let t = 2e-3 * j as f64 / 500.0;
        md.at(&[0.0, 0.0], &[0.0], &[k], t, &mut ws, &mut x).unwrap();
        let e = (-alpha * t).exp();
        let (cs, sn) = ((wd * t).cos(), (wd * t).sin());
        let v = k * t - k * r * c + e * (aa * cs + bb * sn);
        let dv = k + e * ((-alpha * aa + wd * bb) * cs + (-alpha * bb - wd * aa) * sn);
        worst = worst.max((x[0] - v).abs() / w0v(v)).max((x[1] - c * dv).abs() / w0i(c * dv));
    }
    println!("RLC under a ramp: worst {worst:.2e} W0");
    assert!(worst < 1e-3);
}

// ---- adversarial ------------------------------------------------------------------

/// One adversarial matrix: the step against the oracle, with the answer's
/// measured sensitivity `kappa_fd`. `Ok((relative error, kappa))`, or the
/// decline with `kappa`.
fn adversarial_check(name: &str, a: &[f64], n: usize, h: f64) -> Result<(f64, f64), (ModalError, f64)> {
    let b: Vec<f64> = (0..n).map(|i| 1.0 / (1.0 + i as f64)).collect();
    let x0: Vec<f64> = (0..n).map(|i| (i as f64 * 0.7).sin() + 0.5).collect();
    let case = Case { n, m: 1, a: a.to_vec(), b: b.clone(), x0: x0.clone(), u0: vec![1.0], du: vec![0.3 / h], h, class: Class::Gauss };
    let kf = kappa_fd(&case, &mut Rng(n as u64 + 3000));
    let md = match decompose(a, &b, n, 1) {
        Ok(md) => md,
        Err(e) => {
            println!("adversarial {name:34}: declined {e:?}; kappa_fd {kf:.2e}");
            return Err((e, kf));
        }
    };
    let mut ws = md.workspace();
    let mut x = vec![0.0; n];
    md.step(&x0, &[1.0], &[0.3 / h], h, &mut ws, &mut x).map_err(|e| (e, kf))?;
    let (want, rf) = oracle(a, &b, n, 1, &x0, &[1.0], &[0.3 / h], h);
    let rel = x.iter().zip(&want).map(|(p, q)| (p - q).abs()).fold(0.0f64, f64::max) / rf;
    println!(
        "adversarial {name:34}: relative {rel:.2e}, kappa_fd {kf:.2e}, error / R151 allowance {:.3}; blocks {:?}; cond(S) {:.2e}, floor {:.1e}",
        rel / r151_allowed(kf),
        md.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>(),
        md.cond(),
        md.rounding_floor()
    );
    Ok((rel, kf))
}

/// Adversarial matrices.
///
/// Restated by R151 (coordinator ruling, confirmed by the owner 2026-11-24).
/// Old bound: 1e-12 relative on every step the kernel accepts. Replaced
/// because the released crystal behind 1 mohm stepped 0.37 s carries a
/// 2e5 rad/s ring through |z| = 7.6e4 radians, and its exact answer moves by
/// that many eps for a one-ulp change of the data (`kappa_fd`; EX1 measured
/// the step 9.7e-12 off, eps |z|). Criterion: within `R151_ULPS eps
/// kappa_fd`, and `R151_FLOOR` wherever that is smaller; a decline on a
/// member with `kappa_fd` below `WELL_CONDITIONED_KAPPA` is a failure, and
/// the zero matrix, the integrator, the crystal and the order-2 Jordan
/// block must be accepted whatever their sensitivity.
#[test]
fn adversarial_sets() {
    let mut results = Vec::new();
    // Jordan blocks of order 2..6, real and at a complex pair.
    for n in 2..=6 {
        let mut a = vec![0.0; n * n];
        for i in 0..n {
            a[i * n + i] = -3.0;
            if i + 1 < n {
                a[i * n + i + 1] = 1.0;
            }
        }
        results.push((format!("Jordan order {n} at -3"), adversarial_check(&format!("Jordan order {n} at -3"), &a, n, 0.7)));
    }
    let mut a = vec![0.0; 16];
    let blk = [-1.0, 5.0, -5.0, -1.0];
    for r in 0..2 {
        for c in 0..2 {
            a[r * 4 + c] = blk[r * 2 + c];
            a[(r + 2) * 4 + c + 2] = blk[r * 2 + c];
        }
        a[r * 4 + r + 2] = 1.0;
    }
    results.push(("Jordan pair (double complex)".into(), adversarial_check("Jordan pair (double complex)", &a, 4, 0.9)));
    // Kahan matrices (upper triangular, eigenvalues s^i), mirrored to decay.
    for n in [6usize, 9, 12] {
        let th: f64 = 1.2;
        let (s, c) = (th.sin(), th.cos());
        let mut a = vec![0.0; n * n];
        for i in 0..n {
            let si = s.powi(i as i32);
            a[i * n + i] = -si;
            for j in (i + 1)..n {
                a[i * n + j] = c * si;
            }
        }
        results.push((format!("Kahan order {n}"), adversarial_check(&format!("Kahan order {n}"), &a, n, 1.0)));
    }
    results.push(("zero matrix 4".into(), adversarial_check("zero matrix 4", &[0.0; 16], 4, 2.0)));
    // An integrator: x1' = x2, x2' = -x2 + u (a zero eigenvalue).
    results.push(("integrator".into(), adversarial_check("integrator", &[0.0, 1.0, 0.0, -1.0], 2, 3.0)));
    // The released crystal (2e14 /s beside 2e5 rad/s) as a plain matrix.
    let (a, _, _, _) = xtal(5e4, 1e-3);
    results.push(("crystal behind 1 mohm".into(), adversarial_check("crystal behind 1 mohm", &a, 3, 0.37)));
    let (mut over, mut well_declined) = (0, 0);
    for (name, r) in &results {
        match r {
            Ok((rel, kf)) if *rel > r151_allowed(*kf) => {
                over += 1;
                println!("  OVER {name}: {rel:e} (kappa_fd {kf:e})");
            }
            Ok(_) => {}
            Err((e, kf)) => {
                if *kf < WELL_CONDITIONED_KAPPA {
                    well_declined += 1;
                }
                println!("  {name}: declined {e:?}, kappa_fd {kf:.2e}");
            }
        }
    }
    assert_eq!(over, 0, "an accepted step over R151's allowance");
    assert_eq!(well_declined, 0, "a decline on a well-conditioned member");
    for (name, r) in &results {
        if name.starts_with("zero") || name.starts_with("integrator") || name.starts_with("crystal") || name.starts_with("Jordan order 2") {
            assert!(r.is_ok(), "{name}: {r:?}");
        }
    }
}

#[test]
fn terms_reproduce_the_step() {
    // RLC with a ramp: f(t) from terms equals c x(t) from the step.
    let (l, c, r) = (1e-3f64, 1e-6f64, 10.0f64);
    let a = [0.0, 1.0 / c, -1.0 / l, -r / l];
    let b = [0.0, 1.0 / l];
    let md = decompose(&a, &b, 2, 1).unwrap();
    let mut ws = md.workspace();
    let mut tb = md.terms_buffer();
    let x0 = [0.3, -1e-3];
    let (u0, du) = ([2.0], [1e3]);
    let cr = [1.0, 50.0];
    md.terms(&cr, &x0, &u0, &du, &mut tb).unwrap();
    let mut x = vec![0.0; 2];
    let mut worst = 0.0f64;
    for k in 0..200 {
        let t = 3e-3 * k as f64 / 200.0;
        md.at(&x0, &u0, &du, t, &mut ws, &mut x).unwrap();
        let want = cr[0] * x[0] + cr[1] * x[1];
        let mut f = tb.c0 + tb.q * t;
        for j in 0..tb.len {
            let (wr, wi) = tb.w[j];
            let (lr, li) = tb.lambda[j];
            let e = (lr * t).exp();
            f += e * (wr * (li * t).cos() - wi * (li * t).sin());
        }
        worst = worst.max((f - want).abs() / (1.0 + want.abs()));
    }
    println!("terms vs step: worst {worst:.2e}");
    assert!(worst < 1e-12);
    // A merged block is not expressible.
    let md = decompose(&[-3.0, 1.0, 0.0, -3.0], &[], 2, 0).unwrap();
    let mut tb = md.terms_buffer();
    assert_eq!(md.terms(&[1.0, 0.0], &[1.0, 1.0], &[], &[], &mut tb), Err(ModalError::Unsupported));
}

#[test]
fn shapes_and_non_finite_inputs_are_errors() {
    assert_eq!(decompose(&[], &[], 0, 0).err(), Some(ModalError::Shape));
    assert_eq!(decompose(&vec![0.0; 33 * 33], &[], 33, 0).err(), Some(ModalError::Shape));
    assert_eq!(decompose(&[f64::NAN], &[], 1, 0).err(), Some(ModalError::NonFinite));
    let md = decompose(&[-1.0], &[1.0], 1, 1).unwrap();
    let mut ws = md.workspace();
    let mut x = [0.0];
    assert_eq!(md.step(&[1.0], &[1.0], &[0.0], -1.0, &mut ws, &mut x), Err(ModalError::Shape));
    assert_eq!(md.step(&[1.0], &[1.0], &[0.0], f64::INFINITY, &mut ws, &mut x), Err(ModalError::Shape));
    // A growing mode that overflows is an error, not a panic.
    let md = decompose(&[1.0], &[], 1, 0).unwrap();
    let mut ws = md.workspace();
    assert_eq!(md.step(&[1.0], &[], &[], 1e4, &mut ws, &mut x), Err(ModalError::NonFinite));
    // Huge finite entries: no panic, whatever the result.
    let big = [1e300, -1e300, 1e300, 1e-300];
    let _ = decompose(&big, &[], 2, 0);
    let mut rng = Rng(99);
    for _ in 0..200 {
        let n = 1 + rng.below(32);
        let a: Vec<f64> = (0..n * n).map(|_| rng.normal() * rng.logu(-300.0, 300.0)).collect();
        if let Ok(md) = decompose(&a, &[], n, 0) {
            let mut ws = md.workspace();
            let mut x = vec![0.0; n];
            let x0 = vec![1.0; n];
            let _ = md.step(&x0, &[], &[], rng.logu(-300.0, 300.0), &mut ws, &mut x);
        }
    }
}

/// Instructions of one `decompose` (certified, and `core`: the factorisation
/// without the certification), and one `step`, at n = 3, 8, 16, 32: run once
/// per process with MODAL_COST_N, MODAL_COST_MODE (decompose | core | step |
/// base), MODAL_COST_KIND (ladder: distinct rates and pairs; jordan: with
/// repeated eigenvalues, so merged blocks and the certification's probes)
/// and MODAL_COST_REPS, under `/usr/bin/time -l`; the cost is the difference
/// of two repetition counts divided by their difference.
#[test]
#[ignore = "cost run: one process per measurement"]
fn modal_cost() {
    let n: usize = std::env::var("MODAL_COST_N").ok().and_then(|v| v.parse().ok()).unwrap_or(3);
    let mode = std::env::var("MODAL_COST_MODE").unwrap_or_else(|_| "base".into());
    let reps: usize = std::env::var("MODAL_COST_REPS").ok().and_then(|v| v.parse().ok()).unwrap_or(100);
    // A ladder of n states: alternating L and C sections with resistive loss,
    // the shape a coil partition has (distinct complex pairs and real rates).
    let mut rng = Rng(7);
    let jordan = std::env::var("MODAL_COST_KIND").map(|k| k == "jordan").unwrap_or(false);
    let (t, _, _) = draw_t(&mut rng, n, jordan);
    let q = random_orthogonal(&mut rng, n);
    let a = matmul_n(&matmul_n(&q, &t, n), &transpose(&q, n), n);
    let b: Vec<f64> = (0..n).map(|_| rng.normal()).collect();
    let mut acc = 0.0;
    match mode.as_str() {
        "decompose" => {
            for k in 0..reps {
                let mut a2 = a.clone();
                a2[0] += k as f64 * 1e-300;
                let md = decompose(&a2, &b, n, 1).unwrap();
                acc += md.cond();
            }
        }
        "core" => {
            for k in 0..reps {
                let mut a2 = a.clone();
                a2[0] += k as f64 * 1e-300;
                let md = decompose_uncertified(&a2, &b, n, 1).unwrap();
                acc += md.cond();
            }
        }
        "step" => {
            let md = decompose(&a, &b, n, 1).unwrap();
            let mut ws = md.workspace();
            let mut x = vec![0.0; n];
            let x0 = vec![1.0; n];
            for k in 0..reps {
                md.step(&x0, &[1.0], &[0.5], 1e-3 * (1.0 + k as f64 * 1e-9), &mut ws, &mut x).unwrap();
                acc += x[0];
            }
            println!("blocks {:?}", md.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>());
        }
        _ => {
            let md = decompose(&a, &b, n, 1).unwrap();
            acc += md.cond();
            println!("blocks {:?} split {:.0e} floor {:.1e}", md.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>(), md.split_threshold(), md.rounding_floor());
        }
    }
    println!("COST n {n} mode {mode} reps {reps} acc {acc:e}");
}


#[test]
#[ignore = "measurement (EX1B): the uncertified step against R151's allowance, with the rounding floor"]
fn measure_r151() {
    let set = cases(RANDOM_CASES, 1);
    let names = ["ortho", "jordan", "skewed", "gauss"];
    let mut worst = [0.0f64; 4];
    let mut errs = [0usize; 4];
    let mut lines = Vec::new();
    for (idx, c) in set.iter().enumerate() {
        let ci = c.class as usize;
        let md = match decompose_core(&c.a, &c.b, c.n, c.m, SPLIT_NORM_MAX, true) {
            Ok(m) => m,
            Err(e) => {
                errs[ci] += 1;
                let kf = kappa_fd(c, &mut Rng(c.n as u64 + 2000));
                lines.push(format!("ERR {idx} {:?} {e:?} kappa {kf:.2e}", c.class));
                continue;
            }
        };
        let mut ws = md.workspace();
        let mut x = vec![0.0; c.n];
        if md.at(&c.x0, &c.u0, &c.du, c.h, &mut ws, &mut x).is_err() {
            errs[ci] += 1;
            continue;
        }
        let (want, rf) = oracle(&c.a, &c.b, c.n, c.m, &c.x0, &c.u0, &c.du, c.h);
        let rel = x.iter().zip(&want).map(|(a, b)| (a - b).abs()).fold(0.0f64, f64::max) / rf.max(f64::MIN_POSITIVE);
        let r0 = md.rounding_floor();
        let sq = ws.bsq.iter().copied().max().unwrap_or(0);
        if rel > 2e-13 || r0 > 1e-13 || md.split_threshold() < SPLIT_NORM_MAX {
            let kf = kappa_fd(c, &mut Rng(c.n as u64 + 1000));
            let allowed = (100.0 * f64::EPSILON * kf).max(1e-12);
            worst[ci] = worst[ci].max(rel / allowed);
            lines.push(format!(
                "{idx} {:?} n {} rel {rel:.2e} kappa {kf:.2e} ratio {:.3} r0 {r0:.2e} split {:.0e} cond {:.2e} condbal {:.2e} sq {sq} blocks {:?}",
                c.class,
                c.n,
                rel / allowed,
                md.split_threshold(),
                md.cond(),
                md.cond_balanced(),
                md.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>()
            ));
        } else {
            worst[ci] = worst[ci].max(rel / 1e-12);
        }
    }
    for l in &lines {
        println!("M {l}");
    }
    for k in 0..4 {
        println!("MEASURE {}: worst ratio {:.3}, errors {}", names[k], worst[k], errs[k]);
    }
}

/// The balanced and the caller-coordinate rounding floors of a decomposition
/// at the default threshold (EX1B diagnostic).
fn group_and_finish_probe(a: &[f64], b: &[f64]) -> Option<(f64, f64)> {
    let md = decompose_core(a, b, 3, 1, SPLIT_NORM_MAX, false).ok()?;
    Some((md.rounding_floor(), rounding_floor(md.s(), md.s_inv(), 3)))
}

#[test]
#[ignore = "debug (EX1B): one random case's decomposition at each threshold"]
fn debug_case() {
    let idx: usize = std::env::var("MODAL_CASE").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    let set = cases(idx + 1, 1);
    let c = &set[idx];
    println!("CASE {idx} {:?} n {} m {} h {:.3e}", c.class, c.n, c.m, c.h);
    for sp in [1e2, 1e1, 1e0] {
        match decompose_core(&c.a, &c.b, c.n, c.m, sp, false) {
            Ok(m) => println!("  split {sp:e}: floor {:.2e} cond_bal {:.2e} resid {:.2e} blocks {:?}", m.rounding_floor(), m.cond_balanced(), m.residual(), m.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>()),
            Err(e) => println!("  split {sp:e}: {e:?}"),
        }
    }
    match decompose_uncertified(&c.a, &c.b, c.n, c.m) {
        Ok(m) => {
            CERT_TRACE.with(|t| t.set(true));
            let r = certify(&m, &c.a, &c.b);
            CERT_TRACE.with(|t| t.set(false));
            println!("  uncertified ok, split {:e}; certify {:?}", m.split_threshold(), r)
        }
        Err(e) => println!("  uncertified {e:?}"),
    }
}

#[test]
#[ignore = "debugging: certify's probe ladder against the oracle for MODAL_CASE"]
fn debug_probe() {
    let idx: usize = std::env::var("MODAL_CASE").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    let set = cases(idx + 1, 1);
    let c = &set[idx];
    let md = decompose_uncertified(&c.a, &c.b, c.n, c.m).unwrap();
    let mut ws = md.workspace();
    let (mut x, mut x2) = (vec![0.0; c.n], vec![0.0; c.n]);
    // MODAL_PROBE_CERT=1: certify's own probe vectors (its splitmix stream
    // after the shadows' draws).
    let mut st: u64 = 0x2545_F491_4F6C_DD1D;
    let mut rnd = move || {
        st = st.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = st;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        (z >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    };
    for _ in 0..SCE_SAMPLES * (c.n * c.n + c.n * c.m) {
        rnd();
    }
    let certv = std::env::var("MODAL_PROBE_CERT").is_ok();
    let mut c = Case { n: c.n, m: c.m, a: c.a.clone(), b: c.b.clone(), x0: c.x0.clone(), u0: c.u0.clone(), du: c.du.clone(), h: c.h, class: c.class };
    if certv {
        c.x0 = (0..c.n).map(|_| rnd()).collect();
        c.u0 = (0..c.m).map(|_| rnd()).collect();
        c.du = (0..c.m).map(|_| rnd() / c.h).collect();
    }
    let c = &c;
    for (bi, blk) in md.blocks().iter().enumerate() {
        if blk.kind != BlockKind::Merged {
            continue;
        }
        let (re_max, dn) = (md.bdesc[bi].0, md.bdesc[bi].3);
        for k in 0..=CERT_SQ_MAX {
            let h = THETA13 * 2f64.powi(k) / dn;
            let du: Vec<f64> = c.du.iter().map(|d| d * c.h / h).collect();
            md.at(&c.x0, &c.u0, &du, h, &mut ws, &mut x).unwrap();
            md.at_scaled(&c.x0, &c.u0, &du, h, &mut ws, &mut x2, 1).unwrap();
            let (want, rf) = oracle(&c.a, &c.b, c.n, c.m, &c.x0, &c.u0, &du, h);
            let reff = inf_norm(&c.x0).max(inf_norm(&x));
            let err = x.iter().zip(&want).map(|(a, b)| (a - b).abs()).fold(0.0f64, f64::max) / rf;
            let err2 = x2.iter().zip(&want).map(|(a, b)| (a - b).abs()).fold(0.0f64, f64::max) / rf;
            let est = 2.0 * x2.iter().zip(&x).map(|(a, b)| (a - b).abs()).fold(0.0f64, f64::max) / reff;
            let cc = Case { n: c.n, m: c.m, a: c.a.clone(), b: c.b.clone(), x0: c.x0.clone(), u0: c.u0.clone(), du: du.clone(), h, class: c.class };
            let kf = kappa_fd(&cc, &mut Rng(7));
            println!("k {k} h {h:.3e} s {:?} err {err:.2e} err(+1 sq) {err2:.2e} est {est:.2e} kappa_fd {kf:.2e} allow {:.2e} rf {rf:.2e} reff {reff:.2e} re_max*h {:.2e}", ws.bsq, r151_allowed(kf), re_max * h);
            if re_max * h < -39.0 {
                break;
            }
        }
    }
}
