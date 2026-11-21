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
#[test]
fn schur_preserves_the_exponential() {
    let set = cases(RANDOM_CASES / 10, 12);
    let mut worst = 0.0f64;
    for c in &set {
        let n = c.n;
        let mut h = c.a.clone();
        let mut q = vec![0.0; n * n];
        hessenberg(&mut h, &mut q, n).unwrap();
        schur(&mut h, &mut q, n).unwrap();
        let ea = dd_exp_f(&c.a, n, c.h);
        let et = dd_exp_f(&h, n, c.h);
        let back = matmul_n(&matmul_n(&q, &et, n), &transpose(&q, n), n);
        let e = diff_norm1(&back, &ea, n) / norm1(&ea, n).max(1e-300);
        worst = worst.max(e);
    }
    println!("K2 vs oracle: |Q e^(Th) Q^T - e^(Ah)|_1 / |e^(Ah)|_1 worst {worst:.2e} over {}", set.len());
    assert!(worst < 1e-11, "{worst:e}");
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
        for (fk, g) in got.iter().enumerate() {
            let want = [e[fk * 2], e[fk * 2 + 1], e[6 + fk * 2], e[6 + fk * 2 + 1]];
            let scale = want.iter().fold(0.0f64, |s, v| s.max(v.abs())).max(f64::MIN_POSITIVE);
            let err = (0..4).map(|i| (g[i] - want[i]).abs()).fold(0.0f64, f64::max) / scale;
            worst2 = worst2.max(err);
            // Diagnostic: the error in units of the block's own condition
            // (a rounding of b or c moves the result by about eps |z| nn).
            worst2u = worst2u.max(err / (f64::EPSILON * (1.0 + mz * nn.max(1.0 / nn))));
            if nn.max(1.0 / nn) < 2.0 && mz <= 1e3 {
                worst2n = worst2n.max(err);
            }
            assert!(err < 1e-13, "order 2 z {sr:e}+{si:e}i nn {nn:e} f{fk}: {g:?} vs {want:?} ({err:e})");
        }
    }
    // Decayed blocks are exactly 0 past underflow.
    assert_eq!(real_exp_phi(-800.0)[0], 0.0);
    assert_eq!(pair_exp_phi(-800.0, 3.0)[0], (0.0, 0.0));
    println!("K5 closed forms vs oracle: order 1 worst {worst1:.2e} relative; order 2 worst {worst2:.2e} relative to the block norm ({worst2u:.2} units of eps (1 + |z| nn)), near-normal blocks with |z| <= 1e3 worst {worst2n:.2e}");
    assert!(worst2n < 1e-12);
}

// ---- K6 ---------------------------------------------------------------------------

#[test]
fn pade_against_dd() {
    let set = cases(RANDOM_CASES / 5, 31);
    let mut buckets = [(0usize, 0.0f64); 6];
    let mut w = PadeWork::new(12);
    for c in &set {
        let n = c.n;
        for i in 0..n * n {
            w.m[i] = c.a[i] * c.h;
        }
        let norm = norm1(&w.m[..n * n], n);
        let s = if norm > THETA13 { (norm / THETA13).log2().ceil() as usize } else { 0 };
        if pade_exp(&mut w, n).is_err() {
            continue;
        }
        let want = dd_exp_f(&c.a, n, c.h);
        let e = diff_norm1(&w.m[..n * n], &want, n) / norm1(&want, n).max(1e-300);
        let bk = [0, 1, 5, 10, 20, 40].iter().rposition(|&lim| s >= lim).unwrap();
        buckets[bk].0 += 1;
        buckets[bk].1 = buckets[bk].1.max(e);
    }
    for (k, lim) in ["0", "1-4", "5-9", "10-19", "20-39", ">=40"].iter().enumerate() {
        println!("K6 Pade (f64) vs oracle, squarings {lim:>5}: {:5} matrices, worst {:.2e} relative (1-norm)", buckets[k].0, buckets[k].1);
    }
    // The gate's claim: within 1e-14 relative while at most 9 squarings.
    assert!(buckets[0].1 < 1e-14 && buckets[1].1 < 1e-14 && buckets[2].1 < 1e-14, "{buckets:?}");
}

// ---- K4 / K7: the random set ---------------------------------------------------

struct SetStats {
    total: usize,
    errs: [usize; 4],
    counts: [usize; 4],
    worst: [f64; 4],
    silent: usize,
    worst_rt: f64,
    worst_comp: f64,
    cond_hist: Vec<(f64, f64)>,
    notes: Vec<String>,
    declined: [usize; 4],
    declined_well: usize,
    /// Declines whose measured condition kappa_fd is at most 30, 100, 1e3.
    declined_by: [usize; 3],
    tight: f64,
    tight_post: f64,
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

/// The largest |lambda h| over the blocks still alive at h (e^Re(z) > 1e-17):
/// the angle a step must carry, whose input rounding no f64 method avoids.
fn live_z(md: &Modal, h: f64) -> f64 {
    let (n, d) = (md.n(), md.d());
    let mut z = 0.0f64;
    for b in md.blocks() {
        for i in b.start..b.start + b.size {
            let re = d[i * n + i] * h;
            let im = if b.kind != BlockKind::Real && i + 1 < b.start + b.size { pair_omega(d[i * n + i + 1], d[(i + 1) * n + i]) * h } else { 0.0 };
            if re > -39.0 {
                z = z.max(re.hypot(im));
            }
        }
    }
    z
}

fn run_set(set: &[Case], split: f64, composition: bool) -> SetStats {
    let mut st = SetStats { total: set.len(), errs: [0; 4], counts: [0; 4], worst: [0.0; 4], silent: 0, worst_rt: 0.0, worst_comp: 0.0, cond_hist: Vec::new(), notes: Vec::new(), declined: [0; 4], declined_well: 0, declined_by: [0; 3], tight: 0.0, tight_post: 0.0 };
    for c in set {
        let ci = c.class as usize;
        st.counts[ci] += 1;
        let md = match decompose_with(&c.a, &c.b, c.n, c.m, split) {
            Ok(md) => md,
            Err(_) => {
                st.errs[ci] += 1;
                continue;
            }
        };
        let n = c.n;
        let mut ws = md.workspace();
        let mut x = vec![0.0; n];
        match md.at_checked(&c.x0, &c.u0, &c.du, c.h, 1e-12, &mut ws, &mut x) {
            Ok(()) => {}
            Err(ModalError::IllConditioned) => {
                // Declined at 1e-12: the kernel's own call. On a case the
                // f64 data leave well conditioned that is a failure.
                st.declined[ci] += 1;
                let kf = kappa_fd(c, &mut Rng(c.n as u64 + 2000));
                for (slot, lim) in [30.0, 100.0, 1e3].iter().enumerate() {
                    if kf <= *lim {
                        st.declined_by[slot] += 1;
                    }
                }
                if kf <= 30.0 {
                    st.declined_well += 1;
                    let _ = md.at(&c.x0, &c.u0, &c.du, c.h, &mut ws, &mut x);
                    let (want, rf) = oracle(&c.a, &c.b, n, c.m, &c.x0, &c.u0, &c.du, c.h);
                    let rel = x.iter().zip(&want).map(|(a, b)| (a - b).abs()).fold(0.0f64, f64::max) / rf;
                    let est = md.estimate(&c.x0, &c.u0, &c.du, c.h, &mut ws, &mut x).unwrap_or(f64::NAN);
                    st.notes.push(format!(
                        "DECLINED but kappa_fd {kf:.1e} <= 30: {:?} n {} rel {rel:.1e} estimate {est:.2e} cond {:.1e} live z {:.1e} blocks {:?}",
                        c.class,
                        n,
                        md.cond(),
                        live_z(&md, c.h),
                        md.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>()
                    ));
                }
                continue;
            }
            Err(_) => {
                st.errs[ci] += 1;
                continue;
            }
        }
        let (want, rf) = oracle(&c.a, &c.b, n, c.m, &c.x0, &c.u0, &c.du, c.h);
        let rel = x.iter().zip(&want).map(|(a, b)| (a - b).abs()).fold(0.0f64, f64::max) / rf.max(f64::MIN_POSITIVE);
        st.worst[ci] = st.worst[ci].max(rel);
        let est = md.estimate(&c.x0, &c.u0, &c.du, c.h, &mut ws, &mut x).unwrap_or(f64::NAN);
        st.tight = st.tight.max(rel / est);
        if rel > 1e-13 {
            st.notes.push(format!("RATIO {:.2e} rel {rel:.2e} est {est:.2e}", rel / est));
        }
        st.cond_hist.push((md.cond_balanced(), rel));
        if rel > 1e-12 {
            st.silent += 1;
            let kf = kappa_fd(c, &mut Rng(c.n as u64 + 1000));
            st.notes.push(format!(
                "{:?} n {} rel {rel:.2e} kappa_fd {kf:.2e} live |z| {:.2e} cond {:.2e} resid {:.2e} blocks {:?}",
                c.class,
                n,
                live_z(&md, c.h),
                md.cond_balanced(),
                md.residual(),
                md.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>()
            ));
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
                }
            }
        }
    }
    st
}

#[test]
fn random_set_against_oracle() {
    let set = cases(RANDOM_CASES, 1);
    let st = run_set(&set, SPLIT_NORM_MAX, true);
    let names = ["ortho", "jordan", "skewed (cond V <= 100)", "gauss"];
    for k in 0..4 {
        println!(
            "K7 random set, {:24}: {:5} cases, error returns {:4} ({:.2} %), declined at 1e-12 {:4} ({:.2} %), worst relative error of the rest {:.2e}",
            names[k],
            st.counts[k],
            st.errs[k],
            100.0 * st.errs[k] as f64 / st.counts[k].max(1) as f64,
            st.declined[k],
            100.0 * st.declined[k] as f64 / st.counts[k].max(1) as f64,
            st.worst[k]
        );
    }
    let mut ch = st.cond_hist.clone();
    ch.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    for lim in [1e1, 1e2, 1e3, 1e4, 1e5, 1e6] {
        let w = ch.iter().filter(|(c, _)| *c <= lim && *c > lim / 10.0).map(|x| x.1).fold(0.0f64, f64::max);
        let k = ch.iter().filter(|(c, _)| *c <= lim && *c > lim / 10.0).count();
        println!("  cond(S) in ({:.0e}, {lim:.0e}]: {k:5} cases, worst {w:.2e}", lim / 10.0);
    }
    println!("K7: Ok results above 1e-12 relative: {}; S S^-1 round trip worst |S S^-1 - I|_1 / cond {:.2e}; step(h1) + step(h2) vs step(h1 + h2) worst {:.2e}", st.silent, st.worst_rt, st.worst_comp);
    for note in st.notes.iter().filter(|n| !n.starts_with("DECLINED")) {
        println!("  above 1e-12: {note}");
    }
    for note in st.notes.iter().filter(|n| n.starts_with("DECLINED")).take(10) {
        println!("  {note}");
    }
    println!(
        "K7: worst error / estimate {:.2e}; declines with kappa_fd <= 30: {}, <= 100: {}, <= 1e3: {}",
        st.tight, st.declined_by[0], st.declined_by[1], st.declined_by[2]
    );
    // Well conditioned: kappa_fd <= 30, an error floor of 7e-15, 150 times
    // under the gate. A decline needs SCE_FACTOR eps kappa > 1e-12, a sampled
    // kappa above 140; nearer the gate the sampling's spread decides.
    assert_eq!(st.declined_well, 0, "a decline on a well-conditioned case is a failure");
    assert_eq!(st.silent, 0, "a step that cannot meet 1e-12 must return its error");
    assert_eq!(st.errs[Class::Ortho as usize], 0, "an error on a well-conditioned matrix is a failure");
    assert!(st.worst_comp < 1e-12, "{:e}", st.worst_comp);
}

#[test]
#[ignore = "measurement: the random set with and without balancing"]
fn balance_ab() {
    let set = cases(RANDOM_CASES, 1);
    for nb in [false, true] {
        NO_BALANCE.with(|c| c.set(nb));
        let st = run_set(&set, SPLIT_NORM_MAX, false);
        println!("BALANCE skipped={nb}: errs {:?} worst {:?} above 1e-12 {}", st.errs, st.worst, st.silent);
        for note in st.notes.iter().take(12) {
            println!("  {note}");
        }
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
        let errs: usize = st.errs.iter().sum();
        let dec: usize = st.declined.iter().sum();
        let w = st.worst.iter().fold(0.0f64, |s, v| s.max(*v));
        let (xt, xt1) = crystal_ring_error(1e-3, split);
        let merged: usize = set.iter().filter_map(|c| decompose_with(&c.a, &c.b, c.n, c.m, split).ok()).map(|m| m.blocks().iter().filter(|b| b.kind == BlockKind::Merged).map(|b| b.size).sum::<usize>()).sum();
        println!(
            "SWEEP split {split:.0e}: {} cases: error returns {errs}, declined {dec} (of them kappa_fd < 1e3: {}), accepted above 1e-12: {}, worst accepted {w:.2e}, worst error / estimate {:.2e}; states in merged blocks {merged}; crystal ring 1 mohm {xt:.2e} W0 (merged blocks {xt1})",
            st.total, st.declined_well, st.silent, st.tight
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

fn adversarial_check(name: &str, a: &[f64], n: usize, h: f64) -> Result<f64, ModalError> {
    let b: Vec<f64> = (0..n).map(|i| 1.0 / (1.0 + i as f64)).collect();
    let x0: Vec<f64> = (0..n).map(|i| (i as f64 * 0.7).sin() + 0.5).collect();
    let md = decompose(a, &b, n, 1)?;
    let mut ws = md.workspace();
    let mut x = vec![0.0; n];
    md.step(&x0, &[1.0], &[0.3 / h], h, &mut ws, &mut x)?;
    let (want, rf) = oracle(a, &b, n, 1, &x0, &[1.0], &[0.3 / h], h);
    let rel = x.iter().zip(&want).map(|(p, q)| (p - q).abs()).fold(0.0f64, f64::max) / rf;
    println!("adversarial {name:34}: relative {rel:.2e}; blocks {:?}; cond(S) {:.2e}", md.blocks().iter().map(|b| (b.size, b.kind)).collect::<Vec<_>>(), md.cond());
    Ok(rel)
}

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
    let mut silent = 0;
    for (name, r) in &results {
        match r {
            Ok(rel) if *rel > 1e-12 => {
                silent += 1;
                println!("  SILENT {name}: {rel:e}");
            }
            Ok(_) => {}
            Err(e) => println!("  {name}: error {e:?}"),
        }
    }
    assert_eq!(silent, 0);
    // The well-conditioned members must not return errors.
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

/// Instructions of one `decompose` and one `step` at n = 3, 8, 16, 32: run
/// once per process with MODAL_COST_N, MODAL_COST_MODE (decompose | step |
/// base) and MODAL_COST_REPS, under `/usr/bin/time -l`; the cost is the
/// difference of two repetition counts divided by their difference.
#[test]
#[ignore = "cost run: one process per measurement"]
fn modal_cost() {
    let n: usize = std::env::var("MODAL_COST_N").ok().and_then(|v| v.parse().ok()).unwrap_or(3);
    let mode = std::env::var("MODAL_COST_MODE").unwrap_or_else(|_| "base".into());
    let reps: usize = std::env::var("MODAL_COST_REPS").ok().and_then(|v| v.parse().ok()).unwrap_or(100);
    // A ladder of n states: alternating L and C sections with resistive loss,
    // the shape a coil partition has (distinct complex pairs and real rates).
    let mut rng = Rng(7);
    let (t, _, _) = draw_t(&mut rng, n, false);
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
        }
    }
    println!("COST n {n} mode {mode} reps {reps} acc {acc:e}");
}

