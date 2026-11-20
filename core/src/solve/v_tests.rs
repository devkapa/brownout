//! CAPERR4V: an independent check of the extrapolated TR-BDF2 past the guard.
//! V2: random stiff linear partitions in the engine against the matrix
//! exponential (eigen-decomposition done here), the plain trapezoid beside them.

use crate::engine::Engine;
use crate::program::{Device, Options, PinRecord};
use crate::solve::{kinds, stepped, Program};

pub(crate) struct Rng(pub u64);
impl Rng {
    pub fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn log(&mut self, lo: f64, hi: f64) -> f64 {
        (lo.ln() + self.next() * (hi.ln() - lo.ln())).exp()
    }
    pub fn below(&mut self, n: usize) -> usize {
        ((self.next() * n as f64) as usize).min(n - 1)
    }
}

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

/// Symmetric eigen-decomposition by cyclic Jacobi: `(eigenvalues, eigenvectors as columns, row major)`.
pub(crate) fn jacobi(mut a: Vec<f64>, n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut q = vec![0.0; n * n];
    for i in 0..n {
        q[i * n + i] = 1.0;
    }
    for _ in 0..200 {
        let (mut off, mut diag) = (0.0, 0.0);
        for i in 0..n {
            for j in 0..n {
                if i != j {
                    off += a[i * n + j] * a[i * n + j];
                } else {
                    diag += a[i * n + j] * a[i * n + j];
                }
            }
        }
        if off <= 1e-34 * diag.max(1e-300) {
            break;
        }
        for p in 0..n {
            for r in p + 1..n {
                let apq = a[p * n + r];
                if apq == 0.0 {
                    continue;
                }
                let theta = (a[r * n + r] - a[p * n + p]) / (2.0 * apq);
                let t = if theta == 0.0 { 1.0 } else { theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt()) };
                let (c, s) = (1.0 / (t * t + 1.0).sqrt(), t / (t * t + 1.0).sqrt());
                for k in 0..n {
                    let (akp, akq) = (a[k * n + p], a[k * n + r]);
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + r] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (a[p * n + k], a[r * n + k]);
                    a[p * n + k] = c * apk - s * aqk;
                    a[r * n + k] = s * apk + c * aqk;
                }
                for k in 0..n {
                    let (qkp, qkq) = (q[k * n + p], q[k * n + r]);
                    q[k * n + p] = c * qkp - s * qkq;
                    q[k * n + r] = s * qkp + c * qkq;
                }
            }
        }
    }
    ((0..n).map(|i| a[i * n + i]).collect(), q)
}

/// `X = A^-1 B` by partial-pivot elimination (`A` n x n, `B` n x m).
pub(crate) fn solve(a: &[f64], b: &[f64], n: usize, m: usize) -> Vec<f64> {
    let mut a = a.to_vec();
    let mut b = b.to_vec();
    for k in 0..n {
        let piv = (k..n).max_by(|&i, &j| a[i * n + k].abs().total_cmp(&a[j * n + k].abs())).unwrap();
        if piv != k {
            for c in 0..n {
                a.swap(k * n + c, piv * n + c);
            }
            for c in 0..m {
                b.swap(k * m + c, piv * m + c);
            }
        }
        for i in k + 1..n {
            let f = a[i * n + k] / a[k * n + k];
            for c in k..n {
                a[i * n + c] -= f * a[k * n + c];
            }
            for c in 0..m {
                b[i * m + c] -= f * b[k * m + c];
            }
        }
    }
    for k in (0..n).rev() {
        for c in 0..m {
            let mut s = b[k * m + c];
            for j in k + 1..n {
                s -= a[k * n + j] * b[j * m + c];
            }
            b[k * m + c] = s / a[k * n + k];
        }
    }
    b
}

/// A random linear partition: nodes `3 + i` free (`i < n`), `0` ground, `1` and `2` the two sources.
/// The first `ncap` free nodes carry a capacitor to ground.
pub(crate) struct Net {
    pub n: usize,
    pub ncap: usize,
    /// Full conductance matrix of the free nodes (shunt and fixed-node conductances on the diagonal).
    pub g: Vec<f64>,
    /// Conductance of node `i` to source 1 and 2 (the rest of the fixed conductance is to ground).
    pub to_src: Vec<[f64; 2]>,
    pub c: Vec<f64>,
    /// Every resistor: `(node a, node b, ohms)`.
    pub res: Vec<(usize, usize, f64)>,
}

pub(crate) fn random_net(r: &mut Rng) -> Net {
    let ncap = 2 + r.below(5);
    let n = ncap + r.below(4);
    let mut g = vec![0.0; n * n];
    let mut to_src = vec![[0.0; 2]; n];
    let mut res = vec![];
    for i in 0..n {
        g[i * n + i] += 1e-12;
    }
    for i in 0..n {
        if r.next() < 0.7 {
            let y = r.log(1e-6, 1e3);
            let s = r.below(3); // ground, source 1, source 2
            g[i * n + i] += y;
            if s > 0 {
                to_src[i][s - 1] += y;
            }
            res.push((3 + i, s, 1.0 / y));
        }
        for j in i + 1..n {
            if r.next() < 0.6 {
                let y = r.log(1e-6, 1e3);
                g[i * n + i] += y;
                g[j * n + j] += y;
                g[i * n + j] -= y;
                g[j * n + i] -= y;
                res.push((3 + i, 3 + j, 1.0 / y));
            }
        }
    }
    let c = (0..ncap).map(|_| r.log(1e-9, 1e-3)).collect();
    Net { n, ncap, g, to_src, c, res }
}

impl Net {
    pub fn program(&self, v: [f64; 2]) -> Program {
        let mut devices = vec![
            Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![v[0]], blob: vec![] },
            Device { kind: kinds::VOLTAGE_SOURCE, component_index: 1, pins: vec![pin(2), pin(0)], params: vec![v[1]], blob: vec![] },
        ];
        for (k, &(a, b, ohms)) in self.res.iter().enumerate() {
            devices.push(Device { kind: kinds::RESISTOR, component_index: 2 + k, pins: vec![pin(a), pin(b)], params: vec![ohms], blob: vec![] });
        }
        let base = devices.len();
        for k in 0..self.ncap {
            devices.push(Device { kind: kinds::CAPACITOR, component_index: base + k, pins: vec![pin(3 + k), pin(0)], params: vec![self.c[k], 0.0, 0.0], blob: vec![] });
        }
        Program { carry: Vec::new(), node_count: 3 + self.n, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    }

    /// The matrix exponential's answer: the partition's free-node voltages at `t` after its capacitor-node
    /// voltages `xk0`, sources at `v` from then on. Also the largest and the smallest rate.
    pub fn exact(&self, v: [f64; 2], xk0: &[f64]) -> Exact {
        let (n, nk) = (self.n, self.ncap);
        let na = n - nk;
        let b: Vec<f64> = (0..n).map(|i| self.to_src[i][0] * v[0] + self.to_src[i][1] * v[1]).collect();
        let gkk: Vec<f64> = (0..nk * nk).map(|p| self.g[(p / nk) * n + p % nk]).collect();
        let mut s = gkk.clone();
        let mut beta: Vec<f64> = b[..nk].to_vec();
        let mut gaa_inv_ak = vec![];
        let mut gaa_inv_b = vec![];
        if na > 0 {
            let gaa: Vec<f64> = (0..na * na).map(|p| self.g[(nk + p / na) * n + nk + p % na]).collect();
            let gak: Vec<f64> = (0..na * nk).map(|p| self.g[(nk + p / nk) * n + p % nk]).collect();
            let ba: Vec<f64> = b[nk..].to_vec();
            gaa_inv_ak = solve(&gaa, &gak, na, nk);
            gaa_inv_b = solve(&gaa, &ba, na, 1);
            for a in 0..nk {
                for bb in 0..nk {
                    let mut t = 0.0;
                    for q in 0..na {
                        t += self.g[a * n + nk + q] * gaa_inv_ak[q * nk + bb];
                    }
                    s[a * nk + bb] -= t;
                }
                let mut t = 0.0;
                for q in 0..na {
                    t += self.g[a * n + nk + q] * gaa_inv_b[q];
                }
                beta[a] -= t;
            }
        }
        let sym: Vec<f64> = (0..nk * nk).map(|p| s[p] / (self.c[p / nk] * self.c[p % nk]).sqrt()).collect();
        let (lam, q) = jacobi(sym, nk);
        // y = Q^T C^1/2 x ; beta~ = Q^T C^-1/2 beta
        let mut y0 = vec![0.0; nk];
        let mut bt = vec![0.0; nk];
        for m in 0..nk {
            for k in 0..nk {
                y0[m] += q[k * nk + m] * self.c[k].sqrt() * xk0[k];
                bt[m] += q[k * nk + m] * beta[k] / self.c[k].sqrt();
            }
        }
        // the algebraic nodes: x_a = Gaa^-1 (b_a - Gak x_k)
        Exact { net_n: n, nk, na, lam, q, y0, bt, c: self.c.clone(), gaa_inv_ak, gaa_inv_b }
    }
}

pub(crate) struct Exact {
    net_n: usize,
    nk: usize,
    na: usize,
    pub lam: Vec<f64>,
    q: Vec<f64>,
    y0: Vec<f64>,
    bt: Vec<f64>,
    c: Vec<f64>,
    gaa_inv_ak: Vec<f64>,
    gaa_inv_b: Vec<f64>,
}

impl Exact {
    pub fn at(&self, t: f64) -> Vec<f64> {
        let nk = self.nk;
        let y: Vec<f64> = (0..nk)
            .map(|m| {
                let l = self.lam[m];
                let lt = (l * t).max(0.0);
                let phi = if l.abs() * t < 1e-8 { t } else { -(-lt).exp_m1() / l };
                (-lt).exp() * self.y0[m] + self.bt[m] * phi
            })
            .collect();
        let xk: Vec<f64> = (0..nk).map(|k| (0..nk).map(|m| self.q[k * nk + m] * y[m]).sum::<f64>() / self.c[k].sqrt()).collect();
        let mut out = xk.clone();
        for a in 0..self.na {
            let mut v = self.gaa_inv_b[a];
            for k in 0..nk {
                v -= self.gaa_inv_ak[a * nk + k] * xk[k];
            }
            out.push(v);
        }
        debug_assert_eq!(out.len(), self.net_n);
        out
    }
    pub fn lam_max(&self) -> f64 {
        self.lam.iter().cloned().fold(0.0, f64::max)
    }
}

#[derive(Default, Debug)]
struct Tally {
    cases: u64,
    stops: u64,
    nodes: u64,
    trbdf2_steps: u64,
    outside_w0: u64,
    plain_outside_w0: u64,
    worse_and_outside: u64,
    worst_w0: f64,
    plain_worst_w0: f64,
    worst_case: usize,
    nonfinite: u64,
    skipped: u64,
    /// accepted steps by guard side (0: bound x h <= 10, 1: above) and kind (0 neither, 1 extrapolated trapezoid, 2 extrapolated TR-BDF2)
    cls: [[u64; 3]; 2],
    /// the same inside 9.0..11.0
    near: [[u64; 3]; 2],
    misclassified: u64,
}

fn w0(want: f64) -> f64 {
    1e-4 + 1e-3 * want.abs()
}

/// One case: the net run from zero to a random time, the sources flipped (a discontinuity), then 200 host
/// stops of `p / lam_max` each, against the matrix exponential, the plain trapezoid (TR-BDF2 off) beside.
fn run_case(r: &mut Rng, case: usize, tally: &mut Tally, verbose: bool) {
    let _ = ();
    // a net whose fastest rate times the 10 ns floor step is over `zfloor` cannot be stepped within the guard
    // by any step the engine may take (R81's case): not this check's subject, drawn again.
    let zfloor: f64 = std::env::var("V2_ZFLOOR").ok().and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let (net, probe, lmax) = loop {
        let net = random_net(r);
        let probe = net.exact([1.0, 1.0], &vec![0.0; net.ncap]);
        let lmax = probe.lam_max();
        if lmax * stepped::H_MIN <= zfloor {
            break (net, probe, lmax);
        }
        tally.skipped += 1;
    };
    let v0 = [r.next() * 10.0 - 5.0, r.next() * 10.0 - 5.0];
    let v1 = [r.next() * 10.0 - 5.0, r.next() * 10.0 - 5.0];
    let mut lams = probe.lam.clone();
    lams.sort_by(|a, b| a.total_cmp(b));
    let pick = lams[r.below(lams.len())];
    let t0 = (r.log(0.3, 3.0) / pick.max(1e-6)).min(0.2);
    let mut eng = Engine::load(&net.program(v0)).unwrap();
    if eng.advance(t0).is_err() {
        return;
    }
    let t_flip = eng.time();
    let xk0: Vec<f64> = (0..net.ncap).map(|k| eng.voltages()[3 + k]).collect();
    eng.set_params(&[(0, 0, v1[0]), (1, 0, v1[1])]).unwrap();
    let ex = net.exact(v1, &xk0);
    let prod = r.log(10.0, 1e8);
    let h = (prod / lmax).min(1.0);
    if std::env::var("V2_TIME").is_ok() {
        eprintln!("T   lmax {lmax:e} lmin {:e} prod {prod:e} h {h:e} t0 {t0:e}", lams[0]);
    }
    let prod = prod;
    let mut plain = eng.snapshot();
    stepped::EXTRAP_TRBDF2.with(|c| c.set(0));
    stepped::STEP_LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
    let mut ok_new = true;
    let mut ok_plain = true;
    let mut errs_new = vec![];
    for k in 1..=200 {
        let t = t_flip + k as f64 * h;
        ok_new &= eng.advance(t).is_ok();
        if !ok_new {
            break;
        }
        errs_new.push((t, eng.voltages()[3..3 + net.n].to_vec()));
    }
    let used = stepped::EXTRAP_TRBDF2.with(|c| c.get());
    {
        let log = stepped::STEP_LOG.with(|l| l.borrow_mut().take().unwrap());
        let caps: Vec<_> = (0..net.ncap).map(|k| (None, Some(k), net.c[k], 0.0)).collect();
        let bound = stepped::rate_from_matrix(&net.g, net.n, &caps).unwrap();
        for rec in log.iter().filter(|r| r.x.len() == net.n) {
            let pr = bound * rec.h;
            let side = usize::from(pr > 10.0);
            let kind = if rec.trbdf2 { 2 } else if rec.extrap { 1 } else { 0 };
            tally.cls[side][kind] += 1;
            if (9.0..11.0).contains(&pr) {
                tally.near[side][kind] += 1;
            }
            if (side == 0 && kind == 2) || (side == 1 && kind == 1) {
                tally.misclassified += 1;
                if verbose {
                    eprintln!("case {case}: step h {:e} bound {bound:e} product {pr:.4} kind {kind}", rec.h);
                }
            }
        }
    }
    stepped::NO_TRBDF2.with(|c| c.set(true));
    let mut errs_plain = vec![];
    for k in 1..=200 {
        let t = t_flip + k as f64 * h;
        ok_plain &= plain.advance(t).is_ok();
        if !ok_plain {
            break;
        }
        errs_plain.push((t, plain.voltages()[3..3 + net.n].to_vec()));
    }
    stepped::NO_TRBDF2.with(|c| c.set(false));
    if !ok_new || !ok_plain {
        eprintln!("case {case}: advance failed (new ok {ok_new}, plain ok {ok_plain})");
        return;
    }
    tally.cases += 1;
    tally.trbdf2_steps += used;
    let mut case_worst = 0.0f64;
    for (i, (t, vn)) in errs_new.iter().enumerate() {
        let want = ex.at(t - t_flip);
        let vp = &errs_plain[i].1;
        tally.stops += 1;
        for j in 0..net.n {
            tally.nodes += 1;
            let w = w0(want[j]);
            let (en, ep) = ((vn[j] - want[j]).abs(), (vp[j] - want[j]).abs());
            if !en.is_finite() {
                tally.nonfinite += 1;
            }
            if en > w {
                tally.outside_w0 += 1;
                if verbose && tally.outside_w0 < 12 {
                    eprintln!("case {case} stop {} node {j}/{} (ncap {}): new {:e} plain {:e} want {:e} W0 {w:e} prod {prod:.3e} t-t_flip {:e} lam {:?}", i + 1, net.n, net.ncap, vn[j], vp[j], want[j], t - t_flip, ex.lam);
                }
            }
            if ep > w {
                tally.plain_outside_w0 += 1;
            }
            if en > w && en > ep * 1.000001 + 1e-9 {
                tally.worse_and_outside += 1;
                if verbose && tally.worse_and_outside < 20 {
                    eprintln!("case {case} stop {} node {j}: new err {en:e} ({:.2} W0), plain {ep:e}, want {:e}, t {t:e}", i + 1, en / w, want[j]);
                }
            }
            case_worst = case_worst.max(en / w);
            tally.worst_w0 = tally.worst_w0.max(en / w);
            tally.plain_worst_w0 = tally.plain_worst_w0.max(ep / w);
        }
    }
    if case_worst >= tally.worst_w0 {
        tally.worst_case = case;
    }
}

#[test]
fn v2_random_stiff_partitions_in_the_engine_against_the_matrix_exponential() {
    let cases: usize = std::env::var("V2_CASES").ok().and_then(|s| s.parse().ok()).unwrap_or(150);
    let seed: u64 = std::env::var("V2_SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(0x9e3779b97f4a7c15);
    let mut r = Rng(seed);
    let mut tally = Tally::default();
    for case in 0..cases {
        let t = std::time::Instant::now();
        run_case(&mut r, case, &mut tally, true);
        if std::env::var("V2_TIME").is_ok() {
            eprintln!("T case {case} {:.2}s", t.elapsed().as_secs_f64());
        }
    }
    eprintln!("V2 {tally:?}");
    assert_eq!(tally.misclassified, 0);
    assert_eq!(tally.nonfinite, 0);
    assert_eq!(tally.worse_and_outside, 0, "TR-BDF2 reads outside W0 and worse than the plain trapezoid");
}

// ---------------------------------------------------------------------------------------------------------
// V3: the estimate against the true local error of every accepted step.
// ---------------------------------------------------------------------------------------------------------

struct V3Out {
    steps: usize,
    trbdf2: usize,
    /// (h, ratio, local error in W0, trbdf2) of every accepted step of the partition.
    recs: Vec<(f64, f64, f64, bool)>,
    /// (index, previous vc, committed vc, exact vc) of the steps whose error the estimate does not bound
    detail: Vec<(usize, Vec<f64>, Vec<f64>, Vec<f64>)>,
}

fn v3_run(net: &Net, v1: [f64; 2], stops: &[f64], plain: bool) -> Option<V3Out> {
    let mut eng = Engine::load(&net.program([0.0, 0.0])).unwrap();
    eng.advance(1e-9).unwrap();
    let t_flip = eng.time();
    eng.set_params(&[(0, 0, v1[0]), (1, 0, v1[1])]).unwrap();
    stepped::NO_TRBDF2.with(|c| c.set(plain));
    stepped::STEP_LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
    stepped::ACCEPTED.with(|a| *a.borrow_mut() = Some(Vec::new()));
    let mut ok = true;
    for &t in stops {
        ok &= eng.advance(t_flip + t).is_ok();
    }
    stepped::NO_TRBDF2.with(|c| c.set(false));
    let planned = stepped::STEP_LOG.with(|l| l.borrow_mut().take().unwrap());
    let committed = stepped::ACCEPTED.with(|a| a.borrow_mut().take().unwrap());
    // STEP_LOG is written where a step is planned (a host stop re-plans, a trial may be dropped): keep, for every
    // commit, the last planned step ending at the commit's time.
    let mut log = vec![];
    for (tc, _) in &committed {
        if let Some(r) = planned.iter().rev().find(|r| (r.t_end - tc).abs() <= 1e-12 * tc.abs().max(1e-12)) {
            log.push(r.clone());
        }
    }
    if !ok {
        return None;
    }
    // one partition holding the whole net, else the check does not apply
    let recs_all: Vec<_> = log.iter().filter(|r| r.x.len() == net.n && r.vc.len() == net.ncap).cloned().collect();
    if recs_all.is_empty() || recs_all.len() != log.len() {
        return None;
    }
    let mut prev = vec![0.0; net.ncap];
    let mut recs = vec![];
    let mut detail = vec![];
    for (idx, r) in recs_all.iter().enumerate() {
        let ex = net.exact(v1, &prev).at(r.h);
        let mut lerr = 0.0f64;
        for k in 0..net.ncap {
            lerr = lerr.max((r.vc[k] - ex[k]).abs() / w0(ex[k]));
        }
        recs.push((r.h, r.ratio, lerr, r.trbdf2));
        if r.trbdf2 && lerr > r.ratio.max(1e-3) && lerr > 1.0 {
            detail.push((idx, prev.clone(), r.vc.clone(), ex[..net.ncap].to_vec()));
        }
        prev = r.vc.clone();
    }
    Some(V3Out { steps: recs.len(), trbdf2: recs.iter().filter(|r| r.3).count(), recs, detail })
}

fn v3_report(label: &str, net: &Net, v1: [f64; 2], stops: &[f64]) {
    let (Some(n), Some(p)) = (v3_run(net, v1, stops, false), v3_run(net, v1, stops, true)) else {
        eprintln!("V3 {label}: not applicable (several partitions or an advance failed)");
        return;
    };
    for (idx, prev, got, want) in n.detail.iter().take(4) {
        let (h, ratio, lerr, _) = n.recs[*idx];
        eprintln!("V3 {label}: step #{idx} (previous step h {:e}) h {h:e} estimate {ratio:.3e} true local error {lerr:.2} W0; prev {prev:.5?} got {got:.6?} exact {want:.6?}", if *idx > 0 { n.recs[idx - 1].0 } else { 0.0 });
    }
    let tr: Vec<_> = n.recs.iter().filter(|r| r.3).collect();
    let mut q: Vec<f64> = tr.iter().filter(|r| r.1 > 1e-12).map(|r| r.2 / r.1).collect();
    q.sort_by(|a, b| a.total_cmp(b));
    let med = if q.is_empty() { f64::NAN } else { q[q.len() / 2] };
    let unbounded = tr.iter().filter(|r| r.2 > r.1.max(1e-3) && r.2 > 1.0).count();
    let worst_l = tr.iter().fold(0.0f64, |a, r| a.max(r.2));
    let worst_pl = p.recs.iter().fold(0.0f64, |a, r| a.max(r.2));
    let mean_ratio = tr.iter().map(|r| r.1).sum::<f64>() / tr.len().max(1) as f64;
    let small = tr.iter().filter(|r| r.1 < 0.05).count();
    eprintln!(
        "V3 {label}: accepted steps new {} (TR-BDF2 {}), plain {}; TR-BDF2 steps: median true/estimate {med:.3e}, mean accepted ratio {mean_ratio:.3}, ratio<0.05 in {small}, estimate not bounding (true > max(estimate,1e-3) and > 1 W0) in {unbounded}; worst true local error new {worst_l:.3} W0, plain {worst_pl:.3} W0",
        n.steps, n.trbdf2, p.steps
    );
    // the longest step's line
    if let Some(r) = tr.iter().max_by(|a, b| a.0.total_cmp(&b.0)) {
        eprintln!("V3 {label}: longest TR-BDF2 step h {:e}, estimate {:.4}, true local error {:.3e} W0", r.0, r.1, r.2);
    }
}

fn ladder_net(rs: f64) -> Net {
    // nodes: 3 (the ladder's node 2), 4 (its node 3); source 1 behind rs.
    let n = 2;
    let mut g = vec![0.0; n * n];
    let mut to_src = vec![[0.0; 2]; n];
    let res = vec![(3, 1, rs), (3, 0, 1000.0), (3, 4, 1000.0)];
    g[0] = 1.0 / rs + 1e-3 + 1e-3 + 1e-12;
    g[3] = 1e-3 + 1e-12;
    g[1] = -1e-3;
    g[2] = -1e-3;
    to_src[0][0] = 1.0 / rs;
    Net { n, ncap: 2, g, to_src, c: vec![1e-6, 1e-6], res }
}

#[test]
fn v3_estimate_against_true_local_error_on_the_ladder_and_random_stiff_partitions() {
    for rs in [1.0, 5.0, 20.0, 50.0] {
        let stops: Vec<f64> = (1..=40).map(|k| k as f64 * 1e-3).collect();
        v3_report(&format!("ladder rs {rs}"), &ladder_net(rs), [3.3, 0.0], &stops);
    }
    let mut r = Rng(0x5eed_0003);
    let mut done = 0;
    let mut tries = 0;
    while done < 6 && tries < 400 {
        tries += 1;
        let net = random_net(&mut r);
        let lmax = net.exact([0.0, 0.0], &vec![0.0; net.ncap]).lam_max();
        if lmax * stepped::H_MAX < 2e3 || lmax * stepped::H_MIN > 1.0 {
            continue;
        }
        let v1 = [r.next() * 10.0 - 5.0, r.next() * 10.0 - 5.0];
        let stops: Vec<f64> = (1..=40).map(|k| k as f64 * 1e-3).collect();
        let before = done;
        // v3_report prints "not applicable" for a split net; count only the applicable ones
        if v3_run(&net, v1, &stops, false).is_some() {
            v3_report(&format!("random net #{tries} (lmax {lmax:.2e}, ncap {}, n {})", net.ncap, net.n), &net, v1, &stops);
            done += 1;
        }
        let _ = before;
    }
}

// ---------------------------------------------------------------------------------------------------------
// V4: the charge bill of extrapolated TR-BDF2 steps: ladder, ESR on a capacitor, host stops that truncate steps.
// ---------------------------------------------------------------------------------------------------------

fn v4_program(rs: f64, esr1: f64) -> Program {
    let r = |ci: usize, a: usize, b: usize, ohms: f64| Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![ohms], blob: vec![] };
    let cap = |ci: usize, a: usize, c: f64, esr: f64| Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(0)], params: vec![c, esr, 0.0], blob: vec![] };
    Program {
        carry: Vec::new(),
        node_count: 4,
        options: Options::default(),
        digital_slot_count: 0,
        channel_slot_count: 0,
        devices: vec![
            Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![3.3], blob: vec![] },
            r(1, 1, 2, rs),
            r(2, 2, 0, 1000.0),
            r(3, 2, 3, 1000.0),
            cap(4, 2, 1e-6, esr1),
            cap(5, 3, 1e-6, 0.0),
        ],
    }
}

/// Returns (worst relative deviation of a capacitor's span charge from C dvc, worst KCL residual in
/// charge over the span's |q| scale, TR-BDF2 steps taken, spans).
fn v4_run(rs: f64, esr1: f64, stops: &[f64], plain: bool) -> (f64, f64, f64, u64, usize) {
    let p = v4_program(rs, esr1);
    let mut e = Engine::load(&p).unwrap();
    let all: Vec<u32> = (0..p.devices.len() as u32).collect();
    e.charge_arm(&all).unwrap();
    stepped::NO_TRBDF2.with(|c| c.set(plain));
    stepped::EXTRAP_TRBDF2.with(|c| c.set(0));
    // state at the previous stop: node voltages and the c1 series current
    let (mut v_prev, mut i1_prev) = (vec![0.0; 4], 0.0);
    let (mut worst_c1, mut worst_c2, mut worst_kcl) = (0.0f64, 0.0f64, 0.0f64);
    let mut spans = 0;
    for &t in stops {
        e.advance(t).unwrap();
        let b = e.take_charge().to_vec();
        let slots = b.len() - 2 - 2 * all.len();
        let q = &b[2..2 + slots];
        let v = e.voltages().to_vec();
        let i1 = e.currents()[4];
        // c1: vc = v2 - esr * i1 (series current into the capacitor), c2: v3.
        let vc1 = v[2] - esr1 * i1;
        let vc1_prev = v_prev[2] - esr1 * i1_prev;
        let want1 = 1e-6 * (vc1 - vc1_prev);
        let want2 = 1e-6 * (v[3] - v_prev[3]);
        let sc1 = 1e-6 * (vc1.abs() + vc1_prev.abs()) + 1e-15;
        let sc2 = 1e-6 * (v[3].abs() + v_prev[3].abs()) + 1e-15;
        worst_c1 = worst_c1.max((q[4] - want1).abs() / sc1);
        worst_c2 = worst_c2.max((q[5] - want2).abs() / sc2);
        // KCL at node 2: rs resistor (1 -> 2) in; 2k to ground, 1k on to node 3, c1 out. Node 3: the 1k in, c2 out.
        let scale = q[1].abs().max(q[2].abs()).max(q[3].abs()).max(q[4].abs()) + 1e-18;
        let k2 = (q[1] - q[2] - q[3] - q[4]).abs() - 2e-14;
        let k3 = (q[3] - q[5]).abs() - 2e-14;
        worst_kcl = worst_kcl.max(k2.max(k3).max(0.0) / scale);
        v_prev = v;
        i1_prev = i1;
        spans += 1;
    }
    stepped::NO_TRBDF2.with(|c| c.set(false));
    let used = stepped::EXTRAP_TRBDF2.with(|c| c.get());
    (worst_c1, worst_c2, worst_kcl, used, spans)
}

#[test]
fn v4_charge_bill_on_the_ladder_with_an_esr_and_across_truncated_steps() {
    let mut r = Rng(0x5eed_0004);
    for (rs, esr) in [(5.0, 0.0), (5.0, 0.5), (1.0, 0.05), (20.0, 2.0)] {
        // uneven stops, some of them shorter than the step the controller had planned (truncating it)
        let mut t = 0.0;
        let mut stops = vec![];
        for _ in 0..120 {
            t += r.log(2e-8, 2.5e-3);
            stops.push(t);
        }
        let new = v4_run(rs, esr, &stops, false);
        let plain = v4_run(rs, esr, &stops, true);
        eprintln!(
            "V4 rs {rs} esr {esr}: {} spans to {:.4} s, TR-BDF2 steps {}; worst |q_c1 - C dvc1| / (C |v|) {:.2e} (plain {:.2e}), c2 {:.2e} (plain {:.2e}), KCL residual / |q| {:.2e} (plain {:.2e})",
            new.4, t, new.3, new.0, plain.0, new.1, plain.1, new.2, plain.2
        );
    }
}
