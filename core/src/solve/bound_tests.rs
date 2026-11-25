//! CAPERR3: the extrapolation guard's Gershgorin bound against the true largest
//! rate of `C^-1 S` (an eigenvalue computation), on random linear partitions
//! inside the class the bound is claimed for and just outside it, and the
//! extrapolated trapezoid at the guard's edge on the same networks.

use super::stepped::rate_from_matrix;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn log(&mut self, lo: f64, hi: f64) -> f64 {
        (lo.ln() + self.next() * (hi.ln() - lo.ln())).exp()
    }
    fn below(&mut self, n: usize) -> usize {
        ((self.next() * n as f64) as usize).min(n - 1)
    }
}

/// Eigenvalues of a symmetric matrix by cyclic Jacobi.
fn jacobi(mut a: Vec<f64>, n: usize) -> Vec<f64> {
    for _ in 0..100 {
        let mut off = 0.0;
        let mut diag = 0.0;
        for i in 0..n {
            for j in 0..n {
                if i != j {
                    off += a[i * n + j] * a[i * n + j];
                } else {
                    diag += a[i * n + j] * a[i * n + j];
                }
            }
        }
        if off <= 1e-30 * diag.max(1e-300) {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                let apq = a[p * n + q];
                if apq == 0.0 {
                    continue;
                }
                let theta = (a[q * n + q] - a[p * n + p]) / (2.0 * apq);
                let t = if theta == 0.0 { 1.0 } else { theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt()) };
                let (c, s) = (1.0 / (t * t + 1.0).sqrt(), t / (t * t + 1.0).sqrt());
                for k in 0..n {
                    let (akp, akq) = (a[k * n + p], a[k * n + q]);
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (a[p * n + k], a[q * n + k]);
                    a[p * n + k] = c * apk - s * aqk;
                    a[q * n + k] = s * apk + c * aqk;
                }
            }
        }
    }
    (0..n).map(|i| a[i * n + i]).collect()
}

/// `X = A^-1 B` by Gaussian elimination with partial pivoting (`A` n x n, `B` n x m).
fn solve(a: &[f64], b: &[f64], n: usize, m: usize) -> Vec<f64> {
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

/// One random linear partition: `g` the conductance matrix over `n` free
/// nodes (shunt included), the capacitors `(pin0, pin1, c, esr)`.
struct Net {
    n: usize,
    g: Vec<f64>,
    caps: Vec<(Option<usize>, Option<usize>, f64, f64)>,
}

fn stamp(g: &mut [f64], n: usize, a: usize, b: Option<usize>, y: f64) {
    g[a * n + a] += y;
    if let Some(b) = b {
        g[b * n + b] += y;
        g[a * n + b] -= y;
        g[b * n + a] -= y;
    }
}

fn random_net(r: &mut Rng) -> Net {
    let ncap = 2 + r.below(5);
    let n = ncap + r.below(4);
    let mut g = vec![0.0; n * n];
    for i in 0..n {
        g[i * n + i] += 1e-12;
    }
    for i in 0..n {
        stamp(&mut g, n, i, None, if r.next() < 0.7 { r.log(1e-6, 1e3) } else { 0.0 });
        for j in i + 1..n {
            if r.next() < 0.6 {
                stamp(&mut g, n, i, Some(j), r.log(1e-6, 1e3));
            }
        }
    }
    let mut caps = vec![];
    for k in 0..ncap {
        let esr = if r.next() < 0.3 { r.log(1e-3, 1e2) } else { 0.0 };
        let c = r.log(1e-9, 1e-3);
        caps.push(if r.next() < 0.5 { (None, Some(k), c, esr) } else { (Some(k), None, c, esr) });
    }
    Net { n, g, caps }
}

/// The capacitive system `C v' = -S v` of `net`: `S` the Schur complement of
/// the full conductance matrix (an ESR's internal node included) onto the
/// nodes that carry a capacitance, and those capacitances.
fn schur(net: &Net) -> (Vec<f64>, Vec<f64>, usize) {
    let extra = net.caps.iter().filter(|c| c.3 > 0.0).count();
    let m = net.n + extra;
    let mut g = vec![0.0; m * m];
    for i in 0..net.n {
        for j in 0..net.n {
            g[i * m + j] = net.g[i * net.n + j];
        }
    }
    let mut cd = vec![0.0; m];
    let mut next = net.n;
    for &(p0, p1, c, esr) in &net.caps {
        let l = p0.or(p1).unwrap();
        if esr > 0.0 {
            stamp(&mut g, m, l, Some(next), 1.0 / esr);
            cd[next] += c;
            next += 1;
        } else {
            cd[l] += c;
        }
    }
    let k: Vec<usize> = (0..m).filter(|&i| cd[i] > 0.0).collect();
    let al: Vec<usize> = (0..m).filter(|&i| cd[i] == 0.0).collect();
    let (nk, na) = (k.len(), al.len());
    let mut s = vec![0.0; nk * nk];
    for (a, &i) in k.iter().enumerate() {
        for (b, &j) in k.iter().enumerate() {
            s[a * nk + b] = g[i * m + j];
        }
    }
    if na > 0 {
        let gaa: Vec<f64> = al.iter().flat_map(|&i| al.iter().map(move |&j| (i, j))).map(|(i, j)| g[i * m + j]).collect();
        let gak: Vec<f64> = al.iter().flat_map(|&i| k.iter().map(move |&j| (i, j))).map(|(i, j)| g[i * m + j]).collect();
        let y = solve(&gaa, &gak, na, nk);
        for a in 0..nk {
            for b in 0..nk {
                let mut t = 0.0;
                for q in 0..na {
                    t += g[k[a] * m + al[q]] * y[q * nk + b];
                }
                s[a * nk + b] -= t;
            }
        }
    }
    (s, k.iter().map(|&i| cd[i]).collect(), nk)
}

/// The largest rate of `C^-1 S`, by Jacobi on `C^-1/2 S C^-1/2`.
fn true_rate(net: &Net) -> f64 {
    let (s, c, nk) = schur(net);
    let sym: Vec<f64> = (0..nk * nk).map(|ab| s[ab] / (c[ab / nk] * c[ab % nk]).sqrt()).collect();
    jacobi(sym, nk).into_iter().fold(0.0, f64::max)
}

#[test]
fn the_bound_is_an_upper_bound_on_the_largest_rate_of_random_partitions_in_its_class() {
    let mut r = Rng(0x9e3779b97f4a7c15);
    let (mut smallest, mut largest) = (f64::INFINITY, 0.0f64);
    for case in 0..10_000 {
        let net = random_net(&mut r);
        let bound = rate_from_matrix(&net.g, net.n, &net.caps).expect("a symmetric dominant matrix with grounded capacitors has a bound");
        let rate = true_rate(&net);
        smallest = smallest.min(bound / rate);
        largest = largest.max(bound / rate);
        assert!(bound >= rate * (1.0 - 1e-6), "case {case}: bound {bound:e} below the true rate {rate:e} (n {}, caps {:?})", net.n, net.caps);
    }
    eprintln!("bound / true rate over 10,000 cases: smallest {smallest:.4}, largest {largest:.1}");
}

#[test]
fn a_network_just_outside_the_class_has_no_bound_or_one_that_still_holds() {
    let mut r = Rng(12345);
    let (mut none_float, mut none_asym, mut none_dom, mut mixed_given, mut mixed_failed) = (0, 0, 0, 0, 0);
    for case in 0..10_000 {
        let mut net = random_net(&mut r);
        let n = net.n;
        match case % 4 {
            0 => {
                // A capacitor between two free nodes.
                let c = net.caps[0].2;
                net.caps.push((Some(0), Some(1), c, 0.0));
                assert!(rate_from_matrix(&net.g, n, &net.caps).is_none(), "case {case}: a floating capacitor gave a bound");
                none_float += 1;
            }
            1 => {
                // A slightly asymmetric stamp (1e-6 relative).
                let e = net.g[1];
                net.g[1] = if e != 0.0 { e * (1.0 + 1e-6) } else { 1e-9 };
                assert!(rate_from_matrix(&net.g, n, &net.caps).is_none(), "case {case}: an asymmetric matrix gave a bound");
                none_asym += 1;
            }
            2 => {
                // A symmetric off-diagonal larger than the diagonal.
                let big = 10.0 * net.g[0].abs().max(net.g[n + 1].abs()).max(1e-3);
                net.g[1] = big;
                net.g[n] = big;
                assert!(rate_from_matrix(&net.g, n, &net.caps).is_none(), "case {case}: a non-dominant matrix gave a bound");
                none_dom += 1;
            }
            _ => {
                // A negative conductance (a positive off-diagonal) the diagonals still dominate:
                // if a bound is given it must still bound the rate.
                let y = -0.5 * r.log(1e-6, 1e-1);
                stamp(&mut net.g, n, 0, Some(1), y);
                if let Some(b) = rate_from_matrix(&net.g, n, &net.caps) {
                    let rate = true_rate(&net);
                    mixed_given += 1;
                    if b < rate * (1.0 - 1e-6) {
                        mixed_failed += 1;
                        eprintln!("case {case}: mixed-sign bound {b:e} below the rate {rate:e}");
                    }
                }
            }
        }
    }
    eprintln!("no bound: floating {none_float}, asymmetric {none_asym}, non-dominant {none_dom}; mixed-sign: {mixed_given} given a bound, {mixed_failed} below the rate");
    assert_eq!(mixed_failed, 0);
}

/// The trapezoid's one-step map on the capacitive nodes' voltages for
/// `C v' = -S v`: `(C + h/2 S)^-1 (C - h/2 S)`.
fn trap_map(s: &[f64], c: &[f64], nk: usize, h: f64) -> Vec<f64> {
    let mut lhs = vec![0.0; nk * nk];
    let mut rhs = vec![0.0; nk * nk];
    for a in 0..nk {
        for b in 0..nk {
            let d = if a == b { c[a] } else { 0.0 };
            lhs[a * nk + b] = 0.5 * h * s[a * nk + b] + d;
            rhs[a * nk + b] = -0.5 * h * s[a * nk + b] + d;
        }
    }
    solve(&lhs, &rhs, nk, nk)
}

fn mul(a: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut o = vec![0.0; n * n];
    for i in 0..n {
        for k in 0..n {
            for j in 0..n {
                o[i * n + j] += a[i * n + k] * b[k * n + j];
            }
        }
    }
    o
}

#[test]
fn two_hundred_extrapolated_steps_at_the_guards_edge_of_random_partitions_do_not_grow() {
    // The extrapolated map is diagonal in the C-orthogonal eigenvectors of
    // `C^-1 S`, so its C-weighted norm never grows iff |E(lambda h)| <= 1 for
    // every mode; the map is built densely here and iterated on a random state.
    let mut r = Rng(777);
    let mut worst = 0.0f64;
    for case in 0..10_000 {
        let net = random_net(&mut r);
        let bound = rate_from_matrix(&net.g, net.n, &net.caps).unwrap();
        let (s, c, nk) = schur(&net);
        let h = 10.0 / bound;
        let t_h = trap_map(&s, &c, nk, h);
        let t_half = trap_map(&s, &c, nk, h / 2.0);
        let t2 = mul(&t_half, &t_half, nk);
        let m: Vec<f64> = (0..nk * nk).map(|i| (4.0 * t2[i] - t_h[i]) / 3.0).collect();
        let mut v: Vec<f64> = (0..nk).map(|_| r.next() * 2.0 - 1.0).collect();
        let energy = |v: &[f64]| v.iter().zip(&c).map(|(x, c)| c * x * x).sum::<f64>().sqrt();
        let e0 = energy(&v);
        for step in 0..200 {
            let mut o = vec![0.0; nk];
            for i in 0..nk {
                for j in 0..nk {
                    o[i] += m[i * nk + j] * v[j];
                }
            }
            v = o;
            let g = energy(&v) / e0;
            worst = worst.max(g);
            assert!(g <= 1.0 + 1e-6, "case {case} step {step}: energy norm grew to {g}");
        }
    }
    eprintln!("largest energy norm over the start's: {worst:.6}");
}

// ---- the same networks in the engine: 200 stops at the guard's edge ----

mod engine_edge {
    use super::*;
    use crate::engine::Engine;
    use crate::program::{Device, Options, PinRecord};
    use crate::solve::{kinds, stepped, Program};

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    #[test]
    fn two_hundred_stops_at_the_guards_edge_of_random_partitions_never_grow_in_the_engine() {
        // Nodes: 0 ground, 1 and 2 two sources at random voltages, then the free nodes. Every
        // free node has random conductances to ground, to each source and to the others; its
        // capacitors start uncharged, the host stops every `9.9 / bound` (the guard's edge:
        // the accepted step is that long or shorter), so the partition's error against its
        // DC solution, in the capacitance-weighted norm, must not grow past its start's.
        let mut r = Rng(4242);
        let (mut taken, mut cases, mut worst) = (0u64, 0, 0.0f64);
        for case in 0..400 {
            let net = random_net_plain(&mut r);
            let n = net.n;
            let v_src = [r.next() * 10.0 - 5.0, r.next() * 10.0 - 5.0];
            let mut g = net.g.clone();
            let mut devices = vec![
                Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![v_src[0]], blob: vec![] },
                Device { kind: kinds::VOLTAGE_SOURCE, component_index: 1, pins: vec![pin(2), pin(0)], params: vec![v_src[1]], blob: vec![] },
            ];
            let mut b = vec![0.0; n];
            let mut ci = 2;
            let mut res = |devices: &mut Vec<Device>, a: usize, bn: usize, y: f64| {
                devices.push(Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(bn)], params: vec![1.0 / y], blob: vec![] });
                ci += 1;
            };
            // The matrix `g` has its ground conductances already (`stamp(.., None, ..)`): rebuild the devices from it.
            for i in 0..n {
                let mut row = g[i * n + i] - 1e-12;
                for j in 0..n {
                    if j != i {
                        row += g[i * n + j];
                    }
                }
                // `row` is the conductance to the fixed nodes; give it to ground and a source at random.
                if row > 0.0 {
                    let to_src = r.next() < 0.5;
                    let s = 1 + r.below(2);
                    if to_src {
                        res(&mut devices, 1 + i + 2, s, row);
                        g[i * n + i] += 0.0;
                        b[i] += row * v_src[s - 1];
                    } else {
                        res(&mut devices, 1 + i + 2, 0, row);
                    }
                }
                for j in i + 1..n {
                    let y = -g[i * n + j];
                    if y > 0.0 {
                        res(&mut devices, 3 + i, 3 + j, y);
                    }
                }
            }
            for &(p0, p1, c, _) in &net.caps {
                let l = p0.or(p1).unwrap();
                devices.push(Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(3 + l), pin(0)], params: vec![c, 0.0, 0.0], blob: vec![] });
                ci += 1;
            }
            let program = Program { carry: Vec::new(), node_count: 3 + n, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices };
            let caps: Vec<_> = net.caps.iter().map(|c| (c.0, c.1, c.2, 0.0)).collect();
            let bound = rate_from_matrix(&net.g, n, &caps).unwrap();
            // DC: G v = b over every free node.
            let dc = solve(&net.g, &b.iter().map(|&x| x).collect::<Vec<_>>(), n, 1);
            let _ = &dc;
            let h = 9.9 / bound;
            let mut eng = Engine::load(&program).unwrap();
            stepped::EXTRAPOLATED.with(|c| c.set(0));
            let energy = |eng: &Engine| -> f64 {
                net.caps.iter().map(|&(p0, p1, c, _)| { let l = p0.or(p1).unwrap(); let d = eng.voltages()[3 + l] - dc[l]; c * d * d }).sum::<f64>().sqrt()
            };
            let e0 = net.caps.iter().map(|&(p0, p1, c, _)| { let l = p0.or(p1).unwrap(); c * dc[l] * dc[l] }).sum::<f64>().sqrt();
            if e0 < 1e-9 {
                continue;
            }
            cases += 1;
            for k in 1..=200 {
                eng.advance(k as f64 * h).unwrap();
                let e = energy(&eng) / e0;
                worst = worst.max(e);
                assert!(e.is_finite() && e <= 1.0 + 1e-3, "case {case} stop {k}: energy norm {e} of its start's");
            }
            taken += stepped::EXTRAPOLATED.with(|c| c.get());
        }
        eprintln!("{cases} engine cases, {taken} extrapolated steps, worst energy over start's {worst:.5}");
    }

    /// `random_net` without ESR, free nodes only (the engine program has none).
    fn random_net_plain(r: &mut Rng) -> Net {
        let mut net = random_net(r);
        net.caps.iter_mut().for_each(|c| c.3 = 0.0);
        net.caps.iter_mut().for_each(|c| *c = (None, Some(c.1.or(c.0).unwrap()), c.2, 0.0));
        net
    }
}
