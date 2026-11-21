//! MAGCC: a bench PSU in current limit charging a capacitor cluster off its
//! rail, default (rail-shortcut) mode against exact mode against a truth worked
//! in this file: the board's linear ODE, integrated by a fixed-step RK4 at a
//! step where halving it moves nothing at 1e-9 relative, and its closed form
//! `x(t) = e^{At} (x0 + A^-1 b) - A^-1 b` (the matrix exponential by scaling
//! and squaring). Never core output.
//!
//! Boards (PSU {5 V, 20 mA}: in CC from t = 0, the rail well under 5 V):
//! - coil: R1 100 ohm rail-ground, C 1 uF rail-node 2, L 1 mH (dcr 10 ohm)
//!   node 2-ground;
//! - res: the same with a resistor R2 in the coil's place;
//! - none: nothing at node 2 (the capacitor's far end floats).

use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

pub(crate) fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob: vec![] }
}

fn program(node_count: usize, devices: Vec<Device>, options: Options) -> Program {
    Program { node_count, options, digital_slot_count: 0, channel_slot_count: 0, devices, carry: Vec::new() }
}

/// W0 units: |got - want| / (abs + 1e-3 |want|).
fn w0(got: f64, want: f64, abs: f64) -> f64 {
    (got - want).abs() / (abs + 1e-3 * want.abs())
}
fn w0v(got: f64, want: f64) -> f64 {
    w0(got, want, 100e-6)
}
fn w0i(got: f64, want: f64) -> f64 {
    w0(got, want, 1e-9)
}

// ---- the truth ---------------------------------------------------------------

/// A linear board `x' = A x + b` with outputs read off `x`.
struct Lin {
    a: Vec<Vec<f64>>,
    b: Vec<f64>,
}

impl Lin {
    fn f(&self, x: &[f64]) -> Vec<f64> {
        (0..x.len()).map(|i| self.b[i] + (0..x.len()).map(|j| self.a[i][j] * x[j]).sum::<f64>()).collect()
    }
    fn rk4(&self, x0: &[f64], t: f64, h: f64) -> Vec<f64> {
        let n = (t / h).round().max(1.0) as usize;
        let h = t / n as f64;
        let mut x = x0.to_vec();
        let add = |x: &[f64], k: &[f64], s: f64| x.iter().zip(k).map(|(a, b)| a + s * b).collect::<Vec<_>>();
        for _ in 0..n {
            let k1 = self.f(&x);
            let k2 = self.f(&add(&x, &k1, h / 2.0));
            let k3 = self.f(&add(&x, &k2, h / 2.0));
            let k4 = self.f(&add(&x, &k3, h));
            for i in 0..x.len() {
                x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
        }
        x
    }
    /// RK4 at `h`, checked against `h / 2` to 1e-9 relative (else halves `h`).
    fn truth(&self, x0: &[f64], t: f64) -> Vec<f64> {
        if t == 0.0 {
            return x0.to_vec();
        }
        let mut h = 1e-8f64.min(t);
        loop {
            let (a, b) = (self.rk4(x0, t, h), self.rk4(x0, t, h / 2.0));
            let scale = b.iter().fold(0.0f64, |m, v| m.max(v.abs())).max(1e-30);
            if a.iter().zip(&b).all(|(p, q)| (p - q).abs() <= 1e-9 * scale) {
                return b;
            }
            h /= 2.0;
            assert!(h > 1e-14, "RK4 does not converge");
        }
    }
    /// The closed form by the matrix exponential (scaling and squaring of a
    /// Taylor series), on the augmented matrix [[A, b], [0, 0]].
    fn closed(&self, x0: &[f64], t: f64) -> Vec<f64> {
        let n = x0.len() + 1;
        let mut m = vec![vec![0.0; n]; n];
        for i in 0..n - 1 {
            for j in 0..n - 1 {
                m[i][j] = self.a[i][j] * t;
            }
            m[i][n - 1] = self.b[i] * t;
        }
        let norm = m.iter().map(|r| r.iter().map(|v| v.abs()).sum::<f64>()).fold(0.0, f64::max);
        let s = if norm > 0.5 { (norm / 0.5).log2().ceil() as i32 } else { 0 };
        let sc = 2f64.powi(-s);
        let ms: Vec<Vec<f64>> = m.iter().map(|r| r.iter().map(|v| v * sc).collect()).collect();
        let mul = |p: &Vec<Vec<f64>>, q: &Vec<Vec<f64>>| {
            (0..n).map(|i| (0..n).map(|j| (0..n).map(|k| p[i][k] * q[k][j]).sum()).collect()).collect::<Vec<Vec<f64>>>()
        };
        let mut e: Vec<Vec<f64>> = (0..n).map(|i| (0..n).map(|j| if i == j { 1.0 } else { 0.0 }).collect()).collect();
        let mut term = e.clone();
        for k in 1..30 {
            term = mul(&term, &ms);
            for i in 0..n {
                for j in 0..n {
                    term[i][j] /= k as f64;
                    e[i][j] += term[i][j];
                }
            }
        }
        for _ in 0..s {
            e = mul(&e, &e);
        }
        let mut xa = x0.to_vec();
        xa.push(1.0);
        (0..n - 1).map(|i| (0..n).map(|j| e[i][j] * xa[j]).sum()).collect()
    }
}

/// What a board reads at a state: the rail, node 2, and per device index the
/// current the engine publishes (`None` where not compared).
pub(crate) struct Board {
    pub prog: fn(bool) -> Program,
    lin: fn(bool) -> Lin,
    /// (u, v2, currents by device index) at state x, switch closed or not.
    read: fn(&[f64], bool) -> (f64, f64, Vec<Option<f64>>),
    pub name: &'static str,
    nx: usize,
}

const I_LIM: f64 = 0.02;
const R1: f64 = 100.0;
const C1: f64 = 1e-6;
const LL: f64 = 1e-3;
const DCR: f64 = 10.0;
const R2: f64 = 10.0;
const R_SW: f64 = 470.0 + 1e-3;
const NODE_G: f64 = 1e-12;

fn g_rail(closed: bool) -> f64 {
    1.0 / R1 + if closed { 1.0 / R_SW } else { 0.0 } + NODE_G
}

fn base(exact: bool, far: Device) -> Program {
    let options = Options { no_rail_shortcut: exact, ..Options::default() };
    program(4, vec![
        dev(kinds::BENCH_PSU, 0, &[1, 0], vec![5.0, I_LIM]),
        dev(kinds::RESISTOR, 1, &[1, 0], vec![R1]),
        dev(kinds::CAPACITOR, 2, &[1, 2], vec![C1, 0.0, 0.0]),
        far,
        dev(kinds::SWITCH, 4, &[1, 3], vec![0.0]),
        dev(kinds::RESISTOR, 5, &[3, 0], vec![470.0]),
    ], options)
}

// coil: x = [vc, iL]; u = (I - iL) / G; v2 = u - vc.
fn coil_prog(exact: bool) -> Program {
    base(exact, dev(kinds::INDUCTOR, 3, &[2, 0], vec![LL, DCR, 0.0]))
}
fn coil_lin(closed: bool) -> Lin {
    let g = g_rail(closed);
    // diL = (u - vc - dcr iL) / L = (I/G - iL/G - vc - dcr iL) / L
    Lin { a: vec![vec![0.0, 1.0 / C1], vec![-1.0 / LL, -(1.0 / g + DCR) / LL]], b: vec![0.0, I_LIM / g / LL] }
}
fn coil_read(x: &[f64], closed: bool) -> (f64, f64, Vec<Option<f64>>) {
    let u = (I_LIM - x[1]) / g_rail(closed);
    let v2 = u - x[0];
    let isw = if closed { u / R_SW } else { 0.0 };
    (u, v2, vec![Some(-I_LIM), Some(u / R1), Some(x[1]), Some(x[1]), Some(isw), Some(isw)])
}

// res: x = [vc]; i = (u - vc)/R2, u = (I + vc/R2) / (G + 1/R2).
fn res_prog(exact: bool) -> Program {
    base(exact, dev(kinds::RESISTOR, 3, &[2, 0], vec![R2]))
}
fn res_lin(closed: bool) -> Lin {
    let g = g_rail(closed);
    let gt = g + 1.0 / R2;
    // i = (u - vc)/R2 = ((I + vc/R2)/gt - vc)/R2
    Lin { a: vec![vec![(1.0 / (R2 * gt) - 1.0) / R2 / C1]], b: vec![I_LIM / gt / R2 / C1] }
}
fn res_read(x: &[f64], closed: bool) -> (f64, f64, Vec<Option<f64>>) {
    let gt = g_rail(closed) + 1.0 / R2;
    let u = (I_LIM + x[0] / R2) / gt;
    let i = (u - x[0]) / R2;
    let isw = if closed { u / R_SW } else { 0.0 };
    (u, u - x[0], vec![Some(-I_LIM), Some(u / R1), Some(i), Some(i), Some(isw), Some(isw)])
}

// none: the capacitor's far end floats (its node shunt only): u = I / G, the
// capacitor carries nothing (to 1e-12 S).
fn none_prog(exact: bool) -> Program {
    let options = Options { no_rail_shortcut: exact, ..Options::default() };
    program(4, vec![
        dev(kinds::BENCH_PSU, 0, &[1, 0], vec![5.0, I_LIM]),
        dev(kinds::RESISTOR, 1, &[1, 0], vec![R1]),
        dev(kinds::CAPACITOR, 2, &[1, 2], vec![C1, 0.0, 0.0]),
        dev(kinds::SWITCH, 3, &[1, 3], vec![0.0]),
        dev(kinds::RESISTOR, 4, &[3, 0], vec![470.0]),
    ], options)
}
fn none_lin(_closed: bool) -> Lin {
    Lin { a: vec![vec![0.0]], b: vec![0.0] }
}
fn none_read(_x: &[f64], closed: bool) -> (f64, f64, Vec<Option<f64>>) {
    let u = I_LIM / g_rail(closed);
    let isw = if closed { u / R_SW } else { 0.0 };
    (u, u, vec![Some(-I_LIM), Some(u / R1), Some(0.0), Some(isw), Some(isw)])
}

pub(crate) const COIL: Board = Board { prog: coil_prog, lin: coil_lin, read: coil_read, name: "coil", nx: 2 };
pub(crate) const RES: Board = Board { prog: res_prog, lin: res_lin, read: res_read, name: "res", nx: 1 };
pub(crate) const NONE: Board = Board { prog: none_prog, lin: none_lin, read: none_read, name: "none", nx: 1 };

const SAMPLES: [f64; 6] = [1e-6, 10e-6, 100e-6, 2e-4, 5e-4, 1e-3];

/// The truth's state at `t` (switch closing at `t_sw` if any).
fn truth_at(b: &Board, t: f64, t_sw: Option<f64>, closed_form: bool) -> Vec<f64> {
    let x0 = vec![0.0; b.nx];
    let go = |lin: &Lin, x: &[f64], dt: f64| if closed_form { lin.closed(x, dt) } else { lin.truth(x, dt) };
    match t_sw {
        Some(ts) if t > ts => {
            let x1 = go(&(b.lin)(false), &x0, ts);
            go(&(b.lin)(true), &x1, t - ts)
        }
        _ => go(&(b.lin)(false), &x0, t),
    }
}

/// Runs the board to each sample at host cadence `dt` (0: one advance per
/// sample), the switch closed at `t_sw`. Returns per sample (v, i).
fn run(b: &Board, exact: bool, dt: f64, t_sw: Option<f64>) -> Vec<(Vec<f64>, Vec<f64>)> {
    let mut e = Engine::load(&(b.prog)(exact)).unwrap();
    let sw = if b.name == "none" { 3 } else { 4 };
    let mut out = Vec::new();
    let mut stops: Vec<f64> = SAMPLES.to_vec();
    if let Some(ts) = t_sw {
        stops.push(ts);
        stops.sort_by(f64::total_cmp);
        stops.dedup();
    }
    for &t in &stops {
        let mut now = e.time();
        while now < t {
            now = if dt > 0.0 { (now + dt).min(t) } else { t };
            e.advance(now).unwrap();
        }
        if SAMPLES.contains(&t) {
            out.push((e.voltages().to_vec(), e.currents().to_vec()));
        }
        if Some(t) == t_sw {
            e.set_params(&[(sw, 0, 1.0)]).unwrap();
        }
    }
    out
}

/// Prints and returns the worst W0 of `got` against the truth over every
/// node and compared current, per sample.
fn report(b: &Board, label: &str, got: &[(Vec<f64>, Vec<f64>)], t_sw: Option<f64>) -> Vec<f64> {
    let closed_now = |t: f64| t_sw.map_or(false, |ts| t > ts);
    let mut worst = Vec::new();
    for (k, &t) in SAMPLES.iter().enumerate() {
        let x = truth_at(b, t, t_sw, false);
        let (u, v2, cur) = (b.read)(&x, closed_now(t));
        let (v, i) = &got[k];
        let mut parts = vec![("u", v[1], u, w0v(v[1], u)), ("v2", v[2], v2, w0v(v[2], v2))];
        for (d, want) in cur.iter().enumerate() {
            if let Some(w) = want {
                parts.push((["i0", "i1", "i2", "i3", "i4", "i5"][d], i[d], *w, w0i(i[d], *w)));
            }
        }
        let m = parts.iter().map(|p| p.3).fold(0.0, f64::max);
        let s: Vec<String> = parts.iter().map(|p| format!("{}={:.6e}/{:.6e}({:.2})", p.0, p.1, p.2, p.3)).collect();
        if std::env::var("MAGCC_DETAIL").is_ok() {
            eprintln!("{} {label} t={t:e} worst {m:.3} W0: {}", b.name, s.join(" "));
        }
        worst.push(m);
    }
    worst
}

#[test]
fn magcc_truth_rk4_agrees_with_the_closed_form() {
    for b in [&COIL, &RES] {
        for &t in &SAMPLES {
            for t_sw in [None, Some(2e-4)] {
                let (p, q) = (truth_at(b, t, t_sw, false), truth_at(b, t, t_sw, true));
                for (a, c) in p.iter().zip(&q) {
                    assert!((a - c).abs() <= 1e-9 * c.abs().max(1e-6), "{} at {t:e}: RK4 {a:e} closed {c:e}", b.name);
                }
            }
        }
    }
}

/// Prints every board, mode and cadence against the truth (diagnostic).
#[test]
#[ignore]
fn magcc_table() {
    for b in [&COIL, &RES, &NONE] {
        for t_sw in [None, Some(2e-4)] {
            for dt in [1e-6, 10e-6, 0.0] {
                for exact in [false, true] {
                    let label = format!("{} dt={dt:e} sw={:?}", if exact { "exact" } else { "default" }, t_sw);
                    let w = report(b, &label, &run(b, exact, dt, t_sw), t_sw);
                    let s: Vec<String> = w.iter().map(|v| format!("{v:7.3}")).collect();
                    eprintln!("{:4} {label:40} W0 at 1u 10u 100u 200u 500u 1m: {}", b.name, s.join(" "));
                }
            }
        }
    }
}

// ---- M1: a CC entry at load replans the trial planned on the CV rail ---------

/// PSU {5 V, 20 mA}, R1 100 ohm rail-ground, and two 1 uF capacitors in
/// series from the rail to ground (node 2 between them): no capacitor from the
/// rail to a fixed node, so `c_rail` is 0. Cold, the chain holds the rail at 0:
/// the supply is in CC from t = 0 and `s = vc1 + vc2 = u` follows
/// `s(t) = I R1 (1 - exp(-t / tau))`, `tau = R1 C / 2` (closed form by hand).
fn cap_chain(exact: bool) -> Program {
    let options = Options { no_rail_shortcut: exact, ..Options::default() };
    program(3, vec![
        dev(kinds::BENCH_PSU, 0, &[1, 0], vec![5.0, I_LIM]),
        dev(kinds::RESISTOR, 1, &[1, 0], vec![R1]),
        dev(kinds::CAPACITOR, 2, &[1, 2], vec![C1, 0.0, 0.0]),
        dev(kinds::CAPACITOR, 3, &[2, 0], vec![C1, 0.0, 0.0]),
    ], options)
}

fn cap_chain_truth(t: f64) -> (f64, f64, f64) {
    let tau = R1 * C1 / 2.0;
    let u = I_LIM * R1 * -(-t / tau).exp_m1();
    (u, u / 2.0, I_LIM - u / R1)
}

#[test]
fn a_cc_entry_at_load_replans_a_capacitor_chain_on_a_rail_without_rail_capacitors() {
    // MAGCC M1: the chain's partition planned its first trial at load on the
    // CV rail (5 V) and, past no band when linear (and no `replan`: that was
    // R68's, for a rail with capacitors only), committed it in CC: both
    // capacitors charged to 2.5 V over the first step, the rail 4.97 V at
    // 0.1 us against 0.0040 V (1.2e6 W0), 4.71 V at 1 us against 0.039 V.
    // A first host stop after the first step (here 0.1 us, one advance) shows
    // it; a host that first stops inside the first step does not.
    for t in [1e-7, 1e-6] {
        let mut e = Engine::load(&cap_chain(false)).unwrap();
        e.advance(t).unwrap();
        let (u, v2, i) = cap_chain_truth(t);
        let (v, c) = (e.voltages(), e.currents());
        assert!(w0v(v[1], u) <= 1.0, "rail at {t:e}: {} against {u}", v[1]);
        assert!(w0v(v[2], v2) <= 1.0, "node 2 at {t:e}: {} against {v2}", v[2]);
        assert!(w0i(c[2], i) <= 1.0, "chain current at {t:e}: {} against {i}", c[2]);
    }
}

#[test]
fn a_cc_entry_at_load_starts_the_winding_from_the_cc_rail() {
    // MAGCC M1 on the coil board: one advance to 10 ns, the winding 5.0e-5 A
    // (5 V across 1 mH for the first step) against 2.0e-5 (the CC rail's 2 V).
    let mut e = Engine::load(&coil_prog(false)).unwrap();
    e.advance(1e-8).unwrap();
    let x = truth_at(&COIL, 1e-8, None, true);
    let (u, v2, cur) = coil_read(&x, false);
    assert!(w0v(e.voltages()[1], u) <= 1.0 && w0v(e.voltages()[2], v2) <= 1.0, "nodes {:?} against {u} {v2}", &e.voltages()[1..3]);
    let il = cur[3].unwrap();
    assert!(w0i(e.device_state(3).unwrap()[0], il) <= 1.0, "winding {} against {il}", e.device_state(3).unwrap()[0]);
}

// ---- M2 (R99 cause (1)): a CC step is backward Euler ------------------------

/// PSU {5 V, 20 mA}, R1 100 ohm rail-ground, 10 ohm rail-node 2, 1 uF node
/// 2-ground: no capacitor on the rail at all. x = [vc].
fn g_prog(exact: bool) -> Program {
    let options = Options { no_rail_shortcut: exact, ..Options::default() };
    program(3, vec![
        dev(kinds::BENCH_PSU, 0, &[1, 0], vec![5.0, I_LIM]),
        dev(kinds::RESISTOR, 1, &[1, 0], vec![R1]),
        dev(kinds::RESISTOR, 2, &[1, 2], vec![R2]),
        dev(kinds::CAPACITOR, 3, &[2, 0], vec![C1, 0.0, 0.0]),
    ], options)
}
fn g_lin(_closed: bool) -> Lin {
    let gt = 1.0 / R1 + NODE_G + 1.0 / R2;
    // u = (I + vc/R2) / gt, i = (u - vc)/R2, vc' = i / C
    Lin { a: vec![vec![(1.0 / (R2 * gt) - 1.0) / R2 / C1]], b: vec![I_LIM / gt / R2 / C1] }
}
fn g_read(x: &[f64], _closed: bool) -> (f64, f64, Vec<Option<f64>>) {
    let gt = 1.0 / R1 + NODE_G + 1.0 / R2;
    let u = (I_LIM + x[0] / R2) / gt;
    let i = (u - x[0]) / R2;
    (u, x[0], vec![Some(-I_LIM), Some(u / R1), Some(i), Some(i)])
}
pub(crate) const G3: Board = Board { prog: g_prog, lin: g_lin, read: g_read, name: "g", nx: 1 };

// FINDING (MAGCC, reported, not loosened): default mode's CC step is backward
// Euler (R96 left CC steps so; README Open items, "Default-mode CC-rail
// accuracy", R99 cause (1)) where exact mode steps the same partition by
// TR-BDF2 (81514bf): host stops every 1 us, worst W0 at 1u/10u/100u/0.2m/0.5m/1m
// (6c5a152): g 0.10/0.73/1.85/3.91/10.1/13.8, res 0.06/0.57/1.95/4.01/10.2/13.9,
// coil 11.4/7.6/1.8/4.4/11.8/12.3 (with M1 fixed 2.7/8.8/2.0/4.5/12.0/12.4);
// exact mode <= 0.08 on all three. Ignored until R99 (1) is ruled.
#[test]
#[ignore]
fn a_cc_rail_partition_reads_within_w0_of_the_truth_at_1_us_host_stops() {
    for b in [&G3, &RES, &COIL] {
        let w = report(b, "default dt=1e-6", &run(b, false, 1e-6, None), None);
        for (k, v) in w.iter().enumerate() {
            assert!(*v <= 1.0, "{} at {:e}: {v} W0", b.name, SAMPLES[k]);
        }
    }
}
