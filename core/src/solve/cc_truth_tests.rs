//! CCT: a bench supply in current limit (CC) read against a strong truth.
//!
//! The diagnosis CCD found seven mechanisms (M1 to M7) by which the core
//! reads a bench supply in CC wrongly. This file makes it durable: each
//! mechanism's minimal board is a failing-by-design `#[ignore]` test that
//! carries its own truth, the repository's bench-supply fixtures that read
//! right are control tests that pass today, and the first blow-up
//! (M3's +4.4e6 A / -4.4e6 A) is its own test. Nothing here fixes the engine
//! or changes a default. The assertion in each ignored test is the promise
//! (W0 against the truth) and is never weakened: when a mechanism is fixed the
//! `#[ignore]` comes off and the test passes.
//!
//! THE TRUTH (this module, no core output): the board's own circuit ODE,
//! integrated by a fixed-step RK4, with the supply as its law (constant
//! voltage at the setpoint while the current it must deliver is within the
//! limit, constant current at the limit otherwise; each switch-over found by
//! bisection on the step). It is the diagnosis's `rk.rs` (kept in
//! `ship-logs/ccd/harness/` of the drafts folder) in a smaller form: the
//! unknowns are node voltages. A capacitor with an ESR is a resistor to an
//! internal node plus an ideal capacitor from that node, so every capacitor
//! is an ideal one between two nodes; nodes that touch a capacitor are
//! dynamic (`M v' = r`, `M` the capacitance matrix over the dynamic nodes),
//! the others algebraic (solved exactly at each stage, Newton on LEDs with
//! the engine's own junction law). A closed switch is 1 mOhm, every node
//! leaks 1e-12 S (the engine's shunt), the LED carries GMIN_D, as the engine
//! stamps them. A capacitive loop of ideal capacitors (M3) needs no special
//! case: `M` stays regular as long as every capacitor cluster reaches a
//! pinned node.
//!
//! HALVING CHECK: the truth runs at a step `h` and at `h / 2`; the step is
//! halved until every reading (every node voltage, every part current) at
//! every compared instant moves by under 1e-9 relative to that instant's
//! largest reading (largest |v|, largest |i|: a per-quantity relative gap is
//! roundoff in cancelling branch currents). The test fails if that is not
//! reached within `MAX_HALVINGS` halvings.
//!
//! W0 is 100 uV + 1e-3 |v| for a node voltage and 1 nA + 1e-3 |i| for a part
//! current. The boards are the diagnosis's (core/ ship-logs ccd NOTES,
//! magcc NOTES); the diagnosis's numbers are on 1753e39.
//!
//! Set `CC_TRUTH_DETAIL=1` to print every compared reading of a run.

use crate::devices::led::{JunctionLaw, GMIN_D};
use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

// ---- boards -------------------------------------------------------------------

fn dv(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    Device {
        kind,
        component_index: ci,
        pins: pins.iter().map(|&n| PinRecord { node: n, flags: 0, open_hash: 0, dead_hash: 0 }).collect(),
        params,
        blob,
    }
}

/// A board as the host drives it: the parts, the instants the readings are
/// compared at (a toggle due at an instant is applied after that instant's
/// reading), the host's advance cadence between them (0: one advance per
/// instant) and the truth's starting step.
struct Board {
    name: &'static str,
    nc: usize,
    devs: Vec<Device>,
    /// (time, device, value of its param 0).
    toggles: Vec<(f64, usize, f64)>,
    stops: Vec<f64>,
    cadence: f64,
    h0: f64,
}

impl Board {
    fn new(name: &'static str, psu: (f64, f64), nc: usize, h0: f64, cadence: f64, stops: &[f64]) -> Board {
        let devs = vec![dv(kinds::BENCH_PSU, 0, &[1, 0], vec![psu.0, psu.1], vec![])];
        Board { name, nc, devs, toggles: vec![], stops: stops.to_vec(), cadence, h0 }
    }
    fn add(mut self, kind: u32, pins: &[usize], params: Vec<f64>) -> Board {
        let ci = self.devs.len();
        self.devs.push(dv(kind, ci, pins, params, vec![]));
        self
    }
    fn r(self, a: usize, b: usize, ohm: f64) -> Board {
        self.add(kinds::RESISTOR, &[a, b], vec![ohm])
    }
    fn c(self, a: usize, b: usize, farad: f64, esr: f64) -> Board {
        self.add(kinds::CAPACITOR, &[a, b], vec![farad, esr, 0.0])
    }
    fn sw(self, a: usize, b: usize, closed: bool) -> Board {
        self.add(kinds::SWITCH, &[a, b], vec![if closed { 1.0 } else { 0.0 }])
    }
    fn led(self, a: usize, b: usize, vf: f64, n: f64, i_rated: f64) -> Board {
        self.add(kinds::LED, &[a, b], vec![vf, n, i_rated, 0.0, 0.0, 0.0])
    }
    fn toggle(mut self, t: f64, dev: usize, value: f64) -> Board {
        self.toggles.push((t, dev, value));
        self
    }
    fn program(&self, exact: bool) -> Program {
        let options = Options { no_rail_shortcut: exact, ..Options::default() };
        Program { node_count: self.nc, options, digital_slot_count: 32, channel_slot_count: 0, devices: self.devs.clone(), carry: Vec::new() }
    }
}


// ---- the truth ----------------------------------------------------------------

/// How many times the step is halved before the truth gives up (the test
/// fails): h0 / 2^12 is far below any step these boards need.
const MAX_HALVINGS: u32 = 12;
const HALVING_TOL: f64 = 1e-9;
const NODE_LEAK: f64 = 1e-12;
const SWITCH_G: f64 = 1e3;

fn gauss(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for col in 0..n {
        let p = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs())).unwrap();
        assert!(a[p][col] != 0.0, "the truth's network is singular");
        a.swap(col, p);
        b.swap(col, p);
        for r in col + 1..n {
            let f = a[r][col] / a[col][col];
            if f != 0.0 {
                for k in col..n {
                    a[r][k] -= f * a[col][k];
                }
                b[r] -= f * b[col];
            }
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let s: f64 = (r + 1..n).map(|k| a[r][k] * x[k]).sum();
        x[r] = (b[r] - s) / a[r][r];
    }
    x
}

struct CapEl {
    dev: usize,
    /// Pins of the device and the ideal capacitor's two nodes (the internal
    /// node replaces `p` when the ESR is not zero).
    p: usize,
    int: Option<usize>,
    a: usize,
    b: usize,
    c: f64,
    esr: f64,
}

struct Net<'a> {
    bd: &'a Board,
    /// Nodes including the internal ones.
    n: usize,
    caps: Vec<CapEl>,
    leds: Vec<(usize, usize, usize, JunctionLaw)>,
    vset: f64,
    ilim: f64,
    /// A capacitor with a pin on the rail (the supply leaves CC when the rail reaches the setpoint).
    railcap: bool,
    /// An ideal capacitor straight on the rail (the rail is dynamic in CC).
    rail_direct: bool,
}

/// What the network reads at a state.
struct Ev {
    v: Vec<f64>,
    dv: Vec<f64>,
    /// The current the supply delivers (positive when sourcing) with the rail pinned; the CC law otherwise.
    delivered: f64,
}

impl<'a> Net<'a> {
    fn new(bd: &'a Board) -> Net<'a> {
        let mut n = bd.nc;
        let (mut caps, mut leds) = (vec![], vec![]);
        for (k, d) in bd.devs.iter().enumerate() {
            let (p, q) = (d.pins.first().map_or(0, |x| x.node), d.pins.get(1).map_or(0, |x| x.node));
            match d.kind {
                kinds::CAPACITOR => {
                    let (c, esr) = (d.params[0], d.params[1]);
                    if esr > 0.0 {
                        caps.push(CapEl { dev: k, p, int: Some(n), a: n, b: q, c, esr });
                        n += 1;
                    } else {
                        caps.push(CapEl { dev: k, p, int: None, a: p, b: q, c, esr });
                    }
                }
                kinds::LED => leds.push((k, p, q, JunctionLaw::at(d.params[0], d.params[2], d.params[1], None, 25.0))),
                kinds::BENCH_PSU | kinds::RESISTOR | kinds::SWITCH => {}
                other => panic!("the truth has no model for kind {other}"),
            }
        }
        let railcap = caps.iter().any(|c| c.p == 1 || c.b == 1);
        let rail_direct = caps.iter().any(|c| c.int.is_none() && (c.a == 1 || c.b == 1));
        Net { bd, n, caps, leds, vset: bd.devs[0].params[0], ilim: bd.devs[0].params[1], railcap, rail_direct }
    }

    /// The network at node voltages `v_in` (the dynamic ones are the state):
    /// the algebraic nodes solved (Newton when an LED is on one), then the
    /// dynamic nodes' rates `dv` from `M dv = r`.
    fn eval(&self, cv: bool, closed: &[bool], v_in: &[f64]) -> Ev {
        let n = self.n;
        let pinned = |k: usize| k == 0 || (cv && k == 1);
        let mut dynamic = vec![false; n];
        for c in &self.caps {
            for k in [c.a, c.b] {
                if !pinned(k) {
                    dynamic[k] = true;
                }
            }
        }
        let alg: Vec<usize> = (1..n).filter(|&k| !pinned(k) && !dynamic[k]).collect();
        let dyn_nodes: Vec<usize> = (1..n).filter(|&k| dynamic[k]).collect();
        let mut v = v_in.to_vec();
        v[0] = 0.0;
        if cv {
            v[1] = self.vset;
        }
        let (mut g, mut rhs);
        let mut it = 0;
        loop {
            g = vec![vec![0.0; n]; n];
            rhs = vec![0.0; n];
            let stamp = |g: &mut Vec<Vec<f64>>, a: usize, b: usize, s: f64| {
                g[a][a] += s;
                g[b][b] += s;
                g[a][b] -= s;
                g[b][a] -= s;
            };
            for k in 1..n {
                stamp(&mut g, k, 0, NODE_LEAK);
            }
            for (k, d) in self.bd.devs.iter().enumerate() {
                let (p, q) = (d.pins.first().map_or(0, |x| x.node), d.pins.get(1).map_or(0, |x| x.node));
                match d.kind {
                    kinds::RESISTOR => stamp(&mut g, p, q, 1.0 / d.params[0].max(1e-3)),
                    kinds::SWITCH if closed[k] => stamp(&mut g, p, q, SWITCH_G),
                    _ => {}
                }
            }
            for c in &self.caps {
                if let Some(i) = c.int {
                    stamp(&mut g, c.p, i, 1.0 / c.esr);
                }
            }
            for &(_, p, q, law) in &self.leds {
                let vd = v[p] - v[q];
                let vc = vd.min(law.vsat);
                let gd = law.is * (vc / law.vtn).exp() / law.vtn;
                stamp(&mut g, p, q, gd + GMIN_D);
                let ieq = law.current(vd) - gd * vd;
                rhs[p] -= ieq;
                rhs[q] += ieq;
            }
            if !cv {
                rhs[1] += self.ilim;
            }
            if alg.is_empty() {
                break;
            }
            let a: Vec<Vec<f64>> = alg.iter().map(|&i| alg.iter().map(|&j| g[i][j]).collect()).collect();
            let b: Vec<f64> = alg
                .iter()
                .map(|&i| rhs[i] - (0..n).filter(|j| !alg.contains(j)).map(|j| g[i][j] * v[j]).sum::<f64>())
                .collect();
            let x = gauss(a, b);
            let moved = alg.iter().zip(&x).fold(0.0f64, |m, (&i, &xi)| m.max((xi - v[i]).abs()));
            for (&i, &xi) in alg.iter().zip(&x) {
                v[i] = xi;
            }
            it += 1;
            let scale = v.iter().fold(0.0f64, |m, z| m.max(z.abs()));
            if self.leds.is_empty() || moved <= 1e-15 + 1e-14 * scale {
                break;
            }
            assert!(it < 200, "the truth's Newton did not converge");
        }
        // the dynamic nodes: M dv = r, r the resistive inflow
        let mut dvv = vec![0.0; n];
        if !dyn_nodes.is_empty() {
            let pos = |k: usize| dyn_nodes.iter().position(|&z| z == k);
            let mut m = vec![vec![0.0; dyn_nodes.len()]; dyn_nodes.len()];
            for c in &self.caps {
                let (pa, pb) = (pos(c.a), pos(c.b));
                if let Some(i) = pa {
                    m[i][i] += c.c;
                }
                if let Some(j) = pb {
                    m[j][j] += c.c;
                }
                if let (Some(i), Some(j)) = (pa, pb) {
                    m[i][j] -= c.c;
                    m[j][i] -= c.c;
                }
            }
            let r: Vec<f64> = dyn_nodes.iter().map(|&i| rhs[i] - (0..n).map(|j| g[i][j] * v[j]).sum::<f64>()).collect();
            let x = gauss(m, r);
            for (&i, &xi) in dyn_nodes.iter().zip(&x) {
                dvv[i] = xi;
            }
        }
        let delivered = if cv {
            let resistive: f64 = (0..n).map(|j| g[1][j] * v[j]).sum::<f64>() - rhs[1];
            let capout: f64 = self
                .caps
                .iter()
                .map(|c| if c.a == 1 { c.c * (dvv[1] - dvv[c.b]) } else if c.b == 1 { c.c * (dvv[1] - dvv[c.a]) } else { 0.0 })
                .sum();
            resistive + capout
        } else {
            self.ilim
        };
        Ev { v, dv: dvv, delivered }
    }

    /// The current each device publishes at an evaluated state.
    fn currents(&self, ev: &Ev, closed: &[bool], cv: bool) -> Vec<f64> {
        let mut out = vec![f64::NAN; self.bd.devs.len()];
        for (k, d) in self.bd.devs.iter().enumerate() {
            let (p, q) = (d.pins.first().map_or(0, |x| x.node), d.pins.get(1).map_or(0, |x| x.node));
            out[k] = match d.kind {
                kinds::RESISTOR => (ev.v[p] - ev.v[q]) / d.params[0].max(1e-3),
                kinds::SWITCH => {
                    if closed[k] {
                        (ev.v[p] - ev.v[q]) * SWITCH_G
                    } else {
                        0.0
                    }
                }
                kinds::LED => self.leds.iter().find(|l| l.0 == k).map_or(0.0, |l| l.3.current(ev.v[p] - ev.v[q])),
                kinds::CAPACITOR => {
                    let c = self.caps.iter().find(|c| c.dev == k).unwrap();
                    match c.int {
                        Some(i) => (ev.v[c.p] - ev.v[i]) / c.esr,
                        None => c.c * (ev.dv[c.a] - ev.dv[c.b]),
                    }
                }
                kinds::BENCH_PSU => -ev.delivered,
                _ => f64::NAN,
            };
        }
        let _ = cv;
        out
    }

    /// Does the supply leave the regime it is in at state `v`?
    fn leave(&self, cv: bool, closed: &[bool], v: &[f64]) -> bool {
        if cv {
            self.eval(true, closed, v).delivered > self.ilim * (1.0 + 1e-12) + 1e-15
        } else if self.railcap {
            self.eval(false, closed, v).v[1] >= self.vset
        } else {
            self.eval(true, closed, v).delivered <= self.ilim
        }
    }

    fn rk4(&self, cv: bool, closed: &[bool], v: &[f64], h: f64) -> Vec<f64> {
        let adv = |v: &[f64], k: &[f64], s: f64| v.iter().zip(k).map(|(a, b)| a + s * b).collect::<Vec<_>>();
        let k1 = self.eval(cv, closed, v).dv;
        let k2 = self.eval(cv, closed, &adv(v, &k1, h / 2.0)).dv;
        let k3 = self.eval(cv, closed, &adv(v, &k2, h / 2.0)).dv;
        let k4 = self.eval(cv, closed, &adv(v, &k3, h)).dv;
        (0..self.n).map(|i| v[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])).collect()
    }
}

/// One instant's readings: node voltages (the program's nodes) and the
/// current of every device (NaN where the truth has none).
#[derive(Clone)]
struct Reading {
    v: Vec<f64>,
    i: Vec<f64>,
    /// The supply was in CC at the reading.
    cc: bool,
}

struct Truth {
    at: Vec<Reading>,
    /// The instants the supply changed regime at.
    switches: Vec<f64>,
}

/// The truth at step `h`: the supply's CV / CC law, each switch-over found by
/// bisection on the step.
fn truth_at_step(bd: &Board, h: f64) -> Truth {
    let net = Net::new(bd);
    let mut closed: Vec<bool> = bd.devs.iter().map(|d| d.kind == kinds::SWITCH && d.params[0] == 1.0).collect();
    let mut v = vec![0.0; net.n];
    // the engine's load rule: CV iff the draw at the setpoint is within the limit; a rail with an ideal
    // capacitor on it, at 0 V, is CC (the capacitor would draw without bound)
    let mut cv = if net.rail_direct { false } else { !net.leave(true, &closed, &v) };
    if cv {
        v[1] = net.vset;
    }
    let mut out = Truth { at: vec![], switches: vec![] };
    let mut t = 0.0f64;
    let mut ev = 0;
    let mut toggles = bd.toggles.clone();
    toggles.sort_by(|a, b| a.0.total_cmp(&b.0));
    for &tb in &bd.stops {
        while t < tb {
            let hh = (tb - t).min(h);
            let hh = if tb - t - hh < 1e-3 * h { tb - t } else { hh };
            let vn = net.rk4(cv, &closed, &v, hh);
            if net.leave(cv, &closed, &vn) {
                let (mut lo, mut hi) = (0.0, hh);
                for _ in 0..100 {
                    let mid = 0.5 * (lo + hi);
                    if mid <= lo || mid >= hi {
                        break;
                    }
                    if net.leave(cv, &closed, &net.rk4(cv, &closed, &v, mid)) {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                v = net.rk4(cv, &closed, &v, hi);
                t += hi;
                cv = !cv;
                if cv {
                    v[1] = net.vset;
                }
                out.switches.push(t);
                assert!(out.switches.len() < 5000, "the truth's supply chatters");
                continue;
            }
            v = vn;
            t += hh;
        }
        t = tb;
        let e = net.eval(cv, &closed, &v);
        out.at.push(Reading { v: e.v[..bd.nc].to_vec(), i: net.currents(&e, &closed, cv), cc: !cv });
        let mut toggled = false;
        while ev < toggles.len() && toggles[ev].0 <= tb {
            closed[toggles[ev].1] = toggles[ev].2 == 1.0;
            ev += 1;
            toggled = true;
        }
        if toggled && net.leave(cv, &closed, &v) {
            cv = !cv;
            if cv {
                v[1] = net.vset;
            }
            out.switches.push(t);
        }
    }
    out
}

/// The truth to the halving check: the step is halved until no reading at any
/// compared instant moves by 1e-9 relative to that instant's largest. Fails
/// the test when `MAX_HALVINGS` halvings do not get there. Returns the
/// readings, the step reached and the gap there.
fn truth(bd: &Board) -> (Truth, f64, f64) {
    let mut h = bd.h0;
    let mut a = truth_at_step(bd, h);
    for _ in 0..MAX_HALVINGS {
        let b = truth_at_step(bd, h / 2.0);
        let mut gap = 0.0f64;
        for (p, q) in a.at.iter().zip(&b.at) {
            let sv = q.v.iter().fold(1e-6f64, |m, z| m.max(z.abs()));
            let si = q.i.iter().filter(|z| !z.is_nan()).fold(1e-9f64, |m, z| m.max(z.abs()));
            for (x, y) in p.v.iter().zip(&q.v) {
                gap = gap.max((x - y).abs() / sv);
            }
            for (x, y) in p.i.iter().zip(&q.i) {
                if !y.is_nan() {
                    gap = gap.max((x - y).abs() / si);
                }
            }
        }
        if a.switches.len() != b.switches.len() {
            gap = gap.max(1.0);
        }
        if gap < HALVING_TOL {
            return (b, h / 2.0, gap);
        }
        h /= 2.0;
        a = b;
    }
    panic!("{}: the truth's readings still move by more than {HALVING_TOL:e} after {MAX_HALVINGS} halvings from h = {:e}", bd.name, bd.h0);
}

// ---- the engine's readings and W0 ---------------------------------------------

fn w0v(got: f64, want: f64) -> f64 {
    (got - want).abs() / (100e-6 + 1e-3 * want.abs())
}
fn w0i(got: f64, want: f64) -> f64 {
    (got - want).abs() / (1e-9 + 1e-3 * want.abs())
}

/// The engine's readings at each of the board's instants, the host advancing
/// at the board's cadence, toggles applied after the reading due at them.
fn run_engine(bd: &Board, exact: bool) -> Vec<(Vec<f64>, Vec<f64>)> {
    let mut e = Engine::load(&bd.program(exact)).unwrap();
    let mut toggles = bd.toggles.clone();
    toggles.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut ev = 0;
    let mut out = vec![];
    for &t in &bd.stops {
        let mut now = e.time();
        while now < t {
            now = if bd.cadence > 0.0 { (now + bd.cadence).min(t) } else { t };
            e.advance(now).unwrap();
        }
        out.push((e.voltages().to_vec(), e.currents().to_vec()));
        while ev < toggles.len() && toggles[ev].0 <= t {
            e.set_params(&[(toggles[ev].1 as u32, 0, toggles[ev].2)]).unwrap();
            ev += 1;
        }
    }
    out
}

/// The worst W0 of the engine against the truth over every node and device
/// current, per instant; prints every reading under `CC_TRUTH_DETAIL`.
fn worst_w0(bd: &Board, label: &str, got: &[(Vec<f64>, Vec<f64>)], tr: &Truth) -> Vec<(f64, String)> {
    let detail = std::env::var("CC_TRUTH_DETAIL").is_ok();
    let mut out = vec![];
    for (k, &t) in bd.stops.iter().enumerate() {
        let (v, i) = &got[k];
        let want = &tr.at[k];
        let mut worst = (0.0f64, String::from("-"));
        for (j, (&g, &w)) in v.iter().zip(&want.v).enumerate() {
            let x = w0v(g, w);
            if detail {
                eprintln!("{} {label} t={t:e} v{j}: engine {g:.9e} truth {w:.9e} ({x:.3} W0)", bd.name);
            }
            if x > worst.0 {
                worst = (x, format!("v{j} engine {g:e} truth {w:e}"));
            }
        }
        for (j, (&g, &w)) in i.iter().zip(&want.i).enumerate() {
            if w.is_nan() {
                continue;
            }
            let x = w0i(g, w);
            if detail {
                eprintln!("{} {label} t={t:e} i{j}: engine {g:.9e} truth {w:.9e} ({x:.3} W0)", bd.name);
            }
            if x > worst.0 {
                worst = (x, format!("i{j} engine {g:e} truth {w:e}"));
            }
        }
        out.push(worst);
    }
    out
}

/// Engine against truth in one mode: the worst W0 over the instants, with a
/// line saying where and what. Honest about the truth's own halving gap.
fn judge(bd: &Board, tr: &(Truth, f64, f64), exact: bool) -> (f64, String) {
    let (tr, h, gap) = (&tr.0, tr.1, tr.2);
    let label = if exact { "exact" } else { "default" };
    let w = worst_w0(bd, label, &run_engine(bd, exact), tr);
    let (k, (x, what)) = w.iter().enumerate().fold((0, (0.0, String::from("-"))), |m, (k, z)| if z.0 > (m.1).0 { (k, z.clone()) } else { m });
    let per: Vec<String> = bd.stops.iter().zip(&w).map(|(t, z)| format!("{t:.3e}:{:.3}", z.0)).collect();
    (
        x,
        format!(
            "{} {label} mode: worst {x:.3} W0 at t = {:e} ({what}); per instant {}; truth h = {h:.2e}, halving gap {gap:.1e}",
            bd.name,
            bd.stops[k],
            per.join(" ")
        ),
    )
}

// ---- the mechanisms' minimal boards ---------------------------------------------

const MS: f64 = 1e-3;
const US: f64 = 1e-6;
const STOPS_US_MS: [f64; 4] = [US, 10.0 * US, 100.0 * US, MS];

/// M1: PSU 5 V / 20 mA, R 100 rail-ground, C 1 uF rail-n2, C 1 uF n2-ground
/// (no capacitor from the rail to a fixed node): CC from t = 0.
fn board_m1() -> Board {
    Board::new("M1", (5.0, 0.02), 3, 5e-8, US, &STOPS_US_MS).r(1, 0, 100.0).c(1, 2, 1e-6, 0.0).c(2, 0, 1e-6, 0.0)
}

/// M2: PSU 5 V / 20 mA, R 100 rail-ground, R 10 rail-n2, C 1 uF n2-ground
/// (no capacitor on the rail at all): CC from t = 0.
fn board_m2() -> Board {
    Board::new("M2", (5.0, 0.02), 3, 5e-8, US, &[US, 10.0 * US, 100.0 * US, 200.0 * US, MS])
        .r(1, 0, 100.0)
        .r(1, 2, 10.0)
        .c(2, 0, 1e-6, 0.0)
}

/// M3: PSU 5.164233 V / 0.66476 A; C10 1.857 uF rail-ground, C15 78.12 uF
/// rail-n5 (node 3), C50 0.6926 uF n5-ground (an esr-0 capacitive loop
/// through the rail), C21 0.1388 uF n2-rail, R20 384631 ohm n2-ground; CC from t = 0.
fn board_m3() -> Board {
    Board::new("M3", (5.164233, 0.66476), 4, 2e-8, 0.0, &[US, 10.0 * US, 20.0 * US, 100.0 * US, MS])
        .c(1, 0, 1.857e-6, 0.0)
        .c(1, 3, 78.12e-6, 0.0)
        .c(3, 0, 0.6926e-6, 0.0)
        .c(2, 1, 0.1388e-6, 0.0)
        .r(2, 0, 384631.0)
}

/// M4: PSU 5 V / 10 mA, LED rail-ground (vf 2 V, n 2, rated 20 mA), C 1 uF
/// rail-ground: the rail settles onto its CC operating point (tau n Vt C / I
/// = 5.1 us).
fn board_m4() -> Board {
    Board::new("M4", (5.0, 0.01), 2, 1e-7, 0.0, &[US, 10.0 * US, 100.0 * US, MS, 3.0 * MS]).led(1, 0, 2.0, 2.0, 0.02).c(1, 0, 1e-6, 0.0)
}

/// M5: PSU 5 V / 30 mA, R 100 rail-ground, R 10 rail-n2 + C 1 uF n2-ground,
/// R 20 rail-n3 + C 2.2 uF n3-ground (two coupled hub partitions on a CC
/// rail, no rail capacitor).
fn board_m5() -> Board {
    Board::new("M5", (5.0, 0.03), 4, 1e-7, 0.0, &STOPS_US_MS)
        .r(1, 0, 100.0)
        .r(1, 2, 10.0)
        .c(2, 0, 1e-6, 0.0)
        .r(1, 3, 20.0)
        .c(3, 0, 2.2e-6, 0.0)
}

/// M6: PSU 3.082 V / 0.3323 A, C 1.754 uF esr 4.737 ohm rail-ground, R 10
/// rail-ground (a CC rail capacitor with an ESR; CC exits at 57.80 us).
fn board_m6() -> Board {
    Board::new("M6", (3.082, 0.3323), 2, 2e-7, 0.0, &[US, 10.0 * US, 30.0 * US, 100.0 * US, MS]).c(1, 0, 1.754e-6, 4.737).r(1, 0, 10.0)
}

/// M7: PSU 4.862 V / 22 mA, switch rail-ground (closed), R 26.2 rail-ground,
/// R 11.7 rail-n2, C 0.33 uF n2-ground; the switch opens at 10 ms (the supply
/// stays in CC across the toggle, the rail has no capacitor).
fn board_m7() -> Board {
    let t = 10.0 * MS;
    Board::new("M7", (4.862, 0.022), 3, 2e-7, 0.0, &[t, t + 0.1 * US, t + US, t + 10.0 * US, t + 100.0 * US])
        .sw(1, 0, true)
        .r(1, 0, 26.2)
        .r(1, 2, 11.7)
        .c(2, 0, 0.33e-6, 0.0)
        .toggle(t, 1, 0.0)
}

/// The promise of one mechanism's board: the engine within W0 of the truth in
/// `main_exact`'s mode (the mode the mechanism lives in) and, where the
/// diagnosis found the other mode right, in that mode too (so a fix cannot
/// trade one for the other). Both are measured before either is asserted.
fn promise(bd: &Board, main_exact: bool, companion: bool) {
    let tr = truth(bd);
    let (xm, lm) = judge(bd, &tr, main_exact);
    let comp = if companion { Some(judge(bd, &tr, !main_exact)) } else { None };
    let cl = comp.as_ref().map_or(String::from("no companion"), |c| format!("companion: {}", c.1));
    assert!(xm <= 1.0, "{lm}; {cl}");
    if let Some((xc, lc)) = comp {
        assert!(xc <= 1.0, "the other mode: {lc}");
    }
}

// ---- the truth itself -----------------------------------------------------------

/// The closed form of the two linear boards: a single exponential. M1's
/// series pair is one capacitor C/2 across the rail's R1; M2's rail is
/// algebraic and the capacitor sees R2 against the rest.
fn closed_m1(t: f64) -> f64 {
    let g = 1.0 / 100.0 + NODE_LEAK;
    let c = 0.5e-6;
    0.02 / g * -(-t * g / c).exp_m1()
}
fn closed_m2(t: f64) -> (f64, f64) {
    // x = vc; u = (I + x/R2) / gt; x' = (u - x) / (R2 C)
    let gt = 1.0 / 100.0 + NODE_LEAK + 1.0 / 10.0;
    let a = ((1.0 / (10.0 * gt)) - 1.0) / (10.0 * 1e-6);
    let b = 0.02 / gt / (10.0 * 1e-6);
    let x = (b / -a) * -(a * t).exp_m1();
    ((0.02 + x / 10.0) / gt, x)
}

#[test]
fn the_truth_agrees_with_the_closed_form_where_a_board_has_one() {
    let (m1, h1, g1) = truth(&board_m1());
    for (k, &t) in board_m1().stops.iter().enumerate() {
        let u = closed_m1(t);
        assert!((m1.at[k].v[1] - u).abs() <= 1e-7 * u.abs(), "M1 rail at {t:e}: RK4 {} closed {u} (h {h1:e}, gap {g1:e})", m1.at[k].v[1]);
        assert!((m1.at[k].v[2] - u / 2.0).abs() <= 1e-7 * u.abs(), "M1 node 2 at {t:e}");
    }
    let (m2, h2, g2) = truth(&board_m2());
    for (k, &t) in board_m2().stops.iter().enumerate() {
        let (u, x) = closed_m2(t);
        assert!((m2.at[k].v[1] - u).abs() <= 1e-7 * u.abs(), "M2 rail at {t:e}: RK4 {} closed {u} (h {h2:e}, gap {g2:e})", m2.at[k].v[1]);
        assert!((m2.at[k].v[2] - x).abs() <= 1e-7 * x.abs(), "M2 node 2 at {t:e}: RK4 {} closed {x}", m2.at[k].v[2]);
    }
}

#[test]
fn the_truth_reproduces_the_diagnosis_figures_on_the_other_boards() {
    // The diagnosis's own, independent truths (m3truth.py for M3; rk.rs for the
    // rest): M3 rail 2.613537 V at 10 us, in CV (5.164233) from 19.76 us; M4
    // rail 1.964383 V at 1 ms; M5 rail 2.839400 V at 1 ms; M6 rail 2.616554 V
    // at 30 us, exit at 57.80 us; M7 rail 0.181127 V at +0.1 us, 0.208572 V at +1 us.
    let near = |got: f64, want: f64, what: &str| assert!((got - want).abs() <= 2e-6 * want.abs().max(1.0), "{what}: truth {got} diagnosis {want}");
    let (t3, _, _) = truth(&board_m3());
    near(t3.at[1].v[1], 2.613537, "M3 rail at 10 us");
    assert!(t3.at[1].cc && !t3.at[2].cc && !t3.at[3].cc, "M3: CC at 10 us, CV from 19.76 us");
    near(t3.at[3].v[1], 5.164233, "M3 rail at 100 us");
    assert!((t3.switches[0] - 19.76e-6).abs() < 5e-9, "M3 exit {:e}", t3.switches[0]);
    let (t4, _, _) = truth(&board_m4());
    near(t4.at[3].v[1], 1.964383, "M4 rail at 1 ms");
    let (t5, _, _) = truth(&board_m5());
    near(t5.at[3].v[1], 2.839400, "M5 rail at 1 ms");
    let (t6, _, _) = truth(&board_m6());
    near(t6.at[2].v[1], 2.616554, "M6 rail at 30 us");
    assert!((t6.switches[0] - 57.80e-6).abs() < 5e-8, "M6 exit {:e}", t6.switches[0]);
    let (t7, _, _) = truth(&board_m7());
    near(t7.at[1].v[1], 0.181127, "M7 rail at +0.1 us");
    near(t7.at[2].v[1], 0.208572, "M7 rail at +1 us");
}


// ---- one test per mechanism -------------------------------------------------------

/// M1 (diagnosis CCD / MAGCC, "board f"). Board: PSU 5 V / 20 mA, R 100
/// rail-ground, C 1 uF rail-n2, C 1 uF n2-ground, no capacitor from the rail to
/// a fixed node; CC from t = 0 (the rail climbs to I R1 = 2 V with tau R1 C / 2
/// = 50 us); the host steps 1 us. Default mode plans the hub partition's first
/// trial at load on the CV rail (5 V) and commits it: the CC entry at t = 0 does
/// not re-solve it, so the capacitors keep a 5 V first step.
///
/// Truth (RK4, closed form 0.02 * 100 * (1 - exp(-t / 50 us)) agrees): rail
/// 0.039603 V / 0.36254 V / 1.7293 V / 2.0000 V at 1 us / 10 us / 100 us / 1 ms.
/// Default mode on 1753e39: rail 4.9413 / 4.4580 / 2.4095 / 2.0000 V; worst
/// 123,460 W0 at 1 us (the R current 4.94e-2 A against 3.96e-4 A), 11,294 W0 at
/// 10 us, 2,512 W0 at 100 us, 0.1 W0 at 1 ms (rail 35,112 W0 at 1 us). Exact
/// mode (the companion, right today): 0.012 W0 worst.
///
/// Error in W0: see above. Builds: default mode identical on 0.7.0 (c823274),
/// d9d9dfa, 6c5a152, 54ec2fc and 1753e39 (NOTES: 1.2e5 W0 on 0.7.0). Held open by
/// R152 (M1 held on wip/magcc bdca0f0, which is not in 1753e39 and leaves 1.9 W0
/// of M2) and R153 (repair order M3, then M1 with M2, then M7, M5, M4, M6).
/// Expected to pass, with the ignore removed, when M1 and M2 are both fixed.
#[test]
#[ignore = "M1 (R152): a CC entry at load keeps the first trial planned on the CV rail; held open, wip/magcc's bdca0f0 is a partial fix"]
fn m1_a_cc_entry_at_load_replans_the_first_trial_of_a_capacitor_chain() {
    promise(&board_m1(), false, true);
}

/// M2 (R99 cause (1); MAGCC "board g"). Board: PSU 5 V / 20 mA, R 100
/// rail-ground, R 10 rail-n2, C 1 uF n2-ground: no capacitor on the rail at all;
/// CC from t = 0; the host steps 1 us. Default mode's CC steps are plain backward
/// Euler (exact mode's rail partition steps TR-BDF2).
///
/// Truth (RK4 = closed form): rail 0.19827 / 0.33982 / 1.2675 / 1.7049 /
/// 1.9998 V at 1 us / 10 us / 100 us / 200 us / 1 ms. Default mode on 1753e39:
/// 0.19826 / 0.33964 / 1.2661 / 1.7037 / 1.9998 V; worst W0 per instant 0.095 /
/// 0.73 / 1.85 / 3.91 / 13.84, the 13.84 W0 at 1 ms on the capacitor current
/// (2.0910e-6 A against 2.0488e-6 A). Exact mode (companion, right today):
/// 0.005 W0 worst.
///
/// Builds: default mode identical on 0.7.0, d9d9dfa, 6c5a152, 54ec2fc, 1753e39
/// (NOTES: 3.9 W0 at 0.2 ms, 13.8 at 1 ms). Held open by R152 (cause (1) of
/// R99) and R153 (M1 with M2). Expected to pass when M2 is fixed.
#[test]
#[ignore = "M2 (R152, R99 cause (1)): default mode's CC step is backward Euler"]
fn m2_a_cc_step_is_backward_euler_on_a_rail_without_capacitors() {
    promise(&board_m2(), false, true);
}

/// M3 (the sweep board 189/5's cause; five parts). Board: PSU 5.164233 V /
/// 0.66476 A; C10 1.857 uF rail-ground, C15 78.12 uF rail-n5, C50 0.6926 uF
/// n5-ground (an esr-0 capacitive loop through the CC rail), C21 0.1388 uF
/// n2-rail with R20 384631 ohm n2-ground (a second rail capacitor to a resistive
/// node); CC from t = 0 and until the rail reaches 5.164233 V at 19.76 us; the
/// host stops at 1 us, 10 us, 20 us, 100 us, 1 ms (one advance each). Each of
/// the five parts is needed (any four: default within W0 of exact).
///
/// Truth (RK4 on the capacitance matrix; CCD's m3truth.py gives 2.613537 V at
/// 10 us): rail 0.26135 V at 1 us, 2.61354 V at 10 us, then CV, 5.164233 V
/// from 19.76 us. Default mode on 1753e39: rail 0.003957 / 0.04543 / 0.11326 /
/// 0.49309 / 5.164233 V at 1 us / 10 us / 20 us / 100 us / 1 ms, still CC at
/// 100 us, i.e. 712 W0 (1 us), 946 W0 (10 us, rail), 959 W0 (20 us, rail), 887
/// W0 (100 us, rail); the supply current reads CC where the truth is CV, with a
/// capacitor current of -0.6507 A at 100 us against 0 (6.5e8 W0 worst). Exact
/// mode (companion, right today): 0.001 W0 worst. Differences from the
/// diagnosis's NOTES (rail 0.0295 V at 10 us, 0.098 V at 20 us, 0.522 V at
/// 100 us on 54ec2fc): the reading of this build, whose stops differ.
///
/// Builds: 0.7.0 (c823274), m1, d9d9dfa, 54ec2fc, 1753e39. Held open by R152 and
/// first in R153's repair order. Expected to pass when M3 is fixed.
#[test]
#[ignore = "M3 (R152): an esr-0 capacitive loop through a CC rail plus a second rail capacitor, default mode's rail segment"]
fn m3_an_esr_zero_capacitive_loop_through_a_cc_rail_charges_wrongly() {
    promise(&board_m3(), false, true);
}

/// M4. Board: PSU 5 V / 10 mA, LED rail-ground (vf 2 V, n 2, rated 20 mA), C 1 uF
/// rail-ground: the rail settles onto its CC operating point (tau n Vt C / I =
/// 5.1 us) far slower in default mode than in the circuit; the host stops at
/// 1 us, 10 us, 100 us, 1 ms, 3 ms (one advance each).
///
/// Truth: rail 1.964383 V at 1 ms and 3 ms, the capacitor current ~0 (2e-15 A).
/// Default mode on 1753e39: rail 1.964575 V at 1 ms (0.09 W0) but the capacitor
/// current -3.7496e-5 A: 37,496 W0 (the settling current against W0's 1 nA
/// floor); 69.95 W0 at 3 ms; 0 W0 at 1 us, 10 us, 100 us. Exact mode
/// (companion, right today): 0.014 W0 worst. NOTES: 37,496 W0 at 1 ms, 70 at
/// 3 ms (reproduced to the figure).
///
/// Builds: 0.7.0, m1, d9d9dfa, 54ec2fc, 1753e39. At a 1 us host cadence default
/// mode reads this board right (0.000 W0), so the failure is the host's
/// coarse stop. Held open by R152; fifth in R153's order. Expected to pass when
/// M4 is fixed.
#[test]
#[ignore = "M4 (R152): a CC rail capacitor beside a nonlinear direct draw settles onto the CC operating point too slowly"]
fn m4_a_cc_rail_capacitor_beside_an_led_settles_too_slowly() {
    promise(&board_m4(), false, true);
}

/// M5 (R99 cause (2)). Board: PSU 5 V / 30 mA, R 100 rail-ground, R 10 rail-n2 +
/// C 1 uF n2-ground, R 20 rail-n3 + C 2.2 uF n3-ground (two coupled hub
/// partitions on a CC rail, no rail capacitor); CC from t = 0; the host stops at
/// 1 us, 10 us, 100 us, 1 ms (one advance each).
///
/// Truth: rail 0.20041 / 0.30535 / 0.95985 / 2.83940 V. Default mode on 1753e39:
/// 0.20040 / 0.30532 / 0.95961 / 2.83662 V (2.8 mV low at 1 ms); worst W0 per
/// instant 0.068 / 0.175 / 1.73 / 92.48, the 92.48 W0 on the branch current of R 10
/// (5.0995e-4 A against 4.6670e-4 A). No companion: exact mode reads 4.17 W0 on
/// this board at 1 ms (NOTES 3.3), also wrong, so no mode is right to hold.
/// Differences from NOTES (87.8 W0 default, 3.30 exact): the host stops differ
/// (NOTES's intermediate stops are not given).
///
/// Builds: 0.7.0, m1, 1753e39 (d9d9dfa: default 87.8, exact 70.3). Held open by
/// R152; fourth in R153's order. Expected to pass when M5 is fixed.
#[test]
#[ignore = "M5 (R152, R99 cause (2)): several coupled hub partitions on one CC rail"]
fn m5_two_coupled_hub_partitions_on_a_cc_rail() {
    promise(&board_m5(), false, false);
}

/// M6 (exact mode). Board: PSU 3.082 V / 0.3323 A, C 1.754 uF esr 4.737 ohm
/// rail-ground, R 10 rail-ground (a CC rail capacitor with an ESR; CC until the
/// rail reaches 3.082 V at 57.80 us); the host stops at 1 us, 10 us, 30 us,
/// 100 us, 1 ms (one advance each). Exact mode steps this capacitor by backward
/// Euler (CapStep has no TR-BDF2 form); default mode's closed-form segment is
/// exact here. The mode the mechanism lives in is exact; the companion is default.
///
/// Truth: rail 1.153699 / 1.791535 / 2.616554 V, then 3.082 V (CV) from 57.80 us;
/// the capacitor current 0.15003 mA at 100 us. Exact mode on 1753e39: rail
/// 1.153697 / 1.791610 / 2.616968 V, and at 100 us (CV) the capacitor current
/// 1.3445e-4 A against 1.5003e-4 A: 103.12 W0 (the exit comes early, tau
/// esr C = 8.3 us); 0.586 W0 at 30 us. Default mode (companion, right today):
/// 0.003 W0 worst. NOTES: 103.2 W0.
///
/// Builds: 0.7.0 (103.2), 1753e39; d9d9dfa 1467 W0. Held open by R153 (do not
/// hold 1.0.0 for M6; last in the repair order). Expected to pass when M6 is fixed.
#[test]
#[ignore = "M6 (R153): exact mode steps a CC rail capacitor with an esr by backward Euler"]
fn m6_exact_mode_steps_a_cc_rail_capacitor_with_an_esr_by_backward_euler() {
    promise(&board_m6(), true, true);
}

/// M7 (diagnosis case 16/1). Board: PSU 4.862 V / 22 mA, SW rail-ground (closed),
/// R 26.2 rail-ground, R 11.7 rail-n2, C 0.33 uF n2-ground; the switch opens at
/// 10 ms (the supply stays in CC across the toggle, the rail has no capacitor);
/// the host stops at the toggle and at +0.1 us, +1 us, +10 us, +100 us after it
/// (one advance each, one advance from 0 to 10 ms before).
///
/// Truth: rail 2.1999e-5 V before the toggle, 0.181127 / 0.208572 / 0.397286 /
/// 0.576266 V at +0.1 / +1 / +10 / +100 us. Default mode on 1753e39 holds the
/// rail at its first post-toggle value at +0.1 us: 0.177954 V (11.3 W0 on v; 17.74
/// W0 worst on a current), 3.37 W0 at +10 us, 454.6 W0 at +100 us (the capacitor
/// current 7.9086e-6 A against 5.1246e-6 A); exact mode reads 76.3 W0 at +100 us
/// too, so there is no companion. Differences from NOTES: the +1 us stop reads
/// 0.64 W0 here, not the 163.9 W0 of NOTES (rail 0.208508 V against 0.208572 V; the
/// NOTES run's stops are not given); +0.1 us (17.7), +10 us (3.37 vs 3.38) and
/// +100 us (454.6 vs 456) agree.
///
/// Builds: 0.7.0, m1, proto, 1753e39. Held open by R152; third in R153's order.
/// Expected to pass when M7 is fixed.
#[test]
#[ignore = "M7 (R152): a toggle that moves a capacitor-free CC rail leaves the rail at its first post-toggle value"]
fn m7_a_toggle_that_moves_a_capacitor_free_cc_rail_is_not_replanned() {
    promise(&board_m7(), false, false);
}

/// The other mode of M1, M2, M3, M4 and M6 reads right today; this is the
/// guard a fix of one mode must not break (the same companions the ignored
/// tests assert).
#[test]
fn the_mode_each_mechanism_does_not_live_in_reads_right_today() {
    for (bd, main_exact) in [(board_m1(), false), (board_m2(), false), (board_m3(), false), (board_m4(), false), (board_m6(), true)] {
        let tr = truth(&bd);
        let (x, line) = judge(&bd, &tr, !main_exact);
        assert!(x <= 1.0, "{line}");
    }
}

// ---- M3's blow-up -----------------------------------------------------------------

/// Sweep board 189/5 of the diagnosis (seed 189, template 5, rated program):
/// a 5.164 V / 0.66476 A bench supply, a pulse source and a 555 on the same
/// board, rail capacitors C1-0 (1.857 uF, device 15), C1-5 (78.12 uF, device
/// 21) and C2-1 (device 16), a fuse, two switches. The 30 host stops are the
/// generator's, the first toggle (switch 12 closing) at 0.114174 s, the
/// second (switch 19, the rail switch, opening) at 0.178372 s.
fn sweep_189_5() -> Board {
    let devs = vec![
        dv(21, 0, &[1, 0], vec![5.164232936674946, 0.6647600940451716], vec![]),
        dv(24, 1, &[3, 0], vec![-1.2728463726823824, 5.692476352165269, 0.039529564418409664, 0.003793236687023017, 0.003793236687023017, 0.026848598560853386, 0.08221108978364394], vec![8.0]),
        dv(1, 2, &[2, 0], vec![384631.11484718736], vec![]),
        dv(1, 3, &[4, 0], vec![32523.40290237787], vec![]),
        dv(1, 4, &[5, 0], vec![9434.266774985503], vec![]),
        dv(30, 5, &[4, 3], vec![2.512576539159611, 1.9354746465236876, 0.011455040644029397, 0.0, 0.0, 0.0], vec![]),
        dv(1, 6, &[1, 4], vec![66.78796425357532], vec![]),
        dv(40, 7, &[1, 0, 4, 0, 0, 0, 5, 0], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        dv(1, 8, &[1, 0], vec![9346.047332954771], vec![]),
        dv(1, 9, &[0, 5], vec![22090.536411640285], vec![]),
        dv(3, 10, &[5, 0], vec![6.92575748053364e-7, 0.0, 0.0], vec![]),
        dv(1, 11, &[0, 0], vec![213.7148782039266], vec![]),
        dv(4, 12, &[3, 4], vec![0.0], vec![]),
        dv(1, 13, &[4, 0], vec![11.258134164786538], vec![]),
        dv(1, 14, &[0, 1], vec![104307.01242228784], vec![]),
        dv(3, 15, &[1, 0], vec![1.8569728047274209e-6, 0.0, 0.0], vec![]),
        dv(3, 16, &[2, 1], vec![1.3879537740435692e-7, 0.0, 0.0], vec![]),
        dv(10, 17, &[1, 4], vec![0.4591667480115816, 0.015828253322571734], vec![]),
        dv(3, 18, &[5, 0], vec![1.6409647973236055e-7, 0.0, 0.0], vec![]),
        dv(4, 19, &[1, 0], vec![1.0], vec![]),
        dv(1, 20, &[0, 0], vec![3155.201348369437], vec![]),
        dv(3, 21, &[1, 5], vec![7.811928029692748e-5, 0.0, 0.0], vec![]),
        dv(90, 6, &[], vec![6.0, 0.008749789983121297, 66.78796425357532], vec![]),
        dv(90, 8, &[], vec![8.0, 2.5754272426773337e-5, 9346.047332954771], vec![]),
        dv(90, 9, &[], vec![9.0, 9.640529435713827e-6, 22090.536411640285], vec![]),
        dv(90, 11, &[], vec![11.0, 0.0010331919683708062, 213.7148782039266], vec![]),
        dv(90, 13, &[], vec![13.0, 0.9969610350776253, 11.258134164786538], vec![]),
        dv(90, 14, &[], vec![14.0, 1.818368836626857e-6, 104307.01242228784], vec![]),
        dv(90, 20, &[], vec![20.0, 4.125035932584803e-5, 3155.201348369437], vec![]),
    ];
    let stops = vec![0.006272880019576446, 0.006486499104980916, 0.007766744056270871, 0.008128511669033466, 0.009952994332360228, 0.0106112218022351, 0.022185781724778274, 0.022919760597175275, 0.06198394959107022, 0.06939478318773466, 0.08303066555827561, 0.08347973703977739, 0.11417379053790282, 0.11440352832737956, 0.11469493886242019, 0.12471114995425833, 0.13796907800195432, 0.1428906559809075, 0.15047199531465774, 0.15866659844715744, 0.15890575301304713, 0.1592007370730537, 0.1593164370228923, 0.16009291927419514, 0.16171753297466343, 0.16260802408518474, 0.1671350398307532, 0.16755288843039653, 0.1783723673221548, 0.18229417200791387];
    Board {
        name: "189/5",
        nc: 6,
        devs,
        toggles: vec![(0.11417379053790282, 12, 1.0), (0.1783723673221548, 19, 0.0)],
        stops,
        cadence: 0.0,
        h0: 0.0,
    }
}

/// The supply's limit bounds every capacitor current on the rail when the
/// capacitors started empty and the other sources are small (here at most
/// 0.75 A anywhere, 0.6648 A on the rail's capacitors at every stop but one).
fn over_the_limit(bd: &Board, got: &[(Vec<f64>, Vec<f64>)], devices: &[usize], ilim: f64) -> Vec<String> {
    let mut bad = vec![];
    for (k, g) in got.iter().enumerate() {
        for &d in devices {
            let i = g.1[d];
            if !(i.abs() <= ilim + 1e-9 + 1e-3 * ilim) {
                bad.push(format!("t = {:e}: device {d} publishes {i:e} A", bd.stops[k]));
            }
        }
    }
    bad
}

/// M3's blow-up on the sweep board 189/5 (above): at the stop 0.182294 s (right
/// after the rail switch 19 opens at 0.178372 s) default mode publishes +3.5374e6
/// A (device 10, C5-0), -4.3755e6 A (device 15, C1-0), +8.381e5 A (device 18,
/// C5-0) and +4.3755e6 A (device 21, C1-5) on capacitors of a supply limited to
/// 0.66476 A (the circulating pair sums to ~0); exact mode reads ~0 there, and the
/// next stops read 0.113 A. The bound asserted is physical: a capacitor current
/// on this board cannot exceed the limit by more than W0 (1 nA + 1e-3 |i|);
/// the other sources of the board stay under 0.75 A anywhere and the rail's
/// capacitors under 0.6648 A at every other stop. Builds: 54ec2fc, 1753e39
/// (the diagnosis's 4.4e6 A reproduced: 4.3755e6). Held open by R152; M3 is
/// first in R153's order. Expected to pass when M3 is fixed.
#[test]
#[ignore = "M3 blow-up (R152): default mode publishes +4.4e6 A and -4.4e6 A on two rail capacitors of a 0.665 A supply at one stop of sweep board 189/5"]
fn m3_no_capacitor_current_on_the_sweep_board_exceeds_the_supplys_limit() {
    let bd = sweep_189_5();
    let caps: Vec<usize> = (0..bd.devs.len()).filter(|&k| bd.devs[k].kind == kinds::CAPACITOR).collect();
    let bad = over_the_limit(&bd, &run_engine(&bd, false), &caps, 0.66476);
    assert!(bad.is_empty(), "default mode: {}", bad.join("; "));
    let bad = over_the_limit(&bd, &run_engine(&bd, true), &caps, 0.66476);
    assert!(bad.is_empty(), "exact mode: {}", bad.join("; "));
}

/// The same mechanism on the M3 board with a switch shorting the rail (closed
/// from the start, so the supply is in CC and the capacitors stay empty) that
/// opens at a host stop: with no other source on the board no published
/// current may exceed the supply's limit.
/// The same mechanism with no other source on the board: M3's five parts plus
/// a switch rail-ground, closed from t = 0, opening at a host stop t0 (the
/// host stops at t0 and +0.1 us, +1 us, +10 us, +100 us after it, one advance
/// each). The caps start empty, so the limit (0.66476 A) bounds every current.
/// Default mode on 1753e39 publishes -1.89 A (t0 = 1 us), -16.3 A (10 us), -21.0 A
/// (100 us), -23.1 A (1 ms) and -23.2 A (10 ms), all on C10 (device 1), in 1 to 2
/// readings after the opening; exact mode stays at 0.66476 A. (NOTES's 3.5e6 A
/// spike and the 46.3 V rail reading need the sweep board's stop sequence; this
/// board reaches 23 A, 35 times the limit.) Builds: 54ec2fc, 1753e39. Held open
/// by R152; first in R153's order. Expected to pass when M3 is fixed.
#[test]
#[ignore = "M3 blow-up (R152): default mode publishes currents far past the supply's 0.665 A limit on the M3 board when a switch shorting the rail opens"]
fn m3_no_current_exceeds_the_supplys_limit_when_a_switch_shorting_the_rail_opens() {
    let mut bad = vec![];
    for t0 in [US, 10.0 * US, 100.0 * US, MS, 10.0 * MS] {
        let bd = Board::new("M3+switch", (5.164233, 0.66476), 4, 0.0, 0.0, &[t0, t0 + 0.1 * US, t0 + US, t0 + 10.0 * US, t0 + 100.0 * US])
            .c(1, 0, 1.857e-6, 0.0)
            .c(1, 3, 78.12e-6, 0.0)
            .c(3, 0, 0.6926e-6, 0.0)
            .c(2, 1, 0.1388e-6, 0.0)
            .r(2, 0, 384631.0)
            .sw(1, 0, true)
            .toggle(t0, 6, 0.0);
        let all: Vec<usize> = (0..bd.devs.len()).collect();
        for exact in [false, true] {
            let got = run_engine(&bd, exact);
            let worst = over_the_limit(&bd, &got, &all, 0.66476);
            if let Some(first) = worst.first() {
                bad.push(format!("{} mode, switch opening at {t0:e}: {first} ({} readings)", if exact { "exact" } else { "default" }, worst.len()));
            }
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("; "));
}

/// Exact mode keeps every capacitor current of the sweep board and of the
/// shorted-rail board within the supply's limit today (the companion of the two
/// blow-up tests above, which stop at default mode's first violation).
#[test]
fn exact_mode_keeps_every_capacitor_current_within_the_limit_on_the_blow_up_boards_today() {
    let bd = sweep_189_5();
    let caps: Vec<usize> = (0..bd.devs.len()).filter(|&k| bd.devs[k].kind == kinds::CAPACITOR).collect();
    let bad = over_the_limit(&bd, &run_engine(&bd, true), &caps, 0.66476);
    assert!(bad.is_empty(), "189/5 exact mode: {}", bad.join("; "));
}

// ---- control tests: the bench-supply fixtures that read right ---------------------

const S1: [f64; 12] = [0.005, 0.01, 0.0101, 0.0105, 0.011, 0.012, 0.015, 0.02, 0.03, 0.05, 0.08, 0.1];
const S2: [f64; 7] = [1e-5, 1e-4, 1e-3, 0.004, 0.005, 0.006, 0.01];
const S3: [f64; 8] = [0.001, 0.01, 0.03, 0.05, 0.06, 0.07, 0.1, 0.2];

/// The fixtures' wiring as the repository's bench-supply fixtures give it
/// (catalog limit 0.1 A, 5 V): 43 a load step (R 100 plus a switch to R 25 at
/// 10 ms, the overload enters CC), 44 the same with 1 mF (esr 0) on the rail,
/// 45 with 1 mF esr 0.05, 47 the inrush into 100 uF (esr 0), 51 the inrush
/// into 1 mF esr 0.05 beside R 100. Each against the same truth, default mode.
fn fixture(n: u32) -> Board {
    let b = |h0: f64, stops: &[f64], name: &'static str| Board::new(name, (5.0, 0.1), 3, h0, 0.0, stops);
    match n {
        43 => b(1e-3, &S1, "fixture 43").r(1, 0, 100.0).sw(1, 2, false).r(2, 0, 25.0).toggle(0.01, 2, 1.0),
        44 => b(1e-3, &S1, "fixture 44").r(1, 0, 100.0).sw(1, 2, false).r(2, 0, 25.0).c(1, 0, 1e-3, 0.0).toggle(0.01, 2, 1.0),
        45 => b(2e-6, &S1, "fixture 45").r(1, 0, 100.0).sw(1, 2, false).r(2, 0, 25.0).c(1, 0, 1e-3, 0.05).toggle(0.01, 2, 1.0),
        47 => b(1e-5, &S2, "fixture 47").c(1, 0, 1e-4, 0.0),
        51 => b(2e-6, &S3, "fixture 51").c(1, 0, 1e-3, 0.05).r(1, 0, 100.0),
        _ => unreachable!(),
    }
}

fn control(n: u32) {
    let bd = fixture(n);
    let tr = truth(&bd);
    let (x, line) = judge(&bd, &tr, false);
    assert!(x <= 1.0, "{line}");
    // the fixture really enters CC (otherwise it controls nothing)
    assert!(tr.0.at.iter().any(|r| r.cc), "fixture {n} never enters CC in the truth");
}

#[test]
fn fixture_43_a_load_step_into_current_limit_reads_right_in_default_mode() {
    control(43);
}
#[test]
fn fixture_44_a_load_step_with_a_rail_capacitor_reads_right_in_default_mode() {
    control(44);
}
#[test]
fn fixture_45_a_load_step_with_a_rail_capacitor_of_esr_reads_right_in_default_mode() {
    control(45);
}
#[test]
fn fixture_47_an_inrush_into_a_capacitor_reads_right_in_default_mode() {
    control(47);
}
#[test]
fn fixture_51_an_inrush_into_a_capacitor_of_esr_beside_a_load_reads_right_in_default_mode() {
    control(51);
}
