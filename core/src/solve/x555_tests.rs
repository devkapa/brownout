//! X555: exact mode (`noRailShortcut`) on the 555 astables of fixtures 77
//! (`test/fixtures/core/77-battery-555-astable-led`) and 78
//! (`78-battery-555-clocks-161`), against an independent truth (T555's
//! RK4, ported below). The programs are the fixtures' own compiled records
//! (`compileCircuit`, dumped by ship-logs/x555/dump.mts), so the boards are
//! exactly what the facade loads.
//!
//! In exact mode the rail is a free node, so the timing capacitor sits on a
//! stepped partition with the cell. T555 measured its period 0.10 % long at
//! 10 us host stops and 1.4-1.5 % long from 1 ms up (truth 3776.6 us), with
//! the trapezoidal option inside 0.05 % at every stop: the stepped
//! integration, not the crossing or the threshold read.

use crate::engine::Engine;
use crate::program::Program;

/// Fixture 77, compiled (5 V cell, 2 ohm; cdec 22 uF; RA 1000, RB 2200,
/// C 1 uF; OUT through 330 ohm to an LED). Nodes: 1 the rail, 4 the timing
/// capacitor; the 555's OUT flips trace on digital slot 0.
const P77: &[f64] = &[1112490801.0, 3.0, 7.0, 12.0, 3.0, 1.0, 0.0, 5.0, 25.0, 7.0, 0.0, 1.0, 0.0, 20.0, 1.0, 2.0, 1.0, 0.0, 350104658.0, 3585718484.0, 0.0, 0.0, 1332659326.0, 1192844608.0, 9.0, 1.0, 2.0, 1.0, 0.5, 2.0, 0.05, 1.0, 1.0, 0.0, 5.0, 2.0, 0.0, 0.0, 1.0, 5.0, 3.0, 2.0, 2.0, 1.0, 0.0, 1731667923.0, 498246009.0, 0.0, 0.0, 3980015964.0, 2145395718.0, 3.0, 0.000022, 0.0, 0.0, 0.0, 40.0, 3.0, 8.0, 1.0, 0.0, 2704479849.0, 2384089727.0, 0.0, 0.0, 2806278396.0, 2074249638.0, 4.0, 0.0, 2705465587.0, 1408269849.0, 5.0, 0.0, 2671763254.0, 3054086136.0, 1.0, 0.0, 2840672277.0, 2641985739.0, 2.0, 0.0, 525213760.0, 4104203690.0, 4.0, 0.0, 692842855.0, 3745676589.0, 3.0, 0.0, 659140522.0, 1736052716.0, 13.0, 4.5, 16.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 4.0, 2.0, 2.0, 0.0, 1683608697.0, 2707765711.0, 0.0, 0.0, 1649612174.0, 3902657744.0, 3.0, 1e-8, 0.0, 0.0, 0.0, 1.0, 5.0, 2.0, 1.0, 0.0, 4195138313.0, 752397343.0, 3.0, 0.0, 2013606558.0, 3582671328.0, 1.0, 1000.0, 0.0, 1.0, 6.0, 2.0, 3.0, 0.0, 4254480682.0, 1727833452.0, 4.0, 0.0, 2141045141.0, 2633766475.0, 1.0, 2200.0, 0.0, 3.0, 7.0, 2.0, 4.0, 0.0, 274301427.0, 1215391513.0, 0.0, 0.0, 375114236.0, 1881371302.0, 3.0, 0.000001, 0.0, 0.0, 0.0, 1.0, 8.0, 2.0, 5.0, 0.0, 899472740.0, 4096629646.0, 6.0, 0.0, 798659931.0, 1220839233.0, 1.0, 330.0, 0.0, 30.0, 9.0, 2.0, 6.0, 0.0, 2697112015.0, 1240642709.0, 0.0, 0.0, 551254069.0, 835723755.0, 6.0, 1.8, 2.0, 0.02, 0.0, 0.0, 0.0, 0.0, 90.0, 5.0, 0.0, 3.0, 4.0, 0.25, 1000.0, 0.0, 90.0, 6.0, 0.0, 3.0, 5.0, 0.25, 2200.0, 0.0, 90.0, 8.0, 0.0, 3.0, 7.0, 0.25, 330.0, 0.0, 1112490801.0];

/// Fixture 78, compiled (5 V cell, 0.9 ohm; cdec 47 uF; RA 1500, RB 1000,
/// C 1 uF; OUT clocks a 74LS161). Same node and slot roles as 77.
const P78: &[f64] = &[1112490801.0, 3.0, 11.0, 10.0, 3.0, 1.0, 0.0, 5.0, 25.0, 7.0, 0.0, 6.0, 0.0, 20.0, 1.0, 2.0, 1.0, 0.0, 350104658.0, 3585718484.0, 0.0, 0.0, 1332659326.0, 1192844608.0, 9.0, 1.0, 0.9, 1.0, 0.5, 2.0, 0.05, 1.0, 1.0, 0.0, 5.0, 2.0, 0.0, 0.0, 1.0, 5.0, 3.0, 2.0, 2.0, 1.0, 0.0, 1731667923.0, 498246009.0, 0.0, 0.0, 3980015964.0, 2145395718.0, 3.0, 0.000047, 0.0, 0.0, 0.0, 40.0, 3.0, 8.0, 1.0, 0.0, 2704479849.0, 2384089727.0, 0.0, 0.0, 2806278396.0, 2074249638.0, 4.0, 0.0, 2705465587.0, 1408269849.0, 5.0, 0.0, 2671763254.0, 3054086136.0, 1.0, 0.0, 2840672277.0, 2641985739.0, 2.0, 0.0, 525213760.0, 4104203690.0, 4.0, 0.0, 692842855.0, 3745676589.0, 3.0, 0.0, 659140522.0, 1736052716.0, 13.0, 4.5, 16.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 4.0, 2.0, 2.0, 0.0, 1683608697.0, 2707765711.0, 0.0, 0.0, 1649612174.0, 3902657744.0, 3.0, 1e-8, 0.0, 0.0, 0.0, 1.0, 5.0, 2.0, 1.0, 0.0, 4195138313.0, 752397343.0, 3.0, 0.0, 2013606558.0, 3582671328.0, 1.0, 1500.0, 0.0, 1.0, 6.0, 2.0, 3.0, 0.0, 4254480682.0, 1727833452.0, 4.0, 0.0, 2141045141.0, 2633766475.0, 1.0, 1000.0, 0.0, 3.0, 7.0, 2.0, 4.0, 0.0, 274301427.0, 1215391513.0, 0.0, 0.0, 375114236.0, 1881371302.0, 3.0, 0.000001, 0.0, 0.0, 0.0, 70.0, 8.0, 16.0, 1.0, 0.0, 2508224883.0, 4202486425.0, 0.0, 0.0, 2942745494.0, 3127335128.0, 1.0, 0.0, 2139561909.0, 2524824939.0, 5.0, 0.0, 1343188439.0, 3186989341.0, 1.0, 0.0, 844394400.0, 3053069450.0, 1.0, 0.0, 657307008.0, 3506289258.0, 1.0, 0.0, 2938371644.0, 3118540902.0, 0.0, 0.0, 1332766294.0, 1752317976.0, 0.0, 0.0, 3514298049.0, 819633751.0, 0.0, 0.0, 3614816668.0, 1166169798.0, 0.0, 0.0, 1501381127.0, 4280466893.0, 10.0, 1.0, 2765162035.0, 199789273.0, 9.0, 1.0, 2865974844.0, 3088250982.0, 8.0, 1.0, 618024161.0, 1419530871.0, 7.0, 1.0, 718836970.0, 3548581804.0, 6.0, 1.0, 2698068435.0, 108432249.0, 21.0, 4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 2e-8, 1.3e-8, 1.8000000000000002e-8, 2e-8, 2e-8, 2e-8, 1.8000000000000002e-8, 9.000000000000001e-9, 9.000000000000001e-9, 15.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 5.0, 0.0, 3.0, 4.0, 0.25, 1500.0, 0.0, 90.0, 6.0, 0.0, 3.0, 5.0, 0.25, 1000.0, 0.0, 1112490801.0];

/// The truth's board: the timing network and the rail's relaxation.
struct Board {
    ra: f64,
    rb: f64,
    c: f64,
    /// The cell's resistance and the rail capacitor: the rail relaxes with
    /// tau = r_bat c_dec toward the phase's asymptote.
    r_bat: f64,
    c_dec: f64,
    /// The rail's settled value with OUT high and with OUT low, the core's
    /// own (T555, read off its settled rail): the truth does not re-model
    /// the rail's loads (the LED, the 161), only their settled effect.
    v_high: f64,
    v_low: f64,
}

const B77: Board = Board { ra: 1000.0, rb: 2200.0, c: 1e-6, r_bat: 2.0, c_dec: 22e-6, v_high: 4.97172, v_low: 4.98035 };
const B78: Board = Board { ra: 1500.0, rb: 1000.0, c: 1e-6, r_bat: 0.9, c_dec: 47e-6, v_high: 4.99498, v_low: 4.99262 };

/// DIS's on resistance (ne555.rs `DIS_R_OHM`).
const RD: f64 = 10.0;
/// The 555's power-on supply (vccMin 4.5 x 0.9).
const V_ON: f64 = 4.05;

/// The truth (T555's truth.py, cold start: cdec empty, the 555 unpowered
/// until the rail reaches 4.05 V, Ct charging from the ramp all along).
/// OUT high: dVc/dt = (Vr - Vc) / ((RA + RB) C). OUT low: DIS's 10 ohm in
/// parallel with RA, target Vr RD / (RA + RD), tau (RB + RA || RD) C. Flips
/// at Vc = 2/3 Vr (high) and 1/3 Vr (low), strict, Vr the rail then. RK4 at
/// 1 us with each event bisected (T555: halving moves no edge by 1e-13 s).
/// Returns the edges `(t, level)`, the power-on rise first.
fn truth(b: &Board, t_end: f64) -> Vec<(f64, f64)> {
    let tau_r = b.r_bat * b.c_dec;
    let tau_h = (b.ra + b.rb) * b.c;
    let rad = b.ra * RD / (b.ra + RD);
    let tau_l = (b.rb + rad) * b.c;
    let k_l = RD / (b.ra + RD);
    // (high, powered, rail start time, rail at it, rail asymptote)
    let mut s = (true, false, 0.0f64, 0.0f64, b.v_high);
    let vr = |s: &(bool, bool, f64, f64, f64), t: f64| s.4 + (s.3 - s.4) * (-(t - s.2) / tau_r).exp();
    let f = |s: &(bool, bool, f64, f64, f64), t: f64, v: f64| {
        let r = vr(s, t);
        if !s.1 || s.0 { (r - v) / tau_h } else { (r * k_l - v) / tau_l }
    };
    let rk4 = |s: &(bool, bool, f64, f64, f64), t: f64, v: f64, h: f64| {
        let k1 = f(s, t, v);
        let k2 = f(s, t + h / 2.0, v + h / 2.0 * k1);
        let k3 = f(s, t + h / 2.0, v + h / 2.0 * k2);
        let k4 = f(s, t + h, v + h * k3);
        v + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    };
    let g = |s: &(bool, bool, f64, f64, f64), t: f64, v: f64| {
        let r = vr(s, t);
        if !s.1 {
            r - V_ON
        } else if s.0 {
            v - 2.0 / 3.0 * r
        } else {
            r / 3.0 - v
        }
    };
    let h = 1e-6;
    let (mut t, mut v) = (0.0f64, 0.0f64);
    let mut edges = Vec::new();
    while t < t_end {
        let vn = rk4(&s, t, v, h);
        if g(&s, t + h, vn) > 0.0 && g(&s, t, v) <= 0.0 {
            let (mut lo, mut hi) = (0.0, h);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if g(&s, t + mid, rk4(&s, t, v, mid)) > 0.0 { hi = mid } else { lo = mid }
            }
            let te = t + hi;
            let ve = rk4(&s, t, v, hi);
            if !s.1 {
                s.1 = true;
                edges.push((te, 1.0));
            } else {
                let r = vr(&s, te);
                s.0 = !s.0;
                s = (s.0, true, te, r, if s.0 { b.v_high } else { b.v_low });
                edges.push((te, if s.0 { 1.0 } else { 0.0 }));
            }
            t = te;
            v = ve;
            continue;
        }
        t += h;
        v = vn;
    }
    edges
}

/// W0's voltage band at `v` (R121): 1e-4 + 1e-3 |v|.
fn w0_v(v: f64) -> f64 {
    1e-4 + 1e-3 * v.abs()
}

/// The time the timing capacitor takes to cross W0 of the threshold an
/// interval ends on: the threshold over its slope there. An interval that
/// ends on a fall ended an OUT-high phase (charging to 2/3 Vr); one that
/// ends on a rise ended an OUT-low phase (discharging to 1/3 Vr).
fn tau_w0(b: &Board, ends_on_fall: bool) -> f64 {
    if ends_on_fall {
        let vr = b.v_high;
        let slope = (vr / 3.0) / ((b.ra + b.rb) * b.c);
        w0_v(2.0 / 3.0 * vr) / slope
    } else {
        let vr = b.v_low;
        let rad = b.ra * RD / (b.ra + RD);
        let slope = (vr / 3.0 - vr * RD / (b.ra + RD)) / ((b.rb + rad) * b.c);
        w0_v(vr / 3.0) / slope
    }
}

/// The 555's OUT edges in exact mode with the host stopping every `stop`.
fn exact_edges(prog: &[f64], stop: f64, t_end: f64, trap: bool) -> Vec<(f64, f64)> {
    edges_with(prog, stop, t_end, |o| {
        o.no_rail_shortcut = true;
        o.no_solution_cache = true;
        if trap {
            o.integration = 1;
        }
    })
}

/// The 555's OUT edges under the options `set` leaves, the host stopping
/// every `stop`.
fn edges_with(prog: &[f64], stop: f64, t_end: f64, set: impl Fn(&mut crate::program::Options)) -> Vec<(f64, f64)> {
    let mut p = Program::parse(prog).unwrap();
    set(&mut p.options);
    let mut e = Engine::load(&p).unwrap();
    e.trace_enable(true);
    let mut out = Vec::new();
    let n = (t_end / stop).ceil() as usize;
    for k in 1..=n {
        e.advance((k as f64 * stop).min(t_end)).unwrap();
        out.extend(e.trace().chunks(3).filter(|ch| ch[1] == 0.0).map(|ch| (ch[0], ch[2])));
        e.trace_clear();
    }
    out
}

/// Each edge's interval from the one before it against the truth's, in
/// shares of the tau_w0 of the threshold it ends on, and the mean period
/// over whole cycles after the power-on interval against the truth's, as a
/// share of the tau_w0 its two edges carry together (a period is one
/// OUT-high and one OUT-low interval, each closing on its own crossing).
/// Returns (worst interval share, period share, period us, truth period
/// us); the period's share of the rise's tau_w0 alone is printed too.
fn judge(name: &str, b: &Board, got: &[(f64, f64)], truth: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    let n = got.len().min(truth.len());
    assert!(n >= 8, "{name}: {} edges against the truth's {}", got.len(), truth.len());
    for i in 0..n {
        assert_eq!(got[i].1, truth[i].1, "{name}: edge {i} level {} against the truth's {}", got[i].1, truth[i].1);
    }
    let mut worst = 0.0f64;
    for i in 2..n {
        let d = (got[i].0 - got[i - 1].0) - (truth[i].0 - truth[i - 1].0);
        worst = worst.max(d.abs() / tau_w0(b, got[i].1 == 0.0));
    }
    // Rises after the power-on one (edge 0): edges 2, 4, ...
    let rises = |e: &[(f64, f64)]| e[..n].iter().skip(1).filter(|x| x.1 == 1.0).map(|x| x.0).collect::<Vec<_>>();
    let (gr, tr) = (rises(got), rises(truth));
    let per = |r: &[f64]| (r[r.len() - 1] - r[0]) / (r.len() - 1) as f64;
    let (pg, pt) = (per(&gr), per(&tr));
    let tw = tau_w0(b, true) + tau_w0(b, false);
    eprintln!("X555 {name}: the period {:.3} tau_w0 of the rise alone", (pg - pt).abs() / tau_w0(b, false));
    // The mean error of the OUT-high intervals (ending on a fall) and of
    // the OUT-low ones, us: where the period's error sits.
    let mean = |fall: bool| {
        let d: Vec<f64> = (2..n).filter(|&i| (got[i].1 == 0.0) == fall).map(|i| (got[i].0 - got[i - 1].0) - (truth[i].0 - truth[i - 1].0)).collect();
        d.iter().sum::<f64>() / d.len() as f64 * 1e6
    };
    eprintln!("X555 {name}: mean interval error high {:+.3} us, low {:+.3} us", mean(true), mean(false));
    (worst, (pg - pt).abs() / tw, pg * 1e6, pt * 1e6)
}

fn run(name: &str, prog: &[f64], b: &Board, stop: f64, t_end: f64, trap: bool) -> (f64, f64, f64, f64) {
    let t = truth(b, t_end);
    let got = exact_edges(prog, stop, t_end, trap);
    let r = judge(name, b, &got, &t);
    eprintln!("X555 {name}{} stop {stop:.3e}: period {:.2} us (truth {:.2}, {:+.3} %), period {:.3} tau_w0, worst interval {:.3} tau_w0", if trap { " trap" } else { "" }, r.2, r.3, (r.2 / r.3 - 1.0) * 100.0, r.1, r.0);
    r
}

/// Exact mode's fixture-77 555 at 1 ms and 1/60 s host stops: each edge's
/// interval and the mean period within W0 of the truth (as a time: the
/// timing capacitor's W0 at the threshold over its slope there, 2.4 us at
/// the trigger and 6.6 us at the threshold, 9.0 us for a period). Before
/// X555 the period ran 1.4 % long (52 us a cycle, 5.8 of the period's
/// 9.0 us), an interval up to 9.7 tau_w0 long.
#[test]
fn exact_modes_555_on_a_cell_keeps_its_period_at_coarse_host_stops() {
    for stop in [1e-3, 1.0 / 60.0] {
        let (worst, period, _, _) = run("77", P77, &B77, stop, 0.05, false);
        assert!(worst <= 1.0, "fixture 77 at {stop:e} s stops: an interval {worst:.3} tau_w0 off the truth's");
        assert!(period <= 1.0, "fixture 77 at {stop:e} s stops: the period {period:.3} tau_w0 off the truth's");
    }
}

/// The table: both fixtures at 10 us, 100 us, 1 ms and 1/60 s stops, exact
/// mode as it is and with the trapezoidal option (T555's reference path).
/// Asserts the same bounds at every stop on both.
#[test]
#[ignore = "the X555 table: run by hand, --ignored --nocapture"]
fn x555_table() {
    let mut bad = Vec::new();
    for (name, prog, b) in [("77", P77, &B77), ("78", P78, &B78)] {
        for trap in [false, true] {
            for stop in [1e-5, 1e-4, 1e-3, 1.0 / 60.0] {
                let (worst, period, _, _) = run(name, prog, b, stop, 0.1, trap);
                if worst > 1.0 || period > 1.0 {
                    bad.push(format!("{name}{} {stop:e}", if trap { " trap" } else { "" }));
                }
            }
        }
    }
    assert!(bad.is_empty(), "outside W0: {bad:?}");
}

/// The step trace behind the table (X555 point 2): every accepted step of
/// fixture 77 in exact mode at 1 ms host stops over 8 ms to 16 ms, by
/// partition and method, and the steps between two edges.
#[test]
#[ignore = "the X555 step trace: run by hand, --ignored --nocapture"]
fn x555_step_trace() {
    use super::stepped::STEP_LOG;
    for (trap, stop) in [(false, 1e-3), (true, 1e-3), (false, 1.0 / 60.0)] {
        STEP_LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
        let edges = exact_edges(P77, stop, 0.016, trap);
        let log = STEP_LOG.with(|l| l.borrow_mut().take().unwrap());
        let win: Vec<_> = log.iter().filter(|s| s.t_end > 0.008).collect();
        let mut parts: Vec<usize> = win.iter().map(|s| s.p_idx).collect();
        parts.sort();
        parts.dedup();
        for p in parts {
            let of: Vec<_> = win.iter().filter(|s| s.p_idx == p).collect();
            let count = |m: u8| of.iter().filter(|s| s.method == m).count();
            let hmax = of.iter().map(|s| s.h).fold(0.0, f64::max);
            let hmean = of.iter().map(|s| s.h).sum::<f64>() / of.len() as f64;
            eprintln!("X555TRACE trap={trap} stop {stop:.3e} partition {p}: {} steps (BE {}, trap {}, TR-BDF2 {}), be_next {}, cell partition {}, h mean {hmean:.3e} max {hmax:.3e}, nodes {}", of.len(), count(0), count(1), count(2), of.iter().filter(|s| s.be_next).count(), of.iter().filter(|s| s.cell_partition).count(), of[0].x.len());
        }
        let e: Vec<f64> = edges.iter().map(|e| e.0).filter(|&t| t > 0.008).take(2).collect();
        eprintln!("X555TRACE trap={trap} steps between edges {:.6} and {:.6}:", e[0], e[1]);
        for s in log.iter().filter(|s| s.t_end > e[0] - 2e-5 && s.t_end <= e[1] + 2e-5) {
            eprintln!("X555TRACE   p{} t_end {:.7} h {:.3e} ratio {:.3} method {} be_next {} cellp {}", s.p_idx, s.t_end, s.h, s.ratio, s.method, s.be_next, s.cell_partition);
        }
    }
}

/// Default mode's share (X555 point 3): fixture 77 and 78 in default mode
/// with FORCE_STEPPED, so the rail is default's algebraic hub and the
/// timing capacitor sits on a stepped partition, against the truth's warm
/// start (default's rail starts at its working value and its 555 powers on
/// with OUT high, so its first edge is a fall), with the methods its steps
/// took.
#[test]
#[ignore = "the X555 default-mode check: run by hand, --ignored --nocapture"]
fn x555_default_mode_stepped_timing_partition() {
    use super::stepped::STEP_LOG;
    for (name, prog, b) in [("77", P77, &B77), ("78", P78, &B78)] {
        let warm = truth_warm(b, 0.1);
        for stop in [1e-5, 1e-4, 1e-3, 1.0 / 60.0] {
            STEP_LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
            let got = edges_with(prog, stop, 0.1, |o| o.force_stepped = true);
            let log = STEP_LOG.with(|l| l.borrow_mut().take().unwrap());
            // Default has no power-on rise: compare from its first fall,
            // the truth's warm edges likewise.
            let n = got.len().min(warm.len());
            let mut worst = 0.0f64;
            for i in 1..n {
                assert_eq!(got[i].1, warm[i].1, "{name} default: edge {i}");
                let d = (got[i].0 - got[i - 1].0) - (warm[i].0 - warm[i - 1].0);
                worst = worst.max(d.abs() / tau_w0(b, got[i].1 == 0.0));
            }
            let rises = |e: &[(f64, f64)]| e[..n].iter().filter(|x| x.1 == 1.0).map(|x| x.0).collect::<Vec<_>>();
            let (gr, tr) = (rises(&got), rises(&warm));
            let per = |r: &[f64]| (r[r.len() - 1] - r[0]) / (r.len() - 1) as f64;
            let (pg, pt) = (per(&gr), per(&tr));
            let count = |m: u8| log.iter().filter(|s| s.method == m).count();
            eprintln!("X555 default+stepped {name} stop {stop:.3e}: period {:.2} us (truth warm {:.2}, {:+.3} %), worst interval {worst:.3} tau_w0; steps BE {} (be_next {}), trap {}, TR-BDF2 {}", pg * 1e6, pt * 1e6, (pg / pt - 1.0) * 100.0, count(0), log.iter().filter(|s| s.be_next).count(), count(1), count(2));
        }
    }
}

/// The truth with the rail at its OUT-high value from the start and the
/// 555 powered (default mode's start).
fn truth_warm(b: &Board, t_end: f64) -> Vec<(f64, f64)> {
    let tau_h = (b.ra + b.rb) * b.c;
    let rad = b.ra * RD / (b.ra + RD);
    let tau_l = (b.rb + rad) * b.c;
    let k_l = RD / (b.ra + RD);
    let tau_r = b.r_bat * b.c_dec;
    let (mut high, mut ts, mut r0, mut asym) = (true, 0.0f64, b.v_high, b.v_high);
    let (mut t, mut v) = (0.0f64, 0.0f64);
    let h = 1e-6;
    let mut edges = Vec::new();
    while t < t_end {
        let vr = |t: f64| asym + (r0 - asym) * (-(t - ts) / tau_r).exp();
        let f = |t: f64, v: f64| if high { (vr(t) - v) / tau_h } else { (vr(t) * k_l - v) / tau_l };
        let rk4 = |t: f64, v: f64, h: f64| {
            let k1 = f(t, v);
            let k2 = f(t + h / 2.0, v + h / 2.0 * k1);
            let k3 = f(t + h / 2.0, v + h / 2.0 * k2);
            let k4 = f(t + h, v + h * k3);
            v + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        };
        let g = |t: f64, v: f64| if high { v - 2.0 / 3.0 * vr(t) } else { vr(t) / 3.0 - v };
        let vn = rk4(t, v, h);
        if g(t + h, vn) > 0.0 && g(t, v) <= 0.0 {
            let (mut lo, mut hi) = (0.0, h);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if g(t + mid, rk4(t, v, mid)) > 0.0 { hi = mid } else { lo = mid }
            }
            let te = t + hi;
            v = rk4(t, v, hi);
            r0 = vr(te);
            ts = te;
            high = !high;
            asym = if high { b.v_high } else { b.v_low };
            edges.push((te, if high { 1.0 } else { 0.0 }));
            t = te;
            continue;
        }
        t += h;
        v = vn;
    }
    edges
}
