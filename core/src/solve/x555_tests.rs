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

/// The truth's board: the timing network, the rail capacitor, and the
/// 555's OUT load.
struct Board {
    ra: f64,
    rb: f64,
    c: f64,
    c_dec: f64,
    /// OUT's series resistor to the LED (fixture 77), or `None`: OUT
    /// drives only the 161's CLK (78), which stamps no load on it.
    led_r: Option<f64>,
    /// The engine's settled rail with OUT high and with OUT low, as T555
    /// read them: printed beside the hand-derived values, no longer used.
    seen_high: f64,
    seen_low: f64,
}

const B77: Board = Board { ra: 1000.0, rb: 2200.0, c: 1e-6, c_dec: 22e-6, led_r: Some(330.0), seen_high: 4.97172, seen_low: 4.98035 };
const B78: Board = Board { ra: 1500.0, rb: 1000.0, c: 1e-6, c_dec: 47e-6, led_r: None, seen_high: 4.99498, seen_low: 4.99262 };

/// DIS's on resistance (ne555.rs `DIS_R_OHM`).
const RD: f64 = 10.0;
/// OUT's Thevenin resistance (ne555.rs `OUTPUT_R_OHM`).
const R_OUT: f64 = 20.0;
/// The 555's power-on supply (vccMin 4.5 x 0.9).
const V_ON: f64 = 4.05;

/// The rail's sources and loads by hand (X555C finding 4), from the
/// fixture's own records: the cell's E and r from its battery model at its
/// starting SoC (1 Ah: the 0.1 s run moves it by under 4e-7, E by under
/// 2 uV, so it stays there), and the LED's junction law at the program's
/// ambient (no thermal record: the engine holds it there too).
struct Hand {
    e: f64,
    r: f64,
    law: Option<crate::devices::led::JunctionLaw>,
}

fn hand(prog: &[f64]) -> Hand {
    let p = Program::parse(prog).unwrap();
    let cell = p.devices.iter().find(|d| d.kind == crate::kinds::BATTERY_PACK).unwrap();
    let model = crate::devices::battery::BatteryModel::from_record(&cell.params, &cell.blob);
    let soc = crate::devices::battery::BatteryState::initial(&model).soc;
    let law = p.devices.iter().find_map(|d| crate::devices::led::led_law_at(d, p.options.ambient_c));
    Hand { e: model.emf(soc), r: model.resistance(soc), law }
}

/// The current the rail delivers at rail `vr`, timing capacitor `vc`, OUT
/// `high`, the 555 `powered`, and the timing capacitor's dVc/dt:
/// - the timing chain: OUT high (or unpowered, DIS released) RA + RB into
///   Ct; OUT low RA into DIS's node, held by DIS's 10 ohm to ground and RB
///   from Ct, `vd = (vr/RA + vc/RB) / (1/RA + 1/RB + 1/RD)`;
/// - OUT, powered: its Thevenin split between the rails (`out_fraction`,
///   f / 20 S to the rail and (1 - f) / 20 S to ground: idle cross-current
///   even unloaded), into 330 ohm and the LED on 77 (the junction voltage
///   bisected) or nothing on 78.
fn draw(b: &Board, h: &Hand, vr: f64, vc: f64, high: bool, powered: bool) -> (f64, f64) {
    let (chain, dvc) = if !powered || high {
        ((vr - vc) / (b.ra + b.rb), (vr - vc) / ((b.ra + b.rb) * b.c))
    } else {
        let vd = (vr / b.ra + vc / b.rb) / (1.0 / b.ra + 1.0 / b.rb + 1.0 / RD);
        ((vr - vd) / b.ra, (vd - vc) / (b.rb * b.c))
    };
    if !powered {
        return (chain, dvc);
    }
    let f = crate::digital::ne555::out_fraction(high, vr);
    let (g_hi, g_lo) = (f / R_OUT, (1.0 - f) / R_OUT);
    // OUT's node: g_hi (vr - vo) = g_lo vo + i_load(vo).
    let vo = match (b.led_r, &h.law) {
        (Some(rs), Some(law)) => {
            // The junction voltage vj: vo = vj + rs I(vj), and KCL at OUT
            // falls as vj rises.
            let kcl = |vj: f64| {
                let i = law.current(vj);
                let vo = vj + rs * i;
                g_hi * (vr - vo) - g_lo * vo - i
            };
            let (mut lo, mut hi) = (0.0f64, vr.max(0.0));
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if kcl(mid) > 0.0 { lo = mid } else { hi = mid }
            }
            let vj = 0.5 * (lo + hi);
            vj + rs * law.current(vj)
        }
        _ => g_hi * vr / (g_hi + g_lo),
    };
    (chain + g_hi * (vr - vo), dvc)
}

/// The rail settled (its capacitor at rest) at a timing capacitor held at
/// `frac` of it: `vr = E - r draw(vr)`, by fixed point (the loop gain is
/// r over the loads' resistance, under 0.1).
fn settled(b: &Board, h: &Hand, high: bool, frac: f64) -> f64 {
    let mut vr = h.e;
    for _ in 0..100 {
        vr = h.e - h.r * draw(b, h, vr, frac * vr, high, true).0;
    }
    vr
}

/// The truth (T555's truth.py, rebuilt by hand in X555C): the rail and the
/// timing capacitor as one ODE, `c_dec dVr/dt = (E - Vr) / r - draw`,
/// `dVc/dt` from [`draw`], its sources and loads from the fixture's records
/// ([`hand`]) rather than the engine's settled rail. Cold start: cdec and
/// Ct empty, the 555 unpowered until the rail reaches 4.05 V, then OUT
/// high; flips at Vc = 2/3 Vr (high) and 1/3 Vr (low), strict, Vr the rail
/// then. `warm`: default mode's start, the rail at its settled OUT-high
/// value with Ct empty and the 555 powered. RK4 at 1 us with each event
/// bisected. Returns the edges `(t, level)`, a cold run's power-on rise
/// first.
fn truth_from(b: &Board, h: &Hand, t_end: f64, warm: bool) -> Vec<(f64, f64)> {
    let (mut high, mut powered) = (true, warm);
    let mut y = [if warm { settled(b, h, true, 0.0) } else { 0.0 }, 0.0];
    let f = |y: [f64; 2], high: bool, powered: bool| {
        let (i, dvc) = draw(b, h, y[0], y[1], high, powered);
        [((h.e - y[0]) / h.r - i) / b.c_dec, dvc]
    };
    let rk4 = |y: [f64; 2], high: bool, powered: bool, hs: f64| {
        let k1 = f(y, high, powered);
        let k2 = f([y[0] + hs / 2.0 * k1[0], y[1] + hs / 2.0 * k1[1]], high, powered);
        let k3 = f([y[0] + hs / 2.0 * k2[0], y[1] + hs / 2.0 * k2[1]], high, powered);
        let k4 = f([y[0] + hs * k3[0], y[1] + hs * k3[1]], high, powered);
        [y[0] + hs / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]), y[1] + hs / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])]
    };
    let g = |y: [f64; 2], high: bool, powered: bool| {
        if !powered {
            y[0] - V_ON
        } else if high {
            y[1] - 2.0 / 3.0 * y[0]
        } else {
            y[0] / 3.0 - y[1]
        }
    };
    let hs = 1e-6;
    let mut t = 0.0f64;
    let mut edges = Vec::new();
    while t < t_end {
        let yn = rk4(y, high, powered, hs);
        if g(yn, high, powered) > 0.0 && g(y, high, powered) <= 0.0 {
            let (mut lo, mut hi) = (0.0, hs);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if g(rk4(y, high, powered, mid), high, powered) > 0.0 { hi = mid } else { lo = mid }
            }
            y = rk4(y, high, powered, hi);
            t += hi;
            if !powered {
                powered = true;
            } else {
                high = !high;
            }
            edges.push((t, if high { 1.0 } else { 0.0 }));
            continue;
        }
        t += hs;
        y = yn;
    }
    edges
}

/// W0's voltage band at `v` (R121): 1e-4 + 1e-3 |v|.
fn w0_v(v: f64) -> f64 {
    1e-4 + 1e-3 * v.abs()
}

/// The time the timing capacitor takes to cross W0 of the threshold an
/// interval ends on: the threshold over its slope there, the rail settled
/// by hand at that crossing. An interval that ends on a fall ended an
/// OUT-high phase (charging to 2/3 Vr); one that ends on a rise ended an
/// OUT-low phase (discharging to 1/3 Vr).
fn tau_w0(b: &Board, h: &Hand, ends_on_fall: bool) -> f64 {
    let frac = if ends_on_fall { 2.0 / 3.0 } else { 1.0 / 3.0 };
    let vr = settled(b, h, !ends_on_fall, frac);
    let slope = draw(b, h, vr, frac * vr, !ends_on_fall, true).1.abs();
    w0_v(frac * vr) / slope
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
fn judge(name: &str, b: &Board, h: &Hand, got: &[(f64, f64)], truth: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    let n = got.len().min(truth.len());
    assert!(n >= 8, "{name}: {} edges against the truth's {}", got.len(), truth.len());
    for i in 0..n {
        assert_eq!(got[i].1, truth[i].1, "{name}: edge {i} level {} against the truth's {}", got[i].1, truth[i].1);
    }
    let mut worst = 0.0f64;
    let mut at = 0;
    for i in 2..n {
        let d = (got[i].0 - got[i - 1].0) - (truth[i].0 - truth[i - 1].0);
        let share = d.abs() / tau_w0(b, h, got[i].1 == 0.0);
        if share > worst {
            (worst, at) = (share, i);
        }
    }
    let edge = |i: usize| (got[i].0 - truth[i].0) * 1e6;
    eprintln!("X555 {name}: worst interval at edge {at} of {n} ({:.6} s, {}), its edges {:+.3} / {:+.3} us off the truth's; hold cuts so far {}", got[at].0, if got[at].1 == 0.0 { "fall" } else { "rise" }, edge(at - 1), edge(at), super::stepped::HOLD_CUTS.with(|c| c.get()));
    // Rises after the power-on one (edge 0): edges 2, 4, ...
    let rises = |e: &[(f64, f64)]| e[..n].iter().skip(1).filter(|x| x.1 == 1.0).map(|x| x.0).collect::<Vec<_>>();
    let (gr, tr) = (rises(got), rises(truth));
    let per = |r: &[f64]| (r[r.len() - 1] - r[0]) / (r.len() - 1) as f64;
    let (pg, pt) = (per(&gr), per(&tr));
    let tw = tau_w0(b, h, true) + tau_w0(b, h, false);
    eprintln!("X555 {name}: the period {:.3} tau_w0 of the rise alone", (pg - pt).abs() / tau_w0(b, h, false));
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
    let h = hand(prog);
    let t = truth_from(b, &h, t_end, false);
    let got = exact_edges(prog, stop, t_end, trap);
    let r = judge(name, b, &h, &got, &t);
    eprintln!("X555 {name}{} stop {stop:.3e}: period {:.2} us (truth {:.2}, {:+.3} %), period {:.3} tau_w0, worst interval {:.3} tau_w0", if trap { " trap" } else { "" }, r.2, r.3, (r.2 / r.3 - 1.0) * 100.0, r.1, r.0);
    r
}

/// The hand-derived rail (X555C finding 4) against the rail T555 read off
/// the engine settled: within W0 of the rail at each phase's crossing (the
/// engine's was read at an unrecorded point of its phase, and the OUT-high
/// rail moves with the timing current, (1/3) Vr / (RA + RB) over the phase:
/// 1.04 mV on 77, 0.60 mV on 78).
#[test]
fn x555_truths_rail_by_hand_matches_the_engines_settled_rail() {
    for (name, prog, b) in [("77", P77, &B77), ("78", P78, &B78)] {
        let h = hand(prog);
        for (high, frac, seen) in [(true, 2.0 / 3.0, b.seen_high), (false, 1.0 / 3.0, b.seen_low)] {
            let vr = settled(b, &h, high, frac);
            let at_start = settled(b, &h, high, 1.0 - frac);
            eprintln!("X555C {name} OUT {}: E {:.6} r {:.4}: rail by hand {vr:.6} V at its crossing ({at_start:.6} at the phase's start), the engine's settled {seen:.6} ({:+.3} mV, {:.3} W0)", if high { "high" } else { "low" }, h.e, h.r, (vr - seen) * 1e3, (vr - seen).abs() / w0_v(seen));
            assert!((vr - seen).abs() <= w0_v(seen), "{name}: the rail by hand {vr} V against the engine's settled {seen} V");
        }
    }
}

/// Exact mode's fixture-77 555 at 1 ms and 1/60 s host stops: the mean
/// period within W0 of the truth (as a time: the timing capacitor's W0 at
/// the threshold over its slope there, 2.4 us at the trigger and 6.6 us at
/// the threshold, 9.0 us for a period), and each edge's interval within
/// tau_w0 at 1 ms. Before X555 the period ran 1.4 % long (52 us a cycle,
/// 5.8 of the period's 9.0 us), an interval up to 9.7 tau_w0 long.
///
/// At 1/60 s an interval is pinned instead (X555C): against the truth by
/// hand (finding 4) single intervals run up to 1.43 tau_w0 off there (an
/// OUT-high interval at 45 ms, its edges 39 and 42 us early, the error
/// accumulated over the run), and the trapezoidal option, ruling R160's
/// reference path, is as far off at the coarse stops (`x555_table`: 1.26
/// on 77, 2.42 on 78 at 1 ms), while both are within 0.001 at 10 us
/// stops: the step controller's error summed over a long run of steps, not
/// the cell partition's order or its hold (no hold cut on this board). The
/// truth T555 read off the engine's settled rail ran 0.4 us a period short
/// and hid it (worst 0.93). An open item for the coordinator, not this fix:
/// EDGE closed it (the crossing landed by one plain step, and each step's
/// error on the approach held to a share of the crossing's time, see
/// `edge_fixture_77s_...`), so both stops are held to tau_w0 again.
#[test]
fn exact_modes_555_on_a_cell_keeps_its_period_at_coarse_host_stops() {
    // Measured (X555C): worst interval 0.977 and 1.430 tau_w0, period
    // 0.349 and 0.914; the 1/60 s interval pinned about 10 % over.
    for (stop, pin) in [(1e-3, 1.0), (1.0 / 60.0, 1.0)] {
        let (worst, period, _, _) = run("77", P77, &B77, stop, 0.05, false);
        assert!(worst <= pin, "fixture 77 at {stop:e} s stops: an interval {worst:.3} tau_w0 off the truth's, past its pin {pin}");
        assert!(period <= 1.0, "fixture 77 at {stop:e} s stops: the period {period:.3} tau_w0 off the truth's");
    }
}

/// The table: both fixtures at 10 us, 100 us, 1 ms and 1/60 s stops, exact
/// mode as it is and with the trapezoidal option (T555's reference path).
/// Asserts the period within W0 at every stop on both; lists the intervals
/// past tau_w0 (X555C: at 1 ms and 1/60 s, plain and trapezoid alike, see
/// the test above).
#[test]
#[ignore = "the X555 table: run by hand, --ignored --nocapture"]
fn x555_table() {
    let (mut bad, mut over) = (Vec::new(), Vec::new());
    for (name, prog, b) in [("77", P77, &B77), ("78", P78, &B78)] {
        for trap in [false, true] {
            for stop in [1e-5, 1e-4, 1e-3, 1.0 / 60.0] {
                let (worst, period, _, _) = run(name, prog, b, stop, 0.1, trap);
                let what = format!("{name}{} {stop:e}", if trap { " trap" } else { "" });
                if worst > 1.0 {
                    over.push(format!("{what}: {worst:.3}"));
                }
                if period > 1.0 {
                    bad.push(what);
                }
            }
        }
    }
    eprintln!("X555 table: intervals past tau_w0: {over:?}");
    assert!(bad.is_empty(), "periods outside W0: {bad:?}");
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
        let h = hand(prog);
        let warm = truth_from(b, &h, 0.1, true);
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
                worst = worst.max(d.abs() / tau_w0(b, &h, got[i].1 == 0.0));
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

/// EDGE: every edge's accumulated drift against the truth over a long run,
/// in shares of the tau_w0 of the threshold that edge closes on, and where
/// the worst sits. `warm`: default mode (FORCE_STEPPED: the timing
/// capacitor on a stepped partition, the rail default's algebraic hub)
/// against the truth's warm start; else exact mode against the cold one,
/// its power-on rise (edge 0, a crossing of the rail, not of a timing
/// threshold) not judged itself but carried by every edge after it.
/// Returns (worst share, its edge, the mean signed drift of the last ten
/// edges in us, edges).
fn drift(name: &str, prog: &[f64], b: &Board, h: &Hand, truth: &[(f64, f64)], warm: bool, stop: f64, t_end: f64) -> (f64, usize, f64, usize) {
    let t = std::time::Instant::now();
    let got = if warm {
        edges_with(prog, stop, t_end, |o| o.force_stepped = true)
    } else {
        exact_edges(prog, stop, t_end, false)
    };
    let secs = t.elapsed().as_secs_f64();
    // The last edge of either may sit past the other's run end.
    let n = got.len().min(truth.len());
    assert!(n + 1 >= got.len().max(truth.len()) && n >= 8, "{name}: {} edges against the truth's {}", got.len(), truth.len());
    let (mut worst, mut at) = (0.0f64, 0);
    for i in 0..n {
        assert_eq!(got[i].1, truth[i].1, "{name}: edge {i} level {} against the truth's {}", got[i].1, truth[i].1);
        if i == 0 && !warm {
            continue;
        }
        let share = (got[i].0 - truth[i].0).abs() / tau_w0(b, h, got[i].1 == 0.0);
        if share > worst {
            (worst, at) = (share, i);
        }
    }
    let d = |i: usize| (got[i].0 - truth[i].0) * 1e6;
    let tail = (n.saturating_sub(10)..n).map(d).sum::<f64>() / n.min(10) as f64;
    let mark = |k: usize| d((n - 1) * k / 4);
    eprintln!("EDGE {name} stop {stop:.3e}: {n} edges, worst drift {worst:.3} tau_w0 at edge {at} ({:.6} s, {:+.3} us); drift at quarters {:+.3} {:+.3} {:+.3} {:+.3} {:+.3} us; edge 0 {:+.3} us; run {secs:.1} s", got[at].0, d(at), mark(0), mark(1), mark(2), mark(3), mark(4), d(0));
    (worst, at, tail, n)
}

/// EDGE: fixtures 77 and 78 over 1 s in exact mode at 1 ms and 1/60 s host
/// stops: every edge within one tau_w0 of the truth's, its drift
/// accumulated over the whole run. Before EDGE the step controller held
/// each step's voltage error to W0, the crossing landed by one plain step,
/// and the edges drifted early: -432 us (78, 1 ms) and -555 us (1/60 s) by
/// 1 s, linear in time (X555C: 39 to 42 us by 45 ms on 77). At 10 us stops
/// exact mode is within 0.08 (77) and 0.37 tau_w0 (78) of the truth over
/// the same second (`edge_fine_stop_drift`).
///
/// `warm` runs default mode under FORCE_STEPPED against the warm truth
/// instead (`edge_default_mode_long_run`): the truth is not default's
/// model there (see that test).
fn long_run_drift(name: &str, prog: &[f64], b: &Board, modes: &[bool]) {
    let t_end = 1.0;
    let h = hand(prog);
    let mut bad = Vec::new();
    for &warm in modes {
        let truth = truth_from(b, &h, t_end, warm);
        for stop in [1e-3, 1.0 / 60.0] {
            let what = format!("{name} {}", if warm { "default+stepped" } else { "exact" });
            let (worst, at, _, _) = drift(&what, prog, b, &h, &truth, warm, stop, t_end);
            if worst > 1.0 {
                bad.push(format!("{what} {stop:.3e}: {worst:.3} at edge {at}"));
            }
        }
    }
    assert!(bad.is_empty(), "edges drifted past tau_w0 over {t_end} s: {bad:?}");
}

#[test]
fn edge_fixture_77s_555_holds_every_edge_within_tau_w0_over_a_long_run() {
    long_run_drift("77", P77, &B77, &[false]);
}

#[test]
fn edge_fixture_78s_555_holds_every_edge_within_tau_w0_over_a_long_run() {
    long_run_drift("78", P78, &B78, &[false]);
}

/// EDGE: default mode under FORCE_STEPPED over the same second (the timing
/// capacitor on a stepped partition, the rail default's algebraic hub),
/// against the truth's warm start. Not a pass/fail: the hand truth
/// integrates the rail as the cell and its capacitor (exact mode's model)
/// and default's rail is algebraic, so default drifts from it at every
/// stop, the most at the finest (EDGE: +371 us (77) and +120 us (78) by
/// 1 s at 10 us stops, +187 / +44 us on 77 at 1 ms / 1/60 s), and at
/// coarse stops it has lost edges (78 at 1 ms: 814 against 816; 77 at
/// 1/60 s before EDGE: 525 against 528). An open item: no truth of
/// default's own model, and a cadence dependence outside the step
/// controller.
#[test]
#[ignore = "the EDGE default-mode long run: run by hand, --ignored --nocapture"]
fn edge_default_mode_long_run() {
    let off = std::env::var("EDGE_OFF").ok().and_then(|v| v.parse::<u8>().ok()).unwrap_or(0);
    super::stepped::EDGE_OFF.with(|c| c.set(off));
    for (name, prog, b) in [("77", P77, &B77), ("78", P78, &B78)] {
        let h = hand(prog);
        let truth = truth_from(b, &h, 1.0, true);
        for stop in [1e-5, 1e-3, 1.0 / 60.0] {
            let got = edges_with(prog, stop, 1.0, |o| o.force_stepped = true);
            let n = got.len().min(truth.len());
            let d = |i: usize| (got[i].0 - truth[i].0) * 1e6;
            let lv = (0..n).find(|&i| got[i].1 != truth[i].1);
            eprintln!("EDGE default {name} stop {stop:.3e}: {} edges (truth {}), first level mismatch {lv:?}, drift at quarters {:+.3} {:+.3} {:+.3} {:+.3} us", got.len(), truth.len(), d(n / 4), d(n / 2), d(3 * n / 4), d(n - 1));
        }
    }
}

/// EDGE: the drift at 10 us host stops (the truth's own check: there the
/// steps are short and the controller's error small), both modes.
#[test]
#[ignore = "the EDGE fine-stop drift: run by hand, --ignored --nocapture"]
fn edge_fine_stop_drift() {
    for (name, prog, b) in [("77", P77, &B77), ("78", P78, &B78)] {
        let h = hand(prog);
        for warm in [true, false] {
            let truth = truth_from(b, &h, 1.0, warm);
            let what = format!("{name} {}", if warm { "default+stepped" } else { "exact" });
            drift(&what, prog, b, &h, &truth, warm, 1e-5, 1.0);
        }
    }
}

/// EDGE: the step trace behind the drift: each interval's error against the
/// truth beside the crossing-time error its steps' own estimates predict,
/// `sum (coarse - fine) / 3 / slope` over the timing node's steps since the
/// last edge (a voltage error made anywhere in a single-pole RC phase moves
/// the crossing by itself over the slope where it was made: the error and
/// the slope decay together), with the steps' count and lengths.
#[test]
#[ignore = "the EDGE trace: run by hand, --ignored --nocapture"]
fn edge_trace() {
    use super::stepped::WATCH_LOG;
    for (name, prog, b, warm) in [("78", P78, &B78, false), ("77", P77, &B77, false), ("77", P77, &B77, true)] {
        let h = hand(prog);
        let truth = truth_from(b, &h, 0.03, warm);
        for stop in [1e-3, 1.0 / 60.0] {
            WATCH_LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
            let got = if warm { edges_with(prog, stop, 0.03, |o| o.force_stepped = true) } else { exact_edges(prog, stop, 0.03, false) };
            let log = WATCH_LOG.with(|l| l.borrow_mut().take().unwrap());
            let mut nodes: Vec<usize> = log.iter().map(|r| r.3).collect();
            nodes.sort();
            nodes.dedup();
            eprintln!("EDGETRACE {name} warm={warm} stop {stop:.3e}: watched nodes {nodes:?}");
            let ct = 4;
            let rows: Vec<_> = log.iter().filter(|r| r.3 == ct).collect();
            let n = got.len().min(truth.len());
            for i in 1..n.min(9) {
                let (a, z) = (got[i - 1].0, got[i].0);
                let st: Vec<_> = rows.iter().filter(|r| r.1 > a + 1e-12 && r.1 <= z + 1e-12).collect();
                let mut pred = 0.0;
                let mut prev: Option<(f64, f64)> = None;
                let mut hs = Vec::new();
                for r in &st {
                    if let Some((t0, x0)) = prev {
                        let slope = (r.4 - x0) / (r.1 - t0);
                        if slope.abs() > 0.0 && r.5.is_finite() {
                            pred += r.5 / 3.0 / slope;
                        }
                    }
                    prev = Some((r.1, r.4));
                    hs.push(r.2);
                }
                if i == 3 {
                    let mut prev: Option<(f64, f64)> = None;
                    for r in &st {
                        let slope = prev.map(|(t0, x0)| (r.4 - x0) / (r.1 - t0)).unwrap_or(f64::NAN);
                        eprintln!("EDGETRACE     t {:.7} h {:.3e} x {:.6} diff {:+.3e} slope {:.1} contrib {:+.4} us", r.1, r.2, r.4, r.5, slope, r.5 / 3.0 / slope * 1e6);
                        prev = Some((r.1, r.4));
                    }
                }
                let err = ((got[i].0 - got[i - 1].0) - (truth[i].0 - truth[i - 1].0)) * 1e6;
                let hmax = hs.iter().cloned().fold(0.0, f64::max);
                eprintln!("EDGETRACE   interval {i} ({}): err {err:+.3} us, predicted {:+.3} us (sign: time = -dv/slope, so compare -pred), {} steps, h max {hmax:.3e}", if got[i].1 == 0.0 { "high" } else { "low" }, pred * 1e6, st.len());
            }
        }
    }
}
