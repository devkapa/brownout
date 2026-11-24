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
    let (chain, dvc) = timing_chain(b, vr, vc, high, powered);
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

/// The timing chain's draw on the rail and the timing capacitor's dVc/dt
/// ([`draw`]'s first item).
fn timing_chain(b: &Board, vr: f64, vc: f64, high: bool, powered: bool) -> (f64, f64) {
    if !powered || high {
        ((vr - vc) / (b.ra + b.rb), (vr - vc) / ((b.ra + b.rb) * b.c))
    } else {
        let vd = (vr / b.ra + vc / b.rb) / (1.0 / b.ra + 1.0 / b.rb + 1.0 / RD);
        ((vr - vd) / b.ra, (vd - vc) / (b.rb * b.c))
    }
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
/// `warm` runs default mode under FORCE_STEPPED against the warm hand truth
/// instead. That truth integrates the rail as the cell and its capacitor,
/// exact mode's model, which default's algebraic hub is not, so no test
/// passes it: `edge_default_mode_long_run` judges default against
/// [`truth_alg`], default's own model, through [`drift`] directly.
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

/// EDGE, EDGE2: default mode under FORCE_STEPPED (the timing capacitor on
/// a stepped partition beside the cell's hub) over the same second, against
/// default's own model ([`truth_alg`]: the rail algebraic, warm start; the
/// hand truth of the tests above integrates the rail as the cell and its
/// capacitor, exact mode's model): every edge there, at its level, within
/// tau_w0, at 1 ms and 1/60 s stops.
///
/// DEFEDGE measured it failing: 78 at 1 ms lost 2 edges of 816 (+2971 us),
/// at 1/60 s 4 (+3877 us); +142 us even at 10 us stops. The lost edges were
/// late flips: the timing trial is planned inside its commit, before the
/// rail loop solves the rail for that commit's draw, so about one OUT-high
/// phase in five landed its crossing 1e-9 V past 2/3 of a rail a step or
/// two old; the solve lifted the threshold the 555 reads a few uV above the
/// landing, and the next trial, planned against the old rail again, read
/// the node past it, found no crossing and ran ~28 us before the flip
/// landed at its end. EDGE2: a trial whose crossing the rail loop changes
/// is planned again against the solved rail (`SteppedSolver::trial_crossing`).
/// EDGE4: planned against that rail held, the timing capacitor still
/// charged toward a rail half a step behind (it rises 0.64 V/s through 77's
/// OUT-high phase): 77 ran +1.74 us late by 1 s at 1 ms (1.028 tau_w0) once
/// EDGE3 restored the replan's step. The replan reads the rail as a line at
/// its last committed slope (`SteppedSolver::set_rail_lin`): 77 0.475 /
/// 0.581, 78 0.606 / 0.658 tau_w0 (1 ms / 1/60 s).
#[test]
fn edge_default_mode_long_run() {
    let mut bad = Vec::new();
    for (name, prog, b) in [("77", P77, &B77), ("78", P78, &B78)] {
        let h = hand(prog);
        let truth = truth_alg(b, &h, [0.0; 2], 1.0);
        for stop in [1e-3, 1.0 / 60.0] {
            let (worst, at, _, _) = drift(&format!("{name} default+stepped vs alg"), prog, b, &h, &truth, true, stop, 1.0);
            if worst > 1.0 {
                bad.push(format!("{name} {stop:.3e}: {worst:.3} tau_w0 at edge {at}"));
            }
        }
    }
    assert!(bad.is_empty(), "FORCE_STEPPED default drifted past tau_w0 over 1 s: {bad:?}");
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

// DEFEDGE: production default mode (no FORCE_STEPPED, no exact mode) on a
// 555 astable beside a hub rail, clocking a counter, at the app's stop sizes.

use super::defedge_data::{P78_PSU, P78_SPLIT, P78_SPLIT_PSU, SAPCLK_BAT, SAPCLK_PSU};

/// The SAP-1 clock module's astable: R1 1k, R2 + RPOT1 1k + 1 mohm, C2 1 uF,
/// OUT through R3 220 ohm to LED1 (and an LS input, no load in the truth).
const BSAP: Board = Board { ra: 1000.0, rb: 1000.001, c: 1e-6, c_dec: 0.0, led_r: Some(220.0), seen_high: 0.0, seen_low: 0.0 };

/// [`hand`] for a board on a cell or on a bench supply: a bench supply in CV
/// holds its setpoint exactly (r = 0).
fn hand_any(prog: &[f64]) -> Hand {
    let p = Program::parse(prog).unwrap();
    if p.devices.iter().any(|d| d.kind == crate::kinds::BATTERY_PACK) {
        return hand(prog);
    }
    let psu = p.devices.iter().find(|d| d.kind == crate::kinds::BENCH_PSU).unwrap();
    let law = p.devices.iter().find_map(|d| crate::devices::led::led_law_at(d, p.options.ambient_c));
    Hand { e: psu.params[0], r: 0.0, law }
}

/// The truth of default mode's model: the rail algebraic (the hub: the cell's
/// E less r times the 555's draw at each instant, no rail capacitor; on a
/// bench supply exactly its setpoint), the timing capacitor an ODE from
/// [`draw`]; warm start (the 555 powered, OUT high, Ct empty), RK4 at 1 us
/// with each crossing bisected, thresholds 2/3 and 1/3 of the rail then.
/// `other`: the rest of the board's draw on the rail with OUT low and with
/// OUT high, A (0 on a board whose 555 is the rail's only load).
///
/// The rail at each OUT level is affine in the timing capacitor's voltage to
/// rounding (the timing chain's current is affine in it, and the rest moves
/// with a rail that moves under a millivolt through a phase), so it is
/// solved at two points per level, the line checked at a third within 1e-9 V
/// (the LED's junction solve inside every step made the SAP-1 clock's truth
/// 88 s a run in release).
fn truth_alg(b: &Board, h: &Hand, other: [f64; 2], t_end: f64) -> Vec<(f64, f64)> {
    let solve = |vc: f64, high: bool| {
        let mut vr = h.e;
        if h.r > 0.0 {
            for _ in 0..100 {
                vr = h.e - h.r * (draw(b, h, vr, vc, high, true).0 + other[high as usize]);
            }
        }
        vr
    };
    let line = [false, true].map(|high| {
        let (v0, v1, vm) = (solve(0.0, high), solve(h.e, high), solve(0.5 * h.e, high));
        assert!((vm - 0.5 * (v0 + v1)).abs() < 1e-9, "the rail is not affine in the timing capacitor: {v0} {vm} {v1}");
        (v0, (v1 - v0) / h.e)
    });
    let rail = |vc: f64, high: bool| line[high as usize].0 + line[high as usize].1 * vc;
    let f = |vc: f64, high: bool| timing_chain(b, rail(vc, high), vc, high, true).1;
    let rk4 = |vc: f64, high: bool, hs: f64| {
        let k1 = f(vc, high);
        let k2 = f(vc + hs / 2.0 * k1, high);
        let k3 = f(vc + hs / 2.0 * k2, high);
        let k4 = f(vc + hs * k3, high);
        vc + hs / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    };
    let g = |vc: f64, high: bool| if high { vc - 2.0 / 3.0 * rail(vc, high) } else { rail(vc, high) / 3.0 - vc };
    let (hs, mut t, mut vc, mut high) = (1e-6, 0.0f64, 0.0f64, true);
    let mut edges = Vec::new();
    while t < t_end {
        let vn = rk4(vc, high, hs);
        if g(vn, high) > 0.0 && g(vc, high) <= 0.0 {
            let (mut lo, mut hi) = (0.0, hs);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if g(rk4(vc, high, mid), high) > 0.0 { hi = mid } else { lo = mid }
            }
            vc = rk4(vc, high, hi);
            t += hi;
            high = !high;
            edges.push((t, if high { 1.0 } else { 0.0 }));
            continue;
        }
        t += hs;
        vc = vn;
    }
    edges
}

/// The truth of a bench supply with a rail capacitor, cold (fixture 78 on a
/// bench supply): the supply in CC at `i_lim` charges `c_dec` from 0 V, less
/// the 555's draw ([`draw`]: its timing chain unpowered, and from 4.05 V its
/// OUT's cross-current too), until the rail reaches the setpoint, then holds
/// it in CV. The 555 powers on with OUT high at 4.05 V (edge 0, a rise),
/// flips at 2/3 and 1/3 of the rail then. RK4 at 1 us, every event bisected.
fn truth_psu_cold(b: &Board, h: &Hand, i_lim: f64, t_end: f64) -> Vec<(f64, f64)> {
    let (mut powered, mut high, mut cv) = (false, true, false);
    let f = |y: [f64; 2], powered: bool, high: bool, cv: bool| {
        let (i, dvc) = draw(b, h, y[0], y[1], high, powered);
        [if cv { 0.0 } else { (i_lim - i) / b.c_dec }, dvc]
    };
    let rk4 = |y: [f64; 2], p: bool, hi: bool, cv: bool, hs: f64| {
        let k1 = f(y, p, hi, cv);
        let k2 = f([y[0] + hs / 2.0 * k1[0], y[1] + hs / 2.0 * k1[1]], p, hi, cv);
        let k3 = f([y[0] + hs / 2.0 * k2[0], y[1] + hs / 2.0 * k2[1]], p, hi, cv);
        let k4 = f([y[0] + hs * k3[0], y[1] + hs * k3[1]], p, hi, cv);
        let mut n = [y[0] + hs / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]), y[1] + hs / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])];
        if cv {
            n[0] = h.e;
        }
        n
    };
    // The events in force: the power-on, the setpoint, the 555's threshold.
    let gs = |y: [f64; 2], p: bool, hi: bool, cv: bool| -> [f64; 3] {
        [if p { f64::NEG_INFINITY } else { y[0] - V_ON }, if cv { f64::NEG_INFINITY } else { y[0] - h.e }, if !p { f64::NEG_INFINITY } else if hi { y[1] - 2.0 / 3.0 * y[0] } else { y[0] / 3.0 - y[1] }]
    };
    let (hs, mut t, mut y) = (1e-6, 0.0f64, [0.0f64, 0.0f64]);
    let mut edges = Vec::new();
    while t < t_end {
        let yn = rk4(y, powered, high, cv, hs);
        let (g0, g1) = (gs(y, powered, high, cv), gs(yn, powered, high, cv));
        if let Some(k) = (0..3).find(|&k| g1[k] > 0.0 && g0[k] <= 0.0) {
            let (mut lo, mut hi) = (0.0, hs);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if gs(rk4(y, powered, high, cv, mid), powered, high, cv)[k] > 0.0 { hi = mid } else { lo = mid }
            }
            // The earliest event of the step: any other that fires by `hi` too
            // is taken on the next pass, from this landing.
            y = rk4(y, powered, high, cv, hi);
            t += hi;
            match k {
                0 => {
                    powered = true;
                    edges.push((t, 1.0));
                }
                1 => {
                    cv = true;
                    y[0] = h.e;
                }
                _ => {
                    high = !high;
                    edges.push((t, if high { 1.0 } else { 0.0 }));
                }
            }
            continue;
        }
        t += hs;
        y = yn;
    }
    edges
}

/// What a board's edges are judged against.
#[derive(Clone, Copy)]
enum Truth {
    /// [`truth_alg`]: the rail algebraic, warm start.
    Alg,
    /// [`truth_psu_cold`] at this CC limit, A.
    PsuCold(f64),
    /// [`truth_alg`] with the rest of the board's draw on the rail: the
    /// SAP-1 clock on its cell, whose rail carries the module's other parts
    /// (U2 and U3's OUT cross-currents, the LS gates, CLK's LED), a draw no
    /// hand model of U1 alone holds (against U1's alone the edges run 922
    /// us early by 1 s, the same at every stop). That draw is constant
    /// through each OUT phase (CLK follows OUT; the counter moves only at
    /// OUT's fall), so it is two numbers, read off the engine's rail once
    /// in each phase ([`other_draw`]); the timing is integrated as ever.
    /// Exact mode with the trapezoidal option at 10 us stops (ruling R160's
    /// reference) agrees with default within 0.381 tau_w0 over the second
    /// (the survey), 16 s a run in release.
    CellOther { rail: usize, ct: usize },
}

/// One board: its 555's OUT slot (0 on all four), the counter's QA slot,
/// whether the counter advances on OUT's fall (the SAP-1's U43 counts /CLK),
/// and its truth.
struct Clocked {
    name: &'static str,
    prog: &'static [f64],
    b: &'static Board,
    qa: usize,
    on_fall: bool,
    truth: Truth,
}

const CLOCKED: [Clocked; 4] = [
    Clocked { name: "78 cell", prog: P78, b: &B78, qa: 2, on_fall: false, truth: Truth::Alg },
    Clocked { name: "78 psu", prog: P78_PSU, b: &B78, qa: 2, on_fall: false, truth: Truth::PsuCold(1.0) },
    Clocked { name: "sap1-clock cell", prog: SAPCLK_BAT, b: &BSAP, qa: 18, on_fall: true, truth: Truth::CellOther { rail: 1, ct: 2 } },
    Clocked { name: "sap1-clock psu", prog: SAPCLK_PSU, b: &BSAP, qa: 18, on_fall: true, truth: Truth::Alg },
];

/// Fixture 78 with its timing capacitor as two 0.5 uF in parallel: two
/// capacitors, so default mode steps the timing partition (the closed form
/// takes one), beside the cell's hub rail and beside a bench supply's.
const STEPPED_CLOCKED: [Clocked; 2] = [
    Clocked { name: "78 split cell", prog: P78_SPLIT, b: &B78, qa: 2, on_fall: false, truth: Truth::Alg },
    Clocked { name: "78 split psu", prog: P78_SPLIT_PSU, b: &B78, qa: 2, on_fall: false, truth: Truth::PsuCold(1.0) },
];

fn truth_of(c: &Clocked, t_end: f64) -> Vec<(f64, f64)> {
    let h = hand_any(c.prog);
    match c.truth {
        Truth::Alg => truth_alg(c.b, &h, [0.0; 2], t_end),
        Truth::PsuCold(i_lim) => truth_psu_cold(c.b, &h, i_lim, t_end),
        Truth::CellOther { rail, ct } => truth_alg(c.b, &h, other_draw(c, &h, rail, ct), t_end),
    }
}

/// The rail's draw past the 555's own with OUT low and with OUT high: the
/// engine's rail and timing capacitor read at the middle of the second OUT
/// phase of each level (exact mode at 10 us stops, so the read is the
/// reference path's), less [`draw`] there.
fn other_draw(c: &Clocked, h: &Hand, rail: usize, ct: usize) -> [f64; 2] {
    let exact = |o: &mut crate::program::Options| {
        o.no_rail_shortcut = true;
        o.no_solution_cache = true;
    };
    let edges = edges_with(c.prog, 1e-5, 0.012, exact);
    let mut out = [0.0; 2];
    for level in [0.0, 1.0] {
        let i = edges.iter().enumerate().filter(|(_, e)| e.1 == level).nth(std::env::var("DEFEDGE_NTH").ok().and_then(|v| v.parse().ok()).unwrap_or(1)).unwrap().0;
        let mid = 0.5 * (edges[i].0 + edges[i + 1].0);
        let mut p = Program::parse(c.prog).unwrap();
        exact(&mut p.options);
        let mut e = Engine::load(&p).unwrap();
        e.advance(mid).unwrap();
        let (vr, vc) = (e.voltages()[rail], e.voltages()[ct]);
        out[level as usize] = (h.e - vr) / h.r - draw(c.b, h, vr, vc, level == 1.0, true).0;
    }
    out
}

/// Every digital flip `(t, slot, level)` under the options `set` leaves, the
/// host stopping every `stop`.
fn flips_with(prog: &[f64], stop: f64, t_end: f64, set: impl Fn(&mut crate::program::Options)) -> Vec<(f64, usize, f64)> {
    let mut p = Program::parse(prog).unwrap();
    set(&mut p.options);
    let mut e = Engine::load(&p).unwrap();
    e.trace_enable(true);
    let mut out = Vec::new();
    let n = (t_end / stop).ceil() as usize;
    for k in 1..=n {
        e.advance((k as f64 * stop).min(t_end)).unwrap();
        out.extend(e.trace().chunks(3).map(|ch| (ch[0], ch[1] as usize, ch[2])));
        e.trace_clear();
    }
    out
}

/// A run judged against its truth, every count taken before `t_end - 1 ms`
/// so an edge either side of the run's end does not count against it: the
/// 555's OUT edges and the truth's, the counter's QA flips (one per count),
/// OUT's clocking edges and the truth's, the first edge whose level differs
/// from the truth's, the worst |t - truth| in shares of the tau_w0 of the
/// threshold that edge closes on (a power-on rise, closing on no threshold,
/// carried by every edge after it and not judged itself), its edge, and the
/// last edge's drift, us.
struct Judged {
    got: usize,
    truth: usize,
    qa_flips: usize,
    clocks: usize,
    truth_clocks: usize,
    mismatch: Option<usize>,
    worst: f64,
    at: usize,
    last_us: f64,
    secs: f64,
}

impl Judged {
    /// Every edge there, at its level, within tau_w0, and the counter
    /// advanced once for each clocking edge.
    fn holds(&self) -> bool {
        self.got == self.truth && self.mismatch.is_none() && self.qa_flips == self.clocks && self.clocks == self.truth_clocks && self.worst <= 1.0
    }
}

fn judge_run(c: &Clocked, truth: &[(f64, f64)], stop: f64, t_end: f64, set: impl Fn(&mut crate::program::Options)) -> Judged {
    let h = hand_any(c.prog);
    let t0 = std::time::Instant::now();
    let fl = flips_with(c.prog, stop, t_end, set);
    let secs = t0.elapsed().as_secs_f64();
    let cut = t_end - 1e-3;
    let got: Vec<(f64, f64)> = fl.iter().filter(|f| f.1 == 0 && f.0 < cut).map(|f| (f.0, f.2)).collect();
    let tr: Vec<(f64, f64)> = truth.iter().cloned().filter(|e| e.0 < cut).collect();
    let clocking = if c.on_fall { 0.0 } else { 1.0 };
    let qa_flips = fl.iter().filter(|f| f.1 == c.qa && f.0 < cut).count();
    let clocks = got.iter().filter(|e| e.1 == clocking).count();
    let truth_clocks = tr.iter().filter(|e| e.1 == clocking).count();
    let n = got.len().min(tr.len());
    let mismatch = (0..n).find(|&i| got[i].1 != tr[i].1);
    // A power-on rise: an edge to high with no fall before it.
    let power_on = |i: usize| i == 0 && tr[0].1 == 1.0;
    let (mut worst, mut at) = (0.0f64, 0);
    for i in (0..mismatch.unwrap_or(n)).filter(|&i| !power_on(i)) {
        let share = (got[i].0 - tr[i].0).abs() / tau_w0(c.b, &h, got[i].1 == 0.0);
        if share > worst {
            (worst, at) = (share, i);
        }
    }
    let last_us = if n > 0 { (got[n - 1].0 - tr[n - 1].0) * 1e6 } else { f64::NAN };
    Judged { got: got.len(), truth: tr.len(), qa_flips, clocks, truth_clocks, mismatch, worst, at, last_us, secs }
}

fn show(c: &Clocked, what: &str, stop: f64, j: &Judged) -> String {
    let line = format!(
        "DEFEDGE {} {what} stop {stop:.3e}: OUT edges {} (truth {}), counter QA flips {} vs OUT clocking edges {} (truth {}), level mismatch {:?}, worst drift {:.3} tau_w0 at edge {}, last edge {:+.3} us, run {:.1} s",
        c.name, j.got, j.truth, j.qa_flips, j.clocks, j.truth_clocks, j.mismatch, j.worst, j.at, j.last_us, j.secs
    );
    eprintln!("{line}");
    line
}

/// DEFEDGE: production default mode (no FORCE_STEPPED, no exact mode) can
/// drop no 555 OUT edge beside a hub rail. A 555 astable clocking a 74LS161
/// over 1 s at the app's host stops (1/60 s frames and 1 ms): fixture 78 on
/// its cell and on a bench supply (5 V, 1 A, its 47 uF charged from 0 V in
/// CC), and the catalog SAP-1's clock module cut from the battery SAP-1
/// (U1-U6, its switches in RUN, clocking its step counter U43 on /CLK) on
/// its cell and on a bench supply. Every OUT edge the truth has is there,
/// at its level, within the tau_w0 of its threshold over the whole second,
/// and the counter counted each clocking edge once.
///
/// Measured (DEFEDGE survey, 1 s): every board at both stops holds every
/// edge, worst 0.001 (78 cell), 0.003 (78 psu, clock psu) and 0.904 (clock
/// cell) tau_w0: each timing capacitor is one capacitor, so the closed form
/// times it. A STEPPED timing partition beside a cell's hub does not hold
/// (`defedge_default_stepped_555_beside_a_cell_hub`, open).
#[test]
fn defedge_default_555_clocks_its_counter_edge_for_edge_beside_a_hub_rail() {
    let t_end = 1.0;
    let mut bad = Vec::new();
    for c in CLOCKED.iter() {
        let truth = truth_of(c, t_end);
        for stop in [1.0 / 60.0, 1e-3] {
            let j = judge_run(c, &truth, stop, t_end, |_| {});
            let line = show(c, "default", stop, &j);
            if !j.holds() {
                bad.push(line);
            }
        }
    }
    assert!(bad.is_empty(), "default mode lost, added or drifted an edge: {bad:#?}");
}

/// DEFEDGE, fixed by EDGE2: the same check on a stepped timing partition in
/// default mode (`STEPPED_CLOCKED`), on the cell and on a bench supply.
/// Measured before EDGE2 (DEFEDGE survey, 1 s): on the cell, wip/edge
/// (bef5f34) lost 2 of 816 edges at 1 ms (+2992 us) and 3 at 1/60 s (+3657
/// us), +220 us at 10 us stops; before EDGE (`EDGE_OFF=3`) every edge but
/// +91 / -493 / +210 us (255, 469, 200 tau_w0). The lost edges were late
/// flips: about one OUT-high phase in five landed its crossing (the node
/// 1e-9 V past the threshold read at the published rail) and the 555 did
/// not flip there; the next step, uncapped since its node already stood
/// past that threshold, ran ~28 us and the flip landed at its end (+26-30
/// us an interval, `defedge_forced_window_probe` with `DEFEDGE_SPLIT=1`).
/// EDGE2 plans a watching hub partition's trial again against the solved
/// rail (`SteppedSolver::trial_crossing`, `SteppedSolver::replan`): every
/// edge of the cell's 816 holds, within 0.75 tau_w0 (1 ms) and 0.70 (1/60
/// s); the bench supply 0.85 / 0.84. FORCE_STEPPED on fixture 78 itself is
/// the same case (`edge_default_mode_long_run`).
#[test]
fn defedge_default_stepped_555_beside_a_cell_hub() {
    let t_end = 1.0;
    let mut bad = Vec::new();
    for c in STEPPED_CLOCKED.iter() {
        let truth = truth_of(c, t_end);
        for stop in [1.0 / 60.0, 1e-3] {
            let j = judge_run(c, &truth, stop, t_end, |_| {});
            let line = show(c, "default", stop, &j);
            if !j.holds() {
                bad.push(line);
            }
        }
    }
    assert!(bad.is_empty(), "a stepped 555 lost, added or drifted an edge: {bad:#?}");
}

/// DEFEDGE survey: every board in production default mode and default under
/// FORCE_STEPPED at 10 us, 1 ms and 1/60 s stops over `DEFEDGE_T` s (1 by
/// default), against its truth.
#[test]
#[ignore = "the DEFEDGE survey: run by hand, --ignored --nocapture"]
fn defedge_survey() {
    let t_end = std::env::var("DEFEDGE_T").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
    let only = std::env::var("DEFEDGE_ONLY").ok();
    for c in CLOCKED.iter().chain(STEPPED_CLOCKED.iter()) {
        if only.as_deref().is_some_and(|o| !c.name.contains(o)) {
            continue;
        }
        let h = hand_any(c.prog);
        let t0 = std::time::Instant::now();
        let truth = truth_of(c, t_end);
        if let Truth::CellOther { rail, ct } = c.truth {
            let o = other_draw(c, &h, rail, ct);
            eprintln!("DEFEDGE {}: other draw OUT low {:.6} A, high {:.6} A", c.name, o[0], o[1]);
            let r = edges_with(c.prog, 1e-5, t_end, |o| {
                o.no_rail_shortcut = true;
                o.no_solution_cache = true;
                o.integration = 1;
            });
            show(c, "R160 ref (exact+trap 10 us) vs truth", 1e-5, &judge_run(c, &truth, 1e-5, t_end, |o| {
                o.no_rail_shortcut = true;
                o.no_solution_cache = true;
                o.integration = 1;
            }));
            show(c, "default vs R160 ref", 1e-3, &judge_run(c, &r, 1e-3, t_end, |_| {}));
        }
        eprintln!("DEFEDGE {}: truth {} edges ({:.1} s), first {:?}, tau_w0 rise {:.3} us fall {:.3} us", c.name, truth.len(), t0.elapsed().as_secs_f64(), &truth[..2], tau_w0(c.b, &h, false) * 1e6, tau_w0(c.b, &h, true) * 1e6);
        for stop in [1e-5, 1e-3, 1.0 / 60.0] {
            show(c, "default", stop, &judge_run(c, &truth, stop, t_end, |_| {}));
            show(c, "forceStepped", stop, &judge_run(c, &truth, stop, t_end, |o| o.force_stepped = true));
        }
    }
}

/// DEFEDGE: FORCE_STEPPED default mode on fixture 78's cell, split: per stop
/// and EDGE switch (`EDGE_OFF`: 0 as built, 3 = before EDGE), the mean
/// error of the OUT-high and OUT-low intervals against [`truth_alg`], the
/// timing partition's steps per phase, and at each host stop the rail the
/// engine publishes against the algebraic rail at its own timing capacitor
/// and OUT level.
#[test]
#[ignore = "the DEFEDGE FORCE_STEPPED split: run by hand, --ignored --nocapture"]
fn defedge_forced_split() {
    use super::stepped::{EDGE_OFF, STEP_LOG};
    let (prog, b) = (P78, &B78);
    let h = hand(prog);
    let truth = truth_alg(b, &h, [0.0; 2], 0.3);
    let rail_of = |vc: f64, high: bool| {
        let mut vr = h.e;
        for _ in 0..100 {
            vr = h.e - h.r * draw(b, &h, vr, vc, high, true).0;
        }
        vr
    };
    for off in [0u8, 3] {
        EDGE_OFF.with(|c| c.set(off));
        for (stop, force) in [(1e-5f64, true), (1e-4, true), (1e-3, true), (1.0 / 60.0, true), (1e-3, false)] {
            STEP_LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
            let mut p = Program::parse(prog).unwrap();
            p.options.force_stepped = force;
            let mut e = Engine::load(&p).unwrap();
            e.trace_enable(true);
            let mut got = Vec::new();
            let (mut rail_err, mut rail_max, mut n_rail) = (0.0f64, 0.0f64, 0);
            let mut high = true;
            let n = (0.3f64 / stop).ceil() as usize;
            for k in 1..=n {
                e.advance((k as f64 * stop).min(0.3)).unwrap();
                for ch in e.trace().chunks(3).filter(|ch| ch[1] == 0.0) {
                    got.push((ch[0], ch[2]));
                    high = ch[2] == 1.0;
                }
                e.trace_clear();
                let (vr, vc) = (e.voltages()[1], e.voltages()[4]);
                let d = vr - rail_of(vc, high);
                rail_err += d;
                rail_max = rail_max.max(d.abs());
                n_rail += 1;
            }
            let log = STEP_LOG.with(|l| l.borrow_mut().take().unwrap());
            let m = got.len().min(truth.len());
            let mean = |fall: bool| {
                let d: Vec<f64> = (1..m).filter(|&i| (got[i].1 == 0.0) == fall).map(|i| (got[i].0 - got[i - 1].0) - (truth[i].0 - truth[i - 1].0)).collect();
                d.iter().sum::<f64>() / d.len().max(1) as f64 * 1e6
            };
            let mut parts: Vec<usize> = log.iter().map(|s| s.p_idx).collect();
            parts.sort();
            parts.dedup();
            let counts: Vec<(usize, usize)> = parts.iter().map(|&p| (p, log.iter().filter(|s| s.p_idx == p).count())).collect();
            eprintln!(
                "DEFEDGE split off={off} force={force} stop {stop:.3e}: {} edges (truth {}), mean interval error high {:+.3} us, low {:+.3} us, last edge {:+.3} us; steps by partition {counts:?} ({:.1} per phase); rail at stops vs algebraic: mean {:+.3e} V, max {:.3e} V",
                got.len(),
                truth.len(),
                mean(true),
                mean(false),
                (got[m - 1].0 - truth[m - 1].0) * 1e6,
                log.len() as f64 / got.len().max(1) as f64,
                rail_err / n_rail as f64,
                rail_max
            );
        }
    }
}

/// DEFEDGE: inside FORCE_STEPPED's host windows on fixture 78's cell, the
/// rail the timing partition's steps imply (each OUT-high step's exact RC
/// solve for the source it charged Ct towards, `(v1 - v0 a) / (1 - a)`,
/// `a = exp(-h / (RA + RB) C)`) against the algebraic rail at the step's
/// mean Ct, bucketed by the step's place in its host window.
#[test]
#[ignore = "the DEFEDGE window probe: run by hand, --ignored --nocapture"]
fn defedge_forced_window_probe() {
    use super::stepped::{EDGE_OFF, WATCH_LOG};
    // DEFEDGE_SPLIT: fixture 78 with its timing capacitor split in two, in
    // production default (a stepped partition there), no FORCE_STEPPED.
    let split = std::env::var("DEFEDGE_SPLIT").is_ok();
    let (prog, b) = (if split { P78_SPLIT } else { P78 }, &B78);
    let h = hand(prog);
    let rail_of = |vc: f64| {
        let mut vr = h.e;
        for _ in 0..100 {
            vr = h.e - h.r * draw(b, &h, vr, vc, true, true).0;
        }
        vr
    };
    let tau = (b.ra + b.rb) * b.c;
    for off in [0u8, 3] {
        EDGE_OFF.with(|c| c.set(off));
        for stop in [1e-5f64, 1e-4, 1e-3] {
            WATCH_LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
            let fl = flips_with(prog, stop, 0.05, |o| o.force_stepped = !split);
            let rows: Vec<(f64, f64)> = WATCH_LOG.with(|l| l.borrow_mut().take().unwrap()).into_iter().filter(|r| r.3 == 4).map(|r| (r.1, r.4)).collect();
            let level_at = |t: f64| fl.iter().filter(|f| f.1 == 0 && f.0 <= t).last().map_or(1.0, |f| f.2);
            let outs: Vec<(f64, f64)> = fl.iter().filter(|f| f.1 == 0).map(|f| (f.0, f.2)).collect();
            let truth = truth_alg(b, &h, [0.0; 2], 0.05);
            let mut errs: Vec<(f64, usize)> = (1..outs.len().min(truth.len())).map(|k| (((outs[k].0 - outs[k - 1].0) - (truth[k].0 - truth[k - 1].0)) * 1e6, k)).collect();
            errs.sort_by(|x, y| y.0.abs().total_cmp(&x.0.abs()));
            let total: f64 = errs.iter().map(|e| e.0).sum();
            let big: f64 = errs.iter().filter(|e| e.0.abs() > 1.0).map(|e| e.0).sum();
            eprintln!("DEFEDGE window off={off} stop {stop:.0e}: {} intervals, sum {total:+.3} us, of which intervals past 1 us {big:+.3} us; top {:?}", errs.len(), errs.iter().take(6).map(|&(e, k)| format!("edge {k} lvl {} {e:+.2} us at {:.6} (stop+{:.1} us; prev edge stop+{:.1} us)", outs[k].1, outs[k].0, (outs[k].0 % stop) * 1e6, (outs[k - 1].0 % stop) * 1e6)).collect::<Vec<_>>());
            let k_top = errs[0].1;
            for k in [k_top] {
                let te = outs[k].0;
                eprintln!("DEFEDGE window off={off} stop {stop:.0e}: edge {k} level {} at {te:.7} (truth {:.7}, {:+.3} us; interval {:+.3} us), threshold rail {:.6}", outs[k].1, truth[k].0, (te - truth[k].0) * 1e6, ((te - outs[k - 1].0) - (truth[k].0 - truth[k - 1].0)) * 1e6, rail_of(2.0 / 3.0 * 4.99));
                for r in rows.iter().filter(|r| r.0 > outs[k - 1].0 - 2e-5 && r.0 < te + 2e-5) {
                    eprintln!("DEFEDGE   t {:.8} vc {:.6}", r.0, r.1);
                }
            }
            let mut buckets: [Vec<f64>; 4] = Default::default();
            for w in rows.windows(2) {
                let ((t0, v0), (t1, v1)) = (w[0], w[1]);
                let hs = t1 - t0;
                if !(hs > 0.0) || level_at(t0 + 1e-12) != 1.0 || level_at(t1) != 1.0 {
                    continue;
                }
                let a = (-hs / tau).exp();
                let u = (v1 - v0 * a) / (1.0 - a);
                let d = u - rail_of(0.5 * (v0 + v1));
                let frac = ((t1 / stop) - (t1 / stop - 1e-9).floor()).clamp(0.0, 1.0);
                let k = ((frac * 4.0) as usize).min(3);
                buckets[k].push(d);
            }
            let s: Vec<String> = buckets
                .iter_mut()
                .map(|v| {
                    v.sort_by(f64::total_cmp);
                    let q = |f: f64| v.get(((v.len() as f64 - 1.0) * f) as usize).copied().unwrap_or(f64::NAN) * 1e3;
                    format!("median {:+.4} mV (q10 {:+.4}, q90 {:+.4}, n {})", q(0.5), q(0.1), q(0.9), v.len())
                })
                .collect();
            eprintln!("DEFEDGE window off={off} stop {stop:.0e}: implied rail - algebraic by quarter of the window: {s:?}");
        }
    }
}

/// EDGE3: every step the split-Ct fixture 78 board plans over 50 ms at 1 ms
/// stops, with the ratio of each replanned trial's `h` to the trial it
/// replaced (the same partition planned again from the same start): the
/// replan after the rail loop starts from the state the first plan did
/// (`SteppedSolver::restore_plan_start`), so it plans the step first planned
/// up to what the solved rail changes; started from the `h` grown for the
/// step after, it was one growth factor larger (EDGE_OFF bit 3 = 8 turns the
/// restore off). Returns (replans seen, worst ratio, mean ratio).
fn replan_ratios() -> (usize, f64, f64, usize) {
    use super::stepped::STEP_LOG;
    STEP_LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
    super::stepped::DEPENDENT_REPLANS.with(|c| c.set(0));
    flips_with(P78_SPLIT, 1e-3, 0.05, |_| {});
    eprintln!("EDGE3 dependent replans {}", super::stepped::DEPENDENT_REPLANS.with(|c| c.get()));
    let log = STEP_LOG.with(|l| l.borrow_mut().take().unwrap());
    let mut ratios = Vec::new();
    for w in log.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        if a.p_idx == b.p_idx && ((a.t_end - a.h) - (b.t_end - b.h)).abs() < 1e-15 && a.h > 1e-9 {
            ratios.push(b.h / a.h);
        }
    }
    let worst = ratios.iter().copied().fold(0.0, f64::max);
    let mean = ratios.iter().sum::<f64>() / ratios.len().max(1) as f64;
    (ratios.len(), worst, mean, ratios.iter().filter(|&&r| r > 1.25).count())
}

#[test]
fn edge3_a_replanned_trial_starts_from_the_h_the_first_plan_did() {
    use super::stepped::EDGE_OFF;
    EDGE_OFF.with(|c| c.set(8));
    let off = replan_ratios();
    EDGE_OFF.with(|c| c.set(0));
    let on = replan_ratios();
    eprintln!("EDGE3 replans {} / {} (restore off / on), worst h ratio {:.3} / {:.3}, mean {:.3} / {:.3}, over 1.25: {} / {}", off.0, on.0, off.1, on.1, off.2, on.2, off.3, on.3);
    assert!(on.0 >= 20, "the board replans ({} seen)", on.0);
    // EDGE4: a replan reads the rail as a line, so a crossing landing it
    // refines can land some ns from where the first plan, against the held
    // rail, put it: 2 of 4341 replans grow h past a quarter (steps of 1.0
    // and 23 ns, 2.64x and 1.26x). The restore is about the controller's
    // `h`, which the others keep (mean ratio 0.985).
    assert!(on.3 * 1000 <= on.0, "a replanned step is the step first planned, its h at most a quarter larger, but for a landing in a thousand: {on:?}");
    assert!(on.2 < 1.01 && off.3 * 10 > off.0, "and the restore is what does it: {off:?} against {on:?}");
}

/// EDGE3: a crossing's landing ends the approach the crossing-time cap held
/// the step to, so the step after it takes the controller's `h` and not the
/// cap read toward a threshold now behind it. The 555's timing partition on
/// fixture 78's split-Ct board lands its threshold crossings under a
/// binding cap; the first step planned after the landing is longer with the restore than without it (`EDGE_OFF` 16).
#[test]
fn edge3_the_step_after_a_landing_does_not_carry_the_stale_cap() {
    use super::stepped::{EDGE_LAND_RESTORES, EDGE_OFF, STEP_LOG};
    let run = |off: u8| {
        EDGE_OFF.with(|c| c.set(off));
        EDGE_LAND_RESTORES.with(|c| c.set(0));
        STEP_LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
        flips_with(P78_SPLIT, 1e-3, 0.05, |_| {});
        let log = STEP_LOG.with(|l| l.borrow_mut().take().unwrap());
        EDGE_OFF.with(|c| c.set(0));
        (EDGE_LAND_RESTORES.with(|c| c.get()), log.iter().map(|r| (r.p_idx, r.t_end, r.h)).collect::<Vec<_>>())
    };
    let (on, with) = run(0);
    let (off, without) = run(16);
    eprintln!("EDGE3 landing restores {on} / {off}; steps {} / {}", with.len(), without.len());
    assert!(on > 0 && off == 0, "the board lands a crossing under a binding cap ({on}), and the switch turns the restore off ({off})");
    // The runs agree to the first landing; after it the restored one steps longer.
    let k = with.iter().zip(&without).position(|(a, b)| a != b).expect("the restore changes a step");
    assert!(with[k].2 > without[k].2, "the step after the landing is longer with the restore: {:?} against {:?}", with[k], without[k]);
}

/// EDGE2: the ordered solver/555 log around the worst late flip on fixture 78
/// split (a stepped timing partition beside the cell hub), production default.
#[test]
#[ignore = "the EDGE2 late-flip trace: run by hand, --ignored --nocapture"]
fn edge2_late_flip_trace() {
    use super::stepped::E2LOG;
    let (prog, b) = (P78_SPLIT, &B78);
    let h = hand(prog);
    let stop = std::env::var("EDGE2_STOP").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1e-3);
    let t_end = 0.05;
    E2LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
    let fl = flips_with(prog, stop, t_end, |_| {});
    let log = E2LOG.with(|l| l.borrow_mut().take().unwrap());
    let outs: Vec<(f64, f64)> = fl.iter().filter(|f| f.1 == 0).map(|f| (f.0, f.2)).collect();
    let truth = truth_alg(b, &h, [0.0; 2], t_end);
    let mut errs: Vec<(f64, usize)> = (1..outs.len().min(truth.len())).map(|k| (((outs[k].0 - outs[k - 1].0) - (truth[k].0 - truth[k - 1].0)) * 1e6, k)).collect();
    errs.sort_by(|x, y| y.0.abs().total_cmp(&x.0.abs()));
    eprintln!("EDGE2 {} edges (truth {}), top {:?}", outs.len(), truth.len(), &errs[..errs.len().min(6)]);
    let k = errs[0].1;
    let (ta, tz) = (outs[k - 1].0, outs[k].0);
    eprintln!("EDGE2 edge {k} level {} at {tz:.9} (prev {ta:.9}), truth interval end {:.9}", outs[k].1, outs[k - 1].0 + truth[k].0 - truth[k - 1].0);
    // Every log line from the last REFINE/COMMIT before `tz - 60 us` to `tz + 1 us`.
    let time_of = |s: &str| -> Option<f64> {
        let w: Vec<&str> = s.split_whitespace().collect();
        let i = w.iter().position(|x| *x == "t" || *x == "t0" || *x == "now")?;
        w.get(i + 1)?.parse().ok()
    };
    let first = log.iter().position(|s| time_of(s).is_some_and(|t| t > tz - 6e-5)).unwrap_or(0);
    let last = log.iter().rposition(|s| time_of(s).is_some_and(|t| t <= tz + 1e-6)).unwrap_or(log.len() - 1);
    for s in &log[first.saturating_sub(4)..(last + 12).min(log.len())] {
        eprintln!("EDGE2   {s}");
    }
}

/// EDGE2 review item (a): every binding crossing-time cap over 0.2 s at 1 ms
/// stops on each 555 board, default and FORCE_STEPPED, and how many of them
/// were set by a node that moved no more than its own error estimate over
/// the step (noise on a settled node, not an approach).
#[test]
#[ignore = "the EDGE2 cap census: run by hand, --ignored --nocapture"]
fn edge2_cap_census() {
    use super::stepped::EDGE_CAPS;
    for (name, prog) in [("77", P77), ("78", P78), ("78 split", P78_SPLIT), ("78 psu", P78_PSU), ("78 split psu", P78_SPLIT_PSU), ("sap1-clock cell", SAPCLK_BAT), ("sap1-clock psu", SAPCLK_PSU)] {
        for force in [false, true] {
            EDGE_CAPS.with(|c| c.borrow_mut().clear());
            let _ = flips_with(prog, 1e-3, 0.2, |o| o.force_stepped = force);
            let caps = EDGE_CAPS.with(|c| c.borrow().clone());
            let settled: Vec<_> = caps.iter().filter(|c| c.3).collect();
            let cut = |v: &[&(f64, f64, f64, bool)]| v.iter().map(|c| c.2 / c.1).fold(f64::INFINITY, f64::min);
            let all: Vec<_> = caps.iter().collect();
            eprintln!("EDGE2 caps {name} force={force}: {} binding (min cap/next {:.3}), {} by a settled node (min cap/next {:.3}, h at them {:?})", caps.len(), cut(&all), settled.len(), cut(&settled), settled.iter().take(4).map(|c| format!("{:.2e}->{:.2e}", c.1, c.2)).collect::<Vec<_>>());
        }
    }
}

/// EDGE4: which rail each committed trial of the watching hub partition
/// integrated against (`PLAN`'s, the published rail when it was last
/// planned) beside the rail the loop solved at its end (`RAIL`), fixture 77
/// under FORCE_STEPPED: the lag `u_end - u_plan` per step, and what a
/// linear extrapolation from the previous committed step's slope leaves.
#[test]
#[ignore = "the EDGE4 rail-lag trace: run by hand, --ignored --nocapture"]
fn edge4_rail_lag_trace() {
    use super::stepped::E2LOG;
    let stop = std::env::var("EDGE4_STOP").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1e-3);
    let t_end = 0.012;
    E2LOG.with(|l| *l.borrow_mut() = Some(Vec::new()));
    let got = edges_with(P77, stop, t_end, |o| o.force_stepped = true);
    let log = E2LOG.with(|l| l.borrow_mut().take().unwrap());
    let num = |w: &[&str], k: &str| -> Option<f64> { w.iter().position(|x| *x == k).and_then(|i| w.get(i + 1)?.parse().ok()) };
    // The last plan per start instant (a replan supersedes), in order; the last rail per instant.
    let mut plans: Vec<(f64, f64, f64, f64)> = Vec::new();
    let mut rails: Vec<(f64, f64)> = Vec::new();
    for s in &log {
        let w: Vec<&str> = s.split_whitespace().collect();
        if w.first() == Some(&"PLAN") && s.contains("hub true watch true") {
            let (t0, h, u, sl) = (num(&w, "t0").unwrap(), num(&w, "h").unwrap(), num(&w, "rail").unwrap(), num(&w, "slope").unwrap());
            if plans.last().is_some_and(|p| p.0 == t0) {
                plans.pop();
            }
            plans.push((t0, h, u, sl));
        } else if w.first() == Some(&"RAIL") {
            let (now, u) = (num(&w, "now").unwrap(), w[w.len() - 1].parse::<f64>().unwrap());
            if rails.last().is_some_and(|r| r.0 == now) {
                rails.pop();
            }
            rails.push((now, u));
        }
    }
    let rail_at = |t: f64| rails.iter().rev().find(|r| (r.0 - t).abs() < 1e-12).map(|r| r.1);
    let edges: Vec<f64> = got.iter().map(|e| e.0).collect();
    let (mut n, mut lag, mut res, mut lag_hi, mut n_hi) = (0usize, 0.0f64, 0.0f64, 0.0f64, 0usize);
    let mut prev: Option<(f64, f64)> = None;
    for k in 0..plans.len().saturating_sub(1) {
        let (t0, h, u, sl) = plans[k];
        let t1 = t0 + h;
        if (plans[k + 1].0 - t1).abs() > 1e-12 {
            prev = None;
            continue;
        }
        let Some(u_end) = rail_at(plans[k + 1].0) else { prev = None; continue };
        let flip = edges.iter().any(|&e| e > t0 - 1e-12 && e < t1 + 1e-12);
        let ext = prev.map(|(tp, up)| u + (u - up) / (t0 - tp) * h);
        if t0 > 0.004 && t0 < 0.0115 && !flip {
            n += 1;
            lag += u_end - u;
            if let Some(e) = ext {
                res += (u_end - e).abs();
            }
            if (u_end - u) > 0.0 {
                lag_hi += u_end - u;
                n_hi += 1;
            }
            if n <= 40 || (n % 50 == 0) {
                eprintln!("EDGE4 t0 {t0:.9} h {h:.3e} u_plan {u:.9} u_end {u_end:.9} lag {:+.3e} V ({:+.3e} V/s) extrap-resid {:+.3e} line {:+.3e}", u_end - u, (u_end - u) / h, ext.map_or(f64::NAN, |e| u_end - e), if sl != 0.0 { u_end - (u + sl * h) } else { f64::NAN });
            }
        }
        prev = if flip { None } else { Some((t0, u)) };
    }
    eprintln!("EDGE4 {} edges; {n} steps: mean lag {:+.3e} V, rising steps {n_hi} mean {:+.3e} V; mean |extrapolation residual| {:.3e} V", edges.len(), lag / n as f64, lag_hi / n_hi.max(1) as f64, res / n as f64);
}

/// EDGE4: default mode under FORCE_STEPPED at 10 us stops against
/// [`truth_alg`] over 1 s, with and without the rail line (`EDGE_OFF` 32).
#[test]
#[ignore = "the EDGE4 fine-stop check: run by hand, --ignored --nocapture"]
fn edge4_fine_stop_vs_alg() {
    use super::stepped::EDGE_OFF;
    for (name, prog, b) in [("77", P77, &B77), ("78", P78, &B78)] {
        let h = hand(prog);
        let truth = truth_alg(b, &h, [0.0; 2], 1.0);
        for off in [32u8, 0] {
            EDGE_OFF.with(|c| c.set(off));
            drift(&format!("{name} default+stepped vs alg EDGE_OFF {off}"), prog, b, &h, &truth, true, 1e-5, 1.0);
        }
        EDGE_OFF.with(|c| c.set(0));
    }
}
