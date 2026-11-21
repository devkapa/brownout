//! COALV2: an independent verifier's tests for move coalescing after the scan
//! fix (cold 998 tpl 1). C2: a family of small battery boards, each with a
//! rated load that latches open inside the loop (the rail jumps and a chip
//! powers on), a 555 astable with an LED and the chip on the rail: every edge
//! and every slow-event record with coalescing at budgets 0 and 1e-7 against
//! off, at three host stops. C6: the defect that `republish_fixed` drops the
//! readers of a latch's re-solve, pinned against exact mode.

use super::coalv_tests::latch_powers_a_chip_board;
use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

fn pin(n: usize) -> PinRecord {
    PinRecord { node: n, flags: 0, open_hash: 0, dead_hash: 0 }
}
fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob }
}

pub(super) struct Rng(pub(super) u64);
impl Rng {
    pub(super) fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    pub(super) fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub(super) fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.f()
    }
    fn log(&mut self, lo: f64, hi: f64) -> f64 {
        (lo.ln() + (hi.ln() - lo.ln()) * self.f()).exp()
    }
    pub(super) fn chance(&mut self, p: f64) -> bool {
        self.f() < p
    }
}

/// Nodes: 1 rail, 2 divider, 3 chip out, 4..8 the 555 (4 DIS, 5 THR/TRIG, 6 OUT, 7 LED, 8 CV).
/// `variant` 0: one rated load latches; 1: two rated loads (a second latch inside
/// the first's loop or after it); 2: the rated load is the 555's own output load.
fn board(seed: u64) -> Program {
    board_with(seed, None)
}

/// `r2`: the divider's lower resistor, else drawn.
pub(super) fn board_with(seed: u64, r2: Option<f64>) -> Program {
    let mut g = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    for _ in 0..4 {
        g.next();
    }
    let e = g.range(4.4, 5.6);
    let rint = g.log(0.2, 1.2);
    let mut bp = vec![0.0; kinds::battery_param::COUNT];
    bp[kinds::battery_param::OCV_SCALE] = 1.0;
    bp[kinds::battery_param::R_BASE] = rint;
    bp[kinds::battery_param::TEMP_FACTOR] = 1.0;
    bp[kinds::battery_param::DEP_P] = 2.0;
    bp[kinds::battery_param::DEP_K] = 0.05;
    bp[kinds::battery_param::CAPACITY_AH] = g.log(5e-5, 2e-3);
    bp[kinds::battery_param::SOC0] = g.range(0.6, 1.0);
    let mut devs = vec![dev(kinds::BATTERY_PACK, 0, &[1, 0], bp, vec![2.0, 0.0, e * g.range(0.6, 0.9), 1.0, e])];
    let mut ci = 1usize;
    let mut rated: Vec<(usize, f64, f64)> = Vec::new();
    // the rated loads: sag the rail under the chip's vcc_min until they latch
    let n_rated = if g.chance(0.4) { 2 } else { 1 };
    for _ in 0..n_rated {
        let r = g.log(0.8, 8.0);
        devs.push(dev(kinds::RESISTOR, ci, &[1, 0], vec![r], vec![]));
        let p = e * e / r;
        rated.push((ci, p * g.log(0.01, 0.4), r));
        ci += 1;
    }
    // the chip: gate 0 on a divider node and the rail
    let mut p = vec![0.0; kinds::ic_param::COUNT];
    p[kinds::ic_param::VCC_MIN] = g.range(3.6, 4.75);
    p[kinds::ic_param::VCC_MAX] = 5.25 + 4.0;
    p[kinds::ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    p[kinds::ic_param::VIL_FIXED] = 1.0;
    p[kinds::ic_param::VIL] = 0.8;
    p[kinds::ic_param::VIH_FIXED] = 1.0;
    p[kinds::ic_param::VIH] = 2.0;
    p[kinds::ic_param::NOMINAL] = 5.0;
    p[kinds::ic_param::IO_MAX] = 0.008;
    p[kinds::ic_param::DELAY] = g.log(5e-8, 5e-6);
    let a_in = if r2.is_some() || g.chance(0.5) { 2 } else { 1 };
    devs.push(dev(kinds::IC_74LS00, ci, &[1, 0, a_in, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3], p, vec![0.0; 13]));
    ci += 1;
    devs.push(dev(kinds::RESISTOR, ci, &[3, 0], vec![g.log(60.0, 2e3)], vec![]));
    ci += 1;
    devs.push(dev(kinds::RESISTOR, ci, &[1, 2], vec![g.log(300.0, 5e3)], vec![]));
    ci += 1;
    let r2_drawn = g.log(1e3, 5e4);
    devs.push(dev(kinds::RESISTOR, ci, &[2, 0], vec![r2.unwrap_or(r2_drawn)], vec![]));
    ci += 1;
    // a 555 astable with an LED on the same rail
    let (r1, r2, cap) = (g.log(100.0, 2e4), g.log(500.0, 5e4), g.log(1e-7, 1e-5));
    let mut pp = vec![0.0; kinds::ic_param::COUNT];
    pp[kinds::ic_param::VCC_MIN] = 2.0;
    pp[kinds::ic_param::VCC_MAX] = 18.0;
    pp[kinds::ic_param::NOMINAL] = 5.0;
    let mut blob = vec![20.0];
    blob.extend(std::iter::repeat(0.0).take(6));
    devs.push(dev(kinds::NE555, ci, &[1, 0, 5, 6, 1, 8, 5, 4], pp, blob));
    ci += 1;
    devs.push(dev(kinds::RESISTOR, ci, &[1, 4], vec![r1], vec![]));
    ci += 1;
    devs.push(dev(kinds::RESISTOR, ci, &[4, 5], vec![r2], vec![]));
    ci += 1;
    devs.push(dev(kinds::CAPACITOR, ci, &[5, 0], vec![cap, 0.0, 0.0], vec![]));
    ci += 1;
    devs.push(dev(kinds::CAPACITOR, ci, &[8, 0], vec![1e-8, 0.0, 0.0], vec![]));
    ci += 1;
    let rl = g.log(100.0, 2000.0);
    devs.push(dev(kinds::RESISTOR, ci, &[6, 7], vec![rl], vec![]));
    ci += 1;
    devs.push(dev(kinds::LED, ci, &[7, 0], vec![g.range(1.6, 3.4), g.range(1.5, 2.5), g.range(0.004, 0.04), 0.0, 0.0, 0.0], vec![]));
    ci += 1;
    for (idx, pmax, r) in rated {
        devs.push(dev(90, idx, &[], vec![idx as f64, pmax, r], vec![]));
    }
    Program { carry: Vec::new(), node_count: 9, options: Options::default(), digital_slot_count: 32, channel_slot_count: 0, devices: devs }
}

fn same(a: f64, b: f64) -> bool {
    a == b || (a.is_nan() && b.is_nan())
}

type Edges = Vec<(f64, usize, f64)>;

fn run(p: &Program, eps: f64, h: f64, end: f64) -> (Edges, Vec<(u64, u64, f64)>, f64, f64) {
    let mut e = Engine::load(p).unwrap();
    e.debug_set_rail_act_eps(eps);
    e.trace_enable(true);
    let mut out = Vec::new();
    let mut t = 0.0;
    while t < end - 1e-12 {
        t = (t + h).min(end);
        if e.advance(t).is_err() {
            break;
        }
        for c in e.trace().chunks(3) {
            out.push((c[0], c[1] as usize, c[2]));
        }
        e.trace_clear();
    }
    let mut recs = Vec::new();
    for ch in 0..4u32 {
        if let Ok(x) = e.records(ch) {
            for c in x.chunks(7) {
                recs.push((c[0] as u64 + ch as u64 * 1_000_000, c[1] as u64, c[3]));
            }
        }
    }
    recs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    (out, recs, e.stat(14), e.stat(29))
}

/// C2: every edge and every slow event equal, off against budgets 0 and 1e-7.
#[test]
fn latch_power_on_family_edges_and_records_equal() {
    let n: u64 = std::env::var("COALV2_BOARDS").ok().and_then(|x| x.parse().ok()).unwrap_or(120);
    let (mut latched, mut powered, mut worst, mut held_boards) = (0, 0, 0.0f64, 0);
    for seed in 1..=n {
        let p = board(seed);
        for h in [1e-3, 1.0 / 60.0, 0.1] {
            let (off, roff, _, _) = run(&p, -1.0, h, 0.3);
            if h == 1e-3 {
                if roff.iter().any(|r| r.0 >= 3_000_000) {
                    latched += 1;
                }
                if off.iter().any(|e| e.1 < 4) {
                    powered += 1;
                }
            }
            for eps in [0.0, 1e-7] {
                let (on, ron, mv, ac) = run(&p, eps, h, 0.3);
                if h == 1e-3 && eps > 0.0 && ac < mv {
                    held_boards += 1;
                }
                assert_eq!(on.len(), off.len(), "seed {seed} h {h} eps {eps}: edge count {} vs {}", on.len(), off.len());
                for (a, b) in on.iter().zip(&off) {
                    // the two lists are in event order; a pair of simultaneous edges may swap
                    if a.1 != b.1 || !same(a.2, b.2) {
                        let mut sa = on.clone();
                        let mut sb = off.clone();
                        sa.sort_by(|x, y| x.1.cmp(&y.1).then(x.0.partial_cmp(&y.0).unwrap()));
                        sb.sort_by(|x, y| x.1.cmp(&y.1).then(x.0.partial_cmp(&y.0).unwrap()));
                        for (a, b) in sa.iter().zip(&sb) {
                            assert!(a.1 == b.1 && same(a.2, b.2), "seed {seed} h {h} eps {eps}: edge {a:?} against {b:?}");
                            worst = worst.max((a.0 - b.0).abs());
                        }
                        break;
                    }
                    worst = worst.max((a.0 - b.0).abs());
                }
                assert_eq!(ron.len(), roff.len(), "seed {seed} h {h} eps {eps}: slow records {ron:?} vs {roff:?}");
                for (a, b) in ron.iter().zip(&roff) {
                    assert!(a.0 == b.0 && a.1 == b.1 && (a.2 - b.2).abs() < 1e-6, "seed {seed} h {h} eps {eps}: record {a:?} against {b:?}");
                }
            }
        }
    }
    eprintln!("COALV2 family: {n} boards, {latched} latched, {powered} drove a chip edge, {held_boards} held at 1e-7, worst edge shift {worst:e} s");
    assert!(worst < 1e-7, "worst edge shift {worst:e} s");
    assert!(latched >= n as usize / 4, "the family barely latches ({latched} of {n})");
}

/// A latch that powers a chip on is read at once, as exact mode reads it. (COALV2's C6 pinned the default build
/// reading it 47 ms late as a `should_panic`: `republish_fixed` dropped the readers of the latch's re-solve; R141
/// kept them and COAL5 turned the pin into this statement.)
#[test]
fn a_latch_that_powers_a_chip_reads_it_at_once_default_build() {
    let p = latch_powers_a_chip_board();
    let first_low = |p: &Program, eps: f64| {
        let mut e = Engine::load(p).unwrap();
        e.debug_set_rail_act_eps(eps);
        e.trace_enable(true);
        let mut t = 0.0;
        let mut found = None;
        while t < 0.3 - 1e-12 && found.is_none() {
            t = (t + 1e-3f64).min(0.3);
            e.advance(t).unwrap();
            found = e.trace().chunks(3).find(|c| c[1] == 0.0 && c[2] == 0.0).map(|c| c[0]);
            e.trace_clear();
        }
        found
    };
    let mut exact = p.clone();
    exact.options.no_rail_shortcut = true;
    exact.options.no_solution_cache = true;
    let t_exact = first_low(&exact, -1.0).expect("exact mode drives gate 0 low");
    let t_default = first_low(&p, -1.0).expect("default build drives gate 0 low");
    eprintln!("gate 0 low: exact {t_exact:.6} s, default (coalescing off) {t_default:.6} s");
    assert!((t_default - t_exact).abs() < 1e-3, "the default build reads the chip {:.1} ms after exact mode", (t_default - t_exact) * 1e3);
}

/// C2, the adversarial form: the chip's input A on a divider from the rail,
/// calibrated so that its node sits within 20 mV of a threshold at the rail the
/// latch leaves (measured on the off build at 0.1 s), so that the 18 mV passes
/// of the latch's loop straddle it. Edges and records equal at budgets 0, 1e-7.
#[test]
#[ignore = "COALV2: worst edge shift 7e-7 s = 0.93 of tau_w0 at budget 0 and 1e-7 alike (seed 47); run with --ignored"]
fn latch_power_on_divider_at_threshold_edges_equal() {
    let n: u64 = std::env::var("COALV2_DIV_BOARDS").ok().and_then(|x| x.parse().ok()).unwrap_or(60);
    let (mut worst, mut tested, mut chip_edges, mut worst_frac) = (0.0f64, 0, 0, 0.0f64);
    for seed in 1..=n {
        let mut g = Rng(seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1);
        for _ in 0..3 {
            g.next();
        }
        let target = if g.chance(0.5) { 2.0 } else { 0.8 } + g.range(-0.02, 0.02);
        let v2_at = |r2: f64| {
            let mut e = Engine::load(&board_with(seed, Some(r2))).unwrap();
            e.debug_set_rail_act_eps(-1.0);
            if e.advance(0.1).is_err() {
                return f64::NAN;
            }
            e.voltages()[2]
        };
        let (mut lo, mut hi) = (1e2f64, 1e7f64);
        for _ in 0..40 {
            let mid = (lo * hi).sqrt();
            if v2_at(mid) > target { hi = mid } else { lo = mid }
        }
        let r2 = (lo * hi).sqrt();
        let p = board_with(seed, Some(r2));
        tested += 1;
        for h in [1e-3, 1.0 / 60.0, 0.1] {
            let (off, roff, _, _) = run(&p, -1.0, h, 0.3);
            chip_edges += off.iter().filter(|e| e.1 < 4).count();
            for eps in [0.0, 1e-7] {
                let (on, ron, _, _) = run(&p, eps, h, 0.3);
                assert_eq!(on.len(), off.len(), "seed {seed} r2 {r2} h {h} eps {eps}: edge count {} vs {}", on.len(), off.len());
                let (mut sa, mut sb) = (on.clone(), off.clone());
                sa.sort_by(|x, y| x.1.cmp(&y.1).then(x.0.partial_cmp(&y.0).unwrap()));
                sb.sort_by(|x, y| x.1.cmp(&y.1).then(x.0.partial_cmp(&y.0).unwrap()));
                for (a, b) in sa.iter().zip(&sb) {
                    assert!(a.1 == b.1 && same(a.2, b.2), "seed {seed} r2 {r2} h {h} eps {eps}: edge {a:?} against {b:?}");
                    if (a.0 - b.0).abs() > 1e-8 {
                        // the allowance: the node's slope in the off run around the edge
                        let mut frac = 0.0;
                        let slope = {
                            let mut e = Engine::load(&p).unwrap();
                            e.debug_set_rail_act_eps(-1.0);
                            e.advance(b.0 - 2e-6).unwrap();
                            let v0 = e.voltages().to_vec();
                            e.advance(b.0 - 1e-6).unwrap();
                            let v1 = e.voltages().to_vec();
                            let tau = (0..v0.len()).map(|i| ((v1[i] - v0[i]) / 1e-6).abs()).filter(|s| *s > 0.0).zip((0..v0.len()).filter(|&i| v1[i] != v0[i])).map(|(s, i)| (1e-4 + 1e-3 * v1[i].abs()) / s).fold(f64::INFINITY, f64::min);
                            frac = (a.0 - b.0).abs() / tau;
                            (0..v0.len()).map(|i| ((v1[i] - v0[i]) / 1e-6).abs()).fold(0.0f64, f64::max)
                        };
                        worst_frac = worst_frac.max(frac);
                        eprintln!("SHIFT seed {seed} h {h} eps {eps}: edge {} slot {} shift {:e} s, fastest node {slope:e} V/s, shift x slope {:e} V, fraction of tau_w0 {frac:.3e}", b.0, b.1, (a.0 - b.0).abs(), (a.0 - b.0).abs() * slope);
                    }
                    worst = worst.max((a.0 - b.0).abs());
                }
                assert_eq!(ron.len(), roff.len(), "seed {seed} h {h} eps {eps}: slow records {ron:?} vs {roff:?}");
                for (a, b) in ron.iter().zip(&roff) {
                    assert!(a.0 == b.0 && a.1 == b.1 && (a.2 - b.2).abs() < 1e-6, "seed {seed} h {h} eps {eps}: record {a:?} against {b:?}");
                }
            }
        }
    }
    eprintln!("COALV2 divider family: {tested} boards, {chip_edges} chip edges (off, all stops), worst edge shift {worst:e} s, worst fraction of tau_w0 {worst_frac:.3e}");
    assert!(worst < 1e-7, "worst edge shift {worst:e} s");
}

/// Diagnostic (ignored): where the budget-0 run of one divider-family board first parts from off.
#[test]
#[ignore]
fn diag_divider_board_first_parting() {
    let seed: u64 = std::env::var("COALV2_SEED").ok().and_then(|x| x.parse().ok()).unwrap_or(47);
    let mut g = Rng(seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1);
    for _ in 0..3 {
        g.next();
    }
    let target = if g.chance(0.5) { 2.0 } else { 0.8 } + g.range(-0.02, 0.02);
    let v2_at = |r2: f64| {
        let mut e = Engine::load(&board_with(seed, Some(r2))).unwrap();
        e.debug_set_rail_act_eps(-1.0);
        e.advance(0.1).unwrap();
        e.voltages()[2]
    };
    let (mut lo, mut hi) = (1e2f64, 1e7f64);
    for _ in 0..40 {
        let mid = (lo * hi).sqrt();
        if v2_at(mid) > target { hi = mid } else { lo = mid }
    }
    let p = board_with(seed, Some((lo * hi).sqrt()));
    let mk = |eps: f64| {
        let mut e = Engine::load(&p).unwrap();
        e.debug_set_rail_act_eps(eps);
        if std::env::var("COALV2_TRACE").is_ok() {
            e.trace_enable(true);
        }
        e
    };
    let (mut a, mut b) = (mk(-1.0), mk(0.0));
    let mut t = 0.0;
    let mut reported = 0;
    let (mut ea, mut eb): (Edges, Edges) = (Vec::new(), Vec::new());
    while t < 0.3 - 1e-12 {
        t += std::env::var("COALV2_H").ok().and_then(|x| x.parse::<f64>().ok()).unwrap_or(2e-4);
        a.advance(t).unwrap();
        b.advance(t).unwrap();
        for c in a.trace().chunks(3) {
            ea.push((c[0], c[1] as usize, c[2]));
        }
        for c in b.trace().chunks(3) {
            eb.push((c[0], c[1] as usize, c[2]));
        }
        a.trace_clear();
        b.trace_clear();
        let d = a.voltages().iter().zip(b.voltages()).enumerate().map(|(i, (x, y))| (i, (x - y).abs())).fold((0, 0.0f64), |m, c| if c.1 > m.1 { c } else { m });
        if d.1 > 1e-12 && reported < 12 {
            eprintln!("t {t:.4}: node {} differs {:e} V (rail off {:.9} on {:.9}); stats off moves {} acts {} on moves {} acts {}", d.0, d.1, a.voltages()[1], b.voltages()[1], a.stat(14), a.stat(29), b.stat(14), b.stat(29));
            reported += 1;
        }
    }
    eprintln!("edges off {} on {}", ea.len(), eb.len());
    for (x, y) in ea.iter().zip(&eb).filter(|(x, y)| (x.0 - y.0).abs() > 1e-9 || x.1 != y.1).take(6) {
        eprintln!("EDGE off {x:?} on {y:?}");
    }
}

#[test]
#[ignore]
fn diag_divider_board_runs() {
    let seed: u64 = std::env::var("COALV2_SEED").ok().and_then(|x| x.parse().ok()).unwrap_or(47);
    let h: f64 = std::env::var("COALV2_H").ok().and_then(|x| x.parse().ok()).unwrap_or(1.0 / 60.0);
    let mut g = Rng(seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1);
    for _ in 0..3 {
        g.next();
    }
    let target = if g.chance(0.5) { 2.0 } else { 0.8 } + g.range(-0.02, 0.02);
    let v2_at = |r2: f64| {
        let mut e = Engine::load(&board_with(seed, Some(r2))).unwrap();
        e.debug_set_rail_act_eps(-1.0);
        e.advance(0.1).unwrap();
        e.voltages()[2]
    };
    let (mut lo, mut hi) = (1e2f64, 1e7f64);
    for _ in 0..40 {
        let mid = (lo * hi).sqrt();
        if v2_at(mid) > target { hi = mid } else { lo = mid }
    }
    let p = board_with(seed, Some((lo * hi).sqrt()));
    let (off, _, mo, ao) = run(&p, -1.0, h, 0.3);
    let (on, _, mn, an) = run(&p, 0.0, h, 0.3);
    eprintln!("edges off {} (moves {mo} acts {ao}) on {} (moves {mn} acts {an})", off.len(), on.len());
    for (i, (x, y)) in off.iter().zip(&on).enumerate().filter(|(_, (x, y))| (x.0 - y.0).abs() > 1e-9 || x.1 != y.1).take(8) {
        eprintln!("EDGE {i} off {x:?} on {y:?}");
    }
}

fn fz(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    dev(kind, ci, pins, params, blob)
}

/// COALV2 FINDING (cold-sweep generator, HUB=1, seed 2684 template 3, written out by
/// `f5e DUMP`): a 74LS00 whose gate 0 output sits on the rail node. The default build
/// (off) and exact mode record its slow event (dev 11, code 4) at 0.2321 s; coalescing
/// on, at budget 0 as at 1e-7, at 0.0662 s, and slot 0's edge is 166 ms early.
// opts: Options { integration: 0, no_solution_cache: false, no_rail_shortcut: false, force_stepped: false, ambient_c: 25.0, floating_model: 0, start_time: 0.0 }
pub(super) fn board_2684_3() -> Program {
    let devices = vec![
        fz(20, 0, &[1, 0], vec![1.0, 0.497942502384776, 1.0, 0.0, 2.0, 0.05, 0.0014955488918403756, 0.48876553042906545, 0.0], vec![2.0, 0.0, 2.983147143432408, 1.0, 5.848661447429373]),
        fz(1, 1, &[2, 0], vec![628015.0789634953], vec![]),
        fz(1, 2, &[3, 0], vec![979028.6131786414], vec![]),
        fz(1, 3, &[4, 0], vec![882665.6728555678], vec![]),
        fz(1, 4, &[4, 0], vec![188.72102207624573], vec![]),
        fz(30, 5, &[2, 0], vec![3.252701307121887, 1.5063188730233892, 0.00933612402680578, 0.0, 0.0, 0.0], vec![]),
        fz(1, 6, &[1, 2], vec![11.609263899737615], vec![]),
        fz(10, 7, &[1, 0], vec![0.05669379219352309, 0.038571648465563935], vec![]),
        fz(4, 8, &[4, 0], vec![1.0], vec![]),
        fz(1, 9, &[0, 0], vec![29.454782517821346], vec![]),
        fz(1, 10, &[4, 0], vec![4176.963232877594], vec![]),
        fz(50, 11, &[1, 0, 1, 2, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 8.484160445017662e-8], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 12, &[1, 4], vec![136.95413549586928], vec![]),
        fz(1, 13, &[1, 1], vec![633.5213064195519], vec![]),
        fz(1, 14, &[2, 3], vec![171.74840449172285], vec![]),
        fz(30, 15, &[2, 1], vec![2.4919398948971256, 1.9033387615752246, 0.03153572921074482, 0.0, 0.0, 0.0], vec![]),
        fz(1, 16, &[1, 2], vec![806.8632628457473], vec![]),
        fz(1, 17, &[4, 1], vec![16891.910159337127], vec![]),
        fz(1, 18, &[4, 0], vec![5.433270727383342], vec![]),
        fz(90, 4, &[], vec![4.0, 0.06717343784266248, 188.72102207624573], vec![]),
        fz(90, 6, &[], vec![6.0, 3.3875286861784963, 11.609263899737615], vec![]),
        fz(90, 9, &[], vec![9.0, 0.010147581819131384, 29.454782517821346], vec![]),
        fz(90, 10, &[], vec![10.0, 0.00334224024142086, 4176.963232877594], vec![]),
        fz(90, 12, &[], vec![12.0, 0.2965899468902299, 136.95413549586928], vec![]),
        fz(90, 13, &[], vec![13.0, 0.01989436602594795, 633.5213064195519], vec![]),
        fz(90, 16, &[], vec![16.0, 0.009643777676670344, 806.8632628457473], vec![]),
        fz(90, 18, &[], vec![18.0, 0.08079817255409576, 5.433270727383342], vec![]),
    ];
    Program { carry: Vec::new(), node_count: 5, options: Options::default(), digital_slot_count: 32, channel_slot_count: 0, devices }
}

fn slow_records(p: &Program, eps: f64, end: f64) -> (Vec<(u64, u64, f64)>, Vec<(f64, usize, f64)>) {
    let mut e = Engine::load(p).unwrap();
    e.debug_set_rail_act_eps(eps);
    e.trace_enable(true);
    // the generator's switch (device 8) opens at 0.13234120748003386 s
    let mut t = 0.0;
    for (tt, v) in [(0.13234120748003386, 0.0), (end, 0.0)] {
        e.advance(tt.min(end)).unwrap();
        t = tt;
        if tt < end {
            e.set_params(&[(8, 0, v)]).unwrap();
        }
    }
    let _ = t;
    let edges = e.trace().chunks(3).map(|c| (c[0], c[1] as usize, c[2])).collect();
    let mut recs = Vec::new();
    for ch in 0..4u32 {
        for c in e.records(ch).unwrap().chunks(7) {
            recs.push((c[0] as u64 + ch as u64 * 1_000_000, c[1] as u64, c[3]));
        }
    }
    (recs, edges)
}

#[test]
#[ignore = "COALV2 finding: coalescing latches dev 11 166 ms early on cold 2684 tpl 3 (budget 0 too)"]
fn seed_2684_tpl_3_a_coalesced_loop_latches_a_chip_early() {
    let p = board_2684_3();
    let (r_off, e_off) = slow_records(&p, -1.0, 0.3);
    let mut exact = p.clone();
    exact.options.no_rail_shortcut = true;
    exact.options.no_solution_cache = true;
    let (r_ex, _) = slow_records(&exact, -1.0, 0.3);
    eprintln!("off {r_off:?}\nexact {r_ex:?}\nedges off {e_off:?}");
    for eps in [0.0, 1e-7] {
        let (r_on, e_on) = slow_records(&p, eps, 0.3);
        eprintln!("eps {eps}: {r_on:?}\nedges {e_on:?}");
        assert_eq!(r_on.len(), r_off.len(), "eps {eps}: slow records");
        for (a, b) in r_on.iter().zip(&r_off) {
            assert!(a.0 == b.0 && a.1 == b.1 && (a.2 - b.2).abs() < 1e-6, "eps {eps}: record {a:?} against {b:?}");
        }
    }
}
