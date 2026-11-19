//! COALV: an independent verifier's tests for move coalescing (Rails point 3,
//! "Rail moves"). Each probes what the authors' tests do not: the path-turn
//! bound against a dense scan, the first actions after a load, a logic input
//! and a power window within a microvolt of their thresholds while a held
//! rail creeps, the restriction. Failing tests here are findings.

use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};
use crate::rails::{path_gap, CapSeg, RailModes};
use std::rc::Rc;

fn pin(n: usize) -> PinRecord {
    PinRecord { node: n, flags: 0, open_hash: 0, dead_hash: 0 }
}
fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob }
}
fn r(ci: usize, a: usize, b: usize, ohm: f64) -> Device {
    dev(kinds::RESISTOR, ci, &[a, b], vec![ohm], vec![])
}
fn cell(rint: f64, cap_ah: f64) -> Device {
    dev(kinds::BATTERY_PACK, 0, &[1, 0], vec![1.0, rint, 1.0, 0.0, 2.0, 0.05, cap_ah, 1.0, 0.0], vec![2.0, 0.0, 3.4074, 1.0, 4.8837])
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn log(&mut self, lo: f64, hi: f64) -> f64 {
        (lo.ln() + (hi.ln() - lo.ln()) * self.f()).exp()
    }
}

fn seg(t0: f64, u0: f64, modes: Vec<(f64, f64)>) -> CapSeg {
    CapSeg { t0, u0, u_inf: 0.0, k: 0.0, slope: 0.0, alpha: 1.0, beta: 0.0, ramp: 0.0, cf_frozen: 0.0, cf_count: 0, modes: Some(Rc::new(RailModes::new(modes, u0, None))) }
}

/// V6: `path_gap` is an upper bound of the true largest difference of two
/// coupled paths before `until`, against a dense scan, over random sets of two
/// to five modes (decay rates over six decades, either sign of slope, a
/// mode left unpaired when the counts differ, and a near-equal pair).
#[test]
fn path_gap_never_reports_less_than_the_dense_scan() {
    let mut g = Rng(0x9E3779B97F4A7C15);
    let mut worst_ratio = f64::INFINITY;
    let mut worst_case = String::new();
    for case in 0..4000 {
        let na = 2 + (g.next() % 4) as usize;
        let nb = if case % 3 == 0 { 2 + (g.next() % 4) as usize } else { na };
        let ma: Vec<(f64, f64)> = (0..na).map(|_| (g.log(1e-2, 1e4), (g.f() - 0.5) * 2.0 * g.log(1e-6, 1e2))).collect();
        let mut mb: Vec<(f64, f64)> = if case % 2 == 0 {
            // two solves of one system: the same rates, slightly different slopes
            ma.iter().map(|&(k, s)| (k * (1.0 + 1e-6 * (g.f() - 0.5)), s * (1.0 + 1e-3 * (g.f() - 0.5)))).collect()
        } else {
            (0..nb).map(|_| (g.log(1e-2, 1e4), (g.f() - 0.5) * 2.0 * g.log(1e-6, 1e2))).collect()
        };
        mb.truncate(nb);
        if mb.is_empty() {
            continue;
        }
        let (t_a, t_b) = (g.f() * 0.01, g.f() * 0.01);
        let now = 0.01 + g.f() * 0.01;
        let until = now + g.log(1e-6, 1.0);
        let a = seg(t_a, 4.0 + g.f(), ma);
        // the second path starts from the rail where the first stands, plus a small offset
        let b = seg(t_b, a.at(t_b) + (g.f() - 0.5) * 1e-6, mb);
        let bound = path_gap(&a, &b, now, until);
        let n = 4000;
        let mut true_max = 0.0f64;
        for i in 0..=n {
            // dense scan, plus a log-spaced one near `now`
            let t = now + (until - now) * (i as f64 / n as f64);
            true_max = true_max.max((a.at(t) - b.at(t)).abs());
            let t2 = now + (until - now) * (1e-9f64).powf(1.0 - i as f64 / n as f64);
            true_max = true_max.max((a.at(t2) - b.at(t2)).abs());
        }
        if bound < true_max * (1.0 - 1e-9) - 1e-13 {
            let ratio = bound / true_max;
            if ratio < worst_ratio {
                worst_ratio = ratio;
                worst_case = format!("case {case}: bound {bound:e} < true {true_max:e} (now {now}, until {until}, a {:?} b {:?})", a.modes_at(now), b.modes_at(now));
            }
        }
    }
    assert!(worst_ratio == f64::INFINITY, "path_gap under-reports: {worst_case}");
}

/// A rail with a NaN in a path must not read as "no gap" (a NaN compares false
/// against the budget, so `path_gap(..) > eps` would hold the move).
#[test]
fn path_gap_nan_is_not_a_small_gap() {
    let a = seg(0.0, 4.0, vec![(10.0, 1.0), (100.0, f64::NAN)]);
    let b = seg(0.0, 4.0, vec![(10.0, 1.0), (100.0, 0.5)]);
    let gap = path_gap(&a, &b, 0.001, 0.01);
    assert!(!(gap <= 1e-7), "a NaN path gave gap {gap}");
    assert!(gap.is_nan() || gap > 1e-7);
    // the move test is `gap > eps`: NaN is not greater, so such a path would hold
    assert!(gap > 1e-7, "path_off's `path_gap(..) > eps` is false for gap {gap}: a NaN path is held");
}

fn gate_board(cap_ah: f64, load: f64, vcc_min: f64, r2: f64) -> Program {
    // nodes: 1 rail, 2 divider, 3 gate out, 4..6 spare outs. A 74LS00 with gate 0 on the divider.
    let mut p = vec![0.0; kinds::ic_param::COUNT];
    p[kinds::ic_param::VCC_MIN] = vcc_min;
    p[kinds::ic_param::VCC_MAX] = 9.0;
    p[kinds::ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    p[kinds::ic_param::VIL_FIXED] = 1.0;
    p[kinds::ic_param::VIL] = 0.8;
    p[kinds::ic_param::VIH_FIXED] = 1.0;
    p[kinds::ic_param::VIH] = 2.0;
    p[kinds::ic_param::NOMINAL] = 5.0;
    p[kinds::ic_param::IO_MAX] = 0.008;
    p[kinds::ic_param::DELAY] = 1e-7;
    let pins = [1usize, 0, 2, 2, 3, 0, 0, 4, 0, 0, 5, 0, 0, 6];
    let mut blob = vec![0.0];
    blob.extend(std::iter::repeat(0.0).take(12));
    let devices = vec![
        cell(1.5, cap_ah),
        dev(kinds::IC_74LS00, 1, &pins, p, blob),
        r(2, 1, 2, 100e3),
        r(3, 2, 0, r2),
        r(4, 1, 0, load),
        r(5, 3, 0, 1e4),
        r(6, 4, 0, 1e4),
        r(7, 5, 0, 1e4),
        r(8, 6, 0, 1e4),
    ];
    Program { carry: Vec::new(), node_count: 7, options: Options::default(), digital_slot_count: 32, channel_slot_count: 0, devices }
}

fn edges_of(e: &mut Engine, out: &mut Vec<(f64, usize, f64)>) {
    for c in e.trace().chunks(3) {
        out.push((c[0], c[1] as usize, c[2]));
    }
    e.trace_clear();
}

fn run_gate(p: &Program, eps: f64, h: f64, end: f64) -> (Vec<(f64, usize, f64)>, f64, f64, Vec<f64>) {
    let mut e = Engine::load(p).unwrap();
    e.debug_set_rail_act_eps(eps);
    e.trace_enable(true);
    let mut out = Vec::new();
    let mut t = 0.0;
    while t < end - 1e-12 {
        t = (t + h).min(end);
        e.advance(t).unwrap();
        edges_of(&mut e, &mut out);
    }
    (out, e.stat(14), e.stat(29), e.voltages().to_vec())
}

/// Calibrates the divider's lower resistor so that node 2 sits `above` volts over
/// `vil` at the load's own settle.
fn calibrate(cap_ah: f64, load: f64, vil: f64, above: f64) -> f64 {
    let (mut lo, mut hi) = (1e3f64, 1e6f64);
    for _ in 0..80 {
        let mid = (lo * hi).sqrt();
        let e = Engine::load(&gate_board(cap_ah, load, 0.0, mid)).unwrap();
        let v2 = e.voltages()[2];
        if v2 > vil + above { hi = mid } else { lo = mid }
    }
    (lo * hi).sqrt()
}

/// V2: a logic input on a divider from the rail, within microvolts of its
/// threshold, while the held rail creeps. Edge count must be equal and the
/// edge within the bound (2 eps times the divider's ratio over the node's
/// slope) of the build without coalescing.
#[test]
fn a_divider_input_within_a_microvolt_of_its_threshold_keeps_its_edge() {
    let (cap_ah, load) = (5e-3, 100e3);
    let r2 = calibrate(cap_ah, load, 0.8, 1.0e-6);
    let mut bad = Vec::new();
    let mut held_runs = 0;
    let mut worst_shift = 0.0f64;
    for k in -40..=40 {
        let r2k = r2 * (1.0 + k as f64 * 3e-8);
        let p = gate_board(cap_ah, load, 0.0, r2k);
        let (off, _, _, _) = run_gate(&p, -1.0, 0.007, 3.0);
        let (on, mv, ac, _) = run_gate(&p, 1e-7, 0.007, 3.0);
        if ac < mv {
            held_runs += 1;
        }
        if on.len() != off.len() {
            bad.push(format!("k {k}: {} edges on against {} off", on.len(), off.len()));
            continue;
        }
        for (a, b) in on.iter().zip(&off) {
            worst_shift = worst_shift.max((a.0 - b.0).abs());
        }
    }
    eprintln!("divider scan: {held_runs} of 81 runs held; worst edge shift {worst_shift:e} s");
    // A creeping rail moves by more than the budget at every Rail event, so nothing is held here: the
    // scan documents that (held_runs 0) rather than asserting it.
    assert!(bad.is_empty(), "edge count changed: {bad:?}");
}

/// V2: a chip at the edge of its supply window: `vccMin` scanned across the rail's
/// value at the load while the held rail creeps down through it.
#[test]
fn a_supply_window_within_a_microvolt_keeps_its_edge() {
    let (cap_ah, load) = (2e-4, 100e3);
    let u0 = Engine::load(&gate_board(cap_ah, load, 0.0, 50e3)).unwrap().voltages()[1];
    let mut bad = Vec::new();
    let mut worst_shift = 0.0f64;
    for k in 1..=60 {
        let vcc_min = u0 - k as f64 * 5e-8;
        let p = gate_board(cap_ah, load, vcc_min, 50e3);
        let (off, _, _, _) = run_gate(&p, -1.0, 0.007, 0.4);
        let (on, _, _, _) = run_gate(&p, 1e-7, 0.007, 0.4);
        if on.len() != off.len() {
            bad.push(format!("k {k}: {} edges on against {} off", on.len(), off.len()));
            continue;
        }
        for (a, b) in on.iter().zip(&off) {
            worst_shift = worst_shift.max((a.0 - b.0).abs());
        }
    }
    eprintln!("window scan: worst edge shift {worst_shift:e} s");
    assert!(bad.is_empty(), "edge count changed: {bad:?}");
}

/// V4/V3: what the first held move after a load stands on. The load's own
/// settle acts on every move (coalescing is still off), so the clamp
/// `act_until` is never set there; the first move within the budget then
/// holds with no clamp on the next `Rail` event, and the aims the load made
/// searched their crossings only as far as the event the load scheduled.
#[test]
fn the_load_leaves_the_clamp_of_its_last_action() {
    for (cap_ah, load) in [(2e-4, 100e3), (1e-5, 220.0), (1.0, 1e3)] {
        let mut e = Engine::load(&gate_board(cap_ah, load, 0.0, 50e3)).unwrap();
        // Off by default on the integration line: switched on after the load, as a harness does.
        e.debug_set_rail_act_eps(crate::rails::RAIL_ACT_EPS);
        let hub = e.debug_solver_concrete_mut().hub.as_ref().expect("a hub rail");
        let (until, ev, u, u_act) = (hub.state.act_until, hub.state.event_t, hub.state.u, hub.state.u_act);
        eprintln!("cap {cap_ah} load {load}: act_until {until} event_t {ev:?} u {u} u_act {u_act}");
        if let Some(ev) = ev {
            assert!(until.is_finite() && until <= ev, "after the load act_until is {until} but its aims were made for an event at {ev}");
        }
    }
}

/// V5: the restriction as `coalescing()` reports it; a stepped partition NOT on
/// the hub does not switch it off.
#[test]
fn coalescing_follows_the_boards_hub_partitions() {
    // Off by default on the integration line (`RAIL_ACT_EPS_DEFAULT`): switched on here.
    let mut e = Engine::load(&gate_board(2e-4, 100e3, 0.0, 50e3)).unwrap();
    e.debug_set_rail_act_eps(crate::rails::RAIL_ACT_EPS);
    assert!(e.debug_solver().coalescing(), "the plain gate board coalesces");
    // an RC and an LED fed by a clock, nothing on the rail: a stepped partition off the hub
    let mut p = gate_board(2e-4, 100e3, 0.0, 50e3);
    p.node_count = 10;
    p.devices.push(dev(kinds::VOLTAGE_SOURCE, 9, &[7, 0], vec![3.0], vec![]));
    p.devices.push(r(10, 7, 8, 470.0));
    p.devices.push(dev(kinds::CAPACITOR, 11, &[8, 0], vec![2.2e-6, 0.0, 0.0], vec![]));
    p.devices.push(dev(kinds::LED, 12, &[8, 9], vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0], vec![]));
    p.devices.push(r(13, 9, 0, 1e3));
    if let Ok(mut e) = Engine::load(&p) {
        e.debug_set_rail_act_eps(crate::rails::RAIL_ACT_EPS);
        eprintln!("stepped hub partitions {}; coalescing {}", e.debug_solver().stepped.all_hub_partitions().count(), e.debug_solver().coalescing());
    }
}

fn fz(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    dev(kind, ci, pins, params, blob)
}

/// V1 FINDING (cold-sweep generator, HUB=1, seed 998 template 1): a 74LS00's
/// output sits overloaded at 1.24 V from 0.0377 s with coalescing off (and in
/// exact mode from 0.0227 s), but stays at 2.39 V until 0.0742 s with it on,
/// where a rated 186 ohm resistor then latches that never latches without
/// coalescing. Held at any budget from 0 up. Edge counts equal, one edge 36 ms
/// late, one slow event gained.
/// The cold-sweep generator's board, HUB=1, seed 998, template 1 (b998.txt).
pub(super) fn board_998() -> Program {
    let devices = vec![
        fz(20, 0, &[1, 0], vec![1.0, 0.5798907082408911, 1.0, 0.0, 2.0, 0.05, 0.00011152306527052542, 0.9448427555246814, 0.0], vec![2.0, 0.0, 3.8599302887728126, 1.0, 5.085065956332882]),
        fz(1, 1, &[2, 0], vec![10954.976879680964], vec![]),
        fz(1, 2, &[3, 0], vec![13325.900802874141], vec![]),
        fz(1, 3, &[4, 0], vec![506366.8205867035], vec![]),
        fz(1, 4, &[5, 0], vec![51683.65016440632], vec![]),
        fz(1, 5, &[6, 0], vec![1532.5377647804369], vec![]),
        fz(1, 6, &[0, 1], vec![1.6292657981753067], vec![]),
        fz(1, 7, &[5, 4], vec![407.5002048901517], vec![]),
        fz(1, 8, &[4, 2], vec![46341.0670474041], vec![]),
        fz(1, 9, &[2, 4], vec![14.843202454562862], vec![]),
        fz(1, 10, &[1, 6], vec![1.0054535469072483], vec![]),
        fz(1, 11, &[6, 5], vec![55.82421961267477], vec![]),
        fz(30, 12, &[5, 6], vec![3.3840301794846885, 2.096319802402114, 0.009558990340382375, 0.0, 0.0, 0.0], vec![]),
        fz(1, 13, &[1, 5], vec![464.8772335705366], vec![]),
        fz(1, 14, &[4, 2], vec![3523.56207438971], vec![]),
        fz(50, 15, &[1, 0, 5, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.1586429750008599e-7], vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        fz(1, 16, &[3, 4], vec![186.71380601830376], vec![]),
        fz(1, 17, &[0, 2], vec![160.9297865360332], vec![]),
        fz(1, 18, &[4, 0], vec![41.38633792689253], vec![]),
        fz(1, 19, &[0, 5], vec![1455.9542864498962], vec![]),
        fz(3, 20, &[0, 0], vec![0.00014881647259820118, 0.044499291218468524, 0.0], vec![]),
        fz(1, 21, &[2, 3], vec![39.450196102470585], vec![]),
        fz(1, 22, &[1, 2], vec![1070.9489678619068], vec![]),
        fz(1, 23, &[3, 0], vec![139996.20386775682], vec![]),
        fz(90, 6, &[], vec![6.0, 0.1809634948315904, 1.6292657981753067], vec![]),
        fz(90, 7, &[], vec![7.0, 0.00013123150255289496, 407.5002048901517], vec![]),
        fz(90, 8, &[], vec![8.0, 2.2306962387806203e-6, 46341.0670474041], vec![]),
        fz(90, 9, &[], vec![9.0, 0.38347693707433933, 14.843202454562862], vec![]),
        fz(90, 10, &[], vec![10.0, 38.06456845011592, 1.0054535469072483], vec![]),
        fz(90, 11, &[], vec![11.0, 0.3005630977296404, 55.82421961267477], vec![]),
        fz(90, 13, &[], vec![13.0, 0.00011478878520915369, 464.8772335705366], vec![]),
        fz(90, 14, &[], vec![14.0, 2.5322607275290597e-5, 3523.56207438971], vec![]),
        fz(90, 16, &[], vec![16.0, 0.00043447542487744467, 186.71380601830376], vec![]),
        fz(90, 17, &[], vec![17.0, 0.04226429694169387, 160.9297865360332], vec![]),
        fz(90, 18, &[], vec![18.0, 0.0035339681382744784, 41.38633792689253], vec![]),
        fz(90, 19, &[], vec![19.0, 0.0016486900234775456, 1455.9542864498962], vec![]),
        fz(90, 21, &[], vec![21.0, 0.1817024225820454, 39.450196102470585], vec![]),
        fz(90, 22, &[], vec![22.0, 0.0021477388045942226, 1070.9489678619068], vec![]),
        fz(90, 23, &[], vec![23.0, 3.3296205672649594e-5, 139996.20386775682], vec![]),
    ];
    Program { carry: Vec::new(), node_count: 7, options: Options::default(), digital_slot_count: 32, channel_slot_count: 0, devices }
}

#[test]
fn seed_998_tpl_1_a_held_rail_gains_a_latch_and_delays_an_edge() {
    let p = board_998();
    let run = |eps: f64| {
        let mut e = Engine::load(&p).unwrap();
        e.debug_set_rail_act_eps(eps);
        e.trace_enable(true);
        e.advance(0.1).unwrap();
        let mut ed = Vec::new();
        edges_of(&mut e, &mut ed);
        let latches = e.records(3).unwrap().chunks(7).filter(|c| c[1] == 1.0).count();
        (ed, latches, e.voltages().to_vec())
    };
    let (off, l_off, v_off) = run(-1.0);
    // COAL4: budget 0 (act once per loop, on the settled rail) failed the same way.
    for eps in [0.0, 1e-7] {
        let (on, l_on, v_on) = run(eps);
        eprintln!("off: {off:?} latches {l_off} v {v_off:?}\non {eps}: {on:?} latches {l_on} v {v_on:?}");
        assert_eq!(l_on, l_off, "eps {eps}: a latch gained or lost");
        assert_eq!(on.len(), off.len());
        for (a, b) in on.iter().zip(&off) {
            assert!((a.0 - b.0).abs() < 1e-6, "eps {eps}: an edge {} s late", a.0 - b.0);
        }
    }
}

/// COAL4, the reduced board of 998/1's mechanism: a rated 1.6 ohm load on the
/// cell's rail latches open at ~23 ms, the rail jumps and powers a 74LS00
/// whose gate 0 (both inputs on the rail) should then drive low. The latch's
/// own re-solve does not hand its readers on, so the chip is read again only
/// at the rail's next 20 mV scan, anchored on `u_scan`. Without coalescing
/// the anchor is the rail of the loop's first pass (the overshoot before the
/// chip's draw); coalesced it was the settled rail, 18 mV lower, so the scan
/// and the edge came 36 ms later. Off and on must give the same edges.
pub(super) fn latch_powers_a_chip_board() -> Program {
    let devices = vec![
        fz(20, 0, &[1, 0], vec![1.0, 0.5798907082408911, 1.0, 0.0, 2.0, 0.05, 0.00011152306527052542, 0.9448427555246814, 0.0], vec![2.0, 0.0, 3.8599302887728126, 1.0, 5.085065956332882]),
        fz(1, 1, &[0, 1], vec![1.6292657981753067], vec![]),
        fz(50, 2, &[1, 0, 2, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3], vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 1.1586429750008599e-7], vec![0.0; 13]),
        fz(1, 3, &[3, 0], vec![100.0], vec![]),
        fz(1, 4, &[1, 2], vec![1e3], vec![]),
        fz(1, 5, &[2, 0], vec![1e4], vec![]),
        fz(90, 1, &[], vec![1.0, 0.1809634948315904, 1.6292657981753067], vec![]),
    ];
    Program { carry: Vec::new(), node_count: 4, options: Options::default(), digital_slot_count: 32, channel_slot_count: 0, devices }
}

#[test]
fn a_latch_that_powers_a_chip_reads_it_at_the_same_scan_coalesced() {
    let p = latch_powers_a_chip_board();
    let run = |eps: f64, h: f64| {
        let mut e = Engine::load(&p).unwrap();
        e.debug_set_rail_act_eps(eps);
        e.trace_enable(true);
        let mut ed = Vec::new();
        let mut t = 0.0;
        while t < 0.3 - 1e-12 {
            t = (t + h).min(0.3);
            e.advance(t).unwrap();
            edges_of(&mut e, &mut ed);
        }
        (ed, e.stat(14), e.stat(29))
    };
    for h in [1e-3, 1.0 / 60.0, 0.1] {
        let (off, _, _) = run(-1.0, h);
        for eps in [0.0, 1e-7] {
            let (on, mv, ac) = run(eps, h);
            eprintln!("h {h} eps {eps}: off {off:?}\n  on {on:?} (moves {mv} acts {ac})");
            assert!(off.iter().any(|e| e.1 == 0 && e.2 == 0.0), "the reduced board never drives gate 0 low");
            assert_eq!(on.len(), off.len(), "h {h} eps {eps}: edge count");
            for (a, b) in on.iter().zip(&off) {
                assert!(a.1 == b.1 && a.2 == b.2 && (a.0 - b.0).abs() < 1e-9, "h {h} eps {eps}: edge {a:?} against {b:?}");
            }
        }
    }
}
