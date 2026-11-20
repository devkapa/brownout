//! RPFIX: a slow event's re-solve of a device on fixed nodes (a rated load
//! straight across the hub rail) hands its readers to the kernel's delta
//! cycle, as the same event's partition re-solve does in exact mode.

use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

fn pin(n: usize) -> PinRecord {
    PinRecord { node: n, flags: 0, open_hash: 0, dead_hash: 0 }
}
fn fz(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob }
}

fn exact(p: &Program) -> Program {
    let mut x = p.clone();
    x.options.no_rail_shortcut = true;
    x.options.no_solution_cache = true;
    x
}

/// COAL4's reduced board of cold 998/1 (from `coalv_tests`): a rated 1.6 ohm
/// load across a cell latches open at ~23 ms, the rail jumps from under the
/// 74LS00's `vcc_min` to over it, and gate 0 (A on a 1k/10k divider, B on
/// the rail) should then drive low.
fn latch_powers_a_chip_board() -> Program {
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

/// The cold-sweep generator board HUB=1 seed 998 template 1 (from `coalv_tests`).
fn board_998() -> Program {
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

/// The cold-sweep generator board HUB=1 seed 2684 template 3 (from `coalv2_tests`);
/// its switch (device 8) opens at 0.13234120748003386 s.
fn board_2684_3() -> Program {
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

type Edge = (f64, usize, f64);

/// Every digital edge of `p` up to `end` in host steps of `h`, with `edit`
/// called at each stop (it may edit the engine), and the slow records (the
/// latch log's code, device and `since`, then channel 0's and 2's).
fn run_with(p: &Program, h: f64, end: f64, mut edit: impl FnMut(&mut Engine, f64)) -> (Vec<Edge>, Vec<(u64, u64, f64)>) {
    let mut e = Engine::load(p).unwrap();
    e.trace_enable(true);
    let mut edges = Vec::new();
    let mut t = 0.0;
    while t < end - 1e-12 {
        t = (t + h).min(end);
        e.advance(t).unwrap();
        edit(&mut e, t);
        edges.extend(e.trace().chunks(3).map(|c| (c[0], c[1] as usize, c[2])));
        e.trace_clear();
    }
    (edges, records(&mut e))
}

fn records(e: &mut Engine) -> Vec<(u64, u64, f64)> {
    let mut recs = Vec::new();
    for ch in [0u32, 2, 3] {
        for c in e.records(ch).unwrap().chunks(7) {
            recs.push((ch as u64 * 1_000_000 + c[0] as u64, c[1] as u64, c[3]));
        }
    }
    recs
}

fn first(edges: &[Edge], slot: usize, v: f64, after: f64) -> Option<f64> {
    edges.iter().find(|e| e.1 == slot && e.2 == v && e.0 >= after).map(|e| e.0)
}

/// The verifier's C6 test (COALV2): exact mode reads the chip at 22.665 ms,
/// the 0.7.0 default build at 69.722 ms.
#[test]
fn a_latch_that_powers_a_chip_reads_it_at_once_default_build() {
    let p = latch_powers_a_chip_board();
    let first_low = |p: &Program| {
        let mut e = Engine::load(p).unwrap();
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
    let t_exact = first_low(&exact(&p)).expect("exact mode drives gate 0 low");
    let t_default = first_low(&p).expect("default build drives gate 0 low");
    eprintln!("gate 0 low: exact {t_exact:.6} s, default {t_default:.6} s");
    assert!((t_default - t_exact).abs() < 1e-3, "the default build reads the chip {:.1} ms after exact mode", (t_default - t_exact) * 1e3);
}

/// The reduced board's chip delay (its `DELAY` param).
const DELAY: f64 = 1.1586429750008599e-7;

/// Whether `low` is the edge of a read at `at`: at it, or one gate delay
/// after it. Exact mode's load settle never sees the chip powered, so its
/// first read commits at once; the default build's load settle reads it on
/// the hub's open-circuit guess (5.09 V, over `vcc_min`) and commits gate
/// 0 high, so its read at the power-on drives low after the gate's delay.
fn read_at(low: f64, at: f64, delay: f64) -> bool {
    low >= at && low <= at + delay * (1.0 + 1e-9)
}

/// The same at the latch itself: gate 0's edge is the read at the latch's
/// own instant, in both modes.
#[test]
fn the_chip_reads_at_the_latch_instant_in_both_modes() {
    let p = latch_powers_a_chip_board();
    for (name, q) in [("default", p.clone()), ("exact", exact(&p))] {
        let (edges, recs) = run_with(&q, 1e-3, 0.05, |_, _| {});
        let latch = recs.iter().find(|r| r.0 == 3_000_001).map(|r| r.2).expect("the load latches");
        let low = first(&edges, 0, 0.0, 0.0).expect("gate 0 drives low");
        eprintln!("{name}: latch {latch:.9} gate 0 low {low:.9}");
        assert!(read_at(low, latch, DELAY), "{name}: gate 0 drives low {:.3e} s after the latch", low - latch);
    }
}

/// Cold 998/1 (COAL4): the 74LS00 the latch of device 6 powers reads at that
/// instant, as in exact mode (0.0227 s), not at the next 20 mV scan (0.0377 s).
#[test]
fn seed_998_tpl_1_reads_the_chip_at_the_latch() {
    let p = board_998();
    let (ed, recs) = run_with(&p, 1e-3, 0.05, |_, _| {});
    let (ex, _) = run_with(&exact(&p), 1e-3, 0.05, |_, _| {});
    let latch = recs.iter().find(|r| r.0 == 3_000_006).map(|r| r.2).expect("device 6 latches");
    let (d, x) = (first(&ed, 0, 0.0, 0.0).unwrap(), first(&ex, 0, 0.0, 0.0).unwrap());
    eprintln!("998/1: latch {latch:.9}, gate 0 low default {d:.9} exact {x:.9}");
    assert!(read_at(d, latch, DELAY), "gate 0 drives low {:.3e} s after the latch", d - latch);
    assert!((d - x).abs() < 1e-4, "default {d} exact {x}");
}

/// Cold 2684/3 (COALV2): a fuse straight across the rail (device 7) blows
/// at 7.15 us and powers the 74LS00 (device 11). Exact mode reads it there
/// and records its `output_sag` (code 4) at 7.16 us; the 0.7.0 default
/// build read it only at 0.2321 s (the brief took that for exact mode's
/// answer: the verifier's own c2-2684.log has exact at 7.16 us), and the
/// verifier's prototype, whose readers waited for the next re-solve, at the
/// switch edit, 0.1323 s. Fixed, the default build records it within one
/// gate delay (84.8 ns) of exact mode.
#[test]
fn seed_2684_tpl_3_reads_the_chip_at_the_fuse() {
    let p = board_2684_3();
    let go = |q: &Program| {
        let mut e = Engine::load(q).unwrap();
        e.advance(0.13234120748003386).unwrap();
        e.set_params(&[(8, 0, 0.0)]).unwrap();
        e.advance(0.3).unwrap();
        records(&mut e)
    };
    let (d, x) = (go(&p), go(&exact(&p)));
    let sag = |r: &[(u64, u64, f64)]| r.iter().find(|r| r.0 == 3_000_011 && r.1 == 4).map(|r| r.2);
    eprintln!("2684/3: default {d:?}\nexact {x:?}");
    let (sd, sx) = (sag(&d).expect("default records the sag"), sag(&x).expect("exact records the sag"));
    let blown = d.iter().find(|r| r.0 == 3_000_007).map(|r| r.2).expect("the fuse blows");
    assert!(sd > blown && sd - blown < 2e-7, "default records the sag {:.3e} s after the fuse", sd - blown);
    assert!((sd - sx).abs() < 1e-7, "default {sd} exact {sx}");
}

// ---- the other slow events that re-stamp a device on the hub ------------------

/// The reduced board's cell and chip, with `load` straight across the rail.
fn chip_on_a_cell(load: Vec<Device>, cap_ah: f64) -> Program {
    let mut p = latch_powers_a_chip_board();
    p.devices[0].params[6] = cap_ah;
    p.devices.remove(6);
    p.devices.remove(1);
    p.devices.extend(load);
    p
}

fn two(kind: u32, ci: usize, a: usize, b: usize, params: Vec<f64>) -> Device {
    fz(kind, ci, &[a, b], params, vec![])
}

/// A PTC (1.63 ohm, `iHold` 1 A) straight across the cell trips, the rail
/// jumps over the chip's `vcc_min`: gate 0 drives low at the trip.
#[test]
fn a_ptc_trip_that_powers_a_chip_reads_it_at_once() {
    let p = chip_on_a_cell(vec![two(kinds::PTC_FUSE, 6, 1, 0, vec![1.6292657981753067, 1.0])], 1e-3);
    let (ed, recs) = run_with(&p, 1e-3, 0.3, |_, _| {});
    let (ex, rx) = run_with(&exact(&p), 1e-3, 0.3, |_, _| {});
    let trip = |r: &[(u64, u64, f64)]| r.iter().find(|r| r.0 / 1_000_000 == 2).map(|r| r.2).expect("the PTC trips");
    let (td, tx) = (trip(&recs), trip(&rx));
    let (d, x) = (first(&ed, 0, 0.0, 0.0).expect("default: gate 0 low"), first(&ex, 0, 0.0, 0.0).expect("exact: gate 0 low"));
    eprintln!("trip default {td:.9} exact {tx:.9}; gate 0 low default {d:.9} exact {x:.9}");
    assert!(read_at(d, td, DELAY), "default: gate 0 drives low {:.3e} s after the trip", d - td);
    assert!(read_at(x, tx, DELAY), "exact: gate 0 drives low {:.3e} s after the trip", x - tx);
}

/// `resetFailures` after the reduced board's latch: the rated load across the
/// rail returns, the rail drops under `vcc_min`, and the chip is unpowered at
/// the reset, as in exact mode. (A guard: an unpowered chip's slots read its
/// supply, not an evaluation, so 0.7.0 showed this too; the reset's re-solve
/// now hands its readers on, as the latch's does.)
#[test]
fn a_reset_that_unpowers_a_chip_reads_it_at_once() {
    let p = latch_powers_a_chip_board();
    let go = |q: &Program| {
        let mut e = Engine::load(q).unwrap();
        e.trace_enable(true);
        e.advance(0.03).unwrap();
        e.trace_clear();
        e.reset_failures().unwrap();
        let at = e.trace().chunks(3).map(|c| (c[0], c[1] as usize, c[2])).collect::<Vec<_>>();
        e.trace_clear();
        e.advance(0.031).unwrap();
        let later = e.trace().chunks(3).map(|c| (c[0], c[1] as usize, c[2])).collect::<Vec<_>>();
        (at, later)
    };
    let (d, x) = (go(&p), go(&exact(&p)));
    eprintln!("reset: default {d:?}\nexact {x:?}");
    assert!(!x.0.is_empty(), "exact mode reads the chip at the reset");
    let bits = |v: &[Edge]| v.iter().map(|e| (e.1, e.2.to_bits())).collect::<Vec<_>>();
    assert_eq!(bits(&d.0), bits(&x.0), "the edges at the reset");
    assert!(d.0.iter().all(|e| e.0 == 0.03), "at the reset's instant");
}

/// A live edit: a switch straight across the rail opens, the rail jumps over
/// `vcc_min`, and the chip is read at the edit, as in exact mode.
#[test]
fn a_switch_opened_across_the_rail_powers_a_chip_at_once() {
    let p = chip_on_a_cell(vec![two(kinds::SWITCH, 6, 1, 0, vec![1.0])], 1e-2);
    let go = |q: &Program| {
        let mut e = Engine::load(q).unwrap();
        e.trace_enable(true);
        e.advance(0.01).unwrap();
        e.trace_clear();
        e.set_params(&[(5, 0, 0.0)]).unwrap();
        let at = e.trace().chunks(3).map(|c| (c[0], c[1] as usize, c[2])).collect::<Vec<_>>();
        e.trace_clear();
        e.advance(0.011).unwrap();
        (at, e.trace().chunks(3).map(|c| (c[0], c[1] as usize, c[2])).collect::<Vec<_>>())
    };
    let (d, x) = (go(&p), go(&exact(&p)));
    eprintln!("open: default {d:?}\nexact {x:?}");
    assert!(x.0.iter().chain(&x.1).any(|e| e.1 == 0 && e.2 == 0.0), "exact mode drives gate 0 low");
    let low = |v: &(Vec<Edge>, Vec<Edge>)| v.0.iter().chain(&v.1).find(|e| e.1 == 0 && e.2 == 0.0).map(|e| e.0);
    let (ld, lx) = (low(&d).expect("default drives gate 0 low"), low(&x).unwrap());
    assert_eq!(lx, 0.01, "exact mode reads the chip at the edit");
    assert!(read_at(ld, 0.01, DELAY), "default: gate 0 drives low {:.3e} s after the edit", ld - 0.01);
}

// A known difference of default mode from exact mode, separate from RPFIX
// (RPFIX2, cold battery 241/0 and 590/2): the default load settle reads the
// chip on the hub's open-circuit guess (over `vcc_min`) and commits its
// outputs there, so the chip's first commit is spent at load. When a later
// event powers it, its outputs show that load-time level at once (gate 0
// HIGH) and the level read at the power-on lands one gate delay later.
// Exact mode's load never powers the chip: the read at the power-on is its
// first commit, at once, and gate 0 goes straight LOW. An expected failure
// while that difference stands; when it closes, `should_panic` fails the
// gate and this attribute comes off.
#[test]
#[should_panic(expected = "power-on shows the load's commit")]
fn a_chip_powered_after_load_shows_its_read_level_at_once_as_in_exact_mode() {
    let p = chip_on_a_cell(vec![two(kinds::SWITCH, 6, 1, 0, vec![1.0])], 1e-2);
    let go = |q: &Program| {
        let mut e = Engine::load(q).unwrap();
        e.trace_enable(true);
        e.advance(0.01).unwrap();
        e.trace_clear();
        e.set_params(&[(5, 0, 0.0)]).unwrap();
        e.advance(0.011).unwrap();
        e.trace().chunks(3).filter(|c| c[1] == 0.0).map(|c| (c[0], c[2])).collect::<Vec<_>>()
    };
    let (d, x) = (go(&p), go(&exact(&p)));
    eprintln!("gate 0 after the edit: default {d:?}\nexact {x:?}");
    assert_eq!(x, vec![(0.01, 0.0)], "exact mode drives gate 0 low at the edit, at once");
    assert_eq!(d, x, "power-on shows the load's commit: gate 0's edges after the edit");
}

// ---- rollback, snapshot and carried reload across the instant -----------------

/// The reduced board with a 74LS04 of no delay in place of the 74LS00, its 1Y
/// feeding its own 1A: the read at the latch's power-on closes a zero-delay
/// ring, so that instant livelocks (-7) and rolls back.
fn ring_on_the_latch() -> Program {
    let mut p = latch_powers_a_chip_board();
    let chip = &mut p.devices[2];
    chip.kind = kinds::IC_74LS04;
    chip.params[kinds::ic_param::DELAY] = 0.0;
    // 1A = 1Y (node 3), the other inputs grounded.
    for (k, n) in [3usize, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].iter().enumerate() {
        chip.pins[2 + k].node = *n;
    }
    p
}

fn state_bits(e: &mut Engine) -> (u64, Vec<u64>, Vec<u64>, Vec<u64>, Vec<u64>) {
    (
        e.time().to_bits(),
        e.voltages().iter().map(|v| v.to_bits()).collect(),
        e.currents().iter().map(|v| v.to_bits()).collect(),
        e.digital_slots().iter().map(|v| v.to_bits()).collect(),
        {
            let mut r: Vec<u64> = e.records(0).unwrap().iter().map(|v| v.to_bits()).collect();
            r.extend(e.records(2).unwrap().iter().map(|v| v.to_bits()));
            r
        },
    )
}

/// A failed advance in the latch's instant (the power-on read livelocks)
/// leaves the last completed instant exactly as an engine that stopped
/// there has it, owes nothing to the next attempt (that fails the same way,
/// at the same instant), and the latch is not in the log.
#[test]
fn a_failed_instant_at_the_latch_rolls_back_whole() {
    let p = ring_on_the_latch();
    let mut e = Engine::load(&p).unwrap();
    e.advance(0.02).unwrap();
    let snap = e.snapshot();
    let err = e.advance(0.03).unwrap_err();
    assert_eq!(err.code, crate::error::ERR_DELTA_LIVELOCK, "{}", err.message);
    let stopped = e.time();
    eprintln!("livelock at {stopped:.12}: {}", err.message);
    assert!(stopped > 0.0227 && stopped < 0.02273, "it fails at the latch, 22.72 ms (stopped at {stopped})");
    let mut twin = Engine::load(&p).unwrap();
    twin.restore(&snap);
    twin.advance(stopped).unwrap();
    let failed = state_bits(&mut e);
    assert_eq!(failed, state_bits(&mut twin), "as an engine that stopped at that instant");
    assert!(e.records(3).unwrap().chunks(7).all(|c| c[0] != 1.0), "the latch went back with its instant");
    let again = e.advance(0.03).unwrap_err();
    assert_eq!(again.code, err.code);
    assert_eq!(state_bits(&mut e), failed, "the second attempt fails at the same instant");
    let twin_err = twin.advance(0.03).unwrap_err();
    assert_eq!(twin_err.code, err.code);
    assert_eq!(state_bits(&mut twin), failed);
}

/// A snapshot taken before the latch and restored replays it bit for bit:
/// the latch, the power-on read and every edge and record after it.
#[test]
fn a_snapshot_before_the_latch_replays_it_bit_for_bit() {
    let p = latch_powers_a_chip_board();
    let mut e = Engine::load(&p).unwrap();
    e.trace_enable(true);
    e.advance(0.022).unwrap();
    let snap = e.snapshot();
    let tail = |e: &mut Engine| {
        e.trace_clear();
        let mut t = 0.022;
        let mut bits = Vec::new();
        while t < 0.05 - 1e-12 {
            t += 1e-3;
            e.advance(t).unwrap();
            bits.push(state_bits(e));
        }
        let edges: Vec<u64> = e.trace().iter().map(|v| v.to_bits()).collect();
        let log: Vec<u64> = e.records(3).unwrap().iter().map(|v| v.to_bits()).collect();
        (bits, edges, log)
    };
    let first = tail(&mut e);
    assert!(first.1.chunks(3).any(|c| f64::from_bits(c[1]) == 0.0 && f64::from_bits(c[2]) == 0.0), "gate 0 drives low after the latch");
    e.restore(&snap);
    assert_eq!(first, tail(&mut e), "restored before the latch");
}

/// A carried reload before the latch latches and reads the chip at the
/// latch, as the run that never reloaded does; one after it starts with the
/// load open and gate 0 low.
#[test]
fn a_carried_reload_on_either_side_of_the_latch_reads_the_chip() {
    let p = latch_powers_a_chip_board();
    let reload = |at: f64| {
        let mut e = Engine::load(&p).unwrap();
        e.advance(at).unwrap();
        let mut q = p.clone();
        q.carry = e.carry_take().unwrap().to_vec();
        q.options.start_time = at;
        let mut r = Engine::load(&q).unwrap();
        r.trace_enable(true);
        r
    };
    let (cont, crec) = run_with(&p, 1e-3, 0.05, |_, _| {});
    let latch = crec.iter().find(|r| r.0 == 3_000_001).unwrap().2;
    let low = first(&cont, 0, 0.0, 0.0).unwrap();
    let mut before = reload(0.022);
    let mut edges = Vec::new();
    let mut t = 0.022;
    while t < 0.05 - 1e-12 {
        t += 1e-3;
        before.advance(t).unwrap();
        edges.extend(before.trace().chunks(3).map(|c| (c[0], c[1] as usize, c[2])));
        before.trace_clear();
    }
    let rl = records(&mut before).into_iter().find(|r| r.0 == 3_000_001).expect("the reload latches").2;
    let rlow = first(&edges, 0, 0.0, 0.0).expect("the reload drives gate 0 low");
    eprintln!("continuous: latch {latch:.12} low {low:.12}; reloaded at 22 ms: latch {rl:.12} low {rlow:.12}");
    assert!(read_at(rlow, rl, DELAY), "the reload reads the chip {:.3e} s after its latch", rlow - rl);
    assert!((rl - latch).abs() < 1e-6, "the reload latches {:.3e} s off the run", rl - latch);
    let mut after = reload(0.03);
    after.advance(0.031).unwrap();
    let slots = after.digital_slots().to_vec();
    assert_eq!(slots[0], 0.0, "reloaded after the latch, gate 0 is low: {slots:?}");
}
