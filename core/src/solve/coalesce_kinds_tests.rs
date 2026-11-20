//! Move coalescing and the eight kinds of the integration line (MERGEC, 2
//! (d)): each kind on a 9 V cell's hub beside a 555 astable, in the
//! arrangement its own fixtures use, run with coalescing off and at the
//! default budget (`RAIL_ACT_EPS`) in one build. Where the hub coalesces,
//! every logic edge is kept with its value and its time within the budget's
//! allowance, and every reading within the budget's bound (core/README.md,
//! Rails point 3, "Move coalescing", the hand bound); where it does not (a
//! moving `signal_gen` read by a hub partition, which makes that partition
//! stepped), the two runs are bit for bit alike.

use crate::engine::Engine;
use crate::kinds;
use crate::kinds::signal_gen_param as sg;
use crate::program::{Device, Options, PinRecord, Program};
use crate::rails::RAIL_ACT_EPS;

fn pin(n: usize) -> PinRecord {
    PinRecord { node: n, flags: 0, open_hash: 0, dead_hash: 0 }
}
fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob }
}
fn r(ci: usize, a: usize, b: usize, ohm: f64) -> Device {
    dev(kinds::RESISTOR, ci, &[a, b], vec![ohm], vec![])
}
fn c(ci: usize, a: usize, b: usize, f: f64) -> Device {
    dev(kinds::CAPACITOR, ci, &[a, b], vec![f, 0.0, 0.0], vec![])
}
fn red_led(ci: usize, a: usize, k: usize) -> Device {
    dev(kinds::LED, ci, &[a, k], vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0], vec![])
}
fn generator(ci: usize, pos: usize, neg: usize, shape: u32, amplitude: f64, offset: f64, frequency: f64, r_source: f64) -> Device {
    let mut params = vec![0.0; sg::COUNT];
    params[sg::WAVEFORM] = shape as f64;
    params[sg::AMPLITUDE] = amplitude;
    params[sg::OFFSET] = offset;
    params[sg::FREQUENCY] = frequency;
    params[sg::DUTY] = 0.5;
    params[sg::TR] = 1e-6;
    params[sg::TF] = 1e-6;
    params[sg::PW] = 5e-4;
    params[sg::SEED] = 1.0;
    params[sg::ENABLED] = 1.0;
    params[sg::R_SOURCE] = r_source;
    dev(kinds::SIGNAL_GEN, ci, &[pos, neg], params, vec![0.0])
}

const SLOT: f64 = 8.0;
/// Where a live write lands, in both runs alike.
const WRITE_AT: f64 = 0.015;
const H: f64 = 1e-4;
const END: f64 = 0.03;

/// A 9 V cell (6.0 to 9.6 V over its charge, 1.7 ohm, 50 uAh, so it
/// drains about 0.6 V/s at 30 mA) on node 1, and an NE555 astable on it
/// (RA 2438 on 1-2, RB 1014 on 2-3, 68 nF on 3, ctrl 10 nF on 6) whose
/// output (4) lights a red LED through 330 ohm (5), with 1 kohm across the
/// rail. Device indices 0 to 8; nodes 0 to 6; the kind adds from node 7.
fn base(ra: bool, rb: bool) -> Vec<Device> {
    let mut d = vec![
        dev(kinds::BATTERY_PACK, 0, &[1, 0], vec![1.0, 1.7, 1.0, 0.0, 2.0, 0.05, 50e-6, 1.0, 0.0], vec![2.0, 0.0, 6.0, 1.0, 9.6]),
        dev(kinds::NE555, 1, &[1, 0, 3, 4, 1, 6, 3, 2], vec![4.5, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![SLOT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        c(4, 3, 0, 6.8e-8),
        c(5, 6, 0, 1e-8),
        r(6, 4, 5, 330.0),
        red_led(7, 5, 0),
        r(8, 1, 0, 1000.0),
    ];
    if ra {
        d.push(r(2, 1, 2, 2438.0));
    }
    if rb {
        d.push(r(3, 2, 3, 1014.0));
    }
    d
}

fn program(node_count: usize, devices: Vec<Device>) -> Program {
    Program { carry: Vec::new(), node_count, options: Options::default(), digital_slot_count: 64, channel_slot_count: 4, devices }
}

/// One board: its name, the program and the live write made at `WRITE_AT`
/// (none for most).
struct Board {
    name: &'static str,
    program: Program,
    write: Vec<(u32, u32, f64)>,
}

fn boards() -> Vec<Board> {
    let mut out = Vec::new();
    // potentiometer: the 555's RA (cw on the rail, wiper on discharge, a
    // 1 Mohm ccw segment to ground), its knob turned mid-run.
    {
        let mut d = base(false, true);
        let k = d.len() as u32;
        d.push(dev(kinds::POTENTIOMETER, 10, &[1, 2, 0], vec![2438.0, 1e6, 0.0], vec![]));
        out.push(Board { name: "potentiometer (RA, knob write)", program: program(7, d), write: vec![(k, 0, 3200.0), (k, 1, 1e6 - 762.0)] });
    }
    // trimmer: a divider from the rail setting the 555's control pin.
    {
        let mut d = base(true, true);
        d.push(dev(kinds::TRIMMER, 10, &[1, 6, 0], vec![20_000.0, 40_000.0, 0.0], vec![]));
        out.push(Board { name: "trimmer (ctrl divider)", program: program(7, d), write: vec![] });
    }
    // resistor_array: channel 1 from the rail into an LED, channel 2 the
    // 555's RB, channel 3 across the rail (a direct device), channel 4 on
    // ground.
    {
        let mut d = base(true, false);
        d.push(dev(kinds::RESISTOR_ARRAY, 10, &[1, 7, 2, 3, 1, 0, 0, 0], vec![1014.0, 0.0], vec![0.0]));
        d.push(red_led(11, 7, 0));
        out.push(Board { name: "resistor_array (RB, rail LED, direct)", program: program(8, d), write: vec![] });
    }
    // ldr: the 555's RB, the lux moved mid-run.
    {
        let mut d = base(true, false);
        let k = d.len() as u32;
        d.push(dev(kinds::LDR, 10, &[2, 3], vec![1014.0], vec![]));
        out.push(Board { name: "ldr (RB, lux write)", program: program(7, d), write: vec![(k, 0, 1500.0)] });
    }
    // ferrite_bead: a rated 3.5 mohm resistor record (as the compiler emits
    // it) from the rail into an LED load through 330 ohm.
    {
        let mut d = base(true, true);
        let k = d.len() as f64;
        d.push(r(10, 1, 7, 0.0035));
        d.push(r(11, 7, 8, 330.0));
        d.push(red_led(12, 8, 0));
        d.push(dev(kinds::RATING, 0, &[], vec![k, 0.25, 0.0035], vec![]));
        out.push(Board { name: "ferrite_bead (rated, rail LED)", program: program(9, d), write: vec![] });
    }
    // push_dpdt: pole 1 switches the rail between two loads, pole 2 the
    // 555's output between two; pressed mid-run.
    {
        let mut d = base(true, true);
        let k = d.len() as u32;
        d.push(dev(kinds::PUSH_DPDT, 10, &[7, 1, 8, 9, 4, 10], vec![0.0], vec![]));
        d.push(r(11, 7, 0, 470.0));
        d.push(r(12, 8, 0, 1000.0));
        d.push(r(13, 9, 0, 1000.0));
        d.push(r(14, 10, 0, 2000.0));
        out.push(Board { name: "push_dpdt (rail and output poles, press)", program: program(11, d), write: vec![(k, 0, 1.0)] });
    }
    // current_source: 5 mA straight out of the rail (a direct contact with
    // an injection) and 1 mA from the rail into an RC behind a 100 kohm
    // shunt, the first moved mid-run.
    {
        let mut d = base(true, true);
        let k = d.len() as u32;
        d.push(dev(kinds::CURRENT_SOURCE, 10, &[0, 1], vec![5e-3, 0.0], vec![]));
        d.push(dev(kinds::CURRENT_SOURCE, 11, &[7, 1], vec![1e-3, 100_000.0], vec![]));
        d.push(r(12, 7, 0, 4700.0));
        d.push(c(13, 7, 0, 1e-6));
        out.push(Board { name: "current_source (direct injection, RC, write)", program: program(8, d), write: vec![(k, 0, 8e-3)] });
    }
    // signal_gen, ideal 0/5 V square at 1 kHz on node 7, 2.2 kohm from the
    // rail to it (a direct device whose draw steps at the generator's edges).
    {
        let mut d = base(true, true);
        d.push(generator(10, 7, 0, sg::SHAPE_SQUARE, 2.5, 2.5, 1000.0, 0.0));
        d.push(r(11, 1, 7, 2200.0));
        out.push(Board { name: "signal_gen square ideal (direct)", program: program(8, d), write: vec![] });
    }
    // signal_gen, Norton 0/5 V square behind 1 kohm into node 7, 2.2 kohm
    // from the rail and 10 nF to ground there (a hub partition the drive
    // re-solves at each edge).
    {
        let mut d = base(true, true);
        d.push(generator(10, 7, 0, sg::SHAPE_SQUARE, 2.5, 2.5, 1000.0, 1000.0));
        d.push(r(11, 1, 7, 2200.0));
        d.push(c(12, 7, 0, 1e-8));
        out.push(Board { name: "signal_gen square Norton (hub partition)", program: program(8, d), write: vec![] });
    }
    // signal_gen, ideal sine 4.5 +- 2 V at 500 Hz on node 7, 2.2 kohm from
    // the rail (a direct device to a sine: held between events, R99 (3)).
    {
        let mut d = base(true, true);
        d.push(generator(10, 7, 0, sg::SHAPE_SINE, 2.0, 4.5, 500.0, 0.0));
        d.push(r(11, 1, 7, 2200.0));
        out.push(Board { name: "signal_gen sine ideal (direct)", program: program(8, d), write: vec![] });
    }
    // signal_gen, Norton triangle behind 1 kohm into node 7 with 2.2 kohm
    // from the rail and 10 nF to ground: a hub partition reading a moving
    // node, so it is stepped and the hub does not coalesce.
    {
        let mut d = base(true, true);
        d.push(generator(10, 7, 0, sg::SHAPE_TRIANGLE, 2.0, 4.5, 500.0, 1000.0));
        d.push(r(11, 1, 7, 2200.0));
        d.push(c(12, 7, 0, 1e-8));
        out.push(Board { name: "signal_gen triangle Norton (stepped hub partition)", program: program(8, d), write: vec![] });
    }
    out
}

struct Run {
    coalescing: bool,
    moves: f64,
    acts: f64,
    edges: Vec<(f64, f64)>,
    v: Vec<Vec<f64>>,
    i: Vec<Vec<f64>>,
}

fn run(b: &Board, eps: f64) -> Run {
    let mut e = Engine::load(&b.program).unwrap();
    e.debug_set_rail_act_eps(eps);
    e.trace_enable(true);
    let coalescing = e.debug_solver_concrete_mut().coalescing();
    let (mut edges, mut v, mut i) = (Vec::new(), Vec::new(), Vec::new());
    let n = (END / H).round() as usize;
    let mut written = b.write.is_empty();
    for k in 1..=n {
        let t = k as f64 * H;
        if !written && t > WRITE_AT {
            e.set_params(&b.write).unwrap();
            written = true;
        }
        e.advance(t).unwrap();
        edges.extend(e.trace().chunks(3).filter(|c| c[1] == SLOT).map(|c| (c[0], c[2])));
        e.trace_clear();
        v.push(e.voltages().to_vec());
        i.push(e.currents().to_vec());
    }
    Run { coalescing, moves: e.stat(14), acts: e.stat(29), edges, v, i }
}

#[test]
fn each_new_kind_on_a_battery_hub_keeps_its_edges_and_readings_within_the_budget() {
    let mut report = String::new();
    for b in boards() {
        let off = run(&b, -1.0);
        let on = run(&b, RAIL_ACT_EPS);
        assert!(!off.coalescing);
        assert_eq!(off.acts, off.moves, "{}: off, every move acts", b.name);
        assert!(off.moves > 50.0, "{}: {} moves, the board must move its rail", b.name, off.moves);
        assert!(off.edges.len() > 100, "{}: {} edges", b.name, off.edges.len());
        assert_eq!(off.edges.len(), on.edges.len(), "{}: an edge lost or gained", b.name);
        let mut shift = 0.0f64;
        for (x, y) in off.edges.iter().zip(&on.edges) {
            assert_eq!(x.1.to_bits(), y.1.to_bits(), "{}: an edge's value", b.name);
            shift = shift.max((x.0 - y.0).abs());
        }
        let (mut dv, mut dv_w0, mut di, mut di_w0) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        for (a, z) in off.v.iter().zip(&on.v) {
            for (x, y) in a.iter().zip(z) {
                dv = dv.max((x - y).abs());
                dv_w0 = dv_w0.max((x - y).abs() / (100e-6 + 1e-3 * x.abs()));
            }
        }
        for (a, z) in off.i.iter().zip(&on.i) {
            for (x, y) in a.iter().zip(z) {
                di = di.max((x - y).abs());
                di_w0 = di_w0.max((x - y).abs() / (1e-9 + 1e-3 * x.abs()));
            }
        }
        report += &format!(
            "{}: coalescing {}, moves {} -> {}, acts {} -> {}, edges {}, worst edge shift {:.3e} s, V {:.3e} V ({:.2e} W0), I {:.3e} A ({:.2e} W0)\n",
            b.name, on.coalescing, off.moves, on.moves, off.acts, on.acts, on.edges.len(), shift, dv, dv_w0, di, di_w0
        );
        if !on.coalescing {
            assert_eq!(on.acts, on.moves, "{}: a hub that does not coalesce acts on every move", b.name);
            assert_eq!((shift, dv, di), (0.0, 0.0, 0.0), "{}: not coalescing, the runs must be bit for bit alike", b.name);
            continue;
        }
        assert!(on.acts < on.moves, "{}: coalescing held no move", b.name);
        // The hand bound (README, Rails point 3): a published hub node is off
        // by at most `e`, a closed form's drive by `2 e`, so no node by more
        // than `2 e`; a device current by `g e`, far inside 1e-3 of W0 on
        // these loads; a 555 crossing by `2 e` over the timing node's slope
        // at 2/3 of the rail, `(u / 3) / ((RA + RB) C)`, at least 1e4 V/s
        // here (RA 3200 after the knob, 6 V): 2e-11 s. Measured (MERGEC): V
        // 9.1e-8 V, I 1.4e-5 W0, edges 1.3e-15 s.
        let e = RAIL_ACT_EPS;
        assert!(dv <= 2.0 * e, "{}: a reading {dv:e} V off, over 2 eps", b.name);
        assert!(di_w0 <= 1e-3, "{}: a current {di_w0:e} of W0 off", b.name);
        assert!(shift <= 2.0 * e / 1e4, "{}: an edge {shift:e} s off", b.name);
    }
    println!("{report}");
}
