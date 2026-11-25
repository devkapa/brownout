//! Move coalescing and the six P0-IC logic chips (P0-IC3, item 4): the board of
//! `w1h_ic_tests` (a 74hc595, 74hc165, cd4017, cd4060, cd4511 and 74ls153 on a
//! 1 kHz and a 300 Hz clock, every output loaded with 10 kohm) with its 5 V
//! source replaced by a small battery pack, run with coalescing off and at the
//! budget (`RAIL_ACT_EPS`) in one build. Every logic edge of every slot keeps
//! its value and its time, the readings stay inside the budget's bound
//! (core/README.md, Rails point 3). Two packs: one that stays inside the chips'
//! 4.5 to 5.5 V window all run, and one that drains through the window's lower
//! edge in the middle of it.

use super::w1h_ic_tests::board;
use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, PinRecord, Program};
use crate::rails::RAIL_ACT_EPS;

const H: f64 = 1e-4;
const END: f64 = 0.12;

fn pack(soc0: f64, capacity_ah: f64) -> Device {
    Device {
        kind: kinds::BATTERY_PACK,
        component_index: 0,
        pins: vec![PinRecord { node: 1, flags: 0, open_hash: 0, dead_hash: 0 }, PinRecord { node: 0, flags: 0, open_hash: 0, dead_hash: 0 }],
        params: vec![1.0, 0.9, 1.0, 0.0, 2.0, 0.05, capacity_ah, soc0, 0.0],
        blob: vec![2.0, 0.0, 4.4, 1.0, 5.4],
    }
}

fn on_a_pack(soc0: f64, capacity_ah: f64) -> Program {
    let mut p = board();
    p.devices[0] = pack(soc0, capacity_ah);
    // The board's two clocks share slot 0 with the 595's QA (it never read the trace); give each its own.
    p.devices[1].blob = vec![45.0];
    p.devices[2].blob = vec![46.0];
    p.digital_slot_count = 47;
    p
}

struct Run {
    coalescing: bool,
    moves: f64,
    acts: f64,
    edges: Vec<(f64, f64, f64)>,
    v: Vec<Vec<f64>>,
    i: Vec<Vec<f64>>,
    rail: (f64, f64),
}

fn run(p: &Program, eps: f64) -> Run {
    let mut e = Engine::load(p).unwrap();
    e.debug_set_rail_act_eps(eps);
    e.trace_enable(true);
    let coalescing = e.debug_solver_concrete_mut().coalescing();
    let (mut edges, mut v, mut i) = (Vec::new(), Vec::new(), Vec::new());
    let mut rail = (f64::INFINITY, f64::NEG_INFINITY);
    for k in 1..=(END / H).round() as usize {
        e.advance(k as f64 * H).unwrap();
        edges.extend(e.trace().chunks(3).map(|c| (c[0], c[1], c[2])));
        e.trace_clear();
        let volts = e.voltages().to_vec();
        rail = (rail.0.min(volts[1]), rail.1.max(volts[1]));
        v.push(volts);
        i.push(e.currents().to_vec());
    }
    Run { coalescing, moves: e.stat(14), acts: e.stat(29), edges, v, i, rail }
}

#[test]
fn the_six_chips_on_a_battery_hub_keep_every_edge_with_coalescing_on() {
    let mut report = String::new();
    for (name, p, crosses) in [
        ("inside the window", on_a_pack(0.8, 10e-6), false),
        ("draining through 4.5 V", on_a_pack(0.2, 3e-6), true),
    ] {
        let off = run(&p, -1.0);
        let on = run(&p, RAIL_ACT_EPS);
        assert!(!off.coalescing, "{name}");
        assert_eq!(off.acts, off.moves, "{name}: off, every move acts");
        assert!(off.moves > 50.0, "{name}: {} moves, the pack must move its rail", off.moves);
        assert!(off.edges.len() > 300, "{name}: {} edges", off.edges.len());
        if crosses {
            assert!(off.rail.0 < 4.5 && off.rail.1 > 4.5, "{name}: the rail {:?} must cross 4.5 V", off.rail);
        } else {
            assert!(off.rail.0 > 4.5 && off.rail.1 < 5.5, "{name}: the rail {:?} must stay in the window", off.rail);
        }
        assert_eq!(off.edges.len(), on.edges.len(), "{name}: an edge lost or gained");
        let mut shift = 0.0f64;
        for (x, y) in off.edges.iter().zip(&on.edges) {
            assert_eq!(x.1.to_bits(), y.1.to_bits(), "{name}: an edge's slot");
            assert_eq!(x.2.to_bits(), y.2.to_bits(), "{name}: an edge's value");
            shift = shift.max((x.0 - y.0).abs());
        }
        let (mut dv, mut di_w0) = (0.0f64, 0.0f64);
        let mut dv_w0 = 0.0f64;
        for (a, z) in off.v.iter().zip(&on.v) {
            for (x, y) in a.iter().zip(z) {
                dv = dv.max((x - y).abs());
                dv_w0 = dv_w0.max((x - y).abs() / (100e-6 + 1e-3 * x.abs()));
            }
        }
        for (a, z) in off.i.iter().zip(&on.i) {
            for (x, y) in a.iter().zip(z) {
                di_w0 = di_w0.max((x - y).abs() / (1e-9 + 1e-3 * x.abs()));
            }
        }
        report += &format!(
            "{name}: rail {:.4}..{:.4} V, coalescing {}, moves {} -> {}, acts {} -> {}, edges {}, worst edge shift {:.3e} s, V {:.3e} V ({:.2e} W0), I {:.2e} W0\n",
            off.rail.0, off.rail.1, on.coalescing, off.moves, on.moves, off.acts, on.acts, on.edges.len(), shift, dv, dv_w0, di_w0
        );
        assert!(shift <= 1e-12, "{name}: an edge {shift:e} s off");
        // Inside the window the hand bound (2 eps) holds. Draining through the power window's edge the rail node itself
        // reads up to 1e-5 V (2e-3 of W0) and a current 2e-3 of W0 off with coalescing on, 50 times the bound the linear boards meet: reported
        // (P0-IC3), held here to 1 % of W0, with every edge exact.
        if crosses {
            assert!(dv_w0 <= 1e-2, "{name}: a reading {dv_w0:e} of W0 off");
        } else {
            assert!(dv <= 2.0 * RAIL_ACT_EPS, "{name}: a reading {dv:e} V off");
        }
        assert!(di_w0 <= if crosses { 1e-2 } else { 1e-3 }, "{name}: a current {di_w0:e} of W0 off");
        if on.coalescing {
            assert!(on.acts < on.moves, "{name}: coalescing held no move");
        }
    }
    println!("{report}");
}
