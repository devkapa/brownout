//! COALV3 (verifier) probes: D4 (a) where the rail node of the six-chips-on-a-pack board parts between coalescing on and
//! off while it drains through the power window's lower edge, and (b) whether the edge shifts seen on the older w1h board
//! (two clocks and a 595's QA on digital slot 0) are shifts of one signal or two signals paired by slot.
//! Tests only; they change no default.

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

fn pack_board(soc0: f64, capacity_ah: f64, unique_slots: bool) -> Program {
    let mut p = board();
    p.devices[0] = pack(soc0, capacity_ah);
    if unique_slots {
        p.devices[1].blob = vec![45.0];
        p.devices[2].blob = vec![46.0];
        p.digital_slot_count = 47;
    }
    p
}

type Edges = Vec<(f64, f64, f64)>;

fn run(p: &Program, eps: f64, h: f64, end: f64) -> (Edges, Vec<Vec<f64>>, Vec<Vec<f64>>, Vec<f64>) {
    let mut e = Engine::load(p).unwrap();
    e.debug_set_rail_act_eps(eps);
    e.trace_enable(true);
    let (mut edges, mut v, mut i, mut ts) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for k in 1..=(end / h).round() as usize {
        e.advance(k as f64 * h).unwrap();
        edges.extend(e.trace().chunks(3).map(|c| (c[0], c[1], c[2])));
        e.trace_clear();
        v.push(e.voltages().to_vec());
        i.push(e.currents().to_vec());
        ts.push(k as f64 * h);
    }
    (edges, v, i, ts)
}

/// D4 (a): per sample, the rail node's parting and what the chips draw there.
#[test]
fn coalv3_d4a_where_the_rail_parts() {
    let p = pack_board(0.2, 3e-6, false);
    let (_, voff, ioff, ts) = run(&p, -1.0, H, END);
    let (_, von, ion, _) = run(&p, RAIL_ACT_EPS, H, END);
    let mut worst = (0.0f64, 0usize);
    let mut n_over = 0;
    for k in 0..ts.len() {
        let d = (voff[k][1] - von[k][1]).abs();
        if d > 2.0 * RAIL_ACT_EPS {
            n_over += 1;
        }
        if d > worst.0 {
            worst = (d, k);
        }
    }
    println!("D4a samples over 2 eps: {n_over} of {}", ts.len());
    println!("D4a worst {:.3e} V at t={:.4} off {:.9} on {:.9}", worst.0, ts[worst.1], voff[worst.1][1], von[worst.1][1]);
    for k in worst.1.saturating_sub(6)..(worst.1 + 6).min(ts.len()) {
        println!(
            "D4a t={:.4} off {:.9} on {:.9} d {:.3e}  i_src off {:.6e} on {:.6e}",
            ts[k], voff[k][1], von[k][1], voff[k][1] - von[k][1], ioff[k][0], ion[k][0]
        );
    }
    // the first sample that crosses below 4.5 V in the off run
    if let Some(k) = voff.iter().position(|v| v[1] < 4.5) {
        println!("D4a off rail first below 4.5 V at sample t={:.4}: off {:.9} on {:.9}", ts[k], voff[k][1], von[k][1]);
    }
    // the pack's open-circuit rate: rail slope per sample at the start and at the crossing
    println!("D4a rail slope early {:.3e} V/sample, at the end {:.3e}", voff[1][1] - voff[0][1], voff[ts.len() - 1][1] - voff[ts.len() - 2][1]);
}

/// D4 (a) finer: the same board sampled every 1e-5 s over the window the first run flagged.
#[test]
fn coalv3_d4a_fine() {
    let p = pack_board(0.2, 3e-6, false);
    let (_, voff, _, ts) = run(&p, -1.0, 1e-5, END);
    let (_, von, _, _) = run(&p, RAIL_ACT_EPS, 1e-5, END);
    let mut worst = (0.0f64, 0usize);
    for k in 0..ts.len() {
        let d = (voff[k][1] - von[k][1]).abs();
        if d > worst.0 {
            worst = (d, k);
        }
    }
    println!("D4a-fine worst {:.3e} V at t={:.5}", worst.0, ts[worst.1]);
    let mut hist = [0usize; 6];
    for k in 0..ts.len() {
        let d = (voff[k][1] - von[k][1]).abs();
        let b = if d == 0.0 { 0 } else if d < 1e-7 { 1 } else if d < 2e-7 { 2 } else if d < 1e-6 { 3 } else if d < 1e-5 { 4 } else { 5 };
        hist[b] += 1;
    }
    println!("D4a-fine hist [0, <1e-7, <2e-7, <1e-6, <1e-5, >=1e-5] = {hist:?} of {}", ts.len());
}

/// D4 (b): worst index-paired shift, shared slot 0 against one slot per signal, on and off.
#[test]
fn coalv3_d4b_shared_slot() {
    for (name, unique) in [("shared slot 0", false), ("unique slots", true)] {
        for (pname, p) in [("pack 0.8", pack_board(0.8, 10e-6, unique)), ("pack 0.2", pack_board(0.2, 3e-6, unique))] {
            let (off, _, _, _) = run(&p, -1.0, H, END);
            let (on, _, _, _) = run(&p, RAIL_ACT_EPS, H, END);
            let (mut worst, mut at, mut slot_mismatch, mut val_mismatch) = (0.0f64, 0usize, 0, 0);
            for (k, (x, y)) in off.iter().zip(&on).enumerate() {
                if x.1 != y.1 {
                    slot_mismatch += 1;
                }
                if x.2.to_bits() != y.2.to_bits() {
                    val_mismatch += 1;
                }
                if (x.0 - y.0).abs() > worst {
                    worst = (x.0 - y.0).abs();
                    at = k;
                }
            }
            println!("D4b {name} {pname}: edges off {} on {} worst index-paired shift {:.3e} s (edge {at}: off {:?} on {:?}) slot mismatches {slot_mismatch} value mismatches {val_mismatch}", off.len(), on.len(), worst, off.get(at), on.get(at));
        }
    }
}

/// D4 (a): the rail and the edges every 1e-6 s around the worst fine sample.
#[test]
fn coalv3_d4a_zoom() {
    let p = pack_board(0.2, 3e-6, false);
    let mut out = Vec::new();
    for eps in [-1.0, RAIL_ACT_EPS] {
        let mut e = Engine::load(&p).unwrap();
        e.debug_set_rail_act_eps(eps);
        e.trace_enable(true);
        e.advance(0.0681).unwrap();
        e.trace_clear();
        let mut rows = Vec::new();
        for k in 1..=300 {
            let t = 0.0681 + k as f64 * 1e-6;
            e.advance(t).unwrap();
            let ed: Vec<_> = e.trace().chunks(3).map(|c| format!("{:.9}:{}={}", c[0], c[1], c[2])).collect();
            e.trace_clear();
            rows.push((t, e.voltages()[1], e.currents()[0], ed));
        }
        out.push(rows);
    }
    for k in 0..300 {
        let (a, b) = (&out[0][k], &out[1][k]);
        let d = a.1 - b.1;
        if a.0 > 0.068280 && a.0 < 0.068301 {
            println!("D4z t={:.6} off {:.9} on {:.9} d {:.3e} off-edges {:?} on-edges {:?}", a.0, a.1, b.1, d, a.3, b.3);
        }
    }
}

/// D4 (b): every shared-slot edge that parts between on and off, against the nearest edge of the one-slot-per-signal run.
#[test]
fn coalv3_d4b_which_edges_part() {
    let shared = pack_board(0.8, 10e-6, false);
    let uniq = pack_board(0.8, 10e-6, true);
    let (off, _, _, _) = run(&shared, -1.0, H, END);
    let (on, _, _, _) = run(&shared, RAIL_ACT_EPS, H, END);
    let (uoff, _, _, _) = run(&uniq, -1.0, H, END);
    let mut true_times: Vec<f64> = uoff.iter().map(|e| e.0).collect();
    true_times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let near = |t: f64| true_times.iter().map(|x| (x - t).abs()).fold(f64::INFINITY, f64::min);
    let (mut n, mut near_true, mut on_stop) = (0, 0, 0);
    let mut shown = 0;
    for (k, (x, y)) in off.iter().zip(&on).enumerate() {
        if (x.0 - y.0).abs() > 1e-12 {
            n += 1;
            let stop = ((x.0 / H).round() * H - x.0).abs() < 1e-6;
            if stop {
                on_stop += 1;
            }
            if near(x.0) < 1e-9 && near(y.0) < 1e-9 {
                near_true += 1;
            }
            if shown < 12 {
                shown += 1;
                println!("D4w idx {k} off {:.12}={} on {:.12}={} d {:.3e} nearest true edge off {:.3e} on {:.3e}", x.0, x.2, y.0, y.2, y.0 - x.0, near(x.0), near(y.0));
            }
        }
    }
    println!("D4w shared-slot run: {} edges, {n} part (>1e-12 s), of which both ends on a true edge of the one-slot-per-signal run: {near_true}, within 1 us of a host stop: {on_stop}", off.len());
    // the true edges: shifts per slot in the unique-slot run
    let (uon, _, _, _) = run(&uniq, RAIL_ACT_EPS, H, END);
    let worst = uoff.iter().zip(&uon).map(|(a, b)| (a.0 - b.0).abs()).fold(0.0f64, f64::max);
    println!("D4w unique-slot run: {} edges, worst shift {:.3e} s", uoff.len(), worst);
}
