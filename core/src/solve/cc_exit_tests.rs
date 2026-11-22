//! A bench supply's exit from current limit with a loop of capacitors of no
//! resistance through its rail (ruling R68's board: a rail capacitor, and a
//! stepped hub partition joined to the rail and to ground by capacitors of
//! no esr). The last CC step is cut where its rail is just past the setpoint,
//! within `CROSS_TOLERANCE_V` (or within the cut's shortest bracket), and the
//! partition's charges hold that overshoot `d`. At the exit its CV seed holds
//! the rail at the setpoint, so the loop's capacitors no longer sum to it,
//! and the branch formulation of a loop of esr-free capacitors is singular:
//! the seed kept its companion's current, the loop's series capacitance over
//! `SEED_H` times `d`, which the supply's KCL read as a sink. On this board
//! `d` = 2.3 pV read as 1.6 uA, and the supply declined as back-driven
//! (`bench-psu-back-driven`) at host cadences that put `d` past 1.5 pV.
//!
//! The truth is the closed form: in CC the rail ramps at `I / (C10 + Cs)`,
//! `Cs` the series pair, its node at `C15 / (C15 + C50)` of the rail; from
//! the exit at `V (C10 + Cs) / I` the rail holds the setpoint and no current
//! flows.
use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

const V: f64 = 5.164233;
const I_LIM: f64 = 0.66476;
const C10: f64 = 1.857e-6;
const C15: f64 = 7.812e-5;
const C50: f64 = 6.926e-7;

fn pin(n: usize) -> PinRecord {
    PinRecord { node: n, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>) -> Device {
    Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob: vec![] }
}

/// Nodes: 0 ground, 1 the rail, 2 the loop's node. Devices: 0 PSU, 1 C10
/// rail-gnd, 2 C15 rail-n2, 3 C50 n2-gnd.
fn board() -> Program {
    let devices = vec![
        dev(kinds::BENCH_PSU, 0, &[1, 0], vec![V, I_LIM]),
        dev(kinds::CAPACITOR, 1, &[1, 0], vec![C10, 0.0, 0.0]),
        dev(kinds::CAPACITOR, 2, &[1, 2], vec![C15, 0.0, 0.0]),
        dev(kinds::CAPACITOR, 3, &[2, 0], vec![C50, 0.0, 0.0]),
    ];
    Program { carry: Vec::new(), node_count: 3, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
}

/// The readings at `t`: node voltages by node, currents by device (the
/// supply's published leaving its + pin into the rail as negative).
fn truth(t: f64) -> ([f64; 3], [f64; 4]) {
    let cs = C15 * C50 / (C15 + C50);
    let ct = C10 + cs;
    let share = C15 / (C15 + C50);
    let t_exit = V * ct / I_LIM;
    if t < t_exit {
        let u = I_LIM * t / ct;
        ([0.0, u, share * u], [-I_LIM, C10 * I_LIM / ct, cs * I_LIM / ct, cs * I_LIM / ct])
    } else {
        ([0.0, V, share * V], [0.0; 4])
    }
}

fn w0v(p: f64, q: f64) -> f64 {
    (p - q).abs() / (1e-4 + 1e-3 * q.abs())
}

fn w0i(p: f64, q: f64) -> f64 {
    (p - q).abs() / (1e-9 + 1e-3 * q.abs())
}

/// Each stop's worst reading in W0 of the closed form, and what it was.
fn offsets(stops: &[f64]) -> Vec<(f64, f64, String)> {
    let mut e = Engine::load(&board()).expect("loads");
    let mut out = Vec::new();
    for &t in stops {
        if let Err(er) = e.advance(t) {
            panic!("advance to {t} s failed: {} ({}); so far {out:?}", er.code, er.message);
        }
        let (v, i) = (e.voltages().to_vec(), e.currents().to_vec());
        let (tv, ti) = truth(t);
        let (mut worst, mut what) = (0.0f64, String::new());
        for n in 1..3 {
            let x = w0v(v[n], tv[n]);
            if x > worst {
                (worst, what) = (x, format!("v{n} {:.12} truth {:.12}", v[n], tv[n]));
            }
        }
        for d in 0..4 {
            let x = w0i(i[d], ti[d]);
            if x > worst {
                (worst, what) = (x, format!("i{d} {:.6e} truth {:.6e}", i[d], ti[d]));
            }
        }
        out.push((t, worst, what));
    }
    out
}

/// Three host cadences, each of which cut the last CC step past the setpoint
/// by more than 1.5 pV (2.3, 2.6 and 4.4 pV): the first has a stop 0.6 ps
/// after the exit (19.7594564 us).
const CADENCES: [&[f64]; 3] = [
    &[1.5e-5, 1.9759457e-5, 2e-5, 1e-4],
    &[1e-5, 1.5e-5, 1.976e-5, 2e-5, 1e-4, 1e-3],
    &[1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 2e-5, 1e-4, 1e-3],
];

#[test]
fn a_cc_exit_through_a_capacitor_loop_of_no_esr_leaves_the_supply_sourcing_and_reads_the_closed_form() {
    for stops in CADENCES {
        let offs = offsets(stops);
        for (t, w, what) in &offs {
            assert!(*w <= 1.0, "stops {stops:?}, at {t} s: {w:.3} W0 off the closed form ({what}); all: {offs:?}");
        }
    }
}
