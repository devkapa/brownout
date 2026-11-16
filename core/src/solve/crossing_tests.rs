//! Crossings of thresholds that move with a draining cell's rail, or that a
//! node it drives crosses, against a run stopping every 1 us (CAD-3, R121's
//! fine-reference rule): every flip is kept and lands within its tau_w0
//! (the time its node takes to cross W0 of its threshold) at coarse host
//! stops. Each board is built here; every expected value is derived in its
//! test's comment.

use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

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

/// An LED (vf 2 V, n 2, 20 mA rated).
fn led(ci: usize, a: usize, k: usize) -> Device {
    dev(kinds::LED, ci, &[a, k], vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0], vec![])
}

/// A Li-ion-like cell on node 1 (E 3.41 V empty to 4.88 V full), internal
/// resistance `rint`, capacity `cap_ah`, charged to `soc0`.
fn cell(ci: usize, rint: f64, cap_ah: f64, soc0: f64) -> Device {
    dev(kinds::BATTERY_PACK, ci, &[1, 0], vec![1.0, rint, 1.0, 0.0, 2.0, 0.05, cap_ah, soc0, 0.0], vec![2.0, 0.0, 3.4074, 1.0, 4.8837])
}

/// A 555 on the rail (node 1): trigger, output, control, threshold,
/// discharge; its flips trace on `slot`.
fn n555(ci: usize, trig: usize, out: usize, ctrl: usize, thr: usize, dis: usize, slot: usize) -> Device {
    dev(kinds::NE555, ci, &[1, 0, trig, out, 1, ctrl, thr, dis], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![slot as f64, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
}

/// A 74HC74 on the rail (2-6 V supply window, 20 ns delays).
fn hc74(ci: usize, logic: [usize; 12], slot: usize) -> Device {
    let mut pins = vec![pin(1), pin(0)];
    pins.extend(logic.iter().map(|&n| pin(n)));
    let mut p = vec![0.0; 13];
    p[0] = 2.0;
    p[1] = 6.0;
    p[2] = 1.0;
    p[3] = 1.0;
    p[4] = 0.8;
    p[5] = 1.0;
    p[6] = 2.0;
    p[10] = 5.0;
    p.extend(std::iter::repeat_n(20e-9, 4));
    let mut blob = vec![slot as f64];
    blob.extend(std::iter::repeat_n(0.0, 12));
    Device { kind: kinds::IC_74HC74, component_index: ci, pins, params: p, blob }
}

fn prog(n: usize, devices: Vec<Device>) -> Program {
    Program { carry: Vec::new(), node_count: n, options: Options::default(), digital_slot_count: 64, channel_slot_count: 0, devices }
}

/// The 555 astable of SHIPV-E's boards: RA 2438, RB 1014, C 468 nF, timing
/// capacitor on node 3, discharge on node 2, output on node 4, control on
/// node 6 (10 nF); a 35 ohm + LED load on its output when `led_load`.
fn astable(rint: f64, cap_ah: f64, soc0: f64, rl: Option<f64>, led_load: bool) -> Vec<Device> {
    let mut d = vec![cell(0, rint, cap_ah, soc0)];
    if let Some(rl) = rl {
        d.push(r(1, 1, 0, rl));
    }
    d.extend([n555(2, 3, 4, 6, 3, 2, 8), r(3, 1, 2, 2438.0), r(4, 2, 3, 1014.0), c(5, 3, 0, 4.68e-7), c(6, 6, 0, 1e-8)]);
    if led_load {
        d.push(r(7, 4, 5, 35.0));
        d.push(led(8, 5, 0));
    }
    d
}

/// Every flip traced on `slot`, `(t, level)`, with the host stopping at
/// each of `stops`.
fn flips(p: &Program, slot: usize, stops: &[f64]) -> Vec<(f64, f64)> {
    let mut e = Engine::load(p).unwrap();
    e.trace_enable(true);
    let mut out = Vec::new();
    for &t in stops {
        e.advance(t).unwrap();
        out.extend(e.trace().chunks(3).filter(|ch| ch[1] as usize == slot).map(|ch| (ch[0], ch[2])));
        e.trace_clear();
    }
    out
}

/// Host stops every `h` to `t_end`, the last at `t_end`.
fn stops(h: f64, t_end: f64) -> Vec<f64> {
    let n = (t_end / h).ceil() as usize;
    (1..=n).map(|k| (k as f64 * h).min(t_end)).collect()
}

/// `slot`'s flips at each host cadence of `hosts` against a run stopping
/// every 1 us: the same flips, each within `tau_w0`. Returns the worst
/// share of `tau_w0` over the cadences.
fn against_1_us(name: &str, p: &Program, slot: usize, t_end: f64, hosts: &[f64], tau_w0: f64, min_flips: usize) -> f64 {
    let fine = flips(p, slot, &stops(1e-6, t_end));
    assert!(fine.len() >= min_flips, "{name}: {} flips in the 1 us run", fine.len());
    let mut worst_all = 0.0f64;
    for &h in hosts {
        let got = flips(p, slot, &stops(h, t_end));
        let first_off = got.iter().zip(&fine).position(|(g, f)| g.1 != f.1 || (g.0 - f.0).abs() > tau_w0);
        let (worst, at) = got.iter().zip(&fine).enumerate().map(|(i, (g, f))| ((g.0 - f.0).abs(), i)).fold((0.0, 0), |m, x| if x.0 > m.0 { x } else { m });
        eprintln!("{name} at host {h:e}: {} flips against {}, worst {worst:.3e} s (#{at}) = {:.4} tau_w0", got.len(), fine.len(), worst / tau_w0);
        assert_eq!(got.len(), fine.len(), "{name} at host {h:e}: {} flips against the 1 us run's {} (first off: {:?})", got.len(), fine.len(), first_off.map(|i| (fine[i], got.get(i))));
        assert!(first_off.is_none(), "{name} at host {h:e}: flip {:?} off the 1 us run's {:?} by more than tau_w0 {tau_w0:.3e} s", first_off.map(|i| got[i]), first_off.map(|i| fine[i]));
        worst_all = worst_all.max(worst / tau_w0);
    }
    worst_all
}

/// The 555's tau_w0 on the astable at a rail of `vcc`: the smaller of its
/// two crossings'. At the trigger (1/3 vcc) the capacitor discharges through
/// RB alone, `dV/dt = (vcc / 3) / (RB C)`; at the threshold (2/3 vcc) it
/// charges through RA + RB from the rail, `dV/dt = (vcc / 3) / ((RA + RB)
/// C)`; W0_V(v) = 1e-4 + 1e-3 v.
fn astable_tau_w0(vcc: f64) -> f64 {
    let cap = 4.68e-7;
    let trig = (1e-4 + 1e-3 * vcc / 3.0) / ((vcc / 3.0) / (1014.0 * cap));
    let thr = (1e-4 + 2e-3 * vcc / 3.0) / ((vcc / 3.0) / (3452.0 * cap));
    trig.min(thr)
}

#[test]
fn a_555_clocking_a_74hc74_on_a_draining_cell_flips_with_a_1_us_run() {
    // CAD-3, the `ctr` board (SHIPV-E): the astable above on a 0.5 ohm,
    // 2e-5 Ah cell at 90 % (E 4.74 V, the rail 4.72 V), 400 ohm across the
    // rail, its output clocking a 74HC74 wired as a toggle. At a 555
    // crossing the rail solved 1.5e-9 V off the cell's coupled path,
    // stepping the 2/3 threshold back 1.0e-9 V across the node landed a
    // nudge (1e-9 V) past it; the re-aim's root, 9.3e-16 s on, was no
    // crossing the schedule accepts, and the flip waited for the next solve:
    // 54 us late at 1 ms host stops (0.0323 s; 103 tau_w0) before R121, and
    // under R121 two of 548 flips lost to 0.4 s (the trigger at 0.3731 s
    // landed at the 0.374 s stop). The rail falls from 4.72 V; tau_w0 shrinks
    // as the rail rises, so it is taken at 4.72 V: the trigger's, (1e-4 +
    // 1.573e-3) / (1.573 / (1014 x 468e-9)) = 0.504 us. Fails first at 1 ms
    // hosts; with a root at the instant landed at the next accepted one,
    // every flip within 0.002 tau_w0.
    let mut d = astable(0.5, 2e-5, 0.9, Some(400.0), false);
    d.push(hc74(7, [1, 5, 4, 1, 5, 7, 1, 0, 0, 1, 0, 0], 12));
    let p = prog(8, d);
    against_1_us("ctr", &p, 8, 0.4, &[1e-3, 1.0 / 60.0, 0.1], astable_tau_w0(4.72), 500);
}
