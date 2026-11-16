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

/// A 5 V clock on `pos` tracing on `slot`.
fn clock(ci: usize, pos: usize, freq: f64, duty: f64, slot: usize) -> Device {
    dev(kinds::CLOCK, ci, &[pos, 0], vec![freq, duty, 5.0, 0.0, 0.0], vec![slot as f64])
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

#[test]
fn a_555_on_a_fast_draining_cells_coupled_path_keeps_every_trigger_at_coarse_host_stops() {
    // CAD-3, SHIPV-E's repro: the astable with a 35 ohm + LED load on its
    // output, on a 0.5 ohm, 2e-5 Ah cell at 90 % (the rail falls 4.71 V to
    // 4.47 V in 0.3 s, 0.8 V/s), nothing else across the rail (1e9 ohm). At
    // the output's fall (0.29150 s) the LED's load left the drain line's
    // slope 0.95 to 0.14 V/s across a solve 1.9e-17 s earlier; R121's time
    // watch read that as a line changing at 4e16 V/s^2 and put its bound
    // `cad_t` at the solve itself. The crossing search ran to that bound and
    // found nothing; as a watch the bound read as reached and scheduled
    // nothing, and the rail next solved at its drift cap's passage 6.7 ms on:
    // the trigger due 0.33 ms after the fall came 6.4 ms late, and with it
    // 9 of 412 flips were lost by 0.3 s at 1/60 s and 100 ms host stops
    // (11,645 tau_w0). tau_w0 at 4.71 V is the trigger's, (1e-4 + 1.57e-3) /
    // (1.57 / (1014 x 468e-9)) = 0.504 us. Fails first at 1/60 s (403 of
    // 412 flips); with the rail solving by the instant its crossings were
    // searched to, within 0.002 tau_w0 at every cadence.
    let p = prog(7, astable(0.5, 2e-5, 0.9, Some(1e9), true));
    against_1_us("astable", &p, 8, 0.3, &[1e-3, 1.0 / 60.0, 0.1], astable_tau_w0(4.71), 400);
}

#[test]
fn a_555_on_a_cell_a_clock_charges_keeps_every_trigger_at_coarse_host_stops() {
    // CAD-3, SHIPV-E's `charge` board: the same astable and LED load on a
    // 0.5 ohm, 2e-5 Ah cell at 30 %, charged through 100 ohm by a 5 V, 7 Hz
    // clock (its rail rising and falling with each half period). R121 lost a
    // flip here and the base did not: fails first at 100 ms (411 of 412, the
    // trigger at 0.26162 s found 0.51 ms late). The clock's own flips (slot 1) are
    // exact at every cadence; the 555's are held to tau_w0 at the highest
    // rail the clock can pull the cell to, 5 V (tau_w0 shrinks as the rail
    // rises): (1e-4 + 1.667e-3) / (1.667 / (1014 x 468e-9)) = 0.503 us.
    let mut d = vec![cell(0, 0.5, 2e-5, 0.3), clock(9, 7, 7.0, 0.5, 1), r(10, 7, 1, 100.0)];
    d.extend(astable(0.5, 2e-5, 0.3, None, true).into_iter().skip(1));
    let p = prog(8, d);
    against_1_us("charge", &p, 8, 0.3, &[1e-3, 1.0 / 60.0, 0.1], astable_tau_w0(5.0), 100);
    against_1_us("charge clock", &p, 1, 0.3, &[1e-3, 1.0 / 60.0, 0.1], 1e-12, 4);
}

/// A 74LS00-shaped gate on the rail (node 1) whose first gate's inputs are
/// `input` and output `out` (the other three gates grounded), tracing on
/// `slot`: `(vil, vih)` fixed volts (`ratio` false) or fractions of the
/// supply (`ratio` true), an optional Schmitt pair `(vt+, vt-)` at the
/// nominal 5 V (scaled with the supply), powered from `vcc_min` to 5.25 V.
fn gate(ci: usize, input: usize, out: usize, slot: usize, ratio: bool, levels: (f64, f64), schmitt: Option<(f64, f64)>, vcc_min: f64) -> Device {
    let mut pins = vec![pin(1), pin(0)];
    pins.extend([input, input, out, 0, 0, 0, 0, 0, 0, 0, 0, 0].iter().map(|&n| pin(n)));
    let mut p = vec![0.0; 13];
    p[0] = vcc_min;
    p[1] = 5.25;
    p[2] = if ratio { 2.0 } else { 1.0 };
    p[3] = if ratio { 0.0 } else { 1.0 };
    p[4] = levels.0;
    p[5] = if ratio { 0.0 } else { 1.0 };
    p[6] = levels.1;
    if let Some((vp, vm)) = schmitt {
        p[7] = 1.0;
        p[8] = vp;
        p[9] = vm;
    }
    p[10] = 5.0;
    p[11] = 0.008;
    p[12] = 10e-9;
    let mut blob = vec![slot as f64];
    blob.extend((0..12).map(|i| if i % 3 == 2 { 0.0 } else { 1.0 }));
    Device { kind: kinds::IC_74LS00, component_index: ci, pins, params: p, blob }
}

/// The rail's fall over the 1 us run, V/s.
fn drain_rate(p: &Program, t_end: f64) -> f64 {
    let mut e = Engine::load(p).unwrap();
    e.advance(1e-6).unwrap();
    let u0 = e.voltages()[1];
    e.advance(t_end).unwrap();
    (u0 - e.voltages()[1]) / (t_end - 1e-6)
}

/// SHIPV-E's astable (LED load, nothing else across the rail) on a
/// 0.5 ohm, 2e-5 Ah cell at 90 %, the rail falling about 0.8 V/s, with
/// `more` added: every member of the class below rides its coupled path.
fn on_draining_astable(more: Vec<Device>, nodes: usize) -> Program {
    let mut d = astable(0.5, 2e-5, 0.9, None, true);
    d.extend(more);
    prog(nodes, d)
}

#[test]
fn every_threshold_a_draining_cells_rail_moves_or_drives_is_crossed_with_a_1_us_run() {
    // CAD-3 item 2: the crossings R121 left unprotected were those the
    // closed-form solver searches on a draining cell's coupled path only to
    // `cell_until` (`schedule_driven_crossing`), and those a solve at the
    // crossing steps back across (`solve_rail_before_crossing`): every
    // watched pin of a closed-form partition whose threshold reads the rail
    // or whose node the rail drives. One board per member, each on the
    // draining astable (the rail falls at least 0.5 V/s, checked), each
    // flip held to the 1 us run's within the member's tau_w0 at 1 ms, 1/60 s
    // and 100 ms host stops. tau_w0 for an input at level v on the timing
    // capacitor (1.57 to 3.14 V at a 4.71 V rail): discharging through RB
    // its slope is v / (RB C), so W0_V(v) / slope = RB C (1e-4 / v + 1e-3),
    // least at the top, 474.6 us x 1.032e-3 = 0.490 us; charging through
    // RA + RB it is at least 0.86 us. Every input is held to 0.490 us.
    let tau_in = 1014.0 * 4.68e-7 * (1e-4 / 3.14 + 1e-3);
    let hosts = [1e-3, 1.0 / 60.0, 0.1];
    let t_end = 0.3;
    // A logic input of fixed levels (1.8 / 2.2 V) on the timing capacitor,
    // a node the rail drives through RA + RB.
    let fixed = on_draining_astable(vec![gate(9, 3, 9, 0, false, (1.8, 2.2), None, 2.0), r(10, 9, 0, 1e4)], 10);
    // A comparator-like supply-ratio input (CMOS ratio levels, 0.3 / 0.7 of
    // the supply), its levels moving with the rail, on the discharge node
    // (node 2): while the capacitor charges it is `vc + (u - vc) RB / (RA +
    // RB)`, 2.49 to 3.60 V at 4.71 V, crossing 0.7 u = 3.30 V each period
    // at 872 V/s (tau_w0 3.9 us, so 0.490 us is the tighter bound); the
    // discharge pulls it low at the 555's own flip.
    let ratio = on_draining_astable(vec![gate(9, 2, 9, 0, true, (0.0, 0.0), None, 2.0), r(10, 9, 0, 1e4)], 10);
    // A Schmitt input, 2.9 / 2.0 V at the nominal 5 V scaled with the
    // supply (2.73 / 1.88 V at 4.71 V).
    let schmitt = on_draining_astable(vec![gate(9, 3, 9, 0, false, (0.8, 2.0), Some((2.9, 2.0)), 2.0), r(10, 9, 0, 1e4)], 10);
    // A chip's power edge: the fixed-level gate powered from 4.6 V, which
    // the falling rail crosses near 0.14 s; its flips stop there.
    let power = on_draining_astable(vec![gate(9, 3, 9, 0, false, (1.8, 2.2), None, 4.6), r(10, 9, 0, 1e4)], 10);
    for (name, p) in [("fixed-level input", &fixed), ("supply-ratio input", &ratio), ("Schmitt input", &schmitt), ("power edge", &power)] {
        let rate = drain_rate(p, t_end);
        assert!(rate >= 0.5, "{name}: the rail falls {rate:.3} V/s");
        against_1_us(name, p, 0, t_end, &hosts, tau_in, 20);
        against_1_us(&format!("{name}'s 555"), p, 8, t_end, &hosts, astable_tau_w0(4.71), 400);
    }
}

#[test]
fn a_555_reset_an_led_knee_and_a_fuse_on_a_draining_cells_path_cross_with_a_1_us_run() {
    // CAD-3 item 2, the class's other members on the draining astable:
    // - the 555's reset, on a node the rail charges (47k into 1 uF from the
    //   rail): the 555 runs from its release, so every later flip carries
    //   the release's time; the reset node's slope at its level is the RC's,
    //   at least (4.5 - 1) / 47 ms = 74 V/s, so tau_w0 at a 1 V level is
    //   1.1e-3 / 74 = 15 us, and the flips are held to the astable's own
    //   0.504 us (tighter);
    // - an LED's knee on a curved closed-form partition (100 ohm into 10 uF
    //   with an LED across it, charging through the knee near 2 V), which
    //   keeps the 10 uV band on the cell's path (CAD-2): the 555's flips
    //   are held as above and the LED node at every host stop within W0 of
    //   the 1 us run's;
    // - a fuse (rated 50 mA, 1 ohm) feeding a 47 ohm load off the rail,
    //   ~95 mA: its blow is a slow latch the rail's drift drives; the 555's
    //   flips (whose load changes when it blows) are held as above.
    let hosts = [1e-3, 1.0 / 60.0, 0.1];
    let t_end = 0.3;
    let mut reset = astable(0.5, 2e-5, 0.9, None, true);
    reset[1].pins[4] = pin(9);
    reset.extend([r(9, 1, 9, 4.7e4), c(10, 9, 0, 1e-6)]);
    let reset = prog(10, reset);
    let knee = on_draining_astable(vec![r(9, 1, 9, 100.0), c(10, 9, 0, 1e-5), led(11, 9, 0)], 10);
    let fuse = on_draining_astable(vec![dev(kinds::FUSE, 9, &[1, 9], vec![1.0, 0.05], vec![]), r(10, 9, 0, 47.0)], 10);
    for (name, p) in [("reset", &reset), ("LED knee", &knee), ("fuse", &fuse)] {
        let rate = drain_rate(p, t_end);
        assert!(rate >= 0.5, "{name}: the rail falls {rate:.3} V/s");
        against_1_us(name, p, 8, t_end, &hosts, astable_tau_w0(4.71), 50);
    }
    // The LED node at each host stop against the 1 us run there.
    let mut fine = Engine::load(&knee).unwrap();
    let mut by_t = std::collections::HashMap::new();
    for t in stops(1e-6, t_end) {
        fine.advance(t).unwrap();
        by_t.insert((t * 1e6).round() as u64, fine.voltages()[9]);
    }
    for h in [1e-3, 0.1] {
        let mut e = Engine::load(&knee).unwrap();
        let mut worst = 0.0f64;
        for t in stops(h, t_end) {
            e.advance(t).unwrap();
            let want = by_t[&((t * 1e6).round() as u64)];
            worst = worst.max((e.voltages()[9] - want).abs() / (1e-4 + 1e-3 * want.abs()));
        }
        eprintln!("LED knee node at host {h:e}: worst {worst:.4} W0");
        assert!(worst <= 1.0, "the LED node {worst:.3} W0 off the 1 us run at host {h:e}");
    }
}

/// A small deterministic generator (xorshift64*).
struct Rng(u64);

impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn log(&mut self, lo: f64, hi: f64) -> f64 {
        (lo.ln() + self.unit() * (hi / lo).ln()).exp()
    }
}

/// Board `seed` of the property: a 555 astable (even seeds) or a
/// monostable a 5 V clock triggers (odd), on a cell of 1e-6 to 1 Ah (log
/// uniform), 0.05 to 2 ohm, 30 to 95 % charged, with a load of 100 ohm to
/// 100 kohm across the rail or none (a quarter), and an LED load on the
/// output on half. Returns the program and its 555's tau_w0, taken at the
/// full cell's 4.88 V (tau_w0 shrinks as the rail rises): an astable's
/// trigger `RB C (3e-4 / v + 1e-3)` and threshold `(RA + RB) C (3e-4 / v +
/// 2e-3)` (W0_V of v/3 and 2v/3 over the capacitor's slope there), a
/// monostable's threshold `RT C (3e-4 / v + 2e-3)` (its triggers are the
/// clock's own edges).
fn property_board(seed: u64) -> (Program, f64) {
    let mut g = Rng(0x9e37_79b9_7f4a_7c15 ^ (seed + 1).wrapping_mul(0xbf58_476d_1ce4_e5b9));
    let cap_ah = g.log(1e-6, 1.0);
    let rint = g.log(0.05, 2.0);
    let soc = 0.3 + 0.65 * g.unit();
    let rl = if g.unit() < 0.25 { None } else { Some(g.log(100.0, 1e5)) };
    let led_load = g.unit() < 0.5;
    let vmax = 4.8837;
    let mut d = vec![cell(0, rint, cap_ah, soc)];
    if let Some(rl) = rl {
        d.push(r(1, 1, 0, rl));
    }
    let tau = if seed % 2 == 0 {
        let (ra, rb, cap) = (g.log(1e3, 3e4), g.log(1e3, 3e4), g.log(4.7e-8, 1e-6));
        d.extend([n555(2, 3, 4, 6, 3, 2, 8), r(3, 1, 2, ra), r(4, 2, 3, rb), c(5, 3, 0, cap), c(6, 6, 0, 1e-8)]);
        (rb * cap * (3e-4 / vmax + 1e-3)).min((ra + rb) * cap * (3e-4 / vmax + 2e-3))
    } else {
        let (rt, cap) = (g.log(1e3, 3e4), g.log(4.7e-8, 1e-6));
        let width = 1.1 * rt * cap;
        let freq = 1.0 / ((2.0 + 2.0 * g.unit()) * width);
        d.extend([n555(2, 5, 4, 6, 3, 3, 8), r(3, 1, 3, rt), c(4, 3, 0, cap), c(5, 6, 0, 1e-8), clock(6, 5, freq, 0.9, 1)]);
        rt * cap * (3e-4 / vmax + 2e-3)
    };
    if led_load {
        d.extend([r(7, 4, 5, 35.0), led(8, 5, 0)]);
    }
    (prog(7, d), tau)
}

/// Rail events (stat 18) of `p` run to `t_end` with the host stopping every
/// `h`, on R121's cadence or on the 10 uV band.
fn rail_events(p: &Program, h: f64, t_end: f64, band: bool) -> f64 {
    crate::rails::force_fixed_cell_band(band);
    let mut e = Engine::load(p).unwrap();
    for t in stops(h, t_end) {
        e.advance(t).unwrap();
    }
    crate::rails::force_fixed_cell_band(false);
    e.stat(18)
}

#[test]
fn a_small_cell_feeding_a_fast_monostable_solves_its_rail_no_more_often_than_the_band() {
    // CAD-4, SHIPV-G's M272: a 555 monostable (RT 1379 ohm, 219 nF, 4.7 k
    // on its output) fired by a 776 Hz clock, on a 3 uAh, 0.109 ohm cell at
    // 92 % with nothing else across the rail. The band solves every 10 uV
    // of the rail's path: the rail falls from E(0.919) = 3.4074 + 0.919 x
    // 1.4763 = 4.764 V to 4.621 V by 0.1275 s, 0.143 V, so about 14,300
    // solves (14.0k measured at 1 ms hosts; 2.0k at 1 us, where every stop
    // solves it). R121's time watch held the drain's line straight only as
    // long as `C rho_dot tau` kept the timing capacitor's current within
    // 0.2 of its W0, whose 1 nA floor binds: the line turns with every
    // charge of the 219 nF capacitor (its 0.30 ms time constant), rho_dot
    // about 1e3 V/s^2, so tau = 0.2 x 1.07 nA / (219 nF x 1e3 V/s^2) = 1 us
    // (0.6 us probed), and the cadence took 35.6k rail events at 1 ms hosts
    // (17.2k at 1 us). The band held the same line straight for its whole
    // 9 us passage, so within it the readings err no more than the band's;
    // the bound now never falls short of the band's passage. Margin: the
    // cadence also lands its own thresholds (the scan band every 20 mV of
    // travel, the curved partitions' bands, the logic levels), each passed
    // once per such travel; 100 covers them (SHIPV-G's 600 boards: none
    // above the band at coarse hosts, at most 87 above at 1 us).
    let d = vec![cell(0, 0.10869160147277165, 3.006505746061428e-6, 0.9189997502889509), n555(10, 5, 4, 6, 3, 3, 8), r(11, 1, 3, 1378.697367889984), c(12, 3, 0, 2.1865801889116618e-7), c(13, 6, 0, 1e-8), clock(14, 5, 776.4704769516004, 0.9, 1), r(15, 4, 0, 4700.0)];
    let p = prog(7, d);
    let t_end = 0.12747014170965318;
    for h in [1e-3, 1.0 / 60.0, 0.1, 1e-6] {
        let (cad, band) = (rail_events(&p, h, t_end, false), rail_events(&p, h, t_end, true));
        eprintln!("M272 at host {h:e}: {cad} rail events on the cadence, {band} on the band");
        assert!(cad <= band + 100.0, "M272 at host {h:e}: {cad} rail events on the cadence against the band's {band}");
    }
}

/// How an oscillator's flips at a coarse host stand against the 1 us
/// run's, as R127 judges them (the coordinator's ruling, replacing R121
/// (iii)'s "within tau_w0 of the band"): (1) the same flips over the whole
/// run; (2) its period, the mean over the run and the worst single one,
/// relative to the 1 us run's; (3) the worst edge, as a share of `tau_w0`,
/// over the first `per` periods after each edge `rephased` names (an edge
/// something outside the oscillator set: the load, a trigger), and over the
/// whole run for the record.
struct R127 {
    mean_period: f64,
    worst_period: f64,
    rephased_edge: f64,
    any_edge: f64,
}

fn r127(got: &[(f64, f64)], fine: &[(f64, f64)], tau_w0: f64, per: usize, rephased: &dyn Fn(usize) -> bool) -> Result<R127, String> {
    if got.len() != fine.len() {
        return Err(format!("{} flips against the 1 us run's {}", got.len(), fine.len()));
    }
    if let Some(k) = got.iter().zip(fine).position(|(g, f)| g.1 != f.1) {
        return Err(format!("flip {k} to {} at {} against {} at {}", got[k].1, got[k].0, fine[k].1, fine[k].0));
    }
    // A period is between two edges of the same level (two flips apart).
    let (mut worst_period, mut n) = (0.0f64, 0usize);
    for k in 2..fine.len() {
        let (pg, pf) = (got[k].0 - got[k - 2].0, fine[k].0 - fine[k - 2].0);
        worst_period = worst_period.max(((pg - pf) / pf).abs());
        n += 1;
    }
    let mean_period = if n == 0 {
        0.0
    } else {
        let last = fine.len() - 1 - (fine.len() - 1) % 2;
        let (pg, pf) = (got[last].0 - got[0].0, fine[last].0 - fine[0].0);
        ((pg - pf) / pf).abs()
    };
    let (mut rephased_edge, mut any_edge, mut since) = (0.0f64, 0.0f64, usize::MAX);
    for (k, (g, f)) in got.iter().zip(fine).enumerate() {
        if rephased(k) {
            since = 0;
        }
        let share = (g.0 - f.0).abs() / tau_w0;
        if since < 2 * per {
            rephased_edge = rephased_edge.max(share);
        }
        any_edge = any_edge.max(share);
        since = since.saturating_add(1);
    }
    Ok(R127 { mean_period, worst_period, rephased_edge, any_edge })
}

#[test]
#[cfg_attr(debug_assertions, ignore = "300 boards against 1 us runs: release only")]
fn random_555s_on_cells_of_any_drain_meet_r127_against_1_us_runs() {
    // CAD-3 item 3, judged by R127 (CAD-4): 300 random 555 astables and
    // monostables (`property_board`) on cells from 1e-6 Ah (empty within
    // the run under a 100 ohm load) to 1 Ah, at host stops of 1 ms, 1/60 s
    // and 100 ms against a run stopping every 1 us, to 80 ms: (1) no flip
    // lost or gained; (2) the period, the mean over the run and the worst
    // single one, within 1e-6 relative of the 1 us run's; (3) every edge
    // within the 555's tau_w0 over the first 100 periods after a re-phase.
    // An astable is phased once, at the load (its first edge); a
    // monostable by every trigger (each clock edge that fires it), so all
    // of its edges count.
    //
    // The cadence's edges drift from the 1 us run's as the rail travels,
    // not by a fixed offset as CAD-3 had it: board 98 (a 28 uAh, 0.148 ohm
    // cell, an LED on the 555's output) is 0.038 tau_w0 off at 80 ms and
    // 0.93 to 1.10 tau_w0 off over a full discharge, a period bias of about
    // 5e-8 (`a_555_on_a_cell_drained_from_full_to_flat_meets_r127` derives
    // it). Measured (CAD-4): 28,380 flips, none lost; period 1.4e-7 mean
    // and 2.1e-7 worst single, relative; edges within 0.043 tau_w0.
    let hosts = [1e-3, 1.0 / 60.0, 0.1];
    let t_end = 0.08;
    let (mut flips_seen, mut worst) = (0usize, (0.0f64, 0.0f64, 0.0f64, 0.0f64));
    for seed in 0..300u64 {
        let (p, tau) = property_board(seed);
        let fine = flips(&p, 8, &stops(1e-6, t_end));
        flips_seen += fine.len();
        let mono = seed % 2 == 1;
        for &h in &hosts {
            let got = flips(&p, 8, &stops(h, t_end));
            let m = r127(&got, &fine, tau, 100, &|k| mono || k == 0).unwrap_or_else(|e| panic!("board {seed} at host {h:e}: {e}"));
            assert!(m.mean_period <= 1e-6 && m.worst_period <= 1e-6, "board {seed} at host {h:e}: period {:.3e} mean, {:.3e} worst single, relative", m.mean_period, m.worst_period);
            assert!(m.rephased_edge <= 1.0, "board {seed} at host {h:e}: an edge {:.3} tau_w0 off within 100 periods of a re-phase", m.rephased_edge);
            worst = (worst.0.max(m.mean_period), worst.1.max(m.worst_period), worst.2.max(m.rephased_edge), worst.3.max(m.any_edge));
        }
    }
    eprintln!("R127 over 300 boards, {flips_seen} flips in the 1 us runs: period {:.3e} mean, {:.3e} worst single; edges {:.4} tau_w0 after a re-phase, {:.4} anywhere", worst.0, worst.1, worst.2, worst.3);
}

#[test]
#[cfg_attr(debug_assertions, ignore = "3.2 s against a 1 us run: release only")]
fn a_555_on_a_cell_drained_from_full_to_flat_meets_r127() {
    // R127 on board 98 of the property (an astable, RA 5248, RB 2442,
    // C 60.7 nF, 35 ohm + LED on its output, a 28.3 uAh, 0.148 ohm cell,
    // nothing else across the rail) charged full: the rail falls 4.876 to
    // 3.425 V in 3.2 s and stays there. Measured (CAD-4): 14,991 flips at
    // every cadence, as at 1 us; period 4.6e-8 to 5.4e-8 mean and 7.5e-8
    // worst single, relative; edges within 0.0103 tau_w0 (157 ns) over
    // the first 100 periods; accumulated phase 0.93 tau_w0 (147 ns) at
    // 1 ms hosts and 1.10 tau_w0 (173 ns) at 1/60 s and 100 ms, 1.0e-7 to
    // 1.19e-7 s per volt of rail travel (0.64 to 0.76 tau_w0 per volt). The
    // pin, 1.6 tau_w0, fails a doubling. The 10 uV band: 0.004 tau_w0.
    //
    // Why a bias of this size. The charge time (output high) is t_c =
    // (RA + RB) C ln((u - v1)/(u - v2)), v1 and v2 the 1/3 and 2/3 levels.
    // If both levels and the source read a rail off by e, dt_c/de =
    // (RA + RB) C (-1/(2u) + 2/u) = 1.5 (RA + RB) C / u = 1.668e-4 s/V at
    // 4.2 V (the discharge, RB C ln 2, is a ratio and does not move). Read
    // 0.2 W0 stale (0.86 mV at 4.2 V), as R127's note supposes, that would
    // be 1.43e-7 s a period of 426 us, 3.4e-4 relative: 7,000 times the
    // 5e-8 measured. So the levels are not read 0.2 W0 stale; they ride the
    // cell's path, and the path's own error is what they see. Probed at
    // every solve over 0.4 s (CAD-4, b98-cadpath.log), the solved rail
    // stands above the path's prediction by e = k dt^2 with k = 3.4 to 5.2
    // V/s^2 at every interval length, one-signed, only while the output is
    // high (rho -0.75 V/s; -0.09 V/s low): the drain's line held straight,
    // `rho_dot tau^2 / 2` with rho_dot about 8 V/s^2 (the LED's current
    // falls with the rail). The 0.2 W0 slack enters through the drift cap
    // (0.6 mV), which the rail's 0.2 mV travel in a charge phase never
    // reaches: the cadence holds the line over the whole phase (268 us, the
    // 10 uV band 15 us), the error reaching 3.75e-7 V by the flip, 1.25e-7 V
    // on average over the phase (a parabola's third). 1.668e-4 s/V x
    // 1.25e-7 V = 2.1e-11 s a period, 4.9e-8 relative: the measured bias.
    // The held line's own bound (`tau`, its error within RAIL_DRIFT_V) does
    // not bind here (1.6 ms); at that bound the same arithmetic gives
    // 1.668e-4 x 10 uV / 3 / 426 us = 1.3e-6, above R127's 1e-6.
    let (mut p, tau) = property_board(98);
    p.devices[0].params[7] = 1.0;
    let t_end = 3.2;
    let fine = flips(&p, 8, &stops(1e-6, t_end));
    let mut e = Engine::load(&p).unwrap();
    e.advance(1e-6).unwrap();
    let u0 = e.voltages()[1];
    e.advance(t_end).unwrap();
    let travel = u0 - e.voltages()[1];
    for h in [1e-3, 1.0 / 60.0, 0.1] {
        let got = flips(&p, 8, &stops(h, t_end));
        let m = r127(&got, &fine, tau, 100, &|k| k == 0).unwrap_or_else(|e| panic!("board 98 from full at host {h:e}: {e}"));
        let phase = got.last().unwrap().0 - fine.last().unwrap().0;
        eprintln!("board 98 from full at host {h:e}: {} flips, rail {u0:.4} V down {travel:.4} V; period {:.3e} mean, {:.3e} worst single; edges {:.4} tau_w0 in the first 100 periods, {:.4} anywhere; phase {phase:.3e} s = {:.4} tau_w0, {:.3e} s = {:.4} tau_w0 per volt", got.len(), m.mean_period, m.worst_period, m.rephased_edge, m.any_edge, phase / tau, phase / travel, phase / tau / travel);
        assert!(m.mean_period <= 1e-6 && m.worst_period <= 1e-6, "board 98 from full at host {h:e}: period {:.3e} mean, {:.3e} worst single", m.mean_period, m.worst_period);
        assert!(m.rephased_edge <= 1.0, "board 98 from full at host {h:e}: an edge {:.3} tau_w0 off in the first 100 periods", m.rephased_edge);
        assert!(m.any_edge <= 1.6, "board 98 from full at host {h:e}: {:.3} tau_w0 of accumulated phase", m.any_edge);
    }
}
