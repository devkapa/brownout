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

/// `got`'s flips against `fine`'s: the worst offset as a share of
/// `tau_w0`, or why they do not pair (a flip lost or gained, or a level
/// out of turn).
fn share_of(got: &[(f64, f64)], fine: &[(f64, f64)], tau_w0: f64) -> Result<f64, String> {
    if got.len() != fine.len() {
        return Err(format!("{} flips against {}", got.len(), fine.len()));
    }
    let mut worst = 0.0f64;
    for (g, f) in got.iter().zip(fine) {
        if g.1 != f.1 {
            return Err(format!("flip to {} at {} against {} at {}", g.1, g.0, f.1, f.0));
        }
        worst = worst.max((g.0 - f.0).abs() / tau_w0);
    }
    Ok(worst)
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

#[test]
#[cfg_attr(debug_assertions, ignore = "300 boards against 1 us runs: release only")]
fn random_555s_on_cells_of_any_drain_keep_every_flip_within_tau_w0_and_no_farther_than_the_band() {
    // CAD-3 item 3: 300 random 555 astables and monostables (`property_board`)
    // on cells from 1e-6 Ah (empty within the run under a 100 ohm load) to
    // 1 Ah, at host stops of 1 ms, 1/60 s and 100 ms against a run stopping
    // every 1 us, to 80 ms: no flip lost or gained, every flip within the
    // 555's tau_w0, and none farther from the 1 us run than the same board
    // on the 10 uV band (`force_fixed_cell_band`, the build before R121 with
    // this branch's base fix) by more than R121's reading slack: a reading
    // between solves may err `CELL_READING_SLACK` (0.2) W0 more than on the
    // band, which at a threshold is 0.2 tau_w0 of its crossing's time.
    // Measured: no flip lost in 28,380; worst 0.038 tau_w0; three boards
    // (10, 98, 262) sit 0.012 to 0.038 tau_w0 off where the band sits at
    // 0.0001 or less, at every cadence alike (1 ms included), the cadence's
    // own slack rather than a host effect.
    let hosts = [1e-3, 1.0 / 60.0, 0.1];
    let t_end = 0.08;
    let (mut boards, mut worst, mut flips_seen, mut farther) = (0, 0.0f64, 0usize, Vec::new());
    for seed in 0..300u64 {
        let (p, tau) = property_board(seed);
        let fine = flips(&p, 8, &stops(1e-6, t_end));
        flips_seen += fine.len();
        for &h in &hosts {
            let got = flips(&p, 8, &stops(h, t_end));
            crate::rails::force_fixed_cell_band(true);
            let band = flips(&p, 8, &stops(h, t_end));
            crate::rails::force_fixed_cell_band(false);
            let head = share_of(&got, &fine, tau).unwrap_or_else(|e| panic!("board {seed} at host {h:e}: {e}"));
            let base = share_of(&band, &fine, tau);
            assert!(head <= 1.0, "board {seed} at host {h:e}: a flip {head:.3} tau_w0 off the 1 us run (band: {base:?})");
            if base.as_ref().is_ok_and(|&b| head > b + crate::rails::CELL_READING_SLACK) {
                farther.push((seed, h, head, base.clone().unwrap()));
            }
            worst = worst.max(head);
        }
        boards += 1;
    }
    eprintln!("{boards} boards, {flips_seen} flips in the 1 us runs, worst {worst:.4} tau_w0");
    for (seed, h, head, b) in &farther {
        eprintln!("  farther than the band: board {seed} at host {h:e}: {head:.4} against {b:.4} tau_w0");
    }
    assert!(farther.is_empty(), "{} board-cadences farther from the 1 us run than the band", farther.len());
}
