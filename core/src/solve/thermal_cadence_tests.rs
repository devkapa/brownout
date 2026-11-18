//! THERM4, ruling R130: thermal readouts on draining cells under the
//! reading-aware rail cadence (R121, R127; core/README.md, Rails point 4 and
//! "Readouts"). THERM3's measurement boards, each run at host stops of 1 ms,
//! 1/60 s and 100 ms against the same board stopped every microsecond (and
//! at every coarse stop), held to R130:
//! - a part's temperature within W0_T = 1e-3 C + 1e-3 of its rise over
//!   ambient on the 1 us run; an LED's junction temperature, the one that
//!   feeds a law, within a tenth of `THERMAL_BAND_C` (1e-3 C);
//! - its power within 1e-3 of itself (+ 1e-9 W);
//! - a re-arm or latch instant within the time its temperature or stress
//!   takes to cross that tolerance at its rate there;
//! - no slow event (re-arm, latch), nor a 555 flip, lost or gained.
//!
//! The bound the cadence implies, by hand (each board's comment applies
//! it): a thermal part's power is no reading of the drift cap, so it is held
//! from the last rail solve while the rail moves up to `d <= RAIL_DRIFT_V +
//! CELL_READING_SLACK W0(r) / |dr/du|` for the binding reading `r`. A
//! resistor carrying a draw `i` (its own, or one in series with it) of
//! conductance `g = |di/du|` sees `|di| <= g 1e-5 + 0.2 (1e-3 i + 1e-9)`, so
//! its held power `R i^2` is off by `2 |di| / i`, about 4e-4 of itself (the
//! 10 uV band: `2e-5 g / i`, about 4e-6 at 5 V). The rail only falls, so the
//! error keeps one sign, and a temperature, a one-pole of the power, is off
//! by up to that fraction of its rise. Every board here sits inside that and
//! inside R130's tolerance; the tests print the measured worst.
//!
//! A re-arm's instant in a coarse run is located by bisection from the
//! engine's snapshot at the coarse stop before it (each probe advances a copy
//! to a midpoint; a later host target does not change what happens before
//! it), to 1e-7 s; the 1 us run's is the stop after its re-arm, so 1 us is
//! added to each allowance. Release only: every board runs to 0.8 to 1.5 s
//! at 1 us stops (about a minute each in release).

use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};
use crate::slow::thermal::THERMAL_BAND_C;

const TA: f64 = 25.0;

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
/// A red LED as the catalog's: `vf` 1.8 V, `n` 2, `iRated` 20 mA.
fn red_led(ci: usize, a: usize, k: usize) -> Device {
    dev(kinds::LED, ci, &[a, k], vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0], vec![])
}
fn thermal(device: usize, r_theta: f64, tau_s: f64) -> Device {
    Device { kind: kinds::THERMAL, component_index: device, pins: vec![], params: vec![device as f64, r_theta, tau_s, 0.0, 0.0, 0.0], blob: vec![] }
}
fn rating(device: usize, p_max: f64, r_power: f64) -> Device {
    Device { kind: kinds::RATING, component_index: device, pins: vec![], params: vec![device as f64, p_max, r_power], blob: vec![] }
}
/// THERM's cell on node 1: OCV 4 V empty to 5 V full, linear in SoC.
fn cell45(cap_ah: f64, rint: f64) -> Device {
    let mut params = vec![0.0; kinds::battery_param::COUNT];
    params[kinds::battery_param::OCV_SCALE] = 1.0;
    params[kinds::battery_param::R_BASE] = rint;
    params[kinds::battery_param::TEMP_FACTOR] = 1.0;
    params[kinds::battery_param::DEP_P] = 2.0;
    params[kinds::battery_param::DEP_K] = 0.05;
    params[kinds::battery_param::CAPACITY_AH] = cap_ah;
    params[kinds::battery_param::SOC0] = 1.0;
    Device { kind: kinds::BATTERY_PACK, component_index: 0, pins: vec![pin(1), pin(0)], params, blob: vec![2.0, 0.0, 4.0, 1.0, 5.0] }
}
/// `crossing_tests`' Li-ion-like cell (3.41 to 4.88 V) on node 1.
fn cell_li(rint: f64, cap_ah: f64) -> Device {
    dev(kinds::BATTERY_PACK, 0, &[1, 0], vec![1.0, rint, 1.0, 0.0, 2.0, 0.05, cap_ah, 1.0, 0.0], vec![2.0, 0.0, 3.4074, 1.0, 4.8837])
}
fn n555(ci: usize, trig: usize, out: usize, ctrl: usize, thr: usize, dis: usize, slot: usize) -> Device {
    dev(kinds::NE555, ci, &[1, 0, trig, out, 1, ctrl, thr, dis], vec![2.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], vec![slot as f64, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
}
fn prog(n: usize, devices: Vec<Device>) -> Program {
    Program { carry: Vec::new(), node_count: n, options: Options::default(), digital_slot_count: 64, channel_slot_count: 0, devices }
}

/// A watched identified part: its device, whether it is an LED (its junction
/// temperature feeds its law), and its `R_theta`, `tau`.
#[derive(Clone, Copy)]
struct Part {
    device: u32,
    led: bool,
    r_theta: f64,
    tau: f64,
}

struct Board {
    name: &'static str,
    program: Program,
    end: f64,
    parts: Vec<Part>,
    /// The rated device that latches, with its `pMax`.
    latch: Option<(u32, f64)>,
    /// The 555's output slot, whose flips are counted.
    slot: Option<usize>,
}

/// What one run read: `[T, P]` per part and the LED currents at the kept
/// stops, the re-arm instants (all LEDs together, in order: stat 24) with
/// their rate `dT/dt` where known, the latch instant and the flips.
struct Run {
    at: Vec<(f64, Vec<[f64; 2]>, Vec<f64>)>,
    rearms: Vec<(f64, f64)>,
    latch: Vec<f64>,
    /// `integral of P / pMax dt` up to the latch (the 1 us run only).
    q_integral: f64,
    flips: usize,
}

fn grid(h: f64, end: f64) -> Vec<f64> {
    let n = (end / h - 1e-9).ceil() as usize;
    (1..=n).map(|k| (k as f64 * h).min(end)).collect()
}

const HOSTS: [f64; 3] = [1e-3, 1.0 / 60.0, 0.1];

fn read(e: &mut Engine, b: &Board) -> (Vec<[f64; 2]>, Vec<f64>) {
    let mut tp = Vec::new();
    for p in &b.parts {
        let s = e.device_state(p.device).unwrap();
        tp.push([s[0], s[1]]);
    }
    let leds = b.parts.iter().filter(|p| p.led).map(|p| e.currents()[p.device as usize]);
    (tp, leds.collect())
}

fn latches(e: &mut Engine, b: &Board) -> Vec<f64> {
    let Some((d, _)) = b.latch else { return Vec::new() };
    e.records(0).unwrap().chunks(7).filter(|c| c[0] == d as f64).map(|c| c[3]).collect()
}

fn flips(e: &mut Engine, slot: Option<usize>) -> usize {
    let Some(s) = slot else { return 0 };
    let n = e.trace().chunks(3).filter(|ch| ch[1] as usize == s).count();
    e.trace_clear();
    n
}

/// The reference: stops every 1 us and at every coarse host's stops, read
/// at the coarse ones; each re-arm at the stop after it, with the LED's
/// one-pole rate there, `(Ta + R_theta P - T) / tau`.
fn fine_run(b: &Board, gate: bool) -> Run {
    let mut coarse: Vec<f64> = HOSTS.iter().flat_map(|&h| grid(h, b.end)).collect();
    coarse.sort_by(f64::total_cmp);
    coarse.dedup();
    let near = |t: f64| {
        let i = coarse.partition_point(|&x| x < t);
        [i.wrapping_sub(1), i].iter().any(|&j| coarse.get(j).is_some_and(|&x| (x - t).abs() < 1e-12))
    };
    let mut stops: Vec<f64> = grid(1e-6, b.end).into_iter().filter(|&t| !near(t)).chain(coarse.iter().copied()).collect();
    stops.sort_by(f64::total_cmp);
    let mut e = load(b, gate);
    e.trace_enable(b.slot.is_some());
    let led = b.parts.iter().position(|p| p.led);
    let latch_k = b.latch.and_then(|(d, _)| b.parts.iter().position(|p| p.device == d));
    let (mut at, mut rearms, mut q_integral, mut n_flips) = (Vec::new(), Vec::new(), 0.0f64, 0usize);
    let (mut count, mut t_prev) = (e.stat(24), 0.0f64);
    for &t in &stops {
        e.advance(t).unwrap();
        n_flips += flips(&mut e, b.slot);
        let (tp, cur) = read(&mut e, b);
        if let (Some((_, p_max)), Some(k)) = (b.latch, latch_k) {
            if latches(&mut e, b).is_empty() {
                q_integral += tp[k][1] / p_max * (t - t_prev);
            }
        }
        let now = e.stat(24);
        if now > count {
            let p = &b.parts[led.expect("only an LED re-arms")];
            let [temp, power] = tp[led.unwrap()];
            let rate = (TA + p.r_theta * power - temp) / p.tau;
            for _ in 0..(now - count) as usize {
                rearms.push((t, rate));
            }
            count = now;
        }
        if coarse.binary_search_by(|x| x.total_cmp(&t)).is_ok() {
            at.push((t, tp, cur));
        }
        t_prev = t;
    }
    Run { at, rearms, latch: latches(&mut e, b), q_integral, flips: n_flips }
}

/// Finds the re-arms between `a` (where `snap` stands, `ca` re-armed so far)
/// and `b` (`cb`) by bisection on copies of `snap`, to 1e-7 s.
fn locate(snap: &Engine, a: f64, b: f64, ca: f64, cb: f64, out: &mut Vec<f64>) {
    if cb <= ca {
        return;
    }
    if b - a < 1e-7 {
        out.extend(std::iter::repeat_n(b, (cb - ca) as usize));
        return;
    }
    let mid = 0.5 * (a + b);
    let mut x = snap.snapshot();
    x.advance(mid).unwrap();
    let cm = x.stat(24);
    locate(snap, a, mid, ca, cm, out);
    locate(snap, mid, b, cm, cb, out);
}

/// The board loaded, with R132's thermal voltage gate on (the build's
/// default) or switched off.
fn load(b: &Board, gate: bool) -> Engine {
    let mut e = Engine::load(&b.program).unwrap();
    if !gate {
        e.debug_thermal_gate_off();
    }
    e
}

fn coarse_run(b: &Board, h: f64) -> Run {
    let mut e = load(b, true);
    e.trace_enable(b.slot.is_some());
    let has_led = b.parts.iter().any(|p| p.led);
    let (mut at, mut instants, mut n_flips) = (Vec::new(), Vec::new(), 0usize);
    for t in grid(h, b.end) {
        let (from, before) = (e.time(), e.stat(24));
        let snap = has_led.then(|| e.snapshot());
        e.advance(t).unwrap();
        n_flips += flips(&mut e, b.slot);
        if let Some(snap) = snap {
            locate(&snap, from, t, before, e.stat(24), &mut instants);
        }
        let (tp, cur) = read(&mut e, b);
        at.push((t, tp, cur));
    }
    Run { at, rearms: instants.into_iter().map(|t| (t, f64::NAN)).collect(), latch: latches(&mut e, b), q_integral: f64::NAN, flips: n_flips }
}

/// R130's check of one board at the three host cadences; returns the fine
/// run for the board's own hand checks.
fn meets_r130(b: &Board) -> Run {
    meets_r130_against(b, true)
}

/// [`meets_r130`] with the 1 us reference run with R132's thermal voltage
/// gate on (`gate`, as the build runs) or off: off, it holds the gated
/// build at every host cadence to R130 against the ungated truth (R132's
/// condition 3).
fn meets_r130_against(b: &Board, gate: bool) -> Run {
    let fine = fine_run(b, gate);
    for h in HOSTS {
        let g = coarse_run(b, h);
        let mut line = format!("{} {} h={h:.6}", if gate { "THERM4" } else { "R132 gate vs ungated" }, b.name);
        // The latch window: a stop between the two runs' latch instants reads
        // one part latched and the other not, by the instant's own allowance.
        let window = match (g.latch.first(), fine.latch.first()) {
            (Some(&x), Some(&y)) => (x.min(y), x.max(y)),
            _ => (f64::INFINITY, f64::INFINITY),
        };
        for (k, p) in b.parts.iter().enumerate() {
            let (mut te, mut tr, mut pr) = (0.0f64, 0.0f64, 0.0f64);
            for (gs, fs) in g.at.iter().map(|x| (x.0, &x.1[k])).zip(fine.at.iter().filter(|x| grid_has(h, b.end, x.0)).map(|x| &x.1[k])) {
                let (t, gs) = gs;
                let rise = (fs[0] - TA).abs();
                let tol_t = if p.led { THERMAL_BAND_C / 10.0 } else { 1e-3 + 1e-3 * rise };
                let dt = (gs[0] - fs[0]).abs();
                assert!(dt <= tol_t, "{}: T of device {} at {t} s, stops of {h} s: {} against {} ({dt:e} C, allowed {tol_t:e})", b.name, p.device, gs[0], fs[0]);
                te = te.max(dt);
                if rise > 1e-3 {
                    tr = tr.max(dt / rise);
                }
                let in_window = b.latch.is_some_and(|(d, _)| d == p.device) && t >= window.0 && t < window.1;
                if !in_window {
                    let dp = (gs[1] - fs[1]).abs();
                    let tol_p = 1e-3 * fs[1].abs() + 1e-9;
                    assert!(dp <= tol_p, "{}: P of device {} at {t} s, stops of {h} s: {} against {} ({dp:e} W, allowed {tol_p:e})", b.name, p.device, gs[1], fs[1]);
                    if fs[1].abs() > 1e-9 {
                        pr = pr.max(dp / fs[1].abs());
                    }
                }
            }
            line += &format!(" | d{} T {te:.2e} C ({tr:.2e} of rise) P {pr:.2e} rel", p.device);
        }
        let fine_at: Vec<_> = fine.at.iter().filter(|x| grid_has(h, b.end, x.0)).collect();
        assert_eq!(g.at.len(), fine_at.len(), "{}: stops of {h} s", b.name);
        let leds = b.parts.iter().filter(|p| p.led).count();
        for j in 0..leds {
            let worst = g.at.iter().zip(&fine_at).map(|(a, f)| ((a.2[j] - f.2[j]) / f.2[j]).abs()).fold(0.0f64, f64::max);
            line += &format!(" | I_led {worst:.2e} rel");
        }
        // Re-arms: the same count, each within `1e-3 C / |dT/dt|` of the
        // 1 us run's (plus its 1 us resolution).
        assert_eq!(g.rearms.len(), fine.rearms.len(), "{}: re-arms at stops of {h} s", b.name);
        if !fine.rearms.is_empty() {
            let (mut worst, mut use_) = (0.0f64, 0.0f64);
            for (j, (a, f)) in g.rearms.iter().zip(&fine.rearms).enumerate() {
                let dt = (a.0 - f.0).abs();
                let allowed = THERMAL_BAND_C / 10.0 / f.1.abs() + 1e-6;
                assert!(dt <= allowed, "{}: re-arm {j} at stops of {h} s: {} against {} ({dt:e} s, allowed {allowed:e})", b.name, a.0, f.0);
                worst = worst.max(dt);
                use_ = use_.max(dt / allowed);
            }
            line += &format!(" | re-arms {} worst {worst:.2e} s ({use_:.2} of allowed)", g.rearms.len());
        }
        // A latch: once in each, within `1e-3 integral q dt / (q - 1)` at
        // its instant (the stress `S' = q - 1`, `q = P / pMax`, off by at
        // most 1e-3 of `q` at every instant).
        assert_eq!(g.latch.len(), fine.latch.len(), "{}: latches at stops of {h} s", b.name);
        if let (Some(&a), Some(&f), Some((d, p_max))) = (g.latch.first(), fine.latch.first(), b.latch) {
            let k = b.parts.iter().position(|p| p.device == d).unwrap();
            let last = fine.at.iter().filter(|x| x.0 <= f).last().expect("a stop before the latch");
            let q = last.1[k][1] / p_max;
            let allowed = 1e-3 * fine.q_integral / (q - 1.0) + 1e-6;
            let dt = (a - f).abs();
            assert!(dt <= allowed, "{}: latch at stops of {h} s: {a} against {f} ({dt:e} s, allowed {allowed:e})", b.name);
            line += &format!(" | latch {a:.6} s against {f:.6} s ({dt:.2e} s, allowed {allowed:.2e})");
        }
        if b.slot.is_some() {
            assert_eq!(g.flips, fine.flips, "{}: flips at stops of {h} s", b.name);
            line += &format!(" | flips {}", g.flips);
        }
        eprintln!("{line}");
    }
    fine
}

fn grid_has(h: f64, end: f64, t: f64) -> bool {
    let g = grid(h, end);
    g.binary_search_by(|x| x.total_cmp(&t)).is_ok()
}

/// A 240 ohm over 10 ohm from the cell (rint 0.1 ohm), identified (340 K/W,
/// tau 0.5 s) and rated 0.125 W, beside 10 kohm into 10 uF (a closed-form
/// partition, which puts the rail on the cell's coupled path).
fn hot(cap_ah: f64) -> Board {
    Board {
        name: if cap_ah > 1e-4 { "hot-slow" } else { "hot-fast" },
        program: prog(4, vec![cell45(cap_ah, 0.1), r(1, 1, 2, 240.0), r(2, 2, 0, 10.0), r(3, 1, 3, 1e4), c(4, 3, 0, 1e-5), thermal(1, 340.0, 0.5), rating(1, 0.125, 240.0)]),
        end: 1.5,
        parts: vec![Part { device: 1, led: false, r_theta: 340.0, tau: 0.5 }],
        latch: None,
        slot: None,
    }
}

/// 240 ohm at 5 V over 250.1 ohm: `P = 240 (5 / 250.1)^2` = 0.0959 W.
fn first_power(fine: &Run, k: usize) -> f64 {
    fine.at[0].1[k][1]
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn a_hot_resistor_on_a_slow_drain_reads_within_r130_at_any_host_stop() {
    // By hand: 0.0959 W at the start (above), a 1 mAh cell at 20 mA falls 1 V
    // in 180 s, 5.6 mV/s, so the power stays within 0.4 % over 1.5 s and the
    // rise is `340 P (1 - e^(-3))` = 31.0 C. The held power's bound
    // (module doc): `|di|/i <= 1e-5 / (250 x 0.02) + 2e-4` = 2.02e-4, the
    // power 4.0e-4, so T within 4.0e-4 x 31 = 0.012 C of the 1 us run's,
    // inside R130's 1e-3 + 0.031 C. THERM3 measured 3.5e-3 C (1.1e-4 of the
    // rise) at 100 ms hosts, 6.2e-5 C on the 10 uV band.
    let fine = meets_r130(&hot(1e-3));
    let p0 = 240.0 * (5.0f64 / 250.1).powi(2);
    assert!((first_power(&fine, 0) - p0).abs() < 2e-3 * p0, "P at 1 ms: {} against {p0}", first_power(&fine, 0));
    let rise = fine.at.last().unwrap().1[0][0] - TA;
    let want = 340.0 * p0 * (1.0 - (-3.0f64).exp());
    assert!((rise - want).abs() < 5e-3 * want, "the rise at 1.5 s: {rise} against {want}");
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn a_hot_resistor_on_a_fast_drain_reads_within_r130_at_any_host_stop() {
    // By hand: the same 0.0959 W at the start; a 10 uAh cell at 20 mA falls
    // 1 V in 1.8 s, 0.56 V/s, so by 1.5 s the rail is near 4.17 V and the
    // power `P0 (V/5)^2` near 0.067 W. The held power's bound is the slow
    // drain's, 4.0e-4 of it: T within 4.0e-4 x 25 C = 0.010 C, inside
    // R130's 1e-3 + 0.025 C. THERM3 measured 4.9e-3 C (2.0e-4 of the rise),
    // 4.7e-5 C on the 10 uV band.
    let fine = meets_r130(&hot(10e-6));
    let p0 = 240.0 * (5.0f64 / 250.1).powi(2);
    assert!((first_power(&fine, 0) - p0).abs() < 2e-3 * p0, "P at 1 ms: {} against {p0}", first_power(&fine, 0));
    let p_end = fine.at.last().unwrap().1[0][1];
    assert!((p_end - 0.067).abs() < 0.05 * 0.067, "P at 1.5 s: {p_end} against 0.067 W");
}

/// A red LED through 150 ohm from the cell (rint 0.1 ohm), both identified
/// (the LED 560 K/W, tau 2 s; the resistor 340 K/W, 1 s), beside 10 kohm
/// into 10 uF.
fn led(cap_ah: f64) -> Board {
    Board {
        name: if cap_ah > 1e-4 { "led-slow" } else { "led-fast" },
        program: prog(4, vec![cell45(cap_ah, 0.1), r(1, 1, 2, 150.0), red_led(2, 2, 0), r(3, 1, 3, 1e4), c(4, 3, 0, 1e-5), thermal(1, 340.0, 1.0), thermal(2, 560.0, 2.0)]),
        end: 1.5,
        parts: vec![Part { device: 1, led: false, r_theta: 340.0, tau: 1.0 }, Part { device: 2, led: true, r_theta: 560.0, tau: 2.0 }],
        latch: None,
        slot: None,
    }
}

/// The LED's cold operating point by hand: `i = (5 - V) / 150.1`, `V = 1.8 +
/// 2 vt ln(i / 0.02)`; at `i` 21.3 mA, `V` 1.803 V, so the resistor takes
/// 0.068 W and the LED 0.0384 W.
fn led_hand(fine: &Run) {
    let (pr, pl) = (fine.at[0].1[0][1], fine.at[0].1[1][1]);
    assert!((pr - 0.068).abs() < 0.01 * 0.068, "the resistor's power at 1 ms: {pr}");
    assert!((pl - 0.0384).abs() < 0.01 * 0.0384, "the LED's power at 1 ms: {pl}");
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn an_led_at_20_ma_on_a_slow_drain_re_arms_within_r130_at_any_host_stop() {
    // By hand (`led_hand`), and the LED's rise `560 x 0.0384 (1 - e^(-0.75))`
    // = 11.3 C at 1.5 s on a rail that falls 5.6 mV/s: it rises the whole
    // run, so it re-arms once per 0.01 C, `floor(rise / 0.01)` times. The
    // held bound: `|di|/i <= 1e-5 / (150 x 0.021) + 2e-4` = 2.03e-4, the
    // resistor's power 4.1e-4 (T within 0.0074 C of its 18 C rise) and the
    // LED's 2.1e-4 (`(1 + 2 vt / V)`), whose junction then needs `2.1e-4 x
    // 11.3 C` = 2.4e-3 C held over the whole run: past R130's 1e-3 C on the
    // bound alone, so this board shows what is measured. THERM3: 1.5e-5 C on
    // the LED, re-arms within 3.0e-6 s (the band: 2.9e-6 s).
    let fine = meets_r130(&led(1e-3));
    led_hand(&fine);
    let rise = fine.at.last().unwrap().1[1][0] - TA;
    assert!((rise - 560.0 * 0.0384 * (1.0 - (-0.75f64).exp())).abs() < 0.02 * rise, "the LED's rise at 1.5 s: {rise}");
    assert_eq!(fine.rearms.len(), (rise / THERMAL_BAND_C).floor() as usize, "re-arms over a {rise} C rise");
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn an_led_at_20_ma_on_a_fast_drain_re_arms_within_r130_at_any_host_stop() {
    // By hand (`led_hand`), and the rail falling 0.56 V/s (10 uAh at 21 mA):
    // by 1.5 s near 4.15 V, the current `(4.15 - 1.8) / 150` = 15.7 mA, so
    // the LED heats less and turns down before the end (its re-arms count
    // its travel both ways). The held bound is the slow drain's: 2.1e-4 of
    // the LED's rise, 2.0e-3 C on a 9.7 C rise; R130 asks 1e-3 C, and
    // THERM3 measured 8.9e-4 C at 100 ms hosts (1.4e-5 C on the band), with
    // 969 re-arms equal and the worst 2.7e-4 s off (4.4e-6 s on the band).
    let fine = meets_r130(&led(10e-6));
    led_hand(&fine);
    let i_end = fine.at.last().unwrap().2[0];
    assert!((i_end - 0.0157).abs() < 0.05 * 0.0157, "the LED's current at 1.5 s: {i_end}");
}

/// A 96 ohm over 4 ohm from the cell (rint 0.1 ohm), identified (340 K/W,
/// tau 1 s) and rated 0.1 W (`rPower` 96), beside 10 kohm into 10 uF.
fn latching(cap_ah: f64) -> Board {
    Board {
        name: if cap_ah > 1e-4 { "latch-slow" } else { "latch-fast" },
        program: prog(4, vec![cell45(cap_ah, 0.1), r(1, 1, 2, 96.0), r(2, 2, 0, 4.0), r(3, 1, 3, 1e4), c(4, 3, 0, 1e-5), thermal(1, 340.0, 1.0), rating(1, 0.1, 96.0)]),
        end: 1.5,
        parts: vec![Part { device: 1, led: false, r_theta: 340.0, tau: 1.0 }],
        latch: Some((1, 0.1)),
        slot: None,
    }
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn a_rated_resistor_on_a_slow_drain_latches_once_within_r130_at_any_host_stop() {
    // By hand: `P = 96 (5 / 100.1)^2` = 0.2395 W, `q = P / pMax` = 2.395, the
    // stress `S' = q - 1` reaches 1 at `1 / 1.395` = 0.717 s (the rail's
    // 4 mV fall by then delays it 0.3 %). The held power's bound, 4.0e-4 of
    // `q`, moves that by `4e-4 q t / (q - 1)` = 4.9e-4 s at most; R130 allows
    // `1e-3 integral q dt / (q - 1)` = 1.2e-3 s. THERM3: 1.75e-4 s at 100 ms
    // hosts (2.5e-6 s on the band), T within 7.5e-3 C of a 41 C rise.
    let fine = meets_r130(&latching(1e-3));
    let q = 96.0 * (5.0f64 / 100.1).powi(2) / 0.1;
    let want = 1.0 / (q - 1.0);
    let got = fine.latch[0];
    assert!(got > want && got < 1.005 * want, "the latch at {got} s against {want} s on a steady rail");
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn a_rated_resistor_on_a_fast_drain_latches_once_within_r130_at_any_host_stop() {
    // By hand: the same 2.395 start, the rail falling 0.56 V/s (25 uAh at
    // 50 mA), so `q = 2.395 (1 - 0.112 t)^2` and `S = integral (q - 1)`
    // reaches 1 near 0.845 s (by Simpson's rule on that line), where `q` is
    // down to 1.96 and `integral q dt` is 1.8. The held power's bound moves
    // it by `4e-4 x 1.8 / 0.96` = 7.5e-4 s at most; R130 allows `1e-3 x 1.8 /
    // 0.96` = 1.9e-3 s. THERM3: 3.6e-4 s early at 100 ms hosts (3.5e-6 s on
    // the band), T within 0.016 C of a 41 C rise.
    let fine = meets_r130(&latching(25e-6));
    let got = fine.latch[0];
    assert!((got - 0.845).abs() < 0.01, "the latch at {got} s against 0.845 s by hand");
}

#[test]
#[cfg_attr(debug_assertions, ignore = "0.8 s of a 555 against a 1 us run: release only")]
fn a_555_switching_a_resistor_and_an_led_on_a_fast_drain_reads_within_r130_at_any_host_stop() {
    // A 555 astable (RA 2438, RB 1014 ohm, C 68 nF: `T = 0.693 (RA + 2 RB) C`
    // = 0.210 ms, 7,610 flips in 0.8 s) switching 150 ohm and a red LED on
    // its output, both identified (340 K/W, 1 s; 560 K/W, 2 s), on an 8 uAh,
    // 0.148 ohm Li cell (about 0.7 V/s). The held power steps every flip,
    // and every flip solves the rail, so the cadence holds it little longer
    // than the band does: THERM3 measured 2.5e-5 C (4.9e-6 of the rise), the
    // band 4.5e-4 C; flips 7,570 at every cadence, 2.4e-9 s apart at worst.
    let fine = meets_r130(&astable());
    let want = 0.8 / (0.693 * (2438.0 + 2.0 * 1014.0) * 6.8e-8) * 2.0;
    assert!((fine.flips as f64 - want).abs() < 0.02 * want, "{} flips against {want} by hand", fine.flips);
}

/// THERM4's 555 board (its test above).
fn astable() -> Board {
    Board {
        name: "555",
        program: prog(7, vec![cell_li(0.148, 8e-6), n555(2, 3, 4, 6, 3, 2, 8), r(3, 1, 2, 2438.0), r(4, 2, 3, 1014.0), c(5, 3, 0, 6.8e-8), c(6, 6, 0, 1e-8), r(7, 4, 5, 150.0), red_led(8, 5, 0), thermal(6, 340.0, 1.0), thermal(7, 560.0, 2.0)]),
        end: 0.8,
        parts: vec![Part { device: 6, led: false, r_theta: 340.0, tau: 1.0 }, Part { device: 7, led: true, r_theta: 560.0, tau: 2.0 }],
        latch: None,
        slot: Some(8),
    }
}

// ---- R132: the thermal voltage gate against the ungated build ----------
//
// Each board above, run with the gate (as the build runs) at host stops of
// 1 ms, 1/60 s and 100 ms, against the same board stopped every 1 us with
// the gate switched off (`Engine::debug_thermal_gate_off`): R130's
// tolerance, every re-arm, latch and flip kept. The gate's own share of
// each reading's error is at most half of R130's (`thermal_gate_width`),
// beside the cadence's (module doc), and each line prints the measured
// worst.

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn the_thermal_gate_keeps_a_hot_resistor_on_a_slow_drain_within_r130_of_the_ungated_run() {
    meets_r130_against(&hot(1e-3), false);
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn the_thermal_gate_keeps_a_hot_resistor_on_a_fast_drain_within_r130_of_the_ungated_run() {
    meets_r130_against(&hot(10e-6), false);
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn the_thermal_gate_keeps_an_led_on_a_slow_drain_within_r130_of_the_ungated_run() {
    meets_r130_against(&led(1e-3), false);
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn the_thermal_gate_keeps_an_led_on_a_fast_drain_within_r130_of_the_ungated_run() {
    meets_r130_against(&led(10e-6), false);
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn the_thermal_gate_keeps_a_latching_resistor_on_a_slow_drain_within_r130_of_the_ungated_run() {
    meets_r130_against(&latching(1e-3), false);
}

#[test]
#[cfg_attr(debug_assertions, ignore = "1.5 s against a 1 us run: release only")]
fn the_thermal_gate_keeps_a_latching_resistor_on_a_fast_drain_within_r130_of_the_ungated_run() {
    meets_r130_against(&latching(25e-6), false);
}

#[test]
#[cfg_attr(debug_assertions, ignore = "0.8 s of a 555 against a 1 us run: release only")]
fn the_thermal_gate_keeps_the_555_board_within_r130_of_the_ungated_run() {
    meets_r130_against(&astable(), false);
}

/// R132's fast-draining board: a 10 uAh, 0.148 ohm Li cell (4.88 V full)
/// feeding a 555 astable (RA 2438, RB 1014 ohm, 68 nF) whose output drives
/// 150 ohm and a red LED, a second red LED through 330 ohm from the rail and
/// a 220 ohm load across it; every resistor and LED identified as the
/// catalog's are (340 K/W, the LEDs 560 K/W).
fn fast_catalog() -> Board {
    Board {
        name: "fast-catalog",
        program: prog(
            8,
            vec![
                cell_li(0.148, 10e-6),
                n555(1, 3, 4, 6, 3, 2, 8),
                r(2, 1, 2, 2438.0),
                r(3, 2, 3, 1014.0),
                c(4, 3, 0, 6.8e-8),
                c(5, 6, 0, 1e-8),
                r(6, 4, 5, 150.0),
                red_led(7, 5, 0),
                r(8, 1, 7, 330.0),
                red_led(9, 7, 0),
                r(10, 1, 0, 220.0),
                thermal(6, 340.0, 1.0),
                thermal(7, 560.0, 2.0),
                thermal(8, 340.0, 1.0),
                thermal(9, 560.0, 2.0),
                thermal(10, 340.0, 0.5),
            ],
        ),
        end: 0.5,
        parts: vec![
            Part { device: 6, led: false, r_theta: 340.0, tau: 1.0 },
            Part { device: 7, led: true, r_theta: 560.0, tau: 2.0 },
            Part { device: 8, led: false, r_theta: 340.0, tau: 1.0 },
            Part { device: 9, led: true, r_theta: 560.0, tau: 2.0 },
            Part { device: 10, led: false, r_theta: 340.0, tau: 0.5 },
        ],
        latch: None,
        slot: Some(8),
    }
}

#[test]
#[cfg_attr(debug_assertions, ignore = "0.5 s of a 555 against a 1 us run: release only")]
fn the_thermal_gate_keeps_a_fast_draining_catalog_board_within_r130_of_the_ungated_run() {
    // By hand: the draw is the 220 ohm load (4.8 / 220 = 22 mA), the LED
    // through 330 ohm ((4.8 - 1.8) / 330 = 9 mA), the 555's output LED half
    // the time ((4.8 - 1.9) / 150 = 19 mA, 10 mA on average) and its timing
    // network (under 2 mA): about 43 mA, so the 10 uAh (36 mC) cell empties
    // in about 0.8 s and its OCV falls some 1.4 V over it, near 1.8 V/s: the
    // rail slews fast, as R132 asks. The astable flips every half period,
    // `0.693 (RA + 2 RB) C` = 0.210 ms, 4,760 times in 0.5 s; the LEDs re-arm
    // as they heat (their rises some 5 C, 500 re-arms or so).
    let fine = meets_r130_against(&fast_catalog(), false);
    let want = 0.5 / (0.693 * (2438.0 + 2.0 * 1014.0) * 6.8e-8) * 2.0;
    assert!((fine.flips as f64 - want).abs() < 0.03 * want, "{} flips against {want} by hand", fine.flips);
    assert!(fine.rearms.len() > 100, "{} re-arms: the board must exercise the LEDs' slow events", fine.rearms.len());
    let fall = fine.at[0].1[4][1] - fine.at.last().unwrap().1[4][1];
    assert!(fall > 0.0, "the 220 ohm load's power falls with the rail: {fall} W");
}
