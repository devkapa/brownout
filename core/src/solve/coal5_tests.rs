//! COAL5: the divider family's seed 47 traced to the instant coalescing at budget 0 first parts from off, and the
//! bound that holds for it. Diagnostics are ignored; they print.

use super::coalv2_tests::{board_with, Rng};
use crate::engine::Engine;
use crate::program::Program;

/// A test that pins the bits of the build without move coalescing says so: for its own duration the thread's budget
/// is the off value (a negative budget, -1.0), so `BROWNOUT_TEST_ACT_EPS` cannot reach it; the previous override is
/// restored when the guard drops.
pub(super) struct OffBudget(Option<f64>);

impl OffBudget {
    pub(super) fn new() -> Self {
        Self(crate::rails::TEST_ACT_EPS.with(|c| c.replace(Some(-1.0))))
    }
}

impl Drop for OffBudget {
    fn drop(&mut self) {
        crate::rails::TEST_ACT_EPS.with(|c| c.set(self.0));
    }
}

/// The divider-family board of `seed` (coalv2_tests::latch_power_on_divider_at_threshold_edges_equal): the chip's
/// input divider calibrated to sit within 20 mV of a threshold at 0.1 s.
pub(super) fn divider_board(seed: u64) -> Program {
    let mut g = Rng(seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1);
    for _ in 0..3 {
        g.next();
    }
    let target = if g.chance(0.5) { 2.0 } else { 0.8 } + g.range(-0.02, 0.02);
    let v2_at = |r2: f64| {
        let Ok(mut e) = Engine::load(&board_with(seed, Some(r2))) else { return f64::NAN };
        e.debug_set_rail_act_eps(-1.0);
        if e.advance(0.1).is_err() {
            return f64::NAN;
        }
        e.voltages()[2]
    };
    let (mut lo, mut hi) = (1e2f64, 1e7f64);
    for _ in 0..40 {
        let mid = (lo * hi).sqrt();
        if v2_at(mid) > target { hi = mid } else { lo = mid }
    }
    board_with(seed, Some((lo * hi).sqrt()))
}

fn edges(p: &Program, eps: f64, h: f64, end: f64) -> Vec<(f64, usize, f64)> {
    let mut e = Engine::load(p).unwrap();
    e.debug_set_rail_act_eps(eps);
    e.trace_enable(true);
    let mut out = Vec::new();
    let mut t = 0.0;
    while t < end - 1e-12 {
        t = (t + h).min(end);
        if e.advance(t).is_err() {
            break;
        }
        for c in e.trace().chunks(3) {
            out.push((c[0], c[1] as usize, c[2]));
        }
        e.trace_clear();
    }
    out
}

#[test]
#[ignore]
fn diag_seed_edges() {
    let seed: u64 = std::env::var("COAL5_SEED").ok().and_then(|x| x.parse().ok()).unwrap_or(47);
    let h: f64 = std::env::var("COAL5_H").ok().and_then(|x| x.parse().ok()).unwrap_or(1.0 / 60.0);
    let end: f64 = std::env::var("COAL5_END").ok().and_then(|x| x.parse().ok()).unwrap_or(0.3);
    let eps: f64 = std::env::var("COAL5_EPS").ok().and_then(|x| x.parse().ok()).unwrap_or(0.0);
    let p = divider_board(seed);
    let off = edges(&p, -1.0, h, end);
    let on = edges(&p, eps, h, end);
    eprintln!("edges off {} on {}", off.len(), on.len());
    for (i, (a, b)) in off.iter().zip(&on).enumerate() {
        eprintln!("{i:3} slot {:2} off {:.12} on {:.12} d {:+.3e} v {} {}", a.1, a.0, b.0, b.0 - a.0, a.2, b.2);
    }
}

fn worst_shift(a: &[(f64, usize, f64)], b: &[(f64, usize, f64)]) -> Option<f64> {
    if a.len() != b.len() {
        return None;
    }
    let (mut sa, mut sb) = (a.to_vec(), b.to_vec());
    let key = |x: &(f64, usize, f64), y: &(f64, usize, f64)| x.1.cmp(&y.1).then(x.0.partial_cmp(&y.0).unwrap());
    sa.sort_by(key);
    sb.sort_by(key);
    let mut w = 0.0f64;
    for (x, y) in sa.iter().zip(&sb) {
        if x.1 != y.1 || !(x.2 == y.2 || (x.2.is_nan() && y.2.is_nan())) {
            return None;
        }
        w = w.max((x.0 - y.0).abs());
    }
    Some(w)
}

/// The divider family's seeds against two baselines: coalescing at budget 0 and 1e-7 against off, and off against
/// off with the cell's capacity changed by one part in 1e15 (roundoff, no coalescing). Prints one line per seed and
/// the worst of each. Env COAL5_N seeds from COAL5_FROM.
#[test]
#[ignore]
fn diag_family_against_roundoff() {
    let from: u64 = std::env::var("COAL5_FROM").ok().and_then(|x| x.parse().ok()).unwrap_or(1);
    let n: u64 = std::env::var("COAL5_N").ok().and_then(|x| x.parse().ok()).unwrap_or(300);
    let h = 1.0 / 60.0;
    let (mut w0, mut w7, mut wn) = (0.0f64, 0.0f64, 0.0f64);
    let (mut bad0, mut bad7, mut badn) = (0, 0, 0);
    for seed in from..from + n {
        let p = divider_board(seed);
        let mut q = p.clone();
        q.devices[0].params[crate::kinds::battery_param::CAPACITY_AH] *= 1.0 + 1e-15;
        let off = edges(&p, -1.0, h, 0.3);
        let (on0, on7, offq) = (edges(&p, 0.0, h, 0.3), edges(&p, 1e-7, h, 0.3), edges(&q, -1.0, h, 0.3));
        let (a, b, c) = (worst_shift(&off, &on0), worst_shift(&off, &on7), worst_shift(&off, &offq));
        eprintln!("SEED {seed} edges {} on0 {a:?} on1e-7 {b:?} roundoff {c:?}", off.len());
        match a { Some(x) => w0 = w0.max(x), None => bad0 += 1 }
        match b { Some(x) => w7 = w7.max(x), None => bad7 += 1 }
        match c { Some(x) => wn = wn.max(x), None => badn += 1 }
    }
    eprintln!("FAMILY seeds {from}..+{n}: worst shift on0 {w0:e} (count/value differences {bad0}), on1e-7 {w7:e} ({bad7}), roundoff-only {wn:e} ({badn})");
}
