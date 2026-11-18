//! `signal_gen` waveform evaluation (W1h item H3a). This file is the H0 seam:
//! it holds the reader check `kinds.rs` points at, so H3a can tighten it without
//! editing `kinds.rs`, and nothing else yet. H3a adds here `signalGenVoltage` for
//! the eight shapes, the FNV-1a noise hash and its Box-Muller variate (Q-O71's
//! 4-ulp rule), and the event schedules computed from the edge index (Q-O68).
//! The solver integration (the Norton drive channel, the ideal form, the sine
//! time function) is H3b, in the F5-owned solver files, after F5 lands.
//!
//! Param layout: `kinds::signal_gen_param`. The compiler calls classic's
//! `parseSignalGenParams` itself, so every clamp below is classic's, repeated
//! for a hand-built program.

use crate::kinds::signal_gen_param::*;

fn whole(v: f64) -> bool {
    v.fract() == 0.0
}

/// The params and the PWL blob of a `signal_gen` record. The blob is
/// `[n, t0, v0, ..., t(n-1), v(n-1)]` with `n <= PWL_MAX_POINTS`, times
/// non-decreasing, and `n = 0` unless `waveform` is `pwl`, so a record has one
/// encoding per value.
pub(crate) fn signal_gen_check(_: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    let shape = params[WAVEFORM];
    if !(whole(shape) && (0.0..=SHAPE_NOISE as f64).contains(&shape)) {
        return Err(format!("signal_gen waveform must be an integer in [0, {SHAPE_NOISE}], got {shape}"));
    }
    if params[AMPLITUDE] < 0.0 {
        return Err(format!("signal_gen amplitude must be >= 0, got {}", params[AMPLITUDE]));
    }
    if !(params[FREQUENCY] >= 1e-6) {
        return Err(format!("signal_gen frequency must be >= 1e-6, got {}", params[FREQUENCY]));
    }
    if !(0.01..=0.99).contains(&params[DUTY]) {
        return Err(format!("signal_gen duty must be in [0.01, 0.99], got {}", params[DUTY]));
    }
    if params[DELAY] < 0.0 {
        return Err(format!("signal_gen delay must be >= 0, got {}", params[DELAY]));
    }
    for (name, v) in [("tr", params[TR]), ("tf", params[TF]), ("pw", params[PW])] {
        if !(v >= 1e-9) {
            return Err(format!("signal_gen {name} must be >= 1e-9, got {v}"));
        }
    }
    if !whole(params[SEED]) {
        return Err(format!("signal_gen seed must be an integer, got {}", params[SEED]));
    }
    if params[ENABLED] != 0.0 && params[ENABLED] != 1.0 {
        return Err(format!("signal_gen enabled must be 0 or 1, got {}", params[ENABLED]));
    }

    let n = blob[0];
    if !(whole(n) && (0.0..=PWL_MAX_POINTS as f64).contains(&n)) {
        return Err(format!("signal_gen PWL point count must be an integer in [0, {PWL_MAX_POINTS}], got {n}"));
    }
    let n = n as usize;
    if blob.len() != 1 + 2 * n {
        return Err(format!("signal_gen PWL blob must hold 1 + 2 * {n} values, got {}", blob.len()));
    }
    if n > 0 && params[WAVEFORM] != SHAPE_PWL as f64 {
        return Err(format!("signal_gen PWL points are only for the pwl shape, got {n} with waveform {shape}"));
    }
    let times = blob[1..].iter().step_by(2);
    if times.clone().zip(times.skip(1)).any(|(a, b)| b < a) {
        return Err("signal_gen PWL times must be non-decreasing".to_string());
    }
    Ok(())
}
