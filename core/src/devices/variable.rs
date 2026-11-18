//! Variable resistive devices and the constant current source (W1h item H1a).
//! This file is the H0 seam: it exists so that the dispatch arms in
//! `devices/mod.rs` and the reader checks in `kinds.rs` never need an edit when
//! H1a lands. Today every function here is a stub that keeps the behaviour the
//! kinds had before they had a shape (stamp nothing, publish `0`), and none of
//! the kinds is in `CORE_IMPLEMENTED_KINDS`, so no compiled program reaches it.
//!
//! H1a owns this file: the stamps (`potentiometer` and `trimmer`: two
//! conductances `cw-wiper` and `wiper-ccw`; `resistor_array`: four isolated
//! `a_i-b_i` resistors; `ldr`: one resistor; `push_dpdt`: two poles of 1 mohm
//! contacts; `current_source`: a `+I`/`-I` injection plus an optional shunt, the
//! one kind whose [`Contribution::injection`] is non-zero without a capacitor),
//! the `elementI` of each, and the reader checks below. The param layouts are
//! fixed in `kinds.rs` (`pot_param`, `array_param`, `ldr_param`, `dpdt_param`,
//! `current_source_param`); a check may be tightened, a layout may not move.

use super::Contribution;
use crate::kinds::{self, array_param, current_source_param, ldr_param, pot_param};

/// The stamp of one of the H1a kinds. A stub: `Vec::new()`, as before the kind
/// had a shape.
pub fn stamp(kind: u32, pin_count: usize, params: &[f64]) -> Vec<Contribution> {
    let _ = (kind, pin_count, params);
    Vec::new()
}

/// The `elementI` of one of the H1a kinds. A stub: `0.0`, as before the kind
/// had a shape.
pub fn current(kind: u32, pin_count: usize, params: &[f64], pin_voltages: &[f64]) -> f64 {
    let _ = (kind, pin_count, params, pin_voltages);
    0.0
}

/// Whether `kind` is one of the kinds this module owns. The dispatch in
/// `devices/mod.rs` calls it so the list lives here, not there.
pub fn handles(kind: u32) -> bool {
    matches!(kind, kinds::POTENTIOMETER | kinds::TRIMMER | kinds::RESISTOR_ARRAY | kinds::LDR | kinds::PUSH_DPDT | kinds::CURRENT_SOURCE)
}

fn positive(what: &str, v: f64) -> Result<(), String> {
    if v > 0.0 {
        Ok(())
    } else {
        Err(format!("{what} must be > 0, got {v}"))
    }
}

fn non_negative(what: &str, v: f64) -> Result<(), String> {
    if v >= 0.0 {
        Ok(())
    } else {
        Err(format!("{what} must be >= 0, got {v}"))
    }
}

/// `potentiometer` and `trimmer` `[rCw, rCcw, pMax]`: classic floors each
/// segment at 1 ohm, so the compiler never writes a non-positive one; `pMax`
/// is 0 for no rating.
pub(crate) fn pot_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    positive("potentiometer rCw", params[pot_param::R_CW])?;
    positive("potentiometer rCcw", params[pot_param::R_CCW])?;
    non_negative("potentiometer pMax", params[pot_param::P_MAX])
}

/// `resistor_array` `[R, pMax]`, per channel.
pub(crate) fn array_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    positive("resistor_array R", params[array_param::R])?;
    non_negative("resistor_array pMax", params[array_param::P_MAX])
}

/// `ldr` `[R]`.
pub(crate) fn ldr_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    positive("ldr R", params[ldr_param::R])
}

/// `current_source` `[current, rParallel]`: any finite `current`; `rParallel`
/// is 0 for no shunt, so a negative one has no encoding.
pub(crate) fn current_source_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    non_negative("current_source rParallel", params[current_source_param::R_PARALLEL])
}
