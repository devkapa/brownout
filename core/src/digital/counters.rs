//! The counters `cd4017` (decade counter, code 75) and `cd4060` (14-stage ripple
//! counter with oscillator, code 76): W1h item H2b. This file is the H0 seam,
//! as `shift.rs` is for the shift registers: record layout, delay-pair order and
//! the reader checks now, the states and evaluators in H2b, [`handles`] `false`
//! until then.
//!
//! Classic: `digital-ics.ts:755-811` and `885-1047`, spec 4.2.9 and 4.2.10.
//! Rulings that bind H2b: Q-O11 (an open `reset` is hashed 50/50, as classic's
//! plain `logicHigh` reads it), Q-O60 (the 4060's `rtc`/`ctc` phase as the code
//! drives it: `rtc = !RS`, `ctc = RS`), Q-O61 (the 4017 ignores an edge on
//! `clkinh`), R56 10 (datasheet delays).
//!
//! The one new IC mechanism is the 4060's Schmitt read of `clk_in`: params 7-9
//! carry the thresholds (the catalog's `v_t_plus`/`v_t_minus`, scaled by
//! `Vs / nominal`) and apply to that pin only; every other pin reads plainly.
//! `devices::ics` reads a sequential IC's pins without hysteresis state today
//! (`levels::read_plain_into`, or a throwaway scratch bool when `schmitt` is
//! set), so H2b needs one hunk in that F5-owned file in Phase B; see the W1h
//! contract draft.
//!
//! TRAP for H2b: `icPinMap("cd4060")` in classic's `digital.ts` lists the
//! unpinned stages' ids `q3 q2 q1` where the catalog and classic's own stamp
//! use `q9 q8 q10`; the record below follows the catalog.

use super::{FamilyOutput, IcRecord};
use crate::kinds;

/// Flip to `matches!(kind, kinds::IC_CD4017 | kinds::IC_CD4060)` when the
/// evaluators land (H2b).
pub fn handles(kind: u32) -> bool {
    let _ = kind;
    false
}

/// `cd4017` logic pins after `vcc`, `gnd`: `clk clkinh reset` (read), then
/// `q0 .. q9 co` (driven). Digital slots: `<id>` (the count, a bare key) then
/// the eleven driven pins, 12 in all.
pub const PINS_CD4017: [&str; 14] = ["clk", "clkinh", "reset", "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "co"];
pub const OUT_CD4017: [usize; 11] = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13];

/// `cd4060` logic pins: `clk_in reset` (read; `clk_in` through the Schmitt),
/// `q4 q5 q6 q7 q8 q9 q10 q12 q13 q14` (driven, stage `n` is count bit `n - 1`;
/// stages 1-3 and 11 have no pin), then `rtc ctc` (driven). Digital slots: `<id>`
/// (the count) then the twelve driven pins, 13 in all.
pub const PINS_CD4060: [&str; 14] = ["clk_in", "reset", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q12", "q13", "q14", "rtc", "ctc"];
pub const OUT_CD4060: [usize; 12] = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13];
/// The index of `clk_in` in [`PINS_CD4060`], the only pin read through the Schmitt.
pub const CD4060_SCHMITT_PIN: usize = 0;

/// Delay-tail pair order. 4017: `clkQ` (325 ns), `clrQ` (265 ns, the datasheet's
/// RESET to either output), `clkCo` (CLK to carry out, 300 ns). 4060: `clkQ`
/// (CLK to the first pinned stage, 370 ns), `stageQ` (one ripple hop, 100 ns),
/// `clrQ` (180 ns). How the 4060 composes `clkQ` and `stageQ` into a per-stage
/// delay, and which delay `rtc`/`ctc` take, is H2b's to settle (STOP rule).
pub const PAIRS_CD4017: [&str; 3] = ["clkQ", "clrQ", "clkCo"];
pub const PAIRS_CD4060: [&str; 3] = ["clkQ", "stageQ", "clrQ"];

/// The driven pins of a counter kind, `&[]` for any other.
pub fn output_capable(kind: u32) -> &'static [usize] {
    match kind {
        kinds::IC_CD4017 => &OUT_CD4017,
        kinds::IC_CD4060 => &OUT_CD4060,
        _ => &[],
    }
}

/// `cd4017` reader check: the register family's.
pub(crate) fn check_cd4017(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    kinds::seq_check(pin_count, params, blob)
}

/// `cd4060` reader check: the register family's, except that `schmitt` may be 1
/// (then `vtPlus > vtMinus`).
pub(crate) fn check_cd4060(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    kinds::seq_check_with(pin_count, params, blob, true)
}

/// `cd4017` state (H2b fills it in): the count 0-9 and the clock's last read level.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cd4017State {}

/// `cd4060` state (H2b fills it in): the 14-bit count, the oscillator inverter
/// outputs, the last Schmitt level of `clk_in` and its committed hysteresis state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cd4060State {}

impl Cd4017State {
    pub fn power_on(_ic: &IcRecord) -> Self {
        Cd4017State {}
    }
}

impl Cd4060State {
    pub fn power_on(_ic: &IcRecord) -> Self {
        Cd4060State {}
    }
}

/// One evaluation of a powered cd4017 (the `sequential::evaluate_into`
/// contract). A stub: unreachable while [`handles`] is `false`.
pub fn evaluate_cd4017(ic: &IcRecord, state: &mut Cd4017State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let _ = (ic, state, reads, last_reads, edges_live, now, out);
    unreachable!("cd4017 evaluator: W1h item H2b has not landed");
}

/// One evaluation of a powered cd4060. A stub: unreachable while [`handles`] is
/// `false`.
pub fn evaluate_cd4060(ic: &IcRecord, state: &mut Cd4060State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let _ = (ic, state, reads, last_reads, edges_live, now, out);
    unreachable!("cd4060 evaluator: W1h item H2b has not landed");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_tables_agree_with_the_record_shapes() {
        for (kind, pins, out, pairs, slots) in [
            (kinds::IC_CD4017, &PINS_CD4017[..], &OUT_CD4017[..], PAIRS_CD4017.len(), 1 + OUT_CD4017.len()),
            (kinds::IC_CD4060, &PINS_CD4060[..], &OUT_CD4060[..], PAIRS_CD4060.len(), 1 + OUT_CD4060.len()),
        ] {
            let shape = kinds::shape(kind).expect("shape");
            assert_eq!(shape.pins, 2 + pins.len(), "{}: vcc, gnd, then the logic pins", shape.name);
            assert_eq!((shape.digital_slots)(shape.pins), slots, "{}: the count plus one slot per driven pin", shape.name);
            assert_eq!(shape.params, kinds::ic_param::COUNT + 2 * pairs, "{}: 13 IC params and the delay tail", shape.name);
            assert_eq!(kinds::delay_pairs(kind), pairs, "{}", shape.name);
            assert_eq!(output_capable(kind), out);
        }
    }

    #[test]
    fn the_driven_pins_are_the_outputs() {
        let driven: Vec<&str> = OUT_CD4060.iter().map(|&i| PINS_CD4060[i]).collect();
        assert_eq!(driven, ["q4", "q5", "q6", "q7", "q8", "q9", "q10", "q12", "q13", "q14", "rtc", "ctc"]);
        assert_eq!(PINS_CD4060[CD4060_SCHMITT_PIN], "clk_in");
        let driven: Vec<&str> = OUT_CD4017.iter().map(|&i| PINS_CD4017[i]).collect();
        assert_eq!(driven, ["q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "co"]);
    }

    #[test]
    fn the_seam_is_inert_until_the_evaluators_land() {
        assert!(!handles(kinds::IC_CD4017) && !handles(kinds::IC_CD4060));
        assert!(!super::super::is_ic(kinds::IC_CD4017), "a cd4017 record is not an IC to the kernel yet");
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Cd4017State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        n
    }
}

impl crate::snapshot::HeapBytes for Cd4060State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        n
    }
}
