//! The shift registers `74hc595` (SIPO with output latch, code 73) and
//! `74hc165` (PISO, code 74): W1h item H2a. This file is the H0 seam. It fixes
//! the record layout both kinds share with the rest of the register family
//! (pin order, driven pins, slot keys, delay pairs) and holds the reader check
//! `kinds.rs` points at. The state and evaluators are H2a's; until they land
//! [`handles`] is `false`, so neither kind is an IC to the rest of the crate
//! and a program naming one stamps nothing, as before it had a shape.
//!
//! H2a flips [`handles`] to the `matches!` in its comment in the same change
//! that makes the evaluators real.
//!
//! Classic: `digital-ics.ts:515-612`, spec 4.2.7 and 4.2.8. Rulings that bind
//! H2a: Q-O58 (`qh2` stays undriven), Q-O61 (the 165 ignores an edge on
//! `cp_inh`), R56 10 (datasheet delays) and R56 11 (the 595's separate clocks:
//! mirror classic at equal instants, datasheet order at different instants,
//! plus a divergence row).

use super::{FamilyOutput, IcRecord};
use crate::kinds;

/// Flip to `matches!(kind, kinds::IC_74HC595 | kinds::IC_74HC165)` when the
/// evaluators land (H2a).
pub fn handles(kind: u32) -> bool {
    let _ = kind;
    false
}

/// `74hc595` logic pins after `vcc`, `gnd`: `/srclr srclk rclk /oe ser` (read),
/// `qa qb qc qd qe qf qg qh` (driven), `qh2` (QH', listed but never driven,
/// Q-O58). Bit 0 of the register is `qa`.
pub const PINS_74HC595: [&str; 14] = ["/srclr", "srclk", "rclk", "/oe", "ser", "qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh", "qh2"];
/// Indices into [`PINS_74HC595`] that own a driver and a digital slot, in slot
/// order: slot `i` is key `<id>/<PINS[OUT[i]]>`.
pub const OUT_74HC595: [usize; 8] = [5, 6, 7, 8, 9, 10, 11, 12];

/// `74hc165` logic pins: `/pl cp cp_inh ds d0 d1 d2 d3 d4 d5 d6 d7` (read),
/// `q7 /q7` (driven). Bit `i` of the register is `d<i>`; the register shifts
/// toward bit 7 and `q7` is bit 7.
pub const PINS_74HC165: [&str; 14] = ["/pl", "cp", "cp_inh", "ds", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "q7", "/q7"];
pub const OUT_74HC165: [usize; 2] = [12, 13];

/// The delay-tail pair order of each kind (`kinds::delay_pairs`; the TS side's
/// `CORE_IC_DELAY_PAIRS`). 595: `srclkQh` (SRCLK to QH', unused while QH' is
/// undriven), `rclkQ`, `srclrQh` (unused likewise), `oeEnable`, `oeDisable`.
/// 165: `clkQ`, `shldQ`, `dataQ`.
pub const PAIRS_74HC595: [&str; 5] = ["srclkQh", "rclkQ", "srclrQh", "oeEnable", "oeDisable"];
pub const PAIRS_74HC165: [&str; 3] = ["clkQ", "shldQ", "dataQ"];

/// The driven pins of a shift register kind, `&[]` for any other.
pub fn output_capable(kind: u32) -> &'static [usize] {
    match kind {
        kinds::IC_74HC595 => &OUT_74HC595,
        kinds::IC_74HC165 => &OUT_74HC165,
        _ => &[],
    }
}

/// Reader check for both kinds (the register family's: `schmitt` and
/// `OPEN_COLLECTOR` clear, every delay value `>= 0`). H2a may tighten it.
pub(crate) fn check(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    kinds::seq_check(pin_count, params, blob)
}

/// `74hc595` state (H2a fills it in): the shift and storage registers and the
/// last read level of each clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hc595State {}

/// `74hc165` state (H2a fills it in): the shift register and the clock's last
/// read level.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hc165State {}

impl Hc595State {
    pub fn power_on(_ic: &IcRecord) -> Self {
        Hc595State {}
    }
}

impl Hc165State {
    pub fn power_on(_ic: &IcRecord) -> Self {
        Hc165State {}
    }
}

/// One evaluation of a powered 74hc595 (the `sequential::evaluate_into`
/// contract). A stub: unreachable while [`handles`] is `false`.
pub fn evaluate_595(ic: &IcRecord, state: &mut Hc595State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let _ = (ic, state, reads, last_reads, edges_live, now, out);
    unreachable!("74hc595 evaluator: W1h item H2a has not landed");
}

/// One evaluation of a powered 74hc165. A stub: unreachable while [`handles`]
/// is `false`.
pub fn evaluate_165(ic: &IcRecord, state: &mut Hc165State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let _ = (ic, state, reads, last_reads, edges_live, now, out);
    unreachable!("74hc165 evaluator: W1h item H2a has not landed");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_tables_agree_with_the_record_shapes() {
        for (kind, pins, out, pairs) in [
            (kinds::IC_74HC595, &PINS_74HC595[..], &OUT_74HC595[..], PAIRS_74HC595.len()),
            (kinds::IC_74HC165, &PINS_74HC165[..], &OUT_74HC165[..], PAIRS_74HC165.len()),
        ] {
            let shape = kinds::shape(kind).expect("shape");
            assert_eq!(shape.pins, 2 + pins.len(), "{}: vcc, gnd, then the logic pins", shape.name);
            assert_eq!((shape.digital_slots)(shape.pins), out.len(), "{}: one slot per driven pin", shape.name);
            assert_eq!(shape.params, kinds::ic_param::COUNT + 2 * pairs, "{}: 13 IC params and the delay tail", shape.name);
            assert_eq!(kinds::delay_pairs(kind), pairs, "{}", shape.name);
            assert_eq!(output_capable(kind), out);
        }
    }

    #[test]
    fn the_driven_pins_are_the_outputs_and_qh2_is_not_one() {
        let driven: Vec<&str> = OUT_74HC595.iter().map(|&i| PINS_74HC595[i]).collect();
        assert_eq!(driven, ["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh"]);
        assert!(!OUT_74HC595.contains(&13), "Q-O58: qh2 stays undriven");
        let driven: Vec<&str> = OUT_74HC165.iter().map(|&i| PINS_74HC165[i]).collect();
        assert_eq!(driven, ["q7", "/q7"]);
    }

    #[test]
    fn the_seam_is_inert_until_the_evaluators_land() {
        assert!(!handles(kinds::IC_74HC595) && !handles(kinds::IC_74HC165));
        assert!(!super::super::is_ic(kinds::IC_74HC595), "a 595 record is not an IC to the kernel yet");
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Hc595State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        n
    }
}

impl crate::snapshot::HeapBytes for Hc165State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        n
    }
}
