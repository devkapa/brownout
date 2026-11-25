//! The counters `cd4017` (decade counter, code 75) and `cd4060` (14-stage ripple
//! counter with oscillator, code 76): W1h item H2b. [`handles`] makes both kinds
//! sequential ICs to the rest of the crate (`digital::is_sequential_ic`).
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
//! The hysteresis state lives in [`Cd4060State`] so it rolls back with the family
//! state; `devices::ics` does the read (one hunk in `evaluate_impl`) and
//! `levels::watch_thresholds` keeps `reset`'s crossing watch at the plain
//! thresholds. Divergence rows in `w1h-h2b-divergences.md`.
//!
//! TRAP for H2b: `icPinMap("cd4060")` in classic's `digital.ts` lists the
//! unpinned stages' ids `q3 q2 q1` where the catalog and classic's own stamp
//! use `q9 q8 q10`; the record below follows the catalog.

use super::{FamilyOutput, IcRecord};
use crate::drivers::DelaySet;
use crate::events::Mode;
use crate::kinds;

/// The kinds this module evaluates.
pub fn handles(kind: u32) -> bool {
    matches!(kind, kinds::IC_CD4017 | kinds::IC_CD4060)
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
/// `clrQ` (180 ns); stage `n` takes `clkQ + (n - 4) stageQ`, and `rtc`/`ctc`, which
/// the datasheet gives no figure for, take none (see [`evaluate_cd4060`]).
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

/// `cd4017` state: the count 0-9 and the clock's last read level. The level is
/// tracked at every live evaluation whether or not `clkinh` lets the clock
/// through, so releasing `clkinh` while `clk` is high never clocks (Q-O61).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cd4017State {
    pub count: u8,
    pub last_clk: bool,
}

/// `cd4060` state: the 14-bit count, the last level of `clk_in` the counter saw
/// (the falling edge is detected against it) and the committed hysteresis state
/// of the Schmitt read of `clk_in`. The rtc/ctc inverter outputs are not state:
/// they are levels of the same read, recomputed at every evaluation.
/// `devices::ics` reads `clk_in` against `schmitt` and hands the evaluator the
/// resulting level, so the state rolls back with the rest of the family.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cd4060State {
    pub count: u16,
    pub last_clk: bool,
    pub schmitt: bool,
}

impl Cd4017State {
    pub fn power_on(_ic: &IcRecord) -> Self {
        Cd4017State { count: 0, last_clk: false }
    }
}

impl Cd4060State {
    pub fn power_on(_ic: &IcRecord) -> Self {
        Cd4060State { count: 0, last_clk: false, schmitt: false }
    }
}

impl Cd4017State {
    /// W2c's carry layout: `[count, last_clk]`.
    pub const CARRY_LEN: usize = 2;

    pub fn carry_values(&self, out: &mut Vec<f64>) {
        out.extend([f64::from(self.count), crate::carry::ic::bit(self.last_clk)]);
    }

    pub fn carry_seed(&mut self, v: &[f64]) -> Result<(), String> {
        if !(v[0].fract() == 0.0 && (0.0..=9.0).contains(&v[0])) {
            return Err(format!("carry: a 4017's count {} is out of range", v[0]));
        }
        self.count = v[0] as u8;
        self.last_clk = crate::carry::ic::bit_of(v[1])?;
        Ok(())
    }
}

impl Cd4060State {
    /// W2c's carry layout: `[count, last_clk, schmitt]`; `schmitt` is the committed hysteresis level of `clk_in`,
    /// which a reload must keep for a clock held inside the band.
    pub const CARRY_LEN: usize = 3;

    pub fn carry_values(&self, out: &mut Vec<f64>) {
        use crate::carry::ic::bit;
        out.extend([f64::from(self.count), bit(self.last_clk), bit(self.schmitt)]);
    }

    pub fn carry_seed(&mut self, v: &[f64]) -> Result<(), String> {
        use crate::carry::ic::bit_of;
        if !(v[0].fract() == 0.0 && (0.0..=16383.0).contains(&v[0])) {
            return Err(format!("carry: a 4060's count {} is out of range", v[0]));
        }
        self.count = v[0] as u16;
        self.last_clk = bit_of(v[1])?;
        self.schmitt = bit_of(v[2])?;
        Ok(())
    }
}

fn level(bit: bool) -> Mode {
    if bit {
        Mode::High
    } else {
        Mode::Low
    }
}

/// Neither counter ever tri-states: the unselectable rows carry the kind's
/// first pair, as the register family's evaluators do.
fn level_delay((lh, hl): (f64, f64), filler: (f64, f64)) -> DelaySet {
    DelaySet { lh, hl, zh: filler.0, zl: filler.1, hz: filler.0, lz: filler.1 }
}

/// One evaluation of a powered cd4017 (the `sequential::evaluate_into`
/// contract; spec 4.2.9, classic `digital-ics.ts:755-811`). Logic reads, in pin
/// order: `clk clkinh reset` (then the outputs, unread).
///
/// With edges live: `reset` high clears the count (level-sensitive, and it wins
/// over a clock edge of the same evaluation); otherwise a rising `clk` with
/// `clkinh` low advances it, `(count + 1) % 10`. `last_clk` follows `clk` at
/// every live evaluation, so an edge that arrives while `clkinh` is high is
/// consumed and releasing `clkinh` alone never clocks (Q-O61). An open `reset`
/// reads through the floating model (Q-O11), like any other open pin.
///
/// Targets: `q<count>` is the one high decoded output and `co` is high for
/// counts 0-4 (the divide-by-ten square wave). The delay is the cause of the
/// change: a clear `clrQ` on every output, a clock edge `clkQ` on the
/// decoded outputs and `clkCo` on the carry; with no change in this evaluation
/// the first pair.
pub fn evaluate_cd4017(ic: &IcRecord, state: &mut Cd4017State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let _ = (last_reads, now);
    let clk_now = reads[0];
    let inhibit = reads[1];
    let reset = reads[2];

    let old = state.count;
    let mut cleared = false;
    let mut counted = false;
    if edges_live {
        if reset {
            state.count = 0;
            cleared = true;
        } else if clk_now && !state.last_clk && !inhibit {
            state.count = (state.count + 1) % 10;
            counted = true;
        }
        state.last_clk = clk_now;
    }
    let changed = state.count != old;

    let first = ic.delays[0];
    let (q_pair, co_pair) = if cleared && changed {
        (ic.delays[1], ic.delays[1])
    } else if counted && changed {
        (ic.delays[0], ic.delays[2])
    } else {
        (first, first)
    };
    let q_delay = level_delay(q_pair, first);
    let co_delay = level_delay(co_pair, first);

    let count = state.count;
    out.requests.clear();
    out.requests.extend((0..10u8).map(|i| (level(count == i), q_delay)));
    out.requests.push((level(count < 5), co_delay));
    out.slots.clear();
    out.slots.push(count as f64);
    out.slots.extend((0..10u8).map(|i| (count == i) as u8 as f64));
    out.slots.push((count < 5) as u8 as f64);
    out.timers.clear();
}

/// The stage each driven `q` pin of the 4060 is (`Q4 .. Q10`, `Q12 .. Q14`),
/// in [`OUT_CD4060`] order; stage `n` is count bit `n - 1`.
pub const STAGES_CD4060: [u32; 10] = [4, 5, 6, 7, 8, 9, 10, 12, 13, 14];

/// One evaluation of a powered cd4060 (spec 4.2.10, classic
/// `digital-ics.ts:885-1047`). Logic reads, in pin order: `clk_in reset` (then
/// the outputs, unread); `reads[0]` is already the Schmitt level of `clk_in`
/// (`devices::ics`, against [`Cd4060State::schmitt`]), `reset` is a plain read.
///
/// With edges live: `reset` high clears the 14-bit count; otherwise a FALLING
/// `clk_in` (the counter's own RS level, against `last_clk`) advances it. The
/// level is tracked through a reset too.
///
/// Targets, every evaluation: the ten pinned stages from the count, and the
/// oscillator's two inverter stages, which are level functions of the same
/// read: `rtc = !RS` and `ctc = RS` (Q-O60: the code's phase, not the catalog
/// note's), both low while `reset` is high so that oscillation restarts
/// cleanly on release.
///
/// Delays: a clear `clrQ` on every stage; a counting edge reaches stage `n`
/// after `clkQ` (CLK to the first pinned stage, Q4) plus `n - 4` ripple hops of
/// `stageQ`, because stage `n` toggles only once stage `n - 1` has fallen; with
/// no change the same figure. `rtc` and `ctc` are the inverters ahead of the
/// divider, for which the datasheet gives no figure: they take no delay, so an
/// external RC loop sets the oscillator's period alone, as in classic.
pub fn evaluate_cd4060(ic: &IcRecord, state: &mut Cd4060State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let _ = (last_reads, now);
    let rs = reads[0];
    let reset = reads[1];

    let mut cleared = false;
    if edges_live {
        if reset {
            state.count = 0;
            cleared = true;
        } else if state.last_clk && !rs {
            state.count = (state.count + 1) & 0x3fff;
        }
        state.last_clk = rs;
    }

    let first = ic.delays[0];
    let count = state.count;
    out.requests.clear();
    out.requests.extend(STAGES_CD4060.iter().map(|&stage| {
        let hops = (stage - 4) as f64;
        let pair = if cleared { ic.delays[2] } else { (ic.delays[0].0 + hops * ic.delays[1].0, ic.delays[0].1 + hops * ic.delays[1].1) };
        (level((count >> (stage - 1)) & 1 == 1), level_delay(pair, first))
    }));
    let rtc = !reset && !rs;
    let ctc = !reset && rs;
    let inverter = DelaySet::uniform(0.0);
    out.requests.push((level(rtc), inverter));
    out.requests.push((level(ctc), inverter));
    out.slots.clear();
    out.slots.push(count as f64);
    out.slots.extend(STAGES_CD4060.iter().map(|&stage| ((count >> (stage - 1)) & 1) as f64));
    out.slots.push(rtc as u8 as f64);
    out.slots.push(ctc as u8 as f64);
    out.timers.clear();
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
    fn both_kinds_are_sequential_ics_to_the_kernel() {
        assert!(handles(kinds::IC_CD4017) && handles(kinds::IC_CD4060));
        assert!(!handles(kinds::IC_74HC595));
        assert!(super::super::is_ic(kinds::IC_CD4017) && super::super::is_ic(kinds::IC_CD4060));
        assert!(super::super::is_sequential_ic(kinds::IC_CD4017) && super::super::is_sequential_ic(kinds::IC_CD4060));
    }

    // ---- evaluator tests (W1h H2b) ----------------------------------------

    use super::super::IcPin;

    fn pin(node: usize) -> IcPin {
        IcPin { node, open: false, open_high: false, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false }
    }

    fn ic_of(kind: u32, delays: Vec<(f64, f64)>) -> IcRecord {
        IcRecord {
            kind,
            vcc: pin(1),
            gnd: pin(0),
            logic: (0..14).map(|i| pin(10 + i)).collect(),
            vcc_min: 0.0,
            vcc_max: 0.0,
            ttl: false,
            cmos_ratio: true,
            vil: None,
            vih: None,
            schmitt: None,
            nominal: 5.0,
            io_max: None,
            delay: 0.0,
            slot_base: 0,
            delays,
            extra: Vec::new(),
        }
    }

    /// clkQ, clrQ, clkCo: every figure distinct so a wrong pick shows.
    fn ic_4017() -> IcRecord {
        ic_of(kinds::IC_CD4017, vec![(325e-9, 326e-9), (265e-9, 266e-9), (300e-9, 301e-9)])
    }

    /// clkQ, stageQ, clrQ.
    fn ic_4060() -> IcRecord {
        ic_of(kinds::IC_CD4060, vec![(370e-9, 371e-9), (100e-9, 101e-9), (180e-9, 181e-9)])
    }

    struct Rig4017 {
        ic: IcRecord,
        st: Cd4017State,
        last: Option<Vec<bool>>,
        out: FamilyOutput,
    }

    impl Rig4017 {
        fn new() -> Self {
            let ic = ic_4017();
            let st = Cd4017State::power_on(&ic);
            Rig4017 { ic, st, last: None, out: FamilyOutput::default() }
        }
        /// Reads in pin order: `clk clkinh reset`, then the undriven outputs.
        fn step_live(&mut self, clk: bool, inh: bool, reset: bool, live: bool) -> &FamilyOutput {
            let mut reads = vec![clk, inh, reset];
            reads.extend([false; 11]);
            evaluate_cd4017(&self.ic, &mut self.st, &reads, self.last.as_deref(), live, 0.0, &mut self.out);
            self.last = Some(reads);
            &self.out
        }
        fn step(&mut self, clk: bool, inh: bool, reset: bool) -> &FamilyOutput {
            self.step_live(clk, inh, reset, true)
        }
        /// One full clock pulse, the clock left low.
        fn pulse(&mut self) {
            self.step(true, false, false);
            self.step(false, false, false);
        }
    }

    fn levels_of(out: &FamilyOutput) -> Vec<bool> {
        out.requests.iter().map(|r| r.0 == Mode::High).collect()
    }

    #[test]
    fn the_4017_powers_on_at_q0_with_carry_high() {
        let mut r = Rig4017::new();
        let out = r.step(false, false, false);
        let mut expect = vec![false; 11];
        expect[0] = true;
        expect[10] = true;
        assert_eq!(levels_of(out), expect);
        assert_eq!(out.slots, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], "the count, q0..q9, co");
    }

    #[test]
    fn the_4017_advances_on_the_rising_edge_only_and_wraps_at_ten() {
        let mut r = Rig4017::new();
        for n in 1..=10u32 {
            r.step(true, false, false);
            assert_eq!(r.st.count as u32, n % 10, "rising edge {n}");
            r.step(false, false, false);
            assert_eq!(r.st.count as u32, n % 10, "the falling edge does not count");
        }
        r.pulse();
        let out = r.step(false, false, false);
        assert_eq!(out.slots[0], 1.0);
        assert!(levels_of(out)[1], "one-hot on q1");
    }

    #[test]
    fn the_4017_carry_is_high_for_counts_0_to_4_and_low_for_5_to_9() {
        let mut r = Rig4017::new();
        for n in 0..10u32 {
            let out = r.step(false, false, false);
            assert_eq!(levels_of(out)[10], n < 5, "count {n}");
            assert_eq!(out.slots[11], (n < 5) as u8 as f64);
            r.pulse();
        }
    }

    #[test]
    fn the_4017_holds_while_clkinh_is_high_and_an_edge_under_it_is_consumed() {
        let mut r = Rig4017::new();
        r.pulse();
        assert_eq!(r.st.count, 1);
        r.step(true, true, false);
        r.step(false, true, false);
        assert_eq!(r.st.count, 1, "an inhibited edge does not count");
        // Q-O61: clk high under the inhibit, then the inhibit released, never clocks.
        r.step(true, true, false);
        r.step(true, false, false);
        assert_eq!(r.st.count, 1, "releasing clkinh while clk is high is not an edge");
        r.step(false, false, false);
        r.step(true, false, false);
        assert_eq!(r.st.count, 2);
    }

    #[test]
    fn the_4017_reset_clears_at_once_and_wins_over_a_simultaneous_edge() {
        let mut r = Rig4017::new();
        r.pulse();
        r.pulse();
        r.pulse();
        assert_eq!(r.st.count, 3);
        r.step(true, false, true);
        assert_eq!(r.st.count, 0, "clk rising with reset high: reset wins");
        r.step(false, false, true);
        r.step(true, false, true);
        assert_eq!(r.st.count, 0, "held in reset");
        r.step(false, false, false);
        r.step(true, false, false);
        assert_eq!(r.st.count, 1, "counts again once released");
    }

    #[test]
    fn the_4017_acts_on_no_edge_while_edges_are_frozen() {
        let mut r = Rig4017::new();
        r.pulse();
        r.step_live(true, false, false, false);
        assert_eq!(r.st.count, 1, "a frozen settle pass counts nothing");
        assert!(!r.st.last_clk, "and does not move the clock's history, so the live pass still sees the edge");
        r.step_live(true, false, true, false);
        assert_eq!(r.st.count, 1, "nor clears");
        r.step(true, false, false);
        assert_eq!(r.st.count, 2);
    }

    #[test]
    fn the_4017_takes_its_delay_from_the_cause_of_the_change() {
        let mut r = Rig4017::new();
        let out = r.step(true, false, false).clone();
        let (clk_q, clk_co) = ((325e-9, 326e-9), (300e-9, 301e-9));
        assert!(out.requests[..10].iter().all(|q| (q.1.lh, q.1.hl) == clk_q), "decoded outputs: clkQ");
        assert_eq!((out.requests[10].1.lh, out.requests[10].1.hl), clk_co, "carry: clkCo");
        r.step(false, false, false);
        // Clear: every output takes clrQ, the carry too (RESET to either output).
        for _ in 0..3 {
            r.pulse();
        }
        let out = r.step(false, false, true).clone();
        assert_eq!(r.st.count, 0);
        assert!(out.requests.iter().all(|q| (q.1.lh, q.1.hl) == (265e-9, 266e-9)), "clear: clrQ everywhere");
        // No change in this evaluation: the first pair.
        let out = r.step(false, false, false).clone();
        assert!(out.requests.iter().all(|q| (q.1.lh, q.1.hl) == clk_q));
        // The unselectable rows carry the first pair, never zero.
        assert!(out.requests.iter().all(|q| (q.1.zh, q.1.zl, q.1.hz, q.1.lz) == (325e-9, 326e-9, 325e-9, 326e-9)));
    }

    struct Rig4060 {
        ic: IcRecord,
        st: Cd4060State,
        last: Option<Vec<bool>>,
        out: FamilyOutput,
    }

    impl Rig4060 {
        fn new() -> Self {
            let ic = ic_4060();
            let st = Cd4060State::power_on(&ic);
            Rig4060 { ic, st, last: None, out: FamilyOutput::default() }
        }
        /// Reads in pin order: `clk_in` (already the Schmitt level), `reset`, then the undriven outputs.
        fn step_live(&mut self, rs: bool, reset: bool, live: bool) -> &FamilyOutput {
            let mut reads = vec![rs, reset];
            reads.extend([false; 12]);
            evaluate_cd4060(&self.ic, &mut self.st, &reads, self.last.as_deref(), live, 0.0, &mut self.out);
            self.last = Some(reads);
            &self.out
        }
        fn step(&mut self, rs: bool, reset: bool) -> &FamilyOutput {
            self.step_live(rs, reset, true)
        }
        /// One full RS cycle starting from low: up, then the falling edge that counts.
        fn cycle(&mut self) {
            self.step(true, false);
            self.step(false, false);
        }
    }

    #[test]
    fn the_4060_counts_on_the_falling_edge_only() {
        let mut r = Rig4060::new();
        r.step(false, false);
        r.step(true, false);
        assert_eq!(r.st.count, 0, "the rising edge does not count");
        r.step(true, false);
        assert_eq!(r.st.count, 0, "a high level does not");
        r.step(false, false);
        assert_eq!(r.st.count, 1, "the falling edge does");
        r.step(false, false);
        assert_eq!(r.st.count, 1);
        for n in 2..=5 {
            r.cycle();
            assert_eq!(r.st.count, n);
        }
    }

    #[test]
    fn the_4060_is_a_14_bit_counter() {
        let mut r = Rig4060::new();
        r.st.count = 0x3fff;
        r.cycle();
        assert_eq!(r.st.count, 0, "wraps at 2^14");
    }

    #[test]
    fn the_4060_pins_carry_stage_n_as_count_bit_n_minus_1() {
        let mut r = Rig4060::new();
        // Pin order of the driven outputs: q4..q10, q12, q13, q14, rtc, ctc.
        for (count, hot) in [(1u16 << 3, Some(0usize)), (1 << 4, Some(1)), (1 << 9, Some(6)), (1 << 10, None), (1 << 11, Some(7)), (1 << 13, Some(9)), (0b111, None)] {
            r.st.count = count;
            let out = r.step(false, false);
            let q = levels_of(out);
            let want: Vec<bool> = (0..10).map(|i| Some(i) == hot).collect();
            assert_eq!(&q[..10], &want[..], "count {count:#b}: stages 1-3 and 11 have no pin");
            assert_eq!(out.slots[0], count as f64);
            assert_eq!(&out.slots[1..11], &want.iter().map(|&b| b as u8 as f64).collect::<Vec<_>>()[..]);
        }
    }

    #[test]
    fn the_4060_drives_rtc_as_not_rs_and_ctc_as_rs_and_both_low_in_reset() {
        let mut r = Rig4060::new();
        let q = levels_of(r.step(false, false));
        assert_eq!((q[10], q[11]), (true, false), "RS low: rtc high, ctc low (Q-O60)");
        let q = levels_of(r.step(true, false));
        assert_eq!((q[10], q[11]), (false, true), "RS high: rtc low, ctc high");
        let out = r.step(true, true);
        assert_eq!((levels_of(out)[10], levels_of(out)[11]), (false, false), "reset forces both low");
        assert_eq!((out.slots[11], out.slots[12]), (0.0, 0.0));
        let q = levels_of(r.step(false, true));
        assert_eq!((q[10], q[11]), (false, false), "and holds them low whatever RS does");
        let q = levels_of(r.step(false, false));
        assert_eq!((q[10], q[11]), (true, false), "released: the inverters follow RS again");
    }

    #[test]
    fn the_4060_reset_clears_and_a_falling_edge_under_reset_does_not_count() {
        let mut r = Rig4060::new();
        for _ in 0..5 {
            r.cycle();
        }
        assert_eq!(r.st.count, 5);
        r.step(true, true);
        r.step(false, true);
        assert_eq!(r.st.count, 0, "cleared, and the falling edge under reset counted nothing");
        r.step(true, false);
        r.step(false, false);
        assert_eq!(r.st.count, 1);
    }

    #[test]
    fn the_4060_tracks_rs_through_a_reset_so_a_release_during_a_high_rs_is_not_an_edge() {
        let mut r = Rig4060::new();
        r.step(true, true);
        r.step(true, false);
        assert_eq!(r.st.count, 0, "RS high through the release: no falling edge yet");
        r.step(false, false);
        assert_eq!(r.st.count, 1);
    }

    #[test]
    fn the_4060_acts_on_no_edge_while_edges_are_frozen_but_still_drives_the_inverters() {
        let mut r = Rig4060::new();
        r.cycle();
        r.step_live(true, false, false);
        let out = r.step_live(false, false, false).clone();
        assert_eq!(r.st.count, 1, "a frozen pass counts nothing");
        assert!(!r.st.last_clk);
        assert!(levels_of(&out)[10], "but rtc follows the read");
    }

    #[test]
    fn the_4060_ripples_stage_n_after_clkq_plus_n_minus_4_hops_and_clears_at_clrq() {
        let mut r = Rig4060::new();
        r.step(true, false);
        let out = r.step(false, false).clone();
        for (i, &stage) in STAGES_CD4060.iter().enumerate() {
            let hops = (stage - 4) as f64;
            let d = out.requests[i].1;
            assert!((d.lh - (370e-9 + hops * 100e-9)).abs() < 1e-15, "stage {stage} lh: {}", d.lh);
            assert!((d.hl - (371e-9 + hops * 101e-9)).abs() < 1e-15, "stage {stage} hl: {}", d.hl);
            assert_eq!((d.zh, d.zl, d.hz, d.lz), (370e-9, 371e-9, 370e-9, 371e-9), "never tri-stated: the first pair");
        }
        assert!((out.requests[9].1.lh - 1370e-9).abs() < 1e-15, "Q14 is eight hops past Q4");
        // The oscillator stages take no delay.
        for k in [10, 11] {
            assert_eq!(out.requests[k].1, DelaySet::uniform(0.0), "rtc/ctc request {k}");
        }
        let out = r.step(true, true).clone();
        for i in 0..10 {
            assert_eq!((out.requests[i].1.lh, out.requests[i].1.hl), (180e-9, 181e-9), "reset: clrQ on every stage");
        }
        assert_eq!(out.requests[10].1, DelaySet::uniform(0.0));
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Cd4017State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.count, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_clk, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Cd4060State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.count, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_clk, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.schmitt, w); }
        n
    }
}
