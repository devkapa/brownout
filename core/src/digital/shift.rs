//! The shift registers `74hc595` (SIPO with output latch, code 73) and
//! `74hc165` (PISO, code 74): W1h item H2a. The record layout both kinds share
//! with the rest of the register family (pin order, driven pins, slot keys,
//! delay pairs) and the reader check `kinds.rs` points at are H0's seam; the
//! state and evaluators are H2a's. [`handles`] makes both kinds sequential ICs
//! to the rest of the crate (`digital::is_sequential_ic`).
//!
//! Classic: `digital-ics.ts:515-612`, spec 4.2.7 and 4.2.8. Rulings that bind
//! H2a: Q-O58 (`qh2` stays undriven), Q-O61 (the 165 ignores an edge on
//! `cp_inh`), R56 10 (datasheet delays) and R61 (a), which replaced R56 11: the
//! 595's captures are two-phase, every capture of an instant sampling the state
//! as of the instant's start, so coincident SRCLK and RCLK store the pre-shift
//! byte on tied and separate nets alike; edges at different instants keep the
//! datasheet order. Divergence rows in `w1h-h2a-divergences.md`.

use super::{FamilyOutput, IcRecord};
use crate::drivers::DelaySet;
use crate::events::Mode;
use crate::kinds;

/// The kinds this module evaluates.
pub fn handles(kind: u32) -> bool {
    matches!(kind, kinds::IC_74HC595 | kinds::IC_74HC165)
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

/// `74hc595` state: the shift register (bit 0 is `qa`, the newest bit), the
/// storage register that drives `qa..qh`, each clock's last read level, and the
/// two-phase snapshot of R61 (a): the instant of the latest live evaluation,
/// the shift register as that instant began, and the SER level as it stood then
/// (`ser_last` is SER at the previous live evaluation, `ser_pre` its copy for
/// this instant). Every capture of the instant samples the snapshot, so the
/// order of deltas inside an instant never shows. Rolls back with `SeqState`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hc595State {
    pub shift: u8,
    pub latch: u8,
    pub last_srclk: bool,
    pub last_rclk: bool,
    pub instant: Option<f64>,
    pub shift_pre: u8,
    pub ser_pre: bool,
    pub ser_last: Option<bool>,
}

/// `74hc165` state: the shift register (bit `i` is `d<i>`, `q7` is bit 7) and
/// the clock's last read level. The clock's level is tracked whether or not
/// `cp_inh` lets it through, so releasing `cp_inh` alone never clocks (Q-O61).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hc165State {
    pub shift: u8,
    pub last_clk: bool,
}

impl Hc595State {
    pub fn power_on(_ic: &IcRecord) -> Self {
        Hc595State { shift: 0, latch: 0, last_srclk: false, last_rclk: false, instant: None, shift_pre: 0, ser_pre: false, ser_last: None }
    }
}

impl Hc165State {
    pub fn power_on(_ic: &IcRecord) -> Self {
        Hc165State { shift: 0, last_clk: false }
    }
}

/// A carried register byte: an integer 0-255.
fn byte_of(v: f64, what: &str) -> Result<u8, String> {
    if v.fract() == 0.0 && (0.0..=255.0).contains(&v) {
        Ok(v as u8)
    } else {
        Err(format!("carry: a {what} of {v} is out of range"))
    }
}

impl Hc595State {
    /// W2c's carry layout: `[shift, latch, last_srclk, last_rclk, ser_last]`, `ser_last` being `-1` before the first
    /// live evaluation. The instant's two-phase scratch (`instant`, `shift_pre`, `ser_pre`) is not carried: it is
    /// rebuilt from the state by the first live evaluation of any instant, and a reload begins a new one.
    pub const CARRY_LEN: usize = 5;

    pub fn carry_values(&self, out: &mut Vec<f64>) {
        use crate::carry::ic::bit;
        out.extend([f64::from(self.shift), f64::from(self.latch), bit(self.last_srclk), bit(self.last_rclk)]);
        out.push(self.ser_last.map_or(-1.0, bit));
    }

    pub fn carry_seed(&mut self, v: &[f64]) -> Result<(), String> {
        use crate::carry::ic::bit_of;
        self.shift = byte_of(v[0], "595's shift register")?;
        self.latch = byte_of(v[1], "595's storage register")?;
        self.last_srclk = bit_of(v[2])?;
        self.last_rclk = bit_of(v[3])?;
        self.ser_last = if v[4] == -1.0 { None } else { Some(bit_of(v[4])?) };
        self.instant = None;
        self.shift_pre = self.shift;
        self.ser_pre = false;
        Ok(())
    }
}

impl Hc165State {
    /// W2c's carry layout: `[shift, last_clk]`.
    pub const CARRY_LEN: usize = 2;

    pub fn carry_values(&self, out: &mut Vec<f64>) {
        out.extend([f64::from(self.shift), crate::carry::ic::bit(self.last_clk)]);
    }

    pub fn carry_seed(&mut self, v: &[f64]) -> Result<(), String> {
        self.shift = byte_of(v[0], "165's shift register")?;
        self.last_clk = crate::carry::ic::bit_of(v[1])?;
        Ok(())
    }
}

/// Whether `srclk` and `rclk` sit on one real net (classic `14b2a07`:
/// `pinNode(srclk) >= 0 && pinNode(srclk) === pinNode(rclk)`). Node 0 is the
/// datum, classic's row -1 (ground or an unmapped pin), and an open pin has no
/// net of its own; two of either float or sit apart and read separately.
fn clocks_tied(ic: &IcRecord) -> bool {
    match (ic.logic.get(1), ic.logic.get(2)) {
        (Some(s), Some(r)) => s.node == r.node && s.node != 0 && !s.open && !r.open,
        _ => false,
    }
}

/// The larger figure per direction over the causes of one transition, the
/// kind's first pair when none is identified (delays point 3, as the register
/// family's own `combine_causes`).
fn combine_causes(causes: &[(f64, f64)], first: (f64, f64)) -> (f64, f64) {
    if causes.is_empty() {
        first
    } else {
        (causes.iter().map(|c| c.0).fold(f64::NEG_INFINITY, f64::max), causes.iter().map(|c| c.1).fold(f64::NEG_INFINITY, f64::max))
    }
}

fn level(bit: bool) -> Mode {
    if bit {
        Mode::High
    } else {
        Mode::Low
    }
}

/// One evaluation of a powered 74hc595 (the `sequential::evaluate_into`
/// contract; spec 4.2.7, classic `digital-ics.ts:515-564`). Logic reads, in
/// pin order: `/srclr srclk rclk /oe ser` (then the outputs, unread).
///
/// Update, with edges live only (classic's `updateDigital`, two-phase per R61
/// (a)): `/srclr` low clears the shift register and that evaluation does not
/// shift; otherwise a rising SRCLK shifts in the SER level as the instant began.
/// A rising RCLK stores the shift register as the instant began, so RCLK and
/// SRCLK rising in one instant, tied or on separate nets, store the pre-shift
/// byte (both stages sample pre-edge data; the outputs move only after tpd), a
/// `/srclr` of the same instant does not reach the capture, and an SRCLK rise
/// after an RCLK rise and fall of one instant never rewrites it. Edges at
/// different instants keep the datasheet order: an RCLK after an earlier SRCLK
/// stores the shifted byte (TI SCLS041J 8.1 for the tied case).
///
/// Target, live every evaluation: `/oe` high requests HIZ on all eight Q
/// (`oeDisable`), else the stored byte (`rclkQ`, with `oeEnable` coming out of
/// HIZ). QH' stays undriven (Q-O58). The slots show the register, not the pin
/// view (README sequential point 8).
pub fn evaluate_595(ic: &IcRecord, state: &mut Hc595State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let _ = last_reads;
    let srclk_now = reads[1];
    // Tied clocks are one net, read once: a slow edge cannot split across deltas.
    let rclk_now = if clocks_tied(ic) { srclk_now } else { reads[2] };

    if edges_live {
        // Phase one of the instant: the first live evaluation at a new `now`
        // freezes what every capture of this instant will sample. Deltas of one
        // instant share one bit-identical time.
        if state.instant != Some(now) {
            state.instant = Some(now);
            state.shift_pre = state.shift;
            state.ser_pre = state.ser_last.unwrap_or(reads[4]);
        }
        let clr_low = !reads[0];
        let rising_srclk = srclk_now && !state.last_srclk;
        let rising_rclk = rclk_now && !state.last_rclk;
        if clr_low {
            state.shift = 0;
        }
        if !clr_low && rising_srclk {
            state.shift = (state.shift << 1) | state.ser_pre as u8;
        }
        if rising_rclk {
            state.latch = state.shift_pre;
        }
        state.last_srclk = srclk_now;
        state.last_rclk = rclk_now;
        state.ser_last = Some(reads[4]);
    }

    let delay = DelaySet { lh: ic.delays[1].0, hl: ic.delays[1].1, zh: ic.delays[3].0, zl: ic.delays[3].1, hz: ic.delays[4].0, lz: ic.delays[4].1 };
    let tri_stated = reads[3];
    out.requests.clear();
    out.requests.extend((0..8).map(|b| (if tri_stated { Mode::Hiz } else { level((state.latch >> b) & 1 == 1) }, delay)));
    out.slots.clear();
    out.slots.extend((0..8).map(|b| ((state.latch >> b) & 1) as f64));
    out.timers.clear();
}

/// One evaluation of a powered 74hc165 (spec 4.2.8, classic
/// `digital-ics.ts:566-612`). Logic reads, in pin order: `/pl cp cp_inh ds
/// d0..d7`. With edges live: `/pl` low loads `d0..d7` into bits 0-7 at every
/// evaluation (level-sensitive, and it wins over a clock edge); otherwise a
/// rising `cp` with `cp_inh` low shifts `ds` into bit 0. `last_clk` follows
/// `cp` at every live evaluation, so an edge that arrived while `cp_inh` held
/// is consumed and releasing `cp_inh` alone never clocks (Q-O61).
///
/// Targets: `q7` is bit 7 and `/q7` its complement. The delay is the cause of
/// the change of bit 7: a shift edge `clkQ`, a load that began with `/pl`
/// falling `shldQ`, a load that followed a moved `d7` `dataQ`; several causes
/// take the larger figure per direction, and no identified cause the first
/// pair.
pub fn evaluate_165(ic: &IcRecord, state: &mut Hc165State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let _ = now;
    let pl_low = !reads[0];
    let cp_now = reads[1];
    let old_q7 = state.shift & 0x80 != 0;

    let mut edge_shift = false;
    let mut loaded = false;
    if edges_live {
        let rising = cp_now && !state.last_clk && !reads[2];
        if pl_low {
            state.shift = (0..8).fold(0u8, |acc, i| acc | ((reads[4 + i] as u8) << i));
            loaded = true;
        } else if rising {
            state.shift = (state.shift << 1) | reads[3] as u8;
            edge_shift = true;
        }
        state.last_clk = cp_now;
    }

    let q7 = state.shift & 0x80 != 0;
    let mut causes = [(0.0, 0.0); 3];
    let mut n = 0;
    if q7 != old_q7 {
        if edge_shift {
            causes[n] = ic.delays[0];
            n += 1;
        }
        if loaded {
            // /pl falling this evaluation (or no history yet) is the SH/LD path.
            if last_reads.is_none_or(|lr| lr[0]) {
                causes[n] = ic.delays[1];
                n += 1;
            }
            // d7 moving under a low /pl is the direct data path.
            if last_reads.is_none_or(|lr| lr[11] != reads[11]) {
                causes[n] = ic.delays[2];
                n += 1;
            }
        }
    }
    let (lh, hl) = combine_causes(&causes[..n], ic.delays[0]);
    // Neither output ever tri-states: the unselectable rows carry the first pair.
    let filler = ic.delays[0];
    let delay = DelaySet { lh, hl, zh: filler.0, zl: filler.1, hz: filler.0, lz: filler.1 };

    out.requests.clear();
    out.requests.extend([(level(q7), delay), (level(!q7), delay)]);
    out.slots.clear();
    out.slots.extend([q7 as u8 as f64, !q7 as u8 as f64]);
    out.timers.clear();
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

    // ---- evaluator tests (W1h H2a) ----------------------------------------

    use super::super::IcPin;
    use crate::events::Mode;

    fn pin(node: usize) -> IcPin {
        IcPin { node, open: false, open_high: false, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false }
    }

    fn ic_of(kind: u32, n_logic: usize, delays: Vec<(f64, f64)>) -> IcRecord {
        IcRecord {
            kind,
            vcc: pin(1),
            gnd: pin(0),
            // Distinct nodes per logic pin, so no two clocks read as tied.
            logic: (0..n_logic).map(|i| pin(10 + i)).collect(),
            vcc_min: 0.0,
            vcc_max: 0.0,
            ttl: false,
            cmos_ratio: false,
            vil: None,
            vih: None,
            schmitt: None,
            nominal: 0.0,
            io_max: None,
            delay: 0.0,
            slot_base: 0,
            delays,
            extra: Vec::new(),
        }
    }

    /// srclkQh, rclkQ, srclrQh, oeEnable, oeDisable: every figure distinct so a
    /// wrong pick shows.
    fn ic_595() -> IcRecord {
        ic_of(kinds::IC_74HC595, 14, vec![(31e-9, 32e-9), (17e-9, 19e-9), (18e-9, 18e-9), (15e-9, 16e-9), (23e-9, 24e-9)])
    }

    /// The same part with `srclk` and `rclk` wired to one net.
    fn ic_595_tied() -> IcRecord {
        let mut ic = ic_595();
        ic.logic[2] = ic.logic[1];
        ic
    }

    /// clkQ, shldQ, dataQ.
    fn ic_165() -> IcRecord {
        ic_of(kinds::IC_74HC165, 14, vec![(15e-9, 16e-9), (20e-9, 21e-9), (13e-9, 14e-9)])
    }

    /// 595 reads in pin order: `/srclr srclk rclk /oe ser` then the undriven rest.
    fn r595(srclr_n: bool, srclk: bool, rclk: bool, oe_n: bool, ser: bool) -> Vec<bool> {
        let mut r = vec![srclr_n, srclk, rclk, oe_n, ser];
        r.extend([false; 9]);
        r
    }

    /// Defaults: not clearing, output enabled, no clock.
    fn idle595(srclk: bool, rclk: bool, ser: bool) -> Vec<bool> {
        r595(true, srclk, rclk, false, ser)
    }

    /// 165 reads in pin order: `/pl cp cp_inh ds d0..d7`, then the undriven pair.
    fn r165(pl_n: bool, cp: bool, inh: bool, ds: bool, d: u8) -> Vec<bool> {
        let mut r = vec![pl_n, cp, inh, ds];
        r.extend((0..8).map(|i| (d >> i) & 1 == 1));
        r.extend([false; 2]);
        r
    }

    struct Rig595 {
        ic: IcRecord,
        st: Hc595State,
        last: Option<Vec<bool>>,
        out: FamilyOutput,
        /// The instant of the latest evaluation; `step` moves to the next one.
        t: f64,
    }

    impl Rig595 {
        fn new(ic: IcRecord) -> Self {
            let st = Hc595State::power_on(&ic);
            Rig595 { ic, st, last: None, out: FamilyOutput::default(), t: 0.0 }
        }
        fn step_at(&mut self, reads: Vec<bool>, live: bool, now: f64) -> &FamilyOutput {
            self.t = now;
            evaluate_595(&self.ic, &mut self.st, &reads, self.last.as_deref(), live, now, &mut self.out);
            self.last = Some(reads);
            &self.out
        }
        /// A live evaluation at the next instant (a microsecond on).
        fn step(&mut self, reads: Vec<bool>) -> &FamilyOutput {
            let now = self.t + 1e-6;
            self.step_at(reads, true, now)
        }
        /// A live evaluation in the same instant as the last (another delta).
        fn same_instant(&mut self, reads: Vec<bool>) -> &FamilyOutput {
            let now = self.t;
            self.step_at(reads, true, now)
        }
        /// `ser` presented, then one full SRCLK pulse, three instants.
        fn shift_in(&mut self, ser: bool) {
            self.step(idle595(false, false, ser));
            self.step(idle595(true, false, ser));
            self.step(idle595(false, false, ser));
        }
        fn latch_pulse(&mut self) {
            self.step(idle595(false, true, false));
            self.step(idle595(false, false, false));
        }
    }

    struct Rig165 {
        ic: IcRecord,
        st: Hc165State,
        last: Option<Vec<bool>>,
        out: FamilyOutput,
    }

    impl Rig165 {
        fn new() -> Self {
            let ic = ic_165();
            let st = Hc165State::power_on(&ic);
            Rig165 { ic, st, last: None, out: FamilyOutput::default() }
        }
        fn step_live(&mut self, reads: Vec<bool>, live: bool) -> &FamilyOutput {
            evaluate_165(&self.ic, &mut self.st, &reads, self.last.as_deref(), live, 0.0, &mut self.out);
            self.last = Some(reads);
            &self.out
        }
        fn step(&mut self, reads: Vec<bool>) -> &FamilyOutput {
            self.step_live(reads, true)
        }
    }

    fn modes(out: &FamilyOutput) -> Vec<Mode> {
        out.requests.iter().map(|r| r.0).collect()
    }

    fn latch_modes(latch: u8) -> Vec<Mode> {
        (0..8).map(|b| if (latch >> b) & 1 == 1 { Mode::High } else { Mode::Low }).collect()
    }

    // ---- 74hc595 -----------------------------------------------------------

    #[test]
    fn hc595_powers_on_empty_with_eight_outputs_and_no_qh2() {
        let mut rig = Rig595::new(ic_595());
        assert_eq!((rig.st.shift, rig.st.latch), (0, 0));
        let out = rig.step(idle595(false, false, false)).clone();
        assert_eq!(out.requests.len(), 8, "Q-O58: QH' is listed but never driven");
        assert_eq!(out.slots, vec![0.0; 8]);
        assert_eq!(modes(&out), latch_modes(0));
    }

    #[test]
    fn hc595_shifts_ser_in_on_a_rising_srclk_only() {
        let mut rig = Rig595::new(ic_595());
        rig.step(idle595(false, false, true));
        assert_eq!(rig.st.shift, 0, "no clock edge, no shift");
        rig.step(idle595(true, false, true));
        assert_eq!(rig.st.shift, 0b1, "a rising SRCLK shifts SER into bit 0 (qa)");
        rig.step(idle595(true, false, false));
        assert_eq!(rig.st.shift, 0b1, "a steady-high clock does not shift again");
        rig.step(idle595(false, false, false));
        assert_eq!(rig.st.shift, 0b1, "a falling edge does not shift");
        rig.shift_in(false);
        rig.shift_in(true);
        assert_eq!(rig.st.shift, 0b101, "newest bit is qa, older bits move toward qh");
    }

    #[test]
    fn hc595_outputs_hold_the_latch_until_rclk_rises() {
        let mut rig = Rig595::new(ic_595());
        for b in [true, false, true, true] {
            rig.shift_in(b);
        }
        assert_eq!(rig.st.shift, 0b1011);
        let out = rig.step(idle595(false, false, false)).clone();
        assert_eq!(out.slots, vec![0.0; 8], "shifting alone does not reach the outputs");
        assert_eq!(rig.st.latch, 0);
        let out = rig.step_at(idle595(false, true, false), true, 1.0).clone();
        assert_eq!(rig.st.latch, 0b1011, "a rising RCLK stores the shift register");
        assert_eq!(out.slots, vec![1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0], "qa..qh");
        assert_eq!(modes(&out), latch_modes(0b1011));
        rig.step(idle595(false, false, false));
        rig.shift_in(true);
        assert_eq!(rig.st.latch, 0b1011, "a later shift leaves the stored byte alone");
    }

    #[test]
    fn hc595_eight_shifts_fill_it_in_order_the_first_bit_lands_in_qh() {
        let mut rig = Rig595::new(ic_595());
        // Serial 1,0,0,0,0,0,0,1 (first bit first): the first ends in qh.
        for b in [true, false, false, false, false, false, false, true] {
            rig.shift_in(b);
        }
        rig.latch_pulse();
        assert_eq!(rig.st.latch, 0b1000_0001);
        assert_eq!(rig.out.slots[7], 1.0, "the first bit shifted in reaches qh after eight clocks");
        assert_eq!(rig.out.slots[0], 1.0);
    }

    #[test]
    fn hc595_srclr_low_clears_the_shift_register_only_and_does_not_shift() {
        let mut rig = Rig595::new(ic_595());
        for _ in 0..3 {
            rig.shift_in(true);
        }
        rig.latch_pulse();
        assert_eq!((rig.st.shift, rig.st.latch), (0b111, 0b111));
        // /srclr asserted with a rising SRCLK and SER high: cleared, no shift.
        rig.step(r595(false, true, false, false, true));
        assert_eq!(rig.st.shift, 0, "an asserted /srclr clears at once, and that evaluation does not shift");
        assert_eq!(rig.st.latch, 0b111, "the storage register is not cleared");
        // The clock edge was consumed during the clear: releasing /srclr with SRCLK still high does not shift.
        rig.step(r595(true, true, false, false, true));
        assert_eq!(rig.st.shift, 0, "an edge that arrived while a clear held is consumed");
    }

    #[test]
    fn hc595_an_rclk_during_a_clear_stores_zero() {
        let mut rig = Rig595::new(ic_595());
        for _ in 0..4 {
            rig.shift_in(true);
        }
        rig.step(r595(false, false, false, false, false));
        rig.step(r595(false, false, true, false, false));
        assert_eq!((rig.st.shift, rig.st.latch), (0, 0), "RCLK during a clear latches the cleared register");
    }

    #[test]
    fn hc595_oe_high_tri_states_all_eight_outputs_and_the_slots_keep_the_register() {
        let mut rig = Rig595::new(ic_595());
        for _ in 0..2 {
            rig.shift_in(true);
        }
        rig.latch_pulse();
        let out = rig.step(r595(true, false, false, true, false)).clone();
        assert_eq!(modes(&out), vec![Mode::Hiz; 8]);
        assert_eq!(out.slots, vec![1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "the slots show the register, not the pin view");
        let out = rig.step(r595(true, false, false, false, false)).clone();
        assert_eq!(modes(&out), latch_modes(0b11), "/oe low drives the stored byte again");
    }

    #[test]
    fn hc595_oe_is_live_even_when_edges_are_frozen_and_state_is_untouched() {
        let mut rig = Rig595::new(ic_595());
        for _ in 0..2 {
            rig.shift_in(true);
        }
        rig.latch_pulse();
        let before = rig.st;
        // A frozen pass presenting a rising SRCLK, a rising RCLK and /oe high.
        let out = rig.step_at(r595(true, true, true, true, true), false, 2.0).clone();
        assert_eq!(rig.st, before, "no edge is acted on and no last-clock level is written");
        assert_eq!(modes(&out), vec![Mode::Hiz; 8], "the /oe path is live");
    }

    #[test]
    fn hc595_delays_rclkq_for_levels_and_the_oe_pairs_for_enable_and_disable() {
        let mut rig = Rig595::new(ic_595());
        let out = rig.step(idle595(false, false, false)).clone();
        let d = out.requests[0].1;
        assert_eq!((d.lh, d.hl), (17e-9, 19e-9), "rclkQ");
        assert_eq!((d.zh, d.zl), (15e-9, 16e-9), "oeEnable");
        assert_eq!((d.hz, d.lz), (23e-9, 24e-9), "oeDisable");
        assert!(out.requests.iter().all(|r| r.1 == d), "every Q shares one delay set");
    }

    #[test]
    fn hc595_tied_clocks_store_the_register_from_before_this_edge_shift() {
        let mut rig = Rig595::new(ic_595_tied());
        // Three tied edges with ser high. Tied: srclk/rclk read once from reads[1].
        let tied = |clk: bool| idle595(clk, false, true);
        for _ in 0..3 {
            rig.step(tied(true));
            rig.step(tied(false));
        }
        assert_eq!(rig.st.shift, 0b111);
        assert_eq!(rig.st.latch, 0b011, "the storage register trails the shift register by one clock (SCLS041J 8.1)");
    }

    #[test]
    fn hc595_tied_clocks_read_the_net_once_even_if_the_rclk_read_disagrees() {
        let mut rig = Rig595::new(ic_595_tied());
        // reads[2] (rclk) low while reads[1] (srclk) rises: tied, so one clock.
        rig.step(idle595(true, false, true));
        assert_eq!(rig.st.shift, 1);
        assert!(rig.st.last_rclk, "last_rclk follows the single tied read");
    }

    #[test]
    fn hc595_tied_clocks_with_srclr_held_low_from_an_earlier_instant_store_zero() {
        let mut rig = Rig595::new(ic_595_tied());
        for _ in 0..3 {
            rig.step(idle595(true, false, true));
            rig.step(idle595(false, false, true));
        }
        rig.step(r595(false, false, false, false, true));
        rig.step(r595(false, true, false, false, true));
        assert_eq!((rig.st.shift, rig.st.latch), (0, 0), "an edge on a register already cleared stores 0");
    }

    #[test]
    fn hc595_clocks_on_ground_or_open_pins_are_not_tied() {
        let mut ic = ic_595();
        ic.logic[1] = pin(0);
        ic.logic[2] = pin(0);
        let mut rig = Rig595::new(ic);
        rig.shift_in(true);
        rig.step(idle595(false, true, false));
        assert_eq!(rig.st.latch, 1, "two ground pins read apart: separate-clock rule");
        let mut ic = ic_595_tied();
        ic.logic[1].open = true;
        ic.logic[2].open = true;
        let mut rig = Rig595::new(ic);
        rig.shift_in(true);
        rig.step(idle595(false, true, false));
        assert_eq!(rig.st.latch, 1, "two open pins float apart: separate-clock rule");
    }

    // R61 (a): every capture within one instant samples the state as of the
    // instant's start. The register moves only after tpd, so a same-instant
    // RCLK stores what the register held before the instant.

    #[test]
    fn hc595_separate_clocks_rising_in_one_evaluation_store_the_pre_shift_byte() {
        let mut rig = Rig595::new(ic_595());
        rig.shift_in(true);
        rig.step_at(idle595(true, true, true), true, 1.0);
        assert_eq!(rig.st.shift, 0b11);
        assert_eq!(rig.st.latch, 0b1, "both stages sample pre-edge data: the store is the byte from before the instant");
    }

    #[test]
    fn hc595_coincident_clocks_in_either_delta_order_store_the_pre_shift_byte() {
        // SRCLK's delta first, RCLK's second, then the other way round: one answer.
        let mut a = Rig595::new(ic_595());
        a.shift_in(true);
        a.step_at(idle595(true, false, true), true, 1.0);
        a.same_instant(idle595(true, true, true));
        let mut b = Rig595::new(ic_595());
        b.shift_in(true);
        b.step_at(idle595(false, true, true), true, 1.0);
        b.same_instant(idle595(true, true, true));
        assert_eq!((a.st.shift, a.st.latch), (0b11, 0b1), "SRCLK led within the instant");
        assert_eq!((b.st.shift, b.st.latch), (0b11, 0b1), "RCLK led within the instant");
    }

    #[test]
    fn hc595_an_rclk_edge_one_microsecond_before_srclk_stores_the_old_register() {
        let mut rig = Rig595::new(ic_595());
        rig.shift_in(true);
        rig.step_at(idle595(false, true, true), true, 1.0);
        assert_eq!(rig.st.latch, 0b1, "RCLK rose first: it stores what the register held");
        rig.step_at(idle595(true, true, true), true, 1.0 + 1e-6);
        assert_eq!(rig.st.shift, 0b11, "the later SRCLK still shifts");
        assert_eq!(rig.st.latch, 0b1, "datasheet order: a later instant does not rewrite the stored byte");
    }

    #[test]
    fn hc595_an_srclk_edge_one_microsecond_before_rclk_stores_the_shifted_byte() {
        let mut rig = Rig595::new(ic_595());
        rig.shift_in(true);
        rig.step_at(idle595(true, false, true), true, 1.0);
        rig.step_at(idle595(true, true, true), true, 1.0 + 1e-6);
        assert_eq!(rig.st.latch, 0b11, "SRCLK led by a whole instant: the shift is already in the register");
    }

    #[test]
    fn hc595_rclk_and_srclr_in_either_delta_order_of_one_instant_store_the_same_byte() {
        let mut results = Vec::new();
        for rclk_first in [true, false] {
            let mut rig = Rig595::new(ic_595());
            for _ in 0..3 {
                rig.shift_in(true);
            }
            assert_eq!(rig.st.shift, 0b111);
            // One instant: RCLK rises and /SRCLR asserts, in two deltas.
            let (first, second) = (idle595(false, true, false), r595(false, false, true, false, false));
            let (first, second) = if rclk_first { (first, second) } else { (r595(false, false, false, false, false), r595(false, false, true, false, false)) };
            rig.step_at(first, true, 1.0);
            rig.same_instant(second);
            results.push((rig.st.shift, rig.st.latch));
        }
        assert_eq!(results[0], results[1], "delta order inside an instant never changes the capture");
        assert_eq!(results[0], (0, 0b111), "the clear reaches the shift register, not a same-instant capture of it");
    }

    #[test]
    fn hc595_an_rclk_rise_and_fall_then_an_srclk_rise_in_one_instant_do_not_rewrite_the_latch() {
        let mut rig = Rig595::new(ic_595());
        rig.shift_in(true);
        rig.shift_in(true);
        rig.step_at(idle595(false, true, true), true, 1.0);
        assert_eq!(rig.st.latch, 0b11);
        rig.same_instant(idle595(false, false, true));
        rig.same_instant(idle595(true, false, true));
        assert_eq!(rig.st.shift, 0b111, "the shift happened");
        assert_eq!(rig.st.latch, 0b11, "and the stored byte is still the pre-instant register");
    }

    #[test]
    fn hc595_a_clear_in_the_same_instant_as_a_tied_edge_does_not_reach_the_store() {
        let mut rig = Rig595::new(ic_595_tied());
        for _ in 0..3 {
            rig.step(idle595(true, false, true));
            rig.step(idle595(false, false, true));
        }
        rig.step(r595(false, true, false, false, true));
        assert_eq!(rig.st.shift, 0, "the clear took the shift register");
        assert_eq!(rig.st.latch, 0b111, "the capture sampled the register as of the instant's start");
    }

    #[test]
    fn hc595_a_shift_samples_ser_as_of_the_instants_start() {
        let mut rig = Rig595::new(ic_595());
        rig.step(idle595(false, false, false));
        // SER rises in the same instant as SRCLK, in either delta: the shifted bit is the old SER.
        rig.step_at(idle595(false, false, true), true, 1.0);
        rig.same_instant(idle595(true, false, true));
        assert_eq!(rig.st.shift, 0, "SER was low until this instant");
        rig.step(idle595(false, false, true));
        rig.step(idle595(true, false, true));
        assert_eq!(rig.st.shift, 0b01, "SER high since the earlier instant shifts a one");
        // And in a single delta that carries both changes.
        let mut one = Rig595::new(ic_595());
        one.step(idle595(false, false, false));
        one.step_at(idle595(true, false, true), true, 1.0);
        assert_eq!(one.st.shift, 0);
    }

    #[test]
    fn hc595_clear_before_the_instant_is_the_snapshot_the_next_capture_sees() {
        let mut rig = Rig595::new(ic_595());
        for _ in 0..3 {
            rig.shift_in(true);
        }
        rig.step(r595(false, false, false, false, false));
        rig.step(r595(true, false, true, false, false));
        assert_eq!(rig.st.latch, 0, "/SRCLR in an earlier instant emptied the register the RCLK then stores");
    }

    #[test]
    fn hc595_a_frozen_pass_leaves_the_clock_history_alone() {
        let mut rig = Rig595::new(ic_595());
        rig.step_at(idle595(true, false, true), false, 0.0);
        assert_eq!(rig.st.shift, 0);
        assert!(!rig.st.last_srclk, "last_srclk is a live-evaluation write only");
        rig.step_at(idle595(true, false, true), true, 0.0);
        assert_eq!(rig.st.shift, 1, "the first live evaluation still sees the rising edge");
    }

    // ---- 74hc165 -----------------------------------------------------------

    #[test]
    fn hc165_async_load_copies_d0_to_d7_into_the_register_and_q7_is_d7() {
        let mut rig = Rig165::new();
        let out = rig.step(r165(false, false, false, false, 0b1000_0101)).clone();
        assert_eq!(rig.st.shift, 0b1000_0101);
        assert_eq!(out.slots, vec![1.0, 0.0], "q7 and /q7");
        assert_eq!(modes(&out), vec![Mode::High, Mode::Low]);
        // Level-sensitive: data moving while /pl is low reloads.
        let out = rig.step(r165(false, false, false, false, 0b0000_0001)).clone();
        assert_eq!(rig.st.shift, 0b0000_0001);
        assert_eq!(out.slots, vec![0.0, 1.0]);
        assert_eq!(modes(&out), vec![Mode::Low, Mode::High]);
    }

    #[test]
    fn hc165_shifts_ds_in_at_bit_zero_toward_q7_on_a_rising_clock() {
        let mut rig = Rig165::new();
        rig.step(r165(false, false, false, false, 0b0100_0000));
        rig.step(r165(true, false, false, true, 0xFF));
        assert_eq!(rig.st.shift, 0b0100_0000, "released /pl, no clock: holds");
        let out = rig.step(r165(true, true, false, true, 0xFF)).clone();
        assert_eq!(rig.st.shift, 0b1000_0001, "shift left, ds into bit 0");
        assert_eq!(out.slots, vec![1.0, 0.0], "q7 is the old bit 6 now");
        rig.step(r165(true, false, false, false, 0));
        rig.step(r165(true, true, false, false, 0));
        assert_eq!(rig.st.shift, 0b0000_0010, "the old bit 7 fell off the end");
    }

    #[test]
    fn hc165_a_clock_edge_with_cp_inh_high_is_ignored_and_consumed() {
        let mut rig = Rig165::new();
        rig.step(r165(false, false, false, false, 0b0000_0110));
        rig.step(r165(true, false, true, true, 0));
        rig.step(r165(true, true, true, true, 0));
        assert_eq!(rig.st.shift, 0b0000_0110, "Q-O61: the inhibited edge does not clock");
        rig.step(r165(true, true, false, true, 0));
        assert_eq!(rig.st.shift, 0b0000_0110, "releasing cp_inh alone never clocks: the edge was consumed");
        rig.step(r165(true, false, false, true, 0));
        rig.step(r165(true, true, false, true, 0));
        assert_eq!(rig.st.shift, 0b0000_1101, "the next real edge clocks");
    }

    #[test]
    fn hc165_load_wins_over_a_clock_edge() {
        let mut rig = Rig165::new();
        rig.step(r165(false, false, false, true, 0b0001_0000));
        rig.step(r165(false, true, false, true, 0b0001_0000));
        assert_eq!(rig.st.shift, 0b0001_0000, "with /pl low the clock edge shifts nothing");
    }

    #[test]
    fn hc165_a_frozen_pass_neither_loads_nor_shifts() {
        let mut rig = Rig165::new();
        let out = rig.step_live(r165(false, false, false, false, 0xFF), false).clone();
        assert_eq!(rig.st.shift, 0);
        assert!(!rig.st.last_clk);
        assert_eq!(out.slots, vec![0.0, 1.0], "the frozen view is the stored register");
    }

    #[test]
    fn hc165_delays_pick_the_cause_clkq_shldq_or_dataq() {
        // A counting shift: clkQ (15/16 ns).
        let mut rig = Rig165::new();
        rig.step(r165(false, false, false, false, 0b0100_0000));
        rig.step(r165(true, false, false, false, 0));
        let out = rig.step(r165(true, true, false, false, 0)).clone();
        assert_eq!((out.requests[0].1.lh, out.requests[0].1.hl), (15e-9, 16e-9), "clkQ");
        // /pl falling with d7 steady: shldQ (20/21).
        let mut rig = Rig165::new();
        rig.step(r165(true, false, false, false, 0b1000_0000));
        let out = rig.step(r165(false, false, false, false, 0b1000_0000)).clone();
        assert_eq!((out.requests[0].1.lh, out.requests[0].1.hl), (20e-9, 21e-9), "shldQ");
        // d7 moving while /pl is already low: dataQ (13/14).
        let out = rig.step(r165(false, false, false, false, 0)).clone();
        assert_eq!((out.requests[0].1.lh, out.requests[0].1.hl), (13e-9, 14e-9), "dataQ");
        // /pl falling together with a changed d7: the larger figure wins per direction.
        let mut rig = Rig165::new();
        rig.step(r165(true, false, false, false, 0));
        let out = rig.step(r165(false, false, false, false, 0b1000_0000)).clone();
        assert_eq!((out.requests[0].1.lh, out.requests[0].1.hl), (20e-9, 21e-9), "max of shldQ and dataQ");
        assert_eq!(out.requests[0].1, out.requests[1].1, "q7 and /q7 share one set");
    }

    #[test]
    fn hc165_a_165_feeds_a_595_over_a_serial_chain() {
        // Load 0xA5 into the 165, clock both eight times with the 165's q7 on the 595's SER.
        let mut src = Rig165::new();
        let mut dst = Rig595::new(ic_595());
        src.step(r165(false, false, false, false, 0xA5));
        src.step(r165(true, false, false, false, 0xA5));
        for k in 0..8 {
            let q7 = src.out.slots[0] == 1.0;
            let t = k as f64;
            dst.step_at(idle595(false, false, q7), true, t);
            dst.step_at(idle595(true, false, q7), true, t + 0.1);
            dst.step_at(idle595(false, false, q7), true, t + 0.2);
            src.step(r165(true, true, false, false, 0));
            src.step(r165(true, false, false, false, 0));
        }
        dst.latch_pulse();
        assert_eq!(dst.st.latch, 0xA5, "the 165's d7 goes first and ends in qh: the byte arrives intact");
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Hc595State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shift, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.latch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_srclk, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_rclk, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.instant, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shift_pre, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ser_pre, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ser_last, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Hc165State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shift, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_clk, w); }
        n
    }
}
