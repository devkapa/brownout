//! The `cd4511` BCD-to-7-segment latch/decoder/driver (code 60): W1h item H2c.
//! H0 fixed the record layout, the delay pairs and the reader check; H2c added
//! the state and the evaluator.
//!
//! Classic: `digital-ics.ts:813-883`, `digital.ts:631-658` (`evalCD4511`), spec
//! chapter 3 and Q-O9, Q-O12, D30. Rulings that bind H2c: Q-O12 (an open `le`
//! is hashed 50/50, as classic reads it), R56 10 (the Data row, 660 ns
//! low-to-high / 520 ns high-to-low, for the segment outputs; `le` stays on the
//! package delay, param 12, until a figure is verified), R58 (a) (a change
//! caused by `bl_n` or `lt_n` takes its own verified pair). `lt_n` and `bl_n` act
//! within the same solve in classic (`IC_OPEN_HIGH_PINS` makes an open one read
//! HIGH); in the core that is a read at the instant the pin crosses, with no
//! delay on the read itself, and the segment then moves after that cause's pair.
//!
//! The code sits in the combinational range (60) but the part is stateful (the
//! BCD latch), so it registers in the sequential family: [`handles`] feeds
//! `digital::is_sequential_ic`.

use super::{FamilyOutput, IcRecord};
use crate::drivers::DelaySet;
use crate::events::Mode;
use crate::kinds;

pub fn handles(kind: u32) -> bool {
    kind == kinds::IC_CD4511
}

/// `cd4511` logic pins after `vcc`, `gnd`: `lt_n bl_n le` (read, controls),
/// `a b c d` (read, BCD data, bit 0 first), then the segment outputs
/// `a_out b_out c_out d_out e_out f_out g_out` (driven). Digital slots: the
/// seven segment pins, in that order, keyed `<id>/<pin>`.
pub const PINS_CD4511: [&str; 14] = ["lt_n", "bl_n", "le", "a", "b", "c", "d", "a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"];
pub const OUT_CD4511: [usize; 7] = [7, 8, 9, 10, 11, 12, 13];

/// Delay-tail pair order (R56 10 for `dataSeg`, R58 (a) for the other two, all from `w1-datasheet-delays.md`
/// section 8): `dataSeg` (Data to segment: `dH` = 660 ns, `dL` = 520 ns), `blSeg` (BL to segment: 400 / 350 ns) and
/// `ltSeg` (LT to segment: 150 / 250 ns).
pub const PAIRS_CD4511: [&str; 3] = ["dataSeg", "blSeg", "ltSeg"];

/// The driven pins of the decoder, `&[]` for any other kind.
pub fn output_capable(kind: u32) -> &'static [usize] {
    match kind {
        kinds::IC_CD4511 => &OUT_CD4511,
        _ => &[],
    }
}

/// Reader check: the register family's (`schmitt` and `OPEN_COLLECTOR` clear,
/// delay values `>= 0`; the segments are push-pull, `digital.ts` note).
pub(crate) fn check(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    kinds::seq_check(pin_count, params, blob)
}

/// R58 (a): a change caused by BL or LT takes its own verified pair instead of the Data pair. `false` would put
/// every cause on the Data pair; the const exists so the ruling is one line to reverse.
pub const OWN_BL_LT_PAIRS: bool = true;

/// `cd4511` state: classic's `{latchedBcd, lastLE}` (`defaultIcState`, both 0). Plain values, so the kernel's
/// clone-based journal rolls it back with the rest of `SeqState`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cd4511State {
    latched: u8,
    last_le: bool,
}

impl Cd4511State {
    pub fn power_on(_ic: &IcRecord) -> Self {
        Cd4511State { latched: 0, last_le: false }
    }

    /// The BCD value the latch holds (0-15).
    pub fn latched(&self) -> u8 {
        self.latched
    }

    /// W2c's carry layout: `[latched, last_le]`.
    pub const CARRY_LEN: usize = 2;

    pub fn carry_values(&self, out: &mut Vec<f64>) {
        out.extend([f64::from(self.latched), crate::carry::ic::bit(self.last_le)]);
    }

    pub fn carry_seed(&mut self, v: &[f64]) -> Result<(), String> {
        if !(v[0].fract() == 0.0 && (0.0..=15.0).contains(&v[0])) {
            return Err(format!("carry: a 4511's latch {} is out of range", v[0]));
        }
        self.latched = v[0] as u8;
        self.last_le = crate::carry::ic::bit_of(v[1])?;
        Ok(())
    }
}

/// Classic's `CD4511_SEGMENTS` (`digital.ts:351-379`) as bit masks, `a` in bit 0 through `g` in bit 6. 10-15 blank
/// every segment (the CD4511 shows no partial glyphs, unlike the 74ls47).
const GLYPHS: [u8; 16] = [
    0b0111111, // 0: a b c d e f
    0b0000110, // 1: b c
    0b1011011, // 2: a b d e g
    0b1001111, // 3: a b c d g
    0b1100110, // 4: b c f g
    0b1101101, // 5: a c d f g
    0b1111100, // 6: c d e f g (no top bar, no b)
    0b0000111, // 7: a b c
    0b1111111, // 8
    0b1100111, // 9: a b c f g (no bottom bar)
    0, 0, 0, 0, 0, 0,
];

/// `evalCD4511` (`digital.ts:631-658`): lamp test beats blanking beats the latched glyph.
#[inline]
fn segments(latched: u8, lt_low: bool, bl_low: bool) -> u8 {
    if lt_low {
        0x7f
    } else if bl_low {
        0
    } else {
        GLYPHS[(latched & 0xf) as usize]
    }
}

/// The larger figure per direction over `causes` (delays point 3, "with several causes the larger figure for that
/// direction wins"); the kind's first pair when no cause was identified (never zero).
fn combine(causes: &[(f64, f64)], first: (f64, f64)) -> (f64, f64) {
    match causes.split_first() {
        None => first,
        Some((&head, rest)) => rest.iter().fold(head, |acc, c| (acc.0.max(c.0), acc.1.max(c.1))),
    }
}

/// One evaluation of a powered cd4511 (the `sequential::evaluate_into` contract). `reads` is in pin order
/// (`lt_n bl_n le a b c d`, then the unread driven half), `last_reads` the same at the previous powered evaluation.
///
/// Classic's split (`digital-ics.ts:813-883`): `updateDigital` moves the latch (transparent while `le` is low, one
/// capture on its rising edge, held otherwise) and the stamp reads the PREVIOUS step's latch with `lt_n`/`bl_n` read
/// live (an open one reads HIGH through the pin's `open_high` flag). The core has no step to lag by, so a LIVE
/// evaluation targets the latch it has just moved, and a frozen settle pass (`!edges_live`) leaves the latch and
/// `last_le` alone and targets the stored one, which is exactly what classic's seed solve stamps. `lt_n`/`bl_n`
/// never wait for an edge: both are read at every evaluation, frozen or not.
///
/// Delay cause per segment (R56 10, R58 (a)): an input is a cause of a segment's change when that input's move alone
/// would already have changed the segment, or when holding that one move back would have kept the segment from
/// reaching its new level (for `le` and the data bits, "holding back" runs through the latch rule: `le` rising with
/// the data is no cause when the transparent latch reaches the same glyph). Data bits take `dataSeg`, `bl_n` `blSeg`,
/// `lt_n` `ltSeg`, and `le` the package delay (it has no datasheet figure: `w1-datasheet-delays.md` section 8). A
/// latch move needs data bits or `le` to have changed since the last read to have an identified cause; a segment
/// with no identified cause (including every segment at the first evaluation) takes `dataSeg`.
pub fn evaluate_cd4511(ic: &IcRecord, state: &mut Cd4511State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, _now: f64, out: &mut FamilyOutput) {
    let lt_low = !reads[0];
    let bl_low = !reads[1];
    let le = reads[2];
    let bcd = (reads[3] as u8) | (reads[4] as u8) << 1 | (reads[5] as u8) << 2 | (reads[6] as u8) << 3;

    let old_latched = state.latched;
    if edges_live {
        // Transparent while LE is low; on its rising edge the code present is captured; held while it stays high.
        if !le || !state.last_le {
            state.latched = bcd;
        }
        state.last_le = le;
    }

    let target = segments(state.latched, lt_low, bl_low);

    // Which inputs moved since the previous powered read.
    let (data_moved, le_moved, lt_old_low, bl_old_low) = match last_reads {
        Some(last) => (last[3..7] != reads[3..7], last[2] != le, !last[0], !last[1]),
        // No previous read: nothing is identified, every segment takes the first pair.
        None => (false, false, lt_low, bl_low),
    };
    let own = OWN_BL_LT_PAIRS;
    let lt_moved = lt_old_low != lt_low;
    let bl_moved = bl_old_low != bl_low;
    // The segments before this evaluation's moves, and with each input changed alone or held back alone. An input
    // causes a segment's change when applying only its move already changes the segment, or when holding only its
    // move back keeps the segment from reaching the new level (so two moves that each suffice are both causes, and
    // two that only work together are too).
    let old_target = segments(old_latched, lt_old_low, bl_old_low);
    let lt_only = segments(old_latched, lt_low, bl_old_low);
    let bl_only = segments(old_latched, lt_old_low, bl_low);
    let without_lt = segments(state.latched, lt_old_low, bl_low);
    let without_bl = segments(state.latched, lt_low, bl_old_low);

    // The latch under each of its two inputs' moves taken alone or held back. LE and the data bits can only be
    // told apart through the latch rule itself: with LE's move held back the old LE level meets this evaluation's
    // data (low is transparent so the new code, high holds the old latch), and with the data held back the new LE
    // level meets the old code (any transparent or capturing LE passes it, a held one keeps the old latch). A latch
    // that did not move (a frozen pass) has no latch cause at all.
    let (data_alone, data_back, le_alone, le_back) = match last_reads {
        Some(last) if edges_live => {
            let bcd_old = (last[3] as u8) | (last[4] as u8) << 1 | (last[5] as u8) << 2 | (last[6] as u8) << 3;
            let latch_le_back = if last[2] { old_latched } else { bcd };
            let latch_data_back = if le && last[2] { old_latched } else { bcd_old };
            (
                segments(latch_le_back, lt_old_low, bl_old_low) ^ old_target,
                segments(latch_data_back, lt_low, bl_low) ^ target,
                segments(latch_data_back, lt_old_low, bl_old_low) ^ old_target,
                segments(latch_le_back, lt_low, bl_low) ^ target,
            )
        }
        _ => (0, 0, 0, 0),
    };

    let first = ic.delays[0];
    let (bl_pair, lt_pair) = if own { (ic.delays[1], ic.delays[2]) } else { (first, first) };
    let package = (ic.delay, ic.delay);

    out.requests.clear();
    out.slots.clear();
    out.timers.clear();
    for i in 0..7 {
        let bit = 1u8 << i;
        // At most four: lt, bl, data and le can all be causes of one segment.
        let mut causes: [(f64, f64); 4] = [(0.0, 0.0); 4];
        let mut n = 0;
        let mut push = |pair: (f64, f64)| {
            causes[n] = pair;
            n += 1;
        };
        if lt_moved && ((lt_only ^ old_target) | (without_lt ^ target)) & bit != 0 {
            push(lt_pair);
        }
        if bl_moved && ((bl_only ^ old_target) | (without_bl ^ target)) & bit != 0 {
            push(bl_pair);
        }
        if data_moved && (data_alone | data_back) & bit != 0 {
            push(first);
        }
        if le_moved && (le_alone | le_back) & bit != 0 {
            push(package);
        }
        let (lh, hl) = combine(&causes[..n], first);
        let level = if target & bit != 0 { Mode::High } else { Mode::Low };
        // A segment is push-pull and never tri-states: the enable and disable figures carry the first pair.
        out.requests.push((level, DelaySet { lh, hl, zh: first.0, zl: first.1, hz: first.0, lz: first.1 }));
        out.slots.push((target >> i & 1) as f64);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::digital::IcPin;

    const DATA: (f64, f64) = (660e-9, 520e-9);
    const BL: (f64, f64) = (400e-9, 350e-9);
    const LT: (f64, f64) = (150e-9, 250e-9);
    /// Param 12, the package delay `le` stays on: distinct from every pair so a pick is unambiguous.
    const PACKAGE: f64 = 777e-9;

    fn ic() -> IcRecord {
        let no_pin = IcPin { node: 0, open: false, open_high: false, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false };
        IcRecord {
            kind: kinds::IC_CD4511,
            vcc: no_pin,
            gnd: no_pin,
            logic: Vec::new(),
            vcc_min: 0.0,
            vcc_max: 0.0,
            ttl: false,
            cmos_ratio: true,
            vil: None,
            vih: None,
            schmitt: None,
            nominal: 5.0,
            io_max: None,
            delay: PACKAGE,
            slot_base: 0,
            delays: vec![DATA, BL, LT],
            extra: Vec::new(),
        }
    }

    /// Reads in pin order: `lt_n bl_n le a b c d | a_out .. g_out` (the driven half reads false).
    fn reads(lt_n: bool, bl_n: bool, le: bool, bcd: u8) -> Vec<bool> {
        let mut r = vec![lt_n, bl_n, le, bcd & 1 != 0, bcd & 2 != 0, bcd & 4 != 0, bcd & 8 != 0];
        r.extend([false; 7]);
        r
    }

    fn run(st: &mut Cd4511State, r: &[bool], last: Option<&[bool]>, live: bool) -> FamilyOutput {
        let mut out = FamilyOutput::default();
        evaluate_cd4511(&ic(), st, r, last, live, 0.0, &mut out);
        out
    }

    fn levels(out: &FamilyOutput) -> [bool; 7] {
        let mut l = [false; 7];
        for (i, (mode, _)) in out.requests.iter().enumerate() {
            l[i] = match mode {
                Mode::High => true,
                Mode::Low => false,
                Mode::Hiz => panic!("a cd4511 segment never tri-states"),
            };
        }
        l
    }

    /// Classic's `CD4511_SEGMENTS` (digital.ts:351-379), `a` first, transcribed independently of the module's table.
    const GLYPHS: [&str; 16] = ["abcdef", "bc", "abdeg", "abcdg", "bcfg", "acdfg", "cdefg", "abc", "abcdefg", "abcfg", "", "", "", "", "", ""];

    fn glyph(g: &str) -> [bool; 7] {
        let mut l = [false; 7];
        for ch in g.chars() {
            l[(ch as u8 - b'a') as usize] = true;
        }
        l
    }

    #[test]
    fn the_truth_table_matches_classic_for_all_sixteen_codes() {
        for code in 0..16u8 {
            let mut st = Cd4511State::power_on(&ic());
            let out = run(&mut st, &reads(true, true, false, code), None, true);
            assert_eq!(levels(&out), glyph(GLYPHS[code as usize]), "BCD {code}");
            let slots: Vec<f64> = glyph(GLYPHS[code as usize]).iter().map(|&b| b as u8 as f64).collect();
            assert_eq!(out.slots, slots, "BCD {code}: one 0/1 slot per segment, in pin order");
            assert_eq!(out.requests.len(), 7);
            assert!(out.timers.is_empty());
        }
    }

    #[test]
    fn bcd_10_to_15_blank_every_segment_and_6_and_9_have_no_top_bar_and_no_bottom_bar() {
        for code in 10..16u8 {
            let mut st = Cd4511State::power_on(&ic());
            assert_eq!(levels(&run(&mut st, &reads(true, true, false, code), None, true)), [false; 7], "BCD {code}");
        }
        let mut st = Cd4511State::power_on(&ic());
        let six = levels(&run(&mut st, &reads(true, true, false, 6), None, true));
        assert!(!six[0] && !six[1], "6: neither a nor b");
        let nine = levels(&run(&mut st, &reads(true, true, false, 9), None, true));
        assert!(!nine[3], "9: no d");
    }

    #[test]
    fn lamp_test_lights_everything_and_beats_blanking_and_the_latch() {
        let mut st = Cd4511State::power_on(&ic());
        assert_eq!(levels(&run(&mut st, &reads(false, true, false, 1), None, true)), [true; 7]);
        assert_eq!(levels(&run(&mut st, &reads(false, false, false, 15), None, true)), [true; 7], "LT beats BL");
    }

    #[test]
    fn blanking_clears_every_segment_whatever_the_code() {
        let mut st = Cd4511State::power_on(&ic());
        assert_eq!(levels(&run(&mut st, &reads(true, false, false, 8), None, true)), [false; 7]);
    }

    #[test]
    fn lt_and_bl_act_live_even_in_a_frozen_pass_but_the_latch_does_not_move() {
        // The stamp reads the previous step's latch and the CURRENT lt/bl (spec 3.8): a frozen settle pass is the
        // same read, so it follows the live controls and leaves the stored latch alone.
        let mut st = Cd4511State::power_on(&ic());
        run(&mut st, &reads(true, true, false, 5), None, true);
        let before = st;
        assert_eq!(levels(&run(&mut st, &reads(false, true, false, 5), None, false)), [true; 7], "LT low in a frozen pass");
        assert_eq!(levels(&run(&mut st, &reads(true, false, false, 5), None, false)), [false; 7], "BL low in a frozen pass");
        // A BCD change in a frozen pass does not reach the latch, so the segments keep showing the stored 5.
        assert_eq!(levels(&run(&mut st, &reads(true, true, false, 1), None, false)), glyph("acdfg"));
        assert_eq!(st, before, "a frozen pass writes no state");
    }

    #[test]
    fn the_latch_is_transparent_while_le_is_low_and_holds_while_it_is_high() {
        let mut st = Cd4511State::power_on(&ic());
        run(&mut st, &reads(true, true, false, 3), None, true);
        assert_eq!(levels(&run(&mut st, &reads(true, true, false, 4), None, true)), glyph(GLYPHS[4]), "LE low follows the data");
        // LE rises: the code present is captured.
        assert_eq!(levels(&run(&mut st, &reads(true, true, true, 4), None, true)), glyph(GLYPHS[4]));
        // LE stays high: data changes are ignored.
        assert_eq!(levels(&run(&mut st, &reads(true, true, true, 9), None, true)), glyph(GLYPHS[4]), "held");
        assert_eq!(levels(&run(&mut st, &reads(true, true, true, 0), None, true)), glyph(GLYPHS[4]), "still held");
        // LE falls: transparent again.
        assert_eq!(levels(&run(&mut st, &reads(true, true, false, 0), None, true)), glyph(GLYPHS[0]));
    }

    #[test]
    fn a_held_latch_still_obeys_lt_and_bl_and_comes_back() {
        let mut st = Cd4511State::power_on(&ic());
        run(&mut st, &reads(true, true, false, 7), None, true);
        run(&mut st, &reads(true, true, true, 7), None, true);
        assert_eq!(levels(&run(&mut st, &reads(true, false, true, 2), None, true)), [false; 7], "blanked");
        assert_eq!(levels(&run(&mut st, &reads(true, true, true, 2), None, true)), glyph(GLYPHS[7]), "the latch came back");
    }

    #[test]
    fn le_high_at_the_first_evaluation_is_a_rising_edge_that_captures_the_code() {
        // Classic's lastLE starts 0 (defaultIcState), so a powered-on-high LE still captures once.
        let mut st = Cd4511State::power_on(&ic());
        assert_eq!(levels(&run(&mut st, &reads(true, true, true, 6), None, true)), glyph(GLYPHS[6]));
        assert_eq!(levels(&run(&mut st, &reads(true, true, true, 1), None, true)), glyph(GLYPHS[6]));
    }

    #[test]
    fn the_power_on_latch_is_zero() {
        let mut st = Cd4511State::power_on(&ic());
        // A frozen first pass (the load settle) stamps the seeded latch: digit 0.
        assert_eq!(levels(&run(&mut st, &reads(true, true, false, 9), None, false)), glyph(GLYPHS[0]));
    }

    fn delays_of(out: &FamilyOutput) -> Vec<(f64, f64)> {
        out.requests.iter().map(|(_, d)| (d.lh, d.hl)).collect()
    }

    #[test]
    fn a_data_change_takes_the_data_pair_on_every_segment() {
        let mut st = Cd4511State::power_on(&ic());
        let first = reads(true, true, false, 1);
        run(&mut st, &first, None, true);
        let out = run(&mut st, &reads(true, true, false, 2), Some(&first), true);
        for (i, d) in delays_of(&out).iter().enumerate() {
            assert_eq!(*d, DATA, "segment {i}");
        }
    }

    #[test]
    fn a_blanking_change_takes_the_bl_pair_and_a_lamp_test_change_the_lt_pair() {
        let mut st = Cd4511State::power_on(&ic());
        let base = reads(true, true, false, 8);
        run(&mut st, &base, None, true);
        let bl = reads(true, false, false, 8);
        let out = run(&mut st, &bl, Some(&base), true);
        assert!(delays_of(&out).iter().all(|d| *d == BL), "BL: {:?}", delays_of(&out));
        let lt = reads(false, false, false, 8);
        let out = run(&mut st, &lt, Some(&bl), true);
        // LT overrides the blank: the segments rise, and every one of them is caused by LT alone.
        assert!(delays_of(&out).iter().all(|d| *d == LT), "LT: {:?}", delays_of(&out));
        // LT released while BL still blanks: segments fall, LT's falling figure (tPHL 250 ns) applies.
        let released = reads(true, false, false, 8);
        let out = run(&mut st, &released, Some(&lt), true);
        assert!(delays_of(&out).iter().all(|d| *d == LT), "LT release: {:?}", delays_of(&out));
    }

    #[test]
    fn a_data_change_hidden_by_blanking_is_still_a_cause_where_it_alone_would_have_moved_the_segment() {
        // BCD 8 -> 1 with BL asserted at the same instant. Segment a (lit for 8, dark for 1) goes dark because of
        // either move, so it takes the larger of Data and BL; b and c are lit for both codes, so only BL moves them.
        let mut st = Cd4511State::power_on(&ic());
        let base = reads(true, true, false, 8);
        run(&mut st, &base, None, true);
        let out = run(&mut st, &reads(true, false, false, 1), Some(&base), true);
        let d = delays_of(&out);
        assert_eq!(d[0], (DATA.0.max(BL.0), DATA.1.max(BL.1)), "a");
        assert_eq!(d[1], BL, "b");
        assert_eq!(d[2], BL, "c");
    }

    #[test]
    fn a_change_with_two_causes_takes_the_larger_figure_per_direction() {
        // Distinct figures so the per-direction maximum is visible: data (100 up, 500 down), BL (300 up, 200 down).
        let mut rec = ic();
        rec.delays = vec![(100e-9, 500e-9), (300e-9, 200e-9), LT];
        let mut st = Cd4511State::power_on(&rec);
        let go = |st: &mut Cd4511State, r: &[bool], last: Option<&[bool]>| {
            let mut out = FamilyOutput::default();
            evaluate_cd4511(&rec, st, r, last, true, 0.0, &mut out);
            out
        };
        // BL low blanks the display under latch 6 (b dark); then BL releases and the code becomes 1 (b lit):
        // segment b rises because BL released AND because the code changed.
        let base = reads(true, false, false, 6);
        go(&mut st, &base, None);
        let out = go(&mut st, &reads(true, true, false, 1), Some(&base));
        let d = delays_of(&out);
        assert_eq!(d[1], (300e-9, 500e-9), "b: max(dH) of Data and BL, max(dL) of Data and BL");
    }

    #[test]
    fn the_le_cause_takes_the_package_delay_and_beats_nothing_it_has_no_figure_for() {
        // LE falling while the data differs from the latch: the latch changes, and only LE changed (R56 10).
        let mut st = Cd4511State::power_on(&ic());
        run(&mut st, &reads(true, true, false, 1), None, true);
        let held = reads(true, true, true, 1);
        run(&mut st, &held, Some(&reads(true, true, false, 1)), true);
        // The data lines moved while LE was high and the latch held: a separate evaluation, no output change.
        let moved = reads(true, true, true, 2);
        run(&mut st, &moved, Some(&held), true);
        let out = run(&mut st, &reads(true, true, false, 2), Some(&moved), true);
        // Segments a (1 -> 2: off to on) and d, e, g change; none had a Data change in THIS evaluation.
        let d = delays_of(&out);
        assert_eq!(d[0], (PACKAGE, PACKAGE), "a: LE is the cause");
        assert_eq!(d[3], (PACKAGE, PACKAGE), "d: LE is the cause");
        // Segments b (on, on) and f (off, off) do not change at all; they carry the first pair, never zero.
        assert_eq!(d[1], DATA);
    }

    #[test]
    fn le_and_data_moving_together_take_the_larger_of_the_data_pair_and_the_package_delay() {
        let mut st = Cd4511State::power_on(&ic());
        let held = reads(true, true, true, 1);
        run(&mut st, &held, None, true);
        let out = run(&mut st, &reads(true, true, false, 2), Some(&held), true);
        let d = delays_of(&out);
        assert_eq!(d[0], (DATA.0.max(PACKAGE), DATA.1.max(PACKAGE)));
    }

    #[test]
    fn four_causes_on_one_segment_take_the_larger_figure_per_direction_without_overflowing() {
        // lt_n, bl_n, the code (0 -> 1) and le all move in one live evaluation (the four-slot cause array must not
        // overflow however many of them count).
        let mut rec = ic();
        rec.delays = vec![(100e-9, 500e-9), (300e-9, 200e-9), (50e-9, 60e-9)];
        rec.delay = 900e-9;
        let mut st = Cd4511State::power_on(&rec);
        let base = reads(true, true, false, 0);
        let mut out = FamilyOutput::default();
        evaluate_cd4511(&rec, &mut st, &base, None, true, 0.0, &mut out);
        evaluate_cd4511(&rec, &mut st, &reads(false, false, true, 1), Some(&base), true, 0.0, &mut out);
        let d = delays_of(&out);
        // Segment a ends lit through LT. LT, BL and the data move each matter to it; LE rising with the data is no
        // cause (the transparent latch reaches the same code), so the package delay (900, 900) stays out and the
        // figure is the larger per direction of Data (100, 500), BL (300, 200) and LT (50, 60).
        assert_eq!(d[0], (300e-9, 500e-9));
    }

    #[test]
    fn le_rising_with_the_data_is_no_cause_when_the_transparent_latch_reaches_the_same_glyph() {
        // BCD 1 -> 2 and LE rising in one evaluation: holding LE low (transparent) captures the same code 2, so LE
        // changes nothing and the Data pair alone applies, not the (larger) package delay.
        let mut st = Cd4511State::power_on(&ic());
        let base = reads(true, true, false, 1);
        run(&mut st, &base, None, true);
        let out = run(&mut st, &reads(true, true, true, 2), Some(&base), true);
        let d = delays_of(&out);
        assert_eq!(d[0], DATA, "a rises on the data change alone");
        assert_eq!(d[2], DATA, "c falls on the data change alone");
    }

    #[test]
    fn with_no_previous_read_no_cause_is_identified_and_the_first_pair_applies() {
        let mut st = Cd4511State::power_on(&ic());
        let out = run(&mut st, &reads(false, true, false, 5), None, true);
        assert!(delays_of(&out).iter().all(|d| *d == DATA));
    }

    #[test]
    fn a_segment_never_asks_for_a_tristate_and_fills_the_enable_figures_with_the_first_pair() {
        let mut st = Cd4511State::power_on(&ic());
        let out = run(&mut st, &reads(true, true, false, 5), None, true);
        for (_, d) in &out.requests {
            assert_eq!((d.zh, d.zl, d.hz, d.lz), (DATA.0, DATA.1, DATA.0, DATA.1));
        }
    }

    #[test]
    fn the_whole_state_is_a_plain_value_so_a_failed_instant_rolls_back_by_clone() {
        let mut st = Cd4511State::power_on(&ic());
        run(&mut st, &reads(true, true, false, 5), None, true);
        let saved = st;
        run(&mut st, &reads(true, true, true, 5), None, true);
        run(&mut st, &reads(true, true, false, 9), None, true);
        assert_ne!(st, saved);
        let st = saved;
        assert_eq!(st.latched(), 5);
    }

    #[test]
    fn the_tables_agree_with_the_record_shape() {
        let shape = kinds::shape(kinds::IC_CD4511).expect("shape");
        assert_eq!(shape.pins, 2 + PINS_CD4511.len());
        assert_eq!((shape.digital_slots)(shape.pins), OUT_CD4511.len(), "one slot per segment");
        assert_eq!(shape.params, kinds::ic_param::COUNT + 2 * PAIRS_CD4511.len());
        assert_eq!(kinds::delay_pairs(kinds::IC_CD4511), PAIRS_CD4511.len());
        assert_eq!(output_capable(kinds::IC_CD4511), &OUT_CD4511);
        let driven: Vec<&str> = OUT_CD4511.iter().map(|&i| PINS_CD4511[i]).collect();
        assert_eq!(driven, ["a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"]);
    }

    #[test]
    fn the_decoder_registers_as_a_sequential_ic() {
        assert!(handles(kinds::IC_CD4511));
        assert!(!handles(kinds::IC_74HC74));
        assert!(super::super::is_sequential_ic(kinds::IC_CD4511));
        assert!(super::super::is_ic(kinds::IC_CD4511));
        assert!(OWN_BL_LT_PAIRS, "R58 (a): BL and LT carry their own pairs");
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Cd4511State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.latched, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_le, w); }
        n
    }
}
