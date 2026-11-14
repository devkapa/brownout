//! The register family: 74ls161, 74ls173, 74hc74 (core/README.md, "Kernel
//! semantics: sequential ICs (W1d)"). Owned by W1d worker D1.
//!
//! Each kind keeps its own state record (contract point 2) and is evaluated
//! by its own function below, dispatched on `ic.kind` by [`evaluate`]. Every
//! per-kind function follows the same shape (contract point 3): (a) the
//! caller has already read every logic pin into `reads`; (b) when
//! `edges_live`, apply the kind's update priority order (README point 5) and
//! write `lastClk`, entirely skipped otherwise; (c) compute this
//! evaluation's driven targets (README point 6) from the (possibly just
//! updated) state and the CURRENT live reads — this is what still moves
//! during a frozen settle pass, since (c) never checks `edges_live`; (d)
//! build one `DelaySet` per output-capable pin from the cause table (delays
//! point 3) and return the per-pin requests plus the slot readout (README
//! point 8).

use super::{FamilyOutput, IcRecord};
use crate::drivers::DelaySet;
use crate::events::Mode;
use crate::kinds;

/// Register state for one 161/173/74hc74 device (contract point 2), one
/// variant per kind. `Clone` so `devices::ics::IcEntry` (which owns one per
/// device) rolls back with a failed instant, same as every other piece of
/// per-device evaluator state (`core/README.md`, "A failed advance").
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SeqState {
    /// 74ls161: the 4-bit count (0-15) and the clock's read level at the
    /// device's last LIVE evaluation.
    Ls161 { count: u8, last_clk: bool },
    /// 74ls173: the four register bits and the clock's last read level.
    Ls173 { q: [bool; 4], last_clk: bool },
    /// 74hc74: both flip-flops' Q and (explicitly stored, so `Q = Q̄ = 1`
    /// can be represented) Q̄, and each unit's own clock history.
    Hc74 { q1: bool, q1n: bool, last_clk1: bool, q2: bool, q2n: bool, last_clk2: bool },
}

impl SeqState {
    /// W2c's carry values (`crate::carry`'s IC layout, sequential tag): a
    /// 161's count and last clock, a 173's four Q and last clock, a 74hc74's
    /// two flip-flops (Q, /Q, last clock each).
    pub fn carry_values(&self, out: &mut Vec<f64>) {
        use crate::carry::ic::bit;
        match *self {
            SeqState::Ls161 { count, last_clk } => out.extend([f64::from(count), bit(last_clk)]),
            SeqState::Ls173 { q, last_clk } => {
                out.extend(q.iter().map(|&b| bit(b)));
                out.push(bit(last_clk));
            }
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => out.extend([q1, q1n, last_clk1, q2, q2n, last_clk2].map(bit)),
        }
    }

    /// W2c's seed: [`Self::carry_values`]' inverse, at load.
    pub fn carry_seed(&mut self, v: &[f64]) -> Result<(), String> {
        use crate::carry::ic::bit_of;
        match self {
            SeqState::Ls161 { count, last_clk } => {
                if !(v[0].fract() == 0.0 && (0.0..=15.0).contains(&v[0])) {
                    return Err(format!("carry: a 161's count {} is out of range", v[0]));
                }
                *count = v[0] as u8;
                *last_clk = bit_of(v[1])?;
            }
            SeqState::Ls173 { q, last_clk } => {
                for (k, b) in q.iter_mut().enumerate() {
                    *b = bit_of(v[k])?;
                }
                *last_clk = bit_of(v[4])?;
            }
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => {
                for (k, b) in [q1, q1n, last_clk1, q2, q2n, last_clk2].into_iter().enumerate() {
                    *b = bit_of(v[k])?;
                }
            }
        }
        Ok(())
    }

    /// Power-on values (contract point 2: classic's `defaultIcState`, all 0
    /// except `q1n = q2n = 1`).
    pub fn power_on(ic: &IcRecord) -> Self {
        match ic.kind {
            kinds::IC_74LS161 => SeqState::Ls161 { count: 0, last_clk: false },
            kinds::IC_74LS173 => SeqState::Ls173 { q: [false; 4], last_clk: false },
            kinds::IC_74HC74 => SeqState::Hc74 { q1: false, q1n: true, last_clk1: false, q2: false, q2n: true, last_clk2: false },
            other => panic!("SeqState::power_on: {other} is not a sequential IC kind (70-72)"),
        }
    }
}

/// Combines the delay-tail pairs of every cause that applies to one
/// transition into a single `(dH, dL)` pair, taking the larger figure
/// independently per direction (delays point 3: "with several causes the
/// larger figure for that direction wins"). An empty `causes` (no identified
/// cause) falls back to `first` — the kind's first pair, never 0 (point 3:
/// "with no identified cause, the kind's first pair applies").
fn combine_causes(causes: &[(f64, f64)], first: (f64, f64)) -> (f64, f64) {
    if causes.is_empty() {
        first
    } else {
        let dh = causes.iter().map(|c| c.0).fold(f64::NEG_INFINITY, f64::max);
        let dl = causes.iter().map(|c| c.1).fold(f64::NEG_INFINITY, f64::max);
        (dh, dl)
    }
}

/// 74LS161: 4-bit synchronous counter, asynchronous clear (README "Kernel
/// semantics: sequential ICs (W1d)" points 5-6; classic `digital-ics.ts`
/// ~335-397). Logic pins (README's sequential table): `/clr clk /load enp
/// ent a b c d` (0-8, read), `qa qb qc qd rco` (9-13, driven). Delay tail
/// (README's "IC delay tail"): `clkQ, clrQ, clkRco, entRco`.
#[cfg(test)]
fn eval_161(ic: &IcRecord, count: &mut u8, last_clk: &mut bool, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool) -> FamilyOutput {
    let mut out = FamilyOutput::default();
    eval_161_into(ic, count, last_clk, reads, last_reads, edges_live, &mut out);
    out
}

fn eval_161_into(ic: &IcRecord, count: &mut u8, last_clk: &mut bool, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, out: &mut FamilyOutput) {
    let clr_low = !reads[0];
    let clk_now = reads[1];
    let load_low = !reads[2];
    let enp_high = reads[3];
    let ent_high = reads[4];
    let bits = [reads[5], reads[6], reads[7], reads[8]];

    let old_count = *count;
    let mut clear_applied = false;
    let mut edge_applied = false;

    if edges_live {
        let rising = clk_now && !*last_clk;
        if clr_low {
            *count = 0;
            clear_applied = true;
        } else if rising {
            edge_applied = true;
            if load_low {
                let mut loaded = 0u8;
                for (i, &b) in bits.iter().enumerate() {
                    if b {
                        loaded |= 1 << i;
                    }
                }
                *count = loaded;
            } else if enp_high && ent_high {
                *count = (*count + 1) & 0xF;
            }
            // else: hold (ENP/ENT not both high).
        }
        // else: hold (no rising edge).
        // "lastClk is written at every live evaluation whatever branch
        // applies, so an edge that arrives while a clear holds is consumed"
        // (README point 4) — this write is unconditional within the
        // edges_live block, even on the clr_low branch above.
        *last_clk = clk_now;
    }

    let new_count = *count;
    let count_changed = new_count != old_count;

    // Q cause: clrQ when the clear changed the count, else clkQ (delays
    // point 3's table; clkQ is also this kind's first pair, the fallback
    // for "no identified cause").
    let q_causes: &[(f64, f64)] = if clear_applied && count_changed { std::slice::from_ref(&ic.delays[1]) } else { &[] };
    let (q_lh, q_hl) = combine_causes(q_causes, ic.delays[0]);

    // RCO cause: clkRco when the count changed on an edge; clrQ on a clear;
    // entRco when this evaluation's ENT read differs from lastReads (target
    // point 6 reads ENT live every evaluation, so this can fire even while
    // `edges_live` is false). These are gathered independently, not as an
    // either/or chain: a counting edge and an ENT change can land in the
    // same evaluation (e.g. ENT rising together with the clock), and delays
    // point 3's "several causes: the larger figure per direction wins" must
    // see both.
    let ent_changed = last_reads.is_some_and(|lr| lr[4] != ent_high);
    let mut rco_causes = [(0.0, 0.0); 3];
    let mut rco_cause_count = 0;
    if edge_applied && count_changed {
        rco_causes[rco_cause_count] = ic.delays[2]; // clkRco
        rco_cause_count += 1;
    }
    if clear_applied {
        rco_causes[rco_cause_count] = ic.delays[1]; // clrQ
        rco_cause_count += 1;
    }
    if ent_changed {
        rco_causes[rco_cause_count] = ic.delays[3]; // entRco
        rco_cause_count += 1;
    }
    let (rco_lh, rco_hl) = combine_causes(&rco_causes[..rco_cause_count], ic.delays[0]);

    // qa..qd and rco are always driven (no enable pin on a 161): fill the
    // unselectable zh/zl/hz/lz slots with the kind's first pair, never 0
    // (delays point 3's closing rule) — the FSM never actually picks them,
    // since these outputs never request HIZ.
    let filler = ic.delays[0];
    let q_delay = DelaySet { lh: q_lh, hl: q_hl, zh: filler.0, zl: filler.1, hz: filler.0, lz: filler.1 };
    let rco_delay = DelaySet { lh: rco_lh, hl: rco_hl, zh: filler.0, zl: filler.1, hz: filler.0, lz: filler.1 };

    // Target (point 6): qa..qd from the count bits, rco from count==15 && a
    // LIVE ENT read ("ENP never gates RCO").
    let qa = (new_count & 1) != 0;
    let qb = (new_count & 2) != 0;
    let qc = (new_count & 4) != 0;
    let qd = (new_count & 8) != 0;
    let rco = new_count == 15 && ent_high;

    let level = |b: bool| if b { Mode::High } else { Mode::Low };
    out.requests.clear();
    // Pin order (output_capable): qa qb qc qd rco.
    out.requests.extend([(level(qa), q_delay), (level(qb), q_delay), (level(qc), q_delay), (level(qd), q_delay), (level(rco), rco_delay)]);
    out.slots.clear();
    // Slot order (README): the bare count, then qa..qd — no rco slot.
    out.slots.extend([new_count as f64, qa as u8 as f64, qb as u8 as f64, qc as u8 as f64, qd as u8 as f64]);
    out.timers.clear();
}

/// 74LS173: quad D register, 3-state (README points 5-6; classic
/// `digital-ics.ts` ~399-450). Logic pins: `/clr clk m n g1 g2 d1 d2 d3 d4`
/// (0-9, read), `q1 q2 q3 q4` (10-13, driven). The pin named `/clr` is
/// tested active HIGH (classic, matching the real 74LS173's active-HIGH
/// CLR). Delay tail: `clkQ, clrQ, mnEnable, mnDisable`.
#[cfg(test)]
fn eval_173(ic: &IcRecord, q: &mut [bool; 4], last_clk: &mut bool, reads: &[bool], edges_live: bool) -> FamilyOutput {
    let mut out = FamilyOutput::default();
    eval_173_into(ic, q, last_clk, reads, edges_live, &mut out);
    out
}

fn eval_173_into(ic: &IcRecord, q: &mut [bool; 4], last_clk: &mut bool, reads: &[bool], edges_live: bool, out: &mut FamilyOutput) {
    let clr_high = reads[0];
    let clk_now = reads[1];
    let m_high = reads[2];
    let n_high = reads[3];
    let g1_low = !reads[4];
    let g2_low = !reads[5];
    let d = [reads[6], reads[7], reads[8], reads[9]];

    let old_q = *q;
    let mut clear_applied = false;

    if edges_live {
        let rising = clk_now && !*last_clk;
        if clr_high {
            *q = [false; 4];
            clear_applied = true;
        } else if rising && g1_low && g2_low {
            *q = d;
        }
        // else: hold. M and N do not enter the update (README point 5:
        // "M and N do not enter the update").
        *last_clk = clk_now;
    }

    let q_changed = *q != old_q;
    let causes: &[(f64, f64)] = if clear_applied && q_changed { std::slice::from_ref(&ic.delays[1]) } else { &[] };
    let (lh, hl) = combine_causes(causes, ic.delays[0]);
    let mn_enable = ic.delays[2];
    let mn_disable = ic.delays[3];
    let delay = DelaySet { lh, hl, zh: mn_enable.0, zl: mn_enable.1, hz: mn_disable.0, lz: mn_disable.1 };

    // Target (point 6): qi = bit i when M and N both read 0, else HIZ —
    // recomputed from the LIVE reads every evaluation, edges_live or not
    // (README point 7: "the live paths live... 173 M/N").
    let tri_stated = m_high || n_high;
    let level = |bit: bool| if tri_stated { Mode::Hiz } else if bit { Mode::High } else { Mode::Low };

    out.requests.clear();
    out.requests.extend([(level(q[0]), delay), (level(q[1]), delay), (level(q[2]), delay), (level(q[3]), delay)]);
    out.slots.clear();
    // The register, not the tri-stated pin view (README point 8: "the 173
    // shows its register while M or N tri-states the pins").
    out.slots.extend([q[0] as u8 as f64, q[1] as u8 as f64, q[2] as u8 as f64, q[3] as u8 as f64]);
    out.timers.clear();
}

/// One 74HC74 flip-flop's update (contract point 5) and target (point 6).
/// LIVE (`edges_live`): runs the full priority chain against the STATE
/// (`q`, `qn`, `last_clk`) — contract point 3(b) — and the returned target
/// is simply the post-update state. FROZEN (`!edges_live`): point 3 "skip
/// (b) entirely" and point 6's 74hc74 row, "the stored Q and Q̄ with point
/// 5's asynchronous priorities applied from the current reads" — the
/// asynchronous priorities (1-3 only, never the clock capture) are applied
/// to a COPY for the returned target; `q`/`qn`/`last_clk` are left
/// untouched (classic's own 1 ps seed solve stamps this override without
/// ever writing `icState`). Returns `(q_target, qn_target, async_applied)`,
/// `async_applied` the `preClrQ` vs `clkQ` cause (delays point 3).
#[allow(clippy::too_many_arguments)]
fn eval_hc74_unit(clr_low: bool, pre_low: bool, clk_now: bool, d: bool, edges_live: bool, q: &mut bool, qn: &mut bool, last_clk: &mut bool) -> (bool, bool, bool) {
    if edges_live {
        let rising = clk_now && !*last_clk;
        if clr_low && pre_low {
            // Both asserted: Q = Q̄ = 1 (datasheet: "not allowed", but
            // reproduced exactly as classic stores it).
            *q = true;
            *qn = true;
        } else if clr_low {
            *q = false;
            *qn = true;
        } else if pre_low {
            *q = true;
            *qn = false;
        } else if rising {
            *q = d;
            *qn = !d;
        }
        // else: hold.
        *last_clk = clk_now;
        (*q, *qn, clr_low || pre_low)
    } else if clr_low && pre_low {
        (true, true, true)
    } else if clr_low {
        (false, true, true)
    } else if pre_low {
        (true, false, true)
    } else {
        // hold — the frozen-settle target follows the stored value, whose
        // OWN cells are never touched here.
        (*q, *qn, false)
    }
}

/// 74HC74: dual D flip-flop with asynchronous /PRE, /CLR (README points 5-6;
/// classic `digital-ics.ts` ~615-753). Logic pins: `clr1_n d1 clk1 pre1_n`
/// (0-3), `q1 q1_n q2_n q2` (4-7, driven — NOT the order the slots use),
/// `pre2_n clk2 d2 clr2_n` (8-11). Delay tail: `clkQ, preClrQ`.
#[cfg(test)]
#[allow(clippy::too_many_arguments)]
fn eval_74hc74(
    ic: &IcRecord,
    q1: &mut bool,
    q1n: &mut bool,
    last_clk1: &mut bool,
    q2: &mut bool,
    q2n: &mut bool,
    last_clk2: &mut bool,
    reads: &[bool],
    edges_live: bool,
) -> FamilyOutput {
    let mut out = FamilyOutput::default();
    eval_74hc74_into(ic, q1, q1n, last_clk1, q2, q2n, last_clk2, reads, edges_live, &mut out);
    out
}

#[allow(clippy::too_many_arguments)]
fn eval_74hc74_into(
    ic: &IcRecord,
    q1: &mut bool,
    q1n: &mut bool,
    last_clk1: &mut bool,
    q2: &mut bool,
    q2n: &mut bool,
    last_clk2: &mut bool,
    reads: &[bool],
    edges_live: bool,
    out: &mut FamilyOutput,
) {
    let clr1_low = !reads[0];
    let d1 = reads[1];
    let clk1_now = reads[2];
    let pre1_low = !reads[3];
    let pre2_low = !reads[8];
    let clk2_now = reads[9];
    let d2 = reads[10];
    let clr2_low = !reads[11];

    let (q1_target, q1n_target, async1) = eval_hc74_unit(clr1_low, pre1_low, clk1_now, d1, edges_live, q1, q1n, last_clk1);
    let (q2_target, q2n_target, async2) = eval_hc74_unit(clr2_low, pre2_low, clk2_now, d2, edges_live, q2, q2n, last_clk2);

    // Neither flip-flop's output ever tri-states: fill zh/zl/hz/lz with the
    // kind's first pair (clkQ), same reasoning as the 161's Q/RCO.
    let filler = ic.delays[0];
    let delay_for = |async_applied: bool| {
        let causes: &[(f64, f64)] = if async_applied { std::slice::from_ref(&ic.delays[1]) } else { &[] };
        let (lh, hl) = combine_causes(causes, ic.delays[0]);
        DelaySet { lh, hl, zh: filler.0, zl: filler.1, hz: filler.0, lz: filler.1 }
    };
    let d1_delay = delay_for(async1);
    let d2_delay = delay_for(async2);

    let level = |b: bool| if b { Mode::High } else { Mode::Low };
    out.requests.clear();
    // Pin order (output_capable): q1 q1_n q2_n q2 -- the TARGET (the live
    // async override during a frozen pass, contract point 6), not
    // necessarily the stored state.
    out.requests.extend([(level(q1_target), d1_delay), (level(q1n_target), d1_delay), (level(q2n_target), d2_delay), (level(q2_target), d2_delay)]);
    out.slots.clear();
    // Slot order (README, "classic's key order"): q1 q1_n q2 q2_n -- ALWAYS
    // the stored register (point 8), never the frozen-settle target, since
    // `eval_hc74_unit` never writes `q`/`qn` while frozen.
    out.slots.extend([*q1 as u8 as f64, *q1n as u8 as f64, *q2 as u8 as f64, *q2n as u8 as f64]);
    out.timers.clear();
}

/// One evaluation of a powered 161/173/74hc74 (contract point 3): `reads` is
/// this delta's logic-pin levels (in pin order), `last_reads` the same at
/// the IC's own previous powered evaluation (`None` before the first),
/// `edges_live` whether this call may act on an edge (`false` only during
/// `Kernel::settle`'s frozen fixed point — contract point 3(b): "skip (b)
/// entirely"), `now` the current simulation time (unused: none of these
/// three kinds schedules a timer).
pub fn evaluate(ic: &IcRecord, state: &mut SeqState, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64) -> FamilyOutput {
    let mut out = FamilyOutput::default();
    evaluate_into(ic, state, reads, last_reads, edges_live, now, &mut out);
    out
}

/// [`evaluate`], replacing the contents of `out` (the caller's reusable
/// buffers) instead of returning a fresh [`FamilyOutput`]: this runs once per
/// affected reader per delta, and the three vectors are the whole of its
/// allocation.
pub fn evaluate_into(ic: &IcRecord, state: &mut SeqState, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let _ = now;
    match (ic.kind, state) {
        (kinds::IC_74LS161, SeqState::Ls161 { count, last_clk }) => eval_161_into(ic, count, last_clk, reads, last_reads, edges_live, out),
        (kinds::IC_74LS173, SeqState::Ls173 { q, last_clk }) => eval_173_into(ic, q, last_clk, reads, edges_live, out),
        (kinds::IC_74HC74, SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 }) => {
            eval_74hc74_into(ic, q1, q1n, last_clk1, q2, q2n, last_clk2, reads, edges_live, out)
        }
        (kind, _) => panic!("sequential::evaluate: {kind} is not a sequential IC kind (70-72), or its state does not match"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::IcPin;

    fn ic_of(kind: u32, delays: Vec<(f64, f64)>) -> IcRecord {
        let no_pin = IcPin { node: 0, open: false, open_high: false, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false };
        IcRecord {
            kind,
            vcc: no_pin,
            gnd: no_pin,
            logic: Vec::new(),
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

    fn ic_161() -> IcRecord {
        // clkQ, clrQ, clkRco, entRco.
        ic_of(kinds::IC_74LS161, vec![(13e-9, 18e-9), (20e-9, 20e-9), (20e-9, 18e-9), (9e-9, 9e-9)])
    }

    fn ic_173() -> IcRecord {
        // clkQ, clrQ, mnEnable, mnDisable.
        ic_of(kinds::IC_74LS173, vec![(17e-9, 22e-9), (26e-9, 26e-9), (15e-9, 18e-9), (11e-9, 11e-9)])
    }

    fn ic_74hc74() -> IcRecord {
        // clkQ, preClrQ — kept distinct (unlike the datasheet's identical
        // typicals) so a cause-selection test can tell them apart.
        ic_of(kinds::IC_74HC74, vec![(20e-9, 20e-9), (25e-9, 30e-9)])
    }

    /// 161 reads, in pin order: `/clr clk /load enp ent a b c d`.
    fn reads_161(clr_n: bool, clk: bool, load_n: bool, enp: bool, ent: bool, a: bool, b: bool, c: bool, d: bool) -> Vec<bool> {
        vec![clr_n, clk, load_n, enp, ent, a, b, c, d, false, false, false, false, false]
    }

    /// 173 reads, in pin order: `/clr clk m n g1 g2 d1 d2 d3 d4`. `g1`/`g2`
    /// are the pins' own RAW read levels (both active-LOW enables in
    /// `eval_173`, so `false` here is the asserted/enabling level).
    fn reads_173(clr: bool, clk: bool, m: bool, n: bool, g1: bool, g2: bool, d1: bool, d2: bool, d3: bool, d4: bool) -> Vec<bool> {
        vec![clr, clk, m, n, g1, g2, d1, d2, d3, d4, false, false, false, false]
    }

    /// 74hc74 reads, in pin order: `clr1_n d1 clk1 pre1_n | pre2_n clk2 d2 clr2_n`.
    #[allow(clippy::too_many_arguments)]
    fn reads_hc74(clr1_n: bool, d1: bool, clk1: bool, pre1_n: bool, pre2_n: bool, clk2: bool, d2: bool, clr2_n: bool) -> Vec<bool> {
        vec![clr1_n, d1, clk1, pre1_n, false, false, false, false, pre2_n, clk2, d2, clr2_n]
    }

    fn count_of(out: &FamilyOutput) -> u8 {
        out.slots[0] as u8
    }

    // ---- 161: update priority order, wrap, RCO gating ---------------------

    #[test]
    fn ls161_async_clear_beats_a_rising_edge_and_zeroes_the_count() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 7, last_clk: false };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        // clr asserted (LOW) and the clock is rising: clear still wins.
        let out = eval_161(&ic, count, last_clk, &reads_161(false, true, true, true, true, true, true, true, true), None, true);
        assert_eq!(count_of(&out), 0, "an asserted /clr must clear even on a rising edge with /load and count-enable also asserted");
    }

    #[test]
    fn ls161_load_beats_the_count_enable_on_a_rising_edge() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 3, last_clk: false };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        // /load LOW and ENP/ENT both HIGH at once: load must win over count+1.
        let out = eval_161(&ic, count, last_clk, &reads_161(true, true, false, true, true, true, false, true, false), None, true);
        assert_eq!(count_of(&out), 0b0101, "a=1,b=0,c=1,d=0 loaded, not count+1");
    }

    #[test]
    fn ls161_counts_up_only_when_enp_and_ent_both_high_on_a_rising_edge() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 4, last_clk: false };
        {
            let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
            let out = eval_161(&ic, count, last_clk, &reads_161(true, true, true, false, true, false, false, false, false), None, true);
            assert_eq!(count_of(&out), 4, "ENP low must hold the count even on a rising edge");
        }
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        *last_clk = false; // fresh rising edge
        let out = eval_161(&ic, count, last_clk, &reads_161(true, true, true, true, true, false, false, false, false), None, true);
        assert_eq!(count_of(&out), 5, "ENP and ENT both high on a rising edge must count up");
    }

    #[test]
    fn ls161_holds_with_no_rising_edge() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 9, last_clk: true };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        // Clock already HIGH last time and still HIGH now: no rising edge.
        let out = eval_161(&ic, count, last_clk, &reads_161(true, true, true, true, true, true, true, true, true), None, true);
        assert_eq!(count_of(&out), 9, "no rising edge must hold the count");
    }

    #[test]
    fn ls161_wraps_from_15_to_0() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 15, last_clk: false };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        let out = eval_161(&ic, count, last_clk, &reads_161(true, true, true, true, true, false, false, false, false), None, true);
        assert_eq!(count_of(&out), 0, "(15+1) & 0xF must wrap to 0");
        assert_eq!(out.requests[4].0, Mode::Low, "rco must drop once the count leaves 15");
    }

    #[test]
    fn ls161_rco_is_gated_by_ent_but_never_by_enp() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 15, last_clk: true };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        // No rising edge (clock steady HIGH), ENP LOW, ENT HIGH: count holds
        // at 15 and rco must still be driven HIGH from ENT alone.
        let out = eval_161(&ic, count, last_clk, &reads_161(true, true, true, false, true, false, false, false, false), None, true);
        assert_eq!(count_of(&out), 15);
        assert_eq!(out.requests[4].0, Mode::High, "rco must follow ENT regardless of ENP");
    }

    // ---- 161: cause selection ----------------------------------------------

    #[test]
    fn ls161_q_cause_is_clrq_when_the_clear_changed_the_count() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 5, last_clk: false };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        let out = eval_161(&ic, count, last_clk, &reads_161(false, false, true, true, true, false, false, false, false), None, true);
        assert_eq!(out.requests[0].1.hl, ic.delays[1].1, "qa's HL delay must be clrQ's dL when the clear actually changed the count");
    }

    #[test]
    fn ls161_q_cause_is_clkq_on_a_counting_edge() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 0, last_clk: false };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        let out = eval_161(&ic, count, last_clk, &reads_161(true, true, true, true, true, false, false, false, false), None, true);
        assert_eq!(out.requests[0].1.lh, ic.delays[0].0, "qa's LH delay must be clkQ's dH on a counting edge");
    }

    #[test]
    fn ls161_rco_cause_is_entrco_when_only_ent_changed() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 15, last_clk: true };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        let last = reads_161(true, true, true, true, false, false, false, false, false);
        let now_reads = reads_161(true, true, true, true, true, false, false, false, false);
        let out = eval_161(&ic, count, last_clk, &now_reads, Some(&last), true);
        assert_eq!(out.requests[4].1.lh, ic.delays[3].0, "rco's LH delay must be entRco's dH when ENT alone changed");
    }

    #[test]
    fn ls161_rco_combines_clkrco_and_entrco_when_both_apply_in_the_same_evaluation() {
        // clkQ, clrQ, clkRco, entRco -- clkRco and entRco deliberately tuned
        // so the two directions' maxima come from DIFFERENT causes, proving
        // "the larger figure per direction" is taken independently per
        // direction rather than as a single whole-pair pick.
        let ic = ic_of(kinds::IC_74LS161, vec![(1e-9, 1e-9), (1e-9, 1e-9), (20e-9, 5e-9), (9e-9, 30e-9)]);
        let mut state = SeqState::Ls161 { count: 5, last_clk: false };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        let last = reads_161(true, true, true, true, false, false, false, false, false);
        // A rising edge that increments the count (clkRco applies) WITH ENT
        // also differing from lastReads (entRco applies too) -- e.g. ENT
        // rising together with the clock.
        let now_reads = reads_161(true, true, true, true, true, false, false, false, false);
        let out = eval_161(&ic, count, last_clk, &now_reads, Some(&last), true);
        assert_eq!(count_of(&out), 6, "sanity: the count did change on this edge, so clkRco really applies");
        assert_eq!(
            (out.requests[4].1.lh, out.requests[4].1.hl),
            (20e-9, 30e-9),
            "rco's delay must take clkRco's larger dH and entRco's larger dL independently, not just one cause's whole pair"
        );
    }

    #[test]
    fn ls161_unselectable_enable_slots_fall_back_to_the_first_pair_not_zero() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 0, last_clk: false };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        let out = eval_161(&ic, count, last_clk, &reads_161(true, false, true, true, true, false, false, false, false), None, true);
        let (dh, dl) = ic.delays[0];
        for &(_, delays) in &out.requests {
            assert_eq!((delays.zh, delays.zl, delays.hz, delays.lz), (dh, dl, dh, dl), "a 161 output never tri-states; the unused slots must be the kind's first pair, never 0");
        }
    }

    // ---- 161: edges, phantom edge, frozen settle ---------------------------

    #[test]
    fn ls161_an_edge_arriving_while_a_clear_holds_is_consumed() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 0, last_clk: false };
        {
            // /clr held LOW while the clock rises: cleared, but lastClk must
            // still be written so this same HIGH level cannot "rise" again.
            let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
            eval_161(&ic, count, last_clk, &reads_161(false, true, true, true, true, false, false, false, false), None, true);
        }
        // /clr released with the clock still HIGH: since lastClk is already
        // HIGH, this must NOT look like a fresh rising edge.
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        let out = eval_161(&ic, count, last_clk, &reads_161(true, true, true, true, true, true, false, false, false), None, true);
        assert_eq!(count_of(&out), 0, "the clock edge consumed under the clear must not also fire the count-enable branch on release");
    }

    #[test]
    fn ls161_phantom_edge_on_power_on_with_the_clock_already_high() {
        let ic = ic_161();
        let mut state = SeqState::power_on(&ic);
        // power_on's lastClk is false; a clock already reading HIGH on the
        // first live evaluation must count as a rising edge (README point
        // 7's "phantom rising edge").
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        let out = eval_161(&ic, count, last_clk, &reads_161(true, true, true, true, true, false, false, false, false), None, true);
        assert_eq!(count_of(&out), 1, "clock reading HIGH against a power-on lastClk of 0 must count as a rising edge");
    }

    #[test]
    fn ls161_frozen_settle_changes_no_state_but_rco_still_follows_live_ent() {
        let ic = ic_161();
        let mut state = SeqState::Ls161 { count: 15, last_clk: false };
        let (count, last_clk) = match &mut state { SeqState::Ls161 { count, last_clk } => (count, last_clk), _ => unreachable!() };
        // edges_live = false: a rising edge with ENP/ENT both high must NOT
        // increment (and must not write lastClk), but rco must still track
        // the live ENT read.
        let out = eval_161(&ic, count, last_clk, &reads_161(true, true, true, true, false, false, false, false, false), None, false);
        assert_eq!(count_of(&out), 15, "edges_live=false must change no state");
        assert!(!*last_clk, "edges_live=false must not write lastClk either");
        assert_eq!(out.requests[4].0, Mode::Low, "rco must still follow the live (frozen-settle) ENT read");
    }

    // ---- 173: update priority, active-HIGH clear, G1/G2, M/N tri-state ----

    #[test]
    fn ls173_active_high_clear_zeroes_the_register() {
        let ic = ic_173();
        let mut state = SeqState::Ls173 { q: [true; 4], last_clk: false };
        let (q, last_clk) = match &mut state { SeqState::Ls173 { q, last_clk } => (q, last_clk), _ => unreachable!() };
        let out = eval_173(&ic, q, last_clk, &reads_173(true, false, false, false, true, true, true, true, true, true), true);
        assert_eq!(&out.slots, &[0.0, 0.0, 0.0, 0.0], "/clr reading HIGH (active) must clear q1..q4");
    }

    #[test]
    fn ls173_loads_only_on_a_rising_edge_with_g1_and_g2_both_low() {
        let ic = ic_173();
        let mut state = SeqState::Ls173 { q: [false; 4], last_clk: false };
        let (q, last_clk) = match &mut state { SeqState::Ls173 { q, last_clk } => (q, last_clk), _ => unreachable!() };
        // g1/g2 both read LOW (asserted, active-low enables).
        let out = eval_173(&ic, q, last_clk, &reads_173(false, true, false, false, false, false, true, false, true, false), true);
        assert_eq!(&out.slots, &[1.0, 0.0, 1.0, 0.0], "g1/g2 both low on a rising edge must load d1..d4");
    }

    #[test]
    fn ls173_g1_high_blocks_the_load_even_on_a_rising_edge() {
        let ic = ic_173();
        let mut state = SeqState::Ls173 { q: [false; 4], last_clk: false };
        let (q, last_clk) = match &mut state { SeqState::Ls173 { q, last_clk } => (q, last_clk), _ => unreachable!() };
        // g1 reads HIGH (not asserted) while g2 reads LOW (asserted): only
        // g1 blocks the load.
        let out = eval_173(&ic, q, last_clk, &reads_173(false, true, false, false, true, false, true, true, true, true), true);
        assert_eq!(&out.slots, &[0.0, 0.0, 0.0, 0.0], "G1 high must block the load regardless of G2 and the rising edge");
    }

    #[test]
    fn ls173_m_or_n_tri_states_the_pins_but_the_slots_keep_the_register() {
        let ic = ic_173();
        let mut state = SeqState::Ls173 { q: [true, false, true, false], last_clk: false };
        let (q, last_clk) = match &mut state { SeqState::Ls173 { q, last_clk } => (q, last_clk), _ => unreachable!() };
        // M reads HIGH, N reads LOW: still disabled (M OR N).
        let out = eval_173(&ic, q, last_clk, &reads_173(false, false, true, false, true, true, false, false, false, false), true);
        assert_eq!(out.requests.iter().map(|r| r.0).collect::<Vec<_>>(), vec![Mode::Hiz, Mode::Hiz, Mode::Hiz, Mode::Hiz], "M or N high must tri-state every q pin");
        assert_eq!(&out.slots, &[1.0, 0.0, 1.0, 0.0], "the slots must still show the stored register while the pins are tri-stated");
    }

    #[test]
    fn ls173_mn_enable_disable_delays_are_used_for_the_hiz_transitions() {
        let ic = ic_173();
        let mut state = SeqState::Ls173 { q: [false; 4], last_clk: false };
        let (q, last_clk) = match &mut state { SeqState::Ls173 { q, last_clk } => (q, last_clk), _ => unreachable!() };
        let out = eval_173(&ic, q, last_clk, &reads_173(false, false, false, false, true, true, false, false, false, false), true);
        let (zh, zl) = ic.delays[2];
        let (hz, lz) = ic.delays[3];
        assert_eq!((out.requests[0].1.zh, out.requests[0].1.zl), (zh, zl), "zh/zl must be mnEnable");
        assert_eq!((out.requests[0].1.hz, out.requests[0].1.lz), (hz, lz), "hz/lz must be mnDisable");
    }

    // ---- 74hc74: priority order, both-asserted, frozen settle -------------

    #[test]
    fn hc74_both_pre_and_clr_low_makes_q_and_qbar_both_high_and_it_survives_release() {
        let ic = ic_74hc74();
        let mut state = SeqState::Hc74 { q1: false, q1n: true, last_clk1: false, q2: false, q2n: true, last_clk2: false };
        {
            let (q1, q1n, lc1, q2, q2n, lc2) = match &mut state {
                SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => (q1, q1n, last_clk1, q2, q2n, last_clk2),
                _ => unreachable!(),
            };
            let out = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(false, false, false, false, true, false, false, true), true);
            assert_eq!((out.slots[0], out.slots[1]), (1.0, 1.0), "both /PRE and /CLR asserted must drive Q = Q_bar = 1");
        }
        // Release both (still no clock edge): the both-asserted state must
        // hold until the next rising edge, not snap back to Q=0.
        let (q1, q1n, lc1, q2, q2n, lc2) = match &mut state {
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => (q1, q1n, last_clk1, q2, q2n, last_clk2),
            _ => unreachable!(),
        };
        let out = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(true, false, false, true, true, false, false, true), true);
        assert_eq!((out.slots[0], out.slots[1]), (1.0, 1.0), "releasing /PRE and /CLR together must hold Q=Q_bar=1 until the next clock edge");
    }

    #[test]
    fn hc74_clr_alone_beats_a_simultaneous_rising_edge() {
        let ic = ic_74hc74();
        let mut state = SeqState::Hc74 { q1: true, q1n: false, last_clk1: false, q2: false, q2n: true, last_clk2: false };
        let (q1, q1n, lc1, q2, q2n, lc2) = match &mut state {
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => (q1, q1n, last_clk1, q2, q2n, last_clk2),
            _ => unreachable!(),
        };
        // /CLR1 low, D1 high, and the clock rising: clear must still win.
        let out = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(false, true, true, true, true, false, false, true), true);
        assert_eq!((out.slots[0], out.slots[1]), (0.0, 1.0), "/CLR must beat a rising-edge D capture");
    }

    #[test]
    fn hc74_captures_d_only_on_a_rising_edge() {
        let ic = ic_74hc74();
        let mut state = SeqState::Hc74 { q1: false, q1n: true, last_clk1: false, q2: false, q2n: true, last_clk2: false };
        let (q1, q1n, lc1, q2, q2n, lc2) = match &mut state {
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => (q1, q1n, last_clk1, q2, q2n, last_clk2),
            _ => unreachable!(),
        };
        let out = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(true, true, true, true, true, false, false, true), true);
        assert_eq!((out.slots[0], out.slots[1]), (1.0, 0.0), "Q1 must capture D1=1 on the rising edge, Q1_n its complement");
    }

    #[test]
    fn hc74_holds_with_no_rising_edge_and_no_async_priority() {
        let ic = ic_74hc74();
        let mut state = SeqState::Hc74 { q1: true, q1n: false, last_clk1: true, q2: false, q2n: true, last_clk2: true };
        let (q1, q1n, lc1, q2, q2n, lc2) = match &mut state {
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => (q1, q1n, last_clk1, q2, q2n, last_clk2),
            _ => unreachable!(),
        };
        let out = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(true, false, true, true, true, true, false, true), true);
        assert_eq!(out.slots, vec![1.0, 0.0, 0.0, 1.0], "no rising edge and no asserted /PRE or /CLR must hold");
    }

    #[test]
    fn hc74_frozen_settle_drives_the_request_without_writing_any_state() {
        let ic = ic_74hc74();
        let before = SeqState::Hc74 { q1: false, q1n: true, last_clk1: true, q2: false, q2n: true, last_clk2: false };
        let mut state = before;
        let (q1, q1n, lc1, q2, q2n, lc2) = match &mut state {
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => (q1, q1n, last_clk1, q2, q2n, last_clk2),
            _ => unreachable!(),
        };
        // edges_live=false, /PRE1 asserted: the REQUEST must still show the
        // live override (Q1=1, Q1_n=0 -- point 6's "the stored Q and Q̄ with
        // point 5's asynchronous priorities applied from the current
        // reads"), but nothing may be WRITTEN (point 3: "skip (b)
        // entirely"; classic's own 1 ps seed solve stamps this override and
        // never writes icState) -- the slots (the stored register, point 8)
        // and the whole state, lastClk included, must be untouched.
        let out = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(true, false, true, false, true, false, false, true), false);
        assert_eq!((out.requests[0].0, out.requests[1].0), (Mode::High, Mode::Low), "a live /PRE must still drive the REQUEST during a frozen settle pass");
        assert_eq!((out.slots[0], out.slots[1]), (0.0, 1.0), "the slots must keep the stored register, not the frozen-settle target");
        assert_eq!(state, before, "a frozen pass must write no state at all -- not q1/q1n, not lastClk1 -- even for the flip-flop it overrides");
    }

    #[test]
    fn hc74_two_frozen_passes_never_leak_into_the_next_live_read_of_q1() {
        let ic = ic_74hc74();
        let mut state = SeqState::power_on(&ic); // q1 = false, q1n = true.
        let (q1, q1n, lc1, q2, q2n, lc2) = match &mut state {
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => (q1, q1n, last_clk1, q2, q2n, last_clk2),
            _ => unreachable!(),
        };
        // Frozen pass 1: /PRE1 asserted (low) -- the request must show
        // Q1=1 even though the stored Q1 is still 0.
        let out1 = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(true, false, true, false, true, false, false, true), false);
        assert_eq!(out1.requests[0].0, Mode::High, "a live /PRE must drive the request high on the first frozen pass");
        // Frozen pass 2: /PRE1 released (high), nothing else asserted -- the
        // request must fall back to the still-untouched stored Q1=0.
        let out2 = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(true, false, true, true, true, false, false, true), false);
        assert_eq!(out2.requests[0].0, Mode::Low, "with /PRE released, the frozen target must fall back to the untouched stored Q1=0");
        // One live pass, clock steady (no rising edge), nothing asserted:
        // Q1 must still read 0 -- neither frozen pass may have written its
        // override into icState.
        let out3 = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(true, false, false, true, true, false, false, true), true);
        assert_eq!(out3.slots[0], 0.0, "Q1 must still be 0: neither frozen pass may have persisted the /PRE override");
    }

    #[test]
    fn hc74_cause_is_preclrq_under_an_asynchronous_priority_else_clkq() {
        let ic = ic_74hc74();
        let mut state = SeqState::Hc74 { q1: false, q1n: true, last_clk1: false, q2: false, q2n: true, last_clk2: false };
        {
            let (q1, q1n, lc1, q2, q2n, lc2) = match &mut state {
                SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => (q1, q1n, last_clk1, q2, q2n, last_clk2),
                _ => unreachable!(),
            };
            let out = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(false, false, false, true, true, false, false, true), true);
            assert_eq!(out.requests[0].1.hl, ic.delays[1].1, "an asserted /CLR must select preClrQ");
        }
        let (q1, q1n, lc1, q2, q2n, lc2) = match &mut state {
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => (q1, q1n, last_clk1, q2, q2n, last_clk2),
            _ => unreachable!(),
        };
        let out = eval_74hc74(&ic, q1, q1n, lc1, q2, q2n, lc2, &reads_hc74(true, true, true, true, true, false, false, true), true);
        assert_eq!(out.requests[0].1.lh, ic.delays[0].0, "a plain clock-edge capture must select clkQ");
    }

    // ---- combine_causes: the several-causes maximum ------------------------

    #[test]
    fn combine_causes_picks_the_larger_figure_independently_per_direction() {
        let a = (10e-9, 50e-9);
        let b = (30e-9, 20e-9);
        let (dh, dl) = combine_causes(&[a, b], (0.0, 0.0));
        assert_eq!((dh, dl), (30e-9, 50e-9), "each direction's max must be taken independently, not a whole-pair pick");
    }

    #[test]
    fn combine_causes_falls_back_to_the_first_pair_when_nothing_applies() {
        let first = (13e-9, 18e-9);
        assert_eq!(combine_causes(&[], first), first, "no identified cause must fall back to the kind's first pair");
    }

    // ---- power_on -----------------------------------------------------------

    #[test]
    fn power_on_matches_classic_default_ic_state() {
        assert_eq!(SeqState::power_on(&ic_161()), SeqState::Ls161 { count: 0, last_clk: false });
        assert_eq!(SeqState::power_on(&ic_173()), SeqState::Ls173 { q: [false; 4], last_clk: false });
        assert_eq!(
            SeqState::power_on(&ic_74hc74()),
            SeqState::Hc74 { q1: false, q1n: true, last_clk1: false, q2: false, q2n: true, last_clk2: false }
        );
    }
    #[test]
    fn evaluate_into_over_one_reused_output_matches_evaluate_across_kinds_states_and_reads() {
        // One `FamilyOutput` for all three kinds and many passes, each pass
        // checked against a fresh `evaluate` from a copy of the same state:
        // neither the request/slot vectors' leftovers nor their lengths may
        // reach a result, and the state each leaves behind must be the same.
        let kinds_and_records = [(ic_161(), 14usize), (ic_173(), 14), (ic_74hc74(), 12)];
        let mut seed = 0x1234_5679u32;
        let mut bit = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed & 0x40 != 0
        };
        let junk = FamilyOutput { requests: vec![(Mode::High, DelaySet::uniform(1.0)); 9], slots: vec![7.0; 9], timers: vec![(1.0, 2)] };
        let mut reused = junk.clone();
        for (ic, len) in &kinds_and_records {
            let mut state = SeqState::power_on(ic);
            let mut last: Option<Vec<bool>> = None;
            for pass in 0..300 {
                let reads: Vec<bool> = (0..*len).map(|_| bit()).collect();
                let edges_live = pass % 7 != 0;
                let mut fresh_state = state;
                let expected = evaluate(ic, &mut fresh_state, &reads, last.as_deref(), edges_live, 0.0);
                if pass % 2 == 0 {
                    // Every other pass starts from leftovers no evaluation
                    // wrote; the rest reuse what the previous one left.
                    reused = junk.clone();
                }
                evaluate_into(ic, &mut state, &reads, last.as_deref(), edges_live, 0.0, &mut reused);
                assert_eq!(reused, expected, "kind {}, pass {pass}", ic.kind);
                assert_eq!(state, fresh_state, "kind {}, pass {pass}: the state left behind", ic.kind);
                last = Some(reads);
            }
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for SeqState {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            SeqState::Ls161 { count, last_clk } => crate::snapshot::HeapBytes::heap_bytes(count, w) + crate::snapshot::HeapBytes::heap_bytes(last_clk, w),
            SeqState::Ls173 { q, last_clk } => crate::snapshot::HeapBytes::heap_bytes(q, w) + crate::snapshot::HeapBytes::heap_bytes(last_clk, w),
            SeqState::Hc74 { q1, q1n, last_clk1, q2, q2n, last_clk2 } => crate::snapshot::HeapBytes::heap_bytes(q1, w) + crate::snapshot::HeapBytes::heap_bytes(q1n, w) + crate::snapshot::HeapBytes::heap_bytes(last_clk1, w) + crate::snapshot::HeapBytes::heap_bytes(q2, w) + crate::snapshot::HeapBytes::heap_bytes(q2n, w) + crate::snapshot::HeapBytes::heap_bytes(last_clk2, w),
        }
    }
}
