//! The memory family: 74ls189, 28c16, 28c256 (core/README.md, "Kernel
//! semantics: memories (W1d)"). W1d worker D2's scope: `MemState`'s real
//! fields and [`evaluate`]'s real per-kind logic (contract points 2-6),
//! plus [`seed`] and [`take_update`] behind `bo_engine_seed_memory` and
//! `bo_engine_take_eeprom_updates`.
//!
//! The 189 and the two EEPROMs never share a device, so [`MemState`] is an
//! enum picked once at [`MemState::power_on`] from `ic.kind`; [`evaluate`],
//! [`seed`] and [`take_update`] then dispatch on that variant. The EEPROM
//! branch still reads its address width from `ic.kind` (28c16 or 28c256).
//!
//! **EEPROM image, copy-on-write.** The evaluator snapshots every device's
//! state at every instant (contract, "A failed advance"), so the 2 KiB/32 KiB
//! byte image lives behind an `Rc<Vec<u8>>`: an ordinary snapshot is a
//! refcount bump, and [`Rc::make_mut`] on the one write path (write
//! completion) copies the backing `Vec` only the instant two owners
//! (the live state and a held snapshot) actually diverge.

use std::rc::Rc;

use super::{FamilyOutput, IcRecord};
use crate::drivers::DelaySet;
use crate::events::Mode;
use crate::kinds;

/// State for one 74ls189/28c16/28c256 device (memories section point 2).
#[derive(Debug, Clone, PartialEq)]
pub enum MemState {
    Ram189(Ram189State),
    Eeprom(EepromState),
}

/// The 74ls189's 16 nibbles, true sense (the `/o` pins show the complement),
/// the write path's edge history, and the preload the record carried, if any
/// (memories point 2a): what `mem` held at power-on and holds again after a
/// power loss. The preload never changes after [`MemState::power_on`]; it
/// sits here, with the words it restores, so a snapshot or a cloned engine
/// needs nothing else to bring it back.
#[derive(Debug, Clone, PartialEq)]
pub struct Ram189State {
    mem: [u8; 16],
    last_we: bool,
    preload: Option<[u8; 16]>,
}

/// A 28c16/28c256's byte image and write-cycle state machine.
#[derive(Debug, Clone, PartialEq)]
pub struct EepromState {
    /// Copy-on-write: see the module doc comment.
    image: Rc<Vec<u8>>,
    last_we_high: bool,
    last_ce_high: bool,
    /// `(addr, byte, completesAt)`, already masked to the kind's address
    /// width.
    pending: Option<(usize, u8, f64)>,
    poll_reads: u16,
    sdp_step: u8,
    sdp_unlocked: bool,
    dirty: bool,
    /// Whether the previous powered evaluation was itself a DATA-polling
    /// read cycle (`/oe` and `/ce` both read LOW) — memories point 4.4's own
    /// edge detector, separate from `last_reads` because it must survive
    /// (and only needs to track) exactly this one two-pin condition.
    prev_read_cycle: bool,
}

/// Opaque tag for this family's one `FamilyOutput::timers` producer, an
/// EEPROM's write-completion deadline (memories point 4.2 / point 5). The
/// interface says the kernel never inspects it.
const WRITE_COMPLETE_TAG: u64 = 0;

/// SDP unlock sequence, classic's `SDP_ENABLE` (`digital-ics.ts:94-98`):
/// `(address, data)` expected at each of the three steps.
const SDP_SEQUENCE: [(usize, u8); 3] = [(0x5555, 0xAA), (0x2AAA, 0x55), (0x5555, 0xA0)];

impl MemState {
    /// Power-on values (memories section point 2): 189 `mem[0..15] = 0xF`,
    /// `lastWE = 1`; EEPROM the blob's image, `lastWeHigh = lastCeHigh =
    /// true`, no pending write, `pollReads = 0`, SDP locked at step 0,
    /// `dirty = false`. `rom` is the device's own blob tail (`devices::ics`'s
    /// decode): an EEPROM's ROM image (core/README.md's "ROM image"), and for
    /// a 74ls189 its preload, sixteen nibbles or none (memories point 2a).
    /// With a preload `mem` starts as it; with none, at 0xF as before.
    pub fn power_on(ic: &IcRecord, rom: &[u32]) -> Self {
        if ic.kind == kinds::IC_74LS189 {
            let preload: Option<[u8; 16]> = (rom.len() == 16).then(|| std::array::from_fn(|a| (rom[a] & 0xF) as u8));
            MemState::Ram189(Ram189State { mem: preload.unwrap_or([0xF; 16]), last_we: true, preload })
        } else {
            MemState::Eeprom(EepromState {
                image: Rc::new(words_to_bytes(rom)),
                last_we_high: true,
                last_ce_high: true,
                pending: None,
                poll_reads: 0,
                sdp_step: 0,
                sdp_unlocked: false,
                dirty: false,
                prev_read_cycle: false,
            })
        }
    }
}

/// Power loss (memories point 2a): a 74ls189 that has a preload holds it
/// again, so a write does not survive the outage; `lastWE` stays frozen, as
/// every unpowered IC's edge history does. A 189 with no preload and an
/// EEPROM are untouched (their state freezes, sequential point 3). Returns
/// whether a word changed.
pub fn power_lost(state: &mut MemState) -> bool {
    match state {
        MemState::Ram189(Ram189State { mem, preload: Some(preload), .. }) if mem != preload => {
            *mem = *preload;
            true
        }
        _ => false,
    }
}

/// W2c's carry: this memory's IC-record tag and values (`crate::carry`'s
/// IC layout). A 189: its sixteen nibbles and `/WE`'s last level. An EEPROM:
/// its image by hash and length (the image itself reloads from the
/// program's ROM blob, and carries only onto the same contents), then the
/// write cycle: `/WE` and `/CE` history, a pending write as `(address, byte,
/// completes at)` or three `NaN`, the DATA-polling reads, the SDP step and
/// unlock, the dirty flag and the last read cycle.
pub fn carry_values(state: &MemState, out: &mut Vec<f64>) -> f64 {
    use crate::carry::ic::{bit, image_hash, EEPROM, RAM_189};
    match state {
        MemState::Ram189(st) => {
            out.extend(st.mem.iter().map(|&n| f64::from(n)));
            out.push(bit(st.last_we));
            RAM_189
        }
        MemState::Eeprom(st) => {
            let (addr, byte, due) = st.pending.map_or((f64::NAN, f64::NAN, f64::NAN), |(a, b, t)| (a as f64, f64::from(b), t));
            out.extend([image_hash(&st.image), st.image.len() as f64, bit(st.last_we_high), bit(st.last_ce_high), addr, byte, due, f64::from(st.poll_reads), f64::from(st.sdp_step), bit(st.sdp_unlocked), bit(st.dirty), bit(st.prev_read_cycle)]);
            EEPROM
        }
    }
}

/// W2c's seed: [`carry_values`]' inverse, at load. A 189 takes its nibbles
/// and `/WE` history. An EEPROM takes its write cycle only when the carried
/// image is this one (hash and length): the image itself is the program's
/// ROM blob, so the host carries a written image by loading it there.
/// Returns whether the state was seeded (`false` for another image), and
/// `Err` for a value out of range.
pub fn carry_seed(state: &mut MemState, v: &[f64]) -> Result<bool, String> {
    use crate::carry::ic::{bit_of, image_hash};
    let whole = |x: f64, max: f64, what: &str| -> Result<f64, String> {
        if x.is_finite() && x >= 0.0 && x.fract() == 0.0 && x <= max {
            Ok(x)
        } else {
            Err(format!("carry: a memory's {what} {x} is out of range"))
        }
    };
    match state {
        MemState::Ram189(st) => {
            for (k, &n) in v[..16].iter().enumerate() {
                st.mem[k] = whole(n, 15.0, "nibble")? as u8;
            }
            st.last_we = bit_of(v[16])?;
            Ok(true)
        }
        MemState::Eeprom(st) => {
            if v[0] != image_hash(&st.image) || v[1] != st.image.len() as f64 {
                return Ok(false);
            }
            let mask = (st.image.len() - 1) as f64;
            let pending = if v[4].is_nan() && v[5].is_nan() && v[6].is_nan() {
                None
            } else {
                if !v[6].is_finite() {
                    return Err(format!("carry: an EEPROM write completing at {}", v[6]));
                }
                Some((whole(v[4], mask, "pending address")? as usize, whole(v[5], 255.0, "pending byte")? as u8, v[6]))
            };
            st.last_we_high = bit_of(v[2])?;
            st.last_ce_high = bit_of(v[3])?;
            st.pending = pending;
            st.poll_reads = whole(v[7], f64::from(u16::MAX), "poll count")? as u16;
            st.sdp_step = whole(v[8], 3.0, "SDP step")? as u8;
            st.sdp_unlocked = bit_of(v[9])?;
            st.dirty = bit_of(v[10])?;
            st.prev_read_cycle = bit_of(v[11])?;
            Ok(true)
        }
    }
}

/// The timer a seeded state needs: an EEPROM write in flight wakes its
/// device when it completes, as the evaluation that accepted it scheduled.
pub fn carried_timer(state: &MemState) -> Option<(f64, u64)> {
    match state {
        MemState::Eeprom(st) => st.pending.map(|(_, _, at)| (at, WRITE_COMPLETE_TAG)),
        MemState::Ram189(_) => None,
    }
}

/// Little-endian word decode, the README's "ROM image": word `k` is
/// `b[4k] + b[4k+1]*2^8 + b[4k+2]*2^16 + b[4k+3]*2^24`, inverted here.
fn words_to_bytes(rom: &[u32]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(rom.len() * 4);
    for &w in rom {
        bytes.push((w & 0xFF) as u8);
        bytes.push(((w >> 8) & 0xFF) as u8);
        bytes.push(((w >> 16) & 0xFF) as u8);
        bytes.push(((w >> 24) & 0xFF) as u8);
    }
    bytes
}

/// The inverse packing, for `bo_engine_take_eeprom_updates` ("the whole
/// image packed as the ROM blob is").
fn bytes_to_words(bytes: &[u8]) -> Vec<u32> {
    bytes.chunks_exact(4).map(|c| c[0] as u32 | (c[1] as u32) << 8 | (c[2] as u32) << 16 | (c[3] as u32) << 24).collect()
}

/// `n` read bits starting at `reads[base]`, bit 0 = `reads[base]` (every
/// address/data bus in this file is bit 0 first, README's "Bit 0 of every
/// bus is its first pin").
fn bits(reads: &[bool], base: usize, n: usize) -> usize {
    let mut v = 0usize;
    for i in 0..n {
        if reads[base + i] {
            v |= 1 << i;
        }
    }
    v
}

/// The running largest of the delay figures of the causes that apply to one
/// transition, folded in the order they are pushed (`f64::max`, so exactly
/// what a fold over a collected slice gives) without collecting them: `or`
/// yields it, or `fallback` when no cause was pushed (delays point 3: "with no
/// identified cause, the kind's first pair applies" / WHAT TO BUILD point 3:
/// "unselectable entries take the kind's first pair, never 0").
#[derive(Default)]
struct Largest(Option<f64>);

impl Largest {
    fn push(&mut self, v: f64) {
        self.0 = Some(self.0.map_or(v, |a| a.max(v)));
    }

    fn or(self, fallback: f64) -> f64 {
        self.0.unwrap_or(fallback)
    }
}

/// One evaluation of a powered 74ls189/28c16/28c256 (memories section points
/// 3-4) — see `sequential::evaluate`'s doc comment for the parameter
/// convention this mirrors exactly.
pub fn evaluate(ic: &IcRecord, state: &mut MemState, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64) -> FamilyOutput {
    let mut out = FamilyOutput::default();
    evaluate_into(ic, state, reads, last_reads, edges_live, now, &mut out);
    out
}

/// [`evaluate`], replacing the contents of `out` (the caller's reusable
/// buffers) instead of returning a fresh [`FamilyOutput`].
pub fn evaluate_into(ic: &IcRecord, state: &mut MemState, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    match state {
        MemState::Ram189(st) => evaluate_189_into(ic, st, reads, last_reads, edges_live, out),
        MemState::Eeprom(st) => evaluate_eeprom_into(ic, st, reads, last_reads, edges_live, now, out),
    }
}

// ---- 74ls189 ---------------------------------------------------------------
//
// Logic pins (README's sequential/memory table): `a0 a1 a2 a3 d1 d2 d3 d4
// /cs /we` (0-9, read), `/o1 /o2 /o3 /o4` (10-13, driven — not read by this
// function; `output_capable` already gives the dispatcher the right driver
// order for `FamilyOutput.requests`).

const ADDR189: usize = 0;
const DATA189: usize = 4;
const CS189: usize = 8;
const WE189: usize = 9;

#[cfg(test)]
fn evaluate_189(ic: &IcRecord, state: &mut Ram189State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool) -> FamilyOutput {
    let mut out = FamilyOutput::default();
    evaluate_189_into(ic, state, reads, last_reads, edges_live, &mut out);
    out
}

fn evaluate_189_into(ic: &IcRecord, state: &mut Ram189State, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, out: &mut FamilyOutput) {
    let cs_low = !reads[CS189];
    let we_high = reads[WE189];

    // Step 2: write, edges live only, trailing edge of /we (memories point
    // 3.2). lastWE updates whatever /cs reads, still gated by edges_live
    // (the general IC contract's "no ... lastWE update" while frozen).
    if edges_live {
        let we_rising = we_high && !state.last_we;
        if cs_low && we_rising {
            let addr = bits(reads, ADDR189, 4);
            state.mem[addr] = bits(reads, DATA189, 4) as u8;
        }
        state.last_we = we_high;
    }

    // Steps 3-4: drive and publish read the (possibly just-written) mem
    // array live, unconditionally — the 189's own "read, write (edges live
    // only), drive, publish" ordering (WHAT TO BUILD point 2).
    let addr = bits(reads, ADDR189, 4);
    let word = state.mem[addr];
    let delays = delay_set_189(ic, reads, last_reads);

    out.requests.clear();
    if cs_low && we_high {
        for bit in 0..4 {
            let level = (word >> bit) & 1 == 0; // inverted outputs
            out.requests.push((if level { Mode::High } else { Mode::Low }, delays));
        }
    } else {
        out.requests.extend(std::iter::repeat_n((Mode::Hiz, delays), 4));
    }

    out.slots.clear();
    out.slots.resize(4, f64::NAN);
    if cs_low {
        // Write mode included (memories point 3.4 / classic's own DI:489-495).
        for (bit, slot) in out.slots.iter_mut().enumerate() {
            *slot = if (word >> bit) & 1 == 0 { 1.0 } else { 0.0 };
        }
    }
    out.timers.clear();
}

/// `ic.delays` in the README's pair order: `addrO(0) csEnable(1)
/// csDisable(2) weEnable(3) weDisable(4)`.
fn delay_set_189(ic: &IcRecord, reads: &[bool], last_reads: Option<&[bool]>) -> DelaySet {
    let d = &ic.delays;
    let (lh, hl) = d[0];

    let mut zh_c = Largest::default();
    let mut zl_c = Largest::default();
    let mut hz_c = Largest::default();
    let mut lz_c = Largest::default();

    if let Some(last) = last_reads {
        let cs_low_now = !reads[CS189];
        let cs_low_last = !last[CS189];
        let we_high_now = reads[WE189];
        let we_high_last = last[WE189];
        let addr_changed = (0..4).any(|i| reads[ADDR189 + i] != last[ADDR189 + i]);

        if cs_low_now && !cs_low_last {
            zh_c.push(d[1].0);
            zl_c.push(d[1].1); // /cs fell: csEnable
        }
        if we_high_now && !we_high_last {
            zh_c.push(d[3].0);
            zl_c.push(d[3].1); // /we rose: weEnable
        }
        if addr_changed {
            zh_c.push(d[0].0);
            zl_c.push(d[0].1);
        }

        if !cs_low_now && cs_low_last {
            hz_c.push(d[2].0);
            lz_c.push(d[2].1); // /cs rose: csDisable
        }
        if !we_high_now && we_high_last {
            hz_c.push(d[4].0);
            lz_c.push(d[4].1); // /we fell: weDisable
        }
    }

    DelaySet { lh, hl, zh: zh_c.or(d[0].0), zl: zl_c.or(d[0].1), hz: hz_c.or(d[0].0), lz: lz_c.or(d[0].1) }
}

// ---- 28c16 / 28c256 ---------------------------------------------------------
//
// Logic pins: `a0..a(addrBits-1) /oe /ce /we` (read), `io0..io7` (driven AND
// read — sampled for the data bus on a write, per the 74ls245 rule the
// contract's shared-machinery point 1 extends to these).

fn consume_eeprom_write(state: &mut EepromState, sdp_enabled: bool, addr: usize, data: u8) -> bool {
    if !sdp_enabled || state.sdp_unlocked {
        return true;
    }
    let (expect_addr, expect_data) = SDP_SEQUENCE[state.sdp_step as usize];
    if addr == expect_addr && data == expect_data {
        state.sdp_step += 1;
        if state.sdp_step == 3 {
            state.sdp_unlocked = true;
            state.sdp_step = 0;
        }
        return false;
    }
    // "A mismatch sets sdpStep = 0 without re-testing the write against step
    // 0" (memories section, SDP paragraph): one comparison against the step
    // that was active on entry, then reset — never a second comparison
    // against the freshly reset step.
    state.sdp_step = 0;
    false
}

#[cfg(test)]
fn evaluate_eeprom(ic: &IcRecord, state: &mut EepromState, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64) -> FamilyOutput {
    let mut out = FamilyOutput::default();
    evaluate_eeprom_into(ic, state, reads, last_reads, edges_live, now, &mut out);
    out
}

fn evaluate_eeprom_into(ic: &IcRecord, state: &mut EepromState, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64, out: &mut FamilyOutput) {
    let addr_bits = if ic.kind == kinds::IC_28C16 { 11 } else { 15 };
    let mask = (1usize << addr_bits) - 1;
    let oe_pin = addr_bits;
    let ce_pin = addr_bits + 1;
    let we_pin = addr_bits + 2;
    let io_base = addr_bits + 3;

    let oe_low = !reads[oe_pin];
    let ce_low = !reads[ce_pin];
    let we_high = reads[we_pin];
    let addr = bits(reads, 0, addr_bits) & mask;

    let t_wc = ic.extra.first().copied().unwrap_or(10e-3);
    let sdp_enabled = ic.extra.get(1).copied().unwrap_or(0.0) != 0.0;

    // Step 1: completion, not gated by edges_live — a time deadline, not an
    // edge (memories point 4.1; point 5: "the completion itself is step
    // 4.1, so an evaluation then for any other reason completes the
    // write").
    if let Some((pending_addr, pending_byte, completes_at)) = state.pending {
        if now >= completes_at {
            Rc::make_mut(&mut state.image)[pending_addr] = pending_byte;
            state.dirty = true;
            state.pending = None;
            state.poll_reads = 0;
        }
    }

    // Step 2-3: latch and SDP, edges live only (memories point 4.2-4.3).
    let mut newly_accepted_at: Option<f64> = None;
    if edges_live {
        let ce_high = !ce_low;
        let oe_high = !oe_low;
        let we_rising = we_high && !state.last_we_high;
        let ce_rising = ce_high && !state.last_ce_high;
        let we_latch = we_rising && ce_low && oe_high;
        let ce_latch = ce_rising && !we_high && oe_high;

        if (we_latch || ce_latch) && state.pending.is_none() {
            let data = bits(reads, io_base, 8) as u8;
            if consume_eeprom_write(state, sdp_enabled, addr, data) {
                let completes_at = now + t_wc;
                state.pending = Some((addr, data, completes_at));
                state.poll_reads = 0;
                newly_accepted_at = Some(completes_at);
            }
        }
        // Updates whatever the latch decision was — including when an edge
        // was ignored because a write was already pending (memories point
        // 4.2: "An edge while a write is pending is ignored and never
        // reaches SDP. Then lastWeHigh = weHigh, lastCeHigh = ceHigh.").
        state.last_we_high = we_high;
        state.last_ce_high = ce_high;
    }

    // Step 4: the toggle bit. Ungated by edges_live, like drive/publish
    // below — the contract's list ("completion first, latch (edges live
    // only), SDP, the toggle bit, drive, publish") marks only "latch" with
    // the qualifier, mirroring the 189's "write (edges live only)" vs.
    // always-live drive/publish.
    let read_cycle_now = oe_low && ce_low;
    if state.pending.is_some() && read_cycle_now && !state.prev_read_cycle {
        state.poll_reads = state.poll_reads.wrapping_add(1);
    }
    state.prev_read_cycle = read_cycle_now;

    // Steps 5-6: drive and publish.
    let delays = delay_set_eeprom(ic, addr_bits, reads, last_reads);
    let byte = state.image[addr];
    out.requests.clear();
    out.slots.clear();
    out.slots.resize(8, f64::NAN);

    if oe_low && ce_low {
        let polling_byte = state.pending.and_then(|(pending_addr, pending_byte, _)| (pending_addr == addr).then_some(pending_byte));
        for bit in 0..8 {
            let level = match (polling_byte, bit) {
                (Some(pending_byte), 7) => (pending_byte >> 7) & 1 == 0, // complement
                (Some(_), 6) => state.poll_reads & 1 == 1,
                _ => (byte >> bit) & 1 == 1,
            };
            out.requests.push((if level { Mode::High } else { Mode::Low }, delays));
        }
        if state.pending.is_none() {
            for (bit, slot) in out.slots.iter_mut().enumerate() {
                *slot = ((byte >> bit) & 1) as f64;
            }
        }
    } else {
        out.requests.extend(std::iter::repeat_n((Mode::Hiz, delays), 8));
    }

    out.timers.clear();
    if let Some(at) = newly_accepted_at {
        out.timers.push((at, WRITE_COMPLETE_TAG));
    }
}

/// `ic.delays` in the README's pair order: `addrIo(0) ceEnable(1)
/// ceDisable(2) oeEnable(3) oeDisable(4)`.
fn delay_set_eeprom(ic: &IcRecord, addr_bits: usize, reads: &[bool], last_reads: Option<&[bool]>) -> DelaySet {
    let d = &ic.delays;
    let (lh, hl) = d[0];
    let oe_pin = addr_bits;
    let ce_pin = addr_bits + 1;

    let mut zh_c = Largest::default();
    let mut zl_c = Largest::default();
    let mut hz_c = Largest::default();
    let mut lz_c = Largest::default();

    if let Some(last) = last_reads {
        let ce_low_now = !reads[ce_pin];
        let ce_low_last = !last[ce_pin];
        let oe_low_now = !reads[oe_pin];
        let oe_low_last = !last[oe_pin];
        let addr_changed = (0..addr_bits).any(|i| reads[i] != last[i]);

        if ce_low_now && !ce_low_last {
            zh_c.push(d[1].0);
            zl_c.push(d[1].1); // /ce fell: ceEnable
        }
        if oe_low_now && !oe_low_last {
            zh_c.push(d[3].0);
            zl_c.push(d[3].1); // /oe fell: oeEnable
        }
        if addr_changed {
            zh_c.push(d[0].0);
            zl_c.push(d[0].1);
        }

        if !ce_low_now && ce_low_last {
            hz_c.push(d[2].0);
            lz_c.push(d[2].1); // /ce rose: ceDisable
        }
        if !oe_low_now && oe_low_last {
            hz_c.push(d[4].0);
            lz_c.push(d[4].1); // /oe rose: oeDisable
        }
    }

    DelaySet { lh, hl, zh: zh_c.or(d[0].0), zl: zl_c.or(d[0].1), hz: hz_c.or(d[0].0), lz: lz_c.or(d[0].1) }
}

// ---- exports behind the C ABI ----------------------------------------------

/// Replaces a 74ls189's 16 stored nibbles (`bo_engine_seed_memory`, owner
/// decision 2). Validates every word — an integer in `[0, 15]` — before
/// touching `state`, so a rejected seed changes nothing (README:
/// "`len_f64 != 16` ... is `-8`, validated in full before anything
/// changes"; the same rule extends to a bad value). `devices::ics::
/// IcEvaluator::seed_memory` already routes a non-189 kind to `Err("not a
/// memory")` itself and never reaches here for one; the `Eeprom` arm below
/// is unreachable in practice but handled rather than assumed.
pub fn seed(ic: &IcRecord, state: &mut MemState, words: &[f64]) -> Result<(), String> {
    let _ = ic;
    if words.len() != 16 {
        return Err(format!("seed_memory: expected 16 words, got {}", words.len()));
    }
    let mut nibbles = [0u8; 16];
    for (i, &w) in words.iter().enumerate() {
        if !(w.is_finite() && w.fract() == 0.0 && (0.0..=15.0).contains(&w)) {
            return Err(format!("seed_memory: word {i} must be an integer in [0, 15], got {w}"));
        }
        nibbles[i] = w as u8;
    }
    match state {
        MemState::Ram189(st) => {
            st.mem = nibbles;
            Ok(())
        }
        MemState::Eeprom(_) => Err("seed_memory: not a 74ls189".to_string()),
    }
}

/// Collects and clears this device's dirty EEPROM image
/// (`bo_engine_take_eeprom_updates`). `None` for a 74ls189 (never dirty) or
/// a clean EEPROM.
pub fn take_update(state: &mut MemState) -> Option<Vec<u32>> {
    match state {
        MemState::Eeprom(st) if st.dirty => {
            st.dirty = false;
            Some(bytes_to_words(&st.image))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::digital::IcPin;

    // ---- record/state builders ---------------------------------------------

    fn ic_pin() -> IcPin {
        IcPin { node: 0, open: false, open_high: false, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false }
    }

    fn ic_record(kind: u32, logic_len: usize, delays: Vec<(f64, f64)>, extra: Vec<f64>) -> IcRecord {
        IcRecord {
            kind,
            vcc: ic_pin(),
            gnd: ic_pin(),
            logic: vec![ic_pin(); logic_len],
            vcc_min: 4.5,
            vcc_max: 5.5,
            ttl: true,
            cmos_ratio: false,
            vil: Some(0.8),
            vih: Some(2.0),
            schmitt: None,
            nominal: 5.0,
            io_max: None,
            delay: 20e-9,
            slot_base: 0,
            delays,
            extra,
        }
    }

    // addrO, csEnable, csDisable, weEnable, weDisable / addrIo, ceEnable,
    // ceDisable, oeEnable, oeDisable — all distinct so a test can tell which
    // pair a request actually used.
    const D: [(f64, f64); 5] = [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (7.0, 8.0), (9.0, 10.0)];

    fn ic_189() -> IcRecord {
        ic_record(kinds::IC_74LS189, 14, D.to_vec(), Vec::new())
    }

    fn ic_28c16() -> IcRecord {
        ic_record(kinds::IC_28C16, 22, D.to_vec(), vec![10e-3, 0.0])
    }

    fn ic_28c256(sdp: f64) -> IcRecord {
        ic_record(kinds::IC_28C256, 26, D.to_vec(), vec![10e-3, sdp])
    }

    fn eeprom_state(bytes: Vec<u8>) -> EepromState {
        EepromState {
            image: Rc::new(bytes),
            last_we_high: true,
            last_ce_high: true,
            pending: None,
            poll_reads: 0,
            sdp_step: 0,
            sdp_unlocked: false,
            dirty: false,
            prev_read_cycle: false,
        }
    }

    fn reads189(addr: u8, data: u8, cs_low: bool, we_high: bool) -> Vec<bool> {
        let mut r = vec![false; 14];
        for i in 0..4 {
            r[ADDR189 + i] = (addr >> i) & 1 == 1;
            r[DATA189 + i] = (data >> i) & 1 == 1;
        }
        r[CS189] = !cs_low;
        r[WE189] = we_high;
        r
    }

    fn reads_eeprom(addr_bits: usize, addr: usize, data: u8, oe_low: bool, ce_low: bool, we_high: bool) -> Vec<bool> {
        let mut r = vec![false; addr_bits + 3 + 8];
        for i in 0..addr_bits {
            r[i] = (addr >> i) & 1 == 1;
        }
        r[addr_bits] = !oe_low;
        r[addr_bits + 1] = !ce_low;
        r[addr_bits + 2] = we_high;
        for i in 0..8 {
            r[addr_bits + 3 + i] = (data >> i) & 1 == 1;
        }
        r
    }

    /// A full we-controlled latch cycle (idle, then /we's trailing edge)
    /// against a *fresh* `last_reads` history each call, with /oe held HIGH
    /// and /ce held LOW throughout — the simplest way to drive
    /// `consume_eeprom_write`/SDP repeatedly in a test.
    fn latch_write(ic: &IcRecord, state: &mut EepromState, addr: usize, data: u8) {
        let addr_bits = if ic.kind == kinds::IC_28C16 { 11 } else { 15 };
        let before = reads_eeprom(addr_bits, addr, data, false, true, false); // /oe HIGH, /ce LOW, /we LOW
        let _ = evaluate_eeprom(ic, state, &before, None, true, 0.0);
        let after = reads_eeprom(addr_bits, addr, data, false, true, true); // /we rises
        let _ = evaluate_eeprom(ic, state, &after, Some(&before), true, 0.0);
    }

    // ---- 74ls189 -------------------------------------------------------------

    #[test]
    fn write_occurs_on_the_trailing_edge_of_we_while_selected() {
        let ic = ic_189();
        let mut state = Ram189State { mem: [0xF; 16], last_we: false, preload: None };
        let before = reads189(0b0101, 0b1010, true, false); // selected, write mode
        let _ = evaluate_189(&ic, &mut state, &before, None, true);
        assert_eq!(state.mem[0b0101], 0xF, "no write yet: /we has not risen");

        let after = reads189(0b0101, 0b1010, true, true); // trailing edge
        let _ = evaluate_189(&ic, &mut state, &after, Some(&before), true);
        assert_eq!(state.mem[0b0101], 0b1010, "trailing edge of /we writes D into mem[addr]");
    }

    #[test]
    fn write_ignored_while_cs_high_but_last_we_still_updates() {
        let ic = ic_189();
        let mut state = Ram189State { mem: [0xF; 16], last_we: false, preload: None };
        let before = reads189(2, 5, false, false); // deselected, write mode
        let after = reads189(2, 5, false, true); // /we rises, still deselected
        let _ = evaluate_189(&ic, &mut state, &before, None, true);
        let _ = evaluate_189(&ic, &mut state, &after, Some(&before), true);
        assert_eq!(state.mem[2], 0xF, "deselected: no write");
        assert!(state.last_we, "lastWE updates whatever /cs reads");
    }

    #[test]
    fn inverted_outputs_drive_high_for_a_zero_bit_and_low_for_a_one_bit() {
        let ic = ic_189();
        let mut state = Ram189State { mem: [0b0101; 16], last_we: true, preload: None };
        let reads = reads189(0, 0, true, true);
        let out = evaluate_189(&ic, &mut state, &reads, None, true);
        assert_eq!(out.requests[0].0, Mode::Low); // bit0=1
        assert_eq!(out.requests[1].0, Mode::High); // bit1=0
        assert_eq!(out.requests[2].0, Mode::Low); // bit2=1
        assert_eq!(out.requests[3].0, Mode::High); // bit3=0
    }

    #[test]
    fn outputs_go_hiz_outside_selected_read_mode() {
        let ic = ic_189();
        let mut state = Ram189State { mem: [0; 16], last_we: true, preload: None };
        let deselected = reads189(0, 0, false, true);
        let out = evaluate_189(&ic, &mut state, &deselected, None, true);
        assert!(out.requests.iter().all(|&(m, _)| m == Mode::Hiz), "deselected");

        let write_mode = reads189(0, 0, true, false);
        let out2 = evaluate_189(&ic, &mut state, &write_mode, None, true);
        assert!(out2.requests.iter().all(|&(m, _)| m == Mode::Hiz), "write mode");
    }

    #[test]
    fn slots_publish_in_write_mode_too_but_not_when_deselected() {
        let ic = ic_189();
        let mut state = Ram189State { mem: [0b0011; 16], last_we: true, preload: None };
        let write_mode = reads189(0, 0, true, false);
        let out = evaluate_189(&ic, &mut state, &write_mode, None, true);
        assert!(out.slots.iter().all(|v| !v.is_nan()), "slots publish while selected, write mode included");

        let deselected = reads189(0, 0, false, true);
        let out2 = evaluate_189(&ic, &mut state, &deselected, None, true);
        assert!(out2.slots.iter().all(|v| v.is_nan()), "deselected: NaN");
    }

    #[test]
    fn address_change_alone_picks_addro_for_a_level_to_level_transition() {
        let ic = ic_189();
        let mut state = Ram189State { mem: [0; 16], last_we: true, preload: None };
        let before = reads189(0, 0, true, true);
        let _ = evaluate_189(&ic, &mut state, &before, None, true);
        let after = reads189(1, 0, true, true);
        let out = evaluate_189(&ic, &mut state, &after, Some(&before), true);
        assert_eq!((out.requests[0].1.lh, out.requests[0].1.hl), D[0]);
    }

    #[test]
    fn cs_falling_alone_selects_csenable_for_the_enable_pair() {
        let ic = ic_189();
        let mut state = Ram189State { mem: [0; 16], last_we: true, preload: None };
        let before = reads189(0, 0, false, true); // deselected
        let _ = evaluate_189(&ic, &mut state, &before, None, true);
        let after = reads189(0, 0, true, true); // /cs falls, address unchanged
        let out = evaluate_189(&ic, &mut state, &after, Some(&before), true);
        assert_eq!((out.requests[0].1.zh, out.requests[0].1.zl), D[1], "csEnable, not addrO's fallback");
    }

    // ---- 28c16 / 28c256 -------------------------------------------------------

    #[test]
    fn we_controlled_latch_accepts_a_write_on_the_trailing_edge_of_we() {
        let ic = ic_28c16();
        let mut state = eeprom_state(vec![0xFF; 2048]);
        let before = reads_eeprom(11, 5, 0xAB, false, true, false); // /oe HIGH, /ce LOW, /we LOW
        let _ = evaluate_eeprom(&ic, &mut state, &before, None, true, 0.0);
        let after = reads_eeprom(11, 5, 0xAB, false, true, true); // /we rises
        let out = evaluate_eeprom(&ic, &mut state, &after, Some(&before), true, 0.0);
        assert_eq!(state.pending, Some((5, 0xAB, 10e-3)));
        assert_eq!(out.timers, vec![(10e-3, WRITE_COMPLETE_TAG)]);
    }

    #[test]
    fn ce_controlled_latch_accepts_a_write_on_the_trailing_edge_of_ce_when_we_is_already_low() {
        let ic = ic_28c16();
        let mut state = eeprom_state(vec![0xFF; 2048]);
        let before = reads_eeprom(11, 9, 0x12, false, true, false); // /ce LOW, /we LOW, /oe HIGH
        let _ = evaluate_eeprom(&ic, &mut state, &before, None, true, 0.0);
        let after = reads_eeprom(11, 9, 0x12, false, false, false); // /ce rises, /we stays LOW
        let out = evaluate_eeprom(&ic, &mut state, &after, Some(&before), true, 0.0);
        assert_eq!(state.pending, Some((9, 0x12, 10e-3)));
        assert_eq!(out.timers, vec![(10e-3, WRITE_COMPLETE_TAG)]);
    }

    #[test]
    fn an_edge_while_a_write_is_pending_is_ignored_and_never_reaches_sdp() {
        let ic = ic_28c256(1.0);
        let mut state = eeprom_state(vec![0xFF; 32768]);
        latch_write(&ic, &mut state, 100, 0x11); // step 0 mismatch: consumed, sdpStep resets, nothing pending
        assert_eq!(state.pending, None);
        state.sdp_unlocked = true; // isolate "pending write" behaviour from SDP gating
        latch_write(&ic, &mut state, 100, 0x11);
        let original_pending = state.pending;
        assert_eq!(original_pending, Some((100, 0x11, 10e-3)));

        state.sdp_unlocked = false;
        state.sdp_step = 0;
        let addr_bits = 15;
        let before2 = reads_eeprom(addr_bits, 0x5555, 0xAA, false, true, false);
        let _ = evaluate_eeprom(&ic, &mut state, &before2, None, true, 0.0);
        let after2 = reads_eeprom(addr_bits, 0x5555, 0xAA, false, true, true);
        let out = evaluate_eeprom(&ic, &mut state, &after2, Some(&before2), true, 0.0);

        assert_eq!(state.pending, original_pending, "the original pending write must be untouched");
        assert_eq!(state.sdp_step, 0, "the edge never reached SDP, so the step must not have advanced");
        assert!(out.timers.is_empty(), "no new write accepted, so no new Timer");
    }

    #[test]
    fn a_pending_write_completes_at_or_after_its_deadline_from_any_evaluation() {
        let ic = ic_28c16();
        let mut state = eeprom_state(vec![0xFF; 2048]);
        state.pending = Some((7, 0x42, 10e-3));
        state.poll_reads = 3;

        let unrelated = reads_eeprom(11, 0, 0, true, true, true); // deselected, unrelated address
        let _ = evaluate_eeprom(&ic, &mut state, &unrelated, None, true, 5e-3);
        assert_eq!(state.pending, Some((7, 0x42, 10e-3)), "too early: still pending");
        assert_eq!(state.image[7], 0xFF);

        let _ = evaluate_eeprom(&ic, &mut state, &unrelated, Some(&unrelated), true, 10e-3);
        assert_eq!(state.pending, None, "an unrelated evaluation at/after the deadline still completes it");
        assert_eq!(state.image[7], 0x42);
        assert!(state.dirty);
        assert_eq!(state.poll_reads, 0);
    }

    #[test]
    fn dirty_is_set_on_completion_even_when_the_byte_is_unchanged() {
        let ic = ic_28c16();
        let mut state = eeprom_state(vec![0x42; 2048]);
        state.pending = Some((3, 0x42, 1e-3));
        let reads = reads_eeprom(11, 0, 0, true, true, true);
        let _ = evaluate_eeprom(&ic, &mut state, &reads, None, true, 1e-3);
        assert!(state.dirty, "completion sets dirty even when the stored byte does not change");
    }

    #[test]
    fn data_polling_shows_io7_complement_and_io6_toggling_once_per_read_cycle_not_per_evaluation() {
        let ic = ic_28c16();
        let mut state = eeprom_state(vec![0xFF; 2048]);
        state.pending = Some((4, 0b1000_0000, 10e-3)); // pendingByte bit7 = 1

        let read1 = reads_eeprom(11, 4, 0, true, true, false); // /oe, /ce both LOW at the pending addr
        let out1 = evaluate_eeprom(&ic, &mut state, &read1, None, true, 0.0);
        assert_eq!(out1.requests[7].0, Mode::Low, "io7 = complement of pendingByte bit7 (1 -> LOW)");
        assert_eq!(state.poll_reads, 1, "one read cycle counted");
        // Step 4 (the toggle bit) runs before step 5 (drive) within the same
        // evaluation, so the read cycle that FIRST sets pollReads to 1 also
        // sees that same odd value on its own io6 request.
        assert_eq!(out1.requests[6].0, Mode::High, "pollReads became 1 (odd) this evaluation -> io6 HIGH");

        let out2 = evaluate_eeprom(&ic, &mut state, &read1, Some(&read1), true, 0.0);
        assert_eq!(state.poll_reads, 1, "still inside the same read cycle: no second increment");
        assert_eq!(out2.requests[6].0, Mode::High);

        let deselect = reads_eeprom(11, 4, 0, false, false, false); // /oe, /ce HIGH: leave the read cycle
        let _ = evaluate_eeprom(&ic, &mut state, &deselect, Some(&read1), true, 0.0);
        let out3 = evaluate_eeprom(&ic, &mut state, &read1, Some(&deselect), true, 0.0);
        assert_eq!(state.poll_reads, 2, "a fresh read cycle toggles the bit");
        assert_eq!(out3.requests[6].0, Mode::Low, "pollReads is now 2 (even) -> io6 LOW");
    }

    #[test]
    fn sdp_unlock_sequence_accepts_a_write_after_three_matching_writes() {
        let ic = ic_28c256(1.0);
        let mut state = eeprom_state(vec![0xFF; 32768]);
        latch_write(&ic, &mut state, 0x5555, 0xAA);
        assert_eq!(state.pending, None, "unlock-sequence writes are consumed, not stored");
        assert_eq!(state.sdp_step, 1);
        latch_write(&ic, &mut state, 0x2AAA, 0x55);
        assert_eq!(state.sdp_step, 2);
        latch_write(&ic, &mut state, 0x5555, 0xA0);
        assert!(state.sdp_unlocked);
        assert_eq!(state.sdp_step, 0);

        latch_write(&ic, &mut state, 0x10, 0x99);
        assert_eq!(state.pending, Some((0x10, 0x99, 10e-3)), "unlocked: an ordinary write is now accepted");
    }

    #[test]
    fn a_mismatch_resets_the_step_without_retesting_against_the_reset_step() {
        let ic = ic_28c256(1.0);
        let mut state = eeprom_state(vec![0xFF; 32768]);
        latch_write(&ic, &mut state, 0x5555, 0xAA); // step 0 -> 1 (now expects 0x2AAA/0x55)
        assert_eq!(state.sdp_step, 1);
        // This write matches step 0's pair exactly, but the active step is 1.
        latch_write(&ic, &mut state, 0x5555, 0xAA);
        assert_eq!(state.sdp_step, 0, "mismatched against step 1's expected pair, so it resets");
        assert!(!state.sdp_unlocked, "one call does not retry the same write against the freshly reset step 0");
    }

    #[test]
    fn sdp_is_off_on_a_28c16_so_every_write_is_accepted_directly() {
        let ic = ic_28c16();
        let mut state = eeprom_state(vec![0xFF; 2048]);
        latch_write(&ic, &mut state, 5, 0xAA); // would be an SDP unlock byte on a 28c256
        assert_eq!(state.pending, Some((5, 0xAA, 10e-3)), "no SDP gating on a 28c16");
    }

    #[test]
    fn a_28c256_address_uses_a_15_bit_mask_and_reaches_the_top_of_the_image() {
        let ic = ic_28c256(0.0);
        let mut state = eeprom_state(vec![0xFF; 32768]);
        latch_write(&ic, &mut state, 32767, 0x7A); // the highest 15-bit address
        assert_eq!(state.pending, Some((32767, 0x7A, 10e-3)));
        let reads = reads_eeprom(15, 0, 0, true, true, true);
        let _ = evaluate_eeprom(&ic, &mut state, &reads, None, true, 10e-3);
        assert_eq!(state.image[32767], 0x7A);
        assert_eq!(state.image.len(), 32768);
    }

    #[test]
    fn edges_live_false_changes_no_latch_state_even_on_a_would_be_latching_edge() {
        let ic = ic_28c16();
        let mut state = eeprom_state(vec![0xFF; 2048]);
        let before = reads_eeprom(11, 5, 0xAB, false, true, false); // /oe HIGH, /ce LOW: a valid we-latch setup
        let _ = evaluate_eeprom(&ic, &mut state, &before, None, true, 0.0);
        let snapshot = state.clone();
        let after = reads_eeprom(11, 5, 0xAB, false, true, true); // /we would rise -> latch, if live
        let _ = evaluate_eeprom(&ic, &mut state, &after, Some(&before), false, 0.0);
        assert_eq!(state, snapshot, "edges_live = false: no latch, no history update, no state change");
    }

    // ---- the 74ls189's preload (memories point 2a) --------------------------

    fn preload_words() -> Vec<u32> {
        (0..16).map(|a| (7 * a + 3) & 0xF).collect()
    }

    #[test]
    fn a_189_powers_on_holding_its_preload_and_remembers_it() {
        let words = preload_words();
        let expected: [u8; 16] = std::array::from_fn(|a| words[a] as u8);
        // The live words start as the preload, and the preload is kept
        // beside them for the next power loss.
        assert_eq!(MemState::power_on(&ic_189(), &words), MemState::Ram189(Ram189State { mem: expected, last_we: true, preload: Some(expected) }));
    }

    #[test]
    fn a_189_with_no_preload_powers_on_blank_and_holds_none() {
        // Point 2, unchanged: 0xF at every address, lastWE = 1.
        assert_eq!(MemState::power_on(&ic_189(), &[]), MemState::Ram189(Ram189State { mem: [0xF; 16], last_we: true, preload: None }));
    }

    #[test]
    fn a_power_loss_puts_the_preload_back_and_leaves_the_edge_history() {
        let mut state = MemState::power_on(&ic_189(), &preload_words());
        let fresh = state.clone();
        // /we LOW, then HIGH with D = 9 at address 5: writes 9 over the preloaded 6.
        let low = reads189(5, 9, true, false);
        let high = reads189(5, 9, true, true);
        let _ = evaluate(&ic_189(), &mut state, &low, None, true, 0.0);
        let _ = evaluate(&ic_189(), &mut state, &high, Some(&low), true, 0.0);
        assert_ne!(state, fresh, "the write landed");
        assert!(power_lost(&mut state), "a word changed back");
        // lastWE is 1 after the write and 1 at power-on, so the whole state
        // equals the fresh one: the 9 is gone, nothing else moved.
        assert_eq!(state, fresh);
        assert!(!power_lost(&mut state), "already the preload: nothing to change");
    }

    #[test]
    fn a_power_loss_leaves_lastwe_frozen_where_the_outage_found_it() {
        // /we LOW when the power goes: lastWE stays 0 (Q-O4), so /we reading
        // HIGH at power-up is a rising edge and writes, as it does today.
        let mut state = MemState::power_on(&ic_189(), &preload_words());
        let low = reads189(5, 9, true, false);
        let _ = evaluate(&ic_189(), &mut state, &low, None, true, 0.0);
        power_lost(&mut state);
        match &state {
            MemState::Ram189(st) => assert!(!st.last_we, "the outage did not touch lastWE"),
            MemState::Eeprom(_) => unreachable!(),
        }
    }

    #[test]
    fn a_power_loss_changes_nothing_of_a_189_with_no_preload_or_of_an_eeprom() {
        let mut ram = MemState::power_on(&ic_189(), &[]);
        let low = reads189(5, 9, true, false);
        let high = reads189(5, 9, true, true);
        let _ = evaluate(&ic_189(), &mut ram, &low, None, true, 0.0);
        let _ = evaluate(&ic_189(), &mut ram, &high, Some(&low), true, 0.0);
        let written = ram.clone();
        assert!(!power_lost(&mut ram));
        assert_eq!(ram, written, "no preload: the state freezes, the written 9 included");

        let mut rom = MemState::power_on(&ic_28c16(), &vec![0u32; 512]);
        let before = rom.clone();
        assert!(!power_lost(&mut rom));
        assert_eq!(rom, before, "an EEPROM is non-volatile");
    }

    #[test]
    fn cloning_a_28c256_state_does_not_copy_the_image() {
        let ic = ic_28c256(1.0);
        let words = vec![0u32; 32768 / 4];
        let state = MemState::power_on(&ic, &words);
        let clone = state.clone();
        match (&state, &clone) {
            (MemState::Eeprom(a), MemState::Eeprom(b)) => {
                assert!(Rc::ptr_eq(&a.image, &b.image), "clone must share the image, not copy it");
                assert_eq!(Rc::strong_count(&a.image), 2);
            }
            _ => panic!("expected Eeprom state"),
        }
    }

    #[test]
    fn a_write_completing_after_a_clone_copies_the_image_once_and_leaves_the_clone_alone() {
        // W2a: a snapshot shares the image; the live engine's next write
        // pays the one copy, and the snapshot keeps the old byte.
        let ic = ic_28c16();
        let mut state = eeprom_state(vec![0xFF; 2048]);
        state.pending = Some((7, 0x42, 10e-3));
        let snapshot = state.clone();
        assert!(Rc::ptr_eq(&state.image, &snapshot.image));
        let unrelated = reads_eeprom(11, 0, 0, true, true, true);
        let _ = evaluate_eeprom(&ic, &mut state, &unrelated, None, true, 10e-3);
        assert!(!Rc::ptr_eq(&state.image, &snapshot.image), "the write copied the image");
        assert_eq!((state.image[7], snapshot.image[7]), (0x42, 0xFF));
        assert_eq!((Rc::strong_count(&state.image), Rc::strong_count(&snapshot.image)), (1, 1));
    }

    // ---- seed / take_update ----------------------------------------------------

    #[test]
    fn seed_validates_every_word_before_changing_anything() {
        let ic = ic_189();
        let mut state = MemState::Ram189(Ram189State { mem: [0xF; 16], last_we: true, preload: None });
        let mut bad = vec![0.0; 16];
        bad[15] = 16.0; // out of range
        let err = seed(&ic, &mut state, &bad).unwrap_err();
        assert!(err.contains("15"), "names the offending word's index: {err}");
        if let MemState::Ram189(st) = &state {
            assert_eq!(st.mem, [0xF; 16], "a rejected seed changes nothing");
        }

        let good: Vec<f64> = (0..16).map(|i| (i % 16) as f64).collect();
        seed(&ic, &mut state, &good).unwrap();
        if let MemState::Ram189(st) = &state {
            for i in 0..16 {
                assert_eq!(st.mem[i], (i % 16) as u8);
            }
        }

        assert!(seed(&ic, &mut state, &vec![0.0; 10]).is_err(), "wrong length");
    }

    #[test]
    fn take_update_returns_the_image_when_dirty_and_clears_the_flag() {
        let bytes: Vec<u8> = [0x11, 0x22, 0x33, 0x44].into_iter().chain(std::iter::repeat(0xFFu8)).take(2048).collect();
        let mut st = eeprom_state(bytes);
        st.dirty = true;
        let mut state = MemState::Eeprom(st);

        let words = take_update(&mut state).expect("dirty: must return the image");
        assert_eq!(words[0], 0x4433_2211, "little-endian word packing");
        if let MemState::Eeprom(st) = &state {
            assert!(!st.dirty, "take_update clears dirty");
        }
        assert!(take_update(&mut state).is_none(), "not dirty: nothing to report");

        let mut ram = MemState::Ram189(Ram189State { mem: [0xF; 16], last_we: true, preload: None });
        assert!(take_update(&mut ram).is_none(), "a 74ls189 never reports a dirty EEPROM image");
    }
    #[test]
    fn evaluate_into_over_one_reused_output_matches_evaluate_across_kinds_states_and_reads() {
        let mut seed = 0x7654_3211u32;
        let mut bit = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed & 0x80 != 0
        };
        let junk = FamilyOutput { requests: vec![(Mode::Low, DelaySet::uniform(2.0)); 11], slots: vec![3.0; 11], timers: vec![(1.0, 9), (2.0, 9)] };
        let mut reused = junk.clone();
        let mut timers_seen = 0;
        for (ic, len, image) in [(ic_189(), 14usize, Vec::new()), (ic_28c16(), 22, vec![0xA5; 2048])] {
            let mut state = MemState::power_on(&ic, &image.iter().map(|&b| u32::from_le_bytes([b; 4])).collect::<Vec<_>>());
            let mut last: Option<Vec<bool>> = None;
            for pass in 0..400 {
                let reads: Vec<bool> = (0..len).map(|_| bit()).collect();
                let edges_live = pass % 5 != 0;
                let now = pass as f64 * 3e-3;
                let mut fresh_state = state.clone();
                let expected = evaluate(&ic, &mut fresh_state, &reads, last.as_deref(), edges_live, now);
                if pass % 2 == 0 {
                    reused = junk.clone();
                }
                evaluate_into(&ic, &mut state, &reads, last.as_deref(), edges_live, now, &mut reused);
                // Slots compare by bits: a NaN slot ("absent") is never `==`.
                let slot_bits = |o: &FamilyOutput| o.slots.iter().map(|v| v.to_bits()).collect::<Vec<_>>();
                assert_eq!((&reused.requests, slot_bits(&reused), &reused.timers), (&expected.requests, slot_bits(&expected), &expected.timers), "kind {}, pass {pass}", ic.kind);
                assert_eq!(state, fresh_state, "kind {}, pass {pass}: the state left behind", ic.kind);
                timers_seen += reused.timers.len();
                last = Some(reads);
            }
        }
        assert!(timers_seen > 0, "an EEPROM write was accepted, so a scheduled timer was checked too");
    }
    #[test]
    fn largest_folds_exactly_like_the_collect_then_max_or_it_replaced() {
        // The delay-cause maxima used to be collected into a `Vec` and folded
        // by `max_or`; `Largest` folds as the causes arrive. Same values in the
        // same order must give the same bits, NaN and signed zero included.
        fn max_or(vals: &[f64], fallback: f64) -> f64 {
            match vals.iter().copied().fold(None::<f64>, |acc, v| Some(acc.map_or(v, |a| a.max(v)))) {
                Some(m) => m,
                None => fallback,
            }
        }
        let pool = [0.0, -0.0, 1.0, 2.5, -3.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 7e-9, 20e-9];
        let mut seed = 0x0bad_cafeu32;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as usize
        };
        for _ in 0..500 {
            let count = next() % 6;
            let vals: Vec<f64> = (0..count).map(|_| pool[next() % pool.len()]).collect();
            let fallback = pool[next() % pool.len()];
            let mut largest = Largest::default();
            for &v in &vals {
                largest.push(v);
            }
            assert_eq!(largest.or(fallback).to_bits(), max_or(&vals, fallback).to_bits(), "{vals:?} with fallback {fallback}");
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for MemState {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            MemState::Ram189(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            MemState::Eeprom(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for Ram189State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mem, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_we, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.preload, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for EepromState {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.image, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_we_high, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_ce_high, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.poll_reads, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sdp_step, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sdp_unlocked, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dirty, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.prev_read_cycle, w); }
        n
    }
}
