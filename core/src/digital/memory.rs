//! The memory family: 74ls189, 28c16, 28c256 (core/README.md, "Kernel
//! semantics: memories (W1d)"). This commit (W1d worker K1, kernel
//! infrastructure) only stubs the interface `devices::ics`'s dispatcher
//! wires end to end — `MemState`'s real fields (the 189's 16 nibbles and
//! `lastWE`; the EEPROMs' byte image, latch/edge history, pending write,
//! poll-read count and SDP progress) and [`evaluate`]'s real per-kind logic
//! (contract points 2-6) are D2's scope, not this file's.
//!
//! [`evaluate`] drives nothing and reports every digital slot `NaN`, the
//! same placeholder shape as `sequential::evaluate` — see that function's
//! doc comment. [`seed`] always declines (defensively re-checking the word
//! count `devices::ics::IcEvaluator::seed_memory` already validated the
//! shape of before ever routing a 74ls189 here) and [`take_update`] never
//! has anything dirty to report, so `bo_engine_seed_memory` and
//! `bo_engine_take_eeprom_updates` round-trip end to end (`-8` / `0`
//! respectively) before D2's real store lands.

use super::{FamilyOutput, IcRecord};
use crate::drivers::DelaySet;
use crate::events::Mode;
use crate::kinds;

/// State for one 74ls189/28c16/28c256 device (contract point 2 and the
/// memories section's own point 2). Empty for now. `Clone` for the same
/// snapshot reason as `sequential::SeqState`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MemState;

impl MemState {
    /// Power-on values (memories section point 2): 189 `mem[0..15] = 0xF`,
    /// `lastWE = 1`; EEPROM the blob's image, `lastWeHigh = lastCeHigh =
    /// true`, no pending write, `pollReads = 0`, SDP locked at step 0,
    /// `dirty = false`. `rom` is the device's own ROM image (`devices::ics`'s
    /// blob decode, core/README.md's "ROM image"), empty for a 74ls189. The
    /// stub ignores both arguments.
    pub fn power_on(ic: &IcRecord, rom: &[u32]) -> Self {
        let _ = (ic, rom);
        MemState
    }
}

/// One evaluation of a powered 74ls189/28c16/28c256 (memories section points
/// 3-4) — see `sequential::evaluate`'s doc comment for the parameter
/// convention this mirrors exactly.
pub fn evaluate(ic: &IcRecord, state: &mut MemState, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64) -> FamilyOutput {
    let _ = (state, reads, last_reads, edges_live, now);
    let outputs = super::output_capable(ic.kind).len();
    let slots = kinds::shape(ic.kind).map_or(0, |s| (s.digital_slots)(ic.logic.len() + 2));
    FamilyOutput {
        requests: vec![(Mode::Hiz, DelaySet::uniform(0.0)); outputs],
        slots: vec![f64::NAN; slots],
        timers: Vec::new(),
    }
}

/// Replaces a 74ls189's 16 stored nibbles (`bo_engine_seed_memory`, owner
/// decision 2). `devices::ics::IcEvaluator::seed_memory` already routes a
/// non-189 kind to `Err("not a memory")` itself and never reaches here for
/// one; this function still checks the word count defensively
/// (core/README.md: "`len_f64 != 16` ... is `-8`, validated in full before
/// anything changes") and otherwise declines — D2 wires the real 16-nibble
/// store this validates the values against and writes into.
pub fn seed(ic: &IcRecord, state: &mut MemState, words: &[f64]) -> Result<(), String> {
    let _ = (ic, state);
    if words.len() != 16 {
        return Err(format!("seed_memory: expected 16 words, got {}", words.len()));
    }
    Err("74ls189 seeding is not implemented yet".to_string())
}

/// Collects and clears this device's dirty EEPROM image
/// (`bo_engine_take_eeprom_updates`). The stub never has anything dirty — D2
/// wires the real per-byte image and `dirty` flag this drains.
pub fn take_update(state: &mut MemState) -> Option<Vec<u32>> {
    let _ = state;
    None
}
