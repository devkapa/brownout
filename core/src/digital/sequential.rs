//! The register family: 74ls161, 74ls173, 74hc74 (core/README.md, "Kernel
//! semantics: sequential ICs (W1d)"). This commit (W1d worker K1, kernel
//! infrastructure) only stubs the interface `devices::ics`'s dispatcher
//! wires end to end — `SeqState`'s real fields (161's `{count, lastClk}`,
//! 173's `{q1..q4, lastClk}`, 74hc74's `{q1, q1n, lastClk1, q2, q2n,
//! lastClk2}`) and [`evaluate`]'s real per-kind logic (contract points 4-7)
//! are D1's scope, not this file's.
//!
//! [`evaluate`] drives nothing (every output-capable pin requests `Hiz`, at
//! zero delay — irrelevant on a first observation, `drivers::fsm::pick`'s
//! own `none -> any` row) and reports every digital slot `NaN`, so the
//! dispatcher (register creation, powered/unpowered branching, slot
//! publishing, snapshot/restore) compiles and round-trips through a real
//! `Engine::load` before D1's real state machines land.

use super::{FamilyOutput, IcRecord};
use crate::drivers::DelaySet;
use crate::events::Mode;
use crate::kinds;

/// Register state for one 161/173/74hc74 device (contract point 2). Empty
/// for now. `Clone` so `devices::ics::IcEntry` (which owns one per device)
/// rolls back with a failed instant, same as every other piece of per-device
/// evaluator state (`core/README.md`, "A failed advance").
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SeqState;

impl SeqState {
    /// Power-on values (contract point 2: classic's `defaultIcState`, all 0
    /// except `q1n = q2n = 1`). The stub ignores `ic` — nothing here yet
    /// depends on which of the three kinds it is.
    pub fn power_on(ic: &IcRecord) -> Self {
        let _ = ic;
        SeqState
    }
}

/// One evaluation of a powered 161/173/74hc74 (contract point 3): `reads` is
/// this delta's logic-pin levels (in pin order), `last_reads` the same at
/// the IC's own previous powered evaluation (`None` before the first),
/// `edges_live` whether this call may act on an edge (`false` only during
/// `Kernel::settle`'s frozen fixed point — contract point 3(b): "skip (b)
/// entirely"), `now` the current simulation time.
pub fn evaluate(ic: &IcRecord, state: &mut SeqState, reads: &[bool], last_reads: Option<&[bool]>, edges_live: bool, now: f64) -> FamilyOutput {
    let _ = (state, reads, last_reads, edges_live, now);
    let outputs = super::output_capable(ic.kind).len();
    let slots = kinds::shape(ic.kind).map_or(0, |s| (s.digital_slots)(ic.logic.len() + 2));
    FamilyOutput {
        requests: vec![(Mode::Hiz, DelaySet::uniform(0.0)); outputs],
        slots: vec![f64::NAN; slots],
        timers: Vec::new(),
    }
}
