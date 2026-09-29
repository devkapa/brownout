//! The NE555 (core/README.md, "Kernel semantics: NE555 (W1e)"). Owned by W1e
//! worker E1.
//!
//! The public API below is the orchestrator's lock, declared before dispatch
//! so no worker needs to touch `digital/mod.rs`. The bodies are stubs until
//! E1 lands (the underscores go with them): nothing in the dispatcher or the
//! solver calls them yet.

use super::power::IcPower;
use super::IcRecord;
use crate::drivers::{Driver, DriverStage};
use crate::events::Mode;
use crate::solve::Threshold;

/// Logic indices into `IcRecord::logic` (core/README.md, "NE555 record").
pub use super::{NE555_CV as CV, NE555_DIS as DIS, NE555_OUT as OUT, NE555_RST as RST, NE555_THR as THR, NE555_TRIG as TRIG};

/// The latch, the 555's only state (NE555 point 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ne555State {
    pub out_high: bool,
}

impl Ne555State {
    /// Classic's `{outHigh: true}` (DI:233).
    pub fn power_on() -> Self {
        Ne555State { out_high: true }
    }
}

/// NE555 point 3: the comparators over inputs already relative to pin 1,
/// returning the new latch value.
pub fn resolve(_stored: bool, _v_rst: f64, _v_trig: f64, _v_thr: f64, _vs: f64) -> bool {
    todo!("W1e E1")
}

/// One evaluation (NE555 points 2-3 and 9): `None` while unpowered; writes
/// `state` only when `acting_on_edges`.
pub fn evaluate(_ic: &IcRecord, _state: &mut Ne555State, _voltages: &[f64], _acting_on_edges: bool) -> Option<bool> {
    todo!("W1e E1")
}

/// OUT's fraction toward pin 8 (NE555 point 7).
pub fn out_fraction(_high: bool, _vs: f64) -> f64 {
    todo!("W1e E1")
}

/// The `(target node, conductance)` branches a 555 driver stamps from its
/// pin (NE555 point 7): at most two for OUT, one for DIS.
pub fn branches(_stage: DriverStage, _committed: Option<Mode>, _power: &IcPower, _driver: &Driver, _ic: &IcRecord) -> [Option<(usize, f64)>; 2] {
    todo!("W1e E1")
}

/// The current delivered out of pin 3 (NE555 point 8).
pub fn element_current(_high: bool, _vs: f64, _v_out: f64, _v_gnd: f64, _v_vcc: f64) -> f64 {
    todo!("W1e E1")
}

/// The crossing thresholds for one read pin, both directions (NE555 point
/// 5).
pub fn watch_thresholds(_logic_index: usize, _v_vcc: f64, _v_gnd: f64) -> Vec<Threshold> {
    todo!("W1e E1")
}
