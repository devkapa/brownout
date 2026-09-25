//! The device stamp API (W1 plan section 3.2, row B1/item 3): what a purely
//! electrical, non-inertial device contributes to its own partition's
//! static solve, and how it reports its own current afterward.
//! [`passives`] (the resistor) and [`switches`] (switch, push_button,
//! spdt_switch, dip_switch) are the only kinds behind it through W1b step 1.
//!
//! Kept as a small kind-code dispatch (`stamp`/`current` below), matching
//! the rest of the crate's style (`kinds::shape`), rather than a trait
//! object per device: every call site already has the device's `u32` kind
//! code in hand (`program::Device::kind`), so a `match` is simpler than a
//! vtable and costs nothing extra.
//!
//! Designed to grow, not to change shape:
//! - W1b step 2's capacitor is the first exception to "kind code in, stamp
//!   out": its BE/trap companion (spec §7.4) depends on DYNAMIC history (the
//!   step size `h`, the last committed terminal voltage, and — for trap —
//!   the last accepted series current) that `stamp`/`current`'s signatures
//!   below have no way to carry, so it is deliberately NOT dispatched
//!   through them. Only its LEAKAGE — a plain, params-only shunt, exactly
//!   like a resistor — goes through the ordinary `stamp` dispatch below;
//!   `solve::closed_form`/`solve::stepped` call
//!   `devices::passives::capacitor_be_companion`/`capacitor_trap_companion`
//!   directly, with the history only they own. See `passives.rs`'s module
//!   doc comment.
//! - W1c's IC output stages are NOT behind this API at all: an IC output is
//!   a [`crate::drivers::Driver`], an inertial, event-scheduled entity the
//!   kernel itself must own the commit/reset lifecycle of
//!   (`solve::PartitionSolver::resolve_dirty` already stamps a `Driver`
//!   directly, in `solve/static_part.rs`), not a plain per-resolve stamp a
//!   device hands back on request.

pub mod passives;
pub mod switches;

use crate::kinds;

/// One two-terminal linear contribution between two of a device's OWN pins
/// (indices into that device's `pins`/`pin_voltages` arrays — never a raw
/// node id; the caller resolves each side to a local partition index or a
/// fixed voltage, exactly as `solve/static_part.rs` already does for a
/// resistor: added to both diagonals and subtracted cross-wise when both
/// ends are free, or folded into the fixed-neighbour right-hand side when
/// one end is fixed). `injection` is an independent Norton current on top of
/// `conductance`, landing on the same two terminals; every kind through W1b
/// step 1 leaves it `0.0` — step 2's capacitor companion is the first to
/// need it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contribution {
    pub pin_a: usize,
    pub pin_b: usize,
    pub conductance: f64,
    pub injection: f64,
}

impl Contribution {
    /// A plain conductance term, no independent source — every W1b
    /// step 1 kind's only shape.
    pub fn conductance(pin_a: usize, pin_b: usize, conductance: f64) -> Self {
        Contribution { pin_a, pin_b, conductance, injection: 0.0 }
    }
}

/// This device's current stamp, in terms of its own pin indices. Empty for
/// any kind not listed here (a `voltage_source`'s contribution is fixed-node
/// propagation, not a matrix stamp — `partition::derive_fixed_nodes` — and a
/// digital driver is not this API at all; see the module doc).
///
/// `CAPACITOR` here is its LEAKAGE ONLY (a plain shunt, exactly like a
/// resistor) — its ESR/BE/trap companion is not a function of `params`
/// alone and is never reached through this dispatch; see the module doc.
pub fn stamp(kind: u32, pin_count: usize, params: &[f64]) -> Vec<Contribution> {
    match kind {
        kinds::RESISTOR => passives::stamp(params),
        kinds::CAPACITOR => passives::leakage_stamp(params),
        kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH => switches::stamp(kind, pin_count, params),
        _ => Vec::new(),
    }
}

/// This device's published current (`elementI`), given each of its own
/// pins' current voltage (parallel to the device's `pins`).
///
/// `CAPACITOR` is deliberately absent (falls to the `0.0` default): its real
/// current (series plus leakage, spec §7.4) needs the dynamic history this
/// signature cannot carry, so `solve::closed_form`/`solve::stepped` publish
/// it directly into the shared currents array instead of through this
/// function — mirroring how `StaticPartitionSolver::recompute_currents`
/// already computes a `voltage_source`'s current with its own dedicated KCL
/// walk rather than through this generic dispatch.
pub fn current(kind: u32, pin_count: usize, params: &[f64], pin_voltages: &[f64]) -> f64 {
    match kind {
        kinds::RESISTOR => passives::current(params, pin_voltages),
        kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH => switches::current(kind, pin_count, params, pin_voltages),
        _ => 0.0,
    }
}
