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
//! - W1e's LED junctions ([`led`], [`seg7`]) are not behind it either: a
//!   junction's companion depends on the Newton iterate, and one device's
//!   junctions can land in several partitions, so the solver places and
//!   stamps them by node.

pub mod battery;
pub mod ics;
pub mod led;
pub mod passives;
pub mod seg7;
pub mod sources;
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
        kinds::RESISTOR => open_if_failed(passives::resistor_stamp(params, slow_flag(kind, params))),
        kinds::FUSE => open_if_failed(passives::fuse_stamp(params, slow_flag(kind, params))),
        kinds::PTC_FUSE => passives::ptc_stamp(params, slow_flag(kind, params)),
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
        kinds::RESISTOR => passives::resistor_current(params, pin_voltages, slow_flag(kind, params)),
        kinds::FUSE => passives::fuse_current(params, pin_voltages, slow_flag(kind, params)),
        kinds::PTC_FUSE => passives::ptc_current(params, pin_voltages, slow_flag(kind, params)),
        kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH => switches::current(kind, pin_count, params, pin_voltages),
        _ => 0.0,
    }
}

/// W1g's slow state as the stamp reads it (core/README.md W1g points 12 and
/// 17: every stamp path reads the failed flags and the PTC resistance through
/// one accessor). The solver keeps a device's stamp params as its record
/// params plus, once a latch or a PTC trip has changed the stamp, one trailing
/// flag at the record's own length: `1` is a failed `resistor` or `fuse`, or a
/// tripped `ptc_fuse`. The reader never sees it (each kind's param count is
/// exact), so the record itself always reads intact; the solver writes it in
/// exactly one place (`solve::PartitionedSolver`'s stamp refresh) and drops it
/// again at a reset. Every stamp path already reads a device's params through
/// `stamp` (the static base, `stamp_linear_base` for closed-form and stepped
/// partitions, the rail sensitivities and the KCL contributions), so they all
/// see the same state.
pub fn slow_flag(kind: u32, params: &[f64]) -> bool {
    let record_len = match kind {
        kinds::RESISTOR => 1,
        kinds::FUSE => kinds::fuse_param::COUNT,
        kinds::PTC_FUSE => kinds::ptc_param::COUNT,
        _ => return false,
    };
    params.get(record_len).is_some_and(|&f| f != 0.0)
}

/// `params` with the slow flag set as given: the record's own params, plus
/// the flag only when it is set, so an intact device's params are the record
/// bit for bit (and so is every cache or segment key built from them).
pub fn with_slow_flag(kind: u32, record: &[f64], flag: bool) -> Vec<f64> {
    let record_len = match kind {
        kinds::RESISTOR => 1,
        kinds::FUSE => kinds::fuse_param::COUNT,
        kinds::PTC_FUSE => kinds::ptc_param::COUNT,
        _ => record.len(),
    };
    let mut out = record[..record_len.min(record.len())].to_vec();
    if flag {
        out.push(1.0);
    }
    out
}

/// A failed two-terminal device is an open (W1g point 12): it stamps nothing.
/// It is kept as one zero-conductance contribution rather than none, as an
/// open switch contact is, because every consumer indexes a device's
/// contributions by position (the static solver's KCL terms, the hub's direct
/// contacts), and a `g = 0` contribution is already "stamps nothing"
/// everywhere a stamp is folded (`g == 0` is skipped) and adds `0` to every
/// current sum.
fn open_if_failed(stamp: Vec<Contribution>) -> Vec<Contribution> {
    if stamp.is_empty() {
        vec![Contribution::conductance(0, 1, 0.0)]
    } else {
        stamp
    }
}

// ---- W2a: heap bytes (generated by the size walk generator; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Contribution {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pin_a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pin_b, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.conductance, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.injection, w); }
        n
    }
}
