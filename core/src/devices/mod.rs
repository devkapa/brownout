//! The device stamp API (W1 plan section 3.2, row B1/item 3): what a purely
//! electrical, non-inertial device contributes to its own partition's
//! static solve, and how it reports its own current afterward.
//! [`passives`] (the resistor) and [`switches`] (switch, push_button,
//! spdt_switch, dip_switch) are the only kinds behind it through W1b step 1.
//! [`variable`] (W1h: potentiometer, trimmer, resistor_array, ldr, push_dpdt,
//! current_source) is behind it since H1a, routed by the solver since H1b
//! ([`linear`], [`union_rule`]); the inductors, the
//! crystal and the thermistor are stepped-partition devices and, like the
//! capacitor, are not dispatched through it.
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
pub mod variable;

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

/// Whether a linear (stamp API) device's params can move after load, which
/// decides the list it joins in every solver (W1h H1b). `Fixed` stamps are
/// folded once into a static partition's pristine base; `Live` ones are
/// restamped every resolve and take `bo_engine_set_params` writes (the switch
/// family's list since W1b, now every kind with a non-empty `live_params`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Linear {
    Fixed,
    Live,
}

/// The stamp API's kinds and their list, `None` for every other kind. The one
/// place a new resistive kind is added, so the solver arms that dispatch on it
/// (partition, static, closed-form, stepped, hub, the KCL walk) cannot miss it.
pub fn linear(kind: u32) -> Option<Linear> {
    match kind {
        kinds::RESISTOR | kinds::FUSE | kinds::PTC_FUSE | kinds::RESISTOR_ARRAY => Some(Linear::Fixed),
        kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH => Some(Linear::Live),
        kinds::POTENTIOMETER | kinds::TRIMMER | kinds::LDR | kinds::PUSH_DPDT | kinds::CURRENT_SOURCE => Some(Linear::Live),
        _ => None,
    }
}

/// How `partition::build` joins a two-terminal or multi-contact device's free
/// pins (topology only; the stamp reads the current params).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Union {
    /// One pin pair, joined when both ends are free (a resistor, a capacitor,
    /// an LDR, a current source: R56 9 always joins its two pins).
    Pair(usize, usize),
    /// Each `resistor_array` channel `(2i, 2i + 1)` on its own: the channels
    /// are isolated resistors and are never joined to each other.
    Channels,
    /// Every free pin of the device in one partition, whatever its contacts
    /// (the switch family's rule: a contact can run through a fixed pin). A
    /// pot's three pins are one network; a dpdt's two poles share a partition,
    /// a conservative union a later pass may refine to per pole.
    Whole,
}

/// The [`Union`] rule of `kind`, `None` for a kind joined some other way
/// (junctions, IC pins) or not at all.
pub fn union_rule(kind: u32) -> Option<Union> {
    match kind {
        kinds::RESISTOR | kinds::CAPACITOR | kinds::FUSE | kinds::PTC_FUSE | kinds::LDR | kinds::CURRENT_SOURCE | kinds::INDUCTOR => Some(Union::Pair(0, 1)),
        // W1h H3b: the Norton form's stamp joins its two pins; the ideal and
        // disabled forms stamp nothing (`partition::joins`).
        kinds::SIGNAL_GEN => Some(Union::Pair(0, 1)),
        kinds::RESISTOR_ARRAY => Some(Union::Channels),
        // W1h H4b: a coupled inductor's two windings are galvanically apart
        // but coupled through `M` (the 2x2 companion), so all four free pins
        // solve together: `Whole`, not `galvanicGroups`.
        kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH | kinds::POTENTIOMETER | kinds::TRIMMER | kinds::PUSH_DPDT | kinds::COUPLED_INDUCTOR => Some(Union::Whole),
        _ => None,
    }
}

/// Whether `kind`'s published current is the sum of its contacts' `g dV`, the
/// rule every solver has applied to the resistor, fuse, PTC and switch family
/// since W0 (kept as it is, so their readings stay bit for bit). Every other
/// stamp-API kind publishes [`current`] instead: a pot's `elementI` is its
/// cw-wiper segment, a dpdt's is pole 1, a current source's carries `-I`, which
/// no contact sum gives (H1aV note 1).
pub fn current_is_contact_sum(kind: u32) -> bool {
    matches!(kind, kinds::RESISTOR | kinds::FUSE | kinds::PTC_FUSE | kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH)
}

/// Whether contribution `k` of [`stamp`]`(kind, ..)` is part of `kind`'s
/// published current, for the bounds that follow a published current through
/// its contacts (`solve::curvature`, the closed-form coupling quantities). Every
/// contact of a contact-sum kind; the pot's cw-wiper segment (0); the dpdt's
/// pole-1 contact to its active throw; the single contact of an LDR or current
/// source and every channel of an array.
pub fn in_element_current(kind: u32, params: &[f64], k: usize) -> bool {
    match kind {
        kinds::POTENTIOMETER | kinds::TRIMMER => k == 0,
        kinds::PUSH_DPDT => k == usize::from(params[kinds::dpdt_param::POSITION] != 0.0),
        _ => true,
    }
}

/// A `resistor_array`'s four channel pin pairs (`a_i`, `b_i`), in channel order.
pub const ARRAY_CHANNEL_PINS: [(usize, usize); variable::ARRAY_CHANNELS] = [(0, 1), (2, 3), (4, 5), (6, 7)];

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
        // W1h H1a: the variable resistive kinds and the current source (a stub
        // until H1a lands; `variable::handles` owns the list).
        _ if variable::handles(kind) => variable::stamp(kind, pin_count, params),
        // W1h H3b: only the Norton form reaches a stamp path (`solve::placement_of`).
        kinds::SIGNAL_GEN => sources::signal_gen_stamp(params),
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
        _ if variable::handles(kind) => variable::current(kind, pin_count, params, pin_voltages),
        kinds::SIGNAL_GEN => sources::signal_gen_current(params, pin_voltages),
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
    let Some(record_len) = slow_record_len(kind) else { return false };
    params.get(record_len).is_some_and(|&f| f != 0.0)
}

/// Whether `kind` has a slow flag ([`slow_flag`]).
pub fn has_slow_flag(kind: u32) -> bool {
    slow_record_len(kind).is_some()
}

/// The record length after which a kind's slow flag sits, `None` for a kind
/// that has none. W1h H1c: a `potentiometer` or `trimmer` latches open as a
/// whole (classic's one `resistor_overload` per part), so its flag follows
/// `[rCw, rCcw, pMax]`; a knob move writes params 0 and 1 and leaves it.
fn slow_record_len(kind: u32) -> Option<usize> {
    match kind {
        kinds::RESISTOR => Some(1),
        kinds::FUSE => Some(kinds::fuse_param::COUNT),
        kinds::PTC_FUSE => Some(kinds::ptc_param::COUNT),
        kinds::POTENTIOMETER | kinds::TRIMMER => Some(kinds::pot_param::COUNT),
        _ => None,
    }
}

/// `params` with the slow flag set as given: the record's own params, plus
/// the flag only when it is set, so an intact device's params are the record
/// bit for bit (and so is every cache or segment key built from them).
pub fn with_slow_flag(kind: u32, record: &[f64], flag: bool) -> Vec<f64> {
    let record_len = slow_record_len(kind).unwrap_or(record.len());
    let mut out = record[..record_len.min(record.len())].to_vec();
    if flag {
        out.push(1.0);
    }
    out
}

/// W1h H1c: a `resistor_array`'s latched channels as the stamp reads them,
/// bit `c` for channel `c`: one trailing value after `[R, pMax]`, present only
/// once a channel has latched (`resistor_overload` keyed `ch<c + 1>`), as the
/// slow flag is. The channels as every solver stamps them are plain resistor
/// records, which carry the resistor's own flag; this mask is what the
/// array's own stamp, its channel currents and its `elementI` read.
pub fn array_open_mask(params: &[f64]) -> u32 {
    params.get(kinds::array_param::COUNT).map_or(0, |&m| m as u32)
}

/// `record` with the open-channel mask set as given (none when 0).
pub fn with_array_open_mask(record: &[f64], mask: u32) -> Vec<f64> {
    let mut out = record[..kinds::array_param::COUNT.min(record.len())].to_vec();
    if mask != 0 {
        out.push(f64::from(mask));
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

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

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

impl crate::snapshot::HeapBytes for Linear {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Linear::Fixed => 0,
            Linear::Live => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for Union {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Union::Pair(f0, f1) => crate::snapshot::HeapBytes::heap_bytes(f0, w) + crate::snapshot::HeapBytes::heap_bytes(f1, w),
            Union::Channels => 0,
            Union::Whole => 0,
        }
    }
}
