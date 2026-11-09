//! The static partition solver (core/README.md, "W0 kernel semantics"),
//! moved from `engine.rs` (W1 plan section 3.2, row B1) and extended for
//! W1b step 1 with the switch family's fresh-every-resolve stamping and the
//! solution cache (row B1's second half).
//!
//! W1b step 2 (rows B2/B3) adds `ClosedForm`/`Stepped` partitions
//! (`solve/closed_form.rs`, `solve/stepped.rs`), owned by `solve::mod`'s
//! `PartitionedSolver`, not by this module. [`StaticPartitionSolver::build`]
//! now skips any resistor or switch-family device whose free pin(s) land in
//! a non-`Static` partition (`partition::PartitionKind`) — such a device is
//! entirely a dynamic partition's own resistive-network stamp instead (a
//! resistor in series with a capacitor, say), never split between the two —
//! and never allocates a matrix stamp for a non-`Static` partition's own
//! shunt row. `PartitionedSolver` only ever calls
//! [`PartitionSolver::resolve_dirty`] here with `Static`-classified
//! partition ids, so this module needs no defensive check of its own against
//! being handed one it does not own.
//!
//! Re-solving a dirty partition means cloning its pristine per-partition
//! base (shunt + every immutable static-stamp device — today, only the
//! resistor), stamping every currently-committed driver on top of it
//! (unchanged from W1a) and every switch-family device's CURRENT contact
//! state on top of that (new here — a switch's `closed`/`sel`/`sw_i` can
//! change after load, `bo_engine_set_params`, so nothing about it can be
//! baked into the pristine base the way an immutable resistor's `R` is),
//! then a dense LU solve, exactly as W0/W1a's did. The solution cache (below)
//! sits in front of that LU solve, keyed on everything that can change what
//! the clone-and-stamp step produces without changing the partition's own
//! topology.
//!
//! **Iterated partitions (W1c point 3, W1e LEDs points 6, 7 and 9).** A
//! partition holding a free IC supply node or a stamped LED junction is
//! solved by [`newton::solve`], restamped from each iterate from the
//! partition's last published voltages, with classic's per-node test and cap
//! of 50 iterations (`-4` past it). One holding a junction then runs a
//! single polish iteration, so what it publishes is the fixed point classic
//! settles on rather than the first passing iterate. One holding a free IC
//! supply node bypasses the solution cache (its stamp follows its own
//! solution); one holding only junctions keeps it, and stores the polished
//! result. Every other partition takes one direct LU solve, unchanged.
//!
//! Devices in a partition are represented two ways, matching whether their
//! electrical state can change after load:
//! - **Resistors** are folded into `matrix_base`/`rhs_base` once, at
//!   [`StaticPartitionSolver::build`] — `devices::passives::stamp` is called
//!   exactly once per resistor, exactly as before.
//! - **Switch-family devices** keep their own mutable, current `params`
//!   (mutated by [`PartitionSolver::apply_params`]) and are stamped fresh —
//!   `devices::switches::stamp` — on every [`PartitionSolver::resolve_dirty`]
//!   call that touches their partition, the same way a driver's committed
//!   level already is.

use std::collections::{BTreeSet, HashMap};

use crate::devices::{self, led};
use crate::digital;
use crate::drivers::Driver;
use crate::error::BoError;
use crate::events::{DeviceId, Mode, PartitionId};
use crate::kinds;
use crate::lu;
use crate::partition::{self, PartitionKind, ResolvedSource};
use crate::program::Program;
use crate::solve::newton;
use crate::rails::{self, Contrib};
use crate::solve::{
    collect_ics, compile_dyn_shape, ic_driver_branches, ic_pull_target, junction_branch_current, junction_partition, place_junction, rail_sensitivity,
    pin_voltage, stamp_ics_into_dyn, stamp_junctions, stamp_linear_base, Bnd, DynJunction, DynPin, DynShape, PartitionSolver,
};

/// The rounding floor of a curved hub solve, against W0's tolerance for
/// its quantity, past which the solve is refined (ruling R23 (ii), FXE
/// 2026-10-21; `StaticPartitionSolver::refine_hub`).
const REFINE_GATE: f64 = 0.01;

/// Every node except node 0 carries this conductance to the datum (contract,
/// "W0 kernel semantics", point 2): it keeps an isolated net solvable (reads
/// 0 V) and its current is folded into every KCL sum below, exactly as the
/// classic engine stamps it on every node row.
pub const NODE_RSHUNT_G: f64 = 1e-12;

/// The most keys one partition's cache keeps. Reaching it empties that
/// partition's map (a cache only ever costs a solve, never a wrong answer),
/// which bounds memory for a partition whose keys never repeat, and drops the
/// entries a bumped epoch made unreachable.
const CACHE_ENTRIES_PER_PARTITION: usize = 256;

/// One contact of a static-stamp device, reduced to what every consumer of
/// `devices::stamp` reads: the two NODES it joins and its conductance. The
/// zero-conductance ("open") contacts are kept: the current walk has always
/// summed them (a `0.0 * dV` term is `+-0.0` or NaN, and adding it can
/// change a sum's sign of zero), so dropping them would not be identical.
#[derive(Debug, Clone, Copy)]
struct Contact {
    u: usize,
    w: usize,
    g: f64,
}

/// `devices::stamp(kind, pins.len(), params)` reduced to [`Contact`]s. Both
/// callers used to allocate that list on every resolve; it is a pure function
/// of `(kind, pin count, params)`, so it is computed once and again only when
/// `apply_params` changes `params`.
fn contacts_of(kind: u32, pins: &[usize], params: &[f64]) -> Vec<Contact> {
    devices::stamp(kind, pins.len(), params).into_iter().map(|c| Contact { u: pins[c.pin_a], w: pins[c.pin_b], g: c.conductance }).collect()
}

/// One static-stamp device's electrical identity: which device it is, its
/// kind, its own pins (as node ids, in device-pin order) and its current
/// params. Used for BOTH the immutable resistor list (params never change
/// after `build`) and the mutable switch-family list (`params` is exactly
/// what [`PartitionSolver::apply_params`] mutates in place) — the same shape
/// either way, since `devices::stamp`/`devices::current` only ever need a
/// kind, a pin count and a params slice.
#[derive(Debug, Clone)]
struct StaticStamp {
    device_index: usize,
    kind: u32,
    pins: Vec<usize>,
    params: Vec<f64>,
    /// `devices::stamp` of `params`, always in step with them.
    contacts: Vec<Contact>,
}

/// One term of a source child node's KCL sum (`leaving[child]`), in the exact
/// order the whole-circuit walk has always added them: every contact of every
/// resistor and then every switch (a contact adds to its `u` end and then
/// subtracts from its `w` end), then every stamped junction (anode, then
/// cathode), then the node's shunt. The sum is folded from `0.0` in that
/// order, so it is bit-identical to the whole-circuit array walk it replaces.
#[derive(Debug, Clone, Copy)]
enum Term {
    /// `stamp` indexes resistors then switches as one list.
    Contact { stamp: u32, contact: u32, plus: bool },
    Junction { junction: u32, plus: bool },
    Shunt,
}

/// A junction's last branch current and the two voltages it was computed
/// from. `exp` is the one expensive term of the source KCL, and it is a pure
/// function of those two voltages, so the same bits give the same current.
#[derive(Debug, Clone, Copy, Default)]
struct JunctionMemo {
    v_anode: u64,
    v_cathode: u64,
    /// The junction's law generation when the current was computed (W1g
    /// point 4): a re-arm changes the law without moving either voltage, so a
    /// memo of the old law must not be served.
    law_gen: u32,
    current: f64,
    valid: bool,
}

/// W1g's slow state of one device as the static solver stamps it (README
/// W1g point 12): a failed `resistor`, `fuse` or LED-family device, a tripped
/// `ptc_fuse`, a thermal `led`'s law at its held temperature, and that law's
/// generation, which keys the junction memo. The default is the device as
/// loaded.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct StampState {
    pub failed: bool,
    pub tripped: bool,
    pub law: Option<led::JunctionLaw>,
    pub law_gen: u32,
}

/// One partition's solution cache entry. A hub partition's entry also holds
/// the rail sensitivity of its solve (README, Rails point 3, "Static cache").
#[derive(Clone, Debug)]
struct CacheEntry {
    hash: u64,
    key: Box<[u8]>,
    x: Box<[f64]>,
    hub: Option<Box<HubSolve>>,
}

/// A static hub partition's latest solve at rail `u_p`: its solution `x`,
/// sensitivity `s`, draw `i` and slope `g` (Rails point 2). It publishes `x +
/// s (u - u_p)`, within `band` of `u_p` (Rails point 3): its curvature band,
/// `RAIL_RESOLVE_V` for a linear one.
#[derive(Debug, Clone, PartialEq)]
struct HubSolve {
    x: Vec<f64>,
    s: Vec<f64>,
    i: f64,
    g: f64,
    u_p: f64,
    band: Option<f64>,
}

/// A hub partition's IC stamps that are no fixed conductance, by what its
/// band reads (Rails point 3): none (a linear partition), those that follow
/// the rail (`curvature::hub_band`), or, holding a free IC supply, every
/// one that follows a supply (`curvature::supply_band`).
enum HubIc {
    Linear,
    Rail(crate::solve::curvature::IcRail),
    Supply(crate::solve::curvature::IcSupply),
}

/// A hub partition's solve as a whole snapshot holds it (opaque outside this
/// module).
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct HubSolveSnapshot(HubSolve);

/// What a static hub partition needs for its residual stamps: the partition
/// compiled as a dynamic shape (its resistors, switches and junctions), the
/// node map and ICs `stamp_ics_into_dyn` reads, and whether its response to
/// the rail is curved (re-solved past the band, Rails point 3): it holds a
/// junction, a free IC supply, or a pin of an IC supplied from the rail,
/// whose driver's `R_out` follows the rail even into a linear network.
#[derive(Clone, Debug)]
struct StaticHub {
    shape: DynShape,
    node_owner: HashMap<usize, (usize, usize)>,
    touch_ics: Vec<(usize, digital::IcRecord)>,
    nonlinear: bool,
    solve: Option<HubSolve>,
}

/// One partition's solution cache (W1 plan row B1). The key is exactly what
/// the old `CacheKey` struct held, packed into bytes so a lookup allocates nothing
/// and hashes without SipHash:
/// - `(driver id, committed mode)` for every driver whose node lives in this
///   partition and that has a committed mode, ascending id (an IC output's
///   `device`/`open_collector` are fixed identity, so the committed mode
///   alone is enough to key on, like any other driver);
/// - one `high` bit per would-be-pulled pin touching this partition (an
///   `OPEN` `FLOATING_INPUT`, non-`OPEN_HIGH` logic pin), in a fixed
///   enumeration order (device order, then logic-pin order). This partition
///   never reaches the cache when it holds a free IC supply node, so an IC
///   touching it always has BOTH supply nodes fixed and
///   `digital::power::ic_power`'s result is a build-time constant that needs
///   no key entry; only the floating model's bucket-dependent `high` bit can
///   still move the stamp from one resolve to the next. Included regardless
///   of whether the owning IC is CURRENTLY powered (a cheap, always-safe
///   over-inclusion: an unpowered IC's pull never stamps either way);
/// - `epoch`, bumped by `apply_params` on every live-parameter change (owner
///   decision 1), the only thing that can change a switch-family device's
///   stamp after load. A single, solver-wide counter: a bump from an
///   unrelated partition's toggle costs another partition nothing unless it
///   is independently dirtied later anyway, in which case recomputing (rather
///   than serving a stale hit) is exactly what must happen. Entries under an
///   older epoch stay (unreachable) until the map empties, and count toward
///   [`CACHE_ENTRIES_PER_PARTITION`] exactly as they always did.
///
/// W1f appends, for a hub partition only, the rail band `round(u /
/// RAIL_BUCKET_V)` and the power gate of each IC on the rail touching it, and
/// for a partition reading a waveform source, that source's level index; a
/// partition with neither keys exactly as before. Which keys hit, and when
/// the map empties, are exactly what they were with the `HashMap`: a hit is
/// what a junction partition's `iterated_solve` result depends on (it starts
/// from the last published voltages), so that pattern is part of the
/// results.
#[derive(Clone, Debug, Default)]
struct PartCache {
    entries: Vec<CacheEntry>,
    /// Open-addressed table of `entries` indices plus one (`0` is empty),
    /// keyed by the low bits of the entry's hash; a power-of-two length, at
    /// most half full.
    slots: Vec<u32>,
}

impl PartCache {
    fn find(&self, hash: u64, key: &[u8]) -> Option<usize> {
        if self.slots.is_empty() {
            return None;
        }
        let mask = self.slots.len() - 1;
        let mut i = (hash as usize) & mask;
        loop {
            let slot = self.slots[i];
            if slot == 0 {
                return None;
            }
            let e = &self.entries[slot as usize - 1];
            if e.hash == hash && *e.key == *key {
                return Some(slot as usize - 1);
            }
            i = (i + 1) & mask;
        }
    }

    /// The first entry under `key` whose hub solve `accept`s, past any that
    /// does not: a hub partition stores one entry per rail it solved at, all
    /// under one key (Rails point 3).
    fn find_hub(&self, hash: u64, key: &[u8], accept: impl Fn(&HubSolve) -> bool) -> Option<usize> {
        if self.slots.is_empty() {
            return None;
        }
        let mask = self.slots.len() - 1;
        let mut i = (hash as usize) & mask;
        loop {
            let slot = self.slots[i];
            if slot == 0 {
                return None;
            }
            let e = &self.entries[slot as usize - 1];
            if e.hash == hash && *e.key == *key && e.hub.as_deref().is_some_and(&accept) {
                return Some(slot as usize - 1);
            }
            i = (i + 1) & mask;
        }
    }

    fn insert(&mut self, hash: u64, key: &[u8], x: &[f64], hub: Option<Box<HubSolve>>) {
        if self.entries.len() >= CACHE_ENTRIES_PER_PARTITION {
            self.entries.clear();
            self.slots.fill(0);
        }
        if (self.entries.len() + 1) * 2 > self.slots.len() {
            let new_len = (self.slots.len() * 2).max(8);
            self.slots.clear();
            self.slots.resize(new_len, 0);
            let mask = new_len - 1;
            for (ix, e) in self.entries.iter().enumerate() {
                let mut i = (e.hash as usize) & mask;
                while self.slots[i] != 0 {
                    i = (i + 1) & mask;
                }
                self.slots[i] = ix as u32 + 1;
            }
        }
        let mask = self.slots.len() - 1;
        let mut i = (hash as usize) & mask;
        while self.slots[i] != 0 {
            i = (i + 1) & mask;
        }
        self.entries.push(CacheEntry { hash, key: key.into(), x: x.into(), hub });
        self.slots[i] = self.entries.len() as u32;
    }
}

/// Hashes a packed cache key eight bytes at a time (FxHash's mix). Only ever
/// used to pick a slot: a hit still compares the whole key.
fn hash_key(bytes: &[u8]) -> u64 {
    const K: u64 = 0x517c_c1b7_2722_0a95;
    let mut h: u64 = 0;
    let mut chunks = bytes.chunks_exact(8);
    for c in &mut chunks {
        h = (h.rotate_left(5) ^ u64::from_le_bytes(c.try_into().expect("chunk of 8"))).wrapping_mul(K);
    }
    let rest = chunks.remainder();
    if !rest.is_empty() {
        let mut buf = [0u8; 8];
        buf[..rest.len()].copy_from_slice(rest);
        h = (h.rotate_left(5) ^ u64::from_le_bytes(buf)).wrapping_mul(K);
    }
    // The length distinguishes keys that differ only in trailing zero bytes;
    // the fold moves the well-mixed high bits down to where the slot index
    // reads them.
    let h = (h.rotate_left(5) ^ bytes.len() as u64).wrapping_mul(K);
    h ^ (h >> 32)
}

pub(crate) fn mode_discriminant(m: Mode) -> u8 {
    match m {
        Mode::Hiz => 0,
        Mode::High => 1,
        Mode::Low => 2,
    }
}

/// Folds one `(u, w, g)` conductance into the pristine per-partition base —
/// exactly the resistor-folding logic W0 always had, generalized to take its
/// three numbers as arguments instead of being written inline once for the
/// resistor loop only. A `g` of exactly `0.0` is skipped outright (README:
/// "open stamps nothing" — nothing calls this with `g == 0` today, since
/// only the immutable resistor list ever reaches it, but the guard costs
/// nothing and keeps the function honest about its own contract).
#[allow(clippy::too_many_arguments)]
fn fold_static_stamp(
    partitions: &[Vec<usize>],
    local_index: &[Option<(usize, usize)>],
    fixed: &[Option<f64>],
    live: &[bool],
    matrix_base: &mut [Vec<f64>],
    rhs_base: &mut [Vec<f64>],
    live_rhs: &mut [Vec<(usize, f64, usize)>],
    u: usize,
    w: usize,
    g: f64,
) {
    if g == 0.0 {
        return;
    }
    match (local_index[u], local_index[w]) {
        (Some((pu, lu_)), Some((pw, lw))) => {
            debug_assert_eq!(pu, pw, "partition::build must have unioned both ends of any free-free stamp");
            let size = partitions[pu].len();
            let a = &mut matrix_base[pu];
            a[lu_ * size + lu_] += g;
            a[lw * size + lw] += g;
            a[lu_ * size + lw] -= g;
            a[lw * size + lu_] -= g;
        }
        // A live boundary node (the rail, a waveform source's node) is not
        // baked: its term is added at each stamp from its present value
        // (README, Rails point 7).
        (Some((pu, lu_)), None) => {
            let size = partitions[pu].len();
            matrix_base[pu][lu_ * size + lu_] += g;
            if live[w] {
                live_rhs[pu].push((lu_, g, w));
            } else {
                rhs_base[pu][lu_] += g * fixed[w].unwrap();
            }
        }
        (None, Some((pw, lw))) => {
            let size = partitions[pw].len();
            matrix_base[pw][lw * size + lw] += g;
            if live[u] {
                live_rhs[pw].push((lw, g, u));
            } else {
                rhs_base[pw][lw] += g * fixed[u].unwrap();
            }
        }
        (None, None) => {
            // Both ends fixed: no stamp. Current is Ohm's law on two
            // already-known voltages, not a solve.
        }
    }
}

#[derive(Clone, Debug)]
pub struct StaticPartitionSolver {
    /// Node ids per partition, in first-seen order (deterministic; contract
    /// point 3 of "W0 kernel semantics").
    partitions: crate::shared::Shared<[Vec<usize>]>,
    /// node -> (partition index, local index), `None` for a fixed node.
    local_index: crate::shared::Shared<[Option<(usize, usize)>]>,
    /// node -> fixed voltage, `None` for a free node. Node 0 is always
    /// `Some(0.0)`.
    fixed: crate::shared::Shared<[Option<f64>]>,
    /// Every source that fixed a node, in resolution order.
    resolved: crate::shared::Shared<[ResolvedSource]>,
    /// Pristine per-partition conductance matrix: shunt + resistor stamps
    /// only, never a driver's or a switch's. Cloned fresh before every
    /// solve so a driver or switch that has since changed never leaves a
    /// stale stamp behind.
    matrix_base: crate::shared::Shared<[Vec<f64>]>,
    /// Pristine per-partition right-hand side: resistor-to-fixed-neighbour
    /// stamps only.
    rhs_base: crate::shared::Shared<[Vec<f64>]>,
    /// Every resistor (immutable after `build`), kept so `recompute_currents`
    /// can recompute its published current.
    resistors: crate::shared::Shared<[StaticStamp]>,
    /// Every switch-family device, in device order. `params` is mutated in
    /// place by `apply_params` — the only state in this whole struct a live
    /// `bo_engine_set_params` call ever changes.
    switches: crate::shared::Shared<[StaticStamp]>,
    /// device index -> index into `switches`, for O(1) lookup during
    /// `validate_params`/`apply_params`/`param_value`.
    switch_index_by_device: crate::shared::Shared<HashMap<usize, usize>>,
    voltages: Vec<f64>,
    currents: Vec<f64>,
    /// `NO_SOLUTION_CACHE` (option key 2): bypasses the cache below when
    /// `true`, both for reading a hit and for writing a fresh solve back
    /// into it — so cache on and cache off run the identical LU-solve code
    /// path whenever they actually compute anything, which is what makes
    /// the two bit-identical (core/README.md's Options table; W1 plan row
    /// B1's verify step).
    no_solution_cache: bool,
    /// Bumped by `apply_params` on every live-parameter change.
    epoch: u64,
    /// partition index -> every key it has been solved under -> its solved
    /// local voltages ("cached per driver configuration", W1 plan; the
    /// prototype's `part.cache` is a map too). One entry per partition would
    /// never hit on a driver that alternates between two levels, which is
    /// what an LED on a clocked IC output does (W1e, LEDs point 7: "modes
    /// repeat every clock cycle"). Bounded by [`CACHE_ENTRIES_PER_PARTITION`].
    caches: Vec<crate::shared::Shared<PartCache>>,
    /// Contract stat 7.
    cache_hits: f64,
    /// Every combinational IC whose driver/pull pins reach a `Static`
    /// partition (`partition.rs`'s own `-3` decline guarantees every IC's
    /// driver/pull footprint is entirely `Static`-owned this step), by
    /// device index — `solve::collect_ics` over the WHOLE program, not
    /// filtered here, since `Self::stamp_ics_into`'s own per-driver/pull
    /// `in_partition` guard already skips anything that does not touch the
    /// partition being resolved.
    ics: crate::shared::Shared<[(usize, digital::IcRecord)]>,
    /// partition index -> whether it holds at least one free `vcc`/`gnd`
    /// node of some IC (W1c contract point 3): such a partition's R_out
    /// moves with its own solved state, so `Self::resolve_dirty` iterates it
    /// to a fixed point instead of solving it once and reading (or writing)
    /// the solution cache — see `Self::iterated_solve`.
    holds_free_ic_supply: crate::shared::Shared<[bool]>,
    /// partition index -> every stamped LED junction touching it, resolved
    /// against it (W1e, LEDs point 4); empty for a partition this solver does
    /// not own and for one holding none. A non-empty entry makes the
    /// partition nonlinear: iterated and polished by `Self::iterated_solve`.
    junctions: crate::shared::Shared<[Vec<DynJunction>]>,
    /// Every stamped junction this solver owns the current of: those in a
    /// `Static` partition and those across two fixed nodes (which belong to
    /// none). Folded into `Self::recompute_currents`'s source KCL.
    kcl_junctions: crate::shared::Shared<[led::Junction]>,
    /// Contract stat 11's static half: every iteration of every iterated
    /// partition, the polish and any failed attempt's included. Not rolled
    /// back, like every stat.
    newton_iterations: u64,
    /// node -> every IC device index that reads it (its `vcc`/`gnd` pins and
    /// every logic pin, W1c contract point 4's reader rule) — built once so
    /// `Self::resolve_dirty` can turn "this node's published bits changed"
    /// into "these readers must be queued" by a single lookup per node.
    node_to_ic_readers: crate::shared::Shared<[Vec<usize>]>,
    /// The order-independent state `Self::resolve_dirty` keeps so it costs
    /// what CHANGED rather than what the circuit holds. None of it is part
    /// of a snapshot: it is either derived from the immutable topology, a
    /// scratch buffer, or self-validating (see each field).
    ///
    /// partition index -> ids of the drivers whose own node is free in it
    /// (ascending), for the cache key. Derived from the driver table's
    /// `node` fields, which never change after a driver is registered (load
    /// time), so it is rebuilt only when the table's LENGTH differs from
    /// `driver_index_len`; a debug build re-derives and compares it on every
    /// call.
    part_drivers: crate::shared::Shared<[Vec<u32>]>,
    /// IC index (into `ics`) -> ids of every driver of that IC (`device`
    /// equal to its device index), ascending; derived with `part_drivers`.
    ic_drivers: crate::shared::Shared<[Vec<u32>]>,
    driver_index_len: usize,
    /// partition index -> indices into `ics` of every IC with a supply or
    /// logic pin free in it, in `ics` order: the only ICs `stamp_ics_into`'s
    /// `touches` test can ever accept for that partition (a driver's or a
    /// pull's node is one of the IC's logic pins), so the stamp visits these
    /// instead of every IC in the program.
    part_ics: crate::shared::Shared<[Vec<u32>]>,
    /// device index -> index into `ics`, `u32::MAX` for a device that is no IC.
    ic_of_device: crate::shared::Shared<[u32]>,
    /// partition index -> `(index into `ics`, logic-pin index)` of every
    /// would-be-pulled pin in it (an `OPEN` `FLOATING_INPUT`, non-`OPEN_HIGH`
    /// pin of a static IC record), in the key's fixed enumeration order.
    part_pulls: crate::shared::Shared<[Vec<(u32, u32)>]>,
    /// The packed cache key being built, reused across calls.
    key_scratch: Vec<u8>,
    /// Nodes whose published bits changed in the current `resolve_dirty`.
    changed_scratch: Vec<usize>,
    /// node -> ids of the stamps (resistors, then switches, as one list) with
    /// a pin on it, ascending, no repeats.
    stamps_by_node: crate::shared::Shared<[Vec<u32>]>,
    /// Stamps whose params `apply_params` touched since the last resolve.
    param_dirty: Vec<u32>,
    /// Stamps to recompute in the current call, and which are already listed.
    work: Vec<u32>,
    stamp_listed: Vec<bool>,
    /// One per `resolved` source: the terms of its child node's KCL sum.
    resolved_terms: crate::shared::Shared<[Vec<Term>]>,
    /// One per `kcl_junctions` entry.
    junction_memo: Vec<JunctionMemo>,
    /// `extra` of the KCL walk, node-indexed and all `0.0` between calls (the
    /// walk clears exactly the entries it touched).
    walk_extra: Vec<f64>,
    /// Whether the next `resolve_dirty` must recompute every current and
    /// check every value, as the whole-array version always did: set at
    /// `build`, by `restore` (the published arrays just jumped back to a
    /// snapshot) and by a failed resolve (a half-updated state).
    full: bool,
    /// The rollback window (`Self::begin_window`): while it is open, every
    /// published value a resolve changes is recorded first, as `(slot, old
    /// bits)`, so closing the window costs nothing and rolling it back puts
    /// exactly the changed slots back, where a whole-array snapshot copied
    /// both arrays at every instant. Only a value whose bits change is
    /// recorded: writing the bits it already holds changes nothing to undo.
    window: bool,
    v_log: Vec<(u32, u64)>,
    i_log: Vec<(u32, u64)>,
    /// Set by the owning `PartitionedSolver` when nothing reads this solver's
    /// source currents (it computes its own from the whole circuit) and it has
    /// proved every value they could be built from is bounded
    /// (`set_source_walk_skippable`); the walk then only runs when a value
    /// outside the bound could make it non-finite. Off for a solver used on
    /// its own, whose source currents are its output.
    source_walk_skippable: bool,
    /// Set before a call by the owner: every coefficient the source walk
    /// multiplies by is bounded and every published voltage is within
    /// `LAZY_V_LIMIT` as of the last publish. Together with the nodes this
    /// call changes, that bounds everything the walk reads.
    inputs_bounded: bool,
    /// How far from its stored rail a hub partition's cache entry may answer
    /// (Rails point 3): within the entry's own band, which the rail keeps in
    /// step with its regime (`set_hub_cache_rule`); `Flat(0)` for the
    /// re-solve at a CC exit, which must not be a first-order correction
    /// (Rails point 5). A wider hit would hand the rail loop back the solve
    /// it found past its band, and the re-solve would never happen.
    hub_cache_rule: rails::BandRule,
    /// Whether the last resolve skipped the source walk, for the test oracle.
    walk_skipped: bool,
    /// The readers of one resolve before they are sorted and deduplicated,
    /// reused across calls.
    readers_scratch: Vec<usize>,
    /// Test builds keep a whole copy of both arrays from when the window
    /// opened, and check on every window's end that undoing the log
    /// reproduces it: a slot written without being logged fails the test that
    /// ran it, not a rollback nobody happened to write.
    #[cfg(test)]
    w_saved: Option<(Vec<f64>, Vec<f64>)>,

    // ---- W1f: the live boundary and the hub rail --------------------------
    /// The rail node outside exact mode (README, Rails point 1).
    rail: Option<usize>,
    /// partition index -> its resistor terms on live boundary nodes, `(local,
    /// g, node)`, added at each stamp instead of baked into `rhs_base`.
    live_rhs: crate::shared::Shared<[Vec<(usize, f64, usize)>]>,
    /// partition index -> its hub state, `None` for a partition that does
    /// not read the rail.
    hubs: Vec<Option<Box<StaticHub>>>,
    /// partition index -> the waveform sources (by index in device order)
    /// whose level its cache key carries (Rails point 7, "Static cache").
    part_waves: crate::shared::Shared<[Vec<u32>]>,
    /// Each waveform source's level index, set by the owner at its edges.
    wave_levels: Vec<u8>,
    /// Every switch-family device's current params by device index, for the
    /// residual stamps (`stamp_linear_base` reads them by device).
    switch_params: HashMap<usize, Vec<f64>>,
    /// Boundary writes the owner queued for the next resolve, and whether it
    /// re-publishes every hub partition's correction at the present rail.
    pending_bnd: Vec<(usize, f64)>,
    republish_hubs: bool,
    /// How many of `changed_scratch`'s nodes the last resolve changed without
    /// solving (boundary writes and rail corrections): they come first, and
    /// queue no reader (Rails point 3: readers of hub partitions are queued
    /// by a rail scan).
    silent_len: usize,
    /// Contributions the last calls produced, for the owner to drain.
    hub_updates: Vec<(usize, Contrib)>,
    /// Contract stat 16, this solver's share.
    sensitivities: u64,
    /// The window's journal of hub states (first touch).
    hub_journal: Vec<(u32, Option<HubSolve>)>,
    hub_journaled: Vec<bool>,
    /// Exact mode's hub source, set by the owner before each call (README,
    /// Rails point 9); its partition bypasses the cache, whose key does not
    /// carry the source's state.
    exact: Option<crate::solve::ExactStamp>,

    // ---- W1g: slow stamp changes ([`Self::mark_device_changed`]) -----------
    /// node -> live boundary node, kept from the build so a partition's base
    /// can be rebuilt after a latch.
    live: Vec<bool>,
    /// device index -> its index in `resistors` (a resistor, fuse or PTC this
    /// solver owns), `u32::MAX` for any other device.
    stamp_of_device: crate::shared::Shared<[u32]>,
    /// partition index -> every stamped junction placed in it at load, with
    /// its device and channel, in the order `junctions` holds them: the list
    /// `junctions` is rebuilt from after a latch, a re-arm or a reset.
    junction_full: crate::shared::Shared<[Vec<led::Junction>]>,
    /// Per `kcl_junctions` entry: whether its device has not failed, and its
    /// law generation (the memo's key).
    kcl_active: Vec<bool>,
    kcl_law_gen: Vec<u32>,
    /// The slow state of every device whose stamp ever changed.
    stamp_states: HashMap<usize, StampState>,
    /// The caches a slow change cleared in the open window, as they were: a
    /// rollback puts them back, so the run goes on as if the instant had
    /// never happened (an entry is part of a junction partition's result).
    cache_journal: Vec<(u32, crate::shared::Shared<PartCache>)>,
    cache_journaled: Vec<bool>,

    // ---- F5: the power holds (W1c point 1a) -------------------------------
    /// The owner's device-indexed hold table, and whether any entry is not
    /// free. The owner journals it and hands it back after a rollback; the
    /// hold procedure in an iterated partition (`Self::hold_in`) changes
    /// this copy and queues the change for the owner in `hold_requests`.
    holds: std::rc::Rc<Vec<digital::power::PowerHold>>,
    holds_on: bool,
    /// Holds taken and released by the last resolves, for the owner to enter
    /// in its table (`PartitionedSolver::take_static_holds`).
    hold_requests: Vec<(usize, digital::power::PowerHold, f64)>,
    /// Exact mode's CC entry (ruling G, `PartitionedSolver::exact_cc_entry`):
    /// while set, a free-supply solve is a trial at a fixed rail, which runs
    /// no hold procedure and no release, so it takes and frees nothing.
    pub(crate) trial: bool,
    /// The ICs (device indices) whose power edge the exact rail's CC
    /// operating point landed on: the next free-supply solve of a partition
    /// gating them runs the hold procedure on them, as on ICs its Newton saw
    /// alternate (W1c point 1a), and clears the list.
    pub(crate) edge_hold: Vec<usize>,
    /// The copy as the open window found it before this solver first changed
    /// it, which a rollback restores.
    w_holds: Option<(std::rc::Rc<Vec<digital::power::PowerHold>>, bool, bool)>,
    /// Whether this host step's retry of the ICs held off is still to come
    /// (ruling R51 (b), `Self::release_in`): set at load and at each horizon
    /// clamp, the end of a host step, taken by the first retry after it.
    /// Journaled with the copy.
    retry_armed: bool,
}

impl StaticPartitionSolver {
    /// Each table this solver shares with snapshots, by address (W2a tests).
    #[cfg(test)]
    pub(crate) fn shared_tables(&self) -> Vec<(&'static str, usize)> {
        vec![("partitions", crate::shared::Shared::addr(&self.partitions)), ("local_index", crate::shared::Shared::addr(&self.local_index)), ("fixed", crate::shared::Shared::addr(&self.fixed)), ("resolved", crate::shared::Shared::addr(&self.resolved)), ("matrix_base", crate::shared::Shared::addr(&self.matrix_base)), ("rhs_base", crate::shared::Shared::addr(&self.rhs_base)), ("resistors", crate::shared::Shared::addr(&self.resistors)), ("switches", crate::shared::Shared::addr(&self.switches)), ("switch_index_by_device", crate::shared::Shared::addr(&self.switch_index_by_device)), ("ics", crate::shared::Shared::addr(&self.ics)), ("holds_free_ic_supply", crate::shared::Shared::addr(&self.holds_free_ic_supply)), ("junctions", crate::shared::Shared::addr(&self.junctions)), ("kcl_junctions", crate::shared::Shared::addr(&self.kcl_junctions)), ("node_to_ic_readers", crate::shared::Shared::addr(&self.node_to_ic_readers)), ("part_drivers", crate::shared::Shared::addr(&self.part_drivers)), ("ic_drivers", crate::shared::Shared::addr(&self.ic_drivers)), ("part_ics", crate::shared::Shared::addr(&self.part_ics)), ("ic_of_device", crate::shared::Shared::addr(&self.ic_of_device)), ("part_pulls", crate::shared::Shared::addr(&self.part_pulls)), ("stamps_by_node", crate::shared::Shared::addr(&self.stamps_by_node)), ("resolved_terms", crate::shared::Shared::addr(&self.resolved_terms)), ("live_rhs", crate::shared::Shared::addr(&self.live_rhs)), ("part_waves", crate::shared::Shared::addr(&self.part_waves)), ("stamp_of_device", crate::shared::Shared::addr(&self.stamp_of_device)), ("junction_full", crate::shared::Shared::addr(&self.junction_full))]
    }
}


impl StaticPartitionSolver {
    /// Builds the static topology (`partition::build`) and the pristine
    /// per-partition matrix/RHS, but does not solve anything yet — that is
    /// `Kernel::settle`'s job, through `resolve_dirty`, the same way any
    /// later re-solve works.
    pub fn build(program: &Program) -> Result<Self, BoError> {
        let pb = partition::build(program)?;
        let partition::PartitionBuild { partitions, local_index, fixed, resolved, kinds: partition_kinds, rail: rail_topology, live, hub, reads, .. } = pb;
        let partitions_len = partitions.len();
        let mut live_rhs: Vec<Vec<(usize, f64, usize)>> = vec![Vec::new(); partitions_len];

        let mut partition_junctions: Vec<Vec<DynJunction>> = vec![Vec::new(); partitions.len()];
        let mut junction_full: Vec<Vec<led::Junction>> = vec![Vec::new(); partitions.len()];
        let mut kcl_junctions = Vec::new();
        for j in partition::program_junctions(program).into_iter().filter(|j| j.stamped()) {
            match junction_partition(&j, &local_index) {
                Some(p) if partition_kinds[p] == PartitionKind::Static => {
                    partition_junctions[p].push(place_junction(&j, p, &local_index, &fixed).expect("junction_partition just named this partition"));
                    junction_full[p].push(j);
                    kcl_junctions.push(j);
                }
                Some(_) => {} // a dynamic partition's junction: that solver places and publishes it.
                None => kcl_junctions.push(j), // across two fixed nodes: no partition, but its current is real.
            }
        }

        let mut matrix_base: Vec<Vec<f64>> = partitions.iter().map(|p| vec![0.0; p.len() * p.len()]).collect();
        let mut rhs_base: Vec<Vec<f64>> = partitions.iter().map(|p| vec![0.0; p.len()]).collect();

        // Only a `Static` partition's own shunt row is ever meant to be
        // solved here — a non-`Static` partition's matrix stays all-zero
        // (never factored, since `PartitionedSolver` never names it in
        // `resolve_dirty`'s `dirty`), but is still allocated above so every
        // partition id in this vector lines up with `partition::build`'s
        // global numbering (`solve::mod`'s `PartitionedSolver` depends on
        // that alignment to route a dirty id to the right sub-solver).
        for (p_idx, nodes) in partitions.iter().enumerate() {
            if partition_kinds[p_idx] != PartitionKind::Static {
                continue;
            }
            let size = nodes.len();
            for l_idx in 0..size {
                matrix_base[p_idx][l_idx * size + l_idx] += NODE_RSHUNT_G;
            }
        }

        // Whether `device`'s own free pin(s) land in a `Static` partition —
        // `None` (no free pin at all, both terminals fixed) is also this
        // solver's to own, exactly as before W1b step 2. A device's free
        // pins are always in the SAME partition (`partition::build`'s
        // union-find), so the first free pin found settles it.
        let owned_by_static = |device: &crate::program::Device| -> bool {
            device
                .pins
                .iter()
                .find_map(|p| local_index[p.node].map(|(p_idx, _)| partition_kinds[p_idx]))
                .is_none_or(|k| k == PartitionKind::Static)
        };

        let mut resistors = Vec::new();
        let mut switches = Vec::new();
        let mut switch_index_by_device = HashMap::new();

        for (d, device) in program.devices.iter().enumerate() {
            match device.kind {
                // W1g: a fuse and a PTC are fixed two-terminal stamps too, folded
                // into the base with the resistors (Fuse and PTC records).
                kinds::RESISTOR | kinds::FUSE | kinds::PTC_FUSE if owned_by_static(device) => {
                    let pins: Vec<usize> = device.pins.iter().map(|p| p.node).collect();
                    let contacts = contacts_of(device.kind, &pins, &device.params);
                    for c in &contacts {
                        fold_static_stamp(&partitions, &local_index, &fixed, &live, &mut matrix_base, &mut rhs_base, &mut live_rhs, c.u, c.w, c.g);
                    }
                    resistors.push(StaticStamp { device_index: d, kind: device.kind, pins, params: device.params.clone(), contacts });
                }
                kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH if owned_by_static(device) => {
                    let pins: Vec<usize> = device.pins.iter().map(|p| p.node).collect();
                    switch_index_by_device.insert(d, switches.len());
                    let contacts = contacts_of(device.kind, &pins, &device.params);
                    switches.push(StaticStamp { device_index: d, kind: device.kind, pins, params: device.params.clone(), contacts });
                }
                _ => {}
            }
        }

        let mut voltages = vec![0.0; program.node_count];
        for (node, v) in fixed.iter().enumerate() {
            if let Some(v) = v {
                voltages[node] = *v;
            }
        }

        let ics = collect_ics(program);
        let mut holds_free_ic_supply = vec![false; partitions.len()];
        let mut node_to_ic_readers: Vec<Vec<usize>> = vec![Vec::new(); program.node_count];
        let mut part_pulls: Vec<Vec<(u32, u32)>> = vec![Vec::new(); partitions.len()];
        let mut part_ics: Vec<Vec<u32>> = vec![Vec::new(); partitions.len()];
        let mut ic_of_device = vec![u32::MAX; program.devices.len()];
        for (ic_index, (device_index, ic)) in ics.iter().enumerate() {
            ic_of_device[*device_index] = ic_index as u32;
            let mut touched: Vec<usize> = std::iter::once(ic.vcc.node).chain(std::iter::once(ic.gnd.node)).chain(ic.logic.iter().map(|pin| pin.node)).filter_map(|n| local_index[n].map(|(p, _)| p)).collect();
            touched.sort_unstable();
            touched.dedup();
            for p in touched {
                part_ics[p].push(ic_index as u32);
            }
            for &node in &[ic.vcc.node, ic.gnd.node] {
                node_to_ic_readers[node].push(*device_index);
                if let Some((p, _)) = local_index[node] {
                    holds_free_ic_supply[p] = true;
                }
            }
            for (pin_index, pin) in ic.logic.iter().enumerate() {
                node_to_ic_readers[pin.node].push(*device_index);
                if pin.open && pin.floating_input && !pin.open_high {
                    if let Some((p, _)) = local_index[pin.node] {
                        part_pulls[p].push((ic_index as u32, pin_index as u32));
                    }
                }
            }
        }

        // Which stamps a node's voltage feeds, so a resolve recomputes only
        // the devices on nodes whose bits moved.
        let mut stamps_by_node: Vec<Vec<u32>> = vec![Vec::new(); program.node_count];
        for (id, stamp) in resistors.iter().chain(switches.iter()).enumerate() {
            for &node in &stamp.pins {
                if stamps_by_node[node].last() != Some(&(id as u32)) {
                    stamps_by_node[node].push(id as u32);
                }
            }
        }

        // Each source child node's KCL terms, in the whole-circuit walk's
        // historical addition order (see [`Term`]).
        let resolved_terms: Vec<Vec<Term>> = resolved
            .iter()
            .map(|r| {
                let child = r.child_node;
                let mut terms = Vec::new();
                for (id, stamp) in resistors.iter().chain(switches.iter()).enumerate() {
                    for (ci, c) in stamp.contacts.iter().enumerate() {
                        if c.u == child {
                            terms.push(Term::Contact { stamp: id as u32, contact: ci as u32, plus: true });
                        }
                        if c.w == child {
                            terms.push(Term::Contact { stamp: id as u32, contact: ci as u32, plus: false });
                        }
                    }
                }
                for (ji, j) in kcl_junctions.iter().enumerate() {
                    if j.anode == child {
                        terms.push(Term::Junction { junction: ji as u32, plus: true });
                    }
                    if j.cathode == child {
                        terms.push(Term::Junction { junction: ji as u32, plus: false });
                    }
                }
                if child >= 1 {
                    terms.push(Term::Shunt);
                }
                terms
            })
            .collect();
        let junction_memo = vec![JunctionMemo::default(); kcl_junctions.len()];
        let kcl_count = kcl_junctions.len();
        let mut stamp_of_device = vec![u32::MAX; program.devices.len()];
        for (id, stamp) in resistors.iter().enumerate() {
            stamp_of_device[stamp.device_index] = id as u32;
        }
        let ics_len = ics.len();
        let stamp_count = resistors.len() + switches.len();

        // W1f: the hub partitions this solver owns, compiled for their
        // residual stamps, and the waveform sources each partition's key
        // carries.
        let rail = rail_topology.filter(|t| !t.exact).map(|t| t.rail);
        let mut hubs: Vec<Option<Box<StaticHub>>> = (0..partitions_len).map(|_| None).collect();
        if let Some(rail) = rail {
            let rail_ic = crate::solve::curvature::holds_rail_ic_pin(&ics, &local_index, rail, partitions_len);
            for p in 0..partitions_len {
                if !(hub[p] && partition_kinds[p] == PartitionKind::Static) {
                    continue;
                }
                let shape = compile_dyn_shape(p, &partitions[p], &local_index, &fixed, program);
                let node_owner: HashMap<usize, (usize, usize)> = partitions[p].iter().enumerate().map(|(l, &node)| (node, (p, l))).collect();
                let touch_ics: Vec<(usize, digital::IcRecord)> = part_ics[p].iter().map(|&k| ics[k as usize].clone()).collect();
                let nonlinear = holds_free_ic_supply[p] || !partition_junctions[p].is_empty() || rail_ic[p];
                hubs[p] = Some(Box::new(StaticHub { shape, node_owner, touch_ics, nonlinear, solve: None }));
            }
        }
        let mut wave_of_device = vec![u32::MAX; program.devices.len()];
        let mut wave_count = 0u32;
        for (d, device) in program.devices.iter().enumerate() {
            if partition::is_waveform(device.kind) {
                wave_of_device[d] = wave_count;
                wave_count += 1;
            }
        }
        let mut node_waves: Vec<Vec<u32>> = vec![Vec::new(); program.node_count];
        for r in &resolved {
            let mut waves = node_waves[r.parent_node].clone();
            if wave_of_device[r.device_index] != u32::MAX {
                waves.push(wave_of_device[r.device_index]);
            }
            node_waves[r.child_node] = waves;
        }
        let part_waves: Vec<Vec<u32>> = reads
            .iter()
            .map(|nodes| {
                let mut waves: Vec<u32> = nodes.iter().flat_map(|&n| node_waves[n].iter().copied()).collect();
                waves.sort_unstable();
                waves.dedup();
                waves
            })
            .collect();
        let switch_params: HashMap<usize, Vec<f64>> = switches.iter().map(|sw| (sw.device_index, sw.params.clone())).collect();

        Ok(StaticPartitionSolver {
            partitions: partitions.into(),
            local_index: local_index.into(),
            fixed: fixed.into(),
            resolved: resolved.into(),
            matrix_base: matrix_base.into(),
            rhs_base: rhs_base.into(),
            resistors: resistors.into(),
            switches: switches.into(),
            switch_index_by_device: switch_index_by_device.into(),
            voltages,
            currents: vec![0.0; program.devices.len()],
            no_solution_cache: program.options.no_solution_cache,
            epoch: 0,
            caches: (0..partitions_len).map(|_| crate::shared::Shared::default()).collect(),
            cache_hits: 0.0,
            ics: ics.into(),
            holds_free_ic_supply: holds_free_ic_supply.into(),
            junctions: partition_junctions.into(),
            kcl_junctions: kcl_junctions.into(),
            newton_iterations: 0,
            node_to_ic_readers: node_to_ic_readers.into(),
            part_drivers: (vec![Vec::new(); partitions_len]).into(),
            ic_drivers: (vec![Vec::new(); ics_len]).into(),
            driver_index_len: usize::MAX,
            part_ics: part_ics.into(),
            ic_of_device: ic_of_device.into(),
            part_pulls: part_pulls.into(),
            key_scratch: Vec::new(),
            changed_scratch: Vec::new(),
            stamps_by_node: stamps_by_node.into(),
            param_dirty: Vec::new(),
            work: Vec::with_capacity(stamp_count),
            stamp_listed: vec![false; stamp_count],
            resolved_terms: resolved_terms.into(),
            junction_memo,
            walk_extra: vec![0.0; program.node_count],
            full: true,
            window: false,
            v_log: Vec::new(),
            i_log: Vec::new(),
            source_walk_skippable: false,
            inputs_bounded: false,
            hub_cache_rule: rails::BandRule::Curvature { fallback: rails::RAIL_RESOLVE_V },
            walk_skipped: false,
            readers_scratch: Vec::new(),
            #[cfg(test)]
            w_saved: None,
            rail,
            live_rhs: live_rhs.into(),
            hubs,
            part_waves: part_waves.into(),
            wave_levels: (vec![0; wave_count as usize]).into(),
            switch_params,
            pending_bnd: Vec::new(),
            republish_hubs: false,
            silent_len: 0,
            hub_updates: Vec::new(),
            sensitivities: 0,
            hub_journal: Vec::new(),
            hub_journaled: vec![false; partitions_len],
            exact: None,
            live,
            stamp_of_device: stamp_of_device.into(),
            junction_full: junction_full.into(),
            kcl_active: vec![true; kcl_count],
            kcl_law_gen: vec![0; kcl_count],
            stamp_states: HashMap::new(),
            cache_journal: Vec::new(),
            cache_journaled: vec![false; partitions_len],
            holds: std::rc::Rc::new(Vec::new()),
            holds_on: false,
            hold_requests: Vec::new(),
            trial: false,
            edge_hold: Vec::new(),
            w_holds: None,
            retry_armed: true,
        })
    }

    /// Exact mode's hub source for the calls that follow.
    /// The owner's hold table ([`crate::solve::PartitionedSolver`]'s), after
    /// every change to it.
    pub(crate) fn set_power_holds(&mut self, holds: std::rc::Rc<Vec<digital::power::PowerHold>>, on: bool) {
        self.holds = holds;
        self.holds_on = on;
    }

    /// Arms the next host step's retry (the horizon clamp ended this one).
    pub(crate) fn arm_retry(&mut self) {
        if !self.retry_armed {
            // As the owner's (`PartitionedSolver::arm_retry`): observable only
            // through a clamp that fails after arming (F5V2 finding 6,
            // mutation 9).
            self.journal_local_holds();
            self.retry_armed = true;
        }
    }

    /// The iterated partitions holding an IC held off before `now`, while
    /// this host step's retry is still to come: the ones the horizon clamp
    /// re-solves to retry it.
    pub(crate) fn held_partitions(&self, now: f64) -> Vec<PartitionId> {
        let holds = self.holds_slice();
        if holds.is_empty() || !self.retry_armed {
            return Vec::new();
        }
        (0..self.partitions.len())
            .filter(|&p| {
                self.holds_free_ic_supply[p]
                    && self.gated_ics(p).iter().any(|&k| {
                        let hold = digital::power::hold_of(holds, self.ics[k as usize].0);
                        hold.kind == digital::power::HoldKind::Off && hold.release.is_some_and(|r| r.since < now)
                    })
            })
            .map(|p| PartitionId(p as u32))
            .collect()
    }

    /// The holds and releases this solver's resolves took since the last
    /// call, in order.
    #[inline]
    pub(crate) fn has_hold_requests(&self) -> bool {
        !self.hold_requests.is_empty()
    }

    pub(crate) fn take_hold_requests(&mut self) -> Vec<(usize, digital::power::PowerHold, f64)> {
        std::mem::take(&mut self.hold_requests)
    }

    /// The hold table as a verdict reads it: empty while nothing is held.
    #[inline]
    fn holds_slice(&self) -> &[digital::power::PowerHold] {
        if self.holds_on {
            &self.holds
        } else {
            &[]
        }
    }

    pub(crate) fn set_exact(&mut self, exact: Option<crate::solve::ExactStamp>) {
        self.exact = exact;
    }

    /// W1g's one entry point for a slow change to `device`'s stamp (README
    /// W1g point 12): a latch, a junction re-arm, a PTC trip or recovery, or a
    /// reset, given as the device's whole slow state. What `apply_params` does
    /// for a toggle, minus the value write:
    /// - a resistor, fuse or PTC takes its stamp params with the slow flag
    ///   (`devices::slow_flag`), and its partition's base is rebuilt from
    ///   scratch in the build's own order, so a reset gives back the loaded
    ///   base bit for bit;
    /// - an LED device's junctions are re-placed in each partition holding
    ///   one (a failed device's dropped, a held law applied), and its source
    ///   KCL terms follow, the memo keyed by the new law generation;
    /// - every partition it touches has its solution cache cleared (the key
    ///   carries no failure or law state), and `full` is set, since the
    ///   incremental currents would miss a change that moves a current
    ///   without moving a voltage (a device across two fixed nodes).
    ///
    /// Returns the partitions it touches, for the caller to mark dirty.
    /// Calling it again with the same state changes nothing but the caches,
    /// which is how a rollback puts the stamps back (`PartitionedSolver`).
    pub(crate) fn mark_device_changed(&mut self, device: usize, state: StampState) -> Vec<PartitionId> {
        self.set_device_stamp(device, state, true)
    }

    /// [`Self::mark_device_changed`]'s body. A rollback sets a device's stamp
    /// back with `clear_caches` off, since the window's journal has already
    /// put back the caches it found.
    pub(crate) fn set_device_stamp(&mut self, device: usize, state: StampState, clear_caches: bool) -> Vec<PartitionId> {
        if state == StampState::default() {
            self.stamp_states.remove(&device);
        } else {
            self.stamp_states.insert(device, state);
        }
        let mut touched: Vec<usize> = Vec::new();
        if let Some(&id) = self.stamp_of_device.get(device).filter(|&&id| id != u32::MAX) {
            let flag = state.failed || state.tripped;
            let stamp = &mut self.resistors[id as usize];
            stamp.params = devices::with_slow_flag(stamp.kind, &stamp.params, flag);
            stamp.contacts = contacts_of(stamp.kind, &stamp.pins, &stamp.params);
            if flag {
                self.switch_params.insert(device, stamp.params.clone());
            } else {
                self.switch_params.remove(&device);
            }
            self.param_dirty.push(id);
            for &node in &self.resistors[id as usize].pins {
                if let Some((p, _)) = self.local_index[node] {
                    if !touched.contains(&p) {
                        touched.push(p);
                    }
                }
            }
            for &p in &touched {
                self.rebuild_base(p);
            }
        }
        // The junctions: every partition that holds one of the device's.
        let junction_parts: Vec<usize> = (0..self.junction_full.len()).filter(|&p| self.junction_full[p].iter().any(|j| j.device == device)).collect();
        for p in junction_parts {
            self.rebuild_junctions(p);
            if !touched.contains(&p) {
                touched.push(p);
            }
        }
        for k in 0..self.kcl_junctions.len() {
            if self.kcl_junctions[k].device != device {
                continue;
            }
            self.kcl_active[k] = !state.failed;
            self.kcl_law_gen[k] = state.law_gen;
            if let Some(law) = state.law {
                self.kcl_junctions[k].law = law;
            }
        }
        if clear_caches {
            for &p in &touched {
                let old = std::mem::take(&mut self.caches[p]);
                if self.window && !self.cache_journaled[p] {
                    self.cache_journaled[p] = true;
                    self.cache_journal.push((p as u32, old));
                }
            }
        }
        self.full = true;
        touched.sort_unstable();
        touched.into_iter().map(|p| PartitionId(p as u32)).collect()
    }

    /// Partition `p`'s pristine base, folded afresh: the shunt on every row,
    /// then every resistor, fuse and PTC contact touching it, in the build's
    /// device order, so an unchanged set of stamps gives back the same bits.
    fn rebuild_base(&mut self, p: usize) {
        let size = self.partitions[p].len();
        self.matrix_base[p].iter_mut().for_each(|a| *a = 0.0);
        self.rhs_base[p].iter_mut().for_each(|b| *b = 0.0);
        self.live_rhs[p].clear();
        for l in 0..size {
            self.matrix_base[p][l * size + l] += NODE_RSHUNT_G;
        }
        let in_p = |node: usize, local_index: &[Option<(usize, usize)>]| matches!(local_index[node], Some((q, _)) if q == p);
        for stamp in &self.resistors {
            for c in &stamp.contacts {
                if in_p(c.u, &self.local_index) || in_p(c.w, &self.local_index) {
                    fold_static_stamp(&self.partitions, &self.local_index, &self.fixed, &self.live, &mut self.matrix_base, &mut self.rhs_base, &mut self.live_rhs, c.u, c.w, c.g);
                }
            }
        }
    }

    /// Partition `p`'s junction list (and a hub partition's residual shape's)
    /// from the junctions placed at load, in their load order: a failed
    /// device's dropped, a held law applied (README W1g points 4 and 12).
    fn rebuild_junctions(&mut self, p: usize) {
        let placed: Vec<DynJunction> = self.junction_full[p]
            .iter()
            .filter_map(|j| {
                let state = self.stamp_states.get(&j.device).copied().unwrap_or_default();
                if state.failed {
                    return None;
                }
                let mut placed = place_junction(j, p, &self.local_index, &self.fixed).expect("placed in this partition at load");
                if let Some(law) = state.law {
                    placed.law = law;
                }
                Some(placed)
            })
            .collect();
        if let Some(hub) = self.hubs[p].as_mut() {
            hub.shape.junctions = placed.clone();
        }
        self.junctions[p] = placed;
    }

    /// Every junction law this solver stamps or folds into the KCL, with the
    /// KCL's active flags and law generations (tests compare them).
    #[cfg(test)]
    pub(crate) fn junction_laws(&self) -> String {
        format!("{:?} {:?} {:?} {:?}", self.junctions, self.kcl_junctions, self.kcl_active, self.kcl_law_gen)
    }

    /// Each partition's cache, as its entries' solutions in bits (tests
    /// compare them).
    #[cfg(test)]
    pub(crate) fn cache_bits(&self) -> Vec<Vec<Vec<u64>>> {
        self.caches.iter().map(|c| c.entries.iter().map(|e| e.x.iter().map(|v| v.to_bits()).collect()).collect()).collect()
    }

    /// Forgets every junction memo: a rollback can restore a law generation
    /// a memo was computed under for a different law (README W1g point 4).
    pub(crate) fn clear_junction_memo(&mut self) {
        self.junction_memo.iter_mut().for_each(|m| m.valid = false);
    }


    fn apply_exact(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64]) {
        if let Some(e) = self.exact.filter(|e| e.partition == p_idx) {
            e.apply(size, a, b);
        }
    }

    /// Contract stat 11, the static half (`PartitionedSolver` adds the
    /// stepped one).
    pub fn newton_iterations(&self) -> f64 {
        self.newton_iterations as f64
    }

    /// Opens the rollback window: from here every published value a resolve
    /// changes is recorded, until [`Self::end_window`] or
    /// [`Self::rollback_window`].
    pub(crate) fn begin_window(&mut self) {
        self.window = true;
        self.v_log.clear();
        self.i_log.clear();
        self.hub_journal.clear();
        #[cfg(test)]
        {
            self.w_saved = Some((self.voltages.clone(), self.currents.clone()));
        }
    }

    /// Undoes the log on copies of the arrays, newest first, and checks that
    /// they come out as the arrays the window found (test builds only).
    #[cfg(test)]
    fn check_log_reproduces_the_window_start(&mut self) {
        let Some((v0, i0)) = self.w_saved.take() else { return };
        let (mut v, mut i) = (self.voltages.clone(), self.currents.clone());
        for &(node, bits) in self.v_log.iter().rev() {
            v[node as usize] = f64::from_bits(bits);
        }
        for &(device, bits) in self.i_log.iter().rev() {
            i[device as usize] = f64::from_bits(bits);
        }
        for (n, (a, b)) in v.iter().zip(&v0).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "static solver: voltage of node {n} changed in the window without being logged");
        }
        for (d, (a, b)) in i.iter().zip(&i0).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "static solver: current of device {d} changed in the window without being logged");
        }
    }

    /// The window's instant completed: what it recorded is dropped.
    pub(crate) fn end_window(&mut self) {
        #[cfg(test)]
        self.check_log_reproduces_the_window_start();
        self.window = false;
        self.w_holds = None;
        self.v_log.clear();
        self.i_log.clear();
        for (p, _) in self.hub_journal.drain(..) {
            self.hub_journaled[p as usize] = false;
        }
        // Almost always empty: only a slow stamp change journals a cache, and
        // the drain's drop glue is not free on every instant.
        if !self.cache_journal.is_empty() {
            for (p, _) in self.cache_journal.drain(..) {
                self.cache_journaled[p as usize] = false;
            }
        }
    }

    /// The window's instant failed: every slot it changed goes back to the
    /// bits it held when the window opened (the log is undone newest first, so
    /// a slot written twice ends at its first pre-image), and the next resolve
    /// recomputes and checks everything, as after [`PartitionSolver::restore`].
    pub(crate) fn rollback_window(&mut self) {
        for &(node, bits) in self.v_log.iter().rev() {
            self.voltages[node as usize] = f64::from_bits(bits);
        }
        for &(device, bits) in self.i_log.iter().rev() {
            self.currents[device as usize] = f64::from_bits(bits);
        }
        #[cfg(test)]
        if let Some((v0, i0)) = self.w_saved.take() {
            assert!(self.voltages.iter().zip(&v0).all(|(a, b)| a.to_bits() == b.to_bits()), "static solver: a rollback left a voltage changed");
            assert!(self.currents.iter().zip(&i0).all(|(a, b)| a.to_bits() == b.to_bits()), "static solver: a rollback left a current changed");
        }
        self.window = false;
        self.v_log.clear();
        self.i_log.clear();
        for (p, solve) in self.hub_journal.drain(..).rev() {
            self.hub_journaled[p as usize] = false;
            if let Some(hub) = self.hubs[p as usize].as_mut() {
                hub.solve = solve;
            }
        }
        for (p, cache) in self.cache_journal.drain(..).rev() {
            self.cache_journaled[p as usize] = false;
            self.caches[p as usize] = cache;
        }
        // A hold this solver took in the failed instant is undone with it.
        if let Some((holds, on, armed)) = self.w_holds.take() {
            self.holds = holds;
            self.holds_on = on;
            self.retry_armed = armed;
        }
        self.hold_requests.clear();
        // Owner requests are per call and never span a failed instant.
        self.pending_bnd.clear();
        self.republish_hubs = false;
        self.hub_updates.clear();
        self.full = true;
    }

    /// The nodes whose published bits moved in the last resolve (any that
    /// failed midway included), ascending order not guaranteed.
    pub(crate) fn changed_nodes(&self) -> &[usize] {
        &self.changed_scratch
    }

    /// The devices whose published current the last resolve recomputed
    /// (resistors and switches; every one after a `full` resolve). A
    /// current outside this list is exactly as the resolve before left it.
    pub(crate) fn recomputed_devices(&self) -> impl Iterator<Item = usize> + '_ {
        let resistor_count = self.resistors.len();
        self.work.iter().map(move |&id| {
            let stamp = if (id as usize) < resistor_count { &self.resistors[id as usize] } else { &self.switches[id as usize - resistor_count] };
            stamp.device_index
        })
    }

    /// Whether the last resolve skipped the source walk (tests look at it).
    #[cfg(test)]
    pub(crate) fn skipped_the_source_walk(&self) -> bool {
        self.walk_skipped
    }

    /// Whether the next resolve recomputes and checks everything (the field
    /// `full`; tests look at it).
    #[cfg(test)]
    pub(crate) fn needs_full_recompute(&self) -> bool {
        self.full
    }

    /// Every resistor and switch this solver publishes a current for.
    #[cfg(debug_assertions)]
    pub(crate) fn stamp_devices(&self) -> impl Iterator<Item = usize> + '_ {
        self.resistors.iter().chain(self.switches.iter()).map(|s| s.device_index)
    }

    /// See the field of the same name.
    pub(crate) fn set_source_walk_skippable(&mut self, on: bool) {
        self.source_walk_skippable = on;
    }

    /// See the field of the same name.
    pub(crate) fn set_inputs_bounded(&mut self, on: bool) {
        self.inputs_bounded = on;
    }

    /// See the field `hub_cache_rule`.
    pub(crate) fn set_hub_cache_rule(&mut self, rule: rails::BandRule) {
        self.hub_cache_rule = rule;
    }

    #[cfg(test)]
    pub(crate) fn hub_cache_rule(&self) -> rails::BandRule {
        self.hub_cache_rule
    }

    /// [`PartitionSolver::resolve_dirty`] appending its readers to `out`
    /// instead of returning a fresh vector.
    pub(crate) fn resolve_dirty_into(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver], out: &mut Vec<DeviceId>) -> Result<(), String> {
        self.ensure_driver_index(drivers);
        let mut changed = std::mem::take(&mut self.changed_scratch);
        changed.clear();
        let result = self.resolve_dirty_inner(dirty, now, drivers, &mut changed, out);
        if result.is_err() {
            // A half-updated state: the next resolve recomputes everything,
            // as the whole-array version always did (the kernel normally
            // restores a snapshot first, which does the same).
            self.full = true;
        }
        self.changed_scratch = changed;
        result
    }

    /// The number of partitions built (contract stat 0), for `Engine::load`
    /// to snapshot at load time.
    pub fn partition_count(&self) -> usize {
        self.partitions.len()
    }

    /// The largest partition's node count (contract stat 2).
    pub fn largest_partition(&self) -> usize {
        self.partitions.iter().map(|p| p.len()).max().unwrap_or(0)
    }

    /// How many nodes fixed-node propagation fixed, node 0 included
    /// (contract stat 3).
    pub fn fixed_count(&self) -> usize {
        self.fixed.iter().filter(|v| v.is_some()).count()
    }

    /// Derives `part_drivers` from `drivers` when the table's length is not
    /// the one it was derived for (the table only ever grows, at load, and a
    /// driver's `node` never changes once registered).
    fn ensure_driver_index(&mut self, drivers: &[Driver]) {
        if self.driver_index_len != drivers.len() {
            let (part_drivers, ic_drivers) = self.derive_driver_index(drivers);
            self.part_drivers = (part_drivers).into();
            self.ic_drivers = (ic_drivers).into();
            self.driver_index_len = drivers.len();
        }
        #[cfg(debug_assertions)]
        {
            let (part_drivers, ic_drivers) = self.derive_driver_index(drivers);
            debug_assert!(part_drivers == self.part_drivers && ic_drivers == self.ic_drivers, "the driver table changed without changing its length");
        }
    }

    /// `(partition -> its drivers, IC -> its drivers)` for `drivers`, both
    /// ascending by driver id.
    fn derive_driver_index(&self, drivers: &[Driver]) -> (Vec<Vec<u32>>, Vec<Vec<u32>>) {
        let mut part_drivers: Vec<Vec<u32>> = vec![Vec::new(); self.partitions.len()];
        let mut ic_drivers: Vec<Vec<u32>> = vec![Vec::new(); self.ics.len()];
        for (id, d) in drivers.iter().enumerate() {
            if let Some((p, _)) = self.local_index.get(d.node).copied().flatten() {
                part_drivers[p].push(id as u32);
            }
            if let Some(&ic_index) = self.ic_of_device.get(d.device) {
                if ic_index != u32::MAX {
                    ic_drivers[ic_index as usize].push(id as u32);
                }
            }
        }
        (part_drivers, ic_drivers)
    }

    /// Packs partition `p_idx`'s cache key (see [`PartCache`]) into
    /// `self.key_scratch`.
    fn build_key(&mut self, p_idx: usize, now: f64, drivers: &[Driver]) {
        let key = &mut self.key_scratch;
        key.clear();
        key.extend_from_slice(&[0; 4]);
        let mut committed = 0u32;
        for &id in &self.part_drivers[p_idx] {
            if let Some(mode) = drivers[id as usize].committed() {
                key.extend_from_slice(&id.to_le_bytes());
                key.push(mode_discriminant(mode));
                committed += 1;
            }
        }
        key[..4].copy_from_slice(&committed.to_le_bytes());
        // Fixed device order, then logic-pin order: the CONTENTS, not merely
        // the length, must match for two resolves to be the same stamp.
        for &(ic_index, pin_index) in &self.part_pulls[p_idx] {
            let (_, ic) = &self.ics[ic_index as usize];
            let pin = &ic.logic[pin_index as usize];
            key.push(digital::floating::floating_high(pin.open_hash, now, ic.ttl) as u8);
        }
        key.extend_from_slice(&self.epoch.to_le_bytes());
        // W1f: a hub partition's rail band and the level of every waveform
        // source it reads (README, Rails points 3 and 7). Empty for every
        // partition before W1f, so its key is unchanged.
        if let (Some(rail), Some(hub)) = (self.rail, &self.hubs[p_idx]) {
            let band = (self.voltages[rail] / rails::RAIL_BUCKET_V).round();
            key.extend_from_slice(&band.to_bits().to_le_bytes());
            // A power window can sit inside a band, and an unpowered IC
            // stamps nothing: the gate of every IC on the rail is part of
            // the stamp, as a committed mode is.
            for (device, ic) in &hub.touch_ics {
                if ic.vcc.node == rail || ic.gnd.node == rail {
                    key.push(digital::power::verdict(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node], digital::power::hold_of(if self.holds_on { &self.holds } else { &[] }, *device)) as u8);
                }
            }
        }
        for &w in &self.part_waves[p_idx] {
            key.push(self.wave_levels[w as usize]);
        }
        // F5: a held IC stamps nothing, so its hold is part of the stamp of
        // every partition it touches (a hub's `touch_ics` are these same
        // ICs); a powered entry must not answer for it. Only the ICs not
        // free are named (position and kind), so a key with no hold is the
        // key it always was.
        if self.holds_on {
            let holds: &[digital::power::PowerHold] = &self.holds;
            for &ic_index in &self.part_ics[p_idx] {
                let hold = digital::power::hold_of(holds, self.ics[ic_index as usize].0);
                if !hold.is_free() {
                    key.extend_from_slice(&ic_index.to_le_bytes());
                    key.push(hold.kind.tag());
                }
            }
        }
    }

    /// Whether `node` is a FREE node belonging to partition `p_idx`
    /// specifically (`false` for a fixed node, or a free node in some OTHER
    /// partition).
    fn in_partition(&self, node: usize, p_idx: usize) -> bool {
        matches!(self.local_index.get(node).copied().flatten(), Some((p, _)) if p == p_idx)
    }

    /// `node`'s current voltage for the purposes of resolving partition
    /// `p_idx`: the fixed-point iterate `x` when `node` is free and IN
    /// `p_idx` (`Self::iterated_solve`'s own local solution vector, valid for
    /// indices `0..partitions[p_idx].len()`), otherwise `self.fixed[node]`.
    /// Never reads `self.voltages` directly: every node an IC stamp inside
    /// `p_idx` can ever need (its own free pin, or its IC's supply nodes) is
    /// guaranteed by `partition.rs`'s union rule to be either fixed or free
    /// IN `p_idx` — never free in some OTHER partition — so `x` (which may
    /// legitimately be `&[]` when `p_idx` holds no free IC supply node, see
    /// `Self::stamp_ics_into`'s own doc comment) is always the right place
    /// to look for the free case.
    fn read_node(&self, node: usize, p_idx: usize, x: &[f64]) -> f64 {
        match self.local_index.get(node).copied().flatten() {
            Some((p, l)) if p == p_idx => x[l],
            _ => {
                debug_assert!(self.fixed[node].is_some(), "an IC pin's node reachable from a Static partition must be fixed or free within it (partition.rs's union rule / W1c step 2's -3 decline)");
                self.voltages[node]
            }
        }
    }

    /// Folds a two-terminal conductance `g` between global nodes `u` and `w`
    /// into partition `p_idx`'s LOCAL `size x size` scratch matrix `a`/`b` —
    /// the per-resolve counterpart of [`fold_static_stamp`] (which only ever
    /// runs once, at `build`, against the pristine GLOBAL `matrix_base`/
    /// `rhs_base`). Silently does nothing when NEITHER end is a free node in
    /// `p_idx` (both fixed, or free in some OTHER partition): every caller
    /// already guards its own "does this touch `p_idx` at all" question
    /// before calling in (`Self::stamp_switches_into`/`stamp_ics_into`'s own
    /// per-item skip), so that branch is a defensive no-op, never load-
    /// bearing.
    fn fold_local(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], u: usize, w: usize, g: f64) {
        if g == 0.0 {
            return;
        }
        match (self.local_index.get(u).copied().flatten(), self.local_index.get(w).copied().flatten()) {
            (Some((pu, lu_)), Some((pw, lw))) if pu == p_idx && pw == p_idx => {
                a[lu_ * size + lu_] += g;
                a[lw * size + lw] += g;
                a[lu_ * size + lw] -= g;
                a[lw * size + lu_] -= g;
            }
            (Some((pu, lu_)), _) if pu == p_idx => {
                a[lu_ * size + lu_] += g;
                debug_assert!(self.fixed[w].is_some(), "the far end of a within-partition stamp must be fixed when it is not itself free in this partition");
                b[lu_] += g * self.voltages[w];
            }
            (_, Some((pw, lw))) if pw == p_idx => {
                a[lw * size + lw] += g;
                debug_assert!(self.fixed[u].is_some(), "the far end of a within-partition stamp must be fixed when it is not itself free in this partition");
                b[lw] += g * self.voltages[u];
            }
            _ => {}
        }
    }

    /// Folds the supply slope of an IC driver branch from `u` into `w` into
    /// partition `p_idx`'s Newton matrix (W1c point 3): `k (vs - vs0)` more
    /// current, `k = G'(vs0) (v_u - v_w)` at the iterate, `vs` the IC's
    /// supply difference. Rows and columns not free here are skipped or go
    /// to the right-hand side at their fixed voltage, as in
    /// [`Self::fold_local`].
    #[allow(clippy::too_many_arguments)]
    fn fold_supply_slope(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], u: usize, w: usize, ic: &digital::IcRecord, k: f64, vs0: f64) {
        if k == 0.0 || !k.is_finite() {
            return;
        }
        for (row, sign) in [(u, 1.0), (w, -1.0)] {
            let Some((pr, lr)) = self.local_index.get(row).copied().flatten() else { continue };
            if pr != p_idx {
                continue;
            }
            let mut rhs = k * vs0;
            for (col, s) in [(ic.vcc.node, 1.0), (ic.gnd.node, -1.0)] {
                match self.local_index.get(col).copied().flatten() {
                    Some((pc, lc)) if pc == p_idx => a[lr * size + lc] += sign * s * k,
                    _ => rhs -= s * k * self.voltages[col],
                }
            }
            b[lr] += sign * rhs;
        }
    }

    /// Folds an open `FLOATING_INPUT` pin's Norton pull into partition
    /// `p_idx`'s local scratch matrix/rhs (`ic_pull_target`'s own doc
    /// comment: `g` from `pin_node` to the DATUM, plus the injection `g *
    /// target_v` into `pin_node`'s own row) — the pull's counterpart of
    /// [`Self::fold_local`], but simpler: unlike a driver's target, the
    /// datum is never itself a partition member with a KCL row of its own
    /// (node 0 is always fixed, `Self::fixed`'s own doc comment), so there is
    /// no "which end is free" dispatch to do, only the one free end
    /// `Self::stamp_ics_into` already confirmed with its own
    /// `Self::in_partition(pin.node, p_idx)` guard before ever calling in.
    /// Exactly classic's `mna.add(node, node, g); mna.addB(node, target *
    /// g)` (`_stampFloatingDigitalInputs`, sim-engine.ts:5202-5203).
    fn fold_pull(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], pin_node: usize, g: f64, target_v: f64) {
        if g == 0.0 {
            return;
        }
        let (p, l) = self
            .local_index
            .get(pin_node)
            .copied()
            .flatten()
            .expect("Self::stamp_ics_into already confirmed pin_node is free via Self::in_partition before calling fold_pull");
        debug_assert_eq!(p, p_idx, "Self::in_partition(pin_node, p_idx) already confirmed this");
        a[l * size + l] += g;
        b[l] += g * target_v;
    }

    /// Stamps every driver currently committed to HIGH or LOW whose OWN pin
    /// lives in partition `p_idx`, EXCEPT an IC driver (`Driver::device !=
    /// usize::MAX`) — `Self::stamp_ics_into` owns those, since an IC's
    /// conductance is not `driver.conductance` at all (the module's own
    /// pre-W1c behaviour, unchanged for whatever non-IC driver kind exists
    /// in the future).
    fn stamp_non_ic_drivers_into(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], drivers: &[Driver]) -> Result<(), String> {
        for driver in drivers {
            if driver.device != usize::MAX {
                continue;
            }
            let mode = match driver.committed() {
                Some(m) if m != Mode::Hiz => m,
                _ => continue,
            };
            let (driver_partition, l) = match self.local_index.get(driver.node).copied().flatten() {
                Some(loc) => loc,
                None => continue, // the driver's own node is fixed: nothing to stamp into any partition.
            };
            if driver_partition != p_idx {
                continue;
            }
            let f = match mode {
                Mode::High => driver.hi_level,
                Mode::Low => driver.lo_level,
                Mode::Hiz => unreachable!("filtered above"),
            };
            let live = |node: usize| self.fixed.get(node).copied().flatten().map(|_| self.voltages[node]);
            let hi_v = live(driver.hi_node);
            let lo_v = live(driver.lo_node);
            let (hi_v, lo_v) = match (hi_v, lo_v) {
                (Some(hv), Some(lv)) => (hv, lv),
                _ => {
                    return Err(format!(
                        "driver at node {}: a free hi/lo rail node is not supported by the static solver (hub rails are W1f)",
                        driver.node
                    ));
                }
            };
            if f > 1e-12 {
                let gg = driver.conductance * f;
                a[l * size + l] += gg;
                b[l] += gg * hi_v;
            }
            if f < 1.0 - 1e-12 {
                let gg = driver.conductance * (1.0 - f);
                a[l * size + l] += gg;
                b[l] += gg * lo_v;
            }
        }
        Ok(())
    }

    /// Stamps every switch-family device's CURRENT contact state (fresh
    /// every resolve, unlike the resistor's immutable base — see this
    /// module's doc comment). A switch's own free pins are always entirely
    /// within one partition (`partition::build` unions every contact
    /// unconditionally), so a contact whose free end lands outside `p_idx`
    /// belongs to a different partition entirely and is simply skipped by
    /// `Self::fold_local`.
    fn stamp_switches_into(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64]) {
        for sw in &self.switches {
            for c in &sw.contacts {
                if c.g == 0.0 {
                    continue; // README: "open stamps nothing".
                }
                self.fold_local(p_idx, size, a, b, c.u, c.w, c.g);
            }
        }
    }

    /// Stamps every IC driver/pull touching partition `p_idx` (W1c contract
    /// points 3/8): an IC is only even considered here when at least one of
    /// its driver/pull pins is free and IN `p_idx` — `partition.rs`'s union
    /// rule then guarantees its `vcc`/`gnd` are each either fixed or ALSO
    /// free in `p_idx`, never free in some other partition, so
    /// `Self::read_node` never has to fall through to a node this partition
    /// knows nothing about.
    ///
    /// `x` is the CURRENT iterate (`Self::iterated_solve`'s local solution
    /// vector) in an iterated solve, or `&[]` in a direct one:
    /// `Self::read_node` only ever indexes into it for a node
    /// free-in-`p_idx`, and when `p_idx` holds no free IC supply at all,
    /// every IC touching it has both supply nodes fixed by definition
    /// (`self.holds_free_ic_supply[p_idx]` is exactly that fact) — so `x`
    /// is never actually indexed in that case, and an empty slice is safe.
    ///
    /// `SLOPES` is a const parameter, not a runtime flag, so that every stamp
    /// but a second try's compiles to the loop without the slope terms and
    /// asks for no curve or pull target: as a flag, that code alone cost the
    /// exact-mode SAP-1 boards about 40 % more CPU, every second try aside
    /// (F5j: this function's own time 206 -> 668 ms in a profile).
    #[allow(clippy::too_many_arguments)]
    fn stamp_ics_into<const SLOPES: bool>(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], now: f64, drivers: &[Driver], x: &[f64]) {
        for &ic_index in &self.part_ics[p_idx] {
            let (_, ic) = &self.ics[ic_index as usize];
            let driver_ids = &self.ic_drivers[ic_index as usize];
            // `touches` also fires when this IC's own free supply node lives
            // in `p_idx`, even if every one of its driver/pull pins is fixed
            // elsewhere (findings 3/8): an output pin wired straight onto a
            // fixed/ideal node still loads its IC's free `vcc`/`gnd` through
            // `1/R_out`, and that conductance belongs in whichever partition
            // holds the free end, regardless of which end named this IC in
            // the first place.
            let touches = driver_ids.iter().any(|&id| self.in_partition(drivers[id as usize].node, p_idx))
                || ic.logic.iter().any(|p| p.open && p.floating_input && !p.open_high && self.in_partition(p.node, p_idx))
                || self.in_partition(ic.vcc.node, p_idx)
                || self.in_partition(ic.gnd.node, p_idx);
            if !touches {
                continue;
            }
            let v_vcc = self.read_node(ic.vcc.node, p_idx, x);
            let v_gnd = self.read_node(ic.gnd.node, p_idx, x);
            let hold = if self.holds_on { digital::power::hold_of(&self.holds, self.ics[ic_index as usize].0) } else { digital::power::PowerHold::FREE };
            let power = digital::power::ic_power_held(ic, v_vcc, v_gnd, hold);
            if !power.powered {
                continue;
            }
            // `SLOPES` (an iterated solve's second try, `Self::newton_in`):
            // with this IC's supply free here, its stamps move with the
            // iterate, and the Jacobian takes their slopes in the supply
            // too, as the hub polish's does
            // (`curvature::IcSupply::extra_jacobian`). A held-powered probe
            // can read a reversed supply, which the stamps clamp to 0 V:
            // flat there.
            let free_supply = SLOPES && v_vcc > v_gnd && (self.in_partition(ic.vcc.node, p_idx) || self.in_partition(ic.gnd.node, p_idx));
            for &id in driver_ids {
                let driver = &drivers[id as usize];
                // No `in_partition(driver.node, p_idx)` guard here (findings
                // 3/8): `Self::fold_local` already stamps correctly whether
                // the driver's own node or its commit target is the free end
                // in `p_idx` (or no-ops when NEITHER is), so pre-filtering on
                // the driver's own node only, as before, silently dropped
                // the "driver fixed, target free-in-`p_idx`" case entirely.
                // Every branch folds (W1a point 2): a 555's OUT stamps up to
                // two, to its own vcc and gnd nodes.
                for (slot, branch) in ic_driver_branches(&power, driver, ic).into_iter().enumerate() {
                    if let Some((target, g)) = branch {
                        self.fold_local(p_idx, size, a, b, driver.node, target, g);
                        if !free_supply {
                            continue;
                        }
                        if let Some(curve) = crate::solve::ic_driver_curves(driver, ic)[slot] {
                            let w = self.read_node(driver.node, p_idx, x) - self.read_node(target, p_idx, x);
                            self.fold_supply_slope(p_idx, size, a, b, driver.node, target, ic, curve.derivs(power.vs)[1] * w, power.vs);
                        }
                    }
                }
            }
            for pin in &ic.logic {
                if !self.in_partition(pin.node, p_idx) {
                    continue;
                }
                if let Some((g, target_v)) = ic_pull_target(ic, &power, pin, now) {
                    self.fold_pull(p_idx, size, a, b, pin.node, g, target_v);
                    // The pull's target is its supply pin's voltage: free
                    // here, it moves with the iterate (`extra_jacobian`'s
                    // pulls).
                    if !free_supply {
                        continue;
                    }
                    let target = if digital::floating::floating_high(pin.open_hash, now, ic.ttl) { ic.vcc.node } else { ic.gnd.node };
                    if let Some((pt, lt)) = self.local_index.get(target).copied().flatten() {
                        if pt == p_idx {
                            let (_, lp) = self.local_index[pin.node].expect("checked in the partition above");
                            a[lp * size + lt] -= g;
                            b[lp] -= g * x[lt];
                        }
                    }
                }
            }
        }
    }

    /// The ordinary, single-shot solve for a partition that holds no free IC
    /// supply node (`!self.holds_free_ic_supply[p_idx]`): every IC touching
    /// it has both supply nodes fixed, so its power (hence R_out, hence
    /// every driver/pull conductance) is constant for as long as the
    /// partition's topology is — computed once, not iterated (W1c contract
    /// point 3's last sentence: "When every supply node is fixed, the power
    /// is constant and the solve is direct.").
    fn direct_solve(&self, p_idx: usize, now: f64, drivers: &[Driver]) -> Result<Vec<f64>, String> {
        let size = self.partitions[p_idx].len();
        let mut a = self.matrix_base[p_idx].clone();
        let mut b = self.rhs_base[p_idx].clone();
        self.stamp_live_rhs(p_idx, &mut b);
        self.stamp_non_ic_drivers_into(p_idx, size, &mut a, &mut b, drivers)?;
        self.stamp_switches_into(p_idx, size, &mut a, &mut b);
        self.stamp_ics_into::<false>(p_idx, size, &mut a, &mut b, now, drivers, &[]);
        self.apply_exact(p_idx, size, &mut a, &mut b);
        lu::solve(a, size, b).map_err(|_| format!("partition {p_idx} ({size} node(s)): singular or non-finite pivot"))
    }

    /// [`Self::direct_solve`], keeping the factors (a hub partition's rail
    /// sensitivity solves through them). `lu::solve` is this factor and
    /// apply, so the solution is the same either way.
    fn direct_solve_factored(&self, p_idx: usize, now: f64, drivers: &[Driver]) -> Result<(Vec<f64>, lu::LuFactorization), String> {
        let size = self.partitions[p_idx].len();
        let mut a = self.matrix_base[p_idx].clone();
        let mut b = self.rhs_base[p_idx].clone();
        self.stamp_live_rhs(p_idx, &mut b);
        self.stamp_non_ic_drivers_into(p_idx, size, &mut a, &mut b, drivers)?;
        self.stamp_switches_into(p_idx, size, &mut a, &mut b);
        self.stamp_ics_into::<false>(p_idx, size, &mut a, &mut b, now, drivers, &[]);
        let fail = |_| format!("partition {p_idx} ({size} node(s)): singular or non-finite pivot");
        let factors = lu::lu_factor(a, size).map_err(fail)?;
        factors.solve(&mut b).map_err(fail)?;
        Ok((b, factors))
    }

    /// Adds every resistor term on a live boundary node at its present value.
    fn stamp_live_rhs(&self, p_idx: usize, b: &mut [f64]) {
        for &(l, g, node) in &self.live_rhs[p_idx] {
            b[l] += g * self.voltages[node];
        }
    }

    /// One iteration's stamp for an iterated partition (`Self::iterated_solve`):
    /// the pristine base, then every non-IC driver, switch contact, IC
    /// driver and pull (from the iterate's supply voltages when the partition
    /// holds a free supply, W1c point 3) and LED junction companion (about
    /// the iterate, W1e LEDs point 2). Classic restamps all of these from the
    /// latest iterate with one Newton loop; this is that loop restricted to
    /// one partition (LEDs points 9 and 11).
    #[allow(clippy::too_many_arguments)]
    fn stamp_iterate(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], now: f64, drivers: &[Driver], x: &[f64], slopes: bool) -> Result<(), String> {
        a.copy_from_slice(&self.matrix_base[p_idx]);
        b.copy_from_slice(&self.rhs_base[p_idx]);
        self.stamp_live_rhs(p_idx, b);
        self.stamp_non_ic_drivers_into(p_idx, size, a, b, drivers)?;
        self.stamp_switches_into(p_idx, size, a, b);
        if slopes {
            self.stamp_ics_into::<true>(p_idx, size, a, b, now, drivers, x);
        } else {
            self.stamp_ics_into::<false>(p_idx, size, a, b, now, drivers, x);
        }
        stamp_junctions(&self.junctions[p_idx], size, a, b, x, &Bnd::new(&self.voltages));
        self.apply_exact(p_idx, size, a, b);
        Ok(())
    }

    /// The solve for an iterated partition (a free IC supply node, W1c point
    /// 3, or a stamped LED junction, W1e LEDs point 6): [`newton::solve`],
    /// from this partition's last published voltages (0 V before its first
    /// solve, as classic's first solve starts from `netV = {gnd: 0}`), at
    /// most 50 iterations, and one polish iteration when it holds a junction.
    /// A partition that has not passed by then is `-4`. Returns the solved
    /// local voltages; the caller publishes them.
    fn iterated_solve(&mut self, p_idx: usize, now: f64, drivers: &[Driver]) -> Result<Vec<f64>, String> {
        self.iterated_solve_factored(p_idx, now, drivers).map(|(x, _)| x)
    }

    /// [`Self::iterated_solve`] with the factors of its last iteration. A
    /// partition holding a free IC supply counts each such IC's power
    /// verdict flips across the iterates: when the Newton fails with one
    /// flipped twice, the hold procedure runs here (W1c point 1a,
    /// `Self::hold_in`); when it passes with an IC held off, its release is
    /// checked at the solution (`Self::release_in`).
    fn iterated_solve_factored(&mut self, p_idx: usize, now: f64, drivers: &[Driver]) -> Result<(Vec<f64>, lu::LuFactorization), String> {
        if !self.holds_free_ic_supply[p_idx] {
            return self.newton_in(p_idx, now, drivers, &[], None);
        }
        self.free_supply_solve(p_idx, now, drivers, None)
    }

    /// The solve of a partition holding a free IC supply: the Newton and,
    /// for a hub partition (`polish_at`, its rail), the exact-Jacobian
    /// polish, with W1c point 1a around them.
    fn free_supply_solve(&mut self, p_idx: usize, now: f64, drivers: &[Driver], polish_at: Option<f64>) -> Result<(Vec<f64>, lu::LuFactorization), String> {
        let gated = self.gated_ics(p_idx);
        if self.trial {
            return self.solve_once(p_idx, now, drivers, &gated, None, polish_at);
        }
        let mut flips = crate::solve::hold::Flips::new(gated.len());
        let first = self.solve_once(p_idx, now, drivers, &gated, Some(&mut flips), polish_at);
        if !self.edge_hold.is_empty() {
            let a: Vec<usize> = (0..gated.len()).filter(|&k| self.edge_hold.contains(&self.ics[gated[k] as usize].0)).collect();
            if !a.is_empty() {
                self.edge_hold.clear();
                let err = first.err().unwrap_or_else(|| "rail: the CC operating point lands on a power edge".to_string());
                return self.hold_in(p_idx, now, drivers, &gated, &flips, &a, err, polish_at);
            }
        }
        match first {
            Ok(solved) if self.holds_on => self.release_in(p_idx, now, drivers, &gated, solved, polish_at),
            Ok(solved) => Ok(solved),
            Err(err) => {
                let a = flips.alternating();
                if a.is_empty() {
                    return Err(err);
                }
                self.hold_in(p_idx, now, drivers, &gated, &flips, &a, err, polish_at)
            }
        }
    }

    /// One solve of a free-supply partition: the Newton, then the polish at
    /// `polish_at` when given, each noting `gated`'s verdicts in `flips`.
    fn solve_once(&mut self, p_idx: usize, now: f64, drivers: &[Driver], gated: &[u32], mut flips: Option<&mut crate::solve::hold::Flips>, polish_at: Option<f64>) -> Result<(Vec<f64>, lu::LuFactorization), String> {
        let (x, factors) = self.newton_in(p_idx, now, drivers, gated, flips.as_deref_mut())?;
        match polish_at {
            Some(u) => Ok((self.polish_hub(p_idx, now, drivers, x, u, gated, flips)?, factors)),
            None => Ok((x, factors)),
        }
    }

    /// The ICs (indices into `ics`) with a supply node free in `p_idx`: the
    /// ones whose power the partition's own solve decides.
    fn gated_ics(&self, p_idx: usize) -> Vec<u32> {
        self.part_ics[p_idx].iter().copied().filter(|&k| {
            let ic = &self.ics[k as usize].1;
            self.in_partition(ic.vcc.node, p_idx) || self.in_partition(ic.gnd.node, p_idx)
        }).collect()
    }

    /// The supply difference of IC `k` (index into `ics`) at `x`.
    fn supply_at(&self, k: u32, p_idx: usize, x: &[f64]) -> f64 {
        let ic = &self.ics[k as usize].1;
        self.read_node(ic.vcc.node, p_idx, x) - self.read_node(ic.gnd.node, p_idx, x)
    }

    /// Sets this solver's copy of the hold of IC `device` (device index),
    /// saving the copy the window found first.
    fn set_local_hold(&mut self, device: usize, hold: digital::power::PowerHold) {
        self.journal_local_holds();
        let table = std::rc::Rc::make_mut(&mut self.holds);
        if table.len() <= device {
            table.resize(device + 1, digital::power::PowerHold::FREE);
        }
        table[device] = hold;
        self.holds_on = table.iter().any(|h| !h.is_free());
    }

    /// Saves the copy and the retry's arming as the window found them, at
    /// their first change in it.
    fn journal_local_holds(&mut self) {
        if self.window && self.w_holds.is_none() {
            self.w_holds = Some((std::rc::Rc::clone(&self.holds), self.holds_on, self.retry_armed));
        }
    }

    /// W1c point 1a in an iterated partition, after its Newton failed with
    /// the gated ICs at positions `a` of `gated` alternating: probed held
    /// powered (the rest at their last iterate's reading), the ones reading
    /// unpowered probed again held unpowered, and the partition solved with
    /// them held off and the rest as they were. Any failing step is `err`,
    /// with this solver's copy as it was.
    #[allow(clippy::too_many_arguments)]
    fn hold_in(&mut self, p_idx: usize, now: f64, drivers: &[Driver], gated: &[u32], flips: &crate::solve::hold::Flips, a: &[usize], err: String, polish_at: Option<f64>) -> Result<(Vec<f64>, lu::LuFactorization), String> {
        use crate::solve::hold;
        let saved = (std::rc::Rc::clone(&self.holds), self.holds_on);
        let device = |k: usize| self.ics[gated[k] as usize].0;
        let devices: Vec<usize> = (0..gated.len()).map(device).collect();
        let before: Vec<digital::power::PowerHold> = devices.iter().map(|&d| digital::power::hold_of(self.holds_slice(), d)).collect();
        let fail = |this: &mut Self| {
            (this.holds, this.holds_on) = (std::rc::Rc::clone(&saved.0), saved.1);
            Err(err.clone())
        };
        for (k, &d) in devices.iter().enumerate() {
            // One already held off stays so: a probe is no release.
            if before[k].kind == digital::power::HoldKind::Off && !a.contains(&k) {
                continue;
            }
            self.set_local_hold(d, hold::probe(a.contains(&k) || flips.last(k).unwrap_or(false)));
        }
        let Ok((x, _)) = self.solve_once(p_idx, now, drivers, gated, None, polish_at) else { return fail(self) };
        let starving: Vec<(usize, f64)> = a
            .iter()
            .filter_map(|&k| {
                let ic = &self.ics[gated[k] as usize].1;
                let (vv, vg) = (self.read_node(ic.vcc.node, p_idx, &x), self.read_node(ic.gnd.node, p_idx, &x));
                (!digital::power::powered(ic, vv, vg)).then_some((k, vv - vg))
            })
            .collect();
        if starving.is_empty() {
            return fail(self);
        }
        for &(k, _) in &starving {
            self.set_local_hold(devices[k], hold::probe(false));
        }
        let Ok((x, _)) = self.solve_once(p_idx, now, drivers, gated, None, polish_at) else { return fail(self) };
        let mut taken = Vec::new();
        for (k, &d) in devices.iter().enumerate() {
            let held = match starving.iter().find(|&&(s, _)| s == k) {
                Some(&(_, v_on)) => hold::held_off(&self.ics[gated[k] as usize].1, v_on, self.supply_at(gated[k], p_idx, &x), now),
                None => before[k],
            };
            self.set_local_hold(d, held);
            if held != before[k] {
                taken.push((d, held, now));
            }
        }
        match self.solve_once(p_idx, now, drivers, gated, None, polish_at) {
            Ok(solved) => {
                self.hold_requests.extend(taken);
                Ok(solved)
            }
            Err(_) => fail(self),
        }
    }

    /// Whether the horizon clamp's retry (ruling R51 (b)) of partition
    /// `p_idx` would change anything: a held IC freed (it carries itself
    /// probed powered, or reads unpowered both ways) or given a further
    /// level, judged as [`Self::release_in`] judges them, from the
    /// partition's published solution. The clamp re-solves only such a
    /// partition, so a retry that frees nothing leaves the partition's
    /// published state as it was, bit for bit (F5V2 finding 5: the forced
    /// re-solve itself moved it by ulps). The probe writes nothing but this
    /// solver's Newton count.
    pub(crate) fn clamp_retry_changes(&mut self, p_idx: usize, now: f64, drivers: &[Driver]) -> bool {
        let gated = self.gated_ics(p_idx);
        let x: Vec<f64> = self.partitions[p_idx].iter().map(|&n| self.voltages[n]).collect();
        let held: Vec<(u32, usize)> = gated
            .iter()
            .map(|&k| (k, self.ics[k as usize].0))
            .filter(|&(_, d)| {
                let hold = digital::power::hold_of(self.holds_slice(), d);
                hold.kind == digital::power::HoldKind::Off && hold.release.is_some_and(|r| r.since < now)
            })
            .collect();
        if held.is_empty() {
            return false;
        }
        let polish_at = self.hubs[p_idx].as_ref().and(self.rail).map(|rail| self.voltages[rail]);
        let saved = (std::rc::Rc::clone(&self.holds), self.holds_on);
        for &(_, d) in &held {
            self.set_local_hold(d, crate::solve::hold::probe(true));
        }
        let probed = self.solve_once(p_idx, now, drivers, &gated, None, polish_at);
        (self.holds, self.holds_on) = saved;
        // A probe that fails asks nothing of the re-solve either, which
        // keeps `solved` there too.
        let Ok((xp, _)) = probed else { return false };
        #[cfg(test)]
        if crate::solve::hold::RetrySeam::get() == crate::solve::hold::RetrySeam::AsksOnly {
            return false;
        }
        held.iter().any(|&(k, d)| {
            let ic = &self.ics[k as usize].1;
            let on = |v: &[f64]| digital::power::powered(ic, self.read_node(ic.vcc.node, p_idx, v), self.read_node(ic.gnd.node, p_idx, v));
            let (s_on, s_off) = (self.supply_at(k, p_idx, &xp), self.supply_at(k, p_idx, &x));
            let hold = digital::power::hold_of(self.holds_slice(), d);
            let refresh = !digital::power::powered(ic, s_on, 0.0) && digital::power::powered(ic, s_off, 0.0) && hold.release.is_some_and(|r| crate::solve::hold::passed(r, s_off));
            on(&xp) || !on(&x) || refresh
        })
    }

    /// The retry at an iterated partition's solve (W1c point 1a, ruling R51
    /// (b)): every IC held off is probed powered, the rest as they were, as
    /// classic's H8 retries a held chip at its next host step: once per
    /// host step (`retry_armed`), at the first such solve or else at the
    /// horizon clamp, and at any other only one whose supply, held off,
    /// passed its release level. One that
    /// carries itself there, or whose supply reads unpowered both probed
    /// powered and held off (its own draw no longer decides), is free
    /// again, and the partition solves once more with it, holding it again
    /// (with a fresh drop) if it still starves itself. The others stay held
    /// and `solved` stands: the probe only asked.
    fn release_in(&mut self, p_idx: usize, now: f64, drivers: &[Driver], gated: &[u32], solved: (Vec<f64>, lu::LuFactorization), polish_at: Option<f64>) -> Result<(Vec<f64>, lu::LuFactorization), String> {
        let held: Vec<(u32, usize)> = gated
            .iter()
            .map(|&k| (k, self.ics[k as usize].0))
            .filter(|&(k, d)| {
                // Not at the instant it was held: the next retry comes at a
                // later one (the rail loop's passes re-solve this partition
                // within the instant; retried in them, cold fuzz 240 tpl 4
                // was freed and held again at each pass of its load).
                let hold = digital::power::hold_of(self.holds_slice(), d);
                hold.kind == digital::power::HoldKind::Off
                    && hold.release.is_some_and(|r| r.since < now && (self.retry_armed || crate::solve::hold::passed(r, self.supply_at(k, p_idx, &solved.0))))
            })
            .collect();
        if held.is_empty() {
            return Ok(solved);
        }
        #[cfg(test)]
        if crate::solve::hold::RetrySeam::get() == crate::solve::hold::RetrySeam::NoRetry {
            return Ok(solved);
        }
        if self.retry_armed {
            self.journal_local_holds();
            self.retry_armed = false;
        }
        let saved = (std::rc::Rc::clone(&self.holds), self.holds_on);
        for &(_, d) in &held {
            self.set_local_hold(d, crate::solve::hold::probe(true));
        }
        let probed = self.solve_once(p_idx, now, drivers, gated, None, polish_at);
        (self.holds, self.holds_on) = saved;
        let Ok((x, factors)) = probed else { return Ok(solved) };
        #[cfg(test)]
        if crate::solve::hold::RetrySeam::get() == crate::solve::hold::RetrySeam::AsksOnly {
            return Ok(solved);
        }
        // One still starving though its supply passed its release level is
        // held again with the fresh drop (D1 (a)), as on the rail.
        for &(k, d) in &held {
            let ic = &self.ics[k as usize].1;
            let (s_on, s_off) = (self.supply_at(k, p_idx, &x), self.supply_at(k, p_idx, &solved.0));
            let hold = digital::power::hold_of(self.holds_slice(), d);
            if !digital::power::powered(ic, s_on, 0.0) && digital::power::powered(ic, s_off, 0.0) && hold.release.is_some_and(|r| crate::solve::hold::passed(r, s_off)) {
                let fresh = crate::solve::hold::refreshed(ic, hold.release.expect("checked above"), s_on, s_off, now);
                self.set_local_hold(d, fresh);
                self.hold_requests.push((d, fresh, now));
            }
        }
        let due: Vec<usize> = held
            .iter()
            .filter(|&&(k, _)| {
                let ic = &self.ics[k as usize].1;
                let on = |v: &[f64]| digital::power::powered(ic, self.read_node(ic.vcc.node, p_idx, v), self.read_node(ic.gnd.node, p_idx, v));
                on(&x) || !on(&solved.0)
            })
            .map(|&(_, d)| d)
            .collect();
        if due.is_empty() {
            return Ok(solved);
        }
        // Every held IC carries itself in the probe: its verdict there is the
        // one it was probed at, so the probe solves the free partition, and
        // is its solution (as ruling R45 (a) accepts a consistent probe on
        // the rail). A fresh Newton from the held state could 2-cycle across
        // the edge it no longer needs to cross.
        let all_carried = due.len() == held.len()
            && held.iter().all(|&(k, _)| {
                let ic = &self.ics[k as usize].1;
                digital::power::powered(ic, self.read_node(ic.vcc.node, p_idx, &x), self.read_node(ic.gnd.node, p_idx, &x))
            });
        for d in due {
            self.set_local_hold(d, digital::power::PowerHold::FREE);
            self.hold_requests.push((d, digital::power::PowerHold::FREE, now));
        }
        if all_carried {
            return Ok((x, factors));
        }
        let mut flips = crate::solve::hold::Flips::new(gated.len());
        match self.solve_once(p_idx, now, drivers, gated, Some(&mut flips), polish_at) {
            Ok(solved) => Ok(solved),
            Err(err) => {
                let a = flips.alternating();
                if a.is_empty() {
                    return Err(err);
                }
                self.hold_in(p_idx, now, drivers, gated, &flips, &a, err, polish_at)
            }
        }
    }

    /// One Newton solve of an iterated partition from its published
    /// voltages, noting each of `gated`'s verdicts at every iterate in
    /// `flips` when given.
    fn newton_in(&mut self, p_idx: usize, now: f64, drivers: &[Driver], gated: &[u32], mut flips: Option<&mut crate::solve::hold::Flips>) -> Result<(Vec<f64>, lu::LuFactorization), String> {
        let size = self.partitions[p_idx].len();
        let x0: Vec<f64> = self.partitions[p_idx].iter().map(|&n| self.voltages[n]).collect();
        let polish = !self.junctions[p_idx].is_empty();
        // Classic's loop restamps each IC stamp at the iterate's supply, its
        // slope in that supply left out. Where that does not converge, the
        // solve runs once more from the same start with the slopes in the
        // Jacobian (`Self::stamp_ics_into`'s `SLOPES`): frozen, a 555's OUT
        // at a low supply contracted the iteration by only 0.89 a step, and
        // another 2-cycled across the power gate (cold fuzz 1120 tpl 2,
        // 1325 tpl 0). Not at the first try: the full step can overshoot a
        // solution just inside the power window out of it, where the frozen
        // one creeps up to it (FXBv's fed gates). The power hold reads the
        // first try's verdicts only, as before.
        let slopes_too = self.holds_free_ic_supply[p_idx];
        let mut first_err: Option<String> = None;
        for slopes in [false, true] {
            if slopes && !slopes_too {
                break;
            }
            // A closure cannot return the stamp's own error (`Newton` wants
            // `()`), so it parks the first one here and leaves the matrix
            // zero, which fails the solve at once instead of iterating on
            // garbage.
            let mut stamp_error: Option<String> = None;
            let mut noted = if slopes { None } else { flips.as_deref_mut() };
            let result = newton::solve_keeping_factors(size, size, &x0, false, polish, |x, a, b| {
                if stamp_error.is_some() {
                    return;
                }
                if let Some(flips) = noted.as_deref_mut() {
                    for (k, &ic_index) in gated.iter().enumerate() {
                        let (device, ic) = &self.ics[ic_index as usize];
                        let hold = digital::power::hold_of(self.holds_slice(), *device);
                        flips.note(k, digital::power::verdict(ic, self.read_node(ic.vcc.node, p_idx, x), self.read_node(ic.gnd.node, p_idx, x), hold));
                    }
                }
                if let Err(e) = self.stamp_iterate(p_idx, size, a, b, now, drivers, x, slopes) {
                    a.fill(0.0);
                    b.fill(0.0);
                    stamp_error = Some(e);
                }
            });
            if let Some(e) = stamp_error {
                return Err(first_err.unwrap_or(e));
            }
            let (result, factors) = match result {
                Ok(r) => r,
                Err(e) => return Err(first_err.unwrap_or_else(|| format!("partition {p_idx} ({size} node(s)): {e}"))),
            };
            // Counted before the verdict, like every stat: a failed attempt ran.
            self.newton_iterations += result.iterations as u64;
            if result.converged {
                return Ok((result.x, factors));
            }
            first_err.get_or_insert_with(|| format!("partition {p_idx} ({size} node(s)): Newton did not converge within {} iterations", newton::NEWTON_MAX));
        }
        Err(first_err.expect("the first try failed"))
    }

    // ---- W1f: the live boundary and the hub rail ------------------------

    /// Queues a boundary node's new value for the next resolve, which writes
    /// it before it solves anything (README, Rails point 7).
    pub(crate) fn set_boundary(&mut self, node: usize, value: f64) {
        self.pending_bnd.push((node, value));
    }

    /// Asks the next resolve to re-publish every hub partition at the
    /// present rail, `x + s (u - u_p)` (Rails point 3).
    pub(crate) fn request_hub_republish(&mut self) {
        self.republish_hubs = true;
    }

    /// Sets a waveform source's level index (its cache key, Rails point 7).
    pub(crate) fn set_wave_level(&mut self, wave: usize, level: u8) {
        if let Some(slot) = self.wave_levels.get_mut(wave) {
            *slot = level;
        }
    }

    /// The contributions solved since the last drain.
    pub(crate) fn drain_hub_updates(&mut self) -> std::vec::Drain<'_, (usize, Contrib)> {
        self.hub_updates.drain(..)
    }

    pub(crate) fn sensitivities(&self) -> u64 {
        self.sensitivities
    }

    /// Hub partition `p`'s last solve: `(x, s, i, g, u_p)` (tests read it).
    #[cfg(test)]
    pub(crate) fn hub_solve(&self, p: usize) -> Option<(Vec<f64>, Vec<f64>, f64, f64, f64)> {
        let h = self.hubs.get(p)?.as_ref()?.solve.as_ref()?;
        Some((h.x.clone(), h.s.clone(), h.i, h.g, h.u_p))
    }

    /// Hub partition `p`'s curvature band at its last solve (tests read it).
    #[cfg(test)]
    pub(crate) fn hub_band(&self, p: usize) -> Option<f64> {
        self.hubs.get(p)?.as_ref()?.solve.as_ref()?.band
    }

    /// The curved hub partitions whose last solve is more than its band
    /// under `rule` from `u` (Rails point 3).
    pub(crate) fn hubs_past_band(&self, u: f64, rule: rails::BandRule, out: &mut Vec<PartitionId>) {
        for (p, hub) in self.hubs.iter().enumerate() {
            if let Some(hub) = hub {
                if hub.nonlinear && hub.solve.as_ref().is_some_and(|s| (u - s.u_p).abs() > rule.band(s.band)) {
                    out.push(PartitionId(p as u32));
                }
            }
        }
    }

    /// A node of a static hub partition that has solved, as the line it
    /// publishes on the rail (Rails point 3): `x + s (u - u_p)` as `(x - s
    /// u_p, s)`.
    pub(crate) fn hub_node_line(&self, node: usize) -> Option<(f64, f64)> {
        let (p, l) = self.local_index.get(node).copied().flatten()?;
        let h = self.hubs.get(p)?.as_ref()?.solve.as_ref()?;
        Some((h.x[l] - h.s[l] * h.u_p, h.s[l]))
    }

    /// [`Self::hub_node_line`] for a linear hub partition only: its line is
    /// then exact for any rail (W1g ruling R30).
    pub(crate) fn hub_node_line_exact(&self, node: usize) -> Option<(f64, f64)> {
        let (p, l) = self.local_index.get(node).copied().flatten()?;
        let hub = self.hubs.get(p)?.as_ref()?;
        if hub.nonlinear {
            return None;
        }
        let h = hub.solve.as_ref()?;
        Some((h.x[l] - h.s[l] * h.u_p, h.s[l]))
    }

    /// Every hub partition that has solved (a CC exit solves them all again,
    /// Rails point 5).
    pub(crate) fn hub_partitions(&self, out: &mut Vec<PartitionId>) {
        for (p, hub) in self.hubs.iter().enumerate() {
            if hub.as_ref().is_some_and(|h| h.solve.is_some()) {
                out.push(PartitionId(p as u32));
            }
        }
    }

    /// The `u_p` of every curved hub partition and its band under `rule`
    /// (the CC rail capacitor's re-solve thresholds, Rails point 4).
    pub(crate) fn nonlinear_hub_rails(&self, rule: rails::BandRule, out: &mut Vec<(f64, f64)>) {
        out.extend(self.hubs.iter().flatten().filter(|h| h.nonlinear).filter_map(|h| h.solve.as_ref().map(|s| (s.u_p, rule.band(s.band)))));
    }

    /// Every hub partition's solve, for a whole snapshot.
    pub(crate) fn hub_snapshot(&self) -> Vec<Option<HubSolveSnapshot>> {
        self.hubs.iter().map(|h| h.as_ref().and_then(|h| h.solve.clone()).map(HubSolveSnapshot)).collect()
    }

    pub(crate) fn hub_restore(&mut self, snap: Vec<Option<HubSolveSnapshot>>) {
        for (hub, saved) in self.hubs.iter_mut().zip(snap) {
            if let Some(hub) = hub {
                hub.solve = saved.map(|s| s.0);
            }
        }
    }

    /// Writes the queued boundary values and, when asked, every hub
    /// partition's correction, noting each node whose bits moved.
    fn apply_pending_boundary(&mut self, changed: &mut Vec<usize>) {
        for k in 0..self.pending_bnd.len() {
            let (node, value) = self.pending_bnd[k];
            self.write_voltage(node, value, changed);
        }
        self.pending_bnd.clear();
        if std::mem::take(&mut self.republish_hubs) {
            for p in 0..self.hubs.len() {
                self.publish_hub(p, changed);
            }
        }
    }

    fn write_voltage(&mut self, node: usize, value: f64, changed: &mut Vec<usize>) {
        if value.to_bits() != self.voltages[node].to_bits() {
            changed.push(node);
            if self.window {
                self.v_log.push((node as u32, self.voltages[node].to_bits()));
            }
            self.voltages[node] = value;
        }
    }

    /// Publishes hub partition `p` at the present rail: `x + s (u - u_p)`.
    fn publish_hub(&mut self, p: usize, changed: &mut Vec<usize>) {
        let (Some(rail), Some(hub)) = (self.rail, self.hubs[p].as_ref()) else { return };
        let Some(solve) = hub.solve.as_ref() else { return };
        let du = self.voltages[rail] - solve.u_p;
        let values: Vec<f64> = solve.x.iter().zip(&solve.s).map(|(x, s)| x + s * du).collect();
        for (l, value) in values.into_iter().enumerate() {
            let node = self.partitions[p][l];
            self.write_voltage(node, value, changed);
        }
    }

    /// Replaces hub partition `p`'s solve (journaled), publishes it and hands
    /// the rail its contribution.
    fn set_hub_solve(&mut self, p: usize, solve: HubSolve, changed: &mut Vec<usize>) {
        if self.window && !self.hub_journaled[p] {
            self.hub_journaled[p] = true;
            let old = self.hubs[p].as_ref().and_then(|h| h.solve.clone());
            self.hub_journal.push((p as u32, old));
        }
        let contrib = Contrib { active: true, i: solve.i, g: solve.g, u_p: solve.u_p, b: None };
        self.hubs[p].as_mut().expect("a hub partition").solve = Some(solve);
        self.publish_hub(p, changed);
        self.hub_updates.push((p, contrib));
    }

    /// The rail sensitivity of hub partition `p` solved to `x` at rail `u`
    /// through `factors` (Rails point 3): its own stamps, the rail stamped as
    /// one more unknown. A curved partition takes it exactly at `x`, and
    /// hands back its IC stamps that are no fixed conductance: those that
    /// follow the rail, or with a free IC supply, every one that follows a
    /// supply.
    #[allow(clippy::type_complexity)]
    fn hub_sensitivity(&self, p: usize, now: f64, drivers: &[Driver], x: &[f64], u: f64, factors: &lu::LuFactorization) -> Result<(crate::solve::RailSens, HubIc), String> {
        let hub = self.hubs[p].as_ref().expect("a hub partition");
        let rail = self.rail.expect("a hub partition implies a rail");
        let n = hub.shape.n;
        let z: Vec<f64> = if hub.nonlinear { x.iter().copied().chain(std::iter::once(u)).collect() } else { Vec::new() };
        let bnd = Bnd::new(&self.voltages).with_holds(self.holds_slice()).with_ext(rail, n).hold_power_at(u);
        let ic = (hub.nonlinear && !self.holds_free_ic_supply[p]).then(|| {
            let ic = crate::solve::ic_rail_terms(now, drivers, &hub.touch_ics, &hub.node_owner, &bnd, p, &z, n);
            let explicit = ic.explicit_u(&z, n);
            (ic, explicit)
        });
        let supply = self.holds_free_ic_supply[p].then(|| {
            let ic = crate::solve::ic_supply_terms(now, drivers, &hub.touch_ics, &hub.node_owner, &bnd, p, &z);
            let extra = ic.extra_jacobian(&z, n);
            (ic, extra)
        });
        let sens = rail_sensitivity(n, x, u, factors, ic.as_ref().map(|(_, e)| &e[..]), supply.as_ref().map(|(_, e)| &e[..]), |z, a, b| self.hub_residual_stamp(p, now, drivers, u, z, a, b))
            .map_err(|e| format!("partition {p}: {e}"))?;
        let ic = match (ic, supply) {
            (Some((ic, _)), _) => HubIc::Rail(ic),
            (None, Some((ic, _))) => HubIc::Supply(ic),
            (None, None) => HubIc::Linear,
        };
        Ok((sens, ic))
    }

    /// Hub partition `p`'s own stamps with the rail as unknown `n` (`z` its
    /// `n + 1` values), every nonlinear element linearised about `z`, so `A
    /// z - b` is the true residual (`solve::rail_sensitivity`'s
    /// `stamp_ext`); an IC's power is judged at rail `u`.
    #[allow(clippy::too_many_arguments)]
    fn hub_residual_stamp(&self, p: usize, now: f64, drivers: &[Driver], u: f64, z: &[f64], a: &mut [f64], b: &mut [f64]) {
        let hub = self.hubs[p].as_ref().expect("a hub partition");
        let rail = self.rail.expect("a hub partition implies a rail");
        let n = hub.shape.n;
        let bnd = Bnd::new(&self.voltages).with_holds(self.holds_slice()).with_ext(rail, n).hold_power_at(u);
        stamp_linear_base(&hub.shape, n + 1, a, b, &self.switch_params, &bnd);
        stamp_ics_into_dyn(n + 1, a, b, now, drivers, &hub.touch_ics, &hub.node_owner, &bnd, p, z);
        stamp_junctions(&hub.shape.junctions, n + 1, a, b, z, &bnd);
    }

    /// Takes hub partition `p`'s iterated solve `x` (an iterate that passed
    /// Newton's tolerance at rail `u`) on to the solution, to rounding: its
    /// band bounds the correction from the solution, so an offset at the
    /// solve is outside its budget. A free supply's iterated solve restamps
    /// `R_out` at the last iterate's supply, so it converges linearly and
    /// stops up to ~1e-3 V short (FXBv: 9.6e-4 V, 0.51 of W0, on a 167 ohm
    /// feed; ruling R19 (a)); a junction's (with or without a chip on the
    /// rail) is Newton already, but stops at its tolerance plus one polish
    /// iteration (FXC: a 16 mA gate into 1000 ohm, an LED and 47 ohm sat
    /// 6.3e-9 V and 2e-10 A off, 6.5e5x its band's bound; ruling R20 (a)).
    /// These are Newton steps on the exact Jacobian, the one the
    /// sensitivity takes for the partition's kind: the residual stamp's
    /// matrix (junctions linearised about the iterate, so exact; a chip on
    /// the rail stamps conductances fixed by its supply, which never moves
    /// here) plus, with a free supply, the IC supply terms it misses
    /// (`curvature::IcSupply::extra_jacobian`). They converge quadratically
    /// where the stamps are smooth.
    ///
    /// Stop rule: after each step of largest move `m`, the solve has
    /// arrived when `m` is within `POLISH_FLOOR` ulps of the largest
    /// voltage (nothing smaller is representable), or when the step stops
    /// contracting (`m` over a quarter of the step before it, which a
    /// quadratic step far from rounding never is): the steps are then
    /// rounding noise, and applying the last one can leave the solve
    /// further from the solution than the iterate it was taken from (FXDv:
    /// 5-20 % of such exits more than 2x worse). So on a stall the solve
    /// keeps, of every iterate it evaluated and the stepped one, the one
    /// with the smallest residual: each node's KCL imbalance over
    /// its own conductance, in volts ([`scaled_residual`]), read off the
    /// stamp so that every candidate is ranked alike (ruling R23, FXE
    /// 2026-10-21; the Newton step through each one's own factors ranked a
    /// GMIN-held cluster's iterates by its factorization's rounding). The
    /// kept iterate has arrived when the step it asks for (`|J^-1 r|`, the
    /// stepped one's through the stalled step's factors) is at most
    /// `POLISH_STALL` (relative) or the residual's own rounding through the
    /// same factors ([`step_rounding`]).
    /// The second floor is for an ill-conditioned partition, whose solution
    /// floating point fixes only to well above an ulp (an LED off the rail
    /// into 330 ohm and an open switch: both nodes held by 1e-10 S, so
    /// 1e-19 A of residual rounding moves the step 1e-8 V); the solve's
    /// band counts that distance (`curvature::Floors`), and the refinement
    /// takes a solve whose floor nears W0's tolerance further
    /// ([`Self::refine_hub`]). It terminates because it runs at most
    /// `POLISH_MAX` steps; a stall whose kept iterate is above both floors,
    /// or the cap reached still contracting (a kink or a power edge between
    /// the iterate and a solution, which the iterated solve's own ping-pong
    /// declines as well), is `-4`, like an iterated solve that never passed.
    /// Every step is counted with the partition's Newton iterations
    /// (contract stat 11).
    fn polish_hub(&mut self, p: usize, now: f64, drivers: &[Driver], mut x: Vec<f64>, u: f64, gated: &[u32], mut flips: Option<&mut crate::solve::hold::Flips>) -> Result<Vec<f64>, String> {
        const POLISH_MAX: u32 = 8;
        const POLISH_FLOOR: f64 = 16.0 * f64::EPSILON;
        const POLISH_STALL: f64 = 1e-12;
        #[allow(unused_mut)]
        let mut cap = POLISH_MAX;
        #[cfg(test)]
        if let Some(c) = newton::hooks::hub_polish_cap() {
            if c == 0 {
                return Ok(x);
            }
            cap = c;
        }
        let hub = self.hubs[p].as_ref().expect("a hub partition");
        let rail = self.rail.expect("a hub partition implies a rail");
        let n = hub.shape.n;
        let size = n + 1;
        let free_supply = self.holds_free_ic_supply[p];
        let mut z: Vec<f64> = x.iter().copied().chain(std::iter::once(u)).collect();
        let (mut a, mut b) = (vec![0.0; size * size], vec![0.0; size]);
        let residual_step = |a: &[f64], b: &[f64], z: &[f64]| -> Vec<f64> { (0..n).map(|k| b[k] - a[k * size..(k + 1) * size].iter().zip(z).map(|(a, z)| a * z).sum::<f64>()).collect() };
        // The last step's `|J^-1 r|`, and of the iterates evaluated so far
        // the one with the smallest residual ([`scaled_residual`]): `(size,
        // |J^-1 r|, iterate)`.
        let mut prev = f64::INFINITY;
        let mut best: (f64, f64, Vec<f64>) = (f64::INFINITY, f64::INFINITY, Vec::new());
        let mut steps = 0u32;
        #[cfg(test)]
        let mut stalled = false;
        let verdict = loop {
            if steps == cap {
                break Err(format!("partition {p} ({n} node(s)): the hub polish did not reach the solution in {cap} steps"));
            }
            steps += 1;
            // W1c point 1a's trigger: each gated IC's verdict at this iterate.
            if let Some(flips) = flips.as_deref_mut() {
                for (k, &ic_index) in gated.iter().enumerate() {
                    let (device, ic) = &self.ics[ic_index as usize];
                    let hold = digital::power::hold_of(self.holds_slice(), *device);
                    flips.note(k, digital::power::verdict(ic, self.read_node(ic.vcc.node, p, &x), self.read_node(ic.gnd.node, p, &x), hold));
                }
            }
            a.fill(0.0);
            b.fill(0.0);
            self.hub_residual_stamp(p, now, drivers, u, &z, &mut a, &mut b);
            let extra = free_supply.then(|| {
                let bnd = Bnd::new(&self.voltages).with_holds(self.holds_slice()).with_ext(rail, n).hold_power_at(u);
                crate::solve::ic_supply_terms(now, drivers, &hub.touch_ics, &hub.node_owner, &bnd, p, &z).extra_jacobian(&z, n)
            });
            let mut step = residual_step(&a, &b, &z);
            let q = scaled_residual(&step, &a, n);
            #[cfg(test)]
            if steps == 1 {
                newton::hooks::note_polish_entry(q);
            }
            let mut jac = vec![0.0; n * n];
            for r in 0..n {
                for c in 0..n {
                    jac[r * n + c] = a[r * size + c] + extra.as_ref().map_or(0.0, |e| e[r * size + c]);
                }
            }
            let singular = || format!("partition {p} ({n} node(s)): singular or non-finite pivot in the hub polish");
            let Ok(factors) = lu::lu_factor(jac, n) else { break Err(singular()) };
            if factors.solve(&mut step).is_err() {
                break Err(singular());
            }
            if !step.iter().all(|v| v.is_finite()) {
                break Err(format!("partition {p} ({n} node(s)): non-finite entry in the hub polish"));
            }
            let m = step.iter().fold(0.0f64, |acc, v| acc.max(v.abs()));
            let scale = x.iter().fold(0.0f64, |acc, v| acc.max(v.abs())).max(1.0);
            if m <= POLISH_FLOOR * scale {
                for (k, d) in step.iter().enumerate() {
                    x[k] += d;
                }
                break Ok(());
            }
            if q < best.0 {
                best.0 = q;
                best.1 = m;
                if best.2.is_empty() {
                    best.2 = x.clone();
                } else {
                    best.2.copy_from_slice(&x);
                }
            }
            if m > 0.25 * prev {
                // Noise. The candidates: every iterate evaluated (the best
                // of them) and the stepped one (its step through this
                // step's factors).
                let z_next: Vec<f64> = (0..size).map(|k| if k < n { x[k] + step[k] } else { u }).collect();
                let (mut a_next, mut b_next) = (vec![0.0; size * size], vec![0.0; size]);
                self.hub_residual_stamp(p, now, drivers, u, &z_next, &mut a_next, &mut b_next);
                let mut next = residual_step(&a_next, &b_next, &z_next);
                let q_next = scaled_residual(&next, &a_next, n);
                if q_next < best.0 && factors.solve(&mut next).is_ok() && next.iter().all(|v| v.is_finite()) {
                    best = (q_next, next.iter().fold(0.0f64, |acc, v| acc.max(v.abs())), z_next[..n].to_vec());
                }
                let kept = best.1;
                // The rounding floor (`n` solves) only where the stall
                // floor does not settle it.
                if kept <= POLISH_STALL * scale || kept <= step_rounding(&factors, &a, &b, &z, n) {
                    x = std::mem::take(&mut best.2);
                    #[cfg(test)]
                    {
                        stalled = true;
                    }
                    break Ok(());
                }
                break Err(format!("partition {p} ({n} node(s)): the hub polish stalled {kept:e} V from the solution"));
            }
            for (k, d) in step.iter().enumerate() {
                x[k] += d;
                z[k] = x[k];
            }
            prev = m;
        };
        self.newton_iterations += steps as u64;
        #[cfg(test)]
        if stalled && newton::hooks::polish_norms_armed() {
            // The published iterate's residual.
            let z: Vec<f64> = x.iter().copied().chain(std::iter::once(u)).collect();
            a.fill(0.0);
            b.fill(0.0);
            self.hub_residual_stamp(p, now, drivers, u, &z, &mut a, &mut b);
            let scale = x.iter().fold(0.0f64, |acc, v| acc.max(v.abs())).max(1.0);
            newton::hooks::note_polish_exit(scaled_residual(&residual_step(&a, &b, &z), &a, n), scale);
        }
        verdict.map(|()| x)
    }

    /// Hub partition `p`'s residual at `z` (its `n` unknowns, then the
    /// rail), rows `0..n`, summed element by element: each branch's own
    /// current (the node shunts, every conductance [`stamp_linear_base`]
    /// and [`stamp_ics_into_dyn`] stamp, each junction's `I(v) + GMIN v`),
    /// with compensated sums; and each element's rows and rounding, a few
    /// ulps of its current. The residual [`Self::hub_residual_stamp`]'s matrix
    /// gives, `A z - b`, rounds by ulps of the gross terms instead: a 1e-3
    /// ohm resistor between two 5 V nodes stamps 5e3 A terms for a current
    /// of mA, and its diagonal's sum of conductances rounds by ulps of all
    /// of them (ruling R23 (ii), FXE 2026-10-21). For the refinement only
    /// ([`Self::refine_hub`]).
    #[allow(clippy::type_complexity)]
    fn hub_residual_exact(&self, p: usize, now: f64, drivers: &[Driver], u: f64, z: &[f64]) -> (Vec<f64>, Vec<(Option<usize>, Option<usize>, f64)>) {
        let hub = self.hubs[p].as_ref().expect("a hub partition");
        let rail = self.rail.expect("a hub partition implies a rail");
        let n = hub.shape.n;
        let bnd = Bnd::new(&self.voltages).with_holds(self.holds_slice()).with_ext(rail, n).hold_power_at(u);
        let eps = f64::EPSILON;
        // Neumaier's sum per row, `(sum, compensation)`, and each element's
        // rows and rounding.
        let mut acc = vec![(0.0f64, 0.0f64); n];
        let mut pairs = Vec::new();
        let mut add = |l: usize, i: f64| {
            let (s, c) = &mut acc[l];
            let t = *s + i;
            *c += if s.abs() >= i.abs() { (*s - t) + i } else { (i - t) + *s };
            *s = t;
        };
        let mut branch = |pa: &DynPin, pb: &DynPin, i: f64, unc: f64| {
            let (la, lb) = (bnd.local(pa).filter(|&l| l < n), bnd.local(pb).filter(|&l| l < n));
            if let Some(l) = la {
                add(l, i);
            }
            if let Some(l) = lb {
                add(l, -i);
            }
            if la.is_some() || lb.is_some() {
                pairs.push((la, lb, unc));
            }
        };
        let ulps = crate::solve::curvature::ROUND_ULPS * eps;
        let mut element = |pa: &DynPin, pb: &DynPin, g: f64, ieq: f64| {
            let gv = g * (pin_voltage(pa, z, &bnd) - pin_voltage(pb, z, &bnd));
            branch(pa, pb, gv + ieq, ulps * (gv.abs() + ieq.abs()));
        };
        crate::solve::linear_elements(&hub.shape, &self.switch_params, |pa, pb, g| element(pa, pb, g, 0.0));
        crate::solve::ic_elements(now, drivers, &hub.touch_ics, &hub.node_owner, &bnd, p, z, |pa, pb, g, ieq| element(pa, pb, g, ieq));
        for j in &hub.shape.junctions {
            let v = pin_voltage(&j.anode, z, &bnd) - pin_voltage(&j.cathode, z, &bnd);
            let i = j.law.current(v) + led::GMIN_D * v;
            // The law's evaluation (its exponential less one cancels near 0
            // V, so by ulps of `is e^(v/vtn) = gd vtn` too), and its
            // voltage's rounding through its slope.
            let (g, _) = j.law.companion(v);
            branch(&j.anode, &j.cathode, i, ulps * (i.abs() + g * (j.law.vtn + v.abs())));
        }
        for (l, &zl) in z.iter().enumerate().take(n) {
            let i = NODE_RSHUNT_G * zl;
            add(l, i);
            pairs.push((Some(l), None, ulps * i.abs()));
        }
        (acc.into_iter().map(|(s, c)| s + c).collect(), pairs)
    }

    /// Mixed-precision iterative refinement of hub partition `p`'s solve `x`
    /// at rail `u` (ruling R23 (ii), FXE 2026-10-21): the residual summed
    /// element by element ([`Self::hub_residual_exact`]), the correction
    /// solved through `lu`, the factors of the exact Jacobian at `x` the
    /// sensitivity already holds (a chord step: the Jacobian moves only to
    /// second order over the distance left). Extreme resistance ratios in
    /// one partition (1e-3 to 1e10 ohm) left polished solves up to 70 W0
    /// from the exact solution (FXDv's families): the stamp's residual
    /// rounds by ulps of its gross terms, which `J^-1` multiplies by up to
    /// 1e13; the element residual rounds by ulps of the currents
    /// themselves. Run only where the rounding floor passes `REFINE_GATE` of
    /// W0 (`solve_hub`); the band then counts the refined solve's floor.
    ///
    /// Each iterate's residual through `lu` is the step it asks for, `m =
    /// |J^-1 r|`; the iterate with the least `m` is kept. The steps contract
    /// linearly, by about the Jacobian's condition number times an ulp (a
    /// part in five at 1e15), so the refinement goes on while `m` falls and
    /// stops when it does not (the steps are the residual's rounding), when
    /// `m` is within `POLISH_FLOOR` ulps of the largest voltage, or at
    /// `REFINE_MAX` residuals. It terminates because it evaluates at most
    /// `REFINE_MAX` residuals. It fails (`-4`) only on a singular factor:
    /// the kept iterate is never further from the solution than the
    /// polished one it started from, and where `f64` factors cannot place
    /// the floor under the gate (a 1e-3 ohm resistor inside a GMIN-held
    /// cluster: its Jacobian's entries lose the cluster's 1e-12 S to their
    /// own rounding, so `|J^-1|` overstates the floor 1e6-fold, 58 W0 for a
    /// solve 5e-5 W0 from the exact one) the band counts the floor and
    /// narrows to the narrowest. Returns the kept iterate, its residual and
    /// its elements' rounding (its floor's, `curvature::floors`). Every
    /// residual is counted with the partition's Newton iterations (contract
    /// stat 11).
    #[allow(clippy::type_complexity)]
    fn refine_hub(&mut self, p: usize, now: f64, drivers: &[Driver], mut x: Vec<f64>, u: f64, lu: &lu::LuFactorization) -> Result<(Vec<f64>, Vec<f64>, Vec<(Option<usize>, Option<usize>, f64)>), String> {
        const REFINE_MAX: u32 = 8;
        const POLISH_FLOOR: f64 = 16.0 * f64::EPSILON;
        let n = x.len();
        let mut z: Vec<f64> = x.iter().copied().chain(std::iter::once(u)).collect();
        // `(m, x, r, pairs)` of the best evaluated iterate.
        #[allow(clippy::type_complexity)]
        let mut best: (f64, Vec<f64>, Vec<f64>, Vec<(Option<usize>, Option<usize>, f64)>) = (f64::INFINITY, Vec::new(), Vec::new(), Vec::new());
        let mut prev = f64::INFINITY;
        let mut steps = 0u32;
        let verdict = loop {
            steps += 1;
            let (r, pairs) = self.hub_residual_exact(p, now, drivers, u, &z);
            let mut step: Vec<f64> = r.iter().map(|v| -v).collect();
            if lu.solve(&mut step).is_err() || !step.iter().all(|v| v.is_finite()) {
                break Err(format!("partition {p} ({n} node(s)): singular or non-finite pivot in the hub refinement"));
            }
            let m = step.iter().fold(0.0f64, |acc, v| acc.max(v.abs()));
            if m < best.0 {
                best = (m, x.clone(), r, pairs);
            }
            let scale = x.iter().fold(0.0f64, |acc, v| acc.max(v.abs())).max(1.0);
            if m <= POLISH_FLOOR * scale || m >= prev || steps == REFINE_MAX {
                break Ok(());
            }
            for (k, d) in step.iter().enumerate() {
                x[k] += d;
                z[k] = x[k];
            }
            prev = m;
        };
        self.newton_iterations += steps as u64;
        verdict.map(|()| (best.1, best.2, best.3))
    }

    /// [`curvature::floors`] of hub partition `p`'s solve `x` at rail `u`.
    #[allow(clippy::too_many_arguments)]
    fn hub_floors(&self, p: usize, x: &[f64], u: f64, sens: &crate::solve::RailSens, lu_j: &lu::LuFactorization, r: &[f64], rho: &[f64], pairs: &[(Option<usize>, Option<usize>, f64)]) -> Result<crate::solve::curvature::Floors, String> {
        let hub = self.hubs[p].as_ref().expect("a hub partition");
        let rail = self.rail.expect("a hub partition implies a rail");
        let n = hub.shape.n;
        let bnd = Bnd::new(&self.voltages).with_holds(self.holds_slice()).with_ext(rail, n);
        crate::solve::curvature::floors(&hub.shape, &self.switch_params, &bnd, x, u, sens, lu_j, r, rho, pairs).map_err(|e| format!("partition {p}: {e}"))
    }

    /// Solves hub partition `p` at the present rail (Rails points 2 and 3):
    /// a cache hit within its band of its stored rail replays the
    /// contribution; otherwise the partition solves, its sensitivity and
    /// band follow through the same factors, and the entry is stored.
    fn solve_hub(&mut self, p: usize, now: f64, drivers: &[Driver], changed: &mut Vec<usize>) -> Result<(), String> {
        let rail = self.rail.expect("a hub partition implies a rail");
        let u = self.voltages[rail];
        let free_supply = self.holds_free_ic_supply[p];
        let mut key_hash = None;
        if !free_supply && !self.no_solution_cache {
            self.build_key(p, now, drivers);
            let hash = hash_key(&self.key_scratch);
            key_hash = Some(hash);
            let rule = self.hub_cache_rule;
            if let Some(e) = self.caches[p].find_hub(hash, &self.key_scratch, |h| (u - h.u_p).abs() <= rule.band(h.band)) {
                let solve = (**self.caches[p].entries[e].hub.as_ref().expect("a hub entry")).clone();
                self.cache_hits += 1.0;
                self.set_hub_solve(p, solve, changed);
                return Ok(());
            }
        }
        let iterated = free_supply || !self.junctions[p].is_empty();
        // An iterated partition is curved (`HubIc::Rail` or `Supply`), whose
        // sensitivity solves its own exact Jacobian, not through `factors`,
        // so they need not follow the polish. A free IC supply's polish is
        // inside W1c point 1a's procedure.
        let (mut x, factors) = if free_supply {
            self.free_supply_solve(p, now, drivers, Some(u))?
        } else if iterated {
            let (x, factors) = self.iterated_solve_factored(p, now, drivers)?;
            (self.polish_hub(p, now, drivers, x, u, &[], None)?, factors)
        } else {
            self.direct_solve_factored(p, now, drivers)?
        };
        let (mut sens, mut ic) = self.hub_sensitivity(p, now, drivers, &x, u, &factors)?;
        self.sensitivities += 1;
        // A curved partition's band counts the solve's own distance from
        // the solution (`curvature::Floors`), from its residual at `x`
        // through the factors the sensitivity solved with; where that
        // passes `REFINE_GATE` of W0's tolerance the solve is refined.
        let floors = match (&ic, sens.lu.take()) {
            (HubIc::Linear, _) | (_, None) => None,
            (_, Some(lu_j)) => {
                let n = x.len();
                let rho: Vec<f64> = sens.gross[..n].iter().map(|g| crate::solve::curvature::ROUND_ULPS * f64::EPSILON * g).collect();
                let mut floors = self.hub_floors(p, &x, u, &sens, &lu_j, &sens.res, &rho, &[])?;
                if floors.worst_w0 > REFINE_GATE {
                    let (refined, r, pairs) = self.refine_hub(p, now, drivers, x, u, &lu_j)?;
                    x = refined;
                    (sens, ic) = self.hub_sensitivity(p, now, drivers, &x, u, &factors)?;
                    let lu_j = sens.lu.take().ok_or_else(|| format!("partition {p}: a refined hub solve without its Jacobian's factors"))?;
                    floors = self.hub_floors(p, &x, u, &sens, &lu_j, &r, &vec![0.0; n], &pairs)?;
                    #[cfg(test)]
                    newton::hooks::note_refined(floors.worst_w0);
                }
                Some(floors)
            }
        };
        let band = {
            let hub = self.hubs[p].as_ref().expect("a hub partition");
            let bnd = Bnd::new(&self.voltages).with_holds(self.holds_slice()).with_ext(rail, hub.shape.n);
            let floors = floors.unwrap_or_default();
            match ic {
                HubIc::Linear => Some(rails::RAIL_RESOLVE_V),
                HubIc::Rail(ic) => crate::solve::curvature::hub_band(&hub.shape, &self.switch_params, &bnd, &x, u, &sens, &ic, floors),
                HubIc::Supply(ic) => crate::solve::curvature::supply_band(&hub.shape, &self.switch_params, &bnd, &x, u, &sens, &ic, floors),
            }
        };
        let solve = HubSolve { x, s: sens.s, i: sens.i, g: sens.g, u_p: u, band };
        if let Some(hash) = key_hash {
            self.caches[p].insert(hash, &self.key_scratch, &solve.x, Some(Box::new(solve.clone())));
        }
        self.set_hub_solve(p, solve, changed);
        Ok(())
    }

    /// [`PartitionSolver::resolve_dirty`]'s body; `changed` collects every
    /// node whose published bits moved, and the readers of those nodes are
    /// appended to `out`, ascending and without repeats.
    fn resolve_dirty_inner(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver], changed: &mut Vec<usize>, out: &mut Vec<DeviceId>) -> Result<(), String> {
        // W1f: the boundary and the hub corrections move first, so every
        // solve below reads them; they queue no reader of their own.
        self.apply_pending_boundary(changed);
        let silent_len = changed.len();
        self.silent_len = silent_len;
        for &partition in dirty {
            let p_idx = partition.0 as usize;
            let size = self.partitions[p_idx].len();
            if size == 0 {
                continue;
            }
            if self.hubs[p_idx].is_some() {
                self.solve_hub(p_idx, now, drivers, changed)?;
                continue;
            }

            if self.holds_free_ic_supply[p_idx] || self.exact.is_some_and(|e| e.partition == p_idx) {
                // W1c contract point 3/4: R_out moves with this partition's
                // own solved state, so the solution cache never applies
                // (junctions or not, W1e LEDs point 7): every solve restamps
                // from the iterate, and the result goes straight to
                // `self.voltages`.
                let x = self.iterated_solve(p_idx, now, drivers)?;
                self.publish_partition(p_idx, &x, changed);
            } else {
                let mut key_hash = None;
                let mut used_cache = false;
                if !self.no_solution_cache {
                    self.build_key(p_idx, now, drivers);
                    let hash = hash_key(&self.key_scratch);
                    key_hash = Some(hash);
                    if let Some(entry) = self.caches[p_idx].find(hash, &self.key_scratch) {
                        self.cache_hits += 1.0;
                        // Replayed straight out of the entry: nothing is
                        // allocated on a hit.
                        for l in 0..size {
                            let node = self.partitions[p_idx][l];
                            let new = self.caches[p_idx].entries[entry].x[l];
                            if new.to_bits() != self.voltages[node].to_bits() {
                                changed.push(node);
                                if self.window {
                                    self.v_log.push((node as u32, self.voltages[node].to_bits()));
                                }
                            }
                            self.voltages[node] = new;
                        }
                        used_cache = true;
                    }
                }
                if !used_cache {
                    // A partition holding only junctions is nonlinear but
                    // keeps the cache (W1e LEDs point 7): a miss runs the
                    // Newton solve, polish included, and stores that result.
                    let x = if self.junctions[p_idx].is_empty() {
                        self.direct_solve(p_idx, now, drivers)?
                    } else {
                        self.iterated_solve(p_idx, now, drivers)?
                    };
                    self.publish_partition(p_idx, &x, changed);
                    if let Some(hash) = key_hash {
                        self.caches[p_idx].insert(hash, &self.key_scratch, &x, None);
                    }
                }
            }
        }

        // Recomputed on every call (even with an empty `dirty`): a device's
        // current can depend purely on already-fixed voltages, which no
        // partition solve ever touches, so this is the only place that ever
        // (re)publishes it. Only what a changed node or a live-parameter
        // change can have moved is actually recomputed.
        let bad_device = self.recompute_currents(changed);
        #[cfg(test)]
        {
            let reference = self.recompute_currents_reference();
            // A skipped source walk leaves the sources' slots as they were:
            // that is the point, and nothing reads them.
            let sources: std::collections::HashSet<usize> = self.resolved.iter().map(|r| r.device_index).collect();
            for (device, (got, want)) in self.currents.iter().zip(&reference).enumerate() {
                if self.walk_skipped && sources.contains(&device) {
                    continue;
                }
                assert_eq!(got.to_bits(), want.to_bits(), "device {device}: incremental current {got} != whole-circuit {want}");
            }
        }

        // Non-finite values can only sit in what this call just wrote: every
        // other published value passed this check when it was written (or is
        // a snapshot of one that did, `self.full` covering the rest). The
        // lowest node index is reported, as the whole-array scan did.
        let bad_node = if self.full {
            self.voltages.iter().position(|v| !v.is_finite())
        } else {
            changed.iter().copied().filter(|&n| !self.voltages[n].is_finite()).min()
        };
        if let Some(node) = bad_node {
            return Err(format!("node {node}: published voltage {} is not finite", self.voltages[node]));
        }
        let bad_device = if self.full { self.currents.iter().position(|i| !i.is_finite()) } else { bad_device };
        if let Some(device) = bad_device {
            return Err(format!("device {device}: published current {} is not finite", self.currents[device]));
        }
        self.full = false;

        // Ascending and duplicate-free, like the `BTreeSet` it replaces.
        let mut readers = std::mem::take(&mut self.readers_scratch);
        readers.clear();
        for &node in changed[silent_len..].iter() {
            readers.extend_from_slice(&self.node_to_ic_readers[node]);
        }
        readers.sort_unstable();
        readers.dedup();
        out.extend(readers.iter().map(|&d| DeviceId(d as u32)));
        self.readers_scratch = readers;
        Ok(())
    }

    /// Writes partition `p_idx`'s solved local voltages `x` into
    /// `self.voltages`, noting every node whose bits moved (W1c contract
    /// point 5's reader rule compares bitwise: a plain `f64 != f64` would
    /// miss a NaN-to-NaN "no-op", which `to_bits` does not).
    fn publish_partition(&mut self, p_idx: usize, x: &[f64], changed: &mut Vec<usize>) {
        for (l, &node) in self.partitions[p_idx].iter().enumerate() {
            if x[l].to_bits() != self.voltages[node].to_bits() {
                changed.push(node);
                if self.window {
                    self.v_log.push((node as u32, self.voltages[node].to_bits()));
                }
            }
            self.voltages[node] = x[l];
        }
    }

    /// The junction's branch current at the published voltages, from its
    /// memo when neither end's voltage bits moved since it was computed.
    fn memo_junction_current(&mut self, k: usize) -> f64 {
        // A failed LED stamps no junction at all (W1g point 12), GMIN included.
        if !self.kcl_active[k] {
            return 0.0;
        }
        let j = &self.kcl_junctions[k];
        let (va, vk) = (self.voltages[j.anode].to_bits(), self.voltages[j.cathode].to_bits());
        let law_gen = self.kcl_law_gen[k];
        let memo = &mut self.junction_memo[k];
        if !(memo.valid && memo.v_anode == va && memo.v_cathode == vk && memo.law_gen == law_gen) {
            *memo = JunctionMemo { v_anode: va, v_cathode: vk, law_gen, current: junction_branch_current(j, &self.voltages), valid: true };
        }
        memo.current
    }

    /// The published resistor, switch and source currents, brought up to date
    /// with `self.voltages` and the current switch params, at the cost of
    /// what changed instead of the whole circuit. `changed` is every node
    /// whose published bits moved in this resolve. Returns the lowest device
    /// index whose recomputed current is not finite, if any.
    ///
    /// A resistor or switch current is `sum(g * (V[u] - V[w]))` over its
    /// contacts, a pure function of its pins' voltages and its params: only a
    /// device on a changed node, or one `apply_params` touched, can differ
    /// from the value already published (`self.full` recomputes them all).
    /// A source current is the children-before-parents KCL walk (contract,
    /// point 5) over `leaving[child]`, each summed from its own terms in the
    /// historical order (see [`Term`]) so the bits are those of the
    /// whole-array walk.
    fn recompute_currents(&mut self, changed: &[usize]) -> Option<usize> {
        let resistor_count = self.resistors.len();
        let stamp_count = resistor_count + self.switches.len();
        let mut work = std::mem::take(&mut self.work);
        work.clear();
        if self.full {
            work.extend(0..stamp_count as u32);
        } else {
            for &node in changed {
                for &id in &self.stamps_by_node[node] {
                    if !self.stamp_listed[id as usize] {
                        self.stamp_listed[id as usize] = true;
                        work.push(id);
                    }
                }
            }
            for &id in &self.param_dirty {
                if !self.stamp_listed[id as usize] {
                    self.stamp_listed[id as usize] = true;
                    work.push(id);
                }
            }
            for &id in &work {
                self.stamp_listed[id as usize] = false;
            }
        }
        self.param_dirty.clear();

        let mut bad: Option<usize> = None;
        for &id in &work {
            let stamp = if (id as usize) < resistor_count { &self.resistors[id as usize] } else { &self.switches[id as usize - resistor_count] };
            let mut total = 0.0;
            for c in &stamp.contacts {
                total += c.g * (self.voltages[c.u] - self.voltages[c.w]);
            }
            let old = self.currents[stamp.device_index];
            if self.window && total.to_bits() != old.to_bits() {
                self.i_log.push((stamp.device_index as u32, old.to_bits()));
            }
            self.currents[stamp.device_index] = total;
            if !total.is_finite() {
                bad = Some(bad.map_or(stamp.device_index, |b| b.min(stamp.device_index)));
            }
        }
        self.work = work;

        // The source walk sums, for each source, every term at its child node
        // (the rail, in a logic circuit: one term per resistor and junction on
        // it). Its results are read by nothing when the owner computes the
        // source currents itself, and it can only fail (a non-finite sum) if
        // some voltage or coefficient it reads is out of bounds, which the
        // owner has excluded for the coefficients and for every voltage as of
        // the last publish; the nodes this call changed are checked here. So
        // with all of those in hand the walk is skipped, and otherwise it runs
        // exactly as it always did, error included.
        self.walk_skipped = self.source_walk_skippable
            && self.inputs_bounded
            && !self.full
            && changed.iter().all(|&n| self.voltages[n].abs() <= crate::solve::LAZY_V_LIMIT);
        if self.walk_skipped {
            return bad;
        }

        for k in (0..self.resolved.len()).rev() {
            let mut leaving = 0.0;
            for t in 0..self.resolved_terms[k].len() {
                match self.resolved_terms[k][t] {
                    Term::Contact { stamp, contact, plus } => {
                        let stamp = if (stamp as usize) < resistor_count { &self.resistors[stamp as usize] } else { &self.switches[stamp as usize - resistor_count] };
                        let c = stamp.contacts[contact as usize];
                        let i = c.g * (self.voltages[c.u] - self.voltages[c.w]);
                        leaving = if plus { leaving + i } else { leaving - i };
                    }
                    Term::Junction { junction, plus } => {
                        let i = self.memo_junction_current(junction as usize);
                        leaving = if plus { leaving + i } else { leaving - i };
                    }
                    Term::Shunt => {
                        leaving += NODE_RSHUNT_G * self.voltages[self.resolved[k].child_node];
                    }
                }
            }
            let r = &self.resolved[k];
            let total = leaving + self.walk_extra[r.child_node];
            let child_is_pos = !r.parent_is_pos;
            let i_s = if child_is_pos { -total } else { total };
            let old = self.currents[r.device_index];
            if self.window && i_s.to_bits() != old.to_bits() {
                self.i_log.push((r.device_index as u32, old.to_bits()));
            }
            self.currents[r.device_index] = i_s;
            if !i_s.is_finite() {
                bad = Some(bad.map_or(r.device_index, |b| b.min(r.device_index)));
            }
            let contribution = if r.parent_is_pos { i_s } else { -i_s };
            self.walk_extra[r.parent_node] += contribution;
        }
        for r in &self.resolved {
            self.walk_extra[r.parent_node] = 0.0;
        }
        bad
    }

    /// What `recompute_currents` used to be, kept verbatim (over
    /// `devices::stamp` itself, not the cached contacts) as the oracle every
    /// test-build resolve checks the incremental result against, bit for
    /// bit: resistor and switch currents by summing every
    /// contact's own conductance times its own terminal voltage difference
    /// (folding the same total into `leaving` for the KCL walk), then source
    /// currents by the children-before-parents walk (contract, point 5),
    /// from `self.voltages` as it currently stands.
    #[cfg(test)]
    fn recompute_currents_reference(&self) -> Vec<f64> {
        let mut currents = vec![0.0; self.currents.len()];
        let mut leaving = vec![0.0; self.voltages.len()];

        for stamp in self.resistors.iter().chain(self.switches.iter()) {
            let mut total = 0.0;
            for c in devices::stamp(stamp.kind, stamp.pins.len(), &stamp.params) {
                let (u, w) = (stamp.pins[c.pin_a], stamp.pins[c.pin_b]);
                let i = c.conductance * (self.voltages[u] - self.voltages[w]);
                total += i;
                leaving[u] += i;
                leaving[w] -= i;
            }
            currents[stamp.device_index] = total;
        }

        // Every stamped junction's branch current, anode to cathode, GMIN
        // included (W1e, LEDs point 12).
        for (k, j) in self.kcl_junctions.iter().enumerate() {
            if !self.kcl_active[k] {
                continue;
            }
            let i = junction_branch_current(j, &self.voltages);
            leaving[j.anode] += i;
            leaving[j.cathode] -= i;
        }

        // Shunt current, every node but the datum (contract, point 2).
        for (node, v) in self.voltages.iter().enumerate().skip(1) {
            leaving[node] += NODE_RSHUNT_G * v;
        }

        let mut extra = vec![0.0; self.voltages.len()];
        for r in self.resolved.iter().rev() {
            let total = leaving[r.child_node] + extra[r.child_node];
            let child_is_pos = !r.parent_is_pos;
            let i_s = if child_is_pos { -total } else { total };
            currents[r.device_index] = i_s;

            let contribution = if r.parent_is_pos { i_s } else { -i_s };
            extra[r.parent_node] += contribution;
        }

        currents
    }
}

impl PartitionSolver for StaticPartitionSolver {
    /// W1c contract point 5's reader rule ("resolve_dirty returns every IC
    /// reader... that reads a node... whose published voltage changed in
    /// this re-solve, bitwise") is folded into the SAME per-partition loop
    /// that already solves each dirty partition: before either the
    /// cache-hit path or a fresh solve overwrites `self.voltages[node]`,
    /// this captures each node's PRE-resolve bits, then compares against the
    /// POST-resolve bits once whichever path ran has finished — a plain
    /// `f64 != f64` misses a NaN-to-NaN "no-op" (which never legitimately
    /// happens here, `check_finite` rejects NaN outright, but `to_bits`
    /// costs nothing extra and is exactly what "bitwise" calls for anyway).
    fn resolve_dirty(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver]) -> Result<Vec<DeviceId>, String> {
        let mut readers = Vec::new();
        self.resolve_dirty_into(dirty, now, drivers, &mut readers)?;
        Ok(readers)
    }

    fn voltages(&self) -> &[f64] {
        &self.voltages
    }

    fn currents(&self) -> &[f64] {
        &self.currents
    }

    fn restore(&mut self, snapshot: (Vec<f64>, Vec<f64>)) {
        self.voltages = snapshot.0;
        self.currents = snapshot.1;
        // The owner hands its own hold table back after a restore.
        self.hold_requests.clear();
        // The published arrays just jumped to another state: the next resolve
        // recomputes and checks everything rather than trusting which nodes
        // it sees change.
        self.full = true;
    }

    fn set_no_solution_cache(&mut self, off: bool) {
        self.no_solution_cache = off;
    }

    fn cache_hits(&self) -> f64 {
        self.cache_hits
    }

    fn newton_iterations(&self) -> f64 {
        StaticPartitionSolver::newton_iterations(self)
    }

    /// Owner decision 1's "all or nothing": every triple is checked, in
    /// order, against the device it names — out of range, not live, or
    /// failing the kind's own `check` are each an error naming the first
    /// offending triple — before `apply_params` ever touches anything.
    fn validate_params(&self, triples: &[(usize, usize, f64)]) -> Result<(), String> {
        for &(device_index, param_index, value) in triples {
            if !value.is_finite() {
                return Err(format!("device {device_index} param {param_index}: value {value} is not finite"));
            }
            let sw = match self.switch_index_by_device.get(&device_index) {
                Some(&idx) => &self.switches[idx],
                None => return Err(format!("device {device_index}: has no live parameters")),
            };
            let shape = kinds::shape(sw.kind).expect("a stored switch device always has a known kind code");
            if !shape.live_params.contains(&param_index) {
                return Err(format!("device {device_index} ({}): param {param_index} is not live", shape.name));
            }
            if param_index >= sw.params.len() {
                return Err(format!("device {device_index} ({}): param index {param_index} out of range", shape.name));
            }
            let mut new_params = sw.params.clone();
            new_params[param_index] = value;
            // Only switch-family kinds have live params, and none carries a blob.
            (shape.check)(sw.pins.len(), &new_params, &[]).map_err(|msg| format!("device {device_index} ({}): {msg}", shape.name))?;
        }
        Ok(())
    }

    /// Applies an already-`validate_params`-checked batch: mutates every
    /// named device's param, bumps the epoch once (not once per triple —
    /// one `bo_engine_set_params` call is ONE event, owner decision 1) IF
    /// AND ONLY IF at least one triple's value actually differs from what
    /// was already stored, and returns every partition touched,
    /// deduplicated, for the caller (`kernel.rs`) to re-solve as that one
    /// event regardless.
    ///
    /// The "only on an actual change" condition is deliberately narrower
    /// than "every call bumps it": every touched partition is still marked
    /// dirty and re-solved unconditionally (a `bo_engine_set_params` call
    /// always runs its one event, per the contract), but a call that sets a
    /// switch to the value it already holds leaves the electrical system
    /// identical to what the cache already has recorded for it, so nothing
    /// needs to be invalidated. Two toggles back to back that happen to
    /// repeat the same end state (as a harness stimulus replaying a fixed
    /// sequence might) can then still hit the cache on the second one,
    /// instead of every `bo_engine_set_params` call unconditionally forcing
    /// a fresh solve.
    fn apply_params(&mut self, triples: &[(usize, usize, f64)]) -> Vec<PartitionId> {
        let mut dirty = BTreeSet::new();
        let mut changed = false;
        for &(device_index, param_index, value) in triples {
            if let Some(&idx) = self.switch_index_by_device.get(&device_index) {
                if self.switches[idx].params[param_index] != value {
                    changed = true;
                }
                self.switches[idx].params[param_index] = value;
                let sw = &mut self.switches[idx];
                sw.contacts = contacts_of(sw.kind, &sw.pins, &sw.params);
                self.switch_params.insert(device_index, sw.params.clone());
                self.param_dirty.push((self.resistors.len() + idx) as u32);
                for &pin_node in &self.switches[idx].pins {
                    if let Some((p_idx, _)) = self.local_index[pin_node] {
                        dirty.insert(PartitionId(p_idx as u32));
                    }
                }
            }
        }
        if changed {
            self.epoch += 1;
        }
        dirty.into_iter().collect()
    }

    fn param_value(&self, device_index: usize, param_index: usize) -> Option<f64> {
        let &idx = self.switch_index_by_device.get(&device_index)?;
        self.switches[idx].params.get(param_index).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::program::{Device, Options, PinRecord};

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn source(component_index: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: kinds::VOLTAGE_SOURCE, component_index, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }

    fn resistor(component_index: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: kinds::RESISTOR, component_index, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }

    fn switch_dev(component_index: usize, a: usize, b: usize, closed: f64) -> Device {
        Device { kind: kinds::SWITCH, component_index, pins: vec![pin(a), pin(b)], params: vec![closed], blob: vec![] }
    }

    fn spdt(component_index: usize, com: usize, b: usize, c: usize, sel: f64) -> Device {
        Device { kind: kinds::SPDT_SWITCH, component_index, pins: vec![pin(com), pin(b), pin(c)], params: vec![sel], blob: vec![] }
    }

    fn dip(component_index: usize, pins: Vec<usize>, k: usize, sw: [f64; 8]) -> Device {
        let mut params = vec![k as f64];
        params.extend(sw);
        Device { kind: kinds::DIP_SWITCH, component_index, pins: pins.into_iter().map(pin).collect(), params, blob: vec![] }
    }

    fn program(node_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    }

    fn all_partitions(solver: &StaticPartitionSolver) -> Vec<PartitionId> {
        (0..solver.partitions.len() as u32).map(PartitionId).collect()
    }

    /// A closed switch in series with a resistor, across a source: exactly a
    /// two-resistor divider with the switch's 1 mΩ standing in for the
    /// second leg.
    #[test]
    fn a_closed_switch_in_series_with_a_resistor_divides_like_a_tiny_resistor() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 999.999), switch_dev(2, 2, 0, 1.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        // R_switch = 0.001 Ω, R_total ~= 1000 Ω, so node2 ~= 10 * 0.001/1000
        // -- close enough to 0 that this mostly proves it solved at all
        // without a singular matrix, and that the switch's contact really
        // did stamp a conductance (an unstamped/open contact would instead
        // read the source's full 10V at node2, since node2 would then have
        // no path to the datum but the 1e-12 S shunt).
        assert!(solver.voltages()[2].abs() < 0.001, "got {}", solver.voltages()[2]);
        assert!((solver.currents()[2] - solver.currents()[1]).abs() < 1e-9, "series current must match through the switch");
    }

    #[test]
    fn an_open_switch_stamps_nothing_and_the_node_reads_near_the_source_through_the_shunt() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 2, 0, 0.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        // No conductance from node2 to the datum except the 1e-12 S shunt in
        // series with the 1kΩ resistor from the 10V rail: node2 reads
        // essentially the full 10V (the shunt current is negligible).
        assert!((solver.voltages()[2] - 10.0).abs() < 1e-6, "got {}", solver.voltages()[2]);
        assert_eq!(solver.currents()[2], 0.0, "an open switch publishes zero current");
    }

    /// The README's "Pins by id" rule ("a pin id the component lacks maps to
    /// node 0") is applied by the TS compiler before Rust ever sees the
    /// program, but Rust must still handle the result correctly: an
    /// spdt_switch whose `c` pin was absent on the component lands here with
    /// `pins[2].node == 0` (the datum). Selecting that throw must solve
    /// against the datum exactly like any other fixed node, not panic or
    /// treat node 0 specially.
    #[test]
    fn spdt_with_a_missing_pin_mapped_to_the_datum_solves_against_it() {
        // com=2 (free, off the 10V rail through a 1kΩ resistor), b=3 (a
        // free, otherwise-unconnected node), c=0 (the datum -- what a pin id
        // the component lacks resolves to). sel=1 selects c: com is shorted
        // to the datum through the switch's 1 mΩ contact.
        let p = program(4, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), spdt(2, 2, 3, 0, 1.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        // com (node2) is shorted to the datum through 1 mΩ, dominating the
        // 1kΩ feed from the rail: it must read ~0V, not float toward 10V the
        // way an unstamped/open contact would.
        assert!(solver.voltages()[2].abs() < 1e-3, "got {}", solver.voltages()[2]);
        // b's node (3) has no other connection at all: it reads ~0V through
        // the shunt alone, confirming the UNselected contact really did
        // stamp nothing.
        assert!(solver.voltages()[3].abs() < 1e-6, "got {}", solver.voltages()[3]);
        // Essentially the whole loop is 10V over (1kΩ + 1mΩ): ~10mA.
        let i = solver.currents()[2];
        assert!((i - 0.01).abs() < 1e-4, "got {i}");
    }

    #[test]
    fn dip_switch_k_2_solves_two_independent_contacts_through_a_real_program() {
        // Two independent branches off the same 10V rail, each through a
        // 1kΩ resistor into one dip contact straight to the datum: contact 1
        // closed, contact 2 open.
        let p = program(4, vec![
            source(0, 1, 0, 10.0),
            resistor(1, 1, 2, 1000.0),
            resistor(2, 1, 3, 1000.0),
            dip(3, vec![2, 0, 3, 0], 2, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ]);
        // dip pins: a1=2, b1=0 (datum), a2=3, b2=0 (datum).
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        // Contact 1 closed: node2 -> datum through 1kΩ + 1mΩ, so node2
        // reads ~0V (dominated by the 1kΩ leg to the rail).
        assert!(solver.voltages()[2].abs() < 0.01, "got {}", solver.voltages()[2]);
        // Contact 2 open: node3 has no path to the datum except the shunt,
        // so it reads essentially the full 10V through its 1kΩ resistor.
        assert!((solver.voltages()[3] - 10.0).abs() < 1e-6, "got {}", solver.voltages()[3]);
        // Device 3 is the dip switch: its published current is the sum over
        // closed positions only -- contact 1's ~10mA, plus contact 2's 0.
        assert!((solver.currents()[3] - 0.01).abs() < 1e-4, "got {}", solver.currents()[3]);
    }

    #[test]
    fn dip_switch_k_8_solves_eight_independent_contacts_through_a_real_program() {
        let mut nodes = Vec::new();
        let mut devices = vec![source(0, 1, 0, 10.0)];
        let mut next_node = 2usize;
        let mut dip_pins = Vec::new();
        for i in 0..8 {
            let a = next_node;
            next_node += 1;
            devices.push(resistor(1 + i, 1, a, 1000.0));
            dip_pins.push(a);
            dip_pins.push(0); // b_i tied straight to the datum.
            nodes.push(a);
        }
        // All 8 contacts closed.
        devices.push(dip(9, dip_pins, 8, [1.0; 8]));
        let node_count = next_node;
        let p = program(node_count, devices);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        for &a in &nodes {
            assert!(solver.voltages()[a].abs() < 0.01, "node {a} got {}", solver.voltages()[a]);
        }
        // Sum over all 8 (identical) branches.
        let expected_total = 8.0 * (10.0 / 1000.0);
        assert!((solver.currents()[9] - expected_total).abs() < 1e-3, "got {}", solver.currents()[9]);
    }

    // ---- the solution cache (plan row B1) --------------------------------

    #[test]
    fn a_repeated_resolve_with_nothing_changed_is_a_cache_hit_and_bit_identical() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 2, 0, 1.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        let first = solver.voltages().to_vec();
        assert_eq!(solver.cache_hits(), 0.0);

        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        assert_eq!(solver.cache_hits(), 1.0, "an unchanged partition must hit the cache");
        assert_eq!(solver.voltages(), first.as_slice(), "a cache hit must be bit-identical to the fresh solve");
    }

    #[test]
    fn no_solution_cache_bypasses_the_cache_but_matches_it_bit_for_bit() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 2, 0, 1.0)]);
        let parts;
        let with_cache = {
            let mut solver = StaticPartitionSolver::build(&p).unwrap();
            parts = all_partitions(&solver);
            solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
            solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
            assert_eq!(solver.cache_hits(), 1.0);
            solver.voltages().to_vec()
        };
        let without_cache = {
            let mut solver = StaticPartitionSolver::build(&p).unwrap();
            solver.set_no_solution_cache(true);
            solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
            solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
            assert_eq!(solver.cache_hits(), 0.0, "NO_SOLUTION_CACHE must never record a hit");
            solver.voltages().to_vec()
        };
        assert_eq!(with_cache, without_cache, "cache on and cache off must give bit-identical results");
    }

    #[test]
    fn toggling_a_switch_busts_the_cache_and_the_new_value_is_not_reused() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0), switch_dev(2, 2, 0, 0.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        let open_voltage = solver.voltages()[2];

        let dirty = solver.apply_params(&[(2, 0, 1.0)]);
        assert_eq!(dirty, vec![PartitionId(0)]);
        solver.resolve_dirty(&dirty, 0.0, &[]).unwrap();
        let closed_voltage = solver.voltages()[2];

        assert_ne!(open_voltage, closed_voltage);
        assert_eq!(solver.cache_hits(), 0.0, "the epoch bump must prevent a stale hit");
    }

    // ---- set_params validation and application ---------------------------

    #[test]
    fn validate_params_rejects_an_out_of_range_device_index() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        assert!(solver.validate_params(&[(99, 0, 1.0)]).is_err());
    }

    #[test]
    fn validate_params_rejects_a_non_live_device_like_a_resistor() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        // Device 1 is the resistor: it has no live params at all.
        assert!(solver.validate_params(&[(1, 0, 500.0)]).is_err());
    }

    #[test]
    fn validate_params_rejects_a_param_index_not_in_live_params() {
        let p = program(4, vec![source(0, 1, 0, 10.0), dip(1, vec![1, 2, 3, 0], 2, [0.0; 8])]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        // Param 0 is `k`, not live for dip_switch.
        assert!(solver.validate_params(&[(1, 0, 8.0)]).is_err());
    }

    #[test]
    fn validate_params_rejects_a_value_the_kinds_check_function_rejects() {
        let p = program(3, vec![source(0, 1, 0, 10.0), switch_dev(1, 1, 2, 0.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        assert!(solver.validate_params(&[(1, 0, 2.0)]).is_err(), "switch state must be 0 or 1");
    }

    #[test]
    fn validate_params_is_all_or_nothing_one_bad_triple_fails_the_whole_batch() {
        let p = program(3, vec![source(0, 1, 0, 10.0), switch_dev(1, 1, 2, 0.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        // First triple valid, second invalid: the batch as a whole is Err,
        // and (per the caller's contract) nothing gets applied.
        let err = solver.validate_params(&[(1, 0, 1.0), (1, 0, 2.0)]);
        assert!(err.is_err());
    }

    #[test]
    fn apply_params_mutates_bumps_the_epoch_once_and_returns_touched_partitions() {
        let p = program(3, vec![source(0, 1, 0, 10.0), switch_dev(1, 1, 2, 0.0)]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        assert_eq!(solver.param_value(1, 0), Some(0.0));
        let dirty = solver.apply_params(&[(1, 0, 1.0)]);
        assert_eq!(solver.param_value(1, 0), Some(1.0));
        assert_eq!(dirty, vec![PartitionId(0)]);
        assert_eq!(solver.epoch, 1);
    }

    #[test]
    fn param_value_is_none_for_a_device_with_no_live_parameters() {
        let p = program(3, vec![source(0, 1, 0, 10.0), resistor(1, 1, 2, 1000.0)]);
        let solver = StaticPartitionSolver::build(&p).unwrap();
        assert_eq!(solver.param_value(1, 0), None);
        assert_eq!(solver.param_value(99, 0), None);
    }
    // ---- W1e: junction stamping, the polish, the cache (LEDs 6, 7, 9) -----

    use crate::solve::newton::hooks;

    fn seg7_cc(component_index: usize, pins: Vec<PinRecord>) -> Device {
        assert_eq!(pins.len(), 10);
        Device { kind: kinds::SEG7_CC, component_index, pins, params: vec![2.0, 0.02], blob: vec![0.0] }
    }

    /// The drafting probe's worst case (`drled-newton-probe2d.mjs`): a
    /// `seg7_cc` with all eight segments lit through 330 ohm each from 5 V and
    /// a shared 100 ohm common resistor, at -40 C. Nodes: 1 the rail, 2..9
    /// the segment anodes (`a`..`dp`), 10 the common.
    fn all_lit_seg7_at_minus_40() -> Program {
        let mut devices = vec![source(0, 1, 0, 5.0)];
        for i in 0..8 {
            devices.push(resistor(1 + i, 1, 2 + i, 330.0));
        }
        devices.push(resistor(9, 10, 0, 100.0));
        let mut pins: Vec<PinRecord> = (2..10).map(pin).collect();
        pins.push(pin(10)); // com
        pins.push(PinRecord { node: 0, flags: crate::program::PIN_FLAG_OPEN, open_hash: 0, dead_hash: 0 }); // com2
        devices.push(seg7_cc(10, pins));
        let mut p = program(11, devices);
        p.channel_slot_count = 8;
        p.options.ambient_c = -40.0;
        p
    }

    /// The deeply converged fixed point of that circuit, and the warm start
    /// the probe found worst (`ref +- 0.2 V`, alternating), from
    /// `e2b-values.mjs` (Newton run to `|dx| = 0`, independent of the core).
    const POLISH_ANODE: f64 = 4.137971889906031;
    const POLISH_COMMON: f64 = 2.089765111859904;

    fn solve_polish_case(p: &Program) -> (StaticPartitionSolver, f64) {
        let mut solver = StaticPartitionSolver::build(p).unwrap();
        assert_eq!(solver.partitions.len(), 1);
        assert_eq!(solver.partitions[0], (2..=10).collect::<Vec<usize>>(), "the probe's node order");
        for (l, &node) in solver.partitions[0].clone().iter().enumerate() {
            let reference = if node == 10 { POLISH_COMMON } else { POLISH_ANODE };
            solver.voltages[node] = reference + if l % 2 == 1 { 0.2 } else { -0.2 };
        }
        solver.resolve_dirty(&[PartitionId(0)], 0.0, &[]).unwrap();
        let worst = (2..=10)
            .map(|node| (solver.voltages()[node] - if node == 10 { POLISH_COMMON } else { POLISH_ANODE }).abs())
            .fold(0.0f64, f64::max);
        (solver, worst)
    }

    #[test]
    fn the_polish_lands_the_worst_probe_case_within_a_microvolt_of_the_fixed_point() {
        let (solver, worst) = solve_polish_case(&all_lit_seg7_at_minus_40());
        assert!(worst < 1e-6, "polished result is {worst:e} V from the fixed point");
        // The probe's passing iterate came after 9 iterations, and the polish is the 10th.
        assert_eq!(solver.newton_iterations(), 10.0);
    }

    #[test]
    fn with_the_polish_skipped_the_same_case_is_more_than_50_microvolts_off_so_the_test_can_fail() {
        // The seeded fault: without the polish the passing iterate is the
        // published one, 144 uV off (the probe measured 1.4418e-4 V).
        let _guard = hooks::skip_polish();
        let (solver, worst) = solve_polish_case(&all_lit_seg7_at_minus_40());
        assert!(worst > 50e-6, "the seeded fault should show: worst {worst:e} V");
        assert!((worst - 1.4417515579179252e-4).abs() < 1e-9, "and it is the probe's own number: {worst:e}");
        assert_eq!(solver.newton_iterations(), 9.0, "nine iterations to pass, no polish");
    }

    #[test]
    fn a_junction_only_partition_keeps_the_cache_and_a_hit_publishes_the_stored_bits() {
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 220.0), Device {
            kind: kinds::LED,
            component_index: 2,
            pins: vec![pin(2), pin(0)],
            params: vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0],
            blob: vec![],
        }]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        let (first, iterations) = (solver.voltages().to_vec(), solver.newton_iterations());
        assert!(iterations >= 2.0);
        // Disturb the warm start; a hit must not read it.
        solver.voltages[2] = 4.0;
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        assert_eq!(solver.cache_hits(), 1.0);
        assert_eq!(solver.voltages(), first.as_slice(), "the stored vector, bit for bit");
        assert_eq!(solver.newton_iterations(), iterations, "a hit iterates nothing");
    }

    #[test]
    fn a_junction_partition_without_the_cache_re_solves_from_its_last_voltages_and_agrees_within_a_microvolt() {
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 220.0), Device {
            kind: kinds::LED,
            component_index: 2,
            pins: vec![pin(2), pin(0)],
            params: vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0],
            blob: vec![],
        }]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        solver.set_no_solution_cache(true);
        let parts = all_partitions(&solver);
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        let first = solver.voltages()[2];
        let iterations = solver.newton_iterations();
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        assert_eq!(solver.cache_hits(), 0.0);
        assert!(solver.newton_iterations() > iterations, "re-iterated");
        assert!((solver.voltages()[2] - first).abs() < 1e-6);
        // Warm from the fixed point it passes at once: one iteration and the polish.
        assert_eq!(solver.newton_iterations() - iterations, 2.0);
    }

    #[test]
    fn a_partition_that_cannot_converge_is_a_solve_failure_naming_the_iteration_cap() {
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 220.0), Device {
            kind: kinds::LED,
            component_index: 2,
            pins: vec![pin(2), pin(0)],
            params: vec![1.8, 2.0, 0.02, 0.0, 0.0, 0.0],
            blob: vec![],
        }]);
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        let _guard = hooks::force_unconverged(u32::MAX);
        let err = solver.resolve_dirty(&parts, 0.0, &[]).unwrap_err();
        assert!(err.contains("did not converge within 50 iterations"), "{err}");
    }

    /// The packed cache is the `HashMap` it replaced: the same keys hit, the
    /// stored values come back unchanged, and it empties at the same moment
    /// (when an insert finds [`CACHE_ENTRIES_PER_PARTITION`] entries). The
    /// hit/miss pattern is part of the results: a junction partition's
    /// `iterated_solve` starts from the last published voltages, so a hit and
    /// a re-solve can differ in the last bits.
    #[test]
    fn the_packed_cache_hits_and_empties_exactly_like_the_hash_map_it_replaced() {
        let mut cache = PartCache::default();
        let mut reference: HashMap<Vec<u8>, Vec<f64>> = HashMap::new();
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let (mut hits, mut clears) = (0, 0);
        for step in 0..20_000 {
            // A small alphabet over short keys repeats a lot; long ones never do.
            let len = 1 + (next() % 10) as usize;
            let key: Vec<u8> = (0..len).map(|_| (next() % 4) as u8).collect();
            let hash = hash_key(&key);
            let got = cache.find(hash, &key).map(|i| cache.entries[i].x.to_vec());
            assert_eq!(got.as_ref(), reference.get(&key), "step {step}");
            if got.is_some() {
                hits += 1;
            } else {
                let x = vec![step as f64, -(step as f64)];
                if reference.len() >= CACHE_ENTRIES_PER_PARTITION {
                    reference.clear();
                    clears += 1;
                }
                reference.insert(key.clone(), x.clone());
                cache.insert(hash, &key, &x, None);
            }
            assert_eq!(cache.entries.len(), reference.len(), "step {step}");
        }
        assert!(hits > 1000 && clears > 10, "the walk must exercise both hits ({hits}) and the emptying ({clears})");
    }

    /// A rail feeding two resistor legs through switches: the published
    /// currents follow every toggle, every rolled-back toggle and every
    /// re-solve bit for bit as the whole-circuit walk they are computed
    /// incrementally instead of (`resolve_dirty` also asserts this after each
    /// of its own calls in a test build).
    #[test]
    fn the_incremental_currents_match_the_whole_circuit_walk_through_toggles_and_rollbacks() {
        // Device 8 sits across two FIXED nodes (10 V and 3 V), so toggling it
        // moves its own current and the rail's, and no node voltage at all.
        let p = program(
            6,
            vec![
                source(0, 1, 0, 10.0),
                resistor(1, 1, 2, 1000.0),
                switch_dev(2, 2, 3, 1.0),
                resistor(3, 3, 0, 2000.0),
                switch_dev(4, 2, 4, 0.0),
                resistor(5, 4, 0, 500.0),
                resistor(6, 1, 0, 4700.0),
                source(8, 5, 0, 3.0),
                switch_dev(7, 1, 5, 0.0),
            ],
        );
        let mut solver = StaticPartitionSolver::build(&p).unwrap();
        let parts = all_partitions(&solver);
        let check = |solver: &StaticPartitionSolver| {
            let reference = solver.recompute_currents_reference();
            assert_eq!(solver.currents().iter().map(|c| c.to_bits()).collect::<Vec<_>>(), reference.iter().map(|c| c.to_bits()).collect::<Vec<_>>());
        };
        solver.resolve_dirty(&parts, 0.0, &[]).unwrap();
        check(&solver);
        let before = solver.snapshot();

        // Toggle each switch, one at a time and together, re-solving only
        // what the toggle touched.
        for triples in [vec![(2, 0, 0.0)], vec![(8, 0, 1.0)], vec![(4, 0, 1.0)], vec![(2, 0, 1.0), (4, 0, 0.0)], vec![(8, 0, 0.0)], vec![(4, 0, 1.0), (2, 0, 0.0)]] {
            solver.validate_params(&triples).unwrap();
            let dirty = solver.apply_params(&triples);
            solver.resolve_dirty(&dirty, 1e-3, &[]).unwrap();
            check(&solver);
        }
        // A re-solve with nothing dirty leaves them as they are.
        solver.resolve_dirty(&[], 2e-3, &[]).unwrap();
        check(&solver);

        // A rolled-back instant: the published arrays jump back, the params
        // are put back by the caller, and the next resolve is still exact.
        solver.restore(before.clone());
        solver.apply_params(&[(2, 0, 1.0), (4, 0, 0.0)]);
        solver.resolve_dirty(&[], 3e-3, &[]).unwrap();
        check(&solver);
        assert_eq!(solver.voltages().iter().map(|v| v.to_bits()).collect::<Vec<_>>(), before.0.iter().map(|v| v.to_bits()).collect::<Vec<_>>());
    }
}

/// The size of a hub polish iterate's residual `r` (`b - A z` over the
/// partition's `n` rows of the `(n + 1)`-wide residual stamp `a`), which a
/// stalled polish keeps the least of: each row's KCL imbalance over its
/// node's own conductance in the stamp, `max_k |r_k| / |A_kk|`, the volts
/// the node would move to carry it alone. Read off the stamp, not through
/// a factorization, it ranks the iterates the same whichever Jacobian is
/// at hand; a GMIN-held cluster's, near singular, ranks them by its
/// rounding.
fn scaled_residual(r: &[f64], a: &[f64], n: usize) -> f64 {
    let size = n + 1;
    r.iter().enumerate().fold(0.0f64, |acc, (k, v)| acc.max(v.abs() / a[k * size + k].abs()))
}

/// The most a hub polish step (`StaticPartitionSolver::polish_hub`) through
/// `factors` can move from rounding alone: each residual row, `A z - b` over
/// the partition's `n` rows of the `(n + 1)`-wide residual stamp `a`, `b`
/// at `z`, rounds by a few ulps of its gross terms, and the step is that
/// through `J^-1`, so `max_i sum_k |J^-1|_ik r_k`. Solved column by column,
/// only when a step stops contracting.
fn step_rounding(factors: &lu::LuFactorization, a: &[f64], b: &[f64], z: &[f64], n: usize) -> f64 {
    #[cfg(test)]
    newton::hooks::note_stall_solves(n as u64);
    let size = n + 1;
    let gross: Vec<f64> = (0..n).map(|k| 4.0 * f64::EPSILON * (a[k * size..(k + 1) * size].iter().zip(z).map(|(a, z)| (a * z).abs()).sum::<f64>() + b[k].abs())).collect();
    let mut noise = vec![0.0f64; n];
    let mut col = vec![0.0; n];
    for (k, &r) in gross.iter().enumerate() {
        col.fill(0.0);
        col[k] = 1.0;
        if factors.solve(&mut col).is_err() {
            return 0.0;
        }
        for (acc, v) in noise.iter_mut().zip(&col) {
            *acc += v.abs() * r;
        }
    }
    noise.into_iter().fold(0.0, f64::max)
}

// ---- W2a: heap bytes (generated by the size walk generator; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Contact {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for StaticStamp {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pins, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.params, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.contacts, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Term {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Term::Contact { stamp, contact, plus } => crate::snapshot::HeapBytes::heap_bytes(stamp, w) + crate::snapshot::HeapBytes::heap_bytes(contact, w) + crate::snapshot::HeapBytes::heap_bytes(plus, w),
            Term::Junction { junction, plus } => crate::snapshot::HeapBytes::heap_bytes(junction, w) + crate::snapshot::HeapBytes::heap_bytes(plus, w),
            Term::Shunt => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for JunctionMemo {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_anode, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_cathode, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.law_gen, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.current, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.valid, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for StampState {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.failed, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.tripped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.law, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.law_gen, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for CacheEntry {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hash, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.key, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for HubSolve {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.g, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_p, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.band, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for HubSolveSnapshot {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.0, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for StaticHub {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shape, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_owner, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.touch_ics, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nonlinear, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.solve, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PartCache {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.entries, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slots, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for StaticPartitionSolver {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.partitions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.local_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.fixed, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.resolved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.matrix_base, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rhs_base, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.resistors, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.switches, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.switch_index_by_device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.voltages, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.currents, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.no_solution_cache, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.epoch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.caches, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cache_hits, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ics, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds_free_ic_supply, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.junctions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kcl_junctions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.newton_iterations, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_to_ic_readers, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.part_drivers, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ic_drivers, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.driver_index_len, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.part_ics, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ic_of_device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.part_pulls, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.key_scratch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.changed_scratch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stamps_by_node, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.param_dirty, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.work, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stamp_listed, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.resolved_terms, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.junction_memo, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.walk_extra, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.full, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.window, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_log, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i_log, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.source_walk_skippable, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.inputs_bounded, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub_cache_rule, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.walk_skipped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.readers_scratch, w); }
        #[cfg(test)] { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_saved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.live_rhs, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hubs, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.part_waves, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.wave_levels, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.switch_params, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending_bnd, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.republish_hubs, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.silent_len, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub_updates, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sensitivities, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub_journal, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hub_journaled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.exact, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.live, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stamp_of_device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.junction_full, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kcl_active, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kcl_law_gen, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stamp_states, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cache_journal, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cache_journaled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds_on, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.hold_requests, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trial, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.edge_hold, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.w_holds, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.retry_armed, w); }
        n
    }
}
