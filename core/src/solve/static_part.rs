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
use crate::solve::{
    collect_ics, ic_driver_branches, ic_pull_target, junction_branch_current, junction_partition, place_junction, stamp_junctions, DynJunction,
    PartitionSolver,
};

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
    current: f64,
    valid: bool,
}

/// One partition's solution cache entry.
#[derive(Debug)]
struct CacheEntry {
    hash: u64,
    key: Box<[u8]>,
    x: Box<[f64]>,
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
/// `rail_band` (W1f's hub-rail band) was reserved and always 0, so it is not
/// in the packed key. Which keys hit, and when the map empties, are exactly
/// what they were with the `HashMap`: a hit is what a junction partition's
/// `iterated_solve` result depends on (it starts from the last published
/// voltages), so that pattern is part of the results.
#[derive(Debug, Default)]
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

    fn insert(&mut self, hash: u64, key: &[u8], x: &[f64]) {
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
        self.entries.push(CacheEntry { hash, key: key.into(), x: x.into() });
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
fn fold_static_stamp(
    partitions: &[Vec<usize>],
    local_index: &[Option<(usize, usize)>],
    fixed: &[Option<f64>],
    matrix_base: &mut [Vec<f64>],
    rhs_base: &mut [Vec<f64>],
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
        (Some((pu, lu_)), None) => {
            let size = partitions[pu].len();
            matrix_base[pu][lu_ * size + lu_] += g;
            rhs_base[pu][lu_] += g * fixed[w].unwrap();
        }
        (None, Some((pw, lw))) => {
            let size = partitions[pw].len();
            matrix_base[pw][lw * size + lw] += g;
            rhs_base[pw][lw] += g * fixed[u].unwrap();
        }
        (None, None) => {
            // Both ends fixed: no stamp. Current is Ohm's law on two
            // already-known voltages, not a solve.
        }
    }
}

#[derive(Debug)]
pub struct StaticPartitionSolver {
    /// Node ids per partition, in first-seen order (deterministic; contract
    /// point 3 of "W0 kernel semantics").
    partitions: Vec<Vec<usize>>,
    /// node -> (partition index, local index), `None` for a fixed node.
    local_index: Vec<Option<(usize, usize)>>,
    /// node -> fixed voltage, `None` for a free node. Node 0 is always
    /// `Some(0.0)`.
    fixed: Vec<Option<f64>>,
    /// Every source that fixed a node, in resolution order.
    resolved: Vec<ResolvedSource>,
    /// Pristine per-partition conductance matrix: shunt + resistor stamps
    /// only, never a driver's or a switch's. Cloned fresh before every
    /// solve so a driver or switch that has since changed never leaves a
    /// stale stamp behind.
    matrix_base: Vec<Vec<f64>>,
    /// Pristine per-partition right-hand side: resistor-to-fixed-neighbour
    /// stamps only.
    rhs_base: Vec<Vec<f64>>,
    /// Every resistor (immutable after `build`), kept so `recompute_currents`
    /// can recompute its published current.
    resistors: Vec<StaticStamp>,
    /// Every switch-family device, in device order. `params` is mutated in
    /// place by `apply_params` — the only state in this whole struct a live
    /// `bo_engine_set_params` call ever changes.
    switches: Vec<StaticStamp>,
    /// device index -> index into `switches`, for O(1) lookup during
    /// `validate_params`/`apply_params`/`param_value`.
    switch_index_by_device: HashMap<usize, usize>,
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
    caches: Vec<PartCache>,
    /// Contract stat 7.
    cache_hits: f64,
    /// Every combinational IC whose driver/pull pins reach a `Static`
    /// partition (`partition.rs`'s own `-3` decline guarantees every IC's
    /// driver/pull footprint is entirely `Static`-owned this step), by
    /// device index — `solve::collect_ics` over the WHOLE program, not
    /// filtered here, since `Self::stamp_ics_into`'s own per-driver/pull
    /// `in_partition` guard already skips anything that does not touch the
    /// partition being resolved.
    ics: Vec<(usize, digital::IcRecord)>,
    /// partition index -> whether it holds at least one free `vcc`/`gnd`
    /// node of some IC (W1c contract point 3): such a partition's R_out
    /// moves with its own solved state, so `Self::resolve_dirty` iterates it
    /// to a fixed point instead of solving it once and reading (or writing)
    /// the solution cache — see `Self::iterated_solve`.
    holds_free_ic_supply: Vec<bool>,
    /// partition index -> every stamped LED junction touching it, resolved
    /// against it (W1e, LEDs point 4); empty for a partition this solver does
    /// not own and for one holding none. A non-empty entry makes the
    /// partition nonlinear: iterated and polished by `Self::iterated_solve`.
    junctions: Vec<Vec<DynJunction>>,
    /// Every stamped junction this solver owns the current of: those in a
    /// `Static` partition and those across two fixed nodes (which belong to
    /// none). Folded into `Self::recompute_currents`'s source KCL.
    kcl_junctions: Vec<led::Junction>,
    /// Contract stat 11's static half: every iteration of every iterated
    /// partition, the polish and any failed attempt's included. Not rolled
    /// back, like every stat.
    newton_iterations: u64,
    /// node -> every IC device index that reads it (its `vcc`/`gnd` pins and
    /// every logic pin, W1c contract point 4's reader rule) — built once so
    /// `Self::resolve_dirty` can turn "this node's published bits changed"
    /// into "these readers must be queued" by a single lookup per node.
    node_to_ic_readers: Vec<Vec<usize>>,
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
    part_drivers: Vec<Vec<u32>>,
    /// IC index (into `ics`) -> ids of every driver of that IC (`device`
    /// equal to its device index), ascending; derived with `part_drivers`.
    ic_drivers: Vec<Vec<u32>>,
    driver_index_len: usize,
    /// partition index -> indices into `ics` of every IC with a supply or
    /// logic pin free in it, in `ics` order: the only ICs `stamp_ics_into`'s
    /// `touches` test can ever accept for that partition (a driver's or a
    /// pull's node is one of the IC's logic pins), so the stamp visits these
    /// instead of every IC in the program.
    part_ics: Vec<Vec<u32>>,
    /// device index -> index into `ics`, `u32::MAX` for a device that is no IC.
    ic_of_device: Vec<u32>,
    /// partition index -> `(index into `ics`, logic-pin index)` of every
    /// would-be-pulled pin in it (an `OPEN` `FLOATING_INPUT`, non-`OPEN_HIGH`
    /// pin of a static IC record), in the key's fixed enumeration order.
    part_pulls: Vec<Vec<(u32, u32)>>,
    /// The packed cache key being built, reused across calls.
    key_scratch: Vec<u8>,
    /// Nodes whose published bits changed in the current `resolve_dirty`.
    changed_scratch: Vec<usize>,
    /// node -> ids of the stamps (resistors, then switches, as one list) with
    /// a pin on it, ascending, no repeats.
    stamps_by_node: Vec<Vec<u32>>,
    /// Stamps whose params `apply_params` touched since the last resolve.
    param_dirty: Vec<u32>,
    /// Stamps to recompute in the current call, and which are already listed.
    work: Vec<u32>,
    stamp_listed: Vec<bool>,
    /// One per `resolved` source: the terms of its child node's KCL sum.
    resolved_terms: Vec<Vec<Term>>,
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
}

impl StaticPartitionSolver {
    /// Builds the static topology (`partition::build`) and the pristine
    /// per-partition matrix/RHS, but does not solve anything yet — that is
    /// `Kernel::settle`'s job, through `resolve_dirty`, the same way any
    /// later re-solve works.
    pub fn build(program: &Program) -> Result<Self, BoError> {
        let pb = partition::build(program)?;
        let partition::PartitionBuild { partitions, local_index, fixed, resolved, kinds: partition_kinds, .. } = pb;
        let partitions_len = partitions.len();

        let mut partition_junctions: Vec<Vec<DynJunction>> = vec![Vec::new(); partitions.len()];
        let mut kcl_junctions = Vec::new();
        for j in partition::program_junctions(program).into_iter().filter(|j| j.stamped()) {
            match junction_partition(&j, &local_index) {
                Some(p) if partition_kinds[p] == PartitionKind::Static => {
                    partition_junctions[p].push(place_junction(&j, p, &local_index, &fixed).expect("junction_partition just named this partition"));
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
                kinds::RESISTOR if owned_by_static(device) => {
                    let pins: Vec<usize> = device.pins.iter().map(|p| p.node).collect();
                    let contacts = contacts_of(device.kind, &pins, &device.params);
                    for c in &contacts {
                        fold_static_stamp(&partitions, &local_index, &fixed, &mut matrix_base, &mut rhs_base, c.u, c.w, c.g);
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
        let ics_len = ics.len();
        let stamp_count = resistors.len() + switches.len();

        Ok(StaticPartitionSolver {
            partitions,
            local_index,
            fixed,
            resolved,
            matrix_base,
            rhs_base,
            resistors,
            switches,
            switch_index_by_device,
            voltages,
            currents: vec![0.0; program.devices.len()],
            no_solution_cache: program.options.no_solution_cache,
            epoch: 0,
            caches: (0..partitions_len).map(|_| PartCache::default()).collect(),
            cache_hits: 0.0,
            ics,
            holds_free_ic_supply,
            junctions: partition_junctions,
            kcl_junctions,
            newton_iterations: 0,
            node_to_ic_readers,
            part_drivers: vec![Vec::new(); partitions_len],
            ic_drivers: vec![Vec::new(); ics_len],
            driver_index_len: usize::MAX,
            part_ics,
            ic_of_device,
            part_pulls,
            key_scratch: Vec::new(),
            changed_scratch: Vec::new(),
            stamps_by_node,
            param_dirty: Vec::new(),
            work: Vec::with_capacity(stamp_count),
            stamp_listed: vec![false; stamp_count],
            resolved_terms,
            junction_memo,
            walk_extra: vec![0.0; program.node_count],
            full: true,
            window: false,
            v_log: Vec::new(),
            i_log: Vec::new(),
            source_walk_skippable: false,
            inputs_bounded: false,
            walk_skipped: false,
            readers_scratch: Vec::new(),
            #[cfg(test)]
            w_saved: None,
        })
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
        self.v_log.clear();
        self.i_log.clear();
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
            self.part_drivers = part_drivers;
            self.ic_drivers = ic_drivers;
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
            _ => self.fixed[node].expect("an IC pin's node reachable from a Static partition must be fixed or free within it (partition.rs's union rule / W1c step 2's -3 decline)"),
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
                b[lu_] += g * self.fixed[w].expect("the far end of a within-partition stamp must be fixed when it is not itself free in this partition");
            }
            (_, Some((pw, lw))) if pw == p_idx => {
                a[lw * size + lw] += g;
                b[lw] += g * self.fixed[u].expect("the far end of a within-partition stamp must be fixed when it is not itself free in this partition");
            }
            _ => {}
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
            let hi_v = self.fixed.get(driver.hi_node).copied().flatten();
            let lo_v = self.fixed.get(driver.lo_node).copied().flatten();
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
    fn stamp_ics_into(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], now: f64, drivers: &[Driver], x: &[f64]) {
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
            let power = digital::power::ic_power(ic, v_vcc, v_gnd);
            if !power.powered {
                continue;
            }
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
                for branch in ic_driver_branches(&power, driver, ic) {
                    if let Some((target, g)) = branch {
                        self.fold_local(p_idx, size, a, b, driver.node, target, g);
                    }
                }
            }
            for pin in &ic.logic {
                if !self.in_partition(pin.node, p_idx) {
                    continue;
                }
                if let Some((g, target_v)) = ic_pull_target(ic, &power, pin, now) {
                    self.fold_pull(p_idx, size, a, b, pin.node, g, target_v);
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
        self.stamp_non_ic_drivers_into(p_idx, size, &mut a, &mut b, drivers)?;
        self.stamp_switches_into(p_idx, size, &mut a, &mut b);
        self.stamp_ics_into(p_idx, size, &mut a, &mut b, now, drivers, &[]);
        lu::solve(a, size, b).map_err(|_| format!("partition {p_idx} ({size} node(s)): singular or non-finite pivot"))
    }

    /// One iteration's stamp for an iterated partition (`Self::iterated_solve`):
    /// the pristine base, then every non-IC driver, switch contact, IC
    /// driver and pull (from the iterate's supply voltages when the partition
    /// holds a free supply, W1c point 3) and LED junction companion (about
    /// the iterate, W1e LEDs point 2). Classic restamps all of these from the
    /// latest iterate with one Newton loop; this is that loop restricted to
    /// one partition (LEDs points 9 and 11).
    fn stamp_iterate(&self, p_idx: usize, size: usize, a: &mut [f64], b: &mut [f64], now: f64, drivers: &[Driver], x: &[f64]) -> Result<(), String> {
        a.copy_from_slice(&self.matrix_base[p_idx]);
        b.copy_from_slice(&self.rhs_base[p_idx]);
        self.stamp_non_ic_drivers_into(p_idx, size, a, b, drivers)?;
        self.stamp_switches_into(p_idx, size, a, b);
        self.stamp_ics_into(p_idx, size, a, b, now, drivers, x);
        stamp_junctions(&self.junctions[p_idx], size, a, b, x);
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
        let size = self.partitions[p_idx].len();
        let x0: Vec<f64> = self.partitions[p_idx].iter().map(|&n| self.voltages[n]).collect();
        let polish = !self.junctions[p_idx].is_empty();
        // A closure cannot return the stamp's own error (`Newton` wants
        // `()`), so it parks the first one here and leaves the matrix zero,
        // which fails the solve at once instead of iterating on garbage.
        let mut stamp_error: Option<String> = None;
        let result = newton::solve(size, size, &x0, false, polish, |x, a, b| {
            if stamp_error.is_some() {
                return;
            }
            if let Err(e) = self.stamp_iterate(p_idx, size, a, b, now, drivers, x) {
                a.fill(0.0);
                b.fill(0.0);
                stamp_error = Some(e);
            }
        });
        if let Some(e) = stamp_error {
            return Err(e);
        }
        let result = result.map_err(|e| format!("partition {p_idx} ({size} node(s)): {e}"))?;
        // Counted before the verdict, like every stat: a failed attempt ran.
        self.newton_iterations += result.iterations as u64;
        if !result.converged {
            return Err(format!("partition {p_idx} ({size} node(s)): Newton did not converge within {} iterations", newton::NEWTON_MAX));
        }
        Ok(result.x)
    }

    /// [`PartitionSolver::resolve_dirty`]'s body; `changed` collects every
    /// node whose published bits moved, and the readers of those nodes are
    /// appended to `out`, ascending and without repeats.
    fn resolve_dirty_inner(&mut self, dirty: &[PartitionId], now: f64, drivers: &[Driver], changed: &mut Vec<usize>, out: &mut Vec<DeviceId>) -> Result<(), String> {
        for &partition in dirty {
            let p_idx = partition.0 as usize;
            let size = self.partitions[p_idx].len();
            if size == 0 {
                continue;
            }

            if self.holds_free_ic_supply[p_idx] {
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
                        self.caches[p_idx].insert(hash, &self.key_scratch, &x);
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
        for &node in changed.iter() {
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
        let j = &self.kcl_junctions[k];
        let (va, vk) = (self.voltages[j.anode].to_bits(), self.voltages[j.cathode].to_bits());
        let memo = &mut self.junction_memo[k];
        if !(memo.valid && memo.v_anode == va && memo.v_cathode == vk) {
            *memo = JunctionMemo { v_anode: va, v_cathode: vk, current: junction_branch_current(j, &self.voltages), valid: true };
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
        for j in &self.kcl_junctions {
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
                cache.insert(hash, &key, &x);
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
