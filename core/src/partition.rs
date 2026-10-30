//! Partitioning and classification (core/README.md, "W0 kernel semantics"
//! points 1-3; W1 plan section 3.2, rows B1/B2/B3). Fixed-node derivation,
//! the iterative union-find over free nodes, and classifying each resulting
//! partition as static, closed-form or stepped (Decision 14): `solve::mod`'s
//! `PartitionedSolver` (W1b step 2) uses [`PartitionBuild::kinds`] to decide,
//! per partition, whether `solve/static_part.rs`, `solve/closed_form.rs` or
//! `solve/stepped.rs` owns it. `FORCE_STEPPED` (core/README.md's Options
//! table, key 4 — tests only) is applied here, once, by [`build`]: every
//! `ClosedForm` partition becomes `Stepped` instead, so every downstream
//! consumer of `PartitionBuild::kinds` sees the final classification
//! directly and never has to re-check the option itself.
//!
//! Moved here from `engine.rs`'s former `StaticPartitionSolver::build`
//! (W0/W1a): fixed-node propagation from node 0 and the iterative,
//! union-by-size union-find over free nodes are unchanged. What is new:
//! capacitors and switch-family contacts also join the union (so a
//! capacitor-only link between two nodes still lands them in one partition
//! to classify), and every switch-family contact is unioned
//! UNCONDITIONALLY, regardless of whether it is open or closed right now.
//!
//! That last rule is deliberate, not an oversight: unlike a resistor's `R`,
//! a switch's `closed`/`sel`/`sw_i` can change after load
//! (`bo_engine_set_params`, owner decision 1), and nothing ever
//! re-partitions after load — partition membership is decided once, here,
//! at `bo_engine_load` time. If an open switch's pins were only unioned
//! while closed, toggling it could move a node from one partition to
//! another (or split/merge whole partitions) behind the kernel's back,
//! which nothing downstream (`Kernel::add_driver`'s fixed partition
//! assignment, the solution cache's per-partition keying) is built to
//! handle. Unioning unconditionally means a live toggle only ever changes
//! what gets STAMPED into an already-fixed partition
//! (`solve/static_part.rs`), exactly like a driver's commit already does —
//! never which partition a node belongs to. The cost is that an open
//! switch's two nodes still share a (possibly larger) dense partition with
//! whatever else the switch would connect if closed; correctness is
//! unaffected, since a zero-conductance contact contributes nothing to that
//! shared system either way.
//!
//! **A switch-family device's free pins are unioned as a WHOLE, not contact
//! by contact.** A single contact's own two ends (README point 1's second
//! bullet) is not enough: an `spdt_switch`'s `com` can sit on a fixed pin (a
//! source) while `b` and `c` are both free, so neither the `com`-`b` nor the
//! `com`-`c` contact ever has two free ends to union — yet `b` and `c` are
//! still the same device's two alternatives for the SAME current path, and
//! `solve::mod`'s single-owner dispatch (`owner_of`, `owned_by_static`,
//! `PartitionSolver::apply_params`'s dirtying) and `compile_dyn_shape`'s
//! per-partition `resolve` both depend on a device's free pins landing in
//! exactly one partition, no matter which pin happens to be fixed. So `build`
//! below unions every free pin of a switch-family device with every other
//! free pin of that SAME device directly — not only pairwise by contact —
//! whenever the device has two or more free pins at all. This subsumes the
//! plain "both ends of one contact, when both are free" rule as a special
//! case (a contact's two pins are just two of the device's own pins), and it
//! is what makes it safe for `compile_dyn_shape::resolve` to treat any other
//! partition's free node as unreachable, rather than a case it has to
//! silently paper over.
//!
//! **LED junctions (W1e, README "Kernel semantics: LEDs and displays" points
//! 4 and 5) are unioned junction by junction, not device by device.** A
//! stamped junction (its anode and cathode nodes differ) joins its two nodes
//! when both are free, exactly as a resistor does, and a device's other pins
//! join nothing through it: a `bicolor_led`, `rgb_led` or `seg7` whose common
//! pin is fixed puts each channel in the partition of its own drive. So an
//! LED-family device can touch several partitions, and nothing downstream may
//! assume one owner for it: `compile_dyn_shape` and `static_part` place
//! junctions by node. A partition holding a stamped junction is nonlinear
//! ([`PartitionBuild::holds_junction`]): static when it holds no counted
//! capacitor, stepped otherwise, never closed-form. A junction across two
//! fixed nodes belongs to no partition, and a `burnt` device has none.
//!
//! **Two W1e follow-up rules keep an IC's supply and its pins in step.** A
//! `ClosedForm` partition holding a watched read pin of an IC whose supply is
//! free in a `Stepped` partition is promoted to `Stepped`
//! ([`is_watched_read_pin`]): a closed-form crossing is solved once for one
//! threshold, and that supply moves it continuously. And an IC with both
//! supplies free whose output-capable pin sits on a fixed, non-datum node has
//! its two supplies unioned ([`drives_a_fixed_node`]): that driver has no
//! partition of its own, `devices::ics::register` dirties its free supply's,
//! and a driver dirties exactly one.

use crate::devices::battery::{clamp_soc, BatteryModel};
use crate::devices::led::{self, Junction};
use crate::devices::sources::{Clock, Pulse};
use crate::digital;
use crate::error::BoError;
use crate::kinds;
use crate::program::{Device, Program};
use crate::rails::RAIL_TAU_MAX;

/// One entry per source that fixed a node during propagation, in the order
/// it was resolved. `parent_node` was already fixed at the moment `source`
/// (a device index) fixed `child_node`; `parent_is_pos` says whether
/// `parent_node` is that source's `pins[0]` (+) or `pins[1]` (-). Recorded so
/// the current pass can walk the propagation forest children-before-parents
/// (contract, point 5) by simply reversing this list.
#[derive(Debug, Clone)]
pub struct ResolvedSource {
    pub device_index: usize,
    pub parent_node: usize,
    pub child_node: usize,
    pub parent_is_pos: bool,
}

/// How a partition's own devices classify it (Decision 14, W1 plan row B1;
/// the `FORCE_STEPPED` override, row B2/B3, is applied by [`build`] before
/// this ever reaches a caller). Kept as a real three-way enum — not a bool —
/// so `solve::mod`'s `PartitionedSolver` can match on it directly instead of
/// re-deriving the classification from a capacitor count a second time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionKind {
    /// No capacitor at all: a pure conductance network, solved by
    /// `solve/static_part.rs` every time it is dirtied.
    Static,
    /// Exactly one capacitor: eligible for the analytic segment/crossing
    /// solver (Decision 14, `solve/closed_form.rs`) — unless `FORCE_STEPPED`
    /// is set, in which case [`build`] reclassifies it as `Stepped` instead
    /// (tests only, core/README.md's Options table key 4).
    ClosedForm,
    /// More than one capacitor, or any capacitor beside a nonlinear element
    /// (a free IC supply node, a 74LS245 `a`/`b` pin, a stamped LED
    /// junction), or any `ClosedForm` partition under `FORCE_STEPPED`: the
    /// general stepped BE/trap path (`solve/stepped.rs`).
    Stepped,
}

fn classify(capacitor_count: usize) -> PartitionKind {
    match capacitor_count {
        0 => PartitionKind::Static,
        1 => PartitionKind::ClosedForm,
        _ => PartitionKind::Stepped,
    }
}

/// The result of partitioning: everything a solver needs to build its own
/// per-partition state, without redoing fixed-node derivation or
/// union-find. `kinds` is parallel to `partitions`, with `FORCE_STEPPED`
/// already folded in — a caller that only ever handles
/// [`PartitionKind::Static`] (there is none such left in this crate after
/// W1b step 2, but a future standalone tool might) can still ignore the
/// field entirely if it prefers.
pub struct PartitionBuild {
    /// Node ids per partition, in first-seen order (deterministic; contract
    /// point 3 of "W0 kernel semantics").
    pub partitions: Vec<Vec<usize>>,
    /// node -> (partition index, local index), `None` for a fixed node.
    pub local_index: Vec<Option<(usize, usize)>>,
    /// node -> fixed voltage, `None` for a free node. Node 0 is always
    /// `Some(0.0)`.
    pub fixed: Vec<Option<f64>>,
    /// Every source that fixed a node, in resolution order.
    pub resolved: Vec<ResolvedSource>,
    pub kinds: Vec<PartitionKind>,
    /// How many partitions have >= 2 capacitors — real multi-capacitor
    /// linear partitions, counted from the ORIGINAL capacitor count before
    /// `FORCE_STEPPED` could reclassify a single-capacitor `ClosedForm`
    /// partition into `Stepped` too (so this count does not move under that
    /// option). W1b's report to the orchestrator: this is a candidate for a
    /// new `bo_engine_stat` index 15, not yet in the ABI contract table
    /// (core/README.md's "Stat indices") — `engine.rs` already exposes it at
    /// that index, pending the orchestrator adding the row.
    pub multi_capacitor_partitions: usize,
    /// Per partition: whether it holds a stamped LED junction (README, LEDs
    /// point 5), so it is nonlinear. Such a partition is never `ClosedForm`;
    /// with no counted capacitor it is `Static` and iterated.
    pub holds_junction: Vec<bool>,
    /// The hub rail, when the program has one (README, Rails point 1).
    pub rail: Option<RailTopology>,
    /// node -> whether its boundary value can change after load: the rail
    /// (outside exact mode), and every node a waveform source fixes, directly
    /// or through a source stacked on it (Rails point 7).
    pub live: Vec<bool>,
    /// node -> whether a waveform source with a resolved ramp moves it
    /// (Sources point 3).
    pub ramp_node: Vec<bool>,
    /// Per partition: the non-free nodes its stamps read, ascending.
    pub reads: Vec<Vec<usize>>,
    /// Per partition: whether it reads the rail node (a hub partition, Rails
    /// point 2). Always false in exact mode, where the rail is free.
    pub hub: Vec<bool>,
    /// Per partition: whether it is a closed-form hub partition on a bench
    /// PSU's rail with no rail capacitor, which steps with the rail as an
    /// unknown while the PSU is in CC and is a closed form again in CV
    /// (ruling R59 (1) (B), `solve::PartitionedSolver::sync_cc_kinds`). Its
    /// `kinds` entry is `ClosedForm`, the kind it loads in.
    pub cc_switch: Vec<bool>,
}

/// The hub rail's topology (README, Rails point 1).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RailTopology {
    /// The hub source's device index.
    pub device: usize,
    /// Its `pos` node, the rail.
    pub rail: usize,
    /// Its `neg` node, the return: node 0 or an anchor node.
    pub ret: usize,
    /// `NO_RAIL_SHORTCUT`: the rail is a free node (Rails point 9).
    pub exact: bool,
    /// The rail's published value before its first solve: the open-circuit
    /// voltage `E(soc0)` of a battery, `V` of a PSU.
    pub u0: f64,
}

/// Whether a device is an ideal source in the fixed-node propagation: a
/// `voltage_source`, a `bench_psu` with no current limit (`iLimit = 0` is
/// code 2's path, README "Bench PSU record"), and the waveform sources.
pub(crate) fn is_ideal_source(device: &Device) -> bool {
    match device.kind {
        kinds::VOLTAGE_SOURCE | kinds::CLOCK | kinds::CLOCK_GEN | kinds::PULSE_SOURCE | kinds::PULSE_GEN => true,
        kinds::BENCH_PSU => !(device.params[1] > 0.0),
        _ => false,
    }
}

/// Whether a device is a hub candidate: a battery, or a current-limited
/// bench PSU (Rails point 1).
pub(crate) fn is_hub_candidate(device: &Device) -> bool {
    match device.kind {
        kinds::BATTERY_PACK => true,
        kinds::BENCH_PSU => device.params[1] > 0.0,
        _ => false,
    }
}

pub(crate) fn is_waveform(kind: u32) -> bool {
    matches!(kind, kinds::CLOCK | kinds::CLOCK_GEN | kinds::PULSE_SOURCE | kinds::PULSE_GEN)
}

/// An ideal source's value at load: the param of a constant source, the
/// level at `START_TIME` of a waveform source (Sources point 4).
pub(crate) fn source_value_at_load(device: &Device, start_time: f64) -> f64 {
    match device.kind {
        kinds::CLOCK | kinds::CLOCK_GEN => Clock::from_params(&device.params).load_state(start_time).0.value,
        kinds::PULSE_SOURCE | kinds::PULSE_GEN => Pulse::from_params(&device.params).load_state(start_time).0.value,
        _ => device.params[0],
    }
}

/// Whether a waveform source has a resolved ramp (Sources point 3).
fn has_resolved_ramp(device: &Device) -> bool {
    matches!(device.kind, kinds::PULSE_SOURCE | kinds::PULSE_GEN) && Pulse::from_params(&device.params).has_resolved_ramp()
}

/// Every LED-family junction of the program, in device order and, within a
/// device, in channel order (`devices::led::junctions`, at the program's
/// ambient temperature). Self-shorted junctions are listed (their channel
/// slot stays fixed) and report `stamped() == false`.
pub fn program_junctions(program: &Program) -> Vec<Junction> {
    program.devices.iter().enumerate().flat_map(|(d, device)| led::junctions(d, device, program.options.ambient_c)).collect()
}

/// The one pin-index pair (into a device's own `pins`) a resistor or
/// capacitor can ever carry a conductance between, for topology purposes
/// only — never for the actual electrical stamp, which reads the device's
/// CURRENT parameters (`solve/static_part.rs`, `devices::stamp`). A
/// switch-family kind is unioned by [`build`] itself instead (this module's
/// doc comment on why its free pins are unioned as a whole device, not pair
/// by pair the way a resistor's or capacitor's single pair is).
fn resistor_or_capacitor_pair(kind: u32) -> Option<(usize, usize)> {
    match kind {
        // W1g: a fuse and a PTC are two-terminal resistances as a resistor is
        // (W0 point 3). A latched fuse stamps nothing, but membership is fixed
        // at load (W1g point 12), so its two ends stay joined.
        kinds::RESISTOR | kinds::CAPACITOR | kinds::FUSE | kinds::PTC_FUSE => Some((0, 1)),
        _ => None,
    }
}

/// Builds the static topology: fixed-node propagation from node 0
/// ([`derive_fixed_nodes`]), then union-find over free nodes joined by every
/// resistor or capacitor whose two ends are both free, and by every free pin
/// of a switch-family device with every other free pin of that same device
/// (this module's doc comment explains why the whole device, not contact by
/// contact — a switch-family contact can run through a fixed pin), then
/// classifies each resulting partition (Decision 14, `FORCE_STEPPED` applied
/// here). A stamped LED junction with both nodes free also joins them (W1e,
/// this module's doc comment), and a partition holding one is nonlinear:
/// static with no counted capacitor, stepped otherwise, never closed-form. A
/// capacitor with BOTH terminals fixed belongs to no partition
/// either — not a resistive network `solve/static_part.rs` can solve, and
/// not a free-node dynamical system `solve/closed_form.rs`/`solve/stepped.rs`
/// integrate — but, unlike a loop of ideal sources, this is NOT
/// `ERR_UNSUPPORTED_TOPOLOGY` (-3): core/README.md's rule 2a (a decoupling
/// capacitor straight across an ideal supply) gives it an exact closed form
/// of its own, `solve::mod`'s `PartitionedSolver` owns it directly (see that
/// module's `FixedPairCapacitor`), and this function simply excludes it from
/// every partition's `capacitor_count` below, the same way a self-shorted
/// capacitor (both pins the SAME node, free or fixed — contract point 2) is
/// excluded.
pub fn build(program: &Program) -> Result<PartitionBuild, BoError> {
    let n = program.node_count;
    let (mut fixed, mut resolved, anchor_nodes) = derive_fixed_nodes(program, n)?;
    let rail = classify_hub(program, &fixed, &resolved, &anchor_nodes)?;
    if let Some(r) = rail {
        // The rail is neither fixed nor free (Rails point 1): outside exact
        // mode the union-find skips it as it skips a fixed node, so every
        // `fixed[node].is_none()` test below reads it as not free, and a
        // stamp reads it through the live boundary. In exact mode it is free.
        if !r.exact {
            fixed[r.rail] = Some(r.u0);
        }
        // The hub joins the propagation forest as the source of its rail, so
        // the source-current walk computes its current by KCL at the rail
        // (Rails point 5: `-I` in CV, and the battery's branch current).
        resolved.push(ResolvedSource { device_index: r.device, parent_node: r.ret, child_node: r.rail, parent_is_pos: false });
    }

    // Every free IC supply (`vcc`/`gnd`) node — used below to promote a
    // ClosedForm partition that holds one to Stepped (contract point 3).
    // Collected in the SAME pass that unions an IC's driver/pull pins with
    // it, so this pass never has to re-decode an IC's pin flags a second
    // time. `fixed` is all this pass needs (not `local_index`, built after
    // every union below), matching the switch-family union loop's own
    // "topology only needs `fixed`" shape.
    let mut ic_free_supply_nodes: Vec<usize> = Vec::new();

    let mut uf = UnionFind::new(n);
    // LEDs point 4: junction by junction, as a resistor's two ends are
    // joined. Kept for the classification below.
    let junctions = program_junctions(program);
    for j in &junctions {
        if j.stamped() && fixed[j.anode].is_none() && fixed[j.cathode].is_none() {
            uf.union(j.anode, j.cathode);
        }
    }
    for device in &program.devices {
        if let Some((pa, pb)) = resistor_or_capacitor_pair(device.kind) {
            let (u, w) = (device.pins[pa].node, device.pins[pb].node);
            if fixed[u].is_none() && fixed[w].is_none() {
                uf.union(u, w);
            }
            continue;
        }
        if matches!(device.kind, kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH) {
            // Every free pin of this switch-family device joins the SAME
            // partition, however many contacts separate them and regardless
            // of which of the device's other pins are fixed (this module's
            // doc comment) — union the first free pin found with every other
            // one, which is enough to merge them all under union-by-size.
            let mut free_pins = device.pins.iter().map(|p| p.node).filter(|&node| fixed[node].is_none());
            if let Some(first) = free_pins.next() {
                for node in free_pins {
                    uf.union(first, node);
                }
            }
            continue;
        }
        if !digital::is_ic(device.kind) {
            continue;
        }
        // W1c contract point 2: "the union-find also joins each free
        // output-capable pin, and each free OPEN FLOATING_INPUT pin, with
        // every free supply pin of the same IC." `IcRecord::from_device` is
        // safe to call unconditionally here: `program.rs`'s reader has
        // already validated this device against `kinds::shape`'s IC shape
        // (pin/param/blob counts, flag codes) before `partition::build` ever
        // runs (`digital::mod`'s own doc comment on `from_device`). W1d
        // widened this from combinational-only to every IC kind
        // (`digital::is_ic`): a sequential/memory IC's own output-capable
        // and floating-input pins join its supply exactly the same way.
        let ic = digital::IcRecord::from_device(device);
        let free_supply: Vec<usize> =
            [ic.vcc.node, ic.gnd.node].into_iter().filter(|&node| fixed[node].is_none()).collect();
        ic_free_supply_nodes.extend(&free_supply);

        let output_capable = digital::output_capable(ic.kind);
        let free_driver_pull_nodes = output_capable
            .iter()
            .map(|&i| ic.logic[i].node)
            // Point 6's pull never applies to an OPEN_HIGH pin ("IC_OPEN_HIGH_PINS
            // gets no pull at all"), so such a pin never stamps anything and
            // has no business joining this union either — matching
            // `digital::floating::pull`'s own eligibility test exactly (minus
            // its `powered` check, which is a runtime fact, not a topology
            // one: partition membership is fixed at load regardless of
            // whether the IC happens to be powered at that instant).
            .chain(ic.logic.iter().filter(|p| p.open && p.floating_input && !p.open_high).map(|p| p.node))
            .filter(|&node| fixed[node].is_none());
        for node in free_driver_pull_nodes {
            for &supply in &free_supply {
                uf.union(supply, node);
            }
        }
        // W1e follow-up, gap 2: an output-capable pin on a fixed, non-datum
        // node is not in any partition, so nothing above joined it to the
        // supply, yet its driver stamps `1/R_out` from that fixed node into
        // whichever supply is free (HIGH into `vcc`, LOW into `gnd`, and a
        // 555's OUT into both). `devices::ics::register` hands such a driver
        // the partition of its free supply, and a driver dirties exactly one
        // partition, so with both supplies free they must share it or a
        // commit that moves the stamp from `vcc` to `gnd` would re-solve
        // only one side. A pin on the datum stamps nothing (finding 9), so
        // it never needs this.
        if free_supply.len() == 2 && output_capable.iter().any(|&i| drives_a_fixed_node(&ic, i, &fixed)) {
            uf.union(free_supply[0], free_supply[1]);
        }
    }

    // Group free nodes into partitions by union-find representative,
    // preserving first-seen order for determinism (node id order, not
    // representative id order — the latter depends on union direction).
    let mut partition_of: Vec<Option<usize>> = vec![None; n];
    let mut partitions: Vec<Vec<usize>> = Vec::new();
    for node in 1..n {
        if fixed[node].is_some() {
            continue;
        }
        let root = uf.find(node);
        let idx = match partition_of[root] {
            Some(idx) => idx,
            None => {
                let idx = partitions.len();
                partitions.push(Vec::new());
                partition_of[root] = Some(idx);
                idx
            }
        };
        partitions[idx].push(node);
    }

    let mut local_index: Vec<Option<(usize, usize)>> = vec![None; n];
    for (p_idx, nodes) in partitions.iter().enumerate() {
        for (l_idx, &node) in nodes.iter().enumerate() {
            local_index[node] = Some((p_idx, l_idx));
        }
    }

    let mut capacitor_count = vec![0usize; partitions.len()];
    for device in &program.devices {
        if device.kind != kinds::CAPACITOR {
            continue;
        }
        let (u, w) = (device.pins[0].node, device.pins[1].node);
        if u == w {
            // Self-shorted (contract point 2): both terminals land on the
            // SAME node, free or fixed (gnd included) — its companion stamps
            // cancel exactly like classic's `stampConductance` with `i==j`,
            // so it stamps nothing, publishes 0 and must not count toward
            // this node's partition classification (an otherwise-Static
            // node must not be pulled into ClosedForm/Stepped by a capacitor
            // that never actually contributes to the matrix), nor can it
            // ever be the DISTINCT-fixed-nodes case just below — that one is
            // for a capacitor with no free terminal AT ALL, not one whose
            // single shared terminal happens to be fixed.
            continue;
        }
        match (local_index[u], local_index[w]) {
            (Some((p, _)), _) | (_, Some((p, _))) => capacitor_count[p] += 1,
            // Rule 2a: both terminals are FIXED, DISTINCT nodes (a
            // decoupling capacitor straight across an ideal supply — not
            // rule 2's self-shorted case, already handled above). It
            // belongs to no partition and is not `ERR_UNSUPPORTED_TOPOLOGY`:
            // its terminal voltage `dV` never moves (no fixed-node voltage
            // is live in W1b), so it evolves on its own, exactly.
            // `solve::mod::PartitionedSolver` is the only place that ever
            // looks at this device again.
            (None, None) => {}
        }
    }

    // Decision 14 (W1 plan row B2/B3): classify by capacitor count, then fold
    // in `FORCE_STEPPED` — a test-only override, core/README.md's Options
    // table key 4 — which sends every single-capacitor `ClosedForm`
    // partition down the general stepped path instead, so the same fixture
    // can be run both ways to compare the exact closed form against the
    // BE/trap step-doubling controller (W1 plan row B3's verify step: "the
    // RC/RL discrete-conservation benchmarks under `FORCE_STEPPED`").
    // `Stepped` (already >= 2 capacitors, or nonlinear once W1c+ adds a
    // nonlinear kind) is left alone: forcing an already-stepped partition to
    // itself is a no-op.
    let multi_capacitor_partitions = capacitor_count.iter().filter(|&&c| c >= 2).count();

    // W1c contract point 3: "A dynamic partition holding a free IC supply
    // node is stepped" — R_out then moves with that partition's own solved
    // state, which only the general BE/trap path (never the exact
    // closed-form one) can re-stamp from. Computed from `ic_free_supply_nodes`
    // (collected in the union pass above) against the now-final `local_index`,
    // same shape as `capacitor_count`'s own per-partition accumulation.
    // A dynamic partition that must be stepped rather than closed-form
    // (W1c point 3): it holds a free IC supply node, whose R_out moves with
    // the partition's state, or a 74LS245 a/b pin (below), or (W1e, LEDs
    // point 5) a stamped junction: the exact exponential needs a linear
    // network, so an LED beside one capacitor is stepped.
    let mut must_step = vec![false; partitions.len()];
    let mut holds_junction = vec![false; partitions.len()];
    for j in junctions.iter().filter(|j| j.stamped()) {
        // A junction across two fixed nodes matches neither arm: it belongs
        // to no partition. With one free end the junction sits in that end's
        // partition, and with two the union above made them the same one.
        for node in [j.anode, j.cathode] {
            if let Some((p, _)) = local_index[node] {
                holds_junction[p] = true;
                must_step[p] = true;
            }
        }
    }
    for &node in &ic_free_supply_nodes {
        if let Some((p, _)) = local_index[node] {
            must_step[p] = true;
        }
    }
    // A 74LS245's a/b pins (and, W1d, an EEPROM's io0..io7) are read AND
    // driven, so the closed-form crossing watch cannot cover them: watching
    // a pin its own IC drives makes the crossing solve divide by a
    // near-zero coupling (solve/mod.rs, `register_ic_read_pin_watches`). A
    // stepped partition re-reads its ICs after every step instead, so a
    // partition holding one is stepped (core/README.md, W1c point 3 /
    // W1d memories point 1: "the EEPROM's io pins ... follow the 74ls245
    // rule").
    for device in &program.devices {
        if !digital::is_read_and_driven(device.kind) {
            continue;
        }
        let ic = digital::IcRecord::from_device(device);
        for &i in digital::output_capable(ic.kind) {
            if let Some((p, _)) = local_index[ic.logic[i].node] {
                must_step[p] = true;
            }
        }
    }

    let force_stepped = program.options.force_stepped;
    let mut kinds_by_partition: Vec<PartitionKind> = capacitor_count
        .into_iter()
        .map(classify)
        .enumerate()
        .map(|(p, kind)| {
            if kind == PartitionKind::ClosedForm && (force_stepped || must_step[p]) {
                PartitionKind::Stepped
            } else {
                kind
            }
        })
        .collect();

    // W1f: which non-free nodes can move after load (Rails point 7), and
    // which a resolved ramp moves (Sources point 3), walking the forest in
    // resolution order so a source stacked on a moving node moves too.
    let mut live = vec![false; n];
    let mut ramp_node = vec![false; n];
    for r in &resolved {
        let device = &program.devices[r.device_index];
        if rail.is_some_and(|t| t.device == r.device_index) {
            continue;
        }
        live[r.child_node] = live[r.parent_node] || is_waveform(device.kind);
        ramp_node[r.child_node] = ramp_node[r.parent_node] || has_resolved_ramp(device);
    }
    if let Some(t) = rail {
        if !t.exact {
            live[t.rail] = true;
        }
    }
    // A capacitor or LED junction across a ramping node and another non-free
    // node would follow the ramp in a closed form nobody ported (Sources
    // point 3).
    for device in &program.devices {
        if device.kind != kinds::CAPACITOR {
            continue;
        }
        let (u, w) = (device.pins[0].node, device.pins[1].node);
        if u != w && fixed[u].is_some() && fixed[w].is_some() && (ramp_node[u] || ramp_node[w]) {
            return Err(BoError::unsupported_topology(format!(
                "capacitor (component {}) across a resolved pulse ramp: its ramp response is not ported",
                device.component_index
            )));
        }
    }
    for j in junctions.iter().filter(|j| j.stamped()) {
        if fixed[j.anode].is_some() && fixed[j.cathode].is_some() && (ramp_node[j.anode] || ramp_node[j.cathode]) {
            return Err(BoError::unsupported_topology(format!(
                "LED junction (component {}) across a resolved pulse ramp: its ramp response is not ported",
                program.devices[j.device].component_index
            )));
        }
    }
    let reads = partition_reads(program, &partitions, &local_index, &fixed, &junctions);
    let hub: Vec<bool> = match rail {
        Some(t) if !t.exact => reads.iter().map(|nodes| nodes.binary_search(&t.rail).is_ok()).collect(),
        _ => vec![false; partitions.len()],
    };
    // A partition that reads a ramping node is stepped, so each step reads
    // the boundary at its own instant, unless it is a linear closed-form
    // reader: its segment is then driven exactly by the ramp's straight line
    // (Sources point 3, R3 (b)). Membership and classification stay fixed at
    // load.
    let ramp_bound = ramp_nonlinear_readers(program, &local_index, &ramp_node, &hub, rail.filter(|t| !t.exact).map(|t| t.rail));
    for (p, nodes) in reads.iter().enumerate() {
        if nodes.iter().any(|&node| ramp_node[node]) && !(kinds_by_partition[p] == PartitionKind::ClosedForm && !hub[p] && !ramp_bound[p]) {
            kinds_by_partition[p] = PartitionKind::Stepped;
        }
    }
    // Exact mode: the rail's own partition carries the hub source, whose
    // SoC and regime advance per committed step (Rails point 9), so it has
    // steps whenever it holds a capacitor: a closed form has none. A
    // battery's SoC is state on its own (the owner's rule, 2026-10-15): E
    // and r move with it, so a static partition, solved only at events,
    // would hold the rail and the draw at their load values and charge SoC
    // at that frozen current. Its partition is stepped whatever else it
    // holds, with `r = 0` too, where `u = E(soc)` moves the same way.
    if let Some(t) = rail {
        if t.exact {
            if let Some((p, _)) = local_index[t.rail] {
                let battery = program.devices[t.device].kind == kinds::BATTERY_PACK;
                if kinds_by_partition[p] == PartitionKind::ClosedForm || battery {
                    kinds_by_partition[p] = PartitionKind::Stepped;
                }
            }
        }
    }

    // W1e follow-up, gap 1 (W1c point 4): a closed-form node cannot be
    // watched against a supply that moves between events, because the
    // closed-form crossing is solved once for a fixed threshold. A free
    // supply in a Stepped partition is exactly that (a Static one only moves
    // at events, and the watch is re-aimed then). So a ClosedForm partition
    // holding a watched read pin of an IC with such a supply is promoted to
    // Stepped, where the watch follows the supply (`solve::stepped`).
    // One pass is enough: a promoted partition holds no free IC supply
    // (that would already have made it Stepped above), so the promotion
    // never creates a new Stepped supply for a later IC, and it never turns
    // a Static partition dynamic.
    for device in &program.devices {
        if !digital::is_ic(device.kind) {
            continue;
        }
        let ic = digital::IcRecord::from_device(device);
        let supply_in_stepped =
            [ic.vcc.node, ic.gnd.node].iter().any(|&node| matches!(local_index[node], Some((p, _)) if kinds_by_partition[p] == PartitionKind::Stepped));
        if !supply_in_stepped {
            continue;
        }
        for i in 0..ic.logic.len() {
            if !is_watched_read_pin(&ic, i) {
                continue;
            }
            if let Some((p, _)) = local_index[ic.logic[i].node] {
                if kinds_by_partition[p] == PartitionKind::ClosedForm {
                    kinds_by_partition[p] = PartitionKind::Stepped;
                }
            }
        }
    }

    // Ruling R59 (1) (B): with no rail capacitor a bench PSU in CC sets its
    // rail from the draws alone, so a closed-form hub partition's capacitor
    // carries the rail with it, and its segment, aimed at a rail it holds
    // fixed, goes stale as soon as the rail follows the capacitor: re-aimed
    // every `RAIL_DRIFT_V` of the rail's own motion, a 5.8 uF capacitor 0.45
    // ohm from a 1.3 A rail took 2.2M rail events for 6.3 ms (cold 742 tpl
    // 3). Such a partition is stepped with the rail as an unknown
    // (`stepped::CcRail`) while the PSU is in CC, and a closed form in CV,
    // where the rail is its fixed boundary (stepped at load in CV too, 35 CV
    // boards ran ~10x slower). A rail capacitor makes the rail a segment of
    // its own (Rails point 4), and a battery has no CC.
    let cc_switch: Vec<bool> = match rail {
        Some(t) if !t.exact && program.devices[t.device].kind == kinds::BENCH_PSU => {
            let rail_capacitor = program.devices.iter().filter(|d| d.kind == kinds::CAPACITOR).any(|d| {
                let (a, b) = (d.pins[0].node, d.pins[1].node);
                a != b && (a == t.rail || b == t.rail) && fixed[a].is_some() && fixed[b].is_some()
            });
            (0..partitions.len()).map(|p| !rail_capacitor && hub[p] && kinds_by_partition[p] == PartitionKind::ClosedForm).collect()
        }
        _ => vec![false; partitions.len()],
    };

    // W1c step 2b (this worker's own scope, README's "Kernel semantics: the
    // digital core (W1c)" contract point 2's stamping half): an IC driver or
    // floating pull landing in a non-`Static` partition used to be declined
    // outright ("ic stamp in a dynamic partition (W1c step 2b)"), because
    // `solve::closed_form`/`solve::stepped` did not yet know how to stamp an
    // IC. They do now (`solve::stamp_ics_into_dyn`, called from both
    // modules' own segment/step builders), so nothing here needs to reject
    // this topology any more — every partition kind accepts every IC
    // footprint the union rule above can produce.
    Ok(PartitionBuild {
        partitions,
        local_index,
        fixed,
        resolved,
        kinds: kinds_by_partition,
        multi_capacitor_partitions,
        holds_junction,
        rail,
        live,
        ramp_node,
        reads,
        hub,
        cc_switch,
    })
}

/// Per partition, whether an IC makes its response to a resolved ramp
/// something other than a linear drive of its segment (Sources point 3, D3
/// (b)), so that a closed-form reader of the ramp is stepped after all: an IC
/// supplied from a ramping node stamps into it (its `R_out` and power gate
/// follow the supply), or one of its watched read pins sits in it with a
/// threshold that moves between events, against a ramping supply, the rail,
/// or a hub partition's node (the rail's own drive, Rails point 4, which the
/// closed form does not compose with a ramp's).
fn ramp_nonlinear_readers(program: &Program, local_index: &[Option<(usize, usize)>], ramp_node: &[bool], hub: &[bool], rail: Option<usize>) -> Vec<bool> {
    let mut out = vec![false; hub.len()];
    let part = |node: usize| local_index[node].map(|(p, _)| p);
    for device in &program.devices {
        if !digital::is_ic(device.kind) {
            continue;
        }
        let ic = digital::IcRecord::from_device(device);
        let supplies = [ic.vcc.node, ic.gnd.node];
        let ramp_supply = supplies.iter().any(|&s| ramp_node[s]);
        let moving_supply = ramp_supply || supplies.iter().any(|&s| Some(s) == rail || part(s).is_some_and(|p| hub[p]));
        if ramp_supply {
            let stamped = digital::output_capable(ic.kind).iter().map(|&i| ic.logic[i].node).chain(ic.logic.iter().filter(|pin| pin.open && pin.floating_input && !pin.open_high).map(|pin| pin.node)).chain(supplies);
            for p in stamped.filter_map(part) {
                out[p] = true;
            }
        }
        if moving_supply {
            for (i, pin) in ic.logic.iter().enumerate() {
                if let Some(p) = part(pin.node).filter(|_| is_watched_read_pin(&ic, i)) {
                    out[p] = true;
                }
            }
        }
    }
    out
}

/// Per partition, the non-free nodes its stamps read (W1f, Rails points 2
/// and 7): the fixed end of a resistor, capacitor, switch contact or LED
/// junction with a free end in it; for an IC, both supplies wherever one of
/// its drivers, pulls or supplies is free (they set the driver's `R_out` and
/// the power gate), and an output on a non-free, non-datum node in the
/// partition of its free supply. Ascending, without repeats. A superset is
/// harmless: it only dirties or re-keys a partition that did not need it.
fn partition_reads(program: &Program, partitions: &[Vec<usize>], local_index: &[Option<(usize, usize)>], fixed: &[Option<f64>], junctions: &[Junction]) -> Vec<Vec<usize>> {
    let mut reads: Vec<Vec<usize>> = vec![Vec::new(); partitions.len()];
    let part = |node: usize| local_index[node].map(|(p, _)| p);
    let non_free = |node: usize| fixed[node].is_some();
    for device in &program.devices {
        match device.kind {
            kinds::RESISTOR | kinds::CAPACITOR | kinds::FUSE | kinds::PTC_FUSE | kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH => {
                if let Some(p) = device.pins.iter().find_map(|pin| part(pin.node)) {
                    reads[p].extend(device.pins.iter().map(|pin| pin.node).filter(|&node| non_free(node)));
                }
            }
            kind if digital::is_ic(kind) => {
                let ic = digital::IcRecord::from_device(device);
                let supplies: Vec<usize> = [ic.vcc.node, ic.gnd.node].into_iter().filter(|&node| non_free(node)).collect();
                let output_capable = digital::output_capable(ic.kind);
                let mut touched: Vec<usize> = Vec::new();
                touched.extend(output_capable.iter().filter_map(|&i| part(ic.logic[i].node)));
                touched.extend(ic.logic.iter().filter(|pin| pin.open && pin.floating_input && !pin.open_high).filter_map(|pin| part(pin.node)));
                let supply_parts: Vec<usize> = [ic.vcc.node, ic.gnd.node].into_iter().filter_map(part).collect();
                touched.extend(&supply_parts);
                for &p in &touched {
                    reads[p].extend(&supplies);
                }
                for &p in &supply_parts {
                    reads[p].extend(output_capable.iter().map(|&i| ic.logic[i].node).filter(|&node| node != 0 && non_free(node)));
                }
            }
            _ => {}
        }
    }
    for j in junctions.iter().filter(|j| j.stamped()) {
        match (part(j.anode), part(j.cathode)) {
            (Some(p), None) => reads[p].push(j.cathode),
            (None, Some(p)) => reads[p].push(j.anode),
            _ => {}
        }
    }
    for nodes in &mut reads {
        nodes.sort_unstable();
        nodes.dedup();
    }
    reads
}

/// Whether logic pin `logic_index` of `ic` is one whose node voltage a
/// crossing watch follows (W1c point 4, NE555 point 5): a non-open pin that
/// is not output-capable, and for a 555 exactly TRIG, RST and THR (CV is never
/// read, so a bypass capacitor on it must not be promoted or re-aimed).
/// Shared by `build`'s promotion and `solve`'s watch registration so the two
/// can never disagree about which pins a supply can strand.
pub(crate) fn is_watched_read_pin(ic: &digital::IcRecord, logic_index: usize) -> bool {
    if digital::is_ne555(ic.kind) {
        return matches!(logic_index, digital::NE555_TRIG | digital::NE555_RST | digital::NE555_THR);
    }
    !ic.logic[logic_index].open && !digital::output_capable(ic.kind).contains(&logic_index)
}

/// Whether output-capable pin `logic_index` sits on a fixed node other than
/// the datum, so its driver stamps into a free supply's partition and not
/// its own (`build`'s gap 2 union, `devices::ics::register`).
pub(crate) fn drives_a_fixed_node(ic: &digital::IcRecord, logic_index: usize, fixed: &[Option<f64>]) -> bool {
    let node = ic.logic[logic_index].node;
    node != 0 && fixed[node].is_some()
}

/// Fixed-point propagation from node 0 (contract, point 1). Returns each
/// node's fixed voltage (`None` for free nodes), the list of sources that did
/// the fixing, in resolution order, and the anchor nodes. `ERR_UNSUPPORTED_TOPOLOGY`
/// (-3) for a loop of ideal sources or a floating one.
///
/// From W1f every `section_anchor` node is fixed at 0 V first, as a 0 V
/// source from the datum (Rails point 1), and the ideal sources are code 2, a
/// `bench_psu` with no limit and the four waveform kinds, each at its value
/// at `START_TIME` (Sources point 4). A hub candidate is not an ideal source.
fn derive_fixed_nodes(program: &Program, n: usize) -> Result<(Vec<Option<f64>>, Vec<ResolvedSource>, Vec<usize>), BoError> {
    let mut fixed: Vec<Option<f64>> = vec![None; n];
    fixed[0] = Some(0.0);
    let mut resolved = Vec::new();

    let mut anchor_nodes = Vec::new();
    for (device_index, device) in program.devices.iter().enumerate() {
        if device.kind != kinds::SECTION_ANCHOR {
            continue;
        }
        let node = device.pins[0].node;
        if fixed[node].is_some() {
            return Err(BoError::unsupported_topology(format!(
                "section_anchor (component {}): its node is already fixed",
                device.component_index
            )));
        }
        fixed[node] = Some(0.0);
        anchor_nodes.push(node);
        resolved.push(ResolvedSource { device_index, parent_node: 0, child_node: node, parent_is_pos: false });
    }

    let source_indices: Vec<usize> = program
        .devices
        .iter()
        .enumerate()
        .filter(|(_, d)| is_ideal_source(d))
        .map(|(i, _)| i)
        .collect();
    let values: Vec<f64> = source_indices.iter().map(|&i| source_value_at_load(&program.devices[i], program.options.start_time)).collect();
    let mut is_resolved = vec![false; source_indices.len()];

    // Each pass can resolve at least one source or the loop is done; a chain
    // of k sources needs at most k passes in the worst ordering.
    for _pass in 0..=source_indices.len() {
        let mut progress = false;
        for (si, &device_index) in source_indices.iter().enumerate() {
            if is_resolved[si] {
                continue;
            }
            let device = &program.devices[device_index];
            let (pos, neg) = (device.pins[0].node, device.pins[1].node);
            let v = values[si];
            match (fixed[pos], fixed[neg]) {
                (Some(vp), None) => {
                    fixed[neg] = Some(vp - v);
                    is_resolved[si] = true;
                    resolved.push(ResolvedSource { device_index, parent_node: pos, child_node: neg, parent_is_pos: true });
                    progress = true;
                }
                (None, Some(vn)) => {
                    fixed[pos] = Some(vn + v);
                    is_resolved[si] = true;
                    resolved.push(ResolvedSource { device_index, parent_node: neg, child_node: pos, parent_is_pos: false });
                    progress = true;
                }
                // Both fixed or both free: nothing to do this pass. Both-free
                // may still resolve later; both-fixed never will (see the
                // final classification below) — it is left alone here so
                // topology errors are reported once, after propagation truly
                // stalls, not from an order-dependent snapshot.
                _ => {}
            }
        }
        if !progress {
            break;
        }
    }

    for (si, &device_index) in source_indices.iter().enumerate() {
        if is_resolved[si] {
            continue;
        }
        let device = &program.devices[device_index];
        let (pos, neg) = (device.pins[0].node, device.pins[1].node);
        let name = kinds::shape(device.kind).map_or("source", |s| s.name);
        return Err(BoError::unsupported_topology(if fixed[pos].is_some() && fixed[neg].is_some() {
            format!("{name} (component {}): both terminals already fixed — a loop of ideal sources", device.component_index)
        } else {
            format!("{name} (component {}): floating — neither terminal is reachable from the datum", device.component_index)
        }));
    }

    Ok((fixed, resolved, anchor_nodes))
}

/// Hub classification (README, Rails point 1), after the propagation: at
/// most one candidate, its return fixed at 0 V (node 0 or an anchor), its
/// rail free, no ideal source on the rail, no rail capacitor to a node a
/// waveform source moves, and a battery's `R(soc0) C_rail` below
/// `RAIL_TAU_MAX`. Every failure is `-3`.
fn classify_hub(program: &Program, fixed: &[Option<f64>], resolved: &[ResolvedSource], anchor_nodes: &[usize]) -> Result<Option<RailTopology>, BoError> {
    let mut candidates = program.devices.iter().enumerate().filter(|(_, d)| is_hub_candidate(d));
    let Some((device_index, device)) = candidates.next() else { return Ok(None) };
    if candidates.next().is_some() {
        return Err(BoError::unsupported_topology("more than one hub rail (W1 limit)".to_string()));
    }
    let name = kinds::shape(device.kind).map_or("hub", |s| s.name);
    let ci = device.component_index;
    let (rail, ret) = (device.pins[0].node, device.pins[1].node);
    if !(ret == 0 || anchor_nodes.contains(&ret)) {
        return Err(BoError::unsupported_topology(format!("{name} (component {ci}): hub rail return is not fixed at 0 V (node 0 or a section anchor)")));
    }
    if rail == 0 || rail == ret {
        return Err(BoError::unsupported_topology(format!("{name} (component {ci}): hub rail on the datum or on its own return")));
    }
    if fixed[rail].is_some() {
        return Err(BoError::unsupported_topology(format!("{name} (component {ci}): hub rail is fixed by an ideal source")));
    }
    if let Some(other) = program.devices.iter().find(|d| is_ideal_source(d) && d.pins.iter().any(|p| p.node == rail)) {
        return Err(BoError::unsupported_topology(format!(
            "{name} (component {ci}): an ideal source (component {}) has a pin on the hub rail",
            other.component_index
        )));
    }
    // A rail capacitor whose far pin a waveform source moves, directly or
    // through a source stacked on it: an edge there does not solve the rail,
    // and the CC segment ignores the far node moving (Rails point 7). The
    // forest is in resolution order, so a parent is marked before its child.
    let mut by_wave = vec![false; fixed.len()];
    for r in resolved {
        by_wave[r.child_node] = by_wave[r.parent_node] || is_waveform(program.devices[r.device_index].kind);
    }
    // The kind test comes first: a section anchor has one pin.
    let across_wave = |d: &&Device| {
        if d.kind != kinds::CAPACITOR {
            return false;
        }
        let (a, b) = (d.pins[0].node, d.pins[1].node);
        a != b && ((a == rail && by_wave[b]) || (b == rail && by_wave[a]))
    };
    if let Some(cap) = program.devices.iter().find(across_wave) {
        return Err(BoError::unsupported_topology(format!(
            "{name} (component {ci}): capacitor (component {}) from the hub rail to a waveform source's node is not handled",
            cap.component_index
        )));
    }
    let u0 = match device.kind {
        kinds::BATTERY_PACK => {
            let model = BatteryModel::from_record(&device.params, &device.blob);
            let soc0 = clamp_soc(model.soc0);
            // The rail capacitors: one pin on the rail, the other on a fixed
            // node (Rails point 7).
            let c_rail: f64 = program
                .devices
                .iter()
                .filter(|d| d.kind == kinds::CAPACITOR)
                .filter(|d| {
                    let (a, b) = (d.pins[0].node, d.pins[1].node);
                    a != b && ((a == rail && fixed[b].is_some()) || (b == rail && fixed[a].is_some()))
                })
                .map(|d| d.params[0])
                .sum();
            let tau = model.resistance(soc0) * c_rail;
            if !(tau < RAIL_TAU_MAX) {
                return Err(BoError::unsupported_topology(format!(
                    "{name} (component {ci}): battery rail time constant r*C_rail = {tau:e} s is not below {RAIL_TAU_MAX:e} s"
                )));
            }
            model.emf(soc0)
        }
        _ => device.params[0],
    };
    Ok(Some(RailTopology { device: device_index, rail, ret, exact: program.options.no_rail_shortcut, u0 }))
}

struct UnionFind {
    parent: Vec<usize>,
    /// Size of the tree rooted at `i`, meaningful only when `parent[i] == i`.
    /// Drives union-by-size below, so a chain of unions in any order — not
    /// just an already-favorable one — keeps every tree at O(log n) depth.
    size: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        UnionFind { parent: (0..n).collect(), size: vec![1; n] }
    }

    /// Iterative with path halving: every other node on the walk up is
    /// re-parented straight to its grandparent, which flattens the tree over
    /// repeated calls. No recursion means no input — however long a chain of
    /// unions it builds — can make this walk consume call-stack frames the
    /// way a recursive find would (contract, "W0 kernel semantics", point 3).
    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }

    /// Union by size: the smaller tree is grafted under the larger tree's
    /// root, so depth grows as O(log n) regardless of the order unions
    /// arrive in.
    fn union(&mut self, a: usize, b: usize) {
        let mut ra = self.find(a);
        let mut rb = self.find(b);
        if ra == rb {
            return;
        }
        if self.size[ra] < self.size[rb] {
            std::mem::swap(&mut ra, &mut rb);
        }
        self.parent[rb] = ra;
        self.size[ra] += self.size[rb];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinds::ic_param;
    use crate::program::{Device, Options, PinRecord};

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }

    fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }

    fn capacitor(ci: usize, a: usize, b: usize) -> Device {
        Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![1e-6, 0.0, 0.0], blob: vec![] }
    }

    /// A minimal `74ls04` device: `pins[i]` is logic pin `i`'s node
    /// (`icPinMap` order `1a 1y 2a 2y ... 6a 6y`), every pin driven (never
    /// `PIN_FLAG_OPEN`). Standard LS-TTL params, matching
    /// `solve::ic_tests`'s own builder of the same shape.
    fn ls04(ci: usize, vcc: usize, gnd: usize, pins: [usize; 12]) -> Device {
        let mut device_pins = vec![pin(vcc), pin(gnd)];
        device_pins.extend(pins.iter().map(|&n| pin(n)));
        let mut params = vec![0.0; ic_param::COUNT];
        params[ic_param::VCC_MIN] = 4.75;
        params[ic_param::VCC_MAX] = 5.25;
        params[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
        params[ic_param::VIL_FIXED] = 1.0;
        params[ic_param::VIL] = 0.8;
        params[ic_param::VIH_FIXED] = 1.0;
        params[ic_param::VIH] = 2.0;
        params[ic_param::NOMINAL] = 5.0;
        params[ic_param::IO_MAX] = 0.008;
        params[ic_param::DELAY] = 20e-9;
        let mut blob = vec![0.0];
        blob.extend((0..12).map(|i| if i % 2 == 0 { kinds::PIN_FLOATING_INPUT as f64 } else { 0.0 }));
        Device { kind: kinds::IC_74LS04, component_index: ci, pins: device_pins, params, blob }
    }

    fn program(node_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    }

    // ---- step 2b: the former dynamic-partition decline is lifted ---------
    //
    // These two fixtures are exactly the ones that used to prove the `-3`
    // decline (a git-log curiosity now); step 2b's own job is to prove the
    // opposite — the load now succeeds, with the IC's driver/pull footprint
    // classified into whatever dynamic partition it actually lands in. The
    // electrical correctness of what gets stamped there is `solve::ic_tests`'
    // job (a full kernel/solver round trip), not this module's: `partition.rs`
    // only ever decides topology and classification.

    #[test]
    fn an_ic_output_landing_in_a_closed_form_partition_now_loads_and_classifies_closed_form() {
        // vcc/gnd fixed (5V/0V, an ideal supply — contract point 2's "no IC
        // pin joins anything" case), so 1y's own partition is decided purely
        // by what else connects to it: a capacitor to gnd, making it
        // ClosedForm on its own, with no help from the IC's union rule at
        // all. That partition is exactly where 1y's driver needs to stamp —
        // not `Static` — which no longer fails the load.
        let p = program(4, vec![
            source(0, 1, 0, 5.0),
            capacitor(1, 2, 0),
            ls04(2, 1, 0, [0, 2, 0, 3, 0, 3, 0, 3, 0, 3, 0, 3]),
        ]);
        let pb = build(&p).expect("an IC driver landing in a ClosedForm partition must load now that step 2b stamps it");
        let (p_1y, _) = pb.local_index[2].expect("1y is a free node");
        assert_eq!(pb.kinds[p_1y], PartitionKind::ClosedForm);
    }

    #[test]
    fn an_open_floating_input_pin_landing_in_a_stepped_partition_now_loads_and_classifies_stepped() {
        // Two capacitors on the SAME node as an open, FLOATING_INPUT pin (2a
        // here): Stepped (>= 2 capacitors), not ClosedForm — the load must
        // succeed regardless of which dynamic kind the pull lands in.
        let p = program(5, vec![
            source(0, 1, 0, 5.0),
            capacitor(1, 2, 0),
            capacitor(2, 2, 0),
            {
                let mut d = ls04(3, 1, 0, [0, 3, 2, 4, 0, 3, 0, 3, 0, 3, 0, 3]);
                d.pins[4].flags = crate::program::PIN_FLAG_OPEN; // 2a (pins[0..2]=vcc,gnd, then 1a,1y,2a,...): index 4 = 2a.
                d
            },
        ]);
        let pb = build(&p).expect("an open floating-input pull landing in a Stepped partition must load now that step 2b stamps it");
        let (p_2a, _) = pb.local_index[2].expect("2a is a free node");
        assert_eq!(pb.kinds[p_2a], PartitionKind::Stepped);
    }

    // ---- point 3: a dynamic partition holding a free IC supply node is
    // classified Stepped ------------------------------------------------

    #[test]
    fn a_closed_form_partition_holding_a_free_ic_supply_node_is_classified_stepped() {
        // vcc FREE (through a resistor from the 5V source), gnd fixed. Every
        // logic pin is tied straight to a FIXED node (never free), so no IC
        // driver/pull ever lands in vcc's partition at all -- only a plain
        // decoupling capacitor from vcc to gnd does (one free end, so NOT
        // rule 2a's fixed-pair case), isolating the promotion rule from the
        // `-3` decline: this partition holds a free IC supply node and one
        // capacitor, so it must be classified Stepped, not ClosedForm, and
        // `build` must still succeed (nothing here ever needs step 2b).
        let p = program(3, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 2, 1, 10.0),
            capacitor(2, 2, 0),
            ls04(3, 2, 0, [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
        ]);
        let pb = build(&p).unwrap();
        let (p_vcc, _) = pb.local_index[2].expect("vcc is free");
        assert_eq!(pb.kinds[p_vcc], PartitionKind::Stepped, "a free IC supply node forces Stepped, never ClosedForm");
    }

    #[test]
    fn a_closed_form_partition_holding_a_74ls245_a_or_b_pin_is_classified_stepped() {
        // Fixed 5 V rails. a1 (logic pin 2) sits on an RC node: one counted
        // capacitor, so ClosedForm by count, but the pin is read AND driven,
        // so the partition must be Stepped. Every other logic pin is tied to
        // gnd.
        let mut pins = vec![pin(1), pin(0)];
        pins.extend((0..18).map(|i| pin(if i == 2 { 2 } else { 0 })));
        let mut params = vec![0.0; ic_param::COUNT];
        params[ic_param::VCC_MIN] = 4.75;
        params[ic_param::VCC_MAX] = 5.25;
        params[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
        params[ic_param::NOMINAL] = 5.0;
        params[ic_param::IO_MAX] = 0.024;
        params[ic_param::DELAY] = 20e-9;
        // 245's delay tail (W1d retrofit): oeEnable, oeDisable -- this test
        // never exercises timing, only partition classification, but
        // `IcRecord::from_device` indexes `params[13..17]` unconditionally
        // now that `kinds::delay_pairs(IC_74LS245)` is 2, so the vector must
        // carry them to avoid an out-of-bounds panic.
        params.extend([25e-9, 27e-9, 15e-9, 15e-9]);
        let ls245 = Device { kind: kinds::IC_74LS245, component_index: 3, pins, params, blob: vec![0.0; 19] };
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 10_000.0), capacitor(2, 2, 0), ls245]);
        let pb = build(&p).unwrap();
        let (p_a1, _) = pb.local_index[2].expect("a1's node is free");
        assert_eq!(pb.kinds[p_a1], PartitionKind::Stepped);
    }

    #[test]
    fn union_find_handles_a_million_node_chain_without_recursing() {
        // The historical bug (W0): a recursive `find` spends one call-stack
        // frame per link in the parent chain, and a plain "always point a at
        // b" union with no size heuristic turns a chain of unions listed in
        // ascending order into a parent chain as long as the input. At
        // 1,000,000 nodes — the program format's own ceiling
        // (`program::MAX_NODES_OR_DEVICES`) — that recursion overflows the
        // stack. This exercises `UnionFind` directly (no dense solve, so no
        // memory ceiling) at exactly that size: iterative `find` must handle
        // it regardless of how deep any particular union order would have
        // made a naive tree.
        let n = 1_000_000;
        let mut uf = UnionFind::new(n);
        for i in 0..n - 1 {
            uf.union(i, i + 1);
        }
        let root = uf.find(0);
        for i in (0..n).step_by(9973) {
            assert_eq!(uf.find(i), root, "node {i} did not join the single chain's tree");
        }
        assert_eq!(uf.find(n - 1), root);
    }
    // ---- W1e: LED junctions (README, LEDs points 4 and 5) -----------------

    use crate::program::PIN_FLAG_OPEN;

    fn led(ci: usize, anode: usize, cathode: usize, burnt: f64) -> Device {
        Device { kind: kinds::LED, component_index: ci, pins: vec![pin(anode), pin(cathode)], params: vec![1.8, 2.0, 0.02, 0.0, 0.0, burnt], blob: vec![] }
    }

    fn bicolor(ci: usize, a1: usize, a2: usize, k: usize) -> Device {
        Device { kind: kinds::BICOLOR_LED, component_index: ci, pins: vec![pin(a1), pin(a2), pin(k)], params: vec![1.8, 2.0, 0.02, 0.02, 0.0], blob: vec![0.0] }
    }

    fn seg7_cc(ci: usize, pins: Vec<PinRecord>) -> Device {
        Device { kind: kinds::SEG7_CC, component_index: ci, pins, params: vec![2.0, 0.02], blob: vec![0.0] }
    }

    fn open_pin() -> PinRecord {
        PinRecord { node: 0, flags: PIN_FLAG_OPEN, open_hash: 0, dead_hash: 0 }
    }

    fn kind_of(pb: &PartitionBuild, node: usize) -> PartitionKind {
        pb.kinds[pb.local_index[node].expect("a free node").0]
    }

    #[test]
    fn a_led_beside_a_resistor_is_static_and_holds_a_junction() {
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 220.0), led(2, 2, 0, 0.0)]);
        let pb = build(&p).unwrap();
        assert_eq!(pb.partitions.len(), 1);
        assert_eq!(pb.kinds, vec![PartitionKind::Static]);
        assert_eq!(pb.holds_junction, vec![true]);
    }

    #[test]
    fn a_led_beside_one_capacitor_is_stepped_never_closed_form() {
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0), led(3, 2, 0, 0.0)]);
        let pb = build(&p).unwrap();
        assert_eq!(kind_of(&pb, 2), PartitionKind::Stepped, "the exact exponential needs a linear network");
        // The same circuit without the LED is the ordinary closed-form case.
        let plain = build(&program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0)])).unwrap();
        assert_eq!(kind_of(&plain, 2), PartitionKind::ClosedForm);
        // Stat 15 counts capacitors, not junctions.
        assert_eq!(pb.multi_capacitor_partitions, 0);
    }

    #[test]
    fn a_self_shorted_led_leaves_its_partition_linear() {
        // Both pins on node 2: the companion cancels, nothing is stamped, and
        // nothing is classified nonlinear.
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 220.0), led(2, 2, 2, 0.0)]);
        let pb = build(&p).unwrap();
        assert_eq!(pb.holds_junction, vec![false]);
        assert_eq!(pb.kinds, vec![PartitionKind::Static]);
        // Beside one capacitor it stays closed-form.
        let p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 220.0), capacitor(2, 2, 0), led(3, 2, 2, 0.0)]);
        let pb = build(&p).unwrap();
        assert_eq!(kind_of(&pb, 2), PartitionKind::ClosedForm);
    }

    #[test]
    fn a_led_straight_across_an_ideal_source_belongs_to_no_partition_and_is_not_a_topology_error() {
        let p = program(2, vec![source(0, 1, 0, 5.0), led(1, 1, 0, 0.0)]);
        let pb = build(&p).expect("not -3: it is a constant load on the source");
        assert!(pb.partitions.is_empty());
        assert!(pb.holds_junction.is_empty());
    }

    #[test]
    fn a_burnt_led_joins_nothing_and_leaves_a_linear_partition() {
        // 5 V --R-- n2 --LED-- n3 --R-- gnd. Live: one partition holding the
        // junction. Burnt: no junction, so n2 and n3 are two partitions.
        let devices = |burnt: f64| vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 220.0), led(2, 2, 3, burnt), resistor(3, 3, 0, 220.0)];
        let live = build(&program(4, devices(0.0))).unwrap();
        assert_eq!(live.partitions.len(), 1);
        assert_eq!(live.holds_junction, vec![true]);
        let burnt = build(&program(4, devices(1.0))).unwrap();
        assert_eq!(burnt.partitions.len(), 2, "a burnt device joins nothing");
        assert_eq!(burnt.holds_junction, vec![false, false]);
        assert!(burnt.kinds.iter().all(|k| *k == PartitionKind::Static));
    }

    #[test]
    fn a_led_with_one_fixed_end_sits_in_the_partition_of_its_free_end() {
        // Anode on the 5 V rail, cathode free: a junction in the cathode's partition.
        let p = program(3, vec![source(0, 1, 0, 5.0), led(1, 1, 2, 0.0), resistor(2, 2, 0, 330.0)]);
        let pb = build(&p).unwrap();
        assert_eq!(pb.partitions.len(), 1);
        assert_eq!(pb.holds_junction, vec![true]);
    }

    #[test]
    fn a_bicolor_led_with_its_cathode_on_the_datum_puts_a1_and_a2_in_two_partitions() {
        let p = program(4, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 150.0), resistor(2, 1, 3, 330.0), bicolor(3, 2, 3, 0)]);
        let pb = build(&p).unwrap();
        assert_eq!(pb.partitions.len(), 2);
        assert_ne!(pb.local_index[2].unwrap().0, pb.local_index[3].unwrap().0);
        assert_eq!(pb.holds_junction, vec![true, true]);
        // With `k` free they share it.
        let p = program(5, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 150.0), resistor(2, 1, 3, 330.0), bicolor(3, 2, 3, 4), resistor(4, 4, 0, 47.0)]);
        let pb = build(&p).unwrap();
        assert_eq!(pb.partitions.len(), 1);
    }

    #[test]
    fn a_seg7_joins_its_segments_through_a_free_common_and_not_at_all_through_a_fixed_one() {
        let mut pins: Vec<PinRecord> = (2..10).map(pin).collect();
        pins.push(pin(10)); // com
        pins.push(open_pin());
        let mut devices = vec![source(0, 1, 0, 5.0)];
        for i in 0..8 {
            devices.push(resistor(1 + i, 1, 2 + i, 330.0));
        }
        devices.push(resistor(9, 10, 0, 10.0));
        devices.push(seg7_cc(10, pins.clone()));
        let free_common = build(&program(11, devices.clone())).unwrap();
        assert_eq!(free_common.partitions.len(), 1, "eight segments and the common are one partition");
        assert_eq!(free_common.partitions[0].len(), 9);

        // Common straight on the datum: each segment is its own partition.
        pins[8] = pin(0);
        let mut devices = devices;
        devices.pop();
        devices.pop(); // the common resistor and the display
        devices.push(seg7_cc(10, pins));
        let fixed_common = build(&program(10, devices)).unwrap();
        assert_eq!(fixed_common.partitions.len(), 8);
        assert!(fixed_common.partitions.iter().all(|p| p.len() == 1));
        assert!(fixed_common.holds_junction.iter().all(|&h| h));
    }

    #[test]
    fn force_stepped_and_a_junction_together_stay_stepped() {
        let mut p = program(3, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 0), led(3, 2, 0, 0.0)]);
        p.options.force_stepped = true;
        assert_eq!(kind_of(&build(&p).unwrap(), 2), PartitionKind::Stepped);
    }

    // ---- W1e follow-up, gap 1: a watched read pin whose supply is free in a
    // Stepped partition promotes its closed-form partition ------------------

    #[derive(Clone, Copy)]
    enum Supply {
        Fixed,
        Static,
        Stepped,
    }

    /// 0=gnd, 1=5 V rail, 2=the IC's vcc (the rail itself for `Fixed`; 10
    /// ohm from it for `Static`; with a 1 uF decoupling capacitor as well for
    /// `Stepped`), 3=1a (10k from the rail and 1 uF to gnd: one capacitor,
    /// closed-form by count), 4=1y. Every other logic pin is on the datum.
    fn read_pin_program(supply: Supply) -> Program {
        let mut devices = vec![source(0, 1, 0, 5.0), resistor(1, 1, 3, 10_000.0), capacitor(2, 3, 0)];
        let vcc = match supply {
            Supply::Fixed => 1,
            Supply::Static => {
                devices.push(resistor(3, 1, 2, 10.0));
                2
            }
            Supply::Stepped => {
                devices.push(resistor(3, 1, 2, 10.0));
                devices.push(capacitor(4, 2, 0));
                2
            }
        };
        devices.push(ls04(5, vcc, 0, [3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]));
        program(5, devices)
    }

    #[test]
    fn a_closed_form_read_pin_partition_is_promoted_when_the_ic_supply_is_free_in_a_stepped_partition() {
        let pb = build(&read_pin_program(Supply::Stepped)).unwrap();
        let (p_vcc, _) = pb.local_index[2].unwrap();
        let (p_a, _) = pb.local_index[3].unwrap();
        assert_eq!(pb.kinds[p_vcc], PartitionKind::Stepped, "the supply's own partition (a free IC supply beside a capacitor)");
        assert_ne!(p_a, p_vcc, "a read pin is never unioned with its IC's supply");
        assert_eq!(pb.kinds[p_a], PartitionKind::Stepped, "one capacitor makes it closed-form by count; the moving supply promotes it");
    }

    #[test]
    fn a_closed_form_read_pin_partition_stays_closed_form_with_a_fixed_or_static_supply() {
        for supply in [Supply::Fixed, Supply::Static] {
            let pb = build(&read_pin_program(supply)).unwrap();
            let (p_a, _) = pb.local_index[3].unwrap();
            assert_eq!(pb.kinds[p_a], PartitionKind::ClosedForm, "a supply that moves only at events is re-aimed, not stepped");
        }
    }

    #[test]
    fn only_a_555s_trig_rst_and_thr_promote_a_partition_and_its_cv_does_not() {
        // 0=gnd, 1=5 V rail, 2=vcc (10 ohm from the rail + 1 uF: Stepped),
        // 3=TRIG and 5=CV, each on its own 10k/1 uF from the rail
        // (closed-form by count), 4=OUT, 6=DIS. RST on the rail, THR on gnd.
        let p = program(7, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 1, 2, 10.0),
            capacitor(2, 2, 0),
            resistor(3, 1, 3, 10_000.0),
            capacitor(4, 3, 0),
            resistor(5, 1, 5, 10_000.0),
            capacitor(6, 5, 0),
            ne555_dev(7, 2, 0, [3, 4, 1, 5, 0, 6]),
        ]);
        let pb = build(&p).unwrap();
        assert_eq!(pb.kinds[pb.local_index[2].unwrap().0], PartitionKind::Stepped);
        assert_eq!(pb.kinds[pb.local_index[3].unwrap().0], PartitionKind::Stepped, "TRIG is watched");
        assert_eq!(pb.kinds[pb.local_index[5].unwrap().0], PartitionKind::ClosedForm, "CV is never read, so its bypass capacitor stays closed-form");
    }

    /// An `ne555` with no catalog window, pins in the record's order, `logic`
    /// in TRIG/OUT/RST/CV/THR/DIS order.
    fn ne555_dev(ci: usize, vcc: usize, gnd: usize, logic: [usize; 6]) -> Device {
        let mut params = vec![0.0; ic_param::COUNT];
        params[ic_param::VCC_MIN] = 2.0;
        params[ic_param::VCC_MAX] = 18.0;
        params[ic_param::NOMINAL] = 5.0;
        let mut pins = vec![pin(vcc), pin(gnd)];
        pins.extend(logic.map(pin));
        Device { kind: kinds::NE555, component_index: ci, pins, params, blob: vec![0.0; 7] }
    }

    // ---- gap 2: a driver on a fixed node joins both of its free supplies --

    /// 0=datum, 1=5 V rail, 2=1.5 V source, 3=vcc (10 ohm from the rail),
    /// 4=gnd (10 ohm to the datum), 1y on `y_node`. Every other output is on
    /// the datum and every input too, so 1y is the only pin that could join
    /// the supplies.
    fn both_supplies_free(y_node: usize) -> Program {
        program(5, vec![
            source(0, 1, 0, 5.0),
            source(1, 2, 0, 1.5),
            resistor(2, 1, 3, 10.0),
            resistor(3, 4, 0, 10.0),
            ls04(4, 3, 4, [0, y_node, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
        ])
    }

    #[test]
    fn an_output_on_a_fixed_node_joins_the_two_free_supplies_of_its_ic() {
        let pb = build(&both_supplies_free(2)).unwrap();
        assert_eq!(pb.local_index[3].unwrap().0, pb.local_index[4].unwrap().0, "one driver dirties one partition, and its stamp is HIGH into vcc, LOW into gnd");
    }

    #[test]
    fn an_output_on_the_datum_leaves_the_two_free_supplies_apart() {
        // A pin on the datum stamps nothing, so there is nothing to keep in
        // one partition and the supplies stay as their own resistors make them.
        let pb = build(&both_supplies_free(0)).unwrap();
        assert_ne!(pb.local_index[3].unwrap().0, pb.local_index[4].unwrap().0);
    }
}
