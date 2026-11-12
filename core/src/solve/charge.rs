//! W2b piece 4: light means, the charge (core/README.md, "Charge"; W2 design
//! 2.4, ruling R57 D4 (a)).
//!
//! A host arms a list of devices; from then on each one's published current
//! is integrated over time, and each of its channel slots (an LED family
//! device's), until the host takes them: `[t0, t1, then per device its
//! charge and its channels' in channel order, then per device the time its
//! charge is final to]`, the span `[t0, t1]` they cover, and the sums
//! cleared. A light mean is a charge over its span. A device's final time is
//! `t1` except a stepped one's after a yield (below): its charge then runs
//! from its previous take's final time to this one's.
//!
//! Each charge is exact in the model of the solver that publishes the
//! current, read off what that solver already holds, never by stepping or
//! re-solving anything for it (a charged run is the uncharged run bit for
//! bit):
//! - a static partition's or a fixed node's device publishes a current that
//!   holds between instants: it is folded lazily, its current times the time
//!   it held, refolded only when the published value changes;
//! - a resistor or switch across fixed nodes, one of them on a running ramp,
//!   moves linearly between instants: its current at the interval's mean;
//! - a closed-form partition's currents are affine in its capacitor's
//!   voltage, the rail's move and the ramp's time: their means over each
//!   interval between instants, analytically (`publish_mean`), and the
//!   capacitor's `C dvc` exactly;
//! - a stepped partition's currents by its integrator's own quadrature over
//!   each committed step's points (backward Euler, the trapezoid, TR-BDF2's
//!   two stages, `ChargeStep::weights`), which moves each capacitor's
//!   charge exactly `C dvc`; an LED's channels at those points by its law;
//! - a capacitor across two fixed nodes: `C dvc` on its own exponential;
//! - a voltage source (and anything else the propagation forest fixes): by
//!   KCL, from the charges of what leaves the nodes it fixes, each in its
//!   own model, walked as `recompute_source_currents` walks the currents; a
//!   PSU in CC is `-dir iLimit` for as long as it is.
//!
//! Everything is folded at the kernel's `observe` calls, between instants
//! (and at a take): the interval up to the next instant on the state the
//! last one left, the static changes the last instant made, and the steps
//! stepped partitions committed in it. A step commits inside an instant's
//! window and is dropped with it when that instant rolls back, so a
//! rolled-back instant adds nothing. A stepped partition's charge is final
//! up to its last commit: after `advance` (whose horizon clamp commits every
//! partition to `t_end`) that is `t1`; after a yield (`advance_for`) the
//! part of a step not yet committed arrives with the take after its commit.

use super::*;
use crate::solve::stepped::ChargeStep;



/// How an armed device's current is integrated (the module's doc comment).
#[derive(Debug, Clone)]
pub(crate) enum ChargeClass {
    /// Its published current, held between instants.
    Held,
    /// An LED-family device (index in `led_devices`) on held voltages.
    HeldLed(u32),
    /// A 555 (position in `ics`) on held voltages.
    Held555(u32),
    /// A resistor or switch (its index in `kcl_devices`), an LED or a 555
    /// with every node fixed, where a running ramp moves it linearly between
    /// instants: its current at the interval's mean voltages.
    Mean(Option<u32>),
    /// The same on the hub rail or a static hub partition's line in it:
    /// held while the rail holds between its events, at the interval's mean
    /// voltages while it moves (`ChargeState::line_moving`).
    Line(Option<u32>),
    /// A device of dynamic partition `p` (its closed form or its stepped
    /// copy, whichever runs), or an LED or 555 whose nodes are in it.
    Part(u32),
    /// A capacitor across two fixed nodes (index in `fixed_pair_capacitors`).
    FixedCap(u32),
    /// A device the propagation forest fixes a node with.
    Source,
}

#[derive(Debug, Clone)]
pub(crate) struct ArmedDev {
    device: usize,
    /// Its first slot in the take (its charge, then its channels').
    slot: usize,
    nch: usize,
    class: ChargeClass,
    /// Its index in `led_devices`, or its position in `ics` for a 555.
    led: Option<u32>,
    ne555: Option<u32>,
    /// A line's nodes, what its held value is read from.
    nodes: Vec<usize>,
    /// A held LED's last states left, `(key, current, channels)`: a logic
    /// output toggling it re-solves its partition to the same bits, so its
    /// readout comes back without its exponential.
    memo: Vec<(Vec<u64>, f64, Vec<f64>)>,
    /// The held classes: the current and channels published since `since`,
    /// and what they were read from, to tell when they change.
    i: f64,
    ch: Vec<f64>,
    since: f64,
    key: Vec<u64>,
}

/// The KCL side of an armed source: the nodes whose leaving charge its walk
/// reads, the terms that leave them, and their charge so far.
#[derive(Debug, Clone)]
pub(crate) struct SourceTerms {
    rel: Vec<bool>,
    rel_nodes: Vec<usize>,
    kcl: Vec<u32>,
    caps: Vec<u32>,
    junctions: Vec<u32>,
    ics: Vec<u32>,
    /// `∫ leaving` by node, outside and inside a CC stretch of the hub PSU,
    /// and a scratch for a held-rail interval's own (`charge_fold_interval`).
    lq: [Vec<f64>; 3],
    /// The CC stretch's own charge, `-dir iLimit` times its length.
    cc_q: f64,
    cc_device: Option<usize>,
    /// The dynamic partitions its terms sit in.
    parts: Vec<usize>,
    /// Per armed source, whether the hub rail is in its walk; and whether an
    /// interval since the last take closed such a source's KCL only
    /// approximately (several stepped hub partitions on a CC rail with no
    /// capacitor: `charge_rail_carrier`).
    on_rail: Vec<usize>,
    approximate: bool,
    /// Whether the hub PSU was in CC when the last fold left it: the steps
    /// committed since then were taken in that regime.
    cc_prev: bool,
}

#[derive(Debug, Clone)]
pub(crate) struct ChargeState {
    devs: Vec<ArmedDev>,
    q: Vec<f64>,
    t0: f64,
    /// How far everything but the stepped steps not yet committed is folded.
    cursor: f64,
    t_arm: f64,
    /// The dynamic partitions a device or a source term of the charge sits in.
    parts: Vec<usize>,
    /// The nodes it reads that follow the hub rail between instants (the
    /// rail, and nodes of static hub partitions).
    line_nodes: Vec<usize>,
    src: Option<SourceTerms>,
    /// Scratch: device currents at the point being weighed, the voltages
    /// written over for it, channels, the steps drained.
    sc: Vec<f64>,
    saved: Vec<(usize, f64)>,
    chs: Vec<f64>,
    steps: Vec<ChargeStep>,
    published: Vec<usize>,
    /// Intervals folded at a held rail, waiting for the rail's next solve.
    pending: Vec<RailPending>,
    /// What can move a held class: per node (`node_deps[node_start[n] ..
    /// node_start[n + 1]]`) the held LEDs and 555s reading it, per device the
    /// held device publishing it (`u32::MAX`: none); the ones an instant
    /// moved, refolded at the next fold (`ChargeState::note_node`), and
    /// the power holds taken and released when last noted.
    node_start: Vec<u32>,
    node_deps: Vec<u32>,
    /// The nodes some held device reads, as a bitset.
    watched: Vec<u64>,
    dev_held: Vec<u32>,
    dirty: Vec<u32>,
    is_dirty: Vec<bool>,
    holds_seen: [f64; 2],
    /// Whether the hub rail moved between instants at the last fold (the
    /// lines then fold at each interval's means, else held).
    line_moving: bool,
    /// Fixed at arming: whether any interval has anything to fold (a mean, a
    /// fixed-pair capacitor, a dynamic partition's device, a source), whether
    /// any line is armed, and whether anything reads the hub rail's notes.
    has_interval: bool,
    has_line: bool,
    reads_rail: bool,
    /// Per node: the hub rail, or a static hub partition's node on its line.
    line_flag: Vec<bool>,
    /// The devices an interval folds (means, lines, fixed-pair capacitors,
    /// dynamic partitions' devices), and the fixed-pair capacitors it reads
    /// (armed, or a source term's), so an interval walks only those.
    interval_devs: Vec<u32>,
    fixed_caps: Vec<u32>,
    /// The stepped hub partition whose steps carry the rail, as the last fold
    /// found it (`charge_rail_carrier`): its steps fold the rail's lines.
    carrier: Option<usize>,
    /// The rail capacitors' charge (as a rail voltage) where the last fold on
    /// the CC segment left it, to charge them the jump when a rail solve
    /// re-aims the segment from a stepped hub partition's commit
    /// (`charge_rail_jump`); `None` while nothing folds them on a segment.
    rail_q: Option<f64>,
}

impl ChargeState {
    /// Whether a class is folded held now: the held ones, and a line while
    /// the rail holds.
    fn held(&self, class: &ChargeClass) -> bool {
        matches!(class, ChargeClass::Held | ChargeClass::HeldLed(_) | ChargeClass::Held555(_)) || (matches!(class, ChargeClass::Line(_)) && !self.line_moving)
    }

    /// A static or boundary node's published value moved (`copy_in_static`,
    /// `write_boundary`, the only writers of the nodes a held device reads):
    /// the held devices reading it refold, so a fold costs what changed, not
    /// what is armed. A mark from an instant that then rolls back only asks
    /// for a refold that finds nothing moved. A debug build checks at every
    /// fold that no unmarked held device moved.
    pub(crate) fn note_node(&mut self, n: usize) {
        if self.watched.get(n / 64).is_some_and(|w| w >> (n % 64) & 1 != 0) {
            for k in self.node_start[n]..self.node_start[n + 1] {
                let d = self.node_deps[k as usize];
                Self::mark(&mut self.dirty, &mut self.is_dirty, d);
            }
        }
    }

    /// A static device's published current moved (`copy_in_static`).
    pub(crate) fn note_device(&mut self, device: usize) {
        if let Some(&d) = self.dev_held.get(device).filter(|&&d| d != u32::MAX) {
            Self::mark(&mut self.dirty, &mut self.is_dirty, d);
        }
    }

    /// At the end of a completed instant: a power hold taken or released
    /// moves every held 555's power.
    pub(crate) fn note_holds(&mut self, holds: [f64; 2]) {
        if holds != self.holds_seen {
            self.holds_seen = holds;
            for k in 0..self.devs.len() {
                if matches!(self.devs[k].class, ChargeClass::Held555(_)) {
                    Self::mark(&mut self.dirty, &mut self.is_dirty, k as u32);
                }
            }
        }
    }

    fn mark(dirty: &mut Vec<u32>, is_dirty: &mut [bool], d: u32) {
        if !is_dirty[d as usize] {
            is_dirty[d as usize] = true;
            dirty.push(d);
        }
    }
}

/// An interval `[a, b]` folded at the rail `u_held` where nothing models the
/// rail between its solves, with each slot's and leaving charge's
/// sensitivity to the rail's mean (per volt; exact, as every term on the
/// rail's line is affine in it).
#[derive(Debug, Clone)]
pub(crate) struct RailPending {
    a: f64,
    b: f64,
    u_held: f64,
    slots: Vec<(u32, f64)>,
    lq: Vec<(u8, u32, f64)>,
}

/// Whether the walk reads a node, or none.
fn rel_of(src: &Option<SourceTerms>, n: usize) -> bool {
    src.as_ref().is_some_and(|s| s.rel[n])
}

impl PartitionedSolver {
    /// The dynamic partition holding `node`, if any.
    fn dyn_part_of(&self, node: usize) -> Option<usize> {
        match self.local_index.get(node).copied().flatten() {
            Some((p, _)) if self.partition_kinds[p] != PartitionKind::Static => Some(p),
            _ => None,
        }
    }

    /// The partition (any kind) of whichever of `a`, `b` is free: a term
    /// between two partitions does not exist (they meet at fixed nodes).
    fn owner_of(&self, a: usize, b: usize) -> Option<usize> {
        self.local_index[a].or(self.local_index[b]).map(|(p, _)| p)
    }

    /// W2b piece 4: arms the charge on `devices` (`None` disarms), from
    /// `now` on. Re-arming drops what was not taken.
    pub(crate) fn arm_charge_on(&mut self, devices: Option<&[usize]>, now: f64) {
        let Some(devices) = devices else {
            self.charge = None;
            self.stepped.set_charge(None);
            return;
        };
        let drivers = Rc::clone(&self.driver_snapshot);
        self.ensure_ic_driver_ids(&drivers);
        let mut dyn_device: HashMap<usize, usize> = HashMap::new();
        for (p, out) in self.dyn_out.iter().enumerate() {
            if self.partition_kinds.get(p).is_some_and(|k| *k != PartitionKind::Static) {
                for &d in &out.devices {
                    dyn_device.insert(d, p);
                }
            }
        }
        let is_source = |d: usize| self.resolved.iter().any(|r| r.device_index == d);
        let mut devs = Vec::with_capacity(devices.len());
        let mut slot = 0;
        let mut parts: Vec<usize> = Vec::new();
        // Per stepped partition, the nodes outside it the charge reads.
        let mut reads: HashMap<usize, Vec<usize>> = HashMap::new();
        let rail = self.hub.as_ref().map(|h| h.node);
        let mut line_nodes: Vec<usize> = rail.into_iter().collect();
        for &d in devices {
            let led = self.led_devices.iter().position(|e| e.device_index == d);
            let ic = self.ic_position.get(d).copied().filter(|&p| p != u32::MAX);
            let ne555 = ic.filter(|&pos| digital::is_ne555(self.ics[pos as usize].1.kind));
            let kcl = self.kcl_position.get(&d).copied();
            // The nodes its current is read from, where the charge reads it
            // off voltages.
            let nodes: Option<Vec<usize>> = if let Some(k) = led {
                Some(self.led_devices[k].junctions.iter().flat_map(|j| [j.anode, j.cathode]).collect())
            } else if let Some(pos) = ne555 {
                let ic = &self.ics[pos as usize].1;
                Some(vec![ic.logic[digital::NE555_OUT].node, ic.gnd.node, ic.vcc.node])
            } else {
                kcl.map(|k| self.kcl_devices[k].nodes.clone())
            };
            let dynamic = nodes.as_ref().and_then(|ns| ns.iter().find_map(|&n| self.dyn_part_of(n))).or_else(|| dyn_device.get(&d).copied());
            let on_line = |n: usize| rail == Some(n) || self.static_solver.hub_node_line(n).is_some();
            let class = if is_source(d) {
                ChargeClass::Source
            } else if let Some(k) = self.fixed_pair_capacitors.iter().position(|c| c.device_index == d) {
                ChargeClass::FixedCap(k as u32)
            } else if let Some(p) = dynamic {
                reads.entry(p).or_default().extend(nodes.iter().flatten());
                ChargeClass::Part(p as u32)
            } else if let Some(ns) = nodes.as_ref().filter(|ns| ns.iter().all(|&n| self.local_index[n].is_none())) {
                line_nodes.extend(ns.iter().copied());
                ChargeClass::Mean(kcl.map(|k| k as u32))
            } else if let Some(ns) = nodes.as_ref().filter(|ns| ns.iter().any(|&n| on_line(n))) {
                line_nodes.extend(ns.iter().copied());
                ChargeClass::Line(kcl.map(|k| k as u32))
            } else if let Some(k) = led {
                ChargeClass::HeldLed(k as u32)
            } else if let Some(pos) = ne555 {
                ChargeClass::Held555(pos)
            } else {
                ChargeClass::Held
            };
            if let ChargeClass::Part(p) = class {
                parts.push(p as usize);
            }
            let nch = led.map_or(0, |k| self.led_devices[k].channel_count);
            let nodes = if matches!(class, ChargeClass::Line(_)) { nodes.unwrap_or_default() } else { Vec::new() };
            devs.push(ArmedDev { device: d, slot, nch, class, led: led.map(|k| k as u32), ne555, nodes, memo: Vec::new(), i: 0.0, ch: vec![0.0; nch], since: now, key: Vec::new() });
            slot += 1 + nch;
        }
        let src = self.source_terms(devices, &mut parts, &mut reads, &mut line_nodes);
        // Whatever follows the hub rail (a line, a mean on it, a rail
        // capacitor, a source's walk) folds at a carrier's points while one
        // stepped hub partition carries the rail (`charge_rail_carrier`), so
        // every stepped hub partition keeps its steps for the charge, however
        // little of it is armed (W2bV: a resistor across the rail armed alone
        // read 0, its carrier's steps never kept).
        if let Some(rail) = rail {
            let hub_cap = |d: usize| self.fixed_pair_capacitors.iter().any(|c| c.device_index == d && (c.nodes.0 == rail || c.nodes.1 == rail));
            let follows = src.is_some() || devs.iter().any(|d| matches!(d.class, ChargeClass::Line(_) | ChargeClass::Mean(_)) || (matches!(d.class, ChargeClass::FixedCap(_)) && hub_cap(d.device)));
            if follows {
                parts.extend(self.stepped.all_hub_partitions());
            }
        }
        parts.sort_unstable();
        parts.dedup();
        line_nodes.sort_unstable();
        line_nodes.dedup();
        line_nodes.retain(|&n| rail == Some(n) || self.local_index[n].is_some_and(|(p, _)| self.partition_kinds[p] == PartitionKind::Static));
        let stepped: Vec<(usize, Vec<usize>)> = parts.iter().map(|&p| (p, reads.remove(&p).unwrap_or_default())).collect();
        self.stepped.set_charge(Some(&stepped));
        let mut st = ChargeState { devs, q: vec![0.0; slot], t0: now, cursor: now, t_arm: now, parts, line_nodes, src, sc: vec![0.0; self.currents.len()], saved: Vec::new(), chs: Vec::new(), steps: Vec::new(), published: Vec::new(), pending: Vec::new(), node_start: Vec::new(), node_deps: Vec::new(), watched: Vec::new(), dev_held: Vec::new(), dirty: Vec::new(), is_dirty: Vec::new(), holds_seen: self.hold_counts(), line_moving: self.rail_moving(), has_interval: false, has_line: false, reads_rail: false, line_flag: Vec::new(), carrier: None, interval_devs: Vec::new(), fixed_caps: Vec::new(), rail_q: None };
        st.interval_devs = (0..st.devs.len() as u32).filter(|&k| matches!(st.devs[k as usize].class, ChargeClass::Mean(_) | ChargeClass::Line(_) | ChargeClass::FixedCap(_) | ChargeClass::Part(_))).collect();
        st.fixed_caps = (0..self.fixed_pair_capacitors.len() as u32)
            .filter(|&k| {
                let cap = &self.fixed_pair_capacitors[k as usize];
                st.devs.iter().any(|d| matches!(d.class, ChargeClass::FixedCap(j) if j == k)) || rel_of(&st.src, cap.nodes.0) || rel_of(&st.src, cap.nodes.1)
            })
            .collect();
        st.line_flag = (0..self.voltages.len()).map(|n| rail == Some(n) || (self.local_index[n].is_some_and(|(p, _)| self.partition_kinds[p] == PartitionKind::Static) && self.static_solver.hub_node_line(n).is_some())).collect();
        st.carrier = self.charge_rail_carrier();
        st.has_interval = st.src.is_some() || st.devs.iter().any(|d| matches!(d.class, ChargeClass::Mean(_) | ChargeClass::FixedCap(_) | ChargeClass::Part(_)));
        st.has_line = st.devs.iter().any(|d| matches!(d.class, ChargeClass::Line(_)));
        st.reads_rail = self.hub.is_some() && (st.src.is_some() || st.devs.iter().any(|d| matches!(d.class, ChargeClass::Line(_) | ChargeClass::Mean(_))));
        // What can move each held class (`ChargeState::note_node`).
        let mut per_node: Vec<Vec<u32>> = vec![Vec::new(); self.voltages.len()];
        st.dev_held = vec![u32::MAX; self.currents.len()];
        for (k, dev) in st.devs.iter().enumerate() {
            match dev.class {
                ChargeClass::Held => st.dev_held[dev.device] = k as u32,
                ChargeClass::HeldLed(j) => {
                    for jn in &self.led_devices[j as usize].junctions {
                        per_node[jn.anode].push(k as u32);
                        per_node[jn.cathode].push(k as u32);
                    }
                }
                ChargeClass::Held555(pos) => {
                    let ic = &self.ics[pos as usize].1;
                    for n in [ic.logic[digital::NE555_OUT].node, ic.gnd.node, ic.vcc.node] {
                        per_node[n].push(k as u32);
                    }
                }
                ChargeClass::Line(_) => {
                    for &n in &dev.nodes {
                        per_node[n].push(k as u32);
                    }
                }
                _ => {}
            }
        }
        st.watched = vec![0; per_node.len().div_ceil(64)];
        for (n, list) in per_node.iter().enumerate() {
            if !list.is_empty() {
                st.watched[n / 64] |= 1 << (n % 64);
            }
        }
        st.node_start.push(0);
        for list in &per_node {
            st.node_deps.extend_from_slice(list);
            st.node_start.push(st.node_deps.len() as u32);
        }
        st.is_dirty = vec![false; st.devs.len()];
        if let Some(s) = st.src.as_mut() {
            s.cc_prev = self.cc_hub_current().is_some();
        }
        for k in 0..st.devs.len() {
            let (i, key) = self.held_now(&st.devs[k], &mut st.devs[k].ch.clone(), &drivers);
            let dev = &mut st.devs[k];
            if let Some((i, ch)) = i {
                dev.i = i;
                dev.ch = ch;
            }
            dev.key = key;
        }
        self.charge = Some(Box::new(st));
    }

    /// The KCL terms of the armed sources: the nodes their walk reads (each
    /// source's child node and every node fixed under it), what leaves them,
    /// and the dynamic partitions those terms sit in, with the nodes outside
    /// a stepped one they read.
    fn source_terms(&self, devices: &[usize], parts: &mut Vec<usize>, reads: &mut HashMap<usize, Vec<usize>>, line_nodes: &mut Vec<usize>) -> Option<SourceTerms> {
        let armed: Vec<&partition::ResolvedSource> = self.resolved.iter().filter(|r| devices.contains(&r.device_index)).collect();
        if armed.is_empty() {
            return None;
        }
        let n = self.voltages.len();
        let mut rel = vec![false; n];
        // Parents before children (the walk runs this order backwards).
        for r in self.resolved.iter() {
            if devices.contains(&r.device_index) || (r.parent_node != 0 && rel[r.parent_node]) {
                rel[r.child_node] = true;
            }
        }
        rel[0] = false;
        let rel_nodes: Vec<usize> = (0..n).filter(|&k| rel[k]).collect();
        let mut own_parts: Vec<usize> = Vec::new();
        let mut note = |owner: Option<usize>, nodes: &[usize]| {
            line_nodes.extend_from_slice(nodes);
            if let Some(p) = owner.filter(|&p| self.partition_kinds[p] != PartitionKind::Static) {
                parts.push(p);
                own_parts.push(p);
                reads.entry(p).or_default().extend_from_slice(nodes);
            }
        };
        let mut kcl = Vec::new();
        for (k, kd) in self.kcl_devices.iter().enumerate() {
            if kd.nodes.iter().any(|&m| rel[m]) {
                kcl.push(k as u32);
                let owner = kd.nodes.iter().find_map(|&m| self.local_index[m].map(|(p, _)| p));
                note(owner, &kd.nodes);
            }
        }
        let mut caps = Vec::new();
        for (k, &(_, u, w)) in self.cap_devices.iter().enumerate() {
            if rel[u] || rel[w] {
                caps.push(k as u32);
                note(self.owner_of(u, w), &[u, w]);
            }
        }
        let mut junctions = Vec::new();
        for (k, j) in self.junctions.iter().enumerate() {
            if rel[j.anode] || rel[j.cathode] {
                junctions.push(k as u32);
                note(self.owner_of(j.anode, j.cathode), &[j.anode, j.cathode]);
            }
        }
        let mut ics = Vec::new();
        for (pos, (_, ic)) in self.ics.iter().enumerate() {
            let nodes: Vec<usize> = ic.logic.iter().map(|p| p.node).chain([ic.vcc.node, ic.gnd.node]).collect();
            if nodes.iter().any(|&m| rel[m]) {
                ics.push(pos as u32);
                for &m in &nodes {
                    if let Some((p, _)) = self.local_index[m] {
                        note(Some(p), &nodes);
                    }
                }
            }
        }
        let cc_device = self.hub.as_ref().map(|h| h.device).or(self.exact.as_ref().map(|e| e.device));
        own_parts.sort_unstable();
        own_parts.dedup();
        // The armed sources whose walk reads the hub rail.
        let on_rail = match self.hub.as_ref().map(|h| h.node) {
            Some(rail) => armed
                .iter()
                .filter(|a| {
                    let mut mine = vec![false; n];
                    for r in self.resolved.iter() {
                        if r.device_index == a.device_index || (r.parent_node != 0 && mine[r.parent_node]) {
                            mine[r.child_node] = true;
                        }
                    }
                    mine[rail]
                })
                .map(|a| a.device_index)
                .collect(),
            None => Vec::new(),
        };
        Some(SourceTerms { rel, rel_nodes, kcl, caps, junctions, ics, lq: [vec![0.0; n], vec![0.0; n], vec![0.0; n]], cc_q: 0.0, cc_device, parts: own_parts, on_rail, approximate: false, cc_prev: false })
    }

    /// A held class's current and channels on the present voltages, if it
    /// is one, and what they are read from (a published current's bits, an
    /// LED's node voltages and failure, a 555's current).
    #[allow(clippy::type_complexity)]
    fn held_now(&self, dev: &ArmedDev, ch: &mut [f64], drivers: &[Driver]) -> (Option<(f64, Vec<f64>)>, Vec<u64>) {
        let v = &self.voltages;
        match dev.class {
            ChargeClass::Held => (Some((self.currents[dev.device], Vec::new())), vec![self.currents[dev.device].to_bits()]),
            ChargeClass::HeldLed(k) => {
                let e = &self.led_devices[k as usize];
                let failed = self.slow.is_failed(e.device_index);
                let mut key: Vec<u64> = e.junctions.iter().flat_map(|j| [v[j.anode].to_bits(), v[j.cathode].to_bits()]).collect();
                key.push(u64::from(failed));
                let mut i = 0.0;
                led::readout(&e.device, &e.junctions, v, failed, &mut i, ch);
                (Some((i, ch.to_vec())), key)
            }
            ChargeClass::Held555(pos) => {
                let i = self.ne555_current_on(pos as usize, v, drivers);
                (Some((i, Vec::new())), vec![i.to_bits()])
            }
            ChargeClass::Line(_) => {
                let i = self.line_current_on(dev, v, drivers, ch);
                let mut key: Vec<u64> = dev.nodes.iter().map(|&n| v[n].to_bits()).collect();
                key.push(i.to_bits());
                (Some((i, ch.to_vec())), key)
            }
            _ => (None, Vec::new()),
        }
    }

    /// Whether a held class's inputs moved since its key.
    fn held_moved(&self, dev: &ArmedDev, drivers: &[Driver]) -> bool {
        let v = &self.voltages;
        match dev.class {
            ChargeClass::Held => dev.key[0] != self.currents[dev.device].to_bits(),
            ChargeClass::HeldLed(k) => {
                let e = &self.led_devices[k as usize];
                let mut it = dev.key.iter();
                for j in &e.junctions {
                    if it.next() != Some(&v[j.anode].to_bits()) || it.next() != Some(&v[j.cathode].to_bits()) {
                        return true;
                    }
                }
                it.next() != Some(&u64::from(self.slow.is_failed(e.device_index)))
            }
            ChargeClass::Held555(pos) => dev.key[0] != self.ne555_current_on(pos as usize, v, drivers).to_bits(),
            ChargeClass::Line(_) => {
                let mut ch = vec![0.0; dev.nch];
                let i = self.line_current_on(dev, v, drivers, &mut ch);
                dev.nodes.iter().map(|&n| v[n].to_bits()).chain(std::iter::once(i.to_bits())).ne(dev.key.iter().copied())
            }
            _ => false,
        }
    }

    /// Whether a device's current follows the hub rail: a line, a mean
    /// with a node on the rail, or a rail capacitor.
    fn charge_on_line(&self, dev: &ArmedDev, line: &[bool]) -> bool {
        match dev.class {
            ChargeClass::Line(_) => true,
            ChargeClass::FixedCap(j) => self.hub_cap_share(j as usize).is_some(),
            ChargeClass::Mean(Some(kd)) => self.kcl_devices[kd as usize].nodes.iter().any(|&n| line[n]),
            ChargeClass::Mean(None) => match (dev.led, dev.ne555) {
                (Some(k), _) => self.led_devices[k as usize].junctions.iter().any(|j| line[j.anode] || line[j.cathode]),
                (_, Some(pos)) => {
                    let ic = &self.ics[pos as usize].1;
                    line[ic.vcc.node] || line[ic.gnd.node] || line[ic.logic[digital::NE555_OUT].node]
                }
                _ => false,
            },
            _ => false,
        }
    }

    /// A line's or a mean's current on `v`: a resistor's or switch's
    /// contacts, an LED's readout (its channels into `ch`) or a 555's.
    fn line_current_on(&self, dev: &ArmedDev, v: &[f64], drivers: &[Driver], ch: &mut [f64]) -> f64 {
        match dev.class {
            ChargeClass::Line(Some(kd)) | ChargeClass::Mean(Some(kd)) => self.kcl_devices[kd as usize].contributions.iter().map(|&(u, x, g)| g * (v[u] - v[x])).sum::<f64>(),
            _ => self.charge_current_on(dev, v, &[], drivers, ch),
        }
    }

    /// `publish_ne555_currents` for one 555, on `v`.
    fn ne555_current_on(&self, pos: usize, v: &[f64], drivers: &[Driver]) -> f64 {
        let (device_index, ic) = &self.ics[pos];
        let power = digital::power::ic_power_held(ic, v[ic.vcc.node], v[ic.gnd.node], self.power_hold(*device_index));
        let committed = self.ic_driver_ids[pos].iter().map(|&id| &drivers[id as usize]).find(|d| d.stage == DriverStage::Ne555Out).and_then(|d| d.committed());
        match (power.powered, committed) {
            (true, Some(level @ (Mode::High | Mode::Low))) => digital::ne555::element_current(level == Mode::High, power.vs, v[ic.logic[digital::NE555_OUT].node], v[ic.gnd.node], v[ic.vcc.node]),
            _ => 0.0,
        }
    }

    /// A fixed-pair capacitor's mean current over `[a, b]`: `C dvc` on its
    /// exponential and its leakage, or a CC rail capacitor's share of the
    /// rail's charge.
    fn fixed_cap_mean(&self, k: usize, a: f64, b: f64, v: &[f64]) -> f64 {
        let cap = &self.fixed_pair_capacitors[k];
        let dt = b - a;
        if let Some(hub) = self.hub.as_ref() {
            if let Some(seg) = &hub.state.cap_seg {
                if hub.caps.iter().any(|&(j, _)| j as usize == k) {
                    let sign = if cap.nodes.0 == hub.node { 1.0 } else { -1.0 };
                    return sign * cap.c * (seg.charge_at(b) - seg.charge_at(a)) / dt + cap.leak_g * (v[cap.nodes.0] - v[cap.nodes.1]);
                }
            }
        }
        let series = if cap.esr > 0.0 { cap.c * (cap.vc_at(b) - cap.vc_at(a)) / dt } else { 0.0 };
        series + cap.leak_g * cap.d_v
    }

    /// A rail capacitor's share of the rail capacitors' current, `C_k /
    /// C_rail` with the sign of the rail at its first pin; `None` for any
    /// other fixed-pair capacitor.
    fn hub_cap_share(&self, k: usize) -> Option<f64> {
        let hub = self.hub.as_ref()?;
        hub.caps.iter().any(|&(j, _)| j as usize == k).then(|| {
            let cap = &self.fixed_pair_capacitors[k];
            (if cap.nodes.0 == hub.node { 1.0 } else { -1.0 }) * cap.c / hub.c_rail
        })
    }

    /// Writes `value` over `v[node]`, keeping what was there.
    fn charge_put(saved: &mut Vec<(usize, f64)>, v: &mut [f64], node: usize, value: f64) {
        saved.push((node, v[node]));
        v[node] = value;
    }

    fn charge_restore(saved: &mut Vec<(usize, f64)>, v: &mut [f64]) {
        while let Some((node, value)) = saved.pop() {
            v[node] = value;
        }
    }

    /// The device current of armed device `dev` on `v`, with the partition
    /// currents in `sc`, and its channels into `ch`.
    fn charge_current_on(&self, dev: &ArmedDev, v: &[f64], sc: &[f64], drivers: &[Driver], ch: &mut [f64]) -> f64 {
        if let Some(k) = dev.led {
            let e = &self.led_devices[k as usize];
            let mut i = 0.0;
            led::readout(&e.device, &e.junctions, v, self.slow.is_failed(e.device_index), &mut i, ch);
            return i;
        }
        if let Some(pos) = dev.ne555 {
            return self.ne555_current_on(pos as usize, v, drivers);
        }
        sc[dev.device]
    }

    /// `∫ leaving` over a stretch of weight `w` on `v` (`recompute_source_currents`'s
    /// `leaving`, at the walk's nodes only), from the terms `include` takes by
    /// the partition they sit in (`None`: between fixed nodes), the
    /// capacitors' currents from `cap`.
    #[allow(clippy::too_many_arguments)]
    fn charge_leaving(&self, src: &mut SourceTerms, bucket: usize, v: &[f64], cap: &dyn Fn(usize) -> f64, include: &dyn Fn(Option<usize>, bool) -> bool, line: &[bool], t: f64, w: f64, drivers: &[Driver]) {
        let rel = std::mem::take(&mut src.rel);
        // Whether a term follows the hub rail: one of its nodes is the rail
        // or on a static hub partition's line in it.
        let on = |n: usize| line.get(n).copied().unwrap_or(false);
        let lq = &mut src.lq[bucket];
        let mut add = |u: usize, x: usize, i: f64| {
            if rel[u] {
                lq[u] += w * i;
            }
            if rel[x] {
                lq[x] -= w * i;
            }
        };
        for &k in &src.kcl {
            let kd = &self.kcl_devices[k as usize];
            for &(u, x, g) in &kd.contributions {
                if (rel[u] || rel[x]) && include(self.owner_of(u, x), on(u) || on(x)) {
                    add(u, x, g * (v[u] - v[x]));
                }
            }
        }
        for &n in &src.rel_nodes {
            if include(None, on(n)) {
                add(n, 0, static_part::NODE_RSHUNT_G * v[n]);
            }
        }
        for &k in &src.caps {
            let (d, u, x) = self.cap_devices[k as usize];
            if include(self.owner_of(u, x), on(u) || on(x)) {
                add(u, x, cap(d));
            }
        }
        for &k in &src.junctions {
            let j = &self.junctions[k as usize];
            if (self.slow.may_have_failed() && self.slow.is_failed(j.device)) || !include(self.owner_of(j.anode, j.cathode), on(j.anode) || on(j.cathode)) {
                continue;
            }
            add(j.anode, j.cathode, junction_branch_current(j, v));
        }
        for &pos in &src.ics {
            let (device_index, ic) = &self.ics[pos as usize];
            let power = digital::power::ic_power_held(ic, v[ic.vcc.node], v[ic.gnd.node], self.power_hold(*device_index));
            if !power.powered {
                continue;
            }
            // An IC on the rail drives at the rail's power.
            let supply = on(ic.vcc.node) || on(ic.gnd.node);
            for &id in &self.ic_driver_ids[pos as usize] {
                let driver = &drivers[id as usize];
                for branch in ic_driver_branches(&power, driver, ic) {
                    let Some((target, g)) = branch else { continue };
                    if include(self.owner_of(driver.node, target), supply || on(driver.node) || on(target)) {
                        add(driver.node, target, g * (v[driver.node] - v[target]));
                    }
                }
            }
            for pin in &ic.logic {
                if !rel[pin.node] || !include(self.owner_of(pin.node, pin.node), supply || on(pin.node)) {
                    continue;
                }
                if let Some((g, target_v)) = ic_pull_target(ic, &power, pin, t) {
                    add(pin.node, 0, g * (v[pin.node] - target_v));
                }
            }
        }
        src.rel = rel;
    }

    /// The kernel's `observe` hook for the charge: the steps committed since
    /// the last fold, the held currents the last instant changed (from
    /// `now`), and the interval up to `t` on the state at `now`.
    pub(crate) fn charge_observe(&mut self, t: f64, now: f64) {
        let Some(mut st) = self.charge.take() else { return };
        debug_assert!(!self.window, "the charge folds between instants");
        self.charge_fold_steps(&mut st);
        self.charge_fold_held(&mut st, now);
        self.charge_line_regime(&mut st, now);
        self.charge_resolve_rail(&mut st);
        if st.reads_rail || !st.fixed_caps.is_empty() {
            st.carrier = self.charge_rail_carrier();
        }
        self.charge_rail_jump(&mut st);
        if t > st.cursor {
            let a = st.cursor;
            self.charge_fold_interval(&mut st, a, t);
            st.cursor = t;
            if st.rail_q.is_some() {
                st.rail_q = self.hub.as_ref().and_then(|h| h.state.cap_seg.as_ref()).map(|seg| seg.charge_at(t));
            }
        }
        if let Some(s) = st.src.as_mut() {
            s.cc_prev = self.cc_hub_current().is_some();
        }
        self.charge = Some(st);
    }

    /// The rail capacitors' charge at the cursor on the CC segment as it is
    /// now, against where the last fold left it: a rail solve after a stepped
    /// hub partition's commit re-aims the segment from the charge that step
    /// handed it (ruling R68), so the segment the last fold read was only the
    /// frozen draws' estimate. The difference is the capacitors' own charge
    /// (their state moved by it), charged to them at once, so a rail
    /// capacitor's charge is `C dV` of its readout however many stepped hub
    /// partitions move it. Where one stepped hub partition carries the rail
    /// its steps fold the capacitors instead (`charge_rail_carrier`).
    fn charge_rail_jump(&self, st: &mut ChargeState) {
        let seg = self.hub.as_ref().and_then(|h| h.state.cap_seg.as_ref()).filter(|_| st.carrier.is_none() && !st.fixed_caps.is_empty());
        let Some(seg) = seg else {
            st.rail_q = None;
            return;
        };
        let q_now = seg.charge_at(st.cursor);
        let Some(prev) = st.rail_q.replace(q_now) else { return };
        let dq = q_now - prev;
        if dq == 0.0 {
            return;
        }
        let c_rail = self.hub.as_ref().map_or(0.0, |h| h.c_rail);
        let bucket = usize::from(self.cc_hub_current().is_some());
        for &j in &st.fixed_caps {
            let Some(share) = self.hub_cap_share(j as usize) else { continue };
            let cap = &self.fixed_pair_capacitors[j as usize];
            let q = share * c_rail * dq;
            for dev in &st.devs {
                if matches!(dev.class, ChargeClass::FixedCap(k) if k == j) {
                    st.q[dev.slot] += q;
                }
            }
            if let Some(src) = st.src.as_mut() {
                let (u, x) = cap.nodes;
                if src.rel[u] {
                    src.lq[bucket][u] += q;
                }
                if src.rel[x] {
                    src.lq[bucket][x] -= q;
                }
            }
        }
    }

    /// The stepped hub partition whose steps carry the rail, if one does: a
    /// PSU in CC with no rail capacitor, its rail set by the one stepped hub
    /// partition's draws and solved at each point of its steps; or a PSU in
    /// CC with rail capacitors that partition steps as a companion on its
    /// rail row (ruling R68), handing their charge to the rail solve after
    /// each commit, so the CC segment between solves is only the frozen
    /// draws' estimate that the next solve replaces (W2bV: folded on the
    /// segment, the capacitors' charge missed the jump at each solve, up to
    /// 1.2e-2 of `C dV`, and the PSU's KCL 1-7 W0). The rail's lines and
    /// capacitors then fold at that partition's points with its weights, as
    /// its own draws do (a capacitor at its companion current there), so the
    /// PSU's `-dir iLimit` is the KCL of what leaves the rail. With several
    /// stepped hub partitions each step solves the rail against the others'
    /// draws held where they last committed, so no one trajectory exists:
    /// the lines then fold linear between the rail's solves (`RailPending`),
    /// or on the segment, and the PSU's KCL holds only to the steps' coupling
    /// error.
    pub(crate) fn charge_rail_carrier(&self) -> Option<usize> {
        if !self.charge_rail_stepped() {
            return None;
        }
        match self.stepped.hub_partitions()[..] {
            [p] => Some(p),
            _ => None,
        }
    }

    /// Whether the stepped hub partitions step the hub rail itself: a PSU in
    /// CC with no rail capacitor (its rail an unknown of their steps), or
    /// with rail capacitors and no closed form coupled to them (their charge
    /// a companion on the steps' rail row, `sync_stepped_cc`).
    fn charge_rail_stepped(&self) -> bool {
        if self.cc_hub_current().is_none() {
            return false;
        }
        let stepped_caps = self.hub.as_ref().is_some_and(|h| h.c_rail > 0.0 && h.state.cap_seg.as_ref().is_some_and(|s| s.modes.is_none()) && !h.cf_hubs.iter().any(|&p| h.contrib[p].active));
        self.rail_unmodelled() || (stepped_caps && self.stepped.any_hub_stepping())
    }

    /// Whether the charge reads the hub rail's notes (`log_rail`).
    pub(crate) fn charge_reads_rail(&self) -> bool {
        self.charge.as_ref().is_some_and(|st| st.reads_rail)
    }

    /// The earliest time the charge still reads the rail's notes at: its
    /// oldest interval waiting on the rail, else `now`.
    pub(crate) fn charge_rail_from(&self, now: f64) -> f64 {
        self.charge.as_ref().map_or(now, |st| st.pending.iter().fold(now, |m, p| m.min(p.a)))
    }

    /// A line between its two foldings: when the rail starts to move between
    /// instants, its held value closes at `now` and the intervals take it
    /// over; when the rail holds again, it is held from `now`.
    fn charge_line_regime(&mut self, st: &mut ChargeState, now: f64) {
        if !st.has_line {
            return;
        }
        let moving = self.rail_moving();
        if moving == st.line_moving {
            return;
        }
        if moving {
            Self::charge_close_held(st, now);
            st.line_moving = true;
            return;
        }
        st.line_moving = false;
        let drivers = Rc::clone(&self.driver_snapshot);
        for k in 0..st.devs.len() {
            if !matches!(st.devs[k].class, ChargeClass::Line(_)) {
                continue;
            }
            let mut ch = std::mem::take(&mut st.devs[k].ch);
            let (value, key) = self.held_now(&st.devs[k], &mut ch, &drivers);
            let dev = &mut st.devs[k];
            if let Some((i, chs)) = value {
                dev.i = i;
                ch = chs;
            }
            dev.ch = ch;
            dev.key = key;
            dev.since = dev.since.max(now);
        }
    }

    /// Refolds each held class whose published value moved, at `now`.
    fn charge_fold_held(&mut self, st: &mut ChargeState, now: f64) {
        let drivers = Rc::clone(&self.driver_snapshot);
        // The claim the logs carry: a held class no instant marked has not
        // moved (a debug build checks every one at every fold).
        #[cfg(debug_assertions)]
        for (k, dev) in st.devs.iter().enumerate() {
            debug_assert!(st.is_dirty[k] || !st.held(&dev.class) || !self.held_moved(dev, &drivers), "charge: device {} moved with no instant's log saying so", dev.device);
        }
        if st.dirty.is_empty() {
            return;
        }
        let dirty = std::mem::take(&mut st.dirty);
        for &k in &dirty {
            let k = k as usize;
            st.is_dirty[k] = false;
            if !st.held(&st.devs[k].class) || !self.held_moved(&st.devs[k], &drivers) {
                continue;
            }
            {
                let dev = &st.devs[k];
                let (slot, dt) = (dev.slot, (now - dev.since).max(0.0));
                st.q[slot] += dev.i * dt;
                for (c, &value) in dev.ch.iter().enumerate() {
                    st.q[slot + 1 + c] += value * dt;
                }
            }
            if let ChargeClass::HeldLed(j) = st.devs[k].class {
                self.charge_led_state(&mut st.devs[k], j as usize);
                st.devs[k].since = st.devs[k].since.max(now);
                continue;
            }
            let mut ch = std::mem::take(&mut st.devs[k].ch);
            let (value, key) = self.held_now(&st.devs[k], &mut ch, &drivers);
            let dev = &mut st.devs[k];
            if let Some((i, chs)) = value {
                dev.i = i;
                ch = chs;
            }
            dev.ch = ch;
            dev.key = key;
            dev.since = dev.since.max(now);
        }
    }

    /// A held LED's new state, `(key, current, channels)`, without
    /// allocating: back from its memo when its node bits come back (a logic
    /// output toggling it re-solves its partition to the same bits, the same
    /// readout), else its readout into the memo's oldest buffers; the state
    /// it leaves goes to the memo's front.
    fn charge_led_state(&self, dev: &mut ArmedDev, j: usize) {
        let e = &self.led_devices[j];
        let v = &self.voltages;
        let failed = self.slow.is_failed(e.device_index);
        let matches = |key: &[u64]| {
            let mut it = key.iter();
            e.junctions.iter().all(|jn| it.next() == Some(&v[jn.anode].to_bits()) && it.next() == Some(&v[jn.cathode].to_bits())) && it.next() == Some(&u64::from(failed))
        };
        if let Some(m) = dev.memo.iter().position(|m| matches(&m.0)) {
            let entry = &mut dev.memo[m];
            std::mem::swap(&mut dev.key, &mut entry.0);
            std::mem::swap(&mut dev.i, &mut entry.1);
            std::mem::swap(&mut dev.ch, &mut entry.2);
            // The same bits in, the same readout out (a debug build checks).
            #[cfg(debug_assertions)]
            {
                let (mut i, mut c) = (0.0, vec![0.0; dev.ch.len()]);
                led::readout(&e.device, &e.junctions, v, failed, &mut i, &mut c);
                debug_assert!(i.to_bits() == dev.i.to_bits() && c.iter().zip(&dev.ch).all(|(a, b)| a.to_bits() == b.to_bits()), "charge: an LED's memo differs from its readout");
            }
            if m != 0 {
                dev.memo.swap(0, m);
            }
            return;
        }
        let mut next = if dev.memo.len() >= 2 { dev.memo.pop().expect("two kept") } else { (Vec::new(), 0.0, vec![0.0; dev.ch.len()]) };
        next.0.clear();
        next.0.extend(e.junctions.iter().flat_map(|jn| [v[jn.anode].to_bits(), v[jn.cathode].to_bits()]));
        next.0.push(u64::from(failed));
        led::readout(&e.device, &e.junctions, v, failed, &mut next.1, &mut next.2);
        let left = (std::mem::replace(&mut dev.key, next.0), std::mem::replace(&mut dev.i, next.1), std::mem::replace(&mut dev.ch, next.2));
        dev.memo.insert(0, left);
    }

    /// Every held class up to `t`, its current and channels times the time
    /// they held.
    fn charge_close_held(st: &mut ChargeState, t: f64) {
        let line_moving = st.line_moving;
        for dev in &mut st.devs {
            let held = matches!(dev.class, ChargeClass::Held | ChargeClass::HeldLed(_) | ChargeClass::Held555(_)) || (matches!(dev.class, ChargeClass::Line(_)) && !line_moving);
            if !held {
                continue;
            }
            let dt = (t - dev.since).max(0.0);
            st.q[dev.slot] += dev.i * dt;
            for (c, &value) in dev.ch.iter().enumerate() {
                st.q[dev.slot + 1 + c] += value * dt;
            }
            dev.since = dev.since.max(t);
        }
    }

    /// The interval `[a, b]` between instants, on the state the last one
    /// left: the closed forms at their means, the lines (ramped, or on the
    /// hub rail) at theirs, the fixed-pair capacitors on their exponentials,
    /// and the source terms outside stepped partitions. Where the rail moves
    /// with nothing to model it (ruling (a): linear between its solves), the
    /// interval is folded at the rail as last solved and its sensitivity to
    /// the rail kept until the next solve gives the rail's mean there
    /// (`charge_resolve_rail`).
    fn charge_fold_interval(&mut self, st: &mut ChargeState, a: f64, b: f64) {
        if !st.has_interval && !(st.has_line && st.line_moving) {
            return;
        }
        let mut q = std::mem::take(&mut st.q);
        let unmodelled = self.rail_unmodelled() && !st.line_nodes.is_empty() && st.carrier.is_none();
        if !unmodelled {
            self.charge_interval_pass(st, a, b, 0.0, &mut q, None);
            st.q = q;
            return;
        }
        // Twice: at the rail as solved and a volt above it, each into its own
        // sums; the first is added now, their difference waits for the rail.
        let n = q.len();
        let (mut q0, mut q1) = (vec![0.0; n], vec![0.0; n]);
        let u_held = self.rail_mean(a, b).unwrap_or(0.0);
        let bucket = self.charge_interval_pass(st, a, b, 0.0, &mut q0, Some(2));
        let lq0 = st.src.as_mut().map(|s| {
            let n = s.lq[0].len();
            std::mem::replace(&mut s.lq[2], vec![0.0; n])
        });
        self.charge_interval_pass(st, a, b, 1.0, &mut q1, Some(2));
        let mut pending = RailPending { a, b, u_held, slots: Vec::new(), lq: Vec::new() };
        for k in 0..n {
            q[k] += q0[k];
            if q1[k] != q0[k] {
                pending.slots.push((k as u32, q1[k] - q0[k]));
            }
        }
        if let (Some(src), Some(lq0), Some(bucket)) = (st.src.as_mut(), lq0, bucket) {
            let lq1 = std::mem::take(&mut src.lq[2]);
            for (node, (&x0, &x1)) in lq0.iter().zip(&lq1).enumerate() {
                src.lq[bucket][node] += x0;
                if x1 != x0 {
                    pending.lq.push((bucket as u8, node as u32, x1 - x0));
                }
            }
            src.lq[2] = vec![0.0; lq1.len()];
        }
        st.q = q;
        st.pending.push(pending);
    }

    /// Applies each held-rail interval's correction once the rail's solves
    /// bracket it: the rail's mean there, linear between them, against the
    /// rail it was folded at.
    fn charge_resolve_rail(&self, st: &mut ChargeState) {
        if st.pending.is_empty() {
            return;
        }
        let mut keep = Vec::new();
        for p in std::mem::take(&mut st.pending) {
            match (self.rail_at(p.a), self.rail_at(p.b)) {
                (Some((ua, _)), Some((ub, _))) => {
                    let du = 0.5 * (ua + ub) - p.u_held;
                    for &(k, d) in &p.slots {
                        st.q[k as usize] += d * du;
                    }
                    if let Some(src) = st.src.as_mut() {
                        for &(bucket, node, d) in &p.lq {
                            src.lq[bucket as usize][node as usize] += d * du;
                        }
                    }
                }
                _ => keep.push(p),
            }
        }
        st.pending = keep;
    }

    /// One pass over `[a, b]` with the hub rail and its lines `shift` volts
    /// above their means, into `q` and the source terms' bucket (`lq`: the
    /// CC regime's, or the one given); returns the regime's bucket.
    fn charge_interval_pass(&mut self, st: &mut ChargeState, a: f64, b: f64, shift: f64, q: &mut [f64], lq: Option<usize>) -> Option<usize> {
        let dt = b - a;
        let mid = 0.5 * (a + b);
        let drivers = Rc::clone(&self.driver_snapshot);
        let mut v = std::mem::take(&mut self.voltages);
        let mut saved = std::mem::take(&mut st.saved);
        let mut sc = std::mem::take(&mut st.sc);
        // A running ramp's nodes at the interval's mean (they move linearly).
        if self.wave.any_ramping() {
            let moved: Vec<(usize, f64)> = (0..v.len()).filter_map(|n| self.wave.value(n, mid, &v).map(|value| (n, value))).collect();
            for (n, value) in moved {
                Self::charge_put(&mut saved, &mut v, n, value);
            }
        }
        let rail = self.hub.as_ref().map(|h| h.node);
        // The rail at its model's mean, and the static hub partitions' nodes
        // on their lines in it.
        if self.rail_moving() {
            if let Some(u) = self.rail_mean(a, b) {
                let u = u + shift;
                for &n in &st.line_nodes {
                    if rail == Some(n) {
                        Self::charge_put(&mut saved, &mut v, n, u);
                    } else if let Some((a0, s)) = self.static_solver.hub_node_line(n) {
                        Self::charge_put(&mut saved, &mut v, n, a0 + s * u);
                    }
                }
            }
        }
        // The closed forms that published their means (one with no running
        // segment has nothing between instants to add).
        let mut published = std::mem::take(&mut st.published);
        published.clear();
        for &p in &st.parts {
            if self.partition_kinds[p] != PartitionKind::ClosedForm {
                continue;
            }
            for &n in self.dyn_out[p].nodes.iter().chain(rail.iter()) {
                saved.push((n, v[n]));
            }
            if self.closed_form.publish_mean(p, a, b, rail, &self.live_params, &mut v, &mut sc).is_some() {
                published.push(p);
            }
        }
        for &k in &st.fixed_caps {
            let cap = &self.fixed_pair_capacitors[k as usize];
            sc[cap.device_index] = self.fixed_cap_mean(k as usize, a, b, &v);
        }
        let mut chs = std::mem::take(&mut st.chs);
        for &k in &st.interval_devs {
            let dev = &st.devs[k as usize];
            if st.carrier.is_some() && self.charge_on_line(dev, &st.line_flag) {
                continue;
            }
            let i = match dev.class {
                ChargeClass::Mean(Some(kd)) => self.kcl_devices[kd as usize].contributions.iter().map(|&(u, x, g)| g * (v[u] - v[x])).sum::<f64>(),
                ChargeClass::Line(Some(kd)) if st.line_moving => self.kcl_devices[kd as usize].contributions.iter().map(|&(u, x, g)| g * (v[u] - v[x])).sum::<f64>(),
                ChargeClass::FixedCap(j) => sc[self.fixed_pair_capacitors[j as usize].device_index],
                ChargeClass::Mean(None) | ChargeClass::Line(None) if matches!(dev.class, ChargeClass::Mean(_)) || st.line_moving => {
                    chs.clear();
                    chs.resize(dev.nch, 0.0);
                    let i = self.charge_current_on(dev, &v, &sc, &drivers, &mut chs);
                    for (c, &value) in chs.iter().enumerate() {
                        q[dev.slot + 1 + c] += value * dt;
                    }
                    i
                }
                ChargeClass::Part(p) if published.contains(&(p as usize)) => {
                    chs.clear();
                    chs.resize(dev.nch, 0.0);
                    let i = self.charge_current_on(dev, &v, &sc, &drivers, &mut chs);
                    for (c, &value) in chs.iter().enumerate() {
                        q[dev.slot + 1 + c] += value * dt;
                    }
                    i
                }
                _ => continue,
            };
            q[dev.slot] += i * dt;
        }
        st.chs = chs;
        let mut regime = None;
        if let Some(mut src) = st.src.take() {
            let cc = self.cc_hub_current();
            let bucket = usize::from(cc.is_some());
            regime = Some(bucket);
            if let (Some((_, i)), 0.0) = (cc, shift) {
                src.cc_q += i * dt;
            }
            // Several stepped hub partitions step a CC rail (with no
            // capacitor, or its capacitors as their companion): no one rail
            // trajectory closes the KCL (README, "Charge"); said in the take
            // and counted (stat 28).
            if shift == 0.0 && cc.is_some() && st.carrier.is_none() && self.charge_rail_stepped() && !src.on_rail.is_empty() {
                src.approximate = true;
                self.charge_kcl_approx += 1;
            }
            // Static partitions and fixed nodes, and the closed forms that
            // published (a stepped partition's terms come with its steps).
            let kinds = &self.partition_kinds;
            // A line whose rail a stepped partition carries folds with its
            // steps instead.
            let carried = st.carrier.is_some();
            let here = |o: Option<usize>, line: bool| !(carried && line) && o.is_none_or(|p| kinds[p] == PartitionKind::Static || published.contains(&p));
            let cap_i = |d: usize| sc[d];
            self.charge_leaving(&mut src, lq.unwrap_or(bucket), &v, &cap_i, &here, &st.line_flag, mid, dt, &drivers);
            st.src = Some(src);
        }
        st.published = published;
        Self::charge_restore(&mut saved, &mut v);
        self.voltages = v;
        st.saved = saved;
        st.sc = sc;
        regime
    }

    /// The steps the stepped partitions committed since the last fold, each
    /// by its own quadrature over its points.
    fn charge_fold_steps(&mut self, st: &mut ChargeState) {
        let mut steps = std::mem::take(&mut st.steps);
        self.stepped.drain_charge_steps(&mut steps);
        if steps.is_empty() {
            st.steps = steps;
            return;
        }
        let drivers = Rc::clone(&self.driver_snapshot);
        let mut v = std::mem::take(&mut self.voltages);
        let mut saved = std::mem::take(&mut st.saved);
        let mut sc = std::mem::take(&mut st.sc);
        let mut chs = std::mem::take(&mut st.chs);
        let mut src = st.src.take();
        let bucket = usize::from(src.as_ref().is_some_and(|s| s.cc_prev));
        for step in &steps {
            let p = step.p;
            let Some(shape) = self.stepped.shape_of(p) else { continue };
            let reads = self.stepped.charge_reads(p);
            let mut w = step.weights();
            if step.ends_only {
                // Planned before the charge was armed: its end over the part
                // after arming.
                w.iter_mut().for_each(|x| *x = 0.0);
                w[1] = step.end() - step.t0.max(st.t_arm);
            } else if step.t0 < st.t_arm {
                let frac = (step.end() - st.t_arm) / (step.end() - step.t0);
                w.iter_mut().for_each(|x| *x *= frac);
            }
            for (k, &wk) in w.iter().enumerate() {
                if wk == 0.0 {
                    continue;
                }
                let (x, i) = if k == 0 { (&step.x0, &step.i0) } else { (&step.knots[k - 1].x, &step.knots[k - 1].i) };
                for (r, &n) in reads.iter().enumerate() {
                    Self::charge_put(&mut saved, &mut v, n, step.vals[k * reads.len() + r]);
                }
                for (l, &n) in shape.nodes.iter().enumerate() {
                    Self::charge_put(&mut saved, &mut v, n, x[l]);
                }
                for (j, c) in shape.capacitors.iter().enumerate() {
                    let (va, vb) = (pin_voltage_global(&c.pins[0], &v), pin_voltage_global(&c.pins[1], &v));
                    sc[c.device_index] = i[j] + c.leak_g * (va - vb);
                }
                publish_stamped_currents(shape, &v, &self.live_params, &mut sc);
                // The partition carrying the rail: the rail's static lines at
                // this point's rail, its capacitors at their companion's
                // current, and what follows them weighed here too.
                let carries = st.carrier == Some(p);
                if carries {
                    if let Some(u) = self.hub.as_ref().map(|h| v[h.node]) {
                        for &n in &st.line_nodes {
                            if let Some((a0, s)) = self.static_solver.hub_node_line(n) {
                                Self::charge_put(&mut saved, &mut v, n, a0 + s * u);
                            }
                        }
                    }
                    let i_rail = if k == 0 { step.rail_i0 } else { step.knots[k - 1].rail_i };
                    for &j in &st.fixed_caps {
                        let Some(share) = self.hub_cap_share(j as usize) else { continue };
                        debug_assert!(i_rail.is_some(), "charge: a step carrying rail capacitors kept their current at each weighed point");
                        let cap = &self.fixed_pair_capacitors[j as usize];
                        sc[cap.device_index] = share * i_rail.unwrap_or(0.0) + cap.leak_g * (v[cap.nodes.0] - v[cap.nodes.1]);
                    }
                }
                for dev in &st.devs {
                    if carries && self.charge_on_line(dev, &st.line_flag) {
                        chs.clear();
                        chs.resize(dev.nch, 0.0);
                        let cur = if let ChargeClass::FixedCap(j) = dev.class { sc[self.fixed_pair_capacitors[j as usize].device_index] } else { self.line_current_on(dev, &v, &drivers, &mut chs) };
                        st.q[dev.slot] += wk * cur;
                        for (c, &value) in chs.iter().enumerate() {
                            st.q[dev.slot + 1 + c] += wk * value;
                        }
                        continue;
                    }
                    if !matches!(dev.class, ChargeClass::Part(q) if q as usize == p) {
                        continue;
                    }
                    chs.clear();
                    chs.resize(dev.nch, 0.0);
                    let cur = self.charge_current_on(dev, &v, &sc, &drivers, &mut chs);
                    st.q[dev.slot] += wk * cur;
                    for (c, &value) in chs.iter().enumerate() {
                        st.q[dev.slot + 1 + c] += wk * value;
                    }
                }
                if let Some(s) = src.as_mut() {
                    let kinds = &self.partition_kinds;
                    let own = |o: Option<usize>, line: bool| o == Some(p) || (carries && line && o.is_none_or(|q| kinds[q] == PartitionKind::Static));
                    let cap_i = |d: usize| sc[d];
                    // A floating pull at the step's start: the step stamped
                    // it there (a bucket edge is an instant, which cuts it).
                    self.charge_leaving(s, bucket, &v, &cap_i, &own, &st.line_flag, step.t0, wk, &drivers);
                }
                Self::charge_restore(&mut saved, &mut v);
            }
        }
        self.voltages = v;
        st.saved = saved;
        st.sc = sc;
        st.chs = chs;
        st.src = src;
        steps.clear();
        st.steps = steps;
    }

    /// W2b piece 4's take at `now`: `[t0, t1, q x slots]` into `out`, every
    /// sum cleared and the next span starting at `t1`; `false` when nothing
    /// is armed.
    pub(crate) fn take_charge_at(&mut self, now: f64, out: &mut Vec<f64>) -> bool {
        let Some(mut st) = self.charge.take() else { return false };
        self.charge_fold_steps(&mut st);
        self.charge_fold_held(&mut st, now);
        self.charge_resolve_rail(&mut st);
        self.charge_rail_jump(&mut st);
        let t1 = st.cursor.max(now);
        Self::charge_close_held(&mut st, t1);
        if let Some(src) = st.src.as_mut() {
            // The walk, `recompute_source_currents`'s over the charges, once
            // per regime: in CC the hub PSU is its own charge.
            let sc = &mut st.sc;
            sc.iter_mut().for_each(|x| *x = 0.0);
            let mut extra = vec![0.0; self.voltages.len()];
            for bucket in 0..2 {
                extra.iter_mut().for_each(|x| *x = 0.0);
                for r in self.resolved.iter().rev() {
                    let total = src.lq[bucket][r.child_node] + extra[r.child_node];
                    let mut i_s = if !r.parent_is_pos { -total } else { total };
                    if bucket == 1 && src.cc_device == Some(r.device_index) {
                        i_s = src.cc_q;
                    }
                    sc[r.device_index] += i_s;
                    extra[r.parent_node] += if r.parent_is_pos { i_s } else { -i_s };
                }
                src.lq[bucket].iter_mut().for_each(|x| *x = 0.0);
            }
            src.cc_q = 0.0;
            for dev in &st.devs {
                if matches!(dev.class, ChargeClass::Source) {
                    st.q[dev.slot] = sc[dev.device];
                }
            }
        }
        // How far each device's charge is final: `t1`, but a stepped
        // partition's steps only to its last commit (a yield leaves a step
        // pending; its charge comes with the take after it commits), and a
        // source's as far as every stepped partition its walk reads.
        let stepped_final = |p: usize| if self.partition_kinds[p] == PartitionKind::Stepped { self.stepped.committed_time(p).map_or(t1, |t| t.min(t1)) } else { t1 };
        // What the rail's carrier folds is final to its last commit too.
        let carried_final = st.carrier.map_or(t1, stepped_final);
        let src_final = st.src.as_ref().map_or(t1, |s| s.parts.iter().fold(carried_final, |m, &p| m.min(stepped_final(p))));
        out.clear();
        out.extend([st.t0, t1]);
        out.extend_from_slice(&st.q);
        // An interval still folded at a held rail is final only to its start.
        let held_from = |slots: std::ops::Range<usize>| st.pending.iter().filter(|p| p.slots.iter().any(|&(k, _)| slots.contains(&(k as usize)))).fold(t1, |m, p| m.min(p.a));
        let src_held = st.pending.iter().filter(|p| !p.lq.is_empty()).fold(t1, |m, p| m.min(p.a));
        out.extend(st.devs.iter().map(|d| {
            let f = match d.class {
                ChargeClass::Part(p) => stepped_final(p as usize),
                ChargeClass::Source => src_final.min(src_held),
                _ if st.carrier.is_some() && self.charge_on_line(d, &st.line_flag) => carried_final,
                _ => t1,
            };
            f.min(held_from(d.slot..d.slot + 1 + d.nch))
        }));
        // `kcl_approximate`: a source whose walk reads the rail, in a span
        // where an interval closed it only approximately.
        let approximate = st.src.as_ref().filter(|s| s.approximate);
        out.extend(st.devs.iter().map(|d| f64::from(u8::from(matches!(d.class, ChargeClass::Source) && approximate.is_some_and(|s| s.on_rail.contains(&d.device))))));
        if let Some(s) = st.src.as_mut() {
            s.approximate = false;
        }
        st.q.iter_mut().for_each(|x| *x = 0.0);
        st.t0 = t1;
        st.cursor = t1;
        self.charge = Some(st);
        true
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for ChargeClass {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            ChargeClass::Held => 0,
            ChargeClass::HeldLed(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            ChargeClass::Held555(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            ChargeClass::Mean(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            ChargeClass::Line(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            ChargeClass::Part(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            ChargeClass::FixedCap(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            ChargeClass::Source => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for ArmedDev {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slot, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.class, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.led, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ne555, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.memo, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ch, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.since, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.key, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SourceTerms {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rel, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rel_nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kcl, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.caps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.junctions, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ics, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.lq, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc_q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc_device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.parts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.on_rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.approximate, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cc_prev, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for ChargeState {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.devs, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.q, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cursor, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t_arm, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.parts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.line_nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.src, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.saved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.chs, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.steps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.published, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_start, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.node_deps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.watched, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dev_held, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dirty, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.is_dirty, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.holds_seen, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.line_moving, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.has_interval, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.has_line, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.reads_rail, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.line_flag, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.interval_devs, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.fixed_caps, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.carrier, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rail_q, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for RailPending {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.b, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.u_held, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slots, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.lq, w); }
        n
    }
}
