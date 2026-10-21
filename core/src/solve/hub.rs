//! The hub rail and the live boundary, wired into [`PartitionedSolver`]
//! (core/README.md, "Kernel semantics: rails and sources (W1f)"). The scalar
//! mathematics lives in `crate::rails`; this module gathers its inputs from
//! the three partition solvers, runs the rail loop of Rails point 3, keeps
//! the rail's events, and applies waveform source edges to the boundary
//! (Sources points 2-5).

use std::rc::Rc;

use super::*;
use crate::devices::battery::BatteryModel;
use crate::devices::sources::{direction, Clock, Pulse, Regime};
use crate::rails::{self, CapDemand, CapSeg, CellDrive, CfTerm, Contrib, Direct, DriftBound, HubKind, Motion, RailModes, RailState, RailStats, Watch};
use crate::solve::closed_form::HubTerm;
use crate::solve::live::{RampEval, Schedule, WaveSource};

/// How many last-bit steps the CC exit's tie may take to bring the rail
/// capacitors' demand to the limit (`tie_exit`): a step moves each one's
/// charge by one bit, and one or two suffice.
const TIE_STEPS: u32 = 8;

/// Whether a draining cell's closed-form hub partitions advance on their
/// coupled path (`rails::cell_modes`, ruling R16 (B)) rather than each on
/// its own pull and the drain's line, the others' first order.
const CELL_COUPLED: bool = true;

/// The hub rail outside exact mode (Rails points 1-8).
pub(super) struct Hub {
    pub node: usize,
    pub device: usize,
    pub kind: HubKind,
    /// The rail capacitors, as indices into `fixed_pair_capacitors` with the
    /// sign of the rail at their `pins[0]`, and their summed capacitance.
    pub caps: Vec<(u32, f64)>,
    pub c_rail: f64,
    /// `sum(esr_k C_k^2) / C_rail^2`: how far the CC rail sits above the
    /// capacitors' charge per amp they take (Rails point 4).
    pub esr_eff: f64,
    pub state: RailState,
    pub w_state: Option<RailState>,
    /// partition -> its contribution (journaled at first touch in a window).
    pub contrib: Vec<Contrib>,
    journal: Vec<(u32, Contrib)>,
    journaled: Vec<bool>,
    pub stats: RailStats,
    pub cf_hubs: Vec<usize>,
    /// Closed-form partitions holding a watched pin of an IC on the rail
    /// that do not read the rail themselves: re-aimed on a move.
    pub reaim_cf: Vec<usize>,
    /// ICs (positions in `ics`) with `vcc` or `gnd` on the rail, and for each
    /// the partitions its drivers and pulls stamp into: re-solved when the
    /// rail crosses its power window, which changes those stamps.
    pub rail_ics: Vec<u32>,
    pub ic_parts: Vec<Vec<usize>>,
    /// Every reader a rail scan queues (Rails point 3).
    pub scan_readers: Vec<DeviceId>,
    /// The direct devices (Rails point 2): resistor and switch contacts
    /// (`kcl_devices` position, contact index), junctions (index in
    /// `junctions`), rail capacitor leakages `(g, other node)`.
    pub direct_contacts: Vec<(u32, u32)>,
    /// The devices owning a direct contact, ascending: a param change there
    /// moves the rail with no partition to say so.
    pub direct_devices: Vec<usize>,
    /// node -> whether the rail equation reads it outside any partition: the
    /// far end of a direct device, or a pin of an IC on the rail (its power
    /// window and its drivers between fixed nodes). A source edge there must
    /// solve the rail again.
    pub direct_nodes: Vec<bool>,
    pub direct_junctions: Vec<u32>,
    pub direct_leaks: Vec<(f64, usize)>,
    /// Whether the rail must solve at the next opportunity.
    pub dirty: bool,
    w_dirty: bool,
    /// A scan's readers and their reads before the move, compared once the
    /// move is published (stat 20).
    scan_pre: Vec<(DeviceId, Vec<bool>)>,
}

impl Hub {
    fn set_contrib(&mut self, p: usize, c: Contrib, window: bool) {
        let old = self.contrib[p];
        if old == c {
            return;
        }
        if window && !self.journaled[p] {
            self.journaled[p] = true;
            self.journal.push((p as u32, old));
        }
        self.contrib[p] = c;
        if c.b.is_none() {
            let term = |c: &Contrib| if c.active && c.b.is_none() { (c.i - c.g * c.u_p, c.g) } else { (0.0, 0.0) };
            let (a_old, g_old) = term(&old);
            let (a_new, g_new) = term(&c);
            self.state.a_run += a_new - a_old;
            self.state.g_run += g_new - g_old;
            self.state.updates += 1;
            if self.state.updates % rails::RECOMPUTE_EVERY == 0 {
                self.recompute_sums();
            }
        }
        self.dirty = true;
    }

    /// The running sums from scratch (P3:450-460).
    fn recompute_sums(&mut self) {
        let (mut a, mut g) = (0.0, 0.0);
        for c in self.contrib.iter().filter(|c| c.active && c.b.is_none()) {
            a += c.i - c.g * c.u_p;
            g += c.g;
        }
        self.state.a_run = a;
        self.state.g_run = g;
    }

    /// How far the rail may move from a curved hub partition's solve before
    /// it solves again (Rails point 3): in CC with no rail capacitor the
    /// draws set `u` alone, so every one solves within `RAIL_CC_RESOLVE_V`;
    /// otherwise each within its own curvature band, and one without a band
    /// within the flat band of its regime (F1v23 finding 3: 25 mV of
    /// linearisation at a junction's knee is a percent of an LED's current).
    /// Every curved kind sizes a band since ruling R7 (a), a free IC supply
    /// included.
    pub(super) fn resolve_band(&self) -> rails::BandRule {
        match (self.psu(), self.state.reg) {
            (Some(_), Regime::Cc) if self.c_rail > 0.0 => rails::BandRule::Curvature { fallback: rails::RAIL_CC_CAP_RESOLVE_V },
            (Some(_), Regime::Cc) => rails::BandRule::Flat(rails::RAIL_CC_RESOLVE_V),
            _ => rails::BandRule::Curvature { fallback: rails::RAIL_RESOLVE_V },
        }
    }

    fn psu(&self) -> Option<(f64, f64)> {
        match self.kind {
            HubKind::Psu { v, i_limit } => Some((v, i_limit)),
            HubKind::Battery(_) => None,
        }
    }

    fn battery(&self) -> Option<&BatteryModel> {
        match &self.kind {
            HubKind::Battery(m) => Some(m),
            HubKind::Psu { .. } => None,
        }
    }

    pub(super) fn begin_window(&mut self) {
        self.w_state = Some(self.state.clone());
        self.w_dirty = self.dirty;
        self.journal.clear();
    }

    pub(super) fn end_window(&mut self) {
        self.w_state = None;
        for (p, _) in self.journal.drain(..) {
            self.journaled[p as usize] = false;
        }
    }

    pub(super) fn rollback_window(&mut self) {
        for (p, c) in self.journal.drain(..).rev() {
            self.contrib[p as usize] = c;
            self.journaled[p as usize] = false;
        }
        if let Some(s) = self.w_state.take() {
            self.state = s;
        }
        self.dirty = self.w_dirty;
        self.scan_pre.clear();
    }
}

/// Exact mode's hub (Rails point 9): the rail is a free node of one
/// partition, and the source is a stamp there.
pub(super) struct ExactHub {
    pub node: usize,
    pub device: usize,
    pub kind: HubKind,
    pub partition: usize,
    pub local: usize,
    pub reg: Regime,
    pub battery: crate::devices::battery::BatteryState,
    /// When SoC was last advanced, and the current out of `pos` then.
    pub t_last: f64,
    pub held: f64,
    pub w_saved: Option<(Regime, crate::devices::battery::BatteryState, f64, f64)>,
}

impl ExactHub {
    /// The source's stamp from its state at the step's start.
    pub(super) fn stamp(&self) -> ExactStamp {
        let kind = match (&self.kind, self.reg) {
            (HubKind::Battery(m), _) => ExactKind::battery(m, self.battery.soc),
            (HubKind::Psu { v, .. }, Regime::Cv) => ExactKind::Row { value: *v },
            (HubKind::Psu { v, i_limit }, Regime::Cc) => ExactKind::Inject { current: direction(*v) * i_limit },
        };
        ExactStamp { partition: self.partition, local: self.local, kind }
    }
}

/// Everything of W1f a whole snapshot carries (Rails point 8).
pub(super) struct W1fSnapshot {
    hub: Option<(RailState, Vec<Contrib>, bool)>,
    exact: Option<(Regime, crate::devices::battery::BatteryState, f64, f64)>,
    wave: Rc<RampEval>,
    caps: Vec<(f64, f64, f64)>,
    static_hubs: Vec<Option<static_part::HubSolveSnapshot>>,
}

impl W1fSnapshot {
    /// Every captured value, printed in a fixed order (tests compare states by
    /// this).
    #[cfg(test)]
    pub(super) fn digest(&self) -> String {
        let sources: Vec<String> = self.wave.sources.iter().map(|w| format!("{:?}", (w.value.to_bits(), w.slot_value.to_bits(), w.level_index, w.ramp, w.next, w.version))).collect();
        format!("{:?} {:?} {sources:?} {:?} {:?}", self.hub, self.exact, self.caps, self.static_hubs)
    }
}

impl PartitionedSolver {
    pub(super) fn w1f_snapshot(&self) -> W1fSnapshot {
        W1fSnapshot {
            hub: self.hub.as_ref().map(|h| (h.state.clone(), h.contrib.clone(), h.dirty)),
            exact: self.exact.as_ref().map(|e| (e.reg, e.battery, e.t_last, e.held)),
            wave: Rc::clone(&self.wave),
            caps: self.fixed_pair_capacitors.iter().map(|c| (c.d_v, c.vc0, c.t0)).collect(),
            static_hubs: self.static_solver.hub_snapshot(),
        }
    }

    pub(super) fn w1f_restore(&mut self, snap: W1fSnapshot) {
        if let (Some(hub), Some((state, contrib, dirty))) = (self.hub.as_mut(), snap.hub) {
            hub.state = state;
            hub.contrib = contrib;
            hub.dirty = dirty;
            // A whole restore recomputes the running sums (Rails point 2).
            hub.recompute_sums();
        }
        if let (Some(e), Some((reg, battery, t_last, held))) = (self.exact.as_mut(), snap.exact) {
            (e.reg, e.battery, e.t_last, e.held) = (reg, battery, t_last, held);
        }
        if self.exact.is_some() {
            self.push_exact_stamp();
        }
        self.wave = snap.wave;
        self.sync_ramps();
        for (w, src) in self.wave.sources.iter().enumerate() {
            self.static_solver.set_wave_level(w, src.level_index);
        }
        for (cap, (d_v, vc0, t0)) in self.fixed_pair_capacitors.iter_mut().zip(snap.caps) {
            (cap.d_v, cap.vc0, cap.t0) = (d_v, vc0, t0);
        }
        self.static_solver.hub_restore(snap.static_hubs);
    }

    /// Wires W1f at build: the waveform sources and the live boundary, and the
    /// hub rail (or exact mode's hub).
    pub(super) fn wire_w1f(&mut self, program: &Program, rail: Option<partition::RailTopology>, live: &[bool], reads: &[Vec<usize>], hub_parts: &[bool], partitions: &[Vec<usize>]) {
        let node_count = program.node_count;
        let device_count = program.devices.len();
        let start = program.options.start_time;

        // The forest without the hub's own entry, and the waveform sources.
        let forest: Vec<partition::ResolvedSource> = self.resolved.iter().filter(|r| rail.is_none_or(|t| t.device != r.device_index)).cloned().collect();
        let mut wave = RampEval::new(forest, node_count, device_count);
        for r in wave.resolved.clone() {
            let device = &program.devices[r.device_index];
            if partition::is_waveform(device.kind) {
                let schedule = match device.kind {
                    kinds::CLOCK | kinds::CLOCK_GEN => Schedule::Clock(Clock::from_params(&device.params)),
                    _ => Schedule::Pulse(Pulse::from_params(&device.params)),
                };
                let (level, next) = match &schedule {
                    Schedule::Clock(c) => c.load_state(start),
                    Schedule::Pulse(p) => p.load_state(start),
                };
                let high = match device.kind {
                    kinds::CLOCK | kinds::CLOCK_GEN => device.params[2],
                    _ => device.params[1],
                };
                wave.wave_of[r.device_index] = wave.sources.len() as u32;
                wave.sources.push(WaveSource {
                    device: r.device_index,
                    schedule,
                    slot: device.blob[0] as usize,
                    value: level.value,
                    level_index: u8::from(level.value == high),
                    slot_value: level.slot,
                    ramp: level.ramp,
                    next,
                    version: 0,
                    watch_gen: 0,
                });
            } else if device.kind != kinds::SECTION_ANCHOR {
                wave.constant[r.device_index] = device.params[0];
            }
        }
        wave.refresh_ramping();
        // The first edge of every source is scheduled at load and reaches the
        // heap at the settle's first drain; none is due at `START_TIME`, so
        // none fires before the settle ends (Sources point 4).
        for (w, src) in wave.sources.iter().enumerate() {
            if let Some(e) = src.next.and_then(|k| src.edge(k)) {
                self.pending_schedule.push((e.time, EventKind::Source { device: DeviceId(src.device as u32), version: 0 }));
            }
            self.static_solver.set_wave_level(w, src.level_index);
        }

        // Which partitions, ICs, watches and rule-2a capacitors each live
        // node feeds.
        self.node_parts = vec![Vec::new(); node_count];
        for (p, nodes) in reads.iter().enumerate() {
            for &n in nodes {
                if live[n] {
                    self.node_parts[n].push(p as u32);
                }
            }
        }
        self.node_ic_readers = vec![Vec::new(); node_count];
        for (node, readers) in build_ic_reader_index(&self.ics) {
            if live[node] {
                self.node_ic_readers[node] = readers.into_iter().map(|d| d as u32).collect();
            }
        }
        self.node_watch_parts = vec![Vec::new(); node_count];
        for (_, ic) in &self.ics {
            for supply in [ic.vcc.node, ic.gnd.node] {
                if !live[supply] {
                    continue;
                }
                for (i, pin) in ic.logic.iter().enumerate() {
                    if !partition::is_watched_read_pin(ic, i) {
                        continue;
                    }
                    if let Some((p, _)) = self.local_index[pin.node] {
                        if matches!(self.partition_kinds[p], PartitionKind::ClosedForm | PartitionKind::Stepped) && !self.node_watch_parts[supply].contains(&(p as u32)) {
                            self.node_watch_parts[supply].push(p as u32);
                        }
                    }
                }
            }
        }
        self.node_caps = vec![Vec::new(); node_count];
        for (k, cap) in self.fixed_pair_capacitors.iter().enumerate() {
            for node in [cap.nodes.0, cap.nodes.1] {
                if live[node] {
                    self.node_caps[node].push(k as u32);
                    self.live_caps = true;
                }
            }
        }
        self.wave_entries = vec![Vec::new(); wave.sources.len()];
        self.wave_parts = vec![Vec::new(); wave.sources.len()];
        for (w, src) in wave.sources.iter().enumerate() {
            let mut in_tree = vec![false; node_count];
            for (k, r) in wave.resolved.iter().enumerate() {
                if r.device_index == src.device || in_tree[r.parent_node] {
                    in_tree[r.child_node] = true;
                    self.wave_entries[w].push(k as u32);
                }
            }
            let mut parts: Vec<usize> = (0..node_count).filter(|&n| in_tree[n]).flat_map(|n| self.node_parts[n].iter().map(|&p| p as usize)).collect();
            parts.sort_unstable();
            parts.dedup();
            self.wave_parts[w] = parts;
        }
        self.wave = Rc::new(wave);
        self.sync_ramps();
        // A source that loads inside a resolved ramp watches its readers'
        // thresholds from `START_TIME` on (Sources points 3 and 4).
        for w in 0..self.wave.sources.len() {
            if self.wave.sources[w].ramp.is_some() {
                self.schedule_ramp_watches(w, start);
            }
        }

        let Some(t) = rail else { return };
        let device = &program.devices[t.device];
        let kind = match device.kind {
            kinds::BATTERY_PACK => HubKind::Battery(BatteryModel::from_record(&device.params, &device.blob)),
            _ => HubKind::Psu { v: device.params[0], i_limit: device.params[1] },
        };
        if t.exact {
            let (partition, local) = self.local_index[t.rail].expect("exact mode makes the rail free");
            let battery = match &kind {
                HubKind::Battery(m) => crate::devices::battery::BatteryState::initial(m),
                HubKind::Psu { .. } => crate::devices::battery::BatteryState { soc: 1.0, discharged: 0.0, rejected: 0.0 },
            };
            self.exact = Some(Box::new(ExactHub { node: t.rail, device: t.device, kind, partition, local, reg: Regime::Cv, battery, t_last: start, held: 0.0, w_saved: None }));
            self.push_exact_stamp();
            return;
        }

        let caps: Vec<(u32, f64)> = self
            .fixed_pair_capacitors
            .iter()
            .enumerate()
            .filter(|(_, c)| c.nodes.0 == t.rail || c.nodes.1 == t.rail)
            .map(|(k, c)| (k as u32, c.c))
            .collect();
        let c_rail: f64 = caps.iter().map(|(_, c)| c).sum();
        // In CC each capacitor takes its share `C_k / C_rail` of the current,
        // so its esr drop is `esr_k C_k I_cap / C_rail`, and their C-weighted
        // mean is the rail's offset above their charge.
        let esr_eff = if c_rail > 0.0 { caps.iter().map(|&(k, c)| self.fixed_pair_capacitors[k as usize].esr * c * c).sum::<f64>() / (c_rail * c_rail) } else { 0.0 };
        let rail_ics: Vec<u32> = self.ics.iter().enumerate().filter(|(_, (_, ic))| ic.vcc.node == t.rail || ic.gnd.node == t.rail).map(|(i, _)| i as u32).collect();
        let ic_parts: Vec<Vec<usize>> = rail_ics
            .iter()
            .map(|&i| {
                let ic = &self.ics[i as usize].1;
                let mut parts: Vec<usize> = digital::output_capable(ic.kind)
                    .iter()
                    .map(|&k| ic.logic[k].node)
                    .chain(ic.logic.iter().filter(|pin| pin.open && pin.floating_input && !pin.open_high).map(|pin| pin.node))
                    .chain([ic.vcc.node, ic.gnd.node])
                    .filter_map(|n| self.local_index[n].map(|(p, _)| p))
                    .collect();
                parts.sort_unstable();
                parts.dedup();
                parts
            })
            .collect();
        let cf_hubs: Vec<usize> = (0..self.partition_kinds.len()).filter(|&p| hub_parts[p] && self.partition_kinds[p] == PartitionKind::ClosedForm).collect();
        let mut reaim_cf = Vec::new();
        // An IC on a static hub partition's node is read against a supply
        // that moves with the rail on that node's published line, as one on
        // the rail itself is (F1v23 finding 4).
        let on_static_hub = |node: usize| self.local_index[node].is_some_and(|(p, _)| hub_parts[p] && self.partition_kinds[p] == PartitionKind::Static);
        let watched_ics: Vec<u32> = (0..self.ics.len() as u32).filter(|&i| rail_ics.contains(&i) || { let ic = &self.ics[i as usize].1; on_static_hub(ic.vcc.node) || on_static_hub(ic.gnd.node) }).collect();
        for &i in &watched_ics {
            let ic = &self.ics[i as usize].1;
            for (li, pin) in ic.logic.iter().enumerate() {
                if !partition::is_watched_read_pin(ic, li) {
                    continue;
                }
                if let Some((p, _)) = self.local_index[pin.node] {
                    if self.partition_kinds[p] == PartitionKind::ClosedForm && !hub_parts[p] && !reaim_cf.contains(&p) {
                        reaim_cf.push(p);
                    }
                }
            }
        }
        reaim_cf.sort_unstable();
        self.closed_form.set_rail_watched(&reaim_cf);
        // A scan re-reads every IC reading a node of a static hub partition,
        // and every IC on the rail (a 555 included).
        let mut scan: Vec<usize> = rail_ics.iter().map(|&i| self.ics[i as usize].0).collect();
        let readers_of = build_ic_reader_index(&self.ics);
        for (p, nodes) in partitions.iter().enumerate() {
            if !(hub_parts[p] && self.partition_kinds[p] == PartitionKind::Static) {
                continue;
            }
            for n in nodes {
                if let Some(r) = readers_of.get(n) {
                    scan.extend(r);
                }
            }
        }
        scan.sort_unstable();
        scan.dedup();
        let is_rail_end = |a: usize, b: usize| (a == t.rail && b != t.rail && self.fixed_node[b]) || (b == t.rail && a != t.rail && self.fixed_node[a]);
        let mut direct_contacts = Vec::new();
        for (k, kd) in self.kcl_devices.iter().enumerate() {
            for (c, &(u, w, _)) in kd.contributions.iter().enumerate() {
                if is_rail_end(u, w) {
                    direct_contacts.push((k as u32, c as u32));
                }
            }
        }
        let mut direct_devices: Vec<usize> = direct_contacts.iter().map(|&(k, _)| self.kcl_devices[k as usize].device_index).collect();
        direct_devices.sort_unstable();
        direct_devices.dedup();
        let direct_junctions: Vec<u32> = self.junctions.iter().enumerate().filter(|(_, j)| is_rail_end(j.anode, j.cathode)).map(|(k, _)| k as u32).collect();
        let direct_leaks: Vec<(f64, usize)> = caps
            .iter()
            .map(|&(k, _)| &self.fixed_pair_capacitors[k as usize])
            .filter(|c| c.leak_g > 0.0)
            .map(|c| (c.leak_g, if c.nodes.0 == t.rail { c.nodes.1 } else { c.nodes.0 }))
            .collect();
        let mut direct_nodes = vec![false; node_count];
        for &(k, c) in &direct_contacts {
            let (a, b, _) = self.kcl_devices[k as usize].contributions[c as usize];
            direct_nodes[a] = true;
            direct_nodes[b] = true;
        }
        for &k in &direct_junctions {
            let j = &self.junctions[k as usize];
            direct_nodes[j.anode] = true;
            direct_nodes[j.cathode] = true;
        }
        for &(_, other) in &direct_leaks {
            direct_nodes[other] = true;
        }
        for &i in &rail_ics {
            let ic = &self.ics[i as usize].1;
            for pin in [&ic.vcc, &ic.gnd].into_iter().chain(&ic.logic) {
                direct_nodes[pin.node] = true;
            }
        }
        direct_nodes[t.rail] = false;
        let battery = match &kind {
            HubKind::Battery(m) => crate::devices::battery::BatteryState::initial(m),
            HubKind::Psu { .. } => crate::devices::battery::BatteryState { soc: 1.0, discharged: 0.0, rejected: 0.0 },
        };
        let state = RailState {
            u: t.u0,
            u_scan: t.u0,
            reg: Regime::Cv,
            battery,
            t_last: start,
            held_i: 0.0,
            held_cf: Vec::new(),
            version: 0,
            event_t: None,
            cap_seg: None,
            cell_path: None,
            cell_until: f64::INFINITY,
            cell: None,
            a_run: 0.0,
            g_run: 0.0,
            updates: 0,
            settle_changed: false,
            gate_flips: 0,
            gates_before: Vec::new(),
        };
        let parts = self.partition_kinds.len();
        self.hub = Some(Box::new(Hub {
            node: t.rail,
            device: t.device,
            kind,
            caps,
            c_rail,
            esr_eff,
            state,
            w_state: None,
            contrib: vec![Contrib::default(); parts],
            journal: Vec::new(),
            journaled: vec![false; parts],
            stats: RailStats::default(),
            cf_hubs,
            reaim_cf,
            rail_ics,
            ic_parts,
            scan_readers: scan.into_iter().map(|d| DeviceId(d as u32)).collect(),
            direct_contacts,
            direct_devices,
            direct_nodes,
            direct_junctions,
            direct_leaks,
            dirty: true,
            w_dirty: true,
            scan_pre: Vec::new(),
        }));
    }

    /// Hands the stepped and closed-form solvers the running ramps, or none.
    pub(super) fn sync_ramps(&mut self) {
        let ramps = self.wave.any_ramping().then(|| Rc::clone(&self.wave));
        self.closed_form.set_ramps(ramps.clone());
        self.stepped.set_ramps(ramps);
    }

    /// Hands the closed-form solver the rail while it is a known closed form
    /// (a PSU in CC with a rail capacitor, Rails point 4), or none. Called
    /// before every call that can aim a closed-form partition, so it follows
    /// the hub's state through a rollback with nothing to restore.
    pub(super) fn sync_rail_drive(&mut self) {
        let path = self.hub.as_ref().and_then(|h| h.state.cell_path.clone());
        let held = path.is_some();
        let until = self.hub.as_ref().map_or(f64::INFINITY, |h| h.state.cell_until);
        let seg = self.hub.as_ref().and_then(|h| h.state.cap_seg.clone()).or(path);
        // A partition on the cell's coupled path follows it, not the drain's
        // line alone.
        let cell = self.hub.as_ref().and_then(|h| h.state.cell).filter(|_| !held);
        self.closed_form.set_cell(cell);
        self.closed_form.set_rail_held(held, until);
        // A threshold's motion with the rail is measured inside the power
        // window it is read in (`schedule_driven_crossing`).
        let (edges, follow) = if seg.is_some() {
            let mut edges = self.rail_power_edges();
            edges.extend(self.rail_kinks());
            edges.sort_by(f64::total_cmp);
            edges.dedup();
            (edges, self.rail_followers())
        } else {
            (Vec::new(), Vec::new())
        };
        self.closed_form.set_rail_seg(seg, edges, follow);
    }

    /// An IC supply node as a line in the rail, `(a, s)` for `a + s u`: the
    /// rail itself, a fixed node, or a static hub node's published line
    /// (Rails point 3). `None` for a node a dynamic partition moves.
    fn supply_line(&self, node: usize) -> Option<(f64, f64)> {
        let rail = self.hub.as_ref()?.node;
        if node == rail {
            Some((0.0, 1.0))
        } else if self.fixed_node[node] {
            Some((self.voltages[node], 0.0))
        } else {
            self.static_solver.hub_node_line(node)
        }
    }

    /// The rail voltages at which an IC's generic thresholds reach a kink
    /// (`digital::power::threshold_kinks`), for every IC whose supply
    /// difference moves with the rail along a line, ascending. A crossing
    /// under the CC rail capacitor's segment solves each threshold as affine
    /// in the rail, so these split its step and are rail events, as the
    /// power windows' edges are (Rails point 4, F1v23 finding 4).
    fn rail_kinks(&self) -> Vec<f64> {
        let mut out = Vec::new();
        for (_, ic) in &self.ics {
            let kinks = digital::power::threshold_kinks(ic);
            if kinks.is_empty() {
                continue;
            }
            let (Some((av, sv)), Some((ag, sg))) = (self.supply_line(ic.vcc.node), self.supply_line(ic.gnd.node)) else { continue };
            let (a, b) = (av - ag, sv - sg);
            if b != 0.0 {
                out.extend(kinks.iter().map(|k| (k - a) / b).filter(|u| u.is_finite()));
            }
        }
        out.sort_by(f64::total_cmp);
        out.dedup();
        out
    }

    /// The IC supply nodes on static hub partitions, as `(node, a, s)`: a
    /// threshold read against one moves with the rail along its line.
    fn rail_followers(&self) -> Vec<(usize, f64, f64)> {
        let rail = self.hub.as_ref().unwrap().node;
        let mut out: Vec<(usize, f64, f64)> = Vec::new();
        for (_, ic) in &self.ics {
            for node in [ic.vcc.node, ic.gnd.node] {
                if node == rail || self.fixed_node[node] || out.iter().any(|f| f.0 == node) {
                    continue;
                }
                if let Some((a, s)) = self.static_solver.hub_node_line(node).filter(|l| l.1 != 0.0) {
                    out.push((node, a, s));
                }
            }
        }
        out
    }

    /// The direct devices' summed draw from the rail and its slope at `u`
    /// (Rails point 2), each evaluated exactly at the published voltages of
    /// its other end.
    pub(super) fn direct_at(&self, u: f64) -> (f64, f64) {
        let Some(hub) = self.hub.as_ref() else { return (0.0, 0.0) };
        let rail = hub.node;
        let v = &self.voltages;
        // The rail node's own shunt.
        let (mut i, mut g) = (static_part::NODE_RSHUNT_G * u, static_part::NODE_RSHUNT_G);
        for &(k, c) in &hub.direct_contacts {
            let (a, b, gc) = self.kcl_devices[k as usize].contributions[c as usize];
            let other = if a == rail { v[b] } else { v[a] };
            i += gc * (u - other);
            g += gc;
        }
        for &(gl, other) in &hub.direct_leaks {
            i += gl * (u - v[other]);
            g += gl;
        }
        for &k in &hub.direct_junctions {
            let j = &self.junctions[k as usize];
            // A failed LED stamps no junction at all, GMIN included (W1g
            // point 12).
            if self.slow.may_have_failed() && self.slow.is_failed(j.device) {
                continue;
            }
            if j.anode == rail {
                let vd = u - v[j.cathode];
                i += j.law.current(vd) + led::GMIN_D * vd;
                g += j.law.companion(vd).0;
            } else {
                let vd = v[j.anode] - u;
                i -= j.law.current(vd) + led::GMIN_D * vd;
                g += j.law.companion(vd).0;
            }
        }
        // An IC driver between the rail and another non-free node: its
        // conductance follows the IC's supply, so its slope is taken by a
        // difference. Its power is judged at `u` on both sides (Rails point
        // 4: power flips only at a rail event), or a rail within
        // `RAIL_DELTA` under a power floor would difference an unpowered
        // driver against a powered one: `vs / (R_out RAIL_DELTA)`, kS of
        // false slope.
        let ic_draw = |uu: f64| -> f64 {
            let mut total = 0.0;
            for &pos in &hub.rail_ics {
                let (_, ic) = &self.ics[pos as usize];
                let at = |node: usize| if node == rail { uu } else { v[node] };
                let held = |node: usize| if node == rail { u } else { v[node] };
                let mut power = digital::power::ic_power(ic, at(ic.vcc.node), at(ic.gnd.node));
                power.powered = digital::power::powered(ic, held(ic.vcc.node), held(ic.gnd.node));
                if !power.powered {
                    continue;
                }
                for &id in &self.ic_driver_ids[pos as usize] {
                    let Some(driver) = self.driver_snapshot.get(id as usize) else { continue };
                    for (target, gd) in ic_driver_branches(&power, driver, ic).into_iter().flatten() {
                        let (a, b) = (driver.node, target);
                        let free = |n: usize| n != rail && !self.fixed_node[n];
                        if free(a) || free(b) || a == b {
                            continue;
                        }
                        if a == rail {
                            total += gd * (uu - v[b]);
                        } else if b == rail {
                            total += gd * (uu - v[a]);
                        }
                    }
                }
            }
            total
        };
        if !hub.rail_ics.is_empty() {
            let d0 = ic_draw(u);
            if d0 != 0.0 || ic_draw(u + rails::RAIL_DELTA) != 0.0 {
                let d1 = ic_draw(u + rails::RAIL_DELTA);
                i += d0;
                g += (d1 - d0) / rails::RAIL_DELTA;
            }
        }
        (i, g)
    }

    /// Each rail capacitor's `(position in fixed_pair_capacitors, sign, far
    /// node)`: `sign` is `+1` when its `pins[0]` is on the rail, so `sign *
    /// vc` is its charge measured from the far node to the rail.
    fn rail_cap_ends(&self) -> impl Iterator<Item = (usize, f64, usize)> + '_ {
        let hub = self.hub.as_ref().expect("a hub");
        hub.caps.iter().map(move |&(k, _)| {
            let cap = &self.fixed_pair_capacitors[k as usize];
            if cap.nodes.0 == hub.node {
                (k as usize, 1.0, cap.nodes.1)
            } else {
                (k as usize, -1.0, cap.nodes.0)
            }
        })
    }

    /// What the rail capacitors would draw at the setpoint `v` while they
    /// hold the rail short of it, and the rail they hold (Rails point 5):
    /// each one's deficit is `dir (V - V_f - vc)`, read from its charge at
    /// `now`.
    fn rail_cap_demand(&self, now: f64, v: f64) -> CapDemand {
        let hub = self.hub.as_ref().expect("a hub");
        let dir = direction(v);
        CapDemand::of(
            v,
            hub.c_rail,
            self.rail_cap_ends().map(|(k, sign, far)| {
                let cap = &self.fixed_pair_capacitors[k];
                (cap.c, cap.esr, dir * ((v - self.voltages[far]) - sign * cap.charge(now)))
            }),
        )
    }

    /// Drains every contribution the partition solvers produced into the
    /// rail.
    fn drain_contributions(&mut self) {
        let Some(hub) = self.hub.as_mut() else {
            self.static_solver.drain_hub_updates().for_each(drop);
            self.closed_form.drain_hub_updates().for_each(drop);
            self.stepped.drain_hub_updates().for_each(drop);
            return;
        };
        let window = self.window;
        for (p, c) in self.static_solver.drain_hub_updates() {
            hub.set_contrib(p, c, window);
        }
        for (p, c) in self.closed_form.drain_hub_updates() {
            hub.set_contrib(p, c, window);
            // A closed-form contribution's trajectory moves whenever it is
            // re-anchored, even when its terms are unchanged.
            hub.dirty = true;
        }
        for (p, c) in self.stepped.drain_hub_updates() {
            hub.set_contrib(p, c, window);
            hub.dirty = true;
        }
    }

    /// `(A0, G)` of the partitions at `t`: the running sums of the static and
    /// stepped terms, and every closed-form term at its `vc(t)`, with the
    /// closed-form partitions those terms are, in the same order.
    fn partition_totals(&self, t: f64) -> (f64, f64, Vec<HubTerm>, Vec<usize>) {
        let hub = self.hub.as_ref().expect("a hub");
        let (mut a, mut g) = (hub.state.a_run, hub.state.g_run);
        let (mut terms, mut parts) = (Vec::new(), Vec::new());
        for &p in &hub.cf_hubs {
            if !hub.contrib[p].active {
                continue;
            }
            if let Some(term) = self.closed_form.hub_term(p, t) {
                // A curved segment's slope at its present `vc` (R8 (a)).
                let tg = if term.g1 != 0.0 { term.g + term.g1 * term.vc } else { term.g };
                a += term.a + term.b * term.vc - tg * term.u_seg;
                g += tg;
                terms.push(term);
                parts.push(p);
            }
        }
        (a, g, terms, parts)
    }

    /// One rail solve at `now` (Rails points 2, 5 and 6): advances SoC, solves
    /// the rail in the present regime, runs the regime rules (at most one
    /// transition) and holds the draws for the next interval. Returns the
    /// rail before it and whether the regime changed.
    fn solve_rail(&mut self, now: f64) -> Result<(f64, bool), String> {
        // SoC first, from the draws held at the last solve.
        {
            let hub = self.hub.as_mut().expect("a hub");
            if let HubKind::Battery(m) = &hub.kind {
                let dt = now - hub.state.t_last;
                hub.state.battery = rails::advance_soc(m, &hub.state.battery, hub.state.held_i, &hub.state.held_cf, dt);
            }
            hub.state.t_last = now;
        }
        let (a_p, g_p, terms, cf_parts) = self.partition_totals(now);
        // A CC rail capacitor's segment held the closed-form draws where they
        // stood when it was aimed; the charge their motion took since is
        // known in closed form, and comes off the capacitors' charge here.
        // The part of the draws the next segment holds, and the charge, are
        // over the one list of partitions the totals read.
        let cf_drift = self.cf_drift_charge(now, &cf_parts);
        let cf_frozen: f64 = terms.iter().map(|t| t.b_at(self.hub.as_ref().unwrap().state.u) * t.vc).sum();
        let hub = self.hub.as_ref().expect("a hub");
        let direct = |u: f64| self.direct_at(u);
        let old_u = hub.state.u;
        let mut reg = hub.state.reg;
        let mut cap_seg = hub.state.cap_seg.clone();
        let has_cap = hub.c_rail > 0.0;
        let solve_in = |reg: Regime, cap_seg: &mut Option<CapSeg>, u_from: f64| -> Result<f64, String> {
            match (&hub.kind, reg) {
                (HubKind::Battery(m), _) => {
                    let soc = hub.state.battery.soc;
                    rails::solve_battery(m.emf(soc), m.resistance(soc), a_p, g_p, &direct, u_from)
                }
                (HubKind::Psu { v, .. }, Regime::Cv) => {
                    *cap_seg = None;
                    Ok(*v)
                }
                (HubKind::Psu { v, i_limit }, Regime::Cc) => {
                    let dl = direction(*v) * i_limit;
                    if has_cap {
                        // The rail is the capacitors' closed form, re-aimed at
                        // the new draws: their charge is continuous from where
                        // it stands (`u_from` when the segment starts), and the
                        // rail sits `esr_eff I_cap` above it, which steps with
                        // the draws.
                        let q = cap_seg.as_ref().map_or(u_from, |s| s.charge_at(now) - cf_drift);
                        let e = hub.esr_eff;
                        let u = if e > 0.0 { rails::solve_cc_cap(q, e, dl, a_p, g_p, &direct)? } else { q };
                        let (a0, g) = {
                            let (i_d, g_d) = direct.at(u);
                            (a_p + (i_d - g_d * u), g_p + g_d)
                        };
                        let alpha = 1.0 / (1.0 + g * e);
                        let u_inf = (dl - a0) / g;
                        let (k, slope) = (alpha * g / hub.c_rail, alpha * (dl - a0 - g * u) / hub.c_rail);
                        // With closed-form hub partitions on it the rail and they
                        // advance together on the coupled system's own modes
                        // (ruling R15 (a)): each is aimed at `u` next, from its
                        // capacitor's voltage now, as `coupling` reads it.
                        let modes = if cf_parts.is_empty() {
                            None
                        } else {
                            let coupled: Option<Vec<rails::Coupled>> = cf_parts.iter().map(|&p| self.closed_form.coupling(p, now, u)).collect();
                            coupled.and_then(|c| rails::coupled_modes(hub.c_rail, e, alpha, k, slope / alpha, &c)).map(|modes| {
                                let esr = (e > 0.0).then(|| (k + alpha / (e * hub.c_rail), slope / alpha, 1.0 / (e * hub.c_rail)));
                                Rc::new(RailModes { modes, q0: q, esr })
                            })
                        };
                        let coupled = modes.is_some();
                        *cap_seg = Some(CapSeg {
                            t0: now,
                            u0: u,
                            u_inf,
                            k,
                            slope,
                            alpha,
                            beta: alpha * e * (dl - a0),
                            cf_frozen: if coupled { 0.0 } else { cf_frozen },
                            cf_count: if coupled { 0 } else { cf_parts.len() as u32 },
                            modes,
                        });
                        Ok(u)
                    } else {
                        rails::solve_cc(dl, a_p, g_p, &direct, u_from)
                    }
                }
            }
        };
        let mut u = solve_in(reg, &mut cap_seg, old_u)?;
        let mut changed = false;
        if let Some((v, i_limit)) = hub.psu() {
            let demand = if reg == Regime::Cv && has_cap { self.rail_cap_demand(now, v) } else { CapDemand::none(v) };
            let next = rails::next_regime(reg, v, i_limit, has_cap, a_p, g_p, &direct, u, demand)?;
            if next != reg {
                reg = next;
                changed = true;
                // CC with a rail capacitor starts from the charge the
                // capacitors hold: the CV rail it leaves when none is below
                // the setpoint (continuous, Rails point 4), else their own.
                let from = if reg == Regime::Cc && has_cap {
                    cap_seg = None;
                    demand.held
                } else {
                    u
                };
                u = solve_in(reg, &mut cap_seg, from)?;
            }
        }
        // The draws held over the next interval (Rails point 6).
        let (i_d, g_d) = direct.at(u);
        let held_i = hub.state.a_run + hub.state.g_run * u + i_d;
        let held_cf: Vec<CfTerm> = terms.iter().map(|t| CfTerm { a: t.a + t.g * (u - t.u_seg), b: t.b_at(u), vc0: t.vc, inf: t.inf, f: t.f, w: t.w }).collect();
        // A draining cell's closed-form hub partitions follow its rail until
        // the next solve (ruling R15 (a), `rails::CellDrive`): `kappa` from the
        // rail equation of point 2, and the drain's line as the implied
        // trajectory's SoC term (`rail_watches`).
        let cell = match &hub.kind {
            HubKind::Battery(m) if !cf_parts.is_empty() => {
                let soc = hub.state.battery.soc;
                let r = m.resistance(soc);
                let (a0, g) = (a_p + (i_d - g_d * u), g_p + g_d);
                let draw = a0 + g * u;
                let kappa = if r == 0.0 { 0.0 } else { -r / (1.0 + r * g) };
                let rho = if draw > 0.0 && soc > 0.0 {
                    let (de, dr) = rails::soc_slopes(m, soc);
                    let dsoc_dt = -draw / m.capacity_coulombs();
                    (de - dr * draw) / (1.0 + r * g) * dsoc_dt
                } else {
                    0.0
                };
                Some(CellDrive { t0: now, kappa, rho })
            }
            _ => None,
        };
        // With the partitions coupled through the cell (ruling R16 (B)), the
        // rail and they advance together on their system's own modes, as on a
        // CC rail capacitor's path: each follows the others' motion too, not
        // only its own and the drain's.
        let cell_path = match cell {
            Some(cd) if CELL_COUPLED => {
                let coupled: Option<Vec<rails::Coupled>> = cf_parts.iter().map(|&p| self.closed_form.coupling(p, now, u)).collect();
                coupled.and_then(|c| rails::cell_modes(cd.kappa, cd.rho, &c)).map(|modes| {
                    let slope = modes.iter().map(|m| m.1).sum();
                    CapSeg { t0: now, u0: u, u_inf: u, k: 0.0, slope, alpha: 1.0, beta: 0.0, cf_frozen: 0.0, cf_count: 0, modes: Some(Rc::new(RailModes { modes, q0: u, esr: None })) }
                })
            }
            _ => None,
        };
        let moved = (u - old_u).abs() > rails::RAIL_MOVE_EPS;
        let settle = (u - old_u).abs() > rails::SETTLE_MOVE_V || changed;

        let hub = self.hub.as_mut().expect("a hub");
        hub.state.reg = reg;
        hub.state.cap_seg = cap_seg;
        hub.state.cell = cell;
        hub.state.cell_until = cell_path.as_ref().and_then(|p| p.modes.as_ref()).map_or(f64::INFINITY, |m| now + m.passage(rails::RAIL_DRIFT_V).min(m.passage(-rails::RAIL_DRIFT_V)));
        hub.state.cell_path = cell_path;
        hub.state.held_i = held_i;
        hub.state.held_cf = held_cf;
        if moved {
            hub.state.u = u;
            hub.stats.moves += 1;
        }
        if settle {
            hub.state.settle_changed = true;
        }
        Ok((old_u, changed))
    }

    /// What a rail move does at once (Rails point 3): publish `u`, re-aim the
    /// closed forms that read it, re-publish the stepped hub partitions, move
    /// the rail capacitors, wake the ICs whose power flipped, and run a scan
    /// past `RAIL_SCAN_V`. The static hub partitions follow in the next static
    /// pass (queued here).
    fn on_rail_move(&mut self, now: f64, old_u: f64, readers: &mut Vec<DeviceId>, hold_power: bool) -> Vec<usize> {
        let (rail, u) = {
            let hub = self.hub.as_ref().expect("a hub");
            (hub.node, hub.state.u)
        };
        // A scan's pre-move reads, before anything is re-published.
        let scan = {
            let hub = self.hub.as_ref().unwrap();
            (u - hub.state.u_scan).abs() > rails::RAIL_SCAN_V
        };
        if scan {
            let pre: Vec<(DeviceId, Vec<bool>)> = self.hub.as_ref().unwrap().scan_readers.iter().map(|&d| (d, self.reads_of(d.0 as usize))).collect();
            let hub = self.hub.as_mut().unwrap();
            hub.state.u_scan = u;
            hub.stats.scans += 1;
            hub.scan_pre = pre;
        }
        // The ICs whose power gate the move flipped: they read again, and the
        // partitions they stamp into re-solve (their drivers and pulls stamp
        // only while powered).
        let mut flipped_parts = Vec::new();
        {
            let hub = self.hub.as_ref().unwrap();
            for (k, &pos) in hub.rail_ics.iter().enumerate() {
                let (d, ic) = &self.ics[pos as usize];
                let at = |uu: f64, node: usize| if node == rail { uu } else { self.voltages[node] };
                let before = digital::power::powered(ic, at(old_u, ic.vcc.node), at(old_u, ic.gnd.node));
                let after = digital::power::powered(ic, at(u, ic.vcc.node), at(u, ic.gnd.node));
                if before != after && !hold_power {
                    readers.push(DeviceId(*d as u32));
                    flipped_parts.extend(&hub.ic_parts[k]);
                }
            }
        }
        self.write_boundary(rail, u);
        self.static_solver.set_boundary(rail, u);
        self.static_solver.request_hub_republish();
        self.reaim_fixed_pair_caps(rail, now);
        let (cf, plain, stepped): (Vec<usize>, Vec<usize>, Vec<usize>) = {
            let hub = self.hub.as_ref().unwrap();
            (hub.cf_hubs.clone(), hub.reaim_cf.clone(), self.stepped.hub_partitions())
        };
        let mut list: Vec<usize> = cf.iter().chain(&plain).chain(&stepped).copied().collect();
        list.sort_unstable();
        self.begin_dynamic_writes_list(&list);
        for p in cf.into_iter().chain(plain) {
            readers.extend(self.closed_form.reaim(p, now, &self.live_params, &mut self.voltages, &mut self.currents));
        }
        self.stepped.republish_hubs(&self.live_params, &mut self.voltages, &mut self.currents);
        self.end_dynamic_writes();
        if scan {
            readers.extend(self.hub.as_ref().unwrap().scan_readers.iter().copied());
        }
        flipped_parts.sort_unstable();
        flipped_parts.dedup();
        flipped_parts
    }

    /// Whether a move of the rail from `old_u` to `u` flips rail ICs' power
    /// against the CC segment's motion at `now`: off while it rises, on while
    /// it falls.
    fn flip_runs_against_the_ramp(&self, now: f64, old_u: f64, u: f64) -> bool {
        let hub = self.hub.as_ref().unwrap();
        let Some(slope) = hub.state.cap_seg.as_ref().map(|s| s.slope_at(now)) else { return false };
        let rail = hub.node;
        let on = |uu: f64| {
            hub.rail_ics.iter().filter(|&&pos| {
                let ic = &self.ics[pos as usize].1;
                let at = |node: usize| if node == rail { uu } else { self.voltages[node] };
                digital::power::powered(ic, at(ic.vcc.node), at(ic.gnd.node))
            }).count()
        };
        let (before, after) = (on(old_u), on(u));
        (slope > 0.0 && after < before) || (slope < 0.0 && after > before)
    }

    /// The rail's closed form changed with no move (a CC rail capacitor's
    /// segment re-aimed at new draws within one instant, or ended): the
    /// closed forms it drove are re-aimed under the new one, or under none,
    /// as on a move (Rails point 4).
    fn reaim_driven(&mut self, now: f64, readers: &mut Vec<DeviceId>) {
        let mut list: Vec<usize> = {
            let hub = self.hub.as_ref().unwrap();
            hub.cf_hubs.iter().chain(&hub.reaim_cf).copied().collect()
        };
        list.sort_unstable();
        list.dedup();
        self.begin_dynamic_writes_list(&list);
        for &p in &list {
            readers.extend(self.closed_form.reaim(p, now, &self.live_params, &mut self.voltages, &mut self.currents));
        }
        self.end_dynamic_writes();
    }

    /// Before a crossing of a partition the CC rail capacitor's segment drives
    /// lands (Rails point 4): the segment holds the closed-form draws where
    /// they stood, so the rail it drives with trails the one a solve gives by
    /// the charge they have drifted since, up to `RAIL_DRIFT_V`. Landing
    /// first, the node is put `CROSS_NUDGE_V` past a threshold that rides the
    /// stale rail, and the solve the crossing then causes steps the threshold
    /// back across it: the same crossing again a moment later, and on a
    /// plain input a glitch. So when the owed step could move a threshold
    /// against its node by the nudge, the rail solves first and re-aims what
    /// it drives, rescheduling this crossing under the solved rail. Every
    /// threshold the compiler emits moves with the supply by a factor in
    /// [0, 1] (the affine forms `schedule_driven_crossing` lists), as does a
    /// hub node, so a smaller step keeps the landed node past it. Returns
    /// whether the crossing was rescheduled.
    pub(super) fn solve_rail_before_crossing(&mut self, p: usize, now: f64, readers: &mut Vec<DeviceId>) -> Result<bool, String> {
        if self.exact.is_some() || !self.closed_form.is_driven(p) {
            return Ok(false);
        }
        let Some((owed, _, _)) = self.owed_rail_step(now) else { return Ok(false) };
        if owed.abs() < closed_form::CROSS_NUDGE_V {
            return Ok(false);
        }
        let before = self.closed_form.version_of(p);
        let due = self.closed_form.due_crossing(p, now);
        self.hub.as_mut().unwrap().dirty = true;
        let drivers = Rc::clone(&self.driver_snapshot);
        self.rail_loop(now, &drivers, readers)?;
        if self.closed_form.version_of(p) == before {
            return Ok(false);
        }
        // The node reached its threshold at this instant on the old aim, and
        // the solve moved it by no more than the rail step's share: unless
        // that put it back short of the threshold, the root is `now`, which
        // the new aim's schedule cannot accept, and it lands here instead.
        let u = self.hub.as_ref().unwrap().state.u;
        let landed = due.is_some_and(|due| self.closed_form.keep_due_crossing(p, due, u, now));
        Ok(!landed)
    }

    /// The step a rail solve at `now` owes the CC rail capacitor's segment
    /// (Rails point 4), and its two parts: the solve takes the drifted charge
    /// off the capacitors, and the rail's esr offset `beta` moves with `A0`,
    /// which then holds the draws' `b vc` where they stand rather than
    /// frozen. `None` outside a segment, and for a coupled one (ruling R15
    /// (a)), which holds nothing frozen and so owes no step.
    pub(super) fn owed_rail_step(&self, now: f64) -> Option<(f64, f64, f64)> {
        let hub = self.hub.as_ref()?;
        let seg = hub.state.cap_seg.as_ref().filter(|s| s.modes.is_none())?;
        let (_, _, terms, parts) = self.partition_totals(now);
        let b_dvc: f64 = terms.iter().map(|t| t.b_at(hub.state.u) * t.vc).sum::<f64>() - seg.cf_frozen;
        let (drift, esr) = (seg.alpha * self.cf_drift_charge(now, &parts), seg.alpha * hub.esr_eff * b_dvc);
        Some((drift + esr, drift, esr))
    }

    /// Completes a scan once the move is published: each queued reader whose
    /// reads changed counts once in stat 20.
    fn finish_scan(&mut self) {
        let Some(hub) = self.hub.as_mut() else { return };
        if hub.scan_pre.is_empty() {
            return;
        }
        let pre = std::mem::take(&mut hub.scan_pre);
        let mut flips = 0;
        for (d, before) in pre {
            let after = self.reads_of(d.0 as usize);
            if !before.is_empty() && !after.is_empty() && before != after {
                flips += 1;
            }
        }
        self.hub.as_mut().unwrap().stats.flips += flips;
    }

    /// IC `device`'s logic-pin reads at the published voltages, empty while
    /// unpowered. A Schmitt pin reads its definite side, or 0 in its band.
    fn reads_of(&self, device: usize) -> Vec<bool> {
        let Some(&pos) = self.ic_position.get(device).filter(|&&p| p != u32::MAX) else { return Vec::new() };
        let ic = &self.ics[pos as usize].1;
        let v = &self.voltages;
        let power = digital::power::ic_power(ic, v[ic.vcc.node], v[ic.gnd.node]);
        if !power.powered || digital::is_ne555(ic.kind) {
            return Vec::new();
        }
        let mut out = Vec::with_capacity(ic.logic.len());
        match power.schmitt {
            None => digital::levels::read_plain_into(ic, &power, v, self.derived_now, &mut out),
            Some(_) => {
                for pin in &ic.logic {
                    let mut state = false;
                    out.push(digital::levels::read(ic, &power, pin, v[pin.node], self.derived_now, &mut state));
                }
            }
        }
        out
    }

    /// Writes a boundary node's value into the published array, logged.
    pub(super) fn write_boundary(&mut self, node: usize, value: f64) {
        if self.voltages[node].to_bits() != value.to_bits() {
            if self.window {
                self.v_log.push((node as u32, self.voltages[node].to_bits()));
            }
            self.voltages[node] = value;
        }
        fold_bound(&mut self.v_hint, value);
    }

    /// Re-aims every rule-2a capacitor with a pin on `node` at `now` (Rails
    /// point 7): `vc` continuous, the new `dV` its target.
    fn reaim_fixed_pair_caps(&mut self, node: usize, now: f64) {
        for k in 0..self.node_caps.get(node).map_or(0, |v| v.len()) {
            let idx = self.node_caps[node][k] as usize;
            let (a, b) = self.fixed_pair_capacitors[idx].nodes;
            let d_v = self.voltages[a] - self.voltages[b];
            self.fixed_pair_capacitors[idx].reaim(now, d_v);
        }
    }

    /// The rail loop (Rails point 3), at the end of every publishing call:
    /// drain the contributions, solve the rail, publish a move, re-solve
    /// every nonlinear hub partition past its band, and again, until none is,
    /// at most `RAIL_LOOP_MAX` passes. Schedules the next rail event after a
    /// solve.
    pub(super) fn rail_loop(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>) -> Result<(), String> {
        if self.exact.is_some() {
            return self.exact_after_publish(now, drivers, readers);
        }
        if self.hub.is_none() {
            self.drain_contributions();
            return Ok(());
        }
        let mut solved = false;
        // Set by a CC exit with a rail capacitor: the passes after it tie the
        // capacitors again at the draws the exit's re-solves settle to.
        let mut tie = false;
        // The rail after each pass's solve, and whether the re-solves have
        // stopped taking the static cache (below).
        let mut seen: Vec<f64> = Vec::new();
        let mut fresh = false;
        for _pass in 0..rails::RAIL_LOOP_MAX {
            self.drain_contributions();
            if !self.hub.as_ref().unwrap().dirty {
                if solved {
                    self.finish_scan();
                    self.schedule_rail_event(now);
                }
                return Ok(());
            }
            self.hub.as_mut().unwrap().dirty = false;
            if tie && self.hub.as_ref().unwrap().state.reg == Regime::Cv {
                self.tie_exit(now);
            }
            let (seg_before, cell_before) = {
                let state = &self.hub.as_ref().unwrap().state;
                (state.cap_seg.clone().or_else(|| state.cell_path.clone()), state.cell)
            };
            let (old_u, regime_changed) = self.solve_rail(now)?;
            self.sync_hub_cache_band();
            self.sync_rail_drive();
            self.reset_cf_drift(now);
            solved = true;
            let u = self.hub.as_ref().unwrap().state.u;
            let moved = u.to_bits() != old_u.to_bits();
            // A rail back where an earlier pass than the last put it is a
            // cycle of cache replays: two entries on either side of a band
            // edge (a driver's R_out kink, `curvature::Bound::eval`), each
            // first-order correction placing the rail just inside the
            // other's band, alternate without end (INV2, SAP-1 on a battery
            // at 4.8 V). From then on a static hub partition past its band
            // takes only a solve at this very rail, as at a CC exit, and the
            // loop settles as a Newton iteration does. A closed-form segment
            // past its band already builds at the very rail (its key holds
            // the rail's bits), and a stepped partition solves afresh.
            if !fresh && seen.len() >= 2 && seen[..seen.len() - 1].iter().any(|&v| (v - u).abs() <= rails::RAIL_MOVE_EPS) {
                fresh = true;
            }
            // A rail IC whose own power flip steps the rail back across its
            // window's edge (its supply current through the rail capacitors'
            // esr, F1r14's slow inrush at esr 1-5 ohm: 17 nV against an edge
            // landed two reached-widths past) has no consistent state at this
            // instant: the passes alternate. On the pass whose flip runs
            // against the CC segment's motion (an IC turning off while the
            // ramp rises), the flip is held: the partitions keep the stamps
            // of the state the ramp is heading into, and the segment, solved
            // with them, crosses the edge a moment later at the next event,
            // where the gate settles.
            let hold = moved && seen.len() >= 2 && seen[seen.len() - 2].to_bits() == u.to_bits() && self.flip_runs_against_the_ramp(now, old_u, u);
            seen.push(u);
            let flipped = if moved {
                self.on_rail_move(now, old_u, readers, hold)
            } else {
                // The segment changed with no move of `u`: re-aimed at new
                // draws within one instant, or ended by a CC exit within
                // RAIL_MOVE_EPS of the setpoint. What it drives is aimed again
                // under the new one, or under none, as a move would; so is a
                // cell's drive that changed (its slopes, its drain).
                let state = &self.hub.as_ref().unwrap().state;
                if state.cap_seg.clone().or_else(|| state.cell_path.clone()) != seg_before || state.cell != cell_before {
                    self.reaim_driven(now, readers);
                }
                Vec::new()
            };
            let cc_exit = regime_changed && self.hub.as_ref().unwrap().state.reg == Regime::Cv;
            if regime_changed {
                self.on_regime_change(now);
                tie |= cc_exit;
            }
            let rule = self.hub.as_ref().unwrap().resolve_band();
            let mut past_static = Vec::new();
            self.static_solver.hubs_past_band(u, rule, &mut past_static);
            let mut past_stepped = self.stepped.hubs_past_band(u, rule);
            // On a CC rail with no capacitor (the 1 uV band) a stepped hub
            // partition past its band only because the rail moved is re-aimed,
            // not restarted: it keeps the step its controller chose
            // (`SteppedSolver::reaim`). The CC exit's re-solve and a power
            // flip's are discontinuities and restart.
            let reaim: Vec<usize> = if !cc_exit && matches!(rule, rails::BandRule::Flat(_)) { past_stepped.clone() } else { Vec::new() };
            let mut flipped_cf = Vec::new();
            self.closed_form.hubs_past_band(u, rule, &mut flipped_cf);
            if cc_exit {
                // Leaving CC, every hub partition solves again at the setpoint
                // rather than carry a first-order correction from wherever it
                // last solved, maybe volts away on the CC rail (Rails point 5,
                // F1 finding D). A static hub partition's cache answers only
                // with a solve at the setpoint itself: a hit within the band
                // is that first-order correction again, and since the
                // ramp's narrow bands store entries close below the
                // setpoint, cache on and cache off would part by its error.
                self.static_solver.set_hub_cache_rule(rails::BandRule::Flat(0.0));
                self.static_solver.hub_partitions(&mut past_static);
                past_stepped.extend(self.stepped.hub_partitions());
                flipped_cf.extend(self.hub.as_ref().unwrap().cf_hubs.iter().copied());
            }
            let flipped_p = flipped.clone();
            for p in flipped {
                match self.partition_kinds[p] {
                    PartitionKind::Static => past_static.push(PartitionId(p as u32)),
                    PartitionKind::Stepped => past_stepped.push(p),
                    PartitionKind::ClosedForm => flipped_cf.push(p),
                }
            }
            past_static.sort_unstable();
            past_static.dedup();
            past_stepped.sort_unstable();
            past_stepped.dedup();
            flipped_cf.sort_unstable();
            flipped_cf.dedup();
            if !flipped_cf.is_empty() {
                self.begin_dynamic_writes_list(&flipped_cf);
                readers.extend(self.closed_form.resolve_dirty(&flipped_cf, now, drivers, &self.live_params, &mut self.voltages, &mut self.currents)?);
                self.end_dynamic_writes();
            }
            if fresh {
                self.static_solver.set_hub_cache_rule(rails::BandRule::Flat(0.0));
            }
            if moved || !past_static.is_empty() {
                self.static_solver.set_inputs_bounded(false);
                self.static_solver.resolve_dirty_into(&past_static, now, drivers, readers)?;
                self.copy_in_static();
            }
            if cc_exit || fresh {
                self.sync_hub_cache_band();
            }
            if !past_stepped.is_empty() {
                self.begin_dynamic_writes_list(&past_stepped);
                let pre = Rc::clone(&self.driver_snapshot);
                self.sync_stepped_cc(now);
                self.stepped.reaim = reaim.into_iter().filter(|p| !flipped_p.contains(p)).collect();
                let stepped = self.stepped.resolve_dirty(&past_stepped, now, &pre, drivers, &self.live_params, &mut self.voltages, &mut self.currents);
                self.stepped.cc = None;
                self.stepped.reaim.clear();
                readers.extend(stepped?);
                self.end_dynamic_writes();
            }
            if past_static.is_empty() && past_stepped.is_empty() && flipped_cf.is_empty() {
                self.drain_contributions();
                let hub = self.hub.as_mut().unwrap();
                // A move alone changes no contribution: the rail is solved.
                hub.dirty = false;
                self.finish_scan();
                self.schedule_rail_event(now);
                return Ok(());
            }
        }
        Err(rails::loop_unsettled())
    }

    /// Hands the stepped solver the CC rail its hub partitions step against
    /// (`stepped::CcRail`): a bench PSU in CC with no rail capacitor, the
    /// draws at the present rail and each stepped hub partition's own share
    /// of them. Every other regime sets none. The caller takes it back after
    /// the call it is for.
    pub(super) fn sync_stepped_cc(&mut self, now: f64) {
        let cc = self.hub.as_ref().and_then(|hub| {
            let (v, i_limit) = hub.psu()?;
            if hub.state.reg != Regime::Cc || hub.c_rail > 0.0 {
                return None;
            }
            let own: Vec<(usize, f64, f64)> = hub
                .contrib
                .iter()
                .enumerate()
                .filter(|&(p, c)| c.active && c.b.is_none() && self.partition_kinds[p] == PartitionKind::Stepped)
                .map(|(p, c)| (p, c.i - c.g * c.u_p, c.g))
                .collect();
            let (a_p, g_p, _, _) = self.partition_totals(now);
            let u = hub.state.u;
            let (i_d, g_d) = self.direct_at(u);
            Some(stepped::CcRail { rail: hub.node, dl: direction(v) * i_limit, v, a_all: a_p + i_d - g_d * u, g_all: g_p + g_d, own })
        });
        self.stepped.cc = cc;
    }

    /// Keeps the static solver's hub cache band with the regime (after a
    /// rail solve and a rollback).
    pub(super) fn sync_hub_cache_band(&mut self) {
        if let Some(hub) = self.hub.as_ref() {
            // A cache hit answers only within the band the rail loop
            // re-solves at, in every regime: a wider one hands it back the
            // entry it found past its band (CC with no rail capacitor too).
            let rule = hub.resolve_band();
            self.static_solver.set_hub_cache_rule(rule);
        }
    }

    /// After a regime change: the rail capacitors follow the new regime
    /// (Rails points 4 and 7). In CC nothing reads them: the rail's segment
    /// carries each capacitor's current.
    fn on_regime_change(&mut self, now: f64) {
        let hub = self.hub.as_ref().unwrap();
        if hub.state.reg == Regime::Cv && hub.c_rail > 0.0 {
            self.tie_exit(now);
        }
    }

    /// A PSU with a rail capacitor left CC at the setpoint (Rails point 5):
    /// the supply current is continuous across the exit, so the capacitors
    /// take in CV what the limit leaves of the CV solve's own draw at the
    /// setpoint, each its share `C_k / C_rail` as it took its share of the
    /// segment's current, one esr drop below the rail, from where it relaxes
    /// to `dV` as rule 2a's exponential (at esr 0 the charge is the rail and
    /// the current 0, rule 2a's divergence). The CV -> CC test reads that
    /// charge back, so it sees the limit itself rather than the segment's
    /// current at a rail that landed a reached-width past the setpoint, or
    /// the draws before the exit's re-solves (the rail loop ties again after
    /// each); and since a charge rounds at the rail's scale, which `1 / esr`
    /// magnifies, it is stepped toward the rail until the test reads no more
    /// than the limit. Only a real change of the draws re-enters.
    fn tie_exit(&mut self, now: f64) {
        let (a_p, g_p, _, _) = self.partition_totals(now);
        let hub = self.hub.as_ref().unwrap();
        let Some((v, i_limit)) = hub.psu() else { return };
        let (dir, c_rail) = (direction(v), hub.c_rail);
        let draw = rails::draw_at(a_p, g_p, &|u: f64| self.direct_at(u), v);
        let i_cap = dir * i_limit - draw;
        let ends: Vec<(usize, f64, usize)> = self.rail_cap_ends().collect();
        for &(k, sign, far) in &ends {
            let (a, b) = self.fixed_pair_capacitors[k].nodes;
            let d_v = self.voltages[a] - self.voltages[b];
            let v_far = self.voltages[far];
            let cap = &mut self.fixed_pair_capacitors[k];
            let vc = if cap.esr > 0.0 { sign * ((v - v_far) - cap.esr * (cap.c / c_rail) * i_cap) } else { d_v };
            cap.hold_at(now, d_v, vc);
        }
        for _ in 0..TIE_STEPS {
            let demand = self.rail_cap_demand(now, v);
            if !(demand.draw > 0.0 && (draw + dir * demand.draw).abs() > i_limit) {
                break;
            }
            for &(k, sign, _) in &ends {
                let cap = &mut self.fixed_pair_capacitors[k];
                if cap.esr > 0.0 {
                    let held = sign * cap.vc0;
                    let toward = if dir > 0.0 { held.next_up() } else { held.next_down() };
                    cap.vc0 = sign * toward;
                }
            }
        }
    }

    /// Schedules the next `Rail` event from the implied trajectory at `now`
    /// (Rails point 4), superseding the pending one.
    fn schedule_rail_event(&mut self, now: f64) {
        let watches = self.rail_watches(now);
        let mut t = rails::next_event(now, &watches);
        // A driven partition on a rail of several modes holds its coupling
        // band for a time as well as a width (`couple_band`): the rail solves
        // and aims it again by then.
        let expiry = self.closed_form.couple_expiry();
        if expiry.is_finite() {
            let at = if expiry > now + 1e-15 { expiry } else { (now + 1e-15).next_up() };
            t = Some(t.map_or(at, |t| t.min(at)));
        }
        let hub = self.hub.as_mut().unwrap();
        hub.state.version += 1;
        hub.state.event_t = t;
        if let Some(t) = t {
            let version = hub.state.version;
            self.pending_schedule.push((t, EventKind::Rail { version }));
        }
    }

    /// The rail voltages at which an IC whose vcc is the rail and gnd fixed
    /// turns on or off (its power window, W1c point 1), ascending.
    fn rail_power_edges(&self) -> Vec<f64> {
        let hub = self.hub.as_ref().unwrap();
        let mut windows = Vec::new();
        for &pos in &hub.rail_ics {
            let ic = &self.ics[pos as usize].1;
            if ic.vcc.node == hub.node && self.fixed_node[ic.gnd.node] && ic.gnd.node != hub.node {
                let (lo, hi) = digital::levels::power_window(ic);
                let vg = self.voltages[ic.gnd.node];
                windows.push(vg + lo);
                windows.push(vg + hi);
            }
        }
        windows.sort_by(f64::total_cmp);
        windows.dedup();
        windows
    }

    /// The threshold functions of the implied trajectory at `now`.
    fn rail_watches(&self, now: f64) -> Vec<Watch> {
        let hub = self.hub.as_ref().unwrap();
        let u = hub.state.u;
        let (a_p, g_p, terms, _) = self.partition_totals(now);
        let (i_d, g_d) = self.direct_at(u);
        let (a0, g) = (a_p + (i_d - g_d * u), g_p + g_d);
        let draw = a0 + g * u;
        // The trajectory starts from the equation's own value at `now`, not
        // the published `u`: a closed form re-aimed after the last solve hands
        // back the same line rounded differently (its slope is a finite
        // difference), and a watch counted from `u` would land that rounding
        // away from its threshold.
        // Power windows of the ICs whose vcc is the rail and gnd fixed.
        let windows = self.rail_power_edges();
        let cf_motion = |coef: f64| -> Vec<Motion> {
            terms.iter().map(|t| Motion::Exp { coef: coef * t.b_at(u), vc0: t.vc, inf: t.inf, f: t.f }).collect()
        };
        let mut watches = Vec::new();
        match (&hub.kind, hub.state.reg) {
            (HubKind::Battery(m), _) => {
                let soc = hub.state.battery.soc;
                if let Some(path) = hub.state.cell_path.as_ref().and_then(|p| p.modes.clone()) {
                    // The coupled path (ruling R16 (B)): its own first
                    // passages, as a CC rail capacitor's, of the power
                    // windows and the kinks a driven crossing's threshold
                    // motion is affine between, and of the drift cap the
                    // static and direct draws' linearisation holds to.
                    let mut th = windows;
                    th.extend(self.rail_kinks());
                    // A curved partition's coupling band, as on a CC path
                    // (Rails point 3): the drift cap bounds the rail's motion
                    // but not how long a transient's bilinear term runs.
                    let mut rails_p = Vec::new();
                    self.closed_form.nonlinear_hub_rails(hub.resolve_band(), &mut rails_p);
                    for (up, band) in rails_p {
                        th.push(up + band);
                        th.push(up - band);
                    }
                    watches.push(Watch { value: u, thresholds: th, cap: Some(rails::RAIL_DRIFT_V), terms: vec![Motion::Path(path)] });
                    if draw > 0.0 && soc > 0.0 {
                        let knot = m.next_knot_below(soc).unwrap_or(0.0);
                        watches.push(Watch { value: soc, thresholds: vec![knot], cap: None, terms: vec![Motion::Line { rate: -draw / m.capacity_coulombs() }] });
                    }
                    return watches;
                }
                let r = m.resistance(soc);
                let u_star = if r == 0.0 { u } else { (m.emf(soc) - r * a0) / (1.0 + r * g) };
                let mut terms_u = cf_motion(-r / (1.0 + r * g));
                let held = draw;
                if held > 0.0 && soc > 0.0 {
                    let (de, dr) = rails::soc_slopes(m, soc);
                    let dsoc_dt = -held / m.capacity_coulombs();
                    let du_dsoc = (de - dr * draw) / (1.0 + r * g);
                    // Each partition following the drain's line moves the rail
                    // by `kappa b w J(f, 0, τ)` more, and `J <= τ / -f`: the
                    // line's rate takes that as its bound (ruling R15 (a)).
                    let rate = du_dsoc * dsoc_dt;
                    let follow: f64 = terms.iter().filter(|t| t.w != 0.0 && t.f < 0.0).map(|t| (r / (1.0 + r * g) * t.b_at(u) * t.w / -t.f).abs()).sum();
                    terms_u.push(Motion::Line { rate: if follow > 0.0 { rate + rate.signum() * follow } else { rate } });
                    let knot = m.next_knot_below(soc).unwrap_or(0.0);
                    watches.push(Watch { value: soc, thresholds: vec![knot], cap: None, terms: vec![Motion::Line { rate: dsoc_dt }] });
                }
                watches.push(Watch { value: u_star, thresholds: windows, cap: Some(rails::RAIL_DRIFT_V), terms: terms_u });
            }
            (HubKind::Psu { v, i_limit }, Regime::Cv) => {
                let dir = direction(*v);
                let mut f = dir * (a0 + g * v);
                let mut motions = cf_motion(dir);
                // The draw the CV -> CC test reads includes what a rail
                // capacitor short of the setpoint takes there (Rails point
                // 5); at esr > 0 that decays with esr C, a moving term of its
                // own. At esr = 0 a capacitor short of the setpoint has
                // already entered CC.
                for (k, sign, far) in self.rail_cap_ends() {
                    let cap = &self.fixed_pair_capacitors[k];
                    if !(cap.esr > 0.0) {
                        continue;
                    }
                    let vc = cap.vc_at(now);
                    let d = dir * ((v - self.voltages[far]) - sign * vc);
                    if d > 0.0 {
                        f += d / cap.esr;
                        motions.push(Motion::Exp { coef: -dir * sign / cap.esr, vc0: vc, inf: cap.d_v, f: -1.0 / (cap.esr * cap.c) });
                    }
                }
                watches.push(Watch { value: f, thresholds: vec![*i_limit, -*i_limit], cap: None, terms: motions });
            }
            (HubKind::Psu { v, i_limit }, Regime::Cc) => {
                let dir = direction(*v);
                match &hub.state.cap_seg {
                    Some(seg) => {
                        let mut th = windows;
                        th.extend(self.rail_kinks());
                        th.push(*v);
                        th.push(hub.state.u_scan + rails::RAIL_SCAN_V);
                        th.push(hub.state.u_scan - rails::RAIL_SCAN_V);
                        let mut rails_p = Vec::new();
                        let rule = hub.resolve_band();
                        self.static_solver.nonlinear_hub_rails(rule, &mut rails_p);
                        self.stepped.nonlinear_hub_rails(rule, &mut rails_p);
                        self.closed_form.nonlinear_hub_rails(rule, &mut rails_p);
                        for (up, band) in rails_p {
                            th.push(up + band);
                            th.push(up - band);
                        }
                        if let Some(modes) = &seg.modes {
                            // Coupled to its partitions (ruling R15 (a)), the
                            // rail's path is theirs too: nothing is held
                            // frozen, so nothing drifts, and the thresholds are
                            // all that end it, each the first passage of the
                            // path, which may turn.
                            watches.push(Watch { value: u, thresholds: th, cap: None, terms: vec![Motion::Path(Rc::clone(modes))] });
                            return watches;
                        }
                        // Each threshold is a log or a line on the rail's own
                        // segment (Rails point 4): the closed-form draws move
                        // the rail only through the capacitor's integral, which
                        // the next solve re-aims. Sharing the distance with
                        // them made every event close on a threshold
                        // geometrically, and a fast inrush ran out of time
                        // resolution short of it.
                        watches.push(Watch { value: u, thresholds: th, cap: None, terms: vec![Motion::Seg(seg.clone())] });
                        // The closed forms follow the segment exactly, and the
                        // segment holds their draws where they stood: a bound
                        // on what their motion does to the rail solves it
                        // again before that reaches RAIL_DRIFT_V (F1r3 finding
                        // B, option (i); re-aiming them at RAIL_RESOLVE_V,
                        // option (ii), was measured too and not kept). A rail
                        // the coupled path cannot take keeps this form.
                        if let Some(bound) = self.cf_drift_bound(now, seg) {
                            watches.push(Watch { value: 0.0, thresholds: Vec::new(), cap: Some(rails::RAIL_DRIFT_V), terms: vec![Motion::Drift(bound)] });
                        }
                    }
                    None => {
                        let u_star = (dir * i_limit - a0) / g;
                        watches.push(Watch { value: u_star, thresholds: windows, cap: Some(rails::RAIL_DRIFT_V), terms: cf_motion(-1.0 / g) });
                        let f = dir * (a0 + g * v);
                        watches.push(Watch { value: f, thresholds: vec![*i_limit], cap: None, terms: cf_motion(dir) });
                    }
                }
            }
        }
        watches
    }

    /// The charge, as volts on the rail capacitors, that the closed-form hub
    /// partitions `parts` drew at `now` beyond what the CC segment held for
    /// them since it was aimed (Rails point 4): their capacitors moved on
    /// along trajectories known in closed form. 0 outside a driven segment.
    /// `parts` are the partitions whose terms the totals read, the ones whose
    /// `b vc` the segment holds as `cf_frozen`.
    fn cf_drift_charge(&self, now: f64, parts: &[usize]) -> f64 {
        let hub = self.hub.as_ref().expect("a hub");
        // A coupled segment holds nothing frozen (ruling R15 (a)).
        let Some(seg) = hub.state.cap_seg.as_ref().filter(|s| s.modes.is_none()) else { return 0.0 };
        let charges: Vec<f64> = parts.iter().filter_map(|&p| self.closed_form.drift_charge(p, now)).collect();
        // A term the totals read has a segment with rail terms, and so a
        // charge. The segment froze the terms the totals read at its own aim;
        // after load that set only ever grows (a partition contributes from
        // its first segment on), so the same count is the same set.
        debug_assert_eq!(charges.len(), parts.len(), "a closed-form term without its charge");
        debug_assert_eq!(parts.len() as u32, seg.cf_count, "the drift charged over other partitions than the segment froze");
        let drawn: f64 = charges.iter().sum();
        (drawn - seg.cf_frozen * (now - seg.t0)) / hub.c_rail
    }

    /// After a solve that aimed a CC rail capacitor's segment at `now`, the
    /// closed-form draws it holds start their charge afresh there.
    fn reset_cf_drift(&mut self, now: f64) {
        let hub = self.hub.as_ref().expect("a hub");
        if hub.state.cap_seg.is_some() {
            let parts = hub.cf_hubs.clone();
            self.closed_form.reset_drift(&parts, now);
        }
    }

    /// How far the closed-form hub partitions driven by the CC rail
    /// capacitor's segment `seg` can move the rail off it after `now` (Rails
    /// point 4), or none when none of their draws moves.
    fn cf_drift_bound(&self, now: f64, seg: &CapSeg) -> Option<DriftBound> {
        let hub = self.hub.as_ref().unwrap();
        let (mut c_sat, mut c1, mut c2) = (0.0, 0.0, 0.0);
        for &p in &hub.cf_hubs {
            if !hub.contrib[p].active {
                continue;
            }
            let Some((b, dvc, f, w1)) = self.closed_form.hub_drift(p, now, seg) else { continue };
            c_sat += (b * dvc).abs();
            c1 += (b * dvc * f).abs();
            c2 += 0.5 * (b * w1).abs();
        }
        (c1 > 0.0 || c2 > 0.0).then_some(DriftBound { c_sat, c1, c2, esr_eff: hub.esr_eff, inv_c_rail: 1.0 / hub.c_rail, alpha: seg.alpha })
    }

    /// The horizon clamp's share of the rail: the stepped hub partitions it
    /// committed contribute, and a CC rail capacitor's closed form, published
    /// live (Rails point 4), is brought to `t_end`. So is a draining cell whose
    /// closed-form partitions follow its path (ruling R15 (a)): they publish
    /// against the rail they followed, which the rail node holds only from
    /// its last solve.
    pub(super) fn rail_at_clamp(&mut self, t_end: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>) -> Result<(), String> {
        if let Some(hub) = self.hub.as_mut() {
            if hub.state.cap_seg.is_some() || hub.state.cell.is_some() {
                hub.dirty = true;
            }
        }
        self.rail_loop(t_end, drivers, readers)
    }

    /// A due `Rail` event (Rails point 4, stat 18).
    pub(super) fn rail_event_at(&mut self, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
        self.written.clear();
        let mut readers = Vec::new();
        {
            let hub = self.hub.as_mut().expect("a Rail event implies a hub");
            hub.stats.events += 1;
            hub.dirty = true;
        }
        let drivers = Rc::clone(&self.driver_snapshot);
        let gates = self.rail_gates();
        let result = self.rail_loop(now, &drivers, &mut readers);
        self.on_error(result)?;
        let result = self.count_gate_flip(gates);
        self.on_error(result)?;
        let published = self.finish_publish(now);
        self.on_error(published)?;
        Ok((Vec::new(), readers))
    }

    /// Whether each IC on the rail is powered, at the rail's `u`.
    fn rail_gates(&self) -> Vec<bool> {
        let hub = self.hub.as_ref().unwrap();
        let at = |node: usize| if node == hub.node { hub.state.u } else { self.voltages[node] };
        hub.rail_ics.iter().map(|&pos| {
            let ic = &self.ics[pos as usize].1;
            digital::power::powered(ic, at(ic.vcc.node), at(ic.gnd.node))
        }).collect()
    }

    /// The power-gate ping-pong across instants (`w1f-power-gate-pingpong`).
    /// Without a rail capacitor the rail jumps with each flip and the rail
    /// loop alternates within one instant, to its pass cap. With one the rail
    /// is continuous: a flip turns the segment back, and the next power-window
    /// event lands the rail two reached widths past the same edge the other
    /// way femtoseconds later, so every instant settles and time crawls on
    /// forever. Counted here as passes of that loop: `Rail` events in a row
    /// that each flipped the gates back where the one before flipped them
    /// from. A `Rail` event that flips nothing ends the run, and so does any
    /// other event (see [`Self::end_gate_chase`]); the horizon clamp does not,
    /// since a caller stepping event by event clamps after each one.
    fn count_gate_flip(&mut self, before: Vec<bool>) -> Result<(), String> {
        let after = self.rail_gates();
        let state = &mut self.hub.as_mut().unwrap().state;
        if after == before {
            state.gate_flips = 0;
            return Ok(());
        }
        state.gate_flips = if state.gate_flips > 0 && after == state.gates_before { state.gate_flips + 1 } else { 1 };
        state.gates_before = before;
        if state.gate_flips >= rails::RAIL_LOOP_MAX {
            return Err(rails::loop_unsettled());
        }
        Ok(())
    }

    /// An event other than `Rail` changed what the rail sees: a rail that
    /// crosses a power edge again after it is answering that, not chasing its
    /// own gate, as a clock swinging a load across an IC's floor does every
    /// half period. Counting those flips too declined such a rail after 50
    /// of them.
    pub(super) fn end_gate_chase(&mut self) {
        if let Some(hub) = self.hub.as_mut() {
            hub.state.gate_flips = 0;
        }
    }

    /// A due waveform source edge (Sources points 2, 3 and 5, stat 19).
    pub(super) fn source_edge_at(&mut self, device: usize, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
        self.written.clear();
        let w = self.wave.wave_of[device] as usize;
        let k = self.wave.sources[w].next.expect("a current Source event has an edge to apply");
        let edge = self.wave.sources[w].edge(k).expect("the scheduled edge exists");
        let drivers = Rc::clone(&self.driver_snapshot);
        // The stepped partitions this source feeds catch up to the edge with
        // the boundary they had (W1b rule 6).
        if edge.moves_boundary {
            let stepped: Vec<usize> = self.wave_parts[w].iter().copied().filter(|&p| self.partition_kinds[p] == PartitionKind::Stepped).collect();
            if !stepped.is_empty() {
                let result = self.stepped.catch_up(&stepped, now, &drivers, &self.live_params, &self.voltages);
                self.on_error(result)?;
            }
        }
        let was_ramping = self.wave.sources[w].ramp.is_some();
        let ramp_changed = was_ramping != edge.ramp.is_some();
        {
            let wave = Rc::make_mut(&mut self.wave);
            let src = &mut wave.sources[w];
            src.value = edge.value;
            src.slot_value = edge.slot;
            src.level_index = edge.level_index;
            src.ramp = edge.ramp;
            src.version += 1;
            if ramp_changed {
                src.watch_gen += 1;
            }
            src.next = src.edge(k + 1).map(|_| k + 1);
            let version = src.version;
            if let Some(next) = src.next.and_then(|n| src.edge(n)) {
                self.pending_schedule.push((next.time, EventKind::Source { device: DeviceId(device as u32), version }));
            }
            if ramp_changed {
                wave.refresh_ramping();
            }
        }
        self.sync_ramps();
        self.static_solver.set_wave_level(w, edge.level_index);
        if ramp_changed && edge.ramp.is_some() {
            self.schedule_ramp_watches(w, now);
        }

        // The boundary under the source, re-derived as the load propagated it.
        let mut dirty: Vec<PartitionId> = Vec::new();
        let mut readers: Vec<DeviceId> = Vec::new();
        let entries = self.wave_entries[w].clone();
        for k in entries {
            let r = self.wave.resolved[k as usize].clone();
            let v = self.wave.source_value(r.device_index, now);
            let parent = self.voltages[r.parent_node];
            let value = if r.parent_is_pos { parent - v } else { parent + v };
            let node = r.child_node;
            if value.to_bits() != self.voltages[node].to_bits() {
                self.write_boundary(node, value);
                self.static_solver.set_boundary(node, value);
                readers.extend(self.node_ic_readers[node].iter().map(|&d| DeviceId(d)));
                // A ramp's midpoint only flips the slot: the value follows
                // the ramp continuously, which every step already reads.
                if edge.moves_boundary {
                    self.reaim_fixed_pair_caps(node, now);
                    dirty.extend(self.node_parts[node].iter().map(|&p| PartitionId(p)));
                    dirty.extend(self.node_watch_parts[node].iter().map(|&p| PartitionId(p)));
                    // A partition reading the node reaches the rail through
                    // its new contribution; the rail equation's own terms on
                    // it do not.
                    if let Some(hub) = self.hub.as_mut().filter(|h| h.direct_nodes[node]) {
                        hub.dirty = true;
                    }
                }
            }
        }
        // A ramp corner changes the slope the stepped partitions follow: they
        // restart there as after any discontinuity (Sources point 3).
        if edge.moves_boundary && (was_ramping || edge.ramp.is_some()) {
            dirty.extend(self.wave_parts[w].iter().map(|&p| PartitionId(p as u32)));
        }
        dirty.sort_unstable();
        dirty.dedup();
        self.source_edges += 1;
        // The static solver publishes the moved boundary and the currents it
        // feeds now: the kernel re-solves `dirty` next, but a boundary that
        // feeds no partition has no other publish.
        self.static_solver.set_inputs_bounded(false);
        let result = self.static_solver.resolve_dirty_into(&[], now, &drivers, &mut readers);
        self.on_error(result)?;
        self.copy_in_static();
        let result = self.rail_loop(now, &drivers, &mut readers);
        self.on_error(result)?;
        let published = self.finish_publish(now);
        self.on_error(published)?;
        Ok((dirty, readers))
    }

    /// Brings every node a running ramp moves to its value at `now` in the
    /// published array and the static solver's (Rails point 7: the boundary
    /// is read at the instant of each stamp and read). Returns whether one
    /// moved.
    pub(super) fn refresh_ramps(&mut self, now: f64) -> bool {
        if !self.wave.any_ramping() {
            return false;
        }
        let mut moved = false;
        for node in 0..self.wave.ramping.len() {
            if !self.wave.ramping[node] {
                continue;
            }
            let value = self.wave.value(node, now, &self.voltages).expect("a ramping node has a value");
            if value.to_bits() != self.voltages[node].to_bits() {
                self.write_boundary(node, value);
                self.static_solver.set_boundary(node, value);
                moved = true;
            }
        }
        moved
    }

    /// Schedules, for a ramp of source `w` that starts at or runs through
    /// `now`, the instant each reader pin, power gate or 555 comparator on a
    /// node it moves crosses its threshold (Sources point 3: linear in `t`,
    /// exact, accepted only past `now + 1e-15`), 1e-9 V past it as every
    /// crossing lands.
    fn schedule_ramp_watches(&mut self, w: usize, now: f64) {
        let src = &self.wave.sources[w];
        let Some(ramp) = src.ramp else { return };
        let (device, version) = (src.device, live::WATCH_BIT | src.watch_gen);
        let (t0, t1) = (ramp.t0, ramp.t0 + ramp.len);
        let mut times = Vec::new();
        for k in &self.wave_entries[w] {
            let node = self.wave.resolved[*k as usize].child_node;
            if !self.wave.ramping[node] {
                continue;
            }
            let (va, vb) = (self.wave.value(node, t0, &self.voltages).unwrap(), self.wave.value(node, t1, &self.voltages).unwrap());
            if va == vb {
                continue;
            }
            let rising = vb > va;
            let mut thresholds = Vec::new();
            for &d in &self.node_ic_readers[node] {
                let Some(&pos) = self.ic_position.get(d as usize).filter(|&&p| p != u32::MAX) else { continue };
                let ic = &self.ics[pos as usize].1;
                let (vv, vg) = (self.voltages[ic.vcc.node], self.voltages[ic.gnd.node]);
                let (lo, hi) = digital::levels::power_window(ic);
                if ic.vcc.node == node {
                    thresholds.extend([vg + lo, vg + hi]);
                }
                if ic.gnd.node == node {
                    thresholds.extend([vv - lo, vv - hi]);
                }
                if digital::is_ne555(ic.kind) {
                    for li in [digital::NE555_TRIG, digital::NE555_RST, digital::NE555_THR] {
                        if ic.logic[li].node == node {
                            thresholds.extend(digital::ne555::watch_thresholds(li, vv, vg).iter().map(|t| t.value));
                        }
                    }
                } else {
                    let power = digital::power::ic_power(ic, vv, vg);
                    for (li, pin) in ic.logic.iter().enumerate() {
                        if pin.node == node && partition::is_watched_read_pin(ic, li) {
                            thresholds.extend(digital::levels::watch_thresholds(ic, &power, pin));
                        }
                    }
                }
            }
            for th in thresholds {
                let target = th + if rising { closed_form::CROSS_NUDGE_V } else { -closed_form::CROSS_NUDGE_V };
                let f = (target - va) / (vb - va);
                if !(f > 0.0 && f < 1.0) {
                    continue;
                }
                let t = t0 + f * (t1 - t0);
                if crate::kernel::accepts_crossing(t, now) && t < t1 {
                    times.push(t);
                }
            }
        }
        times.sort_by(f64::total_cmp);
        times.dedup();
        for t in times {
            self.pending_schedule.push((t, EventKind::Source { device: DeviceId(device as u32), version }));
        }
    }

    /// A reader's threshold crossing on a running ramp: the ramp's nodes are
    /// published at `now` and every IC reading them is evaluated.
    pub(super) fn ramp_crossing_at(&mut self, device: usize, now: f64) -> Result<(Vec<PartitionId>, Vec<DeviceId>), String> {
        self.written.clear();
        let w = self.wave.wave_of[device] as usize;
        let mut readers = Vec::new();
        self.refresh_ramps(now);
        for k in self.wave_entries[w].clone() {
            let node = self.wave.resolved[k as usize].child_node;
            readers.extend(self.node_ic_readers[node].iter().map(|&d| DeviceId(d)));
        }
        let drivers = Rc::clone(&self.driver_snapshot);
        self.static_solver.set_inputs_bounded(false);
        let result = self.static_solver.resolve_dirty_into(&[], now, &drivers, &mut readers);
        self.on_error(result)?;
        self.copy_in_static();
        let published = self.finish_publish(now);
        self.on_error(published)?;
        Ok((Vec::new(), readers))
    }

    /// Every waveform source's digital slot (Readouts: never `NaN`).
    pub(super) fn wave_slots(&self, out: &mut Vec<(usize, f64)>) {
        for src in &self.wave.sources {
            out.push((src.slot, src.slot_value));
        }
    }

    /// One device's state (C ABI, `bo_engine_device_state`), projected to
    /// `now` without committing anything.
    pub(super) fn hub_device_state(&self, device: usize, now: f64, out: &mut Vec<f64>) -> Option<usize> {
        out.clear();
        let program_kind = self.device_kinds.get(device).copied()?;
        if program_kind == kinds::BENCH_PSU {
            if let Some(hub) = self.hub.as_ref().filter(|h| h.device == device) {
                out.push(hub.state.reg.code());
            } else if let Some(ex) = self.exact.as_ref().filter(|h| h.device == device) {
                out.push(ex.reg.code());
            }
            return Some(out.len());
        }
        if program_kind != kinds::BATTERY_PACK {
            return None;
        }
        let (model, state) = if let Some(hub) = self.hub.as_ref().filter(|h| h.device == device) {
            let m = hub.battery()?;
            (m, rails::advance_soc(m, &hub.state.battery, hub.state.held_i, &hub.state.held_cf, now - hub.state.t_last))
        } else if let Some(ex) = self.exact.as_ref().filter(|h| h.device == device) {
            let HubKind::Battery(m) = &ex.kind else { return None };
            (m, m.advance(&ex.battery, ex.held * (now - ex.t_last).max(0.0)).state)
        } else {
            return None;
        };
        let soc = state.soc;
        out.extend([soc, model.emf(soc), model.resistance(soc), model.capacity_coulombs() * soc, state.discharged, state.rejected]);
        Some(out.len())
    }

    // ---- exact mode (Rails point 9) -------------------------------------

    /// Hands the partition solvers exact mode's source stamp from the hub's
    /// present state.
    pub(super) fn push_exact_stamp(&mut self) {
        let stamp = self.exact.as_ref().map(|e| e.stamp());
        self.static_solver.set_exact(stamp);
        self.stepped.set_exact(stamp);
        if let Some(e) = self.exact.as_ref() {
            self.stepped.set_cell_state(e.battery);
        }
    }

    /// After a publish in exact mode: SoC advances with the committed current
    /// and the regime rules run on the solved rail and draw; a transition
    /// re-solves the rail's partition at once.
    fn exact_after_publish(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>) -> Result<(), String> {
        self.drain_contributions();
        let (node, p) = {
            let e = self.exact.as_ref().unwrap();
            (e.node, e.partition)
        };
        // Only a call that solved or committed the rail's partition is a
        // committed step; it is at the partition's own instant. A stepped
        // partition's seed counts even where no step has landed since the
        // last: at load it is the only solve, and a discharged rail
        // capacitor's draw there is the demand that puts a PSU in CC (Rails
        // point 5), which the first step would otherwise meet in CV, charging
        // the capacitor to the setpoint at once.
        let solved = std::mem::take(&mut self.exact_solved);
        let at = match self.partition_kinds[p] {
            PartitionKind::Static => solved.then_some(now),
            _ => self.stepped.committed(p).map(|(t, _)| t).filter(|&t| solved || t > self.exact.as_ref().unwrap().t_last),
        };
        let Some(at) = at else { return self.plan_deferred(drivers) };
        let u = self.voltages[node];
        let draw = -self.exact_hub_current();
        let e = self.exact.as_mut().unwrap();
        if let HubKind::Battery(m) = &e.kind {
            // Each committed step with its own charge: a call that caught the
            // partition up to an event and then re-solved it there publishes
            // the re-solve's current, which is the draw only from the event
            // on, and the next step charges that.
            #[cfg(test)]
            let mut call = Vec::new();
            for (t, q) in self.stepped.take_bills() {
                debug_assert!(t > e.t_last, "a battery step charged twice: {t} after {}", e.t_last);
                e.battery = m.advance(&e.battery, q).state;
                e.t_last = t;
                #[cfg(test)]
                call.push((t, q));
            }
            #[cfg(test)]
            self.bill_log.push(call);
            debug_assert_eq!(e.t_last, at, "every committed battery step is charged");
            debug_assert_eq!(self.stepped.exact_stamp(), Some(e.stamp()), "the call restamped each step from the SoC charged here");
        }
        e.t_last = at;
        e.held = draw;
        let mut transition = false;
        if let HubKind::Psu { v, i_limit } = e.kind {
            let dir = direction(v);
            let next = match e.reg {
                Regime::Cv if draw.abs() > i_limit => Regime::Cc,
                Regime::Cc if dir * u >= v.abs() => Regime::Cv,
                r => r,
            };
            if next != e.reg {
                e.reg = next;
                transition = true;
            }
        }
        self.push_exact_stamp();
        if transition {
            let id = [PartitionId(p as u32)];
            match self.partition_kinds[p] {
                PartitionKind::Static => {
                    self.static_solver.set_inputs_bounded(false);
                    self.static_solver.resolve_dirty_into(&id, now, drivers, readers)?;
                    self.copy_in_static();
                }
                _ => {
                    self.begin_dynamic_writes_list(&[p]);
                    let pre = Rc::clone(&self.driver_snapshot);
                    readers.extend(self.stepped.resolve_dirty(&[p], now, &pre, drivers, &self.live_params, &mut self.voltages, &mut self.currents)?);
                    self.end_dynamic_writes();
                }
            }
            let e = self.exact.as_mut().unwrap();
            e.held = 0.0;
            let draw = -self.exact_hub_current();
            self.exact.as_mut().unwrap().held = draw;
        }
        self.plan_deferred(drivers)
    }

    /// Plans the battery partition's next step from the stamp just set
    /// (`SteppedSolver::set_defer`).
    fn plan_deferred(&mut self, drivers: &[Driver]) -> Result<(), String> {
        self.stepped.plan_deferred(drivers, &self.live_params, &self.voltages)
    }

    /// The exact hub's current into its `pos` from the circuit: the rail's
    /// KCL over every other element, at the published voltages.
    fn exact_hub_current(&self) -> f64 {
        let e = self.exact.as_ref().unwrap();
        let node = e.node;
        let v = &self.voltages;
        let mut leaving = static_part::NODE_RSHUNT_G * v[node];
        for kd in &self.kcl_devices {
            for &(a, b, g) in &kd.contributions {
                if a == node {
                    leaving += g * (v[a] - v[b]);
                } else if b == node {
                    leaving -= g * (v[a] - v[b]);
                }
            }
        }
        for &(d, a, b) in &self.cap_devices {
            if a == node {
                leaving += self.currents[d];
            } else if b == node {
                leaving -= self.currents[d];
            }
        }
        for j in &self.junctions {
            if self.slow.may_have_failed() && self.slow.is_failed(j.device) {
                continue;
            }
            let i = junction_branch_current(j, v);
            if j.anode == node {
                leaving += i;
            } else if j.cathode == node {
                leaving -= i;
            }
        }
        for (pos, (_, ic)) in self.ics.iter().enumerate() {
            let power = digital::power::ic_power(ic, v[ic.vcc.node], v[ic.gnd.node]);
            if !power.powered {
                continue;
            }
            for &id in &self.ic_driver_ids[pos] {
                let Some(driver) = self.driver_snapshot.get(id as usize) else { continue };
                for (target, g) in ic_driver_branches(&power, driver, ic).into_iter().flatten() {
                    let i = g * (v[driver.node] - v[target]);
                    if driver.node == node {
                        leaving += i;
                    } else if target == node {
                        leaving -= i;
                    }
                }
            }
        }
        match (&e.kind, e.reg) {
            (HubKind::Psu { v: vs, i_limit }, Regime::Cc) => -direction(*vs) * i_limit,
            _ => -leaving,
        }
    }
}
