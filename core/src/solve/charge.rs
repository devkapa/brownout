//! W2b piece 4: light means, the charge (core/README.md, "Charge"; W2 design
//! 2.4, ruling R57 D4 (a)).
//!
//! A host arms a list of devices; from then on each one's published current
//! is integrated over time, and each of its channel slots (an LED family
//! device's), until the host takes them: `[t0, t1, then per device its
//! charge and its channels' in channel order]`, the span `[t0, t1]` they
//! cover, and the sums cleared. A light mean is a charge over its span.
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
    /// whose nodes move linearly between instants (every one fixed, where a
    /// running ramp moves it, or one on the hub rail or a static hub
    /// partition's line in it, which follow the rail's model): its current
    /// at the interval's mean voltages.
    Mean(Option<u32>),
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
    /// `∫ leaving` by node, outside and inside a CC stretch of the hub PSU.
    lq: [Vec<f64>; 2],
    /// The CC stretch's own charge, `-dir iLimit` times its length.
    cc_q: f64,
    cc_device: Option<usize>,
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
            } else if let Some(ns) = nodes.as_ref().filter(|ns| ns.iter().all(|&n| self.local_index[n].is_none()) || ns.iter().any(|&n| on_line(n))) {
                line_nodes.extend(ns.iter().copied());
                ChargeClass::Mean(kcl.map(|k| k as u32))
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
            devs.push(ArmedDev { device: d, slot, nch, class, led: led.map(|k| k as u32), ne555, i: 0.0, ch: vec![0.0; nch], since: now, key: Vec::new() });
            slot += 1 + nch;
        }
        let src = self.source_terms(devices, &mut parts, &mut reads, &mut line_nodes);
        parts.sort_unstable();
        parts.dedup();
        line_nodes.sort_unstable();
        line_nodes.dedup();
        line_nodes.retain(|&n| rail == Some(n) || self.local_index[n].is_some_and(|(p, _)| self.partition_kinds[p] == PartitionKind::Static));
        let stepped: Vec<(usize, Vec<usize>)> = parts.iter().map(|&p| (p, reads.remove(&p).unwrap_or_default())).collect();
        self.stepped.set_charge(Some(&stepped));
        let mut st = ChargeState { devs, q: vec![0.0; slot], t0: now, cursor: now, t_arm: now, parts, line_nodes, src, sc: vec![0.0; self.currents.len()], saved: Vec::new(), chs: Vec::new(), steps: Vec::new(), published: Vec::new() };
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
        let mut note = |owner: Option<usize>, nodes: &[usize]| {
            line_nodes.extend_from_slice(nodes);
            if let Some(p) = owner.filter(|&p| self.partition_kinds[p] != PartitionKind::Static) {
                parts.push(p);
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
        Some(SourceTerms { rel, rel_nodes, kcl, caps, junctions, ics, lq: [vec![0.0; n], vec![0.0; n]], cc_q: 0.0, cc_device, cc_prev: false })
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
            _ => false,
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
    fn charge_leaving(&self, src: &mut SourceTerms, bucket: usize, v: &[f64], cap: &dyn Fn(usize) -> f64, include: &dyn Fn(Option<usize>) -> bool, t: f64, w: f64, drivers: &[Driver]) {
        let rel = std::mem::take(&mut src.rel);
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
                if (rel[u] || rel[x]) && include(self.owner_of(u, x)) {
                    add(u, x, g * (v[u] - v[x]));
                }
            }
        }
        if include(None) {
            for &n in &src.rel_nodes {
                add(n, 0, static_part::NODE_RSHUNT_G * v[n]);
            }
        }
        for &k in &src.caps {
            let (d, u, x) = self.cap_devices[k as usize];
            if include(self.owner_of(u, x)) {
                add(u, x, cap(d));
            }
        }
        for &k in &src.junctions {
            let j = &self.junctions[k as usize];
            if (self.slow.may_have_failed() && self.slow.is_failed(j.device)) || !include(self.owner_of(j.anode, j.cathode)) {
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
            for &id in &self.ic_driver_ids[pos as usize] {
                let driver = &drivers[id as usize];
                for branch in ic_driver_branches(&power, driver, ic) {
                    let Some((target, g)) = branch else { continue };
                    if include(self.owner_of(driver.node, target)) {
                        add(driver.node, target, g * (v[driver.node] - v[target]));
                    }
                }
            }
            for pin in &ic.logic {
                if !rel[pin.node] || !include(self.owner_of(pin.node, pin.node)) {
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
        if t > st.cursor {
            let a = st.cursor;
            self.charge_fold_interval(&mut st, a, t);
            st.cursor = t;
        }
        if let Some(s) = st.src.as_mut() {
            s.cc_prev = self.cc_hub_current().is_some();
        }
        self.charge = Some(st);
    }

    /// Refolds each held class whose published value moved, at `now`.
    fn charge_fold_held(&mut self, st: &mut ChargeState, now: f64) {
        let drivers = Rc::clone(&self.driver_snapshot);
        for k in 0..st.devs.len() {
            if !matches!(st.devs[k].class, ChargeClass::Held | ChargeClass::HeldLed(_) | ChargeClass::Held555(_)) || !self.held_moved(&st.devs[k], &drivers) {
                continue;
            }
            let mut ch = std::mem::take(&mut st.devs[k].ch);
            let dev = &st.devs[k];
            let (slot, dt) = (dev.slot, (now - dev.since).max(0.0));
            st.q[slot] += dev.i * dt;
            for (c, &value) in ch.iter().enumerate() {
                st.q[slot + 1 + c] += value * dt;
            }
            let (value, key) = self.held_now(dev, &mut ch, &drivers);
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

    /// Every held class up to `t`, its current and channels times the time
    /// they held.
    fn charge_close_held(st: &mut ChargeState, t: f64) {
        for dev in &mut st.devs {
            if !matches!(dev.class, ChargeClass::Held | ChargeClass::HeldLed(_) | ChargeClass::Held555(_)) {
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
    /// left: the closed forms at their means, the ramped lines at theirs,
    /// the fixed-pair capacitors on their exponentials, and the source terms
    /// outside stepped partitions.
    fn charge_fold_interval(&mut self, st: &mut ChargeState, a: f64, b: f64) {
        let any = st.devs.iter().any(|d| matches!(d.class, ChargeClass::Mean(_) | ChargeClass::FixedCap(_) | ChargeClass::Part(_))) || st.src.is_some();
        if !any {
            return;
        }
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
        for (k, cap) in self.fixed_pair_capacitors.iter().enumerate() {
            let used = st.devs.iter().any(|d| matches!(d.class, ChargeClass::FixedCap(j) if j as usize == k)) || rel_of(&st.src, cap.nodes.0) || rel_of(&st.src, cap.nodes.1);
            if used {
                sc[cap.device_index] = self.fixed_cap_mean(k, a, b, &v);
            }
        }
        let mut chs = std::mem::take(&mut st.chs);
        for k in 0..st.devs.len() {
            let dev = &st.devs[k];
            let i = match dev.class {
                ChargeClass::Mean(Some(kd)) => self.kcl_devices[kd as usize].contributions.iter().map(|&(u, x, g)| g * (v[u] - v[x])).sum::<f64>(),
                ChargeClass::Mean(None) => {
                    chs.clear();
                    chs.resize(dev.nch, 0.0);
                    let i = self.charge_current_on(dev, &v, &sc, &drivers, &mut chs);
                    for (c, &value) in chs.iter().enumerate() {
                        st.q[dev.slot + 1 + c] += value * dt;
                    }
                    i
                }
                ChargeClass::FixedCap(j) => sc[self.fixed_pair_capacitors[j as usize].device_index],
                ChargeClass::Part(p) if published.contains(&(p as usize)) => {
                    chs.clear();
                    chs.resize(dev.nch, 0.0);
                    let i = self.charge_current_on(dev, &v, &sc, &drivers, &mut chs);
                    for (c, &value) in chs.iter().enumerate() {
                        st.q[dev.slot + 1 + c] += value * dt;
                    }
                    i
                }
                _ => continue,
            };
            st.q[dev.slot] += i * dt;
        }
        st.chs = chs;
        if let Some(mut src) = st.src.take() {
            let cc = self.cc_hub_current();
            let bucket = usize::from(cc.is_some());
            if let Some((_, i)) = cc {
                src.cc_q += i * dt;
            }
            // Static partitions and fixed nodes, and the closed forms that
            // published (a stepped partition's terms come with its steps).
            let kinds = &self.partition_kinds;
            let here = |o: Option<usize>| o.is_none_or(|p| kinds[p] == PartitionKind::Static || published.contains(&p));
            let cap_i = |d: usize| sc[d];
            self.charge_leaving(&mut src, bucket, &v, &cap_i, &here, mid, dt, &drivers);
            st.src = Some(src);
        }
        st.published = published;
        Self::charge_restore(&mut saved, &mut v);
        self.voltages = v;
        st.saved = saved;
        st.sc = sc;
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
                let (t, x, i) = if k == 0 { (step.t0, &step.x0, &step.i0) } else { (step.knots[k - 1].t, &step.knots[k - 1].x, &step.knots[k - 1].i) };
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
                for dev in &st.devs {
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
                    let own = |o: Option<usize>| o == Some(p);
                    let cap_i = |d: usize| sc[d];
                    self.charge_leaving(s, bucket, &v, &cap_i, &own, t, wk, &drivers);
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
        out.clear();
        out.extend([st.t0, t1]);
        out.extend_from_slice(&st.q);
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
        n
    }
}
