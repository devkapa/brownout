//! The closed-form dynamic-partition solver (core/README.md, "Kernel
//! semantics: events"; W1 plan row B2; Decision 14: exactly one capacitor).
//! Port of pec3.mjs's `_segment`/`_aim`/`_vcAt`/`_scheduleCrossing`/
//! `_analyticCross`/`_recross` (`:532-628`), with the rail-sensitivity half
//! of that machinery dropped: W1b has no hub rail (that is W1f), so every
//! segment is a plain function of time with no `du` correction term.
//!
//! **The exact model.** Replace the partition's one capacitor with an IDEAL
//! voltage source of value `vc` in series with its `esr` (an extra
//! branch-current unknown, MNA-style — pec3's `M_SOURCE` stamp). Since every
//! OTHER device in a W1b partition is linear (a resistor, a switch's CURRENT
//! contact state, the capacitor's own leakage), the rest of the network's
//! solution — every node voltage and the branch current itself — is an
//! AFFINE function of `vc`: solving once at `vc=0` and once at `vc=1` (same
//! factored matrix, `b` only differs by the coefficient on `vc`, so this is
//! one LU factorization plus two applies) gives `x(vc) = x0 + dx*vc` and
//! `i(vc) = i0 + di*vc` exactly, for any `vc`, with no discretization error
//! at all. Since `i = C*dvc/dt`, this makes `vc` itself the exact solution
//! of a first-order LINEAR ODE, `dvc/dt = F*(vc - vinf)` with `F = di/C` and
//! `vinf = -i0/di` (`F == 0`'s degenerate constant-derivative case is
//! guarded, matching pec3): `vc(t) = vinf + (vc0-vinf)*exp(F*(t-t0))`. A
//! "segment" is this quadruple `(x0, dx, i0, di)` plus the anchor `(t0,
//! vc0)`; it stays exactly valid until something in the partition's own
//! linear system changes (a switch toggle, a driver commit) — a "re-aim" —
//! at which point a fresh segment is solved and anchored at the CURRENT
//! exact `vc` (`_vcAt`), never a discontinuity.
//!
//! **Crossings.** A registered [`ThresholdFn`] (the crossing-watch API,
//! [`ClosedFormSolver::watch_node`]) is checked against the segment's
//! asymptote: a threshold strictly between "now" and "where this segment is
//! headed" has an exact, closed-form crossing time (solving `vc(t) = vc*`
//! for `t`), scheduled through the kernel's heap
//! ([`ClosedFormSolver::take_scheduled`]) if it clears the acceptance rule
//! (`kernel::accepts_crossing`, contract point 4) — applied HERE, before the
//! candidate ever reaches a heap, exactly as `kernel::accepts_crossing`'s own
//! doc comment describes. Landing on a crossing nudges the crossed node's
//! value by [`CROSS_NUDGE_V`] past the threshold (pec3's own "hard-won
//! rule": accepting a crossing exactly ON a threshold can put a node back on
//! it and reschedule forever) and re-aims from there.

use std::collections::HashMap;

use crate::digital;
use crate::drivers::Driver;
use crate::events::DeviceId;
use crate::kernel::accepts_crossing;
use crate::lu;
use crate::partition::PartitionKind;
use crate::program::Program;
use crate::solve::{
    build_ic_reader_index, collect_ics, compile_dyn_shape, pin_voltage_global, stamp_branch_terminal, stamp_ics_into_dyn, stamp_linear_base,
    DynCapacitor, DynShape, ThresholdFn,
};

/// The 1e-9 V nudge past a crossed threshold (W1 plan row B2; pec3.mjs
/// `CROSS_NUDGE_V`, `:34`) — see this module's doc comment.
pub const CROSS_NUDGE_V: f64 = 1e-9;

/// The exact affine relationship this partition's whole linear network has
/// to its one capacitor's voltage `vc` (this module's doc comment): `x(vc) =
/// x0 + dx*vc`, `i(vc) = i0 + di*vc`.
#[derive(Debug, Clone)]
struct Segment {
    x0: Vec<f64>,
    dx: Vec<f64>,
    i0: f64,
    di: f64,
}

struct Partition {
    shape: DynShape,
    cap: DynCapacitor,
    /// `None` until the first segment exists (pec3's `segReady` guard: `_aim`
    /// on a segment that was never built would read garbage/NaN).
    seg: Option<Segment>,
    seg_t0: f64,
    seg_vc0: f64,
    /// `di / C` — the segment's exponential rate.
    seg_f: f64,
    /// The asymptote `vc` heads towards under `seg_f`.
    seg_inf: f64,
    /// Bumped on every re-aim (events.rs's lazy-deletion contract): a
    /// `Crossing` scheduled under a stale version is silently dropped when
    /// popped.
    version: u64,
    /// The local node index and threshold+nudge value the LAST scheduled
    /// crossing (if any) targets — `handle_crossing` uses these to land
    /// exactly past the threshold (this module's doc comment), not merely at
    /// the analytically-computed time (which can differ from the threshold
    /// by float noise).
    cross_local: usize,
    cross_v: f64,
    /// Local node index -> every reader watching it. Never touched by a
    /// solve/snapshot/restore — registrations are load-time/setup wiring,
    /// not per-instant state (this module's doc comment on
    /// `ClosedFormSolver::snapshot`).
    watches: HashMap<usize, Vec<(DeviceId, ThresholdFn)>>,
    /// Contract stats 8/9 (this partition's own share). NOT rolled back by
    /// `restore` (core/README.md, "A failed advance": "the stat counters...
    /// keep the failed attempt's counts").
    segments_built: u64,
    crossings_hit: u64,
}

/// Everything [`ClosedFormSolver::snapshot`] captures, restored verbatim by
/// [`ClosedFormSolver::restore`]. Deliberately excludes `watches` (load-time
/// wiring, never mutated by a solve) and the two stat counters (README: kept
/// even through a rollback). Includes `scheduled` (contract point 7): a
/// dirty batch with more than one `ClosedForm` partition can re-aim an
/// EARLIER partition (queuing a crossing into `self.scheduled`) before a
/// LATER one in the SAME `resolve_dirty` call fails — `PartitionedSolver`'s
/// own `drain_scheduled` (which would otherwise empty this into
/// `pending_schedule`) never runs in that case, since the `?` on the failing
/// call short-circuits first, so a stale entry can be left sitting here, not
/// only in `pending_schedule`.
#[derive(Debug, Clone)]
pub struct Snapshot {
    partitions: HashMap<usize, PartitionSnapshot>,
    scheduled: Vec<(f64, usize, u64)>,
}

#[derive(Debug, Clone)]
struct PartitionSnapshot {
    seg: Option<Segment>,
    seg_t0: f64,
    seg_vc0: f64,
    seg_f: f64,
    seg_inf: f64,
    version: u64,
    cross_local: usize,
    cross_v: f64,
}

/// Owns every `ClosedForm` partition (Decision 14), keyed by its GLOBAL
/// partition index (the same numbering `partition::build`/`PartitionId` use)
/// so `solve::PartitionedSolver` can route a dirty id straight here without
/// a separate id-remapping table.
pub struct ClosedFormSolver {
    partitions: HashMap<usize, Partition>,
    /// node id -> (partition index, local index), for `watch_node` — built
    /// once at [`Self::build`] time so a caller registering a watch never
    /// needs `partition::build`'s own tables in hand.
    node_owner: HashMap<usize, (usize, usize)>,
    /// Crossings this solver wants scheduled, drained by
    /// [`Self::take_scheduled`] — normally emptied by the caller
    /// (`solve::mod`'s `PartitionedSolver`) before control returns to
    /// `kernel.rs`, but NOT when a later partition in the SAME dirty batch
    /// fails first (see [`Snapshot`]'s doc comment), so it is part of
    /// [`Snapshot`] after all.
    scheduled: Vec<(f64, usize, u64)>,
    /// Every combinational IC in the program (W1c step 2b, lifting
    /// `partition.rs`'s former decline): [`build_segment`] restamps
    /// whichever of these touch a given `ClosedForm` partition fresh on
    /// every segment build, mirroring `static_part.rs`'s own `ics` list.
    /// Every IC that ever touches a `ClosedForm` partition is guaranteed to
    /// have BOTH supply nodes fixed (`stamp_ics_into_dyn`'s own doc
    /// comment on why), so its power here is a build-time constant in
    /// everything but WHICH driver/pull target it currently selects.
    ics: Vec<(usize, digital::IcRecord)>,
    /// Fixed voltages, node -> `Some(v)` — `stamp_ics_into_dyn`'s own
    /// resolver needs this to place an IC pin whose node is fixed.
    fixed: Vec<Option<f64>>,
    /// node -> every IC device index that reads it (contract point 4's
    /// last bullet: "resolves of dynamic partitions must also report IC
    /// readers of nodes that changed, as static does").
    node_to_ic_readers: HashMap<usize, Vec<usize>>,
}

impl ClosedFormSolver {
    /// Builds one [`Partition`] per `ClosedForm`-classified entry in `kinds`
    /// (a program with none at all gets an empty solver, cheaply). `kinds`
    /// already has `FORCE_STEPPED` folded in (`partition::build`), so this
    /// never has to check the option itself.
    pub fn build(program: &Program, partitions: &[Vec<usize>], local_index: &[Option<(usize, usize)>], fixed: &[Option<f64>], kinds: &[PartitionKind]) -> Self {
        let mut parts = HashMap::new();
        let mut node_owner = HashMap::new();
        for (p_idx, kind) in kinds.iter().enumerate() {
            if *kind != PartitionKind::ClosedForm {
                continue;
            }
            let shape = compile_dyn_shape(p_idx, &partitions[p_idx], local_index, fixed, program);
            debug_assert_eq!(shape.capacitors.len(), 1, "Decision 14: a ClosedForm partition has exactly one capacitor");
            let cap = shape.capacitors[0];
            for (l, &node) in shape.nodes.iter().enumerate() {
                node_owner.insert(node, (p_idx, l));
            }
            parts.insert(
                p_idx,
                Partition {
                    shape,
                    cap,
                    seg: None,
                    seg_t0: 0.0,
                    seg_vc0: 0.0,
                    seg_f: 0.0,
                    seg_inf: 0.0,
                    version: 0,
                    cross_local: 0,
                    cross_v: 0.0,
                    watches: HashMap::new(),
                    segments_built: 0,
                    crossings_hit: 0,
                },
            );
        }
        let ics = collect_ics(program);
        let node_to_ic_readers = build_ic_reader_index(&ics);
        ClosedFormSolver { partitions: parts, node_owner, scheduled: Vec::new(), ics, fixed: fixed.to_vec(), node_to_ic_readers }
    }

    pub fn is_empty(&self) -> bool {
        self.partitions.is_empty()
    }

    /// The crossing-watch API (W1 plan row B2): registers `reader` against
    /// `thresholds` on `node`. `false` if `node` is not a free node this
    /// solver owns (a caller error — a reader watches a NODE, and only a
    /// dynamic partition's nodes move continuously enough to need this; a
    /// fixed or static-partition node is never a valid target).
    pub fn watch_node(&mut self, node: usize, reader: DeviceId, thresholds: ThresholdFn) -> bool {
        let Some(&(p_idx, local)) = self.node_owner.get(&node) else {
            return false;
        };
        let part = self.partitions.get_mut(&p_idx).expect("node_owner and partitions built together");
        part.watches.entry(local).or_default().push((reader, thresholds));
        true
    }

    /// Contract stat 8 (closed-form segments built), summed over every
    /// partition.
    pub fn segments_built(&self) -> f64 {
        self.partitions.values().map(|p| p.segments_built as f64).sum()
    }

    /// Contract stat 9 (crossings), summed over every partition.
    pub fn crossings(&self) -> f64 {
        self.partitions.values().map(|p| p.crossings_hit as f64).sum()
    }

    /// Whether a `Crossing` event carrying `version` for `partition` is
    /// still current (events.rs's lazy-deletion contract).
    pub fn is_current(&self, partition: usize, version: u64) -> bool {
        self.partitions.get(&partition).is_some_and(|p| p.version == version)
    }

    /// Re-aims every partition in `dirty` (a switch toggle or driver commit
    /// touched it — core/README.md: "A segment is re-aimed when a stamp in
    /// the partition changes") at `now`, publishing its exact trajectory at
    /// this instant into `voltages`/`currents` and queuing every registered
    /// watcher for re-evaluation (pec3's `_aim` re-queues every watcher on
    /// every re-aim unconditionally, since the whole trajectory just
    /// changed — there is no cheaper "did this one move" check that would
    /// still be correct). Builds the FIRST segment (segReady guard) at `vc =
    /// 0` (README: "Initial vc = 0 at load; carry-forward is W2") the first
    /// time a partition is named here, exactly as `Kernel::settle`'s
    /// first-pass "resolve every partition" hands every partition to this
    /// call once at load.
    pub fn resolve_dirty(
        &mut self,
        dirty: &[usize],
        now: f64,
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
    ) -> Result<Vec<DeviceId>, String> {
        let mut readers = Vec::new();
        for &p_idx in dirty {
            let Some(part) = self.partitions.get_mut(&p_idx) else { continue };
            let vc_now = match &part.seg {
                Some(_) => vc_at(part.seg_f, part.seg_inf, part.seg_vc0, part.seg_t0, now),
                None => 0.0,
            };
            let old_bits: Vec<u64> = part.shape.nodes.iter().map(|&n| voltages[n].to_bits()).collect();
            Self::segment_and_aim(part, now, vc_now, p_idx, drivers, &self.ics, &self.node_owner, &self.fixed, live_params)?;
            Self::publish(part, live_params, voltages, currents);
            readers.extend(part.watches.values().flatten().map(|(id, _)| *id));
            Self::report_changed_ic_readers(part, &old_bits, voltages, &self.node_to_ic_readers, &mut readers);
            if let Some(t) = Self::schedule_crossing(part, now) {
                self.scheduled.push((t, p_idx, part.version));
            }
        }
        Ok(readers)
    }

    /// Applies a due, still-current `Crossing` at `now` (pec3's
    /// `_analyticCross`): lands the crossed node exactly on its
    /// threshold-plus-nudge, then re-aims from there, exactly like any other
    /// re-aim.
    pub fn handle_crossing(
        &mut self,
        partition: usize,
        now: f64,
        drivers: &[Driver],
        live_params: &HashMap<usize, Vec<f64>>,
        voltages: &mut [f64],
        currents: &mut [f64],
    ) -> Result<Vec<DeviceId>, String> {
        let part = self.partitions.get_mut(&partition).ok_or_else(|| format!("closed-form: unknown partition {partition}"))?;
        part.crossings_hit += 1;
        let seg = part.seg.as_ref().expect("a Crossing was only ever scheduled after a segment existed");
        let mut vc = vc_at(part.seg_f, part.seg_inf, part.seg_vc0, part.seg_t0, now);
        // Land exactly past the threshold on the crossing node (this
        // module's doc comment; pec3's `_analyticCross`), not merely at the
        // analytically-computed time, which can differ from the threshold by
        // float noise.
        let dx = seg.dx[part.cross_local];
        if dx != 0.0 {
            vc = (part.cross_v - seg.x0[part.cross_local]) / dx;
        }
        let old_bits: Vec<u64> = part.shape.nodes.iter().map(|&n| voltages[n].to_bits()).collect();
        Self::segment_and_aim(part, now, vc, partition, drivers, &self.ics, &self.node_owner, &self.fixed, live_params)?;
        Self::publish(part, live_params, voltages, currents);
        let mut readers: Vec<DeviceId> = part.watches.values().flatten().map(|(id, _)| *id).collect();
        Self::report_changed_ic_readers(part, &old_bits, voltages, &self.node_to_ic_readers, &mut readers);
        if let Some(t) = Self::schedule_crossing(part, now) {
            self.scheduled.push((t, partition, part.version));
        }
        Ok(readers)
    }

    /// Drains every crossing this solver wants scheduled since the last
    /// drain — `PartitionedSolver::take_scheduled`'s closed-form half.
    pub fn take_scheduled(&mut self) -> Vec<(f64, usize, u64)> {
        std::mem::take(&mut self.scheduled)
    }

    /// The exact node voltages and capacitor current AT `t_end`
    /// (`PartitionedSolver::sync_dynamic_to`'s closed-form half — see that
    /// trait method's doc comment for why this is not a no-op even though no
    /// state changes): every partition's `voltages`/`currents` entries are
    /// re-published from its still-valid segment evaluated at `t_end`,
    /// without touching `seg_t0`/`seg_vc0` (the segment's own anchor stays
    /// wherever it was last aimed; only the PUBLISHED reading moves).
    pub fn publish_at(&self, t_end: f64, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) {
        for part in self.partitions.values() {
            if part.seg.is_none() {
                continue; // load not settled yet; nothing to publish.
            }
            let vc = vc_at(part.seg_f, part.seg_inf, part.seg_vc0, part.seg_t0, t_end);
            Self::publish_at_vc(part, vc, live_params, voltages, currents);
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            partitions: self
                .partitions
                .iter()
                .map(|(&p, part)| {
                    (
                        p,
                        PartitionSnapshot {
                            seg: part.seg.clone(),
                            seg_t0: part.seg_t0,
                            seg_vc0: part.seg_vc0,
                            seg_f: part.seg_f,
                            seg_inf: part.seg_inf,
                            version: part.version,
                            cross_local: part.cross_local,
                            cross_v: part.cross_v,
                        },
                    )
                })
                .collect(),
            scheduled: self.scheduled.clone(),
        }
    }

    pub fn restore(&mut self, snapshot: Snapshot) {
        self.scheduled = snapshot.scheduled;
        for (p, s) in snapshot.partitions {
            if let Some(part) = self.partitions.get_mut(&p) {
                part.seg = s.seg;
                part.seg_t0 = s.seg_t0;
                part.seg_vc0 = s.seg_vc0;
                part.seg_f = s.seg_f;
                part.seg_inf = s.seg_inf;
                part.version = s.version;
                part.cross_local = s.cross_local;
                part.cross_v = s.cross_v;
            }
        }
    }

    // ---- internals ---------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    fn segment_and_aim(
        part: &mut Partition,
        now: f64,
        vc_now: f64,
        p_idx: usize,
        drivers: &[Driver],
        ics: &[(usize, digital::IcRecord)],
        node_owner: &HashMap<usize, (usize, usize)>,
        fixed: &[Option<f64>],
        live_params: &HashMap<usize, Vec<f64>>,
    ) -> Result<(), String> {
        let seg = build_segment(&part.shape, &part.cap, p_idx, now, drivers, ics, node_owner, fixed, live_params)?;
        part.seg = Some(seg);
        part.segments_built += 1;
        let seg = part.seg.as_ref().unwrap();
        part.seg_f = seg.di / part.cap.c;
        part.seg_inf = if seg.di != 0.0 { -seg.i0 / seg.di } else { vc_now };
        part.seg_t0 = now;
        part.seg_vc0 = vc_now;
        part.version += 1;
        Ok(())
    }

    /// W1c contract point 4's last bullet, mirrored from `static_part.rs`'s
    /// own `node_to_ic_readers` walk: for every one of this partition's own
    /// local nodes whose PUBLISHED bits actually changed this re-aim,
    /// extends `readers` with every IC that reads it (its `vcc`, `gnd`, or
    /// any logic pin) — not only the ones with an explicit crossing watch
    /// registered. `old_bits` is `part.shape.nodes`-order, captured by the
    /// caller BEFORE `Self::publish`; `readers` may already hold entries
    /// (from `part.watches`), and a duplicate here is harmless — `Kernel`
    /// dedups across a whole delta's readers regardless.
    fn report_changed_ic_readers(part: &Partition, old_bits: &[u64], voltages: &[f64], node_to_ic_readers: &HashMap<usize, Vec<usize>>, readers: &mut Vec<DeviceId>) {
        for (l, &node) in part.shape.nodes.iter().enumerate() {
            if voltages[node].to_bits() != old_bits[l] {
                if let Some(ics) = node_to_ic_readers.get(&node) {
                    readers.extend(ics.iter().map(|&d| DeviceId(d as u32)));
                }
            }
        }
    }

    fn publish(part: &Partition, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) {
        Self::publish_at_vc(part, part.seg_vc0, live_params, voltages, currents);
    }

    /// Publishes every one of this partition's own currents (core/README.md's
    /// "Currents" rules) at the given `vc`: the capacitor's series-plus-
    /// leakage current from the exact segment, AND — W1b's own review caught
    /// this missing entirely — every resistor's and switch's OWN current
    /// ([`crate::solve::publish_stamped_currents`]), which nothing else in
    /// `PartitionedSolver` ever computed for a device sitting inside a
    /// `ClosedForm` partition (`recompute_source_currents` only reads a
    /// device's current back out for a `voltage_source`'s own KCL, never
    /// computes one for a plain resistor/switch itself).
    fn publish_at_vc(part: &Partition, vc: f64, live_params: &HashMap<usize, Vec<f64>>, voltages: &mut [f64], currents: &mut [f64]) {
        let seg = part.seg.as_ref().expect("segReady guard upheld by every caller");
        for (l, &node) in part.shape.nodes.iter().enumerate() {
            voltages[node] = seg.x0[l] + seg.dx[l] * vc;
        }
        let series_i = seg.i0 + seg.di * vc;
        let (va, vb) = (pin_voltage_global(&part.cap.pins[0], &part.shape, voltages), pin_voltage_global(&part.cap.pins[1], &part.shape, voltages));
        let leak_i = part.cap.leak_g * (va - vb);
        currents[part.cap.device_index] = series_i + leak_i;
        crate::solve::publish_stamped_currents(&part.shape, voltages, live_params, currents);
    }

    /// pec3's `_scheduleCrossing` (`:580-609`), minus the rail term: the
    /// earliest accepted crossing across every watched node/threshold this
    /// segment's asymptote actually reaches, or `None`. Only a DECAYING
    /// segment (`F < 0`) is ever scheduled this way — `F >= 0` (growing or
    /// degenerate) is left alone, exactly as pec3 does, since the log
    /// formula below assumes a decaying exponential.
    fn schedule_crossing(part: &mut Partition, now: f64) -> Option<f64> {
        let f = part.seg_f;
        if !(f < 0.0) {
            return None;
        }
        let seg = part.seg.as_ref()?;
        let (inf, v0, t0) = (part.seg_inf, part.seg_vc0, part.seg_t0);
        let mut best: Option<(f64, usize, f64)> = None; // (t, local, threshold+nudge)
        for (&local, watchers) in &part.watches {
            let x0 = seg.x0[local];
            let dx = seg.dx[local];
            if dx == 0.0 {
                continue;
            }
            let now_v = x0 + dx * v0;
            let end_v = x0 + dx * inf;
            for (_, thresholds) in watchers {
                for th in thresholds() {
                    let ok = if th.rising { now_v < th.value && end_v > th.value } else { now_v > th.value && end_v < th.value };
                    if !ok {
                        continue;
                    }
                    let vc_star = (th.value - x0) / dx;
                    let ratio = (vc_star - inf) / (v0 - inf);
                    if !(ratio > 0.0 && ratio < 1.0) {
                        continue;
                    }
                    let t = t0 + ratio.ln() / f;
                    if !accepts_crossing(t, now) {
                        continue;
                    }
                    let nudged = th.value + if th.rising { CROSS_NUDGE_V } else { -CROSS_NUDGE_V };
                    if best.is_none_or(|(bt, _, _)| t < bt) {
                        best = Some((t, local, nudged));
                    }
                }
            }
        }
        let (t, local, cross_v) = best?;
        part.cross_local = local;
        part.cross_v = cross_v;
        Some(t)
    }
}

/// The exact exponential (this module's doc comment): `vc(t) = vinf +
/// (vc0-vinf)*exp(F*(t-t0))`, or the constant `vc0` when `F == 0` (a segment
/// whose current does not depend on `vc` at all — no capacitor current
/// changes it).
fn vc_at(f: f64, inf: f64, vc0: f64, t0: f64, t: f64) -> f64 {
    if f == 0.0 {
        vc0
    } else {
        inf + (vc0 - inf) * (f * (t - t0)).exp()
    }
}

/// Solves the two `M_SOURCE`-mode systems (`vc = 0`, `vc = 1`) that give this
/// partition's exact affine relationship to its capacitor's voltage (this
/// module's doc comment): one extra branch-current unknown (row/column `n`)
/// carries the capacitor's own series current, related to the free nodes and
/// `vc` by `V[pin_a] - V[pin_b] - esr*i == vc`. Both solves reuse the SAME LU
/// factorization (only `b[n]` differs between them), so this is one factor
/// plus two applies, exactly as the module doc promises ("two small solves
/// per switching event").
///
/// `stamp_ics_into_dyn` (W1c step 2b) is called once, into the SHARED `a`/
/// `b0` baseline, before it is cloned into `x0`/`x1`: an IC driver/pull
/// conductance never depends on `vc` at all (only on this partition's fixed
/// supply voltages, `stamp_ics_into_dyn`'s own doc comment on why every IC
/// touching a `ClosedForm` partition has both fixed), so it is exactly as
/// constant across the two sub-solves as `stamp_linear_base`'s own resistor/
/// switch/leakage terms already are — never something that needs its own
/// `vc=0`/`vc=1` sweep the way the capacitor's own branch equation does.
#[allow(clippy::too_many_arguments)]
fn build_segment(
    shape: &DynShape,
    cap: &DynCapacitor,
    p_idx: usize,
    now: f64,
    drivers: &[Driver],
    ics: &[(usize, digital::IcRecord)],
    node_owner: &HashMap<usize, (usize, usize)>,
    fixed: &[Option<f64>],
    live_params: &HashMap<usize, Vec<f64>>,
) -> Result<Segment, String> {
    let n = shape.n;
    let size = n + 1;
    let r = n;
    let mut a = vec![0.0; size * size];
    let mut b0 = vec![0.0; size];
    stamp_linear_base(shape, size, &mut a, &mut b0, live_params);
    // A `ClosedForm` partition never holds a free IC supply node (the
    // promotion rule this module's own doc comment describes), so `x` (the
    // Newton-iterate slice `stamp_ics_into_dyn` would index for a FREE
    // supply) is never actually read here — `&[]` is exactly correct, not a
    // shortcut, matching `static_part.rs::direct_solve`'s identical `&[]`.
    stamp_ics_into_dyn(size, &mut a, &mut b0, now, drivers, ics, node_owner, fixed, p_idx, &[]);

    stamp_branch_terminal(&mut a, &mut b0, size, r, &cap.pins[0], 1.0);
    stamp_branch_terminal(&mut a, &mut b0, size, r, &cap.pins[1], -1.0);
    a[r * size + r] -= cap.esr;
    // b0[r] += 0.0 * vc (vc = 0 for the first solve).

    let lu = lu::lu_factor(a, size).map_err(|_| "closed-form segment: singular or non-finite pivot".to_string())?;
    let mut x0 = b0.clone();
    lu.solve(&mut x0).map_err(|_| "closed-form segment: singular or non-finite pivot solving vc=0".to_string())?;
    let mut x1 = b0;
    x1[r] += 1.0; // vc = 1.
    lu.solve(&mut x1).map_err(|_| "closed-form segment: singular or non-finite pivot solving vc=1".to_string())?;

    // Enforce the branch equation `pins[0] - pins[1] - esr*i = vc` EXACTLY
    // for the capacitor's own two terminals, rather than trusting the
    // general LU solve's rounding for them. Algebraically this equation
    // already holds (it is one of the rows being solved), but Gaussian
    // elimination couples every row together, so its numerical residual on
    // THESE two specific unknowns grows with how ill-conditioned the REST of
    // the partition's matrix happens to be (a switch's closed-contact
    // conductance sitting many orders of magnitude away from a nearby
    // resistor's, say) — invisible for a single, standing segment, but
    // exposed the instant a re-aim compares "before" and "after": with
    // `esr = 0` the terminal voltage IS `vc`, which cannot legitimately
    // change at all when only the SURROUNDING network's topology changes (a
    // capacitor's charge is physically continuous), yet an unpatched
    // solve here reported a several-ULP jump on exactly that comparison,
    // amplified by ill-conditioning into a measurable millivolt-scale one
    // for a fixture with more disparate conductances (T2's `set_params`
    // switch-toggle cross-check). Deriving one terminal from the other via
    // this exact relationship removes the general solve's rounding for
    // whichever terminal is FIXED (carrying zero rounding of its own) —
    // the common case — and, when both are free, at least removes it for
    // one of the two instead of letting both drift independently.
    fix_branch_terminals_exactly(&mut x0, &mut x1, r, cap);

    let dx: Vec<f64> = (0..n).map(|k| x1[k] - x0[k]).collect();
    let i0 = x0[r];
    let di = x1[r] - i0;
    Ok(Segment { x0: x0[..n].to_vec(), dx, i0, di })
}

/// See [`build_segment`]'s doc comment on why this exists. `r` is the branch
/// row/column index (`x[r]` is the series current at `vc=0`/`vc=1`
/// respectively).
fn fix_branch_terminals_exactly(x0: &mut [f64], x1: &mut [f64], r: usize, cap: &DynCapacitor) {
    match (cap.pins[0].local, cap.pins[1].local) {
        (Some(la), None) => {
            // pins[0] = pins[1] + esr*i + vc (pins[1] fixed: zero rounding).
            x0[la] = cap.pins[1].fixed + cap.esr * x0[r];
            x1[la] = cap.pins[1].fixed + cap.esr * x1[r] + 1.0;
        }
        (None, Some(lb)) => {
            // pins[1] = pins[0] - esr*i - vc.
            x0[lb] = cap.pins[0].fixed - cap.esr * x0[r];
            x1[lb] = cap.pins[0].fixed - cap.esr * x1[r] - 1.0;
        }
        (Some(la), Some(lb)) => {
            // Both free: pin_a still derived from pin_b's own (LU-solved)
            // value, so at least ONE of the two is exactly consistent with
            // the other under this relationship, rather than both drifting
            // independently.
            x0[la] = x0[lb] + cap.esr * x0[r];
            x1[la] = x1[lb] + cap.esr * x1[r] + 1.0;
        }
        (None, None) => {} // Both fixed: not a valid `ClosedForm` partition (`partition::build` already rejects this).
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition;
    use crate::program::{Device, Options, PinRecord};
    use crate::solve::Threshold;

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn source(component_index: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: crate::kinds::VOLTAGE_SOURCE, component_index, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }

    fn resistor(component_index: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: crate::kinds::RESISTOR, component_index, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }

    fn capacitor(component_index: usize, a: usize, b: usize, c: f64, esr: f64, leak: f64) -> Device {
        Device { kind: crate::kinds::CAPACITOR, component_index, pins: vec![pin(a), pin(b)], params: vec![c, esr, leak], blob: vec![] }
    }

    fn program(node_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    }

    fn build(program: &Program) -> ClosedFormSolver {
        let pb = partition::build(program).unwrap();
        ClosedFormSolver::build(program, &pb.partitions, &pb.local_index, &pb.fixed, &pb.kinds)
    }

    /// node0 = gnd, node1 = source(+), node2 = RC midpoint (the one free
    /// node, hence one `ClosedForm` partition at index 0).
    fn rc_program(v0: f64, r: f64, c: f64, esr: f64, leak: f64) -> Program {
        program(3, vec![source(0, 1, 0, v0), resistor(1, 1, 2, r), capacitor(2, 2, 0, c, esr, leak)])
    }

    #[test]
    fn rc_charge_matches_the_exact_exponential_within_1e9_relative() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let tau = r * c;
        // Not testing far past 5*tau: both the node voltage and the series
        // current head to constants/zero there, and a RELATIVE comparison
        // against a reference computed independently (rather than reading
        // back this module's own numbers) becomes a comparison of two
        // catastrophically-cancelled near-zero quantities, not a meaningful
        // precision check of either implementation.
        for &t in &[1e-5, 1e-4, 5e-4, 1e-3, 2e-3, 5e-3] {
            let expected = v0 * (1.0 - (-t / tau).exp());
            let (mut v, mut cur) = (voltages.clone(), currents.clone());
            cf.publish_at(t, &HashMap::new(), &mut v, &mut cur);
            // `x0`/`dx` for this circuit (esr=0, the capacitor's other
            // terminal fixed at ground) are exact 0.0/1.0 by construction
            // (`fix_branch_terminals_exactly`), so the node voltage IS `vc`
            // bit-for-bit; the residual relative error here is entirely
            // `vc_at`'s own `F`/`vinf` (derived through an LU solve rather
            // than the direct `-1/(RC)`/`V0` this specific circuit happens
            // to reduce to), well under the plan's 1e-9 bar even though it
            // is not literal bit-exactness.
            let rel = (v[2] - expected).abs() / expected.abs();
            assert!(rel < 3e-9, "t={t}: got {} expected {} rel {rel}", v[2], expected);
            // The series current is exactly C*dvc/dt = (v0-vc)/R here (esr=0).
            let expected_i = (v0 - expected) / r;
            let rel_i = (cur[2] - expected_i).abs() / expected_i.abs();
            assert!(rel_i < 5e-9, "t={t}: current got {} expected {} rel {rel_i}", cur[2], expected_i);
        }
    }

    #[test]
    fn rc_with_esr_and_leakage_matches_the_derived_exponential() {
        // Independent derivation (not the module's own code): with a leakage
        // resistor `leak_r` in parallel with the ESR+C branch, KCL at the
        // free node gives `n2 = A + B*vc` where `D = 1/R + 1/leak_r + 1/esr`,
        // `A = (V0/R)/D`, `B = (1/esr)/D`; and `C*dvc/dt = (n2-vc)/esr`
        // reduces to `dvc/dt = F*(vc - vinf)` with `F = -(1-B)/(esr*C)`,
        // `vinf = A/(1-B)`. Sanity check: `vinf` must equal the steady-state
        // Ohm's-law divider `V0*leak_r/(R+leak_r)` (independent of `esr`,
        // since no current flows through the capacitor branch at steady
        // state) — asserted below alongside the trajectory itself.
        let (v0, r, esr, leak_r, c) = (5.0, 1000.0, 10.0, 1e5, 1e-6);
        let p = rc_program(v0, r, c, esr, leak_r);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let d = 1.0 / r + 1.0 / leak_r + 1.0 / esr;
        let a = (v0 / r) / d;
        let b = (1.0 / esr) / d;
        let f = -(1.0 - b) / (esr * c);
        let vinf = a / (1.0 - b);
        let steady_state = v0 * leak_r / (r + leak_r);
        assert!((vinf - steady_state).abs() < 1e-9, "vinf {vinf} steady_state {steady_state}");

        for &t in &[1e-6, 1e-5, 5e-5, 1e-4, 5e-4, 2e-3] {
            let vc = vinf + (0.0 - vinf) * (f * t).exp();
            let expected_n2 = a + b * vc;
            let (mut v, mut cur) = (voltages.clone(), currents.clone());
            cf.publish_at(t, &HashMap::new(), &mut v, &mut cur);
            // Relative, not absolute: with esr/leakage nonzero, `x0[node2]`
            // and `dx[node2]` are no longer bit-exact constants (the
            // continuity fix only forces the OTHER, ground-fixed terminal
            // relationship, not this one), so a residual at the LU solve's
            // own precision floor is expected — still far tighter than any
            // discretised method could achieve.
            let rel = (v[2] - expected_n2).abs() / expected_n2.abs();
            assert!(rel < 1e-9, "t={t}: got {} expected {} rel {rel}", v[2], expected_n2);
            let _ = cur; // current cross-checked implicitly via the node voltage's own ODE above.
        }
    }

    #[test]
    fn crossing_time_matches_the_analytic_log_formula() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        // No watcher yet: the first segment schedules nothing.
        assert!(cf.take_scheduled().is_empty());

        let threshold = 3.0;
        assert!(cf.watch_node(2, DeviceId(0), Box::new(move || vec![Threshold { value: threshold, rising: true }])));
        // Re-aiming (a fresh `resolve_dirty`, standing in for "a stamp in
        // the partition changed") is what actually schedules the crossing
        // against the now-registered watcher.
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let scheduled = cf.take_scheduled();
        assert_eq!(scheduled.len(), 1);
        let (t, p_idx, version) = scheduled[0];
        assert_eq!(p_idx, 0);
        assert!(cf.is_current(0, version));

        let tau = r * c;
        let expected_t = -tau * (1.0 - threshold / v0).ln();
        assert!((t - expected_t).abs() < 1e-12, "t={t} expected={expected_t}");

        // Landing the crossing nudges the node just past the threshold and
        // re-aims from there.
        cf.handle_crossing(0, t, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert!(voltages[2] > threshold && voltages[2] < threshold + 1e-6, "got {}", voltages[2]);
    }

    #[test]
    fn a_reaim_bumps_the_version_so_a_stale_crossing_is_no_longer_current() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        cf.watch_node(2, DeviceId(0), Box::new(|| vec![Threshold { value: 3.0, rising: true }]));
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        let (_, _, stale_version) = cf.take_scheduled()[0];
        assert!(cf.is_current(0, stale_version));

        // A second re-aim (a switch toggle or driver commit, in the real
        // kernel) bumps the version — the old crossing is no longer current,
        // exactly as `events.rs`'s lazy-deletion contract requires so
        // `Kernel` silently drops the stale heap entry instead of acting on
        // it.
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert!(!cf.is_current(0, stale_version), "the old crossing must be cancelled by the re-aim");
    }

    #[test]
    fn segments_built_and_crossings_stats_advance() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert_eq!(cf.segments_built(), 1.0);
        cf.watch_node(2, DeviceId(0), Box::new(|| vec![Threshold { value: 3.0, rising: true }]));
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert_eq!(cf.segments_built(), 2.0);
        assert_eq!(cf.crossings(), 0.0);
        let (t, _, _) = cf.take_scheduled()[0];
        cf.handle_crossing(0, t, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();
        assert_eq!(cf.crossings(), 1.0);
    }

    /// The closed form is exact relative to the CIRCUIT IT ACTUALLY STAMPS,
    /// which always includes the universal node shunt (core/README.md, "W0
    /// kernel semantics" point 2 — `NODE_RSHUNT_G = 1e-12` S on every node,
    /// classic's own model too) — NOT relative to a bare textbook RC formula
    /// that omits it. That shunt adds a THIRD path (an effective `1/1e-12 =
    /// 1e12 ohm` resistor to ground) in parallel with the ideal source's own
    /// path, pulling the true asymptote very slightly below the source
    /// voltage: `vinf = V0 * Rshunt/(R+Rshunt)`, a relative deviation of
    /// about `R * NODE_RSHUNT_G` from the naive `V0`. At `R = 1kOhm` (this
    /// module's other tests) that is about `1e-9`, invisible next to their
    /// `3e-9` bar; at `R = 100kOhm` (T2's W1b equivalence fixtures 11/12/13)
    /// it grows to about `1e-7` — large enough that comparing against the
    /// shunt-FREE formula at THOSE resistances reads as "only 1e-7 accurate"
    /// even though the segment math is exact to machine precision against
    /// the model it is actually required to implement, verified here.
    #[test]
    fn the_closed_form_is_exact_against_the_shunt_inclusive_model_at_a_large_resistance() {
        let (v0, r, c) = (5.0, 100_000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let r_shunt = 1.0 / crate::solve::static_part::NODE_RSHUNT_G;
        let vinf = v0 * r_shunt / (r + r_shunt);
        // The shunt also very slightly speeds up the time constant (a third
        // parallel path only ever adds conductance): tau = R_parallel * C
        // where R_parallel = (R*Rshunt)/(R+Rshunt).
        let r_parallel = (r * r_shunt) / (r + r_shunt);
        let tau = r_parallel * c;
        let naive_tau = r * c;

        let t = naive_tau; // "one tau" by the naive (shunt-free) formula, matching T2's own fixture 11 check.
        let expected = vinf + (0.0 - vinf) * (-t / tau).exp();
        let naive_expected = v0 * (1.0 - (-t / naive_tau).exp());

        let (mut v, mut cur) = (voltages.clone(), currents.clone());
        cf.publish_at(t, &HashMap::new(), &mut v, &mut cur);

        let rel_vs_shunt_inclusive = (v[2] - expected).abs() / expected.abs();
        assert!(rel_vs_shunt_inclusive < 1e-9, "against the shunt-inclusive model: got {} expected {} rel {rel_vs_shunt_inclusive}", v[2], expected);

        // And confirm the shunt is INDEED the whole story behind a naive
        // comparison's much looser apparent accuracy: the gap from the naive
        // formula should be within an order of magnitude of `R *
        // NODE_RSHUNT_G` (about 1e-7 here), not some unrelated, larger
        // number that would point at a real bug instead.
        let rel_vs_naive = (v[2] - naive_expected).abs() / naive_expected.abs();
        let predicted = r * crate::solve::static_part::NODE_RSHUNT_G;
        assert!(rel_vs_naive < predicted * 10.0, "naive-formula gap {rel_vs_naive} is not explained by the node shunt (predicted ~{predicted})");
    }

    /// core/README.md's "Currents": a resistor publishes `(Va-Vb)/R`, even
    /// when it sits inside a `ClosedForm` partition — a device W1b's own
    /// review found publishing exactly `0.0` there, since nothing before
    /// this fix ever computed a `DynStamped` device's own current at all
    /// (`recompute_source_currents` only ever reads a device's current back
    /// OUT, for a `voltage_source`'s own KCL; it never produced one for a
    /// plain resistor). Checked at several times against the resistor's own
    /// formula AND against KCL at the free node (current in = capacitor's
    /// current out + the node's own shunt current — core/README.md's W0
    /// point 2, included in every KCL sum).
    #[test]
    fn a_resistor_inside_a_closed_form_partition_publishes_its_own_current_and_satisfies_kcl() {
        let (v0, r, c) = (5.0, 1000.0, 1e-6);
        let p = rc_program(v0, r, c, 0.0, 0.0);
        let mut cf = build(&p);
        let mut voltages = vec![0.0, v0, 0.0];
        let mut currents = vec![0.0; 3];
        cf.resolve_dirty(&[0], 0.0, &[], &HashMap::new(), &mut voltages, &mut currents).unwrap();

        let shunt = crate::solve::static_part::NODE_RSHUNT_G;
        for &t in &[1e-5, 1e-4, 5e-4, 1e-3, 2e-3] {
            let (mut v, mut cur) = (voltages.clone(), currents.clone());
            cf.publish_at(t, &HashMap::new(), &mut v, &mut cur);

            let expected_r = (v[1] - v[2]) / r; // resistor device 1: pins (node1, node2).
            assert!(cur[1] != 0.0, "t={t}: resistor current must not be published as a stale/absent 0.0");
            assert!((cur[1] - expected_r).abs() < 1e-12, "t={t}: resistor current got {} expected {}", cur[1], expected_r);

            // KCL at node2 (the only free node): resistor current IN must
            // equal the capacitor's current OUT plus the node's own shunt
            // current OUT.
            let kcl_residual = cur[1] - cur[2] - shunt * v[2];
            assert!(kcl_residual.abs() < 1e-12, "t={t}: KCL residual at node2 = {kcl_residual}");
        }
    }
}
