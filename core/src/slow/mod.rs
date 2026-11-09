//! Slow state and its journal (G1): core/README.md, "Kernel semantics: thermal, failures and warnings (W1g)", points 1, 2 and 15.

pub mod failures;
pub mod stress;
pub mod thermal;
pub mod warnings;

use crate::digital::sag::{SagCell, SagState};
use stress::{PtcEntry, PtcEvent, PtcParams, Stress};
use thermal::{ThermalConfig, ThermalEntry};

/// A slow event whose time is within this of `now` is latched at once, inside
/// the current instant (point 5): the hard-won crossing rule's margin, so a
/// device never reschedules itself onto a time rounding left it one ulp short
/// of.
pub const SLOW_EPS: f64 = 1e-15;

/// Whether a slow event at `t_event` is due at `now` (point 14, step 3).
pub fn is_due(t_event: f64, now: f64) -> bool {
    t_event <= now + SLOW_EPS
}

/// A node voltage that relaxes exponentially over a closed-form segment:
/// `V(t) = a + b*exp(-(t - t_seg)/tau)`. The caller builds one per device from
/// `closed_form::node_trajectory` (`(v_inf, v0, t0, tau)` per node): a pin on a
/// fixed node subtracts that voltage, two pins in the partition subtract their
/// trajectories. A partition is one capacitor, so every node of a segment
/// shares `tau` and `t_seg`.
///
/// **A ramp** (`ramp`, ruling R30's exact path on a CC rail capacitor): the
/// rail climbs toward an asymptote `u0 + slope/k` that, for a slow segment
/// (a large resistance across the rail), lies far past where CV ends it, so
/// `a` and `b` are huge and nearly cancel (`il·R` volts for a 5 V rail). Its
/// value at `t_seg` is then kept exactly in `v0`, and every closed form reads
/// it as `V = v0 + b·expm1(-s/tau)`, whose terms stay the size of `V` and of
/// its slope `b/tau`: no cancellation at any `tau` (W1gV3). Any other
/// voltage is a physical relaxation whose `a` and `b` are the size of `V`,
/// and keeps the `a + b·e^(-s/tau)` forms bit for bit (`v0` is `a + b`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExpVoltage {
    pub a: f64,
    pub b: f64,
    pub tau: f64,
    pub t_seg: f64,
    /// `V(t_seg)`: exact for a ramp, `a + b` otherwise.
    pub v0: f64,
    pub ramp: bool,
}

impl ExpVoltage {
    pub fn new(a: f64, b: f64, tau: f64, t_seg: f64) -> Self {
        ExpVoltage { a, b, tau, t_seg, v0: a + b, ramp: false }
    }

    /// From a node trajectory `(v_inf, v0, t0, tau)`.
    pub fn from_trajectory(v_inf: f64, v0: f64, t_seg: f64, tau: f64) -> Self {
        ExpVoltage { a: v_inf, b: v0 - v_inf, tau, t_seg, v0, ramp: false }
    }

    /// A ramp (see the type): `v0` at `t_seg`, moving by `b·expm1(-s/tau)`,
    /// so `b/tau` is its slope there.
    pub fn ramp(v0: f64, b: f64, tau: f64, t_seg: f64) -> Self {
        ExpVoltage { a: v0 - b, b, tau, t_seg, v0, ramp: true }
    }

    /// `V` at `s` past `t_seg`: `a + b·e^(-s/tau)`, or a ramp's `v0 +
    /// b·expm1(-s/tau)`.
    pub fn at_offset(&self, s: f64) -> f64 {
        if self.ramp {
            self.v0 + self.b * (-s / self.tau).exp_m1()
        } else {
            self.a + self.b * (-s / self.tau).exp()
        }
    }

    /// The one value this voltage holds when it does not relax: `b == 0` (nothing
    /// to relax), an infinite `tau` (`exp(-s/tau)` stays 1, so `V = a + b`
    /// for good) or a zero `tau` (relaxed before the first instant, `V = a`).
    /// `None` otherwise. Every closed form divides by `tau` or by differences
    /// of `tau`s, so these inputs would give NaN (`b == 0` with `tau == INF` is
    /// `INF/INF`); callers collapse them to the constant drive instead.
    pub fn constant_value(&self) -> Option<f64> {
        if self.b == 0.0 {
            Some(self.a)
        } else if self.tau == f64::INFINITY {
            Some(if self.ramp { self.v0 } else { self.a + self.b })
        } else if self.tau == 0.0 {
            Some(self.a)
        } else {
            None
        }
    }

    /// `V(t)`.
    pub fn at(&self, t: f64) -> f64 {
        match self.constant_value() {
            Some(v) => v,
            None => self.at_offset(t - self.t_seg),
        }
    }

    /// `V - dv`: one pin on a fixed node.
    pub fn offset(self, dv: f64) -> ExpVoltage {
        ExpVoltage { a: self.a - dv, v0: self.v0 - dv, ..self }
    }

    /// `dv - V`: a fixed node minus a pin in the segment.
    pub fn from_fixed(self, dv: f64) -> ExpVoltage {
        ExpVoltage { a: dv - self.a, b: -self.b, v0: dv - self.v0, ..self }
    }

    /// `self - other`: two pins in the same segment.
    pub fn minus(self, other: ExpVoltage) -> ExpVoltage {
        debug_assert!(self.tau == other.tau && self.t_seg == other.t_seg, "one segment has one tau and one start");
        ExpVoltage { a: self.a - other.a, b: self.b - other.b, v0: self.v0 - other.v0, ramp: self.ramp || other.ramp, ..self }
    }

    /// The same function expressed from a later origin `t`, so a window that
    /// starts mid-segment reads `a + b'*exp(-u/tau)` with `u` counted from it.
    pub fn rebased(&self, t: f64) -> ExpVoltage {
        match self.constant_value() {
            Some(v) => ExpVoltage { a: v, b: 0.0, tau: self.tau, t_seg: t, v0: v, ramp: false },
            None => {
                let s = t - self.t_seg;
                let b = self.b * (-s / self.tau).exp();
                ExpVoltage { b, t_seg: t, v0: if self.ramp { self.at_offset(s) } else { self.a + b }, ..*self }
            }
        }
    }
}

/// A table whose writes are journaled per instant (point 15): between
/// [`Self::begin`] and [`Self::end`] or [`Self::rollback`], the first write to
/// an entry saves its pre-image, once, so a completed instant pays for the
/// entries it touched and an instant that touched none pays for nothing. The
/// pattern is `DeviceEvaluator::begin_instant`'s, with `IcEvaluator`'s journal
/// as its worked example.
///
/// Every write goes through [`Self::get_mut`], which saves before it hands out
/// the reference, so no mutator can forget the journal. The entry type is
/// generic over `Clone`, so G2b can hold G2a's `output_sag` table in the same
/// mechanism.
#[derive(Clone, Debug)]
pub struct JournaledTable<E: Clone> {
    /// Shared with snapshots until a write (W2a): a table no instant
    /// touches between two snapshots is not copied.
    entries: crate::shared::Shared<[E]>,
    open: bool,
    /// Per entry: the open window already holds its pre-image.
    marked: Vec<bool>,
    saved: Vec<(usize, E)>,
    /// Pre-image records from finished windows, kept with their storage so the
    /// next window's saves allocate nothing for entries that own heap data.
    spare: Vec<(usize, E)>,
}

impl<E: Clone> JournaledTable<E> {
    pub fn new(entries: Vec<E>) -> Self {
        let marked = vec![false; entries.len()];
        JournaledTable { entries: entries.into(), open: false, marked, saved: Vec::new(), spare: Vec::new() }
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn get(&self, at: usize) -> &E {
        &self.entries[at]
    }

    pub fn as_slice(&self) -> &[E] {
        &self.entries
    }

    /// A write handle: saves the entry's pre-image first if a window is open
    /// and has not saved it yet. Outside a window (load) it saves nothing.
    pub fn get_mut(&mut self, at: usize) -> &mut E {
        if self.open && !self.marked[at] {
            self.marked[at] = true;
            let entry = &self.entries[at];
            match self.spare.pop() {
                Some((_, mut image)) => {
                    image.clone_from(entry);
                    self.saved.push((at, image));
                }
                None => self.saved.push((at, entry.clone())),
            }
        }
        &mut self.entries[at]
    }

    /// Opens the window. Windows do not nest.
    pub fn begin(&mut self) {
        assert!(!self.open && self.saved.is_empty(), "rollback windows do not nest");
        self.open = true;
    }

    /// The instant completed: keep every write, return the pre-images to the
    /// pool.
    pub fn end(&mut self) {
        for (at, image) in self.saved.drain(..) {
            self.marked[at] = false;
            self.spare.push((at, image));
        }
        self.open = false;
    }

    /// The instant failed: every touched entry goes back to its pre-image, and
    /// the entries it never wrote are untouched, which is what makes this equal
    /// to restoring a whole-table copy.
    pub fn rollback(&mut self) {
        for (at, mut image) in self.saved.drain(..) {
            std::mem::swap(&mut self.entries[at], &mut image);
            self.marked[at] = false;
            self.spare.push((at, image));
        }
        self.open = false;
    }

    /// The number of pre-images the open window holds: what the instant has
    /// paid so far.
    pub fn journaled(&self) -> usize {
        self.saved.len()
    }
}

/// A failure key as `bo_engine_records` names it, as plain integers: the
/// device record index, the failure code (1 `resistor_overload`, 2
/// `fuse_tripped`, 3 `led_failed`, 4 `output_sag`) and the pin index, or `-1`
/// for none.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LatchKey {
    pub device: u32,
    pub code: u32,
    pub pin: i32,
}

/// One latched failure, the 7 values of a `bo_engine_records` record.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatchRecord {
    pub device: u32,
    pub code: u32,
    pub pin: i32,
    pub since: f64,
    pub value: f64,
    pub limit: f64,
    pub aux: f64,
}

impl LatchRecord {
    pub fn key(&self) -> LatchKey {
        LatchKey { device: self.device, code: self.code, pin: self.pin }
    }
}

/// The latch bookkeeping: the latch order (the order classic's `failures` map
/// iterates) and the latch log (`bo_engine_records` channel 3).
#[derive(Debug, Default, PartialEq)]
struct LatchBook {
    /// Currently latched keys, in latch order.
    order: Vec<LatchRecord>,
    /// The first latch of each key since the last take, in first-latch order.
    log: Vec<LatchRecord>,
}

impl Clone for LatchBook {
    fn clone(&self) -> Self {
        LatchBook { order: self.order.clone(), log: self.log.clone() }
    }

    /// In place, so a recycled pre-image keeps its buffers: the derived
    /// `clone_from` is `*self = source.clone()`, which allocates on every save.
    fn clone_from(&mut self, source: &Self) {
        self.order.clone_from(&source.order);
        self.log.clone_from(&source.log);
    }
}

/// What changes at a device at an instant but never leaves the slow entry:
/// the pending-event token, the failed flag every stamp path reads, and the
/// junction law generation the solution memo keys on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct DeviceFlags {
    token: u32,
    law_gen: u32,
    failed: bool,
}

/// What a device's pending slow event is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlowEventKind {
    StressLatch,
    ThermalBoundary,
    ShutdownTrip,
    ShutdownRestart,
    PtcTrip,
    PtcRecover,
}

/// The devices a reset touched (point 13).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ResetEffects {
    /// Devices whose stamp returns (an open cleared, a tripped PTC back to its
    /// normal resistance): the caller runs `mark_device_changed` for each.
    pub restamped: Vec<usize>,
    /// Devices whose token was bumped, so their pending slow event is stale:
    /// the caller asks `next_event` and schedules afresh.
    pub rescheduled: Vec<usize>,
}

/// All slow state (point 1): the lazy integrators, the latch flags and order,
/// the per-device tokens. Pure state: no solver types, no kernel calls. Every
/// table is journaled per instant (point 15), and reads never touch the
/// journal, so a device that sits at rate 0 costs a completed instant nothing.
///
/// Tables are indexed by the program's device index, with `None` for a device
/// that has no such integrator.
#[derive(Clone, Debug)]
pub struct SlowState {
    flags: JournaledTable<DeviceFlags>,
    stress: JournaledTable<Option<Stress>>,
    thermal: JournaledTable<Option<ThermalEntry>>,
    ptc: JournaledTable<Option<PtcEntry>>,
    /// One entry: the latch order and log are small and change rarely, so the
    /// whole book is the journal's unit.
    book: JournaledTable<LatchBook>,
    /// G2a's `output_sag` window state, one cell per `(IC, output-capable
    /// pin)` (`digital::sag`, point 9), under the same journal: a window
    /// opened, latched or cleared by a failed instant rolls back with it.
    sag: JournaledTable<SagCell>,
    /// Whether the open window bumped any `law_gen`. Not journaled: it is the
    /// answer `rollback_instant` gives, and each window starts it afresh.
    law_gen_bumped: bool,
    /// Whether any failed flag has been raised since load. Never lowered (not
    /// by a reset, not by a rollback), so `false` proves no device is failed
    /// and the stamp paths that check every junction can skip the check.
    ever_failed: bool,
}

impl SlowState {
    /// A load starts every slow device fresh (point 15, W1g Q8): no stress, no
    /// latch, every PTC untripped, an empty latch log. The `init_*` calls then
    /// register the devices that have each integrator.
    pub fn new(device_count: usize) -> Self {
        SlowState {
            flags: JournaledTable::new(vec![DeviceFlags::default(); device_count]),
            stress: JournaledTable::new(vec![None; device_count]),
            thermal: JournaledTable::new(vec![None; device_count]),
            ptc: JournaledTable::new(vec![None; device_count]),
            book: JournaledTable::new(vec![LatchBook::default()]),
            sag: JournaledTable::new(Vec::new()),
            law_gen_bumped: false,
            ever_failed: false,
        }
    }

    /// Registers the `output_sag` cells, all `Ok` (at load, so nothing
    /// journals). Cell `i` keeps its index for the life of the program.
    pub fn init_sag(&mut self, cells: Vec<SagCell>) {
        self.sag = JournaledTable::new(cells);
    }

    pub fn sag_len(&self) -> usize {
        self.sag.len()
    }

    pub fn sag_cell(&self, i: usize) -> &SagCell {
        self.sag.get(i)
    }

    /// A cell's window state, journaled before the write.
    pub fn sag_state_mut(&mut self, i: usize) -> &mut SagState {
        &mut self.sag.get_mut(i).state
    }

    pub fn device_count(&self) -> usize {
        self.flags.len()
    }

    // ---- registration, at load ----------------------------------------

    /// Gives `dev` a stress integrator with the kind's latch threshold.
    pub fn init_stress(&mut self, dev: usize, threshold: f64, t0: f64) {
        *self.stress.get_mut(dev) = Some(Stress::new(threshold, t0));
    }

    /// Gives `dev` a thermal entry at `T = T_hold = Ta`.
    pub fn init_thermal(&mut self, dev: usize, cfg: ThermalConfig, t0: f64) {
        *self.thermal.get_mut(dev) = Some(ThermalEntry::new(cfg, t0));
    }

    /// Makes `dev` a PTC, untripped.
    pub fn init_ptc(&mut self, dev: usize, params: PtcParams, t0: f64) {
        *self.ptc.get_mut(dev) = Some(PtcEntry::new(params, t0));
    }

    // ---- reads (never journal) -----------------------------------------

    pub fn stress(&self, dev: usize) -> Option<&Stress> {
        self.stress.get(dev).as_ref()
    }

    pub fn thermal(&self, dev: usize) -> Option<&ThermalEntry> {
        self.thermal.get(dev).as_ref()
    }

    pub fn ptc(&self, dev: usize) -> Option<&PtcEntry> {
        self.ptc.get(dev).as_ref()
    }

    /// The failed flag every stamp path and the KCL walk read.
    pub fn is_failed(&self, dev: usize) -> bool {
        self.flags.get(dev).failed
    }

    /// Whether any device may be failed: `false` once proves none is
    /// ([`Self::is_failed`] is `false` for every device).
    pub fn may_have_failed(&self) -> bool {
        self.ever_failed
    }

    /// The device's PTC resistance accessor: `None` for a device that is not a
    /// PTC, else `rNormal` or `rNormal * 200` by its trip state.
    pub fn ptc_resistance(&self, dev: usize) -> Option<f64> {
        self.ptc(dev).map(|p| p.params.resistance(p.tripped))
    }

    pub fn law_gen(&self, dev: usize) -> u32 {
        self.flags.get(dev).law_gen
    }

    pub fn token(&self, dev: usize) -> u32 {
        self.flags.get(dev).token
    }

    /// Whether a popped `Slow` event still names the device's current drive.
    pub fn is_current(&self, dev: usize, token: u32) -> bool {
        self.flags.get(dev).token == token
    }

    // ---- writes (every one journals through `get_mut`) -----------------

    /// Re-drive: bumps the token so the pending event goes stale, and returns
    /// the token the fresh one carries.
    pub fn bump_token(&mut self, dev: usize) -> u32 {
        let flags = self.flags.get_mut(dev);
        flags.token = flags.token.wrapping_add(1);
        flags.token
    }

    /// Re-drive in one call: the pending event goes stale and the fresh one, if
    /// any, comes back with the token it must carry. A caller that finds the
    /// new drive equal to the old one leaves the device alone instead, so a
    /// quiet device is never written (and never journaled).
    pub fn reschedule(&mut self, dev: usize) -> Option<(f64, SlowEventKind, u32)> {
        let token = self.bump_token(dev);
        self.next_event(dev).map(|(t, kind)| (t, kind, token))
    }

    /// A junction's law changed (re-arm, latch or reset): the memo's key moves.
    pub fn bump_law_gen(&mut self, dev: usize) -> u32 {
        self.law_gen_bumped = true;
        let flags = self.flags.get_mut(dev);
        flags.law_gen = flags.law_gen.wrapping_add(1);
        flags.law_gen
    }

    pub fn stress_mut(&mut self, dev: usize) -> Option<&mut Stress> {
        self.stress.get(dev).as_ref()?;
        self.stress.get_mut(dev).as_mut()
    }

    pub fn thermal_mut(&mut self, dev: usize) -> Option<&mut ThermalEntry> {
        self.thermal.get(dev).as_ref()?;
        self.thermal.get_mut(dev).as_mut()
    }

    pub fn ptc_mut(&mut self, dev: usize) -> Option<&mut PtcEntry> {
        self.ptc.get(dev).as_ref()?;
        self.ptc.get_mut(dev).as_mut()
    }

    // ---- latch bookkeeping ---------------------------------------------

    /// Latches a failure. A key already latched keeps its place and its record
    /// (classic: `if failures.has(key) return`, SE:5518, and a latched
    /// `output_sag` re-`set`s in place, SE:6325); a new one goes to the end of
    /// the order and, if it is the first of its key since the last take, into
    /// the log. `opens_device` raises the device's failed flag, for the kinds
    /// whose latch changes a stamp (`output_sag` does not). Returns whether the
    /// key was newly latched.
    pub fn latch(&mut self, rec: LatchRecord, opens_device: bool) -> bool {
        let key = rec.key();
        if self.book.get(0).order.iter().any(|r| r.key() == key) {
            return false;
        }
        let book = self.book.get_mut(0);
        book.order.push(rec);
        if !book.log.iter().any(|r| r.key() == key) {
            book.log.push(rec);
        }
        if opens_device && !self.flags.get(rec.device as usize).failed {
            self.flags.get_mut(rec.device as usize).failed = true;
            self.ever_failed = true;
        }
        true
    }

    /// Clears one latched key, for a kind that clears on its own (`output_sag`,
    /// or an unpublished direction flip). A later re-latch goes to the end of
    /// the order. The failed flag is left alone: an opening latch clears only
    /// with `reset_failures`.
    pub fn clear_latch(&mut self, key: LatchKey) -> bool {
        if !self.book.get(0).order.iter().any(|r| r.key() == key) {
            return false;
        }
        self.book.get_mut(0).order.retain(|r| r.key() != key);
        true
    }

    pub fn is_latched(&self, key: LatchKey) -> bool {
        self.book.get(0).order.iter().any(|r| r.key() == key)
    }

    /// The latched failures in latch order (`bo_engine_records` channel 0; an
    /// `output_sag` entry is re-projected at collect time).
    pub fn latched(&self) -> &[LatchRecord] {
        &self.book.get(0).order
    }

    /// The latch log without taking it.
    pub fn latch_log(&self) -> &[LatchRecord] {
        &self.book.get(0).log
    }

    /// Channel 3: the first latch of every key since the last take, in order.
    /// Taking empties it. A collect is not an instant, so nothing journals.
    pub fn take_latch_log(&mut self) -> Vec<LatchRecord> {
        if self.book.get(0).log.is_empty() {
            return Vec::new();
        }
        std::mem::take(&mut self.book.get_mut(0).log)
    }

    // ---- the next event ------------------------------------------------

    /// The earliest latch this device's integrators will fire under their
    /// current drives: its stress latch or its PTC's trip (ruling R71 (b): a
    /// stepped device schedules only these, from the drive its last step
    /// ended with, so it latches inside the step that follows).
    pub fn next_latch(&self, dev: usize) -> Option<(f64, SlowEventKind)> {
        let stress = self.stress(dev).filter(|_| !self.is_failed(dev)).and_then(|s| s.latch_time()).map(|t| (t, SlowEventKind::StressLatch));
        let trip = self.ptc(dev).and_then(|p| p.next_event()).filter(|&(_, e)| e == PtcEvent::Trip).map(|(t, _)| (t, SlowEventKind::PtcTrip));
        match (stress, trip) {
            (Some(a), Some(b)) => Some(if b.0 < a.0 { b } else { a }),
            (a, b) => a.or(b),
        }
    }

    /// The earliest event this device's integrators will fire under their
    /// current drives, with the earliest kind first on a tie. One pending event
    /// per device carries its token; a publish re-drives (`bump_token`) and
    /// asks again. A latched device's stress does nothing (point 5).
    pub fn next_event(&self, dev: usize) -> Option<(f64, SlowEventKind)> {
        let mut best: Option<(f64, SlowEventKind)> = None;
        let mut offer = |t: Option<f64>, kind: SlowEventKind| {
            if let Some(t) = t {
                if best.is_none_or(|(b, _)| t < b) {
                    best = Some((t, kind));
                }
            }
        };
        if !self.is_failed(dev) {
            if let Some(s) = self.stress(dev) {
                offer(s.latch_time(), SlowEventKind::StressLatch);
            }
        }
        if let Some(th) = self.thermal(dev) {
            // A failed device stamps nothing, so its law no longer matters and a
            // cooling LED would otherwise fire a re-arm per 0.01 C of its fall.
            // The first boundary after a reset is then due at once, since `T`
            // has left the band around the frozen `T_hold`.
            if !self.is_failed(dev) {
                offer(th.boundary_time(), SlowEventKind::ThermalBoundary);
            }
            let kind = if th.shutdown { SlowEventKind::ShutdownRestart } else { SlowEventKind::ShutdownTrip };
            offer(th.shutdown_time(), kind);
        }
        if let Some(p) = self.ptc(dev) {
            if let Some((t, event)) = p.next_event() {
                offer(Some(t), if event == PtcEvent::Trip { SlowEventKind::PtcTrip } else { SlowEventKind::PtcRecover });
            }
        }
        best
    }

    // ---- resetFailures --------------------------------------------------

    /// `resetFailures` (point 13): clears every latch, every stress integrator
    /// and every PTC, and leaves the thermal entries (`T`, `T_hold`, the
    /// shutdown flag) and the latch log alone. An integrator that sat at 0
    /// under a drive that cannot accumulate is not touched, so a reset does not
    /// journal every quiet device. The drives are kept: the voltages have not
    /// moved, and the delta cycle at the reset instant publishes any that did.
    pub fn reset_failures(&mut self, now: f64) -> ResetEffects {
        let mut effects = ResetEffects::default();
        if !self.book.get(0).order.is_empty() {
            self.book.get_mut(0).order.clear();
        }
        for dev in 0..self.device_count() {
            let mut stale = false;
            if self.flags.get(dev).failed {
                self.flags.get_mut(dev).failed = false;
                effects.restamped.push(dev);
                stale = true;
            }
            if let Some(s) = self.stress(dev) {
                if s.s0 != 0.0 || s.drive.accumulates() {
                    self.stress_mut(dev).expect("checked above").reset(now);
                    stale = true;
                }
            }
            if let Some(p) = self.ptc(dev) {
                let was_tripped = p.tripped;
                let idle = matches!(p.drive, stress::PtcDrive::Stress(d) if !d.accumulates());
                if was_tripped || p.stress != 0.0 || p.rt != 0.0 || !idle {
                    self.ptc_mut(dev).expect("checked above").reset(now);
                    if was_tripped {
                        effects.restamped.push(dev);
                    }
                    stale = true;
                }
            }
            if stale {
                self.bump_token(dev);
                effects.rescheduled.push(dev);
            }
        }
        effects
    }

    // ---- the instant ----------------------------------------------------

    /// Opens the rollback window (`begin_instant` on `PartitionSolver`).
    pub fn begin_instant(&mut self) {
        self.law_gen_bumped = false;
        self.flags.begin();
        self.stress.begin();
        self.thermal.begin();
        self.ptc.begin();
        self.book.begin();
        self.sag.begin();
    }

    /// The instant completed.
    pub fn end_instant(&mut self) {
        self.law_gen_bumped = false;
        self.flags.end();
        self.stress.end();
        self.thermal.end();
        self.ptc.end();
        self.book.end();
        self.sag.end();
    }

    /// The instant failed: every table back to the bit pattern `begin_instant`
    /// found. Returns whether the window bumped any `law_gen`: a generation
    /// number rolls back with its instant and can recur for a different law, so
    /// the caller clears the junction memo then (README point 4).
    pub fn rollback_instant(&mut self) -> bool {
        self.flags.rollback();
        self.stress.rollback();
        self.thermal.rollback();
        self.ptc.rollback();
        self.book.rollback();
        self.sag.rollback();
        std::mem::take(&mut self.law_gen_bumped)
    }

    /// Pre-images held by the open window across every table: what the instant
    /// has paid. Zero for an instant that touched no slow entry.
    pub fn journaled(&self) -> usize {
        self.flags.journaled() + self.stress.journaled() + self.thermal.journaled() + self.ptc.journaled() + self.book.journaled() + self.sag.journaled()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use stress::{PtcDrive, StressDrive};
    use thermal::PowerDrive;

    fn cfg() -> ThermalConfig {
        ThermalConfig { ambient_c: 25.0, r_theta: 560.0, tau_s: 2.0, shutdown: None, holds_junction: true }
    }

    fn rec(device: u32, code: u32, pin: i32, since: f64) -> LatchRecord {
        LatchRecord { device, code, pin, since, value: since * 2.0, limit: 3.0, aux: 0.0 }
    }

    /// A state with every table populated: device 0 a stressed resistor, 1 a
    /// thermal LED with stress, 2 a PTC, 3 a quiet resistor, 4 nothing.
    fn populated() -> SlowState {
        let mut s = SlowState::new(5);
        s.init_stress(0, 1.0, 0.0);
        s.stress_mut(0).unwrap().drive = StressDrive::Constant { rate: 1.0, recovery: 1.0, zero: 1.0, q: -1.0 };
        s.init_stress(1, 24.0, 0.0);
        s.init_thermal(1, cfg(), 0.0);
        s.init_ptc(2, PtcParams { r_normal: 0.5, i_hold: 0.5 }, 0.0);
        s.init_stress(3, 1.0, 0.0);
        s
    }

    /// Every field of every table as bits, so two states compare bit for bit.
    fn fingerprint(s: &SlowState) -> Vec<u64> {
        fn drive(out: &mut Vec<u64>, d: &StressDrive) {
            match d {
                StressDrive::Constant { rate, recovery, zero, .. } => out.extend([1, rate.to_bits(), recovery.to_bits(), zero.to_bits()]),
                StressDrive::Segment(g) => out.extend([2, g.k.to_bits(), g.c.to_bits(), g.recovery.to_bits(), g.v.a.to_bits(), g.v.b.to_bits(), g.v.tau.to_bits(), g.v.t_seg.to_bits()]),
            }
        }
        fn record(out: &mut Vec<u64>, r: &LatchRecord) {
            out.extend([u64::from(r.device), u64::from(r.code), r.pin as u32 as u64, r.since.to_bits(), r.value.to_bits(), r.limit.to_bits(), r.aux.to_bits()]);
        }
        let mut out = Vec::new();
        for dev in 0..s.device_count() {
            let f = s.flags.get(dev);
            out.extend([u64::from(f.token), u64::from(f.law_gen), u64::from(f.failed)]);
            match s.stress(dev) {
                None => out.push(0),
                Some(x) => {
                    out.extend([1, x.s0.to_bits(), x.t0.to_bits(), x.threshold.to_bits()]);
                    drive(&mut out, &x.drive);
                }
            }
            match s.thermal(dev) {
                None => out.push(0),
                Some(t) => {
                    out.extend([1, t.temp0.to_bits(), t.t0.to_bits(), t.t_hold.to_bits(), u64::from(t.shutdown)]);
                    match t.drive {
                        PowerDrive::Constant(p) => out.extend([1, p.to_bits()]),
                        PowerDrive::Squared { gain, v } => out.extend([2, gain.to_bits(), v.a.to_bits(), v.b.to_bits(), v.tau.to_bits(), v.t_seg.to_bits()]),
                    }
                }
            }
            match s.ptc(dev) {
                None => out.push(0),
                Some(p) => {
                    out.extend([1, u64::from(p.tripped), p.stress.to_bits(), p.rt.to_bits(), p.t0.to_bits()]);
                    match p.drive {
                        PtcDrive::Stress(d) => {
                            out.push(1);
                            drive(&mut out, &d);
                        }
                        PtcDrive::Cool { from, until } => out.extend([2, from.to_bits(), until.to_bits()]),
                    }
                }
            }
        }
        let book = s.book.get(0);
        out.push(book.order.len() as u64);
        book.order.iter().for_each(|r| record(&mut out, r));
        out.push(book.log.len() as u64);
        book.log.iter().for_each(|r| record(&mut out, r));
        out
    }

    /// Runs `write` inside an open window and checks it moved the state,
    /// then that a rollback restores every bit and a second run's `end` keeps
    /// the write.
    fn assert_rolls_back(label: &str, write: impl Fn(&mut SlowState)) {
        let mut s = populated();
        s.latch(rec(0, 1, -1, 0.5), true);
        let before = fingerprint(&s);
        s.begin_instant();
        write(&mut s);
        let during = fingerprint(&s);
        assert_ne!(before, during, "{label}: the write must change something for the test to mean anything");
        assert!(s.journaled() > 0, "{label}: a write journals its entry");
        s.rollback_instant();
        assert_eq!(fingerprint(&s), before, "{label}: rollback is bit-identical");
        assert_eq!(s.journaled(), 0);
        // The same write kept by `end_instant`.
        s.begin_instant();
        write(&mut s);
        s.end_instant();
        assert_eq!(fingerprint(&s), during, "{label}: end_instant keeps the write");
        assert_eq!(s.journaled(), 0);
    }

    #[test]
    fn rollback_restores_the_token() {
        assert_rolls_back("token", |s| {
            s.bump_token(0);
            s.bump_token(0);
        });
    }

    #[test]
    fn rollback_restores_the_failed_flag() {
        // `latch` with opens_device raises it, and `reset_failures` lowers it.
        assert_rolls_back("failed flag raised", |s| {
            s.latch(rec(1, 3, -1, 0.7), true);
        });
        let mut s = populated();
        s.latch(rec(0, 1, -1, 0.5), true);
        assert!(s.is_failed(0));
        let before = fingerprint(&s);
        s.begin_instant();
        s.reset_failures(1.0);
        assert!(!s.is_failed(0));
        s.rollback_instant();
        assert!(s.is_failed(0));
        assert_eq!(fingerprint(&s), before);
    }

    #[test]
    fn rollback_restores_the_law_generation() {
        assert_rolls_back("law_gen", |s| {
            s.bump_law_gen(1);
        });
    }

    #[test]
    fn rollback_restores_the_stress_integrator() {
        assert_rolls_back("stress", |s| {
            let x = s.stress_mut(0).unwrap();
            x.close(0.4);
            x.drive = StressDrive::Constant { rate: 7.0, recovery: 0.5, zero: 1.0, q: -1.0 };
        });
    }

    #[test]
    fn rollback_restores_the_thermal_entry() {
        assert_rolls_back("thermal", |s| {
            let t = s.thermal_mut(1).unwrap();
            t.drive = PowerDrive::Constant(0.038);
            t.rearm(1.5);
            t.shutdown = true;
        });
    }

    #[test]
    fn rollback_restores_the_ptc_entry() {
        assert_rolls_back("ptc", |s| {
            let p = s.ptc_mut(2).unwrap();
            p.set_drive(PtcDrive::Stress(StressDrive::Constant { rate: 0.75, recovery: 0.0, zero: 1.0, q: -1.0 }));
            p.close(0.05);
            p.trip(0.1);
            p.set_drive(PtcDrive::Cool { from: 0.1, until: f64::INFINITY });
        });
    }

    #[test]
    fn rollback_restores_the_latch_order() {
        // A new key appended, and, separately, a key cleared: each alone, so
        // that the first write's pre-image cannot cover for the second.
        assert_rolls_back("latch order, appended", |s| {
            s.latch(rec(1, 3, -1, 0.9), true);
        });
        assert_rolls_back("latch order, cleared", |s| {
            s.clear_latch(LatchKey { device: 0, code: 1, pin: -1 });
        });
    }

    #[test]
    fn rollback_restores_the_latch_log() {
        // A first latch of a new key enters both the order and the log; the
        // comparison covers both halves of the book.
        assert_rolls_back("latch log", |s| {
            s.latch(rec(3, 2, -1, 0.9), false);
        });
    }

    #[test]
    fn rollback_restores_a_reset() {
        // `reset_failures` writes every table at once (flags, order, stress,
        // PTC), and a failed reset re-solve must put all of it back (W1g Q6).
        let mut s = populated();
        s.latch(rec(0, 1, -1, 0.5), true);
        s.stress_mut(0).unwrap().close(0.4);
        s.ptc_mut(2).unwrap().trip(0.3);
        s.ptc_mut(2).unwrap().set_drive(PtcDrive::Cool { from: 0.3, until: f64::INFINITY });
        let before = fingerprint(&s);
        s.begin_instant();
        s.reset_failures(2.0);
        assert_ne!(fingerprint(&s), before);
        s.rollback_instant();
        assert_eq!(fingerprint(&s), before);
    }

    #[test]
    fn one_instant_rolls_back_a_mix_of_everything_bit_for_bit() {
        let mut s = populated();
        s.latch(rec(0, 1, -1, 0.5), true);
        s.latch(rec(4, 4, 1, 0.6), false);
        let before = fingerprint(&s);
        s.begin_instant();
        s.bump_token(0);
        s.bump_law_gen(1);
        s.stress_mut(1).unwrap().close(2.0);
        s.thermal_mut(1).unwrap().drive = PowerDrive::Constant(0.038);
        s.thermal_mut(1).unwrap().rearm(3.0);
        s.ptc_mut(2).unwrap().trip(3.0);
        s.latch(rec(1, 3, -1, 3.0), true);
        s.clear_latch(LatchKey { device: 4, code: 4, pin: 1 });
        s.latch(rec(4, 4, 1, 3.5), false);
        assert_ne!(fingerprint(&s), before);
        s.rollback_instant();
        assert_eq!(fingerprint(&s), before);
        // And the next window works from a clean journal.
        s.begin_instant();
        s.bump_token(0);
        s.end_instant();
        assert_eq!(s.token(0), 1);
    }

    #[test]
    fn a_completed_instant_that_touched_nothing_pays_nothing() {
        let mut s = populated();
        s.latch(rec(0, 1, -1, 0.5), true);
        s.begin_instant();
        // Reads, projections and failed lookups write nothing.
        let _ = (s.stress(0), s.thermal(1), s.ptc(2), s.is_failed(0), s.token(0), s.law_gen(1), s.next_event(0), s.latched(), s.latch_log());
        let _ = s.stress(0).unwrap().value_at(2.0);
        let _ = s.thermal(1).unwrap().temperature_at(2.0);
        let _ = s.ptc(2).unwrap().projected(2.0);
        assert!(s.stress_mut(4).is_none() && s.thermal_mut(4).is_none() && s.ptc_mut(4).is_none(), "no integrator, no journal entry");
        assert!(!s.clear_latch(LatchKey { device: 9, code: 9, pin: 9 }));
        assert!(!s.latch(rec(0, 1, -1, 0.9), true), "re-latching a latched key writes nothing");
        assert_eq!(s.journaled(), 0, "the window holds no pre-image");
        s.end_instant();
    }

    #[test]
    fn an_entry_is_saved_once_per_instant_however_often_it_is_written() {
        let mut s = populated();
        s.begin_instant();
        for i in 0..50 {
            s.bump_token(0);
            s.stress_mut(0).unwrap().close(0.01 * f64::from(i + 1));
        }
        assert_eq!(s.journaled(), 2, "one flags entry and one stress entry");
        s.rollback_instant();
        assert_eq!(s.token(0), 0);
        assert_eq!(s.stress(0).unwrap().t0, 0.0);
    }

    #[test]
    #[should_panic(expected = "rollback windows do not nest")]
    fn windows_do_not_nest() {
        let mut s = SlowState::new(1);
        s.begin_instant();
        s.begin_instant();
    }

    /// The journal is generic over a `Clone` entry: G2b's `output_sag` table
    /// will sit in the same mechanism.
    #[test]
    fn the_journal_is_generic_over_any_clone_entry() {
        #[derive(Clone, Debug, PartialEq)]
        enum Sag {
            Ok,
            Pending { dir: u8, t_s: f64 },
            Latched { dir: u8 },
        }
        let mut table = JournaledTable::new(vec![Sag::Ok, Sag::Pending { dir: 1, t_s: 0.5 }, Sag::Latched { dir: 0 }]);
        table.begin();
        *table.get_mut(0) = Sag::Pending { dir: 1, t_s: 2.0 };
        *table.get_mut(1) = Sag::Ok;
        *table.get_mut(0) = Sag::Latched { dir: 1 };
        assert_eq!(table.journaled(), 2, "entry 2 was never written");
        table.rollback();
        assert_eq!(table.as_slice(), &[Sag::Ok, Sag::Pending { dir: 1, t_s: 0.5 }, Sag::Latched { dir: 0 }]);
        table.begin();
        *table.get_mut(2) = Sag::Ok;
        table.end();
        assert_eq!(table.get(2), &Sag::Ok);
        assert_eq!(table.journaled(), 0);
        // Outside a window a write saves nothing.
        *table.get_mut(0) = Sag::Latched { dir: 0 };
        assert_eq!(table.journaled(), 0);
        assert_eq!(table.len(), 3);
    }

    #[test]
    fn a_reused_pre_image_record_restores_the_right_entry() {
        // Heap-owning entries recycle their pre-image storage across windows;
        // a stale image must never leak into a later rollback.
        let mut table = JournaledTable::new(vec![vec![1u32, 2, 3], vec![9u32], vec![]]);
        for round in 0..4u32 {
            table.begin();
            table.get_mut(0).push(100 + round);
            table.get_mut(2).extend([7, 8]);
            table.rollback();
            assert_eq!(table.as_slice(), &[vec![1, 2, 3], vec![9], vec![]], "round {round}");
        }
        table.begin();
        table.get_mut(1).clear();
        table.end();
        table.begin();
        table.get_mut(0).clear();
        table.rollback();
        assert_eq!(table.as_slice(), &[vec![1, 2, 3], vec![], vec![]]);
    }

    #[test]
    fn latch_order_moves_a_relatched_key_to_the_end() {
        let mut s = SlowState::new(4);
        let a = rec(0, 4, 1, 1.0);
        let b = rec(1, 4, 2, 2.0);
        let c = rec(2, 1, -1, 3.0);
        assert!(s.latch(a, false) && s.latch(b, false) && s.latch(c, true));
        let order = |s: &SlowState| s.latched().iter().map(|r| r.device).collect::<Vec<_>>();
        assert_eq!(order(&s), [0, 1, 2]);
        // A latched key stays where it is, and keeps its record.
        assert!(!s.latch(rec(0, 4, 1, 9.0), false));
        assert_eq!(order(&s), [0, 1, 2]);
        assert_eq!(s.latched()[0].since, 1.0);
        // A clear followed by a re-latch moves it to the end.
        assert!(s.clear_latch(a.key()));
        assert!(!s.clear_latch(a.key()), "already clear");
        assert_eq!(order(&s), [1, 2]);
        assert!(s.latch(rec(0, 4, 1, 4.0), false));
        assert_eq!(order(&s), [1, 2, 0]);
        assert_eq!(s.latched()[2].since, 4.0);
        assert!(s.is_latched(a.key()) && !s.is_latched(LatchKey { device: 3, code: 4, pin: 0 }));
        // Keys differ by code and by pin as well as by device.
        assert!(s.latch(rec(0, 4, 2, 5.0), false) && s.latch(rec(0, 3, 1, 5.0), false));
        assert_eq!(s.latched().len(), 5);
    }

    #[test]
    fn a_key_that_latches_and_clears_in_one_span_appears_once_in_the_log() {
        let mut s = SlowState::new(2);
        let key = LatchKey { device: 0, code: 4, pin: 1 };
        assert!(s.latch(rec(0, 4, 1, 1.0), false));
        assert!(s.clear_latch(key));
        assert!(s.latch(rec(0, 4, 1, 2.0), false));
        assert!(s.clear_latch(key));
        assert!(s.latch(rec(0, 4, 1, 3.0), false));
        let log = s.take_latch_log();
        assert_eq!(log.len(), 1, "once per key per span");
        assert_eq!(log[0].since, 1.0, "with the first latch's record");
        assert_eq!(log[0].value, 2.0);
        // The key still latches in the live order, at its latest time.
        assert_eq!(s.latched().len(), 1);
        assert_eq!(s.latched()[0].since, 3.0);
    }

    #[test]
    fn taking_the_log_empties_it_and_the_next_span_starts_afresh() {
        let mut s = SlowState::new(3);
        s.latch(rec(0, 1, -1, 1.0), true);
        s.latch(rec(1, 4, 0, 2.0), false);
        let first = s.take_latch_log();
        assert_eq!(first.iter().map(|r| r.device).collect::<Vec<_>>(), [0, 1], "first-latch order");
        assert!(s.latch_log().is_empty());
        assert!(s.take_latch_log().is_empty());
        // A key that latched before the take and is still latched is not logged
        // again; a key that clears and latches again after the take is.
        assert!(!s.latch(rec(0, 1, -1, 5.0), true));
        s.clear_latch(LatchKey { device: 1, code: 4, pin: 0 });
        s.latch(rec(1, 4, 0, 6.0), false);
        let second = s.take_latch_log();
        assert_eq!(second.len(), 1);
        assert_eq!((second[0].device, second[0].since), (1, 6.0));
    }

    #[test]
    fn reset_clears_latches_stress_and_ptcs_and_leaves_thermal_and_the_log() {
        let mut s = populated();
        s.latch(rec(0, 1, -1, 0.5), true);
        s.stress_mut(0).unwrap().close(0.4);
        s.thermal_mut(1).unwrap().drive = PowerDrive::Constant(0.038);
        s.thermal_mut(1).unwrap().rearm(2.0);
        s.thermal_mut(1).unwrap().shutdown = true;
        s.ptc_mut(2).unwrap().trip(0.3);
        s.ptc_mut(2).unwrap().set_drive(PtcDrive::Cool { from: 0.3, until: f64::INFINITY });
        let thermal_before = *s.thermal(1).unwrap();
        let tokens_before: Vec<u32> = (0..5).map(|d| s.token(d)).collect();

        let effects = s.reset_failures(3.0);

        assert!(s.latched().is_empty());
        assert_eq!(s.latch_log().len(), 1, "the log is left alone");
        assert!(!s.is_failed(0));
        let x = s.stress(0).unwrap();
        assert_eq!((x.s0, x.t0), (0.0, 3.0));
        assert_eq!(x.drive, StressDrive::Constant { rate: 1.0, recovery: 1.0, zero: 1.0, q: -1.0 }, "the drive is kept, so the device integrates from 0 at the reset");
        assert_eq!(x.latch_time(), Some(4.0));
        let p = s.ptc(2).unwrap();
        assert_eq!((p.tripped, p.stress, p.rt, p.t0), (false, 0.0, 0.0, 3.0));
        assert_eq!(*s.thermal(1).unwrap(), thermal_before, "T, T_hold and the shutdown flag stay");
        assert_eq!(effects.restamped, [0, 2], "an open cleared and a tripped PTC restored");
        assert_eq!(effects.rescheduled, [0, 2]);
        for dev in [0, 2] {
            assert_ne!(s.token(dev), tokens_before[dev], "the pending event of device {dev} is stale");
        }
        for dev in [1, 3, 4] {
            assert_eq!(s.token(dev), tokens_before[dev], "device {dev} was quiet and keeps its pending event");
        }
    }

    #[test]
    fn a_reset_of_a_quiet_state_journals_nothing() {
        let mut s = populated();
        s.begin_instant();
        let effects = s.reset_failures(1.0);
        assert!(effects.restamped.is_empty());
        // Device 0 accumulates under its drive, so it is touched; the rest are
        // quiet. The flags table is touched only for the token bump of device 0.
        assert_eq!(effects.rescheduled, [0]);
        assert_eq!(s.journaled(), 2 /* flags, stress of device 0 */);
        s.rollback_instant();
        let mut quiet = SlowState::new(3);
        quiet.init_stress(0, 1.0, 0.0);
        quiet.init_ptc(1, PtcParams { r_normal: 0.5, i_hold: 0.5 }, 0.0);
        quiet.begin_instant();
        assert_eq!(quiet.reset_failures(1.0), ResetEffects::default());
        assert_eq!(quiet.journaled(), 0);
        quiet.end_instant();
    }

    #[test]
    fn tokens_go_stale_on_a_redrive() {
        let mut s = populated();
        let t0 = s.token(0);
        assert!(s.is_current(0, t0));
        let t1 = s.bump_token(0);
        assert!(!s.is_current(0, t0) && s.is_current(0, t1));
        // Wrapping never panics.
        s.flags.get_mut(0).token = u32::MAX;
        assert_eq!(s.bump_token(0), 0);
    }

    #[test]
    fn reschedule_hands_back_the_fresh_event_with_its_token() {
        let mut s = populated();
        let before = s.token(0);
        let (t, kind, token) = s.reschedule(0).expect("device 0 latches at t = 1");
        assert_eq!((t, kind), (1.0, SlowEventKind::StressLatch));
        assert!(s.is_current(0, token) && !s.is_current(0, before));
        // A device with nothing pending still goes stale and returns none.
        let before = s.token(3);
        assert_eq!(s.reschedule(3), None);
        assert!(!s.is_current(3, before));
    }

    #[test]
    fn next_event_picks_the_earliest_integrator() {
        let mut s = populated();
        // Device 0: only its stress, latching at t = 1.
        assert_eq!(s.next_event(0), Some((1.0, SlowEventKind::StressLatch)));
        // A latched device's stress does nothing.
        s.latch(rec(0, 1, -1, 1.0), true);
        assert_eq!(s.next_event(0), None);
        // Device 1: a thermal boundary and a stress latch compete.
        s.thermal_mut(1).unwrap().drive = PowerDrive::Constant(0.038);
        s.stress_mut(1).unwrap().drive = StressDrive::Constant { rate: 8.867, recovery: 2.0, zero: 1.0, q: -1.0 };
        let (t, kind) = s.next_event(1).unwrap();
        assert_eq!(kind, SlowEventKind::ThermalBoundary, "a 0.01 C band is crossed long before a 24-unit stress");
        assert!(t < 0.01);
        s.thermal_mut(1).unwrap().drive = PowerDrive::Constant(0.0);
        let (t, kind) = s.next_event(1).unwrap();
        assert_eq!(kind, SlowEventKind::StressLatch);
        assert!((t - 24.0 / 8.867).abs() < 1e-12);
        // Device 2: a PTC's trip, then its recovery.
        let ptc = PtcParams { r_normal: 0.5, i_hold: 0.5 };
        s.ptc_mut(2).unwrap().set_drive(PtcDrive::Stress(ptc.untripped_constant(1.0)));
        let (t, kind) = s.next_event(2).unwrap();
        assert_eq!(kind, SlowEventKind::PtcTrip);
        assert!((t - 0.1).abs() < 1e-14);
        s.ptc_mut(2).unwrap().trip(t);
        s.ptc_mut(2).unwrap().set_drive(ptc.tripped_constant(0.005, t));
        assert_eq!(s.next_event(2).map(|e| e.1), Some(SlowEventKind::PtcRecover));
        // Devices with nothing scheduled.
        assert_eq!(s.next_event(3), None);
        assert_eq!(s.next_event(4), None);
    }

    #[test]
    fn next_event_names_the_shutdown_kind_by_the_flag() {
        let sd = thermal::Shutdown { trip_c: 150.0, restart_c: 130.0 };
        let mut s = SlowState::new(1);
        s.init_thermal(0, ThermalConfig { ambient_c: 25.0, r_theta: 100.0, tau_s: 5.0, shutdown: Some(sd), holds_junction: false }, 0.0);
        s.thermal_mut(0).unwrap().drive = PowerDrive::Constant(2.0);
        assert_eq!(s.next_event(0).map(|e| e.1), Some(SlowEventKind::ShutdownTrip));
        let t = s.next_event(0).unwrap().0;
        s.thermal_mut(0).unwrap().fire_shutdown(t);
        s.thermal_mut(0).unwrap().drive = PowerDrive::Constant(0.0);
        assert_eq!(s.next_event(0).map(|e| e.1), Some(SlowEventKind::ShutdownRestart));
    }

    #[test]
    fn due_events_latch_at_once_within_the_crossing_margin() {
        assert!(is_due(1.0, 1.0));
        assert!(is_due(1.0 + 0.5e-15, 1.0));
        assert!(is_due(0.9, 1.0));
        assert!(!is_due(1.0 + 1e-9, 1.0));
    }

    #[test]
    fn exp_voltage_builds_from_node_trajectories() {
        // A node at 5 V relaxing to 2 V from t0 = 3 with tau = 0.5.
        let node = ExpVoltage::from_trajectory(2.0, 5.0, 3.0, 0.5);
        assert_eq!(node.at(3.0), 5.0);
        assert!((node.at(3.5) - (2.0 + 3.0 * (-1.0f64).exp())).abs() < 1e-15);
        // A resistor from that node to a fixed 1 V, and between two nodes.
        let to_fixed = node.offset(1.0);
        assert!((to_fixed.at(3.5) - (node.at(3.5) - 1.0)).abs() < 1e-15);
        let other = ExpVoltage::from_trajectory(1.0, 0.0, 3.0, 0.5);
        let across = node.minus(other);
        assert!((across.at(3.7) - (node.at(3.7) - other.at(3.7))).abs() < 1e-15);
        // Rebasing leaves the function unchanged.
        let later = node.rebased(3.4);
        assert!((later.at(3.9) - node.at(3.9)).abs() < 1e-14);
        assert_eq!(later.t_seg, 3.4);
    }

    #[test]
    fn ptc_resistance_follows_the_trip_state() {
        let mut s = populated();
        assert_eq!(s.ptc_resistance(2), Some(0.5));
        s.ptc_mut(2).unwrap().trip(0.1);
        assert_eq!(s.ptc_resistance(2), Some(100.0));
        assert_eq!(s.ptc_resistance(0), None);
    }

    #[test]
    fn registration_at_load_does_not_journal() {
        // No window is open at load, so init writes save nothing.
        let mut s = SlowState::new(2);
        s.init_stress(0, 1.0, 0.0);
        s.init_thermal(1, cfg(), 0.0);
        s.init_ptc(1, PtcParams { r_normal: 0.5, i_hold: 0.5 }, 0.0);
        assert_eq!(s.journaled(), 0);
        s.begin_instant();
        s.end_instant();
    }

    #[test]
    fn a_failed_device_schedules_no_thermal_boundary_and_the_first_one_after_a_reset_is_due_at_once() {
        let cfg = ThermalConfig { ambient_c: 25.0, r_theta: 560.0, tau_s: 2.0, shutdown: None, holds_junction: true };
        let mut s = SlowState::new(1);
        s.init_stress(0, 24.0, 0.0);
        s.init_thermal(0, cfg, 0.0);
        // A hot LED, 46.3 C held, that has just failed: P = 0, so it cools.
        {
            let t = s.thermal_mut(0).unwrap();
            t.temp0 = 46.3;
            t.t_hold = 46.3;
            t.t0 = 5.0;
            t.drive = PowerDrive::Constant(0.0);
        }
        assert!(s.thermal(0).unwrap().boundary_time().is_some(), "setup: the entry alone would fire boundaries");
        assert_eq!(s.next_event(0).map(|e| e.1), Some(SlowEventKind::ThermalBoundary));
        s.latch(rec(0, 3, -1, 5.0), true);
        assert!(s.is_failed(0));
        assert_eq!(s.next_event(0), None, "a failed device stamps nothing and re-arms nothing");
        // It cools for 15 s with no event, far outside the band around T_hold.
        s.thermal_mut(0).unwrap().close(20.0);
        assert!((s.thermal(0).unwrap().t_hold - 46.3).abs() < 1e-12);
        assert!(s.thermal(0).unwrap().temp0 < 30.0);
        assert_eq!(s.next_event(0), None);
        // The reset un-fails it: T left the band long ago, so the first boundary
        // is due at t0 and re-arms at once, instead of 2 000 steps of 0.01 C.
        s.reset_failures(20.0);
        assert!(!s.is_failed(0));
        assert_eq!(s.next_event(0), Some((20.0, SlowEventKind::ThermalBoundary)));
        let (t, _, _) = s.reschedule(0).unwrap();
        assert!(is_due(t, 20.0));
        s.thermal_mut(0).unwrap().rearm(t);
        assert!((s.thermal(0).unwrap().t_hold - s.thermal(0).unwrap().temp0).abs() == 0.0);
        assert!(s.thermal(0).unwrap().t_hold < 30.0, "T_hold follows T after the reset");
    }

    #[test]
    fn a_failed_device_keeps_its_shutdown_events() {
        // Only the junction boundary is gated: a shutdown profile still acts.
        let sd = thermal::Shutdown { trip_c: 150.0, restart_c: 130.0 };
        let mut s = SlowState::new(1);
        s.init_thermal(0, ThermalConfig { ambient_c: 25.0, r_theta: 100.0, tau_s: 5.0, shutdown: Some(sd), holds_junction: true }, 0.0);
        s.thermal_mut(0).unwrap().drive = PowerDrive::Constant(2.0);
        s.latch(rec(0, 3, -1, 0.0), true);
        assert_eq!(s.next_event(0).map(|e| e.1), Some(SlowEventKind::ShutdownTrip));
    }

    #[test]
    fn a_recycled_pre_image_keeps_the_latch_book_buffers() {
        // The derived `clone_from` is `*self = source.clone()`, which allocates
        // on every save; the book's own keeps the recycled image's buffers.
        let mut table = JournaledTable::new(vec![LatchBook::default()]);
        for i in 0..10 {
            let rec = rec(i, 4, 0, f64::from(i));
            table.get_mut(0).order.push(rec);
            table.get_mut(0).log.push(rec);
        }
        table.begin();
        table.get_mut(0).order.pop();
        let first = (table.saved[0].1.order.as_ptr(), table.saved[0].1.log.as_ptr());
        assert_eq!(table.saved[0].1.order.len(), 10);
        table.end();
        // Shrink the live book, so the next pre-image fits the recycled buffers.
        table.get_mut(0).order.truncate(4);
        table.get_mut(0).log.truncate(4);
        table.begin();
        table.get_mut(0).order.pop();
        assert_eq!(table.saved[0].1.order.len(), 4);
        assert_eq!(table.saved[0].1.log.len(), 4);
        assert_eq!((table.saved[0].1.order.as_ptr(), table.saved[0].1.log.as_ptr()), first, "the pre-image reused the recycled buffers");
        table.rollback();
        assert_eq!(table.get(0).order.len(), 4);
        assert_eq!(table.get(0).log.len(), 4);
    }

    #[test]
    fn rollback_reports_whether_the_window_bumped_a_law_generation() {
        // README point 4: the junction memo is cleared at a rollback that
        // bumped any `law_gen`, because a number can recur after it.
        let mut s = populated();
        s.begin_instant();
        s.bump_token(0);
        assert!(!s.rollback_instant(), "no law_gen bumped");
        s.begin_instant();
        s.bump_law_gen(1);
        assert!(s.rollback_instant());
        assert_eq!(s.law_gen(1), 0, "the generation rolled back, so it can recur");
        // Each window starts afresh, and a completed one forgets it.
        s.begin_instant();
        s.bump_law_gen(1);
        s.end_instant();
        s.begin_instant();
        assert!(!s.rollback_instant());
    }

    #[test]
    fn a_voltage_that_does_not_relax_has_one_value() {
        let inf = f64::INFINITY;
        assert_eq!(ExpVoltage::new(5.0, 0.0, 0.0, 0.0).constant_value(), Some(5.0));
        assert_eq!(ExpVoltage::new(5.0, 0.0, inf, 0.0).constant_value(), Some(5.0));
        assert_eq!(ExpVoltage::new(3.0, 2.0, inf, 0.0).constant_value(), Some(5.0));
        assert_eq!(ExpVoltage::new(3.0, 2.0, 0.0, 0.0).constant_value(), Some(3.0));
        assert_eq!(ExpVoltage::new(3.0, 2.0, 1.0, 0.0).constant_value(), None);
        assert_eq!(ExpVoltage::new(5.0, 0.0, 1.0, 0.0).constant_value(), Some(5.0), "nothing to relax at any tau");
        // `at` and `rebased` never return NaN for them, even at the origin
        // where `0/0` would.
        for v in [ExpVoltage::new(3.0, 2.0, 0.0, 1.0), ExpVoltage::new(3.0, 2.0, inf, 1.0), ExpVoltage::new(3.0, 0.0, inf, 1.0)] {
            assert!(v.at(1.0).is_finite() && v.at(4.0).is_finite());
            let later = v.rebased(2.0);
            assert!(later.at(2.0).is_finite() && later.at(5.0).is_finite());
            assert_eq!(later.at(5.0), v.at(5.0));
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for ExpVoltage {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.a, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.b, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.tau, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.t_seg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramp, w); }
        n
    }
}

impl<E: Clone + crate::snapshot::HeapBytes> crate::snapshot::HeapBytes for JournaledTable<E> {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.entries, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.open, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.marked, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.saved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.spare, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for LatchKey {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.code, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pin, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for LatchRecord {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.code, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pin, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.since, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.limit, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aux, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for LatchBook {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.order, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.log, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DeviceFlags {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.token, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.law_gen, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.failed, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SlowEventKind {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            SlowEventKind::StressLatch => 0,
            SlowEventKind::ThermalBoundary => 0,
            SlowEventKind::ShutdownTrip => 0,
            SlowEventKind::ShutdownRestart => 0,
            SlowEventKind::PtcTrip => 0,
            SlowEventKind::PtcRecover => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for ResetEffects {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.restamped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rescheduled, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SlowState {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.flags, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stress, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.thermal, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ptc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.book, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sag, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.law_gen_bumped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ever_failed, w); }
        n
    }
}
