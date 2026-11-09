//! The event heap, ordered by (t, seq) (core/README.md, "Kernel semantics:
//! events", point 1). Owned by W1a worker A1.
//!
//! This module is deliberately thin: it owns the heap and the ordering rule,
//! nothing else. It does not know what a "driver" or a "partition" is beyond
//! an opaque id it carries in the payload — `drivers.rs` owns the
//! token-cancellation contract for commits, and `kernel.rs` interprets every
//! payload when an event's time arrives.
//!
//! Every payload that can be superseded (a commit, a crossing) carries a
//! version/token the owning module bumps whenever it retargets or cancels
//! in-flight work. Cancelling never reaches into the heap to remove the old
//! event — heap removal by identity would need an index back into a
//! binary-heap's array, which move on every push/pop. Instead the event stays
//! queued and is popped like any other; the owner compares the carried
//! version against its own current one and silently drops a stale pop. This
//! is the lazy-deletion half of the token-version pattern; `drivers.rs`'s
//! `Fsm::step` produces the fresh token, and `kernel.rs::apply_commit` (or
//! the crossing/timer equivalent) does the comparison.

use std::cmp::Ordering;

/// Global insertion counter. Two events at equal `t` pop in the order they
/// were pushed (FIFO), never in whatever order a heap's internal shuffling
/// would otherwise produce, and never by payload contents (a `Commit` and a
/// `Timer` at the same instant have no natural ordering of their own).
pub type Seq = u64;

/// An opaque driver identity. `drivers.rs` and `kernel.rs` both index a
/// caller-owned `Vec<Driver>` (or equivalent) with this; this module never
/// dereferences it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct DriverId(pub u32);

/// An opaque partition identity, as the partition-solver trait (`solve::PartitionSolver`)
/// names its own partitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PartitionId(pub u32);

/// An opaque device identity, for timers a device schedules against itself
/// (a floating-bucket boundary, a future EEPROM T_WC completion). The kernel
/// never interprets which device this is; only the device-evaluation trait
/// impl does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct DeviceId(pub u32);

/// A driver's inertial commit token (drivers.rs). Bumped on every retarget or
/// cancellation of that driver's in-flight transition, so a stale `Commit`
/// event (superseded before it became due) is recognisable on pop.
pub type Token = u64;

/// A partition's solve epoch (solve::PartitionSolver). Bumped every time the
/// partition is re-solved, so a `Crossing` scheduled against a since-replaced
/// closed-form segment is recognisable on pop — the prototype's
/// `ev.version !== ev.part.version` check (pec3.mjs `advance`).
pub type Version = u64;

/// Driver output level. Re-exported by `drivers.rs`; defined here because
/// `Commit` needs it and events.rs must not depend on drivers.rs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Hiz,
    High,
    Low,
}

/// What happens when an event's time arrives.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EventKind {
    /// A driver's inertial transition becoming due (contract point 2). Valid
    /// only if `token` still matches the driver's current token when popped.
    Commit { driver: DriverId, mode: Mode, token: Token },
    /// A partition threshold crossing becoming due (closed-form dynamics
    /// reaching a comparator boundary). Valid only if `version` still matches
    /// the partition's current solve epoch when popped.
    Crossing { partition: PartitionId, version: Version },
    /// A stepped partition's planned step becoming due (W1b, `solve::stepped`
    /// — pec3.mjs's `EV_STEP`). Valid only if `version` still matches the
    /// partition's current solve epoch when popped: a re-aim (a switch toggle,
    /// a driver commit, or the horizon-clamp's forced resync,
    /// `kernel::Kernel::advance`) discards whatever step was in flight and
    /// plans a fresh one under a new version, exactly like a closed-form
    /// re-aim cancels a stale `Crossing`.
    Step { partition: PartitionId, version: Version },
    /// A device-owned timer, opaque to the kernel: any future per-device
    /// deadline. `tag` lets one device distinguish which of several timers it
    /// scheduled fired; the kernel never inspects it.
    Timer { device: DeviceId, tag: u64 },
    /// A floating-bucket boundary (W1c contract point 5/6, owner decision 9):
    /// scheduled by `solve::PartitionSolver::take_scheduled` at
    /// `digital::floating::next_bucket_boundary(now)` whenever a powered IC
    /// holds an open `FLOATING_INPUT` pin or reads a pin in its dead band.
    /// Carries no partition or version: the solver schedules at most one of
    /// these at a time (deduplicated — `PartitionSolver::take_scheduled`'s own
    /// doc comment), so, unlike a `Crossing`/`Step`, a popped `Bucket` is
    /// never stale by construction and is always handled by calling
    /// `PartitionSolver::bucket_boundary` outright.
    Bucket,
    /// The hub rail's next event (W1f, core/README.md Rails point 4): due when
    /// its implied trajectory drifts, crosses an IC's power window, reaches a
    /// PSU's regime threshold or an OCV knot. One is pending at a time; every
    /// rail solve supersedes it, so it is stale when `version` is not the
    /// rail's pending one (`PartitionSolver::rail_current`).
    Rail { version: Version },
    /// A waveform source's next edge (W1f, Sources points 2, 3 and 5), stale
    /// when `version` is not the source's present one
    /// (`PartitionSolver::source_current`).
    Source { device: DeviceId, version: Version },
    /// A slow device's next event (W1g, core/README.md W1g point 14): a stress
    /// latch, a thermal boundary, a PTC trip or recovery, or an IC's
    /// `output_sag` window running out. Dispatched in phase 1 like `Bucket`,
    /// and stale when the device's token has moved on since it was scheduled
    /// (`PartitionSolver::slow_is_current`): a publish that re-drives the
    /// device bumps the token and schedules afresh.
    Slow { device: DeviceId, token: u32 },
}

/// Set in a `Slow` event's token when it re-anchors a rail-driven drive (W1g
/// ruling R21) rather than standing for the device's own next event: it
/// fires nothing, only re-drives. The rest of the token is the device's.
pub const SLOW_ANCHOR: u32 = 1 << 31;

/// One scheduled occurrence: a time and a payload. `seq` is set by
/// `EventHeap::push`, never by the caller, so it is always consistent with
/// insertion order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Event {
    pub time: f64,
    pub seq: Seq,
    pub kind: EventKind,
}

/// Ordered strictly by (time, seq) using `f64::total_cmp`: NaN never compares
/// equal to itself under `==`, which would silently corrupt a heap built on
/// derived `Eq`/`Ord` (a NaN time is a caller bug in every case that reaches
/// here — the kernel never schedules one — but the heap's own invariants must
/// not depend on that).
impl Eq for Event {}

impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Event {
    fn cmp(&self, other: &Self) -> Ordering {
        self.time.total_cmp(&other.time).then_with(|| self.seq.cmp(&other.seq))
    }
}

/// A binary min-heap on `(time, seq)`. Wraps `std::collections::BinaryHeap`
/// (a max-heap) behind `Reverse`-style ordering rather than porting the
/// prototype's hand-rolled parallel-array heap (`pec2.mjs` `EventHeap`): that
/// version chased raw JS array performance, which Rust's stdlib heap already
/// gives us without hand-written sift-up/down.
// `Clone` is needed by `Kernel::advance`'s "a failed advance" rollback
// (core/README.md): it snapshots the whole heap before a delta cycle that
// might livelock or hit a solve failure, so it can put every event exactly
// back the way it found it if that instant never reaches a quiet fixed
// point. `Event` (and `Reverse<Event>`) is already `Clone`, so this is a
// plain structural derive, not new logic.
#[derive(Debug, Clone, Default)]
pub struct EventHeap {
    heap: std::collections::BinaryHeap<std::cmp::Reverse<Event>>,
    next_seq: Seq,
}

impl EventHeap {
    pub fn new() -> Self {
        EventHeap { heap: std::collections::BinaryHeap::new(), next_seq: 0 }
    }

    pub fn len(&self) -> usize {
        self.heap.len()
    }

    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    /// Schedules `kind` at `time`, assigning the next sequence number. Ties
    /// at equal `time` therefore always resolve FIFO by push order (contract
    /// point 1), regardless of how many other events already share that
    /// time.
    pub fn push(&mut self, time: f64, kind: EventKind) -> Seq {
        let seq = self.next_seq;
        self.next_seq += 1;
        self.heap.push(std::cmp::Reverse(Event { time, seq, kind }));
        seq
    }

    /// The earliest pending time, or `+Infinity` when the heap is empty —
    /// the same convention `bo_engine_next_event_time` publishes (contract,
    /// ABI v2), so a caller polling this needs no separate empty check.
    pub fn peek_time(&self) -> f64 {
        self.heap.peek().map_or(f64::INFINITY, |std::cmp::Reverse(ev)| ev.time)
    }

    /// The earliest-ordered event, left in place, or `None` when empty.
    pub fn peek(&self) -> Option<&Event> {
        self.heap.peek().map(|std::cmp::Reverse(ev)| ev)
    }

    /// Removes and returns the earliest-ordered event, or `None` when empty.
    pub fn pop(&mut self) -> Option<Event> {
        self.heap.pop().map(|std::cmp::Reverse(ev)| ev)
    }

    /// Re-inserts an event exactly as it was, `seq` included, never assigning
    /// a fresh one the way [`Self::push`] does. For a caller that popped an
    /// event only to inspect it and found it still live (`Kernel`'s
    /// `next_event_time`, which must peek past a stale, cancelled `Commit`
    /// without losing the live event under it): giving the reinserted event a
    /// new `seq` would move it after every sibling already in the heap at the
    /// same `time`, breaking the FIFO tie-break contract (point 1) for events
    /// this call never touched.
    pub fn push_event(&mut self, ev: Event) {
        self.heap.push(std::cmp::Reverse(ev));
    }

    /// The sequence number the next [`Self::push`] will assign: a checkpoint
    /// marker for [`Self::rollback`]. Every event pushed after this call has a
    /// `seq` at or above it, and every event already queued has one below it.
    pub fn next_seq(&self) -> Seq {
        self.next_seq
    }

    /// Undoes every push and pop made since [`Self::next_seq`] returned
    /// `checkpoint`, given the events popped in that span (in any order; ones
    /// that were pushed after the checkpoint too are ignored, they did not
    /// exist then). Replaces cloning the whole heap before every instant:
    /// `(time, seq)` is a strict total order (`seq` is unique), so the pop
    /// order depends only on WHICH events are queued, never on the heap
    /// array's internal layout, and a rebuilt heap pops exactly like a
    /// cloned one. `next_seq` goes back too, so later sequence numbers match
    /// what they would have been had the span never run.
    pub fn rollback(&mut self, checkpoint: Seq, popped: &[Event]) {
        let mut events = std::mem::take(&mut self.heap).into_vec();
        events.retain(|std::cmp::Reverse(ev)| ev.seq < checkpoint);
        events.extend(popped.iter().filter(|ev| ev.seq < checkpoint).map(|&ev| std::cmp::Reverse(ev)));
        self.heap = std::collections::BinaryHeap::from(events);
        self.next_seq = checkpoint;
    }

    /// Every still-pending event, in no particular order (the underlying
    /// `BinaryHeap`'s own storage order, not `(time, seq)`) — for a caller
    /// that needs to SCAN the whole heap rather than pop it in order
    /// (`Kernel::next_digital_due_time`, finding 10/8: the earliest pending
    /// driver `Commit` among possibly several, mixed in with `Crossing`/
    /// `Step`/`Bucket` entries this scan must also see so it can filter them
    /// out). Never removes anything, unlike [`Self::pop`], so a caller that
    /// only wants to look does not need to pop-then-`push_event` every entry
    /// back just to answer one question.
    pub fn iter(&self) -> impl Iterator<Item = &Event> {
        self.heap.iter().map(|std::cmp::Reverse(ev)| ev)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn commit(driver: u32, mode: Mode, token: Token) -> EventKind {
        EventKind::Commit { driver: DriverId(driver), mode, token }
    }

    #[test]
    fn pops_in_time_order() {
        let mut h = EventHeap::new();
        h.push(3.0, commit(0, Mode::High, 1));
        h.push(1.0, commit(1, Mode::Low, 1));
        h.push(2.0, commit(2, Mode::Hiz, 1));
        assert_eq!(h.pop().unwrap().time, 1.0);
        assert_eq!(h.pop().unwrap().time, 2.0);
        assert_eq!(h.pop().unwrap().time, 3.0);
        assert!(h.pop().is_none());
    }

    #[test]
    fn equal_times_pop_fifo_by_insertion_sequence() {
        // Contract point 1: "seq is a global insertion counter, so events at
        // equal t run first in, first out." A heap ordered on time alone
        // would let the tree's own shuffling decide, which is not FIFO and
        // not deterministic across different push orders that happen to
        // produce the same multiset of times.
        let mut h = EventHeap::new();
        for i in 0..5u32 {
            h.push(10.0, commit(i, Mode::High, 1));
        }
        for i in 0..5u32 {
            let ev = h.pop().unwrap();
            match ev.kind {
                EventKind::Commit { driver, .. } => assert_eq!(driver, DriverId(i)),
                _ => unreachable!(),
            }
        }
    }

    #[test]
    fn rollback_undoes_pushes_and_pops_since_the_checkpoint_and_keeps_fifo_order() {
        // Replaces cloning the heap before every instant: what was queued at
        // the checkpoint comes back (popped events re-inserted, pushed ones
        // dropped, an event pushed AND popped in the span forgotten), the
        // sequence counter goes back, and the pop order (time, then insertion
        // order) is exactly what an untouched heap would give.
        let build = || {
            let mut h = EventHeap::new();
            h.push(5.0, commit(0, Mode::High, 1));
            h.push(5.0, commit(1, Mode::High, 1));
            h.push(7.0, commit(2, Mode::Low, 1));
            h.push(9.0, commit(3, Mode::Low, 1));
            h
        };
        let mut h = build();
        let checkpoint = h.next_seq();
        let mut popped = Vec::new();
        popped.push(h.pop().unwrap()); // the first t = 5 commit
        h.push(5.0, commit(4, Mode::High, 2)); // pushed after the checkpoint...
        popped.push(h.pop().unwrap()); // ...the second t = 5 commit
        popped.push(h.pop().unwrap()); // ...the pushed one, which must not come back
        h.push(6.0, commit(5, Mode::Low, 1));
        h.rollback(checkpoint, &popped);

        let mut untouched = build();
        assert_eq!(h.next_seq(), untouched.next_seq());
        assert_eq!(h.len(), untouched.len());
        loop {
            let (a, b) = (h.pop(), untouched.pop());
            assert_eq!(a, b);
            if a.is_none() {
                break;
            }
        }
    }

    #[test]
    fn peek_time_is_infinity_when_empty() {
        let h = EventHeap::new();
        assert_eq!(h.peek_time(), f64::INFINITY);
    }

    #[test]
    fn peek_time_matches_the_next_pop_without_removing_it() {
        let mut h = EventHeap::new();
        h.push(5.0, commit(0, Mode::High, 1));
        assert_eq!(h.peek_time(), 5.0);
        assert_eq!(h.len(), 1);
        assert_eq!(h.pop().unwrap().time, 5.0);
    }

    #[test]
    fn total_cmp_orders_events_even_with_extreme_finite_times() {
        // f64::total_cmp is what the contract names explicitly (point 1); it
        // gives a total order over every bit pattern including signed zeros,
        // which matters less here than simply confirming the heap does not
        // rely on `PartialOrd`'s NaN-unordered behaviour for ordinary values.
        let mut h = EventHeap::new();
        h.push(-1.0, commit(0, Mode::High, 1));
        h.push(0.0, commit(1, Mode::High, 1));
        h.push(f64::MAX, commit(2, Mode::High, 1));
        assert_eq!(h.pop().unwrap().time, -1.0);
        assert_eq!(h.pop().unwrap().time, 0.0);
        assert_eq!(h.pop().unwrap().time, f64::MAX);
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for DriverId {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.0, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PartitionId {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.0, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for DeviceId {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.0, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Mode {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Mode::Hiz => 0,
            Mode::High => 0,
            Mode::Low => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for EventKind {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            EventKind::Commit { driver, mode, token } => crate::snapshot::HeapBytes::heap_bytes(driver, w) + crate::snapshot::HeapBytes::heap_bytes(mode, w) + crate::snapshot::HeapBytes::heap_bytes(token, w),
            EventKind::Crossing { partition, version } => crate::snapshot::HeapBytes::heap_bytes(partition, w) + crate::snapshot::HeapBytes::heap_bytes(version, w),
            EventKind::Step { partition, version } => crate::snapshot::HeapBytes::heap_bytes(partition, w) + crate::snapshot::HeapBytes::heap_bytes(version, w),
            EventKind::Timer { device, tag } => crate::snapshot::HeapBytes::heap_bytes(device, w) + crate::snapshot::HeapBytes::heap_bytes(tag, w),
            EventKind::Bucket => 0,
            EventKind::Rail { version } => crate::snapshot::HeapBytes::heap_bytes(version, w),
            EventKind::Source { device, version } => crate::snapshot::HeapBytes::heap_bytes(device, w) + crate::snapshot::HeapBytes::heap_bytes(version, w),
            EventKind::Slow { device, token } => crate::snapshot::HeapBytes::heap_bytes(device, w) + crate::snapshot::HeapBytes::heap_bytes(token, w),
        }
    }
}

impl crate::snapshot::HeapBytes for Event {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.time, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seq, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for EventHeap {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.heap, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.next_seq, w); }
        n
    }
}
