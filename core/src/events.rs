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
}

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
