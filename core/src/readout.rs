//! Digital and channel readout slots and the harness trace (core/README.md,
//! "Readouts" and the v2 ABI). Owned by W1a worker A2.
//!
//! No physics lives here: this module only stores the numbers the engine
//! publishes into slots the compiler assigned, plus the instrumentation
//! trace the harness reads back through `bo_engine_trace_*`. The *keys* a
//! slot means (`u1/qa`, a bare `u1`, a `(componentId, channel)` pair) are
//! built entirely in TS (core/README.md: "Keys are built in TS; Rust
//! publishes numbers") — Rust never sees or stores them.

/// Digital and channel readout slots, sized once at load from the program's
/// declared slot counts, plus the harness instrumentation trace.
///
/// - A digital slot's value is `NaN` when the key it stands for is absent
///   this pass — classic rebuilds `digitalState` from scratch every pass, so
///   "absent" is itself meaningful, not a missing update.
/// - A channel slot holds a per-channel current in amps, `0.0` until some
///   device kind drives it.
/// - The trace is a flat, chronological list of `(t, slot, value)` triples,
///   recorded only while `trace_enabled`. It is test/harness instrumentation
///   only (core/README.md: "Harness instrumentation only; W2's host traces
///   are separate"), not part of the digital/channel slot state itself.
#[derive(Debug, Clone)]
pub struct Readout {
    digital: Vec<f64>,
    channel: Vec<f64>,
    trace_enabled: bool,
    trace: Vec<f64>,
}

impl Readout {
    /// `digital_slot_count` slots start absent (`NaN`); `channel_slot_count`
    /// slots start at `0.0` amps. Tracing starts off and the trace starts
    /// empty (core/README.md's ABI table gives no other default for
    /// `bo_engine_trace_enable`).
    pub fn new(digital_slot_count: usize, channel_slot_count: usize) -> Self {
        Readout {
            digital: vec![f64::NAN; digital_slot_count],
            channel: vec![0.0; channel_slot_count],
            trace_enabled: false,
            trace: Vec::new(),
        }
    }

    pub fn digital_slots(&self) -> &[f64] {
        &self.digital
    }

    pub fn digital_slots_mut(&mut self) -> &mut [f64] {
        &mut self.digital
    }

    pub fn channel_slots(&self) -> &[f64] {
        &self.channel
    }

    pub fn channel_slots_mut(&mut self) -> &mut [f64] {
        &mut self.channel
    }

    pub fn trace_enabled(&self) -> bool {
        self.trace_enabled
    }

    pub fn trace_enable(&mut self, on: bool) {
        self.trace_enabled = on;
    }

    /// Flat `(t, slot, value)` triples, in recorded order; length is always
    /// a multiple of 3 (core/README.md, `bo_engine_trace_len`).
    pub fn trace(&self) -> &[f64] {
        &self.trace
    }

    pub fn trace_clear(&mut self) {
        self.trace.clear();
    }

    /// Appends one `(t, slot, value)` triple when tracing is enabled, a
    /// no-op otherwise. Nothing in W1a's static kernel calls this yet — it
    /// is the digital sub-wave's hook for "record every digital slot change"
    /// once a kind actually writes to a digital slot.
    pub fn record(&mut self, t: f64, slot: usize, value: f64) {
        if !self.trace_enabled {
            return;
        }
        self.trace.push(t);
        self.trace.push(slot as f64);
        self.trace.push(value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn digital_slots_start_absent_and_channel_slots_start_zero() {
        let r = Readout::new(3, 2);
        assert!(r.digital_slots().iter().all(|v| v.is_nan()), "{:?}", r.digital_slots());
        assert_eq!(r.channel_slots(), &[0.0, 0.0]);
    }

    #[test]
    fn zero_sized_slot_counts_are_fine() {
        let r = Readout::new(0, 0);
        assert!(r.digital_slots().is_empty());
        assert!(r.channel_slots().is_empty());
    }

    #[test]
    fn trace_is_off_by_default() {
        let r = Readout::new(1, 0);
        assert!(!r.trace_enabled());
        assert!(r.trace().is_empty());
    }

    #[test]
    fn record_is_a_noop_until_tracing_is_enabled() {
        let mut r = Readout::new(1, 0);
        r.record(1.0, 0, 5.0);
        assert!(r.trace().is_empty(), "recording while disabled must not grow the trace");
    }

    #[test]
    fn record_appends_a_flat_t_slot_value_triple_once_enabled() {
        let mut r = Readout::new(1, 0);
        r.trace_enable(true);
        r.record(1.5, 0, 1.0);
        r.record(2.5, 0, 0.0);
        assert_eq!(r.trace(), &[1.5, 0.0, 1.0, 2.5, 0.0, 0.0]);
        assert_eq!(r.trace().len() % 3, 0);
    }

    #[test]
    fn trace_clear_empties_the_buffer_without_disabling_tracing() {
        let mut r = Readout::new(1, 0);
        r.trace_enable(true);
        r.record(1.0, 0, 1.0);
        r.trace_clear();
        assert!(r.trace().is_empty());
        r.record(2.0, 0, 1.0);
        assert_eq!(r.trace(), &[2.0, 0.0, 1.0], "tracing must still be on after clear");
    }

    #[test]
    fn digital_and_channel_slots_are_independently_mutable() {
        let mut r = Readout::new(2, 1);
        r.digital_slots_mut()[1] = 1.0;
        r.channel_slots_mut()[0] = 0.5;
        assert!(r.digital_slots()[0].is_nan(), "slot 0 was untouched, so it stays absent");
        assert_eq!(r.digital_slots()[1], 1.0);
        assert_eq!(r.channel_slots(), &[0.5]);
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Readout {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.digital, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.channel, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trace_enabled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trace, w); }
        n
    }
}
