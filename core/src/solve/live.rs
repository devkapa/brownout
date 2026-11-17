//! The live boundary (core/README.md, "Kernel semantics: rails and sources
//! (W1f)", Rails point 7 and Sources points 2-5): fixed-node voltages that
//! change after load. A waveform source (codes 22-25) is an ideal source in
//! the fixed-node propagation whose value steps at its edges, or follows a
//! resolved ramp; every node it fixes, and every node fixed by a source
//! stacked on one of those, moves with it.
//!
//! The values are re-derived from the propagation forest in its resolution
//! order with the same arithmetic as `partition::derive_fixed_nodes`
//! (`vp - v` below a source's + terminal, `vn + v` above its − terminal), so
//! a node whose sources have not moved reads exactly the bits it was loaded
//! with.

use crate::devices::sources::{Clock, Pulse, PulseEventKind, RampSeg};
use crate::partition::ResolvedSource;

/// A `Source` event whose version carries this bit is a reader's threshold
/// crossing on a resolved ramp (Sources point 3), its low bits the ramp's
/// generation; any other is the source's next edge.
pub(crate) const WATCH_BIT: u64 = 1 << 63;

/// One waveform source's runtime state.
#[derive(Debug, Clone)]
pub(crate) struct WaveSource {
    pub device: usize,
    pub schedule: Schedule,
    /// The digital slot it publishes.
    pub slot: usize,
    /// Its present value (the start of the running ramp while one runs).
    pub value: f64,
    /// The level index for the static cache key: 1 at `vHigh` (clock) or `v2`
    /// (pulse), 0 otherwise (README, Rails point 7, "Static cache").
    pub level_index: u8,
    /// The digital slot value.
    pub slot_value: f64,
    /// The resolved ramp running now, if any.
    pub ramp: Option<RampSeg>,
    /// The index of the next edge or event to apply, `None` when there is
    /// none left.
    pub next: Option<u64>,
    /// Bumped at every schedule: a `Source` event carrying an older one is
    /// stale.
    pub version: u64,
    /// Bumped whenever a resolved ramp starts or ends: a crossing watch of an
    /// older ramp is stale.
    pub watch_gen: u64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum Schedule {
    Clock(Clock),
    Pulse(Pulse),
}

/// What one edge or event does.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Edge {
    pub time: f64,
    pub value: f64,
    pub slot: f64,
    pub level_index: u8,
    pub ramp: Option<RampSeg>,
    /// Whether the boundary changes at it (a step, or a ramp corner where the
    /// slope changes); a ramp's midpoint only flips the slot.
    pub moves_boundary: bool,
}

impl WaveSource {
    /// The edge with index `k`, if the schedule has one.
    pub(crate) fn edge(&self, k: u64) -> Option<Edge> {
        match &self.schedule {
            Schedule::Clock(c) => c.edge(k).map(|e| Edge {
                time: e.time,
                value: e.level,
                slot: e.slot,
                level_index: u8::from(e.rising),
                ramp: None,
                moves_boundary: true,
            }),
            Schedule::Pulse(p) => p.event(k).map(|e| {
                let (ramp, moves) = match e.kind {
                    PulseEventKind::Step => (None, true),
                    PulseEventKind::RampStart => (e.ramp, true),
                    PulseEventKind::RampMid => (e.ramp, false),
                    PulseEventKind::RampEnd => (None, true),
                };
                // After a rise the source sits at v2; after a fall at v1. A
                // ramp in flight keeps the index of the level it left.
                let level_index = match e.kind {
                    PulseEventKind::Step | PulseEventKind::RampEnd => u8::from(e.rising),
                    PulseEventKind::RampStart | PulseEventKind::RampMid => u8::from(!e.rising),
                };
                Edge { time: e.time, value: e.level, slot: e.slot, level_index, ramp, moves_boundary: moves }
            }),
        }
    }

    /// The value at `t` while a ramp runs, else the held value.
    pub(crate) fn value_at(&self, t: f64) -> f64 {
        match &self.ramp {
            Some(r) => r.value(t),
            None => self.value,
        }
    }
}

/// The nodes a running ramp moves, and how to evaluate them at any instant.
#[derive(Debug, Clone, Default)]
pub(crate) struct RampEval {
    /// The propagation forest (`partition::PartitionBuild::resolved`).
    pub resolved: Vec<ResolvedSource>,
    /// node -> the index in `resolved` of the entry that fixed it.
    pub fixer: Vec<u32>,
    /// device -> its index in `sources`, `u32::MAX` for a constant source.
    pub wave_of: Vec<u32>,
    /// device -> the constant value of a non-waveform source in the forest.
    pub constant: Vec<f64>,
    /// The waveform sources.
    pub sources: Vec<WaveSource>,
    /// node -> whether a running ramp moves it right now.
    pub ramping: Vec<bool>,
    ramping_count: usize,
}

impl RampEval {
    pub(crate) fn new(resolved: Vec<ResolvedSource>, node_count: usize, device_count: usize) -> Self {
        let mut fixer = vec![u32::MAX; node_count];
        for (k, r) in resolved.iter().enumerate() {
            fixer[r.child_node] = k as u32;
        }
        RampEval {
            resolved,
            fixer,
            wave_of: vec![u32::MAX; device_count],
            constant: vec![0.0; device_count],
            sources: Vec::new(),
            ramping: vec![false; node_count],
            ramping_count: 0,
        }
    }

    pub(crate) fn any_ramping(&self) -> bool {
        self.ramping_count > 0
    }

    /// The value of `node` at `t` when a running ramp moves it, `None`
    /// otherwise (the caller then reads the published value).
    pub(crate) fn value(&self, node: usize, t: f64, published: &[f64]) -> Option<f64> {
        if !self.ramping.get(node).copied().unwrap_or(false) {
            return None;
        }
        Some(self.walk(node, t, published))
    }

    fn walk(&self, node: usize, t: f64, published: &[f64]) -> f64 {
        if !self.ramping[node] {
            return published[node];
        }
        let r = &self.resolved[self.fixer[node] as usize];
        let parent = self.walk(r.parent_node, t, published);
        let v = self.source_value(r.device_index, t);
        if r.parent_is_pos {
            parent - v
        } else {
            parent + v
        }
    }

    /// How a running ramp moves `node`, `None` when none does: its slope in
    /// V/s, the sum along the stack of sources above the datum, and the
    /// earliest end of those ramps, where the slope next changes (Sources
    /// point 3, R3 (b)).
    pub(crate) fn node_ramp(&self, node: usize) -> Option<(f64, f64)> {
        if !self.ramping.get(node).copied().unwrap_or(false) {
            return None;
        }
        let r = &self.resolved[self.fixer[node] as usize];
        let (mut slope, mut end) = self.node_ramp(r.parent_node).unwrap_or((0.0, f64::INFINITY));
        if let Some(ramp) = self.wave_of.get(r.device_index).filter(|&&w| w != u32::MAX).and_then(|&w| self.sources[w as usize].ramp) {
            let s = (ramp.v_to - ramp.v_from) / ramp.len;
            slope += if r.parent_is_pos { -s } else { s };
            end = end.min(ramp.t0 + ramp.len);
        }
        Some((slope, end))
    }

    /// [`Self::node_ramp`] with the span the slope holds over: the latest
    /// start and the earliest end of the ramps along the stack (R99 (3)).
    pub(crate) fn node_ramp_span(&self, node: usize) -> Option<(f64, f64, f64)> {
        if !self.ramping.get(node).copied().unwrap_or(false) {
            return None;
        }
        let r = &self.resolved[self.fixer[node] as usize];
        let (mut slope, mut start, mut end) = self.node_ramp_span(r.parent_node).unwrap_or((0.0, f64::NEG_INFINITY, f64::INFINITY));
        if let Some(ramp) = self.wave_of.get(r.device_index).filter(|&&w| w != u32::MAX).and_then(|&w| self.sources[w as usize].ramp) {
            let s = (ramp.v_to - ramp.v_from) / ramp.len;
            slope += if r.parent_is_pos { -s } else { s };
            start = start.max(ramp.t0);
            end = end.min(ramp.t0 + ramp.len);
        }
        Some((slope, start, end))
    }

    /// A forest source's value at `t`.
    pub(crate) fn source_value(&self, device: usize, t: f64) -> f64 {
        match self.wave_of[device] {
            u32::MAX => self.constant[device],
            w => self.sources[w as usize].value_at(t),
        }
    }

    /// Recomputes which nodes a running ramp moves (after a ramp starts or
    /// ends).
    pub(crate) fn refresh_ramping(&mut self) {
        self.ramping.fill(false);
        for r in &self.resolved {
            let moves = self.ramping[r.parent_node]
                || match self.wave_of[r.device_index] {
                    u32::MAX => false,
                    w => self.sources[w as usize].ramp.is_some(),
                };
            if moves {
                self.ramping[r.child_node] = true;
            }
        }
        self.ramping_count = self.ramping.iter().filter(|&&r| r).count();
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for WaveSource {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.device, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.schedule, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slot, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.level_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slot_value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.next, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.version, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.watch_gen, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Schedule {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Schedule::Clock(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            Schedule::Pulse(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for Edge {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.time, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slot, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.level_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.moves_boundary, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for RampEval {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.resolved, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.fixer, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.wave_of, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.constant, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.sources, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramping, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ramping_count, w); }
        n
    }
}
