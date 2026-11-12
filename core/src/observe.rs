//! W2b: the observer (core/README.md, "Acquisition"; W2 design 2.4,
//! rulings R57 D1-D3 (a) and R104).
//!
//! A host arms it with nodes and a sample grid `origin + k * interval`; the
//! solver hands it each sample's value as soon as the model's trajectory
//! there is final, and the host takes the queue. It is passive: a sample is
//! read off what the solver already holds (a closed form's segment, a static
//! node between its instants, a stepped node between the two commits around
//! it), never by moving a step, so a probed run is the unprobed run bit for
//! bit.
//!
//! The times are `origin + k * interval` for whole `k`, never accumulated, so
//! a long run's grid does not drift (classic's worker computes them so). A
//! sample at the very time of an instant reads the state after it (D2: the
//! `t_end` readout is right-continuous).
//!
//! The queue keeps at most `limit` samples (R104: `2^20` by default, the
//! host can set it): past that, later samples are dropped and counted, and
//! the oldest are kept, so a host that falls behind loses its newest data,
//! not a gap in the middle.

/// R104's default queue length, in samples (each a value per probed node).
pub const DEFAULT_LIMIT: usize = 1 << 20;

/// The words before a take's per-node fallback counts and its times:
/// `count, maxSpan, overflowed, dropped`.
pub const TAKE_HEADER: usize = 4;

/// The grid's first index past `t`: the least `k >= 0` whose time `origin +
/// k * interval` (computed so, as everywhere on the grid) is after `t`, or at
/// or after it without `inclusive`. The quotient `(t - origin) / interval`
/// only estimates it, a rounding either side (W2bV: `floor` of 4.999.. lost
/// the grid time at `t`), so the estimate is corrected against the times
/// themselves. Past 2^52 neighbouring indices' times no longer differ and
/// the estimate stands, held below `u64`'s range (a tiny interval
/// overflowed `as u64 + 1`).
fn grid_end(origin: f64, interval: f64, t: f64, inclusive: bool) -> u64 {
    const EXACT: f64 = (1u64 << 52) as f64;
    let past = |k: u64| {
        let tk = origin + k as f64 * interval;
        if inclusive { tk > t } else { tk >= t }
    };
    let q = ((t - origin) / interval).floor();
    if !(q < EXACT) {
        return if q.is_nan() { 0 } else { (q.min(2.0 * EXACT)) as u64 };
    }
    let mut k = q.max(0.0) as u64;
    while k > 0 && past(k - 1) {
        k -= 1;
    }
    while !past(k) {
        k += 1;
    }
    k
}

#[derive(Clone, Debug)]
pub struct Observer {
    nodes: Vec<usize>,
    origin: f64,
    interval: f64,
    /// The index `k` of the next sample to read.
    next: u64,
    times: Vec<f64>,
    /// Sample-major: sample `s`'s value of node `j` at `s * nodes.len() + j`.
    values: Vec<f64>,
    limit: usize,
    dropped: u64,
    /// The widest span between the two stepped commits a sample was read
    /// between, since the last take (0 when none was).
    max_span: f64,
    /// Per probed node, the samples that fell back to linear where R104's
    /// solve was due (its Newton failed), since the last take.
    fallbacks: Vec<u64>,
    row: Vec<f64>,
    row_fallback: Vec<bool>,
}

/// Where an instant's window found the queue, to put it back on a rollback.
#[derive(Clone, Debug)]
pub struct Mark {
    len: usize,
    next: u64,
    dropped: u64,
    max_span: f64,
    fallbacks: Vec<u64>,
}

impl Observer {
    /// Armed at `now`: the first sample is the first grid time at or after
    /// `now`.
    pub fn new(nodes: Vec<usize>, origin: f64, interval: f64, limit: usize, now: f64) -> Self {
        let next = grid_end(origin, interval, now, false);
        let n = nodes.len();
        Observer { nodes, origin, interval, next, times: Vec::new(), values: Vec::new(), limit: limit.max(1), dropped: 0, max_span: 0.0, fallbacks: vec![0; n], row: vec![0.0; n], row_fallback: vec![false; n] }
    }

    /// The grid, `(origin, interval)`.
    pub fn grid(&self) -> (f64, f64) {
        (self.origin, self.interval)
    }

    pub fn nodes(&self) -> &[usize] {
        &self.nodes
    }

    /// The next sample's time.
    pub fn next_time(&self) -> f64 {
        self.origin + self.next as f64 * self.interval
    }

    /// Reads every sample whose time is before `t_limit` (at or before it with
    /// `inclusive`), in order, while `value(j, t)` gives probed node `j`'s
    /// value there with the span of the stepped commits it lay between (0 for
    /// none) and whether it fell back to linear; the first sample a node
    /// cannot give yet (its later commit not made) stops the reading, to be
    /// read again later.
    pub fn read_until(&mut self, t_limit: f64, inclusive: bool, mut value: impl FnMut(usize, f64) -> Option<(f64, f64, bool)>) {
        loop {
            let t = self.next_time();
            if t > t_limit || (!inclusive && t == t_limit) {
                return;
            }
            if self.times.len() >= self.limit {
                // Full: every sample up to the limit is dropped and counted,
                // without reading the model.
                let end = grid_end(self.origin, self.interval, t_limit, inclusive).max(self.next);
                self.dropped = self.dropped.saturating_add(end - self.next);
                self.next = end;
                return;
            }
            let mut span = 0.0f64;
            for j in 0..self.nodes.len() {
                match value(j, t) {
                    Some((v, s, fallback)) => {
                        self.row[j] = v;
                        self.row_fallback[j] = fallback;
                        span = span.max(s);
                    }
                    None => return,
                }
            }
            self.times.push(t);
            self.values.extend_from_slice(&self.row);
            for (f, &b) in self.fallbacks.iter_mut().zip(&self.row_fallback) {
                *f += u64::from(b);
            }
            self.max_span = self.max_span.max(span);
            self.next += 1;
        }
    }

    /// `[count, maxSpan, overflowed, dropped, fallbacks x nodes, t x count,
    /// then node-major values (nodes x count)]`, and an empty queue after it.
    pub fn take(&mut self, out: &mut Vec<f64>) {
        let (count, n) = (self.times.len(), self.nodes.len());
        out.clear();
        out.extend([count as f64, self.max_span, f64::from(u8::from(self.dropped > 0)), self.dropped as f64]);
        out.extend(self.fallbacks.iter().map(|&f| f as f64));
        self.fallbacks.iter_mut().for_each(|f| *f = 0);
        out.extend_from_slice(&self.times);
        for j in 0..n {
            out.extend((0..count).map(|s| self.values[s * n + j]));
        }
        self.times.clear();
        self.values.clear();
        self.dropped = 0;
        self.max_span = 0.0;
    }

    /// D2 at a host's edit: the samples already read at exactly `now` (the
    /// clamp reads its `t_end`) go back to be read again after the edit's
    /// instant, which is at the same time and after them. A stored one
    /// leaves the queue (its fallbacks, the last row read, uncounted), a
    /// dropped one leaves the count; one already taken stays the host's.
    pub fn unread_at(&mut self, now: f64) {
        let n = self.nodes.len();
        while self.next > 0 && self.origin + (self.next - 1) as f64 * self.interval == now {
            if self.times.last() == Some(&now) {
                self.times.pop();
                self.values.truncate(self.times.len() * n);
                for (f, &b) in self.fallbacks.iter_mut().zip(&self.row_fallback) {
                    *f -= u64::from(b && *f > 0);
                }
                self.row_fallback.iter_mut().for_each(|b| *b = false);
            } else if self.dropped > 0 && self.times.len() >= self.limit {
                self.dropped -= 1;
            } else {
                // Taken already: the host has it.
                return;
            }
            self.next -= 1;
        }
    }

    pub fn mark(&self) -> Mark {
        Mark { len: self.times.len(), next: self.next, dropped: self.dropped, max_span: self.max_span, fallbacks: self.fallbacks.clone() }
    }

    /// Puts the queue back to `mark` (a rolled-back instant read nothing).
    pub fn rewind(&mut self, mark: Mark) {
        self.times.truncate(mark.len);
        self.values.truncate(mark.len * self.nodes.len());
        (self.next, self.dropped, self.max_span) = (mark.next, mark.dropped, mark.max_span);
        self.fallbacks = mark.fallbacks;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_grid_starts_at_now_and_reads_in_order_up_to_a_limit() {
        let mut o = Observer::new(vec![3], 0.0, 0.25, DEFAULT_LIMIT, 0.5);
        assert_eq!(o.next_time(), 0.5, "a grid time at now is the first");
        o.read_until(1.0, false, |_, t| Some((t * 10.0, 0.0, false)));
        o.read_until(1.0, true, |_, t| Some((t * 10.0, 0.0, false)));
        let mut out = Vec::new();
        o.take(&mut out);
        assert_eq!(out, vec![3.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.75, 1.0, 5.0, 7.5, 10.0]);
    }

    #[test]
    fn a_sample_a_node_cannot_give_yet_stops_the_reading_and_comes_back() {
        let mut o = Observer::new(vec![1, 2], 0.0, 1.0, DEFAULT_LIMIT, 0.0);
        // `j` indexes the probed nodes: node 1 is 0, node 2 is 1.
        o.read_until(5.0, true, |j, t| if j == 1 && t > 1.0 { None } else { Some((t + (j + 1) as f64, 0.5, false)) });
        assert_eq!(o.next_time(), 2.0);
        o.read_until(5.0, true, |j, t| Some((t + (j + 1) as f64, 0.0, false)));
        let mut out = Vec::new();
        o.take(&mut out);
        assert_eq!(&out[..6], &[6.0, 0.5, 0.0, 0.0, 0.0, 0.0]);
        assert_eq!(&out[6..12], &[0.0, 1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(&out[12..], &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
    }

    #[test]
    fn a_full_queue_keeps_the_oldest_and_counts_the_dropped() {
        let mut o = Observer::new(vec![0], 0.0, 0.1, 3, 0.0);
        o.read_until(1.0, true, |_, t| Some((t, 0.0, false)));
        let mut out = Vec::new();
        o.take(&mut out);
        assert_eq!(&out[..5], &[3.0, 0.0, 1.0, 8.0, 0.0], "0, 0.1, 0.2 kept; 0.3 .. 1.0 dropped");
        assert_eq!(&out[5..8], &[0.0, 0.1, 0.2]);
        o.read_until(1.25, true, |_, t| Some((t, 0.0, false)));
        o.take(&mut out);
        assert_eq!(out[0], 2.0, "after a take it reads again: 1.1, 1.2");
        assert_eq!(out[3], 0.0);
    }

    #[test]
    fn a_full_queue_drops_by_the_grid_rule_to_the_last_index() {
        // W2bV finding 4: the full queue's skip took `floor((t - origin) /
        // interval)`, which lands one index low where the quotient rounds
        // down (origin 0.37 ms, interval 0.25 ms, t the grid's own fifth
        // time: 4.999..), so the grid time at `t` was neither read nor
        // counted. The skip ends where the grid rule says, `origin + k *
        // interval` against `t`, as the reading loop does.
        let mut rng = 0x9e3779b97f4a7c15u64;
        let mut next = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            rng
        };
        let mut low = 0;
        for _ in 0..20000 {
            let origin = [0.0, 0.37e-3, -1e-3, 5e-4, 1.0 / 3.0][(next() % 5) as usize];
            let interval = [1e-4, 2.5e-4, 1e-3, 3.3e-5, 1.0 / 60.0, 1e-7][(next() % 6) as usize];
            let k = 1 + next() % 5000;
            let t = origin + k as f64 * interval;
            low += usize::from(((t - origin) / interval).floor() < k as f64);
            for inclusive in [true, false] {
                let mut o = Observer::new(vec![0], origin, interval, 1, origin);
                o.read_until(t, inclusive, |_, t| Some((t, 0.0, false)));
                let want = if inclusive { k + 1 } else { k };
                assert_eq!(o.next, want, "origin {origin} interval {interval} k {k} inclusive {inclusive}");
                assert_eq!(o.dropped, want - 1, "every grid time after the kept one counted");
            }
        }
        assert!(low > 0, "some quotients round below their index");
    }

    #[test]
    fn a_tiny_interval_neither_overflows_nor_reads_forever() {
        // `as u64 + 1` overflowed where the quotient passed u64's range.
        for interval in [1e-300, 1e-30, 5e-324] {
            let mut o = Observer::new(vec![0], 0.0, interval, 1, 0.0);
            o.read_until(1.0, true, |_, t| Some((t, 0.0, false)));
            let mut out = Vec::new();
            o.take(&mut out);
            assert_eq!(out[0], 1.0);
            assert!(out[3] > 0.0, "the rest counted as dropped");
            o.read_until(2.0, true, |_, t| Some((t, 0.0, false)));
            o.take(&mut out);
            assert_eq!(out[0], 1.0, "{interval}: one more read, the limit's");
        }
    }

    #[test]
    fn a_fallback_is_counted_per_node_and_rewound() {
        let mut o = Observer::new(vec![1, 2], 0.0, 1.0, DEFAULT_LIMIT, 0.0);
        o.read_until(1.0, true, |j, _| Some((0.0, 0.0, j == 1)));
        let m = o.mark();
        o.read_until(3.0, true, |j, _| Some((0.0, 0.0, j == 1)));
        o.rewind(m);
        let mut out = Vec::new();
        o.take(&mut out);
        assert_eq!(&out[..6], &[2.0, 0.0, 0.0, 0.0, 0.0, 2.0], "node 2 fell back at 0 and 1");
        o.read_until(2.0, true, |_, _| Some((0.0, 0.0, false)));
        o.take(&mut out);
        assert_eq!(&out[4..6], &[0.0, 0.0], "a take clears them");
    }

    #[test]
    fn a_rewind_forgets_what_a_rolled_back_instant_read() {
        let mut o = Observer::new(vec![0], 0.0, 1.0, DEFAULT_LIMIT, 0.0);
        o.read_until(2.0, true, |_, t| Some((t, 0.0, false)));
        let m = o.mark();
        o.read_until(5.0, true, |_, t| Some((t, 1.0, false)));
        o.rewind(m);
        let mut out = Vec::new();
        o.take(&mut out);
        assert_eq!(&out[..], &[3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 1.0, 2.0]);
        assert_eq!(o.next_time(), 3.0);
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Observer {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nodes, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.origin, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.interval, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.next, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.times, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.values, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.limit, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dropped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.max_span, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.fallbacks, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.row, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.row_fallback, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Mark {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.len, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.next, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dropped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.max_span, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.fallbacks, w); }
        n
    }
}
