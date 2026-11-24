//! DHT22 (AM2302) and DHT11 sensors on a board pin, ported from brownout's
//! `src/sim/engine/dht.ts` (`DhtSensor`), which the lock-step harness holds
//! this to. The host pulls the line low to ask for a reading and releases
//! it; once the start was long enough, the sensor answers 30 us later with
//! 80 us low, 80 us high, then 40 bits (50 us low, then 27 us high for a 0 or
//! 70 us for a 1) and a final 50 us low (AM2302 section 7, DHT11 section 5).
//! The whole answer is scheduled as clock events the moment the host lets
//! go, as the reference schedules it, so both engines hold the same events.

use super::parts::js_round;

const RESPONSE_DELAY_US: f64 = 30.0;
const RESPONSE_LOW_US: f64 = 80.0;
const RESPONSE_HIGH_US: f64 = 80.0;
const BIT_LOW_US: f64 = 50.0;
const ZERO_HIGH_US: f64 = 27.0;
const ONE_HIGH_US: f64 = 70.0;
const END_LOW_US: f64 = 50.0;
/// "After power-on the sensor needs 1 s to pass its unstable state."
const POWER_UP_S: f64 = 1.0;
/// Edges in an answer: response low and high, two a bit, and the end.
pub const ANSWER_EDGES: usize = 2 + 40 * 2 + 2;

/// The shortest host start each part answers, in us (AM2302 0.8 ms, DHT11 18 ms).
fn start_min_us(variant: u8) -> f64 {
    if variant == 11 { 18000.0 } else { 800.0 }
}

/// The five bytes a reading sends, clamped to the part's range.
pub fn frame(variant: u8, humidity: f64, celsius: f64) -> [u8; 5] {
    let mut b = [0u8; 5];
    if variant == 22 {
        let rh = js_round(humidity.clamp(0.0, 99.9) * 10.0) as i32;
        let t = js_round(celsius.clamp(-40.0, 80.0) * 10.0) as i32;
        let t_word = if t < 0 { 0x8000 | -t } else { t };
        b[..4].copy_from_slice(&[(rh >> 8) as u8, rh as u8, (t_word >> 8) as u8, t_word as u8]);
    } else {
        let rh = js_round(humidity.clamp(20.0, 90.0)) as i32;
        let t = celsius.clamp(0.0, 50.0);
        let whole = t.floor();
        b[..4].copy_from_slice(&[rh as u8, 0, whole as u8, (js_round((t - whole) * 10.0) as i32 % 10) as u8]);
    }
    b[4] = b[0].wrapping_add(b[1]).wrapping_add(b[2]).wrapping_add(b[3]);
    b
}

/// What the sensor asks of the board's line.
pub enum DhtAction {
    None,
    /// Schedule the answer's edges: (cycles from now, pull low) each.
    Answer(Vec<(i64, bool)>),
}

#[derive(Clone, Debug)]
pub struct DhtSensor {
    pub variant: u8,
    pub powered: bool,
    /// Seconds powered; the host keeps it current.
    pub powered_for: f64,
    pub humidity: f64,
    pub celsius: f64,
    pub measured: Option<[u8; 5]>,
    pub started_low_at: Option<u64>,
    pub answering: bool,
    pub answer_span: (u64, u64),
    pub generation: u32,
}

impl DhtSensor {
    pub fn new(variant: u8) -> DhtSensor {
        DhtSensor {
            variant: if variant == 11 { 11 } else { 22 },
            powered: false,
            powered_for: 0.0,
            humidity: 50.0,
            celsius: 25.0,
            measured: None,
            started_low_at: None,
            answering: false,
            answer_span: (0, 0),
            generation: 0,
        }
    }

    pub fn set_ambient(&mut self, humidity: f64, celsius: f64) {
        self.humidity = humidity;
        self.celsius = celsius;
        if self.measured.is_none() {
            self.measured = Some(frame(self.variant, humidity, celsius));
        }
    }

    pub fn reset(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        self.started_low_at = None;
        self.answering = false;
        self.powered_for = 0.0;
        self.measured = None;
    }

    /// The board's drive on the line changed (`Some(false)` low, `Some(true)`
    /// high, `None` not driving) at board cycle `now`, on a `hz` clock.
    pub fn board_drive(&mut self, level: Option<bool>, now: u64, hz: f64) -> DhtAction {
        if self.answering && (now < self.answer_span.0 || now > self.answer_span.1) {
            self.answering = false;
        }
        if !self.powered || self.answering {
            return DhtAction::None;
        }
        if level == Some(false) {
            self.started_low_at = Some(now);
            return DhtAction::None;
        }
        let Some(began) = self.started_low_at.take() else { return DhtAction::None };
        let low_us = ((now - began) as f64 / hz) * 1e6;
        if low_us < start_min_us(self.variant) || self.powered_for < POWER_UP_S {
            return DhtAction::None;
        }
        self.answer(now, hz)
    }

    fn answer(&mut self, now: u64, hz: f64) -> DhtAction {
        self.answering = true;
        let f = self.measured.unwrap_or_else(|| frame(self.variant, self.humidity, self.celsius));
        let mut edges: Vec<(f64, bool)> = Vec::with_capacity(ANSWER_EDGES);
        let mut t = RESPONSE_DELAY_US;
        edges.push((t, true));
        t += RESPONSE_LOW_US;
        edges.push((t, false));
        t += RESPONSE_HIGH_US;
        for i in 0..40 {
            let bit = (f[i >> 3] >> (7 - (i & 7))) & 1;
            edges.push((t, true));
            t += BIT_LOW_US;
            edges.push((t, false));
            t += if bit == 1 { ONE_HIGH_US } else { ZERO_HIGH_US };
        }
        edges.push((t, true));
        t += END_LOW_US;
        edges.push((t, false));
        let cycles_at = |us: f64| js_round(us * hz / 1e6) as i64;
        self.answer_span = (now, now + cycles_at(t) as u64);
        DhtAction::Answer(edges.into_iter().map(|(us, low)| (cycles_at(us), low)).collect())
    }

    /// An answer edge fell due: whether to pull the line (and how), or
    /// `None` when the answer was cut off by a reset or lost power.
    pub fn edge_due(&mut self, generation: u32, index: usize, low: bool) -> Option<bool> {
        if generation != self.generation & 0xffff || !self.powered {
            return None;
        }
        if index == ANSWER_EDGES - 1 {
            self.answering = false;
            // The part measures after each reading; the next one reports it.
            self.measured = Some(frame(self.variant, self.humidity, self.celsius));
        }
        Some(low)
    }

    /// The state the lock-step harness compares, as words.
    pub fn state_words(&self, out: &mut Vec<u32>) {
        out.push(self.powered as u32);
        out.push(self.answering as u32);
        out.push(self.generation);
        match self.started_low_at {
            None => out.push(0),
            Some(c) => out.extend([1, c as u32]),
        }
        out.extend([self.answer_span.0 as u32, self.answer_span.1 as u32]);
        match self.measured {
            None => out.push(0),
            Some(m) => {
                out.push(1);
                out.extend(m.iter().map(|&b| b as u32));
            }
        }
    }
}
