//! The simulation clock and the 32-bit timer model, ported from rp2040js
//! 1.3.3 (MIT): `src/clock/simulation-clock.ts` and `src/utils/timer32.ts`.
//!
//! Time is an f64 nanosecond count and the timer arithmetic is f64, as in the
//! reference; every operation keeps the reference's order so the values agree
//! to the bit.

/// Every alarm a ported block can schedule. R1 has SysTick; R2 adds the
/// timer's four alarms and the watchdog's periodic alarm; later packages add
/// their own.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlarmId {
    Systick,
    Timer(u8),
    Watchdog,
    UsbRead(u8),
    UsbWrite(u8),
    UsbReset,
    UsbSof,
    /// A DMA channel's `transfer` alarm.
    Dma(u8),
    /// A PWM slice's alarm: `slice * 3 + k`, k = 0 compare A, 1 compare B,
    /// 2 the wrap ("bottom").
    Pwm(u8),
    /// The ADC's `sampleAlarm` (a conversion finishing).
    AdcSample,
    /// The ADC's `multiShotAlarm` (the next conversion of a multi-shot run).
    AdcMulti,
}

pub struct Clock {
    pub nanos: f64,
    /// The reference's linked list, earliest first. A new alarm goes before
    /// the first one due at or after it.
    pub alarms: Vec<(AlarmId, f64)>,
}

impl Default for Clock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock {
    pub fn new() -> Self {
        Clock { nanos: 0.0, alarms: Vec::new() }
    }

    /// `ClockAlarm.schedule`: drop any pending instance, then link.
    pub fn schedule(&mut self, id: AlarmId, delta_nanos: f64) {
        self.cancel(id);
        let at = self.nanos + delta_nanos;
        let pos = self.alarms.iter().position(|&(_, t)| !(t < at)).unwrap_or(self.alarms.len());
        self.alarms.insert(pos, (id, at));
    }

    pub fn cancel(&mut self, id: AlarmId) {
        if let Some(pos) = self.alarms.iter().position(|&(a, _)| a == id) {
            self.alarms.remove(pos);
        }
    }

    /// `nanosToNextAlarm`: 0 when nothing is scheduled.
    pub fn nanos_to_next_alarm(&self) -> f64 {
        match self.alarms.first() {
            Some(&(_, t)) => t - self.nanos,
            None => 0.0,
        }
    }
}

/// JavaScript's `Math.round`: nearest integer, halves toward +infinity.
pub fn js_round(x: f64) -> f64 {
    if !x.is_finite() {
        return x;
    }
    let f = x.floor();
    if x - f >= 0.5 {
        f + 1.0
    } else {
        f
    }
}

/// JavaScript's ToInt32 (`x | 0`).
pub fn js_to_i32(x: f64) -> i32 {
    js_to_u32(x) as i32
}

/// JavaScript's ToUint32 (`x >>> 0`).
pub fn js_to_u32(x: f64) -> u32 {
    if !x.is_finite() {
        return 0;
    }
    x.trunc().rem_euclid(4294967296.0) as u32
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerMode {
    Increment,
    Decrement,
    ZigZag,
}

/// `Timer32`. The listener list is the caller's job: every method that the
/// reference ends with `updated()` returns, and the owner then runs its
/// listeners (in R1 SysTick's periodic alarm is the only one).
pub struct Timer32 {
    pub base_value: f64,
    pub base_nanos: f64,
    pub top_value: f64,
    pub prescaler_value: f64,
    pub mode: TimerMode,
    pub enabled: bool,
    pub base_freq: f64,
}

impl Timer32 {
    pub fn new(base_freq: f64) -> Self {
        Timer32 {
            base_value: 0.0,
            base_nanos: 0.0,
            top_value: 4294967295.0,
            prescaler_value: 1.0,
            mode: TimerMode::Increment,
            enabled: true,
            base_freq,
        }
    }

    pub fn raw_counter(&self, now: f64) -> f64 {
        if self.base_freq == 0.0 || self.prescaler_value == 0.0 || !self.enabled {
            return self.base_value;
        }
        let zigzag = self.mode == TimerMode::ZigZag;
        let ticks = ((now - self.base_nanos) / 1e9) * (self.base_freq / self.prescaler_value);
        let top_modulo = if zigzag { self.top_value * 2.0 } else { self.top_value + 1.0 };
        let delta = if self.mode == TimerMode::Decrement { top_modulo - (ticks % top_modulo) } else { ticks };
        let mut current = js_round(self.base_value + delta);
        if self.top_value != 4294967295.0 {
            current %= top_modulo;
        }
        current
    }

    pub fn counter(&self, now: f64) -> u32 {
        let mut current = self.raw_counter(now);
        if self.mode == TimerMode::ZigZag && current > self.top_value {
            current = self.top_value * 2.0 - current;
        }
        js_to_u32(current)
    }

    /// `set(value)`; the caller runs the listeners.
    pub fn set(&mut self, value: f64, now: f64) {
        self.base_value = value;
        self.base_nanos = now;
    }

    /// `set top`; the caller runs the listeners.
    pub fn set_top(&mut self, value: f64, now: f64) {
        let counter = self.counter(now) as f64;
        self.top_value = value;
        self.set(if counter <= self.top_value { counter } else { 0.0 }, now);
    }

    /// `set mode`; returns whether the listeners must run.
    pub fn set_mode(&mut self, value: TimerMode, now: f64) -> bool {
        if self.mode != value {
            let counter = self.counter(now) as f64;
            self.mode = value;
            self.set(counter, now);
            true
        } else {
            false
        }
    }

    /// `set enable`; returns whether the listeners must run.
    pub fn set_enable(&mut self, value: bool, now: f64) -> bool {
        if value != self.enabled {
            if value {
                self.base_nanos = now;
            } else {
                self.base_value = self.counter(now) as f64;
            }
            self.enabled = value;
            true
        } else {
            false
        }
    }

    /// `set prescaler`: the reference re-bases the counter, then re-enables
    /// the timer when the OLD prescaler is non-zero (it assigns `enabled`
    /// before it stores the new value), so setting any prescaler on a
    /// stopped timer starts it. The caller runs the listeners.
    pub fn set_prescaler(&mut self, value: f64, now: f64) {
        self.base_value = self.counter(now) as f64;
        self.base_nanos = now;
        self.enabled = self.prescaler_value != 0.0;
        self.prescaler_value = value;
    }

    /// `advance`: moves the base value; no listeners.
    pub fn advance(&mut self, delta: f64) {
        self.base_value += delta;
    }

    /// `reset`; the caller runs the listeners.
    pub fn reset(&mut self, now: f64) {
        self.base_nanos = now;
        self.base_value = 0.0;
    }

    pub fn to_nanos(&self, cycles: f64) -> f64 {
        (cycles * 1e9) / (self.base_freq / self.prescaler_value)
    }
}

/// `Timer32PeriodicAlarm` state; scheduling lives with the owner, which
/// knows its alarm id.
pub struct PeriodicAlarm {
    pub target_value: f64,
    pub enabled: bool,
}

impl PeriodicAlarm {
    /// `schedule()`: the delay until the timer reaches the target, or `None`
    /// where the reference returns without linking.
    pub fn delay_nanos(&self, timer: &Timer32, now: f64) -> Option<f64> {
        let top = timer.top_value;
        let raw = timer.raw_counter(now);
        let target = self.target_value;
        let mut cycle_delta = target - raw;
        if timer.mode == TimerMode::ZigZag && cycle_delta < 0.0 {
            if cycle_delta < -top {
                cycle_delta += 2.0 * top;
            } else {
                cycle_delta = top * 2.0 - target - raw;
            }
        }
        if top != 4294967295.0 {
            if cycle_delta <= 0.0 {
                cycle_delta += top + 1.0;
            }
            if target > top {
                return None;
            }
        }
        if timer.mode == TimerMode::Decrement {
            cycle_delta = top + 1.0 - cycle_delta;
        }
        let cycles_to_alarm = js_to_u32(cycle_delta);
        Some(timer.to_nanos(cycles_to_alarm as f64))
    }
}
