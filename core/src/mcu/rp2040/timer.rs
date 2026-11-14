//! The system timer (four alarms and their interrupts) and the watchdog,
//! ported from rp2040js 1.3.3 (MIT): `src/peripherals/timer.ts` and
//! `src/peripherals/watchdog.ts`. The watchdog counts with `Timer32` and a
//! periodic alarm (`src/utils/timer32.ts`, in `clock.rs`).

use super::clock::{js_to_u32, AlarmId, PeriodicAlarm, Timer32, TimerMode};
use super::Rp2040;

const TIMEHR: u32 = 0x08;
const TIMELR: u32 = 0x0c;
const TIMERAWH: u32 = 0x24;
const TIMERAWL: u32 = 0x28;
const ALARM0: u32 = 0x10;
const ALARM3: u32 = 0x1c;
const ARMED: u32 = 0x20;
const PAUSE: u32 = 0x30;
const INTR: u32 = 0x34;
const INTE: u32 = 0x38;
const INTF: u32 = 0x3c;
const INTS: u32 = 0x40;

/// `IRQ.TIMER_0`: the four alarms' interrupts are IRQs 0 to 3.
const IRQ_TIMER_0: u32 = 0;

#[derive(Clone, Copy, Default)]
pub struct TimerAlarm {
    pub armed: bool,
    pub target_micros: u32,
}

pub struct Timer {
    pub latched_time_high: u32,
    pub alarms: [TimerAlarm; 4],
    pub int_raw: u32,
    pub int_enable: u32,
    pub int_force: u32,
    pub paused: bool,
}

impl Default for Timer {
    fn default() -> Self {
        Self::new()
    }
}

impl Timer {
    pub fn new() -> Self {
        Timer {
            latched_time_high: 0,
            alarms: [TimerAlarm::default(); 4],
            int_raw: 0,
            int_enable: 0,
            int_force: 0,
            paused: false,
        }
    }

    pub fn int_status(&self) -> u32 {
        (self.int_raw & self.int_enable) | self.int_force
    }
}

fn two_pow_32() -> f64 {
    4294967296.0
}

impl Rp2040 {
    /// `RPTimer.readUint32`; `None` falls to the base's warning.
    pub(crate) fn timer_read(&mut self, offset: u32) -> u32 {
        let time = self.clock.nanos / 1000.0;
        let t = &mut self.periph.timer;
        match offset {
            TIMEHR => t.latched_time_high,
            TIMELR => {
                t.latched_time_high = js_to_u32((time / two_pow_32()).floor());
                js_to_u32(time)
            }
            TIMERAWH => js_to_u32((time / two_pow_32()).floor()),
            TIMERAWL => js_to_u32(time),
            ALARM0..=ALARM3 if offset & 3 == 0 => t.alarms[((offset - ALARM0) / 4) as usize].target_micros,
            PAUSE => t.paused as u32,
            INTR => t.int_raw,
            INTE => t.int_enable,
            INTF => t.int_force,
            INTS => t.int_status(),
            ARMED => {
                let mut v = 0;
                for (i, a) in t.alarms.iter().enumerate() {
                    if a.armed {
                        v |= 1 << i;
                    }
                }
                v
            }
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
                0xffff_ffff
            }
        }
    }

    pub(crate) fn timer_write(&mut self, offset: u32, value: u32, raw: u32) {
        match offset {
            ALARM0..=ALARM3 if offset & 3 == 0 => {
                let index = ((offset - ALARM0) / 4) as usize;
                // A negative int32 (see `Periph::write_signed`) truncates toward zero
                // before the modulo, one microsecond apart from its unsigned twin.
                let js_value = if self.periph.write_signed { value as i32 as f64 } else { value as f64 };
                let delta_micros = js_to_u32(js_value - self.clock.nanos / 1000.0);
                let a = &mut self.periph.timer.alarms[index];
                a.armed = true;
                a.target_micros = value;
                self.clock.schedule(AlarmId::Timer(index as u8), delta_micros as f64 * 1000.0);
            }
            ARMED => {
                for i in 0..4 {
                    if raw & (1 << i) != 0 {
                        self.timer_disarm(i);
                    }
                }
            }
            PAUSE => {
                self.periph.timer.paused = value & 1 != 0;
                // The reference warns "Unimplemented Timer Pause" and does
                // not pause the timer.
                if self.periph.timer.paused {
                    self.warnings += 1;
                }
            }
            INTR => {
                self.periph.timer.int_raw &= !raw;
                self.timer_check_interrupts();
            }
            INTE => {
                self.periph.timer.int_enable = value & 0xf;
                self.timer_check_interrupts();
            }
            INTF => {
                self.periph.timer.int_force = value & 0xf;
                self.timer_check_interrupts();
            }
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
            }
        }
    }

    /// `fireAlarm`.
    pub(crate) fn timer_alarm_fired(&mut self, index: usize) {
        self.timer_disarm(index);
        self.periph.timer.int_raw |= 1 << index;
        self.timer_check_interrupts();
    }

    fn timer_check_interrupts(&mut self) {
        let status = self.periph.timer.int_status();
        for i in 0..4 {
            self.set_interrupt(IRQ_TIMER_0 + i, status & (1 << i) != 0);
        }
    }

    fn timer_disarm(&mut self, index: usize) {
        self.clock.cancel(AlarmId::Timer(index as u8));
        self.periph.timer.alarms[index].armed = false;
    }
}

// ---- watchdog ---------------------------------------------------------------

const WD_CTRL: u32 = 0x00;
const WD_LOAD: u32 = 0x04;
const WD_REASON: u32 = 0x08;
const WD_SCRATCH0: u32 = 0x0c;
const WD_SCRATCH7: u32 = 0x28;
const WD_TICK: u32 = 0x2c;

const WD_TRIGGER: u32 = 1 << 31;
const WD_ENABLE: u32 = 1 << 30;
const WD_PAUSE_DBG1: u32 = 1 << 26;
const WD_PAUSE_DBG0: u32 = 1 << 25;
const WD_PAUSE_JTAG: u32 = 1 << 24;
const WD_TIME_MASK: u32 = 0xff_ffff;
const WD_FORCE: u32 = 1 << 1;
const WD_TIMER: u32 = 1 << 0;
const WD_RUNNING: u32 = 1 << 10;
const WD_TICK_ENABLE: u32 = 1 << 9;

/// Actually 1 MHz, but due to errata RP2040-E1 the timer is decremented
/// twice per tick (the reference's comment).
const TICK_FREQUENCY: f64 = 2_000_000.0;

pub struct Watchdog {
    pub timer: Timer32,
    pub alarm: PeriodicAlarm,
    pub scratch: [u32; 8],
    pub enable: bool,
    pub tick_enable: bool,
    pub reason: u32,
    pub pause_dbg0: bool,
    pub pause_dbg1: bool,
    pub pause_jtag: bool,
    /// Times `onWatchdogTrigger` ran. The reference's default handler only
    /// logs a warning; a host that wants a soft reset reads this.
    pub triggers: u32,
}

impl Default for Watchdog {
    fn default() -> Self {
        Self::new()
    }
}

impl Watchdog {
    /// The constructor: a decrementing 2 MHz timer, stopped, and a disabled
    /// alarm at target 0. Nothing listens to the timer while it is built.
    pub fn new() -> Self {
        let mut timer = Timer32::new(TICK_FREQUENCY);
        timer.set_mode(TimerMode::Decrement, 0.0);
        timer.set_enable(false, 0.0);
        Watchdog {
            timer,
            alarm: PeriodicAlarm { target_value: 0.0, enabled: false },
            scratch: [0; 8],
            enable: false,
            tick_enable: true,
            reason: 0,
            pause_dbg0: true,
            pause_dbg1: true,
            pause_jtag: true,
            triggers: 0,
        }
    }
}

impl Rp2040 {
    fn wd_schedule(&mut self) {
        self.wd_schedule_guarded(false);
    }

    /// `Timer32PeriodicAlarm.schedule`. `after_fire` marks the call from
    /// `handleAlarm`, where one deliberate departure applies: with the
    /// counter exactly on its target (the usual case at the instant the
    /// alarm fires) the reference computes a zero delay, links the alarm at
    /// the current time, and `SimulationClock.tick` fires it again for ever.
    /// The port waits a full 2^32-tick period instead, the next time the
    /// counter passes the target.
    fn wd_schedule_guarded(&mut self, after_fire: bool) {
        let now = self.clock.nanos;
        let wd = &self.periph.watchdog;
        if let Some(mut d) = wd.alarm.delay_nanos(&wd.timer, now) {
            if after_fire && d == 0.0 {
                d = wd.timer.to_nanos(4294967296.0);
            }
            self.clock.schedule(AlarmId::Watchdog, d);
        }
    }

    /// The timer's listener: `Timer32PeriodicAlarm.update`.
    fn wd_timer_updated(&mut self) {
        self.clock.cancel(AlarmId::Watchdog);
        let wd = &self.periph.watchdog;
        if wd.alarm.enabled && wd.timer.enabled {
            self.wd_schedule();
        }
    }

    /// `timer.enable = value`.
    fn wd_set_timer_enable(&mut self, value: bool) {
        let now = self.clock.nanos;
        if self.periph.watchdog.timer.set_enable(value, now) {
            self.wd_timer_updated();
        }
    }

    /// `alarm.enable = value`.
    fn wd_set_alarm_enable(&mut self, value: bool) {
        if value != self.periph.watchdog.alarm.enabled {
            self.periph.watchdog.alarm.enabled = value;
            if value && self.periph.watchdog.timer.enabled {
                self.wd_schedule();
            } else {
                self.clock.cancel(AlarmId::Watchdog);
            }
        }
    }

    /// The alarm's `handleAlarm`: the trigger, then the next period.
    pub(crate) fn watchdog_alarm_fired(&mut self) {
        self.periph.watchdog.reason = WD_TIMER;
        self.periph.watchdog.triggers += 1;
        let wd = &self.periph.watchdog;
        if wd.alarm.enabled && wd.timer.enabled {
            self.wd_schedule_guarded(true);
        }
    }

    pub(crate) fn watchdog_read(&mut self, offset: u32) -> u32 {
        let now = self.clock.nanos;
        let wd = &self.periph.watchdog;
        match offset {
            WD_CTRL => {
                (if wd.timer.enabled { WD_ENABLE } else { 0 })
                    | (if wd.pause_dbg0 { WD_PAUSE_DBG0 } else { 0 })
                    | (if wd.pause_dbg1 { WD_PAUSE_DBG1 } else { 0 })
                    | (if wd.pause_jtag { WD_PAUSE_JTAG } else { 0 })
                    | (wd.timer.counter(now) & WD_TIME_MASK)
            }
            WD_REASON => wd.reason,
            WD_SCRATCH0..=WD_SCRATCH7 if offset & 3 == 0 => wd.scratch[((offset - WD_SCRATCH0) >> 2) as usize],
            // TODO COUNT bits (the reference's)
            WD_TICK => {
                if wd.tick_enable {
                    WD_RUNNING | WD_TICK_ENABLE
                } else {
                    0
                }
            }
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
                0xffff_ffff
            }
        }
    }

    pub(crate) fn watchdog_write(&mut self, offset: u32, value: u32) {
        match offset {
            WD_CTRL => {
                if value & WD_TRIGGER != 0 {
                    self.periph.watchdog.reason = WD_FORCE;
                    self.periph.watchdog.triggers += 1;
                }
                let enable = value & WD_ENABLE != 0;
                self.periph.watchdog.enable = enable;
                let on = enable && self.periph.watchdog.tick_enable;
                self.wd_set_timer_enable(on);
                self.wd_set_alarm_enable(on);
                let wd = &mut self.periph.watchdog;
                wd.pause_dbg0 = value & WD_PAUSE_DBG0 != 0;
                wd.pause_dbg1 = value & WD_PAUSE_DBG1 != 0;
                wd.pause_jtag = value & WD_PAUSE_JTAG != 0;
            }
            WD_LOAD => {
                let now = self.clock.nanos;
                self.periph.watchdog.timer.set((value & WD_TIME_MASK) as f64, now);
                self.wd_timer_updated();
            }
            WD_SCRATCH0..=WD_SCRATCH7 if offset & 3 == 0 => {
                self.periph.watchdog.scratch[((offset - WD_SCRATCH0) >> 2) as usize] = value;
            }
            WD_TICK => {
                self.periph.watchdog.tick_enable = value & WD_TICK_ENABLE != 0;
                // TODO - handle CYCLES (the reference's)
                let on = self.periph.watchdog.enable && self.periph.watchdog.tick_enable;
                self.wd_set_timer_enable(on);
                self.wd_set_alarm_enable(on);
            }
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
            }
        }
    }
}
