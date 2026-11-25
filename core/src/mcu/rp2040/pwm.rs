//! The PWM block (eight slices), ported from rp2040js 1.3.3 (MIT):
//! `src/peripherals/pwm.ts`. Each slice counts with a `Timer32` and three
//! `Timer32PeriodicAlarm`s (compare A, compare B, wrap); the pin outputs are
//! `gpioValue`/`gpioDirection`, which the pin state reads for function 4.
//!
//! Reference quirks kept (each has a `reference_quirk_*` test):
//! - `EN` reads as zero (the channels' `en` is a setter-only accessor).
//! - CSR.PH_ADV and PH_RET never act (they are masked off before they are tested).
//! - `Timer32.prescaler` re-enables a stopped timer.
//! - `gpioOnInput` tests `gpioDirection && 1 << index`, which is true whenever
//!   any direction bit is set, so a B-pin input never reaches the counter.
//! - `gpioDirection` starts as the double 0xffffffff, and the first
//!   `gpioSetDir` compares it with the int32 -1: GPIO1 re-evaluates itself.

use super::clock::{AlarmId, PeriodicAlarm, Timer32, TimerMode};
use super::dma::dreq;
use super::Rp2040;

const CLK_SYS: f64 = 125_000_000.0;
const IRQ_PWM_WRAP: u32 = 4;

const CH_CSR: u32 = 0x00;
const CH_DIV: u32 = 0x04;
const CH_CTR: u32 = 0x08;
const CH_CC: u32 = 0x0c;
const CH_TOP: u32 = 0x10;
const CH_STRIDE: u32 = 0x14;
const EN: u32 = 0xa0;
const INTR: u32 = 0xa4;
const INTE: u32 = 0xa8;
const INTF: u32 = 0xac;
const INTS: u32 = 0xb0;
const INT_MASK: u32 = 0xff;

const CSR_PH_ADV: u32 = 1 << 7;
const CSR_PH_RET: u32 = 1 << 6;
const CSR_DIVMODE_SHIFT: u32 = 4;
const CSR_B_INV: u32 = 1 << 3;
const CSR_A_INV: u32 = 1 << 2;
const CSR_PH_CORRECT: u32 = 1 << 1;
const CSR_EN: u32 = 1 << 0;

const DIV_FREE_RUNNING: u32 = 0;
const DIV_B_GATED: u32 = 1;
const DIV_B_RISING: u32 = 2;
const DIV_B_FALLING: u32 = 3;

pub struct PwmChannel {
    pub timer: Timer32,
    /// Compare A, compare B, wrap.
    pub alarm: [PeriodicAlarm; 3],
    pub csr: u32,
    pub div: u32,
    pub cc: u32,
    pub top: u32,
    pub last_b_value: bool,
    pub counting_up: bool,
    pub cc_updated: bool,
    pub top_updated: bool,
    pub tick_counter: f64,
    pub div_mode: u32,
}

impl PwmChannel {
    fn new() -> Self {
        PwmChannel {
            timer: Timer32::new(CLK_SYS),
            alarm: [0; 3].map(|_| PeriodicAlarm { target_value: 0.0, enabled: true }),
            csr: 0,
            div: 0,
            cc: 0,
            top: 0,
            last_b_value: false,
            counting_up: true,
            cc_updated: false,
            top_updated: false,
            tick_counter: 0.0,
            div_mode: DIV_FREE_RUNNING,
        }
    }
}

fn pin_a1(i: usize) -> usize {
    i * 2
}
fn pin_b1(i: usize) -> usize {
    i * 2 + 1
}
fn pin_a2(i: usize) -> Option<usize> {
    if i < 7 { Some(16 + i * 2) } else { None }
}
fn pin_b2(i: usize) -> Option<usize> {
    if i < 7 { Some(16 + i * 2 + 1) } else { None }
}

pub struct Pwm {
    pub channels: [PwmChannel; 8],
    pub int_raw: u32,
    pub int_enable: u32,
    pub int_force: u32,
    pub gpio_value: u32,
    /// A JavaScript number: 0xffffffff (as a double) after `reset()`, an
    /// int32 once `gpioSetDir` has run.
    pub gpio_direction: i64,
}

impl Default for Pwm {
    fn default() -> Self {
        Self::new()
    }
}

impl Pwm {
    /// The constructor's field state. The alarms the reference links while it
    /// constructs (each channel's three, at time 0) are cancelled by the
    /// first `reset()`, so they are not modelled.
    pub fn new() -> Self {
        Pwm {
            channels: [(); 8].map(|_| PwmChannel::new()),
            int_raw: 0,
            int_enable: 0,
            int_force: 0,
            gpio_value: 0,
            gpio_direction: 0,
        }
    }

    pub fn int_status(&self) -> u32 {
        (self.int_raw & self.int_enable) | self.int_force
    }
}

impl Rp2040 {
    fn pwm_id(ch: usize, k: usize) -> AlarmId {
        AlarmId::Pwm((ch * 3 + k) as u8)
    }

    /// `Timer32PeriodicAlarm.schedule`.
    fn pwm_schedule(&mut self, ch: usize, k: usize) {
        let now = self.clock.nanos;
        let c = &self.periph.pwm.channels[ch];
        if let Some(d) = c.alarm[k].delay_nanos(&c.timer, now) {
            self.clock.schedule(Self::pwm_id(ch, k), d);
        }
    }

    /// The timer's listeners, in registration order: A, B, wrap.
    fn pwm_timer_updated(&mut self, ch: usize) {
        for k in 0..3 {
            self.clock.cancel(Self::pwm_id(ch, k));
            let c = &self.periph.pwm.channels[ch];
            if c.alarm[k].enabled && c.timer.enabled {
                self.pwm_schedule(ch, k);
            }
        }
    }

    /// `alarm.target = value`.
    fn pwm_set_target(&mut self, ch: usize, k: usize, value: f64) {
        let c = &mut self.periph.pwm.channels[ch];
        if c.alarm[k].target_value == value {
            return;
        }
        c.alarm[k].target_value = value;
        if c.alarm[k].enabled && c.timer.enabled {
            self.clock.cancel(Self::pwm_id(ch, k));
            self.pwm_schedule(ch, k);
        }
    }

    fn pwm_set_enable(&mut self, ch: usize, value: bool) {
        let now = self.clock.nanos;
        if self.periph.pwm.channels[ch].timer.set_enable(value, now) {
            self.pwm_timer_updated(ch);
        }
    }

    fn pwm_gpio_read(&self, index: usize) -> bool {
        self.pin_input_value(index)
    }

    fn pwm_gpio_b_value(&self, ch: usize) -> bool {
        self.pwm_gpio_read(pin_b1(ch)) || pin_b2(ch).map_or(false, |p| self.pwm_gpio_read(p))
    }

    fn pwm_update_enable(&mut self, ch: usize) {
        let c = &self.periph.pwm.channels[ch];
        let enable = c.csr & CSR_EN != 0;
        let on = enable
            && (c.div_mode == DIV_FREE_RUNNING || (c.div_mode == DIV_B_GATED && self.pwm_gpio_b_value(ch)));
        self.pwm_set_enable(ch, on);
    }

    fn pwm_update_double_buffered(&mut self, ch: usize) {
        if self.periph.pwm.channels[ch].cc_updated {
            let cc = self.periph.pwm.channels[ch].cc;
            self.pwm_set_target(ch, 1, (cc >> 16) as f64);
            self.pwm_set_target(ch, 0, (cc & 0xffff) as f64);
            self.periph.pwm.channels[ch].cc_updated = false;
        }
        if self.periph.pwm.channels[ch].top_updated {
            let now = self.clock.nanos;
            let top = self.periph.pwm.channels[ch].top as f64;
            self.periph.pwm.channels[ch].timer.set_top(top, now);
            self.pwm_timer_updated(ch);
            self.periph.pwm.channels[ch].top_updated = false;
        }
    }

    fn pwm_check_interrupts(&mut self) {
        let s = self.periph.pwm.int_status() != 0;
        self.set_interrupt(IRQ_PWM_WRAP, s);
    }

    fn pwm_channel_interrupt(&mut self, ch: usize) {
        self.periph.pwm.int_raw |= 1 << ch;
        self.pwm_check_interrupts();
        self.dma_set_dreq(dreq::PWM_WRAP0 + ch as u32);
    }

    fn pwm_wrap(&mut self, ch: usize) {
        self.pwm_channel_interrupt(ch);
        self.pwm_update_double_buffered(ch);
        let c = &self.periph.pwm.channels[ch];
        if c.csr & CSR_PH_CORRECT == 0 {
            let a = c.alarm[0].target_value > 0.0;
            let b = c.alarm[1].target_value > 0.0;
            self.pwm_set_a(ch, a);
            self.pwm_set_b(ch, b);
        }
    }

    fn pwm_gpio_set(&mut self, index: usize, value: bool) {
        let bit = 1u32 << index;
        let pwm = &mut self.periph.pwm;
        let new = if value { pwm.gpio_value | bit } else { pwm.gpio_value & !bit };
        if pwm.gpio_value != new {
            pwm.gpio_value = new;
            self.gpio_check_for_updates(index);
        }
    }

    fn pwm_gpio_set_dir(&mut self, index: usize, output: bool) {
        let bit = 1i32 << index;
        let pwm = &mut self.periph.pwm;
        let cur = pwm.gpio_direction as u32 as i32;
        let new = (if output { cur | bit } else { cur & !bit }) as i64;
        if pwm.gpio_direction != new {
            pwm.gpio_direction = new;
            self.gpio_check_for_updates(index);
        }
    }

    pub(crate) fn pwm_set_a(&mut self, ch: usize, mut value: bool) {
        if self.periph.pwm.channels[ch].csr & CSR_A_INV != 0 {
            value = !value;
        }
        self.pwm_gpio_set(pin_a1(ch), value);
        if let Some(p) = pin_a2(ch) {
            self.pwm_gpio_set(p, value);
        }
    }

    pub(crate) fn pwm_set_b(&mut self, ch: usize, mut value: bool) {
        if self.periph.pwm.channels[ch].csr & CSR_B_INV != 0 {
            value = !value;
        }
        self.pwm_gpio_set(pin_b1(ch), value);
        if let Some(p) = pin_b2(ch) {
            self.pwm_gpio_set(p, value);
        }
    }

    fn pwm_set_b_direction(&mut self, ch: usize, value: bool) {
        self.pwm_gpio_set_dir(pin_b1(ch), value);
        if let Some(p) = pin_b2(ch) {
            self.pwm_gpio_set_dir(p, value);
        }
    }

    /// `gpioBChanged`.
    fn pwm_gpio_b_changed(&mut self, ch: usize) {
        let value = self.pwm_gpio_b_value(ch);
        if value == self.periph.pwm.channels[ch].last_b_value {
            return;
        }
        let c = &mut self.periph.pwm.channels[ch];
        c.last_b_value = value;
        match c.div_mode {
            DIV_B_GATED => self.pwm_update_enable(ch),
            DIV_B_RISING => {
                if value {
                    c.tick_counter += 1.0;
                }
            }
            DIV_B_FALLING => {
                if !value {
                    c.tick_counter += 1.0;
                }
            }
            _ => {}
        }
        let c = &mut self.periph.pwm.channels[ch];
        if c.tick_counter >= c.timer.prescaler_value {
            c.timer.advance(1.0);
            c.tick_counter -= c.timer.prescaler_value;
        }
    }

    /// `gpioOnInput`, run from `GPIOPin.setInputValue` for a pin on function 4.
    pub(crate) fn pwm_gpio_on_input(&mut self, index: usize) {
        // Reference quirk: `this.gpioDirection && 1 << index` is a truthiness
        // test on the whole direction word.
        if self.periph.pwm.gpio_direction != 0 {
            return;
        }
        for ch in 0..8 {
            if pin_b1(ch) == index || pin_b2(ch) == Some(index) {
                self.pwm_gpio_b_changed(ch);
            }
        }
    }

    fn pwm_channel_write(&mut self, ch: usize, offset: u32, value: u32) {
        let now = self.clock.nanos;
        match offset {
            CH_CSR => {
                if value & CSR_EN != 0 && self.periph.pwm.channels[ch].csr & CSR_EN == 0 {
                    self.pwm_update_double_buffered(ch);
                }
                let c = &mut self.periph.pwm.channels[ch];
                c.csr = value & !(CSR_PH_ADV | CSR_PH_RET);
                // PH_ADV / PH_RET are tested after being cleared: dead code.
                c.div_mode = (c.csr >> CSR_DIVMODE_SHIFT) & 3;
                let free = c.div_mode == DIV_FREE_RUNNING;
                self.pwm_set_b_direction(ch, free);
                self.pwm_update_enable(ch);
                let b = self.pwm_gpio_b_value(ch);
                let c = &mut self.periph.pwm.channels[ch];
                c.last_b_value = b;
                let mode = if value & CSR_PH_CORRECT != 0 { TimerMode::ZigZag } else { TimerMode::Increment };
                if c.timer.set_mode(mode, now) {
                    self.pwm_timer_updated(ch);
                }
            }
            CH_DIV => {
                let c = &mut self.periph.pwm.channels[ch];
                c.div = value & 0x000f_ffff;
                let int = (value >> 4) & 0xff;
                let frac = value & 0xf;
                let p = (if int != 0 { int } else { 256 }) as f64 + frac as f64 / 16.0;
                c.timer.set_prescaler(p, now);
                self.pwm_timer_updated(ch);
            }
            CH_CTR => {
                self.periph.pwm.channels[ch].timer.set((value & 0xffff) as f64, now);
                self.pwm_timer_updated(ch);
            }
            CH_CC => {
                let c = &mut self.periph.pwm.channels[ch];
                c.cc = value;
                c.cc_updated = true;
            }
            CH_TOP => {
                let c = &mut self.periph.pwm.channels[ch];
                c.top = value & 0xffff;
                c.top_updated = true;
            }
            _ => {}
        }
    }

    fn pwm_channel_reset(&mut self, ch: usize) {
        self.pwm_channel_write(ch, CH_CSR, 0);
        self.pwm_channel_write(ch, CH_DIV, 0x01 << 4);
        self.pwm_channel_write(ch, CH_CTR, 0);
        self.pwm_channel_write(ch, CH_CC, 0);
        self.pwm_channel_write(ch, CH_TOP, 0xffff);
        self.periph.pwm.channels[ch].counting_up = true;
        self.pwm_set_enable(ch, false);
        let now = self.clock.nanos;
        self.periph.pwm.channels[ch].timer.reset(now);
        self.pwm_timer_updated(ch);
    }

    /// `RPPWM.reset()`.
    pub fn pwm_reset(&mut self) {
        self.periph.pwm.gpio_direction = 0xffff_ffff;
        for ch in 0..8 {
            self.pwm_channel_reset(ch);
        }
    }

    /// `PWMChannel.en = value` (a setter only).
    fn pwm_set_en(&mut self, ch: usize, value: bool) {
        if value && self.periph.pwm.channels[ch].csr & CSR_EN == 0 {
            self.pwm_update_double_buffered(ch);
        }
        let c = &mut self.periph.pwm.channels[ch];
        if value {
            c.csr |= CSR_EN;
        } else {
            c.csr &= !CSR_EN;
        }
        self.pwm_update_enable(ch);
    }

    /// An alarm of slice `ch` fired: `handleAlarm`.
    pub(crate) fn pwm_alarm_fired(&mut self, id: usize) {
        let (ch, k) = (id / 3, id % 3);
        match k {
            0 => self.pwm_set_a(ch, false),
            1 => self.pwm_set_b(ch, false),
            _ => self.pwm_wrap(ch),
        }
        let c = &self.periph.pwm.channels[ch];
        if c.alarm[k].enabled && c.timer.enabled {
            self.pwm_schedule(ch, k);
        }
    }

    pub(crate) fn pwm_read(&mut self, offset: u32) -> Option<u32> {
        if offset < EN {
            let ch = (offset / CH_STRIDE) as usize;
            let c = &self.periph.pwm.channels[ch];
            return Some(match offset % CH_STRIDE {
                CH_CSR => c.csr,
                CH_DIV => c.div,
                CH_CTR => c.timer.counter(self.clock.nanos),
                CH_CC => c.cc,
                CH_TOP => c.top,
                _ => 0,
            });
        }
        match offset {
            // Reference quirk: `en` has no getter, so every channel adds
            // `undefined << n` = 0.
            EN => Some(0),
            INTR => Some(self.periph.pwm.int_raw),
            INTE => Some(self.periph.pwm.int_enable),
            INTF => Some(self.periph.pwm.int_force),
            INTS => Some(self.periph.pwm.int_status()),
            _ => None,
        }
    }

    /// `false` when the register is unmodelled (a warning).
    pub(crate) fn pwm_write(&mut self, offset: u32, value: u32) -> bool {
        if offset < EN {
            self.pwm_channel_write((offset / CH_STRIDE) as usize, offset % CH_STRIDE, value);
            return true;
        }
        match offset {
            EN => {
                for ch in (0..8).rev() {
                    self.pwm_set_en(ch, value & (1 << ch) != 0);
                }
            }
            INTR => {
                self.periph.pwm.int_raw &= !(value & INT_MASK);
                self.pwm_check_interrupts();
            }
            INTE => {
                self.periph.pwm.int_enable = value & INT_MASK;
                self.pwm_check_interrupts();
            }
            INTF => {
                self.periph.pwm.int_force = value & INT_MASK;
                self.pwm_check_interrupts();
            }
            _ => return false,
        }
        true
    }
}
