//! 8- and 16-bit timers, ported from avr8js 0.21.0 `peripherals/timer.js`
//! (MIT, Copyright (c) Uri Shaked and contributors; see NOTICE): timers 0, 1
//! and 2 of the ATmega328P with their compare outputs, PWM modes, external
//! clock inputs and the 16-bit temporary register.
//!
//! The reference's `count` callback is the clock event `EV_TIMER + index`;
//! its register hooks are `Cpu` methods called from the hook tables.

use super::cpu::{Cpu, EV_TIMER};
use super::gpio::{port_by_port_address, PinOverride};
use super::interrupt::Interrupt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerMode {
    Normal,
    PwmPhaseCorrect,
    Ctc,
    FastPwm,
    PwmPhaseFrequencyCorrect,
    Reserved,
}
use TimerMode::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum TovUpdate {
    Max,
    Top,
    Bottom,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum OcrUpdate {
    Immediate,
    Top,
    Bottom,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Top {
    Const(i64),
    OcrA,
    Icr,
}

const OC_TOGGLE: bool = true;

type WgmRow = (TimerMode, Top, OcrUpdate, TovUpdate, bool);

const fn row(m: TimerMode, t: Top, o: OcrUpdate, v: TovUpdate, f: bool) -> WgmRow {
    (m, t, o, v, f)
}

use OcrUpdate as O;
use TovUpdate as V;

const WGM_8: [WgmRow; 8] = [
    row(Normal, Top::Const(0xff), O::Immediate, V::Max, false),
    row(PwmPhaseCorrect, Top::Const(0xff), O::Top, V::Bottom, false),
    row(Ctc, Top::OcrA, O::Immediate, V::Max, false),
    row(FastPwm, Top::Const(0xff), O::Bottom, V::Max, false),
    row(Reserved, Top::Const(0xff), O::Immediate, V::Max, false),
    row(PwmPhaseCorrect, Top::OcrA, O::Top, V::Bottom, OC_TOGGLE),
    row(Reserved, Top::Const(0xff), O::Immediate, V::Max, false),
    row(FastPwm, Top::OcrA, O::Bottom, V::Top, OC_TOGGLE),
];

const WGM_16: [WgmRow; 16] = [
    row(Normal, Top::Const(0xffff), O::Immediate, V::Max, false),
    row(PwmPhaseCorrect, Top::Const(0x00ff), O::Top, V::Bottom, false),
    row(PwmPhaseCorrect, Top::Const(0x01ff), O::Top, V::Bottom, false),
    row(PwmPhaseCorrect, Top::Const(0x03ff), O::Top, V::Bottom, false),
    row(Ctc, Top::OcrA, O::Immediate, V::Max, false),
    row(FastPwm, Top::Const(0x00ff), O::Bottom, V::Top, false),
    row(FastPwm, Top::Const(0x01ff), O::Bottom, V::Top, false),
    row(FastPwm, Top::Const(0x03ff), O::Bottom, V::Top, false),
    row(PwmPhaseFrequencyCorrect, Top::Icr, O::Bottom, V::Bottom, false),
    row(PwmPhaseFrequencyCorrect, Top::OcrA, O::Bottom, V::Bottom, OC_TOGGLE),
    row(PwmPhaseCorrect, Top::Icr, O::Top, V::Bottom, false),
    row(PwmPhaseCorrect, Top::OcrA, O::Top, V::Bottom, OC_TOGGLE),
    row(Ctc, Top::Icr, O::Immediate, V::Max, false),
    row(Reserved, Top::Const(0xffff), O::Immediate, V::Max, false),
    row(FastPwm, Top::Icr, O::Bottom, V::Top, OC_TOGGLE),
    row(FastPwm, Top::OcrA, O::Bottom, V::Top, OC_TOGGLE),
];

/// `timer0Config` .. `timer2Config`.
#[derive(Clone, Copy)]
pub struct TimerConfig {
    pub bits: u8,
    pub comp_a_interrupt: u8,
    pub comp_b_interrupt: u8,
    pub ovf_interrupt: u8,
    pub tifr: u16,
    pub ocra: u16,
    pub ocrb: u16,
    pub icr: u16,
    pub tcnt: u16,
    pub tccra: u16,
    pub tccrb: u16,
    pub tccrc: u16,
    pub timsk: u16,
    pub dividers: [u32; 8],
    pub comp_port_a: u16,
    pub comp_pin_a: u8,
    pub comp_port_b: u16,
    pub comp_pin_b: u8,
    pub external_clock_port: u16,
    pub external_clock_pin: u8,
}

const DIV01: [u32; 8] = [0, 1, 8, 64, 256, 1024, 0, 0];

pub fn timer_config(i: usize) -> TimerConfig {
    match i {
        0 => TimerConfig { bits: 8, comp_a_interrupt: 0x1c, comp_b_interrupt: 0x1e, ovf_interrupt: 0x20, tifr: 0x35, ocra: 0x47, ocrb: 0x48, icr: 0, tcnt: 0x46, tccra: 0x44, tccrb: 0x45, tccrc: 0, timsk: 0x6e, dividers: DIV01, comp_port_a: 0x2b, comp_pin_a: 6, comp_port_b: 0x2b, comp_pin_b: 5, external_clock_port: 0x2b, external_clock_pin: 4 },
        1 => TimerConfig { bits: 16, comp_a_interrupt: 0x16, comp_b_interrupt: 0x18, ovf_interrupt: 0x1a, tifr: 0x36, ocra: 0x88, ocrb: 0x8a, icr: 0x86, tcnt: 0x84, tccra: 0x80, tccrb: 0x81, tccrc: 0x82, timsk: 0x6f, dividers: DIV01, comp_port_a: 0x25, comp_pin_a: 1, comp_port_b: 0x25, comp_pin_b: 2, external_clock_port: 0x2b, external_clock_pin: 5 },
        _ => TimerConfig { bits: 8, comp_a_interrupt: 0x0e, comp_b_interrupt: 0x10, ovf_interrupt: 0x12, tifr: 0x37, ocra: 0xb3, ocrb: 0xb4, icr: 0, tcnt: 0xb2, tccra: 0xb0, tccrb: 0xb1, tccrc: 0, timsk: 0x70, dividers: [0, 1, 8, 32, 64, 128, 256, 1024], comp_port_a: 0x25, comp_pin_a: 3, comp_port_b: 0x2b, comp_pin_b: 3, external_clock_port: 0, external_clock_pin: 0 },
    }
}

const FOCA: u8 = 1 << 7;
const FOCB: u8 = 1 << 6;

pub struct Timer {
    pub attached: bool,
    pub index: usize,
    pub config: TimerConfig,
    pub max: i64,
    pub last_cycle: i64,
    pub ocr_a: i64,
    pub next_ocr_a: i64,
    pub ocr_b: i64,
    pub next_ocr_b: i64,
    pub icr: i64,
    pub tcnt: i64,
    pub tcnt_next: i64,
    pub tcnt_updated: bool,
    pub update_divider: bool,
    pub counting_up: bool,
    pub divider: i64,
    pub external_clock_rising_edge: bool,
    pub high_byte_temp: i64,
    mode: TimerMode,
    top: Top,
    ocr_mode: OcrUpdate,
    tov_mode: TovUpdate,
    pub comp_a: u8,
    pub comp_b: u8,
    pub ovf: Interrupt,
    pub ocfa: Interrupt,
    pub ocfb: Interrupt,
    pub external_clock_port: Option<usize>,
}

fn irq(address: u8, flag_register: u16, flag_mask: u8, enable_register: u16, enable_mask: u8) -> Interrupt {
    Interrupt { address, flag_register, flag_mask, enable_register, enable_mask, constant: false, inverse_flag: false }
}

impl Timer {
    pub fn new(index: usize) -> Timer {
        let c = timer_config(index);
        Timer {
            attached: false,
            index,
            config: c,
            max: if c.bits == 16 { 0xffff } else { 0xff },
            last_cycle: 0,
            ocr_a: 0,
            next_ocr_a: 0,
            ocr_b: 0,
            next_ocr_b: 0,
            icr: 0,
            tcnt: 0,
            tcnt_next: 0,
            tcnt_updated: false,
            update_divider: false,
            counting_up: true,
            divider: 0,
            external_clock_rising_edge: false,
            high_byte_temp: 0,
            mode: Normal,
            top: Top::Const(0xff),
            ocr_mode: OcrUpdate::Immediate,
            tov_mode: TovUpdate::Max,
            comp_a: 0,
            comp_b: 0,
            ovf: irq(c.ovf_interrupt, c.tifr, 1, c.timsk, 1),
            ocfa: irq(c.comp_a_interrupt, c.tifr, 2, c.timsk, 2),
            ocfb: irq(c.comp_b_interrupt, c.tifr, 4, c.timsk, 4),
            external_clock_port: None,
        }
    }

    /// `TOP`.
    pub fn top_value(&self) -> i64 {
        match self.top {
            Top::OcrA => self.ocr_a,
            Top::Icr => self.icr,
            Top::Const(v) => v,
        }
    }

    fn ocr_mask(&self) -> i64 {
        match self.top {
            Top::OcrA | Top::Icr => 0xffff,
            Top::Const(v) => v,
        }
    }

    pub fn mode(&self) -> TimerMode {
        self.mode
    }
}

fn comp_to_override(comp: u8) -> PinOverride {
    match comp {
        1 => PinOverride::Toggle,
        2 => PinOverride::Clear,
        3 => PinOverride::Set,
        _ => PinOverride::Enable,
    }
}

impl Cpu {
    fn timer_cs(&self, t: usize) -> usize {
        (self.data[self.p.timers[t].config.tccrb as usize] & 7) as usize
    }

    /// `updateWGMConfig()`.
    pub(crate) fn timer_update_wgm(&mut self, t: usize) {
        let c = self.p.timers[t].config;
        let tccra = self.data[c.tccra as usize];
        let tccrb = self.data[c.tccrb as usize];
        let mask = if c.bits == 16 { 0x18 } else { 0x8 };
        let wgm = (((tccrb & mask) >> 1) | (tccra & 3)) as usize;
        let (mode, top, ocr_mode, tov_mode, flags) = if c.bits == 16 { WGM_16[wgm] } else { WGM_8[wgm] };
        {
            let tm = &mut self.p.timers[t];
            tm.mode = mode;
            tm.top = top;
            tm.ocr_mode = ocr_mode;
            tm.tov_mode = tov_mode;
        }
        let pwm = matches!(mode, FastPwm | PwmPhaseCorrect | PwmPhaseFrequencyCorrect);
        let prev_a = self.p.timers[t].comp_a;
        let mut comp_a = (tccra >> 6) & 3;
        if comp_a == 1 && pwm && !flags {
            comp_a = 0;
        }
        self.p.timers[t].comp_a = comp_a;
        if (prev_a != 0) != (comp_a != 0) {
            self.timer_update_comp(t, 'A', if comp_a != 0 { PinOverride::Enable } else { PinOverride::None });
        }
        let prev_b = self.p.timers[t].comp_b;
        let mut comp_b = (tccra >> 4) & 3;
        if comp_b == 1 && pwm {
            comp_b = 0;
        }
        self.p.timers[t].comp_b = comp_b;
        if (prev_b != 0) != (comp_b != 0) {
            self.timer_update_comp(t, 'B', if comp_b != 0 { PinOverride::Enable } else { PinOverride::None });
        }
    }

    fn timer_update_comp(&mut self, t: usize, which: char, mode: PinOverride) {
        let c = self.p.timers[t].config;
        let (port, pin) = if which == 'A' { (c.comp_port_a, c.comp_pin_a) } else { (c.comp_port_b, c.comp_pin_b) };
        if let Some(p) = port_by_port_address(port) {
            if self.p.ports[p].attached {
                self.timer_override_pin(p, pin, mode);
            }
        }
    }

    /// `updateCompPin(compValue, pinName, bottom)`.
    fn timer_update_comp_pin(&mut self, t: usize, comp: u8, which: char, bottom: bool) {
        let tm = &self.p.timers[t];
        let inverting = comp == 3;
        let is_set = tm.counting_up == inverting;
        let new = match tm.mode {
            Normal | Ctc => comp_to_override(comp),
            FastPwm => {
                if comp == 1 {
                    if bottom { PinOverride::None } else { PinOverride::Toggle }
                } else if inverting != bottom {
                    PinOverride::Set
                } else {
                    PinOverride::Clear
                }
            }
            PwmPhaseCorrect | PwmPhaseFrequencyCorrect => {
                if comp == 1 {
                    PinOverride::Toggle
                } else if is_set {
                    PinOverride::Set
                } else {
                    PinOverride::Clear
                }
            }
            Reserved => PinOverride::None,
        };
        if new != PinOverride::None {
            self.timer_update_comp(t, which, new);
        }
    }

    /// `timerUpdated(value, prevValue)`.
    fn timer_updated(&mut self, t: usize, value: i64, prev: i64) {
        let (ocr_a, ocr_b) = (self.p.timers[t].ocr_a, self.p.timers[t].ocr_b);
        let overflow = prev > value;
        if ((prev < ocr_a || overflow) && value >= ocr_a) || (prev < ocr_a && overflow) {
            let i = self.p.timers[t].ocfa;
            self.set_interrupt_flag(i);
            let c = self.p.timers[t].comp_a;
            if c != 0 {
                self.timer_update_comp_pin(t, c, 'A', false);
            }
        }
        if ((prev < ocr_b || overflow) && value >= ocr_b) || (prev < ocr_b && overflow) {
            let i = self.p.timers[t].ocfb;
            self.set_interrupt_flag(i);
            let c = self.p.timers[t].comp_b;
            if c != 0 {
                self.timer_update_comp_pin(t, c, 'B', false);
            }
        }
    }

    fn timer_copy_ocr(&mut self, t: usize) {
        let tm = &mut self.p.timers[t];
        tm.ocr_a = tm.next_ocr_a;
        tm.ocr_b = tm.next_ocr_b;
    }

    /// `phasePwmCount(value, delta)`. The reference snapshots OCRA/OCRB, TOP
    /// and `tcntUpdated` on entry; so does this.
    fn timer_phase_pwm_count(&mut self, t: usize, mut value: i64, mut delta: i64) -> i64 {
        let (ocr_a, ocr_b, top, max, tcnt_updated) = {
            let tm = &self.p.timers[t];
            (tm.ocr_a, tm.ocr_b, tm.top_value(), tm.max, tm.tcnt_updated)
        };
        if value == 0 && top == 0 {
            delta = 0;
            if self.p.timers[t].ocr_mode == OcrUpdate::Top {
                self.timer_copy_ocr(t);
            }
        }
        while delta > 0 {
            if self.p.timers[t].counting_up {
                value += 1;
                if value == top && !tcnt_updated {
                    self.p.timers[t].counting_up = false;
                    if self.p.timers[t].ocr_mode == OcrUpdate::Top {
                        self.timer_copy_ocr(t);
                    }
                }
            } else {
                value -= 1;
                if value == 0 && !tcnt_updated {
                    self.p.timers[t].counting_up = true;
                    let i = self.p.timers[t].ovf;
                    self.set_interrupt_flag(i);
                    if self.p.timers[t].ocr_mode == OcrUpdate::Bottom {
                        self.timer_copy_ocr(t);
                    }
                }
            }
            if !tcnt_updated {
                if value == ocr_a {
                    let i = self.p.timers[t].ocfa;
                    self.set_interrupt_flag(i);
                    let c = self.p.timers[t].comp_a;
                    if c != 0 {
                        self.timer_update_comp_pin(t, c, 'A', false);
                    }
                }
                if value == ocr_b {
                    let i = self.p.timers[t].ocfb;
                    self.set_interrupt_flag(i);
                    let c = self.p.timers[t].comp_b;
                    if c != 0 {
                        self.timer_update_comp_pin(t, c, 'B', false);
                    }
                }
            }
            delta -= 1;
        }
        value & max
    }

    /// `count(reschedule, external)`: the timer's clock event.
    pub fn timer_count(&mut self, t: usize, reschedule: bool, external: bool) {
        let cycles = self.cycles as i64;
        let (divider, last_cycle) = (self.p.timers[t].divider, self.p.timers[t].last_cycle);
        let delta = cycles - last_cycle;
        if (divider != 0 && delta >= divider) || external {
            let counter_delta = if external { 1 } else { delta / divider };
            self.p.timers[t].last_cycle += counter_delta * divider;
            let val = self.p.timers[t].tcnt;
            let (mode, top) = (self.p.timers[t].mode, self.p.timers[t].top_value());
            let phase = matches!(mode, PwmPhaseCorrect | PwmPhaseFrequencyCorrect);
            let new_val = if phase { self.timer_phase_pwm_count(t, val, counter_delta) } else { (val + counter_delta) % (top + 1) };
            let overflow = val + counter_delta > top;
            if !self.p.timers[t].tcnt_updated {
                self.p.timers[t].tcnt = new_val;
                if !phase {
                    self.timer_updated(t, new_val, val);
                }
            }
            if !phase {
                if mode == FastPwm && overflow {
                    let (ca, cb) = (self.p.timers[t].comp_a, self.p.timers[t].comp_b);
                    if ca != 0 {
                        self.timer_update_comp_pin(t, ca, 'A', true);
                    }
                    if cb != 0 {
                        self.timer_update_comp_pin(t, cb, 'B', true);
                    }
                }
                if self.p.timers[t].ocr_mode == OcrUpdate::Bottom && overflow {
                    self.timer_copy_ocr(t);
                }
                let tm = &self.p.timers[t];
                if overflow && (tm.tov_mode == TovUpdate::Top || top == tm.max) {
                    let i = tm.ovf;
                    self.set_interrupt_flag(i);
                }
            }
        }
        if self.p.timers[t].tcnt_updated {
            let tm = &mut self.p.timers[t];
            tm.tcnt = tm.tcnt_next;
            tm.tcnt_updated = false;
            let top = tm.top_value();
            if (tm.tcnt == 0 && tm.ocr_mode == OcrUpdate::Bottom) || (tm.tcnt == top && tm.ocr_mode == OcrUpdate::Top) {
                self.timer_copy_ocr(t);
            }
        }
        if self.p.timers[t].update_divider {
            let cs = self.timer_cs(t);
            let c = self.p.timers[t].config;
            let new_divider = c.dividers[cs] as i64;
            self.p.timers[t].last_cycle = if new_divider != 0 { self.cycles as i64 } else { 0 };
            self.p.timers[t].update_divider = false;
            self.p.timers[t].divider = new_divider;
            if c.external_clock_port != 0 && self.p.timers[t].external_clock_port.is_none() {
                self.p.timers[t].external_clock_port = port_by_port_address(c.external_clock_port).filter(|&p| self.p.ports[p].attached);
            }
            let ecp = self.p.timers[t].external_clock_port;
            if let Some(p) = ecp {
                self.p.ports[p].ext_clock[c.external_clock_pin as usize] = None;
            }
            if new_divider != 0 {
                let due = self.p.timers[t].last_cycle + new_divider - self.cycles as i64;
                self.add_clock_event(EV_TIMER + t as u32, due);
            } else if let Some(p) = ecp {
                if cs == 6 || cs == 7 {
                    self.p.ports[p].ext_clock[c.external_clock_pin as usize] = Some(t as u8);
                    self.p.timers[t].external_clock_rising_edge = cs == 7;
                }
            }
            return;
        }
        if reschedule && divider != 0 {
            let due = self.p.timers[t].last_cycle + divider - self.cycles as i64;
            self.add_clock_event(EV_TIMER + t as u32, due);
        }
    }

    /// `externalClockCallback(value)`.
    pub(crate) fn timer_external_clock(&mut self, t: usize, value: bool) {
        if value == self.p.timers[t].external_clock_rising_edge {
            self.timer_count(t, false, true);
        }
    }

    fn timer_force_compare(&mut self, t: usize, value: u8) {
        let mode = self.p.timers[t].mode;
        if matches!(mode, FastPwm | PwmPhaseCorrect | PwmPhaseFrequencyCorrect) {
            return;
        }
        if value & FOCA != 0 {
            let c = self.p.timers[t].comp_a;
            self.timer_update_comp_pin(t, c, 'A', false);
        }
        if value & FOCB != 0 {
            let c = self.p.timers[t].comp_b;
            self.timer_update_comp_pin(t, c, 'B', false);
        }
    }

    // ---- hooks ----------------------------------------------------------------

    /// `readHooks[TCNT]`.
    pub(crate) fn hook_timer_read_tcnt(&mut self, t: usize, addr: usize) -> u8 {
        self.timer_count(t, false, false);
        let tcnt = self.p.timers[t].tcnt;
        if self.p.timers[t].config.bits == 16 {
            self.data[addr + 1] = (tcnt >> 8) as u8;
        }
        self.data[addr] = (tcnt & 0xff) as u8;
        self.data[addr]
    }

    pub(crate) fn hook_timer_tcnt(&mut self, t: usize, value: u8) {
        let tm = &mut self.p.timers[t];
        tm.tcnt_next = (tm.high_byte_temp << 8) | value as i64;
        tm.counting_up = true;
        tm.tcnt_updated = true;
        let (next, has_divider) = (tm.tcnt_next, tm.divider != 0);
        self.update_clock_event(EV_TIMER + t as u32, 0);
        if has_divider {
            self.timer_updated(t, next, next);
        }
    }

    /// OCRnA / OCRnB low byte (`which` 0 or 1).
    pub(crate) fn hook_timer_ocr(&mut self, t: usize, which: u8, value: u8) {
        let tm = &mut self.p.timers[t];
        let next = (tm.high_byte_temp << 8) | value as i64;
        if which == 0 {
            tm.next_ocr_a = next;
            if tm.ocr_mode == OcrUpdate::Immediate {
                tm.ocr_a = next;
            }
        } else {
            tm.next_ocr_b = next;
            if tm.ocr_mode == OcrUpdate::Immediate {
                tm.ocr_b = next;
            }
        }
    }

    pub(crate) fn hook_timer_icr(&mut self, t: usize, value: u8) {
        let tm = &mut self.p.timers[t];
        tm.icr = (tm.high_byte_temp << 8) | value as i64;
    }

    /// TCNTnH / ICRnH: only the temporary register.
    pub(crate) fn hook_timer_temp(&mut self, t: usize, value: u8) {
        self.p.timers[t].high_byte_temp = value as i64;
    }

    /// OCRnAH / OCRnBH (`updateOCRHighRegister`): stores and consumes the write.
    pub(crate) fn hook_timer_ocr_high(&mut self, t: usize, addr: usize, value: u8) {
        let mask = self.p.timers[t].ocr_mask() >> 8;
        let v = value as i64 & mask;
        self.p.timers[t].high_byte_temp = v;
        self.data[addr] = v as u8;
    }

    pub(crate) fn hook_timer_tccra(&mut self, t: usize, value: u8) {
        let reg = self.p.timers[t].config.tccra as usize;
        self.data[reg] = value;
        self.timer_update_wgm(t);
    }

    pub(crate) fn hook_timer_tccrb(&mut self, t: usize, mut value: u8) {
        let c = self.p.timers[t].config;
        if c.tccrc == 0 {
            self.timer_force_compare(t, value);
            value &= !(FOCA | FOCB);
        }
        self.data[c.tccrb as usize] = value;
        self.p.timers[t].update_divider = true;
        self.clear_clock_event(EV_TIMER + t as u32);
        self.add_clock_event(EV_TIMER + t as u32, 0);
        self.timer_update_wgm(t);
    }

    pub(crate) fn hook_timer_tccrc(&mut self, t: usize, value: u8) {
        self.timer_force_compare(t, value);
    }

    pub(crate) fn hook_timer_tifr(&mut self, t: usize, value: u8) {
        let tm = &self.p.timers[t];
        let (reg, ovf, a, b) = (tm.config.tifr as usize, tm.ovf, tm.ocfa, tm.ocfb);
        self.data[reg] = value;
        self.clear_interrupt_by_flag(ovf, value);
        self.clear_interrupt_by_flag(a, value);
        self.clear_interrupt_by_flag(b, value);
    }

    pub(crate) fn hook_timer_timsk(&mut self, t: usize, value: u8) {
        let tm = &self.p.timers[t];
        let (ovf, a, b) = (tm.ovf, tm.ocfa, tm.ocfb);
        self.update_interrupt_enable(ovf, value);
        self.update_interrupt_enable(a, value);
        self.update_interrupt_enable(b, value);
    }
}
