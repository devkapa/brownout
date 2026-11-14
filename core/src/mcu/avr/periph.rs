//! Wiring of the peripherals `ArduinoMcu` constructs (ports B, C and D,
//! timers 0 to 2 and the ADC) to the CPU: the hook tables, hook dispatch and
//! clock-event dispatch. Order matches the reference's constructors, so a
//! later hook on the same address replaces an earlier one as it does there.

use super::adc::Adc;
use super::cpu::{Cpu, ClockEvent, EV_ADC_READ, EV_TIMER};
use super::gpio::{ExtHook, Port, PORT_B, PORT_C, PORT_D};
use super::timer::{timer_config, Timer};

// Write hook kinds (the high byte of a hook code; the low byte is the index).
const W_DDR: u16 = 1;
const W_PORT: u16 = 2;
const W_PIN: u16 = 3;
const W_EXT_OTHER: u16 = 4;
const W_EXT_MASK: u16 = 5;
const W_EXT_FLAG: u16 = 6;
const W_PCIFR: u16 = 7;
const W_PCMSK: u16 = 8;
const W_TCNT: u16 = 9;
const W_TEMP: u16 = 10; // TCNTnH and ICRnH
const W_OCRA: u16 = 11;
const W_OCRB: u16 = 12;
const W_OCR_HIGH: u16 = 13;
const W_ICR: u16 = 14;
const W_TCCRA: u16 = 15;
const W_TCCRB: u16 = 16;
const W_TCCRC: u16 = 17;
const W_TIFR: u16 = 18;
const W_TIMSK: u16 = 19;
const W_ADCSRA: u16 = 20;

const R_TCNT: u16 = 1;

fn code(kind: u16, index: usize) -> u16 {
    (kind << 8) | index as u16
}

impl Cpu {
    fn ensure_periph(&mut self) {
        if !self.p.attached {
            self.p.hook_w = vec![0; 0x200];
            self.p.hook_r = vec![0; 0x200];
            self.p.ports = (0..3).map(Port::new).collect();
            self.p.timers = (0..3).map(Timer::new).collect();
            self.p.attached = true;
            self.p.listening = true;
        }
    }

    /// `new AVRIOPort(cpu, portBConfig)` (and C, D): `i` is `PORT_B`..`PORT_D`.
    pub fn attach_port(&mut self, i: usize) {
        self.ensure_periph();
        self.p.ports[i].attached = true;
        let cfg = &self.p.ports[i].config;
        let (ddr, port, pin) = (cfg.ddr as usize, cfg.port as usize, cfg.pin as usize);
        let (pc, ext) = (cfg.pin_change, cfg.external);
        self.p.hook_w[ddr] = code(W_DDR, i);
        self.p.hook_w[port] = code(W_PORT, i);
        self.p.hook_w[pin] = code(W_PIN, i);
        if let Some(e) = ext.iter().flatten().next() {
            // attachInterruptHook(EICR, 'other'), (EIMSK, 'mask'), (EIFR, 'flag')
            self.p.hook_w[e.eicr as usize] = code(W_EXT_OTHER, i);
            self.p.hook_w[e.eimsk as usize] = code(W_EXT_MASK, i);
            self.p.hook_w[e.eifr as usize] = code(W_EXT_FLAG, i);
        }
        if let Some(pc) = pc {
            self.p.hook_w[pc.pcifr as usize] = code(W_PCIFR, i);
            self.p.hook_w[pc.pcmsk as usize] = code(W_PCMSK, i);
        }
    }

    /// `new AVRTimer(cpu, timerNConfig)`.
    pub fn attach_timer(&mut self, t: usize) {
        self.ensure_periph();
        self.p.timers[t].attached = true;
        let c = timer_config(t);
        self.timer_update_wgm(t);
        let w = &mut self.p.hook_w;
        self.p.hook_r[c.tcnt as usize] = code(R_TCNT, t);
        w[c.tcnt as usize] = code(W_TCNT, t);
        w[c.ocra as usize] = code(W_OCRA, t);
        w[c.ocrb as usize] = code(W_OCRB, t);
        if c.bits == 16 {
            w[c.icr as usize] = code(W_ICR, t);
            w[c.tcnt as usize + 1] = code(W_TEMP, t);
            w[c.ocra as usize + 1] = code(W_OCR_HIGH, t);
            w[c.ocrb as usize + 1] = code(W_OCR_HIGH, t);
            w[c.icr as usize + 1] = code(W_TEMP, t);
        }
        w[c.tccra as usize] = code(W_TCCRA, t);
        w[c.tccrb as usize] = code(W_TCCRB, t);
        if c.tccrc != 0 {
            w[c.tccrc as usize] = code(W_TCCRC, t);
        }
        w[c.tifr as usize] = code(W_TIFR, t);
        w[c.timsk as usize] = code(W_TIMSK, t);
    }

    /// `new AVRADC(cpu, adcConfig)`.
    pub fn attach_adc(&mut self) {
        self.ensure_periph();
        self.p.adc = Some(Adc::new());
        self.p.hook_w[0x7a] = code(W_ADCSRA, 0);
    }

    /// Constructs what `new ArduinoMcu(hex)` does after `new CPU(flash)`:
    /// ports B, C, D, timers 0, 1, 2, the ADC, in that order.
    pub fn attach_arduino_peripherals(&mut self) {
        for i in [PORT_B, PORT_C, PORT_D] {
            self.attach_port(i);
        }
        for t in 0..3 {
            self.attach_timer(t);
        }
        self.attach_adc();
    }

    pub(crate) fn run_read_hook(&mut self, code: u16, addr: usize) -> u8 {
        let t = (code & 0xff) as usize;
        match code >> 8 {
            R_TCNT => self.hook_timer_read_tcnt(t, addr),
            _ => self.rd(addr as i32),
        }
    }

    /// Runs a write hook; true means it consumed the write.
    pub(crate) fn run_write_hook(&mut self, code: u16, value: u8, old: u8, addr: usize, mask: u8) -> bool {
        let i = (code & 0xff) as usize;
        match code >> 8 {
            W_DDR => self.hook_port_ddr(i, value),
            W_PORT => self.hook_port_port(i, value),
            W_PIN => self.hook_port_pin(i, value, mask),
            W_EXT_OTHER => self.hook_external(addr, ExtHook::Other, value),
            W_EXT_MASK => self.hook_external(addr, ExtHook::Mask, value),
            W_EXT_FLAG => self.hook_external(addr, ExtHook::Flag, value),
            W_PCIFR => self.hook_pcifr(value),
            W_PCMSK => self.hook_pcmsk(addr, value),
            W_TCNT => {
                self.hook_timer_tcnt(i, value);
                return false;
            }
            W_TEMP => {
                self.hook_timer_temp(i, value);
                return false;
            }
            W_OCRA => {
                self.hook_timer_ocr(i, 0, value);
                return false;
            }
            W_OCRB => {
                self.hook_timer_ocr(i, 1, value);
                return false;
            }
            W_OCR_HIGH => self.hook_timer_ocr_high(i, addr, value),
            W_ICR => {
                self.hook_timer_icr(i, value);
                return false;
            }
            W_TCCRA => self.hook_timer_tccra(i, value),
            W_TCCRB => self.hook_timer_tccrb(i, value),
            W_TCCRC => {
                self.hook_timer_tccrc(i, value);
                return false;
            }
            W_TIFR => self.hook_timer_tifr(i, value),
            W_TIMSK => {
                self.hook_timer_timsk(i, value);
                return false;
            }
            W_ADCSRA => return self.hook_adcsra(value, old),
            _ => return false,
        }
        true
    }

    /// A due clock event: the reference's callback.
    pub(crate) fn dispatch_event(&mut self, ev: ClockEvent) {
        if !self.p.attached {
            return;
        }
        if ev.id >= EV_TIMER && ev.id < EV_TIMER + 3 {
            self.timer_count((ev.id - EV_TIMER) as usize, true, false);
        } else if ev.id == EV_ADC_READ {
            self.adc_complete(ev.arg);
        }
    }
}
