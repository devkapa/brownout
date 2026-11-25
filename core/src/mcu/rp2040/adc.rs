//! The ADC, ported from rp2040js 1.3.3 (MIT): `src/peripherals/adc.ts`.
//! Channel values are set from outside (`channel_values`); a conversion takes
//! 2 microseconds and then lands in RESULT and, when enabled, the FIFO.
//!
//! Reference quirks kept (each has a `reference_quirk_*` test):
//! - a CS write clears FCS bits (`fcs &= ~(value & CS_ERR_STICKY)`), so
//!   FCS.UNDER clears and CS.ERR_STICKY can never clear;
//! - START_ONE is stored and reads back set;
//! - the round-robin step masks the next channel with `CS_AINSEL_SHIFT` (12)
//!   instead of the 3-bit mask, so it can only select channel 4 or 0;
//! - a FIFO read does not refresh the interrupt line.

use super::clock::{js_to_i32, js_to_u32, AlarmId};
use super::dma::dreq;
use super::usb::Fifo;
use super::Rp2040;

const CS: u32 = 0x00;
const RESULT: u32 = 0x04;
const FCS: u32 = 0x08;
const FIFO_REG: u32 = 0x0c;
const DIV: u32 = 0x10;
const INTR: u32 = 0x14;
const INTE: u32 = 0x18;
const INTF: u32 = 0x1c;
const INTS: u32 = 0x20;

const CS_RROBIN_MASK: u32 = 0x1f;
const CS_RROBIN_SHIFT: u32 = 16;
const CS_AINSEL_MASK: u32 = 0x7;
const CS_AINSEL_SHIFT: u32 = 12;
const CS_ERR_STICKY: u32 = 1 << 10;
const CS_ERR: u32 = 1 << 9;
const CS_READY: u32 = 1 << 8;
const CS_START_MANY: u32 = 1 << 3;
const CS_START_ONE: u32 = 1 << 2;
const CS_TS_EN: u32 = 1 << 1;
const CS_EN: u32 = 1 << 0;
const CS_WRITE_MASK: u32 =
    (CS_RROBIN_MASK << CS_RROBIN_SHIFT) | (CS_AINSEL_MASK << CS_AINSEL_SHIFT) | CS_START_MANY | CS_START_ONE | CS_TS_EN | CS_EN;

const FCS_THRES_MASK: u32 = 0xf;
const FCS_THRESH_SHIFT: u32 = 24;
const FCS_LEVEL_MASK: u32 = 0xf;
const FCS_LEVEL_SHIFT: u32 = 16;
const FCS_OVER: u32 = 1 << 11;
const FCS_UNDER: u32 = 1 << 10;
const FCS_FULL: u32 = 1 << 9;
const FCS_EMPTY: u32 = 1 << 8;
const FCS_DREQ_EN: u32 = 1 << 3;
const FCS_ERR: u32 = 1 << 2;
const FCS_SHIFT: u32 = 1 << 1;
const FCS_EN: u32 = 1 << 0;
const FCS_WRITE_MASK: u32 = (FCS_THRES_MASK << FCS_THRESH_SHIFT) | FCS_DREQ_EN | FCS_ERR | FCS_SHIFT | FCS_EN;

const FIFO_ERR: u32 = 1 << 15;
const DIV_INT_MASK: u32 = 0xffff;
const DIV_INT_SHIFT: u32 = 8;
const DIV_FRAC_MASK: u32 = 0xff;
const FIFO_INT: u32 = 1;
const IRQ_ADC_FIFO: u32 = 22;
const NUM_CHANNELS: u32 = 5;
/// Sample time in microseconds.
const SAMPLE_TIME: f64 = 2.0;

pub struct Adc {
    /// `channelValues`: channels 0..3 are GPIO26..29, 4 the temperature sensor.
    pub channel_values: [f64; 5],
    pub fifo: Fifo,
    pub cs: u32,
    pub fcs: u32,
    pub clock_div: u32,
    pub int_enable: u32,
    pub int_force: u32,
    /// The reference stores the converted value as is (a number).
    pub result: f64,
    pub busy: bool,
    pub err: bool,
    pub current_channel: u32,
}

impl Default for Adc {
    fn default() -> Self {
        Self::new()
    }
}

impl Adc {
    pub fn new() -> Self {
        Adc {
            channel_values: [0.0; 5],
            fifo: Fifo::new(4),
            cs: 0,
            fcs: 0,
            clock_div: 0,
            int_enable: 0,
            int_force: 0,
            result: 0.0,
            busy: false,
            err: false,
            current_channel: 0,
        }
    }

    fn active_channel(&self) -> u32 {
        (self.cs >> CS_AINSEL_SHIFT) & CS_AINSEL_MASK
    }

    /// Reference quirk: the channel is masked with `CS_AINSEL_SHIFT` (12).
    fn set_active_channel(&mut self, channel: u32) {
        self.cs &= !(CS_AINSEL_MASK << CS_AINSEL_SHIFT);
        self.cs |= (channel & CS_AINSEL_SHIFT) << CS_AINSEL_SHIFT;
    }

    pub fn divider(&self) -> f64 {
        1.0 + ((self.clock_div >> DIV_INT_SHIFT) & DIV_INT_MASK) as f64 + (self.clock_div & DIV_FRAC_MASK) as f64 / 256.0
    }

    fn thres(&self) -> u32 {
        (self.fcs >> FCS_THRESH_SHIFT) & FCS_THRES_MASK
    }

    pub fn int_raw(&self) -> u32 {
        if self.fifo.item_count() as u32 >= self.thres() { FIFO_INT } else { 0 }
    }

    pub fn int_status(&self) -> u32 {
        (self.int_raw() & self.int_enable) | self.int_force
    }
}

impl Rp2040 {
    /// Sets a channel's input, in raw 12-bit counts (what the wrapper does).
    pub fn adc_set_channel(&mut self, channel: usize, value: f64) {
        self.periph.adc.channel_values[channel] = value;
    }

    fn adc_check_interrupts(&mut self) {
        let s = self.periph.adc.int_status() != 0;
        self.set_interrupt(IRQ_ADC_FIFO, s);
    }

    fn adc_update_dma(&mut self) {
        let a = &self.periph.adc;
        if a.fcs & FCS_DREQ_EN != 0 {
            if a.fifo.item_count() as u32 >= a.thres() {
                self.dma_set_dreq(dreq::ADC);
            } else {
                self.dma_clear_dreq(dreq::ADC);
            }
        }
    }

    /// `startADCRead` with the default `onADCRead`.
    fn adc_start_read(&mut self) {
        let a = &mut self.periph.adc;
        a.busy = true;
        a.current_channel = a.active_channel();
        self.clock.schedule(AlarmId::AdcSample, SAMPLE_TIME * 1000.0);
    }

    pub(crate) fn adc_sample_alarm_fired(&mut self) {
        let a = &self.periph.adc;
        let v = a.channel_values.get(a.current_channel as usize).copied().unwrap_or(0.0);
        self.adc_complete_read(v, false);
    }

    pub(crate) fn adc_multi_alarm_fired(&mut self) {
        if self.periph.adc.cs & CS_START_MANY != 0 {
            self.adc_start_read();
        }
    }

    /// `completeADCRead`.
    pub fn adc_complete_read(&mut self, value: f64, error: bool) {
        let a = &mut self.periph.adc;
        a.busy = false;
        a.result = value;
        if error {
            a.cs |= CS_ERR_STICKY | CS_ERR;
        } else {
            a.cs &= !CS_ERR;
        }
        if a.fcs & FCS_EN != 0 {
            if a.fifo.is_full() {
                a.fcs |= FCS_OVER;
            } else {
                let mut v = (js_to_i32(value) & 0xfff) as u32;
                if a.fcs & FCS_SHIFT != 0 {
                    v >>= 4;
                }
                if error && a.fcs & FCS_ERR != 0 {
                    v |= FIFO_ERR;
                }
                a.fifo.push(v);
                self.adc_update_dma();
                self.adc_check_interrupts();
            }
        }
        let a = &mut self.periph.adc;
        let round = (a.cs >> CS_RROBIN_SHIFT) & CS_RROBIN_MASK;
        if round != 0 {
            let mut channel = a.active_channel() + 1;
            while round & (1u32.checked_shl(channel).unwrap_or(0)) == 0 {
                channel = (channel + 1) % NUM_CHANNELS;
            }
            a.set_active_channel(channel);
        }
        if a.cs & CS_START_MANY != 0 {
            let sample_ticks = 48.0 * SAMPLE_TIME;
            if a.divider() > sample_ticks {
                let micros = (a.divider() - sample_ticks) / 48.0;
                self.clock.schedule(AlarmId::AdcMulti, micros * 1000.0);
            } else {
                self.adc_start_read();
            }
        }
    }

    pub(crate) fn adc_read(&mut self, offset: u32) -> Option<u32> {
        let a = &mut self.periph.adc;
        Some(match offset {
            CS => a.cs | (if a.err { CS_ERR } else { 0 }) | (if a.busy { 0 } else { CS_READY }),
            RESULT => js_to_u32(a.result),
            FCS => {
                a.fcs
                    | ((a.fifo.item_count() as u32 & FCS_LEVEL_MASK) << FCS_LEVEL_SHIFT)
                    | (if a.fifo.is_full() { FCS_FULL } else { 0 })
                    | (if a.fifo.is_empty() { FCS_EMPTY } else { 0 })
            }
            FIFO_REG => {
                if a.fifo.is_empty() {
                    a.fcs |= FCS_UNDER;
                    0
                } else {
                    let v = a.fifo.pull();
                    self.adc_update_dma();
                    v
                }
            }
            DIV => a.clock_div,
            INTR => a.int_raw(),
            INTE => a.int_enable,
            INTF => a.int_force,
            INTS => a.int_status(),
            _ => return None,
        })
    }

    pub(crate) fn adc_write(&mut self, offset: u32, value: u32) -> bool {
        match offset {
            CS => {
                let a = &mut self.periph.adc;
                a.fcs &= !(value & CS_ERR_STICKY);
                a.cs = (a.cs & !CS_WRITE_MASK) | (value & CS_WRITE_MASK);
                if value & CS_EN != 0 && !a.busy && (value & CS_START_ONE != 0 || value & CS_START_MANY != 0) {
                    self.adc_start_read();
                }
            }
            FCS => {
                let a = &mut self.periph.adc;
                a.fcs &= !(value & (FCS_OVER | FCS_UNDER));
                a.fcs = (a.fcs & !FCS_WRITE_MASK) | (value & FCS_WRITE_MASK);
                self.adc_check_interrupts();
            }
            DIV => self.periph.adc.clock_div = value,
            INTE => {
                self.periph.adc.int_enable = value & FIFO_INT;
                self.adc_check_interrupts();
            }
            INTF => {
                self.periph.adc.int_force = value & FIFO_INT;
                self.adc_check_interrupts();
            }
            _ => return false,
        }
        true
    }
}
