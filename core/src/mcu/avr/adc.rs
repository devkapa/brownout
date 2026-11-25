//! The ADC, ported from avr8js 0.21.0 `peripherals/adc.js` (MIT, Copyright
//! (c) Uri Shaked and contributors; see NOTICE) for the ATmega328P:
//! `AVRADC` with `adcConfig`'s channels and references. The default
//! `onADCRead` is the only one: the result comes from `channel_values`
//! (volts, set from outside) and lands `sampleCycles` later.

use super::cpu::{Cpu, EV_ADC_READ};
use super::interrupt::Interrupt;

const ADMUX: usize = 0x7c;
const ADCSRA: usize = 0x7a;
const ADCSRB: usize = 0x7b;
const ADCL: usize = 0x78;
const ADCH: usize = 0x79;
const ADC_INTERRUPT: u8 = 0x2a;
const NUM_CHANNELS: usize = 8;
const MUX_INPUT_MASK: u8 = 0xf;

const ADPS_MASK: u8 = 0x7;
const ADIE: u8 = 0x8;
const ADIF: u8 = 0x10;
const ADSC: u8 = 0x40;
const ADEN: u8 = 0x80;
const MUX_MASK: u8 = 0x1f;
const ADLAR: u8 = 0x20;
const MUX5: u8 = 0x8;
const REFS_MASK: u8 = 0x3;
const REFS_SHIFT: u8 = 6;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MuxInput {
    SingleEnded(usize),
    Constant(f64),
    Temperature,
}

/// `atmega328Channels`; an input not in the table is the 0 V constant.
fn mux_channel(ch: u8) -> MuxInput {
    match ch {
        0..=7 => MuxInput::SingleEnded(ch as usize),
        8 => MuxInput::Temperature,
        14 => MuxInput::Constant(1.1),
        _ => MuxInput::Constant(0.0),
    }
}

pub struct Adc {
    /// `channelValues`: volts per channel; unset channels read 0.
    pub channel_values: [f64; NUM_CHANNELS],
    pub avcc: f64,
    pub aref: f64,
    pub converting: bool,
    pub conversion_cycles: i64,
    pub interrupt: Interrupt,
}

impl Adc {
    pub fn new() -> Adc {
        Adc {
            channel_values: [0.0; NUM_CHANNELS],
            avcc: 5.0,
            aref: 5.0,
            converting: false,
            conversion_cycles: 25,
            interrupt: Interrupt {
                address: ADC_INTERRUPT,
                flag_register: ADCSRA as u16,
                flag_mask: ADIF,
                enable_register: ADCSRA as u16,
                enable_mask: ADIE,
                constant: false,
                inverse_flag: false,
            },
        }
    }
}

impl Default for Adc {
    fn default() -> Self {
        Adc::new()
    }
}

impl Cpu {
    fn adc_prescaler(&self) -> i64 {
        match self.data[ADCSRA] & ADPS_MASK {
            0 | 1 => 2,
            2 => 4,
            3 => 8,
            4 => 16,
            5 => 32,
            6 => 64,
            _ => 128,
        }
    }

    /// `referenceVoltage`: `adcReferences` is [AREF, AVCC, Reserved,
    /// Internal1V1] and Reserved reads as AVCC.
    fn adc_reference_voltage(&self) -> f64 {
        let adc = self.p.adc.as_ref().unwrap();
        match (self.data[ADMUX] >> REFS_SHIFT) & REFS_MASK {
            0 => adc.aref,
            1 | 2 => adc.avcc,
            _ => 1.1,
        }
    }

    fn adc_sample_cycles(&self) -> i64 {
        self.p.adc.as_ref().unwrap().conversion_cycles * self.adc_prescaler()
    }

    /// The default `onADCRead`.
    fn adc_on_read(&mut self, input: MuxInput) {
        if self.p.log_overrides {
            self.p.adc_reads.push(input);
        }
        let voltage = match input {
            MuxInput::Constant(v) => v,
            MuxInput::SingleEnded(ch) => self.p.adc.as_ref().unwrap().channel_values[ch],
            MuxInput::Temperature => 0.378125,
        };
        let raw = (voltage / self.adc_reference_voltage()) * 1024.0;
        let result = raw.floor().max(0.0).min(1023.0) as u32;
        let cycles = self.adc_sample_cycles();
        self.add_clock_event_arg(EV_ADC_READ, result, cycles);
    }

    /// `completeADCRead(value)`.
    pub(crate) fn adc_complete(&mut self, value: u32) {
        {
            let adc = self.p.adc.as_mut().unwrap();
            adc.converting = false;
            adc.conversion_cycles = 13;
        }
        if self.data[ADMUX] & ADLAR != 0 {
            self.data[ADCL] = ((value << 6) & 0xff) as u8;
            self.data[ADCH] = (value >> 2) as u8;
        } else {
            self.data[ADCL] = (value & 0xff) as u8;
            self.data[ADCH] = ((value >> 8) & 3) as u8;
        }
        self.data[ADCSRA] &= !ADSC;
        let i = self.p.adc.as_ref().unwrap().interrupt;
        self.set_interrupt_flag(i);
    }

    /// `writeHooks[ADCSRA]`: returns whether the write was consumed. (The
    /// reference stores the value itself first, so a write that is not
    /// consumed stores the same value again.)
    pub(crate) fn hook_adcsra(&mut self, value: u8, old: u8) -> bool {
        // `value & ADEN && !(oldValue && ADEN)`: `oldValue && ADEN` is ADEN
        // for any nonzero old value, so this only resets on 0 -> enabled.
        if value & ADEN != 0 && old == 0 {
            self.p.adc.as_mut().unwrap().conversion_cycles = 25;
        }
        self.data[ADCSRA] = value;
        let i = self.p.adc.as_ref().unwrap().interrupt;
        self.update_interrupt_enable(i, value);
        if !self.p.adc.as_ref().unwrap().converting && value & ADSC != 0 {
            if value & ADEN == 0 {
                let cycles = self.adc_sample_cycles();
                self.add_clock_event_arg(EV_ADC_READ, 0, cycles);
                return true;
            }
            let mut channel = self.data[ADMUX] & MUX_MASK;
            if self.data[ADCSRB] & MUX5 != 0 {
                channel |= 0x20;
            }
            channel &= MUX_INPUT_MASK;
            self.p.adc.as_mut().unwrap().converting = true;
            self.adc_on_read(mux_channel(channel));
            return true;
        }
        false
    }

    /// Host side: `adc.channelValues[channel] = volts` (what
    /// `ArduinoMcu.setAnalogVolts` does after clamping).
    pub fn set_adc_channel(&mut self, channel: usize, volts: f64) {
        if let Some(adc) = self.p.adc.as_mut() {
            if channel < NUM_CHANNELS {
                adc.channel_values[channel] = volts;
            }
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for MuxInput {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            MuxInput::SingleEnded(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            MuxInput::Constant(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            MuxInput::Temperature => 0,
        }
    }
}
