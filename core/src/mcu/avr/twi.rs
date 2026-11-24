//! TWI (I2C), ported from avr8js 0.21.0 `peripherals/twi.js` (MIT, Copyright
//! (c) Uri Shaked and contributors; see NOTICE): `AVRTWI` with `twiConfig`,
//! master mode only, as the reference. The event handler is `handler`:
//! `TwiHandler::Noop` is the reference's `NoopTWIEventHandler` (what
//! `ArduinoMcu` keeps: every address NACKs, as an empty bus does);
//! `TwiHandler::Host` leaves each step for the host to finish with the
//! `twi_complete_*` calls, as a device on the bus would. `events` records
//! the handler calls while it is `Some`.

use super::cpu::{Cpu, EV_TWI};
use super::interrupt::Interrupt;

pub const TWBR: usize = 0xb8;
pub const TWSR: usize = 0xb9;
pub const TWDR: usize = 0xbb;
pub const TWCR: usize = 0xbc;

const TWCR_TWINT: u8 = 0x80;
const TWCR_TWEA: u8 = 0x40;
const TWCR_TWSTA: u8 = 0x20;
const TWCR_TWSTO: u8 = 0x10;
const TWCR_TWEN: u8 = 0x4;
const TWCR_TWIE: u8 = 0x1;
const TWSR_TWS_MASK: u8 = 0xf8;
const TWSR_TWPS_MASK: u8 = 0x3;

const STATUS_TWI_IDLE: u8 = 0xf8;
const STATUS_START: u8 = 0x08;
const STATUS_REPEATED_START: u8 = 0x10;
const STATUS_SLAW_ACK: u8 = 0x18;
const STATUS_SLAW_NACK: u8 = 0x20;
const STATUS_DATA_SENT_ACK: u8 = 0x28;
const STATUS_DATA_SENT_NACK: u8 = 0x30;
const STATUS_SLAR_ACK: u8 = 0x40;
const STATUS_SLAR_NACK: u8 = 0x48;
const STATUS_DATA_RECEIVED_ACK: u8 = 0x50;
const STATUS_DATA_RECEIVED_NACK: u8 = 0x58;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TwiHandler {
    Noop,
    Host,
}

/// A call to the event handler.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TwiEvent {
    Start { repeated: bool },
    Stop,
    Connect { addr: u8, write: bool },
    WriteByte(u8),
    ReadByte { ack: bool },
}

pub struct Twi {
    pub freq_hz: u32,
    pub busy: bool,
    pub handler: TwiHandler,
    pub events: Option<Vec<TwiEvent>>,
    pub interrupt: Interrupt,
}

impl Twi {
    pub fn new(freq_hz: u32) -> Twi {
        Twi {
            freq_hz,
            busy: false,
            handler: TwiHandler::Noop,
            events: None,
            interrupt: Interrupt {
                address: 0x30,
                flag_register: TWCR as u16,
                flag_mask: TWCR_TWINT,
                enable_register: TWCR as u16,
                enable_mask: TWCR_TWIE,
                constant: false,
                inverse_flag: false,
            },
        }
    }
}

impl Cpu {
    fn twi(&mut self) -> &mut Twi {
        self.p.twi.as_mut().unwrap()
    }

    pub fn twi_prescaler(&self) -> u32 {
        match self.data[TWSR] & TWSR_TWPS_MASK {
            0 => 1,
            1 => 4,
            2 => 16,
            _ => 64,
        }
    }

    pub fn twi_scl_frequency(&self) -> f64 {
        let f = self.p.twi.as_ref().unwrap().freq_hz as f64;
        f / (16.0 + 2.0 * self.data[TWBR] as f64 * self.twi_prescaler() as f64)
    }

    fn twi_status(&self) -> u8 {
        self.data[TWSR] & TWSR_TWS_MASK
    }

    /// `updateStatus(value)`: also raises TWINT.
    pub(crate) fn twi_update_status(&mut self, value: u8) {
        self.data[TWSR] = (self.data[TWSR] & !TWSR_TWS_MASK) | value;
        let i = self.twi().interrupt;
        self.set_interrupt_flag(i);
    }

    pub fn twi_complete_start(&mut self) {
        self.twi().busy = false;
        let s = if self.twi_status() == STATUS_TWI_IDLE { STATUS_START } else { STATUS_REPEATED_START };
        self.twi_update_status(s);
    }

    pub fn twi_complete_stop(&mut self) {
        self.twi().busy = false;
        self.data[TWCR] &= !TWCR_TWSTO;
        self.twi_update_status(STATUS_TWI_IDLE);
    }

    pub fn twi_complete_connect(&mut self, ack: bool) {
        self.twi().busy = false;
        let s = if self.data[TWDR] & 1 != 0 {
            if ack { STATUS_SLAR_ACK } else { STATUS_SLAR_NACK }
        } else if ack {
            STATUS_SLAW_ACK
        } else {
            STATUS_SLAW_NACK
        };
        self.twi_update_status(s);
    }

    pub fn twi_complete_write(&mut self, ack: bool) {
        self.twi().busy = false;
        self.twi_update_status(if ack { STATUS_DATA_SENT_ACK } else { STATUS_DATA_SENT_NACK });
    }

    pub fn twi_complete_read(&mut self, value: u8) {
        self.twi().busy = false;
        let ack = self.data[TWCR] & TWCR_TWEA != 0;
        self.data[TWDR] = value;
        self.twi_update_status(if ack { STATUS_DATA_RECEIVED_ACK } else { STATUS_DATA_RECEIVED_NACK });
    }

    /// One handler call: recorded, then answered at once by the no-op
    /// handler or left to the host.
    fn twi_handle(&mut self, e: TwiEvent) {
        let t = self.twi();
        if let Some(ev) = t.events.as_mut() {
            ev.push(e);
        }
        if t.handler == TwiHandler::Host {
            return;
        }
        match e {
            TwiEvent::Start { .. } => self.twi_complete_start(),
            TwiEvent::Stop => self.twi_complete_stop(),
            TwiEvent::Connect { .. } => self.twi_complete_connect(false),
            TwiEvent::WriteByte(_) => self.twi_complete_write(false),
            TwiEvent::ReadByte { .. } => self.twi_complete_read(0xff),
        }
    }

    /// `writeHooks[TWCR]`. A write that clears TWINT with TWEN set while idle
    /// is consumed and schedules the next bus step for the next cycle, with
    /// the TWCR value, status and TWDR as they are now (the reference's
    /// closure); any other write is not consumed, so the value is stored
    /// again after the hook (TWINT included).
    pub(crate) fn hook_twcr(&mut self, value: u8) -> bool {
        self.data[TWCR] = value;
        let i = self.twi().interrupt;
        self.clear_interrupt_by_flag(i, value);
        self.update_interrupt_enable(i, value);
        let status = self.twi_status();
        if value & TWCR_TWINT != 0 && value & TWCR_TWEN != 0 && !self.twi().busy {
            let twdr = self.data[TWDR];
            self.add_clock_event_arg(EV_TWI, value as u32 | (status as u32) << 8 | (twdr as u32) << 16, 0);
            return true;
        }
        false
    }

    /// The clock event `hook_twcr` schedules.
    pub(crate) fn twi_event(&mut self, arg: u32) {
        let value = arg as u8;
        let status = (arg >> 8) as u8;
        let twdr = (arg >> 16) as u8;
        let e = if value & TWCR_TWSTA != 0 {
            TwiEvent::Start { repeated: status != STATUS_TWI_IDLE }
        } else if value & TWCR_TWSTO != 0 {
            TwiEvent::Stop
        } else if status == STATUS_START || status == STATUS_REPEATED_START {
            TwiEvent::Connect { addr: twdr >> 1, write: twdr & 1 == 0 }
        } else if status == STATUS_SLAW_ACK || status == STATUS_DATA_SENT_ACK {
            TwiEvent::WriteByte(twdr)
        } else if status == STATUS_SLAR_ACK || status == STATUS_DATA_RECEIVED_ACK {
            TwiEvent::ReadByte { ack: value & TWCR_TWEA != 0 }
        } else {
            return;
        };
        self.twi().busy = true;
        self.twi_handle(e);
    }
}
