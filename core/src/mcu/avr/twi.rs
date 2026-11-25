//! TWI (I2C), ported from avr8js 0.21.0 `peripherals/twi.js` (MIT, Copyright
//! (c) Uri Shaked and contributors; see NOTICE): `AVRTWI` with `twiConfig`,
//! master mode only, as the reference. The event handler is `handler`:
//! `TwiHandler::Noop` is the reference's `NoopTWIEventHandler` (what
//! `ArduinoMcu` keeps: every address NACKs, as an empty bus does);
//! `TwiHandler::Host` leaves each step for the host to finish with the
//! `twi_complete_*` calls, as a device on the bus would. `events` records
//! the handler calls while it is `Some`. `TwiHandler::Timed` is what
//! `ArduinoMcu` installs (its avr-bus-timing.ts, a departure from avr8js
//! toward the 328P): the empty bus's answers, each given only after the
//! step's bus time at the SCL rate (a byte nine SCL periods, a START from
//! idle half of one, a repeated START one and a half, a STOP one), with the
//! step recorded as it starts, a START after a STOP held for the bus-free
//! time (t_BUF), and TWINT, TWSTO and TWSR where the datasheet has them:
//! clear out of reset, not raised by a STOP, cleared only by writing a one,
//! and TWSR's status bits read-only. Under the timed handler the board's
//! parts (`parts.rs`) answer their addresses.

use super::bus::{BusEvent, I2cKind};
use super::cpu::{Cpu, EV_TWI, EV_TWI_DONE};
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
    Timed,
}

// `EV_TWI_DONE`'s argument: which step finishes.
const TWI_DONE_START: u32 = 0;
const TWI_DONE_STOP: u32 = 1;
const TWI_DONE_CONNECT: u32 = 2;
const TWI_DONE_WRITE: u32 = 3;
const TWI_DONE_READ: u32 = 4;

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
    /// The cycle the step in progress began on: a bus event is recorded when
    /// the step completes (only then is the device's answer known) but dated
    /// from its start, as `ArduinoMcu` dates it.
    pub step_cycle: u64,
    /// Steps the timed handler has started and a running FNV-1a over each
    /// one's bus event as the lock-step harness encodes it (kind, start
    /// cycle, length, then address, read and ack or value and ack), which it
    /// compares with the classic engine's.
    pub timed_steps: u64,
    pub timed_hash: u32,
    /// The cycle the last STOP left the bus on (timed handler only).
    pub stop_ended: Option<u64>,
    /// The part (`p.parts.i2c` index, and the address it answered) the last
    /// address reached, until the STOP or the next address (timed handler).
    pub addressed: Option<(usize, u8)>,
}

impl Twi {
    pub fn new(freq_hz: u32) -> Twi {
        Twi {
            freq_hz,
            busy: false,
            handler: TwiHandler::Noop,
            events: None,
            step_cycle: 0,
            timed_steps: 0,
            timed_hash: 0x811c9dc5,
            stop_ended: None,
            addressed: None,
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

    /// A step the host finished: recorded now, dated from its start.
    fn twi_bus_event(&mut self, kind: I2cKind) {
        let cycle = self.twi().step_cycle;
        let cycles = (self.cycles - cycle) as i64;
        self.bus_push(BusEvent::I2c { cycle, cycles, kind });
    }

    pub fn twi_complete_start(&mut self) {
        let idle = self.twi_status() == STATUS_TWI_IDLE;
        self.twi_bus_event(if idle { I2cKind::Start } else { I2cKind::Restart });
        self.twi_finish_start();
    }

    pub fn twi_complete_stop(&mut self) {
        self.twi_bus_event(I2cKind::Stop);
        self.twi_finish_stop();
    }

    pub fn twi_complete_connect(&mut self, ack: bool) {
        let twdr = self.data[TWDR];
        self.twi_bus_event(I2cKind::Address { address: twdr >> 1, read: twdr & 1 != 0, ack });
        self.twi_finish_connect(ack);
    }

    pub fn twi_complete_write(&mut self, ack: bool) {
        let value = self.data[TWDR];
        self.twi_bus_event(I2cKind::Write { value, ack });
        self.twi_finish_write(ack);
    }

    pub fn twi_complete_read(&mut self, value: u8) {
        let ack = self.data[TWCR] & TWCR_TWEA != 0;
        self.twi_bus_event(I2cKind::Read { value, ack });
        self.twi_finish_read(value);
    }

    // The reference's `complete*` bodies, without the record.

    fn twi_finish_start(&mut self) {
        self.twi().busy = false;
        let idle = self.twi_status() == STATUS_TWI_IDLE;
        let s = if idle { STATUS_START } else { STATUS_REPEATED_START };
        self.twi_update_status(s);
    }

    fn twi_finish_stop(&mut self) {
        self.twi().busy = false;
        self.data[TWCR] &= !TWCR_TWSTO;
        self.twi_update_status(STATUS_TWI_IDLE);
    }

    fn twi_finish_connect(&mut self, ack: bool) {
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

    fn twi_finish_write(&mut self, ack: bool) {
        self.twi().busy = false;
        self.twi_update_status(if ack { STATUS_DATA_SENT_ACK } else { STATUS_DATA_SENT_NACK });
    }

    fn twi_finish_read(&mut self, value: u8) {
        self.twi().busy = false;
        let ack = self.data[TWCR] & TWCR_TWEA != 0;
        self.data[TWDR] = value;
        self.twi_update_status(if ack { STATUS_DATA_RECEIVED_ACK } else { STATUS_DATA_RECEIVED_NACK });
    }

    /// One SCL period in CPU cycles, `16 + 2 * TWBR * 4^TWPS`: always even.
    pub fn twi_scl_cycles(&self) -> i64 {
        16 + 2 * self.data[TWBR] as i64 * self.twi_prescaler() as i64
    }

    /// The bus-free time between a STOP and the next START (t_BUF), in whole
    /// cycles rounded up: 4.7 us up to 100 kHz, 1.3 us above. Integer
    /// arithmetic, as avr-bus-timing.ts's `twiBusFreeCycles`.
    pub fn twi_bus_free_cycles(&self, scl: i64) -> i64 {
        let f = self.p.twi.as_ref().unwrap().freq_hz as i64;
        let tenths_of_us = if scl * 100_000 >= f { 47 } else { 13 };
        (f * tenths_of_us + 9_999_999) / 10_000_000
    }

    /// The timed handler: records the step as it starts on the bus, with the
    /// empty bus's answer, and finishes it after its bus time.
    fn twi_timed(&mut self, e: TwiEvent) {
        let scl = self.twi_scl_cycles();
        let wait = match (e, self.twi().stop_ended) {
            (TwiEvent::Start { repeated: false }, Some(ended)) => {
                (ended as i64 + self.twi_bus_free_cycles(scl) - self.cycles as i64).max(0)
            }
            _ => 0,
        };
        // The parts on the bus (parts.rs) answer their addresses; any other
        // address is NACKed and a read gets 0xFF, as on an empty bus. The
        // addressed part hears the START as its address is acknowledged and
        // each byte as the step carrying it starts.
        let (cycles, kind, done) = match e {
            TwiEvent::Start { repeated: false } => (scl / 2, I2cKind::Start, TWI_DONE_START),
            TwiEvent::Start { repeated: true } => (scl * 3 / 2, I2cKind::Restart, TWI_DONE_START),
            TwiEvent::Stop => {
                if let Some(i) = self.twi_current() {
                    self.twi_target_stop(i);
                }
                self.twi().addressed = None;
                (scl, I2cKind::Stop, TWI_DONE_STOP)
            }
            TwiEvent::Connect { addr, write } => {
                let target = self.twi_target(addr);
                self.twi().addressed = target.map(|i| (i, addr));
                if let Some(i) = target {
                    self.twi_target_start(i);
                }
                let ack = target.is_some();
                (9 * scl, I2cKind::Address { address: addr, read: !write, ack }, TWI_DONE_CONNECT | (ack as u32) << 8)
            }
            TwiEvent::WriteByte(value) => {
                let ack = match self.twi_current() {
                    Some(i) => self.twi_target_write(i, value),
                    None => false,
                };
                (9 * scl, I2cKind::Write { value, ack }, TWI_DONE_WRITE | (ack as u32) << 8)
            }
            TwiEvent::ReadByte { ack } => {
                // Nothing driving SDA reads as the pull-ups' ones.
                let value = match self.twi_current() {
                    Some(i) => self.twi_target_read(i),
                    None => 0xff,
                };
                (9 * scl, I2cKind::Read { value, ack }, TWI_DONE_READ | (value as u32) << 8)
            }
        };
        let cycle = self.cycles + wait as u64;
        self.bus_push(BusEvent::I2c { cycle, cycles, kind });
        let (code, x, y, z) = match kind {
            I2cKind::Start => (2, 0, 0, 0),
            I2cKind::Restart => (3, 0, 0, 0),
            I2cKind::Stop => (4, 0, 0, 0),
            I2cKind::Address { address, read, ack } => (5, address as u32, read as u32, ack as u32),
            I2cKind::Write { value, ack } => (6, value as u32, ack as u32, 0),
            I2cKind::Read { value, ack } => (7, value as u32, ack as u32, 0),
        };
        let t = self.twi();
        t.timed_steps += 1;
        for word in [code, cycle as u32, cycles as u32, x, y, z] {
            t.timed_hash = (t.timed_hash ^ word).wrapping_mul(0x01000193);
        }
        self.add_clock_event_arg(EV_TWI_DONE, done, wait + cycles);
    }

    /// The clock event `twi_timed` schedules: the step's bus time is up.
    /// The argument's low byte is the step, the next its answer (the ack, or
    /// the byte read).
    pub(crate) fn twi_done_event(&mut self, arg: u32) {
        let answer = (arg >> 8) as u8;
        match arg & 0xff {
            TWI_DONE_START => self.twi_finish_start(),
            TWI_DONE_STOP => {
                // The reference's completeStop without the TWINT it raises:
                // TWSR reads 0xF8 with TWINT clear.
                self.twi().busy = false;
                self.data[TWCR] &= !TWCR_TWSTO;
                self.data[TWSR] = (self.data[TWSR] & !TWSR_TWS_MASK) | STATUS_TWI_IDLE;
                let now = self.cycles;
                self.twi().stop_ended = Some(now);
            }
            TWI_DONE_CONNECT => self.twi_finish_connect(answer != 0),
            TWI_DONE_WRITE => self.twi_finish_write(answer != 0),
            _ => self.twi_finish_read(answer),
        }
    }

    /// `writeHooks[TWSR]` under the timed handler: only the prescaler bits
    /// take a write; the status bits are the hardware's.
    pub(crate) fn hook_twsr(&mut self, value: u8, old: u8) -> bool {
        self.data[TWSR] = (old & !TWSR_TWPS_MASK) | (value & TWSR_TWPS_MASK);
        true
    }

    /// One handler call: recorded, then answered at once by the no-op
    /// handler or left to the host.
    fn twi_handle(&mut self, e: TwiEvent) {
        let now = self.cycles;
        let t = self.twi();
        t.step_cycle = now;
        if let Some(ev) = t.events.as_mut() {
            ev.push(e);
        }
        if t.handler == TwiHandler::Host {
            return;
        }
        if t.handler == TwiHandler::Timed {
            self.twi_timed(e);
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
        let old = self.data[TWCR];
        let consumed = self.hook_twcr_reference(value);
        if self.twi().handler != TwiHandler::Timed {
            return consumed;
        }
        // The reference stores TWCR whole, so a zero written to TWINT clears
        // it and a write it does not act on stores a one there; on the chip
        // only writing a one clears it.
        let twint = if value & TWCR_TWINT != 0 { 0 } else { old & TWCR_TWINT };
        self.data[TWCR] = (value & !TWCR_TWINT) | twint;
        if twint != 0 {
            let i = self.twi().interrupt;
            self.set_interrupt_flag(i);
        }
        true
    }

    fn hook_twcr_reference(&mut self, value: u8) -> bool {
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

    /// The clock event `hook_twcr` schedules. As twi.js 0.21.0: a status of
    /// START or REPEATED_START (no TWSTA or TWSTO in the write) sends the
    /// address in TWDR, so both connect; other statuses do nothing.
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

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for TwiHandler {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            TwiHandler::Noop => 0,
            TwiHandler::Host => 0,
            TwiHandler::Timed => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for TwiEvent {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            TwiEvent::Start { repeated } => crate::snapshot::HeapBytes::heap_bytes(repeated, w),
            TwiEvent::Stop => 0,
            TwiEvent::Connect { addr, write } => crate::snapshot::HeapBytes::heap_bytes(addr, w) + crate::snapshot::HeapBytes::heap_bytes(write, w),
            TwiEvent::WriteByte(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            TwiEvent::ReadByte { ack } => crate::snapshot::HeapBytes::heap_bytes(ack, w),
        }
    }
}
