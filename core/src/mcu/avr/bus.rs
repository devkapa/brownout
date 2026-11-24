//! Bus events for protocol decoders: brownout's `ArduinoMcu.drainBusEvents`
//! (`McuBusEvent`), on the core. Each UART frame, I2C step and SPI exchange
//! is recorded with the cycle it began on by the peripheral that carries it,
//! which knows each byte exactly; sampling the pins would see an MCU-driven
//! net once per engine step and miss most bits. The log is a ring, as the
//! serial output is: a board nobody decodes keeps only the newest events.

use std::collections::VecDeque;

use super::cpu::Cpu;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum I2cKind {
    Start,
    Restart,
    Stop,
    /// The address byte after a start; `ack` is the device's.
    Address { address: u8, read: bool, ack: bool },
    /// A byte the board wrote; `ack` is the device's.
    Write { value: u8, ack: bool },
    /// A byte the board read; `ack` is the board's (false on the last byte).
    Read { value: u8, ack: bool },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BusEvent {
    /// A frame the program sent (`tx`) or that began arriving at it, lasting
    /// `cycles` at the configured baud rate.
    Uart { cycle: u64, tx: bool, value: u8, cycles: i64 },
    I2c { cycle: u64, kind: I2cKind },
    /// One exchange as bus master: the byte shifted out and the one in.
    Spi { cycle: u64, mosi: u8, miso: u8, cycles: i64 },
}

impl BusEvent {
    pub fn cycle(&self) -> u64 {
        match *self {
            BusEvent::Uart { cycle, .. } | BusEvent::I2c { cycle, .. } | BusEvent::Spi { cycle, .. } => cycle,
        }
    }
}

pub struct BusLog {
    pub events: VecDeque<BusEvent>,
    /// `ArduinoMcu.BUS_EVENT_CAPACITY`.
    pub capacity: usize,
    /// Events dropped, oldest first, since the board was built.
    pub dropped: u64,
}

impl Default for BusLog {
    fn default() -> BusLog {
        BusLog { events: VecDeque::new(), capacity: 4096, dropped: 0 }
    }
}

impl Cpu {
    pub(crate) fn bus_push(&mut self, e: BusEvent) {
        let log = &mut self.p.bus;
        if log.capacity == 0 {
            log.dropped += 1;
            return;
        }
        if log.events.len() == log.capacity {
            log.events.pop_front();
            log.dropped += 1;
        }
        log.events.push_back(e);
    }

    /// Host side: `ArduinoMcu.drainBusEvents().events`.
    pub fn bus_drain(&mut self) -> Vec<BusEvent> {
        std::mem::take(&mut self.p.bus.events).into()
    }

    /// Host side: `ArduinoMcu.drainBusEvents().dropped`.
    pub fn bus_dropped(&self) -> u64 {
        self.p.bus.dropped
    }
}
