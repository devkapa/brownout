//! SPI, ported from avr8js 0.21.0 `peripherals/spi.js` (MIT, Copyright (c)
//! Uri Shaked and contributors; see NOTICE): `AVRSPI` with `spiConfig`.
//! The reference's `onByte` callback is `reply`: its default (`onTransfer`
//! answers 0, `completeTransfer` one byte time later) is `SpiReply::After(0)`;
//! `SpiReply::Host` leaves the transfer open for the host to finish with
//! `spi_complete_transfer`, as a device on the bus would.

use super::bus::BusEvent;
use super::cpu::{Cpu, EV_SPI_DONE};
use super::interrupt::Interrupt;

pub const SPCR: usize = 0x4c;
pub const SPSR: usize = 0x4d;
pub const SPDR: usize = 0x4e;

const SPCR_SPIE: u8 = 0x80;
const SPCR_SPE: u8 = 0x40;
const SPCR_DORD: u8 = 0x20;
const SPCR_MSTR: u8 = 0x10;
const SPCR_CPOL: u8 = 0x8;
const SPCR_CPHA: u8 = 0x4;
const SPSR_SPR_MASK: u8 = 0x3;
const SPSR_SPIF: u8 = 0x80;
const SPSR_WCOL: u8 = 0x40;
const SPSR_SPI2X: u8 = 0x1;
const BITS_PER_BYTE: i64 = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpiReply {
    /// The byte read back, `transferCycles` after the write.
    After(u8),
    /// The host calls `spi_complete_transfer`.
    Host,
}

pub struct Spi {
    pub freq_hz: u32,
    pub transmission_active: bool,
    pub reply: SpiReply,
    /// Bytes written to SPDR that started a transfer, while `Some`.
    pub sent: Option<Vec<u8>>,
    pub interrupt: Interrupt,
    /// The cycle the transfer in progress began on and the byte sent, for its
    /// bus event when a host-answered transfer completes.
    pub started: (u64, u8),
}

impl Spi {
    pub fn new(freq_hz: u32) -> Spi {
        Spi {
            freq_hz,
            transmission_active: false,
            reply: SpiReply::After(0),
            sent: None,
            started: (0, 0),
            interrupt: Interrupt {
                address: 0x22,
                flag_register: SPSR as u16,
                flag_mask: SPSR_SPIF,
                enable_register: SPCR as u16,
                enable_mask: SPCR_SPIE,
                constant: false,
                inverse_flag: false,
            },
        }
    }
}

impl Cpu {
    fn spi(&self) -> &Spi {
        self.p.spi.as_ref().unwrap()
    }

    pub fn spi_is_master(&self) -> bool {
        self.data[SPCR] & SPCR_MSTR != 0
    }

    pub fn spi_lsb_first(&self) -> bool {
        self.data[SPCR] & SPCR_DORD != 0
    }

    /// `spiMode`, as the reference computes it (2*CPHA + CPOL).
    pub fn spi_mode(&self) -> u8 {
        let cpha = self.data[SPCR] & SPCR_CPHA != 0;
        let cpol = self.data[SPCR] & SPCR_CPOL != 0;
        (if cpha { 2 } else { 0 }) | (cpol as u8)
    }

    pub fn spi_clock_divider(&self) -> i64 {
        let base = if self.data[SPSR] & SPSR_SPI2X != 0 { 2 } else { 4 };
        match self.data[SPCR] & SPSR_SPR_MASK {
            0 => base,
            1 => base * 4,
            2 => base * 16,
            _ => base * 32,
        }
    }

    pub fn spi_transfer_cycles(&self) -> i64 {
        self.spi_clock_divider() * BITS_PER_BYTE
    }

    pub fn spi_frequency(&self) -> f64 {
        self.spi().freq_hz as f64 / self.spi_clock_divider() as f64
    }

    /// `completeTransfer(receivedByte)`.
    pub fn spi_complete_transfer(&mut self, received: u8) {
        if self.spi().reply == SpiReply::Host {
            let (cycle, mosi) = self.spi().started;
            let cycles = self.spi_transfer_cycles();
            self.bus_push(BusEvent::Spi { cycle, mosi, miso: received, cycles });
        }
        self.data[SPDR] = received;
        let i = self.spi().interrupt;
        self.set_interrupt_flag(i);
        self.p.spi.as_mut().unwrap().transmission_active = false;
    }

    /// `writeHooks[SPDR]`. With SPE clear the write is not consumed (the byte
    /// lands in SPDR); a started transfer is, so SPDR keeps its old value
    /// until the transfer completes.
    pub(crate) fn hook_spdr(&mut self, value: u8) -> bool {
        if self.data[SPCR] & SPCR_SPE == 0 {
            return false;
        }
        if self.spi().transmission_active {
            self.data[SPSR] |= SPSR_WCOL;
            return true;
        }
        self.data[SPSR] &= !SPSR_WCOL;
        let i = self.spi().interrupt;
        self.clear_interrupt(i, true);
        let now = self.cycles;
        let s = self.p.spi.as_mut().unwrap();
        s.transmission_active = true;
        s.started = (now, value);
        if let Some(sent) = s.sent.as_mut() {
            sent.push(value);
        }
        if let SpiReply::After(b) = s.reply {
            let cycles = self.spi_transfer_cycles();
            // ArduinoMcu records the exchange as it starts (its reply is known then).
            self.bus_push(BusEvent::Spi { cycle: now, mosi: value, miso: b, cycles });
            self.add_clock_event_arg(EV_SPI_DONE, b as u32, cycles);
        }
        true
    }

    /// `writeHooks[SPCR]`: not consumed.
    pub(crate) fn hook_spcr(&mut self, value: u8) -> bool {
        let i = self.spi().interrupt;
        self.update_interrupt_enable(i, value);
        false
    }

    /// `writeHooks[SPSR]`: not consumed. Reference quirk: the plain store
    /// after the hook writes the value back, so a 1 written to SPIF clears
    /// the pending interrupt but leaves the bit set.
    pub(crate) fn hook_spsr(&mut self, value: u8) -> bool {
        self.data[SPSR] = value;
        let i = self.spi().interrupt;
        self.clear_interrupt_by_flag(i, value);
        false
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for SpiReply {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            SpiReply::After(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            SpiReply::Host => 0,
        }
    }
}
