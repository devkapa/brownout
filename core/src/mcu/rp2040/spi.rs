//! The two SPI controllers (PL022), ported from rp2040js 1.3.3 (MIT):
//! `src/peripherals/spi.ts`. The reference hands each transmitted word to a
//! host callback (`onTransmit`) that must answer with `completeTransmit`;
//! here the callback is [`SpiHost`]: the reference's default (answer at once
//! with zero), an answer XORed with a constant (a scripted loopback device),
//! or `Manual`, where the host calls [`Rp2040::spi_complete_transmit`] itself.
//! Every transmitted word is logged in `Periph::spi_events`.
//!
//! The constructor's DMA lines (TX raised, RX lowered) are set in
//! `Periph::new` because the lock-step needs them at step 0.
//!
//! Reference behaviour that departs from the PL022 datasheet (each has a
//! `reference_quirk_*` test): transmission ignores SSE, data written to a full
//! TX FIFO is dropped without a status flag, the TX interrupt starts clear,
//! IMSC and SSPDMACR are stored unmasked.

use super::dma::dreq;
use super::usb::Fifo;
use super::Rp2040;

const SSPCR0: u32 = 0x000;
const SSPCR1: u32 = 0x004;
const SSPDR: u32 = 0x008;
const SSPSR: u32 = 0x00c;
const SSPCPSR: u32 = 0x010;
const SSPIMSC: u32 = 0x014;
const SSPRIS: u32 = 0x018;
const SSPMIS: u32 = 0x01c;
const SSPICR: u32 = 0x020;
const SSPDMACR: u32 = 0x024;

const SPH: u32 = 1 << 7;
const SPO: u32 = 1 << 6;
const SSE: u32 = 1 << 1;
const MS: u32 = 1 << 2;
const BSY: u32 = 1 << 4;
const RFF: u32 = 1 << 3;
const RNE: u32 = 1 << 2;
const TNF: u32 = 1 << 1;
const TFE: u32 = 1 << 0;
const SSPTXINTR: u32 = 1 << 3;
const SSPRXINTR: u32 = 1 << 2;
const SSPRTINTR: u32 = 1 << 1;
const SSPRORINTR: u32 = 1 << 0;
const IRQ_SPI0: u32 = 18;
const CLK_PERI: f64 = 125_000_000.0;

/// What answers a transmitted word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpiHost {
    /// `completeTransmit(word ^ xor)` at once; `Respond(0)` is the reference's default.
    Respond(u32),
    /// Nothing: the host completes the transfer later.
    Manual,
}

pub struct Spi {
    pub rx_fifo: Fifo,
    pub tx_fifo: Fifo,
    pub busy: bool,
    pub control0: u32,
    pub control1: u32,
    pub dma_control: u32,
    pub clock_divisor: u32,
    pub int_raw: u32,
    pub int_enable: u32,
    pub host: SpiHost,
}

impl Default for Spi {
    fn default() -> Self {
        Self::new()
    }
}

impl Spi {
    pub fn new() -> Self {
        Spi {
            rx_fifo: Fifo::new(8),
            tx_fifo: Fifo::new(8),
            busy: false,
            control0: 0,
            control1: 0,
            dma_control: 0,
            clock_divisor: 0,
            int_raw: 0,
            int_enable: 0,
            host: SpiHost::Respond(0),
        }
    }
    pub fn int_status(&self) -> u32 {
        self.int_raw & self.int_enable
    }
    pub fn enabled(&self) -> bool {
        self.control1 & SSE != 0
    }
    /// Data size in bits: 1 to 16 (the field is DSS + 1).
    pub fn data_bits(&self) -> u32 {
        (self.control0 & 0xf) + 1
    }
    pub fn master_mode(&self) -> bool {
        self.control0 & MS == 0
    }
    pub fn spi_mode(&self) -> u32 {
        let (cpol, cpha) = (self.control0 & SPO != 0, self.control0 & SPH != 0);
        if cpol { if cpha { 2 } else { 3 } } else if cpha { 1 } else { 0 }
    }
    pub fn clock_frequency(&self) -> f64 {
        if self.clock_divisor == 0 {
            return 0.0;
        }
        let scr = (self.control0 >> 8) & 0xff;
        CLK_PERI / (self.clock_divisor as f64 * (1 + scr) as f64)
    }
}

/// A word the host saw leave an SPI block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpiEvent {
    pub spi: u8,
    pub value: u32,
    pub nanos: f64,
}

impl Rp2040 {
    fn spi_dreq(i: usize) -> (u32, u32) {
        if i == 0 { (dreq::SPI0_TX, dreq::SPI0_RX) } else { (dreq::SPI1_TX, dreq::SPI1_RX) }
    }

    fn spi_update_dma_tx(&mut self, i: usize) {
        let (tx, _) = Self::spi_dreq(i);
        if self.periph.spi[i].tx_fifo.is_full() {
            self.dma_clear_dreq(tx);
        } else {
            self.dma_set_dreq(tx);
        }
    }

    fn spi_update_dma_rx(&mut self, i: usize) {
        let (_, rx) = Self::spi_dreq(i);
        if self.periph.spi[i].rx_fifo.is_empty() {
            self.dma_clear_dreq(rx);
        } else {
            self.dma_set_dreq(rx);
        }
    }

    fn spi_check_interrupts(&mut self, i: usize) {
        let s = self.periph.spi[i].int_status() != 0;
        self.set_interrupt(IRQ_SPI0 + i as u32, s);
    }

    fn spi_fifos_updated(&mut self, i: usize) {
        let s = &mut self.periph.spi[i];
        let prev = s.int_status();
        if s.tx_fifo.item_count() <= s.tx_fifo.size() / 2 {
            s.int_raw |= SSPTXINTR;
        } else {
            s.int_raw &= !SSPTXINTR;
        }
        if s.rx_fifo.item_count() >= s.rx_fifo.size() / 2 {
            s.int_raw |= SSPRXINTR;
        } else {
            s.int_raw &= !SSPRXINTR;
        }
        if s.int_status() != prev {
            self.spi_check_interrupts(i);
        }
        self.spi_update_dma_tx(i);
        self.spi_update_dma_rx(i);
    }

    fn spi_do_tx(&mut self, i: usize) {
        let s = &mut self.periph.spi[i];
        if !s.busy && !s.tx_fifo.is_empty() {
            let value = s.tx_fifo.pull();
            s.busy = true;
            let host = s.host;
            let nanos = self.clock.nanos;
            self.periph.spi_events.push(SpiEvent { spi: i as u8, value, nanos });
            if let SpiHost::Respond(xor) = host {
                self.spi_complete_transmit(i, value ^ xor);
            }
            self.spi_fifos_updated(i);
        }
    }

    /// `completeTransmit`: the host's answer for the word in flight.
    pub fn spi_complete_transmit(&mut self, i: usize, rx_value: u32) {
        let s = &mut self.periph.spi[i];
        s.busy = false;
        if !s.rx_fifo.is_full() {
            s.rx_fifo.push(rx_value);
        } else {
            s.int_raw |= SSPRORINTR;
        }
        self.spi_fifos_updated(i);
        self.spi_do_tx(i);
    }

    pub(crate) fn spi_read(&mut self, i: usize, offset: u32) -> Option<u32> {
        Some(match offset {
            SSPCR0 => self.periph.spi[i].control0,
            SSPCR1 => self.periph.spi[i].control1,
            SSPDR => {
                if !self.periph.spi[i].rx_fifo.is_empty() {
                    let v = self.periph.spi[i].rx_fifo.pull();
                    self.spi_fifos_updated(i);
                    v
                } else {
                    0
                }
            }
            SSPSR => {
                let s = &self.periph.spi[i];
                (if s.busy || !s.tx_fifo.is_empty() { BSY } else { 0 })
                    | (if s.rx_fifo.is_full() { RFF } else { 0 })
                    | (if !s.rx_fifo.is_empty() { RNE } else { 0 })
                    | (if !s.tx_fifo.is_full() { TNF } else { 0 })
                    | (if s.tx_fifo.is_empty() { TFE } else { 0 })
            }
            SSPCPSR => self.periph.spi[i].clock_divisor,
            SSPIMSC => self.periph.spi[i].int_enable,
            SSPRIS => self.periph.spi[i].int_raw,
            SSPMIS => self.periph.spi[i].int_status(),
            SSPDMACR => self.periph.spi[i].dma_control,
            0xfe0 => 0x22,
            0xfe4 => 0x10,
            0xfe8 => 0x34,
            0xfec => 0x00,
            0xff0 => 0x0d,
            0xff4 => 0xf0,
            0xff8 => 0x05,
            0xffc => 0xb1,
            _ => return None,
        })
    }

    pub(crate) fn spi_write(&mut self, i: usize, offset: u32, value: u32) -> bool {
        match offset {
            SSPCR0 => self.periph.spi[i].control0 = value,
            SSPCR1 => self.periph.spi[i].control1 = value,
            SSPDR => {
                if !self.periph.spi[i].tx_fifo.is_full() {
                    let bits = self.periph.spi[i].data_bits();
                    let mask = (1u32 << bits).wrapping_sub(1);
                    self.periph.spi[i].tx_fifo.push(value & mask);
                    self.spi_do_tx(i);
                    self.spi_fifos_updated(i);
                }
            }
            SSPCPSR => self.periph.spi[i].clock_divisor = value & 0xfe,
            SSPIMSC => {
                self.periph.spi[i].int_enable = value;
                self.spi_check_interrupts(i);
            }
            SSPDMACR => self.periph.spi[i].dma_control = value,
            SSPICR => {
                self.periph.spi[i].int_raw &= !(value & (SSPRTINTR | SSPRORINTR));
                self.spi_check_interrupts(i);
            }
            _ => return false,
        }
        true
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for SpiHost {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            SpiHost::Respond(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            SpiHost::Manual => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for SpiEvent {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.spi, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nanos, w); }
        n
    }
}
