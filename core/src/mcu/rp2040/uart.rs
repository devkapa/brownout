//! The two UARTs, ported from rp2040js 1.3.3 (MIT):
//! `src/peripherals/uart.ts`. Transmitted bytes and baud-rate changes go to
//! `Periph::uart_events` (the reference's `onByte` and `onBaudRateChange`
//! callbacks); the host feeds received bytes with [`Rp2040::uart_feed_byte`].
//!
//! Reference behaviour that departs from the PL011 datasheet (each has a
//! `reference_quirk_*` test): there is no TX FIFO (TXFE is always set and the
//! TX interrupt is raised on every interrupt check), no RX threshold (any
//! byte raises RXINTR), and the RX DMA request is never raised.

use super::clock::js_round;
use super::dma::dreq;
use super::usb::Fifo;
use super::Rp2040;

const UARTDR: u32 = 0x0;
const UARTFR: u32 = 0x18;
const UARTIBRD: u32 = 0x24;
const UARTFBRD: u32 = 0x28;
const UARTLCR_H: u32 = 0x2c;
const UARTCR: u32 = 0x30;
const UARTIMSC: u32 = 0x38;
const UARTIRIS: u32 = 0x3c;
const UARTIMIS: u32 = 0x40;
const UARTICR: u32 = 0x44;

const TXFE: u32 = 1 << 7;
const RXFF: u32 = 1 << 6;
const RXFE: u32 = 1 << 4;
const FEN: u32 = 1 << 4;
const RXE: u32 = 1 << 9;
const TXE: u32 = 1 << 8;
const UARTEN: u32 = 1;
const UARTTXINTR: u32 = 1 << 5;
const UARTRXINTR: u32 = 1 << 4;

const CLK_PERI: f64 = 125_000_000.0;
const IRQ_UART0: u32 = 20;

/// One entry of the host-visible UART stream.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UartEvent {
    pub uart: u8,
    /// 0: a byte was written to UARTDR (`value` is the byte); 1: the baud
    /// rate was recomputed after an IBRD or FBRD write (`value` is the rate).
    pub kind: u8,
    pub value: f64,
    pub nanos: f64,
}

pub struct Uart {
    pub ctrl: u32,
    pub line_ctrl: u32,
    pub rx_fifo: Fifo,
    pub int_mask: u32,
    pub int_status: u32,
    pub int_div: u32,
    pub frac_div: u32,
}

impl Default for Uart {
    fn default() -> Self {
        Self::new()
    }
}

impl Uart {
    pub fn new() -> Self {
        Uart {
            ctrl: RXE | TXE,
            line_ctrl: 0,
            rx_fifo: Fifo::new(32),
            int_mask: 0,
            int_status: 0,
            int_div: 0,
            frac_div: 0,
        }
    }

    pub fn enabled(&self) -> bool {
        self.ctrl & UARTEN != 0
    }
    pub fn fifos_enabled(&self) -> bool {
        self.line_ctrl & FEN != 0
    }
    /// Bits per character.
    pub fn word_length(&self) -> u32 {
        5 + ((self.line_ctrl >> 5) & 3)
    }
    pub fn baud_divider(&self) -> f64 {
        self.int_div as f64 + self.frac_div as f64 / 64.0
    }
    /// `Math.round(clkPeri / (divider * 16))`; infinite while the divider is 0.
    pub fn baud_rate(&self) -> f64 {
        js_round(CLK_PERI / (self.baud_divider() * 16.0))
    }
    pub fn flags(&self) -> u32 {
        (if self.rx_fifo.is_full() { RXFF } else { 0 }) | (if self.rx_fifo.is_empty() { RXFE } else { 0 }) | TXFE
    }
}

impl Rp2040 {
    fn uart_check_interrupts(&mut self, i: usize) {
        let u = &mut self.periph.uart[i];
        u.int_status |= UARTTXINTR;
        let level = u.int_status & u.int_mask != 0;
        self.set_interrupt(IRQ_UART0 + i as u32, level);
    }

    /// `feedByte`: a byte arrives on RX.
    pub fn uart_feed_byte(&mut self, i: usize, value: u8) {
        let u = &mut self.periph.uart[i];
        u.rx_fifo.push(value as u32);
        u.int_status |= UARTRXINTR;
        self.uart_check_interrupts(i);
    }

    fn uart_event(&mut self, i: usize, kind: u8, value: f64) {
        let nanos = self.clock.nanos;
        self.periph.uart_events.push(UartEvent { uart: i as u8, kind, value, nanos });
    }

    pub(crate) fn uart_read(&mut self, i: usize, offset: u32) -> Option<u32> {
        Some(match offset {
            UARTDR => {
                let u = &mut self.periph.uart[i];
                let v = u.rx_fifo.pull();
                if !u.rx_fifo.is_empty() {
                    u.int_status |= UARTRXINTR;
                } else {
                    u.int_status &= !UARTRXINTR;
                }
                self.uart_check_interrupts(i);
                v
            }
            UARTFR => self.periph.uart[i].flags(),
            UARTIBRD => self.periph.uart[i].int_div,
            UARTFBRD => self.periph.uart[i].frac_div,
            UARTLCR_H => self.periph.uart[i].line_ctrl,
            UARTCR => self.periph.uart[i].ctrl,
            UARTIMSC => self.periph.uart[i].int_mask,
            UARTIRIS => self.periph.uart[i].int_status,
            UARTIMIS => self.periph.uart[i].int_status & self.periph.uart[i].int_mask,
            0xfe0 => 0x11,
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

    /// `raw` is the reference's `rawWriteValue` (what UARTICR clears).
    pub(crate) fn uart_write(&mut self, i: usize, offset: u32, value: u32, raw: u32) -> bool {
        match offset {
            UARTDR => self.uart_event(i, 0, (value & 0xff) as f64),
            UARTIBRD => {
                self.periph.uart[i].int_div = value & 0xffff;
                let b = self.periph.uart[i].baud_rate();
                self.uart_event(i, 1, b);
            }
            UARTFBRD => {
                self.periph.uart[i].frac_div = value & 0x3f;
                let b = self.periph.uart[i].baud_rate();
                self.uart_event(i, 1, b);
            }
            UARTLCR_H => self.periph.uart[i].line_ctrl = value,
            UARTCR => {
                self.periph.uart[i].ctrl = value;
                let tx = if i == 0 { dreq::UART0_TX } else { dreq::UART1_TX };
                if self.periph.uart[i].enabled() {
                    self.dma_set_dreq(tx);
                } else {
                    self.dma_clear_dreq(tx);
                }
            }
            UARTIMSC => {
                self.periph.uart[i].int_mask = value & 0x7ff;
                self.uart_check_interrupts(i);
            }
            UARTICR => {
                self.periph.uart[i].int_status &= !raw;
                self.uart_check_interrupts(i);
            }
            _ => return false,
        }
        true
    }
}
