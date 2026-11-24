//! USART0, ported from avr8js 0.21.0 `peripherals/usart.js` (MIT, Copyright
//! (c) Uri Shaked and contributors; see NOTICE): `AVRUSART` with
//! `usart0Config`. The reference's callbacks become host-side records:
//! `onByteTransmit` appends to `tx` (what brownout's `ArduinoMcu` does with
//! it), `onLineTransmit` to `lines` while `line_buffer` is `Some`,
//! `onConfigurationChange` counts in `config_changes`. `onRxComplete` is
//! `ArduinoMcu`'s `_feedSerialInput`: the next byte of `rx_queue` goes to
//! `writeByte`, so typed bytes arrive one frame apart.

use std::collections::VecDeque;

use super::cpu::{Cpu, EV_USART_RX, EV_USART_TX};
use super::interrupt::Interrupt;

pub const UCSRA: usize = 0xc0;
pub const UCSRB: usize = 0xc1;
pub const UCSRC: usize = 0xc2;
pub const UBRRL: usize = 0xc4;
pub const UBRRH: usize = 0xc5;
pub const UDR: usize = 0xc6;

const UCSRA_RXC: u8 = 0x80;
const UCSRA_TXC: u8 = 0x40;
const UCSRA_UDRE: u8 = 0x20;
const UCSRA_U2X: u8 = 0x2;
const UCSRA_MPCM: u8 = 0x1;
const UCSRA_CFG_MASK: u8 = UCSRA_U2X;
const UCSRB_RXCIE: u8 = 0x80;
const UCSRB_TXCIE: u8 = 0x40;
const UCSRB_UDRIE: u8 = 0x20;
const UCSRB_RXEN: u8 = 0x10;
const UCSRB_TXEN: u8 = 0x8;
const UCSRB_UCSZ2: u8 = 0x4;
const UCSRB_CFG_MASK: u8 = UCSRB_UCSZ2 | UCSRB_RXEN | UCSRB_TXEN;
const UCSRC_UPM1: u8 = 0x20;
const UCSRC_UPM0: u8 = 0x10;
const UCSRC_USBS: u8 = 0x8;
const UCSRC_UCSZ1: u8 = 0x4;
const UCSRC_UCSZ0: u8 = 0x2;

pub struct Usart {
    pub freq_hz: u32,
    /// `rxBusyValue`: a received frame is on its way.
    pub rx_busy: bool,
    pub rx_byte: u8,
    /// Every byte the sketch has sent since the host last drained this.
    pub tx: Vec<u8>,
    /// Bytes sent since construction and a running FNV-1a over them (the
    /// lock-step harness compares both; `tx` is drained, these are not).
    pub tx_count: u64,
    pub tx_hash: u32,
    /// Bytes the host has typed that the receiver has not taken yet.
    pub rx_queue: VecDeque<u8>,
    /// `lineBuffer`, kept only while an `onLineTransmit` stands in (`Some`).
    pub line_buffer: Option<String>,
    pub lines: Vec<String>,
    pub config_changes: u32,
    pub rx_completes: u32,
    pub rxc: Interrupt,
    pub udre: Interrupt,
    pub txc: Interrupt,
}

impl Usart {
    pub fn new(freq_hz: u32) -> Usart {
        let irq = |address: u8, flag_mask: u8, enable_mask: u8, constant: bool| Interrupt {
            address,
            flag_register: UCSRA as u16,
            flag_mask,
            enable_register: UCSRB as u16,
            enable_mask,
            constant,
            inverse_flag: false,
        };
        Usart {
            freq_hz,
            rx_busy: false,
            rx_byte: 0,
            tx: Vec::new(),
            tx_count: 0,
            tx_hash: 0x811c9dc5,
            rx_queue: VecDeque::new(),
            line_buffer: None,
            lines: Vec::new(),
            config_changes: 0,
            rx_completes: 0,
            rxc: irq(0x24, UCSRA_RXC, UCSRB_RXCIE, true),
            udre: irq(0x26, UCSRA_UDRE, UCSRB_UDRIE, false),
            txc: irq(0x28, UCSRA_TXC, UCSRB_TXCIE, false),
        }
    }
}

impl Cpu {
    fn usart(&self) -> &Usart {
        self.p.usart.as_ref().unwrap()
    }

    fn usart_mut(&mut self) -> &mut Usart {
        self.p.usart.as_mut().unwrap()
    }

    /// `reset()`, which the constructor calls.
    pub(crate) fn usart_reset(&mut self) {
        self.data[UCSRA] = UCSRA_UDRE;
        self.data[UCSRB] = 0;
        self.data[UCSRC] = UCSRC_UCSZ1 | UCSRC_UCSZ0;
        let u = self.usart_mut();
        u.rx_busy = false;
        u.rx_byte = 0;
        if u.line_buffer.is_some() {
            u.line_buffer = Some(String::new());
        }
    }

    pub fn usart_ubrr(&self) -> u32 {
        ((self.data[UBRRH] as u32) << 8) | self.data[UBRRL] as u32
    }

    pub fn usart_multiplier(&self) -> u32 {
        if self.data[UCSRA] & UCSRA_U2X != 0 {
            8
        } else {
            16
        }
    }

    pub fn usart_rx_enable(&self) -> bool {
        self.data[UCSRB] & UCSRB_RXEN != 0
    }

    pub fn usart_tx_enable(&self) -> bool {
        self.data[UCSRB] & UCSRB_TXEN != 0
    }

    pub fn usart_baud_rate(&self) -> u32 {
        self.usart().freq_hz / (self.usart_multiplier() * (1 + self.usart_ubrr()))
    }

    /// `bitsPerChar`: UCSZ 4-6 are reserved and read as 9 bits, as 7 does.
    pub fn usart_bits_per_char(&self) -> u32 {
        let ucsz = ((self.data[UCSRC] & (UCSRC_UCSZ1 | UCSRC_UCSZ0)) >> 1) | (self.data[UCSRB] & UCSRB_UCSZ2);
        match ucsz {
            0 => 5,
            1 => 6,
            2 => 7,
            3 => 8,
            _ => 9,
        }
    }

    pub fn usart_stop_bits(&self) -> u32 {
        if self.data[UCSRC] & UCSRC_USBS != 0 {
            2
        } else {
            1
        }
    }

    pub fn usart_parity_enabled(&self) -> bool {
        self.data[UCSRC] & UCSRC_UPM1 != 0
    }

    pub fn usart_parity_odd(&self) -> bool {
        self.data[UCSRC] & UCSRC_UPM0 != 0
    }

    /// `cyclesPerChar`: start bit, data bits, stop bits and parity, each
    /// `(UBRR + 1) * multiplier` cycles.
    pub fn usart_cycles_per_char(&self) -> i64 {
        let symbols = 1 + self.usart_bits_per_char() + self.usart_stop_bits() + self.usart_parity_enabled() as u32;
        ((self.usart_ubrr() + 1) * self.usart_multiplier() * symbols) as i64
    }

    /// `writeByte(value, immediate)`. The reference returns `undefined` from
    /// the immediate path, which is false here.
    pub fn usart_write_byte(&mut self, value: u8, immediate: bool) -> bool {
        if self.usart().rx_busy || !self.usart_rx_enable() {
            return false;
        }
        if immediate {
            self.usart_mut().rx_byte = value;
            let rxc = self.usart().rxc;
            self.set_interrupt_flag(rxc);
            self.usart_rx_complete();
            false
        } else {
            self.usart_mut().rx_busy = true;
            let cycles = self.usart_cycles_per_char();
            self.add_clock_event_arg(EV_USART_RX, value as u32, cycles);
            true
        }
    }

    /// `onRxComplete`, as `ArduinoMcu` installs it.
    fn usart_rx_complete(&mut self) {
        self.usart_mut().rx_completes += 1;
        self.serial_feed();
    }

    /// `ArduinoMcu._feedSerialInput`: offers the next queued byte to the
    /// receiver, which refuses while a frame is arriving or RXEN is clear.
    fn serial_feed(&mut self) {
        if let Some(&b) = self.usart().rx_queue.front() {
            if self.usart_write_byte(b, false) {
                self.usart_mut().rx_queue.pop_front();
            }
        }
    }

    /// Host side: `ArduinoMcu.writeSerialInput(bytes)`.
    pub fn serial_write(&mut self, bytes: &[u8]) {
        if self.p.usart.is_none() {
            return;
        }
        self.usart_mut().rx_queue.extend(bytes.iter().copied());
        self.serial_feed();
    }

    /// Host side: `ArduinoMcu.drainSerialOutput()`.
    pub fn serial_drain(&mut self) -> Vec<u8> {
        match self.p.usart.as_mut() {
            Some(u) => std::mem::take(&mut u.tx),
            None => Vec::new(),
        }
    }

    /// The clock event `writeByte` schedules: the frame has arrived.
    pub(crate) fn usart_rx_event(&mut self, value: u8) {
        self.usart_mut().rx_busy = false;
        self.usart_write_byte(value, true);
    }

    /// The clock event a UDR write schedules: the frame has left.
    pub(crate) fn usart_tx_event(&mut self) {
        let (udre, txc) = (self.usart().udre, self.usart().txc);
        self.set_interrupt_flag(udre);
        self.set_interrupt_flag(txc);
    }

    /// `writeHooks[UCSRA]`. Reference quirk: the store keeps only MPCM and
    /// U2X, so writing UCSRA (HardwareSerial does, to clear TXC) also clears
    /// UDRE and RXC until the next event sets them.
    pub(crate) fn hook_ucsra(&mut self, value: u8, old: u8) -> bool {
        self.data[UCSRA] = value & (UCSRA_MPCM | UCSRA_U2X);
        let txc = self.usart().txc;
        self.clear_interrupt_by_flag(txc, value);
        if (value & UCSRA_CFG_MASK) != (old & UCSRA_CFG_MASK) {
            self.usart_mut().config_changes += 1;
        }
        true
    }

    /// `writeHooks[UCSRB]`. The UDRE flag raised on enabling TX is checked
    /// against the old UCSRB, which the hook stores only after.
    pub(crate) fn hook_ucsrb(&mut self, value: u8, old: u8) -> bool {
        let (rxc, udre, txc) = (self.usart().rxc, self.usart().udre, self.usart().txc);
        self.update_interrupt_enable(rxc, value);
        self.update_interrupt_enable(udre, value);
        self.update_interrupt_enable(txc, value);
        if value & UCSRB_RXEN != 0 && old & UCSRB_RXEN != 0 {
            self.clear_interrupt(rxc, true);
        }
        if value & UCSRB_TXEN != 0 && old & UCSRB_TXEN == 0 {
            self.set_interrupt_flag(udre);
        }
        self.data[UCSRB] = value;
        if (value & UCSRB_CFG_MASK) != (old & UCSRB_CFG_MASK) {
            self.usart_mut().config_changes += 1;
        }
        true
    }

    /// `writeHooks[UCSRC]`, `[UBRRH]` and `[UBRRL]`: store and report a
    /// configuration change.
    pub(crate) fn hook_usart_config(&mut self, addr: usize, value: u8) -> bool {
        self.data[addr] = value;
        self.usart_mut().config_changes += 1;
        true
    }

    /// `readHooks[UDR]`: the received byte, masked to the frame size, once.
    pub(crate) fn hook_udr_read(&mut self) -> u8 {
        let mask: u8 = match self.usart_bits_per_char() {
            5 => 0x1f,
            6 => 0x3f,
            7 => 0x7f,
            _ => 0xff,
        };
        let result = self.usart().rx_byte & mask;
        self.usart_mut().rx_byte = 0;
        let rxc = self.usart().rxc;
        self.clear_interrupt(rxc, true);
        result
    }

    /// `writeHooks[UDR]`: not consumed, so the value is also stored in UDR.
    pub(crate) fn hook_udr_write(&mut self, value: u8) -> bool {
        let u = self.usart_mut();
        u.tx.push(value);
        u.tx_count += 1;
        u.tx_hash = (u.tx_hash ^ value as u32).wrapping_mul(0x01000193);
        if let Some(buf) = u.line_buffer.as_mut() {
            if value == b'\n' {
                let line = std::mem::take(buf);
                u.lines.push(line);
            } else {
                // `String.fromCharCode`: a byte is its Latin-1 code point.
                buf.push(value as char);
            }
        }
        let cycles = self.usart_cycles_per_char();
        self.add_clock_event(EV_USART_TX, cycles);
        let (txc, udre) = (self.usart().txc, self.usart().udre);
        self.clear_interrupt(txc, true);
        self.clear_interrupt(udre, true);
        false
    }
}
