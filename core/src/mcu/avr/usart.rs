//! USART0, ported from avr8js 0.21.0 `peripherals/usart.js` (MIT, Copyright
//! (c) Uri Shaked and contributors; see NOTICE): `AVRUSART` with
//! `usart0Config`. The reference's callbacks become host-side records:
//! `onByteTransmit` appends to `tx` (what brownout's `ArduinoMcu` does with
//! it, a ring of the newest bytes), `onLineTransmit` to `lines` while
//! `line_buffer` is `Some`,
//! `onConfigurationChange` counts in `config_changes`. `onRxComplete` is
//! `ArduinoMcu`'s `_feedSerialInput`: the next byte of `rx_queue` goes to
//! `writeByte`, so typed bytes arrive one frame apart.

use std::collections::VecDeque;

use super::cpu::{Cpu, EV_USART_RX, EV_USART_RX_END, EV_USART_RX_SAMPLE, EV_USART_TX};
use super::bus::BusEvent;
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
const UCSRA_DOR: u8 = 0x8;
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

/// `ArduinoMcu.SERIAL_OUTPUT_CAPACITY`.
pub const SERIAL_OUTPUT_CAPACITY: usize = 16384;
const LINES_CAPACITY: usize = 1024;

pub struct Usart {
    pub freq_hz: u32,
    /// `rxBusyValue`: a received frame is on its way.
    pub rx_busy: bool,
    pub rx_byte: u8,
    /// The newest bytes the sketch has sent since the host last drained this,
    /// at most `tx_capacity` (`ArduinoMcu`'s ring): a board that prints
    /// forever with nothing draining must not grow without bound.
    pub tx: VecDeque<u8>,
    pub tx_capacity: usize,
    /// Bytes `tx` dropped, oldest first, since the board was built.
    pub tx_dropped: u64,
    /// Bytes sent since construction and a running FNV-1a over them (the
    /// lock-step harness compares both; `tx` is drained, these are not).
    pub tx_count: u64,
    pub tx_hash: u32,
    /// Bytes the host has typed that the receiver has not taken yet.
    pub rx_queue: VecDeque<u8>,
    /// `lineBuffer`, kept only while an `onLineTransmit` stands in (`Some`).
    pub line_buffer: Option<String>,
    /// The newest `lines_capacity` completed lines, and how many older ones
    /// were dropped.
    pub lines: VecDeque<String>,
    pub lines_capacity: usize,
    pub lines_dropped: u64,
    pub config_changes: u32,
    pub rx_completes: u32,
    pub rxc: Interrupt,
    pub udre: Interrupt,
    pub txc: Interrupt,
    /// `ArduinoMcu`'s `Usart328` (avr-bus-timing.ts), departures from avr8js
    /// toward the 328P's datasheet. Transmit: the first byte goes straight to
    /// the shift register with UDRE left set, a second waits in UDR and starts
    /// when the frame ends, a write while UDR is full is ignored, TXC is not
    /// cleared when a frame starts, and a byte is reported as its frame
    /// starts. Receive: a frame lands in a two-level FIFO at the first stop
    /// bit's middle sample, a third waits in the shift register, a fourth
    /// start bit overruns it (DOR on the next frame read), and disabling the
    /// receiver flushes it. Writing UCSRA keeps the read-only UDRE, RXC and
    /// DOR flags, and UCSRB writes leave RXC and UDRE as the buffers have
    /// them. Off, this is usart.js.
    pub datasheet: bool,
    /// A frame is in the transmit shift register (datasheet model only).
    pub tx_shifting: bool,
    /// The byte waiting in UDR, or -1 (datasheet model only).
    pub tx_pending: i16,
    /// A frame is arriving on RXD (datasheet model only).
    pub rx_receiving: bool,
    /// Received frames in UDR's FIFO with their DOR flags, oldest first, at
    /// most two (datasheet model only).
    pub rx_fifo: VecDeque<(u8, bool)>,
    /// A complete frame waiting in the receive shift register, or -1.
    pub rx_held: i16,
    /// A frame was lost to an overrun; the next one into the FIFO carries DOR.
    pub rx_overrun: bool,
    /// Bumped when the receiver is disabled, so a frame on its way in is
    /// dropped when it lands (24 bits of it ride in the clock event's arg).
    pub rx_generation: u32,
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
            tx: VecDeque::new(),
            tx_capacity: SERIAL_OUTPUT_CAPACITY,
            tx_dropped: 0,
            tx_count: 0,
            tx_hash: 0x811c9dc5,
            rx_queue: VecDeque::new(),
            line_buffer: None,
            lines: VecDeque::new(),
            lines_capacity: LINES_CAPACITY,
            lines_dropped: 0,
            config_changes: 0,
            rx_completes: 0,
            rxc: irq(0x24, UCSRA_RXC, UCSRB_RXCIE, true),
            udre: irq(0x26, UCSRA_UDRE, UCSRB_UDRIE, false),
            txc: irq(0x28, UCSRA_TXC, UCSRB_TXCIE, false),
            datasheet: false,
            tx_shifting: false,
            tx_pending: -1,
            rx_receiving: false,
            rx_fifo: VecDeque::new(),
            rx_held: -1,
            rx_overrun: false,
            rx_generation: 0,
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
        u.rx_receiving = false;
        u.rx_fifo.clear();
        u.rx_held = -1;
        u.rx_overrun = false;
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

    /// `ArduinoMcu`'s feed: offers the next queued byte to the receiver,
    /// which refuses while a frame is arriving or RXEN is clear.
    fn serial_feed(&mut self) {
        if self.usart().datasheet {
            self.usart_rx_feed();
            return;
        }
        if let Some(&b) = self.usart().rx_queue.front() {
            if self.usart_write_byte(b, false) {
                self.usart_mut().rx_queue.pop_front();
                let cycles = self.usart_cycles_per_char();
                self.bus_push(BusEvent::Uart { cycle: self.cycles, tx: false, value: b, cycles });
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
            Some(u) => std::mem::take(&mut u.tx).into(),
            None => Vec::new(),
        }
    }

    /// Host side: `ArduinoMcu.serialOutputDropped()`.
    pub fn serial_dropped(&self) -> u64 {
        self.p.usart.as_ref().map_or(0, |u| u.tx_dropped)
    }

    /// The clock event `writeByte` schedules: the frame has arrived.
    pub(crate) fn usart_rx_event(&mut self, value: u8) {
        self.usart_mut().rx_busy = false;
        self.usart_write_byte(value, true);
    }

    /// Cycles from a frame's start bit to the first stop bit's middle sample
    /// (the third of the three the receiver votes on: 8, 9 and 10 of 16, or
    /// 4, 5 and 6 of 8 with U2X).
    fn usart_rx_sample_cycles(&self) -> i64 {
        let stop_bit = 1 + self.usart_bits_per_char() + self.usart_parity_enabled() as u32;
        let m = self.usart_multiplier();
        ((stop_bit * m + if m == 16 { 9 } else { 5 }) * (self.usart_ubrr() + 1)) as i64
    }

    /// `Usart328.feed`: puts the next typed byte on the wire, if the line is
    /// free and the receiver is listening.
    fn usart_rx_feed(&mut self) {
        if self.usart().rx_receiving || self.usart().rx_queue.is_empty() || !self.usart_rx_enable() {
            return;
        }
        let value = self.usart_mut().rx_queue.pop_front().unwrap();
        let u = self.usart_mut();
        // A start bit with the FIFO full and a frame still waiting in the
        // shift register: that frame is lost.
        if u.rx_held >= 0 && u.rx_fifo.len() == 2 {
            u.rx_held = -1;
            u.rx_overrun = true;
        }
        u.rx_receiving = true;
        let generation = u.rx_generation & 0xff_ffff;
        let frame = self.usart_cycles_per_char();
        self.bus_push(BusEvent::Uart { cycle: self.cycles, tx: false, value, cycles: frame });
        let sample = self.usart_rx_sample_cycles();
        self.add_clock_event_arg(EV_USART_RX_SAMPLE, value as u32 | generation << 8, sample);
        self.add_clock_event(EV_USART_RX_END, frame);
    }

    /// The first stop bit's sample: the frame moves into the FIFO, or waits
    /// in the shift register while the FIFO is full.
    pub(crate) fn usart_rx_sample_event(&mut self, arg: u32) {
        let u = self.usart_mut();
        if arg >> 8 != u.rx_generation & 0xff_ffff {
            return;
        }
        let value = arg as u8;
        if u.rx_fifo.len() < 2 {
            let dor = std::mem::take(&mut u.rx_overrun);
            u.rx_fifo.push_back((value, dor));
        } else {
            u.rx_held = value as i16;
        }
        self.usart_update_rx();
    }

    /// The frame's end: the line is free for the next typed byte.
    pub(crate) fn usart_rx_end_event(&mut self) {
        self.usart_mut().rx_receiving = false;
        self.usart_rx_feed();
    }

    /// RXC and DOR as the FIFO's head has them.
    fn usart_update_rx(&mut self) {
        let head = self.usart().rx_fifo.front().copied();
        if matches!(head, Some((_, true))) {
            self.data[UCSRA] |= UCSRA_DOR;
        } else {
            self.data[UCSRA] &= !UCSRA_DOR;
        }
        let rxc = self.usart().rxc;
        if head.is_some() {
            self.set_interrupt_flag(rxc);
        } else {
            self.clear_interrupt(rxc, true);
        }
    }

    /// The clock event a UDR write schedules: the frame has left.
    pub(crate) fn usart_tx_event(&mut self) {
        let (udre, txc) = (self.usart().udre, self.usart().txc);
        if self.usart().datasheet {
            let pending = self.usart().tx_pending;
            if pending >= 0 {
                self.usart_mut().tx_pending = -1;
                self.usart_tx_begin(pending as u8);
                self.set_interrupt_flag(udre);
            } else {
                self.usart_mut().tx_shifting = false;
                self.set_interrupt_flag(udre);
                self.set_interrupt_flag(txc);
            }
            return;
        }
        self.set_interrupt_flag(udre);
        self.set_interrupt_flag(txc);
    }

    /// `writeHooks[UCSRA]`. Reference quirk: the store keeps only MPCM and
    /// U2X, so writing UCSRA (HardwareSerial does, to clear TXC) also clears
    /// UDRE and RXC until the next event sets them.
    /// The datasheet model keeps the flags instead, as the chip does.
    pub(crate) fn hook_ucsra(&mut self, value: u8, old: u8) -> bool {
        let keep = if self.usart().datasheet { old & !(UCSRA_MPCM | UCSRA_U2X) } else { 0 };
        self.data[UCSRA] = keep | (value & (UCSRA_MPCM | UCSRA_U2X));
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
        // usart.js 0.21.0 clears RXC only when RXEN stays set (in the old
        // value and the new), not when the receiver is switched on or off.
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
        if self.usart().datasheet {
            // RXC and UDRE are levels on the chip: put back from the buffers
            // what the reference's clear and raise above changed.
            if old & UCSRB_RXEN != 0 && value & UCSRB_RXEN == 0 {
                let u = self.usart_mut();
                u.rx_fifo.clear();
                u.rx_held = -1;
                u.rx_overrun = false;
                u.rx_generation = u.rx_generation.wrapping_add(1);
            }
            self.usart_update_rx();
            if self.usart().tx_pending < 0 {
                self.set_interrupt_flag(udre);
            } else {
                self.clear_interrupt(udre, true);
            }
            if value & UCSRB_RXEN != 0 && old & UCSRB_RXEN == 0 {
                self.usart_rx_feed();
            }
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
        if self.usart().datasheet {
            // The oldest frame; one waiting in the shift register moves up.
            let u = self.usart_mut();
            let head = u.rx_fifo.pop_front();
            if u.rx_held >= 0 {
                let dor = std::mem::take(&mut u.rx_overrun);
                u.rx_fifo.push_back((u.rx_held as u8, dor));
                u.rx_held = -1;
            }
            self.usart_update_rx();
            return head.map_or(0, |(v, _)| v & mask);
        }
        let result = self.usart().rx_byte & mask;
        self.usart_mut().rx_byte = 0;
        let rxc = self.usart().rxc;
        self.clear_interrupt(rxc, true);
        result
    }

    /// `writeHooks[UDR]`: not consumed, so the value is also stored in UDR.
    pub(crate) fn hook_udr_write(&mut self, value: u8) -> bool {
        if self.usart().datasheet {
            if self.usart().tx_pending >= 0 {
                // UDRE clear: the datasheet has the transmitter ignore it.
                return false;
            }
            if !self.usart().tx_shifting {
                self.usart_tx_begin(value);
                // UDRE is a level on the chip; the reference clears it when
                // it takes the interrupt, so it is raised again whenever UDR
                // is left empty (avr-bus-timing.ts says the same).
                let udre = self.usart().udre;
                self.set_interrupt_flag(udre);
            } else {
                self.usart_mut().tx_pending = value as i16;
                let udre = self.usart().udre;
                self.clear_interrupt(udre, true);
            }
            return false;
        }
        self.usart_tx_send(value);
        let cycles = self.usart_cycles_per_char();
        self.add_clock_event(EV_USART_TX, cycles);
        let (txc, udre) = (self.usart().txc, self.usart().udre);
        self.clear_interrupt(txc, true);
        self.clear_interrupt(udre, true);
        false
    }

    /// The datasheet transmitter moves a byte into the shift register: it
    /// goes out now, and the frame ends `cyclesPerChar` later. TXC stays as
    /// it is; only a one written to it or its interrupt clears it.
    fn usart_tx_begin(&mut self, value: u8) {
        self.usart_mut().tx_shifting = true;
        self.usart_tx_send(value);
        let cycles = self.usart_cycles_per_char();
        self.add_clock_event(EV_USART_TX, cycles);
    }

    /// `onByteTransmit` and `onLineTransmit` for a byte on the wire.
    fn usart_tx_send(&mut self, value: u8) {
        let cycles = self.usart_cycles_per_char();
        self.bus_push(BusEvent::Uart { cycle: self.cycles, tx: true, value, cycles });
        let u = self.usart_mut();
        if u.tx_capacity == 0 {
            u.tx_dropped += 1;
        } else {
            if u.tx.len() == u.tx_capacity {
                u.tx.pop_front();
                u.tx_dropped += 1;
            }
            u.tx.push_back(value);
        }
        u.tx_count += 1;
        u.tx_hash = (u.tx_hash ^ value as u32).wrapping_mul(0x01000193);
        if let Some(buf) = u.line_buffer.as_mut() {
            if value == b'\n' {
                let line = std::mem::take(buf);
                u.lines.push_back(line);
                while u.lines.len() > u.lines_capacity {
                    u.lines.pop_front();
                    u.lines_dropped += 1;
                }
            } else {
                // `String.fromCharCode`: a byte is its Latin-1 code point.
                buf.push(value as char);
            }
        }
    }
}
