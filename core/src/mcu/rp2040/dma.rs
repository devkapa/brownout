//! The DMA controller, ported from rp2040js 1.3.3 (MIT):
//! `src/peripherals/dma.ts`. Twelve channels, each moving one unit per
//! `transfer` alarm (a zero-delay alarm when its request line is up or the
//! channel is permanently paced, otherwise the delay of one of the four
//! pacing timers), with chaining, the ring, byte swap, the null trigger and
//! the abort and multi-channel trigger registers.
//!
//! The request lines (`DREQChannel`) are shared with the peripherals that pace
//! a channel. The PIO state machines raise and lower theirs (`pio.rs`); the
//! UARTs, the SPIs, the I2Cs, the PWM slices and the ADC do it in package R6
//! through [`Rp2040::dma_set_dreq`] and [`Rp2040::dma_clear_dreq`], the named
//! hooks this file leaves for them.
//!
//! Numerics follow JavaScript where it shows: a channel's addresses are
//! doubles that only ever meet 32-bit bitwise operators or the bus (which
//! wraps them with `>>> 0`), so they are kept as wrapping 32-bit words; the
//! transfer count is a signed double that stops at `> 0`, so it is kept as an
//! i64, and a sign carried by an atomic-alias or sub-word write
//! (`Periph::write_signed`) is kept, as a plain word store's is not.

use super::clock::AlarmId;
use super::Rp2040;

/// `DREQChannel`.
pub mod dreq {
    pub const PIO0_TX0: u32 = 0;
    pub const PIO0_RX0: u32 = 4;
    pub const PIO1_TX0: u32 = 8;
    pub const PIO1_RX0: u32 = 12;
    pub const SPI0_TX: u32 = 16;
    pub const SPI0_RX: u32 = 17;
    pub const SPI1_TX: u32 = 18;
    pub const SPI1_RX: u32 = 19;
    pub const UART0_TX: u32 = 20;
    pub const UART0_RX: u32 = 21;
    pub const UART1_TX: u32 = 22;
    pub const UART1_RX: u32 = 23;
    pub const PWM_WRAP0: u32 = 24;
    pub const I2C0_TX: u32 = 32;
    pub const I2C0_RX: u32 = 33;
    pub const I2C1_TX: u32 = 34;
    pub const I2C1_RX: u32 = 35;
    pub const ADC: u32 = 36;
    pub const XIP_STREAM: u32 = 37;
    pub const XIP_SSITX: u32 = 38;
    pub const XIP_SSIRX: u32 = 39;
    /// `DREQ_MAX`: the reference's array has this many slots; a lookup past it
    /// reads undefined, which is false.
    pub const MAX: u32 = 40;
}

const TREQ_TIMER0: u32 = 0x3b;
const TREQ_PERMANENT: u32 = 0x3f;

// Per-channel registers.
const CH_READ_ADDR: u32 = 0x000;
const CH_WRITE_ADDR: u32 = 0x004;
const CH_TRANS_COUNT: u32 = 0x008;
const CH_CTRL_TRIG: u32 = 0x00c;
const CH_AL1_CTRL: u32 = 0x010;
const CH_AL1_READ_ADDR: u32 = 0x014;
const CH_AL1_WRITE_ADDR: u32 = 0x018;
const CH_AL1_TRANS_COUNT_TRIG: u32 = 0x01c;
const CH_AL2_CTRL: u32 = 0x020;
const CH_AL2_TRANS_COUNT: u32 = 0x024;
const CH_AL2_READ_ADDR: u32 = 0x028;
const CH_AL2_WRITE_ADDR_TRIG: u32 = 0x02c;
const CH_AL3_CTRL: u32 = 0x030;
const CH_AL3_WRITE_ADDR: u32 = 0x034;
const CH_AL3_TRANS_COUNT: u32 = 0x038;
const CH_AL3_READ_ADDR_TRIG: u32 = 0x03c;
const CH_DBG_CTDREQ: u32 = 0x800;
const CH_DBG_TCR: u32 = 0x804;
const CHANNEL_REGISTERS_SIZE: u32 = 12 * 0x40;
const CHANNEL_REGISTERS_MASK: u32 = 0x83f;

// General registers.
const INTR: u32 = 0x400;
const INTE0: u32 = 0x404;
const INTF0: u32 = 0x408;
const INTS0: u32 = 0x40c;
const INTE1: u32 = 0x414;
const INTF1: u32 = 0x418;
const INTS1: u32 = 0x41c;
const TIMER0: u32 = 0x420;
const TIMER1: u32 = 0x424;
const TIMER2: u32 = 0x428;
const TIMER3: u32 = 0x42c;
const MULTI_CHAN_TRIGGER: u32 = 0x430;
const CHAN_ABORT: u32 = 0x444;
const N_CHANNELS: u32 = 0x448;

// CHn_CTRL_TRIG bits.
const BUSY: u32 = 1 << 24;
const BSWAP: u32 = 1 << 22;
const IRQ_QUIET: u32 = 1 << 21;
const TREQ_SEL_MASK: u32 = 0x3f;
const TREQ_SEL_SHIFT: u32 = 15;
const CHAIN_TO_MASK: u32 = 0xf;
const CHAIN_TO_SHIFT: u32 = 11;
const RING_SEL: u32 = 1 << 10;
const RING_SIZE_MASK: u32 = 0xf;
const RING_SIZE_SHIFT: u32 = 6;
const INCR_WRITE: u32 = 1 << 5;
const INCR_READ: u32 = 1 << 4;
const DATA_SIZE_MASK: u32 = 0x3;
const DATA_SIZE_SHIFT: u32 = 2;
const EN: u32 = 1;
const CTRL_TRIG_WRITE_MASK: u32 = 0xff_ffff;
const CTRL_TRIG_WC_MASK: u32 = (1 << 30) | (1 << 29);

/// `IRQ.DMA_IRQ0` and `DMA_IRQ1`.
const IRQ_DMA_IRQ0: u32 = 11;
const IRQ_DMA_IRQ1: u32 = 12;

/// The five `transferFn`s.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferFn {
    Byte,
    Half,
    SwapHalf,
    Word,
    SwapWord,
}

#[derive(Clone)]
pub struct DmaChannel {
    pub ctrl: u32,
    pub read_addr: u32,
    pub write_addr: u32,
    pub trans_count: i64,
    pub dreq_counter: u32,
    pub trans_count_reload: i64,
    pub treq: u32,
    pub data_size: u32,
    pub chain_to: u32,
    pub ring_mask: u32,
    pub transfer_fn: TransferFn,
}

impl DmaChannel {
    fn new() -> Self {
        DmaChannel {
            ctrl: 0,
            read_addr: 0,
            write_addr: 0,
            trans_count: 0,
            dreq_counter: 0,
            trans_count_reload: 0,
            treq: 0,
            data_size: 1,
            chain_to: 0,
            ring_mask: 0,
            transfer_fn: TransferFn::Byte,
        }
    }

    /// `active`: enabled and busy.
    pub fn active(&self) -> bool {
        self.ctrl & EN != 0 && self.ctrl & BUSY != 0
    }
}

pub struct Dma {
    pub channels: [DmaChannel; 12],
    pub int_raw: u32,
    pub int_enable0: u32,
    pub int_force0: u32,
    pub int_enable1: u32,
    pub int_force1: u32,
    pub timer: [u32; 4],
    /// `dreq[]`: 64 slots cover every TREQ_SEL value; the reference's array
    /// is shorter and reads undefined (false) past it.
    pub dreq: [bool; 64],
}

impl Default for Dma {
    fn default() -> Self {
        Self::new()
    }
}

impl Dma {
    /// Construction of the block (the channels' `reset()` write is applied by
    /// `Rp2040::dma_reset_channels`, which needs the machine).
    pub fn new() -> Self {
        Dma {
            channels: std::array::from_fn(|_| DmaChannel::new()),
            int_raw: 0,
            int_enable0: 0,
            int_force0: 0,
            int_enable1: 0,
            int_force1: 0,
            timer: [0; 4],
            dreq: [false; 64],
        }
    }

    pub fn int_status0(&self) -> u32 {
        (self.int_raw & self.int_enable0) | self.int_force0
    }

    pub fn int_status1(&self) -> u32 {
        (self.int_raw & self.int_enable1) | self.int_force1
    }

    /// FNV-1a over the channels' registers, for the lock-step record.
    pub fn channel_digest(&self) -> u32 {
        let mut h = 0x811c_9dc5u32;
        let mut mix = |v: u32| h = (h ^ v).wrapping_mul(0x0100_0193);
        for c in &self.channels {
            mix(c.ctrl);
            mix(c.read_addr);
            mix(c.write_addr);
            mix(c.trans_count as u32);
            mix(c.trans_count_reload as u32);
            mix(c.dreq_counter);
            mix(c.treq);
            mix(c.data_size);
            mix(c.chain_to);
            mix(c.ring_mask);
            mix(c.transfer_fn as u32);
        }
        h
    }

    /// FNV-1a over the interrupt registers and the pacing timers.
    pub fn irq_digest(&self) -> u32 {
        let mut h = 0x811c_9dc5u32;
        for v in [self.int_raw, self.int_enable0, self.int_force0, self.int_enable1, self.int_force1]
            .into_iter()
            .chain(self.timer)
        {
            h = (h ^ v).wrapping_mul(0x0100_0193);
        }
        h
    }

    /// The request lines as a bitmask pair folded to one word.
    pub fn dreq_digest(&self) -> u32 {
        let mut h = 0x811c_9dc5u32;
        for (i, &b) in self.dreq.iter().enumerate() {
            if b {
                h = (h ^ i as u32).wrapping_mul(0x0100_0193);
            }
        }
        h
    }
}

impl Rp2040 {
    /// The channels' constructor-time `reset()`: a write of `index << 11` to
    /// CTRL_TRIG, which chains each channel to itself.
    pub(crate) fn dma_reset_channels(&mut self) {
        for i in 0..12 {
            self.dma_channel_write(i, CH_CTRL_TRIG, (i as u32) << CHAIN_TO_SHIFT, false);
        }
    }

    /// `RPDMAChannel.start`.
    fn dma_start(&mut self, i: usize) {
        let c = &mut self.periph.dma.channels[i];
        if c.ctrl & EN == 0 || c.ctrl & BUSY != 0 {
            return;
        }
        c.ctrl |= BUSY;
        c.trans_count = c.trans_count_reload;
        if c.trans_count != 0 {
            self.dma_schedule_transfer(i);
        }
    }

    /// `scheduleTransfer`: now when the request line is up (or the channel is
    /// permanently paced), otherwise after one period of its pacing timer.
    pub(crate) fn dma_schedule_transfer(&mut self, i: usize) {
        let treq = self.periph.dma.channels[i].treq;
        if self.periph.dma.dreq[treq as usize] || treq == TREQ_PERMANENT {
            self.clock.schedule(AlarmId::Dma(i as u8), 0.0);
        } else {
            let delay = self.dma_get_timer(treq);
            if delay != 0.0 && !delay.is_nan() {
                self.clock.schedule(AlarmId::Dma(i as u8), delay * 1000.0);
            }
        }
    }

    /// `getTimer`: microseconds per cycle of a pacing timer (0: disabled).
    /// Timer 3 shifts by 36, which JavaScript takes modulo 32, so it divides
    /// by the 4-bit shifted word, not the top half (reference quirk).
    fn dma_get_timer(&self, treq: u32) -> f64 {
        let t = &self.periph.dma.timer;
        let (dividend, divisor) = match treq {
            TREQ_PERMANENT => (1u32, 1u32),
            x if x == TREQ_TIMER0 => (t[0] >> 16, t[0] & 0xffff),
            x if x == TREQ_TIMER0 + 1 => (t[1] >> 16, t[1] & 0xffff),
            x if x == TREQ_TIMER0 + 2 => (t[2] >> 16, t[2] & 0xffff),
            x if x == TREQ_TIMER0 + 3 => (t[3] >> (36 & 31), t[3] & 0xffff),
            _ => (0, 1),
        };
        if divisor == 0 {
            return 0.0;
        }
        ((dividend as f64 / divisor as f64) * 1e6) / 125_000_000.0
    }

    /// The transfer alarm's callback, `RPDMAChannel.transfer`.
    pub(crate) fn dma_transfer_alarm_fired(&mut self, i: usize) {
        let (ctrl, data_size, ring_mask, func, read_addr, write_addr) = {
            let c = &self.periph.dma.channels[i];
            (c.ctrl, c.data_size, c.ring_mask, c.transfer_fn, c.read_addr, c.write_addr)
        };
        match func {
            TransferFn::Byte => {
                let v = self.read_u8(read_addr);
                self.write_u8(write_addr, v);
            }
            TransferFn::Half => {
                let v = self.read_u16(read_addr);
                self.write_u16(write_addr, v);
            }
            TransferFn::SwapHalf => {
                let input = self.read_u16(read_addr);
                self.write_u16(write_addr, ((input & 0xff) << 8) | (input >> 8));
            }
            TransferFn::Word => {
                let v = self.read_u32(read_addr);
                self.write_u32(write_addr, v);
            }
            TransferFn::SwapWord => {
                let input = self.read_u32(read_addr);
                self.write_u32(
                    write_addr,
                    ((input & 0x0000_00ff) << 24)
                        | ((input & 0x0000_ff00) << 8)
                        | ((input & 0x00ff_0000) >> 8)
                        | ((input >> 24) & 0xff),
                );
            }
        }
        let c = &mut self.periph.dma.channels[i];
        if ctrl & INCR_READ != 0 {
            if ring_mask != 0 && ctrl & RING_SEL == 0 {
                c.read_addr = (c.read_addr & !ring_mask) | (c.read_addr.wrapping_add(data_size) & ring_mask);
            } else {
                c.read_addr = c.read_addr.wrapping_add(data_size);
            }
        }
        if ctrl & INCR_WRITE != 0 {
            if ring_mask != 0 && ctrl & RING_SEL != 0 {
                c.write_addr = (c.write_addr & !ring_mask) | (c.write_addr.wrapping_add(data_size) & ring_mask);
            } else {
                c.write_addr = c.write_addr.wrapping_add(data_size);
            }
        }
        c.trans_count -= 1;
        if c.trans_count > 0 {
            self.dma_schedule_transfer(i);
        } else {
            c.ctrl &= !BUSY;
            let quiet = c.ctrl & IRQ_QUIET != 0;
            let chain_to = c.chain_to as usize;
            if !quiet {
                self.periph.dma.int_raw |= 1 << i;
                self.dma_check_interrupts();
            }
            if chain_to != i && chain_to < 12 {
                self.dma_start(chain_to);
            }
        }
    }

    fn dma_abort(&mut self, i: usize) {
        self.periph.dma.channels[i].ctrl &= !BUSY;
        self.clock.cancel(AlarmId::Dma(i as u8));
    }

    fn dma_channel_read(&self, i: usize, offset: u32) -> u32 {
        let c = &self.periph.dma.channels[i];
        match offset {
            CH_READ_ADDR | CH_AL1_READ_ADDR | CH_AL2_READ_ADDR | CH_AL3_READ_ADDR_TRIG => c.read_addr,
            CH_WRITE_ADDR | CH_AL1_WRITE_ADDR | CH_AL2_WRITE_ADDR_TRIG | CH_AL3_WRITE_ADDR => c.write_addr,
            CH_TRANS_COUNT | CH_AL1_TRANS_COUNT_TRIG | CH_AL2_TRANS_COUNT | CH_AL3_TRANS_COUNT => c.trans_count as u32,
            CH_CTRL_TRIG | CH_AL1_CTRL | CH_AL2_CTRL | CH_AL3_CTRL => c.ctrl,
            CH_DBG_CTDREQ => c.dreq_counter,
            CH_DBG_TCR => c.trans_count_reload as u32,
            _ => 0,
        }
    }

    /// `RPDMAChannel.writeUint32`; `signed` is whether the word is an int32
    /// in the reference (an atomic alias or a sub-word store).
    fn dma_channel_write(&mut self, i: usize, offset: u32, value: u32, signed: bool) {
        match offset {
            CH_READ_ADDR | CH_AL1_READ_ADDR | CH_AL2_READ_ADDR | CH_AL3_READ_ADDR_TRIG => {
                self.periph.dma.channels[i].read_addr = value;
            }
            CH_WRITE_ADDR | CH_AL1_WRITE_ADDR | CH_AL2_WRITE_ADDR_TRIG | CH_AL3_WRITE_ADDR => {
                self.periph.dma.channels[i].write_addr = value;
            }
            CH_TRANS_COUNT | CH_AL1_TRANS_COUNT_TRIG | CH_AL2_TRANS_COUNT | CH_AL3_TRANS_COUNT => {
                self.periph.dma.channels[i].trans_count_reload =
                    if signed { value as i32 as i64 } else { value as i64 };
            }
            CH_CTRL_TRIG | CH_AL1_CTRL | CH_AL2_CTRL | CH_AL3_CTRL => {
                let c = &mut self.periph.dma.channels[i];
                c.ctrl = (c.ctrl & !CTRL_TRIG_WRITE_MASK) | (value & CTRL_TRIG_WRITE_MASK);
                c.ctrl &= !(value & CTRL_TRIG_WC_MASK);
                c.treq = (c.ctrl >> TREQ_SEL_SHIFT) & TREQ_SEL_MASK;
                c.chain_to = (c.ctrl >> CHAIN_TO_SHIFT) & CHAIN_TO_MASK;
                let ring_size = (c.ctrl >> RING_SIZE_SHIFT) & RING_SIZE_MASK;
                c.ring_mask = if ring_size != 0 { (1u32 << ring_size) - 1 } else { 0 };
                match (c.ctrl >> DATA_SIZE_SHIFT) & DATA_SIZE_MASK {
                    1 => {
                        c.data_size = 2;
                        c.transfer_fn = if c.ctrl & BSWAP != 0 { TransferFn::SwapHalf } else { TransferFn::Half };
                    }
                    2 => {
                        c.data_size = 4;
                        c.transfer_fn = if c.ctrl & BSWAP != 0 { TransferFn::SwapWord } else { TransferFn::Word };
                    }
                    _ => {
                        c.transfer_fn = TransferFn::Byte;
                        c.data_size = 1;
                    }
                }
                let (en, busy) = (c.ctrl & EN != 0, c.ctrl & BUSY != 0);
                if en && busy {
                    self.dma_schedule_transfer(i);
                }
                if !en {
                    self.clock.cancel(AlarmId::Dma(i as u8));
                }
            }
            CH_DBG_CTDREQ => self.periph.dma.channels[i].dreq_counter = 0,
            _ => {}
        }
        if offset == CH_AL3_READ_ADDR_TRIG
            || offset == CH_AL2_WRITE_ADDR_TRIG
            || offset == CH_AL1_TRANS_COUNT_TRIG
            || offset == CH_CTRL_TRIG
        {
            if value != 0 {
                self.dma_start(i);
            } else if self.periph.dma.channels[i].ctrl & IRQ_QUIET != 0 {
                // Null trigger interrupts
                self.periph.dma.int_raw |= 1 << i;
                self.dma_check_interrupts();
            }
        }
    }

    pub(crate) fn dma_read(&mut self, offset: u32) -> u32 {
        if (offset & 0x7ff) < CHANNEL_REGISTERS_SIZE {
            let channel = ((offset & 0x7ff) >> 6) as usize;
            return self.dma_channel_read(channel, offset & CHANNEL_REGISTERS_MASK);
        }
        let d = &self.periph.dma;
        match offset {
            TIMER0 => d.timer[0],
            TIMER1 => d.timer[1],
            TIMER2 => d.timer[2],
            TIMER3 => d.timer[3],
            INTR => d.int_raw,
            INTE0 => d.int_enable0,
            INTF0 => d.int_force0,
            INTS0 => d.int_status0(),
            INTE1 => d.int_enable1,
            INTF1 => d.int_force1,
            INTS1 => d.int_status1(),
            N_CHANNELS => 12,
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
                0xffff_ffff
            }
        }
    }

    pub(crate) fn dma_write(&mut self, offset: u32, value: u32, raw: u32) {
        if (offset & 0x7ff) < CHANNEL_REGISTERS_SIZE {
            let channel = ((offset & 0x7ff) >> 6) as usize;
            let signed = self.periph.write_signed;
            self.dma_channel_write(channel, offset & CHANNEL_REGISTERS_MASK, value, signed);
            return;
        }
        match offset {
            TIMER0 => self.periph.dma.timer[0] = value,
            TIMER1 => self.periph.dma.timer[1] = value,
            TIMER2 => self.periph.dma.timer[2] = value,
            TIMER3 => self.periph.dma.timer[3] = value,
            INTR | INTS0 | INTS1 => {
                self.periph.dma.int_raw &= !raw;
                self.dma_check_interrupts();
            }
            INTE0 => {
                self.periph.dma.int_enable0 = value & 0xffff;
                self.dma_check_interrupts();
            }
            INTF0 => {
                self.periph.dma.int_force0 = value & 0xffff;
                self.dma_check_interrupts();
            }
            INTE1 => {
                self.periph.dma.int_enable1 = value & 0xffff;
                self.dma_check_interrupts();
            }
            INTF1 => {
                self.periph.dma.int_force1 = value & 0xffff;
                self.dma_check_interrupts();
            }
            MULTI_CHAN_TRIGGER => {
                for i in 0..12 {
                    if value & (1 << i) != 0 {
                        self.dma_start(i);
                    }
                }
            }
            CHAN_ABORT => {
                for i in 0..12 {
                    if value & (1 << i) != 0 {
                        self.dma_abort(i);
                    }
                }
            }
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
            }
        }
    }

    /// `setDREQ`: a peripheral raises its request line; every active channel
    /// paced by it tries to go.
    pub fn dma_set_dreq(&mut self, channel: u32) {
        if !self.periph.dma.dreq[channel as usize] {
            self.periph.dma.dreq[channel as usize] = true;
            for i in 0..12 {
                let c = &self.periph.dma.channels[i];
                if c.treq == channel && c.active() {
                    self.dma_schedule_transfer(i);
                }
            }
        }
    }

    /// `clearDREQ`.
    pub fn dma_clear_dreq(&mut self, channel: u32) {
        self.periph.dma.dreq[channel as usize] = false;
    }

    /// `checkInterrupts`.
    pub(crate) fn dma_check_interrupts(&mut self) {
        let s0 = self.periph.dma.int_status0() != 0;
        let s1 = self.periph.dma.int_status1() != 0;
        self.set_interrupt(IRQ_DMA_IRQ0, s0);
        self.set_interrupt(IRQ_DMA_IRQ1, s1);
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for TransferFn {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            TransferFn::Byte => 0,
            TransferFn::Half => 0,
            TransferFn::SwapHalf => 0,
            TransferFn::Word => 0,
            TransferFn::SwapWord => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for DmaChannel {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ctrl, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.read_addr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.write_addr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trans_count, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dreq_counter, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.trans_count_reload, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.treq, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.data_size, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.chain_to, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ring_mask, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.transfer_fn, w); }
        n
    }
}
