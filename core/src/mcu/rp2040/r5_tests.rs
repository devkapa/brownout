//! Unit tests for R5: the DMA controller and the two PIO blocks. Expected
//! values come from the RP2040 datasheet (section 2.5 DMA, section 3 PIO: the
//! register descriptions and the instruction set), derived in each test from
//! field positions and the documented behaviour, not read back from the
//! port. Where rp2040js 1.3.3 departs from the datasheet the port follows it
//! and a `reference_quirk_*` test (or, for behaviour the datasheet leaves
//! open, a test named for what the reference does) pins both.
//!
//! Upstream suites ported by hand, tag v1.3.3 (MIT; see NOTICE):
//! `src/peripherals/dma.spec.ts` (two tests), `src/peripherals/pio.spec.ts`
//! (all of its tests) and `src/utils/pio-assembler.spec.ts` (nine tests). The
//! assembler (`src/utils/pio-assembler.ts`) is ported as a test helper only,
//! in `mod asm` below. rp2040js has no other DMA or PIO tests.

use super::dma::dreq;
use super::*;

const DMA: u32 = 0x5000_0000;
const PIO0: u32 = 0x5020_0000;
const PIO1: u32 = 0x5030_0000;
const NVIC_ISPR: u32 = 0xe000_e200;
const NVIC_ICPR: u32 = 0xe000_e280;
const PADS_BANK0: u32 = 0x4001_c000;
const IO_BANK0: u32 = 0x4001_4000;
const SET: u32 = 0x2000;

fn rd(m: &mut Rp2040, a: u32) -> u32 {
    m.read_u32(a)
}
fn wr(m: &mut Rp2040, a: u32, v: u32) {
    m.write_u32(a, v)
}
/// `clock.tick` by whole nanoseconds.
fn ns(m: &mut Rp2040, nanos: f64) {
    m.clock_tick(nanos);
}
const fn bit(n: u32) -> u32 {
    1 << n
}

/// `src/utils/pio-assembler.ts`.
mod asm {
    pub const SRC_PINS: u32 = 0;
    pub const SRC_X: u32 = 1;
    pub const SRC_Y: u32 = 2;
    pub const SRC_NULL: u32 = 3;
    pub const SRC_STATUS: u32 = 5;
    pub const DEST_PINS: u32 = 0;
    pub const DEST_X: u32 = 1;
    pub const DEST_Y: u32 = 2;
    pub const DEST_NULL: u32 = 3;
    pub const DEST_PINDIRS: u32 = 4;
    pub const DEST_PC: u32 = 5;
    pub const DEST_EXEC: u32 = 7;
    pub const MOV_DEST_PINS: u32 = 0;
    pub const MOV_DEST_X: u32 = 1;
    pub const MOV_DEST_Y: u32 = 2;
    pub const MOV_DEST_ISR: u32 = 6;
    pub const MOV_DEST_OSR: u32 = 7;
    pub const OP_NONE: u32 = 0;
    pub const OP_INVERT: u32 = 1;
    pub const OP_BITREV: u32 = 2;
    pub const WAIT_SRC_GPIO: u32 = 0;
    pub const WAIT_SRC_IRQ: u32 = 2;
    pub const COND_ALWAYS: u32 = 0;
    pub const COND_NOTX: u32 = 1;
    pub const COND_XDEC: u32 = 2;
    pub const COND_NOTY: u32 = 3;
    pub const COND_YDEC: u32 = 4;
    pub const COND_XNEY: u32 = 5;
    pub const COND_PIN: u32 = 6;
    pub const COND_NOTEMPTYOSR: u32 = 7;

    pub fn jmp(cond: u32, address: u32, delay: u32) -> u32 {
        ((delay & 0x1f) << 8) | ((cond & 0x7) << 5) | (address & 0x1f)
    }
    pub fn wait(polarity: bool, src: u32, index: u32, delay: u32) -> u32 {
        (1 << 13) | ((delay & 0x1f) << 8) | ((polarity as u32) << 7) | ((src & 0x3) << 5) | (index & 0x1f)
    }
    pub fn input(src: u32, bit_count: u32, delay: u32) -> u32 {
        (2 << 13) | ((delay & 0x1f) << 8) | ((src & 0x7) << 5) | (bit_count & 0x1f)
    }
    pub fn out(dest: u32, bit_count: u32, delay: u32) -> u32 {
        (3 << 13) | ((delay & 0x1f) << 8) | ((dest & 0x7) << 5) | (bit_count & 0x1f)
    }
    pub fn push(if_full: bool, no_block: bool, delay: u32) -> u32 {
        (4 << 13) | ((delay & 0x1f) << 8) | ((if_full as u32) << 6) | ((no_block as u32) << 5)
    }
    pub fn pull(if_empty: bool, no_block: bool, delay: u32) -> u32 {
        (4 << 13) | ((delay & 0x1f) << 8) | (1 << 7) | ((if_empty as u32) << 6) | ((no_block as u32) << 5)
    }
    pub fn mov(dest: u32, op: u32, src: u32, delay: u32) -> u32 {
        (5 << 13) | ((delay & 0x1f) << 8) | ((dest & 0x7) << 5) | ((op & 0x3) << 3) | (src & 0x7)
    }
    pub fn irq(clear: bool, wait: bool, index: u32, delay: u32) -> u32 {
        (6 << 13) | ((delay & 0x1f) << 8) | ((clear as u32) << 6) | ((wait as u32) << 5) | (index & 0x1f)
    }
    pub fn set(dest: u32, data: u32, delay: u32) -> u32 {
        (7 << 13) | ((delay & 0x1f) << 8) | ((dest & 0x7) << 5) | (data & 0x1f)
    }
}
use asm::*;

// ======================================================================================
// DMA
// ======================================================================================

const CH2_WRITE_ADDR: u32 = 0x5000_0084;
const CH2_TRANS_COUNT: u32 = 0x5000_0088;
const CH2_AL1_CTRL: u32 = 0x5000_0090;
const CH2_AL3_READ_ADDR_TRIG: u32 = 0x5000_00bc;
const CH6_READ_ADDR: u32 = 0x5000_0180;
const CH6_WRITE_ADDR: u32 = 0x5000_0184;
const CH6_TRANS_COUNT: u32 = 0x5000_0188;
const CH6_CTRL_TRIG: u32 = 0x5000_018c;
const DMA_INTR: u32 = 0x5000_0400;
const DMA_INTE0: u32 = 0x5000_0404;
const DMA_INTF0: u32 = 0x5000_0408;
const DMA_INTS0: u32 = 0x5000_040c;
const DMA_INTE1: u32 = 0x5000_0414;
const DMA_INTS1: u32 = 0x5000_041c;
const DMA_TIMER0: u32 = 0x5000_0420;
const DMA_TIMER3: u32 = 0x5000_042c;
const DMA_MULTI_CHAN_TRIGGER: u32 = 0x5000_0430;
const DMA_CHAN_ABORT: u32 = 0x5000_0444;

const EN: u32 = bit(0);
const DATA_SIZE_SHIFT: u32 = 2;
const INCR_WRITE: u32 = bit(5);
const INCR_READ: u32 = bit(4);
const RING_SIZE_SHIFT: u32 = 6;
const RING_SEL: u32 = bit(10);
const CHAIN_TO_SHIFT: u32 = 11;
const TREQ_SEL_SHIFT: u32 = 15;
const IRQ_QUIET: u32 = bit(21);
const BSWAP: u32 = bit(22);
const BUSY: u32 = bit(24);
const TREQ_PERMANENT: u32 = 0x3f;

/// Channel `n`'s register block.
fn ch(n: u32, off: u32) -> u32 {
    DMA + 0x40 * n + off
}
const CTRL_TRIG: u32 = 0x0c;
const AL1_CTRL: u32 = 0x10;
const READ_ADDR: u32 = 0x00;
const WRITE_ADDR: u32 = 0x04;
const TRANS_COUNT: u32 = 0x08;

const RAM: u32 = 0x2000_0000;

/// Starts a permanently paced copy on channel `n`.
fn dma_copy(m: &mut Rp2040, n: u32, src: u32, dst: u32, count: u32, size_field: u32, extra: u32) {
    wr(m, ch(n, READ_ADDR), src);
    wr(m, ch(n, WRITE_ADDR), dst);
    wr(m, ch(n, TRANS_COUNT), count);
    wr(
        m,
        ch(n, CTRL_TRIG),
        EN | extra | (TREQ_PERMANENT << TREQ_SEL_SHIFT) | (size_field << DATA_SIZE_SHIFT) | (n << CHAIN_TO_SHIFT),
    );
}

// ---- src/peripherals/dma.spec.ts ---------------------------------------------------

#[test]
fn upstream_dma_should_support_dma_channel_chaining() {
    let mut m = Rp2040::new();
    // Four 8-byte chunks in different RAM areas copied into one area by
    // channel 2, which channel 6 re-triggers once per chunk.
    let chunks = [0x2000_1000u32, 0x2000_1100, 0x2000_2200, 0x2000_2300];
    let dest = 0x2000_8000u32;
    let control_block = 0x2000_a000u32;
    for (i, &c) in chunks.iter().enumerate() {
        wr(&mut m, c, 0x10 * (2 * i as u32 + 1));
        wr(&mut m, c + 4, 0x10 * (2 * i as u32 + 2));
    }
    for (i, &c) in chunks.iter().enumerate() {
        wr(&mut m, control_block + 4 * i as u32, c);
    }
    wr(&mut m, control_block + 16, 0); // end of chain
    wr(&mut m, CH2_WRITE_ADDR, dest);
    wr(&mut m, CH2_TRANS_COUNT, 2);
    wr(
        &mut m,
        CH2_AL1_CTRL,
        EN | (6 << CHAIN_TO_SHIFT) | INCR_WRITE | INCR_READ | (TREQ_PERMANENT << TREQ_SEL_SHIFT) | (2 << DATA_SIZE_SHIFT),
    );
    wr(&mut m, CH6_WRITE_ADDR, CH2_AL3_READ_ADDR_TRIG);
    wr(&mut m, CH6_READ_ADDR, control_block);
    wr(&mut m, CH6_TRANS_COUNT, 1);
    wr(&mut m, CH6_CTRL_TRIG, EN | INCR_READ | (TREQ_PERMANENT << TREQ_SEL_SHIFT) | (2 << DATA_SIZE_SHIFT));
    assert_eq!(rd(&mut m, CH6_CTRL_TRIG) & BUSY, BUSY);

    // MockClock.advance(32) hands `tick` 32 us worth of nanoseconds.
    ns(&mut m, 32_000.0);

    assert_eq!(rd(&mut m, CH2_AL3_READ_ADDR_TRIG), 0);
    assert_eq!(rd(&mut m, CH2_AL1_CTRL) & BUSY, 0);
    assert_eq!(rd(&mut m, CH6_CTRL_TRIG) & BUSY, 0);
    assert_eq!(rd(&mut m, DMA_INTR), bit(2) | bit(6));
    for i in 0..8u32 {
        assert_eq!(m.read_u16(dest + 4 * i), 0x10 * (i + 1));
    }
    assert_eq!(m.read_u16(dest + 32), 0);
}

#[test]
fn upstream_dma_should_not_dispatch_the_offset_just_past_the_channel_registers_to_a_non_existent_channel() {
    let mut m = Rp2040::new();
    // 0x300 is the first address after the twelve 0x40-byte channel blocks.
    m.read_u32(DMA + 0x300);
    m.write_u32(DMA + 0x300, 0x1234);
    assert!(m.stop.is_none());
}

// ---- datasheet-derived ---------------------------------------------------------------

#[test]
fn dma_channel_registers_reset_with_chain_to_equal_to_the_channel_number() {
    // Datasheet 2.5.7, CHn_CTRL_TRIG: CHAIN_TO [14:11] resets to the channel
    // number (a channel chained to itself is not chained); every other field
    // resets to 0, so the channel is disabled, idle, byte-sized, TREQ 0.
    let mut m = Rp2040::new();
    for n in 0..12 {
        assert_eq!(rd(&mut m, ch(n, CTRL_TRIG)), n << CHAIN_TO_SHIFT, "channel {n}");
        assert_eq!(rd(&mut m, ch(n, READ_ADDR)), 0);
        assert_eq!(rd(&mut m, ch(n, WRITE_ADDR)), 0);
        assert_eq!(rd(&mut m, ch(n, TRANS_COUNT)), 0);
    }
    assert_eq!(rd(&mut m, DMA_INTR), 0);
    assert_eq!(rd(&mut m, DMA + 0x448), 12, "N_CHANNELS");
}

#[test]
fn dma_word_copy_advances_both_addresses_and_counts_down() {
    // Datasheet 2.5.2: each transfer moves one DATA_SIZE unit, READ_ADDR and
    // WRITE_ADDR step by it when INCR_READ and INCR_WRITE are set, TRANS_COUNT
    // counts the remaining transfers, BUSY drops after the last.
    let mut m = Rp2040::new();
    let (src, dst) = (RAM + 0x1000, RAM + 0x2000);
    for i in 0..4 {
        wr(&mut m, src + 4 * i, 0xa000_0000 + i);
    }
    dma_copy(&mut m, 3, src, dst, 4, 2, INCR_READ | INCR_WRITE);
    assert_ne!(rd(&mut m, ch(3, CTRL_TRIG)) & BUSY, 0, "BUSY from the trigger");
    assert_eq!(rd(&mut m, ch(3, TRANS_COUNT)), 4);
    // One transfer per alarm, every one due now (a permanent request): the
    // whole block runs inside a zero-length tick.
    assert_eq!(rd(&mut m, ch(3, READ_ADDR)), src);
    ns(&mut m, 0.0);
    assert_eq!(rd(&mut m, ch(3, CTRL_TRIG)) & BUSY, 0);
    assert_eq!(rd(&mut m, ch(3, TRANS_COUNT)), 0);
    assert_eq!(rd(&mut m, ch(3, READ_ADDR)), src + 16);
    for i in 0..4 {
        assert_eq!(rd(&mut m, dst + 4 * i), 0xa000_0000 + i);
    }
    assert_eq!(rd(&mut m, DMA_INTR), bit(3), "the completion raises the channel's raw interrupt");
}

#[test]
fn dma_data_sizes_and_a_fixed_read_address() {
    // DATA_SIZE 0/1/2 = byte, halfword, word. With INCR_READ clear the same
    // source is read every time (a fill).
    let mut m = Rp2040::new();
    let (src, dst) = (RAM + 0x100, RAM + 0x200);
    m.write_u8(src, 0xab);
    dma_copy(&mut m, 0, src, dst, 5, 0, INCR_WRITE);
    ns(&mut m, 10.0);
    for i in 0..5 {
        assert_eq!(m.read_u8(dst + i), 0xab);
    }
    assert_eq!(m.read_u8(dst + 5), 0);
    wr(&mut m, src, 0x1234_5678);
    dma_copy(&mut m, 1, src, dst + 0x100, 3, 1, INCR_WRITE | INCR_READ);
    ns(&mut m, 10.0);
    assert_eq!(m.read_u16(dst + 0x100), 0x5678);
    assert_eq!(m.read_u16(dst + 0x102), 0x1234);
    assert_eq!(m.read_u16(dst + 0x104), 0);
}

#[test]
fn dma_ring_wraps_the_read_address_in_a_power_of_two_window() {
    // Datasheet 2.5.7 RING_SIZE: 1 << RING_SIZE bytes, wrapped on the read
    // address when RING_SEL is 0 and on the write address when it is 1.
    let mut m = Rp2040::new();
    let (src, dst) = (RAM + 0x400, RAM + 0x800);
    wr(&mut m, src, 0x11);
    wr(&mut m, src + 4, 0x22);
    dma_copy(&mut m, 0, src, dst, 6, 2, INCR_READ | INCR_WRITE | (3 << RING_SIZE_SHIFT));
    ns(&mut m, 10.0);
    let got: Vec<u32> = (0..6).map(|i| rd(&mut m, dst + 4 * i)).collect();
    assert_eq!(got, [0x11, 0x22, 0x11, 0x22, 0x11, 0x22]);
    // The write ring: eight bytes of destination written over and over.
    dma_copy(&mut m, 1, src, RAM + 0xc00, 4, 2, INCR_READ | INCR_WRITE | (3 << RING_SIZE_SHIFT) | RING_SEL);
    wr(&mut m, src + 8, 0x33);
    wr(&mut m, src + 12, 0x44);
    ns(&mut m, 10.0);
    // Reads walk 0x400.. without wrapping (0x11, 0x22, 0x33, 0x44); writes
    // wrap inside 0xc00..0xc08, so the last two land on the first two words.
    assert_eq!(rd(&mut m, RAM + 0xc00), 0x33);
    assert_eq!(rd(&mut m, RAM + 0xc04), 0x44);
}

#[test]
fn dma_byte_swap_reverses_the_bytes_of_each_unit() {
    // Datasheet 2.5.7 BSWAP: byte order of each transfer reversed, for
    // halfwords and words.
    let mut m = Rp2040::new();
    let (src, dst) = (RAM + 0x10, RAM + 0x20);
    wr(&mut m, src, 0x1122_3344);
    dma_copy(&mut m, 0, src, dst, 1, 2, BSWAP);
    ns(&mut m, 1.0);
    assert_eq!(rd(&mut m, dst), 0x4433_2211);
    dma_copy(&mut m, 1, src, dst + 4, 1, 1, BSWAP);
    ns(&mut m, 1.0);
    assert_eq!(m.read_u16(dst + 4), 0x4433);
}

#[test]
fn dma_completion_interrupts_follow_inte_and_intf_and_clear_on_a_written_one() {
    // Datasheet 2.5.7 INTR/INTE0/INTF0/INTS0: INTS = (INTR & INTE) | INTF;
    // writing a 1 to INTR (or to INTS) clears that raw bit. IRQ 0 is NVIC 11,
    // IRQ 1 is NVIC 12.
    let mut m = Rp2040::new();
    wr(&mut m, DMA_INTE0, bit(5));
    wr(&mut m, DMA_INTE1, bit(6));
    dma_copy(&mut m, 5, RAM, RAM + 0x40, 1, 0, 0);
    ns(&mut m, 1.0);
    assert_eq!(rd(&mut m, DMA_INTR), bit(5));
    assert_eq!(rd(&mut m, DMA_INTS0), bit(5));
    assert_eq!(rd(&mut m, DMA_INTS1), 0);
    assert_ne!(m.cpu.pending_interrupts & bit(11), 0);
    assert_eq!(m.cpu.pending_interrupts & bit(12), 0);
    wr(&mut m, DMA_INTR, bit(5));
    assert_eq!(rd(&mut m, DMA_INTR), 0);
    assert_eq!(m.cpu.pending_interrupts & bit(11), 0);
    // Force: IRQ 0 asserts with nothing raw.
    wr(&mut m, DMA_INTF0, bit(9));
    assert_eq!(rd(&mut m, DMA_INTS0), bit(9));
    assert_ne!(m.cpu.pending_interrupts & bit(11), 0);
    // INTE and INTF hold 16 bits.
    wr(&mut m, DMA_INTE0, 0xffff_ffff);
    assert_eq!(rd(&mut m, DMA_INTE0), 0xffff);
    // Clearing through INTS1 clears the raw bit just the same.
    dma_copy(&mut m, 6, RAM, RAM + 0x40, 1, 0, 0);
    ns(&mut m, 1.0);
    assert_eq!(rd(&mut m, DMA_INTR), bit(6));
    wr(&mut m, DMA_INTS1, bit(6));
    assert_eq!(rd(&mut m, DMA_INTR), 0);
}

#[test]
fn dma_irq_quiet_suppresses_the_completion_interrupt_and_a_null_trigger_raises_it() {
    // Datasheet 2.5.7 IRQ_QUIET: no interrupt on completion (only a null
    // trigger raises one), which is how a chain of control blocks ends.
    let mut m = Rp2040::new();
    dma_copy(&mut m, 4, RAM, RAM + 0x40, 1, 0, IRQ_QUIET);
    ns(&mut m, 1.0);
    assert_eq!(rd(&mut m, DMA_INTR), 0);
    wr(&mut m, ch(4, 0x3c), 0); // AL3_READ_ADDR_TRIG: a null trigger
    assert_eq!(rd(&mut m, DMA_INTR), bit(4));
}

#[test]
fn dma_abort_stops_the_channel_and_clears_busy() {
    let mut m = Rp2040::new();
    dma_copy(&mut m, 2, RAM, RAM + 0x100, 8, 0, INCR_READ | INCR_WRITE);
    wr(&mut m, DMA_CHAN_ABORT, bit(2));
    assert_eq!(rd(&mut m, ch(2, CTRL_TRIG)) & BUSY, 0);
    ns(&mut m, 100.0);
    assert_eq!(rd(&mut m, ch(2, TRANS_COUNT)), 8, "no transfer ran");
    assert_eq!(rd(&mut m, DMA_INTR), 0);
}

#[test]
fn dma_multi_channel_trigger_starts_every_selected_enabled_channel() {
    let mut m = Rp2040::new();
    for n in [1u32, 7] {
        wr(&mut m, ch(n, READ_ADDR), RAM);
        wr(&mut m, ch(n, WRITE_ADDR), RAM + 0x100 * n);
        wr(&mut m, ch(n, TRANS_COUNT), 1);
        wr(&mut m, ch(n, AL1_CTRL), EN | (TREQ_PERMANENT << TREQ_SEL_SHIFT) | (n << CHAIN_TO_SHIFT));
        assert_eq!(rd(&mut m, ch(n, CTRL_TRIG)) & BUSY, 0, "an AL1_CTRL write does not trigger");
    }
    wr(&mut m, RAM, 0x5a);
    wr(&mut m, DMA_MULTI_CHAN_TRIGGER, bit(1) | bit(7));
    ns(&mut m, 1.0);
    assert_eq!(m.read_u8(RAM + 0x100), 0x5a);
    assert_eq!(m.read_u8(RAM + 0x700), 0x5a);
    assert_eq!(rd(&mut m, DMA_INTR), bit(1) | bit(7));
}

#[test]
fn dma_atomic_alias_sets_a_ctrl_bit() {
    // +0x2000 is SET (datasheet 2.1.2): the channel's CTRL takes the OR.
    let mut m = Rp2040::new();
    wr(&mut m, ch(0, AL1_CTRL), 2 << DATA_SIZE_SHIFT);
    wr(&mut m, ch(0, AL1_CTRL) + SET, INCR_READ | EN);
    assert_eq!(rd(&mut m, ch(0, AL1_CTRL)), (2 << DATA_SIZE_SHIFT) | INCR_READ | EN);
}

#[test]
fn dma_request_line_gates_a_channel_until_a_peripheral_raises_it() {
    // Datasheet 2.5.3: a channel paced by a peripheral's DREQ moves one
    // transfer per request. The UART, SPI, I2C, PWM and ADC lines are raised
    // by R6's blocks through `dma_set_dreq`; here the test plays them.
    let mut m = Rp2040::new();
    wr(&mut m, ch(0, READ_ADDR), RAM);
    wr(&mut m, ch(0, WRITE_ADDR), RAM + 0x100);
    wr(&mut m, ch(0, TRANS_COUNT), 1);
    wr(&mut m, RAM, 0x77);
    wr(&mut m, ch(0, CTRL_TRIG), EN | (dreq::UART0_TX << TREQ_SEL_SHIFT));
    ns(&mut m, 1000.0);
    assert_eq!(m.read_u8(RAM + 0x100), 0, "no request, no transfer");
    assert_ne!(rd(&mut m, ch(0, CTRL_TRIG)) & BUSY, 0);
    m.dma_set_dreq(dreq::UART0_TX);
    ns(&mut m, 0.0);
    assert_eq!(m.read_u8(RAM + 0x100), 0x77);
    assert_eq!(rd(&mut m, ch(0, CTRL_TRIG)) & BUSY, 0);
    m.dma_clear_dreq(dreq::UART0_TX);
    assert!(!m.periph.dma.dreq[dreq::UART0_TX as usize]);
}

#[test]
fn dma_line_numbers_are_the_datasheet_dreq_table() {
    // Datasheet 2.5.7 TREQ_SEL: 0-3 PIO0 TX, 4-7 PIO0 RX, 8-11 PIO1 TX,
    // 12-15 PIO1 RX, 16-19 SPI0/1 TX/RX, 20-23 UART0/1 TX/RX, 24-31 PWM
    // wraps, 32-35 I2C, 36 ADC, 37-39 XIP.
    assert_eq!(
        [
            dreq::PIO0_TX0,
            dreq::PIO0_RX0,
            dreq::PIO1_TX0,
            dreq::PIO1_RX0,
            dreq::SPI0_TX,
            dreq::SPI1_RX,
            dreq::UART0_TX,
            dreq::UART1_RX,
            dreq::PWM_WRAP0,
            dreq::I2C0_TX,
            dreq::I2C1_RX,
            dreq::ADC,
            dreq::XIP_STREAM,
            dreq::XIP_SSIRX
        ],
        [0, 4, 8, 12, 16, 19, 20, 23, 24, 32, 35, 36, 37, 39]
    );
}

#[test]
fn reference_quirk_dma_pacing_timer_period_is_x_over_y_cycles_not_y_over_x() {
    // Datasheet 2.5.7 TIMERn: pacing happens at (X/Y) * sys_clk, so X = 1 and
    // Y = 2 means one request every 2 cycles (16 ns). rp2040js computes the
    // delay as (X/Y) cycles: 0.5 cycles, 4 ns, so three transfers finish in
    // 12 ns where hardware would need 48.
    let mut m = Rp2040::new();
    wr(&mut m, DMA_TIMER0, (1 << 16) | 2);
    dma_copy(&mut m, 0, RAM, RAM + 0x100, 3, 0, INCR_WRITE);
    // Rewrite the control word with TREQ_SEL = timer 0 (0x3b).
    wr(&mut m, ch(0, AL1_CTRL), EN | BUSY | INCR_WRITE | (0x3b << TREQ_SEL_SHIFT));
    wr(&mut m, ch(0, TRANS_COUNT), 3);
    ns(&mut m, 4.5);
    assert_eq!(rd(&mut m, ch(0, TRANS_COUNT)), 2, "one transfer at 4 ns");
    ns(&mut m, 8.0);
    assert_eq!(rd(&mut m, ch(0, TRANS_COUNT)), 0, "all three by 12 ns");
    assert_eq!(rd(&mut m, ch(0, CTRL_TRIG)) & BUSY, 0);
}

#[test]
fn reference_quirk_dma_timer3_takes_its_dividend_from_a_shift_by_36() {
    // Datasheet: TIMER3 is laid out as TIMER0..2, X in [31:16]. rp2040js
    // shifts by 36, which JavaScript reduces to 4, so the dividend is
    // `value >>> 4` (a 28-bit number) and a (X = 1, Y = 1) timer is not
    // 1 cycle but 0x10001 >>> 4 = 4096 cycles per unit.
    let mut m = Rp2040::new();
    wr(&mut m, DMA_TIMER3, (1 << 16) | 1);
    assert_eq!(rd(&mut m, DMA_TIMER3), 0x0001_0001);
    wr(&mut m, ch(0, READ_ADDR), RAM);
    wr(&mut m, ch(0, WRITE_ADDR), RAM + 0x100);
    wr(&mut m, ch(0, TRANS_COUNT), 1);
    wr(&mut m, ch(0, CTRL_TRIG), EN | (0x3e << TREQ_SEL_SHIFT));
    // delay = (0x1000 / 1) * 1e6 / 125e6 us = 32.768 us.
    let due = m.clock.alarms.iter().find(|(id, _)| matches!(id, clock::AlarmId::Dma(0))).map(|&(_, t)| t);
    assert_eq!(due, Some(m.clock.nanos + (0x1000u32 as f64 / 1.0) * 1e6 / 125e6 * 1000.0));
}

#[test]
fn reference_quirk_dma_zero_count_trigger_leaves_the_channel_busy_with_no_alarm() {
    // A trigger with TRANS_COUNT = 0 sets BUSY, loads a zero count and
    // schedules nothing: the channel never completes and never interrupts.
    let mut m = Rp2040::new();
    wr(&mut m, ch(0, CTRL_TRIG), EN | (TREQ_PERMANENT << TREQ_SEL_SHIFT));
    assert_ne!(rd(&mut m, ch(0, CTRL_TRIG)) & BUSY, 0);
    assert!(m.clock.alarms.is_empty());
    ns(&mut m, 1e6);
    assert_ne!(rd(&mut m, ch(0, CTRL_TRIG)) & BUSY, 0);
    assert_eq!(rd(&mut m, DMA_INTR), 0);
}

#[test]
fn reference_quirk_dma_sub_word_count_write_is_a_signed_number() {
    // A byte store replicates the byte into all four lanes through `|` and
    // `<<`, which makes an int32 in JavaScript. A count byte of 0x80 is
    // therefore 0x80808080 as a negative number: the first transfer ends the
    // block (`count - 1 > 0` fails). A word store of the same value would be
    // 2.1 billion transfers.
    let mut m = Rp2040::new();
    m.write_u8(ch(0, TRANS_COUNT), 0x80);
    assert_eq!(rd(&mut m, DMA + 0x804), 0x8080_8080, "CH0_DBG_TCR holds the reload value");
    wr(&mut m, ch(0, READ_ADDR), RAM);
    wr(&mut m, ch(0, WRITE_ADDR), RAM + 0x40);
    wr(&mut m, ch(0, CTRL_TRIG), EN | (TREQ_PERMANENT << TREQ_SEL_SHIFT));
    ns(&mut m, 0.0);
    assert_eq!(rd(&mut m, ch(0, CTRL_TRIG)) & BUSY, 0, "done after one transfer");
    assert_eq!(rd(&mut m, ch(0, TRANS_COUNT)), 0x8080_807f, "counted down once from -2139062144");
    // The plain word store keeps the unsigned value.
    wr(&mut m, ch(1, TRANS_COUNT), 0x8080_8080);
    assert_eq!(m.periph.dma.channels[1].trans_count_reload, 0x8080_8080);
}

// ======================================================================================
// PIO
// ======================================================================================

const CTRL: u32 = PIO0;
const FSTAT: u32 = PIO0 + 0x04;
const FDEBUG: u32 = PIO0 + 0x08;
const FLEVEL: u32 = PIO0 + 0x0c;
const TXF0: u32 = PIO0 + 0x10;
const RXF0: u32 = PIO0 + 0x20;
const IRQ: u32 = PIO0 + 0x30;
const IRQ_FORCE: u32 = PIO0 + 0x34;
const DBG_PADOUT: u32 = PIO0 + 0x3c;
const DBG_PADOE: u32 = PIO0 + 0x40;
const DBG_CFGINFO: u32 = PIO0 + 0x44;
const INSTR_MEM0: u32 = PIO0 + 0x48;
const INSTR_MEM1: u32 = PIO0 + 0x4c;
const INSTR_MEM2: u32 = PIO0 + 0x50;
const INSTR_MEM3: u32 = PIO0 + 0x54;
const SM0_CLKDIV: u32 = PIO0 + 0xc8;
const SM0_EXECCTRL: u32 = PIO0 + 0xcc;
const SM0_SHIFTCTRL: u32 = PIO0 + 0xd0;
const SM0_ADDR: u32 = PIO0 + 0xd4;
const SM0_INSTR: u32 = PIO0 + 0xd8;
const SM0_PINCTRL: u32 = PIO0 + 0xdc;
const SM2_INSTR: u32 = PIO0 + 0x108;
const INTR: u32 = PIO0 + 0x128;
const IRQ0_INTE: u32 = PIO0 + 0x12c;
const IRQ0_INTF: u32 = PIO0 + 0x130;
const IRQ0_INTS: u32 = PIO0 + 0x134;
const IRQ1_INTE: u32 = PIO0 + 0x138;
const IRQ1_INTS: u32 = PIO0 + 0x140;

const PIO_IRQ0: u32 = 1 << 7;
const INTR_SM0_RXNEMPTY: u32 = 1 << 0;
const INTR_SM0_TXNFULL: u32 = 1 << 4;
const TX0_SHIFT: u32 = 0;
const RX0_SHIFT: u32 = 4;
const FJOIN_RX: u32 = 1 << 30;
const FJOIN_TX: u32 = 1 << 31;
const IN_SHIFTDIR: u32 = 1 << 18;
const OUT_SHIFTDIR: u32 = 1 << 19;
const SHIFTCTRL_AUTOPULL: u32 = 1 << 17;
const SHIFTCTRL_AUTOPUSH: u32 = 1 << 16;
const SHIFTCTRL_PULL_THRESH_SHIFT: u32 = 25;
const SHIFTCTRL_PUSH_THRESH_SHIFT: u32 = 20;
const EXECCTRL_EXEC_STALLED: u32 = 1 << 31;
const EXECCTRL_STATUS_SEL: u32 = 1 << 4;
const EXECCTRL_WRAP_BOTTOM_SHIFT: u32 = 7;
const EXECCTRL_WRAP_TOP_SHIFT: u32 = 12;
const FDEBUG_TXSTALL: u32 = 1 << 24;
const SET_COUNT_SHIFT: u32 = 26;
const SET_COUNT_BASE: u32 = 5;
const OUT_COUNT_SHIFT: u32 = 20;
const VALID_PINS_MASK: u32 = 0x3fff_ffff;

/// `resetStateMachines` of pio.spec.ts.
fn reset_state_machines(m: &mut Rp2040) {
    wr(m, CTRL, 0xf0);
    wr(m, SM0_INSTR, jmp(COND_ALWAYS, 0, 0)); // Jump machine 0 to address 0
    wr(m, SM0_SHIFTCTRL, FJOIN_RX); // "Clear FIFOs"
    wr(m, SM0_SHIFTCTRL, IN_SHIFTDIR | OUT_SHIFTDIR); // Values at reset
    wr(m, SM0_PINCTRL, 5 << SET_COUNT_SHIFT);
}

fn pio_machine(m: &mut Rp2040, p: usize, i: usize) -> &mut pio::StateMachine {
    &mut m.periph.pio[p].machines[i]
}

// ---- src/utils/pio-assembler.spec.ts -------------------------------------------------

#[test]
fn upstream_pio_assembler_jmp_pin_5() {
    assert_eq!(jmp(COND_PIN, 5, 0), 0xc5);
}
#[test]
fn upstream_pio_assembler_wait_1_gpio_12() {
    assert_eq!(wait(true, WAIT_SRC_GPIO, 12, 0), 0x208c);
}
#[test]
fn upstream_pio_assembler_in_x_12() {
    assert_eq!(input(SRC_X, 12, 0), 0x402c);
}
#[test]
fn upstream_pio_assembler_out_y_30() {
    assert_eq!(out(DEST_Y, 30, 0), 0x605e);
}
#[test]
fn upstream_pio_assembler_push_iffull_noblock() {
    assert_eq!(push(true, true, 12), 0x8c60);
}
#[test]
fn upstream_pio_assembler_pull_block() {
    assert_eq!(pull(true, false, 0), 0x80c0);
}
#[test]
fn upstream_pio_assembler_mov_x_not_status() {
    assert_eq!(mov(MOV_DEST_X, OP_INVERT, SRC_STATUS, 0), 0xa02d);
}
#[test]
fn upstream_pio_assembler_irq_set_4() {
    assert_eq!(irq(false, false, 4, 0), 0xc004);
}
#[test]
fn upstream_pio_assembler_set_x_12() {
    assert_eq!(set(MOV_DEST_X, 12, 0), 0xe02c);
}

// ---- src/peripherals/pio.spec.ts -----------------------------------------------------

#[test]
fn upstream_pio_set_pins() {
    // SET PINS, 13 then check DBG_PADOUT against the pins value.
    let (shift, qty, value) = (0u32, 5u32, 13u32);
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, (qty << SET_COUNT_SHIFT) | (shift << SET_COUNT_BASE));
    wr(&mut m, SM0_INSTR, set(DEST_PINS, value, 0));
    assert_eq!(rd(&mut m, DBG_PADOUT) & (((1 << qty) - 1) << shift), value << shift);
}

#[test]
fn upstream_pio_mov_pins_x() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    wr(&mut m, SM0_INSTR, set(DEST_X, 8, 0));
    wr(&mut m, SM0_INSTR, mov(DEST_PINS, OP_NONE, SRC_X, 0));
    assert_eq!(rd(&mut m, DBG_PADOUT), 8);
}

#[test]
fn upstream_pio_mov_pins_not_x() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    wr(&mut m, SM0_INSTR, set(DEST_X, 29, 0));
    wr(&mut m, SM0_INSTR, mov(DEST_PINS, OP_INVERT, SRC_X, 0));
    assert_eq!(rd(&mut m, DBG_PADOUT), !29u32 & VALID_PINS_MASK);
}

#[test]
fn upstream_pio_mov_pins_bit_reversed_x() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    wr(&mut m, SM0_INSTR, set(DEST_X, 0b11001, 0));
    wr(&mut m, SM0_INSTR, mov(DEST_PINS, OP_BITREV, SRC_X, 0));
    assert_eq!(rd(&mut m, DBG_PADOUT), 0x9800_0000 & VALID_PINS_MASK);
}

#[test]
fn upstream_pio_mov_y_x() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    wr(&mut m, SM0_INSTR, set(DEST_X, 11, 0));
    wr(&mut m, SM0_INSTR, mov(MOV_DEST_Y, OP_NONE, SRC_X, 0));
    wr(&mut m, SM0_INSTR, mov(DEST_PINS, OP_NONE, SRC_Y, 0));
    assert_eq!(rd(&mut m, DBG_PADOUT), 11);
}

#[test]
fn upstream_pio_mov_pc_y() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, set(DEST_Y, 23, 0));
    wr(&mut m, SM0_INSTR, mov(5, OP_NONE, SRC_Y, 0)); // MOV PC
    assert_eq!(rd(&mut m, SM0_ADDR), 23);
}

#[test]
fn upstream_pio_mov_isr_status_tx_fifo() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_EXECCTRL, 2);
    wr(&mut m, SM0_INSTR, mov(MOV_DEST_ISR, OP_NONE, SRC_STATUS, 0));
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    assert_eq!(rd(&mut m, RXF0), 0xffff_ffff);
    wr(&mut m, TXF0, 1);
    wr(&mut m, TXF0, 2);
    wr(&mut m, SM0_INSTR, mov(MOV_DEST_ISR, OP_NONE, SRC_STATUS, 0));
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    assert_eq!(rd(&mut m, RXF0), 0);
}

#[test]
fn upstream_pio_mov_isr_status_rx_fifo() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_EXECCTRL, 1 | EXECCTRL_STATUS_SEL);
    wr(&mut m, SM0_INSTR, mov(MOV_DEST_ISR, OP_NONE, SRC_STATUS, 0));
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    wr(&mut m, SM0_INSTR, mov(MOV_DEST_ISR, OP_NONE, SRC_STATUS, 0));
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    assert_eq!(rd(&mut m, RXF0), 0xffff_ffff);
    assert_eq!(rd(&mut m, RXF0), 0);
}

#[test]
fn upstream_pio_jmp_always() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_INSTR, jmp(COND_ALWAYS, 10, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 10);
}

#[test]
fn upstream_pio_jmp_not_x() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, set(DEST_X, 5, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_NOTX, 8, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, set(DEST_X, 0, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_NOTX, 8, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 8);
}

#[test]
fn upstream_pio_jmp_x_decrement() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, set(DEST_X, 5, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_XDEC, 12, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 12);
    wr(&mut m, SM0_INSTR, mov(DEST_PINS, OP_NONE, SRC_X, 0));
    assert_eq!(rd(&mut m, DBG_PADOUT), 4);
    wr(&mut m, SM0_INSTR, set(DEST_X, 0, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_XDEC, 6, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 12);
}

#[test]
fn upstream_pio_jmp_not_y() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, set(DEST_Y, 6, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_NOTY, 8, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, set(DEST_Y, 0, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_NOTY, 8, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 8);
}

#[test]
fn upstream_pio_jmp_y_decrement() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, set(DEST_Y, 15, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_YDEC, 12, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 12);
    wr(&mut m, SM0_INSTR, mov(DEST_PINS, OP_NONE, SRC_Y, 0));
    assert_eq!(rd(&mut m, DBG_PADOUT), 14);
    wr(&mut m, SM0_INSTR, set(DEST_Y, 0, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_YDEC, 6, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 12);
}

#[test]
fn upstream_pio_jmp_x_not_equal_y() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, set(DEST_X, 23, 0));
    wr(&mut m, SM0_INSTR, set(DEST_Y, 23, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_XNEY, 26, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, set(DEST_X, 3, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_XNEY, 26, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 26);
}

#[test]
fn upstream_pio_jmp_osre() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, mov(MOV_DEST_OSR, OP_NONE, SRC_NULL, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_NOTEMPTYOSR, 11, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 11);
    wr(&mut m, SM0_INSTR, out(DEST_NULL, 32, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_NOTEMPTYOSR, 22, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 11);
}

#[test]
fn upstream_pio_program_with_pull() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    wr(&mut m, TXF0, 0x42f0_0d43);
    assert_eq!(rd(&mut m, FLEVEL), 1 << TX0_SHIFT);
    wr(&mut m, SM0_INSTR, pull(false, false, 0));
    assert_eq!(rd(&mut m, FLEVEL), 0);
    wr(&mut m, SM0_INSTR, out(DEST_PINS, 32, 0));
    assert_eq!(rd(&mut m, DBG_PADOUT), 0x42f0_0d43 & VALID_PINS_MASK);
}

#[test]
fn upstream_pio_out_exec() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    wr(&mut m, TXF0, jmp(COND_ALWAYS, 16, 0));
    wr(&mut m, SM0_INSTR, pull(false, false, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, out(DEST_EXEC, 32, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 16);
}

#[test]
fn upstream_pio_out_pc() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, 32 << OUT_COUNT_SHIFT);
    wr(&mut m, TXF0, 29);
    wr(&mut m, SM0_INSTR, pull(false, false, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    wr(&mut m, SM0_INSTR, out(DEST_PC, 32, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 29);
}

#[test]
fn upstream_pio_program_with_push() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    assert_eq!(rd(&mut m, FLEVEL), 0);
    wr(&mut m, SM0_INSTR, set(DEST_X, 9, 0));
    wr(&mut m, SM0_INSTR, input(SRC_X, 32, 0));
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    assert_eq!(rd(&mut m, FLEVEL), 1 << RX0_SHIFT);
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    assert_eq!(rd(&mut m, FLEVEL), 2 << RX0_SHIFT);
    assert_eq!(rd(&mut m, RXF0), 9);
    assert_eq!(rd(&mut m, FLEVEL), 1 << RX0_SHIFT);
    assert_eq!(rd(&mut m, RXF0), 0, "ISR is zeroed after the first push");
    assert_eq!(rd(&mut m, FLEVEL), 0);
}

#[test]
fn upstream_pio_irq_2() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, IRQ, 0xff);
    wr(&mut m, SM0_INSTR, irq(false, false, 2, 0));
    assert_eq!(rd(&mut m, IRQ), 1 << 2);
    wr(&mut m, SM0_INSTR, irq(true, false, 2, 0));
    assert_eq!(rd(&mut m, IRQ), 0);
}

#[test]
fn upstream_pio_irq_3_rel() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, IRQ, 0xff);
    wr(&mut m, SM2_INSTR, irq(false, false, 0x13, 0));
    assert_eq!(rd(&mut m, IRQ), 1 << 1);
    wr(&mut m, SM2_INSTR, irq(true, false, 0x13, 0));
    assert_eq!(rd(&mut m, IRQ), 0);
    wr(&mut m, SM0_INSTR, irq(false, false, 0x13, 0));
    assert_eq!(rd(&mut m, IRQ), 1 << 3);
}

#[test]
fn upstream_pio_wait_1_irq_7() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, IRQ, 0xff);
    wr(&mut m, INSTR_MEM0, mov(MOV_DEST_X, OP_NONE, SRC_X, 0));
    wr(&mut m, INSTR_MEM1, wait(true, WAIT_SRC_IRQ, 7, 0));
    wr(&mut m, INSTR_MEM2, jmp(COND_ALWAYS, 2, 0));
    wr(&mut m, CTRL, 1); // Starts State Machine #0
    assert_eq!(rd(&mut m, SM0_ADDR), 1);
    wr(&mut m, SM2_INSTR, irq(false, false, 5, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 1);
    wr(&mut m, SM2_INSTR, irq(false, false, 7, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 2);
    assert_eq!(rd(&mut m, IRQ), 1 << 5, "the wait cleared IRQ 7");
}

#[test]
fn upstream_pio_wait_0_irq_7() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, IRQ, 0xff);
    wr(&mut m, SM0_INSTR, irq(false, false, 7, 0));
    wr(&mut m, INSTR_MEM0, mov(MOV_DEST_X, OP_NONE, SRC_X, 0));
    wr(&mut m, INSTR_MEM1, wait(false, WAIT_SRC_IRQ, 7, 0));
    wr(&mut m, INSTR_MEM2, jmp(COND_ALWAYS, 2, 0));
    wr(&mut m, CTRL, 1);
    assert_eq!(rd(&mut m, SM0_ADDR), 1);
    wr(&mut m, IRQ, 1 << 7);
    assert_eq!(rd(&mut m, SM0_ADDR), 2);
}

#[test]
fn upstream_pio_intr_follows_irq_2() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, IRQ, 0xff);
    wr(&mut m, SM2_INSTR, irq(false, false, 0x2, 0));
    assert_eq!(rd(&mut m, INTR) & 0xf00, 1 << 10);
}

#[test]
fn upstream_pio_compare_x_to_ffffffff_after_mov_x_not_null() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, TXF0, 0xffff_ffff);
    wr(&mut m, SM0_INSTR, pull(false, false, 0));
    wr(&mut m, SM0_INSTR, mov(DEST_X, OP_INVERT, SRC_NULL, 0));
    wr(&mut m, SM0_INSTR, out(DEST_Y, 32, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_ALWAYS, 8, 0));
    wr(&mut m, SM0_INSTR, jmp(COND_XNEY, 16, 0)); // shouldn't take the jump
    assert_eq!(rd(&mut m, SM0_ADDR), 8);
}

#[test]
fn upstream_pio_wraps_at_wrap_top() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_EXECCTRL, (1 << EXECCTRL_WRAP_BOTTOM_SHIFT) | (2 << EXECCTRL_WRAP_TOP_SHIFT));
    // jmp .label2 / .wrap_target / label1: jmp label1 / label2: mov x, null /
    // .wrap / label3: jmp label3
    wr(&mut m, INSTR_MEM0, jmp(COND_ALWAYS, 2, 0));
    wr(&mut m, INSTR_MEM1, jmp(COND_ALWAYS, 1, 0));
    wr(&mut m, INSTR_MEM2, mov(MOV_DEST_X, OP_NONE, SRC_X, 0));
    wr(&mut m, INSTR_MEM3, jmp(COND_ALWAYS, 3, 0));
    wr(&mut m, CTRL, 1);
    assert_eq!(rd(&mut m, SM0_ADDR), 1);
}

#[test]
fn upstream_pio_autopull() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_SHIFTCTRL, SHIFTCTRL_AUTOPULL | (4 << SHIFTCTRL_PULL_THRESH_SHIFT) | OUT_SHIFTDIR);
    wr(&mut m, TXF0, 0x5);
    wr(&mut m, TXF0, 0x6);
    wr(&mut m, TXF0, 0x7);
    for _ in 0..3 {
        wr(&mut m, SM0_INSTR, out(DEST_X, 4, 0));
        wr(&mut m, SM0_INSTR, input(SRC_X, 4, 0));
    }
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    assert_eq!(rd(&mut m, RXF0), 0x567);
}

#[test]
fn upstream_pio_no_autopull_in_the_middle_of_out() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_SHIFTCTRL, SHIFTCTRL_AUTOPULL | (4 << SHIFTCTRL_PULL_THRESH_SHIFT) | OUT_SHIFTDIR);
    wr(&mut m, TXF0, 0x25);
    wr(&mut m, TXF0, 0x36);
    wr(&mut m, SM0_INSTR, out(DEST_X, 8, 0));
    wr(&mut m, SM0_INSTR, input(SRC_X, 8, 0));
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    assert_eq!(rd(&mut m, RXF0), 0x25);
}

#[test]
fn upstream_pio_out_with_autopull_stalls_until_the_tx_fifo_fills() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_SHIFTCTRL, SHIFTCTRL_AUTOPULL | (4 << SHIFTCTRL_PULL_THRESH_SHIFT) | OUT_SHIFTDIR);
    wr(&mut m, SM0_INSTR, out(DEST_X, 4, 0));
    assert_eq!(rd(&mut m, SM0_EXECCTRL) & EXECCTRL_EXEC_STALLED, EXECCTRL_EXEC_STALLED);
    wr(&mut m, TXF0, 0x36); // unstalls the machine
    assert_eq!(rd(&mut m, SM0_EXECCTRL) & EXECCTRL_EXEC_STALLED, 0);
}

#[test]
fn upstream_pio_autopush() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_SHIFTCTRL, SHIFTCTRL_AUTOPUSH | (8 << SHIFTCTRL_PUSH_THRESH_SHIFT) | OUT_SHIFTDIR);
    wr(&mut m, SM0_INSTR, set(DEST_X, 0x13, 0));
    wr(&mut m, SM0_INSTR, input(SRC_X, 8, 0));
    assert_eq!(rd(&mut m, RXF0), 0x13);
}

#[test]
fn upstream_pio_autopush_only_at_the_end_of_the_in_instruction() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_SHIFTCTRL, SHIFTCTRL_AUTOPUSH | (8 << SHIFTCTRL_PUSH_THRESH_SHIFT) | OUT_SHIFTDIR);
    wr(&mut m, SM0_INSTR, mov(MOV_DEST_X, OP_INVERT, SRC_NULL, 0));
    wr(&mut m, SM0_INSTR, input(SRC_X, 16, 0));
    assert_eq!(rd(&mut m, RXF0), 0xffff);
}

#[test]
fn upstream_pio_in_with_autopush_stalls_until_the_rx_fifo_has_capacity() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_SHIFTCTRL, SHIFTCTRL_AUTOPUSH | (8 << SHIFTCTRL_PUSH_THRESH_SHIFT) | OUT_SHIFTDIR);
    for v in 15..=19 {
        wr(&mut m, SM0_INSTR, set(DEST_X, v, 0));
        wr(&mut m, SM0_INSTR, input(SRC_X, 8, 0)); // the fifth fills the RX FIFO and stalls
    }
    assert_eq!(rd(&mut m, SM0_EXECCTRL) & EXECCTRL_EXEC_STALLED, EXECCTRL_EXEC_STALLED);
    assert_eq!(m.read_u16(RXF0), 15); // unstalls the machine
    assert_eq!(rd(&mut m, SM0_EXECCTRL) & EXECCTRL_EXEC_STALLED, 0);
    for v in 16..=19 {
        assert_eq!(m.read_u16(RXF0), v);
    }
}

#[test]
fn upstream_pio_txnfull_in_intr_follows_the_tx_fifo_level_issue_73() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, IRQ0_INTE, INTR_SM0_TXNFULL);
    assert_eq!(rd(&mut m, INTR) & INTR_SM0_TXNFULL, INTR_SM0_TXNFULL);
    assert_eq!(rd(&mut m, NVIC_ISPR) & PIO_IRQ0, PIO_IRQ0);
    wr(&mut m, TXF0, 1);
    wr(&mut m, TXF0, 2);
    wr(&mut m, TXF0, 3);
    wr(&mut m, NVIC_ICPR, PIO_IRQ0);
    assert_eq!(rd(&mut m, INTR) & INTR_SM0_TXNFULL, INTR_SM0_TXNFULL);
    wr(&mut m, TXF0, 3);
    wr(&mut m, NVIC_ICPR, PIO_IRQ0);
    // The TX FIFO is full: the flag and the interrupt are cleared.
    assert_eq!(rd(&mut m, INTR) & INTR_SM0_TXNFULL, 0);
    assert_eq!(rd(&mut m, NVIC_ISPR) & PIO_IRQ0, 0);
    // Pull an item: not full again.
    wr(&mut m, SM0_INSTR, pull(false, false, 0));
    assert_eq!(rd(&mut m, INTR) & INTR_SM0_TXNFULL, INTR_SM0_TXNFULL);
    assert_eq!(rd(&mut m, NVIC_ISPR) & PIO_IRQ0, PIO_IRQ0);
}

#[test]
fn upstream_pio_txstall_in_fdebug_sets_on_an_empty_pull_and_clears_once_unstalled() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, FDEBUG, 0xffff_ffff);
    assert_eq!(rd(&mut m, FDEBUG), 0);
    assert_eq!(rd(&mut m, FLEVEL), 0 << TX0_SHIFT);
    wr(&mut m, SM0_INSTR, pull(false, false, 0));
    assert_eq!(rd(&mut m, FDEBUG) & FDEBUG_TXSTALL, FDEBUG_TXSTALL);
    // Still stalled: clearing the flag does nothing.
    wr(&mut m, FDEBUG, FDEBUG_TXSTALL);
    assert_eq!(rd(&mut m, FDEBUG) & FDEBUG_TXSTALL, FDEBUG_TXSTALL);
    wr(&mut m, TXF0, 42);
    wr(&mut m, FDEBUG, FDEBUG_TXSTALL);
    assert_eq!(rd(&mut m, FDEBUG) & FDEBUG_TXSTALL, 0);
}

#[test]
fn upstream_pio_rxnempty_in_intr_follows_the_rx_fifo_level_issue_73() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, IRQ0_INTE, INTR_SM0_RXNEMPTY);
    wr(&mut m, NVIC_ICPR, PIO_IRQ0);
    assert_eq!(rd(&mut m, INTR) & INTR_SM0_RXNEMPTY, 0);
    assert_eq!(rd(&mut m, NVIC_ISPR) & PIO_IRQ0, 0);
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    assert_eq!(rd(&mut m, INTR) & INTR_SM0_RXNEMPTY, INTR_SM0_RXNEMPTY);
    assert_eq!(rd(&mut m, NVIC_ISPR) & PIO_IRQ0, PIO_IRQ0);
    rd(&mut m, RXF0);
    wr(&mut m, NVIC_ICPR, PIO_IRQ0);
    assert_eq!(rd(&mut m, INTR) & INTR_SM0_RXNEMPTY, 0);
    assert_eq!(rd(&mut m, NVIC_ISPR) & PIO_IRQ0, 0);
}

// ---- datasheet-derived ---------------------------------------------------------------

#[test]
fn pio_registers_reset_to_the_datasheet_values() {
    // Datasheet 3.7: FSTAT TXEMPTY[27:24] and RXEMPTY[11:8] set for all four
    // machines (0x0f000f00); CLKDIV int 1 (0x00010000); EXECCTRL WRAP_TOP =
    // 31 (0x0001f000); SHIFTCTRL IN/OUT_SHIFTDIR set (0x000c0000); PINCTRL
    // SET_COUNT = 5 (0x14000000); ADDR 0; CTRL, FDEBUG, FLEVEL, IRQ 0.
    let mut m = Rp2040::new();
    for p in [PIO0, PIO1] {
        assert_eq!(rd(&mut m, p + 0x04), 0xf << 24 | 0xf << 8);
        for sm in 0..4 {
            let b = p + 0xc8 + 0x18 * sm;
            assert_eq!(rd(&mut m, b), 1 << 16);
            assert_eq!(rd(&mut m, b + 4), 0x1f << 12);
            assert_eq!(rd(&mut m, b + 8), 0b11 << 18);
            assert_eq!(rd(&mut m, b + 12), 0);
            assert_eq!(rd(&mut m, b + 20), 5 << 26);
        }
        for off in [0x00, 0x08, 0x0c, 0x30] {
            assert_eq!(rd(&mut m, p + off), 0, "offset {off:#x}");
        }
    }
}

#[test]
fn pio_dbg_cfginfo_reports_four_deep_fifos_four_machines_and_32_instructions() {
    // Datasheet 3.7 DBG_CFGINFO: FIFO_DEPTH [5:0], SM_COUNT [11:8],
    // IMEM_SIZE [21:16].
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, DBG_CFGINFO), 4 | (4 << 8) | (32 << 16));
}

#[test]
fn pio_fifos_hold_four_words_and_report_overflow_and_underflow() {
    // Datasheet 3.5.2 and FDEBUG: TXOVER[19:16] on a write to a full TX FIFO,
    // RXUNDER[11:8] on a read of an empty RX FIFO; FLEVEL TXn at [4n+..]
    // nibbles (TX0 [3:0], RX0 [7:4], TX1 [11:8] ...); FSTAT TXFULL [19:16].
    let mut m = Rp2040::new();
    for v in 1..=4 {
        wr(&mut m, TXF0 + 4, v); // machine 1
    }
    assert_eq!(rd(&mut m, FLEVEL), 4 << 8);
    assert_eq!(rd(&mut m, FSTAT) & (1 << 17), 1 << 17, "TXFULL for SM1");
    assert_eq!(rd(&mut m, FSTAT) & (1 << 25), 0, "TXEMPTY for SM1 clear");
    wr(&mut m, TXF0 + 4, 5);
    assert_eq!(rd(&mut m, FLEVEL), 4 << 8, "the fifth word is dropped");
    assert_eq!(rd(&mut m, FDEBUG), 1 << 17, "TXOVER for SM1");
    assert_eq!(rd(&mut m, RXF0 + 8), 0);
    assert_eq!(rd(&mut m, FDEBUG), (1 << 17) | (1 << 10), "RXUNDER for SM2");
    // Write one to clear.
    wr(&mut m, FDEBUG, (1 << 17) | (1 << 10));
    assert_eq!(rd(&mut m, FDEBUG), 0);
}

#[test]
fn pio_irq_flags_are_set_by_irq_force_and_cleared_by_a_written_one() {
    // Datasheet 3.7 IRQ (write 1 to clear) and IRQ_FORCE (set).
    let mut m = Rp2040::new();
    wr(&mut m, IRQ_FORCE, 0b1010_0101);
    assert_eq!(rd(&mut m, IRQ), 0b1010_0101);
    wr(&mut m, IRQ, 0b0000_1111);
    assert_eq!(rd(&mut m, IRQ), 0b1010_0000);
    assert_eq!(rd(&mut m, IRQ_FORCE), 0, "IRQ_FORCE reads as zero");
}

#[test]
fn pio_interrupt_registers_combine_enable_force_and_the_raw_flags() {
    // Datasheet 3.7: INTR bits 0-3 SMn_RXNEMPTY, 4-7 SMn_TXNFULL, 8-11 SMn
    // (the IRQ flags); IRQx_INTS = (INTR & IRQx_INTE) | IRQx_INTF. IRQ0 and
    // IRQ1 are two lines per block (NVIC 7 and 8 for PIO0, 9 and 10 for PIO1).
    let mut m = Rp2040::new();
    // The TX FIFOs start not full: INTR = 0xf0.
    assert_eq!(rd(&mut m, INTR), 0xf0);
    wr(&mut m, IRQ0_INTE, 0x10 | 0x400);
    wr(&mut m, IRQ1_INTE, 0x20);
    wr(&mut m, IRQ0_INTF, 0x1);
    assert_eq!(rd(&mut m, IRQ0_INTS), 0x10 | 0x1);
    assert_eq!(rd(&mut m, IRQ1_INTS), 0x20);
    assert_ne!(m.cpu.pending_interrupts & bit(7), 0);
    assert_ne!(m.cpu.pending_interrupts & bit(8), 0);
    wr(&mut m, IRQ_FORCE, 1 << 2);
    assert_eq!(rd(&mut m, INTR) & 0xf00, 1 << 10);
    assert_eq!(rd(&mut m, IRQ0_INTS), 0x10 | 0x400 | 0x1);
    // INTE and INTF hold 12 bits.
    wr(&mut m, IRQ0_INTE, 0xffff_ffff);
    assert_eq!(rd(&mut m, IRQ0_INTE), 0xfff);
    // PIO1's lines are NVIC 9 and 10.
    wr(&mut m, PIO1 + 0x12c, 0x10);
    assert_ne!(m.cpu.pending_interrupts & bit(9), 0);
}

#[test]
fn pio_set_pindirs_and_side_set_drive_the_pad_registers() {
    // Datasheet 3.4.10 SET and 3.5.1 side-set: SET PINDIRS writes the output
    // enables of SET_COUNT pins from SET_BASE; with SIDESET_COUNT = 2
    // (non-optional) bits [12:11] of every instruction are the side-set value
    // and [10:8] the delay. DBG_PADOE and DBG_PADOUT show them.
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, (5 << SET_COUNT_SHIFT) | (2 << SET_COUNT_BASE));
    wr(&mut m, SM0_INSTR, set(DEST_PINDIRS, 0b10111, 0));
    assert_eq!(rd(&mut m, DBG_PADOE), 0b10111 << 2);
    // Side-set 2 pins from pin 8: value 0b10, delay 5 (field 0b10101).
    wr(&mut m, SM0_PINCTRL, (5 << SET_COUNT_SHIFT) | (2 << 29) | (8 << 10));
    wr(&mut m, SM0_INSTR, set(DEST_Y, 0, 0b10101));
    assert_eq!(rd(&mut m, DBG_PADOUT), 0b10 << 8);
    // The instruction took 1 cycle and the delay 5.
    // JMP (to reset), SET PINDIRS, then this one: 3 + 5 delay cycles.
    assert_eq!(pio_machine(&mut m, 0, 0).cycles, 3 + 5);
    // SIDE_PINDIR: the side-set drives the directions instead.
    wr(&mut m, SM0_EXECCTRL, 1 << 29);
    wr(&mut m, SM0_INSTR, set(DEST_Y, 0, 0b01000));
    assert_eq!(rd(&mut m, DBG_PADOE) >> 8 & 3, 0b01);
}

#[test]
fn pio_optional_side_set_applies_only_when_the_enable_bit_is_set() {
    // Datasheet 3.5.1: with SIDE_EN, the top bit of the side-set field says
    // whether the value follows; SIDESET_COUNT includes it, so a count of 2
    // leaves one data bit.
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, (5 << SET_COUNT_SHIFT) | (2 << 29) | (4 << 10));
    wr(&mut m, SM0_EXECCTRL, (0x1f << 12) | (1 << 30));
    // Field 0b0_1_000: enable clear, data 1: ignored.
    wr(&mut m, SM0_INSTR, set(DEST_Y, 0, 0b01000));
    assert_eq!(rd(&mut m, DBG_PADOUT), 0);
    // Field 0b1_1_000: enable set, data 1: pin 4 high.
    wr(&mut m, SM0_INSTR, set(DEST_Y, 0, 0b11000));
    assert_eq!(rd(&mut m, DBG_PADOUT), 1 << 4);
}

#[test]
fn pio_in_pins_reads_the_gpio_inputs_from_in_base() {
    // Datasheet 3.4.4: IN PINS reads the pins starting at IN_BASE.
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    for pin in [6u32, 7, 9] {
        wr(&mut m, PADS_BANK0 + 4 + 4 * pin, 0x40 | 0x16); // input enable
        m.set_input_value(pin, true);
    }
    wr(&mut m, SM0_PINCTRL, (5 << SET_COUNT_SHIFT) | (6 << 15)); // IN_BASE = 6
    wr(&mut m, SM0_SHIFTCTRL, IN_SHIFTDIR | OUT_SHIFTDIR);
    wr(&mut m, SM0_INSTR, input(SRC_PINS, 4, 0));
    wr(&mut m, SM0_INSTR, push(false, false, 0));
    // Shifting right, 4 bits enter at the top: pins 6,7,8,9 = 1,1,0,1.
    assert_eq!(rd(&mut m, RXF0), 0b1011 << 28);
}

#[test]
fn pio_wait_gpio_resumes_when_the_pin_changes() {
    // Datasheet 3.4.3: WAIT 1 GPIO n stalls until the pin is high (as the
    // pad's input sees it).
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, PADS_BANK0 + 4 + 4 * 7, 0x40 | 0x16);
    wr(&mut m, INSTR_MEM0, wait(true, WAIT_SRC_GPIO, 7, 0));
    wr(&mut m, INSTR_MEM1, jmp(COND_ALWAYS, 1, 0));
    wr(&mut m, CTRL, 1);
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    assert!(pio_machine(&mut m, 0, 0).waiting);
    m.set_input_value(7, true);
    assert!(!pio_machine(&mut m, 0, 0).waiting);
    assert_eq!(rd(&mut m, SM0_ADDR), 1);
}

#[test]
fn pio_jmp_pin_branches_on_the_jmp_pin_input() {
    // Datasheet 3.4.2: JMP PIN branches if the EXECCTRL_JMP_PIN pin is high.
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, PADS_BANK0 + 4 + 4 * 11, 0x40 | 0x16);
    wr(&mut m, SM0_EXECCTRL, (0x1f << 12) | (11 << 24));
    wr(&mut m, SM0_INSTR, jmp(COND_PIN, 9, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    m.set_input_value(11, true);
    wr(&mut m, SM0_INSTR, jmp(COND_PIN, 9, 0));
    assert_eq!(rd(&mut m, SM0_ADDR), 9);
}

#[test]
fn pio_function_select_connects_a_pin_to_the_block_and_reports_the_edges() {
    // Datasheet 2.19.2: FUNCSEL 6 and 7 are PIO0 and PIO1. The pin drives
    // what the block's DBG_PADOUT holds where DBG_PADOE enables it; each
    // change is a pin event.
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, IO_BANK0 + 8 * 5 + 4, 6); // GPIO5 CTRL: FUNCSEL = PIO0
    wr(&mut m, IO_BANK0 + 8 * 6 + 4, 7); // GPIO6 CTRL: FUNCSEL = PIO1
    m.pin_events.clear();
    wr(&mut m, SM0_PINCTRL, (1 << SET_COUNT_SHIFT) | (5 << SET_COUNT_BASE));
    wr(&mut m, SM0_INSTR, set(DEST_PINDIRS, 1, 0));
    wr(&mut m, SM0_INSTR, set(DEST_PINS, 1, 0));
    // The writes of an instruction run through the block's step only; the
    // pad change is reported when the block next steps (checkChangedPins),
    // which is when enabled machines run.
    wr(&mut m, CTRL, 1);
    let ev: Vec<(u32, u32)> = m.pin_events.iter().map(|e| (e.pin, e.state)).collect();
    assert_eq!(ev, [(5, sio::PIN_HIGH)]);
    assert_eq!(rd(&mut m, IO_BANK0 + 8 * 5) & (1 << 8 | 1 << 12), 1 << 8 | 1 << 12, "STATUS: OUTFROMPERI, OEFROMPERI");
    // GPIO6 is on PIO1, whose pins are all low and undriven: still an input.
    assert_eq!(rd(&mut m, IO_BANK0 + 8 * 6) & (1 << 8 | 1 << 12), 0);
}

#[test]
fn pio_fifos_feed_the_dma_request_lines() {
    // Datasheet 2.5.3 DREQ 0..7 for PIO0: TXn is requested while the TX FIFO
    // is not full, RXn while the RX FIFO is not empty.
    let mut m = Rp2040::new();
    let tx = dreq::PIO0_TX0 + 2;
    let rx = dreq::PIO0_RX0 + 2;
    assert!(m.periph.dma.dreq[tx as usize], "TX FIFO empty at reset: request");
    assert!(!m.periph.dma.dreq[rx as usize]);
    for v in 0..4 {
        wr(&mut m, TXF0 + 8, v);
    }
    assert!(!m.periph.dma.dreq[tx as usize], "full: no request");
    wr(&mut m, SM0_INSTR + 0x30, pull(false, false, 0)); // SM2_INSTR
    assert!(m.periph.dma.dreq[tx as usize]);
    wr(&mut m, SM0_INSTR + 0x30, push(false, false, 0));
    assert!(m.periph.dma.dreq[rx as usize]);
    rd(&mut m, RXF0 + 8);
    assert!(!m.periph.dma.dreq[rx as usize]);
}

#[test]
fn pio_tx_dma_fills_the_fifo_to_four_words_and_resumes_as_the_machine_pulls() {
    let mut m = Rp2040::new();
    for i in 0..6u32 {
        wr(&mut m, RAM + 4 * i, 0x100 + i);
    }
    dma_copy(&mut m, 0, RAM, TXF0, 6, 2, INCR_READ);
    // Rewrite CTRL: TREQ_SEL = PIO0 TX0.
    wr(&mut m, ch(0, AL1_CTRL), EN | BUSY | INCR_READ | (2 << DATA_SIZE_SHIFT) | (dreq::PIO0_TX0 << TREQ_SEL_SHIFT));
    ns(&mut m, 100.0);
    assert_eq!(rd(&mut m, FLEVEL) & 0xf, 4, "the FIFO is full: the request line dropped");
    assert_eq!(rd(&mut m, ch(0, TRANS_COUNT)), 2);
    assert_ne!(rd(&mut m, ch(0, CTRL_TRIG)) & BUSY, 0);
    wr(&mut m, SM0_INSTR, pull(false, false, 0));
    ns(&mut m, 100.0);
    assert_eq!(rd(&mut m, ch(0, TRANS_COUNT)), 1, "one more word went in");
    wr(&mut m, SM0_INSTR, pull(false, false, 0));
    ns(&mut m, 100.0);
    assert_eq!(rd(&mut m, ch(0, CTRL_TRIG)) & BUSY, 0);
    // The words arrive in order: the first two were pulled, 4 remain.
    let mut got = vec![];
    for _ in 0..4 {
        wr(&mut m, SM0_INSTR, pull(false, false, 0));
        wr(&mut m, SM0_INSTR, out(DEST_X, 32, 0));
        got.push(pio_machine(&mut m, 0, 0).x);
    }
    assert_eq!(got, [0x102, 0x103, 0x104, 0x105]);
}

#[test]
fn pio_rx_dma_drains_the_fifo_into_memory() {
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    for v in [7u32, 8, 9] {
        wr(&mut m, SM0_INSTR, set(DEST_X, v, 0));
        wr(&mut m, SM0_INSTR, input(SRC_X, 32, 0));
        wr(&mut m, SM0_INSTR, push(false, false, 0));
    }
    wr(&mut m, ch(1, READ_ADDR), RXF0);
    wr(&mut m, ch(1, WRITE_ADDR), RAM + 0x200);
    wr(&mut m, ch(1, TRANS_COUNT), 3);
    wr(&mut m, ch(1, CTRL_TRIG), EN | INCR_WRITE | (2 << DATA_SIZE_SHIFT) | (dreq::PIO0_RX0 << TREQ_SEL_SHIFT) | (1 << CHAIN_TO_SHIFT));
    ns(&mut m, 100.0);
    assert_eq!([rd(&mut m, RAM + 0x200), rd(&mut m, RAM + 0x204), rd(&mut m, RAM + 0x208)], [7, 8, 9]);
    assert_eq!(rd(&mut m, ch(1, CTRL_TRIG)) & BUSY, 0);
}

// ---- how rp2040js runs the block (reference quirks) ---------------------------------

/// `jmp x--, 0` at 0 and 1: a machine whose X counts the steps it took.
fn counting_program(m: &mut Rp2040) {
    wr(m, INSTR_MEM0, jmp(COND_XDEC, 0, 0));
    wr(m, INSTR_MEM1, jmp(COND_XDEC, 1, 0));
    wr(m, SM0_INSTR, mov(MOV_DEST_X, OP_INVERT, SRC_NULL, 0)); // X = 0xffffffff
}

#[test]
fn reference_quirk_pio_runs_a_thousand_steps_when_enabled_then_a_batch_per_host_turn() {
    // Hardware runs a state machine from the system clock for ever; rp2040js
    // runs 1000 steps inside the CTRL write and leaves the rest to
    // `setTimeout(0)` callbacks, 1000 steps each, whenever the host's event
    // loop gets a turn.
    let mut m = Rp2040::new();
    counting_program(&mut m);
    wr(&mut m, CTRL, 1);
    assert_eq!(pio_machine(&mut m, 0, 0).x, 0xffff_ffff - 1000);
    assert_eq!(m.periph.pio_timers, [0]);
    m.pio_host_turn();
    assert_eq!(pio_machine(&mut m, 0, 0).x, 0xffff_ffff - 2000);
    m.pio_host_turn();
    m.pio_host_turn();
    assert_eq!(pio_machine(&mut m, 0, 0).x, 0xffff_ffff - 4000);
    // Time between host turns does not run the block.
    ns(&mut m, 1e9);
    assert_eq!(pio_machine(&mut m, 0, 0).x, 0xffff_ffff - 4000);
}

#[test]
fn reference_quirk_pio_ignores_the_clock_divider() {
    // Datasheet 3.5.5: SMx_CLKDIV slows a machine to sys_clk / (INT + FRAC /
    // 256). rp2040js keeps the fields and reads them back, and never uses
    // them: a divider of 1000 still runs 1000 steps per batch.
    let mut m = Rp2040::new();
    wr(&mut m, SM0_CLKDIV, (1000 << 16) | (0x80 << 8));
    assert_eq!(rd(&mut m, SM0_CLKDIV), (1000 << 16) | (0x80 << 8));
    counting_program(&mut m);
    wr(&mut m, CTRL, 1);
    assert_eq!(pio_machine(&mut m, 0, 0).x, 0xffff_ffff - 1000);
}

#[test]
fn reference_quirk_pio_steps_machines_that_are_not_enabled() {
    // Datasheet 3.7 CTRL.SM_ENABLE: a disabled machine does not execute.
    // rp2040js steps all four machines of a running block, whatever their
    // enable bits say.
    let mut m = Rp2040::new();
    counting_program(&mut m);
    wr(&mut m, CTRL, 1);
    assert!(!pio_machine(&mut m, 0, 1).enabled);
    assert_eq!(pio_machine(&mut m, 0, 1).x, 0xffff_ffff - 999, "SM1 ran and its X wrapped from 0");
    assert_eq!(rd(&mut m, CTRL), 1);
}

#[test]
fn reference_quirk_pio_stopping_and_restarting_before_the_timer_fires_doubles_the_rate() {
    // `run()` arms a new timer each batch and nothing cancels the old one when
    // CTRL stops the block: stop, start again before the host turn, and two
    // timers are pending, so each host turn runs two batches.
    let mut m = Rp2040::new();
    counting_program(&mut m);
    wr(&mut m, CTRL, 1);
    wr(&mut m, CTRL, 0);
    wr(&mut m, CTRL, 1);
    assert_eq!(m.periph.pio_timers, [0, 0]);
    let x = pio_machine(&mut m, 0, 0).x;
    m.pio_host_turn();
    assert_eq!(pio_machine(&mut m, 0, 0).x, x.wrapping_sub(2000));
    assert_eq!(m.periph.pio_timers, [0, 0]);
    // Stopping for good lets the pending timers expire without re-arming.
    wr(&mut m, CTRL, 0);
    m.pio_host_turn();
    assert!(m.periph.pio_timers.is_empty());
}

#[test]
fn reference_quirk_pio_fifo_join_does_not_deepen_or_flush_the_fifos() {
    // Datasheet 3.7 SHIFTCTRL FJOIN_TX: the TX FIFO takes the RX FIFO's
    // storage (8 deep) and both FIFOs flush when the bit changes. rp2040js
    // ignores the bit: still 4 deep, nothing flushed.
    let mut m = Rp2040::new();
    wr(&mut m, TXF0, 1);
    wr(&mut m, TXF0, 2);
    wr(&mut m, SM0_SHIFTCTRL, (0b11 << 18) | FJOIN_TX);
    assert_eq!(rd(&mut m, FLEVEL) & 0xf, 2);
    wr(&mut m, TXF0, 3);
    wr(&mut m, TXF0, 4);
    wr(&mut m, TXF0, 5);
    assert_eq!(rd(&mut m, FLEVEL) & 0xf, 4);
    assert_eq!(rd(&mut m, FDEBUG), 1 << 16, "the fifth word overflowed");
}

#[test]
fn reference_quirk_pio_instruction_memory_is_write_only_and_reads_all_ones() {
    // Datasheet 3.7 INSTR_MEMn are write-only (WO); rp2040js answers a read
    // like any unmodelled register, 0xffffffff.
    let mut m = Rp2040::new();
    wr(&mut m, INSTR_MEM0 + 4 * 5, 0xabcd_1234);
    assert_eq!(m.periph.pio[0].instructions[5], 0x1234, "16 bits are kept");
    assert_eq!(rd(&mut m, INSTR_MEM0 + 4 * 5), 0xffff_ffff);
}

#[test]
fn reference_quirk_pio_clkdiv_restart_and_the_fifo_registers_in_the_wrong_direction_are_unimplemented() {
    // CLKDIV_RESTART (CTRL [11:8]) only logs. A read of a TX FIFO register
    // answers all ones (the reference's unmodelled read).
    let mut m = Rp2040::new();
    wr(&mut m, CTRL, 0xf00);
    assert_eq!(rd(&mut m, CTRL), 0);
    assert_eq!(rd(&mut m, TXF0), 0xffff_ffff);
}

#[test]
fn reference_quirk_pio_wait_after_a_sm_instr_advances_the_program_counter_on_release() {
    // An instruction executed through SMx_INSTR does not move PC (datasheet
    // 3.7). If it stalls and is later released by the FIFO, rp2040js's
    // `checkWait` still calls `nextPC`: PC moves by one.
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    // The assembler's second PULL argument lands in bit 5, which the
    // datasheet calls Block (and the reference treats so): `true` blocks.
    wr(&mut m, SM0_INSTR, pull(false, true, 0)); // empty TX FIFO, blocking
    assert_eq!(rd(&mut m, SM0_ADDR), 0);
    assert_ne!(rd(&mut m, SM0_EXECCTRL) & EXECCTRL_EXEC_STALLED, 0);
    wr(&mut m, TXF0, 1);
    assert_eq!(rd(&mut m, SM0_ADDR), 1);
}

#[test]
fn pio_pin_values_changed_masks_to_thirty_pins_and_a_count_over_31_means_all() {
    // The pin registers are 30 bits (GPIO0-29): VALID_PINS_MASK. An OUT
    // count above 31 writes the whole word from OUT_BASE up, as one wide
    // mask, so OUT_BASE = 4 shifts the word left by four.
    let mut m = Rp2040::new();
    reset_state_machines(&mut m);
    wr(&mut m, SM0_PINCTRL, (32 << OUT_COUNT_SHIFT) | 4);
    wr(&mut m, SM0_INSTR, mov(MOV_DEST_X, OP_INVERT, SRC_NULL, 0));
    wr(&mut m, SM0_INSTR, mov(DEST_PINS, OP_NONE, SRC_X, 0));
    assert_eq!(rd(&mut m, DBG_PADOUT), 0xffff_fff0 & VALID_PINS_MASK);
}

#[test]
fn pio1_is_a_separate_block_with_its_own_irq_lines_and_fifos() {
    let mut m = Rp2040::new();
    wr(&mut m, PIO1 + 0x10, 0x55); // TXF0 of PIO1
    assert_eq!(rd(&mut m, PIO1 + 0x0c), 1);
    assert_eq!(rd(&mut m, FLEVEL), 0);
    wr(&mut m, PIO1 + 0x138, 0x10); // IRQ1_INTE: SM0 TXNFULL
    assert_ne!(m.cpu.pending_interrupts & bit(10), 0, "PIO1 IRQ1 is NVIC 10");
    assert_eq!(m.cpu.pending_interrupts & bit(8), 0);
}
