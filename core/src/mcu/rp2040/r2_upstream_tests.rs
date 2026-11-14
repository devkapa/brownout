//! rp2040js 1.3.3's own tests for the R2 blocks, ported by hand (MIT; see
//! NOTICE), tag v1.3.3 (commit 7701ee0). Upstream has tests for two things
//! R2 ports:
//!
//! - `src/peripherals/timer.spec.ts` (the four `RPTimer` alarm tests), and
//! - three tests of `src/rp2040.spec.ts` ("IO Register Writes": byte
//!   replication, halfword replication, atomic aliases), which R1 ported
//!   against the halting IO_BANK0 stub and which now run against a block
//!   whose register can be read back.
//!
//! Upstream has no spec for clocks, xosc, rtc, watchdog, reset, psm, tbman,
//! syscfg, sysinfo, busctrl, pads or io, nor for `Timer32` (the SysTick and
//! watchdog arithmetic, whose behaviour the lock-step harness holds instead).
//! Its `MockClock.advance(micros)` is `Rp2040::clock_tick(micros * 1000)`.

use super::*;

const ALARM1: u32 = 0x4005_4014;
const ALARM2: u32 = 0x4005_4018;
const ALARM3: u32 = 0x4005_401c;
const ARMED: u32 = 0x4005_4020;
const INTR: u32 = 0x4005_4034;
const INTR_CLEAR: u32 = INTR | 0x3000;
const INTE: u32 = 0x4005_4038;
const INTF: u32 = 0x4005_403c;
const INTS: u32 = 0x4005_4040;

fn advance(m: &mut Rp2040, micros: f64) {
    m.clock_tick(micros * 1000.0);
}

// ---- src/peripherals/timer.spec.ts, describe('RPTimer') / 'Alarms' ----------

#[test]
fn should_set_alarm_1_to_armed_when_writing_to_alarm1_register() {
    let mut m = Rp2040::new();
    m.write_u32(ALARM1, 0x1000);
    assert_eq!(m.read_u32(ARMED), 0x2);
}

#[test]
fn should_disarm_alarm_2_when_writing_0x4_to_the_armed_register() {
    let mut m = Rp2040::new();
    m.write_u32(ALARM2, 0x1000);
    assert_eq!(m.read_u32(ARMED), 0x4);
    // Upstream writes 0xff; the register is write-1-to-clear.
    m.write_u32(ARMED, 0xff);
    assert_eq!(m.read_u32(ARMED), 0);
}

#[test]
fn should_generate_an_irq_3_interrupt_when_alarm_3_fires() {
    let mut m = Rp2040::new();
    // Arm the alarm
    m.write_u32(ALARM3, 1000);
    assert_eq!(m.read_u32(ARMED), 0x8);
    assert_eq!(m.read_u32(INTR), 0);
    // Advance time so that the alarm will fire
    advance(&mut m, 2000.0);
    assert_eq!(m.read_u32(ARMED), 0);
    assert_eq!(m.read_u32(INTR), 0x8);
    assert_eq!(m.read_u32(INTS), 0);
    assert_eq!(m.cpu.pending_interrupts, 0);
    // Enable the interrupts for all alarms
    m.write_u32(INTE, 0xff);
    assert_eq!(m.read_u32(INTS), 0x8);
    assert_eq!(m.cpu.pending_interrupts, 0x8);
    assert!(m.cpu.interrupts_updated);
    // Clear the alarm's interrupt
    m.write_u32(INTR_CLEAR, 0x8);
    assert_eq!(m.read_u32(INTS), 0);
    assert_eq!(m.cpu.pending_interrupts, 0);
}

#[test]
fn should_generate_an_interrupt_if_intf_is_1_even_when_the_inte_bit_is_0() {
    let mut m = Rp2040::new();
    assert_eq!(m.read_u32(INTS), 0);
    assert_eq!(m.read_u32(INTE), 0);
    m.write_u32(INTF, 0x4);
    // The corresponding interrupt bit should be 1
    assert_eq!(m.read_u32(INTS), 0x4);
}

// ---- src/rp2040.spec.ts, describe('IO Register Writes') ------------------------
//
// Upstream plants a spy peripheral at table key 0x10 (address 0x11120) whose
// read returns 0xff. The port has no spy slot, so the tests use a real
// block's register in its place: RESETS.WDSEL (0x4000c004, 25 bits wide),
// which a read returns as written.

const RESETS_WDSEL: u32 = 0x4000_c004;

#[test]
fn should_replicate_8_bit_values_four_times() {
    let mut m = Rp2040::new();
    m.write_u8(RESETS_WDSEL, 0x534);
    // 0x34343434 under the register's 25-bit mask.
    assert_eq!(m.read_u32(RESETS_WDSEL), 0x3434_3434 & 0x01ff_ffff);
    m.write_u8(RESETS_WDSEL + 3, 0x01);
    assert_eq!(m.read_u32(RESETS_WDSEL), 0x0101_0101);
}

#[test]
fn should_replicate_16_bit_values_twice() {
    let mut m = Rp2040::new();
    m.write_u16(RESETS_WDSEL, 0x1234_5678);
    assert_eq!(m.read_u32(RESETS_WDSEL), 0x5678_5678 & 0x01ff_ffff);
}

#[test]
fn should_support_atomic_io_register_write_addresses() {
    let mut m = Rp2040::new();
    m.write_u32(RESETS_WDSEL, 0xff);
    // Upstream: the peripheral reads 0xff, the XOR alias writes 0xff ^ 0x0f.
    m.write_u32(RESETS_WDSEL | 0x1000, 0x0f);
    assert_eq!(m.read_u32(RESETS_WDSEL), 0xf0);
}
