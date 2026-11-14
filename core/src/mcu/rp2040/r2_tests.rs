//! Unit tests for R2, the small system blocks. Expected values are derived
//! in each test from the RP2040 datasheet's register descriptions (field
//! positions, reset values, access types), not read back from the port.
//!
//! Where rp2040js 1.3.3 departs from the datasheet, the port departs with
//! it (the lock-step harness requires that), and a `reference_quirk_*` test
//! pins the reference's behaviour and names the datasheet's beside it.

use super::*;

const SYSINFO: u32 = 0x4000_0000;
const SYSCFG: u32 = 0x4000_4000;
const CLOCKS: u32 = 0x4000_8000;
const RESETS: u32 = 0x4000_c000;
const PSM: u32 = 0x4001_0000;
const IO_BANK0: u32 = 0x4001_4000;
const IO_QSPI: u32 = 0x4001_8000;
const PADS_BANK0: u32 = 0x4001_c000;
const PADS_QSPI: u32 = 0x4002_0000;
const XOSC: u32 = 0x4002_4000;
const PLL_SYS: u32 = 0x4002_8000;
const PLL_USB: u32 = 0x4002_c000;
const BUSCTRL: u32 = 0x4003_0000;
const TIMER: u32 = 0x4005_4000;
const WATCHDOG: u32 = 0x4005_8000;
const RTC: u32 = 0x4005_c000;
const ROSC: u32 = 0x4006_0000;
const VREG: u32 = 0x4006_4000;
const TBMAN: u32 = 0x4006_c000;

const XOR: u32 = 0x1000;
const SET: u32 = 0x2000;
const CLR: u32 = 0x3000;

fn rd(m: &mut Rp2040, a: u32) -> u32 {
    m.read_u32(a)
}
fn wr(m: &mut Rp2040, a: u32, v: u32) {
    m.write_u32(a, v)
}
/// Advances the simulation clock, firing alarms on the way (the harness's
/// `clock.tick`), by whole microseconds.
fn us(m: &mut Rp2040, micros: f64) {
    m.clock_tick(micros * 1000.0);
}

// ---- the generic base, the atomic aliases ---------------------------------------

#[test]
fn atomic_alias_fields_are_xor_set_clear_at_plus_1000_2000_3000() {
    // Datasheet 2.1.2: +0x1000 XOR, +0x2000 SET, +0x3000 CLEAR.
    let mut m = Rp2040::new();
    wr(&mut m, PSM + 0x08, 0b1100); // WDSEL
    wr(&mut m, PSM + 0x08 + XOR, 0b0110);
    assert_eq!(rd(&mut m, PSM + 0x08), 0b1010);
    wr(&mut m, PSM + 0x08 + SET, 0b0001);
    assert_eq!(rd(&mut m, PSM + 0x08), 0b1011);
    wr(&mut m, PSM + 0x08 + CLR, 0b1001);
    assert_eq!(rd(&mut m, PSM + 0x08), 0b0010);
    assert!(m.stop.is_none());
}

#[test]
fn an_atomic_write_applies_the_register_mask_after_the_combination() {
    // SET of bits above the register's width does not stick.
    let mut m = Rp2040::new();
    wr(&mut m, PSM + 0x08 + SET, 0xffff_ffff);
    assert_eq!(rd(&mut m, PSM + 0x08), 0x0001_ffff);
    wr(&mut m, PSM + 0x08 + XOR, 0x0000_ffff);
    assert_eq!(rd(&mut m, PSM + 0x08), 0x0001_0000);
}

#[test]
fn byte_and_halfword_writes_reach_the_register_through_the_bus() {
    let mut m = Rp2040::new();
    m.write_u8(SYSCFG, 0xab);
    assert_eq!(rd(&mut m, SYSCFG), 0xabab_abab);
    m.write_u16(SYSCFG + 2, 0x1234);
    assert_eq!(rd(&mut m, SYSCFG), 0x1234_1234);
}

#[test]
fn reads_through_an_atomic_alias_reach_no_register() {
    // The reference hands a read `address & 0x3fff`, so an alias offset
    // matches no register: all ones (the datasheet makes aliases write-only).
    let mut m = Rp2040::new();
    wr(&mut m, PSM + 0x08, 5);
    assert_eq!(rd(&mut m, PSM + 0x08 + XOR), 0xffff_ffff);
    assert_eq!(rd(&mut m, PSM + 0x08 + SET), 0xffff_ffff);
    assert!(m.stop.is_none());
    assert!(m.warnings >= 2);
}

#[test]
fn unmodelled_registers_read_all_ones_and_ignore_writes() {
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, PSM + 0x40), 0xffff_ffff);
    wr(&mut m, PSM + 0x40, 7);
    assert_eq!(rd(&mut m, PSM + 0x40), 0xffff_ffff);
    assert!(m.stop.is_none());
}

#[test]
fn blocks_the_reference_stubs_out_answer_all_ones_and_never_stop() {
    // IO_QSPI, both PLLs, ROSC and the VREG/chip-reset block are
    // `UnimplementedPeripheral`s: boot code that polls a lock bit sees it set.
    let mut m = Rp2040::new();
    for base in [IO_QSPI, PLL_SYS, PLL_USB, ROSC, VREG] {
        for off in [0u32, 4, 0x10, 0x44] {
            assert_eq!(rd(&mut m, base + off), 0xffff_ffff, "{base:#x}+{off:#x}");
            wr(&mut m, base + off, 0x1234_5678);
            assert_eq!(rd(&mut m, base + off), 0xffff_ffff);
        }
    }
    assert!(m.stop.is_none());
    // PLL CS bit 31 is LOCK in the datasheet: the stub's all ones satisfies a lock wait.
    assert_ne!(rd(&mut m, PLL_SYS) & (1 << 31), 0);
}

#[test]
fn every_peripheral_in_the_table_is_ported() {
    // R6 ported the last blocks (UARTs, SPI, I2C, ADC, PWM): no bus address
    // stops the machine by name any more.
    for &(key, name) in super::bus::PERIPHERALS {
        assert!(periph::Block::from_name(name).is_some(), "{name}");
        let mut m = Rp2040::new();
        m.read_u32(key << 12);
        assert_eq!(m.stop, None, "{name}");
    }
}

// ---- SYSINFO, TBMAN, SYSCFG -------------------------------------------------------

#[test]
fn sysinfo_chip_id_fields() {
    // Datasheet 2.20.2: MANUFACTURER[11:0] = 0x927 (Raspberry Pi, JEP-106),
    // PART[27:12] = 0x0002, REVISION[31:28] = 1 for the B0 stepping.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, SYSINFO), (1 << 28) | (2 << 12) | 0x927);
    // PLATFORM: bit 1 ASIC, bit 0 FPGA.
    assert_eq!(rd(&mut m, SYSINFO + 4), 0b10);
    // GITREF_RP2040 is a hash with no datasheet derivation: the reference's
    // value, "verified against the silicon" in its source.
    assert_eq!(rd(&mut m, SYSINFO + 0x40), 0xe0c9_12e8);
    // Read-only.
    wr(&mut m, SYSINFO, 0);
    assert_eq!(rd(&mut m, SYSINFO), (1 << 28) | (2 << 12) | 0x927);
}

#[test]
fn reference_quirk_tbman_platform_reads_one() {
    // Datasheet 2.22.1: PLATFORM bit 1 is ASIC, bit 0 FPGA, so an ASIC reads
    // 0x2. rp2040js returns its ASIC constant, 1, which is the FPGA bit.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, TBMAN), 1);
    wr(&mut m, TBMAN, 0xffff_ffff);
    assert_eq!(rd(&mut m, TBMAN), 1);
}

#[test]
fn syscfg_proc0_nmi_mask_holds_all_32_bits() {
    // Datasheet 2.20.1: PROC0_NMI_MASK, one bit per IRQ, reset 0.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, SYSCFG), 0);
    wr(&mut m, SYSCFG, 0xa5a5_5a5a);
    assert_eq!(rd(&mut m, SYSCFG), 0xa5a5_5a5a);
    wr(&mut m, SYSCFG + SET, 0x0000_0005);
    assert_eq!(rd(&mut m, SYSCFG), 0xa5a5_5a5f);
}

#[test]
fn reference_quirk_syscfg_proc1_nmi_mask_is_not_modelled() {
    // Datasheet: PROC1_NMI_MASK at 0x4; the reference models core 0 only.
    let mut m = Rp2040::new();
    wr(&mut m, SYSCFG + 4, 0x55);
    assert_eq!(rd(&mut m, SYSCFG + 4), 0xffff_ffff);
}

// ---- RESETS, PSM -------------------------------------------------------------------

#[test]
fn resets_registers_are_25_bits_wide() {
    // Datasheet 2.14.3: one bit per peripheral, adc (bit 0) to usbctrl (24).
    let mut m = Rp2040::new();
    wr(&mut m, RESETS, 0xffff_ffff);
    assert_eq!(rd(&mut m, RESETS), (1 << 25) - 1);
    wr(&mut m, RESETS + 4, 0xffff_ffff); // WDSEL
    assert_eq!(rd(&mut m, RESETS + 4), (1 << 25) - 1);
    wr(&mut m, RESETS + CLR, 1 << 24);
    assert_eq!(rd(&mut m, RESETS), (1 << 24) - 1);
}

#[test]
fn reference_quirk_resets_power_on_value_is_zero() {
    // Datasheet: RESET resets to 0x01ffffff (everything held in reset until
    // firmware releases it). rp2040js starts with nothing in reset.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, RESETS), 0);
}

#[test]
fn reference_quirk_reset_done_is_constant() {
    // Datasheet: RESET_DONE bit n is 1 once peripheral n's reset has been
    // released AND completed, so asserting a reset clears it. rp2040js
    // answers all 25 ones whatever RESET holds.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, RESETS + 8), 0x01ff_ffff);
    wr(&mut m, RESETS, 0x01ff_ffff);
    assert_eq!(rd(&mut m, RESETS + 8), 0x01ff_ffff);
    wr(&mut m, RESETS + 8, 0); // RESET_DONE is read-only
    assert_eq!(rd(&mut m, RESETS + 8), 0x01ff_ffff);
}

#[test]
fn psm_registers_are_17_bits_wide() {
    // Datasheet 2.13.3: 17 power-on-state-machine domains, bits 16:0.
    let mut m = Rp2040::new();
    for off in [0u32, 4, 8] {
        wr(&mut m, PSM + off, 0xffff_ffff);
        assert_eq!(rd(&mut m, PSM + off), (1 << 17) - 1, "offset {off:#x}");
    }
}

#[test]
fn psm_done_is_ready_unless_forced_off() {
    // FRCE_OFF holds a domain off (DONE bit 0); FRCE_ON overrides FRCE_OFF.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, PSM + 0x0c), (1 << 17) - 1);
    wr(&mut m, PSM + 4, 0b0110); // FRCE_OFF: domains 1 and 2
    assert_eq!(rd(&mut m, PSM + 0x0c), ((1 << 17) - 1) & !0b0110);
    wr(&mut m, PSM, 0b0010); // FRCE_ON: domain 1
    assert_eq!(rd(&mut m, PSM + 0x0c), ((1 << 17) - 1) & !0b0100);
    wr(&mut m, PSM + 0x0c, 0); // DONE is read-only
    assert_eq!(rd(&mut m, PSM + 0x0c), ((1 << 17) - 1) & !0b0100);
}

// ---- CLOCKS ------------------------------------------------------------------------

/// Datasheet 2.15.7: each generator's CTRL field masks (bit positions).
const AUXSRC_4: u32 = 0b1111 << 5; // gpout
const AUXSRC_3: u32 = 0b111 << 5; // usb, adc, rtc
const KILL: u32 = 1 << 10;
const ENABLE: u32 = 1 << 11;
const DC50: u32 = 1 << 12;
const PHASE: u32 = 0b11 << 16;
const NUDGE: u32 = 1 << 20;

#[test]
fn clocks_ctrl_registers_keep_exactly_the_datasheet_fields() {
    let mut m = Rp2040::new();
    // (CTRL offset, writable field mask)
    let ctrls = [
        (0x00u32, AUXSRC_4 | KILL | ENABLE | DC50 | PHASE | NUDGE), // CLK_GPOUT0
        (0x0c, AUXSRC_4 | KILL | ENABLE | DC50 | PHASE | NUDGE),    // CLK_GPOUT1
        (0x18, AUXSRC_4 | KILL | ENABLE | DC50 | PHASE | NUDGE),    // CLK_GPOUT2
        (0x24, AUXSRC_4 | KILL | ENABLE | DC50 | PHASE | NUDGE),    // CLK_GPOUT3
        (0x30, 0b11 | (0b11 << 5)),                                 // CLK_REF: SRC[1:0], AUXSRC[6:5]
        (0x3c, 1 | (0b111 << 5)),                                   // CLK_SYS: SRC[0], AUXSRC[7:5]
        (0x48, AUXSRC_3 | KILL | ENABLE),                           // CLK_PERI: AUXSRC, KILL, ENABLE
        (0x54, AUXSRC_3 | KILL | ENABLE | PHASE | NUDGE),           // CLK_USB
        (0x60, AUXSRC_3 | KILL | ENABLE | PHASE | NUDGE),           // CLK_ADC
        (0x6c, AUXSRC_3 | KILL | ENABLE | PHASE | NUDGE),           // CLK_RTC
    ];
    for (off, mask) in ctrls {
        wr(&mut m, CLOCKS + off, 0xffff_ffff);
        assert_eq!(rd(&mut m, CLOCKS + off), mask, "CTRL at {off:#x}");
        wr(&mut m, CLOCKS + off, 0);
        assert_eq!(rd(&mut m, CLOCKS + off), 0);
    }
}

#[test]
fn clocks_div_reset_values_are_one() {
    // Datasheet: DIV resets to INT = 1 (bit 8) for the generators with a
    // fractional divider (gpout, sys, peri, usb).
    let mut m = Rp2040::new();
    for off in [0x04u32, 0x10, 0x1c, 0x28, 0x40, 0x4c, 0x58] {
        assert_eq!(rd(&mut m, CLOCKS + off), 0x100, "DIV at {off:#x}");
    }
    wr(&mut m, CLOCKS + 0x40, 0x0000_0300); // CLK_SYS_DIV: 3.0
    assert_eq!(rd(&mut m, CLOCKS + 0x40), 0x300);
    wr(&mut m, CLOCKS + 0x58, 0x0001_2345);
    assert_eq!(rd(&mut m, CLOCKS + 0x58), 0x0001_2345);
}

#[test]
fn reference_quirk_clocks_ref_adc_rtc_div_read_through_a_0x30_mask() {
    // Datasheet: CLK_REF_DIV is INT[9:8]; ADC and RTC DIV are INT[...] too,
    // all reset to 0x100 and readable back. rp2040js masks the read with
    // 0x30 (bits 5:4), so the reset value reads 0 and bits 9:8 never show.
    let mut m = Rp2040::new();
    for off in [0x34u32, 0x64, 0x70] {
        assert_eq!(rd(&mut m, CLOCKS + off), 0, "reset value of DIV at {off:#x}");
        wr(&mut m, CLOCKS + off, 0x100);
        assert_eq!(rd(&mut m, CLOCKS + off), 0);
        wr(&mut m, CLOCKS + off, 0xffff_ffff);
        assert_eq!(rd(&mut m, CLOCKS + off), 0x30);
    }
}

#[test]
fn clocks_selected_registers_are_one_hot_of_the_source() {
    let mut m = Rp2040::new();
    // CLK_REF_SELECTED: SRC 0 rosc, 1 clksrc_clk_ref_aux, 2 xosc -> bit SRC.
    for src in 0..3u32 {
        wr(&mut m, CLOCKS + 0x30, src);
        assert_eq!(rd(&mut m, CLOCKS + 0x38), 1 << src);
    }
    // CLK_SYS_SELECTED: SRC 0 clk_ref, 1 clksrc_clk_sys_aux.
    for src in 0..2u32 {
        wr(&mut m, CLOCKS + 0x3c, src);
        assert_eq!(rd(&mut m, CLOCKS + 0x44), 1 << src);
    }
    // Generators without a glitchless mux read 1 ("does not have a
    // glitchless mux" in the datasheet): gpout, peri, usb, adc, rtc.
    for off in [0x08u32, 0x14, 0x20, 0x2c, 0x50, 0x5c, 0x68, 0x74] {
        assert_eq!(rd(&mut m, CLOCKS + off), 1, "SELECTED at {off:#x}");
    }
}

#[test]
fn clocks_resus_registers() {
    // Datasheet: CLK_SYS_RESUS_CTRL resets to 0xff (TIMEOUT = 255);
    // the status register reads 0 when no resus has happened.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, CLOCKS + 0x78), 0xff);
    assert_eq!(rd(&mut m, CLOCKS + 0x7c), 0);
    assert!(m.stop.is_none());
}

#[test]
fn reference_quirk_clock_resus_ctrl_is_not_writable() {
    // Datasheet: ENABLE, FRCE, CLEAR and TIMEOUT are read/write. The
    // reference ignores the write (and so never resuscitates a clock).
    let mut m = Rp2040::new();
    wr(&mut m, CLOCKS + 0x78, 0x0001_0000);
    assert_eq!(rd(&mut m, CLOCKS + 0x78), 0xff);
}

#[test]
fn clocks_beyond_the_register_map_are_unmodelled() {
    // FC0 (the frequency counter) at 0x80 and below it, WAKE/SLEEP_EN,
    // ENABLED, INTR... are absent from the reference.
    let mut m = Rp2040::new();
    for off in [0x80u32, 0x98, 0xb4, 0xc4] {
        assert_eq!(rd(&mut m, CLOCKS + off), 0xffff_ffff, "offset {off:#x}");
    }
}

// ---- XOSC --------------------------------------------------------------------------

const XOSC_ENABLE: u32 = 0xfab << 12;
const XOSC_DISABLE: u32 = 0xd1e << 12;
const XOSC_FREQ_1_15MHZ: u32 = 0xaa0;

#[test]
fn xosc_enable_and_disable_follow_the_ctrl_enable_field() {
    // Datasheet 2.16.7: CTRL.ENABLE[23:12]: 0xd1e DISABLE, 0xfab ENABLE;
    // STATUS.ENABLED bit 12, STABLE bit 31.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, XOSC + 4), 0);
    wr(&mut m, XOSC, XOSC_ENABLE | XOSC_FREQ_1_15MHZ);
    assert_eq!(rd(&mut m, XOSC), XOSC_ENABLE | XOSC_FREQ_1_15MHZ);
    assert_eq!(rd(&mut m, XOSC + 4), (1 << 31) | (1 << 12));
    wr(&mut m, XOSC, XOSC_DISABLE | XOSC_FREQ_1_15MHZ);
    assert_eq!(rd(&mut m, XOSC + 4), 0);
}

#[test]
fn xosc_invalid_enable_value_sets_badwrite_until_written_one_to_clear() {
    // STATUS.BADWRITE is bit 24, WC.
    let mut m = Rp2040::new();
    wr(&mut m, XOSC, 0x123 << 12);
    assert_eq!(rd(&mut m, XOSC + 4), 1 << 24);
    wr(&mut m, XOSC + 4, 0); // writing 0 clears nothing
    assert_eq!(rd(&mut m, XOSC + 4), 1 << 24);
    wr(&mut m, XOSC + 4, 1 << 24);
    assert_eq!(rd(&mut m, XOSC + 4), 0);
}

#[test]
fn xosc_dormant_blocks_enabling_until_woken() {
    // DORMANT: write 0x636f6d61 ("coma") to enter, 0x77616b65 ("wake") to leave.
    let mut m = Rp2040::new();
    wr(&mut m, XOSC, XOSC_ENABLE);
    assert_eq!(rd(&mut m, XOSC + 4) & (1 << 31), 1 << 31);
    wr(&mut m, XOSC + 8, 0x636f_6d61);
    assert_eq!(rd(&mut m, XOSC + 8), 0x636f_6d61);
    assert_eq!(rd(&mut m, XOSC + 4) & (1 << 31), 0, "dormant: not stable");
    wr(&mut m, XOSC, XOSC_DISABLE);
    wr(&mut m, XOSC, XOSC_ENABLE);
    assert_eq!(rd(&mut m, XOSC + 4) & (1 << 12), 0, "enabling while dormant does nothing");
    wr(&mut m, XOSC + 8, 0x7761_6b65);
    wr(&mut m, XOSC, XOSC_ENABLE);
    assert_eq!(rd(&mut m, XOSC + 4), (1 << 31) | (1 << 12));
}

#[test]
fn xosc_startup_and_count_widths() {
    // STARTUP: DELAY[13:0], X4 bit 20. COUNT: 8 bits.
    let mut m = Rp2040::new();
    wr(&mut m, XOSC + 0x0c, 0xffff_ffff);
    assert_eq!(rd(&mut m, XOSC + 0x0c), (1 << 20) | 0x3fff);
    wr(&mut m, XOSC + 0x1c, 0xffff_ffff);
    assert_eq!(rd(&mut m, XOSC + 0x1c), 0xff);
}

#[test]
fn reference_quirk_xosc_is_stable_at_once_and_startup_resets_to_zero() {
    // Datasheet: STABLE rises after STARTUP.DELAY x 256 clocks (reset 0xc4);
    // COUNT is a countdown. rp2040js is stable the moment ENABLE is written,
    // resets STARTUP to 0 and never counts COUNT down.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, XOSC + 0x0c), 0);
    wr(&mut m, XOSC + 0x0c, 0xc4);
    wr(&mut m, XOSC + 0x1c, 0x40);
    wr(&mut m, XOSC, XOSC_ENABLE);
    assert_eq!(rd(&mut m, XOSC + 4) & (1 << 31), 1 << 31);
    us(&mut m, 1000.0);
    assert_eq!(rd(&mut m, XOSC + 0x1c), 0x40);
}

// ---- BUSCTRL -----------------------------------------------------------------------

#[test]
fn busctrl_perf_registers() {
    // Datasheet 2.1.5: PERFSEL resets to 0x1f (the disabled event); a write
    // to PERFCTR clears it; PERFSEL is 5 bits; BUS_PRIORITY_ACK reads 1 once
    // a priority change has been applied.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, BUSCTRL + 4), 1);
    for i in 0..4u32 {
        let ctr = BUSCTRL + 0x08 + 8 * i;
        let sel = ctr + 4;
        assert_eq!(rd(&mut m, sel), 0x1f);
        assert_eq!(rd(&mut m, ctr), 0);
        wr(&mut m, sel, 0xffff_ffe3);
        assert_eq!(rd(&mut m, sel), 0x03);
        wr(&mut m, ctr, 0xffff_ffff);
        assert_eq!(rd(&mut m, ctr), 0);
    }
}

#[test]
fn reference_quirk_busctrl_priority_register_is_not_modelled() {
    // Datasheet: BUS_PRIORITY at 0x0 (DMA_R, DMA_W, PROC0, PROC1 bits 0, 4,
    // 8, 12), reset 0. The reference has only the ACK register.
    let mut m = Rp2040::new();
    wr(&mut m, BUSCTRL, 0x1111);
    assert_eq!(rd(&mut m, BUSCTRL), 0xffff_ffff);
}

// ---- TIMER -------------------------------------------------------------------------

#[test]
fn timer_counts_microseconds_of_simulated_time() {
    // Datasheet 4.6.2: TIMELR/TIMERAWL are the low 32 bits of a 64-bit
    // counter that ticks once per microsecond.
    let mut m = Rp2040::new();
    us(&mut m, 1234.0);
    assert_eq!(rd(&mut m, TIMER + 0x28), 1234);
    m.clock_tick(999.0); // 999 ns more: 1234.999 us, still 1234
    assert_eq!(rd(&mut m, TIMER + 0x28), 1234);
    m.clock_tick(1.0);
    assert_eq!(rd(&mut m, TIMER + 0x28), 1235);
    assert_eq!(rd(&mut m, TIMER + 0x0c), 1235);
}

#[test]
fn timer_high_word_is_latched_by_a_read_of_timelr() {
    // Datasheet: reading TIMELR latches the high word into TIMEHR; TIMERAWH
    // is the unlatched high word.
    let mut m = Rp2040::new();
    // 2^32 us = 4294.967296 s; jump the clock near it in one step.
    m.clock.nanos = 4_294_967_295.0 * 1000.0;
    assert_eq!(rd(&mut m, TIMER + 0x24), 0);
    assert_eq!(rd(&mut m, TIMER + 0x0c), 0xffff_ffff);
    assert_eq!(rd(&mut m, TIMER + 0x08), 0);
    m.clock_tick(2000.0); // +2 us: past the 32-bit boundary
    assert_eq!(rd(&mut m, TIMER + 0x24), 1, "TIMERAWH is live");
    assert_eq!(rd(&mut m, TIMER + 0x08), 0, "TIMEHR holds the latch");
    assert_eq!(rd(&mut m, TIMER + 0x28), 1);
    assert_eq!(rd(&mut m, TIMER + 0x0c), 1);
    assert_eq!(rd(&mut m, TIMER + 0x08), 1, "the TIMELR read re-latched it");
}

#[test]
fn timer_alarm_fires_when_the_time_reaches_its_target() {
    // Datasheet: ALARMn arms on write and fires when the low 32 bits of the
    // time equal it, setting INTR bit n and clearing ARMED bit n.
    let mut m = Rp2040::new();
    us(&mut m, 100.0);
    wr(&mut m, TIMER + 0x10, 600); // ALARM0: 500 us from now
    assert_eq!(rd(&mut m, TIMER + 0x10), 600);
    assert_eq!(rd(&mut m, TIMER + 0x20), 1);
    us(&mut m, 499.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 0);
    assert_eq!(rd(&mut m, TIMER + 0x20), 1);
    us(&mut m, 1.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 1);
    assert_eq!(rd(&mut m, TIMER + 0x20), 0);
    // It fires once.
    wr(&mut m, TIMER + 0x34 + CLR, 1);
    us(&mut m, 10_000.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 0);
}

#[test]
fn timer_alarm_at_the_current_time_fires_at_the_next_tick() {
    let mut m = Rp2040::new();
    us(&mut m, 50.0);
    wr(&mut m, TIMER + 0x14, 50);
    assert_eq!(rd(&mut m, TIMER + 0x20), 2);
    m.clock_tick(0.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 2);
}

#[test]
fn timer_alarm_in_the_past_waits_for_the_32_bit_wrap() {
    // The 32-bit compare only matches again after 2^32 us (about 71.6 min).
    let mut m = Rp2040::new();
    us(&mut m, 100.0);
    wr(&mut m, TIMER + 0x18, 99); // ALARM2, one microsecond ago
    us(&mut m, 1_000_000.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 0);
    assert_eq!(rd(&mut m, TIMER + 0x20), 4);
    m.clock_tick((4_294_967_296.0 - 100.0 + 99.0 - 1_000_000.0) * 1000.0 - 1000.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 0);
    m.clock_tick(1000.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 4);
}

#[test]
fn timer_armed_is_write_one_to_clear_and_cancels_the_alarm() {
    let mut m = Rp2040::new();
    wr(&mut m, TIMER + 0x10, 100);
    wr(&mut m, TIMER + 0x1c, 100);
    assert_eq!(rd(&mut m, TIMER + 0x20), 0b1001);
    wr(&mut m, TIMER + 0x20, 0b0001);
    assert_eq!(rd(&mut m, TIMER + 0x20), 0b1000);
    us(&mut m, 200.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 0b1000, "the disarmed alarm 0 never fires");
}

#[test]
fn timer_rearming_an_alarm_replaces_its_pending_target() {
    let mut m = Rp2040::new();
    wr(&mut m, TIMER + 0x10, 100);
    wr(&mut m, TIMER + 0x10, 300);
    us(&mut m, 200.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 0);
    us(&mut m, 100.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 1);
}

#[test]
fn timer_interrupt_registers_follow_the_datasheet() {
    // INTS = (INTR & INTE) | INTF; INTE and INTF are 4 bits; each alarm
    // drives its own NVIC line (TIMER_IRQ_0..3 are IRQs 0..3).
    let mut m = Rp2040::new();
    wr(&mut m, TIMER + 0x38, 0xffff_fff3);
    assert_eq!(rd(&mut m, TIMER + 0x38), 0x3);
    wr(&mut m, TIMER + 0x3c, 0xffff_fff4);
    assert_eq!(rd(&mut m, TIMER + 0x3c), 0x4);
    assert_eq!(rd(&mut m, TIMER + 0x40), 0x4);
    assert_eq!(m.cpu.pending_interrupts, 1 << 2);
    wr(&mut m, TIMER + 0x10, 10);
    wr(&mut m, TIMER + 0x14, 20);
    us(&mut m, 30.0);
    assert_eq!(rd(&mut m, TIMER + 0x34), 0b0011);
    assert_eq!(rd(&mut m, TIMER + 0x40), 0b0111);
    assert_eq!(m.cpu.pending_interrupts, 0b0111);
    wr(&mut m, TIMER + 0x34 + CLR, 0b0001);
    assert_eq!(m.cpu.pending_interrupts, 0b0110);
    wr(&mut m, TIMER + 0x3c, 0);
    assert_eq!(m.cpu.pending_interrupts, 0b0010);
}

#[test]
fn timer_alarm_wakes_a_core_waiting_for_an_interrupt() {
    // The interrupt-driven sleep: WFI, then the alarm's IRQ wakes the core
    // (the step loop fast-forwards the clock to the alarm).
    let mut m = Rp2040::new();
    wr(&mut m, 0xe000_e100, 1 << 1); // NVIC_ISER: enable IRQ 1
    wr(&mut m, TIMER + 0x38, 0b10); // INTE: alarm 1
    wr(&mut m, TIMER + 0x14, 250);
    m.cpu.waiting = true;
    m.step();
    assert_eq!(m.clock.nanos, 250_000.0);
    assert_eq!(m.cpu.pending_interrupts & 2, 2);
}

#[test]
fn reference_quirk_timer_time_cannot_be_set_or_paused() {
    // Datasheet: TIMEHW/TIMELW (0x00, 0x04) set the time; PAUSE (0x30) stops
    // the counter; DBGPAUSE (0x2c) resets to 0x7. The reference has no
    // TIMEHW/TIMELW or DBGPAUSE, and PAUSE only records the bit.
    let mut m = Rp2040::new();
    wr(&mut m, TIMER + 0x04, 5000);
    wr(&mut m, TIMER, 0);
    us(&mut m, 10.0);
    assert_eq!(rd(&mut m, TIMER + 0x28), 10, "TIMELW write ignored");
    assert_eq!(rd(&mut m, TIMER + 0x2c), 0xffff_ffff, "DBGPAUSE unmodelled");
    wr(&mut m, TIMER + 0x30, 1);
    assert_eq!(rd(&mut m, TIMER + 0x30), 1);
    us(&mut m, 10.0);
    assert_eq!(rd(&mut m, TIMER + 0x28), 20, "the paused timer keeps counting");
}

// ---- WATCHDOG ----------------------------------------------------------------------

#[test]
fn watchdog_ctrl_resets_with_all_pause_bits_set() {
    // Datasheet 4.7.5: CTRL resets to 0x07000000: PAUSE_JTAG, PAUSE_DBG0 and
    // PAUSE_DBG1 (bits 24, 25, 26) set, ENABLE (bit 30) clear, TIME 0.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, WATCHDOG), 0x0700_0000);
    wr(&mut m, WATCHDOG, 0);
    assert_eq!(rd(&mut m, WATCHDOG), 0);
    wr(&mut m, WATCHDOG, 0b101 << 24);
    assert_eq!(rd(&mut m, WATCHDOG), 0b101 << 24);
}

#[test]
fn watchdog_scratch_registers_hold_32_bits_each() {
    let mut m = Rp2040::new();
    for i in 0..8u32 {
        wr(&mut m, WATCHDOG + 0x0c + 4 * i, 0xdead_0000 | i);
    }
    for i in 0..8u32 {
        assert_eq!(rd(&mut m, WATCHDOG + 0x0c + 4 * i), 0xdead_0000 | i);
    }
}

#[test]
fn watchdog_counts_down_from_load_and_triggers_at_zero() {
    // LOAD is 24 bits; the timer runs once ENABLE is set. rp2040js models
    // the RP2040-E1 erratum: the counter drops by 2 per 1 us tick.
    let mut m = Rp2040::new();
    wr(&mut m, WATCHDOG + 4, 0xff00_0100); // LOAD: only bits 23:0 count
    assert_eq!(rd(&mut m, WATCHDOG) & 0xff_ffff, 0x100, "LOAD shows at once, even with the timer stopped");
    wr(&mut m, WATCHDOG, (1 << 30) | (0b111 << 24));
    assert_eq!(rd(&mut m, WATCHDOG) & 0xff_ffff, 0x100);
    us(&mut m, 10.0);
    assert_eq!(rd(&mut m, WATCHDOG) & 0xff_ffff, 0x100 - 20);
    assert_eq!(m.periph.watchdog.triggers, 0);
    us(&mut m, 117.0);
    assert_eq!(m.periph.watchdog.triggers, 0, "256 ticks of 0.5 us: 128 us not yet reached");
    us(&mut m, 1.0);
    assert_eq!(m.periph.watchdog.triggers, 1);
    assert_eq!(rd(&mut m, WATCHDOG + 8), 1, "REASON.TIMER is bit 0");
}

#[test]
fn watchdog_trigger_bit_forces_a_reset_and_records_force() {
    // CTRL.TRIGGER (bit 31): REASON.FORCE is bit 1.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, WATCHDOG + 8), 0);
    wr(&mut m, WATCHDOG, 1 << 31);
    assert_eq!(rd(&mut m, WATCHDOG + 8), 2);
    assert_eq!(m.periph.watchdog.triggers, 1);
}

#[test]
fn watchdog_tick_register_gates_the_timer() {
    // TICK.ENABLE is bit 9, RUNNING bit 10; with the tick off, ENABLE does
    // not start the counter.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, WATCHDOG + 0x2c), (1 << 10) | (1 << 9));
    wr(&mut m, WATCHDOG + 0x2c, 0);
    assert_eq!(rd(&mut m, WATCHDOG + 0x2c), 0);
    wr(&mut m, WATCHDOG + 4, 0x1000);
    wr(&mut m, WATCHDOG, 1 << 30);
    assert_eq!(rd(&mut m, WATCHDOG) & (1 << 30), 0, "timer stays stopped");
    us(&mut m, 100.0);
    assert_eq!(m.periph.watchdog.triggers, 0);
    wr(&mut m, WATCHDOG + 0x2c, 1 << 9);
    assert_eq!(rd(&mut m, WATCHDOG) & (1 << 30), 1 << 30, "ENABLE shows once running");
}

#[test]
fn watchdog_load_while_running_restarts_the_count() {
    let mut m = Rp2040::new();
    wr(&mut m, WATCHDOG + 4, 1000);
    wr(&mut m, WATCHDOG, 1 << 30);
    us(&mut m, 100.0);
    wr(&mut m, WATCHDOG + 4, 1000);
    assert_eq!(rd(&mut m, WATCHDOG) & 0xff_ffff, 1000);
    us(&mut m, 499.0);
    assert_eq!(m.periph.watchdog.triggers, 0);
    us(&mut m, 1.0);
    assert_eq!(m.periph.watchdog.triggers, 1);
}

#[test]
fn reference_quirk_watchdog_has_no_reset_effect_and_no_tick_count() {
    // Datasheet: a trigger resets the chip (the PSM blocks selected by
    // PSM_WDSEL and the peripherals by RESETS_WDSEL) and records REASON until
    // the next reset; TICK.CYCLES/COUNT run the 1 us tick. The reference
    // only calls its handler (a warning by default), never clears REASON on
    // its own, and keeps the counter running past zero.
    let mut m = Rp2040::new();
    wr(&mut m, PSM + 8, 0x1ffff);
    wr(&mut m, WATCHDOG + 4, 10);
    wr(&mut m, WATCHDOG, 1 << 30);
    us(&mut m, 100.0);
    assert_eq!(m.periph.watchdog.triggers, 1);
    assert_eq!(rd(&mut m, PSM + 8), 0x1ffff, "PSM WDSEL untouched");
    assert_eq!(rd(&mut m, WATCHDOG + 0x2c) & 0x1ff, 0, "TICK.CYCLES reads 0");
    assert_eq!(rd(&mut m, WATCHDOG + 8), 1);
    // The counter wrapped below zero without stopping.
    assert_ne!(rd(&mut m, WATCHDOG) & 0xff_ffff, 0);
}

#[test]
fn reference_quirk_watchdog_alarm_on_target_would_spin_in_the_reference() {
    // With the counter exactly on zero when the alarm fires, rp2040js
    // computes a zero delay and `SimulationClock.tick` re-fires the alarm at
    // the same instant for ever. The port departs here on purpose: the next
    // trigger is one full 2^32-tick counter period later (2^31 us at 2 ticks
    // per us), which is also when the counter next passes zero.
    let mut m = Rp2040::new();
    wr(&mut m, WATCHDOG + 4, 256);
    wr(&mut m, WATCHDOG, 1 << 30);
    us(&mut m, 128.0);
    assert_eq!(m.periph.watchdog.triggers, 1);
    us(&mut m, 2_147_483_647.0);
    assert_eq!(m.periph.watchdog.triggers, 1);
    us(&mut m, 1.0);
    assert_eq!(m.periph.watchdog.triggers, 2);
}

// ---- RTC ---------------------------------------------------------------------------

const RTC_SETUP0: u32 = RTC + 0x04;
const RTC_SETUP1: u32 = RTC + 0x08;
const RTC_CTRL: u32 = RTC + 0x0c;
const RTC_RTC1: u32 = RTC + 0x18;
const RTC_RTC0: u32 = RTC + 0x1c;

/// Datasheet 4.8.6: RTC_1 = YEAR[23:12], MONTH[11:8], DAY[4:0]; RTC_0 =
/// DOTW[26:24], HOUR[20:16], MIN[13:8], SEC[5:0]. Both SETUP words share
/// those layouts.
fn rtc1(year: u32, month: u32, day: u32) -> u32 {
    (year << 12) | (month << 8) | day
}
fn rtc0(dotw: u32, h: u32, mi: u32, s: u32) -> u32 {
    (dotw << 24) | (h << 16) | (mi << 8) | s
}

fn set_rtc(m: &mut Rp2040, year: u32, month: u32, day: u32, dotw: u32, h: u32, mi: u32, s: u32) {
    wr(m, RTC_SETUP0, rtc1(year, month, day));
    wr(m, RTC_SETUP1, rtc0(dotw, h, mi, s));
    // pico-sdk writes LOAD and ENABLE as two separate stores.
    wr(m, RTC_CTRL, 0x10);
    wr(m, RTC_CTRL, 0x01);
}

#[test]
fn rtc_runs_from_the_reference_epoch_before_it_is_set() {
    // The reference starts at 2021-01-01 00:00:00, which was a Friday (5).
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(2021, 1, 1));
    assert_eq!(rd(&mut m, RTC_RTC0), rtc0(5, 0, 0, 0));
    us(&mut m, 61_000_000.0);
    assert_eq!(rd(&mut m, RTC_RTC0), rtc0(5, 0, 1, 1));
}

#[test]
fn rtc_load_and_enable_set_the_time_and_report_active() {
    // Datasheet: CTRL.ENABLE bit 0, ACTIVE (RO) bit 1, LOAD (SC) bit 4.
    let mut m = Rp2040::new();
    set_rtc(&mut m, 2022, 3, 14, 1, 9, 26, 53); // Pi Day 2022 was a Monday
    assert_eq!(rd(&mut m, RTC_CTRL), 0b11, "ENABLE and ACTIVE, LOAD cleared");
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(2022, 3, 14));
    assert_eq!(rd(&mut m, RTC_RTC0), rtc0(1, 9, 26, 53));
    us(&mut m, 8_000_000.0);
    assert_eq!(rd(&mut m, RTC_RTC0), rtc0(1, 9, 27, 1));
    wr(&mut m, RTC_CTRL, 0);
    assert_eq!(rd(&mut m, RTC_CTRL), 0);
}

#[test]
fn rtc_carries_across_day_month_year_and_leap_boundaries() {
    let mut m = Rp2040::new();
    set_rtc(&mut m, 2023, 12, 31, 0, 23, 59, 50); // a Sunday
    us(&mut m, 20_000_000.0);
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(2024, 1, 1));
    assert_eq!(rd(&mut m, RTC_RTC0), rtc0(1, 0, 0, 10));
    set_rtc(&mut m, 2024, 2, 28, 3, 23, 59, 59); // 2024 is a leap year
    us(&mut m, 1_000_000.0);
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(2024, 2, 29));
    assert_eq!(rd(&mut m, RTC_RTC0), rtc0(4, 0, 0, 0));
    set_rtc(&mut m, 2023, 2, 28, 2, 23, 59, 59); // 2023 is not
    us(&mut m, 1_000_000.0);
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(2023, 3, 1));
    assert_eq!(rd(&mut m, RTC_RTC0), rtc0(3, 0, 0, 0));
    set_rtc(&mut m, 2100, 2, 28, 0, 23, 59, 59); // 2100 is not (century rule)
    us(&mut m, 1_000_000.0);
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(2100, 3, 1));
}

#[test]
fn rtc_setup_registers_read_back_and_load_waits_for_enable() {
    let mut m = Rp2040::new();
    wr(&mut m, RTC_SETUP0, 0x0123_4567);
    wr(&mut m, RTC_SETUP1, 0x89ab_cdef);
    assert_eq!(rd(&mut m, RTC_SETUP0), 0x0123_4567);
    assert_eq!(rd(&mut m, RTC_SETUP1), 0x89ab_cdef);
    // LOAD alone latches nothing; the load happens with the enable.
    wr(&mut m, RTC_CTRL, 0x10);
    assert_eq!(rd(&mut m, RTC_CTRL), 0x10);
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(2021, 1, 1));
    assert_eq!(rd(&mut m, RTC + 0x10), 0, "IRQ_SETUP_0 reads 0");
}

#[test]
fn reference_quirk_rtc_follows_javascript_date_rules() {
    // Out-of-range fields carry as `new Date(y, m, d, h, mi, s)` does (the
    // hardware rejects them), and years 0..99 mean 1900..1999.
    let mut m = Rp2040::new();
    // month 0 is "December of the previous year", day 0 "the day before".
    set_rtc(&mut m, 2022, 0, 1, 0, 0, 0, 0);
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(2021, 12, 1));
    set_rtc(&mut m, 2022, 3, 0, 0, 0, 0, 0);
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(2022, 2, 28));
    // The date's day-of-week comes from the calendar; SETUP's DOTW is unused.
    assert_eq!(rd(&mut m, RTC_RTC0) >> 24, 1);
    set_rtc(&mut m, 5, 6, 7, 0, 0, 0, 0);
    assert_eq!(rd(&mut m, RTC_RTC1), rtc1(1905, 6, 7));
}

#[test]
fn reference_quirk_rtc_has_no_alarm_interrupt() {
    // Datasheet: IRQ_SETUP_0/1 program a match alarm that raises IRQ 25
    // (INTR/INTE/INTF/INTS at 0x28..0x34). The reference reads IRQ_SETUP_0 as
    // 0 and has no interrupt registers.
    let mut m = Rp2040::new();
    wr(&mut m, RTC + 0x10, 0xffff_ffff);
    assert_eq!(rd(&mut m, RTC + 0x10), 0);
    wr(&mut m, RTC + 0x2c, 1);
    assert_eq!(rd(&mut m, RTC + 0x2c), 0xffff_ffff);
}

// ---- IO_BANK0 and the pads --------------------------------------------------------

const GPIO_STATUS: u32 = IO_BANK0; // + 8 * pin
const GPIO_CTRL: u32 = IO_BANK0 + 4; // + 8 * pin
const INTR0: u32 = IO_BANK0 + 0xf0;
const PROC0_INTE0: u32 = IO_BANK0 + 0x100;
const PROC0_INTF0: u32 = IO_BANK0 + 0x110;
const PROC0_INTS0: u32 = IO_BANK0 + 0x120;

const SIO_GPIO_OUT: u32 = 0xd000_0010;
const SIO_GPIO_OE: u32 = 0xd000_0020;
const IRQ_IO_BANK0_BIT: u32 = 1 << 13;

fn pad(pin: u32) -> u32 {
    PADS_BANK0 + 4 + 4 * pin
}

#[test]
fn gpio_ctrl_resets_to_funcsel_null() {
    // Datasheet 2.19.6.1: GPIOn_CTRL.FUNCSEL resets to 0x1f (null), the
    // overrides to 0; STATUS resets to 0 for a floating input.
    let mut m = Rp2040::new();
    for pin in 0..30 {
        assert_eq!(rd(&mut m, GPIO_CTRL + 8 * pin), 0x1f, "GPIO{pin}");
        assert_eq!(rd(&mut m, GPIO_STATUS + 8 * pin), 0, "GPIO{pin}");
    }
}

#[test]
fn gpio_status_reports_sio_output_and_enable() {
    // GPIOn_STATUS bits: OUTFROMPERI 8, OUTTOPAD 9, OEFROMPERI 12, OETOPAD
    // 13, INFROMPAD 17, INTOPERI 19, IRQFROMPAD 24, IRQTOPROC 26.
    let mut m = Rp2040::new();
    wr(&mut m, GPIO_CTRL + 8 * 4, 5); // GPIO4: FUNCSEL = SIO
    wr(&mut m, SIO_GPIO_OE, 1 << 4);
    assert_eq!(rd(&mut m, GPIO_STATUS + 8 * 4), (1 << 12) | (1 << 13));
    wr(&mut m, SIO_GPIO_OUT, 1 << 4);
    assert_eq!(rd(&mut m, GPIO_STATUS + 8 * 4), (1 << 8) | (1 << 9) | (1 << 12) | (1 << 13));
    // Output override 1 inverts what reaches the pad but not OUTFROMPERI.
    wr(&mut m, GPIO_CTRL + 8 * 4, 5 | (1 << 8));
    assert_eq!(rd(&mut m, GPIO_STATUS + 8 * 4), (1 << 8) | (1 << 12) | (1 << 13));
    // Output-enable override 2 forces the pad's OE low.
    wr(&mut m, GPIO_CTRL + 8 * 4, 5 | (2 << 12));
    assert_eq!(rd(&mut m, GPIO_STATUS + 8 * 4), (1 << 8) | (1 << 9) | (1 << 12));
    // A function other than SIO drives nothing.
    wr(&mut m, GPIO_CTRL + 8 * 4, 2);
    assert_eq!(rd(&mut m, GPIO_STATUS + 8 * 4), 0);
}

#[test]
fn gpio_status_is_read_only() {
    let mut m = Rp2040::new();
    wr(&mut m, GPIO_STATUS + 8 * 7, 0xffff_ffff);
    assert_eq!(rd(&mut m, GPIO_STATUS + 8 * 7), 0);
    assert_eq!(rd(&mut m, GPIO_CTRL + 8 * 7), 0x1f);
}

#[test]
fn gpio_ctrl_selects_who_drives_the_pin_and_reaches_the_pin_listeners() {
    // The pin-event stream is what the host sees of a drive change: SIO
    // output enable + FUNCSEL makes the pin a driven output.
    let mut m = Rp2040::new();
    wr(&mut m, GPIO_CTRL + 8 * 9, 5);
    wr(&mut m, SIO_GPIO_OE, 1 << 9);
    wr(&mut m, SIO_GPIO_OUT, 1 << 9);
    let last = *m.pin_events.last().expect("a pin event");
    assert_eq!((last.pin, last.state), (9, sio::PIN_HIGH));
    // Switching the function away from SIO releases it (pulled down by the
    // reset pad value 0x36: PDE bit 2).
    wr(&mut m, GPIO_CTRL + 8 * 9, 0x1f);
    let last = *m.pin_events.last().unwrap();
    assert_eq!((last.pin, last.state), (9, sio::PIN_INPUT_PULLDOWN));
}

#[test]
fn gpio_interrupt_status_layout_and_edge_clearing() {
    // Datasheet 2.19.6.3: four bits per pin, eight pins per register:
    // LEVEL_LOW 0, LEVEL_HIGH 1, EDGE_LOW 2, EDGE_HIGH 3; the edge bits are
    // write-1-to-clear in INTR, the level bits follow the pin.
    let mut m = Rp2040::new();
    wr(&mut m, pad(3), 0x40); // input enable: the pad re-reads its (low) pin
    assert_eq!(rd(&mut m, INTR0), 0b0101 << 12, "the IE write left EDGE_LOW | LEVEL_LOW");
    wr(&mut m, INTR0, 0b1100 << 12);
    assert_eq!(rd(&mut m, INTR0), 0b0001 << 12, "edges cleared, LEVEL_LOW stays");
    m.set_input_value(3, true);
    assert_eq!(rd(&mut m, INTR0), 0b1010 << 12, "GPIO3: EDGE_HIGH | LEVEL_HIGH");
    m.set_input_value(3, false);
    assert_eq!(rd(&mut m, INTR0), 0b1101 << 12, "EDGE_HIGH stays; EDGE_LOW and LEVEL_LOW join");
    wr(&mut m, INTR0, 0b1100 << 12);
    assert_eq!(rd(&mut m, INTR0), 0b0001 << 12, "both edges cleared, LEVEL_LOW stays");
    // Register banks: pins 8..15 live in INTR1, 24..29 in INTR3.
    wr(&mut m, pad(26), 0x40);
    m.set_input_value(26, true);
    assert_eq!(rd(&mut m, INTR0 + 12), 0b1110 << 8, "GPIO26 is nibble 2 of INTR3");
    assert_eq!(rd(&mut m, INTR0 + 4), 0, "INTR1 (pins 8..15) is untouched");
}

#[test]
fn gpio_interrupt_enable_force_and_status_drive_the_io_bank_irq() {
    // PROC0_INTS = (INTR & PROC0_INTE) | PROC0_INTF; IO_IRQ_BANK0 is IRQ 13.
    let mut m = Rp2040::new();
    wr(&mut m, pad(3), 0x40);
    m.set_input_value(3, true);
    assert_eq!(m.cpu.pending_interrupts & IRQ_IO_BANK0_BIT, 0, "nothing enabled yet");
    wr(&mut m, PROC0_INTE0, 0b0010 << 12); // GPIO3 LEVEL_HIGH
    assert_eq!(rd(&mut m, PROC0_INTE0), 0b0010 << 12);
    assert_eq!(rd(&mut m, PROC0_INTS0), 0b0010 << 12);
    assert_eq!(m.cpu.pending_interrupts & IRQ_IO_BANK0_BIT, IRQ_IO_BANK0_BIT);
    m.set_input_value(3, false);
    assert_eq!(rd(&mut m, PROC0_INTS0), 0);
    assert_eq!(m.cpu.pending_interrupts & IRQ_IO_BANK0_BIT, 0);
    wr(&mut m, PROC0_INTF0, 0b0001 << 16); // GPIO4 forced LEVEL_LOW
    assert_eq!(rd(&mut m, PROC0_INTF0), 0b0001 << 16);
    assert_eq!(rd(&mut m, PROC0_INTS0), 0b0001 << 16);
    assert_eq!(m.cpu.pending_interrupts & IRQ_IO_BANK0_BIT, IRQ_IO_BANK0_BIT);
    wr(&mut m, PROC0_INTF0, 0);
    assert_eq!(m.cpu.pending_interrupts & IRQ_IO_BANK0_BIT, 0);
}

#[test]
fn gpio_interrupt_registers_support_the_atomic_aliases() {
    let mut m = Rp2040::new();
    wr(&mut m, PROC0_INTE0 + SET, 0x0000_00f0);
    wr(&mut m, PROC0_INTE0 + SET, 0x0000_000f);
    assert_eq!(rd(&mut m, PROC0_INTE0), 0xff);
    wr(&mut m, PROC0_INTE0 + CLR, 0x0000_0f0f);
    assert_eq!(rd(&mut m, PROC0_INTE0), 0xf0);
    wr(&mut m, PROC0_INTE0 + XOR, 0x0000_00ff);
    assert_eq!(rd(&mut m, PROC0_INTE0), 0x0f);
}

#[test]
fn pads_registers_start_at_0x04_one_word_per_gpio() {
    // Datasheet 2.19.6.2: VOLTAGE_SELECT at 0, GPIO0..GPIO29 at 0x04..0x78.
    let mut m = Rp2040::new();
    wr(&mut m, PADS_BANK0, 0xffff_ffff);
    assert_eq!(rd(&mut m, PADS_BANK0), 1, "VOLTAGE_SELECT is one bit");
    wr(&mut m, pad(0), 0x11);
    wr(&mut m, pad(29), 0x22);
    assert_eq!(rd(&mut m, PADS_BANK0 + 0x04), 0x11);
    assert_eq!(rd(&mut m, PADS_BANK0 + 0x78), 0x22);
    assert_eq!(rd(&mut m, pad(1)), 0x36, "its neighbour keeps the reset value");
}

#[test]
fn reference_quirk_pad_reset_value_and_bank_extent() {
    // Datasheet: GPIOn pad reset is 0x56 (IE set, DRIVE 4 mA, PDE, SCHMITT);
    // SWCLK (0x7c) and SWD (0x80) follow GPIO29. rp2040js resets every pad to
    // 0x36 (IE clear, DRIVE 12 mA) and stops the bank at GPIO29.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, pad(0)), 0x36);
    assert_eq!(rd(&mut m, PADS_BANK0 + 0x7c), 0xffff_ffff);
    wr(&mut m, PADS_BANK0 + 0x80, 0x1);
    assert_eq!(rd(&mut m, PADS_BANK0 + 0x80), 0xffff_ffff);
}

#[test]
fn pad_input_enable_gates_the_input_and_its_interrupt_flags() {
    // Datasheet: IE is pad bit 6. With IE clear the pad reads as low to the
    // peripherals whatever the pin does (STATUS.INTOPERI stays 0) and the
    // level interrupt shows LEVEL_LOW.
    let mut m = Rp2040::new();
    m.set_input_value(5, true);
    assert_eq!(rd(&mut m, GPIO_STATUS + 8 * 5), 1 << 17, "INFROMPAD follows the pin, INTOPERI is gated");
    assert_eq!((rd(&mut m, INTR0) >> 20) & 0xf, 0b0101, "EDGE_LOW | LEVEL_LOW");
    // Setting IE re-evaluates the held input.
    wr(&mut m, pad(5), 0x40);
    assert_eq!(rd(&mut m, GPIO_STATUS + 8 * 5), (1 << 17) | (1 << 19));
    assert_eq!((rd(&mut m, INTR0) >> 20) & 0xf, 0b1110, "level now high; EDGE_LOW kept, EDGE_HIGH new");
    wr(&mut m, pad(5), 0x00);
    assert_eq!(rd(&mut m, GPIO_STATUS + 8 * 5), 1 << 17);
}

#[test]
fn pad_pull_bits_choose_the_released_pin_state() {
    // Datasheet: PUE bit 3, PDE bit 2; both set is the bus keeper.
    let mut m = Rp2040::new();
    wr(&mut m, pad(11), 0b1000);
    assert_eq!(m.pin_events.last().map(|e| (e.pin, e.state)), Some((11, sio::PIN_INPUT_PULLUP)));
    wr(&mut m, pad(11), 0b0100);
    assert_eq!(m.pin_events.last().map(|e| e.state), Some(sio::PIN_INPUT_PULLDOWN));
    wr(&mut m, pad(11), 0b1100);
    assert_eq!(m.pin_events.last().map(|e| e.state), Some(sio::PIN_INPUT_BUSKEEPER));
    wr(&mut m, pad(11), 0b0000); // neither: a floating input
    assert_eq!(m.pin_events.last().map(|e| e.state), Some(sio::PIN_INPUT));
}

#[test]
fn qspi_pads_are_six_registers_with_their_own_voltage_select() {
    // Datasheet: PADS_QSPI VOLTAGE_SELECT at 0, GPIO_QSPI_SCLK, SS, SD0..SD3
    // at 0x04..0x18.
    let mut m = Rp2040::new();
    wr(&mut m, PADS_QSPI, 3);
    assert_eq!(rd(&mut m, PADS_QSPI), 1);
    assert_eq!(rd(&mut m, PADS_BANK0), 0, "the two banks keep separate voltage selects");
    for i in 0..6u32 {
        wr(&mut m, PADS_QSPI + 4 + 4 * i, 0x10 + i);
    }
    for i in 0..6u32 {
        assert_eq!(rd(&mut m, PADS_QSPI + 4 + 4 * i), 0x10 + i);
    }
    assert_eq!(rd(&mut m, PADS_QSPI + 0x1c), 0xffff_ffff);
    assert_eq!(rd(&mut m, PADS_BANK0 + 4), 0x36, "bank 0 is separate");
}

#[test]
fn reference_quirk_io_qspi_is_a_stub() {
    // Datasheet: IO_QSPI has GPIO_QSPI_SCLK..SD3 STATUS/CTRL pairs and an
    // interrupt block. The reference plugs in an unimplemented peripheral.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, IO_QSPI), 0xffff_ffff);
    wr(&mut m, IO_QSPI + 4, 0);
    assert_eq!(rd(&mut m, IO_QSPI + 4), 0xffff_ffff);
}
