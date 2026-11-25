//! The generic peripheral base and the small system blocks every boot
//! touches before any real I/O, ported from rp2040js 1.3.3 (MIT):
//! `src/peripherals/{peripheral,clocks,xosc,reset,psm,tbman,syscfg,
//! sysinfo,busctrl}.ts`, plus the `UnimplementedPeripheral` stubs
//! `src/rp2040.ts` plugs in for the IO_QSPI bank, both PLLs, ROSC and the
//! VREG block.
//!
//! The reference's `BasePeripheral` answers an unmodelled register with
//! 0xffffffff (and a logged warning) and ignores an unmodelled write, and
//! routes the atomic aliases (+0x1000 XOR, +0x2000 SET, +0x3000 CLEAR)
//! through the block's own read and write: `writeUint32Atomic` reads the
//! register (side effects included), combines it with the written value and
//! writes the result. The port does the same, in [`Rp2040::periph_write`].
//! Reads are handed `address & 0x3fff`, so a read through an alias offset
//! reaches no register and answers 0xffffffff.
//!
//! The timer, watchdog, RTC and the IO and pad banks have their own files.

use super::Rp2040;

/// Every block the R2 package ports. Anything else in the reference's
/// peripheral table still halts the machine (`Stop::Peripheral`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Block {
    SysInfo,
    SysCfg,
    Clocks,
    Resets,
    Psm,
    IoBank0,
    IoQspi,
    PadsBank0,
    PadsQspi,
    Xosc,
    PllSys,
    PllUsb,
    BusCtrl,
    Timer,
    Watchdog,
    Rtc,
    Rosc,
    Vreg,
    Tbman,
    Ssi,
    Usb,
    Dma,
    Pio0,
    Pio1,
    Pwm,
    Adc,
    Uart0,
    Uart1,
    Spi0,
    Spi1,
    I2c0,
    I2c1,
}

impl Block {
    /// The block behind a peripheral-table name, when R2 ports it.
    pub fn from_name(name: &str) -> Option<Block> {
        Some(match name {
            "SYSINFO_BASE" => Block::SysInfo,
            "SYSCFG" => Block::SysCfg,
            "CLOCKS_BASE" => Block::Clocks,
            "RESETS_BASE" => Block::Resets,
            "PSM_BASE" => Block::Psm,
            "IO_BANK0_BASE" => Block::IoBank0,
            "IO_QSPI_BASE" => Block::IoQspi,
            "PADS_BANK0_BASE" => Block::PadsBank0,
            "PADS_QSPI_BASE" => Block::PadsQspi,
            "XOSC_BASE" => Block::Xosc,
            "PLL_SYS_BASE" => Block::PllSys,
            "PLL_USB_BASE" => Block::PllUsb,
            "BUSCTRL_BASE" => Block::BusCtrl,
            "TIMER_BASE" => Block::Timer,
            "WATCHDOG_BASE" => Block::Watchdog,
            "RTC_BASE" => Block::Rtc,
            "ROSC_BASE" => Block::Rosc,
            "VREG_AND_CHIP_RESET_BASE" => Block::Vreg,
            "TBMAN_BASE" => Block::Tbman,
            "SSI" => Block::Ssi,
            "USB" => Block::Usb,
            "DMA" => Block::Dma,
            "PIO0" => Block::Pio0,
            "PIO1" => Block::Pio1,
            "PWM_BASE" => Block::Pwm,
            "ADC" => Block::Adc,
            "UART0" => Block::Uart0,
            "UART1" => Block::Uart1,
            "SPI0" => Block::Spi0,
            "SPI1" => Block::Spi1,
            "I2C0" => Block::I2c0,
            "I2C1" => Block::I2c1,
            _ => return None,
        })
    }
}

pub const ATOMIC_NORMAL: u32 = 0;
pub const ATOMIC_XOR: u32 = 1;
pub const ATOMIC_SET: u32 = 2;
pub const ATOMIC_CLEAR: u32 = 3;

/// `atomicUpdate`. The reference's `default` arm (a type outside 1..3 that
/// is not NORMAL) cannot be reached through the bus, whose alias field is
/// two bits wide.
pub fn atomic_update(current: u32, atomic_type: u32, new_value: u32) -> u32 {
    match atomic_type {
        ATOMIC_XOR => current ^ new_value,
        ATOMIC_SET => current | new_value,
        ATOMIC_CLEAR => current & !new_value,
        _ => new_value,
    }
}

// ---- CLOCKS ---------------------------------------------------------------

const CLK_SLICES: usize = 10;
/// Slice order by `offset / 12`: gpout0..3, ref, sys, peri, usb, adc, rtc.
const SLICE_REF: usize = 4;
const SLICE_SYS: usize = 5;
const SLICE_PERI: usize = 6;
const SLICE_USB: usize = 7;
const SLICE_ADC: usize = 8;
const SLICE_RTC: usize = 9;
const CLK_SYS_RESUS_CTRL: u32 = 0x78;
const CLK_SYS_RESUS_STATUS: u32 = 0x7c;

pub struct Clocks {
    pub ctrl: [u32; CLK_SLICES],
    pub div: [u32; CLK_SLICES],
}

impl Clocks {
    fn new() -> Self {
        Clocks { ctrl: [0; CLK_SLICES], div: [0x100; CLK_SLICES] }
    }

    fn read(&self, offset: u32) -> Option<u32> {
        if offset == CLK_SYS_RESUS_CTRL {
            return Some(0xff);
        }
        if offset == CLK_SYS_RESUS_STATUS {
            return Some(0);
        }
        if offset >= CLK_SYS_RESUS_CTRL || offset & 3 != 0 {
            return None;
        }
        let slice = (offset / 12) as usize;
        Some(match (offset % 12) / 4 {
            0 => {
                let mask = match slice {
                    SLICE_REF => 0b000001100011,
                    SLICE_SYS => 0b000011100001,
                    SLICE_PERI => 0b110011100000,
                    SLICE_USB | SLICE_ADC | SLICE_RTC => 0b100110000110011100000,
                    _ => 0b100110001110111100000,
                };
                self.ctrl[slice] & mask
            }
            1 => match slice {
                // b8..9 = int divisor. no frac divisor present (the
                // reference's comment, and its mask: bits 5:4)
                SLICE_REF | SLICE_ADC | SLICE_RTC => self.div[slice] & 0x30,
                _ => self.div[slice],
            },
            _ => match slice {
                SLICE_REF => 1 << (self.ctrl[SLICE_REF] & 0x03),
                SLICE_SYS => 1 << (self.ctrl[SLICE_SYS] & 0x01),
                _ => 1,
            },
        })
    }

    /// `Some(true)` handled, `Some(false)` handled and silent (resus
    /// control), `None` falls to the base's warning.
    fn write(&mut self, offset: u32, value: u32) -> bool {
        if offset == CLK_SYS_RESUS_CTRL {
            return true; // clock resus not implemented
        }
        if offset >= CLK_SYS_RESUS_CTRL || offset & 3 != 0 {
            return false;
        }
        let slice = (offset / 12) as usize;
        match (offset % 12) / 4 {
            0 => self.ctrl[slice] = value,
            1 => self.div[slice] = value,
            _ => return false,
        }
        true
    }
}

// ---- XOSC -----------------------------------------------------------------

const XOSC_CTRL: u32 = 0x00;
const XOSC_STATUS: u32 = 0x04;
const XOSC_DORMANT: u32 = 0x08;
const XOSC_STARTUP: u32 = 0x0c;
const XOSC_COUNT: u32 = 0x1c;
const XOSC_ENABLE_BITS: u32 = 0x00ff_f000;
const XOSC_ENABLE_LSB: u32 = 12;
const XOSC_ENABLE_DISABLE: u32 = 0xd1e;
const XOSC_ENABLE_ENABLE: u32 = 0xfab;
const XOSC_STATUS_STABLE: u32 = 0x8000_0000;
const XOSC_STATUS_BADWRITE: u32 = 0x0100_0000;
const XOSC_STATUS_ENABLED: u32 = 0x0000_1000;
const XOSC_DORMANT_VALUE: u32 = 0x636f_6d61;
const XOSC_WAKE_VALUE: u32 = 0x7761_6b65;
const XOSC_STARTUP_MASK: u32 = 0x0010_0000 | 0x0000_3fff;

#[derive(Default)]
pub struct Xosc {
    pub ctrl: u32,
    pub status: u32,
    pub dormant: u32,
    pub startup: u32,
    pub count: u32,
    pub enabled: bool,
    pub stable: bool,
    pub is_dormant: bool,
}

impl Xosc {
    pub fn status_reg(&self) -> u32 {
        let mut status = self.status;
        if self.stable {
            status |= XOSC_STATUS_STABLE;
        }
        if self.enabled {
            status |= XOSC_STATUS_ENABLED;
        }
        status
    }
}

// ---- RESETS / PSM / BUSCTRL -------------------------------------------------

pub struct Resets {
    pub reset: u32,
    pub wdsel: u32,
    pub reset_done: u32,
}

const PSM_BITS_MASK: u32 = 0x0001_ffff;

#[derive(Default)]
pub struct Psm {
    pub frce_on: u32,
    pub frce_off: u32,
    pub wdsel: u32,
}

pub struct BusCtrl {
    pub perf_ctr: [u32; 4],
    pub perf_sel: [u32; 4],
}

/// The R2 blocks' state that has no file of its own.
pub struct Periph {
    pub clocks: Clocks,
    pub xosc: Xosc,
    pub resets: Resets,
    pub psm: Psm,
    pub busctrl: BusCtrl,
    /// `core.interruptNMIMask`: only SYSCFG reads or writes it.
    pub nmi_mask: u32,
    /// The reference's `warn()` calls that do not stop it: unmodelled
    /// registers, unimplemented stubs. Not part of the compared state.
    pub unimplemented_accesses: u64,
    /// Whether the word the current write carries is a signed JavaScript
    /// number in the reference: the replicated value of a byte or halfword
    /// store and the result of an atomic alias are built with `|`, `<<`,
    /// `^` or `&`, which yield int32, while a plain word store hands over the
    /// unsigned register. Only the timer's alarm arithmetic can tell.
    pub write_signed: bool,
    pub timer: super::timer::Timer,
    pub watchdog: super::timer::Watchdog,
    pub rtc: super::rtc::Rtc,
    pub io: super::gpiobank::GpioBank,
    pub ssi: super::ssi::Ssi,
    pub usb: super::usb::Usb,
    pub cdc: super::usb::Cdc,
    pub dma: super::dma::Dma,
    pub pio: [super::pio::Pio; 2],
    pub pwm: super::pwm::Pwm,
    pub adc: super::adc::Adc,
    pub uart: [super::uart::Uart; 2],
    /// Bytes sent and baud changes, for the host to drain (like `pin_events`).
    pub uart_events: Vec<super::uart::UartEvent>,
    pub spi: [super::spi::Spi; 2],
    /// Words transmitted by the SPI blocks, for the host to drain.
    pub spi_events: Vec<super::spi::SpiEvent>,
    pub i2c: [super::i2c::I2c; 2],
    /// Host callbacks the I2C blocks made, for the host to drain.
    pub i2c_events: Vec<super::i2c::I2cEvent>,
    /// The pending `setTimeout(() => run(), 0)` callbacks of the PIO blocks,
    /// oldest first, each the index of the block that armed it.
    pub pio_timers: Vec<u8>,
}

impl Default for Periph {
    fn default() -> Self {
        Self::new()
    }
}

impl Periph {
    pub fn new() -> Self {
        Periph {
            clocks: Clocks::new(),
            xosc: Xosc::default(),
            resets: Resets { reset: 0, wdsel: 0, reset_done: 0x1ff_ffff },
            psm: Psm::default(),
            busctrl: BusCtrl { perf_ctr: [0; 4], perf_sel: [0x1f; 4] },
            nmi_mask: 0,
            unimplemented_accesses: 0,
            write_signed: false,
            timer: super::timer::Timer::new(),
            watchdog: super::timer::Watchdog::new(),
            rtc: super::rtc::Rtc::new(),
            io: super::gpiobank::GpioBank::new(),
            ssi: super::ssi::Ssi::default(),
            usb: super::usb::Usb::new(),
            cdc: super::usb::Cdc::new(),
            dma: {
                let mut d = super::dma::Dma::new();
                // Each PIO state machine's constructor raises its TX line (the
                // FIFO is not full) and lowers its RX line (it is empty).
                for p in 0..2u32 {
                    for m in 0..4u32 {
                        d.dreq[(super::dma::dreq::PIO0_TX0 + p * 8 + m) as usize] = true;
                    }
                }
                // The SPI blocks' constructors do the same for their TX FIFOs
                // (R6 ports the blocks; the lines are already up).
                d.dreq[super::dma::dreq::SPI0_TX as usize] = true;
                d.dreq[super::dma::dreq::SPI1_TX as usize] = true;
                d
            },
            pio: [super::pio::Pio::new(), super::pio::Pio::new()],
            pwm: super::pwm::Pwm::new(),
            adc: super::adc::Adc::new(),
            uart: [super::uart::Uart::new(), super::uart::Uart::new()],
            uart_events: Vec::new(),
            spi: [super::spi::Spi::new(), super::spi::Spi::new()],
            spi_events: Vec::new(),
            i2c: [super::i2c::I2c::new(), super::i2c::I2c::new()],
            i2c_events: Vec::new(),
            pio_timers: Vec::new(),
        }
    }
}

const BASE_UNIMPLEMENTED: u32 = 0xffff_ffff;

impl Rp2040 {
    /// `BasePeripheral.readUint32`: a warning, and all ones.
    fn base_read(&mut self) -> u32 {
        self.periph.unimplemented_accesses += 1;
        self.warnings += 1;
        BASE_UNIMPLEMENTED
    }

    /// `BasePeripheral.writeUint32`: a warning.
    fn base_write(&mut self) {
        self.periph.unimplemented_accesses += 1;
        self.warnings += 1;
    }

    /// `peripheral.readUint32(address & 0x3fff)`.
    pub(crate) fn periph_read(&mut self, block: Block, offset: u32) -> u32 {
        match block {
            Block::SysInfo => match offset {
                0x0 => 0x1000_2927,
                0x4 => 0x0000_0002,
                0x40 => 0xe0c9_12e8,
                _ => self.base_read(),
            },
            Block::SysCfg => match offset {
                0 => self.periph.nmi_mask,
                _ => self.base_read(),
            },
            Block::Clocks => match self.periph.clocks.read(offset) {
                Some(v) => v,
                None => self.base_read(),
            },
            Block::Resets => match offset {
                0x0 => self.periph.resets.reset,
                0x4 => self.periph.resets.wdsel,
                0x8 => self.periph.resets.reset_done,
                _ => self.base_read(),
            },
            Block::Psm => {
                let p = &self.periph.psm;
                match offset {
                    0x00 => p.frce_on,
                    0x04 => p.frce_off,
                    0x08 => p.wdsel,
                    // Domains are ready unless forced off (FRCE_ON
                    // overrides FRCE_OFF)
                    0x0c => (PSM_BITS_MASK & !p.frce_off) | (p.frce_on & p.frce_off),
                    _ => self.base_read(),
                }
            }
            Block::Tbman => match offset {
                0 => 1,
                _ => self.base_read(),
            },
            Block::Xosc => {
                let x = &self.periph.xosc;
                match offset {
                    XOSC_CTRL => x.ctrl,
                    XOSC_STATUS => x.status_reg(),
                    XOSC_DORMANT => x.dormant,
                    XOSC_STARTUP => x.startup,
                    XOSC_COUNT => x.count,
                    _ => self.base_read(),
                }
            }
            Block::BusCtrl => {
                let b = &self.periph.busctrl;
                match offset {
                    0x004 => 1,
                    0x008 => b.perf_ctr[0],
                    0x00c => b.perf_sel[0],
                    0x010 => b.perf_ctr[1],
                    0x014 => b.perf_sel[1],
                    0x018 => b.perf_ctr[2],
                    0x01c => b.perf_sel[2],
                    0x020 => b.perf_ctr[3],
                    0x024 => b.perf_sel[3],
                    _ => self.base_read(),
                }
            }
            Block::Ssi => self.ssi_read(offset),
            Block::Usb => self.usb_read(offset),
            Block::Dma => self.dma_read(offset),
            Block::Pio0 => self.pio_read(0, offset),
            Block::Pio1 => self.pio_read(1, offset),
            Block::I2c0 => match self.i2c_read(0, offset) {
                Some(v) => v,
                None => self.base_read(),
            },
            Block::I2c1 => match self.i2c_read(1, offset) {
                Some(v) => v,
                None => self.base_read(),
            },
            Block::Spi0 => match self.spi_read(0, offset) {
                Some(v) => v,
                None => self.base_read(),
            },
            Block::Spi1 => match self.spi_read(1, offset) {
                Some(v) => v,
                None => self.base_read(),
            },
            Block::Uart0 => match self.uart_read(0, offset) {
                Some(v) => v,
                None => self.base_read(),
            },
            Block::Uart1 => match self.uart_read(1, offset) {
                Some(v) => v,
                None => self.base_read(),
            },
            Block::Adc => match self.adc_read(offset) {
                Some(v) => v,
                None => self.base_read(),
            },
            Block::Pwm => match self.pwm_read(offset) {
                Some(v) => v,
                None => self.base_read(),
            },
            Block::Timer => self.timer_read(offset),
            Block::Watchdog => self.watchdog_read(offset),
            Block::Rtc => self.rtc_read(offset),
            Block::IoBank0 => self.io_read(offset),
            Block::PadsBank0 => self.pads_read(false, offset),
            Block::PadsQspi => self.pads_read(true, offset),
            Block::IoQspi | Block::PllSys | Block::PllUsb | Block::Rosc | Block::Vreg => self.base_read(),
        }
    }

    /// `peripheral.writeUint32Atomic(address & 0xfff, value, alias)`.
    pub(crate) fn periph_write(&mut self, block: Block, offset: u32, atomic: u32, value: u32, sub_word: bool) {
        self.periph.write_signed = sub_word || atomic != ATOMIC_NORMAL;
        let new_value = if atomic != ATOMIC_NORMAL {
            let current = self.periph_read(block, offset);
            atomic_update(current, atomic, value)
        } else {
            value
        };
        // `rawWriteValue` is the value as the bus delivered it.
        self.periph_write_raw(block, offset, new_value, value);
    }

    fn periph_write_raw(&mut self, block: Block, offset: u32, value: u32, raw: u32) {
        match block {
            Block::SysInfo | Block::Tbman | Block::IoQspi | Block::PllSys | Block::PllUsb | Block::Rosc | Block::Vreg => {
                self.base_write()
            }
            Block::SysCfg => match offset {
                0 => self.periph.nmi_mask = value,
                _ => self.base_write(),
            },
            Block::Clocks => {
                if !self.periph.clocks.write(offset, value) {
                    self.base_write();
                }
            }
            Block::Resets => match offset {
                0x0 => self.periph.resets.reset = value & 0x1ff_ffff,
                0x4 => self.periph.resets.wdsel = value & 0x1ff_ffff,
                _ => self.base_write(),
            },
            Block::Psm => {
                let p = &mut self.periph.psm;
                match offset {
                    0x00 => p.frce_on = value & PSM_BITS_MASK,
                    0x04 => p.frce_off = value & PSM_BITS_MASK,
                    0x08 => p.wdsel = value & PSM_BITS_MASK,
                    _ => self.base_write(),
                }
            }
            Block::Xosc => self.xosc_write(offset, value),
            Block::BusCtrl => {
                let b = &mut self.periph.busctrl;
                match offset {
                    0x008 => b.perf_ctr[0] = 0,
                    0x00c => b.perf_sel[0] = value & 0x1f,
                    0x010 => b.perf_ctr[1] = 0,
                    0x014 => b.perf_sel[1] = value & 0x1f,
                    0x018 => b.perf_ctr[2] = 0,
                    0x01c => b.perf_sel[2] = value & 0x1f,
                    0x020 => b.perf_ctr[3] = 0,
                    0x024 => b.perf_sel[3] = value & 0x1f,
                    _ => self.base_write(),
                }
            }
            Block::Ssi => self.ssi_write(offset, value),
            Block::Usb => self.usb_write(offset, value, raw),
            Block::Dma => self.dma_write(offset, value, raw),
            Block::Pio0 => self.pio_write(0, offset, value, raw),
            Block::Pio1 => self.pio_write(1, offset, value, raw),
            Block::I2c0 | Block::I2c1 => {
                let i = (block == Block::I2c1) as usize;
                if !self.i2c_write(i, offset, value) {
                    self.base_write();
                }
            }
            Block::Spi0 | Block::Spi1 => {
                let i = (block == Block::Spi1) as usize;
                if !self.spi_write(i, offset, value) {
                    self.base_write();
                }
            }
            Block::Uart0 | Block::Uart1 => {
                let i = (block == Block::Uart1) as usize;
                if !self.uart_write(i, offset, value, raw) {
                    self.base_write();
                }
            }
            Block::Adc => {
                if !self.adc_write(offset, value) {
                    self.base_write();
                }
            }
            Block::Pwm => {
                if !self.pwm_write(offset, value) {
                    self.base_write();
                }
            }
            Block::Timer => self.timer_write(offset, value, raw),
            Block::Watchdog => self.watchdog_write(offset, value),
            Block::Rtc => self.rtc_write(offset, value),
            Block::IoBank0 => self.io_write(offset, value, raw),
            Block::PadsBank0 => self.pads_write(false, offset, value),
            Block::PadsQspi => self.pads_write(true, offset, value),
        }
    }

    fn xosc_write(&mut self, offset: u32, value: u32) {
        match offset {
            XOSC_CTRL => {
                let x = &mut self.periph.xosc;
                x.ctrl = value;
                let enable_value = (value & XOSC_ENABLE_BITS) >> XOSC_ENABLE_LSB;
                if enable_value == XOSC_ENABLE_ENABLE {
                    if !x.is_dormant {
                        x.enabled = true;
                        // For simplicity, become stable immediately
                        x.stable = true;
                    }
                } else if enable_value == XOSC_ENABLE_DISABLE {
                    x.enabled = false;
                    x.stable = false;
                } else if enable_value != 0 {
                    // Invalid write to ENABLE field
                    x.status |= XOSC_STATUS_BADWRITE;
                    self.warnings += 1;
                }
            }
            XOSC_STATUS => {
                // Clear BADWRITE bit if written as 1 (write-1-to-clear)
                if value & XOSC_STATUS_BADWRITE != 0 {
                    self.periph.xosc.status &= !XOSC_STATUS_BADWRITE;
                }
            }
            XOSC_DORMANT => {
                let x = &mut self.periph.xosc;
                if value == XOSC_DORMANT_VALUE {
                    x.is_dormant = true;
                    x.stable = false;
                } else if value == XOSC_WAKE_VALUE {
                    x.is_dormant = false;
                    if x.enabled {
                        x.stable = true;
                    }
                }
                x.dormant = value;
            }
            XOSC_STARTUP => self.periph.xosc.startup = value & XOSC_STARTUP_MASK,
            // Writing to COUNT starts the countdown, which the reference
            // does not implement.
            XOSC_COUNT => self.periph.xosc.count = value & 0xff,
            _ => self.base_write(),
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Block {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Block::SysInfo => 0,
            Block::SysCfg => 0,
            Block::Clocks => 0,
            Block::Resets => 0,
            Block::Psm => 0,
            Block::IoBank0 => 0,
            Block::IoQspi => 0,
            Block::PadsBank0 => 0,
            Block::PadsQspi => 0,
            Block::Xosc => 0,
            Block::PllSys => 0,
            Block::PllUsb => 0,
            Block::BusCtrl => 0,
            Block::Timer => 0,
            Block::Watchdog => 0,
            Block::Rtc => 0,
            Block::Rosc => 0,
            Block::Vreg => 0,
            Block::Tbman => 0,
            Block::Ssi => 0,
            Block::Usb => 0,
            Block::Dma => 0,
            Block::Pio0 => 0,
            Block::Pio1 => 0,
            Block::Pwm => 0,
            Block::Adc => 0,
            Block::Uart0 => 0,
            Block::Uart1 => 0,
            Block::Spi0 => 0,
            Block::Spi1 => 0,
            Block::I2c0 => 0,
            Block::I2c1 => 0,
        }
    }
}
