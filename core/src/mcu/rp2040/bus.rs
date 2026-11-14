//! The bus and memory map, ported from rp2040js 1.3.3 (MIT): the
//! `readUint*`/`writeUint*` methods and the peripheral table of
//! `src/rp2040.ts`.
//!
//! Each access follows the reference's order of range checks, its byte and
//! halfword paths (direct for flash and SRAM, read-modify-write of the
//! containing word elsewhere, with the read's side effects), and its
//! wrapping of addresses to 32 bits.

use super::core::SIO_START;
use super::{PeripheralWrite, Rp2040, Stop, DPRAM_BYTES, FLASH_BYTES, SRAM_BYTES};

pub const BOOTROM_END: u32 = 4 * 1024 * 4;
pub const FLASH_START: u32 = 0x1000_0000;
/// Reads see flash mirrored over four 16 MB windows (XIP, NOALLOC,
/// NOCACHE, NOCACHE_NOALLOC); writes and the byte/halfword fast paths only
/// the first.
pub const FLASH_END: u32 = 0x1400_0000;
pub const RAM_START: u32 = 0x2000_0000;
pub const DPRAM_START: u32 = 0x5010_0000;

const RAM_END: u32 = RAM_START + SRAM_BYTES as u32;
const FLASH_WIN_END: u32 = FLASH_START + FLASH_BYTES as u32;
const DPRAM_END: u32 = DPRAM_START + DPRAM_BYTES as u32;
const SIO_END: u32 = SIO_START + 0x1000_0000;

/// The reference's peripheral table, keyed by `(address >>> 14) << 2`.
/// None of these is ported in R1: an access halts the machine.
const PERIPHERALS: &[(u32, &str)] = &[
    (0x18000, "SSI"),
    (0x40000, "SYSINFO_BASE"),
    (0x40004, "SYSCFG"),
    (0x40008, "CLOCKS_BASE"),
    (0x4000c, "RESETS_BASE"),
    (0x40010, "PSM_BASE"),
    (0x40014, "IO_BANK0_BASE"),
    (0x40018, "IO_QSPI_BASE"),
    (0x4001c, "PADS_BANK0_BASE"),
    (0x40020, "PADS_QSPI_BASE"),
    (0x40024, "XOSC_BASE"),
    (0x40028, "PLL_SYS_BASE"),
    (0x4002c, "PLL_USB_BASE"),
    (0x40030, "BUSCTRL_BASE"),
    (0x40034, "UART0"),
    (0x40038, "UART1"),
    (0x4003c, "SPI0"),
    (0x40040, "SPI1"),
    (0x40044, "I2C0"),
    (0x40048, "I2C1"),
    (0x4004c, "ADC"),
    (0x40050, "PWM_BASE"),
    (0x40054, "TIMER_BASE"),
    (0x40058, "WATCHDOG_BASE"),
    (0x4005c, "RTC_BASE"),
    (0x40060, "ROSC_BASE"),
    (0x40064, "VREG_AND_CHIP_RESET_BASE"),
    (0x4006c, "TBMAN_BASE"),
    (0x50000, "DMA"),
    (0x50110, "USB"),
    (0x50200, "PIO0"),
    (0x50300, "PIO1"),
];

pub fn find_peripheral(address: u32) -> Option<&'static str> {
    let key = (address >> 14) << 2;
    PERIPHERALS.iter().find(|&&(k, _)| k == key).map(|&(_, n)| n)
}

fn le32(mem: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([mem[off], mem[off + 1], mem[off + 2], mem[off + 3]])
}
fn le16(mem: &[u8], off: usize) -> u32 {
    u16::from_le_bytes([mem[off], mem[off + 1]]) as u32
}

impl Rp2040 {
    pub fn read_u32(&mut self, address: u32) -> u32 {
        if address & 3 != 0 {
            // rp2040js logs this as an error, and its logger throws.
            self.halt(Stop::UnalignedRead { address });
            return 0;
        }
        if address < BOOTROM_END {
            self.bootrom[(address / 4) as usize]
        } else if (FLASH_START..FLASH_END).contains(&address) {
            le32(&self.flash, (address & 0x00ff_ffff) as usize)
        } else if (RAM_START..RAM_END).contains(&address) {
            le32(&self.sram, (address - RAM_START) as usize)
        } else if (DPRAM_START..DPRAM_END).contains(&address) {
            le32(&self.dpram, (address - DPRAM_START) as usize)
        } else if address >> 12 == 0xe000e {
            self.ppb_read(address & 0xfff)
        } else if (SIO_START..SIO_END).contains(&address) {
            self.sio_read(address - SIO_START)
        } else if let Some(name) = find_peripheral(address) {
            self.halt(Stop::Peripheral { name, address, write: false });
            0xffff_ffff
        } else {
            self.warnings += 1;
            0xffff_ffff
        }
    }

    pub fn read_u16(&mut self, address: u32) -> u32 {
        if (FLASH_START..FLASH_WIN_END).contains(&address) {
            let off = (address - FLASH_START) as usize;
            if off + 2 > self.flash.len() {
                self.halt(Stop::OutOfRange { address });
                return 0;
            }
            le16(&self.flash, off)
        } else if (RAM_START..RAM_END).contains(&address) {
            let off = (address - RAM_START) as usize;
            if off + 2 > self.sram.len() {
                self.halt(Stop::OutOfRange { address });
                return 0;
            }
            le16(&self.sram, off)
        } else {
            let value = self.read_u32(address & 0xffff_fffc);
            if address & 2 != 0 {
                value >> 16
            } else {
                value & 0xffff
            }
        }
    }

    pub fn read_u8(&mut self, address: u32) -> u32 {
        if (FLASH_START..FLASH_WIN_END).contains(&address) {
            self.flash[(address - FLASH_START) as usize] as u32
        } else if (RAM_START..RAM_END).contains(&address) {
            self.sram[(address - RAM_START) as usize] as u32
        } else {
            let value = self.read_u16(address & 0xffff_fffe);
            if address & 1 != 0 {
                (value & 0xff00) >> 8
            } else {
                value & 0xff
            }
        }
    }

    pub fn write_u32(&mut self, address: u32, value: u32) {
        if let Some(name) = find_peripheral(address) {
            self.halt_on_peripheral_write(name, address, value);
        } else if address < BOOTROM_END {
            // A misaligned index is a non-integer property on a typed array,
            // which JavaScript ignores.
            if address & 3 == 0 {
                self.bootrom[(address / 4) as usize] = value;
            }
        } else if (FLASH_START..FLASH_WIN_END).contains(&address) {
            let off = (address - FLASH_START) as usize;
            if off + 4 > self.flash.len() {
                self.halt(Stop::OutOfRange { address });
                return;
            }
            self.flash[off..off + 4].copy_from_slice(&value.to_le_bytes());
        } else if (RAM_START..RAM_END).contains(&address) {
            let off = (address - RAM_START) as usize;
            if off + 4 > self.sram.len() {
                self.halt(Stop::OutOfRange { address });
                return;
            }
            self.sram[off..off + 4].copy_from_slice(&value.to_le_bytes());
        } else if (DPRAM_START..DPRAM_END).contains(&address) {
            // The reference stores the word and then calls the USB
            // controller's DPRAMUpdated, which is R4.
            self.halt(Stop::UsbDpram { address });
        } else if (SIO_START..SIO_END).contains(&address) {
            self.sio_write(address - SIO_START, value);
        } else if address >> 12 == 0xe000e {
            self.ppb_write(address & 0xfff, value);
        } else {
            self.warnings += 1;
        }
    }

    pub fn write_u8(&mut self, address: u32, value: u32) {
        if (RAM_START..RAM_END).contains(&address) {
            self.sram[(address - RAM_START) as usize] = value as u8;
            return;
        }
        let aligned = address & 0xffff_fffc;
        let offset = address & 3;
        if let Some(name) = find_peripheral(address) {
            // A peripheral sees the byte in all four lanes of the word.
            let b = value & 0xff;
            self.halt_on_peripheral_write(name, aligned, b | (b << 8) | (b << 16) | (b << 24));
            return;
        }
        let original = self.read_u32(aligned);
        let shift = 8 * offset;
        let new_value = (original & !(0xff << shift)) | ((value & 0xff) << shift);
        self.write_u32(aligned, new_value);
    }

    pub fn write_u16(&mut self, address: u32, value: u32) {
        if (RAM_START..RAM_END).contains(&address) {
            let off = (address - RAM_START) as usize;
            if off + 2 > self.sram.len() {
                self.halt(Stop::OutOfRange { address });
                return;
            }
            self.sram[off..off + 2].copy_from_slice(&(value as u16).to_le_bytes());
            return;
        }
        let aligned = address & 0xffff_fffc;
        let offset = address & 3;
        if let Some(name) = find_peripheral(address) {
            // A peripheral sees the halfword in both halves of the word.
            let h = value & 0xffff;
            self.halt_on_peripheral_write(name, aligned, h | (h << 16));
            return;
        }
        let original = self.read_u32(aligned);
        if offset == 3 {
            // DataView.setUint16 at byte 3 of a 4-byte buffer: RangeError.
            self.halt(Stop::OutOfRange { address });
            return;
        }
        let shift = 8 * offset;
        let new_value = (original & !(0xffff << shift)) | ((value & 0xffff) << shift);
        self.write_u32(aligned, new_value);
    }

    /// Halts on a write to a peripheral R1 does not port, recording what
    /// the reference would hand that peripheral's `writeUint32Atomic`.
    fn halt_on_peripheral_write(&mut self, name: &'static str, address: u32, value: u32) {
        if self.stop.is_none() {
            self.peripheral_write = Some(PeripheralWrite {
                offset: address & 0xfff,
                atomic: (address & 0x3000) >> 12,
                value,
            });
        }
        self.halt(Stop::Peripheral { name, address, write: true });
    }

    /// A side-effect-free read of plain memory, for the harness's decodes.
    pub fn peek_u32(&self, address: u32) -> u32 {
        let a = address & !3;
        if a < BOOTROM_END {
            self.bootrom[(a / 4) as usize]
        } else if (FLASH_START..FLASH_END).contains(&a) {
            le32(&self.flash, (a & 0x00ff_ffff) as usize)
        } else if (RAM_START..RAM_END).contains(&a) {
            le32(&self.sram, (a - RAM_START) as usize)
        } else {
            0xffff_ffff
        }
    }
}
