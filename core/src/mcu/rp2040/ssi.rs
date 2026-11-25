//! The flash (XIP) SSI controller and the UF2 loader, ported from rp2040js
//! 1.3.3 (MIT): `src/peripherals/ssi.ts`, and `loadUF2IntoFlash` from
//! brownout's own wrapper (`src/sim/engine/rp2040.ts`).
//!
//! rp2040js does not model the flash interface: it keeps the control
//! registers boot2 writes, reports the transmit FIFO empty and the receive
//! FIFO ready, and answers boot2's status poll by zeroing the data register.
//! Flash itself is the plain 16 MB array on the bus (`bus.rs`), readable
//! through the four XIP mirrors from the first instruction, which is why
//! boot2's SSI programming has nothing to wait for.
//!
//! The flash array stays out of the per-step rollback copy (`Rp2040::step`
//! clones the core only), and the state record does not hash it: the lock-step
//! harness hashes it at checkpoints (`rp_flash_hash`).

use super::Rp2040;

const SSI_CTRLR0: u32 = 0x00;
const SSI_CTRLR1: u32 = 0x04;
const SSI_SSIENR: u32 = 0x08;
const SSI_BAUDR: u32 = 0x14;
const SSI_TXFLR: u32 = 0x20;
const SSI_RXFLR: u32 = 0x24;
const SSI_SR: u32 = 0x28;
const SSI_SR_TFNF_BITS: u32 = 0x2;
const SSI_SR_TFE_BITS: u32 = 0x4;
const SSI_SR_RFNE_BITS: u32 = 0x8;
const SSI_IDR: u32 = 0x58;
const SSI_VERSION_ID: u32 = 0x5c;
const SSI_DR0: u32 = 0x60;
const SSI_RX_SAMPLE_DLY: u32 = 0xf0;
const SSI_SPI_CTRL_R0: u32 = 0xf4;
const SSI_TXD_DRIVE_EDGE: u32 = 0xf8;

const CMD_READ_STATUS: u32 = 0x05;

#[derive(Default)]
pub struct Ssi {
    pub dr0: u32,
    pub txflr: u32,
    pub rxflr: u32,
    pub baudr: u32,
    pub ctrlr0: u32,
    pub ctrlr1: u32,
    pub ssienr: u32,
    pub spi_ctrlr0: u32,
    pub rx_sample_dly: u32,
    pub txd_drive_edge: u32,
}

impl Ssi {
    /// FNV-1a over the registers in declaration order, for the harness.
    pub fn digest(&self) -> u32 {
        let mut h = 0x811c_9dc5u32;
        for v in [
            self.dr0, self.txflr, self.rxflr, self.baudr, self.ctrlr0, self.ctrlr1, self.ssienr, self.spi_ctrlr0,
            self.rx_sample_dly, self.txd_drive_edge,
        ] {
            h = (h ^ v).wrapping_mul(0x0100_0193);
        }
        h
    }
}

const UF2_MAGIC_START0: u32 = 0x0a32_4655;
const UF2_MAGIC_START1: u32 = 0x9e5d_5157;
const FLASH_BASE: u32 = 0x1000_0000;

impl Rp2040 {
    pub(crate) fn ssi_read(&mut self, offset: u32) -> u32 {
        let s = &self.periph.ssi;
        match offset {
            SSI_TXFLR => s.txflr,
            SSI_RXFLR => s.rxflr,
            SSI_CTRLR0 => s.ctrlr0,
            SSI_CTRLR1 => s.ctrlr1,
            SSI_SSIENR => s.ssienr,
            SSI_BAUDR => s.baudr,
            SSI_SR => SSI_SR_TFE_BITS | SSI_SR_RFNE_BITS | SSI_SR_TFNF_BITS,
            SSI_IDR => 0x5153_5049,
            SSI_VERSION_ID => 0x3430_312a,
            SSI_RX_SAMPLE_DLY => s.rx_sample_dly,
            SSI_TXD_DRIVE_EDGE => s.txd_drive_edge,
            SSI_SPI_CTRL_R0 => s.spi_ctrlr0,
            SSI_DR0 => s.dr0,
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
                0xffff_ffff
            }
        }
    }

    pub(crate) fn ssi_write(&mut self, offset: u32, value: u32) {
        let s = &mut self.periph.ssi;
        match offset {
            SSI_TXFLR => s.txflr = value,
            SSI_RXFLR => s.rxflr = value,
            SSI_CTRLR0 => s.ctrlr0 = value,
            SSI_CTRLR1 => s.ctrlr1 = value,
            SSI_SSIENR => s.ssienr = value,
            SSI_BAUDR => s.baudr = value,
            SSI_RX_SAMPLE_DLY => s.rx_sample_dly = value & 0xff,
            SSI_TXD_DRIVE_EDGE => s.txd_drive_edge = value & 0xff,
            SSI_SPI_CTRL_R0 => s.spi_ctrlr0 = value,
            SSI_DR0 => {
                if value == CMD_READ_STATUS {
                    s.dr0 = 0; // tell stage2 that we completed a write
                }
            }
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
            }
        }
    }

    /// brownout's `loadUF2IntoFlash`: copies every UF2 block aimed at flash
    /// to its offset and ignores the rest. Returns the blocks applied.
    pub fn load_uf2(&mut self, uf2: &[u8]) -> u32 {
        let word = |off: usize| u32::from_le_bytes([uf2[off], uf2[off + 1], uf2[off + 2], uf2[off + 3]]);
        let mut applied = 0;
        let mut off = 0usize;
        while off + 512 <= uf2.len() {
            if word(off) == UF2_MAGIC_START0 && word(off + 4) == UF2_MAGIC_START1 {
                let target = word(off + 12);
                let size = word(off + 16) as u64;
                // `dest < 0` in the reference is a target below the flash base.
                if target >= FLASH_BASE && (target - FLASH_BASE) as u64 + size <= self.flash.len() as u64 {
                    let dest = (target - FLASH_BASE) as usize;
                    // `subarray` clips at the end of the image.
                    let end = (off + 32 + size as usize).min(uf2.len());
                    let src = &uf2[off + 32..end];
                    self.flash[dest..dest + src.len()].copy_from_slice(src);
                    self.mark_flash_dirty(dest, src.len());
                    applied += 1;
                }
            }
            off += 512;
        }
        applied
    }

    /// Marks the pages of a range written outside the bus (a loader).
    pub fn mark_flash_dirty(&mut self, offset: usize, len: usize) {
        if len == 0 {
            return;
        }
        for p in (offset >> 12)..=((offset + len - 1) >> 12) {
            self.flash_dirty.insert(p as u32);
        }
    }

    /// FNV-1a over the pages written since the last call, each as its index
    /// followed by its words, in page order; clears the set. A full hash
    /// also clears it.
    pub fn flash_dirty_hash(&mut self) -> u32 {
        let mut h = 0x811c_9dc5u32;
        for &p in &self.flash_dirty {
            h = (h ^ p).wrapping_mul(0x0100_0193);
            let start = (p as usize) << 12;
            for w in self.flash[start..start + 4096].chunks_exact(4) {
                h = (h ^ u32::from_le_bytes([w[0], w[1], w[2], w[3]])).wrapping_mul(0x0100_0193);
            }
        }
        self.flash_dirty.clear();
        h
    }

    /// FNV-1a over flash as little-endian words.
    pub fn flash_hash(&self) -> u32 {
        let mut h = 0x811c_9dc5u32;
        for w in self.flash.chunks_exact(4) {
            h = (h ^ u32::from_le_bytes([w[0], w[1], w[2], w[3]])).wrapping_mul(0x0100_0193);
        }
        h
    }
}
