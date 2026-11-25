//! Unit tests for R3: the flash SSI controller, the UF2 loader and the flash
//! hash. Expected values come from the RP2040 datasheet (section 4.10, the
//! DW_apb_ssi register list) and the UF2 format specification; where
//! rp2040js 1.3.3 departs from the datasheet a `reference_quirk_*` test
//! names both. rp2040js has no test suite for its SSI.

use super::*;

const SSI: u32 = 0x1800_0000;
const CTRLR0: u32 = 0x00;
const SSIENR: u32 = 0x08;
const BAUDR: u32 = 0x14;
const SR: u32 = 0x28;
const IDR: u32 = 0x58;
const VERSION_ID: u32 = 0x5c;
const DR0: u32 = 0x60;

fn rd(m: &mut Rp2040, off: u32) -> u32 {
    m.read_u32(SSI + off)
}
fn wr(m: &mut Rp2040, off: u32, v: u32) {
    m.write_u32(SSI + off, v)
}

// ---- the SSI -------------------------------------------------------------------

#[test]
fn ssi_identification_registers() {
    // Datasheet 4.10.13: SSI_IDR reads 0x51535049 and SSI_VERSION_ID
    // 0x3430312a (ASCII "IPSQ" and "*104" little-endian).
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, IDR), u32::from_le_bytes(*b"IPSQ"));
    assert_eq!(rd(&mut m, VERSION_ID), u32::from_le_bytes(*b"*104"));
    wr(&mut m, IDR, 0);
    assert_eq!(rd(&mut m, IDR), 0x5153_5049, "read-only");
}

#[test]
fn ssi_control_registers_hold_what_boot2_writes() {
    // The sequence a boot2 stage 2 runs: disable (SSIENR = 0), program
    // BAUDR, CTRLR0 (DFS_32 = 31 in bits 20:16, TMOD 3 = EEPROM read in bits
    // 9:8), CTRLR1, SPI_CTRLR0, then enable.
    let mut m = Rp2040::new();
    for (off, v) in [
        (SSIENR, 0),
        (BAUDR, 4),
        (CTRLR0, (31 << 16) | (3 << 8)),
        (0x04, 0), // CTRLR1: NDF
        (0xf4, 0x0300_0218), // SPI_CTRLR0
        (SSIENR, 1),
    ] {
        wr(&mut m, off, v);
        assert_eq!(rd(&mut m, off), v, "register {off:#x}");
    }
    assert!(m.stop.is_none());
}

#[test]
fn ssi_sample_delay_and_drive_edge_are_eight_bits() {
    // Datasheet: RX_SAMPLE_DLY.RSD is bits 7:0, TXD_DRIVE_EDGE.TDE bits 7:0.
    let mut m = Rp2040::new();
    wr(&mut m, 0xf0, 0xffff_ffff);
    wr(&mut m, 0xf8, 0x1234_5678);
    assert_eq!(rd(&mut m, 0xf0), 0xff);
    assert_eq!(rd(&mut m, 0xf8), 0x78);
}

#[test]
fn ssi_status_reports_transmit_ready_so_boot2_never_waits() {
    // SR bits: BUSY 0, TFNF 1, TFE 2, RFNE 3. Boot2 polls TFE and BUSY.
    let mut m = Rp2040::new();
    let sr = rd(&mut m, SR);
    assert_eq!(sr & 1, 0, "never busy");
    assert_eq!(sr & 0b110, 0b110, "transmit FIFO empty and not full");
}

#[test]
fn reference_quirk_ssi_status_is_constant_with_receive_data_ready() {
    // Datasheet: SR resets to 0x6 (TFE, TFNF), RFNE rises only with data in
    // the receive FIFO, and the register follows the FIFOs. rp2040js returns
    // 0xe whatever happens, so a boot2 that reads the status byte always
    // finds a byte waiting.
    let mut m = Rp2040::new();
    assert_eq!(rd(&mut m, SR), 0xe);
    wr(&mut m, DR0, 0x35);
    assert_eq!(rd(&mut m, SR), 0xe);
}

#[test]
fn reference_quirk_ssi_data_register_only_answers_the_status_command() {
    // Datasheet: DR0 pushes a word onto the transmit FIFO and reads pop the
    // receive FIFO. rp2040js keeps one register that reads 0 and is cleared
    // by a write of 0x05 (flash command READ STATUS), the poll boot2 uses to
    // wait for a write to finish; any other write is dropped.
    let mut m = Rp2040::new();
    wr(&mut m, DR0, 0xab);
    assert_eq!(rd(&mut m, DR0), 0);
    wr(&mut m, DR0, 0x05);
    assert_eq!(rd(&mut m, DR0), 0);
    assert!(m.stop.is_none());
}

#[test]
fn reference_quirk_ssi_fifo_levels_are_plain_registers() {
    // Datasheet: TXFLR and RXFLR are read-only FIFO levels. rp2040js stores
    // whatever is written.
    let mut m = Rp2040::new();
    wr(&mut m, 0x20, 7);
    wr(&mut m, 0x24, 9);
    assert_eq!((rd(&mut m, 0x20), rd(&mut m, 0x24)), (7, 9));
}

#[test]
fn reference_quirk_ssi_other_registers_are_unmodelled() {
    // SER 0x10, TXFTLR 0x18, RXFTLR 0x1c, IMR 0x2c, ISR 0x30, DMACR 0x4c...
    let mut m = Rp2040::new();
    for off in [0x10u32, 0x18, 0x1c, 0x2c, 0x30, 0x4c] {
        wr(&mut m, off, 0x55);
        assert_eq!(rd(&mut m, off), 0xffff_ffff, "register {off:#x}");
    }
    assert!(m.stop.is_none());
}

#[test]
fn ssi_registers_take_the_atomic_aliases() {
    let mut m = Rp2040::new();
    wr(&mut m, BAUDR, 0b0101);
    m.write_u32(SSI + BAUDR + 0x2000, 0b0010); // SET
    assert_eq!(rd(&mut m, BAUDR), 0b0111);
    m.write_u32(SSI + BAUDR + 0x3000, 0b0001); // CLEAR
    assert_eq!(rd(&mut m, BAUDR), 0b0110);
    m.write_u32(SSI + BAUDR + 0x1000, 0b1111); // XOR
    assert_eq!(rd(&mut m, BAUDR), 0b1001);
}

// ---- the UF2 loader ---------------------------------------------------------------

/// One 512-byte UF2 block, laid out as the UF2 specification gives it: three
/// start words, flags, target address, payload size, block number, block
/// count, family id, a 476-byte data area and the end magic.
fn uf2_block(target: u32, payload: &[u8], n: u32, total: u32) -> Vec<u8> {
    let mut b = vec![0u8; 512];
    let words = [0x0a32_4655u32, 0x9e5d_5157, 0x0000_2000, target, payload.len() as u32, n, total, 0xe48b_ff56];
    for (i, w) in words.iter().enumerate() {
        b[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
    }
    b[32..32 + payload.len()].copy_from_slice(payload);
    b[508..512].copy_from_slice(&0x0ab1_6f30u32.to_le_bytes());
    b
}

#[test]
fn uf2_blocks_land_at_their_flash_offsets() {
    let mut m = Rp2040::new();
    let p0: Vec<u8> = (0..256u32).map(|i| i as u8).collect();
    let p1: Vec<u8> = (0..256u32).map(|i| (255 - i) as u8).collect();
    let mut img = uf2_block(0x1000_0000, &p0, 0, 2);
    img.extend(uf2_block(0x1000_1f00, &p1, 1, 2));
    assert_eq!(m.load_uf2(&img), 2);
    assert_eq!(&m.flash[0..256], &p0[..]);
    assert_eq!(&m.flash[0x1f00..0x2000], &p1[..]);
    assert_eq!(m.flash[256], 0xff, "flash between blocks stays erased");
    // The core sees it through every XIP mirror.
    assert_eq!(m.read_u32(0x1000_0004), u32::from_le_bytes([4, 5, 6, 7]));
    assert_eq!(m.read_u32(0x1300_1f00), u32::from_le_bytes([255, 254, 253, 252]));
}

#[test]
fn uf2_blocks_outside_flash_or_without_the_magic_are_ignored() {
    let mut m = Rp2040::new();
    let payload = [0x11u8; 256];
    let mut img = uf2_block(0x2000_0000, &payload, 0, 5); // SRAM target
    img.extend(uf2_block(0x0fff_ff00, &payload, 1, 5)); // below flash
    img.extend(uf2_block(0x10ff_ff80, &payload, 2, 5)); // runs past 16 MB
    let mut bad = uf2_block(0x1000_0000, &payload, 3, 5);
    bad[0] ^= 1; // first magic wrong
    img.extend(bad);
    img.extend(vec![0x22u8; 300]); // a short tail is not a block
    assert_eq!(m.load_uf2(&img), 0);
    assert!(m.flash.iter().all(|&b| b == 0xff));
    // A block ending exactly at the end of flash is kept.
    let img = uf2_block(0x10ff_ff00, &payload, 0, 1);
    assert_eq!(m.load_uf2(&img), 1);
    assert_eq!(m.flash[0x00ff_ff00], 0x11);
}

#[test]
fn uf2_payload_larger_than_the_data_area_reads_on_into_the_next_block() {
    // The reference's `subarray(off + 32, off + 32 + size)` is not bounded
    // by the block, so a size above 476 takes bytes of what follows.
    let mut m = Rp2040::new();
    let mut img = uf2_block(0x1000_0000, &[0xaa; 476], 0, 2);
    img.extend(uf2_block(0x1000_1000, &[0xbb; 4], 1, 2));
    img[16..20].copy_from_slice(&(476u32 + 4 + 4).to_le_bytes());
    assert_eq!(m.load_uf2(&img), 2);
    assert_eq!(m.flash[475], 0xaa);
    // 476 data bytes, 4 end-magic bytes, then the next block's first 4 bytes.
    assert_eq!(&m.flash[476..480], &0x0ab1_6f30u32.to_le_bytes());
    assert_eq!(&m.flash[480..484], &0x0a32_4655u32.to_le_bytes());
}

#[test]
fn the_wrapper_boot_starts_the_core_at_the_xip_base() {
    // RP2040Mcu.boot(): boot ROM loaded, UF2 into flash, `core.PC =
    // 0x10000000`. The first instructions are boot2's, fetched from flash.
    let mut m = Rp2040::new();
    m.load_bootrom(&[0x2004_1f00, 0xee]);
    let mut img = Vec::new();
    // boot2: LDR r0, =SSI; MOVS r1, #7; STR r1, [r0, #8]; BKPT
    let code: [u16; 8] = [0x4802, 0x2107, 0x6081, 0xbe01, 0xbf00, 0xbf00, 0x0000, 0x1800];
    let bytes: Vec<u8> = code.iter().flat_map(|h| h.to_le_bytes()).collect();
    img.extend(uf2_block(0x1000_0000, &bytes, 0, 1));
    m.load_uf2(&img);
    m.cpu.r[15] = 0x1000_0000;
    m.cpu.r[13] = 0x2004_1f00;
    m.run(10);
    assert_eq!(m.stop, Some(Stop::Break { code: 1 }));
    assert_eq!(rd(&mut m, SSIENR), 7);
}

// ---- the flash hash ------------------------------------------------------------------

#[test]
fn flash_hash_is_fnv_1a_over_little_endian_words() {
    let mut m = Rp2040::new();
    // All 0xff: 4 Mi words of 0xffffffff.
    let mut h = 0x811c_9dc5u32;
    for _ in 0..(16 * 1024 * 1024 / 4) {
        h = (h ^ 0xffff_ffff).wrapping_mul(0x0100_0193);
    }
    assert_eq!(m.flash_hash(), h);
    let before = m.flash_hash();
    m.write_u32(0x1000_0040, 0x1234_5678);
    assert_ne!(m.flash_hash(), before);
    m.write_u32(0x1000_0040, 0xffff_ffff);
    assert_eq!(m.flash_hash(), before);
    // RAM and the SSI registers do not move it.
    m.write_u32(0x2000_0000, 1);
    wr(&mut m, BAUDR, 9);
    assert_eq!(m.flash_hash(), before);
}

#[test]
fn the_per_step_rollback_copies_the_core_only() {
    // A step that stops on an unported peripheral restores the core it cloned
    // before the instruction; flash (16 MB) and SRAM are never in that copy,
    // so a stop costs nothing proportional to them. Flash written before the
    // stop keeps its bytes, like the rest of memory.
    let mut m = Rp2040::new();
    m.write_u32(0x1000_0000, 0x1234_5678);
    let before = m.cpu.clone();
    m.read_u32(0x2000_0002); // unaligned: stops
    assert!(m.stop.is_some());
    assert_eq!(m.cpu.r, before.r);
    assert_eq!(m.read_u32(0x1000_0000), 0x1234_5678);
}
