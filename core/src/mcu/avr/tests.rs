//! AVR1 unit tests. Expected values are derived here from the AVR instruction
//! set manual (the boolean flag equations, the operand and cycle tables),
//! never read back from the port. The places where avr8js 0.21.0 departs
//! from the manual are named "reference quirk": those tests pin the
//! reference's behaviour because the lock-step gate demands it, and say what
//! the manual would give.

use super::cpu::{Cpu, ADDR_EIND, ADDR_RAMPZ, ADDR_SREG};
use super::instr::{avr_instruction, decode, Op};
use super::interrupt::Interrupt;

const C: u8 = 1;
const Z: u8 = 2;
const N: u8 = 4;
const V: u8 = 8;
const S: u8 = 0x10;
const H: u8 = 0x20;
const T: u8 = 0x40;
const I: u8 = 0x80;

/// One CPU with 32K words of flash and the reference's 8 KiB of SRAM.
struct Rig {
    cpu: Cpu,
}

impl Rig {
    fn new() -> Rig {
        Rig { cpu: Cpu::new(vec![0; 0x8000], 8192) }
    }

    /// Places `words` at `at` and executes the one instruction there; returns
    /// the cycles it took.
    fn exec_at(&mut self, at: i32, words: &[u16]) -> u64 {
        for (i, w) in words.iter().enumerate() {
            self.cpu.prog[at as usize + i] = *w;
        }
        self.cpu.pc = at;
        let before = self.cpu.cycles;
        avr_instruction(&mut self.cpu);
        self.cpu.cycles - before
    }

    fn exec(&mut self, words: &[u16]) -> u64 {
        self.exec_at(0, words)
    }

    fn reg(&self, n: usize) -> u8 {
        self.cpu.data[n]
    }

    fn set_reg(&mut self, n: usize, v: u8) {
        self.cpu.data[n] = v;
    }

    fn sreg(&self) -> u8 {
        self.cpu.data[ADDR_SREG]
    }

    fn set_sreg(&mut self, v: u8) {
        self.cpu.data[ADDR_SREG] = v;
    }

    fn set_pair(&mut self, lo: usize, v: u16) {
        self.cpu.set16(lo, v);
    }

    fn pair(&self, lo: usize) -> u16 {
        self.cpu.get16(lo)
    }
}

// Operand encoders, from the manual's encodings.
fn two_reg(base: u16, d: usize, r: usize) -> u16 {
    base | ((r as u16 & 0x10) << 5) | ((d as u16) << 4) | (r as u16 & 0xf)
}
fn reg_imm(base: u16, d: usize, k: u8) -> u16 {
    base | (((k as u16) & 0xf0) << 4) | (((d as u16) - 16) << 4) | (k as u16 & 0xf)
}
fn one_reg(base: u16, d: usize) -> u16 {
    base | ((d as u16) << 4)
}

// ---- the manual's flag equations -------------------------------------------

fn b(x: u8, n: u32) -> bool {
    (x >> n) & 1 != 0
}

/// Flags H, S, V, N, Z, C of `Rd + Rr + c` per the manual.
fn spec_add(d: u8, r: u8, c: u8) -> (u8, u8) {
    let res = d.wrapping_add(r).wrapping_add(c);
    let h = (b(d, 3) && b(r, 3)) || (b(r, 3) && !b(res, 3)) || (!b(res, 3) && b(d, 3));
    let v = (b(d, 7) && b(r, 7) && !b(res, 7)) || (!b(d, 7) && !b(r, 7) && b(res, 7));
    let n = b(res, 7);
    let cy = (b(d, 7) && b(r, 7)) || (b(r, 7) && !b(res, 7)) || (!b(res, 7) && b(d, 7));
    (res, flags(h, n ^ v, v, n, res == 0, cy))
}

/// Flags of `Rd - Rr - c` per the manual (Z is the plain `R == 0`; SBC and
/// CPC AND it with the old Z).
fn spec_sub(d: u8, r: u8, c: u8) -> (u8, u8) {
    let res = d.wrapping_sub(r).wrapping_sub(c);
    let h = (!b(d, 3) && b(r, 3)) || (b(r, 3) && b(res, 3)) || (b(res, 3) && !b(d, 3));
    let v = (b(d, 7) && !b(r, 7) && !b(res, 7)) || (!b(d, 7) && b(r, 7) && b(res, 7));
    let n = b(res, 7);
    let cy = (!b(d, 7) && b(r, 7)) || (b(r, 7) && b(res, 7)) || (b(res, 7) && !b(d, 7));
    (res, flags(h, n ^ v, v, n, res == 0, cy))
}

fn flags(h: bool, s: bool, v: bool, n: bool, z: bool, c: bool) -> u8 {
    (h as u8) * H | (s as u8) * S | (v as u8) * V | (n as u8) * N | (z as u8) * Z | (c as u8)
}

/// Reference quirk: avr8js takes H from bit 0 of the carry chain
/// (`1 & ((d & r) | (r & ~R) | (~R & d))`); the manual takes bit 3.
fn quirk_h_add(d: u8, r: u8, res: u8) -> u8 {
    (((d & r) | (r & !res) | (!res & d)) & 1) * H
}
/// The same for subtraction (`1 & ((~d & r) | (r & R) | (R & ~d))`).
fn quirk_h_sub(d: u8, r: u8, res: u8) -> u8 {
    (((!d & r) | (r & res) | (res & !d)) & 1) * H
}

const KEEP: u8 = I | T; // bits no arithmetic instruction touches
const SVNZC: u8 = S | V | N | Z | C;

// ---- arithmetic ---------------------------------------------------------------

#[test]
fn add_adc_match_the_manual_for_every_operand_pair() {
    let mut rig = Rig::new();
    for carry_in in 0..2u8 {
        for d in 0..=255u8 {
            for r in 0..=255u8 {
                for (adc, op) in [(false, two_reg(0x0c00, 16, 17)), (true, two_reg(0x1c00, 16, 17))] {
                    let c = if adc { carry_in } else { 0 };
                    rig.set_reg(16, d);
                    rig.set_reg(17, r);
                    rig.set_sreg(KEEP | carry_in);
                    let cycles = rig.exec(&[op]);
                    let (res, fl) = spec_add(d, r, c);
                    assert_eq!(rig.reg(16), res, "{} {d} {r} {c}", if adc { "ADC" } else { "ADD" });
                    assert_eq!(rig.sreg() & (KEEP | SVNZC), KEEP | (fl & SVNZC), "d={d} r={r} c={c} adc={adc}");
                    // reference quirk: H is bit 0 of the carry chain
                    assert_eq!(rig.sreg() & H, quirk_h_add(d, r, res));
                    assert_eq!(cycles, 1);
                    assert_eq!(rig.cpu.pc, 1);
                }
            }
        }
    }
}

#[test]
fn reference_quirk_half_carry_is_bit_0_of_the_carry_chain() {
    // 0x08 + 0x08 carries out of bit 3: the manual sets H, avr8js does not.
    let mut rig = Rig::new();
    rig.set_reg(16, 8);
    rig.set_reg(17, 8);
    rig.exec(&[two_reg(0x0c00, 16, 17)]);
    assert_eq!(spec_add(8, 8, 0).1 & H, H);
    assert_eq!(rig.sreg() & H, 0);
    // 0x01 + 0x01 does not carry out of bit 3: the manual clears H, avr8js
    // sets it (bit 0 of d & r).
    rig.set_reg(16, 1);
    rig.set_reg(17, 1);
    rig.exec(&[two_reg(0x0c00, 16, 17)]);
    assert_eq!(spec_add(1, 1, 0).1 & H, 0);
    assert_eq!(rig.sreg() & H, H);
}

#[test]
fn sub_cp_sbc_cpc_match_the_manual_for_every_operand_pair() {
    let mut rig = Rig::new();
    for carry_in in 0..2u8 {
        for z_in in 0..2u8 {
            for d in 0..=255u8 {
                for r in 0..=255u8 {
                    // (opcode, takes carry, keeps Z, writes Rd)
                    let cases = [
                        (two_reg(0x1800, 16, 17), false, false, true), // SUB
                        (two_reg(0x1400, 16, 17), false, false, false), // CP
                        (two_reg(0x0800, 16, 17), true, true, true),   // SBC
                        (two_reg(0x0400, 16, 17), true, true, false),  // CPC
                    ];
                    for (op, uses_c, sticky_z, writes) in cases {
                        rig.set_reg(16, d);
                        rig.set_reg(17, r);
                        rig.set_sreg(KEEP | carry_in | z_in * Z);
                        let c = if uses_c { carry_in } else { 0 };
                        let cycles = rig.exec(&[op]);
                        let (res, mut fl) = spec_sub(d, r, c);
                        if sticky_z {
                            // reference quirk: it tests the unwrapped difference,
                            // so 0 - 0xff - 1 (= -256, a zero byte) never keeps Z
                            let unwrapped = d as i32 - r as i32 - c as i32;
                            fl = (fl & !Z) | if unwrapped == 0 && z_in == 1 { Z } else { 0 };
                        }
                        assert_eq!(rig.reg(16), if writes { res } else { d });
                        assert_eq!(rig.sreg() & (KEEP | SVNZC), KEEP | (fl & SVNZC), "op {op:#06x} d={d} r={r} c={c} z={z_in}");
                        assert_eq!(rig.sreg() & H, quirk_h_sub(d, r, res));
                        assert_eq!(cycles, 1);
                    }
                }
            }
        }
    }
}

#[test]
fn subi_sbci_cpi_match_the_manual() {
    let mut rig = Rig::new();
    for carry_in in 0..2u8 {
        for z_in in 0..2u8 {
            for d in (0..=255u8).step_by(3) {
                for k in 0..=255u8 {
                    let cases = [
                        (reg_imm(0x5000, 20, k), false, false, true), // SUBI
                        (reg_imm(0x4000, 20, k), true, true, true),   // SBCI
                        (reg_imm(0x3000, 20, k), false, false, false), // CPI
                    ];
                    for (op, uses_c, sticky_z, writes) in cases {
                        rig.set_reg(20, d);
                        rig.set_sreg(KEEP | carry_in | z_in * Z);
                        let c = if uses_c { carry_in } else { 0 };
                        let cycles = rig.exec(&[op]);
                        let (res, mut fl) = spec_sub(d, k, c);
                        if sticky_z {
                            // reference quirk: see the register form above
                            let unwrapped = d as i32 - k as i32 - c as i32;
                            fl = (fl & !Z) | if unwrapped == 0 && z_in == 1 { Z } else { 0 };
                        }
                        assert_eq!(rig.reg(20), if writes { res } else { d });
                        assert_eq!(rig.sreg() & (KEEP | SVNZC), KEEP | (fl & SVNZC), "op {op:#06x} d={d} k={k}");
                        assert_eq!(rig.sreg() & H, quirk_h_sub(d, k, res));
                        assert_eq!(cycles, 1);
                    }
                }
            }
        }
    }
}

#[test]
fn inc_dec_com_neg_match_the_manual() {
    let mut rig = Rig::new();
    for d in 0..=255u8 {
        for c_in in 0..2u8 {
            // INC: V when the result is 0x80; C and H untouched.
            rig.set_reg(5, d);
            rig.set_sreg(KEEP | H | c_in);
            assert_eq!(rig.exec(&[one_reg(0x9403, 5)]), 1);
            let res = d.wrapping_add(1);
            let v = res == 0x80;
            let n = b(res, 7);
            assert_eq!(rig.reg(5), res);
            assert_eq!(rig.sreg(), KEEP | H | c_in | flags(false, n ^ v, v, n, res == 0, false));

            // DEC: V when the result is 0x7f.
            rig.set_reg(5, d);
            rig.set_sreg(KEEP | H | c_in);
            assert_eq!(rig.exec(&[one_reg(0x940a, 5)]), 1);
            let res = d.wrapping_sub(1);
            let v = res == 0x7f;
            let n = b(res, 7);
            assert_eq!(rig.reg(5), res);
            assert_eq!(rig.sreg(), KEEP | H | c_in | flags(false, n ^ v, v, n, res == 0, false));

            // COM: result 0xff - Rd, C set, V clear, H untouched.
            rig.set_reg(5, d);
            rig.set_sreg(KEEP | H | c_in | V);
            assert_eq!(rig.exec(&[one_reg(0x9400, 5)]), 1);
            let res = 0xff - d;
            let n = b(res, 7);
            assert_eq!(rig.reg(5), res);
            assert_eq!(rig.sreg(), KEEP | H | flags(false, n, false, n, res == 0, true));

            // NEG: C when the result is nonzero, V when it is 0x80.
            rig.set_reg(5, d);
            rig.set_sreg(KEEP | c_in);
            assert_eq!(rig.exec(&[one_reg(0x9401, 5)]), 1);
            let res = 0u8.wrapping_sub(d);
            let n = b(res, 7);
            assert_eq!(rig.reg(5), res);
            let spec_v = res == 0x80;
            let got = rig.sreg();
            assert_eq!(got & (N | Z | C), flags(false, false, false, n, res == 0, res != 0));
            // reference quirk: V is never set (it compares the unwrapped
            // result with 128); S is therefore N alone.
            assert_eq!(got & V, 0);
            assert_eq!(got & S, if n { S } else { 0 });
            if d == 0x80 {
                assert!(spec_v, "the manual sets V for 0x80");
            }
            // reference quirk: H is bit 0 of (R | Rd), the manual says R3 | Rd3.
            assert_eq!(got & H, ((res | d) & 1) * H);
        }
    }
}

#[test]
fn adiw_sbiw_match_the_manual() {
    let mut rig = Rig::new();
    for lo_pair in [24usize, 26, 28, 30] {
        let sel = ((lo_pair - 24) / 2) as u16;
        for v in [0u16, 1, 0x3f, 0x40, 0x7fc0, 0x7fff, 0x8000, 0x8001, 0xffc0, 0xffff, 0x1234] {
            for k in 0..64u16 {
                let enc = |base: u16| base | ((k & 0x30) << 2) | (sel << 4) | (k & 0xf);
                // ADIW
                rig.set_pair(lo_pair, v);
                rig.set_sreg(KEEP | H);
                assert_eq!(rig.exec(&[enc(0x9600)]), 2);
                let res = v.wrapping_add(k);
                let n = res & 0x8000 != 0;
                let vv = v & 0x8000 == 0 && n;
                let c = res & 0x8000 == 0 && v & 0x8000 != 0;
                assert_eq!(rig.pair(lo_pair), res);
                assert_eq!(rig.sreg(), KEEP | H | flags(false, n ^ vv, vv, n, res == 0, c));
                // SBIW
                rig.set_pair(lo_pair, v);
                rig.set_sreg(KEEP);
                assert_eq!(rig.exec(&[enc(0x9700)]), 2);
                let res = v.wrapping_sub(k);
                let n = res & 0x8000 != 0;
                let vv = v & 0x8000 != 0 && !n;
                let c = n && v & 0x8000 == 0;
                assert_eq!(rig.pair(lo_pair), res);
                assert_eq!(rig.sreg() & (SVNZC | KEEP), KEEP | flags(false, n ^ vv, vv, n, res == 0, c) & SVNZC);
                // reference quirk: SBIW writes H (bit 0 of its carry chain);
                // the manual leaves H untouched.
                let (a, l, r) = (v as i32, k as i32, res as i32 - if (v as i32 - k as i32) < 0 { 0x10000 } else { 0 });
                let chain = ((!a & l) | (l & r) | (r & !a)) & 1;
                assert_eq!(rig.sreg() & H, (chain as u8) * H);
            }
        }
    }
}

// ---- logic and shifts ---------------------------------------------------------

#[test]
fn and_or_eor_andi_ori_match_the_manual() {
    let mut rig = Rig::new();
    for d in (0..=255u8).step_by(5) {
        for r in 0..=255u8 {
            let cases: [(u16, fn(u8, u8) -> u8); 3] = [
                (two_reg(0x2000, 16, 17), |a, b| a & b),
                (two_reg(0x2800, 16, 17), |a, b| a | b),
                (two_reg(0x2400, 16, 17), |a, b| a ^ b),
            ];
            for (op, f) in cases {
                rig.set_reg(16, d);
                rig.set_reg(17, r);
                rig.set_sreg(KEEP | H | C | V);
                assert_eq!(rig.exec(&[op]), 1);
                let res = f(d, r);
                let n = b(res, 7);
                assert_eq!(rig.reg(16), res);
                // V cleared, S = N, C and H unchanged.
                assert_eq!(rig.sreg(), KEEP | H | C | flags(false, n, false, n, res == 0, false));
            }
            let imm_cases: [(u16, fn(u8, u8) -> u8); 2] = [(0x7000, |a, b| a & b), (0x6000, |a, b| a | b)];
            for (base, f) in imm_cases {
                rig.set_reg(20, d);
                rig.set_sreg(KEEP);
                assert_eq!(rig.exec(&[reg_imm(base, 20, r)]), 1);
                let res = f(d, r);
                assert_eq!(rig.reg(20), res);
                assert_eq!(rig.sreg() & (Z | N | V | S), flags(false, b(res, 7), false, b(res, 7), res == 0, false));
            }
        }
    }
}

#[test]
fn lsr_asr_ror_swap_match_the_manual() {
    let mut rig = Rig::new();
    for d in 0..=255u8 {
        for c_in in 0..2u8 {
            // LSR
            rig.set_reg(7, d);
            rig.set_sreg(KEEP | H | c_in);
            assert_eq!(rig.exec(&[one_reg(0x9406, 7)]), 1);
            let res = d >> 1;
            let c = b(d, 0);
            assert_eq!(rig.reg(7), res);
            assert_eq!(rig.sreg(), KEEP | H | flags(false, c, c, false, res == 0, c)); // N = 0, V = N^C = C, S = N^V
            // ASR
            rig.set_reg(7, d);
            rig.set_sreg(KEEP | H | c_in);
            assert_eq!(rig.exec(&[one_reg(0x9405, 7)]), 1);
            let res = (d >> 1) | (d & 0x80);
            let (n, c) = (b(res, 7), b(d, 0));
            assert_eq!(rig.reg(7), res);
            assert_eq!(rig.sreg(), KEEP | H | flags(false, (n ^ c) ^ n, n ^ c, n, res == 0, c));
            // ROR
            rig.set_reg(7, d);
            rig.set_sreg(KEEP | H | c_in);
            assert_eq!(rig.exec(&[one_reg(0x9407, 7)]), 1);
            let res = (d >> 1) | (c_in << 7);
            let (n, c) = (b(res, 7), b(d, 0));
            assert_eq!(rig.reg(7), res);
            assert_eq!(rig.sreg(), KEEP | H | flags(false, (n ^ c) ^ n, n ^ c, n, res == 0, c));
            // SWAP exchanges the nibbles and leaves SREG alone.
            rig.set_reg(7, d);
            rig.set_sreg(0xff);
            assert_eq!(rig.exec(&[one_reg(0x9402, 7)]), 1);
            assert_eq!(rig.reg(7), d.rotate_left(4));
            assert_eq!(rig.sreg(), 0xff);
        }
    }
}

// ---- multiply -------------------------------------------------------------------

#[test]
fn multiply_family_matches_the_manual() {
    let mut rig = Rig::new();
    for x in (0..=255u8).step_by(3) {
        for y in (0..=255u8).step_by(5) {
            // MUL: unsigned x unsigned
            rig.set_reg(2, x);
            rig.set_reg(3, y);
            rig.set_sreg(KEEP | N | V | H);
            assert_eq!(rig.exec(&[two_reg(0x9c00, 2, 3)]), 2);
            let p = x as u16 * y as u16;
            assert_eq!(rig.pair(0), p);
            assert_eq!(rig.sreg() & (Z | C), flags(false, false, false, false, p == 0, p & 0x8000 != 0));
            assert_eq!(rig.sreg() & !(Z | C), KEEP | N | V | H, "MUL leaves the other flags alone");

            // MULS (registers 16..=31): signed x signed
            rig.set_reg(18, x);
            rig.set_reg(19, y);
            assert_eq!(rig.exec(&[0x0200 | (2 << 4) | 3]), 2);
            let p = (x as i8 as i16 * y as i8 as i16) as u16;
            assert_eq!(rig.pair(0), p);
            assert_eq!(rig.sreg() & (Z | C), flags(false, false, false, false, p == 0, p & 0x8000 != 0));

            // MULSU (registers 16..=23): signed x unsigned
            rig.set_reg(18, x);
            rig.set_reg(19, y);
            assert_eq!(rig.exec(&[0x0300 | (2 << 4) | 3]), 2);
            let p = (x as i8 as i32 * y as i32) as u16;
            assert_eq!(rig.pair(0), p);
            assert_eq!(rig.sreg() & (Z | C), flags(false, false, false, false, p == 0, p & 0x8000 != 0));

            // FMUL: unsigned, shifted left one; C is bit 15 of the product
            // before the shift.
            rig.set_reg(18, x);
            rig.set_reg(19, y);
            assert_eq!(rig.exec(&[0x0308 | (2 << 4) | 3]), 2);
            let raw = x as u32 * y as u32;
            let p = (raw << 1) as u16;
            assert_eq!(rig.pair(0), p);
            assert_eq!(rig.sreg() & (Z | C), flags(false, false, false, false, p == 0, raw & 0x8000 != 0));

            // FMULS: signed
            rig.set_reg(18, x);
            rig.set_reg(19, y);
            assert_eq!(rig.exec(&[0x0380 | (2 << 4) | 3]), 2);
            let raw = x as i8 as i32 * y as i8 as i32;
            let p = (raw << 1) as u16;
            assert_eq!(rig.pair(0), p);
            assert_eq!(rig.sreg() & (Z | C), flags(false, false, false, false, p == 0, raw & 0x8000 != 0));

            // FMULSU: signed x unsigned. Reference quirk: Z is set when the
            // product is NOT zero (the manual: when it is zero).
            rig.set_reg(18, x);
            rig.set_reg(19, y);
            assert_eq!(rig.exec(&[0x0388 | (2 << 4) | 3]), 2);
            let raw = x as i8 as i32 * y as i32;
            let p = (raw << 1) as u16;
            assert_eq!(rig.pair(0), p);
            assert_eq!(rig.sreg() & Z, if p != 0 { Z } else { 0 });
            assert_eq!(rig.sreg() & C, if raw & 0x8000 != 0 { C } else { 0 });
        }
    }
}

// ---- bit instructions -------------------------------------------------------------

#[test]
fn bset_bclr_bst_bld_cover_every_bit() {
    let mut rig = Rig::new();
    for s in 0..8u16 {
        rig.set_sreg(0);
        assert_eq!(rig.exec(&[0x9408 | (s << 4)]), 1);
        assert_eq!(rig.sreg(), 1 << s, "BSET {s}");
        rig.set_sreg(0xff);
        assert_eq!(rig.exec(&[0x9488 | (s << 4)]), 1);
        assert_eq!(rig.sreg(), !(1u8 << s), "BCLR {s}");
    }
    for bit in 0..8u16 {
        for val in [0x00u8, 0xff, 0xa5, 0x5a] {
            rig.set_reg(9, val);
            rig.set_sreg(0);
            assert_eq!(rig.exec(&[0xfa00 | (9 << 4) | bit]), 1); // BST
            assert_eq!(rig.sreg(), if (val >> bit) & 1 != 0 { T } else { 0 });
            rig.set_reg(9, val);
            rig.set_sreg(T);
            assert_eq!(rig.exec(&[0xf800 | (9 << 4) | bit]), 1); // BLD
            assert_eq!(rig.reg(9), val | (1 << bit));
            rig.set_sreg(0);
            rig.set_reg(9, val);
            rig.exec(&[0xf800 | (9 << 4) | bit]);
            assert_eq!(rig.reg(9), val & !(1 << bit));
        }
    }
}

#[test]
fn sbi_cbi_and_the_skip_family() {
    let mut rig = Rig::new();
    // SBI/CBI act on I/O addresses 0..31 (data space 0x20..0x3f).
    for a in 0..32u16 {
        for bit in 0..8u16 {
            rig.cpu.data[0x20 + a as usize] = 0;
            assert_eq!(rig.exec(&[0x9a00 | (a << 3) | bit]), 2, "SBI");
            assert_eq!(rig.cpu.data[0x20 + a as usize], 1 << bit);
            rig.cpu.data[0x20 + a as usize] = 0xff;
            let cycles = rig.exec(&[0x9800 | (a << 3) | bit]);
            assert_eq!(rig.cpu.data[0x20 + a as usize], !(1u8 << bit));
            // reference quirk: CBI takes 1 cycle; the manual says 2.
            assert_eq!(cycles, 1, "CBI");
        }
    }
    // SBIC / SBIS / SBRC / SBRS: 1 cycle, +1 skipping a one-word
    // instruction, +2 skipping a two-word one.
    let two_word = [0x9000u16 | (16 << 4), 0x0100]; // LDS r16, 0x0100
    let one_word = [0x0000u16]; // NOP
    for (name, enc, set_cond_true) in [
        ("SBIC", 0x9900u16 | (3 << 3) | 2, false),
        ("SBIS", 0x9b00u16 | (3 << 3) | 2, true),
    ] {
        for cond_true in [false, true] {
            rig.cpu.data[0x20 + 3] = if cond_true { 4 } else { 0 };
            for (next, size) in [(&one_word[..], 1u64), (&two_word[..], 2)] {
                rig.cpu.prog[1] = next[0];
                if next.len() > 1 {
                    rig.cpu.prog[2] = next[1];
                }
                let cycles = rig.exec(&[enc]);
                let skips = cond_true == set_cond_true;
                assert_eq!(cycles, if skips { 1 + size } else { 1 }, "{name}");
                assert_eq!(rig.cpu.pc, if skips { 1 + size as i32 } else { 1 });
            }
        }
    }
    for (name, base, skip_when_set) in [("SBRC", 0xfc00u16, false), ("SBRS", 0xfe00u16, true)] {
        for bit_set in [false, true] {
            rig.set_reg(10, if bit_set { 0x20 } else { 0 });
            for (next, size) in [(&one_word[..], 1u64), (&two_word[..], 2)] {
                rig.cpu.prog[1] = next[0];
                let cycles = rig.exec(&[base | (10 << 4) | 5]);
                let skips = bit_set == skip_when_set;
                assert_eq!(cycles, if skips { 1 + size } else { 1 }, "{name}");
            }
        }
    }
    // CPSE
    rig.set_reg(1, 9);
    rig.set_reg(2, 9);
    rig.cpu.prog[1] = 0x940c; // JMP (two words)
    assert_eq!(rig.exec(&[two_reg(0x1000, 1, 2)]), 3);
    assert_eq!(rig.cpu.pc, 3);
    rig.set_reg(2, 8);
    assert_eq!(rig.exec(&[two_reg(0x1000, 1, 2)]), 1);
    assert_eq!(rig.cpu.pc, 1);
    // every two-word opcode is recognised as such: LDS, STS, CALL, JMP
    for first in [0x9000u16, 0x9200, 0x940e, 0x940c, 0x95fe & 0, 0x9001] {
        rig.set_reg(1, 1);
        rig.set_reg(2, 1);
        rig.cpu.prog[1] = first;
        let cycles = rig.exec(&[two_reg(0x1000, 1, 2)]);
        let two = matches!(first, 0x9000 | 0x9200 | 0x940e | 0x940c);
        assert_eq!(cycles, if two { 3 } else { 2 }, "skip over {first:#06x}");
    }
}

// ---- branches, jumps, calls ------------------------------------------------------------

#[test]
fn brbs_brbc_cover_every_flag_and_offset() {
    let mut rig = Rig::new();
    for s in 0..8u16 {
        for k in [-64i32, -17, -1, 0, 1, 5, 63] {
            let kk = (k & 0x7f) as u16;
            for (name, base, taken_when_set) in [("BRBS", 0xf000u16, true), ("BRBC", 0xf400u16, false)] {
                for flag_set in [false, true] {
                    rig.set_sreg(if flag_set { 1 << s } else { 0 });
                    let at = 100;
                    let cycles = rig.exec_at(at, &[base | (kk << 3) | s]);
                    let taken = flag_set == taken_when_set;
                    assert_eq!(cycles, if taken { 2 } else { 1 }, "{name} s={s} k={k}");
                    assert_eq!(rig.cpu.pc, at + 1 + if taken { k } else { 0 });
                }
            }
        }
    }
}

#[test]
fn rjmp_jmp_ijmp_eijmp() {
    let mut rig = Rig::new();
    for k in [-2048i32, -100, -1, 0, 1, 7, 2047] {
        assert_eq!(rig.exec_at(5000, &[0xc000 | (k & 0xfff) as u16]), 2);
        assert_eq!(rig.cpu.pc, 5000 + 1 + k);
    }
    // JMP k: three cycles, PC = k (the reference parks k - 1 and increments).
    for k in [0u32, 1, 0x1234, 0x7fff] {
        let w1 = 0x940c | (((k >> 16) & 1) as u16) | ((((k >> 17) & 0x1f) as u16) << 4);
        assert_eq!(rig.exec(&[w1, (k & 0xffff) as u16]), 3);
        assert_eq!(rig.cpu.pc, k as i32);
    }
    rig.set_pair(30, 0x2345);
    assert_eq!(rig.exec(&[0x9409]), 2);
    assert_eq!(rig.cpu.pc, 0x2345);
}

#[test]
fn call_ret_reti_use_two_stack_bytes_on_a_16_bit_pc() {
    let mut rig = Rig::new();
    rig.cpu.set_sp(0x08ff);
    // CALL 0x1234 at word 0x100: pushes the address after the 2-word
    // instruction (0x102), low byte at SP.
    let cycles = rig.exec_at(0x100, &[0x940e, 0x1234]);
    assert_eq!(cycles, 4);
    assert_eq!(rig.cpu.pc, 0x1234);
    assert_eq!(rig.cpu.sp(), 0x08fd);
    assert_eq!(rig.cpu.data[0x08ff], 0x02);
    assert_eq!(rig.cpu.data[0x08fe], 0x01);
    // RET
    rig.cpu.prog[0x1234] = 0x9508;
    rig.cpu.pc = 0x1234;
    let before = rig.cpu.cycles;
    avr_instruction(&mut rig.cpu);
    assert_eq!(rig.cpu.cycles - before, 4);
    assert_eq!(rig.cpu.pc, 0x102);
    assert_eq!(rig.cpu.sp(), 0x08ff);
    // RCALL: 3 cycles, pushes PC+1, relative.
    rig.set_sreg(0);
    assert_eq!(rig.exec_at(0x200, &[0xd000 | 0x010]), 3);
    assert_eq!(rig.cpu.pc, 0x200 + 1 + 0x10);
    assert_eq!(rig.cpu.data[0x08ff], 0x01);
    assert_eq!(rig.cpu.data[0x08fe], 0x02);
    assert_eq!(rig.cpu.sp(), 0x08fd);
    // negative RCALL offset
    rig.cpu.set_sp(0x08ff);
    assert_eq!(rig.exec_at(0x200, &[0xd000 | 0xfff]), 3);
    assert_eq!(rig.cpu.pc, 0x200);
    // ICALL: 3 cycles, PC = Z.
    rig.cpu.set_sp(0x08ff);
    rig.set_pair(30, 0x0abc);
    assert_eq!(rig.exec_at(0x300, &[0x9509]), 3);
    assert_eq!(rig.cpu.pc, 0x0abc);
    assert_eq!(rig.cpu.data[0x08ff], 0x01);
    assert_eq!(rig.cpu.data[0x08fe], 0x03);
    // RETI: 4 cycles and sets I.
    rig.set_sreg(0);
    rig.cpu.pc = 0x0abc;
    rig.cpu.prog[0x0abc] = 0x9518;
    let before = rig.cpu.cycles;
    avr_instruction(&mut rig.cpu);
    assert_eq!(rig.cpu.cycles - before, 4);
    assert_eq!(rig.cpu.pc, 0x301);
    assert_eq!(rig.sreg(), I);
}

#[test]
fn a_22_bit_pc_pushes_three_bytes_and_reaches_eijmp_eicall() {
    // More than 128 KiB of flash selects the 22-bit program counter.
    let mut cpu = Cpu::new(vec![0; 0x20001], 8192);
    assert!(cpu.pc22_bits);
    cpu.set_sp(0x08ff);
    cpu.prog[0x100] = 0x940e | 1; // CALL, k bit 16
    cpu.prog[0x101] = 0x0010;
    cpu.pc = 0x100;
    let c0 = cpu.cycles;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.cycles - c0, 5);
    assert_eq!(cpu.pc, 0x10010);
    assert_eq!(cpu.sp(), 0x08fc);
    assert_eq!(&cpu.data[0x08fd..=0x08ff], &[0x00, 0x01, 0x02]);
    cpu.prog[0x10010] = 0x9508; // RET
    let c0 = cpu.cycles;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.cycles - c0, 5);
    assert_eq!(cpu.pc, 0x102);
    // EIJMP: EIND:Z.
    cpu.data[ADDR_EIND] = 1;
    cpu.set16(30, 0x0100);
    cpu.prog[0x200] = 0x9419;
    cpu.pc = 0x200;
    let c0 = cpu.cycles;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.cycles - c0, 2);
    assert_eq!(cpu.pc, 0x10100);
    // EICALL
    cpu.set_sp(0x08ff);
    cpu.prog[0x300] = 0x9519;
    cpu.pc = 0x300;
    let c0 = cpu.cycles;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.cycles - c0, 4);
    assert_eq!(cpu.pc, 0x10100);
    assert_eq!(cpu.sp(), 0x08fc);
}

// ---- data movement -----------------------------------------------------------------------

#[test]
fn ldi_mov_movw() {
    let mut rig = Rig::new();
    for k in 0..=255u8 {
        for d in [16usize, 23, 31] {
            assert_eq!(rig.exec(&[reg_imm(0xe000, d, k)]), 1);
            assert_eq!(rig.reg(d), k);
        }
    }
    rig.set_reg(30, 0x5a);
    rig.set_sreg(0xaa);
    assert_eq!(rig.exec(&[two_reg(0x2c00, 4, 30)]), 1);
    assert_eq!(rig.reg(4), 0x5a);
    assert_eq!(rig.sreg(), 0xaa);
    rig.set_pair(2, 0xbeef);
    assert_eq!(rig.exec(&[0x0100 | (12 << 4) | 1]), 1); // MOVW r25:r24... d=24 pair 12, r=2 pair 1 -> r24 <- r2
    assert_eq!(rig.pair(24), 0xbeef);
}

#[test]
fn lds_sts_and_the_pointer_forms() {
    let mut rig = Rig::new();
    rig.cpu.data[0x0123] = 0x77;
    assert_eq!(rig.exec(&[0x9000 | (5 << 4), 0x0123]), 2);
    assert_eq!(rig.reg(5), 0x77);
    assert_eq!(rig.cpu.pc, 2);
    rig.set_reg(6, 0x99);
    assert_eq!(rig.exec(&[0x9200 | (6 << 4), 0x0200]), 2);
    assert_eq!(rig.cpu.data[0x0200], 0x99);
    assert_eq!(rig.cpu.pc, 2);

    // (load opcode low nibble, store opcode low nibble, pointer base register,
    // pre-decrement, post-increment): X 1100/1101/1110, Y 1000/1001/1010,
    // Z 0000/0001/0010.
    let ptrs = [(26usize, 0x900c_u16, 0x920c_u16, 0x900d_u16, 0x920d_u16, 0x900e_u16, 0x920e_u16),
                (28, 0x8008, 0x8208, 0x9009, 0x9209, 0x900a, 0x920a),
                (30, 0x8000, 0x8200, 0x9001, 0x9201, 0x9002, 0x9202)];
    for (p, ld, st, ldp, stp, ldm, stm) in ptrs {
        rig.set_pair(p, 0x0400);
        rig.cpu.data[0x0400] = 0x11;
        assert_eq!(rig.exec(&[ld | (3 << 4)]), 2);
        assert_eq!(rig.reg(3), 0x11);
        assert_eq!(rig.pair(p), 0x0400);
        // post-increment
        rig.cpu.data[0x0400] = 0x22;
        assert_eq!(rig.exec(&[ldp | (3 << 4)]), 2);
        assert_eq!(rig.reg(3), 0x22);
        assert_eq!(rig.pair(p), 0x0401);
        // pre-decrement
        assert_eq!(rig.exec(&[ldm | (3 << 4)]), 2);
        assert_eq!(rig.reg(3), 0x22);
        assert_eq!(rig.pair(p), 0x0400);
        // stores
        rig.set_reg(4, 0xc3);
        assert_eq!(rig.exec(&[st | (4 << 4)]), 2);
        assert_eq!(rig.cpu.data[0x0400], 0xc3);
        rig.set_reg(4, 0xd4);
        assert_eq!(rig.exec(&[stp | (4 << 4)]), 2);
        assert_eq!(rig.cpu.data[0x0400], 0xd4);
        assert_eq!(rig.pair(p), 0x0401);
        rig.set_reg(4, 0xe5);
        assert_eq!(rig.exec(&[stm | (4 << 4)]), 2);
        assert_eq!(rig.cpu.data[0x0400], 0xe5);
        assert_eq!(rig.pair(p), 0x0400);
    }
    // LDD/STD with displacement q = 0..63 via Y (base 0x8008/0x8208) and Z
    // (0x8000/0x8200); q is spread over bits 13, 11-10, 2-0.
    for q in 1..64u16 {
        let qbits = (q & 7) | ((q & 0x18) << 7) | ((q & 0x20) << 8);
        for (p, ld, st) in [(28usize, 0x8008u16, 0x8208u16), (30, 0x8000, 0x8200)] {
            rig.set_pair(p, 0x0500);
            rig.cpu.data[0x0500 + q as usize] = q as u8 ^ 0x5a;
            assert_eq!(rig.exec(&[ld | qbits | (7 << 4)]), 2);
            assert_eq!(rig.reg(7), q as u8 ^ 0x5a);
            assert_eq!(rig.pair(p), 0x0500, "LDD leaves the pointer alone");
            rig.set_reg(8, q as u8);
            assert_eq!(rig.exec(&[st | qbits | (8 << 4)]), 2);
            assert_eq!(rig.cpu.data[0x0500 + q as usize], q as u8);
        }
    }
    // The pointer wraps at 16 bits on post-increment.
    rig.set_pair(26, 0xffff);
    rig.exec(&[0x900d | (3 << 4)]);
    assert_eq!(rig.pair(26), 0);
}

#[test]
fn push_pop_in_out_and_the_core_registers() {
    let mut rig = Rig::new();
    rig.cpu.set_sp(0x08ff);
    rig.set_reg(5, 0xab);
    assert_eq!(rig.exec(&[0x920f | (5 << 4)]), 2);
    assert_eq!(rig.cpu.data[0x08ff], 0xab);
    assert_eq!(rig.cpu.sp(), 0x08fe);
    assert_eq!(rig.exec(&[0x900f | (6 << 4)]), 2);
    assert_eq!(rig.reg(6), 0xab);
    assert_eq!(rig.cpu.sp(), 0x08ff);
    // OUT/IN address bits are split: A[5:4] at bits 10-9, A[3:0] at 3-0.
    for a in 0..64u16 {
        let enc = ((a & 0x30) << 5) | (a & 0xf);
        rig.set_reg(9, a as u8 ^ 0x3c);
        rig.cpu.data[0x20 + a as usize] = 0;
        let sp = rig.cpu.sp();
        let sreg = rig.sreg();
        assert_eq!(rig.exec(&[0xb800 | enc | (9 << 4)]), 1);
        if a == 0x3d || a == 0x3e || a == 0x3f {
            // SPL, SPH, SREG are the core's own registers: the write lands.
            assert_eq!(rig.cpu.data[0x20 + a as usize], a as u8 ^ 0x3c);
            rig.cpu.set_sp(sp);
            rig.set_sreg(sreg);
        } else {
            assert_eq!(rig.cpu.data[0x20 + a as usize], a as u8 ^ 0x3c);
        }
        rig.cpu.data[0x20 + a as usize] = 0x6d;
        assert_eq!(rig.exec(&[0xb000 | enc | (10 << 4)]), 1);
        assert_eq!(rig.reg(10), 0x6d);
    }
    // SREG and SP through OUT: SREG is I/O address 0x3f, SPL/SPH 0x3d/0x3e.
    rig.set_reg(11, 0x81);
    rig.exec(&[0xb800 | ((0x3f & 0x30) << 5) | (0x3f & 0xf) | (11 << 4)]);
    assert_eq!(rig.sreg(), 0x81);
    rig.set_reg(11, 0x34);
    rig.set_reg(12, 0x12);
    rig.exec(&[0xb800 | ((0x3d & 0x30) << 5) | (0x3d & 0xf) | (11 << 4)]);
    rig.exec(&[0xb800 | ((0x3e & 0x30) << 5) | (0x3e & 0xf) | (12 << 4)]);
    assert_eq!(rig.cpu.sp(), 0x1234);
}

#[test]
fn lpm_and_elpm_read_program_bytes() {
    let mut rig = Rig::new();
    // Program memory is little-endian words: byte 2n is the low byte.
    rig.cpu.prog[0x100] = 0xbeef;
    rig.cpu.prog[0x101] = 0x1234;
    rig.set_pair(30, 0x200); // byte address of word 0x100
    assert_eq!(rig.exec(&[0x95c8]), 3); // LPM -> r0
    assert_eq!(rig.reg(0), 0xef);
    assert_eq!(rig.exec(&[0x9004 | (4 << 4)]), 3); // LPM r4, Z
    assert_eq!(rig.reg(4), 0xef);
    assert_eq!(rig.exec(&[0x9005 | (4 << 4)]), 3); // LPM r4, Z+
    assert_eq!(rig.reg(4), 0xef);
    assert_eq!(rig.pair(30), 0x201);
    assert_eq!(rig.exec(&[0x9005 | (4 << 4)]), 3);
    assert_eq!(rig.reg(4), 0xbe);
    assert_eq!(rig.pair(30), 0x202);
    // ELPM reads RAMPZ:Z; with the 64 KiB flash image RAMPZ must stay 0.
    rig.set_pair(30, 0x202);
    rig.cpu.data[ADDR_RAMPZ] = 0;
    assert_eq!(rig.exec(&[0x95d8]), 3);
    assert_eq!(rig.reg(0), 0x34);
    assert_eq!(rig.exec(&[0x9006 | (5 << 4)]), 3);
    assert_eq!(rig.reg(5), 0x34);
    rig.set_pair(30, 0xffff);
    assert_eq!(rig.exec(&[0x9007 | (5 << 4)]), 3);
    assert_eq!(rig.pair(30), 0);
}

#[test]
fn xch_las_lac_lat_swap_with_memory() {
    let mut rig = Rig::new();
    rig.set_pair(30, 0x0300);
    rig.cpu.data[0x0300] = 0b1100_1010;
    rig.set_reg(2, 0b0101_0101);
    rig.exec(&[0x9204 | (2 << 4)]); // XCH
    assert_eq!(rig.cpu.data[0x0300], 0b0101_0101);
    assert_eq!(rig.reg(2), 0b1100_1010);
    rig.cpu.data[0x0300] = 0b1100_1010;
    rig.set_reg(2, 0b0000_1111);
    rig.exec(&[0x9205 | (2 << 4)]); // LAS: mem <- mem | Rd, Rd <- old mem
    assert_eq!(rig.cpu.data[0x0300], 0b1100_1111);
    assert_eq!(rig.reg(2), 0b1100_1010);
    rig.cpu.data[0x0300] = 0b1100_1010;
    rig.set_reg(2, 0b0000_1111);
    rig.exec(&[0x9206 | (2 << 4)]); // LAC: mem <- mem & !Rd
    assert_eq!(rig.cpu.data[0x0300], 0b1100_0000);
    assert_eq!(rig.reg(2), 0b1100_1010);
    rig.cpu.data[0x0300] = 0b1100_1010;
    rig.set_reg(2, 0b0000_1111);
    rig.exec(&[0x9207 | (2 << 4)]); // LAT: mem <- mem ^ Rd
    assert_eq!(rig.cpu.data[0x0300], 0b1100_0101);
    assert_eq!(rig.reg(2), 0b1100_1010);
}

#[test]
fn nop_sleep_wdr_spm_and_unassigned_opcodes() {
    let mut rig = Rig::new();
    rig.set_sreg(0x55);
    for op in [0x0000u16, 0x9588, 0x95e8, 0x95f8] {
        // NOP, SLEEP, SPM, SPM Z+: one cycle, nothing else (the reference
        // implements SLEEP and SPM as no-ops).
        assert_eq!(rig.exec(&[op]), 1);
        assert_eq!(rig.cpu.pc, 1);
        assert_eq!(rig.sreg(), 0x55);
    }
    assert_eq!(rig.cpu.watchdog_resets, 0);
    assert_eq!(rig.exec(&[0x95a8]), 1);
    assert_eq!(rig.cpu.watchdog_resets, 1);
    // 0x0001 matches no pattern: the reference advances PC and the cycle count.
    assert_eq!(decode(0x0001), Op::Unknown);
    assert_eq!(rig.exec(&[0x0001]), 1);
    assert_eq!(rig.cpu.pc, 1);
}

#[test]
fn pc_wraps_at_the_end_of_flash() {
    let mut rig = Rig::new();
    rig.exec_at(0x7fff, &[0x0000]);
    assert_eq!(rig.cpu.pc, 0);
    // A relative jump past the end lands modulo the flash size.
    rig.exec_at(0x7ffe, &[0xc000 | 5]);
    assert_eq!(rig.cpu.pc, (0x7ffe + 1 + 5) % 0x8000);
}

// ---- decode ------------------------------------------------------------------------------------

/// The manual's opcode encodings, as bit strings; `0`/`1` are fixed bits,
/// letters are operand fields. Independent of `decode`'s masks.
const MANUAL: &[(&str, &str, Op)] = &[
    ("ADC", "000111rdddddrrrr", Op::Adc), ("ADD", "000011rdddddrrrr", Op::Add),
    ("ADIW", "10010110KKddKKKK", Op::Adiw), ("AND", "001000rdddddrrrr", Op::And),
    ("ANDI", "0111KKKKddddKKKK", Op::Andi), ("ASR", "1001010ddddd0101", Op::Asr),
    ("BCLR", "100101001sss1000", Op::Bclr), ("BLD", "1111100ddddd0bbb", Op::Bld),
    ("BRBC", "111101kkkkkkksss", Op::Brbc), ("BRBS", "111100kkkkkkksss", Op::Brbs),
    ("BSET", "100101000sss1000", Op::Bset), ("BST", "1111101ddddd0bbb", Op::Bst),
    ("CALL", "1001010kkkkk111k", Op::Call), ("CBI", "10011000AAAAAbbb", Op::Cbi),
    ("COM", "1001010ddddd0000", Op::Com), ("CP", "000101rdddddrrrr", Op::Cp),
    ("CPC", "000001rdddddrrrr", Op::Cpc), ("CPI", "0011KKKKddddKKKK", Op::Cpi),
    ("CPSE", "000100rdddddrrrr", Op::Cpse), ("DEC", "1001010ddddd1010", Op::Dec),
    ("EICALL", "1001010100011001", Op::Eicall), ("EIJMP", "1001010000011001", Op::Eijmp),
    ("ELPM", "1001010111011000", Op::Elpm), ("ELPM Z", "1001000ddddd0110", Op::ElpmReg),
    ("ELPM Z+", "1001000ddddd0111", Op::ElpmInc), ("EOR", "001001rdddddrrrr", Op::Eor),
    ("FMUL", "000000110ddd1rrr", Op::Fmul), ("FMULS", "000000111ddd0rrr", Op::Fmuls),
    ("FMULSU", "000000111ddd1rrr", Op::Fmulsu), ("ICALL", "1001010100001001", Op::Icall),
    ("IJMP", "1001010000001001", Op::Ijmp), ("IN", "10110AAdddddAAAA", Op::In),
    ("INC", "1001010ddddd0011", Op::Inc), ("JMP", "1001010kkkkk110k", Op::Jmp),
    ("LAC", "1001001rrrrr0110", Op::Lac), ("LAS", "1001001rrrrr0101", Op::Las),
    ("LAT", "1001001rrrrr0111", Op::Lat), ("LD X", "1001000ddddd1100", Op::Ldx),
    ("LD X+", "1001000ddddd1101", Op::LdxInc), ("LD -X", "1001000ddddd1110", Op::LdxDec),
    ("LD Y", "1000000ddddd1000", Op::Ldy), ("LD Y+", "1001000ddddd1001", Op::LdyInc),
    ("LD -Y", "1001000ddddd1010", Op::LdyDec), ("LDD Y", "10q0qq0ddddd1qqq", Op::Lddy),
    ("LD Z", "1000000ddddd0000", Op::Ldz), ("LD Z+", "1001000ddddd0001", Op::LdzInc),
    ("LD -Z", "1001000ddddd0010", Op::LdzDec), ("LDD Z", "10q0qq0ddddd0qqq", Op::Lddz),
    ("LDI", "1110KKKKddddKKKK", Op::Ldi), ("LDS", "1001000ddddd0000", Op::Lds),
    ("LPM", "1001010111001000", Op::Lpm), ("LPM Z", "1001000ddddd0100", Op::LpmReg),
    ("LPM Z+", "1001000ddddd0101", Op::LpmInc), ("LSR", "1001010ddddd0110", Op::Lsr),
    ("MOV", "001011rdddddrrrr", Op::Mov), ("MOVW", "00000001ddddrrrr", Op::Movw),
    ("MUL", "100111rdddddrrrr", Op::Mul), ("MULS", "00000010ddddrrrr", Op::Muls),
    ("MULSU", "000000110ddd0rrr", Op::Mulsu), ("NEG", "1001010ddddd0001", Op::Neg),
    ("NOP", "0000000000000000", Op::Nop), ("OR", "001010rdddddrrrr", Op::Or),
    ("ORI", "0110KKKKddddKKKK", Op::Sbr), ("OUT", "10111AArrrrrAAAA", Op::Out),
    ("POP", "1001000ddddd1111", Op::Pop), ("PUSH", "1001001ddddd1111", Op::Push),
    ("RCALL", "1101kkkkkkkkkkkk", Op::Rcall), ("RET", "1001010100001000", Op::Ret),
    ("RETI", "1001010100011000", Op::Reti), ("RJMP", "1100kkkkkkkkkkkk", Op::Rjmp),
    ("ROR", "1001010ddddd0111", Op::Ror), ("SBC", "000010rdddddrrrr", Op::Sbc),
    ("SBCI", "0100KKKKddddKKKK", Op::Sbci), ("SBI", "10011010AAAAAbbb", Op::Sbi),
    ("SBIC", "10011001AAAAAbbb", Op::Sbic), ("SBIS", "10011011AAAAAbbb", Op::Sbis),
    ("SBIW", "10010111KKddKKKK", Op::Sbiw), ("SBRC", "1111110rrrrr0bbb", Op::Sbrc),
    ("SBRS", "1111111rrrrr0bbb", Op::Sbrs), ("SLEEP", "1001010110001000", Op::Sleep),
    ("SPM", "1001010111101000", Op::Spm), ("SPM Z+", "1001010111111000", Op::SpmInc),
    ("ST X", "1001001rrrrr1100", Op::Stx), ("ST X+", "1001001rrrrr1101", Op::StxInc),
    ("ST -X", "1001001rrrrr1110", Op::StxDec), ("ST Y", "1000001rrrrr1000", Op::Sty),
    ("ST Y+", "1001001rrrrr1001", Op::StyInc), ("ST -Y", "1001001rrrrr1010", Op::StyDec),
    ("STD Y", "10q0qq1rrrrr1qqq", Op::Stdy), ("ST Z", "1000001rrrrr0000", Op::Stz),
    ("ST Z+", "1001001rrrrr0001", Op::StzInc), ("ST -Z", "1001001rrrrr0010", Op::StzDec),
    ("STD Z", "10q0qq1rrrrr0qqq", Op::Stdz), ("STS", "1001001ddddd0000", Op::Sts),
    ("SUB", "000110rdddddrrrr", Op::Sub), ("SUBI", "0101KKKKddddKKKK", Op::Subi),
    ("SWAP", "1001010ddddd0010", Op::Swap), ("WDR", "1001010110101000", Op::Wdr),
    ("XCH", "1001001rrrrr0100", Op::Xch),
];

fn pattern_matches(pat: &str, op: u16) -> bool {
    pat.bytes().enumerate().all(|(i, ch)| {
        let bit = (op >> (15 - i)) & 1;
        match ch {
            b'0' => bit == 0,
            b'1' => bit == 1,
            _ => true,
        }
    })
}

#[test]
fn decode_agrees_with_the_manual_on_all_65536_opcode_words() {
    for op in 0..=0xffffu16 {
        let candidates: Vec<(&str, Op)> =
            MANUAL.iter().filter(|(_, p, _)| pattern_matches(p, op)).map(|(n, _, o)| (*n, *o)).collect();
        let got = decode(op);
        if candidates.is_empty() {
            assert_eq!(got, Op::Unknown, "opcode {op:#06x} is not in the manual");
        } else {
            assert!(
                candidates.iter().any(|(_, o)| *o == got),
                "opcode {op:#06x}: manual says {:?}, decode says {got:?}",
                candidates.iter().map(|(n, _)| *n).collect::<Vec<_>>()
            );
        }
    }
}

// ---- interrupts and clock events --------------------------------------------------------------------

fn irq(address: u8) -> Interrupt {
    Interrupt { address, flag_register: 0x35, flag_mask: 1 << (address % 8), enable_register: 0x6e, enable_mask: 1 << (address % 8), constant: false, inverse_flag: false }
}

#[test]
fn an_enabled_interrupt_is_taken_when_i_is_set() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu.set_sp(0x08ff);
    cpu.pc = 0x0123;
    cpu.cycles = 1000;
    cpu.data[0x6e] = 0xff; // enabled
    let i = irq(0x20);
    cpu.set_interrupt_flag(i);
    assert_eq!(cpu.data[0x35] & 1, 1, "the flag register bit is set");
    assert_eq!(cpu.next_interrupt, 0x20);
    // I clear: nothing happens.
    cpu.data[ADDR_SREG] = 0;
    cpu.tick();
    assert_eq!(cpu.pc, 0x0123);
    cpu.data[ADDR_SREG] = I | C;
    cpu.tick();
    assert_eq!(cpu.pc, 0x20);
    assert_eq!(cpu.cycles, 1002, "two cycles");
    assert_eq!(cpu.sreg(), C, "I cleared, the rest kept");
    assert_eq!(cpu.sp(), 0x08fd);
    assert_eq!(cpu.data[0x08ff], 0x23);
    assert_eq!(cpu.data[0x08fe], 0x01);
    assert_eq!(cpu.data[0x35] & 1, 0, "a non-constant flag is cleared when taken");
    assert_eq!(cpu.next_interrupt, -1);
}

#[test]
fn a_disabled_interrupt_stays_flagged_until_enabled() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    let i = irq(0x10);
    cpu.set_interrupt_flag(i);
    assert_eq!(cpu.next_interrupt, -1, "enable bit is clear: not queued");
    assert_eq!(cpu.data[0x35] & (1 << 0), 1 << 0);
    cpu.data[0x6e] |= i.enable_mask;
    cpu.update_interrupt_enable(i, cpu.data[0x6e]);
    assert_eq!(cpu.next_interrupt, 0x10);
    cpu.data[0x6e] &= !i.enable_mask;
    cpu.update_interrupt_enable(i, cpu.data[0x6e]);
    assert_eq!(cpu.next_interrupt, -1);
    assert_eq!(cpu.data[0x35] & 1, 1, "disabling leaves the flag");
}

#[test]
fn the_lowest_vector_wins_and_the_next_follows() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu.set_sp(0x08ff);
    cpu.data[0x6e] = 0xff;
    cpu.set_interrupt_flag(irq(0x24));
    cpu.set_interrupt_flag(irq(0x12));
    cpu.set_interrupt_flag(irq(0x30));
    assert_eq!(cpu.next_interrupt, 0x12);
    for expect in [0x12, 0x24, 0x30] {
        cpu.data[ADDR_SREG] = I;
        cpu.tick();
        assert_eq!(cpu.pc, expect);
    }
    assert_eq!(cpu.next_interrupt, -1);
}

#[test]
fn constant_and_inverse_flag_interrupts() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu.set_sp(0x08ff);
    cpu.data[0x6e] = 0xff;
    let mut level = irq(0x14);
    level.constant = true;
    cpu.set_interrupt_flag(level);
    cpu.data[ADDR_SREG] = I;
    cpu.tick();
    assert_eq!(cpu.pc, 0x14);
    assert_eq!(cpu.next_interrupt, 0x14, "a constant interrupt stays pending");
    assert_eq!(cpu.data[0x35] & (1 << 4), 1 << 4);
    cpu.clear_interrupt(level, true);
    assert_eq!(cpu.next_interrupt, -1);
    // inverse flag: active when the bit is CLEAR
    let mut inv = irq(0x16);
    inv.inverse_flag = true;
    cpu.data[0x35] = 0xff;
    cpu.set_interrupt_flag(inv);
    assert_eq!(cpu.data[0x35] & inv.flag_mask, 0);
    assert_eq!(cpu.next_interrupt, 0x16);
    // clear by writing the flag bit one
    let other = irq(0x18);
    cpu.set_interrupt_flag(other);
    cpu.clear_interrupt_by_flag(other, other.flag_mask);
    assert_eq!(cpu.data[0x35] & other.flag_mask, 0);
    assert_eq!(cpu.pending_interrupts[0x18], None);
}

#[test]
fn reti_returns_and_reenables_interrupts() {
    let mut rig = Rig::new();
    rig.cpu.set_sp(0x08ff);
    rig.cpu.pc = 0x0050;
    rig.cpu.data[0x6e] = 1;
    let mut i = irq(0x20);
    i.flag_mask = 1;
    i.enable_mask = 1;
    rig.cpu.set_interrupt_flag(i);
    rig.cpu.data[ADDR_SREG] = I;
    rig.cpu.tick();
    assert_eq!(rig.cpu.pc, 0x20);
    assert_eq!(rig.sreg() & I, 0);
    rig.cpu.prog[0x20] = 0x9518; // RETI
    avr_instruction(&mut rig.cpu);
    assert_eq!(rig.cpu.pc, 0x50);
    assert_eq!(rig.sreg() & I, I);
    assert_eq!(rig.cpu.sp(), 0x08ff);
}

#[test]
fn clock_events_fire_in_order_one_per_tick() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu.add_clock_event(1, 10);
    cpu.add_clock_event(2, 5);
    cpu.add_clock_event(3, 5); // equal time: goes before the older one
    cpu.add_clock_event(4, 0); // max(1, 0): one cycle from now
    let order: Vec<(u64, u32)> = cpu.clock_events.iter().map(|e| (e.cycles, e.id)).collect();
    assert_eq!(order, vec![(1, 4), (5, 3), (5, 2), (10, 1)]);
    let mut fired = Vec::new();
    cpu.cycles = 20;
    for _ in 0..6 {
        cpu.tick_with(|_, id| fired.push(id));
    }
    assert_eq!(fired, vec![4, 3, 2, 1], "one event per tick, soonest first");
    // a callback may add an event
    cpu.add_clock_event(7, 1);
    cpu.cycles = 30;
    let mut fired = Vec::new();
    cpu.tick_with(|c, id| {
        fired.push(id);
        c.add_clock_event(8, 5);
    });
    assert_eq!(fired, vec![7]);
    assert_eq!(cpu.clock_events.len(), 1);
    assert_eq!(cpu.clock_events[0].cycles, 35);
    assert!(cpu.update_clock_event(8, 100));
    assert_eq!(cpu.clock_events[0].cycles, 130);
    assert!(cpu.clear_clock_event(8));
    assert!(!cpu.clear_clock_event(8));
}

#[test]
fn reset_restores_sp_and_pc_and_drops_pending_work() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    assert_eq!(cpu.sp(), 8192 + 0x100 - 1);
    cpu.pc = 77;
    cpu.cycles = 5;
    cpu.data[0x6e] = 0xff;
    cpu.set_interrupt_flag(irq(0x20));
    cpu.add_clock_event(1, 3);
    cpu.reset();
    assert_eq!(cpu.pc, 0);
    assert_eq!(cpu.cycles, 5, "the cycle counter is not reset");
    assert_eq!(cpu.next_interrupt, -1);
    assert!(cpu.clock_events.is_empty());
    assert_eq!(cpu.sp(), 8192 + 0x100 - 1);
}

#[test]
fn io_trap_reports_the_first_peripheral_access_only() {
    let mut rig = Rig::new();
    rig.cpu.io_trap = true;
    // SREG/SP/RAM accesses and registers are not peripherals.
    rig.exec(&[0xb800 | (0x3f & 0xf) | ((0x3f & 0x30) << 5)]);
    rig.exec(&[0x9000 | (5 << 4), 0x0200]);
    rig.exec(&[0x9000 | (5 << 4), 0x005d]);
    assert_eq!(rig.cpu.io_hit, None);
    // OUT 0x04 (PORTB-ish) is data address 0x24.
    rig.exec(&[0xb800 | 4 | (5 << 4)]);
    assert_eq!(rig.cpu.io_hit.map(|h| (h.address, h.write)), Some((0x24, true)));
    // a later access does not replace it
    rig.exec(&[0xb000 | 5 | (6 << 4)]);
    assert_eq!(rig.cpu.io_hit.map(|h| h.address), Some(0x24));
    // an extended I/O register through LDS
    rig.cpu.io_hit = None;
    rig.exec(&[0x9000 | (5 << 4), 0x006e]);
    assert_eq!(rig.cpu.io_hit.map(|h| (h.address, h.write)), Some((0x6e, false)));
}

#[test]
fn rig_sanity_a_tiny_program_runs_to_the_expected_state() {
    // ldi r16, 5; ldi r17, 3; add r16, r17; mov r2, r16; rjmp .-1 (loop on the rjmp)
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    let prog = [0xe005u16, 0xe013, 0x0f01, 0x2e20, 0xcfff];
    cpu.prog[..prog.len()].copy_from_slice(&prog);
    for _ in 0..4 {
        avr_instruction(&mut cpu);
        cpu.tick();
    }
    assert_eq!(cpu.data[2], 8);
    assert_eq!(cpu.cycles, 4);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 4, "rjmp to itself");
    assert_eq!(cpu.cycles, 6);
}

// ---- the named departures from the manual ------------------------------------------
// Each is one test, so the list of places the port keeps avr8js's behaviour
// instead of the datasheet's is the list of `reference_quirk_*` tests. (Their
// full-range versions sit in the operand sweeps above.)

#[test]
fn reference_quirk_neg_never_sets_v() {
    // The manual: V set when the result is 0x80, i.e. NEG of 0x80.
    let mut rig = Rig::new();
    rig.set_reg(5, 0x80);
    rig.set_sreg(0);
    rig.exec(&[one_reg(0x9401, 5)]);
    assert_eq!(rig.reg(5), 0x80);
    assert_eq!(rig.sreg() & V, 0);
}

#[test]
fn reference_quirk_fmulsu_sets_z_when_the_product_is_not_zero() {
    let mut rig = Rig::new();
    rig.set_reg(18, 2);
    rig.set_reg(19, 3);
    rig.exec(&[0x0388 | (2 << 4) | 3]); // FMULSU r18, r19: 2*3 << 1 = 12
    assert_eq!(rig.pair(0), 12);
    assert_eq!(rig.sreg() & Z, Z, "the manual: Z clear for a nonzero product");
    rig.set_reg(18, 0);
    rig.exec(&[0x0388 | (2 << 4) | 3]);
    assert_eq!(rig.sreg() & Z, 0, "the manual: Z set for a zero product");
}

#[test]
fn reference_quirk_cbi_takes_one_cycle() {
    let mut rig = Rig::new();
    assert_eq!(rig.exec(&[0x9800 | (1 << 3)]), 1, "the manual: 2");
}

#[test]
fn reference_quirk_sbc_cpc_lose_z_when_the_borrow_wraps_to_zero() {
    // 0 - 0xff - 1 = -256: a zero byte, but the reference tests the unwrapped
    // difference, so Z (which the manual keeps when it was set) is cleared.
    let mut rig = Rig::new();
    for op in [two_reg(0x0800, 16, 17), two_reg(0x0400, 16, 17), reg_imm(0x4000, 16, 0xff)] {
        rig.set_reg(16, 0);
        rig.set_reg(17, 0xff);
        rig.set_sreg(Z | C);
        rig.exec(&[op]);
        assert_eq!(rig.sreg() & Z, 0, "op {op:#06x}");
    }
}

#[test]
fn reference_quirk_sbiw_writes_h() {
    // The manual leaves H alone; the reference clears it and writes bit 0 of
    // its carry chain (here 0 - 0 = 0: chain bit is 0, so a set H is cleared).
    let mut rig = Rig::new();
    rig.set_pair(24, 0);
    rig.set_sreg(H);
    rig.exec(&[0x9700]); // SBIW r25:r24, 0
    assert_eq!(rig.sreg() & H, 0);
}

#[test]
fn reference_quirk_interrupt_costs_two_cycles_and_follows_sei_at_once() {
    // The manual: four cycles of response, and the instruction after SEI runs
    // before any interrupt is taken. The reference takes it on the next tick.
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu.set_sp(0x08ff);
    cpu.data[0x6e] = 1;
    cpu.set_interrupt_flag(irq(0x20));
    cpu.prog[0] = 0x9478; // SEI
    cpu.pc = 0;
    avr_instruction(&mut cpu);
    let c = cpu.cycles;
    cpu.tick();
    assert_eq!(cpu.pc, 0x20, "taken right after SEI");
    assert_eq!(cpu.cycles - c, 2, "two cycles");
}

#[test]
fn reference_quirk_out_of_range_data_reads_zero_and_writes_vanish() {
    // JavaScript reads `undefined` past the end of the data array (which then
    // stores as 0 or turns a program counter into NaN); the port reads 0.
    let mut rig = Rig::new();
    let top = rig.cpu.data.len() as u16; // first address past SRAM
    rig.set_pair(26, top);
    rig.set_reg(3, 0x55);
    rig.exec(&[0x900c | (3 << 4)]); // LD r3, X
    assert_eq!(rig.reg(3), 0);
    rig.exec(&[0x920c | (3 << 4)]); // ST X, r3: nothing to write to
    assert_eq!(rig.cpu.data.len() as u16, top);
}

#[test]
fn reference_quirk_sleep_spm_and_break_do_nothing() {
    // BREAK (0x9598) is not decoded at all; SLEEP and SPM are no-ops.
    assert_eq!(decode(0x9598), Op::Unknown);
    let mut rig = Rig::new();
    assert_eq!(rig.exec(&[0x9598]), 1);
    assert_eq!(rig.cpu.pc, 1);
}
